/*******************************************************************
    Copyright (C) 2016 Triad Semiconductor
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
    3. Neither the name of the the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.
    4. Use of this source code or binary forms in commercial products shall require explicit
       written consent of Triad Semiconductor
    5. Silicon ASIC implementation of algorithims in this source is strictly prohibited, FPGA versions
       shall be permitted provided the source is open and attribution is made to this original work.

    THIS SOFTWARE IS PROVIDED BY THE THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
    OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.

    Originally written by Luke Beno
    Please post support questions to @lgbeno on Twitter.

*******************************************************************/

#include "ts3633_tracking.h"
#include <Arduino.h>

TS3633::TS3633() {
  for (uint8_t i=0;i<NUM_BASESTATIONS;i++) {
    active_basestation_ids[i] = 0x00000000;
    basestation_valid[i]=false;
  }
  two = fixed(2.0f,26);
}

void TS3633::set_reference_db(tracking_reference_db &ref) {
  ref_db = &ref;
}

// A Circular buffer is used to queue pulses to be processed by the main loop function.
// The purpose of this is to allow a fast execution function that can be called from a timer ISR
void TS3633::queue_pulse_for_processing(uint32_t period, uint32_t pulse_width) {
  pulse_fifo[fifo_queue_ptr].p = period;
  pulse_fifo[fifo_queue_ptr].pw = pulse_width;
  fifo_queue_ptr++;
  if (fifo_queue_ptr>=FIFO_DEPTH) {
    fifo_queue_ptr = 0;
  }
}

void TS3633::loop() {
  // Check the fifo to determine if new data has been added
  if (fifo_dequeue_ptr != fifo_queue_ptr) {
    
    uint32_t pulse_period = pulse_fifo[fifo_dequeue_ptr].p;
    uint32_t pulse_width = pulse_fifo[fifo_dequeue_ptr].pw;

    // Increment all time registers with measured time since last pulse (pulse_period)
    frame_time += pulse_period;
    
    //TODO: Use pre-processor or other optimization means to unroll this loop, save some instructions
    for (uint8_t b=0;b<NUM_BASESTATIONS;b++){
      sync_period[b][HORIZONTAL] += pulse_period;
      sync_period[b][VERTICAL] += pulse_period;
      time_since_last_centroid[b][HORIZONTAL] += pulse_period;
      time_since_last_centroid[b][VERTICAL] += pulse_period;
    }
  
    // Use frame time to determine if a frame has completed
    if (frame_time > FRAME_TIMEOUT_TIME) {
      // If the frame is valid, process the rotor data into Centroids and subsequent tracking data
      if (frame_valid) {
        // Calculate the centroid
        int32_t current_centroid = 0;

        if (active_basestation==BASESTATION_B) {
          rotor_hit_frame_time -= OPTICAL_OFFSET_48MHz_Ticks;
        }

        if (active_frame_id.bit.axis == VERTICAL) {
          current_centroid = ((FP_360_DEGREES/rotor_hit_sync_period)*(rotor_hit_frame_time + (rotor_hit_width >> 1)))-FP_90_DEGREES;
        }
        else {
          current_centroid = FP_90_DEGREES-((FP_360_DEGREES/rotor_hit_sync_period)*(rotor_hit_frame_time + (rotor_hit_width >> 1)));
        }
        
  
        // Cache the centroid and velocity info from last frame
        int32_t prev_centroid = centroid[active_basestation][active_frame_id.bit.axis];
        int32_t prev_velocity = centroid_velocity[active_basestation][active_frame_id.bit.axis];
  
        // Calculate the new velocity and acceleration
        int32_t current_velocity = (current_centroid - prev_centroid) / rotor_hit_dt;
        centroid_acceleration[active_basestation][active_frame_id.bit.axis] = (current_velocity - prev_velocity) / rotor_hit_dt;
  
        // Commit calculations to the registers
        centroid[active_basestation][active_frame_id.bit.axis] = current_centroid; 
        centroid_velocity[active_basestation][active_frame_id.bit.axis] = current_velocity;
  
        // Update the time since last centroid regisiter
        time_since_last_centroid[active_basestation][active_frame_id.bit.axis] -= rotor_hit_dt;

        // Detect if a new basestation has been identified
        uint32_t temp_bs_id = bs_ootx[active_basestation].get_basestation_id();
        if (active_basestation_ids[active_basestation] != temp_bs_id) {
          // A new basestation is found, update the internal reference to its pose and cache the ID so that this only runs on change
          active_basestation_ids[active_basestation] = temp_bs_id;
          reference_pose_rtn_t temp_bs_pose_rtn = ref_db->get_reference_pose(temp_bs_id);
          
          if (temp_bs_pose_rtn.valid) {
            basestation_coords[active_basestation] = temp_bs_pose_rtn.pose;
            basestation_valid[active_basestation] = true;
          }

          if (basestationFoundIRQattached) {
            basestation_found_IRQ_func(temp_bs_id);
          }
        }
        
        // if the object is setup for callbacks when new angles are available, convert them to float and publish
        if (angleIRQattached){
          // Convert fixed point values for centroids to floating point for API convenience.
          // Warning: use of floats will make the code slower and consume more flash.
          basestation_angles[active_basestation].basestation_id = bs_ootx[active_basestation].get_basestation_id();
          basestation_angles[active_basestation].horizontal_angle = 360*(float)centroid[active_basestation][HORIZONTAL]/(float)FP_360_DEGREES;
          basestation_angles[active_basestation].vertical_angle = 360*(float)centroid[active_basestation][VERTICAL]/(float)FP_360_DEGREES;

          // Call the function attached to this IRQ to notify the upstream code that new angles are available!
          angle_IRQ_func();
        }

        if (basestation_valid[0] && basestation_valid[1]) {
          frame_count++;
          // Use custom direct solver method to calculate the objects position
          uint8_t current_axis = active_frame_id.bit.axis;
          object_position = beno_algorithim(current_axis, active_basestation, prev_active_basestation);
        
          // if the object is setup for callbacks when new position is available
          if (positionIRQattached){
            // Call the function attached to this IRQ to notify the upstream code that new position is available!
            position_IRQ_func();
          }
        }

      } //end of frame valid check
      // Reset state machine variables to start a new frame
      frame_state = SYNC_DETECT;
      basestation = 0;
      frame_valid = false;
    } //end of frame timeout check
    // Determine if pulse is arriving after the expected frame time that a sync pulse would be expected
    else if (frame_time > SYNC_BLANKING_TIME) {
      // Determine validity of the sync data found earlier in the frame
      if (frame_valid) {
        // Frame is valid, enter rotor detect state
        frame_state = ROTOR_DETECT;
        num_rotors[active_basestation][active_frame_id.bit.axis] = 0;
        rotor_hit_width = 0;
      } // End of Frame Valid State Transition
      else {
        // Frame is not valid, restart the state machine back at the sync detect
        frame_state = SYNC_DETECT;
        basestation = 0;
        frame_valid = false;
      } // End of Frame NOT Valid State Transition
    } //end of sync_blanking_time check
  
    // Decode the pulse to determine if it is a sync, and what information is encoded in this sync
    pulse_id[basestation] = pulse_decode(pulse_width);
  
    // Digest the OOTX bit into the state machine
    bs_ootx[basestation].digest_bit(pulse_id[basestation].bit.data);
  
    if (pulse_id[basestation].bit.is_sync == PULSE_ID_IS_SYNC){
      // Pulse might be a sync because of width
      if (frame_state == SYNC_DETECT){
        // We're expecting syncs and currently in the sync detect state so that is one level of validation
        if (basestation == 0) {
          // Pulse is the first sync in the frame
          frame_time = 0; //reset the frame time register
        } // End of basestation == 0 check
        else {
          // Pulse is not the first in frame and is a sync.
          // All syncs in a given frame should have the same value for AXIS
          if (pulse_id[0].bit.axis != pulse_id[basestation].bit.axis){
            // If the decoded values for axis do not match, this frame should be invalid
            frame_valid = false;
          } // End of AXIS check for syncs in a frame
        } // End of basestation != 0 check
  
        // Check if decoded pulse indicates that it should NOT be skipped 
        if (pulse_id[basestation].bit.skip == SKIP_FALSE) {
          
          // check for any previous Skip FALSE inside the frame
          if (frame_valid) {
            // Redundant skip = false information, unable to determine what the active frame
            // is so we need to invalidate it
            frame_valid = false;
          }
          else {
            // Since SKIP = False, and so far there has not been another, this is the active Rotor, commit this inforamtion 
            
            if (basestation != active_basestation) {
              prev_active_basestation = active_basestation;
              active_basestation = basestation;
            }
            active_frame_id = pulse_id[basestation];
            active_sync_period = sync_period[basestation][pulse_id[basestation].bit.axis];
            frame_valid = true;
          }
        } // End of Skip False Check
        sync_period[basestation][pulse_id[basestation].bit.axis] = 0;
        basestation += 1;
      } // End of frame_state == SYNC_DETECT check
    } // End of "Is sync" check
    else {
      // Pulse is NOT a sync so the it is a rotor
      // Check if the frame is still valid and that the sync period is in an acceptable range 
      if (active_sync_period > MINIMUM_SYNC_PERIOD && active_sync_period < MAXIMUM_SYNC_PERIOD && frame_valid){
        // if everthing looks good, we assume that this is a valid rotor, for the rest of the frame, 
        // find the largest rotor width
        
        // keep count of how many rotors are observed in this frame
        num_rotors[active_basestation][active_frame_id.bit.axis] += 1;
  
        // Experimental Reflection filter, only take the largest width rotor pulse in the frame
        // TODO: We may decide that it is just better to throw away a frame with more than one rotor
        if (pulse_width > rotor_hit_width){
          rotor_hit_width = pulse_width;
          rotor_hit_frame_time = frame_time;
          rotor_hit_sync_period = active_sync_period;
          rotor_hit_dt = time_since_last_centroid[active_basestation][active_frame_id.bit.axis];
        }
      } // End of Frame Valid and Rotor Period Checks
      else {
        // The frame is either not valid or the sync period is out of range
        // Reset the state machine
        frame_time = RESET_FRAME_TIMER;
        frame_valid = false;
      } // end of frame not valid / sync period is not valid check
    }// end of NOT is_sync check

    fifo_dequeue_ptr++;
    if (fifo_dequeue_ptr >= FIFO_DEPTH) {
      fifo_dequeue_ptr = 0;
    }
  }
}

// The pulse decode function analyzes  a sync pulse to decode information that is encoded in its width.  The decoded values
// are based on this un-official resource: https://github.com/nairol/LighthouseRedox/blob/master/docs/Light%20Emissions.md
// This is an un-official resource but appears to be accurate, no "official" resource available

pulse_decode_t TS3633::pulse_decode(uint32_t pulse_width) {
  // Based on pulsewidth, determine if this is a sync
  // Syncs will be between 50usec and 140usec with the current protocol, beware, rotors can have this width too.
  pulse_decode_t rtn;
  if (pulse_width>50*TICKS_PER_US and pulse_width<140*TICKS_PER_US) {
    if (pulse_width<100*TICKS_PER_US) {
      // Skip = False
      if (pulse_width<80*TICKS_PER_US) {
        // OOTX = 0
        if (pulse_width<70*TICKS_PER_US) {
          // Horizontal
          rtn.raw = (uint8_t) (IS_SYNC | SKIP_FALSE | AXIS_H | DATA_0);
        }
        else {
          // Veritcal
          rtn.raw = (uint8_t) (IS_SYNC | SKIP_FALSE | AXIS_V | DATA_0);
        }
      } // End Skip False, OOTX 0
      else {
        // OOTX = 1
        if (pulse_width<90*TICKS_PER_US) {
          // Horizontal
          rtn.raw = (uint8_t) (IS_SYNC | SKIP_FALSE | AXIS_H | DATA_1);
        }
        else {
          // Vertical
          rtn.raw = (uint8_t) (IS_SYNC | SKIP_FALSE | AXIS_V | DATA_1);
        }
      } // End Skip False, OOTX 1
    } // End Skip False
    else {
      // Skip = True
      if (pulse_width<120*TICKS_PER_US) {
        // OOTX = 0
        if (pulse_width<110*TICKS_PER_US) {
          // Horizontal
          rtn.raw = (uint8_t) (IS_SYNC | SKIP_TRUE | AXIS_H | DATA_0);
        }
        else {
          // Vertical
          rtn.raw = (uint8_t) (IS_SYNC | SKIP_TRUE | AXIS_V | DATA_0);
        }
        
      } //End Skip True, OOTX 0
      else {
        // OOTX = 1
        if (pulse_width<130*TICKS_PER_US) {
          // Horizontal
          rtn.raw = (uint8_t) (IS_SYNC | SKIP_TRUE | AXIS_H | DATA_1);
        }
        else {
          // Vertical
          rtn.raw = (uint8_t) (IS_SYNC | SKIP_TRUE | AXIS_V | DATA_1);
        }
      }//End Skip True OOTX 1
    }//End Skip True
  }
  else {
    rtn.raw = (uint8_t) (IS_ROTOR);
  }
  return rtn;
}


// The Beno algorithim is a direct solver method for finding the x,y,z cartesian coordinates of an object based on relative angles from
// two reference objects.  It is highly optimized to run fast and efficent on embedded processors with limited ALU instruction sets
// both fixed point and floating point version of this code exist but fixed point is used here for speed.
struct vec3_t TS3633::beno_algorithim(uint8_t active_axis, uint8_t active_bs,  uint8_t prev_active_bs) {
  vec3_t result;
  /*
  //TODO: Make this work :)
  // Interpolate new centroids based on velocity and acceleration then find the sine/cosine
  for (int b = 0; b< NUM_BASESTATIONS;b++){
    for (int a = 0; a<2;a++){
      centroid[b][a] = centroid[b][a] + centroid_velocity[b][a]*time_since_last_centroid[b][a] + centroid_acceleration[b][a]*time_since_last_centroid[b][a]*time_since_last_centroid[b][a];
      cordic(centroid[b][a]>>1, &sine[b][a], &cosine[b][a], 32);
      // TODO: Write a piplined cordic that will run all 4 calcs in on loop 
    }
  }
  */
  // Temproary use of single cordic, this really only works good for slow moving objects, need interpolation...

  cordic(centroid[active_bs][active_axis]>>1, &sine[active_bs][active_axis], &cosine[active_bs][active_axis], 32);
  
  quaternion_t sensor_q;
  sensor_q.w = fixed(sine[active_bs][HORIZONTAL],ANGLE_Q,false) * fixed(cosine[active_bs][VERTICAL],ANGLE_Q,false);
  sensor_q.x = fixed(-1*cosine[active_bs][HORIZONTAL],ANGLE_Q,false) * fixed(sine[active_bs][VERTICAL],ANGLE_Q,false);
  sensor_q.y = fixed(sine[active_bs][HORIZONTAL],ANGLE_Q,false) * fixed(cosine[active_bs][VERTICAL],ANGLE_Q,false);
  sensor_q.z = fixed(sine[active_bs][HORIZONTAL],ANGLE_Q,false) * fixed(cosine[active_bs][VERTICAL],ANGLE_Q,false);
  
  quaternion_t sensor_global_q = q_mult(sensor_q,basestation_coords[active_bs].rotation);

  anchor_points[active_bs][Px_1] =  two*sensor_global_q.w*sensor_global_q.y + two*sensor_global_q.x*sensor_global_q.z;
  anchor_points[active_bs][Px_2] = ~two*sensor_global_q.w*sensor_global_q.x + two*sensor_global_q.y*sensor_global_q.z;
  anchor_points[active_bs][Px_3] =    sensor_global_q.w*sensor_global_q.w -   sensor_global_q.x*sensor_global_q.x - sensor_global_q.y*sensor_global_q.y + sensor_global_q.z*sensor_global_q.z;
  anchor_points[active_bs][Px_4] = ~two*sensor_global_q.w*sensor_global_q.z + two*sensor_global_q.x*sensor_global_q.y;
  anchor_points[active_bs][Px_5] =    sensor_global_q.w*sensor_global_q.w -   sensor_global_q.x*sensor_global_q.x + sensor_global_q.y*sensor_global_q.y - sensor_global_q.z*sensor_global_q.z;
  anchor_points[active_bs][Px_6] =  two*sensor_global_q.w*sensor_global_q.x + two*sensor_global_q.y*sensor_global_q.z;

  // TODO: Optimize the rest of this function to use these variables by reference instead of putting them on the stack
  fixed u_01 = ~anchor_points[prev_active_bs][Px_1]/anchor_points[active_bs][Px_1];
  
  fixed u_02 = ~anchor_points[prev_active_bs][Px_4]/anchor_points[active_bs][Px_1];

  fixed d_0  =(basestation_coords[prev_active_bs].pos.x/anchor_points[active_bs][Px_1])-(basestation_coords[active_bs].pos.x/anchor_points[active_bs][Px_1]);
  
  fixed l_11 = anchor_points[prev_active_bs][Px_2] + anchor_points[active_bs][Px_2] * u_01;
  
  fixed l_21 = anchor_points[prev_active_bs][Px_3] + anchor_points[active_bs][Px_3] * u_01;
  
  fixed u_12 = anchor_points[prev_active_bs][Px_5]  - anchor_points[active_bs][Px_2] * u_02;
  
  fixed l_22 = anchor_points[prev_active_bs][Px_6] - anchor_points[active_bs][Px_3] * u_02;
  
  fixed d_1  = (basestation_coords[active_bs].pos.y - basestation_coords[prev_active_bs].pos.y + d_0*anchor_points[active_bs][Px_2]) / l_11;

  u_12       = u_12/l_11;
  
  l_22       = l_22 - l_21*u_12;

  fixed x2_numerator =  basestation_coords[active_bs].pos.z - basestation_coords[prev_active_bs].pos.z - d_1*l_21 + d_0*anchor_points[active_bs][Px_3];
  
  fixed x_2 = x2_numerator / l_22;
  
  fixed x_1  = d_1 - u_12*x_2;
  
  fixed x_0  = d_0 - u_01*d_1 + u_01*u_12*x_2 - u_02*x_2;

  result.x = basestation_coords[active_bs].pos.x + anchor_points[active_bs][Px_1] * x_0;
  result.y = basestation_coords[active_bs].pos.y + anchor_points[active_bs][Px_2] * x_0;
  result.z = basestation_coords[active_bs].pos.z + anchor_points[active_bs][Px_3] * x_0;
  
  return result;
}
/*
struct vec3_t TS3633::beno_algorithim_debug(uint8_t active_axis, uint8_t active_bs,  uint8_t prev_active_bs) {
  vec3_t result;
  /*
  //TODO: Make this work :)
  // Interpolate new centroids based on velocity and acceleration then find the sine/cosine
  for (int b = 0; b< NUM_BASESTATIONS;b++){
    for (int a = 0; a<2;a++){
      centroid[b][a] = centroid[b][a] + centroid_velocity[b][a]*time_since_last_centroid[b][a] + centroid_acceleration[b][a]*time_since_last_centroid[b][a]*time_since_last_centroid[b][a];
      cordic(centroid[b][a]>>1, &sine[b][a], &cosine[b][a], 32);
      // TODO: Write a piplined cordic that will run all 4 calcs in on loop 
    }
  }
  
  // Temproary use of single cordic, this really only works good for slow moving objects, need interpolation...
  cordic(centroid[active_bs][active_axis]>>1, &sine[active_bs][active_axis], &cosine[active_bs][active_axis], 32);

  quaternion_t sensor_q;
  sensor_q.w = fixed(sine[active_bs][active_axis],ANGLE_Q,false) * fixed(cosine[active_bs][active_axis],ANGLE_Q,false);
  sensor_q.x = fixed(-1*cosine[active_bs][active_axis],ANGLE_Q,false) * fixed(sine[active_bs][active_axis],ANGLE_Q,false);
  sensor_q.y = fixed(sine[active_bs][active_axis],ANGLE_Q,false) * fixed(cosine[active_bs][active_axis],ANGLE_Q,false);
  sensor_q.z = fixed(sine[active_bs][active_axis],ANGLE_Q,false) * fixed(cosine[active_bs][active_axis],ANGLE_Q,false);
  quaternion_t sensor_global_q = q_mult(basestation_coords[active_bs].rotation,sensor_q);
  anchor_points[active_bs] = point_gen(sensor_global_q);

  // TODO: Optimize the rest of this function to use these variables by reference instead of putting them on the stack
  vec3_t pa = anchor_points[active_bs];
  vec3_t pb = anchor_points[prev_active_bs];

  fixed u_01 =    ~pb.x / pa.x;
  fixed u_02 =    ~pb.y / pa.x;
  fixed d_0b =    basestation_coords[prev_active_bs].pos.x  / pa.x;
  fixed d_0a =    basestation_coords[active_bs].pos.x  / pa.x;
  fixed l_11 = pb.y - pa.y * u_01;
  fixed l_21 = pb.z - pa.z * u_01;
  fixed u_12 = pb.z - pa.y * u_02;
  fixed l_22 = pb.x - pa.z * u_02;
  fixed d_0 = d_0b - d_0a;
  fixed d1_numerator  = pa.y * d_0b - pa.y * d_0a + basestation_coords[active_bs].pos.y - basestation_coords[prev_active_bs].pos.y;
  fixed d_1  = d1_numerator / l_11;

  // Kinda dirty, the shift operator takes the precision down by q-1 because it seems like overflow occurs here
  // the test case is: math.log2(((int(-961426689)<<int(30))/-292183852)) = 31.718300405403845, should cap at 31, WTF?
  u_12 = (u_12>>1)/l_11;
  fixed n_pb = l_21 * u_12;
  l_22 = l_22 - n_pb;
  fixed x_2_numerator = basestation_coords[active_bs].pos.z - basestation_coords[prev_active_bs].pos.z + pa.z * d_0 - l_21 * d_1;
  fixed x_2 = x_2_numerator / l_22;
  fixed x_1 = d_1 - u_12 * x_2;
  fixed x_0 = d_0 - u_01 * x_1 - u_02 * x_2;
  result.x = basestation_coords[active_bs].pos.x + pa.x * x_0;
  result.y = basestation_coords[active_bs].pos.y + pa.y * x_0;
  result.z = basestation_coords[active_bs].pos.z + pa.z * x_0;
  
  return result;
}
*/
struct vec3_t TS3633::point_gen(quaternion_t a) {
  vec3_t result;
  result.x = a.w * a.y + a.w * a.y + a.x * a.z + a.x * a.z;
  result.y = a.y * a.z + a.y * a.z - a.w * a.x - a.w * a.x;
  result.z = a.w * a.w - a.x * a.x - a.y * a.y + a.z * a.z;
  return result;
}

// A Function for the multiplication of 2 quaternions
struct quaternion_t TS3633::q_mult(quaternion_t b, quaternion_t a) {
  quaternion_t result;
  result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return result;
}

// A CORDIC function is an excellent way to efficently calculate both the sine and cosine of an angle at the same time
// using only very simple bitwise operations, this is ideal for the Cortex M0+ limited ALU instruction set
// Very Thankful for excellent open resources, like this: http://www.dcs.gla.ac.uk/~jhw/cordic/
void TS3633::cordic(int32_t theta, int32_t *s, int32_t *c, int32_t n)
{
  int32_t k, d, tx, ty, tz;
  int32_t x=cordic_1K,y=0,z=theta;
  n = (n>CORDIC_NTAB) ? CORDIC_NTAB : n;
  for (k=0; k<n; ++k)
  {
    d = z>>31;
    //get sign. for other architectures, you might want to use the more portable version
    //d = z>=0 ? 0 : -1;
    tx = x - (((y>>k) ^ d) - d);
    ty = y + (((x>>k) ^ d) - d);
    tz = z - ((cordic_ctab[k] ^ d) - d);
    x = tx; y = ty; z = tz;
  }  
 *c = x; *s = y;
}

void TS3633::attachAngleIRQ(void (*func)(void)) {
  angleIRQattached = true;
  angle_IRQ_func = func;
}

void TS3633::attachPositionIRQ(void (*func)(void)) {
  positionIRQattached = true;
  position_IRQ_func = func;
}

void TS3633::attachBasestationFoundIRQ(void (*func)(uint32_t bs_id)) {
  basestationFoundIRQattached = true;
  basestation_found_IRQ_func = func;
}

float_vec3_t TS3633::get_position() {
  float_vec3_t rtn;
  rtn.x = (float) object_position.x;
  rtn.y = (float) object_position.y;
  rtn.z = (float) object_position.z;
  return rtn;
}

void TS3633::debug_print_fixed(fixed num){
  Serial.print(num.rawVal);
  Serial.print(" (q=");
  Serial.print(num.q);
  Serial.print(") ");
}
