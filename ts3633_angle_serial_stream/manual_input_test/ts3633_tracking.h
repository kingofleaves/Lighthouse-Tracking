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

#ifndef ts3633_tracking_h
#define ts3633_tracking_h

#include <stdint.h>
#include "ootx.h"
#include "typedef_3d.h"
#include "third_party_includes/Fp32s.hpp"
#include "tracking_reference_db.h"

#ifndef fixed
  #define fixed Fp::Fp32s
#endif

#define DEBUG_PRINTF

#define TICKS_PER_US 48

#define SYNC_BLANKING_TIME 1000*TICKS_PER_US
#define FRAME_TIMEOUT_TIME 8000*TICKS_PER_US

#define RESET_FRAME_TIMER  FRAME_TIMEOUT_TIME + 1

#define MINIMUM_SYNC_PERIOD 16400*TICKS_PER_US
#define MAXIMUM_SYNC_PERIOD 16900*TICKS_PER_US

#define NUM_BASESTATIONS 2

#define BASESTATION_A   0x00
#define BASESTATION_B   0x01

#define SKIP_FALSE   0x00
#define SKIP_TRUE    0x01
#define AXIS_H       0x00
#define AXIS_V       0x02
#define DATA_0       0x00
#define DATA_1       0x04
#define IS_ROTOR     0x00
#define IS_SYNC      0x08

#define PULSE_ID_IS_SYNC 0x01

#define SYNC_DETECT  0x00
#define ROTOR_DETECT 0x01

#define HORIZONTAL   0x00
#define VERTICAL     0x01
#define SYNC_PULSE   0x01
#define ROTOR_PULSE  0x00

#define OPTICAL_OFFSET_48MHz_Ticks 20000

#define FP_360_DEGREES 1073741824
#define FP_90_DEGREES 1073741824/4
#define cordic_1K 0x26DD3B6A
#define CORDIC_NTAB 32

#define ANGLE_Q 30

#define FIFO_DEPTH 10

#define Px_1 0
#define Px_2 1
#define Px_3 2
#define Px_4 3
#define Px_5 4
#define Px_6 5

typedef struct {
  uint32_t p;
  uint32_t pw;
} ppw_t;

typedef struct {
  uint32_t basestation_id;
  float    horizontal_angle;
  float    vertical_angle;
} basestation_angles_t;

typedef union {
  struct {
    uint8_t skip:1;    // bit 0 used to define if the Sync pulse is associated with the next rotor pulse
    uint8_t axis:1;    // bit 1 used to define if the Sync pulse is associated with a horizontal or vertical rotor
    uint8_t data:1;    // bit 2 used to define the ootx bit for this message
    uint8_t is_sync:1; // bit 3 tells us if the pulse is decoded as a rotor or sync
    uint8_t :4;        // unused
  } bit;
  uint8_t raw;
} pulse_decode_t;

class TS3633 {

  public:
    TS3633();
    void set_reference_db(tracking_reference_db &ref);
    void queue_pulse_for_processing(uint32_t period,uint32_t pulse_width);
    void loop();
    void attachAngleIRQ(void (*func)(void));
    void attachPositionIRQ(void (*func)(void));
    void attachBasestationFoundIRQ(void (*func)(uint32_t));
    basestation_angles_t get_angles_deg();
    float_vec3_t get_position();
    basestation_angles_t basestation_angles[NUM_BASESTATIONS];
    uint32_t frame_count = 0;
    bool debug = false;
    
  private:
    void debug_print_fixed(fixed num);
    bool basestation_valid[NUM_BASESTATIONS];
    tracking_reference_db *ref_db;
    uint32_t active_basestation_ids[NUM_BASESTATIONS];
    // State Machine Variables for protocol decode
    pulse_decode_t pulse_id[NUM_BASESTATIONS+1];
    uint8_t frame_state = SYNC_DETECT;
    uint8_t basestation = 0;
    uint32_t frame_time = FRAME_TIMEOUT_TIME + 1;
    uint8_t active_basestation = 0;
    uint8_t prev_active_basestation = 0;
    uint32_t active_sync_period = 0;
    pulse_decode_t active_frame_id;
    bool frame_valid = false;

    // Cordic Related variables:
    int cordic_ctab [32] = {0x3243F6A8, 0x1DAC6705, 0x0FADBAFC, 0x07F56EA6, 0x03FEAB76, 0x01FFD55B, 0x00FFFAAA, 0x007FFF55, 0x003FFFEA, 0x001FFFFD, 0x000FFFFF, 0x0007FFFF, 0x0003FFFF, 0x0001FFFF, 0x0000FFFF, 0x00007FFF, 0x00003FFF, 0x00001FFF, 0x00000FFF, 0x000007FF, 0x000003FF, 0x000001FF, 0x000000FF, 0x0000007F, 0x0000003F, 0x0000001F, 0x0000000F, 0x00000008, 0x00000004, 0x00000002, 0x00000001, 0x00000000 };

    
    // FSM Variables for Rotor Hits
    uint32_t rotor_hit_width = 0;
    uint32_t rotor_hit_frame_time = 0;
    uint32_t rotor_hit_sync_period = 0;
    uint32_t rotor_hit_dt = 0;
    
    // Raw Decoded Tracking Variables
    uint32_t sync_period[NUM_BASESTATIONS][2];
    int32_t centroid[NUM_BASESTATIONS][2];
    int32_t sine[NUM_BASESTATIONS][2];
    int32_t cosine[NUM_BASESTATIONS][2];
    int32_t centroid_velocity[NUM_BASESTATIONS][2];
    int32_t centroid_acceleration[NUM_BASESTATIONS][2];
    uint8_t num_rotors[NUM_BASESTATIONS][2];
    
    //TODO: Rewrite time since last centroid implementation to be timer based instead of accumulator
    uint32_t time_since_last_centroid[NUM_BASESTATIONS][2];
    
    // Basesation OOTX State Machines
    OOTX bs_ootx[NUM_BASESTATIONS];
    
    // Tracking variables
    fixed two;
    coord_t basestation_coords[NUM_BASESTATIONS];
    fixed anchor_points[NUM_BASESTATIONS][6];
    vec3_t object_position;

    //Private functions
    pulse_decode_t pulse_decode(uint32_t pulse_width);
    struct vec3_t beno_algorithim(uint8_t active_bs, uint8_t active_axis, uint8_t prev_active_bs);
    struct vec3_t beno_algorithim_debug(uint8_t active_bs, uint8_t active_axis, uint8_t prev_active_bs);
    void cordic(int32_t theta, int32_t *s, int32_t *c, int32_t n);
    struct quaternion_t q_mult(quaternion_t b, quaternion_t a);
    struct vec3_t point_gen(quaternion_t a);

    volatile uint8_t fifo_queue_ptr = 0;
    volatile uint8_t fifo_dequeue_ptr = 0;
    volatile ppw_t pulse_fifo[FIFO_DEPTH];

    // Callback methods
    bool angleIRQattached = false;
    bool positionIRQattached = false;
    bool basestationFoundIRQattached = false;
    void (*angle_IRQ_func)(void);
    void (*position_IRQ_func)(void);
    void (*basestation_found_IRQ_func)(uint32_t bs_id);
};

#endif
