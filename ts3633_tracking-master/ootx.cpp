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
    4. Use of this source code and/or binary forms in commercial products shall require explicit
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

#include "ootx.h"
#include <Arduino.h>

// Constructor, initialize variables just like we are waiting for the sync word
OOTX::OOTX(void) {
	syncronized = false;
	num_zeros = 0;
	payload_len = 2;
	wip_payload = 0;
	valid_payload = 1;
}

// For each ootx bit, run this routine, the state machine is based on the protocol description published 
// here: https://github.com/nairol/LighthouseRedox/blob/master/docs/Light%20Emissions.md
// This is an un-official resource but appears to be accurate, no "official" resource available
void OOTX::digest_bit(uint8_t data_bit) {
	if (syncronized){
    if (!skip) {
      payload[wip_payload].raw[byte_index] |= (0x01 & data_bit) << bit_index;
  		if (bit_index >0) {
  			bit_index--;
  		}
		  else {
			  bit_index = 7;
        if (byte_index>0) {
          if (byte_index==1){
            // First two bytes represent the payload length excluding the length bytes and CRC bytes, so add 6
            payload_len = payload[wip_payload].reg.payload_length+6;
          }
          else if (byte_index >= payload_len) {
            //TODO: Actually CRC32 check the payload to make sure that it does not contain errors
            uint8_t temp = valid_payload;
            valid_payload = wip_payload;
            wip_payload = temp;
            syncronized = false;
          }
        }
        byte_index++;
        payload[wip_payload].raw[byte_index] = 0;
        
        // Every 16 bits, there is a stuff bit, we are accululateing bytes so use "stuff bit in byte"
        // Boolean to track if we are expecting a stuff bit, toggle it to skip the stuff bit every other byte
        if (stuff_bit_in_byte) {
          skip = true;
          stuff_bit_in_byte = false;
        }
        else {
          stuff_bit_in_byte = true;
        }
		  }
		}
    else {
      skip = false;
    }
	}
 
  // Always count the number of consecutive zeros to syncronize the OOTX state machine to the "Preamble"
	if (data_bit == 0) {
		num_zeros++;
	}
	else if (data_bit == 1) { 
    // A preamble syncronization word is 17 zeros followed by a 1 so if we see 17 zeros before
    // landing here, we are now syncronized and can start shifting in bits
		if (num_zeros == 17) {
			syncronized = true;
			bit_index = 7;
			byte_index = 0;
      stuff_bit_in_byte = false;
      skip = false;
			payload[wip_payload].raw[byte_index] = 0;
		}
   num_zeros = 0;
	}
}

uint32_t OOTX::get_basestation_id() {
  return  payload[valid_payload].reg.basestation_id;
}

