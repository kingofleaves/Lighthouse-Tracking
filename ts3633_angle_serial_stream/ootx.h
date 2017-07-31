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
       written consent of Triad Semiconductor.
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

#include <stdint.h>

#ifndef ootx_h
#define ootx_h

#define MAX_PAYLOAD 37

// TODO: Figure out how to deal with half precision floating point
typedef struct {
  uint16_t sign:1;
  uint16_t exponent:5;
  uint16_t fraction:10;
} float16_t;

// OOTX packet format as defined here: https://github.com/nairol/LighthouseRedox/blob/master/docs/Base%20Station.md
// This is an un-official resource but appears to be accurate, no "official" resource available
typedef union {
  struct {
    uint16_t payload_length;
    uint16_t fw_version;
    uint32_t basestation_id;
    float16_t fcal_0_phase;
    float16_t fcal_1_phase;
    float16_t fcal_0_tilt;
    float16_t fcal_1_tilt;
    uint8_t unlock_count;
    uint8_t hw_version;
    float16_t fcal_0_curve;
    float16_t fcal_1_curve;
    uint8_t accel_dir_x;
    uint8_t accel_dir_y;
    uint8_t accel_dir_z;
    float16_t fcal_0_gibphase;
    float16_t fcal_1_gibphase;
    float16_t fcal_0_gibmag;
    float16_t fcal_1_gibmag;
    uint32_t crc32;
  } reg;
  uint8_t raw[MAX_PAYLOAD];
} ootx_t;

// Define the class for the OOTX statemachine
class OOTX {
	public:
		OOTX(void);
		void digest_bit(uint8_t bit);
    uint32_t get_basestation_id();

	private:
	  bool syncronized;
    bool stuff_bit_in_byte;
    bool skip;
	  uint8_t num_zeros;
	  uint16_t byte_index;
	  uint8_t bit_index;
	  uint8_t valid_payload;
    uint8_t wip_payload;
	  uint16_t payload_len;
	  ootx_t payload[2];
};
#endif
