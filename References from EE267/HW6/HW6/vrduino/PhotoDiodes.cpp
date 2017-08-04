#include "PhotoDiodes.h"
#include "Utils.h"

#define IR0 5 //rising sensor 1
#define IR1 6 //falling sensor 1
#define IR2 9 //rising sensor 2
#define IR3 10 //falling sensor 2
#define IR4 20 //rising sensor 3
#define IR5 21 //falling sensor 3
#define IR6 22 //rising sensor 4
#define IR7 23 //falling sensor 4

#if defined(KINETISK)
#define CLOCKS_PER_SECOND (F_BUS)
#elif defined(KINETISL)
// PLL is 48 Mhz, which is 24 clocks per microsecond, but
// there is a divide by two for some reason.
#define CLOCKS_PER_SECOND (F_PLL / 2)
#endif

void PhotoDiodes::initTracking()
{
  // Enable interrupts for each photodiode
  sensors_[0].begin(0, IR0, IR1);
  sensors_[1].begin(1, IR2, IR3);
  sensors_[2].begin(2, IR4, IR5);
  sensors_[3].begin(3, IR6, IR7);

  // Mode select switch on VRduino board. Input is pulled up internally, defaulting to B lighthouse.
  pinMode(lightHouseSelectPin_, INPUT_PULLUP);

  pinMode(standbyPin_, OUTPUT);
  digitalWrite(standbyPin_, LOW); //low to turn on position tracker sensors_

  lighthouseToWatch_ = (!digitalRead(lightHouseSelectPin_)) ? 0 : 1; //lighthouse A/B is 0, lighthouse C is 1;
  lighthousePolled_ = -1; //where the sweep came from
}

void PhotoDiodes::printBaseStation() {
  Serial.printf("Selected lighthouse mode: %d \n", lighthouseToWatch_);
}

// Update RAW measurements in clock ticks from the 4 photodiodes
// input:   none
// output:  return true : true when all 4 diodes have been updated

bool PhotoDiodes::updateClockTicks()
{
  // poll each photodiode to see if new timing values are available
  for (int i = 0 ; i < 4 ; i++) {
    // get handle to sensor
    LighthouseSensor * const s = &sensors_[i];

    // update sensor timing
    bool newTimingVal = false;

    // value is -1 if no new timing.
    // valid is 0 or 1 if Bx or By
    // valid is 2 or 3 if Cx or Cy
    int value = s->poll(); 
    if(debug) Serial.printf("Value returned from sensor %d polling: %d: ", i, value);
    newTimingVal = (value != -1); //new time data if val != -1. 
    
    //Only update our timing if we actually have a new value from polling.
    if(newTimingVal){
      if(value < 2) {
        lighthousePolled_ = 0;
      } else {
        lighthousePolled_ = 1;
      }
      if(debug) Serial.printf("lighthouse: %d \n", lighthousePolled_);
      if(lighthouseToWatch_ == lighthousePolled_){ //our polled lighthouse matches the desired lighthouse to update.
        if(debug) Serial.printf("Lighthouse watched matches lighthouse polled! Polled from %d \n", lighthousePolled_); 
        
        int diodeIndex = (2 * i + value%2); // (2*i) puts us at the correct photodiode locaiton and value%2 puts us at the correct axis.
        if(debug) Serial.printf("diodeIndex: %d \n", diodeIndex );
          updatedDiodes[diodeIndex] = 1; // set the true flag for the correct axis in the correct diode.
          //Utils::print2DArray(updatedDiodes,1,8);

          // get raw CPU clock tick values for x and y
          clockTicks_[2 * i]   = s->  raw[0 + 2*lighthousePolled_]; //which raw value depends on the lighthouse polled. See lighthouseSensor for understanding of raw values.
          clockTicks_[2 * i + 1] = s->raw[1 + 2*lighthousePolled_];
        if(debug) {
          Serial.println("======================================================================================================");
          Serial.printf("current sensor values: %d %d %d %d %d \n", i, s->raw[0], s->raw[1], s->raw[2], s->raw[3]);
          Serial.printf("values used for comp: %d, %d, %d, %d \n", i, lighthousePolled_, clockTicks_[2 * i], clockTicks_[2 * i + 1]);
          Serial.println("======================================================================================================");
        }
      }
    }
  }

  // Check if all diodes have been updated
  bool bSuccess = true;
  for (int i = 0; i < 8; i++) {
    if (updatedDiodes[i] == 0)
      bSuccess = false;
  }

  // If they have been updated reset flags, and return true indicating
  // values are ready to use
  if (bSuccess) {
    for (int i = 0; i < 8; i++)
      updatedDiodes[i] = 0;
  }
  //Serial.printf("bSuccess %d \n", bSuccess);
  return bSuccess;
}

// Update 2D positions, stored in projection2D_, from current clock ticks, stored
// in clockTicks_.
// input:   clockTicks_    - 1x8 array of timings from known starting location to horz/vert sweep
//                           intersection with diode
//                           [h0 v0 h1 v1 h2 v2 h3 v3] 
// outputs : projection2D_ - 1x8 array of the x,y projection of each diode onto
//                           the light house "sensor" plane, the plane a unit
//                           distance away
void PhotoDiodes::update2DPositions()
{
  for(int i = 0; i<8; i++){
    float dt = clockTicks_[i]/48000000.0;
    float alpha = (-dt*60.0*360.0+360.0/4.0)*pow(-1,i); // rotation in degrees
    projection2D_[i] = tan(alpha*3.1415/180.0);
    //Serial.println(projection2D_[i]);
  }
}

void PhotoDiodes::unitTest_update2DPositions() {
  uint32_t inputClockTicks[8] = {256122, 135108, 250133, 136912, 249863, 134095, 255844, 132298};
  float correctPos2D[8] = {-0.471734971, -0.55891335, -0.415437728, -0.540463567, -0.412953287, -0.569401562, -0.469068378, -0.588243663};

  memcpy(clockTicks_, inputClockTicks, 8 * sizeof(uint32_t));
  update2DPositions();

  Serial.println("Testing PhotoDiodes::update2DPositions()");
  float diffProj2D = Utils::ComputeL2Error(correctPos2D, projection2D_, 1,8);
  Serial.printf("Difference in Proj2D: %f \n", diffProj2D);
}
