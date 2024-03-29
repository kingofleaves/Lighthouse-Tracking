#include <FreqMeasureMulti.h>
#include "ts3633_tracking.h"
#include "location_2d.h"

#define SENSOR1_PIN 20
#define SENSOR2_PIN 23
 
int max_index = 100;
uint32_t *debug_array = new uint32_t[max_index];
int debug_index = 0;

#define INTENDED_CLOCK_F  48000000
#define INTENDED_TICKS_PER_US  48

volatile uint16_t count = 0;
volatile uint32_t timeSpace = 0;
volatile uint32_t timeMark = 0;
volatile uint32_t timeSpace2 = 0;
volatile uint32_t timeMark2 = 0;
volatile bool cycleComplete = false;
volatile bool isFalling = true;

float *angles = new float[4];
int angle_index = 0;
bool sensor1_updated = false;
bool sensor2_updated = false;


FreqMeasureMulti sensor1_freq;
FreqMeasureMulti sensor2_freq;

TS3633 sensor1;
TS3633 sensor2;

// How the lighthouse works: http://i.kinja-img.com/gawker-media/image/upload/s--wsP3xmPN--/1259287828241194666.gif 

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial);

  pinMode(SENSOR1_PIN, INPUT); 
  pinMode(SENSOR2_PIN, INPUT); 

  sensor1_freq.begin(SENSOR1_PIN, FREQMEASUREMULTI_ALTERNATE);
  sensor2_freq.begin(SENSOR2_PIN, FREQMEASUREMULTI_ALTERNATE);

  Serial.println("Hello");
  Serial.println(TICKS_PER_US);
  Serial.println(F_BUS);
  Serial.println(F_PLL);
  sensor1.attachBasestationFoundIRQ(sensor1_bs_found_irq);
  sensor2.attachBasestationFoundIRQ(sensor2_bs_found_irq);

  // Attach interrupt to call "sensor1_angle_irq" everytime that the angle registers in sensor1 are updated
  sensor1.attachAngleIRQ(sensor1_angle_irq);
  sensor2.attachAngleIRQ(sensor2_angle_irq);

  sensor1.debug = false;
  sensor2.debug = false;

}

boolean run_loop = true;

// the loop function runs over and over again forever
void loop() {
  if (run_loop) {
    queuePulses();
    
    // sensor1 requires this function to be executed on every loop
    sensor1.loop();
    sensor2.loop();

//    // DEBUG:
//    Serial.print(sensor1.debugPulse.bit.skip);
//    Serial.print(sensor1.debugPulse.bit.axis);
//    Serial.print(sensor1.debugPulse.bit.data);
//    Serial.print(sensor1.debugPulse.bit.is_sync);
//    Serial.print(sensor1.debugPulse.raw);
//    Serial.println();
//    if (millis() > 3000) run_loop = false;
  }
  if (Serial.available() > 0) {
      run_loop = false;
  }
}

void queuePulses(void) {
  if(sensor1_freq.available()){
    uint32_t readValue = sensor1_freq.read();
    switch (sensor1_freq.readLevel()){
      case LEVEL_MARK_ONLY:
        // no IR received
        timeMark = readValue;
        break;
      case LEVEL_SPACE_ONLY:
        // IR received
        timeSpace = readValue;
        sensor1.queue_pulse_for_processing(normalizeCount(timeSpace + timeMark), normalizeCount(timeSpace));
        break;  
    }
  } 
  if(sensor2_freq.available()){
    uint32_t readValue = sensor2_freq.read();
    switch (sensor2_freq.readLevel()){
      case LEVEL_MARK_ONLY:
        // no IR received
        timeMark2 = readValue;
        break;
      case LEVEL_SPACE_ONLY:
        // IR received
        timeSpace2 = readValue;
        sensor2.queue_pulse_for_processing(normalizeCount(timeSpace2 + timeMark2), normalizeCount(timeSpace2));
        break;  
    }
  } 

//    if(sensor1_freq.available()){
//    uint32_t readValue = sensor1_freq.read();
//    switch (sensor1_freq.readLevel()){
////      case LEVEL_SPACE_ONLY:
////        // IR received
////        timeSpace = readValue;
////        break;
//      case LEVEL_MARK_ONLY:
//        // no IR received
//        timeMark = readValue;
//        break;
//      case LEVEL_SPACE_ONLY:
//        // IR received
//        timeSpace = readValue;
//        sensor1.queue_pulse_for_processing(normalizeCount(timeSpace + timeMark), normalizeCount(timeSpace));
//        // DEBUG:
//        if (sensor1.debug) {
//          debug_array[debug_index++] = normalizeCount(timeSpace)/48;
//          debug_array[debug_index++] = normalizeCount(timeMark)/48;
//          if (debug_index >= max_index) {
//            debug_index = 0;
//            for (int i = 0; i < max_index; i++) {
//              Serial.println(debug_array[i]);
//            }
//          }
//  //        Serial.print("PW");
//  //        Serial.println(sensor1_freq.countToNanoseconds(timeSpace)/1000);
//  //        Serial.print("Period");
//  //        Serial.println(sensor1_freq.countToNanoseconds(timeSpace + timeMark)/1000);
//        }
//        break;  
//    }
//  } 
}

void test2D(void) {
  LOC2D test_unit;
  test_unit.init(0, 0, 1, 1);
  test_unit.variable_distance = false;
  test_unit.debug = true;
  test_unit.sensor_separation = 0.015;
     
// TODO: Put in initializer for angles array and pass it into LOC2D to test.
    if(test_unit.debug) {  
      Serial.print("angle 1: ");
      Serial.println(angles[0]);
      Serial.print("angle 2: ");
      Serial.println(angles[1]);
      Serial.print("angle 3: ");
      Serial.println(angles[2]);
      Serial.print("angle 4: ");
      Serial.println(angles[3]);
    }

    location_t test_loc = test_unit.get_location(angles);
    Serial.print("Position: X: (in mm) ");
    Serial.println(test_loc.position.x * 1000);
    Serial.print("Position: Y: (in mm) ");
    Serial.println(test_loc.position.y * 1000);
    Serial.print("Orientation: (in deg) ");
    Serial.println(test_loc.orientation);
}


void sensor1_bs_found_irq(uint32_t bs_id) {
  Serial.print("New Basestation Found: ");
  Serial.println(bs_id,HEX);
}

void sensor2_bs_found_irq(uint32_t bs_id) {
  Serial.print("New Basestation Found: ");
  Serial.println(bs_id,HEX);
}

void sensor1_angle_irq() {

  angles[0] = sensor1.basestation_angles[BASESTATION_A].horizontal_angle;
  angles[1] = sensor1.basestation_angles[BASESTATION_A].vertical_angle;
  sensor1_updated = true;
  if (sensor1_updated && sensor2_updated) {
    sensor1_updated = false;
    sensor2_updated = false;
    test2D();
  }
//  Serial.print("BS A (");
//  Serial.print(sensor1.basestation_angles[BASESTATION_A].basestation_id,HEX);
//  Serial.print(") H=");
//  Serial.print(sensor1.basestation_angles[BASESTATION_A].horizontal_angle);
//  Serial.print(", V=");
//  Serial.print(sensor1.basestation_angles[BASESTATION_A].vertical_angle);
//  Serial.print(" | BS B (");
//  Serial.print(sensor1.basestation_angles[BASESTATION_B].basestation_id,HEX);
//  Serial.print(") H=");
//  Serial.print(sensor1.basestation_angles[BASESTATION_B].horizontal_angle);
//  Serial.print(", V=");
//  Serial.println(sensor1.basestation_angles[BASESTATION_B].vertical_angle);
}

void sensor2_angle_irq() {

  angles[2] = sensor2.basestation_angles[BASESTATION_A].horizontal_angle;
  angles[3] = sensor2.basestation_angles[BASESTATION_A].vertical_angle;
  sensor2_updated = true;
  if (sensor1_updated && sensor2_updated) {
    sensor1_updated = false;
    sensor2_updated = false;
    test2D();
  }
}

uint32_t normalizeCount(uint32_t count)
{
  #if defined(__arm__) && defined(TEENSYDUINO) && defined(KINETISK)
//    Serial.println("BUS");
//    Serial.println( (count*INTENDED_TICKS_PER_US)/(F_BUS/1000000) );
//    Serial.println(count/48);
    return (count*INTENDED_TICKS_PER_US)/(F_BUS/1000000);
  #elif defined(__arm__) && defined(TEENSYDUINO) && defined(KINETISL)
//    Serial.println("PLL");
    return (count*INTENDED_TICKS_PER_US)*2/(F_PLL/1000000);
  #else
    Serial.println("error");
    return 0;
  #endif
//  return count;
}


//// Interrupt for TCC0, bind this IRQ to Sensor1
//void TCC0_Handler()
//{
//  // The MC1 interrupt occures when the PPW capture is complete, per the SAMD21 docs, 
//  // The period will be stored in CC0 and the Pulse Width will be stored in CC1
//  if (TCC0->INTFLAG.bit.MC1)  {
//    sensor1.queue_pulse_for_processing(REG_TCC0_CC0, REG_TCC0_CC1);
//  }
//}
//
//void tcc0_init(uint16_t pin_number) {
//  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS;     // Switch on the event system peripheral
// 
//  
//  // Enable clock for TCC0
//  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC0_TCC1) ;
//  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync 
//                          
//  REG_EIC_EVCTRL |= 1<<digitalPinToInterrupt(pin_number);                         // Enable event output on external ie EIC_EVCTRL_EXTINTEO4
//  attachInterrupt(pin_number, NULL, LOW);                                                  // Attach interrupts to digital pin
//
//  REG_EVSYS_USER = EVSYS_USER_CHANNEL(1) |                                        // Attach the event user (receiver) to channel 0 (n + 1)
//                   EVSYS_USER_USER(EVSYS_ID_USER_TCC0_EV_1);                      // Set the event user (receiver) as timer TCC0, event 1
//
//  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |                        // No event edge detection
//                      EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                           // Set event path as asynchronous
//                      EVSYS_CHANNEL_EVGEN(1<<digitalPinToInterrupt(pin_number)) | // Set event generator (sender) as external interrupt ie EVSYS_ID_GEN_EIC_EXTINT_4
//                      EVSYS_CHANNEL_CHANNEL(0);                                   // Attach the generator (sender) to channel 0
//
//  REG_TCC0_EVCTRL |= TCC_EVCTRL_MCEI1 |           // Enable the match or capture channel 1 event input
//                     TCC_EVCTRL_MCEI0 |           //.Enable the match or capture channel 0 event input
//                     TCC_EVCTRL_TCEI1 |           // Enable the TCC event 1 input
//                     /*TCC_EVCTRL_TCINV1 |*/      // Invert the event 1 input         
//                     TCC_EVCTRL_EVACT1_PPW;       // Set up the timer for capture: CC0 period, CC1 pulsewidth
//                                        
//
//  NVIC_SetPriority(TCC0_IRQn, 0);      // Set the Nested Vector Interrupt Controller (NVIC) priority for TCC0 to 0 (highest)
//  NVIC_EnableIRQ(TCC0_IRQn);           // Connect the TCC0 timer to the Nested Vector Interrupt Controller (NVIC)
// 
//  REG_TCC0_INTENSET = TCC_INTENSET_MC1; //|            // Enable compare channel 1 (CC1) interrupts
//                      //TCC_INTENSET_MC0;             // Enable compare channel 0 (CC0) interrupts
// 
//  REG_TCC0_CTRLA |= TCC_CTRLA_CPTEN1 |              // Enable capture on CC1
//                    TCC_CTRLA_CPTEN0 |              // Enable capture on CC0
//                    TCC_CTRLA_PRESCALER_DIV1 |     // Set prescaler to 48MHz
//                    TCC_CTRLA_ENABLE;               // Enable TCC0
//                    
//  while (TCC0->SYNCBUSY.bit.ENABLE);                // Wait for synchronization
// }
