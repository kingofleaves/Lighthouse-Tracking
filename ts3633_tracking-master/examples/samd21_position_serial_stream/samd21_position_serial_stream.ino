#include "ts3633_tracking.h"
#include "tracking_reference_db.h"

#define SENSOR1_PIN 4

TS3633 sensor1;
tracking_reference_db ref_db;

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial);
  tcc0_init(SENSOR1_PIN);

  Serial.println("Hello");

  // Link the tracking reference database to the ts3633
  sensor1.set_reference_db(ref_db);
  

  // Add Tracking References, these are Basestation serials and x,y,z coordinates and quaternions as reported by OpenVR, Unity, Unreal, etc...
  ref_db.add_tracking_reference(0x7C72128C,                                                 // Basestation ID
      2.118725299835205, 2.163483142852783, -1.7002456188201904,                            // Basestation x,y,z location
      0.3422205451790754, -0.019673010972005057, 0.9022262956857341, 0.2616978193536931);   // Basestation quaternion

  ref_db.add_tracking_reference(0x4F9DF94E,                                                 // Basestation ID
      -2.0368056297302246, 2.2454826831817627, 1.924940824508667,                           // Basestation x,y,z location
      0.8961803693737093, -0.3388199222412536, -0.27698794365910884, -0.07307246835137406); // Basestation quaternion

  // Insert more references if you like, the library will automatically lookup references and calculate position
  // this means that you can move the object from one tracked space to another and it can adjust

  // Debug function to write out references to Serial
  ref_db.print_tracking_reference();

  sensor1.attachBasestationFoundIRQ(sensor1_bs_found_irq);

  // Attach interrupt to call "sensor1_position_irq" everytime that the position registers for sensor1 are updated
  sensor1.attachPositionIRQ(sensor1_position_irq);
}

boolean run_loop = true;

// the loop function runs over and over again forever
void loop() {

  if (run_loop) {
    // sensor1 requires this function to be executed on every loop
    sensor1.loop();
  }
  if (Serial.available() > 0) {
      run_loop = false;
  }
}

void sensor1_bs_found_irq(uint32_t bs_id) {
  Serial.print("New Basestation Found: ");
  Serial.println(bs_id,HEX);
}

void sensor1_position_irq() {
  float_vec3_t p1 = sensor1.get_position();
  Serial.print(p1.x);
  Serial.print(",");
  Serial.print(p1.y);
  Serial.print(",");
  Serial.println(p1.z);
}

// Interrupt for TCC0, bind this IRQ to Sensor1
void TCC0_Handler()
{
  // The MC1 interrupt occures when the PPW capture is complete, per the SAMD21 docs, 
  // The period will be stored in CC0 and the Pulse Width will be stored in CC1
  if (TCC0->INTFLAG.bit.MC1)  {
    sensor1.queue_pulse_for_processing(REG_TCC0_CC0, REG_TCC0_CC1);
  }
}

void tcc0_init(uint16_t pin_number) {
  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS;     // Switch on the event system peripheral
 
  
  // Enable clock for TCC0
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC0_TCC1) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync 
                          
  REG_EIC_EVCTRL |= 1<<digitalPinToInterrupt(pin_number);                         // Enable event output on external ie EIC_EVCTRL_EXTINTEO4
  attachInterrupt(pin_number, NULL, LOW);                                                  // Attach interrupts to digital pin

  REG_EVSYS_USER = EVSYS_USER_CHANNEL(1) |                                        // Attach the event user (receiver) to channel 0 (n + 1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC0_EV_1);                      // Set the event user (receiver) as timer TCC0, event 1

  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |                        // No event edge detection
                      EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                           // Set event path as asynchronous
                      EVSYS_CHANNEL_EVGEN(1<<digitalPinToInterrupt(pin_number)) | // Set event generator (sender) as external interrupt ie EVSYS_ID_GEN_EIC_EXTINT_4
                      EVSYS_CHANNEL_CHANNEL(0);                                   // Attach the generator (sender) to channel 0

  REG_TCC0_EVCTRL |= TCC_EVCTRL_MCEI1 |           // Enable the match or capture channel 1 event input
                     TCC_EVCTRL_MCEI0 |           //.Enable the match or capture channel 0 event input
                     TCC_EVCTRL_TCEI1 |           // Enable the TCC event 1 input
                     /*TCC_EVCTRL_TCINV1 |*/      // Invert the event 1 input         
                     TCC_EVCTRL_EVACT1_PPW;       // Set up the timer for capture: CC0 period, CC1 pulsewidth
                                        

  NVIC_SetPriority(TCC0_IRQn, 0);      // Set the Nested Vector Interrupt Controller (NVIC) priority for TCC0 to 0 (highest)
  NVIC_EnableIRQ(TCC0_IRQn);           // Connect the TCC0 timer to the Nested Vector Interrupt Controller (NVIC)
 
  REG_TCC0_INTENSET = TCC_INTENSET_MC1; //|            // Enable compare channel 1 (CC1) interrupts
                      //TCC_INTENSET_MC0;             // Enable compare channel 0 (CC0) interrupts
 
  REG_TCC0_CTRLA |= TCC_CTRLA_CPTEN1 |              // Enable capture on CC1
                    TCC_CTRLA_CPTEN0 |              // Enable capture on CC0
                    TCC_CTRLA_PRESCALER_DIV1 |     // Set prescaler to 48MHz
                    TCC_CTRLA_ENABLE;               // Enable TCC0
                    
  while (TCC0->SYNCBUSY.bit.ENABLE);                // Wait for synchronization
 }

