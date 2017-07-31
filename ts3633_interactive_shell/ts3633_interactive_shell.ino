#include <FreqMeasureMulti.h>
#include "third_party_includes/Cmd.h"
#include "ts3633_tracking.h"
#include "tracking_reference_db.h"

#define SENSOR1_PIN 20

#ifdef TICKS_PER_US
#undef TICKS_PER_US
#endif
#define TICKS_PER_US 96
// To overwrite the settings for SAMD21 in the library nd port over to Teensy 3.2 (with 96MHZ clock)

volatile uint16_t count = 0;
volatile uint32_t timeSpace;
volatile uint32_t timeMark;
volatile bool cycleComplete = false;
volatile bool isFalling = true;

FreqMeasureMulti sensor1_freq;

TS3633 sensor1;
tracking_reference_db ref_db;

bool angle_stream_en = false;
bool position_stream_en = false;

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial);

  cmdInit();
  //cmdAdd("help", cmd_help);
  //cmdAdd("add_tracking_ref", cmd_add_tracking_ref);
  cmdAdd("print_tracking_references", cmd_print_tracking_references);
  cmdAdd("stream_angles", cmd_stream_angles);
  cmdAdd("stream_position", cmd_stream_positions);
  cmdAdd("stop", cmd_stop);

  sensor1_freq.begin(SENSOR1_PIN, FREQMEASUREMULTI_ALTERNATE);

  Serial.println("TS3633 Tracking Interactive Shell");

  // Link the tracking reference database to the ts3633
  sensor1.set_reference_db(ref_db);
  
ref_db.print_tracking_reference();
  // Add Tracking References, these are Basestation serials and x,y,z coordinates and quaternions as reported by OpenVR, Unity, Unreal, etc...
  ref_db.add_tracking_reference(0x7C72128C,                                                 // Basestation ID
      2.118725299835205, 2.163483142852783, -1.7002456188201904,                            // Basestation x,y,z location
      0.3422205451790754, -0.019673010972005057, 0.9022262956857341, 0.2616978193536931);   // Basestation quaternion

  ref_db.add_tracking_reference(0x4F9DF94E,                                                 // Basestation ID
      -2.0368056297302246, 2.2454826831817627, 1.924940824508667,                           // Basestation x,y,z location
      0.8961803693737093, -0.3388199222412536, -0.27698794365910884, -0.07307246835137406); // Basestation quaternion

  ref_db.print_tracking_reference();
  // Insert more references if you like, the library will automatically lookup references and calculate position
  // this means that you can move the object from one tracked space to another and it can adjust

  sensor1.attachBasestationFoundIRQ(sensor1_bs_found_irq);

  // Attach interrupt to call "sensor1_angle_irq" everytime that the angle registers in sensor1 are updated
  sensor1.attachAngleIRQ(sensor1_angle_irq);

  // Attach interrupt to call "sensor1_position_irq" everytime that the position registers for sensor1 are updated
  sensor1.attachPositionIRQ(sensor1_position_irq);
}

// the loop function runs over and over again forever
void loop() {

  queuePulse();

  // sensor1 requires this function to be executed on every loop
  sensor1.loop();
  if (angle_stream_en || position_stream_en) {
    if (cmd_key_pressed() == true) {
      angle_stream_en = false;
      position_stream_en = false;
    }
  }

  //cmdPoll();
}

void queuePulse(void) {
  if(sensor1_freq.available()){
    uint32_t readValue = sensor1_freq.read();
    switch (sensor1_freq.readLevel()){
      case LEVEL_SPACE_ONLY:
        // IR received
        timeSpace = readValue;
        break;
      case LEVEL_MARK_ONLY:
        // no IR received
        timeMark = readValue;
        sensor1.queue_pulse_for_processing(timeSpace + timeMark, timeSpace);
//        Serial.print("Period: ");
//        Serial.println(timeSpace + timeMark);
//        Serial.print("Pulse Width: ");
//        Serial.println(timeSpace);
        break;
    }
  } 
}

void sensor1_bs_found_irq(uint32_t bs_id) {
  Serial.print("\nNew Basestation Found: ");
  Serial.println(bs_id,HEX);
  cmd_display();
}


void sensor1_angle_irq() {
  if (angle_stream_en) {
    Serial.print("BS A (");
    Serial.print(sensor1.basestation_angles[BASESTATION_A].basestation_id,HEX);
    Serial.print(") H=");
    Serial.print(sensor1.basestation_angles[BASESTATION_A].horizontal_angle);
    Serial.print(", V=");
    Serial.print(sensor1.basestation_angles[BASESTATION_A].vertical_angle);
    Serial.print(" | BS B (");
    Serial.print(sensor1.basestation_angles[BASESTATION_B].basestation_id,HEX);
    Serial.print(") H=");
    Serial.print(sensor1.basestation_angles[BASESTATION_B].horizontal_angle);
    Serial.print(", V=");
    Serial.println(sensor1.basestation_angles[BASESTATION_B].vertical_angle);
  }
}

void sensor1_position_irq() {
  if (position_stream_en) {
    float_vec3_t p1 = sensor1.get_position();
    Serial.print(p1.x);
    Serial.print(",");
    Serial.print(p1.y);
    Serial.print(",");
    Serial.println(p1.z);
  }
}

void cmd_print_tracking_references(int arg_cnt, char **args) {
  ref_db.print_tracking_reference();
}

void cmd_stream_angles(int arg_cnt, char **args) {
  angle_stream_en = true;
  look_for_key_press();
}

void cmd_stream_positions(int arg_cnt, char **args) {
  position_stream_en = true;
  look_for_key_press();
}

void cmd_stop(int arg_cnt, char **args) {
  angle_stream_en = false;
  position_stream_en = false;
}


