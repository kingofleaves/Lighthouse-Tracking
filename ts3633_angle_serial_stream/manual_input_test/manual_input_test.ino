#include "ts3633_tracking.h"
#include "location_2d.h"

#define SENSOR1_PIN 20
 
int max_index = 100;
uint32_t *debug_array = new uint32_t[max_index];
int debug_index = 0;

//#ifdef TICKS_PER_US
//#undef TICKS_PER_US
//#endif
//#define TICKS_PER_US 96
//// To overwrite the settings for SAMD21 in the library nd port over to Teensy 3.2 (with 96MHZ clock)

#define INTENDED_CLOCK_F  48000000
#define INTENDED_TICKS_PER_US  48

volatile uint16_t count = 0;
volatile uint32_t timeSpace = 0;
volatile uint32_t timeMark = 0;
volatile bool cycleComplete = false;
volatile bool isFalling = true;


TS3633 sensor1;

// How the lighthouse works: http://i.kinja-img.com/gawker-media/image/upload/s--wsP3xmPN--/1259287828241194666.gif 

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial);

  pinMode(SENSOR1_PIN, INPUT); 

  //tcc0_init(SENSOR1_PIN); // for SAMD21, not teensy 3.2

  Serial.println("Hello");
  Serial.println(TICKS_PER_US);
  Serial.println(F_BUS);
  Serial.println(F_PLL);
  sensor1.attachBasestationFoundIRQ(sensor1_bs_found_irq);

  // Attach interrupt to call "sensor1_angle_irq" everytime that the angle registers in sensor1 are updated
  sensor1.attachAngleIRQ(sensor1_angle_irq);
}

boolean run_loop = true;

// the loop function runs over and over again forever
void loop() {
  if (run_loop) {
    //queuePulse();
    test2D(); 
    // sensor1 requires this function to be executed on every loop
    sensor1.loop();
//    // DEBUG:
//    Serial.print(sensor1.debugPulse.bit.skip);
//    Serial.print(sensor1.debugPulse.bit.axis);
//    Serial.print(sensor1.debugPulse.bit.data);
//    Serial.print(sensor1.debugPulse.bit.is_sync);
//    Serial.print(sensor1.debugPulse.raw);
//    Serial.println();
//    if (millis() > 3000) run_loop = false;
  }
}

void queuePulse(void) {
  String input = "";
  if (Serial.available()) {
    input = Serial.readString();
    int split_index = input.indexOf(',');  //finds location of first ,
    String pulse_width = input.substring(0, split_index).trim();   //captures first data String
    String period = input.substring(split_index+1).trim(); 
//    Serial.println(pulse_width);
//    Serial.println(period);
    sensor1.queue_pulse_for_processing(period.toInt(), pulse_width.toInt());
  }
}

void test2D(void) {
  LOC2D test_unit;
  test_unit.init(0, 0, 1, 1);
  test_unit.variable_distance = true; // TODO: Test to see if this is the cause of the huge jitter -> i.e. change this to false
  test_unit.debug = true;
  test_unit.sensor_separation = 300;
  String input = "";
  if (Serial.available()) {
    input = Serial.readString();
    int split_index_1 = input.indexOf(',');  //finds location of first ,
    String angle_1 = input.substring(0, split_index_1).trim();   
    input = input.substring(split_index_1 + 1);
    int split_index_2 = input.indexOf(',');  //finds location of second,
    String angle_2 = input.substring(0, split_index_2).trim();
    input = input.substring(split_index_2 + 1);
    int split_index_3 = input.indexOf(',');  //finds location of third,
    String angle_3 = input.substring(0, split_index_3).trim();   
    String angle_4 = input.substring(split_index_3 + 1).trim();
     
// TODO: Put in initializer for angles array and pass it into LOC2D to test.
    float *angles = new float[4];
    angles[0] = angle_1.toFloat();
    angles[1] = angle_2.toFloat();
    angles[2] = angle_3.toFloat();
    angles[3] = angle_4.toFloat();

    if(test_unit.debug) {
      Serial.print("angle 1: ");
      Serial.println(angle_1);
      Serial.print("index 1: ");
      Serial.println(split_index_1);
      Serial.print("angle 2: ");
      Serial.println(angle_2);
      Serial.print("index 2: ");
      Serial.println(split_index_2);
      Serial.print("angle 3: ");
      Serial.println(angle_3);
      Serial.print("index 3: ");
      Serial.println(split_index_3);
      Serial.print("angle 4: ");
      Serial.println(angle_4);
      
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
    Serial.print("Position: X: ");
    Serial.println(test_loc.position.x);
    Serial.print("Position: Y: ");
    Serial.println(test_loc.position.y);
    Serial.print("Orientation: ");
    Serial.println(test_loc.orientation);
  }
}

void sensor1_bs_found_irq(uint32_t bs_id) {
  Serial.print("New Basestation Found: ");
  Serial.println(bs_id,HEX);
}

void sensor1_angle_irq() {
//  Serial.print("PW");
//  Serial.println(sensor1_freq.countToNanoseconds(timeSpace)/1000);
//  Serial.print("Period");
//  Serial.println(sensor1_freq.countToNanoseconds(timeSpace + timeMark)/1000);
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

