#include <FreqMeasureMulti.h>

#define SENSOR1_PIN 20
#define TEST_OUTPUT 23

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
int arrayIndex;
int maxIndex = 30;
uint32_t *timeArray = new uint32_t[maxIndex];

FreqMeasureMulti sensor1_freq;

void setup(){
  Serial.begin(115200); 
  while (!Serial);
  // remember to set clock to 48MHz for correct handling of pulse width and period
  pinMode(SENSOR1_PIN, INPUT); 
  pinMode(TEST_OUTPUT, OUTPUT); 

  sensor1_freq.begin(SENSOR1_PIN, FREQMEASUREMULTI_ALTERNATE);
  analogWrite(TEST_OUTPUT, 48);
}

void loop()
{   
  if(sensor1_freq.available()){
    uint32_t readValue = sensor1_freq.read();
    timeArray[arrayIndex++] = sensor1_freq.countToNanoseconds(readValue);
    if (arrayIndex == maxIndex) {
      arrayIndex = 0;
      for (int i = 0; i < maxIndex; i++) {
        Serial.println(timeArray[i]);
      }
    }
    switch (sensor1_freq.readLevel()){
      case LEVEL_SPACE_ONLY:
        // IR received
        Serial.println("SPACE!!");
        timeSpace = readValue;
        break;
      case LEVEL_MARK_ONLY:
        // no IR received
        Serial.println("MARK!!");
        timeMark = readValue;
        //sensor1.queue_pulse_for_processing(timeSpace + timeMark, timeSpace)
//        Serial.print("Period: ");
//        Serial.println(timeSpace + timeMark);
//        Serial.print("Pulse Width: ");
//        Serial.println(timeSpace);
        break;
    }
  }
}


