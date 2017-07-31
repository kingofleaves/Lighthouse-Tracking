#include "ts3633_tracking.h"

#define SENSOR1_PIN 20

bool prevState = true;

TS3633 sensor1;

unsigned long timestamp = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SENSOR1_PIN, INPUT);  
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Hello");
}

void loop() {
  // put your main code here, to run repeatedly:
//  bool currState = digitalRead(SENSOR1_PIN);
//  if (currState != prevState) {
//    prevState = currState;
//    Serial.print("Switched States: ");
//    Serial.println(currState);
//  }

  if (millis() > timestamp) {
    Serial.println(digitalRead(SENSOR1_PIN));
    timestamp += 1000;
  }

} 
