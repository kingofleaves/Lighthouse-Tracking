#define PIN_INT 20

volatile uint16_t count = 0;
volatile uint32_t timeFirstRise;
volatile uint32_t timeFirstFall;
volatile uint32_t timeSecondFall;
volatile bool cycleComplete = false;
volatile bool isFalling = true;

void setup(){
  Serial.begin(115200);

  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  // Set up clock to count 
  // get clock cycles by: cycles = ARM_DWT_CYCCNT;  
  
  pinMode(PIN_INT, INPUT); 
//  attachInterrupt(digitalPinToInterrupt(PIN_INT), ISR_rising, RISING); 
//  attachInterrupt(digitalPinToInterrupt(PIN_INT), ISR_falling, FALLING); 
  attachInterrupt(digitalPinToInterrupt(PIN_INT), ISR_change, CHANGE); 
}

void loop()
{
  if (cycleComplete) {
    uint32_t pulse_width = timeFirstRise - timeFirstFall;
    uint32_t period = timeSecondFall - timeFirstFall;
//    Serial.print("Pulse Width = ");
//    Serial.println(pulse_width);
//    Serial.print("Period = ");
//    Serial.println(period);
    cycleComplete = false;
  }  
}

void ISR_change()
{
  if(cycleComplete) return;
  cli();
  if (isFalling) {
    if (count == 0) {
      timeFirstFall = ARM_DWT_CYCCNT;
      // save current time as starting time of pulse
    } else {
      timeSecondFall = ARM_DWT_CYCCNT;
      // calculate period from current time
      cycleComplete = true;
    }
  } else {
    if (count == 0) {
      timeFirstRise = ARM_DWT_CYCCNT;
      // calculate pulse width from current time
      count++;
    } else {
      count = 0;
    }
  }
  isFalling = !isFalling;
  sei();
}




void ISR_rising()
{
  cli();
  if (count == 0) {
    timeFirstRise = ARM_DWT_CYCCNT;
    // calculate pulse width from current time
    count++;
  } else {
    count = 0;
  }
  sei();
}

void ISR_falling()
{
  cli();
  if (count == 0) {
    cycleComplete = false;
    timeFirstFall = ARM_DWT_CYCCNT;
    // save current time as starting time of pulse
  } else {
    timeSecondFall = ARM_DWT_CYCCNT;
    // calculate period from current time
    cycleComplete = true;
  }
  sei();
}
