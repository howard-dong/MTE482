// #include <Wire.h>
// #include "Adafruit_MCP9808.h"
// #include <PulseSensorPlayground.h> // Includes the PulseSensorPlayground Library.
// #include "include/flash_lib.h"

// const int peltier = PA4;
// const int peltier = PA8;

// #define pin3 D7
#define peltierPin PA4
#define pwmPin PA5 // Green LED on Nucleo
#define ledPin PB13
#define buttonPin PA3
#define switchPin PA2
#define peltierPeriod 1000

bool peltierActive = false;

// void Update_IT_callback(void)
// { // Update event correspond to Rising edge of PWM when configured in PWM1 mode
//   digitalWrite(peltierPin, LOW); // pin2 will be complementary to pin
// }

// void Compare_IT_callback(void)
// { // Compare match event correspond to falling edge of PWM when configured in PWM1 mode
//   if (peltierActive) {
//     digitalWrite(peltierPin, HIGH);
//   }
// }

// int peltierInterval = 667;

void buttonISR() {
  if(HIGH == digitalRead(switchPin)) { // if in manual mode, do some stuff
    // digitalWrite(peltierPin, HIGH);
    digitalWrite(ledPin, !digitalRead(ledPin));
    peltierActive = !peltierActive;
  }
}

void setup() {
  pinMode(peltierPin, OUTPUT);
  digitalWrite(peltierPin, LOW);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(buttonPin, INPUT_PULLDOWN);
  pinMode(switchPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(buttonPin),buttonISR,FALLING);

  // // Automatically retrieve TIM instance and channel associated to pwmPin
  // // This is used to be compatible with all STM32 series automatically.
  // TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pwmPin), PinMap_PWM);
  // uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pwmPin), PinMap_PWM));


  // // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup function is finished.
  // HardwareTimer *MyTim = new HardwareTimer(Instance);

  // MyTim->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pwmPin);
  // // MyTim->setPrescaleFactor(8); // Due to setOverflow with MICROSEC_FORMAT, prescaler will be computed automatically based on timer input clock
  // // MyTim->setOverflow(10000, MICROSEC_FORMAT);                     // 100000 microseconds = 100 milliseconds
  // MyTim->setOverflow(1000, HERTZ_FORMAT);                     // 100000 microseconds = 100 milliseconds
  // MyTim->setCaptureCompare(channel, 40, PERCENT_COMPARE_FORMAT);  // 50%
  // MyTim->attachInterrupt(Update_IT_callback);
  // MyTim->attachInterrupt(channel, Compare_IT_callback);
  // MyTim->resume();
}

bool buttonPressed = false;
uint8_t peltierPWM = LOW;

void loop() {
  ;
  // int pressEvent = digitalRead(buttonPin);
  // if (HIGH == pressEvent) {
  //   digitalWrite(ledPin, HIGH);
  // }
  // else {
  //   digitalWrite(ledPin, LOW);
  // }
  // delay(10);

  unsigned long currentMillis = millis();
  uint8_t pressEvent = digitalRead(buttonPin);
  uint8_t manualMode = digitalRead(switchPin);

  if (LOW == manualMode) { // NOT in manual mode
    digitalWrite(ledPin, LOW);
    digitalWrite(peltierPin, LOW);
    peltierActive = false;

    
  }
  else {
    // digitalWrite(ledPin, HIGH);

    if (peltierActive) {
      if ((currentMillis % peltierPeriod) <= (int)(peltierPeriod*0.5)) {
        peltierPWM = HIGH;
      } 
      else {
        peltierPWM = LOW;
      }
    } 
    else {
      peltierPWM = LOW;
    }
    // digitalWrite(ledPin, peltierPWM);
    digitalWrite(peltierPin, peltierPWM);
  }
  // delay(10);





  // digitalWrite(ledPin, !digitalRead(ledPin));
  // delay(100);
  /* Nothing to do all is done by hardware. Even no interrupt required. */
  // if (LOW == peltierActive) {
  //   peltierActive = HIGH;
  // }
  // else {
  //   peltierActive = LOW; 
  // }
  // // peltierActive = !peltierActive;
  // delay(500);

  // digitalWrite(ledPin, !digitalRead(ledPin));
  // if (peltierActive) {
    
  //   digitalWrite(peltierPin, HIGH);
  //   delay(67);
  //   digitalWrite(peltierPin, LOW);
  //   delay(600);
  // }

}
