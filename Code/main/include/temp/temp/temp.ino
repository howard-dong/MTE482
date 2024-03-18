
// #include <Wire.h>
// #include "Adafruit_MCP9808.h"

// #define ledPin PB13

// // Create the MCP9808 temperature sensor object
// Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

// void setup() {
//   Wire.begin();
//   pinMode(ledPin, OUTPUT); // Set the LED pin as an output
//   digitalWrite(ledPin, HIGH);
//   delay(1000);
//   digitalWrite(ledPin, LOW);
//   delay(1000);
// }

// void loop() {
//     uint8_t address = 0x18; // Address to check
//     Wire.beginTransmission(address);
//     uint8_t error = Wire.endTransmission();

//     if (0U == error) {
//         // Device at address is available, turn on the LED
//         digitalWrite(ledPin, HIGH);
//         for (uint8_t counter = 0U; counter < 10; counter++) {
//           digitalWrite(ledPin, !digitalRead(ledPin));
//           delay(250);
//         }
//     } else if (4U == error) {
//         // Unknown error occurred
//         digitalWrite(ledPin, HIGH);
//         delay(4000); // Wait for 4 seconds
//         digitalWrite(ledPin, LOW); // Turn off the LED
//     } else {
//         // No device found at address
//         digitalWrite(ledPin, HIGH);
//         delay(1000*error);
//         // for (uint8_t counter = 0U; counter < 10; counter++) {
//         //   digitalWrite(ledPin, !digitalRead(ledPin));
//         //   delay(500);
//         // }
//     }

//     digitalWrite(ledPin, LOW);
//     delay(1000);
// }














#include <Wire.h>
#include "Adafruit_MCP9808.h"

#define ledPin PB13

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();



void setup()
{
  // Wire.begin(); // Initialize I2C
  // Wire.setClock(400000); // Set I2C clock frequency to 400kHz

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay(1000);
  digitalWrite(ledPin, LOW);
  delay(1000);  

  // Temperature sensor address (default)
  if (!tempsensor.begin(0x18))
  {
    while (1) {
      digitalWrite(ledPin, !digitalRead(ledPin));
      delay(100);
    }
  }
   else {
    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    delay(1000);
   }

  tempsensor.setResolution(3); // sets the resolution mode of reading, the modes are defined in the table bellow:
                                 // Mode Resolution SampleTime
                                 //  0    0.5°C       30 ms
                                 //  1    0.25°C      65 ms
                                 //  2    0.125°C     130 ms
                                 //  3    0.0625°C    250 ms
    tempsensor.wake(); // wake up, ready to read!
}

int prev_temp = 0;

void loop()
{
    // ---------- DATA COLLECTION ----------

    // TEMPERATURE SENSOR

    float c = tempsensor.readTempC();
    // tempsensor.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere, stops temperature sampling

    digitalWrite(ledPin, HIGH);

    // int num_temp_digits = 4;
    // int temp_int = int(c * (100));
    // int blinks[4] = {};
    // for (int digit = 0; digit < num_temp_digits; digit ++) {
    //   blinks[digit] = temp_int % 10;
    //   temp_int = temp_int / 10;
    // }

    // for (int digit = 0; digit >= 0; digit --) {
    //   for(uint8_t i = 0; i < blinks[digit]; i++) {
    //       digitalWrite(ledPin, HIGH);
    //       delay(100);
    //       digitalWrite(ledPin, LOW);
    //       delay(100);
    //     }
    //     delay(500);
    // }

    if (prev_temp != int(c)) {
      prev_temp = int(c);
      for(uint8_t i = 0; i < 2; i++) {
          digitalWrite(ledPin, HIGH);
          delay(100);
          digitalWrite(ledPin, LOW);
          delay(100);
        }
        delay(500);
    }


 
    // for(uint8_t i = 0; i < 2; i++) {
    //   // digitalWrite(ledPin, !digitalRead(ledPin));
    //   // delay(100);
    //   digitalWrite(ledPin, HIGH);
    //   delay(100);
    //   digitalWrite(ledPin, LOW);
    //   delay(100);
    // }
    // digitalWrite(ledPin, LOW);
    // delay(1000);

    // if (20.0 < c) {
    //   digitalWrite(ledPin, HIGH);
    // }
    // else {
    //   digitalWrite(ledPin, LOW);
    // }

    // delay(20); // considered best practice in a simple sketch.
}
