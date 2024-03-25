// SPDX-FileCopyrightText: 2023 Carter Nelson for Adafruit Industries
//
// SPDX-License-Identifier: MIT
// --------------------------------------
// i2c_scanner
//
// Modified from https://playground.arduino.cc/Main/I2cScanner/
// --------------------------------------

#include <Wire.h>


#define sclPin2 PB10
#define sdaPin2 PB3

// Set I2C bus to use: Wire, Wire1, etc.
// TwoWire Wire2(sdaPin2, sclPin2);
#define WIRE Wire

#define ledPin PB13


void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  WIRE.begin();

  // Serial.begin(9600);
  // while (!Serial)
  // {
  //     digitalWrite(ledPin, !digitalRead(ledPin));
  //     delay(100);
  // }
  // Serial.println("\nI2C Scanner");

  digitalWrite(ledPin, HIGH);
  for (uint8_t i = 0; i < 6; i++) {
    digitalWrite(ledPin, !digitalRead(ledPin));
    delay(100);
  }
  digitalWrite(ledPin, LOW);
}


void loop() {
  byte error, address;
  int nDevices;
  int addrs[8];

  // Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    WIRE.beginTransmission(address);
    error = WIRE.endTransmission();

    if (error == 0)
    {
      // Serial.print("I2C device found at address 0x");
      // if (address<16)
        // Serial.print("0");
      // Serial.print(address,HEX);
      // Serial.println("  !");

      addrs[nDevices] = address;
      nDevices++;
    }
    // else if (error == 4)
    // {
    // //   Serial.print("Unknown error at address 0x");
    // //   if (address<16)
    // //     Serial.print("0");
    // //   Serial.println(address,HEX);
    // }
  }



  digitalWrite(ledPin, HIGH);
  delay(2000);
  for (uint8_t i = 0; i < nDevices*2; i++) {
      digitalWrite(ledPin, !digitalRead(ledPin));
      delay(100);
  }
  digitalWrite(ledPin, LOW);
  delay(2000);

  if (nDevices > 0)
  {
    if (addrs[0] < 100){
      for (int i = 0; i < nDevices; i++)
      {
        int tens = addrs[i] / 10;
        int ones = addrs[i] - tens * 10;
        for (uint8_t i = 0; i < tens; i++) {
          digitalWrite(ledPin, HIGH);
          delay(100);
          digitalWrite(ledPin, LOW);
          delay(100);
        }
        delay(500);
        for (uint8_t i = 0; i < ones; i++) {
          digitalWrite(ledPin, HIGH);
          delay(100);
          digitalWrite(ledPin, LOW);
          delay(100);
        }
        delay(1000);
      }
    }
  }

  // if (nDevices == 0)
  //   Serial.println("No I2C devices found\n");
  // else
  //   Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}
