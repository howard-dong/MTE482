//I2C Master Code (STM32F103C8)
//I2C Communication between STM32 and Arduino
//Circuit Digest

#include<Wire.h>                      
// #include<SoftWire.h>                   //Library for I2C Communication functions
#include "Adafruit_MCP9808.h"

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();


// Defines
#define peltierPin PA4
#define pwmPin PA5 // Green LED on Nucleo
#define ledPin PB13
#define buttonPin PA3
#define switchPin PA2
// TODO: Test what the peltier period needs to be, seems like 8-12ish seconds is safe; care of cycling too fast bricks peltier
#define peltierPeriod 1000

// #define sclPin PB10
// #define sdaPin PB3

// TwoWire myWire(sclPin, sdaPin, 9600);
// TwoWire Wire2(sclPin, sdaPin);
int state = 0;

// Button Interrupt Callback
void buttonISR() {
  if (HIGH == digitalRead(switchPin))
  {
    Wire.beginTransmission(0x08);                 // starts transmit to device (8-Slave Arduino Address)
    Wire.write(state);                             // sends the value state to Slave
    Wire.endTransmission();                    // stop transmitting
    state = 1 - state;
  }
}


void setup() 
{ 

  // Pin Setups
  pinMode(peltierPin, OUTPUT);
  digitalWrite(peltierPin, LOW);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(buttonPin, INPUT_PULLDOWN);
  pinMode(switchPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, FALLING);

  // Serial.begin(9600);                  //Begins Serial Communication at 9600 baud rate
  Wire.begin();                        //Begins I2C communication at pin (PB10, PB3)


  // Temperature sensor
  if (!tempsensor.begin())
  {
    while (!tempsensor.begin(0x18)) { // Can't find MCP9808
      digitalWrite(ledPin, !digitalRead(ledPin));
      delay(100);
    }
  }

  tempsensor.setResolution(3); // sets the resolution mode of reading, the modes are defined in the table bellow:
  // Mode Resolution SampleTime
  //  0    0.5°C       30 ms
  //  1    0.25°C      65 ms
  //  2    0.125°C     130 ms
  //  3    0.0625°C    250 ms

  tempsensor.wake(); // wake up, ready to read!
  float t_read = tempsensor.readTempC();


  // Finish Setup
  digitalWrite(ledPin, HIGH);
  for (uint8_t i = 0; i < 10; i++) {
    digitalWrite(ledPin, !digitalRead(ledPin));
    delay(100);
  }
  digitalWrite(ledPin, LOW);  
}



void loop()
{ 
  if (HIGH == digitalRead(switchPin)) // MASTER READ MODE
  {
    float t_read = tempsensor.readTempC();
    // Press button to change led on slave
    digitalWrite(ledPin, HIGH);
    Wire.beginTransmission(0x08);                 // starts transmit to device (8-Slave Arduino Address)
    uint8_t data[4] = {0, 1, 2, 3};
    Wire.write((byte*)&data, sizeof(data));                             // sends the value state to Slave
    Wire.endTransmission();                    // stop transmitting

    // Wire.beginTransmission(0x08);
    // // Wire.write(state, sizeof(state));
    // Wire.write(state);
    // Wire.endTransmission();
    // state = 1 - state;
  }
  else
  {
    Wire.requestFrom(0x08,1);               // request  bytes from slave arduino(8)
    byte slave_button = Wire.read();                // receive byte from the slave arduino and store in variable slave_button
    if (slave_button==1)                            //Logic to turn Master LED ON (if received value is 1) or OFF (if received value is 0
    { 
      digitalWrite(ledPin,HIGH);
      // Serial.println("Master LED ON");
    }

    else
    {
      digitalWrite(ledPin,LOW);
      // Serial.println("Master LED OFF");
    }
  }
  delay(500);
}