// I2C Preprocessor

#include<Wire.h>
#include "Adafruit_MCP9808.h"


// Defines
#define peltierPin PA4
#define pwmPin PA5 // Green LED on Nucleo
#define ledPin PB13
#define buttonPin PA3
#define switchPin PA2
// TODO: Test what the peltier period needs to be, seems like 8-12ish seconds is safe; care of cycling too fast bricks peltier
#define peltierPeriod 1000

#define pulsePin PA0

#define DATA_LEN sizeof(float) + sizeof(uint8_t) + sizeof(uint32_t)

int state = 0;
const float t_avg = 20.0;
const uint8_t bpm_avg = 80;


// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

// union tempData_S
// {
//   float rawData;
//   byte  byteData[sizeof(float)] = {0};
// } tempData;

// union rawPulseData_S
// {
//   uint32_t rawData;
//   byte byteData[sizeof(uint32_t)] = {0};
// } rawPulseData;

// struct pulseData_S
// {
//   uint8_t bpm = 0;
//   rawPulseData_S rawPulseData;
// };

// union moistureData_S
// {
//   uint32_t rawData = 0U;
//   byte byteData[sizeof(uint32_t)]
// }

// struct i2cData_S
// {
//   // moistureData_S      moistureData = {0};
//   float hr;
//   uint8_t bpm;
//   uint32_t temp;
//   pulseData_S         pulseData;
//   tempData_S          tempData;
// } i2cData;

uint8_t data[DATA_LEN] = {0};


// Button Interrupt Callback
void buttonISR() {
  // if (HIGH == digitalRead(switchPin))
  // {
  //   Wire.beginTransmission(0x08);                 // starts transmit to device (8-Slave Arduino Address)
  //   Wire.write(state);                             // sends the value state to Slave
  //   Wire.endTransmission();                    // stop transmitting
  //   state = 1 - state;
  // }
}

// void updateData() {
//   i2cData.pulseData.rawPulseData.rawData = analogRead(pulsePin);
//   i2cData.pulseData.bpm = bpm_avg;
//   i2cData.tempData.rawData = tempsensor.readTempC();
// }

// void updateData() {
//   i2cData.pulseData.rawPulseData.rawData++;
//   i2cData.pulseData.bpm = bpm_avg;
//   i2cData.tempData.rawData += (random(-200, 200)/100);
// }

void updateData() {
  // memcpy(data[0], tempData.byteData, sizeof(tempData.rawData));  
  // memcpy(data[sizeof(float)], tempData.byteData, sizeof(tempData.rawData));
  // for (int i = 0; i < DATA_LEN)
  // {
  //   memcpy(bytes_array, u.temp_array, 4)    
  // }
  // i2cData.pulseData.rawPulseData.rawData = 500;
  // i2cData.pulseData.bpm = bpm_avg;
  // i2cData.tempData.rawData = 123.45;
  uint32_t pulse = 123;
  uint8_t bpm = 80U;
  float temp = 23.72;
  memcpy(data, (byte*)&temp, 4);
  data[4] = bpm;
  memcpy(&(data[5]), (byte*)&pulse, 4);
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

  updateData();
  delay(200);

  if (HIGH == digitalRead(switchPin)) // MASTER SEND MODE
  {
    digitalWrite(ledPin, HIGH);

    // pulseData_S pulse;
    // pulse.rawPulseData.rawData = 12345;
    // pulse.bpm = 80U;

    // tempData_S temp;
    // temp.rawData = 123.45;

    // i2cData_S i2c;
    // i2c.pulseData = pulse;
    // i2c.tempData = temp;

    // uint8_t data[sizeof(float)+sizeof(uint8_t)+sizeof(uint32_t)] = {0};
    // (float)(data[0]) = temp.rawData;
    // data[sizeof(float)] = pulse.bpm;
    // (uint32_t)(data[sizeof(float)+sizeof(uint8_t)]) = pulse.rawPulseData.rawData;

    Wire.beginTransmission(0x08);                 // starts transmit to device (8-Slave Arduino Address)
    // Wire.write((byte*)&i2c, sizeof(i2c));       // sends the stored values to Slave
    Wire.write(data, sizeof(data));       // sends the stored values to Slave
    Wire.endTransmission();                    // stop transmitting; should technically check if ret == 0
    // Press button to change led on slave
    // digitalWrite(ledPin, HIGH);
    // Wire.beginTransmission(0x08);                 // starts transmit to device (8-Slave Arduino Address)
    // Wire.write(state);                             // sends the value state to Slave
    // Wire.endTransmission();                    // stop transmitting
    // state = 1 - state;
  }
  else
  {
    digitalWrite(ledPin, LOW);

    // Wire.requestFrom(0x08,1);               // request  bytes from slave arduino(8)
    // byte slave_button = Wire.read();                // receive byte from the slave arduino and store in variable slave_button
    // if (slave_button==1)                            //Logic to turn Master LED ON (if received value is 1) or OFF (if received value is 0
    // { 
    //   digitalWrite(ledPin,HIGH);
    //   // Serial.println("Master LED ON");
    // }

    // else
    // {
    //   digitalWrite(ledPin,LOW);
    //   // Serial.println("Master LED OFF");
    // }
  }
  delay(100);
}