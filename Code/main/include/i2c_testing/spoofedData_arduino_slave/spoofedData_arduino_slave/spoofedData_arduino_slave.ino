//I2C Slave Code (Arduino)
//I2C Communication between STM32 and Arduino
//Circuit Digest

#include<Wire.h>                             //Library for I2C Communication functions
#define LED_PIN 7
#define buttonpin 2

#define DATA_LEN sizeof(float) + sizeof(uint8_t) + sizeof(uint32_t)

byte state =0;
int button_pressed = 0;

uint8_t data[DATA_LEN] = {0};

// union tempData_S
// {
//   float rawData;
//   byte  byteData[sizeof(float)] = {0};
// };

// union rawPulseData_S
// {
//   uint32_t rawData;
//   byte byteData[sizeof(uint32_t)] = {0};
// };

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
//   pulseData_S         pulseData;
//   tempData_S          tempData;
// } i2cData;



void setup() 
{
  Serial.begin(9600);                        //Begins Serial Communication at 9600 baud rate
  Serial.println("Serial On\n");
  pinMode(LED_PIN,OUTPUT);                       //Sets pin 7 as output

  Wire.begin(0x08);                             // join i2c bus with its slave Address as 8 at pin (A4,A5)
  Wire.onReceive(receiveEvent);              //Function call when Slave Arduino receives value from master STM32
  // Wire.onRequest(requestEvent);              //Function call when Master STM32 request value from Slave Arduino
}



void loop() 
{
  // int value = digitalRead(buttonpin);          //Reads the status of the pin 2   
  // if (value == LOW)                           //Logic to set the value of state to send to master depending upon input at pin 2
  // {
  //   button_pressed = 1;
  //   Serial.println("Button Pressed");
  // }
  // else 
  // {
  //   button_pressed = 0;
  //   Serial.println("Button Not Pressed");
  // }
  delay(10);
}


void receiveEvent(int howMany)              //This Function is called when Slave Arduino receives value from master STM32
{
  uint8_t i = 0;
  // pulseData_S pulse;
  // pulse.rawPulseData.rawData = 0U;
  // pulse.bpm = 0U;

  // tempData_S temp;
  // temp.rawData = 0.0;

  // i2cData_S i2c;
  // i2c.pulseData = pulse;
  // i2c.tempData = temp;

  while(Wire.available())
  // if(Wire.available())
  {
    data[i] = Wire.read();
    // ((uint8_t*)&i2c)[i] = Wire.read();
    i++;

    // Wire.readBytes((byte*)&i2c, sizeof(i2cData_S));



    // Wire.readBytes((byte*)&i2cData, sizeof(i2cData_S));
    // Serial.println("Data from Master=");
    // Serial.println(i2cData.pulseData.rawPulseData.rawData);
    // Serial.println(i2cData.pulseData.bpm);
    // Serial.println(i2cData.tempData.rawData);
    // byte master_button_state = Wire.read();                      //Used to read value received from master STM32 and store in variable master_button_state
    // if (master_button_state == 1)                                //Logic to turn Slave LED_PIN ON (if received value is 1) or OFF (if received value is 0)
    // {
    //   digitalWrite(LED_PIN,HIGH);
    //   Serial.println("Slave LED_PIN ON");
    // }
    // else
    // {
    //   digitalWrite(LED_PIN,LOW);
    //   Serial.println("Slave LED_PIN OFF");
    // }
  }

  // uint32_t pulse = 123.45;
  // uint8_t bpm = 80U;
  // float temp = 20U;
  uint32_t pulse = 0.0;
  uint8_t bpm = 0U;
  float temp = 0U;
  memcpy((byte*)&temp, data, 4);
  bpm = data[4];
  memcpy((byte*)&pulse, &(data[5]), 4);  

  Serial.println("Data from Master:");
  // Serial.println(pulse.rawData);
  // Serial.println(temp.rawData);
  Serial.println(pulse);
  Serial.println(bpm);
  Serial.println(temp);


  // Serial.println(i2cData.pulseData.rawPulseData.rawData);
  // Serial.println(i2cData.pulseData.bpm);
  // Serial.println(i2cData.tempData.rawData);
  delay(200);
}


// void requestEvent()                            //This Function is called when Master STM32 wants value from slave Arduino
// {
//   int value = digitalRead(buttonpin);          //Reads the status of the pin 2   
//   if (value == LOW)                           //Logic to set the value of state to send to master depending upon input at pin 2
//   {
//     button_pressed = 1;
//     Serial.println("Button Pressed");
//   }
//   else 
//   {
//     button_pressed = 0;
//     Serial.println("Button Not Pressed");
//   }
//   Wire.write(button_pressed);                             // sends one byte of state value to master STM32
// }
