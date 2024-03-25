//I2C Slave Code (Arduino)
//I2C Communication between STM32 and Arduino
//Circuit Digest

#include<Wire.h>                             //Library for I2C Communication functions

#define DATA_LEN sizeof(float) + sizeof(uint8_t) + sizeof(uint32_t)

uint8_t data[DATA_LEN] = {0};

void setup() 
{
  Serial.begin(9600);                        //Begins Serial Communication at 9600 baud rate
  Serial.println("Serial On\n");

  Wire.begin(0x08);                             // join i2c bus with its slave Address as 8 at pin (A4,A5)
  Wire.onReceive(receiveEvent);              //Function call when Slave Arduino receives value from master STM32
}



void loop() 
{
  delay(10);
}


void receiveEvent(int howMany)              //This Function is called when Slave Arduino receives value from master STM32
{
  uint8_t i = 0;

  while(Wire.available())
  {
    data[i] = Wire.read();
    i++;
  }

  uint32_t pulse = 0.0;
  uint8_t bpm = 0U;
  float temp = 0U;
  memcpy((byte*)&temp, data, 4);
  bpm = data[4];
  memcpy((byte*)&pulse, &(data[5]), 4);  

  // Serial.println("Data from Master:");

  // Serial.println(pulse);
  // Serial.println(bpm);
  // Serial.println(temp);
  Serial.print(temp);
  Serial.print(" ");
  Serial.println(bpm);

  delay(200);
}
