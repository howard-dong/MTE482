//I2C Slave Code (Arduino)
//I2C Communication between STM32 and Arduino
//Circuit Digest

#include<Wire.h>                             //Library for I2C Communication functions
#define LED_PIN 7
#define buttonpin 2

byte state =0;
int button_pressed = 0;

void setup() 
{
  Serial.begin(9600);                        //Begins Serial Communication at 9600 baud rate
  Serial.println("Serial On\n");
  pinMode(LED_PIN,OUTPUT);                       //Sets pin 7 as output

  Wire.begin(0x08);                             // join i2c bus with its slave Address as 8 at pin (A4,A5)
  Wire.onReceive(receiveEvent);              //Function call when Slave Arduino receives value from master STM32
  Wire.onRequest(requestEvent);              //Function call when Master STM32 request value from Slave Arduino
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
  uint8_t data[4] = {0};
  uint8_t i = 0;
  while(Wire.available())
  {
    data[i] = Wire.read();
    i++;
  }

  Serial.println("Data from Master:");
  for (i = 0; i < 4; i++)
  {
    Serial.println(data[i]);
  }

  // if(Wire.available())
  // {
  //   uint8_t data[4] = {0};
  //   Wire.readBytes((byte*)&data, sizeof(data));
  //   Serial.println("Data from Master=");
  //   for (int i = 0; i < 4; i++)
  //   {
  //     Serial.println(data[i]);      
  //   }
  // }

  // byte master_button_state = Wire.read();                      //Used to read value received from master STM32 and store in variable master_button_state
  // if (master_button_state == 1)                                //Logic to turn Slave LED_PIN ON (if received value is 1) or OFF (if received value is 0)
  // {
  //  digitalWrite(LED_PIN,HIGH);
  //  Serial.println("Slave LED_PIN ON");
  // }
  // else
  // {
  //   digitalWrite(LED_PIN,LOW);
  //   Serial.println("Slave LED_PIN OFF");
  // }
  delay(100);
}


void requestEvent()                            //This Function is called when Master STM32 wants value from slave Arduino
{
  int value = digitalRead(buttonpin);          //Reads the status of the pin 2   
  if (value == LOW)                           //Logic to set the value of state to send to master depending upon input at pin 2
  {
    button_pressed = 1;
    Serial.println("Button Pressed");
  }
  else 
  {
    button_pressed = 0;
    Serial.println("Button Not Pressed");
  }
  Wire.write(button_pressed);                             // sends one byte of state value to master STM32
}
