
#include <Wire.h>
#include "Adafruit_MCP9808.h"
#include <PulseSensorPlayground.h> // Includes the PulseSensorPlayground Library.
#include "include/flash_lib.h"

/*
 * Dont think we will end up needing this custom conf, currently disabled. 
 * Note the name was also changed to be _XXX in /include to make sure nothing conflicted bc this shit was dense to read. 
 * It's based on the conf that we see in stm based on the selected pins.
 * By default, it seems like stm32duino just enables everything for our specific board. 
*/
// #include "include/hal_conf_custom.h"

// Defines
#define peltierPin PA4
#define pwmPin PA5 // Green LED on Nucleo
#define ledPin PB13
#define buttonPin PA3
#define switchPin PA2
#define peltierPeriod 1000


// Peltier and User Inputs
bool peltierActive = false;
bool buttonPressed = false;
uint8_t peltierPWM = LOW;
uint8_t prevMode;


// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

// Pulse Sensor
const int PulseWire = 0;     // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
const int LED = LED_BUILTIN; // The on-board Arduino LED, close to PIN 13.
int Threshold = 550;         // Determine which Signal to "count as a beat" and which to ignore.
                             // Use the "Gettting Started Project" to fine-tune Threshold Value beyond default setting.
                             // Otherwise leave the default "550" value.

PulseSensorPlayground pulseSensor; // Creates an instance of the PulseSensorPlayground object called "pulseSensor"

// Moisture sensor
const int MoistureWire = 1;
const int MoistureRes = 10;
const int MoistureDelay = 10;


// Algo Setup
const int RESOLUTION = 200;
const float y_threshold = 30;

#define TEMP_THRESHOLD 20
// #define TEMP_THRESHOLD 10

#define TEMP_AVERAGE 22.00
#define MOIST_AVERAGE 1
#define PULSE_AVERAGE 0

float temperatures[RESOLUTION] = {};
int moistures[RESOLUTION] = {};
int pulses[RESOLUTION] = {};
unsigned long timestamps[RESOLUTION] = {};
int data_index = 0;

int w[3] = {8, 4, 1}; // Moisture, Temperature, Pulse

float m_avg = MOIST_AVERAGE;
float t_avg = TEMP_AVERAGE;
float prev_pulse_index = 0.0;

// bool first_pass = true;



// Button Interrupt Callback
void buttonISR() {
  if(HIGH == digitalRead(switchPin)) { // if in manual mode, do some stuff
    digitalWrite(ledPin, !digitalRead(ledPin));
    peltierActive = !peltierActive;
  }
  else { // What happens when user button is pressed when in auto mode.
    digitalWrite(peltierPin, LOW);
    digitalWrite(ledPin, LOW);
    // TODO: How do we learn from this event
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

  attachInterrupt(digitalPinToInterrupt(buttonPin),buttonISR,FALLING);



  // // Temperature sensor
  // Serial.begin(9600);
  // while (!Serial)
  //     ; // waits for serial terminal to be open, necessary in newer arduino boards.
  // Serial.println("Serial");

  // Temperature sensor address (default)
  if (!tempsensor.begin(0x18))
  {
      // Serial.println("Couldn't find MCP9808! Check your connections and verify the address is correct.");
      while (1) {
          digitalWrite(ledPin, !digitalRead(ledPin));
          delay(100);
      }
  }

  // Serial.println("Found MCP9808!");

  tempsensor.setResolution(3); // sets the resolution mode of reading, the modes are defined in the table bellow:
                                // Mode Resolution SampleTime
                                //  0    0.5°C       30 ms
                                //  1    0.25°C      65 ms
                                //  2    0.125°C     130 ms
                                //  3    0.0625°C    250 ms

  tempsensor.wake(); // wake up, ready to read!
  
  float t_read = tempsensor.readTempC();

  // Buffer setup
  for (int i = 0; i < RESOLUTION; i ++) {
    temperatures[i] = t_read;
    moistures[i] = MOIST_AVERAGE;
    pulses[i] = PULSE_AVERAGE;
  }
  // Pulse Sensor (temporarily disabled)
  // Configure the PulseSensor object, by assigning our variables to it.
  // pulseSensor.analogInput(PulseWire);
  // pulseSensor.blinkOnPulse(LED); // auto-magically blink Arduino's LED with heartbeat.
  // pulseSensor.setThreshold(Threshold);

  // // Double-check the "pulseSensor" object was created and "began" seeing a signal.
  // if (pulseSensor.begin())
  // {
  //     Serial.println("We created a pulseSensor Object !"); // This prints one time at Arduino power-up,  or on Arduino reset.
  // }
  digitalWrite(ledPin, HIGH);
  for (uint8_t i = 0; i < 6; i++) {
    digitalWrite(ledPin, !digitalRead(ledPin));
    delay(100);
  }
  digitalWrite(ledPin, LOW);
}

float prev_temp = 0.0;
bool printTemps = true;
unsigned long coolingStartTime = 0U;

void loop()
{
  // Read user inputs
  unsigned long currentMillis = millis();
  uint8_t pressEvent = digitalRead(buttonPin);
  uint8_t manualMode = digitalRead(switchPin);


  // ---------- DATA COLLECTION ----------

  // TEMPERATURE SENSOR
  // tempsensor.wake(); // wake up, ready to read!

  float t_read = tempsensor.readTempC();
  // delay(200);
  // tempsensor.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere, stops temperature sampling
  // delay(200);

  // PULSE SENSOR
  // if (pulseSensor.sawStartOfBeat())
  // {                                                 // Constantly test to see if "a beat happened".
  //     int myBPM = pulseSensor.getBeatsPerMinute();  // Calls function on our pulseSensor object that returns BPM as an "int".
  //                                                   // "myBPM" hold this BPM value now.
  //     Serial.println("♥  A HeartBeat Happened ! "); // If test is "true", print a message "a heartbeat happened".
  //     Serial.print("BPM: ");                        // Print phrase "BPM: "
  //     Serial.println(myBPM);                        // Print the value inside of myBPM.
  // }

  // INTERPRET SENSORS
  
  m_avg -= 1.0 * moistures[data_index] / RESOLUTION;
  t_avg -= temperatures[data_index] / RESOLUTION;

  float m_read = analogRead(MoistureWire) + 1.0;
  float p_read = analogRead(PulseWire); // to be Scaled

  m_avg += m_read / RESOLUTION;
  t_avg += t_read / RESOLUTION;

  moistures[data_index] = int(m_read);
  pulses[data_index] = p_read;
  temperatures[data_index] = t_read;
  timestamps[data_index] = millis();

  float m_score = 1.0 * (m_read - m_avg) / m_avg;
  float t_score = 100.0 * (t_read - t_avg) / t_avg;

  float y = m_score * w[0] + t_score * w[1] + 0 * w[2];
  
  // SERIAL OUTPUT

  // Serial.print("y:");
  // Serial.print(y);
  // Serial.print(", t score:");
  // Serial.print(t_score, 4);
  // Serial.print(", t avg:");
  // Serial.print(t_avg, 4);
  // Serial.print(", m avg:");
  // Serial.print(m_avg, 4);
  // Serial.print(", m score:");
  // Serial.print(m_score, 4);
  // Serial.print(", Temp:");
  // Serial.print(t_read, 4);
  // Serial.print(", Moisture:");
  // Serial.println(m_read, 4);

  data_index += 1;
  if (data_index >= RESOLUTION) {
    data_index = 0;
  }

  // State control
  if (manualMode != prevMode) { // Reset states when switching from manual to auto or vice versa
    digitalWrite(ledPin, LOW);
    digitalWrite(peltierPin, LOW);
    peltierActive = false;
  }

  if (LOW == manualMode) { // NOT in manual mode
    // TODO: AUTO MODE HERE
    // if (t_read != prev_temp) {
    //   prev_temp = t_read;
    //   for(uint8_t i = 0; i < 2; i++) {
    //       digitalWrite(ledPin, HIGH);
    //       delay(100);
    //       digitalWrite(ledPin, LOW);
    //       delay(100);
    //     }
    //     delay(500);
    // }

    // // INDICATE SCORE
    if (((data_index % RESOLUTION) == 0) && printTemps) {
      int tens = t_score / 10;
      int ones = t_score - tens * 10;
      for(uint8_t i = 0; i < tens; i++) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
      }
      delay(500);
      for(uint8_t i = 0; i < ones; i++) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
      }
      delay(500);
    } 
    
    if ((t_score > (TEMP_THRESHOLD)) && (coolingStartTime == 0U)) {
      digitalWrite(ledPin, HIGH);
      digitalWrite(peltierPin, HIGH);
      printTemps = false;
      coolingStartTime = millis();
    } 

    else if ((millis() - coolingStartTime) > 4000U) {
      if (t_score < (TEMP_THRESHOLD*0.8)) {
        coolingStartTime = 0U;
        digitalWrite(ledPin, LOW);
        digitalWrite(peltierPin, LOW);
        printTemps = true;
      }
    }
    // else if (t_score < (TEMP_THRESHOLD*0.65)) { 
    //   digitalWrite(ledPin, LOW);
    //   // digitalWrite(peltierPin, LOW);
    //   // printTemps = true;
    // }
  }
  else {
    if (peltierActive) {
    //   if ((currentMillis % peltierPeriod) <= 300) {
    //   // if ((currentMillis % peltierPeriod) <= (int)(peltierPeriod*0.5)) {
    //     peltierPWM = HIGH;
    //   } 
    //   else {
    //     peltierPWM = LOW;
    //   }
    // } 
    // else {
    //   peltierPWM = LOW;
    // }
    // // digitalWrite(ledPin, peltierPWM);
    // digitalWrite(peltierPin, peltierPWM);

      digitalWrite(peltierPin, HIGH);
      // delay(200);
      // digitalWrite(peltierPin, LOW);
      // delay(700);
    }
    else {
      digitalWrite(peltierPin, LOW);
    }
  }


  prevMode = manualMode;
  delay(10); // considered best practice in a simple sketch.
}
