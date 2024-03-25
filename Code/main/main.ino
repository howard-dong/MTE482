
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

// Debugging options - comment/uncomment - indicators use on board LED
// #define SERIAL
// #define SCORE_INDICATOR // blinks the algo score
#define PULSE_INDICATOR // lights up if pulse reading is over threshold
// #define DATA_CYCLE_INDICATOR // switches on/off on completed data collection cycle
// #define MOISTURE_INDICATOR // lights up if moisture reading is over threshold
// #define TEMPERATURE_INDICATOR // lights up if moisture reading is over threshold

// Write Pins
#define PELTIER_PIN PA4
#define LED_PIN PB13
#define BUTTON_PIN PA3
#define SWITCH_PIN PA2

// Peltier and User Inputs
bool peltierActive = false;
bool buttonPressed = false;
uint8_t peltierPWM = LOW;
uint8_t prevMode;

// I2C Package
#define DATA_LEN sizeof(float) + sizeof(uint8_t) + sizeof(uint32_t)
#define TRANSFER_PERIOD 500
uint8_t data[DATA_LEN] = { 0 };
uint32_t lastTransfer = 0U;

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

// Pulse Sensor
#define PULSE_PIN PA1

// Moisture sensor
#define MOISTURE_WIRE PA0
const int MOISTURE_RESOLUTION = 10;
const int MOISTURE_DELAY = 10;

// Algo Setup
const int RESOLUTION = 200;
const float THRESHOLD_Y = 30;

#define TEMP_THRESHOLD 20
// #define TEMP_THRESHOLD 10

#define TEMP_AVERAGE 22.00
#define PULSE_AVERAGE 0

float temperatures[RESOLUTION] = {};
int moistures[RESOLUTION] = {};
int pulses[RESOLUTION] = {};
unsigned long timestamps[RESOLUTION] = {};
int data_index = 0;

int w[3] = { 8, 4, 1 }; // Moisture, Temperature, Pulse

float m_avg = 0;
float t_avg = TEMP_AVERAGE;
float p_avg = 0;
float prev_pulse_index = 0;

// GLOBAL VARIABLES
float prev_temp = 0.0;
bool printTemps = true;
unsigned long coolingStartTime = 0U;

// Pulse Local Min/Max
const int LOCAL_PULSES = 10;
int max_pulses[LOCAL_PULSES] = {}; // Pulse indicies
int min_pulses[LOCAL_PULSES] = {}; // Pulse indicies
int min_pulse_index = 0;
int max_pulse_index = 0;
float max_p_avg = 0;
float min_p_avg = 0;

float getBPM() {
  int prev_index1 = data_index - 1;
  int prev_index2 = data_index - 2;
  if (data_index == 0) {
    prev_index1 = RESOLUTION - 1;
    prev_index2 = RESOLUTION - 2;
  }
  else if (data_index == 1) {
    prev_index2 = RESOLUTION - 1;
  }

  float left = pulses[prev_index2];
  float middle = pulses[prev_index1];
  float right = pulses[data_index];

  if ((left < middle) && (right < middle)) {
    max_p_avg += middle - pulses[max_pulses[max_pulse_index]];
    max_pulses[max_pulse_index] = prev_index1;
    max_pulse_index += 1;
    if (max_pulse_index >= LOCAL_PULSES) {
      max_pulse_index = 0;
    }
  }

  if ((left > middle) && (right > middle)) {
    min_p_avg += middle - pulses[min_pulses[min_pulse_index]];
    min_pulses[min_pulse_index] = prev_index1;
    min_pulse_index += 1;
    if (min_pulse_index >= LOCAL_PULSES) {
      min_pulse_index = 0;
    }
  }

  // Calculate BPM
  int last_max_index = max_pulse_index + 1;
  if (last_max_index == LOCAL_PULSES) {
    last_max_index == 0;
  }
  int max_time_first = max_pulses[last_max_index];
  int max_time_last = max_pulses[max_pulse_index];

  float bpm_max = (timestamps[max_time_last] - timestamps[max_time_first]) / LOCAL_PULSES / 2;

  int last_min_index = min_pulse_index + 1;
  if (last_min_index == LOCAL_PULSES) {
    last_min_index == 0;
  }
  int min_time_first = min_pulses[last_min_index];
  int min_time_last = min_pulses[min_pulse_index];

  float bpm_min = (timestamps[min_time_last] - timestamps[min_time_first]) / LOCAL_PULSES / 2;

  return (bpm_max + bpm_min) / 2;
}


void updateData(uint32_t pulse, uint8_t bpm, float temp)
{
  memcpy(data, (byte*)&temp, 4);
  data[4] = bpm;
  memcpy(&(data[5]), (byte*)&pulse, 4);

  Wire.beginTransmission(0x08);                 // starts transmit to device (8-Slave Arduino Address)
  Wire.write(data, sizeof(data));       // sends the stored values to Slave
  Wire.endTransmission();                    // stop transmitting; should technically check if ret == 0
}


// Button Interrupt Callback
void buttonISR() {
  if (HIGH == digitalRead(SWITCH_PIN)) { // if in manual mode, do some stuff
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    peltierActive = !peltierActive;
  }
  else { // What happens when user button is pressed when in auto mode.
    digitalWrite(PELTIER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    // TODO: How do we learn from this event
  }
}

void setup()
{
  // Pin Setups
  pinMode(PELTIER_PIN, OUTPUT);
  digitalWrite(PELTIER_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(SWITCH_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

#if SERIAL
  Serial.begin(9600);
  while (!Serial)
    ; // waits for serial terminal to be open, necessary in newer arduino boards.
  Serial.println("Serial");
#endif // SERIAL

  // Temperature sensor
  if (!tempsensor.begin(0x18))
  {
    while (1) { // Can't find MCP9808
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
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
  float m_read = analogRead(MOISTURE_WIRE);
  float p_read = analogRead(PULSE_PIN);

  // Buffer setup
  for (int i = 0; i < RESOLUTION; i++) {
    temperatures[i] = t_read;
    moistures[i] = m_read;
    pulses[i] = p_read;
  }

  // Finish Setup
  digitalWrite(LED_PIN, HIGH);
  for (uint8_t i = 0; i < 6; i++) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
  digitalWrite(LED_PIN, LOW);
}

void loop()
{
  // Read user inputs
  // TODO: Should we delete currentMillis? we dont have the peltier toggling logic rn
  unsigned long currentMillis = millis();
  uint8_t pressEvent = digitalRead(BUTTON_PIN);
  uint8_t manualMode = digitalRead(SWITCH_PIN);


  // ---------- DATA COLLECTION ----------

  // TEMPERATURE SENSOR
  // tempsensor.wake(); // wake up, ready to read!

  float t_read = tempsensor.readTempC();
  // delay(200);
  // tempsensor.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere, stops temperature sampling
  // delay(200);

  float m_read = analogRead(MOISTURE_WIRE) + 1.0;
  float p_read = analogRead(PULSE_PIN); // to be Scaled

  // PULSE SENSOR
  getBPM();

  // INTERPRET SENSORS
  m_avg += (m_read - moistures[data_index]) / RESOLUTION;
  t_avg += (t_read - temperatures[data_index]) / RESOLUTION;
  p_avg += (p_read - pulses[data_index]) / RESOLUTION;

  moistures[data_index] = int(m_read);
  pulses[data_index] = p_read;
  temperatures[data_index] = t_read;
  timestamps[data_index] = millis();

  float m_score = 1.0 * (m_read - m_avg);
  float t_score = 100.0 * (t_read - t_avg) / t_avg;

  float y = m_score * w[0] + t_score * w[1] + 0 * w[2];

#if SERIAL
  Serial.print("y:");
  Serial.print(y);
  Serial.print(", t score:");
  Serial.print(t_score, 4);
  Serial.print(", t avg:");
  Serial.print(t_avg, 4);
  Serial.print(", m avg:");
  Serial.print(m_avg, 4);
  Serial.print(", m score:");
  Serial.print(m_score, 4);
  Serial.print(", Temp:");
  Serial.print(t_read, 4);
  Serial.print(", Moisture:");
  Serial.println(m_read, 4);
#endif // SERIAL

  data_index += 1;
  if (data_index >= RESOLUTION) {

#ifdef DATA_CYCLE_INDICATOR
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
#endif

    data_index = 0;
  }

#ifdef MOISTURE_INDICATOR
  // if (m_score > 0) {
  //   digitalWrite(LED_PIN, HIGH);
  // } else if (m_score < 0) {
  //   digitalWrite(LED_PIN, LOW);
  // }

  int pow = 0;
  int num = 1;
  while (m_score > num) {
    pow += 1;
    num *= 10;
  }

  for (uint8_t i = 0; i < pow; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }

#endif

#ifdef PULSE_INDICATOR
  float z = 0.7;
  if (p_read > p_avg + z * (max_p_avg - p_avg)) {
    digitalWrite(LED_PIN, HIGH);
  }
  else {
    digitalWrite(LED_PIN, LOW);
  }
#endif

  // State control
  if (manualMode != prevMode) { // Reset states when switching from manual to auto or vice versa
    digitalWrite(LED_PIN, LOW);
    digitalWrite(PELTIER_PIN, LOW);
    peltierActive = false;
  }

  // PELTIER ON/OFF LOGIC
  if (LOW == manualMode) { // NOT in manual mode
    // TODO: AUTO MODE HERE
    // if (t_read != prev_temp) {
    //   prev_temp = t_read;
    //   for(uint8_t i = 0; i < 2; i++) {
    //       digitalWrite(LED_PIN, HIGH);
    //       delay(100);
    //       digitalWrite(LED_PIN, LOW);
    //       delay(100);
    //     }
    //     delay(500);
    // }

#if SCORE_INDICATOR
    // Blink tens/ones of two digit 
    if (((data_index % RESOLUTION) == 0) && printTemps) {
      int tens = t_score / 10;
      int ones = t_score - tens * 10;
      for (uint8_t i = 0; i < tens; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
      }
      delay(500);
      for (uint8_t i = 0; i < ones; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
      }
      delay(500);
    }
#endif // SCORE_INDICATOR

    if ((t_score > (TEMP_THRESHOLD)) && (coolingStartTime == 0U)) {
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(PELTIER_PIN, HIGH);
      printTemps = false;
      coolingStartTime = millis();
    }

    // wait for threshold to drop lower than 0.8 
    else if ((millis() - coolingStartTime) > 4000U) {
      if (t_score < (TEMP_THRESHOLD * 0.8)) {
        coolingStartTime = 0U;
        digitalWrite(LED_PIN, LOW);
        digitalWrite(PELTIER_PIN, LOW);
        printTemps = true;
      }
    }
    // else if (t_score < (TEMP_THRESHOLD*0.65)) { 
    //   digitalWrite(LED_PIN, LOW);
    //   // digitalWrite(PELTIER_PIN, LOW);
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
      // // digitalWrite(LED_PIN, peltierPWM);
      // digitalWrite(PELTIER_PIN, peltierPWM);

      digitalWrite(PELTIER_PIN, HIGH);
      // delay(200);
      // digitalWrite(PELTIER_PIN, LOW);
      // delay(700);
    }
    else {
      digitalWrite(PELTIER_PIN, LOW);
    }
  }


  prevMode = manualMode;

  if (millis() - lastTransfer > TRANSFER_PERIOD)
  {
    // TODO: Update 80 to be bpm
    // TODO: Update p_read to be last 500 ms of data so that it visualizes properly
    updateData(p_read, 80, t_read);
    lastTransfer = millis();
  }

  delay(100); // increase loop time for sensor calibration
}
