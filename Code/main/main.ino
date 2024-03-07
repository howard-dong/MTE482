#include <Wire.h>
#include "Adafruit_MCP9808.h"
#include <PulseSensorPlayground.h> // Includes the PulseSensorPlayground Library.

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
const int PeltierPin = 3;


void setup()
{
    // Temperature sensor
    Serial.begin(9600);
    while (!Serial)
        ; // waits for serial terminal to be open, necessary in newer arduino boards.
    Serial.println("Serial");

    // Temperature sensor address (default)
    if (!tempsensor.begin(0x18))
    {
        Serial.println("Couldn't find MCP9808! Check your connections and verify the address is correct.");
        while (1)
            ;
    }

    // Serial.println("Found MCP9808!");

    tempsensor.setResolution(3); // sets the resolution mode of reading, the modes are defined in the table bellow:
                                 // Mode Resolution SampleTime
                                 //  0    0.5°C       30 ms
                                 //  1    0.25°C      65 ms
                                 //  2    0.125°C     130 ms
                                 //  3    0.0625°C    250 ms

    // Pulse Sensor
    // Configure the PulseSensor object, by assigning our variables to it.
    pulseSensor.analogInput(PulseWire);
    pulseSensor.blinkOnPulse(LED); // auto-magically blink Arduino's LED with heartbeat.
    pulseSensor.setThreshold(Threshold);

    // Double-check the "pulseSensor" object was created and "began" seeing a signal.
    if (pulseSensor.begin())
    {
        Serial.println("We created a pulseSensor Object !"); // This prints one time at Arduino power-up,  or on Arduino reset.
    }

    analogWrite(PeltierPin, 200);
}

void loop()
{
    // ---------- DATA COLLECTION ----------

    // TEMPERATURE SENSOR
    tempsensor.wake(); // wake up, ready to read!

    float c = tempsensor.readTempC();
    // delay(200);
    tempsensor.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere, stops temperature sampling
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

    // MOISTURE SENSOR
    int sum = 0;
    for (int i = 0; i < MoistureRes; i++)
    {
        int a = analogRead(MoistureWire);
        sum += a;
        delay(MoistureDelay);
    }

    // SERIAL OUTPUT
    // Serial.print("Temp:");
    // Serial.print(c, 4);
    // Serial.print(",");
    // Serial.print("Moisture:");
    // Serial.println(sum / MoistureRes);

    Serial.println(analogRead(2));


    delay(20); // considered best practice in a simple sketch.
}
