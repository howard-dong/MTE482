#define ledPin PB13

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  
  Serial.begin();
  while(!Serial) {
    digitalWrite(ledPin, !digitalRead(ledPin));
    delay(100);
  }
  
  Serial.println("you are connected!");
}

void loop() {
  // if a character is available, echo to output
  if (Serial)
  {
    Serial.write(Serial.read());
  }
}