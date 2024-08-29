int sensorPin = 13;
int pulseCount = 0;
float RPM = 0;
int lastSensorValue = 1;
void setup() {
  pinMode(sensorPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  unsigned long startTime = millis();
  pulseCount = 0;
  

  while (millis() - startTime < 3000) { // Measure RPM over 3 second
    int sensorValue = digitalRead(sensorPin); // Read sensor state (HIGH or LOW) (if obstacle then 0, if clear than 1)
    if (sensorValue == 0 && lastSensorValue == 1) {
      pulseCount+=lastSensorValue; // Count each pulse
      //Serial.print(pulseCount);
    }
      lastSensorValue = sensorValue;
      delay (1);
  }

  RPM = (pulseCount/3.0 * 60.0);
  Serial.print("RPM: ");
  Serial.println(RPM);
  Serial.print("Pulse Count: ");
  Serial.println(pulseCount);
}




/* int sensorPin = 13; // Replace with the actual pin number where your sensor is connected
int pulseCount = 0;
int RPM = 0;

void setup() {
  pinMode(sensorPin, INPUT);
  Serial.begin(9600); // Initialize serial communication for debugging (optional)
}

void loop() {
  // Detect a pulse (LOW to HIGH transition)
  int sensorValue = digitalRead(sensorPin);
  currenttime = millis ();
  if (sensorValue == LOW) {
    pulseCount++; // Increment pulse count
  }

  // Measure RPM after every complete rotation (two pulses)
  while (currenttime < 2000) {
    if (pulseCount >=1) {
      RPM = (pulseCount) * 60; // Calculate RPM
      pulseCount = 0; // Reset pulse count

      // Output the RPM value
      Serial.print("RPM: ");
      Serial.println(RPM);
    }
  }
}

