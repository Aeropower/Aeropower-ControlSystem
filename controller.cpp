#include <Servo.h>

/* This code is created by Javier, Orlando, and Yamil to simulate the operation of the pitch control in a wind turbine */
// Last update: April 11, 2024 | 12:44 PM

// Constants:
const int Preset[6] = { 20, 0, 90, 0, 5000, 0 };  // {0 = Max Wind|1 = Min Wind|2 = Max Angle|3 = Min Angle|4 = Max RPM|5 = Min RPM}

// Variables:

// Inputs:
int An_Sensor = A0;      // Anemometer (Analog value)
int TM_Sensor = 1;       // Tachometer (Digital value)
int V_Sensor = 2;        // Voltage sensor (Digital value)
int C_Sensor = 3;        // Current sensor (Digital value)
int S_Sensor_Gate = 4;   // Sound sensor (Digital Value)
int S_Sensor_Enve = A1;  // Sound sensor (Analog Value)

// Outputs:
Servo Blades;  // Servo motor for controlling the pitch angle of the blades

unsigned long previousMillis = 0;  // Store the last time in milliseconds
unsigned long counter = 0;         // Counter for RPM calculation
double rpm = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("System starting...");
  Serial.println("Configuring Inputs and Outputs...");
  pinMode(An_Sensor, INPUT);
  pinMode(TM_Sensor, INPUT);
  pinMode(V_Sensor, INPUT);
  pinMode(C_Sensor, INPUT);
  pinMode(S_Sensor_Gate, INPUT);
  pinMode(S_Sensor_Enve, INPUT);
  Blades.attach(9);

  Serial.println("Testing the Outputs...");
  Blades.write(0);
  delay(1000);
  Blades.write(90);
  delay(1000);

  Serial.println("Main code running...");
}

void loop() {
  int W_Speed, V_Detected, E_Sound, G_Sound, P_Angle;

  W_Speed = WindSpeed();         // Measure the wind speed
  E_Sound = Sound_Envelope();    // Measure the sound level
  G_Sound = Sound_Gate();        // Measure the sound level
  V_Detected = VoltageDetect();  // Detect the max output voltage

  // Change the wind speed to pitch angle:
  P_Angle = map(W_Speed, Preset[1], Preset[0], Preset[3], Preset[2]);
  Serial.print(" Pitch Angle: ");
  Serial.println(P_Angle);

  if (G_Sound == 1 || E_Sound > 100) {
    Blades.write(90);
  } else {
    Blades.write(0);
  }

  unsigned long takenTime = millisToSeconds(time());
  if (takenTime >= 60) {  // Only calculate RPM every 60 seconds
    RPMCounter();          // Calculate RPM
    previousMillis = millis();  // Reset the timer
    counter = 0;                // Reset the counter for the next minute
  }
}

int WindSpeed() {
  int A_Val, WindSpeed;
  A_Val = analogRead(An_Sensor);
  WindSpeed = map(A_Val, 0, 1023, Preset[1], Preset[0]);
  Serial.print("Wind Speed: ");
  Serial.print(WindSpeed);

  return WindSpeed;
}

int Vibration() {  // This function is for reading the value of the accelerometer
  // Implement your vibration reading code here
}

int Sound_Gate() {  // This function is for reading the value of the digital output of the sound sensor
  int D_Val = digitalRead(S_Sensor_Gate);
  Serial.print(" Sound Level Gate: ");
  Serial.print(D_Val);
  return D_Val;
}

int Sound_Envelope() {  // This function is for reading the value of the analog output of the sound sensor
  int A_Val = analogRead(S_Sensor_Enve);
  Serial.print(" Sound Level Envelope: ");
  Serial.print(A_Val);
  return A_Val;
}

void RPMCounter() {  // This function reads the revolutions of the blade and converts them into RPM
  rpm = counter*60; // Since counter is incremented per revolution per second
  Serial.print("RPM: ");
  Serial.println(rpm);
}

unsigned long time() {
  unsigned long currentMillis = millis();
  return currentMillis - previousMillis;  // Calculate time elapsed since last measurement
}

int VoltageDetect() {
  return digitalRead(V_Sensor);
}

int CurrentDetect() {
  // Implement your current detection code here
}

unsigned long millisToSeconds(unsigned long millisTime) {
   if (millisTime == 0) {
    return 0;
  }
  return millisTime / 1000;
}
