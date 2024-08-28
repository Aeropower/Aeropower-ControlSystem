#include <Servo.h>



//Constants:
const int Preset[6] = { 20, 0, 90, 0, 5000, 0 };  //{0 = Max Wind|1 = Min Wind|2 = Max Angle|3 = Min Angle|4 = Max RPM|5 = Min RPM}

//Variables:

//Inputs:
int An_Sensor = A0;      //Anenometer (Analog value)
int TM_Sensor = 1;       //Tacometer (Digital value)
int V_Sensor = 2;        //Voltage sensor (Digital value)
int C_Sensor = 3;        //Current sensor (Digital value)
int S_Sensor_Gate = 4;   //Sound sensor (Digital Value)
int S_Sensor_Enve = A1;  //Sound sensor (Analog Value)

//Outputs:
Servo Blades;  //Servo motor for control the pitch angle of the blades



void setup() {
  // put your setup code here, to run once:
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

  W_Speed = WindSpeed();         //measure the wind speed
  E_Sound = Sound_Envelope();    //measure the sound level
  G_Sound = Sound_Gate();        //measure the sound level
  V_Detected = VoltageDetect();  //Detect the max output voltage

  //Change the wind speed to pitch angle:
  P_Angle = map(W_Speed, Preset[1], Preset[0], Preset[3], Preset[2]);
  Serial.print(" Pitch Angle: ");
  Serial.println(P_Angle);


  if (G_Sound == 1 || E_Sound > 100) {
    Blades.write(90);
  } else {
    Blades.write(0);
  }

  delay(1000);
}

int WindSpeed() {
  //This funtion is for read and convert the analog voltage of the sensor into wind speed (m/s)
  int A_Val, WindSpeed;
  A_Val = analogRead(An_Sensor);
  WindSpeed = map(A_Val, 0, 1023, Preset[1], Preset[0]);
  Serial.print("Wind Speed: ");
  Serial.print(WindSpeed);

  return WindSpeed;
}
int Vibration() {  // This function is for read the value of the accelerometer
}
int Sound_Gate() {  //This function is for read the value of the digital output of the sound sensor
  int D_Val, G_Sound;
  D_Val = digitalRead(S_Sensor_Gate);
  Serial.print(" Sound Level Gate: ");
  Serial.print(D_Val);
  G_Sound = D_Val;
  return G_Sound;
}
int Sound_Envelope() {  // This function is for read the value of the analog output of the sound sensor

  int A_Val, E_Sound;
  A_Val = analogRead(S_Sensor_Enve);
  //E_Sound = map(A_Val,0,1023,Preset[1],Preset[0]);

  Serial.print(" Sound Level Envelope: ");
  Serial.print(A_Val);
  E_Sound = A_Val;
  return E_Sound;
}
int RPMCounter() {  // This function is read the revolutions of the blade and convert in RPM
}

int VoltageDetect() {
  int VoltDetected;
  VoltDetected = digitalRead(V_Sensor);
  return VoltDetected;
}
int CurrentDetect() {
}
