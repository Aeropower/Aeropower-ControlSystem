#include <LiquidCrystal.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <ESP32PWM.h>
#include "BluetoothSerial.h"
#include <QuickPID.h
//^^^^^^^^^^^^^^^^^^^^^^^^^
//|||||||||||||||||||||||||
//For information of any function used in this code that is part of the above libraries feel free to search them in the web
/*------------------------------------------------------------------------------
                                DATA AND INTERRUPTS
------------------------------------------------------------------------------*/
// Constants
Servo blades;                           // Servo motor for blade pitch control
volatile unsigned long pulseCount = 0;  // Pulse counter for RPM
volatile bool calculate = false;        // Flag to calculate 
double rpm = 0;                         // Current RPM
float inputVoltage;                     // Initial voltage
float output;                           // Output of the PID, this is a number that isnt neither an angle nor voltage, just a number that comes from 
                                        // substracting setPoint - inputVoltage and that number then maps to an angle
int pitchAngle = 0;
float setPoint = 47.0;                  // Voltage setpoint
BluetoothSerial bluetooth;              // Bluetooth object
float kp, kd, ki;                       // This should be tune with help of the mechanical division, Kp = Proportional gain, Kd = derivative gain, Kp = integral gain
int magnetsPerRevolution = 1;           // Quantity of magnets in the shaft(I think)
float timeWindow = 10;                  // Quanity of time it will take to refresh RPM, pitch control, etc.
const float factor_conversion = 0.0716; // Factor conversion of output voltage that will be the PID input voltage
const int voltagePin = 7;               // Voltage pin

// Initialize the library by associating any needed LCD interface pin
// with the Arduino pin number it is connected to
// This is an example tho
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//PID constructor
QuickPID myPID(&inputVoltage, &output, &setPoint);

// Timer handle pointer
hw_timer_t *timer = NULL;

// ISR function for counting pulses
//Everytime the IR sensor causes an interrupt this function will be called
void IRAM_ATTR countPulse() {
  Serial.println("Interrupt");
  pulseCount++;
}

// Timer interrupt service routine
//Everytime the Timer causes an interrupt this function will be called
void IRAM_ATTR onTimer() {
  Serial.println("Timer");
  calculate = true;
}
/*------------------------------------------------------------------------------
                                FUNCTIONS
------------------------------------------------------------------------------*/
// Function to calculate RPM
double RPMCounter() {
  rpm = (pulseCount * 60.0) / (timeWindow*magnetsPerRevolution);  // Convert pulses to RPM for 10-second intervals
  Serial.print("RPM: ");
  Serial.println(rpm);
  return rpm;
}

//Emergency stop function
void emergencyStop(){
  while(pitchAngle!=0){
    pitchAngle-=5;
    blades.write(pitchAngle);
  }
}

//Liquid Crystal Display(LCD) print function
void printLCD(int angle, float inputVoltage, float rpm){
    lcd.setCursor(0,0);
    lcd.print("RPM: ");
    lcd.print(rpm);
    lcd.setCursor(0,1);
    lcd.print("Voltage: ");
    lcd.print(inputVoltage);
    lcd.setCursor(1,0);
    lcd.print("Angle: ");
    lcd.print(angle);
}
/*------------------------------------------------------------------------------
                                SETUP
------------------------------------------------------------------------------*/
void setup() {
  Serial.begin(921600);
  Serial.println("System starting...");

  //Pins
  pinMode(9, OUTPUT); // Servo pin
  pinMode(7,INPUT);   //VoltageInput
  pinMode(2, INPUT);  // Hall effect sensor input pin
  attachInterrupt(digitalPinToInterrupt(2), countPulse, FALLING);
  
  //Servo setup
  blades.attach(9, 600, 2300);  // Attach servo to pin 9 with min/max pulse widths
  blades.write(pitchAngle);     // Set initial angle

  //PID setup
  myPID.setTunings(kp, ki, kd);
  // Set output limits (0 to 45 degrees)
  myPID.SetOutputLimits(0, 45);
  myPID.setMode(myPID.Control::automatic);

  // Set up the LCD's number of columns and rows: (This is an 16*2 LCD)
  lcd.begin(16, 2);

  //Timer
  timer = timerBegin(1000000);
  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  timerAlarm(timer, 1000000, true, 0);
  // Timer 0, prescaler de 80 para frecuencia de 1 MHz (1 tick = 1 µs)
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 10000000, true, 0);

  //Bluetooth setup
  bluetooth.begin("ESP32_Aeropower");
  Serial.println("Bluetooth activated");
}
/*------------------------------------------------------------------------------
                                LOOP
------------------------------------------------------------------------------*/
void loop() {

  if (calculate) {
    detachInterrupt(digitalPinToInterrupt(2));  // Disable interrupts temporarily
    inputVoltage = analogRead(voltagePin) * factor_conversion; //PID input
    myPID.Compute(); // Compute PID
    pitchAngle = (int)output;
    blades.write(pitchAngle);                                        // Move blades to the correct angle
    Serial.print("Blade Angle: ");
    Serial.println(pitchAngle);
    pulseCount = 0;                                                  // Reset pulse count
    calculate = false;                                               // Reset flag
    attachInterrupt(digitalPinToInterrupt(2), countPulse, FALLING);  // Re-enable interrupts
    printLCD(pitchAngle, inputVoltage, rpmCounter());

  }

  if(bluetooth.available()){
    String message = bluetooth.readString();
    if(message.equals("STOP")){               //If anyone types STOP to the bluetooth, emergency function is called.
      emergencyStop();
    }
  }
}
