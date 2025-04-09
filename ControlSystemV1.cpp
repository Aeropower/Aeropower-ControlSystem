#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <ESP32PWM.h>
//

LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27 for the LCD

const int targetRPM = 45;
float integral = 0;
const float Kd = 0.01;         
const float Ki = 0.1;          
const float dt = 0.5;         
const float Kp = 0.5; 
float previousError = 0;

volatile unsigned long pulseCount = 0;
QueueHandle_t rpmQueue;
Servo blades;
int servoPin = 21;
int pushB = 13;
hw_timer_t * timer = NULL;


void IRAM_ATTR countPulse() {
    
    pulseCount += 10;
}

void IRAM_ATTR onTimer() {
    double rpm = 0;  // Default value to prevent errors
    if (pulseCount > 0) {
        rpm = (pulseCount * 60.0) / 5;  // Ensure divisor is nonzero
    }
    pulseCount = 0;
    xQueueSendFromISR(rpmQueue, &rpm, NULL);
    Serial.println("RPM SENT");
}

void displayRPMTask(void *pvParameters) {
    double receivedRPM;
    while (1) {
        if (xQueueReceive(rpmQueue, &receivedRPM, portMAX_DELAY)) {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("RPM: ");
            lcd.print(receivedRPM);
            Serial.print("RPM Displayed: ");
            Serial.println(receivedRPM);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void servoControl(float targetAngle) {
    float currentAngle = blades.read();
    int step;
    if(targetAngle > currentAngle) {
        step = 10;
    } else {
        step = -10;
    }
    while(currentAngle!= targetAngle){
        currentAngle += step;
        blades.write(currentAngle);
        delay(15);
    }
}

void PIDTask(void *pvParameters) {  
    double receivedRPM;
    
    while (1) {
        if (xQueueReceive(rpmQueue, &receivedRPM, portMAX_DELAY)) {
          
          Serial.println("RPM Received in PIDTask"); // Depuración
          float error = targetRPM - receivedRPM;
          float proportional = Kp * error;
          integral += Ki * error * dt;
          float derivative = Kd * (error - previousError) / dt;
          
          // Calculate the control output
          float output = proportional + integral + derivative;
          // Apply the control output to the servo
           Serial.print("PID Output: "); // Corrección: faltaba texto
          servoControl(output);
   
          previousError = error;

        }
        vTaskDelay(pdMS_TO_TICKS(dt * 1000));
  
}
}



void setup() {
    Serial.begin(921600);
    Serial.println("System starting...");
    lcd.init();
    lcd.init();
    lcd.backlight();
    lcd.print("Initializing...");

    blades.attach(21, 500, 2500);  // Fixed servo attach range
      // Set initial servo position

    pinMode(13, INPUT);
    attachInterrupt(digitalPinToInterrupt(13), countPulse, FALLING);

    rpmQueue = xQueueCreate(10, sizeof(double));
    if (rpmQueue == NULL) {
        Serial.println("Queue creation failed!");
        while(1); // Stop execution if queue creation fails
    }

    // Set timer frequency to 1Mhz
    timer = timerBegin(1000000);
    if (timer) {
     
      // Attach onTimer function to our timer.
      timerAttachInterrupt(timer, &onTimer);

      // Set alarm to call onTimer function every second (value in microseconds).
      // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
       timerAlarm(timer, 5000000, true, 0);
    } else {
        Serial.println("Timer initialization failed!");
    }
    
    xTaskCreate(PIDTask, "PID Calculation Task", 2048, NULL, 1, NULL);
    xTaskCreate(displayRPMTask, "Display RPM Task", 2048, NULL, 1, NULL);
}

void loop() {
    // Empty loop since tasks handle execution
}
