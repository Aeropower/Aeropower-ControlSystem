#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <LiquidCrystal.h>
#include <ESP32Servo.h>
#include <ESP32PWM.h>
//
volatile unsigned long pulseCount = 0;
QueueHandle_t rpmQueue;
Servo blades;
int servoPin = 21;
int pushB = 13;
hw_timer_t * timer = NULL;
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void IRAM_ATTR countPulse() {
    Serial.println("Interrupt");
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

void servoTask(void *pvParameters) {
    int angle = 0;
    while (1) {
        for (angle = 0; angle <= 180; angle += 10) {
            blades.write(angle);
            vTaskDelay(pdMS_TO_TICKS(500));  // Move in steps of 10 degrees every 500ms
        }
        for (angle = 180; angle >= 0; angle -= 10) {
            blades.write(angle);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

void setup() {
    Serial.begin(921600);
    Serial.println("System starting...");
    lcd.begin(16, 2);
    lcd.print("Initializing...");

    blades.attach(21, 500, 2500);  // Fixed servo attach range
    blades.write(90);  // Set initial servo position

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

    xTaskCreate(displayRPMTask, "Display RPM Task", 2048, NULL, 1, NULL);
    xTaskCreate(servoTask, "Servo Task", 2048, NULL, 1, NULL);  // Updated servo task
}

void loop() {
    // Empty loop since tasks handle execution
}
