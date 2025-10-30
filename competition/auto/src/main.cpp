#include <Arduino.h>

#include "gyro_tracker.h"

const byte BUTTON_PIN = 10;

GyroTracker tracker;

void panic() {
    while (true) {
        Serial.println("Panic!!!");
        delay(100);
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    delay(100);

    // Wait for the button press to start the gyro calibration
    while (digitalRead(BUTTON_PIN)) {
        delay(100);
    }
    delay(500);

    if (!tracker.begin()) {
        Serial.println("ERROR: BMI160 initialization failed! Check wiring.");
        panic();
    }

    // IMPORTANT: Keep the device perfectly still during this process.
    tracker.calibrate(500);  // Calibrate using 500 samples
    Serial.print("Yaw tracking started. Offset used (LSB): ");
    Serial.println(tracker.getOffset());

    // Wait for the button press to start the robot
    while (digitalRead(BUTTON_PIN)) {
        delay(100);
    }
    delay(500);
}

void loop() {
    tracker.update();
    float currentYaw = tracker.getYaw();

    Serial.print("Yaw: ");
    Serial.println(currentYaw, 2);

    delay(5);
}
