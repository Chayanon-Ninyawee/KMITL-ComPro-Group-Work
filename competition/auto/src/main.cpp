#include <Arduino.h>

#include "gyro_tracker.h"

GyroTracker tracker;

void panic() {
    while (true) {
        Serial.println("Panic!!!");
        delay(100);
    }
}

void setup() {
    Serial.begin(115200);

    // while (!Serial) {
    //     delay(100);
    // }

    if (!tracker.begin()) {
        Serial.println("ERROR: BMI160 initialization failed! Check wiring.");
        panic();
    }

    // IMPORTANT: Keep the device perfectly still during this process.
    tracker.calibrate(500);  // Calibrate using 500 samples
    Serial.print("Yaw tracking started. Offset used (LSB): ");
    Serial.println(tracker.getOffset());
}

void loop() {
    tracker.update();
    float currentYaw = tracker.getYaw();

    Serial.print("Yaw: ");
    Serial.println(currentYaw, 2);

    delay(5);
}
