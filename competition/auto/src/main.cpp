#include <Arduino.h>

#include <HCSR04.h>
#include <Servo.h>

#include "gyro_tracker.h"

constexpr byte BUTTON_PIN = 10;

constexpr byte SERVO_PIN = A0;
constexpr double SERVO_MIN = 20;
constexpr double SERVO_MAX = 140;

const byte TRIGGER_PIN = 11;
const byte ECHO_COUNT = 1;
byte *ECHO_PINS = new byte[ECHO_COUNT]{12};

GyroTracker tracker;

Servo armServo;

void panic() {
    while (true) {
        Serial.println("Panic!!!");
        delay(100);
    }
}

void setup() {
    Serial.begin(115200);

    armServo.attach(SERVO_PIN);
    HCSR04.begin(TRIGGER_PIN, ECHO_PINS, ECHO_COUNT);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    delay(100);

    armServo.write(SERVO_MAX);

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

    armServo.write(SERVO_MIN);

    // double *distances = HCSR04.measureDistanceCm();

    // for (int i = 0; i < ECHO_COUNT; i++) {
    //     if (i > 0) Serial.print(" | ");
    //     Serial.print(i + 1);
    //     Serial.print(": ");
    //     Serial.print(distances[i]);
    //     Serial.print(" cm");
    // }

    // Serial.println("");

    delay(5);
}
