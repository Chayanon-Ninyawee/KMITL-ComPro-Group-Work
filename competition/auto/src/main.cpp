#include <Arduino.h>

#include <HCSR04.h>
#include <Servo.h>

#include "gyro_tracker.h"
#include "motor_controller.h"

constexpr byte BUTTON_PIN = 10;

constexpr byte SERVO_PIN = A0;
constexpr double SERVO_MIN = 20;
constexpr double SERVO_MAX = 140;

constexpr byte MOTOR_LEFT_1 = 7;
constexpr byte MOTOR_LEFT_2 = 8;
constexpr byte MOTOR_LEFT_EN = 6;

constexpr byte MOTOR_RIGHT_1 = 4;
constexpr byte MOTOR_RIGHT_2 = 5;
constexpr byte MOTOR_RIGHT_EN = 3;

const byte TRIGGER_PIN = 11;
const byte ECHO_COUNT = 1;
byte *ECHO_PINS = new byte[ECHO_COUNT]{12};

GyroTracker tracker;
Servo armServo;
MotorController robotMotors(MOTOR_LEFT_1, MOTOR_LEFT_2, MOTOR_LEFT_EN, MOTOR_RIGHT_1, MOTOR_RIGHT_2, MOTOR_RIGHT_EN);

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
    robotMotors.begin();

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    delay(100);

    armServo.write(SERVO_MAX);

    // Wait for the button press to start the gyro calibration
    while (digitalRead(BUTTON_PIN)) {
        delay(100);
    }
    delay(500);

    Serial.println("1");
    if (!tracker.begin()) {
        Serial.println("ERROR: BMI160 initialization failed! Check wiring.");
        panic();
    }

    Serial.println("Yaw tracking starting...");
    // IMPORTANT: Keep the device perfectly still during this process.
    tracker.calibrate(500);  // Calibrate using 500 samples
    Serial.print("Yaw tracking started. Offset used (LSB): ");
    Serial.println(tracker.getOffset());

    // Wait for the button press to start the robot
    while (digitalRead(BUTTON_PIN)) {
        delay(100);
    }
    armServo.write(SERVO_MIN);
    delay(500);
}

bool forwardPhaseComplete = false;

// Timing and Angular variables
unsigned long forwardStartTime = 0;

// Constants for movement
const int BASE_SPEED = 200;
float Kp = 10.0;

const long FORWARD_DURATION_MS = 2000;
float targetYaw = 0.0;

void loop() {
    tracker.update();
    float currentYaw = tracker.getYaw();

    Serial.print("Yaw: ");
    Serial.println(currentYaw, 2);  // Print to 2 decimal places

    // double *distances = HCSR04.measureDistanceCm();

    // for (int i = 0; i < ECHO_COUNT; i++) {
    //     if (i > 0) Serial.print(" | ");
    //     Serial.print(i + 1);
    //     Serial.print(": ");
    //     Serial.print(distances[i]);
    //     Serial.print(" cm");
    // }

    // Serial.println("");

    if (!forwardPhaseComplete) {

        if (forwardStartTime == 0) {
            // Initializing the start time when the phase begins
            forwardStartTime = millis();
            Serial.println("Phase 1: Starting FORWARD movement.");
        }

        // 1. Calculate Error (shortest path from current to target)
        float error = targetYaw - currentYaw;
        // This 'if/else' block handles the "wrap-around" (e.g., 350 -> 10 deg)
        if (error > 180.0f) {
            error -= 360.0f;
        } else if (error < -180.0f) {
            error += 360.0f;
        }

        // 2. Calculate Proportional Output
        // This is the simplest form of PID: just Kp * error
        float p_Output = Kp * error;

        // --- MOTOR CONTROL ---
        // Apply P-output to the base speed.
        // Assumes: +p_Output means "turn right" (Left Motor Faster, Right Motor Slower)
        // If your robot turns the wrong way, swap the +/- signs.
        int leftSpeed = (int)(BASE_SPEED + p_Output);
        int rightSpeed = (int)(BASE_SPEED - p_Output);

        // Constrain speeds to valid motor range (-255 to 255)
        leftSpeed = constrain(leftSpeed, -255, 255);
        rightSpeed = constrain(rightSpeed, -255, 255);

        // ACTION: Move forward with P-correction
        robotMotors.move(leftSpeed, rightSpeed);
        // TRANSITION: Check if 2 seconds have passed (Non-blocking check)
        if (millis() - forwardStartTime >= FORWARD_DURATION_MS) {
            robotMotors.stop();
            forwardPhaseComplete = true;  // Mark phase 1 as done
            targetYaw += 90.0;
            targetYaw = fmod(targetYaw, 360.0);
            Serial.println("Phase 1 Complete. Preparing Phase 2: Turn.");
        }
    } else {
        // ACTION: Spin in place (Left Fwd, Right Rev)
        robotMotors.move(80, -80);

        float yawDifference = currentYaw - targetYaw;
        yawDifference = fmod(yawDifference + 180.0, 360.0) - 180.0f;

        if (abs(yawDifference) <= 10.0) {
            robotMotors.stop();  // Stop the turn
            // turnPhaseComplete = true;
            // sequenceRunning = false;  // Sequence complete

            forwardPhaseComplete = false;
            forwardStartTime = 0;
            Serial.println("Phase 2 Complete. Sequence finished.");
        }
    }

    // delay(2);
}
