#include <Arduino.h>
#include <avr/pgmspace.h>

#include <Servo.h>

#include "gyro_tracker.h"
#include "motor_controller.h"

// #define USE_LEFT_SCRIPT

#if defined(USE_LEFT_SCRIPT)
#include "seq_l.h"
#else
#include "seq_r.h"
#endif

constexpr byte BUTTON_PIN = 12;

constexpr byte SERVO_PIN = A0;
constexpr double SERVO_MIN = 20;
constexpr double SERVO_MAX = 140;

constexpr byte MOTOR_LEFT_1 = 7;
constexpr byte MOTOR_LEFT_2 = 8;
constexpr byte MOTOR_LEFT_EN = 6;

constexpr byte MOTOR_RIGHT_1 = 4;
constexpr byte MOTOR_RIGHT_2 = 10;
constexpr byte MOTOR_RIGHT_EN = 5;

constexpr byte TRIGGER_PIN = 11;
constexpr byte ECHO_PIN = 3;

constexpr int FULL_BATTERY_FORWARD_SPEED = 130;
constexpr int FULL_BATTERY_BACKWARD_SPEED = -130;
constexpr int FULL_BATTERY_TURN_SPEED = 100;
constexpr double FULL_BATTERY_ULTRASONIC_DISTANCE = 0.065;

constexpr int MID_BATTERY_FORWARD_SPEED = 150;
constexpr int MID_BATTERY_BACKWARD_SPEED = -150;
constexpr int MID_BATTERY_TURN_SPEED = 100;
constexpr double MID_BATTERY_ULTRASONIC_DISTANCE = 0.06;

// NOTE: Change this according to battery level
constexpr int FORWARD_SPEED = FULL_BATTERY_FORWARD_SPEED;
constexpr int BACKWARD_SPEED = FULL_BATTERY_BACKWARD_SPEED;
constexpr int TURN_SPEED = FULL_BATTERY_TURN_SPEED;
constexpr double ULTRASONIC_DISTANCE = FULL_BATTERY_ULTRASONIC_DISTANCE;

GyroTracker gyro;
Servo armServo;
MotorController robotMotors(MOTOR_LEFT_1, MOTOR_LEFT_2, MOTOR_LEFT_EN, MOTOR_RIGHT_1, MOTOR_RIGHT_2, MOTOR_RIGHT_EN);

void panic() {
    while (true) {
        Serial.println("Panic!!!");
        delay(100);
    }
}

const int NUM_STEPS = sizeof(script) / sizeof(script[0]);

// --- Global State Variables ---
int currentStep = 0;          // The index of the step we are on
SequenceState sequenceState;  // The state for the *current* step
RobotData robotData;          // The sensor data for the *current* loop
RobotState robotState;        // The state for the robot

bool sequenceDone = false;  // Flag to stop processing

// --- Volatile variables ---
// These are shared between the main loop and the Interrupt Service Routine (ISR)
// 'volatile' tells the compiler that these variables can change unexpectedly.
volatile unsigned long pulseStartTime = 0;
volatile unsigned long pulseDuration = 0;
volatile boolean newReadingAvailable = false;
void isrHandleEcho() {
    // Check if the pin just went HIGH
    if (digitalRead(ECHO_PIN) == HIGH) {
        pulseStartTime = micros();  // Record the start time
    }
    // Otherwise, the pin must have just gone LOW
    else
    {
        pulseDuration = micros() - pulseStartTime;  // Calculate the pulse duration
        newReadingAvailable = true;                 // Set the flag for the main loop
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), isrHandleEcho, CHANGE);

    armServo.attach(SERVO_PIN);

    robotMotors.begin();

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    delay(100);

    armServo.write(SERVO_MAX);

    // Wait for the button press to start the gyro calibration
    while (digitalRead(BUTTON_PIN)) {
        delay(100);
    }
    delay(1000);

    if (!gyro.begin()) {
        Serial.println("ERROR: BMI160 initialization failed! Check wiring.");
        panic();
    }

    Serial.println("Yaw tracking starting...");
    // IMPORTANT: Keep the device perfectly still during this process.
    gyro.calibrate(500);  // Calibrate using 500 samples
    Serial.print("Yaw tracking started. Offset used (LSB): ");
    Serial.println(gyro.getOffset());

    // Wait for the button press to start the robot
    while (digitalRead(BUTTON_PIN)) {
        delay(100);
    }
    delay(500);
}

unsigned long lastTriggerTime = 0;

void loop() {
    if (sequenceDone) {
        // The script is finished, do nothing.
        return;
    }

    gyro.update();
    robotData.heading = gyro.getHeading();

    unsigned long currentMillis = millis();
    if (currentMillis - lastTriggerTime >= 60) {
        lastTriggerTime = currentMillis;  // Update the last trigger time

        // Send the 10-microsecond trigger pulse
        digitalWrite(TRIGGER_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIGGER_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER_PIN, LOW);
    }
    if (newReadingAvailable) {
        unsigned long duration;  // Local variable to hold the duration

        // --- Critical Section ---
        // Disable interrupts temporarily to safely copy the volatile variable
        noInterrupts();
        duration = pulseDuration;
        newReadingAvailable = false;  // Clear the flag
        interrupts();
        // -----------------------

        // Calculate the distance in meters
        // Sound speed = 343 m/s = 0.000343 m/us
        // Distance = (Duration * Speed of Sound) / 2 (for round trip)
        robotData.ultrasonicDistance = (duration * 0.000343) / 2.0;
    }

    // Serial.print("Heading: ");
    // Serial.println(robotData.heading, 2);  // Print to 2 decimal places

    Serial.print("Ultrasonic Distance: ");
    Serial.print(robotData.ultrasonicDistance);
    Serial.println(" cm");

    const SequenceStep &step = script[currentStep];
    bool isStepFinished = step.run(robotData, &robotState, &sequenceState);
    if (isStepFinished) {
        Serial.print("Finished Step: ");
        Serial.println(currentStep);

        if (sequenceState.nextSequence == -1) {
            currentStep++;
        } else {
            currentStep = sequenceState.nextSequence;
        }

        // Check if the whole script is done
        if (currentStep >= NUM_STEPS) {
            Serial.println("Sequence complete.");
            sequenceDone = true;
        } else {
            // **CRITICAL:** Reset the state for the *new* step
            // This sets isFirstRun back to true
            sequenceState = SequenceState();
        }
    }
}
