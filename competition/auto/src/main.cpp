#include <Arduino.h>

#include <HCSR04.h>
#include <Servo.h>
#define __ARDUINO__
#include <ustd_functional.h>

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

GyroTracker gyro;
Servo armServo;
MotorController robotMotors(MOTOR_LEFT_1, MOTOR_LEFT_2, MOTOR_LEFT_EN, MOTOR_RIGHT_1, MOTOR_RIGHT_2, MOTOR_RIGHT_EN);

/**
 * @brief Holds all "read-only" sensor data for a single loop.
 * This struct is filled by your main loop() before being passed
 * to the active step.
 */
struct RobotData {
    float ultrasonicDistance;
    float heading;
};

/**
 * @brief Holds all "read/write" state for the robot.
 */
struct RobotState {
    double targetHeading = 0.0;
};

/**
 * @brief Holds all "read/write" state for the *currently running* step.
 * This object is reset every time a new step begins.
 */
struct SequenceState {
    bool isFirstRun = true;           // Used to trigger "onStart" logic
    unsigned long stepStartTime = 0;  // For time-based actions

    size_t nextSequence = -1;
};

/**
 * @brief A single step in the sequence.
 * It holds one function that contains all logic for that step.
 * * @param data  Read-only sensor data.
 * @param state Read/write data for this step's internal use.
 * @param state Read/write data for the robot.
 * @return      true when the step is finished, false to keep running.
 */
struct SequenceStep {
    ustd::function<bool(const RobotData &data, RobotState *robotState, SequenceState *sequenceState)> run;
};

void panic() {
    while (true) {
        Serial.println("Panic!!!");
        delay(100);
    }
}

void moveWithHeading(double currentHeading, double targetHeading, int speed, double Kp) {
    float error = targetHeading - currentHeading;
    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }

    float p_Output = Kp * error;

    int leftSpeed = (int)(speed + p_Output);
    int rightSpeed = (int)(speed - p_Output);

    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    robotMotors.move(leftSpeed, rightSpeed);
}

SequenceStep script[] = {

    {[](const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
        if (sequenceState->isFirstRun) {
            sequenceState->stepStartTime = millis();
            sequenceState->isFirstRun = false;
        }

        moveWithHeading(data.heading, robotState->targetHeading, 200, 10.0);

        if (millis() - sequenceState->stepStartTime >= 2000) {
            robotMotors.stop();
            robotState->targetHeading -= 90.0;
            robotState->targetHeading = fmod(robotState->targetHeading, 360.0);
            return true;
        }

        return false;
    }},

    {[](const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
        if (sequenceState->isFirstRun) {
            robotMotors.move(-80, 80);
            sequenceState->isFirstRun = false;
        }

        float diff = data.heading - robotState->targetHeading;
        diff = fmod(diff + 180.0, 360.0) - 180.0f;
        if (abs(diff) <= 25.0) {
            robotMotors.stop();
            sequenceState->nextSequence = 0;
            return true;
        }

        return false;
    }},

    {[](const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
        if (sequenceState->isFirstRun) {
            robotMotors.stop();
            sequenceState->isFirstRun = false;
        }
        return true;  // This step is done immediately
    }}
};
const int NUM_STEPS = sizeof(script) / sizeof(script[0]);

// --- Global State Variables ---
int currentStep = 0;          // The index of the step we are on
SequenceState sequenceState;  // The state for the *current* step
RobotData robotData;          // The sensor data for the *current* loop
RobotState robotState;        // The state for the robot

bool sequenceDone = false;  // Flag to stop processing

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

void loop() {
    if (sequenceDone) {
        // The script is finished, do nothing.
        return;
    }

    gyro.update();
    robotData.heading = gyro.getHeading();

    robotData.ultrasonicDistance = *HCSR04.measureDistanceCm();

    // Serial.print("Heading: ");
    // Serial.println(robotData.heading, 2);  // Print to 2 decimal places

    // Serial.print("Ultrasonic Distance: ");
    // Serial.print(robotData.ultrasonicDistance);
    // Serial.println(" cm");

    SequenceStep &step = script[currentStep];
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
