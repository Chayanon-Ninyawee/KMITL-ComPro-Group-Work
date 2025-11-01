#include <Arduino.h>
#include <avr/pgmspace.h>

#include <Servo.h>
#define __ARDUINO__
#include <ustd_functional.h>

#include "gyro_tracker.h"
#include "motor_controller.h"

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

/**
 * @brief Holds all "read-only" sensor data for a single loop.
 * This struct is filled by your main loop() before being passed
 * to the active step.
 */
struct RobotData {
    double ultrasonicDistance;
    double heading;
};
constexpr double ULTRASONIC_DISTANCE_SENTINEL_VALUE = -1.0;

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

typedef bool (*StepFunc)(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);

/**
 * @brief A single step in the sequence.
 * It holds one function that contains all logic for that step.
 * * @param data  Read-only sensor data.
 * @param state Read/write data for this step's internal use.
 * @param state Read/write data for the robot.
 * @return      true when the step is finished, false to keep running.
 */
struct SequenceStep {
    // ustd::function<bool(const RobotData &data, RobotState *robotState, SequenceState *sequenceState)> run;
    StepFunc run;
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

/**
 * @brief Helper function for turning in place to the target heading.
 * @return true when the step is complete (heading is reached).
 */
bool helper_turnToHeading(const RobotData &data, RobotState *robotState, SequenceState *sequenceState, int leftSpeed, int rightSpeed) {
    if (sequenceState->isFirstRun) {
        robotMotors.move(leftSpeed, rightSpeed);
        sequenceState->isFirstRun = false;
    }

    // Calculate the shortest difference between current and target heading
    float diff = data.heading - robotState->targetHeading;
    diff = fmod(diff + 180.0, 360.0) - 180.0f;

    // Check if we are within the tolerance
    if (abs(diff) <= 35.0) {
        robotMotors.stop();
        return true;  // Step is complete
    }
    return false;  // Step is not complete
}

/**
 * @brief Helper function for moving until an obstacle is detected, then turning.
 * @param turnAngle The angle to add to targetHeading (-90 for left, 90 for right).
 * @return true when the step is complete (obstacle found).
 */
bool helper_moveTillObstacle(const RobotData &data, RobotState *robotState, SequenceState *sequenceState, float turnAngle) {
    if (sequenceState->isFirstRun) {
        sequenceState->isFirstRun = false;
    }

    // Continuously move forward with heading correction
    moveWithHeading(data.heading, robotState->targetHeading, FORWARD_SPEED, 10.0);

    // Check for obstacle
    if (data.ultrasonicDistance <= ULTRASONIC_DISTANCE && data.ultrasonicDistance != ULTRASONIC_DISTANCE_SENTINEL_VALUE) {
        robotMotors.stop();
        // Set new target heading for the *next* step
        robotState->targetHeading += turnAngle;
        robotState->targetHeading = fmod(robotState->targetHeading, 360.0);
        return true;  // Step is complete
    }
    return false;  // Step is not complete
}

/**
 * @brief Helper function for moving forward for a set duration, then turning.
 * @param duration The time to move forward (in milliseconds).
 * @param turnAngle The angle to add to targetHeading (-90 for left, 90 for right).
 * @return true when the step is complete (time has elapsed).
 */
bool helper_moveForTime(
    const RobotData &data,
    RobotState *robotState,
    SequenceState *sequenceState,
    unsigned long duration,
    float turnAngle
) {
    if (sequenceState->isFirstRun) {
        sequenceState->stepStartTime = millis();
        sequenceState->isFirstRun = false;
    }

    moveWithHeading(data.heading, robotState->targetHeading, FORWARD_SPEED, 10.0);

    if (millis() - sequenceState->stepStartTime >= duration) {
        robotMotors.stop();
        // Set new target heading for the *next* step
        robotState->targetHeading += turnAngle;
        robotState->targetHeading = fmod(robotState->targetHeading, 360.0);
        return true;
    }
    return false;
}

/**
 * @brief Helper function for a simple timed wait.
 * @param duration The time to wait (in milliseconds).
 * @return true when the step is complete (time has elapsed).
 */
bool helper_wait(const RobotData &data, RobotState *robotState, SequenceState *sequenceState, unsigned long duration) {
    if (sequenceState->isFirstRun) {
        sequenceState->stepStartTime = millis();
        sequenceState->isFirstRun = false;
    }

    if (millis() - sequenceState->stepStartTime >= duration) {
        return true;
    }
    return false;
}

/**
 * @brief Helper function to set servo position and wait.
 * @param servoPosition The target position for the servo (e.g., SERVO_MIN).
 * @param duration The time to wait (in milliseconds).
 * @return true when the step is complete (time has elapsed).
 */
bool helper_setServo(
    const RobotData &data,
    RobotState *robotState,
    SequenceState *sequenceState,
    int servoPosition,
    unsigned long duration
) {
    if (sequenceState->isFirstRun) {
        robotMotors.stop();  // Good practice to stop motors
        armServo.write(servoPosition);
        sequenceState->stepStartTime = millis();
        sequenceState->isFirstRun = false;
    }

    if (millis() - sequenceState->stepStartTime >= duration) {
        return true;
    }
    return false;
}

/**
 * @brief Helper function to move backward using heading control for a duration.
 * @param duration The time to move (in milliseconds).
 * @return true when the step is complete (time has elapsed).
 */
bool helper_moveBackwardHeading(const RobotData &data, RobotState *robotState, SequenceState *sequenceState, unsigned long duration) {
    if (sequenceState->isFirstRun) {
        sequenceState->stepStartTime = millis();
        sequenceState->isFirstRun = false;
    }

    // Move backward with heading correction
    moveWithHeading(data.heading, robotState->targetHeading, BACKWARD_SPEED, 10.0);

    if (millis() - sequenceState->stepStartTime >= duration) {
        robotMotors.stop();
        return true;
    }
    return false;
}

/**
 * @brief Helper function to move backward with raw motor commands for a duration.
 * @param duration The time to move (in milliseconds).
 * @return true when the step is complete (time has elapsed).
 */
bool helper_moveBackwardRaw(const RobotData &data, RobotState *robotState, SequenceState *sequenceState, unsigned long duration) {
    if (sequenceState->isFirstRun) {
        robotMotors.move(-100, -100);
        sequenceState->stepStartTime = millis();
        sequenceState->isFirstRun = false;
    }

    if (millis() - sequenceState->stepStartTime >= duration) {
        robotMotors.stop();
        return true;
    }
    return false;
}

/**
 * @brief Helper function to wait and then reset the gyro.
 * @param duration The time to wait (in milliseconds).
 * @return true when the step is complete (time has elapsed).
 */
bool helper_waitAndResetGyro(
    const RobotData &data,
    RobotState *robotState,
    SequenceState *sequenceState,
    unsigned long duration,
    double resettedHeading
) {
    if (sequenceState->isFirstRun) {
        sequenceState->stepStartTime = millis();
        sequenceState->isFirstRun = false;
    }

    if (millis() - sequenceState->stepStartTime >= duration) {
        gyro.resetHeading(resettedHeading);
        return true;
    }
    return false;
}

/**
 * @brief Helper function to stop all motors.
 * @return true (step is complete immediately).
 */
bool helper_stop(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    if (sequenceState->isFirstRun) {
        robotMotors.stop();
        sequenceState->isFirstRun = false;
    }
    return true;  // This step is done immediately
}

bool seq_0(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 500);
}
bool seq_1(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_2(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_3(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveForTime(data, robotState, sequenceState, 700, 90.0);
}
bool seq_4(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_5(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveForTime(data, robotState, sequenceState, 750, -90.0);
}
bool seq_6(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_7(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_setServo(data, robotState, sequenceState, SERVO_MIN, 1000);
}
bool seq_8(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveBackwardHeading(data, robotState, sequenceState, 800);
}
bool seq_9(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_setServo(data, robotState, sequenceState, SERVO_MAX, 2000);
}
bool seq_10(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_11(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_12(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_13(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);
}
bool seq_14(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_15(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_16(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);
}
bool seq_17(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_18(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveBackwardRaw(data, robotState, sequenceState, 600);
}
bool seq_19(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_waitAndResetGyro(data, robotState, sequenceState, 400, 0.0);
}
bool seq_20(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_21(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_22(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_23(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_24(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_25(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveForTime(data, robotState, sequenceState, 600, 90.0);
}
bool seq_26(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_27(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_28(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_29(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_30(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_31(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_32(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);
}
bool seq_33(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_34(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_35(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_36(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_37(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_setServo(data, robotState, sequenceState, SERVO_MIN, 2000);
}
bool seq_38(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveBackwardHeading(data, robotState, sequenceState, 800);
}
bool seq_39(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_40(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_setServo(data, robotState, sequenceState, SERVO_MAX, 1000);
}
bool seq_41(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_42(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveBackwardRaw(data, robotState, sequenceState, 600);
}
bool seq_43(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_waitAndResetGyro(data, robotState, sequenceState, 400, 0.0);
}
bool seq_44(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_45(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_46(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_47(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);
}
bool seq_48(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_49(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_50(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveForTime(data, robotState, sequenceState, 1300, 90.0);
}
bool seq_51(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_52(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_53(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_54(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_55(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);
}
bool seq_56(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_57(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_58(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);
}
bool seq_59(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_60(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveBackwardRaw(data, robotState, sequenceState, 600);
}
bool seq_61(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_waitAndResetGyro(data, robotState, sequenceState, 400, 180.0);
}
bool seq_62(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_63(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_64(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_65(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveForTime(data, robotState, sequenceState, 750, -90.0);
}
bool seq_66(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_67(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveBackwardRaw(data, robotState, sequenceState, 600);
}
bool seq_68(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_waitAndResetGyro(data, robotState, sequenceState, 400, 0.0);
}
bool seq_69(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);
}
bool seq_70(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_71(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_72(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 0.0);
}
// NOTE: The final step (stop)
bool seq_73(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_stop(data, robotState, sequenceState);
}

const SequenceStep script_l[] = {{seq_0},  {seq_1},  {seq_2},  {seq_3},  {seq_4},  {seq_5},  {seq_6},  {seq_7},  {seq_8},  {seq_9},
                                 {seq_10}, {seq_11}, {seq_12}, {seq_13}, {seq_14}, {seq_15}, {seq_16}, {seq_17}, {seq_18}, {seq_19},
                                 {seq_20}, {seq_21}, {seq_22}, {seq_23}, {seq_24}, {seq_25}, {seq_26}, {seq_27}, {seq_28}, {seq_29},
                                 {seq_30}, {seq_31}, {seq_32}, {seq_33}, {seq_34}, {seq_35}, {seq_36}, {seq_37}, {seq_38}, {seq_39},
                                 {seq_40}, {seq_41}, {seq_42}, {seq_43}, {seq_44}, {seq_45}, {seq_46}, {seq_47}, {seq_48}, {seq_49},
                                 {seq_50}, {seq_51}, {seq_52}, {seq_53}, {seq_54}, {seq_55}, {seq_56}, {seq_57}, {seq_58}, {seq_59},
                                 {seq_60}, {seq_61}, {seq_62}, {seq_63}, {seq_64}, {seq_65}, {seq_66}, {seq_67}, {seq_68}, {seq_69},
                                 {seq_70}, {seq_71}, {seq_72}, {seq_73}};
const int NUM_STEPS = sizeof(script_l) / sizeof(script_l[0]);

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

    const SequenceStep &step = script_l[currentStep];
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
