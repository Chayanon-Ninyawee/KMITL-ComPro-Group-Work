#pragma once

#include "types.h"

#include "gyro_tracker.h"
#include "motor_controller.h"
#include <Servo.h>

extern MotorController robotMotors;
extern Servo armServo;

extern GyroTracker gyro;

extern const int FORWARD_SPEED;
extern const int BACKWARD_SPEED;
extern const int TURN_SPEED;
extern const double ULTRASONIC_DISTANCE;

extern const double SERVO_MIN;
extern const double SERVO_MAX;

void moveWithHeading(double currentHeading, double targetHeading, int speed, double Kp);

/**
 * @brief Helper function for turning in place to the target heading.
 * @return true when the step is complete (heading is reached).
 */
bool helper_turnToHeading(const RobotData &data, RobotState *robotState, SequenceState *sequenceState, int leftSpeed, int rightSpeed);

/**
 * @brief Helper function for moving until an obstacle is detected, then turning.
 * @param turnAngle The angle to add to targetHeading (-90 for left, 90 for right).
 * @return true when the step is complete (obstacle found).
 */
bool helper_moveTillObstacle(const RobotData &data, RobotState *robotState, SequenceState *sequenceState, float turnAngle);

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
);

/**
 * @brief Helper function for a simple timed wait.
 * @param duration The time to wait (in milliseconds).
 * @return true when the step is complete (time has elapsed).
 */
bool helper_wait(const RobotData &data, RobotState *robotState, SequenceState *sequenceState, unsigned long duration);

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
);

/**
 * @brief Helper function to move backward using heading control for a duration.
 * @param duration The time to move (in milliseconds).
 * @return true when the step is complete (time has elapsed).
 */
bool helper_moveBackwardHeading(const RobotData &data, RobotState *robotState, SequenceState *sequenceState, unsigned long duration);

/**
 * @brief Helper function to move backward with raw motor commands for a duration.
 * @param duration The time to move (in milliseconds).
 * @return true when the step is complete (time has elapsed).
 */
bool helper_moveBackwardRaw(const RobotData &data, RobotState *robotState, SequenceState *sequenceState, unsigned long duration);

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
);

/**
 * @brief Helper function to stop all motors.
 * @return true (step is complete immediately).
 */
bool helper_stop(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
