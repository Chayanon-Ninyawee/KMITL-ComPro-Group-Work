#include "seq_helper.h"

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

bool helper_moveBackwardRaw(const RobotData &data, RobotState *robotState, SequenceState *sequenceState, unsigned long duration) {
    if (sequenceState->isFirstRun) {
        robotMotors.move(-150, -150);
        sequenceState->stepStartTime = millis();
        sequenceState->isFirstRun = false;
    }

    if (millis() - sequenceState->stepStartTime >= duration) {
        robotMotors.stop();
        return true;
    }
    return false;
}

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

bool helper_stop(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    if (sequenceState->isFirstRun) {
        robotMotors.stop();
        sequenceState->isFirstRun = false;
    }
    return true;  // This step is done immediately
}
