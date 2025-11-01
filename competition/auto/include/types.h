#include <Arduino.h>

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
