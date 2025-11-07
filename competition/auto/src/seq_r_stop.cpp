#include "seq_r_stop.h"

bool seq_r_stop_0(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 500);
}
bool seq_r_stop_1(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);
}
bool seq_r_stop_2(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_r_stop_3(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveForTime(data, robotState, sequenceState, 700, -90.0);
}
bool seq_r_stop_4(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_r_stop_5(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_r_stop_6(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_r_stop_7(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveForTime(data, robotState, sequenceState, 710, 0.0);
}

bool seq_r_stop_8(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_stop(data, robotState, sequenceState);
}
