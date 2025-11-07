#include "seq_l_stop.h"

bool seq_l_stop_0(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 500);
}
bool seq_l_stop_1(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);
}
bool seq_l_stop_2(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_l_stop_3(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_l_stop_4(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);
}
bool seq_l_stop_5(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_l_stop_6(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_l_stop_7(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_l_stop_8(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_l_stop_9(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_l_stop_10(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}

bool seq_l_stop_11(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_stop(data, robotState, sequenceState);
}
