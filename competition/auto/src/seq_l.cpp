#include "seq_l.h"

bool seq_l_0(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 500);
}
bool seq_l_1(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_l_2(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_l_3(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveForTime(data, robotState, sequenceState, 700, 90.0);
}
bool seq_l_4(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_l_5(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveForTime(data, robotState, sequenceState, 750, -90.0);
}
bool seq_l_6(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_l_7(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_setServo(data, robotState, sequenceState, SERVO_MIN, 1000);
}
bool seq_l_8(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveBackwardHeading(data, robotState, sequenceState, 800);
}
bool seq_l_9(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_setServo(data, robotState, sequenceState, SERVO_MAX, 2000);
}
bool seq_l_10(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_l_11(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_l_12(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_l_13(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);
}
bool seq_l_14(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_l_15(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_l_16(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);
}
bool seq_l_17(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_l_18(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveBackwardRaw(data, robotState, sequenceState, 600);
}
bool seq_l_19(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_waitAndResetGyro(data, robotState, sequenceState, 400, 0.0);
}
bool seq_l_20(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_l_21(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_l_22(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_l_23(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_l_24(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_l_25(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_l_26(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveForTime(data, robotState, sequenceState, 700, 90.0);
}
bool seq_l_27(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_l_28(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_l_29(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_l_30(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_l_31(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_l_32(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_l_33(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_l_34(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);
}
bool seq_l_35(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_l_36(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_l_37(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_l_38(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_l_39(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_setServo(data, robotState, sequenceState, SERVO_MIN, 2000);
}
bool seq_l_40(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveBackwardHeading(data, robotState, sequenceState, 800);
}
bool seq_l_41(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_l_42(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_setServo(data, robotState, sequenceState, SERVO_MAX, 1000);
}
bool seq_l_43(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_l_44(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveBackwardRaw(data, robotState, sequenceState, 600);
}
bool seq_l_45(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_waitAndResetGyro(data, robotState, sequenceState, 400, 0.0);
}
bool seq_l_46(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_l_47(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_l_48(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_l_49(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);
}
bool seq_l_50(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_l_51(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_l_52(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveForTime(data, robotState, sequenceState, 1300, 90.0);
}
bool seq_l_53(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_l_54(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_l_55(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_l_56(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_l_57(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_l_58(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);
}
bool seq_l_59(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_l_60(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_l_61(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);
}
bool seq_l_62(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_l_63(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveBackwardRaw(data, robotState, sequenceState, 600);
}
bool seq_l_64(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_waitAndResetGyro(data, robotState, sequenceState, 400, 180.0);
}
bool seq_l_65(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);
}
bool seq_l_66(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_l_67(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_l_68(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveForTime(data, robotState, sequenceState, 750, -90.0);
}
bool seq_l_69(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);
}
bool seq_l_70(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveBackwardRaw(data, robotState, sequenceState, 600);
}
bool seq_l_71(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_waitAndResetGyro(data, robotState, sequenceState, 400, 0.0);
}
bool seq_l_72(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);
}
bool seq_l_73(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);
}
bool seq_l_74(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);
}
bool seq_l_75(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 0.0);
}
// NOTE: The final step (stop)
bool seq_l_76(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_stop(data, robotState, sequenceState);
}
