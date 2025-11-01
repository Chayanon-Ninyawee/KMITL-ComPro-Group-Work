#include "seq_r.h"

bool seq_r_0(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 500);  // Unchanged
}
bool seq_r_1(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);  // Flipped heading
}
bool seq_r_2(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);  // Flipped turn
}
bool seq_r_3(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveForTime(data, robotState, sequenceState, 700, -90.0);  // Flipped heading
}
bool seq_r_4(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);  // Flipped turn
}
bool seq_r_5(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveForTime(data, robotState, sequenceState, 750, 90.0);  // Flipped heading
}
bool seq_r_6(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);  // Flipped turn
}
bool seq_r_7(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_setServo(data, robotState, sequenceState, SERVO_MIN, 1000);  // Unchanged
}
bool seq_r_8(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveBackwardHeading(data, robotState, sequenceState, 800);  // Unchanged
}
bool seq_r_9(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_setServo(data, robotState, sequenceState, SERVO_MAX, 2000);  // Unchanged
}
bool seq_r_10(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);  // Flipped heading
}
bool seq_r_11(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);  // Flipped turn
}
bool seq_r_12(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);  // Unchanged
}
bool seq_r_13(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);  // Flipped heading
}
bool seq_r_14(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);  // Flipped turn
}
bool seq_r_15(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);  // Unchanged
}
bool seq_r_16(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);  // Flipped heading
}
bool seq_r_17(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);  // Flipped turn
}
bool seq_r_18(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveBackwardRaw(data, robotState, sequenceState, 600);  // Unchanged
}
bool seq_r_19(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_waitAndResetGyro(data, robotState, sequenceState, 400, 0.0);  // Unchanged
}
bool seq_r_20(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);  // Flipped heading
}
bool seq_r_21(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);  // Flipped turn
}
bool seq_r_22(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);  // Unchanged
}
bool seq_r_23(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);  // Flipped heading
}
bool seq_r_24(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);  // Flipped turn
}
bool seq_r_25(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);  // Unchanged
}
bool seq_r_26(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveForTime(data, robotState, sequenceState, 700, -90.0);  // Flipped heading
}
bool seq_r_27(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);  // Flipped turn
}
bool seq_r_28(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);  // Flipped heading
}
bool seq_r_29(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);  // Flipped turn
}
bool seq_r_30(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);  // Unchanged
}
bool seq_r_31(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);  // Flipped heading
}
bool seq_r_32(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);  // Flipped turn
}
bool seq_r_33(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);  // Unchanged
}
bool seq_r_34(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);  // Flipped heading
}
bool seq_r_35(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);  // Flipped turn
}
bool seq_r_36(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);  // Unchanged
}
bool seq_r_37(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);  // Flipped heading
}
bool seq_r_38(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);  // Flipped turn
}
bool seq_r_39(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_setServo(data, robotState, sequenceState, SERVO_MIN, 2000);  // Unchanged
}
bool seq_r_40(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveBackwardHeading(data, robotState, sequenceState, 800);  // Unchanged
}
bool seq_r_41(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);  // Flipped heading
}
bool seq_r_42(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_setServo(data, robotState, sequenceState, SERVO_MAX, 1000);  // Unchanged
}
bool seq_r_43(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);  // Flipped turn
}
bool seq_r_44(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveBackwardRaw(data, robotState, sequenceState, 600);  // Unchanged
}
bool seq_r_45(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_waitAndResetGyro(data, robotState, sequenceState, 400, 0.0);  // Unchanged
}
bool seq_r_46(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);  // Flipped heading
}
bool seq_r_47(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);  // Flipped turn
}
bool seq_r_48(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);  // Unchanged
}
bool seq_r_49(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);  // Flipped heading
}
bool seq_r_50(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);  // Flipped turn
}
bool seq_r_51(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);  // Unchanged
}
bool seq_r_52(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveForTime(data, robotState, sequenceState, 1350, -90.0);  // Flipped heading
}
bool seq_r_53(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);  // Flipped turn
}
bool seq_r_54(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);  // Unchanged
}
bool seq_r_55(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);  // Flipped heading
}
bool seq_r_56(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);  // Flipped turn
}
bool seq_r_57(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);  // Unchanged
}
bool seq_r_58(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);  // Flipped heading
}
bool seq_r_59(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);  // Flipped turn
}
bool seq_r_60(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);  // Unchanged
}
bool seq_r_61(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);  // Flipped heading
}
bool seq_r_62(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);  // Flipped turn
}
bool seq_r_63(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveBackwardRaw(data, robotState, sequenceState, 600);  // Unchanged
}
bool seq_r_64(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_waitAndResetGyro(data, robotState, sequenceState, 400, 180.0);  // Unchanged
}
bool seq_r_65(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 90.0);  // Flipped heading
}
bool seq_r_66(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);  // Flipped turn
}
bool seq_r_67(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);  // Unchanged
}
bool seq_r_68(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveForTime(data, robotState, sequenceState, 750, 90.0);  // Flipped heading
}
bool seq_r_69(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, TURN_SPEED, -TURN_SPEED);  // Flipped turn
}
bool seq_r_70(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveBackwardRaw(data, robotState, sequenceState, 600);  // Unchanged
}
bool seq_r_71(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_waitAndResetGyro(data, robotState, sequenceState, 400, 0.0);  // Unchanged
}
bool seq_r_72(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, -90.0);  // Flipped heading
}
bool seq_r_73(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_turnToHeading(data, robotState, sequenceState, -TURN_SPEED, TURN_SPEED);  // Flipped turn
}
bool seq_r_74(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_wait(data, robotState, sequenceState, 300);  // Unchanged
}
bool seq_r_75(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_moveTillObstacle(data, robotState, sequenceState, 0.0);  // Unchanged
}
// NOTE: The final step (stop)
bool seq_r_76(const RobotData &data, RobotState *robotState, SequenceState *sequenceState) {
    return helper_stop(data, robotState, sequenceState);  // Unchanged
}
