#pragma once

#include "seq_helper.h"

bool seq_r_stop_0(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
bool seq_r_stop_1(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
bool seq_r_stop_2(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
bool seq_r_stop_3(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
bool seq_r_stop_4(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
bool seq_r_stop_5(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
bool seq_r_stop_6(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
bool seq_r_stop_7(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
bool seq_r_stop_8(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);

const SequenceStep script[] = {
    {seq_r_stop_0},
    {seq_r_stop_1},
    {seq_r_stop_2},
    {seq_r_stop_3},
    {seq_r_stop_4},
    {seq_r_stop_5},
    {seq_r_stop_6},
    {seq_r_stop_7},
    {seq_r_stop_8}
};
