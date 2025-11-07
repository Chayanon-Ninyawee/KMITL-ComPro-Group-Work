#pragma once

#include "seq_helper.h"

bool seq_l_stop_0(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
bool seq_l_stop_1(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
bool seq_l_stop_2(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
bool seq_l_stop_3(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
bool seq_l_stop_4(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
bool seq_l_stop_5(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
bool seq_l_stop_6(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
bool seq_l_stop_7(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
bool seq_l_stop_8(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
bool seq_l_stop_9(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
bool seq_l_stop_10(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);
bool seq_l_stop_11(const RobotData &data, RobotState *robotState, SequenceState *sequenceState);

const SequenceStep script[] = {
    {seq_l_stop_0},
    {seq_l_stop_1},
    {seq_l_stop_2},
    {seq_l_stop_3},
    {seq_l_stop_4},
    {seq_l_stop_5},
    {seq_l_stop_6},
    {seq_l_stop_7},
    {seq_l_stop_8},
    {seq_l_stop_9},
    {seq_l_stop_10},
    {seq_l_stop_11}
};
