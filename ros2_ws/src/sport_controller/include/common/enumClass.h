/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>

#define _START 10
#define _STOP 20
#define _PASSIVE 0
#define _FREE_STAND_MODE 1
#define _FIXED_STAND_MODE 6
#define _STAND_DOWN_MODE 5
#define _VELOCITY_MODE 11
#define _MOVE_BASE 2

enum class CtrlPlatform{
    GAZEBO,
    REALROBOT,
};

enum class RobotType{
    A1,
    Go1
};

enum class UserCommand{
    // EXIT,
    NONE,
    START,      // velocity mode command
    L2_A,       // fixedStand
    L2_X,       // fixedKennel
    L2_B,       // passive
    SELECT,     // freeStand
    L1_X,       // move_base
};

enum class FrameType{
    BODY,
    HIP,
    GLOBAL
};

enum class WaveStatus{
    STANCE_ALL,
    SWING_ALL,
    WAVE_ALL
};

enum class FSMMode{
    NORMAL,
    CHANGE
};

enum class FSMStateName{
    // EXIT,
    INVALID,
    PASSIVE,
    FIXEDSTAND,
    FIXEDKENNEL,
    FREESTAND,
    TROTTING,
    MOVE_BASE,       // move_base
    BALANCETEST,
    SWINGTEST,
    STEPTEST
};

#endif  // ENUMCLASS_H
