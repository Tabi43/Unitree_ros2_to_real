/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FIXEDKENNEL_H
#define FIXEDKENNEL_H

#include "FSM/FSMState.h"

class State_FixedKennel : public FSMState{
public:
    State_FixedKennel(CtrlComponents *ctrlComp);
    ~State_FixedKennel(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
    bool isReadyForModeTransition() const;

private:
    float _targetPos[12] = {0.35, 1.24, -2.81, -0.35, 1.24, -2.81, 
                            0.35, 1.24, -2.81, -0.35, 1.24, -2.81};
    float _startPos[12];
    float _duration = 1000;   //steps
    float _percent = 0;       //%
};

#endif  // FIXEDKENNEL_H
