/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FSM_H
#define FSM_H

#include <cstdint>
#include <cstddef>
#include <memory>
#include <vector>

// FSM States
#include "FSM/FSMState.h"
#include "FSM/State_FixedStand.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FreeStand.h"
#include "FSM/State_Trotting.h"
#include "FSM/State_FixedKennel.h"
#include "FSM/State_BalanceTest.h"
#include "FSM/State_SwingTest.h"
#include "FSM/State_StepTest.h"
#include "common/enumClass.h"
#include "control/CtrlComponents.h"
#include "FSM/State_move_base.h"

struct FSMStateList{
    std::shared_ptr<FSMState> invalid;
    std::shared_ptr<State_Passive> passive;
    std::shared_ptr<State_FixedStand> fixedStand;
    std::shared_ptr<State_FixedKennel> fixedKennel;
    std::shared_ptr<State_FreeStand> freeStand;
    std::shared_ptr<State_Trotting> trotting;
    std::shared_ptr<State_BalanceTest> balanceTest;
    std::shared_ptr<State_SwingTest> swingTest;
    std::shared_ptr<State_StepTest> stepTest;
    std::shared_ptr<State_move_base> moveBase;
};

class FSM{
public:
    FSM(CtrlComponents *ctrlComp);
    ~FSM() = default;
    void initialize();
    void run();
private:
    struct ModePlanStep{
        FSMStateName targetState;
        UserCommand command;
    };

    std::shared_ptr<FSMState> getNextState(FSMStateName stateName);
    bool checkSafty();
    void handleModeRequest();
    bool startModePlan(uint8_t requested, bool replacing);
    bool buildModePlan(uint8_t requested, std::vector<ModePlanStep> &plan) const;
    bool routeToState(FSMStateName &from, FSMStateName target, std::vector<ModePlanStep> &plan) const;
    bool appendTransition(FSMStateName &from, FSMStateName to, std::vector<ModePlanStep> &plan) const;
    bool commandForTransition(FSMStateName from, FSMStateName to, UserCommand &command) const;
    void advanceModePlanIfReady();
    bool isCriticalTransitionActive() const;
    bool isInterruptMode(uint8_t requested) const;
    const char *fsmStateToString(FSMStateName stateName) const;
    const char *userCommandToString(UserCommand command) const;
    const char *highModeToString(uint8_t mode) const;

    CtrlComponents *_ctrlComp;
    std::shared_ptr<FSMState> _currentState;
    std::shared_ptr<FSMState> _nextState;
    FSMStateName _nextStateName;
    FSMStateList _stateList;
    FSMMode _mode;
    std::vector<ModePlanStep> _modePlan;
    size_t _modePlanIndex = 0;
    bool _modePlanActive = false;
    uint8_t _activeModeRequest = _PASSIVE;
    bool _hasDeferredModeRequest = false;
    uint8_t _deferredModeRequest = _PASSIVE;
    long long _startTime;
    int count;
};


#endif  // FSM_H
