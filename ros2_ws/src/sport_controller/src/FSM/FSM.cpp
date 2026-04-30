/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSM.h"
#include "common/Logger.h"
#include <iostream>

FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){

    _stateList.invalid = nullptr;
    _stateList.passive = std::make_shared<State_Passive>(_ctrlComp);
    _stateList.fixedStand = std::make_shared<State_FixedStand>(_ctrlComp);
    _stateList.fixedKennel = std::make_shared<State_FixedKennel>(_ctrlComp);
    _stateList.freeStand = std::make_shared<State_FreeStand>(_ctrlComp);
    _stateList.trotting = std::make_shared<State_Trotting>(_ctrlComp);
    _stateList.balanceTest = std::make_shared<State_BalanceTest>(_ctrlComp);
    _stateList.swingTest = std::make_shared<State_SwingTest>(_ctrlComp);
    _stateList.stepTest = std::make_shared<State_StepTest>(_ctrlComp);
    _stateList.moveBase = std::make_shared<State_move_base>(_ctrlComp);
    initialize();
}

void FSM::initialize(){
    _currentState = _stateList.passive;
    _currentState -> enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
}

void FSM::run(){
    _startTime = getSystemTime();
    _ctrlComp->sendRecv();
    _ctrlComp->runWaveGen();
    _ctrlComp->estimator->run();
    if(!checkSafty()){
        _modePlanActive = false;
        _hasDeferredModeRequest = false;
        _ctrlComp->lowState->userCmd = UserCommand::L2_B;
    } else {
        handleModeRequest();
        advanceModePlanIfReady();
    }

    if(_mode == FSMMode::NORMAL){
        _currentState->run();
        _nextStateName = _currentState->checkChange();
        if(_nextStateName != _currentState->_stateName){
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
            if (_nextState) {
                publish_log("INFO", "Transitioning from " + _currentState->_stateNameString + " to " + _nextState->_stateNameString);
            } else {
                publish_log("ERROR", std::string("Next state is null for state name: ") + fsmStateToString(_nextStateName));
            }
        }
    }
    else if(_mode == FSMMode::CHANGE){
        if (!_nextState) {
            _nextState = _stateList.passive;
            publish_log("WARN", "Falling back to passive state due to null next state");
        }
        _currentState->exit();
        _currentState = _nextState;
        _currentState->enter();
        _mode = FSMMode::NORMAL;
        _currentState->run();
    }

    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
}

void FSM::handleModeRequest(){
    uint8_t requested = _PASSIVE;
    if(!_ctrlComp->ioInter->fetchModeRequest(requested)){
        return;
    }

    if(_modePlanActive){
        if(isInterruptMode(requested)){
            publish_log("WARN", std::string("Interrupting active mode plan with ") + highModeToString(requested));
            startModePlan(requested, true);
        }
        else if(isCriticalTransitionActive()){
            _deferredModeRequest = requested;
            _hasDeferredModeRequest = true;
            publish_log("INFO", std::string("Deferred mode request until posture transition completes: ") + highModeToString(requested));
        }
        else{
            publish_log("INFO", std::string("Replacing active mode plan with ") + highModeToString(requested));
            startModePlan(requested, true);
        }
    }
    else{
        startModePlan(requested, false);
    }
}

bool FSM::startModePlan(uint8_t requested, bool replacing){
    std::vector<ModePlanStep> plan;
    if(!buildModePlan(requested, plan)){
        publish_log("WARN", std::string("Rejected mode request with no valid route: ") + highModeToString(requested));
        return false;
    }

    _modePlan = plan;
    _modePlanIndex = 0;
    _activeModeRequest = requested;
    _modePlanActive = !_modePlan.empty();

    if(_modePlanActive){
        publish_log("INFO", std::string(replacing ? "Started replacement mode plan: " : "Started mode plan: ") + highModeToString(requested));
        for(size_t i = 0; i < _modePlan.size(); ++i){
            publish_log("INFO", "Mode plan step " + std::to_string(i + 1) + ": " +
                   userCommandToString(_modePlan[i].command) + " -> " +
                   fsmStateToString(_modePlan[i].targetState));
        }
    }
    else{
        publish_log("INFO", std::string("Mode request already satisfied: ") + highModeToString(requested));
    }
    return true;
}

bool FSM::buildModePlan(uint8_t requested, std::vector<ModePlanStep> &plan) const{
    plan.clear();
    if(!_currentState){
        return false;
    }

    FSMStateName cursor = _currentState->_stateName;
    if(_mode == FSMMode::CHANGE && _nextState){
        cursor = _nextState->_stateName;
    }

    if(cursor == FSMStateName::FIXEDSTAND || cursor == FSMStateName::FIXEDKENNEL){
        bool postureReady = false;
        if(_mode != FSMMode::CHANGE && _currentState->_stateName == cursor){
            postureReady = _currentState->isReadyForModeTransition();
        }
        if(!postureReady){
            plan.push_back({cursor, UserCommand::NONE});
        }
    }

    switch(requested){
    case _PASSIVE:
        return routeToState(cursor, FSMStateName::PASSIVE, plan);
    case _FIXED_STAND_MODE:
        return routeToState(cursor, FSMStateName::FIXEDSTAND, plan);
    case _FREE_STAND_MODE:
        return routeToState(cursor, FSMStateName::FREESTAND, plan);
    case _STAND_DOWN_MODE:
        return routeToState(cursor, FSMStateName::FIXEDKENNEL, plan);
    case _VELOCITY_MODE:
        return routeToState(cursor, FSMStateName::TROTTING, plan);
    case _MOVE_BASE:
    case _START:
        return routeToState(cursor, FSMStateName::MOVE_BASE, plan);
    case _STOP:
        if(cursor == FSMStateName::PASSIVE || cursor == FSMStateName::FIXEDKENNEL){
            return routeToState(cursor, FSMStateName::PASSIVE, plan);
        }
        if(!routeToState(cursor, FSMStateName::FIXEDSTAND, plan)){
            return false;
        }
        if(!routeToState(cursor, FSMStateName::FIXEDKENNEL, plan)){
            return false;
        }
        return routeToState(cursor, FSMStateName::PASSIVE, plan);
    default:
        return false;
    }
}

bool FSM::routeToState(FSMStateName &from, FSMStateName target, std::vector<ModePlanStep> &plan) const{
    if(from == target){
        return true;
    }

    switch(target){
    case FSMStateName::PASSIVE:
        if(from == FSMStateName::MOVE_BASE){
            return appendTransition(from, FSMStateName::FIXEDSTAND, plan) &&
                   appendTransition(from, FSMStateName::PASSIVE, plan);
        }
        return appendTransition(from, FSMStateName::PASSIVE, plan);
    case FSMStateName::FIXEDSTAND:
        return appendTransition(from, FSMStateName::FIXEDSTAND, plan);
    case FSMStateName::FIXEDKENNEL:
        return routeToState(from, FSMStateName::FIXEDSTAND, plan) &&
               appendTransition(from, FSMStateName::FIXEDKENNEL, plan);
    case FSMStateName::FREESTAND:
        return routeToState(from, FSMStateName::FIXEDSTAND, plan) &&
               appendTransition(from, FSMStateName::FREESTAND, plan);
    case FSMStateName::TROTTING:
        if(from == FSMStateName::FIXEDSTAND ||
           from == FSMStateName::FREESTAND ||
           from == FSMStateName::MOVE_BASE){
            return appendTransition(from, FSMStateName::TROTTING, plan);
        }
        return routeToState(from, FSMStateName::FIXEDSTAND, plan) &&
               appendTransition(from, FSMStateName::TROTTING, plan);
    case FSMStateName::MOVE_BASE:
        return routeToState(from, FSMStateName::FIXEDSTAND, plan) &&
               appendTransition(from, FSMStateName::MOVE_BASE, plan);
    default:
        return false;
    }
}

bool FSM::appendTransition(FSMStateName &from, FSMStateName to, std::vector<ModePlanStep> &plan) const{
    UserCommand command = UserCommand::NONE;
    if(!commandForTransition(from, to, command)){
        publish_log("WARN", std::string("No transition route from ") + fsmStateToString(from) + " to " + fsmStateToString(to));
        return false;
    }

    plan.push_back({to, command});
    from = to;
    return true;
}

bool FSM::commandForTransition(FSMStateName from, FSMStateName to, UserCommand &command) const{
    command = UserCommand::NONE;

    if(to == FSMStateName::PASSIVE){
        switch(from){
        case FSMStateName::PASSIVE:
            command = UserCommand::NONE;
            return true;
        case FSMStateName::FIXEDSTAND:
        case FSMStateName::FIXEDKENNEL:
        case FSMStateName::FREESTAND:
        case FSMStateName::TROTTING:
        case FSMStateName::MOVE_BASE:
        case FSMStateName::BALANCETEST:
        case FSMStateName::SWINGTEST:
        case FSMStateName::STEPTEST:
            command = UserCommand::L2_B;
            return true;
        default:
            return false;
        }
    }

    if(to == FSMStateName::FIXEDSTAND){
        switch(from){
        case FSMStateName::PASSIVE:
        case FSMStateName::FIXEDKENNEL:
        case FSMStateName::FREESTAND:
        case FSMStateName::TROTTING:
        case FSMStateName::MOVE_BASE:
        case FSMStateName::BALANCETEST:
        case FSMStateName::SWINGTEST:
        case FSMStateName::STEPTEST:
            command = UserCommand::L2_A;
            return true;
        default:
            return false;
        }
    }

    if(from == FSMStateName::FIXEDSTAND && to == FSMStateName::FIXEDKENNEL){
        command = UserCommand::L2_X;
        return true;
    }
    if(from == FSMStateName::FIXEDSTAND && to == FSMStateName::FREESTAND){
        command = UserCommand::SELECT;
        return true;
    }
    if((from == FSMStateName::FIXEDSTAND ||
        from == FSMStateName::FREESTAND ||
        from == FSMStateName::MOVE_BASE) &&
       to == FSMStateName::TROTTING){
        command = UserCommand::START;
        return true;
    }
    if(from == FSMStateName::FIXEDSTAND && to == FSMStateName::MOVE_BASE){
        command = UserCommand::L1_X;
        return true;
    }

    return false;
}

void FSM::advanceModePlanIfReady(){
    if(_mode == FSMMode::CHANGE){
        if(_modePlanActive && _modePlanIndex < _modePlan.size()){
            _ctrlComp->lowState->userCmd = _modePlan[_modePlanIndex].command;
        }
        return;
    }

    if(_hasDeferredModeRequest && !isCriticalTransitionActive()){
        const uint8_t deferred = _deferredModeRequest;
        _hasDeferredModeRequest = false;
        startModePlan(deferred, _modePlanActive);
    }

    if(!_modePlanActive){
        return;
    }

    while(_modePlanIndex < _modePlan.size() &&
          _currentState &&
          _currentState->_stateName == _modePlan[_modePlanIndex].targetState &&
          _currentState->isReadyForModeTransition()){
        publish_log("INFO", std::string("Mode plan reached ") + fsmStateToString(_modePlan[_modePlanIndex].targetState));
        ++_modePlanIndex;
    }

    if(_modePlanIndex >= _modePlan.size()){
        _modePlanActive = false;
        publish_log("INFO", std::string("Completed mode plan: ") + highModeToString(_activeModeRequest));
        return;
    }

    const ModePlanStep &step = _modePlan[_modePlanIndex];
    _ctrlComp->lowState->userCmd = step.command;
}

bool FSM::isCriticalTransitionActive() const{
    if(_mode == FSMMode::CHANGE && _nextState){
        return _nextState->_stateName == FSMStateName::FIXEDSTAND ||
               _nextState->_stateName == FSMStateName::FIXEDKENNEL;
    }

    if(!_currentState){
        return false;
    }

    return (_currentState->_stateName == FSMStateName::FIXEDSTAND ||
            _currentState->_stateName == FSMStateName::FIXEDKENNEL) &&
           !_currentState->isReadyForModeTransition();
}

bool FSM::isInterruptMode(uint8_t requested) const{
    return requested == _STOP || requested == _PASSIVE;
}

const char *FSM::fsmStateToString(FSMStateName stateName) const{
    switch(stateName){
    case FSMStateName::INVALID: return "INVALID";
    case FSMStateName::PASSIVE: return "PASSIVE";
    case FSMStateName::FIXEDSTAND: return "FIXEDSTAND";
    case FSMStateName::FIXEDKENNEL: return "FIXEDKENNEL";
    case FSMStateName::FREESTAND: return "FREESTAND";
    case FSMStateName::TROTTING: return "TROTTING";
    case FSMStateName::MOVE_BASE: return "MOVE_BASE";
    case FSMStateName::BALANCETEST: return "BALANCETEST";
    case FSMStateName::SWINGTEST: return "SWINGTEST";
    case FSMStateName::STEPTEST: return "STEPTEST";
    default: return "UNKNOWN";
    }
}

const char *FSM::userCommandToString(UserCommand command) const{
    switch(command){
    case UserCommand::NONE: return "NONE";
    case UserCommand::START: return "START";
    case UserCommand::L2_A: return "L2_A";
    case UserCommand::L2_X: return "L2_X";
    case UserCommand::L2_B: return "L2_B";
    case UserCommand::SELECT: return "SELECT";
    case UserCommand::L1_X: return "L1_X";
    default: return "UNKNOWN";
    }
}

const char *FSM::highModeToString(uint8_t mode) const{
    switch(mode){
    case _PASSIVE: return "_PASSIVE";
    case _FREE_STAND_MODE: return "_FREE_STAND_MODE";
    case _FIXED_STAND_MODE: return "_FIXED_STAND_MODE";
    case _STAND_DOWN_MODE: return "_STAND_DOWN_MODE";
    case _VELOCITY_MODE: return "_VELOCITY_MODE";
    case _MOVE_BASE: return "_MOVE_BASE";
    case _START: return "_START";
    case _STOP: return "_STOP";
    default: return "UNKNOWN_MODE";
    }
}

std::shared_ptr<FSMState> FSM::getNextState(FSMStateName stateName){
    switch (stateName)
    {
    case FSMStateName::INVALID:
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:
        return _stateList.passive;
        break;
    case FSMStateName::FIXEDSTAND:
        return _stateList.fixedStand;
        break;
    case FSMStateName::FIXEDKENNEL:
        return _stateList.fixedKennel;
        break;
    case FSMStateName::FREESTAND:
        return _stateList.freeStand;
        break;
    case FSMStateName::TROTTING:
        return _stateList.trotting;
        break;
    case FSMStateName::BALANCETEST:
        return _stateList.invalid;
        break;
    case FSMStateName::SWINGTEST:
        return _stateList.invalid;
        break;
    case FSMStateName::STEPTEST:
        return _stateList.invalid;
        break;
    case FSMStateName::MOVE_BASE:
        return _stateList.moveBase;
        break;
    default:
        return _stateList.invalid;
        break;
    }
}

bool FSM::checkSafty(){
    // The angle with z axis less than 60 degree
    if(_ctrlComp->lowState->getRotMat()(2,2) < 0.5 ){
        publish_log("ERROR", "Robot is in an unsafe state!");
        return false;
    }
    return true;
}
