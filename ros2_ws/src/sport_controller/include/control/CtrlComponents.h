/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOInterface.h"
#include "interface/CmdPanel.h"
#include "common/unitreeRobot.h"
#include "Gait/WaveGenerator.h"
#include "control/Estimator.h"
#include "control/BalanceCtrl.h"
#include <string>
#include <iostream>
#include <memory>

#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"
#endif  // COMPILE_DEBUG

struct CtrlComponents{
public:
    CtrlComponents(std::shared_ptr<IOInterface> ioInter):ioInter(ioInter){
        lowCmd = std::make_shared<LowlevelCmd>();
        lowState = std::make_shared<LowlevelState>();
        contact = std::make_shared<VecInt4>(0, 0, 0, 0);
        phase = std::make_shared<Vec4>(0.5, 0.5, 0.5, 0.5);
    }
    ~CtrlComponents() = default;
    
    std::shared_ptr<LowlevelCmd> lowCmd;
    std::shared_ptr<LowlevelState> lowState;
    std::shared_ptr<IOInterface> ioInter;
    std::shared_ptr<QuadrupedRobot> robotModel;
    std::shared_ptr<WaveGenerator> waveGen;
    std::shared_ptr<Estimator> estimator;
    std::shared_ptr<BalanceCtrl> balCtrl;

#ifdef COMPILE_DEBUG
    std::shared_ptr<PyPlot> plot;
#endif  // COMPILE_DEBUG

    std::shared_ptr<VecInt4> contact;
    std::shared_ptr<Vec4> phase;

    double dt;
    bool *running;
    CtrlPlatform ctrlPlatform;

    void sendRecv(){
        ioInter->sendRecv(lowCmd.get(), lowState.get());
    }

    void runWaveGen(){
        waveGen->calcContactPhase(*phase, *contact, _waveStatus);
    }

    void setAllStance(){
        _waveStatus = WaveStatus::STANCE_ALL;
    }

    void setAllSwing(){
        _waveStatus = WaveStatus::SWING_ALL;
    }

    void setStartWave(){
        _waveStatus = WaveStatus::WAVE_ALL;
    }

    void geneObj(){
        estimator = std::make_shared<Estimator>(robotModel.get(), lowState.get(), contact.get(), phase.get(), dt);
        balCtrl = std::make_shared<BalanceCtrl>(robotModel.get());

#ifdef COMPILE_DEBUG
        plot = std::make_shared<PyPlot>();
        balCtrl->setPyPlot(plot.get());
        estimator->setPyPlot(plot.get());
#endif  // COMPILE_DEBUG
    }

private:
    WaveStatus _waveStatus = WaveStatus::SWING_ALL;

};

#endif  // CTRLCOMPONENTS_H