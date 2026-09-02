/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include <cstdint>
#include <memory>
#include "common/enumClass.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/CmdPanel.h"
#include <string>

class IOInterface{
public:
IOInterface(){}
virtual ~IOInterface() = default;
virtual void sendRecv(const LowlevelCmd *cmd, LowlevelState *state) = 0;
virtual bool fetchModeRequest(uint8_t &mode) {(void)mode; return false;}

/**
 * @brief Current enable/disable gate of the controller, polled by the FSM every cycle.
 * @return The active ControllerState. Interfaces without a gate stay permanently ENABLED.
 */
virtual ControllerState controlState() const {return ControllerState::ENABLED;}

/**
 * @brief Called by the FSM once a DISABLING sequence has actually reached PASSIVE,
 *        so the interface can complete the transition to DISABLED.
 */
virtual void onDisableComplete() {}
void zeroCmdPanel(){if(cmdPanel){cmdPanel->setZero();}}
void setPassive(){if(cmdPanel){cmdPanel->setPassive();}}

protected:
std::shared_ptr<CmdPanel> cmdPanel;
};

#endif  //IOINTERFACE_H
