/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include <cstdint>
#include <memory>
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
void zeroCmdPanel(){if(cmdPanel){cmdPanel->setZero();}}
void setPassive(){if(cmdPanel){cmdPanel->setPassive();}}

protected:
std::shared_ptr<CmdPanel> cmdPanel;
};

#endif  //IOINTERFACE_H
