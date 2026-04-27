/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef CONTROLFRAME_H
#define CONTROLFRAME_H

#include <memory>
#include "FSM/FSM.h"
#include "control/CtrlComponents.h"

class ControlFrame{
public:
	ControlFrame(CtrlComponents *ctrlComp);
	~ControlFrame() = default;
	void run();
private:
	std::shared_ptr<FSM> _FSMController;
	CtrlComponents *_ctrlComp;
};

#endif  //CONTROLFRAME_H