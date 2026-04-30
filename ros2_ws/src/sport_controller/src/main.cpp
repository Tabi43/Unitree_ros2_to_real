/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>

#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "Gait/WaveGenerator.h"
#include "control/BalanceCtrl.h"
#include "interface/KeyBoard.h"
#include "interface/IOROS.h"
#include "common/Logger.h"

bool running = true;

// over watch the ctrl+c command
void ShutDown(int sig) {
    switch (sig) {
        case SIGINT:
            std::cout << "[WARN] stop the controller after SIGINT received" << std::endl;
            break;
        default:
            break;
    }
    running = false;
}

void setProcessScheduler() {
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1) {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main(int argc, char **argv) {
    /* set real-time process */
    setProcessScheduler();
    /* set the print format */
    std::cout << std::fixed << std::setprecision(3);

    // ROS 2 initialization
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sport_controller");
    init_publish_log(node);

    std::shared_ptr<IOInterface> ioInter;
    CtrlPlatform ctrlPlat;

    ioInter = std::make_shared<IOROS>(node);
    
    ctrlPlat = CtrlPlatform::REALROBOT;
    
    auto ctrlComp = std::make_shared<CtrlComponents>(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.001; // run at 1000hz
    ctrlComp->running = &running;

    ctrlComp->robotModel = std::make_shared<Go1Robot>();

    ctrlComp->waveGen = std::make_shared<WaveGenerator>(0.45, 0.5, Vec4(0, 0.5, 0.5, 0)); // Trot

    ctrlComp->geneObj();

    publish_log("INFO", "Control components initialized");

    ControlFrame ctrlFrame(ctrlComp.get());

    signal(SIGINT, ShutDown);

    while (running) {
        ctrlFrame.run();
    }

    return 0;
}
