/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef STATE_MOVE_BASE_H
#define STATE_MOVE_BASE_H

#include "FSM/State_Trotting.h"
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

class State_move_base : public State_Trotting{
public:
    State_move_base(CtrlComponents *ctrlComp);
    ~State_move_base(){}
    FSMStateName checkChange();
private:
    void getUserCmd();
    void initRecv();
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    rclcpp::Node::SharedPtr _nm;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmdSub;
    double _vx, _vy;
    double _wz;
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
    std::thread executor_thread;
};

#endif  // STATE_MOVE_BASE_H