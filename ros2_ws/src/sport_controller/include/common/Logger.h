/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef LOGGER_H
#define LOGGER_H

#include "rclcpp/rclcpp.hpp"
#include <string>

void init_publish_log(
    const rclcpp::Node::SharedPtr &node,
    const std::string &topic = "/unitree_go1/sport_controller/log");

void publish_log(const std::string &level, const std::string &msg);

#endif  // LOGGER_H
