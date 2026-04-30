/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "common/Logger.h"

#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <mutex>

namespace {

std::mutex log_mutex;
rclcpp::Node::SharedPtr log_node;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub;

}  // namespace

void init_publish_log(const rclcpp::Node::SharedPtr &node, const std::string &topic) {
    if (!node) {
        return;
    }

    std::lock_guard<std::mutex> lock(log_mutex);
    log_node = node;
    log_pub = node->create_publisher<std_msgs::msg::String>(topic, 10);
}

void publish_log(const std::string &level, const std::string &msg) {
    const std::string full = "[" + level + "] " + msg;

    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    {
        std::lock_guard<std::mutex> lock(log_mutex);
        node = log_node;
        publisher = log_pub;
    }

    if (node) {
        if (level == "ERROR") {
            RCLCPP_ERROR(node->get_logger(), "%s", msg.c_str());
        } else if (level == "WARN") {
            RCLCPP_WARN(node->get_logger(), "%s", msg.c_str());
        } else if (level == "DEBUG") {
            RCLCPP_DEBUG(node->get_logger(), "%s", msg.c_str());
        } else {
            RCLCPP_INFO(node->get_logger(), "%s", msg.c_str());
        }
    } else {
        std::ostream &out = (level == "ERROR" || level == "WARN") ? std::cerr : std::cout;
        out << full << std::endl;
    }

    if (publisher) {
        std_msgs::msg::String log_msg;
        log_msg.data = full;
        publisher->publish(log_msg);
    }
}
