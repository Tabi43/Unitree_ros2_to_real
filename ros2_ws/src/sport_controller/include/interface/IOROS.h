/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef IOROS_H
#define IOROS_H

#include "rclcpp/rclcpp.hpp"
#include "interface/IOInterface.h"
#include "unitree_ros2_interface/srv/set_high_mode.hpp"
#include "unitree_legged_msgs/msg/low_cmd.hpp"
#include "unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_msgs/msg/motor_cmd.hpp"
#include "unitree_legged_msgs/msg/motor_state.hpp"
#include "unitree_legged_msgs/msg/wireless_remote.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// Unitree legged robot joint index definition
constexpr int FR_ = 0;  // leg index
constexpr int FL_ = 1;
constexpr int RR_ = 2;
constexpr int RL_ = 3;

constexpr int FR_0 = 0;  // joint index
constexpr int FR_1 = 1;
constexpr int FR_2 = 2;

constexpr int FL_0 = 3;
constexpr int FL_1 = 4;
constexpr int FL_2 = 5;

constexpr int RR_0 = 6;
constexpr int RR_1 = 7;
constexpr int RR_2 = 8;

constexpr int RL_0 = 9;
constexpr int RL_1 = 10;
constexpr int RL_2 = 11;

class IOROS : public IOInterface{
public:
IOROS(rclcpp::Node::SharedPtr node_ptr);
~IOROS();
void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
bool fetchModeRequest(uint8_t &mode) override;

private:
static void RosShutDown(int sig);
void sendCmd(const LowlevelCmd *cmd);
void recvState(LowlevelState *state);
void setHighModeCallback(
    const std::shared_ptr<unitree_ros2_interface::srv::SetHighMode::Request> req,
    std::shared_ptr<unitree_ros2_interface::srv::SetHighMode::Response> res);
bool isValidHighMode(uint8_t mode) const;
void setPendingModeRequest(uint8_t mode);
const char *highModeToString(uint8_t mode) const;

rclcpp::Node::SharedPtr _nm;
rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_state_sub;
rclcpp::Subscription<unitree_legged_msgs::msg::WirelessRemote>::SharedPtr _remote_sub;
std::vector<rclcpp::Subscription<unitree_legged_msgs::msg::MotorState>::SharedPtr> _servo_sub;
std::vector<rclcpp::Publisher<unitree_legged_msgs::msg::MotorCmd>::SharedPtr> _servo_pub;
rclcpp::Publisher<unitree_legged_msgs::msg::LowCmd>::SharedPtr _lowCmd_pub;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _joint_cmd_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_log_;

rclcpp::Service<unitree_ros2_interface::srv::SetHighMode>::SharedPtr mode_service_;

unitree_legged_msgs::msg::LowCmd _lowCmd;
unitree_legged_msgs::msg::LowState _lowState;
std::string _robot_name;
std::unordered_map<std::string, int> joint_index_map;
int joints_map_normal2unitree[12] = {FL_0, FL_1, FL_2, FR_0, FR_1, FR_2, RL_0, RL_1, RL_2, RR_0, RR_1, RR_2};
std::thread executor_thread;

sensor_msgs::msg::JointState _joint_state;
std_msgs::msg::Float64MultiArray _joint_cmd;

bool useRemote = false;
UserCommand remoteUserCommand = UserCommand::NONE;
UserValue remoteUserValue;

std::mutex mode_request_mutex_;
bool has_pending_mode_request_ = false;
uint8_t pending_mode_request_ = _PASSIVE;

//repeated functions for multi-thread
void initRecv();
void initSend();

//Callback functions for ROS
void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

//Callback functions for ROS 2 interface
void initializeJointIndexMap();
void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
void remoteCallback(const unitree_legged_msgs::msg::WirelessRemote::SharedPtr msg);

void inline publish_log(const std::string & level, const std::string & msg) {
  const std::string full = "[" + level + "] " + msg;

  // ROS logger
  if (level == "ERROR") {
    RCLCPP_ERROR(_nm->get_logger(), "%s", msg.c_str());
  } else if (level == "WARN") {
    RCLCPP_WARN(_nm->get_logger(), "%s", msg.c_str());
  } else if (level == "DEBUG") {
    RCLCPP_DEBUG(_nm->get_logger(), "%s", msg.c_str());
  } else {
    RCLCPP_INFO(_nm->get_logger(), "%s", msg.c_str());
  }

  // Topic log
  if (pub_log_) {
    std_msgs::msg::String m;
    m.data = full;
    pub_log_->publish(m);
  }
}

};

#endif  // IOROS_H
