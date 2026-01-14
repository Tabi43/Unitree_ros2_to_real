#ifndef UNITREE_ROS2_INTERFACE_HIGH_INTERFACE_NODE_HPP
#define UNITREE_ROS2_INTERFACE_HIGH_INTERFACE_NODE_HPP

#include <unordered_set>
#include <unordered_map>
#include <cstdint>
#include <utility>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <string>

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_msgs/msg/high_cmd.h"
#include "unitree_legged_msgs/msg/high_state.h"
#include "unitree_legged_msgs/msg/wireless_remote.h"
#include <unitree_ros2_interface/convert.h>
#include "unitree_ros2_interface/srv/set_high_mode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#define IDLE_MODE 0
#define FREE_STAND_MODE 1
#define VELOCITY_MODE 2
#define STAND_DOWN_MODE 5
#define STAND_UP_MODE 6
#define DAMPING_MODE 7
#define RECOVERY_MODE 8
#define START 10
#define STOP 20 

class HighInterface : public rclcpp::Node {
public:
  explicit HighInterface(
    const std::string & prefix,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~HighInterface() override;

  // HighCmd Send
  void highUdpSend() {
    // cmd_vel timeout check (use node clock)
    const auto now = this->now();
    const rclcpp::Duration dt = now - last_cmd_vel_time_;

    // rclcpp::Duration::seconds() exists on current ROS2 distros (Humble+),
    // but if you need maximum portability, compute via nanoseconds().
    const double dt_sec = dt.seconds();

    if (dt_sec > cmd_vel_timeout_) {
      if (high_cmd_.velocity[0] != 0.0 || high_cmd_.velocity[1] != 0.0 || high_cmd_.yawSpeed != 0.0) {
        // Throttle duration in ROS2 is integral milliseconds.
        // Use a local clock variable to avoid macro/lambda quirks on some releases.
        auto clock = this->get_clock();
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *clock, 1000,
          "No cmd_vel received for %.2f seconds, zeroing velocities for safety", dt_sec);

        high_cmd_.velocity[0] = 0.0;
        high_cmd_.velocity[1] = 0.0;
        high_cmd_.yawSpeed = 0.0;
      }
    }

    high_udp_.SetSend(high_cmd_);
    high_udp_.Send();
  }

  // HighState Recv
  void highUdpRecv() {
    high_udp_.Recv();
    high_udp_.GetRecv(high_state_);

    if (mode_ != high_state_.mode && !(wait_check_mode_)) {
        mode_ = high_state_.mode;
        publish_log("WARN", "Detected different mode on robot; actual mode set: " +
        to_string(static_cast<unsigned>(high_state_.mode)));
    }

    if(wait_check_count_ <= wait_check_window_ && wait_check_mode_) {
      wait_check_count_++;
    } else if (wait_check_mode_) {
      wait_check_mode_ = false;
      wait_check_count_ = 0;
    }
  }

  // Cmd Vel Callback
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // HighCmd Callback
  void highCmdCallback(const unitree_legged_msgs::msg::HighCmd::SharedPtr msg);

  // Publish Joint State
  void pubJointState();

  // Publish IMU
  void pubImu();

  // Publish Odometry
  void pubOdom();

  // HighMode service
  void setModeCallback(
    const std::shared_ptr<unitree_ros2_interface::srv::SetHighMode::Request> req,
    std::shared_ptr<unitree_ros2_interface::srv::SetHighMode::Response> res);

  // Mode macro thread
  bool launchModeMacro(const std::vector<std::pair<uint8_t, double>> & sequence);

  void declare_and_get_params();
  void validate_params_or_throw();

  static std::string normalize_ns(const std::string & ns);
  std::string make_topic(const std::string & suffix) const;  // suffix relative to <camera_name>
  void publish_log(const std::string & level, const std::string & msg);

  // Check if mode transition is allowed
  inline bool checkModeTransition(unsigned int new_mode) {
    auto it = allowed_transitions_.find(static_cast<uint8_t>(mode_));
    if (it == allowed_transitions_.end())
      return false;
    const auto & possible = it->second;
    return possible.count(static_cast<uint8_t>(new_mode)) > 0;
  }

  inline const char * modeToString(uint8_t mode) const {
    switch (mode) {
      case IDLE_MODE:        return "IDLE_MODE";
      case FREE_STAND_MODE:  return "FREE_STAND_MODE";
      case VELOCITY_MODE:    return "VELOCITY_MODE";
      case STAND_DOWN_MODE:  return "STAND_DOWN_MODE";
      case STAND_UP_MODE:    return "STAND_UP_MODE";
      case DAMPING_MODE:     return "DAMPING_MODE";
      case RECOVERY_MODE:    return "RECOVERY_MODE";
      case START:            return "START";
      case STOP:             return "STOP";
      default:               return "UNKNOWN_MODE";
    }
  }

  int legs_[4] = {
    UNITREE_LEGGED_SDK::FL_,
    UNITREE_LEGGED_SDK::FR_,
    UNITREE_LEGGED_SDK::RL_,
    UNITREE_LEGGED_SDK::RR_
  };

  int joints_[12] = {
    UNITREE_LEGGED_SDK::FL_0, UNITREE_LEGGED_SDK::FL_1, UNITREE_LEGGED_SDK::FL_2,
    UNITREE_LEGGED_SDK::FR_0, UNITREE_LEGGED_SDK::FR_1, UNITREE_LEGGED_SDK::FR_2,
    UNITREE_LEGGED_SDK::RL_0, UNITREE_LEGGED_SDK::RL_1, UNITREE_LEGGED_SDK::RL_2,
    UNITREE_LEGGED_SDK::RR_0, UNITREE_LEGGED_SDK::RR_1, UNITREE_LEGGED_SDK::RR_2
  };

  std::vector<std::pair<uint8_t, double>> start_seq_ = {
    {DAMPING_MODE,    0.5},
    {STAND_UP_MODE,   1.5},
    {FREE_STAND_MODE, 0.5},
    {VELOCITY_MODE,   0.0}
  };

  std::vector<std::pair<uint8_t, double>> stop_seq_ = {
    {FREE_STAND_MODE, 0.0},
    {STAND_UP_MODE,   0.5},
    {STAND_DOWN_MODE, 1.5},
    {DAMPING_MODE,    0.5},
    {IDLE_MODE,       0.0}
  };

private:
  // SDK
  UNITREE_LEGGED_SDK::UDP high_udp_;
  UNITREE_LEGGED_SDK::HighState high_state_{};
  UNITREE_LEGGED_SDK::HighCmd high_cmd_{};

  std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_udp_send_;
  std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_udp_recv_;
  std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_joint_state_;
  std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_imu_;
  std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_odom_; 

  // High Level Unitree Mode
  unsigned int mode_ = 0;

  // Allowed transitions (define contents in .cpp)
  static const std::unordered_set<uint8_t> allowed_modes_;
  static const std::unordered_map<uint8_t, std::unordered_set<uint8_t>> allowed_transitions_;

  std::atomic_bool macro_running_{false};
  std::mutex mode_mtx_;

  // ROS 2 pubs/subs/services
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_log_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<unitree_legged_msgs::msg::HighCmd>::SharedPtr high_cmd_sub_;

  rclcpp::Service<unitree_ros2_interface::srv::SetHighMode>::SharedPtr mode_service_;

  // Timers if you want periodic send/recv/publish loops
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr recv_timer_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  // Time / params
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
  double cmd_vel_timeout_{0.5};
  bool wait_check_mode_{false}; 
  int wait_check_window_{500};      // [tick]
  int wait_check_count_{0};

  // Cached msgs
  sensor_msgs::msg::JointState joint_state_msg_;
  sensor_msgs::msg::Imu imu_msg_;

  std::string namespace_param_{""};
  std::string joint_states_topic_;
  std::string imu_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string wireless_remote_topic_;
  std::string sdk_cmd_topic_;

  float dt_send_{0.001};
  float dt_recv_{0.001};
  float imu_frequency_{1000};                // [Hz]
  float joint_states_frequency_{500};        // [Hz]
  float remote_frequency_{10};               // [Hz]
  float odom_frequency_{100};                // [Hz]
};

#endif // UNITREE_ROS2_INTERFACE_HIGH_INTERFACE_NODE_HPP