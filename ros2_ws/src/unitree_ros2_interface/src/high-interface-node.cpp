#include "unitree_ros2_interface/high-interface-node.hpp"

#include <chrono>
#include <thread>
#include <utility>

using std::placeholders::_1;
using std::placeholders::_2;

HighInterface::HighInterface(const std::string & prefix,
                             const rclcpp::NodeOptions & options)
: rclcpp::Node("high_interface", options),
  high_udp_(8090, "192.168.123.161", 8082, sizeof(high_cmd_), sizeof(high_state_)),
  prefix_(prefix) {
  // Publishers (QoS: KeepLast(1000) come la queue_size ROS1)
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(prefix_ + "/joint_state", rclcpp::QoS(1000));
  imu_pub_         = this->create_publisher<sensor_msgs::msg::Imu>(prefix_ + "/imu", rclcpp::QoS(1000));
  odom_pub_        = this->create_publisher<nav_msgs::msg::Odometry>(prefix_ + "/odom", rclcpp::QoS(1000));

  // Subscriber cmd_vel
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    prefix_ + "/cmd_vel",
    rclcpp::QoS(1),
    std::bind(&HighInterface::cmdVelCallback, this, std::placeholders::_1)
  );

  // Service
  mode_service_ = this->create_service<unitree_ros2_interface::srv::SetHighMode>(
    prefix_ + "/set_high_mode",
    std::bind(&HighInterface::setModeCallback, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Parametro safety timeout
  cmd_vel_timeout_ = this->declare_parameter<double>("cmd_vel_timeout", 1.0);
  last_cmd_vel_time_ = this->now();

  // Init mode
  mode_ = IDLE_MODE;
  high_cmd_.mode = IDLE_MODE;

  // Setup joint state message
  joint_state_msg_.name = {
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
  };

  joint_state_msg_.position.resize(12);
  joint_state_msg_.velocity.resize(12);
  joint_state_msg_.effort.resize(12);

  // Setup IMU message
  imu_msg_.header.frame_id = "imu_link";

  high_udp_.InitCmdData(high_cmd_);
}

HighInterface::~HighInterface() = default;

void HighInterface::setModeCallback(
  const std::shared_ptr<unitree_ros2_interface::srv::SetHighMode::Request> req,
  std::shared_ptr<unitree_ros2_interface::srv::SetHighMode::Response> res) {

  const uint8_t requested = static_cast<uint8_t>(req->mode);

  RCLCPP_INFO(this->get_logger(), "Received high mode: %u (%s)",
              static_cast<unsigned>(requested), modeToString(requested));

  if (!checkModeTransition(requested)) {
    RCLCPP_WARN(this->get_logger(), "Invalid mode transition from %u (%s) to %u (%s)",
                static_cast<unsigned>(mode_), modeToString(static_cast<uint8_t>(mode_)),
                static_cast<unsigned>(requested), modeToString(requested));
    res->res = false;
    return;
  }

  if (requested == START) {
    RCLCPP_INFO(this->get_logger(),
                "Starting mode macro: %u (%s) -> %u (%s)",
                static_cast<unsigned>(mode_), modeToString(static_cast<uint8_t>(mode_)),
                static_cast<unsigned>(VELOCITY_MODE), modeToString(VELOCITY_MODE));
    res->res = launchModeMacro(start_seq_);
    return;
  }

  if (requested == STOP) {
    RCLCPP_INFO(this->get_logger(),
                "Stopping mode macro: %u (%s) -> %u (%s)",
                static_cast<unsigned>(mode_), modeToString(static_cast<uint8_t>(mode_)),
                static_cast<unsigned>(IDLE_MODE), modeToString(IDLE_MODE));
    res->res = launchModeMacro(stop_seq_);
    return;
  }

  // Single transition
  {
    std::lock_guard<std::mutex> lk(mode_mtx_);
    mode_ = requested;
    high_cmd_.mode = requested;
  }

  RCLCPP_INFO(this->get_logger(), "Mode transition allowed -> %u (%s)",
              static_cast<unsigned>(requested), modeToString(requested));
  res->res = true;
}

void HighInterface::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (mode_ != VELOCITY_MODE) {
    // ROS2 throttle: period in milliseconds + clock :contentReference[oaicite:7]{index=7}
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Robot not in VELOCITY_MODE, ignoring cmd_vel");
    return;
  }

  // Assumo che la tua convert.hpp abbia rosMsg2Cmd(msg) anche in ROS2.
  // In caso contrario: adatta la firma (es. rosMsg2Cmd(*msg)).
  high_cmd_ = rosMsg2Cmd(*msg);

  last_cmd_vel_time_ = this->now();
}

void HighInterface::highCmdCallback(const unitree_legged_msgs::msg::HighCmd::SharedPtr msg) {
  high_cmd_ = rosMsg2Cmd(*msg);
}

void HighInterface::pubJointState() {
  for (int i = 0; i < 12; ++i) {
    joint_state_msg_.position[i] = high_state_.motorState[joints_[i]].q;
    joint_state_msg_.velocity[i] = high_state_.motorState[joints_[i]].dq;
    joint_state_msg_.effort[i]   = high_state_.motorState[joints_[i]].tauEst;
  }

  joint_state_msg_.header.stamp = this->now(); 
  joint_state_pub_->publish(joint_state_msg_);
}

void HighInterface::pubImu() {
  imu_msg_.header.stamp = this->now(); 

  imu_msg_.orientation.x = high_state_.imu.quaternion[1];
  imu_msg_.orientation.y = high_state_.imu.quaternion[2];
  imu_msg_.orientation.z = high_state_.imu.quaternion[3];
  imu_msg_.orientation.w = high_state_.imu.quaternion[0];

  imu_msg_.angular_velocity.x = high_state_.imu.gyroscope[0];
  imu_msg_.angular_velocity.y = high_state_.imu.gyroscope[1];
  imu_msg_.angular_velocity.z = high_state_.imu.gyroscope[2];

  imu_msg_.linear_acceleration.x = high_state_.imu.accelerometer[0];
  imu_msg_.linear_acceleration.y = high_state_.imu.accelerometer[1];
  imu_msg_.linear_acceleration.z = high_state_.imu.accelerometer[2];

  imu_pub_->publish(imu_msg_);
}

void HighInterface::pubOdom() {
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = this->now();
  odom.header.frame_id = "odom";

  odom.pose.pose.position.x = high_state_.position[0];
  odom.pose.pose.position.y = high_state_.position[1];
  odom.pose.pose.position.z = high_state_.position[2];

  odom.pose.pose.orientation.x = high_state_.imu.quaternion[1];
  odom.pose.pose.orientation.y = high_state_.imu.quaternion[2];
  odom.pose.pose.orientation.z = high_state_.imu.quaternion[3];
  odom.pose.pose.orientation.w = high_state_.imu.quaternion[0];

  odom.twist.twist.linear.x  = high_state_.velocity[0];
  odom.twist.twist.linear.y  = high_state_.velocity[1];
  odom.twist.twist.angular.z = high_state_.yawSpeed;

  odom_pub_->publish(odom);
}

bool HighInterface::launchModeMacro(const std::vector<std::pair<uint8_t, double>> & sequence) {
  
    if (macro_running_.exchange(true)) {
    RCLCPP_WARN(this->get_logger(), "A mode macro is already running. Rejecting new request.");
    return false;
  }

  std::thread([this, sequence]() {
    auto stop_macro = [this]() { macro_running_ = false; };

    for (const auto & step : sequence) {
      const uint8_t target   = step.first;
      const double  wait_sec = step.second;

      {
        std::lock_guard<std::mutex> lk(mode_mtx_);

        if (!checkModeTransition(target)) {
          RCLCPP_WARN(this->get_logger(),
                      "Macro aborted: invalid transition from %u (%s) to %u (%s)",
                      static_cast<unsigned>(mode_), modeToString(static_cast<uint8_t>(mode_)),
                      static_cast<unsigned>(target), modeToString(target));
          stop_macro();
          return;
        }

        // Importante: aggiorna mode_ per far funzionare le transizioni step-by-step
        mode_ = target;
        high_cmd_.mode = target;

        RCLCPP_INFO(this->get_logger(),
                    "Macro step -> %u (%s) [wait=%.2fs]",
                    static_cast<unsigned>(target), modeToString(target), wait_sec);
      }

      if (wait_sec > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(wait_sec));
      }
    }

    RCLCPP_INFO(this->get_logger(), "Mode macro completed.");
    stop_macro();
  }).detach();

  return true;
}

// -------------------- Static member definitions --------------------

// NB: niente "IDLE_MODE = 0" qui; usa i simboli
const std::unordered_set<uint8_t> HighInterface::allowed_modes_ = {
  IDLE_MODE,
  FREE_STAND_MODE,
  VELOCITY_MODE,
  STAND_DOWN_MODE,
  STAND_UP_MODE,
  DAMPING_MODE,
  RECOVERY_MODE,
  START,
  STOP
};

const std::unordered_map<uint8_t, std::unordered_set<uint8_t>> HighInterface::allowed_transitions_ = {
  { static_cast<uint8_t>(IDLE_MODE), {
      static_cast<uint8_t>(DAMPING_MODE),
      static_cast<uint8_t>(START)
  }},
  { static_cast<uint8_t>(FREE_STAND_MODE), {
      static_cast<uint8_t>(VELOCITY_MODE),
      static_cast<uint8_t>(STAND_UP_MODE),
      static_cast<uint8_t>(DAMPING_MODE)
  }},
  { static_cast<uint8_t>(VELOCITY_MODE), {
      static_cast<uint8_t>(FREE_STAND_MODE),
      static_cast<uint8_t>(DAMPING_MODE),
      static_cast<uint8_t>(STOP)
  }},
  { static_cast<uint8_t>(STAND_DOWN_MODE), {
      static_cast<uint8_t>(STAND_UP_MODE),
      static_cast<uint8_t>(DAMPING_MODE)
  }},
  { static_cast<uint8_t>(STAND_UP_MODE), {
      static_cast<uint8_t>(FREE_STAND_MODE),
      static_cast<uint8_t>(STAND_DOWN_MODE),
      static_cast<uint8_t>(DAMPING_MODE)
  }},
  { static_cast<uint8_t>(DAMPING_MODE), {
      static_cast<uint8_t>(IDLE_MODE),
      static_cast<uint8_t>(FREE_STAND_MODE),
      static_cast<uint8_t>(VELOCITY_MODE),
      static_cast<uint8_t>(STAND_DOWN_MODE),
      static_cast<uint8_t>(STAND_UP_MODE),
      static_cast<uint8_t>(DAMPING_MODE),
      static_cast<uint8_t>(RECOVERY_MODE)
  }},
  { static_cast<uint8_t>(RECOVERY_MODE), {
      static_cast<uint8_t>(DAMPING_MODE)
  }},
};
