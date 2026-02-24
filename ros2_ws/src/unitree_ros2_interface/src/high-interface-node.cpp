#include "unitree_ros2_interface/high-interface-node.hpp"

#include <chrono>
#include <thread>
#include <utility>

using std::placeholders::_1;
using std::placeholders::_2;

HighInterface::HighInterface(const std::string & prefix,
                             const rclcpp::NodeOptions & options)
: rclcpp::Node("high_interface", options),
  high_udp_(8090, "192.168.123.161", 8082, sizeof(high_cmd_), sizeof(high_state_)) {

  declare_and_get_params();
  
  pub_log_ = this->create_publisher<std_msgs::msg::String>(make_topic("high_interface_log"), rclcpp::QoS(1000));

  validate_params_or_throw();

  // Publishers (QoS: KeepLast(1000) come la queue_size ROS1)
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(make_topic(joint_states_topic_), rclcpp::QoS(1000));
  imu_pub_         = this->create_publisher<sensor_msgs::msg::Imu>(make_topic(imu_topic_), rclcpp::QoS(1000));
  odom_pub_        = this->create_publisher<nav_msgs::msg::Odometry>(make_topic(odom_topic_), rclcpp::QoS(1000));
  bms_pub_         = this->create_publisher<unitree_legged_msgs::msg::BmsState>(make_topic(bms_topic_), rclcpp::QoS(1000));

  // Subscriber cmd_vel
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    make_topic(cmd_vel_topic_),
    rclcpp::QoS(1),
    std::bind(&HighInterface::cmdVelCallback, this, std::placeholders::_1)
  );

  // Service
  mode_service_ = this->create_service<unitree_ros2_interface::srv::SetHighMode>(make_topic("set_high_mode"),
    std::bind(&HighInterface::setModeCallback, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Parametro safety timeout
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

  wait_check_window_ = std::ceil(0.5 * 1000 / dt_recv_);

  // Unitree SDK loops
  // If LoopFunc requires boost::bind in your SDK version, swap std::bind with boost::bind.
  loop_udp_send_ = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("high_udp_send", dt_send_, 3, std::bind(&HighInterface::highUdpSend, this));
  loop_udp_recv_ = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("high_udp_recv", dt_recv_, 3, std::bind(&HighInterface::highUdpRecv, this));
  loop_joint_state_ = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("joint_state", (1/joint_states_frequency_), std::bind(&HighInterface::pubJointState, this));
  loop_imu_ = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("imu", (1/imu_frequency_), std::bind(&HighInterface::pubImu, this));
  loop_odom_ = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("odom", (1/odom_frequency_), std::bind(&HighInterface::pubOdom, this));
  
  loop_udp_send_->start();
  loop_udp_recv_->start();
  loop_joint_state_->start();
  loop_imu_->start();
  loop_odom_->start();
  
  publish_log("INFO", "Unitree ROS2 High Interface started");
}

HighInterface::~HighInterface() = default;

void HighInterface::declare_and_get_params() {
    // Base
    this->declare_parameter<std::string>("namespace", "");

    // UDP
    this->declare_parameter<double>("dt_send", 0.001);
    this->declare_parameter<double>("dt_recv", 0.001);
    this->declare_parameter<std::string>("sdk_cmd_topic", "low_cmd");

    this->declare_parameter<double>("imu_frequency", 1000.0);
    this->declare_parameter<std::string>("imu_topic", "imu");
    this->declare_parameter<double>("joint_state_frequency", 500.0);
    this->declare_parameter<std::string>("joint_states_topic", "joint_states");
    this->declare_parameter<double>("remote_frequency", 10.0);
    this->declare_parameter<std::string>("wireless_remote_topic", "wireless_remote");
    this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    this->declare_parameter<std::string>("odom_topic", "odom");
    this->declare_parameter<double>("odom_frequency", 100.0);
    this->declare_parameter<double>("cmd_vel_timeout", 0.5);

    // Get parameters
    this->get_parameter("namespace", namespace_param_);
    this->get_parameter("sdk_cmd_topic", sdk_cmd_topic_);
    this->get_parameter("dt_send", dt_send_);
    this->get_parameter("dt_recv", dt_recv_);
    this->get_parameter("imu_frequency", imu_frequency_);
    this->get_parameter("imu_topic", imu_topic_);
    this->get_parameter("joint_state_frequency", joint_states_frequency_);
    this->get_parameter("joint_states_topic", joint_states_topic_);
    this->get_parameter("remote_frequency", remote_frequency_);
    this->get_parameter("wireless_remote_topic", wireless_remote_topic_);
    this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("odom_frequency", odom_frequency_);
    this->get_parameter("cmd_vel_timeout", cmd_vel_timeout_);
}

std::string HighInterface::make_topic(const std::string & suffix) const {
  // Desired convention: namespace/camera_name/(left|right)/image_raw
  const std::string desired = normalize_ns(namespace_param_);
  const std::string node_ns = this->get_namespace();  // "/" or "/unitree_go1"

  // If node already has a namespace, do NOT double-prefix.
  const bool node_has_ns = (node_ns != "/" && !node_ns.empty());
  const bool use_param_ns = (!desired.empty() && !node_has_ns);

  const std::string prefix = use_param_ns ? ("/" + desired + "/") : std::string("");
  return prefix + suffix;
}

std::string HighInterface::normalize_ns(const std::string & ns) {
  std::string out = ns;
  while (!out.empty() && out.front() == '/') out.erase(out.begin());
  while (!out.empty() && out.back() == '/') out.pop_back();
  return out;
}

void HighInterface::publish_log(const std::string & level, const std::string & msg) {
  const std::string full = "[" + level + "] " + msg;

  // ROS logger
  if (level == "ERROR") {
    RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
  } else if (level == "WARN") {
    RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
  } else if (level == "DEBUG") {
    RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
  }

  // Topic log
  std_msgs::msg::String m;
  m.data = full;
  pub_log_->publish(m);
}

void HighInterface::validate_params_or_throw() {
    if (imu_frequency_ <= 0.0) {
        throw std::runtime_error("Parameter 'imu_frequency' must be positive.");
    }
    if (joint_states_frequency_ <= 0.0) {
        throw std::runtime_error("Parameter 'joint_state_frequency' must be positive.");
    }
    if (remote_frequency_ <= 0.0) {
        throw std::runtime_error("Parameter 'remote_frequency' must be positive.");
    }
}

void HighInterface::setModeCallback(
  const std::shared_ptr<unitree_ros2_interface::srv::SetHighMode::Request> req,
  std::shared_ptr<unitree_ros2_interface::srv::SetHighMode::Response> res) {

  const uint8_t requested = static_cast<uint8_t>(req->mode);

  publish_log("INFO", "Received high mode request: " +
              std::to_string(static_cast<unsigned>(requested)) + " (" +
              modeToString(requested) + ")");

  if (!checkModeTransition(requested)) {
    publish_log("WARN", "Invalid mode transition from " +
                std::to_string(static_cast<unsigned>(mode_)) + " (" +
                modeToString(static_cast<uint8_t>(mode_)) + ") to " +
                std::to_string(static_cast<unsigned>(requested)) + " (" +
                modeToString(requested) + ")");
    res->res = false;
    return;
  }

  if (requested == START) {
    publish_log("INFO", "Starting mode macro: " +
                std::to_string(static_cast<unsigned>(mode_)) + " (" +
                modeToString(static_cast<uint8_t>(mode_)) + ") -> " +
                std::to_string(static_cast<unsigned>(VELOCITY_MODE)) + " (" +
                modeToString(VELOCITY_MODE) + ")");
    res->res = launchModeMacro(start_seq_);
    return;
  }

  if (requested == STOP) {
    publish_log("INFO", "Stopping mode macro: " +
                std::to_string(static_cast<unsigned>(mode_)) + " (" +
                modeToString(static_cast<uint8_t>(mode_)) + ") -> " +
                std::to_string(static_cast<unsigned>(IDLE_MODE)) + " (" +
                modeToString(IDLE_MODE) + ")");
    res->res = launchModeMacro(stop_seq_);
    return;
  }

  // Single transition
  mode_ = requested;
  high_cmd_.mode = requested;
  wait_check_mode_ = true;

  publish_log("INFO", "Mode transition allowed -> " +
              std::to_string(static_cast<unsigned>(requested)) + " (" +
              modeToString(requested) + ")");
  res->res = true;
}

void HighInterface::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (mode_ != VELOCITY_MODE) {
    // ROS2 throttle: period in milliseconds + clock 
    publish_log("WARN", "Robot not in VELOCITY_MODE, ignoring cmd_vel");
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

void HighInterface::pubBmsState() {
  bms_msg_.bms_status = high_state_.bms.bms_status;
  bms_msg_.cell_vol = high_state_.bms.cell_vol;
  bms_msg_.current = high_state_.bms.current;
  bms_msg_.cycle = high_state_.bms.cycle;
  bms_msg_.soc = high_state_.bms.SOC;

  bms_pub_->publish(bms_msg_);
}

bool HighInterface::launchModeMacro(const std::vector<std::pair<uint8_t, double>> & sequence) {
  
  if (macro_running_.exchange(true)) {
    publish_log("WARN", "A mode macro is already running. Rejecting new request.");
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
          publish_log("WARN", "Macro aborted: invalid transition from " +
                      std::to_string(static_cast<unsigned>(mode_)) + " (" +
                      modeToString(static_cast<uint8_t>(mode_)) + ") to " +
                      std::to_string(static_cast<unsigned>(target)) + " (" +
                      modeToString(target) + ")");
          stop_macro();
          return;
        }

        // Importante: aggiorna mode_ per far funzionare le transizioni step-by-step
        mode_ = target;
        high_cmd_.mode = target;
        wait_check_mode_ = true;

        publish_log("INFO", "Macro step -> " +
                    std::to_string(static_cast<unsigned>(target)) + " (" +
                    modeToString(target) + ") [wait=" + std::to_string(wait_sec) + "s]");
      }

      if (wait_sec > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(wait_sec));
      }
    }

    publish_log("INFO", "Mode macro completed.");
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
      static_cast<uint8_t>(DAMPING_MODE),
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
      static_cast<uint8_t>(RECOVERY_MODE),
      static_cast<uint8_t>(START)
  }},
  { static_cast<uint8_t>(RECOVERY_MODE), {
      static_cast<uint8_t>(DAMPING_MODE)
  }},
};
