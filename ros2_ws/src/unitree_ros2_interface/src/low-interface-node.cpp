#include "unitree_ros2_interface/low-interface-node.hpp"
#include <stdio.h>
#include <mutex>
#include <thread>
#include <chrono>

/*
    Converted to ROS2 (rclcpp). This implementation creates publishers/subscriptions
    using rclcpp APIs. The node accepts an optional parameter `prefix` to namespace
    topics (default empty string).

    UNITREE SDK INFO:
    Under !high-level! control:
    - initialize the target ip and port of udp as ip:192.168.123.161, port:8082

    Under !Low-level! control:
    - the target ip and port of the initialization udp are ip:192.168.123.10, port:8007
*/

InterfaceNode::InterfaceNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("unitree_low_level_interface", options), _lowlevel_udp(UNITREE_LEGGED_SDK::LOWLEVEL, 8091, "192.168.123.10", 8007), safe_(UNITREE_LEGGED_SDK::LeggedType::Go1) {
    
    declare_and_get_params();
    validate_params_or_throw();

    // Create the UDP send/receive loops using Unitree SDK (critical low-level communication)
    loop_udpSend = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("low_udp_send", dt_send_, 3, boost::bind(&InterfaceNode::lowSend, this));
    loop_udpRecv = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("low_udp_recv", dt_recv_, 3, boost::bind(&InterfaceNode::lowRecive, this));
    
    // Setup QoS profiles
    setQoSProfiles();

    // Create ROS2 subscription and publishers
    lowCmd_sub_ = this->create_subscription<unitree_legged_msgs::msg::LowCmd>(make_topic(sdk_cmd_topic_), *lowcmd_qos_,std::bind(&InterfaceNode::lowLevelCmdClbk, this, std::placeholders::_1));
    
    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(make_topic(joint_states_topic_), *joint_state_qos_);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(make_topic(imu_topic_), *imu_qos_);
    wireless_remote_pub_ = this->create_publisher<unitree_legged_msgs::msg::WirelessRemote>(make_topic(wireless_remote_topic_), *wireless_remote_qos_);
    FL_contact_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(make_topic("FL_foot/wrench"), 10);
    FR_contact_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(make_topic("FR_foot/wrench"), 10);
    RL_contact_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(make_topic("RL_foot/wrench"), 10);
    RR_contact_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(make_topic("RR_foot/wrench"), 10);
    pub_log_ = this->create_publisher<std_msgs::msg::String>(make_topic("log"), 10);
    
    initServices();

    state_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&InterfaceNode::threadState, this)
    );

    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2),
        std::bind(&InterfaceNode::watchdog, this)
    );

    // Setup messages static headers
    joint_states_msg_.name.resize(12);
    joint_states_msg_.name = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
                        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

    joint_states_msg_.position.resize(12);
    joint_states_msg_.velocity.resize(12);
    joint_states_msg_.effort.resize(12);
    
    // Setup IMU msg
    imu_msg_.header.frame_id = "imu_link";

    // Initialize LowCmd buffer
    _lowlevel_udp.InitCmdData(lowCmd_SDK_);
    _lowCmd_buf.write(lowCmd_SDK_);

    // Initialize _lowState_SDK to prevent garbage data
    memset(&lowState_SDK_, 0, sizeof(lowState_SDK_));

    initLowCmd();

    // Initialize interface state
    _interface_state = InterfaceState::DISABLED;

    loop_udpSend->start();
    loop_udpRecv->start();

    publish_log("INFO", "InterfaceNode initialized in DISABLED state, waiting for enable command...");
}

void InterfaceNode::declare_and_get_params() {
    // Base
    this->declare_parameter<std::string>("namespace", "");

    // UDP
    this->declare_parameter<double>("dt_send", 0.001);
    this->declare_parameter<double>("dt_recv", 0.001);
    this->declare_parameter<std::string>("sdk_cmd_topic", "/low_cmd");

    this->declare_parameter<double>("imu_frequency", 1000.0);
    this->declare_parameter<std::string>("imu_topic", "imu");
    this->declare_parameter<double>("joint_state_frequency", 500.0);
    this->declare_parameter<std::string>("joint_states_topic", "joint_states");
    this->declare_parameter<double>("remote_frequency", 10.0);
    this->declare_parameter<std::string>("wireless_remote_topic", "wireless_remote");

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
}

void InterfaceNode::validate_params_or_throw() {
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

std::string InterfaceNode::make_topic(const std::string & suffix) const {
  // Desired convention: namespace/camera_name/(left|right)/image_raw
  const std::string desired = normalize_ns(namespace_param_);
  const std::string node_ns = this->get_namespace();  // "/" or "/unitree_go1"

  // If node already has a namespace, do NOT double-prefix.
  const bool node_has_ns = (node_ns != "/" && !node_ns.empty());
  const bool use_param_ns = (!desired.empty() && !node_has_ns);

  const std::string prefix = use_param_ns ? ("/" + desired) : std::string("");
  return prefix + "/" + suffix;
}

std::string InterfaceNode::normalize_ns(const std::string & ns) {
  std::string out = ns;
  while (!out.empty() && out.front() == '/') out.erase(out.begin());
  while (!out.empty() && out.back() == '/') out.pop_back();
  return out;
}

void InterfaceNode::publish_log(const std::string & level, const std::string & msg) {
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

void InterfaceNode::initServices() {
    // Create the SetBool service for enabling/disabling the interface
    set_enabled_srv_ = this->create_service<std_srvs::srv::SetBool>(
        make_topic("enable_low_interface"), 
        std::bind(&InterfaceNode::onSetEnabled, this, std::placeholders::_1, std::placeholders::_2)
    );

    get_status_srv_ = this->create_service<std_srvs::srv::Trigger>(
        make_topic("get_status_low_interface"),
        std::bind(&InterfaceNode::onGetStatus, this, std::placeholders::_1, std::placeholders::_2)
    );
}

void InterfaceNode::threadState() {
    std::lock_guard<std::mutex> lock(_state_mutex);

    rclcpp::Time timestamp = this->now();

    if(isEnabled()) {
        pubImu(lowState_SDK_, timestamp);
        pubJointsState(lowState_SDK_, timestamp);
        pubFeetContact(lowState_SDK_, timestamp);
        pubRemoteState(lowState_SDK_);
    } 
}

void InterfaceNode::watchdog() {
    // This function is called periodically to monitor the health of the interface.

    if (checkEmergencyCommand(lowState_SDK_)) {
        std::lock_guard<std::mutex> lock(_state_mutex);
        if(_interface_state != InterfaceState::EMERGENCY_STOP) {
            RCLCPP_ERROR(this->get_logger(), "Transitioning to EMERGENCY_STOP state!");
            publish_log("ERROR", "Transitioning to EMERGENCY_STOP state!");
            changeInterfaceState(InterfaceState::EMERGENCY_STOP);
        }
    }

    // TODO: If the last cmd received timestamp is too old, consider transitioning to an emergency_stop state.

}

void InterfaceNode::onGetStatus(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    (void)request;  // Suppress unused parameter warning
    
    std::lock_guard<std::mutex> lock(_state_mutex);

    response->success = true;
    response->message = stateToString(_interface_state);
}

void InterfaceNode::onSetEnabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    std::lock_guard<std::mutex> lock(_state_mutex);
    
    if(request->data) {
        // ENABLE REQUEST
        if(_interface_state == InterfaceState::DISABLED) {
            // First send safe command to establish baseline
            if(sendSafeCommandImmediate()) {
                changeInterfaceState(InterfaceState::ENABLED);
                response->success = true;
                response->message = "Interface enabled successfully.";
                RCLCPP_INFO(this->get_logger(), "Interface enabled successfully.");
                publish_log("INFO", "Interface enabled successfully.");
            } else {
                response->success = false;
                response->message = "Failed to send initial safe command. Interface not enabled.";
                RCLCPP_ERROR(this->get_logger(), "Failed to send initial safe command. Interface not enabled.");
                publish_log("ERROR", "Failed to enable interface - communication error.");
            }
        } else {
            std::string current_state_str = stateToString(_interface_state);
            response->success = false;
            response->message = "Interface is not in DISABLED state. Current state: " + current_state_str;
            RCLCPP_WARN(this->get_logger(), "Interface enable request rejected - current state: %s", current_state_str.c_str());
            publish_log("WARN", "Interface enable request rejected - current state: " + current_state_str);
        }
    } else {
        // DISABLE REQUEST  
        if(_interface_state == InterfaceState::ENABLED || _interface_state == InterfaceState::ENABLING) {
            // Initiate safe disable sequence
            RCLCPP_WARN(this->get_logger(), "DISABLE REQUESTED - Initiating safe shutdown sequence...");
            publish_log("WARN", "DISABLE REQUESTED - Initiating safe shutdown sequence...");
            
            // Immediately send a safe command
            if(sendSafeCommandImmediate()) {
                changeInterfaceState(InterfaceState::DISABLING);
                _disabling_safe_sends_count = 1; // We just sent one
                response->success = true;
                response->message = "Interface disable initiated. Safe commands being sent...";
                RCLCPP_INFO(this->get_logger(), "Interface disable initiated. Sending safe commands...");
                publish_log("INFO", "Interface disable initiated. Sending safe commands...");
            } else {
                // If we can't send safe command, force emergency stop
                changeInterfaceState(InterfaceState::EMERGENCY_STOP);
                response->success = true;
                response->message = "Communication error during disable - EMERGENCY STOP activated.";
                publish_log("ERROR", "Communication error during disable - EMERGENCY STOP activated.");
            }
        } else {
            std::string current_state_str = stateToString(_interface_state);
            response->success = false;
            response->message = "Interface is not in ENABLED state. Current state: " + current_state_str;
            publish_log("WARN", "Interface disable request rejected - current state: " + current_state_str);
        }
    }
}

InterfaceNode::~InterfaceNode() {

}

void InterfaceNode::pubRemoteState(UNITREE_LEGGED_SDK::LowState& state) {
    // Copy the remote data from the low state to the struct
    memcpy(&_remoteKeyData, &state.wirelessRemote[0], 40);

    // Kill the zero offset of analogs
    remote_msg_.lx = killZeroOffset(_remoteKeyData.lx, 0.08);
    remote_msg_.ly = killZeroOffset(_remoteKeyData.ly, 0.08);
    remote_msg_.rx = killZeroOffset(_remoteKeyData.rx, 0.08);
    remote_msg_.ry = killZeroOffset(_remoteKeyData.ry, 0.08);

    remote_msg_.l1 = _remoteKeyData.btn.components.L1;
    remote_msg_.l2 = _remoteKeyData.btn.components.L2;
    remote_msg_.r1 = _remoteKeyData.btn.components.R1;
    remote_msg_.r2 = _remoteKeyData.btn.components.R2;
    remote_msg_.f1 = _remoteKeyData.btn.components.F1;
    remote_msg_.f2 = _remoteKeyData.btn.components.F2;
    remote_msg_.a = _remoteKeyData.btn.components.A;
    remote_msg_.b = _remoteKeyData.btn.components.B;
    remote_msg_.x = _remoteKeyData.btn.components.X;
    remote_msg_.y = _remoteKeyData.btn.components.Y;
    remote_msg_.up = _remoteKeyData.btn.components.up;
    remote_msg_.down = _remoteKeyData.btn.components.down;
    remote_msg_.left = _remoteKeyData.btn.components.left;
    remote_msg_.right = _remoteKeyData.btn.components.right;

    wireless_remote_pub_->publish(remote_msg_);
}

bool InterfaceNode::checkEmergencyCommand(UNITREE_LEGGED_SDK::LowState& state) {
    
    memcpy(&_remoteKeyData, &state.wirelessRemote[0], 40);

    if (_remoteKeyData.btn.components.L1 &&
        _remoteKeyData.btn.components.R1 &&
        _remoteKeyData.btn.components.L2 &&
        _remoteKeyData.btn.components.R2) {

            publish_log("ERROR", "EMERGENCY COMMAND RECEIVED VIA REMOTE");
            return true;
    }

    return false;
}

void InterfaceNode::lowLevelCmdClbk(const unitree_legged_msgs::msg::LowCmd::SharedPtr msg) {
    // Safety checks can be added here if needed
    // _lowCmd_SDK = rosMsg2Cmd(msg);
    _lowCmd_buf.write(rosMsg2Cmd(msg));
}

void InterfaceNode::pubJointsState(UNITREE_LEGGED_SDK::LowState& state, rclcpp::Time& timestamp) {
    joint_states_msg_.header.stamp = timestamp;

    for (int i = 0; i < 12; ++i) {
        joint_states_msg_.position[i] = state.motorState[joints_[i]].q;
        joint_states_msg_.velocity[i] = state.motorState[joints_[i]].dq;
        joint_states_msg_.effort[i] = state.motorState[joints_[i]].tauEst;
    }

    joint_states_pub_->publish(joint_states_msg_);
}

void InterfaceNode::pubFeetContact(UNITREE_LEGGED_SDK::LowState& state, rclcpp::Time& timestamp) {
    geometry_msgs::msg::WrenchStamped FL_wrench;
    geometry_msgs::msg::WrenchStamped FR_wrench;
    geometry_msgs::msg::WrenchStamped RL_wrench;
    geometry_msgs::msg::WrenchStamped RR_wrench;

    FL_wrench.header.stamp = timestamp;
    FR_wrench.header.stamp = timestamp;
    RL_wrench.header.stamp = timestamp;
    RR_wrench.header.stamp = timestamp;

    FL_wrench.wrench.force.z = state.footForce[UNITREE_LEGGED_SDK::FL_];
    FR_wrench.wrench.force.z = state.footForce[UNITREE_LEGGED_SDK::FR_];
    RL_wrench.wrench.force.z = state.footForce[UNITREE_LEGGED_SDK::RL_];
    RR_wrench.wrench.force.z = state.footForce[UNITREE_LEGGED_SDK::RR_];

    FL_contact_pub_->publish(FL_wrench);
    FR_contact_pub_->publish(FR_wrench);
    RL_contact_pub_->publish(RL_wrench);
    RR_contact_pub_->publish(RR_wrench);
}

void InterfaceNode::pubImu(UNITREE_LEGGED_SDK::LowState& state, rclcpp::Time& timestamp) {
    imu_msg_.header.stamp = timestamp;
    imu_msg_.orientation.x = state.imu.quaternion[1];
    imu_msg_.orientation.y = state.imu.quaternion[2];
    imu_msg_.orientation.z = state.imu.quaternion[3];
    imu_msg_.orientation.w = state.imu.quaternion[0];
    imu_msg_.angular_velocity.x = state.imu.gyroscope[0];
    imu_msg_.angular_velocity.y = state.imu.gyroscope[1];
    imu_msg_.angular_velocity.z = state.imu.gyroscope[2];
    imu_msg_.linear_acceleration.x = state.imu.accelerometer[0];
    imu_msg_.linear_acceleration.y = state.imu.accelerometer[1];
    imu_msg_.linear_acceleration.z = state.imu.accelerometer[2];

    imu_pub_->publish(imu_msg_);
}

void InterfaceNode::safetyStop() {
    std::lock_guard<std::mutex> lock(_state_mutex);
    
    publish_log("ERROR", "SAFETY STOP ACTIVATED - EMERGENCY PROTOCOL ENGAGED");
    
    // Immediately change state to emergency stop
    changeInterfaceState(InterfaceState::EMERGENCY_STOP);
    
    // Send multiple safe commands immediately to ensure robot safety
    for(int i = 0; i < 5; i++) {
        if(!sendSafeCommandImmediate(1)) {  // Only 1 retry for emergency
            publish_log("ERROR", "CRITICAL: Failed to send emergency safe command #" + std::to_string(i+1));
        } else {
            publish_log("INFO", "Emergency safe command #" + std::to_string(i+1) + " sent successfully");
        }
        // Small delay between commands to ensure they are processed
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    
    publish_log("ERROR", "SAFETY STOP COMPLETE - Robot should be in safe state");
}

void InterfaceNode::initLowCmd() {
    for (int i = 0; i < 12; i++) {
        lowCmd_SDK_.motorCmd[i].mode = 10; // Servo mode
        lowCmd_SDK_.motorCmd[i].q = 0;
        lowCmd_SDK_.motorCmd[i].dq = 0;
        lowCmd_SDK_.motorCmd[i].Kp = 0;
        lowCmd_SDK_.motorCmd[i].Kd = 0;
        lowCmd_SDK_.motorCmd[i].tau = 0;
    }
}

void InterfaceNode::setQoSProfiles() {
    // Define QoS profiles for publishers and subscribers
    imu_qos_ = std::make_shared<rclcpp::SensorDataQoS>();
    joint_state_qos_ = std::make_shared<rclcpp::SensorDataQoS>();
    wireless_remote_qos_ = std::make_shared<rclcpp::QoS>(rclcpp::QoS(10).reliable().durability_volatile());
    lowcmd_qos_ = std::make_shared<rclcpp::QoS>(rclcpp::QoS(1).reliable().durability_volatile()
                    .deadline(rclcpp::Duration::from_seconds(0.01))      // 10 ms
                    .lifespan(rclcpp::Duration::from_seconds(0.05)));    // 50 ms
}

UNITREE_LEGGED_SDK::LowCmd InterfaceNode::createSafeCommand() {
    UNITREE_LEGGED_SDK::LowCmd safe_cmd;
    
    // Initialize the command structure
    memset(&safe_cmd, 0, sizeof(safe_cmd));
    
    // Read current robot state to get current joint positions
    UNITREE_LEGGED_SDK::LowState current_state = _lowState_buf.read();
    
    // Configure each motor to hold current position with moderate stiffness
    for (int i = 0; i < 12; i++) {
        safe_cmd.motorCmd[i].mode = 10;  // Position control mode
        
        // Use current position if available, otherwise use zero
        if (current_state.motorState[i].q != 0.0 || 
            std::abs(current_state.motorState[i].q) > 0.001) {
            safe_cmd.motorCmd[i].q = current_state.motorState[i].q;  // Hold current position
        } else {
            safe_cmd.motorCmd[i].q = 0.0;  // Fallback to zero position
        }
        
        safe_cmd.motorCmd[i].dq = 0.0;        // Zero velocity
        safe_cmd.motorCmd[i].Kp = 20.0;       // Moderate position stiffness
        safe_cmd.motorCmd[i].Kd = 0.5;        // Light damping
        safe_cmd.motorCmd[i].tau = 0.0;       // Zero additional torque
    }
    
    return safe_cmd;
}

bool InterfaceNode::sendSafeCommandImmediate(int retries) {
    UNITREE_LEGGED_SDK::LowCmd safe_cmd = createSafeCommand();
    
    for(int attempt = 0; attempt < retries; attempt++) {
        try {
            _lowlevel_udp.SetSend(safe_cmd);
            _lowlevel_udp.Send();
            
            RCLCPP_INFO(this->get_logger(), "Safe command sent successfully (attempt %d/%d)", attempt+1, retries);
            return true;
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to send safe command (attempt %d/%d): %s", 
                       attempt+1, retries, e.what());
            
            if(attempt < retries - 1) {
                // Brief delay before retry
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }
    
    RCLCPP_ERROR(this->get_logger(), "CRITICAL: Failed to send safe command after %d attempts", retries);
    return false;
}

std::string InterfaceNode::stateToString(InterfaceState state) {
    switch(state) {
        case InterfaceState::DISABLED: return "DISABLED";
        case InterfaceState::ENABLING: return "ENABLING";
        case InterfaceState::ENABLED: return "ENABLED";
        case InterfaceState::DISABLING: return "DISABLING";
        case InterfaceState::EMERGENCY_STOP: return "EMERGENCY_STOP";
        default: return "UNKNOWN";
    }
}

void InterfaceNode::changeInterfaceState(InterfaceState new_state) {
    if(_interface_state != new_state) {
        InterfaceState old_state = _interface_state;
        _interface_state = new_state;
        
        // Log state change using cleaner method
        std::string old_state_str = stateToString(old_state);
        std::string new_state_str = stateToString(new_state);
        
        publish_log("INFO", "Interface state changed: " + old_state_str + " -> " + new_state_str);
            
        // Reset disabling counter when entering disabling state
        if(new_state == InterfaceState::DISABLING) {
            _disabling_safe_sends_count = 0;
        }
    }
}

