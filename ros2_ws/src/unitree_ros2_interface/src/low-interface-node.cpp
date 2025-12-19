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

InterfaceNode::InterfaceNode(const rclcpp::NodeOptions & options, float send_freq, float recv_freq)
: rclcpp::Node("interface_node", options), _lowlevel_udp(UNITREE_LEGGED_SDK::LOWLEVEL, 8091, "192.168.123.10", 8007), _safe(UNITREE_LEGGED_SDK::LeggedType::Go1) {
    // parameter `prefix` allows namespacing topics (similar to previous prefix arg)
    this->declare_parameter<std::string>("prefix", "");
    std::string prefix = this->get_parameter("prefix").as_string();

    // Set frequencies
    setSendFrequency(send_freq);
    setRecvFrequency(recv_freq);

    // Create the UDP send/receive loops using Unitree SDK (critical low-level communication)
    loop_udpSend = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("low_udp_send", dt_send, 3, boost::bind(&InterfaceNode::lowSend, this));
    loop_udpRecv = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("low_udp_recv", dt_recv, 3, boost::bind(&InterfaceNode::lowRecive, this));

    // Setup QoS profiles
    setQoSProfiles();

    // Create ROS2 subscription and publishers
    _lowCmd_sub = this->create_subscription<unitree_legged_msgs::msg::LowCmd>(prefix + "/low_cmd", *_lowcmd_qos,std::bind(&InterfaceNode::lowLevelCmdClbk, this, std::placeholders::_1));
    _joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>(prefix + "/joint_states", *_joint_state_qos);
    _imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(prefix + "/imu", *_imu_qos);
    _wrls_remote_pub = this->create_publisher<unitree_legged_msgs::msg::WirelessRemote>(prefix + "/remote", *_wrls_remote_qos);
    _log_pub = this->create_publisher<std_msgs::msg::String>(prefix + "/interface_log", 10);

    _FL_contact_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>(prefix + "/FL_foot/wrench", 10);
    _FR_contact_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>(prefix + "/FR_foot/wrench", 10);
    _RL_contact_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>(prefix + "/RL_foot/wrench", 10);
    _RR_contact_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>(prefix + "/RR_foot/wrench", 10);

    initServices();

    _state_timer = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&InterfaceNode::threadState, this)
    );

    _watchdog_timer = this->create_wall_timer(
        std::chrono::milliseconds(2),
        std::bind(&InterfaceNode::watchdog, this)
    );

    // Setup messages static headers
    _joint_state.name.resize(12);
    _joint_state.name = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
                        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

    _joint_state.position.resize(12);
    _joint_state.velocity.resize(12);
    _joint_state.effort.resize(12);

    // Setup IMU msg
    _imu.header.frame_id = "imu_link";

    // Initialize LowCmd buffer
    _lowlevel_udp.InitCmdData(_lowCmd_SDK);
    _lowCmd_buf.write(_lowCmd_SDK);

    // Initialize _lowState_SDK to prevent garbage data
    memset(&_lowState_SDK, 0, sizeof(_lowState_SDK));

    initLowCmd();

    // Initialize interface state
    _interface_state = InterfaceState::DISABLED;

    loop_udpSend->start();
    loop_udpRecv->start();

    RCLCPP_INFO(this->get_logger(), "Interface initialized in DISABLED state - waiting for enable command...");
    _log_pub->publish(std_msgs::msg::String().set__data("InterfaceNode initialized in DISABLED state, waiting for enable command..."));
}

void InterfaceNode::initServices() {
    // Create the SetBool service for enabling/disabling the interface
    set_enabled_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/enable_interface", 
        std::bind(&InterfaceNode::onSetEnabled, this, std::placeholders::_1, std::placeholders::_2)
    );

    get_status_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/get_interface_status",
        std::bind(&InterfaceNode::onGetStatus, this, std::placeholders::_1, std::placeholders::_2)
    );
}

void InterfaceNode::threadState() {
    std::lock_guard<std::mutex> lock(_state_mutex);

    rclcpp::Time timestamp = this->now();

    if(isEnabled()) {
        pubImu(_lowState_SDK, timestamp);
        pubJointsState(_lowState_SDK, timestamp);
        pubFeetContact(_lowState_SDK, timestamp);
        pubRemoteState(_lowState_SDK);
    } 
}

void InterfaceNode::watchdog() {
    // This function is called periodically to monitor the health of the interface.

    if (checkEmergencyCommand(_lowState_SDK)) {
        std::lock_guard<std::mutex> lock(_state_mutex);
        if(_interface_state != InterfaceState::EMERGENCY_STOP) {
            RCLCPP_ERROR(this->get_logger(), "Transitioning to EMERGENCY_STOP state!");
            _log_pub->publish(std_msgs::msg::String().set__data("Transitioning to EMERGENCY_STOP state!"));
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
                _log_pub->publish(std_msgs::msg::String().set__data("Interface enabled successfully."));
            } else {
                response->success = false;
                response->message = "Failed to send initial safe command. Interface not enabled.";
                RCLCPP_ERROR(this->get_logger(), "Failed to send initial safe command. Interface not enabled.");
                _log_pub->publish(std_msgs::msg::String().set__data("Failed to enable interface - communication error."));
            }
        } else {
            std::string current_state_str = stateToString(_interface_state);
            response->success = false;
            response->message = "Interface is not in DISABLED state. Current state: " + current_state_str;
            RCLCPP_WARN(this->get_logger(), "Interface enable request rejected - current state: %s", current_state_str.c_str());
            _log_pub->publish(std_msgs::msg::String().set__data("Interface enable request rejected - current state: " + current_state_str));
        }
    } else {
        // DISABLE REQUEST  
        if(_interface_state == InterfaceState::ENABLED || _interface_state == InterfaceState::ENABLING) {
            // Initiate safe disable sequence
            RCLCPP_WARN(this->get_logger(), "DISABLE REQUESTED - Initiating safe shutdown sequence...");
            _log_pub->publish(std_msgs::msg::String().set__data("DISABLE REQUESTED - Initiating safe shutdown sequence..."));
            
            // Immediately send a safe command
            if(sendSafeCommandImmediate()) {
                changeInterfaceState(InterfaceState::DISABLING);
                _disabling_safe_sends_count = 1; // We just sent one
                response->success = true;
                response->message = "Interface disable initiated. Safe commands being sent...";
                RCLCPP_INFO(this->get_logger(), "Interface disable initiated. Sending safe commands...");
                _log_pub->publish(std_msgs::msg::String().set__data("Interface disable initiated. Sending safe commands..."));
            } else {
                // If we can't send safe command, force emergency stop
                changeInterfaceState(InterfaceState::EMERGENCY_STOP);
                response->success = true;
                response->message = "Communication error during disable - EMERGENCY STOP activated.";
                RCLCPP_ERROR(this->get_logger(), "Communication error during disable - EMERGENCY STOP activated.");
                _log_pub->publish(std_msgs::msg::String().set__data("EMERGENCY STOP activated due to communication error."));
            }
        } else {
            std::string current_state_str = stateToString(_interface_state);
            response->success = false;
            response->message = "Interface is not in ENABLED state. Current state: " + current_state_str;
            RCLCPP_WARN(this->get_logger(), "Interface disable request rejected - current state: %s", current_state_str.c_str());
            _log_pub->publish(std_msgs::msg::String().set__data("Interface disable request rejected - current state: " + current_state_str));
        }
    }
}

InterfaceNode::~InterfaceNode() {

}

void InterfaceNode::pubRemoteState(UNITREE_LEGGED_SDK::LowState& state) {
    // Copy the remote data from the low state to the struct
    memcpy(&_remoteKeyData, &state.wirelessRemote[0], 40);

    // Kill the zero offset of analogs
    _remoteMsg.lx = killZeroOffset(_remoteKeyData.lx, 0.08);
    _remoteMsg.ly = killZeroOffset(_remoteKeyData.ly, 0.08);
    _remoteMsg.rx = killZeroOffset(_remoteKeyData.rx, 0.08);
    _remoteMsg.ry = killZeroOffset(_remoteKeyData.ry, 0.08);

    _remoteMsg.l1 = _remoteKeyData.btn.components.L1;
    _remoteMsg.l2 = _remoteKeyData.btn.components.L2;
    _remoteMsg.r1 = _remoteKeyData.btn.components.R1;
    _remoteMsg.r2 = _remoteKeyData.btn.components.R2;
    _remoteMsg.f1 = _remoteKeyData.btn.components.F1;
    _remoteMsg.f2 = _remoteKeyData.btn.components.F2;
    _remoteMsg.a = _remoteKeyData.btn.components.A;
    _remoteMsg.b = _remoteKeyData.btn.components.B;
    _remoteMsg.x = _remoteKeyData.btn.components.X;
    _remoteMsg.y = _remoteKeyData.btn.components.Y;
    _remoteMsg.up = _remoteKeyData.btn.components.up;
    _remoteMsg.down = _remoteKeyData.btn.components.down;
    _remoteMsg.left = _remoteKeyData.btn.components.left;
    _remoteMsg.right = _remoteKeyData.btn.components.right;

    _wrls_remote_pub->publish(_remoteMsg);
}

bool InterfaceNode::checkEmergencyCommand(UNITREE_LEGGED_SDK::LowState& state) {
    
    memcpy(&_remoteKeyData, &state.wirelessRemote[0], 40);

    if (_remoteKeyData.btn.components.L1 &&
        _remoteKeyData.btn.components.R1 &&
        _remoteKeyData.btn.components.L2 &&
        _remoteKeyData.btn.components.R2) {

            RCLCPP_WARN(this->get_logger(), "EMERGENCY COMMAND DETECTED");
            _log_pub->publish(std_msgs::msg::String().set__data("EMERGENCY COMMAND DETECTED")); 
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
    _joint_state.header.stamp = timestamp;

    for (int i = 0; i < 12; ++i) {
        _joint_state.position[i] = state.motorState[joints[i]].q;
        _joint_state.velocity[i] = state.motorState[joints[i]].dq;
        _joint_state.effort[i] = state.motorState[joints[i]].tauEst;
    }

    _joint_state_pub->publish(_joint_state);
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

    _FL_contact_pub->publish(FL_wrench);
    _FR_contact_pub->publish(FR_wrench);
    _RL_contact_pub->publish(RL_wrench);
    _RR_contact_pub->publish(RR_wrench);
}

void InterfaceNode::pubImu(UNITREE_LEGGED_SDK::LowState& state, rclcpp::Time& timestamp) {
    _imu.header.stamp = timestamp;

    _imu.orientation.x = state.imu.quaternion[1];
    _imu.orientation.y = state.imu.quaternion[2];
    _imu.orientation.z = state.imu.quaternion[3];
    _imu.orientation.w = state.imu.quaternion[0];

    _imu.angular_velocity.x = state.imu.gyroscope[0];
    _imu.angular_velocity.y = state.imu.gyroscope[1];
    _imu.angular_velocity.z = state.imu.gyroscope[2];

    _imu.linear_acceleration.x = state.imu.accelerometer[0];
    _imu.linear_acceleration.y = state.imu.accelerometer[1];
    _imu.linear_acceleration.z = state.imu.accelerometer[2];

    _imu_pub->publish(_imu);
}

void InterfaceNode::safetyStop() {
    std::lock_guard<std::mutex> lock(_state_mutex);
    
    RCLCPP_ERROR(this->get_logger(), "SAFETY STOP ACTIVATED - EMERGENCY PROTOCOL ENGAGED");
    _log_pub->publish(std_msgs::msg::String().set__data("SAFETY STOP ACTIVATED - EMERGENCY PROTOCOL ENGAGED"));
    
    // Immediately change state to emergency stop
    changeInterfaceState(InterfaceState::EMERGENCY_STOP);
    
    // Send multiple safe commands immediately to ensure robot safety
    for(int i = 0; i < 5; i++) {
        if(!sendSafeCommandImmediate(1)) {  // Only 1 retry for emergency
            RCLCPP_ERROR(this->get_logger(), "CRITICAL: Failed to send emergency safe command #%d", i+1);
        } else {
            RCLCPP_INFO(this->get_logger(), "Emergency safe command #%d sent successfully", i+1);
        }
        // Small delay between commands to ensure they are processed
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    
    RCLCPP_ERROR(this->get_logger(), "SAFETY STOP COMPLETE - Robot should be in safe state");
    _log_pub->publish(std_msgs::msg::String().set__data("SAFETY STOP COMPLETE - Robot should be in safe state"));
}

void InterfaceNode::initLowCmd() {
    for (int i = 0; i < 12; i++) {
        _lowCmd_SDK.motorCmd[i].mode = 10; // Servo mode
        _lowCmd_SDK.motorCmd[i].q = 0;
        _lowCmd_SDK.motorCmd[i].dq = 0;
        _lowCmd_SDK.motorCmd[i].Kp = 0;
        _lowCmd_SDK.motorCmd[i].Kd = 0;
        _lowCmd_SDK.motorCmd[i].tau = 0;
    }
}

void InterfaceNode::setQoSProfiles() {
    // Define QoS profiles for publishers and subscribers
    _imu_qos = std::make_shared<rclcpp::SensorDataQoS>();
    _joint_state_qos = std::make_shared<rclcpp::SensorDataQoS>();
    _wrls_remote_qos = std::make_shared<rclcpp::QoS>(rclcpp::QoS(10).reliable().durability_volatile());
    _lowcmd_qos = std::make_shared<rclcpp::QoS>(rclcpp::QoS(1).reliable().durability_volatile()
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
        
        RCLCPP_INFO(this->get_logger(), "Interface state changed: %s -> %s", 
                   old_state_str.c_str(), new_state_str.c_str());
        _log_pub->publish(std_msgs::msg::String().set__data(
            "Interface state changed: " + old_state_str + " -> " + new_state_str));
            
        // Reset disabling counter when entering disabling state
        if(new_state == InterfaceState::DISABLING) {
            _disabling_safe_sends_count = 0;
        }
    }
}

