#include "unitree_ros2_interface/legged-sdk-interface.hpp"
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

LeggedSDKInterface::LeggedSDKInterface(const rclcpp::NodeOptions & options):
rclcpp::Node("legged_sdk_interface", options),
safe_(UNITREE_LEGGED_SDK::LeggedType::Go1),
lowlevel_udp_(UNITREE_LEGGED_SDK::LOWLEVEL, 8091, "192.168.123.10", 8007),
highlevel_udp_(8090, "192.168.123.161", 8082, sizeof(high_cmd_), sizeof(high_state_))  {

    pub_log_ = this->create_publisher<std_msgs::msg::String>(make_topic("legged_sdk/log"), 1000);

    declare_and_get_params();
    validate_params_or_throw();
    
    setQoSProfiles();
    
    // Initialize Timer for state monitoring and watchdog
    state_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&LeggedSDKInterface::threadState, this)
    );

    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2),
        std::bind(&LeggedSDKInterface::watchdog, this)
    );

    initServices();
    
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

    if(startup_mode_ == 1) {
        RCLCPP_INFO(this->get_logger(), "Startup mode set to HIGH - attempting to enable high-level interface...");
        if(enableHighInterface()) {
            RCLCPP_INFO(this->get_logger(), "Successfully enabled high-level interface on startup.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable high-level interface on startup. Check connection and parameters.");
        }
    } else if (startup_mode_ == 2) {
        RCLCPP_INFO(this->get_logger(), "Startup mode set to LOW - attempting to enable low-level interface...");
        if(enableLowInterface()) {
            RCLCPP_INFO(this->get_logger(), "Successfully enabled low-level interface on startup.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable low-level interface on startup. Check connection and parameters.");
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "Startup mode set to DISABLED - interfaces will not be enabled on startup.");
    }

};

LeggedSDKInterface::~LeggedSDKInterface() {}

void LeggedSDKInterface::declare_and_get_params() {
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
    this->declare_parameter<std::string>("bms_topic", "bms_state");
    this->declare_parameter<double>("soc_threshold", 20.0);
    this->declare_parameter<int>("startup_mode", 0);     // 0: DISABLED, 1: HIGH, 2: LOW

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
    this->get_parameter("bms_topic", bms_topic_);
    this->get_parameter("soc_threshold", soc_threshold_);
    this->get_parameter("startup_mode", startup_mode_);
}

void LeggedSDKInterface::validate_params_or_throw() {
    if (imu_frequency_ <= 0.0) {
        throw std::invalid_argument("imu_frequency must be > 0");
    }
    if (joint_states_frequency_ <= 0.0) {
        throw std::invalid_argument("joint_state_frequency must be > 0");
    }
    if (remote_frequency_ <= 0.0) {
        throw std::invalid_argument("remote_frequency must be > 0");
    }
    if (odom_frequency_ <= 0.0) {
        throw std::invalid_argument("odom_frequency must be > 0");
    }
    if (dt_send_ <= 0.0) {
        throw std::invalid_argument("dt_send must be > 0");
    }
    if (dt_recv_ <= 0.0) {
        throw std::invalid_argument("dt_recv must be > 0");
    }
    if (cmd_vel_timeout_ <= 0.0) {
        throw std::invalid_argument("cmd_vel_timeout must be > 0");
    }
    if (soc_threshold_ < 0.0 || soc_threshold_ > 100.0) {
        throw std::invalid_argument("soc_threshold must be between 0 and 100");
    }
    if (startup_mode_ < 0 || startup_mode_ > 2) {
        throw std::invalid_argument("startup_mode must be 0 (DISABLED), 1 (HIGH), or 2 (LOW)");
    }
    if (startup_mode_ == 2) {
        RCLCPP_WARN(this->get_logger(), "Startup mode set to LOW - the robot will attempt to enable the low-level interface on startup. Make sure this is intentional!");
    } else if (startup_mode_ == 1) {
        RCLCPP_WARN(this->get_logger(), "Startup mode set to HIGH - the robot will attempt to enable the high-level interface on startup. Make sure this is intentional!");
    }
}

std::string LeggedSDKInterface::make_topic(const std::string & suffix) const {
  // Desired convention: namespace/camera_name/(left|right)/image_raw
  const std::string desired = normalize_ns(namespace_param_);
  const std::string node_ns = this->get_namespace();  // "/" or "/unitree_go1"

  // If node already has a namespace, do NOT double-prefix.
  const bool node_has_ns = (node_ns != "/" && !node_ns.empty());
  const bool use_param_ns = (!desired.empty() && !node_has_ns);

  const std::string prefix = use_param_ns ? ("/" + desired + "/") : std::string("");
  return prefix + suffix;
}

std::string LeggedSDKInterface::normalize_ns(const std::string & ns) {
  std::string out = ns;
  while (!out.empty() && out.front() == '/') out.erase(out.begin());
  while (!out.empty() && out.back() == '/') out.pop_back();
  return out;
}

void LeggedSDKInterface::publish_log(const std::string & level, const std::string & msg) {
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

void LeggedSDKInterface::initServices() {
    // Create the SetBool service for enabling/disabling the interface
    set_enable_low_srv_ = this->create_service<std_srvs::srv::SetBool>(
        make_topic("legged_sdk/enable_low"), 
        std::bind(&LeggedSDKInterface::onSetLowEnable, this, std::placeholders::_1, std::placeholders::_2)
    );

    get_status_low_srv_ = this->create_service<std_srvs::srv::Trigger>(
        make_topic("legged_sdk/get_status_low"),
        std::bind(&LeggedSDKInterface::onGetStatus, this, std::placeholders::_1, std::placeholders::_2)
    );

    set_enable_high_srv_ = this->create_service<std_srvs::srv::SetBool>(
        make_topic("legged_sdk/enable_high"), 
        std::bind(&LeggedSDKInterface::onSetHighEnable, this, std::placeholders::_1, std::placeholders::_2)
    );

    get_status_high_srv_ = this->create_service<std_srvs::srv::Trigger>(
        make_topic("legged_sdk/get_status_high"),
        std::bind(&LeggedSDKInterface::onGetStatus, this, std::placeholders::_1, std::placeholders::_2)
    );
}

bool LeggedSDKInterface::enableLowInterface() {

    if (!isDisabled()) {
        RCLCPP_WARN(this->get_logger(), "Interface not in DISABLED state - cannot enable low interface!");
        publish_log("WARN", "Interface not in DISABLED state - cannot enable low interface!");
        return false;
    }

    // Guard against the window where state is DISABLED but the previous interface's
    // resources have not been cleaned up yet (pending flag set, threadState not fired).
    if (pending_low_cleanup_ || pending_high_cleanup_) {
        RCLCPP_WARN(this->get_logger(), "Cleanup of previous interface still pending - cannot enable low interface!");
        publish_log("WARN", "Cleanup of previous interface still pending - cannot enable low interface!");
        return false;
    }

    // Create the UDP send/receive loops using Unitree SDK (critical low-level communication)
    loop_udpSend = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("low_udp_send", dt_send_, 3, boost::bind(&LeggedSDKInterface::lowSend, this));
    loop_udpRecv = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("low_udp_recv", dt_recv_, 3, boost::bind(&LeggedSDKInterface::lowRecive, this));

    lowCmd_sub_ = this->create_subscription<unitree_legged_msgs::msg::LowCmd>(make_topic(sdk_cmd_topic_), *lowcmd_qos_,std::bind(&LeggedSDKInterface::lowLevelCmdClbk, this, std::placeholders::_1));
    
    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(make_topic(joint_states_topic_), rclcpp::QoS(1000));
    imu_pub_         = this->create_publisher<sensor_msgs::msg::Imu>(make_topic(imu_topic_), rclcpp::QoS(1000));
    wireless_remote_pub_ = this->create_publisher<unitree_legged_msgs::msg::WirelessRemote>(make_topic(wireless_remote_topic_), *wireless_remote_qos_);
    FL_contact_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(make_topic("FL_foot/wrench"), 10);
    FR_contact_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(make_topic("FR_foot/wrench"), 10);
    RL_contact_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(make_topic("RL_foot/wrench"), 10);
    RR_contact_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(make_topic("RR_foot/wrench"), 10);
    bms_pub_ = this->create_publisher<unitree_legged_msgs::msg::BmsState>(make_topic(bms_topic_), 10);

    // Initialize LowCmd buffer
    lowlevel_udp_.InitCmdData(lowCmd_SDK_);
    lowCmd_buf_.write(lowCmd_SDK_);

    // Initialize _lowState_SDK to prevent garbage data
    memset(&lowState_SDK_, 0, sizeof(lowState_SDK_));

    initLowCmd();

    loop_udpSend->start();
    loop_udpRecv->start();

    changeInterfaceState(InterfaceState::ENABLED_LOW);

    return true;
}

bool LeggedSDKInterface::enableHighInterface() {

    if (!isDisabled()) {
        RCLCPP_WARN(this->get_logger(), "Interface not in DISABLED state - cannot enable high interface!");
        publish_log("WARN", "Interface not in DISABLED state - cannot enable high interface!");
        return false;
    }
    if (pending_low_cleanup_ || pending_high_cleanup_) {
        RCLCPP_WARN(this->get_logger(), "Cleanup of previous interface still pending - cannot enable high interface!");
        publish_log("WARN", "Cleanup of previous interface still pending - cannot enable high interface!");
        return false;
    }

    loop_udpSend = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("high_udp_send", dt_send_, 3, std::bind(&LeggedSDKInterface::highUdpSend, this));
    loop_udpRecv = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("high_udp_recv", dt_recv_, 3, std::bind(&LeggedSDKInterface::highUdpRecv, this));

    // Publishers (QoS: KeepLast(1000) come la queue_size ROS1)
    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(make_topic(joint_states_topic_), rclcpp::QoS(1000));
    imu_pub_         = this->create_publisher<sensor_msgs::msg::Imu>(make_topic(imu_topic_), rclcpp::QoS(1000));
    odom_pub_        = this->create_publisher<nav_msgs::msg::Odometry>(make_topic(odom_topic_), rclcpp::QoS(1000));
    bms_pub_         = this->create_publisher<unitree_legged_msgs::msg::BmsState>(make_topic(bms_topic_), rclcpp::QoS(1000));
    wireless_remote_pub_ = this->create_publisher<unitree_legged_msgs::msg::WirelessRemote>(make_topic(wireless_remote_topic_), *wireless_remote_qos_);
    FL_contact_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(make_topic("FL_foot/wrench"), 10);
    FR_contact_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(make_topic("FR_foot/wrench"), 10);
    RL_contact_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(make_topic("RL_foot/wrench"), 10);
    RR_contact_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(make_topic("RR_foot/wrench"), 10);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        make_topic(cmd_vel_topic_),
        rclcpp::QoS(1),
        std::bind(&LeggedSDKInterface::cmdVelCallback, this, std::placeholders::_1)
    );

    // Service
    mode_service_ = this->create_service<unitree_ros2_interface::srv::SetHighMode>(make_topic("legged_sdk/set_high_mode"),
        std::bind(&LeggedSDKInterface::setHighModeCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Parametro safety timeout
    last_cmd_vel_time_ = this->now();

    // Init mode
    high_mode_ = IDLE_MODE;
    high_cmd_.mode = IDLE_MODE;

    highlevel_udp_.InitCmdData(high_cmd_);

    wait_check_window_ = std::ceil(0.5 * 1000 / dt_recv_);

    loop_udpSend->start();
    loop_udpRecv->start();

    changeInterfaceState(InterfaceState::ENABLED_HIGH);

    return true;
}

bool LeggedSDKInterface::disableLowInterface() {
    // Schedule cleanup on the ROS2 timer thread (threadState).
    // Never reset LoopFunc shared_ptrs here: this method may be called
    // from user code whose thread context is unknown.
    pending_low_cleanup_ = true;
    changeInterfaceState(InterfaceState::DISABLING_LOW);
    return true;
}

bool LeggedSDKInterface::disableHighInterface() {
    // Same deferred-cleanup pattern as disableLowInterface.
    pending_high_cleanup_ = true;
    changeInterfaceState(InterfaceState::DISABLING_HIGH);
    return true;
}

void LeggedSDKInterface::cleanupLowResources() {
    // Stop and destroy UDP LoopFunc threads first.
    // reset() blocks until the thread joins, which is fast because lowSend/lowRecive
    // early-return when interface_state_ == DISABLED.
    if (loop_udpSend) { loop_udpSend.reset(); }
    if (loop_udpRecv) { loop_udpRecv.reset(); }

    // Release all low-interface ROS2 entities.
    lowCmd_sub_.reset();
    joint_states_pub_.reset();
    imu_pub_.reset();
    wireless_remote_pub_.reset();
    FL_contact_pub_.reset();
    FR_contact_pub_.reset();
    RL_contact_pub_.reset();
    RR_contact_pub_.reset();
    bms_pub_.reset();

    publish_log("INFO", "Low interface resources released.");
}

void LeggedSDKInterface::cleanupHighResources() {
    // Stop and destroy UDP LoopFunc threads.
    // highUdpSend/highUdpRecv early-return when !isEnabledHigh().
    if (loop_udpSend) { loop_udpSend.reset(); }
    if (loop_udpRecv) { loop_udpRecv.reset(); }

    // Release all high-interface ROS2 entities.
    cmd_vel_sub_.reset();
    high_cmd_sub_.reset();
    joint_states_pub_.reset();
    imu_pub_.reset();
    odom_pub_.reset();
    bms_pub_.reset();
    wireless_remote_pub_.reset();
    FL_contact_pub_.reset();
    FR_contact_pub_.reset();
    RL_contact_pub_.reset();
    RR_contact_pub_.reset();
    mode_service_.reset();

    publish_log("INFO", "High interface resources released.");
}

void LeggedSDKInterface::threadState() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Consume pending cleanup flags. Cleanup is always performed here (ROS2 timer
    // thread) so that LoopFunc objects are never destroyed from within their own
    // callback, which would be undefined behaviour.
    if(isDisabled() && pending_low_cleanup_) {
        cleanupLowResources();
        pending_low_cleanup_.exchange(false);
    }
    if(isDisabled() && pending_high_cleanup_) {
        cleanupHighResources();
        pending_high_cleanup_.exchange(false);
    }

    rclcpp::Time timestamp = this->now();

    if(isEnabledLow()) {
        pubImu(lowState_SDK_.imu, timestamp);
        pubJointsState(lowState_SDK_.motorState, timestamp);
        pubFeetContact(lowState_SDK_.footForce, timestamp);
        pubRemoteState(lowState_SDK_.wirelessRemote);
        pubBmsState(lowState_SDK_.bms);
    } else if(isEnabledHigh()) {
        pubImu(high_state_.imu, timestamp);
        pubJointsState(high_state_.motorState, timestamp);
        pubFeetContact(high_state_.footForce, timestamp);
        pubRemoteState(high_state_.wirelessRemote);
        pubBmsState(high_state_.bms);
        pubOdom();
    }
}

void LeggedSDKInterface::watchdog() {
    // This function is called periodically to monitor the health of the interface.

    // Check the State Of Charge of the battery
    // if(isEnabledLow() && lowState_SDK_.bms.SOC <= soc_threshold_) {
    //     std::lock_guard<std::mutex> lock(state_mutex_);
    //     if(interface_state_ != InterfaceState::EMERGENCY_STOP_LOW) {
    //         RCLCPP_ERROR(this->get_logger(), "Battery SOC critically low (%d %%) - Transitioning to EMERGENCY_STOP_LOW state!", lowState_SDK_.bms.SOC);
    //         publish_log("ERROR", "Battery SOC critically low (" + std::to_string(lowState_SDK_.bms.SOC) + "%%) - Transitioning to EMERGENCY_STOP_LOW state!");
    //         changeInterfaceState(InterfaceState::EMERGENCY_STOP_LOW);
    //     }
    // }

    // if(isEnabledHigh() && high_state_.bms.SOC <= soc_threshold_) {
    //     std::lock_guard<std::mutex> lock(state_mutex_);
    //     if(interface_state_ != InterfaceState::EMERGENCY_STOP_HIGH) {
    //         RCLCPP_ERROR(this->get_logger(), "Battery SOC critically low (%d %%) - Transitioning to EMERGENCY_STOP_HIGH state!", high_state_.bms.SOC);
    //         publish_log("ERROR", "Battery SOC critically low (" + std::to_string(high_state_.bms.SOC) + "%%) - Transitioning to EMERGENCY_STOP_HIGH state!");
    //         changeInterfaceState(InterfaceState::EMERGENCY_STOP_HIGH);
    //     }
    // }

    // TODO: The emergency command should work even for the high-level interface.
    if (checkEmergencyCommand(lowState_SDK_.wirelessRemote) && isEnabledLow())  {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (interface_state_ != InterfaceState::EMERGENCY_STOP_LOW) {
            RCLCPP_ERROR(this->get_logger(), "Emergency stop command received from remote - Transitioning to EMERGENCY_STOP_LOW state!");
            publish_log("ERROR", "Emergency stop command received from remote - Transitioning to EMERGENCY_STOP_LOW state!");
            safetyLowStop();
        }
    }

    // TODO: If the last cmd received timestamp is too old, consider transitioning to an emergency_stop state.

}

void LeggedSDKInterface::onGetStatus(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    (void)request;  // Suppress unused parameter warning
    
    std::lock_guard<std::mutex> lock(state_mutex_);

    response->success = true;
    response->message = stateToString(interface_state_);
}

void LeggedSDKInterface::onSetLowEnable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if(request->data) {
        // ENABLE REQUEST
        if(interface_state_ == InterfaceState::DISABLED) {
            // First send safe command to establish baseline
            if(!enableLowInterface()){
                response->success = false;
                response->message = "Failed to enable low interface.";
                RCLCPP_ERROR(this->get_logger(), "Failed to enable low interface.");
                publish_log("ERROR", "Failed to enable low interface.");
                return;
            }
            if(sendSafeLowCommandImmediate()) {
                response->success = true;
                response->message = "Low Interface enabled successfully.";
                publish_log("INFO", "Low Interface enabled successfully.");
            } else {
                cleanupLowResources();
                response->success = false;
                response->message = "Failed to send initial safe command. Interface not enabled.";
                publish_log("ERROR", "Failed to enable low interface - communication error.");
            }
        } else {
            std::string current_state_str = stateToString(interface_state_);
            response->success = false;
            response->message = "Low Interface is not in DISABLED state. Current state: " + current_state_str;
            publish_log("WARN", "Low Interface enable request rejected - current state: " + current_state_str);
        }
    } else {
        // DISABLE REQUEST  
        if(interface_state_ == InterfaceState::ENABLED_LOW || interface_state_ == InterfaceState::ENABLING_LOW) {
            // Initiate safe disable sequence
            publish_log("WARN", "DISABLE REQUESTED - Initiating safe shutdown sequence...");
            
            // Immediately send a safe command
            if(sendSafeLowCommandImmediate(3)) {
                disableLowInterface();
                _disabling_safe_sends_count = 1; // We just sent one
                response->success = true;
                response->message = "Low Interface disable initiated. Safe commands being sent...";
                publish_log("INFO", "Low Interface disable initiated. Sending safe commands...");
            } else {
                // If we can't send safe command, force emergency stop
                changeInterfaceState(InterfaceState::EMERGENCY_STOP_LOW);
                response->success = false;
                response->message = "Communication error during disable - EMERGENCY STOP activated.";
                publish_log("ERROR", "Communication error during disable - EMERGENCY STOP activated.");
            }
        } else {
            std::string current_state_str = stateToString(interface_state_);
            response->success = false;
            response->message = "Low Interface is not in ENABLED state. Current state: " + current_state_str;
            publish_log("WARN", "Low Interface disable request rejected - current state: " + current_state_str);
        }
    }
}

void LeggedSDKInterface::onSetHighEnable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

    std::lock_guard<std::mutex> lock(state_mutex_);

    if (request->data) {
        // ENABLE REQUEST
        if (interface_state_ == InterfaceState::DISABLED) {
            if (!enableHighInterface()) {
                response->success = false;
                response->message = "Failed to enable high interface.";
                RCLCPP_ERROR(this->get_logger(), "Failed to enable high interface.");
                publish_log("ERROR", "Failed to enable high interface.");
                return;
            }
            response->success = true;
            response->message = "High interface enabled successfully.";           
            publish_log("INFO", "High interface enabled successfully.");
        } else {
            std::string current_state_str = stateToString(interface_state_);
            response->success = false;
            response->message = "Interface is not in DISABLED state. Current state: " + current_state_str;            
            publish_log("WARN", "High interface enable request rejected - current state: " + current_state_str);
        }
    } else {
        // DISABLE REQUEST
        if (interface_state_ == InterfaceState::ENABLED_HIGH || interface_state_ == InterfaceState::ENABLING_HIGH) {
            publish_log("WARN", "DISABLE HIGH REQUESTED - Initiating shutdown...");
            disableHighInterface();
            response->success = true;
            response->message = "High interface shutdown initiated...";
        } else {
            std::string current_state_str = stateToString(interface_state_);
            response->success = false;
            response->message = "High interface is not in ENABLED state. Current state: " + current_state_str;
            publish_log("WARN", "High interface disable request rejected - current state: " + current_state_str);
        }
    }
}

void LeggedSDKInterface::pubRemoteState(std::array<uint8_t, 40>& remote_data) {
    // Copy the remote data from the low state to the struct
    memcpy(&_remoteKeyData, &remote_data[0], 40);

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

void LeggedSDKInterface::pubJointsState(std::array<UNITREE_LEGGED_SDK::MotorState, 20>& motorState, rclcpp::Time& timestamp) {
    joint_states_msg_.header.stamp = timestamp;

    for (int i = 0; i < 12; ++i) {
        joint_states_msg_.position[i] = motorState[joints_[i]].q;
        joint_states_msg_.velocity[i] = motorState[joints_[i]].dq;
        joint_states_msg_.effort[i] = motorState[joints_[i]].tauEst;
    }

    joint_states_pub_->publish(joint_states_msg_);
}

void LeggedSDKInterface::pubFeetContact(std::array<int16_t, 4>& state, rclcpp::Time& timestamp) {
    geometry_msgs::msg::WrenchStamped FL_wrench;
    geometry_msgs::msg::WrenchStamped FR_wrench;
    geometry_msgs::msg::WrenchStamped RL_wrench;
    geometry_msgs::msg::WrenchStamped RR_wrench;

    FL_wrench.header.stamp = timestamp;
    FR_wrench.header.stamp = timestamp;
    RL_wrench.header.stamp = timestamp;
    RR_wrench.header.stamp = timestamp;

    FL_wrench.header.frame_id = "FL_foot";
    FR_wrench.header.frame_id = "FR_foot";
    RL_wrench.header.frame_id = "RL_foot";
    RR_wrench.header.frame_id = "RR_foot";

    FL_wrench.wrench.force.z = state[UNITREE_LEGGED_SDK::FL_];
    FR_wrench.wrench.force.z = state[UNITREE_LEGGED_SDK::FR_];
    RL_wrench.wrench.force.z = state[UNITREE_LEGGED_SDK::RL_];
    RR_wrench.wrench.force.z = state[UNITREE_LEGGED_SDK::RR_];

    FL_contact_pub_->publish(FL_wrench);
    FR_contact_pub_->publish(FR_wrench);
    RL_contact_pub_->publish(RL_wrench);
    RR_contact_pub_->publish(RR_wrench);
}

void LeggedSDKInterface::pubImu(UNITREE_LEGGED_SDK::IMU& imu, rclcpp::Time& timestamp) {
    imu_msg_.header.stamp = timestamp;
    imu_msg_.orientation.x = imu.quaternion[1];
    imu_msg_.orientation.y = imu.quaternion[2];
    imu_msg_.orientation.z = imu.quaternion[3];
    imu_msg_.orientation.w = imu.quaternion[0];
    imu_msg_.angular_velocity.x = imu.gyroscope[0];
    imu_msg_.angular_velocity.y = imu.gyroscope[1];
    imu_msg_.angular_velocity.z = imu.gyroscope[2];
    imu_msg_.linear_acceleration.x = imu.accelerometer[0];
    imu_msg_.linear_acceleration.y = imu.accelerometer[1];
    imu_msg_.linear_acceleration.z = imu.accelerometer[2];

    imu_pub_->publish(imu_msg_);
}

void LeggedSDKInterface::pubBmsState(UNITREE_LEGGED_SDK::BmsState& bms) {
    bms_msg_.bms_status = bms.bms_status;
    bms_msg_.cell_vol = bms.cell_vol;
    bms_msg_.current = bms.current;
    bms_msg_.cycle = bms.cycle;
    bms_msg_.soc = bms.SOC;

    bms_pub_->publish(bms_msg_);
}

bool LeggedSDKInterface::checkEmergencyCommand(std::array<uint8_t, 40>& remote_data) {
    
    memcpy(&_remoteKeyData, &remote_data[0], 40);

    if (_remoteKeyData.btn.components.L1 &&
        _remoteKeyData.btn.components.R1 &&
        _remoteKeyData.btn.components.L2 &&
        _remoteKeyData.btn.components.R2) {
            publish_log("ERROR", "EMERGENCY COMMAND RECEIVED VIA REMOTE");
            return true;
    }

    return false;
}

void LeggedSDKInterface::safetyLowStop() {
    // NOTE: called from watchdog() which already holds state_mutex_ — do NOT re-lock here.
    
    publish_log("ERROR", "SAFETY STOP ACTIVATED - EMERGENCY PROTOCOL ENGAGED");
    
    // Immediately change state to emergency stop
    changeInterfaceState(InterfaceState::EMERGENCY_STOP_LOW);
    
    // Send multiple safe commands immediately to ensure robot safety
    for(int i = 0; i < 5; i++) {
        if(!sendSafeLowCommandImmediate(1)) {  // Only 1 retry for emergency
            publish_log("ERROR", "CRITICAL: Failed to send emergency safe command #" + std::to_string(i+1));
        } else {
            publish_log("INFO", "Emergency safe command # " + std::to_string(i+1) + " sent successfully");
        }
        // Small delay between commands to ensure they are processed
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    
    publish_log("ERROR", "SAFETY STOP COMPLETE - Robot should be in safe state");
    // Schedule resource cleanup on threadState (state is already EMERGENCY_STOP_LOW).
    // Do NOT call disableLowInterface() here — it would overwrite EMERGENCY_STOP_LOW
    // with DISABLING_LOW and the cleanup condition in threadState would re-trigger.
    pending_low_cleanup_ = true;
}

void LeggedSDKInterface::initLowCmd() {
    for (int i = 0; i < 12; i++) {
        lowCmd_SDK_.motorCmd[i].mode = 10; // Servo mode
        lowCmd_SDK_.motorCmd[i].q = 0;
        lowCmd_SDK_.motorCmd[i].dq = 0;
        lowCmd_SDK_.motorCmd[i].Kp = 0;
        lowCmd_SDK_.motorCmd[i].Kd = 0;
        lowCmd_SDK_.motorCmd[i].tau = 0;
    }
}

void LeggedSDKInterface::setQoSProfiles() {
    // Define QoS profiles for publishers and subscribers
    imu_qos_ = std::make_shared<rclcpp::SensorDataQoS>();
    joint_state_qos_ = std::make_shared<rclcpp::SensorDataQoS>();
    wireless_remote_qos_ = std::make_shared<rclcpp::QoS>(rclcpp::QoS(10).reliable().durability_volatile());
    lowcmd_qos_ = std::make_shared<rclcpp::QoS>(rclcpp::QoS(1).reliable().durability_volatile()
                    .deadline(rclcpp::Duration::from_seconds(0.01))      // 10 ms
                    .lifespan(rclcpp::Duration::from_seconds(0.05)));    // 50 ms
}

UNITREE_LEGGED_SDK::LowCmd LeggedSDKInterface::createSafeLowCommand() {
    UNITREE_LEGGED_SDK::LowCmd safe_cmd;
    
    // Initialize the command structure
    memset(&safe_cmd, 0, sizeof(safe_cmd));
    
    // Read current robot state to get current joint positions
    UNITREE_LEGGED_SDK::LowState current_state = lowState_buf_.read();
    
    // Configure each motor to hold current position with moderate stiffness
    for (int i = 0; i < 12; i++) {
        safe_cmd.motorCmd[i].mode = 10;  // Position control mode
        
        // Use current position only if it is meaningfully non-zero (above noise floor).
        // The original code used || which would pass for any tiny non-zero value (sensor
        // noise), causing the safe command to hold a near-zero garbage position instead of
        // falling back to a known-safe zero. Both conditions must be true (&&).
        if (current_state.motorState[i].q != 0.0 &&
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

bool LeggedSDKInterface::sendSafeLowCommandImmediate(int retries) {
    UNITREE_LEGGED_SDK::LowCmd safe_cmd = createSafeLowCommand();

    for(int attempt = 0; attempt < retries; attempt++) {
        try {
            lowlevel_udp_.SetSend(safe_cmd);
            lowlevel_udp_.Send();
            
            RCLCPP_INFO(this->get_logger(), "Safe low command sent successfully (attempt %d/%d)", attempt+1, retries);
            return true;
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to send safe low command (attempt %d/%d): %s", 
                       attempt+1, retries, e.what());
            
            if(attempt < retries - 1) {
                // Brief delay before retry
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }
    
    RCLCPP_ERROR(this->get_logger(), "CRITICAL: Failed to send safe low command after %d attempts", retries);
    return false;
}

std::string LeggedSDKInterface::stateToString(InterfaceState state) {
    switch(state) {
        case InterfaceState::DISABLED: return "DISABLED";
        case InterfaceState::ENABLING_LOW: return "ENABLING_LOW";
        case InterfaceState::ENABLED_LOW: return "ENABLED_LOW";
        case InterfaceState::DISABLING_LOW: return "DISABLING_LOW";
        case InterfaceState::EMERGENCY_STOP_LOW: return "EMERGENCY_STOP_LOW";
        case InterfaceState::ENABLING_HIGH: return "ENABLING_HIGH";
        case InterfaceState::ENABLED_HIGH: return "ENABLED_HIGH";
        case InterfaceState::DISABLING_HIGH: return "DISABLING_HIGH";
        case InterfaceState::EMERGENCY_STOP_HIGH: return "EMERGENCY_STOP_HIGH";
        default: return "UNKNOWN";
    }
}

void LeggedSDKInterface::changeInterfaceState(InterfaceState new_state) {
    if(interface_state_ != new_state) {
        InterfaceState old_state = interface_state_;
        interface_state_ = new_state;

        std::string old_state_str = stateToString(old_state);
        std::string new_state_str = stateToString(new_state);
        publish_log("INFO", "Interface state changed: " + old_state_str + " -> " + new_state_str);

        // Reset disabling counter when entering a disabling state.
        if (new_state == InterfaceState::DISABLING_LOW || new_state == InterfaceState::DISABLING_HIGH) {
            _disabling_safe_sends_count = 0;
        }

        if (new_state == InterfaceState::DISABLED) {
            if (old_state == InterfaceState::DISABLING_LOW ||
                old_state == InterfaceState::EMERGENCY_STOP_LOW ||
                old_state == InterfaceState::ENABLED_LOW ||
                old_state == InterfaceState::ENABLING_LOW) {
                pending_low_cleanup_.exchange(true);
            } else if (old_state == InterfaceState::DISABLING_HIGH ||
                       old_state == InterfaceState::EMERGENCY_STOP_HIGH ||
                       old_state == InterfaceState::ENABLED_HIGH ||
                       old_state == InterfaceState::ENABLING_HIGH) {
                pending_high_cleanup_.exchange(true);
            }
        }
    }
}

void LeggedSDKInterface::setHighModeCallback(
  const std::shared_ptr<unitree_ros2_interface::srv::SetHighMode::Request> req,
  std::shared_ptr<unitree_ros2_interface::srv::SetHighMode::Response> res) {

    const uint8_t requested = static_cast<uint8_t>(req->mode);

    publish_log("INFO", "Received high mode request: " +
                std::to_string(static_cast<unsigned>(requested)) + " (" +
                highModeToString(requested) + ")");

    if (!checkHighModeTransition(requested)) {
        publish_log("WARN", "Invalid high mode transition from " +
                    std::to_string(static_cast<unsigned>(high_mode_)) + " (" +
                    highModeToString(static_cast<uint8_t>(high_mode_)) + ") to " +
                    std::to_string(static_cast<unsigned>(requested)) + " (" +
                    highModeToString(requested) + ")");
        res->res = false;
        return;
    }

    if (requested == START) {
        publish_log("INFO", "Starting high mode macro: " +
                    std::to_string(static_cast<unsigned>(high_mode_)) + " (" +
                    highModeToString(static_cast<uint8_t>(high_mode_)) + ") -> " +
                    std::to_string(static_cast<unsigned>(VELOCITY_MODE)) + " (" +
                    highModeToString(VELOCITY_MODE) + ")");
        res->res = launchHighModeMacro(start_seq_);
        return;
    }

    if (requested == STOP) {
        publish_log("INFO", "Stopping high mode macro: " +
                    std::to_string(static_cast<unsigned>(high_mode_)) + " (" +
                    highModeToString(static_cast<uint8_t>(high_mode_)) + ") -> " +
                    std::to_string(static_cast<unsigned>(IDLE_MODE)) + " (" +
                    highModeToString(IDLE_MODE) + ")");
        res->res = launchHighModeMacro(stop_seq_);
        return;
    }

    // Single transition
    high_mode_ = requested;
    high_cmd_.mode = requested;
    wait_check_mode_ = true;

    publish_log("INFO", "High mode transition allowed -> " +
                std::to_string(static_cast<unsigned>(requested)) + " (" +
                highModeToString(requested) + ")");
    res->res = true;
}

void LeggedSDKInterface::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  
    if(!isEnabledHigh()) {
        publish_log("WARN", "High interface not enabled, ignoring cmd_vel");
        return;
    }
  
    if (high_mode_ != VELOCITY_MODE) {
        // ROS2 throttle: period in milliseconds + clock 
        publish_log("WARN", "Robot not in VELOCITY_MODE, ignoring cmd_vel");
        return;
    }

    // Assumo che la tua convert.hpp abbia rosMsg2Cmd(msg) anche in ROS2.
    // In caso contrario: adatta la firma (es. rosMsg2Cmd(*msg)).
    high_cmd_ = rosMsg2Cmd(*msg);

    last_cmd_vel_time_ = this->now();
}

void LeggedSDKInterface::highCmdCallback(const unitree_legged_msgs::msg::HighCmd::SharedPtr msg) {
    high_cmd_ = rosMsg2Cmd(*msg);
}

void LeggedSDKInterface::lowLevelCmdClbk(const unitree_legged_msgs::msg::LowCmd::SharedPtr msg) {
    if (!isEnabledLow()) {
        return;  // Silently discard commands when not enabled
    }
    UNITREE_LEGGED_SDK::LowCmd sdk_cmd;
    sdk_cmd = rosMsg2Cmd(msg);
    lowCmd_buf_.write(sdk_cmd);
}

void LeggedSDKInterface::pubOdom() {
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

bool LeggedSDKInterface::launchHighModeMacro(const std::vector<std::pair<uint8_t, double>> & sequence) {
  
    if(!isEnabledHigh()) {
        publish_log("WARN", "High interface not enabled - cannot launch high mode macro.");
        return false;
    }

    if (macro_running_.exchange(true)) {
        publish_log("WARN", "A high mode macro is already running. Rejecting new request.");
        return false;
    }

    std::thread([this, sequence]() {
        auto stop_macro = [this]() { macro_running_ = false; };

        for (const auto & step : sequence) {
        const uint8_t target   = step.first;
        const double  wait_sec = step.second;

        {
            std::lock_guard<std::mutex> lk(high_mode_mtx_);

            if (!checkHighModeTransition(target)) {
            publish_log("WARN", "Macro aborted: invalid transition from " +
                        std::to_string(static_cast<unsigned>(high_mode_)) + " (" +
                        highModeToString(static_cast<uint8_t>(high_mode_)) + ") to " +
                        std::to_string(static_cast<unsigned>(target)) + " (" +
                        highModeToString(target) + ")");
            stop_macro();
            return;
            }

            // Importante: aggiorna high_mode_ per far funzionare le transizioni step-by-step
            high_mode_ = target;
            high_cmd_.mode = target;
            wait_check_mode_ = true;

            publish_log("INFO", "Macro step -> " +
                        std::to_string(static_cast<unsigned>(target)) + " (" +
                        highModeToString(target) + ") [wait=" + std::to_string(wait_sec) + "s]");
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
const std::unordered_set<uint8_t> LeggedSDKInterface::allowed_modes_ = {
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

const std::unordered_map<uint8_t, std::unordered_set<uint8_t>> LeggedSDKInterface::allowed_transitions_ = {
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