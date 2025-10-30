#include "unitree_ros2_interface/Interface-node.hpp"
#include <stdio.h>

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

    // Start only UDP loops using Unitree SDK (critical low-level communication)
    loop_udpSend = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("low_udp_send", dt_send, 3, boost::bind(&InterfaceNode::lowSend, this));
    loop_udpRecv = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("low_udp_recv", dt_recv, 3, boost::bind(&InterfaceNode::lowRecive, this));

    // loop_udpSendRecv = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("low_udp_send_recv", 0.001, 3, boost::bind(&InterfaceNode::lowSendRecv, this));

    setQoSProfiles();

    // Create ROS2 subscription and publishers
    _lowCmd_sub = this->create_subscription<unitree_legged_msgs::msg::LowCmd>(prefix + "/low_cmd", *_lowcmd_qos,std::bind(&InterfaceNode::lowLevelCmdClbk, this, std::placeholders::_1));
    _joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>(prefix + "/joint_state", *_joint_state_qos);
    _imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(prefix + "/imu", *_imu_qos);
    _wrls_remote_pub = this->create_publisher<unitree_legged_msgs::msg::WirelessRemote>(prefix + "/remote", *_wrls_remote_qos);

    set_enabled_srv_ = this->create_service<std_srvs::srv::SetBool>("/enable_unitree_interface", 
        std::bind(&InterfaceNode::onSetEnabled, this, std::placeholders::_1, std::placeholders::_2));

    // Setup JS msg
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

    _lowlevel_udp.InitCmdData(_lowCmd_SDK);

    // Initialize _lowState_SDK to prevent garbage data
    memset(&_lowState_SDK, 0, sizeof(_lowState_SDK));

    initLowCmd();

    RCLCPP_INFO(this->get_logger(), "Interface ready for starting");
}

void InterfaceNode::onSetEnabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    _enabled = request->data;
    response->success = true;
    
    if (_enabled) {
        startInterface();

        response->message = "Interface enabled - robot control active";
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Interface ENABLED - Robot control is now active");
    } else {
        safetyStop();

        response->message = "Interface disabled - robot control inactive";
        response->success = true;
        RCLCPP_WARN(this->get_logger(), "Interface DISABLED - Robot control is now inactive");
    }
}

InterfaceNode::~InterfaceNode() {

    loop_udpSend->shutdown();
    loop_udpRecv->shutdown();

    // Stop all publishers' timers
    if (_low_level_state_timer) _low_level_state_timer->cancel();

    RCLCPP_INFO(this->get_logger(), "Interface stopped");
}

void InterfaceNode::pubRemoteState() {
    // Copy the remote data from the low state to the struct
    memcpy(&_remoteKeyData, &_lowState.wireless_remote[0], 40);

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

void InterfaceNode::lowLevelCmdClbk(const unitree_legged_msgs::msg::LowCmd::SharedPtr msg) {
    // Safety checks can be added here if needed
    // _lowCmd_SDK = rosMsg2Cmd(msg);
    _lowCmd_buf.write(rosMsg2Cmd(msg));
}

void InterfaceNode::pubJointsState() {
    _joint_state.header.stamp = this->get_clock()->now();
    
    for (int i = 0; i < 12; ++i) {
        _joint_state.position[i] = _lowState.motor_state[joints[i]].q;
        _joint_state.velocity[i] = _lowState.motor_state[joints[i]].dq;
        _joint_state.effort[i] = _lowState.motor_state[joints[i]].tau_est;
    }

    _joint_state_pub->publish(_joint_state);
}

void InterfaceNode::pubImu() {
    _imu.header.stamp = this->get_clock()->now();

    _imu.orientation.x = _lowState.imu.quaternion[1];
    _imu.orientation.y = _lowState.imu.quaternion[2];
    _imu.orientation.z = _lowState.imu.quaternion[3];
    _imu.orientation.w = _lowState.imu.quaternion[0];

    _imu.angular_velocity.x = _lowState.imu.gyroscope[0];
    _imu.angular_velocity.y = _lowState.imu.gyroscope[1];
    _imu.angular_velocity.z = _lowState.imu.gyroscope[2];

    _imu.linear_acceleration.x = _lowState.imu.accelerometer[0];
    _imu.linear_acceleration.y = _lowState.imu.accelerometer[1];
    _imu.linear_acceleration.z = _lowState.imu.accelerometer[2];

    _imu_pub->publish(_imu);
}

void InterfaceNode::pubLowLevelState() {

    UNITREE_LEGGED_SDK::LowState state = _lowState_buf.read();
    _lowState = state2rosMsg(state);

    auto now = this->get_clock()->now();

    _imu.header.stamp = now;
    _joint_state.header.stamp = now;

    // IMU msg
    _imu.orientation.x = _lowState.imu.quaternion[1];
    _imu.orientation.y = _lowState.imu.quaternion[2];
    _imu.orientation.z = _lowState.imu.quaternion[3];
    _imu.orientation.w = _lowState.imu.quaternion[0];

    _imu.angular_velocity.x = _lowState.imu.gyroscope[0];
    _imu.angular_velocity.y = _lowState.imu.gyroscope[1];
    _imu.angular_velocity.z = _lowState.imu.gyroscope[2];

    _imu.linear_acceleration.x = _lowState.imu.accelerometer[0];
    _imu.linear_acceleration.y = _lowState.imu.accelerometer[1];
    _imu.linear_acceleration.z = _lowState.imu.accelerometer[2];

    // Joint State msg
    for (int i = 0; i < 12; ++i) {
        _joint_state.position[i] = _lowState.motor_state[joints[i]].q;
        _joint_state.velocity[i] = _lowState.motor_state[joints[i]].dq;
        _joint_state.effort[i] = _lowState.motor_state[joints[i]].tau_est;
    }

    // Remote msg
    memcpy(&_remoteKeyData, &_lowState.wireless_remote[0], 40);

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

    // Publish all messages
    _imu_pub->publish(_imu);
    _joint_state_pub->publish(_joint_state);
    _wrls_remote_pub->publish(_remoteMsg);
}

void InterfaceNode::startInterface() {
    RCLCPP_INFO(this->get_logger(), "Starting main interface loop for robot control");

    loop_udpSend->start();
    loop_udpRecv->start();

    startLowLevelStateLoop(); // 500 Hz low-level state publisher

    RCLCPP_INFO(this->get_logger(), "Unitree ROS2 Interface started");
}

void InterfaceNode::safetyStop() {
    RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP ACTIVATED - Disabling interface and applying safety protocols");
    
    // Immediately disable the interface
    _enabled = false;
    
    // Initialize safe command state
    initLowCmd();

    for(int i(0); i<12; i++) {
        _lowCmd_SDK.motorCmd[i].mode = 0; // Lock motors in current position
        _lowCmd_SDK.motorCmd[i].q = _lowState_SDK.motorState[joints[i]].q; // Current position
        _lowCmd_SDK.motorCmd[i].dq = 0;
        _lowCmd_SDK.motorCmd[i].Kp = 50; // Moderate stiffness
        _lowCmd_SDK.motorCmd[i].Kd = 1;  // Some damping
        _lowCmd_SDK.motorCmd[i].tau = 0;
    }
    
    // Send emergency stop command immediately
    try {
        _lowlevel_udp.SetSend(_lowCmd_SDK);
        _lowlevel_udp.Send();
        RCLCPP_INFO(this->get_logger(), "Emergency stop command sent successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send emergency stop command: %s", e.what());
    }

    loop_udpSend->shutdown();
    loop_udpRecv->shutdown();
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
    _wrls_remote_qos = std::make_shared<rclcpp::QoS>(rclcpp::QoS(5).reliable().durability_volatile());
    _lowcmd_qos = std::make_shared<rclcpp::QoS>(rclcpp::QoS(1).reliable().durability_volatile()
                    .deadline(rclcpp::Duration::from_seconds(0.01))      // 10 ms
                    .lifespan(rclcpp::Duration::from_seconds(0.05)));    // 50 ms
}

void InterfaceNode::startLowLevelStateLoop() {
    RCLCPP_INFO(this->get_logger(), "Low level state loop publisher started at %.2f Hz", dt_recv);
    auto period = std::chrono::duration<double>(dt_recv); // Using dt_recv for frequency
    _low_level_state_timer = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&InterfaceNode::pubLowLevelState, this)
    );
}

