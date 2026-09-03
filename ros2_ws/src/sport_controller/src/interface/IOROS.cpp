/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/IOROS.h"
#include "common/Logger.h"
#include <csignal>
#include <iostream>
#include <memory>
#include <string.h>
#include <thread>
#include <unistd.h>

namespace {
// Status service of the low-level SDK interface, and the message it reports when
// the low-level (UDP) path is up. Both must match legged-sdk-interface.cpp.
constexpr const char *kLowStatusService = "/unitree_go1/legged_sdk/get_status_low";
constexpr const char *kLowStatusEnabled = "LOW";
}  // namespace

void IOROS::RosShutDown(int sig){
    (void)sig;
    std::cout << "ROS 2 interface shutting down!" << std::endl;
    rclcpp::shutdown();
}

void IOROS::initializeJointIndexMap(){
    joint_index_map["FL_hip_joint"] = FL_0;
    joint_index_map["FL_thigh_joint"] = FL_1;
    joint_index_map["FL_calf_joint"] = FL_2;
    joint_index_map["FR_hip_joint"] = FR_0;
    joint_index_map["FR_thigh_joint"] = FR_1;
    joint_index_map["FR_calf_joint"] = FR_2;
    joint_index_map["RL_hip_joint"] = RL_0;
    joint_index_map["RL_thigh_joint"] = RL_1;
    joint_index_map["RL_calf_joint"] = RL_2;
    joint_index_map["RR_hip_joint"] = RR_0;
    joint_index_map["RR_thigh_joint"] = RR_1;
    joint_index_map["RR_calf_joint"] = RR_2;
}

IOROS::IOROS(rclcpp::Node::SharedPtr node_ptr) : IOInterface() {
    _nm = node_ptr;

    _nm->declare_parameter("robot_name", "go1");
    _nm->get_parameter("robot_name", _robot_name);
    std::cout << "robot_name: " << _robot_name << std::endl;

    initializeJointIndexMap();
    initRecv();
    initSend();

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), 1
    );

    executor->add_node(_nm);
    executor_thread = std::thread([executor] (){
        executor->spin();
    });
    executor_thread.detach();

    publish_log("INFO", "ROS 2 multi-threaded executor started");
    signal(SIGINT, IOROS::RosShutDown);

    useRemote = true;
    remoteUserCommand = UserCommand::NONE;
    remoteUserValue.setZero();
    publish_log("INFO", "Wireless remote enabled");
    publish_log("INFO", "Controller initialized in DISABLED state, waiting for enable request...");
}

IOROS::~IOROS(){
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

void IOROS::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    sendCmd(cmd);
    recvState(state);

    if(useRemote){
        state->userCmd = remoteUserCommand;
        state->userValue = remoteUserValue;
    } else if (cmdPanel) {
        state->userCmd = cmdPanel->getUserCmd();
        state->userValue = cmdPanel->getUserValue();
    } else {
        state->userCmd = UserCommand::NONE;
        state->userValue.setZero();
    }
}

bool IOROS::fetchModeRequest(uint8_t &mode) {
    std::lock_guard<std::mutex> lock(mode_request_mutex_);
    if (!has_pending_mode_request_) {
        return false;
    }

    mode = pending_mode_request_;
    has_pending_mode_request_ = false;
    return true;
}

void IOROS::setPendingModeRequest(uint8_t mode) {
    std::lock_guard<std::mutex> lock(mode_request_mutex_);
    pending_mode_request_ = mode;
    has_pending_mode_request_ = true;
}

void IOROS::sendCmd(const LowlevelCmd *lowCmd) {
    // Single choke point for the joint commands: while DISABLED the controller
    // owns nothing and must stay off the wire, whoever calls in.
    if(_controlState.load(std::memory_order_relaxed) == ControllerState::DISABLED) {
        return;
    }

    for(int i(0); i < 12; ++i) {
        const int index = joints_map_normal2unitree[i];
        _lowCmd.motor_cmd[index].mode = lowCmd->motorCmd[i].mode;
        _lowCmd.motor_cmd[index].q = lowCmd->motorCmd[i].q;
        _lowCmd.motor_cmd[index].dq = lowCmd->motorCmd[i].dq;
        _lowCmd.motor_cmd[index].tau = lowCmd->motorCmd[i].tau;
        _lowCmd.motor_cmd[index].kd = lowCmd->motorCmd[i].Kd;
        _lowCmd.motor_cmd[index].kp = lowCmd->motorCmd[i].Kp;
    }
    _lowCmd_pub->publish(_lowCmd);
}

void IOROS::recvState(LowlevelState *state) {
    for(int i(0); i < 12; ++i) {
        state->motorState[i].q = _lowState.motor_state[i].q;
        state->motorState[i].dq = _lowState.motor_state[i].dq;
        state->motorState[i].ddq = _lowState.motor_state[i].ddq;
        state->motorState[i].tauEst = _lowState.motor_state[i].tau_est;
    }
    for(int i(0); i < 3; ++i) {
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];
}

void IOROS::initSend() {
    _lowCmd_pub = _nm->create_publisher<unitree_legged_msgs::msg::LowCmd>(
        "/unitree_go1/low_cmd",
        rclcpp::QoS(1)
            .reliable()
            .durability_volatile()
            .deadline(rclcpp::Duration::from_seconds(0.01))
            .lifespan(rclcpp::Duration::from_seconds(0.05)));
}

void IOROS::initRecv() {
    init_publish_log(_nm);
    _imu_sub = _nm->create_subscription<sensor_msgs::msg::Imu>(
        "/unitree_go1/imu", rclcpp::SensorDataQoS(), std::bind(&IOROS::imuCallback, this, std::placeholders::_1));
    _joint_state_sub = _nm->create_subscription<sensor_msgs::msg::JointState>(
        "/unitree_go1/joint_states", rclcpp::SensorDataQoS(), std::bind(&IOROS::jointStateCallback, this, std::placeholders::_1));
    _remote_sub = _nm->create_subscription<unitree_legged_msgs::msg::WirelessRemote>(
        "/unitree_go1/remote", 1, std::bind(&IOROS::remoteCallback, this, std::placeholders::_1));

    mode_service_ = _nm->create_service<unitree_ros2_interface::srv::SetHighMode>(
        "/unitree_go1/sport/set_mode",
        std::bind(&IOROS::setHighModeCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    enable_service_ = _nm->create_service<std_srvs::srv::SetBool>(
        "/unitree_go1/sport/enable",
        std::bind(&IOROS::setEnabledCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    status_service_ = _nm->create_service<std_srvs::srv::Trigger>(
        "/unitree_go1/sport/get_status",
        std::bind(&IOROS::getStatusCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    // false = do NOT hand this group to the node's executor: lowInterfaceReady()
    // spins it itself on a throwaway executor. If the group were owned by the node
    // executor (a single thread, busy inside the enable callback) the reply could
    // never be delivered and every request would time out.
    low_status_cbg_ = _nm->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    low_status_client_ = _nm->create_client<std_srvs::srv::Trigger>(
        kLowStatusService, rmw_qos_profile_services_default, low_status_cbg_);

    publish_log("INFO", "ROS 2 subscribers initialized");
}

/**
 * @brief Ask the low-level SDK interface whether it is currently in LOW mode.
 *
 * Enabling the controller while the SDK interface is disabled (or in HIGH mode)
 * would leave it publishing low_cmd into a void - nothing forwards it to the robot
 * over UDP - and the operator would see a controller claiming to be ENABLED while
 * the robot does not move. So this is a hard precondition for enabling.
 *
 * The SDK node answers success=true unconditionally and reports the real state in
 * the message field ("LOW" / "HIGH" / "DISABLED"), so it is the message that is
 * checked here, not success (see LeggedSDKInterface::onGetStatus).
 *
 * Blocking for up to ~2 s is safe: this runs on the service thread while the
 * controller is still DISABLED, so the FSM is idle and nothing is being published.
 *
 * @param detail Filled with a human-readable reason when the check fails.
 * @return true only if the low-level interface answered and reports "LOW".
 */
bool IOROS::lowInterfaceReady(std::string &detail) {
    if (!low_status_client_->wait_for_service(std::chrono::seconds(1))) {
        detail = std::string("low-level interface not reachable (") + kLowStatusService + " unavailable)";
        return false;
    }

    auto future = low_status_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_callback_group(low_status_cbg_, _nm->get_node_base_interface());
    if (exec.spin_until_future_complete(future, std::chrono::seconds(1)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        // Drop the orphaned request, otherwise it leaks in the client's pending map.
        low_status_client_->remove_pending_request(future);
        detail = "low-level interface did not answer the status request";
        return false;
    }

    const auto response = future.get();
    if (response->message != kLowStatusEnabled) {
        detail = std::string("low-level interface is '") + response->message +
            "', expected '" + kLowStatusEnabled + "'";
        return false;
    }
    return true;
}

void IOROS::setEnabledCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res) {

    const ControllerState current = _controlState.load(std::memory_order_relaxed);

    if (req->data) {
        // ENABLE REQUEST: only legal from a fully released controller.
        if (current != ControllerState::DISABLED) {
            res->success = false;
            res->message = std::string("Enable rejected - controller is not DISABLED. Current state: ") +
                controlStateToString(current);
            publish_log("WARN", res->message);
            return;
        }

        // Safety precondition: refuse to take over unless the low-level interface
        // is actually forwarding low_cmd to the robot.
        std::string reason;
        if (!lowInterfaceReady(reason)) {
            res->success = false;
            res->message = "Enable rejected - " + reason;
            publish_log("WARN", res->message);
            return;
        }

        // Drop any mode request left queued by an aborted run so the controller
        // does not replay it the moment it wakes up.
        {
            std::lock_guard<std::mutex> lock(mode_request_mutex_);
            has_pending_mode_request_ = false;
        }

        _controlState.store(ControllerState::ENABLED, std::memory_order_relaxed);
        res->success = true;
        res->message = "Controller enabled.";
        publish_log("INFO", "Controller state changed: DISABLED -> ENABLED");
        return;
    }

    // DISABLE REQUEST: never cut the commands dead. Queue the high-level _STOP
    // mode, which the FSM expands into FIXEDSTAND -> FIXEDKENNEL -> PASSIVE, and
    // only stop publishing once it reports PASSIVE via onDisableComplete().
    if (current != ControllerState::ENABLED) {
        res->success = false;
        res->message = std::string("Disable rejected - controller is not ENABLED. Current state: ") +
            controlStateToString(current);
        publish_log("WARN", res->message);
        return;
    }

    _controlState.store(ControllerState::DISABLING, std::memory_order_relaxed);
    setPendingModeRequest(_STOP);
    res->success = true;
    res->message = "Disable initiated - walking the robot down to PASSIVE.";
    publish_log("INFO", "Controller state changed: ENABLED -> DISABLING");
}

void IOROS::getStatusCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {

    (void)req;  // Trigger carries no payload
    res->success = true;
    res->message = controlStateToString(_controlState.load(std::memory_order_relaxed));
}

void IOROS::onDisableComplete() {
    _controlState.store(ControllerState::DISABLED, std::memory_order_relaxed);
    publish_log("INFO", "Controller state changed: DISABLING -> DISABLED (robot released, low_cmd stopped)");
}

const char *IOROS::controlStateToString(ControllerState state) {
    switch (state) {
        case ControllerState::DISABLED: return "DISABLED";
        case ControllerState::ENABLED: return "ENABLED";
        case ControllerState::DISABLING: return "DISABLING";
        default: return "UNKNOWN";
    }
}

void IOROS::setHighModeCallback(
    const std::shared_ptr<unitree_ros2_interface::srv::SetHighMode::Request> req,
    std::shared_ptr<unitree_ros2_interface::srv::SetHighMode::Response> res) {

    const uint8_t requested = static_cast<uint8_t>(req->mode);

    publish_log("INFO", "Received high mode request: " +
        std::to_string(static_cast<unsigned>(requested)) + " (" +
        highModeToString(requested) + ")");

    const ControllerState current = _controlState.load(std::memory_order_relaxed);
    if (current != ControllerState::ENABLED) {
        publish_log("WARN", std::string("Mode request rejected - controller is ") + controlStateToString(current));
        res->res = false;
        return;
    }

    if (!isValidHighMode(requested)) {
        publish_log("WARN", "Unsupported high mode requested: " + std::to_string(static_cast<unsigned>(requested)));
        res->res = false;
        return;
    }

    setPendingModeRequest(requested);
    publish_log("INFO", "Queued high mode request: " + std::string(highModeToString(requested)));
    res->res = true;
}

bool IOROS::isValidHighMode(uint8_t mode) const {
    switch (mode) {
        case _PASSIVE:
        case _FIXED_STAND_MODE:
        case _FREE_STAND_MODE:
        case _STAND_DOWN_MODE:
        case _VELOCITY_MODE:
        case _MOVE_BASE:
        case _START:
        case _STOP:
            return true;
        default:
            return false;
    }
}

const char *IOROS::highModeToString(uint8_t mode) const {
    switch (mode) {
        case _PASSIVE: return "_PASSIVE";
        case _FREE_STAND_MODE: return "_FREE_STAND_MODE";
        case _FIXED_STAND_MODE: return "_FIXED_STAND_MODE";
        case _STAND_DOWN_MODE: return "_STAND_DOWN_MODE";
        case _VELOCITY_MODE: return "_VELOCITY_MODE";
        case _MOVE_BASE: return "_MOVE_BASE";
        case _START: return "_START";
        case _STOP: return "_STOP";
        default: return "UNKNOWN_MODE";
    }
}

void IOROS::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    _lowState.imu.quaternion[0] = msg->orientation.w;
    _lowState.imu.quaternion[1] = msg->orientation.x;
    _lowState.imu.quaternion[2] = msg->orientation.y;
    _lowState.imu.quaternion[3] = msg->orientation.z;

    _lowState.imu.gyroscope[0] = msg->angular_velocity.x;
    _lowState.imu.gyroscope[1] = msg->angular_velocity.y;
    _lowState.imu.gyroscope[2] = msg->angular_velocity.z;

    _lowState.imu.accelerometer[0] = msg->linear_acceleration.x;
    _lowState.imu.accelerometer[1] = msg->linear_acceleration.y;
    _lowState.imu.accelerometer[2] = msg->linear_acceleration.z;
}

void IOROS::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
        const auto it = joint_index_map.find(msg->name[i]);
        if (it == joint_index_map.end()) {
            continue;
        }

        const int index = it->second;
        if (i < msg->position.size()) {
            _lowState.motor_state[index].q = msg->position[i];
        }
        if (i < msg->velocity.size()) {
            _lowState.motor_state[index].dq = msg->velocity[i];
        }
        if (i < msg->effort.size()) {
            _lowState.motor_state[index].tau_est = msg->effort[i];
        }
    }
}

void IOROS::remoteCallback(const unitree_legged_msgs::msg::WirelessRemote::SharedPtr msg) {
    // The remote only has authority while the controller is ENABLED. During
    // DISABLING this is mandatory: a stray L2_A would send the FSM back to
    // FIXEDSTAND, PASSIVE would never be reached and the shutdown would hang.
    if (_controlState.load(std::memory_order_relaxed) != ControllerState::ENABLED) {
        remoteUserCommand = UserCommand::NONE;
        remoteUserValue.setZero();
        return;
    }

    if (msg->l2 && msg->b) {
        remoteUserCommand = UserCommand::L2_B;
    }
    else if (msg->l2 && msg->a) {
        remoteUserCommand = UserCommand::L2_A;
    }
    else if (msg->select_btn) {
        remoteUserCommand = UserCommand::SELECT;
    }
    else if (msg->l2 && msg->x) {
        remoteUserCommand = UserCommand::L2_X;
    }
    else if (msg->l1 && msg->x) {
        remoteUserCommand = UserCommand::L1_X;
    }
    else if (msg->start_btn) {
        remoteUserCommand = UserCommand::START;
    }
    else {
        remoteUserCommand = UserCommand::NONE;
    }

    remoteUserValue.lx = killZeroOffset(msg->lx, 0.08);
    remoteUserValue.ly = killZeroOffset(msg->ly, 0.08);
    remoteUserValue.rx = killZeroOffset(msg->rx, 0.08);
    remoteUserValue.ry = killZeroOffset(msg->ry, 0.08);
}
