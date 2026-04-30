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
        "/unitree_go1/sport_controller/set_mode",
        std::bind(&IOROS::setHighModeCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    publish_log("INFO", "ROS 2 subscribers initialized");
}

void IOROS::setHighModeCallback(
    const std::shared_ptr<unitree_ros2_interface::srv::SetHighMode::Request> req,
    std::shared_ptr<unitree_ros2_interface::srv::SetHighMode::Response> res) {

    const uint8_t requested = static_cast<uint8_t>(req->mode);

    publish_log("INFO", "Received high mode request: " +
        std::to_string(static_cast<unsigned>(requested)) + " (" +
        highModeToString(requested) + ")");

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
