#include "unitree_ros2_interface/legged-sdk-interface.hpp"
#include <stdio.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <mutex>
#include <thread>
#include <chrono>
#include <cerrno>
#include <sys/mman.h>

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
// Both sockets are built with the explicit-length constructor rather than the
// level-based UDP(LOWLEVEL, ...) one, because only this overload accepts a RecvEnum.
// The low-level wire frames are COMPRESSED - shorter than sizeof(LowCmd)/sizeof(LowState)
// - so the SDK's exported lengths must be used rather than sizeof. For the high level
// the two happen to coincide (129 / 1087 bytes).
lowlevel_udp_(8091, "192.168.123.10", 8007,
              UNITREE_LEGGED_SDK::LOW_CMD_LENGTH, UNITREE_LEGGED_SDK::LOW_STATE_LENGTH,
              false, UNITREE_LEGGED_SDK::RecvEnum::blockTimeout),
highlevel_udp_(8090, "192.168.123.161", 8082, sizeof(high_cmd_), sizeof(high_state_),
               false, UNITREE_LEGGED_SDK::RecvEnum::blockTimeout)  {

    // These two calls are what actually arm the blocking-with-timeout receive path; the
    // RecvEnum above only seeds the timeout value. Removing them silently reverts both
    // sockets to polling, where a no-data read is indistinguishable from a stale frame.
    // See kUdpRecvTimeoutMs and test/test_udp_recv_contract.cpp.
    lowlevel_udp_.SetRecvTimeout(kUdpRecvTimeoutMs);
    highlevel_udp_.SetRecvTimeout(kUdpRecvTimeoutMs);

    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    declare_and_get_params();
    validate_params_or_throw();

    // Lock the address space before anything hot is allocated. MCL_FUTURE covers the UDP
    // buffers, the publisher thread's stack and the DDS pools created further down, so a
    // page fault cannot stall the command path later. Deliberately NOT done by calling the
    // SDK's InitEnvironment(): that helper also issues
    // sched_setscheduler(getpid(), SCHED_FIFO, 95), which would put every thread in the
    // process - ROS executor and DDS included - above the kernel's networking work.
    // Priorities are applied per thread instead, see applyRtPriorityOnce.
    if (lock_memory_) {
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            RCLCPP_ERROR(this->get_logger(),
                         "mlockall failed (%s). Pages may be swapped out under load; grant the "
                         "container --ulimit memlock=-1.", std::strerror(errno));
        } else {
            RCLCPP_INFO(this->get_logger(), "Address space locked (mlockall).");
        }
    }

    pub_log_ = this->create_publisher<std_msgs::msg::String>(make_topic("legged_sdk/log"), 1000);
    set_led_color_srv_ = this->create_client<unitree_ros2_interface::srv::SetLedColor>(make_topic("set_face_color"));

    // Start from a deterministic low-state buffer; safe commands must never
    // read uninitialized memory before first valid SDK receive.
    memset(&lowState_SDK_, 0, sizeof(lowState_SDK_));
    lowState_buf_.write(lowState_SDK_);
    has_low_state_.store(false, std::memory_order_release);
    low_level_verified_.store(false, std::memory_order_release);
    last_low_state_time_ = this->now();
    last_low_state_time_ns_.store(last_low_state_time_.nanoseconds(), std::memory_order_release);
    _disabling_safe_sends_count.store(0, std::memory_order_release);
    
    setQoSProfiles();
    
    // Housekeeping timer: consumes deferred cleanup flags on the executor thread.
    // Sensor publishing is no longer done here (moved to the dedicated publisher
    // thread), so this can run slowly.
    state_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&LeggedSDKInterface::threadState, this)
    );

    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2),
        std::bind(&LeggedSDKInterface::watchdog, this)
    );

    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        make_topic("legged_sdk/diagnostics"), rclcpp::QoS(10));
    last_diag_time_ns_.store(this->now().nanoseconds(), std::memory_order_relaxed);
    diag_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&LeggedSDKInterface::publishUdpDiagnostics, this)
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
    
    // Setup IMU msg. The covariance diagonals are constant, so they are written once here
    // rather than on every frame. Leaving them at zero (the previous behaviour) tells any
    // REP-145 consumer the measurement is exact, which for the Go1's drifting yaw is the
    // most damaging thing this node could claim.
    imu_msg_.header.frame_id = makeFrame("imu_link");
    imu_msg_.orientation_covariance.fill(0.0);
    imu_msg_.angular_velocity_covariance.fill(0.0);
    imu_msg_.linear_acceleration_covariance.fill(0.0);
    for (size_t i = 0; i < 3; ++i) {
        imu_msg_.orientation_covariance[i * 3 + i] =
            imu_orientation_stddev_[i] * imu_orientation_stddev_[i];
        imu_msg_.angular_velocity_covariance[i * 3 + i] =
            imu_angular_velocity_stddev_ * imu_angular_velocity_stddev_;
        imu_msg_.linear_acceleration_covariance[i * 3 + i] =
            imu_linear_acceleration_stddev_ * imu_linear_acceleration_stddev_;
    }

    if(startup_mode_ == 1) {
        RCLCPP_INFO(this->get_logger(), "Startup mode set to HIGH - attempting to enable high-level interface...");
        if(enableHighInterface()) {
            RCLCPP_INFO(this->get_logger(), "High-level interface enabled on startup. Waiting for first HighState before publishing state topics.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable high-level interface on startup. Check connection and parameters.");
        }
    } else if (startup_mode_ == 2) {
        RCLCPP_INFO(this->get_logger(), "Startup mode set to LOW - attempting to enable low-level interface...");
        if(enableLowInterface()) {
            RCLCPP_INFO(this->get_logger(), "Low-level enable initiated on startup. Waiting for LowState.levelFlag == LOWLEVEL.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable low-level interface on startup. Check connection and parameters.");
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "Startup mode set to DISABLED - interfaces will not be enabled on startup.");
    }

};

LeggedSDKInterface::~LeggedSDKInterface() {
    interface_state_.store(InterfaceState::DISABLED, std::memory_order_release);

    if (state_timer_) {
        state_timer_->cancel();
    }
    if (watchdog_timer_) {
        watchdog_timer_->cancel();
    }
    if (diag_timer_) {
        diag_timer_->cancel();
    }

    // Stop the publisher thread before tearing down the UDP loops / publishers.
    stopPublisherThread();

    if (loop_udpSendRecv) {
        loop_udpSendRecv.reset();
    }
    if (loop_udpSend) {
        loop_udpSend.reset();
    }
    if (loop_udpRecv) {
        loop_udpRecv.reset();
    }
}

void LeggedSDKInterface::declare_and_get_params() {
    // Base
    this->declare_parameter<std::string>("namespace", "");

    // UDP
    this->declare_parameter<double>("dt_send", 0.001);
    this->declare_parameter<double>("dt_recv", 0.001);
    this->declare_parameter<std::string>("sdk_cmd_topic", "low_cmd");
    this->declare_parameter<std::string>("imu_topic", "imu");
    this->declare_parameter<std::string>("joint_states_topic", "joint_states");
    this->declare_parameter<double>("remote_frequency", 10.0);
    this->declare_parameter<std::string>("wireless_remote_topic", "remote");
    this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    this->declare_parameter<std::string>("odom_topic", "odom");
    this->declare_parameter<double>("odom_frequency", 100.0);
    this->declare_parameter<double>("cmd_vel_timeout", 0.5);
    this->declare_parameter<double>("low_state_timeout_sec", 0.1);
    this->declare_parameter<double>("high_state_timeout_sec", 0.5);
    this->declare_parameter<std::string>("bms_topic", "bms_state");
    this->declare_parameter<double>("soc_threshold", 20.0);
    this->declare_parameter<int>("startup_mode", 0);     // 0: DISABLED, 1: HIGH, 2: LOW
    this->declare_parameter<bool>("publish_odom_tf", true);
    this->declare_parameter<double>("bms_frequency", 1.0);
    this->declare_parameter<double>("low_state_frequency", 0.0);

    // Safety guards
    this->declare_parameter<bool>("enable_position_limit", true);
    this->declare_parameter<int>("power_protect_factor", 10);
    this->declare_parameter<double>("position_protect_limit", 0.0);

    // Real-time thread placement
    this->declare_parameter<int>("udp_thread_priority", 80);
    this->declare_parameter<int>("udp_send_cpu", -1);
    this->declare_parameter<int>("udp_recv_cpu", -1);
    this->declare_parameter<bool>("lock_memory", true);

    // Frames and sensor conditioning
    this->declare_parameter<std::string>("frame_prefix", "");
    this->declare_parameter<std::vector<double>>("foot_force_offset", {0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("foot_force_scale", {1.0, 1.0, 1.0, 1.0});
    this->declare_parameter<std::vector<double>>("imu_orientation_stddev", {0.01, 0.01, 0.5});
    this->declare_parameter<double>("imu_angular_velocity_stddev", 0.01);
    this->declare_parameter<double>("imu_linear_acceleration_stddev", 0.1);

    // Get parameters
    this->get_parameter("namespace", namespace_param_);
    this->get_parameter("sdk_cmd_topic", sdk_cmd_topic_);
    this->get_parameter("dt_send", dt_send_);
    this->get_parameter("dt_recv", dt_recv_);
    this->get_parameter("imu_topic", imu_topic_);
    this->get_parameter("joint_states_topic", joint_states_topic_);
    this->get_parameter("remote_frequency", remote_frequency_);
    this->get_parameter("wireless_remote_topic", wireless_remote_topic_);
    this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("odom_frequency", odom_frequency_);
    this->get_parameter("cmd_vel_timeout", cmd_vel_timeout_);
    this->get_parameter("low_state_timeout_sec", low_state_timeout_sec_);
    this->get_parameter("high_state_timeout_sec", high_state_timeout_sec_);
    this->get_parameter("bms_topic", bms_topic_);
    this->get_parameter("soc_threshold", soc_threshold_);
    this->get_parameter("startup_mode", startup_mode_);
    this->get_parameter("publish_odom_tf", publish_odom_tf_);
    this->get_parameter("bms_frequency", bms_frequency_);
    this->get_parameter("low_state_frequency", low_state_frequency_);

    this->get_parameter("enable_position_limit", enable_position_limit_);
    this->get_parameter("power_protect_factor", power_protect_factor_);
    this->get_parameter("position_protect_limit", position_protect_limit_);

    this->get_parameter("udp_thread_priority", udp_thread_priority_);
    this->get_parameter("udp_send_cpu", udp_send_cpu_);
    this->get_parameter("udp_recv_cpu", udp_recv_cpu_);
    this->get_parameter("lock_memory", lock_memory_);

    this->get_parameter("frame_prefix", frame_prefix_);
    this->get_parameter("foot_force_offset", foot_force_offset_);
    this->get_parameter("foot_force_scale", foot_force_scale_);
    this->get_parameter("imu_orientation_stddev", imu_orientation_stddev_);
    this->get_parameter("imu_angular_velocity_stddev", imu_angular_velocity_stddev_);
    this->get_parameter("imu_linear_acceleration_stddev", imu_linear_acceleration_stddev_);

    frame_prefix_ = normalize_ns(frame_prefix_);
}

void LeggedSDKInterface::validate_params_or_throw() {
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
    if (low_state_timeout_sec_ <= 0.0) {
        throw std::invalid_argument("low_state_timeout_sec must be > 0");
    }
    if (high_state_timeout_sec_ <= 0.0) {
        throw std::invalid_argument("high_state_timeout_sec must be > 0");
    }
    if (soc_threshold_ < 0.0 || soc_threshold_ > 100.0) {
        throw std::invalid_argument("soc_threshold must be between 0 and 100");
    }
    if (startup_mode_ < 0 || startup_mode_ > 2) {
        throw std::invalid_argument("startup_mode must be 0 (DISABLED), 1 (HIGH), or 2 (LOW)");
    }
    if (bms_frequency_ <= 0.0) {
        throw std::invalid_argument("bms_frequency must be > 0");
    }
    if (low_state_frequency_ < 0.0) {
        throw std::invalid_argument("low_state_frequency must be >= 0 (0 = publish every frame)");
    }
    // The SDK rejects anything above 10 internally; catching it here turns a silent
    // no-op guard into a startup failure.
    if (power_protect_factor_ < 0 || power_protect_factor_ > 10) {
        throw std::invalid_argument("power_protect_factor must be 0 (disabled) or 1..10");
    }
    if (position_protect_limit_ < 0.0) {
        throw std::invalid_argument("position_protect_limit must be >= 0 (0 = disabled)");
    }
    if (foot_force_offset_.size() != 4 || foot_force_scale_.size() != 4) {
        throw std::invalid_argument("foot_force_offset and foot_force_scale must have 4 elements");
    }
    if (imu_orientation_stddev_.size() != 3) {
        throw std::invalid_argument("imu_orientation_stddev must have 3 elements (roll, pitch, yaw)");
    }
    if (imu_angular_velocity_stddev_ <= 0.0 || imu_linear_acceleration_stddev_ <= 0.0) {
        throw std::invalid_argument("IMU stddev parameters must be > 0; a zero covariance means "
                                    "'known exactly' to REP-145 consumers");
    }
    for (const double s : imu_orientation_stddev_) {
        if (s <= 0.0) {
            throw std::invalid_argument("imu_orientation_stddev entries must be > 0");
        }
    }

    // sched_get_priority_max(SCHED_FIFO) is 99 on Linux; anything above that would make
    // pthread_setschedparam fail at runtime on every loop thread instead of here.
    if (udp_thread_priority_ < 0 || udp_thread_priority_ > sched_get_priority_max(SCHED_FIFO)) {
        throw std::invalid_argument("udp_thread_priority must be 0 (leave alone) or 1.." +
                                    std::to_string(sched_get_priority_max(SCHED_FIFO)));
    }
    if (udp_thread_priority_ == 0) {
        RCLCPP_WARN(this->get_logger(), "udp_thread_priority is 0 - the UDP loops run at normal "
                                        "scheduling priority and can be preempted by DDS traffic.");
    }

    if (power_protect_factor_ == 0) {
        RCLCPP_WARN(this->get_logger(), "power_protect_factor is 0 - the SDK power guard is DISABLED "
                                        "and nothing limits the mechanical power commanded to the motors.");
    }
    if (!enable_position_limit_) {
        RCLCPP_WARN(this->get_logger(), "enable_position_limit is false - commanded joint positions "
                                        "are NOT clamped to the Go1 mechanical limits.");
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
  if (pub_log_) {
    pub_log_->publish(m);
  }
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

    // Create the UDP send/receive loops using Unitree SDK (critical low-level communication).
    // The callbacks are wrapped so each loop thread raises its own scheduling priority on
    // first entry - the SDK never does this despite advertising THREAD_PRIORITY. The third
    // argument is the CPU to pin to: it used to be a hardcoded 3 for BOTH loops, so the
    // send and receive threads fought over a single core.
    loop_udpSend = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>(
        "low_udp_send", dt_send_, udp_send_cpu_,
        UNITREE_LEGGED_SDK::Callback([this] { applyRtPriorityOnce("low_udp_send"); lowSend(); }));
    loop_udpRecv = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>(
        "low_udp_recv", dt_recv_, udp_recv_cpu_,
        UNITREE_LEGGED_SDK::Callback([this] { applyRtPriorityOnce("low_udp_recv"); lowRecive(); }));

    lowCmd_sub_ = this->create_subscription<unitree_legged_msgs::msg::LowCmd>(make_topic(sdk_cmd_topic_), *lowcmd_qos_,std::bind(&LeggedSDKInterface::lowLevelCmdClbk, this, std::placeholders::_1));
    
    // High-rate sensor streams use SensorDataQoS (Best-Effort, KeepLast) so publish()
    // never blocks on slow subscribers and no stale queue builds up.
    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(make_topic(joint_states_topic_), *joint_state_qos_);
    imu_pub_         = this->create_publisher<sensor_msgs::msg::Imu>(make_topic(imu_topic_), *imu_qos_);
    wireless_remote_pub_ = this->create_publisher<unitree_legged_msgs::msg::WirelessRemote>(make_topic(wireless_remote_topic_), *wireless_remote_qos_);
    FL_contact_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(make_topic("FL_foot/wrench"), 10);
    FR_contact_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(make_topic("FR_foot/wrench"), 10);
    RL_contact_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(make_topic("RL_foot/wrench"), 10);
    RR_contact_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(make_topic("RR_foot/wrench"), 10);
    bms_pub_ = this->create_publisher<unitree_legged_msgs::msg::BmsState>(make_topic(bms_topic_), 10);
    // SensorDataQoS (best-effort) rather than a reliable queue: at the robot's frame rate
    // a reliable ~1 kB stream builds history and eventually blocks the publisher thread,
    // which sits between the UDP receive loop and DDS.
    low_state_pub_ = this->create_publisher<unitree_legged_msgs::msg::LowState>(
        make_topic("low_state"), rclcpp::SensorDataQoS());

    // Initialize LowCmd buffer
    lowlevel_udp_.InitCmdData(lowCmd_SDK_);
    lowCmd_buf_.write(lowCmd_SDK_);

    // Initialize _lowState_SDK to prevent garbage data
    memset(&lowState_SDK_, 0, sizeof(lowState_SDK_));
    lowState_buf_.write(lowState_SDK_);
    has_low_state_.store(false, std::memory_order_release);
    low_level_verified_.store(false, std::memory_order_release);
    last_low_state_time_ = this->now();
    last_low_state_time_ns_.store(last_low_state_time_.nanoseconds(), std::memory_order_release);
    _disabling_safe_sends_count.store(0, std::memory_order_release);
    resetRecvCounters();

    initLowCmd();
    lowCmd_buf_.write(lowCmd_SDK_);  // Update buffer with mode=10 set by initLowCmd()

    changeInterfaceState(InterfaceState::ENABLING_LOW);
    publish_log("INFO", "Low interface enable initiated. Waiting for LowState.levelFlag == LOWLEVEL.");

    // Publisher thread must be alive before the receive loop starts so the first
    // latched frame already has a consumer.
    startPublisherThread();

    loop_udpSend->start();
    loop_udpRecv->start();

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

    // Same wrapping and CPU placement as the low-level loops, see enableLowInterface.
    loop_udpSend = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>(
        "high_udp_send", dt_send_, udp_send_cpu_,
        UNITREE_LEGGED_SDK::Callback([this] { applyRtPriorityOnce("high_udp_send"); highUdpSend(); }));
    loop_udpRecv = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>(
        "high_udp_recv", dt_recv_, udp_recv_cpu_,
        UNITREE_LEGGED_SDK::Callback([this] { applyRtPriorityOnce("high_udp_recv"); highUdpRecv(); }));

    // High-rate sensor streams use SensorDataQoS; odom keeps a small reliable queue.
    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(make_topic(joint_states_topic_), *joint_state_qos_);
    imu_pub_         = this->create_publisher<sensor_msgs::msg::Imu>(make_topic(imu_topic_), *imu_qos_);
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

    const rclcpp::Time now = this->now();
    high_enable_start_time_ns_.store(now.nanoseconds(), std::memory_order_release);
    has_high_state_.store(false, std::memory_order_release);
    last_high_state_time_ns_.store(0, std::memory_order_release);
    resetRecvCounters();

    {
        std::lock_guard<std::mutex> lock(high_cmd_mutex_);
        last_cmd_vel_time_ = now;

        // Init mode
        high_mode_ = IDLE_MODE;
        high_cmd_.mode = IDLE_MODE;

        highlevel_udp_.InitCmdData(high_cmd_);

        wait_check_window_ = static_cast<int>(std::ceil(0.5 / dt_recv_));
        wait_check_count_ = 0;
        wait_check_mode_ = false;
    }

    // ENABLING_HIGH, not ENABLED_HIGH: the transition to ENABLED_HIGH is made by
    // highUdpRecv() once a HighState frame has actually been received, which is what
    // arms the handshake path there and the handshake-timeout branch in watchdog().
    // Going straight to ENABLED_HIGH declared the link up before a single byte had
    // arrived and left both of those unreachable.
    changeInterfaceState(InterfaceState::ENABLING_HIGH);
    publish_log("INFO", "High interface enable initiated. Waiting for the first HighState frame.");

    // Publisher thread must be alive before the receive loop starts.
    startPublisherThread();

    loop_udpSend->start();
    loop_udpRecv->start();

    return true;
}

bool LeggedSDKInterface::disableLowInterface() {
    // Schedule cleanup on the ROS2 timer thread (threadState).
    // Never reset LoopFunc shared_ptrs here: this method may be called
    // from user code whose thread context is unknown.
    pending_low_cleanup_.store(true, std::memory_order_release);
    changeInterfaceState(InterfaceState::DISABLING_LOW);
    return true;
}

bool LeggedSDKInterface::disableHighInterface() {
    // Same deferred-cleanup pattern as disableLowInterface.
    pending_high_cleanup_.store(true, std::memory_order_release);
    changeInterfaceState(InterfaceState::DISABLING_HIGH);
    return true;
}

void LeggedSDKInterface::cleanupLowResources() {
    // Stop and destroy UDP LoopFunc threads first.
    // reset() blocks until the thread joins, which is fast because lowSend/lowRecive
    // early-return when interface_state_ == DISABLED.
    if (loop_udpSend) { loop_udpSend.reset(); }
    if (loop_udpRecv) { loop_udpRecv.reset(); }

    // Receive loop is joined: no more frames will be signalled. Join the publisher
    // thread before releasing the publishers so no publish() races a reset().
    stopPublisherThread();

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
    low_state_pub_.reset();
    has_low_state_.store(false, std::memory_order_release);
    low_level_verified_.store(false, std::memory_order_release);
    last_low_state_time_ = this->now();
    last_low_state_time_ns_.store(last_low_state_time_.nanoseconds(), std::memory_order_release);

    publish_log("INFO", "Low interface resources released.");
}

void LeggedSDKInterface::cleanupHighResources() {
    // Stop and destroy UDP LoopFunc threads.
    // highUdpSend/highUdpRecv early-return when !isEnabledHigh().
    if (loop_udpSend) { loop_udpSend.reset(); }
    if (loop_udpRecv) { loop_udpRecv.reset(); }

    // Receive loop is joined: no more frames will be signalled. Join the publisher
    // thread before releasing the publishers so no publish() races a reset().
    stopPublisherThread();

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
    has_high_state_.store(false, std::memory_order_release);
    last_high_state_time_ns_.store(0, std::memory_order_release);

    publish_log("INFO", "High interface resources released.");
}

void LeggedSDKInterface::threadState() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Consume pending cleanup flags. Cleanup is always performed here (ROS2 timer
    // thread) so that LoopFunc objects are never destroyed from within their own
    // callback, which would be undefined behaviour.
    if (isDisabled() && pending_low_cleanup_.load(std::memory_order_acquire)) {
        cleanupLowResources();
        pending_low_cleanup_.store(false, std::memory_order_release);
    }
    if (isDisabled() && pending_high_cleanup_.load(std::memory_order_acquire)) {
        cleanupHighResources();
        pending_high_cleanup_.store(false, std::memory_order_release);
    }

    // Sensor publishing happens on the dedicated publisher thread (publisherThreadLoop),
    // driven by real received frames. This timer only performs deferred cleanup.
}

bool LeggedSDKInterface::dueForPublish(double now_sec, double & last_pub_sec, double freq_hz) {
    if (now_sec - last_pub_sec >= 1.0 / freq_hz) {
        last_pub_sec = now_sec;
        return true;
    }
    return false;
}

void LeggedSDKInterface::startPublisherThread() {
    if (pub_thread_.joinable()) {
        return;  // already running
    }

    // Reset the wake flag and the per-stream rate gates so the first received frame
    // publishes every stream immediately.
    {
        std::lock_guard<std::mutex> lk(pub_cv_mutex_);
        pub_new_frame_ = false;
    }
    last_remote_pub_sec_ = 0.0;
    last_odom_pub_sec_ = 0.0;
    last_bms_pub_sec_ = 0.0;
    last_low_state_pub_sec_ = 0.0;

    pub_thread_run_.store(true, std::memory_order_release);
    pub_thread_ = std::thread(&LeggedSDKInterface::publisherThreadLoop, this);
}

void LeggedSDKInterface::stopPublisherThread() {
    pub_thread_run_.store(false, std::memory_order_release);
    pub_cv_.notify_all();
    if (pub_thread_.joinable()) {
        pub_thread_.join();
    }
}

void LeggedSDKInterface::publisherThreadLoop() {
    while (pub_thread_run_.load(std::memory_order_acquire)) {
        // Sleep until a new frame is signalled or we are told to stop.
        {
            std::unique_lock<std::mutex> lk(pub_cv_mutex_);
            pub_cv_.wait(lk, [this] {
                return pub_new_frame_ || !pub_thread_run_.load(std::memory_order_acquire);
            });
            if (!pub_thread_run_.load(std::memory_order_acquire)) {
                break;
            }
            pub_new_frame_ = false;
        }

        // Stamp with the frame's receive instant (set by the UDP receive path), not
        // the publish instant, so downstream state estimation sees the true
        // measurement time regardless of publisher-thread scheduling latency.
        const int64_t recv_ns = last_frame_recv_ns_.load(std::memory_order_acquire);
        rclcpp::Time stamp = (recv_ns > 0) ? rclcpp::Time(recv_ns, RCL_ROS_TIME) : this->now();
        const double now_sec = stamp.seconds();
        const InterfaceState state = getState();

        if (state == InterfaceState::ENABLED_LOW || state == InterfaceState::ENABLING_LOW) {
            UNITREE_LEGGED_SDK::LowState ls = lowState_buf_.read();
            // imu/joints/feet published at full frame rate. A rate gate here would
            // beat against the robot's bursty ~900 Hz stream and drop frames, so the
            // only decimated streams are the genuinely slow ones (remote below).
            pubImu(ls.imu, stamp);
            pubJointsState(ls.motorState, stamp);
            pubFeetContact(ls.footForce, stamp);   // full rate: used by contact detection
            if (dueForPublish(now_sec, last_remote_pub_sec_, remote_frequency_)) {
                pubRemoteState(ls.wirelessRemote);
            }
            if (dueForPublish(now_sec, last_bms_pub_sec_, bms_frequency_)) {
                pubBmsState(ls.bms);                // battery changes at a few Hz at most
            }
            // /low_state is the largest message this node emits; low_state_frequency_ == 0
            // keeps the previous every-frame behaviour, any positive value decimates it.
            if (low_state_frequency_ <= 0.0 ||
                dueForPublish(now_sec, last_low_state_pub_sec_, low_state_frequency_)) {
                pubLowState(ls);                    // same frame as the topics above
            }
        } else if (state == InterfaceState::ENABLED_HIGH) {
            UNITREE_LEGGED_SDK::HighState hs;
            {
                std::lock_guard<std::mutex> lk(high_state_mutex_);
                hs = high_state_;
            }
            pubImu(hs.imu, stamp);                  // full rate (see LOW branch)
            pubJointsState(hs.motorState, stamp);   // full rate
            pubFeetContact(hs.footForce, stamp);
            if (dueForPublish(now_sec, last_remote_pub_sec_, remote_frequency_)) {
                pubRemoteState(hs.wirelessRemote);
            }
            if (dueForPublish(now_sec, last_bms_pub_sec_, bms_frequency_)) {
                pubBmsState(hs.bms);
            }
            pubOdom(hs);
        }
    }
}

void LeggedSDKInterface::watchdog() {
    // This function is called periodically to monitor the health of the interface.
    // SOC is unreliable in the current setup; keep publishing BMS telemetry,
    // but do not use SOC to drive watchdog state transitions.

    if (getState() == InterfaceState::ENABLING_HIGH) {
        const int64_t start_ns = high_enable_start_time_ns_.load(std::memory_order_acquire);
        const double age_sec = static_cast<double>(this->now().nanoseconds() - start_ns) * 1e-9;
        if (age_sec > high_state_timeout_sec_) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (getState() == InterfaceState::ENABLING_HIGH) {
                publish_log("ERROR", "Watchdog: HighState handshake timed out after " +
                    std::to_string(age_sec) + " s (timeout " +
                    std::to_string(high_state_timeout_sec_) + " s). Initiating disable.");
                disableHighInterface();
            }
        }
    }

    // HighState staleness. Previously the high level had no liveness check at all once
    // enabled: if sport_mode stopped answering, /odom and the sensor topics kept
    // republishing the last frame forever. Now that highUdpRecv() only timestamps
    // accepted frames, the same age test used for the low level applies here.
    if (getState() == InterfaceState::ENABLED_HIGH) {
        double age_sec = 0.0;
        if (!isHighStateFresh(&age_sec)) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (getState() == InterfaceState::ENABLED_HIGH) {
                char msg[256];
                std::snprintf(
                    msg,
                    sizeof(msg),
                    "Watchdog: HighState stale for %.6f s (timeout %.6f s). Initiating disable.",
                    age_sec,
                    high_state_timeout_sec_);
                publish_log("ERROR", msg);
                disableHighInterface();
            }
        }
    }

    if (getState() == InterfaceState::ENABLED_LOW) {
        if (!low_level_verified_.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (getState() == InterfaceState::ENABLED_LOW) {
                publish_log("ERROR", "Watchdog: low-level mode verification lost while ENABLED_LOW. Initiating graceful disable.");
                disableLowInterface();
            }
        } else {
            double age_sec = 0.0;
            if (!isLowStateFresh(&age_sec)) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                if (getState() == InterfaceState::ENABLED_LOW) {
                    char msg[256];
                    std::snprintf(
                        msg,
                        sizeof(msg),
                        "Watchdog: LowState stale for %.6f s (timeout %.6f s). Initiating graceful disable.",
                        age_sec,
                        low_state_timeout_sec_);
                    publish_log("ERROR", msg);
                    disableLowInterface();
                }
            }
        }
    }

    // TODO: The emergency command should work even for the high-level interface.
    // Gate on isEnabledLow() FIRST: otherwise checkEmergencyCommand() decodes a stale
    // lowState_buf_ every tick while in HIGH/DISABLED and can log a spurious emergency.
    if (isEnabledLow()) {
        UNITREE_LEGGED_SDK::LowState wdState = lowState_buf_.read();
        if (checkEmergencyCommand(wdState.wirelessRemote)) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (getState() != InterfaceState::EMERGENCY_STOP_LOW) {
                RCLCPP_ERROR(this->get_logger(), "Emergency stop command received from remote - Transitioning to EMERGENCY_STOP_LOW state!");
                publish_log("ERROR", "Emergency stop command received from remote - Transitioning to EMERGENCY_STOP_LOW state!");
                safetyLowStop();
            }
        }
    }

    // TODO: If the last cmd received timestamp is too old, consider transitioning to an emergency_stop state.

}

void LeggedSDKInterface::publishUdpDiagnostics() {
    if (!diag_pub_) {
        return;
    }

    const InterfaceState state = getState();
    const bool low_active =
        state == InterfaceState::ENABLING_LOW || state == InterfaceState::ENABLED_LOW ||
        state == InterfaceState::DISABLING_LOW || state == InterfaceState::EMERGENCY_STOP_LOW;
    const bool high_active =
        state == InterfaceState::ENABLING_HIGH || state == InterfaceState::ENABLED_HIGH ||
        state == InterfaceState::DISABLING_HIGH || state == InterfaceState::EMERGENCY_STOP_HIGH;

    const uint64_t ok      = recv_ok_count_.load(std::memory_order_relaxed);
    const uint64_t timeout = recv_timeout_count_.load(std::memory_order_relaxed);
    const uint64_t crc_err = recv_crc_err_count_.load(std::memory_order_relaxed);
    const uint64_t head_err = recv_head_err_count_.load(std::memory_order_relaxed);
    const uint64_t other_err = recv_other_err_count_.load(std::memory_order_relaxed);

    // Measured frame rate over the interval since the previous sample. This is the rate
    // the ROBOT actually delivers; it is deliberately not derived from dt_recv, which
    // now only bounds how often the receive thread wakes to find nothing.
    const rclcpp::Time now = this->now();
    const int64_t prev_ns = last_diag_time_ns_.exchange(now.nanoseconds(), std::memory_order_relaxed);
    const uint64_t prev_ok = last_diag_ok_count_.exchange(ok, std::memory_order_relaxed);
    const double dt_sec = static_cast<double>(now.nanoseconds() - prev_ns) * 1e-9;
    const double rate_hz = (dt_sec > 0.0 && ok >= prev_ok)
        ? static_cast<double>(ok - prev_ok) / dt_sec
        : 0.0;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "legged_sdk_interface: UDP link";
    status.hardware_id = low_active ? "192.168.123.10:8007 (low)"
                       : high_active ? "192.168.123.161:8082 (high)"
                                     : "none";

    if (!low_active && !high_active) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "Interface disabled, no UDP link active";
    } else if (ok == prev_ok) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "No frames received in the last diagnostics interval";
    } else if (crc_err > 0 || head_err > 0) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Link up but frames are being rejected (see crc/head counters)";
    } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "Link up";
    }

    auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        status.values.push_back(kv);
    };

    add("interface_state", stateToString(state));
    add("measured_frame_rate_hz", std::to_string(rate_hz));
    add("recv_ok", std::to_string(ok));
    add("recv_timeout", std::to_string(timeout));
    add("recv_crc_error", std::to_string(crc_err));
    add("recv_head_error", std::to_string(head_err));
    add("recv_other_error", std::to_string(other_err));

    // The SDK keeps its own tallies inside the UDP object. They are reported alongside
    // ours because they count at a different layer: RecvCRCError/RecvLoseError are
    // incremented inside Recv() itself, and SendError only exists there.
    const UNITREE_LEGGED_SDK::UDPState & udp_state =
        low_active ? lowlevel_udp_.udpState : highlevel_udp_.udpState;
    add("sdk_send_count", std::to_string(udp_state.SendCount));
    add("sdk_send_error", std::to_string(udp_state.SendError));
    add("sdk_recv_count", std::to_string(udp_state.RecvCount));
    add("sdk_recv_crc_error", std::to_string(udp_state.RecvCRCError));
    add("sdk_recv_lose_error", std::to_string(udp_state.RecvLoseError));

    if (low_active) {
        double age_sec = 0.0;
        isLowStateFresh(&age_sec);
        add("low_state_age_sec", std::to_string(age_sec));
    } else if (high_active) {
        double age_sec = 0.0;
        isHighStateFresh(&age_sec);
        add("high_state_age_sec", std::to_string(age_sec));
    }

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now;
    array.status.push_back(status);
    diag_pub_->publish(array);
}

void LeggedSDKInterface::onGetStatus(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    (void)request;  // Suppress unused parameter warning
    
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (isEnabledLow()) {
        response->success = true;
        response->message = "LOW";
    } else if (isEnabledHigh()) {
        response->success = true;
        response->message = "HIGH";
    } else {
        response->success = true;
        response->message = "DISABLED";
    }
}

void LeggedSDKInterface::onSetLowEnable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    const InterfaceState current_state = getState();

    if(request->data) {
        // ENABLE REQUEST
        if(current_state == InterfaceState::DISABLED) {
            if(!enableLowInterface()){
                response->success = false;
                response->message = "Failed to enable low interface.";
                RCLCPP_ERROR(this->get_logger(), "Failed to enable low interface.");
                publish_log("ERROR", "Failed to enable low interface.");
                auto led_req = std::make_shared<unitree_ros2_interface::srv::SetLedColor::Request>();
                led_req->r = 255;
                led_req->g = 0;
                led_req->b = 0;
                led_req->time = 2.5;
                set_led_color_srv_->async_send_request(led_req);
                return;
            }
            response->success = true;
            response->message = "Low interface enabling initiated. Waiting for LowState.levelFlag == LOWLEVEL.";
            publish_log("INFO", "Low interface enabling initiated. Waiting for LowState.levelFlag == LOWLEVEL.");
        } else {
            std::string current_state_str = stateToString(current_state);
            response->success = false;
            response->message = "Low Interface is not in DISABLED state. Current state: " + current_state_str;
            publish_log("WARN", "Low Interface enable request rejected - current state: " + current_state_str);
            auto led_req = std::make_shared<unitree_ros2_interface::srv::SetLedColor::Request>();
            led_req->r = 255;
            led_req->g = 120;
            led_req->b = 0;
            led_req->time = 2.5;
            set_led_color_srv_->async_send_request(led_req);
            return;
        }
    } else {
        // DISABLE REQUEST  
        if(current_state == InterfaceState::ENABLED_LOW ||
           current_state == InterfaceState::ENABLING_LOW ||
           current_state == InterfaceState::DISABLING_LOW ||
           current_state == InterfaceState::EMERGENCY_STOP_LOW) {
            // Initiate safe disable sequence
            publish_log("WARN", "DISABLE REQUESTED - Initiating safe shutdown sequence...");
            
            // Immediately send a safe command
            if(sendSafeLowCommandImmediate(3)) {
                disableLowInterface();
                _disabling_safe_sends_count.store(1, std::memory_order_release);  // We just sent one
                response->success = true;
                response->message = "Low Interface disable initiated. Safe commands being sent...";
                publish_log("INFO", "Low Interface disable initiated. Sending safe commands...");
                auto led_req = std::make_shared<unitree_ros2_interface::srv::SetLedColor::Request>();
                led_req->r = 0;
                led_req->g = 0;
                led_req->b = 255;
                led_req->time = 2.5;
                set_led_color_srv_->async_send_request(led_req);
                return;
            } else {
                // If we can't send safe command, force emergency stop
                changeInterfaceState(InterfaceState::EMERGENCY_STOP_LOW);
                response->success = false;
                response->message = "Communication error during disable - EMERGENCY STOP activated.";
                publish_log("ERROR", "Communication error during disable - EMERGENCY STOP activated.");
                auto led_req = std::make_shared<unitree_ros2_interface::srv::SetLedColor::Request>();
                led_req->r = 255;
                led_req->g = 0;
                led_req->b = 0;
                led_req->time = 2.5;
                set_led_color_srv_->async_send_request(led_req);
                return;
            }
        } else {
            std::string current_state_str = stateToString(current_state);
            response->success = false;
            response->message = "Low Interface is not in ENABLED/EMERGENCY state. Current state: " + current_state_str;
            publish_log("WARN", "Low Interface disable request rejected - current state: " + current_state_str);
            auto led_req = std::make_shared<unitree_ros2_interface::srv::SetLedColor::Request>();
            led_req->r = 255;
            led_req->g = 120;
            led_req->b = 0;
            led_req->time = 2.5;
            set_led_color_srv_->async_send_request(led_req);
            return;
        }
    }
}

void LeggedSDKInterface::onSetHighEnable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

    std::lock_guard<std::mutex> lock(state_mutex_);

    const InterfaceState current_state = getState();

    if (request->data) {
        // ENABLE REQUEST
        if (current_state == InterfaceState::DISABLED) {
            if (!enableHighInterface()) {
                response->success = false;
                response->message = "Failed to enable high interface.";
                RCLCPP_ERROR(this->get_logger(), "Failed to enable high interface.");
                publish_log("ERROR", "Failed to enable high interface.");
                auto led_req = std::make_shared<unitree_ros2_interface::srv::SetLedColor::Request>();
                led_req->r = 255;
                led_req->g = 0;
                led_req->b = 0;
                led_req->time = 2.5;
                set_led_color_srv_->async_send_request(led_req);
                return;
            }
            response->success = true;
            response->message = "High interface enabled. Waiting for first HighState before publishing state topics.";
            publish_log("INFO", "High interface enabled. Waiting for first HighState before publishing state topics.");
            auto led_req = std::make_shared<unitree_ros2_interface::srv::SetLedColor::Request>();
            led_req->r = 0;
            led_req->g = 255;
            led_req->b = 0;
            led_req->time = 2.5;
            set_led_color_srv_->async_send_request(led_req);
            return;
        } else {
            std::string current_state_str = stateToString(current_state);
            response->success = false;
            response->message = "Interface is not in DISABLED state. Current state: " + current_state_str;            
            publish_log("WARN", "High interface enable request rejected - current state: " + current_state_str);
            auto led_req = std::make_shared<unitree_ros2_interface::srv::SetLedColor::Request>();
            led_req->r = 255;
            led_req->g = 120;
            led_req->b = 0;
            led_req->time = 2.5;
            set_led_color_srv_->async_send_request(led_req);
            return;
        }
    } else {
        // DISABLE REQUEST
        if (current_state == InterfaceState::ENABLED_HIGH || current_state == InterfaceState::ENABLING_HIGH) {
            publish_log("WARN", "DISABLE HIGH REQUESTED - Initiating shutdown...");
            disableHighInterface();
            response->success = true;
            response->message = "High interface shutdown initiated...";
            auto led_req = std::make_shared<unitree_ros2_interface::srv::SetLedColor::Request>();
            led_req->r = 0;
            led_req->g = 255;
            led_req->b = 0;
            led_req->time = 2.5;
            set_led_color_srv_->async_send_request(led_req);
            return;
        } else {
            std::string current_state_str = stateToString(current_state);
            response->success = false;
            response->message = "High interface is not in ENABLED state. Current state: " + current_state_str;
            publish_log("WARN", "High interface disable request rejected - current state: " + current_state_str);
            auto led_req = std::make_shared<unitree_ros2_interface::srv::SetLedColor::Request>();
            led_req->r = 0;
            led_req->g = 120;
            led_req->b = 0;
            led_req->time = 2.5;
            set_led_color_srv_->async_send_request(led_req);
            return;
        }
    }
}

void LeggedSDKInterface::pubRemoteState(std::array<uint8_t, 40>& remote_data) {
    // Decode into a LOCAL struct: this runs on the state-timer thread while
    // checkEmergencyCommand() runs on the watchdog thread, so a shared member
    // would be a data race under the MultiThreadedExecutor.
    xRockerBtnDataStruct key;
    memcpy(&key, &remote_data[0], 40);

    // Kill the zero offset of analogs
    remote_msg_.lx = killZeroOffset(key.lx, 0.08);
    remote_msg_.ly = killZeroOffset(key.ly, 0.08);
    remote_msg_.rx = killZeroOffset(key.rx, 0.08);
    remote_msg_.ry = killZeroOffset(key.ry, 0.08);

    remote_msg_.l1 = key.btn.components.L1;
    remote_msg_.l2 = key.btn.components.L2;
    remote_msg_.r1 = key.btn.components.R1;
    remote_msg_.r2 = key.btn.components.R2;
    remote_msg_.f1 = key.btn.components.F1;
    remote_msg_.f2 = key.btn.components.F2;
    remote_msg_.a = key.btn.components.A;
    remote_msg_.b = key.btn.components.B;
    remote_msg_.x = key.btn.components.X;
    remote_msg_.y = key.btn.components.Y;
    remote_msg_.up = key.btn.components.up;
    remote_msg_.down = key.btn.components.down;
    remote_msg_.left = key.btn.components.left;
    remote_msg_.right = key.btn.components.right;
    remote_msg_.start_btn = key.btn.components.start;
    remote_msg_.select_btn = key.btn.components.select;

    wireless_remote_pub_->publish(remote_msg_);
}

void LeggedSDKInterface::pubLowState(const UNITREE_LEGGED_SDK::LowState & lowState) {
    // state2rosMsg takes a non-const reference, so work on a local copy of the caller's
    // frame. Note the frame's own `tick` (robot motion-controller clock, ms) is carried
    // through by the conversion: it is the only robot-side timebase available and the
    // only way a consumer can align /low_state with the topics published beside it.
    UNITREE_LEGGED_SDK::LowState frame = lowState;

    lowState_ = state2rosMsg(frame);
    low_state_pub_->publish(lowState_);
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
    // LowState::footForce is a raw int16 airbag-sensor count with a per-foot bias, NOT a
    // force in newtons. Publishing it straight into a WrenchStamped claimed SI units it
    // never had. The conditioning below is the calibration knob: with the default offset
    // 0 / scale 1 the value is unchanged raw counts (documented, not silently mislabelled),
    // and a measured per-foot bias and counts-to-newtons scale can be supplied by
    // parameter. Indices are SDK leg order {FR, FL, RR, RL}.
    auto condition = [this](int16_t raw, int leg) {
        return (static_cast<double>(raw) - foot_force_offset_[leg]) * foot_force_scale_[leg];
    };

    geometry_msgs::msg::WrenchStamped FL_wrench;
    geometry_msgs::msg::WrenchStamped FR_wrench;
    geometry_msgs::msg::WrenchStamped RL_wrench;
    geometry_msgs::msg::WrenchStamped RR_wrench;

    FL_wrench.header.stamp = timestamp;
    FR_wrench.header.stamp = timestamp;
    RL_wrench.header.stamp = timestamp;
    RR_wrench.header.stamp = timestamp;

    FL_wrench.header.frame_id = makeFrame("FL_foot");
    FR_wrench.header.frame_id = makeFrame("FR_foot");
    RL_wrench.header.frame_id = makeFrame("RL_foot");
    RR_wrench.header.frame_id = makeFrame("RR_foot");

    FL_wrench.wrench.force.z = condition(state[UNITREE_LEGGED_SDK::FL_], UNITREE_LEGGED_SDK::FL_);
    FR_wrench.wrench.force.z = condition(state[UNITREE_LEGGED_SDK::FR_], UNITREE_LEGGED_SDK::FR_);
    RL_wrench.wrench.force.z = condition(state[UNITREE_LEGGED_SDK::RL_], UNITREE_LEGGED_SDK::RL_);
    RR_wrench.wrench.force.z = condition(state[UNITREE_LEGGED_SDK::RR_], UNITREE_LEGGED_SDK::RR_);

    FL_contact_pub_->publish(FL_wrench);
    FR_contact_pub_->publish(FR_wrench);
    RL_contact_pub_->publish(RL_wrench);
    RR_contact_pub_->publish(RR_wrench);
}

void LeggedSDKInterface::pubImu(UNITREE_LEGGED_SDK::IMU& imu, rclcpp::Time& timestamp) {
    // Covariances are filled once in the constructor (setImuCovariances) and never
    // change, so only the payload is written per frame.
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
    // Local struct: shared with pubRemoteState only through the wire bytes, never
    // through node state, so the two timer threads cannot race.
    xRockerBtnDataStruct key;
    memcpy(&key, &remote_data[0], 40);

    if (key.btn.components.L1 &&
        key.btn.components.R1 &&
        key.btn.components.L2 &&
        key.btn.components.R2) {
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
    // Move to normal safe-disable flow so the LoopFunc thread can finish
    // sending safe frames, then transition to DISABLED and cleanup.
    disableLowInterface();
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
    lowcmd_qos_ = std::make_shared<rclcpp::QoS>(rclcpp::QoS(1).reliable().durability_volatile());
}

UNITREE_LEGGED_SDK::LowCmd LeggedSDKInterface::createSafeLowCommand() {
    // Start from SDK-initialized template so header/reserved fields stay valid.
    UNITREE_LEGGED_SDK::LowCmd safe_cmd = lowCmd_SDK_;
    
    // Read current robot state to get current joint positions
    UNITREE_LEGGED_SDK::LowState current_state = lowState_buf_.read();

    bool can_hold_position = has_low_state_.load(std::memory_order_acquire);
    double max_abs_q = 0.0;
    if (can_hold_position) {
        for (int i = 0; i < 12; ++i) {
            const double q = static_cast<double>(current_state.motorState[i].q);
            if (!std::isfinite(q) || std::abs(q) > 10.0) {
                can_hold_position = false;
                break;
            }
            max_abs_q = std::max(max_abs_q, std::abs(q));
        }
        if (max_abs_q <= 0.05) {
            can_hold_position = false;
        }
    }

    if (!can_hold_position) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Safe LowCmd fallback to passive command because no valid LowState is available yet.");
    }
    
    // Configure each motor either to hold current position (valid state)
    // or to a passive fallback (no valid state yet).
    for (int i = 0; i < 12; i++) {
        safe_cmd.motorCmd[i].mode = 10;  // Position control mode

        if (can_hold_position) {
            safe_cmd.motorCmd[i].q = current_state.motorState[i].q;  // Hold current position
            safe_cmd.motorCmd[i].Kp = 20.0;       // Moderate position stiffness
            safe_cmd.motorCmd[i].Kd = 0.5;        // Light damping
        } else {
            safe_cmd.motorCmd[i].q = 0.0;  // Fallback to zero position
            safe_cmd.motorCmd[i].Kp = 0.0;  // Passive fallback to avoid sudden pulls
            safe_cmd.motorCmd[i].Kd = 1.0;
        }
        
        safe_cmd.motorCmd[i].dq = 0.0;        // Zero velocity
        safe_cmd.motorCmd[i].tau = 0.0;       // Zero additional torque
    }
    
    return safe_cmd;
}

bool LeggedSDKInterface::applySafetyClamps(UNITREE_LEGGED_SDK::LowCmd & cmd) {
    // Clamp first: PositionLimit only bounds the commanded angles, so running it before
    // the protections means the protections judge the command that would actually be sent.
    if (enable_position_limit_) {
        safe_.PositionLimit(cmd);
    }

    // Both protections need the measured state. Without one they cannot evaluate
    // anything, and passing a zeroed LowState would fabricate a "zero power, zero
    // position" reading that always passes - worse than not running them.
    if (power_protect_factor_ <= 0 && position_protect_limit_ <= 0.0) {
        return true;
    }
    if (!has_low_state_.load(std::memory_order_acquire)) {
        return true;
    }

    UNITREE_LEGGED_SDK::LowState state = lowState_buf_.read();
    bool ok = true;

    if (power_protect_factor_ > 0 && safe_.PowerProtect(cmd, state, power_protect_factor_) < 0) {
        publish_log("ERROR", "SDK PowerProtect tripped: mechanical power over budget, all motors "
                             "forced to damping.");
        ok = false;
    }

    if (position_protect_limit_ > 0.0 &&
        safe_.PositionProtect(cmd, state, position_protect_limit_) < 0) {
        publish_log("ERROR", "SDK PositionProtect tripped: measured joint position outside the "
                             "limit table, all motors forced to damping.");
        ok = false;
    }

    return ok;
}

bool LeggedSDKInterface::sendSafeLowCommandImmediate(int retries) {
    UNITREE_LEGGED_SDK::LowCmd safe_cmd = createSafeLowCommand();

    // This path bypasses lowSend(), so it must run the guards itself or it would be the
    // one way to reach the motors unguarded. A trip here is not worth reporting: the
    // command is already a safe hold and the trip only makes it limper.
    applySafetyClamps(safe_cmd);

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
    const InterfaceState old_state = interface_state_.exchange(new_state, std::memory_order_acq_rel);
    if (old_state != new_state) {

        std::string old_state_str = stateToString(old_state);
        std::string new_state_str = stateToString(new_state);
        publish_log("INFO", "Interface state changed: " + old_state_str + " -> " + new_state_str);

        // Reset disabling counter when entering a disabling state.
        if (new_state == InterfaceState::DISABLING_LOW ||
            new_state == InterfaceState::EMERGENCY_STOP_LOW ||
            new_state == InterfaceState::DISABLING_HIGH ||
            new_state == InterfaceState::EMERGENCY_STOP_HIGH) {
            _disabling_safe_sends_count.store(0, std::memory_order_release);
        }

        if (new_state == InterfaceState::DISABLED) {
            if (old_state == InterfaceState::DISABLING_LOW ||
                old_state == InterfaceState::EMERGENCY_STOP_LOW ||
                old_state == InterfaceState::ENABLED_LOW ||
                old_state == InterfaceState::ENABLING_LOW) {
                pending_low_cleanup_.store(true, std::memory_order_release);
            } else if (old_state == InterfaceState::DISABLING_HIGH ||
                       old_state == InterfaceState::EMERGENCY_STOP_HIGH ||
                       old_state == InterfaceState::ENABLED_HIGH ||
                       old_state == InterfaceState::ENABLING_HIGH) {
                pending_high_cleanup_.store(true, std::memory_order_release);
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

    uint8_t current_mode = IDLE_MODE;
    bool transition_allowed = false;
    {
        std::lock_guard<std::mutex> lock(high_cmd_mutex_);
        current_mode = high_mode_;
        transition_allowed = checkHighModeTransitionFrom(current_mode, requested);
    }

    if (!transition_allowed) {
        publish_log("WARN", "Invalid high mode transition from " +
                    std::to_string(static_cast<unsigned>(current_mode)) + " (" +
                    highModeToString(current_mode) + ") to " +
                    std::to_string(static_cast<unsigned>(requested)) + " (" +
                    highModeToString(requested) + ")");
        res->res = false;
        return;
    }

    if (requested == START) {
        publish_log("INFO", "Starting high mode macro: " +
                    std::to_string(static_cast<unsigned>(current_mode)) + " (" +
                    highModeToString(current_mode) + ") -> " +
                    std::to_string(static_cast<unsigned>(VELOCITY_MODE)) + " (" +
                    highModeToString(VELOCITY_MODE) + ")");
        res->res = launchHighModeMacro(start_seq_);
        return;
    }

    if (requested == STOP) {
        publish_log("INFO", "Stopping high mode macro: " +
                    std::to_string(static_cast<unsigned>(current_mode)) + " (" +
                    highModeToString(current_mode) + ") -> " +
                    std::to_string(static_cast<unsigned>(IDLE_MODE)) + " (" +
                    highModeToString(IDLE_MODE) + ")");
        res->res = launchHighModeMacro(stop_seq_);
        return;
    }

    // Single transition
    {
        std::lock_guard<std::mutex> lock(high_cmd_mutex_);
        high_mode_ = requested;
        high_cmd_.mode = requested;
        wait_check_mode_ = true;
        wait_check_count_ = 0;
    }

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
  
    {
        std::lock_guard<std::mutex> lock(high_cmd_mutex_);
        if (high_mode_ != VELOCITY_MODE && high_mode_ != FREE_STAND_MODE) {
            // ROS2 throttle: period in milliseconds + clock
            publish_log("WARN", "Robot not in FREE_STAND_MODE/VELOCITY_MODE, ignoring cmd_vel");
            return;
        }

        // Assumo che la tua convert.hpp abbia rosMsg2Cmd(msg) anche in ROS2.
        // In caso contrario: adatta la firma (es. rosMsg2Cmd(*msg)).
        high_cmd_ = rosMsg2Cmd(*msg);
        high_mode_ = high_cmd_.mode;
        wait_check_mode_ = true;
        wait_check_count_ = 0;
        last_cmd_vel_time_ = this->now();
    }
}

void LeggedSDKInterface::highCmdCallback(const unitree_legged_msgs::msg::HighCmd::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(high_cmd_mutex_);
    high_cmd_ = rosMsg2Cmd(*msg);
    high_mode_ = high_cmd_.mode;
}

void LeggedSDKInterface::lowLevelCmdClbk(const unitree_legged_msgs::msg::LowCmd::SharedPtr msg) {
    const InterfaceState state = getState();
    if (state != InterfaceState::ENABLED_LOW) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Rejecting LowCmd: interface state is %s (requires ENABLED_LOW).",
            stateToString(state).c_str());
        return;
    }

    if (!low_level_verified_.load(std::memory_order_acquire)) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Rejecting LowCmd: low-level mode is not verified.");
        return;
    }

    if (!has_low_state_.load(std::memory_order_acquire)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Rejecting LowCmd: no LowState has been received yet.");
        return;
    }

    double low_state_age_sec = 0.0;
    if (!isLowStateFresh(&low_state_age_sec)) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Rejecting LowCmd: LowState is stale (age: %.6f s, timeout: %.6f s).",
            low_state_age_sec,
            low_state_timeout_sec_);
        return;
    }

    // Keep transport/reserved fields from SDK template and apply only control payload.
    UNITREE_LEGGED_SDK::LowCmd sdk_cmd = lowCmd_SDK_;

    for (int i = 0; i < 12; ++i) {
        sdk_cmd.motorCmd[i] = rosMsg2Cmd(msg->motor_cmd[i]);
    }
    sdk_cmd.bms = rosMsg2Cmd(msg->bms);

    for (int i = 0; i < 40; ++i) {
        sdk_cmd.wirelessRemote[i] = msg->wireless_remote[i];
    }

    if (msg->band_width > 0) {
        sdk_cmd.bandWidth = msg->band_width;
    }

    lowCmd_buf_.write(sdk_cmd);
}

void LeggedSDKInterface::pubOdom(const UNITREE_LEGGED_SDK::HighState & high_state) {
    rclcpp::Time current_time = this->now();
    
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    // Derived from frame_prefix like every other frame this node stamps, instead of the
    // hardcoded "unitree_go1/" that used to apply here and nowhere else.
    odom.header.frame_id = makeFrame("odom");
    odom.child_frame_id = makeFrame("base");

    odom.pose.pose.position.x = high_state.position[0];
    odom.pose.pose.position.y = high_state.position[1];
    odom.pose.pose.position.z = high_state.position[2];

    odom.pose.pose.orientation.x = high_state.imu.quaternion[1];
    odom.pose.pose.orientation.y = high_state.imu.quaternion[2];
    odom.pose.pose.orientation.z = high_state.imu.quaternion[3];
    odom.pose.pose.orientation.w = high_state.imu.quaternion[0];

    odom.twist.twist.linear.x  = high_state.velocity[0];
    odom.twist.twist.linear.y  = high_state.velocity[1];
    odom.twist.twist.angular.z = high_state.yawSpeed;

    odom_pub_->publish(odom);

    if(publish_odom_tf_) {
        // Publish the transform
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = makeFrame("odom");
        odom_tf.child_frame_id = makeFrame("base");

        odom_tf.transform.translation.x = high_state.position[0];
        odom_tf.transform.translation.y = high_state.position[1];
        odom_tf.transform.translation.z = high_state.position[2];

        odom_tf.transform.rotation.x = high_state.imu.quaternion[1];
        odom_tf.transform.rotation.y = high_state.imu.quaternion[2];
        odom_tf.transform.rotation.z = high_state.imu.quaternion[3];
        odom_tf.transform.rotation.w = high_state.imu.quaternion[0];

        tf_broadcaster_->sendTransform(odom_tf);
    }
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
        bool step_already_satisfied = false;

        {
            std::lock_guard<std::mutex> lk(high_cmd_mutex_);

            if (high_mode_ == target) {
                step_already_satisfied = true;
                publish_log("INFO", "Macro step already satisfied -> " +
                            std::to_string(static_cast<unsigned>(target)) + " (" +
                            highModeToString(target) + ") [wait=" +
                            std::to_string(wait_sec) + "s]");
            }

            if (!step_already_satisfied && !checkHighModeTransitionFrom(high_mode_, target)) {
            publish_log("WARN", "Macro aborted: invalid transition from " +
                        std::to_string(static_cast<unsigned>(high_mode_)) + " (" +
                        highModeToString(static_cast<uint8_t>(high_mode_)) + ") to " +
                        std::to_string(static_cast<unsigned>(target)) + " (" +
                        highModeToString(target) + ")");
            stop_macro();
            return;
            }

            if (!step_already_satisfied) {
                // Importante: aggiorna high_mode_ per far funzionare le transizioni step-by-step
                high_mode_ = target;
                high_cmd_.mode = target;
                wait_check_mode_ = true;
                wait_check_count_ = 0;

                publish_log("INFO", "Macro step -> " +
                            std::to_string(static_cast<unsigned>(target)) + " (" +
                            highModeToString(target) + ") [wait=" + std::to_string(wait_sec) + "s]");
            }
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
  { IDLE_MODE, {
      DAMPING_MODE,
      START
  }},
  { FREE_STAND_MODE, {
      VELOCITY_MODE,
      STAND_UP_MODE,
      DAMPING_MODE,
      STOP,
  }},
  { VELOCITY_MODE, {
      FREE_STAND_MODE,
      DAMPING_MODE,
      STOP
  }},
  { STAND_DOWN_MODE, {
      STAND_UP_MODE,
      DAMPING_MODE
  }},
  { STAND_UP_MODE, {
      FREE_STAND_MODE,
      STAND_DOWN_MODE,
      DAMPING_MODE
  }},
  { DAMPING_MODE, {
      IDLE_MODE,
      FREE_STAND_MODE,
      VELOCITY_MODE,
      STAND_DOWN_MODE,
      STAND_UP_MODE,
      DAMPING_MODE,
      RECOVERY_MODE,
      START
  }},
  { RECOVERY_MODE, {
      DAMPING_MODE
  }},
};
