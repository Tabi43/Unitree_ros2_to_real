/**
 * 
 * 
 * 
*/
#ifndef _UNITREE_ROS_INTERFACE_LEGGED_SDK_INTERFACE_HPP_
#define _UNITREE_ROS_INTERFACE_LEGGED_SDK_INTERFACE_HPP_

// Unitree SDK
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_msgs/msg/low_cmd.hpp"
#include "unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_msgs/msg/high_cmd.h"
#include "unitree_legged_msgs/msg/high_state.h"
#include "unitree_legged_msgs/msg/wireless_remote.hpp"

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// ROS2 Services
#include <unitree_ros2_interface/srv/set_led_color.hpp>

// Cpp
#include <pthread.h>
#include <sched.h>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// Interface includes
#include "unitree_ros2_interface/convert.h"
#include "unitree_ros2_interface/srv/set_high_mode.hpp"

// HIGH LEVEL MODE DEFINES
#define IDLE_MODE 0
#define FREE_STAND_MODE 1
#define VELOCITY_MODE 2
#define STAND_DOWN_MODE 5
#define STAND_UP_MODE 6
#define DAMPING_MODE 7
#define RECOVERY_MODE 8
#define START 10
#define STOP 20 

// Defines from the Unitree Motor Driver
#define REST_MODE                0   
#define CALIBRATION_MODE         1 
#define MOTOR_MODE               2   // Standard mode
#define SETUP_MODE               4
#define ENCODER_MODE             5
#define INIT_TEMP_MODE           6

#define PosStopF UNITREE_LEGGED_SDK::PosStopF
#define VelStopF UNITREE_LEGGED_SDK::VelStopF

// Interface state enumeration for safe operation
enum class InterfaceState {
    DISABLED,                   // Interface is disabled, no commands sent
    ENABLING_LOW,               // Transition state - preparing to enable
    ENABLING_HIGH,
    ENABLED_LOW,
    ENABLED_HIGH,
    DISABLING_LOW,               // Transition state - sending safe commands before disabling
    DISABLING_HIGH,     
    EMERGENCY_STOP_HIGH,          // Emergency stop activated
    EMERGENCY_STOP_LOW           // Emergency stop activated
};

/* Service functions */
template<class T>
struct SwapBuf {
  static_assert(std::is_trivially_copyable<T>::value, "T must be POD (Plain Old Data) type.");
  mutable std::mutex mtx;
  T buf{};

  inline void write(const T& v) {
    std::lock_guard<std::mutex> lock(mtx);
    buf = v;
  }

  inline T read() const {
    std::lock_guard<std::mutex> lock(mtx);
    return buf;
  }
};

template<typename T0, typename T1>
inline T0 killZeroOffset(T0 a, const T1 limit) {
    if((a > -limit) && (a < limit)){
        a = 0;
    }
    return a;
}

/*
 * Return codes of UNITREE_LEGGED_SDK::UDP::Recv(). They are not documented in the SDK
 * headers; these values were established by disassembling libunitree_legged_sdk.a
 * (UDP::Recv), and they are only meaningful when the socket is built with
 * RecvEnum::blockTimeout — see the note on kUdpRecvTimeoutMs below.
 *
 * In blockTimeout mode Recv() runs select() on the socket, then drains every queued
 * datagram so the buffer always holds the NEWEST frame, then validates head + CRC.
 * That makes the return value an exact "a new frame arrived" test, which is what the
 * receive paths, the freshness timestamps and the watchdog are all built on.
 *
 * In the SDK's default nonBlock mode Recv() re-validates the PREVIOUS buffer and
 * returns OK even when no datagram arrived at all, so the return value carries no
 * new-frame information and link loss is undetectable. Do not switch back.
 */
constexpr int UDP_RECV_OK          =  0;   // new frame received, head + CRC valid
constexpr int UDP_RECV_BAD_HEAD    = -1;   // frame dropped: header magic mismatch
constexpr int UDP_RECV_TIMEOUT     = -2;   // select() expired, no datagram (link idle/lost)
constexpr int UDP_RECV_CRC_ERROR   = -3;   // frame dropped: CRC mismatch
constexpr int UDP_RECV_NO_DATA     = -4;   // nonBlock-mode "nothing arrived" (should not occur)

/*
 * select() timeout for a single Recv() call, in milliseconds. It only bounds how long
 * the receive thread parks before reporting UDP_RECV_TIMEOUT; it is not a rate. Keep it
 * well below low_state_timeout_sec so the watchdog observes the loss rather than the
 * receive thread hiding it, and strictly below 1000 (the SDK packs it into tv_usec).
 *
 * DO NOT drop the UDP::SetRecvTimeout() calls that apply this value. Measured against
 * the shipped library (test/test_udp_recv_contract.cpp): Recv() picks the select path
 * on the internal timeout being > 0, NOT on the constructor's RecvEnum. RecvEnum only
 * seeds that value - blockTimeout to 2 ms, nonBlock to -1. Without SetRecvTimeout the
 * socket silently falls back to the polling path where no-data reads report -1 rather
 * than UDP_RECV_TIMEOUT, and the whole liveness layer degrades with it.
 */
constexpr int kUdpRecvTimeoutMs = 5;

class LeggedSDKInterface : public rclcpp::Node {

    public:

    // Constructor & Destructor
    LeggedSDKInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~LeggedSDKInterface();

    /**
     * @brief Sets the sending frequency.
     *
     * This function calculates and updates the sending interval (dt_send) based on the specified frequency.
     * The interval is derived as the reciprocal of the provided frequency (i.e., dt_send = 1.0 / freq).
     *
     * @param freq The desired frequency in Hertz (Hz) at which the messages are sent.
    */
    inline void setSendFrequency(double freq) {
        dt_send_ = 1.0 / freq;
    }

    /**
     * @brief Sets the receive frequency.
     *
     * This method sets the internal receive period (dt_recv) based on the given frequency.
     * The period is calculated as the inverse of the frequency.
     *
     * @param freq The frequency (in Hz) at which data is received.
    */
    inline void setRecvFrequency(double freq) {
        dt_recv_ = 1.0 / freq;
    }

    /**
     * @brief Creates a safe low-level command for emergency/hold situations.
     */
    UNITREE_LEGGED_SDK::LowCmd createSafeLowCommand();

    /**
     * @brief Runs the SDK's Safety guards over a command about to be transmitted. This is
     * the single choke point for every outgoing LowCmd - user commands, safe hold
     * commands and emergency commands alike - so no path can reach the motors unguarded.
     *
     * Behaviour of the three guards was established by disassembling libunitree_legged_sdk.a,
     * since the SDK ships as a binary and its header documents none of this:
     *
     *  - PositionLimit(cmd) clamps ONLY motorCmd[i].q to the Go1 mechanical joint limits
     *    (go1_const.h). It never touches tau, Kp or Kd, so it cannot detune a torque
     *    controller: with Kp == 0 the clamped q is ignored by the motor anyway.
     *  - PowerProtect(cmd, state, factor) sums the measured mechanical power
     *    (sum |tauEst * dq|) and, when it stays over budget, sets ALL twelve
     *    motorCmd[i].mode to 0 (damping) and returns -1. It does NOT scale torques:
     *    it either passes the command through untouched or drops the robot limp.
     *  - PositionProtect(cmd, state, limit) trips the same way on measured joint
     *    positions that exceed the limit table by more than `limit` radians.
     *
     * @param cmd In/out: the command to guard; may be forced to damping by a trip.
     * @return true if the command passed, false if a guard tripped (caller should treat
     *         this as a fault: the command now commands damping, not what was asked for).
     */
    bool applySafetyClamps(UNITREE_LEGGED_SDK::LowCmd & cmd);

    /**
     * @brief Raises the CALLING thread to SCHED_FIFO at udp_thread_priority, once per
     * thread. Called at the top of each UDP loop callback because the SDK gives no handle
     * to the threads its LoopFunc creates.
     *
     * The SDK's loop.h advertises THREAD_PRIORITY = 95, but the shipped
     * libunitree_legged_sdk.a contains no pthread_setschedparam/sched_setscheduler call in
     * Loop::start at all - only pthread_create and pthread_setaffinity_np - so the threads
     * silently inherit whatever the process had. Elevating here puts the two UDP threads
     * above the ROS executor and the DDS threads, which is the whole point: the previous
     * setup ran everything at one blanket priority, so DDS serialization could preempt the
     * command loop.
     *
     * Requires CAP_SYS_NICE and a non-zero RLIMIT_RTPRIO. A failure is reported once per
     * thread at ERROR rather than swallowed: running this loop non-RT is a condition the
     * operator has to know about, not a detail.
     *
     * @param thread_name Name used in the log message.
     */
    inline void applyRtPriorityOnce(const char * thread_name) {
        if (udp_thread_priority_ <= 0) {
            return;
        }
        // thread_local: each LoopFunc thread performs the call exactly once, and the
        // per-iteration cost afterwards is a single predicted branch.
        static thread_local bool applied = false;
        if (applied) {
            return;
        }
        applied = true;

        sched_param param{};
        param.sched_priority = udp_thread_priority_;
        const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
        if (rc != 0) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Could not put '%s' on SCHED_FIFO:%d (%s). The UDP loop is running at "
                "normal priority; grant CAP_SYS_NICE and --ulimit rtprio to the container.",
                thread_name, udp_thread_priority_, std::strerror(rc));
        } else {
            RCLCPP_INFO(this->get_logger(), "Thread '%s' running at SCHED_FIFO:%d.",
                        thread_name, udp_thread_priority_);
        }
    }

    /**
     * @brief Prefixes a TF frame name with frame_prefix, giving "<prefix>/<name>" or the
     * bare name when no prefix is configured. Used so that every frame this node stamps
     * (IMU, feet, odometry) lives in one consistent namespace.
     */
    inline std::string makeFrame(const std::string & name) const {
        return frame_prefix_.empty() ? name : (frame_prefix_ + "/" + name);
    }

    /**
     * @brief Sends a safe command immediately and guarantees it's sent.
     * This function bypasses the normal command buffer and sends a safe command directly.
     * @param retries Maximum number of send attempts
     * @return true if command was sent successfully, false otherwise
     */
    bool sendSafeLowCommandImmediate(int retries = 3);

    /**
     * @brief Changes the interface state with proper logging and safety checks.
     * @param new_state The target state to transition to
     */
    void changeInterfaceState(InterfaceState new_state);

    /**
     * @brief Check if low interface is enabled (ready to accept commands)
     * @return true if interface is in ENABLED_LOW state
     */
    bool isEnabledLow() const { 
        return interface_state_.load(std::memory_order_acquire) == InterfaceState::ENABLED_LOW;
    }
    
    /**
     * @brief Check if high interface is enabled (ready to accept commands)
     * @return true if interface is in ENABLED_HIGH state
     */
    bool isEnabledHigh() const { 
        return interface_state_.load(std::memory_order_acquire) == InterfaceState::ENABLED_HIGH;
    }

    /**
     * @brief Check if the interface is currently disabled (not accepting commands)
     */
    bool isDisabled() const {
        return interface_state_.load(std::memory_order_acquire) == InterfaceState::DISABLED;
    }

    bool enableLowInterface();
    bool disableLowInterface();

    bool enableHighInterface();
    bool disableHighInterface();

    /**
     * @brief Get current interface state
     * @return Current InterfaceState
    */
    InterfaceState getState() const { 
        return interface_state_.load(std::memory_order_acquire);
    }

    /**
     * @brief Get string representation of interface state
     * @param state The state to convert to string
     * @return String representation of the state
     */
    static std::string stateToString(InterfaceState state);

    /**
     * @brief Watchdog timer callback to monitor interface health.
     * This function is called periodically to check the status of the interface
     * and ensure that it is functioning correctly. If any issues are detected,
     * appropriate actions can be taken to maintain safe operation.
     */
    void watchdog();

    void setQoSProfiles();

    /**
     * @brief Initializes the ROS2 services offered by the interface.
    */
    void initServices();

    void onGetStatus(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
    );

    /**
     * @brief Service callback to enable or disable the low interface.
    */
    void onSetLowEnable(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response
    );

    /**
     * @brief Service callback to enable or disable the high interface.
    */
    void onSetHighEnable(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response
    );

    /**
     * @brief Service callback to set the high-level mode of the robot.
      * 
      * This function is a service callback that handles requests to change the high-level mode of the robot.
      * It checks the requested mode against allowed modes and transitions, updates the internal state,
      * and sends appropriate commands to the robot to achieve the desired mode change. The response indicates
      * whether the mode change was successful or if any errors occurred (e.g., invalid mode, disallowed transition).
     */
    void setHighModeCallback(
        const std::shared_ptr<unitree_ros2_interface::srv::SetHighMode::Request> req,
        std::shared_ptr<unitree_ros2_interface::srv::SetHighMode::Response> res
    );

    /**
     * @brief Sends low-level command data via UDP.
     * 
     * This function is our UDP consumer for low-level command data. It attempts to send
     * the current low-level command data over the UDP interface. If an error occurs during
     * the sending process, it catches the exception and logs a warning message.
    */
    inline void lowSend() {
        try {
            UNITREE_LEGGED_SDK::LowCmd cmd;
            const InterfaceState state = interface_state_.load(std::memory_order_acquire);
            
            // Determine what command to send based on state
            switch (state) {
                case InterfaceState::DISABLED:
                    // Don't send any commands when disabled
                    return;
                    
                case InterfaceState::ENABLING_LOW:
                    // During handshake send only safe commands.
                    cmd = createSafeLowCommand();
                    break;

                case InterfaceState::ENABLED_LOW: {
                    double low_state_age_sec = 0.0;
                    const bool verified = low_level_verified_.load(std::memory_order_acquire);
                    const bool fresh_low_state = isLowStateFresh(&low_state_age_sec);
                    if (verified && fresh_low_state) {
                        cmd = lowCmd_buf_.read();
                    } else {
                        cmd = createSafeLowCommand();
                        if (!verified) {
                            RCLCPP_ERROR_THROTTLE(
                                this->get_logger(),
                                *this->get_clock(),
                                1000,
                                "ENABLED_LOW but low-level mode is not verified. Sending safe LowCmd.");
                        }
                        if (!fresh_low_state) {
                            RCLCPP_ERROR_THROTTLE(
                                this->get_logger(),
                                *this->get_clock(),
                                1000,
                                "ENABLED_LOW but LowState is stale (age: %.6f s, timeout: %.6f s). Sending safe LowCmd.",
                                low_state_age_sec,
                                low_state_timeout_sec_);
                        }
                    }
                    break;
                }
                    
                case InterfaceState::DISABLING_LOW:
                    // Send safe hold-position commands until the counter threshold is met.
                    cmd = createSafeLowCommand();
                    break;

                case InterfaceState::EMERGENCY_STOP_LOW:
                    // Send safe command
                    cmd = createSafeLowCommand();
                    break;

                // High-level states: low UDP not active, do nothing
                case InterfaceState::ENABLING_HIGH:
                    return;

                case InterfaceState::ENABLED_HIGH:
                    return;

                case InterfaceState::DISABLING_HIGH:
                    return;

                case InterfaceState::EMERGENCY_STOP_HIGH:
                    return;
            }
            
            // Single guard point for every outgoing command, whatever produced it.
            // A trip rewrites `cmd` to damping, so it is still sent - the robot must
            // receive the damping frame, not nothing.
            if (!applySafetyClamps(cmd)) {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "SDK Safety guard tripped: outgoing LowCmd forced to damping.");
            }

            lowlevel_udp_.SetSend(cmd);
            lowlevel_udp_.Send();
            
            // Handle state transitions after successful send
            if (state == InterfaceState::DISABLING_LOW || state == InterfaceState::EMERGENCY_STOP_LOW) {
                const int safe_sends =
                    _disabling_safe_sends_count.fetch_add(1, std::memory_order_acq_rel) + 1;
                // After sending enough safe commands, transition to disabled
                if (safe_sends >= _required_safe_sends) {
                    changeInterfaceState(InterfaceState::DISABLED);
                    pending_low_cleanup_.store(true, std::memory_order_release);
                    publish_log("INFO", "Low interface disabled after sending safe commands.");
                }
            }
            
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "UDP Send error: %s", e.what());
        }
    }

    /**
     * @brief Receives data from the UDP interface and updates the low-level state. 
     * 
     * This function is our UDP producer for low-level state data. It attempts to receive data
     * from the UDP interface, and upon successful reception, it converts the received data
     * into a ROS message format.
    */
    inline void lowRecive() {
        try {
            const InterfaceState state = interface_state_.load(std::memory_order_acquire);
            switch (state) {
                case InterfaceState::DISABLED:
                    return;
                case InterfaceState::ENABLING_LOW:
                    break;
                case InterfaceState::ENABLED_LOW:
                    // Normal operation
                    break;
                case InterfaceState::DISABLING_LOW:
                    // Keep receiving while shutting down to monitor mode.
                    break;
                case InterfaceState::EMERGENCY_STOP_LOW:
                    // Continue receiving to monitor state
                    break;
                // High-level states: low UDP not active, do nothing
                case InterfaceState::ENABLING_HIGH:
                    return;
                case InterfaceState::ENABLED_HIGH:
                    return;
                case InterfaceState::DISABLING_HIGH:
                    return;
                case InterfaceState::EMERGENCY_STOP_HIGH:
                    return;
            }

            // Recv() parks in select() until a datagram arrives or kUdpRecvTimeoutMs
            // expires, so this call — not a polling period — is what paces the loop.
            // Everything below runs ONLY on UDP_RECV_OK: a timeout or a rejected frame
            // must never refresh the freshness timestamps, or the staleness watchdog
            // would keep re-arming itself against a dead link.
            const int rc = lowlevel_udp_.Recv();
            if (rc != UDP_RECV_OK) {
                countRecvFailure(rc);
                return;
            }
            recv_ok_count_.fetch_add(1, std::memory_order_relaxed);

            lowlevel_udp_.GetRecv(lowState_SDK_);

            // Latch the frame. `now` is the instant the receive thread woke on the
            // packet, so it is a true arrival timestamp (± thread wake-up latency),
            // not the "we happened to poll here" timestamp the old nonBlock loop
            // produced. It stamps every ROS message derived from this frame.
            const rclcpp::Time now = this->now();
            lowState_buf_.write(lowState_SDK_);
            last_low_state_time_ = now;
            last_low_state_time_ns_.store(now.nanoseconds(), std::memory_order_release);
            last_frame_recv_ns_.store(now.nanoseconds(), std::memory_order_release);
            has_low_state_.store(true, std::memory_order_release);

            // Every UDP_RECV_OK is by construction a distinct datagram, so the publisher
            // can be woken unconditionally — no duplicate-frame filtering required.
            {
                std::lock_guard<std::mutex> pub_lk(pub_cv_mutex_);
                pub_new_frame_ = true;
            }
            pub_cv_.notify_one();

            if (state == InterfaceState::ENABLING_LOW) {
                // The robot's LowState.levelFlag is unreliable on this Go1 and cannot be
                // used to confirm low-level mode, so there is no levelFlag handshake.
                // What IS verified here is the link itself: we only get to this point
                // after a real datagram passed the SDK's head + CRC checks, so the robot
                // is demonstrably talking to us before any command path is unblocked.
                low_level_verified_.store(true, std::memory_order_release);
                changeInterfaceState(InterfaceState::ENABLED_LOW);
                publish_log("INFO", "Low interface enabled on first valid LowState frame (no levelFlag handshake). Transitioned to ENABLED_LOW.");
                return;
            }

        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "UDP Receive error: %s", e.what());
        }
    }

    void highUdpSend() {
        const InterfaceState state = interface_state_.load(std::memory_order_acquire);
        switch (state) {
            case InterfaceState::DISABLED:
                return;
            case InterfaceState::ENABLING_LOW:
                return;
            case InterfaceState::ENABLED_LOW:
                return;
            case InterfaceState::DISABLING_LOW:
                return;
            case InterfaceState::EMERGENCY_STOP_LOW:
                return;
            case InterfaceState::ENABLING_HIGH:
                break;
            case InterfaceState::ENABLED_HIGH:
                break;
            case InterfaceState::DISABLING_HIGH:
                // Send safe hold-position commands until the counter threshold is met.
                break;
            case InterfaceState::EMERGENCY_STOP_HIGH:
                // Send safe command
                break;
        }

        UNITREE_LEGGED_SDK::HighCmd cmd{};
        {
            std::lock_guard<std::mutex> lock(high_cmd_mutex_);

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
                    this->get_logger(), *clock, 1000, "No cmd_vel received for %.2f seconds, zeroing velocities for safety", dt_sec);

                    high_cmd_.velocity[0] = 0.0;
                    high_cmd_.velocity[1] = 0.0;
                    high_cmd_.yawSpeed = 0.0;
                }
            }

            if (state == InterfaceState::DISABLING_HIGH || state == InterfaceState::EMERGENCY_STOP_HIGH) {
                high_cmd_.mode = IDLE_MODE;
                high_cmd_.velocity[0] = 0.0;
                high_cmd_.velocity[1] = 0.0;
                high_cmd_.yawSpeed = 0.0;
            }

            cmd = high_cmd_;
        }

        highlevel_udp_.SetSend(cmd);
        highlevel_udp_.Send();

        if (state == InterfaceState::DISABLING_HIGH || state == InterfaceState::EMERGENCY_STOP_HIGH) {
            changeInterfaceState(InterfaceState::DISABLED);
            publish_log("INFO", "High interface disabled after sending safe commands.");
        }
    }

    void highUdpRecv() {
        const InterfaceState state = interface_state_.load(std::memory_order_acquire);

        if (state != InterfaceState::ENABLING_HIGH &&
            state != InterfaceState::ENABLED_HIGH &&
            state != InterfaceState::DISABLING_HIGH &&
            state != InterfaceState::EMERGENCY_STOP_HIGH) {
            return;  // Do not attempt to receive if high interface is not enabled
        }

        // Same contract as lowRecive(): only UDP_RECV_OK means a genuinely new frame,
        // and only a new frame may refresh has_high_state_ / the freshness timestamps.
        // HighState carries no `tick` field, so arrival time is the ONLY timebase
        // available for it — getting it from a real wake-up rather than a poll matters
        // more here than on the low side.
        const int rc = highlevel_udp_.Recv();
        if (rc != UDP_RECV_OK) {
            countRecvFailure(rc);
            return;
        }
        recv_ok_count_.fetch_add(1, std::memory_order_relaxed);

        UNITREE_LEGGED_SDK::HighState received_state{};
        highlevel_udp_.GetRecv(received_state);
        const rclcpp::Time now = this->now();
        last_frame_recv_ns_.store(now.nanoseconds(), std::memory_order_release);
        last_high_state_time_ns_.store(now.nanoseconds(), std::memory_order_release);

        {
            std::lock_guard<std::mutex> state_lock(high_state_mutex_);
            high_state_ = received_state;
            has_high_state_.store(true, std::memory_order_release);
        }

        bool handshake_complete = false;
        std::string mode_mismatch_msg;
        {
            std::lock_guard<std::mutex> cmd_lock(high_cmd_mutex_);

            if (state == InterfaceState::ENABLING_HIGH) {
                high_mode_ = received_state.mode;
                high_cmd_.mode = received_state.mode;
                wait_check_mode_ = false;
                wait_check_count_ = 0;
                handshake_complete = true;
                if (received_state.levelFlag != UNITREE_LEGGED_SDK::HIGHLEVEL) {
                    mode_mismatch_msg = "High interface received HighState with unexpected levelFlag=0x" +
                        std::to_string(static_cast<unsigned>(received_state.levelFlag)) +
                        "; continuing for compatibility with SDK variants.";
                }
            }

            if (state == InterfaceState::ENABLED_HIGH &&
                high_mode_ != received_state.mode && !(wait_check_mode_)) {
                const bool expected_velocity_alias =
                    high_mode_ == VELOCITY_MODE && received_state.mode == FREE_STAND_MODE;
                // if (!expected_velocity_alias) {
                //     mode_mismatch_msg = "Detected different mode on robot; reported mode: " +
                //         std::to_string(static_cast<unsigned>(received_state.mode)) +
                //         " (" + highModeToString(received_state.mode) + "), keeping requested mode: " +
                //         std::to_string(static_cast<unsigned>(high_mode_)) +
                //         " (" + highModeToString(high_mode_) + ")";
                // }
            }

            if(wait_check_count_ <= wait_check_window_ && wait_check_mode_) {
                wait_check_count_++;
            } else if (wait_check_mode_) {
                wait_check_mode_ = false;
                wait_check_count_ = 0;
            }
        }

        if (!mode_mismatch_msg.empty()) {
            publish_log("WARN", mode_mismatch_msg);
        }

        if (handshake_complete) {
            changeInterfaceState(InterfaceState::ENABLED_HIGH);
            publish_log("INFO", "High interface handshake complete on first valid HighState frame. Transitioned to ENABLED_HIGH.");
        }

        {
            std::lock_guard<std::mutex> pub_lk(pub_cv_mutex_);
            pub_new_frame_ = true;
        }
        pub_cv_.notify_one();
    }

    /**
     * @brief Checks if the robot is currently in high-level mode based on the received state.
      * This function checks the current interface state to determine if we are in a high-level mode (enabled, disabling, or emergency stop). 
      * If we are in a high-level state, it further checks the levelFlag in the received high-level state to confirm that it matches the expected value for high-level mode. 
      * This provides an additional layer of verification to ensure that the robot is indeed operating in high-level mode before allowing certain operations or transitions.
     */
    inline bool isRobotInHighMode() {
        auto state = interface_state_.load(std::memory_order_acquire);
        if (state == InterfaceState::ENABLED_HIGH || state == InterfaceState::DISABLING_HIGH || state == InterfaceState::EMERGENCY_STOP_HIGH) {
            std::lock_guard<std::mutex> lock(high_state_mutex_);
            return high_state_.levelFlag == UNITREE_LEGGED_SDK::HIGHLEVEL;
        } else if (state == InterfaceState::ENABLED_LOW || state == InterfaceState::DISABLING_LOW || state == InterfaceState::EMERGENCY_STOP_LOW) {
            if (!has_low_state_.load(std::memory_order_acquire)) {
                return false;
            }
            return lowState_buf_.read().levelFlag == UNITREE_LEGGED_SDK::HIGHLEVEL;
        }
        return false;
    }

    /**
     * @brief Checks if the robot is currently in low-level mode based on the received state.
      * This function checks the current interface state to determine if we are in a low-level mode (enabled, disabling, or emergency stop). 
      * If we are in a low-level state, it further checks the levelFlag in the received low-level state to confirm that it does not match the value for high-level mode. 
      * This provides an additional layer of verification to ensure that the robot is indeed operating in low-level mode before allowing certain operations or transitions.
     */
    inline bool isRobotInLowMode() {
        auto state = interface_state_.load(std::memory_order_acquire);
        if (state == InterfaceState::ENABLED_HIGH || state == InterfaceState::DISABLING_HIGH || state == InterfaceState::EMERGENCY_STOP_HIGH) {
            std::lock_guard<std::mutex> lock(high_state_mutex_);
            return high_state_.levelFlag == UNITREE_LEGGED_SDK::LOWLEVEL;
        } else if (state == InterfaceState::ENABLED_LOW || state == InterfaceState::DISABLING_LOW || state == InterfaceState::EMERGENCY_STOP_LOW) {
            if (!has_low_state_.load(std::memory_order_acquire)) {
                return false;
            }
            return lowState_buf_.read().levelFlag == UNITREE_LEGGED_SDK::LOWLEVEL;
        }
        return false;
    }

    inline bool isLowStateFresh(double * age_sec = nullptr) const {
        if (!has_low_state_.load(std::memory_order_acquire)) {
            if (age_sec != nullptr) {
                *age_sec = -1.0;
            }
            return false;
        }

        const int64_t stamp_ns = last_low_state_time_ns_.load(std::memory_order_acquire);
        if (stamp_ns <= 0) {
            if (age_sec != nullptr) {
                *age_sec = -1.0;
            }
            return false;
        }

        const int64_t now_ns = this->now().nanoseconds();
        const double age = static_cast<double>(now_ns - stamp_ns) * 1e-9;
        if (age_sec != nullptr) {
            *age_sec = age;
        }

        return age >= 0.0 && age < low_state_timeout_sec_;
    }

    /**
     * @brief HighState counterpart of isLowStateFresh(): true while the most recent
     * accepted HighState frame is younger than high_state_timeout_sec_. Because
     * HighState has no `tick` field, the age is measured from the frame's arrival
     * instant recorded by highUdpRecv().
     * @param age_sec Optional out-parameter: age of the latest frame in seconds, or
     *                -1.0 when no frame has ever been accepted.
     */
    inline bool isHighStateFresh(double * age_sec = nullptr) const {
        const int64_t stamp_ns = last_high_state_time_ns_.load(std::memory_order_acquire);
        if (!has_high_state_.load(std::memory_order_acquire) || stamp_ns <= 0) {
            if (age_sec != nullptr) {
                *age_sec = -1.0;
            }
            return false;
        }

        const double age = static_cast<double>(this->now().nanoseconds() - stamp_ns) * 1e-9;
        if (age_sec != nullptr) {
            *age_sec = age;
        }

        return age >= 0.0 && age < high_state_timeout_sec_;
    }

    /**
     * @brief Tallies a non-OK UDP::Recv() result into the per-cause counters exported by
     * publishUdpDiagnostics(). Called from both receive paths on every rejected read, so
     * an idle link (timeouts), a wiring/MTU fault (bad head) and a noisy link (CRC
     * errors) stay distinguishable instead of collapsing into one "no data" symptom.
     * @param rc The value returned by UDP::Recv().
     */
    inline void countRecvFailure(int rc) {
        switch (rc) {
            case UDP_RECV_TIMEOUT:   recv_timeout_count_.fetch_add(1, std::memory_order_relaxed); break;
            case UDP_RECV_CRC_ERROR: recv_crc_err_count_.fetch_add(1, std::memory_order_relaxed); break;
            case UDP_RECV_BAD_HEAD:  recv_head_err_count_.fetch_add(1, std::memory_order_relaxed); break;
            default:                 recv_other_err_count_.fetch_add(1, std::memory_order_relaxed); break;
        }
    }

    /**
     * @brief Zeroes the UDP receive tallies and the diagnostics rate baseline. Called
     * from both enable paths so that the counters describe the current session only and
     * the first reported frame rate is not skewed by the previous one.
     */
    inline void resetRecvCounters() {
        recv_ok_count_.store(0, std::memory_order_relaxed);
        recv_timeout_count_.store(0, std::memory_order_relaxed);
        recv_crc_err_count_.store(0, std::memory_order_relaxed);
        recv_head_err_count_.store(0, std::memory_order_relaxed);
        recv_other_err_count_.store(0, std::memory_order_relaxed);
        last_diag_ok_count_.store(0, std::memory_order_relaxed);
        last_diag_time_ns_.store(this->now().nanoseconds(), std::memory_order_relaxed);
    }

    // Check if mode transition is allowed
    inline bool checkHighModeTransition(unsigned int new_mode) {
        std::lock_guard<std::mutex> lock(high_cmd_mutex_);
        return checkHighModeTransitionFrom(high_mode_, new_mode);
    }

    inline bool checkHighModeTransitionFrom(uint8_t current_mode, unsigned int new_mode) const {
        auto it = allowed_transitions_.find(current_mode);
        if (it == allowed_transitions_.end())
            return false;
        const auto & possible = it->second;
        return possible.count(static_cast<uint8_t>(new_mode)) > 0;
    }

    /**
     * @brief Callback function for receiving velocity commands from ROS2 topics.
     */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief Callback function for receiving high-level command messages from ROS2 topics.
     */
    void highCmdCallback(const unitree_legged_msgs::msg::HighCmd::SharedPtr msg);

    /**
     * @brief Executes a predefined sequence of mode transitions as a macro.
      * This function takes a sequence of mode transitions (each defined by a target mode and a duration to hold that mode) and executes them in order. 
      * It checks for allowed transitions before executing each step and ensures that the interface is in the correct state for each transition. 
      * The function runs asynchronously to avoid blocking the main thread, and it handles timing and state management for the macro execution.
     */
    bool launchHighModeMacro(const std::vector<std::pair<uint8_t, double>> & sequence);

    /**
     * @brief Creates a safe low-level command to be sent in emergency situations.
     */
    void initLowCmd();

    /**
     * @brief Main loop for handling interface state and communication.
     */
    void threadState();

    /**
     * @brief Dedicated publisher thread body.
     *
     * Sleeps on pub_cv_ until the UDP receive path signals that a new robot frame
     * has been latched (or until shutdown). On each wake it reads the latest state
     * from the swap buffer and publishes the ROS sensor streams, applying per-stream
     * rate gates (dueForPublish). Running publishing here keeps DDS serialization off
     * both the real-time UDP send loop and the ROS executor.
     */
    void publisherThreadLoop();

    /**
     * @brief Starts the dedicated publisher thread and resets its per-stream rate
     * gates. Must be called after the publishers exist and before the UDP receive
     * loop is started, so the first received frame already has a live consumer.
     */
    void startPublisherThread();

    /**
     * @brief Signals the publisher thread to stop and joins it. Must be called after
     * the UDP receive LoopFunc has been reset (so no further frames are signalled)
     * and before the publishers are destroyed (so no publish races a reset()).
     */
    void stopPublisherThread();

    /**
     * @brief Per-stream rate gate: returns true (and advances last_pub_sec) when at
     * least 1/freq_hz seconds have elapsed since the previous publish. Evaluated on
     * real received frames, so it decimates the true sensor stream without aliasing.
     * @param now_sec Current time in seconds.
     * @param last_pub_sec In/out timestamp of the last publish for this stream.
     * @param freq_hz Target publish frequency in Hz.
     */
    static bool dueForPublish(double now_sec, double & last_pub_sec, double freq_hz);

    /**
     * @brief Sends a safe low-level command immediately, bypassing the normal command buffer.
     * This is used during the disabling process to ensure the robot receives safe commands.
     */
    void safetyLowStop();

    /**
     * @brief Checks if the received low-level state indicates an emergency condition that requires an immediate stop.
     * @param state The low-level state received from the robot
     * @return true if an emergency condition is detected, false otherwise
     */
    bool checkEmergencyCommand(std::array<uint8_t, 40>& remote_data);

    /**
     * @brief Declares and retrieves ROS parameters.
     */
    void declare_and_get_params();

    /**
     * @brief Validates the retrieved parameters and throws an exception if any parameter is invalid.
      * This function checks the values of the parameters against expected ranges or formats. 
     */
    void validate_params_or_throw();

    /**
     * @brief Normalizes a namespace string to ensure it is in the correct format for ROS topics.
     * @param ns The namespace string to normalize
     * @return A normalized namespace string that can be used for topic names
     */
    static std::string normalize_ns(const std::string & ns);

    /**
     * @brief Constructs a full topic name by combining the camera name with a given suffix. 
     */
    std::string make_topic(const std::string & suffix) const;  // suffix relative to <camera_name>
    
    /**
     * @brief Callback function for receiving low-level command messages from ROS2 topics.
     * @param msg The low-level command message received from the topic
     */
    void lowLevelCmdClbk(const unitree_legged_msgs::msg::LowCmd::SharedPtr msg);

    /**
     * @brief Publishes a log message with a specified severity level.
     * @param level The severity level of the log message (e.g., "INFO", "WARN", "ERROR")
     * @param msg The log message to publish    
     */
    void publish_log(const std::string & level, const std::string & msg);

    /**
     * @brief Publishes the joint states of the robot.
     * @param motorState The motor state data received from the robot
     * @param timestamp The timestamp of the received state
     */
    void pubJointsState(std::array<UNITREE_LEGGED_SDK::MotorState, 20>& motorState, rclcpp::Time& timestamp);

    /**
     * @brief Publishes the IMU data of the robot.
     * @param imu The IMU data received from the robot
     * @param timestamp The timestamp of the received state
     */
    void pubImu(UNITREE_LEGGED_SDK::IMU& imu, rclcpp::Time& timestamp);

    /**
     * @brief Publishes the low-level state of the robot as a ROS message.
     * @param lowState The exact frame the caller published every other topic from.
     *                 Passed in rather than re-read from lowState_buf_ so that
     *                 /low_state cannot describe a newer frame than the /imu,
     *                 /joint_states and wrench messages that share its timestamp.
     */
    void pubLowState(const UNITREE_LEGGED_SDK::LowState & lowState);

    /**
     * @brief Publishes UDP link health (accepted frames, measured frame rate, and
     * timeout / CRC / header-error counts, alongside the SDK's own UDPState tallies)
     * as a DiagnosticArray. This is the only place the link quality is observable:
     * a rising timeout count means the robot stopped sending, a rising CRC count
     * means the frames arrive corrupted, and the measured rate is what the robot
     * actually delivers rather than what the loop period requests.
     */
    void publishUdpDiagnostics();

    /**
     * @brief Publishes the wireless remote data of the robot.
     * @param wirelessRemote The wireless remote data received from the robot
     */
    void pubRemoteState(std::array<uint8_t, 40>& wirelessRemote);

    /**
     * @brief Publishes the battery management system (BMS) state of the robot.
      * @param bmsState The BMS state data received from the robot
     */
    void pubBmsState(UNITREE_LEGGED_SDK::BmsState& bmsState);

    /**
     * @brief Publishes foot contact information.
     * @param state The low-level state received from the robot
     * @param timestamp The timestamp of the received state
     */
    void pubFeetContact(std::array<int16_t, 4>& forces, rclcpp::Time& timestamp);

    /**
     * @brief Publishes the odometry information of the robot.
     * @param lowState The low-level state data received from the robot, which contains odometry information
     * @param timestamp The timestamp of the received state
     */
    void pubOdom(const UNITREE_LEGGED_SDK::HighState & high_state);

    /**
    * @brief Converts a high-level mode value to a human-readable string for logging purposes.
     * @param mode The high-level mode value to convert
     * @return A string representation of the high-level mode
    */
    inline const char * highModeToString(uint8_t mode) const {
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

    private:

    /**
     * @brief Release all ROS2 and UDP resources belonging to the low interface.
     * Must be called ONLY from the ROS2 executor thread (e.g. threadState timer),
     * never from inside a LoopFunc callback, to avoid destroying a thread from itself.
     */
    void cleanupLowResources();

    /**
     * @brief Release all ROS2 and UDP resources belonging to the high interface.
     * Same threading constraint as cleanupLowResources().
     */
    void cleanupHighResources();

    // Safe command guarantees
    static constexpr int _required_safe_sends = 10;  // Number of safe commands to send before disabling
    std::atomic_int _disabling_safe_sends_count{0};
    std::mutex state_mutex_;  // Protect state changes
    mutable std::mutex high_cmd_mutex_;
    mutable std::mutex high_state_mutex_;

    // Pending cleanup flags: set from any thread, consumed by threadState (ROS2 timer thread).
    // This ensures LoopFunc objects are always destroyed outside their own callback.
    std::atomic_bool pending_low_cleanup_{false};
    std::atomic_bool pending_high_cleanup_{false};
    std::atomic_bool has_low_state_{false};
    std::atomic_bool has_high_state_{false};
    std::atomic<bool> low_level_verified_{false};

    // Interface state management
    std::atomic<InterfaceState> interface_state_{InterfaceState::DISABLED};

    std::string namespace_param_{""};

    std::string joint_states_topic_;
    std::string imu_topic_;
    std::string wireless_remote_topic_;
    std::string sdk_cmd_topic_;
    std::string odom_topic_;
    std::string cmd_vel_topic_;
    std::string bms_topic_;

    // All frequency/period members are double to match the ROS2 parameter type
    // declared with declare_parameter<double>. Using float here would cause
    // rclcpp::exceptions::InvalidParameterTypeException at startup.
    // imu/joints/feet publish at full frame rate (no gate): gating them near the
    // robot's ~900 Hz rate would beat and drop frames. Only the slow streams below
    // (remote, odom) are decimated.
    double remote_frequency_{10.0};            // [Hz]
    double odom_frequency_{100.0};             // [Hz]
    // BMS changes on the robot at a few Hz at most; publishing it per frame was pure
    // duplicate traffic. /low_state is the largest message this node emits (~1 kB), so
    // it gets its own gate: 0.0 means full frame rate, anything else decimates it.
    double bms_frequency_{1.0};                // [Hz]
    double low_state_frequency_{0.0};          // [Hz], 0 = every frame
    double dt_send_{0.001};                    // Send period (s) - default 1 kHz
    double dt_recv_{0.001};                    // Receive period (s) - default 1 kHz
    float soc_threshold_{20.0};                // Battery State of Charge threshold for emergency stop (%)

    // Last-publish timestamps (seconds) for the decimated streams. Touched only by
    // the publisher thread, so no atomics are needed.
    double last_remote_pub_sec_{0.0};
    double last_odom_pub_sec_{0.0};
    double last_bms_pub_sec_{0.0};
    double last_low_state_pub_sec_{0.0};

    // ---- Safety guards on the outgoing LowCmd (see applySafetyClamps) ----
    bool enable_position_limit_{true};
    // 0 disables PowerProtect; 1..10 selects 10%..100% of the SDK's watt budget.
    // Defaults to the most permissive setting that is still a limit: the guard trips to
    // damping rather than scaling torques, so a nuisance trip would drop the robot.
    int power_protect_factor_{10};
    // 0.0 disables PositionProtect. Disabled by default on purpose: it trips on MEASURED
    // joint angles outside the SDK limit table, and a Go1 sitting on its hocks rests with
    // the calf near -2.818 rad while go1_const.h declares go1_Calf_min = -2.721. The
    // resting pose is already ~0.1 rad outside the table, past the 0.087 rad default
    // tolerance, so enabling this would trip during every bring-up.
    double position_protect_limit_{0.0};

    // ---- Real-time thread placement ----
    // SCHED_FIFO priority for the two UDP loop threads; 0 leaves them alone. Applied by
    // applyRtPriorityOnce because the SDK never sets a priority itself.
    int udp_thread_priority_{80};
    // CPU to pin each UDP loop to, -1 for unpinned. They used to share a hardcoded core 3,
    // so the send and receive loops contended for one CPU. Unpinned by default: pinning to
    // a core outside the container's cpuset just makes the SDK print "Set affinity failed"
    // and carry on unpinned anyway.
    int udp_send_cpu_{-1};
    int udp_recv_cpu_{-1};
    // mlockall() at startup. Without it, a page fault in the command path costs
    // milliseconds of jitter at exactly the wrong moment.
    bool lock_memory_{true};

    // ---- Frame naming ----
    // Prefix applied to every TF frame this node stamps. Previously the odometry frames
    // were hardcoded to "unitree_go1/..." while the IMU and foot frames had no prefix at
    // all, so they could never join the same TF tree.
    std::string frame_prefix_{""};

    // ---- Foot force conditioning ----
    // LowState::footForce is a RAW int16 count from the foot airbag sensor, not newtons,
    // and each foot carries its own bias. These give the calibration knob: the published
    // value is (raw - offset) * scale, per foot, in SDK leg order {FR, FL, RR, RL}. With
    // the defaults the output is unchanged raw counts, which is honest but uncalibrated.
    std::vector<double> foot_force_offset_{0.0, 0.0, 0.0, 0.0};
    std::vector<double> foot_force_scale_{1.0, 1.0, 1.0, 1.0};

    // ---- IMU noise model ----
    // Published as the diagonal of the sensor_msgs/Imu covariances. An all-zero
    // covariance means "known exactly" to REP-145 consumers such as robot_localization,
    // which is the one thing these values must never be. The yaw entry is deliberately
    // large: the Go1's onboard fusion has no heading reference, so yaw drifts without
    // bound.
    std::vector<double> imu_orientation_stddev_{0.01, 0.01, 0.5};   // [rad] roll, pitch, yaw
    double imu_angular_velocity_stddev_{0.01};                      // [rad/s]
    double imu_linear_acceleration_stddev_{0.1};                    // [m/s^2]

    // Time / params
    rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_low_state_time_{0, 0, RCL_ROS_TIME};
    std::atomic<int64_t> high_enable_start_time_ns_{0};
    std::atomic<int64_t> last_low_state_time_ns_{0};
    // Arrival instant (ns, node clock) of the latest ACCEPTED HighState frame; the
    // high-level counterpart of last_low_state_time_ns_, used by isHighStateFresh().
    std::atomic<int64_t> last_high_state_time_ns_{0};
    // Receive instant (ns, node clock) of the latest frame. Written by the UDP
    // receive path, read by the publisher thread to stamp sensor messages at the
    // true measurement time rather than at publish time.
    std::atomic<int64_t> last_frame_recv_ns_{0};
    double cmd_vel_timeout_{0.5};
    double low_state_timeout_sec_{0.1};
    double high_state_timeout_sec_{0.5};
    bool wait_check_mode_{false};
    int wait_check_window_{500};      // [tick]
    int wait_check_count_{0};
    int startup_mode_{0};             // 0: DISABLED, 1: HIGH, 2: LOW
    bool publish_odom_tf_{false};        // Whether to publish odometry transform

    // High Level Unitree Mode — uint8_t matches high_cmd_.mode and SDK constants
    uint8_t high_mode_ = 0;

    // Swap buffer for low-level data
    SwapBuf<UNITREE_LEGGED_SDK::LowCmd>     lowCmd_buf_;        // UDP RX   -> fanout unico
    SwapBuf<UNITREE_LEGGED_SDK::LowState>   lowState_buf_;      // sub ROS2 -> UDP TX 1KHz

    // UDP communication loops
    UNITREE_LEGGED_SDK::Safety safe_;

    // Low Level SDK data structures
    UNITREE_LEGGED_SDK::LowCmd lowCmd_SDK_;
    UNITREE_LEGGED_SDK::LowState lowState_SDK_;

    // High Level SDK data structures
    UNITREE_LEGGED_SDK::HighState high_state_{};
    UNITREE_LEGGED_SDK::HighCmd high_cmd_{};

    // Allowed transitions (define contents in .cpp)
    static const std::unordered_set<uint8_t> allowed_modes_;
    static const std::unordered_map<uint8_t, std::unordered_set<uint8_t>> allowed_transitions_;
    std::atomic_bool macro_running_{false};

    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_udpSend;
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_udpRecv;
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_udpSendRecv;

    // Dedicated publisher thread, woken by the UDP receive path on each new frame.
    std::thread              pub_thread_;
    std::mutex               pub_cv_mutex_;      // guards pub_new_frame_
    std::condition_variable  pub_cv_;
    bool                     pub_new_frame_{false};
    std::atomic_bool         pub_thread_run_{false};

    // UDP receive tallies, written by the receive thread and read by the diagnostics
    // timer. Shared by both levels because only one interface is ever active at a
    // time; they are reset on every enable so each session starts from zero.
    std::atomic<uint64_t> recv_ok_count_{0};
    std::atomic<uint64_t> recv_timeout_count_{0};
    std::atomic<uint64_t> recv_crc_err_count_{0};
    std::atomic<uint64_t> recv_head_err_count_{0};
    std::atomic<uint64_t> recv_other_err_count_{0};

    // Previous diagnostics sample, used to turn the cumulative counters above into a
    // measured frame rate. Atomic because the diagnostics timer and the enable path
    // (via resetRecvCounters) can run concurrently under the MultiThreadedExecutor.
    std::atomic<uint64_t> last_diag_ok_count_{0};
    std::atomic<int64_t>  last_diag_time_ns_{0};

    UNITREE_LEGGED_SDK::UDP lowlevel_udp_;
    UNITREE_LEGGED_SDK::UDP highlevel_udp_;

    // ROS2 subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<unitree_legged_msgs::msg::HighCmd>::SharedPtr high_cmd_sub_;

    // ROS2 publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<unitree_legged_msgs::msg::WirelessRemote>::SharedPtr wireless_remote_pub_;
    rclcpp::Subscription<unitree_legged_msgs::msg::LowCmd>::SharedPtr lowCmd_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_log_;
    rclcpp::Publisher<unitree_legged_msgs::msg::BmsState>::SharedPtr bms_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr FL_contact_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr FR_contact_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr RL_contact_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr RR_contact_pub_;
    rclcpp::Publisher<unitree_legged_msgs::msg::LowState>::SharedPtr low_state_pub_;
    // Created in the constructor and never torn down with an interface: link health
    // must stay observable across enable/disable cycles.
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Services
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_enable_low_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_enable_high_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_status_low_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_status_high_srv_;
    rclcpp::Service<unitree_ros2_interface::srv::SetHighMode>::SharedPtr mode_service_;
    rclcpp::Client<unitree_ros2_interface::srv::SetLedColor>::SharedPtr set_led_color_srv_;

    // ROS2 msgs
    unitree_legged_msgs::msg::LowState lowState_;
    unitree_legged_msgs::msg::WirelessRemote remote_msg_;
    unitree_legged_msgs::msg::BmsState bms_msg_;
    sensor_msgs::msg::JointState joint_states_msg_;
    sensor_msgs::msg::Imu imu_msg_;

    // Timers
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    // Quality of Service profiles
    std::shared_ptr<rclcpp::QoS> imu_qos_;
    std::shared_ptr<rclcpp::QoS> joint_state_qos_;
    std::shared_ptr<rclcpp::QoS> wireless_remote_qos_;
    std::shared_ptr<rclcpp::QoS> lowcmd_qos_;

    /*  Unitree use a different leg indexing by default
        
        Correct order is: FL, FR, RL, RR
        Unitre order is: FR, FL, RR, RL

        This map is used to adapt the correct order of legs joints
    */
    int legs_[4] = {
        UNITREE_LEGGED_SDK::FL_,
        UNITREE_LEGGED_SDK::FR_,
        UNITREE_LEGGED_SDK::RL_,
        UNITREE_LEGGED_SDK::RR_
    };

    int joints_[12] = {  
        UNITREE_LEGGED_SDK::FL_0,
        UNITREE_LEGGED_SDK::FL_1,
        UNITREE_LEGGED_SDK::FL_2,
        UNITREE_LEGGED_SDK::FR_0,
        UNITREE_LEGGED_SDK::FR_1,
        UNITREE_LEGGED_SDK::FR_2,
        UNITREE_LEGGED_SDK::RL_0,
        UNITREE_LEGGED_SDK::RL_1,
        UNITREE_LEGGED_SDK::RL_2,
        UNITREE_LEGGED_SDK::RR_0,
        UNITREE_LEGGED_SDK::RR_1,
        UNITREE_LEGGED_SDK::RR_2
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

};

#endif // _UNITREE_ROS_INTERFACE_LEGGED_SDK_INTERFACE_HPP_
