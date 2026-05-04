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
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// ROS2 Services
#include <unitree_ros2_interface/srv/set_led_color.hpp>

// Cpp
#include <mutex>
#include <atomic>
#include <cstdint>
#include <cstdio>

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
  alignas(64) std::atomic<uint8_t> idx{0};
  alignas(64) T buf[2];

  inline void write(const T& v) {
    const uint8_t r = idx.load(std::memory_order_relaxed);
    const uint8_t w = 1u - r;
    buf[w] = v;
    idx.store(w, std::memory_order_release);
  }
  inline T read() const {
    const uint8_t r = idx.load(std::memory_order_acquire);
    return buf[r];
  }
};

template<typename T0, typename T1>
inline T0 killZeroOffset(T0 a, const T1 limit) {
    if((a > -limit) && (a < limit)){
        a = 0;
    }
    return a;
}

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

            lowlevel_udp_.Recv();
            lowlevel_udp_.GetRecv(lowState_SDK_);

            const uint8_t level_flag = static_cast<uint8_t>(lowState_SDK_.levelFlag);
            const bool robot_is_low = (lowState_SDK_.levelFlag == UNITREE_LEGGED_SDK::LOWLEVEL);

            if (state == InterfaceState::ENABLING_LOW) {
                // Handshake: accept state only if the robot confirms low-level mode.
                if (robot_is_low) {
                    lowState_buf_.write(lowState_SDK_);
                    const rclcpp::Time now = this->now();
                    last_low_state_time_ = now;
                    last_low_state_time_ns_.store(now.nanoseconds(), std::memory_order_release);
                    has_low_state_.store(true, std::memory_order_release);
                    low_level_verified_.store(true, std::memory_order_release);
                    changeInterfaceState(InterfaceState::ENABLED_LOW);
                    publish_log("INFO", "Low interface handshake complete. levelFlag=LOWLEVEL. Transitioned to ENABLED_LOW.");
                    auto led_req = std::make_shared<unitree_ros2_interface::srv::SetLedColor::Request>();
                    led_req->r = 0;
                    led_req->g = 0;
                    led_req->b = 255;
                    led_req->time = 5.0;
                    set_led_color_srv_->async_send_request(led_req);
                } else {
                    publish_log("ERROR",
                        "Low interface handshake failed: robot is not in LOWLEVEL (levelFlag=0x" +
                        std::to_string(level_flag) + "). Initiating disable.");
                    disableLowInterface();
                    auto led_req = std::make_shared<unitree_ros2_interface::srv::SetLedColor::Request>();
                    led_req->r = 255;
                    led_req->g = 0;
                    led_req->b = 0;
                    led_req->time = 5.0;
                    set_led_color_srv_->async_send_request(led_req);
                }
                return;
            }

            // ENABLED_LOW / DISABLING_LOW / EMERGENCY_STOP_LOW: normal state update.
            lowState_buf_.write(lowState_SDK_);
            const rclcpp::Time now = this->now();
            last_low_state_time_ = now;
            last_low_state_time_ns_.store(now.nanoseconds(), std::memory_order_release);
            has_low_state_.store(true, std::memory_order_release);

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
                break;
            case InterfaceState::ENABLED_LOW:
                break;
            case InterfaceState::DISABLING_LOW:
                break;
            case InterfaceState::EMERGENCY_STOP_LOW:
                break;
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

        if (state == InterfaceState::DISABLING_HIGH) {
            changeInterfaceState(InterfaceState::DISABLED);
            publish_log("INFO", "High interface disabled after sending safe commands.");
        }

        highlevel_udp_.SetSend(high_cmd_);
        highlevel_udp_.Send();
    }

    void highUdpRecv() {

        if(!isEnabledHigh()) {
            return;  // Do not attempt to receive if high interface is not enabled
        }

        highlevel_udp_.Recv();
        highlevel_udp_.GetRecv(high_state_);

        if (high_mode_ != high_state_.mode && !(wait_check_mode_)) {
            high_mode_ = high_state_.mode;
            publish_log("WARN", "Detected different mode on robot; actual mode set: " +
            to_string(static_cast<unsigned>(high_state_.mode)));
        }

        if(wait_check_count_ <= wait_check_window_ && wait_check_mode_) {
            wait_check_count_++;
        } else if (wait_check_mode_) {
            wait_check_mode_ = false;
            wait_check_count_ = 0;
        }
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
            return high_state_.levelFlag == UNITREE_LEGGED_SDK::HIGHLEVEL;
        } else if (state == InterfaceState::ENABLED_LOW || state == InterfaceState::DISABLING_LOW || state == InterfaceState::EMERGENCY_STOP_LOW) {
            return lowState_SDK_.levelFlag != UNITREE_LEGGED_SDK::HIGHLEVEL;
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
            return high_state_.levelFlag == UNITREE_LEGGED_SDK::LOWLEVEL;
        } else if (state == InterfaceState::ENABLED_LOW || state == InterfaceState::DISABLING_LOW || state == InterfaceState::EMERGENCY_STOP_LOW) {
            return lowState_SDK_.levelFlag != UNITREE_LEGGED_SDK::LOWLEVEL;
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

    // Check if mode transition is allowed
    inline bool checkHighModeTransition(unsigned int new_mode) {
        auto it = allowed_transitions_.find(static_cast<uint8_t>(high_mode_));
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
     * @param timestamp The timestamp of the received state
     */
    void pubLowState();

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
    void pubOdom();

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

    // Pending cleanup flags: set from any thread, consumed by threadState (ROS2 timer thread).
    // This ensures LoopFunc objects are always destroyed outside their own callback.
    std::atomic_bool pending_low_cleanup_{false};
    std::atomic_bool pending_high_cleanup_{false};
    std::atomic_bool has_low_state_{false};
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
    double imu_frequency_{1000.0};             // [Hz]
    double joint_states_frequency_{500.0};     // [Hz]
    double remote_frequency_{10.0};            // [Hz]
    double odom_frequency_{100.0};             // [Hz]
    double dt_send_{0.001};                    // Send period (s) - default 1 kHz
    double dt_recv_{0.001};                    // Receive period (s) - default 1 kHz
    float soc_threshold_{20.0};                // Battery State of Charge threshold for emergency stop (%)

    // Time / params
    rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_low_state_time_{0, 0, RCL_ROS_TIME};
    std::atomic<int64_t> last_low_state_time_ns_{0};
    double cmd_vel_timeout_{0.5};
    double low_state_timeout_sec_{0.1};
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
    std::mutex high_mode_mtx_;

    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_udpSend;
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_udpRecv;
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_udpSendRecv;

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

    // Quality of Service profiles
    std::shared_ptr<rclcpp::QoS> imu_qos_;
    std::shared_ptr<rclcpp::QoS> joint_state_qos_;
    std::shared_ptr<rclcpp::QoS> wireless_remote_qos_;
    std::shared_ptr<rclcpp::QoS> lowcmd_qos_;

    // Remote data struct 
    xRockerBtnDataStruct _remoteKeyData;

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
