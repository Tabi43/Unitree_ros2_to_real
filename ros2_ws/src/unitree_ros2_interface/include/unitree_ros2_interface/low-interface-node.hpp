#ifndef _UNITREE_ROS_INTERFACE_INTERFACE_HPP_
#define _UNITREE_ROS_INTERFACE_INTERFACE_HPP_

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_msgs/msg/low_cmd.hpp"
#include "unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_msgs/msg/wireless_remote.hpp"
#include "unitree_ros2_interface/convert.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <mutex>
#include <atomic>
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

// Defines from the Unitree Motor Driver
#define REST_MODE                0   
#define CALIBRATION_MODE         1 
#define MOTOR_MODE               2   // Standard mode
#define SETUP_MODE               4
#define ENCODER_MODE             5
#define INIT_TEMP_MODE           6

#define PosStopF 2.146E+9f
#define VelStopF 16000.0f

/*
    0: Low-level mode
    1: High-level mode
*/

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
inline T0 killZeroOffset(T0 a, const T1 limit){
    if((a > -limit) && (a < limit)){
        a = 0;
    }
    return a;
}

// Interface state enumeration for safe operation
enum class InterfaceState {
    DISABLED,           // Interface is disabled, no commands sent
    ENABLING,           // Transition state - preparing to enable
    ENABLED,            // Interface is enabled, normal operation
    DISABLING,          // Transition state - sending safe commands before disabling
    EMERGENCY_STOP      // Emergency stop activated
};

class InterfaceNode : public rclcpp::Node {

    public:
        InterfaceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        ~InterfaceNode();

    UNITREE_LEGGED_SDK::UDP _lowlevel_udp;

    // 1 kHz inner loop
    double dt_send_{0.001};
    double dt_recv_{0.001};

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

    /** @brief Sets the Quality of Service (QoS) profiles for publishers and subscribers. 
     * This method initializes and configures the QoS settings for various ROS2 topics
     * such as IMU data, joint states, wireless remote data, and low-level commands.
    */
    void setQoSProfiles();

    /**
     * @brief Service callback to enable or disable the interface.
    */
    void onSetEnabled(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    /**
     * @brief Initiates an emergency stop by applying safety protocols.
     * This function sends a last command to the robot to lock the motors and stay in the current position.
     * @warning It is intended to be used in emergency situations to prevent damage or injury.
     */
    void safetyStop();

    /**
     * @brief Creates and returns a safe command for the robot.
     * This command locks all motors in their current position with moderate stiffness.
     * @return Safe LowCmd structure
     */
    UNITREE_LEGGED_SDK::LowCmd createSafeCommand();

    /**
     * @brief Sends a safe command immediately and guarantees it's sent.
     * This function bypasses the normal command buffer and sends a safe command directly.
     * @param retries Maximum number of send attempts
     * @return true if command was sent successfully, false otherwise
     */
    bool sendSafeCommandImmediate(int retries = 3);

    /**
     * @brief Changes the interface state with proper logging and safety checks.
     * @param new_state The target state to transition to
     */
    void changeInterfaceState(InterfaceState new_state);

    /**
     * @brief Check if the interface is enabled (ready to accept commands)
     * @return true if interface is in ENABLED state
     */
    bool isEnabled() const { 
        return _interface_state == InterfaceState::ENABLED; 
    }

    /**
     * @brief Get current interface state
     * @return Current InterfaceState
     */
    InterfaceState getState() const { 
        return _interface_state; 
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

    /**
     * @brief Initializes the ROS2 services offered by the interface.
     */
    void initServices();

    void onGetStatus(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void pubJointsState(UNITREE_LEGGED_SDK::LowState& lowState, rclcpp::Time& timestamp);
    void pubImu(UNITREE_LEGGED_SDK::LowState& lowState, rclcpp::Time& timestamp);
    void pubRemoteState(UNITREE_LEGGED_SDK::LowState& lowState);

    // ROS2 subscription callback uses SharedPtr for messages
    void lowLevelCmdClbk(const unitree_legged_msgs::msg::LowCmd::SharedPtr msg);

    UNITREE_LEGGED_SDK::Safety safe_;
    UNITREE_LEGGED_SDK::LowCmd lowCmd_SDK_;
    UNITREE_LEGGED_SDK::LowState lowState_SDK_;

    unitree_legged_msgs::msg::LowState lowState_;
    unitree_legged_msgs::msg::WirelessRemote remote_msg_;

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

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<unitree_legged_msgs::msg::WirelessRemote>::SharedPtr wireless_remote_pub_;
    rclcpp::Subscription<unitree_legged_msgs::msg::LowCmd>::SharedPtr lowCmd_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_log_;

    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr FL_contact_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr FR_contact_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr RL_contact_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr RR_contact_pub_;

    sensor_msgs::msg::JointState joint_states_msg_;
    sensor_msgs::msg::Imu imu_msg_;

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
            
            // Determine what command to send based on state
            switch (_interface_state) {
                case InterfaceState::DISABLED:
                    // Don't send any commands when disabled
                    return;
                    
                case InterfaceState::ENABLING:
                    break;

                case InterfaceState::ENABLED:
                    // Send normal commands from buffer
                    cmd = _lowCmd_buf.read();
                    break;
                    
                case InterfaceState::DISABLING:
                    break;

                case InterfaceState::EMERGENCY_STOP:
                    // Send safe command
                    cmd = createSafeCommand();
                    break;
            }
            
            _lowlevel_udp.SetSend(cmd);
            _lowlevel_udp.Send();
            
            // Handle state transitions after successful send
            if (_interface_state == InterfaceState::DISABLING) {
                _disabling_safe_sends_count++;
                // After sending enough safe commands, transition to disabled
                if (_disabling_safe_sends_count >= _required_safe_sends) {
                    changeInterfaceState(InterfaceState::DISABLED);
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
            switch (_interface_state) {
                case InterfaceState::DISABLED:
                    // Don't send any commands when disabled
                    return;
                case InterfaceState::ENABLING:
                    break;
                case InterfaceState::ENABLED:
                    // Normal operation
                    break;
                case InterfaceState::DISABLING:
                    return;
                case InterfaceState::EMERGENCY_STOP:
                    // Continue receiving to monitor state
                    return;
            }

            _lowlevel_udp.Recv();
            _lowlevel_udp.GetRecv(lowState_SDK_);
            _lowState_buf.write(lowState_SDK_);

        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "UDP Receive error: %s", e.what());
        }
    }

    void initLowCmd();

    void threadState();

    bool checkEmergencyCommand(UNITREE_LEGGED_SDK::LowState& state);

    void pubFeetContact(UNITREE_LEGGED_SDK::LowState& state, rclcpp::Time& timestamp);

    private:

    void declare_and_get_params();
    void validate_params_or_throw();

    static std::string normalize_ns(const std::string & ns);
    std::string make_topic(const std::string & suffix) const;  // suffix relative to <camera_name>
    void publish_log(const std::string & level, const std::string & msg);

    // Interface state management
    InterfaceState _interface_state = InterfaceState::DISABLED;
    
    // Safe command guarantees
    static constexpr int _required_safe_sends = 10;  // Number of safe commands to send before disabling
    int _disabling_safe_sends_count = 0;
    std::mutex _state_mutex;  // Protect state changes

    std::string namespace_param_;

    std::string joint_states_topic_;
    std::string imu_topic_;
    std::string wireless_remote_topic_;
    std::string sdk_cmd_topic_;

    float imu_frequency_{1000};                // [Hz]
    float joint_states_frequency_{500};        // [Hz]
    float remote_frequency_{10};               // [Hz]

    // Swap buffer for low-level data
    SwapBuf<UNITREE_LEGGED_SDK::LowCmd> _lowCmd_buf;        // UDP RX   -> fanout unico
    SwapBuf<UNITREE_LEGGED_SDK::LowState> _lowState_buf;    // sub ROS2 -> UDP TX 1KHz

    // UDP communication loops
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_udpSend;
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_udpRecv;
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_udpSendRecv;

    // Services
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_enabled_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_status_srv_;

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
};

#endif // _UNITREE_ROS_INTERFACE_INTERFACE_HPP_