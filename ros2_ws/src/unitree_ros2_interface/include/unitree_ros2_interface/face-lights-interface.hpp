#ifndef UNITREE_ROS2_INTERFACE_FACE_LIGHTS_INTERFACE_HPP
#define UNITREE_ROS2_INTERFACE_FACE_LIGHTS_INTERFACE_HPP

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// ROS2 service headers (adatta al nome reale generato dal tuo package)
#include <unitree_ros2_interface/srv/set_led_color.hpp>
#include <unitree_ros2_interface/srv/set_led_animation.hpp>

#include <unitree_ros2_interface/face-lights-animations.hpp>

// Unitree client (come nel tuo codice)
#include <FaceLightClient.h>

enum class LED_STATE {
  STATIC,
  ANIMATION,
};

class FaceLedNode : public rclcpp::Node {

public:
    FaceLedNode();
    ~FaceLedNode();

private:
    void onSetLedColor(const std::shared_ptr<unitree_ros2_interface::srv::SetLedColor::Request> req, std::shared_ptr<unitree_ros2_interface::srv::SetLedColor::Response> res);

    void onSetFaceAnimation(const std::shared_ptr<unitree_ros2_interface::srv::SetLedAnimation::Request> req, std::shared_ptr<unitree_ros2_interface::srv::SetLedAnimation::Response> res);

    void onTick();

    void publish_log(const std::string & level, const std::string & msg);

    std::shared_ptr<FaceLightClient> face_light_client_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_log_;
    rclcpp::Service<unitree_ros2_interface::srv::SetLedColor>::SharedPtr srv_color_;
    rclcpp::Service<unitree_ros2_interface::srv::SetLedAnimation>::SharedPtr srv_anim_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Alive publisher settings/state
    int alive_period_ms_;
    rclcpp::Time last_alive_time_;

    std::mutex mtx_;
    std::array<uint8_t, 3> led_color_;
    LED_STATE state_;
    bool timed_color_active_ = false;
    rclcpp::Time timed_color_deadline_;
    unsigned int ticks_;

    // Animation playback state
    const LedAnimation * current_anim_ = nullptr;
    size_t anim_frame_idx_ = 0;
    uint32_t anim_frame_elapsed_ms_ = 0;

    void applyFrame(const LedFrame & frame);
    void stopAnimation();
};

#endif // UNITREE_ROS2_INTERFACE_FACE_LIGHTS_INTERFACE_HPP