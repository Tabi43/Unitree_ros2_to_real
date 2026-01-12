#include <array>
#include <cstdint>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// ROS2 service headers (adatta al nome reale generato dal tuo package)
#include <unitree_ros2_interface/srv/set_led_color.hpp>
#include <unitree_ros2_interface/srv/set_led_animation.hpp>

// Unitree client (come nel tuo codice)
#include <FaceLightClient.h>

enum class LED_STATE {
  STATIC,
  ANIMATION,
};

class FaceLedNode : public rclcpp::Node
{
public:
  FaceLedNode()
  : Node("unitree_face_lights_interface"),
    led_color_{0, 0, 0},
    state_(LED_STATE::STATIC),
    ticks_(0)
  {
    face_light_client_ = std::make_shared<FaceLightClient>();
    if (!face_light_client_) {
      throw std::runtime_error("Failed to create FaceLightClient");
    }

    // Spegni LED all'avvio
    face_light_client_->setAllLed(face_light_client_->black);
    face_light_client_->sendCmd();

    // Publisher per i log
    pub_log_ = this->create_publisher<std_msgs::msg::String>("unitree_go1/face_led_log", 10);

    // Servizi
    srv_color_ = this->create_service<unitree_ros2_interface::srv::SetLedColor>(
      "unitree_go1/set_face_color",
      std::bind(&FaceLedNode::onSetLedColor, this, std::placeholders::_1, std::placeholders::_2));

    srv_anim_ = this->create_service<unitree_ros2_interface::srv::SetLedAnimation>(
      "unitree_go1/set_face_animation",
      std::bind(&FaceLedNode::onSetFaceAnimation, this, std::placeholders::_1, std::placeholders::_2));

    // Timer a 10 Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&FaceLedNode::onTick, this));

    publish_log("INFO", "Unitree ROS2 Face LED node started");
  }

private:
  // ---- callbacks servizi ----
  void onSetLedColor(
    const std::shared_ptr<unitree_ros2_interface::srv::SetLedColor::Request> req,
    std::shared_ptr<unitree_ros2_interface::srv::SetLedColor::Response> res)
  {
    // Se i campi srv sono uint8, il check 0..255 è inutile; lo lascio “compatibile”.
    const std::array<uint8_t, 3> color{static_cast<uint8_t>(req->r),
                                       static_cast<uint8_t>(req->g),
                                       static_cast<uint8_t>(req->b)};

    {
      std::lock_guard<std::mutex> lock(mtx_);
      led_color_ = color;
      state_ = LED_STATE::STATIC;
    }

    res->res = true;
  }

  void onSetFaceAnimation(
    const std::shared_ptr<unitree_ros2_interface::srv::SetLedAnimation::Request> req,
    std::shared_ptr<unitree_ros2_interface::srv::SetLedAnimation::Response> res) {
    std::lock_guard<std::mutex> lock(mtx_);

    publish_log("INFO", "Setting face LED animation ID: " + std::to_string(req->id));

    switch (req->id) {
      case 1:
        state_ = LED_STATE::ANIMATION;
        ticks_ = 0;
        res->res = true;
        break;

      default:
        publish_log("INFO", "Unknown LED animation ID: " + std::to_string(req->id));
        state_ = LED_STATE::STATIC;
        led_color_ = {0, 0, 0};
        res->res = false;
        break;
    }
  }

  // ---- loop 10Hz ----
  void onTick() {
    LED_STATE state_local;
    std::array<uint8_t, 3> color_local;

    {
      std::lock_guard<std::mutex> lock(mtx_);
      state_local = state_;
      color_local = led_color_;
    }

    if (state_local == LED_STATE::STATIC) {
      face_light_client_->setAllLed(color_local.data());
      face_light_client_->sendCmd();
    } else {
      // TODO: implementa animazioni reali (ticks_ ecc.)
      ++ticks_;
    }
  }

  void publish_log(const std::string & level, const std::string & msg) {
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

private:
  std::shared_ptr<FaceLightClient> face_light_client_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_log_;
  rclcpp::Service<unitree_ros2_interface::srv::SetLedColor>::SharedPtr srv_color_;
  rclcpp::Service<unitree_ros2_interface::srv::SetLedAnimation>::SharedPtr srv_anim_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mtx_;
  std::array<uint8_t, 3> led_color_;
  LED_STATE state_;
  unsigned int ticks_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<FaceLedNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
