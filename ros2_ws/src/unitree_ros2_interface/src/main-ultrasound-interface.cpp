// unitree_ultrasound_interface.cpp
// ROS 2 Humble skeleton: class-based node with parameter handling only.

#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/range.hpp"

#include <UltraSound_UART.h>

class UnitreeUltrasoundInterface : public rclcpp::Node {
public:
  explicit UnitreeUltrasoundInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("unitree_ultrasound_interface", options) {
    
    declare_and_get_params();
    validate_params_or_throw();

    pub_log_ = this->create_publisher<std_msgs::msg::String>("ultrasound_" + board_ + "_interface/logs", 10);

    if(board_ == "raspi") {
        pub_left_ = this->create_publisher<sensor_msgs::msg::Range>(make_topic(topic_left_), rclcpp::SensorDataQoS());
        pub_right_ = this->create_publisher<sensor_msgs::msg::Range>(make_topic(topic_right_), rclcpp::SensorDataQoS());

        uLeft = std::make_shared<UltraSound>(1);   // Left ultrasound on RasPi
        uRight = std::make_shared<UltraSound>(2);  // Right ultrasound on Ras
    } else if(board_ == "nano") {
        pub_forward_ = this->create_publisher<sensor_msgs::msg::Range>(make_topic(topic_forward_), rclcpp::SensorDataQoS());

        uForward = std::make_shared<UltraSound>(0); // Forward ultrasound on Nano
    }

    run();
  }

  private:
  // ---------- Params (add here what you need later) ----------
  double ultrasound_pub_frequency_{10.0};
  double forward_distance_{0.0};
  double left_distance_{0.0};
  double right_distance_{0.0};

  std::string namespace_param_{""};

  // Range parameters
  std::string topic_left_{"range/left"};
  std::string topic_right_{"range/right"};
  std::string topic_forward_{"range/forward"};

  std::string frame_left_{"ultraSound_left"};
  std::string frame_right_{"ultraSound_right"};
  std::string frame_forward_{"ultraSound_face"};

  double field_of_view_left_{0.5};
  double field_of_view_right_{0.5};
  double field_of_view_forward_{0.5};

  double min_range_left_{0.02};
  double min_range_right_{0.02};
  double min_range_forward_{0.02};

  double max_range_left_{4.0};
  double max_range_right_{4.0};
  double max_range_forward_{4.0};

  // Example: board-based behaviour (e.g., "raspi" or "nano")
  std::string board_{"raspi"};

  // ROS Pubblishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_log_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_left_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_right_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_forward_;

  std::shared_ptr<UltraSound> uLeft;        // Left ultrasound on RasPi
  std::shared_ptr<UltraSound> uRight;       // Right ultrasound on RasPi
  std::shared_ptr<UltraSound> uForward;     // Forward ultrasound on Nano

  std::string normalize_ns(const std::string & ns) const {
    std::string out = ns;
    while (!out.empty() && out.front() == '/') out.erase(out.begin());
    while (!out.empty() && out.back() == '/') out.pop_back();
    return out;
  }

  std::string make_topic(const std::string & suffix) const {
    // Desired convention: namespace/camera_name/(left|right)/image_raw
    const std::string desired = normalize_ns(namespace_param_);
    const std::string node_ns = this->get_namespace();  // "/" or "/unitree_go1"

    // If node already has a namespace, do NOT double-prefix.
    const bool node_has_ns = (node_ns != "/" && !node_ns.empty());
    const bool use_param_ns = (!desired.empty() && !node_has_ns);

    const std::string prefix = use_param_ns ? ("/" + desired) : std::string("");
    return prefix + "/" + suffix;
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

  // ---------- Methods you asked ----------
  void declare_and_get_params() {
    // Declare (must be declared before get_parameter in ROS 2)
    this->declare_parameter<std::string>("namespace", "");
    this->declare_parameter<double>("ultrasound_pub_frequency", 10.0);

    this->declare_parameter<std::string>("board", "");

    this->declare_parameter<std::string>("range_left.frame", "ultraSound_left");
    this->declare_parameter<std::string>("range_left.topic", "range/left"); 
    this->declare_parameter<double>("range_left.field_of_view", 0.5);   
    this->declare_parameter<double>("range_left.min_range", 0.02);
    this->declare_parameter<double>("range_left.max_range", 4.0);

    this->declare_parameter<std::string>("range_right.frame", "ultraSound_right");    
    this->declare_parameter<std::string>("range_right.topic", "range/right");
    this->declare_parameter<double>("range_right.field_of_view", 0.5);    
    this->declare_parameter<double>("range_right.min_range", 0.02);
    this->declare_parameter<double>("range_right.max_range", 4.0);

    this->declare_parameter<std::string>("range_face.frame", "ultraSound_face");
    this->declare_parameter<std::string>("range_face.topic", "range/forward");
    this->declare_parameter<double>("range_face.field_of_view", 0.5);
    this->declare_parameter<double>("range_face.min_range", 0.02);
    this->declare_parameter<double>("range_face.max_range", 4.0);

    // Get
    this->get_parameter("ultrasound_pub_frequency", ultrasound_pub_frequency_);
    this->get_parameter("namespace", namespace_param_);

    this->get_parameter("board", board_);

    this->get_parameter("range_left.topic", topic_left_);
    this->get_parameter("range_left.frame", frame_left_);
    this->get_parameter("range_left.field_of_view", field_of_view_left_);
    this->get_parameter("range_left.min_range", min_range_left_);
    this->get_parameter("range_left.max_range", max_range_left_);

    this->get_parameter("range_right.topic", topic_right_);
    this->get_parameter("range_right.frame", frame_right_);
    this->get_parameter("range_right.field_of_view", field_of_view_right_);
    this->get_parameter("range_right.min_range", min_range_right_);
    this->get_parameter("range_right.max_range", max_range_right_);

    this->get_parameter("range_face.topic", topic_forward_);
    this->get_parameter("range_face.frame", frame_forward_);
    this->get_parameter("range_face.field_of_view", field_of_view_forward_);
    this->get_parameter("range_face.min_range", min_range_forward_);
    this->get_parameter("range_face.max_range", max_range_forward_);
  }

  void validate_params_or_throw() {

    if (ultrasound_pub_frequency_ <= 0.0) {
      throw std::invalid_argument("Parameter 'ultrasound_frequency' must be > 0");
    }

    if (board_ != "raspi" && board_ != "RasPi" && board_ != "nano" && board_ != "Nano") { 
      throw std::invalid_argument("Parameter 'board' must be either 'raspi' or 'nano'");
    } else if(board_ == "RasPi" ) board_ = "raspi";
    else if(board_ == "Nano" ) board_ = "nano"; 

    auto non_empty_or_throw = [](const std::string & name, const std::string & value) {
      if (value.empty()) {
        throw std::invalid_argument("Parameter '" + name + "' must be non-empty");
      }
    };

    // Validate parameters based on board type
    if (board_ == "raspi") {
      // RasPi has left and right sensors
      non_empty_or_throw("range_left.topic", topic_left_);
      non_empty_or_throw("range_left.frame", frame_left_);
      non_empty_or_throw("range_right.topic", topic_right_);
      non_empty_or_throw("range_right.frame", frame_right_);
      
      if (field_of_view_left_ <= 0.0 || min_range_left_ <= 0.0 || max_range_left_ <= 0.0) {
        throw std::invalid_argument("Left range parameters must be > 0");
      }
      if (field_of_view_right_ <= 0.0 || min_range_right_ <= 0.0 || max_range_right_ <= 0.0) {
        throw std::invalid_argument("Right range parameters must be > 0");
      }
    } else if (board_ == "nano") {
      // Nano has forward/face sensor
      non_empty_or_throw("range_face.topic", topic_forward_);
      non_empty_or_throw("range_face.frame", frame_forward_);
      
      if (field_of_view_forward_ <= 0.0 || min_range_forward_ <= 0.0 || max_range_forward_ <= 0.0) {
        throw std::invalid_argument("Forward range parameters must be > 0");
      }
    }

  }

  void run() {
    publish_log("INFO", "Ultrasound interface node started on board '" + board_ + "'");

    rclcpp::Rate rate(ultrasound_pub_frequency_);
    
    while (rclcpp::ok()) {
      // Read distances from ultrasound sensors
      auto now = this->now();
      
      if(board_ == "raspi") {
          uLeft->measureDistance(left_distance_);    // in meters
          uRight->measureDistance(right_distance_);  // in meters

          // Publish left ultrasound range
          sensor_msgs::msg::Range left_msg;
          left_msg.header.stamp = now;
          left_msg.header.frame_id = frame_left_;
          left_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
          left_msg.field_of_view = field_of_view_left_;
          left_msg.min_range = min_range_left_;
          left_msg.max_range = max_range_left_;
          left_msg.range = left_distance_;
          pub_left_->publish(left_msg);

          // Publish right ultrasound range
          sensor_msgs::msg::Range right_msg;
          right_msg.header.stamp = now;
          right_msg.header.frame_id = frame_right_;
          right_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
          right_msg.field_of_view = field_of_view_right_;
          right_msg.min_range = min_range_right_;
          right_msg.max_range = max_range_right_;
          right_msg.range = right_distance_;
          pub_right_->publish(right_msg);

      } else if(board_ == "nano") {
          uForward->measureDistance(forward_distance_); // in meters

          // Publish forward ultrasound range
          sensor_msgs::msg::Range forward_msg;
          forward_msg.header.stamp = now;
          forward_msg.header.frame_id = frame_forward_;
          forward_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
          forward_msg.field_of_view = field_of_view_forward_;
          forward_msg.min_range = min_range_forward_;
          forward_msg.max_range = max_range_forward_;
          forward_msg.range = forward_distance_;
          pub_forward_->publish(forward_msg);
      }

      rclcpp::spin_some(this->get_node_base_interface());
      rate.sleep();
    }
  }
};

// Minimal main (so it compiles/runs; you can keep or replace)
int main(int argc, char ** argv){
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<UnitreeUltrasoundInterface>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("unitree_ultrasound_interface"),
                 "Startup failed: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
