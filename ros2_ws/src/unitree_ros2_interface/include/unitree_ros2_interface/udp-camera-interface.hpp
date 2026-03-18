#ifndef UNITREE_ROS2_INTERFACE_UDP_CAMERA_INTERFACE_HPP_
#define UNITREE_ROS2_INTERFACE_UDP_CAMERA_INTERFACE_HPP_

#include <atomic>
#include <chrono>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include <unitree_ros2_interface/udp-camera/udp_camera_receiver.hpp> 

namespace unitree_ros2_interface {

class UnitreeUdpCameraInterface : public rclcpp::Node {
public:
  explicit UnitreeUdpCameraInterface(const rclcpp::NodeOptions & options);
  ~UnitreeUdpCameraInterface() override;

private:
  enum class OutputEncoding {
    BGR8,
    RGB8,
    MONO8
  };

  void declare_and_get_params();
  void validate_params_or_throw();
  void load_camera_infos_best_effort();
  void start_capture_thread();
  void stop_capture_thread();
  void capture_loop();

  static std::string normalize_ns(const std::string & ns);
  std::string make_topic(const std::string & suffix) const;
  void publish_log(const std::string & level, const std::string & msg);

  bool load_camera_info_from_url(
    const std::string & url,
    sensor_msgs::msg::CameraInfo & out_info,
    const std::string & fallback_name);

  static inline bool has_scheme(const std::string& s) {
    return s.find("://") != std::string::npos;  // file://, package://...
  }

  std::string build_camera_info_url(
    const std::string& package_name,
    const std::string& calib_filename_or_url);

  void publishStereoMonoFromBGR(
    const cv::Mat& left_bgr,
    const cv::Mat& right_bgr,
    const rclcpp::Time& stamp);

  static std::string upper(const std::string & s);

private:
  // ---- params (come il tuo nodo) ----
  std::string namespace_param_;
  std::string camera_name_;

  int raw_width_{940};     // “expected” (non usato per acquisizione, solo check/warn)
  int raw_height_{400};
  double fps_{30.0};       // opzionale (fallback rate/logica)

  std::string stereo_layout_{"side_by_side"};
  bool swap_lr_{false};

  std::string encoding_str_{"bgr8"};
  std::string encoding_{sensor_msgs::image_encodings::BGR8};
  OutputEncoding output_encoding_{OutputEncoding::BGR8};

  std::string left_frame_id_;
  std::string right_frame_id_;

  std::string camera_info_left_name_;
  std::string camera_info_right_name_;

  bool use_image_transport_{false};
  bool publish_mono_{true};

  // ---- UDP params ----
  std::string udp_bind_ip_{"0.0.0.0"};
  int udp_bind_port_{5000};
  int udp_stream_id_{0};
  int udp_frame_timeout_ms_{80};
  int udp_max_inflight_{16};
  bool udp_store_jpeg_{false};
  int rx_wait_timeout_ms_{50};
  int udp_mode_{1};

  bool debug_stats_{true};
  int debug_stats_period_ms_{1000};

  // ---- pubs ----
  image_transport::Publisher pub_left_image_transport_;
  image_transport::Publisher pub_right_image_transport_;
  image_transport::Publisher pub_left_image_mono_transport_;
  image_transport::Publisher pub_right_image_mono_transport_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_image_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_image_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_image_mono_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_image_mono_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_left_info_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_right_info_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_log_;

  // ---- camera info cached ----
  sensor_msgs::msg::CameraInfo left_info_;
  sensor_msgs::msg::CameraInfo right_info_;
  bool left_info_loaded_{false};
  bool right_info_loaded_{false};

  // ---- receiver + loop ----
  std::unique_ptr<UdpCameraReceiver> rx_;
  std::atomic<bool> running_{false};
  std::thread capture_thread_;

  uint64_t last_published_frame_id_{static_cast<uint64_t>(-1)};

  // Reused buffers to avoid per-frame allocations.
  sensor_msgs::msg::Image left_color_msg_;
  sensor_msgs::msg::Image right_color_msg_;
  sensor_msgs::msg::Image left_mono_msg_;
  sensor_msgs::msg::Image right_mono_msg_;
};

}  // namespace unitree_ros2_interface

#endif  // UNITREE_ROS2_INTERFACE_UDP_CAMERA_INTERFACE_HPP_

