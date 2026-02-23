#include "unitree_ros2_interface/udp-camera-interface.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// camera_calibration_parsers (standard ROS camera calibration YAML)
#include <camera_calibration_parsers/parse_yml.hpp>

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <cctype>
#include <cstring>
#include <fstream>

using namespace std::chrono_literals;

namespace unitree_ros2_interface {

UnitreeUdpCameraInterface::UnitreeUdpCameraInterface(const rclcpp::NodeOptions & options)
: rclcpp::Node("unitree_udp_camera_interface", options) {
  declare_and_get_params();
  pub_log_ = this->create_publisher<std_msgs::msg::String>(make_topic("log"), rclcpp::QoS(10).reliable());

  validate_params_or_throw();
  load_camera_infos_best_effort();

  // publishers
  rmw_qos_profile_t sensor_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();

  if (use_image_transport_) {
    publish_log("INFO", "Publishing images using image_transport.");
    pub_left_image_transport_  = image_transport::create_publisher(this, make_topic("left/image_raw"),  sensor_qos);
    pub_right_image_transport_ = image_transport::create_publisher(this, make_topic("right/image_raw"), sensor_qos);
  } else {
    publish_log("INFO", "Publishing images using rclcpp publishers.");
    pub_left_image_  = this->create_publisher<sensor_msgs::msg::Image>(make_topic("left/image_raw"),  rclcpp::SensorDataQoS());
    pub_right_image_ = this->create_publisher<sensor_msgs::msg::Image>(make_topic("right/image_raw"), rclcpp::SensorDataQoS());
  }

  if (publish_mono_) {
    if (use_image_transport_) {
      publish_log("INFO", "Publishing mono images using image_transport.");
      pub_left_image_mono_transport_  = image_transport::create_publisher(this, make_topic("left/image_mono"),  sensor_qos);
      pub_right_image_mono_transport_ = image_transport::create_publisher(this, make_topic("right/image_mono"), sensor_qos);
    } else {
      publish_log("INFO", "Publishing mono images using rclcpp publishers.");
      pub_left_image_mono_  = this->create_publisher<sensor_msgs::msg::Image>(make_topic("left/image_mono"),  rclcpp::SensorDataQoS());
      pub_right_image_mono_ = this->create_publisher<sensor_msgs::msg::Image>(make_topic("right/image_mono"), rclcpp::SensorDataQoS());
    }
  }

  pub_left_info_  = this->create_publisher<sensor_msgs::msg::CameraInfo>(make_topic("left/camera_info"),  rclcpp::SensorDataQoS());
  pub_right_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(make_topic("right/camera_info"), rclcpp::SensorDataQoS());

  // encoding (ora coerente: se chiedi rgb8 converto BGR->RGB, ecc.)
  const std::string enc = upper(encoding_str_);

  if (enc == "BGR8") {
    encoding_ = sensor_msgs::image_encodings::BGR8;
  } else if (enc == "RGB8") {
    encoding_ = sensor_msgs::image_encodings::RGB8;
  } else if (enc == "MONO8") {
    encoding_ = sensor_msgs::image_encodings::MONO8;
  } else {
    encoding_ = sensor_msgs::image_encodings::BGR8;
    publish_log("WARN", "Unsupported encoding '" + encoding_str_ + "'; defaulting to 'bgr8'.");
  }

  // Receiver
  UdpCameraReceiverConfig cfg;
  cfg.bind_ip = udp_bind_ip_;
  cfg.bind_port = udp_bind_port_;
  cfg.expected_stream_id = static_cast<uint16_t>(udp_stream_id_);
  cfg.frame_timeout_ms = udp_frame_timeout_ms_;
  cfg.max_inflight = udp_max_inflight_;
  cfg.store_jpeg = udp_store_jpeg_;

  rx_ = std::make_unique<UdpCameraReceiver>(cfg);
  if (!rx_->start()) {
    throw std::runtime_error("UdpCameraReceiver start() failed");
  }

  publish_log("INFO", "UnitreeUdpCameraInterface started; publishing stereo frames from UDP.");
  start_capture_thread();
}

UnitreeUdpCameraInterface::~UnitreeUdpCameraInterface()
{
  stop_capture_thread();
  if (rx_) rx_->stop();
  publish_log("INFO", "UnitreeUdpCameraInterface stopped.");
}

void UnitreeUdpCameraInterface::declare_and_get_params()
{
  // Base
  this->declare_parameter<std::string>("namespace", "");
  this->declare_parameter<std::string>("camera_name", "bottom_camera");

  this->declare_parameter<int>("raw_width", 940);
  this->declare_parameter<int>("raw_height", 400);
  this->declare_parameter<double>("raw_fps", 30.0);

  this->declare_parameter<std::string>("stereo_layout", "side_by_side");
  this->declare_parameter<bool>("swap_lr", false);

  this->declare_parameter<std::string>("encoding", "bgr8");

  this->declare_parameter<std::string>("left_frame_id", "");
  this->declare_parameter<std::string>("right_frame_id", "");

  this->declare_parameter<std::string>("camera_info_left_name", "");
  this->declare_parameter<std::string>("camera_info_right_name", "");

  this->declare_parameter<bool>("use_image_transport", false);
  this->declare_parameter<bool>("publish_mono", true);


  // UDP
  this->declare_parameter<int>("UDP.mode", 1);
  this->declare_parameter<std::string>("UDP.bind_ip", "0.0.0.0");
  this->declare_parameter<int>("UDP.bind_port", 5000);
  this->declare_parameter<int>("UDP.stream_id", 0);
  this->declare_parameter<int>("UDP.frame_timeout_ms", 80);
  this->declare_parameter<int>("UDP.max_inflight", 16);
  this->declare_parameter<bool>("UDP.store_jpeg", false);
  this->declare_parameter<int>("UDP.rx_wait_timeout_ms", 50);

  // Get
  this->get_parameter("namespace", namespace_param_);
  this->get_parameter("camera_name", camera_name_);

  this->get_parameter("raw_width", raw_width_);
  this->get_parameter("raw_height", raw_height_);
  this->get_parameter("raw_fps", fps_);

  this->get_parameter("stereo_layout", stereo_layout_);
  this->get_parameter("swap_lr", swap_lr_);

  this->get_parameter("encoding", encoding_str_);

  this->get_parameter("left_frame_id", left_frame_id_);
  this->get_parameter("right_frame_id", right_frame_id_);

  this->get_parameter("camera_info_left_name", camera_info_left_name_);
  this->get_parameter("camera_info_right_name", camera_info_right_name_);

  this->get_parameter("use_image_transport", use_image_transport_);
  this->get_parameter("publish_mono", publish_mono_);

  this->get_parameter("UDP.mode", udp_mode_);
  this->get_parameter("UDP.bind_ip", udp_bind_ip_);
  this->get_parameter("UDP.bind_port", udp_bind_port_);
  this->get_parameter("UDP.stream_id", udp_stream_id_);
  this->get_parameter("UDP.frame_timeout_ms", udp_frame_timeout_ms_);
  this->get_parameter("UDP.max_inflight", udp_max_inflight_);
  this->get_parameter("UDP.store_jpeg", udp_store_jpeg_);
  this->get_parameter("UDP.rx_wait_timeout_ms", rx_wait_timeout_ms_);

  if (left_frame_id_.empty())  left_frame_id_  = camera_name_ + "_left_optical";
  if (right_frame_id_.empty()) right_frame_id_ = camera_name_ + "_right_optical";
}

void UnitreeUdpCameraInterface::validate_params_or_throw()
{
  if (camera_name_.empty()) throw std::runtime_error("camera_name is empty");

  switch (udp_mode_) {
    case 0: publish_log("INFO", "Mode set to 0 (read NewOnly)"); break;
    case 1: publish_log("INFO", "Mode set to 1 (read Oldest)"); break;
    case 2: publish_log("INFO", "Mode set to 2 (wait for new)"); break;
    default:
      throw std::runtime_error("UDP.mode must be 0, 1, or 2");
  }

  stereo_layout_ = upper(stereo_layout_);
  if (stereo_layout_ != "SIDE_BY_SIDE") {
    throw std::runtime_error("Only stereo_layout=side_by_side is supported");
  }

  if (raw_width_ <= 0 || raw_height_ <= 0) {
    publish_log("WARN", "raw_width/raw_height <= 0: will not use them for validation.");
  } else if ((raw_width_ % 2) != 0) {
    throw std::runtime_error("raw_width must be even for side-by-side stereo");
  }

  if (udp_bind_port_ <= 0 || udp_bind_port_ > 65535) {
    throw std::runtime_error("UDP.bind_port out of range");
  }
  if (udp_stream_id_ < 0 || udp_stream_id_ > 65535) {
    throw std::runtime_error("UDP.stream_id out of range");
  }
  if (rx_wait_timeout_ms_ < 1) rx_wait_timeout_ms_ = 1;
}

void UnitreeUdpCameraInterface::start_capture_thread() {
  running_.store(true);
  publish_log("INFO", "Starting capture thread...");
  capture_thread_ = std::thread(&UnitreeUdpCameraInterface::capture_loop, this);
}

void UnitreeUdpCameraInterface::stop_capture_thread()
{
  running_.store(false);
  publish_log("INFO", "Stopping capture thread...");
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
}

void UnitreeUdpCameraInterface::capture_loop()
{
  while (rclcpp::ok() && running_.load()) {
    std::shared_ptr<const ReceivedCameraFrame> f;
    
    bool got = false;
    switch (udp_mode_){
      case 0:
        got = rx_->read(f, CameraReadMode::NewOnly);
        break;
      case 1:
        got = rx_->read(f, CameraReadMode::LatestAlways);
        break;
      case 2:
        got = rx_->waitForNew(f, std::chrono::milliseconds(rx_wait_timeout_ms_));
        break;
      default:
        break;
    }
    
    if (!got || !f || f->bgr.empty()) {
      continue;
    }

    cv::Mat frame = f->bgr;  // SBS BGR
    if ((frame.cols % 2) != 0) {
      publish_log("ERROR", "Received frame width is not even; cannot split side-by-side");
      continue;
    }

    if (raw_width_ > 0 && raw_height_ > 0) {
      if (frame.cols != raw_width_ || frame.rows != raw_height_) {
        publish_log("WARN", "Received frame size " + std::to_string(frame.cols) + "x" + std::to_string(frame.rows) +
                            " differs from expected raw_* " + std::to_string(raw_width_) + "x" + std::to_string(raw_height_));
      }
    }

    const int half_w = frame.cols / 2;
    cv::Mat left_bgr  = frame(cv::Rect(0,      0, half_w, frame.rows));
    cv::Mat right_bgr = frame(cv::Rect(half_w, 0, half_w, frame.rows));

    if (swap_lr_) std::swap(left_bgr, right_bgr);

    const rclcpp::Time stamp = this->now();

    // encoding conversion (input sempre BGR)
    cv::Mat left_out, right_out;
    if (encoding_ == sensor_msgs::image_encodings::BGR8) {
      left_out = left_bgr;
      right_out = right_bgr;
    } else if (encoding_ == sensor_msgs::image_encodings::RGB8) {
      cv::cvtColor(left_bgr, left_out, cv::COLOR_BGR2RGB);
      cv::cvtColor(right_bgr, right_out, cv::COLOR_BGR2RGB);
    } else if (encoding_ == sensor_msgs::image_encodings::MONO8) {
      cv::cvtColor(left_bgr, left_out, cv::COLOR_BGR2GRAY);
      cv::cvtColor(right_bgr, right_out, cv::COLOR_BGR2GRAY);
    } else {
      left_out = left_bgr;
      right_out = right_bgr;
    }

    std_msgs::msg::Header hl; hl.stamp = stamp; hl.frame_id = left_frame_id_;
    std_msgs::msg::Header hr; hr.stamp = stamp; hr.frame_id = right_frame_id_;

    cv_bridge::CvImage left_msg(hl, encoding_, left_out);
    cv_bridge::CvImage right_msg(hr, encoding_, right_out);

    if (use_image_transport_) {
      pub_left_image_transport_.publish(left_msg.toImageMsg());
      pub_right_image_transport_.publish(right_msg.toImageMsg());
    } else {
      pub_left_image_->publish(*left_msg.toImageMsg());
      pub_right_image_->publish(*right_msg.toImageMsg());
    }

    if (publish_mono_) {
      // mono dedicato (sempre da BGR originale)
      publishStereoMonoFromBGR(left_bgr, right_bgr, stamp);
    }

    // CameraInfo
    sensor_msgs::msg::CameraInfo li = left_info_;
    sensor_msgs::msg::CameraInfo ri = right_info_;
    li.header.stamp = stamp;
    ri.header.stamp = stamp;
    li.header.frame_id = left_frame_id_;
    ri.header.frame_id = right_frame_id_;
    pub_left_info_->publish(li);
    pub_right_info_->publish(ri);
  }
}

void UnitreeUdpCameraInterface::publishStereoMonoFromBGR(
  const cv::Mat& left_bgr, const cv::Mat& right_bgr, const rclcpp::Time& stamp)
{
  if (left_bgr.empty() || right_bgr.empty()) return;
  if (left_bgr.size() != right_bgr.size()) return;
  if (left_bgr.type() != CV_8UC3 || right_bgr.type() != CV_8UC3) return;

  const int w = left_bgr.cols;
  const int h = left_bgr.rows;
  const size_t mono_bytes = static_cast<size_t>(w) * static_cast<size_t>(h);

  sensor_msgs::msg::Image left_msg_mono, right_msg_mono;

  auto ensure_alloc = [&](sensor_msgs::msg::Image& msg) {
    msg.height = static_cast<uint32_t>(h);
    msg.width  = static_cast<uint32_t>(w);
    msg.encoding = "mono8";
    msg.is_bigendian = false;
    msg.step = static_cast<uint32_t>(w);
    msg.data.resize(mono_bytes);
  };

  ensure_alloc(left_msg_mono);
  ensure_alloc(right_msg_mono);

  left_msg_mono.header.stamp = stamp;
  left_msg_mono.header.frame_id = left_frame_id_;
  right_msg_mono.header.stamp = stamp;
  right_msg_mono.header.frame_id = right_frame_id_;

  cv::Mat left_out(h, w, CV_8UC1, left_msg_mono.data.data(), left_msg_mono.step);
  cv::Mat right_out(h, w, CV_8UC1, right_msg_mono.data.data(), right_msg_mono.step);

  cv::cvtColor(left_bgr,  left_out,  cv::COLOR_BGR2GRAY);
  cv::cvtColor(right_bgr, right_out, cv::COLOR_BGR2GRAY);

  if (use_image_transport_) {
    pub_left_image_mono_transport_.publish(left_msg_mono);
    pub_right_image_mono_transport_.publish(right_msg_mono);
  } else {
    pub_left_image_mono_->publish(left_msg_mono);
    pub_right_image_mono_->publish(right_msg_mono);
  }
}

void UnitreeUdpCameraInterface::load_camera_infos_best_effort()
{
  left_info_loaded_  = load_camera_info_from_url(camera_info_left_name_, left_info_, camera_name_ + "_left");
  right_info_loaded_ = load_camera_info_from_url(camera_info_right_name_, right_info_, camera_name_ + "_right");

  if (!left_info_loaded_)  publish_log("WARN", "Left CameraInfo not loaded; publishing empty/default CameraInfo.");
  if (!right_info_loaded_) publish_log("WARN", "Right CameraInfo not loaded; publishing empty/default CameraInfo.");

  left_info_.header.frame_id  = left_frame_id_;
  right_info_.header.frame_id = right_frame_id_;
}

std::string UnitreeUdpCameraInterface::build_camera_info_url(
  const std::string& package_name,
  const std::string& calib_filename_or_url)
{
  if (has_scheme(calib_filename_or_url)) return calib_filename_or_url;

  const std::string share_dir = ament_index_cpp::get_package_share_directory(package_name);
  std::filesystem::path p = std::filesystem::path(share_dir) / "calibrations" / calib_filename_or_url;
  if (!p.has_extension()) p.replace_extension(".yaml");

  if (!std::filesystem::exists(p)) {
    throw std::runtime_error("Calibration file not found: " + p.string());
  }

  return std::string("file://") + p.string();
}

bool UnitreeUdpCameraInterface::load_camera_info_from_url(
  const std::string & url,
  sensor_msgs::msg::CameraInfo & out_info,
  const std::string & fallback_name)
{
  if (url.empty()) return false;

  std::string path;
  try {
    path = build_camera_info_url("unitree_ros2_interface", url);
  } catch (const std::exception & e) {
    publish_log("WARN", std::string("Calibration file not found, running without calibration: ") + e.what());
    return false;
  }
  if (path.rfind("file://", 0) == 0) path = path.substr(7);

  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    publish_log("ERROR", "Cannot open CameraInfo file: " + path);
    return false;
  }

  std::string camera_name = fallback_name;
  const bool ok = camera_calibration_parsers::readCalibrationYml(ifs, camera_name, out_info);
  if (!ok) publish_log("ERROR", "readCalibrationYml failed for: " + path);
  return ok;
}

std::string UnitreeUdpCameraInterface::normalize_ns(const std::string & ns)
{
  std::string out = ns;
  while (!out.empty() && out.front() == '/') out.erase(out.begin());
  while (!out.empty() && out.back() == '/') out.pop_back();
  return out;
}

std::string UnitreeUdpCameraInterface::make_topic(const std::string & suffix) const
{
  const std::string desired = normalize_ns(namespace_param_);
  const std::string node_ns = this->get_namespace();

  const bool node_has_ns = (node_ns != "/" && !node_ns.empty());
  const bool use_param_ns = (!desired.empty() && !node_has_ns);

  const std::string prefix = use_param_ns ? ("/" + desired) : std::string("");
  return prefix + "/" + camera_name_ + "/" + suffix;
}

void UnitreeUdpCameraInterface::publish_log(const std::string & level, const std::string & msg)
{
  if (level == "ERROR") RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
  else if (level == "WARN") RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
  else if (level == "DEBUG") RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str());
  else RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());

  std_msgs::msg::String m;
  m.data = "[" + level + "] " + msg;
  if (pub_log_ && rclcpp::ok()) {
    pub_log_->publish(m);
  }
}

std::string UnitreeUdpCameraInterface::upper(const std::string & s) {
  std::string out;
  out.reserve(s.size());
  for (char c : s) out.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
  return out;
}

}  // namespace unitree_ros2_interface

RCLCPP_COMPONENTS_REGISTER_NODE(unitree_ros2_interface::UnitreeUdpCameraInterface)
