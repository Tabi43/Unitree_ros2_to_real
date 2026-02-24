#include "unitree_ros2_interface/camera-interface.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstring>
#include <fstream>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

// V4L2 controls
#include <linux/videodev2.h>

// camera_calibration_parsers (standard ROS camera calibration YAML)
#include <camera_calibration_parsers/parse_yml.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

// OpenCL toggle
#include <opencv2/core/ocl.hpp>

using namespace std::chrono_literals;

namespace unitree_ros2_interface {

static std::optional<int> get_optional_int_param(rclcpp::Node * node, const std::string & name) {
  int v = -1;
  if (node->get_parameter(name, v) && v >= 0) {
    return v;
  }
  return std::nullopt;
}

UnitreeCameraInterface::UnitreeCameraInterface(const rclcpp::NodeOptions & options)
: rclcpp::Node("unitree_camera_interface", options) {
  declare_and_get_params();
  
  // publishers early (log)
  pub_log_ = this->create_publisher<std_msgs::msg::String>(make_topic("log"), rclcpp::QoS(10).reliable());

  validate_params_or_throw();

  configure_opencv_env();

  open_camera();
  apply_v4l2_controls_best_effort();
  load_camera_infos_best_effort();

  // publishers for images + camera_info
  // image_transport API (Humble) supports create_publisher(rclcpp::Node*, topic, qos). 
  rmw_qos_profile_t sensor_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();

  if(use_image_transport_) {
    publish_log("INFO", "Publishing images using image_transport.");
    pub_left_image_transport_  = image_transport::create_publisher(this, make_topic("left/image_raw"),  sensor_qos);
    pub_right_image_transport_ = image_transport::create_publisher(this, make_topic("right/image_raw"), sensor_qos);
  } else {
    publish_log("INFO", "Publishing images using rclcpp publishers.");
    pub_left_image_  = this->create_publisher<sensor_msgs::msg::Image>(make_topic("left/image_raw"),  rclcpp::SensorDataQoS());
    pub_right_image_ = this->create_publisher<sensor_msgs::msg::Image>(make_topic("right/image_raw"), rclcpp::SensorDataQoS());
  }

  if(publish_mono_) {
    if(use_image_transport_) {
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

  publish_log("INFO", "UnitreeCameraInterface started; publishing RAW stereo frames.");

  if(encoding_str_ == "bgr8") {
    encoding_ = sensor_msgs::image_encodings::BGR8;
    publish_log("WARN", "Publishing raw images with encoding 'bgr8'.");
  } else if(encoding_str_ == "rgb8") {
    encoding_ = sensor_msgs::image_encodings::RGB8;
    publish_log("WARN", "Publishing raw images with encoding 'rgb8'.");
  } else if(encoding_str_ == "mono8") {
    encoding_ = sensor_msgs::image_encodings::MONO8;
    publish_log("WARN", "Publishing raw images with encoding 'mono8'.");
  } else {
    encoding_ = sensor_msgs::image_encodings::BGR8;
    publish_log("WARN", "Unsupported encoding '" + encoding_str_ + "'; defaulting to 'bgr8'.");
  }

  start_capture_thread();
}

UnitreeCameraInterface::~UnitreeCameraInterface() {
  stop_capture_thread();
  if (cap_.isOpened()) {
    cap_.release();
  }
  publish_log("INFO", "UnitreeCameraInterface stopped.");
}

void UnitreeCameraInterface::declare_and_get_params() {
  // Base
  this->declare_parameter<std::string>("namespace", "");
  this->declare_parameter<std::string>("camera_name", "bottom_camera");
  this->declare_parameter<int>("device_index", 0);
  this->declare_parameter<std::string>("device_path", "");

  this->declare_parameter<int>("raw_width", 940);
  this->declare_parameter<int>("raw_height", 400);
  this->declare_parameter<double>("raw_fps", 30.0);

  this->declare_parameter<std::string>("stereo_layout", "side_by_side");
  this->declare_parameter<bool>("swap_lr", false);

  this->declare_parameter<std::string>("pixel_format", "MJPG");
  this->declare_parameter<bool>("force_v4l2", true);
  this->declare_parameter<int>("buffer_size", 4);
  this->declare_parameter<int>("warmup_frames", 0);
  this->declare_parameter<std::string>("encoding", "bgr8");

  this->declare_parameter<std::string>("left_frame_id", "");
  this->declare_parameter<std::string>("right_frame_id", "");

  this->declare_parameter<std::string>("camera_info_left_name", "");
  this->declare_parameter<std::string>("camera_info_right_name", "");

  this->declare_parameter<bool>("use_image_transport", false);
  this->declare_parameter<bool>("publish_mono", true);

  // OpenCV knobs
  this->declare_parameter<std::string>("opencv.priority_list", "V4L2");
  this->declare_parameter<bool>("opencv.videoio_debug", false);
  this->declare_parameter<std::string>("opencv.log_level", "INFO");
  this->declare_parameter<bool>("opencv.disable_opencl", true);

  // V4L2 controls (use -1 => ignore)
  this->declare_parameter<int>("v4l2.exposure_auto", -1);
  this->declare_parameter<int>("v4l2.exposure_absolute", -1);
  this->declare_parameter<int>("v4l2.gain", -1);
  this->declare_parameter<int>("v4l2.brightness", -1);
  this->declare_parameter<int>("v4l2.contrast", -1);
  this->declare_parameter<int>("v4l2.saturation", -1);
  this->declare_parameter<int>("v4l2.sharpness", -1);
  this->declare_parameter<int>("v4l2.gamma", -1);
  this->declare_parameter<int>("v4l2.white_balance_temperature_auto", -1);
  this->declare_parameter<int>("v4l2.white_balance_temperature", -1);
  this->declare_parameter<int>("v4l2.power_line_frequency", -1);
  this->declare_parameter<int>("v4l2.backlight_compensation", -1);

  // Get
  this->get_parameter("namespace", namespace_param_);
  this->get_parameter("camera_name", camera_name_);
  this->get_parameter("device_index", device_index_);

  this->get_parameter("raw_width", raw_width_);
  this->get_parameter("raw_height", raw_height_);
  this->get_parameter("raw_fps", fps_);

  // Debug: print the read parameters
  RCLCPP_INFO(this->get_logger(), "Parameters read: raw_width=%d, raw_height=%d, raw_fps=%f", 
              raw_width_, raw_height_, fps_);

  this->get_parameter("stereo_layout", stereo_layout_);
  this->get_parameter("swap_lr", swap_lr_);

  this->get_parameter("pixel_format", pixel_format_);
  this->get_parameter("force_v4l2", force_v4l2_);
  this->get_parameter("buffer_size", buffer_size_);
  this->get_parameter("warmup_frames", warmup_frames_);
  this->get_parameter("encoding", encoding_str_);

  this->get_parameter("left_frame_id", left_frame_id_);
  this->get_parameter("right_frame_id", right_frame_id_);

  this->get_parameter("camera_info_left_name", camera_info_left_name_);
  this->get_parameter("camera_info_right_name", camera_info_right_name_);

  // Debug: print camera info parameters
  RCLCPP_INFO(this->get_logger(), "Camera info parameters: left='%s', right='%s'", 
              camera_info_left_name_.c_str(), camera_info_right_name_.c_str());

  this->get_parameter("use_image_transport", use_image_transport_);
  this->get_parameter("publish_mono", publish_mono_);

  this->get_parameter("opencv.priority_list", opencv_priority_list_);
  this->get_parameter("opencv.videoio_debug", opencv_videoio_debug_);
  this->get_parameter("opencv.log_level", opencv_log_level_);
  this->get_parameter("opencv.disable_opencl", opencv_disable_opencl_);

  v4l2_exposure_auto_    = get_optional_int_param(this, "v4l2.exposure_auto");
  v4l2_exposure_absolute_= get_optional_int_param(this, "v4l2.exposure_absolute");
  v4l2_gain_             = get_optional_int_param(this, "v4l2.gain");
  v4l2_brightness_       = get_optional_int_param(this, "v4l2.brightness");
  v4l2_contrast_         = get_optional_int_param(this, "v4l2.contrast");
  v4l2_saturation_       = get_optional_int_param(this, "v4l2.saturation");
  v4l2_sharpness_        = get_optional_int_param(this, "v4l2.sharpness");
  v4l2_gamma_            = get_optional_int_param(this, "v4l2.gamma");
  v4l2_wb_auto_          = get_optional_int_param(this, "v4l2.white_balance_temperature_auto");
  v4l2_wb_temp_          = get_optional_int_param(this, "v4l2.white_balance_temperature");
  v4l2_power_line_freq_  = get_optional_int_param(this, "v4l2.power_line_frequency");
  v4l2_backlight_comp_   = get_optional_int_param(this, "v4l2.backlight_compensation");

  if (left_frame_id_.empty())  left_frame_id_  = camera_name_ + "_left_optical";
  if (right_frame_id_.empty()) right_frame_id_ = camera_name_ + "_right_optical";
}

void UnitreeCameraInterface::validate_params_or_throw() {
  if (camera_name_.empty()) {
    throw std::runtime_error("camera_name is empty");
  }

  stereo_layout_ = upper(stereo_layout_);
  if (stereo_layout_ != "SIDE_BY_SIDE") {
    throw std::runtime_error("Only stereo_layout=side_by_side is supported in this first version");
  }

  if (raw_width_ <= 0 || raw_height_ <= 0) {
    throw std::runtime_error("raw_width/raw_height must be > 0");
  }
  if ((raw_width_ % 2) != 0) {
    throw std::runtime_error("raw_width must be even for side-by-side stereo");
  }
  if (fps_ <= 0.0) {
    throw std::runtime_error("fps must be > 0");
  }

  pixel_format_ = upper(pixel_format_);
  if (pixel_format_ != "MJPG" && pixel_format_ != "YUYV") {
    throw std::runtime_error("pixel_format must be MJPG or YUYV");
  }

  if (buffer_size_ <= 0) buffer_size_ = 1;
  if (warmup_frames_ < 0) warmup_frames_ = 0;

  // Namespace coherence note (avoid double namespace)
  const std::string node_ns = this->get_namespace();  // e.g. "/"
  const std::string desired = normalize_ns(namespace_param_);
  if (!desired.empty() && node_ns != "/" && node_ns != ("/" + desired)) {
    publish_log(
      "WARN",
      "Node already running in namespace '" + node_ns +
      "' but parameter namespace='" + desired +
      "'. Topics will follow the node namespace (to avoid double prefix).");
  }
}

void UnitreeCameraInterface::configure_opencv_env() {
  // Prefer V4L2 via OpenCV priority list (what you already debugged). :contentReference[oaicite:4]{index=4}
  if (!opencv_priority_list_.empty()) {
    setenv("OPENCV_VIDEOIO_PRIORITY_LIST", opencv_priority_list_.c_str(), 1);
  }
  if (opencv_videoio_debug_) {
    setenv("OPENCV_VIDEOIO_DEBUG", "1", 1);
  }
  if (!opencv_log_level_.empty()) {
    setenv("OPENCV_LOG_LEVEL", opencv_log_level_.c_str(), 1);
  }
  if (opencv_disable_opencl_) {
    cv::ocl::setUseOpenCL(false);
  }
}

void UnitreeCameraInterface::open_camera() {
  const int api = force_v4l2_ ? cv::CAP_V4L2 : cv::CAP_ANY;

  device_path_  = "/dev/video" + std::to_string(device_index_);

  bool ok = false;
  if (!device_path_.empty()) {
    ok = cap_.open(device_path_, api);
  } else {
    ok = cap_.open(device_index_, api);
  }

  if (!ok || !cap_.isOpened()) {
    throw std::runtime_error("Failed to open camera (VideoCapture)");
  }

  // Apply requested settings (best effort)
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(raw_width_));
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(raw_height_));
  cap_.set(cv::CAP_PROP_FPS, fps_);

  const int fourcc = fourcc_from_string(pixel_format_);
  if (fourcc != 0) {
    cap_.set(cv::CAP_PROP_FOURCC, static_cast<double>(fourcc));
  }

  cap_.set(cv::CAP_PROP_BUFFERSIZE, static_cast<double>(buffer_size_));
  cap_.set(cv::CAP_PROP_CONVERT_RGB, 1.0);

  // Warmup
  cv::Mat tmp;
  for (int i = 0; i < warmup_frames_; ++i) {
    (void)cap_.read(tmp);
  }

  // Report actual
  const int aw = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
  const int ah = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
  const double afps = cap_.get(cv::CAP_PROP_FPS);

  publish_log(
    "INFO",
    "Camera opened. Requested " + std::to_string(raw_width_) + "x" + std::to_string(raw_height_) +
    "@" + std::to_string(fps_) + " fmt=" + pixel_format_ +
    " | Actual " + std::to_string(aw) + "x" + std::to_string(ah) +
    "@" + std::to_string(afps)
  );
}

bool UnitreeCameraInterface::set_v4l2_control_best_effort(int control_id, int value) {
  const std::string dev = (!device_path_.empty()) ? device_path_ : ("/dev/video" + std::to_string(device_index_));

  int fd = ::open(dev.c_str(), O_RDWR);
  if (fd < 0) {
    publish_log("WARN", "Cannot open " + dev + " for V4L2 controls: " + std::string(std::strerror(errno)));
    return false;
  }

  v4l2_control ctrl;
  std::memset(&ctrl, 0, sizeof(ctrl));
  ctrl.id = static_cast<__u32>(control_id);
  ctrl.value = value;

  int rc = ioctl(fd, VIDIOC_S_CTRL, &ctrl);
  ::close(fd);

  if (rc < 0) {
    publish_log("WARN", "VIDIOC_S_CTRL failed (id=" + std::to_string(control_id) + " val=" + std::to_string(value) +
                        "): " + std::string(std::strerror(errno)));
    return false;
  }

  return true;
}

void UnitreeCameraInterface::apply_v4l2_controls_best_effort() {
  // Only apply if requested
  if (v4l2_exposure_auto_)     (void)set_v4l2_control_best_effort(V4L2_CID_EXPOSURE_AUTO, *v4l2_exposure_auto_);
  if (v4l2_exposure_absolute_) (void)set_v4l2_control_best_effort(V4L2_CID_EXPOSURE_ABSOLUTE, *v4l2_exposure_absolute_);
  if (v4l2_gain_)              (void)set_v4l2_control_best_effort(V4L2_CID_GAIN, *v4l2_gain_);
  if (v4l2_brightness_)        (void)set_v4l2_control_best_effort(V4L2_CID_BRIGHTNESS, *v4l2_brightness_);
  if (v4l2_contrast_)          (void)set_v4l2_control_best_effort(V4L2_CID_CONTRAST, *v4l2_contrast_);
  if (v4l2_saturation_)        (void)set_v4l2_control_best_effort(V4L2_CID_SATURATION, *v4l2_saturation_);
  if (v4l2_sharpness_)         (void)set_v4l2_control_best_effort(V4L2_CID_SHARPNESS, *v4l2_sharpness_);
  if (v4l2_gamma_)             (void)set_v4l2_control_best_effort(V4L2_CID_GAMMA, *v4l2_gamma_);
  if (v4l2_wb_auto_)           (void)set_v4l2_control_best_effort(V4L2_CID_AUTO_WHITE_BALANCE, *v4l2_wb_auto_);
  if (v4l2_wb_temp_)           (void)set_v4l2_control_best_effort(V4L2_CID_WHITE_BALANCE_TEMPERATURE, *v4l2_wb_temp_);
  if (v4l2_power_line_freq_)   (void)set_v4l2_control_best_effort(V4L2_CID_POWER_LINE_FREQUENCY, *v4l2_power_line_freq_);
  if (v4l2_backlight_comp_)    (void)set_v4l2_control_best_effort(V4L2_CID_BACKLIGHT_COMPENSATION, *v4l2_backlight_comp_);
}

void UnitreeCameraInterface::load_camera_infos_best_effort() {
  // camera_calibration_parsers provides readCalibrationYml for standard calibration YAML.
  left_info_loaded_  = load_camera_info_from_url(camera_info_left_name_, left_info_, camera_name_ + "_left");
  right_info_loaded_ = load_camera_info_from_url(camera_info_right_name_, right_info_, camera_name_ + "_right");

  if (!left_info_loaded_) {
    publish_log("WARN", "Left CameraInfo not loaded (camera_info_left_name). Will publish empty/default CameraInfo.");
  }
  if (!right_info_loaded_) {
    publish_log("WARN", "Right CameraInfo not loaded (camera_info_right_name). Will publish empty/default CameraInfo.");
  }

  left_info_.header.frame_id  = left_frame_id_;
  right_info_.header.frame_id = right_frame_id_;
}

std::string UnitreeCameraInterface::build_camera_info_url(const std::string& package_name, const std::string& calib_filename_or_url) {

  // Se l'utente passa già un URL completo (file:// o package://), non toccarlo.
  if (has_scheme(calib_filename_or_url)) {
    return calib_filename_or_url;
  }

  // Se passa solo un nome file, lo risolvo in <share>/<pkg>/calibrations/<file>
  const std::string share_dir = ament_index_cpp::get_package_share_directory(package_name);
  std::filesystem::path p = std::filesystem::path(share_dir) / "calibrations" / calib_filename_or_url;

  // (Opzionale) se non hanno messo estensione, aggiungi .yaml
  if (!p.has_extension()) {
    p.replace_extension(".yaml");
  }

  if (!std::filesystem::exists(p)) {
    // Qui nel tuo nodo fai RCLCPP_ERROR e/o lanci eccezione
    throw std::runtime_error("Calibration file not found: " + p.string());
  }

  // concatenando "file://" + "/abs/path" ottieni "file:///abs/path" correttamente.
  return std::string("file://") + p.string();
}

bool UnitreeCameraInterface::load_camera_info_from_url(
  const std::string & url,
  sensor_msgs::msg::CameraInfo & out_info,
  const std::string & fallback_name) {
  
  if (url.empty()) {
    publish_log("ERROR", "CameraInfo URL is empty");
    return false;
  }

  std::string path;
  try {
    path = build_camera_info_url("unitree_ros2_interface", url);
  } catch (const std::exception & e) {
    publish_log("WARN", std::string("Calibration file not found, running without calibration: ") + e.what());
    return false;
  }

  // Rimuovi il prefisso file:// se presente
  if (path.rfind("file://", 0) == 0) {
    path = path.substr(7); // rimuovi "file://"
  }

  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    publish_log("ERROR", "Cannot open CameraInfo file: " + path);
    return false;
  }

  std::string camera_name = fallback_name;
  bool ok = camera_calibration_parsers::readCalibrationYml(ifs, camera_name, out_info);
  if (!ok) {
    publish_log("ERROR", "readCalibrationYml failed for: " + path);
  }
  return ok;
}

void UnitreeCameraInterface::start_capture_thread() {
  running_.store(true);
  publish_log("INFO", "Starting capture thread...");
  capture_thread_ = std::thread(&UnitreeCameraInterface::capture_loop, this);
}

void UnitreeCameraInterface::stop_capture_thread() {
  running_.store(false);
  publish_log("INFO", "Stopping capture thread...");
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
}

void UnitreeCameraInterface::capture_loop() {
  rclcpp::WallRate rate(std::max(1.0, fps_));

  while (rclcpp::ok() && running_.load()) {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      publish_log("WARN", "Failed to read frame from VideoCapture");
      usleep(1000);
      continue;
    }

    if ((frame.cols % 2) != 0) {
      publish_log("ERROR", "Captured frame width is not even; cannot split stereo side-by-side");
      rate.sleep();
      continue;
    }

    const int half_w = frame.cols / 2;
    cv::Mat left  = frame(cv::Rect(0,      0, half_w, frame.rows));
    cv::Mat right = frame(cv::Rect(half_w, 0, half_w, frame.rows));

    if (swap_lr_) {
      std::swap(left, right);
    }

    const rclcpp::Time stamp = this->now();

    std_msgs::msg::Header hl;
    hl.stamp = stamp;
    hl.frame_id = left_frame_id_;

    std_msgs::msg::Header hr;
    hr.stamp = stamp;
    hr.frame_id = right_frame_id_;

    // Encode as requested
    cv_bridge::CvImage left_msg(hl, encoding_, left);
    cv_bridge::CvImage right_msg(hr, encoding_, right);

    if(use_image_transport_) {
      pub_left_image_transport_.publish(left_msg.toImageMsg());
      pub_right_image_transport_.publish(right_msg.toImageMsg());
    } else {
      pub_left_image_->publish(*left_msg.toImageMsg());
      pub_right_image_->publish(*right_msg.toImageMsg());
    }

    // Optional mono images
    if (publish_mono_) publishStereoMonoFromBGR(left, right, stamp);

    // CameraInfo (publish every frame so image_proc sees consistent stamps)
    sensor_msgs::msg::CameraInfo li = left_info_;
    sensor_msgs::msg::CameraInfo ri = right_info_;
    li.header.stamp = stamp;
    ri.header.stamp = stamp;
    li.header.frame_id = left_frame_id_;
    ri.header.frame_id = right_frame_id_;

    pub_left_info_->publish(li);
    pub_right_info_->publish(ri);

    rate.sleep();
  }
}

void UnitreeCameraInterface::publishStereoMonoFromBGR(const cv::Mat& left_bgr, const cv::Mat& right_bgr, const rclcpp::Time& stamp) {
  sensor_msgs::msg::Image left_msg_mono, right_msg_mono;
  
  if (left_bgr.empty() || right_bgr.empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty left/right image.");
    return;
  }
  if (left_bgr.size() != right_bgr.size()) {
    RCLCPP_WARN(this->get_logger(), "Left/right sizes differ: left=%dx%d right=%dx%d", left_bgr.cols, left_bgr.rows, right_bgr.cols, right_bgr.rows);
    return;
  }
  if (left_bgr.type() != CV_8UC3 || right_bgr.type() != CV_8UC3) {
    RCLCPP_WARN(this->get_logger(), "Expected CV_8UC3 BGR images. Got left type=%d right type=%d", left_bgr.type(), right_bgr.type());
    return;
  }

  const int w = left_bgr.cols;
  const int h = left_bgr.rows;
  const size_t mono_bytes = static_cast<size_t>(w) * static_cast<size_t>(h);

  auto ensure_alloc = [&](sensor_msgs::msg::Image& msg) {
    if (msg.width != static_cast<uint32_t>(w) || msg.height != static_cast<uint32_t>(h) ||
        msg.encoding != "mono8" || msg.step != static_cast<uint32_t>(w) ||
        msg.data.size() != mono_bytes)
    {
      msg.height = static_cast<uint32_t>(h);
      msg.width  = static_cast<uint32_t>(w);
      msg.encoding = "mono8";
      msg.is_bigendian = false;
      msg.step = static_cast<uint32_t>(w);
      msg.data.resize(mono_bytes);
    }
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

  if(use_image_transport_) {
    pub_left_image_mono_transport_.publish(left_msg_mono);
    pub_right_image_mono_transport_.publish(right_msg_mono);
  } else {
    pub_left_image_mono_->publish(left_msg_mono);
    pub_right_image_mono_->publish(right_msg_mono);
  }
}

std::string UnitreeCameraInterface::normalize_ns(const std::string & ns) {
  std::string out = ns;
  while (!out.empty() && out.front() == '/') out.erase(out.begin());
  while (!out.empty() && out.back() == '/') out.pop_back();
  return out;
}

std::string UnitreeCameraInterface::make_topic(const std::string & suffix) const {
  // Desired convention: namespace/camera_name/(left|right)/image_raw
  const std::string desired = normalize_ns(namespace_param_);
  const std::string node_ns = this->get_namespace();  // "/" or "/unitree_go1"

  // If node already has a namespace, do NOT double-prefix.
  const bool node_has_ns = (node_ns != "/" && !node_ns.empty());
  const bool use_param_ns = (!desired.empty() && !node_has_ns);

  const std::string prefix = use_param_ns ? ("/" + desired + "/") : std::string("");
  return prefix + camera_name_ + "/" + suffix;
}

void UnitreeCameraInterface::publish_log(const std::string & level, const std::string & msg) {
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

int UnitreeCameraInterface::fourcc_from_string(const std::string & fmt) {
  const std::string u = upper(fmt);
  if (u == "MJPG" || u == "MJPEG") {
    return cv::VideoWriter::fourcc('M','J','P','G');
  }
  if (u == "YUYV" || u == "YUY2") {
    return cv::VideoWriter::fourcc('Y','U','Y','V');
  }
  return 0;
}

std::string UnitreeCameraInterface::upper(const std::string & s) {
  std::string out;
  out.reserve(s.size());
  for (char c : s) out.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
  return out;
}

}  // namespace unitree_ros2_interface

RCLCPP_COMPONENTS_REGISTER_NODE(unitree_ros2_interface::UnitreeCameraInterface)