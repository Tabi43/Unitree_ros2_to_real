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

  // Build rectification maps if rectified publishing is requested
  if (publish_rectified_color_ || publish_rectified_mono_) {
    rectify_enabled_ = build_rectification_maps();
    if (!rectify_enabled_) {
      publish_log("WARN", "Rectification requested but maps could not be built. Rectified topics disabled.");
    }
  }

  // publishers for images + camera_info
  rclcpp::QoS img_qos{rclcpp::SensorDataQoS()};
  img_qos.keep_last(1);
  rmw_qos_profile_t sensor_qos = img_qos.get_rmw_qos_profile();

  if(use_image_transport_) {
    publish_log("INFO", "Publishing images using image_transport.");
    pub_left_image_transport_  = image_transport::create_publisher(this, make_topic("left/color/image_raw"),  sensor_qos);
    pub_right_image_transport_ = image_transport::create_publisher(this, make_topic("right/color/image_raw"), sensor_qos);
  } else {
    publish_log("INFO", "Publishing images using rclcpp publishers.");
    pub_left_image_  = this->create_publisher<sensor_msgs::msg::Image>(make_topic("left/color/image_raw"),  img_qos);
    pub_right_image_ = this->create_publisher<sensor_msgs::msg::Image>(make_topic("right/color/image_raw"), img_qos);
  }

  if(publish_mono_) {
    if(use_image_transport_) {
      publish_log("INFO", "Publishing mono images using image_transport.");
      pub_left_image_mono_transport_  = image_transport::create_publisher(this, make_topic("left/mono/image_raw"),  sensor_qos);
      pub_right_image_mono_transport_ = image_transport::create_publisher(this, make_topic("right/mono/image_raw"), sensor_qos);
    } else {
      publish_log("INFO", "Publishing mono images using rclcpp publishers.");
      pub_left_image_mono_  = this->create_publisher<sensor_msgs::msg::Image>(make_topic("left/mono/image_raw"),  img_qos);
      pub_right_image_mono_ = this->create_publisher<sensor_msgs::msg::Image>(make_topic("right/mono/image_raw"), img_qos);
    }
  }

  pub_left_info_  = this->create_publisher<sensor_msgs::msg::CameraInfo>(make_topic("left/camera_info"),  rclcpp::SensorDataQoS());
  pub_right_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(make_topic("right/camera_info"), rclcpp::SensorDataQoS());

  // Rectified publishers
  if (rectify_enabled_ && publish_rectified_color_) {
    if (use_image_transport_rectified_) {
      publish_log("INFO", "Publishing rectified color images using image_transport.");
      pub_left_rect_color_transport_  = image_transport::create_publisher(this, make_topic("left/color/image_rect"),  sensor_qos);
      pub_right_rect_color_transport_ = image_transport::create_publisher(this, make_topic("right/color/image_rect"), sensor_qos);
    } else {
      publish_log("INFO", "Publishing rectified color images using rclcpp publishers.");
      pub_left_rect_color_  = this->create_publisher<sensor_msgs::msg::Image>(make_topic("left/color/image_rect"),  img_qos);
      pub_right_rect_color_ = this->create_publisher<sensor_msgs::msg::Image>(make_topic("right/color/image_rect"), img_qos);
    }
  }
  if (rectify_enabled_ && publish_rectified_mono_) {
    if (use_image_transport_rectified_) {
      publish_log("INFO", "Publishing rectified mono images using image_transport.");
      pub_left_rect_mono_transport_  = image_transport::create_publisher(this, make_topic("left/mono/image_rect"),  sensor_qos);
      pub_right_rect_mono_transport_ = image_transport::create_publisher(this, make_topic("right/mono/image_rect"), sensor_qos);
    } else {
      publish_log("INFO", "Publishing rectified mono images using rclcpp publishers.");
      pub_left_rect_mono_  = this->create_publisher<sensor_msgs::msg::Image>(make_topic("left/mono/image_rect"),  img_qos);
      pub_right_rect_mono_ = this->create_publisher<sensor_msgs::msg::Image>(make_topic("right/mono/image_rect"), img_qos);
    }
  }

  publish_log("INFO", "UnitreeCameraInterface started; publishing RAW stereo frames.");

  const std::string enc = upper(encoding_str_);
  if (enc == "BGR8") {
    encoding_ = sensor_msgs::image_encodings::BGR8;
    output_encoding_ = OutputEncoding::BGR8;
  } else if (enc == "RGB8") {
    encoding_ = sensor_msgs::image_encodings::RGB8;
    output_encoding_ = OutputEncoding::RGB8;
  } else if (enc == "MONO8") {
    encoding_ = sensor_msgs::image_encodings::MONO8;
    output_encoding_ = OutputEncoding::MONO8;
  } else {
    encoding_ = sensor_msgs::image_encodings::BGR8;
    output_encoding_ = OutputEncoding::BGR8;
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
  // When non-empty device_path takes precedence over device_index,
  // mirroring the behaviour of udp_camera_sender.
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
  // When true (default) OpenCV converts the captured frame to BGR before
  // delivering it to the application (CAP_PROP_CONVERT_RGB = 1).
  // Set to false only when you need the native pixel format (e.g. YUYV raw).
  this->declare_parameter<bool>("convert_rgb", true);
  this->declare_parameter<std::string>("encoding", "bgr8");

  this->declare_parameter<std::string>("left_frame_id", "");
  this->declare_parameter<std::string>("right_frame_id", "");

  this->declare_parameter<std::string>("camera_info_left_name", "");
  this->declare_parameter<std::string>("camera_info_right_name", "");

  this->declare_parameter<bool>("use_image_transport", false);
  this->declare_parameter<bool>("publish_mono", true);
  this->declare_parameter<bool>("publish_rectified_color", false);
  this->declare_parameter<bool>("publish_rectified_mono", false);
  this->declare_parameter<bool>("use_image_transport_rectified", false);

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
  // NOTE: device_path is read here and stored as-is. open_camera() will decide
  // which of the two (path vs index) to use: explicit path always wins.
  this->get_parameter("device_path", device_path_);

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
  this->get_parameter("convert_rgb", convert_rgb_);
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
  this->get_parameter("publish_rectified_color", publish_rectified_color_);
  this->get_parameter("publish_rectified_mono", publish_rectified_mono_);
  this->get_parameter("use_image_transport_rectified", use_image_transport_rectified_);

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

  // Prefer an explicit device_path if provided by the user; otherwise derive it
  // from device_index. We always open by path (string overload) so that
  // set_v4l2_control_best_effort() — which also opens by path — always refers to
  // the exact same device node. This matches the behaviour of udp_camera_sender.
  if (device_path_.empty()) {
    device_path_ = "/dev/video" + std::to_string(device_index_);
  }

  bool ok = cap_.open(device_path_, api);

  if (!ok || !cap_.isOpened()) {
    throw std::runtime_error("Failed to open camera (VideoCapture)");
  }

  // FOURCC must be set FIRST: some V4L2 drivers reset width/height/fps when
  // the pixel format changes. Setting it after dimensions may silently revert them.
  const int fourcc = fourcc_from_string(pixel_format_);
  if (fourcc != 0) {
    cap_.set(cv::CAP_PROP_FOURCC, static_cast<double>(fourcc));
    // Log the FOURCC actually negotiated by the driver.
    const int actual_fourcc = static_cast<int>(cap_.get(cv::CAP_PROP_FOURCC));
    char fc[5] = {};
    std::memcpy(fc, &actual_fourcc, 4);
    publish_log("INFO", "FOURCC requested=" + pixel_format_ + " actual=" + std::string(fc));
  }

  // Apply remaining settings after format is locked.
  cap_.set(cv::CAP_PROP_FRAME_WIDTH,  static_cast<double>(raw_width_));
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(raw_height_));
  cap_.set(cv::CAP_PROP_FPS, fps_);

  cap_.set(cv::CAP_PROP_BUFFERSIZE, static_cast<double>(buffer_size_));
  // CAP_PROP_CONVERT_RGB: when true OpenCV converts raw frames (e.g. YUYV) to
  // BGR automatically. This is the same knob exposed in udp_camera_sender via
  // cfg_.convert_rgb; keeping it configurable lets the caller receive native
  // pixel data when needed. Default is true (backward compatible).
  cap_.set(cv::CAP_PROP_CONVERT_RGB, convert_rgb_ ? 1.0 : 0.0);

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
  publish_log("INFO", "Starting capture and process threads...");
  capture_thread_ = std::thread(&UnitreeCameraInterface::capture_loop, this);
  process_thread_ = std::thread(&UnitreeCameraInterface::process_loop, this);
  if (rectify_enabled_) {
    publish_log("INFO", "Starting rectification thread...");
    rectify_thread_ = std::thread(&UnitreeCameraInterface::rectify_loop, this);
  }
}

void UnitreeCameraInterface::stop_capture_thread() {
  running_.store(false);
  frame_cv_.notify_all();  // sblocca process_loop se in attesa
  rect_cv_.notify_all();   // sblocca rectify_loop se in attesa
  publish_log("INFO", "Stopping capture and process threads...");
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  if (process_thread_.joinable()) {
    process_thread_.join();
  }
  if (rectify_thread_.joinable()) {
    rectify_thread_.join();
  }
}

void UnitreeCameraInterface::capture_loop() {
  // Thread A: drains V4L2 buffer at driver rate and stores the latest frame.
  // cap_.read() is a blocking call that delivers each frame at the configured
  // FPS interval. NO processing happens here: split/convert/publish is handled
  // entirely by process_loop() so the driver buffer is always drained promptly.
  static constexpr int kMaxConsecutiveReadFails = 50;
  int consecutive_read_fails = 0;

  while (rclcpp::ok() && running_.load()) {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      ++consecutive_read_fails;
      publish_log("WARN",
        "Failed to read frame from VideoCapture (" +
        std::to_string(consecutive_read_fails) + "/" +
        std::to_string(kMaxConsecutiveReadFails) + ")");
      if (consecutive_read_fails >= kMaxConsecutiveReadFails) {
        publish_log("ERROR", "Too many consecutive read failures. Stopping capture.");
        running_.store(false);
        frame_cv_.notify_all();  // sblocca process_loop
        break;
      }
      // Brief back-off to avoid burning CPU on a broken camera fd.
      std::this_thread::sleep_for(std::chrono::microseconds(100));
      continue;
    }
    consecutive_read_fails = 0;

    // Latest-only: sovrascrive sempre con il frame più recente.
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_frame_ = std::move(frame);
      frame_ready_ = true;
    }
    frame_cv_.notify_one();
  }
}

void UnitreeCameraInterface::process_loop() {
  // Thread B: waits for a new frame from capture_loop(), then does all
  // processing (split, colour conversion, encoding, publishing).
  // Because the slot is latest-only, if this thread is slower than the
  // camera it will drop older frames and always operate on the most recent one.
  while (rclcpp::ok() && running_.load()) {
    cv::Mat frame;
    {
      std::unique_lock<std::mutex> lock(frame_mutex_);
      frame_cv_.wait(lock, [this] { return frame_ready_ || !running_.load(); });
      if (!running_.load() && !frame_ready_) break;
      frame = std::move(latest_frame_);
      frame_ready_ = false;
    }

    if (frame.empty()) continue;

    if ((frame.cols % 2) != 0) {
      publish_log("ERROR", "Captured frame width is not even; cannot split stereo side-by-side");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    const int half_w = frame.cols / 2;
    const rclcpp::Time stamp = this->now();
    const bool need_mono = (publish_mono_ && output_encoding_ != OutputEncoding::MONO8);
    const int channels = (output_encoding_ == OutputEncoding::MONO8) ? 1 : 3;

    // Full-frame color conversion on the SBS image (one cvtColor, then ROI split)
    cv::Mat color_sbs;
    switch (output_encoding_) {
      case OutputEncoding::BGR8:
        color_sbs = frame;  // header-only ref, shared buffer
        break;
      case OutputEncoding::RGB8:
        cv::cvtColor(frame, color_sbs_buf_, cv::COLOR_BGR2RGB);
        color_sbs = color_sbs_buf_;
        break;
      case OutputEncoding::MONO8:
        cv::cvtColor(frame, color_sbs_buf_, cv::COLOR_BGR2GRAY);
        color_sbs = color_sbs_buf_;
        break;
    }

    // Full-frame mono conversion (only when publishing mono alongside color output)
    if (need_mono) {
      cv::cvtColor(frame, mono_sbs_buf_, cv::COLOR_BGR2GRAY);
    }

    // ROI split on converted frames (header-only, no pixel copy)
    cv::Mat left_color  = color_sbs(cv::Rect(0,      0, half_w, frame.rows));
    cv::Mat right_color = color_sbs(cv::Rect(half_w, 0, half_w, frame.rows));
    if (swap_lr_) std::swap(left_color, right_color);

    cv::Mat left_mono_roi, right_mono_roi;
    if (need_mono) {
      left_mono_roi  = mono_sbs_buf_(cv::Rect(0,      0, half_w, frame.rows));
      right_mono_roi = mono_sbs_buf_(cv::Rect(half_w, 0, half_w, frame.rows));
      if (swap_lr_) std::swap(left_mono_roi, right_mono_roi);
    }

    // Build Image message from a (possibly non-contiguous) cv::Mat ROI
    auto make_image = [&](const cv::Mat& roi, const std::string& enc,
                          const std::string& frame_id, int ch)
      -> std::unique_ptr<sensor_msgs::msg::Image>
    {
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->header.stamp = stamp;
      msg->header.frame_id = frame_id;
      msg->height = static_cast<uint32_t>(roi.rows);
      msg->width  = static_cast<uint32_t>(roi.cols);
      msg->encoding = enc;
      msg->is_bigendian = false;
      msg->step = static_cast<uint32_t>(roi.cols * ch);
      msg->data.resize(static_cast<size_t>(msg->step) * roi.rows);
      cv::Mat dst(roi.rows, roi.cols, roi.type(), msg->data.data(), msg->step);
      roi.copyTo(dst);
      return msg;
    };

    // Build all image messages (copies pixel data from ROIs into contiguous buffers)
    auto left_color_msg  = make_image(left_color,  encoding_, left_frame_id_,  channels);
    auto right_color_msg = make_image(right_color, encoding_, right_frame_id_, channels);

    std::unique_ptr<sensor_msgs::msg::Image> left_mono_msg, right_mono_msg;
    if (publish_mono_) {
      if (output_encoding_ == OutputEncoding::MONO8) {
        left_mono_msg  = make_image(left_color,  "mono8", left_frame_id_,  1);
        right_mono_msg = make_image(right_color, "mono8", right_frame_id_, 1);
      } else if (need_mono) {
        left_mono_msg  = make_image(left_mono_roi,  "mono8", left_frame_id_,  1);
        right_mono_msg = make_image(right_mono_roi, "mono8", right_frame_id_, 1);
      }
    }

    // Publish color images (unique_ptr for rclcpp zero-copy handoff)
    if (use_image_transport_) {
      pub_left_image_transport_.publish(*left_color_msg);
      pub_right_image_transport_.publish(*right_color_msg);
    } else {
      pub_left_image_->publish(std::move(left_color_msg));
      pub_right_image_->publish(std::move(right_color_msg));
    }

    // Publish mono images
    if (left_mono_msg && right_mono_msg) {
      if (use_image_transport_) {
        pub_left_image_mono_transport_.publish(*left_mono_msg);
        pub_right_image_mono_transport_.publish(*right_mono_msg);
      } else {
        pub_left_image_mono_->publish(std::move(left_mono_msg));
        pub_right_image_mono_->publish(std::move(right_mono_msg));
      }
    }

    // CameraInfo
    left_info_.header.stamp = stamp;
    right_info_.header.stamp = stamp;
    left_info_.header.frame_id = left_frame_id_;
    right_info_.header.frame_id = right_frame_id_;
    pub_left_info_->publish(left_info_);
    pub_right_info_->publish(right_info_);

    // Feed the rectification thread
    if (rectify_enabled_) {
      {
        std::lock_guard<std::mutex> lock(rect_mutex_);
        frame.copyTo(rect_input_frame_);
        rect_input_stamp_ = stamp;
        rect_input_ready_ = true;
      }
      rect_cv_.notify_one();
    }
  }
}

bool UnitreeCameraInterface::build_rectification_maps() {
  if (!left_info_loaded_ || !right_info_loaded_) {
    publish_log("WARN", "Cannot build rectification maps: CameraInfo not loaded for both cameras.");
    return false;
  }

  auto build_maps = [this](const sensor_msgs::msg::CameraInfo& info,
                           cv::Mat& map1, cv::Mat& map2,
                           const std::string& side) -> bool {
    const cv::Size size(static_cast<int>(info.width), static_cast<int>(info.height));
    if (size.width <= 0 || size.height <= 0) {
      publish_log("ERROR", side + " CameraInfo has invalid dimensions.");
      return false;
    }

    cv::Mat K(3, 3, CV_64F);
    std::memcpy(K.data, info.k.data(), 9 * sizeof(double));

    cv::Mat D(static_cast<int>(info.d.size()), 1, CV_64F);
    if (!info.d.empty()) {
      std::memcpy(D.data, info.d.data(), info.d.size() * sizeof(double));
    }

    cv::Mat R(3, 3, CV_64F);
    std::memcpy(R.data, info.r.data(), 9 * sizeof(double));

    cv::Mat P(3, 4, CV_64F);
    std::memcpy(P.data, info.p.data(), 12 * sizeof(double));
    cv::Mat new_K = P(cv::Rect(0, 0, 3, 3)).clone();

    if (info.distortion_model == "equidistant") {
      cv::Mat D4(4, 1, CV_64F, cv::Scalar(0));
      for (int i = 0; i < std::min(static_cast<int>(info.d.size()), 4); ++i) {
        D4.at<double>(i) = info.d[static_cast<size_t>(i)];
      }
      cv::fisheye::initUndistortRectifyMap(K, D4, R, new_K, size, CV_16SC2, map1, map2);
    } else {
      cv::initUndistortRectifyMap(K, D, R, new_K, size, CV_16SC2, map1, map2);
    }

    publish_log("INFO", side + " rectification maps built (" + info.distortion_model +
                ", " + std::to_string(size.width) + "x" + std::to_string(size.height) + ").");
    return true;
  };

  bool ok_l = build_maps(left_info_, rect_map1_left_, rect_map2_left_, "Left");
  bool ok_r = build_maps(right_info_, rect_map1_right_, rect_map2_right_, "Right");
  return ok_l && ok_r;
}

void UnitreeCameraInterface::rectify_loop() {
  while (rclcpp::ok() && running_.load()) {
    cv::Mat frame;
    rclcpp::Time stamp;
    {
      std::unique_lock<std::mutex> lock(rect_mutex_);
      rect_cv_.wait(lock, [this] { return rect_input_ready_ || !running_.load(); });
      if (!running_.load() && !rect_input_ready_) break;
      frame = std::move(rect_input_frame_);
      stamp = rect_input_stamp_;
      rect_input_ready_ = false;
    }

    if (frame.empty() || (frame.cols % 2) != 0) continue;

    const int half_w = frame.cols / 2;

    // Split SBS BGR into L/R
    cv::Mat left_bgr  = frame(cv::Rect(0,      0, half_w, frame.rows));
    cv::Mat right_bgr = frame(cv::Rect(half_w, 0, half_w, frame.rows));
    if (swap_lr_) std::swap(left_bgr, right_bgr);

    // Rectify (remap)
    cv::remap(left_bgr,  rect_left_buf_,  rect_map1_left_,  rect_map2_left_,  cv::INTER_LINEAR);
    cv::remap(right_bgr, rect_right_buf_, rect_map1_right_, rect_map2_right_, cv::INTER_LINEAR);

    auto make_image = [&](const cv::Mat& roi, const std::string& enc,
                          const std::string& frame_id, int ch)
      -> std::unique_ptr<sensor_msgs::msg::Image>
    {
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->header.stamp = stamp;
      msg->header.frame_id = frame_id;
      msg->height = static_cast<uint32_t>(roi.rows);
      msg->width  = static_cast<uint32_t>(roi.cols);
      msg->encoding = enc;
      msg->is_bigendian = false;
      msg->step = static_cast<uint32_t>(roi.cols * ch);
      msg->data.resize(static_cast<size_t>(msg->step) * roi.rows);
      cv::Mat dst(roi.rows, roi.cols, roi.type(), msg->data.data(), msg->step);
      roi.copyTo(dst);
      return msg;
    };

    // Publish rectified color
    if (publish_rectified_color_) {
      cv::Mat left_out, right_out;
      std::string enc = sensor_msgs::image_encodings::BGR8;
      int ch = 3;
      switch (output_encoding_) {
        case OutputEncoding::BGR8:
          left_out = rect_left_buf_;
          right_out = rect_right_buf_;
          enc = sensor_msgs::image_encodings::BGR8;
          ch = 3;
          break;
        case OutputEncoding::RGB8:
          cv::cvtColor(rect_left_buf_,  rect_left_color_buf_,  cv::COLOR_BGR2RGB);
          cv::cvtColor(rect_right_buf_, rect_right_color_buf_, cv::COLOR_BGR2RGB);
          left_out = rect_left_color_buf_;
          right_out = rect_right_color_buf_;
          enc = sensor_msgs::image_encodings::RGB8;
          ch = 3;
          break;
        case OutputEncoding::MONO8:
          cv::cvtColor(rect_left_buf_,  rect_left_color_buf_,  cv::COLOR_BGR2GRAY);
          cv::cvtColor(rect_right_buf_, rect_right_color_buf_, cv::COLOR_BGR2GRAY);
          left_out = rect_left_color_buf_;
          right_out = rect_right_color_buf_;
          enc = sensor_msgs::image_encodings::MONO8;
          ch = 1;
          break;
      }
      auto left_msg  = make_image(left_out,  enc, left_frame_id_,  ch);
      auto right_msg = make_image(right_out, enc, right_frame_id_, ch);

      if (use_image_transport_rectified_) {
        pub_left_rect_color_transport_.publish(*left_msg);
        pub_right_rect_color_transport_.publish(*right_msg);
      } else {
        pub_left_rect_color_->publish(std::move(left_msg));
        pub_right_rect_color_->publish(std::move(right_msg));
      }
    }

    // Publish rectified mono
    if (publish_rectified_mono_) {
      cv::cvtColor(rect_left_buf_,  rect_left_mono_buf_,  cv::COLOR_BGR2GRAY);
      cv::cvtColor(rect_right_buf_, rect_right_mono_buf_, cv::COLOR_BGR2GRAY);

      auto left_msg  = make_image(rect_left_mono_buf_,  sensor_msgs::image_encodings::MONO8, left_frame_id_,  1);
      auto right_msg = make_image(rect_right_mono_buf_, sensor_msgs::image_encodings::MONO8, right_frame_id_, 1);

      if (use_image_transport_rectified_) {
        pub_left_rect_mono_transport_.publish(*left_msg);
        pub_right_rect_mono_transport_.publish(*right_msg);
      } else {
        pub_left_rect_mono_->publish(std::move(left_msg));
        pub_right_rect_mono_->publish(std::move(right_msg));
      }
    }
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