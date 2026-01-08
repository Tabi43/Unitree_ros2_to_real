#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <cstring>
#include <string>
#include <vector>
#include <cstdlib>  // setenv
#include <opencv2/core/ocl.hpp>

using namespace std::chrono_literals;

class SimpleV4L2CameraNode : public rclcpp::Node
{
public:
  SimpleV4L2CameraNode()
  : Node("simple_v4l2_camera_node")
  {
    // --- Parametri minimi ---
    device_index_ = declare_parameter<int>("device_index", 0);
    width_        = declare_parameter<int>("width", 928);
    height_       = declare_parameter<int>("height", 400);
    fps_          = declare_parameter<int>("fps", 30);
    frame_id_     = declare_parameter<std::string>("frame_id", "camera");
    topic_        = declare_parameter<std::string>("topic", "image_raw");
    fourcc_       = declare_parameter<std::string>("fourcc", "MJPG");
    force_v4l2_   = declare_parameter<bool>("force_v4l2", true);
    disable_gst_  = declare_parameter<bool>("disable_gstreamer", true);
    disable_ocl_  = declare_parameter<bool>("disable_opencl", true);
    opencv_debug_ = declare_parameter<bool>("opencv_videoio_debug", true);

    // --- Configurazione runtime OpenCV (prima di aprire la camera) ---
    // OpenCV documenta OPENCV_VIDEOIO_DEBUG, OPENCV_VIDEOIO_PRIORITY_LIST e OPENCV_VIDEOIO_PRIORITY_<backend>. :contentReference[oaicite:1]{index=1}
    if (opencv_debug_) {
      setenv("OPENCV_VIDEOIO_DEBUG", "1", 0);
    }
    if (disable_gst_) {
      setenv("OPENCV_VIDEOIO_PRIORITY_GSTREAMER", "0", 0); // disabilita backend GStreamer :contentReference[oaicite:2]{index=2}
    }
    if (force_v4l2_) {
      setenv("OPENCV_VIDEOIO_PRIORITY_LIST", "V4L2,GSTREAMER,FFMPEG", 0); // priorità esplicita :contentReference[oaicite:3]{index=3}
    }
    if (disable_ocl_) {
      cv::ocl::setUseOpenCL(false);
    }

    // --- Publisher ---
    pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_, rclcpp::SensorDataQoS());

    // --- Apertura camera ---
    open_camera();

    // --- Timer di acquisizione ---
    auto period = std::chrono::duration<double>(1.0 / std::max(1, fps_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SimpleV4L2CameraNode::tick, this)
    );

    RCLCPP_INFO(get_logger(), "Publishing %s from /dev/video%d (%dx%d @ %d fps), fourcc=%s",
                topic_.c_str(), device_index_, width_, height_, fps_, fourcc_.c_str());
  }

  ~SimpleV4L2CameraNode() override
  {
    if (cap_.isOpened()) {
      cap_.release();
    }
  }

private:
  void open_camera()
  {
    // Forza V4L2 direttamente nell'API: riduce ambiguità sui backend.
    // (Se CAP_V4L2 non è disponibile nella build, OpenCV farà fallback.)
    if (!cap_.open(device_index_, force_v4l2_ ? cv::CAP_V4L2 : cv::CAP_ANY)) {
      throw std::runtime_error("VideoCapture open() failed");
    }

    // Best-effort settings (driver può ignorare).
    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS,          fps_);

    if (fourcc_.size() == 4) {
      int code = cv::VideoWriter::fourcc(fourcc_[0], fourcc_[1], fourcc_[2], fourcc_[3]);
      cap_.set(cv::CAP_PROP_FOURCC, code);
    }

    // Una read di “warm-up” per far partire lo stream e validare output.
    cv::Mat tmp;
    if (!cap_.read(tmp) || tmp.empty()) {
      // Non crashiamo: ma rendiamo esplicito che non arrivano frame.
      RCLCPP_ERROR(get_logger(), "Camera opened but first read() returned empty frame");
    }
  }

  void tick()
  {
    cv::Mat frame;
    if (!cap_.isOpened()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *this->get_clock(), 2000, "VideoCapture not opened");
      return;
    }

    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "Empty frame");
      return;
    }

    // Costruisci sensor_msgs/Image a mano (no cv_bridge).
    sensor_msgs::msg::Image msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    msg.height = static_cast<uint32_t>(frame.rows);
    msg.width  = static_cast<uint32_t>(frame.cols);

    // OpenCV tipicamente produce BGR8 per webcam (3 canali). Gestiamo 1 o 3 canali.
    if (frame.type() == CV_8UC3) {
      msg.encoding = "bgr8";
      msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
      msg.data.resize(frame.total() * frame.elemSize());
      std::memcpy(msg.data.data(), frame.data, msg.data.size());
    } else if (frame.type() == CV_8UC1) {
      msg.encoding = "mono8";
      msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
      msg.data.resize(frame.total() * frame.elemSize());
      std::memcpy(msg.data.data(), frame.data, msg.data.size());
    } else {
      // Converti in BGR8 per coerenza.
      cv::Mat bgr;
      frame.convertTo(bgr, CV_8UC3);
      msg.encoding = "bgr8";
      msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(bgr.step);
      msg.data.resize(bgr.total() * bgr.elemSize());
      std::memcpy(msg.data.data(), bgr.data, msg.data.size());
    }

    pub_->publish(msg);
  }

  int device_index_;
  int width_;
  int height_;
  int fps_;
  std::string frame_id_;
  std::string topic_;
  std::string fourcc_;
  bool force_v4l2_;
  bool disable_gst_;
  bool disable_ocl_;
  bool opencv_debug_;

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<SimpleV4L2CameraNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "Fatal: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
