#ifndef UNITREE_ROS2_INTERFACE_CAMERA_INTERFACE_HPP_
#define UNITREE_ROS2_INTERFACE_CAMERA_INTERFACE_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>
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
#include <string>

#include <opencv2/opencv.hpp>

namespace unitree_ros2_interface {

class UnitreeCameraInterface : public rclcpp::Node {
    public:
    explicit UnitreeCameraInterface(const rclcpp::NodeOptions & options);
    ~UnitreeCameraInterface() override;

    private:
    enum class OutputEncoding {
      BGR8,
      RGB8,
      MONO8
    };

    // ---- lifecycle-like internal steps ----
    void declare_and_get_params();
    void validate_params_or_throw();
    void configure_opencv_env();
    void open_camera();
    void apply_v4l2_controls_best_effort();
    void load_camera_infos_best_effort();
    void start_capture_thread();
    void stop_capture_thread();

    // ---- capture / process loops ----
    void capture_loop();   ///< Thread A: drains V4L2 buffer, stores latest frame
    void process_loop();   ///< Thread B: picks latest frame, converts and publishes
    bool build_rectification_maps();
    void rectify_loop();   ///< Thread C: applies rectification maps and publishes

    // ---- helpers ----
    static std::string normalize_ns(const std::string & ns);
    std::string make_topic(const std::string & suffix) const;  // suffix relative to <camera_name>
    void publish_log(const std::string & level, const std::string & msg);

    bool load_camera_info_from_url(
    const std::string & url,
    sensor_msgs::msg::CameraInfo & out_info,
    const std::string & fallback_name);

    bool set_v4l2_control_best_effort(int control_id, int value);

    static int fourcc_from_string(const std::string & fmt);
    static std::string upper(const std::string & s);

    static inline bool has_scheme(const std::string& s) {
        return s.find("://") != std::string::npos;   // es: file://, package://
    }

    std::string build_camera_info_url(const std::string& package_name, const std::string& calib_filename_or_url);

    private:
    // ---- parameters ----
    std::string namespace_param_;
    std::string camera_name_;

    int device_index_{0};
    std::string device_path_;

    /// Raw width frame dimensions from the read from /dev/videoX
    int raw_width_{940};
    /// Raw height frame dimensions from the read from /dev/videoX
    int raw_height_{400};
    /// Raw fps read from /dev/videoX
    double fps_{30.0};

    std::string stereo_layout_{"side_by_side"};
    bool swap_lr_{false};

    std::string pixel_format_{"MJPG"};
    bool force_v4l2_{true};
    int buffer_size_{4};
    int warmup_frames_{0};
    /// Whether OpenCV should convert the raw pixel format to BGR.
    /// Maps directly to CAP_PROP_CONVERT_RGB. Default true (normal use);
    /// set to false only if you want to receive native YUYV / other raw data.
    bool convert_rgb_{true};
    std::string encoding_str_{"bgr8"};
    std::string encoding_{sensor_msgs::image_encodings::BGR8};
    OutputEncoding output_encoding_{OutputEncoding::BGR8};

    std::string left_frame_id_;
    std::string right_frame_id_;

    std::string camera_info_left_name_;
    std::string camera_info_right_name_;

    bool use_image_transport_{false};
    bool publish_mono_{true};   // must match declare_parameter default in declare_and_get_params()
    bool publish_rectified_color_{false};
    bool publish_rectified_mono_{false};
    bool use_image_transport_rectified_{false};

    // OpenCV env knobs
    std::string opencv_priority_list_{"V4L2"};
    bool opencv_videoio_debug_{false};
    std::string opencv_log_level_{"INFO"};
    bool opencv_disable_opencl_{true};

    // V4L2 controls (optional)
    std::optional<int> v4l2_exposure_auto_;
    std::optional<int> v4l2_exposure_absolute_;
    std::optional<int> v4l2_gain_;
    std::optional<int> v4l2_brightness_;
    std::optional<int> v4l2_contrast_;
    std::optional<int> v4l2_saturation_;
    std::optional<int> v4l2_sharpness_;
    std::optional<int> v4l2_gamma_;
    std::optional<int> v4l2_wb_auto_;
    std::optional<int> v4l2_wb_temp_;
    std::optional<int> v4l2_power_line_freq_;
    std::optional<int> v4l2_backlight_comp_;

    // ---- publishers ----
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

    // ---- rectified publishers ----
    image_transport::Publisher pub_left_rect_color_transport_;
    image_transport::Publisher pub_right_rect_color_transport_;
    image_transport::Publisher pub_left_rect_mono_transport_;
    image_transport::Publisher pub_right_rect_mono_transport_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_rect_color_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_rect_color_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_rect_mono_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_rect_mono_;

    // ---- camera info cached ----
    sensor_msgs::msg::CameraInfo left_info_;
    sensor_msgs::msg::CameraInfo right_info_;
    bool left_info_loaded_{false};
    bool right_info_loaded_{false};

    // ---- capture ----
    cv::VideoCapture cap_;
    std::atomic<bool> running_{false};
    std::thread capture_thread_;
    std::thread process_thread_;

    // Latest-only slot between the two threads.
    // capture_loop always overwrites; process_loop always picks the most recent.
    cv::Mat              latest_frame_;
    std::mutex           frame_mutex_;
    std::condition_variable frame_cv_;
    bool                 frame_ready_{false};

    // Reused OpenCV buffers for full-frame conversion (avoids re-allocation).
    cv::Mat color_sbs_buf_;
    cv::Mat mono_sbs_buf_;

    // ---- rectification ----
    bool rectify_enabled_{false};
    cv::Mat rect_map1_left_, rect_map2_left_;
    cv::Mat rect_map1_right_, rect_map2_right_;
    std::thread rectify_thread_;
    cv::Mat rect_input_frame_;
    rclcpp::Time rect_input_stamp_;
    bool rect_input_ready_{false};
    std::mutex rect_mutex_;
    std::condition_variable rect_cv_;
    cv::Mat rect_left_buf_, rect_right_buf_;
    cv::Mat rect_left_color_buf_, rect_right_color_buf_;
    cv::Mat rect_left_mono_buf_, rect_right_mono_buf_;
};

}  // namespace unitree_ros2_interface

#endif  // UNITREE_ROS2_INTERFACE_CAMERA_INTERFACE_HPP_