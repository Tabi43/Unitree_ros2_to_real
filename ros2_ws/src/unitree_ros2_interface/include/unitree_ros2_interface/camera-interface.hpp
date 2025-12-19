/**
 * @file camera-interface.hpp
 * @brief Header for the CameraInterface class managing camera operations.
 */

#ifndef UNITREE_ROS2_INTERFACE_CAMERA_INTERFACE_HPP_
#define UNITREE_ROS2_INTERFACE_CAMERA_INTERFACE_HPP_

// C Includes 
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <cerrno>
#include <cfenv>
#include <signal.h>
#include <filesystem>

// ROS2 Includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// OpenCV Includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// PCL Includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

//Unitree Camera SDK Includes
#include <UnitreeCameraSDK.hpp>
#include "unitree_ros2_interface/camera.hpp"

using namespace std::chrono_literals;

namespace unitree_ros2_interface {

    class CameraInfoPublisher {
    public:
        CameraInfoPublisher(rclcpp::Node::SharedPtr node,
                            const std::string& camera_name,
                            const std::string& camera_info_url,
                            const CameraSide& side):
                            camera_side_(side),
                            camera_topic_(""),
                            camera_info_url_(camera_info_url),
                            camera_name_(camera_name),
                            node_(node),
                            cam_info_manager_(node_.get(), camera_name_, camera_info_url_) {
            
            switch (camera_side_) {
                case CameraSide::LEFT:
                    camera_topic_ = camera_name_ + "/left/";
                    break;
                case CameraSide::RIGHT:
                    camera_topic_ = camera_name_ + "/right/";
                    break;        
            default:
                throw std::runtime_error("Camera side can only be: left, right, depth");
                break;
            }

            pub_camera_info_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(camera_topic_ + "camera_info", 10);
        }

        ~CameraInfoPublisher() = default;

        CameraSide camera_side_;
        std::string camera_topic_;
        std::string camera_info_url_;
        std::string camera_name_;

        rclcpp::Node::SharedPtr node_;
        camera_info_manager::CameraInfoManager cam_info_manager_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info_;

        inline void publish(rclcpp::Time stamp) {
            sensor_msgs::msg::CameraInfo cam_info = cam_info_manager_.getCameraInfo();
            cam_info.header.stamp = stamp;
            pub_camera_info_->publish(cam_info);
        }
    };

    class CameraInterfaceNode {
    public:
        // Basic Constructor
        CameraInterfaceNode(const rclcpp::NodeOptions& options, const std::string& device_id, rclcpp::Node::SharedPtr node);

        // Destructor
        ~CameraInterfaceNode();

        // Signal handler for graceful shutdown
        void sigKillHandler(int s);
        
        // Static signal handler that calls the instance method
        static void staticSigKillHandler(int s);

        // initialize the camera interface
        bool init();

        void run();

        void publishCameraInfo();
           
    private:
        std::shared_ptr<UnitreeCamera> unitreeCamera;
        std::unique_ptr<CameraInfoPublisher> camera_info_left_;
        std::unique_ptr<CameraInfoPublisher> camera_info_right_;
        CameraInfo camera_info_;

        rclcpp::Node::SharedPtr node_;

        std::string camera_name_;
        std::string device_id_;
        std::string config_file_;
        std::string camera_name_left_;
        std::string camera_name_right_;
        std::string camera_info_url_left_;
        std::string camera_info_url_right_;

        std::string package_share;
        std::string calibration_path;
        std::string config_path;

        bool isRunning = true;
        bool use_config = false;
        bool publish_rectified_ = false;
        bool publish_depth_ = false;
        bool publish_pcl_ = false;
        bool verbose_ = true;

        // Open CV variables
        cv::Size rawFrameSize;
        cv::Size rectFrameSize;

        // Publishers
        image_transport::ImageTransport it_;
        image_transport::Publisher pub_left_image_;
        image_transport::Publisher pub_right_image_;
        image_transport::Publisher pub_rect_perspective_;
        image_transport::Publisher pub_depth_image_;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_log_;        
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_;
    };



} // namespace unitree_ros2_interface

#endif  // UNITREE_ROS2_INTERFACE_CAMERA_INTERFACE_HPP_