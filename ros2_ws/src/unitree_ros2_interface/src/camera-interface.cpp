/**
 * @file camera-interface.cpp
 * @brief Implementation of the CameraInterface class managing camera operations.
 */

#include "unitree_ros2_interface/camera-interface.hpp"
#include <memory>

#define RGB_PCL true

using namespace std::chrono_literals;

namespace unitree_ros2_interface {

    CameraInterfaceNode::CameraInterfaceNode(const rclcpp::NodeOptions& /* options */, const std::string& device_id, rclcpp::Node::SharedPtr node)
        : node_(node)
        , device_id_(device_id)
        , it_(node_) {

        package_share = ament_index_cpp::get_package_share_directory("unitree_ros2_interface");
        calibration_path = package_share + "/calibration";
        config_path = package_share + "/config";
        
        // Declare remaining parameters
        node_->declare_parameter<std::string>("config_file", "");
        node_->declare_parameter<bool>("publish_rectified", false);
        node_->declare_parameter<bool>("publish_depth", false);
        node_->declare_parameter<bool>("publish_pcl", false);
        node_->declare_parameter<bool>("verbose", true);

        // Get the camera name
        if (!node_->get_parameter("camera_name", camera_name_) || camera_name_.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "camera name not defined");
            throw std::runtime_error("camera_name missing");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Camera name: %s", camera_name_.c_str());
        }

        // Create the logger publisher early so we can use it for important messages
        pub_log_ = node_->create_publisher<std_msgs::msg::String>(camera_name_ + "_interface_log", 10);
        pub_log_->publish(std_msgs::msg::String().set__data("Camera name set to: " + camera_name_));

        if (!node_->get_parameter("config_file", config_file_) || config_file_.empty()) {
            RCLCPP_WARN(node_->get_logger(), "No config file provided, proceeding with device ID");
            pub_log_->publish(std_msgs::msg::String().set__data("WARNING: No config file provided, proceeding with device ID"));
            use_config = false;
        } else {
            RCLCPP_INFO(node_->get_logger(), "Using config file: %s", config_file_.c_str());
            pub_log_->publish(std_msgs::msg::String().set__data("Using config file: " + config_file_));
            use_config = true;
        }

        if(getCameraInfo(camera_name_, camera_info_)) {
            RCLCPP_INFO(node_->get_logger(), "Camera info loaded for device ID: %s", device_id_.c_str());
            pub_log_->publish(std_msgs::msg::String().set__data("Camera info loaded for device ID: " + device_id_));
        } else {
            RCLCPP_WARN(node_->get_logger(), "Unknown device ID: %s. Using default camera info.", device_id_.c_str());
            pub_log_->publish(std_msgs::msg::String().set__data("WARNING: Unknown device ID: " + device_id_ + ". Using default camera info."));
        }

        node_->get_parameter("camera_name_left",  camera_name_left_);
        node_->get_parameter("camera_name_right", camera_name_right_);
        node_->get_parameter("camera_info_url_left", camera_info_url_left_);
        node_->get_parameter("camera_info_url_right", camera_info_url_right_);
        node_->get_parameter("config_file", config_file_);
        node_->get_parameter("publish_rectified", publish_rectified_);
        node_->get_parameter("publish_depth", publish_depth_);
        node_->get_parameter("publish_pcl", publish_pcl_);
        node_->get_parameter("verbose", verbose_);

        // Initialize Camera Info Publishers with proper parameters
        camera_info_left_ = std::make_unique<CameraInfoPublisher>(node_, camera_name_left_, camera_info_url_left_, CameraSide::LEFT);
        camera_info_right_ = std::make_unique<CameraInfoPublisher>(node_, camera_name_right_, camera_info_url_right_, CameraSide::RIGHT);

        // Logger publisher was created earlier

        rawFrameSize = cv::Size(1856, 800);  // raw frame size
        rectFrameSize = cv::Size(928, 400);  // rectified frame size

        if(use_config) {
            unitreeCamera = std::make_shared<UnitreeCamera>(config_file_);
        } else {
            unitreeCamera = std::make_shared<UnitreeCamera>(static_cast<int>(camera_info_.nodeId));
            unitreeCamera->setPosNumber(static_cast<int>(camera_info_.devId));
            unitreeCamera->setRawFrameRate(30.0f);
            unitreeCamera->setRawFrameSize(rawFrameSize);
        }

        if(!unitreeCamera->isOpened()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open camera with name: %s", camera_name_.c_str());
            throw std::runtime_error("Camera initialization failed");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Camera opened successfully with name: %s", camera_name_.c_str());
            pub_log_->publish(std_msgs::msg::String().set__data("Camera opened successfully with name: " + camera_name_));
        }

        /* Note: If you are working with high resolution images, do not use shared memory */
        if(publish_depth_ || publish_pcl_) {
            if(!unitreeCamera->startCapture()) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to start camera capture for depth/PCL with name: %s", camera_name_.c_str());
                throw std::runtime_error("Camera capture start failed");
            }
            if(!unitreeCamera->startStereoCompute()) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to start camera compute for depth/PCL with name: %s", camera_name_.c_str());
                throw std::runtime_error("Camera compute start failed");
            }
        }else {
            if(!unitreeCamera->startCapture()) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to start camera capture with name: %s", camera_name_.c_str());
                throw std::runtime_error("Camera capture start failed");
            }
        }

        if(publish_rectified_) {
            pub_left_image_ = it_.advertise(camera_info_.name + "/left/image_rect", 10);
            pub_right_image_ = it_.advertise(camera_info_.name + "/right/image_rect", 10);
            pub_rect_perspective_ = it_.advertise(camera_info_.name + "/rect_perspective", 10);

            if(!use_config) {
                unitreeCamera->setRectFrameSize(rectFrameSize);
            }

            RCLCPP_INFO(node_->get_logger(), "Rectified image publishing enabled for camera: %s", camera_name_.c_str());
            pub_log_->publish(std_msgs::msg::String().set__data("Rectified image publishing enabled for camera: " + camera_name_));
        }else {
            pub_left_image_ = it_.advertise(camera_info_.name + "/left/image_raw", 10);
            pub_right_image_ = it_.advertise(camera_info_.name + "/right/image_raw", 10);

            RCLCPP_INFO(node_->get_logger(), "Raw image publishing enabled for camera: %s", camera_name_.c_str());
            pub_log_->publish(std_msgs::msg::String().set__data("Raw image publishing enabled for camera: " + camera_name_));
        }

        if(publish_depth_) {
            pub_depth_image_ = it_.advertise(camera_info_.name + "/depth/image_depth", 10);
            RCLCPP_INFO(node_->get_logger(), "Depth image publishing enabled for camera: %s", camera_name_.c_str());
            pub_log_->publish(std_msgs::msg::String().set__data("Depth image publishing enabled for camera: " + camera_name_));
        }

        if(publish_pcl_) {
            pub_pcl_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(camera_info_.name + "/point_cloud", 10);
            RCLCPP_INFO(node_->get_logger(), "PointCloud publishing enabled for camera: %s", camera_name_.c_str());
            pub_log_->publish(std_msgs::msg::String().set__data("PointCloud publishing enabled for camera: " + camera_name_));
        }

        RCLCPP_INFO(node_->get_logger(), "CameraInterfaceNode created succesfully with name: %s", camera_name_.c_str());
        pub_log_->publish(std_msgs::msg::String().set__data("CameraInterfaceNode created succesfully with name: " + camera_name_));
    }

    CameraInterfaceNode::~CameraInterfaceNode() {
        RCLCPP_INFO(node_->get_logger(), "CameraInterfaceNode destroyed");
    }

    void CameraInterfaceNode::run() {
        RCLCPP_INFO(node_->get_logger(), "Staring acquisition loop for camera: %s", camera_name_.c_str());
        pub_log_->publish(std_msgs::msg::String().set__data("Staring acquisition loop for camera: " + camera_name_));

        std_msgs::msg::Header header_left, header_right, header_depth, header_feim;

        header_left.frame_id = camera_info_.camera_frame_left;
        header_right.frame_id = camera_info_.camera_frame_right;
        header_depth.frame_id = camera_info_.camera_frame;
        header_feim.frame_id = camera_info_.camera_frame;

        while(rclcpp::ok() && unitreeCamera->isOpened()) {
           
            if(!isRunning) break;

            cv::Mat feim, left, right;
            std::chrono::microseconds timestamp;
            rclcpp::Time now = node_->now();

            if(!publish_rectified_) {
                if(!unitreeCamera->getStereoFrame(right, left, timestamp)) {
                    usleep(1000);
                    continue;
                }
            }else {
                if(!unitreeCamera->getRectStereoFrame(right, left, feim, timestamp)) {
                    usleep(1000);
                    continue;
                }
            }

            if(publish_depth_) {
                cv::Mat depth;
                if(!unitreeCamera->getDepthFrame(depth, true, timestamp)) {
                    usleep(1000);
                    continue;
                }
                header_depth.stamp = now;
                cv_bridge::CvImage depth_msg(header_depth, sensor_msgs::image_encodings::TYPE_16UC1, depth);
                pub_depth_image_.publish(depth_msg.toImageMsg());
            }

            cv_bridge::CvImage left_msg(header_left, sensor_msgs::image_encodings::BGR8, left);
            cv_bridge::CvImage right_msg(header_right, sensor_msgs::image_encodings::BGR8, right);

            header_left.stamp = now;
            header_right.stamp = now;

            if(!publish_rectified_) {
                camera_info_left_->publish(now);
                camera_info_right_->publish(now);
            }else {
                header_feim.stamp = now;
                cv_bridge::CvImage cv_image_feim(header_feim, sensor_msgs::image_encodings::BGR8, feim);
                pub_rect_perspective_.publish(cv_image_feim.toImageMsg());
            }

            pub_left_image_.publish(left_msg.toImageMsg());
            pub_right_image_.publish(right_msg.toImageMsg());

            if(publish_pcl_) {
                std::chrono::microseconds pcl_timestamp;
                sensor_msgs::msg::PointCloud2 pcl_msg;

                #if RGB_PCL
                std::vector<PCLType> pcl_vec;
                if(!unitreeCamera->getPointCloud(pcl_vec, pcl_timestamp)){
                    usleep(1000);
                    continue;
                }
                #else
                std::vector<cv::Vec3f> pcl_vec;
                if(!unitreeCamera->getPointCloud(pcl_vec, pcl_timestamp)){
                    usleep(1000);
                    continue;
                }
                #endif
            
                // Convert the Vector container to a PointCloud Object
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                for(const auto& point : pcl_vec) {
                    pcl::PointXYZRGB p;
                    p.x = point.pts[0];
                    p.y = point.pts[1];
                    p.z = point.pts[2];
                    p.r = point.clr[2];
                    p.g = point.clr[1];
                    p.b = point.clr[0];
                    cloud->points.push_back(p);
                }

                // Remove outliers from the point cloud
                pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
                sor.setInputCloud(cloud);
                sor.setMeanK(30);
                sor.setStddevMulThresh(1.0f);

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
                sor.filter(*cloud_filtered);

                pcl::toROSMsg(*cloud_filtered, pcl_msg);
                pcl_msg.header.stamp = now;
                pcl_msg.header.frame_id = camera_info_.camera_frame;
                pub_pcl_->publish(pcl_msg);            
            }

            rclcpp::spin_some(node_);
        }

        RCLCPP_INFO(node_->get_logger(), "Exiting acquisition loop for camera: %s", camera_name_.c_str());
        pub_log_->publish(std_msgs::msg::String().set__data("Exiting acquisition loop for camera: " + camera_name_));

        if(publish_depth_ || publish_pcl_) {
            unitreeCamera->stopStereoCompute();
            usleep(500000);
        }

        unitreeCamera->stopCapture();

        RCLCPP_INFO(node_->get_logger(), "CameraInterfaceNode run method exited cleanly for camera: %s", camera_name_.c_str());
        pub_log_->publish(std_msgs::msg::String().set__data("CameraInterfaceNode run method exited cleanly for camera: " + camera_name_));
    }

} // namespace unitree_ros2_interface
