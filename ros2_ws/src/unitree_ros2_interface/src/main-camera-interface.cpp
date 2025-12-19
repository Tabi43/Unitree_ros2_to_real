/**
 * @file main-camera-interface.cpp
 * @brief Main entry point for the camera interface node.
 */

#include <rclcpp/rclcpp.hpp>
#include "unitree_ros2_interface/camera-interface.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Get parameters from command line or use defaults
    std::string device_id = "front_camera";
    if (argc > 1) {
        device_id = argv[1];
    }
    
    // Create the node
    auto node = rclcpp::Node::make_shared("camera_interface_node");
    
    try {
        // Create camera interface
        auto camera_interface = std::make_shared<unitree_ros2_interface::CameraInterfaceNode>(
            rclcpp::NodeOptions(), device_id, node);
        
        // Run the node
        camera_interface->run();
        
        // Spin the node
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Camera interface failed: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
