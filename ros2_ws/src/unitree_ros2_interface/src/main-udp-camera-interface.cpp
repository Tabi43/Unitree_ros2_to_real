/**
 *  @file main-udp-camera-interface.cpp
 *  @brief Main entry point for the UDP camera interface node.
 * 
 */

#include <rclcpp/rclcpp.hpp>
#include "unitree_ros2_interface/udp-camera-interface.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Create the node with default options
    rclcpp::NodeOptions options;
    auto node = std::make_shared<unitree_ros2_interface::UnitreeUdpCameraInterface>(options);
    
    try {
        // Spin the node
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "UDP Camera interface failed: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}