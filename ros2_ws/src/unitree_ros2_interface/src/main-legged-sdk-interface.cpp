#include <unitree_ros2_interface/legged-sdk-interface.hpp>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include <memory>
#include <thread>
#include <chrono>
#include <signal.h>

static bool keep_running = true;

void signal_handler(int signal) {
    std::string signal_name;
    switch (signal) {
        case SIGINT:
            signal_name = "SIGINT";
            break;
        case SIGTERM:
            signal_name = "SIGTERM";
            break;
        default:
            break;
    }
    keep_running = false;
    RCLCPP_INFO(rclcpp::get_logger("main"), "Shutdown signal received: %s", signal_name.c_str());
}

int main(int argc, char** argv) {
    // Setup signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create node options and declare parameters
    rclcpp::NodeOptions options;
    auto node = std::make_shared<LeggedSDKInterface>(options);

    // Use an executor explicitly (recommended if you may add timers/services/subs concurrently)
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();  // returns on shutdown signal

    RCLCPP_INFO(node->get_logger(), "Unitree ROS2 Interface stopped");
        
    // Cleanup
    rclcpp::shutdown();
    return 0;
}