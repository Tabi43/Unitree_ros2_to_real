#include "rclcpp/rclcpp.hpp"
#include "unitree_ros2_interface/high-interface-node.hpp"

#include <string>
#include <functional>   // std::bind

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  // Keep same semantic as ROS1 code
  std::string prefix = "/unitree_go1";

  // Create the node (your ROS2 port)
  auto highsdk = std::make_shared<HighInterface>(prefix);

  // Use an executor explicitly (recommended if you may add timers/services/subs concurrently)
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(highsdk);
  exec.spin();  // returns on shutdown signal

  RCLCPP_INFO(rclcpp::get_logger("unitree_ros_high_interface"), "Unitree ROS2 High Interface stopped");

  rclcpp::shutdown();  // safe even if already shutdown by SIGINT 
  return 0;
}
