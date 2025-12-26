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

  // Unitree SDK loops (these run in their own threads managed by LoopFunc)
  // If LoopFunc requires boost::bind in your SDK version, swap std::bind with boost::bind.
  UNITREE_LEGGED_SDK::LoopFunc loop_udpSend(
    "high_udp_send", 0.02, 3,
    std::bind(&HighInterface::highUdpSend, highsdk.get()));

  UNITREE_LEGGED_SDK::LoopFunc loop_udpRecv(
    "high_udp_recv", 0.02, 3,
    std::bind(&HighInterface::highUdpRecv, highsdk.get()));

  UNITREE_LEGGED_SDK::LoopFunc loop_joint_state(
    "joint_state", 0.002,
    std::bind(&HighInterface::pubJointState, highsdk.get()));

  UNITREE_LEGGED_SDK::LoopFunc loop_imu(
    "imu", 0.002,
    std::bind(&HighInterface::pubImu, highsdk.get()));

  UNITREE_LEGGED_SDK::LoopFunc loop_odom(
    "odom", 0.002,
    std::bind(&HighInterface::pubOdom, highsdk.get()));

  loop_udpSend.start();
  loop_udpRecv.start();
  loop_joint_state.start();
  loop_imu.start();
  loop_odom.start();

  RCLCPP_INFO(highsdk->get_logger(), "Unitree ROS2 High Interface started");

  // Use an executor explicitly (recommended if you may add timers/services/subs concurrently)
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(highsdk);
  exec.spin();  // returns on shutdown signal

  RCLCPP_INFO(rclcpp::get_logger("unitree_ros_high_interface"), "Unitree ROS2 High Interface stopped");

  rclcpp::shutdown();  // safe even if already shutdown by SIGINT 
  return 0;
}
