#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_name = "unitree_ros2_interface"

    node_name = LaunchConfiguration("node_name")
    param_file_name = LaunchConfiguration("param_file_name")

    # Build complete path: <share>/<pkg>/config/<param_file_name>
    params_file_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        "config",
        param_file_name
    ])

    declare_node_name = DeclareLaunchArgument(
        "node_name",
        default_value="ultrasound_interface",
        description="ROS2 node name.",
    )

    declare_param_file_name = DeclareLaunchArgument(
        "param_file_name",
        default_value="ultrasound_nano_interface.yaml",
        description="YAML parameter file name in <pkg_share>/config/.",
    )

    ultrasound_interface_node = Node(
        package=pkg_name,
        executable="ultrasound_interface_node",
        name=node_name,
        output="screen",
        parameters=[params_file_path],
    )

    return LaunchDescription([
        declare_node_name,
        declare_param_file_name,
        ultrasound_interface_node,
    ])
