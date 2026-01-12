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
    node_namespace = LaunchConfiguration("node_namespace")
    log_level = LaunchConfiguration("log_level")
    param_file_name = LaunchConfiguration("param_file_name")

    # Build complete path: <share>/<pkg>/config/<param_file_name>
    params_file_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        "config",
        param_file_name
    ])

    declare_node_name = DeclareLaunchArgument(
        "node_name",
        default_value="unitree_low_level_interface",
        description="ROS2 node name.",
    )

    declare_node_namespace = DeclareLaunchArgument(
        "node_namespace",
        default_value="",
        description="Node namespace.",
    )

    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        description="Node log level: INFO, DEBUG, WARN, ERROR, FATAL.",
    )

    declare_param_file_name = DeclareLaunchArgument(
        "param_file_name",
        default_value="low_level_interface.yaml",
        description="YAML parameter file name in <pkg_share>/config/.",
    )

    low_interface_node = Node(
        package=pkg_name,
        executable="low_interface_node",
        name=node_name,
        namespace=node_namespace,
        output="screen",
        parameters=[params_file_path],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription([
        declare_node_name,
        declare_node_namespace,
        declare_log_level,
        declare_param_file_name,
        low_interface_node,
    ])
