#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = "sport_controller"

    node_name = LaunchConfiguration("node_name")
    node_namespace = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration("log_level")

    declare_node_name = DeclareLaunchArgument(
        "node_name",
        default_value="sport_controller",
        description="ROS2 node name.",
    )

    declare_node_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="unitree_go1",
        description="Node namespace.",
    )

    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        description="Node log level: INFO, DEBUG, WARN, ERROR, FATAL.",
    )

    sport_controller_node = Node(
        package=pkg_name,
        executable="junior_control",
        name=node_name,
        namespace=node_namespace,
        output="screen",
        prefix="chrt -f 60",
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription([
        declare_node_name,
        declare_node_namespace,
        declare_log_level,
        sport_controller_node,
    ])
