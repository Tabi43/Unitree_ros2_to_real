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
    node_namespace = LaunchConfiguration("namespace")
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
        default_value="unitree_legged_sdk_interface",
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

    declare_param_file_name = DeclareLaunchArgument(
        "param_file_name",
        default_value="legged_sdk_interface.yaml",
        description="YAML parameter file name in <pkg_share>/config/.",
    )

    # Empty by default. The node now raises only its two UDP loop threads to SCHED_FIFO
    # (udp_thread_priority), which is what a "chrt -f 80" prefix could not express: it put
    # the whole process - ROS executor, DDS, publisher thread - at one blanket real-time
    # priority, so DDS serialization competed with the command loop instead of yielding to
    # it. Set this to e.g. "chrt -f 80" only if you deliberately want process-wide RT.
    declare_rt_prefix = DeclareLaunchArgument(
        "rt_prefix",
        default_value="",
        description="Optional command prefix for the node, e.g. 'chrt -f 80'. Normally left "
                    "empty: the node applies real-time priority per thread instead.",
    )

    legged_sdk_interface = Node(
        package=pkg_name,
        executable="legged_sdk_interface_node",
        name=node_name,
        namespace=node_namespace,
        output="screen",
        prefix=LaunchConfiguration("rt_prefix"),
        parameters=[params_file_path],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription([
        declare_node_name,
        declare_node_namespace,
        declare_log_level,
        declare_param_file_name,
        declare_rt_prefix,
        legged_sdk_interface,
    ])
