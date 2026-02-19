#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_name = "unitree_ros2_interface"

    node_name = LaunchConfiguration("node_name")
    node_namespace = LaunchConfiguration("node_namespace")
    log_level = LaunchConfiguration("log_level")

    # Nuovo: nome file params (solo filename, non path)
    param_file_name = LaunchConfiguration("param_file_name")
    #kill_script_sh = PathJoinSubstitution([FindPackageShare(pkg_name), "scripts", "kill.sh"])

    # Costruzione path completo: <share>/<pkg>/config/<param_file_name>
    params_file_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        "config",
        param_file_name
    ])

    declare_node_name = DeclareLaunchArgument(
        "node_name",
        default_value="udp_camera_node",
        description="ROS2 node name (unique per istanza).",
    )

    declare_node_namespace = DeclareLaunchArgument(
        "node_namespace",
        default_value="",
        description="Namespace del nodo (lascia vuoto se vuoi namespace solo nei topic).",
    )

    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        description="Node log level: INFO, DEBUG, WARN, ERROR, FATAL.",
    )

    declare_param_file_name = DeclareLaunchArgument(
        "param_file_name",
        default_value="stereo_udp_front_camera_config.yaml",
        description="Nome del file YAML in <pkg_share>/config/ (es. bottom_camera_raw.yaml).",
    )

    camera_node = Node(
        package=pkg_name,
        executable="camera_udp_node",
        name=node_name,
        namespace=node_namespace,
        output="screen",
        parameters=[params_file_path],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # kill_script = ExecuteProcess(
    #     cmd=[kill_script_sh],
    #     shell=True,
    # )

    return LaunchDescription([
        # kill_script,  # kill any existing camera processes first
        declare_node_name,
        declare_node_namespace,
        declare_log_level,
        declare_param_file_name,
        camera_node,
    ])
