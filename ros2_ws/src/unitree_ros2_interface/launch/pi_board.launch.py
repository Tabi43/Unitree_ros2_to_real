#!/usr/bin/env python3
# Pi board (192.168.123.161):
#   - ultrasound (pi variant)
#   - face_lights
#   - chin camera (UDP)
#   - legged_sdk_interface

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # --- Launch configurations ---
    namespace = LaunchConfiguration("namespace")

    # Ultrasound
    enable_ultrasound     = LaunchConfiguration("enable_ultrasound")
    ultrasound_param_file = LaunchConfiguration("ultrasound_param_file")
    ultrasound_node_name  = LaunchConfiguration("ultrasound_node_name")

    # Face lights
    enable_face_lights    = LaunchConfiguration("enable_face_lights")
    face_lights_node_name = LaunchConfiguration("face_lights_node_name")

    # Chin camera (UDP)
    chin_camera_name        = LaunchConfiguration("chin_camera_name")
    chin_camera_param_file  = LaunchConfiguration("chin_camera_param_file")
    enable_disparity        = LaunchConfiguration("enable_disparity")
    enable_pcl              = LaunchConfiguration("enable_pcl")
    use_intra_process       = LaunchConfiguration("use_intra_process")
    respawn                 = LaunchConfiguration("respawn")
    respawn_delay           = LaunchConfiguration("respawn_delay")

    # Legged SDK
    enable_legged_sdk     = LaunchConfiguration("enable_legged_sdk")
    legged_sdk_node_name  = LaunchConfiguration("legged_sdk_node_name")
    legged_sdk_param_file = LaunchConfiguration("legged_sdk_param_file")

    # --- Declare launch arguments ---
    declared_args = [
        DeclareLaunchArgument("namespace", default_value="unitree_go1"),

        # Ultrasound (pi board: dedicated pi UART interface)
        DeclareLaunchArgument("enable_ultrasound",     default_value="true"),
        DeclareLaunchArgument("ultrasound_param_file", default_value="ultrasound_pi_interface.yaml"),
        DeclareLaunchArgument("ultrasound_node_name",  default_value="ultrasound_pi_interface"),

        # Face lights
        DeclareLaunchArgument("enable_face_lights",    default_value="true"),
        DeclareLaunchArgument("face_lights_node_name", default_value="face_lights"),

        # Chin camera (UDP)
        DeclareLaunchArgument("chin_camera_name",       default_value="chin_camera"),
        DeclareLaunchArgument("chin_camera_param_file", default_value="stereo_udp_chin_camera_config.yaml"),
        DeclareLaunchArgument("enable_disparity",       default_value="false"),
        DeclareLaunchArgument("enable_pcl",             default_value="false"),
        DeclareLaunchArgument("use_intra_process",      default_value="true"),
        DeclareLaunchArgument("respawn",                default_value="true"),
        DeclareLaunchArgument("respawn_delay",          default_value="2.0"),

        # Legged SDK interface
        DeclareLaunchArgument("enable_legged_sdk",     default_value="true"),
        DeclareLaunchArgument("legged_sdk_node_name",  default_value="legged_sdk_interface"),
        DeclareLaunchArgument("legged_sdk_param_file", default_value="legged_sdk_interface.yaml"),
    ]

    pkg_share = get_package_share_directory("unitree_ros2_interface")
    ultrasound_launch           = os.path.join(pkg_share, "launch", "ultrasound_interface.launch.py")
    camera_udp_container_launch = os.path.join(pkg_share, "launch", "camera_udp_container.launch.py")
    legged_sdk_launch           = os.path.join(pkg_share, "launch", "legged_sdk_interface.launch.py")

    # --- Ultrasound ---
    ultrasound_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ultrasound_launch),
        launch_arguments={
            "namespace":       namespace,
            "node_name":       ultrasound_node_name,
            "param_file_name": ultrasound_param_file,
        }.items(),
        condition=IfCondition(enable_ultrasound),
    )

    # --- Face lights ---
    face_lights_node = Node(
        package="unitree_ros2_interface",
        executable="face_lights_node",
        name=face_lights_node_name,
        namespace=namespace,
        output="screen",
        condition=IfCondition(enable_face_lights),
    )

    # --- Chin camera (UDP) ---
    chin_camera_udp_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_udp_container_launch),
        launch_arguments={
            "namespace":        namespace,
            "camera_name":      chin_camera_name,
            "param_file_name":  chin_camera_param_file,
            "enable_disparity": enable_disparity,
            "enable_pcl":       enable_pcl,
            "use_intra_process": use_intra_process,
            "respawn":          respawn,
            "respawn_delay":    respawn_delay,
        }.items(),
    )

    # --- Legged SDK interface ---
    legged_sdk_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(legged_sdk_launch),
        launch_arguments={
            "namespace":       namespace,
            "node_name":       legged_sdk_node_name,
            "param_file_name": legged_sdk_param_file,
            "log_level":       "INFO",
        }.items(),
        condition=IfCondition(enable_legged_sdk),
    )

    return LaunchDescription([
        *declared_args,
        ultrasound_interface,
        face_lights_node,
        chin_camera_udp_container,
        legged_sdk_interface,
    ])