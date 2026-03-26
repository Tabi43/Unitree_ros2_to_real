#!/usr/bin/env python3
# Main board (192.168.123.15):
#   - bottom_camera  (USB stereo)
#   - face_camera    (UDP)
#   - ultrasound     (nano variant)
#   - legged_sdk_interface

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # --- Launch configurations ---
    namespace = LaunchConfiguration("namespace")

    # Bottom camera (USB)
    bottom_camera_name       = LaunchConfiguration("bottom_camera_name")
    bottom_param_file_name   = LaunchConfiguration("bottom_param_file_name")

    # Face camera (UDP)
    face_camera_name         = LaunchConfiguration("face_camera_name")
    face_camera_param_file   = LaunchConfiguration("face_camera_param_file")

    # Common camera parameters
    enable_disparity  = LaunchConfiguration("enable_disparity")
    enable_pcl        = LaunchConfiguration("enable_pcl")
    use_intra_process = LaunchConfiguration("use_intra_process")
    respawn           = LaunchConfiguration("respawn")
    respawn_delay     = LaunchConfiguration("respawn_delay")

    # Ultrasound
    enable_ultrasound     = LaunchConfiguration("enable_ultrasound")
    ultrasound_param_file = LaunchConfiguration("ultrasound_param_file")
    ultrasound_node_name  = LaunchConfiguration("ultrasound_node_name")

    # Use camera_base instead of container
    camera_base = LaunchConfiguration("camera_base")

    # Legged SDK
    enable_legged_sdk     = LaunchConfiguration("enable_legged_sdk")
    legged_sdk_node_name  = LaunchConfiguration("legged_sdk_node_name")
    legged_sdk_param_file = LaunchConfiguration("legged_sdk_param_file")

    # --- Declare launch arguments ---
    declared_args = [
        DeclareLaunchArgument("namespace", default_value="unitree_go1"),

        # Bottom camera
        DeclareLaunchArgument("bottom_camera_name",     default_value="bottom_camera"),
        DeclareLaunchArgument("bottom_param_file_name", default_value="stereo_bottom_camera_config.yaml"),

        # Face camera (UDP)
        DeclareLaunchArgument("face_camera_name",       default_value="front_camera"),
        DeclareLaunchArgument("face_camera_param_file", default_value="stereo_udp_front_camera_config.yaml"),

        # Common camera parameters
        DeclareLaunchArgument("enable_disparity",  default_value="false"),
        DeclareLaunchArgument("enable_pcl",        default_value="true"),
        DeclareLaunchArgument("use_intra_process", default_value="true"),
        DeclareLaunchArgument("respawn",           default_value="true"),
        DeclareLaunchArgument("respawn_delay",     default_value="5.0"),

        # Ultrasound (main board: Jetson Nano UART interface)
        DeclareLaunchArgument("enable_ultrasound",     default_value="true"),
        DeclareLaunchArgument("ultrasound_param_file", default_value="ultrasound_nano_interface.yaml"),
        DeclareLaunchArgument("ultrasound_node_name",  default_value="ultrasound_nano_interface"),

        # Use camera_base instead of container
        DeclareLaunchArgument("camera_base", default_value="false",
                              description="If true, use camera_base.launch.py/camera_udp_base.launch.py instead of container."),

        # Legged SDK interface
        DeclareLaunchArgument("enable_legged_sdk",     default_value="true"),
        DeclareLaunchArgument("legged_sdk_node_name",  default_value="legged_sdk_interface"),
        DeclareLaunchArgument("legged_sdk_param_file", default_value="legged_sdk_interface.yaml"),
    ]

    pkg_share = get_package_share_directory("unitree_ros2_interface")
    camera_container_launch     = os.path.join(pkg_share, "launch", "camera_container.launch.py")
    camera_udp_container_launch = os.path.join(pkg_share, "launch", "camera_udp_container.launch.py")
    camera_base_launch          = os.path.join(pkg_share, "launch", "camera_base.launch.py")
    camera_udp_base_launch      = os.path.join(pkg_share, "launch", "camera_udp_base.launch.py")
    ultrasound_launch           = os.path.join(pkg_share, "launch", "ultrasound_interface.launch.py")
    legged_sdk_launch           = os.path.join(pkg_share, "launch", "legged_sdk_interface.launch.py")

    # --- Bottom camera (USB) ---
    # bottom_camera_container = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(camera_container_launch),
    #     launch_arguments={
    #         "namespace":        namespace,
    #         "camera_name":      bottom_camera_name,
    #         "param_file_name":  bottom_param_file_name,
    #         "enable_disparity": enable_disparity,
    #         "enable_pcl":       enable_pcl,
    #         "use_intra_process": use_intra_process,
    #         "respawn":          respawn,
    #         "respawn_delay":    respawn_delay,
    #     }.items(),
    #     condition=UnlessCondition(camera_base),
    # )

    # --- Bottom camera base ---
    bottom_camera_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_base_launch),
        launch_arguments={
            "node_name":       bottom_camera_name,
            "namespace":       namespace,
            "param_file_name": bottom_param_file_name,
        }.items(),
        #condition=IfCondition(camera_base),
    )

    # --- Face camera (UDP) ---
    face_camera_udp_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_udp_container_launch),
        launch_arguments={
            "namespace":        namespace,
            "camera_name":      face_camera_name,
            "param_file_name":  face_camera_param_file,
            "enable_disparity": enable_disparity,
            "enable_pcl":       enable_pcl,
            "use_intra_process": use_intra_process,
            "respawn":          respawn,
            "respawn_delay":    respawn_delay,
        }.items(),
        condition=UnlessCondition(camera_base),
    )

    # # --- Face camera base (UDP) ---
    face_camera_udp_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_udp_base_launch),
        launch_arguments={
            "node_name":       face_camera_name,
            "namespace":       namespace,
            "param_file_name": face_camera_param_file,
        }.items(),
        condition=IfCondition(camera_base),
    )

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
        bottom_camera_base,
        face_camera_udp_container,
        face_camera_udp_base,
        ultrasound_interface,
        legged_sdk_interface,
    ])
