#!/usr/bin/env python3
# Body board launches left_camera and right_camera

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # --- Launch configurations ---
    namespace = LaunchConfiguration("namespace")
    
    # Left camera parameters
    left_camera_name = LaunchConfiguration("left_camera_name")
    left_container_name = LaunchConfiguration("left_container_name")
    left_param_file_name = LaunchConfiguration("left_param_file_name")
    
    # Right camera parameters
    right_camera_name = LaunchConfiguration("right_camera_name")
    right_container_name = LaunchConfiguration("right_container_name")
    right_param_file_name = LaunchConfiguration("right_param_file_name")
    
    # Common camera parameters
    enable_disparity = LaunchConfiguration("enable_disparity")
    enable_pcl = LaunchConfiguration("enable_pcl")
    use_intra_process = LaunchConfiguration("use_intra_process")
    respawn = LaunchConfiguration("respawn")
    respawn_delay = LaunchConfiguration("respawn_delay")

    # Ultrasound interface parameters
    enable_ultrasound = LaunchConfiguration("enable_ultrasound")
    ultrasound_param_file = LaunchConfiguration("ultrasound_param_file")
    ultrasound_node_name = LaunchConfiguration("ultrasound_node_name")

    # --- Declare launch arguments ---
    declared_args = [
        DeclareLaunchArgument("namespace", default_value=""),
        
        # Left camera
        DeclareLaunchArgument("left_camera_name", default_value="left_camera"),
        DeclareLaunchArgument("left_param_file_name", default_value="stereo_left_camera_config.yaml"),
        
        # Right camera
        DeclareLaunchArgument("right_camera_name", default_value="right_camera"),
        DeclareLaunchArgument("right_param_file_name", default_value="stereo_right_camera_config.yaml"),
        
        # Common camera parameters
        DeclareLaunchArgument("enable_disparity", default_value="false"),
        DeclareLaunchArgument("enable_pcl", default_value="true"),
        DeclareLaunchArgument("use_intra_process", default_value="true"),
        DeclareLaunchArgument("respawn", default_value="true"),
        DeclareLaunchArgument("respawn_delay", default_value="5.0"),

        DeclareLaunchArgument("enable_ultrasound", default_value="false"),
        DeclareLaunchArgument("ultrasound_param_file", default_value="ultrasound_nano_interface.yaml"),
        DeclareLaunchArgument("ultrasound_node_name", default_value="ultrasound_nano_interface"),
    ]

    # Get package share directory
    pkg_share = get_package_share_directory("unitree_ros2_interface")
    camera_container_launch = os.path.join(pkg_share, "launch", "camera_container.launch.py")
    ultrasound_launch = os.path.join(pkg_share, "launch", "ultrasound_nano_interface.launch.py")

    # --- Left camera container ---
    left_camera_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_container_launch),
        launch_arguments={
            "namespace": namespace,
            "camera_name": left_camera_name,
            "param_file_name": left_param_file_name,
            "enable_disparity": enable_disparity,
            "enable_pcl": enable_pcl,
            "use_intra_process": use_intra_process,
            "respawn": respawn,
            "respawn_delay": respawn_delay,
        }.items(),
    )

    # --- Right camera container ---
    right_camera_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_container_launch),
        launch_arguments={
            "namespace": namespace,
            "camera_name": right_camera_name,
            "param_file_name": right_param_file_name,
            "enable_disparity": enable_disparity,
            "enable_pcl": enable_pcl,
            "use_intra_process": use_intra_process,
            "respawn": respawn,
            "respawn_delay": respawn_delay,
        }.items(),
    )

    ultrasound_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ultrasound_launch),
        launch_arguments={
            "namespace": namespace,
            "param_file_name": ultrasound_param_file,
            "node_name": ultrasound_node_name
        }.items(),
        condition=IfCondition(enable_ultrasound),
    )

    return LaunchDescription([
        *declared_args,
        left_camera_container,
        right_camera_container,
        ultrasound_interface
    ])
