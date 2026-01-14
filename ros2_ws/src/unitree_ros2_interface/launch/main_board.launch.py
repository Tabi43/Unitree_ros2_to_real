#!/usr/bin/env python3
# Main board launches bottom_camera, low_level_interface, and high_level_interface

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # --- Launch configurations ---
    namespace = LaunchConfiguration("namespace")
    
    # Bottom camera parameters
    bottom_camera_name = LaunchConfiguration("bottom_camera_name")
    bottom_param_file_name = LaunchConfiguration("bottom_param_file_name")
    
    # Common camera parameters
    enable_disparity = LaunchConfiguration("enable_disparity")
    enable_pcl = LaunchConfiguration("enable_pcl")
    use_intra_process = LaunchConfiguration("use_intra_process")
    respawn = LaunchConfiguration("respawn")
    respawn_delay = LaunchConfiguration("respawn_delay")

    # --- Declare launch arguments ---
    declared_args = [
        DeclareLaunchArgument("namespace", default_value=""),
        
        # Bottom camera
        DeclareLaunchArgument("bottom_camera_name", default_value="bottom_camera"),
        DeclareLaunchArgument("bottom_param_file_name", default_value="stereo_bottom_camera_config.yaml"),
        
        # Common camera parameters
        DeclareLaunchArgument("enable_disparity", default_value="false"),
        DeclareLaunchArgument("enable_pcl", default_value="true"),
        DeclareLaunchArgument("use_intra_process", default_value="true"),
        DeclareLaunchArgument("respawn", default_value="true"),
        DeclareLaunchArgument("respawn_delay", default_value="5.0")
    ]

    # Get package share directory
    pkg_share = get_package_share_directory("unitree_ros2_interface")
    camera_container_launch = os.path.join(pkg_share, "launch", "camera_container.launch.py")

    # --- Bottom camera container ---
    bottom_camera_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_container_launch),
        launch_arguments={
            "namespace": namespace,
            "camera_name": bottom_camera_name,
            "param_file_name": bottom_param_file_name,
            "enable_disparity": enable_disparity,
            "enable_pcl": enable_pcl,
            "use_intra_process": use_intra_process,
            "respawn": respawn,
            "respawn_delay": respawn_delay,
        }.items(),
    )

    return LaunchDescription([
        *declared_args,
        bottom_camera_container
    ])
