#!/usr/bin/env python3
# Head board launches front_camera and chin_camera

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
    
    # Front camera parameters
    front_camera_name = LaunchConfiguration("front_camera_name")
    front_param_file_name = LaunchConfiguration("front_param_file_name")
    
    # Chin camera parameters
    chin_camera_name = LaunchConfiguration("chin_camera_name")
    chin_param_file_name = LaunchConfiguration("chin_param_file_name")
    
    # Common camera parameters
    enable_disparity = LaunchConfiguration("enable_disparity")
    enable_pcl = LaunchConfiguration("enable_pcl")
    use_intra_process = LaunchConfiguration("use_intra_process")
    respawn = LaunchConfiguration("respawn")
    respawn_delay = LaunchConfiguration("respawn_delay")

    # Other interfaces
    enable_ultrasound = LaunchConfiguration("enable_ultrasound")
    ultrasound_param_file = LaunchConfiguration("ultrasound_param_file")

    # Use camera_base instead of container
    camera_base = LaunchConfiguration("camera_base")

    # --- Declare launch arguments ---
    declared_args = [
        DeclareLaunchArgument("namespace", default_value=""),
        
        # Front camera
        DeclareLaunchArgument("front_camera_name", default_value="front_camera"),
        DeclareLaunchArgument("front_param_file_name", default_value="stereo_front_camera_config.yaml"),
        
        # Chin camera
        DeclareLaunchArgument("chin_camera_name", default_value="chin_camera"),
        DeclareLaunchArgument("chin_param_file_name", default_value="stereo_chin_camera_config.yaml"),
        
        # Common camera parameters
        DeclareLaunchArgument("enable_disparity", default_value="false"),
        DeclareLaunchArgument("enable_pcl", default_value="true"),
        DeclareLaunchArgument("use_intra_process", default_value="true"),
        DeclareLaunchArgument("respawn", default_value="true"),
        DeclareLaunchArgument("respawn_delay", default_value="5.0"),
        
        # Other interfaces
        DeclareLaunchArgument("enable_ultrasound", default_value="false"),
        DeclareLaunchArgument("ultrasound_param_file", default_value="ultrasound_nano_interface.yaml"),

        # Use camera_base instead of container
        DeclareLaunchArgument("camera_base", default_value="false",
                              description="If true, use camera_base.launch.py instead of camera_container.launch.py"),
    ]

    # Get package share directory
    pkg_share = get_package_share_directory("unitree_ros2_interface")
    camera_container_launch = os.path.join(pkg_share, "launch", "camera_container.launch.py")
    camera_base_launch      = os.path.join(pkg_share, "launch", "camera_base.launch.py")
    ultrasound_launch = os.path.join(pkg_share, "launch", "ultrasound_interface.launch.py")

    # --- Front camera container ---
    front_camera_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_container_launch),
        launch_arguments={
            "namespace": namespace,
            "camera_name": front_camera_name,
            "param_file_name": front_param_file_name,
            "enable_disparity": enable_disparity,
            "enable_pcl": enable_pcl,
            "use_intra_process": use_intra_process,
            "respawn": respawn,
            "respawn_delay": respawn_delay,
        }.items(),
        condition=UnlessCondition(camera_base),
    )

    # --- Front camera base ---
    front_camera_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_base_launch),
        launch_arguments={
            "node_name":       front_camera_name,
            "node_namespace":  namespace,
            "param_file_name": front_param_file_name,
        }.items(),
        condition=IfCondition(camera_base),
    )

    # --- Chin camera container ---
    chin_camera_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_container_launch),
        launch_arguments={
            "namespace": namespace,
            "camera_name": chin_camera_name,
            "param_file_name": chin_param_file_name,
            "enable_disparity": enable_disparity,
            "enable_pcl": enable_pcl,
            "use_intra_process": use_intra_process,
            "respawn": respawn,
            "respawn_delay": respawn_delay,
        }.items(),
        condition=UnlessCondition(camera_base),
    )

    # --- Chin camera base ---
    chin_camera_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_base_launch),
        launch_arguments={
            "node_name":       chin_camera_name,
            "node_namespace":  namespace,
            "param_file_name": chin_param_file_name,
        }.items(),
        condition=IfCondition(camera_base),
    )

    # --- Ultrasound interface ---
    ultrasound_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ultrasound_launch),
        launch_arguments={
            "node_name": "ultrasound_interface",
            "param_file_name": ultrasound_param_file,
        }.items(),
        condition=IfCondition(enable_ultrasound),
    )

    return LaunchDescription([
        *declared_args,
        front_camera_container,
        front_camera_base,
        chin_camera_container,
        chin_camera_base,
        ultrasound_interface,
    ])
