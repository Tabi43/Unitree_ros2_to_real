#!/usr/bin/env python3
# Body board (192.168.123.14):
#   - left_camera  (USB stereo)
#   - right_camera (USB stereo)
#   - chin_camera  (UDP)

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

    # Left camera
    left_camera_name       = LaunchConfiguration("left_camera_name")
    left_param_file_name   = LaunchConfiguration("left_param_file_name")

    # Right camera
    right_camera_name      = LaunchConfiguration("right_camera_name")
    right_param_file_name  = LaunchConfiguration("right_param_file_name")

    # Chin camera (UDP)
    enable_chin_camera     = LaunchConfiguration("enable_chin_camera")
    chin_camera_name       = LaunchConfiguration("chin_camera_name")
    chin_camera_param_file = LaunchConfiguration("chin_camera_param_file")

    # Common camera parameters
    enable_disparity  = LaunchConfiguration("enable_disparity")
    enable_pcl        = LaunchConfiguration("enable_pcl")
    use_intra_process = LaunchConfiguration("use_intra_process")
    respawn           = LaunchConfiguration("respawn")
    respawn_delay     = LaunchConfiguration("respawn_delay")

    # Use camera_base instead of container
    camera_base = LaunchConfiguration("camera_base")

    # --- Declare launch arguments ---
    declared_args = [
        DeclareLaunchArgument("namespace", default_value="unitree_go1"),

        # Left camera
        DeclareLaunchArgument("left_camera_name",     default_value="left_camera"),
        DeclareLaunchArgument("left_param_file_name", default_value="stereo_left_camera_config.yaml"),

        # Right camera
        DeclareLaunchArgument("right_camera_name",     default_value="right_camera"),
        DeclareLaunchArgument("right_param_file_name", default_value="stereo_right_camera_config.yaml"),

        # Chin camera (UDP)
        DeclareLaunchArgument("enable_chin_camera",     default_value="true"),
        DeclareLaunchArgument("chin_camera_name",       default_value="chin_camera"),
        DeclareLaunchArgument("chin_camera_param_file", default_value="stereo_udp_chin_camera_config.yaml"),

        # Common camera parameters
        DeclareLaunchArgument("enable_disparity",  default_value="false"),
        DeclareLaunchArgument("enable_pcl",        default_value="true"),
        DeclareLaunchArgument("use_intra_process", default_value="true"),
        DeclareLaunchArgument("respawn",           default_value="true"),
        DeclareLaunchArgument("respawn_delay",     default_value="5.0"),

        # Use camera_base instead of container
        DeclareLaunchArgument("camera_base", default_value="false",
                              description="If true, use camera_base.launch.py/camera_udp_base.launch.py instead of container."),
    ]

    pkg_share = get_package_share_directory("unitree_ros2_interface")
    camera_container_launch     = os.path.join(pkg_share, "launch", "camera_container.launch.py")
    camera_udp_container_launch = os.path.join(pkg_share, "launch", "camera_udp_container.launch.py")
    camera_base_launch          = os.path.join(pkg_share, "launch", "camera_base.launch.py")
    camera_udp_base_launch      = os.path.join(pkg_share, "launch", "camera_udp_base.launch.py")

    # --- Left camera container ---
    left_camera_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_container_launch),
        launch_arguments={
            "namespace":        namespace,
            "camera_name":      left_camera_name,
            "param_file_name":  left_param_file_name,
            "enable_disparity": enable_disparity,
            "enable_pcl":       enable_pcl,
            "use_intra_process": use_intra_process,
            "respawn":          respawn,
            "respawn_delay":    respawn_delay,
        }.items(),
        condition=UnlessCondition(camera_base),
    )

    # --- Left camera base ---
    left_camera_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_base_launch),
        launch_arguments={
            "node_name":       left_camera_name,
            "namespace":       namespace,
            "param_file_name": left_param_file_name,
        }.items(),
        condition=IfCondition(camera_base),
    )

    # --- Right camera container ---
    right_camera_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_container_launch),
        launch_arguments={
            "namespace":        namespace,
            "camera_name":      right_camera_name,
            "param_file_name":  right_param_file_name,
            "enable_disparity": enable_disparity,
            "enable_pcl":       enable_pcl,
            "use_intra_process": use_intra_process,
            "respawn":          respawn,
            "respawn_delay":    respawn_delay,
        }.items(),
        condition=UnlessCondition(camera_base),
    )

    # --- Right camera base ---
    right_camera_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_base_launch),
        launch_arguments={
            "node_name":       right_camera_name,
            "namespace":       namespace,
            "param_file_name": right_param_file_name,
        }.items(),
        condition=IfCondition(camera_base),
    )

    # --- Chin camera (UDP) container ---
    chin_camera_udp_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_udp_container_launch),
        launch_arguments={
            "namespace":         namespace,
            "camera_name":       chin_camera_name,
            "param_file_name":   chin_camera_param_file,
            "enable_disparity":  enable_disparity,
            "enable_pcl":        enable_pcl,
            "use_intra_process": use_intra_process,
            "respawn":           respawn,
            "respawn_delay":     respawn_delay,
        }.items(),
        condition=UnlessCondition(enable_chin_camera),
    )

    # --- Chin camera base (UDP) ---
    chin_camera_udp_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_udp_base_launch),
        launch_arguments={
            "node_name":       chin_camera_name,
            "namespace":       namespace,
            "param_file_name": chin_camera_param_file,
        }.items(),
        condition=IfCondition(enable_chin_camera),
    )

    # --- RSP ---
    """
        We need to inlcude the launchfile that launches the RSP node:
        - There will be the robot_description publisher
        - The RSP node will be used to publish the transforms
    """
    RSP_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "go1_description.launch.py")),
        launch_arguments={
            "namespace": namespace,
            "frame_prefix": "unitree_go1",
        }.items(),
    )

    return LaunchDescription([
        *declared_args,
        left_camera_container,
        left_camera_base,
        right_camera_container,
        right_camera_base,
        chin_camera_udp_container,
        chin_camera_udp_base,
        RSP_launch,
    ])