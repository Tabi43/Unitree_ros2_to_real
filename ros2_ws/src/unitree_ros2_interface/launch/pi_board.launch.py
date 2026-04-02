#!/usr/bin/env python3
# Pi board (192.168.123.161):
#   - ultrasound (pi variant)
#   - face_lights
#   - chin camera (UDP)

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
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
    ]

    pkg_share = get_package_share_directory("unitree_ros2_interface")
    ultrasound_launch           = os.path.join(pkg_share, "launch", "ultrasound_interface.launch.py")

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

    return LaunchDescription([
        *declared_args,
        ultrasound_interface,
        face_lights_node,
    ])