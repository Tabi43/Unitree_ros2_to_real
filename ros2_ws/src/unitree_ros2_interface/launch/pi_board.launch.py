# -*- coding: utf-8 -*-
"""
Template comune per head_board.launch.py / body_board.launch.py / main_board.launch.py

Cambia solo:
- CAMERAS_THIS_BOARD
- (opzionale) quali Node lanciare per ultrasound/face_lights/low/high
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

# Package e launch file “camera_base” da includere (adatta ai tuoi nomi reali)
CAMERA_LAUNCH_PKG = "unitree_ros2_interface"
# ========================================


def generate_launch_description():
    # --- Launch configurations (strings) ---
    board_ip = LaunchConfiguration("board_ip")
    board_role = LaunchConfiguration("board_role")

    enable_camera = LaunchConfiguration("enable_camera")
    enable_ultrasound = LaunchConfiguration("enable_ultrasound")
    enable_face_lights = LaunchConfiguration("enable_face_lights")
    enable_low = LaunchConfiguration("enable_low")
    enable_high = LaunchConfiguration("enable_high")

    publish_rectified = LaunchConfiguration("publish_rectified")
    publish_depth = LaunchConfiguration("publish_depth")
    publish_pcl = LaunchConfiguration("publish_pcl")

    # --- DeclareLaunchArgument: stessi in tutti e tre i file ---
    declared_args = [
        DeclareLaunchArgument(
            "board_ip",
            default_value=TextSubstitution(text=""),
            description="IP della scheda (opzionale; utile per log/parametri).",
        ),
        DeclareLaunchArgument(
            "board_role",
            default_value=TextSubstitution(text=""),
            description="Ruolo della scheda: head|body|main (opzionale).",
        ),

        DeclareLaunchArgument("enable_camera", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("enable_ultrasound", default_value=TextSubstitution(text="false")),
        DeclareLaunchArgument("enable_face_lights", default_value=TextSubstitution(text="false")),
        DeclareLaunchArgument("enable_low", default_value=TextSubstitution(text="false")),
        DeclareLaunchArgument("enable_high", default_value=TextSubstitution(text="true")),

        DeclareLaunchArgument("publish_rectified", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("publish_depth", default_value=TextSubstitution(text="false")),
        DeclareLaunchArgument("publish_pcl", default_value=TextSubstitution(text="true")),
    ]

    actions = []
    actions.extend(declared_args)

    # --- Log utile in avvio ---
    actions.append(
        LogInfo(
            msg=[
                "[board-launch] role=", board_role,
                " ip=", board_ip,
            ]
        )
    )

    # Add some Nodes here if needed

    return LaunchDescription(actions)
