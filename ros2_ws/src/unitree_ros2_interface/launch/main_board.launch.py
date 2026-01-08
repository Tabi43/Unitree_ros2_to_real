# main board handle low/high interface and bottom_camera
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


# ========= PERSONALIZZA PER FILE =========
# Esempi:
# head_board.launch.py  -> ["front_camera", "chin_camera"]
# body_board.launch.py  -> ["left_camera", "right_camera"]
# main_board.launch.py  -> ["bottom_camera"]
CAMERAS_THIS_BOARD = [
    "bottom_camera",
]

# Package e launch file “camera_base” da includere (adatta ai tuoi nomi reali)
CAMERA_LAUNCH_PKG = "unitree_ros2_interface"
CAMERA_LAUNCH_FILE = "camera_base.launch.py"
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
                " cams=", TextSubstitution(text=",".join(CAMERAS_THIS_BOARD) if CAMERAS_THIS_BOARD else "(none)"),
            ]
        )
    )

    # --- Cameras: include per ciascuna camera (abilitato da enable_camera) ---
    camera_pkg_share = get_package_share_directory(CAMERA_LAUNCH_PKG)
    camera_launch_path = os.path.join(camera_pkg_share, "launch", CAMERA_LAUNCH_FILE)

    camera_includes = []
    for cam in CAMERAS_THIS_BOARD:
        camera_includes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch_path),
                launch_arguments={
                    "camera_name": cam,
                    "publish_rectified": publish_rectified,
                    "publish_depth": publish_depth,
                    "publish_pcl": publish_pcl,
                }.items(),
                condition=IfCondition(enable_camera),
            )
        )

    actions.append(
        GroupAction(camera_includes)
    )

    # --- Ultrasound (placeholder: aggiorna package/executable/parametri reali) ---
    actions.append(
        Node(
            package="unitree_ros2_interface",
            executable="ultrasound_node",
            name="ultrasound",
            output="screen",
            parameters=[{
                "board_ip": board_ip,
                "board_role": board_role,
            }],
            condition=IfCondition(enable_ultrasound),
        )
    )

    # --- Low interface (placeholder) ---
    actions.append(
        Node(
            package="unitree_ros2_interface",
            executable="low_interface_node",
            name="low_interface",
            output="screen",
            parameters=[{
                "board_ip": board_ip,
                "board_role": board_role,
            }],
            condition=IfCondition(enable_low),
        )
    )

    # --- High interface (placeholder) ---
    actions.append(
        Node(
            package="unitree_ros2_interface",
            executable="high_interface_node",
            name="high_interface",
            output="screen",
            parameters=[{
                "board_ip": board_ip,
                "board_role": board_role,
            }],
            condition=IfCondition(enable_high),
        )
    )

    return LaunchDescription(actions)
