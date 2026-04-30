#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    hold_seconds_arg = DeclareLaunchArgument(
        "hold_seconds",
        default_value="5.0",
        description="Hold time in seconds after stand-up.",
    )
    transition_steps_arg = DeclareLaunchArgument(
        "transition_steps",
        default_value="1000",
        description="Number of control steps for stand-up and sit-down interpolation.",
    )
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="500.0",
        description="Control loop rate in Hz.",
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.123.10",
        description="Go1 controller IP for direct SDK UDP.",
    )
    local_port_arg = DeclareLaunchArgument(
        "local_port",
        default_value="8080",
        description="Local UDP port used by the SDK client.",
    )
    robot_port_arg = DeclareLaunchArgument(
        "robot_port",
        default_value="8007",
        description="Robot UDP port used by low-level SDK.",
    )
    safety_level_arg = DeclareLaunchArgument(
        "safety_level",
        default_value="1",
        description="Safety.PowerProtect level (0 disables protection).",
    )

    stand_and_sit_sdk_node = Node(
        package="unitree_python_pkg",
        executable="stand_and_sit_sdk_node",
        name="stand_and_sit_sdk_node",
        output="screen",
        parameters=[
            {
                "hold_seconds": ParameterValue(
                    LaunchConfiguration("hold_seconds"), value_type=float
                ),
                "transition_steps": ParameterValue(
                    LaunchConfiguration("transition_steps"), value_type=int
                ),
                "publish_rate": ParameterValue(
                    LaunchConfiguration("publish_rate"), value_type=float
                ),
                "robot_ip": LaunchConfiguration("robot_ip"),
                "local_port": ParameterValue(
                    LaunchConfiguration("local_port"), value_type=int
                ),
                "robot_port": ParameterValue(
                    LaunchConfiguration("robot_port"), value_type=int
                ),
                "safety_level": ParameterValue(
                    LaunchConfiguration("safety_level"), value_type=int
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            hold_seconds_arg,
            transition_steps_arg,
            publish_rate_arg,
            robot_ip_arg,
            local_port_arg,
            robot_port_arg,
            safety_level_arg,
            stand_and_sit_sdk_node,
        ]
    )
