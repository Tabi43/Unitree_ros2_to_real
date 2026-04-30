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
        description="Control/publish loop rate in Hz.",
    )
    low_cmd_topic_arg = DeclareLaunchArgument(
        "low_cmd_topic",
        default_value="/unitree_go1/low_cmd",
        description="Low-level command topic.",
    )
    low_state_topic_arg = DeclareLaunchArgument(
        "low_state_topic",
        default_value="/unitree_go1/low_state",
        description="Low-level state topic.",
    )
    wait_for_low_state_arg = DeclareLaunchArgument(
        "wait_for_low_state",
        default_value="true",
        description="If true, wait for first low_state before starting stand-up.",
    )

    stand_and_sit_node = Node(
        package="unitree_python_pkg",
        executable="stand_and_sit_node",
        name="stand_and_sit_node",
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
                "low_cmd_topic": LaunchConfiguration("low_cmd_topic"),
                "low_state_topic": LaunchConfiguration("low_state_topic"),
                "wait_for_low_state": ParameterValue(
                    LaunchConfiguration("wait_for_low_state"), value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            hold_seconds_arg,
            transition_steps_arg,
            publish_rate_arg,
            low_cmd_topic_arg,
            low_state_topic_arg,
            wait_for_low_state_arg,
            stand_and_sit_node,
        ]
    )
