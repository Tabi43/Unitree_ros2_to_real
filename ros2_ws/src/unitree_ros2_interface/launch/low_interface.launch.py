#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unitree_ros2_interface',
            executable='low_interface_node',
            name='low_interface_node',
            output='screen'
        )
    ])