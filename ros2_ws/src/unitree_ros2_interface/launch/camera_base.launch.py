#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('unitree_ros2_interface')
    
    # Declare launch arguments
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='bottom_camera',
        description='Name of the camera (bottom_camera, front_camera, etc.)'
    )
    
    publish_rectified_arg = DeclareLaunchArgument(
        'publish_rectified',
        default_value='true',
        description='Whether to publish rectified images'
    )
    
    publish_depth_arg = DeclareLaunchArgument(
        'publish_depth',
        default_value='false',
        description='Whether to publish depth images'
    )
    
    publish_pcl_arg = DeclareLaunchArgument(
        'publish_pcl',
        default_value='true',
        description='Whether to publish point cloud'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='unitree_go1',
        description='Namespace for the nodes'
    )
    
    # Get launch configurations
    camera_name = LaunchConfiguration('camera_name')
    publish_rectified = LaunchConfiguration('publish_rectified')
    publish_depth = LaunchConfiguration('publish_depth')
    publish_pcl = LaunchConfiguration('publish_pcl')
    namespace = LaunchConfiguration('namespace')
    
    # Define paths using PathJoinSubstitution
    camera_calib_path = PathJoinSubstitution([pkg_share, 'calibrations'])
    config_file_path = PathJoinSubstitution([
        pkg_share, 'config', 
        ['stereo_', camera_name, '_config.yaml']
    ])
    camera_info_url_left = PathJoinSubstitution([
        camera_calib_path,
        [camera_name, '_left.yaml']
    ])
    camera_info_url_right = PathJoinSubstitution([
        camera_calib_path,
        [camera_name, '_right.yaml']
    ])
    
    # Create the camera node
    camera_node = Node(
        package='unitree_ros2_interface',
        executable='camera_interface_node',
        name=['go1_', camera_name, '_node'],
        output='screen',
        parameters=[{
            'camera_name': camera_name,
            'camera_name_left': [camera_name, '_left'],
            'camera_name_right': [camera_name, '_right'],
            'camera_info_url_left': camera_info_url_left,
            'camera_info_url_right': camera_info_url_right,
            'config_file': config_file_path,
            'publish_rectified': publish_rectified,
            'publish_depth': publish_depth,
            'publish_pcl': publish_pcl,
            'verbose': True
        }]
    )
    
    # Group with namespace
    camera_group = GroupAction([
        PushRosNamespace(namespace),
        camera_node
    ])
    
    return LaunchDescription([
        camera_name_arg,
        publish_rectified_arg,
        publish_depth_arg,
        publish_pcl_arg,
        namespace_arg,
        camera_group
    ])