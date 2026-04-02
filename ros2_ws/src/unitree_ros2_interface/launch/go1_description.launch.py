# launch/go1_description.launch.py
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('unitree_ros2_interface')
    default_go1_urdf = os.path.join(pkg_share, 'urdf', 'go1.urdf')

    # --- Launch arguments ---
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='unitree_go1',
        description='Robot namespace.',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true.',
    )
    frame_prefix_arg = DeclareLaunchArgument(
        'frame_prefix',
        default_value='',
        description='TF frame prefix (e.g. "go1/").',
    )
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=default_go1_urdf,
        description='Absolute path to the robot URDF file.',
    )

    namespace       = LaunchConfiguration('namespace')
    use_sim_time    = LaunchConfiguration('use_sim_time')
    frame_prefix    = LaunchConfiguration('frame_prefix')
    urdf_path       = LaunchConfiguration('urdf_path')

    robot_description = ParameterValue(
        Command(['cat ', urdf_path]),
        value_type=str,
    )

    # --- RSP node ---
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
            'frame_prefix': frame_prefix,
        }],
        output='screen',
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        frame_prefix_arg,
        urdf_path_arg,
        rsp_node,
    ])