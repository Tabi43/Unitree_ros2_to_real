from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("go1_ros2_real"), "rviz", "go1_real.rviz"]
    )

    RSP_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("go1_ros2_real"), "launch", "go1_description.launch.py"]
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time if true",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="unitree_go1",
                description="Robot namespace (empty for global)",
            ),
            RSP_launch,
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
