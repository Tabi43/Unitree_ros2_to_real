#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    namespace         = LaunchConfiguration("namespace")
    camera_name       = LaunchConfiguration("camera_name")
    param_file_name   = LaunchConfiguration("param_file_name")

    enable_disparity  = LaunchConfiguration("enable_disparity")
    enable_pcl        = LaunchConfiguration("enable_pcl")
    use_ipc           = LaunchConfiguration("use_intra_process")

    respawn           = LaunchConfiguration("respawn")
    respawn_delay     = LaunchConfiguration("respawn_delay")

    pkg_share = get_package_share_directory("unitree_ros2_interface")
    param_path = PathJoinSubstitution([pkg_share, "config", param_file_name])

    # disparity required if enable_disparity OR enable_pcl
    need_disparity = PythonExpression([
        "('", enable_disparity, "' == 'true') or ('", enable_pcl, "' == 'true')"
    ])

    ipc_extra = [{"use_intra_process_comms": use_ipc}]

    # 1) Start container ALONE (empty), with respawn enabled.
    container = ComposableNodeContainer(
        name=[camera_name, "_container"],
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        emulate_tty=True,
        respawn=respawn,
        respawn_delay=respawn_delay,
        # Keep descriptions empty here for maximum compatibility; we load nodes after.
        composable_node_descriptions=[],
    )

    # target container full name — absolute path, handles empty namespace
    target_container = PythonExpression(
        ["'/' + ('", namespace, "/' if '", namespace, "' else '') + '", camera_name, "_container'"]
    )

    # 2) Nodes to load (always)
    unitree_udp_cam = ComposableNode(
        package="unitree_ros2_interface",
        plugin="unitree_ros2_interface::UnitreeUdpCameraInterface",
        name=[camera_name, "_interface"],
        parameters=[param_path],
        extra_arguments=ipc_extra,
    )

    rectify_left = ComposableNode(
        package="image_proc",
        plugin="image_proc::RectifyNode",
        name="rectify_left",
        namespace=PythonExpression(["'/' + ('", namespace, "/' if '", namespace, "' else '') + '", camera_name, "/left'"]),
        remappings=[("image", "image_mono")],  # out: image_rect
        extra_arguments=ipc_extra,
    )

    rectify_right = ComposableNode(
        package="image_proc",
        plugin="image_proc::RectifyNode",
        name="rectify_right",
        namespace=PythonExpression(["'/' + ('", namespace, "/' if '", namespace, "' else '') + '", camera_name, "/right'"]),
        remappings=[("image", "image_mono")],
        extra_arguments=ipc_extra,
    )

    load_base = LoadComposableNodes(
        target_container=target_container,
        composable_node_descriptions=[unitree_udp_cam, rectify_left, rectify_right],
    )

    # 3) Optional disparity (remap disparity -> disparity_image)
    disparity_node = ComposableNode(
        package="stereo_image_proc",
        plugin="stereo_image_proc::DisparityNode",
        name="disparity",
        namespace=PythonExpression(["'/' + ('", namespace, "/' if '", namespace, "' else '') + '", camera_name, "'"]),
        extra_arguments=ipc_extra,
    )

    load_disparity = LoadComposableNodes(
        target_container=target_container,
        composable_node_descriptions=[disparity_node],
        condition=IfCondition(need_disparity),
    )

    # 4) Optional pointcloud
    # PointCloudNode expects left/image_rect_color in addition to disparity.
    rectify_left_color = ComposableNode(
        package="image_proc",
        plugin="image_proc::RectifyNode",
        name="rectify_left_color",
        namespace=PythonExpression(["'/' + ('", namespace, "/' if '", namespace, "' else '') + '", camera_name, "/left'"]),
        remappings=[
            ("image", "image_raw"),
            ("image_rect", "image_rect_color"),
        ],
        extra_arguments=ipc_extra,
    )

    pointcloud_node = ComposableNode(
        package="stereo_image_proc",
        plugin="stereo_image_proc::PointCloudNode",
        name="points2",
        namespace=PythonExpression(["'/' + ('", namespace, "/' if '", namespace, "' else '') + '", camera_name, "'"]),
        extra_arguments=ipc_extra,
    )

    load_pointcloud = LoadComposableNodes(
        target_container=target_container,
        composable_node_descriptions=[rectify_left_color, pointcloud_node],
        condition=IfCondition(enable_pcl),
    )

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="unitree_go1"),
        DeclareLaunchArgument("camera_name", default_value="front_camera"),
        DeclareLaunchArgument("param_file_name", default_value="stereo_udp_front_camera_config.yaml"),

        DeclareLaunchArgument("enable_disparity", default_value="false"),
        DeclareLaunchArgument("enable_pcl", default_value="false"),
        DeclareLaunchArgument("use_intra_process", default_value="true"),

        DeclareLaunchArgument("respawn", default_value="true"),
        DeclareLaunchArgument("respawn_delay", default_value="2.0"),

        container,
        load_base,
        load_disparity,
        load_pointcloud,
    ])
