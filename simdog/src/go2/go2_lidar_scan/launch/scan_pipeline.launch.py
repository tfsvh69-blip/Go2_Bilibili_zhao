#!/usr/bin/env python3
"""启动唯一的 /velodyne_points -> /scan 转换器及可选诊断节点。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share_dir = get_package_share_directory("go2_lidar_scan")
    default_params = os.path.join(share_dir, "config", "vlp16_scan.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    cloud_topic = LaunchConfiguration("cloud_topic")
    cloud_ready_topic = LaunchConfiguration("cloud_ready_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    raw_scan_topic = LaunchConfiguration("raw_scan_topic")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("diagnostics", default_value="true"),
        DeclareLaunchArgument("lidar_debug_raw_scan", default_value="false"),
        DeclareLaunchArgument("cloud_topic", default_value="/velodyne_points"),
        DeclareLaunchArgument(
            "cloud_ready_topic",
            default_value="/go2_lidar_scan/leveled_cloud"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("raw_scan_topic", default_value="/scan_raw"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        LogInfo(msg=[
            "Go2 LiDAR 重力对齐转换：", cloud_topic, " -> ", scan_topic,
            "；参数=", params_file,
        ]),
        # 重力对齐器先发布本帧 TF，再把原点云交给同一进程内的上游转换组件。
        # 这消除了两个独立大点云订阅者以及 cloud/TF 的跨进程竞速。
        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container",
            name="go2_lidar_scan_container",
            namespace="",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="go2_lidar_scan",
                    plugin="go2_lidar_scan::LevelFramePublisher",
                    name="go2_lidar_level_frame",
                    parameters=[
                        params_file,
                        {
                            "use_sim_time": use_sim_time,
                            "cloud_topic": cloud_topic,
                            "cloud_ready_topic": cloud_ready_topic,
                        },
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                ComposableNode(
                    package="pointcloud_to_laserscan",
                    plugin=(
                        "pointcloud_to_laserscan::"
                        "PointCloudToLaserScanNode"
                    ),
                    name="go2_lidar_scan_converter",
                    parameters=[params_file, {"use_sim_time": use_sim_time}],
                    remappings=[
                        ("cloud_in", cloud_ready_topic),
                        ("scan", scan_topic),
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="go2_lidar_scan_raw_converter",
            output="screen",
            condition=IfCondition(LaunchConfiguration("lidar_debug_raw_scan")),
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=[("cloud_in", cloud_topic), ("scan", raw_scan_topic)],
        ),
        Node(
            package="go2_lidar_scan",
            executable="scan_diagnostics",
            name="go2_lidar_scan_diagnostics",
            output="screen",
            condition=IfCondition(LaunchConfiguration("diagnostics")),
            parameters=[
                params_file,
                {
                    "use_sim_time": use_sim_time,
                    "cloud_topic": cloud_topic,
                    "scan_topic": scan_topic,
                },
            ],
        ),
    ])
