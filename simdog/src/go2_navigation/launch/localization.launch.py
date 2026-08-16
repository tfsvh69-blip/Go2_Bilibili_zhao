#!/usr/bin/env python3
"""Go2 固定地图定位入口：AMCL 默认，NDT/CUDA NDT 为实验档。"""

import os

import launch
import launch.events
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def _map_server_actions(share_dir, map_dir, use_sim_time, manager_name,
                        managed_nodes):
    """启动静态二维地图及其标准 Nav2 lifecycle manager。"""
    amcl_params = os.path.join(share_dir, "config", "localization_amcl.yaml")
    actions = [
        launch_ros.actions.Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                amcl_params,
                {"yaml_filename": os.path.join(map_dir, "map.yaml")},
                {"use_sim_time": use_sim_time},
            ],
        )
    ]
    actions.append(
        launch_ros.actions.Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name=manager_name,
            output="screen",
            parameters=[
                {"autostart": True},
                {"node_names": managed_nodes},
                {"bond_timeout": 10.0},
                {"use_sim_time": use_sim_time},
            ],
        )
    )
    return actions


def _amcl_actions(share_dir, map_dir, use_sim_time):
    params = os.path.join(share_dir, "config", "localization_amcl.yaml")
    actions = [
        launch_ros.actions.Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[params, {"use_sim_time": use_sim_time}],
        )
    ]
    # 名称必须保持 lifecycle_manager_localization，Nav2 RViz 面板通过它查询
    # 固定图定位的真实 active 状态。
    actions += _map_server_actions(
        share_dir, map_dir, use_sim_time,
        "lifecycle_manager_localization", ["map_server", "amcl"])
    return actions


def _lidar_ndt_actions(share_dir, map_dir, use_sim_time):
    """启动 NDT LifecycleNode，并由二维 EKF 唯一发布 map -> odom。"""
    node = launch_ros.actions.LifecycleNode(
        package="lidar_localization_ros2",
        executable="lidar_localization_node",
        name="lidar_localization_node",
        namespace="",
        output="screen",
        parameters=[
            os.path.join(share_dir, "config", "localization_ndt.yaml"),
            {
                "map_path": os.path.join(map_dir, "GlobalMap.pcd"),
                "enable_map_odom_tf": False,
                "global_frame_id": "map",
                "odom_frame_id": "odom",
                "base_frame_id": "base_footprint",
                "use_sim_time": use_sim_time,
            },
        ],
        remappings=[
            ("/cloud", "/velodyne_points"),
            ("initial_map", "/global_map"),
        ],
    )
    configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state="unconfigured",
            entities=[
                LogInfo(msg="-- NDT 定位节点启动，发送 CONFIGURE --"),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="-- NDT 定位节点已 configure，发送 ACTIVATE --"),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    ekf = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="ndt_global_ekf",
        output="screen",
        parameters=[
            os.path.join(share_dir, "config", "localization_ndt_ekf.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("odometry/filtered", "/odometry/global_filtered")],
    )
    actions = [
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )),
        configure,
        activate,
        node,
        ekf,
    ]
    # NDT 不受 Nav2 bond 管理；map_server 仍由独立 manager 激活。
    actions += _map_server_actions(
        share_dir, map_dir, use_sim_time,
        "lifecycle_manager_map", ["map_server"])
    return actions


def _build(context):
    share_dir = FindPackageShare("go2_navigation").find("go2_navigation")
    map_dir = os.path.abspath(os.path.expanduser(
        LaunchConfiguration("map_dir").perform(context)))
    localization = LaunchConfiguration("localization").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in {
        "1", "true", "yes", "on"
    }

    if localization == "amcl":
        return _amcl_actions(share_dir, map_dir, use_sim_time)
    if localization == "lidar_ndt":
        return _lidar_ndt_actions(share_dir, map_dir, use_sim_time)
    if localization == "ndt_cuda":
        return [
            *_map_server_actions(
                share_dir, map_dir, use_sim_time,
                "lifecycle_manager_map", ["map_server"]),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    os.path.join(
                        FindPackageShare("ndt_relocalization").find(
                            "ndt_relocalization"),
                        "launch", "ndt_localization.launch.py")),
                launch_arguments={
                    "map_path": os.path.join(map_dir, "GlobalMap.pcd"),
                    "input_cloud_topic": "/velodyne_points",
                    "registration_backend": "cuda",
                    "use_sim_time": str(use_sim_time).lower(),
                    "use_rviz": "false",
                }.items(),
            ),
        ]
    raise ValueError(
        "未知定位后端：%s（支持 amcl | lidar_ndt | ndt_cuda）" % localization)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "map_dir", default_value=os.path.join(
                os.environ.get("GO2_PROJECT_ROOT", os.path.expanduser("~")),
                "go2_maps/latest"),
            description="同源地图包目录"),
        DeclareLaunchArgument(
            "localization", default_value="amcl",
            description="定位后端：amcl（默认）| lidar_ndt | ndt_cuda"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_build),
    ])
