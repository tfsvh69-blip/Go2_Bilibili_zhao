#!/usr/bin/env python3
"""Slam Toolbox 在线建图入口；/scan 由导航主入口统一提供。"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _build_actions(context):
    share_dir = FindPackageShare("go2_navigation").find("go2_navigation")
    params_file = os.path.join(share_dir, "config", "online_mapping.yaml")
    use_sim_time = _as_bool(
        LaunchConfiguration("use_sim_time").perform(context))
    map_session = LaunchConfiguration("map_session").perform(context).strip()

    slam_overrides = {"use_sim_time": use_sim_time}
    session_text = "新建空白 pose graph"
    if map_session and map_session != "new":
        session_dir = os.path.abspath(os.path.expanduser(map_session))
        graph_base = os.path.join(session_dir, "slam")
        required = [graph_base + ".posegraph", graph_base + ".data"]
        missing = [path for path in required if not os.path.isfile(path)]
        if missing:
            raise RuntimeError(
                "无法续建在线地图，缺少：%s" % "，".join(missing))
        slam_overrides.update({
            "map_file_name": graph_base,
            "map_start_at_dock": True,
        })
        session_text = "续建 %s" % session_dir

    return [
        LogInfo(msg="Go2 在线 SLAM：%s，/scan 由重力对齐的 Velodyne 水平切片生成"
                % session_text),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[params_file, slam_overrides],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "map_session", default_value="new",
            description="new 或包含 slam.posegraph/slam.data 的已保存目录"),
        OpaqueFunction(function=_build_actions),
    ])
