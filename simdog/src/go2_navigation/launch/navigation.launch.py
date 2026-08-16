#!/usr/bin/env python3
"""Go2 自主导航主入口。

固定地图流程：二维地图校验 -> AMCL（默认）/NDT -> Nav2 -> 安全控制链。
在线建图流程：pointcloud_to_laserscan -> slam_toolbox -> Nav2 -> 安全控制链。
（twist_mux -> velocity_smoother -> collision_monitor -> /cmd_vel）-> RViz。

控制链话题：
    Nav2 /cmd_vel_nav（+ 键盘 /cmd_vel_teleop、Unitree Move /cmd_vel_unitree）
        -> twist_mux /cmd_vel_switched
        -> velocity_smoother /cmd_vel_smoothed
        -> collision_monitor /cmd_vel
        -> CHAMP（quadruped_controller_node）

用法：
    ros2 launch go2_navigation navigation.launch.py \
        navigation_mode:=static_map map_dir:=$HOME/go2_maps/latest
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from go2_navigation.map_utils import MapValidationError, load_static_map
from go2_navigation.validate_map_bundle import validate as _validate_map_bundle

# Nav2 节点：(节点名, 包, 可执行, 话题 remap)
NAV2_NODES = [
    ("controller_server", "nav2_controller", "controller_server",
     [("/cmd_vel", "/cmd_vel_nav")]),
    ("smoother_server", "nav2_smoother", "smoother_server", []),
    ("planner_server", "nav2_planner", "planner_server", []),
    ("behavior_server", "nav2_behaviors", "behavior_server",
     [("/cmd_vel", "/cmd_vel_nav")]),
    # Humble action 需重映射 3 个服务与 2 个话题端点；只重映射 action 根名
    # 不会生效。该组合已在隔离 ROS_DOMAIN_ID 下验证为唯一 raw 服务端。
    ("bt_navigator", "nav2_bt_navigator", "bt_navigator",
     [
         ("/navigate_to_pose/_action/send_goal",
          "/navigate_to_pose_raw/_action/send_goal"),
         ("/navigate_to_pose/_action/get_result",
          "/navigate_to_pose_raw/_action/get_result"),
         ("/navigate_to_pose/_action/cancel_goal",
          "/navigate_to_pose_raw/_action/cancel_goal"),
         ("/navigate_to_pose/_action/feedback",
          "/navigate_to_pose_raw/_action/feedback"),
         ("/navigate_to_pose/_action/status",
          "/navigate_to_pose_raw/_action/status"),
     ]),
    ("waypoint_follower", "nav2_waypoint_follower", "waypoint_follower", []),
    # velocity_smoother：输入 twist_mux 输出，输出给 collision_monitor
    ("velocity_smoother", "nav2_velocity_smoother", "velocity_smoother",
     [("cmd_vel", "/cmd_vel_switched"),
      ("cmd_vel_smoothed", "/cmd_vel_smoothed")]),
    ("collision_monitor", "nav2_collision_monitor", "collision_monitor", []),
]

# map_server/AMCL 由 localization.launch.py 的独立 lifecycle manager 管理。
LIFECYCLE_NODES = ["controller_server", "smoother_server",
                   "planner_server", "behavior_server", "bt_navigator",
                   "waypoint_follower", "velocity_smoother",
                   "collision_monitor"]

CONTROLLER_PROFILES = {
    "forward_rpp": "controller_forward_rpp.yaml",
    "forward_mppi": "controller_forward_mppi.yaml",
    "omni_mppi": "controller_omni_mppi.yaml",
}
NAVIGATION_MODES = {"static_map", "online_slam"}


def _as_bool(value: str) -> bool:
    """兼容 XML 嵌套 launch 传入的 ``True`` 与命令行传入的 ``true``。"""
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _build_actions(context):
    """运行时构建：校验地图包并计算路径，返回全部节点动作。"""
    share_dir = FindPackageShare("go2_navigation").find("go2_navigation")
    map_dir = os.path.abspath(
        LaunchConfiguration("map_dir").perform(context))
    map_session = LaunchConfiguration("map_session").perform(context)
    use_sim_time = _as_bool(
        LaunchConfiguration("use_sim_time").perform(context))
    localization = LaunchConfiguration("localization").perform(context)
    navigation_mode = LaunchConfiguration("navigation_mode").perform(context)
    deprecated_mode = navigation_mode == "static_bundle"
    if deprecated_mode:
        navigation_mode = "static_map"
    controller_profile = LaunchConfiguration("controller_profile").perform(context)
    rviz_on = _as_bool(LaunchConfiguration("rviz").perform(context))
    tuning_gui_on = _as_bool(
        LaunchConfiguration("tuning_gui").perform(context))
    health_on = _as_bool(
        LaunchConfiguration("health_check").perform(context))
    minimum_goal_clearance_m = float(
        LaunchConfiguration("minimum_goal_clearance_m").perform(context))

    if navigation_mode not in NAVIGATION_MODES:
        raise RuntimeError(
            "navigation_mode 必须是 static_map 或 online_slam，实际为 %s"
            % navigation_mode)
    if controller_profile not in CONTROLLER_PROFILES:
        raise RuntimeError(
            "controller_profile 必须是 %s，实际为 %s"
            % (" | ".join(CONTROLLER_PROFILES), controller_profile))

    # AMCL 只消费 Slam Toolbox 保存的二维 PGM/YAML，不应强制要求无关的 3D PCD。
    # NDT 实验档仍要求 PCD 与二维图同源并通过 SHA-256 清单校验。
    if navigation_mode == "static_map":
        if localization == "amcl":
            try:
                load_static_map(map_dir)
            except MapValidationError as exc:
                raise RuntimeError(
                    "AMCL 二维地图校验失败，拒绝启动导航：%s\n"
                    "请先在 online_slam 模式建图并执行 save_online_map.sh。"
                    % exc) from exc
        else:
            ok, problems = _validate_map_bundle(map_dir)
            if not ok:
                lines = "\n".join("  - " + p for p in problems)
                raise RuntimeError(
                    "NDT 同源地图包校验失败，拒绝启动导航：\n%s\n"
                    "请先执行 ros2 run go2_navigation build_map_bundle --map-dir %s"
                    % (lines, map_dir))

    params_file = os.path.join(share_dir, "config", "navigation.yaml")
    controller_params = os.path.join(
        share_dir, "config", CONTROLLER_PROFILES[controller_profile])
    twist_mux_params = os.path.join(share_dir, "config", "twist_mux.yaml")
    rviz_name = (
        "online_mapping_navigation.rviz"
        if navigation_mode == "online_slam" else "navigation.rviz"
    )
    rviz_config = os.path.join(share_dir, "rviz", rviz_name)
    navigation_bt = os.path.join(
        share_dir, "behavior_trees", "go2_navigate_to_pose.xml")
    localization_launch = os.path.join(share_dir, "launch",
                                       "localization.launch.py")
    online_slam_launch = os.path.join(
        share_dir, "launch", "online_slam.launch.py")

    actions = [
        LogInfo(msg=(
            "Go2 导航参数：ROS_DOMAIN_ID=%s，mode=%s，controller=%s，"
            "map_dir=%s，localization=%s，use_sim_time=%s，rviz=%s，"
            "tuning_gui=%s，"
            "health_check=%s，目标余量=%.2f m"
            % (os.environ.get("ROS_DOMAIN_ID", "0"), navigation_mode,
               controller_profile, map_dir, localization, use_sim_time,
               rviz_on, tuning_gui_on, health_on, minimum_goal_clearance_m)))
    ]
    if deprecated_mode:
        actions.append(LogInfo(msg=(
            "[弃用提示] navigation_mode:=static_bundle 已按 static_map 处理；"
            "请更新启动命令。")))

    # 两种模式共用唯一 /scan：VLP-16 水平切片，不重复启动第二套 Gazebo 2D 雷达。
    actions.append(
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            output="screen",
            parameters=[
                os.path.join(share_dir, "config", "online_mapping.yaml"),
                {"use_sim_time": use_sim_time},
            ],
            remappings=[("cloud_in", "/velodyne_points"), ("scan", "/scan")],
        ))

    if navigation_mode == "online_slam":
        actions.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(online_slam_launch),
                launch_arguments={
                    "use_sim_time": str(use_sim_time).lower(),
                    "map_session": map_session,
                }.items(),
            ))
    else:
        actions.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(localization_launch),
                launch_arguments={
                    "map_dir": map_dir,
                    "localization": localization,
                    "use_sim_time": str(use_sim_time).lower(),
                }.items(),
            ))

    # Nav2 规划/控制栈
    for name, pkg, exe, remaps in NAV2_NODES:
        node_parameters = [params_file, controller_params,
                           {"use_sim_time": use_sim_time}]
        if name == "bt_navigator":
            # 多个 goal checker 时，Humble 的默认树会让 FollowPath 发送空 ID。
            node_parameters.append({"default_nav_to_pose_bt_xml": navigation_bt})
        actions.append(
            Node(package=pkg, executable=exe, name=name, output="screen",
                 parameters=node_parameters,
                 remappings=remaps))

    lifecycle_nodes = list(LIFECYCLE_NODES)

    # 统一管理规划/控制 lifecycle；定位节点由各自入口管理。
    actions.append(
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"autostart": True},
                         {"node_names": lifecycle_nodes},
                         {"bond_timeout": 10.0},
                         {"use_sim_time": use_sim_time}]))

    # 安全控制链：速度多路复用
    actions.append(
        Node(package="twist_mux", executable="twist_mux", name="twist_mux",
             output="screen",
             parameters=[twist_mux_params],
             remappings=[("cmd_vel_out", "/cmd_vel_switched")]))

    # 启动早期安全监督保持锁定；定位、Nav2 与地图都健康后自动解锁。
    actions.append(
        Node(package="go2_navigation", executable="goal_guard",
             name="go2_goal_guard", output="screen",
             parameters=[{"map_dir": map_dir,
                          "navigation_mode": navigation_mode,
                          "localization": localization,
                          "minimum_clearance_m": minimum_goal_clearance_m,
                          "use_sim_time": use_sim_time}],
             ))
    actions.append(
        Node(package="go2_navigation", executable="safety_supervisor",
             name="go2_navigation_safety_supervisor", output="screen",
             parameters=[{"navigation_mode": navigation_mode,
                          "localization": localization,
                          "use_sim_time": use_sim_time}],
             ))

    # RViz（由导航入口统一启动）
    if rviz_on:
        actions.append(
            Node(package="rviz2", executable="rviz2", name="rviz2",
                 output="screen", arguments=["-d", rviz_config],
                 parameters=[{"use_sim_time": use_sim_time}]))

    # 可选的标准 ROS 2 动态参数窗口。这里复用 rqt_reconfigure，
    # 不维护自定义参数协议或另一套 Qt 面板。
    if tuning_gui_on:
        actions.append(
            Node(
                package="rqt_gui",
                executable="rqt_gui",
                name="go2_navigation_tuning",
                output="screen",
                arguments=[
                    "--standalone",
                    "rqt_reconfigure.param_plugin.ParamPlugin",
                ],
            ))

    # 可选健康检查
    if health_on:
        actions.append(
            Node(package="go2_navigation", executable="health_check",
                 name="go2_health_check", output="screen",
                 arguments=["--mode", navigation_mode,
                            "--localization", localization,
                            "--map-dir", map_dir],
                 parameters=[{"use_sim_time": use_sim_time}]))

    return actions


def generate_launch_description():
    current_domain = os.environ.get("ROS_DOMAIN_ID", "0")
    return LaunchDescription([
        LogInfo(msg="Go2 导航启动：ROS_DOMAIN_ID=%s（普通仿真期望 0）" % current_domain),
        DeclareLaunchArgument(
            "map_dir",
            default_value=os.path.expanduser("~/go2_maps/latest"),
            description=("固定地图目录：AMCL 需 map.yaml/pgm；NDT 需额外包含 "
                         "GlobalMap.pcd 与 map_bundle.yaml")),
        DeclareLaunchArgument(
            "localization", default_value="amcl",
            description="固定图定位：amcl（默认）| lidar_ndt | ndt_cuda"),
        DeclareLaunchArgument(
            "navigation_mode", default_value="online_slam",
            description="地图/定位模式：online_slam（默认）| static_map；static_bundle 为弃用别名"),
        DeclareLaunchArgument(
            "map_session", default_value="new",
            description="在线模式：new 或包含 slam.posegraph/slam.data 的会话目录"),
        DeclareLaunchArgument(
            "controller_profile", default_value="forward_mppi",
            description=("控制档：forward_mppi（默认）| forward_rpp "
                         "| omni_mppi（对照）")),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "tuning_gui", default_value="false",
            description="启动标准 rqt_reconfigure 动态参数窗口"),
        DeclareLaunchArgument(
            "health_check", default_value="false",
            description="启动后运行健康检查节点"),
        DeclareLaunchArgument(
            "minimum_goal_clearance_m", default_value="0.10",
            description="联调阶段目标/起点到二维障碍和地图边界的最小余量（米）"),
        OpaqueFunction(function=_build_actions),
    ])
