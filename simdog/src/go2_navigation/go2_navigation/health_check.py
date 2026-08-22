#!/usr/bin/env python3
"""Go2 导航链路健康检查：话题、定位、生命周期、动作与最终速度出口。"""

from __future__ import annotations

import argparse
from collections import Counter
import os
import time

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener

from go2_navigation.map_utils import (
    COMMISSIONING_CLEARANCE_M,
    MapValidationError,
    StaticMap,
    default_map_dir,
    load_static_map,
)
from go2_navigation.localization_health import amcl_covariance_problem
from go2_navigation.rviz_config import validate_navigation_rviz


COMMON_TOPICS = [
    ("/velodyne_points", PointCloud2),
    ("/scan", LaserScan),
    ("/odom", Odometry),
    ("/joint_states", JointState),
    ("/imu/data", Imu),
]
NDT_TOPICS = COMMON_TOPICS + [
    ("/pcl_pose", PoseWithCovarianceStamped),
    ("/alignment_status", DiagnosticArray),
]
AMCL_TOPICS = COMMON_TOPICS + [
    ("/amcl_pose", PoseWithCovarianceStamped),
]
CUDA_NDT_TOPICS = COMMON_TOPICS + [
    ("/ndt_pose", PoseStamped),
]
ONLINE_TOPICS = COMMON_TOPICS + [
    ("/map", OccupancyGrid),
]
NAVIGATION_LIFECYCLE_NODES = [
    "controller_server", "smoother_server",
    "planner_server", "behavior_server", "bt_navigator", "waypoint_follower",
    "velocity_smoother", "collision_monitor",
]
COMMON_CRITICAL_NODE_NAMES = [
    "go2_goal_guard",
    "go2_navigation_safety_supervisor",
    "twist_mux",
    "go2_lidar_level_frame",
    "go2_lidar_scan_converter",
]
ACTION_SERVERS = {
    "/navigate_to_pose": "go2_goal_guard",
    "/navigate_to_pose_raw": "bt_navigator",
}
TF_CHAIN = [("map", "odom"), ("odom", "base_footprint"),
            ("base_footprint", "base_link"),
            ("base_footprint", "velodyne_level")]


class HealthCheck(Node):
    """在有限墙钟时间内采样并返回导航可执行性诊断。"""

    def __init__(self, map_dir: str, expected_domain_id: int, msg_timeout: float,
                 mode: str = "online_slam", localization: str = "amcl") -> None:
        super().__init__("go2_health_check")
        mode = "static_map" if mode == "static_bundle" else mode
        if mode not in {"static_map", "online_slam"}:
            raise ValueError("未知健康检查模式：%s" % mode)
        self._mode = mode
        self._localization = localization
        self._expected_domain_id = expected_domain_id
        self._msg_timeout = msg_timeout
        if mode == "online_slam":
            self._topics = ONLINE_TOPICS
            self._lifecycle_nodes = list(NAVIGATION_LIFECYCLE_NODES)
        elif localization == "amcl":
            self._topics = AMCL_TOPICS
            self._lifecycle_nodes = [
                *NAVIGATION_LIFECYCLE_NODES, "map_server", "amcl"]
        elif localization == "lidar_ndt":
            self._topics = NDT_TOPICS
            self._lifecycle_nodes = [
                *NAVIGATION_LIFECYCLE_NODES, "map_server",
                "lidar_localization_node"]
        else:
            self._topics = CUDA_NDT_TOPICS
            self._lifecycle_nodes = [
                *NAVIGATION_LIFECYCLE_NODES, "map_server"]
        self._critical_node_names = [
            *self._lifecycle_nodes,
            *COMMON_CRITICAL_NODE_NAMES,
        ]
        if mode == "online_slam":
            self._critical_node_names.append("slam_toolbox")
        elif localization == "lidar_ndt":
            self._critical_node_names.extend(
                ["lidar_localization_node", "ndt_global_ekf", "map_server"])
        elif localization == "amcl":
            self._critical_node_names.extend(["amcl", "map_server"])
        self._last_message = {
            name: None for name, _message_type in self._topics}
        self._latest_pose: PoseWithCovarianceStamped | None = None
        self._alignment_healthy = False
        # 不得覆盖 rclpy.Node 的内部 _subscriptions 容器，否则销毁节点会崩溃。
        self._topic_subscriptions = []
        for name, message_type in self._topics:
            qos = self._qos_for(message_type)
            if name == "/amcl_pose":
                # AMCL 以 transient_local 发布最后位姿，健康检查可能在初始位姿之后启动。
                qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
            self._topic_subscriptions.append(self.create_subscription(
                message_type, name, self._make_topic_callback(name),
                qos))
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._lifecycle_clients = {
            name: self.create_client(GetState, "/%s/get_state" % name)
            for name in self._lifecycle_nodes
        }
        self._controller_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._navigation_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._map: StaticMap | None = None
        self._map_error = None
        if mode == "static_map":
            try:
                self._map = load_static_map(map_dir)
            except MapValidationError as exc:
                self._map_error = str(exc)

    @staticmethod
    def _qos_for(message_type):
        if message_type in (PointCloud2, Imu, JointState, Odometry, LaserScan):
            return QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        return QoSProfile(depth=10)

    def _make_topic_callback(self, topic: str):
        def callback(message) -> None:
            self._last_message[topic] = time.monotonic()
            if topic in {"/pcl_pose", "/amcl_pose", "/ndt_pose"}:
                self._latest_pose = message
            elif topic == "/alignment_status":
                self._alignment_healthy = self._is_alignment_healthy(message)
        return callback

    @staticmethod
    def _is_alignment_healthy(message: DiagnosticArray) -> bool:
        for status in message.status:
            if status.name != "lidar_localization_ros2/alignment":
                continue
            values = {item.key: item.value for item in status.values}
            return (
                status.message == "ok"
                and values.get("has_converged") == "true"
                and values.get("failure_category") == "healthy"
            )
        return False

    def sample(self, duration_sec: float) -> None:
        """使用墙钟限制采样时长，避免 /clock 暂停造成无限等待。"""
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _check_domain(self, problems: list[str]) -> None:
        actual = os.environ.get("ROS_DOMAIN_ID", "0")
        if actual != str(self._expected_domain_id):
            problems.append(
                "ROS_DOMAIN_ID=%s，期望 %d；请 source scripts/setup_unitree_sim.bash"
                % (actual, self._expected_domain_id)
            )

    def _check_topics(self, problems: list[str]) -> None:
        now = time.monotonic()
        for name, last_time in self._last_message.items():
            if last_time is None:
                problems.append("话题 %s 从未收到消息" % name)
            elif name == "/amcl_pose":
                # AMCL 静止时不需要周期发布；transient_local 的最后位姿加 TF 即可判活。
                continue
            elif now - last_time > self._msg_timeout:
                problems.append("话题 %s 已过期 %.1f s" % (name, now - last_time))

    def _check_map_topics(self, problems: list[str]) -> None:
        """确认 RViz 依赖的二维地图和定位点云都已有实际发布者。"""
        expected = [("/map", "在线 SLAM 地图")]
        if self._mode == "static_map":
            expected = [("/map", "静态地图")]
            if self._localization != "amcl":
                expected.append(("/global_map", "定位点云地图"))
        for topic, description in expected:
            if not self.get_publishers_info_by_topic(topic):
                problems.append("%s 无发布者：%s" % (description, topic))

    def _check_rviz_config(self, problems: list[str]) -> None:
        """在不依赖 RViz 是否已打开的情况下校验其坐标系和用户入口。"""
        try:
            config_path = os.path.join(
                get_package_share_directory("go2_navigation"),
                "rviz", (
                    "navigation.rviz" if self._mode == "static_map"
                    else "online_mapping_navigation.rviz"),
            )
        except (LookupError, OSError) as exc:
            problems.append("找不到 navigation.rviz：%s" % exc)
            return
        problems.extend(validate_navigation_rviz(config_path))

    def _check_tf(self, problems: list[str]) -> None:
        for target, source in TF_CHAIN:
            try:
                self._tf_buffer.lookup_transform(target, source, rclpy.time.Time())
            except Exception as exc:  # noqa: BLE001 - TF 错误类型由运行时决定
                problems.append("TF 缺失 %s -> %s：%s" % (target, source, exc))

    def _check_lifecycle(self, problems: list[str]) -> None:
        for name, client in self._lifecycle_clients.items():
            if not client.wait_for_service(timeout_sec=0.5):
                problems.append("生命周期服务不可用：/%s/get_state" % name)
                continue
            future = client.call_async(GetState.Request())
            # CycloneDDS 在 lo 单播下的服务发现和首个请求可能需要数秒；使用墙钟
            # 上限，既避免误报也不受仿真 /clock 暂停影响。
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if not future.done() or future.result() is None:
                problems.append("生命周期状态查询超时：%s" % name)
                continue
            state = future.result().current_state
            if state.id != 3:
                problems.append("生命周期节点 %s 不是 active：%s [%d]" % (
                    name, state.label, state.id))

    def _check_duplicate_nodes(self, problems: list[str]) -> None:
        """发现重复启动的关键节点，避免两个导航栈争用 action、TF 或速度出口。"""
        counts = Counter(self.get_node_names_and_namespaces())
        duplicates = []
        for name in self._critical_node_names:
            count = counts[(name, "/")]
            if count > 1:
                duplicates.append("/%s (%d 个)" % (name, count))
        if duplicates:
            problems.append(
                "检测到重复关键节点：%s；请停止重复启动的导航栈后冷启动一套"
                % "，".join(duplicates)
            )

    def _action_server_providers(self, action_name: str) -> list[str]:
        """列出提供 action 的 SendGoal 服务的节点实例。"""
        service_name = action_name + "/_action/send_goal"
        providers = []
        for node_name, namespace in self.get_node_names_and_namespaces():
            try:
                services = self.get_service_names_and_types_by_node(
                    node_name, namespace
                )
            except RuntimeError:
                continue
            if any(name == service_name for name, _types in services):
                providers.append("%s%s" % (namespace.rstrip("/"), node_name))
        return providers

    def _check_action_servers(self, problems: list[str]) -> None:
        """公开代理和内部 Nav2 action 均只能有一个服务端。"""
        for action_name, expected_node in ACTION_SERVERS.items():
            providers = self._action_server_providers(action_name)
            if len(providers) != 1:
                problems.append(
                    "%s action server 必须唯一（期望 %s），实际为：%s；"
                    "请停止重复启动的导航栈"
                    % (action_name, expected_node,
                       "，".join(providers or ["无"]))
                )

    def _check_localization(self, problems: list[str]) -> None:
        if self._mode == "online_slam":
            if self._last_message.get("/map") is None:
                problems.append("在线 SLAM 尚未生成 /map")
            return
        if self._localization == "lidar_ndt" and not self._alignment_healthy:
            problems.append("/alignment_status 不是 healthy")
        if self._latest_pose is None:
            problems.append(
                "尚未收到 /amcl_pose，请先在 RViz 使用 2D Pose Estimate"
                if self._localization == "amcl" else "尚未收到定位位姿")
            return
        if (self._localization == "amcl" and
                isinstance(self._latest_pose, PoseWithCovarianceStamped)):
            covariance_problem = amcl_covariance_problem(
                # 使用安全监督的恢复阈值，避免 health_check 已 PASS 但
                # /pause_navigation 仍因滞回保持锁定。
                self._latest_pose.pose.covariance, 0.55, 0.50)
            if covariance_problem:
                problems.append(covariance_problem)
        if self._map is None:
            problems.append("地图语义校验失败：%s" % self._map_error)
            return
        point = (
            self._latest_pose.pose.pose.position
            if isinstance(self._latest_pose, PoseWithCovarianceStamped)
            else self._latest_pose.pose.position)
        reason = self._map.validate_pose(
            point.x, point.y, COMMISSIONING_CLEARANCE_M)
        if reason:
            problems.append("当前 /pcl_pose 不可安全导航：%s" % reason)

    def _check_controller(self, problems: list[str]) -> None:
        """确认四足控制器在线，避免 Nav2 健康但机器人无执行器。"""
        if not self._controller_client.wait_for_service(timeout_sec=1.0):
            problems.append("控制器管理服务不可用：/controller_manager/list_controllers")
            return
        future = self._controller_client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            problems.append("控制器状态查询超时")
            return
        controllers = {item.name: item.state for item in future.result().controller}
        # CHAMP 的步态节点为 quadruped_controller_node；真正由 ros2_control
        # 激活并执行关节轨迹的是 joint_group_effort_controller。
        if controllers.get("joint_group_effort_controller") != "active":
            problems.append(
                "joint_group_effort_controller 不是 active：%s"
                % controllers.get("joint_group_effort_controller", "未找到")
            )

    def _check_action_and_velocity(self, problems: list[str]) -> None:
        if not self._navigation_client.server_is_ready():
            problems.append("/navigate_to_pose action server 不可用")
        publishers = self.get_publishers_info_by_topic("/cmd_vel")
        publisher_names = sorted({endpoint.node_name for endpoint in publishers})
        if len(publishers) != 1 or publisher_names != ["collision_monitor"]:
            hint = ""
            if "teleop_twist_keyboard" in publisher_names:
                hint = (
                    "；请关闭直发 /cmd_vel 的键盘节点，导航时改用 "
                    "-r cmd_vel:=/cmd_vel_teleop")
            problems.append(
                "/cmd_vel 发布者必须唯一为 collision_monitor，实际为：%s（%d 个）%s"
                % (", ".join(publisher_names or ["无"]), len(publishers), hint)
            )
        for topic in ("/cmd_vel_nav", "/cmd_vel_switched", "/cmd_vel_smoothed"):
            if not self.get_publishers_info_by_topic(topic):
                problems.append("速度链缺少发布者：%s" % topic)

    def run(self) -> list[str]:
        problems: list[str] = []
        self._check_domain(problems)
        self._check_topics(problems)
        self._check_map_topics(problems)
        self._check_rviz_config(problems)
        self._check_tf(problems)
        self._check_duplicate_nodes(problems)
        self._check_lifecycle(problems)
        self._check_localization(problems)
        self._check_controller(problems)
        self._check_action_servers(problems)
        self._check_action_and_velocity(problems)
        return problems


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=5.0, help="墙钟采样时长（秒）")
    parser.add_argument("--timeout", type=float, default=1.5, help="话题过期阈值（秒）")
    parser.add_argument("--map-dir", default=default_map_dir(), help="地图包目录")
    parser.add_argument(
        "--mode", choices=("static_bundle", "static_map", "online_slam"),
        default="online_slam", help="健康检查拓扑模式")
    parser.add_argument(
        "--localization", choices=("amcl", "lidar_ndt", "ndt_cuda"),
        default="amcl", help="固定图定位后端")
    parser.add_argument("--expected-domain-id", type=int, default=0, help="普通仿真域")
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = HealthCheck(
        os.path.expanduser(args.map_dir), args.expected_domain_id, args.timeout,
        args.mode, args.localization,
    )
    try:
        node.sample(args.duration)
        problems = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if problems:
        print("健康检查 FAIL：")
        for problem in problems:
            print("  - %s" % problem)
        return 1
    print("健康检查 PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
