#!/usr/bin/env python3
"""把定位、行为和导航图状态汇总为 /pause_navigation 安全锁。"""

from __future__ import annotations

import time

from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from go2_navigation.localization_health import AmclCovarianceHealthTracker


NAVIGATION_LIFECYCLE_NODES = {
    "controller_server",
    "planner_server",
    "smoother_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "collision_monitor",
    "velocity_smoother",
}


class LocalizationHealthTracker:
    """把逐帧 NDT 质量变为带滞回的导航级健康状态。"""

    HEALTHY = "HEALTHY"
    DEGRADED = "DEGRADED"
    LOST = "LOST"

    def __init__(self, failure_grace_sec: float = 2.0,
                 diagnostic_timeout_sec: float = 3.0,
                 recovery_samples: int = 5) -> None:
        self.failure_grace_sec = failure_grace_sec
        self.diagnostic_timeout_sec = diagnostic_timeout_sec
        self.recovery_samples = recovery_samples
        self.state = self.LOST
        self.reinitialization_requested = True
        self.last_diagnostic_time = 0.0
        self.last_healthy_time = 0.0
        self.healthy_samples = 0

    def update_reinitialization(self, requested: bool) -> None:
        self.reinitialization_requested = requested
        if requested:
            self.state = self.LOST
            self.healthy_samples = 0

    def update_alignment(self, healthy: bool, now: float) -> str:
        self.last_diagnostic_time = now
        if healthy:
            self.last_healthy_time = now
            self.healthy_samples += 1
            if (not self.reinitialization_requested and
                    self.healthy_samples >= self.recovery_samples):
                self.state = self.HEALTHY
            elif self.state != self.HEALTHY:
                self.state = self.DEGRADED
            return self.state

        self.healthy_samples = 0
        if (not self.reinitialization_requested and
                self.last_healthy_time > 0.0 and
                now - self.last_healthy_time <= self.failure_grace_sec):
            self.state = self.DEGRADED
        else:
            self.state = self.LOST
        return self.state

    def evaluate(self, now: float) -> str:
        if self.reinitialization_requested:
            self.state = self.LOST
        elif (self.last_diagnostic_time == 0.0 or
              now - self.last_diagnostic_time > self.diagnostic_timeout_sec):
            self.state = self.LOST
        elif (self.state == self.DEGRADED and
              now - self.last_healthy_time > self.failure_grace_sec):
            self.state = self.LOST
        return self.state


class NavigationSafetySupervisor(Node):
    """定位失效、行为占用或关键节点掉线时锁住所有速度输入。"""

    def __init__(self) -> None:
        super().__init__("go2_navigation_safety_supervisor")
        self.declare_parameter("navigation_mode", "online_slam")
        self.declare_parameter("localization", "amcl")
        self.declare_parameter("amcl_lost_position_std_m", 0.75)
        self.declare_parameter("amcl_lost_yaw_std_rad", 0.75)
        self.declare_parameter("amcl_recovery_position_std_m", 0.55)
        self.declare_parameter("amcl_recovery_yaw_std_rad", 0.50)
        # 最终 16x900 完整运动为 8.89 Hz；联调期仍采用 2.0 s，
        # 待更长压力与失效注入后再按 p99 单变量收紧。
        self.declare_parameter("scan_timeout_sec", 2.0)
        mode = str(self.get_parameter("navigation_mode").value)
        self._navigation_mode = "static_map" if mode == "static_bundle" else mode
        self._localization = str(self.get_parameter("localization").value)
        if self._navigation_mode not in {"static_map", "online_slam"}:
            raise RuntimeError("未知 navigation_mode：%s" % self._navigation_mode)

        self._ndt_health = LocalizationHealthTracker()
        self._amcl_health = AmclCovarianceHealthTracker(
            lost_position_std_m=float(
                self.get_parameter("amcl_lost_position_std_m").value),
            lost_yaw_std_rad=float(
                self.get_parameter("amcl_lost_yaw_std_rad").value),
            recovery_position_std_m=float(
                self.get_parameter("amcl_recovery_position_std_m").value),
            recovery_yaw_std_rad=float(
                self.get_parameter("amcl_recovery_yaw_std_rad").value),
        )
        self._localization_pose_seen = False
        self._behavior_running = False
        self._manual_pause = False
        self._cancel_pending = False
        self._map_wall_time = 0.0
        self._scan_wall_time = 0.0
        self._last_lock: bool | None = None
        self._graph_problem = "运行时健康检查尚未完成"

        self._publisher = self.create_publisher(Bool, "/pause_navigation", 10)
        self._stop_velocity_publisher = self.create_publisher(
            Twist, "/cmd_vel_stop", 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        if self._navigation_mode == "static_map":
            if self._localization == "amcl":
                amcl_qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
                self.create_subscription(
                    PoseWithCovarianceStamped, "/amcl_pose",
                    self._pose_callback, amcl_qos)
            elif self._localization == "lidar_ndt":
                self.create_subscription(
                    DiagnosticArray, "/alignment_status",
                    self._alignment_callback, 10)
                self.create_subscription(
                    Bool, "/reinitialization_requested",
                    self._reinit_callback, 10)
            else:
                self.create_subscription(
                    PoseStamped, "/ndt_pose", self._pose_callback, 10)
        else:
            map_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                OccupancyGrid, "/map", self._map_callback, map_qos)

        self.create_subscription(
            LaserScan, "/scan", self._scan_callback,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            String, "/go2_behaviors/status", self._behavior_callback, 10)

        self._lifecycle_nodes = set(NAVIGATION_LIFECYCLE_NODES)
        if self._navigation_mode == "static_map":
            self._lifecycle_nodes.add("map_server")
            if self._localization == "amcl":
                self._lifecycle_nodes.add("amcl")
            elif self._localization == "lidar_ndt":
                self._lifecycle_nodes.add("lidar_localization_node")
        self._lifecycle_states = {
            name: False for name in self._lifecycle_nodes}
        self._lifecycle_pending: set[str] = set()
        self._lifecycle_clients = {
            name: self.create_client(GetState, "/%s/get_state" % name)
            for name in self._lifecycle_nodes
        }

        self._required_nodes = set(NAVIGATION_LIFECYCLE_NODES) | {
            "go2_lidar_level_frame", "go2_lidar_scan_converter",
            "twist_mux", "go2_goal_guard"
        }
        if self._navigation_mode == "online_slam":
            self._required_nodes.add("slam_toolbox")
        else:
            self._required_nodes.add("map_server")
            if self._localization == "amcl":
                self._required_nodes.add("amcl")
            elif self._localization == "lidar_ndt":
                self._required_nodes.update(
                    {"lidar_localization_node", "ndt_global_ekf"})
            else:
                self._required_nodes.add("ndt_relocalization_node")

        self._cancel_clients = [
            self.create_client(CancelGoal, action + "/_action/cancel_goal")
            for action in ("/navigate_to_pose", "/navigate_to_pose_raw")
        ]
        self.create_service(Trigger, "/navigation/stop", self._stop_callback)
        self.create_service(Trigger, "/navigation/resume", self._resume_callback)
        self.create_timer(0.1, self._publish_lock_state)
        # ROS 图枚举和 lifecycle 服务查询限制为 1 Hz，避免辅助安全节点占满 CPU。
        self.create_timer(1.0, self._refresh_runtime_health)
        self.get_logger().info(
            "导航安全监督已启动：mode=%s，localization=%s"
            % (self._navigation_mode, self._localization))

    def _pose_callback(self, message) -> None:
        self._localization_pose_seen = True
        if (self._localization == "amcl" and
                isinstance(message, PoseWithCovarianceStamped)):
            self._amcl_health.update(message.pose.covariance)

    @staticmethod
    def _diagnostic_is_healthy(message: DiagnosticArray) -> bool:
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

    def _alignment_callback(self, message: DiagnosticArray) -> None:
        self._ndt_health.update_alignment(
            self._diagnostic_is_healthy(message), time.monotonic())

    def _reinit_callback(self, message: Bool) -> None:
        self._ndt_health.update_reinitialization(message.data)

    def _behavior_callback(self, message: String) -> None:
        self._behavior_running = message.data.startswith("running:")

    def _map_callback(self, _message: OccupancyGrid) -> None:
        self._map_wall_time = time.monotonic()

    def _scan_callback(self, _message: LaserScan) -> None:
        self._scan_wall_time = time.monotonic()

    def _lifecycle_result(self, name: str, future) -> None:
        self._lifecycle_pending.discard(name)
        try:
            result = future.result()
        except Exception:  # 服务节点可能恰好在关闭
            self._lifecycle_states[name] = False
            return
        self._lifecycle_states[name] = (
            result is not None and result.current_state.id == 3)

    def _refresh_runtime_health(self) -> None:
        live_nodes = {
            name for name, _namespace in self.get_node_names_and_namespaces()}
        missing = sorted(self._required_nodes - live_nodes)
        for name, client in self._lifecycle_clients.items():
            if name in self._lifecycle_pending or not client.service_is_ready():
                self._lifecycle_states[name] = False
                continue
            future = client.call_async(GetState.Request())
            self._lifecycle_pending.add(name)
            future.add_done_callback(
                lambda completed, node_name=name:
                self._lifecycle_result(node_name, completed))
        inactive = sorted(
            name for name, active in self._lifecycle_states.items() if not active)
        if missing:
            self._graph_problem = "关键导航节点缺失：" + ", ".join(missing)
        elif inactive:
            self._graph_problem = "生命周期节点未 active：" + ", ".join(inactive)
        else:
            self._graph_problem = ""

    def _cancel_all_goals(self) -> bool:
        if not all(client.service_is_ready() for client in self._cancel_clients):
            return False
        for client in self._cancel_clients:
            client.call_async(CancelGoal.Request())
        return True

    def _stop_callback(self, _request, response):
        self._manual_pause = True
        self._cancel_pending = True
        self._publish_lock_state()
        canceled = self._cancel_all_goals()
        self._cancel_pending = not canceled
        response.success = True
        response.message = (
            "已锁存导航暂停并取消全部目标" if canceled else
            "已锁存导航暂停；action 发现完成后将继续取消目标")
        return response

    def _resume_callback(self, _request, response):
        locked, reason = self._automatic_lock_reason()
        if locked:
            response.success = False
            response.message = "不能恢复：%s" % reason
            return response
        self._manual_pause = False
        self._cancel_pending = False
        self._publish_lock_state()
        response.success = True
        response.message = "已解除人工暂停；旧目标不会自动续行"
        return response

    def _has_planar_localization_tf(self) -> bool:
        return self._tf_buffer.can_transform(
            "map", "base_footprint", rclpy.time.Time())

    def _automatic_lock_reason(self) -> tuple[bool, str]:
        now = time.monotonic()
        if now - self._scan_wall_time > float(
                self.get_parameter("scan_timeout_sec").value):
            return True, "/scan 尚未收到或已过期"
        if self._navigation_mode == "online_slam":
            if now - self._map_wall_time > 3.0:
                return True, "在线 SLAM /map 尚未生成或已过期"
            if not self._has_planar_localization_tf():
                return True, "在线 SLAM TF 缺少 map -> base_footprint"
        elif self._localization == "lidar_ndt":
            state = self._ndt_health.evaluate(now)
            if state == LocalizationHealthTracker.LOST:
                return True, "NDT 定位 LOST 或请求重定位"
            # DEGRADED 期间继续使用最近一次全局修正与连续 odom，不制造速度抖动。
            if not self._has_planar_localization_tf():
                return True, "NDT 二维融合 TF 缺少 map -> base_footprint"
        else:
            if not self._localization_pose_seen:
                return True, "固定图定位尚未收到初始位姿"
            if self._localization == "amcl" and not self._amcl_health.healthy:
                return True, self._amcl_health.reason
            if not self._has_planar_localization_tf():
                return True, "固定图定位 TF 缺少 map -> base_footprint"
        if self._behavior_running:
            return True, "行为动作占用控制权"
        if self._graph_problem:
            return True, self._graph_problem
        return False, "导航链路健康"

    def _should_lock(self) -> tuple[bool, str]:
        if self._manual_pause:
            return True, "用户手动停止"
        return self._automatic_lock_reason()

    def _publish_lock_state(self) -> None:
        if self._cancel_pending and self._cancel_all_goals():
            self._cancel_pending = False
        locked, reason = self._should_lock()
        self._publisher.publish(Bool(data=locked))
        if locked:
            self._stop_velocity_publisher.publish(Twist())
        if locked != self._last_lock:
            self.get_logger().info(
                ("导航已锁定：" if locked else "导航已解锁：") + reason)
            self._last_lock = locked


def main() -> None:
    rclpy.init()
    node = NavigationSafetySupervisor()
    try:
        # 与 goal_guard 一致限制 Humble rclpy waitable 空转；200 Hz 上限远高于
        # 10 Hz 安全锁发布与 1 Hz 图审计需求。
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
