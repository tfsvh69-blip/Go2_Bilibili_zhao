#!/usr/bin/env python3
"""为 Nav2 目标提供地图、定位与动作就绪门禁。"""

from __future__ import annotations

import math
import time

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from go2_navigation.map_utils import (
    COMMISSIONING_CLEARANCE_M,
    MapValidationError,
    StaticMap,
    load_static_map,
    occupancy_grid_to_static_map,
)
from go2_navigation.localization_health import AmclCovarianceHealthTracker


class GoalGuard(Node):
    """公开安全的 ``/navigate_to_pose``，内部转发给原始 BT Navigator。"""

    def __init__(self) -> None:
        super().__init__("go2_goal_guard")
        self.declare_parameter("map_dir", "~/go2_maps/latest")
        self.declare_parameter("navigation_mode", "online_slam")
        self.declare_parameter("localization", "amcl")
        self.declare_parameter(
            "minimum_clearance_m", COMMISSIONING_CLEARANCE_M)
        self.declare_parameter("pose_timeout_sec", 1.5)
        self.declare_parameter("amcl_lost_position_std_m", 0.75)
        self.declare_parameter("amcl_lost_yaw_std_rad", 0.75)
        self.declare_parameter("amcl_recovery_position_std_m", 0.55)
        self.declare_parameter("amcl_recovery_yaw_std_rad", 0.50)
        self.declare_parameter("raw_action_name", "/navigate_to_pose_raw")
        mode = str(self.get_parameter("navigation_mode").value)
        self._navigation_mode = "static_map" if mode == "static_bundle" else mode
        self._localization = str(self.get_parameter("localization").value)
        if self._navigation_mode not in {"static_map", "online_slam"}:
            raise RuntimeError("未知 navigation_mode：%s" % self._navigation_mode)
        self._map = (
            self._load_map() if self._navigation_mode == "static_map" else None
        )
        self._map_wall_time = 0.0
        self._latest_pose: PoseWithCovarianceStamped | PoseStamped | None = None
        self._latest_pose_wall_time = 0.0
        self._localization_healthy = False
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
        self._rejection_publisher = self.create_publisher(String, "goal_rejected", 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        if self._navigation_mode == "static_map":
            if self._localization == "ndt_cuda":
                self.create_subscription(
                    PoseStamped, "/ndt_pose", self._pose_callback, 10)
            elif self._localization == "amcl":
                amcl_qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
                self.create_subscription(
                    PoseWithCovarianceStamped, "/amcl_pose",
                    self._pose_callback, amcl_qos)
            else:
                self.create_subscription(
                    PoseWithCovarianceStamped,
                    "/pcl_pose",
                    self._pose_callback,
                    10,
                )
            if self._localization == "lidar_ndt":
                self.create_subscription(
                    DiagnosticArray,
                    "/alignment_status",
                    self._alignment_callback,
                    10,
                )
        else:
            map_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                OccupancyGrid,
                "/map",
                self._map_callback,
                map_qos,
            )
        raw_action = self.get_parameter("raw_action_name").value
        self._raw_client = ActionClient(
            self,
            NavigateToPose,
            raw_action,
        )
        self._raw_goals = {}
        self._cancel_requested: set[bytes] = set()
        self._server = ActionServer(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info(
            "目标门禁已启动：mode=%s，公开 /navigate_to_pose，内部转发 %s"
            % (self._navigation_mode, raw_action)
        )

    def _load_map(self) -> StaticMap:
        map_dir = self.get_parameter("map_dir").value
        try:
            return load_static_map(map_dir)
        except MapValidationError as exc:
            raise RuntimeError("无法加载目标门禁地图：%s" % exc) from exc

    def _pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        self._latest_pose = message
        self._latest_pose_wall_time = time.monotonic()
        if (self._localization == "amcl" and
                isinstance(message, PoseWithCovarianceStamped)):
            self._amcl_health.update(message.pose.covariance)

    def _map_callback(self, message: OccupancyGrid) -> None:
        """更新在线 SLAM 的栅格快照。"""
        try:
            self._map = occupancy_grid_to_static_map(
                message.info.width,
                message.info.height,
                message.info.resolution,
                message.info.origin.position.x,
                message.info.origin.position.y,
                message.data,
            )
        except MapValidationError as exc:
            self.get_logger().warn("忽略无效动态地图：%s" % exc)
            return
        self._map_wall_time = time.monotonic()

    def _alignment_callback(self, message: DiagnosticArray) -> None:
        self._localization_healthy = False
        for status in message.status:
            if status.name != "lidar_localization_ros2/alignment":
                continue
            values = {item.key: item.value for item in status.values}
            self._localization_healthy = (
                status.message == "ok"
                and values.get("has_converged") == "true"
                and values.get("failure_category") == "healthy"
            )
            return

    def _publish_rejection(self, reason: str) -> None:
        message = String()
        message.data = reason
        self._rejection_publisher.publish(message)
        self.get_logger().warn("拒绝导航目标：%s" % reason)

    def _validate_pose(self, pose: PoseStamped) -> str | None:
        if pose.header.frame_id != "map":
            return "目标 frame_id 必须是 map"
        point = pose.pose.position
        orientation = pose.pose.orientation
        if not all(math.isfinite(value) for value in (
                point.x, point.y, point.z, orientation.x, orientation.y,
                orientation.z, orientation.w)):
            return "目标位姿包含非有限数值"
        if self._map is None:
            return "尚未收到可用的 /map"
        clearance = float(self.get_parameter("minimum_clearance_m").value)
        return self._map.validate_pose(point.x, point.y, clearance)

    def _online_robot_position(self) -> tuple[float, float] | str:
        try:
            transform = self._tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time())
        except TransformException as exc:
            return "在线 SLAM TF 不完整（map -> base_footprint）：%s" % exc
        point = transform.transform.translation
        return point.x, point.y

    def _validate_request(self, goal: NavigateToPose.Goal) -> str | None:
        reason = self._validate_pose(goal.pose)
        if reason:
            return reason
        if self._navigation_mode == "static_map":
            if self._localization == "lidar_ndt" and not self._localization_healthy:
                return "定位诊断未处于 healthy，先设置 /initialpose 并等待 NDT 收敛"
            if self._latest_pose is None:
                return (
                    "尚未收到 /amcl_pose，请先使用 2D Pose Estimate 设置初始位姿"
                    if self._localization == "amcl" else
                    "/pcl_pose 尚未收到定位位姿")
            if self._localization == "amcl" and not self._amcl_health.healthy:
                return self._amcl_health.reason
            if self._localization == "lidar_ndt":
                pose_timeout = float(self.get_parameter("pose_timeout_sec").value)
                if time.monotonic() - self._latest_pose_wall_time > pose_timeout:
                    return "/pcl_pose 已过期"
            localized = (
                self._latest_pose.pose.pose.position
                if isinstance(self._latest_pose, PoseWithCovarianceStamped)
                else self._latest_pose.pose.position)
            robot_x, robot_y = localized.x, localized.y
        else:
            if time.monotonic() - self._map_wall_time > 3.0:
                return "在线 SLAM /map 过期或尚未生成"
            position = self._online_robot_position()
            if isinstance(position, str):
                return position
            robot_x, robot_y = position
        clearance = float(self.get_parameter("minimum_clearance_m").value)
        if self._map is None:
            return "尚未收到可用的 /map"
        start_reason = self._map.validate_pose(robot_x, robot_y, clearance)
        if start_reason:
            return "当前定位位姿不可用于导航：%s" % start_reason
        if not self._raw_client.server_is_ready():
            return "底层 /navigate_to_pose_raw 尚未可用（Nav2 未 active）"
        return None

    def _goal_callback(self, goal: NavigateToPose.Goal) -> GoalResponse:
        reason = self._validate_request(goal)
        if reason:
            self._publish_rejection(reason)
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _goal_key(goal_handle) -> bytes:
        return bytes(goal_handle.goal_id.uuid)

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        key = self._goal_key(goal_handle)
        self._cancel_requested.add(key)
        raw_goal = self._raw_goals.get(key)
        if raw_goal is not None:
            raw_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    def _forward_feedback(self, goal_handle, feedback) -> None:
        if goal_handle.is_active:
            goal_handle.publish_feedback(feedback.feedback)

    async def _execute(self, goal_handle) -> NavigateToPose.Result:
        """异步等待底层 action，不占用四线程执行器轮询墙钟。"""
        result = NavigateToPose.Result()
        key = self._goal_key(goal_handle)
        try:
            raw_goal = await self._raw_client.send_goal_async(
                goal_handle.request,
                feedback_callback=lambda feedback:
                self._forward_feedback(goal_handle, feedback),
            )
            if raw_goal is None or not raw_goal.accepted:
                self._publish_rejection("底层 Nav2 拒绝了已校验目标")
                goal_handle.abort()
                return result
            self._raw_goals[key] = raw_goal
            if goal_handle.is_cancel_requested or key in self._cancel_requested:
                raw_goal.cancel_goal_async()
            raw_result = await raw_goal.get_result_async()
            if raw_result.status == GoalStatus.STATUS_SUCCEEDED:
                goal_handle.succeed()
            elif raw_result.status == GoalStatus.STATUS_CANCELED:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        except Exception as exc:  # action 关闭或底层节点退出
            self._publish_rejection("底层 Nav2 action 异常：%s" % exc)
            if goal_handle.is_active:
                goal_handle.abort()
            return result
        finally:
            self._raw_goals.pop(key, None)
            self._cancel_requested.discard(key)


def main() -> None:
    rclpy.init()
    node = GoalGuard()
    try:
        # Humble rclpy ActionServer waitable 在 CycloneDDS/lo 下可能持续报告
        # ready，直接 spin 会形成空转。5 ms 退让仍保留 200 Hz action 响应上限。
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
