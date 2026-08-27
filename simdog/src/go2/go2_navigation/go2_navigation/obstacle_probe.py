"""用 Gazebo 标准服务量化 Go2 近距障碍感知可靠性。"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np
import rclpy
from gazebo_msgs.msg import ContactsState, EntityState
from gazebo_msgs.srv import DeleteEntity, GetEntityState, SetEntityState, SpawnEntity
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from go2_navigation.nav_tuning import package_root_from_module


DEFAULT_DISTANCES = (
    1.2, 1.1, 1.0, 0.9, 0.8, 0.6, 0.5, 0.4, 0.3, 0.25, 0.2, 0.15
)
SENSOR_NAMES = ("scan", "velodyne", "d435")
MODEL_NAME_PATTERN = re.compile(r"^[A-Za-z0-9_]+$")


@dataclass(frozen=True)
class FrameObservation:
    """单帧障碍探针观测。"""

    sensor: str
    arrival_monotonic: float
    message_stamp: float
    detected: bool
    measured_distance_m: float | None
    signed_error_m: float | None
    valid_count: int
    inf_count: int
    nan_count: int
    tf_ok: bool


def _percentile(values: Iterable[float], percentage: float) -> float | None:
    finite = [float(value) for value in values if math.isfinite(value)]
    if not finite:
        return None
    return float(np.percentile(np.asarray(finite), percentage))


def _observation(
    sensor: str,
    stamp: float,
    measured: float | None,
    expected_distance: float,
    tolerance: float,
    valid_count: int,
    inf_count: int = 0,
    nan_count: int = 0,
    tf_ok: bool = True,
) -> FrameObservation:
    error = None if measured is None else measured - expected_distance
    detected = error is not None and abs(error) <= tolerance and tf_ok
    return FrameObservation(
        sensor=sensor,
        arrival_monotonic=time.monotonic(),
        message_stamp=stamp,
        detected=detected,
        measured_distance_m=measured,
        signed_error_m=error,
        valid_count=valid_count,
        inf_count=inf_count,
        nan_count=nan_count,
        tf_ok=tf_ok,
    )


def scan_observation(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    expected_distance: float,
    probe_angle: float,
    half_width: float,
    tolerance: float,
    stamp: float = 0.0,
) -> FrameObservation:
    """把 LaserScan 射线投影到探针法向，测量传感器原点到障碍表面距离。"""

    candidates: list[float] = []
    valid_count = 0
    inf_count = 0
    nan_count = 0
    angular_margin = math.radians(1.0)
    half_angle = math.atan2(half_width + tolerance, max(expected_distance, 1e-6))
    for index, raw_range in enumerate(ranges):
        value = float(raw_range)
        if math.isinf(value):
            inf_count += 1
            continue
        if math.isnan(value):
            nan_count += 1
            continue
        if value < range_min or value > range_max:
            continue
        valid_count += 1
        delta = angle_min + index * angle_increment - probe_angle
        if abs(math.atan2(math.sin(delta), math.cos(delta))) > half_angle + angular_margin:
            continue
        forward = value * math.cos(delta)
        lateral = value * math.sin(delta)
        if forward > 0.0 and abs(lateral) <= half_width + tolerance:
            candidates.append(forward)
    measured = min(candidates, key=lambda value: abs(value - expected_distance), default=None)
    return _observation(
        "scan",
        stamp,
        measured,
        expected_distance,
        tolerance,
        valid_count,
        inf_count,
        nan_count,
    )


def cloud_observation(
    sensor: str,
    points: np.ndarray,
    expected_distance: float,
    probe_angle: float,
    half_width: float,
    half_height: float,
    tolerance: float,
    stamp: float = 0.0,
    tf_ok: bool = True,
) -> FrameObservation:
    """在 velodyne 坐标系内，从 PointCloud2 中提取探针前表面。"""

    array = np.asarray(points, dtype=float).reshape((-1, 3))
    finite_mask = np.isfinite(array).all(axis=1)
    finite = array[finite_mask]
    measured = None
    if finite.size:
        cosine = math.cos(probe_angle)
        sine = math.sin(probe_angle)
        forward = finite[:, 0] * cosine + finite[:, 1] * sine
        lateral = -finite[:, 0] * sine + finite[:, 1] * cosine
        region = (
            (forward > 0.0)
            & (np.abs(lateral) <= half_width + tolerance)
            # z 方向只给 2 cm 数值余量，避免把距雷达约 0.323 m 的地面
            # 当成近距方块；检测距离容差只用于方块前表面的法向误差。
            & (np.abs(finite[:, 2]) <= half_height + 0.02)
        )
        values = forward[region]
        if values.size:
            measured = float(values[np.argmin(np.abs(values - expected_distance))])
    return _observation(
        sensor,
        stamp,
        measured,
        expected_distance,
        tolerance,
        int(finite.shape[0]),
        nan_count=int(array.shape[0] - finite.shape[0]),
        tf_ok=tf_ok,
    )


def summarize_group(
    sensor: str,
    distance: float,
    group: int,
    observations: Sequence[FrameObservation],
    required_rate: float,
    maximum_abs_error_p95: float,
    contact_events: int,
) -> dict[str, Any]:
    """计算一组帧的检测率、误差、到达周期和无接触验收结果。"""

    detected = [item for item in observations if item.detected]
    absolute_errors = [abs(item.signed_error_m) for item in detected]
    arrivals = sorted(item.arrival_monotonic for item in observations)
    periods = [later - earlier for earlier, later in zip(arrivals, arrivals[1:])]
    rate = len(detected) / len(observations) if observations else 0.0
    error_p95 = _percentile(absolute_errors, 95.0)
    tf_rate = (
        sum(item.tf_ok for item in observations) / len(observations)
        if observations else 0.0
    )
    passed = (
        bool(observations)
        and rate >= required_rate
        and error_p95 is not None
        and error_p95 <= maximum_abs_error_p95
        and tf_rate == 1.0
        and contact_events == 0
    )
    return {
        "sensor": sensor,
        "distance_m": distance,
        "group": group,
        "frames": len(observations),
        "detections": len(detected),
        "detection_rate": rate,
        "measured_p50_m": _percentile(
            [item.measured_distance_m for item in detected], 50.0
        ),
        "abs_error_p50_m": _percentile(absolute_errors, 50.0),
        "abs_error_p95_m": error_p95,
        "period_p50_s": _percentile(periods, 50.0),
        "period_p95_s": _percentile(periods, 95.0),
        "period_p99_s": _percentile(periods, 99.0),
        "tf_success_rate": tf_rate,
        "contact_events": contact_events,
        "pass": passed,
    }


def reliable_detection_min_distance(
    summaries: Sequence[dict[str, Any]], sensor: str, required_groups: int
) -> float | None:
    """返回所有重复组均通过的最小表面距离。"""

    by_distance: dict[float, list[dict[str, Any]]] = {}
    for summary in summaries:
        if summary["sensor"] == sensor:
            by_distance.setdefault(float(summary["distance_m"]), []).append(summary)
    passing = [
        distance
        for distance, groups in by_distance.items()
        if len(groups) >= required_groups
        and all(bool(group["pass"]) for group in groups[:required_groups])
    ]
    return min(passing) if passing else None


def probe_sdf(model_name: str, depth: float, width: float, height: float) -> str:
    """生成带 ROS ContactSensor 的标准红色方块 SDF。"""

    if not MODEL_NAME_PATTERN.fullmatch(model_name):
        raise ValueError("model_name 只允许字母、数字和下划线")
    for name, value in (("depth", depth), ("width", width), ("height", height)):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} 必须为正有限数")
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>false</static>
    <allow_auto_disable>false</allow_auto_disable>
    <link name="probe_link">
      <gravity>false</gravity>
      <kinematic>true</kinematic>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.028</ixx><iyy>0.028</iyy><izz>0.015</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="probe_collision">
        <geometry><box><size>{depth} {width} {height}</size></box></geometry>
        <surface><contact><ode><min_depth>0.0001</min_depth></ode></contact></surface>
      </collision>
      <visual name="probe_visual">
        <geometry><box><size>{depth} {width} {height}</size></box></geometry>
        <material><ambient>0.9 0.05 0.05 1</ambient><diffuse>0.9 0.05 0.05 1</diffuse></material>
      </visual>
      <sensor name="probe_contact" type="contact">
        <always_on>true</always_on>
        <update_rate>60</update_rate>
        <contact><collision>probe_collision</collision></contact>
        <plugin name="probe_contact_ros" filename="libgazebo_ros_bumper.so">
          <ros>
            <namespace>/go2_obstacle_probe</namespace>
            <remapping>bumper_states:=contacts</remapping>
          </ros>
          <frame_name>world</frame_name>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>
"""


def _message_stamp(message: Any) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


def _quaternion_matrix(quaternion: Any) -> np.ndarray:
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    norm = x * x + y * y + z * z + w * w
    if norm <= 1e-12:
        return np.eye(3)
    scale = 2.0 / norm
    return np.asarray(
        [
            [1 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)],
            [scale * (x * y + z * w), 1 - scale * (x * x + z * z), scale * (y * z - x * w)],
            [scale * (x * z - y * w), scale * (y * z + x * w), 1 - scale * (x * x + y * y)],
        ]
    )


class ObstacleProbeNode(Node):
    """移动探针并同步采集 LaserScan、Velodyne、D435 和碰撞真值。"""

    def __init__(self, arguments: argparse.Namespace) -> None:
        super().__init__(
            "go2_obstacle_probe",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.arguments = arguments
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._recording = False
        self._expected_distance = 0.0
        self._observations = {name: [] for name in SENSOR_NAMES}
        self._contact_events = 0
        self._latest_final_velocity: tuple[float, float] | None = None
        self._spawned = False
        self._service_clients: dict[str, Any] = {}
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(LaserScan, "/scan", self._on_scan, sensor_qos)
        self.create_subscription(
            PointCloud2, "/velodyne_points", self._on_velodyne, sensor_qos
        )
        self.create_subscription(
            PointCloud2, "/depth/color/points", self._on_d435, sensor_qos
        )
        self.create_subscription(
            ContactsState, "/go2_obstacle_probe/contacts", self._on_contacts, 10
        )
        self.create_subscription(Twist, "/cmd_vel", self._on_final_velocity, 10)

    def _call(self, service_type: Any, name: str, request: Any, timeout: float = 15.0) -> Any:
        client = self._service_clients.get(name)
        if client is None:
            client = self.create_client(service_type, name)
            self._service_clients[name] = client
        if not client.wait_for_service(timeout_sec=min(timeout, 5.0)):
            raise RuntimeError(f"服务不可用：{name}")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            raise TimeoutError(f"服务超时：{name}")
        if future.exception() is not None:
            raise RuntimeError(f"服务失败 {name}：{future.exception()}")
        return future.result()

    def stop_navigation(self) -> bool:
        client = self.create_client(Trigger, "/navigation/stop")
        available = client.wait_for_service(timeout_sec=1.0)
        self.destroy_client(client)
        if not available:
            return False
        reply = self._call(Trigger, "/navigation/stop", Trigger.Request(), timeout=10.0)
        return bool(reply.success)

    def wait_for_zero_velocity(self, timeout: float = 5.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                velocity = self._latest_final_velocity
            if velocity is None or max(abs(value) for value in velocity) <= 1e-3:
                return
            time.sleep(0.05)
        raise TimeoutError("最终 /cmd_vel 未归零，拒绝放置近距障碍")

    def spawn_probe(self) -> None:
        if self.arguments.replace_existing:
            request = DeleteEntity.Request()
            request.name = self.arguments.model_name
            try:
                self._call(DeleteEntity, "/delete_entity", request, timeout=5.0)
            except (RuntimeError, TimeoutError):
                pass
        request = SpawnEntity.Request()
        request.name = self.arguments.model_name
        request.xml = probe_sdf(
            self.arguments.model_name,
            self.arguments.box_depth,
            self.arguments.box_width,
            self.arguments.box_height,
        )
        request.reference_frame = "world"
        request.initial_pose.position.x = 50.0
        request.initial_pose.position.y = 50.0
        request.initial_pose.position.z = 2.0
        request.initial_pose.orientation.w = 1.0
        reply = self._call(SpawnEntity, "/spawn_entity", request)
        if not reply.success:
            raise RuntimeError(reply.status_message or "Gazebo 拒绝生成障碍探针")
        self._spawned = True

    def delete_probe(self) -> None:
        if not self._spawned or self.arguments.keep_model:
            return
        request = DeleteEntity.Request()
        request.name = self.arguments.model_name
        reply = self._call(DeleteEntity, "/delete_entity", request, timeout=10.0)
        if not reply.success:
            raise RuntimeError(reply.status_message or "Gazebo 删除探针失败")
        self._spawned = False

    def move_probe(self, surface_distance: float) -> float:
        angle = math.radians(self.arguments.probe_angle_deg)
        center_distance = surface_distance + self.arguments.box_depth / 2.0
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self.arguments.model_name
        request.state.reference_frame = self.arguments.sensor_entity
        request.state.pose.position.x = (
            self.arguments.sensor_offset_x + center_distance * math.cos(angle)
        )
        request.state.pose.position.y = (
            self.arguments.sensor_offset_y + center_distance * math.sin(angle)
        )
        request.state.pose.position.z = self.arguments.sensor_offset_z
        request.state.pose.orientation.z = math.sin(angle / 2.0)
        request.state.pose.orientation.w = math.cos(angle / 2.0)
        reply = self._call(SetEntityState, "/set_entity_state", request)
        if not reply.success:
            raise RuntimeError(f"Gazebo 无法把探针移动到 {surface_distance:.3f} m")

        check = GetEntityState.Request()
        check.name = self.arguments.model_name
        check.reference_frame = self.arguments.sensor_entity
        state = self._call(GetEntityState, "/get_entity_state", check)
        if not state.success:
            raise RuntimeError("无法回读探针相对传感器的位置")
        relative_x = state.state.pose.position.x - self.arguments.sensor_offset_x
        relative_y = state.state.pose.position.y - self.arguments.sensor_offset_y
        actual_center = math.hypot(relative_x, relative_y)
        actual_surface = actual_center - self.arguments.box_depth / 2.0
        if abs(actual_surface - surface_distance) > 0.01:
            raise RuntimeError(
                f"探针定位误差过大：请求 {surface_distance:.3f} m，回读 {actual_surface:.3f} m"
            )
        return actual_surface

    def begin_group(self, expected_distance: float) -> None:
        with self._condition:
            self._expected_distance = expected_distance
            self._observations = {name: [] for name in SENSOR_NAMES}
            self._contact_events = 0
            self._recording = True

    def finish_group(
        self, sensors: Sequence[str], frames: int, timeout: float
    ) -> tuple[dict[str, list[FrameObservation]], int]:
        deadline = time.monotonic() + timeout
        with self._condition:
            while time.monotonic() < deadline:
                if all(len(self._observations[name]) >= frames for name in sensors):
                    break
                self._condition.wait(timeout=0.2)
            self._recording = False
            missing = {
                name: len(self._observations[name])
                for name in sensors if len(self._observations[name]) < frames
            }
            if missing:
                raise TimeoutError(f"采样超时，帧数不足：{missing}")
            return (
                {name: list(self._observations[name][:frames]) for name in sensors},
                self._contact_events,
            )

    def _settings(self) -> tuple[bool, float]:
        with self._lock:
            return self._recording, self._expected_distance

    def _append(self, observation: FrameObservation) -> None:
        with self._condition:
            if not self._recording:
                return
            target = self._observations[observation.sensor]
            if len(target) < self.arguments.frames_per_group:
                target.append(observation)
                self._condition.notify_all()

    def _on_scan(self, message: LaserScan) -> None:
        recording, distance = self._settings()
        if not recording:
            return
        self._append(
            scan_observation(
                message.ranges,
                message.angle_min,
                message.angle_increment,
                message.range_min,
                message.range_max,
                distance,
                math.radians(self.arguments.probe_angle_deg),
                self.arguments.box_width / 2.0,
                self.arguments.detection_tolerance,
                _message_stamp(message),
            )
        )

    def _cloud_points_in_velodyne(self, message: PointCloud2) -> tuple[np.ndarray, bool]:
        # Humble 的 read_points_numpy() 会错误地要求消息中的所有字段同型；
        # VLP-16 还包含 uint16 ring，因此先读取结构化 xyz 再显式堆叠。
        points = point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=False
        )
        array = np.column_stack((points["x"], points["y"], points["z"])).astype(
            float, copy=False
        )
        frame = message.header.frame_id.lstrip("/")
        if frame == "velodyne":
            return array, True
        try:
            transform = self._tf_buffer.lookup_transform(
                "velodyne",
                message.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException:
            return np.empty((0, 3)), False
        rotation = _quaternion_matrix(transform.transform.rotation)
        translation = np.asarray(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ]
        )
        return array @ rotation.T + translation, True

    def _on_cloud(self, sensor: str, message: PointCloud2) -> None:
        recording, distance = self._settings()
        if not recording:
            return
        try:
            points, tf_ok = self._cloud_points_in_velodyne(message)
            observation = cloud_observation(
                sensor,
                points,
                distance,
                math.radians(self.arguments.probe_angle_deg),
                self.arguments.box_width / 2.0,
                self.arguments.box_height / 2.0,
                self.arguments.detection_tolerance,
                _message_stamp(message),
                tf_ok=tf_ok,
            )
        except (AssertionError, ValueError):
            observation = cloud_observation(
                sensor,
                np.empty((0, 3)),
                distance,
                math.radians(self.arguments.probe_angle_deg),
                self.arguments.box_width / 2.0,
                self.arguments.box_height / 2.0,
                self.arguments.detection_tolerance,
                _message_stamp(message),
                tf_ok=False,
            )
        self._append(observation)

    def _on_velodyne(self, message: PointCloud2) -> None:
        self._on_cloud("velodyne", message)

    def _on_d435(self, message: PointCloud2) -> None:
        self._on_cloud("d435", message)

    def _on_contacts(self, message: ContactsState) -> None:
        if not message.states:
            return
        with self._condition:
            if self._recording:
                self._contact_events += len(message.states)

    def _on_final_velocity(self, message: Twist) -> None:
        with self._lock:
            self._latest_final_velocity = (float(message.linear.x), float(message.angular.z))


def _write_results(
    output_dir: Path,
    run_id: str,
    frame_rows: Sequence[dict[str, Any]],
    summaries: Sequence[dict[str, Any]],
    result: dict[str, Any],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    for filename, rows in (
        ("lidar_blind_zone_frames.csv", frame_rows),
        ("lidar_blind_zone_summary.csv", summaries),
    ):
        path = output_dir / filename
        if not rows:
            continue
        with path.open("w", encoding="utf-8", newline="") as stream:
            # 实验数据会进入 Git，固定 LF 避免 csv 模块默认的
            # CRLF 被 diff 误报为行尾空白。
            writer = csv.DictWriter(
                stream, fieldnames=list(rows[0]), lineterminator="\n"
            )
            writer.writeheader()
            writer.writerows(rows)
    (output_dir / "lidar_blind_zone_result.json").write_text(
        json.dumps({"run_id": run_id, **result}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Go2 Gazebo 近距障碍探针")
    parser.add_argument(
        "--distances",
        default=",".join(str(value) for value in DEFAULT_DISTANCES),
        help="传感器原点到障碍表面的逗号分隔距离，单位 m",
    )
    parser.add_argument("--groups", type=int, default=3)
    parser.add_argument("--frames-per-group", type=int, default=20)
    parser.add_argument("--sensors", default=",".join(SENSOR_NAMES))
    parser.add_argument("--frame-timeout", type=float, default=180.0)
    parser.add_argument("--settle-seconds", type=float, default=1.0)
    parser.add_argument("--detection-rate", type=float, default=0.95)
    parser.add_argument("--detection-tolerance", type=float, default=0.08)
    parser.add_argument("--maximum-error-p95", type=float, default=0.05)
    parser.add_argument("--box-depth", type=float, default=0.30)
    parser.add_argument("--box-width", type=float, default=0.30)
    parser.add_argument("--box-height", type=float, default=0.50)
    parser.add_argument("--probe-angle-deg", type=float, default=0.0)
    # Gazebo 会合并 Go2 的固定关节，因此用可查询的 base_link 实体加 URDF
    # base_link→velodyne 静态偏移，距离定义仍然以 Velodyne 原点为准。
    parser.add_argument("--sensor-entity", default="go2::base_link")
    parser.add_argument("--sensor-offset-x", type=float, default=0.20)
    parser.add_argument("--sensor-offset-y", type=float, default=0.0)
    parser.add_argument("--sensor-offset-z", type=float, default=0.1177)
    parser.add_argument("--model-name", default="go2_obstacle_probe")
    parser.add_argument("--replace-existing", action="store_true")
    parser.add_argument("--keep-model", action="store_true")
    parser.add_argument("--no-stop-navigation", action="store_true")
    parser.add_argument("--output-dir", type=Path)
    return parser


def _validated_arguments(arguments: argparse.Namespace) -> tuple[list[float], list[str]]:
    try:
        distances = [float(item.strip()) for item in arguments.distances.split(",")]
    except ValueError as error:
        raise ValueError("--distances 必须是逗号分隔数值") from error
    sensors = [item.strip() for item in arguments.sensors.split(",") if item.strip()]
    if not distances or any(not math.isfinite(value) or value <= 0.0 for value in distances):
        raise ValueError("所有测试距离必须为正有限数")
    if not sensors or any(sensor not in SENSOR_NAMES for sensor in sensors):
        raise ValueError("--sensors 只允许 scan,velodyne,d435")
    if len(sensors) != len(set(sensors)):
        raise ValueError("--sensors 不能包含重复项")
    if arguments.groups < 1 or arguments.frames_per_group < 2:
        raise ValueError("groups 至少为 1，frames-per-group 至少为 2")
    if not 0.0 < arguments.detection_rate <= 1.0:
        raise ValueError("detection-rate 必须在 (0, 1] 内")
    positive_values = (
        ("frame-timeout", arguments.frame_timeout),
        ("detection-tolerance", arguments.detection_tolerance),
        ("maximum-error-p95", arguments.maximum_error_p95),
    )
    if any(not math.isfinite(value) or value <= 0.0 for _, value in positive_values):
        raise ValueError("frame-timeout、detection-tolerance 和 maximum-error-p95 必须为正有限数")
    if not math.isfinite(arguments.settle_seconds) or arguments.settle_seconds < 0.0:
        raise ValueError("settle-seconds 必须为非负有限数")
    pose_values = (
        arguments.probe_angle_deg,
        arguments.sensor_offset_x,
        arguments.sensor_offset_y,
        arguments.sensor_offset_z,
    )
    if any(not math.isfinite(value) for value in pose_values):
        raise ValueError("探针方向和传感器偏移必须为有限数")
    probe_sdf(
        arguments.model_name,
        arguments.box_depth,
        arguments.box_width,
        arguments.box_height,
    )
    return distances, sensors


def main(argv: Sequence[str] | None = None) -> int:
    arguments = create_parser().parse_args(argv)
    try:
        distances, sensors = _validated_arguments(arguments)
    except ValueError as error:
        print(f"参数错误：{error}")
        return 2
    run_id = time.strftime("blind_zone_%Y%m%d_%H%M%S")
    output_dir = arguments.output_dir or (
        package_root_from_module() / "logs" / "blind_zone" / run_id
    )
    rclpy.init(args=None)
    node = ObstacleProbeNode(arguments)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    frame_rows: list[dict[str, Any]] = []
    summaries: list[dict[str, Any]] = []
    result_code = 0

    def current_result(status: str, error: str | None = None) -> dict[str, Any]:
        reliable = {
            sensor: reliable_detection_min_distance(
                summaries, sensor, arguments.groups
            )
            for sensor in sensors
        }
        result = {
            "status": status,
            # complete 只表示采集流程完成；acceptance_pass 才表示所有
            # 传感器/距离/重复组同时满足检出、误差、TF 与无接触门。
            "acceptance_pass": (
                status == "complete"
                and len(summaries)
                == len(distances) * arguments.groups * len(sensors)
                and all(bool(summary["pass"]) for summary in summaries)
            ),
            "distance_definition": "配置的传感器原点（默认 velodyne）到方块前表面的法向距离",
            "distances_m": distances,
            "probe_angle_deg": arguments.probe_angle_deg,
            "box_size_m": {
                "depth": arguments.box_depth,
                "width": arguments.box_width,
                "height": arguments.box_height,
            },
            "sensor_origin": {
                "entity": arguments.sensor_entity,
                "offset_xyz_m": [
                    arguments.sensor_offset_x,
                    arguments.sensor_offset_y,
                    arguments.sensor_offset_z,
                ],
            },
            "sensors": sensors,
            "groups": arguments.groups,
            "frames_per_group": arguments.frames_per_group,
            "detection_rate_threshold": arguments.detection_rate,
            "maximum_abs_error_p95_m": arguments.maximum_error_p95,
            "reliable_detection_min_distance_m": reliable,
            "summary_rows": len(summaries),
            "frame_rows": len(frame_rows),
        }
        if error:
            result["error"] = error
        return result

    try:
        if not arguments.no_stop_navigation:
            stopped = node.stop_navigation()
            print("导航已安全锁停；实验结束后不会自动恢复旧目标。" if stopped else "未发现导航 stop 服务；按纯 Gazebo 采样继续。")
        node.wait_for_zero_velocity()
        node.spawn_probe()
        for distance in distances:
            actual_distance = node.move_probe(distance)
            print(f"距离 {actual_distance:.3f} m：等待传感器稳定...", flush=True)
            time.sleep(arguments.settle_seconds)
            for group in range(1, arguments.groups + 1):
                node.begin_group(actual_distance)
                observations, contact_events = node.finish_group(
                    sensors,
                    arguments.frames_per_group,
                    arguments.frame_timeout,
                )
                for sensor in sensors:
                    group_summary = summarize_group(
                        sensor,
                        actual_distance,
                        group,
                        observations[sensor],
                        arguments.detection_rate,
                        arguments.maximum_error_p95,
                        contact_events,
                    )
                    summaries.append({"run_id": run_id, **group_summary})
                    for frame_index, observation in enumerate(observations[sensor], start=1):
                        frame_rows.append(
                            {
                                "run_id": run_id,
                                "distance_m": actual_distance,
                                "group": group,
                                "frame": frame_index,
                                **asdict(observation),
                                "contact_events_in_group": contact_events,
                            }
                        )
                rates = ", ".join(
                    f"{sensor}={summaries[-len(sensors) + index]['detection_rate']:.1%}"
                    for index, sensor in enumerate(sensors)
                )
                print(
                    f"  组 {group}/{arguments.groups}：{rates}，contact={contact_events}",
                    flush=True,
                )
                _write_results(
                    output_dir,
                    run_id,
                    frame_rows,
                    summaries,
                    current_result("in_progress"),
                )
        result = current_result("complete")
        _write_results(output_dir, run_id, frame_rows, summaries, result)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        print(f"结果目录：{output_dir}")
    except (RuntimeError, TimeoutError, TransformException, ValueError) as error:
        print(f"实验失败：{error}")
        if frame_rows or summaries:
            _write_results(
                output_dir,
                run_id,
                frame_rows,
                summaries,
                current_result("failed", str(error)),
            )
        result_code = 2
    finally:
        try:
            node.delete_probe()
        except (RuntimeError, TimeoutError) as error:
            print(f"清理警告：{error}")
            result_code = 2
        executor.shutdown(timeout_sec=5.0)
        spin_thread.join(timeout=5.0)
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()
    return result_code


if __name__ == "__main__":
    raise SystemExit(main())
