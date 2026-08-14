"""从 URDF collision 与仿真步态 TF 校准 Go2 二维导航足迹。"""

from __future__ import annotations

import argparse
import ast
import csv
import hashlib
import json
import math
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence
from xml.etree import ElementTree

import numpy as np
import rclpy
from geometry_msgs.msg import PolygonStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from go2_navigation.nav_tuning import package_root_from_module


class _VerificationFinished(Exception):
    """仅用于让 verify-only 进入统一停车与资源释放流程。"""


@dataclass(frozen=True)
class CollisionShape:
    """URDF 中一个 collision 几何及其相对 link 的位姿。"""

    source: str
    link: str
    kind: str
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    dimensions: tuple[float, ...]

    def local_points(self, circle_samples: int = 32) -> np.ndarray:
        """返回 collision 边界点，坐标已包含 collision origin。"""

        if circle_samples < 8:
            raise ValueError("circle_samples 至少为 8")
        if self.kind == "box":
            half = np.asarray(self.dimensions, dtype=float) / 2.0
            points = np.asarray(
                [
                    (sx * half[0], sy * half[1], sz * half[2])
                    for sx in (-1.0, 1.0)
                    for sy in (-1.0, 1.0)
                    for sz in (-1.0, 1.0)
                ]
            )
        elif self.kind == "cylinder":
            radius, length = self.dimensions
            points = np.asarray(
                [
                    (
                        radius * math.cos(2.0 * math.pi * index / circle_samples),
                        radius * math.sin(2.0 * math.pi * index / circle_samples),
                        z,
                    )
                    for z in (-length / 2.0, length / 2.0)
                    for index in range(circle_samples)
                ]
            )
        elif self.kind == "sphere":
            radius = self.dimensions[0]
            # 球体旋转不变；用 Fibonacci 球面加六个主轴端点，
            # 使任意 link 倾斜下的 XY 投影仍保持亚毫米级近似。
            count = max(128, circle_samples * 4)
            golden_angle = math.pi * (3.0 - math.sqrt(5.0))
            sphere_points = []
            for index in range(count):
                z = 1.0 - 2.0 * (index + 0.5) / count
                radial = math.sqrt(max(0.0, 1.0 - z * z))
                angle = index * golden_angle
                sphere_points.append(
                    (
                        radius * radial * math.cos(angle),
                        radius * radial * math.sin(angle),
                        radius * z,
                    )
                )
            sphere_points.extend(
                [
                    (radius, 0.0, 0.0),
                    (-radius, 0.0, 0.0),
                    (0.0, radius, 0.0),
                    (0.0, -radius, 0.0),
                    (0.0, 0.0, radius),
                    (0.0, 0.0, -radius),
                ]
            )
            points = np.asarray(sphere_points)
        else:
            raise ValueError(f"不支持的 collision 类型：{self.kind}")
        rotation = rpy_matrix(*self.origin_rpy)
        return points @ rotation.T + np.asarray(self.origin_xyz)


@dataclass(frozen=True)
class TransformRecord:
    """一个采样时刻中 base_footprint 到 collision link 的 TF。"""

    case: str
    sample: int
    wall_time: float
    ros_time: float
    link: str
    tx: float
    ty: float
    tz: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class SampleRecord:
    """一帧全部 collision 投影的边界摘要。"""

    case: str
    sample: int
    wall_time: float
    ros_time: float
    point_count: int
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    final_linear_x: float
    final_linear_y: float
    final_angular_z: float
    odom_x: float
    odom_y: float
    odom_yaw: float


def _numbers(text: str | None, count: int, default: float = 0.0) -> tuple[float, ...]:
    if text is None:
        return tuple(default for _ in range(count))
    values = tuple(float(value) for value in text.split())
    if len(values) != count or any(not math.isfinite(value) for value in values):
        raise ValueError(f"期望 {count} 个有限数，实际为：{text}")
    return values


def parse_collision_shapes(urdf: str) -> list[CollisionShape]:
    """使用 Python 标准库解析 URDF 的 box/cylinder/sphere collision。"""

    root = ElementTree.fromstring(urdf)
    shapes: list[CollisionShape] = []
    unsupported: list[str] = []
    for link in root.findall("link"):
        link_name = link.get("name", "")
        for index, collision in enumerate(link.findall("collision")):
            source = f"{link_name}:{collision.get('name') or index}"
            origin = collision.find("origin")
            xyz = _numbers(origin.get("xyz") if origin is not None else None, 3)
            rpy = _numbers(origin.get("rpy") if origin is not None else None, 3)
            geometry = collision.find("geometry")
            children = [] if geometry is None else list(geometry)
            if len(children) != 1:
                raise ValueError(f"{source} collision geometry 数量不是 1")
            item = children[0]
            if item.tag == "box":
                dimensions = _numbers(item.get("size"), 3)
            elif item.tag == "cylinder":
                dimensions = (
                    float(item.get("radius", "nan")),
                    float(item.get("length", "nan")),
                )
            elif item.tag == "sphere":
                dimensions = (float(item.get("radius", "nan")),)
            else:
                unsupported.append(f"{source}={item.tag}")
                continue
            if any(not math.isfinite(value) or value <= 0.0 for value in dimensions):
                raise ValueError(f"{source} collision 尺寸必须为正有限数")
            shapes.append(
                CollisionShape(source, link_name, item.tag, xyz, rpy, dimensions)
            )
    if unsupported:
        raise ValueError("存在未支持的 collision：" + ", ".join(unsupported))
    if not shapes:
        raise ValueError("robot_description 中没有可用 collision")
    return shapes


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """返回 URDF fixed-axis roll/pitch/yaw 旋转矩阵。"""

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """返回单位四元数旋转矩阵。"""

    norm = x * x + y * y + z * z + w * w
    if norm <= 1.0e-12:
        return np.eye(3)
    scale = 2.0 / norm
    return np.asarray(
        [
            [1 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)],
            [scale * (x * y + z * w), 1 - scale * (x * x + z * z), scale * (y * z - x * w)],
            [scale * (x * z - y * w), scale * (y * z + x * w), 1 - scale * (x * x + y * y)],
        ]
    )


def project_shape(
    shape: CollisionShape,
    translation: Sequence[float],
    quaternion: Sequence[float],
) -> np.ndarray:
    """把 collision 从 link 投影到 base_footprint XY。"""

    points = shape.local_points()
    rotation = quaternion_matrix(*quaternion)
    transformed = points @ rotation.T + np.asarray(translation, dtype=float)
    return transformed[:, :2]


def convex_hull(points: Iterable[Sequence[float]]) -> list[tuple[float, float]]:
    """用 monotonic chain 返回不重复首点的逆时针凸包。"""

    unique = sorted(
        {
            (round(float(point[0]), 9), round(float(point[1]), 9))
            for point in points
            if math.isfinite(float(point[0])) and math.isfinite(float(point[1]))
        }
    )
    if len(unique) < 3:
        raise ValueError("计算足迹至少需要 3 个不共线点")

    def cross(
        origin: tuple[float, float],
        first: tuple[float, float],
        second: tuple[float, float],
    ) -> float:
        return ((first[0] - origin[0]) * (second[1] - origin[1]) -
                (first[1] - origin[1]) * (second[0] - origin[0]))

    lower: list[tuple[float, float]] = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper: list[tuple[float, float]] = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    hull = lower[:-1] + upper[:-1]
    if len(hull) < 3:
        raise ValueError("足迹点全部共线")
    return hull


def polygon_area(points: Sequence[Sequence[float]]) -> float:
    """返回多边形有向面积的绝对值。"""

    return abs(
        sum(
            float(first[0]) * float(second[1]) -
            float(first[1]) * float(second[0])
            for first, second in zip(points, points[1:] + points[:1])
        ) / 2.0
    )


def calibration_statistics(
    sample_points: Sequence[np.ndarray],
    resolution: float,
    direction_count: int = 72,
) -> dict[str, Any]:
    """计算凸包、姿态尾部误差与栅格离散 padding。"""

    if not sample_points:
        raise ValueError("没有足迹采样")
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("costmap resolution 必须为正有限数")
    if direction_count < 8:
        raise ValueError("direction_count 至少为 8")
    all_points = np.vstack(sample_points)
    hull = convex_hull(all_points)
    directions = np.asarray(
        [
            (
                math.cos(2.0 * math.pi * index / direction_count),
                math.sin(2.0 * math.pi * index / direction_count),
            )
            for index in range(direction_count)
        ]
    )
    supports = np.asarray(
        [np.max(points @ directions.T, axis=0) for points in sample_points]
    )
    maximum = np.max(supports, axis=0)
    percentile_99 = np.percentile(supports, 99.0, axis=0)
    tails = maximum - percentile_99
    statistical_error = float(np.max(tails))
    half_cell = resolution / 2.0
    unrounded_padding = statistical_error + half_cell
    padding = math.ceil((unrounded_padding - 1.0e-12) / 0.005) * 0.005
    return {
        "raw_hull": hull,
        "raw_hull_area_m2": polygon_area(hull),
        "direction_count": direction_count,
        "statistical_tail_m": statistical_error,
        "half_costmap_cell_m": half_cell,
        "unrounded_padding_m": unrounded_padding,
        "recommended_padding_m": padding,
        "support_tail_p50_m": float(np.percentile(tails, 50.0)),
        "support_tail_p95_m": float(np.percentile(tails, 95.0)),
        "support_tail_p99_m": float(np.percentile(tails, 99.0)),
    }


def footprint_string(points: Sequence[Sequence[float]], digits: int = 3) -> str:
    """返回 Nav2 可直接使用的 footprint 参数字符串。"""

    rounded = [
        (round(float(point[0]), digits), round(float(point[1]), digits))
        for point in points
    ]
    # 毫米舍入可能把相邻凸包顶点合并。再取一次凸包，
    # 同时消除重复点和新出现的共线中间点。
    rounded_hull = convex_hull(rounded)
    return json.dumps(rounded_hull, ensure_ascii=False, separators=(",", ","))


def parse_footprint_parameter(value: str) -> list[tuple[float, float]]:
    """解析 Nav2 footprint 字符串，不执行任意代码。"""

    data = ast.literal_eval(value)
    if not isinstance(data, (list, tuple)) or len(data) < 3:
        raise ValueError("footprint 至少需要 3 个顶点")
    points = []
    for point in data:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError("footprint 顶点必须是 [x,y]")
        x, y = float(point[0]), float(point[1])
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("footprint 顶点必须为有限数")
        points.append((x, y))
    return points


def pad_footprint(
    points: Sequence[Sequence[float]], padding: float
) -> list[tuple[float, float]]:
    """按 Nav2 1.1.20 ``padFootprint`` 的公开语义计算逐轴外扩。"""

    if not math.isfinite(padding) or padding < 0.0:
        raise ValueError("footprint padding 必须为非负有限数")

    # 兼容依据：nav2_costmap_2d/src/footprint.cpp（Willow Garage BSD-3-Clause
    # 版权头）。这里只独立表达 sign0 与逐轴加 padding 的数值关系，用于验证
    # ROS 进程的输出，不链接或替代 Nav2 实现。
    def sign_zero(value: float) -> float:
        return -1.0 if value < 0.0 else 1.0 if value > 0.0 else 0.0

    return [
        (
            float(point[0]) + sign_zero(float(point[0])) * padding,
            float(point[1]) + sign_zero(float(point[1])) * padding,
        )
        for point in points
    ]


def transform_planar_points(
    points: Sequence[Sequence[float]],
    translation: Sequence[float],
    quaternion: Sequence[float],
) -> list[tuple[float, float]]:
    """使用三维 TF 变换平面点，返回目标坐标系中的 XY。"""

    source = np.asarray(
        [(float(point[0]), float(point[1]), 0.0) for point in points]
    )
    transformed = (
        source @ quaternion_matrix(*quaternion).T
        + np.asarray(translation, dtype=float)
    )
    return [(float(point[0]), float(point[1])) for point in transformed]


def quaternion_to_yaw(quaternion: Any) -> float:
    """从 geometry_msgs Quaternion 取平面 yaw。"""

    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class FootprintCalibratorNode(Node):
    """通过安全速度链驱动步态，同步采集 collision TF。"""

    def __init__(self, arguments: argparse.Namespace) -> None:
        super().__init__(
            "go2_footprint_calibrator",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.arguments = arguments
        self._lock = threading.RLock()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        # rclpy.Node 内部已使用 _clients 列表，不得同名覆盖。
        self._service_clients: dict[str, Any] = {}
        self._latest_velocity = (0.0, 0.0, 0.0)
        self._pause_navigation: bool | None = None
        self._odom_pose = (0.0, 0.0, 0.0)
        self._published_footprints: dict[str, PolygonStamped] = {}
        self._command = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_velocity, 10)
        self.create_subscription(Bool, "/pause_navigation", self._on_pause, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        footprint_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._footprint_subscriptions = [
            self.create_subscription(
                PolygonStamped,
                f"/{scope}_costmap/published_footprint",
                lambda message, name=scope: self._on_footprint(name, message),
                footprint_qos,
            )
            for scope in ("local", "global")
        ]

    def _on_velocity(self, message: Twist) -> None:
        with self._lock:
            self._latest_velocity = (
                float(message.linear.x),
                float(message.linear.y),
                float(message.angular.z),
            )

    def _on_pause(self, message: Bool) -> None:
        with self._lock:
            self._pause_navigation = bool(message.data)

    def _on_odom(self, message: Odometry) -> None:
        with self._lock:
            self._odom_pose = (
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                quaternion_to_yaw(message.pose.pose.orientation),
            )

    def _on_footprint(self, scope: str, message: PolygonStamped) -> None:
        with self._lock:
            self._published_footprints[scope] = message

    def _call(
        self, service_type: Any, name: str, request: Any, timeout: float = 10.0
    ) -> Any:
        client = self._service_clients.get(name)
        if client is None:
            client = self.create_client(service_type, name)
            self._service_clients[name] = client
        if not client.wait_for_service(timeout_sec=min(5.0, timeout)):
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

    def trigger(self, name: str) -> None:
        reply = self._call(Trigger, name, Trigger.Request(), timeout=15.0)
        if not reply.success:
            raise RuntimeError(reply.message or f"{name} 拒绝请求")

    def parameter_values(self, node: str, names: Sequence[str]) -> list[Any]:
        request = GetParameters.Request()
        request.names = list(names)
        reply = self._call(
            GetParameters, f"{node}/get_parameters", request, timeout=15.0
        )
        if len(reply.values) != len(names):
            raise RuntimeError(f"{node} 参数回复数量错误")
        values: list[Any] = []
        for name, value in zip(names, reply.values):
            if value.type == ParameterType.PARAMETER_STRING:
                values.append(value.string_value)
            elif value.type == ParameterType.PARAMETER_DOUBLE:
                values.append(value.double_value)
            else:
                raise RuntimeError(f"{node}.{name} 类型不是 string/double")
        return values

    def robot_description(self) -> str:
        return str(
            self.parameter_values("/robot_state_publisher", ["robot_description"])[0]
        )

    def costmap_geometry(self) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for scope, node in (
            ("local", "/local_costmap/local_costmap"),
            ("global", "/global_costmap/global_costmap"),
        ):
            footprint, padding, resolution = self.parameter_values(
                node, ["footprint", "footprint_padding", "resolution"]
            )
            result[scope] = {
                "footprint": footprint,
                "points": parse_footprint_parameter(footprint),
                "padding": float(padding),
                "resolution": float(resolution),
            }
        if result["local"] != result["global"]:
            raise RuntimeError("local/global footprint、padding 或 resolution 不一致")
        return result

    def publish_command(self, command: Sequence[float]) -> None:
        message = Twist()
        message.linear.x = float(command[0])
        message.linear.y = float(command[1])
        message.angular.z = float(command[2])
        self._command.publish(message)

    def command_for(self, command: Sequence[float], duration: float) -> None:
        deadline = time.monotonic() + duration
        period = 1.0 / self.arguments.command_hz
        while time.monotonic() < deadline:
            self.publish_command(command)
            time.sleep(period)

    def wait_for_pause(self, expected: bool, timeout: float = 10.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                current = self._pause_navigation
            if current is expected:
                return
            time.sleep(0.05)
        raise TimeoutError(f"/pause_navigation 未变为 {expected}")

    def wait_for_zero_velocity(self, timeout: float = 5.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                velocity = self._latest_velocity
            if max(abs(value) for value in velocity) <= 1.0e-3:
                return
            time.sleep(0.05)
        raise TimeoutError("最终 /cmd_vel 未归零")

    def verify_published_footprints(
        self,
        geometry: dict[str, Any],
        tolerance: float,
        timeout: float = 10.0,
    ) -> dict[str, Any]:
        """把发布足迹还原到 base_footprint，并与参数内部效果比较。"""

        if not math.isfinite(tolerance) or tolerance <= 0.0:
            raise ValueError("verification-tolerance 必须为正有限数")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                ready = all(
                    scope in self._published_footprints
                    for scope in ("local", "global")
                )
            if ready:
                break
            time.sleep(0.05)
        if not ready:
            raise TimeoutError("未同时收到 local/global published_footprint")

        results: dict[str, Any] = {}
        for scope in ("local", "global"):
            with self._lock:
                message = self._published_footprints[scope]
            transform = self._tf_buffer.lookup_transform(
                "base_footprint",
                message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=2.0),
            )
            translation = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            )
            quaternion = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
            actual = transform_planar_points(
                [(point.x, point.y) for point in message.polygon.points],
                translation,
                quaternion,
            )
            expected = pad_footprint(
                geometry[scope]["points"], geometry[scope]["padding"]
            )
            count_matches = len(actual) == len(expected)
            errors = [
                math.hypot(left[0] - right[0], left[1] - right[1])
                for left, right in zip(actual, expected)
            ]
            maximum_error = max(errors, default=math.inf)
            passed = count_matches and maximum_error <= tolerance
            results[scope] = {
                "status": "PASS" if passed else "FAIL",
                "message_frame": message.header.frame_id,
                "vertex_count": len(actual),
                "expected_vertex_count": len(expected),
                "maximum_vertex_error_m": maximum_error,
                "tolerance_m": tolerance,
                "bounds_base_footprint_m": {
                    "min_x": min(point[0] for point in actual),
                    "max_x": max(point[0] for point in actual),
                    "min_y": min(point[1] for point in actual),
                    "max_y": max(point[1] for point in actual),
                },
                "actual_points_base_footprint": actual,
                "expected_points_base_footprint": expected,
            }
        return {
            "status": (
                "PASS"
                if all(item["status"] == "PASS" for item in results.values())
                else "FAIL"
            ),
            "coordinate_frame": "base_footprint",
            "scopes": results,
        }

    def projection_sample(
        self,
        case: str,
        sample_index: int,
        shapes_by_link: dict[str, list[CollisionShape]],
    ) -> tuple[np.ndarray, dict[str, np.ndarray], list[TransformRecord], SampleRecord]:
        wall_time = time.monotonic()
        ros_time = self.get_clock().now().nanoseconds * 1.0e-9
        all_points: list[np.ndarray] = []
        per_link: dict[str, np.ndarray] = {}
        transform_rows: list[TransformRecord] = []
        for link, link_shapes in shapes_by_link.items():
            transform = self._tf_buffer.lookup_transform(
                "base_footprint",
                link,
                Time(),
                timeout=Duration(seconds=0.15),
            )
            translation = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            )
            quaternion = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
            points = np.vstack(
                [project_shape(shape, translation, quaternion) for shape in link_shapes]
            )
            per_link[link] = points
            all_points.append(points)
            transform_rows.append(
                TransformRecord(
                    case,
                    sample_index,
                    wall_time,
                    ros_time,
                    link,
                    *translation,
                    *quaternion,
                )
            )
        combined = np.vstack(all_points)
        with self._lock:
            velocity = self._latest_velocity
            odom = self._odom_pose
        sample = SampleRecord(
            case,
            sample_index,
            wall_time,
            ros_time,
            int(combined.shape[0]),
            float(np.min(combined[:, 0])),
            float(np.max(combined[:, 0])),
            float(np.min(combined[:, 1])),
            float(np.max(combined[:, 1])),
            *velocity,
            *odom,
        )
        return combined, per_link, transform_rows, sample

    def run_case(
        self,
        name: str,
        command: Sequence[float],
        duration: float,
        shapes_by_link: dict[str, list[CollisionShape]],
    ) -> tuple[
        list[np.ndarray],
        dict[str, list[np.ndarray]],
        list[TransformRecord],
        list[SampleRecord],
    ]:
        print(
            f"场景 {name}：命令 {tuple(command)}，"
            f"预热 {self.arguments.warmup_seconds:.1f} s",
            flush=True,
        )
        self.command_for(command, self.arguments.warmup_seconds)
        points: list[np.ndarray] = []
        link_points = {link: [] for link in shapes_by_link}
        transforms: list[TransformRecord] = []
        samples: list[SampleRecord] = []
        deadline = time.monotonic() + duration
        period = 1.0 / self.arguments.sample_hz
        next_sample = time.monotonic()
        max_velocity = 0.0
        while time.monotonic() < deadline:
            self.publish_command(command)
            if time.monotonic() >= next_sample:
                try:
                    combined, per_link, tf_rows, sample = self.projection_sample(
                        name, len(samples) + 1, shapes_by_link
                    )
                except TransformException as error:
                    raise RuntimeError(f"{name} TF 采样失败：{error}") from error
                points.append(combined)
                for link, values in per_link.items():
                    link_points[link].append(values)
                transforms.extend(tf_rows)
                samples.append(sample)
                max_velocity = max(
                    max_velocity,
                    abs(sample.final_linear_x),
                    abs(sample.final_linear_y),
                    abs(sample.final_angular_z),
                )
                next_sample += period
            time.sleep(min(0.01, period / 2.0))
        expected = max(abs(float(value)) for value in command)
        if expected > 0.0 and max_velocity < expected * 0.45:
            raise RuntimeError(
                f"{name} 最终速度峰值 {max_velocity:.3f} 未证明命令经安全链落地"
            )
        if len(samples) < max(3, int(duration * self.arguments.sample_hz * 0.5)):
            raise RuntimeError(f"{name} 采样帧数不足：{len(samples)}")
        print(f"  完成 {len(samples)} 帧，最终速度峰值={max_velocity:.3f}", flush=True)
        return points, link_points, transforms, samples


def _write_csv(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=list(rows[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)


def _case_summary(points: Sequence[np.ndarray]) -> dict[str, Any]:
    combined = np.vstack(points)
    hull = convex_hull(combined)
    return {
        "frames": len(points),
        "point_count": int(combined.shape[0]),
        "hull": hull,
        "area_m2": polygon_area(hull),
        "bounds_m": {
            "min_x": float(np.min(combined[:, 0])),
            "max_x": float(np.max(combined[:, 0])),
            "min_y": float(np.min(combined[:, 1])),
            "max_y": float(np.max(combined[:, 1])),
        },
    }


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Go2 URDF + 步态投影 Footprint 校准")
    parser.add_argument("--stand-seconds", type=float, default=4.0)
    parser.add_argument("--motion-seconds", type=float, default=6.0)
    parser.add_argument("--warmup-seconds", type=float, default=1.0)
    parser.add_argument("--sample-hz", type=float, default=10.0)
    parser.add_argument("--command-hz", type=float, default=20.0)
    parser.add_argument("--forward-speed", type=float, default=0.15)
    parser.add_argument("--turn-speed", type=float, default=0.30)
    parser.add_argument("--lateral-speed", type=float, default=0.10)
    parser.add_argument(
        "--verify-only",
        action="store_true",
        help="锁停后只验证当前参数与 local/global 发布足迹，不执行步态采样",
    )
    parser.add_argument(
        "--verification-tolerance",
        type=float,
        default=0.005,
        help="发布足迹逐顶点允许误差，默认 0.005 m",
    )
    parser.add_argument("--output-dir", type=Path)
    return parser


def _validated_arguments(arguments: argparse.Namespace) -> None:
    positive = (
        arguments.stand_seconds,
        arguments.motion_seconds,
        arguments.sample_hz,
        arguments.command_hz,
        arguments.forward_speed,
        arguments.turn_speed,
        arguments.lateral_speed,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in positive):
        raise ValueError("时长、频率和速度必须为正有限数")
    if not math.isfinite(arguments.warmup_seconds) or arguments.warmup_seconds < 0.0:
        raise ValueError("warmup-seconds 必须为非负有限数")
    if (
        not math.isfinite(arguments.verification_tolerance)
        or arguments.verification_tolerance <= 0.0
    ):
        raise ValueError("verification-tolerance 必须为正有限数")
    if arguments.forward_speed > 0.27:
        raise ValueError("forward-speed 不得超过 forward_rpp 0.27 m/s 上限")
    if arguments.lateral_speed > 0.15:
        raise ValueError("lateral-speed 不得超过安全速度链 0.15 m/s 上限")
    if arguments.turn_speed > 0.45:
        raise ValueError("turn-speed 不得超过安全速度链 0.45 rad/s 上限")


def main(argv: Sequence[str] | None = None) -> int:
    arguments = create_parser().parse_args(argv)
    try:
        _validated_arguments(arguments)
    except ValueError as error:
        print(f"参数错误：{error}")
        return 2
    run_id = time.strftime("footprint_%Y%m%d_%H%M%S")
    output_dir = arguments.output_dir or (
        package_root_from_module() / "logs" / "footprint" / run_id
    )
    rclpy.init(args=None)
    node = FootprintCalibratorNode(arguments)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    result_code = 0
    try:
        urdf = node.robot_description()
        shapes = parse_collision_shapes(urdf)
        shapes_by_link: dict[str, list[CollisionShape]] = {}
        for shape in shapes:
            shapes_by_link.setdefault(shape.link, []).append(shape)
        geometry = node.costmap_geometry()
        resolution = float(geometry["local"]["resolution"])
        print(
            f"已读取 {len(shapes)} 个 collision/{len(shapes_by_link)} 个 link；"
            f"costmap resolution={resolution:.3f} m",
            flush=True,
        )

        # 先取消任何旧目标并确认零速，再恢复输入以允许
        # 本工具通过 /cmd_vel_teleop 驱动；不绕过 mux/smoother/collision monitor。
        node.trigger("/navigation/stop")
        node.wait_for_pause(True)
        node.command_for((0.0, 0.0, 0.0), 0.5)
        node.wait_for_zero_velocity()
        if arguments.verify_only:
            verification = node.verify_published_footprints(
                geometry, arguments.verification_tolerance
            )
            verification.update({
                "run_id": run_id,
                "mode": "verify_only",
                "parameters": {
                    scope: {
                        "footprint": geometry[scope]["footprint"],
                        "footprint_padding": geometry[scope]["padding"],
                        "resolution": geometry[scope]["resolution"],
                    }
                    for scope in ("local", "global")
                },
            })
            if arguments.output_dir:
                output_dir.mkdir(parents=True, exist_ok=True)
                (output_dir / "published_footprint_verification.json").write_text(
                    json.dumps(verification, ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
            print(json.dumps(verification, ensure_ascii=False, indent=2))
            if verification["status"] != "PASS":
                result_code = 2
            raise _VerificationFinished
        node.trigger("/navigation/resume")
        node.wait_for_pause(False)

        cases = (
            ("stand", (0.0, 0.0, 0.0), arguments.stand_seconds),
            ("forward", (arguments.forward_speed, 0.0, 0.0), arguments.motion_seconds),
            ("turn", (0.0, 0.0, arguments.turn_speed), arguments.motion_seconds),
            ("lateral", (0.0, arguments.lateral_speed, 0.0), arguments.motion_seconds),
        )
        all_points: list[np.ndarray] = []
        all_link_points = {link: [] for link in shapes_by_link}
        transform_rows: list[TransformRecord] = []
        sample_rows: list[SampleRecord] = []
        case_results: dict[str, Any] = {}
        for name, command, duration in cases:
            points, link_points, transforms, samples = node.run_case(
                name, command, duration, shapes_by_link
            )
            all_points.extend(points)
            for link, values in link_points.items():
                all_link_points[link].extend(values)
            transform_rows.extend(transforms)
            sample_rows.extend(samples)
            case_results[name] = _case_summary(points)
            node.command_for((0.0, 0.0, 0.0), 1.0)

        statistics = calibration_statistics(all_points, resolution)
        raw_hull = statistics["raw_hull"]
        recommended = footprint_string(raw_hull)
        link_results = {
            link: _case_summary(values) for link, values in all_link_points.items()
        }
        result = {
            "run_id": run_id,
            "status": "complete",
            "coordinate_frame": "base_footprint",
            "urdf_sha256": hashlib.sha256(urdf.encode("utf-8")).hexdigest(),
            "collision_count": len(shapes),
            "collision_links": sorted(shapes_by_link),
            "collision_sources": [asdict(shape) for shape in shapes],
            "current_geometry": geometry,
            "cases": case_results,
            "per_link": link_results,
            "statistics": statistics,
            "recommended_footprint": recommended,
            "recommended_vertices": json.loads(recommended),
            "sample_count": len(sample_rows),
            "transform_row_count": len(transform_rows),
        }
        output_dir.mkdir(parents=True, exist_ok=True)
        _write_csv(
            output_dir / "footprint_samples.csv",
            [asdict(row) for row in sample_rows],
        )
        _write_csv(
            output_dir / "footprint_transforms.csv",
            [asdict(row) for row in transform_rows],
        )
        (output_dir / "footprint_calibration_result.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        print(json.dumps({
            "recommended_footprint": recommended,
            "recommended_padding_m": statistics["recommended_padding_m"],
            "statistical_tail_m": statistics["statistical_tail_m"],
            "half_costmap_cell_m": statistics["half_costmap_cell_m"],
            "samples": len(sample_rows),
        }, ensure_ascii=False, indent=2))
        print(f"结果目录：{output_dir}")
    except _VerificationFinished:
        pass
    except (RuntimeError, TimeoutError, TransformException, ValueError) as error:
        print(f"校准失败：{error}")
        result_code = 2
    finally:
        try:
            node.command_for((0.0, 0.0, 0.0), 1.0)
            node.trigger("/navigation/stop")
            node.wait_for_pause(True)
            node.wait_for_zero_velocity()
            print("导航已锁停；校准后不自动恢复，也不续行旧目标。")
        except (RuntimeError, TimeoutError) as error:
            print(f"停车警告：{error}")
            result_code = 2
        executor.shutdown(timeout_sec=5.0)
        spin_thread.join(timeout=5.0)
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()
    return result_code


if __name__ == "__main__":
    raise SystemExit(main())
