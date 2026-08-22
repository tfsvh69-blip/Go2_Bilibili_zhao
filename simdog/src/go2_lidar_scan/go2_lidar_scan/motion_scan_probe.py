#!/usr/bin/env python3
"""只读比较倾斜切片与重力对齐切片，并保存运动期量化证据。"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
import time
from typing import Optional, Sequence

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterEvent, ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_parameter_events
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

from go2_lidar_scan.level_frame_publisher import quaternion_rpy


@dataclass(frozen=True)
class ProjectionMetrics:
    """一次二维投影的有限端点和地面端点数量。"""

    finite_bins: int
    ground_bins: int


def quaternion_rotation_matrix(quaternion) -> np.ndarray:
    """把 ROS 四元数转换为右手系 3×3 旋转矩阵。"""
    values = np.asarray(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(values)):
        raise ValueError("四元数包含非有限值")
    norm = float(np.linalg.norm(values))
    if norm <= 1.0e-12:
        raise ValueError("四元数模长为零")
    x, y, z, w = values / norm
    return np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ])


def sensor_and_level_points(
    points_sensor: np.ndarray,
    sensor_transform,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, tuple[float, float, float]]:
    """返回原始点、重力对齐点、base 高度和原始 r/p/y。"""
    points_sensor = np.asarray(points_sensor, dtype=np.float64).reshape((-1, 3))
    rotation_sensor = quaternion_rotation_matrix(
        sensor_transform.transform.rotation)
    translation = np.asarray([
        sensor_transform.transform.translation.x,
        sensor_transform.transform.translation.y,
        sensor_transform.transform.translation.z,
    ], dtype=np.float64)
    if not np.all(np.isfinite(translation)):
        raise ValueError("雷达平移包含非有限值")
    roll, pitch, yaw = quaternion_rpy(sensor_transform.transform.rotation)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotation_level = np.asarray([
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ])
    points_base = points_sensor @ rotation_sensor.T + translation
    # 行向量表达下，R^-1 等于右乘 R。
    points_level = (points_base - translation) @ rotation_level
    return points_sensor, points_level, points_base[:, 2], (roll, pitch, yaw)


def project_nearest_bins(
    points: np.ndarray,
    ground_z_base: np.ndarray,
    *,
    min_height: float = 0.20,
    max_height: float = 0.30,
    angle_min: float = -3.14159,
    angle_max: float = 3.14159,
    angle_increment: float = 0.00872665,
    range_min: float = 0.90,
    range_max: float = 15.0,
    ground_max_z: float = 0.12,
) -> ProjectionMetrics:
    """复现上游“每个角度格取最近点”的核心选择语义。"""
    points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    ground_z_base = np.asarray(ground_z_base, dtype=np.float64).reshape((-1,))
    if len(points) != len(ground_z_base):
        raise ValueError("点数量与 base 高度数量不一致")
    if len(points) == 0:
        return ProjectionMetrics(0, 0)

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    range_sq = x * x + y * y
    angles = np.arctan2(y, x)
    valid = (
        np.all(np.isfinite(points), axis=1)
        & np.isfinite(ground_z_base)
        & (z >= min_height)
        & (z <= max_height)
        & (range_sq >= range_min * range_min)
        & (range_sq <= range_max * range_max)
        & (angles >= angle_min)
        & (angles <= angle_max)
    )
    if not np.any(valid):
        return ProjectionMetrics(0, 0)

    ranges = np.sqrt(range_sq[valid])
    bins = ((angles[valid] - angle_min) / angle_increment).astype(np.int64)
    ground = ground_z_base[valid]
    bin_count = int(math.ceil((angle_max - angle_min) / angle_increment))
    within = (bins >= 0) & (bins < bin_count)
    bins = bins[within]
    ranges = ranges[within]
    ground = ground[within]
    if len(bins) == 0:
        return ProjectionMetrics(0, 0)

    order = np.lexsort((ranges, bins))
    sorted_bins = bins[order]
    first = np.concatenate((
        np.asarray([True]),
        sorted_bins[1:] != sorted_bins[:-1],
    ))
    winner_indices = order[first]
    winner_ground = ground[winner_indices]
    return ProjectionMetrics(
        finite_bins=int(len(winner_indices)),
        ground_bins=int(np.count_nonzero(winner_ground <= ground_max_z)),
    )


def _point_array(message: PointCloud2) -> np.ndarray:
    # VLP-16 PointCloud2 同时包含 float32 坐标和其他类型字段；Humble 的
    # read_points_numpy 即使只请求 xyz 也会错误要求全部字段同类型。
    values = point_cloud2.read_points(
        message, field_names=["x", "y", "z"], skip_nans=True)
    array = np.asarray(values)
    if array.dtype.names:
        array = np.column_stack([array[name] for name in ("x", "y", "z")])
    return np.asarray(array, dtype=np.float64).reshape((-1, 3))


def _topic_hz(times: Sequence[float]) -> float:
    if len(times) < 2:
        return 0.0
    span = times[-1] - times[0]
    return (len(times) - 1) / span if span > 0.0 else 0.0


def _angle_delta(current: float, previous: float) -> float:
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


class MotionScanProbe(Node):
    """被动采集运动期点云、扫描和 TF，不发布任何控制命令。"""

    def __init__(self, arguments) -> None:
        super().__init__("go2_motion_scan_probe")
        self.arguments = arguments
        self.tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        sensor_qos = rclpy.qos.qos_profile_sensor_data
        self.create_subscription(
            PointCloud2, arguments.cloud_topic, self._cloud_callback, sensor_qos)
        self.create_subscription(
            LaserScan, arguments.scan_topic, self._scan_callback, sensor_qos)
        self.create_subscription(
            LaserScan, arguments.raw_scan_topic,
            self._raw_scan_callback, sensor_qos)
        self.create_subscription(
            ParameterEvent,
            "/parameter_events",
            self._parameter_event_callback,
            qos_profile_parameter_events,
        )
        self.create_timer(0.05, self._sample_map_odom)

        self.min_height = arguments.min_height
        self.max_height = arguments.max_height
        height_service = (
            arguments.converter_node.rstrip("/") + "/get_parameters")
        self.height_parameter_client = self.create_client(
            GetParameters, height_service)
        self.height_sync_future = None
        self.create_timer(0.5, self._request_height_parameters)

        self.cloud_count = 0
        self.cloud_failures = 0
        self.rows: list[dict] = []
        self.scan_times: list[float] = []
        self.raw_scan_times: list[float] = []
        self.scan_frame_mismatches = 0
        self.scan_tf_attempts = 0
        self.scan_tf_failures = 0
        self.scan_level_tilt_max_deg = 0.0
        self.map_odom_previous: Optional[tuple[float, float, float]] = None
        self.map_odom_max_xy_step_m = 0.0
        self.map_odom_max_yaw_step_rad = 0.0
        self.map_odom_samples = 0
        self.get_logger().info(
            "只读运动扫描探针已启动；当前高度窗口 [%.3f, %.3f] m；"
            "请通过 /cmd_vel_teleop 驱动机器狗"
            % (self.min_height, self.max_height)
        )

    def _request_height_parameters(self) -> None:
        """探针晚启动时，主动读取转换器当前值，避免漏掉早先的参数事件。"""
        if self.height_sync_future is not None:
            return
        if not self.height_parameter_client.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = ["min_height", "max_height"]
        self.height_sync_future = self.height_parameter_client.call_async(
            request)
        self.height_sync_future.add_done_callback(
            self._height_parameters_response)

    def _height_parameters_response(self, future) -> None:
        try:
            values = future.result().values
            proposed_min = values[0].double_value
            proposed_max = values[1].double_value
            if proposed_min < proposed_max:
                self.min_height = proposed_min
                self.max_height = proposed_max
                self.get_logger().info(
                    "已从转换器同步高度窗口 [%.3f, %.3f] m"
                    % (self.min_height, self.max_height)
                )
        except (AttributeError, IndexError, RuntimeError) as error:
            self.get_logger().warning("同步转换器高度参数失败：%s" % error)
            self.height_sync_future = None

    def _parameter_event_callback(self, event: ParameterEvent) -> None:
        """记录正式转换器实际接受的高度参数，保证证据与 rqt 同步。"""
        if event.node.rstrip("/") != self.arguments.converter_node.rstrip("/"):
            return
        proposed_min = self.min_height
        proposed_max = self.max_height
        changed = False
        for parameter in (*event.new_parameters, *event.changed_parameters):
            if parameter.value.type != ParameterType.PARAMETER_DOUBLE:
                continue
            if parameter.name == "min_height":
                proposed_min = parameter.value.double_value
                changed = True
            elif parameter.name == "max_height":
                proposed_max = parameter.value.double_value
                changed = True
        if changed and proposed_min < proposed_max:
            self.min_height = proposed_min
            self.max_height = proposed_max
            self.get_logger().info(
                "探针切换到高度窗口 [%.3f, %.3f] m"
                % (self.min_height, self.max_height)
            )

    def _cloud_callback(self, message: PointCloud2) -> None:
        self.cloud_count += 1
        if (self.cloud_count - 1) % self.arguments.sample_every != 0:
            return
        sensor_frame = message.header.frame_id.strip()
        if not sensor_frame:
            self.cloud_failures += 1
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.arguments.reference_frame,
                sensor_frame,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=self.arguments.tf_timeout),
            )
            points = _point_array(message)
            raw_points, level_points, ground_z, rpy = sensor_and_level_points(
                points, transform)
            raw = project_nearest_bins(
                raw_points, ground_z,
                min_height=-0.05,
                max_height=0.10,
                ground_max_z=self.arguments.ground_max_z)
            level = project_nearest_bins(
                level_points, ground_z,
                min_height=self.min_height,
                max_height=self.max_height,
                ground_max_z=self.arguments.ground_max_z)
            self.rows.append({
                "stamp_sec": (
                    float(message.header.stamp.sec)
                    + float(message.header.stamp.nanosec) / 1e9
                ),
                "point_count": int(len(points)),
                "roll_deg": math.degrees(rpy[0]),
                "pitch_deg": math.degrees(rpy[1]),
                "tilt_deg": max(abs(math.degrees(rpy[0])),
                                abs(math.degrees(rpy[1]))),
                "raw_finite_bins": raw.finite_bins,
                "raw_ground_bins": raw.ground_bins,
                "level_finite_bins": level.finite_bins,
                "level_ground_bins": level.ground_bins,
                "min_height_m": self.min_height,
                "max_height_m": self.max_height,
            })
        except (TransformException, ValueError) as error:
            self.cloud_failures += 1
            self.get_logger().warning(
                "点云分析跳过：%s" % error,
                throttle_duration_sec=1.0,
            )

    def _scan_callback(self, message: LaserScan) -> None:
        self.scan_times.append(time.monotonic())
        if message.header.frame_id != self.arguments.level_frame:
            self.scan_frame_mismatches += 1
        self.scan_tf_attempts += 1
        try:
            transform = self.tf_buffer.lookup_transform(
                self.arguments.reference_frame,
                self.arguments.level_frame,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=self.arguments.tf_timeout),
            )
            roll, pitch, _yaw = quaternion_rpy(transform.transform.rotation)
            self.scan_level_tilt_max_deg = max(
                self.scan_level_tilt_max_deg,
                abs(math.degrees(roll)),
                abs(math.degrees(pitch)),
            )
        except (TransformException, ValueError):
            self.scan_tf_failures += 1

    def _raw_scan_callback(self, _message: LaserScan) -> None:
        self.raw_scan_times.append(time.monotonic())

    def _sample_map_odom(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "odom", Time())
            _roll, _pitch, yaw = quaternion_rpy(transform.transform.rotation)
            current = (
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                yaw,
            )
            if self.map_odom_previous is not None:
                previous = self.map_odom_previous
                self.map_odom_max_xy_step_m = max(
                    self.map_odom_max_xy_step_m,
                    math.hypot(current[0] - previous[0],
                               current[1] - previous[1]),
                )
                self.map_odom_max_yaw_step_rad = max(
                    self.map_odom_max_yaw_step_rad,
                    abs(_angle_delta(current[2], previous[2])),
                )
            self.map_odom_previous = current
            self.map_odom_samples += 1
        except (TransformException, ValueError):
            return

    def result(self) -> dict:
        max_tilt = max((row["tilt_deg"] for row in self.rows), default=0.0)
        raw_ground_frames = sum(row["raw_ground_bins"] > 0 for row in self.rows)
        level_ground_frames = sum(
            row["level_ground_bins"] > 0 for row in self.rows)
        raw_ground_total = sum(row["raw_ground_bins"] for row in self.rows)
        level_ground_total = sum(row["level_ground_bins"] for row in self.rows)
        scan_hz = _topic_hz(self.scan_times)
        raw_scan_hz = _topic_hz(self.raw_scan_times)
        tf_success_rate = (
            (self.scan_tf_attempts - self.scan_tf_failures)
            / self.scan_tf_attempts
            if self.scan_tf_attempts else 0.0
        )
        failures = []
        warnings = []
        incomplete = []
        if not self.rows:
            failures.append("没有得到可分析的点云帧")
        if max_tilt < self.arguments.minimum_stress_tilt_deg:
            incomplete.append(
                "最大机身倾斜 %.2f°，未达到压力门 %.2f°"
                % (max_tilt, self.arguments.minimum_stress_tilt_deg)
            )
        if level_ground_total:
            failures.append(
                "重力对齐扫描仍有 %d 个地面获胜端点" % level_ground_total)
        if scan_hz < self.arguments.minimum_scan_rate:
            failures.append(
                "/scan 频率 %.2f Hz 低于 %.2f Hz"
                % (scan_hz, self.arguments.minimum_scan_rate))
        if self.scan_frame_mismatches:
            failures.append(
                "%d 帧 /scan 的 frame_id 不是 %s"
                % (self.scan_frame_mismatches, self.arguments.level_frame))
        if self.scan_tf_failures:
            failures.append(
                "%d/%d 帧 /scan 缺少同时间戳 TF"
                % (self.scan_tf_failures, self.scan_tf_attempts))
        if self.scan_level_tilt_max_deg > self.arguments.level_tilt_tolerance_deg:
            failures.append(
                "velodyne_level 最大倾斜 %.3f° 超过 %.3f°"
                % (self.scan_level_tilt_max_deg,
                   self.arguments.level_tilt_tolerance_deg))
        if self.arguments.require_raw and not self.raw_scan_times:
            failures.append("要求 /scan_raw，但没有收到消息")
        if raw_ground_total == 0:
            warnings.append("原始切片本次未复现地面端点，A/B 对照证据不足")
        if self.map_odom_samples and (
            self.map_odom_max_xy_step_m > 0.10
            or self.map_odom_max_yaw_step_rad > 0.10
        ):
            failures.append("map→odom 最大单步修正超过 0.10 m/rad")

        status = "FAIL" if failures else ("INCOMPLETE" if incomplete else "PASS")
        finite_level = [row["level_finite_bins"] for row in self.rows]
        return {
            "status": status,
            "read_only": True,
            "sampled_cloud_frames": len(self.rows),
            "cloud_tf_failures": self.cloud_failures,
            "maximum_body_tilt_deg": max_tilt,
            "raw_ground_frames": raw_ground_frames,
            "raw_ground_bins_total": raw_ground_total,
            "level_ground_frames": level_ground_frames,
            "level_ground_bins_total": level_ground_total,
            "level_finite_bins_median": (
                statistics.median(finite_level) if finite_level else None),
            "height_slice_start_m": self.min_height,
            "height_slice_end_m": self.max_height,
            "scan_hz": scan_hz,
            "raw_scan_hz": raw_scan_hz,
            "scan_frame_mismatches": self.scan_frame_mismatches,
            "scan_tf_success_rate": tf_success_rate,
            "level_frame_max_tilt_deg": self.scan_level_tilt_max_deg,
            "map_odom_samples": self.map_odom_samples,
            "map_odom_max_xy_step_m": self.map_odom_max_xy_step_m,
            "map_odom_max_yaw_step_rad": self.map_odom_max_yaw_step_rad,
            "failures": failures,
            "warnings": warnings,
            "incomplete_reasons": incomplete,
        }


def package_root_from_module() -> Path:
    """返回源码包根；symlink-install 下仍会落回当前项目。"""
    return Path(__file__).resolve().parents[1]


def write_results(output_dir: Path, rows: list[dict], result: dict) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0]) if rows else []
    csv_path = output_dir / "motion_scan_frames.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        if fieldnames:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
    (output_dir / "motion_scan_result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--sample-every", type=int, default=1)
    parser.add_argument("--output-dir", type=Path)
    # 专用抽样点云仍保留原始 velodyne frame/xyz，但只在同时间戳
    # 水平 TF 已发布后以默认 1/5 频率出现；不会让每帧内部大点云
    # 退化为 DDS 跨进程拷贝。
    parser.add_argument(
        "--cloud-topic", default="/go2_lidar_scan/probe_cloud")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--raw-scan-topic", default="/scan_raw")
    parser.add_argument("--reference-frame", default="base_footprint")
    parser.add_argument("--level-frame", default="velodyne_level")
    parser.add_argument(
        "--converter-node", default="/go2_lidar_scan_converter")
    parser.add_argument("--min-height", type=float, default=0.20)
    parser.add_argument("--max-height", type=float, default=0.30)
    parser.add_argument("--ground-max-z", type=float, default=0.12)
    parser.add_argument("--minimum-stress-tilt-deg", type=float, default=3.0)
    parser.add_argument("--level-tilt-tolerance-deg", type=float, default=0.10)
    parser.add_argument("--minimum-scan-rate", type=float, default=7.0)
    parser.add_argument("--tf-timeout", type=float, default=0.05)
    parser.add_argument("--require-raw", action="store_true")
    return parser


def _validate_arguments(arguments) -> None:
    positive = (
        arguments.duration,
        arguments.ground_max_z,
        arguments.minimum_stress_tilt_deg,
        arguments.level_tilt_tolerance_deg,
        arguments.minimum_scan_rate,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in positive):
        raise ValueError("时长和各阈值必须为正有限数")
    if arguments.sample_every <= 0:
        raise ValueError("sample-every 必须为正整数")
    if not math.isfinite(arguments.tf_timeout) or arguments.tf_timeout < 0.0:
        raise ValueError("tf-timeout 必须为非负有限数")
    if (
        not math.isfinite(arguments.min_height)
        or not math.isfinite(arguments.max_height)
        or arguments.min_height >= arguments.max_height
    ):
        raise ValueError("必须满足有限的 min-height < max-height")


def main(args=None) -> None:
    arguments, ros_args = create_parser().parse_known_args(args=args)
    try:
        _validate_arguments(arguments)
    except ValueError as error:
        raise SystemExit("参数错误：%s" % error) from error

    run_id = time.strftime("motion_scan_%Y%m%d_%H%M%S")
    output_dir = arguments.output_dir or (
        package_root_from_module() / "logs" / run_id)
    rclpy.init(args=ros_args)
    node = MotionScanProbe(arguments)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    deadline = time.monotonic() + arguments.duration
    interrupted = False
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        interrupted = True
    except RuntimeError:
        if rclpy.ok():
            raise
        interrupted = True
    finally:
        result = node.result()
        if interrupted:
            result["warnings"].append("用户提前结束采样")
        write_results(output_dir, node.rows, result)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        print("结果目录：%s" % output_dir)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if result["status"] == "FAIL":
        raise SystemExit(1)
    if result["status"] == "INCOMPLETE":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
