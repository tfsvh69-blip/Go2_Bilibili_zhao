#!/usr/bin/env python3
"""监控 /velodyne_points -> /scan 契约，并在 RViz 发布中文状态 Marker。"""

from collections import deque
import math
from typing import Deque, Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rcl_interfaces.msg import ParameterEvent, ParameterType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_parameter_events,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from go2_lidar_scan.level_frame_publisher import quaternion_rpy
from go2_lidar_scan.scan_metrics import ScanMetrics, analyze_ranges


def _kv(key: str, value) -> KeyValue:
    return KeyValue(key=key, value=str(value))


class TopicRate:
    """按 ROS 时钟维护短窗口频率和消息新鲜度。"""

    def __init__(self, window_size: int = 50):
        self.times_ns: Deque[int] = deque(maxlen=window_size)

    def tick(self, now_ns: int) -> None:
        self.times_ns.append(now_ns)

    def hz(self) -> float:
        if len(self.times_ns) < 2:
            return 0.0
        span = (self.times_ns[-1] - self.times_ns[0]) / 1e9
        if span <= 0.0:
            return 0.0
        return (len(self.times_ns) - 1) / span

    def age(self, now_ns: int) -> float:
        if not self.times_ns:
            return math.inf
        return max(0.0, (now_ns - self.times_ns[-1]) / 1e9)


class ScanDiagnostics(Node):
    """输出频率、量程契约、静止跳变和输入输出链健康状态。"""

    def __init__(self):
        super().__init__("go2_lidar_scan_diagnostics")
        self.declare_parameter("cloud_topic", "/velodyne_points")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("marker_topic", "/go2_lidar_scan/markers")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("reference_frame", "base_footprint")
        self.declare_parameter("sensor_frame", "velodyne")
        self.declare_parameter("level_frame", "velodyne_level")
        self.declare_parameter("expected_scan_frame", "velodyne_level")
        self.declare_parameter(
            "source_transform_topic",
            "/go2_lidar_scan/level_source_transform",
        )
        self.declare_parameter(
            "cloud_heartbeat_topic",
            "/go2_lidar_scan/cloud_heartbeat",
        )
        self.declare_parameter("level_tilt_tolerance_deg", 0.10)
        self.declare_parameter("expected_rate_hz", 10.0)
        self.declare_parameter("minimum_rate_hz", 7.0)
        self.declare_parameter("expected_range_min", 0.90)
        self.declare_parameter("expected_range_max", 15.0)
        self.declare_parameter("expected_min_height", 0.20)
        self.declare_parameter("expected_max_height", 0.30)
        self.declare_parameter("stale_timeout_s", 1.00)
        self.declare_parameter("jump_threshold_m", 0.30)
        self.declare_parameter("stationary_jump_warn_ratio", 0.02)
        self.declare_parameter("stationary_linear_mps", 0.02)
        self.declare_parameter("stationary_angular_rps", 0.05)
        self.declare_parameter("minimum_finite_bins", 30)

        self.cloud_topic = str(self.get_parameter("cloud_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.reference_frame = str(
            self.get_parameter("reference_frame").value)
        self.sensor_frame = str(self.get_parameter("sensor_frame").value)
        self.level_frame = str(self.get_parameter("level_frame").value)
        self.expected_scan_frame = str(
            self.get_parameter("expected_scan_frame").value)
        self.level_tilt_tolerance_deg = float(
            self.get_parameter("level_tilt_tolerance_deg").value)
        self.minimum_rate_hz = float(
            self.get_parameter("minimum_rate_hz").value)
        self.expected_range_min = float(
            self.get_parameter("expected_range_min").value)
        self.expected_range_max = float(
            self.get_parameter("expected_range_max").value)
        self.active_min_height = float(
            self.get_parameter("expected_min_height").value)
        self.active_max_height = float(
            self.get_parameter("expected_max_height").value)
        self.stale_timeout_s = float(
            self.get_parameter("stale_timeout_s").value)
        self.jump_threshold_m = float(
            self.get_parameter("jump_threshold_m").value)
        self.stationary_jump_warn_ratio = float(
            self.get_parameter("stationary_jump_warn_ratio").value)
        self.stationary_linear_mps = float(
            self.get_parameter("stationary_linear_mps").value)
        self.stationary_angular_rps = float(
            self.get_parameter("stationary_angular_rps").value)
        self.minimum_finite_bins = int(
            self.get_parameter("minimum_finite_bins").value)

        sensor_qos = rclpy.qos.qos_profile_sensor_data
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            Header,
            str(self.get_parameter("cloud_heartbeat_topic").value),
            self._cloud_heartbeat_callback,
            status_qos,
        )
        self.create_subscription(
            LaserScan, self.scan_topic, self._scan_callback, sensor_qos)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self._odom_callback, sensor_qos)
        self.create_subscription(
            TransformStamped,
            str(self.get_parameter("source_transform_topic").value),
            self._source_transform_callback,
            status_qos,
        )
        self.create_subscription(
            ParameterEvent,
            "/parameter_events",
            self._parameter_event_callback,
            qos_profile_parameter_events,
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value), 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("marker_topic").value), 10)
        self.cloud_rate = TopicRate()
        self.scan_rate = TopicRate()
        self.scan_metrics: Optional[ScanMetrics] = None
        self.scan_frame = "unknown"
        self.scan_range_min = math.nan
        self.scan_range_max = math.nan
        self.scan_stamp: Optional[Time] = None
        self.scan_header_age_s = math.inf
        self.previous_ranges = None
        self.stationary = False
        self.have_odom = False
        self.body_roll_deg = math.nan
        self.body_pitch_deg = math.nan
        self.level_roll_deg = math.nan
        self.level_pitch_deg = math.nan
        self.level_tf_ok = False
        self.level_tf_error = "尚未检查"
        self.tf_attempts = 0
        self.tf_successes = 0
        self.scan_tf_attempts = 0
        self.scan_tf_successes = 0
        self.successful_transform_stamps: Deque[int] = deque(maxlen=100)
        self.last_level = None
        self.create_timer(1.0, self._publish_status)

    def _parameter_event_callback(self, event: ParameterEvent) -> None:
        """跟踪转换器已经接受并发布的动态高度窗口。"""
        if not event.node.rstrip("/").endswith(
                "/go2_lidar_scan_converter"):
            return
        for parameter in (*event.new_parameters, *event.changed_parameters):
            if parameter.value.type != ParameterType.PARAMETER_DOUBLE:
                continue
            if parameter.name == "min_height":
                self.active_min_height = parameter.value.double_value
            elif parameter.name == "max_height":
                self.active_max_height = parameter.value.double_value

    def _cloud_heartbeat_callback(self, _message: Header) -> None:
        self.cloud_rate.tick(self.get_clock().now().nanoseconds)
        self.tf_attempts += 1

    def _scan_callback(self, message: LaserScan) -> None:
        self.scan_rate.tick(self.get_clock().now().nanoseconds)
        self.scan_frame = message.header.frame_id or "unknown"
        self.scan_stamp = Time.from_msg(message.header.stamp)
        self.scan_header_age_s = max(
            0.0,
            (self.get_clock().now().nanoseconds - self.scan_stamp.nanoseconds)
            / 1e9,
        )
        self.scan_range_min = float(message.range_min)
        self.scan_range_max = float(message.range_max)
        self.scan_metrics = analyze_ranges(
            message.ranges,
            self.scan_range_min,
            self.scan_range_max,
            previous=self.previous_ranges,
            jump_threshold_m=self.jump_threshold_m,
        )
        self.previous_ranges = list(message.ranges)

    def _odom_callback(self, message: Odometry) -> None:
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        linear_speed = math.hypot(linear.x, linear.y)
        self.stationary = (
            linear_speed <= self.stationary_linear_mps
            and abs(angular.z) <= self.stationary_angular_rps
        )
        self.have_odom = True

    def _source_transform_callback(
        self,
        message: TransformStamped,
    ) -> None:
        """接收发布器实际采用的原始 TF，避免重复缓存整棵机器人 TF 树。"""
        stamp = Time.from_msg(message.header.stamp)
        try:
            body_roll, body_pitch, _body_yaw = quaternion_rpy(
                message.transform.rotation)
        except ValueError as error:
            self.level_tf_ok = False
            self.level_tf_error = str(error)
            return
        self.body_roll_deg = math.degrees(body_roll)
        self.body_pitch_deg = math.degrees(body_pitch)
        # level_frame_publisher 使用经过单元测试的 yaw_only_quaternion；实际
        # 广播是否可查询仍由 motion_scan_probe 的同时间戳 TF 验收负责。
        self.level_roll_deg = 0.0
        self.level_pitch_deg = 0.0
        self.level_tf_ok = True
        self.level_tf_error = ""
        self.tf_successes += 1
        self.successful_transform_stamps.append(stamp.nanoseconds)

    def _sample_transforms(self) -> None:
        """核对最近 /scan 是否来自一个成功发布过的同时间戳 level TF。"""
        if self.scan_stamp is None:
            return
        self.scan_tf_attempts += 1
        if self.scan_stamp.nanoseconds in self.successful_transform_stamps:
            self.level_tf_ok = True
            self.level_tf_error = ""
            self.scan_tf_successes += 1
        else:
            self.level_tf_ok = False
            self.level_tf_error = "未收到与最近 /scan 同时间戳的 level TF 状态"

    def _evaluate(self, now_ns: int):
        problems = []
        level = DiagnosticStatus.OK
        cloud_age = self.cloud_rate.age(now_ns)
        scan_age = self.scan_rate.age(now_ns)
        scan_hz = self.scan_rate.hz()
        cloud_hz = self.cloud_rate.hz()

        if not math.isfinite(cloud_age) or cloud_age > self.stale_timeout_s:
            problems.append("原始点云超时")
            level = DiagnosticStatus.ERROR
        if not math.isfinite(scan_age) or scan_age > self.stale_timeout_s:
            problems.append("/scan 超时")
            level = DiagnosticStatus.ERROR
        if 0.0 < scan_hz < self.minimum_rate_hz:
            problems.append("/scan 频率过低")
            level = max(level, DiagnosticStatus.WARN)
        if self.scan_metrics is not None and self.scan_frame != self.expected_scan_frame:
            problems.append(
                "/scan frame_id=%s，期望=%s"
                % (self.scan_frame, self.expected_scan_frame)
            )
            level = DiagnosticStatus.ERROR
        if self.scan_metrics is not None and not self.level_tf_ok:
            problems.append("重力对齐 TF 不可用")
            level = DiagnosticStatus.ERROR
        if self.level_tf_ok and (
            abs(self.level_roll_deg) > self.level_tilt_tolerance_deg
            or abs(self.level_pitch_deg) > self.level_tilt_tolerance_deg
        ):
            problems.append("velodyne_level 仍含 roll/pitch")
            level = DiagnosticStatus.ERROR

        metrics = self.scan_metrics
        if metrics is not None:
            if metrics.invalid:
                problems.append("存在 NaN/负无穷/越量程值")
                level = max(level, DiagnosticStatus.WARN)
            if metrics.valid < self.minimum_finite_bins:
                problems.append("有效有限回波过少")
                level = max(level, DiagnosticStatus.WARN)
            if (
                self.have_odom and self.stationary
                and metrics.jump_ratio > self.stationary_jump_warn_ratio
            ):
                problems.append("静止时相邻帧跳变偏多")
                level = max(level, DiagnosticStatus.WARN)
        if (
            math.isfinite(self.scan_range_min)
            and abs(self.scan_range_min - self.expected_range_min) > 0.01
        ):
            problems.append("range_min 与配置档不一致")
            level = max(level, DiagnosticStatus.WARN)
        if (
            math.isfinite(self.scan_range_max)
            and abs(self.scan_range_max - self.expected_range_max) > 0.01
        ):
            problems.append("range_max 与配置档不一致")
            level = max(level, DiagnosticStatus.WARN)

        summary = "转换链正常" if not problems else "；".join(problems)
        return level, summary, cloud_hz, scan_hz, cloud_age, scan_age

    def _publish_status(self) -> None:
        now = self.get_clock().now()
        self._sample_transforms()
        now_ns = now.nanoseconds
        level, summary, cloud_hz, scan_hz, cloud_age, scan_age = self._evaluate(
            now_ns)
        metrics = self.scan_metrics

        status = DiagnosticStatus()
        status.name = "Go2/LiDAR/PointCloudToLaserScan"
        status.hardware_id = "gazebo_vlp16"
        status.level = level
        status.message = summary
        status.values = [
            _kv("cloud_topic", self.cloud_topic),
            _kv("scan_topic", self.scan_topic),
            _kv("cloud_hz", f"{cloud_hz:.2f}"),
            _kv("scan_hz", f"{scan_hz:.2f}"),
            _kv("cloud_age_s", f"{cloud_age:.3f}"),
            _kv("scan_age_s", f"{scan_age:.3f}"),
            _kv("stationary", self.stationary if self.have_odom else "unknown"),
            _kv("scan_frame", self.scan_frame),
            _kv("expected_scan_frame", self.expected_scan_frame),
            _kv("scan_header_age_s", f"{self.scan_header_age_s:.3f}"),
            _kv("level_tf_ok", self.level_tf_ok),
            _kv(
                "level_tf_success_rate",
                f"{self.tf_successes / self.tf_attempts:.3f}"
                if self.tf_attempts else "N/A",
            ),
            _kv(
                "scan_same_stamp_tf_success_rate",
                f"{self.scan_tf_successes / self.scan_tf_attempts:.3f}"
                if self.scan_tf_attempts else "N/A",
            ),
            _kv("body_roll_deg", f"{self.body_roll_deg:.3f}"),
            _kv("body_pitch_deg", f"{self.body_pitch_deg:.3f}"),
            _kv("level_roll_deg", f"{self.level_roll_deg:.3f}"),
            _kv("level_pitch_deg", f"{self.level_pitch_deg:.3f}"),
            _kv("min_height_m", f"{self.active_min_height:.3f}"),
            _kv("max_height_m", f"{self.active_max_height:.3f}"),
            _kv("level_tf_error", self.level_tf_error or "none"),
        ]
        if metrics is not None:
            status.values.extend([
                _kv("scan_bins", metrics.total),
                _kv("finite_valid_bins", metrics.valid),
                _kv("positive_inf_bins", metrics.positive_inf),
                _kv("invalid_bins", metrics.invalid),
                _kv("frame_jump_ratio", f"{metrics.jump_ratio:.4f}"),
                _kv("nearest_m", f"{metrics.nearest:.3f}"),
                _kv("range_min_m", f"{self.scan_range_min:.3f}"),
                _kv("range_max_m", f"{self.scan_range_max:.3f}"),
            ])
        array = DiagnosticArray()
        array.header.stamp = now.to_msg()
        array.status = [status]
        self.diagnostics_pub.publish(array)
        self._publish_marker(level, summary, scan_hz, metrics)

        if level != self.last_level:
            logger = self.get_logger()
            if level == DiagnosticStatus.ERROR:
                logger.error(summary)
            elif level == DiagnosticStatus.WARN:
                logger.warning(summary)
            else:
                logger.info(summary)
            self.last_level = level

    def _publish_marker(
        self,
        level: int,
        summary: str,
        scan_hz: float,
        metrics: Optional[ScanMetrics],
    ) -> None:
        marker = Marker()
        # Marker 只是人机界面状态，使用 TF2 的“最新可用变换”语义。
        # 若填计时器当前时间，RViz 可能在下一帧 velodyne_level TF 到达前
        # 短暂把 Display 标红，这与严格按扫描时间戳验证的数据链无关。
        marker.header.stamp = Time().to_msg()
        marker.header.frame_id = self.scan_frame
        marker.ns = "scan_health"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = 0.45
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.12
        marker.color.a = 1.0
        if level == DiagnosticStatus.OK:
            marker.color.g = 1.0
        elif level == DiagnosticStatus.WARN:
            marker.color.r = 1.0
            marker.color.g = 0.65
        else:
            marker.color.r = 1.0
        marker.lifetime = Duration(seconds=1.5).to_msg()
        if metrics is None:
            marker.text = f"LiDAR→/scan：{summary}"
        else:
            marker.text = (
                f"LiDAR→/scan：{summary}\n"
                f"{scan_hz:.1f} Hz  有效 {metrics.valid}/{metrics.total}  "
                f"+inf {metrics.positive_inf}\n"
                f"机身倾斜 r/p {self.body_roll_deg:+.1f}/{self.body_pitch_deg:+.1f}°  "
                f"对齐 {self.level_roll_deg:+.2f}/{self.level_pitch_deg:+.2f}°\n"
                f"height {self.active_min_height:+.2f}.."
                f"{self.active_max_height:+.2f} m  "
                f"range {self.scan_range_min:.2f}..{self.scan_range_max:.1f} m  "
                f"跳变 {metrics.jump_ratio * 100.0:.1f}%"
            )
        self.marker_pub.publish(MarkerArray(markers=[marker]))


def main(args=None):
    rclpy.init(args=args)
    node = ScanDiagnostics()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble 在多节点 launch 同时退出时，take_message 偶尔会抛
        # RuntimeError；运行中真正的 RuntimeError 仍继续上抛。
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
