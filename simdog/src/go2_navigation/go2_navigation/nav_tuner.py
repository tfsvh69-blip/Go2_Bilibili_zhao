"""Go2 Nav2 运行时安全调参与监控入口。"""

from __future__ import annotations

import argparse
import csv
import curses
import json
import math
import shlex
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PolygonStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Path as PathMessage
from rcl_interfaces.srv import DescribeParameters, GetParameters, SetParametersAtomically
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from go2_navigation.nav_tuning import (
    Capability,
    PROFILES,
    REGISTRY,
    ParameterSpec,
    RateTracker,
    equivalent,
    get_path,
    laser_counts,
    package_root_from_module,
    parse_value,
    path_length,
    persist_values,
)


MANAGED_LIFECYCLE_NODES = (
    "/controller_server",
    "/smoother_server",
    "/planner_server",
    "/behavior_server",
    "/bt_navigator",
    "/waypoint_follower",
    "/velocity_smoother",
    "/collision_monitor",
)


def _fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return "N/A"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


class RuntimeMonitor(Node):
    """只订阅标准 Nav2 接口，并通过标准参数/Lifecycle 服务实施变更。"""

    def __init__(self) -> None:
        super().__init__("nav_tuner")
        self._lock = threading.RLock()
        self._rates = {
            "scan": RateTracker(),
            "d435": RateTracker(),
            "local_costmap": RateTracker(),
            "global_costmap": RateTracker(),
            "plan": RateTracker(),
            "cmd_vel_nav": RateTracker(),
            "cmd_vel_switched": RateTracker(),
            "cmd_vel_smoothed": RateTracker(),
            "cmd_vel": RateTracker(),
        }
        self._scan: dict[str, Any] = {}
        self._costmaps: dict[str, dict[str, Any]] = {}
        self._footprints: dict[str, dict[str, Any]] = {}
        self._plan: dict[str, Any] = {}
        self._commands: dict[str, tuple[float, float, float]] = {}
        self._pause_navigation: bool | None = None
        self._last_effect: dict[str, str] = {}
        self._controller_config: dict[str, Any] = {}
        self._dirty: dict[str, Any] = {}
        self._messages: deque[str] = deque(maxlen=8)
        self._service_clients: dict[str, Any] = {}
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(LaserScan, "/scan", self._on_scan, sensor_qos)
        self.create_subscription(
            PointCloud2, "/depth/color/points", self._on_d435, sensor_qos
        )
        self.create_subscription(
            Costmap,
            "/local_costmap/costmap_raw",
            lambda msg: self._on_costmap("local", msg),
            costmap_qos,
        )
        self.create_subscription(
            Costmap,
            "/global_costmap/costmap_raw",
            lambda msg: self._on_costmap("global", msg),
            costmap_qos,
        )
        self.create_subscription(
            PolygonStamped,
            "/local_costmap/published_footprint",
            lambda msg: self._on_footprint("local", msg),
            costmap_qos,
        )
        self.create_subscription(
            PolygonStamped,
            "/global_costmap/published_footprint",
            lambda msg: self._on_footprint("global", msg),
            costmap_qos,
        )
        self.create_subscription(PathMessage, "/plan", self._on_plan, 10)
        for name in ("cmd_vel_nav", "cmd_vel_switched", "cmd_vel_smoothed", "cmd_vel"):
            self.create_subscription(
                Twist, f"/{name}", lambda msg, key=name: self._on_cmd(key, msg), 10
            )
        pause_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            # safety_supervisor 使用 volatile；请求 transient_local 会造成 DDS
            # 不兼容，反而读不到当前是否已经暂停。
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Bool, "/pause_navigation", self._on_pause, pause_qos)

    def log_message(self, message: str) -> None:
        with self._lock:
            self._messages.append(f"{time.strftime('%H:%M:%S')} {message}")

    @property
    def dirty(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._dirty)

    def _on_scan(self, message: LaserScan) -> None:
        now = time.monotonic()
        counts = laser_counts(message.ranges, message.range_min, message.range_max)
        with self._lock:
            self._rates["scan"].add(now)
            self._scan = {
                **counts,
                "range_min": float(message.range_min),
                "range_max": float(message.range_max),
                "frame": message.header.frame_id,
            }

    def _on_d435(self, _: PointCloud2) -> None:
        with self._lock:
            self._rates["d435"].add(time.monotonic())

    def _on_costmap(self, name: str, message: Costmap) -> None:
        now = time.monotonic()
        data = np.asarray(message.data, dtype=np.uint8)
        lethal_indices = np.flatnonzero(data == 254)
        width = int(message.metadata.size_x)
        resolution = float(message.metadata.resolution)
        origin_x = float(message.metadata.origin.position.x)
        origin_y = float(message.metadata.origin.position.y)
        if width > 0 and lethal_indices.size:
            lethal_x = origin_x + (lethal_indices % width + 0.5) * resolution
            lethal_y = origin_y + (lethal_indices // width + 0.5) * resolution
            lethal = np.column_stack((lethal_x, lethal_y))
        else:
            lethal = np.empty((0, 2), dtype=float)
        with self._lock:
            self._rates[f"{name}_costmap"].add(now)
            self._costmaps[name] = {
                "frame": message.header.frame_id,
                "resolution": resolution,
                "size_x": width,
                "size_y": int(message.metadata.size_y),
                "lethal_count": int(np.count_nonzero(data == 254)),
                "inflated_count": int(np.count_nonzero((data >= 1) & (data <= 253))),
                "unknown_count": int(np.count_nonzero(data == 255)),
                "lethal": lethal,
            }

    def _on_footprint(self, name: str, message: PolygonStamped) -> None:
        points = [(float(point.x), float(point.y)) for point in message.polygon.points]
        with self._lock:
            self._footprints[name] = {"frame": message.header.frame_id, "points": points}

    def _on_plan(self, message: PathMessage) -> None:
        now = time.monotonic()
        points = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in message.poses
        ]
        with self._lock:
            self._rates["plan"].add(now)
            self._plan = {
                "frame": message.header.frame_id,
                "points": points,
                "length": path_length(points),
            }

    def _on_cmd(self, name: str, message: Twist) -> None:
        with self._lock:
            self._rates[name].add(time.monotonic())
            self._commands[name] = (
                float(message.linear.x),
                float(message.linear.y),
                float(message.angular.z),
            )

    def _on_pause(self, message: Bool) -> None:
        with self._lock:
            self._pause_navigation = bool(message.data)

    def _nearest_lethal(self, costmap: Mapping[str, Any]) -> float | None:
        frame = str(costmap.get("frame", ""))
        lethal = costmap.get("lethal")
        if not frame or not isinstance(lethal, np.ndarray) or lethal.size == 0:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                frame, "base_footprint", rclpy.time.Time()
            )
        except TransformException:
            return None
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        half = float(costmap["resolution"]) / 2.0
        delta = np.maximum(np.abs(lethal - np.asarray([x, y])) - half, 0.0)
        return float(np.min(np.hypot(delta[:, 0], delta[:, 1])))

    @staticmethod
    def _plan_clearance(plan: Mapping[str, Any], costmap: Mapping[str, Any]) -> float | None:
        points = plan.get("points")
        lethal = costmap.get("lethal")
        resolution = float(costmap.get("resolution", 0.0))
        if (
            not points
            or not isinstance(lethal, np.ndarray)
            or lethal.size == 0
            or resolution <= 0
            or
            plan.get("frame") != costmap.get("frame")
        ):
            return None
        dense: list[tuple[float, float]] = [points[0]]
        step = resolution / 2.0
        for start, end in zip(points, points[1:]):
            distance = math.hypot(end[0] - start[0], end[1] - start[1])
            count = max(1, math.ceil(distance / step))
            dense.extend(
                (
                    start[0] + (end[0] - start[0]) * index / count,
                    start[1] + (end[1] - start[1]) * index / count,
                )
                for index in range(1, count + 1)
            )
        half = resolution / 2.0
        nearest = math.inf
        for start in range(0, len(dense), 128):
            chunk = np.asarray(dense[start:start + 128], dtype=float)
            delta = np.maximum(np.abs(chunk[:, None, :] - lethal[None, :, :]) - half, 0.0)
            nearest = min(nearest, float(np.min(np.hypot(delta[:, :, 0], delta[:, :, 1]))))
        return nearest if math.isfinite(nearest) else None

    def metrics(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            rates = {name: tracker.summary(now) for name, tracker in self._rates.items()}
            scan = dict(self._scan)
            costmaps = {name: dict(value) for name, value in self._costmaps.items()}
            footprints = {name: dict(value) for name, value in self._footprints.items()}
            plan = dict(self._plan)
            commands = dict(self._commands)
            controller_config = dict(self._controller_config)
            paused = self._pause_navigation
            messages = list(self._messages)
        for item in costmaps.values():
            item.pop("lethal", None)
        source_costmaps = {name: dict(value) for name, value in self._costmaps.items()}
        nearest = {name: self._nearest_lethal(value) for name, value in source_costmaps.items()}
        clearance = self._plan_clearance(plan, source_costmaps.get("global", {}))
        smoothed = commands.get("cmd_vel_smoothed")
        final = commands.get("cmd_vel")
        collision_limited = None
        if smoothed is not None and final is not None:
            input_norm = math.hypot(smoothed[0], smoothed[1]) + abs(smoothed[2])
            output_norm = math.hypot(final[0], final[1]) + abs(final[2])
            collision_limited = input_norm > 1e-3 and output_norm + 1e-3 < input_norm
        nav_command = commands.get("cmd_vel_nav")
        desired = controller_config.get("rpp_desired_linear_vel")
        regulated = None
        if (
            nav_command is not None
            and isinstance(desired, (int, float))
            and desired > 0.0
        ):
            regulated = (
                abs(nav_command[0]) > 1e-3
                and abs(nav_command[0]) < float(desired) * 0.95
            )
        controller_config["rpp_regulated"] = regulated
        return {
            "sensor": {"scan": {**rates["scan"], **scan}, "d435": rates["d435"]},
            "costmap": {
                name: {**value, **rates[f"{name}_costmap"], "nearest_lethal": nearest.get(name)}
                for name, value in costmaps.items()
            },
            "footprint": footprints,
            "plan": {
                "count": len(plan.get("points", [])),
                "length": plan.get("length"),
                "frame": plan.get("frame"),
                "clearance": clearance,
                **rates["plan"],
            },
            "controller": {
                "commands": commands,
                "rates": {name: rates[name] for name in commands},
                "collision_monitor_limited": collision_limited,
                **controller_config,
            },
            "pause_navigation": paused,
            "messages": messages,
        }

    def _call(self, service_type: Any, name: str, request: Any, timeout: float = 10.0) -> Any:
        client = self._service_clients.get(name)
        if client is None:
            client = self.create_client(service_type, name)
            self._service_clients[name] = client
        if not client.wait_for_service(timeout_sec=min(timeout, 3.0)):
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

    def get_parameters(self, specs: Sequence[ParameterSpec]) -> dict[str, Any]:
        by_node: dict[str, list[ParameterSpec]] = {}
        result: dict[str, Any] = {}
        for spec in specs:
            if spec.node and spec.parameter:
                by_node.setdefault(spec.node, []).append(spec)
            elif spec.alias == "structure.replan_frequency":
                result[spec.alias] = 1.0
        for node, node_specs in by_node.items():
            request = GetParameters.Request()
            request.names = [spec.parameter for spec in node_specs]
            reply = self._call(GetParameters, f"{node}/get_parameters", request)
            for spec, parameter_value in zip(node_specs, reply.values):
                result[spec.alias] = parameter_value_to_python(parameter_value)
        return result

    def refresh_controller_metadata(self) -> None:
        aliases = (
            "structure.controller_plugin",
            "rpp.desired_linear_vel",
            "rpp.use_regulated_linear_velocity_scaling",
            "rpp.use_cost_regulated_linear_velocity_scaling",
            "rpp.use_collision_detection",
        )
        values = self.get_parameters([REGISTRY[alias] for alias in aliases])
        command = self._commands.get("cmd_vel_nav")
        desired = values.get("rpp.desired_linear_vel")
        regulated = None
        if (
            command is not None
            and isinstance(desired, (int, float))
            and desired > 0.0
        ):
            regulated = abs(command[0]) > 1e-3 and abs(command[0]) < float(desired) * 0.95
        with self._lock:
            self._controller_config = {
                "rpp_plugin": values.get("structure.controller_plugin"),
                "rpp_desired_linear_vel": desired,
                "rpp_regulated": regulated,
                "rpp_curvature_regulation": values.get(
                    "rpp.use_regulated_linear_velocity_scaling"
                ),
                "rpp_cost_regulation": values.get(
                    "rpp.use_cost_regulated_linear_velocity_scaling"
                ),
                "rpp_collision_detection": values.get("rpp.use_collision_detection"),
            }

    def describe_parameter(self, spec: ParameterSpec) -> Any:
        request = DescribeParameters.Request()
        request.names = [spec.parameter]
        reply = self._call(DescribeParameters, f"{spec.node}/describe_parameters", request)
        return reply.descriptors[0]

    def _set_node(self, node: str, values: Sequence[tuple[ParameterSpec, Any]]) -> None:
        request = SetParametersAtomically.Request()
        request.parameters = [
            Parameter(spec.parameter, value=value).to_parameter_msg()
            for spec, value in values
        ]
        reply = self._call(
            SetParametersAtomically, f"{node}/set_parameters_atomically", request
        )
        if not reply.result.successful:
            raise RuntimeError(reply.result.reason or f"{node} 拒绝参数")

    def _set_grouped(self, changes: Mapping[str, Any]) -> dict[str, Any]:
        by_node: dict[str, list[tuple[ParameterSpec, Any]]] = {}
        for alias, value in changes.items():
            spec = REGISTRY[alias]
            by_node.setdefault(spec.node, []).append((spec, value))
        for node, values in by_node.items():
            self._set_node(node, values)
        readback = self.get_parameters([REGISTRY[alias] for alias in changes])
        failures = [
            alias
            for alias, requested in changes.items()
            if not equivalent(requested, readback.get(alias))
        ]
        if failures:
            raise RuntimeError("read-back 不一致：" + ", ".join(failures))
        return readback

    def _manager_active(self) -> bool:
        reply = self._call(
            Trigger,
            "/lifecycle_manager_navigation/is_active",
            Trigger.Request(),
            timeout=5.0,
        )
        return bool(reply.success)

    def _all_nodes_active(self) -> bool:
        for node in MANAGED_LIFECYCLE_NODES:
            reply = self._call(
                GetState, f"{node}/get_state", GetState.Request(), timeout=5.0
            )
            if reply.current_state.id != State.PRIMARY_STATE_ACTIVE:
                return False
        return True

    def _manage(self, command: int, timeout: float = 30.0) -> None:
        request = ManageLifecycleNodes.Request()
        request.command = command
        reply = self._call(
            ManageLifecycleNodes,
            "/lifecycle_manager_navigation/manage_nodes",
            request,
            timeout=timeout,
        )
        if not reply.success:
            raise RuntimeError(f"Lifecycle Manager 命令 {command} 失败")

    def _trigger(self, service: str) -> None:
        reply = self._call(Trigger, service, Trigger.Request(), timeout=10.0)
        if not reply.success:
            raise RuntimeError(reply.message or f"{service} 失败")

    def _resume_when_ready(self, timeout: float = 15.0) -> None:
        """等待 safety_supervisor 完成下一轮 1 Hz 图审计后再解除人工暂停。"""

        deadline = time.monotonic() + timeout
        last_error = ""
        while time.monotonic() < deadline:
            try:
                self._trigger("/navigation/resume")
                return
            except RuntimeError as error:
                last_error = str(error)
                time.sleep(0.5)
        raise RuntimeError(f"Nav2 已 active，但安全监督未允许 resume：{last_error}")

    def _wait_healthy(self, timeout: float = 60.0) -> None:
        deadline = time.monotonic() + timeout
        last_error = ""
        while time.monotonic() < deadline:
            try:
                if self._manager_active() and self._all_nodes_active():
                    return
            except Exception as error:  # 节点转换期间服务短暂不可用属于预期
                last_error = str(error)
            time.sleep(0.25)
        raise TimeoutError("Nav2 未恢复 active" + (f"：{last_error}" if last_error else ""))

    def _wait_stopped(self, timeout: float = 5.0) -> None:
        """在拆除 Nav2 插件前确认安全锁和最终速度都已经落地。"""

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                paused = self._pause_navigation
                smoothed = self._commands.get("cmd_vel_smoothed")
                final = self._commands.get("cmd_vel")
            smoothed_zero = (
                smoothed is not None and max(abs(value) for value in smoothed) <= 1e-3
            )
            # Collision Monitor 在静止且没有活动 source 回调时可能不重复发布
            # /cmd_vel；此时“无新最终输出”比伪造第二个 /cmd_vel 发布者更安全。
            final_safe = final is None or max(abs(value) for value in final) <= 1e-3
            if paused is True and smoothed_zero and final_safe:
                return
            time.sleep(0.05)
        raise TimeoutError(
            "停车确认超时：未同时看到 /pause_navigation=true、平滑零速度和安全最终输出"
        )

    def apply_values(self, changes: Mapping[str, Any]) -> dict[str, Any]:
        if not changes:
            raise ValueError("没有参数需要修改")
        specs = [REGISTRY[alias] for alias in changes]
        restart = [spec.alias for spec in specs if spec.capability == Capability.RESTART_REQUIRED]
        if restart:
            raise ValueError(
                "拒绝伪热更新：" + ", ".join(restart) + " 必须修改配置后完整重启导航入口"
            )
        original = self.get_parameters(specs)
        needs_reload = any(spec.capability == Capability.LIFECYCLE_RELOAD for spec in specs)
        if not needs_reload:
            try:
                readback = self._set_grouped(changes)
            except Exception:
                try:
                    self._set_grouped(original)
                except Exception as rollback_error:
                    self.log_message(f"LIVE 参数回滚未完成：{rollback_error}")
                raise
            with self._lock:
                self._dirty.update(readback)
                for alias in changes:
                    self._last_effect[alias] = "原子 set + read-back；可观察量见监控面板"
            if any(alias.startswith("rpp.") for alias in changes):
                self.refresh_controller_metadata()
            return readback

        with self._lock:
            initially_paused = self._pause_navigation
        initially_healthy = self._manager_active() and self._all_nodes_active()
        if not initially_healthy or initially_paused is not False:
            raise RuntimeError("Nav2 初始状态不健康，拒绝自动 reload；请先排除红项")
        tool_stopped = False
        reset_done = False
        try:
            self._trigger("/navigation/stop")
            tool_stopped = initially_paused is False
            self._wait_stopped()
            # Costmap2DROS 是 controller/planner 内部节点。Humble 在父节点
            # cleanup 后会停止其参数服务 callback group，因此无法在
            # unconfigured 状态可靠应答。先写入参数（此时旧 ObservationBuffer
            # 不会伪生效），再 RESET/STARTUP 让插件重建并读取新值。
            readback = self._set_grouped(changes)
            self._manage(ManageLifecycleNodes.Request.RESET)
            reset_done = True
            self._manage(ManageLifecycleNodes.Request.STARTUP)
            self._wait_healthy()
        except Exception:
            self.log_message("reload 失败，正在恢复原值；导航保持暂停")
            try:
                if reset_done:
                    self._manage(ManageLifecycleNodes.Request.STARTUP)
                    self._wait_healthy()
                self._set_grouped(original)
                # 即使失败发生在 RESET 前，source 参数的旧插件也没有读取过
                # 临时值；仍做一次完整重建，确保内部状态与 read-back 一致。
                self._manage(ManageLifecycleNodes.Request.RESET)
                self._manage(ManageLifecycleNodes.Request.STARTUP)
                self._wait_healthy()
            except Exception as rollback_error:
                self.log_message(f"回滚未完成：{rollback_error}")
            raise
        if tool_stopped:
            self._resume_when_ready()
        with self._lock:
            self._dirty.update(readback)
            for alias in changes:
                self._last_effect[alias] = (
                    "停车 + active 态暂存（旧 buffer 忽略）+ RESET/STARTUP "
                    "+ active 健康复核"
                )
        if any(alias.startswith("rpp.") for alias in changes):
            self.refresh_controller_metadata()
        return readback

    def yaml_value(self, spec: ParameterSpec) -> Any:
        if not spec.persistent:
            return spec.plugin_default
        assert spec.yaml_file is not None and spec.yaml_path is not None
        with (package_root_from_module() / spec.yaml_file).open("r", encoding="utf-8") as stream:
            return get_path(yaml.safe_load(stream), spec.yaml_path)

    def snapshot(self) -> dict[str, Any]:
        output = self.metrics()
        try:
            output["parameters"] = self.get_parameters(list(REGISTRY.values()))
        except Exception as error:
            output["parameter_error"] = str(error)
        output["profiles"] = {name: "UNCALIBRATED" for name in PROFILES}
        return output

    def record(self, fields: Mapping[str, str]) -> Path:
        metrics = self.metrics()
        current: dict[str, Any] = {}
        wanted = [
            REGISTRY[name] for name in (
                "local.inflation_radius",
                "local.cost_scaling_factor",
                "global.inflation_radius",
                "global.cost_scaling_factor",
                "rpp.desired_linear_vel",
            )
        ]
        try:
            current = self.get_parameters(wanted)
        except Exception as error:
            self.log_message(f"record 参数读取失败：{error}")
        row = {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "run_id": fields.get("run_id", time.strftime("run_%Y%m%d_%H%M%S")),
            "case": fields.get("case", "manual"),
            "speed_mps": fields.get("speed", current.get("rpp.desired_linear_vel", "")),
            "local_inflation_radius_m": current.get("local.inflation_radius", ""),
            "local_cost_scaling_factor": current.get("local.cost_scaling_factor", ""),
            "global_inflation_radius_m": current.get("global.inflation_radius", ""),
            "global_cost_scaling_factor": current.get("global.cost_scaling_factor", ""),
            "scan_period_p99_s": metrics["sensor"]["scan"].get("period_p99"),
            "d435_period_p99_s": metrics["sensor"]["d435"].get("period_p99"),
            "plan_length_m": metrics["plan"].get("length"),
            "plan_clearance_m": metrics["plan"].get("clearance"),
            "local_nearest_lethal_m": metrics["costmap"].get("local", {}).get("nearest_lethal"),
            "contact_events": fields.get("contact", ""),
            "parameter_effect_method": fields.get("effect", "runtime_monitor"),
            "result": fields.get("result", ""),
            "notes": fields.get("notes", ""),
        }
        path = package_root_from_module() / "logs" / "inflation_tuning.csv"
        path.parent.mkdir(parents=True, exist_ok=True)
        exists = path.exists() and path.stat().st_size > 0
        with path.open("a", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(row))
            if not exists:
                writer.writeheader()
            writer.writerow(row)
        return path


class TunerApplication:
    def __init__(self, node: RuntimeMonitor) -> None:
        self.node = node
        self.running = True

    def _show(self, aliases: Sequence[str]) -> str:
        specs = [REGISTRY[alias] for alias in aliases] if aliases else list(REGISTRY.values())
        runtime: dict[str, Any] = {}
        errors: dict[str, str] = {}
        for spec in specs:
            try:
                runtime.update(self.node.get_parameters([spec]))
            except Exception as error:
                errors[spec.alias] = str(error)
        lines = ["alias | runtime | capability | node.parameter | unit"]
        for spec in specs:
            value = runtime.get(spec.alias, f"ERROR: {errors.get(spec.alias, 'N/A')}")
            lines.append(
                f"{spec.alias} | {value} | {spec.capability.value} | "
                f"{spec.node}.{spec.parameter} | {spec.unit}"
            )
        return "\n".join(lines)

    def execute(self, command_line: str) -> str:
        try:
            arguments = shlex.split(command_line)
        except ValueError as error:
            return f"命令解析失败：{error}"
        if not arguments:
            return ""
        command = arguments[0].lower()
        try:
            if command == "help":
                return (
                    "show [alias] | set <alias> <value> | reset <alias> | "
                    "reset-group <group> | profile <name> | save | "
                    "record [key=value ...] | help | quit"
                )
            if command == "show":
                unknown = [alias for alias in arguments[1:] if alias not in REGISTRY]
                if unknown:
                    raise KeyError("未知别名：" + ", ".join(unknown))
                return self._show(arguments[1:])
            if command == "set":
                if len(arguments) < 3:
                    raise ValueError("用法：set <alias> <value>")
                alias = arguments[1]
                spec = REGISTRY[alias]
                value = parse_value(spec, " ".join(arguments[2:]))
                readback = self.node.apply_values({alias: value})
                return f"已生效：{alias}={readback[alias]}（{spec.capability.value}）"
            if command == "reset":
                if len(arguments) != 2:
                    raise ValueError("用法：reset <alias>")
                alias = arguments[1]
                spec = REGISTRY[alias]
                value = self.node.yaml_value(spec)
                readback = self.node.apply_values({alias: value})
                return f"已恢复 YAML 基线：{alias}={readback[alias]}"
            if command == "reset-group":
                if len(arguments) != 2:
                    raise ValueError("用法：reset-group <group>")
                specs = [spec for spec in REGISTRY.values() if spec.group == arguments[1]]
                if not specs:
                    raise KeyError(f"未知或空组：{arguments[1]}")
                changes = {
                    spec.alias: self.node.yaml_value(spec)
                    for spec in specs if spec.capability != Capability.RESTART_REQUIRED
                }
                readback = self.node.apply_values(changes)
                return f"已恢复组 {arguments[1]}：{len(readback)} 项"
            if command == "profile":
                if len(arguments) != 2 or arguments[1] not in PROFILES:
                    raise ValueError("用法：profile safe|balanced|aggressive")
                if PROFILES[arguments[1]] is None:
                    return f"{arguments[1]}: UNCALIBRATED（阶段 5–7 硬安全门通过前禁止激活）"
                return "profile 数据异常：阶段 0 不应包含标定值"
            if command == "save":
                if len(arguments) != 1:
                    raise ValueError("用法：save")
                result = persist_values(package_root_from_module(), self.node.dirty)
                return f"备份：{result.backup_dir}\n{result.unified_diff or '运行值与 YAML 相同，无文本差异'}"
            if command == "record":
                fields = {}
                for item in arguments[1:]:
                    if "=" not in item:
                        raise ValueError("record 字段格式必须为 key=value")
                    key, value = item.split("=", 1)
                    fields[key] = value
                path = self.node.record(fields)
                return f"已记录：{path}"
            if command in {"quit", "exit"}:
                self.running = False
                return "正在退出"
            return f"未知命令：{command}；输入 help 查看帮助"
        except (KeyError, ValueError, RuntimeError, TimeoutError) as error:
            return f"拒绝/失败：{error}"

    def summary_lines(self) -> list[str]:
        metrics = self.node.metrics()
        scan = metrics["sensor"]["scan"]
        d435 = metrics["sensor"]["d435"]
        local = metrics["costmap"].get("local", {})
        global_map = metrics["costmap"].get("global", {})
        plan = metrics["plan"]
        controller = metrics["controller"]
        lines = [
            "Go2 Nav2 安全调参与监控（safe/balanced/aggressive: UNCALIBRATED）",
            (
                f"Sensor  /scan {_fmt(scan.get('hz'))} Hz age={_fmt(scan.get('age'))} s "
                f"valid/inf/nan={scan.get('valid', 0)}/{scan.get('inf', 0)}/{scan.get('nan', 0)} "
                f"nearest={_fmt(scan.get('nearest'))} m range_min={_fmt(scan.get('range_min'))} m"
            ),
            (
                f"Sensor  D435 {_fmt(d435.get('hz'))} Hz "
                f"age={_fmt(d435.get('age'))} s "
                f"p99={_fmt(d435.get('period_p99'))} s"
            ),
            (
                f"Costmap local lethal/inflated={local.get('lethal_count', 0)}/"
                f"{local.get('inflated_count', 0)} "
                f"nearest={_fmt(local.get('nearest_lethal'))} m "
                f"footprint={len(metrics['footprint'].get('local', {}).get('points', []))} "
                "vertices"
            ),
            (
                f"Costmap global lethal/inflated={global_map.get('lethal_count', 0)}/"
                f"{global_map.get('inflated_count', 0)} "
                f"nearest={_fmt(global_map.get('nearest_lethal'))} m"
            ),
            (
                f"Plan    poses={plan.get('count', 0)} length={_fmt(plan.get('length'))} m "
                f"age={_fmt(plan.get('age'))} s replan_p50/p99="
                f"{_fmt(plan.get('period_p50'))}/{_fmt(plan.get('period_p99'))} s "
                f"clearance={_fmt(plan.get('clearance'))} m"
            ),
            f"Control commands={controller.get('commands', {})}",
            (
                f"Control plugin={controller.get('rpp_plugin')} "
                f"regulated={controller.get('rpp_regulated')} "
                f"collision_monitor_limited={controller.get('collision_monitor_limited')} "
                f"pause_navigation={metrics.get('pause_navigation')}"
            ),
        ]
        lines.extend(metrics.get("messages", [])[-3:])
        return lines

    def run_plain(self) -> None:
        print("\n".join(self.summary_lines()))
        print("输入 help 查看命令。")
        while self.running:
            try:
                response = self.execute(input("nav_tuner> "))
            except EOFError:
                break
            if response:
                print(response)

    def run_monitor_plain(self) -> None:
        try:
            while self.running:
                print("\n".join(self.summary_lines()), flush=True)
                time.sleep(2.0)
        except KeyboardInterrupt:
            pass

    def run_curses(self, screen: Any, monitor_only: bool = False) -> None:
        curses.curs_set(1 if not monitor_only else 0)
        screen.nodelay(True)
        command = ""
        response = ""
        while self.running:
            screen.erase()
            height, width = screen.getmaxyx()
            lines = self.summary_lines()
            if response:
                lines.extend(["", response])
            for row, line in enumerate(lines[: max(0, height - 3)]):
                try:
                    screen.addnstr(row, 0, line, max(1, width - 1))
                except curses.error:
                    pass
            prompt = "按 q 退出监控" if monitor_only else f"nav_tuner> {command}"
            try:
                screen.addnstr(height - 1, 0, prompt, max(1, width - 1))
            except curses.error:
                pass
            screen.refresh()
            key = screen.getch()
            if key == -1:
                time.sleep(0.1)
                continue
            if monitor_only:
                if key in {ord("q"), ord("Q"), 27}:
                    break
                continue
            if key in {10, 13}:
                response = self.execute(command)
                command = ""
            elif key in {curses.KEY_BACKSPACE, 127, 8}:
                command = command[:-1]
            elif key == 27:
                command = ""
            elif 32 <= key <= 126:
                command += chr(key)


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Go2 Nav2 运行时安全调参与监控")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--snapshot", action="store_true", help="等待采样后输出一次 JSON 快照并退出")
    mode.add_argument("--monitor-only", action="store_true", help="只显示监控，不接受调参命令")
    mode.add_argument("--execute", action="append", metavar="COMMAND", help="执行一条命令并退出，可重复")
    parser.add_argument("--sample-seconds", type=float, default=3.0, help="snapshot 启动后的采样秒数")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = create_parser().parse_args(argv)
    rclpy.init(args=None)
    node = RuntimeMonitor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    application = TunerApplication(node)
    result = 0
    try:
        time.sleep(0.2)
        try:
            node.refresh_controller_metadata()
        except Exception as error:
            node.log_message(f"控制器元数据暂不可用：{error}")
        if arguments.snapshot:
            time.sleep(max(0.0, arguments.sample_seconds))
            print(json.dumps(node.snapshot(), ensure_ascii=False, indent=2, default=str))
        elif arguments.execute:
            time.sleep(0.5)
            for command in arguments.execute:
                response = application.execute(command)
                print(response)
                if response.startswith("拒绝/失败"):
                    result = 2
        elif sys.stdin.isatty() and sys.stdout.isatty():
            curses.wrapper(application.run_curses, arguments.monitor_only)
        elif arguments.monitor_only:
            application.run_monitor_plain()
        else:
            application.run_plain()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=5.0)
        spin_thread.join(timeout=5.0)
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
