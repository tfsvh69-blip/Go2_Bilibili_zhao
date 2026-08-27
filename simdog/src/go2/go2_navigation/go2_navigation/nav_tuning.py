"""Nav2 运行时调参的注册表、指标计算和安全持久化工具。

本模块刻意不依赖 ROS，便于对参数能力分类、数值校验和 YAML 定点修改做单元测试。
ROS 服务调用与终端界面位于 :mod:`go2_navigation.nav_tuner`。
"""

from __future__ import annotations

import difflib
import json
import math
import os
import re
import shutil
import tempfile
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import Any, Iterable, Mapping, MutableMapping, Sequence

import yaml


class Capability(str, Enum):
    """参数在 Nav2 1.1.20 中真正生效所需的操作。"""

    LIVE = "LIVE"
    LIFECYCLE_RELOAD = "LIFECYCLE RELOAD"
    RESTART_REQUIRED = "RESTART REQUIRED"


@dataclass(frozen=True)
class ParameterSpec:
    """一个可审计参数别名。"""

    alias: str
    group: str
    node: str
    parameter: str
    value_type: str
    unit: str
    yaml_file: str | None
    yaml_path: tuple[str, ...] | None
    capability: Capability
    description: str
    minimum: float | None = None
    maximum: float | None = None
    choices: tuple[str, ...] = ()
    plugin_default: Any = None
    safety_true_only: bool = False

    @property
    def persistent(self) -> bool:
        return self.yaml_file is not None and self.yaml_path is not None


NAVIGATION_YAML = "config/navigation.yaml"
RPP_YAML = "config/controller_forward_rpp.yaml"
LOCAL_PARAMS = ("local_costmap", "local_costmap", "ros__parameters")
GLOBAL_PARAMS = ("global_costmap", "global_costmap", "ros__parameters")
CONTROLLER_PARAMS = ("controller_server", "ros__parameters")
PLANNER_PARAMS = ("planner_server", "ros__parameters")
SMOOTHER_PARAMS = ("velocity_smoother", "ros__parameters")


def _spec(
    alias: str,
    group: str,
    node: str,
    parameter: str,
    value_type: str,
    unit: str,
    yaml_file: str | None,
    yaml_path: tuple[str, ...] | None,
    capability: Capability,
    description: str,
    **kwargs: Any,
) -> ParameterSpec:
    return ParameterSpec(
        alias=alias,
        group=group,
        node=node,
        parameter=parameter,
        value_type=value_type,
        unit=unit,
        yaml_file=yaml_file,
        yaml_path=yaml_path,
        capability=capability,
        description=description,
        **kwargs,
    )


def build_registry() -> dict[str, ParameterSpec]:
    """构建唯一参数注册表。

    能力分类以 Ubuntu 22.04 上 Nav2 1.1.20 对应插件动态回调为准。ObstacleLayer
    的 observation source 子参数没有动态回调，必须清理并重新配置插件。
    """

    specs: list[ParameterSpec] = []
    for scope, node, root in (
        ("local", "/local_costmap/local_costmap", LOCAL_PARAMS),
        ("global", "/global_costmap/global_costmap", GLOBAL_PARAMS),
    ):
        specs.extend(
            [
                _spec(
                    f"{scope}.inflation_radius",
                    f"{scope}_inflation",
                    node,
                    "inflation_layer.inflation_radius",
                    "float",
                    "m",
                    NAVIGATION_YAML,
                    root + ("inflation_layer", "inflation_radius"),
                    Capability.LIVE,
                    "膨胀层作用半径",
                    minimum=0.0,
                    maximum=5.0,
                    plugin_default=0.55,
                ),
                _spec(
                    f"{scope}.cost_scaling_factor",
                    f"{scope}_inflation",
                    node,
                    "inflation_layer.cost_scaling_factor",
                    "float",
                    "1/m",
                    NAVIGATION_YAML,
                    root + ("inflation_layer", "cost_scaling_factor"),
                    Capability.LIVE,
                    "膨胀代价随距离衰减的指数系数",
                    minimum=0.01,
                    maximum=100.0,
                    plugin_default=10.0,
                ),
                _spec(
                    f"geometry.{scope}_footprint",
                    "geometry",
                    node,
                    "footprint",
                    "footprint",
                    "m",
                    NAVIGATION_YAML,
                    root + ("footprint",),
                    Capability.LIVE,
                    f"{scope} costmap 的机器人二维足迹",
                ),
                _spec(
                    f"geometry.{scope}_padding",
                    "geometry",
                    node,
                    "footprint_padding",
                    "float",
                    "m",
                    NAVIGATION_YAML,
                    root + ("footprint_padding",),
                    Capability.LIVE,
                    f"{scope} costmap 足迹外扩余量",
                    minimum=0.0,
                    maximum=0.5,
                    plugin_default=0.01,
                ),
            ]
        )

    source_rows = (
        (
            "local.scan", "local_scan", "/local_costmap/local_costmap",
            LOCAL_PARAMS + ("scan_layer", "scan"),
        ),
        (
            "local.d435", "local_d435", "/local_costmap/local_costmap",
            LOCAL_PARAMS + ("d435_layer", "d435"),
        ),
        (
            "global.scan", "global_scan", "/global_costmap/global_costmap",
            GLOBAL_PARAMS + ("obstacle_layer", "scan"),
        ),
    )
    source_fields = (
        ("observation_persistence", "float", "s", 0.0, 10.0, 0.0, "障碍观测保留时间"),
        ("expected_update_rate", "float", "s", 0.0, 60.0, 0.0, "期望传感器更新周期，0 表示关闭 stale 检查"),
        ("obstacle_min_range", "float", "m", 0.0, 50.0, 0.0, "用于标记的最小距离"),
        ("obstacle_max_range", "float", "m", 0.0, 100.0, 2.5, "用于标记的最大距离"),
        ("raytrace_min_range", "float", "m", 0.0, 50.0, 0.0, "用于清除的最小射线距离"),
        ("raytrace_max_range", "float", "m", 0.0, 100.0, 3.0, "用于清除的最大射线距离"),
        ("min_obstacle_height", "float", "m", -10.0, 10.0, 0.0, "source 接收的最低障碍高度"),
        ("max_obstacle_height", "float", "m", -10.0, 10.0, 2.0, "source 接收的最高障碍高度"),
        ("marking", "bool", "-", None, None, True, "是否写入障碍单元"),
        ("clearing", "bool", "-", None, None, True, "是否用射线清除障碍单元"),
    )
    for prefix, group, node, yaml_root in source_rows:
        runtime_prefix = (
            "scan_layer.scan" if prefix == "local.scan" else
            "d435_layer.d435" if prefix == "local.d435" else
            "obstacle_layer.scan"
        )
        for field, value_type, unit, minimum, maximum, default, description in source_fields:
            specs.append(
                _spec(
                    f"memory.{prefix}.{field}",
                    group,
                    node,
                    f"{runtime_prefix}.{field}",
                    value_type,
                    unit,
                    NAVIGATION_YAML,
                    yaml_root + (field,),
                    Capability.LIFECYCLE_RELOAD,
                    description,
                    minimum=minimum,
                    maximum=maximum,
                    plugin_default=default,
                    safety_true_only=field in {"marking", "clearing"},
                )
            )

    for scope, node, root, layer in (
        ("local.scan", "/local_costmap/local_costmap", LOCAL_PARAMS, "scan_layer"),
        ("local.d435", "/local_costmap/local_costmap", LOCAL_PARAMS, "d435_layer"),
        ("global.scan", "/global_costmap/global_costmap", GLOBAL_PARAMS, "obstacle_layer"),
    ):
        specs.append(
            _spec(
                f"memory.{scope}.footprint_clearing_enabled",
                scope.replace(".", "_"),
                node,
                f"{layer}.footprint_clearing_enabled",
                "bool",
                "-",
                NAVIGATION_YAML,
                root + (layer, "footprint_clearing_enabled"),
                Capability.LIVE,
                "是否清除机器人足迹覆盖的障碍单元",
                plugin_default=True,
                safety_true_only=True,
            )
        )

    rpp_fields = (
        ("desired_linear_vel", "float", "m/s", 0.0, 1.0, Capability.LIVE),
        ("lookahead_dist", "float", "m", 0.0, 10.0, Capability.LIVE),
        ("min_lookahead_dist", "float", "m", 0.0, 10.0, Capability.LIVE),
        ("max_lookahead_dist", "float", "m", 0.0, 10.0, Capability.LIVE),
        ("lookahead_time", "float", "s", 0.0, 10.0, Capability.LIVE),
        ("min_approach_linear_velocity", "float", "m/s", 0.0, 1.0, Capability.LIVE),
        ("approach_velocity_scaling_dist", "float", "m", 0.0, 10.0, Capability.LIVE),
        ("max_allowed_time_to_collision_up_to_carrot", "float", "s", 0.0, 10.0, Capability.LIVE),
        ("use_regulated_linear_velocity_scaling", "bool", "-", None, None, Capability.LIVE),
        ("use_cost_regulated_linear_velocity_scaling", "bool", "-", None, None, Capability.LIVE),
        ("cost_scaling_dist", "float", "m", 0.0, 10.0, Capability.LIVE),
        ("cost_scaling_gain", "float", "-", 0.0, 10.0, Capability.LIVE),
        ("inflation_cost_scaling_factor", "float", "1/m", 0.01, 100.0, Capability.LIVE),
        ("regulated_linear_scaling_min_radius", "float", "m", 0.0, 10.0, Capability.LIVE),
        ("regulated_linear_scaling_min_speed", "float", "m/s", 0.0, 1.0, Capability.LIVE),
        ("use_collision_detection", "bool", "-", None, None, Capability.LIFECYCLE_RELOAD),
        ("use_interpolation", "bool", "-", None, None, Capability.LIFECYCLE_RELOAD),
    )
    for field, value_type, unit, minimum, maximum, capability in rpp_fields:
        specs.append(
            _spec(
                f"rpp.{field}",
                "rpp",
                "/controller_server",
                f"FollowPath.{field}",
                value_type,
                unit,
                RPP_YAML,
                CONTROLLER_PARAMS + ("FollowPath", field),
                capability,
                f"RPP {field}",
                minimum=minimum,
                maximum=maximum,
                safety_true_only=field == "use_collision_detection",
            )
        )

    planner_fields = (
        ("tolerance", "float", "m", 0.0, 10.0),
        ("cost_travel_multiplier", "float", "-", 0.0, 100.0),
        ("max_iterations", "int", "次", 1.0, 10_000_000.0),
        ("max_on_approach_iterations", "int", "次", 1.0, 10_000_000.0),
        ("allow_unknown", "bool", "-", None, None),
        ("downsample_costmap", "bool", "-", None, None),
        ("downsampling_factor", "int", "倍", 1.0, 100.0),
        ("use_final_approach_orientation", "bool", "-", None, None),
    )
    for field, value_type, unit, minimum, maximum in planner_fields:
        specs.append(
            _spec(
                f"planner.{field}",
                "planner",
                "/planner_server",
                f"GridBased.{field}",
                value_type,
                unit,
                NAVIGATION_YAML,
                PLANNER_PARAMS + ("GridBased", field),
                Capability.LIVE,
                f"SmacPlanner2D {field}",
                minimum=minimum,
                maximum=maximum,
            )
        )

    specs.extend(
        [
            _spec(
                "velocity.max_velocity",
                "velocity",
                "/velocity_smoother",
                "max_velocity",
                "float_array",
                "m/s,rad/s",
                RPP_YAML,
                SMOOTHER_PARAMS + ("max_velocity",),
                Capability.LIVE,
                "Velocity Smoother 三轴最大速度",
            ),
            _spec(
                "structure.controller_plugin",
                "structure",
                "/controller_server",
                "FollowPath.plugin",
                "string",
                "-",
                RPP_YAML,
                CONTROLLER_PARAMS + ("FollowPath", "plugin"),
                Capability.RESTART_REQUIRED,
                "控制器插件类型",
            ),
            _spec(
                "structure.planner_plugin",
                "structure",
                "/planner_server",
                "GridBased.plugin",
                "string",
                "-",
                NAVIGATION_YAML,
                PLANNER_PARAMS + ("GridBased", "plugin"),
                Capability.RESTART_REQUIRED,
                "规划器插件类型",
            ),
            _spec(
                "structure.local_plugins",
                "structure",
                "/local_costmap/local_costmap",
                "plugins",
                "string_array",
                "-",
                NAVIGATION_YAML,
                LOCAL_PARAMS + ("plugins",),
                Capability.RESTART_REQUIRED,
                "局部代价地图插件列表",
            ),
            _spec(
                "structure.local_scan_sources",
                "structure",
                "/local_costmap/local_costmap",
                "scan_layer.observation_sources",
                "string",
                "-",
                NAVIGATION_YAML,
                LOCAL_PARAMS + ("scan_layer", "observation_sources"),
                Capability.RESTART_REQUIRED,
                "ObstacleLayer observation source 结构",
            ),
            _spec(
                "structure.replan_frequency",
                "structure",
                "",
                "behavior_trees/go2_navigate_to_pose.xml/RateController@hz",
                "float",
                "Hz",
                None,
                None,
                Capability.RESTART_REQUIRED,
                "BT 中硬编码的全局重规划频率",
                plugin_default=1.0,
            ),
        ]
    )

    registry = {item.alias: item for item in specs}
    if len(registry) != len(specs):
        raise RuntimeError("参数注册表存在重复别名")
    return registry


REGISTRY = build_registry()
PROFILES: Mapping[str, Mapping[str, Any] | None] = {
    "safe": None,
    "balanced": None,
    "aggressive": None,
}


def parse_value(spec: ParameterSpec, text: str) -> Any:
    """按注册表类型解析并验证用户输入。"""

    raw = text.strip()
    if spec.value_type == "bool":
        lowered = raw.lower()
        if lowered not in {"true", "false", "1", "0", "yes", "no", "on", "off"}:
            raise ValueError("布尔值只能使用 true/false、1/0、yes/no 或 on/off")
        value: Any = lowered in {"true", "1", "yes", "on"}
    elif spec.value_type == "int":
        if not re.fullmatch(r"[+-]?\d+", raw):
            raise ValueError("需要整数")
        value = int(raw)
    elif spec.value_type == "float":
        value = float(raw)
        if not math.isfinite(value):
            raise ValueError("数值必须有限")
    elif spec.value_type in {"float_array", "string_array", "footprint"}:
        value = yaml.safe_load(raw)
        if not isinstance(value, list):
            raise ValueError("需要 YAML/JSON 列表")
        if spec.value_type == "float_array":
            invalid = any(
                isinstance(item, bool) or not isinstance(item, (int, float))
                for item in value
            )
            if not value or invalid:
                raise ValueError("需要非空数值列表")
            value = [float(v) for v in value]
        elif spec.value_type == "string_array":
            if any(not isinstance(v, str) for v in value):
                raise ValueError("需要字符串列表")
        else:
            if len(value) < 3:
                raise ValueError("footprint 至少需要 3 个顶点")
            points: list[list[float]] = []
            for point in value:
                if not isinstance(point, list) or len(point) != 2:
                    raise ValueError("footprint 顶点格式应为 [x, y]")
                if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in point):
                    raise ValueError("footprint 坐标必须是数值")
                coordinates = [float(point[0]), float(point[1])]
                if not all(math.isfinite(v) for v in coordinates):
                    raise ValueError("footprint 坐标必须有限")
                points.append(coordinates)
            # footprint 在 ROS 参数层本质上是一个字符串。PyYAML 默认会按
            # 80 列折行，read-back 后再持久化便会产生带 ``\n`` 的难读标量。
            # 使用紧凑 JSON（同时也是合法 YAML）固定成可审计的单行格式。
            value = json.dumps(points, separators=(",", ":"))
    else:
        value = raw

    if spec.safety_true_only and value is not True:
        raise ValueError(f"{spec.alias} 是不可妥协的安全开关，只允许保持 true")
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if spec.minimum is not None and value < spec.minimum:
            raise ValueError(f"不得小于 {spec.minimum:g} {spec.unit}")
        if spec.maximum is not None and value > spec.maximum:
            raise ValueError(f"不得大于 {spec.maximum:g} {spec.unit}")
    if spec.choices and str(value) not in spec.choices:
        raise ValueError("可选值：" + ", ".join(spec.choices))
    return value


def equivalent(left: Any, right: Any, tolerance: float = 1e-9) -> bool:
    """比较参数服务 read-back 与请求值。"""

    if isinstance(left, (int, float)) and not isinstance(left, bool):
        return isinstance(right, (int, float)) and not isinstance(right, bool) and math.isclose(
            float(left), float(right), rel_tol=tolerance, abs_tol=tolerance
        )
    if isinstance(left, Sequence) and not isinstance(left, (str, bytes)):
        return isinstance(right, Sequence) and len(left) == len(right) and all(
            equivalent(a, b, tolerance) for a, b in zip(left, right)
        )
    return left == right


def get_path(data: Mapping[str, Any], path: Sequence[str]) -> Any:
    current: Any = data
    for key in path:
        if not isinstance(current, Mapping) or key not in current:
            raise KeyError(".".join(path))
        current = current[key]
    return current


def set_path(data: MutableMapping[str, Any], path: Sequence[str], value: Any) -> None:
    current: MutableMapping[str, Any] = data
    for key in path[:-1]:
        child = current.get(key)
        if not isinstance(child, MutableMapping):
            raise KeyError(".".join(path))
        current = child
    if path[-1] not in current:
        raise KeyError(".".join(path))
    current[path[-1]] = value


def flatten(data: Any, prefix: tuple[str, ...] = ()) -> dict[tuple[str, ...], Any]:
    """把 YAML 语义树展开，用于验证未越过注册表白名单。"""

    if isinstance(data, Mapping):
        output: dict[tuple[str, ...], Any] = {}
        for key, value in data.items():
            output.update(flatten(value, prefix + (str(key),)))
        return output
    return {prefix: data}


def format_yaml_scalar(value: Any) -> str:
    """格式化单个 YAML 值，不产生文档结束标记。"""

    if isinstance(value, str):
        rendered = yaml.safe_dump(
            value, default_style='"', width=1_000_000
        ).strip()
        if "\n" in rendered:
            raise ValueError("拒绝把多行字符串写入单行 YAML 标量")
        return rendered
    rendered = yaml.safe_dump(
        value, default_flow_style=True, width=1_000_000
    ).strip()
    return rendered.removesuffix("\n...")


def replace_yaml_scalars(
    text: str,
    replacements: Mapping[tuple[str, ...], Any],
) -> str:
    """只替换已存在的 YAML 标量行，保留注释、顺序与层级。

    本项目注册项均位于块映射中的单行标量。若路径缺失或落在复杂多行结构，函数会拒绝，
    不尝试猜测插入位置。
    """

    remaining = dict(replacements)
    stack: list[tuple[int, str]] = []
    output: list[str] = []
    key_pattern = re.compile(r"^(?P<indent>\s*)(?P<key>[^#\s][^:]*?):(?P<rest>.*)$")
    for line in text.splitlines(keepends=True):
        newline = "\n" if line.endswith("\n") else ""
        body = line[:-1] if newline else line
        match = key_pattern.match(body)
        if not match:
            output.append(line)
            continue
        indent = len(match.group("indent").replace("\t", "        "))
        key = match.group("key").strip().strip("'\"")
        while stack and stack[-1][0] >= indent:
            stack.pop()
        path = tuple(item[1] for item in stack) + (key,)
        rest = match.group("rest")
        stripped = rest.strip()
        if not stripped or stripped.startswith("#"):
            stack.append((indent, key))
            output.append(line)
            continue
        if path not in remaining:
            output.append(line)
            continue
        comment = ""
        quote: str | None = None
        escaped = False
        for index, char in enumerate(rest):
            if escaped:
                escaped = False
                continue
            if char == "\\" and quote == '"':
                escaped = True
                continue
            if char in {"'", '"'}:
                quote = None if quote == char else char if quote is None else quote
            elif char == "#" and quote is None:
                comment = rest[index:]
                break
        rendered = format_yaml_scalar(remaining.pop(path))
        suffix = f"  {comment.lstrip()}" if comment else ""
        output.append(f"{match.group('indent')}{match.group('key')}: {rendered}{suffix}{newline}")
    if remaining:
        missing = ", ".join(".".join(path) for path in sorted(remaining))
        raise KeyError(f"YAML 中不存在注册表路径：{missing}")
    return "".join(output)


@dataclass(frozen=True)
class SaveResult:
    backup_dir: Path
    changed_files: tuple[Path, ...]
    unified_diff: str


def persist_values(
    package_root: Path,
    values: Mapping[str, Any],
    *,
    timestamp: str | None = None,
) -> SaveResult:
    """将运行值原子保存到注册表指定 YAML，并在任一失败时恢复全部文件。"""

    package_root = package_root.resolve()
    by_file: dict[str, dict[tuple[str, ...], Any]] = {}
    for alias, value in values.items():
        if alias not in REGISTRY:
            raise KeyError(f"未知别名：{alias}")
        spec = REGISTRY[alias]
        if not spec.persistent or spec.capability == Capability.RESTART_REQUIRED:
            raise ValueError(f"{alias} 不允许由 nav_tuner 持久化")
        assert spec.yaml_file is not None and spec.yaml_path is not None
        by_file.setdefault(spec.yaml_file, {})[spec.yaml_path] = value
    if not by_file:
        raise ValueError("没有可保存的参数")

    stamp = timestamp or datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    backup_dir = package_root / "logs" / "backups" / stamp
    backup_dir.mkdir(parents=True, exist_ok=False)
    original_text: dict[Path, str] = {}
    new_text: dict[Path, str] = {}
    backup_files: dict[Path, Path] = {}
    temp_files: dict[Path, Path] = {}
    changed: list[Path] = []

    try:
        # 即使本次只改一个归属文件，也固定备份两份受管配置，便于人工复核
        # 和跨文件故障恢复。
        for relative in (NAVIGATION_YAML, RPP_YAML):
            source = (package_root / relative).resolve()
            backup = backup_dir / relative
            backup.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, backup)
            backup_files[source] = backup
        for relative, replacements in by_file.items():
            target = (package_root / relative).resolve()
            if package_root not in target.parents:
                raise ValueError(f"拒绝写出包目录：{target}")
            before = target.read_text(encoding="utf-8")
            after = replace_yaml_scalars(before, replacements)
            before_tree = yaml.safe_load(before)
            after_tree = yaml.safe_load(after)
            before_flat = flatten(before_tree)
            after_flat = flatten(after_tree)
            changed_paths = {
                path for path in set(before_flat) | set(after_flat)
                if before_flat.get(path) != after_flat.get(path)
            }
            unexpected = changed_paths - set(replacements)
            if unexpected:
                paths = ", ".join(".".join(path) for path in sorted(unexpected))
                raise RuntimeError(f"语义复核发现越界改动：{paths}")
            for path, expected in replacements.items():
                if not equivalent(get_path(after_tree, path), expected):
                    raise RuntimeError(f"语义复核失败：{'.'.join(path)}")
            original_text[target] = before
            new_text[target] = after
            descriptor, temp_name = tempfile.mkstemp(prefix=f".{target.name}.", dir=target.parent)
            try:
                with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                    stream.write(after)
                    stream.flush()
                    os.fsync(stream.fileno())
            except Exception:
                Path(temp_name).unlink(missing_ok=True)
                raise
            temp_files[target] = Path(temp_name)

        for target, temporary in temp_files.items():
            os.replace(temporary, target)
            changed.append(target)
        for target, after in new_text.items():
            if yaml.safe_load(target.read_text(encoding="utf-8")) != yaml.safe_load(after):
                raise RuntimeError(f"写入后语义复核失败：{target}")
    except Exception:
        for temporary in temp_files.values():
            temporary.unlink(missing_ok=True)
        for target in changed:
            backup = backup_files[target]
            descriptor, restore_name = tempfile.mkstemp(
                prefix=f".{target.name}.restore.", dir=target.parent
            )
            os.close(descriptor)
            restore = Path(restore_name)
            try:
                shutil.copy2(backup, restore)
                os.replace(restore, target)
            finally:
                restore.unlink(missing_ok=True)
        raise

    diff_parts: list[str] = []
    for target in changed:
        relative = target.relative_to(package_root)
        diff_parts.extend(
            difflib.unified_diff(
                original_text[target].splitlines(keepends=True),
                new_text[target].splitlines(keepends=True),
                fromfile=f"a/{relative}",
                tofile=f"b/{relative}",
            )
        )
    return SaveResult(backup_dir, tuple(changed), "".join(diff_parts))


class RateTracker:
    """使用单调时钟记录消息频率、年龄与周期分位数。"""

    def __init__(self, max_samples: int = 512) -> None:
        self.times: deque[float] = deque(maxlen=max_samples)

    def add(self, now: float) -> None:
        self.times.append(float(now))

    def summary(self, now: float | None = None) -> dict[str, float | int | None]:
        intervals = [b - a for a, b in zip(self.times, list(self.times)[1:]) if b >= a]
        current = float(now) if now is not None else (self.times[-1] if self.times else 0.0)
        return {
            "count": len(self.times),
            "hz": (len(intervals) / sum(intervals)) if intervals and sum(intervals) > 0 else None,
            "age": (current - self.times[-1]) if self.times else None,
            "period_p50": percentile(intervals, 50.0),
            "period_p95": percentile(intervals, 95.0),
            "period_p99": percentile(intervals, 99.0),
        }


def percentile(values: Iterable[float], quantile: float) -> float | None:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * quantile / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def laser_counts(ranges: Sequence[float], range_min: float, range_max: float) -> dict[str, Any]:
    finite = [float(value) for value in ranges if math.isfinite(value)]
    valid = [value for value in finite if range_min <= value <= range_max]
    return {
        "total": len(ranges),
        "valid": len(valid),
        "inf": sum(math.isinf(value) for value in ranges),
        "nan": sum(math.isnan(value) for value in ranges),
        "nearest": min(valid) if valid else None,
    }


def path_length(points: Sequence[tuple[float, float]]) -> float:
    return sum(math.hypot(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(points, points[1:]))


def densify_path(points: Sequence[tuple[float, float]], step: float) -> list[tuple[float, float]]:
    if step <= 0:
        raise ValueError("加密步长必须大于 0")
    if not points:
        return []
    dense = [points[0]]
    for start, end in zip(points, points[1:]):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        segments = max(1, math.ceil(distance / step))
        dense.extend(
            (
                start[0] + (end[0] - start[0]) * index / segments,
                start[1] + (end[1] - start[1]) * index / segments,
            )
            for index in range(1, segments + 1)
        )
    return dense


def conservative_clearance(
    path: Sequence[tuple[float, float]],
    lethal_cells: Sequence[tuple[float, float]],
    resolution: float,
) -> float | None:
    """按半格加密路径，计算到致命栅格方形边界的保守最小间距。"""

    if not path or not lethal_cells or resolution <= 0:
        return None
    half = resolution / 2.0
    closest = math.inf
    for x, y in densify_path(path, half):
        for cell_x, cell_y in lethal_cells:
            dx = max(abs(x - cell_x) - half, 0.0)
            dy = max(abs(y - cell_y) - half, 0.0)
            closest = min(closest, math.hypot(dx, dy))
    return closest if math.isfinite(closest) else None


def package_root_from_module() -> Path:
    """兼容源码运行和 ``--symlink-install``。"""

    return Path(__file__).resolve().parents[1]
