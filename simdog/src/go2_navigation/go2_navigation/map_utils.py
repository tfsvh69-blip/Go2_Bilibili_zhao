#!/usr/bin/env python3
"""导航地图包的纯 Python 校验与栅格查询工具。"""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image
import yaml


MAP_BUNDLE_SCHEMA_VERSION = 1
# 阶段一联调默认值：只保留很小的栅格/边界余量，实际碰撞安全继续由机器人
# footprint、Nav2 costmap 与 collision_monitor 负责。闭环稳定后再按实测收紧。
COMMISSIONING_CLEARANCE_M = 0.10
MAP_FILES = {
    "GlobalMap.pcd": "lidar_localization_map",
    "map.yaml": "nav2_occupancy_grid_metadata",
    "map.pgm": "nav2_occupancy_grid_image",
    "map_stats.json": "nav2_occupancy_grid_statistics",
}


class MapValidationError(ValueError):
    """地图包或栅格语义不满足导航要求。"""


def safe_child_path(parent: Path, relative_path: str) -> Path:
    """解析地图包中的相对路径，并拒绝目录逃逸。"""
    if not relative_path or os.path.isabs(relative_path):
        raise MapValidationError("地图包文件路径必须是非空相对路径")
    candidate = (parent / relative_path).resolve()
    try:
        candidate.relative_to(parent.resolve())
    except ValueError as exc:
        raise MapValidationError("地图包文件路径不能离开 map_dir") from exc
    return candidate


@dataclass(frozen=True)
class StaticMap:
    """从 Nav2 map.yaml/map.pgm 加载的只读二维占据栅格。"""

    image: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    free_threshold: int
    occupied_threshold: int

    @property
    def width(self) -> int:
        return int(self.image.shape[1])

    @property
    def height(self) -> int:
        return int(self.image.shape[0])

    @property
    def min_x(self) -> float:
        return self.origin_x

    @property
    def max_x(self) -> float:
        return self.origin_x + self.width * self.resolution

    @property
    def min_y(self) -> float:
        return self.origin_y

    @property
    def max_y(self) -> float:
        return self.origin_y + self.height * self.resolution

    def world_to_image(self, x: float, y: float) -> tuple[int, int] | None:
        """把 map 坐标转换为 PGM 行列；越界时返回 ``None``。"""
        col = int(math.floor((x - self.origin_x) / self.resolution))
        map_row = int(math.floor((y - self.origin_y) / self.resolution))
        row = self.height - 1 - map_row
        if 0 <= row < self.height and 0 <= col < self.width:
            return row, col
        return None

    def pixel_value(self, x: float, y: float) -> int | None:
        cell = self.world_to_image(x, y)
        return None if cell is None else int(self.image[cell])

    def is_known_free(self, x: float, y: float) -> bool:
        value = self.pixel_value(x, y)
        return value is not None and value >= self.free_threshold

    def clearance_m(self, x: float, y: float) -> float:
        """返回到已知障碍或地图边界的保守距离。"""
        if self.world_to_image(x, y) is None:
            return 0.0
        boundary = min(x - self.min_x, self.max_x - x,
                       y - self.min_y, self.max_y - y)
        occupied_rows, occupied_cols = np.nonzero(
            self.image <= self.occupied_threshold)
        if occupied_rows.size == 0:
            return max(0.0, boundary)
        obstacle_x = self.origin_x + (occupied_cols + 0.5) * self.resolution
        obstacle_y = self.origin_y + (
            self.height - occupied_rows - 0.5) * self.resolution
        obstacle_distance = np.hypot(obstacle_x - x, obstacle_y - y).min()
        return max(0.0, min(boundary, float(obstacle_distance)))

    def validate_pose(self, x: float, y: float, clearance_m: float) -> str | None:
        """返回不可用原因；合法位置返回 ``None``。"""
        if not (math.isfinite(x) and math.isfinite(y)):
            return "目标坐标必须是有限数值"
        if self.world_to_image(x, y) is None:
            return (
                "目标超出地图边界："
                f"x=[{self.min_x:.2f}, {self.max_x:.2f})，"
                f"y=[{self.min_y:.2f}, {self.max_y:.2f})"
            )
        if not self.is_known_free(x, y):
            return "目标不是已知自由栅格（可能是障碍或 unknown）"
        actual_clearance = self.clearance_m(x, y)
        if actual_clearance < clearance_m:
            return (
                f"目标安全余量不足：{actual_clearance:.2f} m，"
                f"要求至少 {clearance_m:.2f} m"
            )
        return None


def load_static_map(map_dir: str | Path) -> StaticMap:
    """加载并验证 ``map.yaml`` 所引用的 PGM 地图。"""
    map_root = Path(map_dir).expanduser().resolve()
    yaml_path = map_root / "map.yaml"
    if not yaml_path.is_file():
        raise MapValidationError(f"缺少地图元数据：{yaml_path}")
    try:
        metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise MapValidationError(f"map.yaml YAML 格式错误：{exc}") from exc
    if not isinstance(metadata, dict):
        raise MapValidationError("map.yaml 根节点必须是对象")

    image_path = safe_child_path(map_root, str(metadata.get("image", "")))
    if image_path.name != "map.pgm" or not image_path.is_file():
        raise MapValidationError("map.yaml 必须引用 map.pgm")
    try:
        resolution = float(metadata["resolution"])
        origin = metadata["origin"]
        origin_x, origin_y = float(origin[0]), float(origin[1])
        negate = int(metadata.get("negate", 0))
        occupied_thresh = float(metadata.get("occupied_thresh", 0.65))
        free_thresh = float(metadata.get("free_thresh", 0.196))
    except (KeyError, IndexError, TypeError, ValueError) as exc:
        raise MapValidationError("map.yaml 缺少有效的 resolution/origin/阈值") from exc
    if not (math.isfinite(resolution) and resolution > 0.0):
        raise MapValidationError("map.yaml resolution 必须大于 0")
    if negate != 0:
        raise MapValidationError("当前导航仅支持 negate: 0 的 PGM 地图")
    if not (0.0 <= free_thresh < occupied_thresh <= 1.0):
        raise MapValidationError("map.yaml 的 free/occupied 阈值无效")

    image = np.asarray(Image.open(image_path).convert("L"), dtype=np.uint8)
    if image.ndim != 2 or image.size == 0:
        raise MapValidationError("map.pgm 必须是非空灰度图")
    return StaticMap(
        image=image,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        free_threshold=int(math.ceil((1.0 - free_thresh) * 255.0)),
        occupied_threshold=int(math.floor((1.0 - occupied_thresh) * 255.0)),
    )


def occupancy_grid_to_static_map(
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        data: list[int] | tuple[int, ...] | np.ndarray,
) -> StaticMap:
    """把 ROS ``OccupancyGrid`` 的栅格数据转成可共用的地图查询对象。

    ROS 栅格从左下角开始按行存储，PGM 从左上角开始，因此需要
    垂直翻转。``-1`` unknown 保留为 205，不会被误判为自由区。
    """
    if width <= 0 or height <= 0:
        raise MapValidationError("动态地图宽高必须大于 0")
    if not (math.isfinite(resolution) and resolution > 0.0):
        raise MapValidationError("动态地图 resolution 必须大于 0")
    cells = np.asarray(data, dtype=np.int16)
    if cells.size != width * height:
        raise MapValidationError(
            f"动态地图数据长度 {cells.size} 与 {width}×{height} 不匹配")
    cells = cells.reshape((height, width))
    image = np.full((height, width), 205, dtype=np.uint8)
    known = cells >= 0
    clipped = np.clip(cells, 0, 100)
    image[known] = np.rint((100 - clipped[known]) * 2.54).astype(np.uint8)
    image = np.flipud(image)
    return StaticMap(
        image=image,
        resolution=float(resolution),
        origin_x=float(origin_x),
        origin_y=float(origin_y),
        free_threshold=206,
        occupied_threshold=89,
    )


def load_bundle_metadata(map_dir: str | Path) -> dict[str, Any]:
    """读取新版本地图包清单，不进行哈希校验。"""
    bundle_path = Path(map_dir).expanduser().resolve() / "map_bundle.yaml"
    if not bundle_path.is_file():
        raise MapValidationError("缺少 map_bundle.yaml，请先运行 build_map_bundle")
    try:
        bundle = yaml.safe_load(bundle_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise MapValidationError(f"map_bundle.yaml YAML 格式错误：{exc}") from exc
    if not isinstance(bundle, dict):
        raise MapValidationError("map_bundle.yaml 根节点必须是对象")
    return bundle
