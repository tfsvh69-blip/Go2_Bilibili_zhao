"""不依赖 ROS 运行时的 LaserScan 统计函数，便于单元测试复用。"""

from dataclasses import dataclass
import math
from typing import Optional, Sequence


@dataclass(frozen=True)
class ScanMetrics:
    """单帧扫描的分类计数与相邻帧跳变统计。"""

    total: int
    valid: int
    positive_inf: int
    negative_inf: int
    nan: int
    below_min: int
    above_max: int
    comparable: int
    jumps: int
    nearest: float
    farthest: float

    @property
    def invalid(self) -> int:
        """返回不符合 LaserScan 量程契约的数量。"""
        return self.negative_inf + self.nan + self.below_min + self.above_max

    @property
    def jump_ratio(self) -> float:
        """返回两帧均为有效有限值的角度中发生大跳变的比例。"""
        if self.comparable == 0:
            return 0.0
        return self.jumps / self.comparable


def _is_valid(value: float, range_min: float, range_max: float) -> bool:
    return math.isfinite(value) and range_min <= value <= range_max


def analyze_ranges(
    ranges: Sequence[float],
    range_min: float,
    range_max: float,
    previous: Optional[Sequence[float]] = None,
    jump_threshold_m: float = 0.30,
) -> ScanMetrics:
    """分类一帧 ranges，并只在两帧都有效时计算同角度距离跳变。"""
    valid_values = []
    positive_inf = 0
    negative_inf = 0
    nan = 0
    below_min = 0
    above_max = 0

    for value in ranges:
        if math.isnan(value):
            nan += 1
        elif math.isinf(value):
            if value > 0.0:
                positive_inf += 1
            else:
                negative_inf += 1
        elif value < range_min:
            below_min += 1
        elif value > range_max:
            above_max += 1
        else:
            valid_values.append(value)

    comparable = 0
    jumps = 0
    if previous is not None and len(previous) == len(ranges):
        for old, new in zip(previous, ranges):
            if _is_valid(old, range_min, range_max) and _is_valid(
                    new, range_min, range_max):
                comparable += 1
                if abs(new - old) > jump_threshold_m:
                    jumps += 1

    return ScanMetrics(
        total=len(ranges),
        valid=len(valid_values),
        positive_inf=positive_inf,
        negative_inf=negative_inf,
        nan=nan,
        below_min=below_min,
        above_max=above_max,
        comparable=comparable,
        jumps=jumps,
        nearest=min(valid_values, default=math.nan),
        farthest=max(valid_values, default=math.nan),
    )
