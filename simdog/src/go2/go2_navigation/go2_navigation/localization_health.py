"""二维定位置信度的纯函数与滞回判断。"""

from __future__ import annotations

import math
from collections.abc import Sequence


def amcl_standard_deviations(
        covariance: Sequence[float]) -> tuple[float, float, float]:
    """从 ROS 6x6 位姿协方差提取 x、y、yaw 标准差。"""
    if len(covariance) < 36:
        return math.inf, math.inf, math.inf
    variances = (covariance[0], covariance[7], covariance[35])
    if any(not math.isfinite(value) or value < 0.0 for value in variances):
        return math.inf, math.inf, math.inf
    return tuple(math.sqrt(value) for value in variances)


def amcl_covariance_problem(
        covariance: Sequence[float], max_position_std_m: float,
        max_yaw_std_rad: float) -> str | None:
    """返回可操作的 AMCL 失信原因；健康时返回 ``None``。"""
    std_x, std_y, std_yaw = amcl_standard_deviations(covariance)
    if not all(math.isfinite(value) for value in (std_x, std_y, std_yaw)):
        return "AMCL 协方差无效"
    if max(std_x, std_y) > max_position_std_m or std_yaw > max_yaw_std_rad:
        return (
            "AMCL 定位失信：std(x/y/yaw)=%.2f/%.2f/%.2f，"
            "上限=%.2f m/%.2f rad；请停止目标并重新使用 2D Pose Estimate"
            % (std_x, std_y, std_yaw, max_position_std_m, max_yaw_std_rad)
        )
    return None


class AmclCovarianceHealthTracker:
    """使用不同的失效/恢复阈值避免协方差临界点反复锁速。"""

    def __init__(
            self, lost_position_std_m: float = 0.75,
            lost_yaw_std_rad: float = 0.75,
            recovery_position_std_m: float = 0.55,
            recovery_yaw_std_rad: float = 0.50) -> None:
        self.lost_position_std_m = lost_position_std_m
        self.lost_yaw_std_rad = lost_yaw_std_rad
        self.recovery_position_std_m = recovery_position_std_m
        self.recovery_yaw_std_rad = recovery_yaw_std_rad
        self.healthy = False
        self.reason = "尚未收到 /amcl_pose"

    def update(self, covariance: Sequence[float]) -> bool:
        """更新健康状态；失效立即生效，恢复必须进入更小的恢复区间。"""
        position_limit = (
            self.lost_position_std_m if self.healthy
            else self.recovery_position_std_m
        )
        yaw_limit = (
            self.lost_yaw_std_rad if self.healthy
            else self.recovery_yaw_std_rad
        )
        problem = amcl_covariance_problem(
            covariance, position_limit, yaw_limit)
        self.healthy = problem is None
        self.reason = "AMCL 定位置信度正常" if self.healthy else problem
        return self.healthy
