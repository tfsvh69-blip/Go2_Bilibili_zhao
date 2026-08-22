#!/usr/bin/env python3
"""重力对齐 TF 的纯算法辅助函数；运行节点由同包 C++ 可执行文件提供。"""

from __future__ import annotations

import math
from typing import Tuple

from geometry_msgs.msg import Quaternion, TransformStamped


def quaternion_rpy(quaternion: Quaternion) -> Tuple[float, float, float]:
    """返回归一化四元数对应的 roll、pitch、yaw。"""
    values = (
        float(quaternion.x),
        float(quaternion.y),
        float(quaternion.z),
        float(quaternion.w),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("四元数包含非有限值")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-12:
        raise ValueError("四元数模长为零")
    x, y, z, w = (value / norm for value in values)

    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return roll, pitch, yaw


def yaw_only_quaternion(quaternion: Quaternion) -> Quaternion:
    """保留输入姿态的 yaw，并把 roll、pitch 精确置零。"""
    _roll, _pitch, yaw = quaternion_rpy(quaternion)
    result = Quaternion()
    result.z = math.sin(yaw * 0.5)
    result.w = math.cos(yaw * 0.5)
    return result


def build_level_transform(
    sensor_transform: TransformStamped,
    level_frame: str,
) -> TransformStamped:
    """由 ``reference -> sensor`` 构造 ``reference -> level`` 变换。"""
    if not sensor_transform.header.frame_id:
        raise ValueError("参考坐标系为空")
    if not level_frame:
        raise ValueError("重力对齐坐标系为空")
    translation = sensor_transform.transform.translation
    translation_values = (
        float(translation.x),
        float(translation.y),
        float(translation.z),
    )
    if not all(math.isfinite(value) for value in translation_values):
        raise ValueError("雷达平移包含非有限值")

    result = TransformStamped()
    result.header.stamp = sensor_transform.header.stamp
    result.header.frame_id = sensor_transform.header.frame_id
    result.child_frame_id = level_frame
    result.transform.translation.x = translation_values[0]
    result.transform.translation.y = translation_values[1]
    result.transform.translation.z = translation_values[2]
    result.transform.rotation = yaw_only_quaternion(
        sensor_transform.transform.rotation)
    return result
