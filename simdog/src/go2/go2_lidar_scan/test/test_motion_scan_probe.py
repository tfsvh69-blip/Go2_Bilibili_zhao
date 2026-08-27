import math

from geometry_msgs.msg import TransformStamped
import numpy as np

from go2_lidar_scan.motion_scan_probe import (
    project_nearest_bins,
    sensor_and_level_points,
)


def _pitch_transform(pitch: float) -> TransformStamped:
    transform = TransformStamped()
    transform.header.frame_id = "base_footprint"
    transform.child_frame_id = "velodyne"
    transform.transform.translation.z = 0.32
    transform.transform.rotation.y = math.sin(pitch * 0.5)
    transform.transform.rotation.w = math.cos(pitch * 0.5)
    return transform


def test_tilted_sensor_slice_accepts_ground_but_level_slice_rejects_it():
    # 传感器俯仰 5° 时，原始 z=0 的 4 m 点已经落到 base 地面附近。
    points = np.asarray([[4.0, 0.0, 0.0]])
    raw_points, level_points, base_z, _rpy = sensor_and_level_points(
        points, _pitch_transform(math.radians(5.0)))

    raw = project_nearest_bins(
        raw_points, base_z, min_height=-0.05, max_height=0.10)
    level = project_nearest_bins(level_points, base_z)

    assert raw.finite_bins == 1
    assert raw.ground_bins == 1
    assert level.finite_bins == 0
    assert level.ground_bins == 0


def test_projection_keeps_only_nearest_endpoint_in_each_angle_bin():
    points = np.asarray([
        [3.0, 0.0, 0.25],
        [2.0, 0.0, 0.25],
        [0.0, 3.0, 0.25],
    ])
    base_z = np.asarray([0.30, 0.05, 0.30])

    metrics = project_nearest_bins(points, base_z)

    assert metrics.finite_bins == 2
    assert metrics.ground_bins == 1
