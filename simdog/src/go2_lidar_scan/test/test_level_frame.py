import math

from geometry_msgs.msg import Quaternion, TransformStamped
import pytest

from go2_lidar_scan.level_frame_publisher import (
    build_level_transform,
    quaternion_rpy,
    yaw_only_quaternion,
)


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def test_yaw_only_quaternion_removes_roll_pitch_and_keeps_yaw():
    source = _quaternion_from_rpy(0.12, -0.08, 0.73)
    leveled = yaw_only_quaternion(source)
    roll, pitch, yaw = quaternion_rpy(leveled)

    assert roll == pytest.approx(0.0, abs=1.0e-12)
    assert pitch == pytest.approx(0.0, abs=1.0e-12)
    assert yaw == pytest.approx(0.73, abs=1.0e-12)


def test_level_transform_preserves_stamp_parent_and_translation():
    source = TransformStamped()
    source.header.stamp.sec = 42
    source.header.stamp.nanosec = 123
    source.header.frame_id = "base_footprint"
    source.child_frame_id = "velodyne"
    source.transform.translation.x = 0.20
    source.transform.translation.y = -0.01
    source.transform.translation.z = 0.32
    source.transform.rotation = _quaternion_from_rpy(0.10, -0.07, 0.20)

    result = build_level_transform(source, "velodyne_level")

    assert result.header == source.header
    assert result.child_frame_id == "velodyne_level"
    assert result.transform.translation == source.transform.translation
    roll, pitch, yaw = quaternion_rpy(result.transform.rotation)
    assert roll == pytest.approx(0.0, abs=1.0e-12)
    assert pitch == pytest.approx(0.0, abs=1.0e-12)
    assert yaw == pytest.approx(0.20, abs=1.0e-12)


def test_invalid_quaternion_and_translation_are_rejected():
    zero = Quaternion()
    zero.w = 0.0
    with pytest.raises(ValueError, match="模长为零"):
        yaw_only_quaternion(zero)

    source = TransformStamped()
    source.header.frame_id = "base_footprint"
    source.transform.rotation.w = 1.0
    source.transform.translation.x = math.nan
    with pytest.raises(ValueError, match="平移"):
        build_level_transform(source, "velodyne_level")
