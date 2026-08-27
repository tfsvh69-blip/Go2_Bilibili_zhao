import math

import pytest

from go2_navigation.simulation_odom import (
    AngularVelocityWindowFilter,
    normalize_angle,
    relative_planar_pose,
)


def test_relative_planar_pose_removes_spawn_offset():
    pose = relative_planar_pose((3.0, -2.0, 0.0), (3.8, -1.7, 0.2))
    assert pose == pytest.approx((0.8, 0.3, 0.2))


def test_relative_planar_pose_rotates_into_initial_heading():
    pose = relative_planar_pose(
        (3.0, 2.0, math.pi / 2.0),
        (3.0, 3.0, math.pi),
    )
    assert pose == pytest.approx((1.0, 0.0, math.pi / 2.0), abs=1.0e-9)


def test_normalize_angle_wraps_positive_pi():
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)


def test_angular_velocity_filter_tracks_rotation_across_yaw_wrap():
    velocity_filter = AngularVelocityWindowFilter(
        window_duration=1.0, minimum_duration=0.5
    )
    values = []
    for index in range(21):
        stamp = index * 0.1
        wrapped_yaw = normalize_angle(3.0 + 0.2 * stamp)
        values.append(velocity_filter.update(stamp, wrapped_yaw))

    assert values[-1] == pytest.approx(0.2, abs=1.0e-9)


def test_angular_velocity_filter_rejects_footfall_sway():
    velocity_filter = AngularVelocityWindowFilter(
        window_duration=1.0, minimum_duration=0.5
    )
    values = []
    for index in range(41):
        stamp = index * 0.1
        # 0.3 rad/s 是真实转向趋势；0.08 rad 是约 2.5 Hz 的机身落足摆动。
        yaw = 0.3 * stamp + 0.08 * math.sin(5.0 * math.pi * stamp)
        values.append(velocity_filter.update(stamp, yaw))

    assert values[-1] == pytest.approx(0.3, abs=0.02)


def test_angular_velocity_filter_resets_when_sim_time_moves_backwards():
    velocity_filter = AngularVelocityWindowFilter(
        window_duration=1.0, minimum_duration=0.5
    )
    velocity_filter.update(1.0, 0.0)
    velocity_filter.update(2.0, 0.4)

    assert velocity_filter.update(0.0, 1.0) == 0.0
