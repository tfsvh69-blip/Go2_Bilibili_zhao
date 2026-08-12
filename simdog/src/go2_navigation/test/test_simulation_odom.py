import math

import pytest

from go2_navigation.simulation_odom import normalize_angle, relative_planar_pose


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
