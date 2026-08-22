"""Gazebo 障碍探针的几何判定与统计测试。"""

import math

import numpy as np
import pytest

from go2_navigation.obstacle_probe import (
    FrameObservation,
    _validated_arguments,
    cloud_observation,
    create_parser,
    probe_sdf,
    reliable_detection_min_distance,
    scan_observation,
    summarize_group,
)


def test_scan_observation_uses_surface_normal_distance():
    ranges = [float("inf")] * 9
    angle_min = -0.4
    increment = 0.1
    for index in (3, 4, 5):
        angle = angle_min + index * increment
        ranges[index] = 1.0 / math.cos(angle)
    observation = scan_observation(
        ranges,
        angle_min,
        increment,
        0.9,
        15.0,
        expected_distance=1.0,
        probe_angle=0.0,
        half_width=0.15,
        tolerance=0.02,
    )
    assert observation.detected
    assert observation.measured_distance_m == pytest.approx(1.0)
    assert observation.valid_count == 3


def test_scan_observation_does_not_call_wall_a_probe_detection():
    observation = scan_observation(
        [2.0] * 5,
        -0.2,
        0.1,
        0.9,
        15.0,
        expected_distance=0.5,
        probe_angle=0.0,
        half_width=0.15,
        tolerance=0.05,
    )
    assert not observation.detected
    assert observation.signed_error_m > 1.0


def test_cloud_observation_filters_width_height_and_rotates_probe_axis():
    points = np.asarray(
        [
            [0.0, 0.8, 0.0],
            [0.2, 0.8, 0.0],
            [0.0, 0.8, 0.4],
        ]
    )
    observation = cloud_observation(
        "d435",
        points,
        expected_distance=0.8,
        probe_angle=math.pi / 2.0,
        half_width=0.15,
        half_height=0.25,
        tolerance=0.03,
    )
    assert observation.detected
    assert observation.measured_distance_m == pytest.approx(0.8)


def _frame(detected: bool, arrival: float, error: float | None = 0.01):
    return FrameObservation(
        sensor="scan",
        arrival_monotonic=arrival,
        message_stamp=arrival,
        detected=detected,
        measured_distance_m=None if error is None else 1.0 + error,
        signed_error_m=error,
        valid_count=10,
        inf_count=2,
        nan_count=0,
        tf_ok=True,
    )


def test_group_summary_requires_rate_error_and_tf_pass():
    observations = [_frame(True, index * 0.2) for index in range(19)]
    observations.append(_frame(False, 3.8, None))
    summary = summarize_group("scan", 1.0, 1, observations, 0.95, 0.05, 0)
    assert summary["detection_rate"] == pytest.approx(0.95)
    assert summary["period_p99_s"] == pytest.approx(0.2)
    assert summary["pass"]


def test_group_summary_rejects_physical_contact_even_when_detection_is_good():
    observations = [_frame(True, index * 0.2) for index in range(20)]
    summary = summarize_group("scan", 0.5, 1, observations, 0.95, 0.05, 1)
    assert summary["detection_rate"] == pytest.approx(1.0)
    assert summary["contact_events"] == 1
    assert not summary["pass"]


def test_reliable_distance_requires_every_repeat_group_to_pass():
    summaries = []
    for distance, states in ((1.0, (True, True, True)), (0.9, (True, False, True))):
        for group, passed in enumerate(states, start=1):
            summaries.append(
                {"sensor": "scan", "distance_m": distance, "group": group, "pass": passed}
            )
    assert reliable_detection_min_distance(summaries, "scan", 3) == pytest.approx(1.0)


def test_probe_sdf_contains_contact_plugin_and_rejects_unsafe_name():
    sdf = probe_sdf("go2_probe", 0.3, 0.3, 0.5)
    assert "libgazebo_ros_bumper.so" in sdf
    assert "probe_collision" in sdf
    with pytest.raises(ValueError, match="model_name"):
        probe_sdf("../../unsafe", 0.3, 0.3, 0.5)


def test_argument_validation_rejects_duplicate_sensor_and_nonfinite_pose():
    parser = create_parser()
    duplicated = parser.parse_args(["--sensors", "scan,scan"])
    with pytest.raises(ValueError, match="重复"):
        _validated_arguments(duplicated)

    nonfinite = parser.parse_args(["--probe-angle-deg", "nan"])
    with pytest.raises(ValueError, match="有限数"):
        _validated_arguments(nonfinite)
