import math

import pytest

from go2_lidar_scan.scan_metrics import analyze_ranges


def test_classifies_laserscan_contract_values():
    metrics = analyze_ranges(
        [0.5, 0.9, 2.0, 15.0, 16.0, math.inf, -math.inf, math.nan],
        0.9,
        15.0,
    )

    assert metrics.total == 8
    assert metrics.valid == 3
    assert metrics.below_min == 1
    assert metrics.above_max == 1
    assert metrics.positive_inf == 1
    assert metrics.negative_inf == 1
    assert metrics.nan == 1
    assert metrics.invalid == 4
    assert metrics.nearest == pytest.approx(0.9)
    assert metrics.farthest == pytest.approx(15.0)


def test_jump_ratio_ignores_no_return_transitions():
    previous = [1.0, 2.0, math.inf, 3.0]
    current = [1.1, 2.5, 4.0, math.inf]

    metrics = analyze_ranges(
        current, 0.9, 15.0, previous=previous, jump_threshold_m=0.3)

    assert metrics.comparable == 2
    assert metrics.jumps == 1
    assert metrics.jump_ratio == pytest.approx(0.5)


def test_range_max_plus_epsilon_is_reported_as_invalid():
    metrics = analyze_ranges([16.0] * 720, 0.9, 15.0)

    assert metrics.above_max == 720
    assert metrics.invalid == 720
    assert metrics.valid == 0
