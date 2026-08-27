"""NDT 导航级健康滞回测试。"""

from go2_navigation.safety_supervisor import LocalizationHealthTracker
from go2_navigation.localization_health import (
    AmclCovarianceHealthTracker,
    amcl_standard_deviations,
)


def _recover(tracker, start=1.0):
    tracker.update_reinitialization(False)
    for index in range(5):
        tracker.update_alignment(True, start + index * 0.1)


def test_single_rejected_scan_is_degraded_not_lost():
    tracker = LocalizationHealthTracker()
    _recover(tracker)
    assert tracker.state == tracker.HEALTHY
    assert tracker.update_alignment(False, 1.5) == tracker.DEGRADED
    assert tracker.evaluate(2.0) == tracker.DEGRADED


def test_sustained_rejection_becomes_lost_after_grace_period():
    tracker = LocalizationHealthTracker()
    _recover(tracker)
    tracker.update_alignment(False, 1.5)
    assert tracker.evaluate(3.41) == tracker.LOST


def test_reinitialization_request_locks_immediately_and_needs_five_good_samples():
    tracker = LocalizationHealthTracker()
    _recover(tracker)
    tracker.update_reinitialization(True)
    assert tracker.state == tracker.LOST
    tracker.update_reinitialization(False)
    for index in range(4):
        tracker.update_alignment(True, 2.0 + index * 0.1)
    assert tracker.state == tracker.DEGRADED
    tracker.update_alignment(True, 2.4)
    assert tracker.state == tracker.HEALTHY


def _covariance(std_x: float, std_y: float, std_yaw: float) -> list[float]:
    values = [0.0] * 36
    values[0] = std_x ** 2
    values[7] = std_y ** 2
    values[35] = std_yaw ** 2
    return values


def test_amcl_covariance_tracker_locks_large_yellow_fan_and_uses_hysteresis():
    tracker = AmclCovarianceHealthTracker()
    assert tracker.update(_covariance(0.50, 0.50, 0.30)) is True
    assert tracker.update(_covariance(0.60, 0.60, 0.60)) is True
    assert tracker.update(_covariance(0.80, 1.18, 1.56)) is False
    assert "std(x/y/yaw)" in tracker.reason
    assert tracker.update(_covariance(0.60, 0.50, 0.40)) is False
    assert tracker.update(_covariance(0.50, 0.50, 0.40)) is True


def test_amcl_standard_deviations_uses_ros_pose_diagonal():
    std_x, std_y, std_yaw = amcl_standard_deviations(
        _covariance(0.2, 0.3, 0.4))
    assert (round(std_x, 3), round(std_y, 3), round(std_yaw, 3)) == (
        0.2, 0.3, 0.4)
