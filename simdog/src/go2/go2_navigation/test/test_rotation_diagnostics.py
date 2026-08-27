"""终点旋转只读诊断的纯计算单元测试。"""

import math

from action_msgs.msg import GoalStatus
from go2_navigation.rotation_diagnostics import (
    PoseSample,
    TransformSample,
    TwistSample,
    acquisition_deadline,
    direction_flips,
    evaluate_manual,
    evaluate_navigation,
    navigation_sampling_complete,
    stationary_rotation_episodes,
    unwrapped_delta,
)


def _twist(
    wz: float,
    *,
    terminal: bool = True,
    vx: float = 0.0,
    stamp: float = 1.0,
    yaw_error=None,
):
    return TwistSample(stamp, vx, 0.0, wz, terminal, yaw_error)


def _manual_chain(wz: float):
    return {
        topic: [_twist(wz), _twist(wz, stamp=2.0)]
        for topic in (
            "/cmd_vel_teleop",
            "/cmd_vel_switched",
            "/cmd_vel_smoothed",
            "/cmd_vel",
        )
    }


def _pose(
    yaw: float,
    wz: float = 0.4,
    x: float = 0.0,
    y: float = 0.0,
    stamp: float = 1.0,
):
    return PoseSample(stamp, x, y, yaw, wz)


def _tf(map_yaw: float, base_yaw: float, stamp: float = 1.0):
    return TransformSample(stamp, 0.0, 0.0, map_yaw, 0.0, 0.0, base_yaw)


def test_angle_delta_handles_wrap_and_flip_deadband():
    delta = unwrapped_delta([3.10, -3.10, -3.00])
    assert math.isclose(delta, 0.183185307, abs_tol=1.0e-6)
    assert direction_flips([0.01, 0.2, 0.01, -0.2, -0.01, -0.3]) == 1


def test_manual_evaluation_accepts_consistent_safe_chain():
    twists = _manual_chain(0.45)
    odom = [_pose(0.0, 0.40), _pose(0.4, 0.40, stamp=2.0)]
    truth = [_pose(0.0), _pose(0.4, stamp=2.0)]
    transforms = [_tf(0.0, 0.0), _tf(0.0, 0.4, 2.0)]

    result = evaluate_manual(
        twists, odom, truth, transforms, [False, False], 0.45
    )
    assert result.passed


def test_manual_evaluation_detects_bottom_control_deadzone():
    result = evaluate_manual(
        _manual_chain(0.45),
        [_pose(0.0, 0.05), _pose(0.05, 0.05, stamp=2.0)],
        [_pose(0.0), _pose(0.05, stamp=2.0)],
        [_tf(0.0, 0.0), _tf(0.0, 0.05, 2.0)],
        [False],
        0.45,
    )
    assert not result.passed
    assert any("70%" in failure for failure in result.failures)


def test_manual_evaluation_only_uses_active_final_command_window():
    twists = _manual_chain(0.45)
    twists["/cmd_vel"] = [
        TwistSample(2.0, 0.0, 0.0, 0.45, True),
        TwistSample(5.0, 0.0, 0.0, 0.45, True),
    ]
    odom = [
        PoseSample(1.0, 1.0, 0.0, 0.0, 0.0),
        PoseSample(2.0, 0.0, 0.0, 0.0, 0.40),
        PoseSample(5.0, 0.05, 0.0, 1.30, 0.40),
        PoseSample(6.0, 1.0, 0.0, 1.30, 0.0),
    ]
    truth = list(odom)
    transforms = [
        TransformSample(2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        TransformSample(5.0, 0.0, 0.0, 0.0, 0.05, 0.0, 1.30),
    ]

    result = evaluate_manual(
        twists, odom, truth, transforms, [False], 0.45
    )
    assert result.passed


def test_manual_evaluation_uses_common_feedback_time_intersection():
    twists = _manual_chain(0.45)
    odom = [
        PoseSample(1.0, 0.0, 0.0, 0.0, 0.40),
        PoseSample(2.0, 0.0, 0.0, 0.4, 0.40),
    ]
    truth = [
        PoseSample(0.9, 0.0, 0.0, -0.04, 0.40),
        PoseSample(1.0, 0.0, 0.0, 0.0, 0.40),
        PoseSample(2.0, 0.0, 0.0, 0.4, 0.40),
    ]
    transforms = [
        _tf(0.0, 0.0, 1.0),
        _tf(0.0, 0.4, 2.0),
        _tf(0.0, 0.44, 2.1),
    ]

    result = evaluate_manual(
        twists, odom, truth, transforms, [False], 0.45
    )
    assert result.passed


def test_navigation_evaluation_accepts_latched_pure_rotation():
    topics = {
        name: [_twist(0.45), _twist(0.30)]
        for name in (
            "/cmd_vel_nav",
            "/cmd_vel_switched",
            "/cmd_vel_smoothed",
            "/cmd_vel",
        )
    }
    result = evaluate_navigation(
        topics,
        [_tf(0.0, 0.0, 1.0), _tf(0.01, 0.2, 2.0)],
        [False],
        [0.5],
        1.0,
        0.20,
        0.10,
        0.20,
        0.10,
        0.02,
        0.0,
        0.30,
        0.15,
        action_status=GoalStatus.STATUS_SUCCEEDED,
    )
    assert result.passed


def test_navigation_evaluation_detects_replan_linear_pulse_and_flip():
    topics = {
        name: [_twist(0.3, vx=0.15), _twist(-0.3)]
        for name in (
            "/cmd_vel_nav",
            "/cmd_vel_switched",
            "/cmd_vel_smoothed",
            "/cmd_vel",
        )
    }
    result = evaluate_navigation(
        topics,
        [_tf(0.0, 0.0, 1.0), _tf(0.2, 0.0, 2.0)],
        [False],
        [1.5],
        1.0,
        0.20,
        0.30,
        0.20,
        0.30,
        0.02,
        0.0,
        0.30,
        0.25,
    )
    assert not result.passed
    assert any("线速度" in failure for failure in result.failures)
    assert any("换向" in failure for failure in result.failures)
    assert any("重规划" in failure for failure in result.failures)
    assert any("AMCL" in failure for failure in result.failures)


def test_navigation_evaluation_detects_rotation_away_from_goal_yaw():
    topics = {
        name: [_twist(-0.3, yaw_error=0.5)]
        for name in (
            "/cmd_vel_nav",
            "/cmd_vel_switched",
            "/cmd_vel_smoothed",
            "/cmd_vel",
        )
    }
    result = evaluate_navigation(
        topics,
        [_tf(0.0, 0.0, 1.0), _tf(0.0, 0.2, 2.0)],
        [False],
        [0.5],
        1.0,
        0.20,
        0.50,
        0.20,
        0.50,
        0.02,
        0.0,
        0.30,
        0.25,
    )
    assert not result.passed
    assert any("增大 yaw 误差" in failure for failure in result.failures)


def test_navigation_evaluation_separates_raw_goal_and_path_endpoint():
    topics = {
        name: [_twist(0.3)]
        for name in (
            "/cmd_vel_nav",
            "/cmd_vel_switched",
            "/cmd_vel_smoothed",
            "/cmd_vel",
        )
    }
    result = evaluate_navigation(
        topics,
        [_tf(0.0, 0.0, 1.0), _tf(0.0, 0.2, 2.0)],
        [False],
        [0.0, 1.0, 2.0],
        2.1,
        0.25,
        0.10,
        0.20,
        0.10,
        0.08,
        0.0,
        0.30,
        0.25,
    )
    assert not result.passed
    assert any("路径末端与原始目标的位置误差" in item
               for item in result.failures)
    assert any("实测频率=1.00 Hz" in item for item in result.details)


def test_navigation_without_terminal_is_incomplete_not_false_goal_failure():
    result = evaluate_navigation(
        {},
        [_tf(0.0, 0.0, 1.0), _tf(0.009, 0.0, 2.0)],
        [False],
        [1.0, 2.0],
        None,
        5.401,
        1.645,
        5.393,
        1.645,
        0.010,
        0.0,
        0.30,
        0.15,
        action_status=GoalStatus.STATUS_EXECUTING,
    )

    assert result.incomplete
    assert not result.failures
    assert any("途中快照" in item for item in result.warnings)
    assert not any("超过目标容差" in item for item in result.failures)


def test_navigation_without_goal_or_path_is_still_incomplete():
    result = evaluate_navigation(
        {}, [], [False], [], None,
        None, None, None, None, None, None, 0.30, 0.15,
    )

    assert result.incomplete
    assert not result.failures
    assert any("无法比较路径末端" in item for item in result.warnings)


def test_navigation_action_abort_is_a_real_failure_before_terminal():
    result = evaluate_navigation(
        {}, [], [False], [], None,
        2.0, 0.5, 2.0, 0.5, 0.01, 0.0, 0.30, 0.15,
        action_status=GoalStatus.STATUS_ABORTED,
    )

    assert result.incomplete
    assert any("ABORTED" in item for item in result.failures)


def test_navigation_action_cancel_is_distinct_from_timeout():
    result = evaluate_navigation(
        {}, [], [False], [], None,
        2.0, 0.5, 2.0, 0.5, 0.01, 0.0, 0.30, 0.15,
        action_status=GoalStatus.STATUS_CANCELED,
    )

    assert result.incomplete
    assert any("CANCELED" in item for item in result.failures)
    assert not any("ABORTED" in item for item in result.failures)


def test_new_goal_restarts_acquisition_deadline():
    assert acquisition_deadline(10.0, None, 120.0) == 130.0
    assert acquisition_deadline(10.0, 80.0, 120.0) == 200.0


def test_success_waits_one_second_for_settled_error():
    assert not navigation_sampling_complete(
        10.9, GoalStatus.STATUS_SUCCEEDED, 10.0, 30.0
    )
    assert navigation_sampling_complete(
        11.0, GoalStatus.STATUS_SUCCEEDED, 10.0, 30.0
    )
    assert navigation_sampling_complete(
        5.0, GoalStatus.STATUS_ABORTED, 5.0, None
    )


def test_stationary_rotation_groups_midroute_episode_and_plan_relation():
    samples = [
        TwistSample(
            stamp, 0.0, 0.0, 0.40, False,
            goal_xy_error=3.0,
            path_heading_error=0.80,
            seconds_since_plan=seconds_since_plan,
        )
        for stamp, seconds_since_plan in (
            (1.0, 0.10),
            (1.1, 0.20),
            (1.2, 0.30),
            (1.3, 0.40),
        )
    ]

    episodes = stationary_rotation_episodes(samples)
    assert len(episodes) == 1
    assert math.isclose(episodes[0].duration, 0.3)
    assert episodes[0].follows_plan_update
    assert episodes[0].max_path_heading_error == 0.80

    result = evaluate_navigation(
        {"/cmd_vel_nav": samples}, [], [False], [], None,
        3.0, 0.5, 3.0, 0.5, 0.01, 0.0, 0.30, 0.15,
        action_status=GoalStatus.STATUS_EXECUTING,
    )
    assert any("普通弯道不应停车" in item for item in result.failures)


def test_stationary_rotation_at_raw_goal_boundary_is_not_midroute():
    samples = [
        TwistSample(
            stamp, 0.0, 0.0, 0.40, False,
            goal_xy_error=0.295,
            path_heading_error=1.0,
            seconds_since_plan=0.5,
        )
        for stamp in (1.0, 1.1, 1.2, 1.3)
    ]

    assert stationary_rotation_episodes(
        samples, goal_xy_tolerance=0.30
    ) == []
