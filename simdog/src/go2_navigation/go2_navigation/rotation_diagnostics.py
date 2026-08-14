#!/usr/bin/env python3
"""只读分析 Go2 纯旋转执行链或 Nav2 终点定向链。

本工具只订阅话题和查询 TF，不发布目标、不发布速度，也不修改参数。手动模式
应与经 ``/cmd_vel_teleop`` 注入的纯旋转命令同时运行；导航模式应在目标进入
终点区域前启动，并会在收到每个新目标后重新计算等待期限。
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import dataclass
import math
import statistics
import time
from typing import DefaultDict, Iterable, Optional

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_action_status_default,
)
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


TWIST_TOPICS = (
    "/cmd_vel_teleop",
    "/cmd_vel_nav",
    "/cmd_vel_switched",
    "/cmd_vel_smoothed",
    "/cmd_vel",
)


def normalize_angle(angle: float) -> float:
    """把角度归一化到 ``[-pi, pi]``。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(orientation) -> float:
    """从 ROS 四元数提取平面 yaw。"""
    return math.atan2(
        2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0
        - 2.0
        * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


def unwrapped_delta(values: Iterable[float]) -> float:
    """计算可跨越 ``±pi`` 的首尾累计角变化。"""
    values = list(values)
    return sum(
        normalize_angle(current - previous)
        for previous, current in zip(values, values[1:])
    )


def direction_flips(values: Iterable[float], threshold: float = 0.03) -> int:
    """统计忽略小角速度后的正负方向切换次数。"""
    signs: list[int] = []
    for value in values:
        if abs(value) < threshold:
            continue
        sign = 1 if value > 0.0 else -1
        if not signs or signs[-1] != sign:
            signs.append(sign)
    return max(0, len(signs) - 1)


def maximum_step(values: Iterable[float], angular: bool = False) -> float:
    """返回相邻标量样本的最大变化量。"""
    values = list(values)
    if angular:
        changes = [
            abs(normalize_angle(current - previous))
            for previous, current in zip(values, values[1:])
        ]
    else:
        changes = [
            abs(current - previous)
            for previous, current in zip(values, values[1:])
        ]
    return max(changes, default=0.0)


@dataclass(frozen=True)
class TwistSample:
    wall_time: float
    linear_x: float
    linear_y: float
    angular_z: float
    terminal: bool
    yaw_error: Optional[float] = None
    goal_xy_error: Optional[float] = None
    path_heading_error: Optional[float] = None
    seconds_since_plan: Optional[float] = None


@dataclass(frozen=True)
class PoseSample:
    wall_time: float
    x: float
    y: float
    yaw: float
    angular_z: float = 0.0


@dataclass(frozen=True)
class TransformSample:
    wall_time: float
    map_odom_x: float
    map_odom_y: float
    map_odom_yaw: float
    odom_base_x: float
    odom_base_y: float
    odom_base_yaw: float


@dataclass(frozen=True)
class StationaryRotationEpisode:
    """终点外线速度近零、角速度非零的一段连续控制输出。"""

    start_time: float
    end_time: float
    max_angular_z: float
    min_goal_xy_error: Optional[float]
    max_path_heading_error: Optional[float]
    follows_plan_update: bool

    @property
    def duration(self) -> float:
        return max(0.0, self.end_time - self.start_time)


@dataclass
class Evaluation:
    failures: list[str]
    warnings: list[str]
    details: list[str]
    incomplete: bool = False

    @property
    def passed(self) -> bool:
        return not self.failures and not self.incomplete


ACTION_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def action_status_name(status: Optional[int]) -> str:
    """把 action_msgs 状态转换为稳定的诊断文本。"""
    if status is None:
        return "UNOBSERVED"
    return ACTION_STATUS_NAMES.get(status, f"UNKNOWN({status})")


def acquisition_deadline(
    started_at: float,
    goal_received_at: Optional[float],
    acquire_timeout: float,
) -> float:
    """新目标到达后重新开始计算终点获取超时。"""
    return (goal_received_at if goal_received_at is not None else started_at) + acquire_timeout


def navigation_sampling_complete(
    now: float,
    action_status: Optional[int],
    action_finished_time: Optional[float],
    terminal_deadline: Optional[float],
    settle_time: float = 1.0,
) -> bool:
    """判断导航采样是否已得到终态或达到终点阶段上限。"""
    if action_status in (
        GoalStatus.STATUS_ABORTED,
        GoalStatus.STATUS_CANCELED,
    ):
        return True
    if (
        action_status == GoalStatus.STATUS_SUCCEEDED
        and action_finished_time is not None
        and now >= action_finished_time + settle_time
    ):
        return True
    return terminal_deadline is not None and now >= terminal_deadline


def stationary_rotation_episodes(
    samples: Iterable[TwistSample],
    *,
    linear_threshold: float = 0.01,
    angular_threshold: float = 0.03,
    maximum_gap: float = 0.30,
    minimum_duration: float = 0.20,
    goal_xy_tolerance: Optional[float] = None,
) -> list[StationaryRotationEpisode]:
    """合并终点外的原地旋转样本，忽略单周期过零噪声。"""
    active = [
        sample for sample in samples
        if not sample.terminal
        and abs(sample.linear_x) <= linear_threshold
        and abs(sample.angular_z) >= angular_threshold
    ]
    if not active:
        return []

    groups: list[list[TwistSample]] = [[active[0]]]
    for sample in active[1:]:
        if sample.wall_time - groups[-1][-1].wall_time <= maximum_gap:
            groups[-1].append(sample)
        else:
            groups.append([sample])

    episodes: list[StationaryRotationEpisode] = []
    for group in groups:
        duration = group[-1].wall_time - group[0].wall_time
        if duration < minimum_duration:
            continue
        goal_errors = [
            sample.goal_xy_error for sample in group
            if sample.goal_xy_error is not None
        ]
        min_goal_xy_error = min(goal_errors) if goal_errors else None
        # 双终点判定可能因路径端点的厘米级偏移稍晚于原始目标判定。
        # 只要这一段已经碰到原始目标 XY 容差，就属于终点边界转向，
        # 不能按“途中普通弯道停车”处理。
        if (
            goal_xy_tolerance is not None
            and min_goal_xy_error is not None
            and min_goal_xy_error <= goal_xy_tolerance
        ):
            continue
        path_errors = [
            abs(sample.path_heading_error) for sample in group
            if sample.path_heading_error is not None
        ]
        episodes.append(StationaryRotationEpisode(
            start_time=group[0].wall_time,
            end_time=group[-1].wall_time,
            max_angular_z=max(abs(sample.angular_z) for sample in group),
            min_goal_xy_error=min_goal_xy_error,
            max_path_heading_error=max(path_errors) if path_errors else None,
            follows_plan_update=any(
                sample.seconds_since_plan is not None
                and sample.seconds_since_plan <= 0.30
                for sample in group
            ),
        ))
    return episodes


def _median(values: Iterable[float]) -> float:
    values = list(values)
    return float(statistics.median(values)) if values else 0.0


def _pose_displacement(samples: list[PoseSample]) -> float:
    if len(samples) < 2:
        return 0.0
    return math.hypot(
        samples[-1].x - samples[0].x,
        samples[-1].y - samples[0].y,
    )


def _within_command_window(samples, start_time: float, end_time: float):
    """只保留最终速度实际非零期间的反馈样本。"""
    return [
        sample for sample in samples
        if start_time <= sample.wall_time <= end_time
    ]


def evaluate_manual(
    twists: dict[str, list[TwistSample]],
    odom: list[PoseSample],
    ground_truth: list[PoseSample],
    transforms: list[TransformSample],
    pause_values: list[bool],
    expected_wz: float,
) -> Evaluation:
    """按纯旋转基线标准评估采样结果。"""
    failures: list[str] = []
    warnings: list[str] = []
    details: list[str] = []

    final_samples = twists.get("/cmd_vel", [])
    active_final = [
        sample for sample in final_samples
        if abs(sample.angular_z) >= 0.03
    ]
    if not active_final:
        failures.append("/cmd_vel 未观测到有效旋转命令")
        command_start = float("-inf")
        command_end = float("inf")
    else:
        command_start = active_final[0].wall_time
        command_end = active_final[-1].wall_time
        median_final = _median(sample.angular_z for sample in active_final)
        command_error = abs(median_final - expected_wz)
        details.append(
            f"/cmd_vel 稳态中位角速度={median_final:.3f} rad/s，"
            f"命令误差={command_error:.3f} rad/s"
        )
        if command_error > 0.02:
            failures.append(
                "最终 /cmd_vel 与 --expected-wz 相差超过 0.02 rad/s"
            )
        max_linear = max(
            max(abs(sample.linear_x), abs(sample.linear_y))
            for sample in active_final
        )
        details.append(f"纯旋转最大平移命令={max_linear:.3f} m/s")
        if max_linear > 0.01:
            failures.append("纯旋转期间最终速度含有超过 0.01 m/s 的平移分量")

        manual_chain = (
            "/cmd_vel_teleop",
            "/cmd_vel_switched",
            "/cmd_vel_smoothed",
            "/cmd_vel",
        )
        chain_medians: list[tuple[str, float]] = []
        for topic in manual_chain:
            active = [
                sample.angular_z for sample in twists.get(topic, [])
                if abs(sample.angular_z) >= 0.03
            ]
            if not active:
                failures.append(f"{topic} 未观测到有效旋转命令")
                continue
            chain_medians.append((topic, _median(active)))
        if chain_medians:
            details.append(
                "手动速度链中位值：" + ", ".join(
                    f"{topic}={value:.3f}" for topic, value in chain_medians
                ) + " rad/s"
            )

        flips = direction_flips(
            sample.angular_z for sample in active_final
        )
        details.append(f"纯旋转最终速度换向={flips} 次")
        if flips:
            failures.append(f"纯旋转最终角速度发生 {flips} 次换向")

    odom_in_command = _within_command_window(
        odom, command_start, command_end
    )
    truth_in_command = _within_command_window(
        ground_truth, command_start, command_end
    )
    transforms_in_command = _within_command_window(
        transforms, command_start, command_end
    )
    # 三路反馈频率不同（真值/odom 约 50 Hz，TF 诊断采样 20 Hz）。使用
    # 共同时间交集，避免仅因首尾样本相差一个周期而虚报累计 yaw 误差。
    if odom_in_command and truth_in_command and transforms_in_command:
        common_start = max(
            odom_in_command[0].wall_time,
            truth_in_command[0].wall_time,
            transforms_in_command[0].wall_time,
        )
        common_end = min(
            odom_in_command[-1].wall_time,
            truth_in_command[-1].wall_time,
            transforms_in_command[-1].wall_time,
        )
        odom_in_command = _within_command_window(
            odom_in_command, common_start, common_end
        )
        truth_in_command = _within_command_window(
            truth_in_command, common_start, common_end
        )
        transforms_in_command = _within_command_window(
            transforms_in_command, common_start, common_end
        )
    actual_active = [
        sample.angular_z for sample in odom
        if command_start <= sample.wall_time <= command_end
        and abs(sample.angular_z) >= 0.02
    ]
    actual_wz = _median(actual_active)
    gain = abs(actual_wz / expected_wz) if expected_wz else 0.0
    details.append(
        f"/odom 稳态中位角速度={actual_wz:.3f} rad/s，执行增益={gain:.1%}"
    )
    if expected_wz * actual_wz <= 0.0:
        failures.append("/odom 角速度方向与请求方向不一致或机体未旋转")
    if abs(expected_wz) >= 0.35 and gain < 0.70:
        failures.append("0.35/0.45 rad/s 档的实际角速度未达到命令的 70%")
    elif abs(expected_wz) < 0.35 and gain < 0.70:
        warnings.append("低速档执行增益低于 70%，可能处于四足旋转死区")

    odom_delta = unwrapped_delta(sample.yaw for sample in odom_in_command)
    truth_delta = unwrapped_delta(sample.yaw for sample in truth_in_command)
    tf_delta = unwrapped_delta(
        sample.odom_base_yaw for sample in transforms_in_command
    )
    details.append(
        "累计 yaw："
        f"ground_truth={truth_delta:.3f}，odom={odom_delta:.3f}，"
        f"TF={tf_delta:.3f} rad"
    )
    if not odom_in_command or not truth_in_command or not transforms_in_command:
        failures.append("缺少 /odom、/odom/ground_truth 或 odom→base_footprint 样本")
    elif max(
        abs(odom_delta - truth_delta),
        abs(tf_delta - truth_delta),
    ) > 0.03:
        failures.append("里程计/TF 与 Gazebo 真值累计 yaw 相差超过 0.03 rad")

    drift = _pose_displacement(truth_in_command)
    drift_per_quarter_turn = (
        drift * (math.pi / 2.0) / abs(truth_delta)
        if abs(truth_delta) >= 0.2 else drift
    )
    details.append(
        f"机身平移漂移={drift:.3f} m，折算每 90°={drift_per_quarter_turn:.3f} m"
    )
    if abs(truth_delta) >= 0.2 and drift_per_quarter_turn > 0.10:
        failures.append("纯旋转折算每 90° 的平移漂移超过 0.10 m")

    if not pause_values:
        warnings.append("未收到 /pause_navigation，无法确认安全锁状态")
    elif any(pause_values):
        failures.append("采样期间 /pause_navigation 曾为 true，速度链结果无效")
    else:
        details.append("/pause_navigation 全程为 false")
    return Evaluation(failures, warnings, details)


def evaluate_navigation(
    twists: dict[str, list[TwistSample]],
    transforms: list[TransformSample],
    pause_values: list[bool],
    plan_times: list[float],
    first_terminal_time: Optional[float],
    final_goal_xy_error: Optional[float],
    final_goal_yaw_error: Optional[float],
    final_path_xy_error: Optional[float],
    final_path_yaw_error: Optional[float],
    path_goal_xy_error: Optional[float],
    path_goal_yaw_error: Optional[float],
    xy_tolerance: float,
    yaw_tolerance: float,
    path_goal_xy_tolerance: float = 0.075,
    path_goal_yaw_tolerance: float = 0.01,
    action_status: Optional[int] = None,
    path_rotation_threshold: float = 1.40,
) -> Evaluation:
    """按终点定向标准评估采样结果。"""
    failures: list[str] = []
    warnings: list[str] = []
    details: list[str] = []
    incomplete = first_terminal_time is None

    status_name = action_status_name(action_status)
    details.append(f"/navigate_to_pose action 状态={status_name}")
    if action_status == GoalStatus.STATUS_ABORTED:
        failures.append("导航 action 在进入或完成终点定向前 ABORTED")
    elif action_status == GoalStatus.STATUS_CANCELED:
        failures.append("导航 action 在诊断期间 CANCELED")
    elif first_terminal_time is None and action_status == GoalStatus.STATUS_SUCCEEDED:
        failures.append("action 已 SUCCEEDED，但诊断未观测到双终点 XY 容差")
    elif first_terminal_time is not None and action_status in (
        GoalStatus.STATUS_ACCEPTED,
        GoalStatus.STATUS_EXECUTING,
        GoalStatus.STATUS_CANCELING,
    ):
        failures.append("已进入终点，但 action 未在终点采样窗口内完成")
    elif action_status is None:
        warnings.append("未关联到公开 /navigate_to_pose action 状态")

    episodes = stationary_rotation_episodes(
        twists.get("/cmd_vel_nav", []),
        goal_xy_tolerance=xy_tolerance,
    )
    details.append(f"终点外 /cmd_vel_nav 原地旋转片段={len(episodes)} 段")
    for index, episode in enumerate(episodes, start=1):
        goal_text = (
            f"{episode.min_goal_xy_error:.3f} m"
            if episode.min_goal_xy_error is not None else "未知"
        )
        heading_text = (
            f"{episode.max_path_heading_error:.3f} rad"
            if episode.max_path_heading_error is not None else "未知"
        )
        details.append(
            f"途中转向#{index}：持续={episode.duration:.2f} s，"
            f"max|wz|={episode.max_angular_z:.3f} rad/s，"
            f"最小目标距离={goal_text}，最大路径夹角={heading_text}，"
            f"紧随 /plan={'是' if episode.follows_plan_update else '否'}"
        )
        if episode.max_path_heading_error is None:
            warnings.append(f"途中转向#{index} 缺少路径夹角，无法判断是否必要")
        elif episode.max_path_heading_error < path_rotation_threshold:
            failures.append(
                f"途中转向#{index} 的路径夹角小于 {path_rotation_threshold:.2f} rad，"
                "普通弯道不应停车原地旋转"
            )

    if first_terminal_time is None:
        warnings.append("采样期限内尚未进入双终点 XY 容差；以下为途中快照")
    else:
        for topic in (
            "/cmd_vel_nav",
            "/cmd_vel_switched",
            "/cmd_vel_smoothed",
            "/cmd_vel",
        ):
            terminal_samples = [
                sample for sample in twists.get(topic, []) if sample.terminal
            ]
            if not terminal_samples:
                warnings.append(f"{topic} 没有终点阶段样本")
                continue
            max_linear = max(abs(sample.linear_x) for sample in terminal_samples)
            flips = direction_flips(sample.angular_z for sample in terminal_samples)
            max_wz = max(abs(sample.angular_z) for sample in terminal_samples)
            wrong_direction = sum(
                1 for sample in terminal_samples
                if sample.yaw_error is not None
                and abs(sample.yaw_error) > yaw_tolerance
                and abs(sample.angular_z) >= 0.03
                and sample.angular_z * sample.yaw_error < 0.0
            )
            details.append(
                f"{topic}：终点 max|vx|={max_linear:.3f} m/s，"
                f"max|wz|={max_wz:.3f} rad/s，换向={flips} 次，"
                f"背离目标 yaw={wrong_direction} 个样本"
            )
            if max_linear > 0.01:
                failures.append(f"{topic} 终点线速度超过 0.01 m/s")
            if flips > 0:
                failures.append(f"{topic} 终点角速度发生 {flips} 次换向")
            if wrong_direction > 0:
                failures.append(
                    f"{topic} 有 {wrong_direction} 个终点角速度样本在增大 yaw 误差"
                )

    if first_terminal_time is not None:
        pre_terminal_plans = [
            stamp for stamp in plan_times if stamp <= first_terminal_time
        ]
        if len(pre_terminal_plans) >= 2:
            interval = pre_terminal_plans[-1] - pre_terminal_plans[0]
            pre_terminal_hz = (
                (len(pre_terminal_plans) - 1) / interval
                if interval > 0.0 else 0.0
            )
            details.append(
                f"终点前 /plan={len(pre_terminal_plans)} 次，"
                f"实测频率={pre_terminal_hz:.2f} Hz"
            )
            if len(pre_terminal_plans) >= 3 and pre_terminal_hz > 1.50:
                failures.append("终点前重规划频率明显超过配置的 1 Hz")
        else:
            details.append(f"终点前 /plan={len(pre_terminal_plans)} 次")
        replans = sum(
            stamp > first_terminal_time + 0.30 for stamp in plan_times
        )
        details.append(f"进入双终点容差 0.30 s 后的 /plan 更新={replans} 次")
        if replans:
            failures.append("进入终点锁存后仍发生重规划")

    snapshot_prefix = "停稳后" if first_terminal_time is not None else "当前"
    if final_goal_xy_error is None or final_goal_yaw_error is None:
        message = (
            "缺少 /navigation/accepted_goal 或 map→base_footprint TF，"
            "无法计算原始目标误差"
        )
        (failures if first_terminal_time is not None else warnings).append(message)
    else:
        details.append(
            f"{snapshot_prefix}机器人→原始目标：XY={final_goal_xy_error:.3f} m，"
            f"yaw={final_goal_yaw_error:.3f} rad"
        )
        if first_terminal_time is not None and final_goal_xy_error > xy_tolerance:
            failures.append("机器人到原始目标的 XY 误差超过目标容差")
        if first_terminal_time is not None and final_goal_yaw_error > yaw_tolerance:
            failures.append("机器人到原始目标的 yaw 误差超过目标容差")

    if final_path_xy_error is None or final_path_yaw_error is None:
        message = "缺少有效 /plan，无法计算路径末端误差"
        (failures if first_terminal_time is not None else warnings).append(message)
    else:
        details.append(
            f"{snapshot_prefix}机器人→路径末端：XY={final_path_xy_error:.3f} m，"
            f"yaw={final_path_yaw_error:.3f} rad"
        )
        if first_terminal_time is not None and final_path_xy_error > xy_tolerance:
            failures.append("机器人到路径末端的 XY 误差超过目标容差")
        if first_terminal_time is not None and final_path_yaw_error > yaw_tolerance:
            failures.append("机器人到路径末端的 yaw 误差超过目标容差")

    if path_goal_xy_error is None or path_goal_yaw_error is None:
        message = "无法比较路径末端与原始目标"
        (failures if first_terminal_time is not None else warnings).append(message)
    else:
        details.append(
            f"路径末端→原始目标：XY={path_goal_xy_error:.3f} m，"
            f"yaw={path_goal_yaw_error:.3f} rad"
        )
        if path_goal_xy_error > path_goal_xy_tolerance:
            failures.append("路径末端与原始目标的位置误差超过匹配容差")
        if path_goal_yaw_error > path_goal_yaw_tolerance:
            failures.append("路径末端与原始目标的 yaw 误差超过匹配容差")

    if transforms:
        map_odom_yaws = [sample.map_odom_yaw for sample in transforms]
        max_yaw_jump = maximum_step(map_odom_yaws, angular=True)
        max_xy_jump = max(
            (
                math.hypot(
                    current.map_odom_x - previous.map_odom_x,
                    current.map_odom_y - previous.map_odom_y,
                )
                for previous, current in zip(transforms, transforms[1:])
            ),
            default=0.0,
        )
        details.append(
            f"map→odom 最大单步修正：XY={max_xy_jump:.3f} m，"
            f"yaw={max_yaw_jump:.3f} rad"
        )
        if max_xy_jump > 0.10 or max_yaw_jump > 0.10:
            failures.append(
                "map→odom 单步修正超过 0.10 m/rad；应归类为地图/AMCL 问题"
            )
    else:
        warnings.append("未采到 map→odom TF，无法排除定位跳变")

    if not pause_values:
        warnings.append("未收到 /pause_navigation，无法确认安全锁状态")
    elif any(pause_values):
        failures.append("采样期间 /pause_navigation 曾为 true")
    else:
        details.append("/pause_navigation 全程为 false")
    return Evaluation(failures, warnings, details, incomplete=incomplete)


class RotationDiagnostics(Node):
    """集中采样速度、真值、里程计、路径和 TF。"""

    def __init__(self, mode: str, xy_tolerance: float) -> None:
        super().__init__("go2_rotation_diagnostics")
        self.mode = mode
        self.xy_tolerance = xy_tolerance
        self.twists: DefaultDict[str, list[TwistSample]] = defaultdict(list)
        self.odom: list[PoseSample] = []
        self.ground_truth: list[PoseSample] = []
        self.amcl: list[PoseSample] = []
        self.transforms: list[TransformSample] = []
        self.pause_values: list[bool] = []
        self.plan_times: list[float] = []
        self.last_plan_time: Optional[float] = None
        self.path: Optional[Path] = None
        self.accepted_goal: Optional[PoseStamped] = None
        self.goal_received_time: Optional[float] = None
        self.goal_generation = 0
        self.action_status: Optional[int] = None
        self.action_goal_id: Optional[bytes] = None
        self.action_goal_stamp_ns = -1
        self.action_finished_time: Optional[float] = None
        self.terminal_active = False
        self.terminal_yaw_error: Optional[float] = None
        self.path_heading_error: Optional[float] = None
        self.first_terminal_time: Optional[float] = None
        self.final_goal_xy_error: Optional[float] = None
        self.final_goal_yaw_error: Optional[float] = None
        self.final_path_xy_error: Optional[float] = None
        self.final_path_yaw_error: Optional[float] = None
        self.path_goal_xy_error: Optional[float] = None
        self.path_goal_yaw_error: Optional[float] = None

        for topic in TWIST_TOPICS:
            self.create_subscription(
                Twist,
                topic,
                lambda message, topic_name=topic: self._twist_callback(
                    topic_name, message
                ),
                50,
            )
        self.create_subscription(Odometry, "/odom", self._odom_callback, 50)
        self.create_subscription(
            Odometry,
            "/odom/ground_truth",
            self._ground_truth_callback,
            50,
        )
        amcl_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_callback,
            amcl_qos,
        )
        self.create_subscription(Path, "/plan", self._plan_callback, 20)
        accepted_goal_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseStamped,
            "/navigation/accepted_goal",
            self._accepted_goal_callback,
            accepted_goal_qos,
        )
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._action_status_callback,
            qos_profile_action_status_default,
        )
        self.create_subscription(
            Bool,
            "/pause_navigation",
            lambda message: self.pause_values.append(message.data),
            20,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.05, self._sample_transforms)

    def _twist_callback(self, topic: str, message: Twist) -> None:
        stamp = time.monotonic()
        self.twists[topic].append(
            TwistSample(
                stamp,
                message.linear.x,
                message.linear.y,
                message.angular.z,
                self.terminal_active,
                self.terminal_yaw_error,
                self.final_goal_xy_error,
                self.path_heading_error,
                (
                    stamp - self.last_plan_time
                    if self.last_plan_time is not None else None
                ),
            )
        )

    @staticmethod
    def _pose_sample(message: Odometry) -> PoseSample:
        pose = message.pose.pose
        return PoseSample(
            time.monotonic(),
            pose.position.x,
            pose.position.y,
            quaternion_yaw(pose.orientation),
            message.twist.twist.angular.z,
        )

    def _odom_callback(self, message: Odometry) -> None:
        self.odom.append(self._pose_sample(message))

    def _ground_truth_callback(self, message: Odometry) -> None:
        self.ground_truth.append(self._pose_sample(message))

    def _amcl_callback(self, message: PoseWithCovarianceStamped) -> None:
        pose = message.pose.pose
        self.amcl.append(
            PoseSample(
                time.monotonic(),
                pose.position.x,
                pose.position.y,
                quaternion_yaw(pose.orientation),
            )
        )

    def _plan_callback(self, message: Path) -> None:
        stamp = time.monotonic()
        self.path = message
        self.plan_times.append(stamp)
        self.last_plan_time = stamp

    def _accepted_goal_callback(self, message: PoseStamped) -> None:
        if self.mode != "navigation":
            return
        # 每个新目标使用独立采样窗口。启动较晚时 transient-local 会先补发
        # 当前目标；随后若用户再发送目标，这里会清除旧目标的速度和路径样本。
        self.accepted_goal = message
        self.goal_received_time = time.monotonic()
        self.goal_generation += 1
        self.path = None
        self.twists.clear()
        self.transforms.clear()
        self.pause_values.clear()
        self.plan_times.clear()
        self.last_plan_time = None
        self.action_status = None
        self.action_goal_id = None
        self.action_goal_stamp_ns = -1
        self.action_finished_time = None
        self.terminal_active = False
        self.terminal_yaw_error = None
        self.path_heading_error = None
        self.first_terminal_time = None
        self.final_goal_xy_error = None
        self.final_goal_yaw_error = None
        self.final_path_xy_error = None
        self.final_path_yaw_error = None
        self.path_goal_xy_error = None
        self.path_goal_yaw_error = None

    @staticmethod
    def _status_stamp_ns(status) -> int:
        stamp = status.goal_info.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _action_status_callback(self, message: GoalStatusArray) -> None:
        if self.mode != "navigation" or self.accepted_goal is None:
            return

        active_states = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        active = [status for status in message.status_list
                  if status.status in active_states]
        if active:
            newest = max(active, key=self._status_stamp_ns)
            newest_stamp = self._status_stamp_ns(newest)
            if newest_stamp >= self.action_goal_stamp_ns:
                self.action_goal_stamp_ns = newest_stamp
                self.action_goal_id = bytes(newest.goal_info.goal_id.uuid)
                self.action_status = newest.status
            return

        if self.action_goal_id is None:
            return
        for status in message.status_list:
            if bytes(status.goal_info.goal_id.uuid) != self.action_goal_id:
                continue
            self.action_status = status.status
            if status.status in (
                GoalStatus.STATUS_SUCCEEDED,
                GoalStatus.STATUS_CANCELED,
                GoalStatus.STATUS_ABORTED,
            ) and self.action_finished_time is None:
                self.action_finished_time = time.monotonic()
            return

    @staticmethod
    def _path_heading_from_pose(
        path: Path,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        sampling_distance: float = 0.50,
    ) -> Optional[float]:
        """近似 Rotation Shim 的前向采样路径夹角。"""
        if not path.poses:
            return None
        closest_index = min(
            range(len(path.poses)),
            key=lambda index: math.hypot(
                path.poses[index].pose.position.x - robot_x,
                path.poses[index].pose.position.y - robot_y,
            ),
        )
        target = path.poses[closest_index].pose.position
        accumulated = math.hypot(target.x - robot_x, target.y - robot_y)
        for index in range(closest_index + 1, len(path.poses)):
            previous = path.poses[index - 1].pose.position
            current = path.poses[index].pose.position
            accumulated += math.hypot(
                current.x - previous.x,
                current.y - previous.y,
            )
            target = current
            if accumulated >= sampling_distance:
                break
        delta_x = target.x - robot_x
        delta_y = target.y - robot_y
        if math.hypot(delta_x, delta_y) < 1.0e-3:
            return None
        return normalize_angle(math.atan2(delta_y, delta_x) - robot_yaw)

    def _sample_transforms(self) -> None:
        try:
            map_odom = self.tf_buffer.lookup_transform(
                "map", "odom", rclpy.time.Time()
            )
            odom_base = self.tf_buffer.lookup_transform(
                "odom", "base_footprint", rclpy.time.Time()
            )
        except TransformException:
            return

        stamp = time.monotonic()
        self.transforms.append(
            TransformSample(
                stamp,
                map_odom.transform.translation.x,
                map_odom.transform.translation.y,
                quaternion_yaw(map_odom.transform.rotation),
                odom_base.transform.translation.x,
                odom_base.transform.translation.y,
                quaternion_yaw(odom_base.transform.rotation),
            )
        )

        if self.mode != "navigation" or self.path is None or not self.path.poses:
            return
        path_goal = self.path.poses[-1].pose
        try:
            map_base = self.tf_buffer.lookup_transform(
                self.path.header.frame_id or "map",
                "base_footprint",
                rclpy.time.Time(),
            )
        except TransformException:
            return
        robot_x = map_base.transform.translation.x
        robot_y = map_base.transform.translation.y
        robot_yaw = quaternion_yaw(map_base.transform.rotation)
        self.path_heading_error = self._path_heading_from_pose(
            self.path, robot_x, robot_y, robot_yaw
        )
        self.final_path_xy_error = math.hypot(
            path_goal.position.x - robot_x,
            path_goal.position.y - robot_y,
        )
        self.terminal_yaw_error = normalize_angle(
            quaternion_yaw(path_goal.orientation)
            - robot_yaw
        )
        self.final_path_yaw_error = abs(self.terminal_yaw_error)

        if (
            self.accepted_goal is not None
            and self.accepted_goal.header.frame_id
            == (self.path.header.frame_id or "map")
        ):
            raw_goal = self.accepted_goal.pose
            self.final_goal_xy_error = math.hypot(
                raw_goal.position.x - robot_x,
                raw_goal.position.y - robot_y,
            )
            self.final_goal_yaw_error = abs(normalize_angle(
                quaternion_yaw(raw_goal.orientation)
                - robot_yaw
            ))
            self.path_goal_xy_error = math.hypot(
                raw_goal.position.x - path_goal.position.x,
                raw_goal.position.y - path_goal.position.y,
            )
            self.path_goal_yaw_error = abs(normalize_angle(
                quaternion_yaw(raw_goal.orientation)
                - quaternion_yaw(path_goal.orientation)
            ))

        self.terminal_active = (
            self.final_path_xy_error <= self.xy_tolerance
            and self.final_goal_xy_error is not None
            and self.final_goal_xy_error <= self.xy_tolerance
        )
        if self.terminal_active and self.first_terminal_time is None:
            self.first_terminal_time = stamp


def _print_evaluation(evaluation: Evaluation) -> None:
    for detail in evaluation.details:
        print(f"  数据：{detail}")
    for warning in evaluation.warnings:
        print(f"  警告：{warning}")
    for failure in evaluation.failures:
        print(f"  失败：{failure}")
    if evaluation.failures:
        print("旋转诊断 FAIL")
    elif evaluation.incomplete:
        print("旋转诊断 INCOMPLETE：目标仍在执行或尚未进入终点阶段")
    else:
        print("旋转诊断 PASS")


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode", choices=("manual", "navigation"), required=True
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="manual 模式总采样时长；navigation 模式进入终点后的最长采样时长",
    )
    parser.add_argument(
        "--acquire-timeout",
        type=float,
        default=120.0,
        help="navigation 模式等待新目标进入双终点 XY 容差的最长时间",
    )
    parser.add_argument("--expected-wz", type=float, default=0.45)
    parser.add_argument("--xy-tolerance", type=float, default=0.30)
    parser.add_argument("--yaw-tolerance", type=float, default=0.15)
    parsed, ros_args = parser.parse_known_args(args=args)
    if parsed.duration <= 0.0:
        parser.error("--duration 必须大于零")
    if parsed.acquire_timeout <= 0.0:
        parser.error("--acquire-timeout 必须大于零")
    if parsed.mode == "manual" and abs(parsed.expected_wz) < 0.03:
        parser.error("manual 模式的 --expected-wz 绝对值必须至少为 0.03")
    if parsed.xy_tolerance <= 0.0 or parsed.yaw_tolerance <= 0.0:
        parser.error("目标容差必须大于零")

    rclpy.init(args=ros_args)
    node = RotationDiagnostics(parsed.mode, parsed.xy_tolerance)
    if parsed.mode == "manual":
        print(
            "开始只读旋转诊断："
            f"mode=manual，duration={parsed.duration:.1f}s"
        )
    else:
        print(
            "开始只读旋转诊断：mode=navigation，"
            f"acquire_timeout={parsed.acquire_timeout:.1f}s，"
            f"terminal_duration={parsed.duration:.1f}s"
        )
        print("  等待已接受的新目标及双终点 XY 容差；新目标会重新开始等待计时")
    started_at = time.monotonic()
    deadline = (
        started_at + parsed.duration
        if parsed.mode == "manual"
        else started_at + parsed.acquire_timeout
    )
    observed_goal_generation = 0
    terminal_deadline: Optional[float] = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if parsed.mode == "manual":
                if now >= deadline:
                    break
                continue

            if node.goal_generation != observed_goal_generation:
                observed_goal_generation = node.goal_generation
                deadline = acquisition_deadline(
                    started_at,
                    node.goal_received_time,
                    parsed.acquire_timeout,
                )
                terminal_deadline = None
            if node.first_terminal_time is not None and terminal_deadline is None:
                terminal_deadline = node.first_terminal_time + parsed.duration
                print(
                    "  已进入双终点 XY 容差，开始终点采样："
                    f"最多 {parsed.duration:.1f}s"
                )
            if navigation_sampling_complete(
                now,
                node.action_status,
                node.action_finished_time,
                terminal_deadline,
            ):
                break
            if terminal_deadline is None and now >= deadline:
                break
        if parsed.mode == "manual":
            evaluation = evaluate_manual(
                dict(node.twists),
                node.odom,
                node.ground_truth,
                node.transforms,
                node.pause_values,
                parsed.expected_wz,
            )
        else:
            evaluation = evaluate_navigation(
                dict(node.twists),
                node.transforms,
                node.pause_values,
                node.plan_times,
                node.first_terminal_time,
                node.final_goal_xy_error,
                node.final_goal_yaw_error,
                node.final_path_xy_error,
                node.final_path_yaw_error,
                node.path_goal_xy_error,
                node.path_goal_yaw_error,
                parsed.xy_tolerance,
                parsed.yaw_tolerance,
                action_status=node.action_status,
            )
        _print_evaluation(evaluation)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if evaluation.failures:
        raise SystemExit(1)
    if evaluation.incomplete:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
