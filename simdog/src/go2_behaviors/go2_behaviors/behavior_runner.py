#!/usr/bin/env python3

"""通过 ros2_control 标准动作接口执行 Go2 仿真动作。"""

import argparse
import copy
import fcntl
import math
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_NAMES = [
    "lf_hip_joint",
    "lf_upper_leg_joint",
    "lf_lower_leg_joint",
    "rf_hip_joint",
    "rf_upper_leg_joint",
    "rf_lower_leg_joint",
    "lh_hip_joint",
    "lh_upper_leg_joint",
    "lh_lower_leg_joint",
    "rh_hip_joint",
    "rh_upper_leg_joint",
    "rh_lower_leg_joint",
]

STAND = {
    "lf_hip_joint": 0.03,
    "lf_upper_leg_joint": 1.02,
    "lf_lower_leg_joint": -2.12,
    "rf_hip_joint": -0.03,
    "rf_upper_leg_joint": 1.02,
    "rf_lower_leg_joint": -2.12,
    "lh_hip_joint": 0.03,
    "lh_upper_leg_joint": 1.02,
    "lh_lower_leg_joint": -2.12,
    "rh_hip_joint": -0.03,
    "rh_upper_leg_joint": 1.02,
    "rh_lower_leg_joint": -2.12,
}

Pose = Dict[str, float]
Keyframe = Tuple[float, Pose]


def pose(**changes: float) -> Pose:
    """从稳定站姿创建关键帧，参数名用双下划线代替关节名中的下划线。"""
    result = copy.deepcopy(STAND)
    for encoded_name, value in changes.items():
        joint_name = encoded_name.replace("__", "_")
        if joint_name not in result:
            raise ValueError(f"未知关节：{joint_name}")
        result[joint_name] = value
    return result


NOD_DOWN = pose(
    lf__upper__leg__joint=1.20,
    lf__lower__leg__joint=-2.30,
    rf__upper__leg__joint=1.20,
    rf__lower__leg__joint=-2.30,
    lh__upper__leg__joint=0.92,
    lh__lower__leg__joint=-1.96,
    rh__upper__leg__joint=0.92,
    rh__lower__leg__joint=-1.96,
)
NOD_UP = pose(
    lf__upper__leg__joint=0.90,
    lf__lower__leg__joint=-1.94,
    rf__upper__leg__joint=0.90,
    rf__lower__leg__joint=-1.94,
    lh__upper__leg__joint=1.15,
    lh__lower__leg__joint=-2.26,
    rh__upper__leg__joint=1.15,
    rh__lower__leg__joint=-2.26,
)
STRETCH_FORWARD = pose(
    lf__upper__leg__joint=1.34,
    lf__lower__leg__joint=-2.44,
    rf__upper__leg__joint=1.34,
    rf__lower__leg__joint=-2.44,
    lh__upper__leg__joint=0.78,
    lh__lower__leg__joint=-1.72,
    rh__upper__leg__joint=0.78,
    rh__lower__leg__joint=-1.72,
)
STRETCH_BACK = pose(
    lf__upper__leg__joint=0.88,
    lf__lower__leg__joint=-1.88,
    rf__upper__leg__joint=0.88,
    rf__lower__leg__joint=-1.88,
    lh__upper__leg__joint=1.28,
    lh__lower__leg__joint=-2.38,
    rh__upper__leg__joint=1.28,
    rh__lower__leg__joint=-2.38,
)
LIE = pose(
    lf__upper__leg__joint=1.50,
    lf__lower__leg__joint=-2.60,
    rf__upper__leg__joint=1.50,
    rf__lower__leg__joint=-2.60,
    lh__upper__leg__joint=1.50,
    lh__lower__leg__joint=-2.60,
    rh__upper__leg__joint=1.50,
    rh__lower__leg__joint=-2.60,
)
WAVE_CENTER = pose(
    rf__hip__joint=-0.03,
    rf__upper__leg__joint=0.70,
    rf__lower__leg__joint=-2.30,
)
WAVE_LEFT = copy.deepcopy(WAVE_CENTER)
WAVE_LEFT["rf_hip_joint"] = 0.10
WAVE_RIGHT = copy.deepcopy(WAVE_CENTER)
WAVE_RIGHT["rf_hip_joint"] = -0.16


BEHAVIORS: Dict[str, List[Keyframe]] = {
    "hello": [
        (0.8, STAND),
        (0.55, NOD_DOWN),
        (0.55, STAND),
        (0.65, WAVE_CENTER),
        (0.40, WAVE_LEFT),
        (0.40, WAVE_RIGHT),
        (0.40, WAVE_LEFT),
        (0.40, WAVE_RIGHT),
        (0.65, STAND),
    ],
    "nod": [
        (0.8, STAND),
        (0.50, NOD_DOWN),
        (0.50, NOD_UP),
        (0.50, NOD_DOWN),
        (0.55, STAND),
    ],
    "stretch": [
        (0.8, STAND),
        (0.9, STRETCH_FORWARD),
        (0.7, STAND),
        (0.9, STRETCH_BACK),
        (0.7, STAND),
    ],
    "lie": [
        (0.8, STAND),
        (0.9, pose(
            lf__upper__leg__joint=1.30,
            lf__lower__leg__joint=-2.42,
            rf__upper__leg__joint=1.30,
            rf__lower__leg__joint=-2.42,
            lh__upper__leg__joint=1.30,
            lh__lower__leg__joint=-2.42,
            rh__upper__leg__joint=1.30,
            rh__lower__leg__joint=-2.42,
        )),
        (0.9, LIE),
    ],
    "wave": [
        (0.8, STAND),
        (0.65, WAVE_CENTER),
        (0.40, WAVE_LEFT),
        (0.40, WAVE_RIGHT),
        (0.40, WAVE_LEFT),
        (0.40, WAVE_RIGHT),
        (0.65, STAND),
    ],
    "dance": [
        (0.8, STAND),
        (0.48, pose(
            lf__hip__joint=0.24,
            rf__hip__joint=0.12,
            lh__hip__joint=-0.12,
            rh__hip__joint=-0.24,
            lf__upper__leg__joint=1.16,
            rh__upper__leg__joint=1.16,
        )),
        (0.48, pose(
            lf__hip__joint=-0.12,
            rf__hip__joint=-0.24,
            lh__hip__joint=0.24,
            rh__hip__joint=0.12,
            rf__upper__leg__joint=1.16,
            lh__upper__leg__joint=1.16,
        )),
        (0.48, pose(
            lf__hip__joint=0.26,
            rf__hip__joint=-0.26,
            lh__hip__joint=0.26,
            rh__hip__joint=-0.26,
            lf__lower__leg__joint=-2.30,
            rf__lower__leg__joint=-2.30,
        )),
        (0.48, pose(
            lf__hip__joint=-0.18,
            rf__hip__joint=0.18,
            lh__hip__joint=-0.18,
            rh__hip__joint=0.18,
            lh__lower__leg__joint=-2.30,
            rh__lower__leg__joint=-2.30,
        )),
        (0.48, pose(
            lf__hip__joint=0.24,
            rf__hip__joint=0.12,
            lh__hip__joint=-0.12,
            rh__hip__joint=-0.24,
            lf__upper__leg__joint=1.16,
            rh__upper__leg__joint=1.16,
        )),
        (0.48, pose(
            lf__hip__joint=-0.12,
            rf__hip__joint=-0.24,
            lh__hip__joint=0.24,
            rh__hip__joint=0.12,
            rf__upper__leg__joint=1.16,
            lh__upper__leg__joint=1.16,
        )),
        (0.7, STAND),
    ],
    "stand": [
        (1.2, STAND),
    ],
}

CHINESE_NAMES = {
    "hello": "打招呼",
    "nod": "点头",
    "stretch": "伸展",
    "lie": "趴下",
    "wave": "挥爪",
    "dance": "简单舞蹈",
    "stand": "恢复站立",
}

JOINT_LIMITS = {
    "lf_hip_joint": (-1.04, 1.04),
    "rf_hip_joint": (-1.04, 1.04),
    "lh_hip_joint": (-1.04, 1.04),
    "rh_hip_joint": (-1.04, 1.04),
    "lf_upper_leg_joint": (-1.57, 3.49),
    "rf_upper_leg_joint": (-1.57, 3.49),
    "lh_upper_leg_joint": (-0.52, 4.53),
    "rh_upper_leg_joint": (-0.52, 4.53),
    "lf_lower_leg_joint": (-2.72, -0.84),
    "rf_lower_leg_joint": (-2.72, -0.84),
    "lh_lower_leg_joint": (-2.72, -0.84),
    "rh_lower_leg_joint": (-2.72, -0.84),
}


def validate_behaviors() -> None:
    """在发送前检查关键帧完整性、有限值和当前 URDF 关节限制。"""
    expected_joints = set(JOINT_NAMES)
    for behavior_name, keyframes in BEHAVIORS.items():
        for _, target_pose in keyframes:
            if set(target_pose) != expected_joints:
                raise ValueError(f"{behavior_name} 存在不完整的关节关键帧")
            for joint_name, value in target_pose.items():
                lower, upper = JOINT_LIMITS[joint_name]
                if not math.isfinite(value) or not lower <= value <= upper:
                    raise ValueError(
                        f"{behavior_name} 的 {joint_name}={value} 超出 [{lower}, {upper}]"
                    )


validate_behaviors()


class BehaviorRunner(Node):
    """协调 CHAMP 控制权并向关节轨迹控制器发送动作。"""

    def __init__(self) -> None:
        super().__init__("go2_behavior")
        self._joint_positions: Dict[str, float] = {}
        self._joint_subscription = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )
        self._ground_truth: Optional[Odometry] = None
        self._odom_subscription = self.create_subscription(
            Odometry, "/odom/ground_truth", self._odom_callback, 10
        )
        self._mode_client = self.create_client(
            SetBool, "/quadruped_controller_node/set_behavior_mode"
        )
        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_group_effort_controller/follow_joint_trajectory",
        )
        self.mode_acquired = False
        self.keep_mode_on_exit = False

    def _joint_state_callback(self, message: JointState) -> None:
        self._joint_positions.update(zip(message.name, message.position))

    def _odom_callback(self, message: Odometry) -> None:
        self._ground_truth = message

    def _wait_for_joint_state(self, timeout: float = 8.0) -> Pose:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(name in self._joint_positions for name in JOINT_NAMES):
                return {name: self._joint_positions[name] for name in JOINT_NAMES}
        raise RuntimeError("未收到包含 12 个腿部关节的 /joint_states")

    def set_behavior_mode(self, enabled: bool) -> None:
        if not self._mode_client.wait_for_service(timeout_sec=8.0):
            raise RuntimeError(
                "找不到 /quadruped_controller_node/set_behavior_mode；"
                "请确认已重新构建 champ_base 并启动完整仿真"
            )

        request = SetBool.Request()
        request.data = enabled
        future = self._mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("切换 CHAMP/行为控制权超时")
        if not future.result().success:
            raise RuntimeError(future.result().message)
        self.mode_acquired = enabled
        self.get_logger().info(future.result().message)

    def validate_body_pose(self, behavior: str) -> None:
        """用 Gazebo 真值检查机身没有侧翻，并区分站立和趴下高度。"""
        if self._ground_truth is None:
            raise RuntimeError("未收到 /odom/ground_truth，无法检查动作后的机身姿态")

        position = self._ground_truth.pose.pose.position
        quaternion = self._ground_truth.pose.pose.orientation
        roll = math.atan2(
            2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z),
            1.0 - 2.0 * (quaternion.x ** 2 + quaternion.y ** 2),
        )
        pitch_term = 2.0 * (
            quaternion.w * quaternion.y - quaternion.z * quaternion.x
        )
        pitch = math.asin(max(-1.0, min(1.0, pitch_term)))

        expected_height = (
            0.06 <= position.z <= 0.14 if behavior == "lie" else position.z >= 0.16
        )
        if not expected_height or abs(roll) > 0.35 or abs(pitch) > 0.35:
            raise RuntimeError(
                "动作后的动力学姿态异常："
                f"z={position.z:.3f} m, roll={roll:.3f} rad, "
                f"pitch={pitch:.3f} rad"
            )

        self.get_logger().info(
            "动力学检查通过："
            f"z={position.z:.3f} m, roll={roll:.3f} rad, "
            f"pitch={pitch:.3f} rad"
        )

    @staticmethod
    def _duration(seconds: float) -> Duration:
        whole_seconds = int(seconds)
        nanoseconds = int(round((seconds - whole_seconds) * 1_000_000_000))
        return Duration(sec=whole_seconds, nanosec=nanoseconds)

    def execute(self, behavior: str) -> None:
        initial_pose = self._wait_for_joint_state()
        self.set_behavior_mode(True)

        if not self._trajectory_client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError(
                "找不到 joint_trajectory_controller 动作服务；"
                "请检查 joint_group_effort_controller 是否为 active"
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        goal.goal_time_tolerance = self._duration(2.0)
        goal.goal_tolerance = [
            JointTolerance(name=name, position=0.16, velocity=20.0)
            for name in JOINT_NAMES
        ]

        elapsed = 0.25
        initial_point = JointTrajectoryPoint()
        initial_point.positions = [initial_pose[name] for name in JOINT_NAMES]
        initial_point.time_from_start = self._duration(elapsed)
        goal.trajectory.points.append(initial_point)

        for segment_duration, target_pose in BEHAVIORS[behavior]:
            elapsed += segment_duration
            point = JointTrajectoryPoint()
            point.positions = [target_pose[name] for name in JOINT_NAMES]
            point.time_from_start = self._duration(elapsed)
            goal.trajectory.points.append(point)

        self.get_logger().info(
            f"开始执行“{CHINESE_NAMES[behavior]}”，轨迹时长 {elapsed:.2f} 秒"
        )
        send_future = self._trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=8.0)
        goal_handle = send_future.result() if send_future.done() else None
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("关节轨迹控制器拒绝了动作目标")

        result_future = goal_handle.get_result_async()
        # 高负载传感器仿真的实时率可能只有 0.2 左右，等待时间必须按仿真时长放大。
        result_timeout = max(30.0, elapsed * 8.0 + 10.0)
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=result_timeout
        )
        if not result_future.done() or result_future.result() is None:
            raise RuntimeError("等待动作完成超时")

        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"动作执行失败，错误码 {result.error_code}：{result.error_string}"
            )

        self.validate_body_pose(behavior)

        if behavior == "lie":
            self.keep_mode_on_exit = True
            self.get_logger().info(
                "“趴下”完成；CHAMP 保持暂停。需要恢复时执行："
                "ros2 run go2_behaviors go2_behavior stand"
            )
        else:
            self.set_behavior_mode(False)
            self.get_logger().info(f"“{CHINESE_NAMES[behavior]}”执行完成")


def parse_arguments(arguments: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="执行 Go2 Gazebo 仿真动作（不会调用真机 Sport API）"
    )
    parser.add_argument(
        "behavior",
        choices=BEHAVIORS.keys(),
        help="hello/nod/stretch/lie/wave/dance，stand 用于从趴下恢复",
    )
    return parser.parse_args(arguments)


def main(arguments: Sequence[str] = None) -> int:
    parsed = parse_arguments(sys.argv[1:] if arguments is None else arguments)
    lock_file = open("/tmp/go2_behavior.lock", "w", encoding="utf-8")
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("已有 Go2 动作正在执行，请等待其完成后重试", file=sys.stderr)
        lock_file.close()
        return 2

    rclpy.init(args=None)
    runner = BehaviorRunner()
    exit_code = 0
    try:
        runner.execute(parsed.behavior)
    except (KeyboardInterrupt, RuntimeError) as error:
        runner.get_logger().error(str(error) or "动作被中断")
        exit_code = 1
    finally:
        if runner.mode_acquired and not runner.keep_mode_on_exit:
            try:
                runner.set_behavior_mode(False)
            except RuntimeError as error:
                runner.get_logger().error(f"恢复 CHAMP 失败：{error}")
        runner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
        lock_file.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
