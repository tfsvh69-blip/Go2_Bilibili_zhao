#!/usr/bin/env python3

"""把 Gazebo/CHAMP 状态与控制映射为 Unitree Go2 ROS 2 接口。"""

import copy
import json
import math
import threading
import time
from typing import Dict, Optional, Sequence, Tuple

import rclpy
from champ_msgs.msg import ContactsStamped
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException
from unitree_api.msg import Request, Response
from unitree_go.msg import IMUState, LowState, SportModeState


API_BALANCE_STAND = 1002
API_STOP_MOVE = 1003
API_STAND_UP = 1004
API_STAND_DOWN = 1005
API_RECOVERY_STAND = 1006
API_EULER = 1007
API_MOVE = 1008
API_SIT = 1009
API_RISE_SIT = 1010
API_HELLO = 1016
API_STRETCH = 1017
API_DANCE1 = 1022

ERROR_INVALID_PARAMETER = -32001
ERROR_UNSUPPORTED_API = -32002
ERROR_BUSY = -32003
ERROR_DOWNSTREAM = -32004

MODE_IDLE = 0
MODE_BALANCE_STAND = 1
MODE_POSE = 2
MODE_LOCOMOTION = 3
MODE_LIE_DOWN = 5
MODE_RECOVERY_STAND = 8
MODE_SIT = 10

MOTOR_JOINTS = [
    "rf_hip_joint",
    "rf_upper_leg_joint",
    "rf_lower_leg_joint",
    "lf_hip_joint",
    "lf_upper_leg_joint",
    "lf_lower_leg_joint",
    "rh_hip_joint",
    "rh_upper_leg_joint",
    "rh_lower_leg_joint",
    "lh_hip_joint",
    "lh_upper_leg_joint",
    "lh_lower_leg_joint",
]
FOOT_FRAMES = ["rf_foot_link", "lf_foot_link", "rh_foot_link", "lh_foot_link"]
CHAMP_TO_UNITREE_FEET = [1, 0, 3, 2]

BEHAVIOR_APIS = {
    API_STAND_UP: ("stand", MODE_IDLE),
    API_STAND_DOWN: ("lie", MODE_LIE_DOWN),
    API_RECOVERY_STAND: ("stand", MODE_RECOVERY_STAND),
    API_SIT: ("lie", MODE_SIT),
    API_RISE_SIT: ("stand", MODE_RECOVERY_STAND),
    API_HELLO: ("hello", MODE_IDLE),
    API_STRETCH: ("stretch", MODE_IDLE),
    API_DANCE1: ("dance", MODE_IDLE),
}


def parse_xyz(parameter: str) -> Tuple[float, float, float]:
    """解析 Unitree Sport API 使用的 x/y/z JSON 参数。"""
    try:
        value = json.loads(parameter)
        result = tuple(value[key] for key in ("x", "y", "z"))
    except (json.JSONDecodeError, KeyError, TypeError) as error:
        raise ValueError("parameter 必须是包含 x、y、z 的 JSON 对象") from error
    if any(
        isinstance(item, bool) or not isinstance(item, (int, float))
        for item in result
    ):
        raise ValueError("x、y、z 必须是数值")
    converted = tuple(float(item) for item in result)
    if not all(math.isfinite(item) for item in converted):
        raise ValueError("x、y、z 必须是有限值")
    return converted


def quaternion_to_rpy(
    x: float, y: float, z: float, w: float
) -> Tuple[float, float, float]:
    """把 ROS xyzw 四元数转换为滚转、俯仰和偏航。"""
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch_term = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, pitch_term)))
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return roll, pitch, yaw


def rpy_to_quaternion(
    roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float, float]:
    """把滚转、俯仰和偏航转换为 ROS xyzw 四元数。"""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class UnitreeSimBridge(Node):
    """发布 Unitree 状态并把受支持的 Sport API 转给 CHAMP。"""

    def __init__(self) -> None:
        super().__init__("go2_unitree_sim_bridge")
        self._lock = threading.RLock()
        self._callback_group = ReentrantCallbackGroup()
        # 发布频率与超时保护必须独立于 Gazebo 的暂停和仿真时间跳变；
        # 消息中的 stamp 仍由节点的 ROS 时钟生成。
        self._timer_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._declare_parameters()

        self._last_odom: Optional[Odometry] = None
        self._last_imu: Optional[Imu] = None
        self._last_joint: Optional[JointState] = None
        self._last_contacts: Optional[ContactsStamped] = None
        self._received_at: Dict[str, float] = {}
        self._joint_accelerations: Dict[str, float] = {}
        self._previous_joint_velocities: Dict[str, float] = {}
        self._previous_joint_time: Optional[float] = None
        self._previous_foot_positions: Optional[Sequence[float]] = None
        self._previous_foot_time: Optional[float] = None
        self._cached_sport: Optional[SportModeState] = None
        self._cached_low: Optional[LowState] = None

        self._mode = MODE_IDLE
        self._gait_type = 0
        self._progress = 0.0
        self._state_error = 0
        self._move_active = False
        self._move_command = Twist()
        self._behavior_busy = False
        self._behavior_target_mode = MODE_IDLE

        self._tf_buffer = Buffer()
        self._create_ros_interfaces()
        self._create_timers()
        self.get_logger().info("Unitree Go2 仿真兼容桥已启动")

    def _declare_parameters(self) -> None:
        defaults = {
            "odom_topic": "/odom/ground_truth",
            "imu_topic": "/imu/data",
            "joint_states_topic": "/joint_states",
            "foot_contacts_topic": "/foot_contacts",
            # 速度输出话题：非导航默认 /cmd_vel；导航模式下应设为 /cmd_vel_unitree
            # 经 twist_mux 接入安全控制链，避免与 collision_monitor 冲突。
            "cmd_vel_topic": "/cmd_vel",
            "base_frame": "base_link",
            "source_timeout": 1.0,
            "sport_rate": 50.0,
            "sport_lf_rate": 10.0,
            "low_rate": 100.0,
            "low_lf_rate": 10.0,
            "foot_raise_height": 0.04,
            "max_velocity_x": 0.3,
            "max_velocity_y": 0.25,
            "max_yaw_speed": 0.5,
            "max_roll": 0.35,
            "max_pitch": 0.35,
            "max_body_yaw": 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _create_ros_interfaces(self) -> None:
        self._sport_publisher = self.create_publisher(
            SportModeState, "/sportmodestate", 10
        )
        self._sport_lf_publisher = self.create_publisher(
            SportModeState, "/lf/sportmodestate", 10
        )
        self._low_publisher = self.create_publisher(LowState, "/lowstate", 10)
        self._low_lf_publisher = self.create_publisher(
            LowState, "/lf/lowstate", 10
        )
        self._response_publisher = self.create_publisher(
            Response, "/api/sport/response", 10
        )
        self._cmd_vel_publisher = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        self._body_pose_publisher = self.create_publisher(
            Pose, "/body_pose", 10
        )

        # 使用独立名称保存引用，不能覆盖 rclpy.Node 的内部 _subscriptions。
        self._owned_subscriptions = [
            self.create_subscription(
                Odometry,
                self.get_parameter("odom_topic").value,
                self._odom_callback,
                10,
                callback_group=self._callback_group,
            ),
            self.create_subscription(
                Imu,
                self.get_parameter("imu_topic").value,
                self._imu_callback,
                20,
                callback_group=self._callback_group,
            ),
            self.create_subscription(
                JointState,
                self.get_parameter("joint_states_topic").value,
                self._joint_callback,
                20,
                callback_group=self._callback_group,
            ),
            self.create_subscription(
                ContactsStamped,
                self.get_parameter("foot_contacts_topic").value,
                self._contacts_callback,
                10,
                callback_group=self._callback_group,
            ),
            self.create_subscription(
                Request,
                "/api/sport/request",
                self._request_callback,
                10,
                callback_group=self._callback_group,
            ),
            self.create_subscription(
                String,
                "/go2_behaviors/status",
                self._behavior_status_callback,
                10,
                callback_group=self._callback_group,
            ),
        ]
        self._owned_subscriptions.extend(
            [
                self.create_subscription(
                    TFMessage,
                    "/tf",
                    self._tf_callback,
                    # 只需要最新的足端变换；深队列会在高频 TF 下延迟控制请求。
                    QoSProfile(depth=1),
                    callback_group=self._callback_group,
                ),
                self.create_subscription(
                    TFMessage,
                    "/tf_static",
                    self._tf_static_callback,
                    QoSProfile(
                        depth=1,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    ),
                    callback_group=self._callback_group,
                ),
            ]
        )

        self._behavior_clients = {
            name: self.create_client(
                Trigger,
                f"/go2_behaviors/{name}",
                callback_group=self._callback_group,
            )
            for name in {value[0] for value in BEHAVIOR_APIS.values()}
        }
        self._behavior_stop_client = self.create_client(
            Trigger,
            "/go2_behaviors/stop",
            callback_group=self._callback_group,
        )

    def _create_timers(self) -> None:
        def period(name: str) -> float:
            rate = float(self.get_parameter(name).value)
            if rate <= 0.0:
                raise ValueError(f"{name} 必须大于零")
            return 1.0 / rate

        self._timers = [
            self.create_timer(
                period("sport_rate"),
                self._publish_sport,
                clock=self._timer_clock,
                callback_group=self._callback_group,
            ),
            self.create_timer(
                period("sport_lf_rate"),
                self._publish_sport_lf,
                clock=self._timer_clock,
                callback_group=self._callback_group,
            ),
            self.create_timer(
                period("low_rate"),
                self._publish_low,
                clock=self._timer_clock,
                callback_group=self._callback_group,
            ),
            self.create_timer(
                period("low_lf_rate"),
                self._publish_low_lf,
                clock=self._timer_clock,
                callback_group=self._callback_group,
            ),
            self.create_timer(
                0.02,
                self._publish_move_command,
                clock=self._timer_clock,
                callback_group=self._callback_group,
            ),
            self.create_timer(
                0.1,
                self._watch_sources,
                clock=self._timer_clock,
                callback_group=self._callback_group,
            ),
        ]

    def _mark_received(self, source: str) -> None:
        self._received_at[source] = time.monotonic()

    def _odom_callback(self, message: Odometry) -> None:
        with self._lock:
            self._last_odom = message
            self._mark_received("odom")

    def _imu_callback(self, message: Imu) -> None:
        with self._lock:
            self._last_imu = message
            self._mark_received("imu")

    def _joint_callback(self, message: JointState) -> None:
        now = time.monotonic()
        velocities = dict(zip(message.name, message.velocity))
        with self._lock:
            if self._previous_joint_time is not None:
                delta = now - self._previous_joint_time
                if delta > 1e-6:
                    for name, velocity in velocities.items():
                        previous = self._previous_joint_velocities.get(
                            name, velocity
                        )
                        self._joint_accelerations[name] = (
                            velocity - previous
                        ) / delta
            self._previous_joint_velocities = velocities
            self._previous_joint_time = now
            self._last_joint = message
            self._mark_received("joint")

    def _contacts_callback(self, message: ContactsStamped) -> None:
        with self._lock:
            self._last_contacts = message
            self._mark_received("contacts")

    def _tf_callback(self, message: TFMessage) -> None:
        for transform in message.transforms:
            self._tf_buffer.set_transform(transform, "go2_unitree_sim_bridge")

    def _tf_static_callback(self, message: TFMessage) -> None:
        for transform in message.transforms:
            self._tf_buffer.set_transform_static(
                transform, "go2_unitree_sim_bridge"
            )

    def _behavior_status_callback(self, message: String) -> None:
        with self._lock:
            if message.data.startswith("running:dance"):
                self._progress = 1.0
            elif message.data == "idle" or message.data.startswith("failed:"):
                self._progress = 0.0
                if not self._move_active:
                    self._mode = MODE_IDLE
                    self._gait_type = 0

    def _sources_ready(self) -> bool:
        return all(
            source in self._received_at
            for source in ("odom", "imu", "joint")
        )

    def _watch_sources(self) -> None:
        with self._lock:
            if not self._sources_ready():
                return
            timeout = float(self.get_parameter("source_timeout").value)
            now = time.monotonic()
            stale = [
                source
                for source in ("odom", "imu", "joint")
                if now - self._received_at[source] > timeout
            ]
            if stale:
                self._state_error = 1
                if self._move_active:
                    self._move_active = False
                    self._publish_zero_velocity()
                self.get_logger().warning(
                    "Unitree bridge 输入超时：" + ", ".join(stale),
                    throttle_duration_sec=5.0,
                )
            else:
                self._state_error = 0

    def _fill_imu(self, target: IMUState) -> None:
        imu = self._last_imu
        if imu is None:
            return
        q = imu.orientation
        target.quaternion = [q.w, q.x, q.y, q.z]
        target.gyroscope = [
            imu.angular_velocity.x,
            imu.angular_velocity.y,
            imu.angular_velocity.z,
        ]
        target.accelerometer = [
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
        ]
        target.rpy = list(quaternion_to_rpy(q.x, q.y, q.z, q.w))

    def _read_feet(self) -> Tuple[Sequence[float], Sequence[float]]:
        positions = []
        base_frame = self.get_parameter("base_frame").value
        for frame in FOOT_FRAMES:
            try:
                transform = self._tf_buffer.lookup_transform(
                    base_frame, frame, Time()
                )
                translation = transform.transform.translation
                positions.extend([translation.x, translation.y, translation.z])
            except TransformException:
                positions.extend([0.0, 0.0, 0.0])

        now = time.monotonic()
        speeds = [0.0] * 12
        if (
            self._previous_foot_positions is not None
            and self._previous_foot_time is not None
        ):
            delta = now - self._previous_foot_time
            if delta > 1e-6:
                speeds = [
                    (current - previous) / delta
                    for current, previous in zip(
                        positions, self._previous_foot_positions
                    )
                ]
        self._previous_foot_positions = positions
        self._previous_foot_time = now
        return positions, speeds

    def _unitree_foot_force(self) -> Sequence[int]:
        if (
            self._last_contacts is None
            or len(self._last_contacts.contacts) < 4
        ):
            return [0, 0, 0, 0]
        contacts = self._last_contacts.contacts
        return [int(bool(contacts[index])) for index in CHAMP_TO_UNITREE_FEET]

    def _build_sport_state(self) -> SportModeState:
        message = SportModeState()
        if self._last_odom is not None:
            message.stamp.sec = self._last_odom.header.stamp.sec
            message.stamp.nanosec = self._last_odom.header.stamp.nanosec
        message.error_code = self._state_error
        self._fill_imu(message.imu_state)
        message.mode = self._mode
        message.progress = self._progress
        message.gait_type = self._gait_type
        message.foot_raise_height = float(
            self.get_parameter("foot_raise_height").value
        )
        odom = self._last_odom
        if odom is not None:
            position = odom.pose.pose.position
            velocity = odom.twist.twist.linear
            message.position = [position.x, position.y, position.z]
            message.body_height = position.z
            message.velocity = [velocity.x, velocity.y, velocity.z]
            message.yaw_speed = odom.twist.twist.angular.z
        message.foot_force = self._unitree_foot_force()
        positions, speeds = self._read_feet()
        message.foot_position_body = positions
        message.foot_speed_body = speeds
        return message

    def _build_low_state(self) -> LowState:
        message = LowState()
        message.head = [0xFE, 0xEF]
        message.level_flag = 0xFF
        self._fill_imu(message.imu_state)
        positions: Dict[str, float] = {}
        velocities: Dict[str, float] = {}
        efforts: Dict[str, float] = {}
        if self._last_joint is not None:
            positions = dict(
                zip(self._last_joint.name, self._last_joint.position)
            )
            velocities = dict(
                zip(self._last_joint.name, self._last_joint.velocity)
            )
            efforts = dict(zip(self._last_joint.name, self._last_joint.effort))
        for index, name in enumerate(MOTOR_JOINTS):
            motor = message.motor_state[index]
            motor.mode = 1
            motor.q = positions.get(name, 0.0)
            motor.dq = velocities.get(name, 0.0)
            motor.ddq = self._joint_accelerations.get(name, 0.0)
            effort = efforts.get(name, 0.0)
            motor.tau_est = effort if math.isfinite(effort) else 0.0
            motor.q_raw = motor.q
            motor.dq_raw = motor.dq
            motor.ddq_raw = motor.ddq
        message.foot_force = self._unitree_foot_force()
        message.foot_force_est = message.foot_force
        message.tick = (
            int(self.get_clock().now().nanoseconds // 1_000_000) & 0xFFFFFFFF
        )
        return message

    def _publish_sport(self) -> None:
        with self._lock:
            if not self._sources_ready():
                return
            self._cached_sport = self._build_sport_state()
            self._sport_publisher.publish(self._cached_sport)

    def _publish_sport_lf(self) -> None:
        with self._lock:
            if self._cached_sport is not None:
                self._sport_lf_publisher.publish(
                    copy.deepcopy(self._cached_sport)
                )

    def _publish_low(self) -> None:
        with self._lock:
            if not self._sources_ready():
                return
            self._cached_low = self._build_low_state()
            self._low_publisher.publish(self._cached_low)

    def _publish_low_lf(self) -> None:
        with self._lock:
            if self._cached_low is not None:
                self._low_lf_publisher.publish(copy.deepcopy(self._cached_low))

    def _publish_move_command(self) -> None:
        with self._lock:
            if self._move_active:
                self._cmd_vel_publisher.publish(self._move_command)

    def _publish_zero_velocity(self) -> None:
        self._move_command = Twist()
        if not rclpy.ok():
            return
        try:
            self._cmd_vel_publisher.publish(self._move_command)
        except Exception:  # 不同 RMW 在关闭竞态中抛出的异常类型不同
            # launch 收到 SIGINT 后上下文可能在检查与发布之间失效。
            pass

    def _publish_response(
        self, request: Request, code: int, message: str
    ) -> None:
        if request.header.policy.noreply:
            return
        response = Response()
        response.header.identity = request.header.identity
        response.header.status.code = code
        response.data = json.dumps(
            {"simulated": True, "message": message}, ensure_ascii=False
        )
        self._response_publisher.publish(response)

    def _request_callback(self, request: Request) -> None:
        api_id = request.header.identity.api_id
        with self._lock:
            if api_id == API_MOVE:
                self._handle_move(request)
            elif api_id == API_EULER:
                self._handle_euler(request)
            elif api_id in (API_STOP_MOVE, API_BALANCE_STAND):
                self._handle_stop(request, api_id == API_BALANCE_STAND)
            elif api_id in BEHAVIOR_APIS:
                self._handle_behavior(request, *BEHAVIOR_APIS[api_id])
            else:
                self._publish_response(
                    request, ERROR_UNSUPPORTED_API, f"不支持 Sport API {api_id}"
                )

    def _handle_move(self, request: Request) -> None:
        if self._behavior_busy:
            self._publish_response(request, ERROR_BUSY, "动作执行期间不能 Move")
            return
        try:
            x, y, yaw = parse_xyz(request.parameter)
        except ValueError as error:
            self._publish_response(
                request, ERROR_INVALID_PARAMETER, str(error)
            )
            return
        command = Twist()
        command.linear.x = clamp(
            x, float(self.get_parameter("max_velocity_x").value)
        )
        command.linear.y = clamp(
            y, float(self.get_parameter("max_velocity_y").value)
        )
        command.angular.z = clamp(
            yaw, float(self.get_parameter("max_yaw_speed").value)
        )
        self._move_command = command
        self._move_active = True
        self._mode = MODE_LOCOMOTION
        self._gait_type = 1
        self._publish_response(request, 0, "Move 已生效")

    def _handle_euler(self, request: Request) -> None:
        if self._behavior_busy:
            self._publish_response(request, ERROR_BUSY, "动作执行期间不能 Euler")
            return
        try:
            roll, pitch, yaw = parse_xyz(request.parameter)
        except ValueError as error:
            self._publish_response(
                request, ERROR_INVALID_PARAMETER, str(error)
            )
            return
        limits = (
            float(self.get_parameter("max_roll").value),
            float(self.get_parameter("max_pitch").value),
            float(self.get_parameter("max_body_yaw").value),
        )
        if any(
            abs(value) > limit
            for value, limit in zip((roll, pitch, yaw), limits)
        ):
            self._publish_response(
                request, ERROR_INVALID_PARAMETER, "Euler 超出仿真安全范围"
            )
            return
        pose = Pose()
        qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        self._body_pose_publisher.publish(pose)
        self._mode = MODE_POSE
        self._gait_type = 0
        self._publish_response(request, 0, "Euler 已生效")

    def _handle_stop(self, request: Request, balance_stand: bool) -> None:
        self._move_active = False
        self._publish_zero_velocity()
        self._mode = MODE_BALANCE_STAND if balance_stand else MODE_IDLE
        self._gait_type = 0
        if (
            self._behavior_busy
            and self._behavior_stop_client.service_is_ready()
        ):
            self._behavior_stop_client.call_async(Trigger.Request())
        self._publish_response(request, 0, "已停止运动")

    def _handle_behavior(
        self, request: Request, behavior: str, mode: int
    ) -> None:
        if self._behavior_busy:
            self._publish_response(request, ERROR_BUSY, "已有动作正在执行")
            return
        client = self._behavior_clients[behavior]
        if not client.service_is_ready():
            self._publish_response(
                request, ERROR_DOWNSTREAM, f"{behavior} 服务未就绪"
            )
            return
        self._move_active = False
        self._publish_zero_velocity()
        self._behavior_busy = True
        self._behavior_target_mode = mode
        self._mode = mode
        self._progress = 1.0 if behavior == "dance" else 0.0
        future = client.call_async(Trigger.Request())

        def done_callback(completed) -> None:
            self._behavior_done(request, behavior, completed)

        future.add_done_callback(done_callback)

    def _behavior_done(self, request: Request, behavior: str, future) -> None:
        with self._lock:
            self._behavior_busy = False
            self._progress = 0.0
            try:
                result = future.result()
            except Exception as error:  # rclpy future 会把服务异常放在此处
                self._mode = MODE_IDLE
                self._publish_response(request, ERROR_DOWNSTREAM, str(error))
                return
            if result is None or not result.success:
                self._mode = MODE_IDLE
                detail = result.message if result is not None else "服务没有响应"
                self._publish_response(request, ERROR_DOWNSTREAM, detail)
                return
            if behavior == "lie":
                self._mode = self._behavior_target_mode
            else:
                self._mode = MODE_IDLE
            self._gait_type = 0
            self._publish_response(request, 0, result.message)


def main() -> None:
    rclpy.init()
    bridge = UnitreeSimBridge()
    try:
        # 回调均短小且已有内部锁。Humble rclpy/CycloneDDS 的 waitable 在当前
        # lo 配置下可能空转；5 ms 退让保留 200 Hz 上限，高于最高 100 Hz 状态发布。
        while rclpy.ok():
            rclpy.spin_once(bridge, timeout_sec=0.02)
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            with bridge._lock:
                bridge._move_active = False
                bridge._publish_zero_velocity()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
