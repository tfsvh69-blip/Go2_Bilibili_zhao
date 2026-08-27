"""把 Gazebo 真值位姿转换为从零开始的导航里程计。

该节点只用于仿真导航。Gazebo 中机器人生成在世界坐标 ``(3, 0)``，而 Nav2
期望 ``odom`` 在每次启动时从机器人附近开始。节点以收到的第一帧真值为原点，
发布 ``/odom`` 和唯一的 ``odom -> base_footprint`` 变换。真机不得使用此节点。
"""

import math
from collections import deque
from typing import Deque, Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """从单位四元数提取平面航向角。"""
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def normalize_angle(angle: float) -> float:
    """把角度归一化到 ``[-pi, pi)``。"""
    return math.atan2(math.sin(angle), math.cos(angle))


class AngularVelocityWindowFilter:
    """用一段位姿历史估计平面角速度，滤除四足机身的落足摆动。

    ``gazebo_ros_p3d`` 给出的瞬时角速度会包含机身在单个落足周期内的快速
    左右摆动。它适合观察物理真值，却不适合直接作为控制器的加速度闭环反馈。
    这里不修改里程计位姿，只用展开后的 yaw 在滑动窗口内求平均斜率。
    """

    def __init__(
        self,
        window_duration: float = 1.0,
        minimum_duration: float = 0.5,
    ) -> None:
        if window_duration <= 0.0:
            raise ValueError("window_duration 必须大于零")
        if minimum_duration <= 0.0 or minimum_duration > window_duration:
            raise ValueError("minimum_duration 必须位于 (0, window_duration] 内")
        self._window_duration = window_duration
        self._minimum_duration = minimum_duration
        self._history: Deque[Tuple[float, float]] = deque()
        self._last_stamp: Optional[float] = None
        self._last_yaw: Optional[float] = None
        self._unwrapped_yaw = 0.0

    def reset(self) -> None:
        """清空历史；仿真时间回退或重启时重新建立窗口。"""
        self._history.clear()
        self._last_stamp = None
        self._last_yaw = None
        self._unwrapped_yaw = 0.0

    def update(self, stamp: float, yaw: float) -> float:
        """加入一个带时间戳的 yaw 样本并返回窗口平均角速度。"""
        if self._last_stamp is not None and stamp <= self._last_stamp:
            self.reset()

        if self._last_yaw is None:
            self._last_stamp = stamp
            self._last_yaw = yaw
            self._history.append((stamp, self._unwrapped_yaw))
            return 0.0

        self._unwrapped_yaw += normalize_angle(yaw - self._last_yaw)
        self._last_stamp = stamp
        self._last_yaw = yaw
        self._history.append((stamp, self._unwrapped_yaw))

        cutoff = stamp - self._window_duration
        while len(self._history) >= 2 and self._history[1][0] <= cutoff:
            self._history.popleft()

        oldest_stamp, oldest_yaw = self._history[0]
        duration = stamp - oldest_stamp
        if duration < self._minimum_duration:
            return 0.0
        return (self._unwrapped_yaw - oldest_yaw) / duration


def relative_planar_pose(
    origin: Tuple[float, float, float],
    current: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """将世界坐标位姿变换到以首帧为原点的里程计坐标。"""
    origin_x, origin_y, origin_yaw = origin
    current_x, current_y, current_yaw = current
    delta_x = current_x - origin_x
    delta_y = current_y - origin_y
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    return (
        cosine * delta_x + sine * delta_y,
        -sine * delta_x + cosine * delta_y,
        normalize_angle(current_yaw - origin_yaw),
    )


class SimulationOdom(Node):
    """发布适合 Nav2 的仿真闭环里程计和 TF。"""

    def __init__(self) -> None:
        super().__init__("go2_simulation_odom")
        self.declare_parameter("ground_truth_topic", "/odom/ground_truth")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self.declare_parameter("angular_velocity_window", 1.0)
        angular_velocity_window = float(
            self.get_parameter("angular_velocity_window").value
        )
        self._angular_velocity_filter = AngularVelocityWindowFilter(
            window_duration=angular_velocity_window,
            minimum_duration=min(0.5, angular_velocity_window),
        )
        self._origin: Optional[Tuple[float, float, float]] = None
        self._publisher = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("ground_truth_topic").value),
            self._ground_truth_callback,
            10,
        )
        self.get_logger().info(
            "仿真导航里程计已启动：/odom/ground_truth -> /odom；"
            "首帧位置作为 odom 原点"
        )

    def _ground_truth_callback(self, source: Odometry) -> None:
        source_pose = source.pose.pose
        world_yaw = quaternion_to_yaw(
            source_pose.orientation.x,
            source_pose.orientation.y,
            source_pose.orientation.z,
            source_pose.orientation.w,
        )
        world_pose = (source_pose.position.x, source_pose.position.y, world_yaw)
        if self._origin is None:
            self._origin = world_pose
            self.get_logger().info(
                "已记录 Gazebo 里程计原点："
                f"x={world_pose[0]:.3f}, y={world_pose[1]:.3f}, "
                f"yaw={world_pose[2]:.3f}"
            )

        relative_x, relative_y, relative_yaw = relative_planar_pose(
            self._origin, world_pose
        )
        half_yaw = relative_yaw * 0.5
        yaw_z = math.sin(half_yaw)
        yaw_w = math.cos(half_yaw)

        odometry = Odometry()
        odometry.header.stamp = source.header.stamp
        odometry.header.frame_id = self._odom_frame
        odometry.child_frame_id = self._base_frame
        odometry.pose.pose.position.x = relative_x
        odometry.pose.pose.position.y = relative_y
        odometry.pose.pose.orientation.z = yaw_z
        odometry.pose.pose.orientation.w = yaw_w
        odometry.pose.covariance = source.pose.covariance

        # gazebo_ros_p3d 的速度以世界轴表达，先旋转到当前机身坐标。
        source_twist = source.twist.twist
        current_cosine = math.cos(world_yaw)
        current_sine = math.sin(world_yaw)
        odometry.twist.twist.linear.x = (
            current_cosine * source_twist.linear.x
            + current_sine * source_twist.linear.y
        )
        odometry.twist.twist.linear.y = (
            -current_sine * source_twist.linear.x
            + current_cosine * source_twist.linear.y
        )
        source_stamp = (
            float(source.header.stamp.sec)
            + float(source.header.stamp.nanosec) * 1.0e-9
        )
        odometry.twist.twist.angular.z = self._angular_velocity_filter.update(
            source_stamp, world_yaw
        )
        odometry.twist.covariance = source.twist.covariance
        self._publisher.publish(odometry)

        transform = TransformStamped()
        transform.header = odometry.header
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = relative_x
        transform.transform.translation.y = relative_y
        transform.transform.rotation.z = yaw_z
        transform.transform.rotation.w = yaw_w
        self._tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    """运行仿真真值里程计适配节点。"""
    rclpy.init(args=args)
    node = SimulationOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
