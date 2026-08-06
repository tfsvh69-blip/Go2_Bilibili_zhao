### 3.4.3 驱动包实现（Python）

#### 1.节点实现

功能包 go2\_driver\_py 的 go2\_driver\_py 目录下，新建 Python 文件 driver.py，并编辑文件，输入如下内容：

```cpp
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from unitree_go.msg import LowState, SportModeState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class Driver(Node):
    def __init__(self):
        super().__init__('driver_py')

        # 初始化参数
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base')

        self.publish_odom_tf = self.get_parameter('publish_odom_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # 发布里程计消息
        self.sport_mode_state_suber_ = self.create_subscription(
            SportModeState, 'lf/sportmodestate', self.state_callback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster_ = TransformBroadcaster(self)

        # 发布关节消息
        self.joint_names_ = [
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
        ]
        self.joint_state_pub_ = self.create_publisher(JointState, '/joint_states', 10)
        self.low_state_suber_ = self.create_subscription(
            LowState, 'lf/lowstate', self.low_state_callback, 10)

    def state_callback(self, data):
        # 创建里程计消息
        odom_msg = Odometry()

        # 设置时间戳
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # 设置位置
        odom_msg.pose.pose.position.x = float(data.position[0])
        odom_msg.pose.pose.position.y = float(data.position[1])
        odom_msg.pose.pose.position.z = float(data.position[2])

        # 设置姿态
        odom_msg.pose.pose.orientation.w = float(data.imu_state.quaternion[0])
        odom_msg.pose.pose.orientation.x = float(data.imu_state.quaternion[1])
        odom_msg.pose.pose.orientation.y = float(data.imu_state.quaternion[2])
        odom_msg.pose.pose.orientation.z = float(data.imu_state.quaternion[3])

        # 设置线速度
        odom_msg.twist.twist.linear.x = float(data.velocity[0])
        odom_msg.twist.twist.linear.y = float(data.velocity[1])
        odom_msg.twist.twist.linear.z = float(data.velocity[2])

        # 设置角速度
        odom_msg.twist.twist.angular.z = float(data.yaw_speed)

        # 发布里程计消息
        self.odom_pub_.publish(odom_msg)

        # 根据参数选择是否发布坐标变换
        if self.publish_odom_tf:
            transformStamped = TransformStamped()

            # 设置时间戳
            transformStamped.header.stamp = self.get_clock().now().to_msg()
            transformStamped.header.frame_id = self.odom_frame
            transformStamped.child_frame_id = self.base_frame

            # 设置平移
            transformStamped.transform.translation.x = float(data.position[0])
            transformStamped.transform.translation.y = float(data.position[1])
            transformStamped.transform.translation.z = float(data.position[2])

            # 设置旋转
            transformStamped.transform.rotation = odom_msg.pose.pose.orientation

            # 发布坐标变换
            self.tf_broadcaster_.sendTransform(transformStamped)

    def low_state_callback(self, data):
        # 填充关节状态消息
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names_
        ms = data.motor_state
        for i in range(12):
            joint_state_msg.position.append(float(ms[i].q))  # 确保转换为float
        self.joint_state_pub_.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    driver = Driver()
    rclpy.spin(driver)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 2.编写params、rviz、launch文件 {#2编辑配置文件}

**1.params文件**

在当前功能包下新建目录 params，params 目录中新建 driver.yaml 文件，并输入如下内容：

```
/**:
  ros__parameters:
    base_frame: base
    odom_frame: odom
    publish_odom_tf: true
    use_sim_time: false
```

**2.rviz文件**

在当前功能包下新建目录 rviz。再启动 rviz2 并将配置文件命名为display.rviz，然后保存至该目录下以作备用。

**3.launch文件**

在当前功能包下新建目录 launch，launch 目录中新建 driver.launch.py，并输入如下内容：

```py
from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


import os

def generate_launch_description():

    go2_descrition_pkg = get_package_share_directory("go2_description")
    go2_driver_pkg = get_package_share_directory("go2_driver_py")

    # 声明一个布尔参数，用于控制是否启动 rviz
    use_rviz = DeclareLaunchArgument(
        name="use_rviz",
        default_value="true",  # 默认值为 true
    )

    return LaunchDescription([
        use_rviz,
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=os.path.join(go2_descrition_pkg,"launch","display.launch.py")
            ),
            launch_arguments=[("use_joint_state_publisher","false")]
        ),
        Node(
            package="go2_driver_py",
            executable="driver",
            output="screen",
            parameters=[os.path.join(go2_driver_pkg,"params","driver.yaml")]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            arguments=["-d",os.path.join(go2_driver_pkg,"rviz","display.rviz")]
        ),
        # 静态坐标变换
        # 官方URDF文件中的雷达坐标系是radar，发布的点云中使用的坐标系是utlidar_lidar,
        # 为了雷达能正常显示,可以直接修改URDF中的坐标系为utlidar_lidar，
        # 或者可以将utlidar_lidar等位姿的变换至radar
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", "radar", "--child-frame-id", "utlidar_lidar"]
        ),
        # 速度指令转换节点，可以将geometry_msgs::msg::Twist 转换成 unitree_api::msg::Request 消息
        Node(
            package="go2_twist_bridge_py",
            executable="twist_bridge"
        )
    ])
```

#### 3.编辑配置文件 {#2编辑配置文件}

##### 1.package.xml {#1packagesxml}

在创建功能包时，所依赖的功能包已经自动配置了，配置内容如下：

```
<depend>rclpy</depend>
<depend>unitree_go</depend>
<depend>sensor_msgs</depend>
<depend>tf2_ros</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
```

##### 2.setup.py {#2cmakeliststxt}

setup.py文件修改的内容如下：

```
......
data_files=[
    ......
    ('share/' + package_name + "/launch", glob("launch/*.launch.py")),
    ('share/' + package_name + "/params", glob("params/*.yaml")),
    ('share/' + package_name + "/rviz", glob("rviz/*.rviz")),
],
......
entry_points={
    'console_scripts': [
        'driver = go2_driver_py.driver:main'
    ],
},
......
```

#### 3.编译 {#3编译}

终端中进入当前工作空间，编译功能包：

```
colcon build --packages-select go2_driver_py
```

#### 4.执行 {#4执行}

在当前工作空间下，启动终端，并输入如下指令：

```
. install/setup.bash
ros2 launch go2_driver_py driver.launch.py
```

后续操作可参考** 3.4.2 驱动包实现（C++）**中的执行操作。

