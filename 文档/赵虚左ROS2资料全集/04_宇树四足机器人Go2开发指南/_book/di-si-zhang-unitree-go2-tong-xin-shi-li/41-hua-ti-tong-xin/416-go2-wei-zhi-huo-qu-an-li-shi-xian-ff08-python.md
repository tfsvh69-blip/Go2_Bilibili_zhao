### 4.1.6 go2 位置获取案例实现（Python）

#### 1.节点实现

功能包go2\_tutorial\_py目录下，新建Python文件go2\_state.py，并编辑文件，输入如下内容：

```py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class SubOdom(Node):
    def __init__(self):
        super().__init__('sub_odom_node')

        # 初始化上一次记录的坐标
        self.last_x = 0.0
        self.last_y = 0.0
        self.is_first = True
        # 声明参数，设置默认距离阈值为0.5
        self.declare_parameter('distance', 0.1)

        # 创建里程计订阅方
        self.sub_odom_ = self.create_subscription(
            Odometry,  # 消息类型
            'odom',     # 话题名称
            self.on_timer,  # 回调函数
            10          # 队列长度
        )

    # 解析里程计数据，并当条件满足时，在终端输出坐标
    def on_timer(self, odom):
        # 获取当前坐标
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y

        if self.is_first:
            self.last_x = x
            self.last_y = y
            self.is_first = False
            return

        # 计算当前坐标与上一次输出坐标的直线距离
        distance_x = x - self.last_x
        distance_y = y - self.last_y
        distance = math.sqrt(distance_x**2 + distance_y**2)

        # 获取参数值
        distance_threshold = self.get_parameter('distance').get_parameter_value().double_value

        # 判断是否符合条件
        if distance >= distance_threshold:
            # 输出当前坐标
            self.get_logger().info(f'当前坐标({x:.2f}, {y:.2f})')
            # 更新上一次记录的坐标
            self.last_x = x
            self.last_y = y

def main(args=None):
    # 初始化ROS2客户端
    rclpy.init(args=args)

    # 创建节点对象
    sub_odom = SubOdom()

    # 调用spin函数，并传入节点对象指针
    rclpy.spin(sub_odom)

    # 资源释放
    sub_odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 2.编写launch和params文件 {#2编辑配置文件}

在 luanch 目录下，新建名为 go2\_state.launch.py 的launch文件，并输入如下内容：

```py
from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
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

import os

def generate_launch_description():

    go2_driver_pkg = get_package_share_directory("go2_driver_py")
    go2_tutorial_pkg = get_package_share_directory("go2_tutorial_py")

    return LaunchDescription([
        # 驱动 launch
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=[os.path.join(go2_driver_pkg,"launch","driver.launch.py")]
            )
        ),
        Node(
            package="go2_tutorial_py",
            executable="go2_state",
            parameters=[os.path.join(go2_tutorial_pkg,"params","go2_state.yaml")]
        )
    ])
```

该 launch 文件加载了机器人驱动且启动了位置获取节点，后者还加载了yaml文件，该文件可以在params目录下创建，将文件名命名为 go2\_state.yaml，并输入如下内容：

```yaml
/**:
  ros__parameters:
    distance: 0.1
    use_sim_time: false
```

通过该文件可以配置参与运算的阈值数据。

#### 3.编辑配置文件 {#2编辑配置文件}

##### 1.package.xml {#1packagesxml}

在 package.xml 中添加如下依赖：

```
<depend>nav_msgs</depend>
```

##### 2.setup.py {#2cmakeliststxt}

setup.py文件中`entry_points`字段的`console_scripts`中添加如下内容：

```
entry_points={
    'console_scripts': [
        ......
        'go2_state = go2_tutorial_py.go2_state:main',
    ],
},
```

#### 4.编译 {#3编译}

终端中进入当前工作空间，编译功能包：

```
colcon build --packages-select go2_tutorial_py
```

#### 5.执行 {#4执行}

在当前工作空间下，启动终端，并输入如下指令：

```
. install/setup.bash
ros2 launch go2_tutorial_py go2_state.launch.py
```

使用键盘或手柄控制机器人运动，当位移超出指定阈值时，就会输出机器人当时坐标。

