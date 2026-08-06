### 4.3.4 go2 导航案例实现（Python）

#### 1.服务端节点实现

功能包go2\_tutorial\_py目录下，新建Python文件go2\_nav\_server.py，并编辑文件，输入如下内容：

```py
"""  
    需求：简单的模拟导航功能，向机器人发送一个前进N米的请求，机器人就会以 0.1m/s的速度前进，
         当与目标点的距离小于0.05m时，机器人就会停止运动，返回机器人的停止坐标，并且在此过程中，
         会连续反馈机器人与目标点之间的剩余距离。
    步骤：
        1.导包；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建动作服务端；
            3-2.生成连续反馈；
            3-3.生成最终响应。
        4.调用spin函数，并传入节点对象；
        5.释放资源。
"""

# 1.导包；
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from go2_tutorial_inter.action import Nav
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from .sport_model import ROBOT_SPORT_API_IDS
from rclpy.action.server import GoalResponse, CancelResponse
import math

class OdomSub(Node):
    def __init__(self):
        super().__init__('odom_sub_server')
        # 创建订阅方，订阅机器人的里程计以获取机器人坐标。
        self.position = None
        self.odom_sub = self.create_subscription(Odometry,"/odom",self.odom_cb,10)

    def odom_cb(self,odom: Odometry):
        self.position = odom.pose.pose.position

    def get_current_pose(self):
        while self.position is None:
            rclpy.spin_once(self)
        rclpy.spin_once(self)
        return self.position
# 3.定义节点类；
class NavServer(Node):

    def __init__(self):
        super().__init__('nav_action_server_py')
        
        self.declare_parameter("vx",0.1)

        self.odom_sub = OdomSub()

        # 创建参数服务客户端
        self.param_client = self.create_client(SetParameters, 'go2_ctrl_node_py/set_parameters')
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                return
            self.get_logger().info('服务未连接')
        self.get_logger().info('已经连接成功速度发送节点的参数服务，可以设置线速度和角速度了')
        # 3-1.创建动作服务端；
        self._action_server = ActionServer(
            self,
            Nav,
            'nav',
            self.execute_callback,
            goal_callback=self.handle_goal,
            )

    def handle_goal(self, goal_request):
        # 解析动作客户端发送的请求
        goal_distance = goal_request.goal
        if goal_distance > 0.0:
            self.get_logger().info(f'请求前进 {goal_distance:.2f} 米')
            self.start_point = self.odom_sub.get_current_pose()
            return GoalResponse.ACCEPT
        else:
            self.get_logger().info('只许进，不许退!')
            return GoalResponse.REJECT

    def go(self):
        
        # 设置机器人速度
        req = SetParameters.Request()
        p1 = Parameter()
        p1.name = "sport_api_id"
        v1 = ParameterValue()
        v1.type = ParameterType.PARAMETER_INTEGER
        v1.integer_value = ROBOT_SPORT_API_IDS["MOVE"]
        p1.value = v1

        p2 = Parameter()
        p2.name = "vx"
        v2 = ParameterValue()
        v2.type = ParameterType.PARAMETER_DOUBLE
        v2.double_value = self.get_parameter("vx").value
        p2.value = v2

        req.parameters = [p1, p2]
        self.param_client.call_async(req)
    
    def stop(self):
       
        # 设置机器人速度
        req = SetParameters.Request()
        p1 = Parameter()
        p1.name = "sport_api_id"
        v1 = ParameterValue()
        v1.type = ParameterType.PARAMETER_INTEGER
        v1.integer_value = ROBOT_SPORT_API_IDS["BALANCESTAND"]
        p1.value = v1

        p2 = Parameter()
        p2.name = "vx"
        v2 = ParameterValue()
        v2.type = ParameterType.PARAMETER_DOUBLE
        v2.double_value = 0.0
        p2.value = v2

        req.parameters = [p1, p2]
        self.param_client.call_async(req)

    def execute_callback(self, goal_handle):
        self.get_logger().info('开始执行任务....')
        # 开始运动
        self.go() 
        # 3-2.生成连续反馈；
        goal = goal_handle.request.goal
        feedback_msg = Nav.Feedback()
        # 组织消息
        while rclpy.ok():
            # 获取当前坐标
            pos = self.odom_sub.get_current_pose()
            # 获取剩余距离
            distance = goal - math.sqrt((pos.x - self.start_point.x) ** 2 + (pos.y - self.start_point.y) ** 2)
            # 生成并发布连续反馈
            feedback_msg.distance = distance
            goal_handle.publish_feedback(feedback_msg)
            if distance <= 0.1:
                break
            time.sleep(0.2)
        # 停止
        self.stop()
         # 3-3.生成最终响应。
        goal_handle.succeed()
        result = Nav.Result()
        result.point = self.odom_sub.get_current_pose()
        self.get_logger().info('任务完成！')
        return result

def main(args=None):

    # 2.初始化 ROS2 客户端；
    rclpy.init(args=args)

    # 4.调用spin函数，并传入节点对象；
    Nav_action_server = NavServer()
    rclpy.spin(Nav_action_server)

    # 5.释放资源。
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

#### 2.客户端节点实现 {#2编辑配置文件}

功能包go2\_tutorial\_py目录下，新建Python文件go2\_nav\_client.py，并编辑文件，输入如下内容：

```py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from go2_tutorial_inter.action import Nav
import sys

class NavClient(Node):
    def __init__(self):
        super().__init__('exe_nav_action_client')
        # 创建动作客户端；
        self.nav_client = ActionClient(self, Nav, 'nav')

    # 发送请求数据，并处理服务端响应；
    def send_goal(self, x):
        # 连接动作服务端，如果超时（5s），那么直接退出。
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("服务连接失败!")
            return

        # 组织请求数据
        goal_msg = Nav.Goal()
        goal_msg.goal = x

        # 发送请求
        self.get_logger().info("发送目标点请求...")
        self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)

    # 处理目标响应；
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("目标请求被服务器拒绝")
            sys.exit()

        self.get_logger().info("目标请求被接收!")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # 处理响应的连续反馈；
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"距离目标点还有 %.2f 米。" % feedback.distance)

    # 处理最终响应。
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"go2最终坐标: (%.2f, %.2f)" % (result.point.x, result.point.y))
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):

    if len(sys.argv) != 2:
        client.get_logger().info("请传入要前进的距离数据")
        return 1

    # 初始化 ROS2 客户端；
    rclpy.init(args=args)
    client = NavClient()
    # 发送目标点
    client.send_goal(float(sys.argv[1]))
    # 调用spin函数，并传入节点；
    rclpy.spin(client)

    # 释放资源。
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 3.编写launch和params文件 {#2编辑配置文件}

在 luanch 目录下，新建名为 go2\_nav\_server.launch.py 的launch文件，并输入如下内容：

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
        # 运动控制
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=[os.path.join(go2_tutorial_pkg,"launch","go2_ctrl.launch.py")]
            )
        ),
        Node(
            package="go2_tutorial_py",
            executable="go2_nav_server",
            parameters=[os.path.join(go2_tutorial_pkg,"params","go2_nav_server.yaml")]
        )
    ])
```

该 launch 文件加载了机器人驱动、运动控制模块，还包含了导航服务节点，该节点还加载了yaml文件，yaml文件可以在params目录下创建，将文件名命名为 go2\_nav\_server.yaml，输入如下内容：

```yaml
/**:
  ros__parameters:
    use_sim_time: false
    vx: 0.1
    error: 0.2
```

通过该文件可以配置巡航速度数据。

#### 4.编辑配置文件 {#2编辑配置文件}

##### 1.package.xml {#1packagesxml}

package.xml 无需修改，使用之前配置即可。

##### 2.setup.py {#2cmakeliststxt}

setup.py文件中`entry_points`字段的`console_scripts`中添加如下内容：

```
entry_points={
    'console_scripts': [
        ......
        'go2_nav_server = go2_tutorial_py.go2_nav_server:main',
        'go2_nav_client = go2_tutorial_py.go2_nav_client:main',
    ],
},
```

#### 5.编译 {#3编译}

终端中进入当前工作空间，编译功能包：

```
colcon build --packages-select go2_tutorial_py
```

#### 6.执行 {#4执行}

在当前工作空间下，启动终端，输入如下指令：

```
. install/setup.bash
ros2 launch go2_tutorial_py go2_nav_server.launch.py
```

指令执行后，其机器人驱动启动，且导航服务就绪。

在当前工作空间下，再启动终端，输入如下指令：

```
. install/setup.bash
ros2 run go2_tutorial_py go2_nav_client 0.5
```

机器人开始导航，并连续反馈剩余距离，到达目标点后会结束导航，并返回机器人的停止坐标。

