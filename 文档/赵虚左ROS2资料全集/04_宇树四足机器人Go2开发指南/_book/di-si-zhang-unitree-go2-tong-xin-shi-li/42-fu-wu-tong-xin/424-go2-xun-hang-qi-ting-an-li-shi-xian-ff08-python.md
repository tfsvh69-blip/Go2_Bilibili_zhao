### 4.2.4 go2 巡航启停案例实现（Python）

#### 1.服务端节点实现

功能包go2\_tutorial\_py目录下，新建Python文件go2\_cruising\_service.py，并编辑文件，输入如下内容：

```py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from go2_tutorial_inter.srv import Cruising
from geometry_msgs.msg import Point

from go2_tutorial_py.sport_model import *
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterType

# 自定义节点类
class CruServer(Node):
    def __init__(self):
        super().__init__('cru_server_node_node')
        
        # 声明参数
        self.declare_parameter('vx', 0.0)
        self.declare_parameter('vy', 0.0)
        self.declare_parameter('vyaw', 0.5)
        
        # 创建参数服务客户端，连接到速度发布节点
        self.param_client = self.create_client(SetParameters, '/go2_ctrl_node_py/set_parameters')
        
        # 等待服务连接
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                return
            self.get_logger().info('服务未连接')
        
        self.get_logger().info('已经连接成功速度发送节点的参数服务，可以设置线速度和角速度了')
        
        # 创建订阅方，订阅机器人的里程计以获取机器人坐标
        self.sub_odom_ = self.create_subscription(Odometry, 'odom', self.on_timer, 10)
        
        # 创建服务端，处理客户端请求
        self.service = self.create_service(Cruising, 'cruising', self.cb)
        
        # 初始化坐标
        self.current_point = Point()

    def cb(self, request, response):
        flag = request.flag
        id = ROBOT_SPORT_API_IDS["BALANCESTAND"]  
        
        # 判断提交的数据
        if flag != 0:  # 开始巡航
            self.get_logger().info('开始巡航......')
            id = ROBOT_SPORT_API_IDS["MOVE"]
        else:  # 结束巡航
            self.get_logger().info('终止巡航......')
        
        # 设置参
        req = SetParameters.Request()
        
        # 运动模式
        p1 = Parameter()
        
        p1.name = "sport_api_id"
        
        v1 = ParameterValue()
        v1.type = ParameterType.PARAMETER_INTEGER
        v1.integer_value = id

        p1.value = v1
        
        # x线速度
        p2 = Parameter()
        
        p2.name = "vx"
        
        v2 = ParameterValue()
        v2.type = ParameterType.PARAMETER_DOUBLE
        v2.double_value = self.get_parameter("vx").value

        p2.value = v2

        # y线速度
        p3 = Parameter()
        
        p3.name = "vy"
        
        v3 = ParameterValue()
        v3.type = ParameterType.PARAMETER_DOUBLE
        v3.double_value = self.get_parameter("vy").value

        p3.value = v3

        # 角速度
        p4 = Parameter()
        
        p4.name = "vyaw"
        
        v4 = ParameterValue()
        v4.type = ParameterType.PARAMETER_DOUBLE
        v4.double_value = self.get_parameter("vyaw").value

        p4.value = v4

        req.parameters = [
            p1,
            p2,
            p3,
            p4
        ]

        self.param_client.call_async(req)
       
        # 返回当前坐标
        response.point = self.current_point
        return response

    def on_timer(self, odom: Odometry):
        self.current_point = odom.pose.pose.position

def main(args=None):
    # 初始化ROS2客户端
    rclpy.init(args=args)
    
    # 创建节点并运行
    cru_server = CruServer()
    rclpy.spin(cru_server)
    
    # 资源释放
    cru_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 2.客户端节点实现 {#2编辑配置文件}

功能包go2\_tutorial\_py目录下，新建Python文件go2\_cruising\_client.py，并编辑文件，输入如下内容：

```py
import sys
import rclpy
from rclpy.node import Node
from go2_tutorial_inter.srv import Cruising

# 自定义节点类
class CruClient(Node):
    def __init__(self):
        super().__init__('cru_client_node_py')
        # 创建客户端
        self.client = self.create_client(Cruising, 'cruising')
        self.get_logger().info("客户端创建，等待连接服务端！")

    def connect_server(self):
        # 连接服务端
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for the service. Exiting.")
                return False
            self.get_logger().info("服务连接中，请稍候...")
        return True

    def send_request(self, flag):
        # 发送请求
        request = Cruising.Request()
        request.flag = flag
        return self.client.call_async(request)

def main(args=None):
    # 处理通过终端提交的数据
    if len(sys.argv) != 2:
        print("请提交一个整型数据！")
        return 1

    # 初始化ROS2客户端
    rclpy.init(args=args)

    # 创建自定义类对象，并调用连接服务以及请求发送请求的函数
    client = CruClient()

    # 连接服务端
    if not client.connect_server():
        client.get_logger().info("服务连接失败!")
        return 0

    # 发送请求
    future = client.send_request(int(sys.argv[1]))

    # 处理响应结果
    rclpy.spin_until_future_complete(client, future)
    if future.done():
        try:
            response = future.result()
            client.get_logger().info("请求正常处理")
            client.get_logger().info(f"响应坐标: (%.2f,%.2f)" % (response.point.x, response.point.y))
        except Exception as e:
            client.get_logger().error(f"请求异常: {e}")
    else:
        client.get_logger().info("请求未完成")

    # 资源释放
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 3.编写launch和params文件 {#2编辑配置文件}

在 luanch 目录下，新建名为 go2\_cruising\_service.launch.py 的launch文件，并输入如下内容：

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
            executable="go2_cruising_service",
            parameters=[os.path.join(go2_tutorial_pkg,"params","go2_cruising_service.yaml")]
        )
    ])
```

该 launch 文件加载了机器人驱动、运动控制模块，还包含了巡航服务节点，该节点还加载了yaml文件，yaml文件可以在params目录下创建，将文件名命名为 go2\_cruising\_service.yaml，输入如下内容：

```yaml
/**:
  ros__parameters:
    use_sim_time: false
    vx: 0.1
    vy: 0.0
    vyaw: 0.5
```

通过该文件可以配置巡航速度数据。

#### 4.编辑配置文件 {#2编辑配置文件}

##### 1.package.xml {#1packagesxml}

在 package.xml 中添加如下依赖：

```
<depend>geometry_msgs</depend>
<depend>go2_tutorial_inter</depend>
```

##### 2.setup.py {#2cmakeliststxt}

setup.py文件中`entry_points`字段的`console_scripts`中添加如下内容：

```
entry_points={
    'console_scripts': [
        ......
        'go2_cruising_service = go2_tutorial_py.go2_cruising_service:main',
        'go2_cruising_client = go2_tutorial_py.go2_cruising_client:main',
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
ros2 launch go2_tutorial_py go2_cruising_service.launch.py
```

指令执行后，其机器人驱动启动，且巡航服务就绪。

在当前工作空间下，再启动终端，输入如下指令：

```
. install/setup.bash
ros2 run go2_tutorial_py go2_cruising_client 1
```

机器人开始巡航，并返回启动时的坐标。

停止指令如下：

```
ros2 run go2_tutorial_py go2_cruising_client 0
```

机器人结束巡航，也会返回停止时的坐标。

