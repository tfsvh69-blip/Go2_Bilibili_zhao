### 4.1.3 go2 巡航案例实现（Python）

#### 1.节点实现

请先将当前工作空间`src/tutorial/go2_driver_py/go2_driver_py`目录下的 sport\_model.hpp 文件，复制到当前功能包 go2\_tutorial\_py 目录下。

新建Python文件go2\_ctrl.py，并编辑文件，输入如下内容：

```py
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request
import json

# from go2_tutorial_py.sport_model import *
# from .sport_model import *
from go2_tutorial_py import sport_model

# import sport_model # ModuleNotFoundError: No module named 'sport_model'

class Go2Ctrl(Node):
    def __init__(self):
        super().__init__('go2_ctrl_node_py')

        # 声明参数
        self.declare_parameter('vx', 0.0)
        self.declare_parameter('vy', 0.0)
        self.declare_parameter('vyaw', 0.0)
        self.declare_parameter('sport_api_id', sport_model.ROBOT_SPORT_API_IDS["BALANCESTAND"])

        # 创建一个ROS2 Publisher
        self.req_puber = self.create_publisher(Request, '/api/sport/request', 10)

        # 创建一个定时器，每100毫秒调用一次cruise函数
        self.timer = self.create_timer(0.1, self.cruise)

    def cruise(self):
        req = Request()  # 创建一个运动请求msg
        id = self.get_parameter('sport_api_id').get_parameter_value().integer_value

        req.header.identity.api_id = sport_model.ROBOT_SPORT_API_IDS["MOVE"]

        if id == sport_model.ROBOT_SPORT_API_IDS["MOVE"]:
            js = {
                "x": self.get_parameter('vx').get_parameter_value().double_value,
                "y": self.get_parameter('vy').get_parameter_value().double_value,
                "z": self.get_parameter('vyaw').get_parameter_value().double_value
            }
            req.parameter = json.dumps(js)

            # 或者直接使用字符串
            # req.parameter = '{"x": 0.0, "y": 0.0, "z": 0.6}'
            # self.get_logger().info(f'req.param = {req.parameter}')

        self.req_puber.publish(req)  # 发布数据

def main(args=None):
    # 初始化ROS2客户端
    rclpy.init(args=args)

    # 创建节点对象
    go2_ctrl = Go2Ctrl()

    # 运行节点
    rclpy.spin(go2_ctrl)

    # 资源释放
    go2_ctrl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 2.编辑配置文件 {#2编辑配置文件}

##### 1.package.xml {#1packagesxml}

在创建功能包时，所依赖的功能包已经自动配置了，配置内容如下：

```
<depend>rclpy</depend>
<depend>unitree_go</depend>
<depend>unitree_api</depend>
```

##### 2.setup.py {#2cmakeliststxt}

setup.py文件中`entry_points`字段的`console_scripts`中添加如下内容：

```
entry_points={
    'console_scripts': [
        'go2_ctrl = go2_tutorial_py.go2_ctrl:main',
    ],
},
```

#### 3.编译 {#3编译}

终端中进入当前工作空间，编译功能包：

```
colcon build --packages-select go2_tutorial_py
```

#### 4.执行 {#4执行}

在当前工作空间下，启动终端，并输入如下指令：

```
. install/setup.bash
ros2 run go2_tutorial_py go2_ctrl
```

再新建一个终端，启动 rqt：

```
rqt
```

配置与操作参考 **4.1.2 go2 巡航案例实现（C++）**即可。

