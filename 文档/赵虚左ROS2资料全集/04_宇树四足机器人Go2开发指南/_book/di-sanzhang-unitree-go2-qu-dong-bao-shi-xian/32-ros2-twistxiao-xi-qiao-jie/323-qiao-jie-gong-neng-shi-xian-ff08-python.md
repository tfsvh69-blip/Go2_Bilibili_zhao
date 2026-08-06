### 3.2.3 桥接功能实现（Python）

#### 1.节点实现

功能包go2\_twist\_bridge\_py的go2\_twist\_bridge\_py目录下，新建Python文件sport\_model.py，并编辑文件，输入如下内容：

```
# 定义常量字典
ROBOT_SPORT_API_IDS = {
    "DAMP": 1001,                    # 阻尼控制
    "BALANCESTAND": 1002,            # 平衡站立
    "STOPMOVE": 1003,                # 停止运动
    "STANDUP": 1004,                 # 站立
    "STANDDOWN": 1005,               # 站立下降
    "RECOVERYSTAND": 1006,           # 恢复站立
    "EULER": 1007,                   # 欧拉角控制
    "MOVE": 1008,                    # 移动
    "SIT": 1009,                     # 坐下
    "RISESIT": 1010,                 # 从坐下恢复站立
    "SWITCHGAIT": 1011,              # 切换步态
    "TRIGGER": 1012,                 # 触发
    "BODYHEIGHT": 1013,              # 身体高度调整
    "FOOTRAISEHEIGHT": 1014,         # 脚部抬起高度调整
    "SPEEDLEVEL": 1015,              # 速度级别调整
    "HELLO": 1016,                   # 打招呼
    "STRETCH": 1017,                 # 伸展
    "TRAJECTORYFOLLOW": 1018,        # 轨迹跟随
    "CONTINUOUSGAIT": 1019,          # 连续步态
    "CONTENT": 1020,                 # 内容
    "WALLOW": 1021,                  # 打滚
    "DANCE1": 1022,                  # 舞蹈1
    "DANCE2": 1023,                  # 舞蹈2
    "GETBODYHEIGHT": 1024,           # 获取身体高度
    "GETFOOTRAISEHEIGHT": 1025,      # 获取脚部抬起高度
    "GETSPEEDLEVEL": 1026,           # 获取速度级别
    "SWITCHJOYSTICK": 1027,          # 切换操纵杆
    "POSE": 1028,                    # 姿态
    "SCRAPE": 1029,                  # 刮擦
    "FRONTFLIP": 1030,               # 前空翻
    "FRONTJUMP": 1031,               # 前跳
    "FRONTPOUNCE": 1032              # 前扑
}
```

再新建Python文件twist\_bridge.py，并输入如下内容：

```py
# 1.导包
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_api.msg import Request
import json
from .sport_model import ROBOT_SPORT_API_IDS

class Twist2Request(Node):
    def __init__(self):
        super().__init__('twist2request_node_py')
        self.get_logger().info("Convert geometry_msgs.msg.Twist to unitree_api.msg.Request.")

        # 3-1. 创建四组机器人速度指令发布对象
        self.req_pub = self.create_publisher(Request, '/api/sport/request', 10)

        # 3-2. 创建ROS2速度指令订阅对象
        self.twist_sub = self.create_subscription(
            Twist, 'cmd_vel', self.twist_to_request, 10
        )

    # 3-3. 在订阅对象的回调函数中将Twist转换成Request并发布
    def twist_to_request(self, twist):
        request = Request()
        api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]

        # 转换 (只需要x、y的线速度和z的角速度)
        x = twist.linear.x
        y = twist.linear.y
        th = twist.angular.z

        if x != 0 or y != 0 or th != 0:
            api_id = ROBOT_SPORT_API_IDS["MOVE"]

            # 设置线速度与角速度
            js = {"x": x, "y": y, "z": th}
            request.parameter = json.dumps(js)
            self.get_logger().info(f"Current speed: {request.parameter}")

        request.header.identity.api_id = api_id
        self.req_pub.publish(request)

def main(args=None):
    # 2. 初始化ROS2客户端
    rclpy.init(args=args)

    # 4. 调用spin函数，并传入节点对象
    node = Twist2Request()
    rclpy.spin(node)

    # 5. 资源释放
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 2.编辑配置文件 {#2编辑配置文件}

##### 1.package.xml {#1packagexml}

在创建功能包时，所依赖的功能包已经自动配置了，配置内容如下：

```
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>unitree_api</depend>
```

##### 2.setup.py {#2setuppy}

setup.py文件中`entry_points`字段的`console_scripts`中添加如下内容：

```
entry_points={
    'console_scripts': [
        'twist_bridge = go2_twist_bridge_py.twist_bridge:main'
    ],
},
```

#### 3.编译 {#3编译}

终端中进入当前工作空间，编译功能包：

```
colcon build --packages-select go2_twist_bridge_py
```

#### 4.执行 {#4执行}

在当前工作空间下，启动终端，并输入如下指令：

```
. install/setup.bash
ros2 run go2_twist_bridge_py twist_bridge
```

再新建一个终端，启动 ROS2 中常用的键盘控制节点：

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

至此，就可以通过键盘控制 go2 的运动了。

