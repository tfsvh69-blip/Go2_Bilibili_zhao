### 1.3.3 HelloWorld \(Python\)

#### 1.创建功能包 {#1创建功能包}

终端下，进入 unitree\_go2\_ws/src/helloworld  目录，使用如下指令创建一个python功能包：

```
ros2 pkg create go2_helloworld_py --build-type ament_python --dependencies rclpy unitree_api
```

#### 2.编辑源文件 {#2编辑源文件}

进入go2\_helloworld\_py/go2\_helloworld\_py 目录，新建hello.py文件，并输入如下内容：

```py
"""  
    需求：unitree go2 在 ROS2 环境下的第一个小程序。
"""
# 1.导包；
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request

# 3.自定义节点类；
class Go2Hello(Node):
    def __init__(self):
        super().__init__("go2_hello_node_py")
        self.req_puber = self.create_publisher(Request,"/api/sport/request",10)
        self.timer = self.create_timer(0.1,self.hello)

    def hello(self):
        request = Request()
        request.header.identity.api_id = 1016
        self.req_puber.publish(request)
def main():
    # 2.初始化ROS2客户端；
    rclpy.init()
    # 4.调用spain函数，并传入节点对象；
    rclpy.spin(Go2Hello())
    # 5.资源释放。 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 3.编辑配置文件 {#3编辑配置文件}

##### 1.package.xml {#1packagexml}

在创建功能包时，所依赖的功能包已经自动配置了，配置内容如下：

```
<depend>rclpy</depend>
<depend>unitree_api</depend>
```

##### 2.setup.py {#2setuppy}

setup.py文件中`entry_points`字段的`console_scripts`中添加如下内容：

```
entry_points={
    'console_scripts': [
        'hello = go2_helloworld_py.hello:main'
    ],
},
```

#### 4.编译 {#4编译}

终端下进入到工作空间，执行如下指令：

```
colcon build --packages-select go2_helloworld_py
```

#### 5.执行 {#5执行}

终端下进入到工作空间，执行如下指令：

```
. install/setup.bash
 ros2 run go2_helloworld_py hello
```

程序执行，go2 将执行打招呼的动作。

