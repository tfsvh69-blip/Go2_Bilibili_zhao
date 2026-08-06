### 3.2.1 功能简介以及实现分析

#### 1.功能简介 {#1案例需求}

**需求：**编写 ROS2 节点，该节点可以将`geometry_msgs/msg/Twist`消息转换成`unitree_api/msg/Request`消息以控制 go2 运动。

#### 2.需求分析 {#2案例分析}

该功能的实现核心是数据的传递，关注点如下：

* **订阅对象**：订阅`geometry_msgs/msg/Twist`消息。

* **消息转换**：解析`geometry_msgs/msg/Twist`消息并生成`unitree_api/msg/Request`消息。

* **发布对象**：发布`unitree_api/msg/Request`消息。

由于`geometry_msgs/msg/Twist`消息包含的内容有限，因此仅实现一般速度指令的转换，例如线速度和角速度。

#### 3.流程简介 {#3流程简介}

主要步骤如下：

1. 编写节点实现；
2. 编辑配置文件；
3. 编译；
4. 执行。

该功能会采用C++和Python分别实现，二者都遵循上述实现流程。

#### 4.准备工作 {#4准备工作}

终端下进入工作空间的src/base目录，调用如下指令分别创建C++功能包和Python功能包。

```
ros2 pkg create go2_twist_bridge --build-type ament_cmake --dependencies rclcpp geometry_msgs unitree_api
ros2 pkg create go2_twist_bridge_py --build-type ament_python --dependencies rclpy geometry_msgs unitree_api
```



