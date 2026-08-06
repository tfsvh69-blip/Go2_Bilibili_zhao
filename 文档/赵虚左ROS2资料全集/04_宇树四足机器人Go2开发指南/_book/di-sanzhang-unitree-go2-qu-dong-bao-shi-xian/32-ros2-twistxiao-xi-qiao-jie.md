## 3.2 ROS2 Twist消息桥接

在ROS2中，运动控制通常使用`geometry_msgs/msg/Twist`消息，而Unitree Go2机器人使用`unitree_api/msg/Request`消息。为了将ROS2生态的功能模块（如导航、键盘控制）移植到Go2，本节将编写一个桥接功能包，用于将`Twist`消息转换为`Request`消息。通过这种方式，`Twist`消息可以间接控制Go2的运动。

