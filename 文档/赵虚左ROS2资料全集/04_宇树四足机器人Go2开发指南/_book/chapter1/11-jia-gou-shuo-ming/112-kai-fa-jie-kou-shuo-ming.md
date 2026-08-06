### 1.1.2 开发接口说明

宇树科技新一代机器人（Go2、B2和H1）采用 DDS 作为消息中间件，ROS2 也使用 DDS 作为通讯工具，因此 Go2 机器人的底层可以兼容 ROS2，直接使用 ROS2 自带的 msg 直接进行通讯和控制。

除此之外，宇树科技还提供了SDK：**unitree\_sdk2**（[https://github.com/unitreerobotics/unitree\_sdk2](https://github.com/unitreerobotics/unitree_sdk2)）。unitree\_sdk2是基于 cyclonedds 实现的一个易用的机器人数据通信机制，应用开发者可以利用这一接口实现机器人的数据通讯和指令控制。

在架构上，ROS2 与 unitree\_sdk2 是平级关系，ROS2与机器人通信时无需通过sdk接口转发。另外需要注意的是：unitree go2 目前支持二次开发的机器人为 EDU 版。其他型号或版本的go2机器人不支持二次开发。

