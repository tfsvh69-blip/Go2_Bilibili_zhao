## 2.3 本章小结

#### 1.核心内容回顾

本章介绍了宇树四足机器人 Go2 的状态获取与机器人控制相关接口，并通过官方例程演示了其使用。包括高层状态接口`unitree_go/msg/SportModeState`、低层状态接口`unitree_go/msg/LowState`、遥控器状态接口`unitree_go/msg/WirelessController`、运动控制接口`unitree_api/msg/Request`和电机控制接口`unitree_go/msg/LowCmd`。这些接口是后续开发中的核心工具。

#### 2.重点难点解析

本章的重点在于深入理解各个接口的数据结构，这也是难点所在，必须根据实际应用理解接口内各个字段的含义。

#### 3.实际应用或意义

这些接口在 unitree go2 的基础功能包编写以及实际应用中被广泛使用，理解这些接口能显著提高开发效率。

#### 4.延伸学习建议

官方提供的接口不仅限于本章介绍的内容，unitree\_api 和 unitree\_go 两个功能包中还定义了许多其他类型的接口，建议读者进一步探索学习。

