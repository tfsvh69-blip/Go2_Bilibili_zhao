### 2.2.2 电机控制

#### 1.消息类型

通过订阅`/lowcmd`topic，并发送`unitree_go::msg::LowCmd`可以实现对电机的力矩、位置、和速度控制。

通过调用如下指令可以获取电机控制对应的消息接口类型：

```
ros2 topic type /lowcmd
```

其输出结果为：`unitree_go/msg/LowCmd`。

#### 2.数据结构

通过如下指令，可以获取电机控制消息接口的数据结构：

```
ros2 interface show unitree_go/msg/LowCmd
```

其输出结果如下：

```
uint8[2] head
uint8 level_flag
uint8 frame_reserve
uint32[2] sn
uint32[2] version
uint16 bandwidth
MotorCmd[20] motor_cmd // 电机指令
        uint8 mode;  //电机控制模式（Foc模式（工作模式）-> 0x01 ，stop模式（待机模式）-> 0x00
        float q;     //关节目标位置
        float dq;    //关节目标速度
        float tau;   //关节目标力矩
        float kp;    //关节刚度系数
        float kd;    //关节阻尼系数
        unsigned long reserve[3];   //保留位
BmsCmd bms_cmd
        uint8 off
        uint8[3] reserve
uint8[40] wireless_remote
uint8[12] led
uint8[2] fan
uint8 gpio
uint32 reserve
uint32 crc
```

接口定义的源文件为：`~/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go/msg/LowCmd.msg`。

#### 3.内置例程

电机控制的完整例程见`example/src/low_level_ctrl.cpp`。

在运行该例程前需要先关闭 Go2 的主运控服务\(sport\_mode\)。 可在 App-设置-服务状态 里点击对应的服务（轮足机器人服务名称为：wheeled\_sport/1.x.x.x，普通四足机器人服务名称为：sport\_mode/1.x.x.x）关闭。为了方便实验，服务关闭后，请将机器人架空。

![](/assets/关闭运动服务.png)

> 注意
>
> 之所以需要先关闭主运控服务\(sport\_mode\)，这是因为底层控制例程也相当于一个运控服务，它们均会发送控制指令给 Go2 ，如果多个运控同时存在，则Go2机器人会同时接收两个或多个控制指令而产生混乱，造成机器狗失控。故在运行底层控制例程序前，需要确保对应的服务处于关闭状态。

最后，在终端中进入`unitree_ros2/example`目录并运行如下指令：

```
./install/unitree_ros2_example/bin/low_level_ctrl
```

左后腿的机身电机和小腿电机会转动到对应关节角度。

