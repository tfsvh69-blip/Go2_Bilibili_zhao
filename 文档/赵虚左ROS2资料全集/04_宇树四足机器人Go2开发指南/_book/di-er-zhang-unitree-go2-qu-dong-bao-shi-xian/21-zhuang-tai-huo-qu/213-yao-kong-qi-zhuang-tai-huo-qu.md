### 2.1.3 遥控器状态获取

#### 1.消息类型

通过订阅`/wirelesscontroller` topic可获取遥控器的摇杆数值和按键键值。

通过调用如下指令可以获取遥控器状态对应的消息接口类型：

```
ros2 topic type /wirelesscontroller
```

其输出结果为：`unitree_go/msg/WirelessController`。

#### 2.数据结构

通过如下指令，可以获取遥控器状态消息接口的数据结构：

```
ros2 interface show unitree_go/msg/WirelessController
```

其输出结果如下：

```
float32 lx //左边摇杆x
float32 ly //左边摇杆y
float32 rx //右边摇杆x
float32 ry //右边摇杆y
uint16 keys //键值
```

接口定义的源文件为：`~/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go/msg/WirelessController.msg`。

#### 3.内置例程

读取遥控器状态的完整例程位于`~/unitree_ros2/example/src/read_wireless_controller.cpp`。

编译完例程后，在终端中进入`unitree_ros2/example`目录并运行如下指令：

```
./install/unitree_ros2_example/bin/read_wireless_controller
```

当操作遥控器时，终端会输出类似如下结果：

```
[INFO] [1740039928.057209218] [wireless_controller_suber]: Wireless controller -- lx: -0.000000; ly: 0.595012; rx: 0.992866; ry: -0.000000; key value: 0
[INFO] [1740039928.107205501] [wireless_controller_suber]: Wireless controller -- lx: -0.000000; ly: 0.595012; rx: 0.988327; ry: -0.000000; key value: 0
[INFO] [1740039928.157260543] [wireless_controller_suber]: Wireless controller -- lx: -0.000000; ly: 0.595012; rx: 0.981193; ry: -0.000000; key value: 0
[INFO] [1740039928.207986271] [wireless_controller_suber]: Wireless controller -- lx: -0.000000; ly: 0.595012; rx: 0.814527; ry: -0.000000; key value: 0
[INFO] [1740039928.257210549] [wireless_controller_suber]: Wireless controller -- lx: -0.000000; ly: 0.548694; rx: -0.000000; ry: -0.000000; key value: 0
[INFO] [1740039928.307294435] [wireless_controller_suber]: Wireless controller -- lx: -0.000000; ly: 0.000000; rx: -0.000000; ry: -0.000000; key value: 0
[INFO] [1740039928.357286837] [wireless_controller_suber]: Wireless controller -- lx: -0.000000; ly: 0.000000; rx: -0.000000; ry: -0.000000; key value: 0
[INFO] [1740039928.407168026] [wireless_controller_suber]: Wireless controller -- lx: -0.000000; ly: 0.000000; rx: -0.000000; ry: -0.000000; key value: 0
```



