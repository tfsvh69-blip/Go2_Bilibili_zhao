### 2.1.2 低层状态获取

#### 1.消息类型

低层状态为机器人的关节电机、电源信息等底层状态。通过订阅`lf/lowstate`或`lowstate`topic，可实现低层状态的获取。其中`lf`表示低频率。

通过调用如下指令可以获取低层状态对应的消息接口类型：

```
ros2 topic type /lowstate
```

或

```
ros2 topic type /lf/lowstate
```

其输出结果为：`unitree_go/msg/LowState`。

#### 2.数据结构

通过如下指令，可以获取低层状态消息接口的数据结构：

```
ros2 interface show unitree_go/msg/LowState
```

其输出结果如下：

```
uint8[2] head
uint8 level_flag
uint8 frame_reserve
uint32[2] sn
uint32[2] version
uint16 bandwidth
IMUState imu_state
        float32[4] quaternion
        float32[3] gyroscope
        float32[3] accelerometer
        float32[3] rpy
        int8 temperature
MotorState[20] motor_state //电机状态
        uint8 mode //运动模式
        float32 q //当前角度
        float32 dq //当前角速度
        float32 ddq //当前角加速度
        float32 tau_est //估计的外力
        float32 q_raw //当前角度原始数值
        float32 dq_raw //当前角速度原始数值
        float32 ddq_raw //当前角加速度原始数值
        int8 temperature //温度
        uint32 lost
        uint32[2] reserve
BmsState bms_state
        uint8 version_high
        uint8 version_low
        uint8 status
        uint8 soc
        int32 current
        uint16 cycle
        int8[2] bq_ntc
        int8[2] mcu_ntc
        uint16[15] cell_vol
int16[4] foot_force //足端力数值
int16[4] foot_force_est //估计的足端力
uint32 tick
uint8[40] wireless_remote
uint8 bit_flag
float32 adc_reel
int8 temperature_ntc1
int8 temperature_ntc2
float32 power_v //电池电压
float32 power_a //电池电流
uint16[4] fan_frequency
uint32 reserve
uint32 crc
```

接口定义的源文件为：`~/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go/msg/LowState.msg`。

#### 3.内置例程

读取高低层状态的完整例程位于`~/unitree_ros2/example/src/read_low_state.cpp`。

编译完例程后，在终端中进入`unitree_ros2/example`目录并运行如下指令：

```
./install/unitree_ros2_example/bin/read_low_state
```

终端会输出类似如下结果：

```
[INFO] [1740039167.324191047] [low_state_suber]: Foot force -- foot0: 14; foot1: 15; foot2: 15; foot3: 16
[INFO] [1740039167.324208281] [low_state_suber]: Estimated foot force -- foot0: 0; foot1: 0; foot2: 0; foot3: 0
[INFO] [1740039167.324224457] [low_state_suber]: Battery state -- current: 0.239272; voltage: 29.588234
[INFO] [1740039167.326196163] [low_state_suber]: Euler angle -- roll: 0.015295; pitch: -0.080054; yaw: 0.013607
[INFO] [1740039167.326263398] [low_state_suber]: Quaternion -- qw: 0.999145; qx: 0.007914; qy: -0.039962; qz: 0.007104
[INFO] [1740039167.326282772] [low_state_suber]: Gyroscope -- wx: -0.005326; wy: 0.009587; wz: -0.005326
[INFO] [1740039167.326300384] [low_state_suber]: Accelerometer -- ax: 0.748188; ay: 0.050278; az: 9.576807
[INFO] [1740039167.326318195] [low_state_suber]: Motor state -- num: 0; q: -0.052630; dq: -0.011627; ddq: 0.000000; tau: -0.049477
[INFO] [1740039167.326336686] [low_state_suber]: Motor state -- num: 1; q: 1.262799; dq: -0.003876; ddq: 0.000000; tau: 0.074215
[INFO] [1740039167.326354628] [low_state_suber]: Motor state -- num: 2; q: -2.782914; dq: 0.030330; ddq: 0.000000; tau: 0.000000
[INFO] [1740039167.326371903] [low_state_suber]: Motor state -- num: 3; q: 0.045272; dq: 0.019378; ddq: 0.000000; tau: -0.123691
[INFO] [1740039167.326389270] [low_state_suber]: Motor state -- num: 4; q: 1.254412; dq: 0.003876; ddq: 0.000000; tau: -0.024738
[INFO] [1740039167.326425888] [low_state_suber]: Motor state -- num: 5; q: -2.782377; dq: -0.046506; ddq: 0.000000; tau: 0.047415
[INFO] [1740039167.326446673] [low_state_suber]: Motor state -- num: 6; q: -0.362283; dq: -0.003876; ddq: 0.000000; tau: 0.049477
[INFO] [1740039167.326464760] [low_state_suber]: Motor state -- num: 7; q: 1.276600; dq: 0.050382; ddq: 0.000000; tau: 0.024738
[INFO] [1740039167.326481852] [low_state_suber]: Motor state -- num: 8; q: -2.807683; dq: 0.020220; ddq: 0.000000; tau: 0.000000
[INFO] [1740039167.326498787] [low_state_suber]: Motor state -- num: 9; q: 0.344843; dq: -0.054257; ddq: 0.000000; tau: 0.024738
[INFO] [1740039167.326516362] [low_state_suber]: Motor state -- num: 10; q: 1.271907; dq: 0.007751; ddq: 0.000000; tau: -0.024738
[INFO] [1740039167.326533615] [low_state_suber]: Motor state -- num: 11; q: -2.801064; dq: -0.038418; ddq: 0.000000; tau: 0.047415
```



