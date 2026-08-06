### 2.1.1 高层状态获取

#### 1.消息类型

高层状态为机器人的速度、位置、足端位置等与运动相关的状态。高层状态的获取可通过订阅`lf/sportmodestate`或`sportmodestate`topic 实现，其中`lf`表示低频率。

通过调用如下指令可以获取高层状态对应的消息接口类型：

```
ros2 topic type /sportmodestate
```

或

```
ros2 topic type /lf/sportmodestate
```

其输出结果为：`unitree_go/msg/SportModeState`。

#### 2.数据结构

通过如下指令，可以获取高层状态消息接口的数据结构：

```
ros2 interface show unitree_go/msg/SportModeState
```

其输出结果如下：

```
TimeSpec stamp //时间戳
    int32 sec //秒
    uint32 nanosec //纳秒 
uint32 error_code //错误代码
IMUState imu_state //IMU状态
    float32[4] quaternion //四元数
    float32[3] gyroscope //角速度
    float32[3] accelerometer //加速度
    float32[3] rpy //欧拉角
    int8 temperature //传感器温度
uint8 mode //运动模式
/*
运动模式
0. idle, default stand 空闲，默认站立
1. balanceStand 平衡站立
2. pose 姿态调整
3. locomotion 运动行走
4. reserve 预留
5. lieDown 趴下
6. jointLock 关节锁定
7. damping 阻尼模式
8. recoveryStand 恢复站立
9. reserve 预留
10. sit 坐下
11. frontFlip 前空翻
12. frontJump 前跳
13. frontPounc 前扑
*/

float32 progress //是否动作执行状态：0. dance false; 1. dance true
uint8 gait_type //步态类型
/*
步态类型
0.idle 空闲  
1.trot 小跑
2.run  奔跑
3.climb stair 爬楼梯
4.forwardDownStair 下楼梯（向前）
9.adjust 调整
*/
float32 foot_raise_height //抬腿高度
float32[3] position //当前位置
float32 body_height //机体高度
float32[3] velocity //线速度
float32 yaw_speed //角速度
float32[4] range_obstacle //障碍物范围 
int16[4] foot_force //足端力数值
float32[12] foot_position_body //足端相对于机体的位置
float32[12] foot_speed_body //足端相对于机体的速度
```

接口定义的源文件为：`~/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go/msg/SportModeState.msg`。

#### 3.内置例程

读取高层状态的完整例程位于`~/unitree_ros2/example/src/read_motion_state.cpp`。

编译完例程后，在终端中进入`unitree_ros2/example`目录并运行如下指令：

```
./install/unitree_ros2_example/bin/read_motion_state
```

终端会输出类似如下结果：

```
[INFO] [1740037460.552882258] [motion_state_suber]: Gait state -- gait type: 0; raise height: 0.090000
[INFO] [1740037460.552949931] [motion_state_suber]: Position -- x: 0.643311; y: 0.890160; z: 0.305879; body height: 0.320000
[INFO] [1740037460.552967169] [motion_state_suber]: Velocity -- vx: -0.000991; vy: -0.004060; vz: -0.013185; yaw: -0.020240
[INFO] [1740037460.552982799] [motion_state_suber]: Foot position and velcity relative to body -- num: 0; x: 0.194726; y: -0.139744; z: -0.303536, vx: -0.010517; vy: 0.012931; vz: -0.002810
[INFO] [1740037460.553008931] [motion_state_suber]: Foot position and velcity relative to body -- num: 1; x: 0.192606; y: 0.136600; z: -0.306353, vx: -0.007322; vy: 0.010824; vz: 0.010986
[INFO] [1740037460.553044563] [motion_state_suber]: Foot position and velcity relative to body -- num: 2; x: -0.190461; y: -0.137815; z: -0.307377, vx: 0.017941; vy: -0.011854; vz: -0.000781
[INFO] [1740037460.553082997] [motion_state_suber]: Foot position and velcity relative to body -- num: 3; x: -0.191764; y: 0.139800; z: -0.307487, vx: 0.008105; vy: -0.025010; vz: -0.005459
```



