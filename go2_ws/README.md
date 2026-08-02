# Go2 Gazebo 仿真平台 (ROS2 Humble + Gazebo Classic 11)

面向**导航 / SLAM / 视觉**上层应用开发的 Unitree Go2 仿真平台。
采用 **planar-move 移动平台**路线:机器人作为刚体按 `/cmd_vel` 在地面滑行(伪行走),
发布 odom / tf / IMU / 激光 / 相机,可直接对接 slam_toolbox、nav2、RTAB-Map 等。

> 若日后需要**真实腿部步态**(强化学习/运动控制),再另起 CHAMP 或 unitree_guide2 方案,
> 与本平台的描述包 `go2_description` 可复用。

## 目录结构

```
go2_ws/
└── src/
    ├── go2_description/          # Go2 URDF + 网格(官方模型,ROS2 化)
    │   ├── urdf/go2_description.urdf
    │   ├── dae/ meshes/          # 网格(package://go2_description/... 引用)
    │   ├── launch/display.launch.py   # 纯 RViz 看模型
    │   └── rviz/display.rviz
    └── go2_gazebo/              # Gazebo 仿真
        ├── scripts/gen_planar_urdf.py # 由描述URDF生成刚体仿真URDF的生成器
        ├── urdf/go2_gazebo.urdf       # 生成产物:12腿关节焊死 + planar_move + IMU + 雷达 + 相机
        ├── worlds/go2.world           # 空世界(sun+ground,可加障碍物)
        ├── launch/spawn.launch.py     # 启动 gazebo + spawn + rsp (+可选rviz)
        └── rviz/go2_sim.rviz
```

## 编译

```bash
cd ~/my/ROS/Go2/go2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 运行

### 1) 只看模型 (RViz)
```bash
ros2 launch go2_description display.launch.py        # 带关节滑块GUI
```

### 2) Gazebo 仿真(默认带 Gazebo 界面)
```bash
ros2 launch go2_gazebo spawn.launch.py               # 只开 Gazebo
ros2 launch go2_gazebo spawn.launch.py rviz:=true    # 同时开 RViz(看雷达/相机/tf)
ros2 launch go2_gazebo spawn.launch.py gui:=false    # 无界面(仅物理,雷达/IMU可用,相机不渲染)
```

### 3) 遥控走一圈
```bash
# 键盘遥控(话题即 /cmd_vel)
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 或直接发一条前进指令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}}" -r 20
```

## 已发布的话题/坐标系

| 话题 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | **输入**:线速度 x/y + 角速度 z |
| `/odom` | nav_msgs/Odometry | 里程计(planar_move 输出) |
| `/tf` | odom → base | 里程计坐标变换 |
| `/scan` | sensor_msgs/LaserScan | 顶部 2D 激光,360°,10Hz,frame=laser_link |
| `/imu` | sensor_msgs/Imu | 机身 IMU,200Hz |
| `/camera/image_raw` | sensor_msgs/Image | 前置 RGB(**需 Gazebo GUI/GPU 渲染才发布**) |
| `/camera/depth/image_raw`,`/camera/points` | 深度/点云 | 同上,需渲染 |

## 验证状态(已在无界面环境跑通)
- ✅ 刚体站立稳定(z≈0.448,不塌不穿地)
- ✅ `/cmd_vel` 前进/旋转,全程保持直立(无摩擦足端修复了翻倒问题)
- ✅ `/odom` + `odom→base` tf 正确
- ✅ `/scan` 激光 10Hz、`/imu` 200Hz
- ⚠️ 相机需要 Gazebo GUI 或 GPU 渲染才会发布话题(headless 不渲染,配置本身正确)

## 下一步(建议顺序)
1. **teleop + 建图**:`slam_toolbox` 订阅 `/scan` + `/odom` → 实时建图
2. **导航**:`nav2` + 已建地图 → 点目标自主导航
3. **视觉**:开 GUI 后用 `/camera/image_raw` 做目标识别 / `/camera/points` 做 RTAB-Map
4. **对接教程的 Twist 桥接**:把 `unitree_go`/`unitree_api` 的 Twist 桥接节点指向本仿真的 `/cmd_vel`,
   仿真与真机共用同一套上层代码

## 关键设计说明
- **为什么焊死 12 个腿关节**:planar-move 路线不模拟腿部动力学,把整机做成刚体最稳,
  站立/滑行都不会散架。腿关节的可动版本保留在 `go2_description`,供未来步态方案使用。
- **为什么足端无摩擦**:planar_move 在 base 处强加速度,足端若有摩擦会被"钉住"产生翻转力矩
  导致机器人脸朝地翻倒;设 mu=0 让身体自由滑行,cmd_vel=0 时插件锁定速度也不会漂移。
- **改传感器/位姿**:编辑 `scripts/gen_planar_urdf.py` 后重新生成:
  ```bash
  python3 src/go2_gazebo/scripts/gen_planar_urdf.py \
      src/go2_description/urdf/go2_description.urdf \
      src/go2_gazebo/urdf/go2_gazebo.urdf
  colcon build --packages-select go2_gazebo
  ```
