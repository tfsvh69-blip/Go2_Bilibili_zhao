# simdog 工作空间包结构详解

> 最后更新：2026-08-09

## 概述

`simdog/` 是本项目的唯一 colcon 工作空间。经 `colcon list` 实际识别到
**21 个 ROS 2 包**：10 个工作空间级功能/接口包，以及 `unitree-go2-ros2/` 下的
8 个 CHAMP 包、1 个遥控包、1 个 Go2 描述包和 1 个 Go2 配置包。
这些包共同组成完整的 Unitree Go2 四足机器人仿真开发环境。

目标系统：Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11。

---

## 一、包全景图

```
simdog/src/
├── fast_gicp/                   # CUDA 加速点云配准库
├── go2_behaviors/               # 常用仿真动作与 CHAMP 控制权仲裁
├── go2_unitree_sim_bridge/      # Unitree Sport API 仿真兼容桥
├── LIO-SAM/                     # 激光-惯性紧耦合 SLAM
├── ndt_omp_ros2/                # OpenMP 加速 NDT 配准
├── ndt_relocalization/          # 基于 PCD 地图的 NDT 重定位
├── pointcloud_to_laserscan/     # 点云 ↔ 激光扫描转换
├── realsense_ros_gazebo/        # RealSense 相机 Gazebo 仿真
├── unitree_ros2_interfaces/     # 官方 v0.3.0 unitree_go/unitree_api
└── unitree-go2-ros2/            # Go2 机器人主工程（含 11 个 ROS 2 包）
    ├── champ/                   # CHAMP 四足控制器框架
    │   ├── champ_base/          #   核心驱动
    │   ├── champ_bringup/       #   基础启动文件
    │   ├── champ_config/        #   参数配置
    │   ├── champ_description/   #   机器人描述
    │   ├── champ_gazebo/        #   Gazebo 仿真集成
    │   ├── champ_msgs/          #   自定义 ROS 消息
    │   └── champ_navigation/    #   Nav2 导航集成
    ├── champ_teleop/            # 键盘/手柄遥控
    └── robots/                  # 机器人实例
        ├── descriptions/go2_description/  # Go2 URDF/Xacro 模型
        └── configs/go2_config/            # Go2 CHAMP 配置与启动
```

---

## 二、你提到的 simdog 核心包具体在哪里体现？

原有 CHAMP 与 Go2 的 11 个包
`champ, champ_base, champ_bringup, champ_config, champ_description,
champ_gazebo, champ_msgs, champ_navigation, champ_teleop, go2_config,
go2_description` 在代码中的分布如下；工作空间顶层的动作包、Unitree 兼容桥和
官方接口快照见 3.8 至 3.10 节。

### 2.1 CHAMP 框架（8 个子包）

路径：`simdog/src/unitree-go2-ros2/champ/`

| 包名 | 路径 | 在代码中的角色 |
|---|---|---|
| **champ** | `champ/champ/` | 元包，定义 CHAMP 整体依赖，核心算法在 `include/champ/` 中（Header-Only C++ 库） |
| **champ_base** | `champ/champ_base/` | 核心驱动代码（`src/`），包含四足步态生成、关节控制、里程计推算等 |
| **champ_bringup** | `champ/champ_bringup/` | 启动文件（`launch/`），负责加载控制器、传感器和状态估计节点 |
| **champ_config** | `champ/champ_config/` | 通用配置模板（`config/` 下含 gait、joints、links、autonomy 等子目录），提供默认参数 |
| **champ_description** | `champ/champ_description/` | 通用 URDF 和 RViz 配置（`urdf/`、`rviz/`），定义基础四足机器人外观 |
| **champ_gazebo** | `champ/champ_gazebo/` | Gazebo 仿真启动与脚本（`launch/`、`scripts/`、`worlds/`），处理仿真环境加载 |
| **champ_msgs** | `champ/champ_msgs/` | 自定义 ROS 2 消息定义（`msg/`），如 `JointState.msg`、`Pose.msg` 等 |
| **champ_navigation** | `champ/champ_navigation/` | Nav2 导航集成启动文件（`launch/`）和 RViz 导航配置 |

**核心代码入口：**
- 步态算法头文件：[champ/include/champ/](../simdog/src/unitree-go2-ros2/champ/champ/include/champ/)
- 基础驱动实现：[champ_base/src/](../simdog/src/unitree-go2-ros2/champ/champ_base/src/)
- 步态配置文件：[go2_config/config/gait/](../simdog/src/unitree-go2-ros2/robots/configs/go2_config/config/gait/)

### 2.2 遥控

路径：`simdog/src/unitree-go2-ros2/champ_teleop/`

| 项 | 说明 |
|---|---|
| 核心文件 | `champ_teleop.py` |
| 基于 | teleop_twist_keyboard，增加了全身姿态（roll/pitch/yaw）控制 |
| 手柄映射 | Logitech F710（见 `README.md`） |

### 2.3 Go2 机器人描述

路径：`simdog/src/unitree-go2-ros2/robots/descriptions/go2_description/`

| 子目录/文件 | 作用 |
|---|---|
| `xacro/robot.xacro` | Go2 机器人主 Xacro 文件，组装所有部件 |
| `xacro/leg.xacro` | 腿部运动学定义 |
| `xacro/velodyne.xacro` | VLP-16 3D 激光雷达挂载 |
| `xacro/laser.xacro` | 2D 激光雷达（Hokuyo）挂载 |
| `xacro/depthcam.xacro` | 深度相机（RealSense）挂载 |
| `xacro/gps.xacro` | GPS 传感器 |
| `xacro/gazebo.xacro` | Gazebo 仿真插件（IMU、力控等） |
| `xacro/transmission.xacro` | ros2_control 传动定义 |
| `xacro/materials.xacro` | 视觉材质（颜色） |
| `xacro/const.xacro` | 常量定义 |
| `urdf/go2_description.urdf` | 编译后的 URDF（由 Xacro 生成） |
| `meshes/`、`dae/` | 3D 网格模型文件 |

### 2.4 Go2 机器人配置

路径：`simdog/src/unitree-go2-ros2/robots/configs/go2_config/`

| 子目录/文件 | 作用 |
|---|---|
| `config/gait/gait.yaml` | **你当前打开的文件** — 步态参数（膝盖朝向、速度限制、支撑相时长等） |
| `config/joints/joints.yaml` | 关节名称与类型映射 |
| `config/links/links.yaml` | 连杆语义映射（base_link、hip、thigh、calf 等） |
| `config/autonomy/` | 自主导航参数 |
| `config/ros_control/` | ros2_control 控制器配置 |
| `launch/gazebo_velodyne.launch.py` | **主启动文件** — Gazebo + VLP-16 + RViz |
| `launch/gazebo.launch.py` | Gazebo 基础启动 |
| `launch/bringup.launch.py` | 纯 RViz 驱动演示 |
| `launch/slam.launch.py` | SLAM 建图启动 |
| `launch/navigate.launch.py` | 自主导航启动 |
| `worlds/` | 5 个 Gazebo 世界文件 |
| `maps/` | 示例地图（.pgm + .yaml） |

---

## 三、各包的详细功能说明

### 3.1 fast_gicp — 快速点云配准后端

**路径：** `simdog/src/fast_gicp/`

**功能：** 提供 5 种 GICP/NDT 点云配准算法，是 NDT 重定位的核心计算后端：

| 算法 | 实现方式 | 性能（参考） |
|---|---|---|
| FastGICP | 多线程 CPU GICP | ~40 FPS |
| FastVGICP | 体素化 + 多线程 GICP | ~70 FPS |
| FastVGICPCuda | CUDA 加速体素化 GICP | ~120 FPS |
| NDTCuda | CUDA 加速 D2D NDT | ~500 FPS |

**关键文件：**
- CUDA 核心：`src/fast_gicp/cuda/`（11 个 .cu 文件）
- Python 绑定：`setup.py`（`pygicp` 模块）
- 集成说明：[GO2_INTEGRATION.md](../simdog/src/fast_gicp/GO2_INTEGRATION.md)（已中文，记录 CUDA 12.8 + RTX 4060 适配）

**在本项目中：** 为 `ndt_relocalization` 提供 GPU 加速的 NDT 配准后端。

---

### 3.2 LIO-SAM — 激光-惯性 SLAM

**路径：** `simdog/src/LIO-SAM/`

**功能：** 基于因子图优化的实时紧耦合激光-惯性里程计（IROS-2020 论文实现）：
- 对 VLP-16 点云进行去畸变和特征提取
- 将 IMU 数据与激光里程计因子紧耦合
- 支持 GPS 因子融合和回环检测
- 最终输出高精度位姿估计和全局一致点云地图

**核心源文件：**
- `src/imageProjection.cpp`（23KB）— 点云去畸变和投影
- `src/imuPreintegration.cpp`（24KB）— IMU 预积分因子图
- `src/mapOptmization.cpp`（85KB）— **主因子图优化**，激光里程计、GPS、回环
- `src/featureExtraction.cpp`（10KB）— 点云特征提取

**在本项目中：** 是核心 SLAM 模块，接收 `/velodyne_points` 和 `/imu/data`，输出 `/lio_sam/mapping/odometry`。通过 `ros2 service call /lio_sam/save_map` 保存 GlobalMap.pcd 供 NDT 重定位使用。

---

### 3.3 ndt_omp_ros2 — CPU 版 NDT 配准

**路径：** `simdog/src/ndt_omp_ros2/`

**功能：** 提供经 OpenMP 多线程优化的 NDT 和 GICP 算法，作为 CUDA NDT 的 CPU 回退方案。相比 PCL 原生版本快约 10 倍。

**三种邻域搜索方法：**
- `KDTREE` — 与 PCL 原版结果完全一致
- `DIRECT7` — 推荐使用，速度与稳定性兼顾
- `DIRECT1` — 极速但可能略有波动

**在本项目中：** 当 CUDA 不可用时作为备选配准后端。

---

### 3.4 ndt_relocalization — NDT 重定位

**路径：** `simdog/src/ndt_relocalization/`

**功能：** 加载预构建的 PCD 点云地图，通过 NDT 扫描匹配实现实时重定位：
- 加载 LIO-SAM 保存的 `GlobalMap.pcd`
- 订阅当前 `/velodyne_points` 与地图进行 NDT 配准
- 发布 `/ndt_pose`、`/ndt_odom` 和动态 `map -> odom`

**核心文件：**
- `src/ndt_relocalization_node.cpp`（32KB）— 主重定位节点
- `src/map_publisher.cpp` — 预构建地图加载与发布
- `launch/ndt_localization.launch.py` — 启动文件，支持 `map_path`、`registration_backend:=cuda` 等参数

**在本项目中：** 依赖 `fast_gicp`（CUDA）或 `ndt_omp_ros2`（CPU）作为配准后端。

---

### 3.5 pointcloud_to_laserscan — 点云/激光扫描转换

**路径：** `simdog/src/pointcloud_to_laserscan/`

**功能：** 在 `sensor_msgs/msg/PointCloud2` 和 `sensor_msgs/msg/LaserScan` 之间互相转换：
- **PointCloudToLaserScanNode：** 3D 点云 → 2D 激光扫描（按高度范围截取并投影）
- **LaserScanToPointCloudNode：** 2D 激光扫描 → 3D 点云（反向转换）

**在本项目中：** 用于将 3D Velodyne 点云降维为 2D 激光扫描，供需要 2D 激光数据的算法（如 laser-based SLAM）使用。

---

### 3.6 realsense_ros_gazebo — RealSense 相机仿真

**路径：** `simdog/src/realsense_ros_gazebo/`

**功能：** 在 Gazebo 中仿真 Intel RealSense 系列相机：
- **T265** — 跟踪相机（双目鱼眼 + IMU），输出里程计
- **R200** — 深度相机（RGB + 深度 + 红外）
- **D435** — 深度相机（RGB + 深度 + 红外）

**在本项目中：** 通过 Xacro 宏嵌入 Go2 模型（`depthcam.xacro`），为 Go2 提供视觉传感器仿真。

---

### 3.7 champ_teleop — 键盘/手柄遥控

**路径：** `simdog/src/unitree-go2-ros2/champ_teleop/`

**功能：** 提供键盘和手柄两种遥控方式，控制 Go2 的运动和姿态：
- 键盘：基于 teleop_twist_keyboard
- 手柄：Logitech F710 映射（左摇杆控制移动，右摇杆控制姿态）

### 3.8 go2_behaviors — 常用 Gazebo 动作

**路径：** `simdog/src/go2_behaviors/`

**功能：** 为当前 Go2 Gazebo 模型提供打招呼、点头、伸展、趴下、挥爪和简单
舞蹈，并提供 `stand` 从保持趴下恢复：

| 命令参数 | 动作 | 结束状态 |
|---|---|---|
| `hello` | 点头后挥动右前爪 | 自动恢复 CHAMP |
| `nod` | 前后腿配合完成两次点头 | 自动恢复 CHAMP |
| `stretch` | 前后方向伸展 | 自动恢复 CHAMP |
| `wave` | 小幅抬起并横摆右前爪 | 自动恢复 CHAMP |
| `dance` | 交替摆髋和屈腿 | 自动恢复 CHAMP |
| `lie` | 分两段降低机身 | 保持趴下并暂停 CHAMP |
| `stand` | 从当前姿态恢复稳定站姿 | 恢复 CHAMP |

**控制链：**

1. 读取 `/joint_states`，使用实际关节位置作为轨迹起点。
2. 调用 `/quadruped_controller_node/set_behavior_mode` 暂停 CHAMP 关节输出。
3. 通过标准
   `/joint_group_effort_controller/follow_joint_trajectory` action 发送轨迹。
4. 使用 `/odom/ground_truth` 检查机身高度、横滚和俯仰。
5. 除 `lie` 外，动作完成后自动把控制权交还 CHAMP。

程序还会检查关节限位并拒绝并行动作。关键帧与时长集中在
`go2_behaviors/behavior_runner.py` 的 `BEHAVIORS` 中。修改后必须在独立 Gazebo
中检查机身高度与姿态，不能只看 action 是否返回成功。

该包复用 CHAMP、`ros2_control` 和标准 `FollowJointTrajectory`，没有复制控制器
源码或定义自有消息。动作仅适配当前 Gazebo 模型，不可直接下发 Unitree 真机。

### 3.9 go2_unitree_sim_bridge — Unitree 接口兼容桥

**路径：** `simdog/src/go2_unitree_sim_bridge/`

**功能：** 把真值里程计、IMU、关节、接触与 TF 转换为官方
`SportModeState`、`LowState`，并把受支持的 `/api/sport/request` 转换为
`/cmd_vel`、`/body_pose` 或 3.8 节行为服务。支持 Move、Euler、站立、坐卧、
恢复、Hello、Stretch 和 Dance1；不实现 `/lowcmd` 或真机固件策略。

### 3.10 unitree_go / unitree_api — 官方消息接口

**路径：** `simdog/src/unitree_ros2_interfaces/`

两个包固定来自 Unitree 官方 `unitree_ros2 v0.3.0`，保留 BSD-3-Clause 许可证和
提交来源。它们只定义 ROS 2 消息，不包含 Unitree 真机运动固件。

---

## 四、包之间的依赖与数据流

```
champ_teleop ── /cmd_vel ──> CHAMP ───────────────┐
                              │                  │
go2_behaviors ── 暂停 CHAMP ──┘                  ▼
       │                              FollowJointTrajectory
       └──────────────────────────────> joint_trajectory_controller
                                                    │
                                                    ▼
                                             Gazebo 12 关节
                                                    │
                   ┌────────────────────────────────┼──────────────┐
                   ▼                                ▼              ▼
          /velodyne_points                     /imu/data      /joint_states
                   │                                │              │
                   └──────────────┬─────────────────┘              │
                                  ▼                                │
                              LIO-SAM                              │
                                  │                                │
                         GlobalMap.pcd                             │
                                  │                                │
                                  ▼                                │
                      ndt_relocalization                           │
                         │              │
                         ▼              ▼
                  /ndt_pose, /ndt_odom  map -> odom

fast_gicp (CUDA) 或 ndt_omp_ros2 (CPU) ──> NDT 配准后端

go2_unitree_sim_bridge <── 状态/TF ── Gazebo
          │
          ├── 官方 Unitree 状态话题
          └── Sport API ──> /cmd_vel、/body_pose、go2_behaviors
```

---

## 五、关于项目中使用的传感器

### 5.1 Velodyne VLP-16（3D 激光雷达）

本项目的 **主传感器**。VLP-16 是一款经典的 16 线机械旋转式激光雷达：

| 参数 | 规格 |
|---|---|
| 线数 | 16 条激光通道 |
| 水平视场角 | 360°（全方位旋转） |
| 垂直视场角 | ±15°（共 30°） |
| 垂直角分辨率 | 2° |
| 最大测距 | ~100 m |
| 点云输出速率 | ~30 万点/秒（单回波） |
| 扫描频率 | 5–20 Hz（可调） |
| 仿真插件 | `gpu_ray`（GPU 加速光线追踪） |

在代码中的体现：
- 模型挂载：`go2_description/xacro/velodyne.xacro`
- Gazebo 插件：在 `gazebo.xacro` 中配置 `libgazebo_ros_velodyne_gpu_laser.so`
- 话题输出：`/velodyne_points`（`sensor_msgs/msg/PointCloud2`）
- 主配置：`go2_config/launch/gazebo_velodyne.launch.py`

### 5.2 IMU（惯性测量单元）

- Gazebo 插件仿真，输出 `/imu/data`
- LIO-SAM 使用 9 轴 IMU 数据进行点云去畸变和姿态初始化
- 参数配置在 `LIO-SAM/config/params.yaml` 中

### 5.3 RealSense D435 深度相机

- 主模型 `robot_VLP.xacro` 当前挂载 D435
- 提供 RGB 图像和深度图像
- `gazebo_velodyne.launch.py` 启动时会加载 `realsense_gazebo_plugin`

---

## 六、CHAMP 步态控制关键文件

如 [gait.yaml](../simdog/src/unitree-go2-ros2/robots/configs/go2_config/config/gait/gait.yaml)
所示，步态参数直接影响 Go2 的行走行为：

| 参数 | 含义 | 单位 |
|---|---|---|
| `knee_orientation` | 膝关节朝向（`.>>` `.><` 等） | — |
| `max_linear_velocity_x` | 前后最大速度 | m/s |
| `max_linear_velocity_y` | 横向最大速度 | m/s |
| `max_angular_velocity_z` | 旋转最大角速度 | rad/s |
| `stance_duration` | 支撑相时长 | s |
| `leg_swing_height` | 摆动相抬脚高度 | m |
| `leg_stance_height` | 支撑相压脚深度 | m |
| `robot_walking_height` | 行走时髋部离地高度 | m |
| `com_x_translation` | 质心 X 方向偏移 | m |
| `odometry_scaler` | 里程计比例因子 | — |

---

## 七、常用启动命令速查

```bash
# 环境加载
source scripts/setup_unitree_sim.bash

# 无界面 Gazebo + VLP-16 + RViz
ros2 launch go2_config gazebo_velodyne.launch.py gui:=false rviz:=true

# 启动 LIO-SAM 建图
ros2 launch lio_sam lidar.launch.py rviz:=true

# 键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 打招呼（其他参数：nod/stretch/wave/dance/lie）
ros2 run go2_behaviors go2_behavior hello

# 从保持趴下恢复
ros2 run go2_behaviors go2_behavior stand

# 保存 LIO-SAM 地图（LIO-SAM 必须仍在运行）
bash simdog/save_Map.sh

# NDT 重定位（需先有 GlobalMap.pcd）
ros2 launch ndt_relocalization ndt_localization.launch.py \
    map_path:=$HOME/go2_maps/latest/GlobalMap.pcd \
    registration_backend:=cuda gpu_device_id:=0 use_rviz:=true

# 验证关键话题
ros2 topic hz /velodyne_points
ros2 topic hz /imu/data
ros2 topic echo --once /odom
ros2 topic echo --once /ndt_pose
ros2 control list_controllers
ros2 topic hz /sportmodestate
ros2 topic hz /lowstate
```

---

## 八、参考链接

- [CHAMP 官方仓库](https://github.com/chvmp/champ)
- [LIO-SAM 论文 (IROS-2020)](../simdog/src/LIO-SAM/config/doc/paper.pdf)
- [fast_gicp 官方仓库](https://github.com/SMRT-AIST/fast_gicp)
- [Unitree SDK2 Go2 SportClient](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/sport/sport_client.hpp)
- [Unitree ROS 2 v0.3.0](https://github.com/unitreerobotics/unitree_ros2/tree/v0.3.0)
- [ROS 2 joint_trajectory_controller](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
