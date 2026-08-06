# simdog 工作空间包结构详解

> 最后更新：2026-08-06

## 概述

`simdog/` 是本项目的唯一 colcon 工作空间，其 `src/` 目录下共有 **16 个 ROS 2 包**（10 个独立功能包和 6 个 CHAMP 子包），共同组成完整的 Unitree Go2 四足机器人仿真开发环境。

目标系统：Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11。

---

## 一、包全景图

```
simdog/src/
├── fast_gicp/                   # CUDA 加速点云配准库
├── LIO-SAM/                     # 激光-惯性紧耦合 SLAM
├── ndt_omp_ros2/                # OpenMP 加速 NDT 配准
├── ndt_relocalization/          # 基于 PCD 地图的 NDT 重定位
├── pointcloud_to_laserscan/     # 点云 ↔ 激光扫描转换
├── realsense_ros_gazebo/        # RealSense 相机 Gazebo 仿真
└── unitree-go2-ros2/            # Go2 机器人主工程（含 9 个子包）
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

你列出的 12 个包 `champ, champ_base, champ_bringup, champ_config, champ_description, champ_gazebo, champ_msgs, champ_navigation, champ_teleop, go2_config, go2_description` 在代码中的分布如下：

### 2.1 CHAMP 框架（7 个子包）

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
- 步态算法算法头文件：[champ/include/champ/](simdog/src/unitree-go2-ros2/champ/champ/include/champ/)
- 基础驱动实现：[champ_base/src/](simdog/src/unitree-go2-ros2/champ/champ_base/src/)
- 步态配置文件：[champ_config/config/gait/](simdog/src/unitree-go2-ros2/champ/champ_config/config/gait/)

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
- 集成说明：[GO2_INTEGRATION.md](simdog/src/fast_gicp/GO2_INTEGRATION.md)（已中文，记录 CUDA 12.8 + RTX 4060 适配）

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
- 输出重定位后的里程计 `/odom/local`

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

---

## 四、包之间的依赖与数据流

```
                          ┌──────────────────────────┐
                          │   Gazebo Classic 11       │
                          │   (物理仿真引擎)            │
                          └──────┬───────────────────┘
                                 │
              ┌──────────────────┼──────────────────┐
              ▼                  ▼                  ▼
     /velodyne_points     /imu/data          /joint_states
     (3D 激光点云)         (IMU 惯导)         (关节角度)
              │                  │                  │
              ▼                  ▼                  ▼
     ┌────────────┐    ┌─────────────┐    ┌──────────────┐
     │  LIO-SAM    │    │ champ_base  │    │ champ_gazebo │
     │ (SLAM 建图)  │    │ (步态控制)   │    │ (仿真桥接)    │
     └─────┬──────┘    └──────┬──────┘    └──────────────┘
           │                  │
           ▼                  ▼
    GlobalMap.pcd      /cmd_vel (速度指令)
           │                  ▲
           ▼                  │
  ┌─────────────────┐  ┌──────┴──────┐
  │ndt_relocalization│  │champ_teleop │
  │  (NDT 重定位)     │  │ (遥控节点)   │
  └────────┬────────┘  └─────────────┘
           │
           ▼
    /odom/local
    (重定位里程计)
           │
           ├── 使用 fast_gicp (CUDA) 作为配准后端
           └── 或回退到 ndt_omp_ros2 (CPU OpenMP)
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

### 5.3 RealSense 深度相机（可选）

- 通过 `depthcam.xacro` 可选挂载
- 提供 RGB 图像和深度图像
- 当前未在主启动流程中启用

---

## 六、CHAMP 步态控制关键文件

如你当前打开的 [gait.yaml](simdog/src/unitree-go2-ros2/robots/configs/go2_config/config/gait/gait.yaml)，步态参数直接影响 Go2 的行走行为：

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
source scripts/setup_simdog.bash

# 无界面 Gazebo + VLP-16 + RViz
ros2 launch go2_config gazebo_velodyne.launch.py gui:=false rviz:=true

# 启动 LIO-SAM 建图
ros2 launch lio_sam lidar.launch.py rviz:=true

# 键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 保存 LIO-SAM 地图
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap

# NDT 重定位（需先有 GlobalMap.pcd）
ros2 launch ndt_relocalization ndt_localization.launch.py \
    map_path:=$HOME/go2_maps/latest/GlobalMap.pcd \
    registration_backend:=cuda gpu_device_id:=0 use_rviz:=true

# 验证关键话题
ros2 topic hz /velodyne_points
ros2 topic hz /imu/data
ros2 topic echo --once /odom/local
```

---

## 八、参考链接

- [CHAMP 官方仓库](https://github.com/chvmp/champ)
- [LIO-SAM 论文 (IROS-2020)](./simdog/src/LIO-SAM/config/doc/paper.pdf)
- [fast_gicp 官方仓库](https://github.com/SMRT-AIST/fast_gicp)
- [宇树科技 Unitree Robotics](https://github.com/unitreerobotics/unitree_ros)
