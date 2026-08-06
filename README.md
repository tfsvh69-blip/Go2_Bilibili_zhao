# Go2 ROS 2 完整四足仿真工作区

本项目面向 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Classic 11，仅维护
`simdog/` 完整四足仿真工作空间。机器人通过 CHAMP 生成真实四足步态并由
`ros2_control` 驱动 12 个腿部关节，不使用焊死腿关节或 planar-move 滑行的
简化模型。

主要能力：

- Unitree Go2 URDF/xacro、Gazebo 物理仿真和 CHAMP 四足步态。
- Velodyne VLP-16、IMU、RealSense D435 和 GPS 仿真。
- LIO-SAM 激光惯性建图与 PCD 地图保存。
- 基于 `fast_gicp` CUDA D2D-NDT 或 OpenMP NDT 的全局重定位。

## 当前硬件基线

当前电脑已实际检测为：

```text
GPU：NVIDIA GeForce RTX 4060 Laptop GPU
显存：8188 MiB
计算能力：8.9
驱动：595.84
CUDA：12.8
GPU NDT 架构：sm_89
```

本机不是 RTX 5070。只有 NDT 点云配准使用 CUDA 计算；Gazebo/RViz2 使用 GPU
进行 OpenGL 渲染，Gazebo 物理、CHAMP、LIO-SAM 的 GTSAM 图优化和大部分 PCL
预处理仍主要运行在 CPU 上。

## 目录结构

```text
.
├── simdog/                         # 唯一 ROS 2 colcon 工作空间
│   ├── src/
│   │   ├── unitree-go2-ros2/       # Go2、CHAMP、ros2_control、Gazebo
│   │   ├── LIO-SAM/                # 激光惯性建图
│   │   ├── ndt_relocalization/     # NDT 重定位节点
│   │   ├── fast_gicp/              # CUDA 点云配准
│   │   ├── ndt_omp_ros2/           # OpenMP CPU 配准
│   │   ├── realsense_ros_gazebo/   # RealSense 仿真
│   │   └── pointcloud_to_laserscan/
│   ├── start.sh                    # 多终端启动入口
│   └── save_Map.sh                 # LIO-SAM 地图保存
├── scripts/
│   ├── install_dependencies.sh
│   ├── install_gpu_dependencies.sh
│   ├── build_workspaces.sh         # 当前只构建 simdog
│   ├── setup_simdog.bash
│   └── verify_gpu_runtime.sh
├── AGENTS.md                       # 协作规则
├── CLAUDE.md                       # 与 AGENTS.md 完全一致
├── GPU_TESTING.md                  # GPU 验证与维护
└── PROJECT_MEMORY.md               # 当前状态与阶段记录
```

## 首次配置

从项目根目录执行：

```bash
cd /home/hao/ROS/Go2_Bilibili_zhao-main
bash scripts/install_dependencies.sh
bash scripts/install_gpu_dependencies.sh
bash scripts/build_workspaces.sh
```

基础依赖脚本会安装 Gazebo、ROS 控制与传感器包、PCL、Nav2、SLAM Toolbox、
LIO-SAM 依赖和 GTSAM 4.1。GPU 脚本从 NVIDIA 官方软件源安装 CUDA 12.8
编译器和最小运行库，不替换现有显卡驱动。

构建脚本检测到 CUDA 12.8 时，会为 RTX 4060 构建 `sm_89` GPU NDT；未检测到
CUDA 时构建 OpenMP CPU 回退版本。

## 加载工作空间

每个新终端从项目根目录执行：

```bash
source scripts/setup_simdog.bash
```

脚本默认设置 `CUDA_VISIBLE_DEVICES=0`，并在双显卡环境中启用 NVIDIA PRIME
Render Offload。可通过 `GO2_GPU_DEVICE` 选择物理 GPU，通过
`GO2_FORCE_NVIDIA_RENDERING=0` 关闭强制 NVIDIA OpenGL。

## 启动完整四足仿真

以下流程是完整四足 Go2 的标准启动方式。先确认没有遗留的同一套 Gazebo、LIO-SAM
或 NDT 进程；每个终端都必须从项目根目录加载同一个 `simdog` 工作空间。

### 场景一：首次运行或采集新地图

此模式由 LIO-SAM 独占发布 `map -> odom`，不要同时启动 NDT。

终端一：启动 Gazebo、完整四足控制器、传感器和基础 RViz2。

```bash
cd /home/hao/ROS/Go2_Bilibili_zhao-main
source scripts/setup_simdog.bash
ros2 launch go2_config gazebo_velodyne.launch.py gui:=false rviz:=true
```

终端二：启动 LIO-SAM 建图和建图 RViz2。

```bash
cd /home/hao/ROS/Go2_Bilibili_zhao-main
source scripts/setup_simdog.bash
ros2 launch lio_sam lidar.launch.py rviz:=true
```

终端三：启动键盘遥控。按 `i` 前进、`,` 后退、`j`/`l` 转向、`k` 停止；先按
`k` 再关闭节点。

```bash
cd /home/hao/ROS/Go2_Bilibili_zhao-main
source scripts/setup_simdog.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

完成巡视后，在**仍保持终端二 LIO-SAM 运行**的情况下另开终端四保存地图：

```bash
cd /home/hao/ROS/Go2_Bilibili_zhao-main
bash simdog/save_Map.sh
test -s "$HOME/go2_maps/latest/GlobalMap.pcd" && echo "地图保存成功"
```

默认地图文件是 `~/go2_maps/latest/GlobalMap.pcd`。若要使用其他目录和分辨率：

```bash
bash simdog/save_Map.sh "$HOME/go2_maps/warehouse" 0.2
```

建图完成后，依次在键盘遥控、LIO-SAM、Gazebo 终端按 `Ctrl+C` 停止。不要依赖
关闭 LIO-SAM 时自动保存地图。

### 场景二：使用已有 PCD 地图重定位

此模式由 NDT 独占发布 `map -> odom`；LIO-SAM 仍负责提供惯导和点云预处理，但必须
显式关闭它的全局 TF 发布。

终端一：启动 Gazebo、完整四足控制器和传感器。

```bash
cd /home/hao/ROS/Go2_Bilibili_zhao-main
source scripts/setup_simdog.bash
ros2 launch go2_config gazebo_velodyne.launch.py gui:=false rviz:=true
```

终端二：启动 LIO-SAM，但让出 `map -> odom`。

```bash
cd /home/hao/ROS/Go2_Bilibili_zhao-main
source scripts/setup_simdog.bash
ros2 launch lio_sam lidar.launch.py rviz:=true publish_map_to_odom:=false
```

终端三：启动 CUDA NDT 重定位。

```bash
cd /home/hao/ROS/Go2_Bilibili_zhao-main
source scripts/setup_simdog.bash
ros2 launch ndt_relocalization ndt_localization.launch.py \
    map_path:=$HOME/go2_maps/latest/GlobalMap.pcd \
    registration_backend:=cuda gpu_device_id:=0 use_rviz:=true
```

终端四：按需启动键盘遥控。

```bash
cd /home/hao/ROS/Go2_Bilibili_zhao-main
source scripts/setup_simdog.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

NDT 会自动以最近一次成功配准结果作为下一帧初值。若初始位置相差较大，在 NDT 的
RViz2 中使用 “2D Pose Estimate” 发布 `/initialpose`，再检查 `/ndt_pose` 是否
持续输出。

### 一键启动

桌面环境也可一次启动：

```bash
bash simdog/start.sh
```

该脚本会启动 Gazebo、LIO-SAM 和键盘遥控；如果存在
`~/go2_maps/latest/GlobalMap.pcd`，还会以定位模式启动 CUDA NDT 并自动让 LIO-SAM
让出 `map -> odom`。没有地图时会以建图模式启动并跳过 NDT。

### 启动后快速检查

在任意已加载工作空间的终端执行：

```bash
ros2 control list_controllers
ros2 topic hz /velodyne_points
ros2 topic hz /imu/data
ros2 topic echo --once /odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link velodyne
```

建图模式还应能查询 `map -> base_link`；重定位模式则同时检查 NDT 输出：

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo --once /ndt_pose
```

Gazebo 单独运行时还没有 `map` 坐标系；若基础 RViz2 出现 “Fixed Frame [map]
does not exist”，将其 Fixed Frame 临时改为 `odom`，或启动 LIO-SAM/NDT 后再改回
`map`。

## 建图与地图保存

建图保存的完整顺序见上方“场景一”。以下命令必须在 LIO-SAM 尚未关闭时执行：

```bash
bash simdog/save_Map.sh
```

默认保存到 `~/go2_maps/latest`。也可以指定目录和分辨率：

```bash
bash simdog/save_Map.sh ~/go2_maps/warehouse 0.2
```

地图保存依赖正在运行的 LIO-SAM `/lio_sam/save_map` 服务。

## NDT 重定位

重定位的完整终端顺序见上方“场景二”。单独启动 NDT 的命令如下：

```bash
ros2 launch ndt_relocalization ndt_localization.launch.py \
    map_path:=$HOME/go2_maps/latest/GlobalMap.pcd \
    registration_backend:=cuda gpu_device_id:=0 use_rviz:=true
```

在 RViz2 中通过 “2D Pose Estimate” 发布 `/initialpose`。节点读取
`/velodyne_points`，发布 `/global_map`、`/ndt_pose`、`/ndt_odom` 和
`map -> odom` TF。

## TF 所有权

主链必须保持为：

```text
map -> odom -> base_footprint -> base_link -> 关节与传感器
```

- 建图时由 LIO-SAM 发布动态 `map -> odom`；重定位时关闭该发布，由 NDT
  唯一发布。
- `footprint_to_odom_ekf` 唯一发布动态 `odom -> base_footprint` 和 `/odom`。
- `base_to_footprint_ekf` 根据 CHAMP 状态估计发布
  `base_footprint -> base_link`。
- `robot_state_publisher` 发布 `base_link` 以下的关节和实际传感器 TF，包括
  `base_link -> velodyne_base_link -> velodyne`。
- LIO-SAM 内部里程计使用 `/lio_sam/imu/odometry`，不会占用 `/odom`，也不会
  创建 `lidar_link`。

不得添加静态 `odom -> base_footprint`，因为机器人运动时这条变换必须动态更新。

强制使用 CPU：

```bash
ros2 launch ndt_relocalization ndt_localization.launch.py \
    map_path:=$HOME/go2_maps/latest/GlobalMap.pcd \
    registration_backend:=omp
```

## 验证

GPU NDT 端到端检查：

```bash
bash scripts/verify_gpu_runtime.sh
```

主仿真启动后检查：

```bash
ros2 topic hz /velodyne_points
ros2 topic hz /imu/data
ros2 topic echo --once /odom/local
ros2 topic echo --once /odom
ros2 topic hz /joint_states
ros2 control list_controllers
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link velodyne
```

GPU NDT 运行时可观察：

```bash
nvidia-smi dmon -s pucm
ros2 topic echo --once /ndt_pose
```

完整判定标准和 CPU 回退方式见 [GPU_TESTING.md](GPU_TESTING.md)。

## 已知限制

- Gazebo Classic GUI 可能受 NVIDIA 驱动与 OGRE 兼容性影响，默认推荐
  `gui:=false` 配合 RViz2。
- NVIDIA OpenGL 异常时，先设置 `GO2_FORCE_NVIDIA_RENDERING=0`；仍有问题再
  按需使用 `LIBGL_ALWAYS_SOFTWARE=1`。
- LIO-SAM 当前关闭回环检测，正式地图应在目标场景重新采集和评估。
- Nav2 和 SLAM Toolbox 依赖已经安装，但 Go2 的完整自主导航参数尚未完成调优。
- NDT 正式使用前必须准备有效 `GlobalMap.pcd` 并设置合理初始位姿。
- 当前准确状态、历史验证结果和遗留问题以
  [PROJECT_MEMORY.md](PROJECT_MEMORY.md) 为准。
