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
- Unitree `unitree_go`/`unitree_api` 官方消息与 Sport API 仿真兼容桥。

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
│   │   ├── go2_behaviors/          # 打招呼、点头、伸展等仿真动作
│   │   ├── go2_unitree_sim_bridge/ # Unitree Sport API 仿真兼容桥
│   │   ├── unitree_ros2_interfaces/ # 固定的官方 v0.3.0 消息接口
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
│   ├── setup_unitree_sim.bash      # CycloneDDS/lo/Domain 1
│   ├── setup_unitree_real.bash     # CycloneDDS/真机网卡/Domain 0
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

需要使用 Unitree 接口时，仿真终端统一改为：

```bash
source scripts/setup_unitree_sim.bash
```

该入口使用 CycloneDDS、回环接口 `lo` 和默认 `ROS_DOMAIN_ID=1`。已有
`ROS_DOMAIN_ID` 或 `GO2_UNITREE_SIM_DOMAIN_ID` 可覆盖默认值。真机只配置通信
环境，不启动仿真桥；必须显式传入已连接且处于 UP 状态的网卡：

```bash
source scripts/setup_unitree_real.bash enp3s0
```

真机入口默认 Domain 0，可用 `GO2_UNITREE_REAL_DOMAIN_ID` 覆盖。本阶段未使用
真机硬件验证。

## 启动完整四足仿真

以下流程是完整四足 Go2 的标准启动方式。先确认没有遗留的同一套 Gazebo、LIO-SAM
或 NDT 进程；每个终端都必须从项目根目录加载同一个 `simdog` 工作空间。

### 场景一：首次运行或采集新地图

此模式由 LIO-SAM 独占发布 `map -> odom`，不要同时启动 NDT。

终端一：启动 Gazebo、完整四足控制器、传感器和基础 RViz2。

```bash
cd /home/hao/ROS/Go2_Bilibili_zhao-main
source scripts/setup_simdog.bash
ros2 launch go2_config gazebo_velodyne.launch.py gui:=true rviz:=true
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

## 仿真动作

只需启动终端一的完整四足 Gazebo，不依赖 LIO-SAM、NDT 或真机。另开一个已加载
工作空间的终端，每次执行一个动作：

```bash
ros2 run go2_behaviors go2_behavior hello    # 打招呼
ros2 run go2_behaviors go2_behavior nod      # 点头
ros2 run go2_behaviors go2_behavior stretch  # 伸展
ros2 run go2_behaviors go2_behavior lie      # 趴下并保持
ros2 run go2_behaviors go2_behavior wave     # 挥爪
ros2 run go2_behaviors go2_behavior dance    # 简单舞蹈
```

`lie` 会保持趴下并暂停 CHAMP，恢复时必须执行：

```bash
ros2 run go2_behaviors go2_behavior stand
```

动作开始前会通过
`/quadruped_controller_node/set_behavior_mode` 暂停 CHAMP 的关节指令，再复用
`joint_trajectory_controller` 标准
`/joint_group_effort_controller/follow_joint_trajectory` 动作接口；动作和步态
不会同时抢控制器。执行动作时不要同时遥控。上述轨迹只适配当前 Gazebo 模型，
不等同于真机固件中的 Unitree Sport API 动作，不能直接用于真机。

实现与开源复用说明见
[`go2_behaviors/README.md`](simdog/src/go2_behaviors/README.md)。

## Unitree ROS 2 兼容桥

主 Gazebo 启动文件默认同时启动 `go2_behavior_server` 和
`go2_unitree_sim_bridge`。无需兼容层时可显式关闭：

```bash
ros2 launch go2_config gazebo_velodyne.launch.py unitree_bridge:=false
```

桥接发布官方类型状态话题：

| 话题 | 类型 | 默认频率 |
|---|---|---:|
| `/sportmodestate` | `unitree_go/msg/SportModeState` | 50 Hz |
| `/lf/sportmodestate` | `unitree_go/msg/SportModeState` | 10 Hz |
| `/lowstate` | `unitree_go/msg/LowState` | 100 Hz |
| `/lf/lowstate` | `unitree_go/msg/LowState` | 10 Hz |
| `/api/sport/response` | `unitree_api/msg/Response` | 按请求 |

订阅 `/api/sport/request`，支持 `BalanceStand(1002)`、`StopMove(1003)`、
`StandUp(1004)`、`StandDown(1005)`、`RecoveryStand(1006)`、`Euler(1007)`、
`Move(1008)`、`Sit(1009)`、`RiseSit(1010)`、`Hello(1016)`、
`Stretch(1017)` 和 `Dance1(1022)`。`Move` 限制为
`x=±0.3 m/s`、`y=±0.25 m/s`、`yaw=±0.5 rad/s`，持续发布 `/cmd_vel`
直到 `StopMove`；这段时间禁止同时运行键盘遥控。

模拟器错误码为 `-32001` 参数错误、`-32002` 不支持、`-32003` 忙和
`-32004` 下游失败。响应复制请求 `identity`，`noreply=true` 时不响应。
更完整的字段映射、服务入口和边界见
[`go2_unitree_sim_bridge/README.md`](simdog/src/go2_unitree_sim_bridge/README.md)。

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
ros2 topic echo --once /odom
ros2 topic hz /joint_states
ros2 control list_controllers
ros2 topic hz /sportmodestate
ros2 topic hz /lowstate
ros2 topic type /api/sport/request
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
- Unitree 兼容桥只保证所列消息、话题和请求的接口级兼容，不模拟 `/lowcmd`、
  BMS、无线遥控、真实足底力、障碍距离或真机固件的平衡与安全策略。
- 仿真动作轨迹不可下发真机；真机环境脚本只完成 DDS 配置，尚无硬件验证结果。
- 当前准确状态、历史验证结果和遗留问题以
  [PROJECT_MEMORY.md](PROJECT_MEMORY.md) 为准。
