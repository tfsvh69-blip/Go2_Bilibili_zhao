# Go2 ROS 2 完整四足仿真工作区

本项目面向 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Classic 11，仅维护
`simdog/` 完整四足仿真工作空间。机器人通过 CHAMP 生成真实四足步态并由
`ros2_control` 驱动 12 个腿部关节，不使用焊死腿关节或 planar-move 滑行的
简化模型。

初次接触 ROS 2、RViz、建图或导航时，先阅读
[《Go2 导航、建图与 RViz 初学者图解手册》](文档/Go2导航建图与RViz初学者图解手册.md)。其中用本项目真实截图解释地图颜色、黄色/紫色协方差、TF、AMCL、Nav2 状态、代价地图和速度安全链。

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
│   │   ├── go2_navigation/         # 自主导航包（同源地图包、Nav2、安全链）
│   │   ├── lidar_localization_ros2/ # NDT/GICP 实验定位库
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
│   ├── setup_unitree_sim.bash      # CycloneDDS/lo/Domain 0
│   ├── setup_unitree_real.bash     # CycloneDDS/真机网卡/Domain 0
│   └── verify_gpu_runtime.sh
├── AGENTS.md                       # 协作规则
├── CLAUDE.md                       # 与 AGENTS.md 完全一致
├── GPU_TESTING.md                  # GPU 验证与维护
├── PROJECT_MEMORY.md               # 当前状态与阶段记录
└── 文档/Go2导航建图与RViz初学者图解手册.md
                                    # 初学者术语、RViz 图例与故障现象索引
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

该入口使用 CycloneDDS、回环接口 `lo` 和固定的 `ROS_DOMAIN_ID=0`，不会继承
终端中遗留的域。隔离测试只能通过 `GO2_UNITREE_SIM_DOMAIN_ID=<id>` 显式覆盖。
真机只配置通信环境，不启动仿真桥；必须显式传入已连接且处于 UP 状态的网卡：

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
ros2 launch go2_config gazebo_velodyne.launch.py rviz:=true
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

## 自主导航

统一入口默认运行 `online_slam`：Slam Toolbox 一边定位一边扩展 `/map`。固定图模式
默认运行 AMCL；`lidar_ndt` 和 `ndt_cuda` 保留为实验档。NDT 实验需要**同源地图包**：

| 文件 | 用途 |
|---|---|
| `GlobalMap.pcd` | NDT/GICP 定位地图 |
| `map.yaml` / `map.pgm` | Nav2 全局代价地图 |
| `map_bundle.yaml` | SHA-256 完整性清单，缺失或不匹配时拒绝启动 |

```bash
# 建图并生成同源地图包
ros2 launch go2_navigation mapping.launch.xml
bash simdog/src/go2_navigation/scripts/save_map.sh ~/go2_maps/latest

# 只由已有 GlobalMap.pcd 重建地图包（支持裁剪到导航区域）
ros2 run go2_navigation build_map_bundle --map-dir ~/go2_maps/latest \
    --x-min -2 --x-max 8 --y-min -4 --y-max 4

# 默认：在线 SLAM + 导航，从空图开始；map_session 可传已有会话目录续建。
ros2 launch go2_navigation simulation_navigation.launch.xml \
    navigation_mode:=online_slam map_session:=new \
    controller_profile:=forward_rpp rviz:=true

# 固定地图 + AMCL（静态 /map 不会继续扩展）
ros2 launch go2_navigation simulation_navigation.launch.xml \
    navigation_mode:=static_map map_dir:=$HOME/go2_maps/online/latest \
    localization:=amcl controller_profile:=forward_rpp rviz:=true \
    tuning_gui:=true

# 健康检查
ros2 run go2_navigation health_check --mode online_slam --localization amcl
```

一次只运行一套 `simulation_navigation.launch.xml`。在线 RViz 的主面板是
`Slam Toolbox`，用于观察、暂停与保存在线图；`Navigation 2` 面板用于发送、取消目标。
固定图 RViz 的 `Static Map` 显示 `/map`，`AMCL Pose` 显示 `/amcl_pose`。两种配置都将
原始点云和两张代价图默认关闭，排障时再打开。固定图启动后先在白色自由区点击
`2D Pose Estimate`，待 `Localization: active` 后再点 `Nav2 Goal`。不要将 Fixed Frame
改成 `odom`。

`AMCL Pose` 的紫色椭圆是 x/y 协方差，黄色扇形是 yaw 协方差，不是雷达视野。扇形
无限拉长表示 AMCL 航向失信；导航安全监督会锁速并提示当前标准差，防止错误的
`map -> odom` 继续驱动机器人。重新定位前先停止目标，再准确设置位置和箭头朝向。

默认规划/控制链为 `SmacPlanner2D → SmoothPath(SimpleSmoother) → RPP`
（前向优先，`vx≤0.27 m/s`）。`forward_mppi` 是 DiffDrive 圆弧对照档，
`omni_mppi` 保留原有全向对照档，两者均不是默认。RViz 中绿色
`Raw Global Plan` 来自 `/plan`，蓝色 `Controller Path (Smoothed)` 来自
`/received_global_plan`，可用来区分“原始路线”与“实际跟随路线”。

`tuning_gui:=true` 会同时打开标准 `rqt_reconfigure` 窗口。只建议在
`/controller_server` 中逐项调整 `desired_linear_vel`、lookahead、
`rotate_to_heading_*`、`max_angular_accel`、`min_approach_linear_velocity` 与目标容差。
运行时修改不会写回 YAML，重启即恢复安全基线；不得用该窗口关闭
Collision Monitor、RPP 碰撞预测或安全监督。
控制链为
`Nav2/键盘/Unitree Move -> twist_mux -> velocity_smoother -> collision_monitor -> /cmd_vel -> CHAMP`。
公开 action 仍是 `/navigate_to_pose`；联调默认只检查地图边界、自由栅格、0.10 m
最小余量与定位健康，非法目标在接触规划器前被拒绝。内部 Nav2 action 为
`/navigate_to_pose_raw`。行为动作、趴下状态、定位失效或关键节点掉线时安全监督
发布 `/pause_navigation` 锁住输入并输出零速度；`/cmd_vel` 的唯一发布者应为
`collision_monitor`。

统一 Gazebo 导航入口会关闭误差较大的 CHAMP 足端平面里程计，改由
`go2_simulation_odom` 把 `/odom/ground_truth` 的首帧设为 `odom` 原点。机器人仍由
CHAMP 四足步态和 Gazebo 物理实际运动；该真值反馈只用于仿真控制闭环，真机不适用。

固定地图的 `/map` 由 `map_server` 从 PGM 读取，本来就不会随机器人移动扩展。
在线模式使用 `pointcloud_to_laserscan + slam_toolbox`，RViz 中的 `Live SLAM Map`
会随观测扩大；结束时使用：

```bash
bash simdog/src/go2_navigation/scripts/save_online_map.sh learning_room
```

该脚本保存的 `map.yaml/map.pgm` 可直接作为固定 AMCL 地图；AMCL 不要求三维 PCD。
脚本还会让 `~/go2_maps/online/latest` 指向最近保存的在线会话。它与
LIO-SAM/NDT 使用的 `~/go2_maps/latest` 不是同一目录；固定 AMCL 不应省略或写错
`map_dir`。固定模式不会自动选择地图质量，实际来源必须这样核实：

```bash
ros2 param get /map_server yaml_filename
ros2 lifecycle get /map_server                 # 期望 active [3]
```

若路径正确但 RViz 仍显示旧图，应关闭旧 RViz/导航进程后只启动一套入口；失活或退出的
`map_server` 不再更新 `/map` 时，RViz 仍可能保留最后收到的画面。在线模式使用
`map_session:=new` 会从空白 pose graph 开始，不会加载旧地图。

LIO-SAM PCD 转栅格只保留给 NDT 同源地图实验，不建议作为默认 AMCL 地图，因为多高度
点投影到同一二维格会产生伪障碍。固定 AMCL 示例：

```bash
ros2 launch go2_navigation simulation_navigation.launch.xml \
    navigation_mode:=static_map localization:=amcl \
    map_dir:=$HOME/go2_maps/online/learning_room rviz:=true
```

导航中普通取消点击 RViz `Navigation 2 -> Cancel`。卡住时使用：

```bash
ros2 service call /navigation/stop std_srvs/srv/Trigger "{}"
ros2 service call /navigation/resume std_srvs/srv/Trigger "{}"
```

导航栈运行时，键盘遥控必须从安全链入口发布：

```bash
source scripts/setup_unitree_sim.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/cmd_vel_teleop
```

## TF 所有权

主链必须保持为：

```text
map -> odom -> base_footprint -> base_link -> 关节与传感器
```

- 建图时由 LIO-SAM 发布动态 `map -> odom`；重定位时关闭该发布，由 NDT
  唯一发布。
- 普通 CHAMP、建图与真机式启动由 `footprint_to_odom_ekf` 发布动态
  `odom -> base_footprint` 和 `/odom`；两个统一 Gazebo 导航入口关闭它，改由
  `go2_simulation_odom` 唯一发布，避免足端接触估计严重低估仿真位移。
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

- Gazebo Classic、导航与建图入口默认打开 GUI；自动化或性能测试可显式传入
  `gui:=false`。GUI 仍可能受 NVIDIA 驱动与 OGRE 兼容性影响。
- NVIDIA OpenGL 异常时，先设置 `GO2_FORCE_NVIDIA_RENDERING=0`；仍有问题再
  按需使用 `LIBGL_ALWAYS_SOFTWARE=1`。
- LIO-SAM 当前关闭回环检测，正式地图应在目标场景重新采集和评估。
- 默认在线 Slam Toolbox、固定图 AMCL、实验 NDT、SmacPlanner2D + RPP/MPPI 与安全链
  已接通；在线模式已通过 12 次连续短目标。10 分钟压力、移动障碍和完整失效注入仍待完成。
- NDT 正式使用前必须准备有效 `GlobalMap.pcd` 并设置合理初始位姿。
- Unitree 兼容桥只保证所列消息、话题和请求的接口级兼容，不模拟 `/lowcmd`、
  BMS、无线遥控、真实足底力、障碍距离或真机固件的平衡与安全策略。
- 仿真动作轨迹不可下发真机；真机环境脚本只完成 DDS 配置，尚无硬件验证结果。
- 当前准确状态、历史验证结果和遗留问题以
  [PROJECT_MEMORY.md](PROJECT_MEMORY.md) 为准。
