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
GPU：NVIDIA GeForce RTX 5070
显存：12227 MiB
计算能力：12.0
驱动：595.84
CUDA：12.8.93
GPU NDT 架构：sm_120（已实测）
```

当前部署已生成并实测 `sm_120` GPU NDT，同时保留 OpenMP CPU 回退。只有 NDT
点云配准使用 CUDA 计算；Gazebo/RViz2 使用 GPU
进行 OpenGL 渲染，Gazebo 物理、CHAMP、LIO-SAM 的 GTSAM 图优化和大部分 PCL
预处理仍主要运行在 CPU 上。

## 目录结构

```text
.
├── simdog/                         # 唯一 ROS 2 colcon 工作空间
│   ├── src/
│   │   ├── README.md               # 包职责、数据链与维护边界索引
│   │   ├── go2/                    # 本项目功能、集成与安全链
│   │   ├── platform/               # Go2/CHAMP/Gazebo/Unitree 接口
│   │   ├── localization/           # LIO-SAM、NDT/GICP 定位实验
│   │   └── vendor/                 # 通用上游传感器与配准组件
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
cd /home/luhao/my/ROS/Go2_Bilibili_zhao-main
bash scripts/install_dependencies.sh
bash scripts/install_gpu_dependencies.sh
bash scripts/build_workspaces.sh
```

基础依赖脚本会安装 Gazebo、ROS 控制与传感器包、PCL、Nav2、SLAM Toolbox、
LIO-SAM 依赖和 GTSAM 4.1。GPU 脚本从 NVIDIA 官方软件源安装 CUDA 12.8
编译器和最小运行库，不替换现有显卡驱动。

构建脚本检测到 CUDA 12.8 时，会从 `nvidia-smi` 读取目标 GPU 的 compute
capability 并构建对应架构（本机 RTX 5070 为 `sm_120`）；未检测到 CUDA 时构建
OpenMP CPU 回退版本。CUDA 12.8 对 `SM_120` 的编译支持见
[NVIDIA CUDA 12.8 Release Notes](https://docs.nvidia.com/cuda/archive/12.8.0/cuda-toolkit-release-notes/index.html)。

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
cd /home/luhao/my/ROS/Go2_Bilibili_zhao-main
source scripts/setup_simdog.bash
ros2 launch go2_config gazebo_velodyne.launch.py gui:=true rviz:=true
```

终端二：启动 LIO-SAM 建图和建图 RViz2。

```bash
cd /home/luhao/my/ROS/Go2_Bilibili_zhao-main
source scripts/setup_simdog.bash
ros2 launch lio_sam lidar.launch.py rviz:=true
```

终端三：启动键盘遥控。按 `i` 前进、`,` 后退、`j`/`l` 转向、`k` 停止；先按
`k` 再关闭节点。

```bash
cd /home/luhao/my/ROS/Go2_Bilibili_zhao-main
source scripts/setup_simdog.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

完成巡视后，在**仍保持终端二 LIO-SAM 运行**的情况下另开终端四保存地图：

```bash
cd /home/luhao/my/ROS/Go2_Bilibili_zhao-main
bash simdog/save_Map.sh
test -s "$GO2_PROJECT_ROOT/go2_maps/latest/GlobalMap.pcd" && echo "地图保存成功"
```

默认地图文件是 `$GO2_PROJECT_ROOT/go2_maps/latest/GlobalMap.pcd`。若要使用其他目录和分辨率：

```bash
bash simdog/save_Map.sh "$GO2_PROJECT_ROOT/go2_maps/warehouse" 0.2
```

建图完成后，依次在键盘遥控、LIO-SAM、Gazebo 终端按 `Ctrl+C` 停止。不要依赖
关闭 LIO-SAM 时自动保存地图。

### 场景二：使用已有 PCD 地图重定位

此模式由 NDT 独占发布 `map -> odom`；LIO-SAM 仍负责提供惯导和点云预处理，但必须
显式关闭它的全局 TF 发布。

终端一：启动 Gazebo、完整四足控制器和传感器。

```bash
cd /home/luhao/my/ROS/Go2_Bilibili_zhao-main
source scripts/setup_simdog.bash
ros2 launch go2_config gazebo_velodyne.launch.py rviz:=true
```

终端二：启动 LIO-SAM，但让出 `map -> odom`。

```bash
cd /home/luhao/my/ROS/Go2_Bilibili_zhao-main
source scripts/setup_simdog.bash
ros2 launch lio_sam lidar.launch.py rviz:=true publish_map_to_odom:=false
```

终端三：启动 CUDA NDT 重定位。

```bash
cd /home/luhao/my/ROS/Go2_Bilibili_zhao-main
source scripts/setup_simdog.bash
ros2 launch ndt_relocalization ndt_localization.launch.py \
    map_path:=$GO2_PROJECT_ROOT/go2_maps/latest/GlobalMap.pcd \
    registration_backend:=cuda gpu_device_id:=0 use_rviz:=true
```

终端四：按需启动键盘遥控。

```bash
cd /home/luhao/my/ROS/Go2_Bilibili_zhao-main
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
`$GO2_PROJECT_ROOT/go2_maps/latest/GlobalMap.pcd`，还会以定位模式启动 CUDA NDT 并自动让 LIO-SAM
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
[`go2_behaviors/README.md`](simdog/src/go2/go2_behaviors/README.md)。

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
[`go2_unitree_sim_bridge/README.md`](simdog/src/go2/go2_unitree_sim_bridge/README.md)。

## 建图与地图保存

建图保存的完整顺序见上方“场景一”。以下命令必须在 LIO-SAM 尚未关闭时执行：

```bash
bash simdog/save_Map.sh
```

默认保存到 `$GO2_PROJECT_ROOT/go2_maps/latest`。也可以指定目录和分辨率：

```bash
bash simdog/save_Map.sh $GO2_PROJECT_ROOT/go2_maps/warehouse 0.2
```

地图保存依赖正在运行的 LIO-SAM `/lio_sam/save_map` 服务。

## NDT 重定位

重定位的完整终端顺序见上方“场景二”。单独启动 NDT 的命令如下：

```bash
ros2 launch ndt_relocalization ndt_localization.launch.py \
    map_path:=$GO2_PROJECT_ROOT/go2_maps/latest/GlobalMap.pcd \
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
bash simdog/src/go2/go2_navigation/scripts/save_map.sh $GO2_PROJECT_ROOT/go2_maps/latest

# 只由已有 GlobalMap.pcd 重建地图包（支持裁剪到导航区域）
ros2 run go2_navigation build_map_bundle --map-dir $GO2_PROJECT_ROOT/go2_maps/latest \
    --x-min -2 --x-max 8 --y-min -4 --y-max 4

# 默认：在线 SLAM + 导航，从空图开始；map_session 可传已有会话目录续建。
ros2 launch go2_navigation simulation_navigation.launch.xml \
    navigation_mode:=online_slam map_session:=new \
    controller_profile:=forward_mppi rviz:=true

# 固定地图 + AMCL（静态 /map 不会继续扩展）
ros2 launch go2_navigation simulation_navigation.launch.xml \
    navigation_mode:=static_map map_dir:=$GO2_PROJECT_ROOT/go2_maps/online/latest \
    localization:=amcl controller_profile:=forward_mppi rviz:=true \
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

默认规划/控制链为
`SmacPlanner2D → SmoothPath(SimpleSmoother) → MPPI(DiffDrive)`。
MPPI 每个控制周期采样 800 条候选轨迹（40 步 × `0.10 s` 预测窗口），由 Goal、
GoalAngle、Obstacles、PathAlign/Follow/Angle 与 PreferForward 等 critic 加权选出
一条更优轨迹，是预测型控制器；前向限定 `vx∈[0,0.27 m/s]`、`vy=0`、`wz≤0.40 rad/s`。
行为树的 `TerminalPathLatch` 会先确认当前路径确属
本次目标：路径末端允许 `0.075 m` 栅格中心误差和 `0.01 rad` 数值航向误差，再用实时
`map→base_footprint` TF 确认机器人同时进入原始目标与路径末端的 `0.30 m` XY 容差。
首次进入后保持锁存并暂停 1 Hz 重规划，防止 Humble 的 `setPlan()` 重置终点定向状态。
定位短时漂出边界不会解除锁存，新目标、行为树 `halt()` 或 recovery 会清除锁存；同一
XY 只改变 yaw 也必须先得到一条新路径。Smac 的 `GridBased.tolerance=0.0`，不可达的
原始目标会明确规划失败，不再静默选择 `0.25 m` 内的替代终点。
`forward_rpp` 是 Rotation Shim + RPP 对照档（原默认），`omni_mppi` 是全向对照档，
两者均需显式传 `controller_profile:=forward_rpp|omni_mppi`。RViz 中绿色
`Raw Global Plan` 来自 `/plan`，蓝色 `Controller Path (Smoothed)` 来自
`/received_global_plan`，可用来区分“原始路线”与“实际跟随路线”。

`forward_rpp` 对照档由外层 Rotation Shim 包裹 RPP：进入 `0.30 m` XY 容差后保持
`linear.x=0`，以 `0.45 rad/s`、最大 `1.0 rad/s²` 对齐到 `0.10 rad` yaw；shim 参数为
`rotate_to_goal_heading=true`、`closed_loop=false`、路径进入/退出阈值
`1.40/0.40 rad`、采样距离 `0.50 m`、旋转碰撞预测 `1.0 s`，内层 RPP 的
`use_rotate_to_heading=false`。普通到达标准为 `0.30 m/0.10 rad`。`closed_loop=false`
只表示 shim 按角加速度约束生成命令，不把当前仿真里程计的 1 秒角速度窗口当成低延迟
反馈，并不是关闭姿态闭环；目标 yaw 仍由 TF 和 GoalChecker 闭环判定。不应通过放宽
yaw 或关闭碰撞保护解决终点摆动。可这样回读当前控制器插件与参数：

```bash
ros2 param get /controller_server FollowPath.plugin
ros2 param get /controller_server FollowPath.vx_max
ros2 param get /controller_server FollowPath.wz_max
# forward_rpp 对照档下可回读 shim 参数：
# ros2 param get /controller_server FollowPath.primary_controller
# ros2 param get /controller_server FollowPath.rotate_to_goal_heading
# ros2 param get /controller_server FollowPath.closed_loop
```

终点转向异常时先取消当前目标，并通过安全输入 `/cmd_vel_teleop` 验证底层；导航运行时
不得直接发布最终 `/cmd_vel`。先在一个终端启动只读诊断，再在另一个终端发布 5 秒命令：

```bash
ros2 run go2_navigation rotation_diagnostics \
    --mode manual --duration 10 --expected-wz 0.45

timeout 5 ros2 topic pub -r 20 /cmd_vel_teleop geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.45}}"

# 单个导航目标开始前运行；它不发布速度或目标
ros2 run go2_navigation rotation_diagnostics \
    --mode navigation --acquire-timeout 120 --duration 10 \
    --xy-tolerance 0.30 --yaw-tolerance 0.10
```

Goal Guard 会把已接受的原始 RViz 目标以 transient-local QoS 发布到只读话题
`/navigation/accepted_goal`。诊断分别报告“机器人→原始目标”“机器人→路径末端”和
“路径末端→原始目标”，并统计终点前实际重规划频率以及锁存后的 `/plan` 数量；据此
区分上游未产生命令、`twist_mux`/平滑/碰撞层拦截、CHAMP 未执行、`odom`/TF 未反馈、
替代路径终点和终点锁存后仍被重规划。若 `map→odom` 单次修正超过
`0.10 m` 或 `0.10 rad`，应先处理地图/AMCL，而不是继续调控制器。
navigation 模式会等待新目标和双终点 XY 容差，进入后最多采样 `--duration` 秒；action
成功后再观察 1 秒停稳误差。`--acquire-timeout` 内尚未进入终点时返回
`INCOMPLETE`（退出码 2），不会再把途中距离误写成终点失败。多行命令的反斜杠 `\`
必须是行末最后一个字符，后面不能留空格。

2026-08-13 的无界面同 XY `+90°` 实测中，action 在 `4.8 s` 内 `SUCCEEDED`；四级速度
终点段均为 `max|linear.x|=0`、`max|angular.z|=0.45 rad/s`、0 次换向，锁存后
`/plan=0`，证明本轮重规划/控制器反复切换已消除。但该次 AMCL 随后产生
`0.414 m` 单步 `map→odom` 修正，停稳后机器人到原始目标约 `1.23 m`，因此整体验收仍
判为 FAIL，需重新校准 `home_02` 初始位姿或地图定位后再做 12 目标测试。

后续现场已经确认终点不再来回左右摆动。一次旧诊断在采样结束时显示机器人到目标
`5.401 m`、路径末端到目标仅 `0.010 m`，且 `map→odom` 单步只有
`0.020 m/0.009 rad`；这代表采样窗口结束时仍在途中，不是终点控制或 AMCL 失败。

本机纯旋转基线已经实际执行，但当前判定为 **FAIL**，所以尚未继续做新实现的 12 目标
验收。四级速度链能把 `±0.45 rad/s` 原样送至 `/cmd_vel`，真值、`/odom` 和
`odom→base_footprint` 累计 yaw 误差在 `0.03 rad` 内；失败集中在 CHAMP/Gazebo 实体层：
`+0.45 rad/s` 稳态增益约 `33%`，`-0.45 rad/s` 约 `94%`，两方向每 90° 等效平移漂移
约 `0.38/0.13 m`，均超过 `0.10 m` 标准。把四脚摩擦系数从 `0.6` 单变量提高到 `1.0`
没有改善正向增益，试验值已撤回。进一步 A/B 发现将 CHAMP `stance_depth` 从 `0.01 m`
改为上游常用的 `0.0 m` 后，双向结果改善到约 `70%/61%`、`0.11/0.06 m/90°`；该修改
已保留，但反向增益和正向漂移仍略未达标。PID、支撑时长和抬脚高度试验均无完整收益并
已撤回。现阶段仍应校准 CHAMP 原地旋转步态和接触模型，不能继续用 Nav2 容差或控制器
参数补偿。

`tuning_gui:=true` 会同时打开标准 `rqt_reconfigure` 窗口，并直接定位到
`/go2_lidar_scan_converter`。当前只有 `min_height`、`max_height` 是经代码保证真正
在线生效的雷达投影参数；其他投影参数会拒绝热修改并提示重启。需要调控制器时再在树中
选择 `/controller_server`，逐项调整 `desired_linear_vel`、lookahead、
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
在线模式使用 `go2_lidar_scan + slam_toolbox`，RViz 中的 `Live SLAM Map`
会随观测扩大。`go2_lidar_scan` 复用上游 `pointcloud_to_laserscan`，集中维护
`/velodyne_points -> /scan` 的重力对齐、中文诊断 Marker 与 Nav2 clearing 契约；
`always_subscribe=true` 保证 Collision Monitor 等订阅者短暂重连后 `/scan` 仍持续输出。
运动时会按每帧点云时间戳生成 `base_footprint -> velodyne_level`：保留雷达平移和 yaw，
去掉四足步态造成的 roll/pitch；`/scan.frame_id` 因此固定为 `velodyne_level`。TF 缺失时
该帧不输出，不会拿旧姿态拼新扫描。正常导航只启动一个 `/scan` 发布者；只有显式传
`lidar_debug_raw_scan:=true` 才增加不接入下游的 `/scan_raw`。
开始建图前可先独立检查转换，不加载 Nav2：

```bash
source scripts/setup_simdog.bash
GO2_D435_GAZEBO_ENABLED=0 \
ros2 launch go2_lidar_scan simulation_scan_debug.launch.xml \
  lidar_debug_raw_scan:=true tuning_gui:=true
```

RViz 中应看到 `Velodyne 3D Points`、橙色 `Leveled Navigation Scan`、洋红色
`Raw Tilting Scan` 和中文 `Scan Health`；该入口与完整导航都会发布 `/scan`，二者不能
同时运行。完整三终端顺序、各 Display 含义、预期结果和失败分支见
[go2_lidar_scan 使用说明](simdog/src/go2/go2_lidar_scan/README.md)。
先前完整运动 A/B 在机身倾斜 `5.15°` 时得到“原始切片 4430、重力对齐 0 个地面
获胜端点”，240 个采样点云 TF 全部成功，`/scan=8.89 Hz`，`map→odom` 最大单步
`0.0224 m/0.00698 rad`，只证明该批样本的倾斜地面端点门 PASS。人工在线复测仍出现
白色扇形线和幽灵代价岛；用户随后通过 rqt 将窗口调为 `+0.20..+0.30 m` 后，现场目视
在线 SLAM 恢复正常。该值现已固化为正式 `/scan` 默认值，并继续支持在 rqt 动态调整；
完整覆盖建图、保存和固定图导航仍待按流程验收。`/scan_raw` 固定保留旧
`-0.05..+0.10 m` 作为 A/B。1800 水平列对照只有 `5.06 Hz`；默认改用 900 列
（0.4°）只为恢复 Gazebo 实时率。导航仿真默认 `use_d435_navigation:=false`，避免与本任务无关的 D435 渲染争抢
资源；普通 Gazebo 入口仍启用 D435，需要联合感知 A/B 时可显式传 `true`。

几何闭环通过后已单独完成 `0.50 m` 候选实验：六处相关量程配置同步修改，前/后/左/右
四方向、四距离各 3×10 帧。后方 0.50 m 为 0% 检出并发生接触，右方也发生接触，故
候选被否决，全部配置恢复 `0.90 m`；不能只改单个 `/scan.range_min`。
结束时使用：

```bash
bash simdog/src/go2/go2_navigation/scripts/save_online_map.sh my_world_full_v1
```

该脚本保存的 `map.yaml/map.pgm` 可直接作为固定 AMCL 地图；AMCL 不要求三维 PCD。
同目录还保存 `slam.posegraph/slam.data` 和记录本次实际雷达参数的 `session.yaml`。
脚本还会让 `$GO2_PROJECT_ROOT/go2_maps/online/latest` 指向最近保存的在线会话。它与
LIO-SAM/NDT 使用的 `$GO2_PROJECT_ROOT/go2_maps/latest` 不是同一目录；固定 AMCL 不应省略或写错
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
    map_dir:=$GO2_PROJECT_ROOT/go2_maps/online/my_world_full_v1 rviz:=true
```

导航中普通取消点击 RViz `Navigation 2 -> Cancel`。卡住时使用：

```bash
ros2 service call /navigation/stop std_srvs/srv/Trigger "{}"
ros2 service call /navigation/resume std_srvs/srv/Trigger "{}"
```

当前 Global Costmap 的现场基线为 `inflation_radius=0.50 m`、
`cost_scaling_factor=0.5`；Local Costmap 保持 `0.30 m/3.0`。配合正式雷达高度窗
`0.20..0.30 m` 后，用户反馈幽灵代价区域已基本不出现，偶尔只在远处看到残余。高度窗
负责减少错误输入端点，全局膨胀参数只控制每个端点周围代价扩散的范围与梯度。当前较宽、
缓慢衰减的全局代价带会让路径更早远离墙面，也可能封住窄通道；不能把远处残余描述为
已从输入端彻底消失。

2026-08-22 已修复 `/scan` 空射线 clearing 契约：无回波恢复为 `+inf`，local/global
source 都启用 `inf_is_valid=true`，并使用 `obstacle_max_range=14.0 m <
raytrace_max_range=15.0 m`。最终 2 m 方块的 scan/Velodyne 各 3×30 帧检出率均为
100%；实际删除后精确方块区域 Local Costmap lethal 格从 64 降至 57，删除前背景为
55。这个删除测试只证明“真实障碍消失后能够清除”，不能证明运动期不会生成新的错误
端点；当前现场结果是主要现象基本消失、远处偶发残余。Global Inflation 已采用
`0.20 m/0.5` 现场基线，但尚未完成安全标定；现有
stop/decel zone 仍位于 LiDAR 近距盲区内，因此系统级障碍安全门仍为 **FAIL**，三个
profile 仍为 `UNCALIBRATED`。不得把本次 clearing PASS 误写成近距防撞已通过。证据和
后续门禁见
[幽灵障碍与近距碰撞调查](simdog/src/go2/go2_navigation/docs/costmap_ghost_obstacle_investigation.md)。

同日运动复测实测出白色放射线的一条输入成因：机身原地转向倾斜 `4–6°` 时，旧坐标
高度切片会让地面成为最近端点；最终样本旧投影为 4430 个地面获胜格，重力对齐后为 0。
`map -> odom` 最大单步为 `0.0224 m/0.00698 rad`，完整栈扫描频率 `8.89 Hz`，独立
RViz A/B 约 `9.15 Hz`。最终 135 秒运动中 costmap 仍记录到两次过旧观测丢弃；这是跳过
迟到帧，不会凭空生成端点，但继续作为时序残余记录。由于后续人工在线 SLAM 仍复现扇形
白线；随后用户现场把高度窗调为 `0.20..0.30 m` 后目视恢复正常，该值现为默认基线。
这不替代完整路线建图、保存和固定图导航验收；下方 Collision Monitor 近距几何门也仍
独立为 FAIL，不得混为一谈。

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
    map_path:=$GO2_PROJECT_ROOT/go2_maps/latest/GlobalMap.pcd \
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
- 默认在线 Slam Toolbox、固定图 AMCL、实验 NDT、SmacPlanner2D + MPPI/RPP 与安全链
  已接通；在线模式已通过 12 次连续短目标。`/scan` 空射线 clearing 已通过一次实际删除
  方块冒烟测试，但近距 Collision Monitor 几何仍为 FAIL；静止/多位置/10 分钟压力、
  移动障碍和完整失效注入尚未完成，系统级安全门仍为 FAIL。
- 默认控制档已由 `forward_rpp`（Rotation Shim + RPP）切换为 `forward_mppi`
  （DiffDrive MPPI）。RPP 对照档仍保留 Rotation Shim、实时 TF 终点路径锁存、
  0.45 rad/s 开环限加速度定向与五层只读诊断。旧实现的两个内部同 XY `±90°` 专项目标曾成功，但新锁存
  的纯旋转基线仍因 CHAMP 实体旋转增益/漂移未全部达标而失败；`stance_depth=0.0 m`
  已显著缩小方向差异，但仍没有继续执行固定
  `home_02` 12 目标；此前 AMCL `map→odom` 米级修正还曾使精确 `180°` 目标进入障碍
  附近，固定图验收不能记为通过。
- NDT 正式使用前必须准备有效 `GlobalMap.pcd` 并设置合理初始位姿。
- Unitree 兼容桥只保证所列消息、话题和请求的接口级兼容，不模拟 `/lowcmd`、
  BMS、无线遥控、真实足底力、障碍距离或真机固件的平衡与安全策略。
- 仿真动作轨迹不可下发真机；真机环境脚本只完成 DDS 配置，尚无硬件验证结果。
- 当前准确状态、历史验证结果和遗留问题以
  [PROJECT_MEMORY.md](PROJECT_MEMORY.md) 为准。
