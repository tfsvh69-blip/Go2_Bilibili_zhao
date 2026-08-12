# 仓库指南

## 语言与规则文件同步

本文件、后续新增或修改的代码注释、文档、提交说明和协作回复均使用中文。命令、ROS 话题、包名、文件名及必要的技术术语保持原样。

根目录 `AGENTS.md` 与 `CLAUDE.md` 是内容完全一致的协作规则镜像。修改任一文件时，必须在同一任务中同步修改另一个文件，并使用 `cmp -s AGENTS.md CLAUDE.md` 验证两者一致；不得只维护其中一份。

## 底层开发思维

坚决杜绝闭门造车和重复造轮子。

1. 启动新项目、开发较大功能或制定重要技术方案前，优先在 GitHub 等代码托管平台检索同类项目、官方文档、实现思路和相关源码。
2. 发现成熟的现成项目或组件时，优先评估其许可证兼容性、维护状态、安全风险、社区活跃度和适配成本；评估通过后优先复用，不盲目重新实现，也不未经评估直接引入。
3. 简单 bug 修复、边界明确的小改动和无需联网的离线任务可不强制开展外部调研，以避免不必要的流程成本。
4. 对相关开源项目进行充分检索、阅读和拆解：符合许可证且适配本项目的内容可直接复用，有参考价值但不适合直接引入的方案应提炼其设计思路后再实现。
5. 重要方案尽量交叉比较多套开源实现，综合其功能完整性、代码质量、性能、可维护性、安全性及 ROS 2 Humble/Gazebo Classic 11 兼容性，吸收优势并规避已知缺陷。
6. 调研结论应在方案说明、代码注释、文档或 `PROJECT_MEMORY.md` 中按任务规模留下必要记录，包括参考来源、选择理由、许可证与适配风险；引用或复用第三方内容时保留所需的版权和许可证声明。

## 闭环优先与约束渐进

功能尚未形成可重复闭环时，优先采用最小必要约束，避免因为过早引入严格门槛而掩盖
真正的链路问题。此规则不允许关闭急停、硬件限位、碰撞保护等不可妥协的安全措施。

1. 联调阶段只拦截明确非法、会导致节点崩溃或存在直接安全风险的输入；可调的安全余量、
   质量阈值、超时和性能门槛先使用宽松且有记录的值。
2. 先证明“输入能够到达执行器、系统产生预期动作、反馈能够闭环”，再根据实测数据逐步
   收紧参数。每次只调整少量相关约束，并保留调整前后的验证证据。
3. 新增门禁或限制时必须给出可操作的拒绝原因、可配置参数和联调默认值；不得用静默丢弃、
   硬编码高阈值或一次叠加多层限制代替故障诊断。
4. 若闭环失败，应先区分通信、坐标系、地图、规划、控制和执行器链路，再判断是否需要收紧
   安全策略；不得在根因未明时持续增加限制。

## 教学与可解释协作

用户处于学习阶段时，除了修复功能，还必须帮助其建立能够迁移到其他机器人项目的
导航、定位、建图和控制心智模型，不得只给出一组命令或静默修改参数。

1. 非平凡故障或算法调整按“肉眼现象 → 数据/控制链 → 根因 → 主流方案 → 本项目选择
   → RViz/CLI 可观察验证”顺序说明，并解释未选方案的取舍。
2. 修改参数时说明它控制的物理或算法含义、当前值的来源、预期可见变化和过大/过小的
   表现；每次只调整少量相关参数并设置可观察检查点。
3. 必须明确区分静态地图、定位点云、在线 SLAM 地图、全局代价图与局部代价图，说明哪些数据
   应当动态变化、哪些不会随机器人移动自动扩展。
4. 操作指南要给出具体界面位置、按钮名、终端命令、预期输出及失败分支；遇到 RViz 红项
   或机器人不动时，先教用户如何停止和保证安全，再开展诊断。
5. 重要方案应提供官方文档或上游源码依据，同时把“已实测”、“静态推断”和“后续实验”分开表达，
   不得把尚未运行验证的推荐写成已经完成的事实。
6. 用户可见的新专有名词、RViz Display、面板字段、颜色、Marker、状态或典型故障截图，
   必须同步维护到 `文档/Go2导航建图与RViz初学者图解手册.md`。解释至少包含数据来源、
   正常含义、异常表现和可观察验证；单靠文字不直观时使用真实截图或标注图，并优先采用
   初学者能迁移理解的比喻。

## 项目定位与结构

本目录是 Unitree Go2 完整四足仿真开发环境，目标系统为 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Classic 11。项目仅维护 `simdog/` 主 colcon 工作空间，不再维护焊死腿关节、依赖 planar-move 滑行的简化机器人。

- `simdog/src/unitree-go2-ros2/`：Go2 模型、CHAMP 四足步态、`ros2_control`、Gazebo 世界和机器人配置。
- `simdog/src/go2_behaviors/`：复用标准关节轨迹接口实现的打招呼、点头、伸展、趴下、挥爪和简单舞蹈。
- `simdog/src/go2_unitree_sim_bridge/`：把 Gazebo/CHAMP 映射为 Unitree Sport API 消息、话题和受支持请求。
- `simdog/src/lidar_localization_ros2/`：BSD-2-Clause 的 NDT/GICP 实验定位库，提供自动初始定位、诊断、重定位与 PCD 转二维地图工具。
- `simdog/src/go2_navigation/`：自主导航包，默认提供在线 Slam Toolbox 建图导航，固定地图默认使用 AMCL，
  NDT + 二维 EKF 作为实验档，并提供 SmacPlanner2D + RPP（默认）/MPPI（对照）、安全控制链与健康检查。
- `simdog/src/unitree_ros2_interfaces/`：固定的 Unitree 官方 `unitree_ros2 v0.3.0` `unitree_go`、`unitree_api` 接口快照。
- `simdog/src/LIO-SAM/`：Velodyne 与 IMU 融合建图。
- `simdog/src/ndt_relocalization/`：基于 PCD 地图的 NDT 重定位 ROS 2 节点。
- `simdog/src/fast_gicp/`：CUDA NDT 点云配准后端。
- `simdog/src/ndt_omp_ros2/`：OpenMP CPU 点云配准回退后端。
- `simdog/src/realsense_ros_gazebo/`、`pointcloud_to_laserscan/`：相机仿真与点云转换组件。
- `scripts/`：依赖安装、构建、环境加载和 GPU 运行验证脚本。
- `build/`、`install/`、`log/`：构建产物，不作为源码修改。
- `文档/`：参考资料，不作为功能源码。

## 硬件与 GPU 基线

当前电脑实际 GPU 是 `NVIDIA GeForce RTX 4060 Laptop GPU`，显存 `8188 MiB`，计算能力 8.9；不是 RTX 5070。当前驱动验证版本为 `595.84`，CUDA 工具链为 12.8，GPU NDT 编译目标为 `sm_89`。

只有 NDT 点云配准使用 CUDA 计算。Gazebo 和 RViz2 可使用 GPU 进行 OpenGL 渲染；Gazebo 物理、CHAMP、LIO-SAM 的 GTSAM 图优化和大部分 PCL 预处理仍主要使用 CPU。硬件、驱动、CUDA 或编译架构发生变化时，必须同步更新 `README.md`、`GPU_TESTING.md`、`PROJECT_MEMORY.md`、`AGENTS.md` 和 `CLAUDE.md`。

## 构建与环境加载

统一从项目根目录执行：

```bash
bash scripts/install_dependencies.sh
bash scripts/install_gpu_dependencies.sh
bash scripts/build_workspaces.sh
source scripts/setup_simdog.bash
```

Unitree 接口仿真使用 `source scripts/setup_unitree_sim.bash`，默认配置 CycloneDDS、`lo` 和 Domain 0；脚本不会继承终端遗留的 `ROS_DOMAIN_ID`，隔离测试只能通过 `GO2_UNITREE_SIM_DOMAIN_ID=<id>` 显式覆盖。真机只使用 `source scripts/setup_unitree_real.bash <网卡名>`，默认 Domain 0，且不得启动仿真 bridge。

`scripts/build_workspaces.sh` 当前只构建 `simdog`。检测到 `/usr/local/cuda-12.8/bin/nvcc` 时，会为 `fast_gicp` 和 `ndt_relocalization` 构建 `sm_89` CUDA 后端；否则构建 OpenMP CPU 回退版本。

修改 xacro、Python 启动文件或通过 `--symlink-install` 链接的资源后，通常可直接重启相应节点。修改 C++、CUDA、消息、服务或 CMake 配置后，应执行：

```bash
cd simdog
colcon build --symlink-install --packages-select <包名>
source install/setup.bash
```

## 常用启动方式

加载 `simdog` 后，按需分终端运行：

```bash
ros2 launch go2_config gazebo_velodyne.launch.py rviz:=true
ros2 launch lio_sam lidar.launch.py rviz:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run go2_behaviors go2_behavior hello
```

自主导航统一入口：

```bash
# 默认：从空图开始在线建图导航，地图会随机器人探索扩展
ros2 launch go2_navigation simulation_navigation.launch.xml
# 保存后得到的 map.yaml/pgm 可直接用于固定 AMCL，不要求 GlobalMap.pcd
bash simdog/src/go2_navigation/scripts/save_online_map.sh learning_room
# 固定二维地图：AMCL 定位，静态地图不会自行扩展
ros2 launch go2_navigation simulation_navigation.launch.xml \
    navigation_mode:=static_map map_dir:=$HOME/go2_maps/online/latest \
    localization:=amcl
# 核实 map_server 实际加载来源；固定模式不会自动挑选地图质量
ros2 param get /map_server yaml_filename
ros2 run go2_navigation health_check --mode online_slam --localization amcl
```

`~/go2_maps/online/latest` 由 `save_online_map.sh` 指向最近保存的 Slam Toolbox
会话；不要与 LIO-SAM/NDT 使用的 `~/go2_maps/latest` 混淆。在线启动传
`map_session:=new` 必须从空白 pose graph 开始，不得隐式读取旧会话。

两种模式都由统一入口将 Unitree bridge 固定接入 `/cmd_vel_unitree`。旧的
`simulation_online_mapping_navigation.launch.xml` 仅保留为兼容 wrapper。默认控制档为
`controller_profile:=forward_rpp`（前向优先），`forward_mppi` 是 DiffDrive 对照，
`omni_mppi` 是全向对照。默认路径链为
`SmacPlanner2D -> SmoothPath(SimpleSmoother) -> RPP`；`PoseProgressChecker` 将平移和
转向都计为进展，普通目标容差为 `0.30 m/0.25 rad`。传递
`tuning_gui:=true` 可打开标准 `rqt_reconfigure`；其修改只在当前运行生效，
不得用于关闭碰撞或锁速保护。导航控制链为
`Nav2/键盘/Unitree Move -> twist_mux -> velocity_smoother -> collision_monitor -> /cmd_vel -> CHAMP`；
目标门禁公开 `/navigate_to_pose`，内部 Nav2 action 为 `/navigate_to_pose_raw`；越界、障碍、定位失效目标会被拒绝而不触发规划器。行为动作、趴下状态、定位失效或关键导航节点掉线时安全监督发布 `/pause_navigation` 锁住输入并输出零速度。
导航中普通取消使用 RViz `Navigation 2 -> Cancel`；卡住时调用 `/navigation/stop`，
健康后调用 `/navigation/resume`，旧目标不会自动续行。
导航栈运行时，键盘必须使用 `source scripts/setup_unitree_sim.bash`，并执行
`ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop`；
不得直接发布最终出口 `/cmd_vel`。
两个统一 Gazebo 导航入口还会关闭 CHAMP 足端平面里程计，使用
`go2_simulation_odom` 将 `/odom/ground_truth` 转换为从零开始的 `/odom` 和
`odom -> base_footprint`；这是仿真闭环基准，真机及普通分组件启动不得使用。

主 Gazebo 启动文件默认 `unitree_bridge:=true`，同时启动行为服务端和 Unitree 兼容桥；不需要兼容层时显式传入 `unitree_bridge:=false`。Unitree `Move` 活动期间不得并行运行键盘遥控。

桌面环境可执行 `bash simdog/start.sh`，依次启动带 GUI 的 Gazebo、LIO-SAM、键盘遥控，并在找到 PCD 地图时启动 NDT。默认地图为 `~/go2_maps/latest/GlobalMap.pcd`；没有地图时必须跳过 NDT。建图完成后使用 `bash simdog/save_Map.sh` 保存地图。

NDT 单独启动示例：

```bash
ros2 launch ndt_relocalization ndt_localization.launch.py \
    map_path:=$HOME/go2_maps/latest/GlobalMap.pcd \
    registration_backend:=cuda gpu_device_id:=0 use_rviz:=true
```

与 NDT 同时运行 LIO-SAM 时，必须使用
`ros2 launch lio_sam lidar.launch.py publish_map_to_odom:=false`，确保
`map -> odom` 只由 NDT 发布；`simdog/start.sh` 会自动选择正确所有者。

Gazebo Classic 与导航/建图入口默认打开 GUI，便于观察实体运动；自动化或性能测试才显式传入 `gui:=false`。图形界面受 NVIDIA 驱动与 OGRE 兼容性影响时，先设置 `GO2_FORCE_NVIDIA_RENDERING=0`，再按需使用 `LIBGL_ALWAYS_SOFTWARE=1`。

## 代码风格与命名

遵循相邻包的既有风格。Python 与 C++ 均使用四个空格缩进；Python 文件和函数、ROS 参数与话题使用 `snake_case`，C++ 类使用 PascalCase，ROS 包名使用小写下划线。启动文件应表达用途，例如 `gazebo_velodyne.launch.py`；新启动文件优先使用 XML 格式（`.launch.xml`）。保持 URDF/xacro 格式，勿手动编辑生成文件。

## 测试要求

当前未配置统一自动化测试或代码检查工具。修改后应构建受影响的包，并以无界面方式启动相应功能。主仿真至少检查：

```bash
ros2 topic hz /velodyne_points
ros2 topic hz /imu/data
ros2 topic echo --once /odom
ros2 topic hz /joint_states
ros2 control list_controllers
ros2 topic hz /sportmodestate
ros2 topic hz /lowstate
```

传感器或控制改动还应验证 `/clock`、`/cmd_vel` 和 TF。GPU NDT 改动必须执行：

```bash
bash scripts/verify_gpu_runtime.sh
```

该脚本使用独立 `ROS_DOMAIN_ID`，不得复用正在运行的 Gazebo ROS 图，以免仿真时间与系统时间造成 TF 冲突。

说明验证结果时，应明确是否需要正式 PCD 地图、Gazebo GUI、NVIDIA GPU 或额外硬件。

## 已知运行边界

- Velodyne 使用 `gpu_ray`；改回高分辨率 CPU `ray` 会显著降低 Gazebo 实时率。
- 当前已验证遥控、四足步态、传感器、LIO-SAM 建图、地图保存和 NDT 重定位。
- 仿真动作通过 `FollowJointTrajectory` 控制当前 Gazebo 模型，不等同于真机固件中的 Unitree Sport API，不可直接下发真机。
- Unitree 兼容桥只保证已列出的消息、话题与请求的接口级兼容，不实现 `/lowcmd`、无线遥控、BMS、真实足底力、障碍距离或真机固件行为；真机环境入口尚未经过硬件验证。
- 固定地图 AMCL、实验 NDT、SmacPlanner2D + RPP/MPPI 与安全控制链已接通；在线 Slam Toolbox
  是默认学习流程，不与 AMCL、NDT 或 map_server 同时启动。在线模式 12 次短目标已通过，
  10 分钟压力、移动障碍和完整失效注入仍未完成。
- 统一 Gazebo 导航已用真值适配里程计验证在线短目标与转向目标；这只能证明仿真控制闭环，
  不代表真机足端/惯性里程计精度已经解决。
- 导航模式不运行完整 LIO-SAM；`map -> odom` 在线模式由 Slam Toolbox、固定图默认由 AMCL
  唯一发布，`lidar_ndt` 通过二维 EKF 发布，CUDA NDT 作为 `localization:=ndt_cuda` 实验档保留。
- LIO-SAM 当前关闭回环检测；正式地图需要在目标场景重新采集并评估质量。
- NDT 使用前必须提供有效 `GlobalMap.pcd`，并通过 `/initialpose` 给出合理初始位姿。
- 固定 AMCL 应优先使用 Slam Toolbox 原生保存的二维 `map.yaml/pgm`；LIO-SAM PCD 高度
  投影容易把多高度点变成二维伪障碍，只用于需要 PCD 同源数据的 NDT 实验档。
- 根目录已配置 Git，默认分支为 `main`，远端为 `origin`；提交前仍须核对工作区，避免混入用户的无关修改。

## 提交与合并请求

提交使用简短中文祈使句和范围，例如 `go2_config: 调整雷达坐标系`，每次提交只处理一个主题。推送前执行 `git status --short` 和与改动风险相称的验证。合并请求应说明受影响包、构建与启动验证结果、修改的 ROS 话题或坐标系；涉及可视化时附 RViz 或 Gazebo 截图。

## 项目记忆维护

将当前项目状态统一维护在根目录 `PROJECT_MEMORY.md`。每次完成一个可验证阶段、修改启动入口、变更关键参数、调整硬件基线或发现运行限制后，必须同步更新：日期、阶段目标、实际操作与结果、验证命令、遗留问题和下一步。该文件只保留当前准确状态和必要的阶段记录；新信息应合并或替换过期条目，不创建 `PROJECT_MEMORY_v2.md` 等旧版本副本。
