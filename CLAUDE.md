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

## 项目定位与结构

本目录是 Unitree Go2 完整四足仿真开发环境，目标系统为 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Classic 11。项目仅维护 `simdog/` 主 colcon 工作空间，不再维护焊死腿关节、依赖 planar-move 滑行的简化机器人。

- `simdog/src/unitree-go2-ros2/`：Go2 模型、CHAMP 四足步态、`ros2_control`、Gazebo 世界和机器人配置。
- `simdog/src/go2_behaviors/`：复用标准关节轨迹接口实现的打招呼、点头、伸展、趴下、挥爪和简单舞蹈。
- `simdog/src/go2_unitree_sim_bridge/`：把 Gazebo/CHAMP 映射为 Unitree Sport API 消息、话题和受支持请求。
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

Unitree 接口仿真使用 `source scripts/setup_unitree_sim.bash`，默认配置 CycloneDDS、`lo` 和 Domain 1；真机只使用 `source scripts/setup_unitree_real.bash <网卡名>`，默认 Domain 0，且不得启动仿真 bridge。

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
ros2 launch go2_config gazebo_velodyne.launch.py gui:=false rviz:=true
ros2 launch lio_sam lidar.launch.py rviz:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run go2_behaviors go2_behavior hello
```

主 Gazebo 启动文件默认 `unitree_bridge:=true`，同时启动行为服务端和 Unitree 兼容桥；不需要兼容层时显式传入 `unitree_bridge:=false`。Unitree `Move` 活动期间不得并行运行键盘遥控。

桌面环境可执行 `bash simdog/start.sh`，依次启动无界面 Gazebo、LIO-SAM、键盘遥控，并在找到 PCD 地图时启动 NDT。默认地图为 `~/go2_maps/latest/GlobalMap.pcd`；没有地图时必须跳过 NDT。建图完成后使用 `bash simdog/save_Map.sh` 保存地图。

NDT 单独启动示例：

```bash
ros2 launch ndt_relocalization ndt_localization.launch.py \
    map_path:=$HOME/go2_maps/latest/GlobalMap.pcd \
    registration_backend:=cuda gpu_device_id:=0 use_rviz:=true
```

与 NDT 同时运行 LIO-SAM 时，必须使用
`ros2 launch lio_sam lidar.launch.py publish_map_to_odom:=false`，确保
`map -> odom` 只由 NDT 发布；`simdog/start.sh` 会自动选择正确所有者。

Gazebo Classic 图形界面可能受 NVIDIA 驱动与 OGRE 兼容性影响，默认使用 `gui:=false`。确需回退时先设置 `GO2_FORCE_NVIDIA_RENDERING=0`，再按需使用 `LIBGL_ALWAYS_SOFTWARE=1`。

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
- Nav2 和 SLAM Toolbox 依赖已安装，但 Go2 的完整自主导航参数尚未完成调优，不能视为开箱即用。
- LIO-SAM 当前关闭回环检测；正式地图需要在目标场景重新采集并评估质量。
- NDT 使用前必须提供有效 `GlobalMap.pcd`，并通过 `/initialpose` 给出合理初始位姿。
- 根目录已配置 Git，默认分支为 `main`，远端为 `origin`；提交前仍须核对工作区，避免混入用户的无关修改。

## 提交与合并请求

提交使用简短中文祈使句和范围，例如 `go2_config: 调整雷达坐标系`，每次提交只处理一个主题。推送前执行 `git status --short` 和与改动风险相称的验证。合并请求应说明受影响包、构建与启动验证结果、修改的 ROS 话题或坐标系；涉及可视化时附 RViz 或 Gazebo 截图。

## 项目记忆维护

将当前项目状态统一维护在根目录 `PROJECT_MEMORY.md`。每次完成一个可验证阶段、修改启动入口、变更关键参数、调整硬件基线或发现运行限制后，必须同步更新：日期、阶段目标、实际操作与结果、验证命令、遗留问题和下一步。该文件只保留当前准确状态和必要的阶段记录；新信息应合并或替换过期条目，不创建 `PROJECT_MEMORY_v2.md` 等旧版本副本。
