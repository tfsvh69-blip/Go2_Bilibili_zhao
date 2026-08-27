# simdog 主仿真工作空间

`simdog` 是本项目的主 ROS 2 Humble 工作空间，包含 CHAMP 四足步态、Go2
Gazebo 模型、Velodyne、RealSense、LIO-SAM 和 NDT 重定位。
当前 `colcon list` 共识别 25 个包，其中包含新的 `go2_lidar_scan`、固定的 Unitree
ROS 2 v0.3.0 接口包与 Sport API 仿真兼容桥。

源码已按 `go2/`、`platform/`、`localization/`、`vendor/` 分层。先阅读
[源码导航](src/README.md) 可以看到每个包的职责、主数据链和允许的依赖方向；构建和运行
仍使用 ROS 包名，不依赖源码目录层级。

## 配置与构建

请从项目根目录统一安装依赖和构建：

```bash
cd /home/luhao/my/ROS/Go2_Bilibili_zhao-main
bash scripts/install_dependencies.sh
bash scripts/install_gpu_dependencies.sh
bash scripts/build_workspaces.sh
source scripts/setup_simdog.bash
```

运行 Unitree 接口仿真时改用 `source scripts/setup_unitree_sim.bash`；它会继续
加载本工作空间，并设置 CycloneDDS、`lo` 和默认 Domain 0。

环境加载脚本默认设置 CUDA 设备 0，并在双显卡环境优先使用 NVIDIA OpenGL。
如需确认整个 CUDA NDT 链路，执行：

```bash
bash scripts/verify_gpu_runtime.sh
```

## 启动

开始采集地图前，建议先独立核对 VLP-16 点云到 `/scan` 的转换：

```bash
GO2_D435_GAZEBO_ENABLED=0 \
ros2 launch go2_lidar_scan simulation_scan_debug.launch.xml \
  lidar_debug_raw_scan:=true tuning_gui:=true
```

它会启动 Gazebo、转换诊断和专用 RViz。确认 `Velodyne 3D Points`、橙色
`Leveled Navigation Scan` 与洋红色 `Raw Tilting Scan` 的墙体端点对应，雷达上方
`Scan Health` 为绿色后按 `Ctrl+C` 退出；完整
在线导航已经包含同一转换管线，两套入口不能同时运行。
历史运动样本在机身倾斜 `5.15°` 时旧投影产生 4430 个地面获胜端点，对齐扫描为 0，
`/scan=8.89 Hz`、同时间戳 TF 成功率 100%。900 水平列用于满足 Gazebo 实时率。人工曾在
较低高度窗复现扇形白线；用户现场确认 `+0.20..+0.30 m` 下 SLAM 目视正常，该值已固化
并继续支持 rqt 动态调节。完整覆盖建图、保存和固定图导航仍待验收；旧
`-0.05..+0.10 m` 由 `/scan_raw`
保留作对照。0.50 m 候选在后/右方向发生接触，默认量程
已恢复为全链一致的 0.90 m；详细数据见 [`go2_lidar_scan`](src/go2/go2_lidar_scan/README.md)。

完整操作分为“首次建图”和“已有地图重定位”两种模式，不能让 LIO-SAM 与 NDT 同时
发布 `map -> odom`。详细的逐终端命令、地图保存、`/initialpose` 和故障排查见
根目录 [README.md](../README.md#启动完整四足仿真)。

首次建图时，在三个已加载环境的终端中分别运行：

```bash
ros2 launch go2_config gazebo_velodyne.launch.py gui:=false rviz:=true
ros2 launch lio_sam lidar.launch.py rviz:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

已有 `$GO2_PROJECT_ROOT/go2_maps/latest/GlobalMap.pcd` 时，改用重定位模式：

```bash
ros2 launch go2_config gazebo_velodyne.launch.py gui:=false rviz:=true
ros2 launch lio_sam lidar.launch.py rviz:=true publish_map_to_odom:=false
ros2 launch ndt_relocalization ndt_localization.launch.py \
    map_path:=$GO2_PROJECT_ROOT/go2_maps/latest/GlobalMap.pcd \
    registration_backend:=cuda gpu_device_id:=0 use_rviz:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

每条命令各占一个终端。桌面环境也可从项目根目录执行 `bash simdog/start.sh`
自动打开多个终端：没有地图时进入建图模式，有地图时自动切换到 NDT 重定位模式。

## 仿真动作

启动完整四足 Gazebo 后，可在另一个已加载工作空间的终端执行：

```bash
ros2 run go2_behaviors go2_behavior hello
ros2 run go2_behaviors go2_behavior nod
ros2 run go2_behaviors go2_behavior stretch
ros2 run go2_behaviors go2_behavior lie
ros2 run go2_behaviors go2_behavior wave
ros2 run go2_behaviors go2_behavior dance
```

`lie` 会保持趴下并暂停 CHAMP，使用以下命令恢复：

```bash
ros2 run go2_behaviors go2_behavior stand
```

程序复用现有 CHAMP、`ros2_control` 和标准
`FollowJointTrajectory` 接口。动作期间由行为节点独占关节控制权，完成后自动恢复
CHAMP；不要并行执行多个动作或同时遥控。详细原理、许可证和真机边界见
[`go2_behaviors/README.md`](src/go2/go2_behaviors/README.md)。

行为服务端也可独立启动，并以服务串行动作：

```bash
ros2 run go2_behaviors go2_behavior_server
ros2 service call /go2_behaviors/hello std_srvs/srv/Trigger '{}'
ros2 service call /go2_behaviors/stop std_srvs/srv/Trigger '{}'
ros2 topic echo /go2_behaviors/status
```

## Unitree 接口兼容

`gazebo.launch.py` 和 `gazebo_velodyne.launch.py` 默认以
`unitree_bridge:=true` 启动兼容层。桥接发布 `/sportmodestate`、
`/lf/sportmodestate`、`/lowstate`、`/lf/lowstate`，并处理
`/api/sport/request`。支持 Move、Euler、站立/坐卧/恢复和现有 Hello、Stretch、
Dance1 动作；Move 生效时不得并行键盘遥控。

接口定义固定来自 Unitree 官方 `unitree_ros2 v0.3.0`，来源和许可证见
[`unitree_ros2_interfaces/README.md`](src/platform/unitree_ros2_interfaces/README.md)；
字段映射、错误码及未模拟边界见
[`go2_unitree_sim_bridge/README.md`](src/go2/go2_unitree_sim_bridge/README.md)。关闭桥接：

```bash
ros2 launch go2_config gazebo_velodyne.launch.py unitree_bridge:=false
```

真机只加载 `source scripts/setup_unitree_real.bash <网卡名>`，不得启动本仿真桥。

## 地图与重定位

LIO-SAM 运行期间执行：

```bash
bash simdog/save_Map.sh
```

默认保存到 `$GO2_PROJECT_ROOT/go2_maps/latest`，也可提供目标目录和分辨率：

```bash
bash simdog/save_Map.sh $GO2_PROJECT_ROOT/go2_maps/warehouse 0.2
```

关闭建图模式的 LIO-SAM 后，再启动重定位模式：

```bash
ros2 launch lio_sam lidar.launch.py rviz:=true publish_map_to_odom:=false
ros2 launch ndt_relocalization ndt_localization.launch.py \
    map_path:=$GO2_PROJECT_ROOT/go2_maps/warehouse/GlobalMap.pcd \
    registration_backend:=cuda gpu_device_id:=0 use_rviz:=true
```

以上两条命令应分终端运行。重定位模式必须将
`publish_map_to_odom:=false`，使 NDT 成为 `map -> odom` 的唯一发布者；
建图模式则保持 LIO-SAM 默认值 `true`。`simdog/start.sh` 会自动选择。

`registration_backend` 支持 `cuda` 和 `omp`，默认使用 `cuda`；CUDA 设备或
GPU 构建不可用时会自动退回 OpenMP。用 `nvidia-smi dmon -s pucm` 可观察
实时 GPU SM、显存和功耗。GPU 后端只加速 NDT 配准，LIO-SAM 图优化仍运行在
CPU 上。完整验证与维护流程见根目录 `GPU_TESTING.md`。

## 修改说明

- 修改 LIO-SAM 参数：`src/localization/LIO-SAM/config/params.yaml`。
- 修改 Go2 仿真世界或启动项：`src/platform/unitree-go2-ros2/robots/configs/go2_config/`。
- 修改 xacro 后通常只需重新启动；修改 C++ 后执行
  `colcon build --symlink-install --packages-select <包名>`。
- Gazebo GUI 若受显卡影响，继续使用 `gui:=false`。只有默认 NVIDIA 渲染
  确实异常时，才设置 `GO2_FORCE_NVIDIA_RENDERING=0` 并按需启用
  `LIBGL_ALWAYS_SOFTWARE=1`。
