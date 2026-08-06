# simdog 主仿真工作空间

`simdog` 是本项目的主 ROS 2 Humble 工作空间，包含 CHAMP 四足步态、Go2
Gazebo 模型、Velodyne、RealSense、LIO-SAM 和 NDT 重定位。

## 配置与构建

请从项目根目录统一安装依赖和构建：

```bash
cd /home/hao/ROS/Go2_Bilibili_zhao-main
bash scripts/install_dependencies.sh
bash scripts/install_gpu_dependencies.sh
bash scripts/build_workspaces.sh
source scripts/setup_simdog.bash
```

环境加载脚本默认设置 CUDA 设备 0，并在双显卡环境优先使用 NVIDIA OpenGL。
如需确认整个 CUDA NDT 链路，执行：

```bash
bash scripts/verify_gpu_runtime.sh
```

## 启动

完整操作分为“首次建图”和“已有地图重定位”两种模式，不能让 LIO-SAM 与 NDT 同时
发布 `map -> odom`。详细的逐终端命令、地图保存、`/initialpose` 和故障排查见
根目录 [README.md](../README.md#启动完整四足仿真)。

首次建图时，在三个已加载环境的终端中分别运行：

```bash
ros2 launch go2_config gazebo_velodyne.launch.py gui:=false rviz:=true
ros2 launch lio_sam lidar.launch.py rviz:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

已有 `~/go2_maps/latest/GlobalMap.pcd` 时，改用重定位模式：

```bash
ros2 launch go2_config gazebo_velodyne.launch.py gui:=false rviz:=true
ros2 launch lio_sam lidar.launch.py rviz:=true publish_map_to_odom:=false
ros2 launch ndt_relocalization ndt_localization.launch.py \
    map_path:=$HOME/go2_maps/latest/GlobalMap.pcd \
    registration_backend:=cuda gpu_device_id:=0 use_rviz:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

每条命令各占一个终端。桌面环境也可从项目根目录执行 `bash simdog/start.sh`
自动打开多个终端：没有地图时进入建图模式，有地图时自动切换到 NDT 重定位模式。

## 地图与重定位

LIO-SAM 运行期间执行：

```bash
bash simdog/save_Map.sh
```

默认保存到 `~/go2_maps/latest`，也可提供目标目录和分辨率：

```bash
bash simdog/save_Map.sh ~/go2_maps/warehouse 0.2
```

关闭建图模式的 LIO-SAM 后，再启动重定位模式：

```bash
ros2 launch lio_sam lidar.launch.py rviz:=true publish_map_to_odom:=false
ros2 launch ndt_relocalization ndt_localization.launch.py \
    map_path:=$HOME/go2_maps/warehouse/GlobalMap.pcd \
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

- 修改 LIO-SAM 参数：`src/LIO-SAM/config/params.yaml`。
- 修改 Go2 仿真世界或启动项：`src/unitree-go2-ros2/robots/configs/go2_config/`。
- 修改 xacro 后通常只需重新启动；修改 C++ 后执行
  `colcon build --symlink-install --packages-select <包名>`。
- Gazebo GUI 若受显卡影响，继续使用 `gui:=false`。只有默认 NVIDIA 渲染
  确实异常时，才设置 `GO2_FORCE_NVIDIA_RENDERING=0` 并按需启用
  `LIBGL_ALWAYS_SOFTWARE=1`。
