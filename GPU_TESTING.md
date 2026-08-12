# GPU 验证与维护指南

## 默认策略

项目在检测到 NVIDIA GPU 和 CUDA 12.8 后默认采用以下策略：

- 阶段一起默认定位改用 `lidar_localization_ros2` 的 CPU NDT_OMP；
  CUDA NDT（`ndt_relocalization`）保留为 `localization:=ndt_cuda` 实验档，
  使用 `registration_backend:=cuda`。
- `ndt_relocalization` 实验档默认使用 `registration_backend:=cuda`。
- `scripts/setup_simdog.bash` 默认设置 `CUDA_VISIBLE_DEVICES=0`。
- 在双显卡笔记本上默认设置 `__NV_PRIME_RENDER_OFFLOAD=1` 和
  `__GLX_VENDOR_LIBRARY_NAME=nvidia`，让 Gazebo、RViz2 和其他 OpenGL
  程序优先使用 NVIDIA GPU。
- `simdog/start.sh` 为其打开的所有终端设置同样的 GPU 环境，并显式以 CUDA
  后端启动 NDT。
- `scripts/build_workspaces.sh` 检测到 CUDA 12.8 时自动构建 `sm_89`
  GPU NDT；否则构建保留 CPU 回退。

当前真正使用 CUDA 计算的是 NDT 点云配准。Gazebo/RViz2 使用 GPU 做 OpenGL
渲染；LIO-SAM 的 GTSAM 图优化、Gazebo 物理、CHAMP 和多数 PCL 预处理仍使用
CPU，不能通过环境变量自动变成 GPU 算法。

## 一键端到端验证

在项目根目录执行：

```bash
cd /home/hao/ROS/Go2_Bilibili_zhao-main
bash scripts/verify_gpu_runtime.sh
```

脚本会自动执行以下操作：

1. 使用独立 `ROS_DOMAIN_ID`，避免当前 Gazebo 的 `/clock` 和 TF 干扰验证。
2. 检查 RTX 4060、驱动、CUDA 12.8、CUDA 动态库和 `sm_89` 内核。
3. 加载项目默认 GPU 环境。
4. 用仓库自带 PCD 启动 3 级 CUDA NDT。
5. 注入 `odom -> base_link` TF 和测试点云。
6. 确认 NVIDIA 计算进程中存在 `ndt_relocalization_node`。
7. 检查 `/ndt_pose` 并自动停止全部测试进程。

默认验证域根据脚本进程号在 100–199 之间选择；如需固定，可设置
`GO2_VERIFY_ROS_DOMAIN_ID`。该隔离只作用于验证脚本及其子进程，不影响已经
运行的仿真。

通过时应看到类似输出：

```text
CUDA NDT enabled on device 0: NVIDIA GeForce RTX 4060 Laptop GPU (compute 8.9)
Registration backend: cuda
Created 3 multi-resolution CUDA NDT objects
NDT GPU 进程：... ndt_relocalization_node ...
[6/6] 验证通过
```

测试点云只来自 `simdog/src/fast_gicp/data/`，不会写入
`~/go2_maps/latest`，也不会覆盖正式地图。由于每次 CUDA 核函数运行很短，
脚本的低频采样可能显示较低瞬时利用率；计算进程、CUDA 启动日志和
`/ndt_pose` 共同作为端到端验证依据。

## 验证正式仿真

终端一：

```bash
cd /home/hao/ROS/Go2_Bilibili_zhao-main
source scripts/setup_simdog.bash
ros2 launch go2_config gazebo_velodyne.launch.py gui:=false rviz:=true
```

终端二：

```bash
cd /home/hao/ROS/Go2_Bilibili_zhao-main
source scripts/setup_simdog.bash
ros2 launch ndt_relocalization ndt_localization.launch.py \
    map_path:=$HOME/go2_maps/latest/GlobalMap.pcd \
    registration_backend:=cuda gpu_device_id:=0 use_rviz:=true
```

终端三检查：

```bash
nvidia-smi dmon -s pucm
nvidia-smi --query-compute-apps=pid,process_name,used_memory \
    --format=csv,noheader
ros2 topic echo --once /ndt_pose
ros2 topic hz /ndt_pose
ros2 param get /ndt_relocalization_node registration_backend
```

判定标准：

- NDT 日志显示 `Registration backend: cuda`。
- NVIDIA 计算进程中出现 `ndt_relocalization_node`。
- `/ndt_pose` 持续输出，且位置不会出现 `nan` 或 `inf`。
- `nvidia-smi dmon` 的 `sm`、显存或功耗在点云到达时发生变化。

## GPU 压力测试

需要观察明显 GPU 峰值时，可运行 `fast_gicp` 自带完整基准：

```bash
cd /home/hao/ROS/Go2_Bilibili_zhao-main
source scripts/setup_simdog.bash
simdog/build/fast_gicp/gicp_align \
    simdog/src/fast_gicp/data/251370668.pcd \
    simdog/src/fast_gicp/data/251371071.pcd
```

同时在另一终端运行：

```bash
nvidia-smi dmon -s pucm
```

该基准会先运行多个 CPU 算法，再运行 `ndt_cuda` 和 `vgicp_cuda`，整体可能持续
一至两分钟。2026-08-05 的验证结果为 CUDA D2D-NDT 单次约 `4.36 ms`，
GPU SM 峰值采样约 `74%`。

## 切换设备与回退

选择另一张物理 GPU：

```bash
export GO2_GPU_DEVICE=1
export CUDA_VISIBLE_DEVICES=1
source scripts/setup_simdog.bash
```

强制使用 CPU NDT：

```bash
ros2 launch ndt_relocalization ndt_localization.launch.py \
    map_path:=$HOME/go2_maps/latest/GlobalMap.pcd \
    registration_backend:=omp
```

如果 NVIDIA OpenGL 在当前桌面环境异常，可只关闭强制 NVIDIA 渲染：

```bash
export GO2_FORCE_NVIDIA_RENDERING=0
source scripts/setup_simdog.bash
```

仅在驱动渲染确实不可用时才启用软件渲染：

```bash
export GO2_FORCE_NVIDIA_RENDERING=0
source scripts/setup_simdog.bash
export LIBGL_ALWAYS_SOFTWARE=1
```

## 维护检查清单

更换 GPU、CUDA 版本、驱动或更新 `fast_gicp` 后，依次执行：

```bash
bash scripts/install_gpu_dependencies.sh
bash scripts/build_workspaces.sh
bash scripts/verify_gpu_runtime.sh
```

并核对：

- `nvidia-smi` 能识别目标 GPU。
- `nvcc --version` 与构建脚本中的 CUDA 路径一致。
- `cuobjdump --list-elf` 中包含目标 GPU 对应的 `sm_*`。
- NDT 节点仍链接 `libcudart.so` 和 `libfast_vgicp_cuda.so`。
- CUDA 与 `omp` 后端都能构建，CUDA 不可用时回退不崩溃。
- 将实际版本、性能结果、已知问题和验证命令同步到 `PROJECT_MEMORY.md`。
