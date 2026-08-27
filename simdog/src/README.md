# `simdog/src` 源码导航

这里按职责分组，但每个 ROS 2 包仍保持独立的 `package.xml`、包名和公开接口。
`colcon` 会递归发现这些包；日常构建应按包名选择，不依赖源码目录深度。

```text
src/
├── go2/             # 本项目直接维护的 Go2 功能与系统集成
├── platform/        # 机器人模型、CHAMP、Gazebo、ros2_control、Unitree 接口
├── localization/    # 建图、定位及重定位实现
└── vendor/          # 可替换的通用上游组件，保留许可证与上游边界
```

## 数据与控制链

```text
platform: Gazebo/Go2/VLP-16
          │ /velodyne_points、/imu/data、/odom
          ▼
go2_lidar_scan ── /scan ──► Slam Toolbox / AMCL / Nav2
                                      │ /cmd_vel_nav
                                      ▼
twist_mux → velocity_smoother → collision_monitor → /cmd_vel
                                      │
                                      ▼
platform: CHAMP → ros2_control → Gazebo
```

定位实验包通过公开话题和 TF 接入导航，不应被在线 Slam Toolbox 默认流程强制依赖。
Unitree bridge 只负责接口适配，不在桥内实现导航、点云投影或四足步态算法。

## 分组说明

| 分组 | 包 | 主要职责 |
|---|---|---|
| `go2/` | `go2_navigation` | 统一建图/固定图导航入口、Nav2、安全链与健康检查 |
| `go2/` | `go2_lidar_scan` | 重力对齐、唯一 `/scan` 参数源、诊断和运动探针 |
| `go2/` | `go2_behaviors` | 仿真动作与 CHAMP 控制权串行切换 |
| `go2/` | `go2_unitree_sim_bridge` | Unitree Sport API 与仿真话题/服务适配 |
| `go2/` | `go2_navigation_bt_plugins` | Nav2 行为树扩展插件 |
| `platform/` | `unitree-go2-ros2` | Go2 模型、CHAMP、Gazebo 世界和 `ros2_control` |
| `platform/` | `unitree_ros2_interfaces` | Unitree v0.3.0 消息接口快照 |
| `localization/` | `LIO-SAM` | 三维激光惯性建图 |
| `localization/` | `lidar_localization_ros2` | NDT/GICP 定位实验与工具 |
| `localization/` | `ndt_relocalization` | PCD 地图 NDT 重定位节点 |
| `vendor/` | `pointcloud_to_laserscan` | 点云投影为 `LaserScan`；本项目仅维护必要的动态高度补丁 |
| `vendor/` | `fast_gicp`、`ndt_omp_ros2` | CUDA/OpenMP 点云配准后端 |
| `vendor/` | `realsense_ros_gazebo` | RealSense Gazebo 仿真插件 |

## 维护边界

1. 新的 Go2 业务逻辑优先放在 `go2/` 的现有包中；只有职责和生命周期明确不同才新建包。
2. 包之间通过 ROS 话题、服务、Action、TF 和参数契约协作，避免跨包导入源码内部模块。
3. `vendor/` 只做 Humble、CUDA 或本项目接口所需的小补丁；修改时保留许可证，并在包文档或
   `PROJECT_MEMORY.md` 记录与上游的差异。
4. `localization/` 是可选实验能力，在线二维建图不得依赖 PCD/NDT 才能启动。
5. 不要按目录路径选择构建目标；使用 `colcon build --packages-select <包名>`。

快速核对递归发现结果：

```bash
cd simdog
colcon list
colcon graph
```
