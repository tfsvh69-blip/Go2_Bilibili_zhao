# go2_navigation

Go2 室内平地建图与导航包。统一入口默认运行 Slam Toolbox 在线建图导航；固定二维图
默认使用 Nav2 AMCL；`lidar_ndt` 与 `ndt_cuda` 仅作为三维定位实验档。规划和控制直接
复用 Nav2 Humble 的 SmacPlanner2D、Regulated Pure Pursuit、Collision Monitor 与
生命周期管理器。

## 三类地图不是同一个东西

| 数据 | 话题/文件 | 是否随探索扩展 | 用途 |
|---|---|---:|---|
| 在线 SLAM 地图 | `/map`、`slam.posegraph/.data` | 是 | 学习未知环境、续建 |
| 固定二维地图 | `map.yaml` + `map.pgm`、`/map` | 否 | AMCL 定位与全局规划 |
| 三维定位地图 | `GlobalMap.pcd`、`/global_map` | 否 | NDT/GICP 实验定位 |
| 全局/局部代价图 | Nav2 costmap 话题 | 动态更新但不保存 | 规划与避障 |

Velodyne 与深度相机不会让 `map_server` 加载的静态地图扩大。要在 RViz 中探索到新区域，
必须选 `navigation_mode:=online_slam`。

AMCL 只需要 Slam Toolbox 原生保存的 `map.yaml + map.pgm`，不要求 `GlobalMap.pcd`。
只有 `lidar_ndt/ndt_cuda` 实验档才要求 PCD 与二维图同源。不要把 LIO-SAM 三维 PCD
简单按高度差投影成 AMCL 主地图：地面、墙面、家具和错层点会在二维格中重叠，容易产生
黑色散点和重复结构，进而使 AMCL 位姿跳变。

## 启动

```bash
# 默认入口：新建在线地图
ros2 launch go2_navigation simulation_navigation.launch.xml \
    navigation_mode:=online_slam map_session:=new rviz:=true

# 载入已有 pose graph 续建；参数是包含 slam.posegraph/slam.data 的会话目录
ros2 launch go2_navigation simulation_navigation.launch.xml \
    navigation_mode:=online_slam map_session:=$HOME/go2_maps/online/latest \
    rviz:=true

# 固定二维图 + AMCL
ros2 launch go2_navigation simulation_navigation.launch.xml \
    navigation_mode:=static_map map_dir:=$HOME/go2_maps/online/latest \
    localization:=amcl rviz:=true

# 固定图 + NDT/二维 EKF 实验档
ros2 launch go2_navigation simulation_navigation.launch.xml \
    navigation_mode:=static_map map_dir:=$HOME/go2_maps/latest \
    localization:=lidar_ndt rviz:=true
```

`simulation_online_mapping_navigation.launch.xml` 是旧命令的兼容 wrapper；新代码只使用
统一入口。`static_bundle` 是 `static_map` 的弃用别名，启动时会打印迁移提示。
上述入口默认打开 Gazebo GUI；只有自动化或性能测试才传 `gui:=false`。

在线保存：

```bash
bash simdog/src/go2_navigation/scripts/save_online_map.sh learning_room
```

保存完成后可直接把输出会话目录用作固定 AMCL 地图：

```bash
ros2 launch go2_navigation simulation_navigation.launch.xml \
    navigation_mode:=static_map localization:=amcl \
    map_dir:=$HOME/go2_maps/online/learning_room rviz:=true
```

`save_online_map.sh` 还会原子更新 `~/go2_maps/online/latest` 软链接，因此日常固定
AMCL 可以使用该路径；需要复现实验时应写明确的会话目录。不要混淆
`~/go2_maps/online/latest` 与 LIO-SAM/NDT 流程使用的 `~/go2_maps/latest`。固定模式
不会自动挑选“质量最好”的地图，`map_dir` 指向哪一目录，就加载哪一目录的
`map.yaml/map.pgm`。启动后可用下列命令核实实际来源：

```bash
ros2 param get /map_server yaml_filename
ros2 lifecycle get /map_server                 # 期望 active [3]
```

在线模式传 `map_session:=new` 会创建空白 pose graph，不会读取任何旧会话；传入具体
会话目录才表示续建。

## RViz 操作

遇到机器人抖动、RViz 红项或目标不取消时，先点击 `Navigation 2 -> Cancel`。仍未停止则：

```bash
ros2 service call /navigation/stop std_srvs/srv/Trigger "{}"
# 排障并确认健康后解除锁存；旧目标不会自动续行
ros2 service call /navigation/resume std_srvs/srv/Trigger "{}"
```

在线模式以 `Slam Toolbox` 面板为主，地图会随观测扩大；`Navigation 2` 面板仅负责目标
反馈和取消，因此其中 `Localization: inactive` 不代表 Slam Toolbox 失效。固定图模式先在
工具栏点 `2D Pose Estimate` 设置初值，确认 `Navigation: active`、
`Localization: active`，再点 `Nav2 Goal`。固定图的 `/map` 边界不会改变。

`AMCL Pose` 前方黄色扇形表示 yaw 协方差，紫色椭圆表示 x/y 协方差；它们持续变大或
横穿地图表示定位已失信，并非雷达视野。安全监督会在位置标准差超过 0.75 m 或航向
标准差超过 0.75 rad 时锁速。此时先取消目标/调用 `/navigation/stop`，再用
`2D Pose Estimate` 在真实位置拖出准确朝向；协方差回落后再恢复导航。

原始 Velodyne 点云、Global Costmap 和 Local Costmap 默认关闭以降低 RViz 负载。排查障碍时
按需勾选；代价图会动态变化，但它们不是建图结果。

## 感知、TF 与控制链

VLP-16 水平切片由 `pointcloud_to_laserscan` 统一生成 `/scan`。AMCL、Slam Toolbox、
全局障碍层、局部障碍层和 Collision Monitor 都复用这条 LaserScan；局部代价图和碰撞监控
还订阅实际 D435 话题 `/depth/color/points`，并按高度过滤地面和机身点。

TF 所有权必须唯一：

| 模式 | `map -> odom` 发布者 |
|---|---|
| `online_slam` | `slam_toolbox` |
| `static_map + amcl` | `amcl` |
| `static_map + lidar_ndt` | `robot_localization` 二维 EKF |
| `static_map + ndt_cuda` | CUDA NDT 实验节点 |

所有二维导航节点使用 `base_footprint`，完整主链为
`map -> odom -> base_footprint -> base_link -> sensors`。仿真统一入口用真值里程计适配器
提供平面的 `odom -> base_footprint`，这只证明 Gazebo 闭环，不代表真机里程计已验证。

速度链保持：

```text
Nav2 / 键盘 / Unitree Move -> twist_mux -> velocity_smoother
                             -> collision_monitor -> /cmd_vel -> CHAMP
```

RPP 默认速度上限为 `0.27 m/s`，接近目标的最低速度为 `0.10 m/s`，普通目标容差为
`0.30 m / 0.25 rad`。`PoseProgressChecker` 将 `0.10 m` 平移或 `0.15 rad`
转向都视为进展。RPP 原地对齐阈值为 `0.85 rad`，普通弯道由控制器
连续画弧跟随。控制频率 10 Hz，RPP 内部碰撞预测和 Collision Monitor 均保留。

每次全局计划后行为树会调用 Nav2 `SmoothPath(SimpleSmoother)`；平滑结果会做
碰撞检查，可选平滑失败时保留原路径，不直接中止导航。用下列命令打开
Gazebo、RViz 和标准动态参数窗口：

```bash
ros2 launch go2_navigation simulation_navigation.launch.xml \
    navigation_mode:=static_map localization:=amcl \
    map_dir:=$HOME/go2_maps/online/latest tuning_gui:=true
```

`rqt_reconfigure` 的改动只对本次进程生效，重启会恢复 YAML。不要动态修改控制器
插件类型、控制频率、Collision Monitor 与安全锁速参数。
安全监督使用 `HEALTHY/DEGRADED/LOST` 滞回：单帧 NDT 拒绝只进入降级，连续失效 2 秒或
明确重定位请求才锁速；NDT 需连续 5 个健康样本、fitness 不高于 5.25 才清锁。

导航栈运行时不得让 `teleop_twist_keyboard` 直接发布最终 `/cmd_vel`，否则它会与
Collision Monitor 争用执行器并绕过安全链。键盘测试必须使用：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/cmd_vel_teleop
```

## 目标接口与健康检查

公开 action 仍是 `/navigate_to_pose`，异步目标门禁转发到标准 `nav2_bt_navigator` 的
`/navigate_to_pose_raw`。取消会向内部 goal 传播。门禁拒绝越界、障碍、非有限坐标和定位
失效目标，不再用轮询线程忙等。

```bash
# 在线模式
ros2 run go2_navigation health_check --mode online_slam --localization amcl

# 固定 AMCL
ros2 run go2_navigation health_check --mode static_map --localization amcl \
    --map-dir $HOME/go2_maps/online/latest
```

预期 `/scan` 持续发布，`/pause_navigation` 为 `false`，TF 主链完整，公开与内部 action
各只有一个 server。Gazebo 高负载时传感器可能低于 xacro 标称 10 Hz；本机无界面实测约
6–7 Hz，因此传感器过期联调值记录为 2 秒，后续应在场景压力测试后逐步收紧。

## 上游依据、许可证与取舍

- [Navigation2](https://github.com/ros-navigation/navigation2) 提供 AMCL、生命周期管理、
  SmacPlanner2D、RPP 和 Collision Monitor，Apache-2.0；本项目使用 Humble 系统包，不复制
  规划器或控制器实现。
- [Slam Toolbox](https://github.com/SteveMacenski/slam_toolbox) 为 LGPL-2.1，
  `pointcloud_to_laserscan` 为 BSD；在线模式复用其标准节点、RViz 插件和 pose graph。
- `lidar_localization_ros2` 为 BSD-2-Clause；实验档关闭其直接 TF 输出，以
  `robot_localization`（BSD-3-Clause）的 `two_d_mode` 平滑发布全局二维 TF。
- Go2 社区方案多数是通用导航栈的适配：`go2_robot` 导航仍在开发，
  `unitree_go2_nav` 面向 Jazzy/RTAB-Map，`autonomy_stack_go2` 使用 Point-LIO 与独立自主栈。
  它们与当前 Humble + Gazebo Classic + CHAMP 版本和控制链不直接兼容，因此只吸收架构经验，
  不整体替换当前工作区。

## 当前验证边界

已实测：默认在线模式冷启动、唯一 TF 所有者、平面 TF、AMCL 与 map_server 生命周期 active、
12 次连续短目标全部成功、在线地图尺寸随移动增长、辅助 Python 节点 CPU 明显下降。

尚未实测：固定 AMCL 的 12 个完整目标与 60 秒协方差指标、10 分钟 RViz 压力、移动障碍、
断开各传感器/节点的完整失效矩阵、真机闭环。不得把仿真结果描述为真机验证。
