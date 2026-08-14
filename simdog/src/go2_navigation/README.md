# go2_navigation

Go2 室内平地建图与导航包。统一入口默认运行 Slam Toolbox 在线建图导航；固定二维图
默认使用 Nav2 AMCL；`lidar_ndt` 与 `ndt_cuda` 仅作为三维定位实验档。规划和控制直接
复用 Nav2 Humble 的 SmacPlanner2D、Rotation Shim、Regulated Pure Pursuit、
Collision Monitor 与生命周期管理器。

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

## 运行时安全调参与监控

导航启动并健康后，在另一个已加载 `simdog` 的终端运行：

```bash
# curses 综合界面；非交互终端自动使用文本命令行
ros2 run go2_navigation nav_tuner

# 只监控，或者采样 10 秒后输出一次 JSON
ros2 run go2_navigation nav_tuner --monitor-only
ros2 run go2_navigation nav_tuner --snapshot --sample-seconds 10

# 不依赖 console entry point 的源码兼容入口
python3 simdog/src/go2_navigation/tools/nav_tuner.py
```

界面同时显示 `/scan` 与 D435 频率/年龄、scan 有效/inf/nan 数与最近距离、全局/局部
costmap 的 lethal/inflated 格、机器人到最近 lethal 格距离、实际发布 footprint、路径长度、
1 Hz 重规划间隔、按半格加密后的保守 clearance，以及
`/cmd_vel_nav → /cmd_vel_switched → /cmd_vel_smoothed → /cmd_vel` 四级速度。

可用命令：

```text
show [alias]
set <alias> <value>
reset <alias>
reset-group <group>
profile safe|balanced|aggressive
save
record [key=value ...]
help
quit
```

工具把参数分成 `LIVE`、`LIFECYCLE RELOAD`、`RESTART REQUIRED`。ObstacleLayer 的
source 子参数虽然可被参数服务接受，但 active 插件不会刷新 observation buffer；工具会
先停车，再通过 Lifecycle RESET/STARTUP 重建插件，确认全部 active 后才恢复输入，旧目标
不会续行。插件列表、插件类型、observation source 结构和 BT 的 1 Hz 频率会被明确拒绝，
不会伪装成热更新。`safe/balanced/aggressive` 在完整障碍安全验收前均显示
`UNCALIBRATED`，不得使用。

`save` 只定点替换注册表管理的 YAML 标量。写入前备份到
`logs/backups/<timestamp>/`；任一文件失败会回滚全部文件并输出 unified diff。完整别名、
类型、单位、YAML 归属、能力分类、实测证据和故障分支见
[运行时参数能力矩阵](docs/nav2_runtime_parameter_matrix.md)。标准 `rqt_reconfigure` 仍保留为
辅助 GUI，但不负责能力判断、reload、效果验证、实验记录或安全保存。

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

### Local Costmap 已勾选但仍空白

Local Costmap 是以机器人为中心的 `5×5 m` 滚动窗口，不是另一张会扩大的地图。先在
RViz 放大机器人附近，并暂时只保留 `Local Costmap`，关闭 `Static Map` 和
`Global Costmap`，避免多个 Map Display 在同一平面相互覆盖。随后检查：

```bash
ros2 lifecycle get /controller_server
ros2 topic hz /scan
ros2 param get /local_costmap/local_costmap scan_layer.scan.max_obstacle_height
ros2 param get /global_costmap/global_costmap obstacle_layer.scan.max_obstacle_height
ros2 topic hz /local_costmap/costmap_updates
```

预期控制器为 `active [3]`、`/scan` 持续发布，两个 source 级高度上限均为 `2.0`，
且局部代价图有更新。Nav2 Humble 的 `<obstacle layer>.<data source>.max_obstacle_height`
默认是 `0.0 m`，它不会继承外层 ObstacleLayer 的 `2.0 m`。若这里回读为 `0.0`，
位于雷达实际高度的 LaserScan 端点会在进入代价图前被全部过滤，表现就是
“话题和插件都存在，但局部图全空白”。上游依据见
[Obstacle Layer 参数说明](https://docs.nav2.org/configuration/packages/costmap-plugins/obstacle.html)
与 [Nav2 Humble 源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/obstacle_layer.cpp)。

默认 RPP 不生成另一条可视化“局部路径”。`/received_global_plan` 是交给控制器的平滑
全局路径；RPP 用 Local Costmap 预测当前追踪弧是否碰撞，行为树则在终点 `0.30 m`
锁存区外以 `1 Hz` 重新发布 `/plan`。动态障碍测试应把障碍放在机器人前方至少约
`1 m` 且有绕行空间的位置：正常现象是局部图先标记障碍，随后全局路径在约 1 秒内改变；
通道完全封死时应安全减速或停车，而不是继续撞击。

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

默认 `forward_rpp` 的外层控制器是
`nav2_rotation_shim_controller::RotationShimController`，其
`primary_controller` 仍为 RPP。RPP 以不超过 `0.27 m/s` 跟踪路径，接近目标的最低速度为
`0.10 m/s`；进入普通目标的 `0.30 m` XY 容差后，shim 保持 `linear.x=0`，以
`0.45 rad/s`、最大 `1.0 rad/s²` 对齐到 `0.15 rad` yaw 容差。shim 使用限加速度开环命令，
路径进入/退出阈值为 `1.40/0.40 rad`，前向采样 `0.50 m`，旋转碰撞预测 `1.0 s`。
内部 RPP 的 `use_rotate_to_heading=false`：普通弯道保持前进画弧，只有接近侧后方的路径
由外层 shim 停车对齐，避免外层 shim 与内层 RPP 重复决定原地旋转。
`PoseProgressChecker` 将 `0.10 m` 平移或 `0.15 rad` 转向都视为进展；RPP 和 shim 的
碰撞预测以及 Collision Monitor 均保留。

`closed_loop=false` 只是不把当前 1 秒滑窗 `/odom.twist.angular.z` 当作 Rotation Shim 的
低延迟角加速度反馈；机器人是否达到目标 yaw 仍由实时 TF 与 GoalChecker 闭环判定，
并不等于关闭导航闭环。

Humble 的 Rotation Shim 会在每次 `setPlan()` 时重置内部终点位置检查器。行为树仍以
1 Hz 重规划，但 `TerminalPathLatch` 保留内部 `RateController` 的计时状态，不再在规划
成功后把子树重置成“首次运行”。它只接受与原始目标 frame 相同、末端位置误差不超过
`0.075 m`、末端 yaw 误差不超过 `0.01 rad` 的新路径，并且实时
`map→base_footprint` TF 确认机器人同时位于原始目标和路径末端 `0.30 m` 内时才锁存。
锁存后短时漂出边界不会恢复重规划；新目标、行为树 `halt()` 或 recovery 会清除状态，
因此同一 XY 只改变 yaw 也必须先生成新路径。未锁存时若 TF 暂不可用，节点输出限频诊断
并继续规划，不复用旧目标状态。`GridBased.tolerance=0.0`，不可达原始目标明确失败，
不会再选择附近替代终点；`use_final_approach_orientation=false` 保留 RViz 指定的末端 yaw。

```bash
ros2 param get /controller_server FollowPath.primary_controller
ros2 param get /controller_server FollowPath.rotate_to_goal_heading
ros2 param get /controller_server FollowPath.closed_loop
```

### 纯旋转与终点定向诊断

先在 RViz `Navigation 2 → Cancel` 取消活动目标，确认 `/pause_navigation=false`，并在距离
障碍至少 `0.8 m` 的位置测试。导航栈运行时只能从 `/cmd_vel_teleop` 注入手动命令，禁止
绕过 Collision Monitor 直接发布 `/cmd_vel`。两个终端分别执行：

```bash
# 终端 A：只读采样四级速度、真值、odom 和 TF
ros2 run go2_navigation rotation_diagnostics \
    --mode manual --duration 10 --expected-wz 0.45

# 终端 B：安全链纯旋转；还应依次测试 ±0.15、±0.25、±0.35、±0.45
timeout 5 ros2 topic pub -r 20 /cmd_vel_teleop geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.45}}"
```

发送目标前或导航途中启动：

```bash
ros2 run go2_navigation rotation_diagnostics \
    --mode navigation --acquire-timeout 120 --duration 10 \
    --xy-tolerance 0.30 --yaw-tolerance 0.15
```

Goal Guard 会把通过门禁的原始目标发布到 transient-local 只读话题
`/navigation/accepted_goal`。程序只订阅话题和查询 TF，不发布目标、速度或参数；它分别
给出机器人到原始目标、机器人到路径末端、路径末端到原始目标三组误差，以及终点前的
真实规划频率和锁存后的 `/plan` 数量。输出对应五层故障：控制器没产生命令；
`twist_mux`、速度平滑或 Collision Monitor 拦截；CHAMP 步态未执行；`odom`/TF 未反馈；
进入终点后 `/plan` 仍更新并重置状态。`map→odom` 单次修正超过 `0.10 m` 或
`0.10 rad` 时按地图/AMCL 故障处理，停止用控制参数补偿。
navigation 模式在收到新目标后重新计算 `--acquire-timeout`，等机器人同时进入原始目标和
路径末端的 XY 容差才开始 `--duration` 终点采样；action 成功后额外观察 1 秒。超时但仍
在途中时输出 `INCOMPLETE` 并返回 2，不再按终点误差判 FAIL。它还会统计终点外
`linear.x≈0` 的原地旋转片段、路径夹角和是否紧随 `/plan`。命令换行符 `\` 后不得有空格。

2026-08-13 的无界面同 XY `+90°` 目标在 `4.8 s` 内 `SUCCEEDED`：四级速度的终点段
`max|linear.x|=0`、`max|angular.z|=0.45 rad/s`、0 次换向，锁存后 `/plan=0`。
控制层专项检查通过；同次运行 AMCL 出现 `0.414 m` 单步修正，停稳后原始目标 XY 误差
约 `1.23 m`，所以固定图整体验收仍失败，必须先处理 `home_02` 地图/初始定位。

随后用户现场确认终点左右摆动已经消失。一次旧版 30 秒诊断结束时目标仍在
`5.401 m` 外，路径末端与原始目标仅差 `0.010 m`，`map→odom` 单步为
`0.020 m/0.009 rad`；这是采样尚未进入终点，不是定位或终点控制失败，正是新增
`INCOMPLETE` 状态要区分的情况。

2026-08-13 的本机纯旋转结果没有通过门槛，因此未继续执行新实现的 12 个导航目标。
`/cmd_vel` 与请求值误差不超过 `0.02 rad/s`，真值、`/odom` 和 TF 累计 yaw 误差不超过
`0.03 rad`，说明速度安全链与仿真反馈链正常；但 `+0.45 rad/s` 实际稳态增益约 `33%`，
`-0.45 rad/s` 约 `94%`，每 90° 等效平移漂移分别约 `0.38 m` 和 `0.13 m`。四脚
`mu1/mu2` 从 `0.6` 提高到 `1.0` 的单变量试验没有改善正向旋转，配置已恢复 `0.6`。
把 `stance_depth` 从 `0.01 m` 调为 `0.0 m` 后改善到约 `70%/61%` 与
`0.11/0.06 m/90°`，因此保留；官方 CHAMP Go1 仿真配置也使用 `0.0 m`。PID、
`stance_duration=0.20/0.30 s` 和 `swing_height=0.05 m` 的试验没有同时改善增益和漂移，
均已撤回。这组基线仍未完全通过，归入 CHAMP gait/Gazebo 接触层，不允许通过放宽
GoalChecker 或继续修改 Nav2 终点状态掩盖。

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
6–7 Hz，因此传感器过期联调值记录为 2 秒，后续应在场景压力测试后逐步收紧。导航档将
`pointcloud_to_laserscan.always_subscribe` 设为 `true`，避免临时 `/scan` 订阅者退出时
lazy 订阅竞态让转换节点停止处理 `/velodyne_points`；该参数在组件中默认仍为 `false`。

## 上游依据、许可证与取舍

- [Navigation2](https://github.com/ros-navigation/navigation2) 提供 AMCL、生命周期管理、
  SmacPlanner2D、RPP、Rotation Shim 和 Collision Monitor，Apache-2.0；本项目使用
  Humble 系统包，不复制规划器或控制器实现。终点策略依据
  [Rotation Shim 官方说明](https://docs.nav2.org/configuration/packages/configuring-rotation-shim-controller.html)
  与 [Humble RPP 源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp)。
- [Slam Toolbox](https://github.com/SteveMacenski/slam_toolbox) 为 LGPL-2.1，
  `pointcloud_to_laserscan` 为 BSD；在线模式复用其标准节点、RViz 插件和 pose graph。
- `lidar_localization_ros2` 为 BSD-2-Clause；实验档关闭其直接 TF 输出，以
  `robot_localization`（BSD-3-Clause）的 `two_d_mode` 平滑发布全局二维 TF。
- Go2 社区方案多数是通用导航栈的适配：`go2_robot` 导航仍在开发，
  `unitree_go2_nav` 面向 Jazzy/RTAB-Map，`autonomy_stack_go2` 使用 Point-LIO 与独立自主栈。
  它们与当前 Humble + Gazebo Classic + CHAMP 版本和控制链不直接兼容，因此只吸收架构经验，
  不整体替换当前工作区。
- [arjun-sadananda/go2_nav2_ros2](https://github.com/arjun-sadananda/go2_nav2_ros2)
  提醒了 CHAMP 状态估计可能存在里程计比例误差，但本项目导航 `/odom` 来自 Gazebo
  `/odom/ground_truth` 适配器，数据源不同，因此只借鉴分层排障思路，不复制其速度倍率补丁。
- [CHAMP robots 的 Go1 gait 基线](https://github.com/chvmp/robots/blob/master/configs/go1_config/config/gait/gait.yaml)
  使用 `stance_depth=0.0`、`stance_duration=0.25` 和 `swing_height=0.04`；仓库为
  BSD-3-Clause。当前 Go2 仅在纯旋转 A/B 证明 `stance_depth=0.0` 有收益后复用这一参数，
  没有整体替换模型或照搬里程计倍率。

## 当前验证边界

已实测：默认在线模式冷启动、唯一 TF 所有者、平面 TF、AMCL 与 map_server 生命周期 active、
在线档 12 次连续短目标全部成功、在线地图尺寸随移动增长、辅助 Python 节点 CPU 明显下降。
旧版锁存专项中，固定 `home_02` 连续两个内部 `/navigate_to_pose_raw` `±90°` 目标均
`SUCCEEDED`，但精确 `180°` 目标因 AMCL `map→odom` 跳变和实体漂移进入地图障碍附近
而未完成。当前实时 TF 锁存、0.45 rad/s 与 `closed_loop=false` 已完成自动化测试。
纯旋转双向基线已运行但因实体旋转增益不对称及漂移超标而失败；按分层验收规则，固定图
12 目标没有继续执行，不能沿用旧专项结果宣称新实现已实测通过。

尚未实测：固定 AMCL 的 12 个完整目标与 60 秒协方差指标、10 分钟 RViz 压力、移动障碍、
断开各传感器/节点的完整失效矩阵、真机闭环。不得把仿真结果描述为真机验证。
