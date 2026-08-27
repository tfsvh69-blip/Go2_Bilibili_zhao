# Go2 阶段一 · 导航链路测试手册

> 逐步验证「在线 SLAM/固定图 AMCL → Nav2 规划控制 → 安全控制链 → 端到端移动」。
> 目标系统：Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11。预计耗时约 1 小时。

> 当前默认是 `online_slam`，固定图默认是 AMCL。本手册中 NDT 章节只是
> `localization:=lidar_ndt|ndt_cuda` 实验档，不是普通固定图导航前置。

## 路径圆滑与动态调参快速验证

```bash
ros2 launch go2_navigation simulation_navigation.launch.xml \
    navigation_mode:=static_map localization:=amcl \
    map_dir:=$GO2_PROJECT_ROOT/go2_maps/online/latest \
    controller_profile:=forward_rpp tuning_gui:=true
```

> 默认档已是 `forward_mppi`；本节调的是 RPP 参数，因此显式传
> `controller_profile:=forward_rpp` 运行 RPP 对照档。

该命令默认打开 Gazebo GUI 和 RViz。RViz 中对照绿色 `Raw Global Plan`
与蓝色 `Controller Path (Smoothed)`；在 rqt 中选 `/controller_server`。只调
`desired_linear_vel`、lookahead、`rotate_to_heading_*`、`max_angular_accel`、
`min_approach_linear_velocity` 和 goal tolerance，一次一项。不得关闭碰撞与锁速保护。
参数修改只对当前进程生效，重启恢复 YAML 基线。

`$GO2_PROJECT_ROOT/go2_maps/online/latest` 由 `save_online_map.sh` 指向最近一次 Slam Toolbox
会话；当前机器上它指向 `home_02`。不要将它与 LIO-SAM/NDT 使用的
`$GO2_PROJECT_ROOT/go2_maps/latest` 混淆。启动后必须确认实际文件：

```bash
ros2 param get /map_server yaml_filename
```

预期为 `$GO2_PROJECT_ROOT/go2_maps/online/home_02/map.yaml`（以后重新保存地图时会变为新会话）。
若 RViz 仍显示旧图，关闭全部旧导航/RViz 进程后只重启这一套入口，并确认
`ros2 lifecycle get /map_server` 返回 `active [3]`。

## 测试地图

| 编号 | 测试 | 关键验证 | 耗时 |
|---|---|---|---|
| A | 前置准备 | 构建 / 环境 / 域 | 5 min |
| 1 | 地图包工具 | 离线校验 + 篡改检测 | 5 min |
| 2 | 传感器链路 | Velodyne / IMU / 控制器（D435 为可选感知层） | 10 min |
| 3 | 定位链路 | NDT 配准 / map→odom | 10 min |
| 4 | 完整导航栈 | 10 节点激活 / 控制链话题 | 15 min |
| 5 | 端到端导航 | 目标 → 路径 → 移动 | 20 min |
| 6 | 安全控制链 | 键盘 / 导航锁 | 10 min |
| 7 | 在线建图导航 | Live SLAM Map / 保存与续建 | 20 min |
| 8 | GPU 实验档 | CUDA NDT 验证 | 5 min |

---

## A · 前置准备

### 1. 确认构建产物

```bash
cd /home/luhao/my/ROS/Go2_Bilibili_zhao-main
bash scripts/build_workspaces.sh            # 全量构建（含 GPU 后端，较慢）
# 或增量构建受影响包：
#   source /opt/ros/humble/setup.bash
#   cd simdog && colcon build --symlink-install --packages-select \
#     go2_lidar_scan go2_navigation go2_description
```

**通过标准**：colcon 全部 Finished，无 failed；`cd simdog && colcon list | wc -l` 输出 `25`；`cmp -s AGENTS.md CLAUDE.md && echo OK` 输出 OK。

### 2. 环境加载（每个终端都要）

```bash
cd /home/luhao/my/ROS/Go2_Bilibili_zhao-main
source scripts/setup_unitree_sim.bash      # CycloneDDS + lo + Domain 0（项目标准）
# 不要 export ROS_DOMAIN_ID=141：所有 Gazebo、定位、Nav2、CLI 必须在同一域 0。
# 隔离验证只能在 source 前设置 GO2_UNITREE_SIM_DOMAIN_ID=<id>。
export GAZEBO_MASTER_URI=http://127.0.0.1:11441   # 仅启动 Gazebo 的终端需要
```

> ⚠️ CycloneDDS 的 lo 接口无多播，CLI 命令首次发现较慢（等 5–30 秒属正常）。这是项目为 Unitree 仿真隔离的标准配置，不是故障。

---

## 1 · 地图包工具（离线 · 最快）

验证同源地图包生成与完整性校验，不启动任何仿真。

### 校验现有地图包

```bash
ros2 run go2_navigation validate_map_bundle --map-dir $GO2_PROJECT_ROOT/go2_maps/latest
echo $?   # 期望 0
```

**预期输出**：`地图包校验通过：$GO2_PROJECT_ROOT/go2_maps/latest`。

### 篡改检测（模拟文件被改坏）

```bash
cp $GO2_PROJECT_ROOT/go2_maps/latest/map.pgm /tmp/map.pgm.bak
printf '\x00' | dd of=$GO2_PROJECT_ROOT/go2_maps/latest/map.pgm bs=1 seek=10 conv=notrunc
ros2 run go2_navigation validate_map_bundle --map-dir $GO2_PROJECT_ROOT/go2_maps/latest
echo $?   # 期望 1
cp /tmp/map.pgm.bak $GO2_PROJECT_ROOT/go2_maps/latest/map.pgm     # 恢复
```

**预期输出**：`校验失败：SHA-256 不匹配：…/map.pgm（地图可能已改动…）`。

### 从 PCD 重建地图包（可选，用最新建图结果）

```bash
ros2 run go2_navigation build_map_bundle --map-dir $GO2_PROJECT_ROOT/go2_maps/latest \
    --x-min -2 --x-max 8 --y-min -4 --y-max 4 --resolution 0.1
# 参数说明：
#   --x-min/--x-max/--y-min/--y-max  裁剪到导航区域（提高点密度）
#   --obstacle-height-m（默认 0.4，调大减少腿部误标）
#   --min-points-per-cell（默认 1，稀疏地图更宽容）
```

**预期输出**：打印「运行上游地图生成脚本…」→「已生成地图包清单：…map_bundle.yaml」→「生成文件：…map.yaml, …map.pgm」。随后 `validate_map_bundle` 应通过。

✅ **链路打通标志**：完好地图包返回 0、篡改返回 1 并报 SHA-256、重建后可再次通过。

---

## 2 · 传感器与底层链路

先用新包独立验证三维点云转换，不加载 SLAM/Nav2：

```bash
# 终端 1 —— Gazebo + /scan 转换诊断 + 专用 RViz
source scripts/setup_simdog.bash
GO2_D435_GAZEBO_ENABLED=0 \
ros2 launch go2_lidar_scan simulation_scan_debug.launch.xml \
  lidar_debug_raw_scan:=true

# 终端 2 —— 只读检查
source scripts/setup_simdog.bash
ros2 topic hz /velodyne_points
ros2 topic hz /scan
ros2 topic echo /diagnostics --once
ros2 param get /go2_lidar_scan_converter use_inf       # 期望 True
ros2 param get /go2_lidar_scan_converter range_min     # 期望 0.9
```

RViz 左侧保持 `Velodyne 3D Points`、橙色 `Leveled Navigation Scan`、洋红色
`Raw Tilting Scan` 与 `Scan Health` 打开。正常时橙色二维点落在三维点云的水平墙体上，
洋红点只作为旧倾斜切片 A/B，不得接入下游；静止墙线不复制、不整体漂移。雷达上方文字为绿色
“转换链正常”。橙色表示低频或静止跳变，红色表示点云/扫描超时；先停止运动，再根据
`/diagnostics` 的 `cloud_hz`、`scan_hz`、`invalid_bins` 和 `frame_jump_ratio` 排查。

独立入口与完整导航都会发布 `/scan`。检查完必须在终端 1 按 `Ctrl+C`，再进行后续
Gazebo/SLAM 测试，不能让两套入口同时运行。

若还要分组件确认控制器和机器人本体，再启动完整四足 Gazebo：

```bash
# 终端 1 —— Gazebo（无界面；本节只测传感器，暂不启动 bridge）
ros2 launch go2_config gazebo_velodyne.launch.py \
    gui:=false unitree_bridge:=false rviz:=false
```

> Gazebo 完整四足模型加载约需 1–2 分钟，日志出现「Desired controller update period…」属已知告警，不影响运行。

### 终端 2 —— 逐项检查

```bash
ros2 topic hz /velodyne_points          # 期望 ~10 Hz
ros2 topic echo --once /odom            # 有数据（EKF 里程计）
ros2 topic hz /d435/depth/color/points  # 可选：D435 深度点云，当前阶段一导航不以其为阻断条件
ros2 control list_controllers           # 期望两个控制器都 active
ros2 topic echo --once /odom/ground_truth --field pose.pose.position
# 期望 z≈0.21（站立高度），roll/pitch≈0（机身水平）
```

| 检查项 | 话题 / 命令 | 通过标准 |
|---|---|---|
| Velodyne 点云 | `/velodyne_points` | 独立入口通常 7–10 Hz；低于 7 Hz 记为性能 FAIL |
| 二维扫描 | `/scan` | 独立入口应 ≥7 Hz，`frame_id=velodyne_level`，空方向允许 `+inf` |
| IMU | `/imu/data` | ≈200 Hz |
| 里程计 | `/odom` | 有数据 |
| 关节状态 | `/joint_states` | 有数据 |
| D435 深度点云（可选） | `/d435/depth/color/points` | 普通 Gazebo 入口默认 10 Hz 标称；LiDAR-only 导航档无话题是预期 |
| 控制器 | `ros2 control list_controllers` | joint_group_effort / joint_states 均 active |
| 机身姿态 | `/odom/ground_truth` | z≈0.21，roll/pitch≈0 |

✅ **链路打通标志**：五个话题有数据、两个控制器 active、机器人站立正常（未趴下未侧翻）。

本节的分组件 Gazebo 使用 CHAMP 足端 EKF。第 5、7 节的统一导航入口为保证仿真闭环
可重复，会改用 `/odom/ground_truth` 适配后的 `/odom`；这不改变四足步态和 Gazebo
物理，只替换控制器的仿真反馈，真机不得照搬。

---

## 3 · 定位链路

单独启动定位器，验证 NDT 配准、发布唯一 `map→odom`。

```bash
# 终端 2 —— 定位器（Gazebo 保持运行）
ros2 launch go2_navigation localization.launch.py \
    map_dir:=$GO2_PROJECT_ROOT/go2_maps/latest localization:=lidar_ndt rviz:=true
```

### 检查节点激活与地图加载

```bash
ros2 lifecycle get /lidar_localization_node      # 期望 active
# 日志应出现：Loading pcd map from …GlobalMap.pcd → Map Size 252468 → Initial Map Published
```

### 发布初始位姿并验证配准

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: 4.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

# 配准成功后：
ros2 topic echo --once /pcl_pose                       # 输出 map 系位姿
ros2 run tf2_ros tf2_echo map odom                     # map→odom 存在
ros2 topic echo --once /alignment_status               # fitness_score < 6.0 表示配准成功
```

> 初始位姿 x/y 应落在机器人出生点附近。若 fitness 明显大于阈值或位姿乱跳，换一个候选点重试（如 (6,0)、(2.5,0)），或重新建图。

✅ **链路打通标志**：节点 active、加载地图、配准分数低于阈值、发布 /pcl_pose 与 map→odom。此时 TF 主链应为 `map → odom → base_footprint → base_link`。

---

## 4 · 完整导航栈

验证 Nav2 全部节点激活、控制链话题齐全。

```bash
# 推荐：停止本手册前面分开启动的 Gazebo/定位器后，一次启动完整闭环
# （本节观察 RPP 对照档行为，故显式传 forward_rpp；默认档已是 forward_mppi）：
ros2 launch go2_navigation simulation_navigation.launch.xml \
    map_dir:=$GO2_PROJECT_ROOT/go2_maps/latest localization:=lidar_ndt \
    controller_profile:=forward_rpp gui:=false rviz:=true

# 已自行启动 Gazebo 时，才使用下列分组件入口：
# ros2 launch go2_navigation navigation.launch.py \
#     map_dir:=$GO2_PROJECT_ROOT/go2_maps/latest localization:=lidar_ndt rviz:=true
```

> ⚠️ 同一时间只能有一套统一导航入口。若先前已经运行过，请在启动前确认没有遗留
> `gzserver`、`rviz2` 或 Nav2 节点；重复启动会造成 action、TF 和速度出口竞争。
> 启动后等待地图与 10 个 lifecycle 节点激活。`map_server` 已排在生命周期首位，
> 因此 `/map` 应在设置初始位姿前就可见。

### RViz 地图与坐标系检查（必须先做）

1. 在左侧 `Global Options` 确认 `Fixed Frame` 为 `map`。如果是 `odom`，先关闭
   RViz 并从统一入口重新打开，避免手工配置覆盖修复后的文件。
2. 确认 `Static Map` 已勾选且显示 `/map`：白色是自由区，黑色是静态障碍，灰色是
   unknown。`Localization Map` 应是浅蓝色 `/global_map` 点云。
3. `Local Costmap` 和 `Global Costmap` 默认不勾选。排查时单独勾选其中一个；机器人
   周围随雷达变化的黑色环带属于局部代价图，不表示现有 PCD 或静态地图必然错误。
4. 不需要先遥控 Go2 重新扫描。当前入口会加载 `$GO2_PROJECT_ROOT/go2_maps/latest/GlobalMap.pcd`
   与 `map.yaml/pgm`；本轮只需用当前雷达点云向已有 PCD 做 NDT 配准。
5. 点击顶部 `2D Pose Estimate`，在约 `(0, 0.8)` 的白色自由区按住左键向机器人朝向
   拖出箭头。终端应显示接受 `frame_id=map`，而不是 `odom`。等待
   `/alignment_status` 为 healthy、出现 `NDT Pose` 和 `map -> odom` 后，才允许发目标。
6. 等 NDT 稳定在约 `(-0.1, 0.9)` 后，点击顶部 `Nav2 Goal`，先在
   `(-0.1, 1.2)` 附近白色自由区拖箭头，再测试 `(-0.1, 1.5)`。终端不应出现
   “目标 frame_id 必须是 map”。不要继续用旧的 `(4,0)` 人工种子；NDT 会最终收敛
   回真实区域，造成机器人在 RViz 中再次跳动。

### 检查 10 个 lifecycle 节点全部 active

```bash
for n in lidar_localization_node map_server controller_server smoother_server \
         planner_server behavior_server bt_navigator waypoint_follower \
         velocity_smoother collision_monitor; do
  printf "%-24s " $n; ros2 lifecycle get /$n | head -1
done
```

**预期输出**：10 行，全部为 `active [3]`。

### 检查控制链话题

```bash
ros2 topic info /cmd_vel                # 唯一发布者必须为 collision_monitor；订阅者 = quadruped_controller_node(CHAMP)
ros2 topic list | grep -E "cmd_vel_nav|cmd_vel_switched|cmd_vel_smoothed"
```

**预期输出**：`/cmd_vel_nav`（Nav2 输出）、`/cmd_vel_switched`（twist_mux 输出）、`/cmd_vel_smoothed`（velocity_smoother 输出）都存在；`/cmd_vel` 的最终订阅者是 CHAMP。

✅ **链路打通标志**：10 节点全部 active、控制链话题齐全、/cmd_vel 正确接入 CHAMP。

---

## 5 · 端到端导航（最终验证）

给定位初值 → 发 Nav2 目标 → 机器人沿路径移动。

### 第一步：定位初值（必须先于目标）

```bash
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.8}, orientation: {z: 0.741, w: 0.671}}}}"
ros2 run tf2_ros tf2_echo map base_link     # 应能查询到完整链
```

### 第二步：发布 Nav2 目标

```bash
# 方式一：RViz2 顶部工具栏「Nav2 Goal」，在地图上点一个 free 区域
# 方式二：命令行（当前地图已验证短目标）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: -0.1, y: 1.5}, orientation: {z: 0.741, w: 0.671}}}}"
```

### 先验证越界目标被门禁拒绝（不会触发规划器崩溃）

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: -3.58, y: 0.53}, orientation: {w: 1.0}}}}"
ros2 lifecycle get /planner_server      # 仍应为 active [3]
```

**预期输出**：action 显示目标被拒绝，`/goal_rejected` 说明其超出地图边界；
`planner_server` 保持 active。之后再下发 `(-0.1,1.5)` 的合法短距离目标。

### 第三步：观察执行

```bash
ros2 topic echo /cmd_vel            # Nav2 输出的速度（经 collision_monitor）
ros2 topic echo /cmd_vel_smoothed   # 平滑后的速度
ros2 topic echo --once /plan        # 全局路径（应有点）
ros2 topic echo --once /odom/ground_truth --field pose.pose.position   # 位置在变 = 在走
ros2 topic echo /cmd_vel_nav --field linear.y  # RPP 对照档下应持续接近 0
```

### 学会看 RPP 对照档的行为

1. 故意把目标箭头画成与当前朝向相差较大；偏差超过约 `0.35 rad`
   时，应先看到机器人转向路径，而不是斜着横移。
2. `/lookahead_arc` 是 RPP 向前检查的跟随弧；弧越短通常表示急弯、靠近障碍或即将
   到达，控制器会降速。
3. 开阔直线上 `/cmd_vel_nav.linear.x` 应能接近 `0.24–0.27 m/s`；弯道上变慢是
   曲率调节，不是无条件的“速度太慢”。
4. 如需对照原行为，冷启动时改为 `controller_profile:=omni_mppi`，比较
   `/cmd_vel_nav.linear.y`、路径耗时和 Gazebo 中的机身朝向。

✅ **到达验收标志**：目标被接受 → `/plan` 生成路径 → 机器人位置持续变化 → action 返回
`SUCCEEDED`。当前地图已完成 `(-0.1,0.9) → (-0.1,1.5) → (-0.1,0.9)` 往返：
去程 Gazebo 真值移动约 `0.517 m`，最终 NDT 位姿约 `(-0.090,1.546)`；回程同样
返回 `SUCCEEDED`。

2026-08-11 回归还完成了静态地图短目标 `(约 3.81,0.02) -> (4.30,0)`：action
返回 `SUCCEEDED`，最终 `/cmd_vel.linear.y=0`，Gazebo 真值移动约 `0.20 m`。

### 导航模式键盘遥控

导航运行时不能直接向最终 `/cmd_vel` 发布；该话题必须保持由 `collision_monitor`
唯一发布。另开终端执行：

```bash
cd /home/luhao/my/ROS/Go2_Bilibili_zhao-main
source scripts/setup_unitree_sim.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/cmd_vel_teleop
```

实测连续按 `i` 后 Gazebo 真值前进约 `0.188 m`。使用 `setup_simdog.bash` 且不 remap
时发布的是最终 `/cmd_vel`，会绕开并争用安全链出口，不能用于导航模式验收。

> `/odom/ground_truth` 的坐标系是 `world`，导航目标是 `map`，当前没有
> `map -> world` TF。它适合确认 Gazebo 中的实际移动；不要直接把它的绝对坐标与
> `map` 目标坐标相减。真值误差验收需要先记录或提供两坐标系间的转换。

**失败排查 · 目标被拒绝**：`/goal_rejected` 会给出越界、障碍/unknown、安全余量或
定位过期等原因。先重新发布合理 `/initialpose`，待 `/alignment_status` 为 healthy，
再从地图内 free 区域选择目标。规划失败（超迭代/无路径）则说明地图质量仍不足，
换更保守的目标或重新建图。

---

## 6 · 安全控制链

验证键盘经安全链控制、导航锁能停住机器人。

### 键盘经 twist_mux 控制

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/cmd_vel_teleop
# 按 i 前进、, 后退、j/l 转向、k 停止；速度经过完整安全链。
```

### 正常取消与锁存停止

```bash
# 机器人卡住：取消公开/内部 action，并锁存零速度
ros2 service call /navigation/stop std_srvs/srv/Trigger "{}"
ros2 topic echo --once /pause_navigation       # 期望 data: true
ros2 topic echo --once /cmd_vel                # 期望全零

# 定位、SLAM、传感器与节点都健康后才能恢复
ros2 service call /navigation/resume std_srvs/srv/Trigger "{}"
```

在 RViz 右侧 `Navigation 2` 面板中，`Cancel` 只用于正常取消当前目标；
`/navigation/stop` 是卡住时的强制锁存停止。`resume` 后旧目标不会复活，
需要重新点击 `Nav2 Goal`。不要直接向 `/pause_navigation` 发一次性 `false`，
安全监督会继续发布它的真实锁状态。

✅ **链路打通标志**：键盘能控制机器人；停止后 0.5 s 内 `/cmd_vel` 归零并保持；
健康时可恢复且旧目标不续行。

---

## 7 · 在线建图 + 手动目标导航

这一节使用 `map_session:=new`，不加载 `$GO2_PROJECT_ROOT/go2_maps/latest`、
`$GO2_PROJECT_ROOT/go2_maps/online/latest` 或任何其他旧会话，也不需要 `2D Pose Estimate`。在启动前停掉
固定地图导航，确保没有 NDT、`map_server` 或另一个 `map -> odom` 发布者。

```bash
# 终端 1 —— 完整在线建图导航；它已经自动启动 go2_lidar_scan
cd /home/luhao/my/ROS/Go2_Bilibili_zhao-main
source scripts/setup_unitree_sim.bash
ros2 launch go2_navigation simulation_navigation.launch.xml \
    navigation_mode:=online_slam \
    map_session:=new controller_profile:=forward_mppi gui:=true rviz:=true \
    lidar_debug_raw_scan:=false use_d435_navigation:=false tuning_gui:=false
```

正式建图前回读 `min_height=0.2`、`max_height=0.3`；`tuning_gui:=false` 是为了避免建图时
误触当前现场有效值。只有重新做单变量 A/B 时才打开 rqt。

```bash
ros2 param get /go2_lidar_scan_converter min_height
ros2 param get /go2_lidar_scan_converter max_height
```

### RViz 里具体看什么

1. 左侧 `Global Options -> Fixed Frame` 应为 `map`。
2. `Live SLAM Map` 默认勾选；白色是已观测自由区，黑色是已观测障碍，
   灰色或空白是 unknown。它与固定模式的 `Static Map` 不是同一数据源。
3. `SLAM Scan` 应在机器人周围显示橙色扫描点。若扫描点在墙上但地图不更新，
   检查 `/scan` 频率与 `map -> odom`；若扫描点本身错乱，先退出完整导航并使用第 2 节
   专用 RViz 排查。当前可在 rqt 只调 `min_height/max_height`；每个候选正式比较时都要
   重启空白 `map_session:=new`，避免旧白线污染结果。
4. 点击 `Nav2 Goal`，只点当前白色自由区靠近 unknown 的边缘。机器人到达后会
   看到更远区域，`Live SLAM Map` 的已知栅格才会增加。
5. `Global Costmap` 是 Nav2 对当前 `/map` 加障碍与膨胀的结果；`Local Costmap`
   是跟随机器人滚动的 5×5 m 窗口。它们会动态变化，但不是保存地图。
6. `LiDAR Scan Health` 显示雷达上方中文 Marker：绿是转换链正常，橙是低频/静止跳变，
   红是点云或 `/scan` 超时。橙/红时先按键盘 `k`，再调用 `/navigation/stop`。

```bash
# 终端 2 —— 只读运动探针，不会发布任何速度
cd /home/luhao/my/ROS/Go2_Bilibili_zhao-main
source scripts/setup_unitree_sim.bash
ros2 run go2_lidar_scan motion_scan_probe --duration 150
```

```bash
# 终端 3 —— 键盘控制，必须走 /cmd_vel_teleop 安全入口
cd /home/luhao/my/ROS/Go2_Bilibili_zhao-main
source scripts/setup_unitree_sim.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/cmd_vel_teleop
# i 前进、, 后退、j/l 左右转、k 停止；不要直接发布 /cmd_vel。
```

```bash
# 探针结束后逐条补充检查；ros2 topic hz 需按 Ctrl+C 才会进入下一条
ros2 topic hz /scan
ros2 topic echo --once /map --field info
ros2 run go2_navigation health_check --mode online_slam

# 结束前同时保存可视化地图和可续建 pose graph
bash simdog/src/go2/go2_navigation/scripts/save_online_map.sh my_world_full_v1
```

驾驶顺序应为外墙一圈、内部平行往返、主要障碍一圈、回到起点闭环；每段按 `k` 停止，
闭环后静止 5–10 秒再保存。保存成功后目录还应包含记录本次 `0.20..0.30 m` 的
`session.yaml`。退出在线栈后，用同一明确目录启动固定图导航：

```bash
ros2 launch go2_navigation simulation_navigation.launch.xml \
  navigation_mode:=static_map localization:=amcl \
  map_dir:=$GO2_PROJECT_ROOT/go2_maps/online/my_world_full_v1 \
  controller_profile:=forward_mppi gui:=true rviz:=true
```

RViz 先用 `2D Pose Estimate` 标定出生位置和朝向，再按“1–2 m 短目标、90° 转弯、长走廊、
回到起点”顺序点击 `Nav2 Goal`。只点白色自由区；异常先 `Cancel` 或
`/navigation/stop`。

下次续建：

```bash
ros2 launch go2_navigation simulation_navigation.launch.xml \
    navigation_mode:=online_slam \
    map_session:=$GO2_PROJECT_ROOT/go2_maps/online/my_world_full_v1 gui:=true rviz:=true
```

✅ **通过标准**：机器人移到观测边缘后已知栅格数或地图边界增加；保存目录同时
包含 `map.pgm`/`map.yaml`、`slam.posegraph`/`slam.data` 和 `session.yaml`；续建后旧区域仍在。

2026-08-22 历史样本用 `/cmd_vel_teleop` 完成正反整圈、四次转弯闭合路线和前后移动，退出前
发布零速度。运动探针采样 240 帧，最大机身倾斜 `5.15°`：旧投影有 4430 个地面获胜
端点、对齐投影为 0；同时间戳 TF 成功率 100%，`/scan=8.89 Hz`，`map→odom` 最大
单步 `0.0224 m/0.00698 rad`，该次探针判定 PASS。后续人工在线 SLAM 曾复现白色扇形线；
用户把高度窗调到 `0.20..0.30 m` 后目视正常，该值已固化，下一步是完整覆盖建图、保存
和固定 AMCL 导航。1800 水平列
对照为 5.06 Hz，900 列只恢复性能。

同次运行两张 costmap 合计仍出现 2 次 `OutTheBack` 旧观测丢弃。它表示该帧没进入代价
图，不会生成幽灵端点；若频繁出现仍要记录为时序问题。RViz 出现红项、机器人不动或
扫描停止时，先按 `k`/空格并调用 `/navigation/stop`，确认 `/cmd_vel` 归零后再看
`/diagnostics`、`ros2 topic hz /scan` 和 TF，不能边运动边重启传感器。

---

## 8 · GPU 实验档（可选）

验证 CUDA NDT 仍工作（独立域，自动隔离）。

```bash
bash scripts/verify_gpu_runtime.sh
```

**预期输出**：`[1/6]…[6/6] 验证通过`，中间出现 `CUDA NDT enabled on device 0: NVIDIA GeForce RTX 4060 Laptop GPU`、NDT 计算进程约 98 MiB、发布 /ndt_pose。

**若报「NDT 节点未链接 CUDA」**：说明 ndt_relocalization 被构建成 CPU 版。必须带 CUDA 参数重建（见「A · 前置准备」），或直接跑 `bash scripts/build_workspaces.sh`。

---

## 故障排查速查表

| 症状 | 可能原因 | 处理 |
|---|---|---|
| 节点一直 inactive | Nav2 在等 map frame（定位未完成） | 先给 /initialpose；等 1–2 分钟；确认定位器日志出现 Map Size |
| CLI 命令很慢 / 超时 | CycloneDDS lo 接口无多播，发现慢 | 命令加长等待；或用 rclpy 脚本替代多次 CLI |
| 规划失败（超迭代） | 起点/目标是障碍或未知 | 换 free 目标点；重新建图；查 /map 该坐标代价 |
| 机器人不动 | /pause_navigation 被锁 / collision_monitor 急停 | 先调 `/navigation/stop`；排障后调 `/navigation/resume`；检查 /cmd_vel |
| tf2_echo 查不到 map→odom | 定位器未配准 / 未给 initialpose | 给 /initialpose；查 /pcl_pose 与 /alignment_status |
| 在线地图不扩大 | 目标未靠近已知/unknown 边界，或 /scan 无数据 | 勾选 Live SLAM Map/SLAM Scan；查 `/scan` 与 TF |
| 固定模式又出现旧图 | `map_dir` 指向旧目录，或旧 RViz 保留最后一帧 `/map` | 查 `/map_server` 的 `yaml_filename` 和 lifecycle；只保留一套导航/RViz |
| GPU 验证报未链接 CUDA | ndt_relocalization 是 CPU 版 | 用 build_workspaces.sh 或带 CUDA 参数重建 |
