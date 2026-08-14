# Go2 Nav2 运行时参数能力矩阵

更新时间：2026-08-14；适用版本：ROS 2 Humble、Navigation2 `1.1.20`、
`controller_profile:=forward_rpp`。

本文是 `nav_tuner` 的审计依据。代码中的唯一机器可读注册表位于
`go2_navigation/nav_tuning.py`；表内“LIVE”不是指参数服务返回成功，而是插件确有动态
回调。“LIFECYCLE RELOAD”表示参数服务会接受并回读，但旧插件内部对象不会刷新；
“RESTART REQUIRED”表示工具只显示并拒绝伪热更新。

## 能力定义与安全流程

| 分类 | 工具行为 | 成功判据 |
|---|---|---|
| `LIVE` | 对单节点调用 `SetParametersAtomically` | service 成功、read-back 相同，并在 costmap、footprint 或控制输出看到对应变化 |
| `LIFECYCLE RELOAD` | `/navigation/stop` → active 态暂存参数 → Lifecycle `RESET` → `STARTUP` → 等待全部 active → `/navigation/resume` | 暂停期间最终速度为零、插件重新初始化、read-back 相同、全部节点 active；旧目标不续行 |
| `RESTART REQUIRED` | 显示实际归属并拒绝 `set/reset/save` | 用户修改结构配置后完整重启统一入口 |

计划中的“RESET 后在 unconfigured 态 set”经隔离实测不可直接用于当前 Humble 进程结构：
`/local_costmap/local_costmap` 和 `/global_costmap/global_costmap` 是
`controller_server/planner_server` 的嵌入式节点，父节点 cleanup 后其参数服务 callback
group 不再应答。实测服务请求直到下一次 configure 才返回，此时已错过插件构造。
因此工具先在 active 态原子暂存 source 值；ObstacleLayer 此时不会刷新旧
`ObservationBuffer`，随后 RESET/STARTUP 重建插件，新 buffer 才读取该值。这个顺序既不把
read-back 冒充生效，又避免在 unconfigured 态死等。依据分别是
[Lifecycle Manager 1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_lifecycle_manager/src/lifecycle_manager.cpp)
和
[ObstacleLayer 1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/plugins/obstacle_layer.cpp)。

只有工具确认初始 Nav2 健康、`/pause_navigation=false`，并且确由工具调用 stop 时，才会
自动 resume。任何 reload/回滚失败都保持停车锁，用户必须先排障；resume 不恢复旧目标。

## Costmap、Footprint 与 Inflation

| 别名 | 节点与完整参数 | 类型/单位 | YAML 路径 | 基线 | 校验 | 能力 |
|---|---|---|---|---:|---|---|
| `local.inflation_radius` | `/local_costmap/local_costmap.inflation_layer.inflation_radius` | float/m | `navigation.yaml: local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius` | 0.30 | 0–5 | LIVE |
| `local.cost_scaling_factor` | `/local_costmap/local_costmap.inflation_layer.cost_scaling_factor` | float/1/m | 同层 `cost_scaling_factor` | 3.0 | 0.01–100 | LIVE |
| `global.inflation_radius` | `/global_costmap/global_costmap.inflation_layer.inflation_radius` | float/m | `navigation.yaml: global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius` | 0.30 | 0–5 | LIVE |
| `global.cost_scaling_factor` | `/global_costmap/global_costmap.inflation_layer.cost_scaling_factor` | float/1/m | 同层 `cost_scaling_factor` | 3.0 | 0.01–100 | LIVE |
| `geometry.local_footprint` | `/local_costmap/local_costmap.footprint` | string polygon/m | `navigation.yaml: local_costmap.local_costmap.ros__parameters.footprint` | 阶段 2 标定的 24 顶点凸包 | ≥3 个有限二维顶点 | LIVE |
| `geometry.global_footprint` | `/global_costmap/global_costmap.footprint` | string polygon/m | `navigation.yaml: global_costmap.global_costmap.ros__parameters.footprint` | 同上 | ≥3 个有限二维顶点 | LIVE |
| `geometry.local_padding` | `/local_costmap/local_costmap.footprint_padding` | float/m | local costmap `footprint_padding` | 0.035 | 0–0.5 | LIVE |
| `geometry.global_padding` | `/global_costmap/global_costmap.footprint_padding` | float/m | global costmap `footprint_padding` | 0.035 | 0–0.5 | LIVE |

InflationLayer 的 `inflation_radius/cost_scaling_factor` 动态回调会重新计算缓存；
Costmap2DROS 的 footprint 与 padding 动态回调会更新发布足迹。上游依据：
[InflationLayer 1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/plugins/inflation_layer.cpp)。

## ObstacleLayer observation source

下表每一格都是注册表中的独立别名。`<source>` 展开为 `local.scan`、`local.d435`、
`global.scan`；其节点/参数/YAML 前缀为：

| `<source>` | 节点 | runtime 参数前缀 | `navigation.yaml` 路径前缀 | reset group |
|---|---|---|---|---|
| `local.scan` | `/local_costmap/local_costmap` | `scan_layer.scan` | `local_costmap.local_costmap.ros__parameters.scan_layer.scan` | `local_scan` |
| `local.d435` | `/local_costmap/local_costmap` | `d435_layer.d435` | `local_costmap.local_costmap.ros__parameters.d435_layer.d435` | `local_d435` |
| `global.scan` | `/global_costmap/global_costmap` | `obstacle_layer.scan` | `global_costmap.global_costmap.ros__parameters.obstacle_layer.scan` | `global_scan` |

| 别名 `memory.<source>.<field>` 的 `<field>` | 类型/单位 | local.scan 基线 | local.d435 基线 | global.scan 基线 | 校验 | 能力 |
|---|---|---:|---:|---:|---|---|
| `observation_persistence` | float/s | 0.0 | 0.0 | 0.0 | 0–10 | LIFECYCLE RELOAD |
| `expected_update_rate` | float/s | 0.0 | 0.0 | 0.0 | 0–60；0 关闭 stale 检查 | LIFECYCLE RELOAD |
| `obstacle_min_range` | float/m | 0.0 | 0.0 | 0.0 | 0–50 | LIFECYCLE RELOAD |
| `obstacle_max_range` | float/m | 15.0 | 2.5 | 15.0 | 0–100 | LIFECYCLE RELOAD |
| `raytrace_min_range` | float/m | 0.0 | 0.0 | 0.0 | 0–50 | LIFECYCLE RELOAD |
| `raytrace_max_range` | float/m | 15.0 | 3.0 | 15.0 | 0–100 | LIFECYCLE RELOAD |
| `min_obstacle_height` | float/m | 0.0 | 0.05 | 0.0 | -10–10 | LIFECYCLE RELOAD |
| `max_obstacle_height` | float/m | 2.0 | 1.50 | 2.0 | -10–10 | LIFECYCLE RELOAD |
| `marking` | bool | true | true | true | 只允许 true | LIFECYCLE RELOAD |
| `clearing` | bool | true | true | true | 只允许 true | LIFECYCLE RELOAD |

另外三个 layer 级别名为
`memory.local.scan.footprint_clearing_enabled`、
`memory.local.d435.footprint_clearing_enabled`、
`memory.global.scan.footprint_clearing_enabled`；分别映射到对应 layer 的同名 bool 参数，
YAML 也位于对应 layer 根下，基线均为 true、只允许 true、能力为 LIVE。

source 参数在 YAML 中显式写出当前插件默认值，目的是让未来 `save` 能做定点标量替换，
不改变阶段 0 运行行为。

## RPP 与 Velocity Smoother

下列 RPP 别名的节点统一为 `/controller_server`，runtime 前缀统一为 `FollowPath.`，
YAML 统一位于 `controller_forward_rpp.yaml: controller_server.ros__parameters.FollowPath`，
reset group 为 `rpp`。

| 别名 | 类型/单位 | 基线 | 校验 | 能力 |
|---|---|---:|---|---|
| `rpp.desired_linear_vel` | float/m/s | 0.27 | 0–1 | LIVE |
| `rpp.lookahead_dist` | float/m | 0.55 | 0–10 | LIVE |
| `rpp.min_lookahead_dist` | float/m | 0.35 | 0–10 | LIVE |
| `rpp.max_lookahead_dist` | float/m | 0.80 | 0–10 | LIVE |
| `rpp.lookahead_time` | float/s | 1.5 | 0–10 | LIVE |
| `rpp.min_approach_linear_velocity` | float/m/s | 0.10 | 0–1 | LIVE |
| `rpp.approach_velocity_scaling_dist` | float/m | 0.50 | 0–10 | LIVE |
| `rpp.max_allowed_time_to_collision_up_to_carrot` | float/s | 1.0 | 0–10 | LIVE |
| `rpp.use_regulated_linear_velocity_scaling` | bool | true | bool | LIVE |
| `rpp.use_cost_regulated_linear_velocity_scaling` | bool | false | bool | LIVE |
| `rpp.cost_scaling_dist` | float/m | 0.25 | 0–10 | LIVE |
| `rpp.cost_scaling_gain` | float | 1.0 | 0–10 | LIVE |
| `rpp.inflation_cost_scaling_factor` | float/1/m | 3.0 | 0.01–100 | LIVE |
| `rpp.regulated_linear_scaling_min_radius` | float/m | 0.40 | 0–10 | LIVE |
| `rpp.regulated_linear_scaling_min_speed` | float/m/s | 0.10 | 0–1 | LIVE |
| `rpp.use_collision_detection` | bool | true | 只允许 true | LIFECYCLE RELOAD |
| `rpp.use_interpolation` | bool | true | bool | LIFECYCLE RELOAD |
| `velocity.max_velocity` | float[3]/m/s,rad/s | `[0.27,0.15,0.45]` | 非空有限数组 | LIVE |

`velocity.max_velocity` 的节点是 `/velocity_smoother`，YAML 位于
`controller_forward_rpp.yaml: velocity_smoother.ros__parameters.max_velocity`，reset group
为 `velocity`。RPP 1.1.20 的动态回调不读取 `use_collision_detection` 和
`use_interpolation`，所以即使参数服务接受也必须 reload。依据：
[RPP 1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp)。

## SmacPlanner2D 与结构项

Planner 别名的节点统一为 `/planner_server`，runtime 前缀 `GridBased.`，YAML 位于
`navigation.yaml: planner_server.ros__parameters.GridBased`，reset group 为 `planner`。

| 别名 | 类型/单位 | 基线 | 校验 | 能力 |
|---|---|---:|---|---|
| `planner.tolerance` | float/m | 0.0 | 0–10 | LIVE |
| `planner.cost_travel_multiplier` | float | 2.0 | 0–100 | LIVE |
| `planner.max_iterations` | int/次 | 100000 | 1–10000000 | LIVE |
| `planner.max_on_approach_iterations` | int/次 | 1000 | 1–10000000 | LIVE |
| `planner.allow_unknown` | bool | true | bool | LIVE |
| `planner.downsample_costmap` | bool | false | bool | LIVE |
| `planner.downsampling_factor` | int/倍 | 1 | 1–100 | LIVE |
| `planner.use_final_approach_orientation` | bool | false | bool | LIVE |
| `structure.controller_plugin` | string | RotationShimController | 只显示 | RESTART REQUIRED |
| `structure.planner_plugin` | string | SmacPlanner2D | 只显示 | RESTART REQUIRED |
| `structure.local_plugins` | string[] | scan/d435/inflation | 只显示 | RESTART REQUIRED |
| `structure.local_scan_sources` | string | scan | 只显示 | RESTART REQUIRED |
| `structure.replan_frequency` | float/Hz | 1.0 | 只显示 | RESTART REQUIRED |

`structure.replan_frequency` 不属于 ROS 参数；来源是
`behavior_trees/go2_navigate_to_pose.xml` 的 `<RateController hz="1.0">`。插件列表、插件类型
和 observation source 结构都会改变对象拓扑，只能完整重启。

## 命令、监控与持久化

```bash
ros2 run go2_navigation nav_tuner
python3 simdog/src/go2_navigation/tools/nav_tuner.py
ros2 run go2_navigation nav_tuner --snapshot --sample-seconds 10
ros2 run go2_navigation nav_tuner --monitor-only
```

交互命令为 `show`、`set <alias> <value>`、`reset <alias>`、
`reset-group <group>`、`profile <name>`、`save`、`record [key=value ...]`、`help`、`quit`。
阶段 0 的 `safe/balanced/aggressive` 均固定显示 `UNCALIBRATED`，不能激活。

监控数据来源与含义：

| 面板 | 来源 | 正常含义 | 异常表现 |
|---|---|---|---|
| Sensor | `/scan`、`/depth/color/points` | Hz、墙钟年龄、p50/p95/p99 周期；scan 的 valid/inf/nan 与最近有限距离 | age 增长、Hz 为 N/A 或最近距离被 `range_min` 截断 |
| Costmap | `*/costmap_raw`、`*/published_footprint`、TF | lethal/inflated 数、机器人到最近 lethal 格方形边界的距离、实际 footprint | 一直全零、footprint 顶点不随参数变、TF 不可用时距离 N/A |
| Plan | `/plan` | 路径长度、年龄、重规划间隔；按半个栅格加密后到 lethal 方格边界的保守 clearance | 目标活动时 plan 过旧、间隔偏离 1 Hz、clearance 过小 |
| Controller | 四级速度话题 | RPP 插件、调速开关、上游是否调速、Collision Monitor 是否进一步限速 | 上游有速度但下游长期为零，或最终速度绕过监控 |

`save` 只保存本进程成功修改过的、注册表管理的标量路径。Costmap/Planner 写
`config/navigation.yaml`；RPP/Velocity Smoother 写
`config/controller_forward_rpp.yaml`。写前备份两个涉及文件到
`logs/backups/<timestamp>/`，临时文件通过语义复核后 `os.replace`；任一替换失败便用备份
恢复全部文件，最后输出 unified diff。注释、顺序和层级保持不变。`record` 追加
`logs/inflation_tuning.csv`，包含 run id、传感器 p99、clearance、接触事件和参数生效方式。

`rqt_reconfigure` 仍可辅助观察标准参数，但它不知道上述能力分类、无法执行安全 reload、
不会验证内部效果，也没有白名单保存和实验记录，因此不是统一入口。

## 2026-08-14 隔离实测

实测与静态推断分开记录：

- **已实测，Domain 220，在线 SLAM、无 GUI：**10 秒 snapshot 中 `/scan=2.717 Hz`，
  周期 p50/p95/p99=`0.277/0.714/0.790 s`，`range_min=0.900 m`；Local Costmap
  `117 lethal/1555 inflated`，Global Costmap `460 lethal/3613 inflated`，两张图均为
  `0.05 m/cell`；`/pause_navigation=false`。
- **已实测 LIVE：**local inflation `0.30→0.40 m` 原子 set/read-back 成功，代表性快照的
  inflated 格从 1623 增至 2129；local padding `0.01→0.03 m` 后发布 footprint 外框约从
  `0.72×0.50 m` 增至 `0.76×0.54 m`；RPP 速度 `0.27→0.26 m/s` 回读成功。三项均恢复
  YAML 基线。
- **已实测 LIFECYCLE RELOAD：**local scan persistence
  `0.0→0.2→0.0 s`；每次都先锁定 `/pause_navigation`，RESET/STARTUP 日志显示
  ObstacleLayer 重新初始化，最终 8 个 Nav2 lifecycle 节点 active、参数回读正确、
  `/pause_navigation=false`。Domain 223 加严复测先确认最终 `/cmd_vel.linear.x=0.1`，
  再执行同一 reload，工具确认零速度后才拆除插件并成功恢复；随后参数恢复 YAML 基线。
  一次高负载复测发现 3 秒的单节点状态查询会误判迟到响应，因此现采用单节点 5 秒、
  整体 60 秒健康窗口。旧目标未自动续行。
- **已实测 RESTART REQUIRED：**尝试 `set structure.replan_frequency 2.0` 被拒绝并以状态 2
  退出，没有修改 BT。
- **已实测控制出口：**`/cmd_vel` 只有 `/collision_monitor` 一个 publisher。
- **用户复核后回归：**修正 `published_footprint` durability 后，Domain 224 固定 AMCL
  快照同时收到 local/global 各 4 个足迹顶点且不再报告 QoS 不兼容；交互界面通过
  `try_shutdown()` 在 Ctrl+C 后以状态 0 退出。
- **待后续实验：**同一在线 10 秒窗口没有收到 D435 点云；另一次隔离运行仅约
  `0.30 Hz`。这说明“已配置”不能当作深度冗余已验证，阶段 5 前禁止据此放宽任何安全门。
  `/scan.range_min=0.9 m` 只作为阶段 1 盲区量化的调查起点。
- **阶段 2 已实测，Domain 228：**从 21 个 URDF collision 采集站立/前进/转向/横移
  共 220 帧、4620 条 TF；推荐 24 顶点凸包和 `0.035 m` padding。四项 LIVE 设置
  原子 set/read-back 成功，local/global `published_footprint` 均为 24 点；反变换到
  `base_footprint` 后外框分别约为
  `[-0.4340,0.3890]×[-0.2370,0.2290] m`。使用消息时间戳查询 TF 后，local/global
  热更新验证的最大逐顶点误差分别为 `4.06×10⁻⁸ m`、`3.47×10⁻⁸ m`；Domain 229
  从 YAML 冷启动后两者均不超过 `2.11×10⁻⁸ m`，健康检查 PASS，最终 `/cmd_vel`
  发布者唯一。长 footprint 现规范化为单行 JSON，`save` 不再产生 PyYAML 折行转义。

结论：阶段 0 工具链、阶段 1 LiDAR 盲区和阶段 2 Footprint 均 PASS；Inflation、
Persistence、Depth 和 Collision Monitor 数值仍未标定，不能把当前配置称为安全最优值。
用户随后观察到 Global Costmap 幽灵障碍和近距方块碰撞，当前系统级安全门已改判为 FAIL。
静态审计确认 `/scan` 使用 `+inf`，而两个 ObstacleLayer source 的 `inf_is_valid` 默认
为 false；现有 stop/decel zone 又位于实测 LiDAR 盲区内。阶段 3 前必须先完成 clearing
与 `map→odom` 基线，详见
[幽灵障碍与近距碰撞调查](costmap_ghost_obstacle_investigation.md)。
