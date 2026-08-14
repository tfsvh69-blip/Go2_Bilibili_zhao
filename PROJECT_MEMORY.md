# 项目记忆

## 当前状态

更新时间：2026-08-14

本目录当前只维护 `simdog/` 一个 ROS 2 Humble colcon 工作空间。它使用 CHAMP
生成完整四足步态，通过 `ros2_control` 驱动 Go2 的 12 个腿部关节，并集成
Velodyne、IMU、RealSense、LIO-SAM 建图和 NDT 重定位。
当前执行 `colcon list` 可识别 24 个 ROS 2 包（含新增的
`go2_navigation_bt_plugins`）。

`go2_navigation` 当前以统一入口提供两条互斥闭环：默认
`online_slam` 使用 Slam Toolbox 边建图边导航；`static_map` 默认以 AMCL 在固定二维图
定位。`lidar_ndt` 通过二维全局 EKF、`ndt_cuda` 作为实验后端保留。两种模式都复用
SmacPlanner2D + SmoothPath + Rotation Shim（内层 RPP，默认）/MPPI（对照）以及
`twist_mux → velocity_smoother → collision_monitor → /cmd_vel` 安全控制链。
全局与局部代价图的 `/scan` observation source 均已显式接受 `0–2.0 m`
高度的障碍；移动障碍 marking、1 Hz 全局重规划和 RPP 碰撞约束链路已接通。但新发现
`+inf` 空射线 clearing 不完整、Global Costmap 幽灵障碍及 Collision Monitor 区域位于
LiDAR 盲区内，当前系统级障碍安全门为 FAIL，不能继续描述为已形成安全闭环。

`go2_navigation` 现提供 `ros2 run go2_navigation nav_tuner` 统一运行时调参和监控入口。
72 个参数别名明确区分 `LIVE`、`LIFECYCLE RELOAD` 和 `RESTART REQUIRED`；
`safe/balanced/aggressive` 在完整安全验收前保持 `UNCALIBRATED`。
可重复障碍探针可以用 Gazebo 标准实体服务移动带 ContactSensor 的方块，
并同步采集 `/velodyne_points`、`/scan`、D435 和接触真值。
local/global costmap 已使用 URDF collision 与 220 帧 CHAMP 步态联合标定的 24 顶点
footprint，padding 为 `0.035 m`；`footprint_calibrator --verify-only` 可直接核对两张
costmap 的实际发布轮廓。

## 2026-08-14 全局代价图幽灵障碍与近距碰撞新阻塞

### 用户现象与当前判定

- 用户提供的连续 RViz 画面显示：Global Costmap 中出现不属于静态墙体的多块
  lethal/inflated 障碍岛；机器人移动后新岛跳到其他位置，旧岛长时间存在后概率性消失。
  标准方块接近机器人时其代价区消失，随后发生真实接触。
- 阶段 0、1、2 的工具/量化/几何子门仍各自 PASS，但系统级障碍安全门改判为 **FAIL**。
  阶段 3 Inflation 尚未开始，三个 profile 继续 `UNCALIBRATED`，阶段 7 重复移动验收
  禁止开始。

### 已确认与待实测根因

- `pointcloud_to_laserscan` 配置为 `use_inf: true`，无回波 bin 输出 `+inf`；local/global
  LaserScan source 未设置 `inf_is_valid`，Nav2 1.1.20 默认 false。官方源码表明只有
  valid-inf callback 会把正无穷替换为 `range_max-0.0001 m` 再投影。因此当前空射线
  clearing 链不完整，可以解释旧格为什么要等有限射线偶然穿过才消失。
- 这不能单独解释错误端点的首次出现。截图中的墙形复制和整块跳位还可能来自
  `map→odom` 修正；目前只有视觉证据，尚无与截图同步的 TF 数值，标为待实测而不是
  已确认根因。
- 阶段 1 已实测 LiDAR 正前可靠表面下限为传感器前 `0.90 m`，传感器位于
  `base_footprint` 前约 `0.20 m`，即最近可靠表面约为基座前 `1.10 m`。现有 Collision
  Monitor decel/stop 前缘只有 `0.72/0.52 m`；按 `0.20 m/s × 1.5 s` 计算，approach
  加外扩 Footprint 前缘也只预测到约 `0.689 m`。三者都位于 LiDAR 盲区内，无法构成
  可靠正前防撞闭环；D435 p99 `72.14 s` 也不能兜底。
- 当前 `inflation_radius=0.30 m` 仍是阶段 2 明确保留的未标定值，小于外扩 Footprint
  外接半径 `0.474 m`。它需要阶段 3 单变量实验，但不能先用它放大幽灵格来掩盖 clearing。

### 后续顺序

1. 固定 `static_map + AMCL`，一次只开 Static Map、LaserScan、Global Costmap，记录
   60 s `/scan`、TF、AMCL 和 raw costmap；先区分地图污染、`map→odom` 跳变和
   ObstacleLayer。
2. 审计 `inf_is_valid` 与 obstacle/raytrace max 的组合。当前三者上限都为 `15.0 m`，
   不能孤立开启 valid-inf，否则转换后的 `14.9999 m` 端点可能进入 marking。将三项作为
   source 语义重置组，先 local 后 global，通过 Lifecycle RESET/STARTUP 或完整重启重建
   callback；以实际删除方块验证清除上界，同时检查最远处无新障碍环。
3. clearing 与 TF PASS 后才进入阶段 3 Inflation，再做阶段 4 Persistence A/B。
4. 完成 D435 审计后重标定 Collision Monitor；stop 边界必须位于至少一个可靠 source
   持续可见的位置。该门 PASS 前不做自主移动碰撞测试。

完整证据等级、CLI/RViz 操作和阶段交接见
`simdog/src/go2_navigation/docs/costmap_ghost_obstacle_investigation.md`。本轮只记录和诊断，
未修改运行参数。

## 2026-08-14 固定 AMCL 足迹显示与验证回归

- 隔离 Domain 230 复现：固定地图启动但尚未发布 `/initialpose` 时，AMCL 已 active，
  local costmap 也持续发布 24 点 `/local_costmap/published_footprint`（frame=`odom`）；
  但 `map→odom` 不存在，planner/global costmap 卡在等待 TF，因而 RViz Fixed Frame
  为 `map` 时看不到绿色轮廓，global costmap 参数服务也可能超时。在线 SLAM 会自动
  建立 `map→odom`，所以没有这一步空窗。
- 连续发布初始位姿并让 AMCL 收到后，实测 `map→odom` 建立、planner/controller 均为
  `active [3]`，local/global published footprint 各为 24 点。这证明 polygon 配置存在，
  肉眼差异的根因是 AMCL 初始定位尚未完成，不应通过修改 footprint 或伪造静态 TF 掩盖。
- `footprint_calibrator` 现在先等待可配置的 `--readiness-timeout`（默认 5 s）确认
  `map→odom`。未就绪时直接解释绿色线、global costmap 与 RViz `2D Pose Estimate` 的
  关系；预检在速度接管前失败时不再调用 `/navigation/stop` 改变用户原有暂停状态。
- 修复验证器启动瞬间的 TF 历史竞态：坚持使用 Polygon 自身时间戳，但会等待一帧落入
  本节点 TF Buffer 范围的新消息，不再把 `extrapolation into the past` 当作几何失败。
- 重建后在同一固定 AMCL 闭环中复测：初始位姿前，预检按预期以 code 2 退出，
  `/pause_navigation` 前后均为 `true`，证明工具未额外改变状态；完成初始位姿后
  `health_check` PASS，`--verify-only` PASS，local/global 均为 24 点，最大逐顶点误差
  分别为 `2.36×10⁻⁸ m` 和 `3.02×10⁻⁸ m`。`go2_navigation` 包级测试现为
  `95 passed, 0 errors, 0 failures, 0 skipped`。

## 2026-08-14 Go2 导航足迹校准阶段 2

### 发现与实际实现

- 旧 local/global footprint 都是手写矩形
  `x=[-0.28,0.42] m、y=[-0.24,0.24] m`，padding 为 `0.01 m`。它前方和左右
  偏保守，但有效后边界约 `-0.29 m`，没有包住动态后腿。
- 新增 `footprint_calibrator` 和源码 wrapper，从运行时
  `/robot_state_publisher.robot_description` 解析全部 `box/cylinder/sphere` collision；
  不支持的 mesh 会明确拒绝。工具通过 TF 将 21 个 collision 投影到
  `base_footprint`，采样站立、前进、原地转向和允许的横移并计算精确凸包。
- 采样速度只发到 `/cmd_vel_teleop`，继续经过
  `twist_mux → velocity_smoother → collision_monitor → /cmd_vel`。工具开始前取消旧目标、
  验证停车，采样结束再次锁停且不自动续行。
- 两套 RViz 配置新增默认开启的绿色 `Robot Footprint (Padded)` Polygon，直接订阅
  `/local_costmap/published_footprint`；不是根据 YAML 另画的静态装饰线。

### 已实测数据与参数

- 隔离 Domain 228 在线 SLAM无 GUI 正式采集 220 帧、21 link/帧，共 4620 条 TF。
  站立 40 帧，前进/转向/横移各 60 帧；四组墙钟采样周期 p99 分别为
  `0.110/0.107/0.110/0.111 s`，速度链最终峰值分别为
  `0/0.15/0.30/0.10`。
- 包络极值来源可追溯：后方 `-0.399337 m` 来自右后上腿，前方
  `0.353896 m` 来自 D435，右侧 `-0.201784 m` 与左侧 `0.193757 m` 来自后足。
  旧有效后边界少覆盖约 `0.109 m`。
- 毫米舍入后的 24 顶点凸包面积为 `0.266358 m²`，原始外接半径
  `0.428412 m`。姿态/落足统计尾差 `0.009611 m` 加半个 `0.05 m` 栅格
  `0.025 m`，按 5 mm 向上取整后 padding 为 `0.035 m`；外扩后外接半径
  `0.474170 m`。
- 通过 `nav_tuner` 对 local/global 的 footprint 与 padding 执行四次 LIVE 原子
  set/read-back，并用 `save` 备份受管配置。测试同时发现 PyYAML 会把长 footprint
  折成带转义的多行字符串，现已统一规范为紧凑单行 JSON/YAML 标量并增加回归测试。
- local/global `/published_footprint` 均为 24 点。反变换到 `base_footprint` 后边界分别为
  `[-0.4340,0.3890]×[-0.2370,0.2290] m`。新增 `--verify-only` 使用 Polygon 自身
  时间戳查询 TF，避免最新 TF 与旧消息错时；local/global 最大逐顶点误差分别为
  `4.06×10⁻⁸ m`、`3.47×10⁻⁸ m`，与 `0.035 m` 动态 padding 的内部效果一致。
- 从持久化 YAML 在 Domain 229 冷启动后，`--verify-only` 再次 PASS，local/global
  误差均不超过 `2.11×10⁻⁸ m`；指定隔离域期望值的 `health_check` PASS，`/cmd_vel`
  只有 `/collision_monitor` 一个发布者。两次 Ctrl+C 关闭仿真时均观察到既有
  `champ_gazebo/contact_sensor` 在退出阶段触发 Boost recursive mutex assertion 并以
  `-6` 结束，其余 Nav2/Gazebo server 节点正常清理；该退出期问题不影响已落盘样本，
  但不将整套进程描述为“全部干净退出”。

### 门禁与下一步

- 原始 `result.json`、220 帧摘要 CSV 和 4620 行 TF CSV 保存在
  `go2_navigation/logs/footprint/stage2_footprint_20260814/`；完整顶点、原理、
  RViz/CLI 复核和适用边界见 `docs/footprint_calibration.md`。
- 阶段 2 门禁为 PASS，但只代表当前 Gazebo collision、CHAMP 步态及采样速度的几何
  闭环，不代表真机、跌倒姿态或更高速度已验证；三个 profile 继续保持
  `UNCALIBRATED`。
- `go2_navigation` 已用 `--symlink-install` 构建；本次回归增加 2 项诊断测试后，包级结果为
  `95 tests, 0 errors, 0 failures, 0 skipped`。
- 阶段 2 完成时原定下一停点是阶段 3 Inflation；后续幽灵障碍/近距碰撞已增加前置门：
  必须先完成空射线 clearing 与 `map→odom` 稳定性验证，才以标定后外扩 footprint 的
  外接半径为几何起点，保持 Planner、RPP 和速度不变按 `0.10 m` 单变量步进。

## 2026-08-14 激光雷达近距盲区阶段 1

### 发现与实际实现

- 阶段 0 停点后，用户已在 RViz 亲自确认“设置 AMCL 初始位姿、下发导航
  目标、热调 RPP 速度”三项可用，因此阶段 0 用户复核为 PASS。
- 新增 `obstacle_probe`及源码 wrapper。工具使用 `/spawn_entity`、
  `/set_entity_state`、`/get_entity_state`、`/delete_entity` 和 Gazebo ROS bumper，
  先停止导航并等待最终 `/cmd_vel` 归零，实验后不续行旧目标。
  world 新增官方 `gazebo_ros_state` WorldPlugin，没有新增自定义消息或服务。
- Gazebo 会把 Go2 固定关节合并进 `base_link`，因此工具使用
  `go2::base_link + (0.20,0,0.1177) m` 还原 Velodyne 原点，每次移动后回读位置。
  方块是禁用重力的 kinematic 模型；static 模型被服务移动后 `gpu_ray` 不会稳定看到，
  所以没有沿用该方案。
- Humble `sensor_msgs_py.read_points_numpy()` 会要求 VLP-16 含 `uint16 ring` 的全部
  字段同类型，工具改为 `read_points()` 只取结构化 xyz；高度窗额外余量限为
  2 cm，避免把距雷达约 0.323 m 的地面误当探针。

### 已实测数据与原理

- 隔离 Domain 227 中，正前方 12 个距离每个 3 组×20 帧；原始点云与
  `/scan` 各 720 帧。1.20/1.10/1.00/0.90 m 三组均为 100%，0.80 m 至
  实际 0.153 m 三组均为 0%。正前方可靠检测下限为 0.90 m。
- `/scan` 帧间隔 p50/p95/p99 为 0.158/0.354/0.480 s，原始 Velodyne 为
  0.158/0.338/0.484 s。在 0.90 m，三组中最大距离误差 p95 分别为
  0.62 cm 和 0.45 cm。0.153 m 组产生 442 个接触事件。
- xacro 中 `gpu_ray` 与 Velodyne plugin 的最小量程都是 0.9 m；原始点云与
  `pointcloud_to_laserscan` 派生数据同时从 100% 降为 0%，证明正前方盲区主因
  是量程下限，不是 `-0.05..0.10 m` 二维切片或 TF。
- 左右 90° 在 0.90 m 检测率仍为 100%，但距离误差 p95 超过 5 cm，
  可靠下限因此为 1.00 m。1.20 m 处 170° 三组均为 100%，175/179/180°
  三组均为 0%，确认 `gpu_ray` 在 ±π 有正后方拼接缝。
- 1.00 m 处的 0.10 m 垂直厚度三组均为 100%，0.02 m 仅 55–65%；
  原始点云与 `/scan` 趋势一致，表明这是 VLP-16 垂直角采样限制。
- D435 小样本仅 1 组×3 帧；帧间隔 p50/p95/p99 为 12.34/64.77/72.14 s，
  1.20 m 距离误差 p95 为 6.21 cm。这不满足 3 组×20 帧门禁，本阶段禁止
  将 D435 宣称为可靠盲区 source。同次运行中 Collision Monitor 也因 D435
  时间戳落后 2–6 s 而明确忽略该 source，这是阶段 5–6 必须处理的安全风险。

### 验证、门禁与下一步

- 正式数据保存在 `go2_navigation/logs/blind_zone/`，帧级 CSV、组级 CSV 与
  JSON 可交叉复核。标准方块距离量化、方向、垂直厚度和接触真值链均已
  形成可重复闭环，阶段 1 门禁为 PASS。
- `go2_config + go2_navigation` 已通过 `--symlink-install` 构建；`go2_navigation`
  包级测试为 83 项全部通过，0 errors、0 failures、0 skipped，两个 XML
  与 Python flake8 检查通过。
- PASS 只表示“盲区已被如实量出”，不是“传感器无盲区”。本阶段不修改
  `range_min`、RPP、Inflation 或 Collision Monitor。
- 下一停点是阶段 2 Footprint：从 URDF collision 几何与实际步态样本生成凸包，
  不再沿用未校准的手写多边形。

## 2026-08-14 Nav2 运行时安全调参与监控阶段 0

### 发现与方案

- Nav2 1.1.20 的参数服务“接受并回读”不等于插件内部已更新。InflationLayer、
  Costmap footprint、SmacPlanner2D 与大部分 RPP 数值有动态回调；ObstacleLayer 的
  observation source 子参数没有更新 `ObservationBuffer` 的动态回调。RPP 的
  `use_collision_detection/use_interpolation` 同样不在动态回调中。
- 隔离运行进一步证明：`/local_costmap/local_costmap` 和
  `/global_costmap/global_costmap` 是 controller/planner 内嵌节点；父节点 cleanup 后，
  child 参数服务 callback group 不应答，所以不能在 unconfigured 态可靠 set。
  工具改为健康时先锁速、active 态原子暂存 source 参数（旧 buffer 不会伪生效），再由
  Lifecycle Manager `RESET/STARTUP` 重建插件，最后等待安全监督图审计并恢复。
- BT 的重规划 `1 Hz` 来自 XML `<RateController>`，插件列表、插件类型和 observation
  source 结构也不是安全热更新项，统一归为 `RESTART REQUIRED` 并拒绝 `set`。

### 实际实现与可观察性

- 新增无 ROS 核心模块 `nav_tuning.py`：72 项注册表、类型/范围/安全开关校验、周期分位数、
  scan 计数、路径长度、半格加密保守 clearance，以及保留注释/顺序/层级的 YAML 标量
  定点修改。双文件保存会先备份到 `logs/backups/<timestamp>/`，临时文件语义复核后
  `os.replace`；任一替换失败恢复全部文件并输出 unified diff。
- 新增 `nav_tuner` curses/文本入口和源码兼容 wrapper，支持 `show/set/reset/reset-group/
  profile/save/record/help/quit`、`--snapshot` 和 `--monitor-only`。监控整合 `/scan`、D435、
  两张 raw costmap、发布 footprint、`/plan`、TF 与四级速度链；不增加自定义消息或服务。
- `navigation.yaml` 只把现有 ObstacleLayer 默认值显式化，以便以后白名单持久化；
  `marking`、`clearing`、`footprint_clearing_enabled` 和 RPP collision detection 均由注册表
  拒绝设为 false。现有 `rqt_reconfigure` 保留为辅助 GUI。
- 参数矩阵、包 README 与初学者图解手册已同步。Navigation2 上游为 Apache-2.0；本阶段
  只调用标准参数/Lifecycle 接口并依据 1.1.20 源码分类，没有复制上游实现。

### 已实测与阶段门禁

- 独立 Domain 220 在线 SLAM 无 GUI 的 10 秒 snapshot：`/scan=2.717 Hz`，周期
  p50/p95/p99 为 `0.277/0.714/0.790 s`，`range_min=0.900 m`；Local Costmap 为
  `117 lethal/1555 inflated`，Global Costmap 为 `460 lethal/3613 inflated`，
  `/pause_navigation=false`。
- LIVE：local inflation `0.30→0.40 m` 后代表性 inflated 格 `1623→2129`；padding
  `0.01→0.03 m` 后发布足迹外框约 `0.72×0.50→0.76×0.54 m`；RPP 速度
  `0.27→0.26 m/s` 原子 set/read-back 成功，随后全部恢复 YAML 基线。
- LIFECYCLE RELOAD：local scan persistence `0.0→0.2→0.0 s` 两次完整重载成功；期间
  `/pause_navigation=true`，最后 8 个 Nav2 节点 active、参数回读正确、
  `/pause_navigation=false`，旧目标不续行。Domain 223 进一步从已确认的最终
  `/cmd_vel.linear.x=0.1` 开始执行 reload，工具在零速度落地后才拆除插件，重载和恢复
  YAML 基线均成功。高负载下曾发现 3 秒状态查询会误判迟到响应，现改用单节点 5 秒、
  整体 60 秒健康窗口。重规划结构项热改被明确拒绝。
- `/cmd_vel` 只有 Collision Monitor 一个 publisher。阶段 0 不包含导航目标或碰撞场景，
  不宣称障碍安全参数已标定。
- `go2_navigation` 包级 pytest/colcon 测试为 76 项全部通过；当前工作区累计结果为
  `210 tests, 0 errors, 0 failures, 6 skipped`。6 个 skipped 均是
  `pointcloud_to_laserscan` 上游因 cppcheck 2.7 性能问题主动跳过的既有静态检查，
  不是本阶段功能失败。
- 同一 10 秒窗口没有 D435 点云；另一次隔离样本约 `0.30 Hz`。这和
  `/scan.range_min=0.9 m` 都只列为后续调查线索，不把“已配置”描述为冗余已验证。
- 阶段 0 门禁为 PASS；下一停点是阶段 1 的可重复 Gazebo 障碍探针与近距盲区量化，
  尚未开始。
- 用户复核发现旧版 `nav_tuner` 以 transient-local 订阅 Humble 的 volatile
  `published_footprint`，导致 QoS 警告和足迹面板为空；同时 SIGINT 已关闭 rcl context 后，
  finally 再调用 `shutdown()` 会抛出 `rcl_shutdown already called`。现已将 footprint 订阅
  改为 reliable + volatile，退出改用 `try_shutdown()`，并容错不支持 `curs_set()` 的 IDE
  终端。独立 Domain 224 固定 AMCL 复测收到 local/global 各 4 个 footprint 顶点，无 QoS
  警告，交互界面 Ctrl+C 正常以状态 0 退出。

## 2026-08-14 动态障碍代价图链路修复

### 现象、数据链与根因

- 肉眼现象是 RViz 已勾选 `Local Costmap` 仍空白，Gazebo 在现有路径上加入
  障碍后也看不到绕行。现场已确认 `controller_server`、`planner_server` 均为
  lifecycle `active`，全局和局部 obstacle layer 也都在订阅 `/scan`，因此不是
  RViz 复选框、节点或插件未启动。
- 修复前 `/scan` 有 428 个有限量测，其中 164 个在 2.5 m 内；但两个 source 层
  `scan.max_obstacle_height` 运行值都是 `0.0 m`，而 `odom -> velodyne` 高度约
  `0.323 m`，所以所有激光观测都被高度门槛过滤。`/local_costmap/get_costmap`
  因而返回 10000 个全零格。将同一帧 scan 以 `base_footprint` 的 `z=0`
  对照重发后，立即出现 117 个致命障碍格和完整膨胀梯度，证明根因是
  source 高度门槛。
- Nav2 Humble 的外层 obstacle layer `max_obstacle_height` 默认为 `2.0 m`，但不会
  级联给各 observation source；[source 级参数文档](https://docs.nav2.org/configuration/packages/costmap-plugins/obstacle.html)
  和 [Humble `obstacle_layer.cpp`](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/obstacle_layer.cpp)
  均显示 source 级默认值是 `0.0 m`。上游 Navigation2 采用 Apache-2.0，本轮只显式配置已有参数，未复制上游代码。

### 实际实现与可观察性

- `global_costmap.obstacle_layer.scan` 与 `local_costmap.scan_layer.scan` 都显式设置
  `max_obstacle_height: 2.0`；`marking/clearing`、`5×5 m` rolling window、RPP、碰撞
  保护和终点 `0.30 m` 锁存区都保持不变。配置回归测试锁定两个 source
  的高度、marking 与 clearing，防止以后只改外层参数。
- 文档已补充 RViz 操作：放大机器人附近，只保留 `Local Costmap`，暂时关闭
  `Static Map` 和 `Global Costmap` 避免覆盖，再通过 lifecycle、`/scan` 频率、
  source 高度参数和 costmap update 逐层排查。
- RPP 不发布额外的“局部路径”；`/received_global_plan` 是交给控制器的平滑全局
  路径。RPP 用 Local Costmap 预测追踪弧上的碰撞，行为树在锁存区外继续以
  1 Hz 调用 SmacPlanner2D 重算 `/plan`。

### 已实测

- 独立 Domain 190 冷启动固定 AMCL 仿真后，两个 source 参数均回读为
  `2.0 m`，`/scan` 约 `7.25 Hz`。局部代价图为 `100×100`，格子统计从
  修复前 `{0: 10000}` 变为含 212 个致命障碍格与多级膨胀梯度。
- 把 `0.55×0.80×1.00 m` 障碍放到原路径前方约 `1.2 m` 处：局部致命格从
  319 增至 409，全局致命格从 3940 增至 4058；第一条新 `/plan` 在
  `1.251 s` 内到达，后续路径与障碍中心的最小距离从 `0.00 m` 增至
  `0.712–0.785 m`。8 秒观测中机器人中心与障碍最小距离为 `1.122 m`，
  随后正常取消测试目标。
- 独立 clearing 实验在障碍中心 `0.45 m` 半径内读取全局代价图：放置前
  252 个格全为自由，放置后出现 28 个致命格及膨胀梯度，删除模型后
  `2.5 s` 内恢复为 252 个自由格。
- GUI 模式的独立 Domain 191 实测 `/depth/color/points` 发布者为
  `GazeboRealsenseNode`，有效数据频率约 `1.24–2.46 Hz`。D435 继续作为近场补充，
  本轮主闭环仍以已验证的 Velodyne `/scan` 为准，未关闭任何安全源。
- `pointcloud_to_laserscan`、`go2_navigation_bt_plugins`、`go2_navigation` 重新构建
  成功；`colcon test-result --verbose` 为 `196 tests, 0 errors, 0 failures, 6 skipped`。
- 发布检查点已作为提交 `82e1e759db142704f0b6114831afc4510f90908b`
  和注解标签 `V1.3.0` 推送；标签仅表示“可以实现基本的基于全局规划器的导航”，
  动态障碍修复位于该标签之后。

## 2026-08-12 建图、定位与导航抖动修复

### 根因与方案

- 固定 `/map` 来自 `map_server`，只含保存范围；VLP-16、D435 和 costmap 不会扩展它。
  默认流程改为 Slam Toolbox 在线模式，固定图流程改用官方 AMCL。
- 旧运行中单帧 NDT 拒绝立即触发 `/pause_navigation`，形成速度反复归零并最终
  `Failed to make progress`。NDT 清锁 fitness 还保留不适配的 1.0，而健康值约 3.5。
- 六自由度 NDT 使用振动的 `base_link` 直接发布全局 TF；二维导航现统一使用
  `base_footprint`。`lidar_ndt` 关闭直接 TF，由 `robot_localization` `two_d_mode` 发布
  平面 `map -> odom`。
- 2D 激光链未接通且 D435 配置订阅错误话题。现统一把 VLP-16 水平切片转换为 `/scan`，
  D435 使用实际 `/depth/color/points`，局部感知按高度排除地面和机身点。

### 实际实现

- `simulation_navigation.launch.xml` 成为唯一推荐入口：
  `navigation_mode:=online_slam|static_map`（默认在线）、
  `localization:=amcl|lidar_ndt|ndt_cuda`（固定图默认 AMCL）、`map_session`、`map_dir`。
  旧在线入口保留兼容 wrapper，`static_bundle` 仅作弃用别名。
- 固定图新增 `map_server + amcl + lifecycle_manager_localization`；在线模式由
  Slam Toolbox 唯一发布 `map -> odom`。标准 `nav2_bt_navigator` 的五个 action endpoint
  显式重映射到 `/navigate_to_pose_raw`，已删除仅负责改名的派生包。
- 感知和导航全部使用 `base_footprint`。全局障碍层使用 `/scan`；局部代价图与
  Collision Monitor 使用 `/scan + /depth/color/points`。传感器过期联调值按本机
  Gazebo 负载实测从计划的 1 秒调整为 2 秒，碰撞保护没有关闭。
- 安全监督增加 `HEALTHY/DEGRADED/LOST` 滞回：单帧 NDT 拒绝不停车，连续失效 2 秒
  或明确重定位请求才停车；fitness 5.25 且连续 5 个健康样本后清锁。ROS 图与 lifecycle
  检查降到 1 Hz。
- 默认 Rotation Shim 的内层 RPP 保持 0.27 m/s 和碰撞预测，控制器 10 Hz，普通容差为
  0.30 m/0.15 rad，
  `min_approach_linear_velocity=0.10 m/s`。BT 降为 20 Hz，action/service 应答门槛
  由 20 ms 改为 500 ms，避免把正常调度抖动误判为服务失败。
- `goal_guard` 改为异步 action 转发和取消传播；bridge 改用单线程执行器；三个 Python
  辅助进程在 `spin_once` 间加入 5 ms 让步。RViz 降为 20 FPS，点云和 costmap 默认关闭。

### 已实测

- 标准 `bt_navigator` 重映射后，公开 `/navigate_to_pose` 仅由目标门禁提供，内部
  `/navigate_to_pose_raw` 仅由 Nav2 提供。
- AMCL 独立启动时 `map_server`、`amcl` 均为 lifecycle `active`，公开
  `/amcl_pose`；这是固定图模式 `Localization: active` 的数据来源。
- 固定 AMCL 统一入口在设置 `/initialpose=(3,0,0)` 后，map_server、AMCL、Nav2 lifecycle
  全部 active，健康检查 PASS；第一个短目标成功。第二个反向目标的恢复动作把机器人带入
  旧静态图的 lethal/unknown 区，随后目标门禁按设计拒绝后续目标，因此固定图 12 目标未通过，
  需要在与场景匹配的 PGM 上继续验收，不能通过关闭门禁或碰撞保护规避。
- 默认在线无界面冷启动健康检查通过，`/pause_navigation=false`；
  `map -> base_footprint` 的 z/roll/pitch 为二维零值。地图从 `197×209` 增长到
  `234×223`（0.05 m/cell）。
- 12 个连续短目标全部 `SUCCEEDED`、无 recovery 和 `aborted`；另一个约 1.2 m 转向目标
  在传感器出现一次超过 2 秒的 Gazebo 调度停顿后进入恢复，本轮不把它计为长目标通过。
- 辅助节点 CPU 从旧运行约 56–85% 降为 bridge 16.8%、goal guard 18.2%、安全监督
  16.6%；`gzserver` 仍约 174%，是剩余负载主项。VLP-16 xacro 标称 10 Hz，本轮在线
  负载实测约 6–7 Hz，因此 10 分钟控制周期 miss <1% 尚未达到或验证。
- 源码测试扩展到 32 项并通过，`go2_navigation` 完成构建。静态 AMCL 12 目标、
  60 秒协方差、移动障碍及失效注入仍列为后续验收项。

### 上游、许可与边界

- 直接复用 Navigation2 Humble（Apache-2.0）的 AMCL、SmacPlanner2D、RPP、Collision
  Monitor 和生命周期管理；Slam Toolbox 为 LGPL-2.1，pointcloud_to_laserscan 为 BSD，
  robot_localization 为 BSD-3-Clause，lidar_localization_ros2 为 BSD-2-Clause。
- 调研的 Go2 社区项目并不直接替换当前架构：`go2_robot` 导航仍在开发，
  `unitree_go2_nav` 面向 Jazzy/RTAB-Map，`autonomy_stack_go2` 使用 Point-LIO 与独立栈。
  当前选择是复用成熟 ROS 2 组件并保留 Humble/Gazebo Classic/CHAMP 兼容性。
- 本轮结论只适用于 RTX 4060 上的 Gazebo Classic 仿真。真值里程计不适用于真机，
  不得把结果描述为真机定位或导航已验证。

### 2026-08-12 地图质量复核

- `home_01` 的 LIO-SAM PCD 轨迹范围约 `17×8 m`，但点云派生栅格达到约
  `106×108 m`；约 7950 个点位于地图原点半径 15 m 外。原 PCD 转图参数又以单格
  `max_z-min_z >= 0.4 m` 判障碍、`min_points_per_cell=1`，会把多高度重叠点和单点噪声
  变成室内黑色散点。该图不再作为 AMCL 推荐输入。
- 固定 AMCL 现只校验并消费 Slam Toolbox 原生 `map.yaml/map.pgm`；不再强制无关的
  `GlobalMap.pcd/map_bundle.yaml`。NDT 实验档仍保留严格同源地图包和哈希校验。
- 健康检查发现 `teleop_twist_keyboard` 直接发布 `/cmd_vel` 时会明确提示关闭并重映射到
  `/cmd_vel_teleop`。最终 `/cmd_vel` 必须仍由 Collision Monitor 唯一发布。
- 现场复核确认一次固定图启动的真实命令为
  `map_dir:=/home/hao/go2_maps/online/home_02`，`map_server` 临时参数文件也明确指向
  `home_02/map.yaml`；系统没有自动回退到 `home_01` 或根目录 `latest`。
  `home_02/map.pgm` 为 `378×184、0.05 m/cell` 的 Slam Toolbox 原生图，文件内部自由区
  较干净；RViz 出现与文件不一致的旧画面时，应检查旧 RViz 缓存画面、`map_server`
  lifecycle 和 `/map` 发布者，而不能仅凭截图判定加载目录。
- 当前地图目录约定：`~/go2_maps/online/latest` 是 `save_online_map.sh` 更新的最近在线
  会话软链接，当前指向 `home_02`，推荐给固定 AMCL 日常启动；
  `~/go2_maps/online/home_02` 用于可复现实验；`~/go2_maps/latest` 保留给 LIO-SAM/NDT
  流程，不作为 AMCL 推荐默认图。`map_session:=new` 明确从空白 pose graph 开始。

### 2026-08-12 AMCL 协方差失信保护与可视化默认值

- 现场 `/amcl_pose` 实测标准差为 `x=0.78 m、y=1.18 m、yaw=1.56 rad`；RViz 中紫色
  椭圆对应 x/y 协方差，黄色长扇形对应 yaw 协方差。该现象证明 AMCL 已失配，而不是
  激光视野或规划通道；错误 `map -> odom` 会让机器人显示位置跳变并诱发左右纠偏。
- AMCL 在仿真真值 `/odom` 档把运动模型噪声 `alpha1..5` 从 0.2 降为 0.05，启用
  `likelihood_field_prob` beam skipping，并把采样光束从 60 增至 90。此参数只作为
  Gazebo 基线，不外推为真机参数。
- 安全监督与目标门禁新增协方差滞回：位置/yaw 标准差超过 `0.75 m/0.75 rad` 立即锁速
  并拒绝新目标，回落至 `0.55 m/0.50 rad` 才恢复可信；健康检查输出实际标准差和
  `2D Pose Estimate` 操作提示。
- `simulation_navigation.launch.xml`、建图入口、兼容 wrapper 与 `simdog/start.sh` 现默认
  打开 Gazebo GUI。无界面只作为自动化/性能测试的显式 `gui:=false` 选项。
- 独立 ROS Domain 187 加载 `home_02` 验证：`map_server` 与 `amcl` 均进入 `active [3]`，
  运行参数确认为 `laser_model_type=likelihood_field_prob`、`do_beamskip=true`、
  `alpha1=0.05`。完整仿真必须冷启动后再做连续目标验收。

### 2026-08-12 初学者术语与 RViz 图解手册

- 新增 `文档/Go2导航建图与RViz初学者图解手册.md`，用本次真实截图解释 AMCL 黄色
  yaw 协方差、紫色 x/y 协方差、固定地图范围、二维地图黑点、三维 PCD、Slam Toolbox
  面板和 Navigation 2 状态。
- 手册覆盖 ROS 2 通信、TF 主链、传感器、SLAM/AMCL/NDT、Nav2、costmap、碰撞保护、
  速度链、主要参数、缩写与包名，并提供“肉眼现象 → 首查项”索引和操作命令。
- 根目录协作规则已规定：新增用户可见术语、RViz 元素或典型故障截图时必须同步维护该
  手册；颜色只能作为辅助，最终以 Display 名称和来源话题为准。

### 2026-08-13 `forward_rpp` 终点航向摆动修复（控制实现完成，整体验收未通过）

#### 现场根因

- 新证据为：目标距离约 `0.26 m < 0.30 m`，剩余 yaw 误差约 `2.47 rad/142°`；
  12 秒内 `/cmd_vel_nav.angular.z` 换向 41 次并夹带前进速度，而 AMCL yaw 标准差只有
  `0.011–0.022 rad`。Collision Monitor 没有造成上游 `/cmd_vel_nav` 换向，根因是
  Humble RPP 在 XY 容差边界反复切换追位置和终点定向。
- 直接换成上游 Rotation Shim 后，专项运行又发现 Humble 的 `setPlan()` 每次都会重置
  shim 内部 `PositionGoalChecker`。默认行为树的 1 Hz 重规划因此仍可能让终点定向退出；
  不能靠继续放宽 yaw 或关闭碰撞保护规避。
- `closed_loop=true` 的首次 A/B 还发现 `/odom/ground_truth.twist.angular.z` 含有四足落足
  周期内约 `-0.74…+1.14 rad/s` 的瞬时机身摆动，会直接进入 shim 的角加速度反馈。

#### 实际实现

- `forward_rpp` 外层改为
  `nav2_rotation_shim_controller::RotationShimController`，内层保持原 RPP 全部前向速度、
  前视和碰撞参数。启用 `rotate_to_goal_heading=true`、`closed_loop=false`；转速
  `0.45 rad/s`、角加速度 `1.0 rad/s²`、旋转预测 `1.0 s`、进入/退出阈值
  `1.40/0.40 rad`、采样距离 `0.50 m`。内部 RPP 的 `use_rotate_to_heading=false`，普通
  弯道保持前进画弧，接近侧后方的路径才由外层 shim 停车对齐。普通 goal checker 为
  `0.30 m/0.15 rad`；PoseProgressChecker、速度平滑和 Collision Monitor 未变。
- `closed_loop=false` 仅避免把当前 1 秒滑窗 `/odom.twist.angular.z` 用作低延迟角加速度
  反馈；目标姿态仍由 TF 与 GoalChecker 闭环判定，不是关闭导航闭环。
- 纯旋转分层失败后按 CHAMP 层继续单变量修复：Go2 gait 的 `stance_depth` 从
  `0.01 m` 改为 `0.0 m`，取消支撑脚额外向地面下压。该值与 BSD-3-Clause 的
  CHAMP robots Go1 仿真基线一致，并且本机 A/B 明确降低旋转漂移；其余 gait/PID/摩擦
  候选没有同时改善双向增益和漂移，均已撤回。
- BSD-3-Clause 包 `go2_navigation_bt_plugins` 的 `TerminalPathLatch` 已改为有状态 BT
  装饰节点。现场进一步确认旧实现每次规划成功都调用 `resetChild()`，把嵌套
  `RateController` 重置为首次运行，造成 12 秒 40 条 `/plan`；新实现覆盖装饰节点默认
  `executeTick()` 生命周期，规划成功后保留 RateController 计时状态。路径末端按
  `0.075 m/0.01 rad` 与原始目标匹配，实时 `map→base_footprint` TF 必须确认机器人同时
  位于原始目标和路径末端 `0.30 m` 内才锁存。新目标必须先成功生成新路径；定位短时漂出
  不解除，`halt()` 或 recovery 清除；TF 暂不可用且未锁存时继续规划并限频诊断。
- `GridBased.tolerance` 已从 `0.25 m` 收紧为 `0.0 m`，不可达目标明确失败；显式设置
  `use_final_approach_orientation=false`，路径末端保持 RViz 目标 yaw，不使用附近替代终点。
- Goal Guard 新增 transient-local 只读话题 `/navigation/accepted_goal`；诊断现在分别计算
  机器人到原始目标、机器人到路径末端、路径末端到原始目标，并报告锁存前规划频率与
  锁存后 `/plan` 数量，避免把规划器替代终点或 AMCL 跳变误判为控制器问题。
- 仿真里程计增加 1 秒展开 yaw 窗口，只平滑输出 `/odom.twist.angular.z`，不修改姿态；
  它用于降低四足落足瞬时速度噪声，不外推到真机，也不再作为 shim 的闭环输入。
- 冷启动期间还定位并修复 `/scan` 在临时订阅者退出后停更：Velodyne 点云已经在目标
  frame，`target_frame` 改为空以走直接订阅。vendored
  `pointcloud_to_laserscan` 增加默认关闭的 `always_subscribe` 参数，导航档显式设为
  `true`，避免输出订阅图变化触发 lazy 订阅竞态。三次临时 `/scan` 订阅/退出后仍持续
  约 `6.6–7.9 Hz`，`/pause_navigation=false`，Collision Monitor 和安全监督未关闭。
- `go2_navigation/package.xml` 显式声明 `nav2_rotation_shim_controller` 与新 BT 包；
  YAML/依赖/行为树/仿真角速度过滤均增加测试。新增只读命令
  `rotation_diagnostics`，可在 `manual`/`navigation` 模式汇总四级速度差、实际旋转增益、
  换向、终点线速度、真值/odom/TF 误差、`map→odom` 单步修正、重规划和安全锁状态。
- 根 README、导航 README、本文档与
  `文档/Go2导航建图与RViz初学者图解手册.md` 已同步，明确 `/cmd_vel_teleop` 是安全手动
  入口、五层故障树和纯旋转测试方法；外部 Go2/CHAMP 项目仅作设计参考，不复用其
  里程计速度倍率补丁。

#### 验证结果与剩余边界

- 当前源码 `go2_navigation` pytest 共 62 项通过。`TerminalPathLatch` 的 12 个 C++ gtest
  覆盖真实 TF 双距离、边界漂移保持、`0.075 m/0.01 rad` 路径匹配边界、同 XY 新 yaw、
  旧路径不匹配、TF 失败、`halt()/recovery` 清除，以及真实 `RateController` 连续快速
  tick 不突破周期，全部通过。两包 `colcon build/test` 通过；当前工作区累计
  `colcon test-result` 为 196 项、0 failure、6 skipped（包含此前其他包结果）。
  `go2_navigation/setup.py` 已声明 pytest 测试依赖，因此计划中的 `colcon test` 会实际
  运行 62 项 Python 测试，不再回退为“Ran 0 tests”的 unittest 空跑。
- 旧锁存实现曾连续完成两个同一 XY 的内部 `±90°` action，并将连续左右摆动从现场
  41 次降为 0/1 次；这只能作为问题链证据，不能替代当前实时 TF 锁存的重新实测。
- 已在安全链实际完成 `±0.15/0.25/0.35/0.45 rad/s` 纯旋转采样。四级速度链最终值与
  请求值误差不超过 `0.02 rad/s`，纯旋转线速度为零，Gazebo 真值、`/odom` 和
  `odom→base_footprint` 累计 yaw 差不超过 `0.03 rad`，因此命令传递与真值里程计适配
  通过。实体层未通过：原始摩擦 `0.6` 下 `+0.45 rad/s` 稳态增益约 `32.5%`、每 90°
  等效漂移约 `0.379 m`；`-0.45 rad/s` 增益约 `93.8%`、等效漂移约 `0.130 m`。
  `±0.25/0.35` 也因每 90° 等效漂移约 `0.19–0.32 m` 失败。四脚摩擦系数单变量提高到
  `1.0` 后正向增益仍约 `33.5%`、漂移约 `0.410 m/90°`，无改善，已恢复 `0.6`。
- `stance_depth=0.0 m` 是唯一保留的底层 A/B：`+0.45 rad/s` 改善到约 `69.7%`、
  `0.113 m/90°`，`-0.45 rad/s` 为约 `61.2%`、`0.064 m/90°`，方向差异和漂移明显
  缩小但仍未全部过线。上游 Go1 PID `180/20/7`、`stance_duration=0.20/0.30 s`、
  `swing_height=0.05 m` 均未同时改善增益与漂移并已恢复原值；这些结果说明剩余问题不能
  靠单纯提高摩擦、PID 或步幅解决，需要后续审查 CHAMP 开环落足轨迹/接触力闭环。
- 新状态机已做一次无界面同 XY `+90°` 固定图专项实测：action 在 `4.8 s` 内
  `SUCCEEDED`；四级速度终点段均为 `max|linear.x|=0`、`max|angular.z|=0.45 rad/s`、
  0 次换向，终点前仅收到 1 条路径，进入双终点容差 `0.30 s` 后 `/plan=0`。这证明本轮
  高频重规划和 Rotation Shim/RPP 反复切换修复有效。该次 AMCL 随后产生 `0.414 m`
  单步 `map→odom` 修正，停稳后机器人到原始目标/路径末端约 `1.228/1.208 m`，所以整体验收
  仍为 FAIL，不能执行或宣称 12 目标通过。下一步先用 `2D Pose Estimate` 准确初始化并
  等待 AMCL 稳定 10 秒；若仍超过 `0.10 m/rad`，重建或修正 `home_02` 地图/定位，而不是
  调控制器。定位通过后再记录 `/plan`、四级速度、`/amcl_pose` 完成 12 目标。
- 用户随后现场确认“好很多，终点不会再来回左右摆动”，验证了锁存修复的肉眼效果。
  同次反馈中的旧诊断在结束时机器人到原始目标/路径末端仍为 `5.401/5.393 m`，路径末端
  到原始目标仅 `0.010 m/0.000 rad`，`map→odom` 单步为 `0.020 m/0.009 rad`。这组数据
  表明采样仍在途中且定位稳定，不能按终点误差判 FAIL。
- `rotation_diagnostics` 的 navigation 模式现改为等待新目标并在每个新目标后重新计算
  `120 s` 获取期限；进入双终点 XY 容差后才执行默认 `10 s` 终点采样，action 成功后再
  等待 1 秒停稳。未进入终点返回 `INCOMPLETE`/退出码 2；同时报告标准 action 状态和
  终点外原地旋转片段、路径夹角及 `/plan` 关联。该改动用于区分“还在路上”和真正终点失败。
- 本轮无界面冷启动已回读运行参数：shim 进入/退出阈值为 `1.40/0.40 rad`、内部 RPP
  `use_rotate_to_heading=false`、普通 yaw 容差为 `0.15 rad`，controller lifecycle 为
  `active`。一次同 XY `+90°` 目标中，终点控制段 `linear.x=0`、角速度上限
  `0.45 rad/s`、换向 0 次且锁存后没有新 `/plan`；action 随后返回 `SUCCEEDED`。
  但该次 `map→odom` 最大单步位置修正达到 `0.586 m`，且成功晚于 10 秒终点采样窗口，
  因此只把它记作控制链证据，不把最终位置/yaw 当作当前地图的验收结果。另一次未发送
  新目标的实跑已确认诊断输出 `INCOMPLETE` 并返回退出码 2，符合新增接口语义。

上游依据为 Navigation2 的
[Rotation Shim 官方说明](https://docs.nav2.org/configuration/packages/configuring-rotation-shim-controller.html)
和 [Humble RPP 源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp)；
Nav2 为 Apache-2.0，本项目新增 BT 插件为 BSD-3-Clause。

以下 2026-08-11 及更早记录保留为历史证据；其中“默认 NDT”、独立在线入口和派生
bt_navigator 等描述已被本节当前架构取代。

当前已增加仅面向 Gazebo 的动作控制包 `go2_behaviors`，可执行打招呼、点头、
伸展、趴下、挥爪和简单舞蹈，并使用 `stand` 从保持趴下恢复。动作复用现有
CHAMP、`ros2_control` 和标准 `FollowJointTrajectory` 接口，不等同于真机
Unitree Sport API。

当前已固定引入 Unitree 官方 `unitree_ros2 v0.3.0` 的 `unitree_go`、
`unitree_api`，并增加 `go2_unitree_sim_bridge`。它让所列 Sport API 上层程序在
Gazebo 与真机间复用官方消息和话题，但不承诺真机固件行为等价。

焊死腿关节、通过 planar-move 滑行的旧简化工作空间 `go2_ws/` 已于
2026-08-06 删除；其专用环境脚本 `scripts/setup_go2_ws.bash` 同时删除。后续
功能开发统一基于 `simdog/`，不再维护两套机器人实现。

统一入口：

```bash
cd /home/hao/ROS/Go2_Bilibili_zhao-main
bash scripts/install_dependencies.sh
bash scripts/install_gpu_dependencies.sh
bash scripts/build_workspaces.sh
source scripts/setup_simdog.bash
```

## 2026-08-11 前向导航、在线 SLAM 与仿真里程计闭环

### 目标与上游选择

- 解决 MPPI Omni 让 Go2 斜向平移、速度偏慢、最后才转到目标朝向的问题；默认改用
  Nav2 Humble `RegulatedPurePursuitController`，保留 MPPI 为显式对照档。
- 明确固定 `/map` 不会自动扩大，新增独立 `pointcloud_to_laserscan + slam_toolbox`
  在线二维建图导航入口，支持从空图开始、保存 PGM/YAML 与 pose graph、续建。
- 上游依据：Nav2 RPP 与 SmacPlanner2D 均来自 Navigation2 Humble（Apache-2.0）；
  Slam Toolbox 为 LGPL-2.1，`pointcloud_to_laserscan` 为 BSD。未引入新的自研规划、
  控制或 SLAM 算法，只增加项目编排、门禁与仿真适配。

### 实际实现

- `controller_profile:=forward_rpp` 成为默认档：期望前向速度 `0.27 m/s`、大角度先
  转向、曲率降速、碰撞预测；`omni_mppi` 完整保留。控制器公共配置与两个档案分离，
  避免 Humble 参数互相污染。短距离进展阈值由 `0.5 m/10 s` 放宽为
  `0.10 m/15 s`，避免正常四足短目标被误判卡住。
- 新增 `simulation_online_mapping_navigation.launch.xml`、`online_slam.launch.py`、
  `online_mapping.yaml` 和在线 RViz 配置。在线模式不启动 NDT/map_server，由
  Slam Toolbox 唯一发布 `map -> odom`，目标门禁直接使用实时 `/map` 与 TF。
- 新增 `save_online_map.sh`：原子保存 `map.pgm`、`map.yaml`、`slam.posegraph`、
  `slam.data` 并维护 `~/go2_maps/online/latest`。修复 Humble `ros2 service call`
  在不同补丁版本中输出 `result=0`/`result: 0` 的兼容解析。
- 新增 `/navigation/stop` 与 `/navigation/resume`。停止会取消公开/内部 action、锁存
  `/pause_navigation`，并通过 twist_mux 最高优先级 `/cmd_vel_stop` 连续注入零速度；
  恢复只解锁，旧目标不续行。
- 运行诊断发现真正的剩余阻断不是 RPP，而是 CHAMP 足端 `/odom` 严重低估 Gazebo
  位移：速度链传到最终 `/cmd_vel=0.10 m/s` 时，真值移动约 `0.093 m`，旧 `/odom`
  仅累计约 `0.007 m`，导致 Nav2 `Failed to make progress`。两个统一 Gazebo 导航
  入口现关闭 `footprint_to_odom_ekf`，由新增 `simulation_odom` 把
  `/odom/ground_truth` 首帧转换为零原点 `/odom` 与唯一
  `odom -> base_footprint`。机器人仍由 CHAMP 步态与 Gazebo 物理执行；真机和普通
  分组件入口不使用真值适配器。
- Gazebo P3D 真值更新率由 10 Hz 提高到 50 Hz，RPP TF 容差调到 0.2 秒；NDT 诊断
  过期锁联调默认放宽为 3 秒。目标门禁、碰撞监控、最终速度唯一出口均未关闭。
- RViz 增加 `Navigation 2` 面板、Nav2 Goal 工具、在线 `Live SLAM Map`/`SLAM Scan`；
  文档和 `AGENTS.md`/`CLAUDE.md` 增加“现象到根因、主流方案、参数含义、具体按键与
  已实测/推断分离”的教学规则。

### 运行验证（已实测）

- 在线模式冷启动健康检查 PASS，8 个 lifecycle 节点 active；`/scan` 约 9.76 Hz，
  RPP 运行时参数回读为期望速度 `0.27 m/s`。短目标 action 返回状态 4
  （`SUCCEEDED`），`/cmd_vel_nav.linear.y=0`，真值与新 `/odom` 位移完全一致。
- 调整进展阈值后的在线目标从约 `(0.38,0.05)` 到 `(0.80,0)` 在约 2 秒内成功，
  无 recovery；最终速度峰值约 `vx=0.237 m/s`、`vy=0`。
- 在线地图在机器人移动中由 `198×209` 扩大到 `230×209`（0.05 m/cell），证明
  `/map` 动态更新；保存产生四个非空文件，随后以 `map_session` 续建加载成功。
- 锁存停止在仍持续发布 `0.10 m/s` 键盘输入时调用成功，最终 `/cmd_vel` 在
  0.3 秒后为零并保持；健康状态下 `resume` 成功。
- 固定地图模式发布 `(4,0)` 初始位姿后 NDT 输出约 `(3.81,0.02)`，10 个 lifecycle
  节点 active、`health_check` PASS；`(4.30,0)` 目标返回 `SUCCEEDED`，最终
  `linear.y=0`，Gazebo 真值移动约 `0.20 m`。
- P3D 调整后 `/odom` 实测约 `48.9 Hz`，在线健康检查再次 PASS。
  `go2_navigation` 与 `go2_description` 构建通过；pytest `19 passed`，Python/YAML/
  launch 解析、flake8、xmllint、规则镜像与 `git diff --check` 全部通过。

### 边界与下一步

- 真值里程计仅是 Gazebo 导航基准，不代表真实 Go2 的状态估计已经解决。后续真机或
  高保真验证必须融合足端接触、IMU 与机身速度，不能使用 `/odom/ground_truth`。
- 当前固定 PGM 只有保存时的已知范围，本来不会扩图；要学习未知区域必须选在线入口。
  在线二维投影会丢失高度信息，正式三维建图仍使用 LIO-SAM。
- 本轮完成短目标与停止/保存/续建验证，未完成 12 次导航、移动障碍、长走廊回环和
  MPPI/RPP 量化对照。

## 2026-08-09 阶段一：室内平地自主导航闭环

### 阶段目标与调研

- 在完整四足仿真上建立「室内平地导航闭环」，废弃旧的轮式 AMCL + DWB 配置
  （`go2_config/config/autonomy/navigation.yaml`，存在轮式模型、无效传感器话题
  `/scan`、`/zed/...`、`use_sim_time: False`、`robot_radius=0.22`、
  `bt_navigator.odom_topic` 硬编码 LIO-SAM 内部话题等问题）。
- 定位默认引入 BSD-2-Clause 的 `lidar_localization_ros2` v1.2.0（固定提交
  `b40a02d4341245c30007159e94d4d13081045327`），复用其 NDT/GICP、自动初始定位
  （BBS，纯 C++ 无 gtsam）、诊断、重定位、PCD 转二维地图和评测能力；现有 CUDA
  NDT（`ndt_relocalization`）保留为 `localization:=ndt_cuda` 实验档。
- 规划器 SmacPlanner2D、控制器 MPPI（Omni 全向），速度限制与 CHAMP 步态一致；
  footprint 用机身多边形而非圆形半径。
- 控制链 `Nav2/键盘/Unitree Move -> twist_mux -> velocity_smoother ->
  collision_monitor -> /cmd_vel -> CHAMP`，行为动作/趴下/定位失效时发布
  `/pause_navigation` 锁住输入并输出零速度。

### 实际操作

- 新增 `simdog/src/go2_navigation`（ament_python）：
  - `go2_navigation/build_map_bundle.py`：`GlobalMap.pcd` → `map.yaml/pgm` +
    `map_bundle.yaml`（SHA-256 清单），支持 `--x-min/--x-max/--y-min/--y-max`
    裁剪聚焦导航区域、`--obstacle-height-m`、`--min-points-per-cell`。
  - `go2_navigation/validate_map_bundle.py`：启动前校验三件套存在且哈希匹配。
  - `go2_navigation/health_check.py`：话题 / TF 链 / 控制器健康检查。
  - `launch/navigation.launch.py`：地图校验 → 定位 → Nav2（自起节点 +
    lifecycle_manager）→ 安全链 → RViz。
  - `launch/localization.launch.py`：定位入口（默认 `lidar_ndt`，事件驱动
    生命周期；实验档 `ndt_cuda`）。
  - `launch/mapping.launch.xml`：建图入口（Gazebo + LIO-SAM）。
  - `scripts/save_map.sh`：保存 PCD 并生成同源地图包。
  - `config/navigation.yaml`、`twist_mux.yaml`、`localization_ndt.yaml`。
- `ndt_omp_ros2` 更新到上游 `rsasaki0109/ndt_omp_ros2` humble 分支（含 NDT
  诊断成员 `last_correspondence_count_` 等），修复与 `lidar_localization_ros2`
  的 API 不匹配。
- `realsense_ros_gazebo/xacro/depthcam.xacro` 开启 D435 深度点云
  （`<pointCloud>true</pointCloud>`），话题 `/depth/color/points`。
- `go2_unitree_sim_bridge` 速度输出话题参数化为 `cmd_vel_topic`（默认
  `/cmd_vel`，导航模式设为 `/cmd_vel_unitree` 经 twist_mux 接入安全链）。
- `scripts/install_dependencies.sh` 增加 `ros-humble-twist-mux`、
  `python3-open3d`。

### 构建与验证

- 构建：`colcon build --packages-select ndt_omp_ros2 ndt_relocalization
  lidar_localization_ros2 go2_navigation go2_unitree_sim_bridge`。
- `lidar_localization_ros2` 上游 `g2_ndt_score` 的 `${ndt_omp_ros2_INCLUDE_DIRS}`
  变量经 ament 导出不含 ndt_omp 自身 include，本地改为直接链接
  `ndt_omp_ros2::ndt_omp`（记录在 CMakeLists 注释中）。
- 独立 `ROS_DOMAIN_ID=141` + CycloneDDS（lo 接口）无界面 Gazebo 验证：
  - D435 点云有数据（frame `d435_depth_optical_frame`，约 5 Hz，受 Gazebo
    负载限制）；
  - 定位器加载 `GlobalMap.pcd`、接收 `/initialpose`、NDT 配准（fitness 约
    0.59 < 6.0）、发布 `/pcl_pose` 与唯一 `map -> odom`；
  - 完整导航栈 10 个节点全部 `active`，控制链话题
    `/cmd_vel_nav -> /cmd_vel_switched -> /cmd_vel_smoothed -> /cmd_vel`
    全部存在，`/cmd_vel` 最终订阅者为 CHAMP `quadruped_controller_node`；
  - 端到端：Nav2 目标被接受、`/plan` 生成路径、机器人沿路径移动（一次测试
    35 s 移动约 1.4 m）。
- 建图：自动驱动机器人走矩形轨迹（原地转圈 + 前后左右移动）供 LIO-SAM 建图，
  保存后生成裁剪到导航区域（约 8×8 m）的同源地图包。
- GPU 验证：`bash scripts/verify_gpu_runtime.sh` 通过——CUDA NDT 在 RTX 4060
  （compute 8.9）启用、3 级多分辨率、NDT 进程约 98 MiB、采样峰值 GPU 31%、
  发布 `/ndt_pose`。
- 经验：手动 `colcon build --packages-select ndt_relocalization` 时若未设置
  CUDA 环境变量，会构建成 CPU 回退版（不链接 libcudart），
  `verify_gpu_runtime.sh` 会失败；GPU 后端必须按
  `scripts/build_workspaces.sh` 的方式带 `-DUSE_FAST_GICP_CUDA=ON` 等参数构建。

### 运行边界与下一步

- **2026-08-09 稳健闭环修复**：普通仿真入口统一固定为 Domain 0，
  `GO2_UNITREE_SIM_DOMAIN_ID` 是唯一允许的显式隔离覆盖，避免遗留 Domain 141
  使 action client 与 Nav2 落在不同 DDS 图。新增
  `simulation_navigation.launch.xml`，一次启动 Gazebo、Unitree bridge（固定
  `/cmd_vel_unitree`）、定位、Nav2、安全链和 RViz。
- 地图包清单升级为 `schema_version: 1`：包含 PCD、`map.yaml`、`map.pgm`、
  `map_stats.json` 的哈希、路径、角色和生成参数；校验器拒绝旧清单、目录逃逸、
  哈希错误和离线膨胀地图。重建会将旧派生产物备份到 `map_backup_<时间戳>`。运行
  验证发现 NDT 的真实初始位置约为 `(-0.1,0.8)`，旧 `[0,8) × [-4,4)` 裁剪恰好
  把机器人和雷达放在边界外；现已保持原始 PCD 不变，在
  `[-2,8) × [-4,4)` 使用 `offline_inflate_radius_m: 0.0` 原子重建，旧派生产物位于
  `~/go2_maps/latest/map_backup_20260809_202436`。
- 新增 `goal_guard`：对外保持 `/navigate_to_pose` 与 RViz `/goal_pose`，内部
  使用 `/navigate_to_pose_raw`。它在触发 SmacPlanner 前检查地图坐标、有限值、
  边界、已知自由栅格、联调默认 0.10 m 余量、定位新鲜度/诊断、起点与底层 action；
  这是对
  Humble 越界目标可能使 planner_server 退出的防护。Smac `max_iterations` 改为
  `100000`，MPPI 参数改为 Humble 实际声明的 critic 名称。
- 因 Humble `nav2_bt_navigator` 将 `navigate_to_pose` action 名硬编码且不对 action
  endpoint remap 生效，新增 `go2_nav2_bt_navigator`（Apache-2.0）。它只派生
  Humble 的 lifecycle 装配与 NavigateToPose 名称，复用系统 Nav2 的规划、控制和
  行为实现；源码头保留来源与许可证说明。
- `behavior_server` 的恢复速度也 remap 到 `/cmd_vel_nav`，不再绕开
  `twist_mux -> velocity_smoother -> collision_monitor`；安全监督在定位失效、
  重定位、行为执行或关键导航节点掉线时持续发布 `/pause_navigation`。健康检查改用
  单调墙钟，覆盖域、10 个 lifecycle 状态、TF、定位、控制器、action 和速度链。
- 采用“闭环优先、约束渐进”的阶段一联调参数：目标门禁最小余量由 0.55 m 调整为
  可配置的 0.10 m，全局 costmap 膨胀从 0.55 m 调整为 0.30 m；越界、占用栅格、
  非有限值、定位失效、碰撞监控和最终速度出口唯一性仍保留。闭环稳定后再依据实测
  逐步收紧，不一次叠加多层高阈值。
- 扩图后 Domain 0 冷启动实测：NDT 收敛至约 `(-0.1,0.9)`，不再出现
  `Sensor origin ... is out of map bounds`，10 个 lifecycle 节点 active，
  `health_check` PASS。`(-0.1,0.9) -> (-0.1,1.5)` action 返回 `SUCCEEDED`，
  Gazebo 真值移动约 `0.517 m`，最终 NDT 位姿为 `(-0.090,1.546)`；返回
  `(-0.1,0.9)` 同样成功。`/cmd_vel` 唯一发布者仍为 `collision_monitor`。
- 导航模式键盘遥控必须把 `cmd_vel` remap 到 `/cmd_vel_teleop`。按此入口连续按 `i`
  实测 Gazebo 真值前进约 `0.188 m`；直接运行未 remap 的键盘节点会争用最终
  `/cmd_vel`，不属于有效的导航安全链测试。
- 统一入口的 `rviz:=true` 曾因 XML 嵌套 launch 的布尔传值与 Python 的严格字符串
  比较而没有启动 RViz。现改为大小写无关的布尔解析，并把 Gazebo 子启动文件置于
  scoped group，避免其内部 `rviz:=false` 覆盖导航开关；重建后冷启动日志已出现
  `rviz2: process started`，且 RViz 正常接收 Velodyne 消息。
- RViz 导航配置修正为 `Fixed Frame: map`，避免 `2D Pose Estimate` 与
  `2D Goal Pose` 携带 `odom` 而被定位器或 `goal_guard` 拒绝。`Static Map` 现明确
  订阅 `/map`，`Local Costmap`、`Global Costmap` 分离且默认关闭，浅蓝色
  `Localization Map` 保留 `/global_map`；`NDT Pose` 修复为订阅 `/pcl_pose`，默认
  采用俯视视角。地图服务在 Nav2 生命周期序列中提前激活，使初始位姿前即可显示
  静态地图。健康检查新增 RViz 配置、`/map`/`/global_map`、重复关键节点、重复
  action server 和最终 `/cmd_vel` 发布端点数量检查；当前地图仍沿用
  `~/go2_maps/latest`，不纳入自动探索或重新建图。
- 冷启动图审计发现默认 NDT 定位器的 PCD 可视化实际发布名为 `/initial_map`，而非
  配置期望的 `/global_map`。定位启动入口现将其显式 remap 到 `/global_map`；这不会
  改动 Nav2 的二维 `/map`，可避免 RViz 的 `Localization Map` 空白和健康检查误报。
- 短距离目标执行日志确认另一处 Humble 控制阻断：默认 BT 的 `FollowPath` 未携带
  `goal_checker_id`，在同时配置 `general_goal_checker` 与 `precise_goal_checker` 时，
  `controller_server` 因空 ID 直接 abort 并进入 recovery，MPPI 尚未产生控制命令。
  新增从官方默认树派生的 `go2_navigate_to_pose.xml`，只显式指定
  `general_goal_checker`，并由 `navigation.launch.py` 传给 BT Navigator。
- 旧的 `(4,0)` 人工种子短测只证明控制器与行为树能够执行；长时间观察后 NDT 会
  收敛到真实地图区域约 `(0,0.8)`，因此不得再把 `(4,0)` 当成当前仿真的初始位姿。
- `/odom/ground_truth` 的父坐标系是 `world`，而导航目标与 NDT 位姿属于 `map`；当前
  未发布 `map -> world`，不能直接将两组绝对坐标相减作为真值到达误差。后续完整验收
  应补充显式坐标转换或以 Gazebo 真值在 `map` 中的对应观测记录误差。
- CycloneDDS + lo 接口（无多播）导致 CLI 发现慢，测试脚本需较长等待；这是
  项目为 Unitree 仿真隔离的标准配置。
- 阶段一尚未完成：完整 12 次静态导航验收、移动障碍测试与实机等价验证。
- `ndt_cuda` 实验档保留但未在本阶段端到端验证。
- 后续阶段二起：IMU 职责修正（删除「IMU 提供线速度」）、LIO-SAM 回环、
  算法对比接口、坡地扩展。

## 2026-08-09 Unitree SDK2/ROS 2 兼容桥 v1

### 阶段目标与调研

- 保留 Gazebo Classic、CHAMP、LIO-SAM、NDT 和传感器链，在唯一 `simdog`
  工作空间增加 Unitree 接口级兼容层，不实现 `/lowcmd`。
- 固定复用 Unitree 官方 `unitree_ros2 v0.3.0` 中 BSD-3-Clause 的
  `unitree_go`、`unitree_api`，来源提交为
  `66ae09858245ac3d2231c0cc209e36a88f8d7d03`。消息定义保持官方版本；仅在
  `package.xml` 补齐 CMake 已使用的 `rosidl_generator_dds_idl` 构建依赖并规范
  占位元数据；消息字段、类型和顺序不变，仅整理尾部空白。
- 参考官方 `unitree_mujoco` 的 CycloneDDS Domain 1/loopback 仿真和
  Domain 0/真机网卡切换方式，但没有引入 MuJoCo。来源：
  <https://github.com/unitreerobotics/unitree_ros2/tree/v0.3.0>、
  <https://github.com/unitreerobotics/unitree_mujoco>。

### 实际操作

- 新增 `go2_unitree_sim_bridge`，发布 `/sportmodestate`、
  `/lf/sportmodestate`、`/lowstate`、`/lf/lowstate` 和
  `/api/sport/response`；订阅 `/api/sport/request`。
- 支持 API 1002、1003、1004、1005、1006、1007、1008、1009、1010、
  1016、1017、1022。Move 按 CHAMP 上限持续发布并由 StopMove 清零；Euler
  拒绝非有限值和超限姿态；站立、坐卧、恢复与表演动作调用现有行为服务。
- `go2_behaviors` 增加串行服务端、`stop` 取消入口和状态话题。主 Gazebo 启动
  文件默认以 `unitree_bridge:=true` 启动服务端与桥接，可显式关闭。
- Sport 状态使用真值里程计、IMU 和足端 TF；LowState 前 12 个电机与四足接触
  统一为 `FR、FL、RR、RL`。发布定时器使用稳态时钟，消息时间戳使用真值里程计
  的仿真时间；输入超时会停止 Move、设置错误状态并节流告警。
- 依赖脚本增加 CycloneDDS RMW 和 DDS IDL 生成器；增加仿真
  `setup_unitree_sim.bash` 与真机 `setup_unitree_real.bash` 环境入口。

### 构建与验证

会话初因 `sudo` 交互密码不可用，曾临时从 ROS 软件源解包
`rosidl_generator_dds_idl`、CycloneDDS RMW 及其运行依赖完成构建和 loopback
验证；随后已于 2026-08-09 通过 `bash scripts/install_dependencies.sh` 正式
安装到系统，不再依赖 `/tmp` 临时环境：

```text
ros-humble-rmw-cyclonedds-cpp 1.3.4
ros-humble-cyclonedds 0.10.5
ros-humble-rosidl-generator-dds-idl 0.8.1
```

已完成的验证：

```bash
test "$(cd simdog && colcon list | wc -l)" -eq 21
colcon build --symlink-install --packages-select \
    unitree_api unitree_go go2_behaviors go2_unitree_sim_bridge go2_config
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
    simdog/src/go2_unitree_sim_bridge/test  # 本机 anyio 插件加载 _pytest.scope 失败，必须禁用自动加载
python3 -m py_compile <新增及修改的 Python 文件>
bash -n scripts/setup_unitree_sim.bash scripts/setup_unitree_real.bash simdog/start.sh
```

在独立 `ROS_DOMAIN_ID=173`、无界面 Gazebo 中确认 `/clock`、里程计、IMU、
关节、Velodyne 和控制器正常；四个 Unitree 状态话题实测约为
`49.8/10.0/100.9/10.0 Hz`，时间戳、IMU、电机、足端 TF 和接触均有有效数据。
Move 限幅、StopMove、Euler、非法参数、不支持 API、`noreply`、忙、取消及原动作
失败响应均符合预期；StandUp、StandDown、RecoveryStand、Sit、RiseSit、Hello、
Stretch、Dance1 均通过兼容桥执行成功。
另外在临时解包的 `rmw_cyclonedds_cpp` 环境中，以 `lo`、Domain 176 完成独立
进程间 `/api/sport/request`/`response` 请求响应，确认 CycloneDDS 环境入口有效。
Domain 177 下 LIO-SAM 的四个核心节点也均能使用 CycloneDDS 启动并等待数据；
12 秒冒烟结束时由 `timeout` 发送 SIGINT。该项没有输入传感器数据或正式地图。
完整 Gazebo 初次切换 CycloneDDS 时暴露了默认自动 participant 索引不足的问题，
两个环境脚本现均设置 `MaxAutoParticipantIndex=100`。修复后在 Domain 179、独立
Gazebo master 下，两个控制器、行为服务端和桥接全部启动，四个状态话题实测
`50.2/10.0/99.9/10.0 Hz`，且时间戳、IMU、电机和足端 TF 均为有效非零数据；
桥接与服务端也能随 launch 干净退出。

### 运行边界与下一步

- `range_obstacle`、BMS、温度、序列号、CRC 和没有可信来源的力矩保持零；足底
  接触只是 `0/1`，不是真实力。输入首次就绪前不发布状态。
- 不模拟 `/lowcmd`、无线遥控、真机内部平衡、安全固件或高风险翻转动作。
- Unitree Move 活动期间禁止键盘遥控；动作轨迹不可下发真机。
- 本阶段没有真机硬件和正式 PCD 地图验证。后续真机测试必须先核对网卡、Domain、
  安全场地和急停措施，仿真桥不得与真机 DDS 图同时启动。

## 2026-08-07 Go2 仿真动作

### 阶段目标

- 在没有真机的情况下实现打招呼、点头、伸展、趴下、挥爪和简单舞蹈。
- 复用成熟控制接口，避免重新实现关节控制器和自定义 ROS 消息。
- 防止 CHAMP 步态与动作轨迹同时写入同一个控制器。

### 调研与选择

- 对照 Unitree SDK2 Go2 `SportClient`，确认官方提供 `Hello`、`Stretch` 等
  真机 RPC 接口，但实际运动策略位于真机固件中，不能直接复用于 Gazebo。
- 对照 CHAMP 上游保留现有四足站姿、关节顺序和步态控制，不复制其控制算法。
- 采用 ROS 2 `joint_trajectory_controller` 已有的标准
  `control_msgs/action/FollowJointTrajectory` 接口，没有新建自定义消息或
  重复实现轨迹控制器。
- 参考来源：
  - CHAMP：<https://github.com/chvmp/champ>
  - Unitree SDK2：
    <https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/sport/sport_client.hpp>
  - ROS 2 控制器文档：
    <https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>
- CHAMP 保持 BSD-3-Clause，ROS 2 控制组件为 Apache-2.0；新增
  `go2_behaviors` 使用 BSD-3-Clause，没有复制或引入第三方源码。

### 实际操作

- 新增 `simdog/src/go2_behaviors` ament Python 包和统一命令：

  ```bash
  ros2 run go2_behaviors go2_behavior \
      {hello,nod,stretch,lie,wave,dance,stand}
  ```

- `champ_base` 新增
  `/quadruped_controller_node/set_behavior_mode` 标准 `SetBool` 服务。进入
  行为模式后停止 CHAMP 关节轨迹并忽略速度、机身姿态输入；普通动作结束后自动
  恢复 CHAMP。
- `lie` 完成后保持动作控制权和趴下姿态，`stand` 平滑恢复站姿后再恢复 CHAMP。
- 动作节点启动时读取实际 `/joint_states` 作为轨迹起点，内置关键帧完整性、有限值
  和 URDF 关节限位检查，并用进程锁拒绝并行动作；动作结束后读取
  `/odom/ground_truth` 检查机身高度、横滚和俯仰，防止把“关节目标成功”误判为
  “机器人动作成功”。
- 为 Gazebo 关节速度抖动设置明确的动作目标容差和结束时间容差，避免标准动作服务
  因默认停止速度阈值长期不返回。
- 更新依赖脚本、根目录 README、`simdog/README.md` 和动作包 README；
  `AGENTS.md`、`CLAUDE.md` 同步加入新包与真机边界。
- 后续文档同步将 `文档/simdog_packages_guide.md` 从过期的 16 包结构更新为
  实测的 18 包结构，补充动作控制链、命令表、调参和故障排查，并校正 NDT 输出、
  D435 启用状态、Git 状态以及失效的相对链接。

### 构建与闭环验证

构建和静态检查通过：

```bash
python3 -m py_compile \
    simdog/src/go2_behaviors/go2_behaviors/behavior_runner.py
colcon build --symlink-install --packages-select champ_base go2_behaviors
cmp -s AGENTS.md CLAUDE.md
test "$(cd simdog && colcon list | wc -l)" -eq 18
git diff --check
```

最终使用独立 `ROS_DOMAIN_ID=109` 和
`GAZEBO_MASTER_URI=http://127.0.0.1:11409` 启动无界面完整四足 Gazebo，
两个 `ros2_control` 控制器均为 `active`。六个动作依次返回成功，且动作结束
后的 `/odom/ground_truth` 结果为：

```text
hello：  z=0.214 m，机身水平
nod：    z=0.215 m，机身水平
stretch：z=0.215 m，机身水平
wave：   z=0.215 m，机身水平
dance：  z=0.216 m，机身水平，存在预期的偏航变化
lie：    z=0.094 m，保持趴下
stand：  z=0.216 m，恢复站立
```

`lie` 保持期间监听
`/joint_group_effort_controller/joint_trajectory` 两秒没有收到 CHAMP 消息，
确认控制权仲裁有效。并行启动第二个动作会以退出码 `2` 拒绝，首个动作继续正常
完成。

初版 `wave` 抬腿幅度过大，实测造成侧翻；最终版本缩短右前腿并降低横摆幅度后，
在两次独立仿真中均保持约 `0.215 m` 站立高度。不能只凭动作服务返回成功判断
动力学动作有效，因此运行入口现在会自动执行上述动力学姿态检查。最终还在独立
`ROS_DOMAIN_ID=110` 中复测 `hello -> lie -> stand`，运行时检查分别得到
`z=0.188/0.092/0.191 m`，横滚和俯仰均在 `0.016 rad` 以内。

### 运行边界

- 这些轨迹只针对当前 Go2 Gazebo 模型和控制器参数，不可直接下发真机。
- 动作必须串行执行；执行期间不要遥控。异常中断或保持趴下后先执行 `stand`。
- 仿真动作是确定性关键帧，不具备真机运动策略的在线平衡、落脚规划和安全保护。

## 2026-08-06 TF 所有权统一

### 阶段目标

- 修复 `map`、`odom` 与机器人模型分成两棵 TF 树的问题。
- 保证 CHAMP/EKF、LIO-SAM、NDT 和 `robot_state_publisher` 对每条 TF
  只有一个所有者。
- 删除 LIO-SAM 硬编码的孤立 `lidar_link`，统一使用 URDF 中的 `velodyne`
  外参。

### 方案调研与选择

- 参考 `robot_localization` 官方状态估计约定：局部估计器在
  `world_frame=odom` 时发布 `odom -> base_link`；全局定位器应发布
  `map -> odom`，并依赖已有的 `odom -> base_link`，避免一个子坐标系拥有
  多个父坐标系。
- 对照 CHAMP 上游已有的双 EKF 结构，恢复项目内被注释的
  `footprint_to_odom_ekf`，没有重新实现一套里程计节点。
- 对照 LIO-SAM 上游数据流保留其 IMU 预积分与建图里程计，只修改本项目的 TF
  边界和实际 Go2 外参适配。
- 参考来源：
  - `robot_localization` 官方文档：
    <https://docs.ros.org/en/kinetic/api/robot_localization/html/state_estimation_nodes.html>
  - CHAMP：<https://github.com/chvmp/champ>
  - LIO-SAM：<https://github.com/TixiaoShan/LIO-SAM>
- 本地 CHAMP 与 LIO-SAM 均保留原有 BSD 许可文件；LIO-SAM 的
  `package.xml` 许可证字段由错误的 `TODO` 校正为 `BSD-3-Clause`。本次没有
  引入新依赖，ROS 2 Humble 与 Gazebo Classic 11 适配风险较低。

### 实际操作

- 恢复 `champ_bringup` 中的 `footprint_to_odom_ekf`，由其唯一动态发布
  `odom -> base_footprint` 和 `/odom`；`publish_odom_tf` 参数现在实际生效。
- 保留 `base_to_footprint_ekf` 发布
  `base_footprint -> base_link`，`robot_state_publisher` 继续负责
  `base_link` 以下的关节和传感器。
- LIO-SAM 内部融合里程计从 `/odom` 隔离到
  `/lio_sam/imu/odometry`，映射结果使用 `map` 坐标系，并默认关闭其
  `odometryFrame -> base_footprint` TF。
- 删除 LIO-SAM 启动文件中的静态 `map -> odom`，改为通过
  `map -> velodyne` 与现有 `odom -> velodyne` 反算并动态发布
  `map -> odom`。
- 删除运行路径中的硬编码 `lidar_link`；LIO-SAM 参考模型改为复用
  `go2_description/xacro/robot_VLP.xacro`，RViz 配置同步移除孤立节点。
- `lidar.launch.py` 增加 `publish_map_to_odom`：
  - 建图模式为 `true`，LIO-SAM 拥有 `map -> odom`；
  - NDT 模式为 `false`，LIO-SAM 不注册 `/tf` 发布端，由 NDT 唯一拥有
    `map -> odom`。
- NDT 将配准结果按实际点云帧解释为 `map -> velodyne`，通过 URDF 外参换算
  `map -> base_link`；`/initialpose` 的 `map -> base_link` 初值也会反向
  换算为 NDT 所需的雷达初值。
- `simdog/start.sh` 根据 `GlobalMap.pcd` 是否存在自动选择上述 TF 所有者。
- LIO-SAM 退出自动保存改为关闭，地图只通过 `simdog/save_Map.sh` 显式保存，
  避免普通节点退出覆盖正式地图。
- 根目录 `README.md`、`simdog/README.md`、`AGENTS.md` 与 `CLAUDE.md` 已同步
  记录建图/重定位两种 TF 所有权模式；规则镜像通过 `cmp -s` 检查。
- 补充根目录 `README.md` 与 `simdog/README.md` 的完整启动指南：按首次建图和
  已有地图重定位分开列出每个终端的环境加载、启动、保存、停止和检查命令，并明确
  LIO-SAM 与 NDT 的 `map -> odom` 所有权切换及 RViz2 的 Fixed Frame 处理方式。

### 验证结果

受影响 C++ 目标和包已成功编译：

```bash
colcon build --symlink-install --packages-select \
    champ_bringup champ_base lio_sam ndt_relocalization go2_description
cmake --build simdog/build/lio_sam \
    --target lio_sam_imuPreintegration lio_sam_mapOptimization -- -j2
cmake --build simdog/build/ndt_relocalization \
    --target ndt_relocalization_node -- -j2
```

使用独立 `ROS_DOMAIN_ID=86` 和独立 Gazebo master 启动无界面仿真后确认：

```text
/odom 发布者：footprint_to_odom_ekf（1 个）
LIO 内部里程计：/lio_sam/imu/odometry（1 个）
TF 动态主链：map -> odom -> base_footprint -> base_link
雷达外参：base_link -> velodyne，平移 [0.2, 0.0, 0.118]
动态 TF 中不存在 lidar_link
```

`publish_map_to_odom:=true` 时，`lio_sam_mapOptimization` 是
`map -> odom` 发布者，完整 `map -> base_link` 可连续查询；
`publish_map_to_odom:=false` 时参数实测为 `False`，LIO-SAM 不注册 `/tf`
发布端，动态 TF 中不再出现 `map -> odom`，为 NDT 留出唯一所有权。

执行 `bash scripts/verify_gpu_runtime.sh` 通过：隔离域为 `132`，RTX 4060
启用三级 CUDA NDT，NDT 计算进程使用约 `98 MiB`，采样峰值 GPU `7%`、总显存
`192 MiB`，并成功发布有限值 `/ndt_pose`。

随后使用独立 `ROS_DOMAIN_ID=93` 和
`GAZEBO_MASTER_URI=http://127.0.0.1:11393` 完成真实数据闭环验证，未接入或
终止用户原有 ROS 图：

1. 启动完整四足 Gazebo 后，两个 `ros2_control` 控制器均为 `active`，
   `/odom` 只有 `footprint_to_odom_ekf` 一个发布者。
2. 以 `publish_map_to_odom:=true` 启动 LIO-SAM，成功发布
   `/lio_sam/mapping/odometry` 和动态 `map -> odom`。
3. 调用 `/lio_sam/save_map` 成功生成临时 `GlobalMap.pcd`，文件为
   `43210` 字节、包含 `2689` 个点。
4. 重启 LIO-SAM 并设为 `publish_map_to_odom:=false` 后，参数实测为
   `False`，其节点不再注册 `/tf` 发布端。
5. NDT 使用刚生成的地图和 `registration_backend:=cuda` 直接完成重定位，
   过滤后地图为 `2424` 点，配准分数约 `0.005–0.008`，`/ndt_pose`
   发布频率约 `8 Hz`。
6. TF 审计共收集到 `34` 条关系，完整主链为
   `map -> odom -> base_footprint -> base_link -> velodyne`；
   `base_link -> velodyne` 平移为 `[0.2, 0.0, 0.118]`，不存在
   `lidar_link`。NDT 是全局 TF 发布者，其 CUDA 进程使用约 `98 MiB`
   计算显存。

验证完成后已停止隔离域中的 NDT、LIO-SAM 和 Gazebo，并删除仅用于测试的临时
地图目录。

### 运行边界

- `map -> odom` 只能由 LIO-SAM 或 NDT 二选一发布；手工同时启动时必须给
  LIO-SAM 传入 `publish_map_to_odom:=false`。
- 第一次隔离测试在关闭 LIO-SAM 时触发了原配置的自动保存，在
  `~/go2_maps/latest` 生成了 `trajectory.pcd`、`transformations.pcd`、
  `cloudCorner.pcd`、`cloudSurf.pcd` 和 `cloudGlobal.pcd`；没有生成或覆盖
  NDT 使用的 `GlobalMap.pcd`。随后已关闭自动保存。
- Gazebo 退出时仍可能出现既有的 `contact_sensor` Boost mutex 断言，不影响
  运行期间 TF 验证。
- Gazebo 加载四组关节硬件时会报告既有的 `hold_joints` 参数重复声明，但本次
  闭环测试中两个控制器仍正常进入 `active`，未阻断步态、传感器或 TF。

## 2026-08-06 文档中文化与包结构梳理

### 阶段目标

- 将 `simdog/src/` 下所有项目级英文 README 翻译为规范中文。
- 创建 `simdog/src/` 包结构汇总中文参考文档。
- 回答 CHAMP 核心包在代码中的分布位置。

### 实际操作

- 翻译以下 10 个项目级 README 为中文：
  - `unitree-go2-ros2/README.md` — Go2 主配置指南
  - `LIO-SAM/README.md` — 激光惯性 SLAM 完整文档
  - `champ/README.md` — CHAMP 四足控制器框架
  - `fast_gicp/README.md` — 快速点云配准库
  - `pointcloud_to_laserscan/README.md` — 点云/激光转换
  - `realsense_ros_gazebo/README.md` — RealSense 仿真
  - `ndt_omp_ros2/README.md` — OpenMP NDT 算法
  - `champ_teleop/README.md` — 遥控节点
  - `robots/README.md` — 机器人配置库
  - `champ/include/champ/README.md` — CHAMP 核心库引用
- 第三方库文档（Eigen、Sophus、nvbio 等 39 个）保持原文不变。
- 创建 [`文档/simdog_packages_guide.md`](文档/simdog_packages_guide.md)，详细记录：
  - 16 个 ROS 2 包的完整路径、功能说明和数据流关系
  - CHAMP 12 个核心包在代码中的具体分布位置
  - Velodyne VLP-16 传感器规格与项目中的代码体现
  - 步态参数说明和常用启动命令速查
- 所有译文保持与原文档相同的 Markdown 结构、代码块和图片链接。

### 验证

```bash
# 确认所有翻译文件存在且非空
for f in \
  simdog/src/unitree-go2-ros2/README.md \
  simdog/src/LIO-SAM/README.md \
  simdog/src/pointcloud_to_laserscan/README.md \
  simdog/src/realsense_ros_gazebo/README.md \
  simdog/src/ndt_omp_ros2/README.md \
  simdog/src/fast_gicp/README.md \
  simdog/src/unitree-go2-ros2/champ/README.md \
  simdog/src/unitree-go2-ros2/champ_teleop/README.md \
  simdog/src/unitree-go2-ros2/robots/README.md \
  simdog/src/unitree-go2-ros2/champ/champ/include/champ/README.md; do
  wc -l "$f"
done
```

本阶段仅修改文档，未改动任何算法源码、启动文件或构建配置。

## 2026-08-06 工作空间收敛与文档校正

### 阶段目标

- 删除没有腿部动力学的简化机器人，只保留完整四足仿真。
- 让 `AGENTS.md` 和 `CLAUDE.md` 内容完全一致并要求同步维护。
- 核对真实 GPU，清理错误硬件信息和失效路径。

### 实际操作

- 删除 `go2_ws/` 的源码、构建产物、安装产物和日志。
- 删除 `scripts/setup_go2_ws.bash`。
- 将 `scripts/build_workspaces.sh` 改为只构建 `simdog`，保留 CUDA 12.8
  检测、`sm_89` 构建和 OpenMP 回退。
- 简化 `scripts/setup_simdog.bash`，移除已删除工作空间的叠加检查。
- 为 `scripts/verify_gpu_runtime.sh` 增加独立 `ROS_DOMAIN_ID`，避免用户正在运行
  的 Gazebo `/clock` 和 TF 与验证节点的系统时间互相干扰。
- 将 `AGENTS.md` 与 `CLAUDE.md` 改为完全相同的中文规则，加入同步维护与
  `cmp -s AGENTS.md CLAUDE.md` 检查要求。
- 更新根目录 README、GPU 文档、`simdog/README.md` 和 VS Code 配置；移除
  旧用户 `/home/luhao/...` 与已删除 `go2_ws` 的路径。

### 硬件核对

2026-08-06 使用 `nvidia-smi` 实际读取：

```text
NVIDIA GeForce RTX 4060 Laptop GPU, 595.84, 8188 MiB, compute 8.9
```

因此本机不是 RTX 5070。当前 CUDA 工具链为 12.8，`fast_gicp` 构建架构为
`sm_89`。

### 本阶段验证

```bash
nvidia-smi --query-gpu=name,driver_version,memory.total,compute_cap \
    --format=csv,noheader
cmp -s AGENTS.md CLAUDE.md
bash -n scripts/build_workspaces.sh
bash -n scripts/setup_simdog.bash
bash -n simdog/start.sh
bash -n simdog/save_Map.sh
colcon list --base-paths simdog/src
bash scripts/verify_gpu_runtime.sh
```

本阶段只调整工作空间结构、脚本和文档，没有修改 CHAMP、Gazebo、LIO-SAM 或
NDT 算法源码。静态检查识别到 `simdog` 的 17 个主要 ROS 2 包。

首次 GPU 验证受到当时正在运行的 Gazebo 仿真时间和 TF 干扰，未收到
`/ndt_pose`；确认不是 GPU 或 CUDA 后端错误后，为验证脚本增加独立 ROS 域。
再次运行时使用 `ROS_DOMAIN_ID=179`，RTX 4060 成功启用三级 CUDA NDT，节点
使用约 `98 MiB` 计算显存，GPU 采样峰值 `46%`，并发布有限值 `/ndt_pose`，
端到端验证通过。用户原有 Gazebo 进程未被终止。

完整 Gazebo 四足场景运行能力沿用 2026-08-05 的验证基线；删除简化工作空间后
没有重新启动第二套 Gazebo 场景，因为当前用户已有完整四足 Gazebo 正在运行。

## 2026-08-05 环境配置与完整四足验证基线

### 依赖与构建

- 安装 Gazebo Classic 11、`gazebo_ros_pkgs`、`gazebo_ros2_control`、
  Velodyne、`robot_localization`、ROS 2 控制器、PCL、Nav2、SLAM Toolbox、
  `teleop_twist_keyboard`、`diagnostic_updater` 和 `ecl_threads` 等依赖。
- 使用 BorgLab 4.1 PPA 安装 `libgtsam-dev` 和
  `libgtsam-unstable-dev`。`rosdep` 可能报告 `ros-humble-gtsam` 未满足，
  原因是项目主动使用 GTSAM 4.1，不应与 ROS 仓库 GTSAM 4.2 混装。
- `simdog` 17 个主要包完成干净构建。
- 修复 `gui:=false` 仍启动 `gzclient` 的问题。
- 为 LIO-SAM 增加 `rviz` 参数，地图保存默认目录改为
  `~/go2_maps/latest`。
- NDT 地图路径、输入话题、配准后端和 GPU 设备均已参数化。

### GPU NDT

- CUDA 12.8 编译器、Runtime 和 cuBLAS 开发库来自 NVIDIA 官方 Ubuntu
  22.04 软件源，没有替换现有显卡驱动。
- `fast_gicp` 基于上游提交
  `0e7ec1441c99f7be453db2ea216d5de029387417`，保留 BSD 3-Clause
  `LICENSE`。
- 针对 CUDA 12.8 修复新版 Thrust 与旧式前置声明的命名空间冲突。
- GPU 库按 RTX 4060 计算能力 8.9 编译；`cuobjdump` 已确认包含 `sm_89`
  cubin，NDT 节点链接 `libcudart.so.12` 和 `libfast_vgicp_cuda.so`。
- `registration_backend` 支持 `cuda` 和 `omp`；CUDA 设备或 GPU 构建不可用
  时自动回退 OpenMP。

### 完整四足运行结果

- `gzserver`、CHAMP、`ros2_control` 和 EKF 正常运行，所需控制器 active。
- `/velodyne_points` 约 10 Hz、`/imu/data` 约 200 Hz、
  `/joint_states` 约 250 Hz，`/odom/local` 有输出。
- 向 `/cmd_vel` 发布约 1 秒的 `0.15 m/s` 前进速度后，机器人前移约
  `0.20 m`。
- LIO-SAM 去畸变点云、特征点云、配准点云和里程计均有输出；
  `/lio_sam/save_map` 成功生成 PCD 文件。
- NDT 成功读取验证地图、发布 `/global_map`，提供 `/initialpose` 后持续输出
  `/ndt_pose`。
- CUDA D2D-NDT 对约 1.7 万点测试点云单次约 `4.36 ms`，100 次约
  `417.72 ms`；GPU SM 峰值 `74%`、显存约 `212 MiB`。同一测试的 CPU
  多线程 VGICP 单次约 `17.52 ms`。
- `scripts/verify_gpu_runtime.sh` 端到端验证通过：NDT 出现在 NVIDIA 计算进程
  中并发布有限值 `/ndt_pose`，脚本退出后没有残留测试进程。

验证命令：

```bash
bash scripts/build_workspaces.sh
bash scripts/verify_gpu_runtime.sh

source scripts/setup_simdog.bash
ros2 launch go2_config gazebo_velodyne.launch.py gui:=false rviz:=false
ros2 topic hz /velodyne_points
ros2 topic hz /imu/data
ros2 topic echo --once /odom/local
ros2 control list_controllers
```

## 已知限制与下一步

- 当前只有 NDT 点云配准使用 CUDA。LIO-SAM 图优化、点云预处理、Gazebo 物理
  和 CHAMP 仍主要使用 CPU；Gazebo/RViz2 只使用 GPU 进行 OpenGL 渲染。
- Gazebo Classic GUI 可能受驱动与 OGRE 兼容性影响，推荐
  `gui:=false` 配合 RViz2。
- Gazebo 退出时 `contact_sensor` 可能触发 Boost mutex 断言；该现象发生在
  停止阶段，不影响运行数据。
- 部分 `ros2_control` 实例可能提示 `hold_joints` 重复声明，但所需控制器保持
  active。
- EKF 会提示当前 IMU 配置包含消息不提供的速度项；后续精细状态估计需要继续
  校准协方差和融合字段。
- LIO-SAM 当前关闭回环检测。
- Nav2 和 SLAM Toolbox 依赖已安装，但完整自主导航参数尚未完成调优。
- 正式使用 NDT 前，应在目标场景完成稳定建图并保存 `GlobalMap.pcd`，再校准
  NDT 分辨率和初始位姿。

## 维护规则

每完成一个可验证阶段、修改启动入口、关键参数或硬件基线后，更新本文件中的日期、
操作、结果、验证命令、限制和下一步；合并或替换过期信息，不创建版本副本。
