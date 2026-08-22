# Go2 全局代价图幽灵障碍与近距碰撞调查

## 当前结论与安全状态

最初记录日期为 2026-08-14；修复与复测日期为 2026-08-22。用户曾在导航画面中观察到
全局代价图出现不属于静态墙体的青色/紫色/粉色障碍岛；机器人移动后，新岛跳到新位置，
旧岛长时间保留后概率性消失。机器人接近标准方块时，方块代价区又会消失并发生接触。

`/scan` 空射线 clearing 现已形成可重复的最小闭环：新包 `go2_lidar_scan` 恢复
`use_inf=true`，local/global source 启用 `inf_is_valid=true`，并将 marking/raytrace
上限分为 `14.0/15.0 m`。最终 2 m 方块的 scan/Velodyne 各 3×30 帧检出率均 100%；
实际删除后精确方块区域 Local Costmap lethal 格由 64 降至 57（删除前背景为 55），
本项判为 **PASS**。独立 RViz 与完整在线 SLAM 中也未见 NaN 或越量程扫描值。

运动时“为什么会画出一部分错误端点”已实测：原地转向使 `velodyne` 随机身产生约
`4–6°` roll/pitch，旧实现却在这个倾斜坐标系中截取 `-0.05..0.10 m`，地面因此赢得
二维最近角度格。默认链路现先变换到 `velodyne_level`（平移/yaw 保留，roll/pitch 为
零）再切片。最终样本在 `5.15°` 倾斜时，旧投影为 4430 个地面获胜端点，新投影为 0；
240 帧点云都能查询精确时间戳 TF，`/scan=8.89 Hz`。这项“倾斜地面端点”子门在该批
数据中 PASS，不是根据截图推测；但后续人工在线 SLAM 仍复现白色扇形线和幽灵代价岛。
用户随后把高度窗改为 `+0.20..+0.30 m` 后，现场目视在线 SLAM 恢复正常。因此该窗口
成为当前正式基线；完整路线建图、保存和固定图导航仍待端到端验收，重力对齐仍不能单独
称为唯一根因或整体根治。

当前正式 `/scan` 的重力对齐高度窗为用户现场确认有效的
`+0.20..+0.30 m`，`min_height/max_height` 仍可通过 rqt 真正在线生效；调试
`/scan_raw` 固定保留旧窗口。该实验只改变高度投影，不改变 0.90 m 量程、分辨率、
SLAM 匹配或 costmap persistence，避免多变量混杂。

用户最新反馈是幽灵代价区域已基本不再出现，偶尔只在很远处出现；因此输入状态更新为
“主要现象基本解决、远处残余待取证”。“障碍安全导航”总门禁仍为 **FAIL**：完整地图/
导航输入验收尚未完成，近距盲区和 Collision Monitor 几何也尚未修复，Inflation 安全
标定、Depth/Persistence 与重复碰撞验收均未完成。三个 profile 继续为 `UNCALIBRATED`。
历史 135 秒运行仍有两次 costmap
`OutTheBack` 旧观测丢弃，已记录为时序残余；它不会生成错误端点。通过输入闭环后曾把
相关六处量程同时临时改为 0.50 m 做四方向实验，但后/右发生接触，故全部恢复 0.90 m。
没有用 Persistence/Inflation 掩盖清除问题。

## 肉眼现象对应哪条数据链

墙体和障碍周围的多圈颜色是 Static/Obstacle/Inflation Layer 合成后的 cost 值，不是
SmacPlanner2D 自己“识别”出来的物体。规划器只读取合成后的 Global Costmap。当前链路为：

```text
/map ─ Static Layer ──────────────────────────────┐
/scan ─ Obstacle Layer（marking + raytrace）──────┼─ Global Costmap ─ SmacPlanner2D
                       Inflation Layer ────────────┘
```

因此，颜色区域错乱首先应查 `/map`、`/scan`、`map→odom` 和 ObstacleLayer；不能先改
Planner、RPP 或目标容差。

截图中的“新障碍岛出现、旧岛随后才偶尔消失”与修复前的 clearing 链高度一致：

```text
有限回波端点 → 在当时的 map 坐标标记 lethal 格
该方向下一帧没有回波 → /scan 写 +inf
inf_is_valid=false → +inf 没被转换成可 raytrace 的远端点 → 旧格留在图中
机器人或 map→odom 改变 → 新有限端点落到另一组格 → 看起来像障碍跳动/复制
后来某条有限射线恰好穿过旧格 → 旧格才被 clearing → 看起来概率性消失
```

`inf_is_valid` 的官方含义就是“是否把激光的无限返回当作可用于 raycast 的有效量测”。
Nav2 Humble 源码在该项为 true 时，会把正无穷替换为 `range_max - 0.0001 m` 后再投影；
修复前曾用 `use_inf: true` 生成 `/scan`，但两个 costmap source 没有显式设置
`inf_is_valid`，运行默认值为 false。当前已把它们作为一组修复。依据见
[Nav2 Obstacle Layer 参数说明](https://docs.nav2.org/configuration/packages/costmap-plugins/obstacle.html)
和
[Nav2 1.1.20 ObstacleLayer 源码](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/plugins/obstacle_layer.cpp)。

clearing 能解释“旧格为什么不及时清”；重力对齐 A/B 则解释了“一部分错误端点为什么
出现”，现场新证据表明仍有未被旧探针分类到的错误端点。同时记录的 `map→odom` 初次最大平移步进为 `0.323 m`，按计划只对仿真真值
里程计做单变量 A/B：把 `/odom/ground_truth` 不应携带的 `gaussian_noise=0.01` 改为
`0.0` 后，最终闭环为 `0.0224 m/0.00698 rad`。两条因果链应分开理解：地面端点来自雷达
切片坐标错误，全局修正跳变来自“真值”话题被人为加噪；刷新率只会改变它们被擦除的
速度。

## 已确认的配置与几何矛盾

| 检查项 | 当前值/实测 | 物理结果 | 证据等级 |
|---|---:|---|---|
| `/scan` 无回波表达 | `use_inf=true` | 空方向输出 `+inf` | 运行参数 + `/scan` 实测 |
| local/global `inf_is_valid` | `true` | `+inf` 进入远距离 clearing raycast | 运行参数 + 官方源码 |
| `observation_persistence` | `0.0 s` | 幽灵格不是“配置了很长记忆”造成 | 静态配置 |
| `marking/clearing` | 均为 `true` | 有限端点标记，空射线擦除 | 静态配置 + 删除实测 |
| scan/obstacle/raytrace max | `15.0/14.0/15.0 m` | `14.9999 m` 空射线端点可 clearing、不可 marking | 官方源码 + 运行回读 |
| 运动期原始/对齐地面端点 | `4430/0`，最大倾斜 `5.15°` | 重力对齐阻断错误输入 | 240 帧同帧点云/TF A/B 实测 |
| 正式/原始高度窗 | `+0.20..+0.30 / -0.05..+0.10 m` | 正式窗现场目视正常，原始窗固定作参照 | 用户现场 A/B；端到端待验收 |
| `velodyne_level` | roll/pitch `0°`，同时间戳 TF 100% | `/scan` 不再随机身倾斜 | 运动探针实测 |
| `map→odom` 最大单步 | `0.0224 m/0.00698 rad` | 低于 `0.10 m/0.10 rad` | 在线 SLAM 实测 |
| `/scan` 墙钟频率 | `8.89 Hz`；独立 RViz 约 `9.15 Hz` | 高于 7 Hz 门 | 在线 SLAM/RViz 实测 |
| 2 m 方块删除 | 精确区域 lethal `64→57`，删除前背景 55 | 本场景未持续残留旧障碍格 | Local Costmap 实际删除实测 |
| 0.50 m 候选 | 后方 0% 检出且接触；右方也接触 | 不满足无接触门，恢复 0.90 m | 四方向、四距离各 3×10 帧 |
| LiDAR 正前可靠下限 | 传感器到障碍表面 `0.90 m` | 0.80 m 及以内三组均 0% | 阶段 1 实测 |
| LiDAR 原点 | `base_footprint` 前约 `0.20 m` | 最近可靠表面约位于基座前 `1.10 m` | URDF + 探针回读 |
| Collision Monitor stop 前缘 | 基座前 `0.52 m` | 障碍进入 stop zone 前已经越过 LiDAR 可靠区 | 几何审计 |
| Collision Monitor decel 前缘 | 基座前 `0.72 m` | 同样位于 LiDAR 盲区内 | 几何审计 |
| approach 最远前向预测 | `0.389 + 0.20×1.5 ≈ 0.689 m` | 按当前最大平滑速度仍短于 `1.10 m` 可靠边界 | 参数计算 |
| D435 | p99 周期 `72.14 s`，曾迟到 2–6 s | 不能代替 LiDAR 为现有 stop zone 提供可靠触发 | 阶段 1 实测 |
| Global/Local Inflation | `0.20/0.30 m`；scaling `0.5/3.0` | 全局现场值缩小远处代价岛；两张图近场安全余量均未系统标定 | 用户现场基线 + 静态配置 |

由此可得一个不依赖截图解释的硬结论：正前方方块在仍被 LiDAR 可靠看见时，其表面最靠近
只能到基座前约 `1.10 m`；现有 decel/stop polygon 的前缘只有 `0.72/0.52 m`。障碍点
不可能既保持 LiDAR 可靠观测、又进入这两个 polygon。`approach` 在 `0.20 m/s`、
`1.5 s` 下的前向预测也只到约 `0.689 m`。因此当前 Collision Monitor 的 LiDAR
几何配置不能作为近距防撞闭环，靠近后代价区消失并继续碰撞是已有数据能够解释的结果。

## 先保证安全

在 clearing 长时间重复、TF 和 stop zone 完成验证前，不继续做自由导航碰撞试验。出现新幽灵格、
目标路径逼近障碍或方块进入近距盲区时，先执行：

```bash
ros2 service call /navigation/stop std_srvs/srv/Trigger "{}"
ros2 topic echo /pause_navigation --once
```

不要用以下方式“让画面看起来正常”：

- 不把 `observation_persistence` 直接调大；这会让真实删除的障碍更久不清。
- 不把 Gazebo `gpu_ray` 的 `range_min` 从 `0.90 m` 直接改小；0.50 m 候选已经通过
  全链同步修改做过四方向实验，并因后/右接触被否决。
- 不关闭 `clearing`、`footprint_clearing_enabled`、RPP collision detection 或
  Collision Monitor。
- 不只加大 Inflation 来掩盖错误端点；它会把每个幽灵格一起放大。

## 下一步执行顺序与 PASS 门

### 1. 先区分地图、定位与 ObstacleLayer

固定使用 `static_map + AMCL + $GO2_PROJECT_ROOT/go2_maps/online/latest`，避免在线 SLAM 同时修改 `/map`。
在 RViz 一次只开启一个 Display：

1. 只开 `Static Map`：异常岛仍存在，说明保存的 PGM 已污染；不进入 costmap 调参。
2. 只开 `LaserScan`，Fixed Frame 先用 `odom` 再用 `map`：只在 `map` 中跳动，说明
   `map→odom` 不稳；先处理 AMCL/地图。
3. 只开 `Global Costmap`：Static Map 干净但异常岛出现，才归到 ObstacleLayer。

同时保存 60 s 的 `/scan`、`/tf`、`/tf_static`、`/amcl_pose`、local/global raw costmap，
统计 `map→odom` 单步变化、幽灵致命格出生/清除时间和传感器 p99。单步变化超过现有健康
门 `0.10 m` 或 `0.10 rad` 时，本轮归类为定位 FAIL。

### 2. 空射线 clearing 修复与实际删除结果

Nav2 1.1.20 会把 `+inf` 变成 `range_max - 0.0001 m`。因此不能孤立打开
`inf_is_valid`：若 marking 与 scan 上限同为 15 m，`14.9999 m` 端点可能又被标成
障碍。当前实现把三项作为一个“LaserScan 空射线语义组”：

```text
use_inf=true
inf_is_valid=true
obstacle_max_range=14.0 m < scan.range_max=15.0 m
raytrace_max_range=15.0 m
```

source 参数组通过完整重启重建 ObstacleLayer，没有把 active 状态下的参数 read-back
误当作内部已刷新，也没有同时修改 Inflation 或 Persistence。

2026-08-22 用 Gazebo 实际删除 2 m 标准方块，而不是把它移入盲区。首次探针 10/10 帧
检出，距离误差 p95 `0.0030 m`，无接触；Local Costmap lethal 格从背景约 `227`
升到 `237`，删除后约 `6.1 s` 回到 `226`，随后保持 `225–228`。数据保存在
`go2_lidar_scan/logs/clearing_smoke_20260822/`。它满足冒烟 PASS：清除时间不超过

```text
observation_persistence + 两个 costmap 更新周期 + p99 传感器周期
```

历史上又做了 135 秒正反整圈、四转弯闭合路线和前后移动压力采样；后续人工曾再次
看到新岛。用户把窗口提高到 `+0.20..+0.30 m` 后现场目视正常，当前应保持这一单变量，
从空白 SLAM 会话完成整场覆盖、保存并用固定图导航。若再次复现，再运行
`motion_scan_probe` 区分高度端点、TF 和 `map→odom`，不能用 Persistence 掩盖。

重力对齐后又执行一次相同类型的实际删除：方块存在时 Local Costmap lethal 格为 155，
删除后的 8 次采样为 `146–149`，回到背景带。证据在
`go2_lidar_scan/logs/clearing_after_level_fix_20260822/`。

最终 900 列生产档进一步取得 `clearing_final_900_20260822/`：2 m 方块两路各 3 组×30
帧均 100% 检出且无接触；`clearing_entity_exact_final_900_20260822/` 对精确方块区域
测得 lethal `64→57`，删除前背景 55。旧的冒烟目录保留用于展示验证递进，最终结论以
这两个目录为准。

### 3. 阶段 3 Inflation

用户已把 Global Costmap 设置为 `inflation_radius=0.20 m`、
`cost_scaling_factor=0.5`，Local Costmap 保持 `0.30 m/3.0`。这作为 V1.5.0 现场基线，
不追认成安全最优值：`0.20 m` 小于外扩 Footprint 外接半径 `0.474 m`，全局路径可能更
贴墙。后续保持 Planner、控制器、速度和 Persistence 不变，记录真实接触、路径中心线
clearance、导航时间以及远处残余端点到 `logs/inflation_tuning.csv`，再决定是否扩大半径。

### 4. 阶段 4 Persistence

依次测 `0.0/0.2/0.3/0.5/0.8/1.0 s`，严格区分：

- Case A：方块仍在 Gazebo，但进入近距漏检区；
- Case B：方块已经从 Gazebo 删除。

只选能覆盖 1～3 帧短漏检且满足清除上界的最小值。

### 5. Depth 与 Collision Monitor 是移动试验前的硬门

D435 先完成频率、p99、有效点、自体/地面误点和负载验证；达不到要求就从可靠 source
集合禁用，不能通过放宽 `source_timeout` 冒充可用。随后依据 LiDAR/D435 的可靠覆盖边界、
速度、p99 延迟和实测减速度重新标定 approach/decel/stop zones。stop 边界必须位于至少
一个可靠 source 仍持续看得见的位置。该门未 PASS 前，禁止九类自主移动碰撞验收。

## 当前阶段交接

| 阶段 | 状态 | 当前产物 | 下一动作 |
|---|---|---|---|
| 0 运行时框架 | PASS | `nav_tuner`、参数矩阵、记录/保存/回滚 | 保持 |
| 1 LiDAR 盲区 | PASS（量化） | 0.50 m 四方向复测否决，恢复 0.90 m | 作为停车区硬输入 |
| 2 Footprint | PASS（几何） | 24 点 polygon、padding 0.035 m | 保持 |
| 2.4 重力对齐输入 | 现场目视 PASS、完整验收待完成 | 原始 4430 个地面端点、对齐后 0；`0.20..0.30 m` 现场正常 | 保持该窗口，完成建图/保存/固定导航 |
| 2.5 空射线 clearing | PASS | 3×30 检出 100%，实际删除 `64→57` | 保持 `+inf` 参数组 |
| 2.6 频率/TF 压力 | PASS（有残余） | 8.89 Hz、TF 100%；2 次旧观测丢弃 | 继续监测，不增加 Persistence |
| 3 Inflation | 现场基线、未安全标定 | Global `0.20 m/0.5`；Local `0.30 m/3.0` | 测路径贴墙、clearance 与远处残余 |
| 4 Persistence | 未开始 | 当前 0.0 s | 只在 clearing 压力门后做 A/B |
| 5 Depth | 未开始 | D435 当前不可靠 | 完成负载与覆盖审计 |
| 6 Collision Monitor | **当前几何 FAIL** | 0.52/0.72 m 区域落在 LiDAR 盲区内 | 移动验收前重标定 |
| 7 重复验收 | 禁止开始 | 无 | 以前述硬门全部 PASS 为条件 |
