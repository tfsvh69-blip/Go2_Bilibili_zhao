# Go2 全局代价图幽灵障碍与近距碰撞调查

## 当前结论与安全状态

记录日期：2026-08-14。用户在导航画面中观察到：全局代价图出现不属于静态
墙体的青色/紫色/粉色障碍岛；机器人移动后，这些岛会跳到新位置，旧岛长时间保留后又
概率性消失。机器人接近标准方块时，方块的代价区会消失，随后发生真实接触。

这组现象使当前“障碍安全导航”总门禁判为 **FAIL**。阶段 0 运行时工具、阶段 1 盲区
量化和阶段 2 Footprint 几何仍各自保持 PASS，但阶段 3 Inflation 尚未开始；必须先修复
ObstacleLayer 的空射线 clearing，并排除 `map→odom` 跳变。三个 profile 继续为
`UNCALIBRATED`，当前配置不得称为安全或已解决近距碰撞。

本轮只完成配置/上游源码审计与文档交接，**没有修改** Inflation、Persistence、传感器
量程或 Collision Monitor 数值。原因是这四组量会互相影响，直接同时改大只能暂时遮住
幽灵格或漏检，无法证明机器人不会撞上真实障碍。

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

截图中的“新障碍岛出现、旧岛随后才偶尔消失”与下面的 clearing 链高度一致：

```text
有限回波端点 → 在当时的 map 坐标标记 lethal 格
该方向下一帧没有回波 → /scan 写 +inf
inf_is_valid=false → +inf 没被转换成可 raytrace 的远端点 → 旧格留在图中
机器人或 map→odom 改变 → 新有限端点落到另一组格 → 看起来像障碍跳动/复制
后来某条有限射线恰好穿过旧格 → 旧格才被 clearing → 看起来概率性消失
```

`inf_is_valid` 的官方含义就是“是否把激光的无限返回当作可用于 raycast 的有效量测”。
Nav2 Humble 源码在该项为 true 时，会把正无穷替换为 `range_max - 0.0001 m` 后再投影；
当前项目恰好使用 `use_inf: true` 生成 `/scan`，但两个 costmap source 没有显式设置
`inf_is_valid`，运行默认值为 false。依据见
[Nav2 Obstacle Layer 参数说明](https://docs.nav2.org/configuration/packages/costmap-plugins/obstacle.html)
和
[Nav2 1.1.20 ObstacleLayer 源码](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/plugins/obstacle_layer.cpp)。

这能解释“旧格为什么不及时清”，但还不能单独证明“最初的错误端点为什么出现”。长条、
墙形复制或整块平移仍需检查 `map→odom`：同一帧 `/scan` 在 `odom` 中稳定、在 `map`
中跳动，说明 AMCL/SLAM 的全局修正把动态障碍层写到了不同地图位置。该项目前是有截图
支持的高概率推断，尚无本次同步 TF 数值，不能写成已实测根因。

## 已确认的配置与几何矛盾

| 检查项 | 当前值/实测 | 物理结果 | 证据等级 |
|---|---:|---|---|
| `/scan` 无回波表达 | `use_inf=true` | 空方向输出 `+inf` | 静态配置 |
| local/global `inf_is_valid` | 未配置，默认 `false` | `+inf` 不进入远距离 clearing raycast | 官方源码 + 静态配置 |
| `observation_persistence` | `0.0 s` | 幽灵格不是“配置了很长记忆”造成 | 静态配置 |
| `marking/clearing` | 均为 `true` | clearing 开关打开，但空射线语义不完整 | 静态配置 |
| scan/obstacle/raytrace max | 都是 `15.0 m` | valid-inf 直接开启时，`14.9999 m` 端点可能又落入 marking 范围 | 官方源码 + 静态配置 |
| LiDAR 正前可靠下限 | 传感器到障碍表面 `0.90 m` | 0.80 m 及以内三组均 0% | 阶段 1 实测 |
| LiDAR 原点 | `base_footprint` 前约 `0.20 m` | 最近可靠表面约位于基座前 `1.10 m` | URDF + 探针回读 |
| Collision Monitor stop 前缘 | 基座前 `0.52 m` | 障碍进入 stop zone 前已经越过 LiDAR 可靠区 | 几何审计 |
| Collision Monitor decel 前缘 | 基座前 `0.72 m` | 同样位于 LiDAR 盲区内 | 几何审计 |
| approach 最远前向预测 | `0.389 + 0.20×1.5 ≈ 0.689 m` | 按当前最大平滑速度仍短于 `1.10 m` 可靠边界 | 参数计算 |
| D435 | p99 周期 `72.14 s`，曾迟到 2–6 s | 不能代替 LiDAR 为现有 stop zone 提供可靠触发 | 阶段 1 实测 |
| Inflation radius | `0.30 m` | 小于外扩 Footprint 外接半径 `0.474 m`，尚未标定安全中心线余量 | 阶段 2 几何 + 静态配置 |

由此可得一个不依赖截图解释的硬结论：正前方方块在仍被 LiDAR 可靠看见时，其表面最靠近
只能到基座前约 `1.10 m`；现有 decel/stop polygon 的前缘只有 `0.72/0.52 m`。障碍点
不可能既保持 LiDAR 可靠观测、又进入这两个 polygon。`approach` 在 `0.20 m/s`、
`1.5 s` 下的前向预测也只到约 `0.689 m`。因此当前 Collision Monitor 的 LiDAR
几何配置不能作为近距防撞闭环，靠近后代价区消失并继续碰撞是已有数据能够解释的结果。

## 先保证安全

在 clearing、TF 和 stop zone 完成重复验证前，不继续做自由导航碰撞试验。出现新幽灵格、
目标路径逼近障碍或方块进入近距盲区时，先执行：

```bash
ros2 service call /navigation/stop std_srvs/srv/Trigger "{}"
ros2 topic echo /pause_navigation --once
```

不要用以下方式“让画面看起来正常”：

- 不把 `observation_persistence` 直接调大；这会让真实删除的障碍更久不清。
- 不把 Gazebo `gpu_ray` 的 `range_min` 从 `0.90 m` 直接改小；阶段 1 已证明这是传感器
  模型边界，改参数后必须重新量化误差与频率。
- 不关闭 `clearing`、`footprint_clearing_enabled`、RPP collision detection 或
  Collision Monitor。
- 不只加大 Inflation 来掩盖错误端点；它会把每个幽灵格一起放大。

## 下一步执行顺序与 PASS 门

### 1. 先区分地图、定位与 ObstacleLayer

固定使用 `static_map + AMCL + ~/go2_maps/online/latest`，避免在线 SLAM 同时修改 `/map`。
在 RViz 一次只开启一个 Display：

1. 只开 `Static Map`：异常岛仍存在，说明保存的 PGM 已污染；不进入 costmap 调参。
2. 只开 `LaserScan`，Fixed Frame 先用 `odom` 再用 `map`：只在 `map` 中跳动，说明
   `map→odom` 不稳；先处理 AMCL/地图。
3. 只开 `Global Costmap`：Static Map 干净但异常岛出现，才归到 ObstacleLayer。

同时保存 60 s 的 `/scan`、`/tf`、`/tf_static`、`/amcl_pose`、local/global raw costmap，
统计 `map→odom` 单步变化、幽灵致命格出生/清除时间和传感器 p99。单步变化超过现有健康
门 `0.10 m` 或 `0.10 rad` 时，本轮归类为定位 FAIL。

### 2. 修复空射线 clearing，再测实际删除

不能孤立地把 `inf_is_valid` 改成 true：Nav2 1.1.20 会把 `+inf` 变成
`range_max - 0.0001 m`，而当前 scan、`obstacle_max_range`、`raytrace_max_range` 都是
`15.0 m`；上游 marking 逻辑会接受严格小于 obstacle max 的端点，因此这个
`14.9999 m` 点可能在图边界反而被标成障碍。正式实验应把 `inf_is_valid`、
`obstacle_max_range` 与 `raytrace_max_range` 作为一个“LaserScan 空射线语义重置组”：
marking 上限必须严格小于无回波替换端点，clearing 上限不能超过传感器声明范围；具体差值
由场景有效范围实验决定，不在本轮文档中猜值。

该 source 参数组决定订阅回调与 observation buffer，Nav2 1.1.20 不能把 active 状态下的
参数 read-back 当作内部已生效；必须通过 Lifecycle RESET/STARTUP 或完整重启重建
ObstacleLayer。先只改 local 组并与基线 A/B，再只改 global 组，不能同时混入 Inflation
或 Persistence。

用 Gazebo 实际删除标准方块，而不是仅把它移入盲区。PASS 条件保持原计划：清除时间不
超过

```text
observation_persistence + 两个 costmap 更新周期 + p99 传感器周期
```

同时要求静止 60 s 不再生成无真值来源的 lethal 岛。若仍出现，继续查 TF/扫描端点，
不得用 Persistence 掩盖。

### 3. 阶段 3 Inflation

只有 costmap 输入稳定后才开始。保持 Planner、RPP、速度和 Persistence 不变，从已标定
外扩 Footprint 外接半径 `0.474 m` 向上按 `0.10 m` 单变量步进 local/global；先选
`inflation_radius`，再单独调整 `cost_scaling_factor`。每次记录真实接触、路径中心线
clearance、导航时间和传感器/代价图延迟到 `logs/inflation_tuning.csv`。

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
| 1 LiDAR 盲区 | PASS（量化） | 正前 0.90 m、侧向 1.00 m、后向缝隙 | 作为停车区硬输入 |
| 2 Footprint | PASS（几何） | 24 点 polygon、padding 0.035 m | 保持 |
| 3 Inflation | **未开始/前置 FAIL** | 当前 0.30 m 仅为旧值 | 先完成 clearing + TF 基线 |
| 4 Persistence | 未开始 | 当前 0.0 s | clearing 正确后做 A/B |
| 5 Depth | 未开始 | D435 当前不可靠 | 完成负载与覆盖审计 |
| 6 Collision Monitor | **当前几何 FAIL** | 0.52/0.72 m 区域落在 LiDAR 盲区内 | 移动验收前重标定 |
| 7 重复验收 | 禁止开始 | 无 | 以前述硬门全部 PASS 为条件 |
