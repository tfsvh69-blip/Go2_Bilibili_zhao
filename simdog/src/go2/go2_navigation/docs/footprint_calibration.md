# Go2 Nav2 二维 Footprint 校准

## 结论与阶段门禁

实测日期：2026-08-14。阶段 2 的“URDF collision + 实际步态包络 + 运行时生效验证”
门禁为 **PASS**。local/global costmap 现使用同一个 24 顶点凸多边形：

```text
[[-0.399,-0.156],[-0.393,-0.163],[-0.373,-0.175],[-0.353,-0.18],
 [-0.162,-0.202],[0.231,-0.178],[0.238,-0.176],[0.24,-0.175],
 [0.247,-0.17],[0.354,-0.046],[0.354,0.045],[0.35,0.058],
 [0.245,0.173],[0.244,0.174],[0.24,0.177],[0.21,0.188],
 [0.205,0.189],[-0.169,0.194],[-0.174,0.194],[-0.361,0.174],
 [-0.38,0.169],[-0.397,0.157],[-0.398,0.156],[-0.399,0.154]]
```

`footprint_padding` 为 **0.035 m**，来源是实测姿态/落足包络的最坏方向统计尾差
`0.00961 m` 加半个 `0.05 m` costmap 栅格，即
`0.00961 + 0.025 = 0.03461 m`，再按 `0.005 m` 向上取整。它不是凭感觉添加的
“保险数字”。

该 PASS 只覆盖当前 Gazebo Go2 collision、CHAMP 步态和本轮速度。它不等于真机腿部
柔性、打滑、更快步态或跌倒姿态已验收；阶段 3 开始前仍保持
`safe/balanced/aggressive=UNCALIBRATED`。

后续现场发现 Global Costmap 幽灵障碍与近距方块碰撞，使系统级安全门改判为 FAIL；
这不推翻本阶段的几何 PASS。当前外扩 Footprint 外接半径 `0.474170 m` 仍是阶段 3
Inflation 的安全比较基准；V1.5.0 的 Global/Local 现场值分别为 `0.20/0.30 m`，均不应
被误写成已经满足该几何基准。ObstacleLayer clearing 和 `map→odom` 必须独立检查，
否则 Inflation 只会改变错误障碍格的可见大小。后续顺序见
[幽灵障碍与近距碰撞调查](costmap_ghost_obstacle_investigation.md)。

## 几何来源与算法

工具从运行中的 `/robot_state_publisher.robot_description` 读取 URDF，而不是复制一份
容易过期的尺寸表。共解析 21 个 collision：trunk、imu、四组 hip/upper leg/lower
leg/foot，以及 D435、Velodyne base 和 GPS。只接受 `box/cylinder/sphere`；遇到未实现的
mesh 会明确失败，不能静默漏掉部件。

每一帧通过 TF 把各 collision 从所属 link 投影到 `base_footprint` 的 XY 平面，再对
站立、前进、原地转向和横移全部样本取凸包。各方向极值的实际来源如下：

| 极值 | 数值 | collision link | 物理含义 |
|---|---:|---|---|
| 后方 `min_x` | -0.399337 m | `rh_upper_leg_link` | 旧矩形漏掉的后腿摆动包络 |
| 前方 `max_x` | 0.353896 m | `d435_link` | 前置相机外壳突出 |
| 右侧 `min_y` | -0.201784 m | `rh_foot_link` | 横移时右后足落足包络 |
| 左侧 `max_y` | 0.193757 m | `lh_foot_link` | 前进时左后足落足包络 |

这种做法符合 Nav2 对 footprint 的定义：它应是机器人几何在地面的二维投影，并用于
完整多边形碰撞检查。上游依据为
[Nav2 Footprint 设置指南](https://docs.nav2.org/setup_guides/footprint/setup_footprint.html)、
[Costmap2D 参数说明](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)
和
[Nav2 1.1.20 Costmap2DROS 源码](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/costmap_2d_ros.cpp)。
验证器的逐轴 padding 语义来自
[Nav2 1.1.20 footprint.cpp](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/footprint.cpp)，
该文件保留 Willow Garage BSD-3-Clause 版权头；Navigation2 仓库主体采用 Apache-2.0。
本工具独立表达相同数值关系并调用标准参数、TF 和消息接口，没有引入或链接新的第三方库。

## 正式样本

隔离 `ROS_DOMAIN_ID=228`、在线 SLAM、无 GUI 环境中，以 `10 Hz` 采集 220 帧；每帧
包含 21 个 link 的 TF，共保存 4620 条变换记录。速度命令只发到
`/cmd_vel_teleop`，必须经过
`twist_mux → velocity_smoother → collision_monitor → /cmd_vel`，工具会验证最终速度
达到命令的至少 45%。

| 场景 | 命令 | 有效采样 | 原始包络 `[min_x,max_x] × [min_y,max_y]` | 采样周期 p99 |
|---|---|---:|---|---:|
| 站立 | 0 | 40 帧 | `[-0.38497,0.35298] × [-0.16782,0.16738] m` | 0.110 s |
| 前进 | `x=0.15 m/s` | 60 帧 | `[-0.39934,0.35386] × [-0.19493,0.19376] m` | 0.107 s |
| 转向 | `z=0.30 rad/s` | 60 帧 | `[-0.39415,0.35390] × [-0.19437,0.19008] m` | 0.110 s |
| 横移 | `y=0.10 m/s` | 60 帧 | `[-0.39254,0.35313] × [-0.20178,0.18928] m` | 0.111 s |

毫米舍入后的原始凸包面积为 `0.266358 m²`，外接半径为 `0.428412 m`；加入
`0.035 m` padding 后外接半径为 `0.474170 m`。这些数值是阶段 3 Inflation
步进实验的几何起点，不在本阶段提前修改 Inflation。

旧配置是 `x=[-0.28,0.42] m、y=[-0.24,0.24] m` 的手写矩形，padding 仅
`0.01 m`。它在前方和左右偏大，却在后方漏掉约 `0.109 m` 的动态腿部包络；所以只比较
矩形面积或外接半径会得出错误结论。新轮廓按每个方向覆盖真实突出部件，而不是把所有
方向一起盲目放大。

## 动态更新与 `/published_footprint` 验证

本轮用 `nav_tuner` 分别原子更新 local/global footprint 和 padding，参数服务 read-back
完全一致，再由 `save` 备份两份受管配置并定点保存。测试同时修复了长 footprint 被
PyYAML 折成多行转义字符串的问题；现在输入会规范化为单行 JSON/YAML 标量。

`/local_costmap/published_footprint` 和
`/global_costmap/published_footprint` 均收到 24 个顶点。把消息从各自的 `odom/map`
坐标反变换回 `base_footprint` 后，实测边界为：

| 来源 | 外扩后 `[min_x,max_x] × [min_y,max_y]` |
|---|---|
| Local Costmap | `[-0.4340,0.3890] × [-0.2370,0.2290] m` |
| Global Costmap | `[-0.4340,0.3890] × [-0.2370,0.2290] m` |

验证使用每条 Polygon 自身的时间戳查询 TF，避免“最新 TF”与旧消息错时制造假误差；
若校准器刚启动时先收到的 Polygon 早于它自身 TF Buffer 的最早记录，会继续等待一帧
可与 TF 精确配对的新 Polygon，而不是把短暂的缓存预热报成校准失败。
Domain 228 热更新验证中 local/global 最大逐顶点误差分别为 `4.06×10⁻⁸ m` 和
`3.47×10⁻⁸ m`，与 Nav2
对各顶点施加 `0.035 m` padding 的结果一致。RViz 两套配置现默认显示绿色
`Robot Footprint (Padded)`，它订阅 local costmap 的实际发布结果，不是另画一条装饰线。

## 如何重复

先启动统一导航。校准时机器人会实际站立、前进、转向和横移，周围必须留出至少
`2 m` 空地：

```bash
source scripts/setup_simdog.bash
ros2 run go2_navigation footprint_calibrator \
  --output-dir simdog/src/go2/go2_navigation/logs/footprint/my_run

# 不依赖 console entry point
python3 simdog/src/go2/go2_navigation/tools/footprint_calibrator.py --help
```

正常输出应依次报告 4 个场景、总计约 220 帧，并给出推荐 polygon/padding。工具开始前
会取消旧目标并确认零速，为采样短暂解除人工暂停；结束后再次调用
`/navigation/stop`，不会恢复旧目标。确认结果和导航健康后才手动执行：

```bash
ros2 service call /navigation/resume std_srvs/srv/Trigger "{}"
```

若报告某场景最终速度不足，先查 `/pause_navigation`、twist mux 和 Collision Monitor，
不要绕过安全链直接发布 `/cmd_vel`。若报告 TF 或不支持的 collision，先修复 URDF/TF，
不能删掉报错部件后继续生成足迹。

参数保存或重启导航后，可不再走动机器人，只验证运行时内部效果：

```bash
ros2 run go2_navigation footprint_calibrator --verify-only \
  --output-dir simdog/src/go2/go2_navigation/logs/footprint/my_run
```

固定地图 AMCL 有一个必须先完成的前置步骤：在 RViz 顶部使用 `2D Pose Estimate` 设置
地图内的真实位置与朝向，并等待 `planner_server`、`controller_server` 都为 `active [3]`。
初始位姿前没有 `map→odom`，此时 local footprint 实际已在 `odom` 发布，但 RViz 的
Fixed Frame 为 `map`，无法画出绿色轮廓；global costmap 也会等待坐标变换而不能完成
激活。可用以下命令区分“尚未定位”与“footprint 真没发布”：

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
timeout 3s ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /local_costmap/published_footprint --once
ros2 topic echo /global_costmap/published_footprint --once
```

正常状态是两个 lifecycle 节点均为 `active [3]`、TF 打印数值、两个 Polygon 各有
24 个点。若 `map→odom` 未建立，校准器会给出 `2D Pose Estimate` 操作提示，并在预检
阶段退出；由于还没有接管速度，它不会改变原来的导航暂停状态。预检通过后，验证过程
仍会主动锁停，结束后需人工调用 `/navigation/resume`，旧目标不会续行。

期望 local/global 都显示 `status=PASS`、`vertex_count=24`，逐顶点误差不超过默认
`0.005 m`。该模式同样会锁停并且不自动 resume。

最终又在全新 Domain 229 从持久化 YAML 冷启动，`health_check --expected-domain-id 229`
为 PASS，`/cmd_vel` 的唯一发布者是 `/collision_monitor`；冷启动后的 local/global
最大逐顶点误差均不超过 `2.11×10⁻⁸ m`。因此不是只在旧进程中热更新成功。

本轮两次 Ctrl+C 关闭隔离 Gazebo 时，`champ_gazebo/contact_sensor` 在退出阶段触发
Boost recursive mutex assertion 并以 `-6` 结束；Nav2、Gazebo server 与其余节点均正常
清理。该问题只发生在统一进程退出后，不影响本轮已落盘样本或运行时 footprint
判定，但作为 CHAMP ContactSensor 的既有退出问题保留，不把它误写成“全部进程干净退出”。

正式原始证据位于
`logs/footprint/stage2_footprint_20260814/`：

- `footprint_calibration_result.json`：URDF SHA-256、全部 collision、逐场景/逐 link
  凸包、padding 推导和推荐值；
- `footprint_samples.csv`：220 帧包络、最终速度与 odom；
- `footprint_transforms.csv`：4620 条可重放的 collision-link TF。
- `published_footprint_verification.json`：参数、发布轮廓、同时间戳 TF 还原点与逐顶点
  误差。
