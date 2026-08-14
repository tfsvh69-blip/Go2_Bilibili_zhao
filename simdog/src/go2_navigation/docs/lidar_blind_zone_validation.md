# Go2 Gazebo 激光雷达近距盲区量化

## 结论与阶段门禁

实测日期：2026-08-14。阶段 1 的“可重复地量化盲区”门禁为 **PASS**，但这不等于
传感器在所有方向都通过安全验收。已确认的边界是：

- 0.30 m 深、0.30 m 宽、0.50 m 高的标准方块位于激光雷达水平面、正前方时，
  `/velodyne_points` 与 `/scan` 的 `reliable_detection_min_distance` 均为 **0.90 m**。
- 左右 90° 方向在 0.90 m 虽然每帧都有回波，但 p95 距离误差为 5.5–7.1 cm，
  超过 5 cm 门槛；可靠下限是 **1.00 m**。
- 正后方的 `gpu_ray` 存在 ±π 角度拼接缝：170° 的 1.20 m 方块三组均为
  100%，175°、179° 和 180° 三组均为 0%。因此不得仅根据 xacro 中的
  `-pi..pi` 配置就宣称“后方已覆盖”。
- D435 小样本能看到 1.20 m 和 0.80 m 方块，但本轮仅 1 组×3 帧，而且整体
  帧间隔 p50/p95/p99 为 12.34/64.77/72.14 s。1.20 m 组的距离误差 p95 为
  6.21 cm。Collision Monitor 在同次运行中也明确报告 D435 时间戳落后 2–6 s
  并忽略该 source。它只记为诊断线索，**不得作为已验证的盲区安全 source**。

## 工具与距离定义

工具复用 Gazebo Classic 的标准 `/spawn_entity`、`/set_entity_state`、
`/get_entity_state` 和 `/delete_entity`，动态生成红色方块。方块 collision 附带
`libgazebo_ros_bumper.so` ContactSensor，因此“检测到”和“真实接触”是两条独立数据链。
实现只调用标准 ROS/Gazebo 接口，没有新增自定义消息或服务。

距离统一定义为：**Velodyne 光学原点沿测试方向到方块前表面的法向距离**。
Gazebo 会合并固定关节，不能直接查询 `go2::velodyne`，所以工具用可查询的
`go2::base_link` 加 URDF 静态偏移 `(0.20, 0.00, 0.1177) m`，并在每次移动后
回读实际位置；误差超过 1 cm 则拒绝采样。方块默认中心与雷达水平面对齐，
高度对照因此表示“穿过雷达水平面的障碍垂直厚度”，不等价于已验收所有
贴地矮障碍。

参考的上游实现是 [Gazebo Classic ContactSensor 教程](https://classic.gazebosim.org/tutorials?cat=sensors&tut=contact_sensor)
和 [ROS 2 Humble `gazebo_msgs/SpawnEntity`](https://docs.ros.org/en/ros2_packages/humble/api/gazebo_msgs/srv/SpawnEntity.html)。
`gazebo_ros` 是 Apache-2.0，`gazebo_plugins` 声明 BSD/Apache-2.0，与本包兼容；
Gazebo Classic 已 EOL，本项目仍固定于 Ubuntu 22.04/ROS 2 Humble/Gazebo Classic 11，
本阶段不引入新仿真器迁移风险。

## 重复方法

先启动隔离导航仿真，再在同一 `ROS_DOMAIN_ID` 运行：

```bash
ros2 run go2_navigation obstacle_probe \
  --sensors scan,velodyne \
  --frame-timeout 60 \
  --settle-seconds 1 \
  --replace-existing \
  --output-dir simdog/src/go2_navigation/logs/blind_zone/my_run

# 不依赖 console entry point
python3 simdog/src/go2_navigation/tools/obstacle_probe.py --help
```

默认距离序列为
`1.2/1.1/1.0/0.9/0.8/0.6/0.5/0.4/0.3/0.25/0.2/0.15 m`，
每个距离 3 组，每组 20 帧。工具会先调用 `/navigation/stop`、等待最终
`/cmd_vel` 归零，实验后不恢复旧目标。仅当下列条件在连续三组都成立时才 PASS：

1. 检测率不低于 95%；
2. 绝对距离误差 p95 不大于 0.05 m；
3. TF 成功率为 100%。

每组完成后都会更新帧级 CSV、汇总 CSV 和 JSON；中途超时保留已完成组。

## 正前方正式数据

| 表面距离 | `/scan` 三组检测率 | `/velodyne_points` 三组检测率 | 结果 |
|---:|---:|---:|---|
| 1.20/1.10/1.00/0.90 m | 每组 100% | 每组 100% | PASS |
| 0.80/0.60/0.50/0.40 m | 每组 0% | 每组 0% | FAIL |
| 0.30/0.25/0.20/0.153 m | 每组 0% | 每组 0% | FAIL |

在全部正前方帧中，`/scan` 帧间隔 p50/p95/p99 为
`0.158/0.354/0.480 s`，原始 Velodyne 为 `0.158/0.338/0.484 s`。在最近可靠点
0.90 m，`/scan` 三组中最大距离误差 p95 为 0.62 cm，原始点云为 0.45 cm。
0.153 m 组出现 442 个 ContactSensor 事件，但两个激光话题仍全为 0%，证明
“没看见”不能代替接触真值。

## 根因区分

| 检查项 | 已实测/静态证据 | 判断 |
|---|---|---|
| 量程下限 | xacro 的 `gpu_ray <range><min>` 和 Velodyne plugin `min_range` 均为 0.9 m | 正前 0.90→0.80 m 断崖的主因 |
| 二维切片 | `pointcloud_to_laserscan` 为 `-0.05..0.10 m`；原始点云与 `/scan` 同时跳变 | 不是正前方盲区的主因 |
| TF | `/scan` frame 为 `velodyne`；D435 转到 `velodyne` 的 TF 能成功查询 | 正式 LiDAR 样本 TF 成功率 100% |
| 障碍垂直厚度 | 1.00 m 处，0.10 m 厚三组 100%；0.02 m 厚三组仅 55–65% | 受 VLP-16 垂直角分辨率影响，不是单纯的 scan 高度切片 |
| 方向 | 170° PASS，175/179/180° FAIL，原始点云与 `/scan` 一致 | 仿真 `gpu_ray` 在 ±π 有后向拼接缝 |

本阶段不改 `range_min`，也不用未标定的 D435 盖住结果。这些数据作为后续
Footprint、Inflation、停车余量与 Collision Monitor 的输入；其中正后向缝隙与
D435 低频必须在阶段 5–6 再决定是否修正或禁用相应 source。

## 原始证据

全部数据位于 `logs/blind_zone/`：

- `stage1_lidar_20260814`：正前方 12 个距离，1440 帧；
- `stage1_direction_left/right/rear*_20260814`：方向对照；
- `stage1_height_0p10/0p02_20260814`：垂直厚度对照；
- `stage1_d435_diagnostic_20260814`：明确标记为非正式的 D435 小样本诊断。
