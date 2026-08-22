# 2026-08-22 `/scan` marking/clearing 冒烟记录

## 实验条件

- 完整入口：`go2_navigation simulation_navigation.launch.xml`，
  `navigation_mode:=online_slam`、`map_session:=new`、无 Gazebo/RViz GUI；
- 隔离通信：CycloneDDS、`GO2_UNITREE_SIM_DOMAIN_ID=220`；
- 障碍：正前方 2.0 m，`0.3×0.3×0.5 m` 方块；
- 探针：`scan`，1 组 × 10 帧，ContactSensor 同步监测；
- Local Costmap：并行观察 `/local_costmap/costmap_raw` 中 cost=254 的致命栅格数。

## 已观察结果

| 阶段 | Local Costmap 致命格 |
|---|---:|
| 放置方块前背景 | 约 227 |
| 方块存在时峰值 | 237 |
| 删除后约 6.1 s | 226 |
| 后续稳定范围 | 225–228 |

方块 10/10 帧检出，距离绝对误差 p95 为 `0.003029 m`，TF 成功率 100%，接触事件为 0。
删除后致命格回到背景范围，本次空射线 clearing 冒烟测试判为 PASS。

CSV/JSON 是 `obstacle_probe` 的原始输出；上表是同一次运行中并行终端对
`/local_costmap/costmap_raw` 的墙钟观察摘要，没有把近似读数伪装成逐帧原始记录。

## 结论边界

本记录只覆盖单个位置、单次删除，不等于静止 60 s、多位置重复、移动障碍或 10 分钟
压力验收，也不证明 `0.90 m` 内的近距防撞已经解决。
