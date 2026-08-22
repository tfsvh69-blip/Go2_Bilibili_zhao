# V1.5.0 发布说明

发布日期：2026-08-22

## 遇到的问题

Go2 在在线 SLAM 中转向或移动时，地图曾出现从机器人向空白区域延伸的白色放射线；
Global/Local Costmap 随后把错误端点膨胀成脱离真实障碍的青色、粉色“幽灵代价岛”。
提高刷新率只能让旧区域更快清除，没有阻止错误端点产生。

## 本版本怎样处理

- 新增独立 `go2_lidar_scan` 包，统一维护 `/velodyne_points -> /scan`；复用
  BSD-3-Clause 上游 `pointcloud_to_laserscan`，没有重写投影算法；
- 每帧按点云时间戳生成重力对齐的 `velodyne_level`，去掉机身步态带来的 roll/pitch，
  TF 失败时不复用陈旧变换；
- 正式 `/scan` 高度窗根据用户现场 A/B 固化为 `0.20..0.30 m`，并保留 rqt 动态调节；
  `/scan_raw` 固定旧窗口，只用于 RViz 对照，不接入 SLAM/Nav2；
- 恢复 `+inf` 空射线 clearing 契约，Nav2 配套启用 `inf_is_valid=true`，避免真实障碍删除后
  长时间残留；
- 增加扫描频率、延迟、TF、frame、高度窗诊断，增加只读运动探针和 RViz 对照视图；
- 在线地图按 `go2_maps/online/<地图名>/` 原子归档，保存 PGM/YAML、Slam Toolbox
  pose graph/data 和记录实际雷达参数的 `session.yaml`；
- Global Costmap 调整为 `inflation_radius=0.20 m`、`cost_scaling_factor=0.5`；
  Local Costmap 保持 `0.30 m/3.0`，避免同时改变近场避障变量。

## 当前结果

用户现场反馈：使用 `0.20..0.30 m` 雷达高度窗后，在线 SLAM 已基本不会出现幽灵代价
区域；偶尔仍可能在很远的位置出现残余。当前结论是“主要现象基本解决”，不是绝对零残余。
高度窗和重力对齐用于减少错误输入；全局膨胀参数只缩小端点周围的代价影响范围，不能删除
仍然存在的远处错误端点。

隔离 Gazebo 中正式高度窗读回为 `0.20..0.30 m`，`/scan.frame_id` 为
`velodyne_level`，频率约 `9.99 Hz`，诊断正常。发布前构建通过，工作区测试汇总为
`260 tests, 0 errors, 0 failures, 6 skipped`。完整在线栈冷启动回读 Global Inflation 为
`0.20 m/0.5`、Local 为 `0.30 m/3.0`，`planner_server` 为 `active [3]`。

## 已知边界

- `inflation_radius=0.20 m` 小于当前外扩 Footprint 的外接半径，可能使全局路径更贴近
  墙体；它是用户指定的现场基线，不代表 Collision Monitor、Inflation 和近距防撞已经
  完成系统安全标定；
- 远处偶发残余仍需保存 `/scan`、TF、`/map` 和 raw costmap 证据继续分流；
- 正式地图应从 `map_session:=new` 开始重新采集，旧 pose graph 中已经写入的幽灵线不会
  因升级参数自动消失。
