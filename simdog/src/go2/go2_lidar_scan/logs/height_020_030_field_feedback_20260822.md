# `0.20..0.30 m` 高度窗现场反馈与固化验证（2026-08-22）

## 证据来源

- 用户在完整在线 SLAM 的 rqt 参数界面把
  `/go2_lidar_scan_converter.min_height` 设为 `0.20 m`、`max_height` 设为 `0.30 m`；
- 用户反馈该窗口下 SLAM 画面“很正常”，并提供两个参数字段的截图；
- 这是人工现场 A/B 反馈，不是本目录工具自动生成的 CSV/JSON。

## 项目处理

- 正式 `/scan` 默认高度窗固化为 `0.20..0.30 m`；
- 诊断期望值、`motion_scan_probe` 默认值和配置回归测试同步；
- `/scan_raw` 继续固定旧 `-0.05..+0.10 m`，不接入 SLAM/Nav2；
- 量程、水平分辨率、TF、SLAM 匹配与 costmap persistence 本轮不改。

## 固化后的项目验证

- `pointcloud_to_laserscan`、`go2_lidar_scan`、`go2_navigation` 构建通过；
- 当前工作区汇总：`259 tests, 0 errors, 0 failures, 6 skipped`；
- 隔离 Domain 226 无界面 Gazebo 读回正式/原始窗口分别为
  `0.20..0.30 m`、`-0.05..+0.10 m`；
- `/scan.frame_id=velodyne_level`，墙钟频率约 `9.99 Hz`；
- `/diagnostics` 为“转换链正常”，显示 `scan_hz=10.00`、level roll/pitch 为
  `0.000°/0.000°`、当前窗口 `0.200..0.300 m`。

以上证明 YAML 默认值已进入实际投影与诊断进程；它不代替用户驾驶的运动/地图验收。

## 结论边界

当前判定为“用户现场目视通过”。只有按空白会话完整覆盖场景、保存地图并使用固定
`static_map + AMCL` 完成导航后，才能把端到端地图输入门写成通过。Collision Monitor
近距盲区、Inflation 与重复碰撞验收是独立安全门，不随本结果自动通过。
