# 动态高度窗口运行验证（2026-08-22）

## 验证目的

只验证 `min_height/max_height` 是否真正进入投影循环、非法/不支持修改是否被拒绝，
以及新默认值能否在隔离 Gazebo 中稳定出流。本页**不证明**在线 SLAM 幽灵图已经消失。
后续用户把窗口调为 `0.20..0.30 m` 后反馈目视正常；当前默认值与结论见
`height_020_030_field_feedback_20260822.md`。

## 合成点云

- 正式转换器以 `allow_runtime_height_update=true` 启动；
- 同一份 10000 点固定随机点云，`[6.0,7.0] m` 窗口有限端点为 0；
- 动态恢复 `[0.05,0.20] m` 后有限端点为 123；
- `min_height >= max_height` 被拒绝，`range_min` 热修改也被拒绝且读回不变；
- `allow_runtime_height_update=false` 的原始转换器拒绝高度修改并保持 `-0.05 m`。

## 隔离 Gazebo

启动档：D435 关闭、Gazebo 无界面、RViz 关闭、同时启用 `/scan_raw`。

- 正式 `/scan` 参数：`0.05..0.20 m`；原始 `/scan_raw`：`-0.05..0.10 m`；
- `/scan` 墙钟频率约 `9.98–10.00 Hz`；
- 静止单帧有限端点：正式 658、原始 427。数量多不等于质量好，只证明两窗确实不同；
- 正式窗口动态改到 `0.00..0.15 m` 后单帧有限端点变为 649；随后恢复默认；
- `/diagnostics` 最终显示 `min_height_m=0.050`、`max_height_m=0.200`、扫描 10.01 Hz、
  同时间戳 TF 成功率 100%、非法值 0；
- rqt 参数插件在正式转换器存在时可正常启动并选中该节点。

只读 `motion_scan_probe` 还用故意错误的本地初值 `-0.50..-0.40 m` 做了晚启动检查，
随后通过 Humble 原生 `/go2_lidar_scan_converter/get_parameters` 服务自动同步为
`0.05..0.20 m`。因此先开 rqt、后开探针时不会把旧默认写进证据。

## 下一验收门

该页的候选实验已由后续现场反馈推进到 `0.20..0.30 m`。现在保持该窗口，用新的
`map_session:=new` 完成整场覆盖、保存与固定 AMCL 导航；若再次复现，再记录白色扇形线、
两张 raw costmap、`motion_scan_probe` CSV/JSON 和前后截图。
