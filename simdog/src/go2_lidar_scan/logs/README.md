# 2026-08-22 雷达修复证据索引

本目录只保存项目内可复核的数据，不把一次短跑写成系统级长期 PASS。

> 2026-08-22 状态更正：表内 PASS 只对应各目录记录的量化子门。后续人工在线 SLAM 曾
> 复现白色扇形线与幽灵代价岛；用户把高度窗调为 `0.20..0.30 m` 后目视正常，该值已
> 固化。完整建图、保存和固定图导航仍待验收。

| 目录/文件 | 内容 | 判定 |
|---|---|---|
| `height_020_030_field_feedback_20260822.md` | 用户通过 rqt 得到 `0.20..0.30 m`，反馈在线 SLAM 目视正常 | 现场目视 PASS；完整闭环待验收 |
| `dynamic_height_runtime_20260822.md` | 动态回调、拒绝语义、合成点云与隔离 Gazebo 静止验证 | 运行机制 PASS；运动幽灵图待人工复测 |
| `motion_scan_online_level_fix_20260822/` | 首次在线 A/B：最大倾斜 5.30°，原始 2522、对齐 0 个地面端点；当时 `map→odom` 仍超门 | 雷达对齐 PASS，里程计 FAIL |
| `motion_scan_online_production_20260822/` | 关闭 `/scan_raw`、去掉真值人工噪声后的生产链运动采样 | 地面端点 PASS，频率 FAIL |
| `motion_scan_online_final_20260822/` | C++ 对齐节点最终运动采样：最大倾斜 4.34°，原始 106、对齐 0；同时间戳 TF 100%；`map→odom=0.0106 m/0.00349 rad` | 几何/TF/SLAM 修正 PASS，3.26 Hz FAIL |
| `motion_scan_final_20260822/` | 1800 水平列、完整正反转/闭合路线：最大倾斜 5.19°，原始 1887、对齐 0；TF/SLAM 修正通过 | 几何 PASS，`5.06 Hz` 性能 FAIL |
| `motion_scan_final_900_20260822/` | 900 水平列：240 帧，最大倾斜 5.15°，原始 4430、对齐 0；同时间戳 TF 100%；`map→odom=0.0224 m/0.00698 rad` | **该次探针 PASS，`8.89 Hz`；非整体根治** |
| `clearing_smoke_20260822/` | `+inf` clearing 契约的首次实际删除方块冒烟 | 单位置 PASS |
| `clearing_after_level_fix_keep_20260822/` | 重力对齐后方块仍存在时的对照采样 | 方块 10/10 检出 |
| `clearing_after_level_fix_20260822/` | 同一方块经 `/delete_entity` 实际删除后的采样 | lethal 格回到 146–149 背景带，PASS |
| `clearing_final_900_20260822/` | 2 m 方块，scan/Velodyne 各 3 组 × 30 帧 | 两路检出率 100%，无接触 |
| `clearing_entity_exact_final_900_20260822/` | 方块精确区域删除前后对照 | lethal 格 `64→57`，删除前背景 55，`+inf` clearing PASS |
| `min_range_050_front_20260822/` | 临时完整配置 0.50 m，前方四距离各 3×10 帧 | 全部 100%，无接触 |
| `min_range_050_left_20260822/` | 同上，左方 | 全部 100%，无接触 |
| `min_range_050_rear_20260822/` | 同上，后方 | 0.50 m 为 0% 且发生接触，FAIL |
| `min_range_050_right_20260822/` | 同上，右方 | 0.50 m 检出但发生接触，FAIL |
| `tf_frames_20260822/` | 两次运行的 TF 树 PDF/GV | 含动态 `base_footprint→velodyne_level` |

最终默认是 900 水平列（0.4°）、16 线和 10 Hz 标称值。1800→900 只解决 Gazebo
实时调度：历史 900 列旧投影同样会产生幽灵障碍；重力对齐消除了已采样的地面端点，
但未消除后续人工复现的全部幽灵图。
导航仿真默认不实例化与本任务无关的 D435 Gazebo 渲染插件，普通 Gazebo 启动仍保持
D435；可用 `use_d435_navigation:=true` 做显式性能 A/B。最终 135 秒运动中 costmap 记录
到 2 次 `OutTheBack` 旧观测丢弃，未观察到错误端点进入 `/scan`，该残余没有从证据中删除。

0.50 m 候选测试时相关六处配置作为一组临时修改，测试后已全部恢复 0.90 m。后方
0.50 m 三组 scan 检出率均 0%，接触事件分别约 2950/2942/3000；右方三组虽检出，仍有
约 965/968/1016 次接触。JSON 的 `status=complete` 只代表采集完成，不代表候选通过，
应以各行 `pass`、检出率、误差和接触事件共同判定。
