# Go2 Unitree 仿真兼容桥

本包保留 Gazebo、CHAMP 和现有感知导航栈，并把仿真状态与控制转换为 Unitree
官方 `unitree_go`、`unitree_api` 消息。它用于上层任务和导航程序的接口级
Sim-to-Real 验证，不模拟真机固件内部的平衡策略和安全保护。

默认发布：

- `/sportmodestate`（50 Hz）、`/lf/sportmodestate`（10 Hz）
- `/lowstate`（100 Hz）、`/lf/lowstate`（10 Hz）
- `/api/sport/response`（按请求）

订阅 `/api/sport/request`，支持 `BalanceStand`、`StopMove`、`StandUp`、
`StandDown`、`RecoveryStand`、`Euler`、`Move`、`Sit`、`RiseSit`、`Hello`、
`Stretch` 和 `Dance1`。未支持接口会返回非零模拟器错误码。

| 请求 | API ID | 仿真实现 |
|---|---:|---|
| BalanceStand | 1002 | 停止速度并进入平衡站立状态 |
| StopMove | 1003 | 清零 `/cmd_vel`，取消当前动作 |
| StandUp / RiseSit / RecoveryStand | 1004 / 1010 / 1006 | `stand` 服务 |
| StandDown / Sit | 1005 / 1009 | `lie` 服务 |
| Euler | 1007 | 安全范围检查后发布 `/body_pose` |
| Move | 1008 | 限幅并持续发布 `/cmd_vel` |
| Hello / Stretch / Dance1 | 1016 / 1017 / 1022 | 现有动作服务 |

`Move` 的 `parameter` 是 `{"x": 前进速度, "y": 横向速度, "z": 偏航速度}`，
限幅依次为 `±0.3 m/s`、`±0.25 m/s` 和 `±0.5 rad/s`。`Euler` 同样使用
`x/y/z` 表示 roll/pitch/yaw，默认安全范围为 `±0.35`、`±0.35`、`±0.5 rad`。
参数必须是有限数值。

错误状态为 `-32001` 参数错误、`-32002` 不支持、`-32003` 忙、`-32004`
下游执行失败。响应原样复制请求的 `identity`；请求设置 `noreply=true` 时不发布
响应。

## 状态来源

- `SportModeState` 的位置、速度和高度来自 `/odom/ground_truth`，IMU 从 ROS
  `xyzw` 转换为 Unitree `wxyz`，足端严格按 `FR、FL、RR、RL` 从 TF 填充。
- `LowState` 前 12 个电机严格按 `FR、FL、RR、RL` 填充关节位置、速度、
  差分加速度和有限的估计力矩；其余 8 个电机保持默认值。
- CHAMP 足底接触顺序会转换为 Unitree 顺序，并以 `0/1` 写入。
- `/odom/ground_truth`、`/imu/data`、`/joint_states` 首次全部就绪前不发布。
  任一输入超过 `source_timeout` 时停止活动 Move、设置 `error_code` 并节流告警。

## 运行

推荐先加载 CycloneDDS 仿真环境，再启动完整 Gazebo：

```bash
source scripts/setup_unitree_sim.bash
ros2 launch go2_config gazebo_velodyne.launch.py gui:=false rviz:=true
```

可通过 `unitree_bridge:=false` 关闭兼容桥。`Move` 会持续占用 `/cmd_vel`，直到
收到 `StopMove`；此期间不要同时使用键盘遥控。

快速检查：

```bash
ros2 topic type /sportmodestate
ros2 topic hz /sportmodestate
ros2 topic hz /lowstate
ros2 topic echo --once /lowstate
```

## 已知边界

- 足底接触只以 `0/1` 填入，并非真实力值；关节源为非有限值时对应估计力矩为零。
- `range_obstacle`、BMS、温度、序列号和 CRC 没有可信仿真来源，保持零。
- 不订阅 `/lowcmd`，不发布无线遥控器状态，不支持翻转等高风险动作。
- 仿真动作轨迹不可直接下发真机。

接口定义来自 Unitree 官方 `unitree_ros2 v0.3.0`，许可证与来源记录见
`simdog/src/platform/unitree_ros2_interfaces`。
