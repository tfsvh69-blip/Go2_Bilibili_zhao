# 项目记忆

## 当前状态

更新时间：2026-08-02

本目录包含两个相互独立的 ROS 2 Humble 工作空间：`simdog/` 是当前主用的 Go2 四足仿真，`go2_ws/` 是用于导航、SLAM 与视觉上层开发的平面移动备选仿真。二者不可在同一终端叠加加载。

源码已于 2026-08-02 同步至 GitHub 仓库 `tfsvh69-blip/Go2_Bilibili_zhao` 的 `main` 分支。远端仅保存可复现源码、启动脚本、工作区设置和项目文档；`build/`、`install/`、`log/`、IDE 缓存及参考压缩/CAD 文件不纳入版本控制。

## 已完成阶段

### 阶段一：主仿真与遥控

已完成 `simdog/` 的 Gazebo Classic 仿真部署。Go2 通过 CHAMP 与 `ros2_control` 产生步态，Velodyne 点云、IMU、里程计和关节状态可供 ROS 2 使用。使用 `ros2 run teleop_twist_keyboard teleop_twist_keyboard` 向 `/cmd_vel` 发布控制指令，可键盘遥控机器人。

启动主仿真的推荐方式：

```bash
cd /home/luhao/my/ROS/Go2/simdog
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch go2_config gazebo_velodyne.launch.py gui:=false rviz:=true
```

本机 RTX 5070 与 Gazebo Classic 图形界面存在兼容性问题，`gzclient` 可能崩溃；默认保持 `gui:=false`，使用 RViz2 观察模型与传感器。若必须打开 Gazebo 界面，使用 `LIBGL_ALWAYS_SOFTWARE=1` 软件渲染，性能会下降。

### 阶段二：SLAM 与多窗口可视化

LIO-SAM 已接入 Velodyne、IMU 与仿真时间。另开终端执行以下命令，会打开带 LIO-SAM 配置的第二个 RViz2 窗口：

```bash
cd /home/luhao/my/ROS/Go2/simdog
source install/setup.bash
ros2 launch lio_sam lidar.launch.py
```

第一个 RViz2 由主仿真的 `rviz:=true` 启动，第二个用于 LIO-SAM 地图、轨迹和点云。还可在已加载环境的终端直接执行 `rviz2`，在“Add”中添加 `/velodyne_points`、`/imu/data`、`/odom/local`、`/scan` 等显示项；每个窗口均应勾选 `Use Sim Time`。

快捷启动脚本为 `simdog/start.sh`，它会启动仿真、LIO-SAM、键盘遥控和 NDT 重定位。仅在已有 PCD 地图并将其路径传入 NDT 启动文件时再启用重定位；否则可按上述命令分别启动前三项。建图完成后使用 `bash save_Map.sh` 保存地图。

### 阶段三：平面移动备选仿真

`go2_ws/` 已完成刚体平面移动方案：12 个腿部关节固定，`planar_move` 根据 `/cmd_vel` 驱动底盘，并发布 `/odom`、`/scan`、`/imu`、相机和 TF。其启动方式：

```bash
cd /home/luhao/my/ROS/Go2/go2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch go2_gazebo spawn.launch.py gui:=false rviz:=true
```

## 验证记录

启动后使用以下命令检查基础接口；有输出或稳定频率即表示对应链路正常。

```bash
ros2 topic echo --once /odom
ros2 topic hz /scan
# simdog 使用 /imu/data；go2_ws 使用 /imu
ros2 topic hz /imu/data
ros2 topic list | rg 'cmd_vel|velodyne_points|joint_states'
```

## 已知限制与下一步

- `simdog` 的 Nav2 启动配置尚未完成，不应作为当前可用的自主导航入口。
- NDT 重定位的默认地图路径为历史路径，启动前必须传入实际 PCD 文件。
- 后续优先完成：稳定建图与保存流程、地图加载和 NDT 参数化、再配置 Nav2 自主导航。

## 维护规则

每完成一个可验证阶段或修改关键启动/运行方式后，更新本文件的日期、操作、验证结果、限制和下一步。合并或删除过期信息，只保留一份当前文档，不创建带版本号的副本。
