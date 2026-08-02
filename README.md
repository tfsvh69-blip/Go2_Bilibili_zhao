# Go2 ROS 2 仿真工作区

这是一个面向 Unitree Go2 的 ROS 2 Humble 开发环境，使用 Gazebo Classic 11 进行仿真。主工作空间 `simdog/` 支持 CHAMP 四足步态、Velodyne 点云、IMU、LIO-SAM 建图和键盘遥控；`go2_ws/` 提供轻量级平面移动备选仿真，适合导航、SLAM 与视觉上层功能开发。

## 快速启动主仿真

首次使用或源码有改动时，先构建：

```bash
cd /home/luhao/my/ROS/Go2/simdog
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

随后用三个终端分别启动仿真、建图和遥控。每个终端均应加载同一个 `simdog` 工作空间。

```bash
# 终端一：Go2 仿真与第一个 RViz2
cd /home/luhao/my/ROS/Go2/simdog
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch go2_config gazebo_velodyne.launch.py gui:=false rviz:=true

# 终端二：LIO-SAM 与第二个 RViz2
cd /home/luhao/my/ROS/Go2/simdog
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lio_sam lidar.launch.py

# 终端三：键盘遥控
cd /home/luhao/my/ROS/Go2/simdog
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

遥控常用按键：`i` 前进、`,` 后退、`j`/`l` 转向、`k` 停止。LIO-SAM 可视化窗口中可查看点云、轨迹和地图；额外运行 `rviz2 --ros-args -p use_sim_time:=true` 可新开传感器观察窗口，并添加 `/velodyne_points`、`/imu/data`、`/odom/local` 或 `/scan`。

## 其他启动入口

- `bash simdog/start.sh`：启动仿真、LIO-SAM、遥控和 NDT 重定位。NDT 需要已有 PCD 地图，初次建图时建议手动启动前三项。
- `bash simdog/save_Map.sh`：保存 LIO-SAM 地图。
- `go2_ws/`：备选平面移动仿真。进入目录、加载 `install/setup.bash` 后运行 `ros2 launch go2_gazebo spawn.launch.py gui:=false rviz:=true`。

## 注意事项

不要在同一终端同时加载 `simdog` 和 `go2_ws`。本机 Gazebo 图形界面可能因显卡兼容性问题崩溃，因此推荐 `gui:=false` 配合 RViz2；必要时可使用 `LIBGL_ALWAYS_SOFTWARE=1` 软件渲染。更多当前进度、验证记录和限制见 [PROJECT_MEMORY.md](PROJECT_MEMORY.md)，协作规范见 [AGENTS.md](AGENTS.md)。
