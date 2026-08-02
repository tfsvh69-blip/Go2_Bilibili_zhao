# 仓库指南

## 语言要求

本文件、后续新增或修改的代码注释、文档、提交说明和协作回复均使用中文。命令、ROS 话题、包名、文件名及必要的技术术语保持原样。

## 项目结构与模块组织

此目录是 Go2 机器人开发环境，而非单一代码仓库；目标环境为 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Classic 11。

- `simdog/`：主 colcon 工作空间，包含 CHAMP 步态、Go2 Gazebo 配置、Velodyne/RealSense 仿真、LIO-SAM 与重定位功能；源码位于 `simdog/src/`。
- `go2_ws/`：轻量级平面移动仿真工作空间。`src/go2_description/` 存放 URDF、网格和 RViz 配置；`src/go2_gazebo/` 存放世界、启动文件和生成的仿真 URDF。
- `Go2 URDF/`、`Go2简化模型/` 和 `学习文档/` 是参考资源，不用于功能开发。`build/`、`install/`、`log/` 为构建产物。

## 构建、测试与开发命令

每个终端只能构建并加载一个工作空间，不要将 `simdog` 与 `go2_ws` 叠加使用。

```bash
cd simdog && colcon build --symlink-install && source install/setup.bash
ros2 launch go2_config gazebo_velodyne.launch.py rviz:=true

cd go2_ws && colcon build && source install/setup.bash
ros2 launch go2_gazebo spawn.launch.py gui:=false rviz:=true
ros2 launch go2_description display.launch.py
```

修改 `go2_ws/src/go2_gazebo/scripts/gen_planar_urdf.py` 后，按 `go2_ws/README.md` 中的命令重新生成 `urdf/go2_gazebo.urdf`，再执行 `colcon build --packages-select go2_gazebo`。修改 simdog 的 xacro 后，通常可直接重新启动仿真。

## 常用启动方式与脚本

优先使用主仿真工作空间 `simdog/`。进入该目录并完成构建、加载后，执行 `bash start.sh` 可在独立终端依次启动无界面 Gazebo、LIO-SAM、键盘遥控和 NDT 重定位。NDT 重定位依赖已有 PCD 地图，尚未提供地图时不要启动它；建图结束可执行 `bash save_Map.sh` 保存 LIO-SAM 地图。

也可按需分终端启动：`ros2 launch go2_config gazebo_velodyne.launch.py gui:=false rviz:=true` 启动仿真及第一个 RViz2；`ros2 launch lio_sam lidar.launch.py` 启动 LIO-SAM 及第二个 RViz2；`ros2 run teleop_twist_keyboard teleop_twist_keyboard` 启动键盘控制。Gazebo 图形界面在本机显卡环境下可能崩溃，默认使用 `gui:=false`；需要时可用 `LIBGL_ALWAYS_SOFTWARE=1` 进行软件渲染。`go2_ws/` 是平面移动备选仿真，使用其 README 中的 `spawn.launch.py` 与 `display.launch.py`。

## 代码风格与命名

遵循相邻包的既有风格。Python 与 C++ 均使用四个空格缩进；Python 文件和函数、ROS 参数与话题使用 `snake_case`，C++ 类使用 PascalCase，ROS 包名使用小写下划线。启动文件应表达用途，例如 `gazebo_velodyne.launch.py`；新启动文件优先使用 XML 格式（`.launch.xml`）。保持 URDF/xacro 格式，勿手动编辑生成文件。

## 测试要求

当前未配置自动化测试或代码检查工具。修改后应构建受影响的包，并以无界面方式启动相应仿真，检查关键接口：

```bash
ros2 topic echo --once /odom
ros2 topic hz /scan
# simdog 使用 /imu/data；go2_ws 使用 /imu
ros2 topic hz /imu/data
```

传感器或控制改动还应验证 `/clock`、`/joint_states` 和 `/cmd_vel`，并说明所需地图、硬件或 Gazebo GUI 条件。

## 提交与合并请求

该目录没有可用 Git 元数据，因此无法从提交历史归纳规范。后续提交使用简短祈使句和范围，例如 `go2_gazebo: 调整雷达坐标系`，每次提交只处理一个主题。合并请求应说明受影响工作空间、构建与启动验证结果、修改的 ROS 话题或坐标系；涉及可视化时附 RViz 或 Gazebo 截图。

## 项目记忆维护

将当前项目状态统一维护在根目录 `PROJECT_MEMORY.md`。每次完成一个可验证阶段、修改启动入口、变更关键参数或发现运行限制后，必须同步更新：日期、阶段目标、实际操作与结果、验证命令、遗留问题和下一步。该文件只保留当前准确状态和必要的阶段记录；新信息应合并或替换过期条目，不创建 `PROJECT_MEMORY_v2.md` 等旧版本副本。
