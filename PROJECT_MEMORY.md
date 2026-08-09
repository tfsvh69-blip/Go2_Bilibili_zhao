# 项目记忆

## 当前状态

更新时间：2026-08-09

本目录当前只维护 `simdog/` 一个 ROS 2 Humble colcon 工作空间。它使用 CHAMP
生成完整四足步态，通过 `ros2_control` 驱动 Go2 的 12 个腿部关节，并集成
Velodyne、IMU、RealSense、LIO-SAM 建图和 NDT 重定位。
当前执行 `colcon list` 可识别 21 个 ROS 2 包。

当前已增加仅面向 Gazebo 的动作控制包 `go2_behaviors`，可执行打招呼、点头、
伸展、趴下、挥爪和简单舞蹈，并使用 `stand` 从保持趴下恢复。动作复用现有
CHAMP、`ros2_control` 和标准 `FollowJointTrajectory` 接口，不等同于真机
Unitree Sport API。

当前已固定引入 Unitree 官方 `unitree_ros2 v0.3.0` 的 `unitree_go`、
`unitree_api`，并增加 `go2_unitree_sim_bridge`。它让所列 Sport API 上层程序在
Gazebo 与真机间复用官方消息和话题，但不承诺真机固件行为等价。

焊死腿关节、通过 planar-move 滑行的旧简化工作空间 `go2_ws/` 已于
2026-08-06 删除；其专用环境脚本 `scripts/setup_go2_ws.bash` 同时删除。后续
功能开发统一基于 `simdog/`，不再维护两套机器人实现。

统一入口：

```bash
cd /home/hao/ROS/Go2_Bilibili_zhao-main
bash scripts/install_dependencies.sh
bash scripts/install_gpu_dependencies.sh
bash scripts/build_workspaces.sh
source scripts/setup_simdog.bash
```

## 2026-08-09 Unitree SDK2/ROS 2 兼容桥 v1

### 阶段目标与调研

- 保留 Gazebo Classic、CHAMP、LIO-SAM、NDT 和传感器链，在唯一 `simdog`
  工作空间增加 Unitree 接口级兼容层，不实现 `/lowcmd`。
- 固定复用 Unitree 官方 `unitree_ros2 v0.3.0` 中 BSD-3-Clause 的
  `unitree_go`、`unitree_api`，来源提交为
  `66ae09858245ac3d2231c0cc209e36a88f8d7d03`。消息定义保持官方版本；仅在
  `package.xml` 补齐 CMake 已使用的 `rosidl_generator_dds_idl` 构建依赖并规范
  占位元数据；消息字段、类型和顺序不变，仅整理尾部空白。
- 参考官方 `unitree_mujoco` 的 CycloneDDS Domain 1/loopback 仿真和
  Domain 0/真机网卡切换方式，但没有引入 MuJoCo。来源：
  <https://github.com/unitreerobotics/unitree_ros2/tree/v0.3.0>、
  <https://github.com/unitreerobotics/unitree_mujoco>。

### 实际操作

- 新增 `go2_unitree_sim_bridge`，发布 `/sportmodestate`、
  `/lf/sportmodestate`、`/lowstate`、`/lf/lowstate` 和
  `/api/sport/response`；订阅 `/api/sport/request`。
- 支持 API 1002、1003、1004、1005、1006、1007、1008、1009、1010、
  1016、1017、1022。Move 按 CHAMP 上限持续发布并由 StopMove 清零；Euler
  拒绝非有限值和超限姿态；站立、坐卧、恢复与表演动作调用现有行为服务。
- `go2_behaviors` 增加串行服务端、`stop` 取消入口和状态话题。主 Gazebo 启动
  文件默认以 `unitree_bridge:=true` 启动服务端与桥接，可显式关闭。
- Sport 状态使用真值里程计、IMU 和足端 TF；LowState 前 12 个电机与四足接触
  统一为 `FR、FL、RR、RL`。发布定时器使用稳态时钟，消息时间戳使用真值里程计
  的仿真时间；输入超时会停止 Move、设置错误状态并节流告警。
- 依赖脚本增加 CycloneDDS RMW 和 DDS IDL 生成器；增加仿真
  `setup_unitree_sim.bash` 与真机 `setup_unitree_real.bash` 环境入口。

### 构建与验证

会话初因 `sudo` 交互密码不可用，曾临时从 ROS 软件源解包
`rosidl_generator_dds_idl`、CycloneDDS RMW 及其运行依赖完成构建和 loopback
验证；随后已于 2026-08-09 通过 `bash scripts/install_dependencies.sh` 正式
安装到系统，不再依赖 `/tmp` 临时环境：

```text
ros-humble-rmw-cyclonedds-cpp 1.3.4
ros-humble-cyclonedds 0.10.5
ros-humble-rosidl-generator-dds-idl 0.8.1
```

已完成的验证：

```bash
test "$(cd simdog && colcon list | wc -l)" -eq 21
colcon build --symlink-install --packages-select \
    unitree_api unitree_go go2_behaviors go2_unitree_sim_bridge go2_config
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
    simdog/src/go2_unitree_sim_bridge/test  # 本机 anyio 插件加载 _pytest.scope 失败，必须禁用自动加载
python3 -m py_compile <新增及修改的 Python 文件>
bash -n scripts/setup_unitree_sim.bash scripts/setup_unitree_real.bash simdog/start.sh
```

在独立 `ROS_DOMAIN_ID=173`、无界面 Gazebo 中确认 `/clock`、里程计、IMU、
关节、Velodyne 和控制器正常；四个 Unitree 状态话题实测约为
`49.8/10.0/100.9/10.0 Hz`，时间戳、IMU、电机、足端 TF 和接触均有有效数据。
Move 限幅、StopMove、Euler、非法参数、不支持 API、`noreply`、忙、取消及原动作
失败响应均符合预期；StandUp、StandDown、RecoveryStand、Sit、RiseSit、Hello、
Stretch、Dance1 均通过兼容桥执行成功。
另外在临时解包的 `rmw_cyclonedds_cpp` 环境中，以 `lo`、Domain 176 完成独立
进程间 `/api/sport/request`/`response` 请求响应，确认 CycloneDDS 环境入口有效。
Domain 177 下 LIO-SAM 的四个核心节点也均能使用 CycloneDDS 启动并等待数据；
12 秒冒烟结束时由 `timeout` 发送 SIGINT。该项没有输入传感器数据或正式地图。
完整 Gazebo 初次切换 CycloneDDS 时暴露了默认自动 participant 索引不足的问题，
两个环境脚本现均设置 `MaxAutoParticipantIndex=100`。修复后在 Domain 179、独立
Gazebo master 下，两个控制器、行为服务端和桥接全部启动，四个状态话题实测
`50.2/10.0/99.9/10.0 Hz`，且时间戳、IMU、电机和足端 TF 均为有效非零数据；
桥接与服务端也能随 launch 干净退出。

### 运行边界与下一步

- `range_obstacle`、BMS、温度、序列号、CRC 和没有可信来源的力矩保持零；足底
  接触只是 `0/1`，不是真实力。输入首次就绪前不发布状态。
- 不模拟 `/lowcmd`、无线遥控、真机内部平衡、安全固件或高风险翻转动作。
- Unitree Move 活动期间禁止键盘遥控；动作轨迹不可下发真机。
- 本阶段没有真机硬件和正式 PCD 地图验证。后续真机测试必须先核对网卡、Domain、
  安全场地和急停措施，仿真桥不得与真机 DDS 图同时启动。

## 2026-08-07 Go2 仿真动作

### 阶段目标

- 在没有真机的情况下实现打招呼、点头、伸展、趴下、挥爪和简单舞蹈。
- 复用成熟控制接口，避免重新实现关节控制器和自定义 ROS 消息。
- 防止 CHAMP 步态与动作轨迹同时写入同一个控制器。

### 调研与选择

- 对照 Unitree SDK2 Go2 `SportClient`，确认官方提供 `Hello`、`Stretch` 等
  真机 RPC 接口，但实际运动策略位于真机固件中，不能直接复用于 Gazebo。
- 对照 CHAMP 上游保留现有四足站姿、关节顺序和步态控制，不复制其控制算法。
- 采用 ROS 2 `joint_trajectory_controller` 已有的标准
  `control_msgs/action/FollowJointTrajectory` 接口，没有新建自定义消息或
  重复实现轨迹控制器。
- 参考来源：
  - CHAMP：<https://github.com/chvmp/champ>
  - Unitree SDK2：
    <https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/sport/sport_client.hpp>
  - ROS 2 控制器文档：
    <https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>
- CHAMP 保持 BSD-3-Clause，ROS 2 控制组件为 Apache-2.0；新增
  `go2_behaviors` 使用 BSD-3-Clause，没有复制或引入第三方源码。

### 实际操作

- 新增 `simdog/src/go2_behaviors` ament Python 包和统一命令：

  ```bash
  ros2 run go2_behaviors go2_behavior \
      {hello,nod,stretch,lie,wave,dance,stand}
  ```

- `champ_base` 新增
  `/quadruped_controller_node/set_behavior_mode` 标准 `SetBool` 服务。进入
  行为模式后停止 CHAMP 关节轨迹并忽略速度、机身姿态输入；普通动作结束后自动
  恢复 CHAMP。
- `lie` 完成后保持动作控制权和趴下姿态，`stand` 平滑恢复站姿后再恢复 CHAMP。
- 动作节点启动时读取实际 `/joint_states` 作为轨迹起点，内置关键帧完整性、有限值
  和 URDF 关节限位检查，并用进程锁拒绝并行动作；动作结束后读取
  `/odom/ground_truth` 检查机身高度、横滚和俯仰，防止把“关节目标成功”误判为
  “机器人动作成功”。
- 为 Gazebo 关节速度抖动设置明确的动作目标容差和结束时间容差，避免标准动作服务
  因默认停止速度阈值长期不返回。
- 更新依赖脚本、根目录 README、`simdog/README.md` 和动作包 README；
  `AGENTS.md`、`CLAUDE.md` 同步加入新包与真机边界。
- 后续文档同步将 `文档/simdog_packages_guide.md` 从过期的 16 包结构更新为
  实测的 18 包结构，补充动作控制链、命令表、调参和故障排查，并校正 NDT 输出、
  D435 启用状态、Git 状态以及失效的相对链接。

### 构建与闭环验证

构建和静态检查通过：

```bash
python3 -m py_compile \
    simdog/src/go2_behaviors/go2_behaviors/behavior_runner.py
colcon build --symlink-install --packages-select champ_base go2_behaviors
cmp -s AGENTS.md CLAUDE.md
test "$(cd simdog && colcon list | wc -l)" -eq 18
git diff --check
```

最终使用独立 `ROS_DOMAIN_ID=109` 和
`GAZEBO_MASTER_URI=http://127.0.0.1:11409` 启动无界面完整四足 Gazebo，
两个 `ros2_control` 控制器均为 `active`。六个动作依次返回成功，且动作结束
后的 `/odom/ground_truth` 结果为：

```text
hello：  z=0.214 m，机身水平
nod：    z=0.215 m，机身水平
stretch：z=0.215 m，机身水平
wave：   z=0.215 m，机身水平
dance：  z=0.216 m，机身水平，存在预期的偏航变化
lie：    z=0.094 m，保持趴下
stand：  z=0.216 m，恢复站立
```

`lie` 保持期间监听
`/joint_group_effort_controller/joint_trajectory` 两秒没有收到 CHAMP 消息，
确认控制权仲裁有效。并行启动第二个动作会以退出码 `2` 拒绝，首个动作继续正常
完成。

初版 `wave` 抬腿幅度过大，实测造成侧翻；最终版本缩短右前腿并降低横摆幅度后，
在两次独立仿真中均保持约 `0.215 m` 站立高度。不能只凭动作服务返回成功判断
动力学动作有效，因此运行入口现在会自动执行上述动力学姿态检查。最终还在独立
`ROS_DOMAIN_ID=110` 中复测 `hello -> lie -> stand`，运行时检查分别得到
`z=0.188/0.092/0.191 m`，横滚和俯仰均在 `0.016 rad` 以内。

### 运行边界

- 这些轨迹只针对当前 Go2 Gazebo 模型和控制器参数，不可直接下发真机。
- 动作必须串行执行；执行期间不要遥控。异常中断或保持趴下后先执行 `stand`。
- 仿真动作是确定性关键帧，不具备真机运动策略的在线平衡、落脚规划和安全保护。

## 2026-08-06 TF 所有权统一

### 阶段目标

- 修复 `map`、`odom` 与机器人模型分成两棵 TF 树的问题。
- 保证 CHAMP/EKF、LIO-SAM、NDT 和 `robot_state_publisher` 对每条 TF
  只有一个所有者。
- 删除 LIO-SAM 硬编码的孤立 `lidar_link`，统一使用 URDF 中的 `velodyne`
  外参。

### 方案调研与选择

- 参考 `robot_localization` 官方状态估计约定：局部估计器在
  `world_frame=odom` 时发布 `odom -> base_link`；全局定位器应发布
  `map -> odom`，并依赖已有的 `odom -> base_link`，避免一个子坐标系拥有
  多个父坐标系。
- 对照 CHAMP 上游已有的双 EKF 结构，恢复项目内被注释的
  `footprint_to_odom_ekf`，没有重新实现一套里程计节点。
- 对照 LIO-SAM 上游数据流保留其 IMU 预积分与建图里程计，只修改本项目的 TF
  边界和实际 Go2 外参适配。
- 参考来源：
  - `robot_localization` 官方文档：
    <https://docs.ros.org/en/kinetic/api/robot_localization/html/state_estimation_nodes.html>
  - CHAMP：<https://github.com/chvmp/champ>
  - LIO-SAM：<https://github.com/TixiaoShan/LIO-SAM>
- 本地 CHAMP 与 LIO-SAM 均保留原有 BSD 许可文件；LIO-SAM 的
  `package.xml` 许可证字段由错误的 `TODO` 校正为 `BSD-3-Clause`。本次没有
  引入新依赖，ROS 2 Humble 与 Gazebo Classic 11 适配风险较低。

### 实际操作

- 恢复 `champ_bringup` 中的 `footprint_to_odom_ekf`，由其唯一动态发布
  `odom -> base_footprint` 和 `/odom`；`publish_odom_tf` 参数现在实际生效。
- 保留 `base_to_footprint_ekf` 发布
  `base_footprint -> base_link`，`robot_state_publisher` 继续负责
  `base_link` 以下的关节和传感器。
- LIO-SAM 内部融合里程计从 `/odom` 隔离到
  `/lio_sam/imu/odometry`，映射结果使用 `map` 坐标系，并默认关闭其
  `odometryFrame -> base_footprint` TF。
- 删除 LIO-SAM 启动文件中的静态 `map -> odom`，改为通过
  `map -> velodyne` 与现有 `odom -> velodyne` 反算并动态发布
  `map -> odom`。
- 删除运行路径中的硬编码 `lidar_link`；LIO-SAM 参考模型改为复用
  `go2_description/xacro/robot_VLP.xacro`，RViz 配置同步移除孤立节点。
- `lidar.launch.py` 增加 `publish_map_to_odom`：
  - 建图模式为 `true`，LIO-SAM 拥有 `map -> odom`；
  - NDT 模式为 `false`，LIO-SAM 不注册 `/tf` 发布端，由 NDT 唯一拥有
    `map -> odom`。
- NDT 将配准结果按实际点云帧解释为 `map -> velodyne`，通过 URDF 外参换算
  `map -> base_link`；`/initialpose` 的 `map -> base_link` 初值也会反向
  换算为 NDT 所需的雷达初值。
- `simdog/start.sh` 根据 `GlobalMap.pcd` 是否存在自动选择上述 TF 所有者。
- LIO-SAM 退出自动保存改为关闭，地图只通过 `simdog/save_Map.sh` 显式保存，
  避免普通节点退出覆盖正式地图。
- 根目录 `README.md`、`simdog/README.md`、`AGENTS.md` 与 `CLAUDE.md` 已同步
  记录建图/重定位两种 TF 所有权模式；规则镜像通过 `cmp -s` 检查。
- 补充根目录 `README.md` 与 `simdog/README.md` 的完整启动指南：按首次建图和
  已有地图重定位分开列出每个终端的环境加载、启动、保存、停止和检查命令，并明确
  LIO-SAM 与 NDT 的 `map -> odom` 所有权切换及 RViz2 的 Fixed Frame 处理方式。

### 验证结果

受影响 C++ 目标和包已成功编译：

```bash
colcon build --symlink-install --packages-select \
    champ_bringup champ_base lio_sam ndt_relocalization go2_description
cmake --build simdog/build/lio_sam \
    --target lio_sam_imuPreintegration lio_sam_mapOptimization -- -j2
cmake --build simdog/build/ndt_relocalization \
    --target ndt_relocalization_node -- -j2
```

使用独立 `ROS_DOMAIN_ID=86` 和独立 Gazebo master 启动无界面仿真后确认：

```text
/odom 发布者：footprint_to_odom_ekf（1 个）
LIO 内部里程计：/lio_sam/imu/odometry（1 个）
TF 动态主链：map -> odom -> base_footprint -> base_link
雷达外参：base_link -> velodyne，平移 [0.2, 0.0, 0.118]
动态 TF 中不存在 lidar_link
```

`publish_map_to_odom:=true` 时，`lio_sam_mapOptimization` 是
`map -> odom` 发布者，完整 `map -> base_link` 可连续查询；
`publish_map_to_odom:=false` 时参数实测为 `False`，LIO-SAM 不注册 `/tf`
发布端，动态 TF 中不再出现 `map -> odom`，为 NDT 留出唯一所有权。

执行 `bash scripts/verify_gpu_runtime.sh` 通过：隔离域为 `132`，RTX 4060
启用三级 CUDA NDT，NDT 计算进程使用约 `98 MiB`，采样峰值 GPU `7%`、总显存
`192 MiB`，并成功发布有限值 `/ndt_pose`。

随后使用独立 `ROS_DOMAIN_ID=93` 和
`GAZEBO_MASTER_URI=http://127.0.0.1:11393` 完成真实数据闭环验证，未接入或
终止用户原有 ROS 图：

1. 启动完整四足 Gazebo 后，两个 `ros2_control` 控制器均为 `active`，
   `/odom` 只有 `footprint_to_odom_ekf` 一个发布者。
2. 以 `publish_map_to_odom:=true` 启动 LIO-SAM，成功发布
   `/lio_sam/mapping/odometry` 和动态 `map -> odom`。
3. 调用 `/lio_sam/save_map` 成功生成临时 `GlobalMap.pcd`，文件为
   `43210` 字节、包含 `2689` 个点。
4. 重启 LIO-SAM 并设为 `publish_map_to_odom:=false` 后，参数实测为
   `False`，其节点不再注册 `/tf` 发布端。
5. NDT 使用刚生成的地图和 `registration_backend:=cuda` 直接完成重定位，
   过滤后地图为 `2424` 点，配准分数约 `0.005–0.008`，`/ndt_pose`
   发布频率约 `8 Hz`。
6. TF 审计共收集到 `34` 条关系，完整主链为
   `map -> odom -> base_footprint -> base_link -> velodyne`；
   `base_link -> velodyne` 平移为 `[0.2, 0.0, 0.118]`，不存在
   `lidar_link`。NDT 是全局 TF 发布者，其 CUDA 进程使用约 `98 MiB`
   计算显存。

验证完成后已停止隔离域中的 NDT、LIO-SAM 和 Gazebo，并删除仅用于测试的临时
地图目录。

### 运行边界

- `map -> odom` 只能由 LIO-SAM 或 NDT 二选一发布；手工同时启动时必须给
  LIO-SAM 传入 `publish_map_to_odom:=false`。
- 第一次隔离测试在关闭 LIO-SAM 时触发了原配置的自动保存，在
  `~/go2_maps/latest` 生成了 `trajectory.pcd`、`transformations.pcd`、
  `cloudCorner.pcd`、`cloudSurf.pcd` 和 `cloudGlobal.pcd`；没有生成或覆盖
  NDT 使用的 `GlobalMap.pcd`。随后已关闭自动保存。
- Gazebo 退出时仍可能出现既有的 `contact_sensor` Boost mutex 断言，不影响
  运行期间 TF 验证。
- Gazebo 加载四组关节硬件时会报告既有的 `hold_joints` 参数重复声明，但本次
  闭环测试中两个控制器仍正常进入 `active`，未阻断步态、传感器或 TF。

## 2026-08-06 文档中文化与包结构梳理

### 阶段目标

- 将 `simdog/src/` 下所有项目级英文 README 翻译为规范中文。
- 创建 `simdog/src/` 包结构汇总中文参考文档。
- 回答 CHAMP 核心包在代码中的分布位置。

### 实际操作

- 翻译以下 10 个项目级 README 为中文：
  - `unitree-go2-ros2/README.md` — Go2 主配置指南
  - `LIO-SAM/README.md` — 激光惯性 SLAM 完整文档
  - `champ/README.md` — CHAMP 四足控制器框架
  - `fast_gicp/README.md` — 快速点云配准库
  - `pointcloud_to_laserscan/README.md` — 点云/激光转换
  - `realsense_ros_gazebo/README.md` — RealSense 仿真
  - `ndt_omp_ros2/README.md` — OpenMP NDT 算法
  - `champ_teleop/README.md` — 遥控节点
  - `robots/README.md` — 机器人配置库
  - `champ/include/champ/README.md` — CHAMP 核心库引用
- 第三方库文档（Eigen、Sophus、nvbio 等 39 个）保持原文不变。
- 创建 [`文档/simdog_packages_guide.md`](文档/simdog_packages_guide.md)，详细记录：
  - 16 个 ROS 2 包的完整路径、功能说明和数据流关系
  - CHAMP 12 个核心包在代码中的具体分布位置
  - Velodyne VLP-16 传感器规格与项目中的代码体现
  - 步态参数说明和常用启动命令速查
- 所有译文保持与原文档相同的 Markdown 结构、代码块和图片链接。

### 验证

```bash
# 确认所有翻译文件存在且非空
for f in \
  simdog/src/unitree-go2-ros2/README.md \
  simdog/src/LIO-SAM/README.md \
  simdog/src/pointcloud_to_laserscan/README.md \
  simdog/src/realsense_ros_gazebo/README.md \
  simdog/src/ndt_omp_ros2/README.md \
  simdog/src/fast_gicp/README.md \
  simdog/src/unitree-go2-ros2/champ/README.md \
  simdog/src/unitree-go2-ros2/champ_teleop/README.md \
  simdog/src/unitree-go2-ros2/robots/README.md \
  simdog/src/unitree-go2-ros2/champ/champ/include/champ/README.md; do
  wc -l "$f"
done
```

本阶段仅修改文档，未改动任何算法源码、启动文件或构建配置。

## 2026-08-06 工作空间收敛与文档校正

### 阶段目标

- 删除没有腿部动力学的简化机器人，只保留完整四足仿真。
- 让 `AGENTS.md` 和 `CLAUDE.md` 内容完全一致并要求同步维护。
- 核对真实 GPU，清理错误硬件信息和失效路径。

### 实际操作

- 删除 `go2_ws/` 的源码、构建产物、安装产物和日志。
- 删除 `scripts/setup_go2_ws.bash`。
- 将 `scripts/build_workspaces.sh` 改为只构建 `simdog`，保留 CUDA 12.8
  检测、`sm_89` 构建和 OpenMP 回退。
- 简化 `scripts/setup_simdog.bash`，移除已删除工作空间的叠加检查。
- 为 `scripts/verify_gpu_runtime.sh` 增加独立 `ROS_DOMAIN_ID`，避免用户正在运行
  的 Gazebo `/clock` 和 TF 与验证节点的系统时间互相干扰。
- 将 `AGENTS.md` 与 `CLAUDE.md` 改为完全相同的中文规则，加入同步维护与
  `cmp -s AGENTS.md CLAUDE.md` 检查要求。
- 更新根目录 README、GPU 文档、`simdog/README.md` 和 VS Code 配置；移除
  旧用户 `/home/luhao/...` 与已删除 `go2_ws` 的路径。

### 硬件核对

2026-08-06 使用 `nvidia-smi` 实际读取：

```text
NVIDIA GeForce RTX 4060 Laptop GPU, 595.84, 8188 MiB, compute 8.9
```

因此本机不是 RTX 5070。当前 CUDA 工具链为 12.8，`fast_gicp` 构建架构为
`sm_89`。

### 本阶段验证

```bash
nvidia-smi --query-gpu=name,driver_version,memory.total,compute_cap \
    --format=csv,noheader
cmp -s AGENTS.md CLAUDE.md
bash -n scripts/build_workspaces.sh
bash -n scripts/setup_simdog.bash
bash -n simdog/start.sh
bash -n simdog/save_Map.sh
colcon list --base-paths simdog/src
bash scripts/verify_gpu_runtime.sh
```

本阶段只调整工作空间结构、脚本和文档，没有修改 CHAMP、Gazebo、LIO-SAM 或
NDT 算法源码。静态检查识别到 `simdog` 的 17 个主要 ROS 2 包。

首次 GPU 验证受到当时正在运行的 Gazebo 仿真时间和 TF 干扰，未收到
`/ndt_pose`；确认不是 GPU 或 CUDA 后端错误后，为验证脚本增加独立 ROS 域。
再次运行时使用 `ROS_DOMAIN_ID=179`，RTX 4060 成功启用三级 CUDA NDT，节点
使用约 `98 MiB` 计算显存，GPU 采样峰值 `46%`，并发布有限值 `/ndt_pose`，
端到端验证通过。用户原有 Gazebo 进程未被终止。

完整 Gazebo 四足场景运行能力沿用 2026-08-05 的验证基线；删除简化工作空间后
没有重新启动第二套 Gazebo 场景，因为当前用户已有完整四足 Gazebo 正在运行。

## 2026-08-05 环境配置与完整四足验证基线

### 依赖与构建

- 安装 Gazebo Classic 11、`gazebo_ros_pkgs`、`gazebo_ros2_control`、
  Velodyne、`robot_localization`、ROS 2 控制器、PCL、Nav2、SLAM Toolbox、
  `teleop_twist_keyboard`、`diagnostic_updater` 和 `ecl_threads` 等依赖。
- 使用 BorgLab 4.1 PPA 安装 `libgtsam-dev` 和
  `libgtsam-unstable-dev`。`rosdep` 可能报告 `ros-humble-gtsam` 未满足，
  原因是项目主动使用 GTSAM 4.1，不应与 ROS 仓库 GTSAM 4.2 混装。
- `simdog` 17 个主要包完成干净构建。
- 修复 `gui:=false` 仍启动 `gzclient` 的问题。
- 为 LIO-SAM 增加 `rviz` 参数，地图保存默认目录改为
  `~/go2_maps/latest`。
- NDT 地图路径、输入话题、配准后端和 GPU 设备均已参数化。

### GPU NDT

- CUDA 12.8 编译器、Runtime 和 cuBLAS 开发库来自 NVIDIA 官方 Ubuntu
  22.04 软件源，没有替换现有显卡驱动。
- `fast_gicp` 基于上游提交
  `0e7ec1441c99f7be453db2ea216d5de029387417`，保留 BSD 3-Clause
  `LICENSE`。
- 针对 CUDA 12.8 修复新版 Thrust 与旧式前置声明的命名空间冲突。
- GPU 库按 RTX 4060 计算能力 8.9 编译；`cuobjdump` 已确认包含 `sm_89`
  cubin，NDT 节点链接 `libcudart.so.12` 和 `libfast_vgicp_cuda.so`。
- `registration_backend` 支持 `cuda` 和 `omp`；CUDA 设备或 GPU 构建不可用
  时自动回退 OpenMP。

### 完整四足运行结果

- `gzserver`、CHAMP、`ros2_control` 和 EKF 正常运行，所需控制器 active。
- `/velodyne_points` 约 10 Hz、`/imu/data` 约 200 Hz、
  `/joint_states` 约 250 Hz，`/odom/local` 有输出。
- 向 `/cmd_vel` 发布约 1 秒的 `0.15 m/s` 前进速度后，机器人前移约
  `0.20 m`。
- LIO-SAM 去畸变点云、特征点云、配准点云和里程计均有输出；
  `/lio_sam/save_map` 成功生成 PCD 文件。
- NDT 成功读取验证地图、发布 `/global_map`，提供 `/initialpose` 后持续输出
  `/ndt_pose`。
- CUDA D2D-NDT 对约 1.7 万点测试点云单次约 `4.36 ms`，100 次约
  `417.72 ms`；GPU SM 峰值 `74%`、显存约 `212 MiB`。同一测试的 CPU
  多线程 VGICP 单次约 `17.52 ms`。
- `scripts/verify_gpu_runtime.sh` 端到端验证通过：NDT 出现在 NVIDIA 计算进程
  中并发布有限值 `/ndt_pose`，脚本退出后没有残留测试进程。

验证命令：

```bash
bash scripts/build_workspaces.sh
bash scripts/verify_gpu_runtime.sh

source scripts/setup_simdog.bash
ros2 launch go2_config gazebo_velodyne.launch.py gui:=false rviz:=false
ros2 topic hz /velodyne_points
ros2 topic hz /imu/data
ros2 topic echo --once /odom/local
ros2 control list_controllers
```

## 已知限制与下一步

- 当前只有 NDT 点云配准使用 CUDA。LIO-SAM 图优化、点云预处理、Gazebo 物理
  和 CHAMP 仍主要使用 CPU；Gazebo/RViz2 只使用 GPU 进行 OpenGL 渲染。
- Gazebo Classic GUI 可能受驱动与 OGRE 兼容性影响，推荐
  `gui:=false` 配合 RViz2。
- Gazebo 退出时 `contact_sensor` 可能触发 Boost mutex 断言；该现象发生在
  停止阶段，不影响运行数据。
- 部分 `ros2_control` 实例可能提示 `hold_joints` 重复声明，但所需控制器保持
  active。
- EKF 会提示当前 IMU 配置包含消息不提供的速度项；后续精细状态估计需要继续
  校准协方差和融合字段。
- LIO-SAM 当前关闭回环检测。
- Nav2 和 SLAM Toolbox 依赖已安装，但完整自主导航参数尚未完成调优。
- 正式使用 NDT 前，应在目标场景完成稳定建图并保存 `GlobalMap.pcd`，再校准
  NDT 分辨率和初始位姿。

## 维护规则

每完成一个可验证阶段、修改启动入口、关键参数或硬件基线后，更新本文件中的日期、
操作、结果、验证命令、限制和下一步；合并或替换过期信息，不创建版本副本。
