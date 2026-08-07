# Go2 仿真动作

本包为当前 Go2 Gazebo 模型提供打招呼、点头、伸展、趴下、挥爪和简单舞蹈。
动作通过 ROS 2 标准
`control_msgs/action/FollowJointTrajectory` 接口发送给现有
`joint_trajectory_controller`，没有新建自定义消息或重复实现关节控制器。

## 使用

先启动完整四足 Gazebo，并在新终端加载工作空间：

```bash
source scripts/setup_simdog.bash
```

每次执行一个动作：

```bash
ros2 run go2_behaviors go2_behavior hello
ros2 run go2_behaviors go2_behavior nod
ros2 run go2_behaviors go2_behavior stretch
ros2 run go2_behaviors go2_behavior lie
ros2 run go2_behaviors go2_behavior wave
ros2 run go2_behaviors go2_behavior dance
```

`lie` 完成后会保持趴下并暂停 CHAMP。使用以下命令安全恢复站立和步态控制：

```bash
ros2 run go2_behaviors go2_behavior stand
```

动作只能串行执行。执行期间不要关闭 Gazebo 或切换
`joint_group_effort_controller`；异常中断时先执行 `stand`。

## 控制权

动作开始前，程序调用
`/quadruped_controller_node/set_behavior_mode` 暂停 CHAMP 的关节轨迹输出，并
忽略这段时间收到的速度和机身姿态命令。动作完成后再把关节控制权交还给 CHAMP，
因此不会出现 CHAMP 与动作节点同时向控制器写入轨迹的情况。动作结束还会通过
`/odom/ground_truth` 检查机身高度、横滚和俯仰；关节目标完成但机器人侧翻时，
命令会明确报告动力学失败。

## 复用与适用边界

- 复用 CHAMP 现有站姿、Go2 URDF 关节顺序和 `ros2_control` 控制链。
- 复用 ROS 2 `joint_trajectory_controller` 的标准动作接口，不包含复制的控制器
  源码。
- 参考 Unitree SDK2 的 Go2 `SportClient` 动作命名，但没有复制或模拟官方闭源
  运动策略。这里的轨迹只针对当前 Gazebo 模型，不可直接下发真机。
- CHAMP 使用 BSD-3-Clause，ROS 2 控制组件使用 Apache-2.0；本包使用
  BSD-3-Clause，没有引入新的第三方源码。

参考：

- CHAMP：<https://github.com/chvmp/champ>
- Unitree SDK2 Go2 SportClient：
  <https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/sport/sport_client.hpp>
- ROS 2 `joint_trajectory_controller`：
  <https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>
