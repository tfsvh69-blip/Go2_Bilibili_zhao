# champ_teleop

CHAMP 四足机器人遥控节点。本包是 [teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py) 的修改版本。

软件已修改为支持控制机器人的全身姿态（横滚、俯仰、偏航）。

## 使用方法

    roslaunch champ_teleop teleop.launch

搭配 Logitech F710 手柄的可选参数：

    roslaunch champ_teleop teleop.launch joy:=true

* 请确保手柄顶部的开关处于 'X' 模式。

## 手柄控制机器人

左摇杆：
- 上/下 — 线速度 X
- 左/右 — 角速度 Z
- L1 + 左/右 — 线速度 Y

右摇杆：
- 上/下 — 机身俯仰角（Pitch）
- 左/右 — 机身横滚角（Roll）
- R1 + 左/右 — 机身偏航角（Yaw）
- R2 + 上/下 — 机身高度（Z）
