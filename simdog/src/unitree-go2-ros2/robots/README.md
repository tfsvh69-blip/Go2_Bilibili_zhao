## 机器人配置库（Zoo）

本仓库包含由 CHAMP [配置助手](https://github.com/chvmp/champ_setup_assistant) 生成的各种四足机器人的配置包。

## 安装

需要先在机器上安装 [CHAMP](https://github.com/chvmp/champ) 才能使这些机器人行走。

### URDF

在 _[install_descriptions](https://github.com/chvmp/robots/blob/master/install_descriptions)_ 文件中列出了所需机器人 URDF 文件的下载资源。也可以运行：

    ./install_descriptions

一次性下载所有 URDF。

### 快速开始指南

预配置的机器人位于 [configs](https://github.com/chvmp/robots/tree/master/configs) 目录中。每个配置包内都有自动生成的 README，包含运行演示的说明。

请注意，虽然 README 中可能包含如何在 Gazebo 中运行的说明，但只有以下预配置机器人可以在 Gazebo 中正常工作：

- Anybotics 的 ANYmal B
- Anybotics 的 ANYmal C
- 波士顿动力的 Spot
- 宇树科技的 Aliengo
- 宇树科技的 Go1
- 宇树科技的 A1
- MIT Mini Cheetah
- OpenDog V2
- Open Quadruped
- Stochlite
- MangDang 的 Mini Pupper
- Stanford Pupper

## 致谢

本仓库中的 URDF 文件 fork/修改/链接自以下项目：

- [Anybotics 的 ANYmal B](https://github.com/ANYbotics/anymal_b_simple_description)
- [Anybotics 的 ANYmal C](https://github.com/ANYbotics/anymal_c_simple_description)
- [波士顿动力的 Little Dog](https://github.com/RobotLocomotion/LittleDog)
- [波士顿动力的 Spot](https://github.com/clearpathrobotics/spot_ros)
- [Dream Walker](https://github.com/Ohaginia/dream_walker)
- [GoogleAI ROBEL D'Kitty](https://github.com/google-research/robel-scenes)
- [MIT Mini Cheetah](https://github.com/chvmp/mini-cheetah-gazebo-urdf)
- [OpenDog V2](https://github.com/XRobots/openDogV2)
- [Open Quadruped](https://github.com/moribots/spot_mini_mini)
- [SpotMicroAI](https://gitlab.com/custom_robots/spotmicroai)
- [宇树科技的 Aliengo](https://github.com/unitreerobotics/unitree_ros)
- [宇树科技的 Go1](https://github.com/unitreerobotics/unitree_ros)
- [宇树科技的 A1](https://github.com/unitreerobotics/unitree_ros)
- [Stochlab 的 Stochlite](https://stochlab.github.io/)
- [MangDang 的 Mini Pupper](https://github.com/mangdangroboticsclub/mini_pupper_ros)
- [Stanford Pupper](https://stanfordstudentrobotics.org/pupper)
