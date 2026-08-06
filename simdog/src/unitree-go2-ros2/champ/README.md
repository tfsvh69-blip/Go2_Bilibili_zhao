# champ [![构建状态](https://travis-ci.org/chvmp/champ.svg?branch=master)](https://travis-ci.org/chvmp/champ) 

CHAMP 四足控制器的 ROS 包集合。

![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/robots.gif)

CHAMP 是一个开源开发框架，用于构建新型四足机器人和开发新的控制算法。其控制框架基于 [*"基于模式调制与阻抗控制的层次化高动态运动控制器：在 MIT Cheetah 机器人上的实现"*](https://dspace.mit.edu/handle/1721.1/85490)。

核心特性：

- 完全自主导航（使用 ROS Navigation Stack）
- 提供 [配置助手](https://github.com/chvmp/champ_setup_assistant) 用于配置新构建的机器人
- 预配置 [URDF](https://github.com/chvmp/robots) 集合，包括 Anymal、MIT Mini Cheetah、波士顿动力 Spot 和 LittleDog
- Gazebo 仿真环境
- 兼容 DIY 四足项目，如 [SpotMicroAI](https://spotmicroai.readthedocs.io/en/latest/) 和 [OpenQuadruped](https://github.com/adham-elarabawy/open-quadruped)
- 演示应用，如 [TOWR](https://github.com/ethz-adrl/towr) 和 [chicken head](https://github.com/chvmp/chicken_head) 姿态稳定
- 轻量级 C++ Header-Only [库](https://github.com/chvmp/libchamp)，可同时在单板计算机和微控制器上运行

已测试环境：

- Ubuntu 16.04 (ROS Kinetic)
- Ubuntu 18.04 (ROS Melodic)
- Ubuntu 22.04 (ROS2 Humble)

ROS2 移植当前状态：

- &check; libchamp 已移植
- &cross; 速度平滑器未移植
- &check; Gazebo 空世界可运行，无力控，仅机器人站立并居中力矩控制器
- &check; RViz 纯演示可运行
- &check; Gazebo + 遥控机器人可运行
- &check; Gazebo + SLAM 演示可运行
- &check; Gazebo + Nav2 集成演示可运行
- &cross; 代码清理和重构未完成
- &cross; 未在实体机器人上测试
- &cross; 配置助手未移植
- &cross; 机器人配置未移植
    - &cross; 机器人描述包未移植到 ROS 2
    - &cross; 机器人 URDF 未移植到 ROS2（未添加 ros2_control 标签）
    - &cross; 机器人参数配置未移植到 ROS2
    - &cross; 机器人启动文件未移植到 ROS2

## 1. 安装

### 1.1 克隆仓库并安装所有依赖：

    sudo apt install -y python3-rosdep
    rosdep update

    cd <你的工作空间>/src
    git clone --recursive https://github.com/chvmp/champ -b ros2
    git clone https://github.com/chvmp/champ_teleop -b ros2
    cd ..
    rosdep install --from-paths src --ignore-src -r -y

如需使用预配置的机器人（如 Anymal、Mini Cheetah、Spot），请按照 [此处的说明](https://github.com/chvmp/robots) 操作。

### 1.2 构建工作空间：

    cd <你的工作空间>
    colcon build
    . <你的工作空间>/install/setup.bash

## 2. 快速开始

无需实体机器人即可运行以下演示。如果你正在搭建实体机器人，可以在第 3 步中了解更多关于配置和运行新机器人的信息。

### 2.1 RViz 行走演示：

#### 2.1.1 运行基础驱动：

    ros2 launch champ_config bringup.launch.py rviz:=true 

#### 2.1.2 运行遥控节点：

    ros2 launch champ_teleop teleop.launch.py 

如需使用 [手柄](https://www.logitechg.com/en-hk/products/gamepads/f710-wireless-gamepad.html)，添加 `joy:=true` 参数。

### 2.2 Gazebo 演示：

#### 2.2.1 启动 Gazebo 环境：
    
    ros2 launch champ_config gazebo.launch.py 

#### 2.2.2 启动 [Nav2](https://navigation.ros.org/) 导航和 [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)：

    ros2 launch champ_config slam.launch.py rviz:=true 

开始建图：

- 点击"2D Nav Goal"
- 在目标位置点击并拖拽

   ![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/slam.gif)

- 保存地图：

      cd <你的工作空间>/src/champ/champ_config/maps
      ros2 run nav2_map_server map_saver_cli -f new_map

之后即可使用 new_map 进行纯导航。

### 2.3 自主导航：

#### 2.3.1 启动 Gazebo 环境： 

    ros2 launch champ_config gazebo.launch.py

#### 2.3.2 启动 [Nav2](https://navigation.ros.org/)：

    ros2 launch champ_config navigate.launch.py rviz:=true

开始导航：

- 点击"2D Nav Goal"
- 在目标位置点击并拖拽

   ![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/navigation.gif)

# 以下内容尚未移植到 ROS2

## 3. 运行你自己的机器人：

有两种在实体机器人上运行 CHAMP 的方式：

Linux 主机方式
- 使用本 ROS 包计算关节角度并发送到硬件接口以控制执行器。可按照 [硬件集成指南](https://github.com/chvmp/champ/wiki/Hardware-Integration) 创建执行器接口。

轻量级方式
- 在 Teensy 系列微控制器上运行 CHAMP 的 [轻量级版本](https://github.com/chvmp/firmware)，直接控制执行器。

### 3.1 生成机器人配置

   - 首先使用 [champ_setup_assistant](https://github.com/chvmp/champ_setup_assistant) 生成配置包。按照 README 中的说明配置你的机器人。生成的包包含：
        - 机器人的 URDF 路径
        - 关节和连杆映射，帮助控制器理解机器人的语义结构
        - 步态参数
        - 硬件驱动
        - 导航参数（move_base、amcl 和 gmapping）
        - 微控制器头文件（步态和轻量级机器人描述），仅适用于使用微控制器运行四足控制器的机器人

     作为参考，你可以查看 [此处](https://github.com/chvmp/robots) 已预配置的机器人集合，其中包括 Anymal、MIT Mini Cheetah、波士顿动力 LittleDog 和 SpotMicroAI 等热门四足机器人。欢迎将这些配置包下载到你的 catkin 工作空间的 `src` 目录中进行尝试。

   - 接下来，构建工作空间以使新生成的包可被发现：

         cd <你的工作空间>
         catkin_make

### 3.2 基础驱动：

运行四足控制器和所有传感器/硬件驱动：

    roslaunch <我的机器人配置包> bringup.launch

可用参数：

  - **rviz** — 同时启动 RViz。默认：false
  - **lite** — 如果使用微控制器运行算法，请始终设为 true。默认：false

使用示例：

查看新配置的机器人：

    roslaunch <我的机器人配置包> bringup.launch rviz:true
    
使用微控制器运行实体机器人：

    roslaunch <我的机器人配置包> bringup.launch lite:=true

### 3.3 创建地图：
运行 gmapping 和 move_base 前必须先启动 3.2 所述的基础驱动。

运行 gmapping 和 move_base：

    roslaunch <我的机器人配置包> slam.launch

打开 RViz 查看地图：

    roscd champ_navigation/rviz 
    rviz -d navigate.rviz

开始建图：

- 点击"2D Nav Goal"
- 在目标位置点击并拖拽

   ![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/slam.gif)

- 保存地图：

      roscd <我的机器人配置包>/maps
      rosrun map_server map_saver

### 3.4 自主导航：

运行 amcl 和 move_base 前必须先启动 3.2 所述的基础驱动。

运行 amcl 和 move_base：

    roslaunch <我的机器人配置包> navigate.launch

打开 RViz 查看地图：

    roscd champ_navigation/rviz 
    rviz -d navigate.rviz

开始导航：

- 点击"2D Nav Goal"
- 在目标位置点击并拖拽

   ![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/navigation.gif)

### 3.5 在 Gazebo 中运行你的机器人

以仿真模式运行 Gazebo 和基础驱动：

    roslaunch <我的机器人配置包> gazebo.launch

* 注意：要使此功能正常工作，URDF 必须兼容 Gazebo 并具备 [ros_control](http://gazebosim.org/tutorials/?tut=ros_control) 能力。控制器已配置好，你只需添加执行器的传动（transmission）定义。还需要正确设置物理参数，如质量、惯量和足部摩擦系数。

   一些获取这些参数的有用资源：

  - 惯性计算 — https://github.com/tu-darmstadt-ros-pkg/hector_models/blob/indigo-devel/hector_xacro_tools/urdf/inertia_tensors.urdf.xacro
  - 转动惯量列表 — https://en.wikipedia.org/wiki/List_of_moments_of_inertia
  - Gazebo 惯性参数 — http://gazebosim.org/tutorials?tut=inertia&cat=build_robot#Overview

也可以参考 [这个 Pull Request](https://github.com/moribots/spot_mini_mini/pull/7) 作为示例。

### 3.6 在 Gazebo 中同时生成多个机器人

运行 Gazebo 和默认仿真世界：

    roslaunch champ_gazebo spawn_world.launch 

也可以通过 `gazebo_world` 参数加载自定义世界文件：

    roslaunch champ_gazebo spawn_world.launch gazebo_world:=<世界文件路径>

生成机器人：

    roslaunch champ_config spawn_robot.launch robot_name:=<唯一机器人名称> world_init_x:=<x坐标> world_init_y:=<y坐标>

* 每个生成的机器人实例必须具有唯一的名称，以防止话题和坐标变换冲突。

## 4. 步态参数调优

机器人的步态配置文件位于 `<我的机器人配置包>/gait/gait.yaml`。

![CHAMP 配置助手](https://raw.githubusercontent.com/chvmp/champ_setup_assistant/master/docs/images/gait_parameters.png)

- **膝关节朝向（Knee Orientation）** — 膝关节弯曲方向。可配置以下朝向模式：`.>>` `.><` `.<<` `.<>`，其中点表示机器人前方。

- **最大线速度 X（Max Linear Velocity X）**（米/秒）— 机器人前后移动的最大速度。

- **最大线速度 Y（Max Linear Velocity Y）**（米/秒）— 机器人横向移动的最大速度。

- **最大角速度 Z（Max Angular Velocity Z）**（弧度/秒）— 机器人旋转的最大速度。

- **支撑相时长（Stance Duration）**（秒）— 行走时每条腿着地的时长。如果不确定，可设为默认值 0.25。支撑相时长越大，足端离参考点的位移越大。

- **摆动相抬腿高度（Leg Swing Height）**（米）— 摆动相期间的轨迹高度。

- **支撑相压腿深度（Leg Stance Height）**（米）— 支撑相期间的轨迹深度。

- **机器人行走高度（Robot Walking Height）**（米）— 行走时髋关节到地面的距离。注意：该参数设置过高可能导致机器人不稳定。

- **质心 X 偏移（CoM X Translation）**（米）— 沿 X 轴移动参考点。当质心不在机器人中心（前髋到后髋之间）时，用于补偿重量分布。例如，如果机器人后部较重，则设为负值将参考点后移。

- **里程计比例因子（Odometry Scaler）** — 作为推算定位速度的乘数，用于补偿开环系统的里程计误差。通常取值范围为 1.0 到 1.20。
