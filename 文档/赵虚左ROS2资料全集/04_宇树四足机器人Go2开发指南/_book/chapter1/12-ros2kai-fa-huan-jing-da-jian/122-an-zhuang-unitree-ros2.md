### 1.2.2 安装 Unitree ROS2

#### 1.下载 unitree\_ros2

从  github 上下载 unitree\_ros2，终端指令如下：

```
git clone https://github.com/unitreerobotics/unitree_ros2
```

其中 unitree\_ros2 包括 cyclonedds\_ws 和 example 两个子级文件夹:

* **cyclonedds\_ws **文件夹为编译和安装 Go2 机器人 ROS2 msg 的工作空间，在子文件夹`cyclonedds_ws/unitree/unitree_go`  
  和`cyclonedds_ws/unitree/unitree_api`中定义了Go2状态获取和控制相关的 ROS2 msg。

* **example **文件夹为 Go2 机器人 ROS2 下的相关例程。

#### 2. 安装依赖

```
sudo apt install ros-foxy-rmw-cyclonedds-cpp
sudo apt install ros-foxy-rosidl-generator-dds-idl
```

_note_

> 为了方便接口的使用，推荐同时安装[unitree\_sdk2](https://github.com/unitreerobotics/unitree_sdk2)

#### 3. 编译 cyclone-dds

由于 Go2 使用的是**cyclonedds 0.10.2**版本，因此需要先更改 ROS2 的 DDS 实现。

编译 cyclonedds 前请确保在启动终端时**没有**source ros2 相关的环境变量，否则会导致 cyclonedds 编译报错。如果安装 ROS2 时在~/.bashrc中添加了 "`source /opt/ros/humble/setup.bash`"，需要修改 ~/.bashrc 文件将其删除：

```
sudo apt install gedit
sudo gedit ~/.bashrc
```

在弹出的窗口中，注释掉 ROS2 相关的环境变量，例如：

```
# source /opt/ros/humble/setup.bash
```

在终端中执行以下操作编译 cyclone-dds

```
cd ~/unitree_ros2/cyclonedds_ws/src
#克隆cyclonedds仓库
git clone https://github.com/ros2/rmw_cyclonedds -b foxy
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd ..
colcon build --packages-select cyclonedds #编译cyclonedds
```

#### 4. 编译 unitree\_go 和 unitree\_api 功能包

编译好 cyclone-dds 后就需要 Ros2 相关的依赖来完成 Go2 功能包的编译，因此编译前需要先 source ROS2 的环境变量。

```
source /opt/ros/foxy/setup.bash #source ROS2 环境变量
colcon build #编译工作空间下的所有功能包
```



