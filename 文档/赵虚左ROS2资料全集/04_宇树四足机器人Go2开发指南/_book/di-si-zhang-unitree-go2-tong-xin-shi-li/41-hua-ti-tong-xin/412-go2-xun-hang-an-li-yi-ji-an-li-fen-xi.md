### 4.1.1 go2 巡航案例以及案例分析

#### 1.案例简介 {#1案例需求}

**需求：**编写一个 ROS2 节点，通过参数设置线速度和角速度，控制机器人以圆周运动的方式巡航。

#### 2.案例分析 {#2案例分析}

该案例与以前的速度指令发布实现类似，但现在速度可以通过参数进行动态调整。因此，为了支持这一动态调整功能，我们需要在原有基础上添加相关的参数服务。

#### 3.流程简介 {#3流程简介}

主要步骤如下：

1. 编写节点实现；
2. 编辑配置文件；
3. 编译；
4. 执行。

案例我们会采用C++和Python分别实现，二者都遵循上述实现流程。

#### 4.准备工作 {#4准备工作}

终端下进入工作空间的src/tutorial目录，调用如下指令分别创建C++功能包和Python功能包。

```
ros2 pkg create go2_tutorial --build-type ament_cmake --dependencies rclcpp unitree_go unitree_api
ros2 pkg create go2_tutorial_py --build-type ament_python --dependencies rclpy unitree_go unitree_api
```

请再在两个功能包中都新建launch目录和params目录，并做相关配置。

go2\_tutorial 功能包的 CMakeLists.txt 中配置如下：

```
install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}  
)
```

go2\_tutorial\_py 功能包的 setup.py 中 data\_files 添加如下配置：

```
data_files=[
    ......
    ('share/' + package_name + "/launch", glob("launch/*.launch.py")),
    ('share/' + package_name + "/params", glob("params/*.yaml")),
    
],
```

本章后续案例，也将在这两个功能包中实现。

