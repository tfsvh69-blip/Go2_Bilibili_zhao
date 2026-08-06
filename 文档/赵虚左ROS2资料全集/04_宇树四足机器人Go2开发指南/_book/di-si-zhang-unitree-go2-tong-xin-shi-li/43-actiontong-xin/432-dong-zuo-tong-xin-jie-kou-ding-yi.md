### 4.3.2 动作通信接口定义

自定义服务接口的主要步骤如下：

1. 创建并编辑`.action`文件；
2. 编辑配置文件；
3. 编译；
4. 测试。

#### 1.创建并编辑 .srv 文件 {#1创建并编辑-msg-文件}

功能包 go2\_tutorial\_inter 下新建 action 文件夹，action 文件夹下新建 Nav.action 文件，文件中输入如下内容：

```
float32 goal
---
geometry_msgs/Point point
---
float32 distance
```

goal代表客户端发送的前进距离，point表示服务端最终响应的机器人坐标，distance则指连续反馈的剩余距离。

#### 2.编辑配置文件 {#2编辑配置文件}

##### 1.package.xml文件 {#1packagexml文件}

在package.xml中需要添加一些依赖包，具体内容如下：

```
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>action_msgs</depend>
```

##### 2.CMakeLists.txt文件 {#2cmakeliststxt文件}

为了将`.action`文件转换成对应的C++和Python代码，还需要修改rosidl\_generate\_interfaces函数即可，修改后的内容如下：

```
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Cruising.srv"
  "action/Nav.action"
  DEPENDENCIES geometry_msgs
)
```

#### 3.编译 {#3编译}

终端中进入当前工作空间，编译功能包：

```
colcon build --packages-select go2_tutorial_inter
```

#### 4.测试 {#4测试}

编译完成之后，在工作空间下的install目录下将生成 Nav.srv 文件对应的C++和Python文件，我们也可以在终端下进入工作空间，通过如下命令查看文件定义以及编译是否正常：

```
. install/setup.bash
ros2 interface show go2_tutorial_inter/action/Nav
```

正常情况下，终端将会输出如下内容：

```
float32 goal
---
geometry_msgs/Point point
        float64 x
        float64 y
        float64 z
---
float32 distance
```



