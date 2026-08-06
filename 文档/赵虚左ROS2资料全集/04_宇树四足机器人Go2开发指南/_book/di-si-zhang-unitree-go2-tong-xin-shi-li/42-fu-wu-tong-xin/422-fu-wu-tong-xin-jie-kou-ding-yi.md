### 4.2.2 服务通信接口定义

自定义服务接口的主要步骤如下：

1. 创建并编辑`.srv`文件；
2. 编辑配置文件；
3. 编译；
4. 测试。

#### 1.创建并编辑 .srv 文件 {#1创建并编辑-msg-文件}

功能包 go2\_tutorial\_inter 下新建 srv 文件夹，srv 文件夹下新建 Cruising.srv 文件，文件中输入如下内容：

```
int32 flag
---
geometry_msgs/Point point
```

flag 为客户端发送的标记数据，point 为服务端响应的坐标点信息。

#### 2.编辑配置文件 {#2编辑配置文件}

##### 1.package.xml文件 {#1packagexml文件}

在package.xml中需要添加一些依赖包，具体内容如下：

```
<depend>geometry_msgs</depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

##### 2.CMakeLists.txt文件 {#2cmakeliststxt文件}

为了将`.srv`文件转换成对应的C++和Python代码，还需要在CMakeLists.txt中添加如下配置：

```
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Cruising.srv"
  DEPENDENCIES geometry_msgs
)
```

#### 3.编译 {#3编译}

终端中进入当前工作空间，编译功能包：

```
colcon build --packages-select go2_tutorial_inter
```

#### 4.测试 {#4测试}

编译完成之后，在工作空间下的install目录下将生成 Cruising.srv 文件对应的C++和Python文件，我们也可以在终端下进入工作空间，通过如下命令查看文件定义以及编译是否正常：

```
. install/setup.bash
ros2 interface show go2_tutorial_inter/srv/Cruising
```

正常情况下，终端将会输出如下内容：

```
int32 flag
---
geometry_msgs/Point point
        float64 x
        float64 y
        float64 z
```



