### 4.1.5 go2 位置获取案例实现（C++）

#### 1.节点实现

功能包src目录下，新建C++文件go2\_state.cpp，并编辑文件，输入如下内容：

```cpp
/*
    需求：订阅里程计消息，每当机器狗位移距离超过指定值，即在终端输出机器狗当前坐标。
    流程：
        1.包含头文件；
        2.初始化ROS2客户端；
        3.自定义节点类；
          3-1.创建里程计订阅方；
          3-2.解析里程计数据，并当条件满足时，在终端输出坐标。
        4.调用spin函数，并传入节点对象指针；
        5.资源释放。
*/

// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::placeholders;
// 3.自定义节点类；
class SubOdom: public rclcpp::Node{
public:
    SubOdom():Node("sub_odom_node"){
        last_x = 0.0;
        last_y = 0.0;
        is_first = true;
        this->declare_parameter<double>("distance",0.5);
        // 3-1.创建里程计订阅方；
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("odom",10,std::bind(&SubOdom::on_timer,this,_1));

    }
private:
    // 用于记录上一次输出坐标的变量
    double last_x, last_y;
    bool is_first;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    // 3-2.解析里程计数据，并当条件满足时，在终端输出坐标。
    void on_timer(const nav_msgs::msg::Odometry & odom){
      double x = odom.pose.pose.position.x;
      double y = odom.pose.pose.position.y;

      if (is_first)
      {
        last_x = x;
        last_y = y;
        is_first = false;
        return;
      }


      // 计算当前坐标与上一次输出坐标的直线距离
      double distance_x = x - last_x;
      double distance_y = y - last_y;
      double distance = sqrt(distance_x * distance_x + distance_y * distance_y);

      // 判断是否符合条件
      if(distance >= this->get_parameter("distance").as_double()){
        // 输出
        RCLCPP_INFO(this->get_logger(),"当前坐标(%.2f,%.2f)",x,y);
        // 重赋值
        last_x = x;
        last_y = y;
      }

    }
};

int main(int argc, char const *argv[])
{
    // 2.初始化ROS2客户端；
    rclcpp::init(argc,argv);
    // 4.调用spain函数，并传入节点对象指针；
    rclcpp::spin(std::make_shared<SubOdom>());
    // 5.资源释放。
    rclcpp::shutdown();
    return 0;
}
```

#### 2.编写launch和params文件 {#2编辑配置文件}

在 luanch 目录下，新建名为 go2\_state.launch.py 的launch文件，并输入如下内容：

```py
from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    go2_driver_pkg = get_package_share_directory("go2_driver")
    go2_tutorial_pkg = get_package_share_directory("go2_tutorial")

    return LaunchDescription([
        # 驱动 launch
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=[os.path.join(go2_driver_pkg,"launch","driver.launch.py")]
            )
        ),
        Node(
            package="go2_tutorial",
            executable="go2_state",
            parameters=[os.path.join(go2_tutorial_pkg,"params","go2_state.yaml")]
        )
    ])
```

该 launch 文件加载了机器人驱动且启动了位置获取节点，后者还加载了yaml文件，该文件可以在params目录下创建，将文件名命名为 go2\_state.yaml，并输入如下内容：

```yaml
/**:
  ros__parameters:
    distance: 0.1
    use_sim_time: false
```

通过该文件可以配置参与运算的阈值数据。

#### 3.编辑配置文件 {#2编辑配置文件}

##### 1.package.xml {#1packagesxml}

在 package.xml 中添加如下依赖：

```
<depend>nav_msgs</depend>
```

##### 2.CMakeLists.txt {#2cmakeliststxt}

CMakeLists.txt 中添加如下配置：

```
add_executable(go2_state src/go2_state.cpp)

ament_target_dependencies(
  go2_state
  "rclcpp"
  "nav_msgs"
)

install(TARGETS 
  go2_state 
  DESTINATION lib/${PROJECT_NAME})
```

#### 4.编译 {#3编译}

终端中进入当前工作空间，编译功能包：

```
colcon build --packages-select go2_tutorial
```

#### 5.执行 {#4执行}

在当前工作空间下，启动终端，并输入如下指令：

```
. install/setup.bash
ros2 launch go2_tutorial go2_state.launch.py
```

使用键盘或手柄控制机器人运动，当位移超出指定阈值时，就会输出机器人当时坐标。

