### 4.2.3 go2 巡航启停案例实现（C++）

#### 1.服务端节点实现

功能包src目录下，新建C++文件go2\_cruising\_service.cpp，并编辑文件，输入如下内容：

```cpp
/*
    需求：巡航服务端，当客户端发送请求时，如果请求数据是1那么开始巡航，如果是0那么就结束巡航，
         并且无论是开始巡航还是结束巡航，都需要返回机器人当时的坐标。
    流程：
        1.包含头文件；
        2.初始化ROS2客户端；
        3.自定义节点类；
          3-1.创建参数服务客户端，连接到速度发布节点；
          3-2.创建订阅方，订阅机器人的里程计以获取机器人坐标。
          3-3.创建服务端，处理客户端请求，如果提交的是1,那么通过参数客户端设置有效的角速度、线速度数据，
              如果是0,那么通过参数客户端将速度指令置0。
        4.调用spin函数，并传入节点对象指针；
        5.资源释放。
*/
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "go2_tutorial_inter/srv/cruising.hpp"
#include "sport_model.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
// 3.自定义节点类；
class CruServer: public rclcpp::Node{
public:
    CruServer():Node("cru_server_node"){
        this->declare_parameter("vx",0.0);
        this->declare_parameter("vy",0.0);
        this->declare_parameter("vyaw",0.5);
        // 3-1.创建参数服务客户端，连接到速度发布节点；
        paramClient = std::make_shared<rclcpp::AsyncParametersClient>(this,"go2_ctrl_node");
        // 等待服务连接
        while (!paramClient->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
              return;
            }  
            RCLCPP_INFO(this->get_logger(),"服务未连接");
        }
        //
        RCLCPP_INFO(this->get_logger(),"已经连接成功速度发送节点的参数服务，可以设置线速度和角速度了");
        // 3-2.创建订阅方，订阅机器人的里程计以获取机器人坐标。
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("odom",10,std::bind(&CruServer::on_timer,this,_1));
        // 3-3.创建服务端，处理客户端请求，如果提交的是1,那么通过参数客户端设置有效的角速度、线速度数据，
        //     如果是0,那么通过参数客户端将速度指令置0。
        service = this->create_service<go2_tutorial_inter::srv::Cruising>("cruising",std::bind(&CruServer::cb,this,_1,_2));
    }
private:
    // rclcpp::SyncParametersClient::SharedPtr paramClient;
    rclcpp::AsyncParametersClient::SharedPtr paramClient;
    rclcpp::Service<go2_tutorial_inter::srv::Cruising>::SharedPtr service;
    geometry_msgs::msg::Point current_point;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    void cb(const go2_tutorial_inter::srv::Cruising::Request::SharedPtr req,
            const go2_tutorial_inter::srv::Cruising::Response::SharedPtr res){
        int flag = req->flag;
        int64_t id = ROBOT_SPORT_API_ID_BALANCESTAND;
        // 判断提交的数据
        if(flag != 0){ // 开始
            RCLCPP_INFO(this->get_logger(),"开始巡航......");
            id = ROBOT_SPORT_API_ID_MOVE;
        } else { // 结束
            RCLCPP_INFO(this->get_logger(),"终止巡航......");
        }
        paramClient->set_parameters({
                this->get_parameter("vx"),
                this->get_parameter("vy"),
                this->get_parameter("vyaw"),
                rclcpp::Parameter("sport_api_id",id)
            });
        res->point = current_point;
    }

    void on_timer(const nav_msgs::msg::Odometry & odom){
        current_point = odom.pose.pose.position;
    }

};
int main(int argc, char const *argv[])
{
    // 2.初始化ROS2客户端；
    rclcpp::init(argc,argv);
    // 4.调用spain函数，并传入节点对象指针；
    rclcpp::spin(std::make_shared<CruServer>());
    // 5.资源释放。
    rclcpp::shutdown();
    return 0;
}
```

#### 2.客户端节点实现 {#2编辑配置文件}

功能包src目录下，新建C++文件go2\_cruising\_client.cpp，并编辑文件，输入如下内容：

```cpp
/*
    需求：向巡航服务端发送请求数据，如果发送的是非0数据，那么就开始巡航，否则就终止巡航，
         不论何种请求，服务端响应的数据是机器人的坐标，客户端还需要解析结果并输出在终端。
    流程：
        1.包含头文件；
        2.初始化ROS2客户端；
        3.自定义节点类；
          3-1.创建客户端；
          3-2.连接服务端；
          3-3.发送请求。
        4.创建自定义类对象，并调用连接服务以及请求发送请求的函数；
        5.处理响应结果；
        6.资源释放。
*/
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "go2_tutorial_inter/srv/cruising.hpp"

using namespace std::chrono_literals;
// 3.自定义节点类；
class CruClient: public rclcpp::Node{
public:
    CruClient():Node("cru_client_node"){
        client = this->create_client<go2_tutorial_inter::srv::Cruising>("cruising");
        RCLCPP_INFO(this->get_logger(),"客户端创建，等待连接服务端！");
    }
    bool connect_server(){
      while (!client->wait_for_service(1s))
      {
        if (!rclcpp::ok())
        {
          return false;
        }

        RCLCPP_INFO(this->get_logger(),"服务连接中，请稍候...");
      }
      return true;
    }
    rclcpp::Client<go2_tutorial_inter::srv::Cruising>::FutureAndRequestId send_request(int32_t flag){
      auto request = std::make_shared<go2_tutorial_inter::srv::Cruising::Request>();
      request->flag = flag;
      return client->async_send_request(request);
    }
private:
    rclcpp::Client<go2_tutorial_inter::srv::Cruising>::SharedPtr client;
};

int main(int argc, char const *argv[])
{
    // 处理通过终端提交的数据
    if (argc != 2){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"请提交一个整型数据！");
      return 1;
    }
    // 2.初始化ROS2客户端；
    rclcpp::init(argc,argv);

    // 4.创建自定义类对象，并调用连接服务以及请求发送请求的函数；    
    auto client = std::make_shared<CruClient>();
    // 连接服务端
    bool flag = client->connect_server();
    if (!flag)
    {
      RCLCPP_INFO(client->get_logger(),"服务连接失败!");
      return 0;
    }
    auto response = client->send_request(atoi(argv[1]));

    // 5.处理响应结果；
    if (rclcpp::spin_until_future_complete(client,response) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(client->get_logger(),"请求正常处理");
      auto cru_res = response.get();
      RCLCPP_INFO(client->get_logger(),"响应坐标:(%.3f,%.3f)", cru_res->point.x,cru_res->point.y);

    } else {
      RCLCPP_INFO(client->get_logger(),"请求异常");
    }

    // 6.资源释放。
    rclcpp::shutdown();
    return 0;
}
```

#### 3.编写launch和params文件 {#2编辑配置文件}

在 luanch 目录下，新建名为 go2\_cruising\_service.launch.py 的launch文件，并输入如下内容：

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
        # 运动控制
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=[os.path.join(go2_tutorial_pkg,"launch","go2_ctrl.launch.py")]
            )
        ),
        Node(
            package="go2_tutorial",
            executable="go2_cruising_service",
            parameters=[os.path.join(go2_tutorial_pkg,"params","go2_cruising_service.yaml")]
        )
    ])
```

该 launch 文件加载了机器人驱动、运动控制模块，还包含了巡航服务节点，该节点还加载了yaml文件，yaml文件可以在params目录下创建，将文件名命名为 go2\_cruising\_service.yaml，输入如下内容：

```yaml
/**:
  ros__parameters:
    use_sim_time: false
    vx: 0.1
    vy: 0.0
    vyaw: 0.5
```

通过该文件可以配置巡航速度数据。

#### 4.编辑配置文件 {#2编辑配置文件}

##### 1.package.xml {#1packagesxml}

在 package.xml 中添加如下依赖：

```
<depend>geometry_msgs</depend>
<depend>go2_tutorial_inter</depend>
```

##### 2.CMakeLists.txt {#2cmakeliststxt}

CMakeLists.txt 中添加如下配置：

```
add_executable(go2_cruising_service src/go2_cruising_service.cpp)
add_executable(go2_cruising_client src/go2_cruising_client.cpp)

ament_target_dependencies(
  go2_cruising_service
  "rclcpp"
  "unitree_go"
  "unitree_api"
  "nav_msgs"
  "go2_tutorial_inter"
  "geometry_msgs"
)

ament_target_dependencies(
  go2_cruising_client
  "rclcpp"
  "unitree_go"
  "unitree_api"
  "nav_msgs"
  "go2_tutorial_inter"
)

install(TARGETS 
  go2_cruising_service 
  go2_cruising_client
  DESTINATION lib/${PROJECT_NAME})
```

#### 5.编译 {#3编译}

终端中进入当前工作空间，编译功能包：

```
colcon build --packages-select go2_tutorial
```

#### 6.执行 {#4执行}

在当前工作空间下，启动终端，输入如下指令：

```
. install/setup.bash
ros2 launch go2_tutorial go2_cruising_service.launch.py
```

指令执行后，其机器人驱动启动，且巡航服务就绪。

在当前工作空间下，再启动终端，输入如下指令：

```
. install/setup.bash
ros2 run go2_tutorial go2_cruising_client 1
```

机器人开始巡航，并返回启动时的坐标。

停止指令如下：

```
ros2 run go2_tutorial go2_cruising_client 0
```

机器人结束巡航，也会返回停止时的坐标。





