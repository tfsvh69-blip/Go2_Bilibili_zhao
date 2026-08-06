### 4.3.3 go2 导航案例实现（C++）

#### 1.服务端节点实现

功能包src目录下，新建C++文件 go2\_nav\_server.cpp，并编辑文件，输入如下内容：

```cpp
/*
    需求：简单的模拟导航功能，向机器人发送一个前进N米的请求，机器人就会以 0.1m/s的速度前进，
         当与目标点的距离小于0.05m时，机器人就会停止运动，返回机器人的停止坐标，并且在此过程中，
         会连续反馈机器人与目标点之间的剩余距离。
    流程：
        1.包含头文件；
        2.初始化ROS2客户端；
        3.自定义节点类；
          3-1.创建参数服务客户端，连接到速度发布节点；
          3-2.创建订阅方，订阅机器人的里程计以获取机器人坐标；
          3-3.创建动作服务端，解析客户端的相关并生成响应。
        4.调用spin函数，并传入节点对象指针；
        5.资源释放。
*/
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "go2_tutorial_inter/action/nav.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sport_model.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
// 3.自定义节点类；
class NavServer: public rclcpp::Node{
public:
    NavServer():Node("nav_server_node_cpp"){
        
        this->declare_parameter("vx",0.1);
        this->declare_parameter("error",0.2);
        
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
        // 3-2.创建订阅方，订阅机器人的里程计以获取机器人坐标；
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    "odom",
                    10,
                    std::bind(&NavServer::on_timer,this,_1));
        // 3-3.创建动作服务端，解析客户端的相关并生成响应。
        nav_action_server_ = rclcpp_action::create_server<go2_tutorial_inter::action::Nav>(
                    this,
                    "nav",
                    std::bind(&NavServer::handle_goal,this,_1,_2),
                    std::bind(&NavServer::handle_cancel,this,_1),
                    std::bind(&NavServer::handle_accepted,this,_1)
        );
    }
private:
    rclcpp::AsyncParametersClient::SharedPtr paramClient;
    geometry_msgs::msg::Point current_point;
    geometry_msgs::msg::Point start_point; // 每次导航时，机器人起点坐标
    double error; // 终点允许误差
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp_action::Server<go2_tutorial_inter::action::Nav>::SharedPtr nav_action_server_;

    void on_timer(const nav_msgs::msg::Odometry & odom){
        current_point = odom.pose.pose.position;
    }

    // 解析动作客户端发送的请求；
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & goal_uuid, std::shared_ptr<const go2_tutorial_inter::action::Nav::Goal> goal){
        (void)goal_uuid;
        float goal_distance = goal->goal;
        if (goal_distance > 0.0)
        {
          RCLCPP_INFO(this->get_logger(),"请求前进%.2f米", goal_distance);
          start_point = current_point;
        } else {
          RCLCPP_INFO(this->get_logger(),"只许进，不许退!");
          return rclcpp_action::GoalResponse::REJECT;
        }
        paramClient->set_parameters({
            this->get_parameter("vx"),
            rclcpp::Parameter("sport_api_id",ROBOT_SPORT_API_ID_MOVE)
            });
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // 处理动作客户端发送的取消请求；
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<go2_tutorial_inter::action::Nav>> goal_handle){
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(),"任务取消中....!");
        balance_stand();
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void balance_stand(){
        paramClient->set_parameters({
            rclcpp::Parameter("vx",0.0),
            rclcpp::Parameter("sport_api_id",ROBOT_SPORT_API_ID_BALANCESTAND)
            });
    }
    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<go2_tutorial_inter::action::Nav>> goal_handle){
        RCLCPP_INFO(this->get_logger(),"开始执行任务......");
        // 获取目标距离
        float goal = goal_handle->get_goal()->goal;
        // 连续反馈
        auto feedback = std::make_shared<go2_tutorial_inter::action::Nav::Feedback>();
        // 最终结果
        auto result = std::make_shared<go2_tutorial_inter::action::Nav::Result>();

        // 设置连续反馈
        rclcpp::Rate rate(1.0);
        while(rclcpp::ok()){
          // 检查任务是否被取消；
          if(goal_handle->is_canceling()){
            result->point = current_point;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "任务取消");
            // start_point = current_point;
            balance_stand();
            return;
          }   

          double distance = sqrt(pow(current_point.x - start_point.x,2) + pow(current_point.y - start_point.y,2));
          feedback->distance = goal - distance;
          goal_handle->publish_feedback(feedback);

          if (goal - distance <= this->get_parameter("error").as_double())
          {
            break;
          }
          
          rate.sleep();
        }
        // 设置最终结果
        if (rclcpp::ok()) {
          result->point = current_point;
          goal_handle->succeed(result);
          balance_stand();
          RCLCPP_INFO(this->get_logger(), "任务完成！");
        }
    }
    // 创建新线程处理请求；
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<go2_tutorial_inter::action::Nav>> goal_handle){
        std::thread{std::bind(&NavServer::execute,this,_1),goal_handle}.detach();
    }

};
int main(int argc, char const *argv[])
{
    // 2.初始化ROS2客户端；
    rclcpp::init(argc,argv);
    // 4.调用spain函数，并传入节点对象指针；
    rclcpp::spin(std::make_shared<NavServer>());
    // 5.资源释放。
    rclcpp::shutdown();
    return 0;
}
```

#### 2.客户端节点实现 {#2编辑配置文件}

功能包src目录下，新建C++文件go2\_nav\_client.cpp，并编辑文件，输入如下内容：

```cpp
/*
   需求：向导航动作服务端发送目标点数据，并处理服务端的响应数据。
   步骤：
       1.包含头文件；
       2.初始化 ROS2 客户端；
       3.定义节点类；
            3-1.创建动作客户端；
            3-2.发送请求数据，并处理服务端响应；
            3-3.处理目标响应；
            3-4.处理响应的连续反馈；
            3-5.处理最终响应。
       4.调用spin函数，并传入节点对象指针；
       5.释放资源。
*/
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "go2_tutorial_inter/action/nav.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

// 3.定义节点类；
class NavClient: public rclcpp::Node{
public:
    NavClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    :Node("exe_nav_action_client",options){
        // 3-1.创建动作客户端；
        nav_client = rclcpp_action::create_client<go2_tutorial_inter::action::Nav>(this,"nav");
    }
    // 3-2.发送请求数据，并处理服务端响应；
    void send_goal(float x){
        // 连接动作服务端，如果超时（5s），那么直接退出。
        if (!nav_client->wait_for_action_server(5s))
        {
            RCLCPP_ERROR(this->get_logger(),"服务连接失败!");
            return;
        }
        // 组织请求数据
        auto goal_msg = go2_tutorial_inter::action::Nav::Goal();
        goal_msg.goal = x;

        rclcpp_action::Client<go2_tutorial_inter::action::Nav>::SendGoalOptions options;
        options.goal_response_callback = std::bind(&NavClient::goal_response_callback, this, _1);
        options.feedback_callback = std::bind(&NavClient::feedback_callback, this, _1, _2);
        options.result_callback = std::bind(&NavClient::result_callback, this, _1);
        // 发送
        nav_client->async_send_goal(goal_msg,options);
        // 判断是否关闭终端
    }
    ~NavClient() {
        nav_client->async_cancel_all_goals();
    }
private:
    rclcpp_action::Client<go2_tutorial_inter::action::Nav>::SharedPtr nav_client;

    // 3-3.处理目标响应；
    void goal_response_callback(rclcpp_action::ClientGoalHandle<go2_tutorial_inter::action::Nav>::SharedPtr goal_handle){
        if(!goal_handle){
            RCLCPP_ERROR(this->get_logger(),"目标请求被服务器拒绝");
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(),"目标请求被接收!");
            // std::thread(&NavClient::cancel_goals,this,goal_handle).detach();
        }
    }
    // void cancel_goals(rclcpp_action::ClientGoalHandle<go2_tutorial_inter::action::Nav>::SharedPtr goal_handle){
    //     while (rclcpp::ok()){}
    //     // nav_client->async_cancel_all_goals();
    //     nav_client->async_cancel_goal(goal_handle);
    // }
    // 3-4.处理响应的连续反馈；
    void feedback_callback(rclcpp_action::ClientGoalHandle<go2_tutorial_inter::action::Nav>::SharedPtr goal_handle, 
        const std::shared_ptr<const go2_tutorial_inter::action::Nav::Feedback> feedback){
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(),"距离目标点还有 %.2f 米。",feedback->distance);

    }
    // 3-5.处理最终响应。
    void result_callback(const rclcpp_action::ClientGoalHandle<go2_tutorial_inter::action::Nav>::WrappedResult & result){
        switch (result.code){
        case rclcpp_action::ResultCode::SUCCEEDED :
            RCLCPP_INFO(this->get_logger(),"go2最终坐标:(%.2f,%.2f)",result.result->point.x,result.result->point.y);
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(),"任务被取消");
            break;      
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(),"任务被中止");
            break;   
        default:
            RCLCPP_ERROR(this->get_logger(),"未知异常");
            break;
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char const *argv[])
{
    if (argc != 2)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"请传入要前进的距离数据");
        return 1;
    }
    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc,argv);
    // 4.调用spin函数，并传入节点对象指针；
    auto client = std::make_shared<NavClient>();
    // 发送目标点
    client->send_goal(atof(argv[1]));
    rclcpp::spin(client);
    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}
```

#### 3.编写launch和params文件 {#2编辑配置文件}

在 luanch 目录下，新建名为 go2\_nav\_server.launch.py 的launch文件，并输入如下内容：

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
            executable="go2_nav_server",
            parameters=[os.path.join(go2_tutorial_pkg,"params","go2_nav_server.yaml")]
        )
    ])
```

该 launch 文件加载了机器人驱动、运动控制模块，还包含了导航服务节点，该节点还加载了yaml文件，yaml文件可以在params目录下创建，将文件名命名为 go2\_nav\_server.yaml，输入如下内容：

```yaml
/**:
  ros__parameters:
    use_sim_time: false
    vx: 0.1
    error: 0.1
```

通过该文件可以配置导航速度和容忍误差。

#### 4.编辑配置文件 {#2编辑配置文件}

##### 1.package.xml {#1packagesxml}

在 package.xml 中添加如下依赖：

```
<depend>rclcpp_action</depend>
```

##### 2.CMakeLists.txt {#2cmakeliststxt}

CMakeLists.txt 中添加如下配置：

```
add_executable(go2_nav_server src/go2_nav_server.cpp)
add_executable(go2_nav_client src/go2_nav_client.cpp)

ament_target_dependencies(
  go2_nav_server
  "rclcpp"
  "unitree_go"
  "unitree_api"
  "nav_msgs"
  "go2_tutorial_inter"
  "rclcpp_action"
  "geometry_msgs"
)

ament_target_dependencies(
  go2_nav_client
  "rclcpp"
  "unitree_go"
  "unitree_api"
  "nav_msgs"
  "go2_tutorial_inter"
  "rclcpp_action"
  "geometry_msgs"
)

install(TARGETS 
  go2_nav_server
  go2_nav_client
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
ros2 launch go2_tutorial go2_nav_server.launch.py
```

指令执行后，其机器人驱动启动，且导航服务就绪。

在当前工作空间下，再启动终端，输入如下指令：

```
. install/setup.bash
ros2 run go2_tutorial go2_nav_client 0.5
```

机器人开始导航，并连续反馈剩余距离，到达目标点后会结束导航，并返回机器人的停止坐标。

