### 3.2.2 桥接功能实现（C++）

#### 1.节点实现

请先将`~/unitree_ros2/example/src/include`目录下的 nlohmann 目录，复制到当前功能包 include 目录下。

在 include 目录下，在新建  sport\_model.hpp 文件，并输入如下内容：

```cpp
#ifndef _SPORT_MODEL_
#define _SPORT_MODEL_
#include <iostream>

#pragma pack(1)

// 1001: 阻尼控制
const int32_t ROBOT_SPORT_API_ID_DAMP = 1001;

// 1002: 平衡站立，控制机器人进入平衡站立状态，保持直立和稳定
const int32_t ROBOT_SPORT_API_ID_BALANCESTAND = 1002;

// 1003: 停止运动，停止机器人的所有运动，使其立即静止
const int32_t ROBOT_SPORT_API_ID_STOPMOVE = 1003;

// 1004: 站立，控制机器人从其他姿势（如坐下或躺下）恢复到站立姿势
const int32_t ROBOT_SPORT_API_ID_STANDUP = 1004;

// 1005: 站立下降，控制机器人从站立姿势转变为其他姿势（如坐下或躺下）
const int32_t ROBOT_SPORT_API_ID_STANDDOWN = 1005;

// 1006: 恢复站立，控制机器人在失去平衡后恢复到站立姿势
const int32_t ROBOT_SPORT_API_ID_RECOVERYSTAND = 1006;

// 1007: 欧拉角控制，用于调整机器人的姿态（如俯仰、横滚、偏航）
const int32_t ROBOT_SPORT_API_ID_EULER = 1007;

// 1008: 移动，控制机器人进行移动，可能是直线移动或转向
const int32_t ROBOT_SPORT_API_ID_MOVE = 1008;

// 1009: 坐下，控制机器人进入坐下姿势
const int32_t ROBOT_SPORT_API_ID_SIT = 1009;

// 1010: 从坐下恢复站立，控制机器人从坐下姿势恢复到站立姿势
const int32_t ROBOT_SPORT_API_ID_RISESIT = 1010;

// 1011: 切换步态，切换机器人的步态（如行走、跑步、爬行等）
const int32_t ROBOT_SPORT_API_ID_SWITCHGAIT = 1011;

// 1012: 触发，可能用于触发某个特定的动作或事件
const int32_t ROBOT_SPORT_API_ID_TRIGGER = 1012;

// 1013: 身体高度调整，调整机器人的身体高度
const int32_t ROBOT_SPORT_API_ID_BODYHEIGHT = 1013;

// 1014: 脚部抬起高度调整，调整机器人脚部的抬起高度
const int32_t ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT = 1014;

// 1015: 速度级别调整，调整机器人的运动速度级别
const int32_t ROBOT_SPORT_API_ID_SPEEDLEVEL = 1015;

// 1016: 打招呼，控制机器人执行“打招呼”的动作
const int32_t ROBOT_SPORT_API_ID_HELLO = 1016;

// 1017: 伸展，控制机器人执行“伸展”动作
const int32_t ROBOT_SPORT_API_ID_STRETCH = 1017;

// 1018: 轨迹跟随，控制机器人按照预定的轨迹进行移动
const int32_t ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW = 1018;

// 1019: 连续步态，控制机器人进行连续的步态运动
const int32_t ROBOT_SPORT_API_ID_CONTINUOUSGAIT = 1019;

// 1020: 内容，可能用于控制机器人显示或执行某种内容（如播放视频或音频）
const int32_t ROBOT_SPORT_API_ID_CONTENT = 1020;

// 1021: 打滚，控制机器人执行“打滚”或“翻滚”动作
const int32_t ROBOT_SPORT_API_ID_WALLOW = 1021;

// 1022: 舞蹈1，控制机器人执行第一种舞蹈动作
const int32_t ROBOT_SPORT_API_ID_DANCE1 = 1022;

// 1023: 舞蹈2，控制机器人执行第二种舞蹈动作
const int32_t ROBOT_SPORT_API_ID_DANCE2 = 1023;

// 1024: 获取身体高度，获取机器人当前的身体高度
const int32_t ROBOT_SPORT_API_ID_GETBODYHEIGHT = 1024;

// 1025: 获取脚部抬起高度，获取机器人当前脚部的抬起高度
const int32_t ROBOT_SPORT_API_ID_GETFOOTRAISEHEIGHT = 1025;

// 1026: 获取速度级别，获取机器人当前的运动速度级别
const int32_t ROBOT_SPORT_API_ID_GETSPEEDLEVEL = 1026;

// 1027: 切换操纵杆，切换操纵杆的控制模式或功能
const int32_t ROBOT_SPORT_API_ID_SWITCHJOYSTICK = 1027;

// 1028: 姿态，控制机器人进入特定的姿态或姿势
const int32_t ROBOT_SPORT_API_ID_POSE = 1028;

// 1029: 刮擦，控制机器人执行“刮擦”动作
const int32_t ROBOT_SPORT_API_ID_SCRAPE = 1029;

// 1030: 前空翻，控制机器人执行前空翻动作
const int32_t ROBOT_SPORT_API_ID_FRONTFLIP = 1030;

// 1031: 前跳，控制机器人执行前跳动作
const int32_t ROBOT_SPORT_API_ID_FRONTJUMP = 1031;

// 1032: 前扑，控制机器人执行前扑动作
const int32_t ROBOT_SPORT_API_ID_FRONTPOUNCE = 1032;
#endif
```

该文件定义了一些机器人运动所需的协议码。

功能包src目录下，新建C++文件twist\_bridge.cpp，并编辑文件，输入如下内容：

```cpp
/*
    需求：将ROS2中常用的速度指令（geometry_msgs::msg::Twist），
         转换成四组机器人的速度指令（unitree_api::msg::Request）。
    流程：
        1.包含头文件；
        2.初始化ROS2客户端；
        3.自定义节点类；
          3-1.创建四组机器人速度指令发布对象；
          3-2.创建ROS2速度指令订阅对象；
          3-3.在订阅对象的回调函数中将Twsit转换成Request并发布。
        4.调用spin函数，并传入节点对象指针；
        5.资源释放。
*/
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sport_model.hpp"
#include "nlohmann/json.hpp"

using namespace std::placeholders;

// 3.自定义节点类；
class Twsit2Request: public rclcpp::Node{
public:
    Twsit2Request():Node("twsit2request_node"){
        RCLCPP_INFO(this->get_logger(),"convert geometry_msgs::msg::Twist to unitree_api::msg::Request.");
        // 3-1.创建四组机器人速度指令发布对象；
        req_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request",10);
        // 3-2.创建ROS2速度指令订阅对象；
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>
                    ("cmd_vel",10,std::bind(&Twsit2Request::twist_to_request,this,_1));
        // 3-3.在订阅对象的回调函数中将Twsit转换成Request并发布。
    }
private:
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    void twist_to_request(const geometry_msgs::msg::Twist::SharedPtr twist){
        unitree_api::msg::Request request;
        auto api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
        // 转换 (只需要x、y的线速度和z的角速度)
        double x = twist->linear.x;
        double y = twist->linear.y;
        double th = twist->angular.z;
        if (x != 0 || y !=0 || th != 0)
        {
            api_id = ROBOT_SPORT_API_ID_MOVE;

            // 设置线速度与角速度
            nlohmann::json js;
            js["x"] = x;
            js["y"] = y;
            js["z"] = th;
            request.parameter = js.dump();
            RCLCPP_INFO(this->get_logger(),"current speed: %s",request.parameter.c_str());
        }

        request.header.identity.api_id = api_id;
        req_pub_->publish(request);
    }
};

int main(int argc, char const *argv[])
{
    // 2.初始化ROS2客户端；
    rclcpp::init(argc,argv);
    // 4.调用spain函数，并传入节点对象指针；
    rclcpp::spin(std::make_shared<Twsit2Request>());
    // 5.资源释放。
    rclcpp::shutdown();
    return 0;
}
```

#### 2.编辑配置文件 {#2编辑配置文件}

##### 1.package.xml {#1packagesxml}

在创建功能包时，所依赖的功能包已经自动配置了，配置内容如下：

```
<depend>rclcpp</depend>
<depend>geometry_msgs</depend>
<depend>unitree_api</depend>
```

##### 2.CMakeLists.txt {#2cmakeliststxt}

CMakeLists.txt 中添加如下配置：

```
add_executable(twist_bridge src/twist_bridge.cpp)

target_include_directories(twist_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  twist_bridge
  "rclcpp"
  "geometry_msgs"
  "unitree_api"
)

install(TARGETS twist_bridge
  DESTINATION lib/${PROJECT_NAME})
```

#### 3.编译 {#3编译}

终端中进入当前工作空间，编译功能包：

```
colcon build --packages-select go2_twist_bridge
```

#### 4.执行 {#4执行}

在当前工作空间下，启动终端，并输入如下指令：

```
. install/setup.bash
ros2 run go2_twist_bridge twist_bridge
```

再新建一个终端，启动 ROS2 中常用的键盘控制节点：

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

至此，就可以通过键盘控制 go2 的运动了。

