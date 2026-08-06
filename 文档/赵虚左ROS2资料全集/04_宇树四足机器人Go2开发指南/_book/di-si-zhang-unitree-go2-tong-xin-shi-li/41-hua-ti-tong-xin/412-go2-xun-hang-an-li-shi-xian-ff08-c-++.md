### 4.1.2 go2 巡航案例实现（C++）

#### 1.节点实现

请先将当前工作空间`src/base/go2_driver/include`目录下的 nlohmann 目录和sport\_model.hpp 文件，复制到当前功能包 include 目录下。

功能包src目录下，新建C++文件go2\_ctrl.cpp，并编辑文件，输入如下内容：

```cpp
/*
    需求：控制机器人以圆周运动方式巡航。
*/
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "nlohmann/json.hpp"
#include "sport_model.hpp"

// 3.自定义节点类；
class Go2Ctrl: public rclcpp::Node{
public:
    Go2Ctrl():Node("go2_ctrl_node"){

      this->declare_parameter<double>("vx",0.0);
      this->declare_parameter<double>("vy",0.0);
      this->declare_parameter<double>("vyaw",0.0);
      this->declare_parameter<int64_t>("sport_api_id",ROBOT_SPORT_API_ID_BALANCESTAND);

      //创建一个ros2 pubilsher
      req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&Go2Ctrl::cruise,this));
    }
private:
    void cruise(){
      unitree_api::msg::Request req; //创建一个运动请求msg
      int64_t id = this->get_parameter("sport_api_id").as_int();
      req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE;

      if (id == ROBOT_SPORT_API_ID_MOVE)
      {
        nlohmann::json js;
        js["x"] = this->get_parameter("vx").as_double();
        js["y"] = this->get_parameter("vy").as_double();
        js["z"] = this->get_parameter("vyaw").as_double();
        req.parameter = js.dump();

        // 或
        // req.parameter = "{\"x\": 0.0, \"y\": 0.0, \"z\": 0.6}";
        // RCLCPP_INFO(this->get_logger(),"req.param = %s", req.parameter.c_str());
      }

      req_puber->publish(req); //发布数据  

    }
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char const *argv[])
{
    // 2.初始化ROS2客户端；
    rclcpp::init(argc,argv);
    // 4.调用spain函数，并传入节点对象指针；
    rclcpp::spin(std::make_shared<Go2Ctrl>());
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
<depend>unitree_go</depend>
<depend>unitree_api</depend>
```

##### 2.CMakeLists.txt {#2cmakeliststxt}

CMakeLists.txt 中添加如下配置：

```
target_include_directories(go2_ctrl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  go2_ctrl
  "rclcpp"
  "unitree_go"
  "unitree_api"
)
install(TARGETS 
  go2_ctrl 
  DESTINATION lib/${PROJECT_NAME})
```

#### 3.编译 {#3编译}

终端中进入当前工作空间，编译功能包：

```
colcon build --packages-select go2_tutorial
```

#### 4.执行 {#4执行}

在当前工作空间下，启动终端，并输入如下指令：

```
. install/setup.bash
ros2 run go2_tutorial go2_ctrl
```

再新建一个终端，启动 rqt：

```
rqt
```

选择菜单栏Plugins下的Configuration，再点击Dynamic Reconfigure，选定当前节点，将`sport_api_id`设置为1008，再设置 `vx`、`vy`、`vyaw`的值，即可控制机器人巡航的速度了。

![](/assets/go2_ctrl.PNG)

