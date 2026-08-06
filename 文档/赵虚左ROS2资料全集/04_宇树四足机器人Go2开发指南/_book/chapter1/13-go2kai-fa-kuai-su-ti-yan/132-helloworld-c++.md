### 1.3.2 HelloWorld \(C++\)

#### 1.创建功能包 {#2编辑源文件}

终端下，进入 unitree\_go2\_ws/src/helloworld 目录，调用如下指令创建C++功能包：

```
ros2 pkg create go2_helloworld --build-type ament_cmake --dependencies rclcpp unitree_api
```

#### 2.编辑源文件 {#2编辑源文件}

进入 go2\_helloworld/src 目录，新建 hello.cpp文件，并输入如下内容：

```cpp
/*
    需求：unitree go2 在 ROS2 环境下的第一个小程序。
*/
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"

// 3.自定义节点类；
class Go2Hello: public rclcpp::Node{
public:
    Go2Hello():Node("go2_hello_node"){
      //创建一个ros2 pubilsher
      req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&Go2Hello::hello,this));
    }
private:
    void hello(){
      unitree_api::msg::Request req; //创建一个运动请求msg
      req.header.identity.api_id = 1016;
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
    rclcpp::spin(std::make_shared<Go2Hello>());
    // 5.资源释放。
    rclcpp::shutdown();
    return 0;
}
```

#### 3.编辑配置文件 {#3编辑配置文件}

##### 1.package.xml {#1packagesxml}

在创建功能包时，所依赖的功能包已经自动配置了，配置内容如下：

```
<depend>rclcpp</depend>
<depend>unitree_api</depend>
```

##### 2.CMakeLists.txt {#2cmakeliststxt}

CMakeLists.txt文件需要添加如下内容：

```CMake
add_executable(hello src/hello.cpp)

ament_target_dependencies(
  hello
  "rclcpp"
  "unitree_api"
)

install(TARGETS hello
  DESTINATION lib/${PROJECT_NAME})
```

#### 4.编译 {#4编译}

终端下进入到工作空间，执行如下指令：

```bash
colcon build --packages-select go2_helloworld
```

#### 5.执行 {#5执行}

终端下进入到工作空间，执行如下指令：

```bash
. install/setup.bash
ros2 run go2_helloworld hello
```

程序执行，go2 将执行打招呼的动作。

