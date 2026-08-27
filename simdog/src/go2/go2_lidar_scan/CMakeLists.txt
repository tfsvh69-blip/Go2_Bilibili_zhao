cmake_minimum_required(VERSION 3.8)
project(go2_lidar_scan)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(level_frame_publisher_component SHARED
  src/level_frame_publisher.cpp
)
ament_target_dependencies(
  level_frame_publisher_component
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_compile_features(level_frame_publisher_component PUBLIC cxx_std_17)
target_include_directories(
  level_frame_publisher_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_nodes(
  level_frame_publisher_component
  "go2_lidar_scan::LevelFramePublisher"
)

add_executable(
  level_frame_publisher
  src/level_frame_publisher_main.cpp
)
target_link_libraries(level_frame_publisher level_frame_publisher_component)
ament_target_dependencies(level_frame_publisher rclcpp)
target_compile_features(level_frame_publisher PUBLIC cxx_std_17)

ament_python_install_package(${PROJECT_NAME})

install(
  TARGETS level_frame_publisher level_frame_publisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(
  PROGRAMS
    scripts/motion_scan_probe
    scripts/scan_diagnostics
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_flake8(${PROJECT_NAME} test scripts)
  # 用户目录中的新版 anyio pytest 插件与 Ubuntu 22.04 自带 pytest 6 不兼容。
  # 本包测试不依赖外部插件，因此关闭自动加载，保证 colcon 使用系统测试基线。
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  ament_add_pytest_test(
    ${PROJECT_NAME}_pytest
    test
    PYTHON_EXECUTABLE /usr/bin/python3
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_xmllint()
endif()

ament_package()
