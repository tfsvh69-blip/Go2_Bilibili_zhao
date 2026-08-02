cmake_minimum_required(VERSION 3.8)
project(go2_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch worlds rviz config scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
