cmake_minimum_required(VERSION 3.8)
project(go2_description)

find_package(ament_cmake REQUIRED)

# 安装模型资源与 launch(mesh 通过 package://go2_description/... 解析)
install(DIRECTORY urdf dae meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
