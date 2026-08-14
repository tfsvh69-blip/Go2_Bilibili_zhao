cmake_minimum_required(VERSION 3.8)
project(go2_navigation_bt_plugins)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(go2_terminal_path_latch_bt_node SHARED
  src/terminal_path_latch.cpp
)
ament_target_dependencies(go2_terminal_path_latch_bt_node
  behaviortree_cpp_v3
  geometry_msgs
  nav_msgs
  nav2_util
  rclcpp
  tf2_ros
)
target_include_directories(go2_terminal_path_latch_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(go2_terminal_path_latch_bt_node PUBLIC cxx_std_17)
target_compile_definitions(go2_terminal_path_latch_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS go2_terminal_path_latch_bt_node
  EXPORT export_go2_terminal_path_latch_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nav2_behavior_tree REQUIRED)
  ament_add_gtest(test_terminal_path_latch test/test_terminal_path_latch.cpp)
  if(TARGET test_terminal_path_latch)
    target_link_libraries(test_terminal_path_latch go2_terminal_path_latch_bt_node)
    ament_target_dependencies(test_terminal_path_latch
      behaviortree_cpp_v3
      geometry_msgs
      nav_msgs
      nav2_behavior_tree
      rclcpp
      tf2_ros
    )
  endif()
endif()

ament_export_targets(export_go2_terminal_path_latch_bt_node HAS_LIBRARY_TARGET)
ament_export_dependencies(
  behaviortree_cpp_v3
  geometry_msgs
  nav_msgs
  nav2_util
  rclcpp
  tf2_ros
)

ament_package()
