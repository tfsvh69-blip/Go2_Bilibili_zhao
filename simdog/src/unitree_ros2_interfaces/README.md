# Unitree ROS 2 接口快照

本目录固定收录 Unitree 官方 `unitree_ros2` `v0.3.0` 中的 `unitree_go` 和
`unitree_api` ROS 2 接口包，来源提交为
`66ae09858245ac3d2231c0cc209e36a88f8d7d03`：

<https://github.com/unitreerobotics/unitree_ros2/tree/v0.3.0/cyclonedds_ws/src/unitree>

上游许可证为 BSD-3-Clause，完整许可证见同目录 `LICENSE`。本项目只对两个
`package.xml` 补充了上游 `CMakeLists.txt` 实际使用但未声明的
`rosidl_generator_dds_idl` 构建依赖，并把占位描述与许可证字段规范化；消息字段、
类型、顺序和 CMake 生成规则保持不变，仅整理了消息文件的尾部空白。
