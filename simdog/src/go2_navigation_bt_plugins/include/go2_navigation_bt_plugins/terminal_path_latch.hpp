// Copyright (c) 2026 hao
// SPDX-License-Identifier: BSD-3-Clause

#ifndef GO2_NAVIGATION_BT_PLUGINS__TERMINAL_PATH_LATCH_HPP_
#define GO2_NAVIGATION_BT_PLUGINS__TERMINAL_PATH_LATCH_HPP_

#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace go2_navigation_bt_plugins
{

/**
 * @brief 进入目标 XY 容差后锁存当前终点路径，停止重规划。
 *
 * Nav2 Humble 的 RotationShimController 会在每次 setPlan() 时重置内部
 * PositionGoalChecker。该装饰节点使用实时 TF 判断机器人是否真正进入目标
 * XY 容差；一旦进入，就保持锁存直到新目标、halt 或 recovery，避免定位边界
 * 抖动让控制器在“追位置”和“终点定向”之间反复切换。
 */
class TerminalPathLatch : public BT::DecoratorNode
{
public:
  TerminalPathLatch(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus executeTick() override;
  BT::NodeStatus tick() override;
  void halt() override;

  /** @brief 测试和诊断用：返回当前 action 是否已进入终点锁存。 */
  bool isLatched() const;

private:
  static bool sameGoal(
    const geometry_msgs::msg::PoseStamped & lhs,
    const geometry_msgs::msg::PoseStamped & rhs);

  static bool pathMatchesGoal(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & goal,
    double path_goal_xy_tolerance,
    double path_goal_yaw_tolerance);

  bool canLatch(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & goal,
    double xy_tolerance,
    double path_goal_xy_tolerance,
    double path_goal_yaw_tolerance,
    const std::string & global_frame,
    const std::string & robot_base_frame,
    double transform_tolerance);

  bool robotWithinTolerance(
    const geometry_msgs::msg::PoseStamped & goal,
    double xy_tolerance,
    const std::string & global_frame,
    const std::string & robot_base_frame,
    double transform_tolerance);

  void resetLatch();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::optional<geometry_msgs::msg::PoseStamped> active_goal_;
  bool latched_{false};
  bool fresh_path_for_goal_{false};
};

}  // namespace go2_navigation_bt_plugins

#endif  // GO2_NAVIGATION_BT_PLUGINS__TERMINAL_PATH_LATCH_HPP_
