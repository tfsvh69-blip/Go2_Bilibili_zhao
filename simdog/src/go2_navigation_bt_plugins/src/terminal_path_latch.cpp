// Copyright (c) 2026 hao
// SPDX-License-Identifier: BSD-3-Clause

#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "go2_navigation_bt_plugins/terminal_path_latch.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/utils.h"

namespace go2_navigation_bt_plugins
{

TerminalPathLatch::TerminalPathLatch(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}

BT::PortsList TerminalPathLatch::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "当前传给 FollowPath 的路径"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "当前导航目标"),
    BT::InputPort<double>("xy_tolerance", 0.30, "终点位置锁存半径（m）"),
    BT::InputPort<double>(
      "path_goal_xy_tolerance", 0.075, "路径末端与原始目标的最大位置误差（m）"),
    BT::InputPort<double>(
      "path_goal_yaw_tolerance", 0.01, "路径末端与原始目标的最大航向误差（rad）"),
    BT::InputPort<std::string>("global_frame", "map", "目标与机器人位姿参考坐标系"),
    BT::InputPort<std::string>(
      "robot_base_frame", "base_footprint", "机器人平面基座坐标系"),
    BT::InputPort<double>("transform_tolerance", 0.20, "TF 查询超时（s）"),
  };
}

BT::NodeStatus TerminalPathLatch::executeTick()
{
  // BehaviorTree.CPP v3 的 DecoratorNode::executeTick() 会在本节点返回
  // SUCCESS 时 resetChild()。这里必须使用 TreeNode 的通用执行入口，否则
  // 内层 RateController 每次都会回到 IDLE，1 Hz 限频就会永久失效。
  // halt() 仍由本类覆写，因此 action 取消和 recovery 依然会清理子树。
  return BT::TreeNode::executeTick();
}

BT::NodeStatus TerminalPathLatch::tick()
{
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped goal;
  double xy_tolerance = 0.30;
  double path_goal_xy_tolerance = 0.075;
  double path_goal_yaw_tolerance = 0.01;
  double transform_tolerance = 0.20;
  std::string global_frame = "map";
  std::string robot_base_frame = "base_footprint";
  getInput("path", path);
  if (!getInput("goal", goal)) {
    throw BT::RuntimeError("TerminalPathLatch 未收到有效 goal");
  }
  if (!getInput("xy_tolerance", xy_tolerance) || xy_tolerance <= 0.0) {
    throw BT::RuntimeError("TerminalPathLatch 的 xy_tolerance 必须大于零");
  }
  if (!getInput("path_goal_xy_tolerance", path_goal_xy_tolerance) ||
    path_goal_xy_tolerance <= 0.0)
  {
    throw BT::RuntimeError("TerminalPathLatch 的 path_goal_xy_tolerance 必须大于零");
  }
  if (!getInput("path_goal_yaw_tolerance", path_goal_yaw_tolerance) ||
    path_goal_yaw_tolerance <= 0.0)
  {
    throw BT::RuntimeError("TerminalPathLatch 的 path_goal_yaw_tolerance 必须大于零");
  }
  if (!getInput("transform_tolerance", transform_tolerance) || transform_tolerance < 0.0) {
    throw BT::RuntimeError("TerminalPathLatch 的 transform_tolerance 不得小于零");
  }
  if (!getInput("global_frame", global_frame) || global_frame.empty()) {
    throw BT::RuntimeError("TerminalPathLatch 的 global_frame 不得为空");
  }
  if (!getInput("robot_base_frame", robot_base_frame) || robot_base_frame.empty()) {
    throw BT::RuntimeError("TerminalPathLatch 的 robot_base_frame 不得为空");
  }

  // 新 action 或同一 XY 改变 yaw 都必须先解除旧锁存并生成属于新目标的路径。
  if (!active_goal_ || !sameGoal(*active_goal_, goal)) {
    resetLatch();
    active_goal_ = goal;
  }

  if (latched_) {
    return BT::NodeStatus::SUCCESS;
  }

  if (fresh_path_for_goal_ && canLatch(
      path, goal, xy_tolerance, path_goal_xy_tolerance, path_goal_yaw_tolerance,
      global_frame, robot_base_frame, transform_tolerance))
  {
    latched_ = true;
    RCLCPP_INFO(
      node_->get_logger(),
      "终点路径已锁存：机器人同时进入原始目标和路径末端 %.3f m 容差，停止重规划",
      xy_tolerance);
    if (child_node_->status() == BT::NodeStatus::RUNNING) {
      resetChild();
    }
    return BT::NodeStatus::SUCCESS;
  }

  // 未进入终点时保留原来的 1 Hz 规划行为。
  setStatus(BT::NodeStatus::RUNNING);
  const BT::NodeStatus child_status = child_node_->executeTick();
  if (child_status == BT::NodeStatus::SUCCESS) {
    // ComputePathToPose/SmoothPath 已在本 tick 更新黑板。重新读取路径，确保
    // 新 action 不会把上一目标的旧路径误认为当前目标的有效路径。
    nav_msgs::msg::Path fresh_path;
    getInput("path", fresh_path);
    if (!pathMatchesGoal(
        fresh_path, goal, path_goal_xy_tolerance, path_goal_yaw_tolerance))
    {
      fresh_path_for_goal_ = false;
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "新路径末端与当前目标不匹配；拒绝交给 FollowPath 并等待重新规划");
      return BT::NodeStatus::FAILURE;
    }
    fresh_path_for_goal_ = true;

    if (canLatch(
        fresh_path, goal, xy_tolerance, path_goal_xy_tolerance,
        path_goal_yaw_tolerance, global_frame, robot_base_frame,
        transform_tolerance))
    {
      latched_ = true;
      RCLCPP_INFO(
        node_->get_logger(),
        "终点路径已锁存：机器人同时进入原始目标和路径末端 %.3f m 容差，停止重规划",
        xy_tolerance);
      return BT::NodeStatus::SUCCESS;
    }
  }

  // 不能在 SUCCESS 后 resetChild()：RateController 会自行保存计时起点并在
  // 周期未到时返回 RUNNING。把它 halt 回 IDLE 会使每个 BT tick 都被当成
  // “第一次执行”，从而把 1 Hz 重规划放大到规划器可完成的最高频率。
  return child_status;
}

void TerminalPathLatch::halt()
{
  resetLatch();
  active_goal_.reset();
  resetChild();
  BT::DecoratorNode::halt();
}

bool TerminalPathLatch::isLatched() const
{
  return latched_;
}

bool TerminalPathLatch::sameGoal(
  const geometry_msgs::msg::PoseStamped & lhs,
  const geometry_msgs::msg::PoseStamped & rhs)
{
  constexpr double position_epsilon = 1.0e-3;
  constexpr double quaternion_dot_epsilon = 1.0e-6;
  if (lhs.header.frame_id != rhs.header.frame_id) {
    return false;
  }

  const auto & lhs_orientation = lhs.pose.orientation;
  const auto & rhs_orientation = rhs.pose.orientation;
  const double quaternion_dot = std::abs(
    lhs_orientation.x * rhs_orientation.x +
    lhs_orientation.y * rhs_orientation.y +
    lhs_orientation.z * rhs_orientation.z +
    lhs_orientation.w * rhs_orientation.w);

  return
    std::hypot(
      lhs.pose.position.x - rhs.pose.position.x,
      lhs.pose.position.y - rhs.pose.position.y) <= position_epsilon &&
    quaternion_dot >= 1.0 - quaternion_dot_epsilon;
}

bool TerminalPathLatch::pathMatchesGoal(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & goal,
  const double path_goal_xy_tolerance,
  const double path_goal_yaw_tolerance)
{
  if (path.poses.empty() || path.header.frame_id != goal.header.frame_id) {
    return false;
  }

  const auto & path_goal = path.poses.back().pose;
  const double xy_error = std::hypot(
    path_goal.position.x - goal.pose.position.x,
    path_goal.position.y - goal.pose.position.y);
  const double yaw_error = std::abs(tf2::getYaw(path_goal.orientation) -
    tf2::getYaw(goal.pose.orientation));
  const double normalized_yaw_error = std::abs(std::atan2(
      std::sin(yaw_error), std::cos(yaw_error)));
  return xy_error <= path_goal_xy_tolerance &&
         normalized_yaw_error <= path_goal_yaw_tolerance;
}

bool TerminalPathLatch::canLatch(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & goal,
  const double xy_tolerance,
  const double path_goal_xy_tolerance,
  const double path_goal_yaw_tolerance,
  const std::string & global_frame,
  const std::string & robot_base_frame,
  const double transform_tolerance)
{
  if (!pathMatchesGoal(
      path, goal, path_goal_xy_tolerance, path_goal_yaw_tolerance))
  {
    return false;
  }

  geometry_msgs::msg::PoseStamped path_goal;
  path_goal.header.frame_id = path.header.frame_id;
  path_goal.pose = path.poses.back().pose;
  // 原始目标距离保证用户要求的 0.30 m 语义；路径末端距离保证
  // RotationShimController 的 PositionGoalChecker 会在锁存后立即接管。
  return robotWithinTolerance(
    goal, xy_tolerance, global_frame, robot_base_frame, transform_tolerance) &&
    robotWithinTolerance(
    path_goal, xy_tolerance, global_frame, robot_base_frame, transform_tolerance);
}

bool TerminalPathLatch::robotWithinTolerance(
  const geometry_msgs::msg::PoseStamped & goal,
  const double xy_tolerance,
  const std::string & global_frame,
  const std::string & robot_base_frame,
  const double transform_tolerance)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf_buffer_, global_frame, robot_base_frame, transform_tolerance))
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "TerminalPathLatch 暂时无法取得 %s -> %s TF；保持未锁存并继续规划",
      global_frame.c_str(), robot_base_frame.c_str());
    return false;
  }

  geometry_msgs::msg::PoseStamped goal_in_global = goal;
  if (goal.header.frame_id != global_frame && !nav2_util::transformPoseInTargetFrame(
      goal, goal_in_global, *tf_buffer_, global_frame, transform_tolerance))
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "TerminalPathLatch 无法把目标从 %s 转换到 %s；保持未锁存并继续规划",
      goal.header.frame_id.c_str(), global_frame.c_str());
    return false;
  }

  return std::hypot(
    robot_pose.pose.position.x - goal_in_global.pose.position.x,
    robot_pose.pose.position.y - goal_in_global.pose.position.y) <= xy_tolerance;
}

void TerminalPathLatch::resetLatch()
{
  latched_ = false;
  fresh_path_for_goal_ = false;
}

}  // namespace go2_navigation_bt_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<go2_navigation_bt_plugins::TerminalPathLatch>(
    "TerminalPathLatch");
}
