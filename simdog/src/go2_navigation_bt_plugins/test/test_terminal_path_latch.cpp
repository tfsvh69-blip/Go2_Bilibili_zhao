// Copyright (c) 2026 hao
// SPDX-License-Identifier: BSD-3-Clause

#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/actions/always_success_node.h"
#include "gtest/gtest.h"
#include "go2_navigation_bt_plugins/terminal_path_latch.hpp"
#include "nav2_behavior_tree/plugins/decorator/rate_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace
{

geometry_msgs::msg::Quaternion yawQuaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.z = std::sin(yaw * 0.5);
  orientation.w = std::cos(yaw * 0.5);
  return orientation;
}

geometry_msgs::msg::PoseStamped goalPose(const double x, const double y, const double yaw)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.orientation = yawQuaternion(yaw);
  return goal;
}

nav_msgs::msg::Path pathToGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = goal.header.frame_id;
  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = goal.header.frame_id;
  start.pose.orientation.w = 1.0;
  path.poses = {start, goal};
  return path;
}

class CountingSuccessNode : public BT::SyncActionNode
{
public:
  CountingSuccessNode(
    const std::string & name, const BT::NodeConfiguration & config, int & ticks)
  : BT::SyncActionNode(name, config), ticks_(ticks) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    ++ticks_;
    return BT::NodeStatus::SUCCESS;
  }

private:
  int & ticks_;
};

class TerminalPathLatchFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("terminal_path_latch_test");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);
    blackboard_ = BT::Blackboard::create();
    blackboard_->set<rclcpp::Node::SharedPtr>("node", node_);
    blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer_);

    BT::NodeConfiguration latch_config;
    latch_config.blackboard = blackboard_;
    latch_config.input_ports = {
      {"path", "{path}"},
      {"goal", "{goal}"},
      {"xy_tolerance", "0.30"},
      {"path_goal_xy_tolerance", "0.075"},
      {"path_goal_yaw_tolerance", "0.01"},
      {"global_frame", "map"},
      {"robot_base_frame", "base_footprint"},
      {"transform_tolerance", "0.00"},
    };
    latch_ = std::make_unique<go2_navigation_bt_plugins::TerminalPathLatch>(
      "TerminalPathLatch", latch_config);

    child_ = std::make_unique<BT::AlwaysSuccessNode>("Planner");
    latch_->setChild(child_.get());
  }

  void setRobotPose(const double x, const double y, const double yaw)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node_->now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_footprint";
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.rotation = yawQuaternion(yaw);
    ASSERT_TRUE(tf_buffer_->setTransform(transform, "test", true));
  }

  void setInputs(
    const geometry_msgs::msg::PoseStamped & goal,
    const nav_msgs::msg::Path & path)
  {
    blackboard_->set("goal", goal);
    blackboard_->set("path", path);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<go2_navigation_bt_plugins::TerminalPathLatch> latch_;
  std::unique_ptr<BT::AlwaysSuccessNode> child_;
};

TEST_F(TerminalPathLatchFixture, LatchesUsingRealRobotTfDistance)
{
  const auto goal = goalPose(2.0, -1.0, 1.57);
  setRobotPose(1.75, -1.0, 0.0);
  setInputs(goal, pathToGoal(goal));

  EXPECT_EQ(latch_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(latch_->isLatched());
}

TEST_F(TerminalPathLatchFixture, DoesNotLatchOutsideTolerance)
{
  const auto goal = goalPose(2.0, -1.0, 1.57);
  setRobotPose(1.69, -1.0, 0.0);
  setInputs(goal, pathToGoal(goal));

  EXPECT_EQ(latch_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(latch_->isLatched());
}

TEST_F(TerminalPathLatchFixture, BoundaryDriftDoesNotReleaseExistingLatch)
{
  const auto goal = goalPose(2.0, -1.0, 1.57);
  setRobotPose(1.75, -1.0, 0.0);
  setInputs(goal, pathToGoal(goal));
  ASSERT_EQ(latch_->executeTick(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(latch_->isLatched());

  tf_buffer_->clear();
  EXPECT_EQ(latch_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(latch_->isLatched());
}

TEST_F(TerminalPathLatchFixture, SamePositionWithNewYawClearsLatch)
{
  const auto first_goal = goalPose(2.0, -1.0, 1.57);
  setRobotPose(1.75, -1.0, 0.0);
  setInputs(first_goal, pathToGoal(first_goal));
  ASSERT_EQ(latch_->executeTick(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(latch_->isLatched());

  const auto new_goal = goalPose(2.0, -1.0, -1.57);
  setInputs(new_goal, pathToGoal(first_goal));
  EXPECT_EQ(latch_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(latch_->isLatched());

  setInputs(new_goal, pathToGoal(new_goal));
  EXPECT_EQ(latch_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(latch_->isLatched());
}

TEST_F(TerminalPathLatchFixture, RejectsPathBelongingToAnotherGoal)
{
  const auto goal = goalPose(2.0, -1.0, 1.57);
  const auto other_goal = goalPose(2.1, -1.0, 1.57);
  setRobotPose(1.75, -1.0, 0.0);
  setInputs(goal, pathToGoal(other_goal));

  EXPECT_EQ(latch_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(latch_->isLatched());
}

TEST_F(TerminalPathLatchFixture, AcceptsFiveCentimeterGridCenterOffset)
{
  const auto goal = goalPose(2.0, -1.0, 1.57);
  auto path_goal = goal;
  path_goal.pose.position.x += 0.05;
  setRobotPose(1.80, -1.0, 0.0);
  setInputs(goal, pathToGoal(path_goal));

  EXPECT_EQ(latch_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(latch_->isLatched());
}

TEST_F(TerminalPathLatchFixture, RejectsPathBeyondGridMatchingTolerance)
{
  const auto goal = goalPose(2.0, -1.0, 1.57);
  auto path_goal = goal;
  path_goal.pose.position.x += 0.08;
  setRobotPose(1.80, -1.0, 0.0);
  setInputs(goal, pathToGoal(path_goal));

  EXPECT_EQ(latch_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(latch_->isLatched());
}

TEST_F(TerminalPathLatchFixture, RejectsPathBeyondYawMatchingTolerance)
{
  const auto goal = goalPose(2.0, -1.0, 1.57);
  auto path_goal = goal;
  path_goal.pose.orientation = yawQuaternion(1.59);
  setRobotPose(1.80, -1.0, 0.0);
  setInputs(goal, pathToGoal(path_goal));

  EXPECT_EQ(latch_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(latch_->isLatched());
}

TEST_F(TerminalPathLatchFixture, RequiresRobotNearRawGoalAndPathEndpoint)
{
  const auto goal = goalPose(2.0, -1.0, 1.57);
  auto path_goal = goal;
  path_goal.pose.position.x += 0.05;
  // 到原始目标 0.29 m，但到路径末端 0.34 m，Rotation Shim 尚不能接管。
  setRobotPose(1.71, -1.0, 0.0);
  setInputs(goal, pathToGoal(path_goal));

  EXPECT_EQ(latch_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(latch_->isLatched());
}

TEST_F(TerminalPathLatchFixture, PreservesRateControllerTimerAfterPlannerSuccess)
{
  const auto goal = goalPose(2.0, -1.0, 1.57);
  setRobotPose(1.0, -1.0, 0.0);
  setInputs(goal, pathToGoal(goal));

  BT::NodeConfiguration latch_config;
  latch_config.blackboard = blackboard_;
  latch_config.input_ports = {
    {"path", "{path}"},
    {"goal", "{goal}"},
    {"xy_tolerance", "0.30"},
    {"path_goal_xy_tolerance", "0.075"},
    {"path_goal_yaw_tolerance", "0.01"},
    {"global_frame", "map"},
    {"robot_base_frame", "base_footprint"},
    {"transform_tolerance", "0.00"},
  };
  auto rate_latch = std::make_unique<go2_navigation_bt_plugins::TerminalPathLatch>(
    "RateTerminalPathLatch", latch_config);
  int planner_ticks = 0;
  BT::NodeConfiguration counter_config;
  counter_config.blackboard = blackboard_;
  auto counter = std::make_unique<CountingSuccessNode>(
    "CountingPlanner", counter_config, planner_ticks);
  BT::NodeConfiguration rate_config;
  rate_config.blackboard = blackboard_;
  rate_config.input_ports = {{"hz", "1.0"}};
  auto rate = std::make_unique<nav2_behavior_tree::RateController>(
    "OneHzPlanner", rate_config);
  rate->setChild(counter.get());
  rate_latch->setChild(rate.get());

  EXPECT_EQ(rate_latch->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(planner_ticks, 1);
  for (int index = 0; index < 10; ++index) {
    EXPECT_EQ(rate_latch->executeTick(), BT::NodeStatus::RUNNING);
  }
  EXPECT_EQ(planner_ticks, 1);
}

TEST_F(TerminalPathLatchFixture, TfFailureKeepsPlanningWithoutLatch)
{
  const auto goal = goalPose(2.0, -1.0, 1.57);
  setInputs(goal, pathToGoal(goal));

  EXPECT_EQ(latch_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(latch_->isLatched());
}

TEST_F(TerminalPathLatchFixture, HaltAndRecoveryClearLatch)
{
  const auto goal = goalPose(2.0, -1.0, 1.57);
  setRobotPose(1.75, -1.0, 0.0);
  setInputs(goal, pathToGoal(goal));
  ASSERT_EQ(latch_->executeTick(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(latch_->isLatched());

  // PipelineSequence 在 action 结束或 recovery 开始时都会向装饰节点传播 halt。
  latch_->halt();
  EXPECT_FALSE(latch_->isLatched());
}

}  // namespace
