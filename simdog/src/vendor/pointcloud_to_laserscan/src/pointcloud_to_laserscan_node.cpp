/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include "pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp"

#include <cmath>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace pointcloud_to_laserscan
{

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_laserscan", options)
{
  target_frame_ = this->declare_parameter("target_frame", "");
  tolerance_ = this->declare_parameter("transform_tolerance", 0.01);
  // TODO(hidmic): adjust default input queue size based on actual concurrency levels
  // achievable by the associated executor
  input_queue_size_ = this->declare_parameter(
    "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  always_subscribe_ = this->declare_parameter("always_subscribe", false);
  allow_runtime_height_update_ = this->declare_parameter(
    "allow_runtime_height_update", false);

  if (allow_runtime_height_update_) {
    // 为 rqt_reconfigure 提供明确的物理范围与 1 cm 步长。上下界之间的关系仍由
    // parameterCallback 原子校验，避免界面显示成功但转换器使用了非法窗口。
    rcl_interfaces::msg::FloatingPointRange height_range;
    height_range.from_value = -0.50;
    height_range.to_value = 1.50;
    height_range.step = 0.01;
    rcl_interfaces::msg::ParameterDescriptor min_height_descriptor;
    min_height_descriptor.description =
      "水平雷达坐标系中的高度切片下界（m）；过低会把地面投影进 /scan";
    min_height_descriptor.floating_point_range = {height_range};
    rcl_interfaces::msg::ParameterDescriptor max_height_descriptor;
    max_height_descriptor.description =
      "水平雷达坐标系中的高度切片上界（m）；过低会漏掉墙体和矮障碍";
    max_height_descriptor.floating_point_range = {height_range};
    min_height_ = this->declare_parameter(
      "min_height", 0.0, min_height_descriptor);
    max_height_ = this->declare_parameter(
      "max_height", 1.0, max_height_descriptor);
  } else {
    // 保持上游通用节点的无界默认值；项目正式转换器才启用受控动态窗口。
    min_height_ = this->declare_parameter(
      "min_height", std::numeric_limits<double>::min());
    max_height_ = this->declare_parameter(
      "max_height", std::numeric_limits<double>::max());
  }
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter("use_inf", true);

  if (!std::isfinite(min_height_) || !std::isfinite(max_height_) ||
    min_height_ >= max_height_)
  {
    throw std::invalid_argument("min_height/max_height 必须为有限数且 min_height < max_height");
  }

  using std::placeholders::_1;
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&PointCloudToLaserScanNode::parameterCallback, this, _1));

  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty()) {
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    message_filter_ = std::make_unique<MessageFilter>(
      sub_, *tf2_, target_frame_, input_queue_size_,
      this->get_node_logging_interface(),
      this->get_node_clock_interface());
    message_filter_->registerCallback(
      std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  } else {  // otherwise setup direct subscription
    sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  }

  if (always_subscribe_) {
    rclcpp::SensorDataQoS qos;
    qos.keep_last(input_queue_size_);
    sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
    RCLCPP_INFO(
      this->get_logger(),
      "always_subscribe=true，持续订阅点云，不随 /scan 订阅者变化启停");
  } else {
    subscription_listener_thread_ = std::thread(
      std::bind(&PointCloudToLaserScanNode::subscriptionListenerThreadLoop, this));
  }
}

rcl_interfaces::msg::SetParametersResult PointCloudToLaserScanNode::parameterCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;

  double proposed_min;
  double proposed_max;
  {
    std::lock_guard<std::mutex> lock(height_mutex_);
    proposed_min = min_height_;
    proposed_max = max_height_;
  }

  bool height_changed = false;
  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == "use_sim_time") {
      // rclcpp 自己维护该通用参数，允许其按 ROS 2 标准语义修改。
      continue;
    }
    if (name != "min_height" && name != "max_height") {
      result.reason =
        "运行时只支持 min_height/max_height；其余投影参数需改配置并重启，避免界面值已变但算法未变";
      return result;
    }
    if (!allow_runtime_height_update_) {
      result.reason = "本节点未启用 allow_runtime_height_update，需改配置并重启";
      return result;
    }
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      result.reason = name + " 必须是 double";
      return result;
    }
    const double value = parameter.as_double();
    if (!std::isfinite(value)) {
      result.reason = name + " 必须是有限数";
      return result;
    }
    if (name == "min_height") {
      proposed_min = value;
    } else {
      proposed_max = value;
    }
    height_changed = true;
  }

  if (proposed_min >= proposed_max) {
    result.reason = "必须满足 min_height < max_height";
    return result;
  }

  if (height_changed) {
    std::lock_guard<std::mutex> lock(height_mutex_);
    min_height_ = proposed_min;
    max_height_ = proposed_max;
    RCLCPP_INFO(
      this->get_logger(), "二维扫描高度窗口已生效：[%.3f, %.3f] m", min_height_, max_height_);
  }
  result.successful = true;
  result.reason = "参数已生效";
  return result;
}

PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
{
  alive_.store(false);
  if (subscription_listener_thread_.joinable()) {
    subscription_listener_thread_.join();
  }
  sub_.unsubscribe();
}

void PointCloudToLaserScanNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();

  const std::chrono::milliseconds timeout(100);
  while (rclcpp::ok(context) && alive_.load()) {
    int subscription_count = pub_->get_subscription_count() +
      pub_->get_intra_process_subscription_count();
    if (subscription_count > 0) {
      if (!sub_.getSubscriber()) {
        RCLCPP_INFO(
          this->get_logger(),
          "Got a subscriber to laserscan, starting pointcloud subscriber");
        rclcpp::SensorDataQoS qos;
        qos.keep_last(input_queue_size_);
        sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
      }
    } else if (sub_.getSubscriber()) {
      RCLCPP_INFO(
        this->get_logger(),
        "No subscribers to laserscan, shutting down pointcloud subscriber");
      sub_.unsubscribe();
    }
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }
  sub_.unsubscribe();
}

void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  double min_height;
  double max_height;
  {
    std::lock_guard<std::mutex> lock(height_mutex_);
    min_height = min_height_;
    max_height = max_height_;
  }

  // build laserscan output
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;
  if (!target_frame_.empty()) {
    scan_msg->header.frame_id = target_frame_;
  }

  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  // Transform cloud if necessary
  if (scan_msg->header.frame_id != cloud_msg->header.frame_id) {
    try {
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_));
      cloud_msg = cloud;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
      return;
    }
  }

  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
    iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for nan in point(%f, %f, %f)\n",
        *iter_x, *iter_y, *iter_z);
      continue;
    }

    if (*iter_z > max_height || *iter_z < min_height) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for height %f not in range (%f, %f)\n",
        *iter_z, min_height, max_height);
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
        range, range_min_, *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
        range, range_max_, *iter_x, *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for angle %f not in range (%f, %f)\n",
        angle, scan_msg->angle_min, scan_msg->angle_max);
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
    if (range < scan_msg->ranges[index]) {
      scan_msg->ranges[index] = range;
    }
  }
  pub_->publish(std::move(scan_msg));
}

}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)
