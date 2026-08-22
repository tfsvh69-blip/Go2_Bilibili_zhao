#ifndef GO2_LIDAR_SCAN__LEVEL_FRAME_PUBLISHER_HPP_
#define GO2_LIDAR_SCAN__LEVEL_FRAME_PUBLISHER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace go2_lidar_scan
{

class LevelFramePublisher : public rclcpp::Node
{
public:
  explicit LevelFramePublisher(const rclcpp::NodeOptions & options);

private:
  void cloud_callback(std::unique_ptr<sensor_msgs::msg::PointCloud2> message);

  std::string cloud_topic_;
  std::string cloud_ready_topic_;
  std::string probe_cloud_topic_;
  std::string reference_frame_;
  std::string level_frame_;
  std::string source_transform_topic_;
  std::string cloud_heartbeat_topic_;
  double lookup_timeout_s_{0.05};
  std::size_t probe_stride_{5};
  std::size_t received_clouds_{0};
  std::size_t published_transforms_{0};
  std::size_t failed_transforms_{0};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr
    source_transform_pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr cloud_heartbeat_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_ready_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr probe_cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

}  // namespace go2_lidar_scan

#endif  // GO2_LIDAR_SCAN__LEVEL_FRAME_PUBLISHER_HPP_
