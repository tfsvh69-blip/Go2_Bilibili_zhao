#include "go2_lidar_scan/level_frame_publisher.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace go2_lidar_scan
{

LevelFramePublisher::LevelFramePublisher(const rclcpp::NodeOptions & options)
: Node("go2_lidar_level_frame", options)
{
  cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/velodyne_points");
  cloud_ready_topic_ = declare_parameter<std::string>(
    "cloud_ready_topic", "/go2_lidar_scan/leveled_cloud");
  probe_cloud_topic_ = declare_parameter<std::string>(
    "probe_cloud_topic", "/go2_lidar_scan/probe_cloud");
  reference_frame_ = declare_parameter<std::string>("reference_frame", "base_footprint");
  level_frame_ = declare_parameter<std::string>("level_frame", "velodyne_level");
  source_transform_topic_ = declare_parameter<std::string>(
    "source_transform_topic", "/go2_lidar_scan/level_source_transform");
  cloud_heartbeat_topic_ = declare_parameter<std::string>(
    "cloud_heartbeat_topic", "/go2_lidar_scan/cloud_heartbeat");
  lookup_timeout_s_ = declare_parameter<double>("lookup_timeout_s", 0.05);
  const auto probe_stride = declare_parameter<int64_t>("probe_stride", 5);

  if (cloud_topic_.empty() || cloud_ready_topic_.empty() || probe_cloud_topic_.empty()) {
    throw std::invalid_argument("cloud_topic、cloud_ready_topic 和 probe_cloud_topic 不得为空");
  }
  if (cloud_topic_ == cloud_ready_topic_) {
    throw std::invalid_argument("原始点云与 TF 就绪点云话题不得相同");
  }
  if (reference_frame_.empty() || level_frame_.empty()) {
    throw std::invalid_argument("reference_frame 和 level_frame 不得为空");
  }
  if (reference_frame_ == level_frame_) {
    throw std::invalid_argument("reference_frame 和 level_frame 不得相同");
  }
  if (!std::isfinite(lookup_timeout_s_) || lookup_timeout_s_ < 0.0) {
    throw std::invalid_argument("lookup_timeout_s 必须是非负有限数");
  }
  if (probe_stride <= 0) {
    throw std::invalid_argument("probe_stride 必须是正整数");
  }
  probe_stride_ = static_cast<std::size_t>(probe_stride);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  // TF 监听器使用自己的节点和线程：Humble 不允许开启 intra-process 的组件
  // 创建 transient-local 的 /tf_static 订阅。这样既保留静态 TF 语义，又只
  // 对下面的易失性就绪点云启用同进程零拷贝。
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
  source_transform_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(
    source_transform_topic_, status_qos);
  cloud_heartbeat_pub_ = create_publisher<std_msgs::msg::Header>(
    cloud_heartbeat_topic_, status_qos);

  auto cloud_qos = rclcpp::SensorDataQoS();
  cloud_qos.keep_last(2);
  // 只有本节点直接接收 Gazebo 大点云。成功发布本帧的重力对齐 TF 后，才把
  // 原消息零拷贝交给同进程的上游转换组件；失败帧不会进入 /scan。
  cloud_ready_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    cloud_ready_topic_, cloud_qos);
  // 外部 Python 诊断只需抽样几何证据。专用低频话题避免它让
  // 每帧大点云退化为 DDS 跨进程拷贝，导航转换链仍保持零拷贝。
  probe_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    probe_cloud_topic_, cloud_qos);
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, cloud_qos,
    std::bind(&LevelFramePublisher::cloud_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "重力对齐坐标系已启动：%s -> %s，输入=%s，就绪输出=%s",
    reference_frame_.c_str(), level_frame_.c_str(), cloud_topic_.c_str(),
    cloud_ready_topic_.c_str());
}

void LevelFramePublisher::cloud_callback(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> message)
{
  ++received_clouds_;
  cloud_heartbeat_pub_->publish(message->header);
  const auto sensor_frame = message->header.frame_id;
  if (sensor_frame.empty() || sensor_frame == level_frame_) {
    ++failed_transforms_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "点云 frame_id 为空或等于 level_frame，本帧不发布重力对齐 TF 和 /scan");
    return;
  }

  try {
    auto sensor_transform = tf_buffer_->lookupTransform(
      reference_frame_, sensor_frame, rclcpp::Time(message->header.stamp),
      rclcpp::Duration::from_seconds(lookup_timeout_s_));
    const auto & translation = sensor_transform.transform.translation;
    if (!std::isfinite(translation.x) || !std::isfinite(translation.y) ||
      !std::isfinite(translation.z))
    {
      throw std::invalid_argument("雷达平移包含非有限值");
    }

    const auto & rotation = sensor_transform.transform.rotation;
    const double norm = std::sqrt(
      rotation.x * rotation.x + rotation.y * rotation.y +
      rotation.z * rotation.z + rotation.w * rotation.w);
    if (!std::isfinite(norm) || norm <= 1.0e-12) {
      throw std::invalid_argument("雷达四元数非法");
    }
    tf2::Quaternion quaternion(
      rotation.x / norm, rotation.y / norm, rotation.z / norm, rotation.w / norm);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    (void)roll;
    (void)pitch;

    geometry_msgs::msg::TransformStamped level_transform;
    level_transform.header.stamp = message->header.stamp;
    level_transform.header.frame_id = reference_frame_;
    level_transform.child_frame_id = level_frame_;
    level_transform.transform.translation = translation;
    tf2::Quaternion yaw_only;
    yaw_only.setRPY(0.0, 0.0, yaw);
    level_transform.transform.rotation = tf2::toMsg(yaw_only);

    // 输出严格沿用点云时间戳；查询失败时绝不复用上一帧。发布顺序也是接口
    // 契约：先 TF，再放行对应点云，避免转换器与 TF 在不同进程中竞速。
    tf_broadcaster_->sendTransform(level_transform);
    sensor_transform.header.stamp = message->header.stamp;
    source_transform_pub_->publish(sensor_transform);
    ++published_transforms_;
    if (probe_cloud_pub_->get_subscription_count() > 0 &&
      (published_transforms_ - 1) % probe_stride_ == 0)
    {
      probe_cloud_pub_->publish(*message);
    }
    cloud_ready_pub_->publish(std::move(message));
  } catch (const std::exception & error) {
    ++failed_transforms_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "无法按点云时间发布 %s：%s；本帧不会获得 /scan",
      level_frame_.c_str(), error.what());
  }
}

}  // namespace go2_lidar_scan

RCLCPP_COMPONENTS_REGISTER_NODE(go2_lidar_scan::LevelFramePublisher)
