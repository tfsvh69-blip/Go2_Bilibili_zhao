#include <memory>

#include "go2_lidar_scan/level_frame_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<go2_lidar_scan::LevelFramePublisher>(
      rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
