#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdlib>
#include <string>

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;

class MapPublisher : public rclcpp::Node {
public:
    MapPublisher() : Node("map_publisher") {
        const char *home = std::getenv("HOME");
        const std::string default_map_path = home == nullptr
            ? "go2_maps/latest/GlobalMap.pcd"
            : std::string(home) + "/go2_maps/latest/GlobalMap.pcd";
        declare_parameter("map_path", default_map_path);
        declare_parameter("map_topic", "/global_map");
        declare_parameter("frame_id", "map");

        const std::string map_path = get_parameter("map_path").as_string();
        const std::string map_topic = get_parameter("map_topic").as_string();
        const std::string frame_id = get_parameter("frame_id").as_string();

        // 加载全局 PCD 地图。
        PointCloud::Ptr global_map(new PointCloud);
        if (pcl::io::loadPCDFile<PointT>(map_path, *global_map) == -1) {
            RCLCPP_ERROR(
                this->get_logger(), "Failed to load PCD file: %s", map_path.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map->size());

        // 转换并发布地图。
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(map_topic, 1);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this, global_map, frame_id]() {
                sensor_msgs::msg::PointCloud2 msg;
                pcl::toROSMsg(*global_map, msg);
                msg.header.stamp = this->now();
                msg.header.frame_id = frame_id;
                map_pub_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Published global map");
            }
        );
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
