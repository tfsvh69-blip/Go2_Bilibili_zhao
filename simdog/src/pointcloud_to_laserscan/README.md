# ROS 2 点云 ↔ 激光扫描转换器

本 ROS 2 包提供将 `sensor_msgs/msg/PointCloud2` 消息转换为 `sensor_msgs/msg/LaserScan` 消息以及反向转换的组件。本质上是原始 ROS 1 包的移植版本。

## pointcloud_to_laserscan::PointCloudToLaserScanNode

此 ROS 2 组件将 `sensor_msgs/msg/PointCloud2` 消息投影为 `sensor_msgs/msg/LaserScan` 消息。

### 发布话题

* `scan` (`sensor_msgs/msg/LaserScan`) — 输出激光扫描。

### 订阅话题

* `cloud_in` (`sensor_msgs/msg/PointCloud2`) — 输入点云。若 `scan` 话题没有至少一个订阅者，则不处理任何输入。

### 参数

* `min_height`（double，默认：2.2e-308）— 点云中采样的最小高度，单位米。
* `max_height`（double，默认：1.8e+308）— 点云中采样的最大高度，单位米。
* `angle_min`（double，默认：-π）— 最小扫描角度，单位弧度。
* `angle_max`（double，默认：π）— 最大扫描角度，单位弧度。
* `angle_increment`（double，默认：π/180）— 激光扫描分辨率，单位弧度/每射线。
* `queue_size`（double，默认：检测到的 CPU 核心数）— 输入点云队列大小。
* `scan_time`（double，默认：1.0/30.0）— 扫描周期，单位秒。仅用于填充输出 LaserScan 消息的 scan_time 字段。
* `range_min`（double，默认：0.0）— 返回的最小距离，单位米。
* `range_max`（double，默认：1.8e+308）— 返回的最大距离，单位米。
* `target_frame`（str，默认：无）— 若提供，在转换为激光扫描前先将点云变换到此坐标系。否则，激光扫描将生成在输入点云的同一坐标系中。
* `transform_tolerance`（double，默认：0.01）— 坐标变换查找的时间容差，单位秒。仅在提供 `target_frame` 时使用。
* `use_inf`（boolean，默认：true）— 若禁用，将无限距离（无障碍物）报告为 range_max + 1；否则报告为 +inf。

## pointcloud_to_laserscan::LaserScanToPointCloudNode

此 ROS 2 组件将 `sensor_msgs/msg/LaserScan` 消息重新发布为 `sensor_msgs/msg/PointCloud2` 消息。

### 发布话题

* `cloud` (`sensor_msgs/msg/PointCloud2`) — 输出点云。

### 订阅话题

* `scan_in` (`sensor_msgs/msg/LaserScan`) — 输入激光扫描。若 `cloud` 话题没有至少一个订阅者，则不处理任何输入。

### 参数

* `queue_size`（double，默认：检测到的 CPU 核心数）— 输入激光扫描队列大小。
* `target_frame`（str，默认：无）— 若提供，在转换为激光扫描前先将点云变换到此坐标系。否则，激光扫描将生成在输入点云的同一坐标系中。
* `transform_tolerance`（double，默认：0.01）— 坐标变换查找的时间容差，单位秒。仅在提供 `target_frame` 时使用。
