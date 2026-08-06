#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/registration/ndt.h> // 注释掉原始NDT
#include <pclomp/ndt_omp.h> // 使用OpenMP加速的NDT
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

#include <cstdlib>
#include <string>

#ifdef NDT_CUDA_ENABLED
#include <cuda_runtime_api.h>
#include <fast_gicp/ndt/ndt_cuda.hpp>
#endif

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;
// 使用pclomp命名空间的NDT实现
using NormalDistributionsTransform = pclomp::NormalDistributionsTransform<PointT, PointT>;
#ifdef NDT_CUDA_ENABLED
using CudaNormalDistributionsTransform = fast_gicp::NDTCuda<PointT, PointT>;
#endif

class NDTLocalization : public rclcpp::Node {
public:
    NDTLocalization() : Node("ndt_relocalization_node") {
        // 从参数服务器获取参数
        const char *home = std::getenv("HOME");
        const std::string default_map_path = home == nullptr
            ? "go2_maps/latest/GlobalMap.pcd"
            : std::string(home) + "/go2_maps/latest/GlobalMap.pcd";
        declare_parameter("map_path", default_map_path);
        
        // NDT参数
        declare_parameter("ndt_resolution", 1.0);
        declare_parameter("ndt_step_size", 0.1);
        declare_parameter("ndt_epsilon", 0.01);
        declare_parameter("ndt_max_iterations", 50);
        
        // 多分辨率NDT参数
        declare_parameter("use_multi_scale_ndt", true);
        declare_parameter("ndt_resolutions", std::vector<double>{4.0, 2.0, 1.0});
        declare_parameter("ndt_num_threads", 4);
        declare_parameter("registration_backend", "cuda");
        declare_parameter("gpu_device_id", 0);
        
        // 点云预处理参数
        declare_parameter("voxel_leaf_size", 0.2);
        declare_parameter("input_cloud_topic", "/velodyne_points");
        declare_parameter("input_cloud_frame", "velodyne");
        
        // 坐标系配置
        declare_parameter("global_frame_id", "map");
        declare_parameter("odom_frame_id", "odom");
        declare_parameter("robot_frame_id", "base_link");
        declare_parameter("publish_tf", true);
        
        // 其他配置
        declare_parameter("map_topic", "/global_map");
        declare_parameter("debug_mode", false);
        declare_parameter("init_pose_topic", "/initialpose");
        declare_parameter("use_initial_pose", false);
        declare_parameter("initial_pose_x", 0.0);
        declare_parameter("initial_pose_y", 0.0);
        declare_parameter("initial_pose_z", 0.0);
        declare_parameter("initial_pose_qx", 0.0);
        declare_parameter("initial_pose_qy", 0.0);
        declare_parameter("initial_pose_qz", 0.0);
        declare_parameter("initial_pose_qw", 1.0);
        
        // 获取配置值
        std::string map_path = get_parameter("map_path").as_string();
        ndt_resolution_ = get_parameter("ndt_resolution").as_double();
        ndt_step_size_ = get_parameter("ndt_step_size").as_double();
        ndt_epsilon_ = get_parameter("ndt_epsilon").as_double();
        ndt_max_iterations_ = get_parameter("ndt_max_iterations").as_int();
        
        use_multi_scale_ndt_ = get_parameter("use_multi_scale_ndt").as_bool();
        ndt_resolutions_ = get_parameter("ndt_resolutions").as_double_array();
        ndt_num_threads_ = get_parameter("ndt_num_threads").as_int();
        registration_backend_ = get_parameter("registration_backend").as_string();
        gpu_device_id_ = get_parameter("gpu_device_id").as_int();
        configureRegistrationBackend();
        
        voxel_leaf_size_ = get_parameter("voxel_leaf_size").as_double();
        std::string input_cloud_topic = get_parameter("input_cloud_topic").as_string();
        input_cloud_frame_ = get_parameter("input_cloud_frame").as_string();
        
        global_frame_id_ = get_parameter("global_frame_id").as_string();
        odom_frame_id_ = get_parameter("odom_frame_id").as_string();
        robot_frame_id_ = get_parameter("robot_frame_id").as_string();
        publish_tf_ = get_parameter("publish_tf").as_bool();
        
        map_topic_ = get_parameter("map_topic").as_string();
        debug_mode_ = get_parameter("debug_mode").as_bool();
        std::string init_pose_topic = get_parameter("init_pose_topic").as_string();
        use_initial_pose_ = get_parameter("use_initial_pose").as_bool();
        
        // 设置初始猜测姿态
        has_initial_guess_ = false;
        initial_guess_is_base_pose_ = false;
        if (use_initial_pose_) {
            initial_guess_ = Eigen::Matrix4f::Identity();
            double x = get_parameter("initial_pose_x").as_double();
            double y = get_parameter("initial_pose_y").as_double();
            double z = get_parameter("initial_pose_z").as_double();
            double qx = get_parameter("initial_pose_qx").as_double();
            double qy = get_parameter("initial_pose_qy").as_double();
            double qz = get_parameter("initial_pose_qz").as_double();
            double qw = get_parameter("initial_pose_qw").as_double();
            
            Eigen::Vector3d translation(x, y, z);
            Eigen::Quaterniond rotation(qw, qx, qy, qz);
            
            initial_guess_.block<3, 3>(0, 0) = rotation.toRotationMatrix().cast<float>();
            initial_guess_.block<3, 1>(0, 3) = translation.cast<float>();
            has_initial_guess_ = true;
            initial_guess_is_base_pose_ = true;
            RCLCPP_INFO(this->get_logger(), "Using initial pose: [%f, %f, %f]", x, y, z);
        }
        
        RCLCPP_INFO(this->get_logger(), "Loading map from: %s", map_path.c_str());
        RCLCPP_INFO(this->get_logger(), "NDT resolution: %f", ndt_resolution_);
        RCLCPP_INFO(this->get_logger(), "Multi-scale NDT: %s", use_multi_scale_ndt_ ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "Registration backend: %s", registration_backend_.c_str());
        RCLCPP_INFO(this->get_logger(), "Input cloud topic: %s", input_cloud_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "TF frames: %s -> %s -> %s", 
                   global_frame_id_.c_str(), odom_frame_id_.c_str(), robot_frame_id_.c_str());
        
        // 1. 加载全局地图 (PCD)
        global_map_ = std::make_shared<PointCloud>();
        if (pcl::io::loadPCDFile<PointT>(map_path, *global_map_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file from %s!", map_path.c_str());
            return;
        }

        // 2. 预处理全局地图：移除无效点并进行下采样
        PointCloud::Ptr filtered_map(new PointCloud);
        for (const auto& point : global_map_->points) {
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                filtered_map->points.push_back(point);
            }
        }
        filtered_map->width = filtered_map->points.size();
        filtered_map->height = 1;
        filtered_map->is_dense = true;

        // 对全局地图进行体素下采样
        pcl::VoxelGrid<PointT> voxel_grid;
        voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel_grid.setInputCloud(filtered_map);
        voxel_grid.filter(*filtered_map);

        global_map_ = filtered_map;
        RCLCPP_INFO(this->get_logger(), "Loaded and filtered global map with %zu points", global_map_->size());
        
        // 创建多分辨率地图
        if (use_multi_scale_ndt_) {
            if (registration_backend_ == "cuda") {
                createMultiResolutionCudaMaps();
            } else {
                createMultiResolutionMaps();
            }
        } else {
            // 创建单一分辨率NDT对象
            if (registration_backend_ == "cuda") {
                createCudaNDT();
            } else {
                createNDT();
            }
        }

        // 初始化TF监听器
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 3. 订阅实时点云和初始姿态
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_topic, 10,
            std::bind(&NDTLocalization::cloudCallback, this, std::placeholders::_1));
            
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            init_pose_topic, 10,
            std::bind(&NDTLocalization::initialPoseCallback, this, std::placeholders::_1));

        // 4. 发布位姿和地图
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ndt_pose", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/ndt_odom", 10);
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(map_topic_, 1);
        
        // 创建定时器定期发布地图点云
        map_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&NDTLocalization::publishMapCallback, this));

        // 5. TF 广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        RCLCPP_INFO(this->get_logger(), "NDT Localization node initialized successfully");
    }

private:
    void configureRegistrationBackend() {
        if (registration_backend_ != "cuda" && registration_backend_ != "omp") {
            RCLCPP_WARN(
                this->get_logger(),
                "Unknown registration_backend '%s', falling back to omp",
                registration_backend_.c_str());
            registration_backend_ = "omp";
        }

        if (registration_backend_ != "cuda") {
            return;
        }

#ifdef NDT_CUDA_ENABLED
        int device_count = 0;
        const cudaError_t count_result = cudaGetDeviceCount(&device_count);
        if (count_result != cudaSuccess || device_count == 0) {
            RCLCPP_WARN(
                this->get_logger(),
                "CUDA device unavailable (%s), falling back to omp",
                cudaGetErrorString(count_result));
            registration_backend_ = "omp";
            return;
        }

        if (gpu_device_id_ < 0 || gpu_device_id_ >= device_count) {
            RCLCPP_WARN(
                this->get_logger(),
                "gpu_device_id=%d is out of range [0, %d), falling back to device 0",
                gpu_device_id_, device_count);
            gpu_device_id_ = 0;
        }

        const cudaError_t set_result = cudaSetDevice(gpu_device_id_);
        if (set_result != cudaSuccess) {
            RCLCPP_WARN(
                this->get_logger(),
                "Cannot select CUDA device %d (%s), falling back to omp",
                gpu_device_id_, cudaGetErrorString(set_result));
            registration_backend_ = "omp";
            return;
        }

        cudaDeviceProp device_properties{};
        const cudaError_t properties_result =
            cudaGetDeviceProperties(&device_properties, gpu_device_id_);
        if (properties_result == cudaSuccess) {
            RCLCPP_INFO(
                this->get_logger(),
                "CUDA NDT enabled on device %d: %s (compute %d.%d)",
                gpu_device_id_, device_properties.name,
                device_properties.major, device_properties.minor);
        }
#else
        RCLCPP_WARN(
            this->get_logger(),
            "This build has no CUDA NDT support, falling back to omp");
        registration_backend_ = "omp";
#endif
    }

    void createNDT() {
        // 设置NDT参数
        ndt_ = std::make_shared<NormalDistributionsTransform>();
        ndt_->setResolution(ndt_resolution_);
        ndt_->setStepSize(ndt_step_size_);
        ndt_->setTransformationEpsilon(ndt_epsilon_);
        ndt_->setMaximumIterations(ndt_max_iterations_);
        ndt_->setInputTarget(global_map_);
        
        // 设置OpenMP线程数
        ndt_->setNumThreads(ndt_num_threads_);
        
        // 设置邻域搜索方法，可选KDTREE、DIRECT26、DIRECT7或DIRECT1
        ndt_->setNeighborhoodSearchMethod(pclomp::KDTREE);
        
        RCLCPP_INFO(this->get_logger(), "NDT parameters: resolution=%f, step_size=%f, epsilon=%f, max_iter=%d, threads=%d",
                   ndt_resolution_, ndt_step_size_, ndt_epsilon_, ndt_max_iterations_, ndt_num_threads_);
    }

    void createCudaNDT() {
#ifdef NDT_CUDA_ENABLED
        cuda_ndt_ = std::make_shared<CudaNormalDistributionsTransform>();
        cuda_ndt_->setResolution(ndt_resolution_);
        cuda_ndt_->setDistanceMode(fast_gicp::NDTDistanceMode::D2D);
        cuda_ndt_->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
        cuda_ndt_->setTransformationEpsilon(ndt_epsilon_);
        cuda_ndt_->setMaximumIterations(ndt_max_iterations_);
        cuda_ndt_->setInputTarget(global_map_);
        RCLCPP_INFO(
            this->get_logger(),
            "CUDA NDT parameters: resolution=%f, epsilon=%f, max_iter=%d",
            ndt_resolution_, ndt_epsilon_, ndt_max_iterations_);
#else
        createNDT();
#endif
    }
    
    void createMultiResolutionMaps() {
        // 清理旧的NDT对象列表
        ndt_vector_.clear();
        
        // 创建不同分辨率的NDT对象
        for (const auto& resolution : ndt_resolutions_) {
            auto ndt = std::make_shared<NormalDistributionsTransform>();
            ndt->setResolution(resolution);
            ndt->setStepSize(ndt_step_size_);
            ndt->setTransformationEpsilon(ndt_epsilon_);
            ndt->setMaximumIterations(ndt_max_iterations_);
            ndt->setInputTarget(global_map_);
            
            // 设置OpenMP线程数
            ndt->setNumThreads(ndt_num_threads_);
            
            // 设置邻域搜索方法
            ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
            
            ndt_vector_.push_back(ndt);
            RCLCPP_INFO(this->get_logger(), "Added NDT at resolution: %f with %d threads", resolution, ndt_num_threads_);
        }
        RCLCPP_INFO(this->get_logger(), "Created %zu multi-resolution NDT objects", ndt_vector_.size());
    }

    void createMultiResolutionCudaMaps() {
#ifdef NDT_CUDA_ENABLED
        cuda_ndt_vector_.clear();
        for (const auto& resolution : ndt_resolutions_) {
            auto ndt = std::make_shared<CudaNormalDistributionsTransform>();
            ndt->setResolution(resolution);
            ndt->setDistanceMode(fast_gicp::NDTDistanceMode::D2D);
            ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
            ndt->setTransformationEpsilon(ndt_epsilon_);
            ndt->setMaximumIterations(ndt_max_iterations_);
            ndt->setInputTarget(global_map_);
            cuda_ndt_vector_.push_back(ndt);
            RCLCPP_INFO(
                this->get_logger(),
                "Added CUDA NDT at resolution: %f", resolution);
        }
        RCLCPP_INFO(
            this->get_logger(),
            "Created %zu multi-resolution CUDA NDT objects",
            cuda_ndt_vector_.size());
#else
        createMultiResolutionMaps();
#endif
    }

    void publishMapCallback() {
        publishMap();
    }
    
    void publishMap() {
        if(global_map_->empty()) {
            RCLCPP_WARN(this->get_logger(), "Cannot publish empty map");
            return;
        }
        
        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*global_map_, map_msg);
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = global_frame_id_;
        map_pub_->publish(map_msg);
        
        if (debug_mode_) {
            RCLCPP_INFO(this->get_logger(), "Published global map with %zu points to topic: %s", 
                       global_map_->size(), map_topic_.c_str());
        }
    }
    
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        if (msg->header.frame_id != global_frame_id_) {
            RCLCPP_WARN(this->get_logger(), "Initial pose must be in %s frame, got %s", 
                      global_frame_id_.c_str(), msg->header.frame_id.c_str());
            return;
        }
        
        initial_guess_ = Eigen::Matrix4f::Identity();
        
        // 提取平移部分
        initial_guess_(0, 3) = msg->pose.pose.position.x;
        initial_guess_(1, 3) = msg->pose.pose.position.y;
        initial_guess_(2, 3) = msg->pose.pose.position.z;
        
        // 提取旋转部分
        Eigen::Quaternionf q(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        );
        
        initial_guess_.block<3, 3>(0, 0) = q.toRotationMatrix();
        has_initial_guess_ = true;
        initial_guess_is_base_pose_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Received initial pose: [%f, %f, %f]", 
                   initial_guess_(0, 3), initial_guess_(1, 3), initial_guess_(2, 3));
    }

    Eigen::Matrix4f transformToMatrix(
        const geometry_msgs::msg::Transform & transform) const
    {
        Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
        const Eigen::Quaternionf rotation(
            transform.rotation.w,
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z);
        result.block<3, 3>(0, 0) = rotation.normalized().toRotationMatrix();
        result(0, 3) = transform.translation.x;
        result(1, 3) = transform.translation.y;
        result(2, 3) = transform.translation.z;
        return result;
    }
    
    // 多分辨率NDT配准方法
    Eigen::Matrix4f performMultiResolutionNDT(const PointCloud::Ptr& source_cloud, const Eigen::Matrix4f& initial_guess) {
        if (ndt_vector_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No NDT objects available for multi-resolution alignment");
            return initial_guess;
        }
        
        // 准备不同分辨率的点云
        std::vector<PointCloud::Ptr> downsampled_clouds;
        
        for (size_t i = 0; i < ndt_vector_.size(); i++) {
            float leaf_size = static_cast<float>(ndt_resolutions_[i]) / 10.0f; // 根据分辨率设置下采样大小
            PointCloud::Ptr downsampled_cloud(new PointCloud);
            
            pcl::ApproximateVoxelGrid<PointT> approx_voxel_grid;
            approx_voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
            approx_voxel_grid.setInputCloud(source_cloud);
            approx_voxel_grid.filter(*downsampled_cloud);
            
            downsampled_clouds.push_back(downsampled_cloud);
            
            if (debug_mode_) {
                RCLCPP_INFO(this->get_logger(), "Level %zu: resolution=%f, leaf_size=%f, points=%zu", 
                             i, ndt_resolutions_[i], leaf_size, downsampled_cloud->size());
            }
        }
        
        // 从粗到细执行NDT配准
        Eigen::Matrix4f result_transform = initial_guess;
        double final_score = 0.0;
        
        for (size_t i = 0; i < ndt_vector_.size(); i++) {
            auto& ndt = ndt_vector_[i];
            ndt->setInputSource(downsampled_clouds[i]);
            
            pcl::PointCloud<PointT> aligned_cloud;
            ndt->align(aligned_cloud, result_transform);
            
            if (ndt->hasConverged()) {
                result_transform = ndt->getFinalTransformation();
                final_score = ndt->getFitnessScore();
                
                if (debug_mode_) {
                    RCLCPP_INFO(this->get_logger(), "Level %zu: score=%f, iterations=%d", 
                                 i, ndt->getFitnessScore(), ndt->getFinalNumIteration());
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "NDT at level %zu did not converge", i);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Multi-resolution NDT final score: %f", final_score);
        return result_transform;
    }

    Eigen::Matrix4f performMultiResolutionCudaNDT(
        const PointCloud::Ptr& source_cloud,
        const Eigen::Matrix4f& initial_guess) {
#ifdef NDT_CUDA_ENABLED
        if (cuda_ndt_vector_.empty()) {
            RCLCPP_WARN(
                this->get_logger(),
                "No CUDA NDT objects available for multi-resolution alignment");
            return initial_guess;
        }

        Eigen::Matrix4f result_transform = initial_guess;
        double final_score = 0.0;

        for (size_t i = 0; i < cuda_ndt_vector_.size(); ++i) {
            const float leaf_size =
                static_cast<float>(ndt_resolutions_[i]) / 10.0f;
            PointCloud::Ptr downsampled_cloud(new PointCloud);
            pcl::ApproximateVoxelGrid<PointT> voxel_grid;
            voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
            voxel_grid.setInputCloud(source_cloud);
            voxel_grid.filter(*downsampled_cloud);

            auto& ndt = cuda_ndt_vector_[i];
            ndt->setInputSource(downsampled_cloud);
            PointCloud aligned_cloud;
            ndt->align(aligned_cloud, result_transform);

            if (ndt->hasConverged()) {
                result_transform = ndt->getFinalTransformation();
                final_score = ndt->getFitnessScore();
                if (debug_mode_) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "CUDA level %zu: resolution=%f, points=%zu, score=%f",
                        i, ndt_resolutions_[i],
                        downsampled_cloud->size(), final_score);
                }
            } else {
                RCLCPP_WARN(
                    this->get_logger(),
                    "CUDA NDT at level %zu did not converge", i);
            }
        }

        if (debug_mode_) {
            RCLCPP_INFO(
                this->get_logger(),
                "Multi-resolution CUDA NDT final score: %f", final_score);
        }
        return result_transform;
#else
        return performMultiResolutionNDT(source_cloud, initial_guess);
#endif
    }
    
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        const std::string cloud_frame = msg->header.frame_id.empty()
            ? input_cloud_frame_
            : msg->header.frame_id;

        // NDT配准的是雷达点云，先读取URDF中的base_link -> velodyne外参。
        Eigen::Matrix4f base_to_cloud = Eigen::Matrix4f::Identity();
        if (cloud_frame != robot_frame_id_) {
            try {
                const auto transform = tf_buffer_->lookupTransform(
                    robot_frame_id_, cloud_frame, msg->header.stamp);
                base_to_cloud = transformToMatrix(transform.transform);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(
                    this->get_logger(), "无法获取 %s -> %s 外参：%s",
                    robot_frame_id_.c_str(), cloud_frame.c_str(), ex.what());
                return;
            }
        }

        // 1. 转换 ROS2 PointCloud2 -> PCL PointCloud
        PointCloud::Ptr cloud(new PointCloud);
        pcl::fromROSMsg(*msg, *cloud);

        // 2. 预处理点云：移除无效点并进行统计滤波和体素下采样
        PointCloud::Ptr filtered_cloud(new PointCloud);
        
        // 移除 NaN 和无穷大点
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *filtered_cloud, indices);
        
        // 统计滤波，去除离群点
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(filtered_cloud);
        sor.setMeanK(50);  // 考虑50个最近邻点
        sor.setStddevMulThresh(1.0);  // 标准差阈值
        PointCloud::Ptr cleaned_cloud(new PointCloud);
        sor.filter(*cleaned_cloud);
        
        // 体素下采样以减少点数，提高效率
        pcl::VoxelGrid<PointT> voxel_grid;
        voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel_grid.setInputCloud(cleaned_cloud);
        PointCloud::Ptr downsampled_cloud(new PointCloud);
        voxel_grid.filter(*downsampled_cloud);

        // 检查点云是否为空
        if (downsampled_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Filtered point cloud is empty!");
            return;
        }

        if (debug_mode_) {
            RCLCPP_INFO(this->get_logger(), "Point cloud processing: original=%zu, filtered=%zu, final=%zu",
                        cloud->size(), cleaned_cloud->size(), downsampled_cloud->size());
        }

        // 3. 执行 NDT 配准
        Eigen::Matrix4f final_transform;
        
        try {
            // 准备初始猜测
            Eigen::Matrix4f init_guess;
            
            if (has_initial_guess_) {
                init_guess = initial_guess_;
                if (initial_guess_is_base_pose_) {
                    // /initialpose描述map -> base_link，NDT需要map -> 雷达。
                    init_guess = init_guess * base_to_cloud;
                }
                if (debug_mode_) {
                    RCLCPP_INFO(this->get_logger(), "Using initial guess: [%f, %f, %f]",
                                init_guess(0, 3), init_guess(1, 3), init_guess(2, 3));
                }
            } else {
                // 如果没有初始猜测，使用单位矩阵
                init_guess = Eigen::Matrix4f::Identity();
                if (debug_mode_) {
                    RCLCPP_INFO(this->get_logger(), "No initial guess, using identity matrix");
                }
            }

            // 根据配置选择使用单一分辨率或多分辨率NDT
            if (use_multi_scale_ndt_) {
                // 执行多分辨率NDT配准
                if (registration_backend_ == "cuda") {
                    final_transform =
                        performMultiResolutionCudaNDT(downsampled_cloud, init_guess);
                } else {
                    final_transform =
                        performMultiResolutionNDT(downsampled_cloud, init_guess);
                }
            } else {
                // 执行传统单一分辨率NDT配准
                if (registration_backend_ == "cuda") {
#ifdef NDT_CUDA_ENABLED
                    if (!cuda_ndt_) {
                        RCLCPP_ERROR(
                            this->get_logger(),
                            "Single resolution CUDA NDT object not initialized!");
                        return;
                    }
                    cuda_ndt_->setInputSource(downsampled_cloud);
                    PointCloud output;
                    cuda_ndt_->align(output, init_guess);
                    if (!cuda_ndt_->hasConverged()) {
                        RCLCPP_WARN(this->get_logger(), "CUDA NDT did not converge");
                        return;
                    }
                    final_transform = cuda_ndt_->getFinalTransformation();
                    if (debug_mode_) {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "Single-scale CUDA NDT: score=%f",
                            cuda_ndt_->getFitnessScore());
                    }
#endif
                } else {
                    if (!ndt_) {
                        RCLCPP_ERROR(this->get_logger(), "Single resolution NDT object not initialized!");
                        return;
                    }
                    ndt_->setInputSource(downsampled_cloud);
                    pcl::PointCloud<PointT> output;
                    ndt_->align(output, init_guess);

                    if (!ndt_->hasConverged()) {
                        RCLCPP_WARN(this->get_logger(), "NDT did not converge");
                        return;
                    }

                    final_transform = ndt_->getFinalTransformation();

                    if (debug_mode_) {
                        RCLCPP_INFO(this->get_logger(), "Single-scale NDT: score=%f, iterations=%d",
                                    ndt_->getFitnessScore(), ndt_->getFinalNumIteration());
                    }
                }
            }
        } catch (const pcl::PCLException& e) {
            RCLCPP_ERROR(this->get_logger(), "NDT alignment failed: %s", e.what());
            return;
        }

        // 保存结果用于下一次配准的初始猜测
        initial_guess_ = final_transform;
        has_initial_guess_ = true;
        initial_guess_is_base_pose_ = false;
        
        // 5. 获取odom到base_link的变换
        geometry_msgs::msg::TransformStamped odom_to_base;
        try {
            odom_to_base = tf_buffer_->lookupTransform(
                odom_frame_id_, robot_frame_id_, msg->header.stamp);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", 
                        odom_frame_id_.c_str(), robot_frame_id_.c_str(), ex.what());
            return;
        }
        
        // 6. 计算map到odom的变换
        geometry_msgs::msg::TransformStamped map_to_odom;
        map_to_odom.header.stamp = msg->header.stamp;
        map_to_odom.header.frame_id = global_frame_id_;
        map_to_odom.child_frame_id = odom_frame_id_;
        
        // final_transform是map -> 雷达，结合雷达到base_link的逆外参得到map -> base_link。
        const Eigen::Matrix4f map_to_base =
            final_transform * base_to_cloud.inverse();
        
        // 转换为ROS消息格式
        geometry_msgs::msg::PoseStamped base_in_map;
        base_in_map.header.stamp = msg->header.stamp;
        base_in_map.header.frame_id = global_frame_id_;
        
        // 提取平移
        base_in_map.pose.position.x = map_to_base(0, 3);
        base_in_map.pose.position.y = map_to_base(1, 3);
        base_in_map.pose.position.z = map_to_base(2, 3);
        
        // 提取旋转
        Eigen::Matrix3f rot = map_to_base.block<3, 3>(0, 0);
        Eigen::Quaternionf quat(rot);
        base_in_map.pose.orientation.x = quat.x();
        base_in_map.pose.orientation.y = quat.y();
        base_in_map.pose.orientation.z = quat.z();
        base_in_map.pose.orientation.w = quat.w();
        
        // 发布NDT计算的位姿
        pose_pub_->publish(base_in_map);
        
        // 获取odom中base的位姿
        geometry_msgs::msg::Pose base_in_odom;
        base_in_odom.position.x = odom_to_base.transform.translation.x;
        base_in_odom.position.y = odom_to_base.transform.translation.y;
        base_in_odom.position.z = odom_to_base.transform.translation.z;
        base_in_odom.orientation = odom_to_base.transform.rotation;
        
        // 计算map到odom的变换
        tf2::Transform tf2_base_in_map;
        tf2::Transform tf2_base_in_odom;
        tf2::fromMsg(base_in_map.pose, tf2_base_in_map);
        tf2::fromMsg(base_in_odom, tf2_base_in_odom);
        
        tf2::Transform tf2_map_to_odom = tf2_base_in_map * tf2_base_in_odom.inverse();
        
        // 发布TF
        if (publish_tf_) {
            // 转换为ROS消息格式
            map_to_odom.transform = tf2::toMsg(tf2_map_to_odom);
            tf_broadcaster_->sendTransform(map_to_odom);
            
            if (debug_mode_) {
                RCLCPP_INFO(this->get_logger(), 
                    "Publishing TF: %s -> %s [%f, %f, %f]", 
                    global_frame_id_.c_str(), odom_frame_id_.c_str(),
                    map_to_odom.transform.translation.x,
                    map_to_odom.transform.translation.y,
                    map_to_odom.transform.translation.z);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), 
            "NDT position: [%f, %f, %f]", 
            base_in_map.pose.position.x, 
            base_in_map.pose.position.y, 
            base_in_map.pose.position.z);
    }

    // 成员变量
    std::shared_ptr<NormalDistributionsTransform> ndt_;  // 使用共享指针
    std::vector<std::shared_ptr<NormalDistributionsTransform>> ndt_vector_; // 使用共享指针
#ifdef NDT_CUDA_ENABLED
    std::shared_ptr<CudaNormalDistributionsTransform> cuda_ndt_;
    std::vector<std::shared_ptr<CudaNormalDistributionsTransform>> cuda_ndt_vector_;
#endif
    
    PointCloud::Ptr global_map_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr map_timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // NDT参数
    double ndt_resolution_;
    double ndt_step_size_;
    double ndt_epsilon_;
    int ndt_max_iterations_;
    bool use_multi_scale_ndt_;
    std::vector<double> ndt_resolutions_;
    int ndt_num_threads_;
    std::string registration_backend_;
    int gpu_device_id_;
    
    // 点云处理参数
    double voxel_leaf_size_;
    
    // 初始猜测位姿
    Eigen::Matrix4f initial_guess_;
    bool has_initial_guess_;
    bool initial_guess_is_base_pose_;
    bool use_initial_pose_;
    
    // 参数
    std::string global_frame_id_;
    std::string odom_frame_id_;
    std::string robot_frame_id_;
    std::string input_cloud_frame_;
    std::string map_topic_;
    bool publish_tf_;
    bool debug_mode_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NDTLocalization>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
