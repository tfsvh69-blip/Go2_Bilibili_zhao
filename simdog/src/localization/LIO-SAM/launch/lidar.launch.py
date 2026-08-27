import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 包路径配置
    lio_sam_dir = get_package_share_directory('lio_sam')
    
    # 2. 参数声明
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(lio_sam_dir, 'config', 'params.yaml'),
        description='Full path to LIO-SAM parameter file'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )

    declare_publish_map_to_odom = DeclareLaunchArgument(
        'publish_map_to_odom',
        default_value='true',
        description='由LIO-SAM发布动态map到odom；启动NDT时应设为false'
    )

    # 4. 节点配置
    nodes = [
        # LIO-SAM 核心节点
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'publishMapToOdom': ParameterValue(
                        LaunchConfiguration('publish_map_to_odom'),
                        value_type=bool
                    )
                }
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        ),

        # RViz2可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='lio_sam_rviz2',
            arguments=['-d', os.path.join(lio_sam_dir, 'config', 'rviz2.rviz')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz'))
        )
    ]

    return LaunchDescription([
        declare_use_sim_time,
        declare_params,
        declare_rviz,
        declare_publish_map_to_odom,
        *nodes
    ])
