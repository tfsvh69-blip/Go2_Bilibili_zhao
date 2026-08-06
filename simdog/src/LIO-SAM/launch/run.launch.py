import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import launch_ros

def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="go2_description"
    ).find("go2_description")
   
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    publish_map_to_odom_declare = DeclareLaunchArgument(
        'publish_map_to_odom',
        default_value='true',
        description='由LIO-SAM发布动态map到odom；启动NDT时应设为false')

    print("urdf_file_name : {}".format(xacro_path))

    return LaunchDescription([
        params_declare,
        publish_map_to_odom_declare,
       
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[
                parameter_file,
                {
                    'publishMapToOdom': ParameterValue(
                        LaunchConfiguration('publish_map_to_odom'),
                        value_type=bool
                    )
                }
            ],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='lio_rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])




