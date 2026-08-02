# RViz 可视化 launch:加载 Go2 URDF,启动 robot_state_publisher + joint_state_publisher(_gui) + rviz2
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('go2_description')
    urdf_path = os.path.join(pkg, 'urdf', 'go2_description.urdf')
    rviz_path = os.path.join(pkg, 'rviz', 'display.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    use_gui = LaunchConfiguration('use_gui')

    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='true=用滑块GUI手动拖12个关节; false=发布全0关节'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description}]),

        # use_gui=true:滑块 GUI
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(use_gui)),

        # use_gui=false:普通 joint_state_publisher(发布全 0)
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(use_gui)),

        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', rviz_path]),
    ])
