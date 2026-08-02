# 在 Gazebo Classic 中启动 Go2 移动平台:gazebo(world) + robot_state_publisher + spawn
import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('go2_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    urdf_path = os.path.join(pkg_gazebo, 'urdf', 'go2_gazebo.urdf')
    world_path = os.path.join(pkg_gazebo, 'worlds', 'go2.world')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    # 防御性剥掉 XML 声明,避免 spawn_entity.py(lxml)报 encoding 错误
    robot_description = re.sub(r'^\s*<\?xml[^>]*\?>\s*', '', robot_description)

    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    z = LaunchConfiguration('z')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'gui': gui,
            'verbose': 'false',
        }.items(),
    )

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', 'go2', '-topic', 'robot_description',
                   '-x', '0', '-y', '0', '-z', z],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg_gazebo, 'rviz', 'go2_sim.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否开 Gazebo 图形界面(headless 测试设 false)'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='是否同时启动 RViz(rviz:=true)'),
        DeclareLaunchArgument('world', default_value=world_path),
        DeclareLaunchArgument('z', default_value='0.45',
                              description='初始生成高度,略高于站立高度让其落到地面'),
        gazebo,
        rsp,
        spawn,
        rviz,
    ])
