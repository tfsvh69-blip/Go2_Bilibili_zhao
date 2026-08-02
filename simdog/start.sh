#!/bin/bash

# 等待时间（秒）
WAIT_TIME=8

# 启动Gazebo仿真环境 3d_to_2d slamtoolbox
# 【防崩】本机 RTX 5070 与 Gazebo Classic 的 gzclient 不兼容(SIGKILL),故 gui:=false 不开 Gazebo 窗口,
#   改用 rviz:=true 在 RViz 里可视化;velodyne 由 gzserver 在 NVIDIA 上渲染(有显示即可)。
gnome-terminal -- bash -c "source install/setup.bash && ros2 launch go2_config gazebo_velodyne.launch.py gui:=false rviz:=true; exec bash"
# 【备选】若上面 /velodyne_points 无数据,改用这行:软件渲染并保留 Gazebo 窗口(慢但稳):
# gnome-terminal -- bash -c "source install/setup.bash && LIBGL_ALWAYS_SOFTWARE=1 ros2 launch go2_config gazebo_velodyne.launch.py rviz:=true; exec bash"
# gnome-terminal -- bash -c "source install/setup.bash && ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py; exec bash"
# gnome-terminal -- bash -c "source install/setup.bash && ros2 launch go2_config slam.launch.py; exec bash"   #启动slam_toolbox建图模式，如果需要定位，可以在配置文件里改，然后将导航的amcl注释掉
# gnome-terminal -- bash -c "source install/setup.bash && ros2 launch go2_config navigate.launch.py; exec bash"  #启动导航 暂时未配好 等待github 更新

sleep $WAIT_TIME

gnome-terminal -- bash -c "source install/setup.bash && ros2 launch lio_sam lidar.launch.py; exec bash"
gnome-terminal -- bash -c "source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard; exec bash"
gnome-terminal -- bash -c "source install/setup.bash && ros2 launch ndt_relocalization ndt_localization.launch.py; exec bash"
