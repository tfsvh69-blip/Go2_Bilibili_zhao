#!/usr/bin/env python3
"""从 go2_description 的扁平 URDF 生成 Gazebo「移动平台」版 URDF。

策略(planar-move 路线,面向导航/SLAM/视觉开发):
  1. 把 12 个腿部 revolute 关节改成 fixed —— 整机成为一个刚体,能稳稳站在地面;
  2. 注入 planar_move 插件(订阅 /cmd_vel,发布 /odom 和 odom->base 的 tf);
  3. 注入 ROS2 IMU 传感器;
  4. 给足端设置摩擦系数。
mesh 仍通过 package://go2_description/... 解析,无需拷贝。
"""
import sys
import xml.etree.ElementTree as ET

SRC = sys.argv[1]
DST = sys.argv[2]

tree = ET.parse(SRC)
root = tree.getroot()

# 1) 12 个腿关节 revolute -> fixed
leg_joints = {f'{leg}_{seg}_joint'
              for leg in ('FL', 'FR', 'RL', 'RR')
              for seg in ('hip', 'thigh', 'calf')}
fixed = []
for j in root.findall('joint'):
    if j.get('name') in leg_joints and j.get('type') == 'revolute':
        j.set('type', 'fixed')
        # 去掉 fixed 关节不需要的子元素
        for tag in ('axis', 'limit', 'dynamics'):
            e = j.find(tag)
            if e is not None:
                j.remove(e)
        fixed.append(j.get('name'))

# 2) 追加 Gazebo 相关块(用字符串再解析,便于书写)
gazebo_xml = """
<root>
  <!-- planar_move:把 /cmd_vel 变成底盘平面运动,并发布 odom 与 tf -->
  <gazebo>
    <plugin name="planar_move" filename="libgazebo_ros_planar_move.so">
      <ros>
        <namespace>/</namespace>
        <remapping>cmd_vel:=cmd_vel</remapping>
        <remapping>odom:=odom</remapping>
      </ros>
      <update_rate>50</update_rate>
      <publish_rate>50</publish_rate>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base</robot_base_frame>
      <covariance_x>0.0001</covariance_x>
      <covariance_y>0.0001</covariance_y>
      <covariance_yaw>0.01</covariance_yaw>
    </plugin>
  </gazebo>

  <!-- IMU 传感器(ROS2) -->
  <gazebo reference="imu">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>200</update_rate>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
        <ros>
          <namespace>/</namespace>
          <remapping>~/out:=imu</remapping>
        </ros>
        <frame_name>imu</frame_name>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
    </sensor>
  </gazebo>
</root>
"""
extra = ET.fromstring(gazebo_xml)
for child in list(extra):
    root.append(child)

# 3) 足端设为「无摩擦」——planar-move 是运动学滑行,足端有摩擦会被"钉住"导致整机翻倒。
#    法向靠 kp/kd 支撑高度,水平无摩擦让身体自由滑动、且 cmd_vel=0 时由插件锁定不漂移。
for leg in ('FL', 'FR', 'RL', 'RR'):
    g = ET.SubElement(root, 'gazebo')
    g.set('reference', f'{leg}_foot')
    ET.SubElement(g, 'mu1').text = '0.0'
    ET.SubElement(g, 'mu2').text = '0.0'
    ET.SubElement(g, 'kp').text = '1000000.0'
    ET.SubElement(g, 'kd').text = '100.0'


# 4) 传感器 ---------------------------------------------------------------
def add_link(name, size, color='0.1 0.1 0.1 1'):
    root.append(ET.fromstring(f'''<link name="{name}">
      <inertial><origin xyz="0 0 0"/><mass value="0.05"/>
        <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial>
      <visual><origin xyz="0 0 0"/><geometry><box size="{size}"/></geometry>
        <material name="{name}_mat"><color rgba="{color}"/></material></visual>
    </link>'''))

def add_fixed_joint(name, parent, child, xyz, rpy='0 0 0'):
    root.append(ET.fromstring(f'''<joint name="{name}" type="fixed">
      <parent link="{parent}"/><child link="{child}"/>
      <origin xyz="{xyz}" rpy="{rpy}"/></joint>'''))

# 4a) 2D 激光雷达(顶部,发布 /scan 供 slam_toolbox / nav2)
add_link('laser_link', '0.05 0.05 0.04', '0.9 0.1 0.1 1')
add_fixed_joint('laser_joint', 'base', 'laser_link', '0.0 0.0 0.15')
root.append(ET.fromstring('''<gazebo reference="laser_link">
  <sensor name="lidar_2d" type="ray">
    <always_on>true</always_on><update_rate>10</update_rate><visualize>false</visualize>
    <ray>
      <scan><horizontal><samples>360</samples><resolution>1</resolution>
        <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal></scan>
      <range><min>0.15</min><max>12.0</max><resolution>0.01</resolution></range>
    </ray>
    <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
      <ros><namespace>/</namespace><remapping>~/out:=scan</remapping></ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>laser_link</frame_name>
    </plugin>
  </sensor>
</gazebo>'''))

# 4b) 深度相机(前部,发布 RGB + 深度 + 点云,供视觉/RTAB-Map)
add_link('camera_link', '0.02 0.06 0.02', '0.1 0.1 0.9 1')
add_fixed_joint('camera_joint', 'base', 'camera_link', '0.28 0.0 0.02')
# 光学系:ROS 相机点云约定(z 向前, x 向右, y 向下)
add_fixed_joint('camera_optical_joint', 'camera_link', 'camera_optical_link',
                '0 0 0', '-1.5708 0 -1.5708')
root.append(ET.fromstring('<link name="camera_optical_link"/>'))
root.append(ET.fromstring('''<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth">
    <always_on>true</always_on><update_rate>15</update_rate>
    <camera>
      <horizontal_fov>1.204</horizontal_fov>
      <image><width>640</width><height>480</height><format>R8G8B8</format></image>
      <clip><near>0.1</near><far>10.0</far></clip>
    </camera>
    <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
      <ros><namespace>/</namespace></ros>
      <camera_name>camera</camera_name>
      <frame_name>camera_optical_link</frame_name>
      <min_depth>0.1</min_depth><max_depth>10.0</max_depth>
    </plugin>
  </sensor>
</gazebo>'''))

# 注意:不写 XML 声明。spawn_entity.py 用 lxml 解析,带 encoding 声明的字符串会报错
tree.write(DST, encoding='unicode', xml_declaration=False)
print(f'已生成 {DST}')
print(f'焊死的腿关节({len(fixed)}): {sorted(fixed)}')
