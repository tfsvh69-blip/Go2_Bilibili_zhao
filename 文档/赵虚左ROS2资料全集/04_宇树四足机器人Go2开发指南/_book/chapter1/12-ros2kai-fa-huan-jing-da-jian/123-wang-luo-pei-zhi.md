### 1.2.3 网络配置

使用网线连接 Go2 和计算机，使用 **ifconfig **查看网络信息，确认机器人连接到的以太网网卡（例如如图中的enp3s0，以实际为准）。![](/assets/网络连接1.png)接着打开网络设置，找到机器人所连接的网卡，进入 IPv4 ，将 IPv4 方式改为手动，地址设置为192.168.123.99，子网掩码设置为255.255.255.0，完成后点击应用，等待网络重新连接。![](/assets/网络连接2.png)打开 setup.sh 文件

```
sudo gedit ~/unitree_ros2/setup.sh
```

bash 的内容如下：

```
#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/foxy/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp3s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
```

将`foxy`修改为`humble`，并且 `enp3s0` 为 Go2 所连接的网卡名称，根据实际情况修改为对应的网卡名称。在终端中执行：

```
source ~/unitree_ros2/setup.sh
```

至此，即完成 Go2 开发环境的设置。如果不希望每次打开新终端都执行一次 bash 脚本，也可将setup.sh 中的内容写入到 ~/.bashrc中，但是当系统有多个 Ros 环境共存需要注意。

**补充说明**：如果电脑没有连接到机器人，但仍希望能使用 unitree ros2 实现仿真等功能， 可以使用本地回环 "lo" 作为网卡:

```
source ~/unitree_ros2/setup_local.sh # 使用 "lo" 作为网卡
```

或

```
source ~/unitree_ros2/setup_default.sh # 不指定网卡
```

完成上述配置后，建议重启一下电脑再进行测试。



