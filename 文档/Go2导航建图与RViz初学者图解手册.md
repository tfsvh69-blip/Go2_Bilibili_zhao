# Go2 导航、建图与 RViz 初学者图解手册

> 最后更新：2026-08-12
> 适用环境：Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11、Nav2、CHAMP 以及本仓库 `simdog/` 工作空间。
> 文档目标：看到一个名词、颜色、箭头或状态时，能够回答“它是什么、来自哪里、正常应怎样、异常该查什么”。

## 1. 先建立一张总图

可以把整套系统想象成一个人在陌生房间里行走：

- **传感器**是眼睛和内耳：Velodyne 看四周，D435 看近处，IMU 感知转动和倾斜。
- **里程计 odom** 是“闭着眼数步数”：短时间连续，但走久可能累计误差。
- **SLAM/AMCL/NDT** 是“拿环境特征对地图”：纠正数步数产生的误差。
- **地图**是房屋平面图；**代价地图**是在平面图上再画一圈“不要贴太近”的警戒带。
- **规划器**像导航软件，选择从当前位置到目标点的路线。
- **控制器**像司机，把路线变成前进和转向速度。
- **Collision Monitor** 像最后一道自动刹车，任何上游命令都必须经过它。
- **Gazebo** 是实际发生物理运动的虚拟世界；**RViz** 是读取 ROS 数据后画出的仪表盘。

本项目默认导航数据链如下：

```text
Gazebo 中的 Go2 与传感器
        │
        ├─ /velodyne_points ─ pointcloud_to_laserscan ─ /scan
        ├─ /depth/color/points
        ├─ /imu/data
        └─ /odom/ground_truth ─ go2_simulation_odom ─ /odom + odom→base_footprint
                                                        │
在线建图：slam_toolbox ───────────── map→odom + /map ────┤
固定地图：map_server + AMCL ───────── map→odom + /map ───┤
                                                        ▼
目标 → goal_guard → Nav2 规划器 → 控制器 → twist_mux
   → velocity_smoother → collision_monitor → /cmd_vel → CHAMP → 四条腿
```

最重要的排障顺序是：

```text
传感器有没有数据
  → TF 是否完整且只有一个发布者
  → 地图是否正确
  → 定位是否可信
  → 路径是否生成
  → 速度是否连续穿过安全链
  → Gazebo 中实体是否真的运动
```

不要一看到狗不动就先改目标容差，也不要一看到 RViz 跳动就先改控制器。上游定位错误时，控制器越努力，实体动作反而越奇怪。

## 2. Gazebo 和 RViz 到底有什么区别

| 名词 | 实际含义 | 初学者比喻 | 本项目中怎么看 |
|---|---|---|---|
| Gazebo | 带重力、碰撞、关节和传感器的物理仿真器 | 虚拟试验场 | 狗是否真的迈腿、碰墙、摔倒，以 Gazebo 为准 |
| RViz2 | ROS 数据可视化工具，本身不计算物理 | 汽车仪表盘和地图屏幕 | 地图、TF、路径、点云、定位估计都在这里显示 |
| Gazebo GUI | `gzclient` 图形窗口 | 试验场的监控摄像头 | 本项目导航/建图入口默认开启；性能测试可用 `gui:=false` |
| `gzserver` | Gazebo 仿真和物理计算后端 | 试验场本身 | 没有 GUI 时它仍可运行 |
| Fixed Frame | RViz 所有图形共同参考的坐标系 | 把所有透明胶片对齐所用的底板 | 导航必须用 `map`，不要随意改成 `odom` |

判断“跳”的来源：

- **只有 RViz 中的狗跳，Gazebo 实体位置连续**：优先检查定位、`map -> odom` 和 TF。
- **Gazebo 实体也左右乱走**：检查错误定位是否仍允许 Nav2 输出速度，以及 `/cmd_vel` 是否有多个发布者。
- **两边都卡顿但没有位置突变**：检查 CPU 负载、Gazebo real-time factor、传感器频率。

## 3. 读懂这次 RViz 截图

### 3.1 黄色扇形和紫色椭圆

![AMCL 协方差标注图](images/rviz_guide/amcl_covariance_annotated.svg)

上图来自本次真实运行截图。这里的图形属于 RViz 的 `PoseWithCovariance` 显示，订阅 `/amcl_pose`。

| 画面元素 | 具体数据 | 正常含义 | 异常含义 |
|---|---|---|---|
| 紫色椭圆 | 6×6 pose covariance 中 x/y 的方差 | AMCL 认为位置落在一个较小范围内 | 椭圆变大表示位置越来越拿不准 |
| 黄色扇形/梯形/长带 | yaw 方差 | AMCL 认为朝向只在一个小角度内波动 | 横穿地图表示航向几乎失去判断能力 |
| 橙色箭头 | 估计位姿的朝向 | 应与 Gazebo 中狗头方向基本一致 | 指向错误会让规划和控制建立在错误朝向上 |
| 绿色曲线 | `Raw Global Plan=/plan` | 规划器刚算出的路线 | 路径起点随定位跳动时，根因通常在定位上游 |
| 蓝色曲线 | `Controller Path (Smoothed)=/received_global_plan` | 实际交给控制器的路线；平滑失败时会安全回退为原始路线 | 会随机器人前进裁掉已走部分，不是路径丢失 |

**协方差 covariance** 不是“误差本身”，而是算法对自己不确定程度的估计。可以把它理解为天气预报的降雨范围：范围越大，不代表一定在边缘下雨，而是预报员越拿不准。

方差的单位不直观，所以实际排障使用标准差：

```text
标准差 = sqrt(方差)
```

本次现场读到：

```text
std(x)   ≈ 0.99 m
std(y)   ≈ 1.31 m
std(yaw) ≈ 1.72 rad ≈ 99°
```

这等于 AMCL 在说：“我连狗朝哪边都基本不知道。”这不是雷达视野，也不是速度死区。

![较小时的 AMCL 协方差](images/rviz_guide/amcl_covariance_small.png)

上图中的扇形和椭圆比较小，表示相对更有把握。注意“小”不自动等于“绝对正确”；还要确认机器人模型与 Gazebo 实体处在同一个真实位置和朝向。

![定位失信时的协方差](images/rviz_guide/amcl_covariance_lost.png)

上图中紫色椭圆和黄色长带已经明显扩大。本项目安全监督当前采用带滞回的宽松联调阈值：

- 位置标准差超过 `0.75 m`，或 yaw 标准差超过 `0.75 rad`：锁速并拒绝新目标。
- 回落到位置 `0.55 m` 且 yaw `0.50 rad` 以内：才恢复可信。

![极端航向失信](images/rviz_guide/amcl_covariance_extreme.png)

上图黄色长带贯穿整张地图，是典型的 AMCL 航向失信。它看起来像一束探照灯，但实际不是任何传感器光束。

遇到这种画面时：

```bash
# 1. 先保证不再执行旧目标
ros2 service call /navigation/stop std_srvs/srv/Trigger "{}"

# 2. 查看实际协方差
ros2 topic echo --once /amcl_pose

# 3. 在 RViz 使用 2D Pose Estimate 重新给出真实位置和狗头朝向

# 4. 健康后再解除人工锁；旧目标不会自动恢复
ros2 service call /navigation/resume std_srvs/srv/Trigger "{}"
```

### 3.2 一张完整的固定地图导航窗口

![固定地图导航状态](images/rviz_guide/static_navigation_status.png)

这张截图需要逐块理解：

| 位置/文字 | 含义 | 这张图反映什么 |
|---|---|---|
| `Fixed Frame: OK` | RViz 能用固定坐标系变换画图 | 只说明当前所需 TF 可查询，不代表定位一定准确 |
| `Static Map` | `map_server` 发布的 `/map` | 固定地图不会随机器人继续扩展 |
| `Navigation: active` | Nav2 导航生命周期节点处于 active | 导航服务器已启动，不等于当前目标一定能成功 |
| `Localization: inactive` | RViz 未看到定位生命周期管理器处于 active | 固定 AMCL 模式下是不正常项；在线 SLAM 模式则不应拿它判断 Slam Toolbox |
| `Feedback: aborted` | 最近一次 NavigateToPose action 失败终止 | 需要看日志中的具体失败原因，不能仅凭 `aborted` 猜参数 |
| `Distance remaining: 0.00 m` | 当前没有有效剩余路径，或 action 已结束 | `aborted` 后为零不等于到达成功 |
| 地图外深灰区 | OccupancyGrid 未知/地图外区域 | 固定图不能在这里安全选目标 |
| 白/浅灰区域 | 地图中的自由栅格 | 候选目标仍要满足障碍余量 |
| 黑色像素 | 被地图判为占用的栅格 | 连续墙线通常合理，空地孤立点通常是噪声 |

**Navigation 2 面板状态不是总健康结论。** 在线 `online_slam` 模式由 Slam Toolbox 负责定位和建图，Nav2 面板可能显示 `Localization: inactive`，这不等价于 SLAM 故障。固定 `static_map + amcl` 模式下，`Localization: active` 才是预期状态。

### 3.3 为什么固定地图只能在已有范围里选目标

`map_server` 像把一张已经打印好的纸质平面图摊在桌上。机器人后来看到的新房间不会自动印到这张纸上。Velodyne、D435、Local Costmap 即使都有数据，也不会让静态 `/map` 变大。

- 想边走边扩图：使用 `navigation_mode:=online_slam`。
- 想在已经保存的图上稳定导航：使用 `navigation_mode:=static_map localization:=amcl`。
- 地图外目标被拒绝是安全门禁的预期行为，不是 RViz 鼠标坏了。

### 3.4 为什么会“又看到以前那张图”

固定地图模式不会自动判断哪张图质量最好，它只按启动参数 `map_dir` 找
`map.yaml`，再由 `map.yaml` 读取同目录的 `map.pgm`。这就像给打印机明确指定了
一份文件：指定旧文件，它就会忠实打印旧内容。

当前电脑上的目录必须这样区分：

| 目录 | 当前含义 | 固定 AMCL 是否推荐 |
|---|---|---|
| `~/go2_maps/home_01` | LIO-SAM 三维 PCD 投影出的旧二维图，范围过大且伪障碍多 | 否 |
| `~/go2_maps/latest` | LIO-SAM/NDT 地图流程使用的目录 | 不作为 AMCL 默认图 |
| `~/go2_maps/online/home_02` | 2026-08-12 保存的 Slam Toolbox 二维会话 | 是，当前可复现会话 |
| `~/go2_maps/online/latest` | `save_online_map.sh` 更新的软链接，指向最近保存的在线会话 | 是，日常推荐 |

固定地图推荐命令：

```bash
ros2 launch go2_navigation simulation_navigation.launch.xml \
    navigation_mode:=static_map localization:=amcl \
    map_dir:=$HOME/go2_maps/online/latest gui:=true rviz:=true
```

启动后不要只凭 RViz 画面猜测，直接查询 `map_server` 实际读取的文件：

```bash
ros2 param get /map_server yaml_filename
```

当前预期路径应解析到 `~/go2_maps/online/home_02/map.yaml`。如果参数正确但 RViz
仍保留旧画面，通常是旧 RViz 尚未退出，或新的 `map_server` 已经掉线而 RViz 仍显示
最后一次收到的地图。先关闭所有旧导航/RViz 终端，只启动一套入口，再检查：

```bash
ros2 lifecycle get /map_server
ros2 topic info -v /map
```

`map_server` 应为 `active [3]`，`/map` 应只有当前 `map_server` 发布。RViz 默认关闭
`Velodyne Points`、`Global Costmap` 和 `Local Costmap`；按需勾选这些动态层后看到的
散点或膨胀色块不是静态 `map.pgm` 本身。判断底图是否有噪声，应直接查看对应
`map.pgm`，或只保留 `Static Map` Display。

在线模式的 `map_session:=new` 含义不同：它创建空白 pose graph，不会加载上述任意
旧地图；只有显式传入会话目录时才会续建：

```bash
# 从空图重新探索
ros2 launch go2_navigation simulation_navigation.launch.xml \
    navigation_mode:=online_slam map_session:=new gui:=true rviz:=true

# 续建最近一次在线会话
ros2 launch go2_navigation simulation_navigation.launch.xml \
    navigation_mode:=online_slam \
    map_session:=$HOME/go2_maps/online/latest gui:=true rviz:=true
```

## 4. 读懂地图、点云和黑色散点

### 4.1 二维栅格地图 OccupancyGrid

二维地图是规则小格组成的棋盘，每格保存“被占用的可能性”：

| 常见显示 | 栅格语义 | 比喻 |
|---|---|---|
| 白色/浅灰 | free，自由空间 | 已确认可以放脚的地板 |
| 黑色 | occupied，占用 | 墙、家具或被误判为障碍的噪点 |
| 灰色 | unknown，未知 | 还没看过，不能当作必然安全 |

![噪声较多的二维地图](images/rviz_guide/noisy_2d_map.png)

这张图里连续、笔直的黑线像墙，基本合理；房间内部大量孤立黑点和右侧密集“芝麻点”不合理。它们会造成三个连锁问题：

1. 目标门禁认为许多空地不可用。
2. 全局规划器绕来绕去，甚至找不到路径。
3. 激光实时观测与错误地图不匹配，AMCL 粒子分散并可能跳到另一处相似结构。

本次旧 `home_01` 图来自三维 LIO-SAM PCD 的简单二维投影：地面、墙面、家具和不同高度的点挤进同一格，再加上远处离群点，形成大量假障碍。它不再作为 AMCL 默认地图。AMCL 推荐使用 Slam Toolbox 根据同一 `/scan` 原生生成的 `map.yaml + map.pgm`。

### 4.2 三维点云 PCD

![LIO-SAM 三维点云](images/rviz_guide/lio_sam_3d_map.png)

这张彩色图是三维点云，不是二维导航地图：

- 每个彩色点都有 x/y/z，高处墙面、低处地面和家具可以同时存在。
- 颜色通常由高度、强度或显示器的颜色变换产生；它不自动表示“红色危险、蓝色安全”。
- 点云看起来轮廓漂亮，不代表直接压扁成二维图就适合 AMCL。

比喻：三维 PCD 像一栋房子的立体扫描模型；二维 OccupancyGrid 像消防疏散平面图。把立体模型里所有楼层和天花板直接压到一张纸上，平面图自然会布满黑点。

| 地图文件 | 内容 | 主要消费者 | 能否直接互换 |
|---|---|---|---|
| `map.pgm` | 二维灰度栅格图片 | `map_server`、AMCL、Nav2 | 不能替代 PCD |
| `map.yaml` | PGM 路径、分辨率、原点和阈值 | `map_server` | 与对应 PGM 配套 |
| `slam.posegraph` | Slam Toolbox 位姿图 | Slam Toolbox 续建/定位 | 不是给 AMCL 直接读取的图片 |
| `slam.data` | Slam Toolbox 序列化传感器/图数据 | Slam Toolbox | 与 posegraph 配套 |
| `GlobalMap.pcd` | 三维点集合 | NDT/GICP、点云查看 | 不能直接当二维栅格 |
| `map_bundle.yaml` | PCD/PGM/YAML 同源性与哈希清单 | NDT 实验档门禁 | AMCL 原生二维图不要求它 |

### 4.3 地图、定位地图和代价地图不要混淆

| 名词 | 会不会随观测变化 | 作用 |
|---|---|---|
| Static Map | 不会 | 保存好的长期二维环境 |
| Live SLAM Map | 会扩展/修正 | 正在学习中的二维环境 |
| Localization Map `/global_map` | 通常固定 | NDT 用来对齐实时三维点云的 PCD |
| Global Costmap | 动态更新 | 在整张任务区域叠加实时障碍和膨胀代价 |
| Local Costmap | 动态滚动 | 机器人附近短距离避障窗口 |

## 5. Slam Toolbox 面板图解

![Slam Toolbox 面板](images/rviz_guide/slam_toolbox_panel.png)

| 控件 | 实际作用 | 使用建议 |
|---|---|---|
| `Interactive Mode` | 允许交互式调整图优化约束 | 初学阶段保持默认，避免误拖子图 |
| `Accept New Scans` | 是否接收新激光并继续建图 | 建图时必须允许 |
| `Clear Changes` | 清除尚未保存的交互修改 | 不是“清空整张地图”按钮 |
| `Save Changes` | 保存交互修改 | 仅在明确做过图编辑时用 |
| `Save Map` | 保存二维栅格 | 项目更推荐统一保存脚本，避免漏掉 pose graph |
| `Serialize Map` | 保存可续建的 pose graph/data | 续建需要它，不只是 PGM/YAML |
| `Deserialize Map` | 加载序列化会话 | 用于继续已有会话 |
| `Start At Dock` | 从记录的 dock 位姿开始 | 有可靠 dock 定义时用 |
| `Start At Pose Est.` | 从给定 x/y/θ 开始 | 已知地图内初始位姿时用 |
| `Start At Curr. Odom` | 用当前 odom 作为起点 | 续建且 odom 原点关系明确时用 |
| `Localize` | 加载地图后只定位 | 不再扩图的 Slam Toolbox 定位档 |
| `Clear Measurement Queue` | 清掉等待处理的激光 | 传感器队列积压或恢复联调时使用 |
| `Add Submap / Generate Map` | 合并或生成地图 | 高级多会话流程，普通采图暂不使用 |

项目保存命令：

```bash
bash simdog/src/go2_navigation/scripts/save_online_map.sh learning_room
```

![地图保存终端](images/rviz_guide/map_save_terminal.png)

终端中 `waiting for service to become available...` 表示脚本正在等待保存服务被 ROS 发现，不等于保存已经成功。必须等它继续输出成功路径和文件，再关闭建图节点。

## 6. ROS 2 基础名词

| 名词 | 含义 | 本项目例子 |
|---|---|---|
| ROS graph | 当前所有节点、话题、服务和 action 的连接关系 | `ros2 node list`、`ros2 topic list` |
| Node 节点 | 一个长期运行、完成单一职责的进程/组件 | `/amcl`、`/controller_server` |
| Topic 话题 | 连续广播数据，发布者不等待接收者答复 | `/scan`、`/odom`、`/cmd_vel` |
| Publisher | 向话题发布消息的一方 | Collision Monitor 发布最终 `/cmd_vel` |
| Subscriber | 订阅话题的一方 | AMCL 订阅 `/scan` |
| Service | 一次请求、一次回复的短操作 | `/navigation/stop` |
| Action | 可持续数秒/分钟、带反馈且可取消的任务 | `/navigate_to_pose` |
| Parameter | 节点启动或运行参数 | `controller_frequency=10.0` |
| Launch | 一次启动一组节点和参数 | `simulation_navigation.launch.xml` |
| Lifecycle | 节点的 unconfigured/inactive/active/finalized 状态机 | `map_server`、AMCL、Nav2 节点 |
| QoS | 话题可靠性、缓存和历史策略 | 激光常用 Best Effort，地图常用 Transient Local |
| DDS | ROS 2 底层发现和传输机制 | 本项目使用 CycloneDDS |
| ROS_DOMAIN_ID | 隔离 ROS 图的域编号 | 不同域的节点彼此看不见 |
| Remap | 不改源码，把话题/服务/action 名接到另一名字 | 键盘 `cmd_vel` 重映射到 `/cmd_vel_teleop` |
| Namespace | 给一组名称加前缀，避免冲突 | 本项目主要使用根命名空间 `/` |

三个通信概念的比喻：

- Topic 像广播电台：持续播，没人听也可以播。
- Service 像柜台办事：问一次，答一次。
- Action 像叫车订单：下单后持续看到进度，可以中途取消，最后得到成功/失败结果。

常见 QoS：

- **Reliable**：尽量确保收到，像挂号信；适合地图和控制状态。
- **Best Effort**：来不及就丢旧帧，像直播；适合高频激光，宁愿要新帧也不积压。
- **Transient Local**：新订阅者上线也能拿到最后一份数据；适合静态地图和最后一次 AMCL 位姿。

## 7. TF 和坐标系

**TF** 是“不同尺子之间的换算关系”。机器人身上每个传感器都有自己的坐标尺；TF 让系统知道一个激光点换算到地图上在哪里。

本项目导航主链：

```text
map → odom → base_footprint → base_link → velodyne / d435 / imu / 各腿关节
```

| Frame | 含义 | 是否应随步态上下振动 |
|---|---|---|
| `map` | 长期全局地图坐标系 | 否 |
| `odom` | 连续局部里程计坐标系 | 原点固定，机器人在其中连续移动 |
| `base_footprint` | 机器人投影到地面的二维底盘中心 | z/roll/pitch 在二维导航中应为零 |
| `base_link` | 真实机身中心 | 会随四足步态升降、俯仰和横滚 |
| `velodyne` | VLP-16 光心 | 通过固定外参连接 base_link |
| `d435_*` | 深度相机及光学坐标系 | 光学 frame 的轴约定与机身 frame 不同 |
| `imu_link` | IMU 坐标系 | 用于解释角速度、加速度和姿态 |

### `map -> odom` 为什么最关键

`odom` 保证短期连续，定位算法计算 `map -> odom` 来纠正累计偏差。它像“地图北”和“你数步数的临时坐标”之间的校准旋钮。

- 在线模式：只能由 `slam_toolbox` 发布。
- 固定 AMCL：只能由 `amcl` 发布。
- NDT 实验档：NDT pose 进入二维全局 EKF，由 EKF 发布。
- 同时有两个发布者会互相抢校准旋钮，画面必然跳。

常用检查：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 topic info /tf --verbose
```

| TF 现象 | 通常原因 |
|---|---|
| RViz 红色 `No transform` | 某段 TF 缺失、时间戳不一致或节点未启动 |
| `map -> odom` 突然大跳 | AMCL/NDT 失配或多个发布者竞争 |
| `odom -> base_footprint` 断流 | 里程计适配器停止或仿真时间卡住 |
| 地图、激光、机器人彼此错位 | frame_id、外参、时间或定位错误 |

## 8. 传感器与数据类型

| 名词 | 具体含义 | 本项目话题 |
|---|---|---|
| Velodyne VLP-16 | 16 线三维旋转激光雷达 | `/velodyne_points` |
| PointCloud2 | 带 x/y/z 等字段的三维点集合 | `/velodyne_points`、`/depth/color/points` |
| LaserScan | 按角度排列的一圈二维距离 | `/scan` |
| `pointcloud_to_laserscan` | 从 VLP-16 水平高度切片生成二维扫描 | 输入点云，输出 `/scan` |
| RealSense D435 | RGB-D 深度相机 | `/depth/color/points` 等 |
| IMU | 加速度计和陀螺仪 | `/imu/data` |
| JointState | 关节位置、速度、力 | `/joint_states` |
| Odometry | 位姿和速度的局部连续估计 | `/odom` |
| Ground truth | 仿真引擎直接给出的真值 | `/odom/ground_truth`，只用于仿真闭环基准 |
| Hz | 每秒消息次数 | `/scan` 目标约 10 Hz，重负载下可能降低 |
| Latency | 数据从产生到消费的延迟 | 太大会让控制基于过时障碍和位姿 |
| Timeout | 多久没收到数据就判失效 | 传感器断开后安全锁速依据之一 |

`pointcloud_to_laserscan` 的高度过滤像从一叠立体 CT 切片里只抽取与雷达水平面接近的一层。切得太低会把地面当墙，切得太高会漏掉矮障碍，切得太厚会把多个高度压到同一二维方向。

```bash
ros2 topic hz /velodyne_points
ros2 topic hz /scan
ros2 topic hz /depth/color/points
ros2 topic echo --once /scan
```

## 9. 建图和定位算法名词

| 名词 | 解释 | 在本项目中的位置 |
|---|---|---|
| Mapping | 把传感器观测积累成地图 | Slam Toolbox 或 LIO-SAM |
| Localization | 已知地图时估计机器人位姿 | 固定图默认 AMCL |
| SLAM | 同时做定位与建图 | 在线模式使用 Slam Toolbox |
| Slam Toolbox | ROS 2 成熟二维激光 SLAM | 默认在线扩图 |
| LIO-SAM | LiDAR + IMU 的三维因子图 SLAM | 生成三维 PCD 实验地图 |
| Loop Closure | 认出“又回到了走过的地方”并整体校正地图 | 像把画歪的闭合路线重新扣上 |
| Scan Matching | 把当前激光形状与地图/前一帧对齐 | Slam Toolbox、AMCL、NDT 都涉及不同形式的匹配 |
| AMCL | 自适应蒙特卡洛定位，使用大量粒子表示可能位姿 | 固定二维图默认后端 |
| Particle | AMCL 的一个“机器人可能在这里”的假设 | 粒子聚成一团表示更确定 |
| Initial Pose | 给定位算法的初始大概位置和朝向 | RViz `2D Pose Estimate` 发布 `/initialpose` |
| NDT | 把点云空间划格并用概率分布做配准 | 三维固定地图实验后端 |
| GICP | 融合点到点/平面协方差的点云配准方法 | `lidar_localization_ros2` 可选方案 |
| Fitness | 点云配准误差/质量指标 | 通常越小越好，但阈值必须与实现和地图实测匹配 |
| Relocalization | 定位丢失后重新寻找全局位姿 | 不等于普通短期跟踪 |
| EKF | 扩展卡尔曼滤波器，融合连续运动和带噪观测 | NDT 实验档二维 `map -> odom` 平滑 |
| `two_d_mode` | 强制只估计 x/y/yaw | 防止机身步态 z/roll/pitch 污染二维全局 TF |
| Beam Skipping | AMCL 忽略少量与地图不一致的激光束 | 动态物体/孤立噪点不拖走整批粒子 |

AMCL 的粒子可以想成一群侦察员：每个侦察员猜一个位置，拿那里应该看到的墙与真实激光对比。匹配好的侦察员获得更多“后代”，匹配差的逐渐消失。如果地图有大量相似走廊或假黑点，侦察员可能分成几群，甚至集体跑到错误房间。

## 10. Nav2 规划与控制名词

| 名词 | 作用 | 本项目实现 |
|---|---|---|
| Nav2 / Navigation2 | ROS 2 导航框架总称 | 规划、控制、行为树、代价图和生命周期 |
| NavigateToPose | 从当前位姿导航到一个目标位姿的 action | 对外 `/navigate_to_pose` |
| Goal Guard | 目标门禁 | 检查地图、定位、目标安全后转发到底层 action |
| Planner | 在地图上找一条可行路径 | `SmacPlanner2D` |
| Raw Global Plan | 规划器刚算出的整条路线 | RViz 绿色 `/plan` |
| Controller Path (Smoothed) | 实际交给控制器的路线；平滑失败时为原始路线 | RViz 蓝色 `/received_global_plan` |
| Controller | 根据当前位姿追踪路径并输出速度 | 默认 RPP |
| RPP | Regulated Pure Pursuit，受约束纯追踪 | 前向优先，带碰撞预测 |
| Lookahead | 控制器在路径前方选择的追踪点/弧 | 太近易抖，太远易切弯 |
| SmoothPath | Nav2 行为树调用的路径平滑 action | 现在已接到每次全局计划后 |
| SimpleSmoother | 对折线点做快速平滑的 Nav2 插件 | 适合当前 SmacPlanner2D |
| MPPI | 基于采样预测的模型预测路径积分控制器 | `forward_mppi`/`omni_mppi` 对照档 |
| Behavior Tree / BT | 按条件组织规划、跟随、恢复和取消 | `bt_navigator` |
| Recovery | 主导航失败后的恢复行为 | 清图、旋转、后退、等待等 |
| Goal Checker | 判定是否已到目标 | 普通档 0.30 m / 0.25 rad |
| PoseProgressChecker | 判定平移或转向是否取得进展 | 0.10 m 或 0.15 rad 任一达到即刷新 |
| Tolerance | 容许误差 | 不是越小越好；小于四足落足波动会在终点反复修正 |
| ETA | Estimated Time of Arrival，预计到达时间 | 只能作为估计，不是安全保证 |
| Feedback | action 执行中的进度信息 | executing、distance remaining 等 |
| Succeeded | action 成功到达 | 通过 goal checker |
| Canceled | 用户/系统取消 | 与失败不同 |
| Aborted | action 因规划、控制、进度或服务器故障终止 | 必须结合日志找具体原因 |

### 目标不是只有一个点

RViz `Nav2 Goal` 拖出的箭头同时包含：

- x/y：狗最后应该站在哪里。
- yaw：狗最后应该朝哪个方向。

只点击不正确拖方向，可能出现“位置到了但一直转”的现象。普通目标容差 `0.30 m / 0.25 rad` 表示允许站位约差 30 cm、方向约差 14°。

### `Failed to make progress`

它表示 progress checker 在限定时间内没有观察到足够位移，不等于唯一根因是“腿迈不动”。可能原因包括：

- 安全锁或 Collision Monitor 反复把速度归零。
- 定位跳变让控制器不断改方向。
- 路径被假障碍堵住。
- 仿真负载导致控制和传感器长时间断流。
- 速度低于四足接触模型能持续位移的下限。

### 10.1 案例：已经到目标附近，却为了最终朝向来回摆动

肉眼现象：不要求明显转向的目标容易成功；需要在终点改变朝向时，机器人长期停留在约
`0.3 m`，前后/左右调整，随后可能后退恢复，再重新接近。

2026-08-12 现场数据表明，这次不是 AMCL 再次乱跳：

```text
AMCL 标准差：x≈0.011 m、y≈0.029 m、yaw≈0.016 rad（约 0.9°）
目标：(-10.125, -1.635, yaw≈-179°)
机器人旋转期间距目标：0.231 m → 0.263 m → 0.272 m
普通 xy_goal_tolerance：0.25 m
```

机器人围绕 `0.25 m` 边界进进出出。Humble RPP 只有在几何距离小于位置容差时才进入
“线速度归零、原地对准目标 yaw”；一旦四足原地踏步带来几厘米平移、距离又超过
`0.25 m`，它就重新追位置。像把车停进一个直径很小的圆圈：车头刚开始调正，轮胎挪动
又让车身越线，于是司机重新挪位置，永远在“停车”和“摆正车头”之间切换。

第二个问题是当前 `SimpleProgressChecker` 只把平移超过 `0.10 m` 算作进展。原地转向即使
yaw 正在持续接近目标，也会在 15 秒后触发 `Failed to make progress`。现场日志随后确实
进入 `BackUp`，所以观察到的 `linear.x=-0.05 m/s` 是恢复后退，不是 RPP 的终点动作。

业内通常按以下层次解决，而不是只把 yaw 容差无限放大：

1. **位置与朝向要有滞回或足够大的接受圈。** 新版 RPP 可锁存“XY 已到达”；
   Humble 1.1.20 没有该 RPP 参数，所以本项目先用实测可重复的 `0.30 m`
   位置容差，避免把四足落足波动当成重新追位置的理由。
2. **进度检查必须承认“转向也是进展”。** 把 `SimpleProgressChecker` 换成上游已有的
   `nav2_controller::PoseProgressChecker`，同时检查平移和 yaw 变化，避免正常原地转向被
   误判卡死并触发 BackUp。
3. **终点进入条件使用符合平台能力的容差和滞回。** 四足原地踏步会带来数厘米位置漂移；
   通用导航常先实测 12 个终点的漂移分布，再把 XY 容差设为“可重复达到的范围”，而不是
   追求纸面上很小的数字。Go2 仿真建议先对照 `0.30 m` 与 `0.35 m`，yaw 仍保留
   `0.25 rad`，一次只改一个值并记录最终误差。
4. **标定最小连续角速度。** 分别测试 `0.15/0.20/0.25/0.30 rad/s` 的原地旋转；选择能够
   持续克服 CHAMP/Gazebo 接触阻力的最小值再加小余量。角速度过大容易越过目标后反向，
   太小则腿在动但机身不持续转动。当前 `0.45 rad/s` 应作为待对照值，不直接认定最优。
5. **只有任务不关心最终朝向时才忽略 yaw。** 可使用 `PositionGoalChecker`，或上层只发送
   位置目标。若是充电、对接、面向操作台等强姿态任务，应使用专门 docking/final-alignment
   流程，而不是牺牲通用导航的稳定性换厘米级对接精度。

本项目已实施的优先顺序是：

```text
PoseProgressChecker
  → 0.30 m XY / 0.25 rad yaw 稳定容差
  → RPP 普通弯道连续跟随，只在超过 0.85 rad 时原地对齐
  → CHAMP 最小连续角速度阶梯测试
```

不优先选择：关闭碰撞检测、不断增加 recovery、直接换掉整个 Nav2 控制栈，或把 yaw 容差
放大到任何朝向都算成功。DWB 的 `RotateToGoalCritic`、MPPI 的角度 critic、Smac
Hybrid-A*/State Lattice 是有效替代路线，但对当前能原地转向、主体路径跟随已正常的 Go2
而言，不宜在首轮同时替换整套规划控制链。`RotationShimController` 也是可选上游方案，
但 Humble 1.1.20 在每次 `setPlan()` 时会重置其位置锁存；当行为树以 1 Hz
重规划时，不适合未经专项验证就直接作为本次默认修复。

官方源码依据：Humble RPP 的终点旋转条件直接比较当前距离与 goal checker 的 XY 容差；
`PoseProgressChecker` 会在平移或角度任一取得足够变化时刷新进度；Humble
`RotationShimController` 的 `rotate_to_goal_heading` 可在 XY 到达后执行带加速度和碰撞
检查的原地旋转。新发行版 RPP 又增加了 `stateful` 参数来锁存“XY 已到达”状态，但本机
Humble 1.1.20 的 RPP 没有该运行参数，不能把新版文档字段直接抄进当前 YAML。

### 10.2 案例：路线不圆滑，走一段就原地转一次

这个现象有两层，像“先画路”和“再开车”：

- `SmacPlanner2D` 画出的 `/plan` 决定路线点是否像折线。
- RPP 决定是沿弧线跟路，还是停下原地对齐。

旧配置的 `rotate_to_heading_min_angle=0.35 rad`（约 20°）太小，普通转弯也会
触发“停车转向”。现在基线为 `0.85 rad`（约 49°），并将原本没有
接入行为树的 `SmoothPath(SimpleSmoother)` 接在每次规划后。平滑失败只退回
原始有效路径，不会因为“不够好看”就立即 abort。

RViz 观察方法：

| Display | 颜色/话题 | 如何理解 |
|---|---|---|
| Raw Global Plan | 绿色 `/plan` | 规划器刚画的路 |
| Controller Path (Smoothed) | 蓝色 `/received_global_plan` | 实际交给控制器的路；平滑失败时为原始路线，并会随前进裁掉已走部分 |
| RPP Lookahead Arc | 橙色前视弧 | 控制器这一刻准备怎样转 |

要打开动态参数窗口：

```bash
ros2 launch go2_navigation simulation_navigation.launch.xml \
    navigation_mode:=static_map localization:=amcl \
    map_dir:=$HOME/go2_maps/online/latest tuning_gui:=true
```

在 `rqt_reconfigure` 左侧选 `/controller_server`，右侧搜索 `FollowPath`。一次只改一项：

| 参数 | 基线 | 调大后 | 调小后 |
|---|---:|---|---|
| `desired_linear_vel` | 0.27 m/s | 更快，转弯过冲风险上升 | 更稳，但可能低于连续行走速度 |
| `lookahead_dist` | 0.55 m | 弧线更圆，也更容易切弯 | 贴路更准，可能摇头 |
| `max_lookahead_dist` | 0.80 m | 转向更早 | 转向更贴近弯点 |
| `rotate_to_heading_min_angle` | 0.85 rad | 更少原地转 | 更频繁先对齐再前进 |
| `rotate_to_heading_angular_vel` | 0.35 rad/s | 转得快但可能越过目标 | 更细腻，过小可能克服不了接触阻力 |
| `max_angular_accel` | 1.0 rad/s² | 转向响应快、冲击大 | 启停圆滑、响应慢 |

这些改动只在本次进程内生效，重启就回到 YAML 基线。插件类型、
`controller_frequency`、Collision Monitor、安全监督和锁速参数不属于日常动态调参范围。

## 11. Costmap、Footprint 和碰撞保护

| 名词 | 解释 | 比喻 |
|---|---|---|
| Costmap | 每个栅格保存通过代价，而不只是黑/白 | 道路热力图 |
| Static Layer | 从 `/map` 继承固定墙体 | 印刷好的道路底图 |
| Obstacle Layer | 用实时 `/scan` 标记和清除障碍 | 实时路况 |
| Inflation Layer | 在障碍周围扩出渐变代价 | 墙边的安全缓冲带 |
| Global Costmap | 大范围规划使用 | 城市地图 |
| Local Costmap | 机器人附近滚动窗口 | 前挡风玻璃附近视野 |
| Footprint | 机器人俯视占地多边形 | 狗身体在地面的影子 |
| Lethal Cost | 规划器认为不可穿越 | 实墙/绝对禁区 |
| Unknown | 尚未确认安全或占用 | 没勘察过的区域 |
| Raytracing Clearing | 激光射线穿过的空间用于清除旧障碍 | 手电筒照到空处后擦掉旧标记 |
| Collision Monitor | 规划器之后的独立速度过滤/急停 | 最终自动刹车 |
| Slowdown Zone | 障碍进入后按比例减速 | 黄色警戒区 |
| Stop Zone | 障碍进入后速度归零 | 红色急停线 |

截图中墙边青色、紫色、红色的多圈轮廓来自打开的 costmap 及其代价值着色。**颜色受 RViz `costmap` 色表、透明度以及多个 Display 重叠影响，不能仅凭某一种颜色断言是哪一层。** 要在左侧 Displays 中一次只勾选 `Static Map`、`Global Costmap`、`Local Costmap` 来辨认。

RPP 自带的路径碰撞预测和 Collision Monitor 是两层不同保护：前者帮助控制器不沿危险弧线前进，后者是不依赖规划是否正确的最终速度防线。本项目不通过关闭它们来掩盖地图或传感器问题。

## 12. 速度链与四足执行

```text
Nav2                → /cmd_vel_nav ┐
键盘（正确重映射）  → /cmd_vel_teleop ├→ twist_mux
Unitree Move        → /cmd_vel_unitree┘
    → /cmd_vel_switched
    → velocity_smoother
    → /cmd_vel_smoothed
    → collision_monitor
    → /cmd_vel
    → CHAMP / ros2_control / 12 个关节
```

| 名词 | 作用 |
|---|---|
| `Twist` | 线速度和角速度消息；常见分量是 `linear.x` 与 `angular.z` |
| `cmd_vel` | 速度命令的惯用话题名，不代表命令一定已执行 |
| `twist_mux` | 多路速度命令仲裁，同一时刻选正确来源 |
| `velocity_smoother` | 限制速度和加速度突变，避免命令阶跃 |
| `/pause_navigation` | 安全监督锁，true 时速度链被置零 |
| CHAMP | 把机身速度要求转成四足步态与足端轨迹 |
| `ros2_control` | 控制器管理和关节命令执行框架 |
| Controller Manager | 加载、激活和查询关节控制器 |
| Unitree bridge | 在 Unitree Sport API 消息与仿真控制话题之间转换 |

最终 `/cmd_vel` 必须只有 `collision_monitor` 一个发布者。直接运行默认键盘会绕过安全链并制造第二个发布者；正确命令是：

```bash
source scripts/setup_unitree_sim.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_teleop
```

检查：

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic echo /pause_navigation
ros2 topic hz /cmd_vel
ros2 control list_controllers
```

## 13. RViz Displays 和工具栏词典

### Displays

| 显示项 | 订阅/数据 | 看到什么 | 排障用途 |
|---|---|---|---|
| Grid | RViz 本地网格 | 米制参考格 | 判断尺度，不是地图 |
| RobotModel | `/robot_description` + TF | Go2 模型和腿 | 模型散架通常是关节 TF/状态问题 |
| TF | `/tf`、`/tf_static` | 坐标轴和父子树 | 查缺失、跳动、重复发布 |
| Static Map | `/map` | 固定二维栅格 | 查看 AMCL 使用的地图 |
| Live SLAM Map | `/map`、`/map_updates` | 在线增长地图 | 判断新区域是否真正加入 |
| Global Costmap | `/global_costmap/costmap` | 全局代价 | 查全局路径为什么绕行 |
| Local Costmap | `/local_costmap/costmap` | 近场滚动代价 | 查狗附近假障碍和停走 |
| Velodyne Points | `/velodyne_points` | 三维激光点 | 查原始传感器和外参，默认关闭以降负载 |
| SLAM Scan | `/scan` | 二维激光点 | 查激光是否贴合墙线 |
| Localization Map | `/global_map` | NDT 三维 PCD | 仅 NDT 实验档需要 |
| Raw Global Plan | `/plan` | 绿色原始规划路径 | 查规划器给出的路线是否折线化 |
| Controller Path (Smoothed) | `/received_global_plan` | 蓝色控制器路径 | 对照平滑是否生效；回退时会接近原始路径，正常会裁掉已走部分 |
| RPP Lookahead Arc | `/lookahead_collision_arc` | 追踪/碰撞预测弧 | 查控制器准备怎样转弯 |
| AMCL Pose | `/amcl_pose` | 位姿箭头与协方差 | 固定图 AMCL 健康核心显示 |

### 工具栏

| 工具 | 作用 | 常见误区 |
|---|---|---|
| Interact | 操作可交互标记 | 不用于发送目标 |
| Move Camera | 平移/旋转视角 | 只改变视图，不改变机器人 |
| Select | 选择显示对象 | 不等于选目标 |
| Focus Camera | 让视角聚焦对象 | 不改变 Fixed Frame |
| Measure | 测距 | 不发布导航任务 |
| 2D Pose Estimate | 发布 `/initialpose` | 是告诉 AMCL“狗大概在哪和朝哪”，不是移动狗 |
| Nav2 Goal | 发送导航目标位姿 | 需要在有效地图自由区拖出最终朝向 |
| Publish Point | 发布 `/clicked_point` | 普通单点，不等于 NavigateToPose action |

## 14. Navigation 2 面板词典

| 字段/按钮 | 含义 |
|---|---|
| Navigation: active/inactive | Nav2 导航生命周期管理状态 |
| Localization: active/inactive | 定位生命周期管理状态；在线 SLAM 模式不能单独据此判断 Slam Toolbox |
| Feedback | 最近/当前 action 状态，如 executing、succeeded、canceled、aborted |
| ETA | 预计剩余时间 |
| Distance remaining | 沿路径估计的剩余距离，不是直线尺量 |
| Time taken | 本次 action 已执行时间 |
| Recoveries | 已触发恢复行为次数 |
| Pause | 暂停 Navigation 2 执行逻辑；项目硬停止优先用 `/navigation/stop` |
| Reset | 重置面板/导航状态，不是修复地图或定位的万能按钮 |
| Cancel | 取消当前目标，普通停止首选 |
| Waypoint / Nav Through Poses Mode | 切换单目标与多航点模式 |

状态关系：

- `active + succeeded`：服务器在线且目标成功。
- `active + aborted`：服务器在线，但该目标失败。
- `inactive`：相关 lifecycle 节点没激活、已被 manager 重置，或该模式不使用这套定位 manager。
- `Distance remaining=0` 与 `aborted` 同时出现：action 已结束，不代表成功到达。

## 15. 常见参数的物理意义

| 参数 | 当前基线 | 太小的表现 | 太大的表现 |
|---|---:|---|---|
| `controller_frequency` | 10 Hz | 转向和避障反应迟钝 | 超过传感器/CPU能力会频繁 deadline miss |
| `desired_linear_vel` | 0.27 m/s | 四足可能克服不了接触阻力 | 转弯、制动困难 |
| `min_approach_linear_velocity` | 0.10 m/s | 终点前走走停停 | 可能冲过目标 |
| `xy_goal_tolerance` | 0.25 m | 为最后几厘米反复挪动 | 停得离目标较远 |
| `yaw_goal_tolerance` | 0.25 rad | 终点反复左右摆头 | 最终朝向偏差明显 |
| `inflation_radius` | 0.30 m | 路径贴墙 | 窄通道被全部封死 |
| `source_timeout` | 2.0 s 联调值 | Gazebo 偶发延迟导致误急停 | 真断传感器后停车太慢 |
| AMCL `alpha1..5` | 仿真 0.05 | 过度相信里程计 | 粒子无谓扩散、对称环境易多峰 |
| AMCL `max_beams` | 90 | 环境约束不足 | CPU 增加，收益逐渐变小 |
| `scan_timeout_sec` | 2.0 s | 负载抖动误锁 | 真失效响应变慢 |

这些是 Gazebo 闭环基线，不应直接复制到真机。真机足端滑移、IMU 噪声和机身振动都不同。

## 16. 一眼现象到根因的速查表

| 肉眼现象 | 首先怀疑 | 第一条检查命令/动作 |
|---|---|---|
| 黄色扇形横穿地图 | AMCL yaw 协方差爆炸 | `ros2 topic echo --once /amcl_pose` |
| 紫色椭圆越来越大 | AMCL x/y 不确定度增长 | 看实时 `/scan` 是否贴合 `/map` |
| RViz 狗瞬移，Gazebo 狗没瞬移 | `map -> odom` 跳变 | `ros2 run tf2_ros tf2_echo map odom` |
| Gazebo 狗左右乱移 | 错误定位仍驱动控制，或多路速度竞争 | 先 `/navigation/stop`，再查 `/cmd_vel --verbose` |
| 走一下停一下 | `/pause_navigation`、Collision Monitor、速度断流 | `ros2 topic echo /pause_navigation` |
| 空地有大量黑点 | 地图投影噪声或地面/自体点 | 分别只开 Static Map 和 `/scan` 对比 |
| 路径贴墙/绕很远 | inflation 和障碍层代价 | 单独打开 Global Costmap |
| 目标点点不了地图外 | 固定图/门禁拒绝未知区 | 切在线 SLAM 扩图，或重新采完整地图 |
| `Feedback: aborted` | 规划、控制、进度或节点故障之一 | 看 `ros2 launch` 终端中 action 失败原因 |
| `Localization: inactive` | 固定 AMCL manager 未 active，或当前是在线 SLAM 模式 | 先确认 `navigation_mode` |
| `No transform` 红字 | TF 缺失/过期/时间域错误 | `ros2 run tf2_ros tf2_echo <target> <source>` |
| 激光不贴墙 | 外参、frame、时间、地图或定位错误 | 只开 Static Map + SLAM Scan + TF |
| 地图不再变大 | 当前是 static_map，或 SLAM 不收新 scan | 查 `/map` 发布者和 `/scan` Hz |
| 生命周期节点反复 reset | bond 超时、过载或进程退出 | 看 lifecycle manager 日志和 CPU |

## 17. 推荐启动与健康检查

每个终端先执行：

```bash
cd /home/hao/ROS/Go2_Bilibili_zhao-main
source scripts/setup_unitree_sim.bash
```

默认在线建图导航（默认显示 Gazebo 和 RViz）：

```bash
ros2 launch go2_navigation simulation_navigation.launch.xml
```

固定二维地图 + AMCL：

```bash
ros2 launch go2_navigation simulation_navigation.launch.xml \
  navigation_mode:=static_map \
  localization:=amcl \
  map_dir:=$HOME/go2_maps/online/home_02
```

固定图启动后先用 RViz `2D Pose Estimate` 设置准确位置和朝向，再检查：

```bash
ros2 run go2_navigation health_check \
  --mode static_map \
  --localization amcl \
  --map-dir "$HOME/go2_maps/online/home_02"
```

反斜杠 `\` 必须是该行最后一个字符，后面不能再有空格。

安全停止：

```bash
ros2 service call /navigation/stop std_srvs/srv/Trigger "{}"
```

确认问题已经排除后恢复：

```bash
ros2 service call /navigation/resume std_srvs/srv/Trigger "{}"
```

## 18. 缩写索引

| 缩写 | 全称 | 中文理解 |
|---|---|---|
| AMCL | Adaptive Monte Carlo Localization | 自适应粒子滤波定位 |
| BT | Behavior Tree | 行为树 |
| CUDA | Compute Unified Device Architecture | NVIDIA GPU 计算平台 |
| DDS | Data Distribution Service | ROS 2 底层通信中间件标准 |
| EKF | Extended Kalman Filter | 扩展卡尔曼滤波 |
| GICP | Generalized Iterative Closest Point | 广义迭代最近点配准 |
| IMU | Inertial Measurement Unit | 惯性测量单元 |
| LIO | LiDAR-Inertial Odometry | 激光惯性里程计 |
| MPPI | Model Predictive Path Integral | 模型预测路径积分控制 |
| NDT | Normal Distributions Transform | 正态分布变换点云配准 |
| OMP | Open Multi-Processing | CPU 多线程并行接口 |
| PCD | Point Cloud Data | 点云文件格式 |
| PGM | Portable Graymap | 二维灰度地图图片格式 |
| QoS | Quality of Service | 通信质量策略 |
| RPP | Regulated Pure Pursuit | 受约束纯追踪控制器 |
| RGB-D | Color + Depth | 彩色加深度相机 |
| ROS | Robot Operating System | 机器人软件通信与组件框架 |
| RViz | ROS Visualization | ROS 可视化工具 |
| SLAM | Simultaneous Localization and Mapping | 同时定位与建图 |
| TF | Transform | 坐标变换系统 |
| URDF | Unified Robot Description Format | 机器人模型描述格式 |
| VLP-16 | Velodyne Puck 16-line | 16 线三维激光雷达型号 |
| YAML | YAML Ain't Markup Language | 参数/元数据文本格式 |

## 19. 本项目 ROS 包名速查

ROS **package（包）** 是一组相关源码、配置、启动文件和依赖的发布单元，类似电脑上的一个应用模块。本项目只构建 `simdog/` 工作空间，主要包如下：

| 包名 | 用一句话解释 |
|---|---|
| `go2_navigation` | 本项目导航总装包：模式入口、Nav2 参数、地图工具、目标门禁、安全监督和 RViz 配置 |
| `go2_behaviors` | 打招呼、点头、趴下等仿真行为，并负责与导航互斥 |
| `go2_unitree_sim_bridge` | 把 Unitree Sport API 请求和状态映射到 Gazebo/CHAMP |
| `unitree_ros2_interfaces` | Unitree 官方 `unitree_go`、`unitree_api` 消息接口快照 |
| `lidar_localization_ros2` | 基于 NDT/GICP 的三维定位实验库 |
| `ndt_relocalization` | 另一套读取 PCD 并执行 NDT 重定位的实验节点 |
| `ndt_omp_ros2` | 使用 OpenMP 在 CPU 上加速 NDT |
| `fast_gicp` | 提供 CUDA/CPU GICP 与体素化 NDT 配准能力 |
| `lio_sam` | LIO-SAM 三维激光惯性建图包 |
| `pointcloud_to_laserscan` | 把三维点云的高度切片转换为二维 `/scan` |
| `realsense_ros_gazebo` | RealSense 相机的 Gazebo 模型和插件 |
| `go2_description` | Go2 外观、碰撞模型、关节、传感器 xacro/URDF |
| `go2_config` | Go2 的 Gazebo 世界、CHAMP 参数和主要仿真启动入口 |
| `champ` | CHAMP 四足运动学/步态核心头文件库及元包 |
| `champ_base` | 四足步态生成、状态估计和速度到关节轨迹的核心节点 |
| `champ_bringup` | CHAMP 通用启动编排 |
| `champ_config` | 通用 gait、joints、links 等配置模板 |
| `champ_description` | CHAMP 通用机器人描述和 RViz 资源 |
| `champ_gazebo` | CHAMP 与 Gazebo Classic 的集成 |
| `champ_msgs` | CHAMP 自定义消息定义 |
| `champ_navigation` | 上游 CHAMP 的基础 Nav2 示例；本项目正式导航使用 `go2_navigation` |
| `champ_teleop` | CHAMP 键盘/手柄遥控 |

`build/`、`install/`、`log/` 不是 ROS 包：分别是编译中间文件、安装产物和日志。源码改动应发生在 `simdog/src/`，不要直接修改 `install/` 中的生成副本。

## 20. 文档维护规则

这是一份随项目演进维护的活文档，而不是一次性故障记录：

1. 新增用户可见的 RViz Display、面板、颜色、Marker、路径或状态字段时，在本手册补充“来源话题、正常含义、异常表现”。
2. 新增导航、定位、建图、传感器、控制或安全术语时，同时补充中文解释和至少一个本项目例子。
3. 如果单靠文字容易误解，应优先加入真实截图；截图旁必须说明模式、话题和结论，不能只贴图不解释。
4. 参数修改必须记录物理意义、当前值、过大/过小的可见表现，并区分“已实测”和“待验证”。
5. 颜色可能由 RViz 配置改变，因此文档必须以 Display 名称和话题作为最终依据，不能只靠颜色认图。
6. 真机与 Gazebo 参数必须明确分开；当前手册中的控制和 AMCL 基线只完成仿真验证。

如果后续截图出现本手册没有解释的元素，应把截图、对应话题和解释补进本文件，而不是只在聊天中留下结论。
