# Go2 项目地图目录

所有地图都保存在项目根内，不使用 `~/go2_maps`。两类目录不能混用：

```text
go2_maps/
├── README.md
├── online/                         # Slam Toolbox 二维在线地图
│   ├── my_world_full_v1/           # 一次不可覆盖的命名会话
│   │   ├── map.pgm                 # 固定 AMCL 使用的占据栅格图像
│   │   ├── map.yaml                # 分辨率、原点及图像路径
│   │   ├── slam.posegraph          # 以后续建在线地图
│   │   ├── slam.data               # pose graph 配套数据
│   │   └── session.yaml            # 保存时间和本次雷达参数
│   └── latest -> my_world_full_v1  # 最近成功保存的在线会话
└── latest/                         # LIO-SAM/NDT 三维 PCD 地图，不能给 AMCL 混用
```

保存在线地图时使用便于辨认的 ASCII 名称，建议格式为
`<场景>_<覆盖范围>_v<版本>`，例如：

```bash
bash simdog/src/go2_navigation/scripts/save_online_map.sh my_world_full_v1
```

脚本拒绝覆盖已有同名目录，先在隐藏临时目录生成并检查完整，再原子移动到
`go2_maps/online/<地图名>/`，最后更新 `online/latest`。复现实验或正式导航时优先写明确
目录；`online/latest` 只适合快速打开最近地图。

固定二维地图启动示例：

```bash
ros2 launch go2_navigation simulation_navigation.launch.xml \
  navigation_mode:=static_map localization:=amcl \
  map_dir:=$GO2_PROJECT_ROOT/go2_maps/online/my_world_full_v1 \
  rviz:=true
```
