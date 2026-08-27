"""navigation.rviz 的纯配置校验，不依赖 ROS 图。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def _topic_value(item: dict[str, Any]) -> str | None:
    """读取 RViz 显示项或工具中的嵌套 Topic 值。"""
    topic = item.get("Topic")
    return topic.get("Value") if isinstance(topic, dict) else None


def validate_navigation_rviz(path: str | Path) -> list[str]:
    """返回 navigation.rviz 不符合导航入口约定的原因列表。"""
    try:
        document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        return ["无法读取 RViz 配置：%s" % exc]

    if not isinstance(document, dict):
        return ["RViz 配置根节点必须是 YAML 映射"]
    manager = document.get("Visualization Manager")
    if not isinstance(manager, dict):
        return ["RViz 配置缺少 Visualization Manager"]

    problems: list[str] = []
    global_options = manager.get("Global Options")
    if not isinstance(global_options, dict) or global_options.get("Fixed Frame") != "map":
        problems.append("RViz Fixed Frame 必须为 map")

    displays = manager.get("Displays")
    display_by_name = {
        item.get("Name"): item for item in displays
        if isinstance(item, dict) and isinstance(item.get("Name"), str)
    } if isinstance(displays, list) else {}
    online_mode = Path(path).name == "online_mapping_navigation.rviz"
    map_display_name = "Live SLAM Map" if online_mode else "Static Map"
    expected_displays = {
        map_display_name: ("rviz_default_plugins/Map", "/map"),
        "Local Costmap": ("rviz_default_plugins/Map", "/local_costmap/costmap"),
        "Global Costmap": ("rviz_default_plugins/Map", "/global_costmap/costmap"),
        "Raw Global Plan": ("rviz_default_plugins/Path", "/plan"),
        "Controller Path (Smoothed)": (
            "rviz_default_plugins/Path", "/received_global_plan"),
    }
    expected_displays["Leveled Navigation Scan"] = (
        "rviz_default_plugins/LaserScan", "/scan")
    if not online_mode:
        expected_displays["AMCL Pose"] = (
            "rviz_default_plugins/PoseWithCovariance", "/amcl_pose")
    for name, (expected_class, expected_topic) in expected_displays.items():
        display = display_by_name.get(name)
        if not isinstance(display, dict):
            problems.append("RViz 缺少显示项：%s" % name)
            continue
        if display.get("Class") != expected_class:
            problems.append("RViz 显示项 %s 的 Class 不正确" % name)
        if _topic_value(display) != expected_topic:
            problems.append("RViz 显示项 %s 必须订阅 %s" % (name, expected_topic))

    panels = document.get("Panels")
    panel_classes = {
        item.get("Class") for item in panels if isinstance(item, dict)
    } if isinstance(panels, list) else set()
    if "nav2_rviz_plugins/Navigation 2" not in panel_classes:
        problems.append("RViz 必须包含 Navigation 2 面板，以便取消目标")
    if online_mode and "slam_toolbox::SlamToolboxPlugin" not in panel_classes:
        problems.append("在线 RViz 必须包含 Slam Toolbox 面板")

    tools = manager.get("Tools")
    tool_topics = {
        item.get("Class"): _topic_value(item) for item in tools
        if isinstance(item, dict) and isinstance(item.get("Class"), str)
    } if isinstance(tools, list) else {}
    if (not online_mode and
            tool_topics.get("rviz_default_plugins/SetInitialPose") != "/initialpose"):
        problems.append("2D Pose Estimate 必须发布 /initialpose")
    if "nav2_rviz_plugins/GoalTool" not in tool_topics:
        problems.append("RViz 必须使用 Nav2 Goal 工具经 action 下发目标")

    current_view = manager.get("Views", {}).get("Current", {})
    if not isinstance(current_view, dict) or current_view.get("Class") != (
            "rviz_default_plugins/TopDownOrtho"):
        problems.append("RViz 默认视角必须为 TopDownOrtho")
    return problems
