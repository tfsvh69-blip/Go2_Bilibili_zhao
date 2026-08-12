"""确保 Go2 导航树为 Humble FollowPath 显式选择目标检查器。"""

from __future__ import annotations

from pathlib import Path
from xml.etree import ElementTree

import yaml


TREE_PATH = (
    Path(__file__).parents[1]
    / "behavior_trees"
    / "go2_navigate_to_pose.xml"
)


def test_follow_path_uses_configured_goal_checker():
    root = ElementTree.parse(TREE_PATH).getroot()
    follow_paths = root.findall(".//FollowPath")

    assert len(follow_paths) == 1
    assert follow_paths[0].attrib["controller_id"] == "FollowPath"
    assert follow_paths[0].attrib["goal_checker_id"] == "general_goal_checker"


def test_computed_path_is_smoothed_with_collision_check():
    root = ElementTree.parse(TREE_PATH).getroot()
    smooth_paths = root.findall(".//SmoothPath")

    assert len(smooth_paths) == 1
    smoother = smooth_paths[0].attrib
    assert smoother["unsmoothed_path"] == "{path}"
    assert smoother["smoothed_path"] == "{path}"
    assert smoother["smoother_id"] == "simple_smoother"
    assert smoother["check_for_collisions"] == "true"


def test_smoothing_failure_falls_back_to_raw_path():
    root = ElementTree.parse(TREE_PATH).getroot()
    fallback = root.find(".//Fallback[@name='UseRawPathWhenSmoothingFails']")

    assert fallback is not None
    assert [child.tag for child in fallback] == ["SmoothPath", "AlwaysSuccess"]


def test_smooth_path_bt_plugin_is_loaded():
    config_path = TREE_PATH.parents[1] / "config" / "navigation.yaml"
    document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    libraries = document["bt_navigator"]["ros__parameters"]["plugin_lib_names"]

    assert "nav2_smooth_path_action_bt_node" in libraries
