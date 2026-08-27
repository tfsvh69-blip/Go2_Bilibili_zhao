"""navigation.rviz 坐标系、地图显示与 RViz 用户入口测试。"""

from __future__ import annotations

from pathlib import Path

import yaml

from go2_navigation.rviz_config import validate_navigation_rviz


RVIZ_CONFIG = Path(__file__).parents[1] / "rviz" / "navigation.rviz"
ONLINE_RVIZ_CONFIG = (
    Path(__file__).parents[1] / "rviz" / "online_mapping_navigation.rviz"
)


def test_navigation_rviz_matches_navigation_contract():
    assert validate_navigation_rviz(RVIZ_CONFIG) == []


def test_online_mapping_rviz_matches_navigation_contract():
    assert validate_navigation_rviz(ONLINE_RVIZ_CONFIG) == []


def test_rviz_compares_raw_and_controller_paths():
    for path in (RVIZ_CONFIG, ONLINE_RVIZ_CONFIG):
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        displays = document["Visualization Manager"]["Displays"]
        topics = {
            item.get("Name"): item.get("Topic", {}).get("Value")
            for item in displays if isinstance(item, dict)
        }
        assert topics["Raw Global Plan"] == "/plan"
        assert topics["Controller Path (Smoothed)"] == "/received_global_plan"


def test_rviz_profiles_limit_frame_rate_and_online_uses_slam_panel():
    fixed = yaml.safe_load(RVIZ_CONFIG.read_text(encoding="utf-8"))
    online = yaml.safe_load(ONLINE_RVIZ_CONFIG.read_text(encoding="utf-8"))
    assert fixed["Visualization Manager"]["Global Options"]["Frame Rate"] == 20
    assert online["Visualization Manager"]["Global Options"]["Frame Rate"] == 20
    assert any(
        panel.get("Class") == "slam_toolbox::SlamToolboxPlugin"
        for panel in online["Panels"])


def test_navigation_rviz_reports_wrong_fixed_frame(tmp_path):
    document = yaml.safe_load(RVIZ_CONFIG.read_text(encoding="utf-8"))
    document["Visualization Manager"]["Global Options"]["Fixed Frame"] = "odom"
    invalid_config = tmp_path / "navigation.rviz"
    invalid_config.write_text(yaml.safe_dump(document), encoding="utf-8")

    assert "RViz Fixed Frame 必须为 map" in validate_navigation_rviz(invalid_config)
