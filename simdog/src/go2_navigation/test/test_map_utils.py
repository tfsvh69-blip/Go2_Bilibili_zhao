"""go2_navigation 地图包与目标栅格校验单元测试。"""

from __future__ import annotations

import hashlib
import json

from PIL import Image
import pytest
import yaml

from go2_navigation.map_utils import (
    COMMISSIONING_CLEARANCE_M,
    MAP_FILES,
    load_static_map,
    occupancy_grid_to_static_map,
)
from go2_navigation.validate_map_bundle import validate


def _sha256(path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _create_bundle(tmp_path):
    image = Image.new("L", (20, 20), color=254)
    image.putpixel((10, 10), 0)
    image.save(tmp_path / "map.pgm")
    (tmp_path / "GlobalMap.pcd").write_bytes(b"pcd")
    (tmp_path / "map_stats.json").write_text(json.dumps({"width": 20}), encoding="utf-8")
    (tmp_path / "map.yaml").write_text(yaml.safe_dump({
        "image": "map.pgm",
        "resolution": 0.1,
        "origin": [0.0, 0.0, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }), encoding="utf-8")
    files = {
        name: {
            "path": name,
            "sha256": _sha256(tmp_path / name),
            "role": role,
        }
        for name, role in MAP_FILES.items()
    }
    (tmp_path / "map_bundle.yaml").write_text(yaml.safe_dump({
        "schema_version": 1,
        "frame_id": "map",
        "files": files,
        "generation": {"offline_inflate_radius_m": 0.0},
    }), encoding="utf-8")


def test_validate_current_schema_bundle(tmp_path):
    _create_bundle(tmp_path)
    assert validate(str(tmp_path)) == (True, [])


def test_rejects_legacy_or_offline_inflated_bundle(tmp_path):
    _create_bundle(tmp_path)
    manifest_path = tmp_path / "map_bundle.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    manifest["generation"]["offline_inflate_radius_m"] = 0.6
    manifest_path.write_text(yaml.safe_dump(manifest), encoding="utf-8")
    ok, problems = validate(str(tmp_path))
    assert not ok
    assert any("离线膨胀" in problem for problem in problems)


def test_goal_validation_checks_bounds_occupancy_and_clearance(tmp_path):
    _create_bundle(tmp_path)
    static_map = load_static_map(tmp_path)
    assert static_map.validate_pose(1.8, 1.8, 0.1) is None
    assert "超出地图边界" in static_map.validate_pose(-0.1, 1.0, 0.1)
    assert "自由栅格" in static_map.validate_pose(1.05, 0.95, 0.1)
    assert "安全余量" in static_map.validate_pose(0.1, 1.0, 0.55)


def test_commissioning_clearance_keeps_only_a_small_guard_margin():
    assert 0.0 < COMMISSIONING_CLEARANCE_M <= 0.15


def test_map_yaml_rejects_path_escape(tmp_path):
    _create_bundle(tmp_path)
    metadata_path = tmp_path / "map.yaml"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    metadata["image"] = "../outside.pgm"
    metadata_path.write_text(yaml.safe_dump(metadata), encoding="utf-8")
    with pytest.raises(ValueError, match="不能离开"):
        load_static_map(tmp_path)


def test_online_occupancy_grid_uses_ros_coordinates_and_unknown_semantics():
    # ROS OccupancyGrid 从左下角开始：左下自由、右下 unknown、
    # 左上占用、右上自由。
    dynamic_map = occupancy_grid_to_static_map(
        width=2,
        height=2,
        resolution=1.0,
        origin_x=-1.0,
        origin_y=-1.0,
        data=[0, -1, 100, 0],
    )
    assert dynamic_map.is_known_free(-0.5, -0.5)
    assert not dynamic_map.is_known_free(0.5, -0.5)
    assert not dynamic_map.is_known_free(-0.5, 0.5)
    assert dynamic_map.is_known_free(0.5, 0.5)


def test_online_occupancy_grid_rejects_invalid_shape():
    with pytest.raises(ValueError, match="数据长度"):
        occupancy_grid_to_static_map(2, 2, 0.05, 0.0, 0.0, [0, 0])
