"""nav_tuner 无 ROS 核心逻辑的单元测试。"""

from __future__ import annotations

import shutil
from pathlib import Path

import pytest
import yaml

from go2_navigation import nav_tuning
from go2_navigation.nav_tuner import TunerApplication
from go2_navigation.nav_tuning import (
    Capability,
    PROFILES,
    REGISTRY,
    RateTracker,
    conservative_clearance,
    get_path,
    laser_counts,
    parse_value,
    path_length,
    persist_values,
    replace_yaml_scalars,
)


PACKAGE_ROOT = Path(__file__).parents[1]


def test_registry_aliases_are_unique_and_paths_exist():
    assert len(REGISTRY) >= 60
    assert len(REGISTRY) == len(set(REGISTRY))
    for spec in REGISTRY.values():
        if not spec.persistent:
            continue
        document = yaml.safe_load(
            (PACKAGE_ROOT / spec.yaml_file).read_text(encoding="utf-8")
        )
        get_path(document, spec.yaml_path)


def test_matrix_classifies_obstacle_sources_and_structures_conservatively():
    assert REGISTRY["local.inflation_radius"].capability == Capability.LIVE
    assert REGISTRY["geometry.local_footprint"].capability == Capability.LIVE
    assert (
        REGISTRY["memory.local.scan.observation_persistence"].capability
        == Capability.LIFECYCLE_RELOAD
    )
    assert (
        REGISTRY["rpp.use_collision_detection"].capability
        == Capability.LIFECYCLE_RELOAD
    )
    assert (
        REGISTRY["structure.replan_frequency"].capability
        == Capability.RESTART_REQUIRED
    )


def test_profiles_remain_uncalibrated_in_stage_zero():
    assert PROFILES == {"safe": None, "balanced": None, "aggressive": None}


class _FakeRuntimeNode:
    def __init__(self):
        self.applied = {}

    def apply_values(self, changes):
        self.applied.update(changes)
        return dict(changes)


def test_command_parser_sets_validated_alias_value():
    node = _FakeRuntimeNode()
    application = TunerApplication(node)
    response = application.execute("set local.inflation_radius 0.45")
    assert response == "已生效：local.inflation_radius=0.45（LIVE）"
    assert node.applied == {"local.inflation_radius": 0.45}


def test_profile_command_refuses_uncalibrated_values():
    application = TunerApplication(_FakeRuntimeNode())
    assert "UNCALIBRATED" in application.execute("profile safe")
    assert "用法" in application.execute("profile unknown")


def test_command_parser_reports_unknown_and_malformed_commands():
    application = TunerApplication(_FakeRuntimeNode())
    assert "未知命令" in application.execute("launch")
    assert "命令解析失败" in application.execute("set 'unterminated")


def test_parse_value_checks_type_range_and_safety_switches():
    radius = REGISTRY["local.inflation_radius"]
    assert parse_value(radius, "0.45") == pytest.approx(0.45)
    with pytest.raises(ValueError, match="不得小于"):
        parse_value(radius, "-0.1")
    with pytest.raises(ValueError, match="必须有限"):
        parse_value(radius, "nan")

    safety = REGISTRY["rpp.use_collision_detection"]
    assert parse_value(safety, "true") is True
    with pytest.raises(ValueError, match="只允许保持 true"):
        parse_value(safety, "false")


def test_parse_footprint_returns_runtime_string_and_rejects_bad_polygon():
    spec = REGISTRY["geometry.local_footprint"]
    value = parse_value(spec, "[[0.4, 0.2], [0.4, -0.2], [-0.3, 0.0]]")
    assert yaml.safe_load(value) == [[0.4, 0.2], [0.4, -0.2], [-0.3, 0.0]]
    with pytest.raises(ValueError, match="至少需要 3"):
        parse_value(spec, "[[0.0, 0.0], [1.0, 0.0]]")


def test_yaml_replacement_preserves_comments_order_and_only_selected_path():
    source = """root:\n  keep: 1  # 不动\n  tune: 0.3  # 保留注释\n"""
    result = replace_yaml_scalars(source, {("root", "tune"): 0.45})
    assert "keep: 1  # 不动" in result
    assert "tune: 0.45  # 保留注释" in result
    assert list(yaml.safe_load(result)["root"]) == ["keep", "tune"]
    with pytest.raises(KeyError, match="不存在"):
        replace_yaml_scalars(source, {("root", "missing"): 1})


def _temporary_package(tmp_path: Path) -> Path:
    package = tmp_path / "go2_navigation"
    (package / "config").mkdir(parents=True)
    (package / "logs").mkdir()
    for name in ("navigation.yaml", "controller_forward_rpp.yaml"):
        shutil.copy2(PACKAGE_ROOT / "config" / name, package / "config" / name)
    return package


def test_persist_values_creates_backup_semantic_diff_and_atomic_files(tmp_path):
    package = _temporary_package(tmp_path)
    result = persist_values(
        package,
        {
            "local.inflation_radius": 0.4,
            "rpp.desired_linear_vel": 0.21,
        },
        timestamp="unit_test",
    )
    assert len(result.changed_files) == 2
    assert "inflation_radius: 0.4" in result.unified_diff
    assert "desired_linear_vel: 0.21" in result.unified_diff
    assert (result.backup_dir / "config" / "navigation.yaml").is_file()
    assert (result.backup_dir / "config" / "controller_forward_rpp.yaml").is_file()


def test_persist_single_owner_still_backs_up_both_managed_files(tmp_path):
    package = _temporary_package(tmp_path)
    result = persist_values(
        package,
        {"local.inflation_radius": 0.4},
        timestamp="single_owner",
    )
    assert len(result.changed_files) == 1
    assert (result.backup_dir / "config" / "navigation.yaml").is_file()
    assert (result.backup_dir / "config" / "controller_forward_rpp.yaml").is_file()


def test_persist_values_rolls_back_all_files_if_second_replace_fails(
    tmp_path, monkeypatch
):
    package = _temporary_package(tmp_path)
    navigation = package / "config" / "navigation.yaml"
    controller = package / "config" / "controller_forward_rpp.yaml"
    originals = {
        navigation: navigation.read_text(encoding="utf-8"),
        controller: controller.read_text(encoding="utf-8"),
    }
    real_replace = nav_tuning.os.replace
    calls = 0

    def fail_second_replace(source, destination):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise OSError("模拟第二个文件替换失败")
        return real_replace(source, destination)

    monkeypatch.setattr(nav_tuning.os, "replace", fail_second_replace)
    with pytest.raises(OSError, match="第二个文件"):
        persist_values(
            package,
            {
                "local.inflation_radius": 0.4,
                "rpp.desired_linear_vel": 0.21,
            },
            timestamp="rollback_test",
        )
    assert navigation.read_text(encoding="utf-8") == originals[navigation]
    assert controller.read_text(encoding="utf-8") == originals[controller]


def test_sensor_plan_and_clearance_metrics_are_deterministic():
    counts = laser_counts([float("inf"), float("nan"), 0.5, 1.0, 3.0], 0.9, 2.0)
    assert counts == {"total": 5, "valid": 1, "inf": 1, "nan": 1, "nearest": 1.0}
    assert path_length([(0.0, 0.0), (3.0, 4.0)]) == pytest.approx(5.0)
    clearance = conservative_clearance(
        [(0.0, 0.0), (1.0, 0.0)],
        [(0.5, 0.5)],
        0.1,
    )
    assert clearance == pytest.approx(0.45)


def test_rate_tracker_reports_frequency_age_and_percentiles():
    tracker = RateTracker()
    for stamp in (1.0, 1.2, 1.4, 1.8):
        tracker.add(stamp)
    summary = tracker.summary(2.0)
    assert summary["hz"] == pytest.approx(3 / 0.8)
    assert summary["age"] == pytest.approx(0.2)
    assert summary["period_p50"] == pytest.approx(0.2)
    assert summary["period_p99"] == pytest.approx(0.396)
