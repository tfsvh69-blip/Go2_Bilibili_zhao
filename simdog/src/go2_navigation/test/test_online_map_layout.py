from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = PACKAGE_ROOT.parents[2]


def test_online_map_save_writes_traceable_session_metadata():
    script = (
        PACKAGE_ROOT / "scripts" / "save_online_map.sh"
    ).read_text(encoding="utf-8")

    assert "go2_maps/online" in script
    assert "go2_online_map_session_v1" in script
    assert "session.yaml" in script
    assert "lidar_min_height_m" in script
    assert "lidar_max_height_m" in script
    assert "ln -sfn" in script
    assert "目标地图目录已存在" in script


def test_project_tracks_only_the_map_directory_guide():
    ignore = (PROJECT_ROOT / ".gitignore").read_text(encoding="utf-8")
    guide = (PROJECT_ROOT / "go2_maps" / "README.md").read_text(
        encoding="utf-8")

    assert "/go2_maps/*" in ignore
    assert "!/go2_maps/README.md" in ignore
    assert "session.yaml" in guide
    assert "online/latest" in guide
    assert "LIO-SAM/NDT" in guide
