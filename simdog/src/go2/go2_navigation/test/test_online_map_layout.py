from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = PACKAGE_ROOT.parents[3]


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
    # 脚本位于 simdog/src/go2/go2_navigation/scripts；源码分层后
    # 必须回退五层才是项目根，不得把地图写入 simdog/go2_maps。
    assert '${script_dir}/../../../../..' in script


def test_lio_sam_save_uses_project_root_and_workspace_setup():
    script = (PACKAGE_ROOT / "scripts" / "save_map.sh").read_text(
        encoding="utf-8")

    assert 'project_root="$(cd -- "${script_dir}/../../../../.."' in script
    assert '${script_dir}/../../../../install/setup.bash' in script


def test_project_tracks_only_the_map_directory_guide():
    ignore = (PROJECT_ROOT / ".gitignore").read_text(encoding="utf-8")
    guide = (PROJECT_ROOT / "go2_maps" / "README.md").read_text(
        encoding="utf-8")

    assert "/go2_maps/*" in ignore
    assert "!/go2_maps/README.md" in ignore
    assert "session.yaml" in guide
    assert "online/latest" in guide
    assert "LIO-SAM/NDT" in guide
