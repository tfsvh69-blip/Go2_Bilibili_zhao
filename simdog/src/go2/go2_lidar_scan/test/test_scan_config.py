from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PACKAGE_ROOT.parents[1]
GO2_ROOT = SRC_ROOT / "go2"
PLATFORM_ROOT = SRC_ROOT / "platform"
VENDOR_ROOT = SRC_ROOT / "vendor"


def test_converter_uses_valid_infinity_contract():
    data = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "vlp16_scan.yaml").read_text(
            encoding="utf-8"))
    params = data["go2_lidar_scan_converter"]["ros__parameters"]

    assert params["use_inf"] is True
    assert params["target_frame"] == "velodyne_level"
    assert params["range_min"] == 0.9
    assert params["range_max"] == 15.0
    assert params["min_height"] == 0.48
    assert params["max_height"] == 0.60
    assert params["always_subscribe"] is True
    assert params["allow_runtime_height_update"] is True

    raw = data["go2_lidar_scan_raw_converter"]["ros__parameters"]
    assert raw["target_frame"] == ""
    assert raw["range_min"] == params["range_min"]
    assert raw["min_height"] == -0.05
    assert raw["max_height"] == 0.10
    assert raw["allow_runtime_height_update"] is False

    level = data["go2_lidar_level_frame"]["ros__parameters"]
    assert level["reference_frame"] == "base_footprint"
    assert level["level_frame"] == "velodyne_level"
    assert level["cloud_heartbeat_topic"] == (
        "/go2_lidar_scan/cloud_heartbeat")
    assert level["cloud_ready_topic"] == (
        "/go2_lidar_scan/leveled_cloud")
    assert level["probe_cloud_topic"] == "/go2_lidar_scan/probe_cloud"
    assert level["probe_stride"] == 5

    diagnostics = data["go2_lidar_scan_diagnostics"]["ros__parameters"]
    assert diagnostics["stale_timeout_s"] == 1.0
    assert diagnostics["minimum_rate_hz"] == 7.0
    assert diagnostics["expected_scan_frame"] == "velodyne_level"
    assert diagnostics["expected_min_height"] == 0.48
    assert diagnostics["expected_max_height"] == 0.60
    assert diagnostics["cloud_heartbeat_topic"] == (
        "/go2_lidar_scan/cloud_heartbeat")


def test_launch_defaults_to_one_navigation_scan_and_debug_enables_raw_scan():
    pipeline = (PACKAGE_ROOT / "launch" / "scan_pipeline.launch.py").read_text(
        encoding="utf-8")
    debug = (
        PACKAGE_ROOT / "launch" / "simulation_scan_debug.launch.xml"
    ).read_text(encoding="utf-8")

    assert '"lidar_debug_raw_scan", default_value="false"' in pipeline
    assert 'name="go2_lidar_scan_raw_converter"' in pipeline
    assert '"go2_lidar_scan::LevelFramePublisher"' in pipeline
    assert '"cloud_in", cloud_ready_topic' in pipeline
    assert '"use_intra_process_comms": True' in pipeline
    assert 'name="lidar_debug_raw_scan" default="true"' in debug
    assert 'name="tuning_gui" default="false"' in debug
    assert "/go2_lidar_scan_converter" in debug


def test_converter_really_updates_height_and_rejects_fake_runtime_changes():
    source = (
        VENDOR_ROOT / "pointcloud_to_laserscan" / "src" /
        "pointcloud_to_laserscan_node.cpp"
    ).read_text(encoding="utf-8")
    nav_launch = (
        GO2_ROOT / "go2_navigation" / "launch" /
        "navigation.launch.py"
    ).read_text(encoding="utf-8")

    assert "add_on_set_parameters_callback" in source
    assert "min_height_ = proposed_min" in source
    assert "max_height_ = proposed_max" in source
    assert "allow_runtime_height_update_" in source
    assert "min_height < max_height" in source
    assert "运行时只支持 min_height/max_height" in source
    assert "height_range.step = 0.01" in source
    assert "过低会把地面投影进 /scan" in source
    assert '"--args"' in nav_launch
    assert '"/go2_lidar_scan_converter"' in nav_launch


def test_nav2_costmap_matches_scan_clearing_contract():
    navigation = yaml.safe_load(
        (GO2_ROOT / "go2_navigation" / "config" /
         "navigation.yaml").read_text(encoding="utf-8"))
    scan_sources = [
        navigation["global_costmap"]["global_costmap"]["ros__parameters"]
        ["obstacle_layer"]["scan"],
        navigation["local_costmap"]["local_costmap"]["ros__parameters"]
        ["scan_layer"]["scan"],
    ]

    for source in scan_sources:
        assert source["inf_is_valid"] is True
        assert source["obstacle_max_range"] < 15.0
        assert source["raytrace_max_range"] == 15.0
        assert source["marking"] is True
        assert source["clearing"] is True

    global_costmap = navigation["global_costmap"]["global_costmap"][
        "ros__parameters"]
    local_costmap = navigation["local_costmap"]["local_costmap"][
        "ros__parameters"]
    assert global_costmap["transform_tolerance"] == 0.50
    assert local_costmap["transform_tolerance"] == 0.50


def test_gazebo_vlp16_uses_motion_realtime_sampling():
    xacro_path = (
        PLATFORM_ROOT / "unitree-go2-ros2" / "robots" / "descriptions" /
        "go2_description" / "xacro" / "velodyne.xacro"
    )
    text = xacro_path.read_text(encoding="utf-8")

    assert "<samples>900</samples>" in text
    assert "<min>0.9</min>" in text
    assert "<min_range>0.9</min_range>" in text


def test_d435_render_load_matches_navigation_profile():
    xacro_path = (
        VENDOR_ROOT / "realsense_ros_gazebo" / "xacro" /
        "depthcam.xacro"
    )
    text = xacro_path.read_text(encoding="utf-8")
    d435 = text.split('<xacro:macro name="realsense_R200"', 1)[0]

    assert d435.count(
        "<width>$(optenv GO2_D435_IMAGE_WIDTH 640)</width>") == 4
    assert d435.count(
        "<height>$(optenv GO2_D435_IMAGE_HEIGHT 480)</height>") == 4
    assert d435.count(
        "<always_on>$(optenv GO2_D435_AUX_STREAMS 1)</always_on>") == 3
    assert '<xacro:if value="$(optenv GO2_D435_GAZEBO_ENABLED 1)">' in d435
    assert d435.count("<update_rate>${rate}</update_rate>") == 1
    assert (
        "<update_rate>$(optenv GO2_D435_DEPTH_RATE 10)</update_rate>"
        in d435)
    assert d435.count("<update_rate>1</update_rate>") == 2
    assert "<infraredUpdateRate>1</infraredUpdateRate>" in d435
    assert "<update_rate>90</update_rate>" not in d435

    navigation_launch = (
        GO2_ROOT / "go2_navigation" / "launch" /
        "simulation_navigation.launch.xml"
    ).read_text(encoding="utf-8")
    assert 'name="use_d435_navigation" default="false"' in navigation_launch
    assert (
        'name="GO2_D435_GAZEBO_ENABLED" '
        'value="$(var use_d435_navigation)"' in navigation_launch)


def test_navigation_ground_truth_does_not_inject_pose_noise():
    gazebo_path = (
        PLATFORM_ROOT / "unitree-go2-ros2" / "robots" /
        "descriptions" / "go2_description" / "xacro" / "gazebo.xacro"
    )
    text = gazebo_path.read_text(encoding="utf-8")

    assert "<gaussian_noise>0.0</gaussian_noise>" in text
