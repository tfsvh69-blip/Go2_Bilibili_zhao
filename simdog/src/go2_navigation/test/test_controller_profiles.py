"""RPP/MPPI 控制 profile 与在线建图入口静态测试。"""

from __future__ import annotations

import ast
from pathlib import Path
from xml.etree import ElementTree

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


def _parameters(filename: str, node_name: str) -> dict:
    document = yaml.safe_load(
        (PACKAGE_ROOT / "config" / filename).read_text(encoding="utf-8"))
    return document[node_name]["ros__parameters"]


def test_forward_rpp_is_fast_forward_facing_humble_profile():
    controller = _parameters(
        "controller_forward_rpp.yaml", "controller_server")["FollowPath"]
    assert controller["plugin"] == (
        "nav2_rotation_shim_controller::RotationShimController")
    assert controller["primary_controller"] == (
        "nav2_regulated_pure_pursuit_controller::"
        "RegulatedPurePursuitController")
    assert controller["rotate_to_goal_heading"] is True
    assert controller["closed_loop"] is False
    assert controller["angular_dist_threshold"] == 1.40
    assert controller["angular_disengage_threshold"] == 0.40
    assert controller["forward_sampling_distance"] == 0.50
    assert controller["simulate_ahead_time"] == 1.0
    assert controller["desired_linear_vel"] == 0.27
    assert controller["lookahead_dist"] == 0.55
    assert controller["min_lookahead_dist"] == 0.35
    assert controller["max_lookahead_dist"] == 0.80
    assert controller["use_rotate_to_heading"] is False
    assert controller["rotate_to_heading_min_angle"] == 1.40
    assert controller["rotate_to_heading_angular_vel"] == 0.45
    assert controller["max_angular_accel"] == 1.0
    assert controller["use_collision_detection"] is True
    assert controller["max_allowed_time_to_collision_up_to_carrot"] == 1.0
    assert controller["use_cost_regulated_linear_velocity_scaling"] is False
    assert not any(key.startswith("vy_") for key in controller)
    smoother = _parameters(
        "controller_forward_rpp.yaml", "velocity_smoother")
    assert smoother["max_velocity"] == [0.27, 0.15, 0.45]
    assert smoother["min_velocity"] == [-0.27, -0.15, -0.45]


def test_forward_rpp_declares_rotation_shim_runtime_dependency():
    root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
    dependencies = {
        element.text for element in root.findall("exec_depend")
        if element.text
    }
    assert "nav2_rotation_shim_controller" in dependencies


def test_go2_gait_does_not_press_feet_below_nominal_stance():
    gait_path = (
        PACKAGE_ROOT.parent
        / "unitree-go2-ros2"
        / "robots"
        / "configs"
        / "go2_config"
        / "config"
        / "gait"
        / "gait.yaml"
    )
    gait_document = yaml.safe_load(gait_path.read_text(encoding="utf-8"))
    gait = gait_document["/**"]["ros__parameters"]["gait"]

    # 原地旋转 A/B 表明 1 cm 支撑下压会显著放大实体侧滑；CHAMP
    # Go1 上游基线同样使用 0.0，且不改变碰撞或速度安全上限。
    assert gait["stance_depth"] == 0.0
    assert gait["stance_duration"] == 0.25
    assert gait["swing_height"] == 0.04


def test_forward_mppi_is_bounded_diff_drive_comparison_profile():
    controller = _parameters(
        "controller_forward_mppi.yaml", "controller_server")["FollowPath"]
    assert controller["plugin"] == "nav2_mppi_controller::MPPIController"
    assert controller["motion_model"] == "DiffDrive"
    assert controller["model_dt"] == 0.10
    assert controller["batch_size"] == 800
    assert controller["vx_max"] == 0.27
    assert controller["vx_min"] == 0.0
    assert controller["vy_max"] == 0.0
    assert controller["visualize"] is False


def test_omni_mppi_profile_remains_available_for_comparison():
    controller = _parameters(
        "controller_omni_mppi.yaml", "controller_server")["FollowPath"]
    assert controller["plugin"] == "nav2_mppi_controller::MPPIController"
    assert controller["motion_model"] == "Omni"
    assert controller["vy_max"] == 0.15


def test_unified_navigation_defaults_to_online_slam_amcl_and_forward_mppi():
    root = ElementTree.parse(
        PACKAGE_ROOT / "launch" / "simulation_navigation.launch.xml").getroot()
    arguments = {item.attrib["name"]: item.attrib.get("default")
                 for item in root.findall("arg")}
    assert arguments["controller_profile"] == "forward_mppi"
    assert arguments["navigation_mode"] == "online_slam"
    assert arguments["localization"] == "amcl"
    assert arguments["gui"] == "true"
    assert arguments["tuning_gui"] == "false"
    assert arguments["lidar_debug_raw_scan"] == "false"


def test_global_inflation_uses_field_baseline_without_changing_local_layer():
    navigation = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "navigation.yaml").read_text(
            encoding="utf-8"))
    global_inflation = navigation["global_costmap"]["global_costmap"][
        "ros__parameters"]["inflation_layer"]
    local_inflation = navigation["local_costmap"]["local_costmap"][
        "ros__parameters"]["inflation_layer"]

    assert global_inflation["inflation_radius"] == 0.20
    assert global_inflation["cost_scaling_factor"] == 0.5
    assert local_inflation["inflation_radius"] == 0.30
    assert local_inflation["cost_scaling_factor"] == 3.0


def test_old_online_entry_is_compatibility_wrapper():
    text = (PACKAGE_ROOT / "launch" /
            "simulation_online_mapping_navigation.launch.xml").read_text(
                encoding="utf-8")
    assert 'navigation_mode" value="online_slam"' in text
    assert "simulation_navigation.launch.xml" in text
    assert "gazebo_velodyne.launch.py" not in text
    assert 'tuning_gui" value="$(var tuning_gui)"' in text


def test_all_navigation_and_mapping_entries_show_gazebo_by_default():
    for filename in (
            "simulation_navigation.launch.xml",
            "simulation_online_mapping_navigation.launch.xml",
            "mapping.launch.xml"):
        root = ElementTree.parse(PACKAGE_ROOT / "launch" / filename).getroot()
        arguments = {
            item.attrib["name"]: item.attrib.get("default")
            for item in root.findall("arg")
        }
        assert arguments["gui"] == "true"


def test_online_map_config_has_frequent_updates_and_single_slam_tf_owner():
    slam = _parameters("online_mapping.yaml", "slam_toolbox")
    assert slam["mode"] == "mapping"
    assert slam["map_update_interval"] == 1.0
    assert slam["minimum_travel_distance"] == 0.10
    assert slam["scan_queue_size"] == 1
    assert slam["transform_publish_period"] > 0.0


def test_navigation_uses_planar_frames_scan_sources_and_correct_depth_topic():
    params = _parameters("navigation.yaml", "controller_server")
    assert params["controller_frequency"] == 10.0
    assert params["general_goal_checker"]["xy_goal_tolerance"] == 0.30
    assert params["general_goal_checker"]["yaw_goal_tolerance"] == 0.15
    assert params["precise_goal_checker"]["xy_goal_tolerance"] == 0.10
    progress = params["progress_checker"]
    assert progress["plugin"] == "nav2_controller::PoseProgressChecker"
    assert progress["required_movement_radius"] == 0.10
    assert progress["required_movement_angle"] == 0.15
    document = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "navigation.yaml").read_text(encoding="utf-8"))
    planner = document["planner_server"]["ros__parameters"]["GridBased"]
    assert planner["tolerance"] == 0.0
    assert planner["use_final_approach_orientation"] is False
    assert document["bt_navigator"]["ros__parameters"][
        "robot_base_frame"] == "base_footprint"
    collision = document["collision_monitor"]["ros__parameters"]
    assert collision["base_frame_id"] == "base_footprint"
    assert collision["polygons"] == ["footprint", "decel_zone", "stop_zone"]
    assert collision["footprint"]["enabled"] is True
    assert collision["decel_zone"]["enabled"] is True
    assert collision["stop_zone"]["enabled"] is True
    assert collision["scan"]["topic"] == "/scan"
    assert collision["scan"]["enabled"] is True
    assert collision["d435"]["topic"] == "/depth/color/points"
    assert collision["d435"]["enabled"] is True
    global_costmap = document["global_costmap"]["global_costmap"][
        "ros__parameters"]
    global_scan = global_costmap["obstacle_layer"]["scan"]
    assert global_scan["max_obstacle_height"] == 2.0
    assert global_scan["inf_is_valid"] is True
    assert global_scan["obstacle_max_range"] == 14.0
    assert global_scan["raytrace_max_range"] == 15.0
    assert global_scan["marking"] is True
    assert global_scan["clearing"] is True
    local = document["local_costmap"]["local_costmap"]["ros__parameters"]
    local_footprint = ast.literal_eval(local["footprint"])
    global_footprint = ast.literal_eval(global_costmap["footprint"])
    assert local_footprint == global_footprint
    assert len(local_footprint) == 24
    assert min(point[0] for point in local_footprint) == -0.399
    assert max(point[0] for point in local_footprint) == 0.354
    assert min(point[1] for point in local_footprint) == -0.202
    assert max(point[1] for point in local_footprint) == 0.194
    assert local["footprint_padding"] == 0.035
    assert global_costmap["footprint_padding"] == 0.035
    assert "\n" not in local["footprint"]
    assert local["robot_base_frame"] == "base_footprint"
    assert local["scan_layer"]["scan"]["data_type"] == "LaserScan"
    assert local["scan_layer"]["scan"]["max_obstacle_height"] == 2.0
    assert local["scan_layer"]["scan"]["inf_is_valid"] is True
    assert local["scan_layer"]["scan"]["obstacle_max_range"] == 14.0
    assert local["scan_layer"]["scan"]["raytrace_max_range"] == 15.0
    assert local["scan_layer"]["scan"]["marking"] is True
    assert local["scan_layer"]["scan"]["clearing"] is True
    assert local["d435_layer"]["d435"]["topic"] == "/depth/color/points"


def test_amcl_is_standard_fixed_map_localization_backend():
    amcl = _parameters("localization_amcl.yaml", "amcl")
    assert amcl["base_frame_id"] == "base_footprint"
    assert amcl["scan_topic"] == "/scan"
    assert amcl["robot_model_type"] == "nav2_amcl::DifferentialMotionModel"
    assert amcl["laser_model_type"] == "likelihood_field_prob"
    assert amcl["do_beamskip"] is True
    assert amcl["max_beams"] == 90
    assert amcl["alpha1"] == 0.05


def test_ndt_experiment_uses_two_d_ekf_and_clearable_reinit_latch():
    ndt = _parameters("localization_ndt.yaml", "/**")
    assert ndt["enable_map_odom_tf"] is False
    assert ndt["base_frame_id"] == "base_footprint"
    assert ndt["reinitialization_request_clear_max_fitness"] == 5.25
    ekf = _parameters("localization_ndt_ekf.yaml", "ndt_global_ekf")
    assert ekf["two_d_mode"] is True
    assert ekf["world_frame"] == "map"
    assert ekf["base_link_frame"] == "base_footprint"


def test_navigation_launch_reuses_standard_bt_navigator_and_disables_respawn():
    text = (PACKAGE_ROOT / "launch" / "navigation.launch.py").read_text(
        encoding="utf-8")
    assert '"bt_navigator", "nav2_bt_navigator", "bt_navigator"' in text
    for endpoint in ("send_goal", "get_result", "cancel_goal", "feedback", "status"):
        assert '"/navigate_to_pose/_action/%s"' % endpoint in text
        assert '"/navigate_to_pose_raw/_action/%s"' % endpoint in text
    assert "go2_nav2_bt_navigator" not in text
    assert "respawn=True" not in text
    assert '{"bond_timeout": 10.0}' in text
    assert '"forward_mppi": "controller_forward_mppi.yaml"' in text
    assert 'LaunchConfiguration("tuning_gui")' in text
    assert 'LaunchConfiguration("lidar_debug_raw_scan")' in text
    assert '"lidar_debug_raw_scan": str(lidar_debug_raw_scan).lower()' in text
    assert 'rqt_reconfigure.param_plugin.ParamPlugin' in text


def test_both_rviz_profiles_show_padded_runtime_footprint():
    for filename in ("navigation.rviz", "online_mapping_navigation.rviz"):
        text = (PACKAGE_ROOT / "rviz" / filename).read_text(encoding="utf-8")
        assert "Name: Robot Footprint (Padded)" in text
        assert "Value: /local_costmap/published_footprint" in text
        assert "Durability Policy: Volatile" in text


def test_amcl_accepts_native_two_d_map_while_ndt_keeps_bundle_validation():
    text = (PACKAGE_ROOT / "launch" / "navigation.launch.py").read_text(
        encoding="utf-8")
    assert 'if localization == "amcl":' in text
    assert "load_static_map(map_dir)" in text
    assert "ok, problems = _validate_map_bundle(map_dir)" in text
    assert "AMCL 二维地图校验失败" in text
    assert "NDT 同源地图包校验失败" in text


def test_goal_guard_forwards_action_asynchronously():
    source = (PACKAGE_ROOT / "go2_navigation" / "goal_guard.py").read_text(
        encoding="utf-8")
    tree = ast.parse(source)
    execute = next(
        item for item in ast.walk(tree)
        if isinstance(item, ast.AsyncFunctionDef) and item.name == "_execute")
    calls = {
        ast.unparse(item.func)
        for item in ast.walk(execute)
        if isinstance(item, ast.Call)
    }
    assert "self._raw_client.send_goal_async" in calls
    assert "raw_goal.get_result_async" in calls
    assert "time.sleep" not in calls
    assert "MultiThreadedExecutor" not in source


def test_goal_guard_exposes_accepted_raw_goal_for_diagnostics():
    source = (PACKAGE_ROOT / "go2_navigation" / "goal_guard.py").read_text(
        encoding="utf-8")
    assert '"/navigation/accepted_goal"' in source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert "self._accepted_goal_publisher.publish(goal.pose)" in source
