# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this directory is

`~/my/ROS/Go2` is **not a single codebase or a git repo** — it is the root of a Unitree Go2 robotics development setup that holds several independent ROS2 (Humble) colcon workspaces plus read-only source assets. Ubuntu 22.04, ROS2 Humble, Gazebo **Classic 11** (the version paired with Humble; `gazebo`, not `gz sim`).

Three moving parts:

| Path | Role |
|---|---|
| `simdog/` | **Primary simulation.** fishros/simdog: standard `ros2_control` + CHAMP real trot gait + Velodyne 3D lidar + LIO-SAM 3D SLAM. |
| `go2_ws/` | **Lightweight backup** hand-built here: `go2_description` + `go2_gazebo`, a *planar-move* platform (rigid body that slides on `/cmd_vel`, no leg dynamics). |
| `~/unitree_ros2` (in `$HOME`, outside this dir) | Official **DDS comms layer** (CycloneDDS + `unitree_go`/`unitree_api` msgs) for talking to the real robot / unified interface. Not a simulator. |

Read-only inputs (do not treat as code to edit): `Go2 URDF/` (official URDF, source of `go2_ws` model), `Go2简化模型/` (a `.stp` CAD file — no kinematics, not directly usable), `学习文档/` (GitBook tutorial that covers **real-robot DDS development, not Gazebo**).

## Build & run

Each workspace is built and sourced independently. **Never source two of these workspaces in the same shell.**

```bash
# --- simdog (primary) ---
cd ~/my/ROS/Go2/simdog
colcon build --symlink-install        # 16 packages; LIO-SAM takes a few min
source install/setup.bash
ros2 launch go2_config gazebo_velodyne.launch.py rviz:=true   # or: bash start.sh
bash save_Map.sh                       # save the LIO-SAM map after driving around

# --- go2_ws (backup planar-move) ---
cd ~/my/ROS/Go2/go2_ws
colcon build && source install/setup.bash
ros2 launch go2_gazebo spawn.launch.py rviz:=true      # gui:=false / z:=<h> also accepted
ros2 launch go2_description display.launch.py           # RViz-only model view

# --- teleop (works for either sim; publishes /cmd_vel) ---
ros2 run teleop_twist_keyboard teleop_twist_keyboard    # i=fwd , =back  j/l=turn  k=stop; don't spam q

# --- comms layer ---
source ~/unitree_ros2/setup_local.sh   # lo interface; setup.sh=enp3s0 real robot, setup_default.sh=any
```

There are no unit tests / linters configured in these workspaces. "Verifying a change" means launching the sim headless and checking topics: real-time factor from `/clock`, plus `/odom/*`, `/velodyne_points`, `/scan`, `/imu`, `/joint_states`.

## Critical gotchas (this hardware / these repos)

- **gzclient crashes on this machine's RTX 5070** (Gazebo Classic OGRE 1.9 vs new NVIDIA driver → SIGKILL). RViz is fine. Run Gazebo with **`gui:=false rviz:=true`**, or prefix **`LIBGL_ALWAYS_SOFTWARE=1`** to keep the Gazebo window. `simdog/start.sh` is already patched this way.
- **Velodyne must be `gpu_ray`, never `ray`.** CPU `ray` (16×1800 = 28800 casts/frame) single-threads gzserver and drops real-time factor to ~0.12. Already fixed to `gpu_ray` in `simdog/src/unitree-go2-ros2/robots/descriptions/go2_description/xacro/velodyne.xacro`, restoring RTF to ~1.0. Because simdog uses `--symlink-install`, editing that xacro in `src/` takes effect on the next Gazebo launch **without rebuilding**.
- **simdog's Nav2 is not configured** — `navigate.launch.py` is an upstream stub ("等待github更新"). Autonomous navigation must be built ourselves. LIO-SAM mapping and teleop do work. `start.sh` also launches `ndt_relocalization`, which errors unless a prebuilt map exists at the hardcoded path — skip it during mapping.
- **New launch files must be written in XML** (`.launch.xml`), not Python — user preference.
- **`sudo` requires a password** (no non-interactive apt); there is **no `pip`/`ensurepip`**. Ask the user to run `sudo`/`apt` steps.
- **Building CycloneDDS** (in `~/unitree_ros2/cyclonedds_ws`) must be done with **ROS2 NOT sourced** (clear `AMENT_PREFIX_PATH`/`CMAKE_PREFIX_PATH` and strip `/opt/ros` from `PATH`), or it fails. The `setup*.sh` scripts there were changed from `foxy` to `humble`.

## simdog architecture (the big picture)

`simdog/src/unitree-go2-ros2/` bundles the whole CHAMP stack under `champ/` (no separate clone). Control and SLAM data flow:

- **Locomotion**: `/cmd_vel` → `quadruped_controller_node` (CHAMP gait generator, `champ_base`) → `joint_group_effort_controller` (ros2_control, via `gazebo_ros2_control`) → Gazebo. State estimate from `state_estimation_node` + `robot_localization` EKF fuses `/odom/raw` + `/imu/data` → `/odom/local`.
- **Perception/SLAM**: Velodyne sensor → `/velodyne_points` → LIO-SAM nodes (`imageProjection` → `featureExtraction` → `imuPreintegration` → `mapOptimization`) → 3D map. `pointcloud_to_laserscan` can flatten the cloud to a 2D `/scan` for 2D SLAM/nav.
- Go2-specific config lives in `robots/configs/go2_config` (launch, worlds, controllers) and `robots/descriptions/go2_description` (xacro incl. `velodyne.xacro`, `robot_VLP.xacro`). The generic quadruped code is in `champ/champ_*`.

## go2_ws (planar-move backup) architecture

`go2_gazebo/scripts/gen_planar_urdf.py` generates `go2_gazebo/urdf/go2_gazebo.urdf` from the clean `go2_description` URDF by: welding the 12 leg revolute joints to **fixed** (rigid body — won't collapse), setting feet **frictionless** (`mu=0`, or planar_move tips it over), and injecting the `planar_move` plugin + IMU + 2D lidar + depth camera. The generated URDF must carry **no XML declaration** (spawn_entity's lxml rejects the `encoding` attribute). See `go2_ws/README.md`. Regenerate after editing the script, then `colcon build --packages-select go2_gazebo`.
