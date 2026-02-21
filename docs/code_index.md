# Code Index

This document maps the current `harmonic_ws` layout: packages, launch files, worlds, nodes, and key topics/parameters.  
Last updated: 2025-12-20.

## Workspace layout (relevant to this repo)
- `src/ardupilot_gazebo/` — Models/worlds for ArduPilot Gazebo integration (installed into `install/ardupilot_gazebo/...`).
- `src/tag_hover_sim/` — AprilTag hover/yaw-search simulation (Gazebo Harmonic + ArduPilot SITL).
- `src/tag_hover_two_tags/` — Two-tag relative pose measurement stack (Gazebo Harmonic + ArduPilot SITL).
- `LOCKON_NOTES.md` — Hands-on notes for the lockon/tag hover workflow.

## Packages

### tag_hover_sim
- `package.xml` — ROS 2 package manifest.
- `setup.py` — installs launch + config; registers console-scripts.
- `launch/`
  - `sim_lockon_backbone.launch.py` — Minimal bringup: MAVROS + `hover_yaw_search` (camera bridge, tag detector, and PnP TF run separately).
    - Args: `mode` (SEARCH/LOCK), `fcu_url` (e.g., `udp://:14555@127.0.0.1:14550` for ArduPilot SITL).
- `config/`
  - `apriltag_params.yaml` — AprilTag detector config (36h11, size 0.0376 m).
- `models/`
  - `iris_with_ardupilot`, `iris_with_rgb_camera` — Drone models (RGB variant has fixed gimbal).
  - `gimbal_small_3d_fixed` — Locked-joint gimbal used by the RGB model.
- `worlds/`
  - `apriltag_test.sdf`, `testing_nodrone.sdf` — Local test worlds.
- `tag_hover_sim/` (Controllers and helpers)
  - `hover_yaw_search_v1.py` — **Frozen stable baseline** (camera-frame IBVS, do not edit).
  - `hover_yaw_search.py` — Development version (v2, active improvements).
  - `hover_yaw_search_v2.py` — Phase-1 ALIGN template.
  - `hover_yaw_search_sensor_lock.py` — **3-phase hybrid (hardware target)**. Phases: SEARCH → ALIGN → HOVER_BOX → SENSOR_HOVER.
  - `apriltag_pnp_broadcaster.py` — PnP TF broadcaster (canonical source for all packages).
  - `apriltag_tf_broadcaster.py` — Direct TF broadcaster from /detections.
- Docs: `README.md`, `QUICK_REFERENCE.md`, `PROJECT_CONTEXT.md`, `CONTROLLER_DEV_NOTES.md`.
- Key topics: `/camera/image_raw`, `/camera/camera_info`, `/detections`, `/tf` (camera → tag36h11:0), `/mavros/state`, `/mavros/setpoint_velocity/cmd_vel_unstamped`, `/hover_yaw_cmd`.

### tag_hover_two_tags
- `package.xml` — ROS 2 package manifest.
- `setup.py` — installs launch + config; registers console-scripts.
- `launch/`
  - `sim_vision_stack.launch.py` — Gazebo + camera bridge + apriltag detector + tag pose selectors.
  - `sim_lockon_backbone.launch.py` — Relative pose estimator + CSV logger.
- `config/`
  - `apriltag_params.yaml` — AprilTag detector config (36h11).
- `worlds/`
  - `apriltag_two_tags.sdf` — Two-tag test world.
- `tag_hover_two_tags/` (Nodes)
  - `tag_pose_selector.py` — Extracts PoseStamped for a specific tag ID.
  - `relative_vibration_pose.py` — Time-sync + relative transform + CSV logging.
  - `tag_oscillator.py` — Sinusoidal joint command publisher for the vibrating tag.
  - `apriltag_tf_broadcaster.py` — TF broadcaster for detections.
  - `apriltag_pnp_broadcaster.py` — PnP TF broadcaster using camera intrinsics.
- `tag_hover_controller/`
  - `hover_yaw_search.py` — Standalone yaw-search controller (not registered as a console script).
- Key topics: `/detections`, `/apriltag_ref/pose`, `/apriltag_vib/pose`, `/relative_vibration_pose`.

### ardupilot_gazebo
- Provides models/worlds for ArduPilot + Gazebo (e.g., `iris_with_ardupilot`, `zephyr` variants, parachute, gimbal worlds).
- Installed share paths (after build): `install/ardupilot_gazebo/share/ardupilot_gazebo/models` and `.../worlds`.
- Use `GZ_SIM_RESOURCE_PATH` to point to these when launching Gazebo.

## Quick topic graph (tag hover focus)
- Camera: `/camera/image_raw`, `/camera/camera_info` (bridged from Gazebo).
- Detection: `/detections` (from `apriltag_ros`), TF `camera -> tag36h11:0`.
- Control: `/hover_yaw_cmd` (debug), `/mavros/setpoint_velocity/cmd_vel_unstamped` (to FCU).
- State: `/mavros/state` (connection/mode), `/mavros/local_position/...` (if needed).

## Quick topic graph (two-tag measurement)
- Camera: `/camera/image_raw`, `/camera/camera_info` (bridged from Gazebo).
- Detection: `/detections` (from `apriltag_ros`).
- Pose selection: `/apriltag_ref/pose`, `/apriltag_vib/pose` (PoseStamped).
- Relative output: `/relative_vibration_pose` (PoseStamped, ref frame).

## Paths and env hints
- Workspace root: `~/harmonic_ws`
- Source/setup:
  ```bash
  source /opt/ros/jazzy/setup.bash
  source ~/harmonic_ws/install/setup.bash
  ```
- Models in source:
  ```bash
  export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/harmonic_ws/src/ardupilot_gazebo/models:$HOME/harmonic_ws/src/tag_hover_sim/models
  ```
- Models in install:
  ```bash
  export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/harmonic_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/models:$HOME/harmonic_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/worlds
  ```

## Reminders
- Only one MAVROS per ROS domain — duplicate causes allocator crash.
- ArduPilot SITL: must use `--out=127.0.0.1:14550 --out=127.0.0.1:14555` for MAVLink to reach MAVROS.
- Camera bridge: use full scoped Gazebo model path with `--ros-args -r` remapping (simple bridge does not work).
- `camera_frame` parameter must match exactly across PnP broadcaster and controller.
- `GZ_SIM_RESOURCE_PATH` must include `src/tag_hover_sim/models` and `src/ardupilot_gazebo/models`.
- ArduPilot SITL lives at `src/ardupilot/`; activate `drone-venv/` before running `sim_vehicle.py`.

## Build and run
```bash
# Build all packages
cd ~/harmonic_ws
colcon build --symlink-install
source install/setup.bash

# Build single package
colcon build --packages-select tag_hover_sim --symlink-install

# Full bringup: see QUICK_REFERENCE.md in each package
```
