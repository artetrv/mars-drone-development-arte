# START HERE — Project Guide for `harmonic_ws`

Date: 2025-12-20  
Stack: ROS 2 Jazzy + Gazebo Harmonic + ArduPilot SITL + MAVROS  
Goal: Single entrypoint for this repo, with pointers into per-project docs.

---

## How to use this guide
- Read this first to understand what’s in the repo right now.
- Jump to the project-specific docs (e.g., Tag Hover Sim) for detailed steps.
- Use the shared guides for common tasks (ArduPilot setup, flight ops, code map).

---

## Repo overview (docs/)
- `docs/ALL_WORK_SUMMARY.md` — Narrative of recent work and status.
- `docs/LOCKON_NOTES.md` — AprilTag hover/yaw-search integration notes (Gazebo + ArduPilot SITL).
- `docs/ardupilot_setup.md` — ArduPilot SITL install + plugin setup for Harmonic.
- `docs/flight_guide.md` — How to fly (arming, modes, expected warnings).
- `docs/code_index.md` — Map of packages, launch files, nodes, worlds, topics.
- `docs/DRONE_FLIGHT_STACK_REAL.md` — Real hardware stack notes.
- `THESIS_SYNC_AGENT.md` — Reserved/planned (currently empty).

---

## Current project focus: Tag Hover Simulation
Location: `src/tag_hover_sim/`

- Purpose: Simulate AprilTag-based hover/yaw-search with ArduPilot SITL before deploying to hardware.
- Key files:
  - `launch/sim_lockon_backbone.launch.py` — Minimal launch: MAVROS + `hover_yaw_search` (camera bridge, tag detector, and PnP TF run separately).
  - `worlds/` — AprilTag test worlds (e.g., `apriltag_test.sdf`).
  - `models/iris_with_ardupilot`, `models/iris_with_rgb_camera` — Drone models; the RGB variant carries a fixed gimbal camera.
  - `models/gimbal_small_3d_fixed` — Fixed (locked joints) gimbal used by the RGB camera model.
  - `config/apriltag_params.yaml` — AprilTag detector config.
  - `tag_hover_sim/hover_yaw_search.py` — Controller node: SEARCH yaw sweep; LOCK uses TF tag yaw error.
- Integration notes: see `docs/LOCKON_NOTES.md` for setup, resource paths, and step-by-step bringup.

### Typical launch flow (sim)
1) Start SITL (set `--out` ports, e.g., 14550/14555).  
2) Launch `sim_lockon_backbone.launch.py` with correct `fcu_url`.  
3) Bridge camera → ROS (ros_gz_bridge), run `apriltag_ros` with `apriltag_params.yaml`, run PnP TF broadcaster.  
4) Check `/mavros/state` (connected + GUIDED/LOITER), arm, take off. SEARCH rotates until tag is seen; LOCK centers yaw on the tag.  

---

## New experiment: Two-Tag Relative Pose Measurement
Location: `src/tag_hover_two_tags/`

- Purpose: Measure vibration/relative motion by differencing two AprilTag poses (reference + vibrating tag).
- Key files:
  - `launch/sim_vision_stack.launch.py` — Gazebo + camera bridge + AprilTag detector + pose selectors.
  - `launch/sim_lockon_backbone.launch.py` — Relative pose estimator + CSV logger.
  - `worlds/apriltag_two_tags.sdf` — Two-tag test world.
  - `tag_hover_two_tags/relative_vibration_pose.py` — Relative transform + logging.
- Topics: `/apriltag_ref/pose`, `/apriltag_vib/pose`, `/relative_vibration_pose`.

---

## Future pattern: per-project docs
Each project directory should carry a brief guide (like `docs/LOCKON_NOTES.md`) covering:
- Purpose and expected behavior.  
- Key models/worlds/launch files.  
- Required bridges/detectors/controllers.  
- Known issues and troubleshooting.  
Place the guide alongside the project and link it from this START HERE.

---

## Shared guides (apply to all projects)
- `docs/ardupilot_setup.md` — Prepare SITL, build/install ArduPilot Gazebo plugin, Python venv/MAVProxy.  
- `docs/flight_guide.md` — Arming/modes, common warnings, verification checks.  
- `docs/code_index.md` — Quick lookup for where launches/nodes/models live.  

---

## Quick sanity commands
Rebuild:
```bash
colcon build --symlink-install --packages-select tag_hover_sim
source install/setup.bash
```
Check MAVROS connection:
```bash
ros2 topic echo /mavros/state --once
```
Check camera bridge:
```bash
ros2 topic hz /camera/image_raw
```
Switch controller mode:
```bash
ros2 param set /hover_yaw_search mode LOCK
```

---

## Where to read next
- Tag Hover Sim details: `docs/LOCKON_NOTES.md`  
- Setup: `docs/ardupilot_setup.md`  
- Flying: `docs/flight_guide.md`  
- Code map: `docs/code_index.md`  
- Real hardware stack: `docs/DRONE_FLIGHT_STACK_REAL.md`  
