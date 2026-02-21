# Project Context — tag_hover_two_tags

> **Scope:** Package-specific. Governs agents working inside `src/tag_hover_two_tags/`.
> **Parent context:** `../../PROJECT_CONTEXT.md` (workspace master — read first)
> **Last updated:** 2026-02-20

---

## 1. Package Overview

- **Package:** `tag_hover_two_tags`
- **Type:** ROS 2 simulation environment (ament_python)
- **Purpose:** Measure the relative motion (vibration) of a target object with respect to a stationary world reference using dual AprilTag pose estimation — robust to UAV motion, attitude drift, and camera vibration.

### Core concept
This is a **measurement system**, not a flight controller. Two AprilTags are simultaneously visible to the onboard camera:
- **Tag A (Reference):** Mounted on a static wall/structure. Defines a local world frame.
- **Tag B (Vibrating):** Mounted on the moving/vibrating target. Motion is what we want to measure.

The relative transform T_vib_ref = inv(T_ref_cam) · T_vib_cam cancels all UAV motion, leaving only the true relative vibration signal. This is logged to CSV for post-processing (FFT, amplitude analysis).

**Key insight:** Closed-loop flight control is NOT required. The drone holds position roughly (LOITER/GUIDED/manual). The math removes UAV imprecision from the measurement.

---

## 2. Current Objective

Complete the first end-to-end validation run: oscillate the simulated vibrating tag using `tag_oscillator`, capture the relative pose signal via `relative_vibration_pose`, verify the CSV output contains a clean sinusoidal signal, and confirm UAV motion is canceled.

---

## 3. Success Criteria

| Criterion | Measure |
|---|---|
| Both tags detected simultaneously | `/detections` shows both tag IDs when in view |
| Pose selectors publishing | `/apriltag_ref/pose` and `/apriltag_vib/pose` publishing at camera rate |
| Relative pose computed | `/relative_vibration_pose` publishing at sync rate |
| UAV motion cancellation | Static drone: relative pose near identity (< 0.01 m translation noise) |
| Oscillator visible in data | CSV x or y column shows sinusoidal signal at oscillator frequency |
| CSV logging functional | File written to `~/.ros/tag_hover_two_tags/relative_vibration_*.csv` |

---

## 4. Architecture

### Pipeline
```
Gazebo Harmonic (apriltag_two_tags.sdf)
    → ros_gz_bridge (camera bridge)
    → /camera/image_raw + /camera/camera_info
    → apriltag_ros → /detections
    → tag_pose_selector (ref, ID=0) → /apriltag_ref/pose  (PoseStamped)
    → tag_pose_selector (vib, ID=1) → /apriltag_vib/pose  (PoseStamped)
    → relative_vibration_pose → /relative_vibration_pose (PoseStamped) + CSV

Sim only:
    → tag_oscillator → /model/apriltag_vib_oscillator/joint/oscillator_joint/cmd_pos

Optional flight stack (if testing with SITL):
    → hover_yaw_search_v1 → MAVROS → ArduPilot SITL
```

### Coordinate math (core)
```
T_ref_cam = pose from apriltag_ros for reference tag (camera → ref)
T_vib_cam = pose from apriltag_ros for vibrating tag (camera → vib)

T_vib_ref = inv(T_ref_cam) · T_vib_cam
          = relative pose of vibrating tag in reference tag frame
          → UAV motion, attitude, and camera vibration are all canceled
```

### TF tree (when PnP broadcaster is running)
```
world
  └── camera_frame
        ├── tag36h11:0   (reference tag)
        └── tag36h11:1   (vibrating tag)
```

---

## 5. Key Nodes

| Node | Executable | Input | Output | Notes |
|---|---|---|---|---|
| `tag_pose_selector` | `tag_pose_selector` | `/detections` | `/apriltag_ref/pose`, `/apriltag_vib/pose` | Params: `ref_tag_id` (0), `vib_tag_id` (1) |
| `relative_vibration_pose` | `relative_vibration_pose` | `/apriltag_ref/pose`, `/apriltag_vib/pose` | `/relative_vibration_pose`, CSV | Time-synchronized via `message_filters` |
| `tag_oscillator` | `tag_oscillator` | — | Gazebo joint cmd topic | **Sim only.** Sinusoidal position command |
| `tag_overlay` | `tag_overlay` | `/camera/image_raw`, `/detections` | `/image_with_tags` | Optional debug visualization |
| `apriltag_pnp_broadcaster` | `apriltag_pnp_broadcaster` | `/detections`, `/camera/camera_info` | TF tree | Optional — apriltag_ros already provides detection poses |
| `hover_yaw_search` (controller) | `hover_yaw_search` | MAVROS state, TF | velocity setpoints | **Optional** — only if flying with SITL |

---

## 6. Key Parameters

### tag_pose_selector
| Parameter | Default | Notes |
|---|---|---|
| `ref_tag_id` | `0` | AprilTag ID for reference tag |
| `vib_tag_id` | `1` | AprilTag ID for vibrating tag |
| `camera_frame` | `iris_with_rgb_camera/gimbal/pitch_link/camera` | Must match world model |

### relative_vibration_pose
| Parameter | Default | Notes |
|---|---|---|
| `ref_pose_topic` | `/apriltag_ref/pose` | Reference tag pose input |
| `vib_pose_topic` | `/apriltag_vib/pose` | Vibrating tag pose input |
| `csv_dir` | `~/.ros/tag_hover_two_tags/` | CSV output directory |
| `csv_basename` | `relative_vibration` | CSV filename prefix |

### tag_oscillator
| Parameter | Default | Notes |
|---|---|---|
| `frequency` | `[TBD Hz]` | Oscillation frequency (sinusoidal) |
| `amplitude` | `[TBD m]` | Oscillation amplitude |
| `joint_cmd_topic` | `/model/apriltag_vib_oscillator/joint/oscillator_joint/cmd_pos` | Gazebo joint command topic |

### AprilTag detector (`config/apriltag_params.yaml`)
- Family: `36h11`
- Tag size: `0.127 m` (5 inches) — updated 2026-02-16
- Both tags (IDs 0 and 1) detected by a single `apriltag_ros` node

---

## 7. World and Models

| Asset | Path | Notes |
|---|---|---|
| Primary world | `worlds/apriltag_two_tags.sdf` | Iris + static ref tag + vibrating tag on prismatic joint |
| Reference tag | `models/apriltag_36h11_0/` | Static, ID=0 |
| Vibrating tag | `models/Apriltag36_11_00001/` | ID=1, driven by oscillator joint |
| Iris with camera | `models/iris_with_rgb_camera/` | Same as tag_hover_sim |

**Tag configuration:**
- Reference tag (ID 0): static, mounted on left
- Vibrating tag (ID 1): right side, driven by a prismatic joint in the SDF world
- Both tags: 5 inches (0.127 m) — must match `tag_size` in `apriltag_params.yaml`

**Resource path required:**
```bash
export GZ_SIM_RESOURCE_PATH=$(pwd)/src/tag_hover_two_tags/models:$(pwd)/src/ardupilot_gazebo/models
```

---

## 8. Launch Files

| File | Purpose | Starts |
|---|---|---|
| `sim_vision_stack.launch.py` | Full vision pipeline | Gazebo + camera bridge + apriltag detector + pose selectors |
| `sim_lockon_backbone.launch.py` | Measurement stack | `relative_vibration_pose` + CSV logger |
| `hover_controller.launch.py` | Optional flight controller | `hover_yaw_search` controller with configurable params |
| `tag_oscillator.launch.py` | Oscillator only | `tag_oscillator` node |

### Typical 2-terminal quick start (measurement only):
```bash
# Terminal 1
ros2 launch tag_hover_two_tags sim_vision_stack.launch.py
# Terminal 2
ros2 launch tag_hover_two_tags sim_lockon_backbone.launch.py
```

---

## 9. CSV Output Format

File: `~/.ros/tag_hover_two_tags/relative_vibration_<timestamp>.csv`

| Column | Type | Description |
|---|---|---|
| `stamp_sec` | float | Sync timestamp (seconds) |
| `ref_stamp_sec` | float | Reference pose message timestamp |
| `vib_stamp_sec` | float | Vibrating pose message timestamp |
| `x` | float | Relative translation X (meters) in ref frame |
| `y` | float | Relative translation Y (meters) in ref frame |
| `z` | float | Relative translation Z (meters) in ref frame |
| `roll` | float | Relative roll (radians) |
| `pitch` | float | Relative pitch (radians) |
| `yaw` | float | Relative yaw (radians) |

**Post-processing:** Apply FFT to x(t), y(t), or z(t) column to extract vibration frequency and amplitude.

---

## 10. Known Issues / Open Work

- **End-to-end test pending:** Full run (oscillator + vision stack + relative pose + CSV) not yet executed after tag resize to 5 inches (2026-02-16).
- **Code duplication:** This package contains copies of `tag_hover_sim` nodes (`hover_yaw_search` variants, `apriltag_pnp_broadcaster`). Canonical source is `tag_hover_sim`. If those nodes are updated in `tag_hover_sim`, the copies here may fall behind.
- **Single apriltag_ros node:** Both tags are detected by one node instance. If per-tag tuning is needed later, consider running two separate nodes with filtered tag families.
- **No hardware path defined:** Two-tag measurement on real hardware is future work. Camera calibration and physical tag sizes will differ from simulation.

---

## 11. Decision Authority (Package-Level)

- **Agents may:** fix detection issues, improve CSV logging, add oscillator parameters, add debug topics, update launch files, add visualization nodes.
- **Agents must NOT:** change the SE(3) math in `relative_vibration_pose.py` without owner review, change tag IDs in the world without updating detector params, modify `tag_hover_sim` copies in this package without syncing from canonical source.
- **Owner approval required for:** hardware deployment plan, FFT/frequency estimation pipeline design, new measurement modalities, changes to world SDF that affect experimental repeatability.

---

## 12. Difference From tag_hover_sim

| Aspect | tag_hover_sim | tag_hover_two_tags |
|---|---|---|
| Primary goal | Flight control (IBVS hover) | Sensing (vibration measurement) |
| Tags | 1 tag, ID=0 | 2 tags: ID=0 (ref) + ID=1 (vib) |
| Flight control | Required (MAVROS + controller) | Optional — drone holds position manually or in LOITER |
| Output | Velocity setpoints → MAVROS | CSV log of T_vib_ref(t) |
| UAV motion handling | Compensated by visual servoing | Canceled mathematically via pose differencing |
| Hardware readiness | Deployment-ready (Phase 1 complete) | Sim validation pending; hardware TBD |
