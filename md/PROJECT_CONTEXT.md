# Project Context — tag_hover_sim

> **Scope:** Package-specific. Governs agents working inside `src/tag_hover_sim/`.
> **Parent context:** `../../PROJECT_CONTEXT.md` (workspace master — read first)
> **Last updated:** 2026-02-20

---

## 1. Package Overview

- **Package:** `tag_hover_sim`
- **Type:** ROS 2 simulation environment (ament_python)
- **Purpose:** Simulate and validate the single-tag AprilTag IBVS hover/yaw-search pipeline in Gazebo Harmonic + ArduPilot SITL before deploying to real Pixhawk hardware.

### Core concept
A drone starts in SEARCH mode (constant yaw rotation). When the AprilTag enters the camera field of view, the controller automatically switches to LOCK mode and regulates a 4-DOF relative pose: yaw alignment + forward distance + lateral centering + vertical centering. The simulation environment mirrors the real hardware pipeline exactly — camera bridge replaces V4L2, UDP replaces serial.

---

## 2. Current Objective

Validate the 3-phase hybrid controller (`hover_yaw_search_sensor_lock`) in sim as the final step before hardware deployment. The primary deliverable is a confirmed sim run where the drone transitions SEARCH → ALIGN → HOVER_BOX → SENSOR_HOVER and holds position with no active velocity commands.

---

## 3. Success Criteria

| Criterion | Measure |
|---|---|
| SEARCH → LOCK auto-switch | Tag enters view → controller switches within 1 control cycle |
| Yaw alignment | Yaw error < 0.1 rad sustained |
| Distance regulation | Hover at `target_distance ± 0.3 m` |
| Lateral centering | Lateral error < 0.1 m sustained |
| Sensor hover handoff | Zero velocity commands after HOVER_BOX equilibrium reached |
| No oscillation | No bang-bang behavior, no divergence |

---

## 4. Controller Versions — Policy

| File | Status | Policy |
|---|---|---|
| `hover_yaw_search_v1.py` | **FROZEN BASELINE** | **DO NOT EDIT.** Reference implementation. Used as ground truth for regression. |
| `hover_yaw_search.py` | Development (v2) | Active development target. Improvements go here. |
| `hover_yaw_search_v2.py` | Phase-1 ALIGN template | Intermediate; may be merged into hover_yaw_search.py |
| `hover_yaw_search_sensor_lock.py` | **Hardware target** | 3-phase supervisory FSM. Development-stable; final tuning in progress. |

**Agents must never modify `hover_yaw_search_v1.py`.** All improvements go into `hover_yaw_search.py` or `hover_yaw_search_sensor_lock.py`.

---

## 5. Architecture

### Pipeline (simulation)
```
ArduPilot SITL
    ↕ (UDP FDM, ports 9002/9003)
Gazebo Harmonic (apriltag_test.sdf)
    → ros_gz_bridge (full scoped path + remapping)
    → /camera/image_raw + /camera/camera_info
    → apriltag_ros → /detections
    → apriltag_pnp_broadcaster → TF (camera_frame → tag36h11:0)
    → hover_yaw_search controller
    → /mavros/setpoint_velocity/cmd_vel_unstamped
    → MAVROS → ArduPilot SITL (UDP, port 14555→14550)
```

### TF tree
```
world
  └── iris_with_rgb_camera/...../camera   (Gazebo model frame)
                                  └── tag36h11:0   (published by PnP broadcaster when tag visible)
```

### Key frames
- **Camera frame:** `iris_with_rgb_camera/gimbal/pitch_link/camera` — must match `-p camera_frame:=` on both broadcaster and controller.
- **Tag frame:** `tag36h11:0`
- **Body frame:** `base_link` (used for body-frame transformation in v2+)

---

## 6. Key Parameters

### Controller (`hover_yaw_search` / `hover_yaw_search_v1`)
| Parameter | Default | Notes |
|---|---|---|
| `mode` | `SEARCH` | SEARCH or LOCK |
| `rate_hz` | `20.0` | Control loop rate |
| `camera_frame` | `iris_with_rgb_camera/gimbal/pitch_link/camera` | **CRITICAL — must match TF** |
| `tag_frame` | `tag36h11:0` | Tag TF child frame |
| `body_frame` | `base_link` | Drone body frame |
| `target_distance` | `1.0` | Standoff distance in meters |
| `search_yaw` | `0.25` | Yaw rate in SEARCH (rad/s) |
| `lock_k_yaw` | `0.1` | P gain for yaw |
| `lock_k_distance` | `0.2` | P gain for forward/back (m/s per m) |
| `lock_k_lateral` | `0.1` | P gain for lateral (m/s per m) |
| `lock_k_vertical` | `0.1` | P gain for vertical (m/s per m) |
| `max_yaw_rate` | `0.6` | Max yaw rate clamp (rad/s) |
| `mavros_wait_timeout` | `10.0` | Seconds to wait for MAVROS before starting |

### AprilTag detector (`config/apriltag_params.yaml`)
- Family: `36h11`
- Tag size: `0.0376 m` (sim) — update to physical size for hardware
- Camera topics: remapped to `/camera/image_raw` and `/camera/camera_info`

---

## 7. World and Models

| Asset | Path | Notes |
|---|---|---|
| Primary world | `worlds/apriltag_test.sdf` | Iris with RGB camera + AprilTag board |
| Drone model | `models/iris_with_rgb_camera/` | Iris + fixed gimbal + front-facing camera |
| Gimbal | `models/gimbal_small_3d_fixed/` | Locked joints (no drift) |
| AprilTag | `models/apriltag_36h11_0/` | PBR albedo texture (SDF 1.9) |

**Resource path required:**
```bash
export GZ_SIM_RESOURCE_PATH=$(pwd)/src/tag_hover_sim/models:$(pwd)/src/ardupilot_gazebo/models
```

---

## 8. Launch Files

| File | Purpose | Notes |
|---|---|---|
| `sim_lockon_backbone.launch.py` | MAVROS + controller | Camera bridge, detector, PnP broadcaster run separately |
| `sim_vision_stack.launch.py` | Vision pipeline only | No MAVROS or controller |

**Arguments (`sim_lockon_backbone.launch.py`):**
- `fcu_url` — MAVLink URL (default: `udp://:14555@127.0.0.1:14550`)
- `mode` — Controller start mode (default: `SEARCH`)

---

## 9. Known Issues / Open Work

- **Lateral drift (v2):** Residual rightward drift in `hover_yaw_search.py` due to pure-P IBVS + camera–body misalignment. Root cause documented. Recommended fix: `LATERAL_DEADBAND = 0.05 m`.
- **Vertical control disabled (v2):** `lock_k_vertical` ignored; vertical held at 0.0. FCU maintains altitude.
- **Yaw gating random walk:** Intermittent lateral "bursts" when yaw error hovers near threshold. Mitigated by removing gating or tightening deadband.
- **Sim camera rate ~6-7 Hz:** Gazebo Harmonic limit at current world complexity. Acceptable for current project scope.
- **Code copies in tag_hover_two_tags:** This package is the canonical source for all controller and broadcaster nodes. Copies in `tag_hover_two_tags/tag_hover_sim/` may drift.

---

## 10. Decision Authority (Package-Level)

- **Agents may:** implement improvements in `hover_yaw_search.py`, add deadbands/clamps, add debug topics, add launch arguments, update docs.
- **Agents must NOT:** modify `hover_yaw_search_v1.py`, change `apriltag_params.yaml` tag family/size without owner approval, restructure the package.
- **Owner approval required for:** changing control architecture, introducing new dependencies, modifying SDF worlds/models, any hardware-related changes.

---

## 11. Sim Bringup Sequence

See `../../docs/LOCKON_NOTES.md` for the authoritative step-by-step launch sequence and all working commands.

Quick reference:
1. SITL: `sim_vehicle.py ... --out=127.0.0.1:14550 --out=127.0.0.1:14555`
2. Gazebo: `gz sim -r src/tag_hover_sim/worlds/apriltag_test.sdf`
3. Camera bridge: full scoped path with remapping
4. MAVROS: `ros2 launch mavros apm.launch fcu_url:=udp://:14555@127.0.0.1:14550`
5. AprilTag detector: `ros2 run apriltag_ros apriltag_node ...`
6. PnP broadcaster: `ros2 run tag_hover_sim apriltag_pnp_broadcaster ...`
7. Controller: `ros2 run tag_hover_sim hover_yaw_search_v1 ...`
8. Verify → Set GUIDED → arm → takeoff
