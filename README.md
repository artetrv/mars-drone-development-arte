# harmonic_ws

ROS 2 Jazzy + Gazebo Harmonic + ArduPilot SITL simulation workspace for AprilTag-based UAV inspection research.

**Stack:** ROS 2 Jazzy · Gazebo Harmonic · ArduPilot SITL · MAVROS
**Hardware target:** Raspberry Pi 5 (companion) + Pixhawk (ArduPilot firmware)

---

## What's in this repo

### Simulation packages (`src/`)

| Package | Purpose |
|---|---|
| `tag_hover_sim` | Single-tag IBVS hover/yaw-search — sim and hardware controller |
| `tag_hover_two_tags` | Dual-tag vibration measurement — cancels UAV motion from relative pose |
| `ardupilot_gazebo` | C++ Gazebo plugin + Iris drone models (submodule) |
| `ardupilot` | ArduPilot firmware + SITL (submodule) |

### Key docs (`docs/`)

| File | What it covers |
|---|---|
| `LOCKON_NOTES.md` | **Primary sim runbook** — full bringup, working commands, troubleshooting |
| `DRONE_FLIGHT_STACK_REAL.md` | Hardware deployment guide (Raspberry Pi 5 + Pixhawk) |
| `TWO_TAG_NOTES.md` | Two-tag vibration measurement pipeline |
| `ardupilot_setup.md` | SITL install, drone-venv, Gazebo plugin |
| `flight_guide.md` | Arming, modes, common warnings |
| `code_index.md` | Full map of packages, nodes, topics, launch files |
| `PROGRESS_LOG.md` | Session-by-session work log |

---

## Quick start (tag_hover_sim)

```bash
# Build
colcon build --symlink-install
source install/setup.bash

# Full terminal sequence → see:
src/tag_hover_sim/QUICK_REFERENCE.md
```

## Quick start (two-tag measurement)

```bash
# 2-terminal quick start (measurement only)
ros2 launch tag_hover_two_tags sim_vision_stack.launch.py
ros2 launch tag_hover_two_tags sim_lockon_backbone.launch.py

# Full reference → see:
src/tag_hover_two_tags/QUICK_REFERENCE.md
```

---

## Project context

- `PROJECT_CONTEXT.md` — master scope, objectives, decision authority, known issues
- `src/tag_hover_sim/PROJECT_CONTEXT.md` — single-tag sim scope
- `src/tag_hover_two_tags/PROJECT_CONTEXT.md` — two-tag measurement scope

---

## Sanity checks

```bash
# MAVROS connected?
ros2 topic echo /mavros/state --once

# Camera publishing?
ros2 topic hz /camera/image_raw

# Tag detected?
ros2 topic echo /detections --no-arr

# TF available?
ros2 run tf2_ros tf2_echo iris_with_rgb_camera/gimbal/pitch_link/camera tag36h11:0
```
