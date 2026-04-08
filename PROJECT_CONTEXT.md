# Project Context (Master)

> **Scope:** Workspace-level. Governs all agents and all sub-projects in this repo.
> **Sub-project contexts:** `src/tag_hover_sim/PROJECT_CONTEXT.md`, `src/tag_hover_two_tags/PROJECT_CONTEXT.md`
> **Last updated:** 2026-02-20

---

## 1. Project Overview

- **Project type:** Academic research prototype
- **Domain:** Robotics — autonomous UAV inspection, vision-based control, relative pose sensing
- **Intended users:** Project contributors and collaborators

This workspace implements two complementary UAV inspection behaviors validated in Gazebo Harmonic simulation with ArduPilot SITL, targeting deployment to a Raspberry Pi 4 + Pixhawk hardware platform:

1. **Single-tag IBVS hover** (`tag_hover_sim`): A drone autonomously searches for an AprilTag, locks on, and maintains a regulated standoff pose using image-based visual servoing.
2. **Two-tag vibration measurement** (`tag_hover_two_tags`): Dual AprilTags decouple UAV motion from target motion to measure vibration of a structure relative to a stationary reference — without requiring closed-loop flight control.

---

## 2. Product Owner Intent

- **Current objective:** Complete simulation validation of both sub-projects, then deploy the single-tag IBVS controller to real hardware (Raspberry Pi 4 + Pixhawk) for hardware experiments.
- **Why this matters now:** Simulation-to-hardware deployment is the key project milestone. The control pipeline must be proven equivalent in both environments.
- **Deadline/time pressure:** [TBD]

---

## 3. Primary Objective

- **Problem solved:** Enable a UAV to autonomously inspect a target (hover in front of it, measure vibrations on it) using only a monocular camera and AprilTag markers — robust to UAV drift and without GPS.
- **Measurable success signals:**
  - Single-tag: Drone achieves stable hover within `target_distance ± 0.3 m` and yaw error < 0.1 rad for ≥ 10 s in simulation and on hardware.
  - Two-tag: Relative vibration pose T_vib_ref is computed correctly (UAV motion canceled), CSV data recoverable for analysis, oscillation frequency detectable from CSV output.

---

## 4. Secondary Objectives

- Hardware deployment guide complete and verified (Raspberry Pi 4 + Pixhawk).
- 3-phase hybrid controller (`hover_yaw_search_sensor_lock`) validated for silent handoff to optical flow hover.
- Two-tag pipeline validated with sinusoidal oscillator in sim, producing clean CSV for FFT analysis.
- Research-quality documentation: architecture diagrams, result plots, reproducible experiment scripts.

---

## 5. Explicit Non-Goals

- No GPS-dependent navigation (target: GUIDED_NOGPS / optical flow operation).
- No multi-drone coordination.
- No obstacle avoidance.
- No adaptive/online gain tuning.
- No PX4 support — ArduPilot firmware only.
- No real-time FFT/frequency estimation in ROS nodes (post-processing of CSV is sufficient).
- Do not add CI/CD or automated test pipelines unless explicitly requested.

---

## 6. Technical Constraints

### Required stack
- **ROS 2 Jazzy** (Python 3, ament_python packages)
- **Gazebo Harmonic** (SDF 1.9, `gz-sim8`)
- **ArduPilot SITL** (ArduCopter, `gazebo-iris` frame, JSON model interface)
- **MAVROS** (APM variant: `ros2 launch mavros apm.launch`)
- **apriltag_ros** (36h11 family, `solvePnP` for 3D pose)
- **ros_gz_bridge** (camera bridge — must use full scoped path with remapping)

### Hardware target (real deployment)
- Companion computer: **Raspberry Pi 4**
- FCU: **Pixhawk** (ArduPilot firmware)
- Camera: **RealSense D455** (calibration file required)
- Serial: `/dev/ttyS0` at 57600 baud; MAVROS connects via UDP
- FCU mode: `GUIDED` (optical flow active)

### Hard constraints
- **Only one MAVROS instance per ROS domain** — duplicate causes allocator crash.
- **Camera frame_id must exactly match TF tree** — `camera_frame` parameter must be consistent across PnP broadcaster and controller.
- **`GZ_SIM_RESOURCE_PATH`** must include `src/tag_hover_sim/models` and `src/ardupilot_gazebo/models` before launching Gazebo.
- **Camera bridge**: simple `/camera` bridge does NOT work — use the full scoped model path with `--ros-args -r` remapping.
- **hover_yaw_search_v1** is the frozen stable baseline — do NOT modify it.
- **ArduPilot SITL** requires `--out=127.0.0.1:14550` to send MAVLink to MAVROS (not just FDM to Gazebo).
- Python virtual environment (`drone-venv`) is for ArduPilot SITL only — do not source it for ROS 2 nodes.
- `drone-venv/` has `COLCON_IGNORE` — do not remove it.

### Performance limits
- Controller loop rate: 20 Hz (hardware-appropriate for MAVROS velocity setpoints).
- Camera bridge: ~6–7 Hz in simulation (Gazebo Harmonic limit at current world complexity).
- Max yaw rate: 0.6 rad/s. Max forward/lateral velocity: 0.5 m/s (safety limit for indoor flight).

---

## 7. Decision Authority

| Decision | Authority |
|---|---|
| New controller architectures or phases | Product owner approval required |
| New packages or major dependencies | Product owner approval required |
| Gain tuning within existing controller parameters | Agent may propose; owner approves before hardware |
| Adding deadbands, clamps, or minor control patches | Agent may implement in v2/dev controller |
| Adding new ROS nodes or topics | Agent may implement; document in code_index.md |
| Modifying `hover_yaw_search_v1.py` | **Forbidden** — frozen baseline, never edit |
| Modifying `ai-workbench/` canonical files | **Forbidden** without promotion gate in AGENTS.md |
| Modifying Gazebo world/model SDF files | Agent may propose; owner reviews visual/physics impact |
| Hardware deployment (real flight tests) | Product owner only |

---

## 8. Quality Bar

- **Prototype-grade** (not production): prioritize correctness and reproducibility over robustness.
- **No automated tests** required, but each node should have documented verification commands.
- **Code comments**: required for any non-obvious math (TF inversions, axis conventions, frame transforms).
- **Docs**: every change to a node or launch file should be reflected in `docs/code_index.md` and the relevant sub-project notes.
- **Axis convention**: explicitly document any sign convention or frame assumption at the point of use.

---

## 9. Delegation Plan

### Current phase: Simulation validation → Hardware deployment preparation

| Role | Scope |
|---|---|
| **Architect Owner** | Phase planning, scope decisions, handoff packets |
| **Backend Engineer** | ROS node implementation, controller logic, launch files |
| **Debug Reliability** | MAVROS issues, TF errors, detection pipeline failures |
| **Docs Knowledge** | code_index.md, LEARNING_LOG.md, decisions.md updates |
| **QA Test Engineer** | Verification command checklists, parameter bounds testing |
| **Research Decisions** | Controller architecture comparisons, IBVS literature |

### Handoff requirement
All implementation tasks must include:
- Verification commands showing expected output.
- Any new parameters documented with defaults and ranges.
- `code_index.md` updated if new nodes/topics are added.

---

## 10. Key Artifacts & Links

| Artifact | Location |
|---|---|
| This context (master) | `PROJECT_CONTEXT.md` |
| Single-tag sim context | `src/tag_hover_sim/PROJECT_CONTEXT.md` |
| Two-tag sim context | `src/tag_hover_two_tags/PROJECT_CONTEXT.md` |
| Sim bringup runbook | `docs/LOCKON_NOTES.md` |
| Hardware deployment guide | `docs/DRONE_FLIGHT_STACK_REAL.md` |
| Two-tag notes | `docs/TWO_TAG_NOTES.md` |
| Code map | `docs/code_index.md` |
| Progress log | `docs/PROGRESS_LOG.md` |
| ArduPilot SITL setup | `docs/ardupilot_setup.md` |
| Flight operations guide | `docs/flight_guide.md` |
| Task backlog | `tasks.md` (create from template) |
| Decision log | `decisions.md` (create from template) |
| Learning log | `LEARNING_LOG.md` (create from template) |

---

## 11. Current State

### What Works
- **Single-tag pipeline (sim):** Full SITL → Gazebo → camera bridge → apriltag_ros → PnP TF → controller → MAVROS pipeline verified end-to-end.
- **SEARCH → LOCK auto-switch:** Drone spins in SEARCH, automatically locks when AprilTag enters view.
- **hover_yaw_search_v1:** Stable camera-frame IBVS with yaw + distance + lateral control. Frozen as baseline.
- **hover_yaw_search_sensor_lock:** 3-phase supervisory state machine (SEARCH → ALIGN → HOVER_BOX → SENSOR_HOVER) implemented and ready for hardware.
- **Two-tag measurement pipeline:** All nodes implemented — `tag_pose_selector`, `relative_vibration_pose`, `tag_oscillator`, `tag_overlay`, `apriltag_pnp_broadcaster`.
- **Two-tag CSV logging:** `~/.ros/tag_hover_two_tags/relative_vibration_*.csv` with timestamps and SE(3) relative pose.
- **ardupilot_gazebo plugin:** Built and installed; provides Iris drone physics and sensor models.

### Known Issues / Open Work
- Two-tag end-to-end test (oscillator + relative pose + CSV) not yet run after tag size change to 5 inches (0.127 m).
- hover_yaw_search_v2 has residual lateral drift (understood root cause: pure-P IBVS + camera–body misalignment). Mitigation: lateral deadband 0.05 m.
- hover_yaw_search_v2 vertical control currently disabled (held at 0.0).
- Code duplication: `tag_hover_two_tags` contains copies of `tag_hover_sim` nodes — canonical source is `tag_hover_sim`. Copies may drift.
- Loose Python files at `src/` root (`apriltag_pnp_broadcaster.py`, `apriltag_tf_broadcaster.py`, `tag_overlay.py`) — origin unclear, may be stale.
- `tag_hover_two_tags` lacks an oscillator launch in `sim_vision_stack.launch.py` (runs separately via `tag_oscillator.launch.py`).

---

## 12. Learning Priorities

- **MAVROS allocator crash:** Caused by duplicate MAVROS instance or topic type mismatch. Fix: single MAVROS per domain, correct `--out` port on SITL. See `docs/LOCKON_NOTES.md`.
- **Camera bridge silent failure:** Simple `/camera` bridge produces no data; must use full scoped Gazebo model path with remapping. See `docs/LOCKON_NOTES.md`.
- **Camera frame mismatch:** `camera_frame` parameter must exactly match what the PnP broadcaster uses as the TF parent. See `docs/LOCKON_NOTES.md`.
- **Pure-P controller steady-state bias:** Lateral deadband is the recommended minimal fix.
- **Prevent:** Always verify `GZ_SIM_RESOURCE_PATH` includes all model paths before `gz sim`. Models load silently without them (world loads but drone/tag textures fail).
