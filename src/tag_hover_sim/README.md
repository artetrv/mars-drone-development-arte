# tag_hover_sim

ROS 2 package — AprilTag hover/yaw-search simulation in Gazebo Harmonic + ArduPilot SITL.

Mirrors the real Raspberry Pi 5 + Pixhawk hardware pipeline exactly. Only two things differ in sim: the camera source (Gazebo bridge instead of V4L2) and the FCU link (UDP instead of serial).

## Architecture

```
ArduPilot SITL ↔ Gazebo Harmonic (UDP FDM 9002/9003)
    Gazebo camera → ros_gz_bridge → /camera/image_raw + /camera/camera_info
    → apriltag_ros → /detections
    → apriltag_pnp_broadcaster → TF (camera_frame → tag36h11:0)
    → hover_yaw_search controller → /mavros/setpoint_velocity/cmd_vel_unstamped
    → MAVROS → ArduPilot SITL (UDP 14555→14550)
```

## Prerequisites

```bash
sudo apt install \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-mavros \
  ros-jazzy-mavros-extras \
  ros-jazzy-apriltag-ros
```

## Build

```bash
cd ~/your_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tag_hover_sim --symlink-install
source install/setup.bash
```

## Bringup (7 terminals)

**See `QUICK_REFERENCE.md` for the full copy-paste terminal sequence.**

Full step-by-step notes including troubleshooting: `docs/LOCKON_NOTES.md`.

### Overview

| Terminal | Component | Sim or both |
|---|---|---|
| 1 | ArduPilot SITL | Sim only |
| 2 | Gazebo Harmonic | Sim only |
| 3 | Camera bridge (ros_gz_bridge) | Sim only |
| 4 | MAVROS | Both |
| 5 | apriltag_ros detector | Both |
| 6 | PnP TF broadcaster | Both |
| 7 | hover controller | Both |

## Controller versions

| Executable | Status | Use for |
|---|---|---|
| `hover_yaw_search_v1` | **Frozen baseline** — do not edit | Flight testing, demos, regression |
| `hover_yaw_search` | Development (v2) | Iterating improvements |
| `hover_yaw_search_sensor_lock` | **Hardware target** | 3-phase FSM for Pixhawk deployment |

## Key parameters

| Parameter | Default | Notes |
|---|---|---|
| `camera_frame` | `iris_with_rgb_camera/gimbal/pitch_link/camera` | **Must match PnP broadcaster** |
| `tag_frame` | `tag36h11:0` | Tag TF child frame |
| `target_distance` | `1.0` | Desired standoff (meters) |
| `mode` | `SEARCH` | `SEARCH` or `LOCK` |
| `lock_k_yaw` | `0.1` | P gain for yaw |
| `lock_k_distance` | `0.2` | P gain for forward/back |
| `lock_k_lateral` | `0.1` | P gain for lateral |
| `max_yaw_rate` | `0.6` | Max yaw rate clamp (rad/s) |

## Verification commands

```bash
# Camera publishing (~6-7 Hz in sim)
ros2 topic hz /camera/image_raw

# AprilTag detections (point camera at tag)
ros2 topic echo /detections --no-arr

# TF from camera to tag
ros2 run tf2_ros tf2_echo iris_with_rgb_camera/gimbal/pitch_link/camera tag36h11:0

# MAVROS connected
ros2 topic echo /mavros/state --once

# Controller output
ros2 topic echo /hover_yaw_cmd
```

## Arm and fly

Once all nodes are up, arm and take off through the **MAVProxy console** that opened with SITL (same workflow as real hardware with a GCS):

```
mode GUIDED
arm throttle
takeoff 5
```

Controller starts in SEARCH (constant yaw). When the tag enters camera view → auto-switches to LOCK.

> **ROS alternative** (useful for scripting):
> ```bash
> ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
> ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
> ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 5.0}"
> ```

## Documentation

- `QUICK_REFERENCE.md` — copy-paste terminal commands
- `PROJECT_CONTEXT.md` — package scope, decision authority, controller policy
- `docs/LOCKON_NOTES.md` — full bringup guide, troubleshooting, 3-phase controller
- `docs/DRONE_FLIGHT_STACK_REAL.md` — hardware deployment guide
- `CONTROLLER_DEV_NOTES.md` — v2 controller development notes and diagnosis
