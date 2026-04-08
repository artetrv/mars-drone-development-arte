# tag_hover_two_tags

Two-tag AprilTag relative pose measurement stack with oscillating vibration for Gazebo Harmonic + ArduPilot SITL.

## Overview

This package measures relative motion of a vibrating target tag with respect to a fixed reference tag using a single onboard camera. The vibrating tag oscillates side-to-side with configurable frequency and amplitude. The core output is the relative transform:

$T_{vib}^{ref} = (T_{ref}^{cam})^{-1} \cdot T_{vib}^{cam}$

This cancels UAV motion and camera drift, turning AprilTag detections into a vibration sensing signal.

**Physical Setup:**
- **Reference tag (ID 0)**: Static tag positioned on the left
- **Vibrating tag (ID 1)**: Oscillating tag on the right, controlled by a prismatic joint
- Both tags visible in camera frame simultaneously

**Architecture (sim or hardware):**
```
Camera node
  -> /camera/image_raw, /camera/camera_info
  -> apriltag_ros
  -> /detections
  -> tag_pose_selector (ref + vib)
  -> /apriltag_ref/pose, /apriltag_vib/pose
  -> relative_vibration_pose
  -> /relative_vibration_pose + CSV log

tag_oscillator
  -> /apriltag_vib_oscillator/cmd_pos
  -> Gazebo joint controller (side-to-side motion)
```

## Prerequisites

- ROS 2 Jazzy
- Gazebo Harmonic
- `ros_gz_bridge`
- `apriltag_ros`
- `apriltag_msgs`
- `message_filters`

Install missing dependencies:

```bash
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-apriltag-ros ros-jazzy-message-filters
```

## Build

```bash
cd ~/your_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tag_hover_two_tags --symlink-install
source install/setup.bash
```

## Usage (Sim)

1) **Vision stack + Oscillator** (Gazebo + camera bridge + AprilTag + tag pose selectors + oscillator)
```bash
source /opt/ros/jazzy/setup.bash
source ~/your_ws/install/setup.bash
ros2 launch tag_hover_two_tags sim_vision_stack.launch.py
```

**Customize oscillation parameters:**
```bash
ros2 launch tag_hover_two_tags sim_vision_stack.launch.py \
  oscillation_freq:=2.0 \
  oscillation_amp:=0.05
```
- `oscillation_freq`: Oscillation frequency in Hz (default: 1.0)
- `oscillation_amp`: Oscillation amplitude in meters (default: 0.08, range: ±0.1m)
- `ref_tag_id`: Reference tag ID (default: 0)
- `vib_tag_id`: Vibrating tag ID (default: 1)

2) **Relative pose estimator + logger**
```bash
source /opt/ros/jazzy/setup.bash
source ~/your_ws/install/setup.bash
ros2 launch tag_hover_two_tags sim_lockon_backbone.launch.py
```

3) **Optional: ArduPilot SITL** (only if you want a full flight stack)
```bash
cd ~/ardupilot/ArduCopter
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=127.0.0.1:14550 --out=127.0.0.1:14555
```

## Topics

- `/detections` (AprilTag detections)
- `/apriltag_ref/pose` (PoseStamped for reference tag)
- `/apriltag_vib/pose` (PoseStamped for vibrating tag)
- `/relative_vibration_pose` (PoseStamped in reference frame)
- `/apriltag_vib_oscillator/cmd_pos` (Float64 joint position command)

## Parameters

### tag_pose_selector
- tag_id (int) - AprilTag ID to publish
- detections_topic (string)
- output_topic (string)
- camera_frame (string) - override header frame

### relative_vibration_pose
- ref_pose_topic (string)
- vib_pose_topic (string)
- output_topic (string)
- reference_frame (string) - if empty, uses ref tag frame_id
- csv_dir (string) - default ~/.ros/tag_hover_two_tags
- csv_basename (string)
- sync_queue_size (int)
- sync_slop_sec (float)

## World notes

The default world is [src/tag_hover_two_tags/worlds/apriltag_two_tags.sdf](src/tag_hover_two_tags/worlds/apriltag_two_tags.sdf) and includes two tag models. The second tag currently reuses the same texture as tag 0; replace it with a unique tag model/texture for correct ID separation.

## Verification

```bash
ros2 topic list
ros2 topic echo /apriltag_ref/pose --once
ros2 topic echo /apriltag_vib/pose --once
ros2 topic echo /relative_vibration_pose --once
```

## Documentation

- [src/tag_hover_two_tags/QUICK_REFERENCE.md](src/tag_hover_two_tags/QUICK_REFERENCE.md)
- [docs/code_index.md](docs/code_index.md)

## License

MIT
