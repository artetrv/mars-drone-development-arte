# Two-Tag Measurement Notes

## Purpose
Two-tag AprilTag pipeline that computes a relative transform $T_{vib}^{ref}$ to isolate vibrating motion from camera/UAV drift. Intended for vibration measurement experiments with a reference tag and a vibrating tag in the same camera view.

## Pipeline overview
1) Camera → `/camera/image_raw`, `/camera/camera_info`
2) `apriltag_ros` → `/detections`
3) `tag_pose_selector` (ref + vib) → `/apriltag_ref/pose`, `/apriltag_vib/pose`
4) `relative_vibration_pose` → `/relative_vibration_pose` + CSV log
5) `tag_oscillator` → Gazebo joint command (sim only)
6) `tag_overlay` → `/image_with_tags` (RViz2 overlay, optional)
6) `hover_yaw_search` (controller) → `/mavros/setpoint_velocity/cmd_vel_unstamped` (optional, if flying)

## Launch patterns

### Quick start (2 terminals — measurement only)
```bash
# Terminal 1: Vision stack
ros2 launch tag_hover_two_tags sim_vision_stack.launch.py

# Terminal 2: Relative pose + logger
ros2 launch tag_hover_two_tags sim_lockon_backbone.launch.py
```

### Full flight stack (5 terminals — with SITL + MAVROS)
```bash
# Terminal 1: SITL
cd ~/harmonic_ws/src/ardupilot
source ../../drone-venv/bin/activate
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=127.0.0.1:14550 --out=127.0.0.1:14555

# Terminal 2: Vision stack
ros2 launch tag_hover_two_tags sim_vision_stack.launch.py

# Terminal 3: MAVROS
ros2 launch mavros apm.launch fcu_url:=udp://:14555@127.0.0.1:14550

# Terminal 4: Relative pose + logger
ros2 launch tag_hover_two_tags sim_lockon_backbone.launch.py

# Terminal 5: Controller (stable baseline from tag_hover_sim)
ros2 run tag_hover_sim hover_yaw_search_v1 --ros-args \
  -p body_frame:=base_link \
  -p camera_frame:=iris_with_rgb_camera/gimbal/pitch_link/camera \
  -p tag_frame:=tag36h11:1 \
  -p mavros_prefix:=/mavros \
  -p mode:=SEARCH \
  -p rate_hz:=20.0 \
  -p search_yaw:=0.25 \
  -p lock_k_yaw:=0.1 \
  -p lock_k_distance:=0.2 \
  -p lock_k_lateral:=0.1 \
  -p lock_k_vertical:=0.1 \
  -p target_distance:=1.0 \
  -p max_yaw_rate:=0.6 \
  -p mavros_wait_timeout:=10.0
```

### Debug split (7–9 terminals — component inspection)
See [src/tag_hover_two_tags/QUICK_REFERENCE.md](../src/tag_hover_two_tags/QUICK_REFERENCE.md) for full terminal breakdown.

### RViz2 overlay view (optional)
Run the overlay node and then add an Image display in RViz2 pointing at `/image_with_tags`.

## Worlds and models
- World: `src/tag_hover_two_tags/worlds/apriltag_two_tags.sdf`
- Tags:
  - Reference tag (ID 0) is static on the left.
  - Vibrating tag (ID 1) is on the right and driven by a prismatic joint.
  - Tag size: **5 inches (0.127 m)**
- Oscillator joint command topic (sim):
  - `/model/apriltag_vib_oscillator/joint/oscillator_joint/cmd_pos`

## Key nodes and topics
- `tag_pose_selector` → `/apriltag_ref/pose`, `/apriltag_vib/pose`
- `tag_oscillator` → joint command topic (sim)
- `relative_vibration_pose` → `/relative_vibration_pose` (CSV logged)
- `hover_yaw_search_v1` (controller, optional for flight testing)
  - Subscribes: `/mavros/state`, TF `camera → tag`
  - Publishes: `/mavros/setpoint_velocity/cmd_vel_unstamped`
  - Modes: SEARCH (constant yaw) or LOCK (distance + yaw + lateral + vertical)

## CSV logging
- Default output: `~/.ros/tag_hover_two_tags/relative_vibration_*.csv`
- Columns: `stamp_sec`, `ref_stamp_sec`, `vib_stamp_sec`, `x`, `y`, `z`, `roll`, `pitch`, `yaw`
- Configurable via `csv_dir` and `csv_basename` parameters

## Quick checks
```bash
ros2 topic echo /apriltag_ref/pose --once
ros2 topic echo /apriltag_vib/pose --once
ros2 topic echo /relative_vibration_pose --once
ros2 topic list  # Verify all expected topics are present
```

## Common issues
- **Tag IDs mismatch**: Verify `ref_tag_id` and `vib_tag_id` parameters match world configuration.
- **No detections**: Check camera bridge is running and `/camera/image_raw` is publishing at ~6-7 Hz.
- **Pose selectors silent**: If detections exist but pose topics don't publish, check the camera frame parameter consistency.
- **MAVROS connection failed**: Ensure SITL is running with correct `--out` ports (14550/14555).
- **Controller won't lock**: Verify TF is flowing (`ros2 tf2_tools view_frames`) and camera/tag frames match your launch parameters.
