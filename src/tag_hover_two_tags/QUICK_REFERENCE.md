# Simulation Quick Reference (Two-Tag Measurement with Oscillation)

## Quick Start (2 terminals — Measurement only)

### Terminal 1: Vision stack + Oscillator
```bash
cd ~/harmonic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tag_hover_two_tags sim_vision_stack.launch.py
```

Optional parameters:
- `oscillation_freq:=2.0` — Set oscillation frequency (Hz)
- `oscillation_amp:=0.05` — Set amplitude (meters)
- `ref_tag_id:=0` — Reference tag ID
- `vib_tag_id:=1` — Vibrating tag ID

### Terminal 2: Relative pose estimator + logger
```bash
cd ~/harmonic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tag_hover_two_tags sim_lockon_backbone.launch.py
```

**Result:** Relative pose publishes to `/relative_vibration_pose` + CSV logs to `~/.ros/tag_hover_two_tags/`

---

## Full Flight Stack (5 terminals — with SITL + MAVROS)

### Terminal 1: ArduPilot SITL
```bash
cd ~/harmonic_ws/src/ardupilot
source ../../drone-venv/bin/activate
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=127.0.0.1:14550 --out=127.0.0.1:14555
```

### Terminal 2: Vision stack + Oscillator
```bash
cd ~/harmonic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tag_hover_two_tags sim_vision_stack.launch.py oscillation_freq:=1.0 oscillation_amp:=0.08
```

### Terminal 3: MAVROS
```bash
cd ~/harmonic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mavros apm.launch fcu_url:=udp://:14555@127.0.0.1:14550
```

### Terminal 4: Relative pose estimator + logger
```bash
cd ~/harmonic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tag_hover_two_tags sim_lockon_backbone.launch.py
```

### Terminal 5: Controller (yaw search/lock)
```bash
cd ~/harmonic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run tag_hover_two_tags hover_yaw_search --ros-args \
  -p mode:=SEARCH \
  -p camera_frame:=iris_with_rgb_camera/gimbal/pitch_link/camera \
  -p tag_frame:=tag36h11:1 \
  -p lock_k_yaw:=1.0 \
  -p search_yaw:=0.25
```

---

## Debug Split (7-8 terminals — Individual component inspection)

For detailed debugging, split the vision stack:

### Terminal 1: SITL
```bash
cd ~/harmonic_ws/src/ardupilot
source ../../drone-venv/bin/activate
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=127.0.0.1:14550 --out=127.0.0.1:14555
```

### Terminal 2: Gazebo
```bash
cd ~/harmonic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GZ_SIM_RESOURCE_PATH=~/harmonic_ws/src/tag_hover_two_tags/models:$GZ_SIM_RESOURCE_PATH
gz sim -r src/tag_hover_two_tags/worlds/apriltag_two_tags.sdf
```

### Terminal 3: Camera bridge
```bash
cd ~/harmonic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/apriltag_two_tags/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /world/apriltag_two_tags/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  --ros-args \
  -r /world/apriltag_two_tags/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/image:=/camera/image_raw \
  -r /world/apriltag_two_tags/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/camera_info:=/camera/camera_info
```

### Terminal 4: MAVROS
```bash
cd ~/harmonic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mavros apm.launch fcu_url:=udp://:14555@127.0.0.1:14550
```

### Terminal 5: AprilTag detector
```bash
cd ~/harmonic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:=/camera/image_raw \
  -r camera_info:=/camera/camera_info \
  --params-file ~/harmonic_ws/src/tag_hover_two_tags/config/apriltag_params.yaml
```

### Terminal 6: Tag overlay (RViz2 image box)
```bash
cd ~/harmonic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run tag_hover_two_tags tag_overlay --ros-args \
  -p image_topic:=/camera/image_raw \
  -p detections_topic:=/detections \
  -p output_topic:=/image_with_tags
```

### Terminal 7: Tag oscillator
```bash
cd ~/harmonic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run tag_hover_two_tags tag_oscillator --ros-args -p frequency:=1.0 -p amplitude:=0.08
```

### Terminal 8: Pose selectors (ref + vib)
```bash
cd ~/harmonic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# Reference tag selector
ros2 run tag_hover_two_tags tag_pose_selector --ros-args \
  -p tag_id:=0 \
  -p detections_topic:=/detections \
  -p output_topic:=/apriltag_ref/pose \
  -p camera_frame:=iris_with_rgb_camera/gimbal/pitch_link/camera &

# Vibrating tag selector
ros2 run tag_hover_two_tags tag_pose_selector --ros-args \
  -p tag_id:=1 \
  -p detections_topic:=/detections \
  -p output_topic:=/apriltag_vib/pose \
  -p camera_frame:=iris_with_rgb_camera/gimbal/pitch_link/camera
```

### Terminal 9: Relative pose estimator + logger
```bash
cd ~/harmonic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tag_hover_two_tags sim_lockon_backbone.launch.py
```

### Terminal 10: Controller (optional, stable baseline)
```bash
cd ~/harmonic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
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

---

## Tag Configuration

The simulation world contains:
- **Reference tag (ID 0)**: Static tag on the left at `y=-0.7m`
- **Vibrating tag (ID 1)**: Oscillating tag on the right at `y=+0.7m`
  - Oscillates side-to-side (Y-axis) with configurable frequency and amplitude
  - Tag size: **5 inches (0.127 m)**

## Oscillation Control

The oscillator runs automatically with the vision stack. To manually control:
```bash
ros2 topic pub /model/apriltag_vib_oscillator/joint/oscillator_joint/cmd_pos std_msgs/msg/Float64 "data: 0.05"
```

## Check topics
```bash
ros2 topic list
ros2 topic echo /apriltag_ref/pose --once
ros2 topic echo /apriltag_vib/pose --once
ros2 topic echo /relative_vibration_pose --once
ros2 topic echo /apriltag_vib_oscillator/cmd_pos  # Monitor oscillator commands
```

## Log output

CSV logs default to:
- `~/.ros/tag_hover_two_tags/relative_vibration_*.csv`

## Troubleshooting

```bash
# Is Gazebo camera publishing?
ros2 topic hz /camera/image_raw

# Is AprilTag detector running?
ros2 node list | grep apriltag

# Are tag poses publishing?
ros2 topic echo /apriltag_ref/pose --once
ros2 topic echo /apriltag_vib/pose --once

# Is the oscillator running?
ros2 node info /tag_oscillator
ros2 topic echo /apriltag_vib_oscillator/cmd_pos

# Check Gazebo joint state (if needed)
ros2 topic echo /world/apriltag_two_tags/model/apriltag_vib_oscillator/joint_state
```
