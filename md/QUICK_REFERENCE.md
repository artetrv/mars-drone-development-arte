# tag_hover_sim — Quick Reference

All commands use the correct working configuration from `docs/LOCKON_NOTES.md`.

## Environment (source in every new terminal)

```bash
cd ~/your_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source setup_harmonic_env.sh   # sets up Gazebo env vars
```

---

## Standard Bringup (7 terminals)

### Terminal 1 — ArduPilot SITL
```bash
cd ~/your_ws/src/ardupilot
source ../../drone-venv/bin/activate
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console \
  --out=127.0.0.1:14550 --out=127.0.0.1:14555
```
MAVProxy console opens here. Use it to arm/takeoff (see below).

### Terminal 2 — Gazebo
```bash
cd ~/your_ws
source setup_harmonic_env.sh
export GZ_SIM_RESOURCE_PATH=$(pwd)/src/tag_hover_sim/models:$(pwd)/src/ardupilot_gazebo/models
gz sim -r src/tag_hover_sim/worlds/apriltag_test.sdf
```

### Terminal 3 — Camera bridge
```bash
cd ~/your_ws && source setup_harmonic_env.sh
ros2 run ros_gz_bridge parameter_bridge \
  /world/apriltag_test/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /world/apriltag_test/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  --ros-args \
  -r /world/apriltag_test/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/image:=/camera/image_raw \
  -r /world/apriltag_test/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/camera_info:=/camera/camera_info
```
Verify: `ros2 topic hz /camera/image_raw` → ~6-7 Hz

### Terminal 4 — MAVROS
```bash
cd ~/your_ws && source setup_harmonic_env.sh
ros2 launch mavros apm.launch fcu_url:=udp://:14555@127.0.0.1:14550
```

### Terminal 5 — AprilTag detector
```bash
cd ~/your_ws && source setup_harmonic_env.sh
ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:=/camera/image_raw \
  -r camera_info:=/camera/camera_info \
  --params-file ~/your_ws/src/tag_hover_sim/config/apriltag_params.yaml
```

### Terminal 6 — PnP TF broadcaster
```bash
cd ~/your_ws && source setup_harmonic_env.sh
ros2 run tag_hover_sim apriltag_pnp_broadcaster --ros-args \
  -p camera_frame:=iris_with_rgb_camera/gimbal/pitch_link/camera \
  -p detections_topic:=/detections
```

### Terminal 7 — Controller (stable baseline v1)
```bash
cd ~/your_ws && source setup_harmonic_env.sh
ros2 run tag_hover_sim hover_yaw_search_v1 \
  --ros-args \
  -p body_frame:=base_link \
  -p camera_frame:=iris_with_rgb_camera/gimbal/pitch_link/camera \
  -p mavros_prefix:=/mavros \
  -p mode:=SEARCH \
  -p rate_hz:=20.0 \
  -p search_yaw:=0.25 \
  -p lock_k_yaw:=0.1 \
  -p max_yaw_rate:=0.6 \
  -p target_distance:=1.0 \
  -p mavros_wait_timeout:=10.0
```

---

## Arm and fly (MAVProxy console — Terminal 1)

```
mode GUIDED
arm throttle
takeoff 5
```

Drone spins in SEARCH → auto-locks when tag enters view.

---

## Quick checks

```bash
# Camera publishing?
ros2 topic hz /camera/image_raw

# Tag detected?
ros2 topic echo /detections --no-arr

# TF camera → tag?
ros2 run tf2_ros tf2_echo iris_with_rgb_camera/gimbal/pitch_link/camera tag36h11:0

# MAVROS connected?
ros2 topic echo /mavros/state --once

# Controller output?
ros2 topic echo /hover_yaw_cmd
```

---

## Runtime tuning

```bash
# Switch to LOCK manually
ros2 param set /hover_yaw_search mode LOCK

# Tune yaw gain at runtime
ros2 param set /hover_yaw_search lock_k_yaw 0.08
```

---

## 3-phase hardware controller (sensor_lock)

Replace Terminal 7 with:
```bash
ros2 run tag_hover_sim hover_yaw_search_sensor_lock \
  --ros-args \
  -p body_frame:=base_link \
  -p camera_frame:=iris_with_rgb_camera/gimbal/pitch_link/camera \
  -p mavros_prefix:=/mavros \
  -p mode:=SEARCH \
  -p rate_hz:=20.0 \
  -p target_distance:=1.0
```
See `docs/LOCKON_NOTES.md` for state machine details and equilibrium parameters.

---

## Shutdown

1. `disarm` in MAVProxy console
2. Ctrl+C all ROS terminals
3. Close Gazebo
4. Exit SITL (`exit` in MAVProxy)
