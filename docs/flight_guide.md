# Flight Guide (harmonic_ws)

Date: 2025-12-20  
Stack: ArduPilot SITL + MAVROS + Gazebo Harmonic

## Pre-flight checklist (sim)
- `GZ_SIM_RESOURCE_PATH` includes local models: `$HOME/harmonic_ws/src/ardupilot_gazebo/models:$HOME/harmonic_ws/src/tag_hover_sim/models` (and install equivalents).
- ArduPilot SITL running (e.g., `--out 127.0.0.1:14550` and `--out 127.0.0.1:14555`).
- MAVROS launched (single instance) with matching `fcu_url`.
- Gazebo world loaded (e.g., `src/tag_hover_sim/worlds/apriltag_test.sdf`).
- Bridge + AprilTag nodes running (if testing lockon).

## Launch sequence (single UAV, lockon sim)
1) Start SITL:
```bash
cd ~/ardupilot && source venv/bin/activate
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map --out=127.0.0.1:14550 --out=127.0.0.1:14555
```
2) Start Gazebo:
```bash
cd ~/harmonic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/harmonic_ws/src/ardupilot_gazebo/models:$HOME/harmonic_ws/src/tag_hover_sim/models
gz sim -r src/tag_hover_sim/worlds/apriltag_test.sdf
```
3) Start MAVROS (one instance):
```bash
ros2 launch mavros apm.launch fcu_url:=udp://:14555@127.0.0.1:14550
```
4) Start controller (lockon):
```bash
ros2 run tag_hover_sim hover_yaw_search \
  --ros-args \
  -p mavros_prefix:=/mavros \
  -p mode:=SEARCH \
  -p rate_hz:=20.0 \
  -p search_yaw:=0.25 \
  -p lock_k_yaw:=0.0025 \
  -p max_yaw_rate:=0.6 \
  -p mavros_wait_timeout:=10.0
```
5) Start camera bridge + apriltag_ros + TF/PnP broadcaster (see `LOCKON_NOTES.md` for exact commands). If `ros2 run` fails for the broadcaster, call the installed binary directly (same params):
```bash
~/harmonic_ws/install/tag_hover_sim/lib/tag_hover_sim/apriltag_tf_broadcaster --ros-args \
	-p camera_frame:=iris_with_rgb_camera/gimbal/pitch_link/camera \
	-p detections_topic:=/detections
```

## Verify connection
```bash
ros2 topic echo /mavros/state --once
```
Look for `connected: true`. Set mode (GUIDED/LOITER), arm, take off (via MAVProxy or a service call).

Service call examples:
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 5.0}"
```

## 3-Phase Hybrid Controller (Hardware Ready)

### State Machine (SENSOR_LOCK)
```
SEARCH → ALIGN → HOVER_BOX → SENSOR_HOVER
(any) ←──────────────────────────────────
      tag lost (fallback)
```

### Launch
```bash
ros2 run tag_hover_sim hover_yaw_search_sensor_lock
```

### Expected Behavior
```
[STATE] SEARCH → ALIGN (tag detected at 2.34m)
[EQUILIBRIUM] Timer started
[EQUILIBRIUM] In box: 1.23s / 2.00s
[STATE] ALIGN → HOVER_BOX (equilibrium reached)
[EQUILIBRIUM] Dwell: 1.50s / 2.00s
[STATE] HOVER_BOX → SENSOR_HOVER (silent handoff)
SENSOR_HOVER (silent) | tag at 2.00m  ← No velocity commands published
```

### Key Parameters
| Parameter | Default | Purpose |
|-----------|---------|---------|
| lock_k_yaw | 0.1 | P gain for yaw |
| lock_k_distance | 0.2 | P gain for forward/back |
| lock_k_lateral | 0.1 | P gain for left/right |
| target_distance | 2.0 m | Desired standoff |
| lateral_box_m | 0.25 m | ±tolerance (left/right) |
| distance_box_m | 0.30 m | ±tolerance (forward/back) |
| yaw_box_rad | 0.08 rad | ±tolerance (yaw ~4.6°) |
| equilibrium_time_s | 2.0 s | Dwell timer per transition |

Tune at runtime: `ros2 param set /hover_yaw_search lock_k_yaw 0.08`

## Common warnings
- `AUTOPILOT_VERSION`/time jump warnings: typically benign; wait a few seconds after startup.
- Multiple MAVROS instances cause crashes; ensure only one per ROS domain (or use unique node names/namespaces).

## Troubleshooting quick checks
- No camera/detections: verify bridge topics and `apriltag_params.yaml` path; check `/camera/image_raw` hz.
- No TF tag frame: verify apriltag_ros publishes `/detections`, PnP broadcaster running.
- No yaw motion: ensure `/mavros/state` is connected and mode is GUIDED/LOITER; check `/hover_yaw_cmd`.

## Safe shutdown
- Disarm via MAVProxy (`disarm`) or MAVROS service if needed.
- Ctrl+C nodes; stop SITL last.
