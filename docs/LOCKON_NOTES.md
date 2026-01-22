# Simulation + Lockon Integration Notes

## Worlds and Models
   - `src/tag_hover_sim/worlds/apriltag_test.sdf` (SDF 1.9) includes `model://iris_with_ardupilot` and an AprilTag board.
- AprilTag model now uses PBR (albedo_map) instead of Ogre script:
  - File: `src/tag_hover_sim/models/apriltag_36h11_0/model.sdf` (SDF 1.9, `<pbr><metal><albedo_map>materials/textures/tag36h11_0.png</albedo_map>`).
  - Image: `src/tag_hover_sim/models/apriltag_36h11_0/materials/textures/tag36h11_0.png` (36h11, id=0).
  - Pose in world: moved to the right of the drone and rotated to face it: `<pose>0 2 1.5 0 1.5708 1.5708</pose>`.
  - Ensure `GZ_SIM_RESOURCE_PATH` includes `src/tag_hover_sim/models` so the texture resolves.
- `src/tag_hover_sim/models/iris_with_ardupilot` includes `iris_with_standoffs` and references links/joints as `iris_with_ardupilot::iris_with_standoffs::rotor_*` plus the IMU.
- `src/tag_hover_sim/models/iris_with_rgb_camera` adds a fixed gimbal by including `gimbal_small_3d_fixed`, attaching it via `gimbal_mount` to the base, and orienting it forward.

## Gimbal Fix
- Cloned `gimbal_small_3d` to `gimbal_small_3d_fixed`, updated mesh URIs, and locked yaw/roll/pitch joint limits to 0 so it does not drift.
- Included `gimbal_small_3d_fixed` in `iris_with_rgb_camera`, fixed joint to the mount, and adjusted pose to face forward.

## Resource Paths
- Ensure `GZ_SIM_RESOURCE_PATH` includes `src/tag_hover_sim/models` (which contains both `iris_with_rgb_camera`, `iris_with_standoffs`, and `apriltag_36h11_0`), and also include `src/ardupilot_gazebo/models` if needed for other models.

## AprilTag + Camera
- Camera frame published by detector: `iris_with_rgb_camera/gimbal/pitch_link/camera` (use this everywhere).
- Bridge the Gazebo camera to ROS 2 using `ros_gz_bridge` (parameter_bridge). **Use the scoped path with remapping** (simple bridge doesn't work):
  ```bash
  ros2 run ros_gz_bridge parameter_bridge \
    /world/apriltag_test/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
    /world/apriltag_test/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
    --ros-args \
    -r /world/apriltag_test/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/image:=/camera/image_raw \
    -r /world/apriltag_test/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/camera_info:=/camera/camera_info
  ```
  Verify with: `ros2 topic hz /camera/image_raw` (should show ~6-7 Hz).
- Run `apriltag_ros` with `src/tag_hover_sim/config/apriltag_params.yaml`, remapping `image_rect`/`camera_info` to the bridged topics:
  ```bash
  ros2 run apriltag_ros apriltag_node --ros-args \
    -r image_rect:=/camera/image_raw \
    -r camera_info:=/camera/camera_info \
    --params-file ~/harmonic_ws/src/tag_hover_sim/config/apriltag_params.yaml
  ```

## Controller (`hover_yaw_search` / `hover_yaw_search_v1`)
- **Architecture**: 4-DOF relative pose regulation (similar to AprilTag precision landing, but frozen mid-air at target distance)
- Subscribes to `/mavros/state` and TF `camera -> tag`.
- Publishes body-frame velocities to `/mavros/setpoint_velocity/cmd_vel_unstamped` (and `/hover_yaw_cmd` for debug).
- **Two versions available:**
  - **`hover_yaw_search_v1`** (RECOMMENDED): Stable, tested version. Uses camera-frame errors directly without body-frame transformation. Has small position offset but reliable convergence.
  - **`hover_yaw_search`** (EXPERIMENTAL): Includes body-frame transformation to eliminate offset, but currently under development.
- **Control loops (continuous, no gating):**
  - **Yaw**: aligns camera optical axis with tag normal using `yaw_error = atan2(x, z)`
  - **Forward/backward**: regulates distance to `target_distance` using `distance_error = z - target_distance`
  - **Lateral (left/right)**: centers tag horizontally using `lateral_error = -x`
  - **Vertical (up/down)**: centers tag vertically using `vertical_error = -y`
- **Modes:**
  - SEARCH: constant `search_yaw` (default 0.25 rad/s) until tag found, then auto-switches to 4-DOF control
  - LOCK: continuous 4-DOF relative pose regulation; falls back to `search_yaw` if TF is missing
- **Key parameters:**
  - `lock_k_yaw`: P gain for yaw (default 0.1)
  - `lock_k_distance`: P gain for forward/backward (default 0.2 m/s per m)
  - `lock_k_lateral`: P gain for lateral (default 0.1 m/s per m)
  - `lock_k_vertical`: P gain for vertical (default 0.1 m/s per m)
  - `target_distance`: desired standoff distance in meters (default 2.0)
  - `body_frame`: drone body frame name (default `base_link`)
  - **`camera_frame`**: CRITICAL - must match broadcaster's camera frame (e.g., `iris_with_rgb_camera/gimbal/pitch_link/camera`)
- Requires MAVROS connected and FCU mode GUIDED/LOITER/GUIDED_NOGPS.

## MAVROS Launch
Two patterns are supported:

1) Split (recommended while debugging)
   - Start MAVROS via its own launch:
     ```bash
     ros2 launch mavros apm.launch fcu_url:=udp://:14555@127.0.0.1:14550
     ```
   - Start controller separately, pointing at MAVROS topics:
     ```bash
     ros2 run tag_hover_sim hover_yaw_search --ros-args \
       -p mavros_prefix:=/mavros \
       -p mode:=SEARCH \
       -p rate_hz:=20.0 \
       -p search_yaw:=0.25 \
       -p lock_k_yaw:=0.0025 \
       -p max_yaw_rate:=0.6 \
       -p mavros_wait_timeout:=10.0
     ```

2) Combined (MAVROS + controller in one):
   - `src/tag_hover_sim/launch/sim_lockon_backbone.launch.py` starts both MAVROS (`mavros_node`) and `hover_yaw_search`.
   - Example:
     ```bash
     ros2 launch tag_hover_sim sim_lockon_backbone.launch.py fcu_url:=udp://:14555@127.0.0.1:14550
     ```

Avoid running a second MAVROS in the same ROS domain. If you need to, give it a unique `__node` and `namespace`. Typical ArduPilot SITL ports: listen 14555, send 14550; use `fcu_url:=udp://:14555@127.0.0.1:14550`.

## Common Issues
- MAVROS crash “existing topic name … incompatible type”: typically caused by duplicate names/namespaces (or double-prefix remaps) leading to the same topic being created twice with different types. Fix: kill stale `mavros_node`, ensure a single instance, and avoid double-prefixing (__ns plus manual remaps).
- Time jump warnings: benign; set `use_sim_time:=true` if using `/clock`.- **Controller drift/oscillation (FIXED)**: Early versions used gated control (only commanded forward/lateral when yaw aligned). This caused bang-bang motion and drift. Current version uses continuous 4-DOF relative pose regulation - all control loops run simultaneously without gating for smooth convergence.
- **Camera frame mismatch**: If controller doesn't lock on despite valid detections/TF, verify `-p camera_frame:=` matches the broadcaster's camera frame. Check startup logs for `camera_frame=` value.
## MAVROS allocator crash (invalid allocator at subscription.c:261)
- Likely trigger: a prior error (`create_subscription() called for existing topic ... incompatible type`) due to duplicate topics; allocator error follows when internal state is corrupted.
- Fix steps:
  1) Ensure SITL sends MAVLink to MAVROS (`--out=127.0.0.1:14550`).
  2) Launch a single MAVROS with correct `fcu_url:=udp://:14555@127.0.0.1:14550`.
  3) Use either namespace OR a custom node/topic prefix, but not both in a way that doubles paths.
  4) Verify `/mavros/state` shows `connected: true` before arming.

## Launch Sequence (working)
1) Start SITL (set `--out` ports, e.g., 14550/14555):
   ```bash
   cd ~/harmonic_ws/src/ardupilot
   source ../../drone-venv/bin/activate
   ./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=127.0.0.1:14550 --out=127.0.0.1:14555
   ```

2) Launch Gazebo world (use setup_harmonic_env.sh for proper environment):
   ```bash
   cd ~/harmonic_ws
   source setup_harmonic_env.sh
   export GZ_SIM_RESOURCE_PATH=$(pwd)/src/tag_hover_sim/models
   gz sim -r src/tag_hover_sim/worlds/apriltag_test.sdf
   ```

3) Start the camera bridge (full scoped path with remapping):
   ```bash
   cd ~/harmonic_ws
   source setup_harmonic_env.sh
   ros2 run ros_gz_bridge parameter_bridge \
     /world/apriltag_test/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
     /world/apriltag_test/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
     --ros-args \
     -r /world/apriltag_test/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/image:=/camera/image_raw \
     -r /world/apriltag_test/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/camera_info:=/camera/camera_info
   ```

4) Start MAVROS:
   ```bash
   cd ~/harmonic_ws
   source setup_harmonic_env.sh
   ros2 launch mavros apm.launch fcu_url:=udp://:14555@127.0.0.1:14550
   ```

5) Start AprilTag detector (subscribes to camera images, publishes tag detections on `/detections`):
   ```bash
   cd ~/harmonic_ws
   source setup_harmonic_env.sh
   ros2 run apriltag_ros apriltag_node --ros-args \
     -r image_rect:=/camera/image_raw \
     -r camera_info:=/camera/camera_info \
     --params-file ~/harmonic_ws/src/tag_hover_sim/config/apriltag_params.yaml
   ```

6) Start AprilTag TF broadcaster (listens to `/detections`, broadcasts TF `camera_frame → tag36h11:0`):
   ```bash
   cd ~/harmonic_ws
   source setup_harmonic_env.sh
   ros2 run tag_hover_sim apriltag_tf_broadcaster --ros-args \
     -p camera_frame:=iris_with_rgb_camera/gimbal/pitch_link/camera \
     -p detections_topic:=/detections
   ```
   
   **Alternative: PnP broadcaster** (refines pose using Perspective-n-Point for better accuracy):
   ```bash
   cd ~/harmonic_ws
   source setup_harmonic_env.sh
   ros2 run tag_hover_sim apriltag_pnp_broadcaster --ros-args \
     -p camera_frame:=iris_with_rgb_camera/gimbal/pitch_link/camera \
     -p detections_topic:=/detections
   ```

7) Start hover/yaw controller (subscribes to `/mavros/state` and TF, publishes yaw rate):
   
   **Option A: v1 Controller (STABLE BASELINE - locked, do not edit):**
   ```bash
   cd ~/harmonic_ws
   source setup_harmonic_env.sh
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
     -p mavros_wait_timeout:=10.0
   ```
   
   **Option B: v2 Controller (DEVELOPMENT VERSION - all improvements go here):**
   ```bash
   cd ~/harmonic_ws
   source setup_harmonic_env.sh
   ros2 run tag_hover_sim hover_yaw_search \
     --ros-args \
     -p body_frame:=base_link \
     -p camera_frame:=iris_with_rgb_camera/gimbal/pitch_link/camera \
     -p mavros_prefix:=/mavros \
     -p mode:=SEARCH \
     -p rate_hz:=20.0 \
     -p search_yaw:=0.25 \
     -p lock_k_yaw:=0.1 \
     -p max_yaw_rate:=0.6 \
     -p mavros_wait_timeout:=10.0
   ```
   Status: Phase-1 camera-frame IBVS with yaw gating + 0.05 m lateral deadband; forward sign matches v1; vertical held at 0.0. Residual: slight right drift remains but reduced.
   Next steps (open): add vertical safety clamp/deadband; add ROS↔MAVROS axis sanity log; Phase-2 body-frame regulation later.

**Verify and fly:**
- Check `/mavros/state` shows `connected: true`
- Set mode to GUIDED: `ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"`
- Arm: `ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"`
- Takeoff (or use MAVProxy): `ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 5.0}"`
- Drone will spin (SEARCH); once tag enters view, controller locks automatically.

## Debugging AprilTag Detection & Broadcaster

Use these commands to verify each component is working:

```bash
# 1. Verify camera is publishing
ros2 topic hz /camera/image_raw

# 2. View camera image
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# 3. Check if AprilTag detections are published
ros2 topic echo /detections

# 4. Verify TF broadcaster is working (check if tag frame exists)
ros2 run tf2_tools view_frames

# 5. Echo the TF between camera and tag
ros2 run tf2_ros tf2_echo iris_with_rgb_camera/gimbal/pitch_link/camera tag36h11:0

# 6. Monitor controller debug output
ros2 topic echo /hover_yaw_cmd

# 7. Check if broadcaster node is running
ros2 node list | grep broadcaster
```

## Controller Not Locking On (Stuck in SEARCH)

If the controller keeps spinning and doesn't slow down when the AprilTag is visible:

**Diagnostics:**
```bash
# Check controller node is running and its parameters
ros2 node info /hover_yaw_search

# Verify TF is being published (should NOT timeout)
ros2 run tf2_ros tf2_echo iris_with_rgb_camera/gimbal/pitch_link/camera tag36h11:0

# Check controller logs for errors
ros2 topic echo /rosout | grep -i "hover_yaw"

# Verify broadcaster is publishing frequently
ros2 topic hz /tf
```

**Common causes:**
- **TF frame not found**: If `tf2_echo` times out or says frame doesn't exist, the broadcaster isn't publishing. Check broadcaster logs: `ros2 node info /apriltag_tf_broadcaster` or `ros2 node info /apriltag_pnp_broadcaster`
- **Mode stuck in SEARCH**: Verify controller is launched with `-p mode:=LOCK` instead of SEARCH, OR start in SEARCH and wait for tag to be detected (should auto-switch)
- **Broadcaster not running**: Check `ros2 node list | grep broadcaster` - ensure it's listed
- **Detections topic issues**: Verify `/detections` has data: `ros2 topic echo /detections` (should show detection objects when tag is visible)
