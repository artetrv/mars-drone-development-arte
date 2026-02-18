# PROGRESS LOG

## 2026-02-16
**Summary:**
- Added `tag_oscillator.launch.py` and fixed `hover_yaw_search` entry-point imports in two-tag package.
- Moved joint controller plugin inside the vibrating tag model and removed world-level joint state publisher.
- Resized AprilTags to 5 inches (0.127 m) across two-tag models/world and updated detector `tag_size`.
- Set controller default `target_distance` to 1.0 m in the stable `hover_yaw_search_v1`/`v2` variants.
- Confirmed overlay command and diagnostics for image/detections/overlay topics.

**Notes:**
- Recommended controller for sim: `tag_hover_sim hover_yaw_search_v1`.
- Ensure `GZ_SIM_RESOURCE_PATH` includes `src/tag_hover_two_tags/models` for tag textures.

**Next Todos:**
- [ ] Run full end-to-end two-tag sim (vision + oscillator + controller) after size change.
- [ ] Verify CSV logging output and relative pose signal quality.

## 2026-01-08 (Continued)
**Summary:**
- Fixed environment setup: added `COLCON_IGNORE` to `drone-venv` to prevent colcon scanning errors.
- Installed wxPython in drone-venv to fix MAVProxy console/map GUI modules.
- Successfully launched Terminals 1-3 (SITL, Gazebo, Camera Bridge).
- **Key finding:** Simple camera bridge (`/camera/image_raw`) doesn't work; must use full scoped path with remapping (see below).
- Camera now publishing at ~6-7 Hz on `/camera/image_raw` and `/camera/camera_info`.

**MAVROS + Controller bringup (split works reliably):**
- Launched MAVROS standalone via `apm.launch`:
  ```bash
  ros2 launch mavros apm.launch fcu_url:=udp://:14555@127.0.0.1:14550
  ```
- Launched controller pointing to `/mavros`:
  ```bash
  ros2 run tag_hover_sim hover_yaw_search --ros-args -p mavros_prefix:=/mavros -p mode:=SEARCH
  ```
- Verified: controller logs show "MAVROS connected and ready"; SEARCH mode yaw spin observed.
- Verified log lines (controller):
  - `[INFO] [...] [hover_yaw_search]: Waiting for MAVROS initialization... (X.Xs/10.0s)`
  - `[INFO] [...] [hover_yaw_search]: MAVROS connected and ready`
  - `[WARN] [...] [hover_yaw_search]: FCU mode is LAND, expected GUIDED/GUIDED_NOGPS/LOITER for testing.`
- Set mode/arm/takeoff via services or MAVProxy:
  ```bash
  ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
  ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
  # Optional: CommandTOL or MAVProxy 'takeoff 5'
  ```

**Note on combined launch:**
- `sim_lockon_backbone.launch.py` can still launch MAVROS + controller together, but ensure unique names/prefixes to avoid duplicate-topic errors.

**Working Commands:**
- **Terminal 3 (Camera Bridge):** Use the scoped path with remapping (not simple bridge):
  ```bash
  ros2 run ros_gz_bridge parameter_bridge \
    /world/apriltag_test/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
    /world/apriltag_test/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
    --ros-args \
    -r /world/apriltag_test/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/image:=/camera/image_raw \
    -r /world/apriltag_test/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/camera_info:=/camera/camera_info
  ```

**Next Todos:**
- [ ] Launch Terminal 4 (MAVROS + Controller) and verify `/mavros/state` shows `connected: true`.
- [ ] Launch Terminal 5 (AprilTag Detector).
- [ ] Launch Terminal 6 (AprilTag TF Broadcaster).
- [ ] Launch Terminal 7 (AprilTag PnP Broadcaster).

## 2026-01-28 (3-Phase Hybrid Controller)
**Status:** ✅ IMPLEMENTATION COMPLETE

Implemented `hover_yaw_search_sensor_lock.py` (431 lines) — 3-phase supervisory state machine for hardware deployment.

**Phases:**
- **Phase 1 (ALIGN):** Continuous P-control (yaw, distance, lateral, vertical). Entry: tag found. Exit: tag in box for 2.0s.
- **Phase 2 (HOVER_BOX):** Event-based bounding box (zero velocity inside, corrections outside). Box: ±0.25m lateral, ±0.30m distance, ±0.08rad yaw.
- **Phase 3 (SENSOR_HOVER):** Silent handoff (no velocity commands). FCU holds via optical flow + rangefinder. Duration: indefinite until tag lost.

**Key Features:** ControlPhase enum (4 states), equilibrium dwell timers, tag loss fallback, comprehensive logging ([STATE], [EQUILIBRIUM] tags).

**Launch:** `ros2 run tag_hover_sim hover_yaw_search_sensor_lock`

**Ready for:** Raspberry Pi 5 + Pixhawk optical flow deployment.
- [ ] Verify full system integration and test arm/takeoff/SEARCH→LOCK modes.
 - [ ] Tune `lock_k_yaw` and `search_yaw` as needed.

---

## 2026-01-12
**Summary:**
- AprilTag model updated to SDF 1.9 with PBR albedo_map; texture now renders reliably.
- Tag moved to the right of the drone (`pose 0 2 1.5 0 1.5708 1.5708`) in apriltag_test world.
- Detector publishes frame `iris_with_rgb_camera/gimbal/pitch_link/camera`; matched across detector, TF broadcaster, and controller.
- Verified `/detections` stream; TF echo now works when broadcaster is launched with correct camera_frame.
- `apriltag_tf_broadcaster` / `apriltag_pnp_broadcaster` executables present under install; can be run directly if `ros2 run` path not working.

**Commands (working set):**
- TF broadcaster direct:
  ```bash
  ~/harmonic_ws/install/tag_hover_sim/lib/tag_hover_sim/apriltag_tf_broadcaster --ros-args \
    -p camera_frame:=iris_with_rgb_camera/gimbal/pitch_link/camera \
    -p detections_topic:=/detections
  ```
- Controller with matching frames:
  ```bash
  ros2 run tag_hover_sim hover_yaw_search --ros-args \
    -p mavros_prefix:=/mavros \
    -p mode:=SEARCH \
    -p camera_frame:=iris_with_rgb_camera/gimbal/pitch_link/camera \
    -p tag_frame:=tag36h11:0
  ```

**Outstanding:**
- Confirm end-to-end SEARCH→LOCK behavior in flight once TF is flowing (controller should auto-lock when TF present).
- Optional: clean up unused Ogre material scripts (PBR now in use).

---

## 2026-01-14 (Final)
**Summary:**
- SEARCH→LOCK auto-switch implemented and verified working
- Drone spins in SEARCH mode, automatically locks when AprilTag detected
- Controller uses P-gain yaw control: when tag centered, yaw_cmd → 0 (correct behavior)
- PnP broadcaster confirmed as working TF source (detector alone doesn't provide pose)
- Full end-to-end pipeline tested: SITL → Gazebo → camera → detector → PnP → controller → MAVROS

**Key Fix:** Controller defaults to `camera_frame='camera'` but detector publishes `iris_with_rgb_camera/gimbal/pitch_link/camera`. Must pass `-p camera_frame:=iris_with_rgb_camera/gimbal/pitch_link/camera` to match.

**Verified Log Output (when tag in view):**
```
[INFO] [...] [hover_yaw_search]: TAG FOUND! Auto-locking. yaw_error=-0.793 rad, yaw_cmd=-0.002 rad/s, tag_pos_cam=[-1.82, 0.31, 1.79]
```

**Expected Behavior:**
1. Drone arms, takes off (GUIDED mode)
2. Controller starts in SEARCH: drone yaws at 0.25 rad/s
3. AprilTag enters camera view → detector publishes `/detections` (with corners)
4. PnP broadcaster reads detections, computes 3D pose, broadcasts TF
5. Controller detects TF, auto-switches to yaw-lock behavior
6. Drone yaws to center tag (yaw_error → 0), then holds position
7. When tag leaves view, detections becomes empty `[]`, controller reverts to SEARCH

**Ready for:**
- Real hardware deployment (same pipeline applies)
- Tuning gains (adjust `-p lock_k_yaw` for faster/slower response)
- Adding pitch/roll centering (currently yaw-only)
- Altitude hold integration

---

## 2026-01-07
**Summary:**
- Reviewed project status and documentation baseline (ALL_WORK_SUMMARY.md, LOCKON_NOTES.md).
- Set up PROGRESS_LOG.md as working session tracker with date stamps and next todos.
- Created STARTUP_CHECKLIST.md with environment sourcing reference.
- Established THESIS_SYNC_AGENT.md purpose (thesis-relevant insights only).

**Issues Resolved:**
- Fixed `ardupilot_sitl` build failure by adding `COLCON_IGNORE` to drone-venv.
- Clarified sourcing strategy: use `setup_harmonic_env.sh` for ROS 2 + Gazebo, use `drone-venv` for ArduPilot SITL.
- Harmless warning: `ardupilot_sitl` package missing `local_setup.bash` (can skip this package in future builds).
