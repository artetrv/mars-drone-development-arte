# THESIS_SYNC_AGENT

## Purpose
This document stores detailed notes and insights directly relevant to thesis work. Unlike PROGRESS_LOG.md (which tracks session-by-session development), this file is reserved for:
- Experiment results and performance data
- Validated behaviors and findings (e.g., "Tag hover + yaw-lock works reliably at X distance")
- Technical insights that should go into the thesis narrative
- Key papers, references, or design decisions that shaped the implementation
- Lessons learned and design trade-offs

**Add an entry here only when you have a concrete result, test outcome, or insight worth documenting for your thesis.**

---

## 2026-01-14: AprilTag Yaw-Lock Proportional Control - Validated

### System Architecture (Verified in Simulation)

**Detection Pipeline:**
```python
# apriltag_pnp_broadcaster.py excerpt
# Computes 3D pose from corner detections + camera intrinsics
# Publishes TF: camera_frame -> tag36h11:0 at detector frequency (~6 Hz)

from apriltag_ros_py import ApriltagNode
from tf2_ros import TransformBroadcaster

# Subscribe to /detections (corners + homography)
# Use PnP to solve 3D pose from intrinsics
# Broadcast transform with full 6-DOF pose
```

**Controller Core (Yaw Lock with Auto-Switch):**
```python
# hover_yaw_search.py - SEARCH → LOCK auto-switching
def _on_timer(self):
    if self.mode.upper() == 'LOCK':
        # Try to lock on tag
        tf = self._tf_buffer.lookup_transform(self.camera_frame, self.tag_frame, Time())
        x, y, z = tf.transform.translation.x/y/z  # tag position in camera frame
        
        # Proportional yaw control (corrected sign)
        yaw_error = -math.atan2(x, z)  # Negative: tag-right → clockwise yaw
        yaw_cmd = self.lock_k_yaw * yaw_error  # P gain ~0.1 rad/(rad·rad)
        yaw_cmd = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_cmd))  # Saturate
        cmd.angular.z = yaw_cmd
        
    else:  # SEARCH mode
        # Auto-lock when TF valid, else keep searching
        try:
            tf = self._tf_buffer.lookup_transform(self.camera_frame, self.tag_frame, Time())
            if valid_pose(tf):
                yaw_error = -math.atan2(x, z)
                yaw_cmd = self.lock_k_yaw * yaw_error  # Auto-switch behavior
                # ... publish lock command
            else:
                cmd.angular.z = self.search_yaw  # Continue spinning
        except TransformException:
            cmd.angular.z = self.search_yaw  # No tag, search
```

### Key Findings

**Proportional Control Behavior:**
- **Sign Convention**: Camera frame (x=right, y=down, z=forward). Tag at position `(x, z)` produces error `atan2(x, z)`. Negating ensures: tag-right → clockwise yaw (intuitive).
- **Gain Sensitivity**: `lock_k_yaw = 0.1` provides stable centering with visible response. At gain = 0.0025, control commands were imperceptible (< 0.001 rad/s). Recommend range: 0.05–0.2 depending on aggressiveness.
- **Steady-State**: P-only controller centers tag but may exhibit small oscillations (~±0.05 rad offset) due to zero integral/derivative terms. Acceptable for video capture if tag remains within acceptable error band.
- **Response Latency**: Control loop 20 Hz, camera ~6–7 Hz → effective system delay ~100 ms. Does not cause oscillation with current gains.

---

## 2026-01-14: Mentor Meeting - Vibration Analysis Framework (Next Steps)

### Objective
Develop autonomous drone-based vibration measurement system using AprilTag visual tracking and multi-sensor fusion to isolate structural vibrations from platform dynamics.

### Phase 1: Autonomous Positioning & Data Collection

**Goal**: Drone recognizes AprilTag marker, locks yaw to center tag, and maintains ideal standoff distance/angle for video capture.

**Requirements:**
1. **Extended Locking**: Expand current yaw-lock to 3-DOF (pitch, roll, distance-based forward/backward)
   - Current: yaw-only centering via P control
   - Next: X–Y centering (pitch/roll) and Z distance feedback
   
2. **Distance/Angle Constraints**:
   - Maintain target standoff distance (e.g., 1–2 m) for optimal tag resolution
   - Ensure camera optical axis nearly perpendicular to test surface
   - Prevent extreme angles that degrade feature tracking

3. **Video Acquisition**:
   - Record stereo or mono H.264/VP9 at 30–60 Hz (synchronized with control loop telemetry)
   - Log timestamps of yaw_error, distance, pitch/roll for post-analysis correlation

### Phase 2: Vibration Decoupling (Dual-Tag Reference Frame)

**Problem Statement**: 
Drone's motor vibrations and control corrections introduce high-frequency noise in video. Simply analyzing video of test structure will conflate structural vibrations with platform dynamics.

**Solution: Reference Tag Method**

**Configuration:**
- **Tag A (Reference)**: Static frame fixed to drone or stable reference point in test environment
- **Tag B (Test Object)**: Affixed to structure under test
- Both tags: AprilTag 36h11 family, tracked simultaneously at camera frequency

**Video Analysis Protocol:**
```
Frame-by-frame image processing:

1. Detect centroids of Tag A (reference) and Tag B (test) in pixel coordinates
2. Compute Tag A drift: Δ_ref = centroid_A(t) - centroid_A(t-1)
3. Compute Tag B displacement: Δ_test = centroid_B(t) - centroid_B(t-1)
4. Isolate structural vibration: Δ_struct = Δ_test - Δ_ref
   (Removes common-mode drone dynamics)
5. Convert pixel offsets to 3D coordinates using calibrated camera intrinsics
6. Output: Time series of structural motion corrected for platform vibration
```

**Advantages:**
- No assumptions about drone motion model
- Automatically compensates for:
  - Motor vibration (high frequency, ~100–300 Hz)
  - Control loop corrections (low frequency, ~0.1–2 Hz)
  - Optical flow from wind/disturbances
- Reference tag captures drone's vibrational "signature"; difference isolates test object

### Phase 3: Sensor Fusion for Robust Measurement

**Problem**: Even with reference tag, residual noise persists from optical flow, rolling shutter, and focus tracking artifacts.

**Multi-Sensor Approach:**

**Available Onboard Sensors (Iris drone in ArduPilot/MAVROS):**
- **IMU (Accelerometer + Gyroscope)**: 6-DOF inertial measurement at ~200 Hz
  - Captures drone body vibration directly
  - Can model rigid-body dynamics and remove from video analysis

- **Optical Flow Sensor** (if equipped): Captures apparent motion on ground
  - Indicates platform drift independent of vision features
  - Useful for detecting environmental wind disturbance

- **VIO / Visual Odometry** (ROS-based if integrated):
  - Estimates drone pose and velocity from camera features
  - Provides global reference frame for video interpretation

**Fusion Strategy:**
```
Measurement Layer:
  - Video: High-resolution feature tracking (pixels → 3D via calibration)
  - IMU: Drone acceleration/angular rate
  - Optical flow: Apparent ground motion
  - VIO: Drone pose estimate

Filtering (Extended Kalman Filter or Graph-Based):
  1. State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw, αx, αy, αz]_drone
                   + [x, y, z, vx, vy, vz]_test_structure
  2. Observations:
     - Video: Δ_test - Δ_ref (relative position measurement)
     - IMU: acceleration (constrains drone state dynamics)
     - Optical flow: validates optical flow estimates
  3. Output: Estimated structural motion + uncertainty bounds
```

**Expected Benefits:**
- Reduce video noise by ~70–80% through sensor fusion
- Distinguish low-frequency structural modes from high-frequency noise
- Robust to temporary feature loss or occlusion

### Phase 4: Experimental Campaign

**Experiment 1: Baseline Validation (Simulation)**
- Test dual-tag video analysis on synthetic data (Gazebo + AprilTag model)
- Validate reference tag method removes drone yaw/pitch/roll artifacts
- Measure residual error with ideal camera + no real vibration

**Experiment 2: Real Hardware (Lab Environment)**
- Mount Iris + AprilTag on test rig
- Attach reference tag to drone frame
- Test object: Cantilever beam or suspension structure with known resonance
- Excite test object and record video + IMU + optical flow
- Compare video-only vs. dual-tag vs. sensor-fused measurements against ground truth (e.g., accelerometer on test object)

**Experiment 3: Environmental Effects**
- Repeat Exp. 2 with:
  - Varying standoff distances (0.5 m, 1.0 m, 2.0 m)
  - Different camera angles (0°, 15°, 30° from perpendicular)
  - Controlled disturbances (fan-induced wind, manual platform shaking)
- Characterize sensitivity of measurement accuracy to positioning/environment

### Phase 5: Deliverables & Milestones

**Short-term (Next 4 weeks):**
- [ ] Extend controller to 3-DOF lock (X, Y, Z feedback)
- [ ] Integrate dual-tag detection in existing AprilTag pipeline
- [ ] Implement reference-tag differencing in post-processing script (OpenCV)
- [ ] Validate on simulation data (no real vibration)

**Medium-term (Weeks 5–12):**
- [ ] Deploy on real Iris hardware
- [ ] Conduct baseline lab experiments with known test structures
- [ ] Integrate IMU/optical flow into fusion framework (ROS 2 node)
- [ ] Generate preliminary measurement accuracy report

**Long-term (Weeks 13+):**
- [ ] Field deployment on larger/more complex structures
- [ ] Publish methodology and validation results
- [ ] Compare against traditional (ground-based) vibration monitoring

### Technical Considerations

1. **Camera Calibration**: Precise intrinsics (focal length, principal point, distortion) are critical for metric accuracy. Recommend OpenCV checkerboard or ROS camera_calibration package.

2. **Lighting Conditions**: AprilTag detection degrades in low-light or high-glare conditions. May require controlled lighting or multiple exposure frames.

3. **Computational Load**: Dual-tag tracking + video encoding + sensor fusion at real-time rates. Offboard processing (laptop via WiFi) may be needed for feature-heavy analysis.

4. **Time Synchronization**: Correlate video frames, IMU samples, and control loop updates. Use ROS 2 time stamps and hardware sync if available.

5. **Structural Design Limitations**: Drone carrying capacity, payload dynamics, and control authority must be validated before field deployment.

---

## Active Roadmap (Tracking)

**Short-term (Weeks 1–4):**
- [ ] Extend controller to 3-DOF lock (X, Y, Z)
- [ ] Integrate dual-tag detection in AprilTag pipeline
- [ ] Implement reference-tag differencing post-processing script
- [ ] Validate dual-tag method on simulation (Gazebo)

**Medium-term (Weeks 5–12):**
- [ ] Deploy controller + dual-tag on real Iris hardware
- [ ] Conduct Experiment 1: Baseline validation (simulation)
- [ ] Conduct Experiment 2: Real hardware lab environment
- [ ] Conduct Experiment 3: Environmental sensitivity analysis
- [ ] Integrate IMU/optical flow into sensor fusion framework
- [ ] Generate preliminary measurement accuracy report

**Long-term (Weeks 13+):**
- [ ] Field deployment on larger structures
- [ ] Publish methodology and validation results

---

