# Two-Tag Measurement Architecture Notes

---

## 1. High-level idea

**What this does (one sentence)**

Instead of controlling the UAV from AprilTag feedback, we use dual AprilTag pose estimation as a sensing system to measure relative motion (vibration) of a target object with respect to a stationary world reference.

**Why this approach is effective**

- No closed-loop flight control required (significant reduction in risk)
- Works even if the UAV is imperfectly stable
- Lets you quantify vibration frequency, amplitude, and drift
- Produces hardware-validated data that simulation can't fake
- Directly relevant to inspection and structural monitoring

**Key principle**

You are not comparing tag poses in the camera frame. You are computing:

$$T_{vib}^{world} \ominus T_{ref}^{world}$$

Which removes: UAV drift, small attitude changes, camera vibration, optical flow / IMU imperfections.

---

## 2. System architecture

**Physical setup**

- **Tag A (Reference Tag):** Large tag (e.g. AprilTag 36h11, ~15–20 cm). Mounted on a stationary wall/structure. Defines the local world frame.
- **Tag B (Vibration Tag):** Smaller tag on the vibrating object. Its motion is what we measure.
- **UAV:** Holds position roughly (manual / Loiter / Guided). Does not need precision hover.

**Software components (ROS)**

```
Camera Driver
   |
   +--> apriltag_node_ref   (detects Tag A only)
   |
   +--> apriltag_node_vib   (detects Tag B only)
            |
            v
     Pose Fusion / Differencing Node
            |
            v
     Vibration Estimator + Logger
```

---

## 3. AprilTag detection: why two nodes is correct

Running two AprilTag nodes is the cleanest way to handle this.

**Why not one node?**

- Different tag families / sizes / detection thresholds
- Independent tuning (critical on hardware)
- Cleaner topic separation
- Easier debugging and logging

**Node responsibilities**

- `apriltag_ref_node`: Detects only the stationary tag. Publishes pose of reference tag in camera frame. This pose defines a dynamic world reference tied to the real environment.
- `apriltag_vib_node`: Detects only the vibrating tag. Publishes pose of vibrating tag in camera frame.

---

## 4. Coordinate frames

If this is wrong, everything is wrong.

**Frames involved**
- `camera_frame`
- `ref_tag_frame`
- `vib_tag_frame`
- `virtual_world_frame` (defined by ref tag)

**What each node gives you**

From AprilTag detection:

$$T_{ref}^{cam}, \quad T_{vib}^{cam}$$

Each is a full SE(3) transform: position (x, y, z) + orientation (quaternion / rotation matrix).

---

## 5. Pose differencing math

**Step 1: Invert the reference pose**

$$T_{cam}^{ref} = (T_{ref}^{cam})^{-1}$$

This gives the camera pose expressed in the reference frame.

**Step 2: Transform vibrating tag into reference frame**

$$T_{vib}^{ref} = T_{cam}^{ref} \cdot T_{vib}^{cam}$$

Now:
- UAV motion is canceled
- Camera motion is canceled
- What remains is true relative motion

**Step 3: Extract vibration signals**

From $T_{vib}^{ref}$:

- Translation: $x(t), y(t), z(t)$
- Rotation: $roll(t), pitch(t), yaw(t)$

These time series are the raw vibration signals.

---

## 6. What "controller" is actually needed

**Short answer: none for flight**

This is a measurement system, not a flight controller.

**Components needed:**

1. **Transform Controller (kinematic)**
   - Subscribe to both tag pose topics
   - Time-synchronize messages
   - Compute relative transform
   - Publish relative pose (and optionally relative twist)
   - Pure math, no dynamics

2. **Vibration Estimator**
   - Maintain time buffer
   - Compute: displacement amplitude, dominant frequency (FFT), RMS vibration
   - Optional: band-pass filtering, windowed FFT

3. **Logger / Dataset Generator**
   - Save: raw poses, relative pose, timestamps
   - Format: CSV / ROS bag
   - Output becomes: result plots, future controller training data

---

## 7. Future extensions (not blocking current work)

This architecture sets up:

- Vision-based disturbance observers
- Feedforward vibration compensation
- Visual servoing with disturbance rejection
- Model-based vibration tracking

None of that blocks current progress.

---

## 8. Failure modes and mitigations

| Issue | Mitigation |
|---|---|
| Ref tag briefly lost | Hold last valid transform |
| Vib tag occluded | Skip frame |
| UAV yaw drift | Automatically canceled |
| Camera vibration | Automatically canceled |
| Lighting changes | Independent detector tuning |

This is a robust measurement pipeline, not a fragile controller.

---

## 9. System framing

This system is a vision-based relative pose sensing framework for vibration measurement in UAV inspection scenarios. It is designed to decouple perception from flight control, making it robust to imperfect hover.

It supports:

- Sensor fusion discussion
- Estimation vs control separation
- Simulation → hardware realism validation

---

## 10. Reference implementation prompt

The following prompt can be used to generate or verify the core relative pose node:

```
I am working on a ROS-based UAV perception system using AprilTags.

I want to build a Python ROS node that subscribes to two AprilTag pose topics:
  /apriltag_ref/pose → pose of a stationary reference tag detected in the camera frame
  /apriltag_vib/pose → pose of a vibrating target tag detected in the camera frame

Each topic publishes geometry_msgs/PoseStamped.

The node should:
1. Time-synchronize the two pose messages.
2. Convert both poses into SE(3) homogeneous transformation matrices.
3. Invert the reference tag transform to obtain the camera pose in the reference frame.
4. Compute the relative transform of the vibrating tag with respect to the reference tag:
   T_vib_ref = inv(T_ref_cam) * T_vib_cam
5. Publish the relative pose as a new PoseStamped topic called /relative_vibration_pose.
6. Log timestamped relative translation (x,y,z) and rotation (roll,pitch,yaw) to CSV.

Assume ROS 2 Python (rclpy), numpy, and tf_transformations are available.
Structure the code with helper functions for transform math.
```

---

## 11. Summary

This is a strategic design choice:

- Produces real hardware data
- Avoids unstable flight control debugging
- Publishes meaningful, analyzable results
- Lays groundwork for future control work
