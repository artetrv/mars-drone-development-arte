1. High-level idea (clean framing)
What you are doing (one sentence)

Instead of controlling the UAV from AprilTag feedback, we use dual AprilTag pose estimation as a sensing system to measure relative motion (vibration) of a target object with respect to a stationary world reference.

Why this is smart

No closed-loop flight control required (huge reduction in risk)

Works even if the UAV is imperfectly stable

Lets you quantify vibration frequency, amplitude, and drift

Produces hardware-validated data that simulation can’t fake

Still directly relevant to inspection & structural monitoring

Key principle

You are not comparing tag poses in the camera frame.

You are computing:

𝑇
𝑣
𝑖
𝑏
𝑤
𝑜
𝑟
𝑙
𝑑
⊖
𝑇
𝑟
𝑒
𝑓
𝑤
𝑜
𝑟
𝑙
𝑑
T
vib
world
	​

⊖T
ref
world
	​


Which removes:

UAV drift

Small attitude changes

Camera vibration

Optical flow / IMU imperfections

2. System architecture (what runs where)
Physical setup

Tag A (Reference Tag)

Large tag (e.g. AprilTag 36h11, size ~15–20 cm)

Mounted on a stationary wall / structure

Defines the local world frame

Tag B (Vibration Tag)

Smaller tag on the vibrating object

Motion is what you want to measure

UAV

Holds position roughly (manual / Loiter / Guided)

Does not need precision hover

Software components (ROS-centric view)
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

3. AprilTag detection: why two nodes is correct

Running two AprilTag nodes is actually the cleanest way to do this.

Why not one node?

Different tag families / sizes / detection thresholds

Independent tuning (critical on hardware)

Cleaner topic separation

Easier debugging & logging

Node responsibilities
Node 1: apriltag_ref_node

Detects only the stationary tag

Publishes:

Pose of reference tag in camera frame

This pose defines a dynamic world reference tied to the real environment

Node 2: apriltag_vib_node

Detects only the vibrating tag

Publishes:

Pose of vibrating tag in camera frame

4. Coordinate frames (this is the most important part)

If this is wrong, everything is wrong — so let’s be precise.

Frames involved
camera_frame
ref_tag_frame
vib_tag_frame
virtual_world_frame (defined by ref tag)

What each node gives you

From AprilTag detection:

𝑇
𝑟
𝑒
𝑓
𝑐
𝑎
𝑚
T
ref
cam
	​


𝑇
𝑣
𝑖
𝑏
𝑐
𝑎
𝑚
T
vib
cam
	​


Each is a full SE(3) transform:

position (x, y, z)

orientation (quaternion / rotation matrix)

5. Pose differencing math (core idea)

You want vibration relative to the stationary reference, not the drone.

Step 1: Invert the reference pose
𝑇
𝑐
𝑎
𝑚
𝑟
𝑒
𝑓
=
(
𝑇
𝑟
𝑒
𝑓
𝑐
𝑎
𝑚
)
−
1
T
cam
ref
	​

=(T
ref
cam
	​

)
−1

This gives you:

camera pose expressed in the reference frame

Step 2: Transform vibrating tag into reference frame
𝑇
𝑣
𝑖
𝑏
𝑟
𝑒
𝑓
=
𝑇
𝑐
𝑎
𝑚
𝑟
𝑒
𝑓
⋅
𝑇
𝑣
𝑖
𝑏
𝑐
𝑎
𝑚
T
vib
ref
	​

=T
cam
ref
	​

⋅T
vib
cam
	​


Now:

UAV motion is canceled

Camera motion is canceled

What remains is true relative motion

Step 3: Extract vibration signals

From 
𝑇
𝑣
𝑖
𝑏
𝑟
𝑒
𝑓
T
vib
ref
	​

:

Translation:

𝑥
(
𝑡
)
,
𝑦
(
𝑡
)
,
𝑧
(
𝑡
)
x(t),y(t),z(t)

Rotation:

roll(t), pitch(t), yaw(t)

These time series are your raw vibration signals.

6. What “controller” do you actually need right now?
Short answer: none for flight

You are building a measurement controller, not a flight controller.

Components you do need
1. Transform Controller (kinematic)

Responsibilities:

Subscribe to both tag pose topics

Time-synchronize messages

Compute relative transform

Publish:

relative pose

relative twist (optional)

This is pure math, no dynamics.

2. Vibration Estimator

Responsibilities:

Maintain time buffer

Compute:

displacement amplitude

dominant frequency (FFT)

RMS vibration

Optional:

band-pass filtering

windowed FFT

3. Logger / Dataset Generator

Responsibilities:

Save:

raw poses

relative pose

timestamps

Format:

CSV / ROS bag

This becomes:

thesis plots

future controller training data

7. What controllers come later (but not now)

This architecture sets you up for:

Vision-based disturbance observers

Feedforward vibration compensation

Visual servoing with disturbance rejection

Model-based vibration tracking

But none of that blocks today’s progress.

8. Failure modes & mitigations (important for thesis)
Issue	Mitigation
Ref tag briefly lost	Hold last valid transform
Vib tag occluded	Skip frame
UAV yaw drift	Automatically canceled
Camera vibration	Automatically canceled
Lighting changes	Independent detector tuning

This is a robust measurement pipeline, not a fragile controller.

9. How this fits your thesis narrative

You can frame this as:

A vision-based relative pose sensing framework for vibration measurement in UAV inspection scenarios, validated on hardware and designed to decouple perception from flight control.

It strengthens:

sensor fusion discussion

estimation vs control separation

simulation → hardware realism

10. Copilot-ready prompt (paste this exactly)

Below is a direct prompt you can drop into GitHub Copilot or Copilot Chat:

Copilot Prompt:

I am working on a ROS-based UAV perception system using AprilTags.

I want to build a Python ROS node that subscribes to two AprilTag pose topics:

/apriltag_ref/pose → pose of a stationary reference tag detected in the camera frame

/apriltag_vib/pose → pose of a vibrating target tag detected in the camera frame

Each topic publishes geometry_msgs/PoseStamped.

The node should:

Time-synchronize the two pose messages.

Convert both poses into SE(3) homogeneous transformation matrices.

Invert the reference tag transform to obtain the camera pose in the reference frame.

Compute the relative transform of the vibrating tag with respect to the reference tag:
T_vib_ref = inv(T_ref_cam) * T_vib_cam

Publish the relative pose as a new PoseStamped topic called /relative_vibration_pose.

Log timestamped relative translation (x,y,z) and rotation (roll,pitch,yaw) to CSV.

Assume standard ROS Python (rospy), numpy, and tf.transformations are available.

Please structure the code cleanly with helper functions for transform math.

11. Final take

This is not a compromise — it’s a strategic move.

You:

get real hardware data

avoid unstable flight control debugging

still publish meaningful results

lay groundwork for future control

If you want, next we can:

sketch the exact ROS topic graph

write the relative transform node together

design the FFT/vibration analysis pipeline

help you write the methods section for the paper