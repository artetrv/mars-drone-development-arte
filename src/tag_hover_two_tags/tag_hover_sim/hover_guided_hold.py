#!/usr/bin/env python3
"""
hover_guided_hold.py — Coarse-position-and-hold controller for two-tag vibration measurement

STRATEGY
--------
Rather than trying to perfectly lock on to the tag (which causes limit-cycle oscillations
on real hardware due to low detection rates and PnP noise), this controller:
  1. Coarsely aligns the drone so both tags are centred in frame.
  2. Settles for 1.5 s (motion damps out, optical flow engages).
  3. Goes fully silent — optical flow sensor holds position.
     At HOLD entry, publishes True on /measurement_hold_active so the
     relative_vibration_pose node begins CSV logging automatically.
     No extra terminal or manual trigger needed.

STATE MACHINE
-------------
  SEARCH
    Spin at search_yaw rad/s until BOTH tag 0 (ref) AND tag 1 (vib) have
    fresh TF simultaneously (age < stale_tf_timeout).

  COARSE_ALIGN
    Sequential one-DOF-per-tick corrections in priority order:
      Step 1 — Yaw:      face horizontal midpoint of both tags  (|yaw_err| < yaw_tol)
      Step 2 — Distance: drive to target_distance from tag 0    (|z0-target| < dist_tol)
      Step 3 — Lateral:  centre horizontal midpoint mid_x → 0   (|mid_x| < lat_tol)
      Step 4 — Vertical: centre vertical midpoint mid_y → 0     (|mid_y| < vert_tol)
    Each tick applies only the highest-priority out-of-tolerance DOF,
    then returns. Rechecks from Step 1 every tick so no DOF drifts out
    while correcting a lower-priority one.
    When ALL four steps are within tolerance AND the tighter HOLD entry
    conditions hold for grace_hold_frames consecutive ticks → SETTLE.

  SETTLE
    Publish zero velocity for settle_duration seconds so drone motion damps.

  HOLD
    Publish zero velocity. Optical flow maintains position.
    Publish std_msgs/Bool(True) on /measurement_hold_active → measurement
    nodes begin CSV logging.
    Log drone position status every 5 s (informational only — no corrections).
    No automatic exit. Terminate with Ctrl-C; MAVProxy `mode land` to land.

MEASUREMENT AUTO-TRIGGER
------------------------
  HOLD entry  → publishes Bool(True)  on /measurement_hold_active
  Any revert  → publishes Bool(False) on /measurement_hold_active
  On shutdown → publishes Bool(False) to stop logging cleanly

Usage (sim):
  ros2 run tag_hover_sim hover_guided_hold --ros-args \\
    -p camera_frame:=iris_with_rgb_camera/gimbal/pitch_link/camera \\
    -p target_distance:=1.5

Usage (hardware — GUIDED mode with optical flow):
  ros2 run tag_hover_sim hover_guided_hold --ros-args \\
    -p camera_frame:=camera_color_optical_frame \\
    -p target_distance:=1.5 \\
    -p rate_hz:=10.0 \\
    -p use_sim_time:=false
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Bool
import tf2_ros
from tf2_ros import TransformException


SEARCH       = 'SEARCH'
COARSE_ALIGN = 'COARSE_ALIGN'
SETTLE       = 'SETTLE'
HOLD         = 'HOLD'


class HoverGuidedHold(Node):

    def __init__(self):
        super().__init__('hover_guided_hold')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('camera_frame',        'iris_with_rgb_camera/gimbal/pitch_link/camera')
        self.declare_parameter('ref_tag_id',          0)       # stationary reference tag
        self.declare_parameter('vib_tag_id',          1)       # vibrating structure tag
        self.declare_parameter('tag_frame_prefix',    'tag36h11')
        self.declare_parameter('target_distance',     1.5)     # m — desired distance to ref tag
        self.declare_parameter('search_yaw',          0.25)    # rad/s — spin rate in SEARCH
        # COARSE_ALIGN proportional gains
        self.declare_parameter('k_yaw',               0.4)     # rad/s per rad
        self.declare_parameter('k_dist',              0.3)     # m/s per m
        self.declare_parameter('k_lat',               0.3)     # m/s per m
        self.declare_parameter('k_vert',              0.2)     # m/s per m
        # Velocity clamps
        self.declare_parameter('max_yaw_rate',        0.5)     # rad/s
        self.declare_parameter('max_forward_vel',     0.3)     # m/s
        self.declare_parameter('max_lateral_vel',     0.2)     # m/s
        self.declare_parameter('max_vertical_vel',    0.2)     # m/s
        # COARSE_ALIGN step tolerances — loose (just need "good enough")
        self.declare_parameter('yaw_tol',             0.3)     # rad (~17 deg)
        self.declare_parameter('dist_tol',            0.4)     # m
        self.declare_parameter('lat_tol',             0.15)    # m
        self.declare_parameter('vert_tol',            0.1)     # m
        # HOLD entry conditions — tighter; must hold for grace_hold_frames consecutive ticks
        self.declare_parameter('hold_lat_tol',        0.2)     # m — |mid_x| must be < this
        self.declare_parameter('hold_dist_tol',       0.5)     # m — |z0 - target| must be < this
        self.declare_parameter('grace_hold_frames',   15)      # ticks (at 10 Hz = 1.5 s)
        # Fix 1 — tag loss grace: hold still in COARSE_ALIGN instead of jolting to SEARCH spin
        self.declare_parameter('loss_grace_frames',   8)       # ticks to hold still before reverting (0.8s at 10Hz)
        # Fix 2 — entry settling: pause corrections briefly on SEARCH→COARSE_ALIGN to absorb spin momentum
        self.declare_parameter('entry_settle_frames', 3)       # ticks of zero-vel on entry (0.3s at 10Hz)
        # SETTLE
        self.declare_parameter('settle_duration',     1.5)     # s
        # TF staleness and sanity bounds
        self.declare_parameter('stale_tf_timeout',    2.0)     # s — Pi detects ~1.2 Hz; 2 s is safe
        self.declare_parameter('max_z_m',             8.0)     # m — reject PnP back-solutions (~17m)
        # HOLD drift recovery — loose thresholds so minor wobble doesn't interrupt measurement
        self.declare_parameter('drift_lat_tol',       0.4)     # m — |mid_x| before re-aligning
        self.declare_parameter('drift_dist_tol',      0.8)     # m — |z0-target| before re-aligning
        # System
        self.declare_parameter('rate_hz',             10.0)
        self.declare_parameter('mavros_prefix',       '/mavros')
        self.declare_parameter('mavros_wait_timeout', 10.0)

        # Resolve frame names from parameters
        cam    = self.get_parameter('camera_frame').get_parameter_value().string_value
        ref_id = self.get_parameter('ref_tag_id').get_parameter_value().integer_value
        vib_id = self.get_parameter('vib_tag_id').get_parameter_value().integer_value
        pfx    = self.get_parameter('tag_frame_prefix').get_parameter_value().string_value

        self.camera_frame      = cam
        self.ref_frame         = f"{pfx}:{ref_id}"
        self.vib_frame         = f"{pfx}:{vib_id}"
        self.target_distance   = self.get_parameter('target_distance').get_parameter_value().double_value
        self.search_yaw        = self.get_parameter('search_yaw').get_parameter_value().double_value
        self.k_yaw             = self.get_parameter('k_yaw').get_parameter_value().double_value
        self.k_dist            = self.get_parameter('k_dist').get_parameter_value().double_value
        self.k_lat             = self.get_parameter('k_lat').get_parameter_value().double_value
        self.k_vert            = self.get_parameter('k_vert').get_parameter_value().double_value
        self.max_yaw_rate      = self.get_parameter('max_yaw_rate').get_parameter_value().double_value
        self.max_fwd           = self.get_parameter('max_forward_vel').get_parameter_value().double_value
        self.max_lat           = self.get_parameter('max_lateral_vel').get_parameter_value().double_value
        self.max_vert          = self.get_parameter('max_vertical_vel').get_parameter_value().double_value
        self.yaw_tol           = self.get_parameter('yaw_tol').get_parameter_value().double_value
        self.dist_tol          = self.get_parameter('dist_tol').get_parameter_value().double_value
        self.lat_tol           = self.get_parameter('lat_tol').get_parameter_value().double_value
        self.vert_tol          = self.get_parameter('vert_tol').get_parameter_value().double_value
        self.hold_lat_tol      = self.get_parameter('hold_lat_tol').get_parameter_value().double_value
        self.hold_dist_tol     = self.get_parameter('hold_dist_tol').get_parameter_value().double_value
        self.grace_hold_frames  = self.get_parameter('grace_hold_frames').get_parameter_value().integer_value
        self.loss_grace_frames  = self.get_parameter('loss_grace_frames').get_parameter_value().integer_value
        self.entry_settle_frames = self.get_parameter('entry_settle_frames').get_parameter_value().integer_value
        self.settle_duration   = self.get_parameter('settle_duration').get_parameter_value().double_value
        self.stale_tf_timeout  = self.get_parameter('stale_tf_timeout').get_parameter_value().double_value
        self.max_z_m           = self.get_parameter('max_z_m').get_parameter_value().double_value
        self.drift_lat_tol     = self.get_parameter('drift_lat_tol').get_parameter_value().double_value
        self.drift_dist_tol    = self.get_parameter('drift_dist_tol').get_parameter_value().double_value
        self.rate_hz           = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.mavros_wait_timeout = self.get_parameter('mavros_wait_timeout').get_parameter_value().double_value

        raw = self.get_parameter('mavros_prefix').get_parameter_value().string_value
        if not raw.startswith('/'):
            raw = '/' + raw
        self.mavros_prefix = raw.rstrip('/')

        # ── MAVROS state ─────────────────────────────────────────────────
        self._state        = None
        self._have_state   = False
        self._startup_time = rclpy.clock.Clock().now()
        self._mavros_ready = False
        self._drone_yaw    = 0.0  # ENU yaw from /mavros/local_position/pose

        self._state_sub = self.create_subscription(
            State, f"{self.mavros_prefix}/state", self._state_cb, 10)
        _sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self._pose_sub = self.create_subscription(
            PoseStamped, f"{self.mavros_prefix}/local_position/pose", self._pose_cb, _sensor_qos)

        # ── Publishers ───────────────────────────────────────────────────
        self._vel_pub   = self.create_publisher(
            Twist, f"{self.mavros_prefix}/setpoint_velocity/cmd_vel_unstamped", 10)
        self._debug_pub = self.create_publisher(Twist, '/hover_guided_hold_cmd', 10)
        # Measurement trigger — relative_vibration_pose subscribes and starts/stops CSV logging
        self._hold_pub  = self.create_publisher(Bool, '/measurement_hold_active', 10)

        # ── TF ───────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Controller state ─────────────────────────────────────────────
        self._ctrl_state        = SEARCH
        self._hold_ready_count  = 0     # consecutive ticks meeting HOLD entry conditions
        self._settle_start      = None  # rclpy Time when SETTLE began
        self._hold_active       = False # last value published on /measurement_hold_active
        self._hold_log_accum    = 0.0   # s since last HOLD status log
        self._loss_count        = 0     # Fix 1: ticks since tags lost in COARSE_ALIGN
        self._entry_count       = 0     # Fix 2: ticks spent in entry settling on COARSE_ALIGN entry

        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.1
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"hover_guided_hold READY | "
            f"ref={self.ref_frame} vib={self.vib_frame} cam={self.camera_frame} | "
            f"target={self.target_distance}m settle={self.settle_duration}s | "
            f"step tols: yaw={self.yaw_tol}rad dist={self.dist_tol}m "
            f"lat={self.lat_tol}m vert={self.vert_tol}m | "
            f"HOLD entry: |mid_x|<{self.hold_lat_tol}m |dist_err|<{self.hold_dist_tol}m "
            f"for {self.grace_hold_frames} ticks "
            f"({self.grace_hold_frames / self.rate_hz:.1f}s)"
        )

    # ── Callbacks ────────────────────────────────────────────────────────

    def _state_cb(self, msg: State):
        self._state      = msg
        self._have_state = True

    def _pose_cb(self, msg: PoseStamped):
        """Extract ENU yaw from quaternion — needed for body→world velocity rotation."""
        q = msg.pose.orientation
        self._drone_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    # ── Helpers ──────────────────────────────────────────────────────────

    def _clamp(self, val: float, limit: float) -> float:
        return max(-limit, min(limit, val))

    def _lookup_fresh(self, child_frame: str) -> tuple:
        """
        Look up camera→child_frame TF. Returns (x, y, z) if fresh and valid.
        Camera optical frame: X right, Y down, Z forward (into scene).
        Raises TransformException if stale, non-finite, or z < 0.2 m (PnP flip).
        """
        tf  = self._tf_buffer.lookup_transform(self.camera_frame, child_frame, Time())
        age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
        if age > self.stale_tf_timeout:
            raise TransformException(f"Stale {child_frame}: {age:.2f}s old")
        t = tf.transform.translation
        if not (math.isfinite(t.x) and math.isfinite(t.y) and math.isfinite(t.z)):
            raise TransformException(f"Non-finite TF: {child_frame}")
        if t.z < 0.2:
            # PnP occasionally returns a "back" solution (z ≤ 0) — reject it
            raise TransformException(f"z={t.z:.3f}m < 0.2 for {child_frame} (PnP flip?)")
        if t.z > self.max_z_m:
            # PnP back-solution (~17 m when drone is ~1.5 m away) — reject it
            raise TransformException(f"z={t.z:.3f}m > {self.max_z_m}m for {child_frame} (PnP back-solution)")
        return t.x, t.y, t.z

    def _body_to_world(self, fwd: float, lat: float) -> tuple:
        """
        Rotate body-frame (fwd, lat) velocity to ENU world frame by drone yaw.
        Required because cmd_vel_unstamped is in ENU, not body frame.
        Body convention: fwd = +X (forward), lat = +Y (left).
        """
        yaw = self._drone_yaw
        wx  = fwd * math.cos(yaw) - lat * math.sin(yaw)
        wy  = fwd * math.sin(yaw) + lat * math.cos(yaw)
        return wx, wy

    def _set_hold_active(self, active: bool):
        """Publish measurement trigger only when state changes (avoids bus chatter)."""
        if self._hold_active != active:
            self._hold_active = active
            msg = Bool()
            msg.data = active
            self._hold_pub.publish(msg)
            self.get_logger().info(
                f">>> /measurement_hold_active = {active} <<<"
            )

    def _publish(self, cmd: Twist):
        self._vel_pub.publish(cmd)
        self._debug_pub.publish(cmd)

    # ── Main loop ────────────────────────────────────────────────────────

    def _on_timer(self):
        # ── 1) MAVROS readiness gate ──────────────────────────────────────
        if not self._have_state:
            elapsed = (rclpy.clock.Clock().now() - self._startup_time).nanoseconds / 1e9
            if elapsed < self.mavros_wait_timeout:
                self.get_logger().info('Waiting for MAVROS...', throttle_duration_sec=2.0)
            else:
                self.get_logger().error('MAVROS timeout — check connection', throttle_duration_sec=5.0)
            return

        if not self._state.connected:
            self.get_logger().warn('MAVROS not connected', throttle_duration_sec=2.0)
            return

        if not self._mavros_ready:
            self._mavros_ready = True
            self.get_logger().info('MAVROS connected and ready')

        fcu_mode = self._state.mode.upper() if self._state.mode else ''
        if fcu_mode not in ['GUIDED', 'GUIDED_NOGPS', 'LOITER']:
            self.get_logger().warn(
                f'FCU mode={fcu_mode} — need GUIDED/GUIDED_NOGPS/LOITER',
                throttle_duration_sec=5.0)
            return

        cmd = Twist()  # default: all zeros

        # ── HOLD: optical flow holds; drift monitor triggers re-alignment ───
        if self._ctrl_state == HOLD:
            self._set_hold_active(True)
            try:
                x0, _, z0 = self._lookup_fresh(self.ref_frame)
                x1,  _, _  = self._lookup_fresh(self.vib_frame)
                mid_x    = (x0 + x1) / 2.0
                dist_err = z0 - self.target_distance

                if abs(mid_x) > self.drift_lat_tol or abs(dist_err) > self.drift_dist_tol:
                    self._set_hold_active(False)
                    self._ctrl_state = COARSE_ALIGN
                    self._entry_count = 0
                    self._loss_count  = 0
                    self._hold_ready_count = 0
                    self.get_logger().warn(
                        f"[HOLD] Drift exceeded — re-aligning | "
                        f"mid_x={mid_x:+.3f}m (tol±{self.drift_lat_tol}m) "
                        f"dist_err={dist_err:+.3f}m (tol±{self.drift_dist_tol}m)"
                    )
                    self._publish(cmd)
                    return

                self._hold_log_accum += 1.0 / self.rate_hz
                if self._hold_log_accum >= 5.0:
                    self._hold_log_accum = 0.0
                    self.get_logger().info(
                        f"[HOLD] z0={z0:.2f}m mid_x={mid_x:+.3f}m — "
                        f"measurement active, optical flow holding"
                    )

            except TransformException as ex:
                self._set_hold_active(False)
                self._ctrl_state = SEARCH
                self._loss_count  = 0
                self._entry_count = 0
                self._hold_ready_count = 0
                self.get_logger().warn(f"[HOLD] Tags lost — returning to SEARCH | {ex}")
                cmd.angular.z = self.search_yaw

            self._publish(cmd)
            return

        # ── SETTLE: zero velocity for settle_duration seconds ─────────────
        if self._ctrl_state == SETTLE:
            if self._settle_start is None:
                self._settle_start = self.get_clock().now()
                self.get_logger().info(
                    f"[SETTLE] zeroing velocity for {self.settle_duration:.1f}s ..."
                )
            elapsed = (self.get_clock().now() - self._settle_start).nanoseconds / 1e9
            if elapsed >= self.settle_duration:
                self._ctrl_state = HOLD
                self._hold_log_accum = 0.0
                self.get_logger().info(
                    f"SETTLE → HOLD | motion damped ({elapsed:.1f}s) — "
                    f"going silent, triggering measurement"
                )
            else:
                self.get_logger().info(
                    f"[SETTLE] {elapsed:.1f}/{self.settle_duration:.1f}s",
                    throttle_duration_sec=0.5
                )
            self._publish(cmd)
            return

        # ── Try to get both tags ──────────────────────────────────────────
        try:
            x0, y0, z0 = self._lookup_fresh(self.ref_frame)
            x1, y1, z1 = self._lookup_fresh(self.vib_frame)
        except TransformException as ex:
            # Fix 1: in COARSE_ALIGN, hold still during detection gaps instead of jolting to SEARCH spin
            if self._ctrl_state == COARSE_ALIGN:
                self._loss_count += 1
                if self._loss_count <= self.loss_grace_frames:
                    self.get_logger().warn(
                        f"[COARSE_ALIGN] tag lost — holding still "
                        f"({self._loss_count}/{self.loss_grace_frames} grace) | {ex}",
                        throttle_duration_sec=0.5
                    )
                    self._publish(cmd)  # zero velocity — no jolt
                    return
                # Grace expired — genuinely lost, revert to SEARCH
                self.get_logger().warn(
                    f"COARSE_ALIGN → SEARCH | grace expired ({self.loss_grace_frames} ticks) | {ex}"
                )
                self._ctrl_state = SEARCH
                self._loss_count = 0
                self._hold_ready_count = 0
                self._entry_count = 0
            elif self._ctrl_state != SEARCH:
                self._ctrl_state = SEARCH
                self._loss_count = 0
                self._hold_ready_count = 0
                self._entry_count = 0
                self._set_hold_active(False)
                self.get_logger().warn(f"→ SEARCH | tag lost: {ex}")
            else:
                self.get_logger().info(
                    f"[SEARCH] spinning at {self.search_yaw:.2f} rad/s ...",
                    throttle_duration_sec=2.0
                )
            cmd.angular.z = self.search_yaw
            self._publish(cmd)
            return

        # Both tags have fresh TF — reset loss counter and compute midpoint
        self._loss_count = 0
        mid_x = (x0 + x1) / 2.0
        mid_y = (y0 + y1) / 2.0

        if self._ctrl_state == SEARCH:
            self._ctrl_state = COARSE_ALIGN
            self._loss_count  = 0
            self._entry_count = 0
            self.get_logger().info(
                f"SEARCH → COARSE_ALIGN | "
                f"z0={z0:.2f}m z1={z1:.2f}m mid=({mid_x:+.3f},{mid_y:+.3f})m | "
                f"settling {self.entry_settle_frames} ticks before corrections"
            )

        # Fix 2: entry settling — hold zero velocity for entry_settle_frames ticks
        # after SEARCH→COARSE_ALIGN to absorb spin momentum before corrections start
        if self._entry_count < self.entry_settle_frames:
            self._entry_count += 1
            self.get_logger().info(
                f"[COARSE_ALIGN] settling ({self._entry_count}/{self.entry_settle_frames}) "
                f"| z0={z0:.2f}m mid_x={mid_x:+.3f}m",
                throttle_duration_sec=0.5
            )
            self._publish(cmd)  # zero velocity — drone physically decelerates
            return

        # ── COARSE_ALIGN: sequential one-DOF-per-tick correction ──────────
        # Yaw error: angle from camera optical axis to horizontal midpoint.
        # We use z0 as the depth reference (drone aligns to face the ref tag).
        yaw_err = -math.atan2(mid_x, z0)

        # Step 1 — Yaw
        if abs(yaw_err) > self.yaw_tol:
            yaw_cmd = self._clamp(self.k_yaw * yaw_err, self.max_yaw_rate)
            cmd.angular.z = yaw_cmd
            self.get_logger().info(
                f"[COARSE/YAW] yaw_err={math.degrees(yaw_err):+.1f}deg "
                f"cmd={yaw_cmd:+.3f}rad/s | mid_x={mid_x:+.3f}m",
                throttle_duration_sec=0.5
            )
            self._hold_ready_count = 0
            self._publish(cmd)
            return

        # Step 2 — Distance (forward/back to target_distance from ref tag)
        dist_err = z0 - self.target_distance
        if abs(dist_err) > self.dist_tol:
            fwd_body  = self._clamp(self.k_dist * dist_err, self.max_fwd)
            wx, wy    = self._body_to_world(fwd_body, 0.0)
            cmd.linear.x = wx
            cmd.linear.y = wy
            self.get_logger().info(
                f"[COARSE/DIST] z0={z0:.2f}m err={dist_err:+.3f}m "
                f"fwd={fwd_body:+.3f}m/s",
                throttle_duration_sec=0.5
            )
            self._hold_ready_count = 0
            self._publish(cmd)
            return

        # Step 3 — Lateral (centre horizontal midpoint, mid_x → 0)
        # mid_x > 0 = midpoint is right of centre → move right → negative body-Y (body Y+ = left)
        if abs(mid_x) > self.lat_tol:
            lat_body  = self._clamp(-self.k_lat * mid_x, self.max_lat)
            wx, wy    = self._body_to_world(0.0, lat_body)
            cmd.linear.x = wx
            cmd.linear.y = wy
            self.get_logger().info(
                f"[COARSE/LAT] mid_x={mid_x:+.3f}m "
                f"lat_body={lat_body:+.3f}m/s",
                throttle_duration_sec=0.5
            )
            self._hold_ready_count = 0
            self._publish(cmd)
            return

        # Step 4 — Vertical (centre vertical midpoint, mid_y → 0)
        # Camera Y+ is down; mid_y > 0 = midpoint below centre → move down → negative ENU-Z
        if abs(mid_y) > self.vert_tol:
            vert_cmd = self._clamp(-self.k_vert * mid_y, self.max_vert)
            cmd.linear.z = vert_cmd
            self.get_logger().info(
                f"[COARSE/VERT] mid_y={mid_y:+.3f}m "
                f"vert={vert_cmd:+.3f}m/s",
                throttle_duration_sec=0.5
            )
            self._hold_ready_count = 0
            self._publish(cmd)
            return

        # ── All four steps within tolerance ─────────────────────────────
        # Check tighter HOLD entry conditions and accumulate grace counter
        hold_ok = (abs(mid_x) < self.hold_lat_tol and abs(dist_err) < self.hold_dist_tol)

        if hold_ok:
            self._hold_ready_count += 1
            pct = self._hold_ready_count / self.grace_hold_frames * 100
            self.get_logger().info(
                f"[COARSE_ALIGN] HOLD conditions met "
                f"{self._hold_ready_count}/{self.grace_hold_frames} ({pct:.0f}%) | "
                f"mid_x={mid_x:+.3f}m z0={z0:.2f}m yaw={math.degrees(yaw_err):+.1f}deg",
                throttle_duration_sec=0.5
            )
            if self._hold_ready_count >= self.grace_hold_frames:
                self._ctrl_state = SETTLE
                self._settle_start = None
                self._hold_ready_count = 0
                self.get_logger().info(
                    f"COARSE_ALIGN → SETTLE | "
                    f"stable for {self.grace_hold_frames} ticks "
                    f"({self.grace_hold_frames / self.rate_hz:.1f}s)"
                )
        else:
            if self._hold_ready_count > 0:
                self.get_logger().warn(
                    f"[COARSE_ALIGN] HOLD conditions lost "
                    f"(had {self._hold_ready_count} ticks) | "
                    f"mid_x={mid_x:+.3f}m (need <{self.hold_lat_tol}m) "
                    f"dist_err={dist_err:+.3f}m (need <{self.hold_dist_tol}m)"
                )
            self._hold_ready_count = 0

        # Zero command while accumulating grace ticks
        self._publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = HoverGuidedHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Signal measurement nodes to stop logging on shutdown
        try:
            stop = Bool()
            stop.data = False
            node._hold_pub.publish(stop)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
