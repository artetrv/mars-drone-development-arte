#!/usr/bin/env python3
"""
hover_single_tag_controller.py — Controller B (Single-tag positioning)

Professor's requested variant: requires BOTH tags to be in frame before locking,
then positions the drone based on tag 0 (reference/stationary) ONLY.
Tag 1 (vibrating) is NOT used for positioning — it is monitored to ensure it
remains in frame, but its motion is not fed back into the control loop.

Rationale: Positioning on the static reference tag (tag 0) provides a stable,
non-vibrating anchor. If we positioned on the vibrating tag, the drone would
attempt to cancel the very motion it is trying to measure — corrupting the signal.
Tag 1 must remain visible so the measurement pipeline (relative_vibration_pose)
has both tags available, but its position does not influence the drone's pose.

Tag layout:
  - Tag 0 (reference): stationary background structure, ~1.5 m from drone
  - Tag 1 (vibrating): foreground vibrating structure, ~0.5 m from drone

State machine:
  SEARCH   — spin until BOTH tag 0 AND tag 1 have fresh TF
             (ensures tag 1 is visible before committing to a hover position)
  ALIGNING — yaw to centre tag 0 horizontally; no translation
             until |yaw_err| < yaw_align_threshold (0.25 rad, ~14 deg)
  LOCKED   — full 4-DOF on tag 0 only:
               yaw      → x0 → 0   (centre ref tag horizontally)
               forward  → z0 → target_distance
               lateral  → x0 → 0
               vertical → y0 → 0
             Tag 0 lost → SEARCH immediately (no grace — it is the positioning anchor)
             Tag 1 lost → 5-frame grace, then SEARCH (must keep it in frame)

Gain basis (THESIS_NOTES §11, eq. 4 — τ = 1/k_p):
  k_yaw=0.6  → τ=1.7s    k_dist=k_lat=0.35 → τ=2.9s    k_vert=0.15 → τ=6.7s

Usage (sim):
  ros2 run tag_hover_sim hover_single_tag --ros-args \\
    -p camera_frame:=iris_with_rgb_camera/gimbal/pitch_link/camera \\
    -p target_distance:=1.5

Usage (hardware):
  ros2 run tag_hover_sim hover_single_tag --ros-args \\
    -p camera_frame:=camera_color_optical_frame \\
    -p target_distance:=1.5 \\
    -p use_sim_time:=false
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, PointStamped
from mavros_msgs.msg import State
import tf2_ros
from tf2_ros import TransformException


SEARCH   = 'SEARCH'
ALIGNING = 'ALIGNING'
LOCKED   = 'LOCKED'


class HoverSingleTagController(Node):

    def __init__(self):
        super().__init__('hover_single_tag')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('camera_frame',        'iris_with_rgb_camera/gimbal/pitch_link/camera')
        self.declare_parameter('ref_tag_id',          0)       # positioning anchor (stationary)
        self.declare_parameter('vib_tag_id',          1)       # monitored only — not used for positioning
        self.declare_parameter('tag_frame_prefix',    'tag36h11')
        self.declare_parameter('target_distance',     1.5)     # m — desired distance to ref tag
        self.declare_parameter('search_yaw',          0.25)    # rad/s — spin rate in SEARCH
        self.declare_parameter('yaw_align_threshold', 0.25)    # rad (~14 deg) — gate for translation
        self.declare_parameter('lock_k_yaw',          0.6)     # P gain yaw     (τ ≈ 1.7 s)
        self.declare_parameter('lock_k_distance',     0.35)    # P gain forward  (τ ≈ 2.9 s)
        self.declare_parameter('lock_k_lateral',      0.35)    # P gain lateral  (τ ≈ 2.9 s)
        self.declare_parameter('lock_k_vertical',     0.15)    # P gain vertical (τ ≈ 6.7 s)
        self.declare_parameter('max_yaw_rate',        0.5)     # rad/s clamp
        self.declare_parameter('max_forward_vel',     0.3)     # m/s clamp
        self.declare_parameter('max_lateral_vel',     0.2)     # m/s clamp
        self.declare_parameter('max_vertical_vel',    0.2)     # m/s clamp
        self.declare_parameter('deadband_distance',   0.05)    # m
        self.declare_parameter('deadband_lateral',    0.03)    # m
        self.declare_parameter('deadband_vertical',   0.03)    # m
        self.declare_parameter('stale_tf_timeout',    0.5)     # s
        self.declare_parameter('loss_grace_frames',   5)       # frames before LOCKED→SEARCH on vib tag loss
        self.declare_parameter('rate_hz',             20.0)
        self.declare_parameter('mavros_prefix',       '/mavros')
        self.declare_parameter('mavros_wait_timeout', 10.0)

        cam    = self.get_parameter('camera_frame').get_parameter_value().string_value
        ref_id = self.get_parameter('ref_tag_id').get_parameter_value().integer_value
        vib_id = self.get_parameter('vib_tag_id').get_parameter_value().integer_value
        prefix = self.get_parameter('tag_frame_prefix').get_parameter_value().string_value

        self.camera_frame        = cam
        self.ref_frame           = f"{prefix}:{ref_id}"
        self.vib_frame           = f"{prefix}:{vib_id}"
        self.target_distance     = self.get_parameter('target_distance').get_parameter_value().double_value
        self.search_yaw          = self.get_parameter('search_yaw').get_parameter_value().double_value
        self.yaw_align_threshold = self.get_parameter('yaw_align_threshold').get_parameter_value().double_value
        self.k_yaw               = self.get_parameter('lock_k_yaw').get_parameter_value().double_value
        self.k_dist              = self.get_parameter('lock_k_distance').get_parameter_value().double_value
        self.k_lat               = self.get_parameter('lock_k_lateral').get_parameter_value().double_value
        self.k_vert              = self.get_parameter('lock_k_vertical').get_parameter_value().double_value
        self.max_yaw_rate        = self.get_parameter('max_yaw_rate').get_parameter_value().double_value
        self.max_forward_vel     = self.get_parameter('max_forward_vel').get_parameter_value().double_value
        self.max_lateral_vel     = self.get_parameter('max_lateral_vel').get_parameter_value().double_value
        self.max_vertical_vel    = self.get_parameter('max_vertical_vel').get_parameter_value().double_value
        self.db_dist             = self.get_parameter('deadband_distance').get_parameter_value().double_value
        self.db_lat              = self.get_parameter('deadband_lateral').get_parameter_value().double_value
        self.db_vert             = self.get_parameter('deadband_vertical').get_parameter_value().double_value
        self.stale_tf_timeout    = self.get_parameter('stale_tf_timeout').get_parameter_value().double_value
        self.loss_grace_frames   = self.get_parameter('loss_grace_frames').get_parameter_value().integer_value
        self.rate_hz             = self.get_parameter('rate_hz').get_parameter_value().double_value
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

        self._state_sub = self.create_subscription(
            State, f"{self.mavros_prefix}/state", self._state_cb, 10)

        # ── Publishers ───────────────────────────────────────────────────
        self._vel_pub        = self.create_publisher(
            Twist, f"{self.mavros_prefix}/setpoint_velocity/cmd_vel_unstamped", 10)
        self._debug_pub      = self.create_publisher(Twist, '/hover_single_tag_cmd', 10)
        # Diagnostics — published only in LOCKED; use for thesis validation plots
        self._diag_err_pub   = self.create_publisher(Twist, '/hover_diagnostics', 10)
        self._diag_pose_pub  = self.create_publisher(PointStamped, '/hover_ref_tag_pose', 10)

        # ── TF ───────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Controller state ─────────────────────────────────────────────
        self._ctrl_state   = SEARCH
        self._vib_loss_ctr = 0   # grace counter for vibrating tag (tag 1) only

        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.05
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"hover_single_tag [Controller B — single-tag positioning] | "
            f"ref={self.ref_frame} (positioning) vib={self.vib_frame} (monitor) cam={self.camera_frame} | "
            f"target={self.target_distance}m "
            f"k_yaw={self.k_yaw} k_dist={self.k_dist} k_lat={self.k_lat} k_vert={self.k_vert} | "
            f"thresh={self.yaw_align_threshold}rad "
            f"db=({self.db_dist},{self.db_lat},{self.db_vert})m "
            f"grace={self.loss_grace_frames}frames"
        )

    # ── Callbacks ────────────────────────────────────────────────────────

    def _state_cb(self, msg: State):
        self._state      = msg
        self._have_state = True

    # ── Helpers ──────────────────────────────────────────────────────────

    def _clamp(self, val: float, limit: float) -> float:
        return max(-limit, min(limit, val))

    def _deadband(self, val: float, db: float) -> float:
        """Return 0 if |val| < db, else val. Prevents chasing micro-errors."""
        return 0.0 if abs(val) < db else val

    def _lookup_fresh(self, child_frame: str):
        """
        Look up camera→child_frame TF. Returns (x, y, z) if fresh and valid.
        Raises TransformException if missing, stale, or non-finite.
        """
        tf  = self._tf_buffer.lookup_transform(self.camera_frame, child_frame, Time())
        age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
        if age > self.stale_tf_timeout:
            raise TransformException(f"Stale TF {child_frame}: {age:.2f}s old")
        t = tf.transform.translation
        if not (math.isfinite(t.x) and math.isfinite(t.y) and math.isfinite(t.z)):
            raise TransformException(f"Non-finite TF values for {child_frame}")
        if abs(t.z) < 1e-3:
            raise TransformException(f"TF z≈0 for {child_frame} (tag behind camera?)")
        return t.x, t.y, t.z

    # ── Main loop ────────────────────────────────────────────────────────

    def _on_timer(self):
        # 1) Wait for MAVROS
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
            self.get_logger().info('MAVROS ready')

        fcu_mode = self._state.mode.upper() if self._state.mode else ''
        if fcu_mode not in ['GUIDED', 'GUIDED_NOGPS', 'LOITER']:
            self.get_logger().warn(
                f'FCU mode={fcu_mode}, need GUIDED/GUIDED_NOGPS/LOITER',
                throttle_duration_sec=5.0)
            return

        cmd = Twist()

        # 2) SEARCH: require both tags before committing to a hover position
        if self._ctrl_state == SEARCH:
            try:
                self._lookup_fresh(self.ref_frame)
                self._lookup_fresh(self.vib_frame)
                # Both found — move to ALIGNING
                self._ctrl_state   = ALIGNING
                self._vib_loss_ctr = 0
                self.get_logger().info('Both tags found → ALIGNING on ref tag')
            except TransformException:
                cmd.angular.z = self.search_yaw
            self._publish(cmd)
            return

        # 3) ALIGNING / LOCKED: ref tag (tag 0) is the positioning anchor
        try:
            x0, y0, z0 = self._lookup_fresh(self.ref_frame)
        except TransformException as ex:
            # Ref tag lost — SEARCH immediately, no grace (it is the anchor)
            self._ctrl_state   = SEARCH
            self._vib_loss_ctr = 0
            self.get_logger().warn(f'Ref tag (ID 0) lost → SEARCH | {ex}')
            cmd.angular.z = self.search_yaw
            self._publish(cmd)
            return

        # 4) Monitor vib tag (tag 1) with grace period
        vib_in_frame = True
        try:
            self._lookup_fresh(self.vib_frame)
            self._vib_loss_ctr = 0
        except TransformException as ex:
            self._vib_loss_ctr += 1
            if self._vib_loss_ctr > self.loss_grace_frames:
                # Grace expired — must go back to SEARCH to relocate both tags
                self._ctrl_state   = SEARCH
                self._vib_loss_ctr = 0
                self.get_logger().warn(
                    f'Vib tag (ID 1) lost beyond grace ({self.loss_grace_frames} frames) → SEARCH | {ex}')
                cmd.angular.z = self.search_yaw
                self._publish(cmd)
                return
            vib_in_frame = False
            self.get_logger().warn(
                f'Vib tag lost ({self._vib_loss_ctr}/{self.loss_grace_frames} grace frames) | {ex}',
                throttle_duration_sec=0.5)

        # 5) Yaw error: angle to centre tag 0 horizontally
        # atan2(x0, z0) — horizontal angle from optical axis to ref tag
        yaw_error = -math.atan2(x0, z0)
        yaw_cmd   = self._clamp(self.k_yaw * yaw_error, self.max_yaw_rate)
        aligned   = abs(yaw_error) < self.yaw_align_threshold

        # 6) ALIGNING state
        if self._ctrl_state == ALIGNING:
            if aligned:
                self._ctrl_state = LOCKED
                self.get_logger().info(
                    f'Yaw aligned → LOCKED | z0={z0:.2f}m target={self.target_distance:.2f}m')
                # Fall through to LOCKED below
            else:
                cmd.angular.z = yaw_cmd
                self.get_logger().info(
                    f'[ALIGNING] yaw_err={yaw_error:.3f}rad ({math.degrees(yaw_error):.1f}deg) '
                    f'x0={x0:.3f}m z0={z0:.2f}m vib_in_frame={vib_in_frame}',
                    throttle_duration_sec=0.5)
                self._publish(cmd)
                return

        # 7) LOCKED: full 4-DOF regulation on tag 0 only
        dist_err = self._deadband(z0 - self.target_distance, self.db_dist)
        lat_err  = self._deadband(x0, self.db_lat)
        vert_err = self._deadband(y0, self.db_vert)

        fwd_cmd  = self._clamp(self.k_dist * dist_err,  self.max_forward_vel)
        lat_cmd  = self._clamp(-self.k_lat  * lat_err,  self.max_lateral_vel)
        vert_cmd = self._clamp(-self.k_vert * vert_err, self.max_vertical_vel)

        cmd.linear.x  = fwd_cmd
        cmd.linear.y  = lat_cmd
        cmd.linear.z  = vert_cmd
        cmd.angular.z = yaw_cmd

        self.get_logger().info(
            f'[LOCKED] yaw={yaw_error:.3f}rad | '
            f'z0={z0:.2f}m dist_err={dist_err:+.3f}m fwd={fwd_cmd:+.3f} | '
            f'x0={x0:+.3f}m lat={lat_cmd:+.3f} | '
            f'y0={y0:+.3f}m vert={vert_cmd:+.3f} | '
            f'vib_in_frame={vib_in_frame}',
            throttle_duration_sec=0.5)

        # ── Diagnostics (LOCKED only) ─────────────────────────────────────
        # /hover_diagnostics: errors per axis — use for validation plots
        #   linear.x = dist_err (m)   linear.y = lat_err (m)
        #   linear.z = vert_err (m)   angular.z = yaw_err (rad)
        diag_err = Twist()
        diag_err.linear.x  = float(dist_err)
        diag_err.linear.y  = float(lat_err)
        diag_err.linear.z  = float(vert_err)
        diag_err.angular.z = float(yaw_error)
        self._diag_err_pub.publish(diag_err)

        # /hover_ref_tag_pose: raw x0, y0, z0 — use for position time-series plots
        pt = PointStamped()
        pt.header.stamp    = self.get_clock().now().to_msg()
        pt.header.frame_id = self.camera_frame
        pt.point.x = float(x0)
        pt.point.y = float(y0)
        pt.point.z = float(z0)
        self._diag_pose_pub.publish(pt)

        self._publish(cmd)

    def _publish(self, cmd: Twist):
        self._vel_pub.publish(cmd)
        self._debug_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = HoverSingleTagController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
