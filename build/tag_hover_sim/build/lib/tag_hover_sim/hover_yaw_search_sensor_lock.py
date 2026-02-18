#!/usr/bin/env python3
import math
from enum import Enum

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time


class ControlPhase(Enum):
    """State machine phases for hybrid visual-sensor control."""
    SEARCH = 0          # Yaw search until tag detected
    ALIGN = 1           # Vision-based alignment and distance regulation (continuous control)
    HOVER_BOX = 2       # Event-based correction within bounding box (pause control inside)
    SENSOR_HOVER = 3    # Sensor-based autonomous hover (go silent, let FCU hold)


class HoverYawSearch(Node):
    """
    3-Phase Hybrid Visual-Sensor Hover Controller for AprilTag Visual Servoing
    
    Architecture: State machine with vision-based alignment followed by sensor-based hold.
    
    Phase 1 (ALIGN): Vision-based continuous control (yaw + distance + lateral alignment)
    Phase 2 (HOVER_BOX): Event-based correction via 3D bounding box; zero velocity inside box
    Phase 3 (SENSOR_HOVER): Silent handoff to FCU's optical flow + onboard estimators
    
    Inputs:
    - SUB: /mavros/state (mavros_msgs/State) - FCU connection status
    - TF:  camera_frame -> tag_frame - relative pose from apriltag_pnp_broadcaster
    
    Outputs:
    - PUB: /hover_yaw_cmd (geometry_msgs/Twist) - debug echo of velocity command
    - PUB: /mavros/setpoint_velocity/cmd_vel_unstamped (Twist) - body-frame velocity to FCU (or zero in SENSOR_HOVER)

    State Transitions:
    - SEARCH → ALIGN (tag detected)
    - ALIGN → HOVER_BOX (equilibrium reached: tag in box for N seconds)
    - HOVER_BOX → SENSOR_HOVER (tag stable in box for N seconds)
    - Any → SEARCH (tag lost)
    """

    def __init__(self):
        super().__init__('hover_yaw_search')

        # Parameters: Phase 1 (ALIGN) - Vision-based control
        self.declare_parameter('mode', 'SEARCH')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('search_yaw', 0.25)          # rad/s
        self.declare_parameter('lock_k_yaw', 0.1)           # P gain for yaw
        self.declare_parameter('lock_k_distance', 0.2)      # P gain for forward/backward (m/s per meter error)
        self.declare_parameter('lock_k_lateral', 0.1)       # P gain for left/right (m/s per meter error)
        self.declare_parameter('yaw_align_threshold', 0.1)  # Radians; only move forward/lateral when |yaw_error| < this
        self.declare_parameter('target_distance', 2.0)      # Target distance from tag in meters
        self.declare_parameter('max_forward_vel', 0.5)      # m/s clamp for forward/backward
        self.declare_parameter('max_lateral_vel', 0.5)      # m/s clamp for left/right
        
        # Parameters: Phase 2 (HOVER_BOX) - Bounding box thresholds (3D camera-relative pose)
        self.declare_parameter('lateral_box_m', 0.25)       # ± meters (left/right tolerance)
        self.declare_parameter('distance_box_m', 0.30)      # ± meters (forward/back tolerance around target_distance)
        self.declare_parameter('yaw_box_rad', 0.08)         # ± radians (yaw alignment tolerance)
        
        # Parameters: Phase transitions
        self.declare_parameter('equilibrium_time_s', 2.0)   # Time tag must stay in box before advancing phase
        
        # Common parameters
        self.declare_parameter('camera_frame', 'camera')
        self.declare_parameter('body_frame', 'base_link')
        self.declare_parameter('tag_frame', 'tag36h11:0')
        self.declare_parameter('max_yaw_rate', 0.6)         # rad/s clamp
        self.declare_parameter('mavros_wait_timeout', 10.0)  # seconds to wait for MAVROS
        self.declare_parameter('mavros_prefix', '/mavros')   # base path for MAVROS topics

        # Load parameters
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.search_yaw = self.get_parameter('search_yaw').get_parameter_value().double_value
        self.lock_k_yaw = self.get_parameter('lock_k_yaw').get_parameter_value().double_value
        self.lock_k_distance = self.get_parameter('lock_k_distance').get_parameter_value().double_value
        self.lock_k_lateral = self.get_parameter('lock_k_lateral').get_parameter_value().double_value
        self.yaw_align_threshold = self.get_parameter('yaw_align_threshold').get_parameter_value().double_value
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.max_forward_vel = self.get_parameter('max_forward_vel').get_parameter_value().double_value
        self.max_lateral_vel = self.get_parameter('max_lateral_vel').get_parameter_value().double_value
        
        self.lateral_box_m = self.get_parameter('lateral_box_m').get_parameter_value().double_value
        self.distance_box_m = self.get_parameter('distance_box_m').get_parameter_value().double_value
        self.yaw_box_rad = self.get_parameter('yaw_box_rad').get_parameter_value().double_value
        self.equilibrium_time_s = self.get_parameter('equilibrium_time_s').get_parameter_value().double_value
        
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.body_frame = self.get_parameter('body_frame').get_parameter_value().string_value
        self.tag_frame = self.get_parameter('tag_frame').get_parameter_value().string_value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').get_parameter_value().double_value
        self.mavros_wait_timeout = self.get_parameter('mavros_wait_timeout').get_parameter_value().double_value
        raw_prefix = self.get_parameter('mavros_prefix').get_parameter_value().string_value
        raw_prefix = raw_prefix if raw_prefix is not None else '/mavros'
        if not raw_prefix.startswith('/'):
            raw_prefix = '/' + raw_prefix
        self.mavros_prefix = raw_prefix.rstrip('/') or ''

        # State machine
        self.phase = ControlPhase.SEARCH
        self.equilibrium_start_time = None  # Track when tag entered box

        # State from MAVROS
        self._state: State | None = None
        self._have_state = False
        self._startup_time = rclpy.clock.Clock().now()
        self._mavros_ready_logged = False

        state_topic = f"{self.mavros_prefix}/state" if self.mavros_prefix else '/mavros/state'
        self._state_sub = self.create_subscription(
            State,
            state_topic,
            self._state_cb,
            10
        )

        # AprilTag detections for tag size measurement
        # Use generic message subscription to avoid import errors
        self._detections = None
        try:
            # Try to get the message type from apriltag_msgs (not apriltag_ros)
            msg_type = get_message('apriltag_msgs/msg/AprilTagDetectionArray')
            self._detections_sub = self.create_subscription(
                msg_type,
                '/detections',
                self._detections_cb,
                10
            )
        except Exception as e:
            self.get_logger().warn(f"Could not subscribe to apriltag detections: {e}. Distance control disabled.")

        # Debug command topic
        self._hover_cmd_pub = self.create_publisher(
            Twist,
            '/hover_yaw_cmd',
            10
        )

        # Velocity setpoint to FCU
        vel_topic = f"{self.mavros_prefix}/setpoint_velocity/cmd_vel_unstamped" if self.mavros_prefix else '/mavros/setpoint_velocity/cmd_vel_unstamped'
        self._vel_pub = self.create_publisher(
            Twist,
            vel_topic,
            10
        )

        # TF listener for camera -> tag
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Timer
        period = 1.0 / self.rate_hz if self.rate_hz > 0.0 else 0.05
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"hover_yaw_search started. mode={self.mode}, "
            f"rate_hz={self.rate_hz}, search_yaw={self.search_yaw} rad/s, "
            f"lock_k_yaw={self.lock_k_yaw}, lock_k_distance={self.lock_k_distance} (m/s per m), "
            f"lock_k_lateral={self.lock_k_lateral} (m/s per m), yaw_align_threshold={self.yaw_align_threshold} rad, "
            f"target_distance={self.target_distance} m, "
            f"camera_frame={self.camera_frame}, tag_frame={self.tag_frame}"
        )

    # ------------------------- Callbacks -------------------------

    def _state_cb(self, msg: State):
        self._state = msg
        self._have_state = True

    def _detections_cb(self, msg):
        """Callback for AprilTag detections (generic message)."""
        self._detections = msg

    # ------------------------- Helper Methods -------------------------

    def _get_tag_size(self) -> float | None:
        """
        DEPRECATED: Use 3D distance from TF instead.
        Kept for reference only.
        """
        return None

    # ------------------------- Core loop -------------------------

    def _on_timer(self):
        """
        3-Phase state machine:
        PHASE 1 (ALIGN):    Vision-based continuous P-control (align to tag, move to target distance)
        PHASE 2 (HOVER_BOX): Event-based bounding box mode (only correct if outside box)
        PHASE 3 (SENSOR_HOVER): Silent handoff (FCU holds via optical flow, no vision commands)
        
        Transitions:
        - SEARCH → ALIGN: Tag detected
        - ALIGN → HOVER_BOX: Tag stable in box for equilibrium_time_s
        - HOVER_BOX → SENSOR_HOVER: Tag stable in box for equilibrium_time_s more
        - Any → SEARCH: Tag lost
        """
        # 1) Wait for MAVROS to initialize
        if not self._have_state or self._state is None:
            elapsed = (rclpy.clock.Clock().now() - self._startup_time).nanoseconds / 1e9
            if not self._mavros_ready_logged and elapsed < self.mavros_wait_timeout:
                self.get_logger().info(f'Waiting for MAVROS initialization... ({elapsed:.1f}s/{self.mavros_wait_timeout:.1f}s)')
                return
            elif elapsed >= self.mavros_wait_timeout:
                self.get_logger().error(f'MAVROS did not initialize within {self.mavros_wait_timeout}s timeout')
                return
            else:
                self._mavros_ready_logged = True
                self.get_logger().info(f'MAVROS state received after {elapsed:.1f}s')
        
        if not self._state.connected:
            self.get_logger().warn('MAVROS connected: False', throttle_duration_sec=2.0)
            return
        
        if not self._mavros_ready_logged:
            self._mavros_ready_logged = True
            self.get_logger().info('MAVROS connected and ready')

        # Verify FCU is in a compatible mode
        mode = self._state.mode.upper() if self._state.mode else ''
        if mode not in ['GUIDED', 'GUIDED_NOGPS', 'LOITER']:
            self.get_logger().warn(
                f'FCU mode is {mode}, expected GUIDED/GUIDED_NOGPS/LOITER for testing.',
                throttle_duration_sec=5.0
            )
            return

        cmd = Twist()

        # ==================== TRY TO GET TAG POSE ====================
        try:
            tf = self._tf_buffer.lookup_transform(
                self.camera_frame,
                self.tag_frame,
                Time()
            )

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.translation.z

            # Sanity check
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)) or abs(z) < 1e-3:
                raise TransformException("Non-finite TF or z ≈ 0")

            # ==================== COMPUTE RELATIVE ERRORS ====================
            lateral_error = -x
            vertical_error = -y
            distance_error = z - self.target_distance
            yaw_error = -math.atan2(x, z)

            # ==================== STATE MACHINE ====================
            
            if self.phase == ControlPhase.SEARCH:
                # Tag just found; transition to ALIGN
                self.phase = ControlPhase.ALIGN
                self.equilibrium_start_time = None
                self.get_logger().info(f"[STATE] SEARCH → ALIGN (tag detected at {z:.2f}m)")

            if self.phase == ControlPhase.ALIGN:
                # Apply Phase 1 continuous P-control on all DOFs
                yaw_cmd = self.lock_k_yaw * yaw_error
                yaw_cmd = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_cmd))

                forward_cmd = self.lock_k_distance * distance_error
                forward_cmd = max(-self.max_forward_vel, min(self.max_forward_vel, forward_cmd))

                lateral_cmd = self.lock_k_lateral * lateral_error
                lateral_cmd = max(-self.max_lateral_vel, min(self.max_lateral_vel, lateral_cmd))

                max_vertical_vel = 0.3
                vertical_cmd = self.lock_k_vertical * vertical_error
                vertical_cmd = max(-max_vertical_vel, min(max_vertical_vel, vertical_cmd))

                cmd.linear.x = forward_cmd
                cmd.linear.y = lateral_cmd
                cmd.linear.z = vertical_cmd
                cmd.angular.z = yaw_cmd

                # Check if tag is within bounding box (candidate for Phase 2)
                in_box = (abs(lateral_error) <= self.lateral_box_m and
                          abs(distance_error) <= self.distance_box_m and
                          abs(yaw_error) <= self.yaw_box_rad)

                if in_box:
                    if self.equilibrium_start_time is None:
                        self.equilibrium_start_time = self.get_clock().now()
                        self.get_logger().info(f"[EQUILIBRIUM] Timer started (ALIGN → HOVER_BOX candidate)")
                    else:
                        elapsed = (self.get_clock().now() - self.equilibrium_start_time).nanoseconds / 1e9
                        if elapsed >= self.equilibrium_time_s:
                            self.phase = ControlPhase.HOVER_BOX
                            self.equilibrium_start_time = None
                            self.get_logger().info(f"[STATE] ALIGN → HOVER_BOX (equilibrium reached after {elapsed:.2f}s)")
                        else:
                            self.get_logger().debug(
                                f"[EQUILIBRIUM] In box: {elapsed:.2f}s / {self.equilibrium_time_s:.2f}s",
                                throttle_duration_sec=1.0
                            )
                else:
                    if self.equilibrium_start_time is not None:
                        self.equilibrium_start_time = None
                        self.get_logger().debug("[EQUILIBRIUM] Tag left box; timer reset")

                self.get_logger().debug(
                    f"ALIGN | yaw_err={yaw_error:.3f}rad | dist={z:.2f}m dist_err={distance_error:.3f}m | "
                    f"lat_err={lateral_error:.3f}m | in_box={in_box}",
                    throttle_duration_sec=1.0
                )

            elif self.phase == ControlPhase.HOVER_BOX:
                # Phase 2: Only correct if tag is OUTSIDE the bounding box
                in_box = (abs(lateral_error) <= self.lateral_box_m and
                          abs(distance_error) <= self.distance_box_m and
                          abs(yaw_error) <= self.yaw_box_rad)

                if in_box:
                    # Tag is inside box; publish zero velocity (pause correction)
                    cmd.linear.x = 0.0
                    cmd.linear.y = 0.0
                    cmd.linear.z = 0.0
                    cmd.angular.z = 0.0

                    if self.equilibrium_start_time is None:
                        self.equilibrium_start_time = self.get_clock().now()
                        self.get_logger().info(f"[EQUILIBRIUM] Dwell timer started (HOVER_BOX → SENSOR_HOVER candidate)")
                    else:
                        elapsed = (self.get_clock().now() - self.equilibrium_start_time).nanoseconds / 1e9
                        if elapsed >= self.equilibrium_time_s:
                            self.phase = ControlPhase.SENSOR_HOVER
                            self.equilibrium_start_time = None
                            self.get_logger().info(f"[STATE] HOVER_BOX → SENSOR_HOVER (silent handoff after {elapsed:.2f}s)")
                        else:
                            self.get_logger().debug(
                                f"[EQUILIBRIUM] In box (dwell): {elapsed:.2f}s / {self.equilibrium_time_s:.2f}s",
                                throttle_duration_sec=1.0
                            )
                else:
                    # Tag outside box; resume vision control
                    yaw_cmd = self.lock_k_yaw * yaw_error
                    yaw_cmd = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_cmd))

                    forward_cmd = self.lock_k_distance * distance_error
                    forward_cmd = max(-self.max_forward_vel, min(self.max_forward_vel, forward_cmd))

                    lateral_cmd = self.lock_k_lateral * lateral_error
                    lateral_cmd = max(-self.max_lateral_vel, min(self.max_lateral_vel, lateral_cmd))

                    max_vertical_vel = 0.3
                    vertical_cmd = self.lock_k_vertical * vertical_error
                    vertical_cmd = max(-max_vertical_vel, min(max_vertical_vel, vertical_cmd))

                    cmd.linear.x = forward_cmd
                    cmd.linear.y = lateral_cmd
                    cmd.linear.z = vertical_cmd
                    cmd.angular.z = yaw_cmd

                    if self.equilibrium_start_time is not None:
                        self.equilibrium_start_time = None
                        self.get_logger().debug("[EQUILIBRIUM] Tag exited box; resuming corrections")

                self.get_logger().debug(
                    f"HOVER_BOX | yaw_err={yaw_error:.3f}rad | dist={z:.2f}m dist_err={distance_error:.3f}m | "
                    f"lat_err={lateral_error:.3f}m | in_box={in_box}",
                    throttle_duration_sec=1.0
                )

            elif self.phase == ControlPhase.SENSOR_HOVER:
                # Phase 3: Silent mode — do NOT publish any velocity commands
                # FCU holds position via optical flow + rangefinder + EKF
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.linear.z = 0.0
                cmd.angular.z = 0.0

                # Supervisor: check if tag is still visible and in box; if lost, fall back to SEARCH
                in_box = (abs(lateral_error) <= self.lateral_box_m and
                          abs(distance_error) <= self.distance_box_m and
                          abs(yaw_error) <= self.yaw_box_rad)

                self.get_logger().debug(
                    f"SENSOR_HOVER (silent) | tag at {z:.2f}m | in_box={in_box}",
                    throttle_duration_sec=2.0
                )

        except TransformException as ex:
            # Tag lost: fall back to SEARCH mode in all cases
            if self.phase != ControlPhase.SEARCH:
                self.phase = ControlPhase.SEARCH
                self.equilibrium_start_time = None
                self.get_logger().warn(
                    f"[STATE] Tag lost ({str(ex)}); falling back to SEARCH"
                )

            # SEARCH mode: spin yaw to find tag again
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.z = self.search_yaw

        # Publish velocity command (both debug and actual)
        self._hover_cmd_pub.publish(cmd)
        self._vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = HoverYawSearch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
