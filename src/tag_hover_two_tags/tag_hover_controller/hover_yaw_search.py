#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from mavros_msgs.msg import State

import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time


class HoverYawSearch(Node):
    """
    Semi-autonomous yaw controller:

    - SUB: /mavros/state (mavros_msgs/State)
    - TF:  camera -> tag36h11:0   (from apriltag_pnp_broadcaster)
    - PUB: /hover_yaw_cmd (geometry_msgs/Twist)               [debug]
    - PUB: /mavros/setpoint_velocity/cmd_vel_unstamped (Twist) [to FCU]

    Modes:
      SEARCH: constant yaw rate (search_yaw) until tag seen
      LOCK:   yaw to keep tag centered using simple P controller on yaw_error
    """

    def __init__(self):
        super().__init__('hover_yaw_search')

        # Parameters
        self.declare_parameter('mode', 'SEARCH')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('search_yaw', 0.25)          # rad/s
        self.declare_parameter('lock_k_yaw', 1.0)           # P gain for yaw
        self.declare_parameter('camera_frame', 'camera')
        self.declare_parameter('tag_frame', 'tag36h11:0')
        self.declare_parameter('max_yaw_rate', 0.6)         # rad/s clamp

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.search_yaw = self.get_parameter('search_yaw').get_parameter_value().double_value
        self.lock_k_yaw = self.get_parameter('lock_k_yaw').get_parameter_value().double_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.tag_frame = self.get_parameter('tag_frame').get_parameter_value().string_value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').get_parameter_value().double_value

        # State from MAVROS
        self._state: State | None = None
        self._have_state = False

        self._state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self._state_cb,
            10
        )

        # Debug command topic
        self._hover_cmd_pub = self.create_publisher(
            Twist,
            '/hover_yaw_cmd',
            10
        )

        # Velocity setpoint to FCU
        self._vel_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
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
            f"camera_frame={self.camera_frame}, tag_frame={self.tag_frame}"
        )

    # ------------------------- Callbacks -------------------------

    def _state_cb(self, msg: State):
        self._state = msg
        self._have_state = True

    # ------------------------- Core loop -------------------------

    def _on_timer(self):
        # 1) Basic safety: ensure MAVROS is giving us state
        if not self._have_state or self._state is None:
            self.get_logger().warn('No /mavros/state received yet.', throttle_duration_sec=2.0)
            return

        if not self._state.connected:
            self.get_logger().warn('MAVROS connected: False', throttle_duration_sec=2.0)
            return

        # For now, we only *test* in GUIDED/LOITER/GUIDED_NOGPS
        mode = self._state.mode.upper() if self._state.mode else ''
        if mode not in ['GUIDED', 'GUIDED_NOGPS', 'LOITER']:
            self.get_logger().warn(
                f'FCU mode is {mode}, expected GUIDED/GUIDED_NOGPS/LOITER for testing.',
                throttle_duration_sec=5.0
            )
            return

        cmd = Twist()

        if self.mode.upper() == 'LOCK':
            # Try to get camera -> tag transform
            try:
                # Latest transform
                tf = self._tf_buffer.lookup_transform(
                    self.camera_frame,
                    self.tag_frame,
                    Time()
                )

                x = tf.transform.translation.x
                y = tf.transform.translation.y
                z = tf.transform.translation.z

                # Basic sanity
                if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z) or abs(z) < 1e-3:
                    raise TransformException("Non-finite TF values or z ≈ 0")

                # In camera frame (x right, y down, z forward), yaw error ~ atan2(x, z)
                yaw_error = math.atan2(x, z)

                yaw_cmd = self.lock_k_yaw * yaw_error
                # Clamp yaw rate
                yaw_cmd = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_cmd))

                cmd.angular.z = yaw_cmd

                # Optional: small forward/backward nudges based on distance (z)
                # For now, keep linear all zero; you manually control height/pos
                # cmd.linear.x = 0.0
                # cmd.linear.y = 0.0
                # cmd.linear.z = 0.0

                self.get_logger().info(
                    f"LOCK: yaw_error={yaw_error:.3f} rad, yaw_cmd={yaw_cmd:.3f} rad/s, "
                    f"tag_pos_cam=[{x:.2f}, {y:.2f}, {z:.2f}]",
                    throttle_duration_sec=0.5
                )

            except TransformException as ex:
                # If we can't get TF, fall back to SEARCH behavior
                self.get_logger().warn(
                    f"LOCK: no valid TF {self.camera_frame}->{self.tag_frame}: {str(ex)}; "
                    f"falling back to SEARCH yaw.",
                    throttle_duration_sec=2.0
                )
                cmd.angular.z = self.search_yaw

        else:
            # SEARCH mode: constant yaw
            cmd.angular.z = self.search_yaw

        # Publish debug & actual velocity command
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
