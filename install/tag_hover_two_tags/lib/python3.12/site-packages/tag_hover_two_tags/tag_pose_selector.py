#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray


def to_int_id(det):
    for attr in ('id', 'ids', 'fiducial_id', 'fiducial_ids'):
        if hasattr(det, attr):
            val = getattr(det, attr)
            try:
                return int(val[0]) if len(val) > 0 else 0
            except TypeError:
                try:
                    return int(val)
                except Exception:
                    pass
    return 0


def extract_pose_and_header(det):
    try:
        return det.pose.pose.pose, det.pose.header
    except AttributeError:
        pass
    try:
        if hasattr(det.pose, 'pose') and hasattr(det.pose, 'header'):
            return det.pose.pose, det.pose.header
    except AttributeError:
        pass
    try:
        if hasattr(det.pose, 'position') and hasattr(det.pose, 'orientation'):
            return det.pose, None
    except AttributeError:
        pass

    raise RuntimeError(
        f"Unsupported pose layout. det.pose attrs={dir(det.pose) if hasattr(det, 'pose') else 'NO pose'}"
    )


class TagPoseSelector(Node):
    def __init__(self):
        super().__init__('tag_pose_selector')

        self.declare_parameter('tag_id', 0)
        self.tag_id = int(self.get_parameter('tag_id').value)
        self.detections_topic = self.declare_parameter('detections_topic', '/detections').get_parameter_value().string_value
        self.output_topic = self.declare_parameter('output_topic', '/apriltag_ref/pose').get_parameter_value().string_value
        self.camera_frame = self.declare_parameter('camera_frame', '').get_parameter_value().string_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self._pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self._sub = self.create_subscription(AprilTagDetectionArray, self.detections_topic, self._on_detections, qos)

        self.get_logger().info(
            f"tag_pose_selector started. tag_id={self.tag_id}, detections_topic={self.detections_topic}, "
            f"output_topic={self.output_topic}"
        )

    def _on_detections(self, msg: AprilTagDetectionArray):
        if not msg.detections:
            return

        for det in msg.detections:
            if to_int_id(det) != self.tag_id:
                continue

            try:
                pose, header = extract_pose_and_header(det)
            except Exception as exc:
                self.get_logger().warn(f"Failed to extract pose for tag {self.tag_id}: {exc}")
                return

            out = PoseStamped()
            if header is not None and hasattr(header, 'stamp'):
                out.header.stamp = header.stamp
                out.header.frame_id = getattr(header, 'frame_id', '')
            else:
                out.header.stamp = self.get_clock().now().to_msg()
                out.header.frame_id = self.camera_frame or 'camera'

            if self.camera_frame:
                out.header.frame_id = self.camera_frame

            out.pose = pose
            self._pub.publish(out)
            self.get_logger().info(
                f"Published pose for tag {self.tag_id} on {self.output_topic}",
                throttle_duration_sec=2.0
            )
            return


def main(args=None):
    rclpy.init(args=args)
    node = TagPoseSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
