#!/usr/bin/env python3
"""Draw AprilTag boxes on the camera image for RViz2 viewing."""

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge


def _tag_id_of(det) -> int:
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


def _corners_of(det):
    for attr in ('corners', 'pixel_corners', 'px', 'points'):
        if hasattr(det, attr):
            corners = getattr(det, attr)
            pts = []
            for p in corners[:4]:
                try:
                    pts.append((float(p.x), float(p.y)))
                except AttributeError:
                    try:
                        pts.append((float(p[0]), float(p[1])))
                    except Exception:
                        pass
            if len(pts) == 4:
                return pts
    if hasattr(det, 'center') and hasattr(det, 'size'):
        cx, cy = float(det.center.x), float(det.center.y)
        s = float(det.size) if hasattr(det, 'size') else 40.0
        return [(cx - s, cy - s), (cx + s, cy - s), (cx + s, cy + s), (cx - s, cy + s)]
    return None


class TagOverlay(Node):
    def __init__(self):
        super().__init__('tag_overlay')
        self.bridge = CvBridge()
        self.last_img = None
        self.last_header = None

        qos_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_det = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        img_topic = self.declare_parameter('image_topic', '/camera/image_raw').get_parameter_value().string_value
        det_topic = self.declare_parameter('detections_topic', '/detections').get_parameter_value().string_value
        out_topic = self.declare_parameter('output_topic', '/image_with_tags').get_parameter_value().string_value

        self.sub_img = self.create_subscription(Image, img_topic, self._on_image, qos_img)
        self.sub_det = self.create_subscription(AprilTagDetectionArray, det_topic, self._on_dets, qos_det)
        self.pub = self.create_publisher(Image, out_topic, 10)

        self.get_logger().info(
            f"Overlay listening image={img_topic}, detections={det_topic}, publishing {out_topic}"
        )

    def _on_image(self, msg: Image):
        try:
            self.last_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_header = msg.header
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge error: {exc}")

    def _on_dets(self, msg: AprilTagDetectionArray):
        if self.last_img is None:
            return

        frame = self.last_img.copy()
        for det in msg.detections:
            tid = _tag_id_of(det)
            pts = _corners_of(det)
            if pts:
                pts_i = np.array(pts, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(frame, [pts_i], isClosed=True, thickness=2, color=(0, 255, 0))
                x, y = int(pts[0][0]), int(pts[0][1])
            else:
                h, w = frame.shape[:2]
                x, y = w // 2, h // 2

            cv2.putText(
                frame,
                f"tag {tid}",
                (x, max(0, y - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        if self.last_header is not None:
            out.header = self.last_header
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TagOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
