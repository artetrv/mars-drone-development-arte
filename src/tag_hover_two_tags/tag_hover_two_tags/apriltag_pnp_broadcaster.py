#!/usr/bin/env python3
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


def quat_from_rvec(rvec):
    R, _ = cv2.Rodrigues(rvec)
    qw = math.sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2]) / 2.0
    qx = (R[2, 1] - R[1, 2]) / (4 * qw)
    qy = (R[0, 2] - R[2, 0]) / (4 * qw)
    qz = (R[1, 0] - R[0, 1]) / (4 * qw)
    q = Quaternion()
    q.x, q.y, q.z, q.w = float(qx), float(qy), float(qz), float(qw)
    return q


class TagPnPBroadcaster(Node):
    def __init__(self):
        super().__init__('apriltag_pnp_broadcaster')
        self.camera_frame = self.declare_parameter('camera_frame', 'camera').get_parameter_value().string_value
        self.tag_prefix = self.declare_parameter('tag_prefix', 'tag36h11').get_parameter_value().string_value
        self.tag_size = float(self.declare_parameter('tag_size_m', 0.0673).get_parameter_value().double_value)
        det_topic = self.declare_parameter('detections_topic', '/detections').get_parameter_value().string_value
        cam_info_topic = self.declare_parameter('camera_info_topic', '/camera_info').get_parameter_value().string_value

        qos_det = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_cam = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)

        self.br = TransformBroadcaster(self)
        self.K = None
        self.D = None
        self.sub_cam = self.create_subscription(CameraInfo, cam_info_topic, self.on_caminfo, qos_cam)
        self.sub_det = self.create_subscription(AprilTagDetectionArray, det_topic, self.on_dets, qos_det)
        self.get_logger().info(
            f"PnP broadcaster listening det={det_topic}, cam={cam_info_topic}, tag_size={self.tag_size} m"
        )

        s = self.tag_size / 2.0
        self.objp = np.array([[-s, -s, 0], [s, -s, 0], [s, s, 0], [-s, s, 0]], dtype=np.float32)

    def on_caminfo(self, msg: CameraInfo):
        if len(msg.k) == 9:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64).reshape(-1) if msg.d else np.zeros(5)
        else:
            self.K = None

    def on_dets(self, msg: AprilTagDetectionArray):
        if self.K is None:
            return
        for det in msg.detections:
            tid = int(det.id[0]) if hasattr(det, 'id') and hasattr(det.id, '__len__') else int(det.id) if hasattr(det, 'id') else 0
            if not hasattr(det, 'corners') or len(det.corners) < 4:
                continue
            img_pts = np.array([
                [det.corners[0].x, det.corners[0].y],
                [det.corners[1].x, det.corners[1].y],
                [det.corners[2].x, det.corners[2].y],
                [det.corners[3].x, det.corners[3].y],
            ], dtype=np.float32)

            ok, rvec, tvec = cv2.solvePnP(self.objp, img_pts, self.K, self.D, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                self.get_logger().warn("solvePnP failed")
                continue

            t = TransformStamped()
            # Use the detection message timestamp (already in sim time via camera bridge).
            # Do NOT use self.get_clock().now() — it returns zero until /clock syncs.
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.camera_frame
            t.child_frame_id = f"{self.tag_prefix}:{tid}"
            t.transform.translation.x = float(tvec[0])
            t.transform.translation.y = float(tvec[1])
            t.transform.translation.z = float(tvec[2])
            t.transform.rotation = quat_from_rvec(rvec)
            self.br.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(TagPnPBroadcaster())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
