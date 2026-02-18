#!/usr/bin/env python3
import csv
import math
import os
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        return np.eye(3)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def rot_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(R))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

    return (qx, qy, qz, qw)


def rpy_from_rot(R: np.ndarray) -> tuple[float, float, float]:
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.atan2(-R[2, 0], math.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def pose_to_matrix(pose) -> np.ndarray:
    t = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)
    q = pose.orientation
    R = quat_to_rot(q.x, q.y, q.z, q.w)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def matrix_to_pose(T: np.ndarray):
    from geometry_msgs.msg import Pose

    pose = Pose()
    pose.position.x = float(T[0, 3])
    pose.position.y = float(T[1, 3])
    pose.position.z = float(T[2, 3])

    qx, qy, qz, qw = rot_to_quat(T[:3, :3])
    pose.orientation.x = float(qx)
    pose.orientation.y = float(qy)
    pose.orientation.z = float(qz)
    pose.orientation.w = float(qw)
    return pose


def invert_transform(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class RelativeVibrationPose(Node):
    def __init__(self):
        super().__init__('relative_vibration_pose')

        self.ref_topic = self.declare_parameter('ref_pose_topic', '/apriltag_ref/pose').get_parameter_value().string_value
        self.vib_topic = self.declare_parameter('vib_pose_topic', '/apriltag_vib/pose').get_parameter_value().string_value
        self.output_topic = self.declare_parameter('output_topic', '/relative_vibration_pose').get_parameter_value().string_value
        self.reference_frame = self.declare_parameter('reference_frame', '').get_parameter_value().string_value
        self.queue_size = int(self.declare_parameter('sync_queue_size', 20).get_parameter_value().integer_value)
        self.slop = float(self.declare_parameter('sync_slop_sec', 0.05).get_parameter_value().double_value)
        self.csv_dir = self.declare_parameter('csv_dir', '~/harmonic_ws/src/tag_hover_two_tags/logs').get_parameter_value().string_value
        self.csv_basename = self.declare_parameter('csv_basename', 'relative_vibration').get_parameter_value().string_value

        self._pub = self.create_publisher(PoseStamped, self.output_topic, 10)

        self._csv_file = None
        self._csv_writer = None
        self._csv_path = None
        self._init_csv_logging()

        self._ref_sub = Subscriber(self, PoseStamped, self.ref_topic)
        self._vib_sub = Subscriber(self, PoseStamped, self.vib_topic)
        self._sync = ApproximateTimeSynchronizer(
            [self._ref_sub, self._vib_sub],
            queue_size=self.queue_size,
            slop=self.slop,
            allow_headerless=False
        )
        self._sync.registerCallback(self._on_sync)

        self.get_logger().info(
            f"relative_vibration_pose started. ref_topic={self.ref_topic}, vib_topic={self.vib_topic}, "
            f"output_topic={self.output_topic}"
        )
        if self._csv_path:
            self.get_logger().info(f"Logging relative pose to {self._csv_path}")

    def _init_csv_logging(self):
        if not self.csv_dir:
            return

        csv_dir = os.path.expanduser(self.csv_dir)
        os.makedirs(csv_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"{self.csv_basename}_{timestamp}.csv"
        self._csv_path = os.path.join(csv_dir, filename)

        self._csv_file = open(self._csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'stamp_sec',
            'ref_stamp_sec',
            'vib_stamp_sec',
            'x', 'y', 'z',
            'roll', 'pitch', 'yaw'
        ])

    def _on_sync(self, ref_msg: PoseStamped, vib_msg: PoseStamped):
        T_ref_cam = pose_to_matrix(ref_msg.pose)
        T_vib_cam = pose_to_matrix(vib_msg.pose)

        T_cam_ref = invert_transform(T_ref_cam)
        T_vib_ref = T_cam_ref @ T_vib_cam

        out = PoseStamped()
        out.header.stamp = vib_msg.header.stamp
        out.header.frame_id = self.reference_frame or ref_msg.header.frame_id or 'ref_tag'
        out.pose = matrix_to_pose(T_vib_ref)

        self._pub.publish(out)

        if self._csv_writer is not None:
            R = T_vib_ref[:3, :3]
            roll, pitch, yaw = rpy_from_rot(R)
            stamp_sec = stamp_to_sec(out.header.stamp)
            self._csv_writer.writerow([
                f"{stamp_sec:.9f}",
                f"{stamp_to_sec(ref_msg.header.stamp):.9f}",
                f"{stamp_to_sec(vib_msg.header.stamp):.9f}",
                f"{T_vib_ref[0, 3]:.6f}",
                f"{T_vib_ref[1, 3]:.6f}",
                f"{T_vib_ref[2, 3]:.6f}",
                f"{roll:.6f}",
                f"{pitch:.6f}",
                f"{yaw:.6f}",
            ])

    def destroy_node(self):
        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RelativeVibrationPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
