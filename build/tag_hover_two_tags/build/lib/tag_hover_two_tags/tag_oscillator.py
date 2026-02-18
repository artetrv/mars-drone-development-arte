#!/usr/bin/env python3
"""
Oscillates the vibrating AprilTag by publishing sinusoidal position commands.

This node publishes to the joint position controller to create a smooth
side-to-side oscillation for vibration measurement experiments.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math


class TagOscillator(Node):
    def __init__(self):
        super().__init__('tag_oscillator')
        
        # Parameters
        self.declare_parameter('frequency', 1.0)  # Hz
        self.declare_parameter('amplitude', 0.08)  # meters (±8cm)
        self.declare_parameter('update_rate', 50.0)  # Hz
        
        self.frequency = self.get_parameter('frequency').value
        self.amplitude = self.get_parameter('amplitude').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Publisher for joint position commands (Gazebo Harmonic format)
        self.position_pub = self.create_publisher(
            Float64,
            '/model/apriltag_vib_oscillator/joint/oscillator_joint/cmd_pos',
            10
        )
        
        # Timer for publishing
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_position
        )
        
        self.start_time = self.get_clock().now()
        self.publish_count = 0
        
        self.get_logger().info(
            f'Tag oscillator started: freq={self.frequency}Hz, '
            f'amp={self.amplitude}m, rate={self.update_rate}Hz'
        )
    
    def publish_position(self):
        """Publish sinusoidal position command."""
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Sinusoidal position: pos = amplitude * sin(2π * frequency * time)
        position = self.amplitude * math.sin(2.0 * math.pi * self.frequency * elapsed)
        
        msg = Float64()
        msg.data = position
        self.position_pub.publish(msg)
        
        self.publish_count += 1
        if self.publish_count % 50 == 0:  # Log every 50 publishes (1 sec at 50 Hz)
            self.get_logger().debug(
                f'Oscillator publishing: elapsed={elapsed:.3f}s, position={position:.4f}m'
            )


def main(args=None):
    rclpy.init(args=args)
    node = TagOscillator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
