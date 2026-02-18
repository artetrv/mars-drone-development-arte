#!/usr/bin/env python3
"""
Measurement backbone launch file — relative pose estimator for two-tag pipeline.

Camera, detector, and tag pose selectors run separately via sim_vision_stack.launch.py.
This launch starts the relative pose fusion + logging node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
  ref_pose_topic_arg = DeclareLaunchArgument(
    'ref_pose_topic',
    default_value='/apriltag_ref/pose',
    description='PoseStamped topic for the reference tag'
  )

  vib_pose_topic_arg = DeclareLaunchArgument(
    'vib_pose_topic',
    default_value='/apriltag_vib/pose',
    description='PoseStamped topic for the vibrating tag'
  )

  output_topic_arg = DeclareLaunchArgument(
    'output_topic',
    default_value='/relative_vibration_pose',
    description='Relative pose output topic'
  )

  csv_dir_arg = DeclareLaunchArgument(
    'csv_dir',
    default_value='~/.ros/tag_hover_two_tags',
    description='Directory for CSV logs (empty disables logging)'
  )

  relative_pose_node = Node(
    package='tag_hover_two_tags',
    executable='relative_vibration_pose',
    name='relative_vibration_pose',
    output='screen',
    parameters=[
      {'ref_pose_topic': LaunchConfiguration('ref_pose_topic')},
      {'vib_pose_topic': LaunchConfiguration('vib_pose_topic')},
      {'output_topic': LaunchConfiguration('output_topic')},
      {'csv_dir': LaunchConfiguration('csv_dir')},
      {'use_sim_time': True}
    ]
  )

  return LaunchDescription([
    ref_pose_topic_arg,
    vib_pose_topic_arg,
    output_topic_arg,
    csv_dir_arg,
    relative_pose_node,
  ])
