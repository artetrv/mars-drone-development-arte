#!/usr/bin/env python3
"""
Measurement backbone launch file — MAVROS + relative vibration pose node.

Terminal 3 of the 4-terminal practical setup.
Run AFTER sim_vision_stack.launch.py (Terminal 2).
Run BEFORE the hover controller (Terminal 4).

Usage:
  ros2 launch tag_hover_two_tags measurement_backbone.launch.py
  ros2 launch tag_hover_two_tags measurement_backbone.launch.py fcu_url:=udp://:14555@127.0.0.1:14550
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14555@127.0.0.1:14550',
        description='MAVLink FCU URL (SITL listens on 14555, sends to 14550)'
    )

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

    csv_dir_arg = DeclareLaunchArgument(
        'csv_dir',
        default_value='~/.ros/tag_hover_two_tags',
        description='Directory for relative pose CSV logs (empty string disables logging)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use /clock (true for sim, false for hardware)'
    )

    # MAVROS — ArduPilot bridge
    mavros = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'mavros', 'apm.launch',
            ['fcu_url:=', LaunchConfiguration('fcu_url')],
        ],
        output='screen',
    )

    # Relative vibration pose — computes T_vib_ref and logs to CSV
    relative_pose_node = Node(
        package='tag_hover_two_tags',
        executable='relative_vibration_pose',
        name='relative_vibration_pose',
        output='screen',
        parameters=[
            {'ref_pose_topic': LaunchConfiguration('ref_pose_topic')},
            {'vib_pose_topic': LaunchConfiguration('vib_pose_topic')},
            {'csv_dir': LaunchConfiguration('csv_dir')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        fcu_url_arg,
        ref_pose_topic_arg,
        vib_pose_topic_arg,
        csv_dir_arg,
        use_sim_time_arg,
        mavros,
        relative_pose_node,
    ])
