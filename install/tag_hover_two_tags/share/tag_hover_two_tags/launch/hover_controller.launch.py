#!/usr/bin/env python3
"""
Hover controller launch file for tag_hover_two_tags measurement stack.

Usage (with vision stack + relative pose):
  ros2 launch tag_hover_two_tags hover_controller.launch.py mode:=SEARCH

Override parameters:
  ros2 launch tag_hover_two_tags hover_controller.launch.py \
    mode:=LOCK \
    lock_k_yaw:=0.15 \
    camera_frame:=iris_with_rgb_camera/gimbal/pitch_link/camera \
    tag_frame:=tag36h11:1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='SEARCH',
        description='Controller mode: SEARCH or LOCK'
    )

    rate_hz_arg = DeclareLaunchArgument(
        'rate_hz',
        default_value='20.0',
        description='Control loop frequency in Hz'
    )

    search_yaw_arg = DeclareLaunchArgument(
        'search_yaw',
        default_value='0.25',
        description='Yaw rate in SEARCH mode (rad/s)'
    )

    lock_k_yaw_arg = DeclareLaunchArgument(
        'lock_k_yaw',
        default_value='1.0',
        description='P gain for yaw in LOCK mode'
    )

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera',
        description='Camera frame name for TF lookups'
    )

    tag_frame_arg = DeclareLaunchArgument(
        'tag_frame',
        default_value='tag36h11:0',
        description='AprilTag frame name for TF lookups'
    )

    max_yaw_rate_arg = DeclareLaunchArgument(
        'max_yaw_rate',
        default_value='0.6',
        description='Maximum yaw rate command (rad/s)'
    )

    controller_node = Node(
        package='tag_hover_two_tags',
        executable='hover_yaw_search',
        name='hover_yaw_search',
        output='screen',
        parameters=[
            {'mode': LaunchConfiguration('mode')},
            {'rate_hz': LaunchConfiguration('rate_hz')},
            {'search_yaw': LaunchConfiguration('search_yaw')},
            {'lock_k_yaw': LaunchConfiguration('lock_k_yaw')},
            {'camera_frame': LaunchConfiguration('camera_frame')},
            {'tag_frame': LaunchConfiguration('tag_frame')},
            {'max_yaw_rate': LaunchConfiguration('max_yaw_rate')},
        ],
    )

    return LaunchDescription([
        mode_arg,
        rate_hz_arg,
        search_yaw_arg,
        lock_k_yaw_arg,
        camera_frame_arg,
        tag_frame_arg,
        max_yaw_rate_arg,
        controller_node,
    ])
