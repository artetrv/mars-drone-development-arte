#!/usr/bin/env python3
"""
Hover controller launch file for tag_hover_two_tags measurement stack.

Uses hover_yaw_search_v1 — full 4-DOF controller (yaw + distance + lateral + vertical)
with automatic SEARCH→LOCK transition when tag is detected.

Sim usage (default camera_frame):
  ros2 launch tag_hover_two_tags hover_controller.launch.py mode:=SEARCH

Hardware usage (D455 RealSense):
  ros2 launch tag_hover_two_tags hover_controller.launch.py \
    camera_frame:=camera_color_optical_frame \
    tag_frame:=tag36h11:0 \
    target_distance:=1.5
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

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='iris_with_rgb_camera/gimbal/pitch_link/camera',
        description='Camera optical frame. Sim default shown; hardware: camera_color_optical_frame'
    )

    tag_frame_arg = DeclareLaunchArgument(
        'tag_frame',
        default_value='tag36h11:0',
        description='AprilTag TF frame to lock on'
    )

    search_yaw_arg = DeclareLaunchArgument(
        'search_yaw',
        default_value='0.25',
        description='Yaw rate in SEARCH mode (rad/s)'
    )

    lock_k_yaw_arg = DeclareLaunchArgument(
        'lock_k_yaw',
        default_value='0.1',
        description='P gain for yaw in LOCK mode'
    )

    lock_k_distance_arg = DeclareLaunchArgument(
        'lock_k_distance',
        default_value='0.2',
        description='P gain for forward/backward (m/s per m error)'
    )

    lock_k_lateral_arg = DeclareLaunchArgument(
        'lock_k_lateral',
        default_value='0.1',
        description='P gain for left/right (m/s per m error)'
    )

    lock_k_vertical_arg = DeclareLaunchArgument(
        'lock_k_vertical',
        default_value='0.1',
        description='P gain for up/down (m/s per m error)'
    )

    target_distance_arg = DeclareLaunchArgument(
        'target_distance',
        default_value='2.0',
        description='Desired standoff distance from tag in meters'
    )

    yaw_align_threshold_arg = DeclareLaunchArgument(
        'yaw_align_threshold',
        default_value='0.1',
        description='Only move forward/lateral when |yaw_error| < this (radians)'
    )

    max_yaw_rate_arg = DeclareLaunchArgument(
        'max_yaw_rate',
        default_value='0.6',
        description='Maximum yaw rate command (rad/s)'
    )

    max_forward_vel_arg = DeclareLaunchArgument(
        'max_forward_vel',
        default_value='0.5',
        description='Maximum forward/backward velocity (m/s)'
    )

    max_lateral_vel_arg = DeclareLaunchArgument(
        'max_lateral_vel',
        default_value='0.5',
        description='Maximum lateral velocity (m/s)'
    )

    mavros_wait_timeout_arg = DeclareLaunchArgument(
        'mavros_wait_timeout',
        default_value='10.0',
        description='Seconds to wait for MAVROS before giving up'
    )

    controller_node = Node(
        package='tag_hover_two_tags',
        executable='hover_yaw_search_v1',
        name='hover_yaw_search',
        output='screen',
        parameters=[
            {'mode':                 LaunchConfiguration('mode')},
            {'rate_hz':              LaunchConfiguration('rate_hz')},
            {'camera_frame':         LaunchConfiguration('camera_frame')},
            {'tag_frame':            LaunchConfiguration('tag_frame')},
            {'search_yaw':           LaunchConfiguration('search_yaw')},
            {'lock_k_yaw':           LaunchConfiguration('lock_k_yaw')},
            {'lock_k_distance':      LaunchConfiguration('lock_k_distance')},
            {'lock_k_lateral':       LaunchConfiguration('lock_k_lateral')},
            {'lock_k_vertical':      LaunchConfiguration('lock_k_vertical')},
            {'target_distance':      LaunchConfiguration('target_distance')},
            {'yaw_align_threshold':  LaunchConfiguration('yaw_align_threshold')},
            {'max_yaw_rate':         LaunchConfiguration('max_yaw_rate')},
            {'max_forward_vel':      LaunchConfiguration('max_forward_vel')},
            {'max_lateral_vel':      LaunchConfiguration('max_lateral_vel')},
            {'mavros_wait_timeout':  LaunchConfiguration('mavros_wait_timeout')},
        ],
    )

    return LaunchDescription([
        mode_arg,
        rate_hz_arg,
        camera_frame_arg,
        tag_frame_arg,
        search_yaw_arg,
        lock_k_yaw_arg,
        lock_k_distance_arg,
        lock_k_lateral_arg,
        lock_k_vertical_arg,
        target_distance_arg,
        yaw_align_threshold_arg,
        max_yaw_rate_arg,
        max_forward_vel_arg,
        max_lateral_vel_arg,
        mavros_wait_timeout_arg,
        controller_node,
    ])
