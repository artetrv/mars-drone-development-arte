#!/usr/bin/env python3
"""
Hardware controller launch (OAK edition) — hover_guided_hold preconfigured
for the Luxonis OAK camera.

Terminal 7 of the hardware setup. Launch ONLY after:
  - hardware_vision_stack_oak.launch.py is running (Terminal 3)
  - relative_vibration_pose is running (Terminal 6)
  - the drone is airborne and in GUIDED mode (via MAVProxy, Terminal 1)

Usage:
  ros2 launch tag_hover_two_tags hover_controller_oak.launch.py

Override anything if needed:
  ros2 launch tag_hover_two_tags hover_controller_oak.launch.py \
    camera_frame:=oak_rgb_camera_optical_frame target_distance:=2.0

IMPORTANT: camera_frame here MUST be identical to the camera_frame used in
hardware_vision_stack_oak.launch.py, or the controller will never find the
tag TFs and will spin in SEARCH forever.

To end a run: Ctrl-C here (publishes False on /measurement_hold_active),
then `mode land` in MAVProxy.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='oak_rgb_camera_optical_frame',
        description=(
            'OAK optical frame — must match hardware_vision_stack_oak.launch.py. '
            'Verify: ros2 topic echo /oak/rgb/camera_info --once | grep frame_id'
        ),
    )

    target_distance_arg = DeclareLaunchArgument(
        'target_distance',
        default_value='1.5',
        description='Desired hold distance from the reference tag (m)',
    )

    rate_hz_arg = DeclareLaunchArgument(
        'rate_hz',
        default_value='10.0',
        description='Controller tick rate (Hz)',
    )

    ref_tag_id_arg = DeclareLaunchArgument(
        'ref_tag_id',
        default_value='0',
        description='Stationary reference tag ID (must match vision stack)',
    )

    vib_tag_id_arg = DeclareLaunchArgument(
        'vib_tag_id',
        default_value='1',
        description='Vibrating structure tag ID (must match vision stack)',
    )

    controller = Node(
        # NOTE: the two-tag hover_guided_hold is registered as an executable
        # under tag_hover_sim (verified with `ros2 pkg executables`); the
        # tag_hover_two_tags package only registers the _se3 variant.
        package='tag_hover_sim',
        executable='hover_guided_hold',
        name='hover_guided_hold',
        output='screen',
        parameters=[{
            'camera_frame':    LaunchConfiguration('camera_frame'),
            'ref_tag_id':      LaunchConfiguration('ref_tag_id'),
            'vib_tag_id':      LaunchConfiguration('vib_tag_id'),
            'target_distance': LaunchConfiguration('target_distance'),
            'rate_hz':         LaunchConfiguration('rate_hz'),
            'use_sim_time':    False,
        }],
    )

    return LaunchDescription([
        camera_frame_arg,
        target_distance_arg,
        rate_hz_arg,
        ref_tag_id_arg,
        vib_tag_id_arg,
        controller,
    ])
