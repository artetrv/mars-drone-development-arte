#!/usr/bin/env python3
"""
Hardware vision stack (OAK edition) — Luxonis OAK (depthai) + AprilTag +
PnP TF broadcaster + pose selectors.

OAK replacement for hardware_vision_stack.launch.py (D455 version).
Terminal 3 of the hardware setup. Everything downstream of the camera
(detector, PnP, selectors, measurement, controller) is unchanged.

BEFORE FIRST USE — verify the OAK's actual topic and frame names:
  1. ros2 launch depthai_ros_driver camera.launch.py        # v2 driver
       (or: ros2 launch depthai_ros_driver_v3 driver.launch.py  # v3 driver)
  2. ros2 topic list | grep -i oak
  3. ros2 topic echo /oak/rgb/camera_info --once | grep frame_id
  4. If they differ from the defaults below, override via launch args:
       ros2 launch tag_hover_two_tags hardware_vision_stack_oak.launch.py \
         oak_image_topic:=/oak/rgb/image_raw \
         oak_info_topic:=/oak/rgb/camera_info \
         camera_frame:=oak_rgb_camera_optical_frame

Raspberry Pi 5 usage (Pi-optimised AprilTag params):
  ros2 launch tag_hover_two_tags hardware_vision_stack_oak.launch.py \
    apriltag_params:=$HOME/harmonic_ws/src/tag_hover_two_tags/config/apriltag_params_pi.yaml

REMINDER for Terminal 7 (controller) — camera_frame must match this file:
  ros2 run tag_hover_sim hover_guided_hold --ros-args \
    -p camera_frame:=oak_rgb_camera_optical_frame \
    -p target_distance:=1.5 -p rate_hz:=10.0 -p use_sim_time:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Launch arguments ──────────────────────────────────────────────────
    apriltag_params_arg = DeclareLaunchArgument(
        'apriltag_params',
        default_value=os.path.join(
            get_package_share_directory('tag_hover_two_tags'),
            'config',
            'apriltag_params.yaml',
        ),
        description=(
            'AprilTag detector params file. '
            'Pi: use apriltag_params_pi.yaml (quad_decimate=2 for lower CPU load).'
        ),
    )

    # OAK topic / frame names — defaults are the depthai_ros_driver v2
    # conventions. VERIFY against `ros2 topic list` on your setup and
    # override if needed (v3 driver may differ).
    oak_image_topic_arg = DeclareLaunchArgument(
        'oak_image_topic',
        default_value='/oak/rgb/image_raw',
        description='OAK RGB image topic. Verify with: ros2 topic list | grep -i oak',
    )

    oak_info_topic_arg = DeclareLaunchArgument(
        'oak_info_topic',
        default_value='/oak/rgb/camera_info',
        description='OAK RGB camera_info topic (intrinsics).',
    )

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='oak_rgb_camera_optical_frame',
        description=(
            'Optical frame published by the OAK driver. '
            'Check with: ros2 topic echo /oak/rgb/camera_info --once | grep frame_id. '
            'MUST match the -p camera_frame:=... passed to hover_guided_hold.'
        ),
    )

    oak_params_file_arg = DeclareLaunchArgument(
        'oak_params_file',
        default_value='',
        description=(
            'Optional depthai driver params yaml (e.g. to lower RGB resolution/fps '
            'for the Pi). Empty = driver defaults.'
        ),
    )

    ref_tag_id_arg = DeclareLaunchArgument(
        'ref_tag_id',
        default_value='0',
        description='AprilTag ID for the stationary reference tag',
    )

    vib_tag_id_arg = DeclareLaunchArgument(
        'vib_tag_id',
        default_value='1',
        description='AprilTag ID for the vibrating structure tag',
    )

    detections_topic_arg = DeclareLaunchArgument(
        'detections_topic',
        default_value='/detections',
        description='AprilTag detections topic',
    )

    # ── OAK camera (depthai_ros_driver) ──────────────────────────────────
    # Publishes (v2 driver defaults):
    #   /oak/rgb/image_raw       (RGB image)
    #   /oak/rgb/camera_info     (factory-calibrated intrinsics)
    #   frame_id: oak_rgb_camera_optical_frame
    #
    # Requires: ros-jazzy-depthai-ros-driver
    #   sudo apt install ros-jazzy-depthai-ros-driver
    #
    # NOTE: if you installed the v3 driver (depthai_ros_driver_v3), change
    # the package name below and use driver.launch.py instead.
    oak_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('depthai_ros_driver'),
                'launch',
                'camera.launch.py',
            )
        ),
        launch_arguments={
            # Pass a params file only if the user provided one
            'params_file': LaunchConfiguration('oak_params_file'),
        }.items(),
    )

    # ── AprilTag detector ────────────────────────────────────────────────
    # Same detector as the D455 stack — only the input remappings change.
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        output='screen',
        parameters=[
            LaunchConfiguration('apriltag_params'),
            {'use_sim_time': False},
        ],
        remappings=[
            ('image_rect',  LaunchConfiguration('oak_image_topic')),
            ('camera_info', LaunchConfiguration('oak_info_topic')),
        ],
    )

    # ── PnP TF broadcaster ───────────────────────────────────────────────
    # Publishes TF: <camera_frame> → tag36h11:0 and tag36h11:1
    # Required by the flight controller (TF lookups).
    # tag_size_m unchanged — same physical tags as the D455 setup.
    pnp_broadcaster = Node(
        package='tag_hover_two_tags',
        executable='apriltag_pnp_broadcaster',
        name='apriltag_pnp_broadcaster',
        output='screen',
        parameters=[{
            'camera_frame':      LaunchConfiguration('camera_frame'),
            'tag_size_m':        0.0673,
            'detections_topic':  '/detections',
            'camera_info_topic': LaunchConfiguration('oak_info_topic'),
            'use_sim_time':      False,
        }],
    )

    # ── Tag pose selectors ───────────────────────────────────────────────
    # Unchanged from the D455 stack except camera_frame.
    ref_pose_selector = Node(
        package='tag_hover_two_tags',
        executable='tag_pose_selector',
        name='tag_pose_selector_ref',
        output='screen',
        parameters=[{
            'tag_id':           LaunchConfiguration('ref_tag_id'),
            'detections_topic': LaunchConfiguration('detections_topic'),
            'output_topic':     '/apriltag_ref/pose',
            'camera_frame':     LaunchConfiguration('camera_frame'),
            'use_sim_time':     False,
        }],
    )

    vib_pose_selector = Node(
        package='tag_hover_two_tags',
        executable='tag_pose_selector',
        name='tag_pose_selector_vib',
        output='screen',
        parameters=[{
            'tag_id':           LaunchConfiguration('vib_tag_id'),
            'detections_topic': LaunchConfiguration('detections_topic'),
            'output_topic':     '/apriltag_vib/pose',
            'camera_frame':     LaunchConfiguration('camera_frame'),
            'use_sim_time':     False,
        }],
    )

    return LaunchDescription([
        apriltag_params_arg,
        oak_image_topic_arg,
        oak_info_topic_arg,
        camera_frame_arg,
        oak_params_file_arg,
        ref_tag_id_arg,
        vib_tag_id_arg,
        detections_topic_arg,
        oak_camera,
        apriltag_node,
        pnp_broadcaster,
        ref_pose_selector,
        vib_pose_selector,
    ])
