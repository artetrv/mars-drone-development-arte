#!/usr/bin/env python3
"""
Hardware vision stack — D455 RealSense + AprilTag + PnP TF broadcaster + pose selectors.

Hardware equivalent of sim_vision_stack.launch.py.
No Gazebo, no camera bridge, no oscillator.

Terminal 3 of the hardware setup (see docs/QUICK_RUNBOOK.md Part B):
  Terminal 1: MAVProxy (serial → UDP relay)
  Terminal 2: ros2 launch mavros apm.launch fcu_url:=udp://:14555@127.0.0.1:14550
  Terminal 3: this file
  Terminal 6: relative_vibration_pose (after this is running)
  Terminal 7: ros2 run tag_hover_sim hover_yaw_search_v1 ... (after airborne)

Camera frame note:
  RealSense D455 publishes the RGB optical frame as 'camera_color_optical_frame'.
  This must match camera_frame across the PnP broadcaster and the flight controller.
  Verify with: ros2 topic echo /camera/color/camera_info | grep frame_id

Raspberry Pi usage — optional on a Pi 5 (defaults are fine); use if CPU-bound:
  ros2 launch tag_hover_two_tags hardware_vision_stack.launch.py \
    color_width:=640 color_height:=480 color_fps:=10 \
    apriltag_params:=$HOME/harmonic_ws/src/tag_hover_two_tags/config/apriltag_params_pi.yaml

Standard usage:
  ros2 launch tag_hover_two_tags hardware_vision_stack.launch.py
  ros2 launch tag_hover_two_tags hardware_vision_stack.launch.py ref_tag_id:=0 vib_tag_id:=1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    apriltag_params_arg = DeclareLaunchArgument(
        'apriltag_params',
        default_value=os.path.join(
            get_package_share_directory('tag_hover_two_tags'),
            'config',
            'apriltag_params.yaml',
        ),
        description=(
            'AprilTag detector params file. Optional: apriltag_params_pi.yaml '
            '(quad_decimate=2) lowers CPU load if needed; defaults are fine on a Pi 5.'
        ),
    )

    color_width_arg = DeclareLaunchArgument(
        'color_width',
        default_value='1280',
        description='RealSense color stream width. Pi recommended: 640',
    )

    color_height_arg = DeclareLaunchArgument(
        'color_height',
        default_value='720',
        description='RealSense color stream height. Pi recommended: 480',
    )

    color_fps_arg = DeclareLaunchArgument(
        'color_fps',
        default_value='30',
        description='RealSense color stream FPS. Pi recommended: 10',
    )

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_color_optical_frame',
        description=(
            'Camera optical frame published by RealSense. '
            'Check with: ros2 topic echo /camera/color/camera_info | grep frame_id'
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

    # ── D455 RealSense camera ─────────────────────────────────────────────────
    # Publishes:
    #   /camera/color/image_raw       (RGB image)
    #   /camera/color/camera_info     (intrinsics)
    #   frame_id: camera_color_optical_frame
    #
    # Requires: ros-jazzy-realsense2-camera
    #   sudo apt install ros-jazzy-realsense2-camera
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': False,
            'enable_infra1': False,
            'enable_infra2': False,
            'color_fps':    LaunchConfiguration('color_fps'),
            'color_width':  LaunchConfiguration('color_width'),
            'color_height': LaunchConfiguration('color_height'),
        }],
    )

    # ── AprilTag detector ─────────────────────────────────────────────────────
    # Remapped from RealSense RGB topics to standard apriltag_ros inputs
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        output='screen',
        parameters=[
            LaunchConfiguration('apriltag_params'),
            {'use_sim_time': False},
        ],
        remappings=[
            ('image_rect',  '/camera/color/image_raw'),
            ('camera_info', '/camera/color/camera_info'),
        ],
    )

    # ── PnP TF broadcaster ────────────────────────────────────────────────────
    # Publishes TF: camera_color_optical_frame → tag36h11:0 and tag36h11:1
    # Required by the flight controller (hover_yaw_search_v1 uses TF lookups).
    pnp_broadcaster = Node(
        package='tag_hover_two_tags',
        executable='apriltag_pnp_broadcaster',
        name='apriltag_pnp_broadcaster',
        output='screen',
        parameters=[{
            'camera_frame':      LaunchConfiguration('camera_frame'),
            'tag_size_m':        0.0673,
            'detections_topic':  '/detections',
            'camera_info_topic': '/camera/color/camera_info',
            'use_sim_time':      False,
        }],
    )

    # ── Tag pose selectors ────────────────────────────────────────────────────
    # Extracts per-tag PoseStamped from the detection array.
    # Used by relative_vibration_pose (measurement node).
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
        camera_frame_arg,
        ref_tag_id_arg,
        vib_tag_id_arg,
        detections_topic_arg,
        color_width_arg,
        color_height_arg,
        color_fps_arg,
        realsense_node,
        apriltag_node,
        pnp_broadcaster,
        ref_pose_selector,
        vib_pose_selector,
    ])
