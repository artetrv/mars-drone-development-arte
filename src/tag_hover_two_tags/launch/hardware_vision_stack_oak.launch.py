#!/usr/bin/env python3
"""
Hardware vision stack (OAK edition) — Luxonis OAK (depthai) + AprilTag +
PnP TF broadcaster + pose selectors.

OAK replacement for hardware_vision_stack.launch.py (D455 version).
Everything downstream of the camera (detector, PnP, selectors,
measurement, controller) is unchanged.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    apriltag_params_arg = DeclareLaunchArgument(
        'apriltag_params',
        default_value=os.path.join(
            get_package_share_directory('tag_hover_two_tags'),
            'config',
            'apriltag_params.yaml',
        ),
        description='AprilTag detector params file.',
    )

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
        description='Optical frame published by the OAK driver. MUST match the controller.',
    )

    oak_params_file_arg = DeclareLaunchArgument(
        'oak_params_file',
        default_value='',
        description='Optional depthai driver params yaml. Empty = driver defaults.',
    )

    tag_size_m_arg = DeclareLaunchArgument(
        'tag_size_m',
        default_value='0.0673',
        description='Physical tag size in meters (black square side). MEASURE the printed tags.',
    )

    ref_tag_id_arg = DeclareLaunchArgument(
        'ref_tag_id', default_value='0',
        description='AprilTag ID for the stationary reference tag',
    )

    vib_tag_id_arg = DeclareLaunchArgument(
        'vib_tag_id', default_value='1',
        description='AprilTag ID for the vibrating structure tag',
    )

    detections_topic_arg = DeclareLaunchArgument(
        'detections_topic', default_value='/detections',
        description='AprilTag detections topic',
    )

    _has_params_file = PythonExpression(
        ["'", LaunchConfiguration('oak_params_file'), "' != ''"]
    )

    oak_camera_with_params = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('depthai_ros_driver'),
                'launch', 'camera.launch.py',
            )
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('oak_params_file'),
        }.items(),
        condition=IfCondition(_has_params_file),
    )

    oak_camera_default = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('depthai_ros_driver'),
                'launch', 'camera.launch.py',
            )
        ),
        condition=UnlessCondition(_has_params_file),
    )

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

    pnp_broadcaster = Node(
        package='tag_hover_two_tags',
        executable='apriltag_pnp_broadcaster',
        name='apriltag_pnp_broadcaster',
        output='screen',
        parameters=[{
            'camera_frame':      LaunchConfiguration('camera_frame'),
            'tag_size_m':        ParameterValue(LaunchConfiguration('tag_size_m'), value_type=float),
            'detections_topic':  LaunchConfiguration('detections_topic'),
            'camera_info_topic': LaunchConfiguration('oak_info_topic'),
            # The broadcaster also publishes each tag's PoseStamped
            # (/apriltag_ref/pose, /apriltag_vib/pose) for the measurement
            # pipeline — Jazzy's apriltag detections carry no pose, so the
            # old tag_pose_selector nodes cannot provide these.
            'ref_tag_id':        ParameterValue(LaunchConfiguration('ref_tag_id'), value_type=int),
            'vib_tag_id':        ParameterValue(LaunchConfiguration('vib_tag_id'), value_type=int),
            'use_sim_time':      False,
        }],
    )

    return LaunchDescription([
        apriltag_params_arg,
        oak_image_topic_arg,
        oak_info_topic_arg,
        camera_frame_arg,
        oak_params_file_arg,
        tag_size_m_arg,
        ref_tag_id_arg,
        vib_tag_id_arg,
        detections_topic_arg,
        oak_camera_with_params,
        oak_camera_default,
        apriltag_node,
        pnp_broadcaster,
    ])
