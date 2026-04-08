#!/usr/bin/env python3
"""Sim vision stack: Gazebo world + camera bridge + AprilTag + PnP broadcaster.

This launch file intentionally excludes SITL, MAVROS, and the controller so you
can iterate on those separately.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os


def generate_launch_description():
    # Get the models directory for this package
    models_dir = PathJoinSubstitution([
        FindPackageShare('tag_hover_two_tags'),
        'models'
    ])
    
    # Set Gazebo resource path to find our custom AprilTag models
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_dir, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use /clock for sim time'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('tag_hover_two_tags'),
            'worlds',
            'apriltag_two_tags.sdf'
        ]),
        description='Path to the Gazebo world file'
    )

    apriltag_params_arg = DeclareLaunchArgument(
        'apriltag_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('tag_hover_two_tags'),
            'config',
            'apriltag_params.yaml'
        ]),
        description='AprilTag detector params file'
    )

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='iris_with_rgb_camera/gimbal/pitch_link/camera',
        description='Camera frame used by the TF broadcaster'
    )

    detections_topic_arg = DeclareLaunchArgument(
        'detections_topic',
        default_value='/detections',
        description='AprilTag detections topic'
    )

    ref_tag_id_arg = DeclareLaunchArgument(
        'ref_tag_id',
        default_value='0',
        description='AprilTag ID for the reference tag'
    )

    vib_tag_id_arg = DeclareLaunchArgument(
        'vib_tag_id',
        default_value='1',
        description='AprilTag ID for the vibrating tag'
    )

    oscillation_freq_arg = DeclareLaunchArgument(
        'oscillation_freq',
        default_value='1.0',
        description='Oscillation frequency in Hz'
    )

    oscillation_amp_arg = DeclareLaunchArgument(
        'oscillation_amp',
        default_value='0.08',
        description='Oscillation amplitude in meters'
    )

    gz_image_topic = '/world/apriltag_two_tags/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/image'
    gz_camera_info_topic = '/world/apriltag_two_tags/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/camera_info'
    ros_image_topic = '/camera/image_raw'
    ros_camera_info_topic = '/camera/camera_info'

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', LaunchConfiguration('world')],
        output='screen'
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            f'{gz_image_topic}@sensor_msgs/msg/Image@gz.msgs.Image',
            f'{gz_camera_info_topic}@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args',
            '-r', f'{gz_image_topic}:={ros_image_topic}',
            '-r', f'{gz_camera_info_topic}:={ros_camera_info_topic}',
        ],
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        output='screen',
        parameters=[
            LaunchConfiguration('apriltag_params'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('image_rect', ros_image_topic),
            ('camera_info', ros_camera_info_topic),
        ],
    )

    ref_pose_selector = Node(
        package='tag_hover_two_tags',
        executable='tag_pose_selector',
        name='tag_pose_selector_ref',
        output='screen',
        parameters=[
            {'tag_id': LaunchConfiguration('ref_tag_id')},
            {'detections_topic': LaunchConfiguration('detections_topic')},
            {'output_topic': '/apriltag_ref/pose'},
            {'camera_frame': LaunchConfiguration('camera_frame')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    vib_pose_selector = Node(
        package='tag_hover_two_tags',
        executable='tag_pose_selector',
        name='tag_pose_selector_vib',
        output='screen',
        parameters=[
            {'tag_id': LaunchConfiguration('vib_tag_id')},
            {'detections_topic': LaunchConfiguration('detections_topic')},
            {'output_topic': '/apriltag_vib/pose'},
            {'camera_frame': LaunchConfiguration('camera_frame')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    tag_oscillator = Node(
        package='tag_hover_two_tags',
        executable='tag_oscillator',
        name='tag_oscillator',
        output='screen',
        parameters=[
            {'frequency': LaunchConfiguration('oscillation_freq')},
            {'amplitude': LaunchConfiguration('oscillation_amp')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # PnP broadcaster — publishes TF for all detected tags (required by flight controller)
    # Publishes: camera_frame → tag36h11:0 (ref) and camera_frame → tag36h11:1 (vib)
    pnp_broadcaster = Node(
        package='tag_hover_two_tags',
        executable='apriltag_pnp_broadcaster',
        name='apriltag_pnp_broadcaster',
        output='screen',
        parameters=[
            {'camera_frame': LaunchConfiguration('camera_frame')},
            {'tag_size_m': 0.127},
            {'detections_topic': LaunchConfiguration('detections_topic')},
            {'camera_info_topic': '/camera/camera_info'},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        set_gz_resource_path,
        use_sim_time_arg,
        world_arg,
        apriltag_params_arg,
        camera_frame_arg,
        detections_topic_arg,
        ref_tag_id_arg,
        vib_tag_id_arg,
        oscillation_freq_arg,
        oscillation_amp_arg,
        gz_sim,
        camera_bridge,
        apriltag_node,
        pnp_broadcaster,
        ref_pose_selector,
        vib_pose_selector,
        tag_oscillator,
    ])
