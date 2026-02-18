#!/usr/bin/env python3
"""Sim vision stack: Gazebo world + camera bridge + AprilTag + PnP broadcaster.

This launch file intentionally excludes SITL, MAVROS, and the controller so you
can iterate on those separately.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use /clock for sim time'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('tag_hover_sim'),
            'worlds',
            'apriltag_test.sdf'
        ]),
        description='Path to the Gazebo world file'
    )

    apriltag_params_arg = DeclareLaunchArgument(
        'apriltag_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('tag_hover_sim'),
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

    gz_image_topic = '/world/apriltag_test/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/image'
    gz_camera_info_topic = '/world/apriltag_test/model/iris_with_rgb_camera/model/gimbal/link/pitch_link/sensor/camera/camera_info'
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

    pnp_broadcaster = Node(
        package='tag_hover_sim',
        executable='apriltag_pnp_broadcaster',
        output='screen',
        parameters=[
            {'camera_frame': LaunchConfiguration('camera_frame')},
            {'detections_topic': LaunchConfiguration('detections_topic')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        apriltag_params_arg,
        camera_frame_arg,
        detections_topic_arg,
        gz_sim,
        camera_bridge,
        apriltag_node,
        pnp_broadcaster,
    ])
