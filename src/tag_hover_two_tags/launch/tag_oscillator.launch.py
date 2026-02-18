#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tag_hover_two_tags',
            executable='tag_oscillator',
            name='tag_oscillator',
            output='screen',
        ),
    ])
