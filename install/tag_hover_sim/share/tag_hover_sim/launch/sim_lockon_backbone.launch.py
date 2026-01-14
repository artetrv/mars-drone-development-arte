#!/usr/bin/env python3
"""
Simulation backbone launch file — minimal parity with real hardware lockon_backbone.launch.py

Real hardware launches only MAVROS (serial) and hover_yaw_search; camera, detector,
and TF broadcaster run separately. Simulation mirrors that workflow, launching only
MAVROS (UDP to SITL) and the controller here.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
  fcu_url_arg = DeclareLaunchArgument(
    'fcu_url',
    default_value='udp://:14555@127.0.0.1:14550',
    description='MAVROS FCU URL (sim: UDP to SITL; real: serial:///dev/ttyAMA0:57600)'
  )

  mavros_name_arg = DeclareLaunchArgument(
    'mavros_name',
    default_value='mavros_lockon',
    description='MAVROS node name (use unique value to avoid clashes)'
  )

  mavros_ns_arg = DeclareLaunchArgument(
    'mavros_ns',
    default_value='',
    description='MAVROS namespace (use unique value to avoid clashes)'
  )

  mavros_prefix_arg = DeclareLaunchArgument(
    'mavros_prefix',
    default_value='/mavros',
    description='Base topic prefix for MAVROS (e.g., /mavros or /mavros_lockon)'
  )

  mode_arg = DeclareLaunchArgument(
    'mode',
    default_value='SEARCH',
    description='Controller mode: SEARCH or LOCK'
  )

  mavros_node = Node(
    package='mavros',
    executable='mavros_node',
    name=LaunchConfiguration('mavros_name'),
    namespace=LaunchConfiguration('mavros_ns'),
    output='screen',
    parameters=[
      {'fcu_url': LaunchConfiguration('fcu_url')},
      {'target_system_id': 1},
      {'target_component_id': 1},
      {'use_sim_time': True},
      {'gcs_url': ''},
    ]
  )

  hover_yaw_search_node = Node(
    #package='tag_hover_controller',
    package='tag_hover_sim',
    executable='hover_yaw_search',
    name='hover_yaw_search',
    output='screen',
    parameters=[
      {'mode': LaunchConfiguration('mode')},
      {'rate_hz': 20.0},                   # control loop rate
      {'search_yaw': 0.25},                # spin speed in SEARCH mode (rad/s)
      {'lock_k_yaw': 0.0025},              # P gain for yaw lock
      {'camera_frame': 'camera'},          # camera frame name
      {'tag_frame': 'tag36h11:0'},         # tag frame name
      {'max_yaw_rate': 0.6},               # max yaw rate (rad/s)
      {'mavros_wait_timeout': 10.0},       # wait up to 10s for MAVROS
      {'mavros_prefix': LaunchConfiguration('mavros_prefix')},
      {'use_sim_time': True}
    ]
  )

  return LaunchDescription([
    fcu_url_arg,
    mavros_name_arg,
    mavros_ns_arg,
    mavros_prefix_arg,
    mode_arg,
    mavros_node,
    hover_yaw_search_node,
  ])
