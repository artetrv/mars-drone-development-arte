from setuptools import setup
import os
from glob import glob

package_name = 'tag_hover_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Install world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mars Drone Development',
    maintainer_email='todo@example.com',
    description='Simulation integration package for AprilTag hover/yaw-search stack in Gazebo Harmonic + ArduPilot SITL',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hover_yaw_search = tag_hover_sim.hover_yaw_search:main',
            'hover_yaw_search_v1 = tag_hover_sim.hover_yaw_search_v1:main',
            'hover_yaw_search_v1_orig = tag_hover_sim.hover_yaw_search_v1_orig:main',
            'hover_two_tags = tag_hover_sim.hover_two_tags_controller:main',
            'hover_single_tag = tag_hover_sim.hover_single_tag_controller:main',
            'hover_yaw_search_v2 = tag_hover_sim.hover_yaw_search_v2:main',
            'hover_yaw_search_sensor_lock = tag_hover_sim.hover_yaw_search_sensor_lock:main',
            'apriltag_tf_broadcaster = tag_hover_sim.apriltag_tf_broadcaster:main',
            'apriltag_pnp_broadcaster = tag_hover_sim.apriltag_pnp_broadcaster:main',
            'hover_guided_hold = tag_hover_sim.hover_guided_hold:main'
        ],
    },
)
