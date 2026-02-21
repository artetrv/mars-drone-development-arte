from setuptools import setup
import os
from glob import glob

package_name = 'tag_hover_two_tags'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'tag_hover_controller', 'tag_hover_sim'],
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
        # Install models
        (os.path.join('share', package_name, 'models'),
            []),
    ] + [
        (os.path.join('share', package_name, 'models', os.path.dirname(f).replace('models/', '')),
         [f]) for f in glob('models/**/*', recursive=True) if os.path.isfile(f)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Javier Becerril',
    maintainer_email='javierbecerril@example.com',
    description='Two-tag AprilTag relative pose measurement stack for Gazebo Harmonic + ArduPilot SITL',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tag_pose_selector = tag_hover_two_tags.tag_pose_selector:main',
            'relative_vibration_pose = tag_hover_two_tags.relative_vibration_pose:main',
            'apriltag_tf_broadcaster = tag_hover_two_tags.apriltag_tf_broadcaster:main',
            'apriltag_pnp_broadcaster = tag_hover_two_tags.apriltag_pnp_broadcaster:main',
            'tag_oscillator = tag_hover_two_tags.tag_oscillator:main',
            'hover_yaw_search = tag_hover_controller.hover_yaw_search:main',
            'tag_overlay = tag_hover_two_tags.tag_overlay:main',
            'video_vibration_analyzer = tag_hover_two_tags.video_vibration_analyzer:main',
        ],
    },
)
