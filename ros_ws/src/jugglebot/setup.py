from setuptools import setup
import os
from glob import glob

package_name = 'jugglebot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', glob('srv/*.srv')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'resources'), glob('resources/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harrison low',
    maintainer_email='harrisonlow.jugglebot@gmail.com',
    description="v1 of (proper) Jugglebot code. Now with a state machine!",
    license='MIT Licence',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yasmin_state_machine = jugglebot.yasmin_state_machine:main',
            'can_interface_node = jugglebot.can_interface_node:main',
            'spacemouse_handler = jugglebot.spacemouse_handler:main',
            'sp_ik = jugglebot.sp_ik:main',
            'robot_geometry = jugglebot.robot_geometry:main',
            'level_platform_node = jugglebot.level_platform_node:main',
            'mocap_interface_node = jugglebot.mocap_interface_node:main',
            'ball_prediction_node = jugglebot.ball_prediction_node:main',
            'catch_thrown_ball_node = jugglebot.catch_thrown_ball_node:main',
            'catch_dropped_ball_node = jugglebot.catch_dropped_ball_node:main',
            'mocap_visualizer_node = jugglebot.mocap_visualizer_node:main',
            'landing_analysis_node = jugglebot.landing_analysis_node:main',
            'hand_testing_node = jugglebot.hand_testing_node:main',
            'calibrate_platform_node = jugglebot.calibrate_platform_node:main',
            'pose_correction_node = jugglebot.pose_correction_node:main',
        ],
    },
)

