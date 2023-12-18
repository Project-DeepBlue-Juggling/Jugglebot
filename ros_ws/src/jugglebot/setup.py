from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'jugglebot'

setup(
    name=package_name,
    version='0.0.0',
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
    description="For all things Jugglebot! Will probably be expanded into multiple packages in the future, but I don't really know what I'm doing so I'm keeping it simple :)",
    license='MIT Licence',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sp_ik = jugglebot.sp_ik:main',
            'spacemouse_handler = jugglebot.spacemouse_handler:main',
            'robot_geometry = jugglebot.robot_geometry:main',
            'platform_plotter = jugglebot.platform_plotter:main',
            'can_bus_handler_node = jugglebot.can_bus_handler_node:main',
        ],
    },
)

