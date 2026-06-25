"""Standalone launch for the can-bridge Teensy node.

This is a convenience launch for bench bring-up of the bridge in isolation.
The bridge is now folded into ``jugglebot_launch.py`` (the cutover is complete),
so do NOT run this *alongside* the main launch — the node owns the production
leg/hand/BB/cone topic + service names (Phase 11 / U4 promoted the legs/hand off
the old side-by-side ``/teensy/*`` namespace), so a second instance would create
duplicate publishers on those names. Use this only when the main launch is down.

Safety: setpoint output is OFF by default (``enable_setpoint_output:=false``).
The bridge sends ``mpc_active=0`` to the Teensy until an operator explicitly
flips the parameter AFTER bench validation. Do not set it true here.

Usage::

    ros2 launch jugglebot teensy_bridge_launch.py
    ros2 launch jugglebot teensy_bridge_launch.py teensy_ip:=192.168.42.2
    # Only after bench validation with motors powered:
    ros2 launch jugglebot teensy_bridge_launch.py enable_setpoint_output:=true
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# teensy_bridge_node imports from controller.teensy_link (the can-bridge UDP
# transport layer), which lives at the repo root — OUTSIDE the ROS install
# tree. When ros2 launch starts the entry-point script from install/, PYTHONPATH
# knows nothing about that path. The Python tests work because conftest.py
# inserts the repo root into sys.path itself; ros2 launch needs an env var.
#
# Override with JUGGLEBOT_REPO if running on a host where the repo lives
# somewhere else. Long-term fix: install controller/teensy_link/ as part of
# the jugglebot ROS package (deferred — separate cleanup commit).
_JUGGLEBOT_REPO = os.environ.get('JUGGLEBOT_REPO', '/home/jetson/Desktop/Jugglebot')


def generate_launch_description():
    teensy_ip = LaunchConfiguration('teensy_ip')
    enable_setpoint_output = LaunchConfiguration('enable_setpoint_output')

    # Prepend the repo root so `from controller.teensy_link import ...` resolves.
    existing_pp = os.environ.get('PYTHONPATH', '')
    pythonpath = (f"{_JUGGLEBOT_REPO}:{existing_pp}"
                  if existing_pp else _JUGGLEBOT_REPO)

    return LaunchDescription([
        DeclareLaunchArgument(
            'teensy_ip', default_value='192.168.42.2',
            description='Static IP of the can-bridge Teensy (ADR-0007).'),
        DeclareLaunchArgument(
            'enable_setpoint_output', default_value='false',
            description='SAFETY: gate for the 40 Hz setpoint downlink + '
                        'mpc_active heartbeat flag. Keep false until '
                        'bench-validated with motors powered.'),
        Node(
            package='jugglebot',
            executable='teensy_bridge_node',
            name='teensy_bridge_node',
            output='screen',
            parameters=[{
                'teensy_ip': teensy_ip,
                'enable_setpoint_output': enable_setpoint_output,
            }],
            additional_env={'PYTHONPATH': pythonpath},
        ),
    ])
