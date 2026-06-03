"""Standalone launch for the Phase-10b can-bridge Teensy node.

This is **deliberately separate** from ``jugglebot_launch.py``: during the
side-by-side migration the can-bridge bridge runs alongside the production
``can_node`` and must be started **manually** by the operator, never as part of
the default robot bring-up. It owns only ``/teensy/*`` topics, so it cannot
interfere with the production control path.

Safety: setpoint output is OFF by default (``enable_setpoint_output:=false``).
The bridge sends ``mpc_active=0`` to the Teensy until an operator explicitly
flips the parameter AFTER bench validation. Do not set it true here.

Usage::

    ros2 launch jugglebot teensy_bridge_launch.py
    ros2 launch jugglebot teensy_bridge_launch.py teensy_ip:=192.168.42.2
    # Only after bench validation with motors powered:
    ros2 launch jugglebot teensy_bridge_launch.py enable_setpoint_output:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    teensy_ip = LaunchConfiguration('teensy_ip')
    enable_setpoint_output = LaunchConfiguration('enable_setpoint_output')

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
        ),
    ])
