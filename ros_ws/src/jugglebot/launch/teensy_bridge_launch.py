"""Standalone launch for the can-bridge Teensy node.

This is a convenience launch for bench bring-up of the bridge in isolation.
The bridge is now folded into ``jugglebot_launch.py`` (the cutover is complete),
so do NOT run this *alongside* the main launch — the node owns the production
leg/hand/BB/cone topic + service names (the leg/hand cutover promoted them off
the old side-by-side ``/teensy/*`` namespace), so a second instance would create
duplicate publishers on those names. Use this only when the main launch is down.

Safety: the bridge ALWAYS starts DISARMED (``mpc_active=0``). Arming is
runtime-only via ``/set_setpoint_output`` (SetBool), whose stream-then-arm
pre-check refuses to arm without a live, seeded :5557 stream — see
``ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md`` (A1). The old
``enable_setpoint_output:=true`` boot-arm is INERT since 2026-07-15: it armed
with zero preconditions before anything could stream, so the firmware's
MPC-staleness watchdog latched within one guard tick (the arm-before-stream
trap). The bridge logs an ERROR if the arg is set.

Usage::

    ros2 launch jugglebot teensy_bridge_launch.py
    ros2 launch jugglebot teensy_bridge_launch.py teensy_ip:=192.168.42.2
    # Arm at runtime, once a producer is streaming on :5557:
    ros2 service call /set_setpoint_output std_srvs/srv/SetBool "{data: true}"
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
            description='DEPRECATED + INERT (arming contract, 2026-07-15): '
                        'the bridge always starts disarmed and logs an ERROR '
                        'if this is true. Arm at runtime via '
                        '/set_setpoint_output.'),
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
