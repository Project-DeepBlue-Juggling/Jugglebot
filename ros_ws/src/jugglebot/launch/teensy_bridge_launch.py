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


# teensy_bridge_node imports from ``teensy_link`` (the can-bridge UDP transport
# layer), a top-level package at the REPO ROOT — OUTSIDE the ROS install tree.
# When ros2 launch starts the entry-point script from install/, PYTHONPATH knows
# nothing about that path. The Python tests work because conftest.py inserts the
# repo root into sys.path itself; ros2 launch needs an env var.
#
# Override with JUGGLEBOT_REPO if running on a host where the repo lives
# somewhere else.
#
# ARCHITECTURE (decided 2026-08-01, refactor-2026-07 Phase 4). Living at the
# repo root and being reached by this injection is the DECIDED end state, not a
# stopgap — the older note here proposed "install teensy_link/ into the
# jugglebot ROS package" as the long-term fix, and that option was considered
# and rejected. Rationale: teensy_link carries the repo's hottest production
# code (protocol.py alone took 16 commits since May) and the injection makes the
# bridge run the LIVE tree, so an edit is live at the next relaunch. Installing
# it would put every wire-format edit behind a `colcon build`, and a forgotten
# build is silent — you get last week's frame layouts against this week's
# firmware with no error, which is precisely the staleness trap Phase 5's
# drift check exists to shout about. The cost accepted in exchange is that the
# bridge runs live-tree teensy_link beside the frozen installed jugglebot.*.
_JUGGLEBOT_REPO = os.environ.get('JUGGLEBOT_REPO', '/home/jetson/Desktop/Jugglebot')


def generate_launch_description():
    teensy_ip = LaunchConfiguration('teensy_ip')
    enable_setpoint_output = LaunchConfiguration('enable_setpoint_output')

    # Prepend the repo root so `from teensy_link import ...` resolves.
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
