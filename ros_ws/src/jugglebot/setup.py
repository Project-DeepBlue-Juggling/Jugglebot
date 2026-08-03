from setuptools import setup
import os
from glob import glob

package_name = 'jugglebot'

# The single source-of-truth hardware config lives at the repo root
# (config/hardware_config.yaml), three levels up from this setup.py
# (jugglebot -> src -> ros_ws -> repo).  motor_guard's friction_ff loader
# reads it at runtime; under the colcon install tree it resolves via the
# ament share dir, so install the YAML into share/jugglebot/config/.
# Referenced by a package-relative path (colcon rejects absolute data_files
# sources) so there is no in-package copy to drift — colcon copies the real
# repo-root file at build time.  NOTE: because this is a build-time copy, a
# friction re-tune of the source YAML only takes effect after `colcon build`
# (or via the JUGGLEBOT_HW_CONFIG override — see friction_ff_params.py).
_HW_CONFIG_YAML = os.path.join('..', '..', '..', 'config', 'hardware_config.yaml')

# The machine-written tilt calibration map (contract C-LEVEL-2,
# ros_ws/docs/levelling_frame.md), installed into the same share/config dir so
# trajectory_node's loader has an ament fallback when it runs from the install
# tree.  CONDITIONAL because this file does not exist until an operator has run
# the Phase-3 acquisition tool: an ament data_files entry naming a missing
# source is a hard `colcon build` failure, which would make a fresh clone
# unbuildable over a calibration that is optional by contract (an absent map is
# a silent, non-gating state).
#
# NOTE the loader resolves SOURCE TREE FIRST and this share copy only as a
# fallback — the exact inverse of hardware_config.yaml above, because the
# acquisition tool rewrites the source file at runtime and reloads it live.  So
# this row exists for the detached/install-tree case.  See motion/tilt_map.py's
# module docstring.
#
# Source-first means a stale copy here can never shadow a fresh CAPTURE (the
# source file exists, so it wins).  It does NOT mean this copy is harmless: it
# shadows an UNINSTALL.  Once any colcon build has run with the source file
# present, moving the source file aside makes the resolver fall through to THIS
# copy — the map stays loaded and the reload reports success.  tilt_cal_grid.py's
# --force-uninstall therefore moves every existing candidate aside, this one
# included, and says so when a map survives.
_TILT_CAL_YAML = os.path.join('..', '..', '..', 'config', 'tilt_calibration.yaml')
_CONFIG_YAMLS = [_HW_CONFIG_YAML]
if os.path.exists(_TILT_CAL_YAML):
    _CONFIG_YAMLS.append(_TILT_CAL_YAML)

setup(
    name=package_name,
    version='1.0.0',
    packages=[
        package_name,
        f'{package_name}.can',
        f'{package_name}.motion',
        f'{package_name}.motion.trajectory',
        f'{package_name}.tracking',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', glob('srv/*.srv')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'resources'), glob('resources/*')),
        (os.path.join('share', package_name, 'config'), _CONFIG_YAMLS),
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
            # ── Core nodes ────────────────────────────────────
            'orchestrator_node = jugglebot.orchestrator_node:main',
            'motion_bridge_node = jugglebot.motion_bridge_node:main',
            # The can-bridge Teensy node (owns the Jetson<->Teensy UDP link). Launched
            # by jugglebot_launch.py in production; also runnable standalone via
            # launch/teensy_bridge_launch.py.
            'teensy_bridge_node = jugglebot.teensy_bridge_node:main',
            # MVP Jetson-side trajectory generator (replaces run_mpc.py's hot path;
            # streams 40 Hz knots on :5557 → teensy_bridge_node → can-hub Teensy).
            'trajectory_node = jugglebot.trajectory_node:main',
            # mpc_bridge_node source retained (MPC return path is dormant, not
            # deleted); dropped from jugglebot_launch.py in the MVP cutover.
            'mpc_bridge_node = jugglebot.mpc_bridge_node:main',
            'mocap_node = jugglebot.mocap_node:main',
            'spacemouse_handler = jugglebot.spacemouse_handler:main',
            # ── Ball tracking & catch ──────────────────────────
            'ball_tracker_node = jugglebot.ball_tracker_node:main',
            'catch_coordinator_node = jugglebot.catch_coordinator_node:main',
            'catch_correlation_node = jugglebot.catch_correlation_node:main',
            'ball_butler_node = jugglebot.ball_butler_node:main',
            # BB→Jugglebot reload action (Phase 7): orchestrates only — trajectory_node
            # plans the catch, catch_coordinator arms the hand, ball_butler throws.
            'reload_coordinator_node = jugglebot.reload_coordinator_node:main',
            # ── Standalone processes (not ROS2 nodes) ─────────
            'motor_guard = jugglebot.motion.motor_guard:main',
        ],
    },
)

