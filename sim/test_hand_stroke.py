"""Quick visual test: move the hand through its full stroke.

Usage:
    python sim/test_hand_stroke.py
"""

import os
import sys
import time
import math

# Make sim/, controller/, the jugglebot ROS2 package, and generated config
# importable when run directly on a fresh clone — without requiring the
# jugglebot package to be pip-installed (as it is on the Jetson venv).
# Mirrors the path entries in tests/conftest.py.
_sim_dir = os.path.dirname(os.path.abspath(__file__))
_repo_root = os.path.dirname(_sim_dir)
for _p in (
    _sim_dir,                                                # bare plant/, hand/, ball_butler/ imports
    _repo_root,                                              # controller.*
    os.path.join(_repo_root, 'ros_ws', 'src', 'jugglebot'),  # jugglebot.motion.* (pure-Python ROS2 pkg)
    os.path.join(_repo_root, 'config', 'generated'),         # generated hardware/protocol config
):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import mujoco.viewer
from plant.mujoco_plant import MuJoCoPlant

CONTROL_DT = 1.0 / 50  # 50 Hz


def main():
    plant = MuJoCoPlant()
    stroke_mm = 355.0
    period = 4.0  # seconds per full cycle

    with mujoco.viewer.launch_passive(plant.model, plant.data) as viewer:
        start = time.monotonic()
        sim_time = 0.0

        while viewer.is_running():
            # Sinusoidal sweep: 0 → full stroke → 0, repeat
            t = sim_time % period
            pos_mm = stroke_mm * 0.5 * (1.0 - math.cos(2.0 * math.pi * t / period))
            plant.command_hand(pos_mm)
            plant.step(CONTROL_DT)
            sim_time += CONTROL_DT

            viewer.sync()

            elapsed = time.monotonic() - start
            sleep_time = sim_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


if __name__ == '__main__':
    main()
