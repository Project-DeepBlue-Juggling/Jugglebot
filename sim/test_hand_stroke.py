"""Quick visual test: move the hand through its full stroke.

Usage:
    python sim/test_hand_stroke.py
"""

import os
import sys
import time
import math

# Single path bootstrap (repo root, ros_ws pkg, config/generated);
# see sim/_paths.py.  Runnable entry scripts only — library modules under
# sim/ never touch sys.path.
_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
from sim._paths import bootstrap_paths  # noqa: E402
bootstrap_paths()

import mujoco.viewer
from sim.plant.mujoco_plant import MuJoCoPlant

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
