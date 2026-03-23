"""Phase 1: Validate MuJoCoPlant and simulation harness.

Tests:
  - Plant loads and returns valid state at home
  - Command z+50mm → platform settles within tolerance
  - Command combined pose → settles within tolerance
  - Reset returns to home
  - Telemetry logger writes correct CSV
"""

import os
import sys
import tempfile

import numpy as np
import pytest

_sim_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)

from plant.mujoco_plant import MuJoCoPlant
from plant.interface import PlantState
from viz.telemetry import TelemetryLogger, StepRecord, record_from_arrays


SETTLE_TIME_S = 2.0
POS_TOL_MM = 1.0
ORI_TOL_DEG = 0.5
CONTROL_DT = 0.02  # 50 Hz


@pytest.fixture(scope='module')
def plant():
    return MuJoCoPlant()


def _settle(plant: MuJoCoPlant, duration: float = SETTLE_TIME_S) -> PlantState:
    """Step the plant until settled and return final state."""
    n_steps = int(duration / CONTROL_DT)
    for _ in range(n_steps):
        plant.step(CONTROL_DT)
    return plant.get_state()


class TestPlantHome:
    """Plant state at home position is correct."""

    def test_home_state(self, plant):
        state = plant.get_state()
        assert state.platform_pos_mm == pytest.approx([0, 0, 0], abs=0.5)
        assert state.platform_rot == pytest.approx([0, 0, 0], abs=0.001)
        assert state.time == pytest.approx(0.0, abs=0.01)

    def test_home_extensions(self, plant):
        state = plant.get_state()
        # STOW-relative convention: extensions = 0 at STOW (motor = 0 rev).
        # init_leg_lengths_mm is now the geometric home length, so IK
        # extensions at home (slide=0) are 0 by construction.
        assert np.all(np.abs(state.leg_extensions_mm) < 0.01)


class TestPlantCommand:
    """Command poses and verify the platform reaches them."""

    def test_z_plus_50(self, plant):
        plant.reset()
        target = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        plant.command(plant.pose_to_extensions(target))

        state = _settle(plant)
        pos_err = np.linalg.norm(state.platform_pos_mm - target[:3])
        ori_err = np.degrees(np.linalg.norm(state.platform_rot - target[3:]))

        print(f"z+50: pos_err={pos_err:.4f} mm, ori_err={ori_err:.4f} deg")
        assert pos_err < POS_TOL_MM, f"Position error {pos_err:.3f} mm"
        assert ori_err < ORI_TOL_DEG, f"Orientation error {ori_err:.3f} deg"

    def test_combined_pose(self, plant):
        plant.reset()
        target = np.array([30.0, -20.0, 80.0, 0.05, -0.03, 0.0])
        plant.command(plant.pose_to_extensions(target))

        state = _settle(plant)
        pos_err = np.linalg.norm(state.platform_pos_mm - target[:3])
        ori_err = np.degrees(np.linalg.norm(state.platform_rot - target[3:]))

        print(f"combined: pos_err={pos_err:.4f} mm, ori_err={ori_err:.4f} deg")
        assert pos_err < POS_TOL_MM, f"Position error {pos_err:.3f} mm"
        assert ori_err < ORI_TOL_DEG, f"Orientation error {ori_err:.3f} deg"


class TestPlantReset:
    """Reset returns the platform to home."""

    def test_reset_to_home(self, plant):
        # First move away from home
        plant.command(plant.pose_to_extensions(np.array([0, 0, 50.0, 0, 0, 0])))
        _settle(plant)

        # Now reset
        plant.reset()
        state = plant.get_state()
        assert state.platform_pos_mm == pytest.approx([0, 0, 0], abs=0.5)

    def test_reset_to_pose(self, plant):
        target = np.array([0.0, 0.0, 40.0, 0.0, 0.0, 0.0])
        plant.reset(target)
        # After reset with a pose, actuators are commanded but need settling
        state = _settle(plant)
        pos_err = np.linalg.norm(state.platform_pos_mm - target[:3])
        assert pos_err < POS_TOL_MM


class TestTelemetryLogger:
    """Telemetry CSV output is correct."""

    def test_write_csv(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, 'test.csv')
            logger = TelemetryLogger(path)

            rec = record_from_arrays(
                time=0.5,
                ref_pose=np.array([0, 0, 50, 0, 0, 0], dtype=float),
                ref_twist=np.zeros(6),
                actual_pose=np.array([0.1, 0.2, 49.8, 0.001, 0.0, 0.0]),
                actual_twist=np.zeros(6),
                cmd_extensions=np.ones(6) * 70.0,
                actual_extensions=np.ones(6) * 69.5,
            )
            logger.append(rec)
            logger.flush()

            assert os.path.exists(path)
            with open(path) as f:
                lines = f.readlines()
            assert len(lines) == 2  # header + 1 data row
            assert 'time' in lines[0]
            assert 'tracking_error_mm' in lines[0]
