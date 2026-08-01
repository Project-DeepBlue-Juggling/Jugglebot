"""Tests for Ball Butler throw sequences (BB1-BB4).

Validates end-to-end catch pipeline with realistic ball trajectories from
a simulated Ball Butler thrower.  These are integration tests exercising
MPC + MuJoCo + HandCoordinator with realistic inter-throw timing.
"""

from __future__ import annotations

import numpy as np
import pytest

from sim.plant.mujoco_plant import MuJoCoPlant
from controller import MPCController, MPCParams
from sim.hand.coordinator import HandCoordinator
from sim.input.scripted import get_catch_sequence
from helpers import run_catch_sim


@pytest.fixture(scope='module')
def plant():
    return MuJoCoPlant()


@pytest.fixture
def mpc(plant):
    params = MPCParams(
        max_cpu_time=2.0,
        max_iter=500,
        max_leg_vel_mmps=1000.0,
        prime_solver=False,
    )
    return MPCController.from_plant(params, plant)


def _run_bb(plant, mpc, name):
    """Run a BB sequence and return the result dict."""
    plant.reset()
    coordinator = HandCoordinator()
    sequence, duration = get_catch_sequence(name)
    for target, ball in sequence:
        coordinator.submit_target(target, ball)
    return run_catch_sim(plant, mpc, coordinator, duration)


class TestBB1SingleThrow:
    """BB1: Single Ball Butler throw at centre."""

    def test_capture(self, plant, mpc):
        """Ball should be captured."""
        result = _run_bb(plant, mpc, 'BB1')
        assert result['captures'] == 1, \
            f"Expected 1 capture, got {result['captures']}"

    def test_arrival_accuracy(self, plant, mpc):
        """Platform arrives within 5 mm of catch target."""
        result = _run_bb(plant, mpc, 'BB1')
        assert len(result['events']) >= 1
        ev = result['events'][0]
        assert ev.arrival_error_mm < 5.0, \
            f"BB1 arrival error {ev.arrival_error_mm:.1f} mm exceeds 5 mm"


class TestBB3EdgeThrow:
    """BB3: Throw near edge of catch envelope (80mm lateral offset)."""

    def test_capture_at_edge(self, plant, mpc):
        """Ball should be captured at workspace edge."""
        result = _run_bb(plant, mpc, 'BB3')
        assert result['captures'] == 1, \
            f"Expected 1 capture at edge, got {result['captures']}"

    def test_arrival_accuracy(self, plant, mpc):
        """Platform arrives within 8 mm (relaxed for edge)."""
        result = _run_bb(plant, mpc, 'BB3')
        assert len(result['events']) >= 1
        ev = result['events'][0]
        assert ev.arrival_error_mm < 8.0, \
            f"BB3 arrival error {ev.arrival_error_mm:.1f} mm exceeds 8 mm"


class TestBB4RapidSuccession:
    """BB4: Two throws 1.5s apart — tests throughput under time pressure."""

    def test_no_solver_failures(self, plant, mpc):
        """MPC should not enter failure cascade during rapid succession."""
        result = _run_bb(plant, mpc, 'BB4')
        assert mpc.consecutive_failures < 5, \
            f"MPC had {mpc.consecutive_failures} consecutive failures"
