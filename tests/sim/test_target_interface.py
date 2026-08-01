"""Phase 2: Tests for the target-based MPC solve() interface.

Validates that the MPC correctly tracks target poses with:
  - ASAP mode (no arrival_time): converges as fast as constraints allow
  - Timed mode (arrival_time): arrives near the deadline
"""

from __future__ import annotations

import numpy as np
import pytest

from sim.plant.mujoco_plant import MuJoCoPlant
from controller.mpc import MPCController
from controller.params import MPCParams
from controller.target import flat_target_to_events

CONTROL_DT = 0.025


def _create_mpc(plant, **overrides):
    defaults = dict(max_cpu_time=2.0, max_iter=500, max_leg_vel_mmps=1000.0,
                    prime_solver=False)
    defaults.update(overrides)
    return MPCController.from_plant(MPCParams(**defaults), plant)


def _run_target(plant, mpc, target_pose, arrival_time, duration_s,
                target_twist=None):
    """Run MPC tracking a target pose, return list of state dicts."""
    n_steps = int(duration_s / CONTROL_DT)
    states = []
    for _ in range(n_steps):
        state = plant.get_state()
        current_pose = np.concatenate([state.platform_pos_mm, state.platform_rot])
        ref_events = flat_target_to_events(
            current_pose, state.platform_twist, target_pose, state.time,
            target_twist=target_twist, arrival_time=arrival_time)
        cmd, _cmd_vel, diag = mpc.solve(
            state, target_pose,
            ref_events=ref_events,
        )
        plant.command(cmd)
        plant.step(CONTROL_DT)

        actual = np.concatenate([state.platform_pos_mm, state.platform_rot])
        states.append({
            'time': state.time,
            'pose': actual.copy(),
            'cmd': cmd.copy(),
            'diag': diag,
        })
    return states


@pytest.fixture(scope='module')
def plant():
    return MuJoCoPlant()


# ---------------------------------------------------------------------------
# ASAP mode (arrival_time=None)
# ---------------------------------------------------------------------------

class TestASAPMode:
    """Target pose with no arrival deadline — converge as fast as possible."""

    def test_converges_to_target(self, plant):
        """Platform reaches z+50mm target within 1.5s."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0, 0, 50, 0, 0, 0.0])

        states = _run_target(plant, mpc, target, arrival_time=None, duration_s=1.5)

        final = states[-1]['pose']
        pos_err = np.linalg.norm(final[:3] - target[:3])
        ori_err = np.degrees(np.linalg.norm(final[3:] - target[3:]))

        assert pos_err < 1.0, f"ASAP final pos error {pos_err:.2f} mm > 1.0 mm"
        assert ori_err < 0.5, f"ASAP final ori error {ori_err:.3f}° > 0.5°"

    def test_no_solver_failures(self, plant):
        """No solver failures after warm-up in ASAP mode."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0, 0, 50, 0, 0, 0.0])

        states = _run_target(plant, mpc, target, arrival_time=None, duration_s=1.5)

        failures = [s for s in states[5:] if 'fallback' in s['diag']['status']]
        assert len(failures) == 0, f"{len(failures)} solver failures"

    def test_combined_pose(self, plant):
        """ASAP mode handles combined translation + tilt."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([20, -15, 60, 0.05, -0.03, 0.0])

        states = _run_target(plant, mpc, target, arrival_time=None, duration_s=2.0)

        final = states[-1]['pose']
        pos_err = np.linalg.norm(final[:3] - target[:3])
        ori_err = np.degrees(np.linalg.norm(final[3:] - target[3:]))

        assert pos_err < 2.0, f"Combined pos error {pos_err:.2f} mm > 2.0 mm"
        assert ori_err < 1.0, f"Combined ori error {ori_err:.3f}° > 1.0°"


# ---------------------------------------------------------------------------
# Timed mode (arrival_time specified)
# ---------------------------------------------------------------------------

class TestTimedArrival:
    """Target pose with arrival deadline — terminal cost (Qf) provides timing."""

    def test_arrives_by_deadline(self, plant):
        """Platform reaches z+50mm by t=1.0s deadline."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0, 0, 50, 0, 0, 0.0])
        arrival = 1.0  # absolute time

        states = _run_target(plant, mpc, target, arrival_time=arrival, duration_s=1.5)

        # Find state nearest to deadline
        at_deadline = min(states, key=lambda s: abs(s['time'] - arrival))
        pos_err = np.linalg.norm(at_deadline['pose'][:3] - target[:3])

        # Should arrive within 3mm by deadline (allowing for actuator lag)
        assert pos_err < 3.0, (
            f"At deadline pos error {pos_err:.2f} mm > 3.0 mm "
            f"(time={at_deadline['time']:.3f}s)"
        )

    def test_holds_after_deadline(self, plant):
        """After arrival, platform holds at target pose."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0, 0, 50, 0, 0, 0.0])
        arrival = 0.8

        states = _run_target(plant, mpc, target, arrival_time=arrival, duration_s=2.0)

        # Check hold quality after deadline + settling (0.3s after deadline)
        hold_states = [s for s in states if s['time'] >= arrival + 0.3]
        assert len(hold_states) > 10

        hold_errors = [np.linalg.norm(s['pose'][:3] - target[:3]) for s in hold_states]
        max_hold_err = np.max(hold_errors)

        assert max_hold_err < 1.0, f"Hold pos error {max_hold_err:.2f} mm > 1.0 mm"

    def test_long_horizon_target(self, plant):
        """Target at 1.2s (within 1.35s horizon) is tracked accurately."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0, 0, 80, 0, 0, 0.0])
        arrival = 1.2

        states = _run_target(plant, mpc, target, arrival_time=arrival, duration_s=1.8)

        at_deadline = min(states, key=lambda s: abs(s['time'] - arrival))
        pos_err = np.linalg.norm(at_deadline['pose'][:3] - target[:3])

        assert pos_err < 3.0, (
            f"Long-horizon pos error {pos_err:.2f} mm > 3.0 mm"
        )

    def test_past_deadline_still_tracks(self, plant):
        """Target beyond horizon (2.0s) still works — MPC moves toward it."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0, 0, 50, 0, 0, 0.0])
        arrival = 2.0  # beyond 1.35s horizon

        states = _run_target(plant, mpc, target, arrival_time=arrival, duration_s=2.5)

        # Should eventually reach target
        final = states[-1]['pose']
        pos_err = np.linalg.norm(final[:3] - target[:3])
        assert pos_err < 1.0, f"Beyond-horizon final err {pos_err:.2f} mm > 1.0 mm"


# ---------------------------------------------------------------------------
# Reference construction
# ---------------------------------------------------------------------------

class TestBuildReference:
    """Unit tests for _build_reference() with event-based references."""

    def test_no_events_holds_target(self, plant):
        """Without events, reference holds at target pose."""
        mpc = _create_mpc(plant)
        plant.reset()
        state = plant.get_state()
        target = np.array([10, 20, 50, 0.01, 0, 0.0])

        ref, twist, _accel = mpc._build_reference(state, target)

        N = mpc.params.N
        assert ref.shape == (N + 1, 6)
        for k in range(N + 1):
            np.testing.assert_allclose(ref[k], target, atol=1e-10)
        np.testing.assert_allclose(twist, 0.0, atol=1e-10)

    def test_event_reference_endpoints(self, plant):
        """Event-based reference matches event poses at event times."""
        from controller.target import ReferenceEvent
        mpc = _create_mpc(plant)
        plant.reset()
        state = plant.get_state()

        p0 = np.array([0, 0, 50, 0, 0, 0.0])
        p1 = np.array([0, 0, 100, 0, 0, 0.0])
        events = [
            ReferenceEvent(time=state.time, pose=p0, twist=np.zeros(6),
                           accel=np.zeros(6)),
            ReferenceEvent(time=state.time + 1.0, pose=p1, twist=np.zeros(6),
                           accel=np.zeros(6)),
        ]

        ref, twist, _accel = mpc._build_reference(state, p0, ref_events=events)

        # Node 0 (t=state.time) should match first event
        np.testing.assert_allclose(ref[0], p0, atol=1e-10)
        np.testing.assert_allclose(twist[0], np.zeros(6), atol=1e-10)

    def test_event_reference_with_twist(self, plant):
        """Event-based reference propagates twist at event boundary."""
        from controller.target import ReferenceEvent
        mpc = _create_mpc(plant)
        plant.reset()
        state = plant.get_state()

        tw = np.array([100, 0, 0, 0, 0, 0.0])
        events = [
            ReferenceEvent(time=state.time + 0.05, pose=np.zeros(6),
                           twist=tw, accel=np.zeros(6)),
        ]

        _, twist, _accel = mpc._build_reference(state, np.zeros(6),
                                                  ref_events=events)

        # Last node (well past event) should extrapolate with the event's twist
        np.testing.assert_allclose(twist[-1], tw, atol=1e-8)


# ---------------------------------------------------------------------------
# flat_target_to_events — start-twist clamping (MPC_OVERSHOOT_SATURATION)
# ---------------------------------------------------------------------------

class TestFlatTargetTwistClamp:
    """``clamp_start_twist_mmps`` guards the reference against an FK spike.

    Without clamping, a noisy ``state.platform_twist`` that briefly reports
    >>v_max becomes the start-velocity boundary condition of the quintic —
    the MPC then has to plan a trajectory that matches a reference whose
    initial velocity exceeds physical actuator limits.  See logbook
    2026-04-18-move5-overshoot-stall-and-plant-collapse.md.
    """

    def _make(self, start_twist, clamp):
        current_pose = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
        target_pose = np.array([0.0, 0.0, 220.0, 0.0, 0.0, 0.0])
        return flat_target_to_events(
            current_pose, np.asarray(start_twist, dtype=float),
            target_pose, t_now=0.0,
            clamp_start_twist_mmps=clamp,
        )

    def test_noisy_linear_twist_is_clipped(self):
        """FK-reported +500 mm/s is clipped to ±v_max on linear components."""
        events = self._make([500.0, -500.0, 500.0, 0.0, 0.0, 0.0],
                            clamp=140.0)
        # First event is the synthetic t=t_now warm-up carrying plant state
        np.testing.assert_allclose(events[0].twist[:3], [140.0, -140.0, 140.0])

    def test_in_band_linear_twist_unchanged(self):
        """A plausible twist (within ±v_max) is passed through untouched."""
        events = self._make([50.0, -30.0, 10.0, 0.0, 0.0, 0.0],
                            clamp=140.0)
        np.testing.assert_allclose(events[0].twist[:3], [50.0, -30.0, 10.0])

    def test_angular_components_untouched(self):
        """Angular twist (rad/s) is left alone — the clamp is mm/s only."""
        events = self._make([0.0, 0.0, 0.0, 2.5, -3.1, 4.0],
                            clamp=140.0)
        np.testing.assert_allclose(events[0].twist[3:], [2.5, -3.1, 4.0])

    def test_none_disables_clamp(self):
        """Passing ``None`` (default) means the twist is passed through raw."""
        events = self._make([500.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                            clamp=None)
        np.testing.assert_allclose(events[0].twist[:3], [500.0, 0.0, 0.0])

    def test_clamp_does_not_mutate_caller_array(self):
        """The clamp works on a copy — the caller's twist array is preserved."""
        original = np.array([500.0, -500.0, 500.0, 0.0, 0.0, 0.0])
        twist_arg = original.copy()
        self._make(twist_arg, clamp=140.0)
        np.testing.assert_allclose(twist_arg, original)
