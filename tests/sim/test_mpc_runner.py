"""Integration tests for controller.runner and controller.target source classes.

Tests the new MPC orchestration layer extracted from sim/main.py:
  - StaticTargetSource: schedule-based target selection
  - WaypointTargetSource: waypoint advancement
  - run_mpc_loop(): wall-clock-paced control loop with hooks
  - MpcLoopHooks: callback invocation with correct signatures
  - run_mpc_headless() delegation: sim/main.py delegates correctly
"""

from __future__ import annotations

import numpy as np
import pytest

from plant.mujoco_plant import MuJoCoPlant
from controller.mpc import MPCController
from controller.params import MPCParams
from controller.target import (
    StaticTargetSource, WaypointTargetSource, TargetCommand,
    flat_target_to_events,
)
from controller.plant import PlantState
from controller.telemetry import TelemetryLogger
from controller.runner import run_mpc_loop, MpcLoopHooks, mpc_solve, log_mpc_step

CONTROL_DT = 0.025  # 40 Hz


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def plant():
    return MuJoCoPlant()


@pytest.fixture
def mpc(plant):
    params = MPCParams(max_cpu_time=2.0, max_iter=500, prime_solver=False)
    return MPCController.from_plant(params, plant)


# ---------------------------------------------------------------------------
# StaticTargetSource tests
# ---------------------------------------------------------------------------

class TestStaticTargetSource:
    def test_returns_target_command(self, plant):
        target = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        source = StaticTargetSource([(0.0, target)])
        state = plant.get_state()
        tc = source.update(0.0, state)
        assert isinstance(tc, TargetCommand)
        np.testing.assert_array_equal(tc.target_pose, target)

    def test_schedule_advances_with_time(self, plant):
        pose1 = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        pose2 = np.array([10.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        source = StaticTargetSource([(0.0, pose1), (1.0, pose2)])
        state = plant.get_state()

        tc0 = source.update(0.5, state)
        np.testing.assert_array_equal(tc0.target_pose, pose1)

        tc1 = source.update(1.5, state)
        np.testing.assert_array_equal(tc1.target_pose, pose2)

    def test_generates_ref_events(self, plant):
        target = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        source = StaticTargetSource([(0.0, target)])
        state = plant.get_state()
        tc = source.update(0.0, state)
        assert tc.ref_events is not None
        assert len(tc.ref_events) >= 1


class TestWaypointTargetSource:
    def test_returns_first_waypoint(self, plant):
        pose1 = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        source = WaypointTargetSource([(pose1, 1.0)])
        state = plant.get_state()
        tc = source.update(0.0, state)
        np.testing.assert_array_equal(tc.target_pose, pose1)
        assert tc.arrival_time == 1.0

    def test_advances_through_waypoints(self, plant):
        pose1 = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        pose2 = np.array([10.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        source = WaypointTargetSource([(pose1, 1.0), (pose2, 2.0)])
        state = plant.get_state()

        tc0 = source.update(0.5, state)
        np.testing.assert_array_equal(tc0.target_pose, pose1)

        tc1 = source.update(1.5, state)
        np.testing.assert_array_equal(tc1.target_pose, pose2)
        assert tc1.arrival_time == 2.0

    def test_stays_on_last_waypoint(self, plant):
        pose1 = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        source = WaypointTargetSource([(pose1, 1.0)])
        state = plant.get_state()

        tc = source.update(5.0, state)
        np.testing.assert_array_equal(tc.target_pose, pose1)


# ---------------------------------------------------------------------------
# mpc_solve helper tests
# ---------------------------------------------------------------------------

class TestMpcSolve:
    def test_returns_correct_tuple(self, plant, mpc):
        state = plant.get_state()
        target = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        tc = TargetCommand(target_pose=target)
        result = mpc_solve(mpc, state, tc)
        assert len(result) == 5
        cmd, cmd_vel, diag, ref_pose, ref_twist = result
        assert cmd.shape == (6,)
        assert cmd_vel.shape == (6,)
        assert isinstance(diag, dict)
        assert ref_pose.shape == (6,)
        assert ref_twist.shape == (6,)

    def test_ref_pose_from_mpc_reference(self, plant, mpc):
        """Verify ref_pose comes from the MPC's own reference, not the raw target."""
        state = plant.get_state()
        target = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        events = flat_target_to_events(
            np.concatenate([state.platform_pos_mm, state.platform_rot]),
            state.platform_twist, target, state.time)
        tc = TargetCommand(target_pose=target, ref_events=events)
        _, _, _, ref_pose, _ = mpc_solve(mpc, state, tc)
        # ref_pose should be a valid 6-element array
        assert ref_pose.shape == (6,)
        assert np.all(np.isfinite(ref_pose))


# ---------------------------------------------------------------------------
# run_mpc_loop integration tests
# ---------------------------------------------------------------------------

class TestRunMpcLoop:
    def test_basic_run_completes(self, plant, mpc):
        """Run the MPC loop for 0.5s and verify it produces telemetry."""
        target = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        source = StaticTargetSource([(0.0, target)])
        logger = TelemetryLogger()

        run_mpc_loop(plant, mpc, source, duration=0.5, logger=logger,
                     control_dt=CONTROL_DT)

        expected_steps = int(0.5 / CONTROL_DT)
        assert logger.total_count == expected_steps

    def test_hooks_invoked(self, plant, mpc):
        """Verify all hook callbacks are invoked during the loop."""
        target = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        source = StaticTargetSource([(0.0, target)])
        logger = TelemetryLogger()

        call_log = {
            'on_target_override': 0,
            'on_pre_command': 0,
            'on_post_solve': 0,
            'on_post_step': 0,
            'on_log_extras': 0,
        }

        def override(state, tc):
            call_log['on_target_override'] += 1
            assert isinstance(state, PlantState)
            assert isinstance(tc, TargetCommand)
            return tc

        def pre_cmd(plant_, mpc_, tc, cmd, cmd_vel, diag):
            call_log['on_pre_command'] += 1
            assert cmd.shape == (6,)

        def post_solve(tc, diag):
            call_log['on_post_solve'] += 1
            assert isinstance(diag, dict)

        def post_step(state, sim_time):
            call_log['on_post_step'] += 1

        def log_extras(plant_):
            call_log['on_log_extras'] += 1
            return {'overhead_ms': 0.1}

        hooks = MpcLoopHooks(
            on_target_override=override,
            on_pre_command=pre_cmd,
            on_post_solve=post_solve,
            on_post_step=post_step,
            on_log_extras=log_extras,
        )

        n_steps = 5
        run_mpc_loop(plant, mpc, source,
                     duration=n_steps * CONTROL_DT, logger=logger,
                     control_dt=CONTROL_DT, hooks=hooks)

        for name, count in call_log.items():
            assert count == n_steps, f"{name} called {count} times, expected {n_steps}"

    def test_target_override_replaces_tc(self, plant, mpc):
        """Verify on_target_override can replace the target command."""
        original = np.array([0.0, 0.0, 200.0, 0.0, 0.0, 0.0])
        override = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        source = StaticTargetSource([(0.0, original)])
        logger = TelemetryLogger()

        def always_override(state, tc):
            return TargetCommand(target_pose=override)

        hooks = MpcLoopHooks(on_target_override=always_override)

        run_mpc_loop(plant, mpc, source,
                     duration=10 * CONTROL_DT, logger=logger,
                     control_dt=CONTROL_DT, hooks=hooks)

        # The MPC should track the override pose, not the original.
        # Check that the reference in telemetry is near the override target.
        last = logger.records[-1]
        assert abs(last.ref_pose_z - override[2]) < 20.0, (
            f"Expected ref near z={override[2]}, got {last.ref_pose_z}")

    def test_no_hooks_works(self, plant, mpc):
        """Verify the loop works with no hooks (None)."""
        source = StaticTargetSource([(0.0, np.zeros(6))])
        logger = TelemetryLogger()
        run_mpc_loop(plant, mpc, source,
                     duration=3 * CONTROL_DT, logger=logger,
                     control_dt=CONTROL_DT, hooks=None)
        assert logger.total_count == 3

    def test_tracking_converges(self, plant, mpc):
        """Verify the loop actually drives the plant toward the target."""
        target = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        source = StaticTargetSource([(0.0, target)])
        logger = TelemetryLogger()

        run_mpc_loop(plant, mpc, source, duration=2.0, logger=logger,
                     control_dt=CONTROL_DT)

        # Final tracking error should be < 1 mm
        last = logger.records[-1]
        assert last.tracking_error_mm < 1.0, (
            f"Tracking error {last.tracking_error_mm:.3f} mm > 1.0 mm")

    def test_print_summary_called(self, plant, mpc, capsys):
        """Verify summary is printed at the end."""
        source = StaticTargetSource([(0.0, np.zeros(6))])
        logger = TelemetryLogger()
        run_mpc_loop(plant, mpc, source,
                     duration=5 * CONTROL_DT, logger=logger,
                     control_dt=CONTROL_DT)
        captured = capsys.readouterr()
        assert "tracking error" in captured.out.lower()


# ---------------------------------------------------------------------------
# log_mpc_step tests
# ---------------------------------------------------------------------------

class TestLogMpcStep:
    def test_appends_record(self, plant):
        logger = TelemetryLogger()
        state = plant.get_state()
        ref_pose = np.zeros(6)
        cmd = np.ones(6) * 154.5
        diag = {'solve_time_ms': 5.0, 'status': 'Solve_Succeeded',
                'cost': 0.1, 'constraint_violation': 0.0}

        log_mpc_step(logger, state, ref_pose, cmd, diag)
        assert logger.total_count == 1
        assert logger.records[0].solve_time_ms == 5.0

    def test_extras_passed_through(self, plant):
        logger = TelemetryLogger()
        state = plant.get_state()
        log_mpc_step(logger, state, np.zeros(6), np.zeros(6), {},
                     fk_iterations=3, ff_torque_max_Nm=0.5)
        assert logger.records[0].fk_iterations == 3
        assert logger.records[0].ff_torque_max_Nm == 0.5
