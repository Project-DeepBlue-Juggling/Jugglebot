"""Tests for the cold-start robustness fixes.

Covers the three fixes introduced to eliminate the initial solver-saturation
cluster observed in 2026-04-17-130811:

  * Approach 1 — AOT solver compilation (staleness detection only; the
    full AOT path requires gcc + a writable generated/ dir, tested via
    an integration smoke test when the .so is present).
  * Approach 2 — Reference-clock freeze on fallback-class solver status.
    Covers both the MPC-side ``t_now`` kwarg and the runner-side gating.
  * Approach 3a — Prime solve during ``MPCController.__init__``.
  * Approach 3b — Simplified ``_cold_start`` (single IK call, q-space
    interpolation).
"""

from __future__ import annotations

from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest

from plant.mujoco_plant import MuJoCoPlant
from controller.mpc import MPCController
from controller.params import MPCParams
from controller.target import (
    TargetCommand, TargetSource, flat_target_to_events,
)
from controller.telemetry import TelemetryLogger
from controller.runner import run_mpc_loop


CONTROL_DT = 0.025


@pytest.fixture(scope='module')
def plant():
    return MuJoCoPlant()


@pytest.fixture
def fast_mpc(plant):
    """MPC with a generous CPU budget so solves always succeed in CI."""
    plant.reset()
    params = MPCParams(
        max_cpu_time=2.0, max_iter=500, max_leg_vel_mmps=280.0,
        prime_solver=False,
    )
    return MPCController.from_plant(params, plant)


# ---------------------------------------------------------------------------
# Approach 3b — _cold_start simplification
# ---------------------------------------------------------------------------

class TestSimplifiedColdStart:
    def test_cold_start_calls_ik_once(self, fast_mpc):
        """_cold_start must call numerical IK exactly once (was N)."""
        N = fast_mpc._N
        p_cur = np.zeros(6)
        q_cur = np.zeros(6)
        ref_traj = np.zeros((N + 1, 6))
        ref_traj[1:, 2] = 50.0  # z=50 target for each future node

        with patch.object(
            fast_mpc, '_numerical_ik',
            wraps=fast_mpc._numerical_ik,
        ) as mock_ik:
            w0 = fast_mpc._cold_start(p_cur, q_cur, ref_traj)
        assert mock_ik.call_count == 1, (
            f"_cold_start should call IK exactly once; saw {mock_ik.call_count}"
        )
        assert w0.shape == (fast_mpc._n_w,)
        assert np.all(np.isfinite(w0))

    def test_cold_start_q_endpoints(self, fast_mpc):
        """q block should start near q_cur and end near IK(ref_traj[N])."""
        N = fast_mpc._N
        p_cur = np.zeros(6)
        q_cur = np.full(6, 10.0)
        ref_traj = np.zeros((N + 1, 6))
        ref_traj[:, 2] = 40.0

        w0 = fast_mpc._cold_start(p_cur, q_cur, ref_traj)
        q_block = w0[6 * N: 12 * N].reshape(N, 6)
        q_final_expected = np.clip(
            fast_mpc._numerical_ik(ref_traj[N]), 0.0, fast_mpc._params.stroke_mm
        )
        # First q-row should be closer to q_cur than to q_final; last row the
        # reverse.  We only test monotonicity, not exact interpolation values.
        first_to_cur = np.linalg.norm(q_block[0] - q_cur)
        last_to_final = np.linalg.norm(q_block[-1] - q_final_expected)
        first_to_final = np.linalg.norm(q_block[0] - q_final_expected)
        assert first_to_cur < first_to_final
        assert last_to_final < 1e-6


# ---------------------------------------------------------------------------
# Approach 3a — prime-solve
# ---------------------------------------------------------------------------

class TestPrimeSolver:
    def test_prime_solver_off_skips_prime(self, plant):
        """prime_solver=False must not invoke the solver at construction."""
        params = MPCParams(
            max_cpu_time=0.5, max_iter=50, prime_solver=False,
        )
        with patch.object(
            MPCController, '_prime_solver', autospec=True
        ) as mock_prime:
            MPCController.from_plant(params, plant)
        assert mock_prime.call_count == 0

    def test_prime_solver_on_invokes_prime(self, plant):
        """prime_solver=True must call _prime_solver once."""
        params = MPCParams(
            max_cpu_time=0.5, max_iter=50, prime_solver=True,
        )
        with patch.object(
            MPCController, '_prime_solver', autospec=True
        ) as mock_prime:
            MPCController.from_plant(params, plant)
        assert mock_prime.call_count == 1

    def test_prime_skips_seeding_when_plant_at_stow(self, plant):
        """When the plant reports zero leg extensions (e.g. freshly-reset
        sim at home keyframe), ``from_plant`` intentionally skips the
        warm-start seeding path — there's nothing meaningful to seed from
        at STOW — so ``_prev_w`` stays None after the prime-solve completes.
        """
        plant.reset()
        params = MPCParams(
            max_cpu_time=0.5, max_iter=50, prime_solver=True,
        )
        mpc = MPCController.from_plant(params, plant)
        assert mpc._prev_w is None
        assert mpc._timeout_hint is None

    def test_prime_seeds_warm_start_from_non_stow_state(self, plant):
        """When supplied with a non-zero, IK-consistent (p, q) pair, the
        prime solver seeds ``_prev_w`` / ``_prev_u`` so the first real
        solve is warm-started at the live pose.
        """
        plant.reset()
        params = MPCParams(
            max_cpu_time=2.0, max_iter=500, prime_solver=False,
        )
        mpc = MPCController.from_plant(params, plant)
        p_seed = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        q_seed = mpc._numerical_ik(p_seed)
        mpc._prime_solver(
            init_pose_6dof=p_seed,
            init_leg_extensions_mm=q_seed,
        )
        assert mpc._prev_w is not None, "expected warm-start seed after prime"
        assert mpc._prev_u is not None
        assert np.all(np.isfinite(mpc._prev_w))

    def test_prime_rejects_inconsistent_seed(self, plant):
        """An (p, q) pair that fails IK-consistency (as would happen during
        HardwarePlant cold-start when motor telemetry arrives before FK
        converges) is rejected; ``_prev_w`` stays None.
        """
        plant.reset()
        params = MPCParams(
            max_cpu_time=0.5, max_iter=50, prime_solver=False,
        )
        mpc = MPCController.from_plant(params, plant)
        # Inconsistent: claim pose is at STOW while legs are fully extended.
        mpc._prime_solver(
            init_pose_6dof=np.zeros(6),
            init_leg_extensions_mm=np.full(6, 100.0),
        )
        assert mpc._prev_w is None


# ---------------------------------------------------------------------------
# Approach 2 — reference-clock freeze
# ---------------------------------------------------------------------------

class TestReferenceClockOverride:
    def test_t_now_overrides_state_time_in_solve(self, fast_mpc, plant):
        """Passing t_now makes MPC sample the reference at that time."""
        plant.reset()
        state = plant.get_state()
        target = np.array([0.0, 0.0, 30.0, 0.0, 0.0, 0.0])
        current = np.concatenate([state.platform_pos_mm, state.platform_rot])
        ref_events = flat_target_to_events(
            current, state.platform_twist, target, t_now=state.time,
            arrival_time=state.time + 1.0,
        )

        # Solve with explicit t_now in the future: node 0 reference should
        # correspond to the later sample time.
        fast_mpc.solve(state, target, ref_events=ref_events,
                       t_now=state.time + 0.5)
        ref_later = fast_mpc.last_ref_traj[0].copy()

        fast_mpc.solve(state, target, ref_events=ref_events,
                       t_now=state.time)
        ref_early = fast_mpc.last_ref_traj[0].copy()

        # Later sample should be closer to the target than the earlier one.
        assert (np.linalg.norm(ref_later - target)
                < np.linalg.norm(ref_early - target))


class _ScriptedSolveSource(TargetSource):
    """Returns a flat target; records the sim_time it was called with."""
    def __init__(self, target):
        self._target = np.asarray(target, dtype=float)
        self.seen_times: list[float] = []

    def update(self, sim_time, state):
        self.seen_times.append(sim_time)
        current = np.concatenate([state.platform_pos_mm, state.platform_rot])
        ref_events = flat_target_to_events(
            current, state.platform_twist, self._target,
            t_now=sim_time, arrival_time=sim_time + 1.0,
        )
        return TargetCommand(
            target_pose=self._target, ref_events=ref_events)


class TestRunnerFreezesRefOnFallback:
    def test_t_ref_frozen_when_solver_returns_fallback(self, plant):
        """Runner should hold t_ref while solve_status is in a fallback class.

        We patch ``mpc.solve`` to return a fallback status for the first
        N ticks and then success.  The scripted source records every
        sim_time it saw; the frozen window should see the same value.
        """
        plant.reset()
        params = MPCParams(
            max_cpu_time=2.0, max_iter=500, max_leg_vel_mmps=280.0,
            prime_solver=False,
        )
        mpc = MPCController.from_plant(params, plant)

        target = np.array([0.0, 0.0, 30.0, 0.0, 0.0, 0.0])
        source = _ScriptedSolveSource(target)
        logger = TelemetryLogger()

        original_solve = mpc.solve
        call_idx = {'n': 0}

        def fake_solve(state, target_pose, **kwargs):
            cmd, cmd_vel, diag = original_solve(state, target_pose, **kwargs)
            # Force fallback status for ticks 2..6 inclusive (5 ticks).
            if 2 <= call_idx['n'] <= 6:
                diag = dict(diag)
                diag['status'] = 'fallback(Maximum_CpuTime_Exceeded)'
            call_idx['n'] += 1
            return cmd, cmd_vel, diag

        with patch.object(mpc, 'solve', side_effect=fake_solve):
            run_mpc_loop(plant, mpc, source, duration=0.25, logger=logger,
                         control_dt=CONTROL_DT)

        assert len(source.seen_times) >= 7, "expected at least 7 ticks"
        # t_ref deltas across ticks; zero during the fallback window.
        deltas = np.diff(source.seen_times)
        # The exact indices depend on the first-tick initialisation, but at
        # least one delta in the fallback window (ticks 2..6) should be ~0.
        frozen_deltas = deltas[2:6]
        assert np.all(frozen_deltas < CONTROL_DT / 4), (
            f"reference advanced during fallback: {frozen_deltas}"
        )
        # After recovery, deltas should resume ~CONTROL_DT.
        post_deltas = deltas[7:]
        assert np.all(np.abs(post_deltas - CONTROL_DT) < CONTROL_DT / 4), (
            f"reference did not resume after fallback: {post_deltas}"
        )


# ---------------------------------------------------------------------------
# Approach 1 — AOT staleness detection
# ---------------------------------------------------------------------------

class TestAotStaleness:
    def test_stale_so_hard_fails(self, plant, tmp_path, monkeypatch):
        """A present .so with a mismatched hash must raise on build."""
        so_path = tmp_path / 'mpc_gen.so'
        hash_path = tmp_path / 'mpc_gen.hash'
        so_path.write_bytes(b'\x7fELF')  # file must merely exist
        hash_path.write_text('deadbeef' * 8)  # intentional mismatch

        monkeypatch.setattr(MPCController, '_AOT_SO_PATH', so_path)
        monkeypatch.setattr(MPCController, '_AOT_HASH_PATH', hash_path)

        with pytest.raises(RuntimeError, match='generate_solver.py'):
            MPCController.from_plant(
                MPCParams(max_cpu_time=0.5, prime_solver=False),
                plant,
            )

    def test_absent_so_falls_back_to_in_process(self, plant, tmp_path,
                                                 monkeypatch):
        """No .so at the expected path should NOT raise; build in-process."""
        so_path = tmp_path / 'mpc_gen.so'  # does not exist
        hash_path = tmp_path / 'mpc_gen.hash'

        monkeypatch.setattr(MPCController, '_AOT_SO_PATH', so_path)
        monkeypatch.setattr(MPCController, '_AOT_HASH_PATH', hash_path)

        mpc = MPCController.from_plant(
            MPCParams(max_cpu_time=0.5, prime_solver=False),
            plant,
        )
        assert mpc._solver is not None
