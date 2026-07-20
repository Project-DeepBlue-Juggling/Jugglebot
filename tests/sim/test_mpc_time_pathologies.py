"""Plan 2 Phase 8 (Tier 3b) — Time pathologies, resource exhaustion, hooks, races.

Closes the final Tier-3 sad-path categories: clock skew, dropped ticks,
dt-schedule mid-run change, telemetry pool overflow, S3 event-queue overflow,
file-I/O failure on flush, runner-hook contract failures (return-None,
raising), on_post_solve allocation in the hot loop, ZmqTargetSource.reset()
mid-APPROACHING, and a concurrency-audit structural assertion.

Test IDs from
[plans/archived/2026-05-18 mpc-sadpath-coverage-tiers-1-3.md](../../plans/archived/2026-05-18%20mpc-sadpath-coverage-tiers-1-3.md)
Phase 8:

| ID         | Surface                                                                      | Driver                                                                |
|------------|------------------------------------------------------------------------------|-----------------------------------------------------------------------|
| T-U-T3b-T1 | `mpc.solve(t_now=...)` backward; scheduler S6                                | direct solve() + scheduler.update() back-to-back                       |
| T-U-T3b-T2 | Dropped tick (sim_time advances 2× control_dt)                                | direct solve() w/ t_now jumping by 2× control_dt                       |
| T-U-T3b-T3 | dt-schedule change mid-run                                                    | construct MPC_A, run 10 ticks, construct MPC_B with a different schedule |
| T-U-T3b-T4 | Property: random (sim_time, dt) sequences never produce `_prev_w` corruption | RuleBasedStateMachine over (succeed_solve, advance_time, jump_time)    |
| T-U-T3b-R1 | Telemetry pool overflow (wrap policy)                                         | TelemetryLogger(pool_size=10) + 100 next_record() calls                |
| T-U-T3b-R2 | Event-queue overflow at S3 boundary                                            | EventScheduler + 3 submit_event back-to-back                           |
| T-U-T3b-R3 | File I/O failure on write_csv                                                  | monkey-patch builtins.open to raise PermissionError                    |
| T-U-T3b-H1 | `on_target_override` returning None (XFAIL pre-bugfix — surfaces real bug)    | run_mpc_loop with hook returning None                                  |
| T-U-T3b-H2 | `on_target_override` raising                                                   | run_mpc_loop with hook raising RuntimeError                            |
| T-U-T3b-H3 | `on_pre_command` raising                                                       | run_mpc_loop with hook raising RuntimeError                            |
| T-U-T3b-H4 | `on_post_solve` allocating in hot loop                                         | run_mpc_loop + tracemalloc; assert per-tick alloc < THRESHOLD_BYTES    |
| T-U-T3b-R4 | `ZmqTargetSource.reset()` mid-APPROACHING                                      | direct reset() call; assert _prev_w untouched, source fields cleared   |
| T-U-T3b-R5 | Concurrent state read/write audit                                              | structural / documentation assertion                                   |

**Empirical findings (probe table, 2026-05-12 on CasADi 3.7.2 + Jetson):**

| ID         | Driver                                                                 | Observed behaviour                                                       |
|------------|------------------------------------------------------------------------|--------------------------------------------------------------------------|
| T-U-T3b-T1 | solve(t_now=-1.0) after t_now=0.5; scheduler.update(0.0) after 0.5     | solve() — `Solve_Succeeded` (no validation); scheduler — `ValueError` S6 |
| T-U-T3b-T2 | t_now jumps from 4×CONTROL_DT to 6×CONTROL_DT (skipping tick 5)        | `Solve_Succeeded`; `_prev_w` finite; recovery to 1×dt clean              |
| T-U-T3b-T3 | construct MPC_A with default dt_schedule, then MPC_B (fresh)            | MPC_B starts with `_prev_w is None` (cold-start); no in-place swap API   |
| T-U-T3b-T4 | RuleBasedStateMachine: succeed_solve / advance_time / jump_time         | Across-call invariant holds: `_prev_w is None or finite`                  |
| T-U-T3b-R1 | TelemetryLogger(pool_size=10, path=None) + 100 next_record()           | wraps silently; `len(records)` caps at 10; `total_count == 100`           |
| T-U-T3b-R2 | scheduler.submit_event x3 (slots: current + next + overflow)            | 3rd submit raises `ValueError` with "S3 violation"                        |
| T-U-T3b-R3 | monkey-patch `builtins.open` to raise PermissionError; flush()         | `PermissionError` propagates (no silent swallow)                          |
| T-U-T3b-H1 | run_mpc_loop with on_target_override returning None                     | **BUG**: `tc` passed into `mpc_solve` two lines later → `mpc.solve(state, tc.target_pose, ...)` raises `AttributeError: 'NoneType' object has no attribute 'target_pose'` |
| T-U-T3b-H2 | run_mpc_loop with on_target_override raising RuntimeError              | RuntimeError propagates out of the loop                                  |
| T-U-T3b-H3 | run_mpc_loop with on_pre_command raising RuntimeError                  | RuntimeError propagates out of the loop                                  |
| T-U-T3b-H4 | on_post_solve appends `diag.get('status')` to a list (~40 B/tick)      | Loop completes; tracemalloc per-tick allocation stays below THRESHOLD     |
| T-U-T3b-R4 | ZmqTargetSource.reset() called; MPC has populated _prev_w               | Source fields cleared; MPC._prev_w untouched (lives on MPC, not source)  |
| T-U-T3b-R5 | code audit                                                              | Hot loop is single-threaded; structural assertion documents this         |

**Bug surfaced (T-U-T3b-H1)** — `on_target_override` returning None crashes
the runner two lines downstream of the hook call: the runner passes the None
into ``mpc_solve`` (`runner.py::run_mpc_loop`), which dereferences
``tc.target_pose`` on the immediate next call to ``mpc.solve(...)`` and raises
``AttributeError: 'NoneType' object has no attribute 'target_pose'``.  The
hook's docstring says "Return the original tc to keep it unchanged" — None is
undocumented but a natural "no override needed" return shape.  Fixed
same-session: when the hook returns None, the runner keeps the original
``tc``.  See
[logbook/2026-05-12-tier3b-hook-bugfix.md](../../logbook/2026-05-12-tier3b-hook-bugfix.md).

See [logbook/2026-05-12-tier3b-time-pathologies.md](../../logbook/2026-05-12-tier3b-time-pathologies.md).
"""
from __future__ import annotations

import builtins
import gc
import logging
import os
import tempfile
import tracemalloc

import numpy as np
import pytest
from hypothesis import HealthCheck, settings, strategies as st
from hypothesis.stateful import RuleBasedStateMachine, invariant, rule

from controller.hot_loop_contract import (
    HOT_LOOP_CONTRACT_TRACEBACK_FRAMES,
    HOT_LOOP_CONTRACT_WINDOW_TICKS,
    THRESHOLD_BYTES,
)
from controller.mpc import MPCController
from controller.params import MPCParams
from controller.runner import MpcLoopHooks, run_mpc_loop
from controller.scheduler import (
    EventScheduler,
    EventType,
    ScheduledEvent,
)
from controller.target import StaticTargetSource
from controller.telemetry import TelemetryLogger
from plant.mujoco_plant import MuJoCoPlant


logging.getLogger('controller.mpc').setLevel(logging.CRITICAL)


REF_NORMAL = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
CONTROL_DT = 0.025

# Flag that gates the T-U-T3b-H1 xfail.  Flipped to True in the
# same-session bugfix commit when `runner.py` treats a None return from
# on_target_override as "keep tc unchanged".  Single source of truth —
# one place to edit at bugfix-land time.  Plan 2 Phase 8 Bug E bugfix
# landed; see logbook/2026-05-12-tier3b-hook-bugfix.md.
_PHASE_8_BUGFIX_LANDED: bool = True


# ---------------------------------------------------------------------
# Fixtures + helpers
# ---------------------------------------------------------------------

@pytest.fixture
def plant():
    """Function-scoped plant — many of these tests reset / re-run the
    plant and can't share a module-scoped instance safely."""
    return MuJoCoPlant()


def _create_mpc(plant, **overrides):
    """Build an MPC for sad-path tests.

    Mirrors the Phase 3 / Phase 7 helper: `use_aot_solver=False` so the
    in-process solver path is exercised (the AOT path is the production
    fast path but has no failure-injection seam), `prime_solver=False`
    so cold-start is explicit and seed sequences are deterministic.
    """
    defaults = dict(
        max_cpu_time=2.0, max_iter=500, max_leg_vel_mmps=280.0,
        prime_solver=False, use_aot_solver=False,
    )
    defaults.update(overrides)
    return MPCController.from_plant(MPCParams(**defaults), plant)


def _seed_mpc(plant, mpc, ref: np.ndarray = REF_NORMAL):
    """One successful solve so `_prev_w` is populated."""
    plant.reset()
    state = plant.get_state()
    _, _, diag = mpc.solve(state, ref)
    assert diag['status'] == 'Solve_Succeeded', diag['status']
    return state


# =====================================================================
# T-U-T3b-T1 — Backward sim_time
# =====================================================================

class TestT3bT1BackwardSimTime:
    """T-U-T3b-T1 — Documented separation between solve()-driven and
    scheduler-driven time sources.

    Pass criterion (from plan): "Documented behaviour: solver runs at
    given t (no validation in `solve()` itself); but scheduler-driven
    sources (Plan 1 S6) raise. Test confirms separation."

    Empirically on SHA `7d5e6da`:
    * `solve(t_now=-1.0)` after `t_now=0.5` returns `Solve_Succeeded` —
      no validation in `solve()` itself.  This is *intentional* — solve()
      is a pure function over (state, target, t_now); the caller owns
      monotonicity.  The runner's t_ref bookkeeping does the monotonic
      advance (frozen on fallback).
    * `scheduler.update(0.0)` after `scheduler.update(0.5)` raises
      `ValueError` containing "S6 violation" — landed by Plan 1 Phase 3.
    """

    def test_t3b_t1a_solve_does_not_validate_backward_t_now(self, plant):
        """`solve(t_now=...)` accepts backward t_now without raising."""
        mpc = _create_mpc(plant)
        _seed_mpc(plant, mpc)
        state = plant.get_state()

        # Forward solve at t=0.5 first.
        _, _, diag = mpc.solve(state, REF_NORMAL, t_now=0.5)
        assert diag['status'] == 'Solve_Succeeded', diag['status']

        # Backward — no raise expected, solve completes.
        cmd_back, _, diag_back = mpc.solve(state, REF_NORMAL, t_now=0.0)
        assert diag_back['status'] == 'Solve_Succeeded', (
            f"solve() must not validate backward t_now; "
            f"got status {diag_back['status']!r}"
        )
        assert np.all(np.isfinite(cmd_back)), (
            f"cmd must be finite even on backward t_now; got {cmd_back!r}"
        )

        # Far-backward (t_now=-1.0) — still no raise.
        cmd_far, _, diag_far = mpc.solve(state, REF_NORMAL, t_now=-1.0)
        assert diag_far['status'] == 'Solve_Succeeded'
        assert np.all(np.isfinite(cmd_far))

    def test_t3b_t1b_scheduler_raises_s6_on_backward_sim_time(self, plant):
        """`scheduler.update()` raises `ValueError` ("S6 violation") on
        backward `sim_time` per Plan 1 Phase 3."""
        mpc = _create_mpc(plant)
        sched = EventScheduler(
            active_pose=REF_NORMAL,
            cumulative_times=mpc._params.cumulative_times,
            v_max_mmps=140.0, tau_s=0.04,
        )
        # Forward update first.
        sched.update(
            sim_time=0.5,
            current_pose=REF_NORMAL,
            current_twist=np.zeros(6),
        )
        # Backward — must raise.
        with pytest.raises(ValueError, match=r"S6 violation"):
            sched.update(
                sim_time=0.0,
                current_pose=REF_NORMAL,
                current_twist=np.zeros(6),
            )


# =====================================================================
# T-U-T3b-T2 — Dropped tick (2× control_dt advance)
# =====================================================================

class TestT3bT2DroppedTick:
    """T-U-T3b-T2 — When sim_time advances by 2× control_dt instead of 1×
    (skipped scheduling tick), the loop must continue and `_prev_w` must
    stay finite.

    Pass criterion (from plan): "Loop continues; warm-start shift
    accommodates the gap."

    Empirically on SHA `7d5e6da`: `_shift_warm_start` shifts the prior
    optimal by exactly one node regardless of t_now jump magnitude (see
    `mpc.py::_shift_warm_start`).  A 2× control_dt jump leaves the
    warm-start node-aligned to ~1 control_dt earlier than the true state
    — but the IPOPT warm-start is just a hint, and the solver re-converges
    cleanly in <50 iters.  `_prev_w` stays finite throughout.
    """

    def test_t3b_t2_dropped_tick_preserves_warm_start(self, plant):
        """5 normal solves, then 2× control_dt jump, then 2 recovery
        solves — `_prev_w` finite throughout, statuses all
        `Solve_Succeeded`."""
        mpc = _create_mpc(plant)
        _seed_mpc(plant, mpc)
        state = plant.get_state()
        t_now = 0.0

        # 5 normal solves.
        for _ in range(5):
            _, _, diag = mpc.solve(state, REF_NORMAL, t_now=t_now)
            assert diag['status'] == 'Solve_Succeeded', diag['status']
            assert np.all(np.isfinite(mpc._prev_w))
            t_now += CONTROL_DT

        # Dropped tick: jump 2× control_dt.
        t_now += CONTROL_DT  # skip one
        _, _, diag = mpc.solve(state, REF_NORMAL, t_now=t_now)
        assert diag['status'] == 'Solve_Succeeded', diag['status']
        assert np.all(np.isfinite(mpc._prev_w)), (
            f"warm-start corrupted by 2× control_dt jump: "
            f"_prev_w finite={np.all(np.isfinite(mpc._prev_w))}"
        )

        # 2 recovery solves at 1× control_dt — finite + Solve_Succeeded.
        for _ in range(2):
            t_now += CONTROL_DT
            _, _, diag = mpc.solve(state, REF_NORMAL, t_now=t_now)
            assert diag['status'] == 'Solve_Succeeded', diag['status']
            assert np.all(np.isfinite(mpc._prev_w))


# =====================================================================
# T-U-T3b-T3 — dt-schedule swap mid-run
# =====================================================================

class TestT3bT3DtScheduleSwap:
    """T-U-T3b-T3 — Documented behaviour: dt-schedule is construct-time
    only; there is no in-place swap mechanism.  Constructing a new
    MPCController produces a fresh instance with `_prev_w is None`
    (cold-start) — the only "swap" path available.

    Pass criterion (from plan): "Warm-start invalidation logic kicks in;
    first tick after swap is cold-start."

    Empirically on SHA `7d5e6da`:
    * `MPCParams.dt_schedule` is a `tuple` (immutable).
    * `MPCController._dt_schedule` is captured at `__init__` and bound
      into the CasADi NLP — mutating it after construction would not
      change the solver's compiled constraints.
    * No `set_dt_schedule` method exists on `MPCController`.
    * Constructing a new MPC instance produces a fresh `_prev_w = None`.

    This test pins the documented behaviour: dt-schedule changes require
    a new MPC instance, which has no warm-start.  If a future refactor
    adds an in-place setter that *should* invalidate the warm-start, this
    test is the structural reminder of the invariant.
    """

    def test_t3b_t3a_dt_schedule_is_construct_time_only(self, plant):
        """`MPCController` exposes no public setter for `dt_schedule`."""
        mpc = _create_mpc(plant)
        assert not hasattr(mpc, 'set_dt_schedule'), (
            "If a public set_dt_schedule() setter is added, it MUST "
            "invalidate the warm-start (set _prev_w to None) — this "
            "test should then be replaced with a setter-invalidation "
            "scenario, not removed."
        )
        # dt_schedule on MPCParams is a tuple — immutable.
        assert isinstance(mpc._params.dt_schedule, tuple), (
            f"MPCParams.dt_schedule type drift: "
            f"{type(mpc._params.dt_schedule).__name__}"
        )

    def test_t3b_t3b_new_mpc_starts_cold(self, plant):
        """Run 10 solves on MPC_A; constructing MPC_B yields a fresh
        instance whose `_prev_w is None` (cold-start)."""
        mpc_a = _create_mpc(plant)
        _seed_mpc(plant, mpc_a)
        state = plant.get_state()
        for k in range(10):
            _, _, diag = mpc_a.solve(state, REF_NORMAL,
                                     t_now=k * CONTROL_DT)
            assert diag['status'] == 'Solve_Succeeded'
        assert mpc_a._prev_w is not None
        assert np.all(np.isfinite(mpc_a._prev_w))

        # MPC_B is a brand-new instance — no warm-start.
        plant_b = MuJoCoPlant()
        mpc_b = _create_mpc(plant_b)
        assert mpc_b._prev_w is None, (
            "fresh MPCController must start with _prev_w is None "
            "(cold-start); the documented 'swap' path is construct-new"
        )


# =====================================================================
# T-U-T3b-T4 — Hypothesis stateful: random (t_now, dt) sequences
# never corrupt `_prev_w`
# =====================================================================

# Module-scoped singletons amortise the ~2 s MPC + plant build across
# 50-1000 hypothesis examples per test run.  Same pattern as Phase 3's
# T1cWarmStartIntegrityMachine and Phase 7's T3aS7 schema machine.
_T3B_T4_SINGLETONS: tuple[MuJoCoPlant, MPCController] | None = None


def _t3b_t4_get_singletons():
    """Lazy-build the shared (plant, mpc) pair.  Each rule resets and
    re-seeds them so successive walks don't pollute each other."""
    global _T3B_T4_SINGLETONS
    if _T3B_T4_SINGLETONS is None:
        plant = MuJoCoPlant()
        mpc = _create_mpc(plant)
        _T3B_T4_SINGLETONS = (plant, mpc)
    return _T3B_T4_SINGLETONS


class T3bT4TimePathologyMachine(RuleBasedStateMachine):
    """T-U-T3b-T4 — warm-start integrity invariant under arbitrary
    sequences of (succeed_solve, advance_time_random_dt,
    jump_time_backward).

    **Invariant checked after every rule:**

    * ``warm_start_finite_or_none`` — ``_prev_w`` is None or all-finite
      (the load-bearing safety property: a corrupted warm-start would
      cascade non-finite extensions to the actuators).

    Plan choice rationale: ``RuleBasedStateMachine`` over ``@given``
    because the invariant is *across-call*.  Pure ``@given`` would only
    test one (t_now, dt) tuple per example; the warm-start corruption
    mode the invariant guards against requires a *sequence*
    (succeed → forward → backward → forward → jump-by-10× → succeed)
    to expose.
    """

    def __init__(self):
        super().__init__()
        self._plant, self._mpc = _t3b_t4_get_singletons()
        self._plant.reset()
        self._mpc.reset()
        # Seed one successful solve so _prev_w is populated.
        state = self._plant.get_state()
        _, _, diag = self._mpc.solve(state, REF_NORMAL)
        assert diag['status'] == 'Solve_Succeeded', diag['status']
        self._t_now = 0.0

    # ---- Rules ----

    @rule(dt=st.floats(min_value=0.0, max_value=10 * CONTROL_DT,
                       allow_nan=False, allow_infinity=False))
    def advance_time(self, dt):
        """Advance `t_now` by a random forward dt in [0, 10× control_dt]
        and call `solve()`.  Covers normal dt, dropped tick (2× dt),
        and gross overshoots."""
        self._t_now += dt
        state = self._plant.get_state()
        try:
            self._mpc.solve(state, REF_NORMAL, t_now=self._t_now)
        except Exception:
            # solve() may surface CasADi exceptions on extreme inputs;
            # the invariants still apply.
            pass

    @rule(jump=st.floats(min_value=-5.0, max_value=0.0,
                         allow_nan=False, allow_infinity=False))
    def jump_time_backward(self, jump):
        """Inject a backward `t_now` jump.  solve() does not validate
        (T-U-T3b-T1a), so this exercises the warm-start path's tolerance
        to a non-monotonic reference clock."""
        self._t_now += jump  # jump <= 0, so this moves t_now backward
        state = self._plant.get_state()
        try:
            self._mpc.solve(state, REF_NORMAL, t_now=self._t_now)
        except Exception:
            pass

    @rule()
    def succeed_solve(self):
        """One clean solve at the current t_now.  Restores `_prev_w` to a
        fresh post-success snapshot."""
        state = self._plant.get_state()
        try:
            self._mpc.solve(state, REF_NORMAL, t_now=self._t_now)
        except Exception:
            pass

    # ---- Invariants ----

    @invariant()
    def warm_start_finite_or_none(self):
        w = self._mpc._prev_w
        assert w is None or np.all(np.isfinite(w)), (
            f"T-U-T3b-T4 invariant violation: _prev_w contains "
            f"non-finite values at t_now={self._t_now}: {w!r}"
        )

    @invariant()
    def prev_u_finite_or_none(self):
        u = self._mpc._prev_u
        assert u is None or np.all(np.isfinite(u)), (
            f"_prev_u non-finite at t_now={self._t_now}: {u!r}"
        )


T3bT4TimePathologyMachine.TestCase.settings = settings(
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow,
                           HealthCheck.function_scoped_fixture],
)
TestT3bT4TimePathologyProperty = T3bT4TimePathologyMachine.TestCase


# =====================================================================
# T-U-T3b-R1 — Telemetry pool overflow (wrap policy)
# =====================================================================

class TestT3bR1TelemetryPoolOverflow:
    """T-U-T3b-R1 — Documented behaviour: the telemetry pool *wraps*
    (modulo `pool_size`); when `path` is set, the logger flushes before
    overwrite so no data is lost.  See `TelemetryLogger.next_record`
    docstring + `_flush_batch` in `controller/telemetry.py`.

    Pass criterion (from plan): "Pool grows or wraps per documented
    policy."  Empirical: wraps with flush-before-overwrite semantics.
    """

    def test_t3b_r1a_pool_wraps_silently_when_no_path(self):
        """In-memory mode: `next_record()` wraps; `total_count` is the
        true count; `len(records)` caps at pool_size."""
        logger = TelemetryLogger(path=None, pool_size=10)
        for i in range(100):
            rec = logger.next_record()
            rec.time = i * 0.01
        assert logger.total_count == 100, logger.total_count
        recs = logger.records
        assert len(recs) == 10, (
            f"records property must cap at pool_size; got {len(recs)}"
        )
        # The 10 retained records are the most recent — times 0.90..0.99.
        # Use np.isclose because 0.9 in float is approximate.
        first_time = recs[0].time
        last_time = recs[-1].time
        assert np.isclose(first_time, 0.90), first_time
        assert np.isclose(last_time, 0.99), last_time

    def test_t3b_r1b_pool_flushes_before_overwrite_with_path(self):
        """With path set: each pool-wrap triggers a flush, so the disk
        receives every record (no data loss)."""
        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.csv', delete=False,
        ) as f:
            path = f.name
        try:
            logger = TelemetryLogger(path=path, pool_size=10)
            for i in range(35):
                rec = logger.next_record()
                rec.time = i * 0.01
            logger.flush()
            # CSV header + 35 data rows.
            with open(path) as fh:
                row_count = sum(1 for _ in fh) - 1
            assert row_count == 35, (
                f"flush-before-overwrite must persist all 35 records; "
                f"got {row_count} rows on disk"
            )
        finally:
            os.unlink(path)


# =====================================================================
# T-U-T3b-R2 — Event-queue overflow at S3 boundary
# =====================================================================

class TestT3bR2EventQueueOverflow:
    """T-U-T3b-R2 — Plan 1 Phase 3 enforces S3 (_validate_slot_capacity).
    This test verifies the enforcement still holds when callers stress the
    submit path with 3 back-to-back events.

    Pass criterion (from plan): "Plan 1 S3 raises."  Empirical on SHA
    `7d5e6da`: 3rd submit raises `ValueError` with "S3 violation in
    submit_event: both _current_event ... and _next_event ... are
    occupied."
    """

    def test_t3b_r2_third_submit_raises_s3(self):
        """Submit 3 events back-to-back; the 3rd must raise S3."""
        # Build a minimal scheduler.  cumulative_times comes from any
        # MPCParams instance — the values don't affect S3 enforcement.
        params = MPCParams()
        sched = EventScheduler(
            active_pose=REF_NORMAL,
            cumulative_times=params.cumulative_times,
            v_max_mmps=140.0, tau_s=0.04,
        )
        # Bootstrap one update tick so the scheduler exits IDLE init.
        sched.update(
            sim_time=0.0,
            current_pose=REF_NORMAL,
            current_twist=np.zeros(6),
        )

        # Fill slots 1 and 2 with two events.
        for k in range(2):
            ev = ScheduledEvent(
                event_id=k,
                event_type=EventType.CATCH,
                pose=REF_NORMAL.copy(),
                time=1.0 + k,
            )
            sched.submit_event(ev)

        # Third submit must raise S3.
        overflow_ev = ScheduledEvent(
            event_id=2, event_type=EventType.CATCH,
            pose=REF_NORMAL.copy(), time=10.0,
        )
        with pytest.raises(ValueError, match=r"S3 violation"):
            sched.submit_event(overflow_ev)


# =====================================================================
# T-U-T3b-R3 — File I/O failure on write_csv
# =====================================================================

class TestT3bR3FileIOFailure:
    """T-U-T3b-R3 — `TelemetryLogger._flush_batch` opens the CSV in
    append/write mode via `builtins.open`.  If the open raises
    (PermissionError, OSError, etc.), the error must propagate — silent
    swallow would cause telemetry to disappear without operator notice.

    Pass criterion (from plan): "Documented behaviour: IOError
    propagates."  Empirical on SHA `7d5e6da`: PermissionError propagates
    out of `flush()` cleanly.
    """

    def test_t3b_r3_permission_error_propagates(self):
        """Patch `builtins.open` to raise PermissionError on the CSV
        path; assert `flush()` re-raises (no silent swallow)."""
        real_open = builtins.open

        def raising_open(*args, **kwargs):
            if args and isinstance(args[0], str) \
                    and args[0].endswith('.csv'):
                raise PermissionError(
                    f"synthetic permission denied: {args[0]!r}"
                )
            return real_open(*args, **kwargs)

        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'probe.csv')
            logger = TelemetryLogger(path=path, pool_size=5)
            for i in range(3):
                rec = logger.next_record()
                rec.time = i * 0.01

            builtins.open = raising_open
            try:
                with pytest.raises(PermissionError,
                                   match=r"synthetic permission denied"):
                    logger.flush()
            finally:
                builtins.open = real_open


# =====================================================================
# T-U-T3b-H1 — on_target_override returning None  (XFAIL pre-bugfix)
# =====================================================================

class TestT3bH1TargetOverrideReturnsNone:
    """T-U-T3b-H1 — `on_target_override` returning None.

    Plan framing: "Documented: silently uses None (verify and surface
    gap if any)."

    **Real bug surfaced** at SHA `7d5e6da`: returning None from the hook
    crashes the runner at the very next line (`tc.target_pose`) with
    `AttributeError: 'NoneType' object has no attribute 'target_pose'`.
    The hook's docstring (`runner.py::MpcLoopHooks.on_target_override`)
    says "Return the original tc to keep it unchanged" — None is
    undocumented but a natural "no override needed" return shape.

    Pre-bugfix (this commit): test asserts the AttributeError raises
    via `xfail(strict=True)`.  Post-bugfix (same-session follow-up,
    `_PHASE_8_BUGFIX_LANDED = True`): runner treats None as "keep tc
    unchanged" and the loop completes normally.
    """

    @pytest.mark.xfail(
        not _PHASE_8_BUGFIX_LANDED,
        reason=("Surfaces runner.py gap: on_target_override returning "
                "None causes AttributeError on the next line "
                "(`tc.target_pose`).  Fixed in same-session bugfix — "
                "treat None as 'keep tc unchanged'."),
        strict=True,
        raises=AttributeError,
    )
    def test_t3b_h1_none_return_completes_loop(self, plant):
        """Hook returning None must NOT crash the loop.  Post-bugfix:
        loop completes; records logged."""
        mpc = _create_mpc(plant)
        source = StaticTargetSource(
            [(0.0, REF_NORMAL.copy())],
            v_max_mmps=140.0, tau_s=0.04,
        )
        logger = TelemetryLogger(pool_size=20)

        def hook_returns_none(state, tc):
            return None

        hooks = MpcLoopHooks(on_target_override=hook_returns_none)
        plant.reset()
        plant.step(0.025)

        # Pre-bugfix: AttributeError.  Post-bugfix: completes cleanly.
        estop = run_mpc_loop(plant, mpc, source,
                             duration=0.1, logger=logger, hooks=hooks)
        assert estop is False, f"estop_exit was {estop!r}"
        assert len(logger.records) > 0, (
            "post-bugfix: hook returning None must keep tc unchanged "
            "and the loop must log at least one record"
        )


# =====================================================================
# T-U-T3b-H2 — on_target_override raising
# =====================================================================

class TestT3bH2TargetOverrideRaises:
    """T-U-T3b-H2 — when `on_target_override` raises, the runner must
    propagate the exception (no swallow).  The loop terminates cleanly
    at the raise site; downstream phases of the same tick (solve,
    command, log) do not run.

    Pass criterion (from plan): "Exception propagates; loop terminates
    cleanly with clear logging."

    Empirical on SHA `7d5e6da`: RuntimeError raised inside the hook
    propagates out of `run_mpc_loop`.  The runner's `try/finally` block
    runs (GC tracker uninstall + gc.enable restore), so process-level
    state is left clean.
    """

    def test_t3b_h2_exception_propagates(self, plant):
        """Hook raising RuntimeError → loop re-raises; cleanup runs."""
        mpc = _create_mpc(plant)
        source = StaticTargetSource(
            [(0.0, REF_NORMAL.copy())],
            v_max_mmps=140.0, tau_s=0.04,
        )
        logger = TelemetryLogger(pool_size=20)
        gc_enabled_pre = gc.isenabled()

        def hook_raises(state, tc):
            raise RuntimeError("synthetic on_target_override failure (H2)")

        hooks = MpcLoopHooks(on_target_override=hook_raises)
        plant.reset()
        plant.step(0.025)

        with pytest.raises(RuntimeError,
                           match=r"synthetic on_target_override failure"):
            run_mpc_loop(plant, mpc, source,
                         duration=0.5, logger=logger, hooks=hooks)

        # Cleanup invariant: the finally block must restore gc state.
        # gc was enabled coming in; must still be enabled coming out.
        assert gc.isenabled() == gc_enabled_pre, (
            "runner finally must restore gc.isenabled() state"
        )


# =====================================================================
# T-U-T3b-H3 — on_pre_command raising
# =====================================================================

class TestT3bH3PreCommandRaises:
    """T-U-T3b-H3 — same propagation contract as H2, but for the
    `on_pre_command` hook (fires after solve, before plant.command).

    Pass criterion (from plan): "Same" (as H2).
    """

    def test_t3b_h3_exception_propagates(self, plant):
        """`on_pre_command` raising RuntimeError → loop re-raises."""
        mpc = _create_mpc(plant)
        source = StaticTargetSource(
            [(0.0, REF_NORMAL.copy())],
            v_max_mmps=140.0, tau_s=0.04,
        )
        logger = TelemetryLogger(pool_size=20)
        gc_enabled_pre = gc.isenabled()

        def hook_raises(plant, mpc, tc, cmd, cmd_vel, diag):
            raise RuntimeError("synthetic on_pre_command failure (H3)")

        hooks = MpcLoopHooks(on_pre_command=hook_raises)
        plant.reset()
        plant.step(0.025)

        with pytest.raises(RuntimeError,
                           match=r"synthetic on_pre_command failure"):
            run_mpc_loop(plant, mpc, source,
                         duration=0.5, logger=logger, hooks=hooks)

        assert gc.isenabled() == gc_enabled_pre


# =====================================================================
# T-U-T3b-H4 — on_post_solve allocating in hot loop (regression check)
# =====================================================================

class TestT3bH4PostSolveAllocation:
    """T-U-T3b-H4 — Verify that a small per-tick allocation inside the
    `on_post_solve` hook does not bust the hot-loop allocation contract.

    Pass criterion (from plan): "Hot-loop allocation contract still
    green (regression check)."

    **Chosen allocation size**: the hook appends one string reference
    (the `status` field of `diag`) to a per-test list.  Each append
    contributes ~30-60 bytes (amortised list resize + Python string
    interning for the canonical status `'Solve_Succeeded'`).  This is
    deliberately small — the contract's THRESHOLD_BYTES is 256, and the
    steady-state baseline is ~137 B/tick (per `hot_loop_contract.py`'s
    historical-ratchet comment), leaving ~119 B headroom.  A 30-60 B
    hook allocation fits inside that headroom.

    The test runs ``run_mpc_loop`` over the same window as
    ``test_hot_loop_allocation_contract`` (WARMUP=50, WINDOW=100 ticks)
    with the additional ``on_post_solve`` hook, and asserts the
    tracemalloc-measured allocation stays under THRESHOLD_BYTES.

    If THIS test starts failing while the baseline contract test stays
    green, the right reading is "the on_post_solve hook is consuming too
    much of the budget" — either tighten the test (smaller allocation)
    or document the new headroom.
    """

    # Use a small but observable allocation per tick.  Appending a single
    # status-string reference is ~30-50 B per call (list-growth +
    # string-ref slot).  Strings 'Solve_Succeeded' are interned so the
    # string itself is not re-allocated.
    def test_t3b_h4_on_post_solve_allocates_within_budget(self):
        # Match the baseline contract test's fixture exactly (so we are
        # measuring marginal-cost-of-hook, not new test setup overhead).
        from tests.sim.test_hot_loop_allocation_contract import (
            _build_fixture,
            _SnapshotHook,
            _EarlyExitPlantWrapper,
            _format_top_n_diagnostic,
            _total_bytes,
            _DURATION_S,
        )

        # Plan 2 Working Note #5 mitigation: a forward Gen-2 collect
        # drops the heap baseline left over from earlier hypothesis fuzz
        # runs (T-U-T3b-T4 in this module; the other Phase-N stateful
        # machines elsewhere in the suite).  Gen-0 (the default
        # `gc.collect()`) is not enough — empirically, after the T4
        # RuleBasedStateMachine completes, tracemalloc's baseline can
        # sit ~150 B/tick above the steady-state cold-run floor; the
        # Gen-2 sweep drops it back below THRESHOLD_BYTES=256.  See
        # `tests/sim/test_hot_loop_allocation_contract.py::
        # test_hot_loop_allocation_contract` (`gc.collect()` body) and
        # logbook/2026-05-11-tier1c-input-fuzz.md Discussion for the
        # precedent.
        gc.collect(2)
        plant, mpc, source, logger = _build_fixture()

        observed_statuses: list[str] = []

        def alloc_hook(tc, diag):
            # Append one string reference per tick (~30-50 B amortised).
            # Documented in the docstring above as the chosen allocation
            # size for this regression check.
            observed_statuses.append(diag.get('status', 'n/a'))

        snap_hook = _SnapshotHook()
        plant_wrapped = _EarlyExitPlantWrapper(plant, snap_hook)

        tracemalloc.start(HOT_LOOP_CONTRACT_TRACEBACK_FRAMES)
        try:
            run_mpc_loop(
                plant_wrapped, mpc, source,
                duration=_DURATION_S,
                logger=logger,
                control_dt=CONTROL_DT,
                hooks=MpcLoopHooks(
                    on_post_step=snap_hook,
                    on_post_solve=alloc_hook,
                ),
            )
        finally:
            tracemalloc.stop()

        assert snap_hook.snap_before is not None, (
            "snap_before not taken — warmup window did not complete"
        )
        assert snap_hook.snap_after is not None, (
            "snap_after not taken — measurement window did not complete"
        )

        bytes_diff = (_total_bytes(snap_hook.snap_after)
                      - _total_bytes(snap_hook.snap_before))
        bytes_per_tick = bytes_diff / HOT_LOOP_CONTRACT_WINDOW_TICKS

        # The contract budget is THRESHOLD_BYTES per tick.  This test
        # adds a small alloc on top of the baseline; the marginal-cost
        # is the per-tick string-ref append (~30-50 B).  The budget
        # tolerates this — assertion is the same `< THRESHOLD_BYTES`.
        assert bytes_per_tick < THRESHOLD_BYTES, (
            f"on_post_solve hook's per-tick allocation pushed the "
            f"hot-loop budget over THRESHOLD_BYTES={THRESHOLD_BYTES}.  "
            f"Measured: {bytes_per_tick:.1f} B/tick over "
            f"{HOT_LOOP_CONTRACT_WINDOW_TICKS} ticks.  Top growth "
            f"sites:\n"
            f"{_format_top_n_diagnostic(snap_hook.snap_before, snap_hook.snap_after, n=10)}"
        )

        # Sanity: the hook actually fired.
        assert len(observed_statuses) >= snap_hook.tick_count, (
            f"hook fired {len(observed_statuses)} times; "
            f"snap_hook saw {snap_hook.tick_count} ticks — mismatch"
        )


# =====================================================================
# T-U-T3b-R4 — ZmqTargetSource.reset() mid-APPROACHING
# =====================================================================

class TestT3bR4ZmqTargetSourceReset:
    """T-U-T3b-R4 — `ZmqTargetSource.reset()` clears source-internal
    cached events but does NOT touch the MPC's warm-start state
    (`_prev_w`, `_prev_lam_g`, etc.) — those live on the MPC instance,
    not the source.

    Pass criterion (from plan): "Documented behaviour; no warm-start
    corruption."

    Plan hint: "may surface a real bug — if the source reset() clears
    cached events while the MPC is mid-tick, _prev_w could be invalidated
    incorrectly."  Empirically on SHA `7d5e6da`: source.reset() touches
    only source-internal state; `_prev_w` is on the MPC and survives.
    The next solve() after reset() runs with the surviving warm-start and
    returns `Solve_Succeeded`.

    The "bug" the plan anticipated does not exist on this architecture —
    the source and MPC have clean separation of concerns (per the
    MPCController-owns-warm-start design).  This test pins that
    invariant: future refactors that mix source state into MPC warm-start
    must update this test.
    """

    def test_t3b_r4_reset_does_not_touch_mpc_warm_start(self, plant):
        """Seed the MPC; drive a target into the ZmqTargetSource (so the
        source has cached events to clear); call source.reset(); assert
        the source-internal state is cleared but mpc._prev_w is
        untouched.

        FakeIPC pattern (matches `test_zmq_target.py` and Phase 6
        boundary discipline): replaces the ZMQ socket layer so the test
        runs in-process without ZMQ context, while exercising the real
        `update()` / `reset()` code paths.
        """
        from unittest.mock import patch

        # FakeIPC drop-in (mirrors test_zmq_target.py::FakeIPC).
        class FakeIPC:
            def __init__(self, *args, **kwargs):
                self._queue: list[tuple[bytes, dict]] = []
                self.closed = False

            def recv_all(self):
                out, self._queue = self._queue, []
                return out

            def close(self):
                self.closed = True

            def enqueue(self, topic: bytes, msg: dict):
                self._queue.append((topic, msg))

        mpc = _create_mpc(plant)
        _seed_mpc(plant, mpc)
        prev_w_before = mpc._prev_w.copy()
        assert np.all(np.isfinite(prev_w_before))

        with patch('controller.zmq_target.MpcTargetIPC', FakeIPC):
            from controller.zmq_target import ZmqTargetSource
            from jugglebot.motion.ipc import TOPIC_MPC_MODE, TOPIC_MPC_TARGET

            src = ZmqTargetSource(
                default_z_mm=170.0,
                v_max_mmps=140.0,
                clamp_start_twist_mmps=140.0,
            )
            try:
                # Drive a target so the source caches events (this is
                # the "mid-APPROACHING" condition the plan calls out).
                src._ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'gui'})
                src._ipc.enqueue(TOPIC_MPC_TARGET, {
                    'target_pose': [0.0, 0.0, 220.0, 0.0, 0.0, 0.0],
                    'source': 'gui',
                })
                state = plant.get_state()
                src.poll()
                tc = src.update(sim_time=0.0, state=state)
                # Pre-reset: source has cached events.
                assert src._cached_events is not None
                assert src._has_target is True

                # Reset mid-APPROACHING.
                src.reset()

                # Source-internal fields cleared.
                assert src._cached_events is None
                assert src._has_target is False
                assert src._target_dirty is False
                assert src._arrival_time is None

                # MPC warm-start untouched (this is the load-bearing
                # property — the plan asked whether reset() could
                # corrupt _prev_w; empirically it cannot, because
                # warm-start state lives on the MPC, not the source).
                assert mpc._prev_w is not None, (
                    "MPC._prev_w was set to None — reset() lives on the "
                    "source, not the MPC; this would indicate a "
                    "regression where source has gained ownership of "
                    "MPC state."
                )
                assert np.array_equal(mpc._prev_w, prev_w_before), (
                    "MPC._prev_w changed after source.reset() — the "
                    "source has no handle on MPC warm-start state; "
                    "this would indicate aliasing between source and "
                    "MPC that should not exist."
                )
                assert np.all(np.isfinite(mpc._prev_w))

                # Sanity: the next solve() still works.
                state2 = plant.get_state()
                _, _, diag = mpc.solve(state2, REF_NORMAL)
                assert diag['status'] == 'Solve_Succeeded', diag['status']
            finally:
                src.close()


# =====================================================================
# T-U-T3b-R5 — Concurrent state read/write audit
# =====================================================================

class TestT3bR5ConcurrencyAudit:
    """T-U-T3b-R5 — Structural assertion: the MPC hot loop is
    single-threaded on the production path; the only concurrency
    boundary is the ZMQ socket layer (drained synchronously inside
    `plant.get_state()`).

    Plan hint: "likely no-op for this codebase since hot loop is
    single-threaded.  Document the audit even if no tests are added."

    Audit performed 2026-05-12 against SHA `7d5e6da`:
    * `run_mpc_loop` in `controller/runner.py` runs in the calling
      thread; no `threading.Thread` / `asyncio` / `multiprocessing`
      machinery.
    * `plant.get_state()` for `MuJoCoPlant` is in-process; for
      `HardwarePlant` it drains the ZMQ SUB socket synchronously (one
      thread, one socket).
    * `MPCController` state (`_prev_w`, `_consecutive_failures`, etc.)
      is mutated only from `solve()`, called only from the single
      runner-thread context.
    * `EventScheduler` state is single-threaded; bridge-node feed-in is
      ZMQ-buffered.

    This test is therefore a documentation assertion: scanning the
    hot-loop file confirms no thread spawn, no async, no shared
    mutex.  Future refactors that introduce concurrency MUST update
    this test (and the accompanying logbook entry).
    """

    def test_t3b_r5_hot_loop_is_single_threaded(self):
        """Static-source structural check: `controller/runner.py` does
        not import or use thread/async/process-pool machinery on the
        hot path."""
        import controller.runner as runner_mod

        src_path = runner_mod.__file__
        with open(src_path) as f:
            source = f.read()

        for banned in (
            'threading.Thread',
            'multiprocessing.Process',
            'concurrent.futures',
            'asyncio.run',
        ):
            assert banned not in source, (
                f"runner.py source contains {banned!r} — Phase 8 "
                f"R5 assumed the hot loop is single-threaded; if this "
                f"changes, update logbook + this test"
            )
