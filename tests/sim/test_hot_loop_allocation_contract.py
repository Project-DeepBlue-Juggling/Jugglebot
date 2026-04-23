"""W3 — enforcement test for the MPC 40 Hz hot-loop zero-allocation contract.

This test is the CI guard for ``controller/HOT_LOOP_CONTRACT.md``.  It
drives ``run_mpc_loop`` against a ``MuJoCoPlant`` + ``StaticTargetSource``
fixture for 200 ticks, takes ``tracemalloc`` snapshots at tick 50 and
tick 150 (100-tick measurement window after 50-tick warmup), and fails
when the per-tick Python allocation rate exceeds ``THRESHOLD_BYTES``
(see ``controller/hot_loop_contract.py``).

On failure, the test emits the top-10 allocation sites from
``snapshot.compare_to(...)`` so the offending call site is visible in
CI logs without having to rerun locally.

### Why ``@pytest.mark.xfail`` during W4

The test is introduced at the end of W3, BEFORE the W4 pre-allocation
fixes land.  At that point the measured allocation rate is ~19 KB/tick
against a 1024 B/tick threshold — the test is EXPECTED to fail.  The
xfail marker lets W4a–W4d land incrementally (each reducing allocations
by some amount) without making the whole test suite red.  W4e lifts the
marker once the contract passes under the real threshold.

### Design notes

* The test uses ``run_mpc_loop`` directly (not a replica), so any future
  code added to the hot-loop body is automatically in scope.
* Snapshotting uses the ``on_post_step`` hook.  Taking snapshots mid-
  tick is acceptable because tracemalloc excludes its own allocations
  from the traced set; symmetric placement of S1 and S2 (same relative
  point in both ticks) cancels any residual overhead in the delta.
* Filtering excludes tracemalloc-internal frames and the
  ``<frozen importlib._bootstrap>`` noise so the top-10 diagnostic lists
  genuine hot-path sites.
* ``prime_solver=False`` — the first-tick cold-start cost is paid inside
  the warmup window.  Priming would shift the cost out of the loop but
  wouldn't change what the contract measures.
* ``control_dt=0.025`` is the real 40 Hz period.  ``run_mpc_loop``'s
  wall-clock pacing means the test takes ~5 s — acceptable for a CI
  test of this weight.
"""

from __future__ import annotations

import tracemalloc
from typing import Any

import numpy as np
import pytest

from plant.mujoco_plant import MuJoCoPlant
from controller.hot_loop_contract import (
    THRESHOLD_BYTES,
    HOT_LOOP_CONTRACT_WINDOW_TICKS,
    HOT_LOOP_CONTRACT_WARMUP_TICKS,
    HOT_LOOP_CONTRACT_TRACEBACK_FRAMES,
)
from controller.mpc import MPCController
from controller.params import MPCParams
from controller.plant import PlantState
from controller.runner import MpcLoopHooks, run_mpc_loop
from controller.target import StaticTargetSource
from controller.telemetry import TelemetryLogger


CONTROL_DT = 0.025
TARGET_POSE = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])  # default Active-like hold

# Total ticks = warmup + measurement window.  Any extra run-in after the
# window is irrelevant; we stop the loop as soon as S2 is taken.
_TOTAL_TICKS = HOT_LOOP_CONTRACT_WARMUP_TICKS + HOT_LOOP_CONTRACT_WINDOW_TICKS

# Duration passed to run_mpc_loop.  We add a small pad because the loop's
# `n_steps = int(duration / control_dt)` can truncate.  The hook stops
# measurement at the exact tick count; extra ticks beyond that are just
# unused.
_DURATION_S = (_TOTAL_TICKS + 2) * CONTROL_DT


def _build_fixture():
    """Build the sim fixture the contract test runs against.

    Kept as a helper (not a pytest fixture) so the diagnostic printout on
    failure can show the exact setup a local debugger would need.
    """
    plant = MuJoCoPlant()
    params = MPCParams(
        max_cpu_time=2.0,          # sim: generous IPOPT budget
        max_iter=500,
        max_leg_vel_mmps=280.0,    # mirrors test_mpc_static.py baseline
        prime_solver=False,        # cold-start cost absorbed by warmup window
    )
    mpc = MPCController.from_plant(params, plant)
    source = StaticTargetSource(
        schedule=[(0.0, TARGET_POSE)],
        v_max_mmps=params.max_leg_vel_mmps,
        tau_s=params.tau,
    )
    # pool_size tuned to the warmup window so the pool wraps exactly
    # once before measurement begins.  Without this, the measurement
    # window would see fresh-slot growth (new Python floats per
    # assignment with no matching free), inflating the delta.  See
    # ``controller/HOT_LOOP_CONTRACT.md`` for the "pool must wrap before
    # the window opens" rationale.
    logger = TelemetryLogger(pool_size=HOT_LOOP_CONTRACT_WARMUP_TICKS)
    return plant, mpc, source, logger


def _snapshot_filters() -> tuple[tracemalloc.Filter, ...]:
    """Exclude tracemalloc-internal and importlib noise from the snapshot.

    Without these filters, the top-10 diagnostic is dominated by pytest
    plugin imports and tracemalloc's own trace-table growth.
    """
    return (
        tracemalloc.Filter(False, tracemalloc.__file__),
        tracemalloc.Filter(False, '<frozen importlib._bootstrap>'),
        tracemalloc.Filter(False, '<frozen importlib._bootstrap_external>'),
    )


def _total_bytes(snapshot: tracemalloc.Snapshot) -> int:
    """Sum the sizes of every trace in the snapshot."""
    return sum(stat.size for stat in snapshot.statistics('filename'))


def _format_top_n_diagnostic(
    s_before: tracemalloc.Snapshot,
    s_after: tracemalloc.Snapshot,
    n: int = 10,
) -> str:
    """Format the top-N growth sites for the failure message.

    Uses ``compare_to`` on 'lineno' grouping so each entry is tied to a
    specific ``file.py:lineno``.  Includes the traceback (one frame) so
    the failing contributor can navigate directly to the call site.
    """
    diffs = s_after.compare_to(s_before, 'lineno')
    # Sort by bytes added (descending), then by count added.
    diffs = sorted(diffs, key=lambda d: (-d.size_diff, -d.count_diff))
    top = diffs[:n]
    lines = [
        f"Top {len(top)} allocation sites (measured over "
        f"{HOT_LOOP_CONTRACT_WINDOW_TICKS} ticks):",
        f"{'#':>3}  {'bytes/tick':>11}  {'count/tick':>11}  site",
        f"{'-' * 3}  {'-' * 11}  {'-' * 11}  {'-' * 60}",
    ]
    for idx, stat in enumerate(top, start=1):
        bytes_per_tick = stat.size_diff / HOT_LOOP_CONTRACT_WINDOW_TICKS
        count_per_tick = stat.count_diff / HOT_LOOP_CONTRACT_WINDOW_TICKS
        # stat.traceback is a Traceback; formatting as a list of frames.
        frames = stat.traceback.format()
        # First frame of the traceback (innermost) is the file:lineno.
        # Use the last entry of format() because tracemalloc reverses
        # frame order: most-recent-call-last is at the end.
        site = frames[-1].strip() if frames else '<no traceback>'
        lines.append(
            f"{idx:>3}  {bytes_per_tick:>+10.1f}  "
            f"{count_per_tick:>+10.2f}  {site}"
        )
    return "\n".join(lines)


class _SnapshotHook:
    """State container for the tick-counting ``on_post_step`` callback.

    Can't be a closure because we need to read ``snap_before`` /
    ``snap_after`` after the loop returns.
    """

    def __init__(self) -> None:
        self.tick_count: int = 0
        self.snap_before: tracemalloc.Snapshot | None = None
        self.snap_after: tracemalloc.Snapshot | None = None
        # Used to short-circuit the loop once measurement is complete.
        # Set the plant's estop_requested attr through this flag.
        self.done: bool = False

    def __call__(self, state: PlantState, sim_time: float) -> None:
        self.tick_count += 1
        # Take S1 at end of warmup tick (tick number == warmup count),
        # S2 at end of warmup + window tick.
        if self.tick_count == HOT_LOOP_CONTRACT_WARMUP_TICKS:
            self.snap_before = tracemalloc.take_snapshot().filter_traces(
                _snapshot_filters()
            )
        elif self.tick_count == _TOTAL_TICKS:
            self.snap_after = tracemalloc.take_snapshot().filter_traces(
                _snapshot_filters()
            )
            self.done = True


class _EarlyExitPlantWrapper:
    """Wraps a PlantInterface, exposing ``estop_requested`` that flips
    once the measurement hook says it's done.

    ``run_mpc_loop`` checks ``getattr(plant, 'estop_requested', False)``
    at the top of every iteration and exits cleanly when it trips.
    Using this instead of ``source.done`` because the lifecycle semantics
    of a hardware-style estop match what we want here — immediate,
    non-lossy exit at a tick boundary.
    """

    def __init__(self, inner: Any, hook: _SnapshotHook) -> None:
        self._inner = inner
        self._hook = hook

    def __getattr__(self, name: str) -> Any:
        return getattr(self._inner, name)

    @property
    def estop_requested(self) -> bool:
        return self._hook.done


def test_hot_loop_allocation_contract():
    """The MPC 40 Hz hot loop MUST allocate < THRESHOLD_BYTES per tick.

    Defined by ``controller/HOT_LOOP_CONTRACT.md``.  The body of
    ``run_mpc_loop`` — from ``plant.get_state()`` through the last line
    of ``log_mpc_step`` — is measured via ``tracemalloc`` over a
    100-tick window after a 50-tick warmup.
    """
    plant, mpc, source, logger = _build_fixture()

    hook = _SnapshotHook()
    plant_wrapped = _EarlyExitPlantWrapper(plant, hook)

    tracemalloc.start(HOT_LOOP_CONTRACT_TRACEBACK_FRAMES)
    try:
        run_mpc_loop(
            plant_wrapped, mpc, source,
            duration=_DURATION_S,
            logger=logger,
            control_dt=CONTROL_DT,
            hooks=MpcLoopHooks(on_post_step=hook),
        )
    finally:
        tracemalloc.stop()

    assert hook.snap_before is not None, (
        f"S1 snapshot never taken (warmup={HOT_LOOP_CONTRACT_WARMUP_TICKS} "
        f"ticks, reached tick {hook.tick_count}) — loop exited before warmup "
        f"completed"
    )
    assert hook.snap_after is not None, (
        f"S2 snapshot never taken (total={_TOTAL_TICKS} ticks, reached tick "
        f"{hook.tick_count}) — loop exited before measurement window "
        f"completed"
    )

    total_before = _total_bytes(hook.snap_before)
    total_after = _total_bytes(hook.snap_after)
    total_delta = total_after - total_before
    bytes_per_tick = total_delta / HOT_LOOP_CONTRACT_WINDOW_TICKS

    if bytes_per_tick >= THRESHOLD_BYTES:
        diagnostic = _format_top_n_diagnostic(
            hook.snap_before, hook.snap_after, n=10,
        )
        pytest.fail(
            f"Hot loop allocated {bytes_per_tick:.0f} B/tick "
            f"(threshold: {THRESHOLD_BYTES} B/tick).\n"
            f"Total delta over {HOT_LOOP_CONTRACT_WINDOW_TICKS} ticks: "
            f"{total_delta:,} bytes.\n\n"
            f"{diagnostic}\n\n"
            f"See controller/HOT_LOOP_CONTRACT.md for the invariant and "
            f"implementation guidance."
        )


def test_no_in_tick_gc_events_under_w5_disable():
    """Under W5's gc.disable() wrapper, no GC event lands inside any tick.

    Companion to the allocation-budget contract: even if some future
    code path retains a reference cycle, Python's GC must not be able to
    sweep it mid-tick and violate the 25 ms budget.  W5 guarantees this
    by calling ``gc.disable()`` at loop entry and scheduling collections
    manually on the idle-sleep window.

    The assertion here reads each StepRecord's ``gc_ms`` field (which
    the runner fills from the per-tick ``_GCTracker.drain_since``) and
    fails if any is non-zero.  The runner's own ``W5 VIOLATION ...``
    stdout print catches the same condition at runtime; this test
    surfaces it in CI.

    Also asserts ``gc.isenabled()`` state is restored after the loop
    exits — verifies the ``finally`` branch of the gc.enable/disable
    bracket works correctly.
    """
    import gc
    assert gc.isenabled(), (
        "Pre-test: GC should be enabled before run_mpc_loop runs.  "
        "Another test leaked gc.disable()?"
    )

    plant, mpc, source, logger = _build_fixture()
    # Short run — 100 ticks ≈ 2.5 s wall-clock.  Well below the scheduled
    # Gen-2 collect cadence (``_GC_COLLECT_EVERY_N_TICKS = 1200`` in
    # ``controller/runner.py``, bumped in 8cab8fe after W6 hardware
    # validation), so any gc event recorded in-tick is an unambiguous
    # violation.  If that cadence is ever lowered to <=100, update this
    # test accordingly.
    _TEST_TICKS = 100

    class _TickCounter:
        def __init__(self):
            self.tick_count = 0
            self.done = False
        def __call__(self, state, sim_time):
            self.tick_count += 1
            if self.tick_count == _TEST_TICKS:
                self.done = True

    hook = _TickCounter()
    plant_wrapped = _EarlyExitPlantWrapper(plant, hook)

    run_mpc_loop(
        plant_wrapped, mpc, source,
        duration=(_TEST_TICKS + 2) * CONTROL_DT,
        logger=logger,
        control_dt=CONTROL_DT,
        hooks=MpcLoopHooks(on_post_step=hook),
    )

    assert gc.isenabled(), (
        "Post-test: gc.isenabled() should be restored to True by the "
        "finally block in run_mpc_loop.  W5 failed to re-enable GC."
    )

    records = logger.records
    assert len(records) > 0, "Logger captured no records — loop exited too early?"

    # Skip the first few ticks (JIT warmup can leave pending gc events
    # from before the disable took effect).  Measurement window starts
    # after tick 5.
    violations = [
        (i, r.gc_ms, r.time) for i, r in enumerate(records)
        if i >= 5 and r.gc_ms > 0.0
    ]
    if violations:
        pytest.fail(
            f"W5 violation: {len(violations)} ticks had non-zero gc_ms "
            f"(gc.disable() should have prevented this).\n"
            f"First few violations: {violations[:5]}\n\n"
            f"See controller/HOT_LOOP_CONTRACT.md, Enforcement section."
        )
