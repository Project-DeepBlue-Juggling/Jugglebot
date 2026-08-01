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

import gc
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


# `serial` — every test in this module measures a process-global resource
# (tracemalloc heap growth, GC events) whose baseline is set by whatever else has
# already run in the interpreter.  Under xdist that baseline becomes
# non-deterministic, so the whole module runs in the serial phase — see the
# `serial` marker definition in pyproject.toml and ./run_tests.sh.
#
# `nightly` — the MPC HARDWARE chain is operationally dormant
# (plans/active/refactor-2026-07.md Phase 3: jugglebot_launch.py no longer starts
# motor_guard/motion_bridge_node, and run_mpc.py is not launched; the leg path is
# trajectory_node -> teensy_bridge_node -> the Teensy MAX_DEVIATION guard).
#
# Be precise about what that does and does not mean, because the MPC revival's
# "promote these back" step will be judged against it: `run_mpc_loop` itself is
# NOT unreachable — sim/main.py:961 still calls it from `run_mpc_headless`, i.e.
# every `python sim/main.py --mpc --no-viewer` run, and run_mpc.py drives it if
# started by hand. What is dormant is every path where the loop's output reaches
# a motor. This contract measures per-tick allocation determinism, which is a
# REAL-TIME property: a GC pause costs a 40 Hz hardware loop a missed setpoint,
# and costs an offline sim run nothing at all. With no real-time consumer, the
# contract is parked with the deadline it protects — nightly via
# tools/nightly_suite.sh and on `./run_tests.sh --full`, which is mandatory
# before any hardware sitting and pre-commit for controller/ and sim/ changes.
# Promotion back to per-commit is step 4 of the MPC revival.
#
# BOTH marks matter and the gate composition depends on it: `serial` alone would
# leave these in the default serial phase; `nightly` alone would let them run
# under xdist on `--full` and corrupt their own baseline.  ./run_tests.sh's
# phase filters are `not serial and not nightly` / `serial and not nightly`
# (default) and `not serial` / `serial` (--full), so a serial+nightly test lands
# in exactly one bucket per invocation.
pytestmark = [pytest.mark.serial, pytest.mark.nightly]


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
    """Exclude tracemalloc-internal, importlib, and non-hot-loop module noise.

    Without these filters, the top-10 diagnostic is dominated by pytest
    plugin imports and tracemalloc's own trace-table growth, and the
    total byte count can be polluted by module-level imports that a
    sibling test dragged into the process.

    The excluded paths and their rationale:

    * ``tracemalloc.__file__`` / ``<frozen importlib._bootstrap{,_external}>``
      — the interpreter's own per-snapshot state.
    * ``*/sim/analysis/*`` — ``analyse_oscillation`` and its siblings are
      post-hoc analysis utilities, never imported from the 40 Hz hot
      loop.  A test that loads them (e.g. ``test_diagnose_oscillation``)
      pulls matplotlib in transitively via ``plot_diagnosis.py``, which
      then sits in ``sys.modules`` for the rest of the pytest session.
      Matplotlib's internal caches grow lazily on use, so a subsequent
      contract test could inherit tens of KB of unrelated allocation
      if those caches warm mid-run.
    * ``*/sim/viz/*`` — same concern, different path (dashboard /
      plotting helpers that aren't hot-path).
    * ``*/matplotlib/*`` — belt-and-braces, covers any future sibling
      test that imports matplotlib directly.

    The list is deliberately NOT a positive allow-list (e.g.  "only
    count ``controller/`` allocations") — the contract's intent is
    "any application allocation on the hot-loop body", and a hot-loop
    leak originating from ``run_mpc.py`` or ``ros_ws/.../motion/ipc.py``
    must still trip the test.  The negative-list approach keeps the
    contract open-by-default.
    """
    return (
        tracemalloc.Filter(False, tracemalloc.__file__),
        tracemalloc.Filter(False, '<frozen importlib._bootstrap>'),
        tracemalloc.Filter(False, '<frozen importlib._bootstrap_external>'),
        tracemalloc.Filter(False, '*/sim/analysis/*'),
        tracemalloc.Filter(False, '*/sim/viz/*'),
        tracemalloc.Filter(False, '*/matplotlib/*'),
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

    Plan 2 Working Note #5: hypothesis fuzz suites earlier in the run
    (Plan 2 Phase 2 introduced the first stateful machine, Phase 3 adds
    NaN/Inf input fuzz) leave the heap in a state that occasionally
    pushes ``tracemalloc``'s baseline above the per-tick threshold even
    though the loop body itself allocates well under it.  A single
    ``gc.collect()`` here drops that baseline before snapshotting and
    restores deterministic behaviour at ci-deep without changing the
    contract under test.  See
    [logbook/2026-05-11-tier1c-input-fuzz.md](../../logbook/2026-05-11-tier1c-input-fuzz.md)
    Discussion for the rationale.
    """
    gc.collect()
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


# ---------------------------------------------------------------------------
# Hardware-plant variant.
#
# The MuJoCo-path test above cannot see allocations on HardwarePlant's hot
# path.  Per the W7 audit (Pass 2 #3), 5 per-tick allocation sites in
# HardwarePlant were invisible to the MuJoCo fixture.  This variant patches
# ZMQ, pumps synthetic telemetry, wires the production `_on_pre_command`
# hook (so `plant.set_pose()` is exercised), and re-runs the same
# tracemalloc bracketing against HardwarePlant.
#
# Shares ``THRESHOLD_BYTES`` with the MuJoCo case — same contract.
# ---------------------------------------------------------------------------


def _build_hardware_fixture():
    """Hardware-plant contract fixture: patched ZMQ, synthetic telemetry.

    Returns (plant, mpc, source, logger) mirroring the MuJoCo fixture's
    shape so the test body can share the same snapshot-bracketing helpers.

    Design notes:
        * ZMQ PUB / SUB and ``time.sleep`` are mocked only during plant
          construction; after init we swap in a lambda for
          ``plant._pub.send_multipart`` (so MagicMock's call_args_list
          doesn't grow each tick) and a tiny callable pump for
          ``plant._sub.recv_multipart`` that alternates frame / Again.
        * Synthetic telemetry reports motor positions corresponding to the
          target pose, so the MPC sees a steady-state plant at its goal
          and settles into a regular command pattern within the warmup
          window.
        * FK, jacobian, and rot_matrix conversions run real — not stubbed
          — so the measured allocation profile matches production.
    """
    import zmq as _zmq_real
    import msgpack as _msgpack
    from unittest.mock import MagicMock, patch

    # Pre-compute motor positions at TARGET_POSE via MuJoCoPlant's IK so
    # the synthetic telemetry represents a plant already at its goal.
    # Done once at fixture build, outside the measurement window.
    from plant.mujoco_plant import MuJoCoPlant as _SimPlant
    _sim_tmp = _SimPlant()
    target_ext_mm = _sim_tmp.pose_to_extensions(TARGET_POSE)
    target_motor_rev = target_ext_mm * _sim_tmp.geom.mm_to_rev
    del _sim_tmp

    telem_dict = {
        'motor_pos': [float(v) for v in target_motor_rev],
        'motor_vel': [0.0] * 6,
    }
    telem_payload = _msgpack.packb(telem_dict, use_bin_type=True)
    telem_frame = [b'telemetry', telem_payload]

    class _FramePump:
        """recv_multipart replacement: yields one frame, then raises Again,
        alternating forever.  One frame per get_state() call — the drain
        loop in get_state reads one frame and breaks on the next Again.
        """
        __slots__ = ('_frame', '_yield_frame')

        def __init__(self, frame):
            self._frame = frame
            self._yield_frame = True

        def __call__(self, *args, **kwargs):
            if self._yield_frame:
                self._yield_frame = False
                return self._frame
            self._yield_frame = True
            raise _zmq_real.Again()

    pump = _FramePump(telem_frame)

    with patch('controller.hardware_plant.zmq') as mock_zmq, \
         patch('controller.hardware_plant.time.sleep'):
        mock_ctx = MagicMock()
        mock_pub = MagicMock()
        mock_sub = MagicMock()
        mock_ctx.socket.side_effect = [mock_pub, mock_sub]
        mock_zmq.Context.return_value = mock_ctx
        mock_zmq.Again = _zmq_real.Again
        mock_zmq.NOBLOCK = 0

        from controller.hardware_plant import HardwarePlant
        plant = HardwarePlant(control_dt=CONTROL_DT)

    # Replace MagicMock hot methods with plain callables.  MagicMock
    # records every call in ``call_args_list``, whose backing list grows
    # per tick — that growth would show up as per-tick allocation in
    # tracemalloc even though none of it is in the plant's hot path.
    plant._pub.send_multipart = lambda *a, **k: None
    plant._sub.recv_multipart = pump

    params = MPCParams(
        max_cpu_time=2.0,          # sim: generous IPOPT budget
        max_iter=500,
        max_leg_vel_mmps=280.0,
        prime_solver=False,        # cold-start cost absorbed by warmup
    )
    mpc = MPCController.from_plant(params, plant)
    source = StaticTargetSource(
        schedule=[(0.0, TARGET_POSE)],
        v_max_mmps=params.max_leg_vel_mmps,
        tau_s=params.tau,
    )
    # pool_size tuned to the warmup window so the pool wraps exactly
    # once before measurement begins (see MuJoCo fixture for rationale).
    logger = TelemetryLogger(pool_size=HOT_LOOP_CONTRACT_WARMUP_TICKS)
    return plant, mpc, source, logger


def _production_hooks_for_hardware(snap_hook: _SnapshotHook) -> MpcLoopHooks:
    """Import the production hooks from ``run_mpc.py`` exactly.

    Previously this replicated the hook bodies locally, which left a
    coverage gap — the W4d augmented-assign closure bug (fix 149070d)
    would not have been caught by this test because the test used its
    own (correctly-written) copy of the hook while the production
    ``run_mpc.py`` hook could silently regress.  Post-refactor the
    ``_on_pre_command`` closure lives in
    ``controller/hardware_hooks.py::make_feedforward_pre_command_hook``
    (module-level factory, importable).  This test exercises the same
    callable ``run_mpc.py`` wires in, so any regression in the
    production hook trips CI at commit time.

    The ``_on_log_extras`` hook is still lightweight enough to keep
    locally.
    """
    from types import SimpleNamespace
    from controller.hardware_hooks import make_feedforward_pre_command_hook

    _on_pre_command = make_feedforward_pre_command_hook()

    _log_extras_ns = SimpleNamespace(fk_iterations=0, ff_torque_max_Nm=0.0)

    def _on_log_extras(plant_):
        _log_extras_ns.fk_iterations = getattr(
            plant_, 'last_fk_iterations', 0)
        _log_extras_ns.ff_torque_max_Nm = getattr(
            plant_, 'last_ff_torque_max_Nm', 0.0)
        return _log_extras_ns

    return MpcLoopHooks(
        on_pre_command=_on_pre_command,
        on_log_extras=_on_log_extras,
        on_post_step=snap_hook,
    )


def test_hot_loop_allocation_contract_hardware():
    """The MPC 40 Hz hot loop MUST allocate < THRESHOLD_BYTES per tick
    against HardwarePlant as well as MuJoCoPlant.

    Mirrors ``test_hot_loop_allocation_contract`` but runs against
    HardwarePlant with patched ZMQ, synthetic telemetry, and the
    production ``_on_pre_command`` hook from run_mpc.py wired so
    ``plant.set_pose()`` is exercised.

    Plan 2 Working Note #5 mitigation — see the MuJoCo variant above
    for rationale.  Same one-line ``gc.collect()`` baseline reset.
    """
    gc.collect()
    plant, mpc, source, logger = _build_hardware_fixture()

    hook = _SnapshotHook()
    plant_wrapped = _EarlyExitPlantWrapper(plant, hook)
    hooks = _production_hooks_for_hardware(hook)

    tracemalloc.start(HOT_LOOP_CONTRACT_TRACEBACK_FRAMES)
    try:
        run_mpc_loop(
            plant_wrapped, mpc, source,
            duration=_DURATION_S,
            logger=logger,
            control_dt=CONTROL_DT,
            hooks=hooks,
        )
    finally:
        tracemalloc.stop()

    assert hook.snap_before is not None, (
        f"S1 snapshot never taken (warmup={HOT_LOOP_CONTRACT_WARMUP_TICKS} "
        f"ticks, reached tick {hook.tick_count}) — loop exited before warmup"
    )
    assert hook.snap_after is not None, (
        f"S2 snapshot never taken (total={_TOTAL_TICKS} ticks, reached tick "
        f"{hook.tick_count}) — loop exited before measurement window"
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
            f"Hot loop (HardwarePlant path) allocated {bytes_per_tick:.0f} "
            f"B/tick (threshold: {THRESHOLD_BYTES} B/tick).\n"
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
