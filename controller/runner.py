"""Generic MPC control loop — shared by simulation and hardware entry points.

Provides :func:`run_mpc_loop`, a wall-clock-paced 40 Hz loop that:
    1. Reads plant state
    2. Queries the target source
    3. Solves the MPC
    4. Commands the plant
    5. Logs telemetry

Sim-specific behavior (ball spawning, hand commands, ball capture) and
hardware-specific behavior (feedforward torques, stale-telemetry override,
target feedback) are injected via the :class:`MpcLoopHooks` dataclass.
"""

from __future__ import annotations

import gc as _gc
import time as _time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

import numpy as np

from types import SimpleNamespace

from .plant import PlantInterface, PlantState
from .target import TargetCommand, TargetSource, pose_6dof_from_state
from .telemetry import TelemetryLogger, fill_record_from_arrays, record_from_arrays


# ---------------------------------------------------------------------------
# GC instrumentation — attribute Python garbage-collection pauses to MPC ticks
# ---------------------------------------------------------------------------

class _GCTracker:
    """Accumulates GC pause durations via gc.callbacks, bucketed per tick.

    Python's generational GC can pause the interpreter for tens of ms on a
    Jetson when a generation collection runs.  The MPC loop cannot control
    when GC fires, so attributing pauses to ticks requires wall-clock
    bracketing: the callback records 'start' on phase=='start' and 'stop'
    on phase=='stop', yielding (gen, elapsed_ms) events.  The loop then
    sums events whose stop-time falls inside the tick window.

    Overhead per callback invocation is ~1 μs — negligible compared to the
    multi-ms pauses being measured.
    """

    def __init__(self):
        self._starts: list[float] = []  # stack, one entry per active GC phase
        self._events: list[tuple[int, float, float]] = []  # (gen, stop_time, elapsed_ms)

    def _callback(self, phase: str, info: dict) -> None:
        if phase == 'start':
            self._starts.append(_time.perf_counter())
        elif phase == 'stop' and self._starts:
            t_start = self._starts.pop()
            t_stop = _time.perf_counter()
            self._events.append((info.get('generation', -1),
                                 t_stop,
                                 (t_stop - t_start) * 1000.0))

    def install(self) -> None:
        _gc.callbacks.append(self._callback)

    def uninstall(self) -> None:
        try:
            _gc.callbacks.remove(self._callback)
        except ValueError:
            pass

    def drain_since(self, t_window_start: float) -> tuple[float, int, int]:
        """Return (total_ms, count, max_gen) for GC events since t_window_start.

        Drains the event buffer so subsequent ticks start clean.  Events whose
        stop time predates the window are simply discarded (they belong to
        earlier ticks that already consumed them).
        """
        events, self._events = self._events, []
        total_ms = 0.0
        count = 0
        max_gen = -1
        for gen, t_stop, elapsed_ms in events:
            if t_stop >= t_window_start:
                total_ms += elapsed_ms
                count += 1
                if gen > max_gen:
                    max_gen = gen
        return total_ms, count, max_gen


# ---------------------------------------------------------------------------
# Hooks for caller-specific behavior
# ---------------------------------------------------------------------------

@dataclass
class MpcLoopHooks:
    """Extension points for sim-specific or hardware-specific behavior.

    All callbacks are optional (None = no-op).  The loop calls them at the
    appropriate phase of each iteration.
    """

    on_target_override: Callable[[PlantState, TargetCommand], TargetCommand] | None = None
    """Called after source.update().  May return a replacement TargetCommand
    (e.g. hold-in-place on stale telemetry).  Return the original tc to
    keep it unchanged."""

    on_pre_command: Callable[[PlantInterface, Any, TargetCommand, np.ndarray, np.ndarray, dict], None] | None = None
    """Called after MPC solve, before plant.command().  Receives
    (plant, mpc, tc, cmd, cmd_vel, diag).  Use for feedforward setup
    (e.g. plant.set_pose with predicted twist/accel)."""

    on_post_solve: Callable[[TargetCommand, dict], None] | None = None
    """Called after MPC solve.  Receives (tc, diag).  Use for target
    feedback publishing (accept/reject for catch coordinator)."""

    on_post_step: Callable[[PlantState, float], None] | None = None
    """Called after plant.step().  Receives (state, sim_time).  Use for
    ball capture detection (sim) or other per-step side effects."""

    on_log_extras: Callable[[PlantInterface], dict] | None = None
    """Called before logging.  Returns a dict of extra keyword arguments
    for record_from_arrays (e.g. hand_cmd_mm, fk_iterations, ff_torque_max_Nm).
    The dict keys must match record_from_arrays parameter names."""


# ---------------------------------------------------------------------------
# MPC solve helper
# ---------------------------------------------------------------------------

def mpc_solve(mpc, state: PlantState, tc: TargetCommand,
              t_now: float | None = None):
    """Call mpc.solve() with the target from a TargetCommand.

    Returns (cmd, cmd_vel, diag, ref_pose, ref_twist).

    ref_pose and ref_twist come from the MPC's own node-0 reference
    (quintic Hermite between events).  This ensures telemetry tracking
    error is measured against the reference the MPC actually optimized
    against.

    ``t_now`` overrides ``state.time`` for reference event sampling.
    ``None`` uses ``state.time``; the main loop passes the frozen ``t_ref``
    so the reference does not run away during fallback chains.
    """
    cmd, cmd_vel, diag = mpc.solve(
        state, tc.target_pose,
        ref_events=tc.ref_events,
        boost_vel_weights=tc.boost_vel_weights,
        t_now=t_now,
        warm_start_valid=tc.warm_start_valid,
    )

    # Use the MPC's own reference for telemetry.
    mpc_ref = mpc.last_ref_traj
    mpc_twist = mpc.last_twist_traj
    if mpc_ref is not None:
        ref_pose = mpc_ref[0]
        ref_twist = mpc_twist[0] if mpc_twist is not None else np.zeros(6)
    else:
        ref_pose = tc.target_pose
        ref_twist = np.zeros(6)

    return cmd, cmd_vel, diag, ref_pose, ref_twist


# ---------------------------------------------------------------------------
# Telemetry helpers
# ---------------------------------------------------------------------------

_LOG_ZERO6 = np.zeros(6)


def log_mpc_step(logger: TelemetryLogger, state: PlantState,
                 ref_pose: np.ndarray, cmd_ext: np.ndarray,
                 diag: dict, ref_twist: np.ndarray | None = None,
                 dashboard=None, t_ref_s: float = 0.0,
                 dashboard_time_sink: list[float] | None = None,
                 **extras) -> None:
    """Record one telemetry step (MPC mode) — hot-loop body.

    Uses the pool-recycle path: ``logger.next_record()`` returns a
    pre-allocated slot, ``fill_record_from_arrays`` mutates it in place.

    ``dashboard_time_sink``: optional one-element list receiving the
    dashboard broadcast duration in ms, so callers can subtract it from
    t_log_ms and attribute the two costs separately.  Leave None to skip.
    """
    record = logger.next_record()
    fill_record_from_arrays(
        record,
        time=state.time,
        ref_pose=ref_pose,
        ref_twist=ref_twist if ref_twist is not None else _LOG_ZERO6,
        actual_pose=pose_6dof_from_state(state),
        actual_twist=state.platform_twist,
        cmd_extensions=cmd_ext,
        actual_extensions=state.leg_extensions_mm,
        leg_velocities=state.leg_velocities_mmps,
        hand_pos_mm=state.hand_pos_mm if state.hand_pos_mm is not None else 0.0,
        hand_vel_mmps=state.hand_vel_mmps if state.hand_vel_mmps is not None else 0.0,
        solve_time_ms=diag.get('solve_time_ms', 0.0),
        solve_status=diag.get('status', 'n/a'),
        cost=diag.get('cost', 0.0),
        constraint_violation=diag.get('constraint_violation', 0.0),
        ipopt_iter=diag.get('iter_count', 0),
        t_ref_s=t_ref_s,
        **extras,
    )
    if dashboard is not None:
        t_dash0 = _time.perf_counter()
        dashboard.broadcast(record)
        if dashboard_time_sink is not None:
            dashboard_time_sink[0] = (_time.perf_counter() - t_dash0) * 1000.0


def print_mpc_summary(logger: TelemetryLogger) -> None:
    """Print summary statistics from an MPC run."""
    if not logger.records:
        return
    final = logger.records[-1]
    solve_times = [r.solve_time_ms for r in logger.records if r.solve_time_ms > 0]
    print(f"Final tracking error: {final.tracking_error_mm:.3f} mm, "
          f"{final.tracking_error_deg:.4f} deg")
    if solve_times:
        print(f"Solve time: mean={np.mean(solve_times):.1f} ms, "
              f"max={np.max(solve_times):.1f} ms, "
              f"p95={np.percentile(solve_times, 95):.1f} ms")

    # Overhead verification diagnostics
    overhead = [r.overhead_ms for r in logger.records if r.overhead_ms > 0]
    if overhead:
        print(f"Non-solve overhead: median={np.median(overhead):.1f} ms, "
              f"p95={np.percentile(overhead, 95):.1f} ms")
    fk_iters = [r.fk_iterations for r in logger.records if r.fk_iterations > 0]
    if fk_iters:
        print(f"FK iterations: mean={np.mean(fk_iters):.1f}, "
              f"max={np.max(fk_iters)}")
    ff_torques = [r.ff_torque_max_Nm for r in logger.records
                  if r.ff_torque_max_Nm > 0]
    if ff_torques:
        print(f"FF torque max: mean={np.mean(ff_torques):.3f} Nm, "
              f"max={np.max(ff_torques):.3f} Nm")


# ---------------------------------------------------------------------------
# Main MPC loop
# ---------------------------------------------------------------------------

def run_mpc_loop(
    plant: PlantInterface,
    mpc,
    source,
    duration: float,
    logger: TelemetryLogger,
    *,
    control_dt: float = 0.025,
    dashboard=None,
    hooks: MpcLoopHooks | None = None,
) -> bool:
    """Wall-clock-paced MPC loop shared by simulation and hardware.

    Paces iterations to wall-clock ``control_dt`` so the MPC's horizon
    predictions match real elapsed time.  If a solve overruns the budget,
    the next iteration runs immediately (no accumulated debt).

    Returns
    -------
    bool
        True if the loop exited because ``plant.estop_requested`` tripped
        (clean shutdown triggered by HardwarePlant.estop()).  False if the
        loop ran to its full duration.  The caller uses this to select a
        distinct exit code.

    Parameters
    ----------
    plant : PlantInterface
        Plant to command (MuJoCoPlant or HardwarePlant).
    mpc : MPCController
        The MPC solver instance.
    source : TargetSource
        Any object with ``update(sim_time, state) -> TargetCommand``.
        May optionally provide ``poll()``, ``enabled``, ``notify_capture()``,
        and ``print_summary()``.
    duration : float
        Total run duration in seconds.
    logger : TelemetryLogger
        Telemetry accumulator.
    control_dt : float
        Control loop period (seconds).  Default 0.025 = 40 Hz.
    dashboard : DashboardServer or None
        Optional live telemetry dashboard.
    hooks : MpcLoopHooks or None
        Callbacks for sim- or hardware-specific behavior.
    """
    if hooks is None:
        hooks = MpcLoopHooks()

    n_steps = int(duration / control_dt)
    wall_budget = 0.0
    start_wall = _time.monotonic()

    # Lifecycle support for sources that manage their own mode
    # (e.g. ZmqTargetSource with .enabled / .poll()).
    has_lifecycle = hasattr(source, 'enabled')
    was_enabled = not has_lifecycle  # non-lifecycle sources start enabled

    # Reference clock — advances only on success-class solver status.
    # Initialised lazily on the first iteration (to match the plant's
    # actual start time, which may be non-zero on hardware).  While the
    # solver is in a fallback class the reference stays frozen so the
    # commanded platform does not fall behind a running reference.
    t_ref: float | None = None
    _FALLBACK_KEYWORDS = ('fallback', 'hold', 'cold_hold')

    # Install GC tracker — records pause durations so the per-tick breakdown
    # can attribute overhead spikes to Python garbage collection.  Uninstalled
    # in finally below so leftover callbacks don't accumulate across runs.
    gc_tracker = _GCTracker()
    gc_tracker.install()
    # Print `OH SPIKE step=N ...` breakdown when overhead exceeds this.
    _OH_SPIKE_THRESHOLD_MS = 15.0

    # W4c — pre-allocated buffer for pose_6dof_from_state's inline
    # expansion in the per-tick record-fill path.  Before W4c this was a
    # fresh np.concatenate alloc every tick; now the runner writes
    # state.platform_pos_mm / state.platform_rot into a single 6-vector
    # in place and passes a view.
    _actual_pose_buf = np.empty(6)

    # W5 — GC scheduling on the idle sleep.
    #
    # The hot-loop zero-allocation contract (controller/HOT_LOOP_CONTRACT.md)
    # keeps per-tick allocation < 1024 B, but Python's generational GC could
    # still fire inside a tick on accumulated retention from elsewhere (e.g.
    # CasADi solver state, ZMQ drain).  Disabling GC for the loop body and
    # running a scheduled Gen-2 collection between ticks on the wall-clock
    # sleep window guarantees:
    #   * No in-tick GC pause can ever land inside a 25 ms budget.
    #   * Accumulation is bounded — the periodic collect frees cycles that
    #     refcount alone cannot.
    # Re-enabled in the finally block so exceptions or normal exits don't
    # leave the interpreter with GC disabled globally.
    # Cadence tuned on W6 hardware validation (2026-04-23): the scheduled
    # Gen-2 collect costs ~140 ms on the Jetson Orin Nano (much more than
    # the 5 ms I initially predicted, because CasADi internals + numpy
    # wrappers accumulate enough Gen 2 candidates every few seconds that a
    # sweep walks a non-trivial object graph).  The collect itself fires
    # during the idle-sleep window so it cannot land inside a tick — but
    # every firing produces a ~140 ms gap in the 40 Hz command stream
    # that the motor guard's Hermite interpolation has to absorb.
    # 1200 ticks = 30 s gives ~2 collects/minute instead of the ~4/minute
    # we measured at 200 ticks, without changing any correctness
    # property.  If hardware testing ever shows Gen-2 memory accumulating
    # faster than once per 30 s, shorten this — W5's gc.disable() keeps
    # in-tick GC off regardless of cadence.
    _GC_COLLECT_EVERY_N_TICKS = 1200    # 30 s at 40 Hz
    _GC_COLLECT_MIN_SLEEP_S = 0.005     # only fire when >=5 ms sleep budget
    # gc.isenabled() capture and gc.disable() live inside the try/finally so
    # a signal raised in the microsecond between them can never leave the
    # interpreter with GC globally disabled — the finally block must always
    # run after a successful gc.disable().  Default True is the safe fallback
    # if the interrupt lands before we read the real state.
    _was_gc_enabled = True

    # Set True by the loop when plant.estop_requested trips, so the caller
    # can report it (e.g. run_mpc.py's distinct exit code).
    estop_exit = False

    try:
        _was_gc_enabled = _gc.isenabled()
        _gc.disable()
        for _step_idx in range(n_steps):
            # --- Clean-shutdown flag (hardware only) ---
            # HardwarePlant.estop() flips this after publishing the ESTOP
            # mode message, so motor_guard is already in ESTOP by the time
            # we break out.  Sim plants don't expose the attribute; getattr
            # default keeps the sim path unchanged.
            if getattr(plant, 'estop_requested', False):
                print("MPC loop: plant estop_requested — exiting cleanly")
                estop_exit = True
                break

            # --- Source-driven termination ---
            # Sources that represent a finite plan (TossMotionSource,
            # AutoSequenceTargetSource) expose a ``.done`` property.  When
            # they flag completion the loop exits cleanly — no E-stop, no
            # duration wait.  Sources without .done (ZmqTargetSource, etc.)
            # are unaffected because getattr default is False.
            if getattr(source, 'done', False):
                print("MPC loop: source signalled done — exiting cleanly")
                break

            # --- Lifecycle: enable/disable plant based on source mode ---
            if has_lifecycle:
                source.poll()  # drain ZMQ so .enabled is up-to-date
                now_enabled = source.enabled
                if now_enabled and not was_enabled:
                    if hasattr(plant, 'enable'):
                        plant.enable()
                        _time.sleep(0.05)  # let motor guard process enable
                    print(f"MPC loop: source enabled "
                          f"(mode={getattr(source, 'mode', '?')})")
                elif not now_enabled and was_enabled:
                    if hasattr(plant, 'disable'):
                        plant.disable()
                    print("MPC loop: source disabled")
                was_enabled = now_enabled

                if not now_enabled:
                    _time.sleep(control_dt)
                    start_wall = _time.monotonic()
                    wall_budget = 0.0
                    continue

            _t_overhead = _time.perf_counter()

            # --- get_state (ZMQ drain + FK + Jacobian) ---
            _t_seg = _time.perf_counter()
            state = plant.get_state()
            _t_getstate_ms = (_time.perf_counter() - _t_seg) * 1000.0

            if t_ref is None:
                t_ref = state.time

            # --- target source update + on_target_override ---
            _t_seg = _time.perf_counter()
            tc = source.update(t_ref, state)
            if hooks.on_target_override is not None:
                tc = hooks.on_target_override(state, tc)
            _t_target_ms = (_time.perf_counter() - _t_seg) * 1000.0

            # --- MPC solve (ref build + param pack + IPOPT) ---
            _t_seg = _time.perf_counter()
            cmd, cmd_vel, diag, ref_pose, ref_twist = mpc_solve(
                mpc, state, tc, t_now=t_ref)
            _t_solve_total_ms = (_time.perf_counter() - _t_seg) * 1000.0
            _t_solve_setup_ms = max(
                0.0, _t_solve_total_ms - diag.get('solve_time_ms', 0.0))

            # --- hooks: post-solve + pre-command (feedforward dynamics) ---
            _t_seg = _time.perf_counter()
            if hooks.on_post_solve is not None:
                hooks.on_post_solve(tc, diag)
            if hooks.on_pre_command is not None:
                hooks.on_pre_command(plant, mpc, tc, cmd, cmd_vel, diag)
            _t_hooks_ms = (_time.perf_counter() - _t_seg) * 1000.0

            # --- plant.command + plant.step ---
            _t_seg = _time.perf_counter()
            cmd_next = diag.get('cmd_next_mm')
            cmd_next2 = diag.get('cmd_next2_mm')
            plant.command(cmd, vel_mm_s=cmd_vel,
                          cmd_next_mm=cmd_next, cmd_next2_mm=cmd_next2)
            plant.step(control_dt)
            _t_cmd_ms = (_time.perf_counter() - _t_seg) * 1000.0

            # Non-solve overhead (same formula as before — net of IPOPT solve time)
            _overhead_ms = ((_time.perf_counter() - _t_overhead) * 1000.0
                            - diag.get('solve_time_ms', 0.0))

            # Hook: post-step (e.g. ball capture, sim-specific side effects)
            if hooks.on_post_step is not None:
                hooks.on_post_step(state, state.time)

            # --- GC attribution (events that finished inside this tick) ---
            # With W5's gc.disable() active, the steady-state expectation is
            # _gc_count == 0 on every tick.  Any non-zero count here means
            # either (a) a manual gc.collect() call sneaked in from
            # application code (unlikely; grep shows none on the hot path),
            # or (b) gc.enable() got called by some third-party library.
            # Either way it's a contract violation — emit a visible warning.
            _gc_ms, _gc_count, _gc_max_gen = gc_tracker.drain_since(_t_overhead)
            if _gc_count > 0:
                print(
                    f"W5 VIOLATION step={_step_idx}: in-tick GC fired "
                    f"(count={_gc_count}, gen={_gc_max_gen}, "
                    f"ms={_gc_ms:.1f}) — gc.isenabled()={_gc.isenabled()}"
                )

            # --- telemetry logging (record build + CSV append + dashboard) ---
            _drain_count = int(getattr(plant, 'last_drain_count', 0))
            _fk_iterations = 0
            _ff_torque_max_Nm = 0.0
            if hooks.on_log_extras is not None:
                _hook_extras = hooks.on_log_extras(plant)
                # Hook may return a dict (legacy) or a SimpleNamespace
                # (contract-compliant pre-alloc).  Read via getattr/dict
                # so either works; unknown keys are ignored.
                if isinstance(_hook_extras, dict):
                    _fk_iterations = _hook_extras.get('fk_iterations', 0)
                    _ff_torque_max_Nm = _hook_extras.get('ff_torque_max_Nm', 0.0)
                else:
                    _fk_iterations = getattr(_hook_extras, 'fk_iterations', 0)
                    _ff_torque_max_Nm = getattr(_hook_extras, 'ff_torque_max_Nm', 0.0)

            _t_log_start = _time.perf_counter()
            # Inline the fill-record path so no ``extras`` dict is built
            # for kwargs expansion.  log_mpc_step is retained for sim
            # callers that still take the legacy path.
            _rec = logger.next_record()
            # W4c: inline pose_6dof_from_state into the pre-allocated
            # buffer so the fill doesn't allocate a fresh (6,) every tick.
            _actual_pose_buf[:3] = state.platform_pos_mm
            _actual_pose_buf[3:] = state.platform_rot
            fill_record_from_arrays(
                _rec,
                time=state.time,
                ref_pose=ref_pose,
                ref_twist=ref_twist if ref_twist is not None else _LOG_ZERO6,
                actual_pose=_actual_pose_buf,
                actual_twist=state.platform_twist,
                cmd_extensions=cmd,
                actual_extensions=state.leg_extensions_mm,
                leg_velocities=state.leg_velocities_mmps,
                hand_pos_mm=(state.hand_pos_mm
                             if state.hand_pos_mm is not None else 0.0),
                hand_vel_mmps=(state.hand_vel_mmps
                               if state.hand_vel_mmps is not None else 0.0),
                solve_time_ms=diag.get('solve_time_ms', 0.0),
                solve_status=diag.get('status', 'n/a'),
                cost=diag.get('cost', 0.0),
                constraint_violation=diag.get('constraint_violation', 0.0),
                ipopt_iter=diag.get('iter_count', 0),
                t_ref_s=t_ref,
                overhead_ms=_overhead_ms,
                t_getstate_ms=_t_getstate_ms,
                t_target_ms=_t_target_ms,
                t_solve_setup_ms=_t_solve_setup_ms,
                t_hooks_ms=_t_hooks_ms,
                t_cmd_ms=_t_cmd_ms,
                gc_ms=_gc_ms,
                zmq_drain_count=_drain_count,
                fk_iterations=_fk_iterations,
                ff_torque_max_Nm=_ff_torque_max_Nm,
            )
            if dashboard is not None:
                dashboard.broadcast(_rec)
            _t_log_ms = (_time.perf_counter() - _t_log_start) * 1000.0
            # Stamp the just-written record with the measured log duration.
            # Pool slot is still held by _rec — no list[-1] allocation.
            _rec.t_log_ms = _t_log_ms

            # Stdout diagnostic: attribute overhead when it spikes.  Quiet on
            # steady-state ticks.  `other` lumps residual time (pacing math,
            # dict construction, stdout, and unaccounted-for slack).
            if _overhead_ms > _OH_SPIKE_THRESHOLD_MS:
                _t_accounted = (_t_getstate_ms + _t_target_ms
                                + _t_solve_setup_ms + _t_hooks_ms
                                + _t_cmd_ms + _t_log_ms)
                _residual = max(0.0, _overhead_ms - _t_accounted)
                print(
                    f"OH SPIKE step={_step_idx} oh={_overhead_ms:.1f} "
                    f"getstate={_t_getstate_ms:.1f} "
                    f"target={_t_target_ms:.1f} "
                    f"solve_setup={_t_solve_setup_ms:.1f} "
                    f"hooks={_t_hooks_ms:.1f} "
                    f"cmd={_t_cmd_ms:.1f} "
                    f"log={_t_log_ms:.1f} "
                    f"other={_residual:.1f} "
                    f"gc=[n={_gc_count},gen={_gc_max_gen},ms={_gc_ms:.1f}] "
                    f"drain={_drain_count} "
                    f"solve={diag.get('solve_time_ms', 0.0):.1f} "
                    f"status={diag.get('status', 'n/a')}"
                )

            # Advance t_ref only when the solver returned a success-class
            # status.  Fallback / hold / cold_hold all hold the platform
            # commands — the reference must pause too, or it accumulates lag
            # at v_max per tick and the platform lands far behind when the
            # solver recovers.
            status = diag.get('status', '')
            is_fallback = any(kw in status for kw in _FALLBACK_KEYWORDS)
            if not is_fallback:
                t_ref += control_dt

            # Wall-clock pacing
            wall_budget += control_dt
            elapsed = _time.monotonic() - start_wall
            sleep_time = wall_budget - elapsed

            # W5 — scheduled Gen-2 collection on the idle sleep window.
            # Only fires when (a) we're at the cadence tick, and (b) there's
            # real sleep budget to absorb the collection cost.  Gen 2 is
            # explicitly requested (not Gen 0/1) because the point is to
            # free the rare long-lived cycles that refcount cannot; the
            # shorter Gen 0/1 sweeps would be wasted here given how little
            # we allocate per-tick post-W4d.
            if (sleep_time > _GC_COLLECT_MIN_SLEEP_S
                    and _step_idx > 0
                    and _step_idx % _GC_COLLECT_EVERY_N_TICKS == 0):
                _t_gc_start = _time.perf_counter()
                _gc.collect(generation=2)
                _gc_manual_ms = (_time.perf_counter() - _t_gc_start) * 1000.0
                # Subtract elapsed collection time so pacing stays honest.
                sleep_time -= _gc_manual_ms / 1000.0

            if sleep_time > 0:
                _time.sleep(sleep_time)
            elif sleep_time < -control_dt:
                start_wall = _time.monotonic()
                wall_budget = 0.0
    finally:
        gc_tracker.uninstall()
        # Restore the pre-loop GC state.  Using enable() unconditionally
        # would clobber a caller who had already disabled GC for their own
        # reasons; record state at entry and restore exactly.
        if _was_gc_enabled:
            _gc.enable()

    logger.flush()
    print_mpc_summary(logger)
    if hasattr(source, 'print_summary'):
        source.print_summary()
    return estop_exit
