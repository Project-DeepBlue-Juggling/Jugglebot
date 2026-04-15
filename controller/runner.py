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

import time as _time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

import numpy as np

from .plant import PlantInterface, PlantState
from .target import TargetCommand, TargetSource, _pose_6dof_from_state
from .telemetry import TelemetryLogger, record_from_arrays


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

def mpc_solve(mpc, state: PlantState, tc: TargetCommand):
    """Call mpc.solve() with the target from a TargetCommand.

    Returns (cmd, cmd_vel, diag, ref_pose, ref_twist).

    ref_pose and ref_twist come from the MPC's own node-0 reference
    (quintic Hermite between events).  This ensures telemetry tracking
    error is measured against the reference the MPC actually optimized
    against.
    """
    cmd, cmd_vel, diag = mpc.solve(
        state, tc.target_pose,
        ref_events=tc.ref_events,
        boost_vel_weights=tc.boost_vel_weights,
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

def log_mpc_step(logger: TelemetryLogger, state: PlantState,
                 ref_pose: np.ndarray, cmd_ext: np.ndarray,
                 diag: dict, ref_twist: np.ndarray | None = None,
                 dashboard=None, **extras) -> None:
    """Record one telemetry step (MPC mode)."""
    record = record_from_arrays(
        time=state.time,
        ref_pose=ref_pose,
        ref_twist=ref_twist if ref_twist is not None else np.zeros(6),
        actual_pose=_pose_6dof_from_state(state),
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
        **extras,
    )
    logger.append(record)
    if dashboard is not None:
        dashboard.broadcast(record)


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
) -> None:
    """Wall-clock-paced MPC loop shared by simulation and hardware.

    Paces iterations to wall-clock ``control_dt`` so the MPC's horizon
    predictions match real elapsed time.  If a solve overruns the budget,
    the next iteration runs immediately (no accumulated debt).

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

    for _ in range(n_steps):
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
        state = plant.get_state()
        tc = source.update(state.time, state)

        # Hook: target override (e.g. stale telemetry → hold-in-place)
        if hooks.on_target_override is not None:
            tc = hooks.on_target_override(state, tc)

        # MPC solve
        cmd, cmd_vel, diag, ref_pose, ref_twist = mpc_solve(mpc, state, tc)

        # Hook: post-solve (e.g. target feedback publishing)
        if hooks.on_post_solve is not None:
            hooks.on_post_solve(tc, diag)

        # Hook: pre-command (e.g. feedforward torque setup)
        if hooks.on_pre_command is not None:
            hooks.on_pre_command(plant, mpc, tc, cmd, cmd_vel, diag)

        plant.command(cmd, vel_mm_s=cmd_vel)
        plant.step(control_dt)

        # Non-solve overhead
        _overhead_ms = ((_time.perf_counter() - _t_overhead) * 1000.0
                        - diag.get('solve_time_ms', 0.0))

        # Hook: post-step (e.g. ball capture, sim-specific side effects)
        if hooks.on_post_step is not None:
            hooks.on_post_step(state, state.time)

        # Telemetry logging
        extras = {'overhead_ms': _overhead_ms}
        if hooks.on_log_extras is not None:
            extras.update(hooks.on_log_extras(plant))
        log_mpc_step(logger, state, ref_pose, cmd, diag,
                     ref_twist=ref_twist, dashboard=dashboard, **extras)

        # Wall-clock pacing
        wall_budget += control_dt
        elapsed = _time.monotonic() - start_wall
        sleep_time = wall_budget - elapsed
        if sleep_time > 0:
            _time.sleep(sleep_time)
        elif sleep_time < -control_dt:
            start_wall = _time.monotonic()
            wall_budget = 0.0

    logger.flush()
    print_mpc_summary(logger)
    if hasattr(source, 'print_summary'):
        source.print_summary()
