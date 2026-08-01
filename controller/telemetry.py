"""Telemetry logging and plotting for the MPC harness.

Provides a structured per-step log record (StepRecord), a TelemetryLogger that
accumulates records and writes CSV, and plotting utilities for post-hoc analysis.

The same schema is used for sim and hardware — enables direct comparison.
"""

from __future__ import annotations

import csv
import os
from dataclasses import dataclass, field, fields, asdict
from typing import Sequence

import numpy as np


@dataclass
class StepRecord:
    """One row of telemetry, captured each MPC step (40 Hz)."""

    time: float = 0.0  # simulation time (s)

    # Reference
    ref_pose_x: float = 0.0
    ref_pose_y: float = 0.0
    ref_pose_z: float = 0.0
    ref_pose_rx: float = 0.0
    ref_pose_ry: float = 0.0
    ref_pose_rz: float = 0.0
    ref_twist_vx: float = 0.0
    ref_twist_vy: float = 0.0
    ref_twist_vz: float = 0.0
    ref_twist_wx: float = 0.0
    ref_twist_wy: float = 0.0
    ref_twist_wz: float = 0.0

    # Actual platform state
    actual_pose_x: float = 0.0
    actual_pose_y: float = 0.0
    actual_pose_z: float = 0.0
    actual_pose_rx: float = 0.0
    actual_pose_ry: float = 0.0
    actual_pose_rz: float = 0.0
    actual_twist_vx: float = 0.0
    actual_twist_vy: float = 0.0
    actual_twist_vz: float = 0.0
    actual_twist_wx: float = 0.0
    actual_twist_wy: float = 0.0
    actual_twist_wz: float = 0.0

    # Actuators (6 legs each)
    cmd_ext_0: float = 0.0
    cmd_ext_1: float = 0.0
    cmd_ext_2: float = 0.0
    cmd_ext_3: float = 0.0
    cmd_ext_4: float = 0.0
    cmd_ext_5: float = 0.0
    actual_ext_0: float = 0.0
    actual_ext_1: float = 0.0
    actual_ext_2: float = 0.0
    actual_ext_3: float = 0.0
    actual_ext_4: float = 0.0
    actual_ext_5: float = 0.0
    leg_vel_0: float = 0.0
    leg_vel_1: float = 0.0
    leg_vel_2: float = 0.0
    leg_vel_3: float = 0.0
    leg_vel_4: float = 0.0
    leg_vel_5: float = 0.0

    # Hand actuator
    hand_cmd_mm: float = 0.0
    hand_pos_mm: float = 0.0
    hand_vel_mmps: float = 0.0

    # Leg accelerations (mm/s^2; numerical derivative of leg_velocities
    # computed by the broadcaster).  Sim juggle-demo only — zero elsewhere.
    leg_acc_0: float = 0.0
    leg_acc_1: float = 0.0
    leg_acc_2: float = 0.0
    leg_acc_3: float = 0.0
    leg_acc_4: float = 0.0
    leg_acc_5: float = 0.0

    # Balls 0 and 1 — sim juggle-demo only.  position in mm; velocity in
    # mm/s; held=1 while the ball is welded to the hand cup, 0 otherwise.
    # NaN positions signal "no ball present" (e.g. before BB priming).
    ball0_x: float = 0.0
    ball0_y: float = 0.0
    ball0_z: float = 0.0
    ball0_vx: float = 0.0
    ball0_vy: float = 0.0
    ball0_vz: float = 0.0
    ball0_held: int = 0
    ball1_x: float = 0.0
    ball1_y: float = 0.0
    ball1_z: float = 0.0
    ball1_vx: float = 0.0
    ball1_vy: float = 0.0
    ball1_vz: float = 0.0
    ball1_held: int = 0

    # Throw director (sim juggle-demo only).  ``throw_phase`` reports
    # the kind of the most recently dispatched timeline event
    # (``bb_throw`` / ``hand_throw`` / ``hand_catch`` / ``aborted``),
    # or empty before any event has fired.  ``catches_total`` is the
    # running BallManager.check_capture count.
    throw_phase: str = ""
    catches_total: int = 0

    # MPC diagnostics (populated from Phase 2 onward)
    solve_time_ms: float = 0.0
    solve_status: str = "n/a"
    cost: float = 0.0
    constraint_violation: float = 0.0

    # Derived metrics
    tracking_error_mm: float = 0.0
    tracking_error_deg: float = 0.0

    # Overhead verification (optional, zero when not populated)
    overhead_ms: float = 0.0           # non-solve overhead per MPC step
    fk_iterations: int = 0             # FK Newton-Raphson iterations
    ff_torque_max_Nm: float = 0.0      # max feedforward torque magnitude
    ipopt_iter: int = 0                # IPOPT iteration count per solve

    # Per-segment overhead breakdown (attributes overhead_ms to specific
    # phases of the MPC loop).  Populated by the hardware/sim runner;
    # zero when not measured.  Their sum approximates overhead_ms modulo
    # measurement skew.
    t_getstate_ms: float = 0.0         # plant.get_state() (ZMQ drain + FK + Jacobian)
    t_target_ms: float = 0.0           # source.update + on_target_override
    t_solve_setup_ms: float = 0.0      # mpc_solve wrapper minus solve_time_ms (ref build, param pack, warm-start shift)
    t_hooks_ms: float = 0.0            # on_post_solve + on_pre_command (e.g. feedforward dynamics)
    t_cmd_ms: float = 0.0              # plant.command (ZMQ send)
    t_log_ms: float = 0.0              # log_mpc_step (record build + logger.append + dashboard broadcast)
    gc_ms: float = 0.0                 # time spent in Python GC callbacks during this tick
    zmq_drain_count: int = 0           # messages drained from telemetry SUB in get_state

    # Reference clock — advances only on success-class solver status so the
    # reference does not run away from the platform during fallback chains.
    # Lags state.time during saturation; equals state.time during healthy
    # operation.  See controller/runner.py (t_ref gating).
    t_ref_s: float = 0.0


def record_from_arrays(
    time: float,
    ref_pose: np.ndarray,
    ref_twist: np.ndarray,
    actual_pose: np.ndarray,
    actual_twist: np.ndarray,
    cmd_extensions: np.ndarray,
    actual_extensions: np.ndarray,
    leg_velocities: np.ndarray | None = None,
    leg_accelerations: np.ndarray | None = None,
    hand_cmd_mm: float = 0.0,
    hand_pos_mm: float = 0.0,
    hand_vel_mmps: float = 0.0,
    ball_positions_mm: np.ndarray | None = None,
    ball_velocities_mms: np.ndarray | None = None,
    ball_held: np.ndarray | None = None,
    throw_phase: str = "",
    catches_total: int = 0,
    solve_time_ms: float = 0.0,
    solve_status: str = "n/a",
    cost: float = 0.0,
    constraint_violation: float = 0.0,
    overhead_ms: float = 0.0,
    fk_iterations: int = 0,
    ff_torque_max_Nm: float = 0.0,
    ipopt_iter: int = 0,
    t_ref_s: float = 0.0,
    t_getstate_ms: float = 0.0,
    t_target_ms: float = 0.0,
    t_solve_setup_ms: float = 0.0,
    t_hooks_ms: float = 0.0,
    t_cmd_ms: float = 0.0,
    t_log_ms: float = 0.0,
    gc_ms: float = 0.0,
    zmq_drain_count: int = 0,
) -> StepRecord:
    """Build a StepRecord from numpy arrays (convenience helper).

    Allocates a fresh ``StepRecord``.  NOT hot-path safe — the 40 Hz MPC
    loop uses ``fill_record_from_arrays`` against a pool slot instead.
    This function is retained for sim paths (``sim/main.py``,
    ``sim/demo_mpc.py``) that predate the pool design.

    The ``leg_accelerations`` / ``ball_*`` / ``throw_*`` / ``catches_total``
    kwargs are sim-juggle-demo additions; hardware MPC callers leave them
    at their defaults and the corresponding StepRecord fields stay at 0.
    """
    rec = StepRecord()
    fill_record_from_arrays(
        rec, time=time, ref_pose=ref_pose, ref_twist=ref_twist,
        actual_pose=actual_pose, actual_twist=actual_twist,
        cmd_extensions=cmd_extensions, actual_extensions=actual_extensions,
        leg_velocities=leg_velocities,
        leg_accelerations=leg_accelerations,
        hand_cmd_mm=hand_cmd_mm, hand_pos_mm=hand_pos_mm,
        hand_vel_mmps=hand_vel_mmps,
        ball_positions_mm=ball_positions_mm,
        ball_velocities_mms=ball_velocities_mms,
        ball_held=ball_held,
        throw_phase=throw_phase,
        catches_total=catches_total,
        solve_time_ms=solve_time_ms, solve_status=solve_status,
        cost=cost, constraint_violation=constraint_violation,
        overhead_ms=overhead_ms, fk_iterations=fk_iterations,
        ff_torque_max_Nm=ff_torque_max_Nm, ipopt_iter=ipopt_iter,
        t_ref_s=t_ref_s,
        t_getstate_ms=t_getstate_ms, t_target_ms=t_target_ms,
        t_solve_setup_ms=t_solve_setup_ms, t_hooks_ms=t_hooks_ms,
        t_cmd_ms=t_cmd_ms, t_log_ms=t_log_ms,
        gc_ms=gc_ms, zmq_drain_count=zmq_drain_count,
    )
    return rec


_ZERO6 = np.zeros(6)


def fill_record_from_arrays(
    rec: StepRecord,
    *,
    time: float,
    ref_pose: np.ndarray,
    ref_twist: np.ndarray,
    actual_pose: np.ndarray,
    actual_twist: np.ndarray,
    cmd_extensions: np.ndarray,
    actual_extensions: np.ndarray,
    leg_velocities: np.ndarray | None = None,
    leg_accelerations: np.ndarray | None = None,
    hand_cmd_mm: float = 0.0,
    hand_pos_mm: float = 0.0,
    hand_vel_mmps: float = 0.0,
    ball_positions_mm: np.ndarray | None = None,
    ball_velocities_mms: np.ndarray | None = None,
    ball_held: np.ndarray | None = None,
    throw_phase: str = "",
    catches_total: int = 0,
    solve_time_ms: float = 0.0,
    solve_status: str = "n/a",
    cost: float = 0.0,
    constraint_violation: float = 0.0,
    overhead_ms: float = 0.0,
    fk_iterations: int = 0,
    ff_torque_max_Nm: float = 0.0,
    ipopt_iter: int = 0,
    t_ref_s: float = 0.0,
    t_getstate_ms: float = 0.0,
    t_target_ms: float = 0.0,
    t_solve_setup_ms: float = 0.0,
    t_hooks_ms: float = 0.0,
    t_cmd_ms: float = 0.0,
    t_log_ms: float = 0.0,
    gc_ms: float = 0.0,
    zmq_drain_count: int = 0,
) -> None:
    """Fill a pre-existing StepRecord from numpy arrays — hot-loop body.

    Mutates ``rec`` in place; does not allocate a new StepRecord.  Used
    by ``run_mpc_loop`` against a pool slot from ``logger.next_record()``.

    Field-by-field assignment uses ``float(arr[i])`` rather than
    ``arr[i]`` so the slot stores Python floats (size ~16 B) rather
    than numpy.float64 scalars (size ~32 B).  Once the pool wraps each
    new assignment frees the prior Python float, keeping tracemalloc
    net-growth flat.
    """
    lv = leg_velocities if leg_velocities is not None else _ZERO6
    la = leg_accelerations if leg_accelerations is not None else _ZERO6

    # Tracking-error reduction uses temporaries that are immediately
    # freed (refcount=0 at end of statement) — cheap.
    pos_err = float(np.linalg.norm(actual_pose[:3] - ref_pose[:3]))
    ori_err = float(np.degrees(np.linalg.norm(actual_pose[3:] - ref_pose[3:])))

    rec.time = time
    rec.ref_pose_x = float(ref_pose[0])
    rec.ref_pose_y = float(ref_pose[1])
    rec.ref_pose_z = float(ref_pose[2])
    rec.ref_pose_rx = float(ref_pose[3])
    rec.ref_pose_ry = float(ref_pose[4])
    rec.ref_pose_rz = float(ref_pose[5])
    rec.ref_twist_vx = float(ref_twist[0])
    rec.ref_twist_vy = float(ref_twist[1])
    rec.ref_twist_vz = float(ref_twist[2])
    rec.ref_twist_wx = float(ref_twist[3])
    rec.ref_twist_wy = float(ref_twist[4])
    rec.ref_twist_wz = float(ref_twist[5])
    rec.actual_pose_x = float(actual_pose[0])
    rec.actual_pose_y = float(actual_pose[1])
    rec.actual_pose_z = float(actual_pose[2])
    rec.actual_pose_rx = float(actual_pose[3])
    rec.actual_pose_ry = float(actual_pose[4])
    rec.actual_pose_rz = float(actual_pose[5])
    rec.actual_twist_vx = float(actual_twist[0])
    rec.actual_twist_vy = float(actual_twist[1])
    rec.actual_twist_vz = float(actual_twist[2])
    rec.actual_twist_wx = float(actual_twist[3])
    rec.actual_twist_wy = float(actual_twist[4])
    rec.actual_twist_wz = float(actual_twist[5])
    rec.cmd_ext_0 = float(cmd_extensions[0])
    rec.cmd_ext_1 = float(cmd_extensions[1])
    rec.cmd_ext_2 = float(cmd_extensions[2])
    rec.cmd_ext_3 = float(cmd_extensions[3])
    rec.cmd_ext_4 = float(cmd_extensions[4])
    rec.cmd_ext_5 = float(cmd_extensions[5])
    rec.actual_ext_0 = float(actual_extensions[0])
    rec.actual_ext_1 = float(actual_extensions[1])
    rec.actual_ext_2 = float(actual_extensions[2])
    rec.actual_ext_3 = float(actual_extensions[3])
    rec.actual_ext_4 = float(actual_extensions[4])
    rec.actual_ext_5 = float(actual_extensions[5])
    rec.leg_vel_0 = float(lv[0])
    rec.leg_vel_1 = float(lv[1])
    rec.leg_vel_2 = float(lv[2])
    rec.leg_vel_3 = float(lv[3])
    rec.leg_vel_4 = float(lv[4])
    rec.leg_vel_5 = float(lv[5])
    rec.leg_acc_0 = float(la[0])
    rec.leg_acc_1 = float(la[1])
    rec.leg_acc_2 = float(la[2])
    rec.leg_acc_3 = float(la[3])
    rec.leg_acc_4 = float(la[4])
    rec.leg_acc_5 = float(la[5])
    rec.hand_cmd_mm = hand_cmd_mm
    rec.hand_pos_mm = hand_pos_mm
    rec.hand_vel_mmps = hand_vel_mmps
    if ball_positions_mm is not None:
        rec.ball0_x = float(ball_positions_mm[0, 0])
        rec.ball0_y = float(ball_positions_mm[0, 1])
        rec.ball0_z = float(ball_positions_mm[0, 2])
        rec.ball1_x = float(ball_positions_mm[1, 0])
        rec.ball1_y = float(ball_positions_mm[1, 1])
        rec.ball1_z = float(ball_positions_mm[1, 2])
    if ball_velocities_mms is not None:
        rec.ball0_vx = float(ball_velocities_mms[0, 0])
        rec.ball0_vy = float(ball_velocities_mms[0, 1])
        rec.ball0_vz = float(ball_velocities_mms[0, 2])
        rec.ball1_vx = float(ball_velocities_mms[1, 0])
        rec.ball1_vy = float(ball_velocities_mms[1, 1])
        rec.ball1_vz = float(ball_velocities_mms[1, 2])
    if ball_held is not None:
        rec.ball0_held = int(ball_held[0])
        rec.ball1_held = int(ball_held[1])
    rec.throw_phase = throw_phase
    rec.catches_total = int(catches_total)
    rec.solve_time_ms = solve_time_ms
    rec.solve_status = solve_status
    rec.cost = cost
    rec.constraint_violation = constraint_violation
    rec.tracking_error_mm = pos_err
    rec.tracking_error_deg = ori_err
    rec.overhead_ms = overhead_ms
    rec.fk_iterations = fk_iterations
    rec.ff_torque_max_Nm = ff_torque_max_Nm
    rec.ipopt_iter = ipopt_iter
    rec.t_ref_s = t_ref_s
    rec.t_getstate_ms = t_getstate_ms
    rec.t_target_ms = t_target_ms
    rec.t_solve_setup_ms = t_solve_setup_ms
    rec.t_hooks_ms = t_hooks_ms
    rec.t_cmd_ms = t_cmd_ms
    rec.t_log_ms = t_log_ms
    rec.gc_ms = gc_ms
    rec.zmq_drain_count = zmq_drain_count


class TelemetryLogger:
    """Accumulates StepRecords and periodically flushes them to CSV.

    Records are written to disk in batches to bound memory usage and provide
    crash safety.  A rolling tail of recent records is kept in memory for
    end-of-run summary stats.  Use :meth:`load` to read the full history back
    from the CSV after the run.

    Hot-loop zero-allocation contract:
        In steady state, ``next_record()`` + ``fill_record_from_arrays()``
        recycles records from a pre-allocated pool of ``_POOL_SIZE`` slots.
        Per-tick allocation is limited to the Python floats written into
        pool slots; once the pool wraps (every ``_POOL_SIZE`` ticks), new
        assignments free the prior floats and retention stays flat.  See
        ``controller/HOT_LOOP_CONTRACT.md``.

    Legacy API::

        logger = TelemetryLogger("logs/run_001.csv")
        for step in sim_loop:
            logger.append(record)    # copies into pool; still allocates
        logger.flush()
        all_records = logger.load()

    Hot-loop API::

        logger = TelemetryLogger("logs/run_001.csv")
        for step in sim_loop:
            rec = logger.next_record()
            fill_record_from_arrays(rec, time=..., ref_pose=..., ...)
        logger.flush()
    """

    # Default pool size — sized to cover any existing test
    # (max duration 3.0 s × 40 Hz = 120 ticks) plus generous headroom.
    # 500 records at ~1.2 KB each = ~600 KB resident memory.  Production
    # path= runs flush to disk every POOL_SIZE records, i.e. every
    # 12.5 s at 40 Hz — crash-safety window is bounded.
    _DEFAULT_POOL_SIZE: int = 500

    # Retained for backward-compat with test code that references the
    # attribute.  After the pool refactor it's equal to the effective
    # pool size.
    _TAIL_SIZE: int = 500

    def __init__(self, path: str | None = None, *, pool_size: int | None = None):
        """Create a telemetry logger backed by a pre-allocated record pool.

        Parameters
        ----------
        path : str or None
            CSV output path.  None = in-memory only (no disk flush).
        pool_size : int or None
            Number of pre-allocated StepRecord slots.  Drives both the
            in-memory record window and the disk-flush cadence (when
            path is set, one flush per pool-wrap).  The hot-loop zero-
            allocation contract requires pool_size ≤ warmup ticks for
            the measurement window to see a steady-state wrap — see
            ``controller/HOT_LOOP_CONTRACT.md``.
            None = ``_DEFAULT_POOL_SIZE`` (500).
        """
        ps = int(pool_size) if pool_size is not None else self._DEFAULT_POOL_SIZE
        if ps < 1:
            raise ValueError(f"pool_size must be >= 1 (got {ps})")
        # Pre-allocate the record pool once.  Each slot is a StepRecord
        # with its default zero values; ``next_record()`` returns slots
        # for in-place mutation and the caller is responsible for
        # overwriting every field relevant to the record.
        self._pool_size: int = ps
        self._pool: list[StepRecord] = [StepRecord() for _ in range(ps)]
        self._path = path
        self._total_count: int = 0
        self._write_idx: int = 0     # monotonic — number of records ever written
        self._flushed_idx: int = 0   # monotonic — number already persisted to disk
        self._header_written: bool = False

    @property
    def pool_size(self) -> int:
        """Number of records retained in memory (rolling window size)."""
        return self._pool_size

    # ------------------------------------------------------------------
    # Hot-loop-contract API
    # ------------------------------------------------------------------

    def next_record(self) -> StepRecord:
        """Return the next pool slot for in-place population — hot-loop body.

        Advances the monotonic write cursor.  The caller MUST fill every
        relevant field of the returned record before the next call to
        ``next_record()`` (the slot currently holds stale data from a
        previous cycle).  When ``path`` is set, triggers a disk flush
        just before the cursor would overwrite records not yet on disk.

        Contract-test relevance: this is the sole record-source call on
        the hot path.  Pre-allocation guarantees the slot is already
        allocated, so the only per-tick allocations inside the fill path
        are the Python floats being assigned into the slot's attributes.
        Once the pool wraps (after ``_POOL_SIZE`` writes), those
        allocations are matched by frees of the previous cycle's floats,
        so tracemalloc sees no net growth.
        """
        # Flush before we overwrite records not yet on disk.  Skips when
        # nothing needs flushing (path=None, or we haven't filled the
        # pool yet).
        if (self._path
                and self._write_idx - self._flushed_idx >= self._pool_size):
            self._flush_batch()
        slot = self._write_idx % self._pool_size
        self._write_idx += 1
        self._total_count += 1
        return self._pool[slot]

    def last_record(self) -> StepRecord | None:
        """Return the most recently written pool slot, or None if empty.

        Hot-loop-safe read path for the runner's post-log ``t_log_ms``
        stamp.  Does not allocate (unlike ``logger.records[-1]`` which
        materialises a fresh list).
        """
        if self._write_idx == 0:
            return None
        return self._pool[(self._write_idx - 1) % self._pool_size]

    # ------------------------------------------------------------------
    # Legacy API (backward-compat)
    # ------------------------------------------------------------------

    def append(self, record: StepRecord) -> None:
        """Copy an externally-constructed StepRecord into the pool.

        Allocates (field-by-field copy) and is NOT contract-compliant.
        Retained so sim/demo_mpc.py and sim/main.py continue to work
        without simultaneous migration.  Hot-path callers (run_mpc_loop)
        MUST use ``next_record()`` + ``fill_record_from_arrays()``.
        """
        slot = self.next_record()
        for f in fields(StepRecord):
            setattr(slot, f.name, getattr(record, f.name))

    @property
    def records(self) -> list[StepRecord]:
        """Recent records, oldest first.  Fresh list on every read.

        Safe to use after the run ends; NOT hot-path safe (list alloc).
        The hot-loop code path uses ``last_record()`` instead.
        """
        start = max(0, self._write_idx - self._pool_size)
        return [self._pool[i % self._pool_size]
                for i in range(start, self._write_idx)]

    @property
    def total_count(self) -> int:
        """Total number of records appended (including those already flushed)."""
        return self._total_count

    # ------------------------------------------------------------------
    # Persistence
    # ------------------------------------------------------------------

    def _flush_batch(self) -> None:
        """Write pool records that haven't yet been persisted to disk."""
        if not self._path or self._flushed_idx >= self._write_idx:
            return
        os.makedirs(os.path.dirname(self._path) or '.', exist_ok=True)
        field_names = [f.name for f in fields(StepRecord)]
        mode = 'a' if self._header_written else 'w'
        with open(self._path, mode, newline='') as f:
            writer = csv.DictWriter(f, fieldnames=field_names)
            if not self._header_written:
                writer.writeheader()
                self._header_written = True
            for i in range(self._flushed_idx, self._write_idx):
                writer.writerow(asdict(self._pool[i % self._pool_size]))
        self._flushed_idx = self._write_idx

    def flush(self) -> None:
        """Flush remaining in-memory records to disk."""
        if self._path and self._write_idx > self._flushed_idx:
            self._flush_batch()

    def load(self) -> list[StepRecord]:
        """Read the full history back from the CSV on disk.

        Automatically flushes any pending records before reading.
        Returns whatever is in memory if no path was set or the file
        doesn't exist.
        """
        self.flush()
        if not self._path or not os.path.exists(self._path):
            return list(self.records)  # fallback: return whatever is in memory
        return load_records(self._path)

    def close(self) -> None:
        self.flush()


# ---------------------------------------------------------------------------
# CSV loading
# ---------------------------------------------------------------------------

# StepRecord annotation -> converter restoring a CSV cell to its field type.
# Every StepRecord field must map to one of these; load_records() hard-fails
# otherwise, so a new field type can never silently round-trip as str.
_CSV_CONVERTERS = {
    'float': float,
    'int': lambda v: int(float(v)),  # tolerate historical '1.0'-style cells
    'str': str,
}


def _csv_converters() -> "dict[str, object]":
    convs = {}
    for f in fields(StepRecord):
        t = f.type if isinstance(f.type, str) else getattr(f.type, '__name__', repr(f.type))
        if t not in _CSV_CONVERTERS:
            raise TypeError(
                f"StepRecord.{f.name} is annotated {t!r}; the CSV round-trip "
                "supports float/int/str only - extend _CSV_CONVERTERS alongside "
                "the new field"
            )
        convs[f.name] = _CSV_CONVERTERS[t]
    return convs


def load_records(path: str) -> "list[StepRecord]":
    """Load a StepRecord CSV, restoring each column to its annotated type.

    The single canonical loader: TelemetryLogger.load() and the
    sim/analysis consumers (via analysis.compare.load_csv) all delegate
    here.  Unknown columns are ignored; columns absent from the file
    (older schemas) keep their StepRecord defaults.
    """
    convs = _csv_converters()
    records: "list[StepRecord]" = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            records.append(StepRecord(**{
                key: convs[key](val) for key, val in row.items() if key in convs
            }))
    return records


# ---------------------------------------------------------------------------
# Plotting utilities
# ---------------------------------------------------------------------------

def plot_tracking(records: Sequence[StepRecord], show: bool = True) -> None:
    """Plot reference vs actual for each DoF, plus tracking error over time.

    Requires matplotlib (imported lazily so the module loads without it).
    """
    import matplotlib.pyplot as plt

    times = [r.time for r in records]

    dof_labels = ['x (mm)', 'y (mm)', 'z (mm)', 'rx (rad)', 'ry (rad)', 'rz (rad)']
    ref_keys = ['ref_pose_x', 'ref_pose_y', 'ref_pose_z',
                'ref_pose_rx', 'ref_pose_ry', 'ref_pose_rz']
    act_keys = ['actual_pose_x', 'actual_pose_y', 'actual_pose_z',
                'actual_pose_rx', 'actual_pose_ry', 'actual_pose_rz']

    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    fig.suptitle('Reference vs Actual Pose')
    for idx, ax in enumerate(axes.flat):
        ref_vals = [getattr(r, ref_keys[idx]) for r in records]
        act_vals = [getattr(r, act_keys[idx]) for r in records]
        ax.plot(times, ref_vals, 'b--', label='ref', linewidth=1)
        ax.plot(times, act_vals, 'r-', label='actual', linewidth=1)
        ax.set_ylabel(dof_labels[idx])
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
    axes[-1, 0].set_xlabel('Time (s)')
    axes[-1, 1].set_xlabel('Time (s)')
    fig.tight_layout()

    # Tracking error plot
    fig2, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 5), sharex=True)
    fig2.suptitle('Tracking Error')
    ax1.plot(times, [r.tracking_error_mm for r in records], 'r-')
    ax1.set_ylabel('Position error (mm)')
    ax1.grid(True, alpha=0.3)
    ax2.plot(times, [r.tracking_error_deg for r in records], 'b-')
    ax2.set_ylabel('Orientation error (deg)')
    ax2.set_xlabel('Time (s)')
    ax2.grid(True, alpha=0.3)
    fig2.tight_layout()

    if show:
        plt.show()


def plot_solve_times(records: Sequence[StepRecord], show: bool = True) -> None:
    """Histogram of MPC solve times."""
    import matplotlib.pyplot as plt

    times_ms = [r.solve_time_ms for r in records if r.solve_time_ms > 0]
    if not times_ms:
        print("No solve time data to plot.")
        return

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.hist(times_ms, bins=50, edgecolor='black', alpha=0.7)
    ax.axvline(20.0, color='r', linestyle='--', label='20 ms budget')
    ax.set_xlabel('Solve time (ms)')
    ax.set_ylabel('Count')
    ax.set_title('MPC Solve Time Distribution')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    if show:
        plt.show()
