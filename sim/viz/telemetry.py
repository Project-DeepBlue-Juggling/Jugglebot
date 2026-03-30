"""Telemetry logging and plotting for the simulation harness.

Provides a structured per-step log record (StepRecord), a TelemetryLogger that
accumulates records and writes CSV, and plotting utilities for post-hoc analysis.

The same schema is used for sim and hardware — enables direct comparison in Phase 6.
"""

from __future__ import annotations

import csv
import os
from dataclasses import dataclass, field, fields, asdict
from typing import Sequence

import numpy as np


@dataclass
class StepRecord:
    """One row of telemetry, captured each MPC step (50 Hz)."""

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

    # MPC diagnostics (populated from Phase 2 onward)
    solve_time_ms: float = 0.0
    solve_status: str = "n/a"
    cost: float = 0.0
    constraint_violation: float = 0.0

    # Derived metrics
    tracking_error_mm: float = 0.0
    tracking_error_deg: float = 0.0


def record_from_arrays(
    time: float,
    ref_pose: np.ndarray,
    ref_twist: np.ndarray,
    actual_pose: np.ndarray,
    actual_twist: np.ndarray,
    cmd_extensions: np.ndarray,
    actual_extensions: np.ndarray,
    leg_velocities: np.ndarray | None = None,
    hand_cmd_mm: float = 0.0,
    hand_pos_mm: float = 0.0,
    hand_vel_mmps: float = 0.0,
    solve_time_ms: float = 0.0,
    solve_status: str = "n/a",
    cost: float = 0.0,
    constraint_violation: float = 0.0,
) -> StepRecord:
    """Build a StepRecord from numpy arrays (convenience helper)."""
    pos_err = np.linalg.norm(actual_pose[:3] - ref_pose[:3])
    ori_err = np.degrees(np.linalg.norm(actual_pose[3:] - ref_pose[3:]))
    lv = leg_velocities if leg_velocities is not None else np.zeros(6)

    return StepRecord(
        time=time,
        ref_pose_x=ref_pose[0], ref_pose_y=ref_pose[1], ref_pose_z=ref_pose[2],
        ref_pose_rx=ref_pose[3], ref_pose_ry=ref_pose[4], ref_pose_rz=ref_pose[5],
        ref_twist_vx=ref_twist[0], ref_twist_vy=ref_twist[1], ref_twist_vz=ref_twist[2],
        ref_twist_wx=ref_twist[3], ref_twist_wy=ref_twist[4], ref_twist_wz=ref_twist[5],
        actual_pose_x=actual_pose[0], actual_pose_y=actual_pose[1], actual_pose_z=actual_pose[2],
        actual_pose_rx=actual_pose[3], actual_pose_ry=actual_pose[4], actual_pose_rz=actual_pose[5],
        actual_twist_vx=actual_twist[0], actual_twist_vy=actual_twist[1], actual_twist_vz=actual_twist[2],
        actual_twist_wx=actual_twist[3], actual_twist_wy=actual_twist[4], actual_twist_wz=actual_twist[5],
        cmd_ext_0=cmd_extensions[0], cmd_ext_1=cmd_extensions[1], cmd_ext_2=cmd_extensions[2],
        cmd_ext_3=cmd_extensions[3], cmd_ext_4=cmd_extensions[4], cmd_ext_5=cmd_extensions[5],
        actual_ext_0=actual_extensions[0], actual_ext_1=actual_extensions[1], actual_ext_2=actual_extensions[2],
        actual_ext_3=actual_extensions[3], actual_ext_4=actual_extensions[4], actual_ext_5=actual_extensions[5],
        leg_vel_0=lv[0], leg_vel_1=lv[1], leg_vel_2=lv[2],
        leg_vel_3=lv[3], leg_vel_4=lv[4], leg_vel_5=lv[5],
        hand_cmd_mm=hand_cmd_mm, hand_pos_mm=hand_pos_mm,
        hand_vel_mmps=hand_vel_mmps,
        solve_time_ms=solve_time_ms, solve_status=solve_status,
        cost=cost, constraint_violation=constraint_violation,
        tracking_error_mm=pos_err, tracking_error_deg=ori_err,
    )


class TelemetryLogger:
    """Accumulates StepRecords and periodically flushes them to CSV.

    Records are written to disk in batches to bound memory usage and provide
    crash safety.  A rolling tail of recent records is kept in memory for
    end-of-run summary stats.  Use :meth:`load` to read the full history back
    from the CSV after the run.

    Usage::

        logger = TelemetryLogger("logs/run_001.csv")
        for step in sim_loop:
            logger.append(record)
        logger.flush()          # writes remaining records to disk
        all_records = logger.load()  # read full history from CSV
    """

    _FLUSH_EVERY: int = 5000   # records between disk writes
    _TAIL_SIZE: int = 200      # keep last N in memory for summary stats

    def __init__(self, path: str | None = None):
        self._records: list[StepRecord] = []
        self._path = path
        self._total_count: int = 0
        self._header_written: bool = False
        self._flushed_tail: int = 0  # number of leading records already on disk

    def append(self, record: StepRecord) -> None:
        self._records.append(record)
        self._total_count += 1
        if self._path and len(self._records) >= self._FLUSH_EVERY:
            self._flush_batch()

    @property
    def records(self) -> list[StepRecord]:
        """Recent records (rolling window).  Use load() for full history."""
        return self._records

    @property
    def total_count(self) -> int:
        """Total number of records appended (including those already flushed)."""
        return self._total_count

    def _flush_batch(self) -> None:
        """Append new records to CSV, then trim to tail."""
        if not self._path or not self._records:
            return
        new_records = self._records[self._flushed_tail:]
        if not new_records:
            return
        os.makedirs(os.path.dirname(self._path) or '.', exist_ok=True)
        field_names = [f.name for f in fields(StepRecord)]
        mode = 'a' if self._header_written else 'w'
        with open(self._path, mode, newline='') as f:
            writer = csv.DictWriter(f, fieldnames=field_names)
            if not self._header_written:
                writer.writeheader()
                self._header_written = True
            for rec in new_records:
                writer.writerow(asdict(rec))
        self._records = self._records[-self._TAIL_SIZE:]
        self._flushed_tail = len(self._records)

    def flush(self) -> None:
        """Flush remaining in-memory records to disk."""
        if self._path and self._records:
            self._flush_batch()

    def load(self) -> list[StepRecord]:
        """Read the full history back from the CSV on disk.

        Automatically flushes any pending records before reading.
        Returns whatever is in memory if no path was set or the file
        doesn't exist.
        """
        self.flush()
        if not self._path or not os.path.exists(self._path):
            return list(self._records)  # fallback: return whatever is in memory
        out: list[StepRecord] = []
        field_names = {f.name for f in fields(StepRecord)}
        float_fields = {f.name for f in fields(StepRecord) if f.type == 'float'}
        with open(self._path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                kwargs = {}
                for key, val in row.items():
                    if key not in field_names:
                        continue
                    if key in float_fields:
                        kwargs[key] = float(val)
                    else:
                        kwargs[key] = val
                out.append(StepRecord(**kwargs))
        return out

    def close(self) -> None:
        self.flush()


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
