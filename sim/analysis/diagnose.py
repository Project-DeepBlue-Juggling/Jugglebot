# -*- coding: utf-8 -*-
"""Hardware diagnosis analysis engine.

Reads MPC telemetry CSVs and rosbag (MCAP) recordings to produce a structured
JSON diagnostic report.  Designed to be invoked by the /diagnose Claude Code
slash command, which interprets the output and presents it to the user.

Primary data sources:
  - MPC telemetry CSV (temp/logs/mpc_*.csv) — 55-field StepRecord at 40 Hz
  - Rosbag MCAP (~/Desktop/rosbags/<timestamp>/) — 19 ROS2 topics recorded
    automatically by the launch file

Note: ROS2 Foxy does NOT write per-node text log files (that's Humble+).
The --ros-log-dir option is retained for forward compatibility but is not
part of the standard diagnosis workflow.

Usage:
    python sim/analysis/diagnose.py <csv_path> --json
    python sim/analysis/diagnose.py <csv_path> --rosbag <path> --json
    python sim/analysis/diagnose.py <csv_path> --ros-log-dir <path> --json  (optional)

Runs on Python 3.8+ (Jetson).  Requires: numpy (optional: pyyaml, rosbags).
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import re
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

# Single path bootstrap (repo root, ros_ws pkg, config/generated);
# see sim/_paths.py.  Runnable entry scripts only — library modules under
# sim/ never touch sys.path.
_repo_root = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
from sim._paths import bootstrap_paths  # noqa: E402
bootstrap_paths()

from sim.analysis.compare import load_csv, estimate_tau
from sim.analysis.plot_diagnosis import generate_diagnostic_plots, parse_categories
from sim.viz.telemetry import StepRecord


# ---------------------------------------------------------------------------
# Configuration / thresholds
# ---------------------------------------------------------------------------

# Stroke limits (mm) — from hardware_config
STROKE_MIN_MM = 5.0
STROKE_MAX_MM = 275.0
STROKE_SOFT_MARGIN_MM = 15.0

# MPC solve budget (ms) — at 40 Hz
DEFAULT_BUDGET_MS = 24.0

# Oscillation detection
CHATTER_THRESHOLD = 0.5          # ratio of sign changes / samples
AMPLITUDE_GROWTH_WINDOW = 10     # samples for envelope regression

# Discontinuity detection (mm)
CMD_JUMP_THRESHOLD_MM = 5.0
ACTUAL_JUMP_THRESHOLD_MM = 2.0

# Tracking thresholds
WORST_LEG_RATIO_THRESHOLD = 1.5
STEADY_STATE_RMS_THRESHOLD_MM = 0.5

# Reference change detection (mm for position, rad for orientation)
REF_CHANGE_THRESHOLD = 0.01

# Plant telemetry collapse detection.  v_max mirrors
# controller.params.MPCParams.max_leg_vel_mmps (the rate cap MPC enforces).
# Free-fall signature: all 6 legs retract together at mean speed beyond the
# rate cap (the MPC couldn't command that profile, so it's uncommanded).
# The "all legs exactly 0.0" sentinel is what HardwarePlant's ZMQ SUB hands
# back when telemetry stops flowing.
V_MAX_MM_S = 140.0
PLANT_COLLAPSE_MEAN_SPEED_MM_S = V_MAX_MM_S  # mean(|leg_vel|) across 6 legs
PLANT_COLLAPSE_SAME_SIGN_MIN = 5             # at least this many legs moving same direction
PLANT_COLLAPSE_ZERO_RUN_MIN = 20             # consecutive all-zero-extension steps
PLANT_COLLAPSE_WINDOW_S = 0.5                # overspeed must precede zero-run within this window

# Overshoot-induced NLP saturation detection.  Signature: IPOPT hits the
# CPU cap before finishing one iteration while the plant pose is far from
# the reference and the reference is still moving.  Distinct from generic
# MPC_STALENESS (which has no overshoot context).
OVERSHOOT_ZERO_ITER_RUN_MIN = 10         # consecutive ipopt_iter==0 steps
OVERSHOOT_TRACKING_ERR_MM = 10.0         # tracking_error_mm must exceed this in the run

# Hold-phase quiescence detection.  Identifies the longest run in which the
# reference pose is stationary, skips a settling period, then reports per-leg
# noise stats on the TAIL of that run — the Level-1 gain-tuning metric.
# Settle-skip matters: the platform takes a second or two to converge on the
# new ref after it becomes static, and including that transient contaminates
# the stdev numbers.
HOLD_REF_STATIC_TOL = 0.01               # same scale as REF_CHANGE_THRESHOLD
HOLD_SETTLE_SKIP_S = 2.0                 # drop this many seconds after ref becomes static
HOLD_MIN_TAIL_S = 3.0                    # minimum useful tail after skip
HOLD_MAX_TAIL_S = 5.0                    # cap the measurement window (stats plateau quickly)
assert HOLD_MAX_TAIL_S >= HOLD_MIN_TAIL_S, (
    "HOLD_MAX_TAIL_S must be >= HOLD_MIN_TAIL_S, otherwise analyse_hold_phase "
    "truncates every tail below the minimum and detected is never True."
)
HOLD_ASYMMETRY_WARN = 1.5                # worst/quietest ratio > this triggers a flag
HOLD_LEG_STD_WARN_UM = 20.0              # per-leg act_std above this triggers a flag

# MPC loop overhead (dt minus solve_time) analysis.  Used to separate
# solver-induced latency from other causes (GC pauses, interpreter
# housekeeping, logging, ZMQ drain surges).
OVERHEAD_ISOLATED_RATIO = 3.0            # spike >= this × local neighbourhood
OVERHEAD_NEIGHBOUR_WINDOW = 5            # ticks on each side for neighbourhood mean
OVERHEAD_MIN_MS_FOR_SPIKE = 15.0         # floor: ignore below this absolute level
OVERHEAD_BUDGET_MS = 10.0                # typical well-behaved overhead ceiling

# cmd_ext spectral content during hold.  If >HF_ENERGY_WARN_PCT of the cmd
# energy sits above 10 Hz, flag it so the operator doesn't mistake MPC
# numerical-noise-floor jitter for a bug.
CMD_SPECTRAL_HF_HZ = 10.0
CMD_SPECTRAL_MID_HZ = 2.0
HF_ENERGY_WARN_PCT = 25.0

# Motion-onset dead-time detection.  Separates the "cmd has started ramping
# but actual hasn't moved yet" silence from the stick-slip leap that follows.
# Per-leg: find each hold→ramp transition in cmd_ext, then measure how long
# until actual_ext diverges from the pre-onset hold position by more than
# MOTION_DETECTION_THRESHOLD_MM.  The 6-leg sync window tells us whether
# legs break free together (shared breakaway) or staggered (per-leg μ_s
# asymmetry).
#
# Thresholds are picked so the detector fires on the Apr 18 signature
# (~120 ms dead-time, 2 mm first-tick leap, all 6 legs within 25-50 ms)
# without tripping on steady tracking or ordinary hold-phase noise.
MOTION_ONSET_HOLD_VEL_MMPS = 1.0          # |cmd_vel| below this counts as hold
MOTION_ONSET_RAMP_VEL_MMPS = 5.0          # |cmd_vel| above this counts as ramp
MOTION_ONSET_PREHOLD_SAMPLES = 8          # 8 × 25 ms = 200 ms of sustained hold required
MOTION_ONSET_SMOOTH_SAMPLES = 3           # rolling-mean window on cmd-vel
MOTION_DETECTION_THRESHOLD_MM = 0.5       # actual deviation from hold-pos to count as "moved"
MOTION_ONSET_SYNC_WINDOW_MS = 100.0       # group per-leg onsets within this window into one event
MOTION_ONSET_SEARCH_WINDOW_S = 1.0        # per onset, give up after this long if actual never moves
MOTION_ONSET_LATENCY_WARN_MS = 40.0       # latency above this triggers a warning flag
MOTION_ONSET_LATENCY_ERROR_MS = 80.0      # latency above this triggers an error flag

# Motor/leg parameters loaded from hardware_config.yaml for electrical-angle
# derivation.  Populated lazily on first access so diagnose.py remains usable
# when pyyaml is unavailable (fields return None and downstream study code
# skips cogging-angle emission rather than failing).
_MOTOR_PARAMS_CACHE: Optional[Dict[str, Any]] = None


def _load_motor_params() -> Dict[str, Any]:
    """Load motor_pole_pairs / motor_stator_slots / mm_to_rev[6] from YAML.

    Returns a dict with keys {'pole_pairs', 'stator_slots', 'mm_to_rev'}.
    Any missing key is set to None.  Result is cached.
    """
    global _MOTOR_PARAMS_CACHE
    if _MOTOR_PARAMS_CACHE is not None:
        return _MOTOR_PARAMS_CACHE
    params: Dict[str, Any] = {'pole_pairs': None, 'stator_slots': None, 'mm_to_rev': None}
    try:
        import yaml  # type: ignore
        yaml_path = os.path.join(_repo_root, 'config', 'hardware_config.yaml')
        with open(yaml_path, 'r') as f:
            cfg = yaml.safe_load(f)
        dynamics = cfg.get('dynamics', {}) or {}
        geometry = cfg.get('jugglebot_geometry', {}) or {}
        params['pole_pairs'] = dynamics.get('motor_pole_pairs')
        params['stator_slots'] = dynamics.get('motor_stator_slots')
        params['mm_to_rev'] = geometry.get('mm_to_rev')
    except Exception:
        pass
    _MOTOR_PARAMS_CACHE = params
    return params


# ---------------------------------------------------------------------------
# MPC CSV analysis
# ---------------------------------------------------------------------------

def _extract(records: List[StepRecord], attr: str) -> np.ndarray:
    """Extract a single attribute as a numpy array."""
    return np.array([getattr(r, attr) for r in records])


def analyse_tracking(records: List[StepRecord]) -> Dict[str, Any]:
    """Per-leg and aggregate tracking error analysis."""
    n = len(records)
    if n == 0:
        return {}

    # Per-leg tracking: cmd_ext vs actual_ext
    per_leg = []
    leg_rms_values = []
    for leg in range(6):
        cmd = _extract(records, f'cmd_ext_{leg}')
        actual = _extract(records, f'actual_ext_{leg}')
        error = cmd - actual
        rms = float(np.sqrt(np.mean(error**2)))
        peak = float(np.max(np.abs(error)))
        mean = float(np.mean(np.abs(error)))
        per_leg.append({
            'leg': leg,
            'rms_mm': round(rms, 4),
            'peak_mm': round(peak, 4),
            'mean_mm': round(mean, 4),
        })
        leg_rms_values.append(rms)

    # Worst leg analysis
    median_rms = float(np.median(leg_rms_values))
    worst_idx = int(np.argmax(leg_rms_values))
    worst_rms = leg_rms_values[worst_idx]
    worst_ratio = worst_rms / median_rms if median_rms > 1e-6 else 0.0

    # Aggregate position and orientation error
    pos_err = _extract(records, 'tracking_error_mm')
    ori_err = _extract(records, 'tracking_error_deg')

    return {
        'position_rms_mm': round(float(np.sqrt(np.mean(pos_err**2))), 4),
        'position_peak_mm': round(float(np.max(pos_err)), 4),
        'position_p95_mm': round(float(np.percentile(pos_err, 95)), 4),
        'orientation_rms_deg': round(float(np.sqrt(np.mean(ori_err**2))), 4),
        'orientation_peak_deg': round(float(np.max(ori_err)), 4),
        'per_leg': per_leg,
        'worst_leg': worst_idx,
        'worst_leg_ratio': round(worst_ratio, 2),
        'median_leg_rms_mm': round(median_rms, 4),
    }


def analyse_solve_times(records: List[StepRecord],
                        budget_ms: float = DEFAULT_BUDGET_MS) -> Dict[str, Any]:
    """MPC solve time distribution analysis."""
    solve_ms = _extract(records, 'solve_time_ms')
    valid = solve_ms[solve_ms > 0]

    if len(valid) == 0:
        return {'n_valid': 0}

    # Budget violations
    violations = valid > budget_ms
    # Consecutive violations
    max_consecutive = 0
    current_run = 0
    for v in violations:
        if v:
            current_run += 1
            max_consecutive = max(max_consecutive, current_run)
        else:
            current_run = 0

    return {
        'n_valid': int(len(valid)),
        'p50_ms': round(float(np.median(valid)), 2),
        'p95_ms': round(float(np.percentile(valid, 95)), 2),
        'p99_ms': round(float(np.percentile(valid, 99)), 2),
        'max_ms': round(float(np.max(valid)), 2),
        'budget_ms': budget_ms,
        'budget_violations': int(np.sum(violations)),
        'budget_violation_pct': round(float(np.mean(violations) * 100), 2),
        'max_consecutive_violations': max_consecutive,
        'first_sample_ms': round(float(valid[0]), 2) if len(valid) > 0 else 0,
    }


def analyse_solver_status(records: List[StepRecord]) -> Dict[str, Any]:
    """Tally the MPC solver-status column — the richer signal than solve time alone.

    A solve can land under the time budget and still be a fallback/hold (e.g.
    Maximum_CpuTime_Exceeded that IPOPT returned slightly under the cap), or
    land over budget and still be a successful solve.  The `solve_status`
    column captures what the solver ACTUALLY returned each step:

      - 'Solve_Succeeded' / 'Solved_To_Acceptable_Level' — good solve
      - 'fallback(<reason>)'  — applied shifted warm-start (previous trajectory)
      - 'hold(<reason>)'      — held previous command (degraded fallback)
      - 'cold_hold(<reason>)' — first-solve failure; held at q_cur

    This is the structured equivalent of the MPC stdout 'solve failed' log
    lines, but per-step and already in the CSV — no stdout tee required.
    """
    if not records:
        return {'total': 0}

    statuses = [r.solve_status or 'n/a' for r in records]
    from collections import Counter
    by_status = Counter(statuses)

    total = len(statuses)
    succeeded = sum(1 for s in statuses
                    if s in ('Solve_Succeeded', 'Solved_To_Acceptable_Level'))

    def _classify(s: str) -> str:
        if s.startswith('hold('):
            return 'hold'
        if s.startswith('cold_hold('):
            return 'cold_hold'
        if s.startswith('fallback('):
            return 'fallback'
        if s in ('Solve_Succeeded', 'Solved_To_Acceptable_Level'):
            return 'success'
        return 'other'

    class_counts = Counter(_classify(s) for s in statuses)

    # Consecutive non-success runs (any non-success)
    max_consec_nonsuccess = 0
    cur = 0
    for s in statuses:
        if _classify(s) != 'success':
            cur += 1
            max_consec_nonsuccess = max(max_consec_nonsuccess, cur)
        else:
            cur = 0

    # Consecutive timeouts (hold/fallback/cold_hold with Maximum_CpuTime_Exceeded)
    def _is_timeout(s: str) -> bool:
        return 'Maximum_CpuTime_Exceeded' in s and _classify(s) != 'success'

    max_consec_timeout = 0
    cur = 0
    timeout_total = 0
    for s in statuses:
        if _is_timeout(s):
            cur += 1
            timeout_total += 1
            max_consec_timeout = max(max_consec_timeout, cur)
        else:
            cur = 0

    # Other (non-timeout) failure reasons — surface them for investigation
    other_reasons: Dict[str, int] = {}
    for s, n in by_status.items():
        if _classify(s) != 'success' and not _is_timeout(s):
            other_reasons[s] = n

    return {
        'total': total,
        'succeeded': succeeded,
        'success_rate_pct': round(100.0 * succeeded / total, 2) if total else 0.0,
        'by_class': dict(class_counts),
        'by_status': dict(by_status),
        'timeout_total': timeout_total,
        'timeout_pct': round(100.0 * timeout_total / total, 2) if total else 0.0,
        'max_consecutive_non_success': max_consec_nonsuccess,
        'max_consecutive_timeout': max_consec_timeout,
        'other_failure_reasons': other_reasons,
    }


def analyse_oscillation(records: List[StepRecord]) -> Dict[str, Any]:
    """Detect oscillation via chatter ratio (sign changes in command deltas).

    Two noise sources are filtered before the sign-change count:

    1. **Sub-LSB float noise.**  On a steady-state hold the MPC's cmd_ext
       values drift by microns at every tick — a float-rounding signature
       of the IPOPT solver, not physical oscillation.  ``np.sign`` promotes
       any tiny non-zero delta to ±1 and ~half of those flip signs,
       producing a spurious chatter ratio near 0.5 on clean holds.  The
       motor guard's dead-band (``1 / (enc_cpr * mm_to_rev)`` — see
       controller/hardware_plant.py) filters these before CAN transmit,
       so they are not physically observable.  We apply the same floor:
       deltas with |Δ| below ``_CHATTER_MIN_DELTA_MM`` are treated as
       "no change" and excluded from both the numerator (sign changes)
       and denominator (non-zero delta count).

    2. **Scheduled-GC inter-tick gaps.**  The MPC loop's W5 scheduled
       ``gc.collect(2)`` produces a ~149 ms wall-clock gap once per 30 s
       (see logbook 2026-04-23-hot-loop-zero-allocation-contract).  The
       associated delta can be large (motion accumulated during the gap),
       which would corrupt the amplitude_growing slope fit.  Samples with
       ``dt > 5× median`` are masked out before the envelope regression.

    Without (1), the W6 hold-at-ACTIVE session reported "Oscillation
    detected on all 6 legs" with chatter ~0.5 despite an operator-verified
    clean ODrive pos_setpoint trace.  Without (2), a handful of GC-gap
    deltas inflate the amplitude-growth slope.
    """
    # Chatter magnitude floor.  One encoder LSB on a typical leg is
    # ~1 / (8192 counts/rev × 8 rev/mm) ≈ 15 µm; 10 µm sits just under
    # that, capturing sub-LSB solver noise while leaving any genuinely
    # physical oscillation (even at encoder resolution) above the floor.
    # Hard-coded here rather than pulling from hardware_config to keep
    # sim/analysis/ independent of ros_ws/.  If enc_cpr or mm_to_rev
    # ever changes, recompute.
    _CHATTER_MIN_DELTA_MM = 0.01
    n = len(records)
    if n < 10:
        return {'detected': False, 'per_leg_chatter': [], 'per_dof_chatter': []}

    # Build a mask over the diff indices: keep index i iff records[i+1].time
    # - records[i].time is within 5× the median dt.  Median over mean is
    # deliberate — mean is dragged by the GC gaps we want to ignore.
    times = np.asarray([r.time for r in records], dtype=float)
    dt_series = np.diff(times)
    if len(dt_series) >= 2:
        median_dt = float(np.median(dt_series))
        # Fall back to "no mask" if median is degenerate (constant time
        # field, e.g. a synthetic fixture).  Otherwise a 5× envelope keeps
        # real jitter (pacing slack up to ~2× expected) and rejects only
        # the scheduled-GC outliers (typically 6× expected).
        if median_dt > 1e-9:
            dt_mask = dt_series <= 5.0 * median_dt
        else:
            dt_mask = np.ones(len(dt_series), dtype=bool)
    else:
        dt_mask = np.ones(len(dt_series), dtype=bool) if len(dt_series) else \
            np.array([], dtype=bool)

    # Per-leg chatter (command extensions)
    per_leg_chatter = []
    for leg in range(6):
        cmd = _extract(records, f'cmd_ext_{leg}')
        delta = np.diff(cmd)
        if len(delta) < 2:
            per_leg_chatter.append(0.0)
            continue
        # Magnitude floor: sub-LSB deltas are treated as "no change".
        signs = np.sign(delta)
        signs[np.abs(delta) < _CHATTER_MIN_DELTA_MM] = 0
        nonzero = signs[signs != 0]
        if len(nonzero) < 2:
            per_leg_chatter.append(0.0)
            continue
        sign_changes = np.sum(np.abs(np.diff(nonzero)) > 0)
        chatter = float(sign_changes / len(nonzero))
        per_leg_chatter.append(round(chatter, 3))

    # Per-DoF chatter (actual pose).  Pose is in mm / rad — the same
    # _CHATTER_MIN_DELTA_MM floor is a reasonable lower bound for mm
    # translations; rotation columns see ~100× smaller deltas (rad scale)
    # so the floor effectively zeros rotation chatter unless there's real
    # motion.  Good default; revisit if the diagnostic ever needs to
    # distinguish genuine sub-milliradian rotation chatter.
    dof_attrs = ['actual_pose_x', 'actual_pose_y', 'actual_pose_z',
                 'actual_pose_rx', 'actual_pose_ry', 'actual_pose_rz']
    per_dof_chatter = []
    for attr in dof_attrs:
        vals = _extract(records, attr)
        delta = np.diff(vals)
        if len(delta) < 2:
            per_dof_chatter.append(0.0)
            continue
        signs = np.sign(delta)
        signs[np.abs(delta) < _CHATTER_MIN_DELTA_MM] = 0
        nonzero = signs[signs != 0]
        if len(nonzero) < 2:
            per_dof_chatter.append(0.0)
            continue
        sign_changes = np.sum(np.abs(np.diff(nonzero)) > 0)
        per_dof_chatter.append(round(float(sign_changes / len(nonzero)), 3))

    # Amplitude growth detection for legs with high chatter
    amplitude_growing = False
    for leg in range(6):
        if per_leg_chatter[leg] > CHATTER_THRESHOLD:
            cmd = _extract(records, f'cmd_ext_{leg}')
            delta = np.diff(cmd)
            # Apply the same dt-outlier mask so the GC-gap delta doesn't
            # show up as a spurious envelope peak.
            delta = delta[dt_mask] if len(delta) == len(dt_mask) else delta
            # Compute rolling envelope (abs of delta)
            envelope = np.abs(delta)
            if len(envelope) > AMPLITUDE_GROWTH_WINDOW * 2:
                # Simple linear regression on envelope
                x = np.arange(len(envelope))
                slope = np.polyfit(x, envelope, 1)[0]
                if slope > 0.001:  # positive slope = growing
                    amplitude_growing = True

    detected = any(c > CHATTER_THRESHOLD for c in per_leg_chatter)

    return {
        'detected': detected,
        'amplitude_growing': amplitude_growing,
        'per_leg_chatter': per_leg_chatter,
        'per_dof_chatter': per_dof_chatter,
    }


def analyse_discontinuities(records: List[StepRecord]) -> Dict[str, Any]:
    """Detect single-step jumps in commanded or actual extensions."""
    times = _extract(records, 'time')
    cmd_jumps = []
    actual_jumps = []

    for leg in range(6):
        cmd = _extract(records, f'cmd_ext_{leg}')
        actual = _extract(records, f'actual_ext_{leg}')

        cmd_delta = np.abs(np.diff(cmd))
        actual_delta = np.abs(np.diff(actual))

        for i in np.where(cmd_delta > CMD_JUMP_THRESHOLD_MM)[0]:
            cmd_jumps.append({
                'leg': leg,
                'time_s': round(float(times[i + 1]), 4),
                'magnitude_mm': round(float(cmd_delta[i]), 2),
            })

        for i in np.where(actual_delta > ACTUAL_JUMP_THRESHOLD_MM)[0]:
            actual_jumps.append({
                'leg': leg,
                'time_s': round(float(times[i + 1]), 4),
                'magnitude_mm': round(float(actual_delta[i]), 2),
            })

    return {
        'cmd_jumps': sorted(cmd_jumps, key=lambda x: x['time_s']),
        'actual_jumps': sorted(actual_jumps, key=lambda x: x['time_s']),
    }


def _rolling_mean(x: np.ndarray, w: int) -> np.ndarray:
    """Centred rolling mean, edge-extended so output has the same length as x."""
    if w <= 1 or len(x) < w:
        return x.astype(float, copy=True)
    kernel = np.ones(w, dtype=float) / float(w)
    return np.convolve(x, kernel, mode='same')


def analyse_motion_onset(records: List[StepRecord]) -> Dict[str, Any]:
    """Measure hold→ramp motion-onset dead-time per leg.

    At each transition where cmd_ext emerges from a sustained hold into a
    ramp, records how long (ms) until actual_ext diverges from the pre-onset
    hold position by more than MOTION_DETECTION_THRESHOLD_MM.  Groups
    per-leg onsets that fall within MOTION_ONSET_SYNC_WINDOW_MS of each
    other into a single synchronised event (expected when all legs share
    the same breakaway cadence).

    Distinct from analyse_discontinuities: that detects the leap itself
    (>=2 mm in one tick), this detects the silence before it.
    """
    n = len(records)
    if n < MOTION_ONSET_PREHOLD_SAMPLES + 4:
        return {'detected': False, 'events': [], 'per_leg_summary': {}}

    times = _extract(records, 'time')
    dt_median = float(np.median(np.diff(times))) if n > 1 else 0.025
    if dt_median <= 0:
        return {'detected': False, 'events': [], 'per_leg_summary': {}}

    # Per-leg onset detection -------------------------------------------
    per_leg_onsets: Dict[int, List[Dict[str, Any]]] = {leg: [] for leg in range(6)}

    search_span = int(MOTION_ONSET_SEARCH_WINDOW_S / dt_median)

    # Motor params for electrical-angle derivation.  If YAML / pyyaml unavailable,
    # angle fields are emitted as None rather than skipped, so downstream tools
    # can see the sample count without guessing the schema.
    motor_params = _load_motor_params()
    pole_pairs = motor_params.get('pole_pairs')
    mm_to_rev_list = motor_params.get('mm_to_rev') or [None] * 6

    for leg in range(6):
        cmd = _extract(records, f'cmd_ext_{leg}')
        actual = _extract(records, f'actual_ext_{leg}')
        mm_to_rev_leg = mm_to_rev_list[leg] if leg < len(mm_to_rev_list) else None
        cmd_vel = np.diff(cmd) / dt_median                  # length n-1, mm/s
        cmd_vel_smooth = _rolling_mean(np.abs(cmd_vel),
                                       MOTION_ONSET_SMOOTH_SAMPLES)

        is_hold = cmd_vel_smooth < MOTION_ONSET_HOLD_VEL_MMPS
        is_ramp = cmd_vel_smooth >= MOTION_ONSET_RAMP_VEL_MMPS

        # Build list of ramp-onset indices (into cmd_vel / cmd_vel_smooth).
        # Case A: the session starts with cmd already ramping — the preceding
        # hold happened before the MPC logger came up.  Treat sample 0 as an
        # implicit onset.  Case B: mid-session hold→ramp transitions with a
        # sustained preceding hold of >= MOTION_ONSET_PREHOLD_SAMPLES.
        onset_indices: List[int] = []
        if len(is_ramp) > 0 and is_ramp[0]:
            onset_indices.append(0)

        hold_run = 0
        last_onset = -10**9
        for i in range(len(cmd_vel_smooth)):
            if is_hold[i]:
                hold_run += 1
            elif is_ramp[i]:
                if (hold_run >= MOTION_ONSET_PREHOLD_SAMPLES
                        and i - last_onset > search_span):
                    onset_indices.append(i)
                    last_onset = i
                hold_run = 0
            # else: in the transition band — don't reset the hold counter.

        for onset_cmd_idx in onset_indices:
            # cmd_vel[i] reflects delta between sample i and i+1; the onset's
            # anchoring record-index is i+1 in the record series.  At sample 0
            # (pre-capture hold), use sample 0 directly.
            record_idx = onset_cmd_idx if onset_cmd_idx == 0 else onset_cmd_idx + 1
            onset_time = float(times[record_idx])
            # Hold-position reference: mean of up-to-8 samples before onset.
            # If pre-capture (record_idx==0), use the first actual sample.
            ref_lo = max(0, record_idx - MOTION_ONSET_PREHOLD_SAMPLES)
            hold_pos = float(np.mean(actual[ref_lo:record_idx + 1]))

            search_limit = min(len(actual), record_idx + search_span)
            moving_idx = None
            for j in range(record_idx + 1, search_limit):
                if abs(actual[j] - hold_pos) > MOTION_DETECTION_THRESHOLD_MM:
                    moving_idx = j
                    break

            if moving_idx is not None:
                latency_ms = float((times[moving_idx] - times[record_idx]) * 1000.0)
                leap_mm = float(abs(actual[moving_idx] - actual[moving_idx - 1]))
                # Rest angles: derived from the "stuck" sample immediately before
                # the leap.  Use hold_pos_mm (the averaged pre-onset reference)
                # rather than a single sample — it's the position the motor was
                # actually parked at, filtered against encoder LSB jitter.
                rest_elec_angle_rad: Optional[float] = None
                rest_mech_angle_rad: Optional[float] = None
                if pole_pairs is not None and mm_to_rev_leg is not None:
                    motor_rev = hold_pos * mm_to_rev_leg
                    rest_mech_angle_rad = float((motor_rev * 2.0 * np.pi) % (2.0 * np.pi))
                    rest_elec_angle_rad = float((motor_rev * pole_pairs * 2.0 * np.pi)
                                                 % (2.0 * np.pi))
                per_leg_onsets[leg].append({
                    'onset_time_s': round(onset_time, 4),
                    'latency_ms': round(latency_ms, 1),
                    'first_tick_leap_mm': round(leap_mm, 3),
                    'hold_pos_mm': round(hold_pos, 3),
                    'moving_time_s': round(float(times[moving_idx]), 4),
                    'pre_capture_hold': onset_cmd_idx == 0,
                    'rest_elec_angle_rad': (round(rest_elec_angle_rad, 4)
                                            if rest_elec_angle_rad is not None else None),
                    'rest_mech_angle_rad': (round(rest_mech_angle_rad, 4)
                                            if rest_mech_angle_rad is not None else None),
                })

    # Group onsets across legs into synchronised events -----------------
    all_onsets: List[Tuple[float, int, Dict[str, Any]]] = []
    for leg, lst in per_leg_onsets.items():
        for o in lst:
            all_onsets.append((o['onset_time_s'], leg, o))
    all_onsets.sort(key=lambda x: x[0])

    events: List[Dict[str, Any]] = []
    used = [False] * len(all_onsets)
    sync_window_s = MOTION_ONSET_SYNC_WINDOW_MS / 1000.0
    for i, (t0, leg0, o0) in enumerate(all_onsets):
        if used[i]:
            continue
        group = [(leg0, o0)]
        used[i] = True
        for j in range(i + 1, len(all_onsets)):
            if used[j]:
                continue
            tj, legj, oj = all_onsets[j]
            if tj - t0 > sync_window_s:
                break
            if any(g[0] == legj for g in group):
                continue  # same leg already in this event
            group.append((legj, oj))
            used[j] = True

        latencies = [o['latency_ms'] for _, o in group]
        leaps = [o['first_tick_leap_mm'] for _, o in group]
        onset_times = [o['onset_time_s'] for _, o in group]
        per_leg = {leg: {'latency_ms': o['latency_ms'],
                         'first_tick_leap_mm': o['first_tick_leap_mm'],
                         'hold_pos_mm': o['hold_pos_mm'],
                         'rest_elec_angle_rad': o.get('rest_elec_angle_rad'),
                         'rest_mech_angle_rad': o.get('rest_mech_angle_rad')}
                   for leg, o in group}
        events.append({
            'onset_time_s': round(t0, 4),
            'legs_involved': sorted(leg for leg, _ in group),
            'median_latency_ms': round(float(np.median(latencies)), 1),
            'max_latency_ms': round(float(np.max(latencies)), 1),
            'min_latency_ms': round(float(np.min(latencies)), 1),
            'max_first_tick_leap_mm': round(float(np.max(leaps)), 3),
            'sync_window_ms': round((max(onset_times) - min(onset_times)) * 1000.0, 1),
            'per_leg': per_leg,
        })

    # Per-leg summary across all events ---------------------------------
    per_leg_summary: Dict[str, Any] = {}
    for leg in range(6):
        lats = [o['latency_ms'] for o in per_leg_onsets[leg]]
        if lats:
            per_leg_summary[f'leg_{leg}'] = {
                'n_onsets': len(lats),
                'median_latency_ms': round(float(np.median(lats)), 1),
                'max_latency_ms': round(float(np.max(lats)), 1),
            }
        else:
            per_leg_summary[f'leg_{leg}'] = {'n_onsets': 0}

    all_lats = [o['latency_ms'] for lst in per_leg_onsets.values() for o in lst]
    aggregate = {
        'total_onsets_detected': len(all_lats),
        'median_latency_ms': round(float(np.median(all_lats)), 1) if all_lats else None,
        'max_latency_ms': round(float(np.max(all_lats)), 1) if all_lats else None,
    }

    return {
        'detected': len(events) > 0,
        'events': events,
        'per_leg_summary': per_leg_summary,
        'aggregate': aggregate,
    }


def analyse_workspace(records: List[StepRecord]) -> Dict[str, Any]:
    """Workspace usage: how close to stroke limits."""
    per_leg = []
    for leg in range(6):
        ext = _extract(records, f'actual_ext_{leg}')
        min_ext = float(np.min(ext))
        max_ext = float(np.max(ext))
        per_leg.append({
            'leg': leg,
            'min_mm': round(min_ext, 2),
            'max_mm': round(max_ext, 2),
            'margin_to_lower_mm': round(min_ext - STROKE_MIN_MM, 2),
            'margin_to_upper_mm': round(STROKE_MAX_MM - max_ext, 2),
        })

    all_ext = np.concatenate([
        _extract(records, f'actual_ext_{leg}') for leg in range(6)
    ])

    return {
        'min_extension_mm': round(float(np.min(all_ext)), 2),
        'max_extension_mm': round(float(np.max(all_ext)), 2),
        'margin_to_lower_mm': round(float(np.min(all_ext)) - STROKE_MIN_MM, 2),
        'margin_to_upper_mm': round(STROKE_MAX_MM - float(np.max(all_ext)), 2),
        'per_leg': per_leg,
    }


def analyse_steady_state(records: List[StepRecord]) -> Dict[str, Any]:
    """Segment into transient and steady-state, report metrics for each."""
    n = len(records)
    if n < 10:
        return {}

    times = _extract(records, 'time')
    pos_err = _extract(records, 'tracking_error_mm')

    # Detect when reference stops changing
    ref_x = _extract(records, 'ref_pose_x')
    ref_y = _extract(records, 'ref_pose_y')
    ref_z = _extract(records, 'ref_pose_z')
    ref_rx = _extract(records, 'ref_pose_rx')
    ref_ry = _extract(records, 'ref_pose_ry')
    ref_rz = _extract(records, 'ref_pose_rz')

    ref_delta = (np.abs(np.diff(ref_x)) + np.abs(np.diff(ref_y)) +
                 np.abs(np.diff(ref_z)) + np.abs(np.diff(ref_rx)) * 1000 +
                 np.abs(np.diff(ref_ry)) * 1000 + np.abs(np.diff(ref_rz)) * 1000)

    # Steady state = where reference isn't changing AND error is small
    ref_static = ref_delta < REF_CHANGE_THRESHOLD
    if not np.any(ref_static):
        return {
            'has_steady_state': False,
            'transient_peak_mm': round(float(np.max(pos_err)), 4),
        }

    # Find first sustained static region (10+ consecutive static samples)
    ss_start_idx = None
    run_count = 0
    for i, is_static in enumerate(ref_static):
        if is_static:
            run_count += 1
            if run_count >= 10 and ss_start_idx is None:
                ss_start_idx = i - run_count + 1
        else:
            run_count = 0

    if ss_start_idx is None:
        return {
            'has_steady_state': False,
            'transient_peak_mm': round(float(np.max(pos_err)), 4),
        }

    ss_start_s = float(times[ss_start_idx])
    ss_errors = pos_err[ss_start_idx:]
    transient_errors = pos_err[:ss_start_idx] if ss_start_idx > 0 else pos_err[:1]

    # Settling time: time from start until error first drops below 2x SS RMS
    ss_rms = float(np.sqrt(np.mean(ss_errors**2)))
    settle_threshold = max(ss_rms * 2, 0.1)  # at least 0.1mm
    settling_time = None
    for i in range(len(pos_err)):
        if pos_err[i] < settle_threshold:
            settling_time = float(times[i] - times[0])
            break

    return {
        'has_steady_state': True,
        'ss_start_s': round(ss_start_s, 3),
        'ss_rms_mm': round(ss_rms, 4),
        'ss_peak_mm': round(float(np.max(ss_errors)), 4),
        'transient_peak_mm': round(float(np.max(transient_errors)), 4),
        'settling_time_s': round(settling_time, 3) if settling_time is not None else None,
    }


def analyse_torques(records: List[StepRecord]) -> Dict[str, Any]:
    """Feedforward torque analysis (if available in CSV)."""
    # Check if ff_torque_max_Nm is populated
    torques = _extract(records, 'ff_torque_max_Nm')
    if np.all(torques == 0):
        return {'available': False}

    return {
        'available': True,
        'max_Nm': round(float(np.max(torques)), 4),
        'mean_Nm': round(float(np.mean(torques)), 4),
        'p95_Nm': round(float(np.percentile(torques, 95)), 4),
    }


def analyse_plant_collapse(records: List[StepRecord]) -> Dict[str, Any]:
    """Detect a plant collapse + stale-telemetry event in the CSV.

    Signature (both conditions required, second following the first within
    PLANT_COLLAPSE_WINDOW_S):
      1. A step where mean(|leg_vel|) across the 6 legs exceeds
         PLANT_COLLAPSE_MEAN_SPEED_MM_S AND at least
         PLANT_COLLAPSE_SAME_SIGN_MIN legs share the same direction of
         motion — i.e., the platform is falling/rising uncommanded at a
         speed the MPC rate cap would not have produced.
      2. A run of >= PLANT_COLLAPSE_ZERO_RUN_MIN consecutive steps where
         every actual_ext_i == 0.0 — the sentinel/empty payload the
         HardwarePlant ZMQ SUB hands back when the feedback publisher dies.

    When matched, everything at or after the overspeed timestamp is
    downstream of an exogenous failure (operator Ctrl+C on the launch,
    motor guard crash, CAN disconnect) and must not be conflated with MPC
    solver misbehaviour.  See logbook entry
    2026-04-18-move5-overshoot-stall-and-plant-collapse.md.
    """
    n = len(records)
    if n < PLANT_COLLAPSE_ZERO_RUN_MIN:
        return {'detected': False}

    times = _extract(records, 'time')
    leg_vels = np.stack(
        [_extract(records, f'leg_vel_{i}') for i in range(6)], axis=1)     # (n, 6)
    actual_exts = np.stack(
        [_extract(records, f'actual_ext_{i}') for i in range(6)], axis=1)  # (n, 6)

    mean_speed = np.mean(np.abs(leg_vels), axis=1)                         # (n,)
    # Count legs moving "forward" (>0) vs "backward" (<0); treat exact zeros
    # as neutral so a single stalled leg doesn't kill the signature.
    n_pos = np.sum(leg_vels > 0, axis=1)
    n_neg = np.sum(leg_vels < 0, axis=1)
    same_sign_legs = np.maximum(n_pos, n_neg)                              # (n,)

    overspeed_mask = (
        (mean_speed > PLANT_COLLAPSE_MEAN_SPEED_MM_S) &
        (same_sign_legs >= PLANT_COLLAPSE_SAME_SIGN_MIN)
    )                                                                       # (n,)

    # All-zero-extension rows (exact zeros — the sentinel, not near-zero)
    zero_mask = np.all(actual_exts == 0.0, axis=1)                         # (n,)

    # Find runs of >= PLANT_COLLAPSE_ZERO_RUN_MIN consecutive zero rows
    runs = []
    run_start = None
    for i, z in enumerate(zero_mask):
        if z:
            if run_start is None:
                run_start = i
        else:
            if run_start is not None:
                if i - run_start >= PLANT_COLLAPSE_ZERO_RUN_MIN:
                    runs.append((run_start, i - 1))
                run_start = None
    if run_start is not None and n - run_start >= PLANT_COLLAPSE_ZERO_RUN_MIN:
        runs.append((run_start, n - 1))

    if not runs:
        return {'detected': False}

    # Pair the first qualifying zero-run with a preceding overspeed event
    overspeed_idx = np.flatnonzero(overspeed_mask)
    for zero_start, zero_end in runs:
        t_zero = float(times[zero_start])
        # Most recent overspeed step strictly before the zero-run start
        preceding = overspeed_idx[overspeed_idx < zero_start]
        if preceding.size == 0:
            continue
        t_overspeed = float(times[preceding[-1]])
        if t_zero - t_overspeed > PLANT_COLLAPSE_WINDOW_S:
            continue
        # Match.
        overspeed_step = int(preceding[-1])
        return {
            'detected': True,
            'overspeed_step': overspeed_step,
            'overspeed_time_s': round(t_overspeed, 4),
            'zero_run_start_step': int(zero_start),
            'zero_run_start_time_s': round(t_zero, 4),
            'zero_run_length': int(zero_end - zero_start + 1),
            'peak_mean_leg_speed_mmps': round(float(np.max(mean_speed)), 1),
            'peak_abs_leg_vel_mmps': round(float(np.max(np.abs(leg_vels))), 1),
            'overspeed_threshold_mmps': round(PLANT_COLLAPSE_MEAN_SPEED_MM_S, 1),
        }

    return {'detected': False}


def analyse_overshoot_saturation(records: List[StepRecord]) -> Dict[str, Any]:
    """Detect overshoot-induced NLP saturation.

    Signature (all three required over a window of >= OVERSHOOT_ZERO_ITER_RUN_MIN
    consecutive steps):
      1. `ipopt_iter == 0` on every step — IPOPT hits the CPU cap before
         completing one iteration, signalled by the fallback status
         codes (`fallback`, `fallback_extrap`, `fallback_hold`,
         `cold_hold`; substring detection downstream is unchanged —
         uses the `'fallback'` / `'hold'` / `'cold_hold'` keywords).
      2. `tracking_error_mm` exceeds OVERSHOOT_TRACKING_ERR_MM — the plant
         pose is far from the reference pose during the run.
      3. The reference is still changing (ref_delta > REF_CHANGE_THRESHOLD)
         somewhere in the window — otherwise this is a hold, not an
         overshoot/settle failure.

    Distinct from the generic MPC_STALENESS known-issue: that one fires on
    any consecutive-timeout run.  Overshoot saturation additionally
    requires pose-level tracking error with a moving ref, which points to
    a specific fix (ref-from-current-plant-state on move boundaries — see
    logbook entry 2026-04-18-move5-overshoot-stall-and-plant-collapse.md,
    option (b)).

    Note: tracking_error_mm is the magnitude of |actual_pose - ref_pose|
    over the position DOFs; we do not infer direction here (the interesting
    direction — plant ahead of ref — is a human-side interpretation, not
    what triggers the detector).
    """
    n = len(records)
    if n < OVERSHOOT_ZERO_ITER_RUN_MIN:
        return {'detected': False, 'events': []}

    times = _extract(records, 'time')
    ipopt_iters = _extract(records, 'ipopt_iter').astype(int)
    tracking_err = _extract(records, 'tracking_error_mm')

    # Reference motion indicator (same weighting as analyse_steady_state)
    ref_x = _extract(records, 'ref_pose_x')
    ref_y = _extract(records, 'ref_pose_y')
    ref_z = _extract(records, 'ref_pose_z')
    ref_rx = _extract(records, 'ref_pose_rx')
    ref_ry = _extract(records, 'ref_pose_ry')
    ref_rz = _extract(records, 'ref_pose_rz')
    ref_delta = (np.abs(np.diff(ref_x)) + np.abs(np.diff(ref_y)) +
                 np.abs(np.diff(ref_z)) + np.abs(np.diff(ref_rx)) * 1000 +
                 np.abs(np.diff(ref_ry)) * 1000 + np.abs(np.diff(ref_rz)) * 1000)
    # Pad to length n so indexing matches; step i's ref motion is delta[i-1]
    ref_moving = np.concatenate(([False], ref_delta > REF_CHANGE_THRESHOLD))  # (n,)

    # Walk the record, gather runs that satisfy (1)+(2); mark those that also
    # meet (3) somewhere inside the run.
    events = []
    run_start = None
    for i in range(n):
        cond = ((ipopt_iters[i] == 0) and
                (tracking_err[i] > OVERSHOOT_TRACKING_ERR_MM))
        if cond:
            if run_start is None:
                run_start = i
        else:
            if run_start is not None and (i - run_start) >= OVERSHOOT_ZERO_ITER_RUN_MIN:
                if np.any(ref_moving[run_start:i]):
                    events.append({
                        'start_step': int(run_start),
                        'end_step': int(i - 1),
                        'length': int(i - run_start),
                        'start_time_s': round(float(times[run_start]), 4),
                        'end_time_s': round(float(times[i - 1]), 4),
                        'peak_tracking_err_mm': round(
                            float(np.max(tracking_err[run_start:i])), 2),
                    })
            run_start = None
    if run_start is not None and (n - run_start) >= OVERSHOOT_ZERO_ITER_RUN_MIN:
        if np.any(ref_moving[run_start:n]):
            events.append({
                'start_step': int(run_start),
                'end_step': int(n - 1),
                'length': int(n - run_start),
                'start_time_s': round(float(times[run_start]), 4),
                'end_time_s': round(float(times[n - 1]), 4),
                'peak_tracking_err_mm': round(
                    float(np.max(tracking_err[run_start:n])), 2),
            })

    return {
        'detected': bool(events),
        'events': events,
        'max_consecutive': int(max((e['length'] for e in events), default=0)),
    }


def _find_longest_hold_window(records: List[StepRecord]) -> Optional[Tuple[int, int]]:
    """Return (start_idx, end_idx_exclusive) of the longest reference-static run.

    Uses the same ref-delta definition as analyse_steady_state so hold-phase
    metrics and steady-state metrics agree on what counts as "settled".
    Returns None if no ref-static window is present.
    """
    n = len(records)
    if n < 2:
        return None
    ref_x = _extract(records, 'ref_pose_x')
    ref_y = _extract(records, 'ref_pose_y')
    ref_z = _extract(records, 'ref_pose_z')
    ref_rx = _extract(records, 'ref_pose_rx')
    ref_ry = _extract(records, 'ref_pose_ry')
    ref_rz = _extract(records, 'ref_pose_rz')
    ref_delta = (np.abs(np.diff(ref_x)) + np.abs(np.diff(ref_y)) +
                 np.abs(np.diff(ref_z)) + np.abs(np.diff(ref_rx)) * 1000 +
                 np.abs(np.diff(ref_ry)) * 1000 + np.abs(np.diff(ref_rz)) * 1000)
    static = ref_delta < HOLD_REF_STATIC_TOL           # (n-1,)

    best_start = best_end = -1
    best_len = 0
    cur_start = None
    for i, s in enumerate(static):
        if s:
            if cur_start is None:
                cur_start = i
        else:
            if cur_start is not None:
                cur_len = i - cur_start
                if cur_len > best_len:
                    best_len = cur_len
                    best_start = cur_start
                    best_end = i
                cur_start = None
    if cur_start is not None:
        cur_len = len(static) - cur_start
        if cur_len > best_len:
            best_len = cur_len
            best_start = cur_start
            best_end = len(static)
    if best_len == 0:
        return None
    # Ref delta at index i compares records[i] and records[i+1]; the static
    # window covers samples (best_start .. best_end+1] in record space.
    return (best_start + 1, best_end + 1)


def analyse_hold_phase(records: List[StepRecord]) -> Dict[str, Any]:
    """Per-leg cmd_ext and actual_ext noise stats during the longest hold.

    The Level-1 gain-tuning metric (see plans/active/leg-gain-tuning-methodology.md).

    Uses the tail of the longest ref-static run: skip HOLD_SETTLE_SKIP_S after
    ref becomes static to let the platform converge, then take at most
    HOLD_MAX_TAIL_S of remaining samples.  Returns `detected: False` if the
    useful tail (after the skip) is shorter than HOLD_MIN_TAIL_S.
    """
    n = len(records)
    if n < 10:
        return {'detected': False, 'reason': 'too_few_samples'}
    window = _find_longest_hold_window(records)
    if window is None:
        return {'detected': False, 'reason': 'ref_never_static'}
    raw_start, end = window
    times = _extract(records, 'time')
    raw_t_start = float(times[raw_start])
    t_end = float(times[end - 1]) if end > raw_start else raw_t_start
    total_static_s = t_end - raw_t_start

    # Skip settling samples.
    skip_cutoff = raw_t_start + HOLD_SETTLE_SKIP_S
    start = raw_start
    while start < end and float(times[start]) < skip_cutoff:
        start += 1
    # Cap tail length: stats plateau once we have a few seconds.
    t_start = float(times[start]) if start < end else raw_t_start
    tail_s = t_end - t_start
    if tail_s > HOLD_MAX_TAIL_S:
        # Walk `start` forward until only HOLD_MAX_TAIL_S remain.
        cap_cutoff = t_end - HOLD_MAX_TAIL_S
        while start < end and float(times[start]) < cap_cutoff:
            start += 1
        t_start = float(times[start]) if start < end else raw_t_start
        tail_s = t_end - t_start
    duration_s = tail_s
    if duration_s < HOLD_MIN_TAIL_S:
        return {
            'detected': False,
            'reason': 'tail_too_short',
            'raw_t_start_s': round(raw_t_start, 3),
            'total_static_s': round(total_static_s, 3),
            'tail_s': round(tail_s, 3),
            'min_tail_s': HOLD_MIN_TAIL_S,
            'settle_skip_s': HOLD_SETTLE_SKIP_S,
        }

    per_leg = []
    for leg in range(6):
        cmd = _extract(records, f'cmd_ext_{leg}')[start:end]
        act = _extract(records, f'actual_ext_{leg}')[start:end]
        vel = _extract(records, f'leg_vel_{leg}')[start:end]
        # Stdev guarded against single-sample windows.
        cmd_std = float(np.std(cmd, ddof=1)) if len(cmd) > 1 else 0.0
        act_std = float(np.std(act, ddof=1)) if len(act) > 1 else 0.0
        act_range = float(np.ptp(act)) if len(act) > 0 else 0.0
        vel_abs_max = float(np.max(np.abs(vel))) if len(vel) > 0 else 0.0
        per_leg.append({
            'leg': leg,
            'cmd_std_um': round(cmd_std * 1000, 2),
            'act_std_um': round(act_std * 1000, 2),
            'act_range_um': round(act_range * 1000, 2),
            'vel_abs_max_mmps': round(vel_abs_max, 2),
        })
    leg_stds = [l['act_std_um'] for l in per_leg]
    worst_leg = int(np.argmax(leg_stds))
    quietest_leg = int(np.argmin(leg_stds))
    worst_std = leg_stds[worst_leg]
    quietest_std = max(leg_stds[quietest_leg], 0.01)  # avoid div/0
    asymmetry = worst_std / quietest_std
    aggregate = float(np.mean(leg_stds))
    return {
        'detected': True,
        't_start_s': round(t_start, 3),
        't_end_s': round(t_end, 3),
        'duration_s': round(duration_s, 3),
        'n_samples': int(end - start),
        'raw_t_start_s': round(raw_t_start, 3),
        'total_static_s': round(total_static_s, 3),
        'settle_skip_s': HOLD_SETTLE_SKIP_S,
        'per_leg': per_leg,
        'worst_leg': worst_leg,
        'worst_act_std_um': round(worst_std, 2),
        'quietest_leg': quietest_leg,
        'quietest_act_std_um': round(quietest_std, 2),
        'asymmetry_ratio': round(asymmetry, 2),
        'aggregate_act_std_um': round(aggregate, 2),
    }


def analyse_overhead(records: List[StepRecord]) -> Dict[str, Any]:
    """MPC loop overhead = dt − solve_time per tick.

    Separates solver time from everything else in the control loop (GC pause,
    interpreter housekeeping, ZMQ drain, logging).  Flags "isolated spikes" —
    single ticks whose overhead is >= OVERHEAD_ISOLATED_RATIO × its local
    neighbourhood mean, which is the characteristic GC-pause signature.
    """
    n = len(records)
    if n < 3:
        return {'n_valid': 0}
    times = _extract(records, 'time')
    solve_ms = _extract(records, 'solve_time_ms')
    # dt[i] is the interval between step i and step i+1; we attribute the
    # overhead it reveals to step i+1 (the one that was served late).
    dt_ms = np.diff(times) * 1000.0
    overhead_ms = dt_ms - solve_ms[1:]
    # Clamp negatives (numerical, or solve_time logged after dt end) to 0.
    overhead_ms = np.clip(overhead_ms, 0.0, None)
    if len(overhead_ms) == 0:
        return {'n_valid': 0}

    p50 = float(np.median(overhead_ms))
    p95 = float(np.percentile(overhead_ms, 95))
    p99 = float(np.percentile(overhead_ms, 99))
    max_oh = float(np.max(overhead_ms))
    above_budget = overhead_ms > OVERHEAD_BUDGET_MS
    above_count = int(np.sum(above_budget))

    # Isolated-spike detection: at each tick i with overhead >= floor, compare
    # to the MEDIAN of the surrounding window (excluding i and its ±1
    # neighbours).  Use the median, not the mean, so a cluster of adjacent
    # spikes can't pull up its own baseline — a ±5 window that contains 3
    # adjacent spikes will have a mean dragged up enough to suppress the
    # whole cluster, while the median stays at the ambient level and the
    # cluster is correctly flagged.  The ±1 gap further blunts leakage from
    # dt-alignment jitter around a single pause.  The absolute floor
    # (OVERHEAD_BASELINE_FLOOR_MS) prevents a perfectly quiet neighbourhood
    # from producing infinite ratios and flagging every tick above the
    # spike floor.
    OVERHEAD_BASELINE_FLOOR_MS = 1.0
    spikes = []
    w = OVERHEAD_NEIGHBOUR_WINDOW
    for i in range(len(overhead_ms)):
        if overhead_ms[i] < OVERHEAD_MIN_MS_FOR_SPIKE:
            continue
        lo = max(0, i - w)
        hi = min(len(overhead_ms), i + w + 1)
        # Exclude i-1, i, i+1 from the neighbourhood.
        left = overhead_ms[lo:max(lo, i - 1)]
        right = overhead_ms[min(hi, i + 2):hi]
        neighbours = np.concatenate([left, right])
        if len(neighbours) == 0:
            continue
        neigh_median = float(np.median(neighbours))
        baseline_for_ratio = max(neigh_median, OVERHEAD_BASELINE_FLOOR_MS)
        ratio = overhead_ms[i] / baseline_for_ratio
        if ratio < OVERHEAD_ISOLATED_RATIO:
            continue
        spikes.append({
            'sample_idx': int(i + 1),               # back to record index
            'time_s': round(float(times[i + 1]), 4),
            'overhead_ms': round(float(overhead_ms[i]), 2),
            'solve_ms': round(float(solve_ms[i + 1]), 2),
            # Keep the key name 'neighbour_mean_ms' for schema stability; the
            # value is a median, which is the robust baseline we actually use.
            'neighbour_mean_ms': round(neigh_median, 2),
            'ratio': round(float(ratio), 1),
            'signature': 'GC_PAUSE_CANDIDATE',
        })

    return {
        'n_valid': int(len(overhead_ms)),
        'p50_ms': round(p50, 2),
        'p95_ms': round(p95, 2),
        'p99_ms': round(p99, 2),
        'max_ms': round(max_oh, 2),
        'above_budget_count': above_count,
        'above_budget_pct': round(100.0 * above_count / len(overhead_ms), 2),
        'budget_ms': OVERHEAD_BUDGET_MS,
        'isolated_spikes': spikes,
        'isolated_spike_count': len(spikes),
    }


def analyse_cmd_spectral(records: List[StepRecord]) -> Dict[str, Any]:
    """FFT cmd_ext during the longest hold to expose HF numerical noise.

    Bins the energy into three bands (low/mid/HF) and reports the peak
    frequency per leg.  Above HF_ENERGY_WARN_PCT of energy in the HF band
    triggers a flag — usually "MPC numerical noise floor" (IPOPT variance
    between solves showing up in cmd_ext).
    """
    n = len(records)
    if n < 32:
        return {'detected': False, 'reason': 'too_few_samples'}
    window = _find_longest_hold_window(records)
    if window is None:
        return {'detected': False, 'reason': 'ref_never_static'}
    raw_start, end = window
    times = _extract(records, 'time')
    raw_t_start = float(times[raw_start])
    t_end = float(times[end - 1]) if end > raw_start else raw_t_start
    # Apply the same settle-skip AND HOLD_MAX_TAIL_S cap as analyse_hold_phase,
    # so spectral energy is computed on the same samples the stdev metric
    # sees.  Without the cap, holds > HOLD_MAX_TAIL_S post-settle would FFT
    # over a longer window here than is measured for stdev, producing
    # subtly different bin energies across sessions.
    skip_cutoff = raw_t_start + HOLD_SETTLE_SKIP_S
    start = raw_start
    while start < end and float(times[start]) < skip_cutoff:
        start += 1
    t_start = float(times[start]) if start < end else raw_t_start
    if (t_end - t_start) > HOLD_MAX_TAIL_S:
        cap_cutoff = t_end - HOLD_MAX_TAIL_S
        while start < end and float(times[start]) < cap_cutoff:
            start += 1
    length = end - start
    if length < 32:
        return {'detected': False, 'reason': 'tail_too_short'}
    dt_mean = float(np.mean(np.diff(times[start:end])))
    if dt_mean <= 0:
        return {'detected': False, 'reason': 'bad_dt'}
    fs = 1.0 / dt_mean
    nyquist = fs / 2.0

    per_leg = []
    for leg in range(6):
        cmd = _extract(records, f'cmd_ext_{leg}')[start:end].astype(np.float64)
        cmd = cmd - cmd.mean()
        # Hanning window for clean bin energy.
        w = np.hanning(len(cmd))
        X = np.fft.rfft(cmd * w)
        freqs = np.fft.rfftfreq(len(cmd), dt_mean)
        mag2 = np.abs(X) ** 2
        mid_cut = min(CMD_SPECTRAL_MID_HZ, nyquist * 0.5)
        hf_cut = min(CMD_SPECTRAL_HF_HZ, nyquist * 0.9)
        band_mask = freqs > 0.1                      # ignore DC / sub-band leakage
        e_low = float(np.sum(mag2[band_mask & (freqs < mid_cut)]))
        e_mid = float(np.sum(mag2[(freqs >= mid_cut) & (freqs < hf_cut)]))
        e_hf = float(np.sum(mag2[freqs >= hf_cut]))
        total = e_low + e_mid + e_hf
        # Confine peak-frequency reporting to the bands we actually report —
        # otherwise peak_freq_hz can land at a DC residue (0.0 / 0.05 Hz) and
        # mislead the reader into thinking the cmd has low-frequency motion.
        if band_mask.any():
            masked_mag2 = mag2.copy()
            masked_mag2[~band_mask] = -np.inf
            peak_freq = float(freqs[int(np.argmax(masked_mag2))])
        else:
            peak_freq = 0.0
        if total > 0:
            per_leg.append({
                'leg': leg,
                'pct_low': round(100.0 * e_low / total, 1),
                'pct_mid': round(100.0 * e_mid / total, 1),
                'pct_hf': round(100.0 * e_hf / total, 1),
                'peak_freq_hz': round(peak_freq, 2),
            })
        else:
            per_leg.append({
                'leg': leg, 'pct_low': 0.0, 'pct_mid': 0.0,
                'pct_hf': 0.0, 'peak_freq_hz': 0.0,
            })
    hf_max = max((l['pct_hf'] for l in per_leg), default=0.0)
    return {
        'detected': True,
        'fs_hz': round(fs, 2),
        'band_low_hz': [0.1, round(mid_cut, 2)],
        'band_mid_hz': [round(mid_cut, 2), round(hf_cut, 2)],
        'band_hf_hz': [round(hf_cut, 2), round(nyquist, 2)],
        'per_leg': per_leg,
        'hf_pct_max': round(hf_max, 1),
        'hf_warn_pct': HF_ENERGY_WARN_PCT,
    }


def analyse_csv(records: List[StepRecord],
                budget_ms: float = DEFAULT_BUDGET_MS) -> Dict[str, Any]:
    """Run all MPC CSV analyses and return combined results.

    When a plant telemetry collapse is detected mid-run, aggregate analyses
    (tracking, solve_times, workspace, steady_state, etc.) are computed on
    the pre-collapse slice only. Including post-collapse steps would fold
    zero-sentinel feedback and ghost solve-loop activity into the numbers
    and make a healthy run look like a failure (the Move 5 experience).
    The collapse detection itself always runs on the full record set.
    """
    n = len(records)
    if n == 0:
        return {'error': 'Empty CSV'}

    times = _extract(records, 'time')
    duration = float(times[-1] - times[0])
    dt_values = np.diff(times)
    dt_median = float(np.median(dt_values)) if len(dt_values) > 0 else 0

    # Run collapse detection on the full record set first.
    collapse = analyse_plant_collapse(records)

    # If a collapse was detected, analyse the aggregate metrics on the
    # pre-collapse slice only. Anything at/after the overspeed step is
    # downstream of an exogenous failure and must not contaminate the numbers.
    if collapse.get('detected'):
        cut = int(collapse['overspeed_step'])
        analysed_records = records[:cut] if cut > 0 else records
    else:
        analysed_records = records

    return {
        'n_samples': n,
        'analysed_samples': len(analysed_records),
        'duration_s': round(duration, 3),
        'dt_median_ms': round(dt_median * 1000, 2),
        'tracking': analyse_tracking(analysed_records),
        'solve_times': analyse_solve_times(analysed_records, budget_ms),
        'solver_status': analyse_solver_status(analysed_records),
        'oscillation': analyse_oscillation(analysed_records),
        'discontinuities': analyse_discontinuities(analysed_records),
        'workspace': analyse_workspace(analysed_records),
        'steady_state': analyse_steady_state(analysed_records),
        'torques': analyse_torques(analysed_records),
        'plant_collapse': collapse,
        'overshoot_saturation': analyse_overshoot_saturation(analysed_records),
        'hold_phase': analyse_hold_phase(analysed_records),
        'overhead': analyse_overhead(analysed_records),
        'cmd_spectral': analyse_cmd_spectral(analysed_records),
        'motion_onset': analyse_motion_onset(analysed_records),
    }


# ---------------------------------------------------------------------------
# ROS2 plain-text log parsing
# ---------------------------------------------------------------------------

# ROS2 Foxy log line pattern:
#   [INFO] [1234567890.123456789] [node_name]: message
# or (older format):
#   [INFO] [node_name]: message
_ROS2_LOG_RE = re.compile(
    r'\[(?P<level>DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]'
    r'\s+\[(?P<timestamp>[\d.]+)\]'
    r'\s+\[(?P<node>[^\]]+)\]:\s*(?P<message>.*)'
)

# Fallback pattern without nanosecond timestamp
_ROS2_LOG_RE_SIMPLE = re.compile(
    r'\[(?P<level>DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]'
    r'\s+\[(?P<node>[^\]]+)\]:\s*(?P<message>.*)'
)

# Patterns of interest in ROS2 logs
_INTERESTING_PATTERNS = [
    # State machine transitions
    (re.compile(r'\[SM\]'), 'state_transition'),
    # Errors and faults
    (re.compile(r'(?i)error|fault|disarm|rejected|failed|timeout'), 'error'),
    # E-stop events
    (re.compile(r'(?i)e-?stop|estop|emergency'), 'estop'),
    # CAN bus issues
    (re.compile(r'(?i)CAN bus restored|watchdog'), 'can_watchdog'),
    # Workspace limits
    (re.compile(r'(?i)workspace.*limit|WORKSPACE'), 'workspace'),
    # Mode changes
    (re.compile(r'MPC mode|MPC session|control mode|Control mode'), 'mode'),
    # Motor guard
    (re.compile(r'Motor guard|motor guard'), 'motor_guard'),
    # Homing
    (re.compile(r'(?i)homing|homed|encoder search'), 'homing'),
]


@dataclass
class ROS2Event:
    """A parsed event from a ROS2 log file."""
    timestamp: Optional[float]
    level: str
    node: str
    message: str
    category: str = 'other'

    def to_dict(self) -> Dict[str, Any]:
        return {
            'timestamp': self.timestamp,
            'level': self.level,
            'node': self.node,
            'message': self.message,
            'category': self.category,
        }


def parse_ros2_log_file(path: str) -> List[ROS2Event]:
    """Parse a single ROS2 log file into events."""
    events = []
    try:
        with open(path, 'r', errors='replace') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue

                m = _ROS2_LOG_RE.match(line)
                if m:
                    ts = float(m.group('timestamp'))
                    level = m.group('level')
                    node = m.group('node')
                    msg = m.group('message')
                else:
                    m = _ROS2_LOG_RE_SIMPLE.match(line)
                    if m:
                        ts = None
                        level = m.group('level')
                        node = m.group('node')
                        msg = m.group('message')
                    else:
                        continue

                # Normalize WARNING → WARN
                if level == 'WARNING':
                    level = 'WARN'

                # Categorize
                category = 'other'
                for pattern, cat in _INTERESTING_PATTERNS:
                    if pattern.search(msg):
                        category = cat
                        break

                # Only keep interesting events (not routine DEBUG/INFO)
                if level in ('ERROR', 'FATAL', 'WARN') or category != 'other':
                    events.append(ROS2Event(
                        timestamp=ts, level=level, node=node,
                        message=msg, category=category))
    except OSError as e:
        events.append(ROS2Event(
            timestamp=None, level='ERROR', node='diagnose',
            message=f'Failed to read log file {path}: {e}',
            category='error'))

    return events


def parse_ros2_log_dir(log_dir: str) -> Dict[str, Any]:
    """Parse all log files in a ROS2 session directory."""
    if not os.path.isdir(log_dir):
        return {'error': f'Not a directory: {log_dir}', 'events': []}

    all_events = []
    log_files = glob.glob(os.path.join(log_dir, '*.log'))
    if not log_files:
        # Try subdirectories (some ROS2 versions nest logs)
        log_files = glob.glob(os.path.join(log_dir, '**', '*.log'), recursive=True)

    for path in sorted(log_files):
        events = parse_ros2_log_file(path)
        all_events.extend(events)

    # Sort by timestamp (put None-timestamp events at the end)
    all_events.sort(key=lambda e: (e.timestamp is None, e.timestamp or 0))

    # Summary statistics
    level_counts = defaultdict(int)  # type: Dict[str, int]
    category_counts = defaultdict(int)  # type: Dict[str, int]
    for e in all_events:
        level_counts[e.level] += 1
        category_counts[e.category] += 1

    # Look for session start marker
    session_csv = None
    for e in all_events:
        if 'MPC session started:' in e.message:
            # Extract CSV filename
            match = re.search(r'MPC session started:\s*(\S+)', e.message)
            if match:
                session_csv = match.group(1)

    return {
        'n_log_files': len(log_files),
        'n_events': len(all_events),
        'level_counts': dict(level_counts),
        'category_counts': dict(category_counts),
        'session_csv': session_csv,
        'events': [e.to_dict() for e in all_events],
    }


# ---------------------------------------------------------------------------
# Rosbag (MCAP) analysis
# ---------------------------------------------------------------------------

def _f(kv: Dict[str, Any], key: str) -> Optional[float]:
    """Parse a trajectory/diagnostics KeyValue string to float (None if absent)."""
    try:
        return float(kv[key])
    except (KeyError, TypeError, ValueError):
        return None


def summarise_trajectory_moves(traj_diag: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Per-move leg-peak summary from the ``/trajectory/diagnostics`` stream.

    Segments the diagnostics samples into contiguous ``plan_kind == 'move'``
    windows (each an accepted go_to_pose during the ramp battery), and for each
    reports the gate-PREDICTED and REALIZED leg peaks, the session limits in force,
    the jerk/vel/acc **headroom** (% of the limit used — the binding-constraint
    signal the ramp workflow raises limits by feel against), and the lean A/B arm
    (``lean_gain``). The realized peaks are running maxima reset per install, so the
    last sample of a move window carries that move's realized peak.

    **Segmentation.** A completed move's plan stays ``plan_kind == 'move'`` for its
    whole terminal-hold lifetime, so the ramp battery's back-to-back moves (no hold
    between them) form one unbroken ``'move'`` run. A window therefore breaks on a
    ``plan_kind`` change OR a change in ``move_seq`` (the node's per-move install
    counter). Streams lacking ``move_seq`` (older bags) degrade to plan_kind-only
    segmentation, merging any consecutive holdless moves — the pre-``move_seq``
    behaviour.

    ``used_pct`` is computed from the REALIZED peaks; a parallel ``used_pct_predicted``
    from the gate's fine-sampled peaks. For **jerk** prefer ``used_pct_predicted`` —
    realized jerk is a coarse 40 Hz knot-rate difference that under-measures the fine
    peak the gate actually enforces. vel/acc realized are exact wire values, so their
    ``used_pct`` is authoritative.

    This is the Phase-4 ``/diagnose`` extension: it turns a ramp session's rosbag
    into a ``move → peaks + headroom`` table the operator reviews before persisting
    a limit bump to ``hardware_config.yaml``.
    """
    if not traj_diag:
        return {'available': False, 'reason': 'no /trajectory/diagnostics samples'}

    moves: List[Dict[str, Any]] = []
    window: List[Dict[str, Any]] = []

    def flush():
        if not window:
            return
        last = window[-1]                      # accumulated realized peaks live here
        first = window[0]
        lv = _f(last, 'limit_leg_vel_mmps')
        la = _f(last, 'limit_leg_acc_mmps2')
        lj = _f(last, 'limit_leg_jerk_mmps3')
        rv = _f(last, 'realized_peak_leg_vel_mmps')
        ra = _f(last, 'realized_peak_leg_acc_mmps2')
        rj = _f(last, 'realized_peak_leg_jerk_mmps3')
        pv = _f(last, 'peak_leg_vel_mmps')
        pa = _f(last, 'peak_leg_acc_mmps2')
        pj = _f(last, 'peak_leg_jerk_mmps3')

        def pct(peak, lim):
            if peak is None or not lim:
                return None
            return round(100.0 * peak / lim, 1)

        moves.append({
            't_start_s': first['t_s'],
            't_end_s': last['t_s'],
            'move_seq': last.get('move_seq'),
            'lean_gain': _f(last, 'lean_gain'),
            'predicted': {'vel_mmps': pv, 'acc_mmps2': pa, 'jerk_mmps3': pj},
            'realized': {'vel_mmps': rv, 'acc_mmps2': ra, 'jerk_mmps3': rj},
            'limits': {'vel_mmps': lv, 'acc_mmps2': la, 'jerk_mmps3': lj},
            # % of the session limit the REALIZED peak used (headroom = 100 − this).
            'used_pct': {'vel': pct(rv, lv), 'acc': pct(ra, la),
                         'jerk': pct(rj, lj)},
            # Gate-authoritative % from the fine-sampled PREDICTED peaks — prefer this
            # for jerk (realized jerk is a coarse knot-rate proxy that under-measures).
            'used_pct_predicted': {'vel': pct(pv, lv), 'acc': pct(pa, la),
                                   'jerk': pct(pj, lj)},
        })

    cur_seq = None
    for s in traj_diag:
        if s.get('plan_kind') != 'move':
            flush()
            window = []
            cur_seq = None
            continue
        seq = s.get('move_seq')
        if window and seq != cur_seq:      # a new install inside the 'move' run
            flush()
            window = []
        window.append(s)
        cur_seq = seq
    flush()

    return {
        'available': True,
        'num_moves': len(moves),
        'moves': moves,
    }


def analyse_rosbag(rosbag_path: str) -> Dict[str, Any]:
    """Analyse a rosbag recording for diagnostic information.

    Uses the rosbags library (pure Python MCAP reader).  Degrades gracefully
    if rosbags is not installed.

    Performs a two-pass scan:
      1. Lightweight pass — count messages per topic.
      2. Targeted deep scan — extract state transitions, motor errors, and
         detect pilot E-stop events (DC_BUS_UNDER_VOLTAGE cascade → FAULT).
    """
    try:
        from rosbags.highlevel import AnyReader  # type: ignore
    except ImportError:
        return {
            'available': False,
            'error': 'rosbags library not installed (pip install rosbags)',
        }

    if not os.path.isdir(rosbag_path):
        return {'available': False, 'error': f'Not a directory: {rosbag_path}'}

    results = {
        'available': True,
        'topics_found': [],
        'duration_s': 0,
    }  # type: Dict[str, Any]

    try:
        from pathlib import Path as _Path
        with AnyReader([_Path(rosbag_path)]) as reader:
            start_ns = reader.start_time
            end_ns = reader.end_time
            start_s = start_ns / 1e9
            results['start_time_s'] = start_s
            results['end_time_s'] = end_ns / 1e9
            results['duration_s'] = round((end_ns - start_ns) / 1e9, 3)

            # Topic inventory
            topic_info = {}
            for conn in reader.connections:
                topic_info.setdefault(conn.topic, {
                    'msgtype': conn.msgtype,
                    'count': 0,
                })

            # Deep scans
            state_transitions: List[Dict[str, Any]] = []
            mode_transitions: List[Dict[str, Any]] = []
            motor_errors: List[Dict[str, Any]] = []   # [{t,motor,error,prev_error}]
            traj_diag: List[Dict[str, Any]] = []       # trajectory/diagnostics samples
            prev_state = None
            prev_mode = None
            # Per-motor last active_errors value, for edge detection
            prev_motor_err: Dict[int, int] = {}

            for conn, ts, raw in reader.messages():
                if conn.topic in topic_info:
                    topic_info[conn.topic]['count'] += 1

                rel = ts / 1e9 - start_s

                if conn.topic == '/orchestrator_state':
                    msg = reader.deserialize(raw, conn.msgtype)
                    data = getattr(msg, 'data', None)
                    if data is not None and data != prev_state:
                        state_transitions.append({'t_s': round(rel, 3),
                                                  'state': data})
                        prev_state = data

                elif conn.topic == '/control_mode_topic':
                    msg = reader.deserialize(raw, conn.msgtype)
                    data = getattr(msg, 'data', None)
                    if data != prev_mode:
                        mode_transitions.append({'t_s': round(rel, 3),
                                                 'mode': data or ''})
                        prev_mode = data

                elif conn.topic == '/trajectory/diagnostics':
                    msg = reader.deserialize(raw, conn.msgtype)
                    kv = {getattr(e, 'key', ''): getattr(e, 'value', '')
                          for e in (getattr(msg, 'values', None) or [])}
                    if kv:
                        kv['t_s'] = round(rel, 3)
                        traj_diag.append(kv)

                elif conn.topic == '/robot_state':
                    msg = reader.deserialize(raw, conn.msgtype)
                    motor_states = getattr(msg, 'motor_states', None) or []
                    for i, ms in enumerate(motor_states):
                        err = (getattr(ms, 'active_errors', None)
                               or getattr(ms, 'error', None) or 0)
                        if err != prev_motor_err.get(i):
                            # Edge (0→err or err_a→err_b); record rising only
                            if err and err != 0 and err != prev_motor_err.get(i, -1):
                                motor_errors.append({
                                    't_s': round(rel, 3),
                                    'motor': i,
                                    'active_errors': int(err),
                                })
                            prev_motor_err[i] = err

            results['topics'] = topic_info
            results['topics_found'] = list(topic_info.keys())
            results['state_transitions'] = state_transitions
            results['mode_transitions'] = mode_transitions
            results['motor_errors'] = motor_errors
            results['trajectory'] = summarise_trajectory_moves(traj_diag)
            results['estop'] = detect_estop_event(
                motor_errors=motor_errors,
                state_transitions=state_transitions,
                mode_transitions=mode_transitions,
            )

    except Exception as e:
        results['error'] = f'Failed to read rosbag: {e}'

    return results


# ---------------------------------------------------------------------------
# Pilot E-stop detection
# ---------------------------------------------------------------------------

# ODrive error bit for DC bus under-voltage — pressing the physical E-stop
# cuts the bus voltage, so this is the reliable signature of an E-stop.
# Definition mirrors ros_ws/src/jugglebot/jugglebot/can/odrive.py:ERR_DC_BUS_UNDER_VOLTAGE
ERR_DC_BUS_UNDER_VOLTAGE = 512

# Cascade window for co-occurring motor undervolts — tight because a physical
# E-stop collapses the bus within microseconds; 200ms is a generous bound that
# still excludes unrelated faults.
ESTOP_CASCADE_WINDOW_S = 0.200

# Max delay between motor undervolt cascade and orchestrator FAULT for the
# FAULT to be attributable to the E-stop (vs. an independent fault).
ESTOP_TO_FAULT_WINDOW_S = 0.500


def detect_estop_event(
    motor_errors: List[Dict[str, Any]],
    state_transitions: List[Dict[str, Any]],
    mode_transitions: List[Dict[str, Any]],
) -> Dict[str, Any]:
    """Detect pilot E-stop from motor-error cascade + orchestrator FAULT.

    The physical E-stop cuts the ODrive DC bus, so every motor reports
    DC_BUS_UNDER_VOLTAGE (bit 512) within a few ms of each other, followed
    within ~500ms by an orchestrator FAULT.  This is unambiguous and always
    operator-initiated — the E-stop is a RESPONSE, never a primary cause.

    Returns a dict with 'detected' and, when detected, timing and affected
    motors.  `bag_time_s` is the timestamp of the first undervolt edge.
    """
    undervolts = [e for e in motor_errors
                  if (e['active_errors'] & ERR_DC_BUS_UNDER_VOLTAGE)]
    if len(undervolts) < 2:
        return {'detected': False}

    # Cluster the first cascade
    undervolts.sort(key=lambda e: e['t_s'])
    first_t = undervolts[0]['t_s']
    cluster = [e for e in undervolts
               if e['t_s'] - first_t <= ESTOP_CASCADE_WINDOW_S]
    affected = sorted({e['motor'] for e in cluster})
    if len(affected) < 2:
        return {'detected': False}

    # Find an orchestrator FAULT shortly after the cascade (diagnostic, not
    # required — the undervolt cascade alone is strong evidence)
    fault_t: Optional[float] = None
    for st in state_transitions:
        if 'FAULT' in (st.get('state') or '').upper() and st['t_s'] >= first_t:
            if st['t_s'] - first_t <= ESTOP_TO_FAULT_WINDOW_S:
                fault_t = st['t_s']
            break

    mode_error_t: Optional[float] = None
    for md in mode_transitions:
        if 'ERROR' in (md.get('mode') or '').upper() and md['t_s'] >= first_t:
            if md['t_s'] - first_t <= ESTOP_TO_FAULT_WINDOW_S:
                mode_error_t = md['t_s']
            break

    cascade_span = max(e['t_s'] for e in cluster) - first_t

    return {
        'detected': True,
        'bag_time_s': round(first_t, 3),
        'affected_motors': affected,
        'cascade_span_ms': round(cascade_span * 1000, 1),
        'orchestrator_fault_at_s': (round(fault_t, 3)
                                    if fault_t is not None else None),
        'control_mode_error_at_s': (round(mode_error_t, 3)
                                    if mode_error_t is not None else None),
        'evidence': (
            f"DC_BUS_UNDER_VOLTAGE (err 512) on motors "
            f"{affected} within {cascade_span*1000:.0f}ms"
            + (f", orchestrator FAULT at +{(fault_t-first_t)*1000:.0f}ms"
               if fault_t is not None else "")
        ),
        'attribution_rule': (
            "Pilot E-stop is ALWAYS a RESPONSE to concerning behaviour, never "
            "a primary cause.  All events at or after bag_time_s are downstream."
        ),
    }


# ---------------------------------------------------------------------------
# Companion MPC stdout log (optional)
# ---------------------------------------------------------------------------

# Regex catalogue for interesting MPC stdout lines — extends the structured
# per-step signal in `solve_status` with session-level context (solver
# initialisation, final summary, FK warnings, etc.) that only appears in
# stdout.
_MPC_STDOUT_PATTERNS = [
    (re.compile(r'MPC solve failed \((\d+) consecutive\): '
                r'(?P<reason>\S+)\s*\((?P<ms>[\d.]+)\s*ms\)'),
     'solve_failed'),
    (re.compile(r'FK did not converge'), 'fk_did_not_converge'),
    (re.compile(r'MPC controller initialised\s+\((?P<config>.*?)\)'),
     'mpc_init'),
    (re.compile(r'Final tracking error:\s+(?P<pos>[\d.]+)\s*mm,\s*'
                r'(?P<ori>[\d.]+)\s*deg'),
     'final_tracking'),
    (re.compile(r'Solve time:\s+mean=(?P<mean>[\d.]+)\s*ms,\s+'
                r'max=(?P<max>[\d.]+)\s*ms,\s+p95=(?P<p95>[\d.]+)\s*ms'),
     'solve_summary'),
    (re.compile(r'HardwarePlant:\s+(?P<msg>.*)'), 'hardware_plant'),
    (re.compile(r'TargetFeedbackPub:\s+(?P<msg>.*)'), 'target_feedback_pub'),
    (re.compile(r'Logging to:\s+(?P<path>\S+)'), 'logging_to'),
]


def _parse_companion_stdout_log(csv_path: str) -> Dict[str, Any]:
    """Parse an MPC stdout log co-located with the CSV, if present.

    Looks for either `<csv_stem>.log` or `<csv_stem>.stdout.log` next to the
    telemetry CSV.  Returns a structured summary of matched lines plus the
    raw line count.  Unmatched lines are reported as `unmatched_lines` count
    (not stored verbatim, to keep JSON output small).

    Users can capture MPC stdout with:
        python run_mpc.py --pose ... 2>&1 | tee temp/logs/mpc_<ts>.log
    """
    csv_dir = os.path.dirname(csv_path) or '.'
    stem = os.path.basename(csv_path)
    if stem.lower().endswith('.csv'):
        stem = stem[:-4]

    candidates = [
        os.path.join(csv_dir, f'{stem}.log'),
        os.path.join(csv_dir, f'{stem}.stdout.log'),
    ]
    log_path = next((p for p in candidates if os.path.isfile(p)), None)
    if log_path is None:
        return {'available': False,
                'searched': candidates,
                'hint': ("Capture MPC stdout with: "
                         "`python run_mpc.py ... 2>&1 | tee "
                         "temp/logs/mpc_<ts>.log`")}

    result: Dict[str, Any] = {
        'available': True,
        'path': log_path,
        'total_lines': 0,
        'unmatched_lines': 0,
        'events': {},   # category -> count
        'solve_failures': {},  # reason -> count
        'max_consecutive_solve_failures': 0,
        'fk_non_convergences': 0,
        'mpc_config': None,
        'final_tracking': None,
        'solve_summary': None,
        'excerpts': [],  # first occurrence per category (human-readable)
    }

    try:
        with open(log_path, 'r', errors='replace') as f:
            for line in f:
                result['total_lines'] += 1
                matched = False
                for rx, cat in _MPC_STDOUT_PATTERNS:
                    m = rx.search(line)
                    if not m:
                        continue
                    matched = True
                    result['events'][cat] = result['events'].get(cat, 0) + 1
                    if cat == 'solve_failed':
                        reason = m.group('reason')
                        result['solve_failures'][reason] = (
                            result['solve_failures'].get(reason, 0) + 1)
                        try:
                            n_consec = int(m.group(1))
                            result['max_consecutive_solve_failures'] = max(
                                result['max_consecutive_solve_failures'],
                                n_consec)
                        except (TypeError, ValueError):
                            pass
                    elif cat == 'fk_did_not_converge':
                        result['fk_non_convergences'] += 1
                    elif cat == 'mpc_init' and result['mpc_config'] is None:
                        result['mpc_config'] = m.group('config')
                    elif cat == 'final_tracking':
                        result['final_tracking'] = {
                            'pos_mm': float(m.group('pos')),
                            'ori_deg': float(m.group('ori')),
                        }
                    elif cat == 'solve_summary':
                        result['solve_summary'] = {
                            'mean_ms': float(m.group('mean')),
                            'max_ms': float(m.group('max')),
                            'p95_ms': float(m.group('p95')),
                        }
                    # Capture first excerpt per category for human context
                    if not any(e['category'] == cat for e in result['excerpts']):
                        result['excerpts'].append({
                            'category': cat,
                            'line': line.rstrip(),
                        })
                    break
                if not matched:
                    result['unmatched_lines'] += 1
    except Exception as e:
        result['error'] = f'Failed to read stdout log: {e}'

    return result


# ---------------------------------------------------------------------------
# Cross-source correlation
# ---------------------------------------------------------------------------

def correlate_sources(csv_result: Dict[str, Any],
                      ros2_result: Optional[Dict[str, Any]],
                      time_window_s: float = 0.5) -> List[Dict[str, Any]]:
    """Find temporal coincidences between MPC anomalies and ROS2 events.

    For now, this correlates MPC discontinuities and solve time spikes with
    ROS2 error-level events.  Returns a list of correlated findings.
    """
    if ros2_result is None or 'events' not in ros2_result:
        return []

    correlations = []

    # Collect MPC anomaly timestamps
    mpc_anomalies = []
    for jump in csv_result.get('discontinuities', {}).get('cmd_jumps', []):
        mpc_anomalies.append((jump['time_s'], f"cmd discontinuity leg {jump['leg']}: {jump['magnitude_mm']:.1f}mm"))
    for jump in csv_result.get('discontinuities', {}).get('actual_jumps', []):
        mpc_anomalies.append((jump['time_s'], f"actual discontinuity leg {jump['leg']}: {jump['magnitude_mm']:.1f}mm"))

    # Collect ROS2 error events with timestamps
    ros2_errors = []
    for event in ros2_result.get('events', []):
        if event.get('level') in ('ERROR', 'FATAL') and event.get('timestamp') is not None:
            ros2_errors.append((event['timestamp'], event['node'], event['message']))

    # Simple temporal correlation (within time_window_s)
    # Note: MPC time is relative (starts at 0), ROS2 timestamps are absolute.
    # Without a shared clock reference, correlation relies on relative timing
    # between events within each source.  The slash command can refine this
    # using the session start marker timestamp.
    # For now, we just report both sets of anomalies for the LLM to interpret.

    return correlations


# ---------------------------------------------------------------------------
# Flag generation
# ---------------------------------------------------------------------------

def generate_flags(result: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Generate diagnostic flags from analysis results."""
    flags = []

    tracking = result.get('tracking', {})
    solve = result.get('solve_times', {})
    status = result.get('solver_status', {})
    osc = result.get('oscillation', {})
    disc = result.get('discontinuities', {})
    ws = result.get('workspace', {})
    ss = result.get('steady_state', {})

    # Tracking flags
    if tracking.get('worst_leg_ratio', 0) > WORST_LEG_RATIO_THRESHOLD:
        flags.append({
            'severity': 'warning',
            'source': 'mpc',
            'message': (f"Leg {tracking['worst_leg']} tracking "
                        f"{tracking['worst_leg_ratio']:.1f}x worse than median "
                        f"(RMS {tracking['per_leg'][tracking['worst_leg']]['rms_mm']:.3f}mm "
                        f"vs median {tracking['median_leg_rms_mm']:.3f}mm)"),
        })

    # Solve time flags
    if solve.get('first_sample_ms', 0) > 15.0:
        flags.append({
            'severity': 'info',
            'source': 'mpc',
            'message': f"First-sample cold solve: {solve['first_sample_ms']:.1f}ms (expected JIT warmup)",
        })

    if solve.get('max_consecutive_violations', 0) >= 3:
        flags.append({
            'severity': 'error',
            'source': 'mpc',
            'message': (f"MPC solve budget exceeded: {solve['max_consecutive_violations']} "
                        f"consecutive violations > {solve['budget_ms']:.0f}ms "
                        f"(max {solve['max_ms']:.1f}ms)"),
        })
    elif solve.get('budget_violations', 0) > 0:
        flags.append({
            'severity': 'warning',
            'source': 'mpc',
            'message': (f"MPC solve budget exceeded {solve['budget_violations']} times "
                        f"({solve['budget_violation_pct']:.1f}%, max {solve['max_ms']:.1f}ms)"),
        })

    # Solver-status flags — richer than solve-time alone.  A solve can finish
    # under the time cap yet still be a fallback (IPOPT terminated with
    # Maximum_CpuTime_Exceeded at ~cap ms).  `status` counts what IPOPT
    # actually returned per step.
    if status.get('total', 0) > 0:
        timeout_pct = status.get('timeout_pct', 0.0)
        success_pct = status.get('success_rate_pct', 100.0)
        max_consec_timeout = status.get('max_consecutive_timeout', 0)
        other_reasons = status.get('other_failure_reasons', {})

        if timeout_pct >= 50.0:
            flags.append({
                'severity': 'error',
                'source': 'mpc',
                'message': (f"MPC solver saturated: {timeout_pct:.1f}% of steps "
                            f"timed out (Maximum_CpuTime_Exceeded), "
                            f"success rate {success_pct:.1f}%, "
                            f"max {max_consec_timeout} consecutive timeouts"),
            })
        elif timeout_pct >= 10.0 or max_consec_timeout >= 5:
            flags.append({
                'severity': 'warning',
                'source': 'mpc',
                'message': (f"MPC solver timeouts: {timeout_pct:.1f}% of steps, "
                            f"max {max_consec_timeout} consecutive "
                            f"(success rate {success_pct:.1f}%)"),
            })

        if other_reasons:
            # Non-timeout failure reasons (numerical issues, infeasibility, etc.)
            reason_summary = ', '.join(f"{k}×{v}" for k, v in other_reasons.items())
            flags.append({
                'severity': 'warning',
                'source': 'mpc',
                'message': f"Non-timeout solver failures: {reason_summary}",
            })

    # Oscillation flags
    if osc.get('detected', False):
        msg = "Oscillation detected: chatter ratio > 0.5 on legs " + \
              ', '.join(str(i) for i, c in enumerate(osc.get('per_leg_chatter', []))
                        if c > CHATTER_THRESHOLD)
        if osc.get('amplitude_growing', False):
            msg += " (AMPLITUDE GROWING — possible instability)"
            flags.append({'severity': 'error', 'source': 'mpc', 'message': msg})
        else:
            flags.append({'severity': 'warning', 'source': 'mpc', 'message': msg})

    # Discontinuity flags (include time_s so post-E-stop tagging can downgrade
    # STOW-drop discontinuities caused by the operator's E-stop)
    for jump in disc.get('cmd_jumps', []):
        flags.append({
            'severity': 'error',
            'source': 'mpc',
            'time_s': jump['time_s'],
            'message': (f"Command discontinuity: leg {jump['leg']} jumped "
                        f"{jump['magnitude_mm']:.1f}mm at t={jump['time_s']:.3f}s"),
        })

    # Workspace flags
    if ws.get('margin_to_lower_mm', 999) < STROKE_SOFT_MARGIN_MM:
        flags.append({
            'severity': 'warning',
            'source': 'mpc',
            'message': (f"Near lower stroke limit: {ws['margin_to_lower_mm']:.1f}mm margin "
                        f"(min extension {ws['min_extension_mm']:.1f}mm)"),
        })
    if ws.get('margin_to_upper_mm', 999) < STROKE_SOFT_MARGIN_MM:
        flags.append({
            'severity': 'warning',
            'source': 'mpc',
            'message': (f"Near upper stroke limit: {ws['margin_to_upper_mm']:.1f}mm margin "
                        f"(max extension {ws['max_extension_mm']:.1f}mm)"),
        })

    # Steady-state flags — only trust the SS metric when the ref-static
    # window is long enough to average over.  Short-hold runs (5s moves with
    # 3s settling) mix transient into "steady state" and routinely exceed
    # the 0.5mm threshold for uninteresting reasons.
    hold = result.get('hold_phase', {}) or {}
    hold_is_trustworthy = bool(hold.get('detected'))
    if ss.get('has_steady_state') and ss.get('ss_rms_mm', 0) > STEADY_STATE_RMS_THRESHOLD_MM:
        if hold_is_trustworthy:
            flags.append({
                'severity': 'warning',
                'source': 'mpc',
                'message': (f"High steady-state error: {ss['ss_rms_mm']:.3f}mm RMS "
                            f"(threshold {STEADY_STATE_RMS_THRESHOLD_MM}mm, "
                            f"hold window {hold.get('duration_s', 0):.1f}s)"),
            })
        # else: skip the flag.  Hold tail was shorter than HOLD_MIN_TAIL_S, so
        # the SS RMS is dominated by settling transient, not steady state.

    # Hold-phase per-leg quiescence (Level-1 gain-tuning metric).
    if hold.get('detected'):
        asym = hold.get('asymmetry_ratio', 1.0)
        worst_leg = hold.get('worst_leg')
        worst_std = hold.get('worst_act_std_um', 0.0)
        quietest_leg = hold.get('quietest_leg')
        quietest_std = hold.get('quietest_act_std_um', 0.0)
        if asym > HOLD_ASYMMETRY_WARN:
            flags.append({
                'severity': 'warning',
                'source': 'mpc',
                'message': (f"Hold-phase asymmetry: leg {worst_leg} "
                            f"{worst_std:.1f}um stdev vs leg {quietest_leg} "
                            f"{quietest_std:.1f}um ({asym:.1f}x) — candidate "
                            f"for per-leg gain tuning"),
            })
        if worst_std > HOLD_LEG_STD_WARN_UM:
            flags.append({
                'severity': 'warning',
                'source': 'mpc',
                'message': (f"Leg {worst_leg} hold-phase noise "
                            f"{worst_std:.1f}um stdev "
                            f"(above {HOLD_LEG_STD_WARN_UM:.0f}um threshold) "
                            f"over {hold.get('duration_s', 0):.1f}s hold window"),
            })

    # Overhead isolated-spike detection (non-solve latency pops).  These are
    # the GC-pause / interpreter-housekeeping signature: single tick with
    # overhead >> its neighbours AND a short solve.
    oh = result.get('overhead', {}) or {}
    if oh.get('isolated_spike_count', 0) > 0:
        n_spikes = oh['isolated_spike_count']
        max_spike = max((s['overhead_ms'] for s in oh['isolated_spikes']),
                        default=0.0)
        first_spike_t = oh['isolated_spikes'][0]['time_s']
        flags.append({
            'severity': 'warning',
            'source': 'mpc',
            'message': (f"{n_spikes} isolated MPC overhead spike(s): "
                        f"max {max_spike:.1f}ms, first at t={first_spike_t:.3f}s "
                        f"(candidate GC pause — inspect vs solve_time)"),
        })

    # Cmd-spectral HF energy (MPC numerical noise floor visible in cmd_ext).
    spec = result.get('cmd_spectral', {}) or {}
    if spec.get('detected') and spec.get('hf_pct_max', 0.0) > HF_ENERGY_WARN_PCT:
        band = spec.get('band_hf_hz', [10.0, 20.0])
        flags.append({
            'severity': 'info',
            'source': 'mpc',
            'message': (f"Cmd has {spec['hf_pct_max']:.0f}% of energy in "
                        f"{band[0]:.1f}-{band[1]:.1f}Hz band during hold "
                        f"(MPC numerical noise floor — visible in pos_setpoint, "
                        f"below mechanical bandwidth)"),
        })

    # Motion-onset dead-time — the silence between cmd ramp-onset and first
    # actual motion.  Distinct from actual_jumps (which fires on the leap
    # itself).  Worst-event latency drives the severity.
    onset = result.get('motion_onset', {}) or {}
    if onset.get('detected'):
        worst_event = max(onset.get('events', []),
                          key=lambda e: e.get('max_latency_ms', 0.0),
                          default=None)
        agg = onset.get('aggregate', {}) or {}
        if worst_event is not None:
            worst_ms = worst_event['max_latency_ms']
            severity = None
            if worst_ms >= MOTION_ONSET_LATENCY_ERROR_MS:
                severity = 'error'
            elif worst_ms >= MOTION_ONSET_LATENCY_WARN_MS:
                severity = 'warning'
            if severity is not None:
                flags.append({
                    'severity': severity,
                    'source': 'mpc',
                    'time_s': worst_event['onset_time_s'],
                    'message': (
                        f"Motion-onset dead-time: worst leg {worst_ms:.0f}ms "
                        f"at t={worst_event['onset_time_s']:.3f}s "
                        f"(median {worst_event['median_latency_ms']:.0f}ms, "
                        f"sync {worst_event['sync_window_ms']:.0f}ms, "
                        f"first-tick leap {worst_event['max_first_tick_leap_mm']:.2f}mm); "
                        f"session median {agg.get('median_latency_ms', 0):.0f}ms "
                        f"over {agg.get('total_onsets_detected', 0)} onsets — "
                        f"stick-slip / backlash signature"),
                })

    # Plant telemetry collapse — exogenous failure (motor guard death, CAN loss,
    # operator killing the launch).  Emit at error severity with time_s set to
    # the overspeed event; _tag_post_event_flags will downgrade everything after
    # it so MPC-side flags are not blamed.
    collapse = result.get('plant_collapse', {})
    if collapse.get('detected'):
        flags.append({
            'severity': 'error',
            'source': 'plant',
            'time_s': collapse['overspeed_time_s'],
            'plant_collapse_event': True,
            'message': (
                f"Plant telemetry collapse: uncommanded overspeed "
                f"(peak {collapse['peak_abs_leg_vel_mmps']:.0f} mm/s "
                f">{collapse['overspeed_threshold_mmps']:.0f} threshold) "
                f"at t={collapse['overspeed_time_s']:.3f}s, followed by "
                f"{collapse['zero_run_length']}-step zero-extension sentinel "
                f"from t={collapse['zero_run_start_time_s']:.3f}s — "
                f"exogenous failure, downstream MPC flags are consequences."),
        })

    # Overshoot-induced NLP saturation — distinct from generic MPC_STALENESS.
    # Fires when IPOPT spends every ms on solve setup without completing one
    # iteration while the plant is ahead of cmd and ref is still moving.
    # Warning severity: the Tier-1 fallback (cmd-stream extrapolation with
    # 500 ms cold_hold escalation; logbook
    # 2026-05-20-hold-extrap-positive-feedback-chaotic-motion.md) keeps
    # this safe, but it IS a tracking hole that matters for dynamic motion.
    overshoot = result.get('overshoot_saturation', {})
    if overshoot.get('detected'):
        ev0 = overshoot['events'][0]
        n_events = len(overshoot['events'])
        flags.append({
            'severity': 'warning',
            'source': 'mpc',
            'time_s': ev0['start_time_s'],
            'message': (
                f"MPC overshoot saturation: "
                f"{overshoot['max_consecutive']} consecutive ipopt_iter=0 "
                f"steps with tracking err peak {ev0['peak_tracking_err_mm']:.1f}mm "
                f"while ref still moving, starting t={ev0['start_time_s']:.3f}s"
                + (f" ({n_events} events total)" if n_events > 1 else "")),
        })

    # ROS2 flags
    ros2 = result.get('ros2_events_summary') or {}
    error_count = (ros2.get('level_counts') or {}).get('ERROR', 0)
    fatal_count = (ros2.get('level_counts') or {}).get('FATAL', 0)
    if fatal_count > 0:
        flags.append({
            'severity': 'error',
            'source': 'ros2',
            'message': f"{fatal_count} FATAL-level events in ROS2 logs",
        })
    if error_count > 0:
        flags.append({
            'severity': 'warning',
            'source': 'ros2',
            'message': f"{error_count} ERROR-level events in ROS2 logs",
        })

    # Sort by severity
    severity_order = {'error': 0, 'warning': 1, 'info': 2}
    flags.sort(key=lambda f: severity_order.get(f['severity'], 3))

    return flags


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _parse_csv_start_time_s(csv_path: str) -> Optional[float]:
    """Extract wall-clock start time (unix seconds) from an mpc_<YYYYMMDD>_<HHMMSS>.csv.

    Returns None if the filename doesn't match.  Used to align CSV relative
    time (t=0 at session start) with rosbag absolute time.
    """
    import re as _re
    from datetime import datetime as _dt
    m = _re.search(r'mpc_(\d{8})_(\d{6})', os.path.basename(csv_path))
    if not m:
        return None
    try:
        return _dt.strptime(m.group(1) + m.group(2), '%Y%m%d%H%M%S').timestamp()
    except ValueError:
        return None


def _estop_csv_time_s(rosbag_result: Dict[str, Any],
                      csv_path: str) -> Optional[float]:
    """Translate the E-stop's rosbag-relative time into CSV-relative time.

    CSV time starts at 0 when the MPC session begins; rosbag time starts at 0
    when recording began.  Alignment comes from the CSV filename timestamp
    (wall-clock session start) minus the rosbag start time.  Returns None if
    an E-stop wasn't detected or alignment isn't possible.
    """
    if not rosbag_result or not rosbag_result.get('available'):
        return None
    estop = rosbag_result.get('estop') or {}
    if not estop.get('detected'):
        return None

    bag_t_estop = estop['bag_time_s']
    bag_start = rosbag_result.get('start_time_s')
    csv_start = _parse_csv_start_time_s(csv_path)
    if bag_start is None or csv_start is None:
        return None

    # CSV-relative time = rosbag-absolute time - CSV start time
    # rosbag-absolute time = bag_start + bag_t_estop
    return (bag_start + bag_t_estop) - csv_start


def _tag_post_event_flags(flags: List[Dict[str, Any]],
                          estop_csv_t: Optional[float],
                          estop_bag_t: Optional[float],
                          collapse_csv_t: Optional[float] = None) -> None:
    """Mark flags that occur at or after an E-stop or plant-collapse event
    as downstream consequences.

    Downstream flags are tagged with `post_estop: True` (for E-stop) or
    `post_collapse: True` (for plant-telemetry collapse) and downgraded to
    'info' severity with an explanatory prefix.  Rationale: the triggering
    event collapses the bus / kills telemetry, which causes commanded STOW
    drops, motor disarms, tracking blowups, zero-sentinel feedback, etc.
    These are consequences of the exogenous failure, not independent faults.

    The event whose flag itself carries `estop_event` or `plant_collapse_event`
    is exempt from downgrading — we keep it at its original severity so it
    remains visible as the trigger.
    """
    if (estop_csv_t is None and estop_bag_t is None
            and collapse_csv_t is None):
        return

    # 20ms ≈ 1 MPC step; absorbs the sub-step granularity of the trigger time.
    TOL = 0.020

    def _after_event(flag: Dict[str, Any]) -> Optional[str]:
        """Return the event kind that already happened, or None."""
        t = flag.get('time_s')
        if t is None:
            return None
        src = flag.get('source', 'mpc')
        # Plant collapse is always in CSV time — applies to any mpc/plant source.
        if (collapse_csv_t is not None
                and src in ('mpc', 'plant')
                and t >= collapse_csv_t - TOL):
            return 'collapse'
        # E-stop comes from the rosbag; 'rosbag' flags use bag time, others CSV.
        if src == 'mpc' and estop_csv_t is not None and t >= estop_csv_t - TOL:
            return 'estop'
        if src == 'rosbag' and estop_bag_t is not None and t >= estop_bag_t - TOL:
            return 'estop'
        return None

    for f in flags:
        # Never downgrade the trigger-marker flag itself.
        if f.get('estop_event') or f.get('plant_collapse_event'):
            continue
        kind = _after_event(f)
        if kind is None:
            continue
        f['original_severity'] = f.get('severity')
        f['severity'] = 'info'
        if kind == 'collapse':
            f['post_collapse'] = True
            f['message'] = f"[post-collapse downstream] {f['message']}"
        else:
            f['post_estop'] = True
            f['message'] = f"[post-E-stop downstream] {f['message']}"


# Back-compat alias — keep the old name callable.
def _tag_post_estop_flags(flags: List[Dict[str, Any]],
                          estop_csv_t: Optional[float],
                          estop_bag_t: Optional[float]) -> None:
    _tag_post_event_flags(flags, estop_csv_t, estop_bag_t, collapse_csv_t=None)


def run_diagnosis(csv_path: str,
                  ros_log_dir: Optional[str] = None,
                  rosbag_path: Optional[str] = None,
                  budget_ms: float = DEFAULT_BUDGET_MS,
                  plots: Any = None) -> Dict[str, Any]:
    """Run full diagnosis and return structured result.

    plots: None=no plots, 'auto'=auto-select, list of str=specific categories.
    """
    result = {
        'file': os.path.basename(csv_path),
        'source': 'hardware' if 'hardware' in csv_path.lower() else 'unknown',
    }  # type: Dict[str, Any]

    # Detect source from filename
    basename = os.path.basename(csv_path)
    if basename.startswith('mpc_'):
        result['source'] = 'mpc'

    # MPC CSV analysis
    records = load_csv(csv_path)
    csv_result = analyse_csv(records, budget_ms)
    result.update(csv_result)

    # Optional: companion MPC stdout log
    result['mpc_stdout'] = _parse_companion_stdout_log(csv_path)

    # ROS2 log analysis
    if ros_log_dir:
        ros2_result = parse_ros2_log_dir(ros_log_dir)
        result['ros2_events_summary'] = {
            k: v for k, v in ros2_result.items() if k != 'events'
        }
        result['ros2_events'] = ros2_result.get('events', [])
    else:
        ros2_result = None
        result['ros2_events_summary'] = None
        result['ros2_events'] = []

    # Rosbag analysis
    if rosbag_path:
        result['rosbag'] = analyse_rosbag(rosbag_path)
    else:
        result['rosbag'] = None

    # Session group — derived from the rosbag directory name.  All CSVs sharing
    # the same rosbag belong to the same operational session, even though each
    # /diagnose run targets one CSV at a time.  Downstream (log_index.json,
    # /investigate --latest) uses this to auto-group moves.
    if rosbag_path:
        result['session_group'] = os.path.basename(os.path.normpath(rosbag_path))
    else:
        result['session_group'] = None

    # Cross-source correlation
    result['correlations'] = correlate_sources(
        csv_result, ros2_result)

    # E-stop alignment: translate bag time to CSV time for downstream tagging
    rosbag_res = result.get('rosbag') or {}
    estop = (rosbag_res.get('estop') or {}) if isinstance(rosbag_res, dict) else {}
    estop_csv_t = _estop_csv_time_s(rosbag_res, csv_path) if estop.get('detected') else None
    estop_bag_t = estop.get('bag_time_s') if estop.get('detected') else None
    result['estop_alignment'] = {
        'detected': bool(estop.get('detected')),
        'estop_bag_time_s': estop_bag_t,
        'estop_csv_time_s': (round(estop_csv_t, 3)
                             if estop_csv_t is not None else None),
    }

    # Generate flags (attach time_s where applicable so we can tag downstream)
    flags = generate_flags(result)

    # Tag post-E-stop and post-plant-collapse flags as downstream consequences.
    # A plant collapse (operator killed the launch, motor_guard died, CAN lost)
    # produces the same kind of cascading downstream noise an E-stop does:
    # tracking blowups, workspace-margin trips, MPC solver timeouts against
    # stale feedback. Without this tagging, the MPC flags steal the blame.
    collapse = (result.get('plant_collapse') or {})
    collapse_csv_t = (collapse.get('overspeed_time_s')
                      if collapse.get('detected') else None)
    _tag_post_event_flags(flags, estop_csv_t, estop_bag_t,
                          collapse_csv_t=collapse_csv_t)

    # Add an info-severity marker for the E-stop itself (not a fault — pilot
    # intervention).  Place it first so it reads at the top of the flag list
    # after sorting by severity.
    if estop.get('detected'):
        flags.append({
            'severity': 'info',
            'source': 'rosbag',
            'message': (
                f"Pilot E-stop detected at bag_t={estop['bag_time_s']:.3f}s"
                + (f" (CSV t={estop_csv_t:.3f}s)" if estop_csv_t is not None else "")
                + f" — {estop.get('evidence','')}. "
                "This is operator-initiated; look upstream for what prompted it."
            ),
            'estop_event': True,
            'time_s': estop_csv_t if estop_csv_t is not None else estop['bag_time_s'],
        })

    # Re-sort by severity after tagging
    severity_order = {'error': 0, 'warning': 1, 'info': 2}
    flags.sort(key=lambda f: severity_order.get(f.get('severity', 'info'), 3))
    result['flags'] = flags

    # Generate diagnostic plots (if requested)
    # plots=None => no plots, 'auto' => auto-select, list => specific categories
    if plots is not None:
        prefix = csv_path
        if prefix.lower().endswith('.csv'):
            prefix = prefix[:-4]
        categories = None if plots == 'auto' else plots
        result['plots'] = generate_diagnostic_plots(
            records, result, prefix, categories=categories)
        result['plots_generated'] = list(result['plots'].keys())
    else:
        result['plots'] = {}
        result['plots_generated'] = []

    return result


def main():
    parser = argparse.ArgumentParser(
        description='Hardware diagnosis analysis engine')
    parser.add_argument('csv_path', help='Path to MPC telemetry CSV')
    parser.add_argument('--ros-log-dir', default=None,
                        help='Path to ROS2 session log directory')
    parser.add_argument('--rosbag', default=None,
                        help='Path to rosbag (MCAP) recording directory')
    parser.add_argument('--budget-ms', type=float, default=DEFAULT_BUDGET_MS,
                        help=f'MPC solve budget in ms (default: {DEFAULT_BUDGET_MS})')
    parser.add_argument('--json', action='store_true',
                        help='Output structured JSON (for slash command consumption)')
    parser.add_argument('--plots', default=None, metavar='CATEGORIES',
                        help='Generate plots: "auto", "all", "none", or '
                             'comma-separated list (e.g. legs,solver,tracking)')
    parser.add_argument('--html', action='store_true', default=True,
                        help='Generate interactive HTML report (default: on)')
    parser.add_argument('--no-html', dest='html', action='store_false',
                        help='Disable HTML report generation')
    parser.add_argument('--static-plots', action='store_true', default=False,
                        help='Use static matplotlib PNGs instead of interactive Plotly')
    args = parser.parse_args()

    # --html implies --plots auto (unless user explicitly passed --plots)
    if args.html and args.plots is None:
        args.plots = 'auto'

    # Parse --plots: None=no plots, 'auto'=auto-select, list=specific categories
    plot_cats = None  # default: no plots
    if args.plots is not None:
        parsed = parse_categories(args.plots)
        # parse_categories returns None for 'auto', [] for 'none', list for specific
        if parsed is None:
            plot_cats = 'auto'
        elif parsed == []:
            plot_cats = None  # 'none' => no plots
        else:
            plot_cats = parsed

    # Run the core analysis (no matplotlib plots for the default Plotly path)
    use_static = args.static_plots
    result = run_diagnosis(
        csv_path=args.csv_path,
        ros_log_dir=args.ros_log_dir,
        rosbag_path=args.rosbag,
        budget_ms=args.budget_ms,
        plots=plot_cats if use_static else None,
    )

    # Generate HTML report
    if args.html and plot_cats is not None:
        html_prefix = args.csv_path
        if html_prefix.lower().endswith('.csv'):
            html_prefix = html_prefix[:-4]
        html_path = f'{html_prefix}_report.html'

        if use_static:
            # Static matplotlib PNGs embedded in HTML
            from sim.analysis.report_html import generate_html_report
            html_path = generate_html_report(result, html_path)
            print(f'Static HTML report: {html_path}', file=sys.stderr)
        else:
            # Interactive Plotly report (default)
            try:
                from sim.analysis.plot_interactive import generate_interactive_report
                # Load records for Plotly (run_diagnosis doesn't expose them)
                telemetry_records = load_csv(args.csv_path)
                categories = None if plot_cats == 'auto' else plot_cats
                html_path = generate_interactive_report(
                    telemetry_records, result, html_path,
                    categories=categories)
                print(f'Interactive report: {html_path}', file=sys.stderr)
            except ImportError as exc:
                print(f'[diagnose] Plotly not available ({exc}), '
                      f'falling back to static plots. '
                      f'Install with: pip install plotly', file=sys.stderr)
                # Re-run with matplotlib plots and generate static HTML
                result = run_diagnosis(
                    csv_path=args.csv_path,
                    ros_log_dir=args.ros_log_dir,
                    rosbag_path=args.rosbag,
                    budget_ms=args.budget_ms,
                    plots=plot_cats,
                )
                from sim.analysis.report_html import generate_html_report
                html_path = generate_html_report(result, html_path)
                print(f'Static HTML report: {html_path}', file=sys.stderr)

        result['html_report'] = html_path

    if args.json:
        # Compact JSON for machine consumption
        print(json.dumps(result, indent=2, default=str))
    else:
        # Human-readable summary
        print(f"\n{'=' * 70}")
        print(f"  HARDWARE DIAGNOSIS: {result['file']}")
        print(f"{'=' * 70}")
        print(f"  Samples: {result['n_samples']}, "
              f"Duration: {result['duration_s']:.1f}s, "
              f"dt: {result['dt_median_ms']:.1f}ms")
        print()

        t = result.get('tracking', {})
        if t:
            print(f"  TRACKING: pos RMS={t['position_rms_mm']:.3f}mm, "
                  f"peak={t['position_peak_mm']:.3f}mm, "
                  f"worst leg={t['worst_leg']} ({t['worst_leg_ratio']:.1f}x)")

        s = result.get('solve_times', {})
        if s.get('n_valid', 0) > 0:
            print(f"  SOLVE: p50={s['p50_ms']:.1f}ms, p95={s['p95_ms']:.1f}ms, "
                  f"max={s['max_ms']:.1f}ms, violations={s['budget_violations']}")

        print()
        flags = result.get('flags', [])
        if flags:
            print(f"  FLAGS ({len(flags)}):")
            for f in flags:
                print(f"    [{f['severity'].upper():>7s}] [{f['source']}] {f['message']}")
        else:
            print("  No flags — clean run.")
        print(f"{'=' * 70}")


if __name__ == '__main__':
    main()
