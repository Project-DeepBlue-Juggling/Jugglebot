# -*- coding: utf-8 -*-
"""Hardware diagnosis analysis engine.

Reads MPC telemetry CSVs and rosbag (MCAP) recordings to produce a structured
JSON diagnostic report.  Designed to be invoked by the /diagnose Claude Code
slash command, which interprets the output and presents it to the user.

Primary data sources:
  - MPC telemetry CSV (sim/logs/mpc_*.csv) — 55-field StepRecord at 40 Hz
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

# Ensure sim/ and repo root are importable
_analysis_dir = os.path.dirname(os.path.abspath(__file__))
_sim_dir = os.path.dirname(_analysis_dir)
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)
_repo_root = os.path.dirname(_sim_dir)
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)

from analysis.compare import load_csv, estimate_tau
from analysis.plot_diagnosis import generate_diagnostic_plots, parse_categories
from viz.telemetry import StepRecord


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


def analyse_oscillation(records: List[StepRecord]) -> Dict[str, Any]:
    """Detect oscillation via chatter ratio (sign changes in command deltas)."""
    n = len(records)
    if n < 10:
        return {'detected': False, 'per_leg_chatter': [], 'per_dof_chatter': []}

    # Per-leg chatter (command extensions)
    per_leg_chatter = []
    for leg in range(6):
        cmd = _extract(records, f'cmd_ext_{leg}')
        delta = np.diff(cmd)
        if len(delta) < 2:
            per_leg_chatter.append(0.0)
            continue
        signs = np.sign(delta)
        # Remove zeros (no change)
        nonzero = signs[signs != 0]
        if len(nonzero) < 2:
            per_leg_chatter.append(0.0)
            continue
        sign_changes = np.sum(np.abs(np.diff(nonzero)) > 0)
        chatter = float(sign_changes / len(nonzero))
        per_leg_chatter.append(round(chatter, 3))

    # Per-DoF chatter (actual pose)
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


def analyse_csv(records: List[StepRecord],
                budget_ms: float = DEFAULT_BUDGET_MS) -> Dict[str, Any]:
    """Run all MPC CSV analyses and return combined results."""
    n = len(records)
    if n == 0:
        return {'error': 'Empty CSV'}

    times = _extract(records, 'time')
    duration = float(times[-1] - times[0])
    dt_values = np.diff(times)
    dt_median = float(np.median(dt_values)) if len(dt_values) > 0 else 0

    return {
        'n_samples': n,
        'duration_s': round(duration, 3),
        'dt_median_ms': round(dt_median * 1000, 2),
        'tracking': analyse_tracking(records),
        'solve_times': analyse_solve_times(records, budget_ms),
        'oscillation': analyse_oscillation(records),
        'discontinuities': analyse_discontinuities(records),
        'workspace': analyse_workspace(records),
        'steady_state': analyse_steady_state(records),
        'torques': analyse_torques(records),
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

def analyse_rosbag(rosbag_path: str) -> Dict[str, Any]:
    """Analyse a rosbag recording for diagnostic information.

    Uses the rosbags library (pure Python MCAP reader).  Degrades gracefully
    if rosbags is not installed.
    """
    try:
        from rosbags.rosbag2 import Reader
        from rosbags.serde import deserialize_cdr
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
        with Reader(rosbag_path) as reader:
            # Report available topics
            topic_info = {}
            for conn in reader.connections:
                topic_info[conn.topic] = {
                    'msgtype': conn.msgtype,
                    'count': 0,
                }

            # Count messages per topic (lightweight scan)
            for conn, _ts, _data in reader.messages():
                if conn.topic in topic_info:
                    topic_info[conn.topic]['count'] += 1

            results['topics'] = topic_info
            results['topics_found'] = list(topic_info.keys())

            # TODO: Deep analysis of specific topics (robot_state, leg_lengths,
            # tracking_error, diagnostics) will be added as we understand the
            # exact message types and their CDR layout on the Jetson.
            # For now, topic inventory and message counts are sufficient for
            # the slash command to report what data is available.

    except Exception as e:
        results['error'] = f'Failed to read rosbag: {e}'

    return results


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

    # Discontinuity flags
    for jump in disc.get('cmd_jumps', []):
        flags.append({
            'severity': 'error',
            'source': 'mpc',
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

    # Steady-state flags
    if ss.get('has_steady_state') and ss.get('ss_rms_mm', 0) > STEADY_STATE_RMS_THRESHOLD_MM:
        flags.append({
            'severity': 'warning',
            'source': 'mpc',
            'message': (f"High steady-state error: {ss['ss_rms_mm']:.3f}mm RMS "
                        f"(threshold {STEADY_STATE_RMS_THRESHOLD_MM}mm)"),
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

    # Cross-source correlation
    result['correlations'] = correlate_sources(
        csv_result, ros2_result)

    # Generate flags
    result['flags'] = generate_flags(result)

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
                        help='Generate self-contained HTML report (default: on)')
    parser.add_argument('--no-html', dest='html', action='store_false',
                        help='Disable HTML report generation')
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

    result = run_diagnosis(
        csv_path=args.csv_path,
        ros_log_dir=args.ros_log_dir,
        rosbag_path=args.rosbag,
        budget_ms=args.budget_ms,
        plots=plot_cats,
    )

    # Generate HTML report if requested
    if args.html:
        from analysis.report_html import generate_html_report
        html_prefix = args.csv_path
        if html_prefix.lower().endswith('.csv'):
            html_prefix = html_prefix[:-4]
        html_path = generate_html_report(result, f'{html_prefix}_report.html')
        result['html_report'] = html_path
        print(f'HTML report: {html_path}', file=sys.stderr)

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
