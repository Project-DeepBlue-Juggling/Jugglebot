#!/usr/bin/env python3
"""Production-faithful offline replay of every BB (Ball Butler) calibration sweep in the bags.

What it replays
    Each operator yaw sweep (heartbeat state CALIBRATING -> not CALIBRATING) found in
    ~/Desktop/rosbags/2026-10-1[0-4]_*, through the SAME functions ``mocap_node`` calls, with
    the SAME inputs rebuilt from the bag (/mocap_data, /bb/heartbeat, /bb/axis_estimates):
    ``bb_calibration.run_calibration`` (template fit, lag fit, axis fit, yaw offset),
    ``bb_base_frame.estimate_base_pose`` / ``bb_in_base`` / ``pool_kappa`` /
    ``check_base_frame_consistency`` and ``bb_calibration.check_calibration_consistency``.
    The node's gate STATE (the kappa pool + base_ref of bb_base_frame_state.json, and the
    world-gate reference of bb_calibration_last_accepted.json) is simulated in MEMORY,
    chronologically across bags, with the node's own persistence rules; nothing under
    ~/bb_calibration_sessions is read or written.

    Each sweep gets two verdicts: ``verdict_current`` (what the node's gates say, in sequence)
    and ``verdict_intrinsic`` (history-free checks only: the ValueErrors of run_calibration /
    estimate_sweep_yaw_offset incl. CONSTELLATION_RESIDUAL at 1.0 mm; never delta-kappa, delta-yaw,
    delta-axis or the 0.5 mm template-residual gate). ``advisory_*`` columns are the history
    distances the intrinsic verdict ignores.

Fidelity contract
    For the sweeps the node logged (~/.ros/log/<launch>/launch.log), the replayed yaw offset,
    kappa, template residual, lag, sigma and the verdict are compared with the logged values
    (tolerances in TOL). A mismatch is reported, never tuned away.

Provenance
    logbook/2026-10-14-bb-calibration-one-attempt-investigation.md;
    plans/active/bb-calibration-one-attempt.md Phase 0. The Phase 0 test unit references this
    probe (``--emit-fixture`` writes the one-sweep re-solve fixture it needs).

Differences from the node that cannot be removed offline (listed in summary.md as well)
    * heartbeat yaw samples are stamped with the bag message's log_time, not the node's
      get_clock().now() (only matters when the yaw source falls back to the heartbeat);
    * /bb/markers is not in the bags: the per-index BB rows are rebuilt from the labelled
      markers of /mocap_data ('Ball Butler - i'), the way mocap_interface builds them;
    * QTM_DROPOUT_MID_SWEEP (is_receiving) cannot be evaluated; only the 60 s timeout is;
    * the running BaseFrameMonitor (fallback pose when a window misses the base) is fed at its
      0.2 s cadence only from 40 s before a sweep, not from process start;
    * accepted_at stamps are the sweep's end time, not datetime.now() (not read by any gate).

Usage
    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/probes/bb_calibration_replay.py [-v] [--bags GLOB] [--sweep BAG:IDX]
        [--bb-yaw-source auto|stamped|heartbeat] [--bb-pose-source auto|sweep|base_frame]
        [--ros-log-dir DIR] [--no-bb-moved] [--state-json PATH]
        [--emit-fixture BAG:IDX --out PATH]
"""
from __future__ import annotations

import argparse
import copy
import csv
import datetime
import glob
import json
import math
import os
import re
import sys
import time

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'ros_ws', 'src', 'jugglebot'))

from jugglebot.protocol_config import BallButlerStates  # noqa: E402
import jugglebot.hardware_config as hw  # noqa: E402
from jugglebot.bb_calibration import (  # noqa: E402
    run_calibration, load_marker_template, check_calibration_consistency,
    stamped_yaw_samples_from_joint_state, BB_MARKER_COUNT, sweep_kinematics,
)
from jugglebot.bb_base_frame import (  # noqa: E402
    BaseFrameMonitor, KappaRecord, BASE_MAX_RESIDUAL_MM, DEFAULT_MIN_POOLED_SWEEPS,
    base_candidate_points, bb_in_base, check_base_frame_consistency, compose_bb_pose,
    estimate_base_pose, kappa_sweep_sd_deg, load_base_frame, pool_kappa, MAX_POOLED_RECORDS,
)

#: Node constants mirrored from mocap_node.py (asserted against the module text in --selfcheck).
MIN_STAMPED_YAW_SAMPLES = 100
CALIBRATION_TIMEOUT_S = 60.0
BASE_MONITOR_PERIOD_S = 0.2
BASE_MONITOR_MAX_AGE_S = 30.0
MONITOR_PRE_S = 40.0

#: Fidelity tolerances (replay brief).
TOL = {'yaw_offset_deg': 0.002, 'kappa_deg': 0.002, 'template_residual_mm': 0.005,
       'yaw_lag_ms': 0.5, 'sigma_deg': 0.002}

RES_DIR = os.path.join(REPO, 'ros_ws', 'src', 'jugglebot', 'resources')
BAG_ROOT = os.path.expanduser('~/Desktop/rosbags')


# ───────────────────────────── bag reading ─────────────────────────────

def _reader(path):
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    return make_reader(open(path, 'rb'), decoder_factories=[DecoderFactory()])


def bag_mcap(bag_dir):
    p = sorted(glob.glob(os.path.join(bag_dir, '*.mcap')))
    return p[0] if p else None


def read_heartbeats(path):
    out = []
    for _s, _c, m, d in _reader(path).iter_decoded_messages(topics=['/bb/heartbeat']):
        out.append((m.log_time * 1e-9, int(d.state), float(d.yaw_deg)))
    return out


def read_events(path, windows):
    """Generator of (log_time_ns, topic, decoded) for /mocap_data and /bb/axis_estimates inside
    the (non-overlapping, sorted) windows ([(t0, t1)] unix s), in log_time order. Streamed: a long
    window decoded into a list is several GB."""
    rd = _reader(path)
    for t0, t1 in windows:
        for _s, ch, m, d in rd.iter_decoded_messages(
                topics=['/mocap_data', '/bb/axis_estimates'],
                start_time=int(t0 * 1e9), end_time=int(t1 * 1e9)):
            yield (m.log_time, ch.topic, d)


# ───────────────────────── logged (truth) parsing ─────────────────────────

_LINE = re.compile(r'^(\d+\.\d+) \[([^\]]+)\] \[(\w+)\] \[[\d.]+\] \[([^\]]+)\]: (.*)$')


def _f(rx, s, cast=float):
    m = re.search(rx, s)
    return cast(m.group(1)) if m else None


def refusal_class(text):
    mm = re.match(r'([A-Z_]{6,}):', text)
    if mm:
        return mm.group(1)
    if text.startswith('Circle centres'):
        return 'CIRCLE_CENTRES'
    return 'OTHER'


def parse_launch_logs(ros_log_dir):
    """(sweeps, arms): the node's logged sweeps 2026-10-10..14 and every 'bb_moved armed' time.
    Launches 2026-10-11..14 only (the 10-10 launches ran older code: scored mismatches there are version, not fidelity). A sweep: t_start, outcome, refusal_class, and the numbers of its 'Yaw offset' DEBUG line."""
    sweeps, arms = [], []
    for lp in sorted(glob.glob(os.path.join(ros_log_dir, '2026-10-1[1-4]-*', 'launch.log'))):
        ld = os.path.basename(os.path.dirname(lp))
        cur = None
        idx = 0
        for raw in open(lp, errors='replace'):
            m = _LINE.match(raw.rstrip('\n'))
            if not m or m.group(4) != 'mocap_node':
                continue
            t, msg = float(m.group(1)), m.group(5)
            if 'bb_moved armed' in msg:
                arms.append(t)
            if 'BB calibration started' in msg:
                idx += 1
                cur = {'launch': ld, 'idx_in_launch': idx, 't_start': t, 'outcome': None,
                       'class': None}
                sweeps.append(cur)
                continue
            if cur is None:
                continue
            if 'Yaw offset' in msg:
                cur['yaw_offset_deg'] = _f(r'Yaw offset (?:\(sweep\) )?([+-]\d+\.\d+)°', msg)
                cur['template_residual_mm'] = _f(r'template residual ([\d.]+) mm', msg)
                cur['sigma_deg'] = _f(r'σ ([\d.]+)°', msg)
                cur['yaw_lag_ms'] = _f(r'lag ([+-]?[\d.]+) ms', msg)
                cur['kappa_deg'] = _f(r'BB-in-base κ ([+-][\d.]+)°', msg)
                cur['n_frames'] = _f(r'(\d+) frames \(', msg, int)
                cur['gate_pooled_n'] = _f(r'pooled BB-in-base \(n (\d+)', msg, int)
            if 'Published calibration result' in msg:
                cur['outcome'], cur['class'] = 'ACCEPTED', 'ACCEPTED'
            elif 'BB calibration FAILED' in msg:
                body = msg.split('FAILED: ', 1)[1]
                cur['outcome'], cur['class'] = 'REFUSED', refusal_class(body)
    return sweeps, arms


# ───────────────────────────── node simulation ─────────────────────────────

class GateState:
    """The two persisted states, in memory: kappa pool + base_ref, and the world reference."""

    def __init__(self, base_model, snapshot=None):
        self.records = list(base_model.records)
        self.base_ref = base_model.base_ref
        self.world_ref = None          # {'yaw_offset_deg', 'yaw_offset_std_deg', 'position_mm', 'accepted_at'}
        if snapshot:
            bs = snapshot.get('base_state')
            if bs:
                self.records = [KappaRecord.from_dict(r) for r in bs['records']]
                self.base_ref = bs.get('base_ref') or self.base_ref
            self.world_ref = snapshot.get('accepted') or None

    def dump(self):
        return {'records': [r.to_dict() for r in self.records], 'base_ref': self.base_ref,
                'world_ref': self.world_ref}


class NodeSim:
    """One node process (one bag): the calibration window state machine and the finalise path."""

    def __init__(self, args, template, base_model, gate, bb_moved_arms):
        self.args, self.template, self.base_model, self.gate = args, template, base_model, gate
        self.bb_moved_arms = sorted(bb_moved_arms)     # unix s, this launch
        self.bb_moved_armed = False
        self.last_state = None
        self.calibrating = False
        self.blocked = False
        self.invalid = None
        self.t_start = None
        self.reset_window()
        self.monitor = BaseFrameMonitor(base_model.template_mm)
        self.monitor_last_t = None
        self.rows = []
        self.capture = None            # raw-input capture of the most recent sweep
        self.keep_capture = False
        self.capture_idx = None        # 1-based sweep number in this bag to capture (None: last)

    def reset_window(self):
        self.calib_data = {}
        self.calib_times = {}
        self.yaw_readings = []
        self.frames = []
        self.last_frame_ns = None
        self.yaw_samples = []
        self.stamped = []
        self.base_points = []
        self.last_base_ns = None

    # -- events ------------------------------------------------------------
    def _arm_until(self, t):
        while self.bb_moved_arms and self.bb_moved_arms[0] <= t:
            self.bb_moved_arms.pop(0)
            self.bb_moved_armed = True

    def heartbeat(self, t, state, yaw):
        self._arm_until(t)
        previous = self.last_state
        if state != self.last_state:
            self.last_state = state
        if state != BallButlerStates.CALIBRATING:
            self.blocked = False
        if state == BallButlerStates.CALIBRATING and not self.calibrating and not self.blocked:
            self.calibrating = True
            self.invalid = None
            self.t_start = t
            self.reset_window()
            self.calib_data = {i: [] for i in range(BB_MARKER_COUNT)}
            self.calib_times = {i: [] for i in range(BB_MARKER_COUNT)}
        if self.calibrating and self.invalid is None:
            self.yaw_readings.append(yaw)
            self.yaw_samples.append((t, float(yaw)))
        if previous == BallButlerStates.CALIBRATING and state != BallButlerStates.CALIBRATING:
            if self.calibrating:
                if self.invalid is not None:
                    self._row_refused(t, self.invalid, 'solver skipped (window invalidated)')
                elif state == BallButlerStates.ERROR:
                    self._row_refused(t, 'BB entered ERROR state during calibration',
                                      'BB entered ERROR state during calibration')
                else:
                    self._finalize(t)
                self.end_window()

    def end_window(self, blocked=False):
        self.calibrating = False
        self.blocked = blocked
        self.reset_window()

    def check_timeout(self, t):
        if self.calibrating and t - self.t_start > CALIBRATION_TIMEOUT_S:
            if self.invalid is None:
                self.invalid = 'CALIBRATION_TIMEOUT'
                self._row_refused(t, 'CALIBRATION_TIMEOUT: BB never left CALIBRATING',
                                  'CALIBRATION_TIMEOUT')
            self.end_window(blocked=True)

    def axis(self, d):
        if self.calibrating and self.invalid is None:
            self.stamped.extend(stamped_yaw_samples_from_joint_state(d))

    def mocap(self, t, d):
        stamp_ns = d.stamp.sec * 1_000_000_000 + d.stamp.nanosec
        frame_ns = stamp_ns if stamp_ns > 0 else None
        labelled, unlabelled = [], []
        bb_rows = [None] * BB_MARKER_COUNT
        for mk in d.markers:
            p = mk.position
            lab = mk.label
            if lab:
                labelled.append((lab, p.x, p.y, p.z, mk.residual))
                m = re.fullmatch(r'Ball Butler - (\d+)', lab)
                if m and 1 <= int(m.group(1)) <= BB_MARKER_COUNT:
                    bb_rows[int(m.group(1)) - 1] = (p.x, p.y, p.z)
            else:
                unlabelled.append((p.x, p.y, p.z, mk.residual))
        # base feed (_publish_frame: _base_feed_due -> _feed_base_frame)
        if frame_ns is not None:
            due_window = self.calibrating and self.invalid is None and frame_ns != self.last_base_ns
            tt = frame_ns * 1e-9
            due_mon = self.monitor_last_t is None or tt - self.monitor_last_t >= BASE_MONITOR_PERIOD_S
            if due_window or due_mon:
                pts = base_candidate_points(unlabelled, labelled)
                if pts.shape[0]:
                    if due_window:
                        self.last_base_ns = frame_ns
                        self.base_points.append((tt, np.array(pts[:, :3], dtype=float)))
                    if due_mon:
                        self.monitor_last_t = tt
                        self.monitor.update(tt, np.array(pts[:, :3], dtype=float))
        if self.calibrating and self.invalid is None:
            frame = []
            for i, row in enumerate(bb_rows):
                if row is None or any(math.isnan(v) for v in row):
                    continue
                p = np.array(row)
                self.calib_data[i].append(p)
                self.calib_times[i].append(stamp_ns * 1e-9 if stamp_ns > 0 else None)
                frame.append(p)
            if frame and stamp_ns > 0 and stamp_ns != self.last_frame_ns:
                self.last_frame_ns = stamp_ns
                self.frames.append((stamp_ns * 1e-9, np.array(frame)))

    # -- finalise ----------------------------------------------------------
    def _blank_row(self, t_end):
        return {'t_start': self.t_start, 't_start_iso': iso(self.t_start),
                'duration_s': t_end - self.t_start, 'n_frames': len(self.frames),
                'n_stamped': len(self.stamped), 'bb_moved_applied': False}

    def _kin_fields(self, row, samples):
        k = sweep_kinematics(samples)
        if k:
            row['sweep_start_deg'], row['sweep_end_deg'] = k['start_deg'], k['end_deg']
        row['n_direction_changes'] = direction_changes(samples)

    def _row_refused(self, t_end, text, cls_text):
        row = self._blank_row(t_end)
        src = self.stamped if len(self.stamped) >= MIN_STAMPED_YAW_SAMPLES else self.yaw_samples
        self._kin_fields(row, src)
        cls = refusal_class(text) if ':' in text else text
        row.update(verdict_current=cls, verdict_current_text=text,
                   verdict_intrinsic=cls, yaw_source='n/a')
        self.rows.append(row)

    def _finalize(self, t_end):
        a = self.args
        row = self._blank_row(t_end)
        stamped_ok = len(self.stamped) >= MIN_STAMPED_YAW_SAMPLES
        src = self.stamped if stamped_ok else self.yaw_samples
        self._kin_fields(row, src)
        gate_before = copy.deepcopy(self.gate.dump())
        cap = None if (self.capture_idx and len(self.rows) + 1 != self.capture_idx) else {'marker_frames': [(t, r4(p)) for t, p in self.frames],
               'stamped_yaw': [list(s) for s in self.stamped],
               'heartbeat_yaw': [list(s) for s in self.yaw_samples],
               'yaw_readings_deg': list(self.yaw_readings),
               'calibration_data': {str(i): [r4(p) for p in v] for i, v in self.calib_data.items()},
               'base_points': [(t, r4(p)) for t, p in self.base_points],
               'params': {'bb_yaw_source': a.bb_yaw_source, 'bb_pose_source': a.bb_pose_source,
                          'bb_base_min_sweeps': DEFAULT_MIN_POOLED_SWEEPS,
                          'pitch_z_offset_mm': hw.BB_GEOM_PITCH_Z_OFFSET_MM,
                          'bb_moved_armed': self.bb_moved_armed},
               'state_before': gate_before}
        if not self.keep_capture:
            cap = None

        total = sum(len(v) for v in self.calib_data.values())
        if total == 0:
            return self._finish(row, cap, 'No marker data collected', 'No marker data collected')
        requested = a.bb_yaw_source
        if requested == 'stamped' and not stamped_ok:
            return self._finish(row, cap, 'BB_YAW_SOURCE_UNAVAILABLE: too few stamped samples',
                                'BB_YAW_SOURCE_UNAVAILABLE')
        if requested == 'stamped' or (requested == 'auto' and stamped_ok):
            yaw_samples, yaw_source = self.stamped, 'stamped'
        else:
            yaw_samples, yaw_source = self.yaw_samples, 'heartbeat'
        row['yaw_source'] = yaw_source
        row['clip_yaw_deg'] = None if a.clip is None else '%g:%g' % a.clip
        calib_data, yaw_readings, frames = self.calib_data, self.yaw_readings, self.frames
        if a.clip is not None:
            lo, hi = a.clip
            ys = sorted(yaw_samples)
            yt = np.array([x[0] for x in ys])
            yv = np.array([x[1] for x in ys])

            def inside(t):
                return t is None or lo <= float(np.interp(t, yt, yv)) <= hi
            frames = [(t, p) for t, p in self.frames if inside(t)]
            calib_data = {i: [p for p, t in zip(v, self.calib_times[i]) if inside(t)]
                          for i, v in self.calib_data.items()}
            yaw_samples = [x for x in yaw_samples if lo <= x[1] <= hi]
            yaw_readings = [y for y in self.yaw_readings if lo <= y <= hi]
            self.base_points = [(t, p) for t, p in self.base_points if inside(t)]
            row['n_frames_clip_in'] = len(frames)
        try:
            result = run_calibration(
                calibration_data=calib_data, yaw_readings_deg=yaw_readings,
                pitch_z_offset_mm=hw.BB_GEOM_PITCH_Z_OFFSET_MM, marker_frames=frames,
                yaw_samples=yaw_samples, template=self.template, yaw_source=yaw_source)
        except ValueError as e:
            return self._finish(row, cap, str(e), refusal_class(str(e)))
        if result.yaw_method != 'constellation':
            return self._finish(row, cap, 'internal: yaw offset not constellation', 'internal')

        est = result.yaw_estimate
        arcs = [m.arc_span_deg for m in result.marker_metrics.values() if m.status == 'ok']
        row.update(
            yaw_offset_deg=math.degrees(result.yaw_offset_rad), sigma_deg=result.yaw_offset_std_deg,
            lag_ms=est.lag_s * 1e3, template_residual_mm=est.template_residual_mm,
            template_residual_window_mm=est.template_residual_window_mm,
            mocap_clock_pct=(None if est.frame_clock_off_grid is None
                             else est.frame_clock_off_grid * 100.0),
            n_frames=est.n_frames, n_moving=est.n_frames_moving, n_rejected=est.n_frames_rejected,
            axis_point_mm=[float(v) for v in result.bb_position_mm],
            arc_position_mm=(None if result.arc_position_mm is None
                             else [float(v) for v in result.arc_position_mm]),
            axis_tilt_deg=result.axis_tilt_deg,
            arc_span_deg=(float(np.median(arcs)) if arcs else None),
            sweep_end_deg=est.sweep_end_deg, sweep_start_deg=est.sweep_start_deg,
            verdict_intrinsic='ACCEPTED')

        bb_moved = self.bb_moved_armed
        row['bb_moved_applied'] = bool(bb_moved)
        view = self._base_frame_view(result)
        row['base_seen'] = view['pose'] is not None
        if view['pose'] is not None:
            bp = view['pose']
            row.update(base_origin_mm=[float(v) for v in bp.origin_mm], base_heading_deg=bp.heading_deg,
                       base_tilt_deg=bp.tilt_deg, base_residual_mm=bp.residual_mm,
                       base_n_frames=bp.n_frames, kappa_deg=view['kappa'],
                       p_b_mm=[float(v) for v in view['p_b']])
        row['pool_n_before'] = None if view['pooled'] is None else view['pooled'].n
        if view['pooled'] is not None:
            row['pooled_kappa_deg'], row['pooled_se_deg'] = view['pooled'].kappa_deg, view['pooled'].se_deg
        pose_source = self._resolve_pose_source(a.bb_pose_source, view)
        row['pose_source'] = pose_source
        yaw_deg = math.degrees(result.yaw_offset_rad)

        # advisory (history) distances, computed whichever gate decides
        if view['pose'] is not None and view['pooled'] is not None:
            row['advisory_dkappa_deg'] = float(
                wrap_deg(view['kappa'] - view['pooled'].kappa_deg))
        wref = self.gate.world_ref
        if wref:
            row['advisory_dyaw_deg'] = float(wrap_deg(yaw_deg - wref['yaw_offset_deg']))
            row['advisory_daxis_mm'] = float(np.linalg.norm(
                np.asarray(result.bb_position_mm) - np.asarray(wref['position_mm'])))

        if pose_source == 'base_frame':
            bv = check_base_frame_consistency(
                view['kappa'], kappa_sweep_sd_deg(est.yaw_source), view['p_b'], view['pose'],
                view['pooled'], self.gate.base_ref, bb_moved=bb_moved)
            if not bv.accepted:
                return self._finish(row, cap, bv.message, refusal_class(bv.message))
            if bv.overridden:
                self.bb_moved_armed = False
            records = self._pool_records(view, result, bv.reset_pool, t_end)
            pooled = pool_kappa(records, self.template.pinned_yaw_offset_deg,
                                self.template.repeatability_deg)
            yaw_b, pos_b = compose_bb_pose(view['pose'], pooled.kappa_deg, pooled.axis_point_b_mm)
            sigma_b = math.hypot(pooled.se_deg, view['pose'].heading_se_deg)
            self._persist_base(records, view['pose'], t_end)
            self._persist_world(yaw_b, sigma_b, pos_b, t_end)
            return self._finish(row, cap, 'ACCEPTED ' + bv.message, 'ACCEPTED')
        # world / sweep path
        verdict = check_calibration_consistency(
            yaw_deg, result.yaw_offset_std_deg, result.bb_position_mm, est.template_residual_mm,
            self._world_reference(), bb_moved=bb_moved,
            reference_label='accepted ' + str((wref or {}).get('accepted_at', '?'))[:19])
        bverdict = None
        if view['pose'] is not None:
            bverdict = check_base_frame_consistency(
                view['kappa'], kappa_sweep_sd_deg(est.yaw_source), view['p_b'], view['pose'],
                view['pooled'], self.gate.base_ref, bb_moved=bb_moved and verdict.accepted)
        if not verdict.accepted:
            msg = verdict.message
            if msg.startswith('CALIBRATION_INCONSISTENT') and self._base_expected(a.bb_pose_source, view):
                msg = (f'{view["error"]} — check QTM sees the 4 shelf markers; world gate also '
                       f'refused: Δyaw {verdict.delta_deg:+.2f}°')
            return self._finish(row, cap, msg, refusal_class(msg))
        if self.bb_moved_armed and verdict.overridden:
            self.bb_moved_armed = False
        if bverdict is not None and bverdict.accepted:
            self._persist_base(self._pool_records(view, result, bverdict.reset_pool, t_end),
                               view['pose'], t_end)
        self._persist_world(yaw_deg, result.yaw_offset_std_deg, result.bb_position_mm, t_end)
        return self._finish(row, cap, 'ACCEPTED ' + verdict.message, 'ACCEPTED')

    def _finish(self, row, cap, text, cls):
        row['verdict_current'], row['verdict_current_text'] = cls, text
        row.setdefault('verdict_intrinsic', cls)
        row.setdefault('yaw_source', 'n/a')
        # an intrinsic refusal keeps its class; a history refusal is intrinsically ACCEPTED
        if cls != 'ACCEPTED' and 'template_residual_mm' not in row:
            row['verdict_intrinsic'] = cls
        if cap is not None:
            self.capture = {'inputs': cap, 'row': row}
        self.rows.append(row)

    # -- base-frame helpers (mirrors of the node's private methods) ---------
    def _base_frame_view(self, result):
        view = {'pose': None, 'error': '', 'kappa': None, 'p_b': None, 'pooled': None,
                'from_monitor': False}
        T = self.base_model.template_mm
        try:
            view['pose'] = estimate_base_pose(self.base_points, T)
        except ValueError as e:
            view['error'] = str(e)
            last = self.monitor_last_t
            pose = self.monitor.pose(last)
            if pose is not None and last is not None and self.base_points and \
                    self.base_points[-1][0] - pose.t_span_s[1] <= BASE_MONITOR_MAX_AGE_S:
                view['pose'], view['from_monitor'] = pose, True
        tpl = self.template
        view['pooled'] = pool_kappa(self.gate.records, tpl.pinned_yaw_offset_deg, tpl.repeatability_deg)
        if view['pose'] is not None:
            view['kappa'], view['p_b'] = bb_in_base(
                view['pose'], math.degrees(result.yaw_offset_rad), result.bb_position_mm)
        return view

    def _resolve_pose_source(self, requested, view):
        if requested == 'sweep':
            return 'sweep'
        if requested == 'base_frame':
            return 'base_frame' if view['pose'] is not None else 'sweep'
        n = 0 if view['pooled'] is None else view['pooled'].n
        if view['pose'] is None or view['pose'].residual_mm > BASE_MAX_RESIDUAL_MM \
                or n < DEFAULT_MIN_POOLED_SWEEPS:
            return 'sweep'
        return 'base_frame'

    def _base_expected(self, requested, view):
        if requested not in ('auto', 'base_frame'):
            return False
        if view['pose'] is not None or not view['error'].startswith('BASE_FRAME_NOT_SEEN'):
            return False
        n = 0 if view['pooled'] is None else view['pooled'].n
        return n >= DEFAULT_MIN_POOLED_SWEEPS

    def _pool_records(self, view, result, reset, t_end):
        rec = KappaRecord(
            kappa_deg=float(view['kappa']), sigma_deg=float(result.yaw_offset_std_deg),
            axis_point_b_mm=tuple(float(v) for v in view['p_b']),
            pin_deg=float(self.template.pinned_yaw_offset_deg), accepted_at=iso(t_end),
            yaw_offset_deg=math.degrees(result.yaw_offset_rad),
            base_heading_deg=float(view['pose'].heading_deg),
            source=f'mocap_node ({result.yaw_estimate.yaw_source})')
        return ([] if reset else list(self.gate.records)) + [rec]

    def _persist_base(self, records, pose, t_end):
        self.gate.records = list(records)[-MAX_POOLED_RECORDS:]
        self.gate.base_ref = {'origin_mm': [float(v) for v in pose.origin_mm],
                              'heading_deg': float(pose.heading_deg), 'accepted_at': iso(t_end)}

    def _persist_world(self, yaw_deg, std, pos, t_end):
        self.gate.world_ref = {'yaw_offset_deg': float(yaw_deg), 'yaw_offset_std_deg': float(std),
                               'position_mm': [float(v) for v in pos], 'accepted_at': iso(t_end)}

    def _world_reference(self):
        w = self.gate.world_ref
        return None if not w else {k: w[k] for k in ('yaw_offset_deg', 'yaw_offset_std_deg', 'position_mm')}


# ───────────────────────────── small helpers ─────────────────────────────

def r4(a):
    """Coordinates rounded to 1e-4 mm (QTM data is float32: its own resolution at 1 m is 6e-5 mm)."""
    return np.round(np.asarray(a, dtype=float), 4).tolist()


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def iso(t):
    return datetime.datetime.fromtimestamp(t, datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')


def direction_changes(samples, min_speed=5.0, win=0.25):
    """Sign changes of the yaw velocity (deg/s over ~win s), ignoring |v| < min_speed."""
    s = sorted((float(t), float(v)) for t, v in samples if math.isfinite(v))
    if len(s) < 5:
        return 0
    t = np.array([x[0] for x in s])
    y = np.unwrap(np.radians([x[1] for x in s]))
    y = np.degrees(y)
    j = np.searchsorted(t, t + win)
    ok = j < len(t)
    v = np.zeros(len(t))
    v[ok] = (y[j[ok]] - y[ok]) / np.maximum(t[j[ok]] - t[ok], 1e-6)
    sg = np.sign(v[np.abs(v) >= min_speed])
    return int(np.count_nonzero(np.diff(sg) != 0)) if len(sg) > 1 else 0


def jsonable(o):
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, KappaRecord):
        return o.to_dict()
    raise TypeError(type(o))


# ───────────────────────────── driver ─────────────────────────────

def replay_bag(bag_dir, args, template, base_model, gate, arms, want_capture, capture_idx=None):
    path = bag_mcap(bag_dir)
    if path is None:
        return [], None
    hb = read_heartbeats(path)
    if not hb:
        return [], None
    # sweep windows from the heartbeat state edges
    windows, start, last = [], None, None
    for t, st, _y in hb:
        if st == BallButlerStates.CALIBRATING and start is None:
            start = t
        if start is not None and st != BallButlerStates.CALIBRATING:
            windows.append((start - MONITOR_PRE_S, t + 1.0))
            start = None
        last = t
    if start is not None:
        windows.append((start - MONITOR_PRE_S, last + 1.0))
    merged = []
    for w0, w1 in sorted(windows):          # overlapping windows would decode (and feed) a message twice
        if merged and w0 <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], w1)
        else:
            merged.append([w0, w1])
    ev = read_events(path, merged) if merged else iter(())
    t0, t1 = hb[0][0], hb[-1][0]
    bag_arms = [a for a in arms if t0 - 5 <= a <= t1 + 5]
    sim = NodeSim(args, template, base_model, gate, bag_arms)
    sim.keep_capture = want_capture
    sim.capture_idx = capture_idx
    # merge heartbeats and window events by log_time
    import heapq
    hb_stream = ((int(t * 1e9 + 0.5), 'hb', (st, y)) for t, st, y in hb)
    stream = heapq.merge(hb_stream, ev, key=lambda e: e[0])
    for lt, kind, d in stream:
        t = lt * 1e-9
        sim.check_timeout(t)
        if kind == 'hb':
            sim.heartbeat(t, d[0], d[1])
        elif kind == '/mocap_data':
            sim.mocap(t, d)
        else:
            sim.axis(d)
    for i, r in enumerate(sim.rows):
        r['sweep_idx'] = i + 1
        r['bag'] = os.path.basename(bag_dir)
    return sim.rows, sim.capture if want_capture else None


def match_logged(rows, logged):
    """Pair each replayed row with the nearest logged sweep (<= 4 s), one-to-one."""
    used = set()
    for r in sorted(rows, key=lambda r: r['t_start']):
        best = None
        for i, L in enumerate(logged):
            if i in used:
                continue
            dt = abs(L['t_start'] - r['t_start'])
            if dt <= 4.0 and (best is None or dt < best[0]):
                best = (dt, i)
        if best:
            used.add(best[1])
            r['logged'] = logged[best[1]]
    return [L for i, L in enumerate(logged) if i not in used]


def compare(r):
    """List of (field, logged, replayed, delta, ok) for a matched row + verdict match."""
    L = r.get('logged')
    out = []
    if L is None:
        return out
    pairs = [('yaw_offset_deg', 'yaw_offset_deg'), ('kappa_deg', 'kappa_deg'),
             ('template_residual_mm', 'template_residual_mm'), ('yaw_lag_ms', 'lag_ms'),
             ('sigma_deg', 'sigma_deg')]
    for lf, rf in pairs:
        lv, rv = L.get(lf), r.get(rf)
        if lv is None:
            continue            # not logged for this sweep (refused before / without the field)
        if rv is None:
            out.append((lf, lv, None, None, False))
            continue
        d = rv - lv
        out.append((lf, lv, rv, d, abs(d) <= TOL[lf] + 1e-9))
    return out


def fmt(v, p=4):
    return 'n/a' if v is None else (f'{v:.{p}f}' if isinstance(v, float) else str(v))


def write_outputs(outdir, rows, unmatched, args, elapsed):
    os.makedirs(outdir, exist_ok=True)
    cols = ['bag', 'sweep_idx', 't_start', 't_start_iso', 'duration_s', 'n_frames', 'n_moving',
            'n_rejected', 'yaw_source', 'n_stamped', 'yaw_offset_deg', 'sigma_deg', 'lag_ms',
            'template_residual_mm', 'template_residual_window_mm', 'mocap_clock_pct',
            'axis_point_mm', 'arc_position_mm', 'axis_tilt_deg', 'arc_span_deg', 'sweep_end_deg',
            'sweep_start_deg', 'n_direction_changes', 'base_seen', 'base_origin_mm',
            'base_heading_deg', 'base_tilt_deg', 'base_residual_mm', 'base_n_frames', 'kappa_deg',
            'p_b_mm', 'pool_n_before', 'pooled_kappa_deg', 'pooled_se_deg', 'pose_source',
            'verdict_current', 'verdict_current_text', 'verdict_intrinsic', 'advisory_dkappa_deg',
            'advisory_dyaw_deg', 'advisory_daxis_mm', 'bb_moved_applied', 'logged_outcome',
            'logged_class', 'logged_pool_n', 'match_measure', 'match_verdict', 'clip_yaw_deg', 'n_frames_clip_in']
    for r in rows:
        L = r.get('logged')
        cmp_ = compare(r)
        r['logged_outcome'] = None if L is None else L.get('outcome')
        r['logged_class'] = None if L is None else L.get('class')
        r['logged_pool_n'] = None if L is None else L.get('gate_pooled_n')
        r['match_measure'] = None if L is None else all(c[4] for c in cmp_)
        r['match_verdict'] = None if L is None else (r['verdict_current'] == L.get('class'))
    with open(os.path.join(outdir, 'sweeps.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for r in rows:
            w.writerow([json.dumps(r.get(c), default=jsonable) if isinstance(r.get(c), (list, dict))
                        else r.get(c) for c in cols])
    with open(os.path.join(outdir, 'sweeps.json'), 'w') as fh:
        json.dump([dict(r) for r in rows], fh, indent=1, default=jsonable)

    # fidelity
    scored = [r for r in rows if r.get('logged') is not None]
    n_meas = n_meas_ok = n_ver_ok = 0
    lines = ['# Fidelity: replay vs the node\'s logged values', '',
             'Tolerances: ' + ', '.join(f'{k} {v}' for k, v in TOL.items()), '',
             '| sweep | field | logged | replayed | delta | match |', '|---|---|---|---|---|---|']
    mism = []
    for r in scored:
        name = f'{r["bag"]}#{r["sweep_idx"]}'
        for lf, lv, rv, d, ok in compare(r):
            n_meas += 1
            n_meas_ok += ok
            lines.append(f'| {name} | {lf} | {fmt(lv)} | {fmt(rv)} | {fmt(d)} | {"ok" if ok else "MISMATCH"} |')
            if not ok:
                mism.append((name, lf, lv, rv, d))
        L = r['logged']
        vok = r['verdict_current'] == L.get('class')
        n_ver_ok += vok
        lines.append(f'| {name} | verdict | {L.get("class")} | {r["verdict_current"]} | | '
                     f'{"ok" if vok else "MISMATCH"} |')
        if not vok:
            mism.append((name, 'verdict', L.get('class'), r['verdict_current'], None))
        if L.get('gate_pooled_n') is not None:
            pn = r.get('pool_n_before')
            lines.append(f'| {name} | pool n (info) | {L["gate_pooled_n"]} | {pn} | | '
                         f'{"ok" if pn == L["gate_pooled_n"] else "differs"} |')
    n_sw = len(scored)
    sweeps_all_ok = sum(1 for r in scored if r['match_measure'])
    score = (f'{n_meas_ok} of {n_meas} measurements match, {n_ver_ok} of {n_sw} verdicts match '
             f'({sweeps_all_ok} of {n_sw} sweeps match on every measurement; '
             f'{len(unmatched)} logged sweeps have no bag)')
    lines += ['', '**' + score + '**', '', '## Mismatches', '']
    lines += [f'- {n}: {f} logged {fmt(lv)} replayed {fmt(rv)} delta {fmt(d)}' for n, f, lv, rv, d in mism] or ['- none']
    lines += ['', '## Logged sweeps with no replayed counterpart (no bag / unmatched in time)', '']
    lines += [f'- {L["launch"]}#{L["idx_in_launch"]} {iso(L["t_start"])} {L.get("class")}' for L in unmatched] or ['- none']
    open(os.path.join(outdir, 'fidelity.md'), 'w').write('\n'.join(lines) + '\n')

    # summary
    refused = [r for r in scored if r['logged'].get('class') != 'ACCEPTED']
    intr_acc = sum(1 for r in refused if r['verdict_intrinsic'] == 'ACCEPTED')
    S = ['# BB calibration replay summary', '', f'Run {iso(time.time())}, {elapsed:.0f} s; '
         f'args: yaw_source={args.bb_yaw_source} pose_source={args.bb_pose_source} '
         f'bb_moved={"off" if args.no_bb_moved else "on"} clip_yaw_deg={args.clip_yaw_deg}', '', '**' + score + '**', '',
         '## Per bag: starts / accepted / refused (current) / accepted (intrinsic)', '',
         '| bag | starts | current ACC | current REF | intrinsic ACC |', '|---|---|---|---|---|']
    bags = sorted({r['bag'] for r in rows})
    for b in bags:
        rs = [r for r in rows if r['bag'] == b]
        S.append(f'| {b} | {len(rs)} | {sum(r["verdict_current"] == "ACCEPTED" for r in rs)} | '
                 f'{sum(r["verdict_current"] != "ACCEPTED" for r in rs)} | '
                 f'{sum(r["verdict_intrinsic"] == "ACCEPTED" for r in rs)} |')
    S += ['', f'**The {len(refused)} logged-refused sweeps under verdict_intrinsic: {intr_acc} accepted '
          f'({len(refused) - intr_acc} still refused).**', '',
          '| sweep | logged class | intrinsic | note |', '|---|---|---|---|']
    for r in refused:
        S.append(f'| {r["bag"]}#{r["sweep_idx"]} | {r["logged"].get("class")} | {r["verdict_intrinsic"]} | '
                 f'{(r["verdict_current_text"] or "")[:90]} |')
    # sweep_end per launch around 10-13 22:00 (AEDT, UTC+11)
    cut = datetime.datetime(2026, 10, 13, 22, 0, tzinfo=datetime.timezone(datetime.timedelta(hours=11))).timestamp()
    S += ['', '## sweep_end_deg (max yaw reached) per bag, before/after 2026-10-13 22:00 AEDT', '',
          '| bag | n | mean | min | max | side |', '|---|---|---|---|---|---|']
    for b in bags:
        v = [r['sweep_end_deg'] for r in rows if r['bag'] == b and r.get('sweep_end_deg') is not None
             and abs(r['sweep_end_deg']) > 30]
        if v:
            side = 'after' if min(r['t_start'] for r in rows if r['bag'] == b) >= cut else 'before'
            S.append(f'| {b} | {len(v)} | {np.mean(v):.2f} | {min(v):.2f} | {max(v):.2f} | {side} |')
    for side, sel in (('before', lambda r: r['t_start'] < cut), ('after', lambda r: r['t_start'] >= cut)):
        v = [r['sweep_end_deg'] for r in rows if sel(r) and r.get('sweep_end_deg') is not None
             and abs(r['sweep_end_deg']) > 30]
        if v:
            S.append(f'| ALL {side} | {len(v)} | {np.mean(v):.2f} | {min(v):.2f} | {max(v):.2f} | {side} |')
    open(os.path.join(outdir, 'summary.md'), 'w').write('\n'.join(S) + '\n')
    return score, intr_acc, len(refused)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bags', nargs='+', default=[os.path.join(BAG_ROOT, '2026-10-1[0-4]*')],
                    help='one or more bag-dir globs')
    ap.add_argument('--clip-yaw-deg', metavar='LO:HI',
                    help='experiment: solve only the part of each sweep whose BB yaw is in [LO, HI]')
    ap.add_argument('--sweep', help='BAG:IDX to replay one sweep (state is built from prior sweeps of that bag only)')
    ap.add_argument('--ros-log-dir', default=os.path.expanduser('~/.ros/log'))
    ap.add_argument('--no-bb-moved', action='store_true')
    ap.add_argument('--state-json', help='snapshot {"base_state": {...}, "accepted": {...}} to start from')
    ap.add_argument('--bb-yaw-source', default='auto', choices=('auto', 'stamped', 'heartbeat'))
    ap.add_argument('--bb-pose-source', default='auto', choices=('auto', 'sweep', 'base_frame'))
    ap.add_argument('--emit-fixture', metavar='BAG:IDX')
    ap.add_argument('--out', help='fixture output path (with --emit-fixture)')
    ap.add_argument('-v', '--verbose', action='store_true')
    args = ap.parse_args()

    t_run = time.time()
    template = load_marker_template(os.path.join(RES_DIR, 'bb_marker_template.json'))
    base_model = load_base_frame(os.path.join(RES_DIR, 'bb_base_frame.json'))
    snap = json.load(open(args.state_json)) if args.state_json else None
    gate = GateState(base_model, snap)
    logged, arms = parse_launch_logs(args.ros_log_dir)
    if args.no_bb_moved:
        arms = []
    want = (args.emit_fixture.rsplit(':', 1) if args.emit_fixture else None)
    only = (args.sweep.rsplit(':', 1) if args.sweep else None)

    bag_dirs = sorted({d for g in args.bags for d in glob.glob(g) if os.path.isdir(d)})
    args.clip = tuple(float(x) for x in args.clip_yaw_deg.split(':')) if args.clip_yaw_deg else None
    rows, fixture = [], None
    for bd in bag_dirs:
        name = os.path.basename(bd)
        cap_this = bool(want and want[0] == name)
        t0 = time.time()
        r, cap = replay_bag(bd, args, template, base_model, gate, arms, cap_this,
                            int(want[1]) if want else None)
        rows += r
        print(f'{name}: {len(r)} sweeps ({time.time() - t0:.0f} s)', flush=True)
        if cap and want:
            idx = int(want[1])
            fixture = cap
        if args.verbose:
            for x in r:
                print(f'  #{x["sweep_idx"]} {x["t_start_iso"]} {x["verdict_current"]} '
                      f'yaw {fmt(x.get("yaw_offset_deg"))} kappa {fmt(x.get("kappa_deg"))} '
                      f'res {fmt(x.get("template_residual_mm"), 2)} end {fmt(x.get("sweep_end_deg"), 1)} '
                      f'n_dir {x.get("n_direction_changes")} | {x["verdict_current_text"][:100]}')
    unmatched = match_logged(rows, logged)
    if only:
        rows = [r for r in rows if r['bag'] == only[0] and r['sweep_idx'] == int(only[1])]
    outdir = os.path.join(REPO, 'temp', 'probes', 'bb_calibration_replay',
                          datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    score, intr, nref = write_outputs(outdir, rows, unmatched, args, time.time() - t_run)
    print(f'SUMMARY: {len(rows)} sweeps replayed from {len(bag_dirs)} bags; {score}; '
          f'logged-refused under intrinsic verdict: {intr} of {nref} accepted; outputs {outdir}')
    if want:
        if fixture is None:
            print('fixture: sweep not found', file=sys.stderr)
            return 2
        out = args.out or os.path.join(outdir, 'fixture.json')
        fixture = dict(fixture)
        fixture['recipe'] = ('python tools/probes/bb_calibration_replay.py --bags '
                             '"$HOME/Desktop/rosbags/%s" --emit-fixture %s:%s --out %s'
                             % (want[0], want[0], want[1], args.out or 'tools/probes/data/bb_calibration_replay_fixture.json'))
        with open(out, 'w') as fh:
            json.dump(fixture, fh, default=jsonable, separators=(',', ':'))
        print(f'fixture: {out} {os.path.getsize(out) / 1e6:.2f} MB')
    return 0


if __name__ == '__main__':
    sys.exit(main())
