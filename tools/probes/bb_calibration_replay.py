#!/usr/bin/env python3
"""Production-faithful offline replay of every BB (Ball Butler) calibration sweep in the bags.

What it replays
    Each operator yaw sweep (heartbeat state CALIBRATING -> not CALIBRATING) found in
    ~/Desktop/rosbags/2026-10-1[0-6]_*, through the SAME functions ``mocap_node`` calls, with
    the SAME inputs rebuilt from the bag (/mocap_data, /bb/heartbeat, /bb/axis_estimates):
    ``bb_calibration.run_calibration`` (template fit, lag fit, axis fit, yaw offset), then the
    Phase 1 path in the node's order: ``bb_base_frame.estimate_base_pose`` / ``bb_in_base`` /
    ``assess_base_frame`` / ``append_record``, ``bb_calibration.assess_sweep_vs_reference`` and
    ``format_outcome_body``. The node's state (the schema-2 kappa-epoch state of
    bb_base_frame_state.json and the last-accepted reference) is simulated in MEMORY,
    chronologically across bags; nothing under ~/bb_calibration_sessions is read or written.

    Phase 1 never refuses on history: ``verdict_p1`` is ACCEPTED or the solver's own refusal
    class (it equals ``verdict_intrinsic`` by construction; both columns are kept so the table
    reads). The history distances are ADVISORIES: ``advisory_flags`` (';'-joined),
    ``epoch_id`` / ``epoch_opened`` / ``epoch_rejoined``, ``delta_kappa_deg``,
    ``delta_yaw_deg``, ``same_world_epoch`` and the node's ``outcome_line``. The Phase 0 replay
    simulated the 2026-10-11..15 gates (``verdict_current``, retired); that fidelity result (38/39
    verdicts, 185/185 measurements) is recorded in the Phase 0 logbook entry and the gates no
    longer exist in the library.

Phase 2 columns (the 2026-10-16 controlled-pair measurement, now standing; sweeps.csv / summary.md)
    Labelling: ``label_frac`` (7: fraction of window frames in which QTM labelled 'Ball Butler - i'),
    ``rest_n_frames`` / ``rest_label_frac`` / ``rest_pair_err_max_mm`` / ``rest_unlab_near_axis_per_frame`` (the
    15 s before the sweep). Fit: ``marker_vis`` / ``marker_rms_mm`` / ``marker_body_mean_mm`` (7, 7, 7x3),
    ``worst_pair`` / ``worst_pair_mm`` (the solver's template residual, by pair). Label-free pickup:
    ``yaw_labelfree_deg`` / ``kappa_labelfree_deg`` / ``template_residual_labelfree_mm`` / ``labelfree_matched`` /
    ``labelfree_marker_vis`` / ``labelfree_unlab_per_frame`` / ``labelfree_n_unlab`` / ``labelfree_n_within_3mm`` /
    ``labelfree_p05_min_dist_mm`` (one extra solve per sweep on EVERY marker, labelled or not, within
    LABELFREE_RADIUS_MM of the fitted axis). Direction split: ``yaw_out_deg`` / ``yaw_ret_deg`` /
    ``kappa_out_deg`` / ``kappa_ret_deg``. Shelf: ``base_pair_err_mm`` (C-M, C-E, C-S, M-E, M-S, E-S). Opt-in
    flags: ``--subset 1,2,4,6,7`` (``subset``, ``yaw_subset_deg``, ``kappa_subset_deg``,
    ``template_residual_subset_mm``) and ``--loo`` (``yaw_loo_deg`` / ``kappa_loo_deg``, 7 refits per sweep). A
    refused variant stores its refusal class in the yaw column. The label-free pickup is a standing NULL check:
    the 2026-10-16 measurement found nothing to pick up (0 of ~80k unlabelled near-axis points were BB markers),
    and a non-zero ``labelfree_n_within_3mm`` is what would change that.

Fidelity contract
    For the sweeps the node logged (~/.ros/log/<launch>/launch.log), the replayed yaw offset,
    kappa, template residual, lag and sigma are compared with the logged values (tolerances in
    TOL; ``match_measure``). A mismatch is reported, never tuned away. ``logged_class`` is the
    OLD node's verdict, kept as the historical column the Phase 1 table is read against.

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
    * the last-accepted reference and the epoch state start from the bb_base_frame.json seed
      records, not from the node's files on the day (the persisted files are not in the bags);
    * accepted_at stamps are the sweep's end time, not datetime.now() (not read by any gate).

Usage
    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/probes/bb_calibration_replay.py [-v] [--bags GLOB] [--sweep BAG:IDX]
        [--bb-yaw-source auto|stamped|heartbeat]
        [--ros-log-dir DIR] [--state-json PATH]
        [--subset 1,2,4,6,7] [--loo] [--emit-fixture BAG:IDX --out PATH]
"""
from __future__ import annotations

import argparse
import copy
import csv
import dataclasses
import datetime
import itertools
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
    run_calibration, load_marker_template, assess_sweep_vs_reference, format_outcome_body,
    stamped_yaw_samples_from_joint_state, BB_MARKER_COUNT, sweep_kinematics,
    SWEEP_MIN_SPEED_DEG_S, canonical_yaw_deg, estimate_sweep_yaw_offset, track_constellation,
)
from jugglebot.bb_base_frame import (  # noqa: E402
    BaseFrameMonitor, KappaRecord, append_record, assess_base_frame,
    base_candidate_points, bb_in_base, current_epoch_id, estimate_base_pose,
    kappa_sweep_sd_deg, load_base_frame, seed_base_state, track_base_frame,
)

#: Node constants mirrored from mocap_node.py (asserted against the module text in --selfcheck).
MIN_STAMPED_YAW_SAMPLES = 100
CALIBRATION_TIMEOUT_S = 60.0
BASE_MONITOR_PERIOD_S = 0.2
BASE_MONITOR_MAX_AGE_S = 30.0
MONITOR_PRE_S = 40.0
#: Phase 2: rest-pose look-back before a sweep (s) and the label-free pickup radius about the fitted axis (mm).
REST_SPAN_S = 15.0
LABELFREE_RADIUS_MM = 250.0

#: Fidelity tolerances (replay brief).
TOL = {'yaw_offset_deg': 0.002, 'kappa_deg': 0.002, 'template_residual_mm': 0.005,
       'yaw_lag_ms': 0.5, 'sigma_deg': 0.002}

#: Phase 2 columns (see the module docstring).
PHASE2_COLS = [
    'label_frac', 'marker_vis', 'marker_rms_mm', 'marker_body_mean_mm', 'worst_pair', 'worst_pair_mm',
    'yaw_labelfree_deg', 'template_residual_labelfree_mm', 'labelfree_matched', 'labelfree_marker_vis',
    'labelfree_unlab_per_frame', 'labelfree_n_unlab', 'labelfree_n_within_3mm', 'labelfree_p05_min_dist_mm',
    'yaw_out_deg', 'yaw_ret_deg', 'rest_n_frames', 'rest_label_frac', 'rest_pair_err_max_mm',
    'rest_unlab_near_axis_per_frame', 'subset', 'yaw_subset_deg', 'template_residual_subset_mm', 'yaw_loo_deg',
    'kappa_labelfree_deg', 'kappa_out_deg', 'kappa_ret_deg', 'kappa_subset_deg', 'kappa_loo_deg',
    'base_pair_err_mm']

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
    """The node's logged sweeps (2026-10-11..16 launches; the 10-16 launches, 15:07:58 Phase 1 code and 16:10:37,
    log the same 'Yaw offset (sweep)' DEBUG line).
    Launches 2026-10-11..16 only (the 10-10 launches ran older code: scored mismatches there are version, not
    fidelity). A sweep: t_start, outcome, refusal_class, and the numbers of its 'Yaw offset' DEBUG line."""
    sweeps = []
    for lp in sorted(glob.glob(os.path.join(ros_log_dir, '2026-10-1[1-6]-*', 'launch.log'))):
        ld = os.path.basename(os.path.dirname(lp))
        cur = None
        idx = 0
        for raw in open(lp, errors='replace'):
            m = _LINE.match(raw.rstrip('\n'))
            if not m or m.group(4) != 'mocap_node':
                continue
            t, msg = float(m.group(1)), m.group(5)
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
    return sweeps


# ───────────────────────────── node simulation ─────────────────────────────

class GateState:
    """The node's persisted state, in memory: the schema-2 kappa-epoch state (records, base_ref,
    epochs) and the last-accepted reference the world advisory compares with."""

    def __init__(self, base_model, snapshot=None):
        recs, ref = list(base_model.records), base_model.base_ref
        self.world_ref = None          # {'yaw_offset_deg', 'yaw_offset_std_deg', 'position_mm', 'accepted_at'}
        if snapshot:
            bs = snapshot.get('base_state')
            if bs:
                recs = [KappaRecord.from_dict(r) for r in bs['records']]
                ref = bs.get('base_ref') or ref
            self.world_ref = snapshot.get('accepted') or None
        self.state = seed_base_state(recs, ref)

    def dump(self):
        return {'records': [r.to_dict() for r in self.state['records']],
                'epochs': copy.deepcopy(self.state['epochs']),
                'base_ref': self.state['base_ref'], 'world_ref': self.world_ref}


class NodeSim:
    """One node process (one bag): the calibration window state machine and the finalise path."""

    def __init__(self, args, template, base_model, gate):
        self.args, self.template, self.base_model, self.gate = args, template, base_model, gate
        self.last_state = None
        self.calibrating = False
        self.blocked = False
        self.invalid = None
        self.t_start = None
        self.reset_window()
        self.monitor = BaseFrameMonitor(base_model.template_mm)
        self.monitor_last_t = None
        self.rows = []
        self.recent_all = []           # Phase 2: rolling REST_SPAN_S + 2 s of every marker of every frame
        self._last_all_ns = None
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
        self.window_all = []           # Phase 2: (stamp_s, labels, xyz) of EVERY marker, while calibrating
        self.rest_snapshot = []

    # -- events ------------------------------------------------------------
    def heartbeat(self, t, state, yaw):
        previous = self.last_state
        if state != self.last_state:
            self.last_state = state
        if state != BallButlerStates.CALIBRATING:
            self.blocked = False
        if state == BallButlerStates.CALIBRATING and not self.calibrating and not self.blocked:
            self.calibrating = True
            self.invalid = None
            self.t_start = t
            rest = list(self.recent_all)
            self.reset_window()
            self.rest_snapshot = rest
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
        if frame_ns is not None and frame_ns != self._last_all_ns:
            self._last_all_ns = frame_ns
            fr = (frame_ns * 1e-9, [mk.label or '' for mk in d.markers],
                  np.array([(mk.position.x, mk.position.y, mk.position.z) for mk in d.markers],
                           dtype=np.float32).reshape(-1, 3))
            self.recent_all.append(fr)
            lo = fr[0] - (REST_SPAN_S + 2.0)
            k = 0
            while k < len(self.recent_all) and self.recent_all[k][0] < lo:
                k += 1
            if k:
                del self.recent_all[:k]
            if self.calibrating and self.invalid is None:
                self.window_all.append(fr)
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
                'n_stamped': len(self.stamped)}

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
        row.update(verdict_p1=cls, verdict_text=text, verdict_intrinsic=cls, yaw_source='n/a')
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
               'params': {'bb_yaw_source': a.bb_yaw_source,
                          'pitch_z_offset_mm': hw.BB_GEOM_PITCH_Z_OFFSET_MM},
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
        self._solve_frames = frames
        self.phase2_yaw(row, result, yaw_samples, yaw_source, t_end)

        return self.phase1(row, cap, result, t_end)

    def phase1(self, row, cap, result, t_end):
        """The node's Phase 1 flow after a successful run_calibration, on the in-memory state:
        base-frame view -> assess_base_frame -> assess_sweep_vs_reference -> append_record ->
        format_outcome_body. Never refuses."""
        est = result.yaw_estimate
        yaw_deg = math.degrees(result.yaw_offset_rad)
        state = self.gate.state
        view = self._base_frame_view(result)
        pose = view['pose']
        row['base_seen'] = pose is not None
        if pose is not None:
            row.update(base_origin_mm=[float(v) for v in pose.origin_mm], base_heading_deg=pose.heading_deg,
                       base_tilt_deg=pose.tilt_deg, base_residual_mm=pose.residual_mm,
                       base_n_frames=pose.n_frames, kappa_deg=view['kappa'],
                       p_b_mm=[float(v) for v in view['p_b']])
            h = pose.heading_deg
            for col, src in (('kappa_labelfree_deg', 'yaw_labelfree_deg'), ('kappa_out_deg', 'yaw_out_deg'),
                             ('kappa_ret_deg', 'yaw_ret_deg'), ('kappa_subset_deg', 'yaw_subset_deg')):
                v = row.get(src)
                row[col] = wrap_deg(v - h) if isinstance(v, float) else None
            if 'yaw_loo_deg' in row:
                row['kappa_loo_deg'] = [wrap_deg(v - h) if isinstance(v, float) else None
                                        for v in row['yaw_loo_deg']]
            row['base_pair_err_mm'] = base_pair_errors(self.base_points, self.base_model.template_mm)
        cur = current_epoch_id(state)
        cur_e = next((e for e in state['epochs'] if e['id'] == cur), None)
        row['pool_n_before'] = None if cur_e is None else cur_e['n']
        row['pooled_kappa_deg'] = None if cur_e is None else cur_e['median_kappa_deg']
        row['pose_source'] = 'base_frame' if pose is not None else 'sweep'

        adv = None
        if pose is not None:
            adv = assess_base_frame(view['kappa'], kappa_sweep_sd_deg(est.yaw_source), view['p_b'],
                                    pose, state,
                                    float(self.template.pinned_yaw_offset_deg))
        same = None if adv is None else adv.same_world_epoch
        ref = self._world_reference()
        sw = assess_sweep_vs_reference(
            yaw_deg, result.yaw_offset_std_deg, result.bb_position_mm, est.template_residual_mm,
            est.lag_s * 1e3, ref, same, yaw_source=est.yaw_source)
        pooled_info = None
        if adv is not None and adv.pooled:
            rec = KappaRecord(
                kappa_deg=float(view['kappa']), sigma_deg=float(result.yaw_offset_std_deg),
                axis_point_b_mm=tuple(float(v) for v in view['p_b']),
                pin_deg=float(self.template.pinned_yaw_offset_deg), accepted_at=iso(t_end),
                yaw_offset_deg=yaw_deg, base_heading_deg=float(pose.heading_deg),
                source=f'mocap_node ({est.yaw_source})')
            new = append_record(state, rec, adv.assignment, float(self.template.pinned_yaw_offset_deg))
            new['base_ref'] = {'origin_mm': [float(v) for v in pose.origin_mm],
                               'heading_deg': float(pose.heading_deg), 'accepted_at': iso(t_end)}
            self.gate.state = new
            e = next(e for e in new['epochs'] if e['id'] == adv.assignment.epoch_id)
            pooled_info = {'kappa_deg': e['median_kappa_deg'], 'epoch_id': e['id'],
                           'delta_deg': adv.delta_kappa_deg, 'n': e['n'],
                           'opened': bool(adv.assignment.opened),
                           'rejoined': bool(adv.assignment.rejoined)}
            row.update(epoch_id=adv.assignment.epoch_id, epoch_opened=bool(adv.assignment.opened),
                       epoch_rejoined=bool(adv.assignment.rejoined))
        else:
            row.update(epoch_id=None, epoch_opened=False, epoch_rejoined=False)
        self._persist_world(yaw_deg, result.yaw_offset_std_deg, result.bb_position_mm, t_end)
        flags = list(sw.flags) + ([] if adv is None else list(adv.flags))
        row.update(delta_kappa_deg=None if adv is None else adv.delta_kappa_deg,
                   delta_axis_b_mm=None if adv is None else adv.delta_axis_mm,
                   delta_yaw_deg=sw.delta_deg, axis_shift_mm=sw.axis_shift_mm,
                   same_world_epoch=same, advisory_flags=';'.join(flags))
        row['outcome_line'] = format_outcome_body(
            yaw_deg, result.yaw_offset_std_deg, result.bb_position_mm, result.axis_tilt_deg,
            est.template_residual_mm, est.lag_s * 1e3, sw, adv, pooled_info)
        return self._finish(row, cap, 'ACCEPTED ' + row['outcome_line'], 'ACCEPTED')

    def _finish(self, row, cap, text, cls):
        row['verdict_p1'], row['verdict_text'] = cls, text
        row.setdefault('verdict_intrinsic', cls)
        row.setdefault('yaw_source', 'n/a')
        # an intrinsic refusal keeps its class; a history refusal is intrinsically ACCEPTED
        if cls != 'ACCEPTED' and 'template_residual_mm' not in row:
            row['verdict_intrinsic'] = cls
        if cap is not None:
            self.capture = {'inputs': cap, 'row': row}
        self.rows.append(row)

    # -- Phase 2 columns -----------------------------------------------------
    def phase2_yaw(self, row, result, yaw_samples, yaw_source, t_end):
        """Fill the YAW-level Phase 2 columns (labelling, per-marker residuals, label-free pickup, the direction
        split, rest-pose statistics, optional subset / leave-one-out refits). Never raises: a refused variant
        stores its refusal class in the yaw column."""
        a, tpl = self.args, self.template
        est = result.yaw_estimate
        frames = self._solve_frames
        K = len(tpl.points_mm)
        row['label_frac'] = marker_label_fractions(self.calib_times, len(self.frames))
        trk = track_constellation(frames, tpl)
        mask = moving_mask(trk, yaw_samples, est.lag_s)
        ms = marker_fit_stats(trk, mask)
        row['marker_vis'] = [m['vis'] for m in ms]
        row['marker_rms_mm'] = [_fin(m['rms_mm']) for m in ms]
        row['marker_body_mean_mm'] = [[_fin(v) for v in m['body_mean_mm']] for m in ms]
        wp, wv = worst_pair(trk, mask)
        row['worst_pair'], row['worst_pair_mm'] = wp, wv
        # label-free pickup: every marker (any label or none) within the radius of the fitted axis point
        posed = {float(t): (Rm, o) for t, Rm, o in zip(trk.t, trk.R, trk.origin)}
        window_all = self.window_all
        if a.clip is not None:
            window_all = [f for f in window_all if self._inside_clip(f[0], yaw_samples)]
        lf_frames, st = labelfree_frames(window_all, self.t_start, t_end, est.axis_point_mm,
                                         LABELFREE_RADIUS_MM, posed, tpl.points_mm)
        row.update(labelfree_unlab_per_frame=st['unlab_per_frame'], labelfree_n_unlab=st['n_unlab'],
                   labelfree_n_within_3mm=st['n_within_3mm'], labelfree_p05_min_dist_mm=st['p05_min_dist_mm'])
        lf = solve_variant(lf_frames, yaw_samples, yaw_source, tpl)
        if isinstance(lf, str):
            row.update(yaw_labelfree_deg=lf, template_residual_labelfree_mm=None, labelfree_matched=None,
                       labelfree_marker_vis=None)
        else:
            lest, ltrk = lf
            row.update(yaw_labelfree_deg=math.degrees(lest.yaw_offset_rad),
                       template_residual_labelfree_mm=lest.template_residual_mm,
                       labelfree_matched='%s-%s' % (lest.matched_min, lest.matched_max),
                       labelfree_marker_vis=[m['vis'] for m in marker_fit_stats(
                           ltrk, moving_mask(ltrk, yaw_samples, lest.lag_s))])
        # direction split
        yo, yr = yaw_by_direction(trk, yaw_samples, tpl, est.lag_s, est.phi_raw_deg)
        row['yaw_out_deg'], row['yaw_ret_deg'] = yo, yr
        # rest pose
        rs = rest_stats(self.rest_snapshot, self.t_start, tpl, est.axis_point_mm)
        row.update(rest_n_frames=rs['n_frames'], rest_label_frac=rs['label_frac'],
                   rest_pair_err_max_mm=rs['pair_err_max_mm'],
                   rest_unlab_near_axis_per_frame=rs['unlab_near_axis_per_frame'])
        sub = getattr(a, 'subset', None)
        if sub:
            idx = [int(i) - 1 for i in sub.split(',')]
            v = solve_variant(frames, yaw_samples, yaw_source, sub_template(tpl, idx))
            row['subset'] = sub
            if isinstance(v, str):
                row['yaw_subset_deg'], row['template_residual_subset_mm'] = v, None
            else:
                row['yaw_subset_deg'] = math.degrees(v[0].yaw_offset_rad)
                row['template_residual_subset_mm'] = v[0].template_residual_mm
        if getattr(a, 'loo', False):
            loo = []
            for k in range(K):
                v = solve_variant(frames, yaw_samples, yaw_source,
                                  sub_template(tpl, [i for i in range(K) if i != k]))
                loo.append(None if isinstance(v, str) else math.degrees(v[0].yaw_offset_rad))
            row['yaw_loo_deg'] = loo

    def _inside_clip(self, t, yaw_samples):
        lo, hi = self.args.clip
        ys = sorted(yaw_samples)
        return lo <= float(np.interp(t, [x[0] for x in ys], [x[1] for x in ys])) <= hi

    # -- base-frame helpers (mirrors of the node's private methods) ---------
    def _base_frame_view(self, result):
        view = {'pose': None, 'error': '', 'kappa': None, 'p_b': None,
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
        if view['pose'] is not None:
            view['kappa'], view['p_b'] = bb_in_base(
                view['pose'], math.degrees(result.yaw_offset_rad), result.bb_position_mm)
        return view

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


def _fin(v):
    """nan/inf -> None so the JSON columns stay valid."""
    return float(v) if v is not None and math.isfinite(v) else None


# ───────────────────────────── Phase 2 measurements (pure; tests call them on the fixture) ─────────────────────────────

def moving_mask(track, yaw_samples, lag_s):
    """The solver's moving mask: track frames whose yaw rate at (t + lag) exceeds SWEEP_MIN_SPEED_DEG_S."""
    ys = np.array(sorted((float(t), float(v)) for t, v in yaw_samples)).reshape(-1, 2)
    w = np.gradient(canonical_yaw_deg(ys[:, 1]), ys[:, 0])
    return np.abs(np.interp(track.t + lag_s, ys[:, 0], w)) > SWEEP_MIN_SPEED_DEG_S


def marker_label_fractions(calib_times, n_frames):
    """Per QTM index: fraction of the window's distinct frames in which 'Ball Butler - i' was labelled."""
    return [len(calib_times.get(i, ())) / max(n_frames, 1) for i in range(BB_MARKER_COUNT)]


def marker_fit_stats(track, mask):
    """Per template marker over the frames in mask: visibility (fraction of posed frames matched), RMS 3D
    residual (mm) and the mean residual vector in the BODY frame (mm); a systematic body-frame offset is a
    template error for that marker. Dicts {'vis', 'rms_mm', 'body_mean_mm': [x, y, z], 'n'}."""
    T = track.template_mm
    out = []
    Rm, om, Pm = track.R[mask], track.origin[mask], track.points[mask]
    for k in range(len(T)):
        posed = om + np.einsum('nij,j->ni', Rm, T[k])
        e = Pm[:, k] - posed
        ok = np.all(np.isfinite(e), axis=1)
        if ok.sum() == 0:
            out.append({'vis': 0.0, 'rms_mm': float('nan'), 'body_mean_mm': [float('nan')] * 3, 'n': 0})
            continue
        eb = np.einsum('nji,nj->ni', Rm[ok], e[ok])            # R^T e
        out.append({'vis': float(ok.mean()), 'rms_mm': float(np.sqrt(np.mean(np.sum(e[ok] ** 2, axis=1)))),
                    'body_mean_mm': [float(v) for v in eb.mean(axis=0)], 'n': int(ok.sum())})
    return out


def worst_pair(track, mask):
    """('Qa-Qb', |pose-binned separation error| mm) of the worst marker pair: the SAME statistic as the
    solver's template residual (1-based QTM indices)."""
    ap = np.abs(track.pair_residual_pose_binned_mm(mask))
    if not np.any(np.isfinite(ap)):
        return None, None
    a, b = np.unravel_index(np.nanargmax(np.where(np.isfinite(ap), ap, -1)), ap.shape)
    return 'Q%d-Q%d' % (a + 1, b + 1), float(ap[a, b])


def labelfree_frames(window_all, t0, t1, axis_mm, radius_mm, posed, template_mm=None):
    """Label-free pickup: per window frame, EVERY marker (any label or none) within radius_mm of axis_mm.
    window_all: [(stamp_s, labels, xyz (n,3))]. posed: {stamp: (R, origin)} of the production track. Returns
    (frames [(t, (n,3))], stats). stats: unlab_per_frame (unlabelled near-axis points per posed frame),
    n_unlab, n_other_labelled (near-axis points with a non-BB label), n_within_3mm / p05_min_dist_mm (for each
    UNLABELLED near-axis point, its min distance to the production track's posed template markers at that
    stamp: within 3 mm would be a BB marker QTM failed to label; needs template_mm (K,3), the body-frame
    template the track was fitted with)."""
    ax = np.asarray(axis_mm, float)
    frames, dmins = [], []
    n_unlab = n_other = n_seen = 0
    for stamp, labels, xyz in window_all:
        if stamp is None or not (t0 <= stamp <= t1):
            continue
        xyz = np.asarray(xyz, float).reshape(-1, 3)
        ok = np.all(np.isfinite(xyz), axis=1)
        near = ok & (np.linalg.norm(xyz - ax, axis=1) <= radius_mm)
        pz = posed.get(stamp) if posed else None
        if pz is not None:
            n_seen += 1
        if not near.any():
            continue
        for i in np.flatnonzero(near):
            lab = labels[i]
            if lab.startswith('Ball Butler - '):
                continue
            if lab:
                n_other += 1
                continue
            n_unlab += 1
            if pz is not None and template_mm is not None:
                Rm, o = pz
                tw = np.asarray(template_mm, float) @ Rm.T + o
                dmins.append(float(np.min(np.linalg.norm(tw - xyz[i], axis=1))))
        frames.append((stamp, xyz[near]))
    dm = np.array(dmins)
    return frames, {'unlab_per_frame': n_unlab / max(n_seen or len(frames), 1), 'n_unlab': n_unlab,
                    'n_other_labelled': n_other, 'n_within_3mm': int(np.count_nonzero(dm <= 3.0)),
                    'p05_min_dist_mm': float(np.percentile(dm, 5)) if dm.size else None}


def yaw_by_direction(track, yaw_samples, template, lag_s, phi_raw_deg):
    """(yaw_out_deg, yaw_ret_deg): the yaw offset from the OUT (yaw rising) and RETURN (yaw falling) halves of
    the sweep alone, at the production lag, in the PINNED gauge (phi + gauge_shift, wrapped like the solver).
    phi per moving yaw sample = theta(t - lag) - y - E(y), branch-aligned to phi_raw_deg. None if a half is empty."""
    ys = np.array(sorted((float(t), float(v)) for t, v in yaw_samples)).reshape(-1, 2)
    y = canonical_yaw_deg(ys[:, 1])
    w = np.gradient(y, ys[:, 0])
    th = track.theta_deg
    tq = ys[:, 0] - lag_s
    ok = ((tq >= track.t[0]) & (tq <= track.t[-1]) & (np.abs(w) > SWEEP_MIN_SPEED_DEG_S)
          & (y >= template.e_valid_yaw_deg[0]) & (y <= template.e_valid_yaw_deg[1]))
    phi = np.interp(tq, track.t, th) - y - template.e_deg(y)
    phi = phi - np.round((phi - phi_raw_deg) / 360.0) * 360.0
    res = []
    for sel in (ok & (w > 0), ok & (w < 0)):
        res.append(wrap_deg(float(phi[sel].mean()) + template.gauge_shift_deg) if sel.any() else None)
    return res[0], res[1]


def sub_template(template, idx):
    """The template restricted to marker indices idx (0-based)."""
    idx = list(idx)
    return dataclasses.replace(template, points_mm=template.points_mm[idx],
                               names=tuple(template.names[i] for i in idx))


def solve_variant(frames, yaw_samples, yaw_source, template):
    """One track + estimate_sweep_yaw_offset on arbitrary frames/template. Returns (estimate, track), or the
    refusal class string when the solver refuses."""
    try:
        trk = track_constellation(frames, template)
        est = estimate_sweep_yaw_offset(frames, yaw_samples, template, yaw_source=yaw_source, track=trk)
    except ValueError as e:
        return refusal_class(str(e))
    return est, trk


def rest_stats(recent_all, t_start, template, axis_mm, span_s=REST_SPAN_S, radius_mm=LABELFREE_RADIUS_MM):
    """Rest pose (the span_s before the sweep start, up to 0.5 s before): {'n_frames', 'label_frac' (per QTM
    index), 'pair_err_max_mm' (max |mean separation error vs the template| over labelled pairs),
    'unlab_near_axis_per_frame'}."""
    K = len(template.points_mm)
    T = template.points_mm
    dT = np.linalg.norm(T[:, None] - T[None], axis=2)
    ax = np.asarray(axis_mm, float)
    n_fr, lab_cnt, nu_tot = 0, np.zeros(K), 0
    pair_err = {}
    for stamp, labels, xyz in recent_all:
        if stamp is None or not (t_start - span_s <= stamp < t_start - 0.5):
            continue
        n_fr += 1
        xyz = np.asarray(xyz, float).reshape(-1, 3)
        rows = {}
        for i, lab in enumerate(labels):
            if not np.all(np.isfinite(xyz[i])):
                continue
            m = re.fullmatch(r'Ball Butler - (\d+)', lab)
            if m and 1 <= int(m.group(1)) <= K:
                rows[int(m.group(1)) - 1] = xyz[i]
                lab_cnt[int(m.group(1)) - 1] += 1
            elif not lab and np.linalg.norm(xyz[i] - ax) <= radius_mm:
                nu_tot += 1
        for p, q in itertools.combinations(sorted(rows), 2):
            pair_err.setdefault((p, q), []).append(np.linalg.norm(rows[p] - rows[q]) - dT[p, q])
    errs = [abs(float(np.mean(v))) for v in pair_err.values()]
    return {'n_frames': n_fr, 'label_frac': (lab_cnt / max(n_fr, 1)).tolist(),
            'pair_err_max_mm': max(errs) if errs else None, 'unlab_near_axis_per_frame': nu_tot / max(n_fr, 1)}


def base_pair_errors(base_points, base_template):
    """Mean observed - template separation (mm) of the shelf marker pairs over the window, in the order
    C-M, C-E, C-S, M-E, M-S, E-S; None for a pair never seen / no track."""
    TB = np.asarray(base_template, float)
    dT = np.linalg.norm(TB[:, None] - TB[None], axis=2)
    try:
        bt = track_base_frame(base_points, TB)
    except ValueError:
        return [None] * 6
    out = []
    for a, b in itertools.combinations(range(4), 2):
        d = np.linalg.norm(bt.points[:, a] - bt.points[:, b], axis=1)
        d = d[np.isfinite(d)]
        out.append(float(d.mean()) - float(dT[a, b]) if d.size else None)
    return out


def jsonable(o):
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, KappaRecord):
        return o.to_dict()
    raise TypeError(type(o))


# ───────────────────────────── driver ─────────────────────────────

def replay_bag(bag_dir, args, template, base_model, gate, want_capture, capture_idx=None):
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
    sim = NodeSim(args, template, base_model, gate)
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
            'p_b_mm', 'pool_n_before', 'pooled_kappa_deg', 'pose_source',
            'verdict_p1', 'verdict_text', 'verdict_intrinsic', 'epoch_id', 'epoch_opened',
            'epoch_rejoined', 'delta_kappa_deg', 'delta_axis_b_mm', 'delta_yaw_deg', 'axis_shift_mm',
            'same_world_epoch', 'advisory_flags', 'outcome_line', 'logged_outcome',
            'logged_class', 'logged_pool_n', 'match_measure', 'clip_yaw_deg', 'n_frames_clip_in'] + PHASE2_COLS
    for r in rows:
        L = r.get('logged')
        cmp_ = compare(r)
        r['logged_outcome'] = None if L is None else L.get('outcome')
        r['logged_class'] = None if L is None else L.get('class')
        r['logged_pool_n'] = None if L is None else L.get('gate_pooled_n')
        r['match_measure'] = None if L is None else all(c[4] for c in cmp_)
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
    n_meas = n_meas_ok = 0
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
        lines.append(f'| {name} | logged verdict (Phase 0 node, info) | {L.get("class")} | '
                     f'{r["verdict_p1"]} (Phase 1) | | info |')
    n_sw = len(scored)
    sweeps_all_ok = sum(1 for r in scored if r['match_measure'])
    score = (f'{n_meas_ok} of {n_meas} measurements match '
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
         f'args: yaw_source={args.bb_yaw_source} clip_yaw_deg={args.clip_yaw_deg}', '', '**' + score + '**', '',
         '## Per bag: starts / accepted / refused (Phase 1) / flagged', '',
         '| bag | starts | P1 ACC | P1 REF | flagged |', '|---|---|---|---|---|']
    bags = sorted({r['bag'] for r in rows})
    for b in bags:
        rs = [r for r in rows if r['bag'] == b]
        S.append(f'| {b} | {len(rs)} | {sum(r["verdict_p1"] == "ACCEPTED" for r in rs)} | '
                 f'{sum(r["verdict_p1"] != "ACCEPTED" for r in rs)} | '
                 f'{sum(bool(r.get("advisory_flags")) for r in rs)} |')
    S += ['', '## Phase 1 verdict table', '',
          f'**Of the {len(refused)} logged-refused sweeps (Phase 0 node), {intr_acc} are ACCEPTED under Phase 1 '
          f'({len(refused) - intr_acc} still refused by the solver).**', '',
          '| sweep | logged class -> Phase 1 | flags | epoch | note |', '|---|---|---|---|---|']
    for r in sorted(scored, key=lambda r: r['t_start']):
        L = r['logged']
        ep = '' if r.get('epoch_id') is None else (
            f'K{r["epoch_id"]}' + (' OPENED' if r.get('epoch_opened') else '')
            + (' REJOINED' if r.get('epoch_rejoined') else ''))
        S.append(f'| {r["bag"]}#{r["sweep_idx"]} | {L.get("class")} -> {r["verdict_p1"]} | '
                 f'{r.get("advisory_flags") or "-"} | {ep} | {(r.get("verdict_text") or "")[:80]} |')
    unlogged = [r for r in rows if r.get('logged') is None]
    if unlogged:
        S += ['', f'{len(unlogged)} replayed sweeps have no logged counterpart (10-10 launch logs not read, '
              'or the node did not log them):', '',
              '| sweep | Phase 1 | flags | epoch |', '|---|---|---|---|']
        for r in sorted(unlogged, key=lambda r: r['t_start']):
            ep = '' if r.get('epoch_id') is None else (
                f'K{r["epoch_id"]}' + (' OPENED' if r.get('epoch_opened') else '')
                + (' REJOINED' if r.get('epoch_rejoined') else ''))
            S.append(f'| {r["bag"]}#{r["sweep_idx"]} | {r["verdict_p1"]} | {r.get("advisory_flags") or "-"} | {ep} |')
    from collections import Counter
    hist = Counter(f for r in rows for f in (r.get('advisory_flags') or '').split(';') if f)
    n_acc = sum(r['verdict_p1'] == 'ACCEPTED' for r in rows)
    S += ['', '### Totals', '',
          f'- replayed sweeps: {len(rows)}; Phase 1 ACCEPTED {n_acc}, refused {len(rows) - n_acc} '
          f'({dict(Counter(r["verdict_p1"] for r in rows if r["verdict_p1"] != "ACCEPTED"))})',
          f'- flagged sweeps: {sum(bool(r.get("advisory_flags")) for r in rows)}; flags histogram: '
          f'{dict(sorted(hist.items())) or "none"}',
          '- epochs opened: ' + (', '.join(
              f'K{r["epoch_id"]} at {r["bag"]}#{r["sweep_idx"]} ({r.get("t_start_iso")})'
              for r in sorted(rows, key=lambda r: r['t_start']) if r.get('epoch_opened')) or 'none'),
          '- epochs rejoined: ' + (', '.join(
              f'K{r["epoch_id"]} at {r["bag"]}#{r["sweep_idx"]}'
              for r in sorted(rows, key=lambda r: r['t_start']) if r.get('epoch_rejoined')) or 'none'),
          '']
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
    S += phase2_summary(rows)
    open(os.path.join(outdir, 'summary.md'), 'w').write('\n'.join(S) + '\n')
    return score, intr_acc, len(refused)


def phase2_summary(rows):
    """summary.md section '## Phase 2: labelling, per-marker residuals, refit variants'."""
    K = BB_MARKER_COUNT
    p2 = sorted((r for r in rows if r.get('label_frac') is not None), key=lambda r: r['t_start'])
    S = ['', '## Phase 2: labelling, per-marker residuals, refit variants', '']
    if not p2:
        return S + ['No sweep reached the Phase 2 measurements.']
    q = ' | '.join('Q%d' % (i + 1) for i in range(K))
    tag = lambda r: '%s#%s' % (r['bag'], r['sweep_idx'])
    f2 = lambda v: 'n/a' if v is None else '%.2f' % v
    S += ['### Label coverage (fraction of frames labelled "Ball Butler - i": window, then 15 s rest before)', '',
          '| sweep | src | n frames | ' + q + ' | rest: ' + ' | '.join('Q%d' % (i + 1) for i in range(K)) + ' |',
          '|---|---|---|' + '---|' * (2 * K)]
    for r in p2:
        S.append('| %s | %s | %s | %s | %s |' % (
            tag(r), r.get('yaw_source'), r.get('n_frames'), ' | '.join(f2(v) for v in r['label_frac']),
            ' | '.join(f2(v) for v in (r.get('rest_label_frac') or [None] * K))))
    S += ['', '### Kappa variants (deg) and template residuals (mm)', '',
          '| sweep | kappa | label-free | out | ret | subset | template residual | LF residual |',
          '|---|---|---|---|---|---|---|---|']
    f3 = lambda v: 'n/a' if v is None else ('%.3f' % v if isinstance(v, float) else str(v))
    for r in p2:
        lf = r.get('kappa_labelfree_deg')
        if lf is None and isinstance(r.get('yaw_labelfree_deg'), str):
            lf = r['yaw_labelfree_deg']
        sb = r.get('kappa_subset_deg')
        if sb is None and isinstance(r.get('yaw_subset_deg'), str):
            sb = r['yaw_subset_deg']
        S.append('| %s | %s | %s | %s | %s | %s | %s | %s |' % (
            tag(r), f3(r.get('kappa_deg')), f3(lf), f3(r.get('kappa_out_deg')), f3(r.get('kappa_ret_deg')),
            f3(sb), f3(r.get('template_residual_mm')), f3(r.get('template_residual_labelfree_mm'))))
    n_pts = sum(r.get('labelfree_n_unlab') or 0 for r in p2)
    n3 = sum(r.get('labelfree_n_within_3mm') or 0 for r in p2)
    ident = 0
    for r in p2:
        lv = r.get('labelfree_marker_vis')
        if lv is not None and all(abs(a - b) <= 0.005 for a, b in zip(lv, r['marker_vis'])):
            ident += 1
    S += ['', 'label-free pickup: %d unlabelled near-axis points over %d sweeps, %d within 3 mm of a posed BB '
          'marker; per-marker visibility identical with and without pickup in %d of %d sweeps '
          '(a standing null check: the 2026-10-16 measurement found nothing to pick up)'
          % (n_pts, len(p2), n3, ident, len(p2)), '',
          '### Per-marker body-frame mean residual (x, y, z mm; a systematic offset is a template error)', '',
          '| sweep | ' + q + ' |', '|---|' + '---|' * K]
    for r in p2:
        cells = []
        for m in r['marker_body_mean_mm']:
            cells.append('(' + ', '.join('n/a' if v is None else '%.2f' % v for v in m) + ')')
        S.append('| %s | %s |' % (tag(r), ' | '.join(cells)))
    return S + ['']


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bags', nargs='+', default=[os.path.join(BAG_ROOT, '2026-10-1[0-6]*')],
                    help='one or more bag-dir globs')
    ap.add_argument('--clip-yaw-deg', metavar='LO:HI',
                    help='experiment: solve only the part of each sweep whose BB yaw is in [LO, HI]')
    ap.add_argument('--sweep', help='BAG:IDX to replay one sweep (state is built from prior sweeps of that bag only)')
    ap.add_argument('--ros-log-dir', default=os.path.expanduser('~/.ros/log'))
    ap.add_argument('--state-json', help='snapshot {"base_state": {...}, "accepted": {...}} to start from')
    ap.add_argument('--bb-yaw-source', default='auto', choices=('auto', 'stamped', 'heartbeat'))
    ap.add_argument('--subset', metavar='I,J,..', help='Phase 2: also refit on this subset of 1-based QTM indices '
                    '(e.g. 1,2,4,6,7); adds subset / yaw_subset_deg / kappa_subset_deg columns')
    ap.add_argument('--loo', action='store_true',
                    help='Phase 2: leave-one-marker-out refits (7 extra solves per sweep): yaw_loo_deg / kappa_loo_deg')
    ap.add_argument('--emit-fixture', metavar='BAG:IDX')
    ap.add_argument('--out', help='fixture output path (with --emit-fixture)')
    ap.add_argument('-v', '--verbose', action='store_true')
    args = ap.parse_args()

    t_run = time.time()
    template = load_marker_template(os.path.join(RES_DIR, 'bb_marker_template.json'))
    base_model = load_base_frame(os.path.join(RES_DIR, 'bb_base_frame.json'))
    snap = json.load(open(args.state_json)) if args.state_json else None
    gate = GateState(base_model, snap)
    logged = parse_launch_logs(args.ros_log_dir)
    want = (args.emit_fixture.rsplit(':', 1) if args.emit_fixture else None)
    only = (args.sweep.rsplit(':', 1) if args.sweep else None)

    bag_dirs = sorted({d for g in args.bags for d in glob.glob(g) if os.path.isdir(d)})
    args.clip = tuple(float(x) for x in args.clip_yaw_deg.split(':')) if args.clip_yaw_deg else None
    rows, fixture = [], None
    for bd in bag_dirs:
        name = os.path.basename(bd)
        cap_this = bool(want and want[0] == name)
        t0 = time.time()
        r, cap = replay_bag(bd, args, template, base_model, gate, cap_this,
                            int(want[1]) if want else None)
        rows += r
        print(f'{name}: {len(r)} sweeps ({time.time() - t0:.0f} s)', flush=True)
        if cap and want:
            idx = int(want[1])
            fixture = cap
        if args.verbose:
            for x in r:
                print(f'  #{x["sweep_idx"]} {x["t_start_iso"]} {x["verdict_p1"]} '
                      f'yaw {fmt(x.get("yaw_offset_deg"))} kappa {fmt(x.get("kappa_deg"))} '
                      f'res {fmt(x.get("template_residual_mm"), 2)} end {fmt(x.get("sweep_end_deg"), 1)} '
                      f'n_dir {x.get("n_direction_changes")} | {x["verdict_text"][:100]} | {x.get("advisory_flags", "")}')
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
