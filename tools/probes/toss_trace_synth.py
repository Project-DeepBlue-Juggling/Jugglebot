#!/usr/bin/env python3
"""toss_trace_synth — synthetic-trace generator + end-to-end verification
matrix for the Phase-3 toss choreography checker
(``tests/hardware/toss_trace_recorder.py check``).

Motivating logbook entry: ``logbook/2026-07-25-toss-phase3-prep-trace-harness.md``
(single-ball-toss Phase 3 prep).  The checker it exercises:
``tests/hardware/toss_trace_recorder.py`` (DT-1..DT-14 dry invariants,
RJ-1..RJ-4 reject invariants).

Why this exists as a committed probe: the checker is the hard gate before any
ball flies, and the only way to trust an offline checker is to feed it traces
whose verdict is known by construction — both directions.  The generator
produces:

* the two HAPPY-PATH traces (waived dry choreography ending ``Toss MISSED``;
  un-waived ``REJECTED_NO_BALL`` reject) — the checker must PASS every
  invariant and exit 0 on each;
* ONE VIOLATION TRACE PER INVARIANT (18 cases, ``viol_dt1..viol_dt14`` and
  ``viol_rj1..viol_rj4``) — each flips exactly the ordering/property its
  invariant checks, and the checker must FAIL exactly that invariant with
  zero collateral (all other invariants PASS); and
* the ``dry_no_release`` VARIANT trace (``ABORTED_NO_RELEASE`` — the throw
  was dispatched, feedback reached THROWING, but no stroke/release evidence
  ever appeared and the release deadline aborted the goal; the live
  ERR_TIMEOUT-epidemic shape).  Expected post-DT-5-fix behaviour: the
  NO_RELEASE banner prints, EXACTLY DT-9 and DT-12 SKIP (the two
  release-dependent invariants), DT-5 still EVALUATES — the THROWING
  feedback rows exist on this variant, so the dispatch-time proxy is live
  and the pre-announcement negative scan runs clean — everything else
  PASSes, exit 0.

The ``viol_dt5`` case is the empirically-confirmed 2026-07-25 audit
inversion: the hand cmd-echo changes BEFORE the announcement (dispatch
evidence preceding the announcement).  The pre-fix checker PASSED this
inverted trace, because its scan anchored at the announcement row and
rediscovered the already-changed echo as a positive gap; the fixed checker's
pre-announcement negative scan hard-FAILs it.  Re-run this matrix after any
checker change.

Trace rows mirror the recorder's JSONL schema exactly
(``{'t', 't_ros', 'topic', 'n', 'd'}``; ``t_ros == t`` — the synthetic clock
is consistent).  Pure stdlib; runs under any Python >= 3.8, venv or system.

Usage (from the repo root)::

    python tools/probes/toss_trace_synth.py --all            # generate the 21 traces
    python tools/probes/toss_trace_synth.py --all --verify   # + run the checker matrix
    python tools/probes/toss_trace_synth.py --case viol_dt5 --verify
    python tools/probes/toss_trace_synth.py --list

Outputs go to ``temp/probes/toss_trace_synth/<case>.jsonl``.  ``--verify``
exits 0 iff every matrix row matches its expectation (happy: exit 0, all
PASS; violation: exit 1, the targeted invariant FAILs, every other invariant
PASSes).
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
CHECKER = REPO_ROOT / 'tests' / 'hardware' / 'toss_trace_recorder.py'
OUT_DIR = REPO_ROOT / 'temp' / 'probes' / 'toss_trace_synth'

# Import the checker module for the topic-name constants so the generator can
# never drift from the recorder's schema.  Module import is pure stdlib (the
# rclpy imports live inside the record subcommand).
_spec = importlib.util.spec_from_file_location('toss_trace_recorder',
                                               str(CHECKER))
rec = importlib.util.module_from_spec(_spec)
sys.modules['toss_trace_recorder'] = rec   # dataclasses resolves __module__
_spec.loader.exec_module(rec)

GID = 'aabbccdd'
VERDICT_RE = re.compile(r'^(DT-\d+|RJ-\d+)\s+(PASS|FAIL|AMBIGUOUS|SKIP)\b')


# ── row builders ─────────────────────────────────────────────────────────────

def row(t, topic, d):
    t = round(t, 4)
    return {'t': t, 't_ros': t, 'topic': topic, 'n': 0, 'd': d}


def hand(t, pos_meas=0.05, vel_meas=0.0, pos_cmd=0.0, vel_ff=0.0):
    return row(t, rec.T_HAND, {
        'pos_meas': pos_meas, 'vel_meas': vel_meas, 'pos_cmd': pos_cmd,
        'vel_ff_cmd': vel_ff, 'tor_ff_cmd': 0.0, 'iq_meas': 0.0})


def rosout(t, node, msg, level=30):
    return row(t, rec.T_ROSOUT, {'node': node, 'level': level, 'msg': msg})


def fb(t, phase):
    return row(t, rec.T_TOSS_FB, {'goal_id': GID, 'phase': phase})


def status(t, st):
    return row(t, rec.T_TOSS_STATUS, {'goals': [[GID, st]]})


def traj(t, plan_kind='hold', streaming=True, mode='TRAJECTORY'):
    return row(t, rec.T_TRAJ_STATUS, {
        'streaming': streaming, 'mode': mode, 'plan_kind': plan_kind,
        'seq': 0, 'last_rejection': ''})


def boolrow(t, topic, value):
    return row(t, topic, {'value': bool(value)})


def frange(t0, t1, step):
    """Inclusive-of-t1 float range on a rounded grid."""
    out = []
    k = 0
    while True:
        t = round(t0 + k * step, 4)
        if t > t1 + 1e-9:
            break
        out.append(t)
        k += 1
    return out


# ── dry trace (waived choreography, outcome MISSED) ──────────────────────────
#
# Anchor timeline (matches the production choreography's shape; all gaps are
# comfortably above the 5 ms epsilon and the 10 ms >=1-tick floor):
#   100.000  action status EXECUTING (goal window start)
#   100.050  waiver WARN (exactly one, at goal start)
#   100.05-  POSITIONING feedback; traj plan_kind move -> hold
#   101.000  prime_hold True
#   101.050  vel_scale (PREPARE bundle) ... 101.052 prime_dispatched stamp
#   101.060  armed True (+ 'catch latch armed' log)
#   101.110  self-announcement (throw_time 106.0)
#   101.115  CCN pre-tilt log; 101.120 dynamic_target (0,0,170); 101.130 ack
#   101.155  first THROWING feedback (the DT-5 dispatch-time proxy)
#   106.000  stroke onset (vel >= 40 rev/s), BALL_IN_FLIGHT fb from 106.150
#   108.000  'Toss MISSED'; teardown: armed False 108.100 -> disarm log
#            108.150 -> belt stamp False 108.200 -> prime_hold False 108.400
#   108.500  action status ABORTED (goal window end)

def gen_dry(case=None):
    t_hold = 101.053 if case == 'viol_dt1' else 101.000
    t_vs = 101.100 if case == 'viol_dt3' else 101.050
    t_arm = 101.150 if case == 'viol_dt2' else 101.060
    t_dyn = 101.160 if case == 'viol_dt2' else 101.120
    t_tfb = 101.170 if case == 'viol_dt2' else 101.130
    missed_msg = ('Toss MISSED (tracker corrupt)' if case == 'viol_dt12'
                  else 'Toss MISSED')

    R = []
    # goal window
    R.append(status(100.000, 2))
    R.append(status(108.500, 6))
    # waiver WARN (one, at goal start)
    R.append(rosout(100.050, 'reload_coordinator_node',
                    'waiving the ball-possession latch — trace-only bench '
                    'affordance', level=30))
    # feedback phases
    for t in frange(100.050, 100.950, 0.15):
        R.append(fb(t, 'POSITIONING'))
    for t in (101.005, 101.055, 101.105, 101.145):
        R.append(fb(t, 'PREPARING'))
    for t in frange(101.155, 105.655, 0.5) + [106.055]:
        R.append(fb(t, 'THROWING'))
    for t in (106.150, 106.450, 106.750):
        R.append(fb(t, 'BALL_IN_FLIGHT'))
    for t in (106.950, 107.250):
        R.append(fb(t, 'CATCHING'))
    # trajectory status: positioning move -> hold; control mode
    R.append(traj(100.100, 'move'))
    R.append(traj(100.300, 'move'))
    for t in frange(100.500, 108.000, 0.5):
        R.append(traj(t, 'hold'))
    for t in frange(100.200, 108.200, 1.0):
        R.append(row(t, rec.T_CONTROL_MODE, {'value': 'TRAJECTORY'}))
    # PREPARE bundle -> armed -> announcement -> pre-tilt
    R.append(boolrow(t_hold, rec.T_PRIME_HOLD, True))
    R.append(row(t_vs, rec.T_VEL_SCALE, {'value': 0.8}))
    R.append(boolrow(101.052, rec.T_PRIME_DISPATCHED, True))
    R.append(boolrow(t_arm, rec.T_ARMED, True))
    R.append(rosout(round(t_arm + 0.002, 4), 'trajectory_node',
                    'catch latch armed', level=20))
    R.append(row(101.110, rec.T_ANNOUNCE, {
        'thrower_name': 'jugglebot', 'target_id': 'jugglebot',
        'throw_time': 106.0, 'landing_time': 106.8,
        'predicted_tof_sec': 0.8, 'landing_position': [0.0, 0.0, 170.0],
        'initial_velocity': [0.0, 0.0, 3.93]}))
    R.append(rosout(101.115, 'catch_coordinator_node',
                    'pre-tilt target published from announcement', level=20))
    R.append(row(t_dyn, rec.T_DYN_TARGET, {
        'target_pos': [0.0, 0.0, 170.0], 'target_quat': [1.0, 0.0, 0.0, 0.0],
        'target_vel': [0.0, 0.0, 0.0], 'arrival_time': 106.8}))
    R.append(row(t_tfb, rec.T_TARGET_FB, {
        'accepted': True, 'code': 'OK', 'reason': '', 'arrival_time': 106.8,
        'source': 'catch'}))
    # outcome + teardown (prime_hold released LAST)
    R.append(rosout(108.000, 'reload_coordinator_node', missed_msg, level=30))
    R.append(boolrow(108.100, rec.T_ARMED, False))
    R.append(rosout(108.150, 'trajectory_node', 'catch latch disarmed',
                    level=20))
    R.append(boolrow(108.200, rec.T_PRIME_DISPATCHED, False))
    R.append(boolrow(108.400, rec.T_PRIME_HOLD, False))

    # hand telemetry: rest grid + the single stroke + descent back to park
    echo_early = case == 'viol_dt5'
    for t in frange(99.0, 110.4, 0.02):
        if 106.0 - 1e-9 <= t <= 107.5 + 1e-9:
            continue
        if echo_early and 100.90 - 1e-9 <= t <= 101.10 + 1e-9:
            # the audit-confirmed inversion: dispatch echo BEFORE the ann
            R.append(hand(t, pos_meas=0.05, vel_meas=0.0,
                          pos_cmd=10.0, vel_ff=50.0))
        else:
            R.append(hand(t))
    stroke = [(106.00, 0.6, 45.0), (106.02, 2.5, 52.0), (106.04, 5.0, 55.0),
              (106.06, 7.5, 54.0), (106.08, 9.0, 50.0), (106.10, 9.5, 44.0)]
    for t, p, v in stroke:
        R.append(hand(t, pos_meas=p, vel_meas=v, pos_cmd=10.0, vel_ff=50.0))
    if case == 'viol_dt9':
        # second stroke episode (re-dispatch class) at 107.0
        for t in frange(106.12, 106.96, 0.02):
            R.append(hand(t, pos_meas=max(0.3, 9.0 - (t - 106.12) * 6.5),
                          vel_meas=-20.0))
        for t, p, v in ((107.00, 0.8, 46.0), (107.02, 2.5, 50.0),
                        (107.04, 5.0, 50.0), (107.06, 7.0, 45.0)):
            R.append(hand(t, pos_meas=p, vel_meas=v, pos_cmd=10.0,
                          vel_ff=50.0))
        for t in frange(107.08, 107.5, 0.02):
            R.append(hand(t, pos_meas=max(0.3, 7.0 - (t - 107.08) * 18.0),
                          vel_meas=-20.0))
    else:
        for t in frange(106.12, 107.5, 0.02):
            R.append(hand(t, pos_meas=max(0.3, 9.0 - (t - 106.12) * 6.5),
                          vel_meas=-20.0))

    # single-invariant injections
    if case == 'viol_dt4':
        R.append(boolrow(99.800, rec.T_PRIME_DISPATCHED, True))
    if case == 'viol_dt6':
        R.append(rosout(103.000, 'catch_coordinator_node',
                        'auto-priming hand for catch (soft gains)', level=20))
    if case == 'viol_dt7':
        R.append(row(101.500, rec.T_DYN_TARGET, {
            'target_pos': [0.0, 0.0, 170.0],
            'target_quat': [1.0, 0.0, 0.0, 0.0],
            'target_vel': [0.0, 0.0, 0.0], 'arrival_time': 106.8}))
    if case == 'viol_dt8':
        R.append(row(100.500, rec.T_DYN_TARGET, {
            'target_pos': [0.0, 0.0, 170.0],
            'target_quat': [1.0, 0.0, 0.0, 0.0],
            'target_vel': [0.0, 0.0, 0.0], 'arrival_time': 106.8}))
    if case == 'viol_dt10':
        R.append(fb(103.000, 'PREPARING'))     # regression mid-THROWING
    if case == 'viol_dt11':
        R.append(row(108.600, rec.T_VEL_SCALE, {'value': 1.0}))
    if case == 'viol_dt13':
        R.append(traj(104.000, 'hold', streaming=False))
    if case == 'viol_dt14':
        R.append(row(103.000, rec.T_BALLS, {'n': 1, 'balls': [
            {'id': 1, 'status': 2, 'destination': 'jugglebot',
             'tracking': 1}]}))
    return R


# ── dry NO_RELEASE variant (dispatch eaten: THROWING reached, no stroke) ─────
#
# Faithful to toss_sequencer._step_throwing: the FSM enters THROWING and
# dispatches (feedback THROWING from that tick on), the dispatch ack is
# OK/AMBIGUOUS but neither throw_stroke_seen nor ball_track_confirmed ever
# arrives, and at t_release + grace the goal aborts NO_RELEASE (SAFE_ABORT
# teardown; the hand never leaves the park band, the cmd-echo never changes).

def gen_dry_no_release():
    R = []
    R.append(status(100.000, 2))
    R.append(status(107.100, 6))
    R.append(rosout(100.050, 'reload_coordinator_node',
                    'waiving the ball-possession latch — trace-only bench '
                    'affordance', level=30))
    for t in frange(100.050, 100.950, 0.15):
        R.append(fb(t, 'POSITIONING'))
    for t in (101.005, 101.055, 101.105, 101.145):
        R.append(fb(t, 'PREPARING'))
    # THROWING feedback continues until the release-deadline abort tick
    # (~t_release 106.0 + grace 0.5; the aborting tick publishes no feedback)
    for t in frange(101.155, 106.455, 0.5):
        R.append(fb(t, 'THROWING'))
    R.append(traj(100.100, 'move'))
    R.append(traj(100.300, 'move'))
    for t in frange(100.500, 108.000, 0.5):
        R.append(traj(t, 'hold'))
    for t in frange(100.200, 108.200, 1.0):
        R.append(row(t, rec.T_CONTROL_MODE, {'value': 'TRAJECTORY'}))
    R.append(boolrow(101.000, rec.T_PRIME_HOLD, True))
    R.append(row(101.050, rec.T_VEL_SCALE, {'value': 0.8}))
    R.append(boolrow(101.052, rec.T_PRIME_DISPATCHED, True))
    R.append(boolrow(101.060, rec.T_ARMED, True))
    R.append(rosout(101.062, 'trajectory_node', 'catch latch armed', level=20))
    R.append(row(101.110, rec.T_ANNOUNCE, {
        'thrower_name': 'jugglebot', 'target_id': 'jugglebot',
        'throw_time': 106.0, 'landing_time': 106.8,
        'predicted_tof_sec': 0.8, 'landing_position': [0.0, 0.0, 170.0],
        'initial_velocity': [0.0, 0.0, 3.93]}))
    R.append(rosout(101.115, 'catch_coordinator_node',
                    'pre-tilt target published from announcement', level=20))
    R.append(row(101.120, rec.T_DYN_TARGET, {
        'target_pos': [0.0, 0.0, 170.0], 'target_quat': [1.0, 0.0, 0.0, 0.0],
        'target_vel': [0.0, 0.0, 0.0], 'arrival_time': 106.8}))
    R.append(row(101.130, rec.T_TARGET_FB, {
        'accepted': True, 'code': 'OK', 'reason': '', 'arrival_time': 106.8,
        'source': 'catch'}))
    # release deadline passes with zero evidence -> abort + SAFE_ABORT teardown
    R.append(rosout(106.600, 'reload_coordinator_node',
                    'Toss ABORTED_NO_RELEASE', level=30))
    R.append(boolrow(106.700, rec.T_ARMED, False))
    R.append(rosout(106.750, 'trajectory_node', 'catch latch disarmed',
                    level=20))
    R.append(boolrow(106.800, rec.T_PRIME_DISPATCHED, False))
    R.append(boolrow(107.000, rec.T_PRIME_HOLD, False))
    # hand: parked the whole time — no stroke, cmd-echo never changes
    for t in frange(99.0, 110.4, 0.02):
        R.append(hand(t))
    return R


# ── reject trace (un-waived REJECTED_NO_BALL, total silence) ─────────────────

def gen_reject(case=None):
    outcome = ('Toss REJECTED_WRONG_MODE' if case == 'viol_rj1'
               else 'Toss REJECTED_NO_BALL')
    R = []
    R.append(status(200.000, 2))
    R.append(status(200.300, 6))
    R.append(rosout(200.250, 'reload_coordinator_node', outcome, level=30))
    for t in frange(199.4, 202.2, 0.4):
        R.append(traj(t, 'hold'))
    for t in (199.5, 200.5, 201.5):
        R.append(row(t, rec.T_CONTROL_MODE, {'value': 'TRAJECTORY'}))
    for t in frange(198.0, 202.2, 0.02):
        R.append(hand(t))
    if case == 'viol_rj2':
        R.append(boolrow(200.100, rec.T_PRIME_HOLD, True))
    if case == 'viol_rj3':
        R.append(rosout(200.050, 'reload_coordinator_node',
                        'waiving the ball-possession latch — trace-only '
                        'bench affordance', level=30))
    if case == 'viol_rj4':
        R.append(fb(200.100, 'CHECKING'))
    return R


# ── case table ───────────────────────────────────────────────────────────────
# case name -> (mode, targeted invariant or None, one-line description)

CASES = {
    'dry_happy': ('dry', None, 'full waived choreography, outcome MISSED'),
    'reject_happy': ('reject', None, 'un-waived REJECTED_NO_BALL, silence'),
    'dry_no_release': ('dry', None, 'ABORTED_NO_RELEASE variant: dispatch '
                                    'eaten, THROWING reached, no stroke — '
                                    'banner + DT-9/DT-12 SKIP, rest PASS'),
    'viol_dt1': ('dry', 'DT-1', 'prime_hold->armed gap 7 ms (< 1-tick floor)'),
    'viol_dt2': ('dry', 'DT-2', 'armed True AFTER the announcement (40 ms)'),
    'viol_dt3': ('dry', 'DT-3', 'vel_scale AFTER armed True (40 ms)'),
    'viol_dt4': ('dry', 'DT-4', 'second prime_dispatched stamp (re-stamping)'),
    'viol_dt5': ('dry', 'DT-5', 'hand cmd-echo changes BEFORE the '
                                'announcement (the audit inversion)'),
    'viol_dt6': ('dry', 'DT-6', 'CCN auto-prime log inside the hold window'),
    'viol_dt7': ('dry', 'DT-7', 'duplicated dynamic_target after the ann'),
    'viol_dt8': ('dry', 'DT-8', 'dynamic_target BEFORE armed True'),
    'viol_dt9': ('dry', 'DT-9', 'second stroke episode (re-dispatch class)'),
    'viol_dt10': ('dry', 'DT-10', 'phase regression THROWING -> PREPARING'),
    'viol_dt11': ('dry', 'DT-11', 'vel_scale row AFTER prime_hold False'),
    'viol_dt12': ('dry', 'DT-12', 'MISSED line carries a diagnostic suffix'),
    'viol_dt13': ('dry', 'DT-13', 'streaming=False row inside the window'),
    'viol_dt14': ('dry', 'DT-14', 'balls row in the window (pollution)'),
    'viol_rj1': ('reject', 'RJ-1', 'wrong reject code (REJECTED_WRONG_MODE)'),
    'viol_rj2': ('reject', 'RJ-2', 'prime_hold row during the reject'),
    'viol_rj3': ('reject', 'RJ-3', 'waiver WARN during the un-waived capture'),
    'viol_rj4': ('reject', 'RJ-4', 'a feedback publish during the reject'),
}


# Variant cases: not all-PASS, not single-FAIL — expected exit code, the
# exact SKIP set (every other invariant must PASS), and a required stdout
# banner substring.
SPECIAL_EXPECT = {
    'dry_no_release': {
        'exit': 0,
        'skips': ('DT-9', 'DT-12'),
        'banner': 'NO_RELEASE variant',
    },
}


def write_case(name):
    mode, _target, _desc = CASES[name]
    if name == 'dry_no_release':
        rows = gen_dry_no_release()
    elif mode == 'dry':
        rows = gen_dry(name)
    else:
        rows = gen_reject(name)
    rows.sort(key=lambda r: r['t'])
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUT_DIR / ('%s.jsonl' % name)
    with open(path, 'w') as fh:
        for r in rows:
            fh.write(json.dumps(r) + '\n')
    return path


def run_checker(path, mode):
    proc = subprocess.run(
        [sys.executable, str(CHECKER), 'check', str(path), '--%s' % mode],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    out = proc.stdout.decode(errors='replace')
    verdicts = {}
    for line in out.splitlines():
        m = VERDICT_RE.match(line.strip())
        if m:
            verdicts[m.group(1)] = m.group(2)
    return proc.returncode, verdicts, out


def verify(names):
    all_ids = {'dry': ['DT-%d' % i for i in range(1, 15)],
               'reject': ['RJ-%d' % i for i in range(1, 5)]}
    print('%-14s %-6s %-4s %-10s %-9s %-22s %s'
          % ('case', 'mode', 'exit', 'targeted', 'verdict', 'others', 'RESULT'))
    print('-' * 92)
    ok = True
    for name in names:
        mode, target, _desc = CASES[name]
        path = write_case(name)
        code, verdicts, out = run_checker(path, mode)
        expected_ids = all_ids[mode]
        missing = [i for i in expected_ids if i not in verdicts]
        if name in SPECIAL_EXPECT:
            exp = SPECIAL_EXPECT[name]
            skips = exp['skips']
            bad = {i: v for i, v in verdicts.items()
                   if i in expected_ids
                   and v != ('SKIP' if i in skips else 'PASS')}
            banner_ok = exp['banner'] in out
            row_ok = (code == exp['exit'] and not missing and not bad
                      and banner_ok)
            target = '/'.join(skips)
            tgt_v = '/'.join(verdicts.get(i, 'MISSING') for i in skips)
            n_pass = sum(1 for i, v in verdicts.items()
                         if i in expected_ids and v == 'PASS')
            others_s = '%d PASS' % n_pass
            others_s += ' banner' if banner_ok else ' NO-BANNER'
            if bad:
                others_s += ' + ' + ','.join(
                    '%s=%s' % kv for kv in sorted(bad.items()))
            if missing:
                others_s += ' MISSING:%s' % ','.join(missing)
            print('%-14s %-6s %-4d %-10s %-9s %-22s %s'
                  % (name, mode, code, target, tgt_v, others_s,
                     'OK' if row_ok else 'MISMATCH'))
            ok = ok and row_ok
            continue
        others = {i: v for i, v in verdicts.items()
                  if i in expected_ids and i != target}
        n_pass = sum(1 for v in others.values() if v == 'PASS')
        bad_others = {i: v for i, v in others.items() if v != 'PASS'}
        if target is None:
            row_ok = (code == 0 and not missing and not bad_others)
            tgt_v = '-'
        else:
            tgt_v = verdicts.get(target, 'MISSING')
            row_ok = (code == 1 and not missing and tgt_v == 'FAIL'
                      and not bad_others)
        others_s = ('%d PASS' % n_pass) + (
            '' if not bad_others else ' + ' + ','.join(
                '%s=%s' % kv for kv in sorted(bad_others.items())))
        if missing:
            others_s += ' MISSING:%s' % ','.join(missing)
        print('%-14s %-6s %-4d %-10s %-9s %-22s %s'
              % (name, mode, code, target or '-', tgt_v, others_s,
                 'OK' if row_ok else 'MISMATCH'))
        ok = ok and row_ok
    print('-' * 92)
    print('matrix: %s' % ('CLEAN — every case matches its expectation'
                          if ok else 'MISMATCH(ES) — see rows above'))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(
        description='Synthetic-trace generator + verification matrix for the '
                    'toss choreography checker (see module docstring)')
    ap.add_argument('--all', action='store_true',
                    help='generate all %d traces' % len(CASES))
    ap.add_argument('--case', choices=sorted(CASES), action='append',
                    help='generate one named case (repeatable)')
    ap.add_argument('--verify', action='store_true',
                    help='run the checker on the generated traces and assert '
                         'the expectation matrix')
    ap.add_argument('--list', action='store_true',
                    help='list the cases and exit')
    args = ap.parse_args()
    if args.list:
        for name, (mode, target, desc) in CASES.items():
            print('%-14s %-6s %-6s %s' % (name, mode, target or '-', desc))
        return 0
    names = list(CASES) if (args.all or not args.case) else args.case
    if args.verify:
        return verify(names)
    for name in names:
        print('wrote %s' % write_case(name))
    return 0


if __name__ == '__main__':
    sys.exit(main())
