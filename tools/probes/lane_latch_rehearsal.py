#!/usr/bin/env python3
"""Lane-latch recovery DRESS REHEARSAL (R5 sitting 8, before any balls).

Injects, ON PURPOSE and on the powered robot, the real fault mechanism that
latched a streamed lane at sitting 7: a ~100 ms Jetson stall of
``trajectory_node`` in the middle of a leg move, so the firmware's scheduled
playback loses its cover (SCHED_COVER 50 ms + SCHED_GRACE_S 20 ms = 70 ms),
refuses the discontinuous resume (legs moving faster than
SCHED_RESUME_TOL_VEL_LEG_RPS = 0.3 rps) and sets ``sched_hold_latched``
(``/link_status`` heartbeat flag, bit 15). It then watches the automatic
recovery (skill_node T2 -> bridge ``/recover_lane``) and reports EVERY
expectation at once. Design: scratch ``unitC_design.md`` § 7 / logbook
``2026-10-13-skill-stack-r5-wave-2-after-sitting-7.md``. The mocked tests are
blind to this choreography (does the arm edge clear bit 15, do the ODrives hold
across the disarm, the 0.2 s dwell) -- this is the only check of it.

Run with the SYSTEM python and ROS sourced (NOT the project venv; the venv has
no rclpy), launch UP and loaded (bag + GUI), mode TRAJECTORY, ``fault_state``
NONE, before balls. Run the observation-only pass first:

    bash -c 'source /opt/ros/foxy/setup.bash && \\
             source ~/Desktop/Jugglebot-skills/ros_ws/install/setup.bash && \\
             cd ~/Desktop/Jugglebot-skills && \\
             python3 tools/probes/lane_latch_rehearsal.py --no-freeze'
    # ... then, with the owner's OK for injecting the fault:
    bash -c '... same ... python3 tools/probes/lane_latch_rehearsal.py'

Options: ``--freeze-ms 100`` (refused >= 150: SETPOINT_STALE is 250 ms),
``--at-ms 450`` (into the move), ``--dx-mm 50``, ``--move-s 1.2``,
``--no-freeze`` (everything except SIGSTOP/SIGCONT), ``--out <json>``
(default ``temp/probes/lane_latch_rehearsal_<ts>.json``).

Leg-speed check (the latch needs legs moving at the resume): a quintic
point-to-point move peaks at 1.875 x its mean speed, 50 mm / 1.2 s -> mean
41.7 mm/s, peak 78 mm/s (40 mm / 0.5 s is REFUSED by the planner as TOO_FAST,
minimal feasible 0.822-0.823 s at the current leg limits). ``leg_peak_rps`` projects that through the Stewart
Jacobian at the ACTIVE pose (z = 170) and ``mm_to_rev``; the pre-flight
REFUSES a dx/move-s that does not exceed 0.3 rps on at least one leg (so a
rehearsal can never "pass" by not latching). Measured value at the defaults
(0.25 0.40 0.40 0.25 0.15 0.15 rps, 2026-10-14) is printed by the pre-flight and pinned by the test.

SAFETY. The script sends exactly ONE ``trajectory/go_to_pose`` and ONE
``trajectory/go_home`` and nothing else: never ``/recover``, ``/recover_lane``,
clear_errors, arm/disarm, or any hand service. It freezes ONLY the single
``trajectory_node`` pid it verified (refuses on 0 or > 1 matches) and always
SIGCONTs in a ``finally`` (a leaked STOP is a SETPOINT_STALE E-STOP after
250 ms). If ``fault_state`` leaves NONE it reports a FAIL row and does NOT try
to recover: run /recover (DEACTIVATE then ACTIVATE), then re-run.

Structure: a pure layer (pre-flight, leg-speed check, verdict) with no I/O,
unit-tested in ``tests/motion/test_lane_latch_rehearsal.py``; and a thin
rclpy/OS layer below ``# --- ROS / OS layer``.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import signal
import sys
import time

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

# --- constants (literals on purpose: this probe is a RECORD of the expectation)
RESUME_TOL_VEL_LEG_RPS = 0.3     # canbridge_config.h SCHED_RESUME_TOL_VEL_LEG_RPS
QUINTIC_PEAK_FACTOR = 1.875      # peak/mean speed of a quintic rest-to-rest move
ACTIVE_Z_MM = 170.0              # STOW-relative z of the ACTIVE pose
SETPOINT_STALE_S = 0.250
FREEZE_MAX_MS = 150.0            # refuse >= this (leaves 100 ms margin to STALE)
COVER_MS = 70.0                  # SCHED_COVER 50 + SCHED_GRACE_S 20
IN_PROGRESS_WITHIN_S = 1.0       # latch rise -> lane_recovery in_progress
DONE_WITHIN_S = 10.0             # latch rise -> done
MAX_LIVE_DEV_REV = 0.1           # through the recovery (converge done -> DONE)
LANE_ESCAPE_DEV_REV = 0.5        # teensy_bridge_node._LANE_ESCAPE_DEV_REV (early disarm)
MAX_DEVIATION_REV = 1.0          # canbridge_config.h MAX_DEVIATION_REV (firmware E-STOP)
CONVERGED_DEV_REV = 0.05         # live_dev below this = converge stage done
WATCH_S = 20.0
STAGES = ('converge', 'rest', 'disarm', 'arm', 'confirm')
FAULT_TEXT = 'real-fault path: run /recover (DEACTIVATE then ACTIVATE), then re-run'
NO_LATCH_TEXT = 'the freeze did not latch: lengthen --freeze-ms or --dx-mm'


# ===================================================================== pure layer
def parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--freeze-ms', type=float, default=100.0)
    ap.add_argument('--at-ms', type=float, default=450.0)
    ap.add_argument('--dx-mm', type=float, default=50.0)
    ap.add_argument('--move-s', type=float, default=1.2)
    ap.add_argument('--no-freeze', action='store_true')
    ap.add_argument('--out', default=None)
    return ap.parse_args(argv)


def check_args(args):
    """Argument problems (list of strings; empty = ok)."""
    bad = []
    if args.freeze_ms >= FREEZE_MAX_MS:
        bad.append('--freeze-ms %.0f >= %.0f: SETPOINT_STALE is %.0f ms and a late '
                   'SIGCONT E-STOPs the legs' % (args.freeze_ms, FREEZE_MAX_MS,
                                                 SETPOINT_STALE_S * 1e3))
    if args.freeze_ms <= 0 and not args.no_freeze:
        bad.append('--freeze-ms must be > 0')
    if args.move_s <= 0 or args.move_s > 5.0:
        bad.append('--move-s %.2f outside (0, 5] s' % args.move_s)
    if abs(args.dx_mm) <= 0 or abs(args.dx_mm) > 80.0:
        bad.append('--dx-mm %.1f outside (0, 80] mm' % args.dx_mm)
    if args.at_ms < 50 or args.at_ms > args.move_s * 1e3 - args.freeze_ms - 50:
        bad.append('--at-ms %.0f must leave the freeze INSIDE the move '
                   '(50 ms .. move - freeze - 50 ms)' % args.at_ms)
    return bad


def leg_peak_rps(dx_mm, move_s, mm_to_rev=None):
    """Peak per-leg speed (rev/s) of a lateral (x) quintic move of dx over move_s,
    at the ACTIVE pose. Returns the 6 leg values. Lazy numpy/kinematics import."""
    import numpy as np
    sys.path.insert(0, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'))
    from jugglebot.motion.geometry import StewartGeometry
    from jugglebot.motion.ik_solver import compute_jacobian
    geom = StewartGeometry()
    if mm_to_rev is None:
        mm_to_rev = np.asarray(geom.mm_to_rev, dtype=float)
    pos = np.array([0.0, 0.0, ACTIVE_Z_MM])
    J = compute_jacobian(pos, np.eye(3), geom)
    v = QUINTIC_PEAK_FACTOR * abs(dx_mm) / move_s
    twist = np.array([v, 0.0, 0.0, 0.0, 0.0, 0.0])
    return [abs(float(x)) for x in (J @ twist) * np.asarray(mm_to_rev)]


def preflight(snap, args, peak_rps=None):
    """Every failed precondition, all at once (list of strings; empty = go).

    snap keys: link_age_s, bridge_link, fault_state, sched_hold_latched,
    lane_recovery, mode, traj_pids (list), have_skill_node, pose_known,
    mm_to_rev_ok(optional)."""
    bad = []
    age = snap.get('link_age_s')
    if age is None or age > 1.0:
        bad.append('/link_status not fresh (age %s s)' % (
            'none' if age is None else '%.1f' % age))
    if snap.get('bridge_link') in (None, '', 'LOST', 'NO_HEARTBEAT'):
        bad.append('bridge link not UP (bridge_link=%r)' % (snap.get('bridge_link'),))
    if snap.get('fault_state') != 'NONE':
        bad.append('fault_state is %r, need NONE' % (snap.get('fault_state'),))
    if str(snap.get('sched_hold_latched')) != '0':
        bad.append('sched_hold_latched is %r, need 0 (a lane is ALREADY latched)'
                   % (snap.get('sched_hold_latched'),))
    if snap.get('lane_recovery') != 'idle':
        bad.append('lane_recovery is %r, need idle' % (snap.get('lane_recovery'),))
    if snap.get('mode') != 'TRAJECTORY':
        bad.append('trajectory_node mode is %r, need TRAJECTORY' % (snap.get('mode'),))
    n = len(snap.get('traj_pids') or [])
    if n != 1:
        bad.append('found %d trajectory_node processes, need exactly 1 '
                   '(its pid is what gets frozen)' % n)
    if not snap.get('have_skill_node'):
        bad.append('skill_node is not in the ROS graph: T2 lives there, so nothing '
                   'would trigger the recovery')
    if not snap.get('pose_known'):
        bad.append('current platform pose unknown (/robot_state stale or FK failed)')
    if not args.no_freeze and peak_rps is not None \
            and max(peak_rps) <= RESUME_TOL_VEL_LEG_RPS:
        bad.append('max leg peak %.2f rps <= %.1f rps resume tolerance: the move is '
                   'too gentle to latch; raise --dx-mm or shorten --move-s'
                   % (max(peak_rps), RESUME_TOL_VEL_LEG_RPS))
    return bad


def parse_live_dev(text):
    try:
        vals = [abs(float(x)) for x in str(text).split(',') if x.strip()]
    except ValueError:
        return None
    return max(vals) if vals else None


def _row(name, status, evidence):
    return {'expectation': name, 'status': status, 'evidence': evidence}


LIVE_DEV_PEAK_NAME = ('peak live_deviation while latched (remaining travel on '
                      'the refused stream)')
LIVE_DEV_RECOVERY_NAME = ('live_deviation <= %.2f rev through the recovery '
                          '(converge done -> DONE)' % MAX_LIVE_DEV_REV)


def _recovery_start(samples):
    """Index of the first in_progress sample whose live_dev < 0.05 rev (converge
    done) or whose stage is past converge; None when there is no such sample."""
    for k, s in enumerate(samples):
        if s.get('lane_recovery') != 'in_progress':
            continue
        d = s.get('live_dev')
        past = s.get('stage') in STAGES and s['stage'] != 'converge'
        if past or (d is not None and d < CONVERGED_DEV_REV):
            return k
    return None


LIVE_DEV_OBS_NAME = ('live_deviation <= %.2f rev (observation pass, no latch)'
                     % MAX_LIVE_DEV_REV)


def _observation_dev_row(samples):
    win = [s for s in samples if s.get('live_dev') is not None]
    if not win:
        return _row(LIVE_DEV_OBS_NAME, 'FAIL', 'no live_deviation samples')
    pk = max(win, key=lambda s: s['live_dev'])
    return _row(LIVE_DEV_OBS_NAME,
                'PASS' if pk['live_dev'] <= MAX_LIVE_DEV_REV else 'FAIL',
                'max %.4f rev over %d samples' % (pk['live_dev'], len(win)))


def _peak_dev_row(samples):
    """While latched the Jetson keeps emitting the quintic to a lane the firmware
    holds still, so deviation = remaining travel (grew to 0.158 rev on 2026-10-14)
    -- expected, not a fault; only a value near the bridge escape matters."""
    if not any(s.get('bit15') for s in samples):
        return _row(LIVE_DEV_PEAK_NAME, 'SKIPPED', 'no latched samples')
    start = _recovery_start(samples)
    pre = samples if start is None else samples[:start]
    lat = [s for s in pre if s.get('live_dev') is not None]
    if not lat:
        return _row(LIVE_DEV_PEAK_NAME, 'SKIPPED', 'no live_deviation samples')
    pk = max(lat, key=lambda s: s['live_dev'])
    return _row(LIVE_DEV_PEAK_NAME,
                'FAIL' if pk['live_dev'] >= LANE_ESCAPE_DEV_REV else 'INFO',
                'max %.4f rev at t=%.2f; bridge escape %.1f, firmware trip %.1f'
                % (pk['live_dev'], pk['t'], LANE_ESCAPE_DEV_REV, MAX_DEVIATION_REV))


def _recovery_dev_row(samples):
    """Window: first in_progress sample whose live_dev < 0.05 rev (converge
    done) or whose stage is past converge, to the last 'done' sample."""
    start = _recovery_start(samples)
    if start is None:
        return _row(LIVE_DEV_RECOVERY_NAME, 'SKIPPED', 'no recovery window (no latch)')
    end = max((k for k, s in enumerate(samples) if s.get('lane_recovery') == 'done'),
              default=len(samples) - 1)
    win = [s for s in samples[start:end + 1] if s.get('live_dev') is not None]
    if not win:
        return _row(LIVE_DEV_RECOVERY_NAME, 'FAIL', 'no live_deviation samples')
    pk = max(win, key=lambda s: s['live_dev'])
    return _row(LIVE_DEV_RECOVERY_NAME,
                'PASS' if pk['live_dev'] <= MAX_LIVE_DEV_REV else 'FAIL',
                'max %.4f rev at t=%.2f over %d samples (t=%.2f..%.2f)'
                % (pk['live_dev'], pk['t'], len(win), samples[start]['t'],
                   samples[end]['t']))


def build_verdict(samples, run):
    """Every expectation as a row. samples: time-ordered dicts {t, bit15 (0/1),
    lane_recovery, stage, live_dev (float|None), fault_state, sched_refused (int)}.
    run: {no_freeze, move_accepted, move_msg, freeze_actual_ms, home_accepted,
    home_msg, t_freeze_end}."""
    rows = []
    no_freeze = run.get('no_freeze', False)
    refused = not run.get('move_accepted')
    ev = run.get('move_msg', '')
    m_min = re.search(r'minimal feasible (\d+\.\d+)\s*s', ev)
    if refused and m_min:
        ev += ' -- re-run with --move-s >= %.2f' % (float(m_min.group(1)) + 0.2)
    rows.append(_row('go_to_pose accepted', 'FAIL' if refused else 'PASS', ev))
    if refused:
        # Nothing moved, so nothing was frozen and nothing can latch: every
        # downstream row is SKIPPED (not "freeze did not latch").
        why = 'move refused'
        for nm in ('freeze bounded (CONT sent)', 'bit 15 (sched_hold_latched) rose',
                   'recovery in_progress within %.1f s' % IN_PROGRESS_WITHIN_S,
                   'stages in order', 'recovery done within %.0f s' % DONE_WITHIN_S,
                   'bit 15 cleared at the end', LIVE_DEV_PEAK_NAME,
                   LIVE_DEV_RECOVERY_NAME):
            rows.append(_row(nm, 'SKIPPED', why))
        faults = sorted({s.get('fault_state') for s in samples} - {'NONE'})
        rows.append(_row('fault_state unchanged (NONE throughout)',
                         'FAIL' if faults else 'PASS',
                         ('left NONE: %s -- %s' % (','.join(map(str, faults)),
                                                   FAULT_TEXT)) if faults
                         else 'NONE in all %d samples' % len(samples)))
        rows.append(_row('go_home accepted afterwards', 'SKIPPED', 'not attempted'))
        return rows
    fa = run.get('freeze_actual_ms')
    if no_freeze:
        rows.append(_row('freeze bounded (CONT sent)', 'SKIPPED', '--no-freeze'))
    elif fa is None:
        rows.append(_row('freeze bounded (CONT sent)', 'FAIL', 'no freeze recorded'))
    else:
        rows.append(_row('freeze bounded (CONT sent)',
                         'PASS' if fa < FREEZE_MAX_MS else 'FAIL',
                         'STOP->CONT %.0f ms (cover %.0f, STALE %.0f)'
                         % (fa, COVER_MS, SETPOINT_STALE_S * 1e3)))

    rise = next((s for s in samples if s.get('bit15')), None)
    faults = sorted({s.get('fault_state') for s in samples} - {'NONE'})
    if no_freeze:
        rows.append(_row('no spurious latch (observation pass)',
                         'FAIL' if rise else 'PASS',
                         'bit 15 rose at t=%.2f' % rise['t'] if rise
                         else 'bit 15 stayed clear for %d samples' % len(samples)))
        for nm in ('recovery in_progress within %.1f s' % IN_PROGRESS_WITHIN_S,
                   'stages in order', 'recovery done within %.0f s' % DONE_WITHIN_S,
                   'bit 15 cleared at the end'):
            rows.append(_row(nm, 'SKIPPED', '--no-freeze'))
    elif rise is None:
        rows.append(_row('bit 15 (sched_hold_latched) rose', 'FAIL', NO_LATCH_TEXT))
        for nm in ('recovery in_progress within %.1f s' % IN_PROGRESS_WITHIN_S,
                   'stages in order', 'recovery done within %.0f s' % DONE_WITHIN_S,
                   'bit 15 cleared at the end'):
            rows.append(_row(nm, 'SKIPPED', 'no latch to recover from'))
    else:
        t0 = rise['t']
        rows.append(_row('bit 15 (sched_hold_latched) rose', 'PASS', 't=%.2f s' % t0))
        prog = next((s for s in samples if s['t'] >= t0
                     and s.get('lane_recovery') == 'in_progress'), None)
        if prog is None:
            rows.append(_row('recovery in_progress within %.1f s' % IN_PROGRESS_WITHIN_S,
                             'FAIL', 'lane_recovery never left %r (T2 did not fire? '
                             'skill_node up, executor idle?)'
                             % (samples[-1].get('lane_recovery'),)))
        else:
            dt = prog['t'] - t0
            rows.append(_row('recovery in_progress within %.1f s' % IN_PROGRESS_WITHIN_S,
                             'PASS' if dt <= IN_PROGRESS_WITHIN_S else 'FAIL',
                             '+%.2f s after the latch' % dt))
        seq = []
        for s in samples:
            st = s.get('stage')
            if st in STAGES and (not seq or seq[-1] != st):
                seq.append(st)
        idx = [STAGES.index(x) for x in seq]
        ordered = bool(seq) and idx == sorted(set(idx))
        skipped = [x for x in STAGES if x not in seq]
        ok_ends = bool(seq) and seq[0] == 'converge' and seq[-1] == 'confirm'
        rows.append(_row('stages in order (converge>rest>disarm>arm>confirm)',
                         'PASS' if ordered and ok_ends else 'FAIL',
                         'observed %s%s' % ('>'.join(seq) or 'none',
                                            ('; not observed (10 Hz sampling or '
                                             'short-circuit): %s' % ','.join(skipped))
                                            if skipped and seq else '')))
        fin = next((s for s in samples if s['t'] >= t0
                    and s.get('lane_recovery') in ('done', 'failed')), None)
        if fin is None:
            rows.append(_row('recovery done within %.0f s' % DONE_WITHIN_S, 'FAIL',
                             'no done/failed within the watch window'))
        elif fin['lane_recovery'] == 'failed':
            rows.append(_row('recovery done within %.0f s' % DONE_WITHIN_S, 'FAIL',
                             'lane_recovery=failed at stage %s (+%.2f s); fallback: '
                             'DEACTIVATE then ACTIVATE' % (fin.get('stage'),
                                                           fin['t'] - t0)))
        else:
            dt = fin['t'] - t0
            rows.append(_row('recovery done within %.0f s' % DONE_WITHIN_S,
                             'PASS' if dt <= DONE_WITHIN_S else 'FAIL',
                             'done +%.2f s after the latch' % dt))
        last = samples[-1]
        rows.append(_row('bit 15 cleared at the end',
                         'FAIL' if last.get('bit15') else 'PASS',
                         'final sched_hold_latched=%d' % int(last.get('bit15', 0))))

    if no_freeze:
        rows.append(_observation_dev_row(samples))
    else:
        rows.append(_peak_dev_row(samples))
        rows.append(_recovery_dev_row(samples))
    rows.append(_row('fault_state unchanged (NONE throughout)',
                     'FAIL' if faults else 'PASS',
                     ('left NONE: %s -- %s' % (','.join(map(str, faults)), FAULT_TEXT))
                     if faults else 'NONE in all %d samples' % len(samples)))
    sr = [s.get('sched_refused') for s in samples if s.get('sched_refused') is not None]
    if sr:
        rows.append(_row('sched_refused (info)', 'INFO',
                         '%d -> %d' % (sr[0], sr[-1])))
    if run.get('home_accepted') is None:
        rows.append(_row('go_home accepted afterwards', 'SKIPPED', 'not attempted'))
    else:
        rows.append(_row('go_home accepted afterwards',
                         'PASS' if run['home_accepted'] else 'FAIL',
                         run.get('home_msg', '')))
    return rows


def exit_code(rows):
    return 0 if all(r['status'] in ('PASS', 'SKIPPED', 'INFO') for r in rows) \
        and any(r['status'] == 'PASS' for r in rows) else 1


def format_table(rows):
    w = max(len(r['expectation']) for r in rows)
    out = ['%-*s  %-7s  %s' % (w, 'EXPECTATION', 'STATUS', 'EVIDENCE')]
    out += ['%-*s  %-7s  %s' % (w, r['expectation'], r['status'], r['evidence'])
            for r in rows]
    return '\n'.join(out)


# ============================================================ ROS / OS layer
def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt('signal %d during the freeze' % signum)


def find_trajectory_pids(self_pid=None):
    """pids whose argv has an element with basename trajectory_node[.py]."""
    self_pid = self_pid or os.getpid()
    pids = []
    for d in os.listdir('/proc'):
        if not d.isdigit() or int(d) == self_pid:
            continue
        try:
            with open('/proc/%s/cmdline' % d, 'rb') as f:
                argv = f.read().decode('utf-8', 'replace').split('\0')
        except OSError:
            continue
        if any(os.path.basename(a) in ('trajectory_node', 'trajectory_node.py')
               for a in argv if a):
            pids.append(int(d))
    return sorted(pids)


def _kv(values, key, default=''):
    for kv in values:
        if kv.key == key:
            return kv.value
    return default


def run_live(args):                                        # pragma: no cover
    import rclpy
    import threading
    import numpy as np
    from diagnostic_msgs.msg import DiagnosticStatus
    from geometry_msgs.msg import Pose
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Trigger
    from jugglebot_interfaces.msg import RobotState, TrajectoryStatus
    from jugglebot_interfaces.srv import GoToPose
    sys.path.insert(0, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'))
    from jugglebot.motion.geometry import StewartGeometry
    from jugglebot.motion.ik_solver import (leg_lengths_to_pose,
                                            rot_matrix_to_quat)

    rclpy.init()
    node = rclpy.create_node('lane_latch_rehearsal')
    lock = threading.Lock()
    st = {'link': None, 'link_t': None, 'mode': None, 'pos_rev': None,
          'pos_t': None, 'samples': [], 'rec': False}
    t_origin = time.monotonic()

    def on_link(msg):
        v = msg.values
        now = time.monotonic()
        with lock:
            st['link'] = {
                'bridge_link': _kv(v, 'bridge_link'),
                'fault_state': _kv(v, 'fault_state', 'UNKNOWN'),
                'sched_hold_latched': _kv(v, 'sched_hold_latched', '?'),
                'lane_recovery': _kv(v, 'lane_recovery', 'idle'),
                'stage': _kv(v, 'lane_recovery_stage', 'none')}
            st['link_t'] = now
            if st['rec']:
                try:
                    sref = int(_kv(v, 'sched_refused', '0'))
                except ValueError:
                    sref = None
                st['samples'].append({
                    't': now - t_origin,
                    'bit15': 1 if _kv(v, 'sched_hold_latched') == '1' else 0,
                    'lane_recovery': _kv(v, 'lane_recovery', 'idle'),
                    'stage': _kv(v, 'lane_recovery_stage', 'none'),
                    'live_dev': parse_live_dev(_kv(v, 'live_deviation')),
                    'fault_state': _kv(v, 'fault_state', 'UNKNOWN'),
                    'sched_refused': sref})

    def on_mode(msg):
        with lock:
            st['mode'] = msg.mode

    def on_robot(msg):
        s = getattr(msg, 'motor_states', None)
        if s and len(s) >= 6:
            with lock:
                st['pos_rev'] = [float(s[i].pos_estimate) for i in range(6)]
                st['pos_t'] = time.monotonic()

    node.create_subscription(DiagnosticStatus, 'link_status', on_link, 10)
    node.create_subscription(TrajectoryStatus, 'trajectory/status', on_mode, 10)
    node.create_subscription(RobotState, 'robot_state', on_robot, 10)
    go_cli = node.create_client(GoToPose, 'trajectory/go_to_pose')
    home_cli = node.create_client(Trigger, 'trajectory/go_home')
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(1.5)                                        # let the topics arrive

    geom = StewartGeometry()
    pose = None
    with lock:
        pos_rev, pos_t = st['pos_rev'], st['pos_t']
    if pos_rev is not None and time.monotonic() - pos_t < 1.0:
        try:
            p, rot, _ = leg_lengths_to_pose(
                np.asarray(pos_rev) / np.asarray(geom.mm_to_rev), geom)
            pose = (np.asarray(p, dtype=float), rot)
        except Exception:                                  # noqa: BLE001
            pose = None
    with lock:
        link, link_t, mode = st['link'], st['link_t'], st['mode']
    names = [n for n in node.get_node_names()]
    pids = find_trajectory_pids()
    snap = {'link_age_s': None if link_t is None else time.monotonic() - link_t,
            'mode': mode, 'traj_pids': pids,
            'have_skill_node': 'skill_node' in names, 'pose_known': pose is not None}
    snap.update(link or {})
    peak = leg_peak_rps(args.dx_mm, args.move_s)
    bad = preflight(snap, args, peak)
    print('pre-flight: leg peak rps at dx=%.0f mm / %.2f s: %s (resume tol %.1f)'
          % (args.dx_mm, args.move_s, ' '.join('%.2f' % x for x in peak),
             RESUME_TOL_VEL_LEG_RPS))
    if bad:
        print('REFUSED (%d):' % len(bad))
        for b in bad:
            print('  - ' + b)
        os._exit(2)

    pid = pids[0]
    run = {'no_freeze': args.no_freeze, 'freeze_actual_ms': None,
           'home_accepted': None, 'move_accepted': False, 'move_msg': ''}
    req = GoToPose.Request()
    pos, rot = pose
    req.pose = Pose()
    req.pose.position.x = float(pos[0]) + args.dx_mm
    req.pose.position.y = float(pos[1])
    req.pose.position.z = float(pos[2])
    w, x, y, z = rot_matrix_to_quat(rot)
    req.pose.orientation.w, req.pose.orientation.x = float(w), float(x)
    req.pose.orientation.y, req.pose.orientation.z = float(y), float(z)
    req.duration_s = float(args.move_s)

    def call(cli, rq, timeout):
        fut = cli.call_async(rq)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.005)
        return fut.result() if fut.done() else None

    with lock:
        st['rec'] = True
    stopped = False
    try:
        if not go_cli.wait_for_service(timeout_sec=2.0):
            run['move_msg'] = 'trajectory/go_to_pose unavailable'
        else:
            rsp = call(go_cli, req, 3.0)
            if rsp is None:
                run['move_msg'] = 'go_to_pose: no response in 3 s'
            else:
                run['move_accepted'] = bool(rsp.accepted)
                run['move_msg'] = '%s %s (planned %.2f s)' % (
                    rsp.code, rsp.message, rsp.planned_duration_s)
        if run['move_accepted'] and not args.no_freeze:
            time.sleep(args.at_ms / 1e3)
            t_stop = time.monotonic()
            # SIGTERM's default action skips `finally`; turn it into an exception
            # so a kill during the freeze still SIGCONTs.
            prev_term = signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
            try:
                os.kill(pid, signal.SIGSTOP)
                stopped = True
                time.sleep(args.freeze_ms / 1e3)
            finally:
                if stopped:
                    os.kill(pid, signal.SIGCONT)
                    stopped = False
                    run['freeze_actual_ms'] = (time.monotonic() - t_stop) * 1e3
                signal.signal(signal.SIGTERM, prev_term)
        elif run['move_accepted']:
            time.sleep(args.move_s)
        if run['move_accepted']:
            t_w = time.monotonic()
            settled = None
            while time.monotonic() - t_w < WATCH_S:
                time.sleep(0.1)
                with lock:
                    last = st['samples'][-1] if st['samples'] else None
                if last and last['lane_recovery'] in ('done', 'failed'):
                    settled = settled or time.monotonic()
                if args.no_freeze and time.monotonic() - t_w > 2.0 + args.move_s:
                    break
                if settled and time.monotonic() - settled > 2.0:
                    break
                if (last and not args.no_freeze and not last['bit15']
                        and last['lane_recovery'] == 'idle'
                        and time.monotonic() - t_w > 3.0
                        and not any(s['bit15'] for s in st['samples'])):
                    break                                   # never latched
            rsp = call(home_cli, Trigger.Request(), 5.0)
            if rsp is None:
                run['home_accepted'], run['home_msg'] = False, 'no response'
            else:
                run['home_accepted'] = bool(rsp.success)
                run['home_msg'] = rsp.message
            time.sleep(1.0)
    finally:
        if stopped:
            os.kill(pid, signal.SIGCONT)
        with lock:
            st['rec'] = False

    with lock:
        samples = list(st['samples'])
    if not samples:
        samples = [{'t': 0.0, 'bit15': 0, 'lane_recovery': 'idle', 'stage': 'none',
                    'live_dev': None, 'fault_state': 'UNKNOWN',
                    'sched_refused': None}]
    rows = build_verdict(samples, run)
    print(format_table(rows))
    out = args.out or os.path.join(
        _REPO, 'temp', 'probes',
        'lane_latch_rehearsal_%s.json' % time.strftime('%Y%m%d_%H%M%S'))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w') as f:
        json.dump({'args': vars(args), 'run': run, 'rows': rows,
                   'samples': samples}, f, indent=1)
    print('record: ' + out)
    code = exit_code(rows)
    sys.stdout.flush()
    os._exit(code)                      # daemon spin thread would trace on shutdown


def main(argv=None):
    args = parse_args(argv)
    bad = check_args(args)
    if bad:
        print('REFUSED (%d):' % len(bad))
        for b in bad:
            print('  - ' + b)
        return 2
    return run_live(args)


if __name__ == '__main__':
    sys.exit(main())
