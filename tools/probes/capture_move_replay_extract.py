"""Extract the capture-move replay numbers for the 2026-10-19 refused reload catches.

Unit 0 of the capture-move fix (design: held-axis CATCH takes a lateral re-aim by
a planned capture move inside the window). For each ball id named on the command
line it pulls, from an MCAP bag plus the matching launch.log, the inputs the
CATCH_AXIS refusal (or acceptance) was computed from:

  * the ThrowAnnouncement the reload schedule was compiled from
    (/throw_announcements; WORLD frame, mm, mm/s),
  * the tracker's first and last converged fit (/balls, landing_from_fit) at or
    before the CATCH dispatch (WORLD frame),
  * the commanded platform state around the dispatch: xyz from
    /trajectory/commanded_position (schedule/commanded frame), tilt from forward
    kinematics of /leg_setpoint_echo (the legs' physical commanded pose, so it
    carries the gravity-levelling correction; its level-rest baseline is reported
    so the hold tilt can be read relative to it),
  * the launch.log lines: BB aim, hold tilt, robot-frame snapshot, mocap->schedule
    correction, CATCH dispatch / refusal / AIM-LATERAL-CLAMPED notes.

Run (venv; no ROS needed - mcap_ros2 decodes from the schema the bag carries):

  python -I tools/probes/capture_move_replay_extract.py \
      --bag ~/Desktop/rosbags/2026-10-19_11-04-03 \
      --launch-log ~/.ros/log/2026-10-19-11-04-03-078330-jetson-3068207/launch.log \
      --balls 10,17,18

Add ``--emit-fixture tests/fixtures/capture_move_replay_2026-10-19.yaml`` to also write the
commented test fixture. Writes <out>/capture_move_replay.json (default out = temp/probes/capture_move_replay/<ts>/)
and prints a summary table. ~1 min for the 2026-10-19 bag.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import time

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'ros_ws', 'src', 'jugglebot'))

TOPICS = ['/throw_announcements', '/balls', '/trajectory/commanded_position',
          '/leg_setpoint_echo']
NEEDLE = re.compile(r'^(\d+\.\d+) \[(\S+?)-\d+\] \[(\w+)\] \[[\d.]+\] \[(\w+)\]: (.*)$')


def _t(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def read_bag(bag_dir, t_lo, t_hi):
    """Decode only [t_lo, t_hi] (log_time) of the topics we need."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    path = [os.path.join(bag_dir, f) for f in sorted(os.listdir(bag_dir))
            if f.endswith('.mcap')][0]
    out = {k: [] for k in TOPICS}
    with open(path, 'rb') as f:
        rd = make_reader(f, decoder_factories=[DecoderFactory()])
        for sch, ch, msg, dec in rd.iter_decoded_messages(
                topics=TOPICS, start_time=int(t_lo * 1e9), end_time=int(t_hi * 1e9)):
            t = msg.log_time * 1e-9
            if ch.topic == '/throw_announcements':
                out[ch.topic].append((t, dec))
            elif ch.topic == '/balls':
                for b in dec.balls:
                    out[ch.topic].append((t, b))
            elif ch.topic == '/trajectory/commanded_position':
                out[ch.topic].append((t, (dec.x, dec.y, dec.z)))
            else:
                out[ch.topic].append((t, list(dec.data)))
    return out


def parse_log(path):
    rows = []
    with open(path, errors='replace') as f:
        for i, line in enumerate(f, 1):
            m = NEEDLE.match(line.rstrip('\n'))
            if m:
                rows.append((i, float(m.group(1)), m.group(2), m.group(5)))
    return rows


def log_for_ball(rows, ball, accept_window=40.0):
    """The log facts of one reload attempt, keyed off 'Ball <id> announced'."""
    k = [r for r in rows if re.search(r'Ball %d announced by' % ball, r[3])]
    if not k:
        raise SystemExit('ball %d: no "announced" line in the launch log' % ball)
    i0, t0 = k[0][0], k[0][1]
    win = [r for r in rows if t0 - accept_window <= r[1] <= t0 + 12.0]
    pick = lambda pat, lo=-1e9, hi=1e9: [r for r in win if lo <= r[1] - t0 <= hi
                                          and re.search(pat, r[3])]
    d = {'announced_line': i0, 'announced_log_t': t0}
    r = pick(r'BB aim: robot frame', -3, 3)
    if r:
        m = re.search(r'robot frame \(([-\d.]+), ([-\d.]+), ([-\d.]+)\) mm -> world '
                      r'\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)', r[0][3])
        d['bb_aim_robot_mm'] = [float(v) for v in m.groups()[:3]]
        d['bb_aim_world_mm'] = [float(v) for v in m.groups()[3:]]
        d['bb_aim_line'] = r[0][0]
    r = pick(r'resulting hold', -3, 3)
    if r:
        d['hold_tilt_deg'] = float(re.search(r'resulting hold ([\d.]+) deg', r[0][3]).group(1))
        d['hold_tilt_line'] = r[0][0]
    r = pick(r'robot frame snapshotted', -30, 3)
    if r:
        d['robot_frame_snapshot'] = r[-1][3]
        d['robot_frame_snapshot_line'] = r[-1][0]
    r = pick(r'tracker landings are now corrected by', -60, 3)
    if r:
        m = re.search(r'corrected by \(x ([-+\d.]+), y ([-+\d.]+)\)', r[-1][3])
        d['mocap_to_schedule_correction_mm'] = [float(m.group(1)), float(m.group(2))]
        d['mocap_to_schedule_correction_line'] = r[-1][0]
    r = pick(r'CATCH skill 1: fresh origin', 0, 8)
    r2 = pick(r'END CATCH_AXIS at skill 1', 0, 8)
    r3 = pick(r'CATCH-AIM skill 1', 0, 8)
    if r3:
        m = re.search(r'source=(\w+) landing=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\) mm t_land=([\d.]+)', r3[0][3])
        d['catch_aim'] = {'source': m.group(1), 'landing_mm': [float(m.group(i)) for i in (2, 3, 4)],
                          't_land': float(m.group(5)), 'line': r3[0][0]}
    if r2:
        d['dispatch_t'] = float(re.match(r'(\d+\.\d+) END', r2[0][3]).group(1))
        d['dispatch_line'] = r2[0][0]
        d['outcome'] = 'END CATCH_AXIS (fresh-origin dispatch refused)'
    elif r:
        d['dispatch_t'] = float(re.match(r'(\d+\.\d+) CATCH', r[0][3]).group(1))
        d['dispatch_line'] = r[0][0]
        d['outcome'] = 'installed'
    ref = pick(r'install_segment refused: .*CATCH_AXIS', 0, 8)
    if ref:
        d['refusal_mm'] = float(re.search(r'is ([\d.]+) mm off the line', ref[0][3]).group(1))
        d['refusal_line'] = ref[0][0]
    cl = pick(r'AIM-LATERAL-CLAMPED', 0, 8)
    if cl:
        d['aim_lateral_clamped'] = cl[0][3]
        d['aim_lateral_clamped_line'] = cl[0][0]
    rs = pick(r'RESEND-REFUSED CATCH_AXIS', 0, 8)
    if rs:
        d['resend_refused'] = rs[0][3][:160]
        d['resend_t'] = float(re.match(r'(\d+\.\d+) RESEND', rs[0][3]).group(1))
    return d


def _ann_dict(t, a):
    return {'bag_t': t, 'header_stamp': _t(a.header.stamp), 'thrower': a.thrower_name,
            'initial_position_mm': [a.initial_position.x, a.initial_position.y, a.initial_position.z],
            'initial_velocity_mmps': [a.initial_velocity.x, a.initial_velocity.y, a.initial_velocity.z],
            'throw_time': _t(a.throw_time), 'predicted_tof_s': a.predicted_tof_sec,
            'landing_position_mm': [a.landing_position.x, a.landing_position.y, a.landing_position.z],
            'landing_velocity_mmps': [a.landing_velocity.x, a.landing_velocity.y, a.landing_velocity.z],
            'landing_time': _t(a.landing_time)}


def _fit_dict(t, b):
    return {'bag_t': t, 'header_stamp': _t(b.header.stamp), 'ball_id': b.id,
            'landing_position_mm': [b.landing_position.x, b.landing_position.y, b.landing_position.z],
            'landing_velocity_mmps': [b.landing_velocity.x, b.landing_velocity.y, b.landing_velocity.z],
            'time_at_land': _t(b.time_at_land), 'landing_from_fit': bool(b.landing_from_fit)}


def echo_pose(u0):
    """Commanded platform pose (mm, tilt deg + direction deg) from one /leg_setpoint_echo row."""
    from jugglebot.motion.geometry import StewartGeometry
    from jugglebot.motion.ik_solver import leg_lengths_to_pose
    geom = StewartGeometry()
    ext = np.asarray(u0, float) / geom.mm_to_rev
    pos, rot, _ = leg_lengths_to_pose(ext, geom)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, rot[2, 2]))))
    # cup axis (body z in world) lateral components: direction of the tilt
    ax = rot[:, 2]
    return {'xyz_mm': [float(v) for v in pos], 'tilt_deg': tilt,
            'axis_xy': [float(ax[0]), float(ax[1])]}


#: the executor's lateral aim authority on a one-site/reload catch (mm per axis);
#: AIM-LATERAL-CLAMPED in the launch log names it. Mirrors
#: ``skill_node`` ``learner_lateral_authority_mm`` (default 20).
AIM_AUTHORITY_MM = 20.0


def reproduce(rec):
    """|refined - announced| lateral, raw and per-axis clamped, in the bag's (world) frame.

    The refusal is the clamped re-aim: ``hypot(clip(dx, +-20), clip(dy, +-20))``
    (the seed sits on the announced line, the CATCH site is the clamped fit).
    """
    ann = np.asarray(rec['announcement']['landing_position_mm'][:2])
    out = {}
    for key in ('tracker_fit_last_before_dispatch', 'tracker_fit_last_before_resend'):
        f = rec.get(key)
        if not f:
            continue
        d = np.asarray(f['landing_position_mm'][:2]) - ann
        c = np.clip(d, -AIM_AUTHORITY_MM, AIM_AUTHORITY_MM)
        out[key] = {'delta_mm': [float(v) for v in d], 'delta_norm_mm': float(np.hypot(*d)),
                    'clamped_delta_mm': [float(v) for v in c],
                    'clamped_norm_mm': float(np.hypot(*c))}
    out['logged_refusal_mm'] = rec['launch_log'].get('refusal_mm')
    return out


def _v(xs, nd=4):
    return '[' + ', '.join(('%.*f' % (nd, x)) for x in xs) + ']'


def emit_fixture(result, path, bag, launch_log):
    """Commented YAML fixture, one record per ball (consumed by the capture-move replay tests)."""
    L = []
    w = L.append
    w('# capture_move_replay_2026-10-19 - replay inputs for the held-axis reload CATCH refusals')
    w('# Generated by tools/probes/capture_move_replay_extract.py; see capture_move_replay/README.md.')
    w('# Frames: ANNOUNCEMENT and TRACKER values are the bag\'s WORLD (mocap) frame, mm; the skill_node')
    w('# converts them to the schedule frame with the robot-frame snapshot (near identity, below).')
    w('# Commanded xyz is the trajectory/schedule frame. All times are epoch seconds on the bag/log clock.')
    w('bag: %s' % os.path.basename(os.path.normpath(bag)))
    w('launch_log: %s' % launch_log.replace(os.path.expanduser('~'), '~'))
    w('installed_commit: 84519014   # Jugglebot-skills install, built 10:35; see README')
    w('aim_authority_mm: %.1f   # executor per-axis lateral aim clamp (AIM-LATERAL-CLAMPED)' % AIM_AUTHORITY_MM)
    w('balls:')
    for b, r in result.items():
        Lg, an = r['launch_log'], r['announcement']
        w('  ball_%s:' % b)
        w('    ball_id: %s' % b)
        w('    role: %s' % ('control (reload CATCH installed from the schedule aim, caught)' if Lg['outcome'] == 'installed'
                           else 'refused (fresh-origin CATCH dispatch -> CATCH_AXIS -> END)'))
        w('    # --- ThrowAnnouncement: /throw_announcements, bag log_time %.4f, header.stamp %.4f' % (an['bag_t'], an['header_stamp']))
        w('    announcement:')
        w('      landing_mm: %s          # landing_position, world frame (the schedule\'s announced landing)' % _v(an['landing_position_mm']))
        w('      landing_velocity_mmps: %s   # arrival velocity at the landing plane' % _v(an['landing_velocity_mmps']))
        w('      initial_position_mm: %s' % _v(an['initial_position_mm']))
        w('      initial_velocity_mmps: %s' % _v(an['initial_velocity_mmps']))
        w('      throw_time: %.4f          # release instant' % an['throw_time'])
        w('      predicted_tof_s: %.5f' % an['predicted_tof_s'])
        w('      landing_time: %.4f' % an['landing_time'])
        for key, title in (('tracker_fit_first', 'first converged fit'),
                           ('tracker_fit_last_before_dispatch', 'last converged fit with log_time <= dispatch (what a dispatch-time read sees)'),
                           ('tracker_fit_first_after_dispatch', 'first fit after the dispatch'),
                           ('tracker_fit_last_before_resend', 'last fit before the re-send (ball 10 only)')):
            f = r.get(key)
            if not f:
                continue
            w('    # --- /balls ball id %d, landing_from_fit=true, bag log_time %.4f, header.stamp %.4f: %s' % (
                f['ball_id'], f['bag_t'], f['header_stamp'], title))
            w('    %s:' % key)
            w('      landing_mm: %s         # world frame' % _v(f['landing_position_mm']))
            w('      landing_velocity_mmps: %s' % _v(f['landing_velocity_mmps']))
            w('      time_at_land: %.4f' % f['time_at_land'])
        w('    # --- skill_node launch.log (line numbers are in the file named above)')
        w('    dispatch_t: %.3f          # CATCH skill 1 dispatch, log line %d' % (Lg['dispatch_t'], Lg['dispatch_line']))
        w('    hold_tilt_deg: %.3f       # "resulting hold", log line %d' % (Lg['hold_tilt_deg'], Lg['hold_tilt_line']))
        w('    bb_aim_robot_mm: %s    # "BB aim: robot frame", log line %d' % (_v(Lg['bb_aim_robot_mm'], 1), Lg['bb_aim_line']))
        w('    bb_aim_world_mm: %s' % _v(Lg['bb_aim_world_mm'], 1))
        w('    robot_frame_snapshot: "%s"   # log line %d' % (Lg['robot_frame_snapshot'].split(': ', 1)[1], Lg['robot_frame_snapshot_line']))
        w('    mocap_to_schedule_correction_mm: %s   # "tracker landings are now corrected by", log line %d' % (
            _v(Lg['mocap_to_schedule_correction_mm'], 1), Lg['mocap_to_schedule_correction_line']))
        if 'catch_aim' in Lg:
            ca = Lg['catch_aim']
            w('    catch_aim: {source: %s, landing_mm: %s, t_land: %.3f}   # CATCH-AIM, log line %d (schedule frame)' % (
                ca['source'], _v(ca['landing_mm'], 1), ca['t_land'], ca['line']))
        else:
            w('    catch_aim: null         # dispatch REFUSED before a CATCH-AIM line was logged; aim was the clamped tracker fit')
        if 'aim_lateral_clamped' in Lg:
            w('    aim_lateral_clamped: "%s"   # log line %d' % (Lg['aim_lateral_clamped'], Lg['aim_lateral_clamped_line']))
        w('    logged_catch_axis_refusal_mm: %.1f   # trajectory_node install_segment refused, log line %d%s' % (
            Lg['refusal_mm'], Lg['refusal_line'],
            '  (RESEND refusal; the schedule-aimed CATCH stayed installed)' if Lg['outcome'] == 'installed' else '  (dispatch refusal -> END)'))
        c = r['commanded_position_at_dispatch']
        w('    # --- commanded platform at the dispatch knot (the pre-tilt REST terminal, schedule frame)')
        w('    commanded_xyz_mm: %s   # /trajectory/commanded_position, last msg <= dispatch, bag log_time %.4f' % (_v(c['xyz_mm']), c['bag_t']))
        k = r['commanded_pose_fk_at_dispatch']
        w('    commanded_fk: {xyz_mm: %s, tilt_deg: %.3f, axis_xy: %s}   # FK of /leg_setpoint_echo at bag log_time %.4f (the legs\' physical command: includes the %.2f deg level baseline, so tilt - baseline ~ hold_tilt_deg)' % (
            _v(k['xyz_mm']), k['tilt_deg'], _v(k['axis_xy'], 5), k['bag_t'], r['commanded_pose_fk_level_baseline']['tilt_deg']))
        w('    commanded_fk_level_baseline_tilt_deg: %.3f   # FK of /leg_setpoint_echo at the level rest, bag log_time %.4f' % (
            r['commanded_pose_fk_level_baseline']['tilt_deg'], r['commanded_pose_fk_level_baseline']['bag_t']))
        rp = r['reproduction']
        w('    # --- reproduction (script): clamped |refined - announced| lateral, world frame')
        for key in ('tracker_fit_last_before_dispatch', 'tracker_fit_last_before_resend'):
            if key in rp:
                w('    reproduced_%s: {delta_mm: %s, delta_norm_mm: %.2f, clamped_delta_mm: %s, clamped_norm_mm: %.2f}' % (
                    key.replace('tracker_fit_', ''), _v(rp[key]['delta_mm'], 2), rp[key]['delta_norm_mm'],
                    _v(rp[key]['clamped_delta_mm'], 2), rp[key]['clamped_norm_mm']))
    with open(path, 'w') as f:
        f.write('\n'.join(L) + '\n')


def nearest(series, t):
    ts = [s[0] for s in series]
    j = int(np.argmin(np.abs(np.asarray(ts) - t)))
    return series[j]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bag', required=True)
    ap.add_argument('--launch-log', required=True)
    ap.add_argument('--balls', default='10,17,18')
    ap.add_argument('--out', default=None)
    ap.add_argument('--emit-fixture', default=None,
                    help='also write the commented YAML fixture to this path')
    a = ap.parse_args(argv)
    rows = parse_log(a.launch_log)
    balls = [int(x) for x in a.balls.split(',')]
    logs = {b: log_for_ball(rows, b) for b in balls}
    t_lo = min(l['announced_log_t'] for l in logs.values()) - 90.0
    t_hi = max(l['announced_log_t'] for l in logs.values()) + 8.0
    # decode one contiguous slice per ball (the bag is large)
    result = {}
    for b in balls:
        L = logs[b]
        t0 = L['announced_log_t']
        data = read_bag(a.bag, t0 - 75.0, t0 + 6.0)
        td = L['dispatch_t']
        ann = [x for x in data['/throw_announcements'] if x[0] >= t0 - 1.0 and x[0] <= t0 + 1.0]
        fits = [(t, s) for t, s in data['/balls']
                if s.landing_from_fit and abs(_t(s.throw_time) - _t(ann[0][1].throw_time)) < 0.05]
        pre = [f for f in fits if f[0] <= td]
        rec = {'ball': b, 'launch_log': L,
               'announcement': _ann_dict(*ann[0]) if ann else None,
               'tracker_fit_first': _fit_dict(*fits[0]) if fits else None,
               'tracker_fit_last_before_dispatch': _fit_dict(*pre[-1]) if pre else None,
               'tracker_fit_first_after_dispatch': _fit_dict(*[f for f in fits if f[0] > td][0])
               if [f for f in fits if f[0] > td] else None}
        cmd = data['/trajectory/commanded_position']
        c = [x for x in cmd if x[0] <= td]
        rec['commanded_position_at_dispatch'] = {
            'bag_t': c[-1][0], 'xyz_mm': list(c[-1][1]), 'note': 'last /trajectory/commanded_position <= dispatch'}
        ech = data['/leg_setpoint_echo']
        e = nearest(ech, td)
        rec['commanded_pose_fk_at_dispatch'] = dict(echo_pose(e[1]), bag_t=e[0])
        # level-rest baseline: the echo at the announcement (the pre-level has landed, platform level)
        e0 = nearest(ech, t0)
        rec['commanded_pose_fk_level_baseline'] = dict(echo_pose(e0[1]), bag_t=e0[0])
        # tilt history through the window, 0.25 s steps from the announcement to dispatch + 0.5 s
        hist = []
        for tt in np.arange(t0, td + 0.6, 0.25):
            ee = nearest(ech, tt)
            p = echo_pose(ee[1])
            hist.append({'bag_t': ee[0], 'xyz_mm': p['xyz_mm'], 'tilt_deg': p['tilt_deg']})
        rec['commanded_pose_fk_history'] = hist
        rt = L.get('resend_t')
        if rt is not None:
            rp = [f for f in fits if f[0] <= rt]
            rec['tracker_fit_last_before_resend'] = _fit_dict(*rp[-1]) if rp else None
        rec['reproduction'] = reproduce(rec)
        result[str(b)] = rec
        print('ball %d: dispatch %.3f, outcome %s, logged refusal %s mm; reproduced %s' % (
            b, td, L.get('outcome'), L.get('refusal_mm'),
            {k: round(v['clamped_norm_mm'], 2) for k, v in rec['reproduction'].items()
             if isinstance(v, dict)}))
    out = a.out or os.path.join(REPO, 'temp', 'probes', 'capture_move_replay',
                                time.strftime('%Y%m%d_%H%M%S'))
    os.makedirs(out, exist_ok=True)
    p = os.path.join(out, 'capture_move_replay.json')
    with open(p, 'w') as f:
        json.dump(result, f, indent=1)
    print('wrote', p)
    if a.emit_fixture:
        emit_fixture(result, a.emit_fixture, a.bag, a.launch_log)
        print('wrote', a.emit_fixture)
    return result


if __name__ == '__main__':
    main()
