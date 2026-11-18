#!/usr/bin/env python3
"""P2 offline gate of the human_catch Pattern (PWAY-11, U6): the bench bags through the
real Tracker + event emitter + ``HumanCatchPolicy`` + the command-time admissibility check.

Chain, per bag (no hardware, no ROS node, no Platform dynamics):

1. ``human_throw_bench`` machinery (``replay`` with ``detect_human_throws`` on, the BUG-92
   box on, the DEFAULT ``HumanThrowEventEmitter``) gives the event stream; the bench's
   truth (``throws``) and ``serve_map`` / ``match_flight`` say which Throw each event
   stream belongs to.  The stream is cached per bag (``--cache-dir``) so parameter
   variations never re-run the Tracker.
2. A 40 Hz clock drives a real ``HumanCatchPolicy`` exactly as ``skill_node`` does:
   ``on_event`` at the event's stamp, ``poll`` and the plan at the tick.  A PLAN is
   compiled with ``schedule.compile_human_catch`` and tested with
   ``executor.check_segment`` at the Sitting limits from the wait REST at W
   (``human_catch_reach``'s own context).  A later revision of the claimed key is a
   RE-AIM subject to the executor's fences (freeze ``CATCH_FREEZE_S`` before touch-down,
   contact window, 10 mm / 10 ms, ``resend_max``, the lateral clamp about the first
   Landing at ``reaim_authority_mm``) and to the gate on the live record
   (``install_segment`` then ``check_segment``).
3. Score against the bench's truth: the planned Cup xy at touch-down vs the Ball's real
   crossing of the Catch plane, in the SAME frame.

Frames (``--frame``):

* ``hand`` (D11, the design's scoring): the Landings of each direction (release hand of the
  bench's k-means) are translated so that direction's mean crossing is the home Site;
  one Attempt per direction.
* ``robot``: the bag's ``/robot_frame`` (world -> robot, rotation AND translation) and the
  mocap Platform position at rest, as ``skill_node._world_to_schedule`` /
  ``_frame_check`` do (commanded xy at rest taken as 0).  The Throws of these bags did not
  land near the Platform, so most are NOT_AT_ROBOT / out of reach: that is the absolute
  geometry, reported as the honest "as flown" number.

Models, NOT measured (see the results file): the executor ends ``HOLD_AFTER_S`` after
touch-down; a Catch whose planned Cup is within the capture radius and ``--time-tol-ms``
of truth counts as CAUGHT, otherwise MISSED_CATCH; the cup then reads EMPTY after
``--hold-s``.

usage: python tools/probes/human_catch_replay.py BAG [--frames-pkl F] [--t-start HH:MM:SS
          --t-end HH:MM:SS] [--frame hand|robot] [--sweep] [--out-dir DIR]
"""
from __future__ import annotations

import argparse
import collections
import math
import pickle
import sys
import time
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO))
sys.path.insert(0, str(_REPO / 'config' / 'generated'))
sys.path.insert(0, str(_REPO / 'tools' / 'probes'))
sys.path.insert(0, str(_REPO / 'ros_ws' / 'src' / 'jugglebot'))

import human_throw_bench as B                                     # noqa: E402
import human_catch_reach as hcr                                   # noqa: E402
from jugglebot.motion.skills import executor as ex               # noqa: E402
from jugglebot.motion.skills import human_catch as hcp           # noqa: E402
from jugglebot.motion.skills import schedule as sc               # noqa: E402

TICK_S = 0.025                   # the node's 40 Hz tick
HOME_XY = (-50.0, 0.0)           # the Pattern's Site 0 (policy default)
HOLD_AFTER_S = 0.25              # executor done this long after touch-down (model)
CAP_RADIUS_MM = 35.0             # GEOM_HAND_RADIUS_MM (config/generated/hardware_config.py)
WORKSPACE_HALF_MM = 150.0        # CupCycleConfig.workspace_xy_m


# ---------------------------------------------------------------------------
# pure scoring
# ---------------------------------------------------------------------------

def _pct(v, q):
    v = [x for x in v if x is not None and np.isfinite(x)]
    return float(np.percentile(v, q)) if v else float('nan')


def score_rows(rows, phantoms, cap_mm=CAP_RADIUS_MM, time_tol_ms=20.0,
               half_mm=WORKSPACE_HALF_MM):
    """Summarise one run.  ``rows``: one dict per real Throw with ``outcome`` in
    PLANNED / REFUSED / DECLINED / NO_EVENT, ``reason`` (decline reason or gate code),
    ``cross`` (bool: it crosses the plane), ``truth_xy`` (schedule frame) and, for PLANNED,
    ``dist_mm`` (planned Cup xy at touch-down vs truth), ``dt_ms`` (planned touch-down minus
    truth), ``reaims`` (dict of counts).  ``phantoms``: dicts with ``outcome`` (as above).
    Returns plain numbers; no policy and no tracker involved."""
    n = len(rows)
    planned = [r for r in rows if r['outcome'] == 'PLANNED']
    dist = [r['dist_mm'] for r in planned]
    dt = [r['dt_ms'] for r in planned]
    within = [r for r in planned if r['dist_mm'] <= cap_mm and abs(r['dt_ms']) <= time_tol_ms]
    d10 = [r for r in planned if r['dist_mm'] <= 30.0 and abs(r['dt_ms']) <= 20.0]
    in_reach = [r for r in rows if r.get('cross') and r.get('truth_xy') is not None
                and abs(r['truth_xy'][0]) <= half_mm and abs(r['truth_xy'][1]) <= half_mm]
    in_reach_d10 = [r for r in in_reach if r in d10]
    reasons = collections.Counter(r['reason'] for r in rows
                                  if r['outcome'] in ('REFUSED', 'DECLINED', 'NO_EVENT'))
    reaims = collections.Counter()
    for r in planned:
        reaims.update(r.get('reaims') or {})
    return dict(
        n_throws=n, n_cross=sum(1 for r in rows if r.get('cross')),
        n_planned=len(planned), n_admitted=len(planned),
        dist_p50=_pct(dist, 50), dist_p90=_pct(dist, 90),
        dt_p50=_pct([abs(x) for x in dt], 50), dt_p90=_pct([abs(x) for x in dt], 90),
        n_within_cap=len(within), frac_within_cap=(len(within) / len(planned)
                                                   if planned else float('nan')),
        n_d10_good=len(d10), n_in_reach=len(in_reach), n_in_reach_d10=len(in_reach_d10),
        in_reach_planned=sum(1 for r in in_reach if r['outcome'] == 'PLANNED'),
        declined=dict(reasons), reaims=dict(reaims),
        n_phantom=len(phantoms),
        n_phantom_planned=sum(1 for p in phantoms if p['outcome'] == 'PLANNED'),
        n_phantom_refused=sum(1 for p in phantoms if p['outcome'] == 'REFUSED'))


# ---------------------------------------------------------------------------
# stream (Tracker + emitter), cached per bag
# ---------------------------------------------------------------------------

def load_frames(bag, frames_pkl=None, t_start=None, t_end=None):
    if frames_pkl:
        with open(frames_pkl, 'rb') as fh:
            fr = pickle.load(fh)
        return [f[:3] for f in fr], (fr[0][3] if len(fr[0]) > 3 else None)
    f4 = B.load_frames(Path(bag), with_wall=True)
    t_first, wall0 = f4[0][0], f4[0][3]
    lo, hi = B._parse_clock(t_start, wall0, t_first), B._parse_clock(t_end, wall0, t_first)
    return [f[:3] for f in f4 if (lo is None or f[0] >= lo) and (hi is None or f[0] <= hi)], wall0


def robot_frame_info(bag, wall_lo=None, window_s=20.0):
    """(RobotFrame, platform xy in the robot frame, spread mm) from the bag's /robot_frame and
    /rigid_body_poses ('Platform') over ``window_s`` from ``wall_lo`` (None = the bag start).
    Returns None when the bag carries none."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    from jugglebot.robot_frame import RobotFrame
    mc = B.R._find_mcap(Path(bag))
    with open(mc, 'rb') as fh:
        rd = make_reader(fh, decoder_factories=[DecoderFactory()])
        t0 = rd.get_summary().statistics.message_start_time * 1e-9
        lo = (t0 if wall_lo is None else wall_lo)
        rf = None
        for _s, ch, m, d in rd.iter_decoded_messages(topics=['/robot_frame']):
            rf = d
            if m.log_time * 1e-9 >= lo:
                break
        if rf is None:
            return None
        frame = RobotFrame.from_pose(
            (rf.pose.position.x, rf.pose.position.y, rf.pose.position.z), (rf.pose.orientation.x, rf.pose.orientation.y,
                     rf.pose.orientation.z, rf.pose.orientation.w))
        pts = []
        for _s, ch, m, d in rd.iter_decoded_messages(
                topics=['/rigid_body_poses'], start_time=int(lo * 1e9),
                end_time=int((lo + window_s) * 1e9)):
            for b in d.bodies:
                if b.name == 'Platform' and np.all(np.isfinite(
                        [b.pose.pose.position.x, b.pose.pose.position.y])):
                    pts.append([b.pose.pose.position.x, b.pose.pose.position.y,
                                b.pose.pose.position.z])
        if not pts:
            return None
        pts = np.array(pts)
        r = frame.to_robot(pts)
        spread = float(np.hypot(np.ptp(r[:, 0]), np.ptp(r[:, 1])))
        return frame, np.median(r[:, :2], axis=0), spread


def build_stream(frames):
    """Run the real Tracker + default emitter over ``frames``; return the cacheable dict."""
    from jugglebot.tracking.human_throw_event import HumanThrowEventConfig as C
    th = B.throws(B.flights(B.link(frames), 6.0), 830.0)
    if len(th) >= 2:
        _c, hand = B.kmeans(np.array([x['p_rel'][:2] for x in th]), 2)
    else:
        hand = np.zeros(len(th), int)
    for x, h in zip(th, hand):
        x['hand'] = int(h)
    ems = {'default': B.make_emitter('default', C())}
    ev, seen = {}, {}
    hist = B.replay(frames, emitters=ems, box=True, flights_seen=seen, events_out=ev)
    serve = B.serve_map(hist, th)
    sids = {k: {bid for _, bid, _ in v} for k, v in serve.items()}
    first = {}
    for e in ev['default']:
        first.setdefault((e.track_id, e.flight_seq), e.flight_start)
    k_of = {key: B.match_flight(key[0], fs, th, sids) for key, fs in first.items()}
    events = [dict(track_id=e.track_id, flight_seq=e.flight_seq, revision=e.revision,
                   stamp=float(e.stamp), landing=[float(v) for v in e.landing_position],
                   vel=[float(v) for v in e.landing_velocity],
                   t_land=float(e.landing_time), fit_age=float(e.fit_age_s),
                   final=bool(e.final), k=k_of[(e.track_id, e.flight_seq)])
              for e in ev['default']]
    keep = ('t_rel', 't_end', 't_cross', 'xy_cross', 'apex', 'hand')
    return dict(events=events, truth=[{k: x[k] for k in keep} for x in th],
                t0=float(frames[0][0]), span=float(frames[-1][0] - frames[0][0]),
                rejections=dict(ems['default'].rejections))


# ---------------------------------------------------------------------------
# admissibility (the real compile + check_segment, as the node would)
# ---------------------------------------------------------------------------

_GATE_CACHE = {}


def _ctx(wait_xy):
    return hcr._ctx((round(float(wait_xy[0]), 1), round(float(wait_xy[1]), 1)))


def _limit_name(box):
    return hcr._limiting(box)[0]


def check_plan(wait_xy, landing_mm, vel, lead_s):
    """compile_human_catch + check_segment from REST at W.  Returns (ok, code, limiting)
    or ('WINDOW_TOO_SHORT') when the window cannot compile."""
    key = ('plan', round(wait_xy[0], 1), round(wait_xy[1], 1), round(landing_mm[0], 1),
           round(landing_mm[1], 1), round(landing_mm[2], 0), round(lead_s, 3),
           round(vel[2] / 20.0), round(vel[0] / 50.0), round(vel[1] / 50.0))
    if key in _GATE_CACHE:
        return _GATE_CACHE[key]
    c = _ctx(wait_xy)
    try:
        sched = sc.compile_human_catch(np.array(landing_mm, float), np.array(vel, float),
                                       hcr.T_NOW + lead_s, hcr.T_NOW,
                                       hcr.DISPATCH_LOOKAHEAD_S)
    except ValueError:
        out = (False, ex.WINDOW_TOO_SHORT, '-', None)
        _GATE_CACHE[key] = out
        return out
    sk = sched.skills[0]
    with hcr._capture_reports() as box:
        res = ex.check_segment(None, c.seed(), sc.CATCH, hcr._terminal(sched),
                               sk.dispatch_s(), cfg=c.cfg, limits=c.limits, geom=c.geom,
                               reserve_fresh_lead=True)
        name = _limit_name(box)
    if res.planned:
        ok = bool(float(res.margin) >= 0.0)
        out = (ok, 'OK' if ok else 'MARGIN', name, sched)
    else:
        out = (False, str(res.code), name, sched)
    _GATE_CACHE[key] = out
    return out


def install_first(wait_xy, sched):
    """The live record of an admitted first plan (None if the install refuses)."""
    c = _ctx(wait_xy)
    sk = sched.skills[0]
    rec, res, _seg = ex.install_segment(None, c.seed(), sc.CATCH, hcr._terminal(sched),
                                        sk.dispatch_s(), cfg=c.cfg, limits=c.limits,
                                        geom=c.geom, reserve_fresh_lead=True)
    return rec if res.accepted else None


def check_reaim_live(wait_xy, rec, sched0, landing_mm, vel, t_land_rel, t_now_rel):
    """The gate on the LIVE record for a re-aim at (relative) instant ``t_now_rel``.
    Returns (ok, code, new_rec)."""
    c = _ctx(wait_xy)
    sk = sched0.skills[0]
    t1 = ex.CatchTerminal(
        landing_mm=np.array(landing_mm, float), landing_vel_mm_s=np.array(vel, float),
        t_land_s=float(t_land_rel), rest_site_mm=sk.site.rest_site_mm(), then_throw=None,
        hold_tilt=None, receive_tilt=(0.0, 0.0))
    res = ex.check_segment(rec, None, sc.CATCH, t1, t_now_rel, cfg=c.cfg, limits=c.limits,
                           geom=c.geom, reserve_fresh_lead=True)
    if not res.planned:
        return False, str(res.code), None
    if float(res.margin) < 0.0:
        return False, 'MARGIN', None
    rec2, r2, _ = ex.install_segment(rec, None, sc.CATCH, t1, t_now_rel, cfg=c.cfg,
                                     limits=c.limits, geom=c.geom, reserve_fresh_lead=True)
    return (True, 'OK', rec2) if r2.accepted else (False, str(r2.code), None)


# ---------------------------------------------------------------------------
# frames
# ---------------------------------------------------------------------------

class Xform:
    """world (mocap) landing -> schedule frame for one direction / frame mode."""

    def __init__(self, mode, mean_cross=None, robot=None):
        self.mode = mode
        self.mean = None if mean_cross is None else np.asarray(mean_cross, float)[:2]
        self.robot = robot

    def xy(self, p):
        p = np.asarray(p, float)
        if self.mode == 'robot':
            frame, plat = self.robot[0], self.robot[1]
            return frame.to_robot(np.array([p[0], p[1], 830.0]))[:2] - plat
        return p[:2] - self.mean + np.asarray(HOME_XY)

    def vel(self, v):
        v = np.asarray(v, float)
        if self.mode == 'robot':
            return self.robot[0].vel_to_robot(v)
        return v.copy()


# ---------------------------------------------------------------------------
# one Attempt
# ---------------------------------------------------------------------------

def run_attempt(events, truth, xf, policy_kw, authority_mm=None, hold_s=1.0,
                cap_mm=CAP_RADIUS_MM, time_tol_ms=20.0, tick=TICK_S, t_origin=0.0,
                resend_max=2):
    """Drive one real ``HumanCatchPolicy`` over ``events`` (dicts, any order; each with
    ``k`` = index into ``truth`` or None) and return ``(rows, phantoms, log)``."""
    pol = hcp.HumanCatchPolicy(**policy_kw)
    pol.opening_done()
    auth = pol.reaim_authority_mm if authority_mm is None else float(authority_mm)
    evs = sorted(events, key=lambda e: (e['stamp'], e['revision']))
    sched_ev = []
    for e in evs:
        pos = xf.xy(e['landing'])
        z = 830.0
        v = xf.vel(e['vel'])
        sched_ev.append((e, hcp.HumanThrowEvt(
            track_id=e['track_id'], flight_seq=e['flight_seq'], revision=e['revision'],
            landing_mm=(float(pos[0]), float(pos[1]), z),
            landing_vel_mm_s=tuple(float(a) for a in v), landing_time_s=e['t_land'],
            fit_age_s=e['fit_age'], final=e['final'])))
    key_k = {(e['track_id'], e['flight_seq']): e['k'] for e in evs}
    verdict = {}      # key -> dict(outcome, reason, ...)
    log = collections.Counter()
    latest = {}
    live = None
    done_at = None
    hold_until = None
    ret_until = None
    ended = 0
    plan_due = None
    i = 0
    t = (math.floor((evs[0]['stamp'] - t_origin) / tick) * tick + t_origin) if evs else 0.0
    t_end = (evs[-1]['stamp'] + 4.0) if evs else 0.0

    def note(key, outcome, reason, **kw):
        cur = verdict.get(key)
        rank = {'PLANNED': 3, 'REFUSED': 2, 'DECLINED': 1, 'PENDING': 0}
        if cur is None or rank[outcome] >= rank[cur['outcome']]:
            verdict[key] = dict(outcome=outcome, reason=reason, **kw)

    while t <= t_end:
        while i < len(sched_ev) and sched_ev[i][0]['stamp'] <= t:
            _raw, ev = sched_ev[i]
            i += 1
            latest[ev.key] = ev
            d = pol.on_event(ev, _raw['stamp'])
            if d.kind == hcp.PLAN:
                plan_due = d
                note(ev.key, 'PENDING', 'PLANNING')
            elif d.kind == hcp.PENDING:
                note(ev.key, 'PENDING', 'HELD')
            elif d.reason not in (hcp.KEY_DECLINED, hcp.KEY_CLAIMED):
                note(ev.key, 'DECLINED', d.reason)
        # the tick: phase machine, then plan window
        if pol.phase == hcp.CATCHING and live is not None and t >= live['done_at']:
            code = ('MISSED_CATCH' if not live['caught'] else 'COMPLETED')
            out = pol.on_executor_done(code, moved=True)
            live['result'] = out.next
            if out.next == hcp.TO_HOLDING:
                hold_until = t + hold_s
            live = None
        if pol.phase == hcp.HOLDING and hold_until is not None and t >= hold_until:
            pol.observe_cup(hcp.CUP_EMPTY, t)     # debounced inside the policy (clear_s)
        if pol.phase == hcp.RETURNING and ret_until is None:
            _xy, moved = pol.begin_return()
            log['wait_updates'] += 1 if moved else 0
            ret_until = t + pol.return_s
        if pol.phase == hcp.RETURNING and ret_until is not None and t >= ret_until:
            pol.return_done()
            ret_until = None
        d = plan_due
        plan_due = None
        if d is None and pol.phase == hcp.WAITING:
            d = pol.poll(t)
        if d is not None and d.kind == hcp.IGNORE and d.reason not in (hcp.KEY_DECLINED,
                                                                         hcp.KEY_CLAIMED):
            note(d.key, 'DECLINED', d.reason)
        if d is not None and d.kind == hcp.PLAN:
            evt = d.evt                      # the node compiles from the policy's claimed revision
            lead = evt.landing_time_s - t
            wait = pol.wait_xy_mm.copy()
            ok, code, name, sched = check_plan(wait, evt.landing_mm, evt.landing_vel_mm_s, lead)
            if code == ex.WINDOW_TOO_SHORT or not ok:
                out = pol.on_executor_done(code, moved=False)
                ended += 1 if out.next == hcp.END_ATTEMPT else 0
                note(d.key, 'REFUSED' if code != ex.WINDOW_TOO_SHORT else 'DECLINED',
                     code, limiting=name, lead=lead,
                     dist=float(np.linalg.norm(np.asarray(evt.landing_mm[:2]) - wait)))
            else:
                rec = install_first(wait, sched)
                if rec is None:
                    out = pol.on_executor_done('INFEASIBLE', moved=False)
                    note(d.key, 'REFUSED', 'INSTALL', limiting=name, lead=lead)
                else:
                    live = dict(key=d.key, wait=wait, sched=sched, rec=rec, t_plan=t,
                                first_xy=np.asarray(evt.landing_mm[:2], float),
                                xy=np.asarray(evt.landing_mm[:2], float),
                                vel=np.asarray(evt.landing_vel_mm_s, float),
                                z=evt.landing_mm[2], t_land=evt.landing_time_s,
                                t_last=t, resends=0, reaims=collections.Counter(),
                                first_lead=lead,
                                first_dist=float(np.linalg.norm(
                                    np.asarray(evt.landing_mm[:2]) - wait)),
                                first_rev=evt.revision)
                    live['done_at'] = live['t_land'] + HOLD_AFTER_S
                    note(d.key, 'PLANNED', 'OK', live=live)
        elif live is not None and pol.phase == hcp.CATCHING:
            _reaim(live, latest.get(live['key']), t, auth, resend_max)
        if live is not None:
            _finalise(live, truth, xf, key_k, cap_mm, time_tol_ms)
        t += tick
        if i >= len(sched_ev) and pol.phase == hcp.WAITING and pol.pending is None \
                and live is None:
            break
    log['ended_attempts'] = ended
    return _rows(verdict, evs, truth, xf, pol), _phantoms(verdict, evs), dict(log)


def _reaim(live, ev, t, auth, resend_max):
    """One tick's re-aim decision for the live CATCH (the executor's fences, in order)."""
    if ev is None:
        return
    r = live['reaims']
    tl = live['t_land']
    if t >= tl - ex.CATCH_FREEZE_S:
        live['frozen'] = True
        return
    if live.get('dead') or t - live['t_last'] < 0.025:
        return
    t_s = t + ex.LEAD_S
    if tl - t_s < sc.MIN_WINDOW_S - 1e-12:
        live['dead'] = True
        return
    first = live['first_xy']
    new_xy = np.asarray(ev.landing_mm[:2], float)
    clamped = first + np.clip(new_xy - first, -auth, auth)
    moved = float(np.max(np.abs(clamped - live['xy'])))
    moved_s = float(ev.landing_time_s) - live['t_land']
    if moved <= 10.0 and abs(moved_s) <= 0.010:
        return                                  # WITHIN-TOLERANCE: nothing requested
    if live['resends'] >= resend_max:
        r['declined_CAP-SPENT'] = 1
        return
    if t_s >= float(ev.landing_time_s) - ex.CUP_CONTACT_WINDOW_LEAD_S:
        r['declined_CONTACT-WINDOW'] = 1
        return
    r['requested'] += 1
    off = hcr.T_NOW + (t - live['t_plan'])
    ok, code, rec2 = check_reaim_live(
        live['wait'], live['rec'], live['sched'],
        (clamped[0], clamped[1], live['z']), ev.landing_vel_mm_s,
        hcr.T_NOW + (float(ev.landing_time_s) - live['t_plan']), off)
    live['resends'] += 1
    live['t_last'] = t
    if ok:
        r['admitted'] += 1
        live['rec'] = rec2
        live['xy'] = clamped
        live['vel'] = np.asarray(ev.landing_vel_mm_s, float)
        live['t_land'] = float(ev.landing_time_s)
        live['done_at'] = live['t_land'] + HOLD_AFTER_S
    else:
        r['refused_' + code] += 1
        live['dead'] = True


def _finalise(live, truth, xf, key_k, cap_mm, time_tol_ms):
    """Decide CAUGHT / MISSED for the live CATCH from its CURRENT terminal vs truth (re-evaluated
    every tick: the terminal at ``done_at`` is the one that counts)."""
    k = key_k.get(live['key'])
    if k is None:
        live['caught'] = False
        return
    tr = truth[k]
    if not np.isfinite(tr['t_cross']):
        live['caught'] = False
        return
    txy = xf.xy(tr['xy_cross'])
    d = float(np.linalg.norm(live['xy'] - txy))
    dt = (live['t_land'] - tr['t_cross']) * 1000.0
    live['caught'] = bool(d <= cap_mm and abs(dt) <= time_tol_ms)


def _rows(verdict, evs, truth, xf, pol):
    keys_of = collections.defaultdict(list)
    for e in evs:
        key = (e['track_id'], e['flight_seq'])
        if e['k'] is not None and key not in keys_of[e['k']]:
            keys_of[e['k']].append(key)
    rows = []
    for k, tr in enumerate(truth):
        cross = bool(np.isfinite(tr['t_cross']))
        r = dict(k=k, cross=cross, outcome='NO_EVENT', reason='NO_EVENT',
                 truth_xy=(xf.xy(tr['xy_cross']) if cross else None), hand=tr.get('hand'))
        vs = [verdict[key] for key in keys_of.get(k, ()) if key in verdict]
        if k not in keys_of:
            rows.append(r)
            continue
        if not vs:
            r.update(outcome='DECLINED', reason='NO_USABLE_EVENT')
            rows.append(r)
            continue
        best = max(vs, key=lambda v: {'PLANNED': 3, 'REFUSED': 2, 'DECLINED': 1,
                                      'PENDING': 0}[v['outcome']])
        if best['outcome'] == 'PENDING':
            best = dict(outcome='DECLINED', reason='HELD_NEVER_PLANNED')
        r['outcome'], r['reason'] = best['outcome'], best['reason']
        if best['outcome'] == 'PLANNED':
            lv = best['live']
            r['dist_mm'] = float(np.linalg.norm(lv['xy'] - r['truth_xy'])) if cross else float('nan')
            r['dt_ms'] = (lv['t_land'] - tr['t_cross']) * 1000.0 if cross else float('nan')
            r['reaims'] = dict(lv['reaims'])
            r['first_lead'] = lv['first_lead']
            r['first_dist_mm'] = lv['first_dist']
            r['first_xy_err'] = (float(np.linalg.norm(lv['first_xy'] - r['truth_xy']))
                                 if cross else float('nan'))
            r['caught'] = bool(lv.get('caught'))
            if not cross:
                r['outcome'], r['reason'] = 'PLANNED', 'NO_CROSSING'
        elif best['outcome'] == 'REFUSED':
            r['limiting'] = best.get('limiting')
            r['lead'] = best.get('lead')
        rows.append(r)
    return rows


def _phantoms(verdict, evs):
    seen = {}
    for e in evs:
        if e['k'] is None:
            seen[(e['track_id'], e['flight_seq'])] = e['stamp']
    out = []
    for key, t in seen.items():
        v = verdict.get(key)
        oc = 'NO_EVENT' if v is None else v['outcome']
        out.append(dict(key=key, t=t, outcome=oc, reason='' if v is None else v['reason']))
    return out


# ---------------------------------------------------------------------------
# a bag, all directions
# ---------------------------------------------------------------------------

def run_bag(S, frame_mode, robot=None, params=None, authority_mm=None, hold_s=1.0,
            cap_mm=CAP_RADIUS_MM, time_tol_ms=20.0):
    params = dict(params or {})
    truth, events = S['truth'], S['events']
    cross = [x for x in truth if np.isfinite(x['t_cross'])]
    rows, phantoms, log = [], [], collections.Counter()
    if frame_mode == 'robot':
        dirs = [(None, Xform('robot', robot=robot), events, list(range(len(truth))))]
    else:
        means = {}
        for h in sorted({x['hand'] for x in cross}):
            means[h] = np.mean([x['xy_cross'] for x in cross if x['hand'] == h], axis=0)
        # a phantom (k None) joins the direction whose mean crossing is nearest its Landing
        dirs = []
        for h, m in means.items():
            ev_h = []
            for e in events:
                if e['k'] is not None:
                    if truth[e['k']]['hand'] == h:
                        ev_h.append(e)
                else:
                    near = min(means, key=lambda hh: np.linalg.norm(
                        np.asarray(e['landing'][:2]) - means[hh]))
                    if near == h:
                        ev_h.append(e)
            dirs.append((h, Xform('hand', mean_cross=m), ev_h,
                         [k for k, x in enumerate(truth) if x['hand'] == h]))
    for h, xf, ev_h, ks in dirs:
        sub_truth = truth
        rs, ph, lg = run_attempt(ev_h, sub_truth, xf, params, authority_mm, hold_s, cap_mm,
                                 time_tol_ms, t_origin=S['t0'])
        keep = set(ks)
        rows += [r for r in rs if r['k'] in keep]
        phantoms += ph
        log.update(lg)
    rows.sort(key=lambda r: r['k'])
    return rows, phantoms, dict(log)


def fmt_summary(name, s):
    d = s['declined']
    return ('%s | %d (%d) | %d | %s / %s mm | %s / %s ms | %d/%d (%.0f%%) | %d/%d | %s | %s | %d (%d planned)'
            % (name, s['n_throws'], s['n_cross'], s['n_planned'],
               '%.0f' % s['dist_p50'] if np.isfinite(s['dist_p50']) else '-',
               '%.0f' % s['dist_p90'] if np.isfinite(s['dist_p90']) else '-',
               '%.0f' % s['dt_p50'] if np.isfinite(s['dt_p50']) else '-',
               '%.0f' % s['dt_p90'] if np.isfinite(s['dt_p90']) else '-',
               s['n_within_cap'], s['n_planned'],
               100 * s['frac_within_cap'] if s['n_planned'] else float('nan'),
               s['n_in_reach_d10'], s['n_in_reach'],
               ', '.join('%s %d' % kv for kv in sorted(d.items())) or '-',
               ', '.join('%s %d' % kv for kv in sorted(s['reaims'].items())) or '-',
               s['n_phantom'], s['n_phantom_planned']))


VARIANTS = (
    ('baseline plan 0.62 / first 0.55 / auth 50 / mean5 clamp150', {}, None),
    ('plan_lead 0.58', dict(plan_lead_s=0.58), None),
    ('plan_lead 0.66', dict(plan_lead_s=0.66), None),
    ('first_lead 0.52', dict(first_lead_s=0.52), None),
    ('authority 30', {}, 30.0),
    ('authority 80', {}, 80.0),
    ('wait fixed at home (-50,0)', dict(wait_radius_mm=0.0), None),
    ('wait fixed at centre (0,0)', dict(wait_radius_mm=0.0, home_xy_mm=(0.0, 0.0)), None),
)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--frames-pkl', default=None, help='cached frame list (same format as the bench cache)')
    ap.add_argument('--t-start', default=None)
    ap.add_argument('--t-end', default=None)
    ap.add_argument('--frame', choices=('hand', 'robot', 'both'), default='both')
    ap.add_argument('--sweep', action='store_true', help='run the one-at-a-time variants')
    ap.add_argument('--hold-s', type=float, default=1.0)
    ap.add_argument('--cap-mm', type=float, default=CAP_RADIUS_MM)
    ap.add_argument('--out-dir', default=str(_REPO / 'temp' / 'probes' / 'p2'))
    a = ap.parse_args(argv)
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    bag = Path(a.bag)
    cache = out / ('%s_stream.pkl' % bag.name)
    frames_wall = None
    if cache.exists():
        S = pickle.load(open(cache, 'rb'))
    else:
        frames, frames_wall = load_frames(bag, a.frames_pkl, a.t_start, a.t_end)
        t0 = time.time()
        S = build_stream(frames)
        S['build_s'] = time.time() - t0
        S['wall0'] = frames_wall
        pickle.dump(S, open(cache, 'wb'))
    print('stream: %d events, %d truth Throws' % (len(S['events']), len(S['truth'])), flush=True)
    robot = None
    if a.frame in ('robot', 'both'):
        robot = robot_frame_info(bag, S.get('wall0'))
        print('robot frame:', None if robot is None else
              'R yaw %.1f deg, t %s, platform in robot frame %s spread %.1f mm' % (
                  math.degrees(math.atan2(robot[0].R[1, 0], robot[0].R[0, 0])),
                  np.round(robot[0].t, 1), np.round(robot[1], 1), robot[2]), flush=True)
    lines = ['## %s' % bag.name, '',
             '| variant | Throws (crossing) | planned (admitted) | planned Cup vs truth xy p50 / p90 | |dt| p50 / p90 | within %.0f mm and %.0f ms | D10 good / in reach | declined / refused by reason | re-aims | phantom streams |'
             % (a.cap_mm, 20.0), '|' + '---|' * 10]
    modes = ['hand', 'robot'] if a.frame == 'both' else [a.frame]
    for mode in modes:
        if mode == 'robot' and robot is None:
            lines.append('| robot frame: bag carries no /robot_frame | | | | | | | | | |')
            continue
        for name, kw, auth in (VARIANTS if a.sweep else VARIANTS[:1]):
            t0 = time.time()
            rows, ph, lg = run_bag(S, mode, robot, kw, auth, a.hold_s, a.cap_mm)
            s = score_rows(rows, ph, a.cap_mm)
            lines.append('| [%s] ' % mode + fmt_summary(name, s) + ' |')
            print('[%s] %s: %.1f s' % (mode, name, time.time() - t0), flush=True)
            if name.startswith('baseline'):
                pickle.dump(dict(rows=rows, phantoms=ph, log=lg),
                            open(out / ('%s_%s_baseline.pkl' % (bag.name, mode)), 'wb'))
    text = '\n'.join(lines)
    (out / ('%s_replay.md' % bag.name)).write_text(text + '\n')
    print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
