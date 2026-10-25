"""Capture-move scenario harness: the sim reload gate with the capture leg reachable.

Written 2026-10-19 (Unit 5b of the capture-move fix, promoted from the Unit 3
throwaway driver).  Logbook:
logbook/2026-10-19-held-axis-reload-catch-refuses-tracker-reaim.md.

WHAT IT DOES
  Runs ``sim/skills_gate.py``'s reload attempt (``SkillsGate.run_reload_seed``)
  with a landing bias, a ball-flight lead and the lateral authorities chosen on
  the command line, wraps ``executor.install_segment`` to record every install,
  and prints ONE results line for the capture leg (the accepted CATCH install
  that was spliced mid-flight, else the last accepted CATCH):

    CAPTURE bias=(bx,by) flight_lead=.. authority=.. seed=.. passed=.. makes=..
      end_code=.. capture_install=.. k_h=.. splice_k=.. leg_jerk_peak=<mm/s3>
      plat_xy_speed_peak=<mm/s> plat_xy_at_k_h=(x,y) measured_xy_at_touchdown=(x,y)

  Leg jerk is the third difference of the leg extensions of the PLAN poses
  (``plant.pose_to_extensions``) over the knots before k_h; platform xy speed is
  the plan's ``pose_vel`` norm; k_h is ``plan.contact_knots[0]``.  The measured
  platform xy comes from the MuJoCo trace (``plant.step`` wrapped).  The full
  per-install dump and the executor lines go to
  temp/probes/capture_move_sim/<timestamp>/run.log.

WHY THE STOCK GATE CANNOT REACH THE CAPTURE
  1. Flight lead.  ``SkillsGate._RELOAD_BALL_FLIGHT_LEAD_S`` is 0.35 s, but the
     reload CATCH is dispatched about 0.774 s before landing.  The ball does not
     exist at the dispatch, and the tracker fit converges about 0.05 s BEFORE the
     re-send freeze, so the reload CATCH is never re-aimed and a landing bias has
     no effect (the dispatch-capture path, a fit before the dispatch, needs more
     than 0.77 s of flight and cannot be reached under the floor; it is covered by
     tests/motion/test_skills_capture_replay.py instead).  ``--flight-lead 0.65``
     spawns the ball at z ~77 mm (above the floor plane) and exercises the
     RE-SEND path as the robot runs it.
  2. Authority.  The gate builds the executor with the lateral authorities
     UNSET, so an immature 12-sample fit re-aims 66-91 mm (a correct CATCH_AXIS
     cap refusal).  ``--authority`` (default on) passes the live values: lateral
     20 mm, inward 35 mm.  The same gap is the cause of the stock gate's
     LIMIT_JERK (the unclamped 41-45 mm re-aim on the second catch); see the
     plan residue note in plans/archived/columns-empty-cup-two-feed.md.
  Use ``--reload-bias-mm`` (a landing bias off the ANNOUNCED landing; compile
  still sees the announcement) to put a known lateral error on the ball.

RECORDED RECIPE (2026-10-19 ~12:50-13:00 AEDT, seed 0, venv, OPENBLAS_NUM_THREADS=1)
  run                         command                                          result
  stock, bias -20 -20         python sim/skills_gate.py --reload --seeds 0     FAIL makes=1, LIMIT_JERK
                                --no-viewer --reload-bias-mm -20 -20           (post-catch REST, install 6/7);
                                                                               skill 1 never re-aimed
  stock, bias 5.6 -20         same with 5.6 -20                                identical FAIL
  stock, no bias (control)    flight lead 0.35, no authority                   identical FAIL
  capture control             capture_move_sim.py 0 0                          PASS makes 2; capture to (-20,0)
                                                                               (immature fit x-noise -71 mm, clamped)
  capture -20 -20             capture_move_sim.py -20 -20                      PASS makes 2, no END; captured
  capture 5.6 -20             capture_move_sim.py 5.6 -20                      PASS makes 2, no END; captured
  Capture leg (install 2, spliced re-send at knot 8, k_h = 8): peak plan leg jerk
  35.7k (control) / 48.4k (both biased) mm/s3 against the 200k cap; acc 1.15-1.55
  m/s2; peak plan xy vel 175 / 245 mm/s; platform xy plan (-33.14, 0) ->
  (-53.71, -20.00) by k_h; measured platform reaches (-53.2, -20.0) at touch-down.
  Caveats: the clamp ((-20,-20) = 28 mm) is what is captured in all three runs (the
  first fit lands -65..-91 mm in x regardless of the bias), so the sim does not
  discriminate (5.6,-20) from (-20,-20); and a second re-send is refused CATCH_AXIS
  ("only N steps left"), benign and non-fatal.

USAGE (venv; ~minute per seed)
  python tools/probes/capture_move_sim.py --help
  python tools/probes/capture_move_sim.py -20 -20 --seed 0
  python tools/probes/capture_move_sim.py 0 0 --flight-lead 0.35 --no-authority   # the stock-gate control
"""
from __future__ import annotations

import argparse
import datetime
import os
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def _parse(argv):
    p = argparse.ArgumentParser(
        description='Run the sim reload gate with a landing bias and print the capture-leg evidence line.')
    p.add_argument('bx', nargs='?', type=float, default=0.0,
                   help="landing bias x off the announced landing, mm (the gate's --reload-bias-mm BX)")
    p.add_argument('by', nargs='?', type=float, default=0.0, help='landing bias y, mm')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--flight-lead', type=float, default=0.65, metavar='S',
                   help='ball flight lead, s (stock gate 0.35; 0.65 reaches the re-send path). Default 0.65')
    g = p.add_mutually_exclusive_group()
    g.add_argument('--authority', dest='authority', action='store_true', default=True,
                   help='pass the live lateral (20 mm) / inward (35 mm) authorities to the executor (default)')
    g.add_argument('--no-authority', dest='authority', action='store_false',
                   help='leave the authorities unset, as the stock gate does')
    p.add_argument('--out', default=None, help='output dir (default temp/probes/capture_move_sim/<timestamp>/)')
    return p.parse_args(argv)


def main(argv=None) -> int:
    a = _parse(sys.argv[1:] if argv is None else argv)
    out = a.out or os.path.join(REPO, 'temp', 'probes', 'capture_move_sim',
                                datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(out, exist_ok=True)

    import numpy as np
    sys.path[:0] = [REPO, os.path.join(REPO, 'sim'), os.path.join(REPO, 'ros_ws', 'src', 'jugglebot')]
    import skills_gate as sg
    from jugglebot.motion.skills import executor as ex

    bx, by = a.bx, a.by
    calls = []
    orig_install = ex.install_segment

    def wrapped(record, seed_rest, kind, terminal, t_now_s, **kw):
        rec, res, seg = orig_install(record, seed_rest, kind, terminal, t_now_s, **kw)
        calls.append(dict(kind=kind, t_now=t_now_s, acc=res.accepted, code=res.code, msg=res.message,
                          splice_k=res.splice_k, t0=res.t0_s, event_t=res.event_t_s, seg=seg))
        return rec, res, seg

    ex.install_segment = wrapped
    exes = []
    orig_init = ex.SkillExecutor.__init__

    def new_init(self, *args, **kw):
        if a.authority:
            kw['lateral_authority_m'] = 0.020
            kw['catch_inward_authority_m'] = 0.035
        orig_init(self, *args, **kw)
        exes.append(self)

    ex.SkillExecutor.__init__ = new_init
    lines_log = []

    def _wrap(name):
        o = getattr(ex.SkillExecutor, name)

        def w(self, *args, **kw):
            res = o(self, *args, **kw)
            ls = res[0] if isinstance(res, tuple) else res
            lines_log.extend((name, l) for l in (ls or []))
            return res
        setattr(ex.SkillExecutor, name, w)

    _wrap('_resend_live_catch')
    _wrap('_dispatch')

    cfg = sg.SelfTossGateConfig()
    cfg.reload_bias_mm = (bx, by)
    sg.SkillsGate._RELOAD_BALL_FLIGHT_LEAD_S = a.flight_lead
    gate = sg.SkillsGate(cfg)
    plant = gate.plant
    trace = []
    orig_step = plant.step

    def step(dt, *args, **kw):
        orig_step(dt, *args, **kw)
        s = plant.get_state()
        trace.append((float(plant.data.time), float(s.platform_pos_mm[0]), float(s.platform_pos_mm[1]),
                      float(s.platform_twist[0]), float(s.platform_twist[1])))

    plant.step = step
    r = gate.run_reload_seed(a.seed)

    cap = None
    for i, c in enumerate(calls):
        if c['kind'] == 'CATCH' and c['acc'] and c['seg'] is not None:
            if cap is None or c['splice_k'] is not None:
                cap = (i, c)
    with open(os.path.join(out, 'run.log'), 'w') as fh:
        fh.write('SEED RESULT %r\nBIAS %r flight_lead=%s authority=%s\n' % (
            {k: r.get(k) for k in ('seed', 'end_code', 'makes', 'drops', 'installs_total',
                                   'installs_accepted', 'passed')}, (bx, by), a.flight_lead, a.authority))
        for i, c in enumerate(calls):
            fh.write('INSTALL %d kind=%s t_now=%.3f accepted=%s code=%s splice_k=%s\n' % (
                i, c['kind'], c['t_now'], c['acc'], c['code'], c['splice_k']))
            if not c['acc'] or c['code']:
                fh.write('   msg: %s\n' % c['msg'])
        for n, l in lines_log:
            fh.write('%s: %s\n' % (n, l[:400]))
        for e in exes:
            for sev, txt in e.notices:
                fh.write('[%s] %s\n' % (sev, txt))

    tail = ''
    if cap is not None:
        i, c = cap
        pl = c['seg'].plan
        pose = np.asarray(pl.pose)
        dt = pl.dt
        ext = np.array([plant.pose_to_extensions(p) for p in pose])
        jerk = np.diff(ext, n=3, axis=0) / dt ** 3
        pv = np.asarray(pl.pose_vel)
        xyv = np.hypot(pv[:, 0], pv[:, 1])
        ck = pl.contact_knots
        kh = ck[0][0] if ck and isinstance(ck[0], (tuple, list)) else (ck[0] if ck else None)
        m = np.array(trace)
        sel = (m[:, 0] >= c['t_now']) & (m[:, 0] <= c['t0'] + c['event_t'])
        meas = '(%.2f,%.2f)' % (m[sel][-1, 1], m[sel][-1, 2]) if sel.any() else 'n/a'
        if kh is not None:
            jk = np.abs(jerk[:max(kh - 2, 1)]).max()
            tail = (' capture_install=%d k_h=%d splice_k=%s leg_jerk_peak=%.0f plat_xy_speed_peak=%.1f'
                    ' plat_xy_at_k_h=(%.2f,%.2f) measured_xy_at_touchdown=%s' % (
                        i, kh, c['splice_k'], jk, xyv[:kh + 1].max(), pose[kh, 0], pose[kh, 1], meas))
        else:
            tail = ' capture_install=%d k_h=none (no contact knots)' % i
    else:
        tail = ' capture_install=none (no accepted CATCH install)'
    print('CAPTURE bias=(%g,%g) flight_lead=%g authority=%s seed=%d passed=%s makes=%s end_code=%s%s' % (
        bx, by, a.flight_lead, a.authority, a.seed, r.get('passed'), r.get('makes'), r.get('end_code'), tail))
    print('log: %s' % os.path.join(out, 'run.log'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
