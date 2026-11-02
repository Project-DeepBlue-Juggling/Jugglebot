#!/usr/bin/env python3
"""The Hand's launch and brake shape around a release, continuous vs STEPPED.

WHAT IT DOES
------------
Plans the columns-site self-throw (release 860, hand limit 4500) through the
production chain, a THROW from REST and the folded CATCH-with-throw (STEADY)
seeded at its release, once with ``unified_cycle.RELEASE_ACC_STEP`` off and
once on, per Apex, and prints the launch and brake peaks against their
constant-acceleration ideals (kappa = peak / (v^2 / 2d)) plus the gate verdict.

WHY IT EXISTS
-------------
C-HAND-5 (logbook ``2026-11-02-hand-release-step``) rests on these numbers: the
continuous release ran launch / brake kappa 1.81 / 1.56-1.68 and refused Apex
1.42; the stepped one runs ~1.3 / ~1.1 and plans Apex 1.5.  Re-run it after any
change to the release rows, the brake seed or the objective.

    python tools/probes/release_step_shape.py [apex ...]     # default 0.95 1.05 1.2 1.42
"""
from __future__ import annotations

import os
import sys

for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for _p in (ROOT, os.path.join(ROOT, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(ROOT, 'tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np                                    # noqa: E402
import admissible_sweep as A                          # noqa: E402
from jugglebot.motion import unified_cycle as uc      # noqa: E402
from jugglebot.motion.skills import schedule as sc    # noqa: E402
from jugglebot.motion.skills import segments as sg    # noqa: E402
from jugglebot.motion.skills import sites as st       # noqa: E402

GEOM = A.StewartGeometry()
LIM = A._limits(350, 5000, 200000, 4500)
SITE, _ = st.columns_sites(150)


def hermite_acc(plan):
    """``(a_right, a_left)`` of the hand at each knot."""
    p, v, h = plan.hand_rev, plan.hand_vel_rps, plan.dt
    a_right = (-6 * p[:-1] + 6 * p[1:]) / h ** 2 + (-4 * v[:-1] - 2 * v[1:]) / h
    a_left = (6 * p[:-1] - 6 * p[1:]) / h ** 2 + (2 * v[:-1] + 4 * v[1:]) / h
    return a_right, a_left


def kappas(plan, k_rel, k_start=0, brake_from=None):
    ar, al = hermite_acc(plan)
    p, v = plan.hand_rev, plan.hand_vel_rps
    out = {}
    if k_rel is not None:
        pk = float(np.max(np.r_[ar[k_start:k_rel], al[k_start:k_rel]]))
        ideal = v[k_rel] ** 2 / (2 * (p[k_rel] - p[k_start:k_rel].min()))
        out['launch'] = (pk, pk / ideal)
    if brake_from is not None:
        k0 = brake_from
        k_stop = k0 + int(np.argmax(v[k0:] <= 0.5))
        pk = float(-np.min(np.r_[ar[k0:k_stop], al[k0:k_stop]]))
        ideal = v[k0] ** 2 / (2 * (p[k_stop] - p[k0]))
        out['brake'] = (pk, pk / ideal)
    return out


def plan_pair(apex):
    T = sc.flight_s(apex)
    term = sg.ThrowTerminal(site_mm=SITE.throw_site_mm(), target_mm=SITE.throw_site_mm(),
                            flight_s=T, t_release_s=0.4)
    seg0 = sg.plan_segment(sg.THROW, A._rest_state(SITE.rest_site_mm()), term,
                           sg.SegmentConfig(), LIM, GEOM)
    tt = sg.ThrowAfterCatch(t_release_s=T + 0.30, site_mm=SITE.throw_site_mm(),
                            target_mm=SITE.throw_site_mm(), flight_s=T)
    cterm = sg.CatchTerminal(landing_mm=SITE.catch_site_mm(),
                             landing_vel_mm_s=A.bal.arrival_velocity(seg0.takeoff_vel_mm_s, T),
                             t_land_s=T, rest_site_mm=SITE.rest_site_mm(), then_throw=tt)
    seg1 = sg.plan_segment(sg.CATCH, seg0.release_state, cterm, sg.SegmentConfig(), LIM, GEOM)
    return seg0, seg1, T


def main(apexes):
    for on in (False, True):
        uc.RELEASE_ACC_STEP = on
        for apex in apexes:
            try:
                seg0, seg1, T = plan_pair(apex)
            except Exception as exc:              # a refusal is a result here
                print('step=%d apex %.2f: REFUSED %s' % (on, apex, str(exc)[:140]))
                continue
            k0 = int(round(seg0.event_t_s / seg0.plan.dt))
            a = kappas(seg0.plan, k0, brake_from=k0)
            kr = int(round(seg1.release_t_s / seg1.plan.dt))
            b = kappas(seg1.plan, kr, int(round(T / seg1.plan.dt)), brake_from=0)
            print('step=%d apex %.2f: THROW launch %4.0f (k %.2f) brake %4.0f (k %.2f) | '
                  'STEADY launch %4.0f (k %.2f) brake %4.0f (k %.2f) | peak %4.0f %s'
                  % (on, apex, *a['launch'], *a['brake'], *b['launch'], *b['brake'],
                     seg1.meta.report.peak_hand_acc_rps2, seg1.meta.report.code))


if __name__ == '__main__':
    main([float(x) for x in sys.argv[1:]] or [0.95, 1.05, 1.2, 1.42])
