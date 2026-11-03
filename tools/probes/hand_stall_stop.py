#!/usr/bin/env python3
"""Where would the Hand stop if the scheduled stream ran dry near a release?

WHAT IT DOES
------------
Models the can-bridge scheduled lane (FW 36 ``leg_interp.cpp::sched_advance``;
constants re-checked 2026-11-03 against canbridge_config.h / hardware_config.h:
SCHED_GRACE_S 0.020, Hand stop 4500 rev/s^2 and 350k rev/s^3, knot 25 ms; FW 35
-> 36 was the Hand stall guard in leg_activate.cpp and does not touch the lane):
a frame covers two 25 ms spans; if no newer frame arrives, the lane continues
the last span's cubic for ``SCHED_GRACE_S`` (20 ms, ``s`` up to 1.8) and then
runs the C2 stop from that state (peak |a| <= max(A, |a_entry|), |jerk| <= J).
For each knot around the release of a production self-throw (release 860) it
assumes the cover ends on that knot and prints the rev the stop ends at,
against the 10.501 rev stroke clip (10.701 hard stop).

WHY IT EXISTS
-------------
Found while landing C-HAND-5 (logbook ``2026-11-02-hand-release-step``): with
the continuous release a cover ending one knot before the release stops at
15.5 rev (Apex 1.05); with the step the worst knot is the release itself, 14.6
(both at the then stop accel 3500; the stop accel is 4500 since 8ffeba66 and the
default run now gives 15.3 / 13.2; ``--accel 3500`` reproduces the original).
Apex 1.42 with the continuous release is now REFUSED (peak 4859 > 4500).
Neither fits the stroke, because the grace alone travels ~3 rev at 150 rev/s and
the planned brake already uses the stroke above the release.  Re-run it when
the stop limits, the grace or the cover change.

OPTIONS (JB-145 / JB-154, which firmware change fits the stroke?)
-----------------------------------------------------------------
FW 37 IMPLEMENTS OPTION 2 (logbook ``2026-11-03-jb145-hand-stall-stop-options``,
``leg_interp.cpp::hand_c1_stop_eval``): ``--option 2`` reproduces the firmware's
stop height to within +0.03 rev, the one difference being that FW 37 also
jerk-limits the ramp-OUT (a back to 0 at standstill, +A^3/(24 J^2) = 0.031 rev);
the firmware threshold is strict (v > 50 rev/s) where the probe uses >=.
``--option baseline`` is FW 36 and earlier.

``--option`` (default ``baseline`` = the FW 36 lane, output unchanged;
``all`` prints every option):
  baseline  grace (cubic continues 20 ms) then the C2 stop (a0 kept, jerk-limited).
  1         "no grace for a fast upward Hand": if v at the cover end is > 0 and
            >= ``--fast-up-thresh`` (50 rev/s) the stop starts AT the cover end
            (same C2 stop); below the threshold the baseline is kept.
  2         option 1 plus a C1-entry stop: p and v continuous, acceleration steps
            to -``--stop-accel`` (4500) at once, no jerk ramp-in, travel v^2/(2A)
            (the jerk-limited return of a to 0 at standstill is not modelled).
  2-jerk    as 2 but the ramp-in from a_entry to -A and the ramp-out to 0 obey the
            jerk limit (``--jerk``): shows what the jerk limit costs.
Sanity: a stop from v at constant A travels v^2/(2A) (150 rev/s, 4500: 2.5 rev).

    python tools/probes/hand_stall_stop.py [--accel 4500] [--clamp-entry]
        [--option baseline|1|2|2-jerk|all] [--fast-up-thresh 50] [--stop-accel 4500]
"""
from __future__ import annotations

import argparse
import os
import sys

for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for _p in (ROOT, os.path.join(ROOT, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(ROOT, 'tools'), os.path.dirname(os.path.abspath(__file__))):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np                                    # noqa: E402
import jugglebot.hardware_config as hw                # noqa: E402
from jugglebot.motion import unified_cycle as uc      # noqa: E402
import release_step_shape as rs                       # noqa: E402

T = float(hw.JB_TRAJ_KNOT_DT_S)
GRACE_S = 0.020                                       # canbridge_config.h SCHED_GRACE_S
S_MAX = 1.0 + GRACE_S / T
CLIP_REV = 10.501                                     # HAND_MOTOR_MAX_POSITION


def span(p0, v0, p1, v1, s):
    s2, s3 = s * s, s ** 3
    p = (2 * s3 - 3 * s2 + 1) * p0 + (s3 - 2 * s2 + s) * T * v0 \
        + (-2 * s3 + 3 * s2) * p1 + (s3 - s2) * T * v1
    v = ((6 * s2 - 6 * s) / T) * p0 + (3 * s2 - 4 * s + 1) * v0 \
        + ((-6 * s2 + 6 * s) / T) * p1 + (3 * s2 - 2 * s) * v1
    a = ((6 - 12 * s) * (p1 - p0) / T + (6 * s - 4) * v0 + (6 * s - 2) * v1) / T
    return p, v, a


def stop_travel(v0, a0, accel, jerk):
    """Travel (rev) of the firmware's C2 stop from (v0, a0): the smallest D whose
    profile a(tau) = c2 tau^2 + c1 tau + c0 meets both peaks (bisection)."""
    if v0 <= 0.0:
        return 0.0

    def coeffs(D):
        x = v0 / D
        return a0, -6 * x - 4 * a0, 6 * x + 3 * a0

    tau = np.linspace(0.0, 1.0, 801)

    def ok(D):
        c0, c1, c2 = coeffs(D)
        a = c2 * tau * tau + c1 * tau + c0
        j = (2 * c2 * tau + c1) / D
        return (np.max(np.abs(a)) <= max(accel, abs(a0)) + 1e-6
                and np.max(np.abs(j)) <= jerk + 1e-6)

    lo, hi = 1e-4, 0.5
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        lo, hi = (lo, mid) if ok(mid) else (mid, hi)
    c0, c1, c2 = coeffs(hi)
    t = np.linspace(0.0, 1.0, 2001)
    a = c2 * t * t + c1 * t + c0
    v = v0 + hi * np.concatenate([[0.0], np.cumsum(0.5 * (a[1:] + a[:-1]) * np.diff(t))])
    return hi * float(np.trapz(v, t))


def stop_travel_c1(v0, a0, A, jerk=None):
    """Travel (rev) of a C1-entry stop.  jerk None: a steps to -A at once (v0^2/2A).
    Else: a ramps from a0 to -A (and back to 0) at +-jerk (integrated, dt 20 us)."""
    if v0 <= 0.0:
        return 0.0
    if jerk is None:
        return v0 * v0 / (2.0 * A)
    dt, x, v, a = 2e-5, 0.0, v0, a0
    ramp_up = False
    while v > 1e-9:
        if not ramp_up and v <= a * a / (2.0 * jerk) and a < 0.0:
            ramp_up = True
        if ramp_up:
            a = min(a + jerk * dt, 0.0)
        else:
            a = max(a - jerk * dt, -A)
        v += a * dt
        x += v * dt
        if ramp_up and a >= 0.0:
            break
    return x


def cover_end_stop(opt, sp1, args):
    """End position and entry state of the stop for option `opt`, given the span-2
    segment (p0, v0, p1, v1).  Returns (end_rev, v_entry, a_entry, started_at)."""
    p0, v0, p1, v1 = sp1
    if opt != 'baseline':
        pc, vc, ac = span(p0, v0, p1, v1, 1.0)
        if vc >= args.fast_up_thresh:                  # fast upward Hand: no grace
            if opt == '1':
                a_in = min(ac, 0.0) if args.clamp_entry else ac
                return pc + stop_travel(vc, a_in, args.accel, args.jerk), vc, a_in, 'cover end'
            jerk = args.jerk if opt == '2-jerk' else None
            return pc + stop_travel_c1(vc, ac, args.stop_accel, jerk), vc, ac, 'cover end'
    pe, ve, ae = span(p0, v0, p1, v1, S_MAX)           # below threshold: today's lane
    if args.clamp_entry:
        ae = min(ae, 0.0)
    return pe + stop_travel(ve, ae, args.accel, args.jerk), ve, ae, 'grace end'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--accel', type=float, default=float(hw.JB_TRAJ_STREAM_STOP_HAND_ACCEL_RPS2))
    ap.add_argument('--jerk', type=float, default=float(hw.JB_TRAJ_STREAM_STOP_HAND_JERK_RPS3))
    ap.add_argument('--clamp-entry', action='store_true',
                    help='start the stop from min(a_entry, 0) (a C1 stop entry)')
    ap.add_argument('--option', default='baseline',
                    choices=['baseline', '1', '2', '2-jerk', 'all'])
    ap.add_argument('--fast-up-thresh', type=float, default=50.0,
                    help='rev/s: options 1/2 skip the grace above this upward v')
    ap.add_argument('--stop-accel', type=float, default=4500.0,
                    help='rev/s^2: option 2 C1-entry stop deceleration')
    ap.add_argument('apex', nargs='*', type=float, default=[0.95, 1.05, 1.2, 1.42])
    args = ap.parse_args()
    opts = ['baseline', '1', '2', '2-jerk'] if args.option == 'all' else [args.option]
    for apex in args.apex:
        for on in (False, True):
            uc.RELEASE_ACC_STEP = on
            try:
                seg, _, _ = rs.plan_pair(apex)
            except Exception as exc:
                print('apex %.2f step=%d: REFUSED %s' % (apex, on, str(exc)[:100]))
                continue
            k = int(round(seg.event_t_s / seg.plan.dt))
            p, v = seg.plan.hand_rev, seg.plan.hand_vel_rps
            for opt in opts:
                ends = []
                for kk in range(k - 3, k + 3):
                    ends.append((kk - k,) + cover_end_stop(
                        opt, (p[kk - 1], v[kk - 1], p[kk], v[kk]), args))
                dw, worst, vw, aw, how = max(ends, key=lambda e: e[1])
                tag = '' if args.option == 'baseline' else ' opt=%s' % opt
                print('apex %.2f step=%d%s: stop end rev by cover-end knot (rel. release) %s '
                      'worst %.2f %s' % (apex, on, tag,
                                         ' '.join('%+d:%.2f' % (e[0], e[1]) for e in ends),
                                         worst, 'PAST CLIP' if worst > CLIP_REV else 'ok'))
                if args.option != 'baseline':
                    print('    worst knot %+d: entry v %.1f rev/s a %.0f rev/s^2 (%s), '
                          'margin to clip %+.2f, to hard stop %+.2f rev'
                          % (dw, vw, aw, how, CLIP_REV - worst, CLIP_REV + 0.2 - worst))


if __name__ == '__main__':
    main()
