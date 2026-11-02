#!/usr/bin/env python3
"""Where would the Hand stop if the scheduled stream ran dry near a release?

WHAT IT DOES
------------
Models the can-bridge scheduled lane (FW 35 ``leg_interp.cpp::sched_advance``):
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
15.5 rev (Apex 1.05); with the step the worst knot is the release itself, 14.6.
Neither fits the stroke, because the grace alone travels ~3 rev at 150 rev/s and
the planned brake already uses the stroke above the release.  Re-run it when
the stop limits, the grace or the cover change.

    python tools/probes/hand_stall_stop.py [--accel 3500] [--clamp-entry]
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--accel', type=float, default=float(hw.JB_TRAJ_STREAM_STOP_HAND_ACCEL_RPS2))
    ap.add_argument('--jerk', type=float, default=float(hw.JB_TRAJ_STREAM_STOP_HAND_JERK_RPS3))
    ap.add_argument('--clamp-entry', action='store_true',
                    help='start the stop from min(a_entry, 0) (a C1 stop entry)')
    ap.add_argument('apex', nargs='*', type=float, default=[0.95, 1.05, 1.2, 1.42])
    args = ap.parse_args()
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
            ends = []
            for kk in range(k - 3, k + 3):
                pe, ve, ae = span(p[kk - 1], v[kk - 1], p[kk], v[kk], S_MAX)
                if args.clamp_entry:
                    ae = min(ae, 0.0)
                ends.append((kk - k, pe + stop_travel(ve, ae, args.accel, args.jerk)))
            worst = max(e for _, e in ends)
            print('apex %.2f step=%d: stop end rev by cover-end knot (rel. release) %s worst %.2f %s'
                  % (apex, on, ' '.join('%+d:%.2f' % (d, e) for d, e in ends), worst,
                     'PAST CLIP' if worst > CLIP_REV else 'ok'))


if __name__ == '__main__':
    main()
