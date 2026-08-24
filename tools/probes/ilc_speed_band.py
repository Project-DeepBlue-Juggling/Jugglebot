#!/usr/bin/env python3
"""Where is the ILC `event_vel_trim` actually ADMISSIBLE, and what bounds it?

    plans/active/critical-point-ilc.md § The 2026-08-21 fold-in (C2, and
        "Two consequences the fold-in derives" (b))
    plans/active/toss-selftuning.md § 11.5a
    logbook/2026-08-21-ilc-primary-foldin.md § Discussion

THE QUESTION
------------
``ilc_fit_lib.ILC_SPEED_AUTHORITY`` is a SCALAR ±0.15, approved at Gate 1 on the
argument that "neither ``event_vel`` rail is reachable anywhere in the
sequencer's flight-time band". That argument was made against the hand-picked
``[0.55, 1.10] s`` band and the bridge's ``[0.3, 7.0] m/s`` wire clamp. The band
is now DERIVED (contract C-HAND-3, ``throw_envelope``), and the wire clamp
"bounds nothing physical" — so the question has to be re-asked against the real
gate, which is ``throw_envelope.evaluate(T, v)``.

This probe sweeps ``k_v − 1`` at fixed flight time and reports the admitted
interval plus the bound that closes each side.

RESULT (run 2026-08-21, `mvp-trajectory-bringup` post ILC fold-in)
------------------------------------------------------------------
    T=0.4949 v0=2.4400  k_v-1 in [+0.000, >=+0.5]  neg=ARM_WINDOW
    T=0.5500 v0=2.7089  k_v-1 in [-0.409, >=+0.5]  neg=ARM_WINDOW
    T=0.9032 v0=4.4358  k_v-1 in [<=-0.5, +0.270]  pos=DECEL_FF_HEADROOM
    T=1.0000 v0=4.9097  k_v-1 in [<=-0.5, +0.148]  pos=DECEL_FF_HEADROOM
    T=1.1000 v0=5.3994  k_v-1 in [<=-0.5, +0.043]  pos=DECEL_FF_HEADROOM
    T=1.1485 v0=5.6368  k_v-1 in [<=-0.5, +0.000]  pos=DECEL_FF_HEADROOM

So ±0.15 is NOT admissible near either end, and the corpus's measured demand
(−0.1076, the ~11 % fast throw) is INADMISSIBLE at exactly ``T = 0.4949 s`` —
the R5-prime cadence target.

THE MECHANISM, which is the counter-intuitive half and the reason this is a
probe rather than an argument
-----------------------------------------------------------------------------
``arm_window_s(T, v) = latest − earliest`` where ``latest`` depends on **T
alone** and ``earliest = throw_decel_s(v) + ARM_SUPPRESS_MARGIN_S``. And
``throw_decel_s = INERTIA_RATIO · t_acc`` **GROWS as the release speed falls**
(0.10488 s at v = 2.440 m/s → 0.11654 s at v = 2.196), because a slower stroke
spends longer over the same stroke geometry. So a SLOW-DOWN trim pushes the
earliest admissible arm instant LATER and NARROWS the window. At the band floor
the window is exactly ``ARM_WINDOW_MARGIN_S`` (0.0500 s) wide by construction,
so any negative trim at all breaks it.

It is NOT "a slower throw shortens the achieved flight past
``ARM_WINDOW_CLOSES_AT_S``" — ``evaluate`` takes ``(T, v)`` as independent
arguments and never re-derives a flight time from ``v``. That reading was the
first draft's, and this probe is what refuted it.

RE-RUN IT after any change to ``throw_envelope``, ``hand_stroke``'s stroke
geometry, ``decel_ff_current_headroom_frac``, ``hand_curr_limit_a``, or the
catch-arm lead terms — the whole point of the fold-in's build step 1 is to
replace the scalar authority with this T-dependent band, and this is the
computation that produces it.

Offline and read-only: imports the production modules, touches no hardware, no
ROS2, no sockets, no files.

    python tools/probes/ilc_speed_band.py [--step 0.001] [--reach 0.5]
"""

from __future__ import annotations

import argparse
import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
_PKG = os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot')
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from jugglebot.motion.trajectory import hand_stroke as hs  # noqa: E402
from jugglebot.motion.trajectory import throw_envelope as te  # noqa: E402


def admitted_band(flight_s, step=0.001, reach=0.5):
    """``(v_nominal, lo, hi)`` — the admitted ``k_v - 1`` interval at ``T``.

    Scans outward from 0 in both directions and STOPS at the first refusal, so
    the answer is the connected interval around the nominal command — which is
    the only interval a trust-region step can walk into.  ``lo``/``hi`` saturate
    at ``-reach``/``+reach``.
    """
    v0 = te.vertical_release_speed_mps(flight_s)
    lo = hi = 0.0
    x = 0.0
    while x >= -reach and te.evaluate(flight_s, v0 * (1.0 + x)).ok:
        lo, x = x, x - step
    x = 0.0
    while x <= reach and te.evaluate(flight_s, v0 * (1.0 + x)).ok:
        hi, x = x, x + step
    return v0, lo, hi


def _bound_beyond(flight_s, v0, k, step):
    """The refusal reason just outside an edge (empty string if it saturated)."""
    verdict = te.evaluate(flight_s, v0 * (1.0 + k + step))
    return verdict.bound if not verdict.ok else ''


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--step', type=float, default=0.001,
                        help='trim resolution of the sweep (default 0.001)')
    parser.add_argument('--reach', type=float, default=0.5,
                        help='how far to sweep each way (default 0.5)')
    args = parser.parse_args(argv)

    print('derived band  [%.4f, %.4f] s   arm window closes at %.4f s   '
          'ILC_SPEED_AUTHORITY = %.2f'
          % (te.MIN_FLIGHT_TIME_S, te.MAX_FLIGHT_TIME_S,
             te.ARM_WINDOW_CLOSES_AT_S, 0.15))
    print()
    flights = (te.MIN_FLIGHT_TIME_S, 0.55, 0.9032, 1.0, 1.10,
               te.MAX_FLIGHT_TIME_S)
    for flight_s in flights:
        v0, lo, hi = admitted_band(flight_s, args.step, args.reach)
        neg = _bound_beyond(flight_s, v0, lo, -args.step)
        pos = _bound_beyond(flight_s, v0, hi, args.step)
        print('T=%.4f v0=%.4f  k_v-1 in [%+.3f, %+.3f]  neg=%-18s pos=%-18s '
              'window=%.4f'
              % (flight_s, v0, lo, hi, neg or '(saturated)',
                 pos or '(saturated)', te.arm_window_s(flight_s, v0)))

    print('\nmechanism at the band floor -- throw_decel_s vs release speed:')
    flight_s = te.MIN_FLIGHT_TIME_S
    v0 = te.vertical_release_speed_mps(flight_s)
    for k in (0.10, 0.05, 0.0, -0.02, -0.05, -0.10):
        v = v0 * (1.0 + k)
        print('  k=%+0.2f v=%.4f  t_dec=%.5f  earliest=%.5f  window=%.5f'
              % (k, v, hs.throw_decel_s(v),
                 hs.throw_decel_s(v) + hs.ARM_SUPPRESS_MARGIN_S,
                 te.arm_window_s(flight_s, v)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
