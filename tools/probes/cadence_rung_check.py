#!/usr/bin/env python3
"""Cadence-ladder rung checker — does a published rung actually THROW?

``tests/hardware/session_cadence_ladder.md`` publishes a table of
``(throw_height_m, dwell_time_s, throw_delay_s)`` triples and asks an operator to
arm the machine at each one.  Until 2026-08-22 that table was verified only as
far as *"the session accepts it AND the cycle's CHECKING pass reaches
POSITIONING"* — which is two gates short of the one that actually refuses a fast
rung.  ``ABORTED_CANT_MAKE_RELEASE`` is minted in ``_step_preparing``, i.e.
**after** POSITIONING, so a rung can clear both published checks and still abort
every cycle at ``cycle_start + 0.06 s`` with the hand retracting under a seated
ball.

This probe closes that gap.  For each rung it drives BOTH real FSMs, at the real
node tick, and reports the phase the cycle actually reaches:

  1. :class:`toss_session.TossSessionSequencer` — the session accept gate
     (``_checking_reject`` via a real ``step``), which must emit
     ``ACTION_START_CYCLE``.
  2. :class:`toss_sequencer.TossSequencer` — the cycle FSM, ticked at
     ``NODE_TICK_S`` and fed the **fastest node behaviour that is legal**:

       * every live precondition already true on the first tick,
       * POSITIONING answered on the very next tick (``note_position_noop`` on a
         LEVEL chain, ``note_position_result`` with the planner's
         ``min_move_duration_s`` floor on an AIMED one),
       * PREPARE succeeding on the next tick after that,
       * the announcement acknowledged instantly.

     Nothing here is optimistic in the machine's favour beyond what the node can
     actually do, so a rung this probe reds cannot be rescued by a faster node —
     and a rung it greens still has to survive the three synchronous service
     round-trips the PREPARE bundle makes, which this probe charges as zero.

**LEVEL vs AIMED is the load-bearing split.**  The census-B1 positioning skip
(``reload_coordinator_node._toss_already_positioned``) returns ``False`` on every
TILTED release by construction — ``trajectory/commanded_position`` carries no
orientation, so the node cannot verify it is already at a pre-tilt pose.  Every
cadence number in the ladder's § 2 table assumes the skip fires, i.e. assumes a
LEVEL chain; but the ladder is the runbook for arming ILC-primary, whose whole
output is a tilted release.  So each rung is reported twice and the AIMED column
is the one an ILC sitting flies.

Usage::

    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/probes/cadence_rung_check.py             # the published ladder
    python tools/probes/cadence_rung_check.py --solve     # + the minimum flyable
                                                          #   throw_delay per rung
    python tools/probes/cadence_rung_check.py --frontier  # + the fastest legal
                                                          #   cadence per flight

Read-only: constructs FSMs in-process, commands nothing, touches no hardware and
no ROS graph.  Outputs to stdout only (see ``tools/probes/README.md``).

Findings on the day it was written (2026-08-22, working tree at 78daf4b + the
ILC-primary audit fixes).  ``--frontier`` is monotone across the whole C-HAND-3
band, fastest at the FLOOR::

    fastest LEVEL  1.1051 s = 54.3 throws/min  (T 0.4949, delay 0.4168, dwell 0.6102)
    fastest AIMED  1.4850 s = 40.4 throws/min  (T 0.4949, delay 0.7968, dwell 0.9901)

The ladder's published operating point was **0.993 s = 60.4 throws/min**, which
is not reachable on either chain: the accept-time ``throw_delay`` floor models
the kind-0 dispatch budget alone, while the runtime guard measures the same
budget AFTER the whole pre-dispatch sequence has elapsed — 0.080 s of it on a
level chain and 0.460 s on an aimed one.  Closing that gap is a design decision
(the session cannot know at accept time whether the node will take the B1 skip),
so the ladder now publishes the reachable frontier and names the shortfall.
"""

from __future__ import annotations

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..'))
# FIRST on sys.path, deliberately: the shell profile exports a PYTHONPATH
# pointing at the colcon INSTALL tree, and a probe that silently measured the
# last `colcon build` instead of the working tree would be worse than no probe.
sys.path.insert(0, os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot'))

import jugglebot.hardware_config as hw                           # noqa: E402
from jugglebot import toss_sequencer as ts                       # noqa: E402
from jugglebot import toss_session as tsess                      # noqa: E402
from jugglebot.ball_possession import EVIDENCE_SEATED            # noqa: E402
from jugglebot.motion.trajectory import hand_stroke              # noqa: E402
from jugglebot.motion.trajectory.toss_release import (           # noqa: E402
    apex_height_from_flight_time, flight_time_from_height)
from jugglebot.toss_sequencer import (                           # noqa: E402
    FLIGHT_TIME_MAX_S, FLIGHT_TIME_MIN_S)

#: The reload coordinator's FSM tick, as of c938c1d (was 0.05 before B3).
NODE_TICK_S = 0.02

#: The planner floor a real (non-skipped) go_to_pose pays even for a zero-length
#: pre-tilt move.  Sourced from the generated config, never hardcoded.
MIN_MOVE_DURATION_S = float(hw.JB_TRAJ_MIN_MOVE_DURATION_S)

#: The ladder as published in tests/hardware/session_cadence_ladder.md § 2, in
#: BOTH of its variants: ``(name, throw_height_m, (dwell, delay) LEVEL,
#: (dwell, delay) AIMED)``.  R0-R3 use one pair for both chains — their delays
#: are seconds clear of every floor, so the B1 skip is not load-bearing there.
LADDER = [
    ('R0',       0.78, (5.60, 5.00), (5.60, 5.00)),
    ('R1',       0.78, (4.10, 3.50), (4.10, 3.50)),
    ('R2',       0.78, (3.00, 2.40), (3.00, 2.40)),
    ('R3',       0.78, (1.50, 0.90), (1.50, 0.90)),
    ('R4',       0.45, (0.65, 0.45), (1.00, 0.82)),
    ('R5',       0.31, (0.70, 0.47), (1.08, 0.85)),
    ('R5-prime', 0.31, (0.63, 0.43), (1.01, 0.81)),
]

#: The ladder as it was published from 78daf4b until the 2026-08-22 audit fix —
#: kept as the REGRESSION case, because three of its rungs abort every cycle and
#: a probe that can no longer show that has lost the finding.
LADDER_PRE_AUDIT = [
    ('R0',       0.78, 5.60, 5.00),
    ('R1',       0.78, 4.10, 3.50),
    ('R2',       0.78, 3.00, 2.40),
    ('R3',       0.78, 1.50, 0.90),
    ('R4',       0.45, 0.75, 0.55),
    ('R5',       0.31, 0.60, 0.40),
    ('R5-prime', 0.31, 0.49, 0.35),
]


def flight_for_height(h_m: float) -> float:
    """Apex height -> flight time, through the SAME single conversion the node
    uses (``reload_coordinator_node._resolve_toss_flight_s``)."""
    return flight_time_from_height(float(h_m))


def _observations(now: float, **over):
    """Every live precondition TRUE — the fastest legal machine."""
    kw = dict(
        control_mode=ts.TOSS_CONTROL_MODE, streaming=True, mocap_fresh=True,
        platform_levelled=True, hand_fresh=True, hand_parked=True,
        ball_seated=True, ball_evidence=EVIDENCE_SEATED,
        track_active=False, platform_at_target=True,
    )
    kw.update(over)
    return ts.TossObservations(now=now, **kw)


def run_cycle(T: float, delay_s: float, *, aimed: bool,
              max_ticks: int = 4000):
    """Drive one cycle FSM to its first terminal-or-dispatch outcome.

    Returns ``(verdict, t_reached)`` where verdict is ``'FLIES'`` or the FSM's
    own ``REJECTED_*``/``ABORTED_*`` outcome, and ``t_reached`` is the elapsed
    time from ``start()`` at which that happened.
    """
    T = float(T)
    seq = ts.TossSequencer(
        catch_pose_stow_mm=(0.0, 0.0, ts.TOSS_ACTIVE_Z_MM),
        flight_time_s=T,
        throw_delay_s=float(delay_s),
        event_vel_mps=ts.vertical_event_vel_mps(T),
        throw_site_known=True,
    )
    t0 = 0.0
    seq.start(t0)
    now = t0
    prepare_pending = False
    for _ in range(int(max_ticks)):
        dec = seq.step(now, _observations(now))
        if dec.action == ts.ACTION_DISPATCH_THROW:
            return 'FLIES', now - t0
        if dec.done and dec.result is not None:
            return dec.result.outcome, now - t0
        # The elif CHAIN below mirrors reload_coordinator_node._step_toss_sequence
        # exactly, including which branches are mutually exclusive. Getting this
        # wrong by one tick is a 20 ms error in a budget whose whole margin is
        # 3 ms, so it is modelled rather than approximated.
        if dec.action == ts.ACTION_POSITION_PLATFORM:
            # _position_platform_for_toss calls note_position_* SYNCHRONOUSLY,
            # inside this tick: the go_to_pose ack (or the B1 no-op decision) is
            # a blocking call on the coordinator thread.
            if aimed:
                # A tilted release can never take the B1 skip, so it pays the
                # planner's min_move_duration floor plus the settle pad.
                seq.note_position_result(now, True, MIN_MOVE_DURATION_S)
            else:
                seq.note_position_noop(now)
        elif dec.action == ts.ACTION_PREPARE_CATCH:
            # The verified-arrival tick raises catch/prime_hold ALONE and DEFERS
            # the bundle to the NEXT tick, so the two travel in different
            # catch_coordinator wait-set cycles. note_prepare_result therefore
            # lands one full tick later — and the bundle's three synchronous
            # service round-trips inside it are charged as zero here.
            prepare_pending = True
        elif dec.action == ts.ACTION_ANNOUNCE:
            seq.note_announcement()          # _announce_toss, synchronous
        elif prepare_pending:
            prepare_pending = False
            seq.note_prepare_result(True)
        now += NODE_TICK_S
    return 'NO_TERMINAL', now - t0


def session_accepts(T: float, dwell_s: float, delay_s: float):
    """The SESSION gate — returns ``None`` on accept, else its reject code."""
    sess = tsess.TossSessionSequencer(
        num_throws=5, dwell_time_s=float(dwell_s),
        throw_delay_s=float(delay_s), flight_time_s=float(T),
    )
    return sess._checking_reject()


def min_flyable_delay(T: float, *, aimed: bool,
                      hi: float = 3.0, tol: float = 1e-4) -> float:
    """Smallest ``throw_delay_s`` at which the cycle reaches DISPATCH_THROW.

    Monotone in ``throw_delay`` (a longer delay only ever grants MORE lead to
    the same fixed pre-dispatch sequence), so a bisection is exact."""
    lo = 0.0
    if run_cycle(T, hi, aimed=aimed)[0] != 'FLIES':
        return float('inf')
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if run_cycle(T, mid, aimed=aimed)[0] == 'FLIES':
            hi = mid
        else:
            lo = mid
    return hi


def fastest_at(T: float, *, aimed: bool):
    """The fastest LEGAL cycle at this flight time: ``(delay, dwell, period)``.

    ``delay`` is the smallest one the cycle FSM will fly; ``dwell`` is the
    smallest the session will accept AT that delay; ``period`` is what an
    operator actually counts (``dwell + T`` — landing to landing).
    """
    delay = min_flyable_delay(T, aimed=aimed)
    if not (delay < float('inf')):
        return float('inf'), float('inf'), float('inf')
    dwell = required_dwell(T, delay)
    return delay, dwell, dwell + float(T)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--solve', action='store_true',
                    help='also bisect the minimum flyable throw_delay per rung')
    ap.add_argument('--frontier', action='store_true',
                    help='sweep the C-HAND-3 band for the fastest legal cadence')
    args = ap.parse_args()

    print('node tick {:.3f} s | min_move_duration {:.3f} s | settle pad {:.3f} s'
          .format(NODE_TICK_S, MIN_MOVE_DURATION_S,
                  ts.TOSS_POSITION_SETTLE_PAD_S))
    print()
    print('THE PUBLISHED LADDER — each variant checked on ITS OWN chain')
    head = ('{:<9} {:>5} {:>7} {:>7} {:>7} | {:>6} {:>6} {:>10} {:>26}'
            .format('rung', 'h', 'T', 'floor', 'park',
                    'dwell', 'delay', 'session', 'cycle'))
    print(head)
    print('-' * len(head))
    ok = True
    for name, h, level, aimed in LADDER:
        T = flight_for_height(h)
        v = ts.vertical_event_vel_mps(T)
        floor = hand_stroke.min_throw_event_delay_s(v)
        park = hand_stroke.catch_park_reentry_s(v, tsess.DEFAULT_CATCH_VEL_SCALE)
        for chain, (dwell, delay) in (('LEVEL', level), ('AIMED', aimed)):
            rej = session_accepts(T, dwell, delay)
            got = run_cycle(T, delay, aimed=(chain == 'AIMED'))[0]
            ok = ok and rej is None and got == 'FLIES'
            print('{:<9} {:>5.2f} {:>7.4f} {:>7.4f} {:>7.4f} | {:>6.2f} {:>6.2f} '
                  '{:>10} {:>26}   {} period {:.4f} = {:.1f}/min'
                  .format(name, h, T, floor, park, dwell, delay,
                          rej or 'ACCEPT', got, chain, dwell + T,
                          60.0 / (dwell + T)))
    print()
    print('PUBLISHED LADDER: {}'.format('all rungs FLY' if ok else 'RED'))

    print()
    print('THE PRE-AUDIT LADDER (78daf4b) — the regression this probe found')
    for name, h, dwell, delay in LADDER_PRE_AUDIT:
        T = flight_for_height(h)
        rej = session_accepts(T, dwell, delay)
        lvl = run_cycle(T, delay, aimed=False)[0]
        aim = run_cycle(T, delay, aimed=True)[0]
        print('{:<9} dwell {:>4.2f} delay {:>4.2f} | session {:<14} '
              'LEVEL {:<26} AIMED {}'
              .format(name, dwell, delay, rej or 'ACCEPT', lvl, aim))

    if args.solve:
        print()
        print('{:<9} {:>7} {:>14} {:>14} {:>16} {:>16}'.format(
            'rung', 'T', 'min_delay_lvl', 'min_delay_aim',
            'req_dwell_lvl', 'req_dwell_aim'))
        for name, h, _level, _aimed in LADDER:
            T = flight_for_height(h)
            dl, rl, _ = fastest_at(T, aimed=False)
            da, ra, _ = fastest_at(T, aimed=True)
            print('{:<9} {:>7.4f} {:>14.4f} {:>14.4f} {:>16.4f} {:>16.4f}'
                  .format(name, T, dl, da, rl, ra))

    if args.frontier:
        print()
        print('THE REACHABLE FRONTIER — fastest legal cadence per flight time')
        print('{:>7} {:>6} | {:>7} {:>7} {:>7} {:>7} | {:>7} {:>7} {:>7} {:>7}'
              .format('T', 'apex', 'dly', 'dwell', 'period', 'tpm',
                      'dly', 'dwell', 'period', 'tpm'))
        print('{:>7} {:>6} | {:^31} | {:^31}'
              .format('', '', 'LEVEL chain', 'AIMED chain (ILC)'))
        best = {}
        T = float(FLIGHT_TIME_MIN_S)
        while T <= float(FLIGHT_TIME_MAX_S) + 1e-9:
            apex = apex_height_from_flight_time(T)
            row = [T, apex]
            for aimed in (False, True):
                d, w, p = fastest_at(T, aimed=aimed)
                row += [d, w, p, 60.0 / p]
                key = 'aimed' if aimed else 'level'
                if key not in best or p < best[key][0]:
                    best[key] = (p, T, apex, d, w)
            print('{:>7.4f} {:>6.3f} | {:>7.4f} {:>7.4f} {:>7.4f} {:>7.1f} | '
                  '{:>7.4f} {:>7.4f} {:>7.4f} {:>7.1f}'.format(*row))
            T += 0.05
        print()
        for key in ('level', 'aimed'):
            p, T, apex, d, w = best[key]
            print('FASTEST {:<6} period {:.4f} s = {:.1f} throws/min at '
                  'T {:.4f} (apex {:.3f} m), throw_delay {:.4f}, dwell {:.4f}'
                  .format(key, p, 60.0 / p, T, apex, d, w))
    return 0


def required_dwell(T: float, delay_s: float) -> float:
    """``TossSessionSequencer.required_dwell_s`` at this (T, delay)."""
    sess = tsess.TossSessionSequencer(
        num_throws=5, dwell_time_s=99.0, throw_delay_s=float(delay_s),
        flight_time_s=float(T),
    )
    return float(sess.required_dwell_s)


if __name__ == '__main__':
    raise SystemExit(main())
