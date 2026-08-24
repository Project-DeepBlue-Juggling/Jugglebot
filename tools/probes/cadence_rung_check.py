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
       * POSITIONING answered on the very next tick (``note_position_noop`` when
         the census-B1 skip fires, ``note_position_result`` with the planner's
         ``min_move_duration_s`` floor when it does not),
       * PREPARE succeeding on the next tick after that,
       * the announcement acknowledged instantly.

     Nothing here is optimistic in the machine's favour beyond what the node can
     actually do, so a rung this probe reds cannot be rescued by a faster node —
     and a rung it greens still has to survive the three synchronous service
     round-trips the PREPARE bundle makes, which this probe charges as zero.

**THE SPLIT THAT USED TO BE HERE IS GONE (2026-08-23).**  From 2026-08-22 this
probe reported every rung twice, on a LEVEL chain and an AIMED one 0.38 s apart,
because ``reload_coordinator_node._toss_already_positioned`` returned ``False``
on every TILTED release by construction — ``trajectory/commanded_position``
carried no orientation, so the node could not verify a pre-tilt pose and had to
command a zero-millimetre move on every cycle of every aimed sitting.  That
orientation is now published (``trajectory/commanded_pose``, intent frame), so
the skip works on an aimed chain and the discriminator became **"does POSITIONING
command a move"**, which is a per-CYCLE fact rather than a per-SITTING one:

  * the FIRST cycle of a sitting commands the move (the platform is not yet at
    the pre-tilt pose).  The node grants it the extra lead —
    ``_build_toss_cycle(delay_is_cadence=True)`` — so it flies rather than dying;
  * every CHAINED cycle takes the skip, because a CAUGHT toss ends in
    ``ACTION_STAY`` holding the pose it threw and caught from.

So each rung is checked on BOTH — ``chained`` is the steady state an operator
counts, ``first`` is the one cycle that pays — and twice more for
``ilc_speed_trim_possible``, because a loaded layer-3 artifact makes the session
judge every floor at the slowest release its apply seam could command.

Usage::

    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/probes/cadence_rung_check.py             # the published ladder
    python tools/probes/cadence_rung_check.py --solve     # + the per-rung floors
    python tools/probes/cadence_rung_check.py --frontier  # + the fastest legal
                                                          #   cadence per flight
    python tools/probes/cadence_rung_check.py --grid      # + accept-implies-flies
                                                          #   over the whole grid

Read-only: constructs FSMs in-process, commands nothing, touches no hardware and
no ROS graph.  Outputs to stdout only (see ``tools/probes/README.md``).
Exit code 1 if any rung reds or the grid finds a violation.

Findings on the tree of 2026-08-23, after the accept-floor redesign and the B1
orientation surface.  ``--frontier`` is monotone across the whole C-HAND-3 band,
fastest at the FLOOR, and the two columns MEET there (the throw envelope refuses
a negative speed trim at the band floor, so an armed ILC costs nothing)::

    fastest, aim disarmed  1.1050 s = 54.3 throws/min  (T 0.4949, delay 0.4168, dwell 0.6101)
    fastest, ILC loaded    1.1050 s = 54.3 throws/min  (T 0.4949, delay 0.4168, dwell 0.6101)

The aimed frontier was **40.4 throws/min** on 2026-08-22.  The whole 34 % is the
B1 skip becoming reachable on a tilted release; the accept-floor redesign bought
no cadence at all, and was never going to — what it bought is that a rung the
gates accept cannot abort mid-sequence with the hand committed.
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

#: The ladder as published in tests/hardware/session_cadence_ladder.md § 2:
#: ``(name, throw_height_m, dwell, delay)`` — ONE pair per rung again.
#:
#: It carried TWO pairs from 2026-08-22 to 2026-08-23, a LEVEL one and an AIMED
#: one 0.38 s apart, because the B1 positioning skip could not verify a tilted
#: pose and every aimed cycle therefore commanded a zero-millimetre move. With
#: the orientation published that split is gone: a CHAINED cycle takes the skip
#: whatever its tilt, so one pair serves both. Each pair below is legal with a
#: layer-3 artifact LOADED as well as without it — the stricter case, because
#: this runbook exists to arm ILC-primary.
LADDER = [
    ('R0',       0.78, 5.60, 5.00),
    ('R1',       0.78, 4.10, 3.50),
    ('R2',       0.78, 3.00, 2.40),
    ('R3',       0.78, 1.50, 0.90),
    ('R4',       0.45, 0.65, 0.45),
    ('R5',       0.31, 0.70, 0.47),
    ('R5-prime', 0.31, 0.66, 0.44),
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

    ``aimed`` is now exactly "does POSITIONING command a move" — the same
    boolean the node caches at cycle build and feeds the FSM as
    ``positioning_move_expected``. Before 2026-08-23 it also meant "is the
    release tilted", because the B1 skip refused every tilted release; the two
    stopped being the same question when the skip learned to verify an
    orientation, so the AIMED column is now the FIRST cycle of a sitting (or any
    cycle after a go_home / reload) and the LEVEL column is every chained cycle,
    aimed or not.
    """
    T = float(T)
    seq = ts.TossSequencer(
        catch_pose_stow_mm=(0.0, 0.0, ts.TOSS_ACTIVE_Z_MM),
        flight_time_s=T,
        throw_delay_s=float(delay_s),
        event_vel_mps=ts.vertical_event_vel_mps(T),
        throw_site_known=True,
        positioning_move_expected=bool(aimed),
    )
    t0 = 0.0
    seq.start(t0)
    now = t0
    prepare_pending = False
    # `now` is recomputed as t0 + k*tick from an INTEGER k rather than
    # accumulated, deliberately: 23 additions of 0.02 land 1.1e-16 above
    # 23*0.02, and the release-window guard at the last tick compares against a
    # budget that a delay sitting exactly on the accept floor matches EXACTLY.
    # Accumulating turns that equality into an abort and makes the probe report
    # a violation the arithmetic does not have.
    for tick in range(int(max_ticks)):
        now = t0 + tick * NODE_TICK_S
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
    return 'NO_TERMINAL', now - t0


def session_accepts(T: float, dwell_s: float, delay_s: float,
                    *, ilc_trim: bool = False):
    """The SESSION gate — returns ``None`` on accept, else its reject code.

    ``ilc_trim`` is ``ilc_speed_trim_possible``: with a layer-3 artifact loaded
    the session judges every floor at the SLOWEST release the apply seam could
    command, not the untrimmed one. Defaults False — the shipped machine
    (``JB_OP_TOSS_ILC_ENABLED`` is false) and the ladder's published rungs."""
    sess = tsess.TossSessionSequencer(
        num_throws=5, dwell_time_s=float(dwell_s),
        throw_delay_s=float(delay_s), flight_time_s=float(T),
        ilc_speed_trim_possible=bool(ilc_trim),
    )
    return sess._checking_reject()


def accept_implies_flies(T: float, dwell_s: float, delay_s: float,
                         *, aimed: bool, ilc_trim: bool = False,
                         strict: bool = False):
    """THE contract, evaluated at one grid point.

    Returns ``None`` when the point is consistent, else a string naming the
    inconsistency.  Two strengths, and both are pinned by
    ``tests/motion/test_cadence_rung_check.py``:

    * always — a goal the SESSION accepts must never die
      ``ABORTED_CANT_MAKE_RELEASE``.  That terminal is minted in
      ``_step_preparing``, i.e. with the catch latch UP, the announcement out
      and the hand committed, and its cleanup retracts the hand under a seated
      ball.  Nothing about the goal's static arithmetic may lead there.
    * ``strict`` (the CHAINED cycle, ``aimed=False``) — the session's accept must
      also imply the CYCLE's own CHECKING accept.  The session mirrors the
      steady-state cycle, so any daylight between them at ``positioning_move =
      False`` is the two gates having drifted apart again.

    On a cycle that must COMMAND its positioning move (``aimed``, i.e. the first
    cycle of a sitting) a ``REJECTED_CANT_MAKE_LEAD`` after a session accept is
    the DESIGNED outcome and not a violation: the session's floor is the chained
    steady state, and the node grants that one cycle the extra lead rather than
    refusing (``_build_toss_cycle(delay_is_cadence=True)``).  This probe drives
    the FSM directly, without the node, so it sees the un-granted case."""
    session = session_accepts(T, dwell_s, delay_s, ilc_trim=ilc_trim)
    if session is not None:
        return None
    verdict, when = run_cycle(T, delay_s, aimed=aimed)
    if verdict == 'FLIES':
        return None
    if verdict.startswith('REJECTED_'):
        # A pre-throw refusal at CHECKING: nothing moved, nothing armed.
        if strict:
            return 'session ACCEPTED but the cycle refused {}'.format(verdict)
        if verdict.startswith('REJECTED_CANT_MAKE_LEAD'):
            return None
        return 'unexpected CHECKING refusal {}'.format(verdict)
    return ('session ACCEPTED but the cycle died {} at +{:.3f} s'
            .format(verdict, when))


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


def fastest_at(T: float, *, ilc_trim: bool = False):
    """The fastest LEGAL cadence at this flight time: ``(delay, dwell, period)``.

    A SESSION question, not a cycle one — an operator counts landing-to-landing,
    and what bounds that is what the session will ACCEPT. ``delay`` is
    ``TossSessionSequencer.min_throw_delay_s`` (the kind-0 dispatch budget plus
    the CHAINED pre-dispatch sequence); ``dwell`` is the smallest
    ``required_dwell_s`` at that delay; ``period`` is ``dwell + T``.

    ``ilc_trim`` is ``ilc_speed_trim_possible``: with a layer-3 artifact loaded
    every floor is judged at the slowest release the apply seam could command.
    """
    delay = session_floor_delay(T, ilc_trim=ilc_trim)
    dwell = required_dwell(T, delay, ilc_trim=ilc_trim)
    return delay, dwell, dwell + float(T)


def grid_violations(*, verbose: bool = False):
    """Sweep the whole ``(T, dwell, delay, aim)`` grid for contract violations.

    Returns a list of ``(T, dwell, delay, aimed, ilc_trim, reason)``. Empty is
    the contract holding. The grid is deliberately coarse in ``T`` and fine
    around the FLOORS, because that is where accept-vs-runtime disagreements
    live: a delay seconds clear of every gate cannot expose one."""
    out = []
    T = float(FLIGHT_TIME_MIN_S)
    while T <= float(FLIGHT_TIME_MAX_S) + 1e-9:
        for ilc_trim in (False, True):
            sess = tsess.TossSessionSequencer(
                num_throws=5, dwell_time_s=99.0, throw_delay_s=1.0,
                flight_time_s=T, ilc_speed_trim_possible=ilc_trim)
            floor = sess.min_throw_delay_s
            # Straddle the floor tightly, then walk out to the operator-scale
            # delays. ±1 ms either side of a floor is where an off-by-a-tick in
            # either gate shows up.
            delays = [floor - 0.001, floor, floor + 0.001,
                      floor + 0.02, floor + 0.10, 0.90, 2.40, 5.00]
            for delay in delays:
                if delay <= 0.0:
                    continue
                probe = tsess.TossSessionSequencer(
                    num_throws=5, dwell_time_s=99.0, throw_delay_s=delay,
                    flight_time_s=T, ilc_speed_trim_possible=ilc_trim)
                req = probe.required_dwell_s
                for dwell in (req - 0.001, req, req + 0.05, req + 0.50):
                    if dwell <= 0.0:
                        continue
                    for aimed in (False, True):
                        why = accept_implies_flies(
                            T, dwell, delay, aimed=aimed, ilc_trim=ilc_trim,
                            strict=not aimed)
                        if why is not None:
                            out.append((T, dwell, delay, aimed, ilc_trim, why))
                            if verbose:
                                print('  VIOLATION T={:.4f} dwell={:.4f} '
                                      'delay={:.4f} {} ilc_trim={}: {}'
                                      .format(T, dwell, delay,
                                              'first' if aimed else 'chained',
                                              ilc_trim, why))
        T += 0.05
    return out


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--solve', action='store_true',
                    help='also bisect the minimum flyable throw_delay per rung')
    ap.add_argument('--frontier', action='store_true',
                    help='sweep the C-HAND-3 band for the fastest legal cadence')
    ap.add_argument('--grid', action='store_true',
                    help='sweep (T, dwell, delay, aim) for accept-implies-flies '
                         'violations')
    args = ap.parse_args()

    print('node tick {:.3f} s | min_move_duration {:.3f} s | settle pad {:.3f} s'
          .format(NODE_TICK_S, MIN_MOVE_DURATION_S,
                  ts.TOSS_POSITION_SETTLE_PAD_S))
    print()
    print('THE PUBLISHED LADDER — every rung, ILC artifact loaded AND not')
    head = ('{:<9} {:>5} {:>7} {:>7} {:>7} | {:>6} {:>6} {:>5} {:>10} {:>7} {:>7}'
            .format('rung', 'h', 'T', 'floor', 'park', 'dwell', 'delay',
                    'ilc', 'session', 'chained', 'first'))
    print(head)
    print('-' * len(head))
    ok = True
    for name, h, dwell, delay in LADDER:
        T = flight_for_height(h)
        v = ts.vertical_event_vel_mps(T)
        floor = hand_stroke.min_throw_event_delay_s(v)
        park = hand_stroke.catch_park_reentry_s(v, tsess.DEFAULT_CATCH_VEL_SCALE)
        for ilc in (False, True):
            rej = session_accepts(T, dwell, delay, ilc_trim=ilc)
            # The CHAINED cycle: POSITIONING takes the census-B1 skip, because a
            # CAUGHT toss left the platform holding the pose it threw from.
            chained = run_cycle(T, delay, aimed=False)[0]
            # …and the FIRST cycle of the sitting, which must command the move.
            # The node grants it the extra lead (`_build_toss_cycle` with
            # delay_is_cadence=True), so the check is against the GRANTED delay,
            # exactly as the session path will run it.
            granted = max(float(delay),
                          ts.min_throw_delay_for_release_s(v, True))
            first = run_cycle(T, granted, aimed=True)[0]
            ok = ok and rej is None and chained == 'FLIES' and first == 'FLIES'
            print('{:<9} {:>5.2f} {:>7.4f} {:>7.4f} {:>7.4f} | {:>6.2f} {:>6.2f} '
                  '{:>5} {:>10} {:>7} {:>7}   period {:.4f} = {:.1f}/min'
                  .format(name, h, T, floor, park, dwell, delay,
                          'ON' if ilc else 'off', rej or 'ACCEPT',
                          chained, first, dwell + T, 60.0 / (dwell + T)))
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
              'chained {:<26} first {}'
              .format(name, dwell, delay, rej or 'ACCEPT', lvl, aim))

    if args.solve:
        print()
        print('PER-RUNG FLOORS — the smallest (delay, dwell) the SESSION admits')
        print('{:<9} {:>7} {:>12} {:>12} {:>12} {:>12}'.format(
            'rung', 'T', 'delay_ilc0', 'delay_ilc1',
            'dwell_ilc0', 'dwell_ilc1'))
        for name, h, _dwell, _delay in LADDER:
            T = flight_for_height(h)
            d0, w0, _ = fastest_at(T, ilc_trim=False)
            d1, w1, _ = fastest_at(T, ilc_trim=True)
            print('{:<9} {:>7.4f} {:>12.4f} {:>12.4f} {:>12.4f} {:>12.4f}'
                  .format(name, T, d0, d1, w0, w1))
        print()
        print('…and the CYCLE FSM\'s own bisected minimum flyable delay, both '
              'sequences:')
        print('{:<9} {:>7} {:>14} {:>14}'.format(
            'rung', 'T', 'chained', 'first(move)'))
        for name, h, _dwell, _delay in LADDER:
            T = flight_for_height(h)
            print('{:<9} {:>7.4f} {:>14.4f} {:>14.4f}'.format(
                name, T, min_flyable_delay(T, aimed=False),
                min_flyable_delay(T, aimed=True)))

    if args.frontier:
        print()
        print('THE REACHABLE FRONTIER — fastest legal cadence per flight time')
        print('{:>7} {:>6} | {:>7} {:>7} {:>7} {:>7} | {:>7} {:>7} {:>7} {:>7}'
              .format('T', 'apex', 'dly', 'dwell', 'period', 'tpm',
                      'dly', 'dwell', 'period', 'tpm'))
        print('{:>7} {:>6} | {:^31} | {:^31}'
              .format('', '', 'aim DISARMED', 'ILC artifact LOADED'))
        best = {}
        T = float(FLIGHT_TIME_MIN_S)
        while T <= float(FLIGHT_TIME_MAX_S) + 1e-9:
            apex = apex_height_from_flight_time(T)
            row = [T, apex]
            for ilc in (False, True):
                d, w, p = fastest_at(T, ilc_trim=ilc)
                row += [d, w, p, 60.0 / p]
                key = 'ilc' if ilc else 'disarmed'
                if key not in best or p < best[key][0]:
                    best[key] = (p, T, apex, d, w)
            print('{:>7.4f} {:>6.3f} | {:>7.4f} {:>7.4f} {:>7.4f} {:>7.1f} | '
                  '{:>7.4f} {:>7.4f} {:>7.4f} {:>7.1f}'.format(*row))
            T += 0.05
        print()
        for key in ('disarmed', 'ilc'):
            p, T, apex, d, w = best[key]
            print('FASTEST {:<8} period {:.4f} s = {:.1f} throws/min at '
                  'T {:.4f} (apex {:.3f} m), throw_delay {:.4f}, dwell {:.4f}'
                  .format(key, p, 60.0 / p, T, apex, d, w))

    if args.grid:
        print()
        print('ACCEPT-IMPLIES-FLIES over the whole (T, dwell, delay, aim) grid')
        bad = grid_violations(verbose=True)
        print('{} violation(s)'.format(len(bad)))
        if bad:
            ok = False
    return 0 if ok else 1


def required_dwell(T: float, delay_s: float, *, ilc_trim: bool = False) -> float:
    """``TossSessionSequencer.required_dwell_s`` at this (T, delay)."""
    sess = tsess.TossSessionSequencer(
        num_throws=5, dwell_time_s=99.0, throw_delay_s=float(delay_s),
        flight_time_s=float(T), ilc_speed_trim_possible=bool(ilc_trim),
    )
    return float(sess.required_dwell_s)


def session_floor_delay(T: float, *, ilc_trim: bool = False) -> float:
    """``TossSessionSequencer.min_throw_delay_s`` at this flight time."""
    sess = tsess.TossSessionSequencer(
        num_throws=5, dwell_time_s=99.0, throw_delay_s=1.0,
        flight_time_s=float(T), ilc_speed_trim_possible=bool(ilc_trim),
    )
    return float(sess.min_throw_delay_s)


if __name__ == '__main__':
    raise SystemExit(main())
