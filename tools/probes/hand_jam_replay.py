#!/usr/bin/env python3
"""Replay recorded ``/hand_telemetry`` (+ ``/robot_state`` motor 6) through the
hand-jam detector, ``jugglebot.motion.hand_jam.JamDetector``.

WHY THIS EXISTS
----------------
The detector's discriminators (P1-P6 + the 100 ms sustain) were derived from
one event: the 2026-10-02 ball pinch in bag ``2026-10-02_18-11-14``, which
the guard latched at T = 1790929469.795 (``logbook/2026-10-04-skill-stack-r5-
sitting-3.md``). A detector that also fires on ordinary catches, rests, top
stops or homing would stop a healthy pattern and run a recovery the hand did
not need, so the false-positive side is checked against every recorded hand
sample of the sittings, not just synthetic traces. A fire is PASS only when
it lands inside one of ``EXPECTED``'s per-bag windows (below); any other fire
is a FAIL, so a detector change that starts firing on ordinary catches,
rests, top stops or homing is caught here.

THE AGE RECONSTRUCTION IS PESSIMISTIC (2026-10-04, R5 sitting 4)
-----------------------------------------------------------------
P6 gates on how old axis 6's DIAGNOSTIC is (``JamConfig.max_age_s``). The
live bridge node computes that honestly (RX-thread wall time since the last
DIAGNOSTIC UDP frame updated the cache, `` teensy_bridge_node._hand_jam_sample``)
and over 2026-10-04_19-52-08 / _20-10-26 that age ran to ~1 s in a steady
clamp stall (iq constant -> no on-change send -> only the firmware's 1 Hz
forced refresh, ``Teensy_code_canbridge/telemetry.cpp`` ``DIAG_FORCE_PERIOD_US``),
which a ``max_age_s`` of 0.05 could never survive. This replay (and the
fixture it feeds) used to compute age from the 100 Hz ``/robot_state``
REPUBLISH instead (age = hand row time minus the latest ``/robot_state``
row, <= ~10 ms) and so fired where the live node could not.

The fix here: reconstruct the age from the moments the motor-6 DIAGNOSTIC
TUPLE actually changes in ``/robot_state`` -- ``current_state``,
``active_errors``, ``disarm_reason``, ``iq_measured``, ``iq_setpoint``,
``fet_temp``, ``motor_temp``, ``bus_voltage``, ``bus_current``,
``procedure_result`` (NOT ``pos_estimate``/``vel_estimate``: those update at
the 100 Hz telemetry rate regardless of the diagnostic cadence and would
make every sample look fresh). ``/robot_state`` republishes the bridge's
cached diagnostic values at 100 Hz, so between two real DIAGNOSTIC arrivals
those ten fields are bit-identical across many consecutive rows; a change in
them is therefore good evidence a new DIAGNOSTIC just landed. The one blind
spot: the firmware's 1 Hz FORCED refresh can resend the SAME content (nothing
changed enough to warrant an on-change send), and that resend is invisible to
a tuple-equality check -- so this reconstruction's age keeps growing past the
point where the real diagnostic age was reset to ~0, i.e. it OVER-ESTIMATES
the true age. That is the pessimistic direction: wherever this replay's P6
passes, the live node's P6 (whose true age is <= this estimate) would have
passed too, so a PASS here is also a live-node PASS, and this probe can only
under-report fires relative to the live node, never over-report them.

HOW IT REPLAYS
--------------
One pass per bag over both topics in log-time order, through ``mcap_ros2``
(decodes from the schema the bag carries; no ``jugglebot_interfaces`` build
needed). Each ``/hand_telemetry`` row becomes a ``HandSample`` on its header
stamp; P6's axis state / active_errors come from the latest ``/robot_state``
motor-6 sample, and its age from the pessimistic reconstruction above.
``curr_limit_a`` is the shipped 50 A (the bag does not carry the live limit).
After a fire the detector is reset once the stall conjunction has been false
for 1 s, so later, independent fires are counted rather than hidden behind
the live node's latch.

Bags run in parallel (one process each); a 500 MB bag takes several minutes,
so run it in the background:

    python tools/probes/hand_jam_replay.py > temp/probes/hand_jam_replay_20261002.log 2>&1 &
    python tools/probes/hand_jam_replay.py --bags ~/Desktop/rosbags/2026-10-02_18-11-14

To regenerate the fixture CSV from a single bag and an absolute time window
(``--fixture-window T0 T1``; the window used for the committed fixture is
read back from its first/last ``t_minus_T_s`` row plus the reference T
below):

    python tools/probes/hand_jam_replay.py \\
        --bags ~/Desktop/rosbags/2026-10-02_18-11-14 \\
        --fixture-window 1790929468.9458 1790929469.9443 \\
        --fixture-out tests/motion/fixtures/hand_jam_20261002.csv
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
import sys
from concurrent.futures import ProcessPoolExecutor

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'))

from jugglebot.motion.hand_jam import (  # noqa: E402
    HandSample, JamConfig, JamDetector, Verdict)

#: The 2026-10-02 bag's guard-latch report -- the fixture's time reference,
#: unchanged from the original fixture (``t_minus_T_s`` in the CSV).
GUARD_LATCH_T = 1790929469.795

#: Per-bag expected fire windows, as ``(onset_s, before_s, after_s, required)``:
#: a fire at time ``t`` is EXPECTED iff ``onset - before_s <= t <= onset + after_s``
#: for some entry. ``required`` entries must be matched by at least one fire
#: for the bag to PASS; non-required entries are merely ALLOWED (a fire there
#: does not count as unexpected, but its absence is not a failure either --
#: this is the historical ``KNOWN_STALLS`` treatment).
#:
#: 2026-10-02_18-11-14: the guard-latch report (REQUIRED, fire within 0.3 s
#: BEFORE it, never after -- unchanged from the original EVENT_T check) plus
#: the two un-latched pinches this probe's first run found on 2026-10-04
#: (ALLOWED, +/-0.5 s -- the original KNOWN_STALLS tolerance, kept exactly).
#: A THIRD sub-fire at 1790929362.911 was found once P1's measured-descent
#: alternative existed (2026-10-04, R5 sitting 4 implementation): the dense
#: trace confirms the hand is UNBROKEN at the clamp from 358.431 to 363.922
#: (pos_cmd flat at the 0.307 rev rest the whole time, iq pegged ~-50 A) --
#: this is the SAME un-latched pinch already known to re-trigger more than
#: once under the 1 s reset-after-clear counting below (that is what the
#: "358.652/360.831" pair already was), now with one more sub-fire from a
#: real, if brief, -1.76 rev/s creep sample (pos_meas actually moved, unlike
#: the pure-noise case ruled out above) during the SAME 5.5 s episode.
#: 2026-10-04_19-52-08 / _20-10-26: the R5 sitting-4 jam-detector defect bags
#: (facts.md / brief_j1_jam_fix.md D1/D2); these onsets are the STALL ONSET
#: itself (not a later report), so the fire is REQUIRED within 0.3 s AFTER.
#: 2026-10-04_19-26-00: the runsheet § 2 BENCH pinch (adjudicated by the
#: orchestrator from the bag, 2026-10-04): the owner's ``/hand_move_to 0.0``
#: descended at ~1 rev/s onto a ball placed on the funnel ring, iq ramped
#: -10 -> -49 A from 1791102419.6 as the ball resisted, the hand stalled at
#: +2.9 rev at 1791102420.28 and sat at the clamp for 6.1 s until the owner
#: E-STOPped (the move's deferred reply then came back ERR_BUS_DOWN, which is
#: why the launch log shows the 0.0 request as a FAILURE at 426.5). The live
#: detector was silent for the same reason as the two defect bags (P6's 50 ms
#: age bound against the 1 Hz diagnostic) -- the stale ``pos_cmd`` 0.0 that
#: the owner noticed made P3 true by coincidence but was never the blocker.
#: REQUIRED within 0.3 s AFTER the stall onset, like the two defect bags.
EXPECTED = {
    '2026-10-02_18-11-14': (
        (GUARD_LATCH_T, 0.300, 0.0, True),
        (1790929174.165, 0.5, 0.5, False),
        (1790929358.652, 0.5, 0.5, False),
        (1790929362.911, 0.5, 0.5, False),
        (1790929360.831, 0.5, 0.5, False),
    ),
    '2026-10-04_19-26-00': (
        (1791102420.28, 0.0, 0.3, True),
    ),
    '2026-10-04_19-52-08': (
        (1791104785.2, 0.0, 0.3, True),
    ),
    '2026-10-04_20-10-26': (
        (1791105152.8, 0.0, 0.3, True),
    ),
}

#: All six bags facts.md names for this sitting (bench check, level
#: measurement and the two defect-finding fed-columns bags, plus the
#: 2026-10-02 pinch bag and its earlier sibling). Named explicitly, not
#: globbed: ``~/Desktop/rosbags/`` also holds other 2026-10-02/-04 bags
#: (e.g. ``2026-10-02_12-43-28``, ``2026-10-02_18-10-23``) that are not part
#: of this set.
DEFAULT_BAGS = tuple(os.path.expanduser(f'~/Desktop/rosbags/{b}') for b in (
    '2026-10-02_18-11-14', '2026-10-02_18-47-03',
    '2026-10-04_19-26-00', '2026-10-04_19-39-06',
    '2026-10-04_19-52-08', '2026-10-04_20-10-26',
))

_RESET_AFTER_CLEAR_S = 1.0

#: The motor-6 fields a DIAGNOSTIC carries (NOT pos/vel estimates, which
#: update at the 100 Hz telemetry rate regardless of the diagnostic cadence).
_DIAG_FIELDS = ('current_state', 'active_errors', 'disarm_reason', 'iq_measured',
                'iq_setpoint', 'fet_temp', 'motor_temp', 'bus_voltage',
                'bus_current', 'procedure_result')


def _diag_tuple(motor6):
    return tuple(getattr(motor6, f) for f in _DIAG_FIELDS)


def _bag_key(bag_dir: str) -> str:
    return os.path.basename(os.path.normpath(bag_dir))


def _replay(bag_dir: str, fixture_window=None) -> dict:
    from mcap_ros2.reader import read_ros2_messages
    mcaps = sorted(glob.glob(os.path.join(bag_dir, '*.mcap')))
    det = JamDetector(JamConfig())
    rs = None               # (log_t, state, errors) -- latest /robot_state motor-6 view
    last_diag_t = None      # log time of the /robot_state row where the diag tuple last changed
    prev_diag = None
    fires, n_ht, n_rs, t_lo, t_hi = [], 0, 0, None, None
    clear_since = None
    fixture_rows = []
    ep = None            # the clamp episode after the latest fire: [fire_idx, t_last_clamp, min_lag]
    for mcap in mcaps:
        for m in read_ros2_messages(mcap, topics=['/hand_telemetry', '/robot_state']):
            tl = m.log_time_ns * 1e-9
            r = m.ros_msg
            if m.channel.topic == '/robot_state':
                ms = r.motor_states
                if len(ms) > 6:
                    m6 = ms[6]
                    rs = (tl, int(m6.current_state), int(m6.active_errors))
                    n_rs += 1
                    diag = _diag_tuple(m6)
                    if diag != prev_diag:
                        last_diag_t, prev_diag = tl, diag
                continue
            n_ht += 1
            t = r.timestamp.sec + 1e-9 * r.timestamp.nanosec
            t_lo = t if t_lo is None else t_lo
            t_hi = t
            if rs is None:
                state, err, age = 0, 0, 1e9      # no motor-6 view yet: P6 false
            else:
                state, err = rs[1], rs[2]
                age = 1e9 if last_diag_t is None else max(0.0, tl - last_diag_t)
            s = HandSample(t, r.pos_cmd, r.vel_ff_cmd, r.pos_meas, r.vel_meas, r.iq_meas,
                           state, err, age, 50.0)
            if fixture_window is not None and fixture_window[0] <= t <= fixture_window[1]:
                fixture_rows.append((t, r.pos_cmd, r.vel_ff_cmd, r.pos_meas, r.vel_meas,
                                     r.iq_meas, state, err, age))
            v = det.step(s)
            if v is not Verdict.NONE:
                fires.append([t, v.value, r.pos_meas, r.pos_cmd, r.iq_meas, det.last.text(),
                              t, r.pos_cmd - r.pos_meas])
                ep = len(fires) - 1
            if ep is not None:
                # Evidence for the log: how long the hand STAYED at the clamp
                # under a command below it after the fire (what a missed jam
                # costs), and the deepest lag (vs the 2.5 rev guard).
                lag = r.pos_cmd - r.pos_meas
                # P6 gates it: an ODrive gone IDLE/faulted leaves FROZEN iq/pos
                # in the cache (2026-10-02 364-402 s: state 1, DC_BUS_UNDER_VOLTAGE,
                # iq "-50.3 A" constant for 36 s), which is not a stall.
                if (det.last.p6 and abs(r.iq_meas) >= 0.9 * 50.0
                        and lag < -JamConfig().lag_rev):
                    fires[ep][6] = t
                    fires[ep][7] = min(fires[ep][7], lag)
                elif t - fires[ep][6] > 0.1:
                    ep = None
            if det.fired is not Verdict.NONE:
                if det.last.stall:
                    clear_since = None
                elif clear_since is None:
                    clear_since = t
                elif t - clear_since >= _RESET_AFTER_CLEAR_S:
                    det.reset()
                    clear_since = None
    return {'bag': bag_dir, 'hand_rows': n_ht, 'rs_rows': n_rs, 't0': t_lo, 't1': t_hi,
            'fires': fires, 'fixture_rows': fixture_rows}


def _match(fires_t, expected):
    """``(unexpected, missing_required)`` for one bag's fire times against
    its ``EXPECTED`` window list."""
    unexpected = []
    matched = [False] * len(expected)
    for t in fires_t:
        hit = None
        for i, (onset, before_s, after_s, _req) in enumerate(expected):
            if onset - before_s <= t <= onset + after_s:
                hit = i
                break
        if hit is None:
            unexpected.append(t)
        else:
            matched[hit] = True
    missing_required = [expected[i][0] for i, (_, _, _, req) in enumerate(expected)
                        if req and not matched[i]]
    return unexpected, missing_required


def _write_fixture(path: str, rows, window, flag_name='--fixture-out'):
    rows = sorted(rows, key=lambda r: r[0])
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        f.write(f'# 2026-10-02 hand jam (bag 2026-10-02_18-11-14): /hand_telemetry rows, '
                f'header stamp minus T={GUARD_LATCH_T} (the guard-latch report), joined to '
                f'the latest /robot_state motor-6 sample (axis_state, active_errors). The age '
                f"column is the PESSIMISTIC reconstruction (hand_jam_replay.py's "
                f'{flag_name}/--fixture-window: age = hand row time minus the last '
                f'/robot_state row at which the motor-6 diagnostic tuple changed -- see the '
                f'module docstring). Decoded from the bag, not hand-edited.\n')
        w.writerow(['t_minus_T_s', 'pos_cmd', 'vel_ff_cmd', 'pos_meas', 'vel_meas', 'iq_meas',
                   'axis_state', 'active_errors', 'diag_age_s'])
        for (t, pc, vf, pm, vm, iq, st, er, age) in rows:
            w.writerow([f'{t - GUARD_LATCH_T:.4f}', f'{pc:.5f}', f'{vf:.4f}', f'{pm:.5f}',
                       f'{vm:.4f}', f'{iq:.3f}', st, er, f'{age:.4f}'])
    print(f'wrote {len(rows)} rows to {path} (window {window[0]:.4f}..{window[1]:.4f})')


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bags', nargs='*', default=list(DEFAULT_BAGS))
    ap.add_argument('--fixture-window', type=float, nargs=2, default=None,
                    metavar=('T0', 'T1'),
                    help='absolute epoch-seconds window to collect fixture rows from')
    ap.add_argument('--fixture-out', default=None,
                    help='write the fixture rows (requires --fixture-window) to this CSV')
    a = ap.parse_args(argv)
    if a.fixture_out and not a.fixture_window:
        ap.error('--fixture-out requires --fixture-window T0 T1')
    with ProcessPoolExecutor(max_workers=max(1, min(4, len(a.bags)))) as ex:
        results = list(ex.map(_replay, a.bags, [a.fixture_window] * len(a.bags)))
    overall_ok = True
    all_fixture_rows = []
    for res in results:
        bag_key = _bag_key(res['bag'])
        expected = EXPECTED.get(bag_key, ())
        dur = (res['t1'] - res['t0']) if res['t0'] is not None else 0.0
        fires_t = [f[0] for f in res['fires']]
        unexpected, missing = _match(fires_t, expected)
        bag_ok = not unexpected and not missing
        overall_ok = overall_ok and bag_ok
        print(f"{bag_key}: {res['hand_rows']} /hand_telemetry rows, "
              f"{res['rs_rows']} /robot_state rows, {dur:.0f} s, {len(res['fires'])} fire(s) "
              f"-- {'PASS' if bag_ok else 'FAIL'}")
        for (t, v, pm, pc, iq, preds, t_last, min_lag) in res['fires']:
            print(f"   FIRE {v} at {t:.3f} pos_meas {pm:+.3f} pos_cmd {pc:+.3f} iq {iq:+.1f} A "
                  f"[{preds}]; stayed at the clamp "
                  f"{t_last - t + JamConfig().sustain_s:.2f} s from the stall, deepest lag "
                  f"{min_lag:+.3f} rev (guard latches at -2.5)")
        if unexpected:
            print(f"   UNEXPECTED fire(s) outside every EXPECTED window: "
                  f"{[round(t, 3) for t in unexpected]}")
        if missing:
            print(f"   MISSING required fire(s) near onset(s): {[round(t, 3) for t in missing]}")
        all_fixture_rows.extend(res['fixture_rows'])
    print(f"RESULT: {'PASS' if overall_ok else 'FAIL'} across {len(results)} bag(s)")
    if a.fixture_out:
        _write_fixture(a.fixture_out, all_fixture_rows, a.fixture_window)
    return 0 if overall_ok else 1


if __name__ == '__main__':
    sys.exit(main())
