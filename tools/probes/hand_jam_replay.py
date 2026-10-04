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
sample of the sittings, not just synthetic traces. PASS = the latched event
fires within the 300 ms before T, and every other fire is one of the KNOWN
clamp stalls listed in ``KNOWN_STALLS`` (each is a real ball pinch the guard
never latched, found by this probe's first run on 2026-10-04: the hand sat at
the 50 A clamp with the command 2.0 rev below it at 1790929174.17 for 0.16 s,
and at 1790929358.65 / 360.83 for about 5.5 s in all, until the hand ODrive
went IDLE on DC_BUS_UNDER_VOLTAGE -- the motor thermistor went 27 to 49 C).
"Exactly one fire" was the first pass condition; it was wrong, because the
2026-10-02 sitting had THREE pinches and only one of them latched. A fire
outside the known list is still a FAIL with every fire listed, so a detector
change that starts firing on ordinary catches, rests, top stops or homing is
caught here.

HOW IT REPLAYS
--------------
One pass per bag over both topics in log-time order, through ``mcap_ros2``
(decodes from the schema the bag carries; no ``jugglebot_interfaces`` build
needed). Each ``/hand_telemetry`` row becomes a ``HandSample`` on its header
stamp; P6's axis state / active_errors / age come from the latest
``/robot_state`` motor-6 sample (age = the hand row's log time minus that
sample's log time). ``curr_limit_a`` is the shipped 50 A (the bag does not
carry the live limit). After a fire the detector is reset once the stall
conjunction has been false for 1 s, so later, independent fires are counted
rather than hidden behind the live node's latch.

Bags run in parallel (one process each); a 500 MB bag takes several minutes,
so run it in the background:

    python tools/probes/hand_jam_replay.py > temp/probes/hand_jam_replay_20261002.log 2>&1 &
    python tools/probes/hand_jam_replay.py --bags ~/Desktop/rosbags/2026-10-02_18-11-14
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
from concurrent.futures import ProcessPoolExecutor

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'))

from jugglebot.motion.hand_jam import (  # noqa: E402
    HandSample, JamConfig, JamDetector, Verdict)

EVENT_T = 1790929469.795
#: Clamp stalls in bag 2026-10-02_18-11-14 besides the latched event (onsets, s):
#: the two un-latched pinches the first replay found (see the module docstring).
KNOWN_STALLS = (1790929174.165, 1790929358.652, 1790929360.831)

_RESET_AFTER_CLEAR_S = 1.0


def _replay(bag_dir: str) -> dict:
    from mcap_ros2.reader import read_ros2_messages
    mcaps = sorted(glob.glob(os.path.join(bag_dir, '*.mcap')))
    det = JamDetector(JamConfig())
    rs = None            # (log_t, state, errors)
    fires, n_ht, n_rs, t_lo, t_hi = [], 0, 0, None, None
    clear_since = None
    ep = None            # the clamp episode after the latest fire: [fire_idx, t_last_clamp, min_lag]
    for mcap in mcaps:
        for m in read_ros2_messages(mcap, topics=['/hand_telemetry', '/robot_state']):
            tl = m.log_time_ns * 1e-9
            r = m.ros_msg
            if m.channel.topic == '/robot_state':
                ms = r.motor_states
                if len(ms) > 6:
                    rs = (tl, int(ms[6].current_state), int(ms[6].active_errors))
                    n_rs += 1
                continue
            n_ht += 1
            t = r.timestamp.sec + 1e-9 * r.timestamp.nanosec
            t_lo = t if t_lo is None else t_lo
            t_hi = t
            if rs is None:
                state, err, age = 0, 0, 1e9      # no motor-6 view yet: P6 false
            else:
                state, err, age = rs[1], rs[2], max(0.0, tl - rs[0])
            s = HandSample(t, r.pos_cmd, r.vel_ff_cmd, r.pos_meas, r.vel_meas, r.iq_meas,
                           state, err, age, 50.0)
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
            'fires': fires}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bags', nargs='*',
                    default=sorted(glob.glob(os.path.expanduser(
                        '~/Desktop/rosbags/2026-10-02_*'))))
    ap.add_argument('--event-t', type=float, default=EVENT_T)
    ap.add_argument('--known-stalls', type=float, nargs='*', default=KNOWN_STALLS,
                    help='stall onsets (s) that may fire besides the latched event')
    ap.add_argument('--known-tol-s', type=float, default=0.5)
    a = ap.parse_args(argv)
    with ProcessPoolExecutor(max_workers=max(1, min(4, len(a.bags)))) as ex:
        results = list(ex.map(_replay, a.bags))
    all_fires = []
    for res in results:
        dur = (res['t1'] - res['t0']) if res['t0'] is not None else 0.0
        print(f"{os.path.basename(res['bag'])}: {res['hand_rows']} /hand_telemetry rows, "
              f"{res['rs_rows']} /robot_state rows, {dur:.0f} s, {len(res['fires'])} fire(s)")
        for (t, v, pm, pc, iq, preds, t_last, min_lag) in res['fires']:
            print(f"   FIRE {v} at {t:.3f} (T{t - a.event_t:+.3f} s) pos_meas {pm:+.3f} "
                  f"pos_cmd {pc:+.3f} iq {iq:+.1f} A [{preds}]; stayed at the clamp "
                  f"{t_last - t + JamConfig().sustain_s:.2f} s from the stall, deepest lag "
                  f"{min_lag:+.3f} rev (guard latches at -2.5)")
            all_fires.append((t, v))
    event = [t for (t, v) in all_fires
             if v == Verdict.HAND_JAM.value and -0.300 <= t - a.event_t <= 0.0]
    unknown = [t for (t, v) in all_fires
               if t not in event
               and not any(abs(t - k) <= a.known_tol_s for k in a.known_stalls)]
    ok = len(event) == 1 and not unknown
    lead = (a.event_t - event[0]) * 1e3 if len(event) == 1 else float('nan')
    print(f"RESULT: {'PASS' if ok else 'FAIL'} — {len(all_fires)} fire(s) across "
          f"{len(results)} bag(s); the latched event T={a.event_t:.3f} "
          + (f'fired {lead:.0f} ms before the latch' if len(event) == 1 else 'did NOT fire')
          + f"; {len(all_fires) - len(event)} other fire(s), {len(unknown)} outside the "
          f"known stall list {a.known_stalls}"
          + (f': UNKNOWN at {[round(t, 3) for t in unknown]}' if unknown else ''))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
