#!/usr/bin/env python3
"""Measure the SEATED delay behind ``executor.CAUGHT_WINDOW_S``, from a rosbag.

The question: how long after the ball's landing does the hand's possession
sensor first read SEATED?  ``skills.executor`` finalises a throw's outcome a
window after the landing and scores ``caught`` from any SEATED reading inside
it, so that window has to cover this delay -- and the 2026-09-16 R3 sitting
showed the provisional 0.15 s did not (four of five real catches scored as
misses; ``logbook/2026-09-16-outcome-window-and-computed-catch-deferral.md``).

Two series, paired per throw:

* the LANDING -- ``/balls`` (``BallStateArray``), grouped by ``id``, the
  DESCENDING crossing of the 830 mm catch plane (``skills.sites.CATCH_CUP_Z_MM``)
  linearly interpolated between the samples either side of it.  Candidate
  tracks are filtered to those that actually rise above 880 mm, so a track that
  never left the cup cannot contribute a spurious near-plane crossing.
* the SEAT -- ``/hand_telemetry``, the first sample with
  ``ball_held_valid and ball_held`` at or after ``landing - search_lead``.
  ``ball_held`` is already the DEBOUNCED verdict (see the ``.msg``), so this is
  the delay the executor actually sees, debounce included.

Both series carry their own header stamps and those are what is used -- never
the bag receive time (measured offset under 1 ms either way, but the receive
time is the wrong clock in principle).  The stamps are the same Unix epoch the
executor's schedule runs on, so the delays are directly comparable to
``CAUGHT_WINDOW_S``.

Usage (venv; ROS2 Foxy sourced for the message types)::

    python tools/probes/caught_window_bag_probe.py \
        ~/Desktop/rosbags/2026-09-16_14-16-38 \
        ~/Desktop/rosbags/2026-09-15_18-51-37

**Known limitation (2026-09-16, found by comparing against an independent pass
over the same bags).** Only the FIRST descending crossing of each ``/balls``
track is reported (the ``break`` in :func:`_landings`). When the tracker keeps
one ``id`` across two flights, that discards the second, real landing and can
report a spurious early crossing instead: on ``2026-09-16_14-16-38`` this probe
reports track 3 crossing at 311.3665 with a -47.5 ms "delay" (paired against a
cup still holding the PREVIOUS ball), where the independent pass reported that
throw landing at 311.9977 and seating +281.9 ms later. The +191.2 ms maximum and
the whole 09-15 series agree between the two to the millisecond, so the WINDOW
this probe exists to set is unaffected -- but do not read a single row as
authoritative without checking the crossing against the schedule's own
``CATCH-AIM ... t_land``. The fix is to report every descending crossing per
track (and to pair against a SEATED rising EDGE, not any SEATED sample); it is
deliberately NOT applied here, because the numbers recorded in the logbook are
this version's output.

Prints one row per landing and the per-bag max/median.  Report the two bags
SEPARATELY: the 09-16 delays were systematically positive (tens to hundreds of
ms) and the 09-15 ones systematically negative (-39..-48 ms), and a statistic
pooled across that sign flip is not a physical quantity.  Outputs go to stdout;
nothing is written under the repo.
"""
from __future__ import annotations

import argparse
import os
import statistics
import sys

CATCH_PLANE_MM = 830.0
#: A candidate track must rise this far above the plane to count as a flight.
FLIGHT_ZMAX_MM = 880.0
#: How far before the landing a SEATED sample may sit and still be this catch's
#: (the cup rim reaches the plane before the ball's estimated centroid does).
SEARCH_LEAD_S = 0.05


def _sec(v) -> float:
    """Seconds from a ``builtin_interfaces/Time`` or a bare float.

    ``HandTelemetryMessage.timestamp`` is a ``Time``; other layers carry plain
    seconds. Accept both rather than guessing per topic.
    """
    if hasattr(v, 'sec'):
        return float(v.sec) + float(v.nanosec) * 1e-9
    return float(v)


def _stamp(header) -> float:
    return _sec(header.stamp)


def _read(bag_dir, topics):
    """``{topic: [(t_recv_s, msg), ...]}`` from an MCAP rosbag2 directory.

    Read with ``mcap_ros2`` (which decodes each message from the SCHEMA the bag
    itself carries) rather than ``rosbag2_py`` + ``rosidl_runtime_py``: this box
    records MCAP, ``rosbag2_py`` is not installed in the project venv on Foxy,
    and going through the embedded schema means the probe does not need
    ``jugglebot_interfaces`` built or sourced to read a bag.
    """
    from mcap_ros2.reader import read_ros2_messages

    paths = [os.path.join(bag_dir, f) for f in sorted(os.listdir(bag_dir))
             if f.endswith('.mcap')]
    if not paths:
        raise SystemExit('no .mcap in %s' % bag_dir)
    out = {t: [] for t in topics}
    want = {t.lstrip('/'): t for t in topics}
    for path in paths:
        for m in read_ros2_messages(path):
            key = want.get(m.channel.topic.lstrip('/'))
            if key is not None:
                out[key].append((m.log_time_ns * 1e-9, m.ros_msg))
    for rows in out.values():
        rows.sort(key=lambda r: r[0])
    return out


def _landings(balls):
    """``[(ball_id, t_cross_s)]`` — descending 830 mm crossings, one per track."""
    tracks = {}
    for t_recv, msg in balls:
        t = _stamp(msg.header) if hasattr(msg, 'header') else t_recv
        for b in msg.balls:
            tracks.setdefault(int(b.id), []).append((t, float(b.position.z)))
    out = []
    for bid, samples in sorted(tracks.items()):
        samples.sort()
        if not samples or max(z for _t, z in samples) <= FLIGHT_ZMAX_MM:
            continue                      # never left the cup: not a flight
        for (t0, z0), (t1, z1) in zip(samples, samples[1:]):
            if z0 >= CATCH_PLANE_MM > z1:
                frac = (z0 - CATCH_PLANE_MM) / (z0 - z1)
                out.append((bid, t0 + frac * (t1 - t0)))
                break                     # first descending crossing only
    return sorted(out, key=lambda r: r[1])


def _seats(hand):
    """``[t_s]`` — every sample whose DEBOUNCED possession verdict is SEATED."""
    out = []
    for t_recv, msg in hand:
        t = _sec(getattr(msg, 'timestamp', 0.0)) or t_recv
        if bool(getattr(msg, 'ball_held_valid', False)) and bool(msg.ball_held):
            out.append(t)
    return sorted(out)


def run(bag_dir) -> list:
    data = _read(bag_dir, ['/balls', '/hand_telemetry'])
    seats = _seats(data['/hand_telemetry'])
    rows = []
    for bid, t_land in _landings(data['/balls']):
        seat = next((t for t in seats if t >= t_land - SEARCH_LEAD_S), None)
        rows.append((bid, t_land, seat,
                     None if seat is None else seat - t_land))
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bags', nargs='+')
    args = ap.parse_args(argv)
    for bag in args.bags:
        rows = run(os.path.expanduser(bag))
        print('\n== %s ==' % bag)
        print('%6s %18s %18s %10s' % ('ball', 't_land_obs', 't_first_seated',
                                      'delay_ms'))
        for bid, t_land, seat, d in rows:
            print('%6d %18.4f %18s %10s'
                  % (bid, t_land, '--' if seat is None else '%.4f' % seat,
                     '--' if d is None else '%+.1f' % (d * 1e3)))
        got = [d for *_r, d in rows if d is not None]
        if got:
            print('n=%d  max=%+.1f ms  median=%+.1f ms'
                  % (len(got), max(got) * 1e3, statistics.median(got) * 1e3))
        missed = [bid for bid, _t, seat, _d in rows if seat is None]
        if missed:
            print('no SEATED ever (not caught): %s' % (missed,))
    return 0


if __name__ == '__main__':
    sys.exit(main())
