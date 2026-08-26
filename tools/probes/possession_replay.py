#!/usr/bin/env python3
"""Replay a sitting's cup-sensor stream through the PRODUCTION possession
verdict, offline — the instrument behind owner decision D1 (2026-08-26).

WHAT IT DOES
------------
For every self-toss cycle in a bag it reconstructs exactly what the live node
would have asked the ball-in-cup sensor, and asks it:

  * the cycle's SCHEDULED landing (``t_release + flight_time``, the FSM's own
    ``landing_perf``);
  * the cadence clamps ``reload_coordinator_node._set_toss_next_cycle_perf``
    latches — ``next_release = landing + dwell``, ``next_landing = next_release +
    flight``, ``prev_landing`` = the previous cycle's landing — including the
    ``intends_another_cycle`` rule that decides whether they are latched at all;
  * the real ``/hand_telemetry`` stream, fed sample-by-sample through
    ``HandBallSensorSource.note_sample`` on the perf clock;
  * the verdict read at the FSM's OWN deadline, ``landing +
    CATCH_CONFIRM_WINDOW_S``, through ``merge_possession`` with no tracker.

It is the production surface, not a re-implementation: the only thing this file
decides is *when* to ask, and it takes that from the same arithmetic the node
uses.

WHY IT IS COMMITTED (and not a /tmp one-off)
--------------------------------------------
It is the generator behind ``tests/ros/toss_verdict_replay_fixtures.py`` — the
committed evidence that survives on a machine without the bag — in the pattern
``tools/probes/toss_record_miner.py --emit-fixture`` established for
``tests/ros/toss_record_fixtures.py`` and
``tools/probes/possession_verdict_bag_check.py --emit-fixtures`` for
``tests/ros/possession_fixtures.py``.

Consuming test: ``tests/ros/test_possession_replay.py``.
Contract: ``ros_ws/docs/ball_possession_contract.md`` (C-POSSESS-1 § 3.2).
Entry: ``logbook/2026-08-26-possession-verdicts-become-sensor-only.md``.

USAGE
-----
    source ~/Desktop/PDJ_venv/venv/bin/activate

    # score a capture (prints the per-cycle table and the census)
    python tools/probes/possession_replay.py --bag ~/Desktop/rosbags/2026-08-26_14-25-16

    # regenerate the committed fixture from that capture
    python tools/probes/possession_replay.py \
        --bag ~/Desktop/rosbags/2026-08-26_14-25-16 --emit-fixture

Strictly offline and read-only on the robot: it opens ``.mcap`` files and (with
``--emit-fixture``) writes ONE file under ``tests/ros/``.
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import pprint
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'))
sys.path.insert(0, _REPO)

import jugglebot.hardware_config as hw                          # noqa: E402
from jugglebot import toss_record                               # noqa: E402
from jugglebot.ball_possession import (                         # noqa: E402
    ARRIVAL_CONFIRMED,
    HandBallSensorSource,
    arrival_blind,
    merge_possession,
)
from jugglebot.toss_sequencer import CATCH_CONFIRM_WINDOW_S     # noqa: E402

#: Samples this far either side of a cycle's landing are enough for every window
#: the source can open: the arrival window is ``[landing - lead, landing +
#: JB_BD_ARRIVAL_WINDOW_S]`` and retention runs at most one further retention
#: window past the seat edge. Generous by ~2x, because the cost of being wrong is
#: a silently blind replay and the cost of being generous is a bigger fixture.
FIXTURE_PRE_S = 3.0
FIXTURE_POST_S = 4.0


def _load_miner():
    """Import ``toss_record_miner`` as a library (it is a script, not a package
    module) — the same trick its own callers use."""
    spec = importlib.util.spec_from_file_location(
        'toss_record_miner', os.path.join(_HERE, 'toss_record_miner.py'))
    mod = importlib.util.module_from_spec(spec)
    sys.modules['toss_record_miner'] = mod
    spec.loader.exec_module(mod)
    return mod


def _sensor():
    """A source constructed EXACTLY as ``reload_coordinator_node`` constructs it,
    from the generated config rather than from literals — ``_HAND_STATE_STALE_S``
    included, which is the node's own staleness window and not a fourth config
    key."""
    from jugglebot.reload_coordinator_node import _HAND_STATE_STALE_S
    return HandBallSensorSource(
        arrival_lead_s=float(hw.JB_BD_ARRIVAL_LEAD_S),
        arrival_window_s=float(hw.JB_BD_ARRIVAL_WINDOW_S),
        retention_window_s=float(hw.JB_BD_RETENTION_WINDOW_S),
        stale_s=float(_HAND_STATE_STALE_S))


def cycles_from_rows(rows):
    """Group mined+declared rows into runs (one per goal) in declaration order,
    and attach the schedule each cycle's node would have latched.

    -> list of dicts, one per cycle that reached a release.
    """
    runs = []
    cur = None
    for r in rows:
        gid = r.get('goal_id')
        if cur is None or cur['goal_id'] != gid:
            cur = {'goal_id': gid, 'rows': []}
            runs.append(cur)
        cur['rows'].append(r)
    out = []
    for run_index, run in enumerate(runs, 1):
        prev_landing = None
        throws = 0
        for r in run['rows']:
            rel = r.get('t_release_perf')
            land = r.get('t_landing_sched_perf')
            flight = r.get('flight_time_s')
            if not rel or not land or not flight:
                continue
            dwell = r.get('goal_dwell_time_s')
            num = r.get('goal_num_throws')
            # toss_session.intends_another_cycle: throws + 1 < num_throws, where
            # `throws` counts the CAUGHT/MISSED cycles already scored. Invariant
            # under D1 — a CAUGHT and a MISSED both count as a throw — so the
            # clamps this replay uses are the ones the node really latched.
            intends = bool(num and dwell and (throws + 1) < int(num))
            next_release = (land + float(dwell)) if intends else None
            next_landing = ((next_release + float(flight))
                            if next_release is not None else None)
            out.append({
                'run': run_index,
                'cycle_index': int(r.get('cycle_index') or 0),
                'goal_id': run['goal_id'],
                'flight_time_s': float(flight),
                'dwell_time_s': float(dwell) if dwell else None,
                'throw_delay_s': (float(r['goal_throw_delay_s'])
                                  if r.get('goal_throw_delay_s') else None),
                't_release_perf': float(rel),
                'landing_perf': float(land),
                'next_release_perf': next_release,
                'next_landing_perf': next_landing,
                'prev_landing_perf': prev_landing,
                'fsm_outcome': str(r.get('outcome') or ''),
                'sensor_label': r.get('label'),
                'perf_minus_ros_s': float(r.get('perf_minus_ros_s') or 0.0),
            })
            prev_landing = float(land)
            if str(r.get('outcome') or '').startswith(('CAUGHT', 'MISSED')):
                throws += 1
    return out


def samples_for(hand, offset_s, t0, t1):
    """The ``/hand_telemetry`` samples inside ``[t0, t1]`` on the PERF clock.

    ``offset_s`` is the record's ``perf_minus_ros_s``; ``data.hand`` is ROS-stamped
    (``toss_record_miner.hand_sample_time``), and the node feeds the source its own
    monotonic clock, so the whole stream is shifted once rather than the windows
    being shifted the other way — a window shifted the wrong way is a silently
    blind replay."""
    out = []
    for s in hand:
        t = float(s.t) + float(offset_s)
        if t < t0:
            continue
        if t > t1:
            break
        out.append((round(t - t0, 6), bool(s.held), bool(s.raw), bool(s.valid)))
    return out


#: A gap larger than this many nominal steps STARTS A NEW SEGMENT rather than
#: being interpolated over. It has to be well under the source's staleness window
#: (``_HAND_STATE_STALE_S``) so that a real dropout — the thing that makes a
#: window BLIND, and therefore the thing a compressed fixture must never smooth
#: away — survives compression as a genuine hole.
GAP_STEPS = 3.0


def compress_stream(samples, step_s):
    """``[(dt, held, raw, valid), ...]`` -> ``[(t0, t1, held, raw, valid), ...]``.

    A segment is a maximal run of consecutive samples carrying the same three bits
    with no inter-sample gap over ``GAP_STEPS`` nominal steps. **21 753** raw
    samples over the reference sitting compress to **162** segments, which is the
    difference between a 1.2 MB fixture and a readable one.

    ⚠ **The claim is VERDICT-EQUIVALENCE, not losslessness** (audit fix W11,
    2026-08-26). What :func:`main` checks, at emit time and against the source
    bag, is that the expanded stream produces the same ``(label, blind,
    catch_dt_s)`` as the raw one for every cycle. That is what the fixture is
    evidence for and all it is evidence for. It is NOT a claim that the expansion
    reproduces the sample stream: the run-length encoding drops the true sample
    instants inside a run and re-lays them on a nominal ``step_s`` grid, so an
    observable that read sample TIMING rather than the three bits could differ.
    The gate is sound for THIS source because ``HandBallSensorSource`` reacts to
    exactly three things — a bit change, a validity change, and an inter-sample
    gap over ``stale_s`` — and all three are segment boundaries here; it is not
    sound in general, and a new source would need the gate re-argued rather than
    inherited.

    ``blind`` joined the compared tuple in the same fix. It was omitted, and it
    is not derivable from ``label``: ``MISSED_SENSOR_BLIND`` and a blind-but-
    ``CAUGHT`` cycle both carry it, and after the ``_blind_between`` liveness fix
    (an OPEN gap past ``stale_s`` now reads blind) a compression that moved a gap
    boundary could flip it while leaving the label alone."""
    out = []
    prev_t = None
    for dt, held, raw, valid in samples:
        key = (bool(held), bool(raw), bool(valid))
        gap = None if prev_t is None else dt - prev_t
        if (not out or key != tuple(out[-1][2:])
                or (gap is not None and gap > GAP_STEPS * step_s)):
            out.append([round(dt, 6), round(dt, 6), key[0], key[1], key[2]])
        else:
            out[-1][1] = round(dt, 6)
        prev_t = dt
    return [tuple(s) for s in out]


def expand_stream(segments, step_s):
    """The inverse of :func:`compress_stream` — the ONE expansion both the probe
    and ``tests/ros/test_possession_replay.py`` use, so a fixture cannot mean two
    different streams.

    Each segment's FIRST instant is reproduced exactly (that is where every edge
    lives, and an edge quantised onto a grid moves ``catch_event_dt_s``); the
    interior is re-emitted at the nominal step."""
    out = []
    for t0, t1, held, raw, valid in segments:
        t = float(t0)
        while t < float(t1) - 1e-9:
            out.append((t, bool(held), bool(raw), bool(valid)))
            t += float(step_s)
        out.append((float(t1), bool(held), bool(raw), bool(valid)))
    return out


def replay_cycle(cycle, samples_rel):
    """Feed one cycle's stream through the production source and read the verdict
    at the FSM's settle deadline. -> (label, blind, catch_dt_s).

    ``samples_rel`` are ``(dt, held, raw, valid)`` relative to the window start,
    which is what the fixture stores: the absolute perf clock of a 2026 sitting
    is 70-odd thousand seconds of noise that no assertion reads."""
    land = float(cycle['landing_perf'])
    t0 = land - FIXTURE_PRE_S
    src = _sensor()
    for dt, held, raw, valid in samples_rel:
        src.note_sample(t0 + dt, held=held, valid=valid, raw=raw)
    deadline = land + CATCH_CONFIRM_WINDOW_S
    verdict = merge_possession(sensor=src.observe(
        deadline, land,
        next_release_t=cycle.get('next_release_perf'),
        next_landing_t=cycle.get('next_landing_perf'),
        prev_landing_t=cycle.get('prev_landing_perf')))
    edge = src.arrival_time(land,
                            next_landing_t=cycle.get('next_landing_perf'),
                            prev_landing_t=cycle.get('prev_landing_perf'))
    label = ('CAUGHT' if verdict.confirmed
             else ('MISSED_SENSOR_BLIND' if arrival_blind(verdict) else 'MISSED'))
    return label, arrival_blind(verdict), (edge - land if edge == edge else None)


_FIXTURE_HEADER = '''"""Measured possession-replay fixtures — DO NOT hand-edit.

Every self-toss cycle of {bag} that reached a release, with the real
``/hand_telemetry`` stream around its scheduled landing and the schedule the live
node latched for it. This is the committed evidence for owner decision **D1**
(2026-08-26): possession verdicts are the ball-in-cup sensor's alone.

Regenerate with:
    python tools/probes/possession_replay.py \\
        --bag ~/Desktop/rosbags/{bag} --emit-fixture

Consumed by tests/ros/test_possession_replay.py. Contract:
ros_ws/docs/ball_possession_contract.md (C-POSSESS-1).

The bag itself is machine-local and gitignored, so this file is what carries the
evidence across a fresh clone — the tests/ros/possession_fixtures.py pattern.
"""

from __future__ import annotations

#: The reference sitting. Every number below is measured FROM it.
REFERENCE_BAG = {bag!r}

#: Each cycle's ``segments`` are RUN-LENGTH compressed samples,
#: ``(t0, t1, held, raw, valid)``, relative to ``landing_perf - {pre}`` s;
#: expand them with ``tools/probes/possession_replay.expand_stream`` at the
#: cycle's own ``step_s``. Absolute perf instants from a 2026 sitting are ~70 ks
#: of noise no assertion reads, and 21 753 raw samples are a 1.2 MB fixture.
#: The compression is VERDICT-EQUIVALENT at emit time — not lossless, and the
#: probe checks the claim it makes: it replays both the raw and the expanded
#: stream against the source bag and refuses to write a fixture whose
#: ``(label, blind, catch_dt_s)`` differ.
FIXTURE_PRE_S = {pre}
FIXTURE_POST_S = {post}

#: One entry per cycle that reached a release, in bag order.
#:  fsm_outcome   what the SHIPPED (tracker-primary) code minted, verbatim
#:  sensor_label  the offline cup label from toss_record.label_from_sensor
CYCLES = '''


def emit_fixture(path, bag_name, rows):
    body = _FIXTURE_HEADER.format(bag=bag_name, pre=FIXTURE_PRE_S,
                                  post=FIXTURE_POST_S)
    # Python literals, not JSON: the fixture is an importable module, and JSON's
    # null/true/false are NameErrors at import.
    with open(path, 'w') as fh:
        fh.write(body)
        fh.write(pprint.pformat([{k: r[k] for k in sorted(r)} for r in rows],
                                indent=1, width=86, sort_dicts=False))
        fh.write('\n')
    return path


def median_step_s(samples):
    """The measured inter-sample cadence of one cycle's window. Read from the
    stream rather than assumed to be ``JB_BD_CHECK_INTERVAL_MS``: the poll cadence
    has been an elevated 71 ms on a degraded plant before now
    (``logbook/2026-08-24-hand-sensor-poll-cadence.md``), and a fixture expanded
    at a cadence the bag did not have is a fixture about a different machine."""
    d = sorted(b[0] - a[0] for a, b in zip(samples, samples[1:]))
    return d[len(d) // 2] if d else 0.02


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag', required=True)
    ap.add_argument('--emit-fixture', action='store_true')
    args = ap.parse_args(argv)

    miner = _load_miner()
    bag = os.path.expanduser(args.bag)
    bag_name = os.path.basename(bag.rstrip('/'))
    data = miner.read_bag(bag, sensor_only=True)
    mined = miner.mine_bag(data, robot='jugglebot', plane_mm=None)
    rows = toss_record.join(data.declarations, mined)
    cycles = cycles_from_rows(rows)

    rows = []
    census = {}
    fsm_census = {}
    print('%3s %3s %-26s %-10s %-20s %9s' % (
        'run', 'cyc', 'fsm_outcome', 'cup_label', 'replayed_verdict',
        'catch_dt'))
    for c in cycles:
        land = c['landing_perf']
        raw = samples_for(data.hand, c['perf_minus_ros_s'],
                          land - FIXTURE_PRE_S, land + FIXTURE_POST_S)
        step = median_step_s(raw)
        segments = compress_stream(raw, step)
        label, blind, dt = replay_cycle(c, raw)
        # THE VERDICT-EQUIVALENCE GATE. A compressed fixture that answers
        # differently from the stream it was cut from is evidence about nothing,
        # and the failure would be silent — so it is checked here, at emit time,
        # where the raw stream still exists. `blind` is in the tuple because it
        # is a SEPARATE answer from the label (a blind CAUGHT carries it too) and
        # because the liveness term in `_blind_between` keys on an inter-sample
        # gap, which is exactly the quantity a run-length encoding moves.
        r_label, r_blind, r_dt = replay_cycle(c, expand_stream(segments, step))
        if (r_label, r_blind, r_dt) != (label, blind, dt):
            raise SystemExit(
                'compression is NOT verdict-equivalent for run {} cycle {}: raw '
                '{} blind={} {} vs expanded {} blind={} {}'.format(
                    c['run'], c['cycle_index'], label, blind, dt,
                    r_label, r_blind, r_dt))
        census[label] = census.get(label, 0) + 1
        fsm_census[c['fsm_outcome']] = fsm_census.get(c['fsm_outcome'], 0) + 1
        print('%3d %3d %-26s %-10s %-20s %9s' % (
            c['run'], c['cycle_index'], c['fsm_outcome'],
            str(c['sensor_label']), label,
            '-' if dt is None else '%+.3f' % dt))
        row = dict(c)
        row.pop('perf_minus_ros_s', None)
        row['step_s'] = round(step, 6)
        row['segments'] = segments
        row['replayed_verdict'] = label
        row['catch_event_dt_s'] = None if dt is None else round(dt, 6)
        rows.append(row)
    print('\nreplayed census :', dict(sorted(census.items())))
    print('shipped  census :', dict(sorted(fsm_census.items())))

    if args.emit_fixture:
        out = os.path.join(_REPO, 'tests', 'ros',
                           'toss_verdict_replay_fixtures.py')
        emit_fixture(out, bag_name, rows)
        print('wrote', out)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
