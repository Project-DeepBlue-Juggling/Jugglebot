#!/usr/bin/env python3
"""Mine a rosbag into the per-toss record corpus — the MEASUREMENT half.

WHAT IT DOES
------------
Reads one sitting's ``.mcap`` and emits one ``toss_record/1`` row per self-toss:
the sensor block and its label, the mocap arrival offset, the provenance the bag
witnesses, and — when the coordinator was publishing them — the ``/toss/record``
DECLARATIONS joined on ``announce_throw_time_ros``. (The PLANT block is deferred;
see ``mine_bag``.)

    python tools/probes/toss_record_miner.py --bag 2026-08-10_16-30-44
    python tools/probes/toss_record_miner.py --bag A --bag B --jsonl
    python tools/probes/toss_record_miner.py --self-check
    python tools/probes/toss_record_miner.py --bag 2026-08-10_16-30-44 \
        --emit-fixture

THE INVERSION THAT MAKES THIS WORK (plan § 3.3)
-----------------------------------------------
**The miner must be able to produce the whole record ALONE, in degraded form;
the declaration only upgrades specific fields from mined to declared.** That is
not a nicety — it is what lets the three 2026-08-10 bags become a corpus *today*,
years of sittings before ``/toss/record`` existed, and it makes replayability
structural rather than a feature somebody has to remember to preserve.

The reference bag proves it: ``2026-08-10_16-30-44`` predates every line of this
phase, carries no ``/toss/record``, no ``/rosout``, no ``/catch/*`` and no action
feedback, and this miner still produces a full corpus from it — every row
``record_provenance = mined-only``.

WHAT IS IMPORTED RATHER THAN RE-IMPLEMENTED (plan D11)
-------------------------------------------------------
* the LABEL and the sensor edges — ``jugglebot.toss_record.label_from_sensor``,
  the installed production module the node imports too;
* the announced-ball latch — ``jugglebot.toss_record.latch_announced_ball``,
  extracted verbatim from ``reload_coordinator_node``;
* the mocap arrival estimator — ``ball_arrival_offset.fit_plane_crossing_full``.

The PLANT block (``hand_stroke_timeline``) is **not wired in 2a** — see
``mine_bag`` for why, and the plan's § 10 for the field mapping it will use.

If any of those were re-implemented here, the live and offline definitions would
drift and nobody would notice until a calibration map had been fitted on the
wrong labels. The precedent is explicit: ``possession_verdict_bag_check.py``
builds the arrival source *exactly as the node builds it*, deliberately.

CLOCKS
------
Sensor work runs on the ``/hand_telemetry`` **header stamp** (ROS wall), which is
the same clock ``ThrowAnnouncement.throw_time`` is in — so the sensor block needs
no derived offset at all. ``/mocap_data`` has no header, so its only clock is the
bag's ``log_time``; the median ``log_time - header stamp`` over the hand stream
crosses the announcement into it, and ``t_land_bag`` is reported in bag time and
labelled as such.

Strictly offline and read-only: opens ``.mcap`` files, nothing else. No node, no
socket, no hardware. Exit 0 = every requested bag mined; 1 = a bag was unreadable
or the self-check failed.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
from datetime import datetime

_HERE = os.path.dirname(os.path.abspath(__file__))          # tools/probes
_REPO = os.path.dirname(os.path.dirname(_HERE))             # repo root
# _HERE is on the list because the sibling probe import (ball_arrival_offset) is
# lazy: running `python tools/probes/toss_record_miner.py` puts tools/probes on
# sys.path implicitly, but a consuming test loading this module through
# importlib.spec_from_file_location does NOT — and the failure then surfaces
# minutes later, inside the fit, as a bare ModuleNotFoundError.
for _p in (_HERE, _REPO, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import jugglebot.hardware_config as hw                          # noqa: E402
from jugglebot import toss_record                                # noqa: E402
from jugglebot.toss_record import (                              # noqa: E402
    LABEL_UNKNOWN,
    SensorSample,
    SensorWindows,
    edges,
    label_from_sensor,
    latch_announced_ball,
)

DEFAULT_ROOT = os.path.expanduser('~/Desktop/rosbags')
FIXTURE_PATH = os.path.join(_REPO, 'tests', 'ros', 'toss_record_fixtures.py')
OUT_DIR = os.path.join(_REPO, 'temp', 'probes')

#: Held segments shorter than this are reported as seat-then-leave events. The
#: value Phase 1's ledger used, kept so the two censuses are comparable.
QUICK_DROP_S = 1.5

#: BallState.status: 0 TO_BE_THROWN, 1 IN_FLIGHT, 2 CAUGHT. Mirrored from
#: reload_coordinator_node rather than imported, because importing the node
#: drags rclpy in and this probe must run without ROS. Pinned by the consuming
#: test.
BALL_STATUS_IN_FLIGHT = 1

#: The cup plane a vertical 8a toss arrives at, in GLOBAL mm. NOT
#: ball_arrival_offset's 1000 mm default: for a vertical toss ``v_xy ~ 0`` so a
#: wrong plane looks perfect right up until a displaced throw, at which point
#: every node is biased in a direction correlated with displacement (plan § 7
#: R1). It is the plane the mocap crossing is FITTED at, chosen before any
#: declaration is joined, so `--plane` is the only way to move it; a declaration
#: that disagrees by more than the tolerance below is REFUSED rather than
#: silently re-fitted (`enforce_declared_planes`).
DEFAULT_PLANE_MM = (float(hw.GEOM_INITIAL_HEIGHT_MM)
                    + float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM)
                    + float(hw.HAND_CATCH_OFFSET_MM))
#: The miner REFUSES a declared catch plane this far from the plane it fitted at.
PLANE_MISMATCH_TOL_MM = 5.0

BAND_MM = 300.0
LATERAL_MM = 300.0
MIN_FIT_SAMPLES = 5
PRE_S, POST_S = 1.2, 1.2


def make_windows() -> SensorWindows:
    """The windows exactly as ``reload_coordinator_node`` builds its source.

    Drift-guarded by the consuming test: a bench instrument that scores a capture
    against windows the robot never ran agrees with the robot only by
    coincidence.
    """
    return SensorWindows(arrival_lead_s=float(hw.JB_BD_ARRIVAL_LEAD_S),
                         arrival_window_s=float(hw.JB_BD_ARRIVAL_WINDOW_S),
                         retention_window_s=float(hw.JB_BD_RETENTION_WINDOW_S))


# ── Bag reading ───────────────────────────────────────────────────────────────

def _stamp_s(t) -> float:
    return float(t.sec) + float(t.nanosec) * 1e-9


class BagData(object):
    """Everything one bag contributes, decoded once."""

    def __init__(self):
        self.hand = []            # SensorSample, ROS-stamped
        self.announcements = []   # dicts
        self.declarations = []    # decoded /toss/record rows
        self.balls = []           # (t_bag, [ball, ...])
        self.mocap = []           # (t_bag, x, y, z) unlabelled markers
        self.link = []            # (t_bag, {key: value})
        self.traj = []            # (t_bag, tilt_map_loaded, version, gravity)
        self.log_minus_stamp_s = 0.0
        self.topics = set()


#: Read in TWO passes, and this is not premature optimisation. A sitting's
#: ``/mocap_data`` carries ~137k messages of many unlabelled markers each and
#: ``/balls`` ~93k — decoding those in full costs minutes and gigabytes, while
#: the miner only ever looks at +-1.2 s around each landing. Pass 1 is cheap and
#: yields the announcement instants; pass 2 decodes the heavy streams only inside
#: those windows. ``ball_arrival_offset.read_bag`` splits for the same reason.
_PASS1_TOPICS = ('/hand_telemetry', '/throw_announcements', '/toss/record',
                 '/link_status', '/trajectory/status')
_PASS2_TOPICS = ('/balls', '/mocap_data')

#: Pass-2 window around each landing, in the BAG clock. Wider than the fit's own
#: +-PRE_S/POST_S so the ball latch can see the track appear before release.
_WINDOW_PRE_S = 6.0
_WINDOW_POST_S = 3.0


def read_bag(path: str, *, sensor_only: bool = False) -> BagData:
    """-> :class:`BagData`. Raises ``IOError`` on an unreadable bag.

    ``sensor_only=True`` skips pass 2 entirely — no ``/mocap_data``, no
    ``/balls``, so no mocap block and no tracker evidence. It exists because the
    sensor census (the acceptance this phase is gated on) needs only the light
    half, and pass 2 costs minutes on a full sitting: a consuming test that reads
    the heavy streams to assert a hand-telemetry count would put those minutes
    inside `./run_tests.sh` for no assertion. Rows are still complete records;
    the mocap fields are simply null, which is the same degraded shape a bag with
    no ball marker produces anyway.

    ``mcap`` is imported HERE so ``--self-check`` and the fixture-backed tests
    run without the bag reader installed.
    """
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    files = sorted(glob.glob(os.path.join(path, '*.mcap')))
    if not files:
        raise IOError('no .mcap in {}'.format(path))
    data = BagData()
    raw_hand = []
    for f in files:
        with open(f, 'rb') as fh:
            reader = make_reader(fh, decoder_factories=[DecoderFactory()])
            for _sch, ch, msg, dec in reader.iter_decoded_messages(
                    topics=list(_PASS1_TOPICS)):
                t = msg.log_time / 1e9
                data.topics.add(ch.topic)
                if ch.topic == '/hand_telemetry':
                    raw_hand.append((t, _stamp_s(dec.timestamp),
                                     bool(dec.ball_held),
                                     bool(dec.ball_held_raw),
                                     bool(dec.ball_held_valid),
                                     _stamp_s(dec.ball_held_stamp)))
                elif ch.topic == '/throw_announcements':
                    data.announcements.append({
                        'thrower': str(dec.thrower_name),
                        'target': str(getattr(dec, 'target_id', '') or ''),
                        'throw_time': _stamp_s(dec.throw_time),
                        'landing_time': _stamp_s(dec.landing_time),
                        'landing_position': [float(dec.landing_position.x),
                                             float(dec.landing_position.y),
                                             float(dec.landing_position.z)],
                        't_bag': t,
                    })
                elif ch.topic == '/toss/record':
                    try:
                        data.declarations.append(toss_record.decode(dec.data))
                    except (ValueError, TypeError) as exc:
                        print('WARN  undecodable /toss/record at t={:.3f}: {}'
                              .format(t, exc))
                elif ch.topic == '/link_status':
                    data.link.append(
                        (t, {kv.key: kv.value for kv in dec.values}))
                elif ch.topic == '/trajectory/status':
                    data.traj.append((
                        t, bool(getattr(dec, 'tilt_map_loaded', False)),
                        str(getattr(dec, 'tilt_map_version', '') or ''),
                        bool(getattr(dec, 'gravity_correction_loaded', False))))
    if not raw_hand:
        raise IOError('no /hand_telemetry in {}'.format(path))
    raw_hand.sort()
    offs = sorted(t - s for t, s, _d, _r, _v, _st in raw_hand if s > 0)
    data.log_minus_stamp_s = offs[len(offs) // 2] if offs else 0.0
    # ROS-stamped where the publisher gave us a stamp, else crossed from the
    # bag clock by the median offset — never silently zero, which would put the
    # whole hand stream 56 years away from the announcements.
    for t, s, d, r, v, st in raw_hand:
        data.hand.append(SensorSample(
            t=(s if s > 0 else t - data.log_minus_stamp_s),
            held=d, raw=r, valid=v, stamp=st))

    if sensor_only:
        return data
    windows = [(a['landing_time'] + data.log_minus_stamp_s - _WINDOW_PRE_S,
                a['landing_time'] + data.log_minus_stamp_s + _WINDOW_POST_S)
               for a in data.announcements if a['landing_time'] > 0.0]
    if not windows:
        return data
    lo = min(w[0] for w in windows)
    hi = max(w[1] for w in windows)

    def inside(t):
        if t < lo or t > hi:
            return False
        return any(a <= t <= b for a, b in windows)

    for f in files:
        with open(f, 'rb') as fh:
            reader = make_reader(fh, decoder_factories=[DecoderFactory()])
            for _sch, ch, msg, dec in reader.iter_decoded_messages(
                    topics=list(_PASS2_TOPICS)):
                t = msg.log_time / 1e9
                data.topics.add(ch.topic)
                if not inside(t):
                    continue
                if ch.topic == '/balls':
                    data.balls.append((t, list(dec.balls)))
                else:
                    for mk in dec.markers:
                        if mk.label:
                            continue
                        px, py = float(mk.position.x), float(mk.position.y)
                        if abs(px) > LATERAL_MM or abs(py) > LATERAL_MM:
                            continue
                        data.mocap.append((t, px, py, float(mk.position.z)))
    data.mocap.sort()
    data.balls.sort(key=lambda row: row[0])
    return data


# ── The bag-level sensor ledger (the ACCEPTANCE census) ───────────────────────

def ledger(samples, quick_drop_s: float = QUICK_DROP_S) -> dict:
    """The whole-bag sensor census: departures, catches and seat-then-leaves.

    This is the hand-countable cross-check. ``2026-08-10_16-30-44`` was mined by
    hand at **39 departures / 38 catches / 3 quick-drops**, and
    ``tests/ros/toss_record_fixtures.py`` pins those numbers: an edge rule that
    drifts fails here rather than at the bench.

    Transitions are the DEBOUNCED verdict over VALID samples — the same rule
    ``ball_possession.HandBallSensorSource`` applies and the same rule
    :func:`toss_record.edges` implements, so this table and the per-toss rows are
    two views of one stream rather than two conventions.
    """
    rises, falls = edges(samples, debounced=True)
    segments = []
    open_t = None
    merged = sorted([(t, True) for t in rises] + [(t, False) for t in falls])
    for t, is_rise in merged:
        if is_rise:
            open_t = t
        elif open_t is not None:
            segments.append((open_t, t - open_t))
            open_t = None
    valid = [s for s in samples if s.valid]
    if open_t is not None and valid:
        segments.append((open_t, valid[-1].t - open_t))
    # A ledger that opens with the cup already full has a held segment running
    # from the first sample; it is reported as a segment only when it closes.
    if valid and valid[0].held and rises and (not falls or falls[0] < rises[0]):
        segments.append((valid[0].t, falls[0] - valid[0].t))
        segments.sort()
    quick = [(t, d) for t, d in segments if d < quick_drop_s]
    n = len(samples)
    n_valid = len(valid)
    return {
        'n_samples': n,
        'n_valid': n_valid,
        'valid_frac': (float(n_valid) / n) if n else 0.0,
        'n_catches': len(rises),         # empty -> held
        'n_departures': len(falls),      # held  -> empty
        'n_segments': len(segments),
        'segment_s': sorted(d for _t, d in segments),
        'quick_drops': quick,
        'poll_dt_ms_median': toss_record.poll_dt_ms_median(samples),
    }


# ── Per-toss mining ───────────────────────────────────────────────────────────

def self_tosses(announcements, robot: str):
    """The self-toss announcements, in order.

    ``thrower_name == target_id == robot`` — the filter the plan names (§ 3.4).
    It is what separates our own tosses from the BallButler reload throws that
    ride the SAME topic and land in the SAME cup; on the reference bag it keeps
    32 of 40 announcements, and the 8 it drops have no departure edge because a
    Butler ball never left our hand.
    """
    return [a for a in announcements
            if a['thrower'] == robot and (a['target'] or robot) == robot]


def mine_mocap(data: BagData, landing_ros: float, plane_mm: float,
               band_mm: float = BAND_MM) -> dict:
    """The arrival block for ONE toss, via the IMPORTED estimator."""
    from ball_arrival_offset import fit_plane_crossing_full          # noqa: E402
    out = {'land_plane_mm': float(plane_mm)}
    if not data.mocap:
        return out
    land_bag = landing_ros + data.log_minus_stamp_s
    lo, hi = land_bag - PRE_S, land_bag + POST_S
    window = [(t, x, y, z) for t, x, y, z in data.mocap
              if lo <= t <= hi and abs(x) <= LATERAL_MM and abs(y) <= LATERAL_MM]
    got = fit_plane_crossing_full(window, plane_mm, band_mm)
    if got is None:
        return out
    x, y, n, z_lo, z_hi, rms, _t_lo, _t_hi = got
    gaps = [window[i + 1][0] - window[i][0] for i in range(len(window) - 1)]
    out.update({
        'land_xy_global_mm': [x, y],
        'n_fit': int(n),
        'fit_rms_mm': float(rms),
        'fit_sparse': bool(n < MIN_FIT_SAMPLES),
        'apex_z_mm': max((z for _t, _x, _y, z in window), default=None),
        't_land_bag': land_bag,
        'mocap_gap_ms_max': (max(gaps) * 1e3 if gaps else None),
    })
    del z_lo, z_hi          # reported by the estimator, not carried in the schema
    return out


def mine_link(data: BagData, t_ros: float) -> dict:
    """Provenance the bag witnesses at the release instant.

    ``uptime_ms_at_release`` is mined rather than declared: the coordinator does
    not subscribe to ``/link_status``, and adding a subscription to the node that
    owns the hand, the latch and the abort ladder to obtain a pure covariate is
    exactly the surface D10 spends its argument keeping out. The bag records the
    topic at 5 Hz, so the number is recoverable to ~200 ms — three orders finer
    than the hours-scale drift it exists to partition on.
    """
    if not data.link:
        return {}
    t_bag = t_ros + data.log_minus_stamp_s
    best = min(data.link, key=lambda row: abs(row[0] - t_bag))
    values = best[1]
    out = {}
    uptime = values.get('uptime_ms')
    if uptime is not None:
        try:
            out['uptime_ms_at_release'] = int(float(uptime))
        except (TypeError, ValueError):
            pass
    if values.get('bridge_fw_version'):
        out['bridge_fw_version'] = str(values['bridge_fw_version'])
    if values.get('platform_fw_version'):
        out['platform_fw_version'] = str(values['platform_fw_version'])
    for key, field in (('hand_traj_acks', 'hand_traj_acks'),
                       ('can3_errors', 'can_errors')):
        raw = values.get(key)
        if raw is None:
            continue
        try:
            out[field] = int(float(str(raw).split()[0].rstrip(',')))
        except (TypeError, ValueError, IndexError):
            pass
    if values.get('bridge_tx_diag'):
        out['bridge_tx_diag'] = str(values['bridge_tx_diag'])
    return out


def mine_traj(data: BagData, t_ros: float) -> dict:
    if not data.traj:
        return {}
    t_bag = t_ros + data.log_minus_stamp_s
    t, loaded, version, gravity = min(
        data.traj, key=lambda row: abs(row[0] - t_bag))
    del t
    return {'tilt_map_version': version or None,
            'tilt_map_applied': bool(loaded and gravity),
            'gravity_correction_loaded': bool(gravity)}


def mine_ball_latch(data: BagData, throw_ros: float, landing_ros: float,
                    robot: str):
    """Replay ``/balls`` through the PRODUCTION latch for one toss.

    -> ``(announced_id, untagged)``. The baseline snapshot is the set already
    IN_FLIGHT at the throw instant, which is the node's own GOAL-START rule: our
    ball cannot be airborne before our announcement's throw_time.
    """
    t_lo = throw_ros + data.log_minus_stamp_s
    t_hi = landing_ros + data.log_minus_stamp_s + 1.0
    preexisting = set()
    for t, balls in data.balls:
        if t > t_lo:
            break
        preexisting = {int(b.id) for b in balls
                       if int(b.status) == BALL_STATUS_IN_FLIGHT}
    announced_id, untagged = None, False
    for t, balls in data.balls:
        if t < t_lo or t > t_hi:
            continue
        announced_id, untagged = latch_announced_ball(
            balls, robot_name=robot, announced_id=announced_id,
            preexisting_ids=preexisting, untagged_latch=untagged,
            in_flight_status=BALL_STATUS_IN_FLIGHT)
    return announced_id, untagged


def mine_bag(data: BagData, *, robot: str = 'jugglebot',
             windows: SensorWindows = None, plane_mm: float = None) -> list:
    """-> the mined half: one row per self-toss announcement, in order."""
    windows = windows or make_windows()
    rows = []
    anchored = _stamp_wall_anchored(data)
    for ann in self_tosses(data.announcements, robot):
        throw_ros = ann['throw_time']
        landing_ros = ann['landing_time']
        row = toss_record.blank_record()
        # cycle_index and action are left NULL on the mined side on purpose.
        # The schema's M source for cycle_index is the action feedback stream
        # (§ 3.3), not the announcement's position in the bag: "the 7th
        # announcement in this sitting" and "cycle 7 of this session" are
        # different quantities, and filling one with the other would fabricate a
        # D+M agreement on every single-toss bag and a D+M CONFLICT on every
        # session bag. Likewise a bag alone cannot tell a single Toss from a
        # TossContinuous cycle. Both arrive with the declaration, or not at all.
        #
        # The PLANT block ships NULL in 2a, and `plant_block_source` with it —
        # a source string on an empty block would assert a provenance that does
        # not exist. It is not re-implementable here by design (§ 3.4 says the
        # row builder is IMPORTED), and wiring `hand_stroke_timeline` needs its
        # session-relative clock mapped back onto these ROS instants. Getting
        # that mapping wrong writes plausible WRONG numbers into a corpus, which
        # is precisely the failure class this whole phase exists to prevent, and
        # nothing here can currently validate the result. Deferred with the
        # field mapping already worked out — see the plan's § 10.
        row.update({
            'announce_throw_time_ros': throw_ros,
            'announce_landing_time_ros': landing_ros,
        })
        block = label_from_sensor(data.hand, throw_time=throw_ros,
                                 landing_time=landing_ros, windows=windows,
                                 stamp_wall_anchored=anchored)
        row.update(block.fields)
        row['label'] = block.label
        row['label_source'] = 'hand_ball_sensor'
        row['label_reason'] = block.reason
        row['label_confidence'] = block.confidence
        row['rimshot'] = bool((block.fields.get('sensor_edge_count') or 0) > 2)
        this_plane = plane_mm if plane_mm is not None else DEFAULT_PLANE_MM
        row.update(mine_mocap(data, landing_ros, this_plane))
        row.update(mine_link(data, throw_ros))
        row.update(mine_traj(data, throw_ros))
        ball_id, _untagged = mine_ball_latch(data, throw_ros, landing_ros, robot)
        row['ball_track_confirmed'] = ball_id is not None
        # land_err needs a catch point, and the ANNOUNCED landing position is
        # that point on BOTH tiers — not just 8a. `_announce_toss` builds the
        # announcement from `_toss_release_state`, which is the UNCORRECTED
        # release, and the declaration's `catch_point_global_mm` comes from the
        # same object; an aim correction only ever moves `_toss_release_cmd`
        # (D4). So this IS the schema's definition of land_err_mm (`land_xy −
        # catch_point_global_mm[:2]`) computed from the mined half, and the join
        # deliberately does NOT recompute it — there is nothing to correct.
        # (The comment here used to claim the join overrode it. It never did,
        # and it never needed to; corrected with the § 7 R1 audit fix.)
        # What the join DOES add is the plane check — see
        # `enforce_declared_planes`, which needs both halves and so cannot run
        # from here.
        cup = ann.get('landing_position')
        if cup and row.get('land_xy_global_mm'):
            row['land_err_mm'] = [row['land_xy_global_mm'][0] - cup[0],
                                  row['land_xy_global_mm'][1] - cup[1]]
            row['land_err_norm_mm'] = math.hypot(*row['land_err_mm'])
        _mark_usable(row)
        rows.append(row)
    return rows


def _stamp_wall_anchored(data: BagData):
    """Is ``ball_held_stamp`` wall-epoch, or still bridge-boot-relative?

    A bridge-relative stamp joined to wall time mis-times every edge by the whole
    bridge uptime — the largest silent error in this pipeline (plan § 7 R2). The
    discriminator is blunt on purpose: a wall stamp sits within a minute of the
    sample's own ROS stamp, a boot-relative one is a small number of seconds
    since 1970. Nothing subtle can go wrong with it.
    """
    for s in data.hand:
        if s.valid and s.stamp > 0.0:
            return abs(s.stamp - s.t) < 60.0
    return None


def _mark_usable(row: dict) -> None:
    """Guards G1/G2/G9 as far as a bag alone can decide them.

    The rest (G3 apex sanity, G4 plant health, G5 levelling, G6 uptime trend,
    G7/G8 outlier and change detection) belong to the FIT, not the record: they
    need the partition and the running estimate, which a single row does not
    have. Marking them here would be a guard that looks enforced and is not.
    """
    reasons = []
    if row.get('label') in (LABEL_UNKNOWN, None):
        reasons.append('label_unknown')
    if row.get('t_departure_raw_ros') is None and not row.get(
            'ball_track_confirmed'):
        reasons.append('no_release_evidence')          # G1
    if row.get('land_xy_global_mm') is None:
        reasons.append('no_mocap_fit')
    elif row.get('fit_sparse') or (row.get('fit_rms_mm') or 0.0) > 3.0:
        reasons.append('mocap_fit_quality')            # G2
    row['usable_for_aim_fit'] = not reasons
    timing_reasons = list(reasons)
    if row.get('ball_held_stamp_wall_anchored') is False:
        timing_reasons.append('bridge_stamp_not_wall_anchored')
    if row.get('rimshot'):
        timing_reasons.append('rimshot_candidate')
    row['usable_for_timing_fit'] = not timing_reasons
    row['usable_for_speed_fit'] = not reasons
    row['excluded_reason'] = ','.join(sorted(set(reasons))) or None


def mismatched_planes(rows) -> list:
    """The rows whose declared cup plane disagrees with the plane they were
    fitted at. PURE — it decides nothing and marks nothing.

    A declared ``catch_point_global_mm[2]`` more than 5 mm from the plane the fit
    actually ran at means the corpus is scoring arrivals at the wrong height —
    invisible for a vertical toss, and biased in a direction correlated with
    displacement the moment a throw is displaced.
    """
    bad = []
    for row in rows:
        cup = row.get('catch_point_global_mm')
        plane = row.get('land_plane_mm')
        if not cup or plane is None or cup[2] is None:
            continue
        if abs(float(cup[2]) - float(plane)) > PLANE_MISMATCH_TOL_MM:
            bad.append(row)
    return bad


def check_declared_planes(rows) -> list:
    """LOUD, one line each, never averaged away (plan § 7 R1). PURE."""
    return ['PLANE MISMATCH {}: declared cup z {:.1f} mm, fitted at {:.1f} mm '
            '— REFUSED for the aim + speed fits; re-mine with --plane {:.1f}'
            .format(row.get('toss_uid'),
                    float(row['catch_point_global_mm'][2]),
                    float(row['land_plane_mm']),
                    float(row['catch_point_global_mm'][2]))
            for row in mismatched_planes(rows)]


def enforce_declared_planes(rows) -> list:
    """REFUSE every row whose declared cup plane disagrees with the fit plane,
    and return the loud lines. Called on the JOINED rows, which is the earliest
    point both halves exist.

    AUDIT FIX 2026-08-11. Plan § 7 R1 says the miner *refuses* a plane differing
    from ``catch_point_global_mm[2]`` by more than 5 mm; it was only PRINTING,
    and the mismatched rows kept ``usable_for_aim_fit: true``. A printed warning
    protects nothing downstream: ``toss_cal_fit`` and the record miner's own
    ``--jsonl`` corpus both select on the flag, not on a human having read stdout
    — and the whole point of R1 is that this error is *invisible in the
    residuals* until a displaced throw, which is exactly when the map gets used.

    Aim and speed are refused; TIMING is not. The timing fit reads sensor edges
    and release instants, which the fit plane has no bearing on whatsoever, and
    refusing it would throw away good data for an unrelated fault.

    The plane cannot simply be corrected here: ``mine_mocap`` fits the crossing
    at a plane chosen BEFORE the declaration is joined, so a mismatch means the
    mocap window was already reduced against the wrong height. Re-mining with
    ``--plane <declared z>`` is the fix, and the message says so.
    """
    lines = check_declared_planes(rows)
    for row in mismatched_planes(rows):
        row['usable_for_aim_fit'] = False
        row['usable_for_speed_fit'] = False
        reasons = [r for r in (row.get('excluded_reason') or '').split(',') if r]
        row['excluded_reason'] = ','.join(
            sorted(set(reasons + ['plane_mismatch'])))
    return lines


# ── Reporting ─────────────────────────────────────────────────────────────────

def print_report(name: str, rows, led: dict, data: BagData) -> None:
    print('\n=== {}'.format(name))
    print('  topics present: {}'.format(
        ', '.join(sorted(data.topics)) or 'none'))
    print('  ledger: {} samples, ball_held_valid {}/{} ({:.1f} %), '
          '{} catches (empty->held), {} departures (held->empty), '
          '{} held segments'.format(
              led['n_samples'], led['n_valid'], led['n_samples'],
              100.0 * led['valid_frac'], led['n_catches'], led['n_departures'],
              led['n_segments']))
    if led['poll_dt_ms_median'] is not None:
        print('  MEASURED sensor poll cadence: {:.0f} ms median (configured '
              'JB_BD_CHECK_INTERVAL_MS = {})'.format(
                  led['poll_dt_ms_median'], hw.JB_BD_CHECK_INTERVAL_MS))
    if led['quick_drops']:
        print('  quick-drops (seat-then-leave): {}'.format(
            ', '.join('t={:.3f}s dur={:.3f}s'.format(t, d)
                      for t, d in led['quick_drops'])))
    else:
        print('  quick-drops (seat-then-leave): none')
    print('  {:>4} {:>12} {:>10} {:>9} {:>9} {:>9}  {}'.format(
        '#', 'throw[ros]', 'label', 'dep[ms]', 'catch[ms]', 'err[mm]', 'prov'))
    # The leading column is the row's position in THIS report, not cycle_index:
    # a mined-only row has no cycle_index, because "the 7th announcement in the
    # bag" is a different quantity from "cycle 7 of the session" (see mine_bag).
    for i, row in enumerate(rows, 1):
        throw = row.get('announce_throw_time_ros') or 0.0
        dep = row.get('t_departure_raw_ros')
        cat = row.get('t_catch_raw_ros')
        land = row.get('announce_landing_time_ros') or 0.0
        err = row.get('land_err_norm_mm')
        print('  {:>4} {:12.3f} {:>10} {:>9} {:>9} {:>9}  {}'.format(
            i, throw, row.get('label'),
            '{:+.0f}'.format((dep - throw) * 1e3) if dep else '-',
            '{:+.0f}'.format((cat - land) * 1e3) if cat else '-',
            '{:.1f}'.format(err) if err is not None else '-',
            row.get('record_provenance') or '-'))
    counts = {}
    for row in rows:
        counts[row['label']] = counts.get(row['label'], 0) + 1
    print('  labels: ' + ', '.join('{}={}'.format(k, counts[k])
                                   for k in sorted(counts)))
    usable = sum(1 for r in rows if r.get('usable_for_aim_fit'))
    print('  usable_for_aim_fit: {}/{}'.format(usable, len(rows)))
    for problem in check_declared_planes(rows):
        print('  ' + problem)


def write_outputs(name: str, rows, led: dict) -> str:
    os.makedirs(OUT_DIR, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    base = os.path.join(OUT_DIR, 'toss_records_{}_{}'.format(name, stamp))
    with open(base + '.jsonl', 'w') as fh:
        for row in rows:
            fh.write(toss_record.encode(row) + '\n')
    with open(base + '_meta.json', 'w') as fh:
        json.dump({'bag': name, 'schema': toss_record.SCHEMA,
                   'ledger': {k: v for k, v in led.items()
                              if k != 'segment_s'},
                   'n_rows': len(rows)}, fh, indent=1)
    return base


# ── Fixture (committed cut + its regeneration recipe) ─────────────────────────

_FIXTURE_HEADER = '''"""Measured toss-record fixtures — DO NOT hand-edit.

The hand-mined ground truth for {bag}, and the acceptance
this phase is gated on: the miner has to reproduce it from the bag alone.

Regenerate with:
    python tools/probes/toss_record_miner.py --bag {bag} --emit-fixture

Consumed by tests/ros/test_toss_record_miner.py. Schema: {schema}
(jugglebot/toss_record.py). Plan: plans/active/toss-selftuning.md § 3.3.

The bag itself is machine-local and gitignored, so the bag-backed test skips on
any machine without it — including a fresh clone. These constants are what
carries the evidence across that boundary, which is the whole point of the
tests/ros/possession_fixtures.py pattern this file follows.
"""

from __future__ import annotations

#: The reference sitting. Every number below is measured FROM it.
REFERENCE_BAG = {bag!r}

'''


def emit_fixture(name: str, led: dict, rows, path: str = FIXTURE_PATH) -> str:
    """Write the committed acceptance census for ``name``.

    Deliberately a CENSUS, not a sample cut: the hand-mined ground truth for this
    bag is *counts* (39 departures / 38 catches / 3 quick-drops), the counts are
    what an orchestrator can verify by eye, and a fixture that stored 70,666
    samples would be a 3 MB blob nobody re-derives. The sample-level cut already
    exists next door in ``hand_sensor_replay_fixture.json`` and pins the edge
    instants; this pins the whole-bag totals those instants have to add up to.

    Emitted as an importable ``.py`` of constants — the
    ``tests/ros/possession_fixtures.py`` shape — so the numbers are diffable in
    review and a re-baseline is a visible commit rather than a blob swap.
    """
    dep = sorted(round((r['t_departure_raw_ros'] - r['announce_throw_time_ros'])
                       * 1e3, 1)
                 for r in rows if r.get('t_departure_raw_ros'))
    lag = sorted(round((r['t_departure_deb_ros'] - r['t_departure_raw_ros'])
                       * 1e3, 1)
                 for r in rows
                 if r.get('t_departure_deb_ros') and r.get('t_departure_raw_ros'))
    body = [
        ('LEDGER', 'The whole-bag sensor census. n_catches = empty->held edges,\n'
                   '#: n_departures = held->empty, both on the DEBOUNCED verdict over\n'
                   '#: VALID samples. Hand-mined 2026-08-10: 39 departures, 38 catches.',
         {'n_samples': led['n_samples'], 'n_valid': led['n_valid'],
          'n_catches': led['n_catches'], 'n_departures': led['n_departures'],
          'n_segments': led['n_segments']}),
        ('QUICK_DROP_S', 'Held segments shorter than this are seat-then-leave events.',
         QUICK_DROP_S),
        ('QUICK_DROP_DURATIONS_S',
         'The three sub-second seat-then-leaves. Phase 1 sized\n'
         '#: JB_BD_RETENTION_WINDOW_S on these.',
         tuple(round(d, 4) for _t, d in sorted(led['quick_drops']))),
        ('POLL_DT_MS_MEDIAN',
         'MEASURED hand-sensor poll cadence (median ball_held_stamp advance),\n'
         '#: against a CONFIGURED JB_BD_CHECK_INTERVAL_MS of 20. The gap is the\n'
         '#: reason sensor_poll_dt_ms_median is a mined field and not an assumption.',
         round(led['poll_dt_ms_median'], 3)),
        ('N_SELF_TOSSES',
         'Announcements with thrower_name == target_id == the robot.',
         len(rows)),
        ('LABELS', 'label_from_sensor over every self-toss in the bag.',
         _label_counts(rows)),
        ('DEPARTURE_DT_MS',
         'RAW held->empty edge minus the announced throw_time, per self-toss.\n'
         '#: This is the band DEPARTURE_WINDOW_S is sized against.',
         tuple(dep)),
        ('DEBOUNCE_FALL_LAG_MS',
         'Debounced departure minus RAW departure. THE D12 measurement: the\n'
         '#: debounce is asymmetric (0 ms on a rise, ~240 ms on a fall), so a\n'
         '#: timing fit taken off the debounced bit carries a systematic lag\n'
         '#: comparable to the uptime dispatch shift it would be measuring.',
         tuple(lag)),
        ('RECORD_PROVENANCE',
         'This bag predates /toss/record, so every row is mined-only — which is\n'
         '#: exactly the degraded-form capability § 3.3 requires of the miner.',
         tuple(sorted(set(r.get('record_provenance') or 'mined-only'
                          for r in rows)))),
    ]
    lines = [_FIXTURE_HEADER.format(bag=name, schema=toss_record.SCHEMA)]
    for key, doc, value in body:
        lines.append('#: {}\n{} = {!r}\n\n'.format(doc, key, value))
    with open(path, 'w') as fh:
        fh.write(''.join(lines).rstrip() + '\n')
    return path


def _label_counts(rows) -> dict:
    counts = {}
    for row in rows:
        counts[row['label']] = counts.get(row['label'], 0) + 1
    return counts


# ── Self-check (bag-free instrument acceptance) ───────────────────────────────

def _synth(throw=100.0, flight=0.8, *, departure_dt=0.17, catch_dt=None,
           drop_after=None, blind_from=None, blind_to=None, dt=0.01,
           debounce_fall_s=0.24):
    """A synthetic hand stream with the REAL debounce asymmetry baked in.

    ``debounce_fall_s`` reproduces the measured behaviour — a fall on the raw bit
    reaches the debounced verdict ~240 ms later, a rise reaches it immediately
    (``plans/active/hand-ball-sensor.md`` § Debounce asymmetry; measured
    0/0/0 ms rise and 232/241/295 ms fall on 2026-08-10_16-30-44). A self-check
    driven by a SYMMETRIC synthetic stream would pass while the labeller read the
    wrong bit, which is the whole failure D12 exists to prevent.
    """
    landing = throw + flight
    t_dep = None if departure_dt is None else throw + departure_dt
    t_catch = None if catch_dt is None else landing + catch_dt
    t_drop = None if (t_catch is None or drop_after is None) else (
        t_catch + drop_after)
    out = []
    t = throw - 3.0
    end = landing + 6.0
    while t < end:
        raw = True
        if t_dep is not None and t >= t_dep:
            raw = False
        if t_catch is not None and t >= t_catch:
            raw = True
        if t_drop is not None and t >= t_drop:
            raw = False
        # Debounced: falls lag, rises do not.
        deb = True
        if t_dep is not None and t >= t_dep + debounce_fall_s:
            deb = False
        if t_catch is not None and t >= t_catch:
            deb = True
        if t_drop is not None and t >= t_drop + debounce_fall_s:
            deb = False
        valid = not (blind_from is not None and blind_from <= t <= blind_to)
        out.append(SensorSample(t=t, held=deb, raw=raw, valid=valid,
                                stamp=t if valid else 0.0))
        t += dt
    return out, throw, landing


def self_check() -> int:
    """Two-sided by construction, and pinned to MEASURED instants.

    Every ACCEPT case has a REFUSE twin, and the twins are ABSOLUTE where they
    guard a shipped constant: a case written relative to the constant it guards
    follows that constant wherever it goes and can only catch a NARROWING — the
    one-sidedness Phase 1's audit found in the retention guard next door.
    """
    fails = []
    checks = [0]
    windows = make_windows()

    def check(name, got, want):
        checks[0] += 1
        if got != want:
            fails.append('{}: got {!r}, want {!r}'.format(name, got, want))

    def run(**kw):
        samples, throw, landing = _synth(**kw)
        return label_from_sensor(samples, throw_time=throw,
                                 landing_time=landing, windows=windows)

    # ACCEPT — the measured catch band, both ends (min +137 ms, max +798 ms on
    # the reference bag). The +798 row is the one that SIZED arrival_window_s.
    check('earliest measured catch (+137 ms)',
          run(catch_dt=0.137).label, 'CAUGHT')
    check('latest measured catch (+798 ms)',
          run(catch_dt=0.798).label, 'CAUGHT')
    # REFUSE — beyond the shipped arrival window. ABSOLUTE (1.6 s), so a widened
    # window fails here. The plan's draft used CATCH_CONFIRM_WINDOW_S = 0.70 s,
    # which relabels the +798 ms row above — one of the reference bag's 25
    # catches, and the population maximum that sized the shipped window.
    check('an arrival past the SHIPPED 1.5 s window',
          run(catch_dt=1.6).label, 'MISSED')
    check('no arrival at all', run(catch_dt=None).label, 'MISSED')
    # Departure gates.
    check('the measured departure band, late end (+212 ms)',
          run(departure_dt=0.212, catch_dt=0.4).label, 'CAUGHT')
    check('no departure edge => NO_RELEASE',
          run(departure_dt=None, catch_dt=None).label, 'NO_RELEASE')
    check('a NO_RELEASE cup stays held at dispatch',
          run(departure_dt=None,
              catch_dt=None).fields['sensor_held_at_dispatch'], True)
    # Retention, both sides, absolute on the widen side.
    check('a 0.999 s seat-then-leave (the longest measured) is a BOUNCE',
          run(catch_dt=0.4, drop_after=0.999).label, 'BOUNCED')
    check('a 1.55 s hold at the SHIPPED 1.50 s window (widen guard)',
          run(catch_dt=0.4, drop_after=1.55).label, 'CAUGHT')
    # UNKNOWN never collapses to a verdict (D13).
    check('blind across the arrival window => UNKNOWN',
          run(catch_dt=0.4, blind_from=100.5, blind_to=101.5).label, 'UNKNOWN')
    check('UNKNOWN carries no usable flag',
          run(catch_dt=0.4, blind_from=100.5,
              blind_to=101.5).confidence, 0.0)
    # D12, the whole point: the debounced fall is ~240 ms late and the RAW edge
    # is the one a timing fit may use.
    block = run(departure_dt=0.17, catch_dt=0.4)
    raw_dt = block.fields['t_departure_raw_ros'] - 100.0
    deb_dt = block.fields['t_departure_deb_ros'] - 100.0
    check('raw departure lands in the measured band',
          0.16 <= raw_dt <= 0.19, True)
    check('debounced departure lags raw by the measured ~240 ms',
          0.20 <= (deb_dt - raw_dt) <= 0.28, True)
    # The departure window cannot reach into the arrival window.
    check('a bounce-out is NOT mistaken for a departure',
          run(departure_dt=0.17, catch_dt=0.4,
              drop_after=0.5).fields['t_departure_raw_ros'] < 100.5, True)
    # Schema round-trip.
    rec = toss_record.blank_record()
    rec.update({'toss_uid': 'x', 'action': 'toss', 'outcome': 'CAUGHT',
                'catch_error_mm_fsm': float('nan')})
    check('a NaN encodes as null, not a bare NaN token',
          toss_record.decode(toss_record.encode(rec))['catch_error_mm_fsm'],
          None)
    check('a blank record validates once its required fields are set',
          toss_record.validate(toss_record.decode(toss_record.encode(rec))), ())
    check('validate REFUSES a NaN that bypassed the encoder',
          toss_record.validate(rec),
          ('catch_error_mm_fsm: want a finite number',))
    check('the join tolerates a mined row with no declaration',
          toss_record.join([], [{'announce_throw_time_ros': 1.0}]
                           )[0]['record_provenance'], 'mined-only')
    check('the join keeps an unmatched declaration',
          toss_record.join([{'announce_throw_time_ros': 1.0,
                             'toss_uid': 'd'}], [])[0]['record_provenance'],
          'declared-only')
    joined = toss_record.join(
        [{'announce_throw_time_ros': 1.0, 'toss_uid': 'd',
          'tilt_map_version': 'A'}],
        [{'announce_throw_time_ros': 1.002, 'tilt_map_version': 'B'}])
    check('a D+M conflict is RECORDED, never resolved',
          len(joined[0]['disagreement']), 1)
    check('the join residual is reported in ms',
          round(joined[0]['join_residual_ms'], 3), 2.0)
    check('outside the tolerance the halves do NOT join',
          toss_record.join([{'announce_throw_time_ros': 1.0}],
                           [{'announce_throw_time_ros': 1.02}]
                           )[0]['record_provenance'], 'mined-only')

    for f in fails:
        print('FAIL  {}'.format(f))
    print('{}/{} self-check cases pass'.format(checks[0] - len(fails), checks[0]))
    return 1 if fails else 0


# ── CLI ───────────────────────────────────────────────────────────────────────

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bag', action='append', default=[],
                    help='bag directory name under --root (repeatable)')
    ap.add_argument('--root', default=DEFAULT_ROOT)
    ap.add_argument('--robot', default='jugglebot')
    ap.add_argument('--plane', type=float, default=None,
                    help='fit plane in GLOBAL mm (default: the 8a cup plane, '
                         '{:.1f})'.format(DEFAULT_PLANE_MM))
    ap.add_argument('--jsonl', action='store_true',
                    help='write the corpus + meta to temp/probes/')
    ap.add_argument('--sensor-only', action='store_true',
                    help='skip the mocap + tracker pass (minutes faster; the '
                         'mocap block comes out null)')
    ap.add_argument('--self-check', action='store_true')
    ap.add_argument('--emit-fixture', action='store_true',
                    help='write the committed acceptance census (one --bag)')
    args = ap.parse_args(argv)

    if args.self_check:
        return self_check()
    if not args.bag:
        ap.error('need --bag (or --self-check)')

    rc = 0
    for name in args.bag:
        path = os.path.join(args.root, name)
        try:
            data = read_bag(path, sensor_only=args.sensor_only)
        except (IOError, OSError) as exc:
            print('ERROR  {}: {}'.format(name, exc))
            rc = 1
            continue
        mined = mine_bag(data, robot=args.robot, plane_mm=args.plane)
        rows = toss_record.join(data.declarations, mined)
        # Plan § 7 R1's REFUSAL, applied at the earliest point both halves of
        # the comparison exist: the mined row owns `land_plane_mm`, the
        # declaration owns `catch_point_global_mm`, and only the join has both.
        # Before `write_outputs`, so the flag the corpus carries is the enforced
        # one — a jsonl written with `usable_for_aim_fit: true` on a
        # wrong-plane row is exactly the artefact the refusal exists to stop.
        enforce_declared_planes(rows)
        led = ledger(data.hand)
        print_report(name, rows, led, data)
        if args.jsonl:
            print('  wrote {}.jsonl'.format(write_outputs(name, rows, led)))
        if args.emit_fixture:
            print('  wrote {}'.format(emit_fixture(name, led, rows)))
    return rc


if __name__ == '__main__':
    sys.exit(main())
