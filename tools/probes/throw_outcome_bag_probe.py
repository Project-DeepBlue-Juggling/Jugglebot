#!/usr/bin/env python3
"""Ground-truth a self-toss THROW outcome from a rosbag — skill-stack R3 probe.

WHAT IT DOES
------------
For every machine self-toss (a ``/throw_announcements`` with
``thrower_name == target_id``, or, when no announcement exists, a
``/toss/record`` declaration) in one or more rosbags:

1. Fits a **known-gravity parabola** to the raw ``/mocap_data`` unlabelled
   marker positions over the ball's free-flight window (least squares via
   ``sim.juggle_noise.BallisticEstimator`` — the one estimator this probe
   uses; it is not re-implemented here) and reports the ANALYTIC crossing of
   two planes: ``z = CATCH_CUP_Z_MM`` (830.0 mm, ``motion/skills/sites.py``,
   the skill stack's catch plane) and ``z = 809.08 mm`` (the tracker node's
   own landing plane, ``GEOM_INITIAL_HEIGHT_MM`` + ``JB_OP_DEFAULT_ACTIVE_Z_MM``
   + ``HAND_CATCH_OFFSET_MM`` — see ``ball_tracker_node.py``). This is the
   probe's GROUND TRUTH.
2. Scores ``/balls`` (``BallStateArray``) landing-prediction CANDIDATES
   against that ground truth (plan § 2.7 asks which one the learner's
   observed ``y`` should be):
     (a) the LAST prediction before the ball crosses the tracker's own plane
         (809.08 mm) or is caught, whichever is earlier -- scored against the
         809.08 mm ground truth (that IS the plane this field targets);
     (b) the prediction at ``t_cross_830 - 0.1 s`` (the CATCH freeze instant
         the executor stops re-aiming at);
     (c) the prediction ~150 ms after the physical release;
     (e) the tracker's own (position, velocity) at its LAST ``/balls`` update
         while the ball is still above ``830 + 50`` mm, projected forward to
         z = 830 mm via
         ``jugglebot.motion.trajectory.ballistics_bc.arrival_state_at_z``
         (the same call ``sim/skills_gate.py``'s tracker uses);
     (e') the same projection, from the update nearest the executor's CATCH
         freeze instant (``t_cross_830 - executor.CATCH_FREEZE_S``).
   Candidate (d) (retired 2026-09-13) re-projected from candidate (a)'s
   state, which is already at/under 809.08 mm -- i.e. already PAST the
   830 mm plane on the way down, so no forward 830 mm crossing exists from
   it; (e)/(e') fix this by picking a state that is provably still above the
   830 mm plane (by 50 mm) before projecting forward.
3. Counts flights with no usable track, a late-starting track, or a phantom
   distractor track (§ 2.7's "no observation" question), using a checkable
   predicate.
4. Reports ``flight_s`` = release -> ground-truth crossing, using the
   ANNOUNCED ``throw_time`` as the commanded release (what the R3 schedule
   will have) when a ``/throw_announcements`` exists for the flight, and the
   offset between that commanded release and the mocap fit's first free-flight
   sample (the PHYSICAL departure).
5. Reports, for flights with a departure-then-SEATED sequence, how long after
   the ground-truth 830 mm crossing ``/hand_telemetry``'s
   ``ball_held_raw and ball_held_valid`` first reads True (the ``caught``
   signal source, plan § 2.7).

CLOCK DOMAIN (a deliberate simplification — read before trusting a sub-5 ms
number): every timestamp in this probe is the bag's **``log_time``**
(mcap arrival-order clock), not each message's own ``header.stamp``. This
matches how ``/mocap_data`` must be handled (it carries no header at all —
see ``toss_record_miner.py``'s CLOCKS section) and lets every topic here be
compared on one shared axis without a five-way clock-conversion chain that
this probe's actual question does not need to the millisecond. The tradeoff:
a header-stamped topic's ``log_time`` lags its ``header.stamp`` by whatever
that publisher's queueing + transport latency was (a few ms, typically) — see
``ros_bag_offset_probe_s`` in the summary, which reports the measured
BallState ``header.stamp`` vs ``log_time`` offset for this bag so a reader can
judge whether it matters for a given number.

Candidates (e)/(e') are defined against the row's ``header.stamp`` ("the
tracker's (position, velocity, header stamp)") but implemented as
``row.log_time + ttl`` — adding ``ttl`` to ``header.stamp`` and then
converting the predicted instant back to ``log_time`` via THIS SAME row's
own ``(log_time - header.stamp)`` offset is arithmetically identical to
``row.log_time + ttl``, so the two are the same number; using the row's own
exact pair (not the bag-wide MEDIAN ``ros_bag_offset_probe_s``) is what
makes that identity exact rather than approximate.

REUSE, NOT REIMPLEMENTATION
----------------------------
* ``sim.juggle_noise.BallisticEstimator`` — the ONE gravity-parabola fit.
* ``jugglebot.motion.trajectory.ballistics_bc.arrival_state_at_z`` —
  candidates (e)/(e')'s re-projection; the same boundary-condition solver
  ``sim/skills_gate.py``'s tracker calls.
* Mocap marker selection (fixture-cell dropping, a lateral gate) is a fresh,
  deliberately small greedy nearest-neighbour tracker — NOT
  ``toss_record_miner``'s whole-arc / E-1-corrected estimator, because this
  probe's job is to score that production pipeline's ``/balls`` output
  against an INDEPENDENT ground truth, not to reuse it.

USAGE
-----
    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/probes/throw_outcome_bag_probe.py --bag 2026-09-06_19-30-10
    python tools/probes/throw_outcome_bag_probe.py --bag A --bag B --bag C

Writes ``temp/probes/throw_outcome_<bag>.csv`` (one row per flight) and
``temp/probes/throw_outcome_summary.md`` (all bags, one report). Deterministic,
read-only, pure offline (no ROS runtime) — opens ``.mcap`` files only.
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import sys
from datetime import datetime

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))          # tools/probes
_REPO = os.path.dirname(os.path.dirname(_HERE))
_ROS_PKG = os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot')
sys.path.insert(0, _ROS_PKG)
sys.path.insert(0, _REPO)

from sim.juggle_noise import BallisticEstimator                 # noqa: E402
from jugglebot.motion.trajectory.ballistics_bc import (          # noqa: E402
    arrival_state_at_z)
from jugglebot.motion.skills.executor import CATCH_FREEZE_S      # noqa: E402
import jugglebot.hardware_config as hw                           # noqa: E402

GRAVITY_MMPS2 = 9806.0
G_VEC = np.array([0.0, 0.0, -GRAVITY_MMPS2])

CATCH_CUP_Z_MM = 830.0
TRACKER_LANDING_Z_MM = (hw.GEOM_INITIAL_HEIGHT_MM
                         + hw.JB_OP_DEFAULT_ACTIVE_Z_MM
                         + hw.HAND_CATCH_OFFSET_MM)
assert abs(TRACKER_LANDING_Z_MM - 809.08) < 0.01, TRACKER_LANDING_Z_MM

# Mocap track-builder tuning (greedy nearest-neighbour association).
_MAX_SPEED_MMPS = 9000.0        # generous upper bound on ball speed
_GATE_MARGIN_MM = 60.0          # + constant slack on the association gate
_MAX_GAP_S = 0.08               # longest mocap dropout a track survives
_FIXTURE_CELL_MM = 20.0         # grid cell used to drop labelled-marker echoes
_FIXTURE_TOL_MM = 15.0          # distance to a fixture cell that drops a row
_MIN_TRACK_PTS = 6
_MIN_Z_RANGE_MM = 300.0         # a real self-toss excursion, not rig jitter;
                                 # NOT tied to the exact 830/809 planes -- a
                                 # self-toss can be physically caught above
                                 # or below either nominal plane, and requiring
                                 # the sample z-range to straddle both planes
                                 # with margin silently rejected real flights
                                 # whose measured catch height differs from
                                 # the plan's nominal 809.08 mm.

CATCH_FREEZE_LEAD_S = 0.10      # plan § 2.4 / § 2.7: t_cross_830 - this
RELEASE_SAMPLE_LEAD_S = 0.150   # candidate (c)


# ─────────────────────────── bag reading helpers ──────────────────────────

def _reader(path):
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    return make_reader(open(path, 'rb'), decoder_factories=[DecoderFactory()])


def _iter(bag_dir, topics):
    for path in sorted(glob.glob(os.path.join(bag_dir, '*.mcap'))):
        r = _reader(path)
        for schema, channel, message, ros_msg in r.iter_decoded_messages(
                topics=topics):
            yield channel.topic, message.log_time * 1e-9, ros_msg


class Flight(object):
    """One self-toss: identity + everything mined about it."""

    def __init__(self, idx, source):
        self.idx = idx
        self.source = source            # 'announcement' | 'toss_record'
        self.t_release_cmd = None       # commanded release, bag clock
        self.t_land_pred = None         # announcement's own landing estimate
        self.toss_uid = None
        self.record = None              # raw toss_record dict, if joined

        # Ground truth (filled by _fit_ground_truth)
        self.track_pts = None
        self.t_release_phys = None
        self.p0 = self.v0 = None
        self.t_ref = None
        self.t_cross_830 = None
        self.t_cross_809 = None
        self.gt_xy_830 = None
        self.gt_xy_809 = None
        self.candidate_tracks_seen = 0  # z-range-plausible tracks in window,
                                         # for the phantom-ball census
        self.gt_labels = None           # distinct QTM labels in the chosen
                                         # ground-truth track ('' = unlabelled)
        self.gt_fit_rms_mm = None
        self.gt_source = None           # 'unlabelled' | 'labelled-fallback'

        # /balls candidates
        self.ball_id = None
        self.no_track = None            # never reached tracking==CONFIRMED
        self.late_start_s = None        # first CONFIRMED sample - t_release_phys
        self.n_distinct_ids_near = None
        self.observed_predicate = None  # probe (1b) "observed" predicate
        self.candidates = {}            # 'a'/'b'/'c'/'d' -> dict(xy, t, err_mm, err_ms)

        # Possession
        self.t_seated = None
        self.caught = None


def parse_bag_name(spec):
    """Accept a bare bag name or a path; resolve under ~/Desktop/rosbags."""
    if os.path.isdir(spec):
        return spec
    return os.path.expanduser(os.path.join('~/Desktop/rosbags', spec))


# ─────────────────────────── mocap track builder ───────────────────────────

def _cell(x, y, z, size):
    return (round(x / size), round(y / size), round(z / size))


def build_fixture_cells(rows, size=_FIXTURE_CELL_MM):
    return {_cell(r[1], r[2], r[3], size) for r in rows}


def drop_fixture_echoes(rows, fixture_cells, size=_FIXTURE_CELL_MM,
                         tol=_FIXTURE_TOL_MM):
    """Drop unlabelled rows landing within ``tol`` mm of a labelled-marker
    cell anywhere in the (wider) fixture window — single-frame label
    dropouts of rig markers, not the ball (toss_record_miner precedent)."""
    if not fixture_cells:
        return rows
    out = []
    for (t, x, y, z) in rows:
        c = _cell(x, y, z, size)
        near = False
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if (c[0] + dx, c[1] + dy, c[2] + dz) in fixture_cells:
                        near = True
                        break
                if near:
                    break
            if near:
                break
        if not near:
            out.append((t, x, y, z))
    return out


def build_tracks(rows, max_speed_mms=_MAX_SPEED_MMPS,
                  max_gap_s=_MAX_GAP_S, gate_margin_mm=_GATE_MARGIN_MM):
    """Greedy nearest-neighbour association -> list of [(t,x,y,z,label), ...].

    ``rows`` entries are ``(t, x, y, z)`` or ``(t, x, y, z, label)``; a
    missing label defaults to ``''`` (unlabelled).
    """
    def _unpack(r):
        return r if len(r) == 5 else (r[0], r[1], r[2], r[3], '')

    rows = sorted(rows, key=lambda r: r[0])
    tracks = []   # dicts: last_t, last_pos (np.array), pts (list)
    for r in rows:
        t, x, y, z, label = _unpack(r)
        pos = np.array([x, y, z])
        best = None
        best_d = None
        for tr in tracks:
            dt = t - tr['last_t']
            if dt <= 0.0 or dt > max_gap_s:
                continue
            gate = max_speed_mms * dt + gate_margin_mm
            d = float(np.linalg.norm(pos - tr['last_pos']))
            if d <= gate and (best_d is None or d < best_d):
                best = tr
                best_d = d
        if best is None:
            tracks.append({'last_t': t, 'last_pos': pos,
                            'pts': [(t, x, y, z, label)]})
        else:
            best['pts'].append((t, x, y, z, label))
            best['last_t'] = t
            best['last_pos'] = pos
    return [tr['pts'] for tr in tracks]


_TRIM_RESID_TARGET_MM = 5.0   # probe (1b) fix: "residual stays < a few mm"
_TRIM_MIN_PTS = 6              # same floor as _MIN_TRACK_PTS -- guards a
                                # 6-unknown (p0,v0 x 3 axes) fit against
                                # shrinking to a near-tautological point count


def trim_to_free_flight(pts):
    """Shrink ``pts`` from BOTH ends down to its free-flight span.

    Probe run 1 (2026-09-13) found every ground-truth track had pre-release
    "resting" / in-hand samples glued onto its FRONT: ``build_tracks``'s
    greedy associator correctly follows one continuous physical marker
    straight through the throw stroke into free flight (no discontinuity in
    IDENTITY, only in DYNAMICS -- the hand accelerates the ball
    non-ballistically before release), so the untrimmed fit's window started
    ~190 ms before the physical release with RMS 470-590 mm. The same
    contamination can occur at the BACK (ball dwelling in the receiving
    hand after the catch).

    Iteratively fit the current ``[lo, hi]`` window, then drop whichever
    endpoint has the larger z-residual against that fit, until the fit's RMS
    is under ``_TRIM_RESID_TARGET_MM`` or the window reaches
    ``_TRIM_MIN_PTS`` points (whichever comes first). Symmetric on purpose:
    the contamination is not guaranteed to be front-only.

    -> ``(pts_trimmed, p0, v0, t_ref, rms_mm)``, or ``None`` if fewer than
    ``_TRIM_MIN_PTS`` samples are available at all.
    """
    pts = sorted(pts, key=lambda p: p[0])
    lo, hi = 0, len(pts) - 1
    best = None
    while hi - lo + 1 >= _TRIM_MIN_PTS:
        window = pts[lo:hi + 1]
        est = BallisticEstimator(G_VEC)
        for row in window:
            est.add(row[0], (row[1], row[2], row[3]))
        p0, v0 = est.estimate()
        t_ref = window[-1][0]
        resid = []
        for (t, _x, _y, z, _label) in window:
            dt = t - t_ref
            z_hat = p0[2] + v0[2] * dt + 0.5 * G_VEC[2] * dt * dt
            resid.append(z - z_hat)
        rms = math.sqrt(sum(r * r for r in resid) / len(resid))
        best = (window, p0, v0, t_ref, rms)
        if rms <= _TRIM_RESID_TARGET_MM:
            break
        if abs(resid[0]) >= abs(resid[-1]):
            lo += 1
        else:
            hi -= 1
    return best


def pick_ball_track(tracks):
    """-> (chosen_pts_or_None, n_plausible_candidates).

    "Plausible" = a genuine vertical excursion (>= _MIN_Z_RANGE_MM) with
    >= _MIN_TRACK_PTS samples -- big enough that only a thrown ball (not rig
    jitter or a stationary reflector) produces it. Among plausible tracks the
    longest (most points) wins.
    """
    plausible = []
    for pts in tracks:
        if len(pts) < _MIN_TRACK_PTS:
            continue
        zs = [p[3] for p in pts]
        if (max(zs) - min(zs)) >= _MIN_Z_RANGE_MM:
            plausible.append(pts)
    if not plausible:
        return None, 0
    plausible.sort(key=len, reverse=True)
    return plausible, len(plausible)


#: Loose on purpose: guards ONLY against a track that merges TWO different
#: physical objects (e.g. a rocking platform rim marker gated onto the ball's
#: track) -- trim_to_free_flight() now removes the pre-release/post-catch
#: contamination that used to run this guard up to ~500 mm (probe run 1,
#: 2026-09-13); a genuine single-ball trimmed fit should land single-digit mm
#: (target _TRIM_RESID_TARGET_MM) and this stays as a generous backstop for
#: whatever trim_to_free_flight cannot fully separate (e.g. _TRIM_MIN_PTS
#: reached before the residual target).
_FIT_RMS_OK_MM = 5000.0


def fit_best_plausible(plausible):
    """Try each plausible track (longest first), trim it to free flight
    (:func:`trim_to_free_flight`), and return the first trimmed fit that is
    actually ballistic (rms <= _FIT_RMS_OK_MM) -- guards against a spurious
    merged track (two objects gated together) passing the z-range test with
    a garbage fit. -> None if nothing plausible fits well.
    """
    for pts in plausible:
        fit = trim_to_free_flight(pts)
        if fit is not None and fit[-1] <= _FIT_RMS_OK_MM:
            return fit
    return None


def solve_z_crossing(p0z, v0z, g_z, target_z):
    """-> dt (relative to the fit's t_ref) of the DESCENDING crossing of
    ``target_z``, or None if the parabola never reaches it."""
    a = 0.5 * g_z
    b = v0z
    c = p0z - target_z
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    roots = [(-b + sq) / (2.0 * a), (-b - sq) / (2.0 * a)]
    descending = [dt for dt in roots if (v0z + g_z * dt) < 0.0]
    if descending:
        return max(descending)
    return max(roots)


# ─────────────────────────── per-bag mining ────────────────────────────────

def mine_bag(bag_dir, verbose=True):
    bag_dir = parse_bag_name(bag_dir)
    bag_name = os.path.basename(bag_dir.rstrip('/'))

    announcements = []       # (t_bag, thrower, target, throw_time, land_time)
    toss_records = []        # (t_bag, dict)
    mocap_unlabelled = []    # (t, x, y, z)
    mocap_labelled = []      # (t, x, y, z)
    balls_by_id = {}         # id -> list of (t, status, tracking, source, pos, vel, lpos, lvel, tland)
    hand_rows = []           # (t, ball_held_raw, ball_held_valid)
    ballstate_header_vs_log = []

    topics = ('/throw_announcements', '/toss/record', '/mocap_data',
              '/balls', '/hand_telemetry')
    for topic, t_bag, msg in _iter(bag_dir, topics):
        if topic == '/throw_announcements':
            if str(msg.thrower_name) != str(msg.target_id):
                continue   # not a self-toss
            throw_time = (msg.throw_time.sec
                           + msg.throw_time.nanosec * 1e-9)
            land_time = (msg.landing_time.sec
                          + msg.landing_time.nanosec * 1e-9)
            announcements.append((t_bag, str(msg.thrower_name), throw_time,
                                   land_time))
        elif topic == '/toss/record':
            import json
            try:
                d = json.loads(msg.data)
            except ValueError:
                continue
            if d.get('schema') != 'toss_record/1':
                continue
            toss_records.append((t_bag, d))
        elif topic == '/mocap_data':
            for mk in msg.markers:
                x, y, z = (float(mk.position.x), float(mk.position.y),
                           float(mk.position.z))
                if mk.label:
                    mocap_labelled.append((t_bag, x, y, z, str(mk.label)))
                else:
                    mocap_unlabelled.append((t_bag, x, y, z))
        elif topic == '/balls':
            for b in msg.balls:
                hdr = b.header.stamp.sec + b.header.stamp.nanosec * 1e-9
                ballstate_header_vs_log.append(t_bag - hdr)
                balls_by_id.setdefault(int(b.id), []).append((
                    t_bag, int(b.status), int(b.tracking), str(b.source),
                    np.array([b.position.x, b.position.y, b.position.z]),
                    np.array([b.velocity.x, b.velocity.y, b.velocity.z]),
                    np.array([b.landing_position.x, b.landing_position.y,
                              b.landing_position.z]),
                    np.array([b.landing_velocity.x, b.landing_velocity.y,
                              b.landing_velocity.z]),
                    b.time_at_land.sec + b.time_at_land.nanosec * 1e-9,
                ))
        elif topic == '/hand_telemetry':
            hand_rows.append((t_bag, bool(msg.ball_held_raw),
                               bool(msg.ball_held_valid)))

    mocap_unlabelled.sort()
    mocap_labelled.sort()
    for k in balls_by_id:
        balls_by_id[k].sort(key=lambda r: r[0])
    hand_rows.sort()
    announcements.sort()

    ros_bag_offset = (float(np.median(ballstate_header_vs_log))
                       if ballstate_header_vs_log else None)

    # ---- Build the flight list -------------------------------------------
    flights = []
    if announcements:
        for i, (t_bag, thrower, throw_time, land_time) in enumerate(announcements):
            fl = Flight(i + 1, 'announcement')
            fl.t_release_cmd = throw_time
            fl.t_land_pred = land_time
            flights.append(fl)
    else:
        # Degraded case (e.g. 2026-09-06_23-55-35): no announcements at all.
        # Fall back to one flight per toss_record, searched around the
        # record's own bag time with a wide window; t_release_cmd stays None
        # (Q4's commanded-vs-physical offset is then unanswerable, reported
        # as such rather than guessed).
        for i, (t_bag, d) in enumerate(toss_records):
            fl = Flight(i + 1, 'toss_record')
            fl.t_release_cmd = None
            fl.t_land_pred = t_bag + 3.5
            fl.record = d
            fl.toss_uid = d.get('toss_uid')
            flights.append(fl)

    # Join toss_record rows to announcement-derived flights on nearest
    # release-adjacent bag time within a loose 2 s tolerance (context only:
    # goal_*, tilt map, etc. -- not required for the ground-truth fit).
    if announcements and toss_records:
        rec_times = [t for (t, _d) in toss_records]
        for fl in flights:
            if fl.t_release_cmd is None:
                continue
            diffs = [abs(t - fl.t_release_cmd) for t in rec_times]
            j = int(np.argmin(diffs)) if diffs else None
            if j is not None and diffs[j] < 2.0:
                fl.record = toss_records[j][1]
                fl.toss_uid = fl.record.get('toss_uid')

    # ---- Per-flight ground truth + candidates ------------------------------
    for fl in flights:
        lo = (fl.t_release_cmd - 0.20) if fl.t_release_cmd is not None \
            else (fl.t_land_pred - 2.0)
        hi = fl.t_land_pred + 0.60

        fix_lo, fix_hi = lo - 0.5, hi + 0.5
        fixture_rows = [r for r in mocap_labelled if fix_lo <= r[0] <= fix_hi]
        fixture_cells = build_fixture_cells(fixture_rows)

        window_rows = [r for r in mocap_unlabelled if lo <= r[0] <= hi]
        window_rows = drop_fixture_echoes(window_rows, fixture_cells)
        tracks = build_tracks(window_rows)
        plausible, n_plaus = pick_ball_track(tracks)
        fl.candidate_tracks_seen = n_plaus
        fit = fit_best_plausible(plausible) if plausible else None
        fl.gt_source = 'unlabelled' if fit is not None else None

        if fit is None:
            # FALLBACK: the ball's marker can get folded into a stale rigid
            # body's LABELLED set (2026-09-06 "Catching Cone" binding, plan
            # § 2.7 / Q3) rather than showing up unlabelled at all. Retry
            # against every marker in the window, labelled or not.
            fallback_rows = ([(r[0], r[1], r[2], r[3], '')
                               for r in mocap_unlabelled if lo <= r[0] <= hi]
                              + [r for r in mocap_labelled if lo <= r[0] <= hi])
            fb_tracks = build_tracks(fallback_rows)
            fb_plausible, fb_n = pick_ball_track(fb_tracks)
            fb_fit = fit_best_plausible(fb_plausible) if fb_plausible else None
            if fb_fit is not None:
                fit = fb_fit
                fl.candidate_tracks_seen = fb_n
                fl.gt_source = 'labelled-fallback'
                fl.gt_labels = sorted({lab for (_t, _x, _y, _z, lab)
                                        in fit[0] if lab})

        if fit is None:
            continue   # ground truth unavailable for this flight

        chosen, p0, v0, t_ref, rms = fit
        fl.track_pts = chosen
        fl.t_release_phys = chosen[0][0]
        fl.p0, fl.v0 = p0, v0
        fl.t_ref = t_ref
        fl.gt_fit_rms_mm = rms

        dt830 = solve_z_crossing(p0[2], v0[2], G_VEC[2], CATCH_CUP_Z_MM)
        dt809 = solve_z_crossing(p0[2], v0[2], G_VEC[2], TRACKER_LANDING_Z_MM)
        if dt830 is not None:
            fl.t_cross_830 = fl.t_ref + dt830
            fl.gt_xy_830 = (p0[0] + v0[0] * dt830, p0[1] + v0[1] * dt830)
        if dt809 is not None:
            fl.t_cross_809 = fl.t_ref + dt809
            fl.gt_xy_809 = (p0[0] + v0[0] * dt809, p0[1] + v0[1] * dt809)

        # ---- /balls candidates ---------------------------------------------
        # Identify the id: the id whose messages carry source == thrower and
        # whose stream is temporally active across [lo, hi].
        thrower = getattr(fl, '_thrower', None)
        best_id, best_overlap = None, 0
        for bid, rows in balls_by_id.items():
            in_win = [r for r in rows if lo - 0.1 <= r[0] <= hi + 0.1]
            if not in_win:
                continue
            src = in_win[0][3]
            if announcements and src != announcements[0][1]:
                # source should equal thrower_name for every self-toss id
                # (all self-tosses in a bag share one thrower name)
                pass
            if len(in_win) > best_overlap:
                best_overlap = len(in_win)
                best_id = bid
        fl.ball_id = best_id
        if best_id is None:
            fl.no_track = True
            continue
        rows = [r for r in balls_by_id[best_id] if lo - 0.1 <= r[0] <= hi + 0.1]
        confirmed = [r for r in rows if r[2] == 1]
        fl.no_track = (len(confirmed) == 0)
        if confirmed:
            fl.late_start_s = confirmed[0][0] - fl.t_release_phys

        # distractor census: other ids active in-window with tracking==1
        near = 0
        for bid, brows in balls_by_id.items():
            if bid == best_id:
                continue
            for r in brows:
                if lo <= r[0] <= hi and r[2] == 1:
                    near += 1
                    break
        fl.n_distinct_ids_near = near

        # "Observed" predicate (probe 1b brief): one CONFIRMED id (no
        # competing distractor id in-window), >= 5 tracker updates after
        # release, and at least one update while the ball is still above
        # 830 + 50 mm (i.e. candidate (e) is scoreable). A flight failing
        # this is a "blind flight" for the learner.
        n_confirmed_after_release = sum(
            1 for r in confirmed if r[0] > fl.t_release_phys)
        has_high_update = any(
            r[4][2] > (CATCH_CUP_Z_MM + 50.0) for r in confirmed)
        fl.observed_predicate = (
            not fl.no_track
            and fl.n_distinct_ids_near == 0
            and n_confirmed_after_release >= 5
            and has_high_update
        )

        def _err(xy_pred, xy_gt):
            return math.hypot(xy_pred[0] - xy_gt[0], xy_pred[1] - xy_gt[1])

        # (a) last prediction before crossing 809 / before being caught
        cutoff_a = fl.t_cross_809 if fl.t_cross_809 is not None else hi
        cand_a = [r for r in rows if r[0] <= cutoff_a]
        if cand_a and fl.gt_xy_809 is not None:
            r = cand_a[-1]
            err_mm = _err((r[6][0], r[6][1]), fl.gt_xy_809)
            err_ms = (r[8] - fl.t_cross_809) * 1e3
            fl.candidates['a'] = dict(t=r[0], xy=(r[6][0], r[6][1]),
                                       err_mm=err_mm, err_ms=err_ms)

        # (b) prediction at t_cross_830 - lead
        if fl.t_cross_830 is not None:
            target_t = fl.t_cross_830 - CATCH_FREEZE_LEAD_S
            before = [r for r in rows if r[0] <= target_t]
            if before and fl.gt_xy_830 is not None:
                r = before[-1]
                err_mm = _err((r[6][0], r[6][1]), fl.gt_xy_830)
                err_ms = (r[8] - fl.t_cross_830) * 1e3
                fl.candidates['b'] = dict(t=r[0], xy=(r[6][0], r[6][1]),
                                           err_mm=err_mm, err_ms=err_ms)

        # (c) prediction ~150 ms after physical release
        target_t = fl.t_release_phys + RELEASE_SAMPLE_LEAD_S
        if rows:
            r = min(rows, key=lambda rr: abs(rr[0] - target_t))
            if abs(r[0] - target_t) < 0.10 and fl.gt_xy_830 is not None:
                err_mm = _err((r[6][0], r[6][1]), fl.gt_xy_830)
                err_ms = (r[8] - fl.t_cross_830) * 1e3 if fl.t_cross_830 else None
                fl.candidates['c'] = dict(t=r[0], xy=(r[6][0], r[6][1]),
                                           err_mm=err_mm, err_ms=err_ms)

        def _project(r):
            """Candidates (e)/(e'): ``r``'s (position, velocity) projected
            forward to z = CATCH_CUP_Z_MM via arrival_state_at_z (the same
            boundary-condition call sim/skills_gate.py's tracker uses).
            -> dict(t, xy, err_mm, err_ms) or None if the ball's fitted
            trajectory from this state never reaches 830 mm in the future
            (touchdown_time raises ValueError)."""
            pos, vel = r[4], r[5]
            try:
                lp, _lv, ttl = arrival_state_at_z(
                    pos, vel, CATCH_CUP_Z_MM, descending=True)
            except ValueError:
                return None
            err_mm = _err((lp[0], lp[1]), fl.gt_xy_830)
            # r[0] + ttl == header_stamp + ttl converted back to log_time via
            # THIS row's own (log_time - header_stamp) offset -- see module
            # docstring's CLOCK DOMAIN note.
            pred_abs_t = r[0] + ttl
            err_ms = (pred_abs_t - fl.t_cross_830) * 1e3
            return dict(t=r[0], xy=(lp[0], lp[1]), err_mm=err_mm, err_ms=err_ms)

        # (e) LAST /balls update while the ball is still provably above the
        # 830 mm plane (+50 mm margin) -- guarantees a genuine forward
        # projection (candidate (d), retired 2026-09-13, projected from a
        # state already at/under 809.08 mm, i.e. already past 830 mm).
        if fl.gt_xy_830 is not None:
            high_rows = [r for r in rows
                         if r[4][2] > (CATCH_CUP_Z_MM + 50.0)]
            if high_rows:
                cand = _project(high_rows[-1])
                if cand is not None:
                    fl.candidates['e'] = cand

        # (e') the same projection, from the update nearest the executor's
        # own CATCH-freeze instant (t_cross_830 - executor.CATCH_FREEZE_S)
        # rather than "last high update" -- the state the executor would
        # actually still be re-aiming from just before it freezes.
        if fl.t_cross_830 is not None and fl.gt_xy_830 is not None and rows:
            target_t = fl.t_cross_830 - CATCH_FREEZE_S
            r = min(rows, key=lambda rr: abs(rr[0] - target_t))
            if abs(r[0] - target_t) < 0.10:
                cand = _project(r)
                if cand is not None:
                    fl.candidates['e2'] = cand

        # ---- possession -----------------------------------------------------
        if fl.t_cross_830 is not None:
            seated = [r for r in hand_rows
                      if r[0] >= fl.t_cross_830 - 0.05
                      and r[0] <= fl.t_cross_830 + 1.0
                      and r[1] and r[2]]
            if seated:
                fl.t_seated = seated[0][0]
                fl.caught = True
            else:
                fl.caught = False

    return bag_name, flights, ros_bag_offset


# ─────────────────────────── output ────────────────────────────────────────

def write_csv(bag_name, flights, out_dir):
    import csv
    path = os.path.join(out_dir, f'throw_outcome_{bag_name}.csv')
    fields = [
        'flight', 'source', 't_release_cmd', 't_release_phys',
        'dispatch_offset_ms', 'track_n', 'gt_source', 'gt_fit_rms_mm',
        'gt_labels', 't_cross_830', 't_cross_809',
        'gt_x_830', 'gt_y_830', 'gt_x_809', 'gt_y_809',
        'flight_s_cmd_to_830', 'candidate_tracks_seen', 'ball_id',
        'no_track', 'late_start_s', 'n_distinct_ids_near',
        'observed_predicate',
        'a_err_mm', 'a_err_ms', 'b_err_mm', 'b_err_ms',
        'c_err_mm', 'c_err_ms', 'e_err_mm', 'e_err_ms',
        'eprime_err_mm', 'eprime_err_ms',
        't_seated', 'seated_after_830_ms', 'caught',
    ]
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(fields)
        for fl in flights:
            disp_off = None
            if fl.t_release_cmd is not None and fl.t_release_phys is not None:
                disp_off = (fl.t_release_phys - fl.t_release_cmd) * 1e3
            flight_s = None
            if fl.t_release_cmd is not None and fl.t_cross_830 is not None:
                flight_s = fl.t_cross_830 - fl.t_release_cmd
            seated_ms = None
            if fl.t_seated is not None and fl.t_cross_830 is not None:
                seated_ms = (fl.t_seated - fl.t_cross_830) * 1e3
            row = [
                fl.idx, fl.source, fl.t_release_cmd, fl.t_release_phys,
                disp_off, (len(fl.track_pts) if fl.track_pts else 0),
                fl.gt_source, fl.gt_fit_rms_mm,
                ('|'.join(fl.gt_labels) if fl.gt_labels else ''),
                fl.t_cross_830, fl.t_cross_809,
                fl.gt_xy_830[0] if fl.gt_xy_830 else None,
                fl.gt_xy_830[1] if fl.gt_xy_830 else None,
                fl.gt_xy_809[0] if fl.gt_xy_809 else None,
                fl.gt_xy_809[1] if fl.gt_xy_809 else None,
                flight_s, fl.candidate_tracks_seen, fl.ball_id, fl.no_track,
                fl.late_start_s, fl.n_distinct_ids_near,
                fl.observed_predicate,
            ]
            for key in ('a', 'b', 'c', 'e', 'e2'):
                c = fl.candidates.get(key)
                row.append(c['err_mm'] if c else None)
                row.append(c['err_ms'] if c else None)
            row += [fl.t_seated, seated_ms, fl.caught]
            w.writerow(row)
    return path


def _dist(values):
    v = sorted(x for x in values if x is not None)
    if not v:
        return None
    n = len(v)
    med = v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])
    p90 = v[min(n - 1, int(math.ceil(0.9 * n)) - 1)]
    return dict(n=n, median=med, p90=p90, max=v[-1])


def write_summary(all_results, out_dir):
    path = os.path.join(out_dir, 'throw_outcome_summary.md')
    lines = ['# throw_outcome_bag_probe summary', '']
    lines.append(f'Generated {datetime.now().isoformat(timespec="seconds")}')
    lines.append('')
    for bag_name, flights, ros_bag_offset in all_results:
        lines.append(f'## {bag_name}')
        lines.append(f'- ros_bag_offset_probe_s (median BallState '
                      f'log_time - header.stamp): {ros_bag_offset}')
        n = len(flights)
        n_gt = sum(1 for f in flights if f.t_cross_830 is not None)
        n_no_track = sum(1 for f in flights if f.no_track)
        n_caught = sum(1 for f in flights if f.caught)
        n_observed = sum(1 for f in flights if f.observed_predicate)
        n_blind = sum(1 for f in flights if not f.observed_predicate)
        gt_rms = _dist([f.gt_fit_rms_mm for f in flights
                        if f.gt_fit_rms_mm is not None])
        disp_offs = _dist([
            (f.t_release_phys - f.t_release_cmd) * 1e3 for f in flights
            if f.t_release_cmd is not None and f.t_release_phys is not None])
        lines.append(f'- flights: {n}, ground truth found: {n_gt}, '
                      f'no /balls track: {n_no_track}, caught (SEATED): {n_caught}')
        lines.append(f'  - ground-truth fit RMS (mm) after free-flight '
                      f'trimming: {gt_rms}')
        lines.append(f'  - release lag (physical departure - commanded '
                      f'throw_time, ms): {disp_offs}')
        lines.append(f'  - observed (predicate: 1 CONFIRMED id, >=5 updates '
                      f'after release, an update above 830+50mm): '
                      f'{n_observed}/{n}; blind: {n_blind}/{n}')
        excluded = [f.idx for f in flights if f.t_cross_830 is None]
        if excluded:
            lines.append(f'  - EXCLUDED (no ground truth / not a scoreable '
                          f'throw): flights {excluded}')
        for key, label in (('a', 'last-before-cross-809'),
                           ('b', 'freeze-100ms'), ('c', 'release+150ms'),
                           ('e', 'last-above-880-reprojected-830'),
                           ('e2', "eprime-CATCH_FREEZE_S-reprojected-830")):
            errs_mm = [f.candidates[key]['err_mm'] for f in flights
                       if key in f.candidates]
            errs_ms = [f.candidates[key]['err_ms'] for f in flights
                       if key in f.candidates and f.candidates[key]['err_ms']
                       is not None]
            lines.append(f'  - candidate {key} ({label}) '
                         f'err_mm: {_dist(errs_mm)}  err_ms: {_dist(errs_ms)}')
        lines.append('')
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', action='append', required=True,
                     help='bag name under ~/Desktop/rosbags, or a full path; '
                          'repeatable')
    args = ap.parse_args(argv)

    out_dir = os.path.join(_REPO, 'temp', 'probes')
    os.makedirs(out_dir, exist_ok=True)

    all_results = []
    for spec in args.bag:
        bag_name, flights, ros_bag_offset = mine_bag(spec)
        path = write_csv(bag_name, flights, out_dir)
        print(f'{bag_name}: {len(flights)} flight(s) -> {path}')
        all_results.append((bag_name, flights, ros_bag_offset))

    summary_path = write_summary(all_results, out_dir)
    print(f'summary -> {summary_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
