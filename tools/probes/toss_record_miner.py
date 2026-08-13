#!/usr/bin/env python3
"""Mine a rosbag into the per-toss record corpus — the MEASUREMENT half.

WHAT IT DOES
------------
Reads one sitting's ``.mcap`` and emits one ``toss_record/1`` row per self-toss:
the sensor block and its label, the mocap arrival offset, the provenance the bag
witnesses, and — when the coordinator was publishing them — the ``/toss/record``
DECLARATIONS joined on ``announce_throw_time_ros``. (The PLANT block is deferred;
see ``mine_bag``.)

Since 2026-08-12 it also mines the **throw critical point** for
``plans/active/critical-point-ilc.md`` Phase 0a/0c: the ball's arrival velocity
at the catch-plane crossing and its direction/speed error against the nominal,
the flight-time error, the **release-state backcast** (measured release
position, velocity and instant from the ascending branch), and the exact
RELEASE-vs-FLIGHT split of the landing error. :func:`mine_arc` is the definition
point for all of it; the ballistics it uses are the production ones.

Since 2026-08-13 every **LATERAL** velocity in that block comes from a WHOLE-ARC
fit rather than a per-branch one, and a ``coverage_asym_s`` field gates it. That
is the resolution of the ILC plan's entry condition E-1 — the mocap tracks the
centroid of the visible retroreflective cap, which carries a height-locked
position bias, and a per-branch lateral slope reads that bias as ±100 mm/s of
velocity on exactly the aim channels. The parity argument and the numbers are
written out above :data:`ASCENT_REF_LEAD_S`; the diagnostic that measured them is
``tools/probes/mocap_parity_bias.py``.

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

import numpy as np

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
# The PRODUCTION ballistics and the PRODUCTION release geometry — imported, never
# re-derived (critical-point-ilc plan, design constraint 1: "one forward chain,
# differentiated numerically, never a symbolic twin"). Everything this file adds
# on top is *estimation* (least squares over noisy markers), which has no
# production home; every propagation, crossing solve and nominal arrival velocity
# goes through `ballistics_bc`.
from jugglebot.motion.trajectory import ballistics_bc            # noqa: E402
from jugglebot.motion.trajectory.toss_release import (           # noqa: E402
    HAND_THROW_OFFSET_MM,
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
#: The miner REFUSES a catch plane this far from the plane it fitted at — whether
#: it comes from a DECLARATION (`enforce_declared_planes`, on the joined rows) or
#: from the ANNOUNCEMENT alone (`mine_bag`, on every row, declaration or not).
PLANE_MISMATCH_TOL_MM = 5.0

# ── The release-fit admission gate (ILC Phase 0a/0c) ──────────────────────────
# `usable_for_speed_fit` selects on the LANDING fit — `land_xy_global_mm` and its
# RMS — which is the wrong gate for a statistic about the RELEASE velocity: the
# two are different fits over different branches, and the reference bag proves
# they dissociate (rows 28/29 carry a clean 58/67-sample arc pair and NO landing
# fit at all, while rows 1-3 carry a good landing fit and are excluded for a
# 4-5 mm landing RMS that says nothing about the backcast). Before this flag the
# "clean fits" population behind the headline release-speed number was not
# expressible from the corpus — it existed only as a filter somebody applied by
# hand in a report, which is how a headline becomes unreproducible.

#: Fewest rows a branch may have and still admit its release-velocity estimate.
#: 5x :data:`MIN_ARC_SAMPLES` — the floor below which a fit is refused outright —
#: and it sits in the middle of a 5x gap in the measured corpus, not at its edge:
#: across the three mocap bags every admitted branch carries 49-80 rows and every
#: refused one carries 4-10. A threshold anywhere in 11..48 selects the same
#: population, so this is not a boundary fitted to the data.
RELEASE_FIT_MIN_SAMPLES = 20
#: Largest 1-sigma standard error on the fitted release velocity that still
#: admits the row, mm/s. The effect being MEASURED on this corpus is a release
#: speed error of ~+480 mm/s, so this bounds the estimator's own uncertainty an
#: order below the signal. Measured: admitted rows run se 5-21 mm/s, refused ones
#: 101-1610 — the same clean separation, and for the same reason (a 60 ms branch
#: cannot determine a velocity, see :func:`fit_ballistic`'s ``se_mms``).
RELEASE_FIT_MAX_SE_MMS = 50.0

BAND_MM = 300.0
LATERAL_MM = 300.0
MIN_FIT_SAMPLES = 5
PRE_S, POST_S = 1.2, 1.2

# ── Arc mining (ILC Phase 0a/0c) ──────────────────────────────────────────────
#: ONE gravity, imported from the production ballistics. The repo has exactly one
#: value — 9806.0 mm/s², in `motion/trajectory/ballistics_bc` AND in
#: `tracking/ballistics` (``GRAVITY_MMPS2``), both of which say "must match
#: hw.GRAVITY_MPS2 * 1000". The "tracker-side 9810" that `toss_release`'s module
#: header and `toss_sequencer`'s comment warn about is not a live PRODUCTION
#: constant anywhere — which is the claim that matters here and the only one this
#: file's fits depend on. It is NOT gone from the tree: `ball_arrival_offset`'s
#: synthetic-track generator uses it, two stale test-local constants still carry
#: it (`tests/ros/test_reload_integration.py`, `tests/ros/test_toss_integration.py`
#: — whose "the 9810 the tracking stack uses" comment is now false), and `attic/`
#: is full of it. (The census here used to say "the only 9810 left in the tree",
#: which was over-claimed: a reader who greps finds four live hits and stops
#: trusting the rest of the paragraph.) Using the same constant on both sides of
#: every difference below is what makes a gravity-convention error cancel
#: exactly instead of masquerading as a plant residual.
GRAVITY_MMS2 = ballistics_bc.GRAVITY_MMS2

# WHY THE VELOCITY FITS SPAN A WHOLE BRANCH AND NOT A BAND NEAR THE PLANE.
# ``/mocap_data`` carries NO HEADER, so its only clock is the bag's ``log_time``,
# and that clock is not locally faithful. Measured on 2026-08-12_19-02-52 toss 8
# (144 tracked samples over a full arc):
#
#   * x(t) and y(t) fit straight lines to 2.2 / 8.0 mm RMS — the geometry is
#     fine;
#   * z(t) against the production gravity leaves 34 mm RMS, and a free-g fit
#     reads 9131 mm/s^2 instead of 9806. Both are the same defect seen through a
#     fast axis: at 4.9 m/s a 7 ms timestamp error IS 34 mm, while on the
#     24 mm/s lateral axis it is 0.2 mm.
#   * rolling 8-sample (30-90 ms) windows — the band-shaped estimator this file
#     used first — return vz scattered by +-800 mm/s about the truth, i.e. +-17 %
#     at the plane. That estimator is not noisy, it is unusable.
#   * the SAME data fitted over a whole branch (73 ascending / 72 descending
#     samples) returns +4900.0 / -4841.8 mm/s at the two planes against
#     +4924.3 / -4910.9 derived independently from the apex height — agreement to
#     0.5 % and 1.4 %.
#
# So each branch is fitted whole. The branch SPLIT is kept FOR THE VERTICAL
# CHANNELS: it is what makes release-vs-flight a real discriminator there rather
# than one fit imposed on both ends, and it is what stops drag cancelling itself
# — a no-drag fit over a full arc hides the asymmetry, a per-branch pair exposes
# it as a release-vs-arrival speed difference.
#
# THE LATERAL CHANNELS DO NOT USE THE SPLIT (E-1, resolved 2026-08-13).
# ``plans/active/critical-point-ilc.md`` § Phase 1, entry condition E-1: the
# tracked point is the centroid of the visible retroreflective cap, biased by a
# height-locked b(z) of ~20 mm — CONFIRMED by parity decomposition (probe:
# ``tools/probes/mocap_parity_bias.py``). The parity argument, which is the whole
# of why this file changed:
#
#   * height is an EVEN function of time about a ballistic apex, so a
#     height-locked position bias enters the lateral track as an EVEN function of
#     tau = t - t_apex, on top of the true straight line;
#   * a PER-BRANCH line fit therefore reads ``v_true +- <db/dz . |vz|>`` — equal
#     and opposite on the two branches. Measured: v_y +94.6 ascending against
#     -13.5 descending, a 104.4 mm/s branch delta that repeats across 19 arcs,
#     three cup positions and two sittings at 1.45 mm cross-toss sd;
#   * a WHOLE-ARC fit is bias-immune by construction: the least-squares slope
#     picks up ``cov(tau, b)/var(tau)``, and ``cov(tau, even(tau)) = 0`` exactly
#     when the sample coverage is symmetric about the apex. What survives is the
#     COVERAGE ASYMMETRY, measured at 2.1 mm/s median / 7.0 mm/s worst against
#     the 104.4 mm/s it replaces — see :data:`COVERAGE_ASYM_MAX_S`.
#
# What the per-branch estimator cost, stated as the number that forced the
# change: the descending branch's offset from the whole-arc truth is
# (-53.3, +16.2) mm/s, i.e. |dv| ~ 55.7 mm/s over a ~5.1 m/s arrival = 10.9 mrad
# of phantom arrival-direction error per toss — about 44 mm of phantom aim error
# through the (4h + dz) landing gain, which is 4007 mm/rad at the corpus goal
# (`toss_trim.aim_landing_jacobian(T = 0.9032 s, z = 170 mm)`, so 0.0109 rad x
# 4007 = 43.7 mm). That is THREE times the aim channel's own scatter (the
# corpus's per-axis landing sd, 15.7 / 13.6 mm, pooled to 14.7 mm), and it is
# REPEATABLE, so it does not average away.
#
# Height-restricting the fit window does NOT work and was tried: the bias
# gradient is essentially constant over the whole arc, so a narrow window trades
# the same bias for a worse conditioned fit.
#
# STANDING CAVEAT, and it is not closed by any of this: only the bias GRADIENT is
# measured. The absolute offset (bounded by ~a ball radius) is not, so any
# mocap-closed aim loop converges to the measurement's cup rather than the
# world's. Pre-existing across the whole mocap aim stack; closure is a ~20-minute
# no-robot capture of the taped ball fixtured with conventional point markers.

#: How far before the first ascending sample the backcast is anchored. Purely a
#: parameterisation of the SAME fitted arc — the state is propagated there with
#: `ballistics_bc.position_at`/`velocity_at` — and it exists because
#: `ballistics_bc.touchdown_time` refuses a crossing in the past (a production
#: guard worth keeping), so the reference instant has to sit below the release
#: plane for the ascending root to be "in the future".
ASCENT_REF_LEAD_S = 0.5

#: QTM already TOLD us which points are rig. Any unlabelled row landing in a cell
#: a LABELLED marker has occupied anywhere in the capture is dropped. This is
#: what identifies the single-frame label dropouts that otherwise poison the
#: fits: measured on 2026-08-12_19-02-52, the platform's own rim markers lose
#: their labels for single frames and reappear as UNLABELLED markers at
#: z ≈ 1056-1068 mm — inside both the lateral gate and the descending fit band,
#: within ~1 mm of the labelled position they just lost. Left in, they scored
#: toss 8 of that bag with a 44 mm fit residual and a 12° arrival-direction
#: error, all of it contamination. Cell size, with the 26 neighbours also
#: checked, puts the effective tolerance at 6-10 mm: two orders below the >70 mm
#: the ball ever passes from a platform rim marker.
#:
#: TWO MOTION-BASED FILTERS WERE TRIED FIRST AND BOTH ARE WRONG HERE, recorded so
#: nobody re-adds them. (1) *"Did anything move near this row recently?"* measures
#: displacement between DIFFERENT markers, so a stray sitting beside a 4.4 m/s
#: ball reads as moving. (2) *"Does this row have a twin ≥0.15 s away and within
#: 8 mm?"* is worse: it deletes the BALL. A near-vertical toss passes each height
#: twice, seconds apart and — because the lateral drift is ~24 mm/s — millimetres
#: apart, so the test fires on precisely the throws this phase exists to measure.
#: It silently removed the whole ascending branch of 4 of the 5 tracked tosses.
FIXTURE_CELL_MM = 6.0

#: Trimmed-fit floor: a row is only rejected as an outlier if it is BOTH beyond
#: 3x the MEDIAN residual AND beyond this absolute distance. Median, not RMS: one
#: 200 mm stray inflates the RMS enough to protect itself, which is exactly how
#: the first version of this fit kept the platform markers it was written to
#: reject. Without the absolute floor a clean 4-sample arc rejects its own worst
#: point every pass.
TRIM_FLOOR_MM = 25.0
#: Fewest rows that may produce a velocity. Two rows determine a line exactly and
#: report zero residual, which reads as "perfect" — the same trap `fit_sparse`
#: exists for on the position side.
MIN_ARC_SAMPLES = 4

#: Largest ``|coverage_asym_s|`` — the mean sample time's offset from the fitted
#: apex, seconds — that still admits a row's LATERAL quantities (E-1, 2026-08-13).
#: It is the one term the whole-arc estimator does not kill: ``cov(tau, b)`` is
#: exactly zero for coverage symmetric about the apex, so what leaks is the
#: asymmetry.
#:
#: Sized on the measured corpus, not chosen — and the two populations must be
#: named, because they answer different questions and their statistics differ by
#: an order of magnitude (re-mine of 2026-08-13, the three mocap bags):
#:
#: * among the **19 rows the release-fit gate admits** — the population any ILC
#:   fit actually reads — ``|coverage_asym_s|`` runs 0.0001-0.0732 s, median
#:   **0.0148 s**, and this gate refuses **0 of them**;
#: * across **all 32 mined arcs**, median **0.052 s**, worst **0.600 s**, and
#:   this gate refuses **12** — every one a half-seen arc that
#:   ``usable_for_release_fit`` already refuses on its own.
#:
#: So 0.1 s does NOT sit above the worst arc the corpus contains: the worst is
#: 6x ABOVE the gate, and 12 of 32 arcs are refused by it. That is the gate
#: working as intended rather than a hole in it — it is a GROSS-TRUNCATION guard
#: (one branch cut by an occlusion or a short window), not a filter fitted to the
#: data, and the evidence for "not fitted to the data" is that it costs the
#: release-fit population nothing. On a ~1.0 s arc it refuses a coverage
#: imbalance past ~10 %.
#:
#: HONEST LIMIT, and this is the DEMOTION the E-1 close-out recorded rather than
#: a caveat added afterwards: the plan's one-line phrasing ("residual leak =
#: coverage asymmetry") reads tighter than the measurement supports. This is the
#: FIRST MOMENT of the coverage, and a zero first moment does not prove a
#: symmetric distribution. One arc (2026-08-12_17-45-44 #0 — the PARITY PROBE's
#: arc index, not a miner row index) has ``coverage_asym_s`` = 0.0003 s and still
#: leaks 6.0 mm/s, because its coverage is asymmetric in SHAPE. So this field is
#: a gross-truncation guard and NOT a leak predictor; the residual ~7 mm/s
#: (~1.4 mrad at a 5 m/s arrival) is a standing uncertainty on the lateral
#: velocity channels, an order under the artefact it replaces and reported here
#: rather than assumed away.
#:
#: The measured leak the whole-arc estimator does clear: 2.1 mm/s median /
#: 7.0 mm/s worst, against the 104.4 mm/s per-branch artefact it replaces.
COVERAGE_ASYM_MAX_S = 0.1

#: The descending branch ends when the ball comes back UP by this much. Sized
#: above the measured z noise (25-34 mm RMS on a whole branch — see
#: :data:`ASCENT_REF_LEAD_S`'s preamble) so ordinary jitter never truncates a
#: good descent, and far below a real bounce-out (the reference sitting's ball 11
#: reappeared 345 mm above its free-fall prediction).
BOUNCE_TOL_MM = 40.0


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


def hand_sample_time(stamp_s: float, log_time_s: float,
                     log_minus_stamp_s: float) -> float:
    """THE clock rule for a ``/hand_telemetry`` sample, in one place.

    ROS-stamped where the publisher gave us a stamp, else crossed from the bag
    clock by the median ``log_time - stamp`` offset — never silently zero, which
    would put the whole hand stream 56 years away from the announcements.

    A function rather than two inline copies because ``hand_contact_softness``
    makes its own second decode pass over the same topic for the drive fields,
    and its samples must land on the SAME clock as the sensor bits they are read
    against. A restated rule is a rule that drifts, and the drift here is
    invisible: every drive metric would simply be measured somewhere else.
    """
    return (float(stamp_s) if stamp_s > 0.0
            else float(log_time_s) - float(log_minus_stamp_s))


class BagData(object):
    """Everything one bag contributes, decoded once."""

    def __init__(self):
        self.hand = []            # SensorSample, ROS-stamped
        self.announcements = []   # dicts
        self.declarations = []    # decoded /toss/record rows
        self.balls = []           # (t_bag, [ball, ...])
        self.mocap = []           # (t_bag, x, y, z) unlabelled markers
        self.fixture_cells = {}   # {int(t_bag): {cell, ...}} for LABELLED
        #                           markers — see FIXTURE_CELL_MM
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
                        # The COMMANDED release state, as the thrower published
                        # it. `build_announcement_fields` fills these from the
                        # same ReleaseState the declaration reports, so they are
                        # the command reference on a bag that predates
                        # /toss/record — which is the whole § 3.3 inversion.
                        'initial_position': [float(dec.initial_position.x),
                                             float(dec.initial_position.y),
                                             float(dec.initial_position.z)],
                        'initial_velocity': [float(dec.initial_velocity.x),
                                             float(dec.initial_velocity.y),
                                             float(dec.initial_velocity.z)],
                        'predicted_tof_sec': float(
                            getattr(dec, 'predicted_tof_sec', 0.0) or 0.0),
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
    for t, s, d, r, v, st in raw_hand:
        data.hand.append(SensorSample(
            t=hand_sample_time(s, t, data.log_minus_stamp_s),
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
                        px, py = float(mk.position.x), float(mk.position.y)
                        if abs(px) > LATERAL_MM or abs(py) > LATERAL_MM:
                            continue
                        pz = float(mk.position.z)
                        if mk.label:
                            # A LABELLED marker is rig, by QTM's own account.
                            # Remembered as (SECOND, cell): a cell, not a point,
                            # so the lookup is O(27) instead of an O(n*m) scan;
                            # bucketed by second because the platform MOVES
                            # between tosses and a session-wide fixture set is a
                            # swath of space, not a set of points. Measured on
                            # 2026-08-12_19-02-52 the session-wide version held
                            # 3374 cells and deleted 301 of toss 8's 341
                            # candidate rows — it ate the ball. Per-second
                            # buckets hold the same information at the only
                            # scope where "the rig is here" is true.
                            data.fixture_cells.setdefault(
                                int(math.floor(t)), set()).add(
                                    _cell(px, py, pz))
                            continue
                        data.mocap.append((t, px, py, pz))
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


def _cell(x, y, z, size: float = FIXTURE_CELL_MM):
    return (int(math.floor(x / size)), int(math.floor(y / size)),
            int(math.floor(z / size)))


def fixtures_near(fixture_cells, lo: float, hi: float) -> set:
    """The labelled-marker cells occupied in ``[lo, hi]``, second buckets ±1."""
    out = set()
    for sec in range(int(math.floor(lo)) - 1, int(math.floor(hi)) + 2):
        cells = fixture_cells.get(sec)
        if cells:
            out |= cells
    return out


def drop_fixtures(rows, cells) -> list:
    """Unlabelled rows that do NOT sit where a labelled marker has been. PURE.

    ``cells`` is a flat set — see :func:`fixtures_near` for the time scoping. The
    26 neighbouring cells are checked as well as the row's own, so the effective
    tolerance is 6-10 mm rather than "same cell": a marker one micron across a
    cell boundary must not survive.
    """
    if not cells:
        return list(rows)
    keep = []
    for t, x, y, z in rows:
        cx, cy, cz = _cell(x, y, z)
        if any((cx + i, cy + j, cz + k) in cells
               for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)):
            continue
        keep.append((t, x, y, z))
    return keep


def fit_ballistic(rows, *, g: float = GRAVITY_MMS2,
                  trim_floor_mm: float = TRIM_FLOOR_MM,
                  min_samples: int = MIN_ARC_SAMPLES, passes: int = 3):
    """Least-squares ballistic state from ``(t, x, y, z)`` rows.

    -> ``(pos_mm, vel_mms, t_ref, n, rms_mm, se_mms)`` or ``None``;
    ``pos``/``vel`` are the state AT ``t_ref`` (the mean sample time), so that

        ``ballistics_bc.position_at(pos, vel, t - t_ref)``

    reproduces the fitted arc exactly. The forward model IS that production
    function — ``x`` and ``y`` linear in time, ``z`` with the one gravity — and
    :func:`self_check` pins the round trip so the estimator here can never end up
    integrating a different physics from the planner it is scoring.

    Trimmed once per pass: a row beyond ``max(3·median residual,
    trim_floor_mm)`` is dropped and the fit repeated, never below
    ``min_samples`` rows. Robustness is not optional here — a single mislabelled
    platform marker inside the band moves an arrival velocity by more than the
    effect being measured — but a *hard* 3·rms rule on a clean 4-sample arc
    rejects its own worst point every pass, hence the absolute floor.

    ``se_mms`` is the per-axis 1-sigma standard error of the fitted VELOCITY,
    ``sigma_axis / sqrt(sum (t - t_mean)^2)`` with ``sigma_axis`` from that
    axis's own residuals at ``n - 2`` dof. It is the field that makes a short
    branch confessable: on 2026-08-12_19-02-52 toss 6 the ascending branch fits
    6 rows over 60 ms with an RMS of 11.5 mm — which looks clean — and returns a
    release speed 1400 mm/s wrong. Its ``se`` is an order larger than toss 8's,
    which spans 490 ms. A corpus that carried the velocity and not the ``se``
    would be carrying a plausible wrong number, which is the one thing the
    record exists to prevent.
    """
    kept = sorted(rows)
    if len(kept) < min_samples:
        return None
    result = None
    for _ in range(max(1, passes)):
        n = len(kept)
        if n < min_samples:
            break
        ts = np.array([r[0] for r in kept], dtype=float)
        t_ref = float(ts.mean())
        u = ts - t_ref
        design = np.stack([np.ones_like(u), u], axis=1)
        # z is made linear by subtracting the KNOWN gravity term, so the fit
        # solves for (position, velocity) only — 2 unknowns per axis. Fitting g
        # as a third unknown was tried and rejected on evidence: on
        # 2026-08-12_19-02-52 toss 8 a free-g fit reads 8426 (ascending) and
        # 7315 (descending) mm/s², i.e. it absorbs the bag-clock error into the
        # physics and then reports velocities consistent with a gravity the
        # planet does not have. Holding g at the planner's value is also what
        # the corpus actually wants measured: "how does this throw differ from
        # the model the machine flew?"
        cols = []
        for idx, extra in ((1, None), (2, None), (3, 0.5 * g * u * u)):
            v = np.array([r[idx] for r in kept], dtype=float)
            cols.append(v if extra is None else v + extra)
        coef = [np.linalg.lstsq(design, c, rcond=None)[0] for c in cols]
        pos = np.array([coef[0][0], coef[1][0], coef[2][0]])
        vel = np.array([coef[0][1], coef[1][1], coef[2][1]])
        pred = np.stack([design.dot(c) for c in coef], axis=1)
        meas = np.stack([np.array([r[1] for r in kept], dtype=float),
                         np.array([r[2] for r in kept], dtype=float),
                         cols[2]], axis=1)
        resid = np.sqrt(((pred - meas) ** 2).sum(axis=1))
        rms = float(np.sqrt((resid ** 2).mean()))
        suu = float((u * u).sum())
        dof = max(n - 2, 1)
        se = np.array([
            float(np.sqrt(((pred[:, k] - meas[:, k]) ** 2).sum() / dof
                          / suu)) if suu > 0.0 else float('inf')
            for k in range(3)])
        result = (pos, vel, t_ref, n, rms, se)
        limit = max(3.0 * float(np.median(resid)), float(trim_floor_mm))
        survivors = [r for r, e in zip(kept, resid) if e <= limit]
        if len(survivors) == n or len(survivors) < min_samples:
            break
        kept = survivors
    return result


def _lean_rad(vel):
    """Per-axis lean of a velocity off the vertical it travels along, radians.

    ``[atan2(vx, |vz|), atan2(vy, |vz|)]`` — positive x means "drifting toward
    +x". ``|vz|`` rather than ``vz`` so a release (``vz > 0``) and the arrival it
    produces (``vz < 0``) are described in the SAME sense: a throw that leans
    toward +x arrives leaning toward +x, and the two error channels can be read
    against each other without a sign table.
    """
    v = np.asarray(vel, dtype=float).reshape(3)
    ref = abs(float(v[2]))
    return (math.atan2(float(v[0]), ref), math.atan2(float(v[1]), ref))


def _angle_between_rad(a, b) -> float:
    """Total angle between two vectors, radians. ``None``-safe on zero length."""
    va = np.asarray(a, dtype=float).reshape(3)
    vb = np.asarray(b, dtype=float).reshape(3)
    na, nb = float(np.linalg.norm(va)), float(np.linalg.norm(vb))
    if na <= 0.0 or nb <= 0.0:
        return float('nan')
    return float(np.arccos(np.clip(float(va.dot(vb)) / (na * nb), -1.0, 1.0)))


def mine_mocap(data: BagData, landing_ros: float, plane_mm: float,
               band_mm: float = BAND_MM) -> dict:
    """The arrival block for ONE toss, via the IMPORTED estimator."""
    from ball_arrival_offset import fit_plane_crossing_full          # noqa: E402
    out = {'land_plane_mm': float(plane_mm)}
    if not data.mocap:
        return out
    land_bag = landing_ros + data.log_minus_stamp_s
    window = mocap_window(data, land_bag)
    if not window:
        return out
    # THE DESCENDING BRANCH, made explicit — and this is a FIX, not a tidy-up.
    # `fit_plane_crossing_full` locates the descending branch by cutting at the
    # first SUB-PLANE sample in time order. That is right for the window it was
    # written for (a Ball-Butler arrival, which opens with the ball already
    # falling) and wrong for a self toss, whose window opens with OUR OWN ball
    # sitting in the cup ~110 mm BELOW the plane: the very first row cuts the run
    # to zero length and the fit returns None. Measured on 2026-08-12_19-02-52:
    # all 17 self tosses reported `land_xy_global_mm: null` while the bag carries
    # a clean arc for several of them. So the apex is found first and only
    # post-apex rows reach the shipped estimator, whose own cut then still does
    # the job it was written for — excluding the post-contact excursion.
    ball = ball_rows(data, land_bag)
    apex_t = max(ball, key=lambda r: r[3])[0] if ball else None
    descending = ([r for r in ball if r[0] >= apex_t] if apex_t is not None
                  else [])
    got = fit_plane_crossing_full(descending, plane_mm, band_mm)
    gaps = [window[i + 1][0] - window[i][0] for i in range(len(window) - 1)]
    out.update({
        'apex_z_mm': (max(r[3] for r in ball) if ball else None),
        't_land_bag': land_bag,
        'mocap_gap_ms_max': (max(gaps) * 1e3 if gaps else None),
    })
    if got is None:
        return out
    x, y, n, z_lo, z_hi, rms, _t_lo, _t_hi = got
    out.update({
        'land_xy_global_mm': [x, y],
        'n_fit': int(n),
        'fit_rms_mm': float(rms),
        'fit_sparse': bool(n < MIN_FIT_SAMPLES),
    })
    del z_lo, z_hi          # reported by the estimator, not carried in the schema
    return out


def mocap_window(data: BagData, land_bag: float) -> list:
    """The laterally-gated ``(t, x, y, z)`` rows around one landing, sorted."""
    lo, hi = land_bag - PRE_S, land_bag + POST_S
    return [(t, x, y, z) for t, x, y, z in data.mocap
            if lo <= t <= hi and abs(x) <= LATERAL_MM and abs(y) <= LATERAL_MM]


def ball_rows(data: BagData, land_bag: float) -> list:
    """THE ball-candidate rows for one toss: gated, de-fixtured, de-dwelled.

    One selection, used by BOTH the landing fit and the arc fits, so the mocap
    block and the arrival/backcast block can never end up describing different
    sets of markers.
    """
    lo, hi = land_bag - PRE_S, land_bag + POST_S
    return drop_fixtures(mocap_window(data, land_bag),
                         fixtures_near(data.fixture_cells, lo, hi))


def command_reference(ann: dict, plane_mm: float) -> dict:
    """The COMMANDED release state the mined errors are differenced against.

    -> ``{'cmd_launch_vel_mms', 'cmd_flight_time_s', 'cmd_release_source',
    'release_plane_mm'}``; the velocity/time keys are ``None`` when the bag
    witnesses no command.

    Sourced from the announcement, which `build_announcement_fields` fills from
    the very ``ReleaseState`` the declaration reports — so a bag that predates
    ``/toss/record`` still carries the reference (the § 3.3 inversion). The join
    later upgrades `launch_vel_mms` / `flight_time_s` from the declaration; these
    stay mined so the errors are auditable on a mined-only row, where every 'D'
    field is null by construction.

    ``release_plane_mm`` is the announcement's own commanded release z when the
    bag has one, else the fit plane walked down the two GENERATED offsets
    (``hand_catch_offset − hand_throw_offset`` = 6.736 mm — the cup plane sits
    ABOVE the release plane). Never a third copy of the geometry.
    """
    vel = ann.get('initial_velocity')
    tof = ann.get('predicted_tof_sec') or 0.0
    ann_flight = float(ann.get('landing_time') or 0.0) - float(
        ann.get('throw_time') or 0.0)
    if not tof and ann_flight > 0.0:
        tof = ann_flight
    pos = ann.get('initial_position')
    release_plane = (float(pos[2]) if pos and float(pos[2]) > 0.0
                     else float(plane_mm) - float(hw.HAND_CATCH_OFFSET_MM)
                     + float(HAND_THROW_OFFSET_MM))
    out = {'release_plane_mm': release_plane,
           'cmd_launch_vel_mms': None, 'cmd_flight_time_s': None,
           'cmd_release_source': None}
    if vel and any(v != 0.0 for v in vel) and tof > 0.0:
        out['cmd_launch_vel_mms'] = [float(v) for v in vel]
        out['cmd_flight_time_s'] = float(tof)
        out['cmd_release_source'] = 'announcement'
    return out


def branches(rows, release_plane_mm: float, plane_mm: float):
    """-> ``(ascending, descending)`` row lists for one toss. PURE.

    Only rows at or above the RELEASE plane are considered: below it the ball is
    still in the hand being accelerated, so those samples are not on a ballistic
    arc at all and would drag the backcast toward the stroke.

    The descending list stops as soon as the ball comes back UP by more than
    :data:`BOUNCE_TOL_MM` — the same intent as
    `ball_arrival_offset.fit_plane_crossing_full`'s sub-plane cut, for the same
    reason (a bounce-out's post-contact excursion is large and its direction
    correlates with the outcome being scored), expressed as "it stopped
    descending" because the release-plane gate has already removed everything
    that fell INTO the cup. ``plane_mm`` is accepted for symmetry with the
    shipped estimator and to keep the two selections one call apart.
    """
    del plane_mm                     # the gate above the release plane is enough
    above = sorted(r for r in rows if r[3] >= release_plane_mm)
    if not above:
        return [], []
    apex_t = max(above, key=lambda r: r[3])[0]
    asc = [r for r in above if r[0] <= apex_t]
    desc = []
    z_min = None
    for row in above:
        if row[0] < apex_t:
            continue
        if z_min is not None and row[3] > z_min + BOUNCE_TOL_MM:
            break
        desc.append(row)
        z_min = row[3] if z_min is None else min(z_min, row[3])
    return asc, desc


def mine_arc(data: BagData, ann: dict, plane_mm: float,
             land_xy=None) -> dict:
    """Arrival kinematics (0a) + release-state backcast (0c) for ONE toss.

    THE definition point for every ``arrival`` / ``backcast`` / ``split`` field
    (``plans/active/critical-point-ilc.md`` Phase 0a/0c). Two ballistic fits over
    the SAME ball rows `land_xy` is derived from (:func:`ball_rows`), split at
    the apex by :func:`branches`:

    * **descending** — the arrival velocity where the fitted arc crosses the
      catch plane, and with it the arrival direction/speed errors against the
      nominal;
    * **ascending** — backcast to the release-plane crossing, giving the
      measured release position, velocity and INSTANT.

    Each branch is fitted WHOLE, not over a band near its plane. That is a
    measured decision, not a preference — see :data:`ASCENT_REF_LEAD_S`'s
    preamble for the numbers: a band-shaped estimator over 30-90 ms returns
    ``vz`` scattered by ±800 mm/s because ``/mocap_data`` has no header and the
    bag clock is only faithful in the large.

    **AND A THIRD FIT, over the WHOLE ARC, which owns every LATERAL velocity**
    (E-1, resolved 2026-08-13 — see the block above :data:`ASCENT_REF_LEAD_S`).
    A per-branch lateral slope reads ``v_true ± <db/dz·|vz|>`` under a
    height-locked centroid bias, which is a 104 mm/s branch-to-branch artefact on
    exactly the aim channels; the whole-arc slope cancels it by parity. So:

    * ``arrival_vel_mms[0:2]`` and ``release_vel_track_mms[0:2]`` — and with them
      ``arrival_dir_err_rad``, ``release_dir_err_rad`` and the lateral landing
      split — come from the WHOLE-ARC fit propagated to each plane crossing;
    * every VERTICAL quantity (``[2]`` of each velocity, both crossing instants,
      the flight time, both speed errors, both standard errors and both fit
      counts) is unchanged and still per-branch. The vertical channels were never
      contaminated: a height-locked bias is a *lateral* error by construction, and
      the vertical branch split is what exposes drag.

    The propagation is written out through ``ballistics_bc.velocity_at`` even
    though a no-drag lateral velocity is constant along the arc, so that a future
    model with a lateral force does not silently keep a constant.

    **The discriminator.** The measured release state is propagated to the cup
    plane by ``ballistics_bc.arrival_state_at_z`` — the production catch-side
    boundary condition — and the landing error is split exactly:

        ``land_err_mm = land_err_release_mm + land_err_flight_mm``

    the first being what the measured release already predicts (RELEASE-side
    fault: stroke speed, aim, dispatch timing), the second everything the
    no-drag ballistic model does not explain. The identity is pinned in
    :func:`self_check` and is unchanged.

    **What the split MEANS is not unchanged, and this is the re-marking E-1's
    resolution requires.** Both halves are lateral (xy) quantities, and under the
    bias-immune estimator there is no measurable lateral flight-phase physics
    left in them: the parity decomposition bounds any lateral aerodynamic force
    at the sub-mm/s scale (Magnus would need 3.4-8.8 rev/s against an observed
    ~0.5, and would land in the ODD channel, which is empty), and the remaining
    difference between the two halves is the difference between two ESTIMATORS of
    the same crossing — the descending-band position fit that produced
    ``land_xy_global_mm`` against the propagated release state. So
    **lateral landing error is release-side**; ``land_err_flight_mm`` is an
    estimator-agreement diagnostic, not a flight-physics channel, and nothing may
    fit it as one. The VERTICAL split (``release_speed_err_mms`` against
    ``flight_time_err_s``) is untouched and stays meaningful — it is two disjoint
    branches measuring the same throw, and it is what V2b cross-checks on.
    """
    ref = command_reference(ann, plane_mm)
    out = {'cmd_launch_vel_mms': ref['cmd_launch_vel_mms'],
           'cmd_flight_time_s': ref['cmd_flight_time_s'],
           'cmd_release_source': ref['cmd_release_source']}
    if not data.mocap:
        return out
    land_bag = float(ann.get('landing_time') or 0.0) + data.log_minus_stamp_s
    release_plane = float(ref['release_plane_mm'])
    asc, desc = branches(ball_rows(data, land_bag), release_plane, plane_mm)

    # ── E-1: the WHOLE-ARC fit, which owns both LATERAL velocities ────────────
    # The union, not a re-selection: exactly the rows the two branch fits see,
    # de-duplicated at the apex row they share. Fitting a different set here
    # would make the lateral and vertical channels describe different arcs.
    arc_lat_arrival = arc_lat_release = None
    arc_fit = fit_ballistic(sorted(set(asc) | set(desc)))
    if arc_fit is not None:
        pos, vel, t_ref, n, rms, se = arc_fit
        out['arc_fit_n'] = int(n)
        out['arc_fit_rms_mm'] = float(rms)
        out['arc_lateral_vel_se_mms'] = float(max(se[0], se[1]))
        # The apex instant of the FITTED arc, so the asymmetry is measured
        # against the parity centre rather than against the noisiest sample.
        # t_ref is the mean sample time by construction (`fit_ballistic`), so
        # this is exactly the probe's `mean(tau)` and reduces to -vz(t_ref)/g.
        out['coverage_asym_s'] = float(t_ref - (t_ref + float(vel[2])
                                                / GRAVITY_MMS2))
        for tag, target, descending in (('arrival', plane_mm, True),
                                        ('release', release_plane, False)):
            try:
                dt = ballistics_bc.touchdown_time(pos, vel, target,
                                                  descending=descending)
            except ValueError:
                # The ascending crossing is in the PAST of t_ref, which
                # `touchdown_time` refuses on purpose; anchor below the release
                # plane exactly as the backcast does, and for the same reason.
                if descending:
                    continue
                lead = float(asc[0][0]) - ASCENT_REF_LEAD_S - t_ref
                p_lead = ballistics_bc.position_at(pos, vel, lead)
                v_lead = ballistics_bc.velocity_at(vel, lead)
                try:
                    dt = lead + ballistics_bc.touchdown_time(
                        p_lead, v_lead, target, descending=False)
                except ValueError:
                    continue
            lat = ballistics_bc.velocity_at(vel, dt)
            if tag == 'arrival':
                arc_lat_arrival = (float(lat[0]), float(lat[1]))
            else:
                arc_lat_release = (float(lat[0]), float(lat[1]))

    def _lateral(vel, lateral):
        """The branch fit's VERTICAL component with the whole-arc LATERAL pair.

        ``None`` when the whole-arc fit did not produce one: the lateral channels
        are then absent rather than silently per-branch, because a per-branch
        lateral velocity is the E-1 artefact itself.
        """
        if lateral is None:
            return None
        return np.array([lateral[0], lateral[1], float(vel[2])])

    # ── 0a: the descending branch → arrival state at the catch plane ──────────
    arrival_vel = None
    fit = fit_ballistic(desc)
    if fit is not None:
        pos, vel, t_ref, n, rms, se = fit
        out['arrival_fit_n'] = int(n)
        out['arrival_fit_rms_mm'] = float(rms)
        out['arrival_vel_se_mms'] = float(se[2])
        try:
            dt = ballistics_bc.touchdown_time(pos, vel, plane_mm,
                                              descending=True)
        except ValueError:
            dt = None
        if dt is not None:
            arrival_vel = _lateral(ballistics_bc.velocity_at(vel, dt),
                                   arc_lat_arrival)
            if arrival_vel is not None:
                out['arrival_vel_mms'] = [float(v) for v in arrival_vel]
            out['t_arrival_fit_bag'] = float(t_ref + dt)

    # ── 0c: the ascending branch → release state at the release plane ─────────
    release_vel = release_pos = None
    fit = fit_ballistic(asc)
    if fit is not None:
        pos, vel, t_ref, n, rms, se = fit
        out['backcast_fit_n'] = int(n)
        out['backcast_fit_rms_mm'] = float(rms)
        out['release_vel_se_mms'] = float(se[2])
        # Anchor the state BEFORE the release so the ascending crossing is in
        # the future — `touchdown_time` refuses a past crossing, and that guard
        # is worth keeping rather than working around with a private solver.
        lead = float(asc[0][0]) - ASCENT_REF_LEAD_S - t_ref
        p_lead = ballistics_bc.position_at(pos, vel, lead)
        v_lead = ballistics_bc.velocity_at(vel, lead)
        try:
            dt = ballistics_bc.touchdown_time(p_lead, v_lead, release_plane,
                                              descending=False)
        except ValueError:
            dt = None
        if dt is not None:
            release_pos = ballistics_bc.position_at(p_lead, v_lead, dt)
            release_vel = _lateral(ballistics_bc.velocity_at(v_lead, dt),
                                   arc_lat_release)
            out['release_pos_track_mm'] = [float(v) for v in release_pos]
            if release_vel is not None:
                out['release_vel_track_mms'] = [float(v) for v in release_vel]
            out['t_release_fit_bag'] = float(t_ref + lead + dt)
            throw_ros = ann.get('throw_time')
            if throw_ros:
                out['release_time_err_ms'] = (
                    (t_ref + lead + dt - data.log_minus_stamp_s)
                    - float(throw_ros)) * 1e3

    # ── The measured flight time, and its error against the command ──────────
    if out.get('t_arrival_fit_bag') is not None and \
            out.get('t_release_fit_bag') is not None:
        flight = out['t_arrival_fit_bag'] - out['t_release_fit_bag']
        out['achieved_flight_s_mocap'] = float(flight)
        if ref['cmd_flight_time_s']:
            out['flight_time_err_s'] = float(flight - ref['cmd_flight_time_s'])

    # ── The errors against the command ────────────────────────────────────────
    cmd_v = ref['cmd_launch_vel_mms']
    cmd_t = ref['cmd_flight_time_s']
    if cmd_v is not None and cmd_t:
        if release_vel is not None:
            out['release_vel_err_mms'] = [
                float(release_vel[i] - cmd_v[i]) for i in range(3)]
            out['release_speed_err_mms'] = float(
                np.linalg.norm(release_vel) - np.linalg.norm(cmd_v))
            meas, nom = _lean_rad(release_vel), _lean_rad(cmd_v)
            out['release_dir_err_rad'] = [meas[0] - nom[0], meas[1] - nom[1]]
        if arrival_vel is not None:
            # The nominal arrival velocity is the PRODUCTION propagation of the
            # commanded release — never a second copy of the ballistics.
            nom_arr = ballistics_bc.arrival_velocity(cmd_v, cmd_t)
            out['arrival_speed_err_mms'] = float(
                np.linalg.norm(arrival_vel) - np.linalg.norm(nom_arr))
            meas, nom = _lean_rad(arrival_vel), _lean_rad(nom_arr)
            out['arrival_dir_err_rad'] = [meas[0] - nom[0], meas[1] - nom[1]]
            out['arrival_dir_err_norm_rad'] = _angle_between_rad(
                arrival_vel, nom_arr)

    # ── The RELEASE-vs-FLIGHT split of the landing error ─────────────────────
    cup = ann.get('landing_position')
    if (release_pos is not None and release_vel is not None
            and land_xy is not None and cup):
        try:
            pred, _v, _t = ballistics_bc.arrival_state_at_z(
                release_pos, release_vel, plane_mm, descending=True)
        except ValueError:
            pred = None
        if pred is not None:
            out['land_err_release_mm'] = [float(pred[0] - cup[0]),
                                          float(pred[1] - cup[1])]
            out['land_err_flight_mm'] = [float(land_xy[0] - pred[0]),
                                         float(land_xy[1] - pred[1])]
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
        # ILC Phase 0a/0c. AFTER the land fit, because the release-vs-flight
        # split differences against `land_xy_global_mm` and the identity
        # land_err = release-part + flight-part only holds if both halves read
        # the same landing point.
        row.update(mine_arc(data, ann, this_plane,
                            land_xy=row.get('land_xy_global_mm')))
        _mark_usable(row)
        # THE SAME PLANE CHECK, on the MINED half. `enforce_declared_planes`
        # compares the fit plane against the DECLARATION, so it can only fire on
        # a joined row — and the whole reference corpus is `mined-only`, which
        # means the check that exists to catch "the corpus is scoring arrivals at
        # the wrong height" never ran on a single row of it. The announcement's
        # own `landing_position[2]` is the same quantity the declaration's
        # `catch_point_global_mm[2]` is (both come from `_toss_release_state`;
        # see the land_err comment above), so it answers the same question on a
        # bag that predates `/toss/record`. Same constant, same refusal.
        if cup and len(cup) > 2 and cup[2] is not None and abs(
                float(cup[2]) - float(this_plane)) > PLANE_MISMATCH_TOL_MM:
            refuse_plane(row)
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
    # The RELEASE-side gate is deliberately NOT `reasons`: it asks whether the
    # two ARC fits are trustworthy, and says nothing about the landing fit those
    # reasons are about. See RELEASE_FIT_MIN_SAMPLES for the dissociation this
    # rests on, and why re-using `usable_for_speed_fit` here would both drop good
    # rows and admit bad ones.
    row['usable_for_release_fit'] = _release_fit_ok(row)
    # E-1 (2026-08-13). A SEPARATE flag, for the same reason
    # `usable_for_release_fit` is one: it asks a different question of a
    # different fit. `usable_for_aim_fit` gates the AIM MAP's fit, which consumes
    # `land_err_mm` — a landing POSITION, which the branch artefact never
    # touched — so folding the lateral-velocity guard into it would drop good
    # rows from a fit that does not depend on the estimator being guarded. The
    # vertical flags are untouched for the mirror-image reason.
    row['usable_for_lateral_fit'] = _lateral_fit_ok(row)
    if row.get('coverage_asym_s') is not None and not row[
            'usable_for_lateral_fit']:
        # Recorded only when a REAL measurement was refused. An absent
        # `coverage_asym_s` is not appended: there is no lateral estimate to
        # exclude, and the null field plus the False flag already say so.
        reasons.append('coverage_asym')
    row['excluded_reason'] = ','.join(sorted(set(reasons))) or None


def _lateral_fit_ok(row: dict) -> bool:
    """Is this row's LATERAL (whole-arc) estimate admissible? (E-1, 2026-08-13)

    Two ways to fail, and they are different failures:

    * ``coverage_asym_s`` **absent** — either the row carries no arc at all, or
      it was mined BEFORE the whole-arc estimator existed and its lateral
      velocities ARE the per-branch artefact (~104 mm/s, ~11 mrad of phantom
      arrival direction, ~44 mm of phantom aim). A corpus mixes mines freely —
      ``ilc_fit_lib.load_corpus`` globs ``toss_records_*.jsonl`` — so absence has
      to REFUSE rather than default-pass, or one stale file silently re-opens
      E-1 inside a fit that has stopped masking the lateral channels.
    * present and beyond :data:`COVERAGE_ASYM_MAX_S` — a half-seen arc, where
      the whole-arc fit's parity cancellation no longer holds.
    """
    asym = row.get('coverage_asym_s')
    if asym is None:
        return False
    try:
        asym = float(asym)
    except (TypeError, ValueError):
        return False
    return math.isfinite(asym) and abs(asym) <= COVERAGE_ASYM_MAX_S


def _release_fit_ok(row: dict) -> bool:
    """Is this row's RELEASE-velocity estimate admissible? (ILC Phase 0c)

    Three conditions, all on the fits' own self-report: both branches carry at
    least :data:`RELEASE_FIT_MIN_SAMPLES` rows, and the fitted release velocity's
    1-sigma standard error is under :data:`RELEASE_FIT_MAX_SE_MMS`. The ARRIVAL
    branch is required as well as the ascending one because a row whose
    descending branch never fitted is a row whose arc was only half seen — the
    corpus's short-branch failures are exactly those rows, and their release
    speeds are wrong by 1000-1600 mm/s against a signal of 480.
    """
    for name in ('backcast_fit_n', 'arrival_fit_n'):
        n = row.get(name)
        if n is None or int(n) < RELEASE_FIT_MIN_SAMPLES:
            return False
    se = row.get('release_vel_se_mms')
    if se is None or not math.isfinite(float(se)):
        return False
    return float(se) <= RELEASE_FIT_MAX_SE_MMS


def refuse_plane(row: dict) -> None:
    """Mark ONE row refused for a catch-plane mismatch. The single enforcement
    point, called from both the declared path (:func:`enforce_declared_planes`)
    and the mined-only path (:func:`mine_bag`).

    EVERY fit flag is cleared, not a subset — see
    :func:`enforce_declared_planes` for what each of them depends on the plane
    for.
    """
    for flag in ('usable_for_aim_fit', 'usable_for_speed_fit',
                 'usable_for_timing_fit', 'usable_for_release_fit',
                 'usable_for_lateral_fit'):
        row[flag] = False
    reasons = [r for r in (row.get('excluded_reason') or '').split(',') if r]
    row['excluded_reason'] = ','.join(sorted(set(reasons + ['plane_mismatch'])))


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

    **EVERY fit flag is refused, TIMING included (audit fix 2026-08-12).** The
    carve-out this used to make — "the timing fit reads sensor edges and release
    instants, which the fit plane has no bearing on" — was true when it was
    written and stopped being true the moment Phase 0a landed. Release-side
    timing still is plane-independent: ``release_time_err_ms`` compares the
    ascending backcast's crossing of the RELEASE plane against the announced
    ``throw_time``. **Arrival-side timing is not.** ``t_arrival_fit_bag`` IS the
    ``plane_mm`` crossing of the descending fit, and ``achieved_flight_s_mocap``
    and ``flight_time_err_s`` are both derived from it — so a 20 mm plane error
    moves the measured flight time by ~4 ms at a 4.9 m/s arrival, the same order
    as the dispatch shift the timing fit exists to measure. One flag cannot be
    half-true, so the row is refused.

    **The LATERAL flag too** (E-1, 2026-08-13). The whole-arc lateral VELOCITY is
    plane-independent — no-drag ``vx``/``vy`` are constant along the arc — but
    ``arrival_dir_err_rad`` is a LEAN, ``atan2(vx, |vz|)``, and its ``|vz|`` is
    the descending fit evaluated at ``plane_mm``. A 20 mm plane error moves that
    denominator, so the direction channel is as plane-dependent as the flight
    time is.

    The plane cannot simply be corrected here: ``mine_mocap`` fits the crossing
    at a plane chosen BEFORE the declaration is joined, so a mismatch means the
    mocap window was already reduced against the wrong height. Re-mining with
    ``--plane <declared z>`` is the fix, and the message says so.
    """
    lines = check_declared_planes(rows)
    for row in mismatched_planes(rows):
        refuse_plane(row)
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
        # Every cell is None-safe. A declared-only row (a declaration the bag
        # kept but whose announcement it lost) carries NO derived label at all,
        # and formatting that None used to abort the whole report AFTER the
        # per-toss table had printed — losing the ledger, the plane checks and
        # the jsonl for a bag whose only fault was an unmatched declaration.
        # Found running 2026-08-12_19-02-52 (19 declarations, 17 announcements).
        print('  {:>4} {:>13} {:>10} {:>9} {:>9} {:>9}  {}'.format(
            i, '{:.3f}'.format(throw) if throw else '-',
            row.get('label') or '-',
            '{:+.0f}'.format((dep - throw) * 1e3) if dep and throw else '-',
            '{:+.0f}'.format((cat - land) * 1e3) if cat and land else '-',
            '{:.1f}'.format(err) if err is not None else '-',
            row.get('record_provenance') or '-'))
    counts = {}
    for row in rows:
        key = row['label'] or '(none)'          # declared-only rows have none
        counts[key] = counts.get(key, 0) + 1
    print('  labels: ' + ', '.join('{}={}'.format(k, counts[k])
                                   for k in sorted(counts)))
    usable = sum(1 for r in rows if r.get('usable_for_aim_fit'))
    rel = sum(1 for r in rows if r.get('usable_for_release_fit'))
    lat = sum(1 for r in rows if r.get('usable_for_lateral_fit'))
    print('  usable_for_aim_fit: {}/{}   usable_for_release_fit: {}/{}   '
          'usable_for_lateral_fit: {}/{}'.format(
              usable, len(rows), rel, len(rows), lat, len(rows)))
    asym = [abs(r['coverage_asym_s']) for r in rows
            if r.get('coverage_asym_s') is not None]
    if asym:
        print('  coverage_asym_s: n={} median {:.4f} s  max {:.4f} s  '
              '(refusal at {:.2f} s: {} row(s))'.format(
                  len(asym), float(np.median(asym)), max(asym),
                  COVERAGE_ASYM_MAX_S,
                  sum(1 for v in asym if v > COVERAGE_ASYM_MAX_S)))
    for problem in check_declared_planes(rows):
        print('  ' + problem)
    print_arc_report(rows)


def _fmt(value, spec='{:.1f}'):
    return '-' if value is None else spec.format(value)


def print_arc_report(rows) -> None:
    """The ILC Phase-0a/0c block: arrival kinematics and release backcast.

    Printed separately from the label table because it answers a different
    question — *did this sitting see the ball at all, and where did the throw
    error come from?* — and because a bag with no mocap has an entirely empty
    block, which is itself the finding worth seeing at a glance.
    """
    have = {i for i, r in enumerate(rows)
            if r.get('arrival_vel_mms') or r.get('release_vel_track_mms')}
    print('  --- arc (0a arrival / 0c backcast): {}/{} rows carry a fit'
          .format(len(have), len(rows)))
    if not have:
        return
    head = ('#', 'n_as', 'se_as', 'n_ar', 'se_ar', 'v_rel_z', 'v_arr_z',
            'dv_rel', 'dirE_mr', 'trelE_ms', 'Tflt', 'dT_ms')
    fmt = ('    {:>4} {:>5} {:>7} {:>5} {:>7} {:>9} {:>9} {:>9} {:>8} {:>8} '
           '{:>8} {:>8}')
    print(fmt.format(*head))
    for i, row in enumerate(rows, 1):
        if (i - 1) not in have:
            continue
        rv = row.get('release_vel_track_mms')
        av = row.get('arrival_vel_mms')
        de = row.get('arrival_dir_err_norm_rad')
        print(fmt.format(
            i, _fmt(row.get('backcast_fit_n'), '{:d}'),
            _fmt(row.get('release_vel_se_mms'), '{:.0f}'),
            _fmt(row.get('arrival_fit_n'), '{:d}'),
            _fmt(row.get('arrival_vel_se_mms'), '{:.0f}'),
            _fmt(rv[2] if rv else None, '{:.0f}'),
            _fmt(av[2] if av else None, '{:.0f}'),
            _fmt(row.get('release_speed_err_mms'), '{:+.0f}'),
            _fmt(de * 1e3 if de is not None else None, '{:.1f}'),
            _fmt(row.get('release_time_err_ms'), '{:+.0f}'),
            _fmt(row.get('achieved_flight_s_mocap'), '{:.3f}'),
            _fmt((row.get('flight_time_err_s') or 0.0) * 1e3
                 if row.get('flight_time_err_s') is not None else None,
                 '{:+.0f}')))
        split_r = row.get('land_err_release_mm')
        split_f = row.get('land_err_flight_mm')
        if split_r and split_f:
            print('         split: land_err {} mm = RELEASE {} + FLIGHT {}'
                  .format(['{:+.1f}'.format(v) for v in row['land_err_mm']],
                          ['{:+.1f}'.format(v) for v in split_r],
                          ['{:+.1f}'.format(v) for v in split_f]))
    for name, key, spec in (
            ('release speed err (mm/s)', 'release_speed_err_mms', '{:+.0f}'),
            ('release time err (ms)', 'release_time_err_ms', '{:+.0f}'),
            ('arrival dir err (mrad)', 'arrival_dir_err_norm_rad', '{:.1f}'),
            ('flight time err (ms)', 'flight_time_err_s', '{:+.1f}')):
        vals = [r.get(key) for r in rows if r.get(key) is not None]
        if key == 'arrival_dir_err_norm_rad':
            vals = [v * 1e3 for v in vals]
        if key == 'flight_time_err_s':
            vals = [v * 1e3 for v in vals]
        if not vals:
            continue
        vals.sort()
        med = vals[len(vals) // 2]
        p95 = vals[min(len(vals) - 1, int(round(0.95 * (len(vals) - 1))))]
        print('    {:26s} n={:<3d} median {:>9} p95 {:>9}'.format(
            name, len(vals), spec.format(med), spec.format(p95)))
    # THE HEADLINE POPULATION, expressible from the corpus rather than applied by
    # hand in a report: the rows whose ARC fits are admissible. Printed next to
    # the all-rows line above so the difference between them is visible — on the
    # reference bag it is 16 clean rows against 22 with any release number at
    # all, and the six excluded ones are wrong by 1000-1600 mm/s.
    clean = sorted(r['release_speed_err_mms'] for r in rows
                   if r.get('usable_for_release_fit')
                   and r.get('release_speed_err_mms') is not None)
    print('    {:26s} n={:<3d} median {:>9} (usable_for_release_fit)'.format(
        'release speed err, CLEAN', len(clean),
        '-' if not clean else '{:+.0f}'.format(clean[len(clean) // 2])))


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


def synth_arc(release_pos, release_vel, *, plane_mm, t_release=10.0, dt=0.005,
              wobble_mm=0.0):
    """A pure ballistic arc sampled at a mocap-like rate -> ``(t, x, y, z)``.

    Built with ``ballistics_bc.position_at``, i.e. the SAME propagator the fit
    inverts. That is deliberate and it is not circular: these cases test the
    ESTIMATOR (does least squares recover the state, does the crossing solve pick
    the right root, does trimming reject a stray), not the physics. The physics
    is written out longhand and checked independently in
    ``tests/ros/test_toss_record_miner.py``.

    ``wobble_mm`` adds a deterministic sawtooth to x and z so a case can be run
    against a non-exact track without depending on a random seed.
    """
    out = []
    t = 0.0
    end = ballistics_bc.touchdown_time(release_pos, release_vel, plane_mm,
                                       descending=True) + 0.02
    i = 0
    while t <= end:
        p = ballistics_bc.position_at(release_pos, release_vel, t)
        w = wobble_mm * ((i % 3) - 1)
        out.append((t_release + t, float(p[0]) + w, float(p[1]),
                    float(p[2]) + w))
        t += dt
        i += 1
    return out


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

    # ── ILC Phase 0a/0c: the arc estimator ───────────────────────────────────
    plane = 809.08
    rel_plane = plane - float(hw.HAND_CATCH_OFFSET_MM) + HAND_THROW_OFFSET_MM
    rel_pos = np.array([150.0, -120.0, rel_plane])
    rel_vel = np.array([60.0, -25.0, 4436.0])
    t_rel = 1000.0
    arc = synth_arc(rel_pos, rel_vel, plane_mm=plane, t_release=t_rel)
    asc, desc = branches(arc, rel_plane, plane)
    check('the branch split finds both halves of a synthetic arc',
          len(asc) > 20 and len(desc) > 20, True)

    def _release_from(rows):
        pos, vel, t_ref, _n, _rms, _se = fit_ballistic(rows)
        lead = float(rows[0][0]) - ASCENT_REF_LEAD_S - t_ref
        p = ballistics_bc.position_at(pos, vel, lead)
        v = ballistics_bc.velocity_at(vel, lead)
        dt = ballistics_bc.touchdown_time(p, v, rel_plane, descending=False)
        return (ballistics_bc.position_at(p, v, dt),
                ballistics_bc.velocity_at(v, dt), t_ref + lead + dt)

    got_pos, got_vel, got_t = _release_from(asc)
    check('a pure ballistic ascent backcasts to its own release VELOCITY',
          bool(np.max(np.abs(got_vel - rel_vel)) < 1.0), True)
    check('a pure ballistic ascent backcasts to its own release POSITION',
          bool(np.max(np.abs(got_pos - rel_pos)) < 0.1), True)
    check('a pure ballistic ascent backcasts to its own release INSTANT',
          abs(got_t - t_rel) < 1e-3, True)

    def _arrival_from(rows):
        pos, vel, t_ref, _n, _rms, _se = fit_ballistic(rows)
        dt = ballistics_bc.touchdown_time(pos, vel, plane, descending=True)
        return ballistics_bc.velocity_at(vel, dt), t_ref + dt

    arr_vel, arr_t = _arrival_from(desc)
    # The nominal the corpus differences against is the PRODUCTION propagation of
    # the commanded release. On a noise-free arc the two must agree exactly, and
    # a disagreement here means the miner and the planner are running different
    # ballistics — the one failure design constraint 1 exists to make impossible.
    flight = ballistics_bc.touchdown_time(rel_pos, rel_vel, plane,
                                          descending=True)
    check('the descending fit recovers the PRODUCTION arrival velocity',
          bool(np.max(np.abs(
              arr_vel - ballistics_bc.arrival_velocity(rel_vel, flight)))
              < 1.0), True)
    check('the measured flight time recovers the production touchdown time',
          abs((arr_t - got_t) - flight) < 2e-3, True)

    # REFUSE — an arc with fewer than MIN_ARC_SAMPLES rows yields no state at
    # all. Two rows determine a line exactly and report zero residual, which
    # would read as a perfect measurement.
    check('a 3-row arc is REFUSED rather than fitted',
          fit_ballistic(arc[:3]) is None, True)

    # The fit's forward model IS the production propagator (constraint 1).
    pos, vel, t_ref, _n, _rms, _se = fit_ballistic(desc)
    row = desc[len(desc) // 2]
    fwd = ballistics_bc.position_at(pos, vel, row[0] - t_ref)
    check('the fitted state re-generates the arc through ballistics_bc',
          bool(max(abs(fwd[0] - row[1]), abs(fwd[1] - row[2]),
                   abs(fwd[2] - row[3])) < 1e-6), True)

    # Two-sided on the TRIM: a stray inside the branch must not move the answer,
    # and the untrimmed fit must be shown to move — a trim that is not
    # load-bearing is a trim nobody would notice going missing.
    # A stray at the END of a branch has the most leverage on the slope, which
    # is exactly where the real ones landed (the platform rim markers reappear
    # while the ball is near the cup plane).
    sparse = desc[-12:]                 # the branch length real tosses deliver
    sparse_vel = fit_ballistic(sparse)[1]
    stray = list(sparse) + [(sparse[-1][0] + 0.005, sparse[-1][1] + 300.0,
                             sparse[-1][2], sparse[-1][3] + 250.0)]
    trimmed = fit_ballistic(stray)
    naive = fit_ballistic(stray, trim_floor_mm=1e9)
    check('a 250 mm stray at the end of the branch is TRIMMED out',
          bool(abs(float(trimmed[1][2] - sparse_vel[2])) < 1.0), True)
    check('and the untrimmed fit really is moved by it (the trim matters)',
          bool(abs(float(naive[1][2] - sparse_vel[2])) > 200.0), True)

    # The standard error is what makes a SHORT branch confessable — comparable
    # RMS, an order more uncertainty. Toss 6 of 2026-08-12_19-02-52 is the real
    # case: 6 rows, 11.5 mm RMS, and a release speed 1400 mm/s wrong. Run on a
    # WOBBLED arc, because on a noise-free one both standard errors are zero and
    # the case could not fail.
    noisy = synth_arc(rel_pos, rel_vel, plane_mm=plane, t_release=t_rel,
                      wobble_mm=8.0)
    n_asc, _n_desc = branches(noisy, rel_plane, plane)
    short = [r for r in n_asc if r[0] <= n_asc[0][0] + 0.06]
    long_se = fit_ballistic(n_asc)[5][2]
    short_se = (fit_ballistic(short)[5][2] if len(short) >= MIN_ARC_SAMPLES
                else float('inf'))
    check('a 60 ms branch reports a far larger velocity standard error',
          short_se > 5.0 * long_se, True)

    # A labelled-marker cell is refused; the ball a few cm away is not.
    cells = {int(math.floor(t_rel)): {_cell(0.0, 0.0, 1000.0)}}
    kept = drop_fixtures([(t_rel, 1.0, 1.0, 1001.0), (t_rel, 60.0, 0.0, 1000.0)],
                         cells[int(math.floor(t_rel))])
    check('an unlabelled row sitting on a LABELLED cell is dropped',
          [r[1] for r in kept], [60.0])

    # The branch rules, both of them, on the shapes that actually broke:
    # a ball resting in the cup BELOW the release plane (which used to cut the
    # landing fit to zero length), and a post-contact excursion.
    with_rest = [(t_rel - 0.3, 150.0, -120.0, 700.0)] + arc
    a2, d2 = branches(with_rest, rel_plane, plane)
    check('the ball resting in the cup is not part of the ascending branch',
          len(a2), len(asc))
    bounced = list(arc) + [(arc[-1][0] + 0.02 * i, 400.0, -400.0, plane + 200.0)
                           for i in range(1, 6)]
    _a3, d3 = branches(bounced, rel_plane, plane)
    check('a post-contact excursion is cut out of the descending branch',
          len(d3), len(desc))

    # The lean convention: a throw leaning +x arrives leaning +x, so release and
    # arrival errors can be read against each other without a sign table.
    check('the lean sign is the same going up and coming down',
          _lean_rad(rel_vel)[0] > 0.0
          and _lean_rad(ballistics_bc.arrival_velocity(rel_vel, flight))[0] > 0.0,
          True)

    # The RELEASE-FIT admission flag, both sides. `usable_for_speed_fit` gates on
    # the LANDING fit and so cannot express the population behind the headline
    # release-speed number: the reference bag carries rows with a clean 58/67-row
    # arc pair and no landing fit at all, and rows with a good landing fit whose
    # ascending branch is 6 rows long and 1400 mm/s wrong.
    def _rel(bc, ar, se, **extra):
        row = toss_record.blank_record()
        row.update({'backcast_fit_n': bc, 'arrival_fit_n': ar,
                    'release_vel_se_mms': se, 'label': 'CAUGHT',
                    't_departure_raw_ros': 1.0, 'land_xy_global_mm': [0.0, 0.0],
                    'fit_rms_mm': 0.5, 'coverage_asym_s': 0.0})
        row.update(extra)
        _mark_usable(row)
        return row

    check('a long-branch, low-SE arc is admissible for the release fit',
          _rel(60, 60, 7.0)['usable_for_release_fit'], True)
    check('a 6-row ascending branch is NOT, whatever its RMS',
          _rel(6, 60, 380.0)['usable_for_release_fit'], False)
    check('nor is one whose DESCENDING branch never fitted',
          _rel(60, None, 7.0)['usable_for_release_fit'], False)
    check('nor is a long branch with a large velocity standard error',
          _rel(60, 60, RELEASE_FIT_MAX_SE_MMS + 1.0)['usable_for_release_fit'],
          False)
    check('the flag is INDEPENDENT of the landing fit, both ways',
          (_rel(60, 60, 7.0, land_xy_global_mm=None)['usable_for_release_fit'],
           _rel(6, None, 300.0)['usable_for_speed_fit']),
          (True, True))

    # ── E-1: the WHOLE-ARC lateral estimator ─────────────────────────────────
    # The artefact, reproduced on a SYNTHETIC arc with a known height-locked
    # bias, and then removed. This is the whole of E-1 in eight lines: b(z) is
    # applied to y as a linear function of height, so the per-branch fits split
    # by +-<db/dz . |vz|> while the whole-arc fit returns the truth. Written
    # against an EXACT prediction, not a tolerance around a measurement: with
    # b(z) = k.(z - z0) the ascending slope error is +k.<|vz|> and the
    # descending is -k.<|vz|>, so the branch delta is 2.k.<|vz|> and the
    # whole-arc estimate is exact for symmetric coverage.
    k_bias = 0.02                       # mm of y bias per mm of height
    biased = [(t, x, y + k_bias * (z - rel_plane), z) for t, x, y, z in arc]
    b_asc, b_desc = branches(biased, rel_plane, plane)
    v_asc = fit_ballistic(b_asc, trim_floor_mm=1e9)[1]
    v_desc = fit_ballistic(b_desc, trim_floor_mm=1e9)[1]
    v_whole = fit_ballistic(sorted(set(b_asc) | set(b_desc)),
                            trim_floor_mm=1e9)[1]
    # Measured on this arc at k = 0.02: asc +19.6, desc -68.7 (delta -88.3)
    # against a truth of -25.0, which the whole-arc fit returns to 0.5 mm/s.
    check('a height-locked bias SPLITS the two branches\' lateral velocity',
          float(v_desc[1] - v_asc[1]) < -50.0, True)
    check('...while the WHOLE-ARC fit recovers the true lateral velocity',
          bool(abs(float(v_whole[1]) - float(rel_vel[1])) < 1.0), True)
    # ... and the miner's own estimator, end to end, agrees with the whole-arc
    # fit rather than with either branch. `mine_arc` is the definition point, so
    # this is the case that would fail if a lateral component ever went back to
    # a branch fit.
    class _ArcData(object):
        hand = []
        balls = []
        link = []
        traj = []
        declarations = []
        fixture_cells = {}
        log_minus_stamp_s = 0.0
        announcements = []

        def __init__(self, rows):
            self.mocap = list(rows)

    ann = {'thrower': 'jugglebot', 'target': 'jugglebot',
           'throw_time': t_rel, 'landing_time': t_rel + flight,
           'landing_position': [float(rel_pos[0]), float(rel_pos[1]),
                                float(plane)],
           'initial_position': [float(v) for v in rel_pos],
           'initial_velocity': [float(v) for v in rel_vel],
           'predicted_tof_sec': float(flight)}
    # `ball_rows` windows +-1.2 s about the ANNOUNCED landing, so the synthetic
    # arc has to sit inside that window — it does, at 0.9 s of flight.
    mined = mine_arc(_ArcData(biased), ann, plane)
    check('mine_arc takes the LATERAL arrival velocity from the whole arc',
          bool(abs(mined['arrival_vel_mms'][1] - float(v_whole[1])) < 1.0),
          True)
    check('...and NOT from the descending branch',
          bool(abs(mined['arrival_vel_mms'][1] - float(v_desc[1])) > 30.0),
          True)
    check('...and the release lateral velocity from the whole arc too',
          bool(abs(mined['release_vel_track_mms'][1] - float(v_whole[1]))
               < 1.0), True)
    check('the VERTICAL arrival velocity is still the descending branch',
          bool(abs(mined['arrival_vel_mms'][2]
                   - float(_arrival_from(b_desc)[0][2])) < 1e-6), True)
    check('a symmetric synthetic arc reports a small coverage asymmetry',
          abs(mined['coverage_asym_s']) < COVERAGE_ASYM_MAX_S, True)
    # The gate, both sides, ABSOLUTE — a widened COVERAGE_ASYM_MAX_S fails here.
    check('a coverage asymmetry inside 0.1 s ADMITS the lateral estimate',
          _rel(60, 60, 7.0, coverage_asym_s=0.09)['usable_for_lateral_fit'],
          True)
    check('a coverage asymmetry past 0.1 s REFUSES it',
          _rel(60, 60, 7.0, coverage_asym_s=0.11)['usable_for_lateral_fit'],
          False)
    check('...and says why, without touching the VERTICAL flags',
          [_rel(60, 60, 7.0, coverage_asym_s=0.11)[f] for f in
           ('excluded_reason', 'usable_for_release_fit',
            'usable_for_speed_fit')],
          ['coverage_asym', True, True])
    # A PRE-E-1 mine carries no coverage_asym_s at all, and its lateral
    # velocities ARE the artefact. Absence must refuse, not default-pass.
    stale = _rel(60, 60, 7.0)
    stale.pop('coverage_asym_s')
    _mark_usable(stale)
    check('a row mined BEFORE the whole-arc estimator is refused for lateral',
          (stale['usable_for_lateral_fit'], stale['excluded_reason']),
          (False, None))

    # The plane refusal on a MINED-ONLY row. `enforce_declared_planes` needs a
    # declaration, so before this the whole reference corpus — every row
    # `mined-only` — was never plane-checked at all.
    mined_bad = _rel(60, 60, 7.0)
    refuse_plane(mined_bad)
    check('a plane-refused row loses EVERY fit flag, timing included',
          [mined_bad[f] for f in ('usable_for_aim_fit', 'usable_for_speed_fit',
                                  'usable_for_timing_fit',
                                  'usable_for_release_fit',
                                  'usable_for_lateral_fit')],
          [False, False, False, False, False])
    check('...and says why', mined_bad['excluded_reason'], 'plane_mismatch')

    # The split IDENTITY. It is what makes "release vs flight" a decomposition
    # rather than two loosely-related numbers, and it must hold to the bit.
    cup = [150.0, -120.0, plane]
    pred, _v, _t = ballistics_bc.arrival_state_at_z(got_pos, got_vel, plane,
                                                   descending=True)
    land = [float(pred[0]) + 12.0, float(pred[1]) - 7.0]
    rel_part = [float(pred[0]) - cup[0], float(pred[1]) - cup[1]]
    fly_part = [land[0] - float(pred[0]), land[1] - float(pred[1])]
    check('land_err == release-part + flight-part, exactly',
          [round(rel_part[i] + fly_part[i], 9) for i in (0, 1)],
          [round(land[i] - cup[i], 9) for i in (0, 1)])

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
