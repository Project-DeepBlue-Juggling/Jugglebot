#!/usr/bin/env python3
"""Per-throw hand stroke timeline from a recorded toss trace or session rosbag.

WHAT IT MEASURES
----------------
For every throw in a session, reconstructs the hand's commanded-vs-measured
timeline around the throw stroke and emits, in one row per throw:

    stroke_start    first sample where the COMMANDED velocity leaves rest
    rel_ann         modelled release instant, anchored on the announcement's
                    ``throw_time``  (the host's *intended* release)
    rel_fit         modelled release instant, recovered by least-squares
                    time-shifting the CLOSED-FORM stroke profile onto the
                    observed ``pos_cmd`` ascent (the *physical* release)
    shift           rel_fit - rel_ann, i.e. the dispatch-path latency that
                    lands the physical stroke late.  **This is the number a
                    host-side stroke-busy window has to cover.**
    x2_cross        first sample whose pos_meas passes the modelled release
                    position x2, and the measured velocity there
    trunc           instant ``pos_cmd`` stops following the stroke and freezes
                    at the live encoder value (the queue was cleared and
                    re-preluded mid-stroke), with that value
    seeds           EVERY from-rest quintic seed in the window, with how far
                    each sits from the live ``pos_meas`` just before it.  A
                    double arm dispatch shows up here as two seeds ~17 ms
                    apart, each equal to the encoder value at its own instant
    quintic         the from-rest quintic that replaced the rest of the stroke:
                    its target, its modelled duration T, its observed end
    peak            coasting peak of pos_meas after the truncation
    pullback        most negative pos-loop velocity after the peak
    dip             dip bottom, and depth in rev / mm / % of effective stroke
    dip_below_x3    how far under the stroke end x3 the hand ended up.  **This,
                    not ``dip``, is the row the operator runbook gates on** —
                    see the note below
    recovery        first instant pos_meas is back at the quintic target
    first_neg_cmd   first DOWNWARD command of any kind after the release (a
                    braking prelude shows up here, not as ``catch_desc``)
    catch_desc      onset of the armed catch descent — the downward command that
                    actually travels toward x5

A throw whose ``pos_cmd`` follows the planned decel ramp all the way to x3
reports ``trunc=-``, ``seeds=0`` and ``dip_below_x3 = 0``: that is the fixed
shape Phase 5 gates on.

WHY THE GATED DIP ROW IS ``dip_below_x3`` AND NOT ``dip``
--------------------------------------------------------
``dip`` is peak-minus-bottom, so it is NON-ZERO on every capture that overshoots
and settles — including a perfectly fixed one (the ``--gate`` clean synthetic
reads 0.6 mm) and including the legitimate overshoot plan Phase 4 step 2 says
the velocity-continuous prelude will produce (the 10.60 rev synthetic reads
20.2 mm, which is the size of the *smallest pre-fix defect*).  A runbook row
reading "PASS: no dip printed" is therefore unachievable, and an operator
following it aborts a working fix.  What actually separates the defect from a
healthy stroke is the SIGN of the excursion relative to the stroke end: pre-fix
the position loop yanks the hand **0.339-1.748 rev = 10.7-55.3 mm BELOW** x3,
whereas a healthy stroke settles onto x3 from above and never goes under (the
four ``--gate`` post-fix synthetics read 0.000-0.001 rev).

``pullback`` is likewise reported but only ABORTS below -5.0 rev/s, and only once
``peak`` has passed: a healthy settle from the coasting peak is genuinely a
negative measured velocity, and its magnitude is set by the peak (-0.31 rev/s at
0.02 rev of overshoot, -1.58 at the runbook's 10.060 rev ceiling, -10.03 at
10.60 rev), so it discriminates nothing on its own.  Pre-fix it read -17.9 to
-42.4 rev/s.

WHY THIS EXISTS
---------------
``plans/active/hand-command-continuity.md``.  Any kind-0/1/2 HAND_TRAJ_CMD
makes ``Teensy_code_platform.ino:539`` clear the whole packed queue and re-prelude from
the LIVE encoder position with ``makeSmoothMove``, whose quintic is seeded
``v = a = 0`` (``Trajectory.h:242-301``; ``current_hand_velocity`` is declared
``extern`` at :47 and never read).  When the hand-catch arm lands inside the
throw stroke's deceleration ramp the queue is replaced by a rest-to-rest ramp
computed from a position the hand is travelling through at ~120 rev/s, so the
hand coasts past the planned stroke end, the position loop yanks it back, and
the operator sees a ~43 mm dip before the catch.  The same clear discards the
throw's own decel ramp, so the ball's departure conditions are set by the
position loop's reaction to a frozen setpoint at whatever instant the arm frame
lands - a throw-repeatability defect, not only a cosmetic one.

Phases 1-2 of that plan assert a THRESHOLD (``release + t_dec + margin``), so
per ``CLAUDE.md``'s empirical-probe rule the recipe is prototyped and confirmed
here before any test encodes it.  Phase 5 re-runs this probe on the post-fix
capture to turn it into a verdict.

Consuming tests (written in Phases 1-2, referenced here so the link stays live):
``tests/motion/test_hand_stroke.py`` (the closed-form model pinned against the
shipped header constants) and ``tests/ros/test_catch_coordinator_node.py`` (the
stroke-busy window).

SHARED MODEL, NOT A COPY
------------------------
Phase 1 moved this probe's ``StrokeModel`` and ``smooth_move_duration_s`` into
``jugglebot/motion/trajectory/hand_stroke.py`` and imports them back (see the
import block below).  Before that, ``Trajectory.h``'s algebra existed three
times host-side: here, in ``sim/hand/trajectory.py``, and about to be a fourth
in ``catch_coordinator_node``.  A pinned duplicate was rejected: this probe is
the Phase-5 VERDICT instrument, so if it and the shipped suppression window
disagreed about ``t_dec``, the bench would score the fix against a different
model than the one that shipped — and a pin only catches the quantities it
happens to assert.  Now there is nothing to pin.

HOW THE INSTANTS ARE FOUND (all detectors are stated so a reviewer can refute)
-----------------------------------------------------------------------------
Nothing here reads the modelled release back out of the observed truncation -
that would make the probe useless as an instrument for the fix.  The two
release estimates are independent of the dip:

  * ``rel_ann``  comes only from ``/throw_announcements``.
  * ``rel_fit``  fits the closed-form throw profile ``x(t)`` to the observed
    ``pos_cmd`` samples of the ASCENT, where the ascent is delimited by the
    COMMANDED velocity (``vel_ff_cmd`` above ``_ASCENT_VEL_REV_S``) and not by
    the truncation.  The fit window is further trimmed to samples at or below
    x2 (the release position), so the decel ramp - the segment the truncation
    destroys - never enters the fit.

  * ``trunc`` is the first sample after the ascent's commanded-velocity peak
    where ``vel_ff_cmd`` collapses below ``_COLLAPSE_VEL_REV_S`` while
    ``pos_cmd`` is still short of x3.

  * ``seeds`` adds every LATER from-rest quintic: a sample whose ``pos_cmd``
    steps by more than ``_REPACK_STEP_REV`` while ``vel_ff_cmd`` is still
    ~zero.  A legitimate quintic cannot move 0.5 rev in one 10 ms telemetry
    period with a commanded velocity near zero, so that shape is only ever a
    fresh ``packedMsgs.clear()`` + ``makeSmoothMove``.  ``Trajectory.h``'s
    quintic starts at ``start_rev = current_hand_position`` exactly, so each
    seed is cross-checked against the live ``pos_meas`` of the preceding few
    samples (``vs_meas`` in the report) — 0.000x rev there is the fingerprint
    of "the queue was re-seeded from the live encoder mid-stroke".

    NOTE for Phase 2: the plan states that an armed stroke has no observable
    until its event time.  That is true of ARMING, but a repack that clobbers
    a LIVE stroke is directly observable right here — which is what makes the
    repack guard testable against a capture even though the arm itself is not.

BIAS IN ``rel_fit`` / ``shift``
------------------------------
``hand_telemetry`` publishes at ~100 Hz a snapshot of the last CAN frame the
Teensy sent, and the stroke frames go out at 500 Hz.  So each sample's
timestamp trails the frame it reports by 0-10 ms.  During the ascent the
command moves at up to 120 rev/s, so that sampling lag biases ``rel_fit``
LATE by roughly half a telemetry period (~5 ms), and ``shift`` is therefore an
OVER-estimate of the true dispatch latency by about that much.  The bias has
the safe sign for sizing a stroke-busy margin (it inflates the window), but do
not quote ``shift`` as a calibrated latency without subtracting it.

CLOCKS
------
A ``toss_trace_*.jsonl`` row carries both ``t`` (the recorder's
``time.perf_counter()``) and ``t_ros``.  Announcement ``throw_time`` fields are
ROS-clock, so they are mapped to the trace clock with the offset carried by
each announcement's OWN row (the offset's session-wide spread is printed, so a
drifting clock is visible rather than silent).  A rosbag has one clock (the
bag's log time) and needs no mapping.

All printed instants are relative to the session's own zero — the FIRST RECORD
of any topic in the trace (for a bag, the summary's ``message_start_time``).
That is the convention the plan's Context table uses; it is not the first
hand-telemetry sample, which in the reference trace is 1.606 s later.  The
absolute value of the zero is printed on every run so a reader can recover
wall time.

``/rosout`` IS DETECTED, NEVER ASSUMED
--------------------------------------
The ``arms`` column (how many ``Arming hand catch`` dispatches a throw got, the
``_MAX_ARM_DISPATCHES`` accounting) needs ``/rosout``.  Whether a source has it
is a property of the RECORDING, not of the format: the three 2026-07-25 evidence
bags were recorded without it, but the § Recording command in
``tests/hardware/session_anomaly_fixes.md`` records it, so BOTH input paths read
it when it is present and report ``?`` only when it is genuinely absent.  A real
zero and an unrecorded channel must not collapse into the same glyph (the
``cone_bag_decode.py`` lesson) — and equally, an available channel must not be
discarded because the format was assumed not to carry it.  The launch log
(``~/.ros/log/<stamp>/launch.log``) is the fallback source when neither has it.

The same applies to a TRACE with no ``/rosout`` rows (the committed gate fixture
predating this; any trace predating ``ROSOUT_NODE_FILTER`` gaining
``catch_coordinator_node``): that prints ``?``.  A source that does carry
``/rosout`` but no arming line keeps a true ``0``.

THE ``--gate`` REFERENCE, AND HOW TO REGENERATE IT
--------------------------------------------------
The recording the plan's Context table was measured from lives under ``temp/``,
which ``.gitignore`` excludes, so ``--gate`` falls back to a committed cut of it
at ``tools/probes/data/hand_stroke_timeline_gate_ref.jsonl`` (the full trace is
preferred whenever present, so the gate keeps exercising the real recording on
this Jetson).  ``--emit-gate-fixture`` is the recipe for that cut - a committed
fixture without a committed recipe is only half a live reference.  To re-baseline
on a new session::

    python tools/probes/hand_stroke_timeline.py \\
        --trace temp/logs/toss_trace_<stamp>.jsonl \\
        --emit-gate-fixture tools/probes/data/hand_stroke_timeline_gate_ref.jsonl
    python tools/probes/hand_stroke_timeline.py --gate     # then update _GATE_EXPECT

USAGE
-----
    python tools/probes/hand_stroke_timeline.py --trace temp/logs/toss_trace_2026-07-25_15-24-25.jsonl
    python tools/probes/hand_stroke_timeline.py --bag ~/Desktop/rosbags/2026-07-25_15-17-48
    python tools/probes/hand_stroke_timeline.py --all
    python tools/probes/hand_stroke_timeline.py --gate      # assert the plan's Context table
    python tools/probes/hand_stroke_timeline.py --trace ... --preview   # per-sample window

``--preview`` prints the raw per-sample window around each throw instead of
opening a matplotlib figure: this is an offline analysis probe on a headless
Jetson, where a figure is unusable and the sample table is what a reviewer needs
to check an instant by hand.  ``tools/README.md``'s ``--preview`` convention is
scoped to the operator-facing hardware harnesses and says so.  Likewise
``--emit-gate-fixture`` writes into ``tools/probes/data/``, which
``tools/probes/README.md`` names as the home for committed reference fixtures —
run outputs still go to ``temp/probes/``.

Strictly offline and read-only: reads ``.jsonl`` and ``.mcap`` files, opens no
socket, starts no node, commands nothing.  Needs NO ROS2 (``mcap_ros2`` decodes
the schemas embedded in the bag).  JSON output goes to
``temp/probes/hand_stroke_timeline/``.

Exit 0 = analysis completed (and, under ``--gate``, the gate passed);
1 = gate failed or an input was unreadable.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import statistics
import sys
import time
from dataclasses import dataclass, field, asdict
from typing import Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..'))
for _p in (os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_REPO_ROOT, 'config', 'generated'), _REPO_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# Generated constants — the single source of truth. Read through the jugglebot
# package copy (byte-identical to config/generated/hardware_config.py; codegen
# writes both from one string) so this file and the shared stroke model imported
# below cannot read two different modules.
from jugglebot import hardware_config as hw  # noqa: E402
# THE stroke model, shared with catch_coordinator_node's C-HAND-1 suppression
# window. Previously a local copy here, which made three host-side copies of
# Trajectory.h's algebra; plan Phase 1 step 2 collapsed them into one. Import,
# not a pinned duplicate: this probe is the Phase-5 VERDICT instrument, so if it
# and the shipped window disagreed about t_dec the bench would score the fix
# against a different model than the one that shipped — and a pin only catches
# the quantities it happens to assert.
from jugglebot.motion.trajectory.hand_stroke import (  # noqa: E402
    LINEAR_GAIN_REV_PER_M,
    TOTAL_STROKE_M,
    TOTAL_STROKE_REV,
    HandStrokeModel as StrokeModel,
    rev_to_mm,
    smooth_move_duration_s,
)

_DEFAULT_TRACE_GLOB = os.path.join(_REPO_ROOT, 'temp', 'logs', 'toss_trace_*.jsonl')
_DEFAULT_BAG_ROOT = os.path.expanduser('~/Desktop/rosbags')
_OUT_DIR = os.path.join(_REPO_ROOT, 'temp', 'probes', 'hand_stroke_timeline')

# The --gate reference.  A COMMITTED extract of the trace the plan's Context
# table was measured from (the first record, so the session zero is preserved,
# the one announcement, and the hand_telemetry rows in the analysis window).
# The full recording lives under temp/, which .gitignore excludes and any
# housekeeping sweep or reimage will remove — so pointing the gate at it would
# make the operator runbook's mandatory instrument self-check unrunnable on a
# fresh clone and the plan's Phase 0 evidence unreproducible by anyone else.
# The full trace is still preferred when present, so the gate keeps exercising
# the real recording on this Jetson.
_GATE_FIXTURE = os.path.join(_HERE, 'data', 'hand_stroke_timeline_gate_ref.jsonl')
_GATE_FULL_TRACE = os.path.join(_REPO_ROOT, 'temp', 'logs',
                                'toss_trace_2026-07-25_15-24-25.jsonl')
_GATE_TRACE = (_GATE_FULL_TRACE if os.path.exists(_GATE_FULL_TRACE)
               else _GATE_FIXTURE)


# ══════════════════════════════════════════════════════════════════════════
#  Closed-form stroke model
# ══════════════════════════════════════════════════════════════════════════
#  Lives in jugglebot/motion/trajectory/hand_stroke.py and is imported at the
#  top of this file — LINEAR_GAIN_REV_PER_M, TOTAL_STROKE_M, TOTAL_STROKE_REV,
#  StrokeModel (= HandStrokeModel), rev_to_mm and smooth_move_duration_s.  The
#  positions it produces are motor revs from the firmware's encoder zero, the
#  throw timeline is relative to BALL RELEASE (makeThrow's shiftTime(-t2)) and
#  the catch timeline to the firmware's kind-1 anchor at the START of the catch
#  velocity hold (shiftTime(-(t5-t4))) — unchanged from when the model was local
#  to this file, and now covered by tests/motion/test_hand_stroke.py.
#
#  smooth_move_duration_s now takes an optional v0 (Phase 4's
#  velocity-continuous prelude), and this probe deliberately calls it with the
#  default v0 = 0: quintic_T_model_s models the OBSERVED PRE-fix from-rest
#  quintic that replaced a truncated stroke, which is by definition the v0 = 0
#  branch.  The v0 == 0 path is the historical expression verbatim, so every
#  number this probe prints — and the committed --gate fixture's expectations —
#  are unchanged to the last bit by Phase 4.  Post-fix, a capture that still
#  shows a from-rest seed is either the arm-gate failing (bench row 2) or
#  makeSmoothMove's documented cannot-fit fallback; both are meant to be visible
#  here rather than silent.


# ══════════════════════════════════════════════════════════════════════════
#  Session loading
# ══════════════════════════════════════════════════════════════════════════

@dataclass
class HandSample:
    t: float
    pos_meas: float
    vel_meas: float
    pos_cmd: float
    vel_ff_cmd: float


@dataclass
class Announcement:
    t: float                 # session clock, publish instant
    t_release: float         # session clock, announced throw_time
    tof_s: float
    v_mps: float
    thrower: str


@dataclass
class Session:
    name: str
    kind: str                       # 'trace' | 'bag'
    samples: list
    announcements: list
    arms: Optional[list]            # [(t, text)] or None if the source has no /rosout
    t0: float
    clock_offset_spread_ms: Optional[float] = None
    notes: list = field(default_factory=list)


def load_trace(path: str) -> Session:
    samples, anns, arms, offsets = [], [], [], []
    # A trace whose recorder never carried /rosout (the committed gate fixture,
    # and any trace predating ROSOUT_NODE_FILTER gaining catch_coordinator_node)
    # must report the arm count as UNAVAILABLE, not as zero — same reason the bag
    # path prints '?'.  Without this, the fixture reads `arms=0` on a toss that
    # really got one dispatch, which is the cone_bag_decode.py failure mode:
    # "channel absent" and "genuinely none" collapsing into one glyph.  A trace
    # that DOES carry /rosout with no arming line keeps a true 0.
    saw_rosout = False
    t_first = None
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except ValueError:
                continue                      # a torn final line; the rest is valid
            topic = r.get('topic')
            if t_first is None and r.get('t') is not None:
                t_first = r['t']
            if r.get('t_ros') is not None:
                offsets.append(r['t_ros'] - r['t'])
            if topic == 'hand_telemetry':
                d = r['d']
                samples.append(HandSample(r['t'], d['pos_meas'], d['vel_meas'],
                                          d['pos_cmd'], d['vel_ff_cmd']))
            elif topic in ('throw_announcements', '/throw_announcements'):
                d = r['d']
                off = ((r['t_ros'] - r['t']) if r.get('t_ros') is not None
                       else 0.0)
                v = d.get('initial_velocity') or [0.0, 0.0, 0.0]
                anns.append(Announcement(
                    t=r['t'],
                    t_release=float(d['throw_time']) - off,
                    tof_s=float(d.get('predicted_tof_sec', 0.0)),
                    v_mps=math.sqrt(sum((c / 1000.0) ** 2 for c in v)),
                    thrower=str(d.get('thrower_name', '?'))))
            elif topic == '/rosout':
                saw_rosout = True
                msg = str(r['d'].get('msg', ''))
                if 'Arming hand catch' in msg:
                    arms.append((r['t'], msg))
    spread = ((max(offsets) - min(offsets)) * 1000.0) if len(offsets) > 1 else None
    if t_first is None:
        t_first = samples[0].t if samples else 0.0
    notes = ([] if saw_rosout else
             ['no /rosout rows in this trace -> arm-dispatch count unavailable'])
    return Session(name=os.path.basename(path), kind='trace', samples=samples,
                   announcements=anns, arms=(arms if saw_rosout else None),
                   t0=t_first, clock_offset_spread_ms=spread, notes=notes)


def _sec(x) -> float:
    """builtin_interfaces/Time or a float -> seconds."""
    try:
        return float(x)
    except (TypeError, ValueError):
        return float(getattr(x, 'sec', 0)) + float(getattr(x, 'nanosec', 0)) * 1e-9


def load_bag(bag_dir: str) -> Session:
    try:
        from mcap_ros2.reader import read_ros2_messages
    except ImportError as exc:                                # pragma: no cover
        raise SystemExit(f"ERROR: mcap_ros2 not importable ({exc}); "
                         "bag input needs it (pip install mcap-ros2-support)")
    files = sorted(glob.glob(os.path.join(bag_dir, '*.mcap')))
    if not files:
        raise SystemExit(f"ERROR: no .mcap files under {bag_dir}")
    samples, anns, arms = [], [], []
    # Whether a bag carries /rosout is a property of the RECORDING, not of bags:
    # the § Recording command in tests/hardware/session_anomaly_fixes.md records
    # it, while the three 2026-07-25 evidence bags happen not to.  So detect it
    # instead of asserting it — otherwise the probe prints "bags carry no /rosout"
    # at an operator whose bag has it, and discards the arm-dispatch count that is
    # sitting right there.
    saw_rosout = False
    t_first = None
    for f in files:                     # bag zero = first message on ANY topic
        try:
            from mcap.reader import make_reader
            with open(f, 'rb') as fh:
                stats = make_reader(fh).get_summary().statistics
            if stats is not None and stats.message_start_time:
                cand = stats.message_start_time / 1e9
                t_first = cand if t_first is None else min(t_first, cand)
        except Exception:               # no summary chunk — fall back below
            pass
    for f in files:
        for m in read_ros2_messages(
                f, topics=['/hand_telemetry', '/throw_announcements', '/rosout']):
            t = m.log_time_ns / 1e9
            msg = m.ros_msg
            if m.channel.topic.endswith('hand_telemetry'):
                samples.append(HandSample(t, msg.pos_meas, msg.vel_meas,
                                          msg.pos_cmd, msg.vel_ff_cmd))
            elif m.channel.topic.endswith('rosout'):
                saw_rosout = True
                text = str(getattr(msg, 'msg', ''))
                if 'Arming hand catch' in text:
                    arms.append((t, text))
            else:
                v = msg.initial_velocity
                anns.append(Announcement(
                    t=t, t_release=_sec(msg.throw_time),
                    tof_s=float(msg.predicted_tof_sec),
                    v_mps=math.sqrt((v.x / 1000.0) ** 2 + (v.y / 1000.0) ** 2
                                    + (v.z / 1000.0) ** 2),
                    thrower=str(msg.thrower_name)))
    samples.sort(key=lambda s: s.t)
    anns.sort(key=lambda a: a.t)
    arms.sort(key=lambda a: a[0])
    if t_first is None:
        t_first = samples[0].t if samples else 0.0
    notes = ([] if saw_rosout else
             ['THIS bag carries no /rosout (the three 2026-07-25 evidence bags '
              'do not; the runbook\'s Recording command does) -> arm-dispatch '
              'count unavailable; use the jsonl trace or the launch log'])
    return Session(name=os.path.basename(bag_dir.rstrip('/')), kind='bag',
                   samples=samples, announcements=anns,
                   arms=(arms if saw_rosout else None), t0=t_first, notes=notes)


# ══════════════════════════════════════════════════════════════════════════
#  Detection thresholds — every one is stated, none is tuned to a result
# ══════════════════════════════════════════════════════════════════════════

_ASCENT_VEL_REV_S = 20.0     # commanded velocity that unambiguously means "stroking"
_REST_VEL_REV_S = 1.0        # commanded velocity that means "at rest"
_COLLAPSE_VEL_REV_S = 10.0   # commanded velocity below which the stroke is gone
_X3_SHORT_REV = 0.05         # how far short of x3 counts as "truncated mid-stroke"
_SEED_VEL_REV_S = 2.0        # a from-rest quintic's opening commanded velocity
_REPACK_STEP_REV = 0.5       # pos_cmd step that no legitimate near-rest quintic
                             # can make inside one 10 ms telemetry period
_SEED_LOOKBACK = 3           # samples searched back for the live pos_meas a seed
                             # was taken from (covers the ~20 ms command path)
_PLATEAU_TOL_REV = 0.002     # pos_cmd settled onto its target
_RECOVERY_TOL_REV = 0.005    # ~0.16 mm — well above pos quantisation, far below
                             # anything physically meaningful
_CATCH_DESC_VEL_REV_S = -1.0  # commanded velocity turning negative = a DOWNWARD
                              # command of some kind (not yet the catch descent)
_CATCH_DESC_ABOVE_X5_REV = 0.33  # how far into the CATCH REGION the COMMANDED
                              # position must actually travel before a downward
                              # command counts as the ARMED CATCH descent: it must
                              # reach `x5 + this`.  A negative commanded velocity
                              # alone is not enough: a `makeSmoothMove` BRAKE —
                              # which plan Phase 4 charters for the
                              # at-target-but-moving case — is also downward.
                              #
                              # SIZED AGAINST THE SHIPPED CLAMP, not against the
                              # pre-fix brake sample.  This threshold was
                              # originally 0.5 rev below x3, justified by the
                              # brakes visible in the seven observed tosses
                              # (0.206-0.365 rev, all of them the settle-from-
                              # above case).  That premise died with Phase 4: an
                              # honoured brake seeded from a DOWNWARD velocity
                              # dives |v0|*T*16/81 = 0.00778*v0^2 rev below x3,
                              # crossing 0.5 rev at just 8.05 rev/s, and the
                              # clamp honours it out to 3.126 rev (v0 = 20.04
                              # rev/s, where Trajectory.h's duration cap takes
                              # over).  A brake read as the descent collapses the
                              # peak/pullback/dip window onto itself and makes the
                              # end-stop `peak` row — the one guarding the 10.8
                              # rev hard stop, bench row H4.5 — UNDER-report.
                              #
                              # x5 + 0.33 rev = 6.457 rev = x3 - 3.502 rev, so it
                              # sits 0.38 rev clear of the deepest brake the
                              # firmware can honour and 0.33 rev clear of x5,
                              # which every armed descent commands.  Biased deep
                              # on purpose: too shallow fails SILENTLY on the row
                              # that guards the end stop, too deep leaves
                              # `catch_desc` unset and floods the report with
                              # spurious truncations — a loud failure is the safer
                              # side of this trade.
                              # Without this, a brake is mistaken for the
                              # descent, and because `catch_desc` bounds BOTH the
                              # truncation scan and the peak/pullback/dip window,
                              # those windows shrink to the brake.  Mutation-
                              # measured on the `braking-prelude` gate case: the
                              # dip/peak window collapsed from ~600 ms to ~130 ms
                              # and the end-stop `peak` row read 10.1298 rev
                              # against a real 10.1588 — i.e. the one row guarding
                              # the 10.8 rev hard stop under-reports the excursion.
_DIP_BELOW_X3_BAND_REV = 0.10    # `dip_below_x3_rev` at or under this reads as "no
                              # dip": the hand settled onto the stroke end rather
                              # than being yanked below it.  Same 0.10 rev the
                              # plan's Phase 5 allows for overshoot ABOVE x3, so
                              # the criterion is symmetric about the stroke end
                              # rather than being a free parameter.
                              # Measured separation: the four synthetic post-fix
                              # shapes read 0.000-0.001 rev (100x under the band),
                              # while the seven pre-fix tosses read 0.339-1.748
                              # rev = 10.7-55.3 mm.  So 3.4x margin on the
                              # TIGHTEST defect (ball 34) and ~100x on the healthy
                              # side — the tight side is the one to watch if a
                              # future capture lands near 0.3 rev.
_PRE_WINDOW_S = 0.60
_POST_WINDOW_S = 0.60
_TRUNC_SCAN_MARGIN_S = 0.050  # how far past the modelled stroke end the truncation
                              # scan may look: ~5 telemetry NEW-VALUE periods, so a
                              # late-REPORTED mid-stroke freeze is still caught,
                              # while the armed catch descent (>= 363 ms later even
                              # at FLIGHT_TIME_MIN_S, per the plan's window table)
                              # is decisively out of range
_DIP_WINDOW_S = 0.60          # how long after the anchor the dip/peak search runs
_FIXTURE_MARGIN_S = 0.40      # slack kept either side of the analysis window when
                              # cutting the committed gate fixture, so widening
                              # _PRE/_POST_WINDOW_S later does not silently
                              # truncate the fixture's evidence


@dataclass
class ThrowTimeline:
    session: str
    thrower: str
    v_cmd_mps: float
    tof_s: float
    arms: Optional[int]
    # release estimates
    rel_ann: Optional[float] = None
    rel_fit: Optional[float] = None
    shift_ms: Optional[float] = None
    fit_rms_rev: Optional[float] = None
    # model
    t_acc: float = 0.0
    t_vel: float = 0.0
    t_dec: float = 0.0
    x2_rev: float = 0.0
    x3_rev: float = 0.0
    # observed
    stroke_start: Optional[float] = None
    x2_cross: Optional[float] = None
    x2_cross_vel_rev_s: Optional[float] = None
    x2_cross_vel_mps: Optional[float] = None
    trunc: Optional[float] = None
    trunc_pos_rev: Optional[float] = None
    trunc_cmd_meas_gap_rev: Optional[float] = None
    seeds: list = field(default_factory=list)   # [{t, pos_rev, vs_meas_rev}]
    n_seeds: int = 0
    quintic_target_rev: Optional[float] = None
    quintic_T_model_s: Optional[float] = None
    quintic_end_obs: Optional[float] = None
    peak: Optional[float] = None
    peak_pos_rev: Optional[float] = None
    peak_pos_mm: Optional[float] = None
    peak_over_x3_rev: Optional[float] = None
    headroom_to_limit_rev: Optional[float] = None
    pullback: Optional[float] = None
    pullback_vel_rev_s: Optional[float] = None
    pullback_vel_mps: Optional[float] = None
    dip_bottom: Optional[float] = None
    dip_bottom_pos_rev: Optional[float] = None
    dip_depth_rev: Optional[float] = None
    dip_depth_mm: Optional[float] = None
    dip_depth_pct_stroke: Optional[float] = None
    # How far BELOW the stroke end x3 the hand was pulled.  This — not
    # `dip_depth_*`, which is peak-minus-bottom and so is non-zero on any capture
    # that overshoots and settles — is the quantity that separates the defect
    # from a healthy stroke, and it is what the operator runbook gates on.
    dip_below_x3_rev: Optional[float] = None
    dip_below_x3_mm: Optional[float] = None
    recovery: Optional[float] = None
    # Any downward command after the release, catch or not.  Reported separately
    # from `catch_desc` so a braking prelude is VISIBLE rather than shadowing the
    # descent it is not.
    first_neg_cmd_vel: Optional[float] = None
    first_neg_cmd_vel_rev_s: Optional[float] = None
    catch_desc: Optional[float] = None
    catch_desc_pos_rev: Optional[float] = None
    status: str = 'ok'


def _win(samples, lo, hi):
    return [s for s in samples if lo <= s.t <= hi]


def _fit_release(model: StrokeModel, ascent) -> tuple:
    """Least-squares time-shift of the closed-form profile onto ``pos_cmd``.

    Returns ``(t_release_fit, rms_rev)``.  Only ascent samples at or below x2
    are used, so the decel ramp — the segment a truncation destroys — cannot
    influence the estimate.  Coarse-to-fine scan (5 ms then 0.1 ms) over a
    +-150 ms bracket around the samples' own span; a 1-D convex-ish objective
    on ~10 samples does not warrant an optimiser dependency.
    """
    pts = [(s.t, s.pos_cmd) for s in ascent if s.pos_cmd <= model.x2_rev]
    if len(pts) < 4:
        return None, None

    def cost(t_rel):
        return sum((p - model.pos_rev(t - t_rel)) ** 2 for t, p in pts)

    # seed: the stroke ought to start at the first ascent sample
    best = pts[0][0] - model.stroke_start_rel
    for step, half in ((0.005, 0.15), (0.0001, 0.01)):
        lo = best - half
        n = int(round(2.0 * half / step)) + 1
        best = min((lo + i * step for i in range(n)), key=cost)
    return best, math.sqrt(cost(best) / len(pts))


def analyse_throw(session: Session, ann: Announcement) -> ThrowTimeline:
    model = StrokeModel(ann.v_mps)
    tl = ThrowTimeline(session=session.name, thrower=ann.thrower,
                       v_cmd_mps=ann.v_mps, tof_s=ann.tof_s, arms=None,
                       rel_ann=ann.t_release,
                       t_acc=model.t_acc, t_vel=model.t_vel, t_dec=model.t_dec,
                       x2_rev=model.x2_rev, x3_rev=model.x3_rev)
    if session.arms is not None:
        lo, hi = ann.t, ann.t_release + max(ann.tof_s, 0.2) + _POST_WINDOW_S
        tl.arms = sum(1 for t, _ in session.arms if lo <= t <= hi)

    win = _win(session.samples, ann.t_release - _PRE_WINDOW_S,
               ann.t_release + max(ann.tof_s, 0.2) + _POST_WINDOW_S)
    if len(win) < 10:
        tl.status = 'no-telemetry'
        return tl

    # ── stroke start: commanded velocity leaves rest ──────────────────────
    i_asc = next((i for i, s in enumerate(win)
                  if s.vel_ff_cmd > _ASCENT_VEL_REV_S), None)
    if i_asc is None:
        tl.status = ('no-throw-stroke' if ann.thrower != 'jugglebot'
                     else 'no-throw-stroke(!)')
        _analyse_catch_descent(tl, win, model, from_t=ann.t_release)
        return tl
    i_start = i_asc
    while i_start > 0 and win[i_start - 1].vel_ff_cmd > _REST_VEL_REV_S:
        i_start -= 1
    tl.stroke_start = win[i_start].t

    # ── ascent samples (commanded), used for the model fit ────────────────
    i_peak = max(range(i_asc, len(win)), key=lambda i: win[i].vel_ff_cmd)
    ascent = [s for s in win[i_start:i_peak + 1] if s.vel_ff_cmd > _REST_VEL_REV_S]
    tl.rel_fit, tl.fit_rms_rev = _fit_release(model, ascent)
    if tl.rel_fit is not None:
        tl.shift_ms = (tl.rel_fit - ann.t_release) * 1000.0

    # ── observed crossing of the modelled release position ────────────────
    cross = next((s for s in win[i_start:] if s.pos_meas >= model.x2_rev), None)
    if cross is not None:
        tl.x2_cross = cross.t
        tl.x2_cross_vel_rev_s = cross.vel_meas
        tl.x2_cross_vel_mps = cross.vel_meas / LINEAR_GAIN_REV_PER_M

    # ── the armed catch descent is located FIRST: it bounds every later
    #    search window, the truncation scan included ─────────────────────────
    t_release_obs = tl.stroke_start + model.t_acc + model.t_vel
    _analyse_catch_descent(tl, win, model, from_t=t_release_obs)

    # ── truncation: commanded velocity collapses short of x3, WITHIN the
    #    stroke.  The scan MUST be time-bounded.  The armed catch descent also
    #    has `vel_ff_cmd` below the collapse threshold AND `pos_cmd` below x3,
    #    so an unbounded scan reports the DESCENT as a truncation on any
    #    capture where the stroke was never truncated — i.e. exactly on a
    #    POST-FIX capture, the one shape this probe exists to score, where it
    #    would emit a spurious trunc/seed/dip/pullback and measure `peak` at
    #    the descent onset instead of the coasting peak (a false PASS on the
    #    row that guards the 10.8 rev end stop).
    #    Verified on temp/logs/toss_trace_2026-07-25_15-24-25.jsonl: the
    #    predicate is TRUE from the descent's second sample onward (pos_cmd
    #    9.7253 rev, vel_ff_cmd -17.31 rev/s); the pre-fix data hides it only
    #    because next() short-circuits on the genuine truncation 663 ms
    #    earlier.  `--gate`'s synthetic fixed-shape case pins this.
    scan_end = t_release_obs + model.t_dec + _TRUNC_SCAN_MARGIN_S
    if tl.catch_desc is not None:
        scan_end = min(scan_end, tl.catch_desc)
    i_tr = next((i for i in range(i_peak, len(win))
                 if win[i].t <= scan_end
                 and win[i].vel_ff_cmd < _COLLAPSE_VEL_REV_S
                 and win[i].pos_cmd < model.x3_rev - _X3_SHORT_REV), None)

    if i_tr is None:
        tl.status = 'not-truncated'
    else:
        tr = win[i_tr]
        tl.trunc = tr.t
        tl.trunc_pos_rev = tr.pos_cmd
        tl.trunc_cmd_meas_gap_rev = abs(tr.pos_cmd - tr.pos_meas)

        # ── every from-rest quintic seed: the truncation itself, then each
        #    later pos_cmd step taken with a near-zero commanded velocity ────
        def _seed(i):
            back = win[max(0, i - _SEED_LOOKBACK):i + 1]
            return {'t': win[i].t, 'pos_rev': win[i].pos_cmd,
                    'vs_meas_rev': min(abs(win[i].pos_cmd - b.pos_meas)
                                       for b in back)}

        tl.seeds = [_seed(i_tr)]
        for i in range(i_tr + 1, len(win)):
            # abs() on the velocity, not a signed test: the armed catch
            # stroke's descent frames step pos_cmd by up to 0.6 rev per sample,
            # and they would read as spurious seeds under a signed test.
            if (abs(win[i].vel_ff_cmd) < _SEED_VEL_REV_S
                    and abs(win[i].pos_cmd - win[i - 1].pos_cmd)
                    > _REPACK_STEP_REV):
                tl.seeds.append(_seed(i))
        tl.n_seeds = len(tl.seeds)

        # ── the replacement quintic: target = the pos_cmd plateau after it ──
        tail = win[i_tr:]
        tl.quintic_target_rev = max(s.pos_cmd for s in tail)
        tl.quintic_T_model_s = smooth_move_duration_s(
            tl.quintic_target_rev - tl.trunc_pos_rev)
        tl.quintic_end_obs = next(
            (s.t for s in tail
             if s.pos_cmd >= tl.quintic_target_rev - _PLATEAU_TOL_REV), None)

    # ── coasting peak / pullback / dip, all on the MEASURED position ────────
    # Measured UNCONDITIONALLY, truncated or not.  `peak` /
    # `headroom_to_limit_rev` are the end-stop SAFETY rows: a post-fix capture
    # is expected to overshoot legitimately (plan Phase 4 step 2 — a
    # velocity-continuous prelude does not stop at the target), so returning
    # early on `not-truncated` would print `-` for the peak on exactly the
    # captures whose excursion has to be watched.  Pre-fix behaviour is
    # unchanged: the window still opens at the release and pos_meas is
    # ascending from there to the truncation, so the max is the same sample.
    #
    # The window must END at the catch descent, not at a fixed 0.6 s: at
    # FLIGHT_TIME_MIN_S = 0.55 the descent begins ~340 ms after the truncation,
    # so a fixed window would hand the descent's -60 rev/s to `pullback` and the
    # descent's low point to `dip_bottom`.
    anchor = tl.trunc if tl.trunc is not None else t_release_obs
    dip_end = anchor + _DIP_WINDOW_S
    if tl.catch_desc is not None:
        dip_end = min(dip_end, tl.catch_desc)
    dip_win = [s for s in win if t_release_obs <= s.t <= dip_end]
    if not dip_win:
        return tl

    pk = max(dip_win, key=lambda s: s.pos_meas)
    tl.peak, tl.peak_pos_rev = pk.t, pk.pos_meas
    tl.peak_pos_mm = rev_to_mm(pk.pos_meas)
    tl.peak_over_x3_rev = pk.pos_meas - model.x3_rev
    tl.headroom_to_limit_rev = hw.GEOM_HAND_MOTOR_HARD_STOP_REVS - pk.pos_meas

    after_pk = [s for s in dip_win if s.t >= pk.t]
    pb = min(after_pk, key=lambda s: s.vel_meas)
    tl.pullback, tl.pullback_vel_rev_s = pb.t, pb.vel_meas
    tl.pullback_vel_mps = pb.vel_meas / LINEAR_GAIN_REV_PER_M

    bot = min(after_pk, key=lambda s: s.pos_meas)
    tl.dip_bottom, tl.dip_bottom_pos_rev = bot.t, bot.pos_meas
    tl.dip_depth_rev = pk.pos_meas - bot.pos_meas
    tl.dip_depth_mm = rev_to_mm(tl.dip_depth_rev)
    tl.dip_depth_pct_stroke = 100.0 * tl.dip_depth_rev / TOTAL_STROKE_REV
    # `dip_depth_*` is peak-minus-bottom, so it is NON-ZERO on any capture that
    # overshoots and settles — including a perfectly fixed one, and including the
    # legitimate overshoot plan Phase 4 step 2 says the velocity-continuous
    # prelude will produce.  It therefore cannot be the criterion.  What separates
    # the defect from a healthy stroke is whether the hand ended up BELOW the
    # stroke end: pre-fix the position loop yanks it 0.339-1.748 rev under x3
    # (10.7-55.3 mm over the seven 2026-07-25 tosses), whereas a healthy stroke
    # settles ONTO x3 from above and never goes under it.
    tl.dip_below_x3_rev = max(0.0, model.x3_rev - bot.pos_meas)
    tl.dip_below_x3_mm = rev_to_mm(tl.dip_below_x3_rev)

    # `recovery` is defined against the replacement quintic's target, so it is
    # meaningful only when there WAS a truncation.
    if tl.quintic_target_rev is not None:
        tl.recovery = next(
            (s.t for s in win if s.t > bot.t
             and s.pos_meas >= tl.quintic_target_rev - _RECOVERY_TOL_REV), None)
    return tl


def _analyse_catch_descent(tl: ThrowTimeline, win, model: StrokeModel,
                           from_t: float) -> None:
    """Onset of the ARMED CATCH descent, and the first downward command of any kind.

    Two separate readings, deliberately:

    ``first_neg_cmd_vel``  the first sample after ``from_t`` whose COMMANDED
                           velocity is below ``_CATCH_DESC_VEL_REV_S``.  This is
                           what the probe used to call the catch descent.
    ``catch_desc``         the onset of the descent that actually TRAVELS to the
                           catch — the commanded position must reach
                           ``x5 + _CATCH_DESC_ABOVE_X5_REV``, i.e. into the catch
                           region itself — found by locating an unambiguous point
                           inside the descent and then walking back to the start
                           of its negative-velocity run.

    The distinction is load-bearing, not pedantic.  ``catch_desc`` bounds both the
    truncation scan and the peak/pullback/dip window, so identifying it from a
    lone negative-velocity sample lets any downward move shrink those windows.
    The downward move that matters is a ``makeSmoothMove`` BRAKE from the coasting
    peak back to x3 — precisely what plan Phase 4 step 3 charters for the
    at-target-but-moving case, i.e. a shape that only exists on the POST-FIX
    captures this probe has to score.  Measured on a synthetic post-fix capture
    carrying such a brake at release+105 ms: the old predicate put ``catch_desc``
    at the brake (release+130 ms) instead of the true descent 567 ms later, so the
    peak/pullback/dip window collapsed from ~600 ms to ~130 ms and the end-stop
    ``peak`` row read 10.1298 rev against a real 10.1588.  ``--gate``'s
    ``braking-prelude`` case pins this.
    """
    first = next((s for s in win
                  if s.t > from_t and s.vel_ff_cmd < _CATCH_DESC_VEL_REV_S), None)
    if first is not None:
        tl.first_neg_cmd_vel = first.t
        tl.first_neg_cmd_vel_rev_s = first.vel_ff_cmd

    floor = model.x5_rev + _CATCH_DESC_ABOVE_X5_REV
    i = next((k for k, s in enumerate(win)
              if s.t > from_t and s.pos_cmd < floor
              and s.vel_ff_cmd < _CATCH_DESC_VEL_REV_S), None)
    if i is None:
        return
    while (i > 0 and win[i - 1].t > from_t
           and win[i - 1].vel_ff_cmd < _CATCH_DESC_VEL_REV_S):
        i -= 1
    tl.catch_desc = win[i].t
    tl.catch_desc_pos_rev = win[i].pos_meas


# ══════════════════════════════════════════════════════════════════════════
#  Reporting
# ══════════════════════════════════════════════════════════════════════════

def _f(v, fmt='%.4f', dash='-'):
    return dash if v is None else (fmt % v)


def print_session(session: Session, timelines, preview: bool = False) -> None:
    t0 = session.t0
    print('=' * 78)
    print('%s  [%s]  %d hand samples  %d announcement(s)'
          % (session.name, session.kind, len(session.samples),
             len(session.announcements)))
    if session.samples:
        dts = [b.t - a.t for a, b in zip(session.samples, session.samples[1:])]
        dts = [d for d in dts if d > 0]
        fresh = [b.t - a.t for a, b in zip(session.samples, session.samples[1:])
                 if b.pos_meas != a.pos_meas]
        print('   hand telemetry: median dt %.1f ms   median NEW-VALUE dt %.1f ms'
              % (1000.0 * statistics.median(dts) if dts else float('nan'),
                 1000.0 * statistics.median(fresh) if fresh else float('nan')))
    if session.clock_offset_spread_ms is not None:
        print('   perf<->ros offset spread over the session: %.1f ms'
              % session.clock_offset_spread_ms)
    print("   t=0 for every instant below: the session's first record, "
          "any topic (t=%.6f)" % t0)
    for n in session.notes:
        print('   NOTE: %s' % n)
    # x2 / x3 are velocity-INDEPENDENT (x3 = accelSt + velHold = totalStroke
    # algebraically), so one representative model prints them for the session.
    _m = StrokeModel(hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS)
    print('   stroke model: x2=%.4f rev  x3=%.4f rev  stroke limit=%.2f rev'
          % (_m.x2_rev, _m.x3_rev, hw.GEOM_HAND_MOTOR_HARD_STOP_REVS))

    for k, tl in enumerate(timelines, 1):
        print('   ' + '-' * 74)
        print('   throw %d/%d  thrower=%s  v_cmd=%.3f m/s  tof=%.3f s  arms=%s  [%s]'
              % (k, len(timelines), tl.thrower, tl.v_cmd_mps, tl.tof_s,
                 '?' if tl.arms is None else tl.arms, tl.status))
        print('      model   t_acc=%.4f  t_vel=%.4f  t_dec=%.4f s   '
              'stroke spans release%+.4f .. %+.4f s'
              % (tl.t_acc, tl.t_vel, tl.t_dec,
                 -(tl.t_acc + tl.t_vel), tl.t_dec))
        print('      release  ann=%s  fit=%s  shift=%s ms  (fit rms %s rev)'
              % (_f(None if tl.rel_ann is None else tl.rel_ann - t0),
                 _f(None if tl.rel_fit is None else tl.rel_fit - t0),
                 _f(tl.shift_ms, '%+.1f'), _f(tl.fit_rms_rev, '%.4f')))
        rows = [
            ('stroke_start   pos_cmd leaves rest', tl.stroke_start, ''),
            ('x2_cross       pos_meas passes %.3f rev' % tl.x2_rev, tl.x2_cross,
             'at %s rev/s = %s m/s' % (_f(tl.x2_cross_vel_rev_s, '%.1f'),
                                       _f(tl.x2_cross_vel_mps, '%.2f'))),
            ('trunc          pos_cmd freezes', tl.trunc,
             'at %s rev  (|cmd-meas| this sample = %s rev)'
             % (_f(tl.trunc_pos_rev, '%.4f'),
                _f(tl.trunc_cmd_meas_gap_rev, '%.4f'))),
            ('seeds          from-rest quintics', tl.trunc,
             '%d:  %s' % (tl.n_seeds, '  '.join(
                 '%.4f rev @%+.3f s (vs_meas %.4f)'
                 % (s['pos_rev'], s['t'] - (tl.trunc or 0.0), s['vs_meas_rev'])
                 for s in tl.seeds)) if tl.seeds else '-'),
            ('quintic        replacement move', tl.trunc,
             'target %s rev  T_model %s s  ends %s'
             % (_f(tl.quintic_target_rev, '%.4f'),
                _f(tl.quintic_T_model_s, '%.4f'),
                _f(None if tl.quintic_end_obs is None
                   else tl.quintic_end_obs - t0))),
            ('peak           coasting peak', tl.peak,
             '%s rev = %s mm  (%s rev past x3; %s rev headroom to limit)'
             % (_f(tl.peak_pos_rev, '%.4f'), _f(tl.peak_pos_mm, '%.1f'),
                _f(tl.peak_over_x3_rev, '%+.3f'),
                _f(tl.headroom_to_limit_rev, '%.3f'))),
            ('pullback       pos-loop reacts', tl.pullback,
             '%s rev/s = %s m/s' % (_f(tl.pullback_vel_rev_s, '%.1f'),
                                    _f(tl.pullback_vel_mps, '%.2f'))),
            ('dip_bottom     lowest point', tl.dip_bottom,
             '%s rev   depth %s rev = %s mm = %s %% of stroke'
             % (_f(tl.dip_bottom_pos_rev, '%.4f'), _f(tl.dip_depth_rev, '%.3f'),
                _f(tl.dip_depth_mm, '%.1f'),
                _f(tl.dip_depth_pct_stroke, '%.1f'))),
            ('dip_below_x3   *** THE GATED ROW ***', tl.dip_bottom,
             '%s rev = %s mm below x3   [%s, band %.2f rev]'
             % (_f(tl.dip_below_x3_rev, '%.3f'), _f(tl.dip_below_x3_mm, '%.1f'),
                ('-' if tl.dip_below_x3_rev is None else
                 ('OK' if tl.dip_below_x3_rev <= _DIP_BELOW_X3_BAND_REV
                  else 'OVER')),
                _DIP_BELOW_X3_BAND_REV)),
            ('recovery       back at target', tl.recovery, ''),
            ('first_neg_cmd  any downward command', tl.first_neg_cmd_vel,
             'at %s rev/s%s' % (_f(tl.first_neg_cmd_vel_rev_s, '%.1f'),
                                '' if (tl.catch_desc is None
                                       or tl.first_neg_cmd_vel is None
                                       or abs(tl.first_neg_cmd_vel
                                              - tl.catch_desc) < 1e-9)
                                else '  <-- NOT the catch descent (a brake?)')),
            ('catch_desc     armed catch descends', tl.catch_desc,
             'pos_meas %s rev' % _f(tl.catch_desc_pos_rev, '%.4f')),
        ]
        for label, t, extra in rows:
            print('      %-8s %-38s %s'
                  % (_f(None if t is None else t - t0), label, extra))

        if preview:
            _print_preview(session, tl)

    if timelines:
        shifts = [tl.shift_ms for tl in timelines if tl.shift_ms is not None]
        if shifts:
            print('   ' + '-' * 74)
            print('   dispatch shift (fit - announcement) over %d throw(s): '
                  'min %+.1f  median %+.1f  max %+.1f ms'
                  % (len(shifts), min(shifts), statistics.median(shifts),
                     max(shifts)))


def _print_preview(session: Session, tl: ThrowTimeline) -> None:
    lo = (tl.stroke_start or tl.rel_ann) - 0.10
    hi = (tl.catch_desc or tl.rel_ann) + 0.10
    print('      --- per-sample window ---')
    print('      %-9s %-10s %-10s %-10s %-10s'
          % ('rel', 'pos_meas', 'vel_meas', 'pos_cmd', 'vel_ff_cmd'))
    for s in session.samples:
        if lo <= s.t <= hi:
            print('      %9.4f %10.4f %10.3f %10.4f %10.3f'
                  % (s.t - session.t0, s.pos_meas, s.vel_meas, s.pos_cmd,
                     s.vel_ff_cmd))


# ══════════════════════════════════════════════════════════════════════════
#  --gate : reproduce the plan's Context table
# ══════════════════════════════════════════════════════════════════════════

# plans/active/hand-command-continuity.md § "What actually happens", measured
# from toss_trace_2026-07-25_15-24-25.jsonl.  Instants are relative to the
# session's own zero (first record, any topic).
#
# Each tolerance is  max(instrument resolution, half a unit in the plan's last
# quoted digit):
#   instants     0.020 s   = two hand-telemetry NEW-VALUE periods (measured
#                            median 10.0-10.2 ms across all six 2026-07-25
#                            sources; the probe prints the median it measured
#                            on every run, so a slower stream is visible)
#   positions    0.010 rev = 0.32 mm, ~3 x the pos_cmd print precision
#   velocities   0.5 rev/s
#   dip %        0.5 pp    — the plan quotes "14 %" to two significant figures
#   shift        5.0 ms    — the ~5 ms sampling-lag bias documented above is
#                            INSIDE this tolerance, so the row pins the fit's
#                            behaviour without pretending to a calibration
#
# Model rows (x2/x5/n_seeds/shift) are here because the instant rows do NOT pin
# them: a 5 % error in x2 moves x2_cross by only ~2.4 ms, well inside the 20 ms
# instant tolerance, and a broken release fit or seed detector moves no instant
# at all.  Both quantities are load-bearing downstream — `shift` sizes Phase 1's
# stroke-busy margin and `n_seeds` is a runbook ABORT row — so a mutation that
# breaks either has to fail here rather than pass the 20 instant/position rows.
_GATE_EXPECT = [
    # (field,                    expected, tol,   units)
    ('stroke_start',             20.836,   0.020, 's'),
    ('x2_cross',                 20.927,   0.020, 's'),
    ('x2_cross_vel_rev_s',      119.6,     0.5,   'rev/s'),
    ('x2_rev',                    5.9138,  0.0010, 'rev'),
    ('x3_rev',                    9.9594,  0.0010, 'rev'),
    ('shift_ms',                 23.4,     5.0,   'ms'),
    ('n_seeds',                   1,       0,     'seeds'),
    ('trunc',                    20.942,   0.020, 's'),
    ('trunc_pos_rev',             7.700,   0.010, 'rev'),
    ('quintic_target_rev',       10.05,    0.010, 'rev'),
    ('quintic_T_model_s',         0.369,   0.005, 's'),
    ('peak',                     20.992,   0.020, 's'),
    ('peak_pos_rev',             10.174,   0.010, 'rev'),
    ('peak_pos_mm',             321.8,     0.5,   'mm'),
    ('headroom_to_limit_rev',     0.93,    0.010, 'rev'),
    ('pullback',                 21.025,   0.020, 's'),
    ('pullback_vel_rev_s',      -31.3,     0.5,   'rev/s'),
    ('dip_bottom',               21.077,   0.020, 's'),
    ('dip_bottom_pos_rev',        8.814,   0.010, 'rev'),
    ('dip_depth_rev',             1.36,    0.010, 'rev'),
    ('dip_depth_mm',             43.0,     0.5,   'mm'),
    ('dip_depth_pct_stroke',     14.0,     0.5,   '%'),
    # the GATED row: how far under the stroke end the position loop pulled the
    # hand.  x3 9.9594 - dip_bottom 8.814.  Pinned here so a change to the
    # measure cannot pass on the pre-fix capture either.
    ('dip_below_x3_rev',          1.145,   0.010, 'rev'),
    ('recovery',                 21.294,   0.020, 's'),
    ('catch_desc',               21.605,   0.020, 's'),
]
_GATE_INSTANT_FIELDS = {'stroke_start', 'x2_cross', 'trunc', 'peak', 'pullback',
                        'dip_bottom', 'recovery', 'catch_desc'}


_SYNTH_BRAKE_T_S = 0.14      # duration of the synthetic braking prelude


def _synth_fixed_session(peak_rev: float, tof_s: float = 0.80,
                         v_mps: float = 3.9308, hz: float = 100.0,
                         lag_s: float = 0.008,
                         brake_at_rel: Optional[float] = None,
                         brake_dive_rev: Optional[float] = None) -> tuple:
    """An in-memory capture of the POST-FIX shape, built from ``StrokeModel``.

    ``pos_cmd`` follows the modelled stroke all the way to x3, holds there, then
    the armed kind-1 catch descends — no queue clear, no repack, no truncation.
    ``pos_meas`` tracks it with a fixed lag, and coasts to ``peak_rev`` after the
    stroke ends before settling back to x3.

    ``brake_at_rel`` (seconds after release) additionally injects a
    ``makeSmoothMove`` BRAKE: the command leaves x3, arcs to where the coasting
    hand would stop, and returns to x3.  That is the shape plan Phase 4 step 3
    charters for the at-target-but-moving case, and the one thing that makes a
    POST-fix capture carry a legitimate negative COMMANDED velocity long before
    the catch.  Nothing is truncated in this case either — the stroke reached x3
    first — so every gated row must still read clean.

    Returns ``(session, t_catch_desc_true)`` so the gate can assert ``catch_desc``
    against the descent onset it built, rather than against whatever the probe
    happens to report.

    Generated rather than recorded, deliberately: every one of the nine
    analysable tosses in the evidence base truncates, so NO real capture
    exercises the fixed branch — which is precisely how an unbounded truncation
    scan survived Phase 0's first gate on the Context-table rows alone.  A
    committed synthetic *file*
    would instead go stale the moment the shipped ``TEENSY_TRAJ_*`` constants
    moved; deriving the shape from the same ``StrokeModel`` the probe itself
    uses keeps the assertion honest against whatever the header says today.
    """
    m = StrokeModel(v_mps)
    t_rel = 2.0
    t_desc = t_rel + tof_s - m.t_acc_catch
    x3 = m.x3_rev
    t_brake = None if brake_at_rel is None else t_rel + brake_at_rel
    # A dive of D rev is the excursion of a v0 = sqrt(D / 0.00778) rev/s brake,
    # whose duration is the firmware's own |v0|*H2/A_max — so the synthetic
    # carries the real time cost of the depth rather than a made-up one.
    brake_T = _SYNTH_BRAKE_T_S
    if brake_dive_rev is not None:
        brake_T = (math.sqrt(brake_dive_rev
                             / (hw.TEENSY_TRAJ_QUINTIC_H_MAX * hw.TEENSY_TRAJ_QUINTIC_H2_MAX
                                / hw.TEENSY_TRAJ_MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2))
                   * hw.TEENSY_TRAJ_QUINTIC_H2_MAX
                   / hw.TEENSY_TRAJ_MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2)

    def coast(t):
        u = (t - (t_rel + m.t_dec)) / 0.20
        if 0.0 <= u <= 1.0:
            return x3 + (peak_rev - x3) * math.sin(math.pi * u)
        return x3

    def pos_cmd_at(t):
        rel = t - t_rel
        if rel <= m.t_dec:
            return m.pos_rev(rel)
        if t_brake is not None and t_brake <= t < t_brake + brake_T:
            s = (t - t_brake) / brake_T
            if brake_dive_rev is not None:
                # A DOWNWARD brake, in the firmware's own shape: with the target
                # at x3 and the hand moving down through it, makeSmoothMove emits
                # x3 + u*h(tau) with u = v0*T < 0.  h(0) = h(1) = 0, so it leaves
                # x3, dives |u|*16/81 and lands back on x3 at rest.
                u = -brake_dive_rev / hw.TEENSY_TRAJ_QUINTIC_H_MAX
                return x3 + u * (s * (1.0 - s) ** 3 * (3.0 * s + 1.0))
            # out-and-back from the live position's stopping point down to x3
            top = coast(t_brake) + 0.05
            return x3 + (top - x3) * (1.0 - s) ** 2 * (1.0 + 2.0 * s)
        if t < t_desc:
            return x3
        tau = t - t_desc
        return max(m.x5_rev,
                   (m.x3_m + 0.5 * m.catchA * tau * tau) * LINEAR_GAIN_REV_PER_M)

    def pos_meas_at(t):
        base = pos_cmd_at(t - lag_s)
        # coasting excursion after the stroke ends, then settle back to x3
        u = (t - (t_rel + m.t_dec)) / 0.20
        if 0.0 <= u <= 1.0 and t < t_desc:
            base = coast(t)
        return base

    samples = []
    n = int((t_rel + tof_s + 0.7) * hz)
    dt = 1.0 / hz
    for i in range(n + 1):
        t = i * dt
        pc, pm = pos_cmd_at(t), pos_meas_at(t)
        vc = (pos_cmd_at(t + 5e-4) - pos_cmd_at(t - 5e-4)) / 1e-3
        vm = (pos_meas_at(t + 5e-4) - pos_meas_at(t - 5e-4)) / 1e-3
        samples.append(HandSample(t, pm, vm, pc, vc))
    ann = Announcement(t=t_rel - 1.0, t_release=t_rel, tof_s=tof_s,
                       v_mps=v_mps, thrower='jugglebot')
    return (Session(name='synthetic-fixed-shape', kind='synthetic',
                    samples=samples, announcements=[ann], arms=[], t0=0.0),
            t_desc)


def run_fixed_shape_gate() -> int:
    """Assert the FIXED shape reads as FIXED — the half of the charter no real
    capture exercises.

    An instrument validated only against the broken shape scores a working fix as
    FAILED at the bench, routes correct work back for rework, and burns a powered
    sitting.  So every gated runbook row is asserted here against a synthetic
    POST-fix capture, four of them:

      clean            command follows the ramp to x3, hand settles at x3.
      overshoot        same command, hand coasts to 10.60 rev (past the runbook's
                       10.5 rev HARD ABORT).  `peak` must report the real
                       excursion — otherwise the one row guarding the 10.8 rev
                       end stop reads PASS with the hand 0.5 rev from it.
      short-flight     `FLIGHT_TIME_MIN_S = 0.55`.  Here the armed catch descent
                       begins INSIDE `_DIP_WINDOW_S`, so this is the only case in
                       which the `catch_desc` cap on the dip window is
                       load-bearing: without the cap the descent's own velocity
                       and low point become `pullback` and `dip_bottom`, and the
                       probe reports ~122 mm of dip on a clean capture.
      braking-prelude  a legitimate `makeSmoothMove` brake at release+105 ms —
                       exactly what plan Phase 4 step 3 charters.  `catch_desc`
                       must still be the real descent, not the brake; the whole
                       dip/peak window depends on it.

    Every case pins `dip_below_x3_rev`, not `dip_depth_rev`: the latter is
    peak-minus-bottom and is non-zero on all four of these healthy captures.
    """
    print('=' * 78)
    print('GATE — the FIXED shape (synthetic; no real capture has it)')
    x3 = StrokeModel(3.9308).x3_rev
    x3s = StrokeModel(2.6971).x3_rev            # velocity-independent, but be exact
    # The deepest DOWNWARD brake makeSmoothMove can honour: the duration cap
    # (smoothMoveMaxDuration) stops it at v0 = A*T_cap/H2, whose excursion is
    # v0*T_cap*H_MAX = 3.126 rev.  This is the case the old 0.5-rev separator got
    # wrong — see _CATCH_DESC_ABOVE_X5_REV.
    _v0_cap = (hw.TEENSY_TRAJ_MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2
               * math.sqrt(hw.GEOM_HAND_MOTOR_HARD_STOP_REVS
                           * hw.TEENSY_TRAJ_QUINTIC_S2_MAX
                           / hw.TEENSY_TRAJ_MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2)
               / hw.TEENSY_TRAJ_QUINTIC_H2_MAX)
    _deepest_brake_rev = (_v0_cap * _v0_cap * hw.TEENSY_TRAJ_QUINTIC_H2_MAX
                          * hw.TEENSY_TRAJ_QUINTIC_H_MAX
                          / hw.TEENSY_TRAJ_MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2)
    cases = [
        # (label,            peak_rev,  tof,  v,      brake_at_rel)
        ('clean',            x3 + 0.02, 0.80, 3.9308, None),
        ('overshoot',        10.60,     0.80, 3.9308, None),
        ('short-flight',     x3s + 0.02, 0.55, 2.6971, None),
        ('braking-prelude',  x3 + 0.20, 0.80, 3.9308, 0.105),
    ]
    bad = 0
    for label, peak_rev, tof, v, brake in cases:
        s, t_desc_true = _synth_fixed_session(peak_rev, tof_s=tof, v_mps=v,
                                             brake_at_rel=brake)
        tl = analyse_throw(s, s.announcements[0])
        want_peak = peak_rev
        checks = [
            ('status', tl.status, 'not-truncated', tl.status == 'not-truncated'),
            ('trunc', tl.trunc, None, tl.trunc is None),
            ('n_seeds', tl.n_seeds, 0, tl.n_seeds == 0),
            ('peak_pos_rev', tl.peak_pos_rev, want_peak,
             tl.peak_pos_rev is not None
             and abs(tl.peak_pos_rev - want_peak) <= 0.010),
            ('headroom_to_limit_rev', tl.headroom_to_limit_rev,
             hw.GEOM_HAND_MOTOR_HARD_STOP_REVS - want_peak,
             tl.headroom_to_limit_rev is not None
             and abs(tl.headroom_to_limit_rev
                     - (hw.GEOM_HAND_MOTOR_HARD_STOP_REVS - want_peak))
             <= 0.010),
            # the runbook's gated dip row: a healthy capture never goes under x3
            ('dip_below_x3_rev', tl.dip_below_x3_rev,
             '<= %.2f' % _DIP_BELOW_X3_BAND_REV,
             tl.dip_below_x3_rev is not None
             and tl.dip_below_x3_rev <= _DIP_BELOW_X3_BAND_REV),
            # the descent must be the DESCENT, within one telemetry period of the
            # onset the synthetic built
            ('catch_desc', tl.catch_desc, '%.3f' % t_desc_true,
             tl.catch_desc is not None and abs(tl.catch_desc - t_desc_true)
             <= 0.020),
        ]
        if brake is not None:
            # and the brake must be VISIBLE rather than silently swallowed
            checks.append(
                ('first_neg_cmd_vel', tl.first_neg_cmd_vel, '< catch_desc',
                 tl.first_neg_cmd_vel is not None and tl.catch_desc is not None
                 and tl.first_neg_cmd_vel < tl.catch_desc - 0.10))
        for name, got, want, ok in checks:
            bad += 0 if ok else 1
            print('  %-4s %-16s %-22s want %-14s got %s'
                  % ('OK' if ok else 'FAIL', label, name, want, got))

    # ── deep-brake: the SEPARATOR at the clamp's reach ────────────────────────
    # The four cases above are all CLEAN captures.  This one deliberately is not:
    # it carries the deepest DOWNWARD brake makeSmoothMove can honour (3.126 rev
    # below x3, at the duration cap), which is the shape the old 0.5-rev
    # separator misread as the armed catch descent.  Only two properties are
    # asserted — the two the separator owns — and the rest are PRINTED, because a
    # 3.2 rev dive is legitimately not a clean capture and the operator must see
    # it as itself rather than have the gate paper over it.  Concretely: `trunc`
    # and `dip_below_x3` both fire on this shape, and § CHECK HAND-4 rows H4.3 /
    # H4.4 tell the operator how to route that.
    s, t_desc_true = _synth_fixed_session(x3 + 0.02, tof_s=1.10, v_mps=5.3955,
                                          brake_at_rel=0.050,
                                          brake_dive_rev=_deepest_brake_rev)
    tl = analyse_throw(s, s.announcements[0])
    deep_checks = [
        # the whole point: a brake at the clamp's reach is NOT the catch descent
        ('catch_desc', tl.catch_desc, '%.3f' % t_desc_true,
         tl.catch_desc is not None and abs(tl.catch_desc - t_desc_true) <= 0.020),
        # ...so the peak window is still the coasting peak, not the brake
        ('peak_pos_rev', tl.peak_pos_rev, x3 + 0.02,
         tl.peak_pos_rev is not None and abs(tl.peak_pos_rev - (x3 + 0.02)) <= 0.010),
    ]
    for name, got, want, ok in deep_checks:
        bad += 0 if ok else 1
        print('  %-4s %-16s %-22s want %-14s got %s'
              % ('OK' if ok else 'FAIL', 'deep-brake', name, want, got))
    print('  %-4s %-16s %-22s want %-14s got %s'
          % ('--', 'deep-brake', 'dip_below_x3_rev', 'reported, not hidden',
             tl.dip_below_x3_rev))
    print('  %-4s %-16s %-22s want %-14s got %s'
          % ('--', 'deep-brake', 'first_neg_cmd_vel', '< catch_desc',
             tl.first_neg_cmd_vel))

    print('  %s — fixed-shape branch' % ('GATE PASS' if bad == 0 else 'GATE FAIL'))
    return 0 if bad == 0 else 1


def run_gate(path: str) -> int:
    print('=' * 78)
    print('GATE — reproduce plans/active/hand-command-continuity.md Context table')
    print('       from %s' % os.path.basename(path))
    if not os.path.exists(path):
        print('  GATE UNAVAILABLE: reference trace not found:')
        print('    %s' % path)
        print('  The committed fixture is %s' % _GATE_FIXTURE)
        print('  This is NOT an instrument regression — no HAND verdict is')
        print('  invalidated by it; restore the fixture and re-run.')
        return 1
    session = load_trace(path)
    throws = [analyse_throw(session, a) for a in session.announcements]
    throws = [t for t in throws if t.status == 'ok']
    if len(throws) != 1:
        print('  FAIL: expected exactly 1 analysable throw, got %d' % len(throws))
        return 1
    tl = throws[0]
    bad = 0
    for name, want, tol, unit in _GATE_EXPECT:
        got = getattr(tl, name)
        if got is None:
            print('  FAIL %-24s expected %-8s got None' % (name, want))
            bad += 1
            continue
        if name in _GATE_INSTANT_FIELDS:
            got = got - session.t0
        ok = abs(got - want) <= tol
        bad += 0 if ok else 1
        print('  %-4s %-24s expected %9.3f  got %9.3f  delta %+8.4f %s'
              % ('OK' if ok else 'FAIL', name, want, got, got - want, unit))
    print('  %s — %d/%d rows within tolerance'
          % ('GATE PASS' if bad == 0 else 'GATE FAIL',
             len(_GATE_EXPECT) - bad, len(_GATE_EXPECT)))
    return (0 if bad == 0 else 1) | run_fixed_shape_gate()


# ══════════════════════════════════════════════════════════════════════════
#  --emit-gate-fixture : regenerate the committed gate reference
# ══════════════════════════════════════════════════════════════════════════

def emit_gate_fixture(src_trace: str, out_path: str) -> int:
    """Write the committed ``--gate`` reference from a full ``toss_trace`` jsonl.

    The fixture exists because the recording it is cut from lives under
    ``temp/``, which ``.gitignore`` excludes — so the gate (and the operator
    runbook's mandatory instrument self-check) would be unrunnable on a fresh
    clone.  A committed fixture with no committed RECIPE is only half a live
    reference: the next person to re-baseline has to reverse-engineer which rows
    matter.  This is that recipe.

    Kept rows, all re-emitted as their ORIGINAL text so no float re-serialises:

      * the trace's very first record, whatever its topic — ``load_trace``
        anchors ``t0`` on it, and every printed instant is relative to that
        zero, so dropping it would silently shift the whole timeline;
      * every ``throw_announcements`` row;
      * ``hand_telemetry`` and ``/rosout`` rows inside the analysis window
        (``_PRE_WINDOW_S`` / ``_POST_WINDOW_S``) plus ``_FIXTURE_MARGIN_S`` of
        slack, so a later widening of either window does not silently truncate
        the fixture's evidence.

    ``/rosout`` is kept deliberately: it carries the ``Arming hand catch``
    lines, so the fixture reproduces the ``arms`` column too rather than
    reporting it unavailable.
    """
    kept, n_in = [], 0
    rows = []
    with open(src_trace) as fh:
        for line in fh:
            s = line.rstrip('\n')
            if not s.strip():
                continue
            n_in += 1
            try:
                rows.append((s, json.loads(s)))
            except ValueError:
                rows.append((s, None))
    if not rows:
        print('ERROR: %s: no parsable rows' % src_trace)
        return 1
    windows = []
    for _s, r in rows:
        if r and r.get('topic') in ('throw_announcements', '/throw_announcements'):
            off = ((r['t_ros'] - r['t']) if r.get('t_ros') is not None else 0.0)
            rel = float(r['d']['throw_time']) - off
            tof = max(float(r['d'].get('predicted_tof_sec', 0.0)), 0.2)
            windows.append((rel - _PRE_WINDOW_S - _FIXTURE_MARGIN_S,
                            rel + tof + _POST_WINDOW_S + _FIXTURE_MARGIN_S))
    if not windows:
        print('ERROR: %s: no throw_announcements row to anchor a window on'
              % src_trace)
        return 1

    def in_window(t):
        return any(lo <= t <= hi for lo, hi in windows)

    for i, (s, r) in enumerate(rows):
        if i == 0:                                   # the t0 anchor
            kept.append(s)
            continue
        if r is None:
            continue
        topic = r.get('topic')
        if topic in ('throw_announcements', '/throw_announcements'):
            kept.append(s)
        elif topic in ('hand_telemetry', '/rosout') and in_window(r.get('t', -1)):
            kept.append(s)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, 'w') as fh:
        for s in kept:
            fh.write(s + '\n')
    print('wrote %s: %d of %d rows (from %s)'
          % (out_path, len(kept), n_in, os.path.basename(src_trace)))
    print('re-run  python tools/probes/hand_stroke_timeline.py --gate  to verify')
    return 0


# ══════════════════════════════════════════════════════════════════════════

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description='Per-throw hand stroke timeline from a toss trace or rosbag.')
    ap.add_argument('--trace', action='append', default=[],
                    help='a toss_trace_*.jsonl (repeatable)')
    ap.add_argument('--bag', action='append', default=[],
                    help='a rosbag directory containing .mcap (repeatable)')
    ap.add_argument('--all', action='store_true',
                    help='every trace under temp/logs/ and every bag under '
                         '--bag-root')
    ap.add_argument('--bag-root', default=_DEFAULT_BAG_ROOT)
    ap.add_argument('--match', default=None,
                    help='substring filter applied to --all discovery')
    ap.add_argument('--preview', action='store_true',
                    help='also print the raw per-sample window per throw')
    ap.add_argument('--json', action='store_true',
                    help='write per-throw rows to temp/probes/hand_stroke_timeline/')
    ap.add_argument('--gate', action='store_true',
                    help="assert the plan's Context table against the reference "
                         'trace and exit non-zero on any mismatch')
    ap.add_argument('--emit-gate-fixture', metavar='OUT_JSONL', default=None,
                    help='regenerate the committed --gate reference from the '
                         '--trace given (explicit output path; see '
                         'emit_gate_fixture)')
    args = ap.parse_args(argv)

    if args.emit_gate_fixture:
        if len(args.trace) != 1:
            ap.error('--emit-gate-fixture needs exactly one --trace')
        return emit_gate_fixture(args.trace[0], args.emit_gate_fixture)

    if args.gate and not (args.trace or args.bag or args.all):
        return run_gate(_GATE_TRACE)

    traces, bags = list(args.trace), list(args.bag)
    if args.all:
        traces += [p for p in sorted(glob.glob(_DEFAULT_TRACE_GLOB))
                   if p not in traces]
        bags += [d for d in sorted(glob.glob(os.path.join(args.bag_root, '*')))
                 if os.path.isdir(d) and glob.glob(os.path.join(d, '*.mcap'))
                 and d not in bags]
    if args.match:
        traces = [p for p in traces if args.match in p]
        bags = [d for d in bags if args.match in d]
    if not traces and not bags:
        ap.error('nothing to analyse: pass --trace / --bag / --all / --gate')

    rc = 0
    all_rows = []
    for path in traces + bags:
        try:
            session = (load_trace(path) if path.endswith('.jsonl')
                       else load_bag(path))
        # SystemExit stays named: it is not an Exception subclass, and load_bag
        # raises it for a missing mcap_ros2 / an empty bag directory.
        except (Exception, SystemExit) as exc:            # noqa: BLE001 - see below
            # Deliberately broad.  A session killed mid-write leaves a
            # TRUNCATED .mcap whose footer decodes as garbage, and the mcap
            # reader raises its own class for that, not OSError: observed on
            # ~/Desktop/rosbags/2026-06-08_22-25-46 as
            #   mcap.exceptions.RecordLengthLimitExceeded: unknown (opcode 160)
            #   record has length 6302866126619 that exceeds limit 4294967296
            # Under the narrower `except (OSError, SystemExit)` that one bag
            # aborted the whole `--all` sweep with a traceback, and because the
            # discovery order is sorted by date it died BEFORE reaching the
            # 2026-07-25 sessions this plan is about — i.e. the failure mode was
            # "the instrument silently analyses none of the sessions you care
            # about".  Report the bad input, keep the sweep, exit non-zero.
            print('ERROR: %s: %s: %s'
                  % (path, type(exc).__name__, str(exc).split('\n')[0]))
            rc = 1
            continue
        if not session.samples:
            print('=' * 78)
            print('%s: no hand_telemetry — skipped' % os.path.basename(path))
            continue
        timelines = [analyse_throw(session, a) for a in session.announcements]
        print_session(session, timelines, preview=args.preview)
        all_rows += [asdict(t) for t in timelines]

    if args.json:
        os.makedirs(_OUT_DIR, exist_ok=True)
        # Timestamped, like fk_convergence_bag_check.py and cone_bag_decode.py.
        # A fixed name silently destroys the previous analysis, and the operator
        # runbook runs this command once per CHECK HAND-n in the SAME sitting —
        # so a fixed name would leave the pre/post comparison Phase 5 depends on
        # reconstructible only from terminal scrollback.
        out = os.path.join(_OUT_DIR, 'hand_stroke_timeline_%s.json'
                           % time.strftime('%Y%m%d_%H%M%S'))
        with open(out, 'w') as fh:
            json.dump(all_rows, fh, indent=2)
        print('\nwrote %s (%d throw rows)' % (out, len(all_rows)))

    if args.gate:
        rc |= run_gate(_GATE_TRACE)
    return rc


if __name__ == '__main__':
    sys.exit(main())
