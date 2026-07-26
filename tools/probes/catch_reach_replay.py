#!/usr/bin/env python3
"""Offline reproduction of a recorded CATCH REACH, through the production planner.

WHAT IT ANSWERS
---------------
Given a rosbag containing ``/leg_setpoint_echo``, ``/trajectory/status``,
``/trajectory/diagnostics``, ``/trajectory/target_feedback``,
``/catch/dynamic_target``, ``/gravity_offset`` and ``/throw_announcements``, it
answers three questions about one toss's pre-tilt catch reach:

1. **How many plan installs spanned the excursion?**  An *install census* built
   from raw messages — ``move_seq``, ``plan_kind``, ``last_rejection``, every
   accepted ``target_feedback``, every ``catch/dynamic_target``, plus plan-end
   drift (``t + plan_time_remaining_s``). Five clauses, none of them sufficient
   alone; ``install_census``'s docstring says exactly what each one can and
   cannot see. Two corrections the 2026-07-26 review forced, both worth carrying
   because they were originally stated the other way round:

   * ``/trajectory/status`` and ``/trajectory/diagnostics`` are published by ONE
     5 Hz timer (``trajectory_node.py`` line 478 -> ``_publish_status``, which
     publishes both), 1453 messages each over the 290.6 s reference session.
     There is no rate advantage of either over the other.
   * **plan-end drift is NOT the decisive signal.** It is structurally blind to
     a re-plan to the same absolute arrival — the exact hypothesis it was
     credited with refuting — because ``_plan_and_install_catch`` derives its
     lead from a fixed ``arrival_perf``, leaving the plan end where it was. The
     reference bag hands over the counterexample for free: eleven installs
     (``move_seq`` 53 -> 64, 14 accepted feedbacks) across release +0.10..+0.70 s
     drift the plan end **1.02 ms**, LESS than the 1.9 ms measured across the
     genuinely zero-install window. The load-bearing signals are single-valued
     ``move_seq``, single-valued ``plan_kind``, and zero accepted
     ``target_feedback``.

2. **Is the ``peak_leg_*`` diagnostic per-plan or stale?**  It rebuilds the
   plan through ``planner.build_catch`` and compares the resulting
   ``FeasibilityReport`` peaks against what the bag published. See
   THE ``peak_leg_*`` QUESTION below — the answer is *per-plan, with a narrow
   documented hole*, not "stale/cached".

3. **Does the production planner, seeded only from recorded inputs, reproduce
   the commanded ``rx(t)`` the robot actually ran?**  ``build_catch`` is called
   with the FK-reconstructed seed pose, the levelling-corrected wire target and
   the lead implied by the recorded arrival, then sampled and differenced
   against the FK of ``/leg_setpoint_echo``.

THE 2026-07-25 EXCURSION, AND WHAT IT WAS
-----------------------------------------
Reference capture: ``~/Desktop/rosbags/2026-07-25_15-17-48``, toss #4, scheduled
release ``1784956866.878553785``. A catch target of ``rx = -0.77878 deg``
produced a **+2.32 deg** commanded excursion in the OPPOSITE direction, then
settled and held at ``-1.0784 deg`` = 1.385x the target, with a leg
acceleration/jerk spike at release - 0.6 s. All three are ONE plan, installed
ONCE, doing exactly what ``build_catch`` specifies:

* **the opposite-sign excursion** is the quintic ACQUIRING its specified
  terminal arrival RATE. ``build_catch`` gives the reach a non-zero arrival
  twist ``rate * tdir`` (the tilt-through-seat residual, so the rim is still
  moving through the seat angle at contact). A quintic from rest to
  ``(p1, v1, 0)`` decomposes exactly as

      p(s) = p0 + (p1-p0) * psi(s) + v1 * T * phi(s)
      psi  =  10s^3 - 15s^4 +  6s^5      (rest-to-rest, monotone 0 -> 1)
      phi  =  -4s^3 +  7s^4 -  3s^5      (arrival-rate basis, phi(1)=0, phi'(1)=1)

  ``phi`` has its extremum ``-16/81`` at ``s = 2/3``, so a NEGATIVE arrival rate
  drives a POSITIVE excursion of ``(16/81)*|v1|*T`` before turning back. At the
  reference lead that term alone is ``+2.92 deg``; the ``-0.78 deg``
  displacement term contributes ``-0.62 deg`` at the same instant; net
  ``+2.32 deg``. It is a *specified boundary condition of the reach*, not an
  artefact, and it grows LINEARLY WITH THE CATCH LEAD.

* **the 1.385x settle** is the through-seat overshoot:
  ``settle = target * (1 + 0.5*rate*decay / |tilt|)``. For the reference offset
  ``|tilt| = 0.0136459 rad`` and ``0.5*0.07*0.15 = 0.00525 rad``, so the factor
  is ``1.384732`` — matching neither a single nor a double application of the
  levelling correction because it is neither.

* **the acc/jerk spike** is the tilt-through-seat DECAY segment. Gated
  segment-by-segment at the reference lead: the 3.707 s reach peaks at
  ``13.9 mm/s^2 / 35 mm/s^3``; the 0.150 s decay peaks at
  ``142.4 mm/s^2 / 3950 mm/s^3`` — 10.2x the acceleration and 113x the jerk, in a
  segment 25x shorter. It lands at the catch arrival, which the coordinator
  schedules at ``landing - _PRETILT_EARLY_S`` (1.5 s) = release - 0.7 s. The
  tilt "going flat" at the same moment is the same event: the decay ends and the
  quiescent hold begins. Reproducing the node's own 40 Hz REALIZED-peak tracker
  over the rebuilt plan puts the acceleration inside the phase-swept band
  (model 138.7-142.4, bag 138.8); the realized JERK lands a couple of percent
  under it (model 2551-3148, bag 2496) because it is a DIFFERENCE of consecutive
  knots and so carries the emitter's tick jitter (25.0-27.6 ms measured this
  session) on top of the phase. That quantity is REPORTED and never gated.
  **The band does not "bracket" the bag** and must not be quoted as if it did —
  comparing a mid-window running max against an end-of-plan band is what
  produced that false claim in the plan's first draft. The like-for-like
  comparison is a running max truncated at the SAME sample: at the reference
  lead, model 2416.3 against a bag 2403 at the -0.6138 s diagnostics sample.

WHY AN 11 DEG REACH IS CLEAN AND A 0.78 DEG ONE REVERSES
--------------------------------------------------------
The excursion term ``(16/81)*rate*|tdir_i|*T`` depends on the tilt DIRECTION and
the lead, never on the tilt MAGNITUDE; the displacement term IS the magnitude.
So the amplification is

    peak unrequested excursion / requested displacement = (16/81) * rate * T / |tilt|

— identical on both tilt axes, and inversely proportional to how much tilt was
asked for. Measured through this harness on the reference session:

    5 self-toss pre-tilts  |tilt| 0.013646 rad, lead ~3.71 s -> ratio **3.76**
    2 reload  pre-tilts    |tilt| 0.188204 rad, lead ~2.37 s -> ratio **0.17**

A 22x difference in amplification out of one code path, and all seven reaches
reproduce here. The commanded tilt leaves the park in the WRONG DIRECTION once
the ratio exceeds **40/81 = 0.4938** — measured 2026-07-26 as ``min_s
psi(s)/|phi(s)| = 2.5`` in ``|v1|T/|d|`` units, and the exact factor C-CATCH-1
bounds a planner-derived twist to. (An earlier draft of this file and of the plan
said ``psi(2/3) = 0.790``; that is where the value AT ``s = 2/3`` crosses zero,
which is LATER than the first crossing. ``sign_reversal_lead_s`` below still
reports the 0.790 form and is labelled as such.) The reload leg is not
"well behaved" for any reason of its own — it is the same overshoot, 22x smaller
relative to what it was asked to do. It carries the through-seat settle too
(``+1.8235 / -10.9330 deg`` = target x 1.0279, matching the bag exactly).

**Why the through-seat engages at all for a level catch** is a frame defect, not
a tuning value: ``build_catch`` read ``catch_pose[3:5]`` as "the receive tilt",
a premise that holds only while the commanded frame IS the gravity frame. With
a levelling correction loaded, a gravity-level catch arrives as a NON-ZERO
plan-frame tilt — the correction itself — so the through-seat aimed along the
correction.

**CLOSED 2026-07-26 by C-CATCH-1** (``ros_ws/docs/catch_arrival_contract.md``):
``build_catch`` now takes the gravity-referenced receive tilt as its own argument
and bounds every departure from the target that an arrival twist it DERIVES
creates — reach excursion and settle overshoot — to ``40/81`` of the catch's
physical tilt SCALE (the larger of the seed -> target travel and the receive-tilt
magnitude). Every scored row below therefore prints a **C-CATCH-1
COUNTERFACTUAL** block — the same recorded inputs rebuilt through the post-fix
path (:func:`build_fixed`), so a reader sees what the machine would command for
that exact ball. On the reference session: the self-toss reach's unrequested
excursion goes ``2.3239 -> 0.0000 deg`` and its settle ``-1.078408 ->
-0.778784 deg`` (dead on the target, 0.3008 deg of residual gone), while the
reload reach keeps its seat with the aim rotated 4.0997 deg and the settle moved
0.0215 deg. The counterfactual is REPORTED and never gated: the verdict is about
reproducing what the robot DID run.

**AND THEN THE SEAT WAS TURNED OFF, 2026-07-26 (operator decision).**
``planner._CATCH_TILT_THROUGH_RATE_RADPS`` now ships at **0.0**: the trajectory
builder manufactures no motion, so an unopinionated catch arrives with zero twist,
carries no decay segment and settles exactly on its target. Platform motion at a
catch stays PERMITTED — an explicitly-supplied ``tilt_through_rate_radps`` is
honoured verbatim, which is both the seam a future optimising planner uses and the
seam :func:`build_replay` uses — it is simply never MANDATED by a constant. Two
consequences for this probe, and they pull in opposite directions:

* :func:`build_replay` is UNAFFECTED. It passes the rate explicitly, so every
  recorded reach still reproduces bit-for-bit and every VERDICT below is unchanged.
* :func:`build_fixed` IS affected — the counterfactual now shows what a zero-seat
  planner would command. On the reload that is the visible change: the rim is
  stationary at contact and the settle loses its 0.3008 deg overshoot.

``THROUGH_SEAT_RATE_RADPS`` below is therefore a CAPTURE RECORD (0.07, what the
bag ran at), not a mirror of the live default; the live default has its own mirror,
``PLANNER_DEFAULT_THROUGH_SEAT_RATE_RADPS``, and self-check case 7 compares both.

:func:`build_replay` deliberately keeps rebuilding the PRE-fix plan (see its
docstring) — a probe whose job is to reproduce a recorded capture must be able to
express the physics that capture contains.

THE ``peak_leg_*`` QUESTION
--------------------------
``plans/active/catch-reach-degenerate-overshoot.md`` flagged ``peak_leg_vel/acc/
jerk`` reading identically ``14.2 / 142.4 / 3950`` before AND after the install
as evidence the field is "stale or cached rather than per-plan", which would
have invalidated the single-install reading. **Refuted.** The field is written
at every install that carries a ``FeasibilityReport``
(``_svc_go_to_pose``, ``_plan_and_install_timed``, ``_plan_and_install_catch``,
the follower replan) — in the reference bag the immediately preceding
``go_to_pose`` install wrote ``0.0 / 0.0 / 0`` and the catch install wrote
``14.2 / 142.4 / 3950``, identical to the rebuilt plan's report at the published
precision (the publisher formats vel/acc to 0.1 and jerk to 1 mm/s^3; rebuilt
14.2401 / 142.4440 / 3949.7 -> ``:.0f`` -> 3950, so bit-identity is not
verifiable from a bag in principle). That ``0.0 / 0.0 / 0`` one install earlier
is the decisive half: the field is WRITTEN, not carried.

**Why the repetition, and the qualifier that must not be lost.** The values
repeat across *these* catch installs because the plan-level report is the
through-seat decay's peaks, and the decay is a fixed 0.15 s shape — so they are
lead-invariant WHENEVER THE REACH IS SMALL ENOUGH FOR THE DECAY TO DOMINATE.
That is not a general property of catch plans. Measured 2026-07-26 through the
production planner on a reload-sized reach (target ``(11.877, 2.867, 171.163)``,
``rx +1.774 / ry -10.636 deg``), the plan-level ``peak_leg_vel_mmps`` runs
**82.5 -> 30.1 -> 24.7 -> 15.2 -> 14.0 mm/s** at leads 0.8 / 2.0 / 2.371 /
3.707 / 8.0 s — the reach term, which scales as displacement/lead, above the
decay's 14.0 floor until the lead gets long. The reference bag says the same
thing: ``move_seq`` 4 and 15 (the two reload catch installs) published
``23.6`` and ``23.8``, while the toss catches and the post-release republishes
published ``14.0``-``14.2``. So a future reader who sees the predicted vel move
between two consecutive catch installs must NOT infer that a non-catch plan came
in between — that is the same class of inference error this section exists to
kill. Pinned both ways by
``tests/motion/test_catch_reach_replay_probe.py``.

The narrow hole that IS real: six install paths bump ``move_seq`` without
writing the field — ``_svc_hold``, ``_svc_go_home``, ``_install_guard_descent``,
``_retry_pending_stop``, ``_install_graceful_stop`` and the follower's
input-loss stop — so after any of those the PREDICTED peaks describe the
*superseded* plan while ``move_seq`` has already advanced. Read ``peak_leg_*``
only for a ``move_seq`` whose install carried a report; the REALIZED peaks are
reset unconditionally by ``_install`` and are always current.

HOW THE REPLAY IS SEEDED (all from recorded inputs — nothing fitted)
-------------------------------------------------------------------
``seed pose``   FK of ``/leg_setpoint_echo`` immediately before the install
                (at rest: the previous plan was past its total duration, so
                ``_current_state()`` returned its terminal hold).
``target``      ``levelling.correct_pose`` of the ``catch/dynamic_target`` wire
                pose, with the correction from the bag's ``/gravity_offset``.
``arrival``     ``landing_time - _PRETILT_EARLY_S`` from the throw announcement
                (``catch_coordinator_node._pretilt_arrival_perf``), cross-checked
                against ``t + plan_time_remaining_s - tilt_decay - settle_hold``
                from ``/trajectory/status``. On the reference bag those two
                agree to 1.2 ms — that residual IS the perf/ROS clock skew.
``lead D``      ``arrival - install_time``, install taken as the log time of the
                ``catch/dynamic_target`` that triggered it.

Two small unrecorded latencies remain: the callback delay between that message
and ``perf_counter()`` inside ``_plan_and_install_catch``, and the
emit -> Teensy -> bridge -> echo -> record pipeline lag on the comparison
series. The probe therefore fits **one** scalar — the echo lag — over a bounded
grid, and REPORTS the zero-lag residual alongside it. On the reference bag the
fitted lag is ``+10.5 ms`` (under half a 25 ms knot, and positive, as physics
requires), and the residual barely moves without it: rms 0.0189 deg / max
0.0656 deg at zero lag vs rms 0.0049 deg / max 0.0269 deg at +10.5 ms. The lag
trades off against the install estimate monotonically — pushing the install to
the *feedback* log time instead drives the lag NEGATIVE (unphysical: the echo
cannot precede the command), which is what pins the install to within a few ms
of the triggering message.

SELF-CHECK
----------
``--self-check`` needs no bag. It validates the instrument's own model against
the PRODUCTION planner: the psi/phi decomposition to 1e-9, the settle-scale
closed form, the peak's linearity in the catch lead, the segment-level
attribution of the acc/jerk spike to the through-seat decay, and an end-to-end
round trip (synthesise an echo series from a production plan, score it, recover
the peak/settle/lead). If the planner's constants move, this fails rather than
silently keeping the old story. Guarded in CI by
``tests/motion/test_catch_reach_replay_probe.py``.

RELATED
-------
* Plan:    ``plans/active/catch-reach-degenerate-overshoot.md`` (Phase 0/1)
* Sibling: ``tools/probes/levelling_tilt_bag_check.py`` (park-frame verdict; this
           probe reuses its FK reconstruction so both read the commanded pose the
           same way)
* Contract:``ros_ws/docs/levelling_frame.md`` (C-LEVEL-1)
* Tests:   ``tests/motion/test_catch_reach_replay_probe.py``,
           ``tests/ros/test_levelling_frame.py``

SAFETY / SCOPE
--------------
Strictly offline and read-only: it opens ``.mcap`` files and imports pure-Python
planner modules. Never starts a node, never opens a socket, never touches
hardware. Does NOT need ROS2 — ``mcap_ros2`` decodes the schemas embedded in the
bag itself.

USAGE
-----
    source ~/Desktop/PDJ_venv/venv/bin/activate
    # the reference excursion (the plan calls it "toss #4"; it is the 2nd
    # SELF-thrown announcement in the bag — use --list to map the indices):
    python tools/probes/catch_reach_replay.py \
        --bag ~/Desktop/rosbags/2026-07-25_15-17-48 --toss 2 --json --csv
    # the clean-reload CONTRAST case, through the same harness:
    python tools/probes/catch_reach_replay.py \
        --bag ~/Desktop/rosbags/2026-07-25_15-17-48 --thrower ball_butler --toss 2
    # newest bag, list the tosses it contains:
    python tools/probes/catch_reach_replay.py --list
    # prove the instrument's model still matches the production planner:
    python tools/probes/catch_reach_replay.py --self-check

Exit code 0 = the reach reproduced within ``--tol-deg`` (and the census is
self-consistent); 1 = not reproduced, census contradictory, or the bag could not
be read. A NON-reproduction is a publishable result, not a probe bug — read the
FEATURE LEDGER before assuming the harness is at fault.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'))
sys.path.insert(0, _HERE)                       # sibling probe reuse

try:
    import numpy as np
except ImportError:                                              # pragma: no cover
    sys.exit("ERROR: numpy not importable — activate the project venv "
             "(source ~/Desktop/PDJ_venv/venv/bin/activate)")

import levelling_tilt_bag_check as ltbc         # FK reconstruction + bag discovery

from jugglebot.motion import levelling
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    accel_to_leg_accels,
    quat_to_rot_matrix,
    rot_matrix_to_rotvec,
    rotvec_to_rot_matrix,
    twist_to_leg_velocities,
)
from jugglebot.motion.trajectory import planner
from jugglebot.motion.trajectory.feasibility import validate_follow
from jugglebot.motion.trajectory.limits import TrajectoryLimits
from jugglebot.motion.trajectory.plan import TrajectoryPlan
import jugglebot.hardware_config as hw

DEFAULT_ROOT = ltbc.DEFAULT_ROOT
OUT_DIR = ltbc.OUT_DIR

STATUS = '/trajectory/status'
DIAG = '/trajectory/diagnostics'
FEEDBACK = '/trajectory/target_feedback'
DYNTARGET = '/catch/dynamic_target'
GRAVITY = '/gravity_offset'
THROWS = '/throw_announcements'
EVENT_TOPICS = (STATUS, DIAG, FEEDBACK, DYNTARGET, GRAVITY, THROWS)

# ── Production constants this probe MIRRORS rather than imports ──────────────
# Same reasoning as levelling_tilt_bag_check: a probe is a RECORD of what the
# numbers were when a session was scored. If the production constants move, the
# self-check must FAIL loudly rather than silently re-scoring old captures under
# new physics.

#: ``catch_coordinator_node._PRETILT_EARLY_S`` — the pre-tilt target is scheduled
#: to arrive this far BEFORE the announced landing.
PRETILT_EARLY_S = 1.5
#: ``planner.build_catch(tilt_decay_s=...)`` default.
TILT_DECAY_S = 0.15
#: The tilt-through-seat rate the REFERENCE SESSION RAN AT — a capture record, not
#: a mirror of a live production constant. ``planner._CATCH_TILT_THROUGH_RATE_RADPS``
#: shipped to **0.0** on 2026-07-26 (the builder manufactures no motion); this stays
#: at 0.07 because every recorded reach in this bag was planned with it, and
#: :func:`build_replay` passes it EXPLICITLY so the replay keeps rebuilding the plan
#: the robot actually ran. Syncing this to the new default would make every reach in
#: every pre-2026-07-26 capture score NOT-REPRODUCED — the instrument-fails-a-working
#: -system trap. Self-check case 7 pins it to 0.07 for exactly that reason.
THROUGH_SEAT_RATE_RADPS = 0.07
#: ``planner._CATCH_TILT_THROUGH_RATE_RADPS`` — the LIVE default, mirrored so a
#: seat-tuning session that raises it off zero breaks the self-check and has to come
#: back here. :func:`build_fixed` (the post-fix counterfactual) rides on this value.
PLANNER_DEFAULT_THROUGH_SEAT_RATE_RADPS = 0.0
#: ``planner._CATCH_TILT_OVERSHOOT_FRAC``.
THROUGH_SEAT_OVERSHOOT_FRAC = 0.5
#: ``emitter.KNOT_DT_S`` — the 40 Hz knot the realized-peak tracker differences.
KNOT_DT_S = 0.025
#: ``max |phi|`` of the arrival-rate basis ``phi = -4s^3+7s^4-3s^5``, at s = 2/3.
ARRIVAL_TWIST_BASIS_PEAK = 16.0 / 81.0
#: ``planner._CATCH_ARRIVAL_RATE_BOUND`` — C-CATCH-1's bound on a PLANNER-DERIVED
#: arrival twist: ``|v1|*T <= 2.5*scale``, where ``scale`` is the catch's physical
#: tilt size (``planner._catch_scale``: the LARGER of the seed -> target travel and
#: the receive-tilt magnitude). So the reach excursion it adds may not exceed
#: ``40/81`` of that scale. Mirrored, not imported, for the same reason as the
#: constants above.
ARRIVAL_RATE_BOUND = 2.5
#: ``planner._CATCH_EXCURSION_FRAC_BOUND`` — the same 40/81, applied to the OTHER
#: departure the arrival rate manufactures (the settle overshoot past the seat).
#: Kept as its own mirror so the two halves of C-CATCH-1 cannot drift apart:
#: ``EXCURSION_FRAC_BOUND == ARRIVAL_TWIST_BASIS_PEAK * ARRIVAL_RATE_BOUND``.
EXCURSION_FRAC_BOUND = 40.0 / 81.0

# ── Reference capture (2026-07-25_15-17-48, toss #4) ────────────────────────
REFERENCE_OFFSET = (0.013592347421588673, 0.001207157476773584)
REFERENCE_RELEASE_ABS = 1784956866.878553785
#: Measured from the bag (FK of /leg_setpoint_echo), plan frame, park at rx = 0.
REFERENCE_PEAK_DEG = 2.3224
REFERENCE_SETTLE_DEG = -1.0784
REFERENCE_TARGET_RX_DEG = -0.77878414
#: The lead implied by the recorded arrival and the triggering message's log time.
REFERENCE_LEAD_S = 3.707
#: The plan's predicted leg peaks as published on /trajectory/diagnostics.
REFERENCE_PRED_PEAKS = (14.2, 142.4, 3950.0)


def source_constant(module_filename, name):
    """Read a module-level literal constant out of a ROS node's SOURCE.

    ``catch_coordinator_node`` imports ``rclpy``, and this probe's whole scope
    claim is that it needs no ROS. Parsing the assignment out of the source file
    keeps the self-check able to compare against the shipping value without
    taking the dependency. Returns ``None`` if the file or the name is absent —
    which the self-check reports as drift, not as a pass.
    """
    import ast
    path = os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot', 'jugglebot',
                        module_filename)
    try:
        with open(path, 'r', encoding='utf-8') as fh:
            tree = ast.parse(fh.read(), filename=path)
    except (OSError, SyntaxError):                               # pragma: no cover
        return None
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        for tgt in node.targets:
            if isinstance(tgt, ast.Name) and tgt.id == name:
                try:
                    return ast.literal_eval(node.value)
                except ValueError:                               # pragma: no cover
                    return None
    return None


# ═══════════════════════════════════════════════════════════════════════════
# closed-form model (the story this probe tells, in arithmetic)
# ═══════════════════════════════════════════════════════════════════════════

def psi(s):
    """Rest-to-rest quintic basis: 0 at s=0, 1 at s=1, zero end derivatives."""
    s = np.asarray(s, dtype=float)
    return 10.0 * s ** 3 - 15.0 * s ** 4 + 6.0 * s ** 5


def phi(s):
    """Arrival-rate quintic basis: 0 at both ends, unit *time* derivative at s=1.

    This is the term that makes a near-degenerate catch reach bulge: with
    ``p0 == p1`` the whole excursion IS ``v1 * T * phi(s)``, and ``phi`` reaches
    ``-16/81`` at ``s = 2/3`` — so the platform swings OPPOSITE to a specified
    arrival rate before turning back into it.
    """
    s = np.asarray(s, dtype=float)
    return -4.0 * s ** 3 + 7.0 * s ** 4 - 3.0 * s ** 5


def rest_seeded_quintic(p0, p1, v1, duration, s):
    """``p0 + (p1-p0)*psi + v1*T*phi`` — the reach, in closed form.

    Exactly equal to ``QuinticSegment(p0, 0, 0, p1, v1, 0, duration).eval()``;
    the self-check pins that equality against the production segment so this
    decomposition cannot drift away from what the robot runs.
    """
    return p0 + (p1 - p0) * psi(s) + v1 * float(duration) * phi(s)


def settle_scale(tilt_x, tilt_y, rate=THROUGH_SEAT_RATE_RADPS,
                 decay=TILT_DECAY_S, frac=THROUGH_SEAT_OVERSHOOT_FRAC):
    """Factor by which ``build_catch``'s settle overshoots the commanded tilt.

    ``1 + frac*rate*decay / |tilt|``. The 1.385x that "matches neither a single
    nor a double application of the levelling correction" is this number, and it
    is *not* a frame arithmetic error.
    """
    tmag = float(np.hypot(tilt_x, tilt_y))
    if tmag <= 1e-12:
        return 1.0
    return 1.0 + frac * rate * decay / tmag


def excursion_ratio(tilt_x, tilt_y, lead_s, rate=THROUGH_SEAT_RATE_RADPS):
    """Peak UNREQUESTED excursion / the requested tilt displacement.

    ``(16/81) * rate * T / |tilt|`` — identical on both tilt axes, because the
    excursion and the displacement carry the same ``|tdir_i|`` factor.

    **This one number answers the contrast the plan called its most useful fact.**
    The excursion term depends on the tilt DIRECTION and the lead, never on the
    tilt MAGNITUDE; the displacement term is the magnitude. So the amplification
    is inversely proportional to how much tilt was asked for, at fixed lead:

        2026-07-25 toss pre-tilt  : |tilt| 0.013646 rad, lead 3.707 s -> **3.76**
        2026-07-25 reload pre-tilt: |tilt| 0.188204 rad, lead 2.371 s -> **0.17**

    A 22x difference in amplification out of the same code path — which is why an
    11 deg reach looks monotonic and a 0.78 deg one reverses. Note the reload
    denominator is the REQUESTED tilt (10.783 deg = 0.188204 rad), not the
    11.08 deg SETTLE — the settle is already the through-seat overshoot
    (request x 1.0279) and using it here is a category slip.

    The platform leaves the park the WRONG way once the ratio exceeds
    **40/81 = 0.4938** — measured 2026-07-26 as ``min_s psi(s)/|phi(s)| = 5/2``
    in ``|v1|T/|d|`` units, and the factor C-CATCH-1 bounds a planner-derived
    arrival twist to. ``sign_reversal_lead_s`` in the scored row reports the older
    ``psi(2/3) = 0.790`` form instead (the lead at which the value AT ``s = 2/3``
    crosses zero — a LATER, weaker threshold); it is kept for continuity with the
    plan's published numbers and is labelled as such where it is printed. Do not
    size a threshold off 0.790: at that ratio the reach has already reversed.
    """
    tmag = float(np.hypot(tilt_x, tilt_y))
    if tmag <= 1e-12:
        return float('inf')
    return float(ARRIVAL_TWIST_BASIS_PEAK * rate * float(lead_s) / tmag)


def aim_delta_deg(plan_frame_tilt, gravity_tilt):
    """Angle between the PRE-fix and POST-fix through-seat aim directions, in deg.

    Pre-fix ``build_catch`` aimed along the plan-frame tilt (the corrected target);
    post-fix it aims along the gravity-referenced receive tilt (the wire
    orientation). For a gravity-LEVEL catch the second is the zero vector and the
    angle is undefined — the seat is simply not engaged at all, which is reported
    as ``None`` rather than as a rotation.
    """
    a = np.asarray(plan_frame_tilt, dtype=float)[:2]
    b = np.asarray(gravity_tilt, dtype=float)[:2]
    na, nb = float(np.hypot(*a)), float(np.hypot(*b))
    if na <= 1e-12 or nb <= 1e-12:
        return None
    return float(np.degrees(np.arccos(
        float(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0)))))


def peak_off_park_deg(plan, park_deg, n=8001):
    """Largest tilt departure from ``park_deg`` (deg, ``(rx, ry)``) over a plan.

    Sampled across the WHOLE plan on both tilt axes: the 2026-07-25 signature was a
    mid-plan excursion, so an endpoint-only reading would have scored it as clean.
    """
    ts = np.linspace(0.0, float(plan.total_duration), int(n))
    worst = 0.0
    for axis, ref in ((3, float(park_deg[0])), (4, float(park_deg[1]))):
        vals = np.degrees([plan.state_at(float(t))[0][axis] for t in ts])
        worst = max(worst, float(np.max(np.abs(vals - ref))))
    return worst


def wrong_side_excursion_deg(plan, park_deg, target_deg, n=8001):
    """How far the commanded tilt leaves the park AWAY from the target, in deg.

    Per tilt axis, and reported as the worst of the two. This is the exact quantity
    C-CATCH-1's bound factor is derived from: ``2.5`` is the largest
    ``|v1|*T / |target - park|`` for which this is zero. Pre-fix the 2026-07-25
    toss reach scores **2.32 deg** here against a 0.78 deg request; a reach that
    only ever moves toward its target scores 0.
    """
    ts = np.linspace(0.0, float(plan.total_duration), int(n))
    worst = 0.0
    for axis, p, t in ((3, float(park_deg[0]), float(target_deg[0])),
                       (4, float(park_deg[1]), float(target_deg[1]))):
        vals = np.degrees([plan.state_at(float(x))[0][axis] for x in ts])
        away = -np.sign(t - p) if abs(t - p) > 1e-12 else 1.0
        worst = max(worst, float(np.max(np.maximum(away * (vals - p), 0.0))))
    return worst


def arrival_twist_slope_deg_per_s(tilt_x, tilt_y, rate=THROUGH_SEAT_RATE_RADPS):
    """Peak tilt excursion above the park, per second of catch lead, in degrees.

    ``(16/81) * rate * |tdir_x|``. Linear in the lead — which is why a fixed
    threshold on the peak is really a threshold on the catch lead.
    """
    tmag = float(np.hypot(tilt_x, tilt_y))
    if tmag <= 1e-12:
        return 0.0
    return float(np.degrees(ARRIVAL_TWIST_BASIS_PEAK * rate * abs(tilt_x) / tmag))


# ═══════════════════════════════════════════════════════════════════════════
# bag reading
# ═══════════════════════════════════════════════════════════════════════════

def read_events(bag_dir, topics=EVENT_TOPICS):
    """-> ``[(t_abs_s, topic, msg), ...]`` sorted by log time."""
    try:
        from mcap_ros2.reader import read_ros2_messages
    except ImportError as exc:                                   # pragma: no cover
        sys.exit(f"ERROR: mcap_ros2 not importable ({exc}).\n"
                 "       activate the project venv "
                 "(source ~/Desktop/PDJ_venv/venv/bin/activate)")
    out = []
    for path in sorted(glob.glob(os.path.join(bag_dir, '*.mcap'))):
        for m in read_ros2_messages(path, topics=list(topics)):
            out.append((m.log_time_ns * 1e-9, m.channel.topic, m.ros_msg))
    out.sort(key=lambda r: r[0])
    return out


def echo_epoch(bag_dir):
    """Absolute log time of the FIRST ``/leg_setpoint_echo`` message.

    ``ltbc.reconstruct`` deliberately zero-bases its time axis; this probe has to
    align against a perf-domain arrival and a throw announcement, so it needs the
    epoch back. Read with the same ``sorted(glob(...))`` file order
    ``reconstruct`` uses, so the two agree on which message is first.
    """
    try:
        from mcap_ros2.reader import read_ros2_messages
    except ImportError as exc:                                   # pragma: no cover
        sys.exit(f"ERROR: mcap_ros2 not importable ({exc})")
    for path in sorted(glob.glob(os.path.join(bag_dir, '*.mcap'))):
        for m in read_ros2_messages(path, topics=[ltbc.SETPOINT_ECHO]):
            return m.log_time_ns * 1e-9
    return None


def commanded_series(bag_dir, geom, t_lo_abs, t_hi_abs):
    """FK-reconstructed commanded pose over an ABSOLUTE time window.

    -> ``(t_abs, poses Nx6 [mm, rad], n_fk_failures)``. Reuses
    ``ltbc.reconstruct`` so the FK convention (and any future change to the
    solver's convergence criterion) is shared with the levelling probe rather
    than re-derived here.
    """
    epoch = echo_epoch(bag_dir)
    if epoch is None:
        return None, None, 0
    t_rel, poses, failures = ltbc.reconstruct(
        bag_dir, geom, t0=t_lo_abs - epoch, t1=t_hi_abs - epoch)
    if t_rel is None:
        return None, None, failures
    return t_rel + epoch, poses, failures


def self_toss_releases(events, thrower='jugglebot'):
    """Throw announcements this robot made -> ``[{release_abs, landing_abs, ...}]``."""
    out = []
    for t_abs, topic, msg in events:
        if topic != THROWS or str(msg.thrower_name) != thrower:
            continue
        out.append(dict(
            announced_abs=float(t_abs),
            release_abs=float(msg.throw_time.sec) + float(msg.throw_time.nanosec) * 1e-9,
            landing_abs=float(msg.landing_time.sec) + float(msg.landing_time.nanosec) * 1e-9,
            thrower=str(msg.thrower_name)))
    return out


def bag_gravity_offset(events, before_abs=None):
    """The ``/gravity_offset`` in force at ``before_abs`` -> ``(off, meta)``.

    ``before_abs=None`` means "end of bag". **Bounding this matters**: the node
    applies whatever ``_gravity_correction`` was live at INGEST, and ``level`` is
    per-boot, so a session where the operator re-levels mid-sitting publishes two
    different offsets. Taking the last one in the whole bag would score the
    earlier reaches against a correction the node never applied to them — and on
    this session's scale a 0.5 mrad delta is ~0.03 deg of target error against a
    0.05 deg tolerance, i.e. enough to flip a healthy reach to NOT-REPRODUCED.
    ``meta`` reports every distinct value in the bag and which one was used, so a
    re-level is visible in the output rather than silently absorbed.
    """
    off, t_used = None, None
    distinct, seen = [], set()
    for t, topic, msg in events:
        if topic != GRAVITY or len(msg.data) < 2:
            continue
        val = (float(msg.data[0]), float(msg.data[1]))
        if val not in seen:
            seen.add(val)
            distinct.append(dict(t_abs=float(t), offset=list(val)))
        if before_abs is None or t <= before_abs:
            off, t_used = val, float(t)
    meta = dict(n_distinct=len(distinct), distinct=distinct,
                t_used_abs=t_used, bounded_at_abs=(
                    None if before_abs is None else float(before_abs)))
    return off, meta


# ═══════════════════════════════════════════════════════════════════════════
# install census — the "how many plans spanned this?" answer
# ═══════════════════════════════════════════════════════════════════════════

def install_census(events, t_lo_abs, t_hi_abs):
    """Everything the bag says about plan installs inside ``[t_lo, t_hi]``.

    No single signal here is sufficient, and it is worth being precise about
    what each one can and cannot see — the 2026-07-26 review found the original
    docstring crediting the wrong one:

    ``move_seq``            distinct values on ``/trajectory/diagnostics``.
                            ``_install`` bumps a monotonic COUNTER, so a 5 Hz
                            sample still detects every install that goes through
                            ``_install`` at all. Blind to a *follower* install
                            (deliberately does not bump) and to the direct
                            ``_active_plan =`` assignments that bypass
                            ``_install`` entirely.
    ``plan_kinds``          ``plan.kind`` per status sample — ``'move'`` for a
                            segmented plan, ``'hold'`` for a ``HoldPlan``. This
                            is what catches the bypass paths: the emitter's step
                            backstop, the escalation hold and the freeze-in-place
                            all install a ``HoldPlan`` without touching
                            ``move_seq`` or publishing feedback, and they flip
                            this field on the very next 5 Hz sample.
    ``accepted_targets``    accepted ``/trajectory/target_feedback``. Event-rate,
                            not 5 Hz: the timed/catch install path publishes one
                            per install, so this is the highest-resolution
                            install detector the bag carries.
    ``plan_end_*``          ``t + plan_time_remaining_s`` per status sample with
                            time left. **NOT decisive, and deliberately not
                            described as such.** It detects installs that change
                            the plan END (a hold, a stop, a re-aimed arrival),
                            and is STRUCTURALLY BLIND to the one hypothesis it
                            was originally credited with refuting: a re-plan to
                            the SAME absolute arrival. ``_plan_and_install_catch``
                            computes ``lead = arrival_perf - perf_counter()`` and
                            installs at ``t0 = perf_counter()``, so the plan end
                            is ``arrival_perf + tilt_decay + settle_hold``
                            regardless of WHEN the install happened. Measured
                            counterexample in the reference bag itself
                            (2026-07-26): the post-release republish burst runs
                            ``move_seq`` 53 -> 64 with 14 accepted feedbacks
                            across release +0.10..+0.70 s, and the plan-end drifts
                            **1.02 ms** — LESS than the 1.9 ms measured across the
                            genuinely zero-install window.
    ``rejections``          in-window ``last_rejection`` values. ``trajectory_node``
                            LATCHES this field for the node's lifetime (the only
                            assignment of ``''`` is in ``__init__``), so a bare
                            "non-empty" test would make one early rejection
                            anywhere in a session force NOT-REPRODUCED on every
                            later reach. Only a value that CHANGED relative to
                            the last sample before the window counts as an
                            in-window event; the latched carry-over is reported
                            as ``rejection_latched_before`` and never gated.
    """
    move_seqs, plan_ends, kinds, rejections = [], [], set(), set()
    accepted, frozen, dyntargets, pred_peaks, realized = [], [], [], [], []
    n_status = 0
    # `last_rejection` is a sticky lifetime latch — carry the value in force
    # immediately BEFORE the window so an in-window change is distinguishable
    # from a carry-over. Without this the gate fires on unrelated history.
    prior_rejection = ''
    for t_abs, topic, msg in events:
        if topic == STATUS and t_abs < t_lo_abs:
            prior_rejection = str(msg.last_rejection)
    new_rejections = set()
    for t_abs, topic, msg in events:
        if not (t_lo_abs <= t_abs <= t_hi_abs):
            continue
        if topic == STATUS:
            n_status += 1
            kinds.add(str(msg.plan_kind))
            if str(msg.last_rejection):
                rejections.add(str(msg.last_rejection))
                if str(msg.last_rejection) != prior_rejection:
                    new_rejections.add(str(msg.last_rejection))
            rem = float(msg.plan_time_remaining_s)
            if rem > 0.0:
                plan_ends.append(t_abs + rem)
        elif topic == DIAG:
            kv = {k.key: k.value for k in msg.values}
            if 'move_seq' in kv:
                move_seqs.append(int(kv['move_seq']))
            try:
                pred_peaks.append((float(kv['peak_leg_vel_mmps']),
                                   float(kv['peak_leg_acc_mmps2']),
                                   float(kv['peak_leg_jerk_mmps3'])))
            except (KeyError, ValueError):
                pass
            try:
                realized.append((float(t_abs),
                                 float(kv['realized_peak_leg_vel_mmps']),
                                 float(kv['realized_peak_leg_acc_mmps2']),
                                 float(kv['realized_peak_leg_jerk_mmps3'])))
            except (KeyError, ValueError):
                pass
        elif topic == FEEDBACK:
            row = dict(t_abs=float(t_abs), code=str(msg.code),
                       source=str(msg.source), arrival=float(msg.arrival_time))
            if bool(msg.accepted):
                accepted.append(row)
            elif str(msg.code) == 'FROZEN':
                frozen.append(row)
        elif topic == DYNTARGET:
            dyntargets.append(dict(t_abs=float(t_abs),
                                   arrival=float(msg.arrival_time)))

    plan_ends = np.asarray(plan_ends, dtype=float)
    row = dict(
        window=[float(t_lo_abs), float(t_hi_abs)],
        n_status=int(n_status),
        move_seqs=sorted(set(move_seqs)),
        n_move_seq_changes=int(len(set(move_seqs)) - 1) if move_seqs else 0,
        plan_kinds=sorted(kinds),
        rejections=sorted(rejections),
        new_rejections=sorted(new_rejections),
        rejection_latched_before=prior_rejection,
        accepted_targets=accepted,
        frozen_targets=frozen,
        dynamic_targets=dyntargets,
        pred_peaks=sorted(set(pred_peaks)),
        realized_series=realized,
        realized_final=(list(realized[-1][1:]) if realized else None),
    )
    if plan_ends.size:
        row['plan_end_mean_abs'] = float(np.mean(plan_ends))
        row['plan_end_drift_ms'] = float(np.ptp(plan_ends) * 1e3)
        row['plan_end_n'] = int(plan_ends.size)
    else:
        row['plan_end_mean_abs'] = None
        row['plan_end_drift_ms'] = None
        row['plan_end_n'] = 0
    # An install count is only defensible if EVERY signal agrees. Each clause
    # closes a path the others cannot see:
    #   move_seqs        every `_install` that bumps the counter;
    #   plan_kinds       the HoldPlan bypasses, which bump nothing at all;
    #   accepted_targets the event-rate detector for the timed/catch path;
    #   new_rejections   a rejection RAISED in the window (not one latched
    #                    before it — the field never clears);
    #   plan_end_drift   installs that move the plan END. Blind to a re-plan to
    #                    the same absolute arrival, which is why it is one clause
    #                    among five rather than "the decisive one".
    row['single_install'] = bool(
        len(row['move_seqs']) == 1
        and len(row['plan_kinds']) == 1
        and not row['accepted_targets']
        and not row['new_rejections']
        and row['plan_end_drift_ms'] is not None
        and row['plan_end_drift_ms'] <= 10.0)
    return row


# ═══════════════════════════════════════════════════════════════════════════
# the replay
# ═══════════════════════════════════════════════════════════════════════════

def catch_target_from_msg(msg, correction):
    """``catch/dynamic_target`` -> the pose ``trajectory_node`` would install.

    Mirrors ``trajectory_node._catch_target_from_msg`` exactly, including the
    C-LEVEL-1 ingest correction — reproduce the target the node BUILT, not the
    one on the wire, or the whole comparison is against the wrong plan.
    """
    q = msg.target_quat
    rot = quat_to_rot_matrix(float(q.w), float(q.x), float(q.y), float(q.z))
    rotvec = rot_matrix_to_rotvec(rot)
    wire = np.array([float(msg.target_pos.x), float(msg.target_pos.y),
                     float(msg.target_pos.z),
                     rotvec[0], rotvec[1], rotvec[2]], dtype=float)
    return levelling.correct_pose(wire, correction), wire


def build_replay(seed_pose, target, lead_s, limits, geom,
                 settle_hold_s=None, tilt_decay_s=TILT_DECAY_S,
                 receive_tilt=None, rate_radps=THROUGH_SEAT_RATE_RADPS):
    """Seed ``planner.build_catch`` the way ``_plan_and_install_catch`` did.

    **Past tense on purpose.** The default arguments rebuild the PRE-C-CATCH-1
    plan, which is the only thing that can reproduce a capture recorded before
    ``ros_ws/docs/catch_arrival_contract.md`` landed:

    * ``receive_tilt=None`` -> the seat is aimed at ``target[3:5]``, the PLAN-FRAME
      tilt. That is the frame defect itself; with a levelling correction loaded it
      aims a gravity-level catch's through-seat along the correction.
    * ``rate_radps`` is passed EXPLICITLY, which under C-CATCH-1 means "the caller
      requested this arrival twist" and is honoured verbatim. Left implicit, the
      shipping planner would BOUND the manufactured rate to
      ``2.5*|target_tilt - seed_tilt|/T`` and the rebuilt plan would no longer be
      the one the robot ran — the probe would then score a faithful recording as
      NOT-REPRODUCED, which is the "instrument fails a working system" trap.

    :func:`build_fixed` is the post-fix counterpart, and every scored row reports
    the two side by side.
    """
    if settle_hold_s is None:
        settle_hold_s = float(hw.JB_TRAJ_CATCH_SETTLE_HOLD_S)
    state0 = (np.asarray(seed_pose, dtype=float), np.zeros(6), np.zeros(6))
    return planner.build_catch(state0, np.asarray(target, dtype=float),
                               float(lead_s), limits, geom,
                               settle_hold_s=float(settle_hold_s),
                               tilt_decay_s=float(tilt_decay_s),
                               receive_tilt=(None if receive_tilt is None else
                                             np.asarray(receive_tilt, float)),
                               tilt_through_rate_radps=(
                                   None if rate_radps is None
                                   else float(rate_radps)),
                               hold_after=True)


def build_fixed(seed_pose, target, wire_tilt, lead_s, limits, geom,
                settle_hold_s=None):
    """The SAME recorded inputs through the post-C-CATCH-1 production path.

    Differs from :func:`build_replay` in exactly the two ways the fix does — the
    seat is aimed at the **gravity-referenced** receive tilt (the wire orientation
    BEFORE the C-LEVEL-1 ingest correction, which is what
    ``trajectory_node._catch_target_from_msg`` now hands over), and the arrival rate
    is left to the planner so C-CATCH-1 bounds it. Nothing else moves: same seed,
    same corrected target, same lead.

    Since 2026-07-26 "left to the planner" means **zero**
    (``planner._CATCH_TILT_THROUGH_RATE_RADPS = 0.0``), so this rebuilds a catch
    that manufactures no motion at all: two segments, a stationary rim at contact
    and a settle exactly on the target. That is deliberate — this function must
    track the shipping planner, whatever it currently does, or the counterfactual
    stops being one. The rate is intentionally NOT passed here; passing it would
    take C-CATCH-1's requested branch and rebuild a plan no caller would get.

    This is the counterfactual the operator reads before a post-fix sitting — what
    the machine WOULD have commanded on this exact recorded ball.
    """
    if settle_hold_s is None:
        settle_hold_s = float(hw.JB_TRAJ_CATCH_SETTLE_HOLD_S)
    state0 = (np.asarray(seed_pose, dtype=float), np.zeros(6), np.zeros(6))
    return planner.build_catch(state0, np.asarray(target, dtype=float),
                               float(lead_s), limits, geom,
                               settle_hold_s=float(settle_hold_s),
                               receive_tilt=np.asarray(wire_tilt, float)[:2],
                               hold_after=True)


def segment_peaks(plan, limits, geom):
    """Per-segment gate-predicted leg peaks — the acc/jerk-spike attribution.

    Gating each segment ALONE is what turns "there is a spike somewhere" into
    "the spike is the through-seat decay": the reach and the decay differ by an
    order of magnitude in acceleration and two in jerk.
    """
    out = []
    for i, seg in enumerate(plan.segments):
        rep = validate_follow(TrajectoryPlan((seg,), seg.p1), limits, geom)
        out.append(dict(index=i, duration_s=float(seg.duration),
                        peak_leg_vel_mmps=float(rep.peak_leg_vel_mmps),
                        peak_leg_acc_mmps2=float(rep.peak_leg_acc_mmps2),
                        peak_leg_jerk_mmps3=float(rep.peak_leg_jerk_mmps3)))
    return out


def realized_peaks(plan, geom, dt=KNOT_DT_S, phase_s=0.0):
    """Reproduce ``trajectory_node._track_realized_peaks`` over a whole plan.

    Same chain the emitter ships: ``twist_to_leg_velocities`` /
    ``accel_to_leg_accels`` at each 40 Hz knot, jerk as the tick difference of
    the emitted leg acceleration. ``phase_s`` offsets the knot grid — the
    through-seat decay is only ~6 knots long, so WHICH knots land inside it
    moves the sampled maximum by a few percent, and a reproduction that ignores
    the phase will look wrong by exactly that much.

    -> ``(tau, run_peak_vel, run_peak_acc, run_peak_jerk)`` running maxima.
    """
    taus, pv, pa, pj = [], [], [], []
    run_v = run_a = run_j = 0.0
    prev = None
    k = 0
    while phase_s + k * dt <= plan.total_duration:
        tau = phase_s + k * dt
        pose, twist, accel = plan.state_at(tau)
        pos = pose[:3]
        rot = rotvec_to_rot_matrix(pose[3:6])
        lv = twist_to_leg_velocities(twist, pos, rot, geom)
        la = accel_to_leg_accels(accel, twist, pos, rot, geom)
        run_v = max(run_v, float(np.max(np.abs(lv))))
        run_a = max(run_a, float(np.max(np.abs(la))))
        if prev is not None:
            run_j = max(run_j, float(np.max(np.abs(la - prev)) / dt))
        prev = la
        taus.append(tau)
        pv.append(run_v)
        pa.append(run_a)
        pj.append(run_j)
        k += 1
    return (np.asarray(taus), np.asarray(pv), np.asarray(pa), np.asarray(pj))


def align_and_score(plan, install_abs, t_abs, poses, lag_grid_s=None):
    """Difference the sampled plan against the FK series, over the plan window.

    Fits ONE scalar (the echo pipeline lag) on a bounded grid and reports the
    zero-lag residual too, so a reader can see how much of the agreement the fit
    is responsible for (on the reference bag: almost none).
    """
    if lag_grid_s is None:
        lag_grid_s = np.round(np.arange(-0.030, 0.0305, 0.0005), 6)
    lo, hi = install_abs, install_abs + plan.total_duration
    mask = (t_abs >= lo) & (t_abs <= hi)
    te = t_abs[mask]
    if te.size < 8:
        return dict(error=f'only {int(te.size)} commanded samples inside the '
                          f'plan window — widen the bag or check the timestamps')
    rxe = np.degrees(poses[mask, 3])
    rye = np.degrees(poses[mask, 4])

    def model(lag, axis):
        return np.array([np.degrees(plan.state_at(t - install_abs - lag)[0][axis])
                         for t in te])

    best = None
    for lag in lag_grid_s:
        r = model(float(lag), 3) - rxe
        rms = float(np.sqrt(np.mean(r ** 2)))
        if best is None or rms < best[1]:
            best = (float(lag), rms, float(np.max(np.abs(r))))
    lag, rms, mx = best
    mx_rx = model(lag, 3)
    mx_ry = model(lag, 4)
    r0 = model(0.0, 3) - rxe
    r_ry = mx_ry - rye
    return dict(
        n=int(te.size),
        window_abs=[float(lo), float(hi)],
        lag_s=lag,
        rx_rms_deg=rms, rx_max_deg=mx,
        rx_rms_deg_zero_lag=float(np.sqrt(np.mean(r0 ** 2))),
        rx_max_deg_zero_lag=float(np.max(np.abs(r0))),
        ry_rms_deg=float(np.sqrt(np.mean(r_ry ** 2))),
        ry_max_deg=float(np.max(np.abs(r_ry))),
        model_peak_deg=float(np.max(mx_rx)),
        model_peak_t_abs=float(te[int(np.argmax(mx_rx))]),
        echo_peak_deg=float(np.max(rxe)),
        echo_peak_t_abs=float(te[int(np.argmax(rxe))]),
        model_settle_deg=float(mx_rx[-1]),
        echo_settle_deg=float(rxe[-1]),
        model_settle_ry_deg=float(mx_ry[-1]),
        echo_settle_ry_deg=float(rye[-1]),
        series=dict(t_abs=te.tolist(), echo_rx_deg=rxe.tolist(),
                    model_rx_deg=mx_rx.tolist(), echo_ry_deg=rye.tolist(),
                    model_ry_deg=mx_ry.tolist()),
    )


# ═══════════════════════════════════════════════════════════════════════════
# the whole check, for one toss
# ═══════════════════════════════════════════════════════════════════════════

def check_toss(bag_dir, geom, limits, toss, offset, *, pre_s=1.0, post_s=0.5,
               tol_deg=0.05, tol_frac=0.01):
    """Reproduce one toss's pre-tilt catch reach. -> a result row."""
    row = dict(bag=os.path.basename(bag_dir), error=None, verdict=None,
               release_abs=toss['release_abs'], landing_abs=toss['landing_abs'])
    events = read_events(bag_dir)

    # The arrival the coordinator scheduled, from recorded inputs only.
    arrival_abs = toss['landing_abs'] - PRETILT_EARLY_S
    row['arrival_abs'] = float(arrival_abs)
    row['arrival_rel_release_s'] = float(arrival_abs - toss['release_abs'])

    # Which message installed the reach that spans the excursion? The LAST
    # `catch/dynamic_target` that trajectory_node ACCEPTED at or before the
    # arrival: `_republish_pretilt` re-asserts the same pose every balls tick, so
    # an earlier republish is superseded, and republishes inside the reach-freeze
    # window are FROZENed rather than installed. Anchoring on the recorded
    # acceptance (rather than on message order alone) is what makes this correct
    # when the pre-tilt is republished.
    lo_abs = toss['announced_abs'] - 1.0
    acc_fb = [t for t, tp, m in events
              if tp == FEEDBACK and bool(m.accepted) and str(m.source) == 'catch'
              and lo_abs <= t <= arrival_abs]
    dynt = [(t, m) for t, tp, m in events
            if tp == DYNTARGET and lo_abs <= t <= arrival_abs]
    if not dynt:
        row['error'] = f'no {DYNTARGET} between the announcement and the arrival'
        return row
    if acc_fb:
        before = [(t, m) for t, m in dynt if t <= acc_fb[-1]]
        install_abs, dmsg = (before or dynt)[-1]
    else:
        install_abs, dmsg = dynt[-1]
    row['n_accepted_catch_targets_pre_arrival'] = len(acc_fb)

    # Resolve the levelling offset the node had loaded AT INGEST — bounded by the
    # install, not by the end of the bag (see `bag_gravity_offset`).
    if offset is None:
        offset, off_meta = bag_gravity_offset(events, before_abs=install_abs)
        row['offset_source'] = off_meta
    else:
        row['offset_source'] = dict(n_distinct=None, distinct=[],
                                    t_used_abs=None, bounded_at_abs=None)
    if offset is None:
        row['error'] = (f'no {GRAVITY} at or before the install and no --offset '
                        f'given — the ingest correction cannot be reproduced')
        return row
    row['offset'] = [float(offset[0]), float(offset[1])]
    correction = levelling.correction_from_offset(*offset)

    target, wire = catch_target_from_msg(dmsg, correction)
    row['install_abs'] = float(install_abs)
    row['install_rel_release_s'] = float(install_abs - toss['release_abs'])
    row['wire_target'] = [float(v) for v in wire]
    row['target'] = [float(v) for v in target]
    row['target_rx_deg'] = float(np.degrees(target[3]))
    row['target_ry_deg'] = float(np.degrees(target[4]))

    lead = float(arrival_abs - install_abs)
    row['lead_s'] = lead
    settle_hold_s = float(hw.JB_TRAJ_CATCH_SETTLE_HOLD_S)
    row['settle_hold_s'] = settle_hold_s
    row['tilt_decay_s'] = TILT_DECAY_S

    # FK the commanded pose over [install - pre, plan end + post].
    plan_len = lead + TILT_DECAY_S + settle_hold_s
    t_abs, poses, failures = commanded_series(
        bag_dir, geom, install_abs - pre_s, install_abs + plan_len + post_s)
    if t_abs is None:
        row['error'] = f'no usable {ltbc.SETPOINT_ECHO} vectors in the window'
        return row
    row['fk_failures'] = int(failures)

    # Seed: the commanded pose immediately BEFORE the install, at rest.
    pre = t_abs < install_abs
    if not np.any(pre):
        row['error'] = 'no commanded samples before the install — widen --pre'
        return row
    seed_pose = poses[pre][-1].copy()
    row['seed_pose'] = [float(v) for v in seed_pose]
    row['park_rx_deg'] = float(np.degrees(np.mean(poses[pre][:, 3])))
    row['park_ry_deg'] = float(np.degrees(np.mean(poses[pre][:, 4])))
    row['park_span_deg'] = float(np.ptp(np.degrees(poses[pre][:, 3])))

    # ── replay ──────────────────────────────────────────────────────────
    try:
        plan, report = build_replay(seed_pose, target, lead, limits, geom,
                                    settle_hold_s=settle_hold_s)
    except Exception as exc:                                     # noqa: BLE001
        row['error'] = f'build_catch rejected the recorded inputs: {exc}'
        return row

    # ── census over the plan's own window ────────────────────────────────
    # The window opens strictly AFTER the acceptance that created this plan
    # (`_plan_and_install_catch` publishes target_feedback after `_install`), so
    # any accepted feedback the census finds is a genuine RE-install — otherwise
    # every single-install plan would score as two. It closes at the REBUILT
    # plan's own end, not at an assumed length: a level catch (``tmag == 0``)
    # carries no through-seat decay segment at all, and an assumed length would
    # run the window 0.15 s past the end into the NEXT install.
    install_fb = [t for t, tp, m in events
                  if tp == FEEDBACK and bool(m.accepted) and str(m.source) == 'catch'
                  and install_abs <= t <= install_abs + 0.5]
    census_lo = (install_fb[0] if install_fb else install_abs) + 1e-3
    row['install_feedback_abs'] = float(install_fb[0]) if install_fb else None
    row['install_latency_bracket_ms'] = (
        float((install_fb[0] - install_abs) * 1e3) if install_fb else None)
    census = install_census(events, census_lo,
                            install_abs + plan.total_duration - 1e-3)
    row['census'] = census

    # ── what this probe does NOT score ───────────────────────────────────────
    # Everything above is anchored on `arrival_abs = landing - PRETILT_EARLY_S`,
    # i.e. the pre-tilt install. On the reload path that is NOT the plan that runs
    # through ball contact: `catch_coordinator._republish_pretilt` re-asserts the
    # pose every balls tick, `trajectory_node` releases the reach freeze at
    # `arrival + settle_hold` and re-latches it at `arrival - reach_freeze`, so a
    # BURST of further catch installs is accepted in the last ~0.7 s and the last
    # of them is what is frozen through the catch.
    #
    # Counted and printed rather than scored, because an instrument that silently
    # ignores a whole class of installs reads PASS on a machine whose behaviour at
    # contact has changed (finalize review, 2026-07-26: sizing C-CATCH-1's bound on
    # the residual travel de-rated exactly these installs 15.7x while every scored
    # row here was unchanged). A non-zero count is NORMAL on the reload path and is
    # not a failure; it tells the operator which window the verdict covers.
    settle_hold_abs = arrival_abs + float(hw.JB_TRAJ_CATCH_SETTLE_HOLD_S)
    post = [t for t, tp, m in events
            if tp == FEEDBACK and bool(m.accepted) and str(m.source) == 'catch'
            and arrival_abs < t <= toss['landing_abs']]
    row['n_accepted_catch_installs_post_arrival'] = len(post)
    row['post_arrival_install_window_s'] = (
        [float(post[0] - toss['landing_abs']), float(post[-1] - toss['landing_abs'])]
        if post else None)
    row['post_arrival_freeze_release_rel_landing_s'] = float(
        settle_hold_abs - toss['landing_abs'])

    row['plan_total_duration_s'] = float(plan.total_duration)
    row['plan_segments_s'] = [float(s.duration) for s in plan.segments]
    row['report_peaks'] = [float(report.peak_leg_vel_mmps),
                           float(report.peak_leg_acc_mmps2),
                           float(report.peak_leg_jerk_mmps3)]
    row['segment_peaks'] = segment_peaks(plan, limits, geom)

    # The plan end the bag reported, from t + plan_time_remaining_s.
    if census.get('plan_end_mean_abs') is not None:
        row['plan_end_bag_abs'] = census['plan_end_mean_abs']
        row['plan_end_model_abs'] = float(install_abs + plan.total_duration)
        row['plan_end_residual_ms'] = float(
            (row['plan_end_model_abs'] - row['plan_end_bag_abs']) * 1e3)

    score = align_and_score(plan, install_abs, t_abs, poses)
    if 'error' in score:
        row['error'] = score['error']
        return row
    row.update({k: v for k, v in score.items() if k != 'series'})
    row['series'] = score['series']

    # ── the three features, each as model vs measured ───────────────────
    tilt = target[3:5]
    row['closed_form'] = dict(
        settle_scale=float(settle_scale(tilt[0], tilt[1])),
        settle_rx_deg=float(np.degrees(target[3]) * settle_scale(tilt[0], tilt[1])),
        settle_ry_deg=float(np.degrees(target[4]) * settle_scale(tilt[0], tilt[1])),
        arrival_twist_deg_per_s=arrival_twist_slope_deg_per_s(tilt[0], tilt[1]),
        twist_term_peak_deg=float(
            arrival_twist_slope_deg_per_s(tilt[0], tilt[1]) * lead),
        displacement_term_at_two_thirds_deg=float(
            (np.degrees(target[3]) - row['park_rx_deg']) * float(psi(2.0 / 3.0))),
        tilt_mag_rad=float(np.hypot(tilt[0], tilt[1])),
        excursion_ratio=excursion_ratio(tilt[0], tilt[1], lead),
        sign_reversal_lead_s=float(
            psi(2.0 / 3.0) * float(np.hypot(tilt[0], tilt[1]))
            / (ARRIVAL_TWIST_BASIS_PEAK * THROUGH_SEAT_RATE_RADPS)),
    )

    # Feature 3, in the units the bag published it: the REALIZED (40 Hz emitted
    # knot) peaks the node's own tracker would have accumulated. The emitter's
    # knot grid is not phase-locked to the install, and the through-seat decay is
    # only ~6 knots long, so the sampled maximum depends on WHICH knots land
    # inside it — sweep the phase over one knot and report the achievable band
    # rather than a single number that will always look a few percent off.
    bands = [realized_peaks(plan, geom, phase_s=float(p))
             for p in np.arange(0.0, KNOT_DT_S, KNOT_DT_S / 10.0)]
    finals = np.array([[b[1][-1], b[2][-1], b[3][-1]] for b in bands])
    row['realized_model_band'] = dict(
        vel_mmps=[float(finals[:, 0].min()), float(finals[:, 0].max())],
        acc_mmps2=[float(finals[:, 1].min()), float(finals[:, 1].max())],
        jerk_mmps3=[float(finals[:, 2].min()), float(finals[:, 2].max())])
    row['realized_bag_final'] = census.get('realized_final')

    # ── the C-CATCH-1 counterfactual, from the SAME recorded inputs ──────
    # What the post-fix planner would have commanded for this exact ball: seat
    # aimed at the gravity-referenced wire tilt, arrival rate left to the planner
    # so the C-CATCH-1 bound applies. Reported for every reach, never gated — the
    # verdict above is about reproducing what the robot DID.
    try:
        fplan, freport = build_fixed(seed_pose, target, wire[3:5], lead,
                                     limits, geom, settle_hold_s=settle_hold_s)
    except Exception as exc:                                     # noqa: BLE001
        row['fixed'] = dict(error=f'post-fix rebuild rejected: {exc}')
    else:
        park = np.degrees(np.asarray(seed_pose, dtype=float)[3:5])
        tgt_deg = np.degrees(np.asarray(target, dtype=float)[3:5])
        pre_peak = peak_off_park_deg(plan, park)
        fixed_peak = peak_off_park_deg(fplan, park)
        row['fixed'] = dict(
            wrong_side_deg=wrong_side_excursion_deg(fplan, park, tgt_deg),
            pre_wrong_side_deg=wrong_side_excursion_deg(plan, park, tgt_deg),
            n_segments=len(fplan.segments),
            segments_s=[float(s.duration) for s in fplan.segments],
            settle_rx_deg=float(np.degrees(fplan.final_pose[3])),
            settle_ry_deg=float(np.degrees(fplan.final_pose[4])),
            settle_delta_rx_deg=float(np.degrees(fplan.final_pose[3]
                                                 - plan.final_pose[3])),
            settle_delta_ry_deg=float(np.degrees(fplan.final_pose[4]
                                                 - plan.final_pose[4])),
            peak_above_park_deg=fixed_peak,
            pre_peak_above_park_deg=pre_peak,
            report_peaks=[float(freport.peak_leg_vel_mmps),
                          float(freport.peak_leg_acc_mmps2),
                          float(freport.peak_leg_jerk_mmps3)],
            aim_delta_deg=aim_delta_deg(target[3:5], wire[3:5]),
            # C-CATCH-1's scale is the catch's physical tilt SIZE — the LARGER of
            # the seed -> target travel and the receive-tilt magnitude — not the
            # travel alone (`planner._catch_scale`). Using travel alone here would
            # under-report the allowance on any reach seeded near its target and
            # print "BINDS" for a plan the production planner does not bind.
            rate_bound_radps=float(
                ARRIVAL_RATE_BOUND * max(
                    float(np.linalg.norm(np.asarray(target)[3:5]
                                         - np.asarray(seed_pose)[3:5])),
                    float(np.linalg.norm(np.asarray(wire)[3:5]))) / lead),
        )

    # ── verdict ─────────────────────────────────────────────────────────
    # The tolerance is ``max(absolute, fraction of the excursion this reach
    # actually made)``. A purely ABSOLUTE bound would score the reload leg's
    # clean 11 deg pre-tilt as NOT-REPRODUCED for a 0.5 % error, because the echo
    # is a 25 ms-sampled reconstruction of a MOVING command and its residual
    # scales with the motion — routing a correct result back for rework, the
    # exact failure the sibling probe's two-sided self-check exists to prevent.
    # A purely RELATIVE bound would let a near-degenerate reach (tiny span) pass
    # on a large absolute error, which is precisely the case under investigation.
    span = float(max(np.ptp(np.degrees(poses[:, 3])),
                     np.ptp(np.degrees(poses[:, 4]))))
    tol = max(float(tol_deg), float(tol_frac) * span)
    ok = (row['rx_max_deg'] <= tol
          and row['ry_max_deg'] <= tol
          and abs(row['lag_s']) <= KNOT_DT_S
          and census['single_install'])
    row['commanded_span_deg'] = span
    row['tol_deg'] = float(tol_deg)
    row['tol_frac'] = float(tol_frac)
    row['tol_applied_deg'] = float(tol)
    row['verdict'] = 'REPRODUCED' if ok else 'NOT-REPRODUCED'
    return row


# ═══════════════════════════════════════════════════════════════════════════
# reporting
# ═══════════════════════════════════════════════════════════════════════════

def _print_row(row):
    print(f"\nbag: {row['bag']}")
    if row['error']:
        print(f"  ERROR: {row['error']}")
        return
    rel = row['release_abs']
    print(f"  release {rel:.6f}   landing {row['landing_abs'] - rel:+.3f} s   "
          f"levelling offset {row['offset']}")
    om = row.get('offset_source') or {}
    if om.get('n_distinct'):
        print(f"  /gravity_offset: {om['n_distinct']} distinct value(s) in the "
              f"bag; used the one in force at the install"
              + ('  <- RE-LEVEL in this session, check the mapping'
                 if om['n_distinct'] > 1 else ''))

    print("\n  ── INSTALL CENSUS (plan window) ───────────────────────────")
    c = row['census']
    print(f"  move_seq values inside the window : {c['move_seqs']}")
    print(f"  plan_kind values                 : {c['plan_kinds']}  "
          f"<- a HoldPlan bypass flips this and bumps nothing else")
    print(f"  rejections RAISED in the window  : {c['new_rejections'] or 'none'}"
          + (f"   [latched before: {c['rejection_latched_before']!r} — "
             f"advisory, not gated]" if c['rejection_latched_before'] else ''))
    print(f"  accepted target_feedback         : {len(c['accepted_targets'])}"
          f"  <- event-rate, the sharpest install detector here")
    print(f"  catch/dynamic_target             : {len(c['dynamic_targets'])} "
          f"({len(c['frozen_targets'])} FROZEN)")
    drift = c['plan_end_drift_ms']
    print(f"  plan-end (t + remaining) drift   : "
          f"{'n/a' if drift is None else f'{drift:.1f} ms'} over "
          f"{c['plan_end_n']} status samples")
    print(f"     (catches installs that move the plan END; BLIND to a re-plan "
          f"to the same absolute arrival — not the decisive signal)")
    print(f"  peak_leg_* values seen           : {c['pred_peaks']}")
    print(f"  => SINGLE INSTALL: {c['single_install']}")
    n_post = row.get('n_accepted_catch_installs_post_arrival')
    if n_post:
        w = row.get('post_arrival_install_window_s') or [float('nan')] * 2
        print(f"  !! NOT SCORED BY THIS ROW: {n_post} further catch install(s) "
              f"accepted AFTER the scored arrival,")
        print(f"     landing{w[0]:+.3f}..{w[1]:+.3f} s (reach freeze released at "
              f"landing{row['post_arrival_freeze_release_rel_landing_s']:+.3f} s).")
        print(f"     The LAST of those is the plan frozen through ball contact. "
              f"This row scores the PRE-TILT reach only.")

    print("\n  ── REPLAY SEED (recorded inputs only) ─────────────────────")
    print(f"  install      {row['install_rel_release_s']:+.4f} s rel. release "
          f"(log time of the catch/dynamic_target that triggered it)")
    if row.get('install_latency_bracket_ms') is not None:
        print(f"               its accepted target_feedback followed "
              f"{row['install_latency_bracket_ms']:.1f} ms later — the install "
              f"happened somewhere inside that bracket")
    print(f"  arrival      {row['arrival_rel_release_s']:+.4f} s rel. release "
          f"(= landing - {PRETILT_EARLY_S} s pre-tilt earliness)")
    print(f"  lead D       {row['lead_s']:.4f} s   segments "
          f"{['%.4f' % s for s in row['plan_segments_s']]}")
    print(f"  seed pose    x {row['seed_pose'][0]:+.3f}  y {row['seed_pose'][1]:+.3f}  "
          f"z {row['seed_pose'][2]:.3f} mm   rx {np.degrees(row['seed_pose'][3]):+.4f} "
          f"ry {np.degrees(row['seed_pose'][4]):+.4f} deg  "
          f"(park span {row['park_span_deg']:.4f} deg)")
    print(f"  target       rx {row['target_rx_deg']:+.6f}  "
          f"ry {row['target_ry_deg']:+.6f} deg  (wire rx "
          f"{np.degrees(row['wire_target'][3]):+.6f} + C-LEVEL-1 ingest)")
    if 'plan_end_residual_ms' in row:
        print(f"  plan end     model - bag = {row['plan_end_residual_ms']:+.1f} ms "
              f"(perf/ROS clock skew; NOT fitted)")

    print("\n  ── PREDICTED LEG PEAKS (the peak_leg_* question) ──────────")
    v, a, j = row['report_peaks']
    print(f"  rebuilt plan's report : vel {v:.1f}  acc {a:.1f}  jerk {j:.0f} mm/s^3")
    print(f"  bag published         : {c['pred_peaks']}")
    print(f"  {'seg':>4} {'dur_s':>8} {'vel':>9} {'acc':>10} {'jerk':>10}")
    for s in row['segment_peaks']:
        print(f"  {s['index']:>4} {s['duration_s']:>8.4f} "
              f"{s['peak_leg_vel_mmps']:>9.2f} {s['peak_leg_acc_mmps2']:>10.2f} "
              f"{s['peak_leg_jerk_mmps3']:>10.0f}")
    print("  (segment 1 is the tilt-through-seat decay — the acc/jerk spike)")

    print("\n  ── REPRODUCTION vs FK OF /leg_setpoint_echo ───────────────")
    print(f"  samples {row['n']}   echo lag fitted {row['lag_s'] * 1e3:+.1f} ms "
          f"(one scalar; |lag| must stay under one {KNOT_DT_S * 1e3:.0f} ms knot)")
    print(f"  rx residual   rms {row['rx_rms_deg']:.4f}  max {row['rx_max_deg']:.4f} deg"
          f"   [zero-lag: rms {row['rx_rms_deg_zero_lag']:.4f} "
          f"max {row['rx_max_deg_zero_lag']:.4f}]")
    print(f"  ry residual   rms {row['ry_rms_deg']:.4f}  max {row['ry_max_deg']:.4f} deg")

    print("\n  ── FEATURE LEDGER ────────────────────────────────────────")
    cf = row['closed_form']
    print(f"  1  opposite-sign excursion   model {row['model_peak_deg']:+.4f} deg @ "
          f"{row['model_peak_t_abs'] - rel:+.4f} s | echo "
          f"{row['echo_peak_deg']:+.4f} @ {row['echo_peak_t_abs'] - rel:+.4f} s")
    print(f"     = arrival-twist term {cf['twist_term_peak_deg']:+.4f} "
          f"({cf['arrival_twist_deg_per_s']:.4f} deg per s of lead x "
          f"{row['lead_s']:.3f} s) + displacement term "
          f"{cf['displacement_term_at_two_thirds_deg']:+.4f} at s = 2/3")
    print(f"  2  settle                    model {row['model_settle_deg']:+.4f} | echo "
          f"{row['echo_settle_deg']:+.4f} deg   closed form "
          f"{cf['settle_rx_deg']:+.6f} = target x {cf['settle_scale']:.5f}")
    print(f"     ry                        model {row['model_settle_ry_deg']:+.4f} | echo "
          f"{row['echo_settle_ry_deg']:+.4f} deg   closed form "
          f"{cf['settle_ry_deg']:+.6f}")
    sp = row['segment_peaks']
    if len(sp) >= 2:
        print(f"  3  acc/jerk spike            reach {sp[0]['peak_leg_acc_mmps2']:.1f} "
              f"mm/s^2 / {sp[0]['peak_leg_jerk_mmps3']:.0f} mm/s^3  ->  decay "
              f"{sp[1]['peak_leg_acc_mmps2']:.1f} / {sp[1]['peak_leg_jerk_mmps3']:.0f}, "
              f"at arrival = {row['arrival_rel_release_s']:+.3f} s")
    band = row.get('realized_model_band')
    bagr = row.get('realized_bag_final')
    if band:
        print(f"     realized (40 Hz emitted knots, phase-swept over one knot): "
              f"acc {band['acc_mmps2'][0]:.1f}-{band['acc_mmps2'][1]:.1f}  "
              f"jerk {band['jerk_mmps3'][0]:.0f}-{band['jerk_mmps3'][1]:.0f}"
              + (f"   | bag {bagr[1]:.1f} / {bagr[2]:.0f}" if bagr else ''))
        print(f"     REPORT ONLY, never gated: the realized jerk is a DIFFERENCE "
              f"of consecutive knots, so on top of the phase it also carries the "
              f"emitter's tick jitter")
        print(f"     (25.0-27.6 ms measured on this session against a nominal 25 "
              f"ms divisor) — expect it a few percent under the band.")
    print(f"\n  AMPLIFICATION  (16/81)*rate*T / |tilt| = "
          f"{cf['excursion_ratio']:.3f}  "
          f"(|tilt| {cf['tilt_mag_rad']:.6f} rad, lead {row['lead_s']:.3f} s)")
    print(f"     unrequested excursion / requested displacement. Direction-only,"
          f" so it blows up as the")
    print(f"     requested tilt -> 0. C-CATCH-1 bounds a PLANNER-DERIVED twist to "
          f"40/81 = 0.494 of the catch")
    print(f"     SCALE (max of this displacement and the receive-tilt magnitude),")
    print(f"     which is exactly where the reach first leaves the park the WRONG "
          f"way (measured 2026-07-26;")
    print(f"     the 0.790 quoted elsewhere is where the value AT s = 2/3 crosses "
          f"zero, which is later).")

    fx = row.get('fixed')
    if fx and not fx.get('error'):
        print("\n  ── C-CATCH-1 COUNTERFACTUAL (same recorded inputs, fixed aim) ──")
        aim = fx['aim_delta_deg']
        print(f"  through-seat aim   "
              + ('NOT ENGAGED post-fix (gravity-referenced receive tilt is zero '
                 '— a level catch)' if aim is None
                 else f'rotates {aim:.4f} deg (plan-frame tilt -> '
                      f'gravity-referenced receive tilt)'))
        _dflt = PLANNER_DEFAULT_THROUGH_SEAT_RATE_RADPS
        print(f"  arrival-rate bound 2.5*scale/T = {fx['rate_bound_radps']:.5f} rad/s"
              f" vs the {_dflt} rad/s shipped default -> "
              + ('MANUFACTURES NOTHING (default 0.0 since 2026-07-26 — the bound '
                 'is dormant, not removed)' if _dflt <= 0.0
                 else ('BINDS' if fx['rate_bound_radps'] < _dflt
                       else 'does not bind')))
        print(f"  peak off the park  {fx['pre_peak_above_park_deg']:.4f} -> "
              f"{fx['peak_above_park_deg']:.4f} deg")
        print(f"  UNREQUESTED (wrong-side) excursion  "
              f"{fx['pre_wrong_side_deg']:.4f} -> {fx['wrong_side_deg']:.4f} deg"
              f"   <- the quantity C-CATCH-1 bounds")
        print(f"  settle             rx {row['model_settle_deg']:+.6f} -> "
              f"{fx['settle_rx_deg']:+.6f} ({fx['settle_delta_rx_deg']:+.6f}) deg"
              f"   ry {row['model_settle_ry_deg']:+.6f} -> "
              f"{fx['settle_ry_deg']:+.6f} ({fx['settle_delta_ry_deg']:+.6f}) deg")
        print(f"  segments           {['%.4f' % s for s in row['plan_segments_s']]}"
              f" -> {['%.4f' % s for s in fx['segments_s']]}")
        pv, pa, pj = fx['report_peaks']
        print(f"  predicted peaks    {v:.1f} / {a:.1f} / {j:.0f}  ->  "
              f"{pv:.1f} / {pa:.1f} / {pj:.0f}  (vel / acc / jerk)")
        print("  REPORTED, never gated — the verdict above is about reproducing "
              "what the robot DID run.")

    print(f"\n  VERDICT: {row['verdict']} (tolerance +-{row['tol_applied_deg']:.4f} deg "
          f"= max({row['tol_deg']:.3f}, {row['tol_frac']:.3f} x "
          f"{row['commanded_span_deg']:.3f} deg commanded span))")


# ═══════════════════════════════════════════════════════════════════════════
# self-check — the instrument's model, against the production planner
# ═══════════════════════════════════════════════════════════════════════════

def synth_echo(offset, *, park_pose=None, lead_s=REFERENCE_LEAD_S, dt=KNOT_DT_S,
               pre_s=1.0, install_abs=1000.0):
    """A synthetic commanded-pose series built through the PRODUCTION planner.

    Returns ``(t_abs, poses, plan, target, install_abs)`` — the same shapes
    :func:`commanded_series` produces, so the scoring half runs end-to-end with
    no bag. ``park_pose=None`` reproduces the 2026-07-25 shape (park at
    plan-frame rx = 0 while the catch target is corrected).
    """
    geom = StewartGeometry()
    limits = TrajectoryLimits.from_config(hw)
    correction = levelling.correction_from_offset(*offset)
    neutral = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    target = levelling.correct_pose(neutral, correction)
    park = neutral if park_pose is None else np.asarray(park_pose, dtype=float)
    plan, _report = build_replay(park, target, lead_s, limits, geom)

    times, poses = [], []
    n_pre = int(round(pre_s / dt))
    for k in range(n_pre, 0, -1):
        times.append(install_abs - k * dt)
        poses.append(park.copy())
    k = 0
    while k * dt <= plan.total_duration:
        times.append(install_abs + k * dt)
        poses.append(np.asarray(plan.state_at(k * dt)[0], dtype=float))
        k += 1
    return (np.asarray(times), np.asarray(poses), plan, target, install_abs)


def self_check():
    """Score the instrument's own model against the production planner."""
    from jugglebot.motion.trajectory.segment import QuinticSegment

    geom = StewartGeometry()
    limits = TrajectoryLimits.from_config(hw)
    bad = 0

    def check(label, ok, detail):
        nonlocal bad
        bad += 0 if ok else 1
        print(f"{'OK  ' if ok else 'BAD '} {label}\n     {detail}")

    # 1. psi/phi decomposition == the production QuinticSegment.
    p0 = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    p1 = np.array([3.0, -2.0, 171.0, -0.0136, -0.0012, 0.0])
    v1 = np.array([0.0, 0.0, 0.0, -0.0697, -0.0062, 0.0])
    T = 3.707
    seg = QuinticSegment(p0=p0, v0=np.zeros(6), a0=np.zeros(6),
                         p1=p1, v1=v1, a1=np.zeros(6), duration=T)
    worst = 0.0
    for s in np.linspace(0.0, 1.0, 401):
        got = seg.eval(float(s) * T)[0]
        want = rest_seeded_quintic(p0, p1, v1, T, float(s))
        worst = max(worst, float(np.max(np.abs(got - want))))
    check('psi/phi decomposition reproduces QuinticSegment',
          worst < 1e-9, f'max |closed form - production| = {worst:.3e}')

    # 2. |phi| peaks at -16/81 at s = 2/3.
    s_grid = np.linspace(0.0, 1.0, 100001)
    k = int(np.argmin(phi(s_grid)))
    check('arrival-rate basis extremum',
          abs(phi(s_grid)[k] + ARRIVAL_TWIST_BASIS_PEAK) < 1e-9
          and abs(s_grid[k] - 2.0 / 3.0) < 1e-4,
          f'min phi = {phi(s_grid)[k]:.9f} at s = {s_grid[k]:.5f} '
          f'(want {-ARRIVAL_TWIST_BASIS_PEAK:.9f} at 0.66667)')

    # 3. Peak above a DEGENERATE park (p0 == p1) is linear in the lead.
    correction = levelling.correction_from_offset(*REFERENCE_OFFSET)
    target = levelling.correct_pose(np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0]),
                                    correction)
    slope = arrival_twist_slope_deg_per_s(target[3], target[4])
    worst_rel = 0.0
    for lead in (0.8, 1.2, 2.0, 3.707, 5.0):
        plan, _ = build_replay(target, target, lead, limits, geom)
        samp = np.array([np.degrees(plan.state_at(t)[0][3])
                         for t in np.linspace(0.0, lead, 2001)])
        peak = float(np.max(samp) - np.degrees(target[3]))
        worst_rel = max(worst_rel, abs(peak - slope * lead) / (slope * lead))
    check('degenerate-park peak is (16/81)*rate*|tdir_x|*lead',
          worst_rel < 2e-4,
          f'slope {slope:.6f} deg/s, worst relative error {worst_rel:.2e} '
          f'over leads 0.8..5.0 s')

    # 4. The acc/jerk spike belongs to the through-seat decay, structurally.
    plan, report = build_replay(
        np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0]), target,
        REFERENCE_LEAD_S, limits, geom)
    segs = segment_peaks(plan, limits, geom)
    ratio_a = segs[1]['peak_leg_acc_mmps2'] / max(segs[0]['peak_leg_acc_mmps2'], 1e-9)
    ratio_j = segs[1]['peak_leg_jerk_mmps3'] / max(segs[0]['peak_leg_jerk_mmps3'], 1e-9)
    check('through-seat decay dominates the reach in acc and jerk',
          ratio_a > 5.0 and ratio_j > 20.0 and len(segs) == 3,
          f'reach {segs[0]["peak_leg_acc_mmps2"]:.1f} / '
          f'{segs[0]["peak_leg_jerk_mmps3"]:.0f}  ->  decay '
          f'{segs[1]["peak_leg_acc_mmps2"]:.1f} / '
          f'{segs[1]["peak_leg_jerk_mmps3"]:.0f}  '
          f'(x{ratio_a:.1f} acc, x{ratio_j:.0f} jerk)')

    # 5. Plan-level predicted peaks match what the reference session published.
    got = (report.peak_leg_vel_mmps, report.peak_leg_acc_mmps2,
           report.peak_leg_jerk_mmps3)
    check('reference plan reproduces the published peak_leg_*',
          all(abs(g - w) <= 0.06 * max(w, 1.0)
              for g, w in zip(got, REFERENCE_PRED_PEAKS)),
          f'rebuilt {tuple(round(float(g), 1) for g in got)} vs published '
          f'{REFERENCE_PRED_PEAKS}')

    # 6. Settle closed form.
    scale = settle_scale(target[3], target[4])
    check('settle scale closed form',
          abs(scale - 1.384732) < 1e-5
          and abs(np.degrees(target[3]) * scale - REFERENCE_SETTLE_DEG) < 5e-4,
          f'scale {scale:.6f}, settle {np.degrees(target[3]) * scale:+.6f} deg '
          f'(bag {REFERENCE_SETTLE_DEG:+.4f})')

    # 7. The MIRRORED constants still equal their production sources.
    #
    # This case exists because the mirroring block above promises the self-check
    # will "FAIL loudly rather than silently re-score old captures under new
    # physics" — and a 2026-07-26 mutation test showed it only kept that promise
    # for two of the five. `build_replay` PASSES `tilt_decay_s=TILT_DECAY_S` into
    # every production call, so moving `build_catch`'s default from 0.15 to 0.25
    # left the self-check passing 7/7; nothing compared `PRETILT_EARLY_S` against
    # the coordinator at all. Both are constants Phase 2 is expected to move, and
    # a probe that silently keeps building 0.15 s decays would score a CORRECT
    # post-fix capture as NOT-REPRODUCED — routing working code back for rework.
    # Compare the values directly rather than relying on a downstream symptom.
    mirrored = [
        ('build_catch(tilt_decay_s=)',
         planner.build_catch.__kwdefaults__['tilt_decay_s'], TILT_DECAY_S),
        # The LIVE default (0.0 since 2026-07-26). Raising it off zero must break
        # this: `build_fixed`'s counterfactual and every "-> does not bind" line
        # this probe prints are computed against it.
        ('planner._CATCH_TILT_THROUGH_RATE_RADPS',
         planner._CATCH_TILT_THROUGH_RATE_RADPS,
         PLANNER_DEFAULT_THROUGH_SEAT_RATE_RADPS),
        # The RECORDED-session rate, pinned to itself. Not a production mirror any
        # more — but if somebody "helpfully" syncs it to the new default, every
        # pre-2026-07-26 reach silently stops reproducing and the probe fails a
        # faithful recording. Cheaper to catch here than at a powered sitting.
        ('recorded-session rate (capture record, NOT a live mirror)',
         THROUGH_SEAT_RATE_RADPS, 0.07),
        ('planner._CATCH_TILT_OVERSHOOT_FRAC',
         planner._CATCH_TILT_OVERSHOOT_FRAC, THROUGH_SEAT_OVERSHOOT_FRAC),
        ('planner._CATCH_ARRIVAL_RATE_BOUND',
         planner._CATCH_ARRIVAL_RATE_BOUND, ARRIVAL_RATE_BOUND),
        ('planner._CATCH_EXCURSION_FRAC_BOUND',
         planner._CATCH_EXCURSION_FRAC_BOUND, EXCURSION_FRAC_BOUND),
        ('C-CATCH-1 halves share one factor',
         planner._CATCH_EXCURSION_FRAC_BOUND,
         ARRIVAL_TWIST_BASIS_PEAK * ARRIVAL_RATE_BOUND),
        ('hw.JB_TRAJ_CATCH_SETTLE_HOLD_S',
         float(hw.JB_TRAJ_CATCH_SETTLE_HOLD_S), 0.5),
        ('catch_coordinator_node._PRETILT_EARLY_S',
         source_constant('catch_coordinator_node.py', '_PRETILT_EARLY_S'),
         PRETILT_EARLY_S),
    ]
    drifted = [f'{n}: production {p!r} != mirrored {m!r}'
               for n, p, m in mirrored if p != m]
    check('mirrored production constants have not drifted',
          not drifted,
          '; '.join(drifted) if drifted
          else ', '.join(f'{n}={p!r}' for n, p, _m in mirrored))

    # 8. End-to-end round trip through the scoring half.
    t_abs, poses, plan, _t, install_abs = synth_echo(REFERENCE_OFFSET)
    score = align_and_score(plan, install_abs, t_abs, poses)
    check('end-to-end round trip through align_and_score',
          score.get('rx_max_deg', 9e9) < 1e-6 and abs(score['lag_s']) < 1e-9,
          f"rx max {score.get('rx_max_deg', float('nan')):.3e} deg, "
          f"lag {score['lag_s'] * 1e3:+.3f} ms, peak "
          f"{score['model_peak_deg']:+.4f} deg (bag {REFERENCE_PEAK_DEG:+.4f}), "
          f"settle {score['model_settle_deg']:+.4f} "
          f"(bag {REFERENCE_SETTLE_DEG:+.4f})")

    # 9. The C-CATCH-1 counterfactual is TWO-SIDED, like the bag verdict.
    #
    # `build_replay` must still rebuild the pre-fix excursion (or the probe cannot
    # reproduce any capture recorded before the fix) AND `build_fixed` must remove
    # it on the identical inputs (or the counterfactual is not measuring the fix).
    # One-sided would be worthless: a `build_fixed` that silently fell back to the
    # plan-frame aim would print "no change" forever and read as a healthy result.
    park = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    pre_plan, _pre_rep = build_replay(park, target, REFERENCE_LEAD_S, limits, geom)
    fix_plan, _fix_rep = build_fixed(park, target, np.zeros(2), REFERENCE_LEAD_S,
                                     limits, geom)
    park_deg, target_deg = np.degrees(park[3:5]), np.degrees(target[3:5])
    pre_wrong = wrong_side_excursion_deg(pre_plan, park_deg, target_deg)
    fix_wrong = wrong_side_excursion_deg(fix_plan, park_deg, target_deg)
    check('C-CATCH-1 counterfactual is two-sided on the reference reach',
          pre_wrong > 2.0 and fix_wrong < 1e-9 and len(fix_plan.segments) == 2
          and abs(float(np.degrees(fix_plan.final_pose[3]))
                  - REFERENCE_TARGET_RX_DEG) < 1e-9,
          f'wrong-side excursion pre {pre_wrong:.4f} deg -> fixed '
          f'{fix_wrong:.3e} deg; segments {len(pre_plan.segments)} -> '
          f'{len(fix_plan.segments)}; settle rx '
          f'{np.degrees(pre_plan.final_pose[3]):+.6f} -> '
          f'{np.degrees(fix_plan.final_pose[3]):+.6f} '
          f'(target {REFERENCE_TARGET_RX_DEG:+.6f})')

    # 10. The bound factor still equals the quintic geometry it was derived from.
    #
    # 2.5 is not a tuning value: it is `min_s psi(s)/|phi(s)|`, the exact ratio at
    # which a rest-seeded quintic first leaves p0 on the far side from p1. If a
    # future segment implementation stops being that quintic, the bound stops
    # meaning what the contract says it means — and would silently permit (or
    # forbid) motion on a threshold nobody re-derived.
    ss = np.linspace(1e-9, 1.0 - 1e-9, 200001)
    got_bound = float(np.min(psi(ss) / np.abs(phi(ss))))
    d, w = -1.0, -ARRIVAL_RATE_BOUND
    at_bound = float(np.max(-np.sign(d) * (d * psi(ss) + w * phi(ss))))
    over = float(np.max(-np.sign(d) * (d * psi(ss) + 1.02 * w * phi(ss))))
    check('C-CATCH-1 bound factor IS min psi/|phi| (not a tuning value)',
          abs(got_bound - ARRIVAL_RATE_BOUND) < 1e-6
          and at_bound <= 1e-9 < over,
          f'min psi/|phi| = {got_bound:.9f} (mirror {ARRIVAL_RATE_BOUND}); at the '
          f'bound the wrong-side excursion is {at_bound:.2e}, 2 % over it '
          f'{over:.2e}')

    print(f"\nSELF-CHECK: {'PASS' if bad == 0 else f'FAIL ({bad} case(s))'}")
    return 0 if bad == 0 else 1


# ═══════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bag', help='bag directory (default: newest under --root)')
    ap.add_argument('--root', default=DEFAULT_ROOT, help='rosbag root directory')
    ap.add_argument('--toss', type=int, default=1,
                    help='1-based index into the self-thrown announcements')
    ap.add_argument('--thrower', default='jugglebot',
                    help='ThrowAnnouncement.thrower_name to select on')
    ap.add_argument('--list', action='store_true',
                    help='list the tosses in the bag and exit')
    ap.add_argument('--offset', nargs=2, type=float, metavar=('TILT_X', 'TILT_Y'),
                    help='levelling offset in RADIANS (default: from /gravity_offset)')
    ap.add_argument('--tol-deg', type=float, default=0.05,
                    help='absolute floor on the reproduction tolerance, degrees '
                         '(default 0.05)')
    ap.add_argument('--tol-frac', type=float, default=0.01,
                    help='fraction of the commanded excursion added to the '
                         'tolerance floor (default 0.01) — the echo residual '
                         'scales with the motion, so a big reach needs a bigger '
                         'absolute budget')
    ap.add_argument('--pre', type=float, default=1.0,
                    help='seconds of commanded pose to read before the install')
    ap.add_argument('--post', type=float, default=0.5,
                    help='seconds to read after the plan ends')
    ap.add_argument('--json', action='store_true',
                    help=f'write a JSON summary to {OUT_DIR}/')
    ap.add_argument('--csv', action='store_true',
                    help=f'write the model-vs-commanded series to {OUT_DIR}/')
    ap.add_argument('--self-check', action='store_true',
                    help='validate the probe model against the production '
                         'planner and exit — no bag needed')
    args = ap.parse_args(argv)

    if args.self_check:
        return self_check()

    bag = args.bag
    if bag is None:
        if not os.path.isdir(args.root):
            sys.exit(f"ERROR: --root {args.root} is not a directory")
        bag = ltbc._newest_bag(args.root)
        if bag is None:
            sys.exit(f"ERROR: no bag directories with *.mcap under {args.root}")
    bag = os.path.expanduser(bag)
    if not os.path.isdir(bag):
        sys.exit(f"ERROR: --bag {bag} is not a directory")

    events = read_events(bag, topics=(THROWS, GRAVITY))
    tosses = self_toss_releases(events, thrower=args.thrower)
    if args.list or not tosses:
        # Enumerate EVERY thrower in one table, on a shared time origin, with
        # both indices. The plan's prose numbers all the announcements in a
        # session ("toss #4"); `--toss` indexes per-thrower. Printing one
        # thrower at a time — each zero-based on its OWN first announcement —
        # made those two impossible to reconcile without going back to the raw
        # `release_abs`, so an operator asking for "toss #4" would score the
        # wrong reach and never know.
        every = [t for _t, tp, m in events if tp == THROWS
                 for t in self_toss_releases([(_t, tp, m)], thrower=str(m.thrower_name))]
        if not every:
            print("no ThrowAnnouncement in the bag")
            return 1
        every.sort(key=lambda r: r['release_abs'])
        t0 = every[0]['release_abs']
        per = {}
        print(f"{'all#':>5} {'thrower':>14} {'per-thrower#':>13} "
              f"{'release_abs':>18} {'rel_first_s':>12} {'flight_s':>9}")
        for i, t in enumerate(every, 1):
            per[t['thrower']] = per.get(t['thrower'], 0) + 1
            print(f"{i:>5} {t['thrower']:>14} {per[t['thrower']]:>13} "
                  f"{t['release_abs']:>18.6f} {t['release_abs'] - t0:>12.3f} "
                  f"{t['landing_abs'] - t['release_abs']:>9.3f}")
        print("\n  --toss N indexes the per-thrower column, within --thrower "
              "(default 'jugglebot').")
        if not tosses:
            print(f"  (no ThrowAnnouncement with thrower_name == "
                  f"{args.thrower!r})")
            return 1
        return 0
    if not 1 <= args.toss <= len(tosses):
        sys.exit(f"ERROR: --toss {args.toss} out of range (bag has "
                 f"{len(tosses)}; use --list)")

    geom = StewartGeometry()
    limits = TrajectoryLimits.from_config(hw)
    row = check_toss(bag, geom, limits, tosses[args.toss - 1],
                     tuple(args.offset) if args.offset else None,
                     pre_s=args.pre, post_s=args.post, tol_deg=args.tol_deg,
                     tol_frac=args.tol_frac)
    row['toss_index'] = int(args.toss)
    _print_row(row)

    if args.json or args.csv:
        os.makedirs(OUT_DIR, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    if args.json:
        out = os.path.join(OUT_DIR, f'catch_reach_replay_{stamp}.json')
        with open(out, 'w', encoding='utf-8') as fh:
            json.dump({k: v for k, v in row.items() if k != 'series'}, fh,
                      indent=2)
        print(f"\nJSON -> {out}")
    if args.csv and row.get('series'):
        out = os.path.join(OUT_DIR, f'catch_reach_replay_{stamp}.csv')
        s = row['series']
        with open(out, 'w', encoding='utf-8') as fh:
            fh.write('t_abs,t_rel_release_s,echo_rx_deg,model_rx_deg,'
                     'err_rx_deg,echo_ry_deg,model_ry_deg,err_ry_deg\n')
            for i, t in enumerate(s['t_abs']):
                fh.write(f"{t:.6f},{t - row['release_abs']:.6f},"
                         f"{s['echo_rx_deg'][i]:.6f},{s['model_rx_deg'][i]:.6f},"
                         f"{s['model_rx_deg'][i] - s['echo_rx_deg'][i]:.6f},"
                         f"{s['echo_ry_deg'][i]:.6f},{s['model_ry_deg'][i]:.6f},"
                         f"{s['model_ry_deg'][i] - s['echo_ry_deg'][i]:.6f}\n")
        print(f"CSV  -> {out}")

    return 0 if row['verdict'] == 'REPRODUCED' else 1


if __name__ == '__main__':
    sys.exit(main())
