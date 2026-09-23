"""The per-cycle orchestrator for the unified 7-DoF planner (plan Phase 4).

WHAT THIS IS
------------
One function call that turns *session goals + the measured state* into a
7-channel :class:`~motion.trajectory.cycle_plan.CyclePlan` plus the metadata the
rest of the stack needs (release time and velocity, touch-down time, the
announcement fields, the arm/stroke timing twins, the feasibility report).  It is
the single place the Phase-1 chain is driven::

    cup_cycle.plan_window   →  cup Cartesian trajectory for one window (convex QP)
    cup_realize.tilt_schedule →  where the cup points, per knot (banking + pins)
    cup_realize.decompose   →  6-DoF platform pose + slider, per knot
    cycle_plan.CyclePlan    →  the 7-channel plan object, one clock
    feasibility.validate_cycle →  the gate; a non-OK report is a REFUSAL

Nothing here solves anything itself, and nothing here holds state.  Planning is
**per cycle, off the emitter thread** (the determinism rule: no solve and no
blocking I/O in the 40 Hz loop), so this module is a pure function of its
arguments and every refusal is an exception carrying an operator-readable
outcome string.

Pure Python + numpy + the generated ``hardware_config``.  No ROS2 imports, no
``sim`` imports, no repo-root / ``controller`` imports — ``motion/`` is shared by
the simulation and the hardware stack and must stay importable from both.

FRAMES AND UNITS — read this before writing a goal
--------------------------------------------------
Three frames meet in this module and mixing them is the easiest way to move the
robot to the wrong place, so they are stated once, here:

* **Cup site (this module's boundary, and ``CycleGoals``): millimetres, xy in the
  platform frame, z GLOBAL.**  That is exactly ``1000 ×`` the SI vectors
  ``sim/cycle_gate.py`` hands ``plan_window`` (its ``THROW_CUP_Z_M = 0.86`` is
  0.86 m above the floor), and it is the frame ``cup_realize`` already works in:
  :data:`cup_realize.CUP_Z_BASE_MM` = 659.6 mm is the GLOBAL height of the cup
  opening at zero slider.  The xy half needs no conversion at all —
  ``toss_release.stow_to_global_mm`` adds ``GEOM_INITIAL_HEIGHT_MM`` to z and
  leaves x and y alone — so "stow-relative xy" and "global xy" are the same
  numbers, and only z differs between the two conventions.  A caller holding a
  STOW-relative cup point converts with that function; a caller holding a
  ``Toss.action`` goal (a *platform* pose, not a cup point) converts with it too
  and then adds the hand offset.
* **Platform pose (``CycleState.pose``, and every pose in the emitted plan):
  millimetres and radians, STOW-relative**, z = 0 the stow plane and
  z ≈ 170 the ACTIVE plane — the ``DynamicTargetCommand.msg`` convention every
  other plan in this stack uses.  ``cup_realize.decompose`` emits exactly this.
* **Announcement fields: global millimetres**, per ``ThrowAnnouncement.msg``.
  :func:`announcement_fields` emits that frame directly, because a cup site is
  already global in z and global in xy.

The cup QP itself is SI (metres) because the ballistics are natural there; the
conversion happens once, at this module's boundary, and never inside a solve.

There is a **fourth** frame, and it is a *rotational* one: the machine's
levelling frame.  Everything this module aims at — the ballistic take-off
velocity, the observed catch arrival, the apparent-gravity field the banking
schedule tracks — is **gravity-referenced**, while ``CycleState.pose`` and every
pose in the emitted plan are in the **PLAN frame** (``R_gravity @ R_request``),
because that is what the emitter IKs and what ``trajectory_node``'s C-LEVEL-1
ingests already produced.  :func:`_realize` is the single place the two meet:
the tilt schedule is built gravity-referenced and re-expressed into the plan
frame in one step before ``decompose``.  See ``ros_ws/docs/levelling_frame.md``
row **E8**, and :attr:`CycleState.levelling_correction` for how the correction
gets here.

THE FOUR WINDOW KINDS, AND WHY A SESSION NEEDS ALL FOUR
-------------------------------------------------------
``plan_window`` v1 expressed only the steady-state window: it STARTS at a release
and ENDS at a release.  A session cannot be built out of only that shape — there
is no way in and no way out — so the Phase-4 generalisation adds the other three
(see :func:`cup_cycle.plan_window`'s table).  This module names them:

======================  ================================================
:data:`LAUNCH`          from rest → a release.  UH-5's throw-only rung.
:data:`STEADY`          release → catch → release.  UH-6's full cycle.
:data:`LANDING`         release → catch → rest.  UH-4's catch-only rung.
:data:`SETTLE`          release → rest, no catch.  The way out of a throw.
======================  ================================================

Windows CHAIN at a release instant: the terminal state of one is exactly the
start state of the next, which is what :func:`release_state_from_meta` and
:func:`extend` exist to make precise rather than approximate.  A "single toss"
is ``LAUNCH`` then ``LANDING``.

Plan: ``plans/archived/unified-7dof-planner.md`` § 4 Phase 4.
"""

from __future__ import annotations

import dataclasses
import math
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

import jugglebot.hardware_config as hw
from jugglebot.motion import levelling
from jugglebot.motion.trajectory import ballistics_bc
from jugglebot.motion.trajectory import cup_cycle as cc
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import feasibility as fz
from jugglebot.motion.trajectory import tilt_geometry as tg
from jugglebot.motion.trajectory.cycle_plan import CyclePlan
from jugglebot.outcome_detail import bound_msg

#: Hand rev/m — measured (``jugglebot_geometry.hand_mm_per_rev``), not the
#: retired firmware-fudge-factor gain.
HAND_REV_PER_M = float(hw.HAND_REV_PER_M)

# ── Window kinds ─────────────────────────────────────────────────────────────
LAUNCH = 'launch'
STEADY = 'steady'
LANDING = 'landing'
SETTLE = 'settle'
JOINED = 'joined'        #: the output of :func:`extend`
SPLICED = 'spliced'      #: the output of :func:`splice_at`

KINDS = (LAUNCH, STEADY, LANDING, SETTLE)

#: Which events each kind puts on the timeline, and whether it follows a release.
#: ``(has_throw, has_catch, post_release)`` — the whole of what distinguishes the
#: four calls into ``plan_window``.
_KIND_SHAPE = {
    LAUNCH: (True, False, False),
    STEADY: (True, True, True),
    LANDING: (False, True, True),
    SETTLE: (False, False, True),
}


# ── Refusal codes ────────────────────────────────────────────────────────────
#: The planner refused the cycle. The SUBCODE carries which layer said so:
#: a ``cup_cycle`` reason (``CATCH_RUNWAY``, ``CATCH_TOO_EARLY``, ``UNVERIFIED``,
#: ``SINGULAR``, ``SETTLE_SITE``, ``START_BELOW_BOX``, ``START_ABOVE_BOX``,
#: ``ACC_BOX``, ``INFEASIBLE``), a
#: ``validate_cycle`` code (``WORKSPACE``, ``LIMIT_JERK``, ``HAND_STROKE``, …) or
#: one of the three below that this module owns.
TILT_PIN = 'TILT_PIN'                    #: a tilt pin outside the 12° ceiling

#: How far a held-attitude window's SEED may be from the attitude it holds (rad).
#:
#: ``_start_tilt_for``'s measurement: knot-0 tilt costs **0.0494 leg rev per
#: degree** through the 744.3 mm ``CUP_TILT_CENTER_Z_MM`` lever, and
#: ``trajectory_node._install_continuity_ok`` refuses an install at 0.06 rev.
#: 1e-3 rad is 0.0573° ⇒ 0.0028 rev, a factor ~21 inside that gate — small
#: enough that the one-or-two-knot slew the schedule's rate sweeps use to close
#: it cannot be seen by the install gate, and far too small to be the whole
#: pre-tilt (which is a REST's job, not a catch's).
HOLD_TILT_SEAM_TOL_RAD = 1e-3
CHAIN_DISCONTINUITY = 'CHAIN_DISCONTINUITY'   #: :func:`extend` seam mismatch
REPLAN_WINDOW = 'REPLAN_WINDOW'          #: no usable tail at the splice knot

#: The outcome CODE every refusal from this module carries. The parenthetical
#: detail carries the subcode and the numbers, per ``outcome_detail``'s contract,
#: so a guard matching on the bare code keeps matching once a refusal starts
#: carrying its numbers.
OUTCOME_CODE = 'REJECTED_CYCLE_INFEASIBLE'


# ── Derived geometry ─────────────────────────────────────────────────────────
#: Cup-opening world z (m) at the BOTTOM and TOP of the slider's operating band,
#: with the platform at the active-z pin and the cup level.  Derived, not
#: restated: ``cup z = CUP_Z_BASE_MM + slider_mm`` at level, the operating band is
#: slider ``SLIDER_REV_ZERO_MM … SLIDER_REV_ZERO_MM + prime/gain``, and a second
#: spelling of either endpoint is a number that drifts.
_CUP_Z_BOTTOM_M = (cr.CUP_Z_BASE_MM + cr.SLIDER_REV_ZERO_MM) / 1000.0
_CUP_Z_TOP_M = (_CUP_Z_BOTTOM_M
                + float(hw.JB_OP_HAND_CATCH_PRIME_REV) / HAND_REV_PER_M)

#: Inset (m) applied to both ends of that band to get the cup QP's position box.
#: The box is on the cup opening, but the slider is what has to reach it, and a
#: TILTED cup sits lower than a level one by ``arm·(1 − cos θ)`` — at the 12°
#: ceiling and the 250 mm top-of-band lever that is 5.5 mm.  10 mm covers it with
#: room for the solve's own residual, and keeps the box strictly inside the band
#: so a knot on the box boundary is still realisable rather than exactly at the
#: stroke clamp.  Lands at 0.6896 / 0.9846 m, reproducing ``sim/cycle_gate.py``'s
#: hand-set 0.690 / 0.985 to within 0.4 mm at both ends; that gate keeps its own
#: literals so its Phase-1 numbers stay reproducible.
_CUP_Z_INSET_M = 0.010

#: Cup-opening world z (mm) a cycle SETTLES at — the rest site every LANDING and
#: SETTLE window is aimed at, and therefore the hand position a session hands back
#: to the legacy path between cycles.
#:
#: WHY IT IS A CLAMP AND NOT "THE PARKED HEIGHT".  The obvious rest is the hand's
#: own park, ``HAND_RETRACT_REV`` = 0.0 rev, i.e. cup z = ``CUP_Z_BASE_MM +
#: SLIDER_REV_ZERO_MM`` = 679.6 mm.  That is what ``toss_sequencer``'s
#: ``HAND_NOT_PARKED`` gate measures against (``|pos| <= hw.HOMING_HAND_PARK_BAND_REV``,
#: 0.5 rev) and what the pre-R1 ``hand_source`` settle band was centred on
#: (retired at R1 along with the latch; kept as the historical constraint this
#: constant was tuned against).
#: But it is 10 mm BELOW this module's own cup box — the box is inset by
#: :data:`_CUP_Z_INSET_M` at both ends — so a window asked to settle there is
#: refused ``SETTLE_SITE`` before it plans (MEASURED 2026-09-04, the shipped
#: chained LAUNCH+LANDING at session limits: *"settle site z 0.6796 m is outside
#: the cup box [0.6896, 0.9846] m"*).
#:
#: So the settle is the parked height CLAMPED UP into the box: 689.6 mm =
#: **0.3162 rev**, which is inside ``HOMING_HAND_PARK_BAND_REV`` with 37 % of the band to
#: spare, and is a state the NEXT cycle's LAUNCH can also be planned FROM (probed
#: at 689.60 / 690.0 / 692.0 / 695.0 mm — all ACCEPT), so a session's cycle N+1
#: starts where cycle N stopped.  Written as ``max`` rather than as the box floor
#: so it collapses to the true park the moment the box reaches it.
#:
#: NOT inside the pre-R1 firmware's ±0.10 rev ``hand_source`` settle band (that
#: would need 682.8 mm, further out of the box still).  Nothing depended on it
#: being: the latch switch was refused while the setpoint output was armed
#: regardless, so it was never attempted from inside a session.  ``hand_source``
#: and its latch are deleted at R1; this note is historical.
SETTLE_CUP_Z_MM = max(
    (cr.CUP_Z_BASE_MM + cr.SLIDER_REV_ZERO_MM),
    (_CUP_Z_BOTTOM_M + _CUP_Z_INSET_M) * 1000.0)

#: Gravity as this module's mm-native vector — ``ballistics_bc``'s, so the
#: announcement's ballistics and the QP's release equality cannot drift apart
#: (both are 9806 mm/s²; ``cup_cycle.GRAVITY`` is the same number in m/s²).
_G_MM_S2 = ballistics_bc.G_VEC_MMS2

#: Hand speed (rev/s) at or below which the planned hand counts as STOPPED, for
#: the two timing twins.  0.1 rev/s is 3.2 mm/s of slider through the package's
#: gain — two orders below the ~140 rev/s peaks the gate measures on a cycle, and
#: below anything the machine resolves.  The twins are insensitive to it: see
#: ``tests/motion/test_unified_cycle.py::test_timing_twins_are_insensitive_to_the_rest_band``.
_HAND_REST_EPS_RPS = 0.1

#: Equality residual (m) the cup QP is allowed to leave on any hard row —
#: ``cup_cycle``'s own ``feas_tol``, read from the config rather than restated so
#: the seam bar below cannot drift from the solver that produces the number.
_SEAM_FEAS_TOL_M = float(cc.CupCycleConfig.feas_tol)

#: Margin over :data:`_SEAM_FEAS_TOL_M` the seam bars carry.  A bar AT the
#: residual bound would be a bar the solver is licensed to trip on a perfectly
#: chained pair; 10× is enough headroom for the realisation to carry a
#: worst-case residual through ``decompose``'s ``1/(2 − a_z)`` (≤ 1) and its
#: ``arm·axis`` lever without the bar becoming the thing that fails.
_SEAM_MARGIN = 10.0

#: Positional tolerance (mm / rev) on a chain seam, per channel.
#:
#: PROVENANCE — this is NOT a float-round-trip bound.  The two sides of a seam
#: are not the same number computed twice: ``plan_a``'s terminal knot is
#: ``decompose`` of the cup position the QP **solved** for at knot ``n``, while
#: ``plan_b``'s knot 0 is ``decompose`` of the site the chain **pinned** (the
#: goal's ``throw_site_mm``, carried by :func:`release_state_from_meta`).  The
#: two agree only to the QP's terminal-position equality residual, which
#: ``cup_cycle._verify`` bounds at ``feas_tol`` = 1e-7 m — i.e. 1e-4 mm, three
#: orders LOOSER than the 1e-6 mm this constant used to claim.  Measured on the
#: reference chain (2026-09-04): terminal residual 1.28e-11 mm, seam pose gap
#: **exactly 0.0 mm / 0.0 rad** and hand gap **4.07e-13 rev** — so the bar has
#: never been near either value, but it was licensed to be.
#:
#: The rev bar is the same residual through the slider gain
#: (``HAND_REV_PER_M``, 30.705 rev/m), because a cup-z disagreement is what
#: reaches the hand channel.  The ROTATION channels are compared against the mm
#: bar too: they come from the PINNED tilt series and not from the solve, so
#: their gap is exactly zero, and 1e-3 rad still catches the failure this check
#: exists for — a missing ``start_tilt`` leaves 1.4e-2 rad (0.80°, 1.586 mm of
#: centroid), which
#: ``tests/motion/test_unified_cycle.py::test_the_start_tilt_pin_is_what_closes_the_seam``
#: measures.
_SEAM_POS_TOL_MM = _SEAM_MARGIN * _SEAM_FEAS_TOL_M * 1000.0
_SEAM_POS_TOL_REV = _SEAM_MARGIN * _SEAM_FEAS_TOL_M * HAND_REV_PER_M

#: Knots BEFORE the seam that :func:`extend`'s gate must still visit, so its
#: verdict over the new window is identical to a whole-plan run's.
#:
#: DERIVED from ``feasibility.validate_cycle``'s own stencils, not chosen — it is
#: the widest distance backwards, in knots, that any of its passes reaches:
#:
#: * **the per-sample pass** (geometry, leg vel/acc) reads ``state_at`` /
#:   ``hand_at``, and ``CyclePlan._locate`` resolves a sample to the SINGLE
#:   enclosing span (``k = int(t/dt)``, Hermite over knots ``k`` and ``k+1``).
#:   Reach: **0 knots**;
#: * **the leg-jerk pass** finite-differences consecutive SUB-samples
#:   (``np.diff(acc_samples)/sub_dt``, ``sub_dt = dt/samples_per_knot``), so the
#:   difference landing ON the seam needs the sample one ``dt/m`` earlier.
#:   Reach: **1 sub-sample**, strictly inside one knot;
#: * **the hand-span pass** iterates spans ``k → k+1``, so the span that touches
#:   the seam from below is ``k_seam − 1``.  Reach: **1 knot**;
#: * **the per-knot step pass** compares knot ``k`` with knot ``k−1``, so the step
#:   INTO the seam needs knot ``k_seam − 1``.  Reach: **1 knot**;
#: * the catch-runway pass is a single indexed lookup, not a difference, and is
#:   left to run over the whole plan (it is O(1)).
#:
#: One knot is therefore the exact bound, and it is deliberately expressed as a
#: constant with this derivation attached rather than inlined: if a future pass
#: widens its stencil, THIS is the number that has to move with it, and
#: ``test_the_extend_gate_range_covers_every_validate_cycle_stencil`` fails if it
#: does not.
_VALIDATE_STENCIL_KNOTS = 1


class CycleInfeasible(RuntimeError):
    """No trustworthy plan for this cycle — with the layer and the numbers.

    A ``RuntimeError`` subclass for the same reason
    :class:`cup_cycle.CupCycleInfeasible` is one: the callers on this path
    already catch ``RuntimeError`` around a planning call, so an existing guard
    keeps working, while a new caller can catch this precisely and read
    :attr:`code` / :attr:`reasons` / :meth:`outcome`.

    :attr:`code` is the SUBCODE — which layer refused (a ``cup_cycle`` reason, a
    ``validate_cycle`` code, or one of this module's three).  The operator-facing
    string is :meth:`outcome`, which composes it into the single
    ``REJECTED_CYCLE_INFEASIBLE(...)`` outcome that reaches the action result,
    the session's ``per_cycle_outcomes[]`` and the log line at once.
    """

    def __init__(self, code: str, reasons=(), report=None):
        self.code = str(code)
        self.reasons = tuple(str(r) for r in reasons)
        #: The :class:`feasibility.FeasibilityReport` when the gate refused, else
        #: ``None``.  Carried so a caller can log the peaks that were measured,
        #: not only the one that failed.
        self.report = report
        super().__init__(self.outcome())

    def outcome(self) -> str:
        """``'REJECTED_CYCLE_INFEASIBLE(<CODE>: <detail>)'``.

        Round-trips ``outcome_detail.base_outcome`` (which splits at the first
        ``(``) and ``outcome_detail.outcome_subcode`` (which reads the leading
        all-caps token before the ``:``), so a guard can match on
        ``(code, subcode)`` instead of on the whole string — the failure this
        module's outcome would otherwise reintroduce is silent, because a guard
        that stops matching simply does nothing.
        """
        detail = self.code
        if self.reasons:
            detail = '{}: {}'.format(self.code, '; '.join(self.reasons))
        return '{}({})'.format(OUTCOME_CODE, detail)


# ─────────────────────────────────────────────────────────────────────────────
# Goals
# ─────────────────────────────────────────────────────────────────────────────

def _vec3(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.shape != (3,):
        raise ValueError("%s must be a 3-vector, got shape %s"
                         % (name, np.shape(value)))
    if not np.all(np.isfinite(arr)):
        raise ValueError("%s must be finite, got %r" % (name, arr.tolist()))
    return arr


@dataclasses.dataclass(frozen=True)
class CycleGoals:
    """What the session wants of one window.  All lengths mm, all times seconds.

    Sites are CUP-OPENING positions: xy in the platform frame, z GLOBAL — see the
    module docstring's frame block.  ``catch_vel_mm_s`` is the OBSERVED incoming
    ball's arrival velocity (the tracker's), not a cup velocity: the planner
    matches a configured fraction of it (``catch_slider_vel_ratio``) so the seat
    is soft, and derives the receive tilt from its direction.

    The throw fields (``throw_site_mm`` / ``throw_target_mm`` / ``flight_s``) are
    required for the kinds that end in a release, the catch fields for the kinds
    that contain one, and ``settle_site_mm`` for the kinds that end at rest —
    :func:`plan_cycle` checks that per kind rather than accepting a goal that is
    silently half-specified.
    """

    period_s: float
    throw_site_mm: Optional[np.ndarray] = None
    throw_target_mm: Optional[np.ndarray] = None
    flight_s: Optional[float] = None
    catch_site_mm: Optional[np.ndarray] = None
    catch_vel_mm_s: Optional[np.ndarray] = None
    #: Catch instant as a fraction of ``period_s``.  Exactly one of this and
    #: ``catch_t_s`` must be given when the window has a catch.
    catch_frac: Optional[float] = None
    #: Catch instant in seconds from the window start (the explicit form — what a
    #: tracker landing estimate produces).
    catch_t_s: Optional[float] = None
    banking_enabled: bool = True
    #: Whether the cup is holding a ball at knot 0 — C-CUP-2's contact window
    #: opens there when it is.  EXPLICIT, never inferred from ``post_release``:
    #: the re-plan/splice paths solve mid-carry tails with ``post_release=False``
    #: and such a tail can equally be mid-FLIGHT, so inferring would arm the gate
    #: (and its refusal) for a ball that is not there.  A :data:`LAUNCH` sets it
    #: by kind — a launch throws the ball it is holding, which is what a launch
    #: IS — so only a :data:`SETTLE` that holds one needs to say so.
    holds_ball: bool = False
    #: Cup site the window comes to rest at.  Defaults to the catch site for a
    #: LANDING (the cup stops where it caught, which is the seat the ball is
    #: already in); REQUIRED for a SETTLE, which has no catch to default from.
    settle_site_mm: Optional[np.ndarray] = None
    ball_id: int = 0
    #: The receive attitude (rx, ry) rad this LANDING HOLDS, constant, through
    #: the whole window — the FSM's pre-tilted catch (see
    #: ``cup_cycle.CatchEvent.axis``).  ``None`` is the level-catch window.
    #: Banking is off for a held attitude: the two are competing answers to the
    #: same question, and the ball is seated by the tilt, not by the bank.
    hold_tilt: Optional[Tuple[float, float]] = None
    #: The attitude (rx, ry) rad a SETTLE ENDS at — the PRE-TILT that puts the
    #: platform at the receive attitude before the ball arrives, and the DECAY
    #: REST that takes it back to level after the seat.  ``None`` is the
    #: level-terminal REST.  Banking is off, and the slew is the quintic
    #: smoothstep (``cup_realize._smooth_slew``), not a rate-limited ramp.
    rest_tilt: Optional[Tuple[float, float]] = None

    def catch_time_s(self) -> float:
        """The catch instant on the window clock, from whichever form was given."""
        if (self.catch_frac is None) == (self.catch_t_s is None):
            raise ValueError(
                "give exactly one of catch_frac / catch_t_s (got %r / %r)"
                % (self.catch_frac, self.catch_t_s))
        if self.catch_t_s is not None:
            return float(self.catch_t_s)
        return float(self.catch_frac) * float(self.period_s)


# ─────────────────────────────────────────────────────────────────────────────
# The forward map: platform + slider  →  cup opening
# ─────────────────────────────────────────────────────────────────────────────

def cup_state_from_platform(pose, hand_rev, cfg=None) -> np.ndarray:
    """Cup-opening position (mm; xy platform-frame, z global) for a pose + slider.

    The **exact inverse** of :func:`cup_realize.decompose`'s position map, solved
    in closed form.  ``decompose`` writes, per knot::

        arm    = cup_z − CUP_TILT_CENTER_Z_MM
        drop   = arm · (1 − cup_axis_z)
        slider = cup_z − base(z) + drop        (base(z) = CUP_Z_BASE_MM + z − active_z)
        cup_xy = centroid_xy + arm · cup_axis_xy

    The slider relation is linear in ``cup_z`` on both sides, so it inverts
    without iteration::

        cup_z = (slider + Δz + CUP_Z_BASE_MM + C·(1 − a_z)) / (2 − a_z)

    with ``C = CUP_TILT_CENTER_Z_MM``, ``Δz = pose_z − active_z`` and
    ``a = cup_axis(rx, ry)``.  At level (``a_z = 1``) it collapses to
    ``cup_z = slider + Δz + CUP_Z_BASE_MM``, which is the level realisation read
    backwards.  ``tilt_geometry.cup_axis`` is used — not ``shaping``'s closed-form
    twin — because ``decompose`` uses it and the two disagree at ~8e-17; a forward
    map that is only *mathematically* the inverse would not round-trip to float
    precision, and that round trip is what
    ``tests/motion/test_unified_cycle.py`` pins.

    **It cannot invert a SATURATED knot, by construction.**  ``decompose`` clamps
    the slider to ``[0, stroke]``, and a clamp destroys the information this map
    would need.  That is not a gap: a clamped slider leaves the operating band
    ``[0, JB_OP_HAND_CATCH_PRIME_REV]`` on either side, so
    ``feasibility.validate_cycle`` refuses the cycle with ``HAND_STROKE`` before
    any caller gets to ask this question.  ``decompose``'s
    ``RealizedCycle.slider_saturated`` is the per-knot witness.
    """
    cfg = cr.RealizeConfig() if cfg is None else cfg
    p = np.asarray(pose, dtype=float).reshape(-1)
    if p.shape != (6,):
        raise ValueError("pose must be a 6-vector, got shape %s" % (np.shape(pose),))
    a = tg.cup_axis(float(p[3]), float(p[4]))
    slider_mm = (float(hand_rev) / HAND_REV_PER_M * 1000.0
                 + float(cfg.slider_rev_zero_mm))
    dz = float(p[2]) - float(cfg.active_z_mm)
    c_z = float(tg.CUP_TILT_CENTER_Z_MM)
    cup_z = ((slider_mm + dz + float(cfg.cup_z_base_mm) + c_z * (1.0 - float(a[2])))
             / (2.0 - float(a[2])))
    arm = tg.cup_lever_arm_mm(cup_z)
    return np.array([p[0] + arm * a[0], p[1] + arm * a[1], cup_z])


def hand_rev_for_cup_z(cup_z_mm: float, cfg=None) -> float:
    """Slider rev whose LEVEL realisation puts the cup opening at ``cup_z_mm``.

    The LEVEL half of :func:`cup_state_from_platform`, read backwards, exported
    because three call sites outside this module were each re-deriving it from
    ``cup_realize``'s two constants and the gain — and a fourth spelling of a map
    is how a rest height drifts away from the park band it is supposed to sit in.
    At level (``a_z = 1``, platform at the active-z pin) that map collapses to
    ``cup_z = CUP_Z_BASE_MM + slider_mm`` with ``slider_mm = SLIDER_REV_ZERO_MM +
    rev/gain·1000``, which inverts directly.

    LEVEL, deliberately: this answers *"where does the slider have to be to park
    the cup here"*, which is a question about a stationary, untilted machine (a
    rest site, a park band, a settle).  A TILTED cup sits lower for the same
    slider by ``arm·(1 − cos θ)``, and the full inverse of that is
    :func:`cup_state_from_platform` — use it, not this, for anything mid-cycle.
    """
    cfg = cr.RealizeConfig() if cfg is None else cfg
    slider_mm = float(cup_z_mm) - float(cfg.cup_z_base_mm)
    return ((slider_mm - float(cfg.slider_rev_zero_mm)) / 1000.0
            * HAND_REV_PER_M)


def cup_z_for_hand_rev(hand_rev: float, cfg=None) -> float:
    """Cup-opening world z (mm) a LEVEL cup sits at for slider ``hand_rev``.

    The exact inverse of :func:`hand_rev_for_cup_z`; the pair exists so a caller
    that has one of the two never has to restate the map to get the other.
    """
    cfg = cr.RealizeConfig() if cfg is None else cfg
    slider_mm = (float(hand_rev) / HAND_REV_PER_M * 1000.0
                 + float(cfg.slider_rev_zero_mm))
    return float(cfg.cup_z_base_mm) + slider_mm


def _cup_axis_rate(rx: float, ry: float, rx_dot: float, ry_dot: float):
    """``d/dt cup_axis(rx, ry)`` for tilt rates ``(rx_dot, ry_dot)``.

    Closed form of the same rotation ``tilt_geometry.cup_axis`` builds through the
    IK helper: with ``θ = |(rx, ry)|`` and ``s = sin θ / θ``,
    ``a = (ry·s, −rx·s, cos θ)``.  Differentiating gives the expression below; at
    ``θ → 0`` it degenerates smoothly (``s → 1``, ``ds/dθ → 0``) and the small-θ
    branch is the limit, not an approximation with a different value.

    Used only for the VELOCITY half of the forward map — the position half goes
    through ``tilt_geometry.cup_axis`` for the bit-exact round trip.
    """
    theta = math.hypot(rx, ry)
    if theta < 1e-9:
        return (np.array([ry, -rx, 1.0]),
                np.array([ry_dot, -rx_dot, 0.0]))
    sin_t = math.sin(theta)
    cos_t = math.cos(theta)
    s = sin_t / theta
    ds = (cos_t * theta - sin_t) / (theta * theta)
    theta_dot = (rx * rx_dot + ry * ry_dot) / theta
    a = np.array([ry * s, -rx * s, cos_t])
    a_dot = np.array([ry_dot * s + ry * ds * theta_dot,
                      -rx_dot * s - rx * ds * theta_dot,
                      -sin_t * theta_dot])
    return a, a_dot


def cup_velocity_from_platform(pose, pose_vel, hand_rev, hand_vel_rps,
                               cfg=None) -> np.ndarray:
    """Cup-opening velocity (mm/s) for a pose + pose rate + slider state.

    The derivative of :func:`cup_state_from_platform`, obtained by differentiating
    the same two relations ``decompose`` differentiates forwards and solving for
    the cup terms::

        v_z  = (slider_dot + Δz_dot + arm·ȧ_z) / (2 − a_z)
        v_xy = centroid_dot + v_z·a_xy + arm·ȧ_xy

    NOT an exact numerical inverse of ``decompose``'s ``pose_vel`` /
    ``slider_vel_rev_s``, and deliberately so: ``decompose`` finite-differences
    the tilt series to get ``ȧ`` (it has a knot series and no analytic tilt rate),
    while this has an analytic tilt rate and no series.  The two agree to the
    finite difference's own O(dt²) truncation, which is what
    ``tests/motion/test_unified_cycle.py`` measures and pins — a tighter claim
    would be false.

    Whenever the caller KNOWS the cup velocity exactly — at rest it is zero, and
    just after a release it is the take-off velocity the QP pinned — it should
    pass it as :attr:`CycleState.cup_vel_mm_s` instead of routing it through here.
    """
    cfg = cr.RealizeConfig() if cfg is None else cfg
    p = np.asarray(pose, dtype=float).reshape(-1)
    pv = np.asarray(pose_vel, dtype=float).reshape(-1)
    if p.shape != (6,) or pv.shape != (6,):
        raise ValueError("pose and pose_vel must both be 6-vectors")
    # ``arm`` needs the cup HEIGHT, which needs the slider POSITION — hence
    # ``hand_rev`` here as well as ``hand_vel_rps``; the lever is a height-
    # dependent quantity, not a constant (``cup_realize.decompose``'s ``arm``).
    cup_z = float(cup_state_from_platform(p, hand_rev, cfg)[2])
    a = tg.cup_axis(float(p[3]), float(p[4]))
    _, a_dot = _cup_axis_rate(float(p[3]), float(p[4]),
                              float(pv[3]), float(pv[4]))
    slider_dot = float(hand_vel_rps) / HAND_REV_PER_M * 1000.0
    return _cup_velocity(pv, slider_dot, float(pv[2]), a, a_dot, cup_z)


def _cup_velocity(pv, slider_dot, dz_dot, a, a_dot, cup_z):
    arm = tg.cup_lever_arm_mm(cup_z)
    v_z = (slider_dot + dz_dot + arm * float(a_dot[2])) / (2.0 - float(a[2]))
    v_xy = pv[:2] + v_z * a[:2] + arm * a_dot[:2]
    return np.array([v_xy[0], v_xy[1], v_z])


# ─────────────────────────────────────────────────────────────────────────────
# State
# ─────────────────────────────────────────────────────────────────────────────

@dataclasses.dataclass(frozen=True)
class CycleState:
    """The boundary condition a window is planned FROM.

    ``pose`` / ``pose_vel`` / ``pose_accel`` are the platform's STOW-frame state
    (mm, rad, and their time derivatives); ``hand_rev`` / ``hand_vel_rps`` the
    slider's, in the firmware's homed ODrive frame.  Together with
    ``detach_axis`` and ``post_release`` they are exactly what the plant reports
    and what the previous plan's terminal knot carries, so a state can be built
    from a measurement or from a chain without changing shape.  ``pose_accel``
    is carried for that shape and for callers that keep the whole state, but
    :meth:`to_cup_state` NEVER READS IT: the cup acceleration comes from
    :attr:`cup_accel_mm_s2` or from the post-release fallback, deliberately, for
    the reason the next paragraph gives.

    **``cup_pos_mm`` / ``cup_vel_mm_s`` / ``cup_accel_mm_s2`` — the exact-override
    fields, and why they exist.**  The QP's start-of-window rows treat the
    supplied cup position, velocity
    and acceleration as EXACT: the detach-cone equalities constrain the direction
    of the acceleration at knots 1..n_detach against ``detach_axis``, and their
    whole purpose is that the ball leaving the cup gets no lateral shove.
    Feeding a finite-differenced or measurement-noisy acceleration into that block
    would put noise straight into the one constraint the ball's trajectory depends
    on.  Both states this orchestrator actually starts from know the answer from
    physics rather than from a sensor — at rest the cup velocity and acceleration
    are zero; just after a release the velocity is the take-off velocity the QP
    pinned and the acceleration is ``g`` exactly — so those are supplied, not
    derived.  ``cup_pos_mm`` is the same discipline one step milder: the inverse
    map recovers it to ~1e-13 mm, which is harmless on its own but would make a
    chained window's first knot merely CLOSE to the previous window's last, and
    :func:`extend` exists to notice exactly that.  When these are ``None``,
    :meth:`to_cup_state` falls back to the inverse map for position, the analytic
    forward map for velocity, and ``g``-if-post-release / zero otherwise for
    acceleration; the fallbacks are documented there.
    """

    pose: np.ndarray
    pose_vel: np.ndarray
    pose_accel: np.ndarray
    hand_rev: float
    hand_vel_rps: float
    detach_axis: Optional[np.ndarray] = None
    post_release: bool = False
    cup_pos_mm: Optional[np.ndarray] = None
    cup_vel_mm_s: Optional[np.ndarray] = None
    cup_accel_mm_s2: Optional[np.ndarray] = None
    #: The gravity-levelling correction the PLAN frame is expressed in — the 3×3
    #: matrix ``levelling.correction_for_pose`` builds, or ``None`` for "this
    #: state's rotations ARE gravity-referenced" (every pure/sim caller, and an
    #: unlevelled node).  Contract **C-LEVEL-1/2 row E8**,
    #: ``ros_ws/docs/levelling_frame.md``.
    #:
    #: It is on the STATE rather than a keyword on :func:`plan_cycle` because it
    #: describes :attr:`pose`: knot 0 must equal the machine's *commanded* pose,
    #: which is plan-frame, while every aim this module pins is gravity-
    #: referenced.  Carrying it here also means a chained or re-planned window
    #: inherits the frame its predecessor was built in *for free* —
    #: :func:`release_state_from_meta` copies it off the meta — which is
    #: C-LEVEL-1's in-flight rule ("a live plan keeps the frame it was built
    #: in") rather than a second policy.  A chain whose two halves disagreed
    #: about the frame would put the whole correction (11.7 mrad ⇒ ~1.8 mm of
    #: cup lever) into one 25 ms seam, and :func:`_seam_check` would refuse it.
    levelling_correction: Optional[np.ndarray] = None

    # ── constructors ──

    @classmethod
    def at_rest(cls, pose, hand_rev, cfg=None, *,
                levelling_correction=None) -> 'CycleState':
        """A stationary platform + slider: the LAUNCH boundary condition."""
        pose = np.asarray(pose, dtype=float).reshape(6)
        return cls(pose=pose, pose_vel=np.zeros(6), pose_accel=np.zeros(6),
                   hand_rev=float(hand_rev), hand_vel_rps=0.0,
                   detach_axis=None, post_release=False,
                   cup_pos_mm=cup_state_from_platform(pose, hand_rev, cfg),
                   cup_vel_mm_s=np.zeros(3), cup_accel_mm_s2=np.zeros(3),
                   levelling_correction=levelling_correction)

    # ── conversion ──

    def to_cup_state(self, cfg=None) -> cc.CupState:
        """The SI :class:`cup_cycle.CupState` this window is solved from.

        Position: :attr:`cup_pos_mm` when supplied (the chained case, where it is
        the *same float* the previous window's terminal equality pinned), else the
        exact inverse map.  Velocity: :attr:`cup_vel_mm_s`, else the analytic
        forward map.  Acceleration: :attr:`cup_accel_mm_s2`, else ``g`` when this
        state follows a release (the free-fall equality the previous window ended
        on) and zero otherwise — the two cases the constructors cover.  A state
        that is neither at rest nor just-after-release MUST supply
        ``cup_accel_mm_s2``; see the class docstring for why nothing here tries to
        infer it.
        """
        cfg = cr.RealizeConfig() if cfg is None else cfg
        pos_mm = (cup_state_from_platform(self.pose, self.hand_rev, cfg)
                  if self.cup_pos_mm is None
                  else _vec3(self.cup_pos_mm, 'cup_pos_mm'))
        if self.cup_vel_mm_s is not None:
            vel_mm = _vec3(self.cup_vel_mm_s, 'cup_vel_mm_s')
        else:
            p = np.asarray(self.pose, dtype=float).reshape(6)
            pv = np.asarray(self.pose_vel, dtype=float).reshape(6)
            a = tg.cup_axis(float(p[3]), float(p[4]))
            _, a_dot = _cup_axis_rate(float(p[3]), float(p[4]),
                                      float(pv[3]), float(pv[4]))
            vel_mm = _cup_velocity(
                pv, float(self.hand_vel_rps) / HAND_REV_PER_M * 1000.0,
                float(pv[2]), a, a_dot, float(pos_mm[2]))
        if self.cup_accel_mm_s2 is not None:
            acc_mm = _vec3(self.cup_accel_mm_s2, 'cup_accel_mm_s2')
        else:
            acc_mm = _G_MM_S2.copy() if self.post_release else np.zeros(3)
        axis = (None if self.detach_axis is None
                else _vec3(self.detach_axis, 'detach_axis'))
        return cc.CupState(pos=pos_mm / 1000.0, vel=vel_mm / 1000.0,
                           acc=acc_mm / 1000.0, detach_axis=axis,
                           post_release=bool(self.post_release))


# ─────────────────────────────────────────────────────────────────────────────
# Metadata
# ─────────────────────────────────────────────────────────────────────────────

@dataclasses.dataclass(frozen=True)
class ReleaseMark:
    """One release on a plan's own clock.  Positions global mm, velocities mm/s."""

    t_s: float
    site_mm: np.ndarray
    vel_mm_s: np.ndarray
    flight_s: float
    target_mm: np.ndarray
    tilt: np.ndarray                  #: ``(rx, ry)`` throw tilt, rad
    #: Instant after which the hand motion belonging to THIS release has finished
    #: — a fact read off the plan itself (no reactive-arm model involved, since
    #: R1 deletes it).  ``None`` when the plan carries no knots after the release
    #: (the deceleration is in the next window); see :func:`plan_stroke_clear_s`.
    stroke_clear_s: Optional[float] = None


@dataclasses.dataclass(frozen=True)
class CatchMark:
    """One touch-down on a plan's own clock."""

    t_s: float                        #: INTERPOLATED touch-down, not ``knot·dt``
    knot: int                         #: ``catch_k`` — the knot at/just before it
    site_mm: np.ndarray
    vel_mm_s: np.ndarray              #: the BALL's arrival velocity
    #: Lead before touch-down at which the plan's hand catch motion begins — a
    #: fact read off the plan itself (no reactive-arm model involved, since R1
    #: deletes it).  See :func:`plan_arm_lead_s`.
    arm_lead_s: Optional[float] = None
    #: Slider travel (rev) left below the catch, minus what the achieved catch
    #: speed needs to stop in.  ``validate_cycle`` refuses at < 0; this is the
    #: margin it measured.
    runway_margin_rev: Optional[float] = None


@dataclasses.dataclass(frozen=True)
class CycleMeta:
    """Everything about a planned window that is not the trajectory itself.

    The scalar convenience fields mirror the LAST release and the FIRST catch,
    which for a single window is the only one of each; ``releases`` / ``catches``
    are the general form and are what a joined plan carries.
    """

    kind: str
    n_knots: int
    dt: float
    duration_s: float
    releases: Tuple[ReleaseMark, ...]
    catches: Tuple[CatchMark, ...]
    report: 'fz.FeasibilityReport'
    plan_wall_s: float
    tilts: np.ndarray                          #: (n, 2) the realised tilt schedule
    receive_tilt: np.ndarray                   #: (2,) the catch pin, rad
    throw_tilt: np.ndarray                     #: (2,) the release pin, rad
    #: Instant after which the hand motion of the release the window FOLLOWS has
    #: finished.  ``None`` when the window does not follow a release.
    stroke_clear_s: Optional[float] = None
    #: The TERMINAL window's take-off velocity (m/s), straight off
    #: ``cup_cycle.CupCyclePlan.takeoff_vel`` — a per-WINDOW quantity, not a
    #: per-plan one.  On a JOINED meta it is ``meta_b``'s, so a
    #: launch+landing pair reports **zeros** (a LANDING ends at rest and its
    #: sentinel take-off is zeros) even though the plan very much contains a
    #: throw.  For "the throw this plan makes" read
    #: :attr:`release_vel_mm_s` / ``releases[-1].vel_mm_s``, which follow the
    #: last RELEASE rather than the last window and are what
    #: :func:`announcement_fields` and :func:`release_state_from_meta` use.
    takeoff_vel_mps: Optional[np.ndarray] = None
    warm_start: Optional['cc.SolverState'] = None
    #: The source cup trajectory.  Present for a single window (and for a spliced
    #: one, where the two halves are concatenated on the joint clock); it is what
    #: :func:`_seam_velocity` reads the exact cup state at the seam knot from.
    cup_plan: Optional['cc.CupCyclePlan'] = None
    goals: Optional[CycleGoals] = None
    #: The levelling frame this plan was BUILT in — the same matrix
    #: :attr:`CycleState.levelling_correction` carried in, recorded so that
    #: everything that continues this plan re-uses it rather than re-reading a
    #: correction that may have changed since.  ``None`` ⇒ the plan's rotations
    #: are gravity-referenced (no correction was supplied).
    #:
    #: This IS C-LEVEL-1's in-flight rule, expressed as data: a `/gravity_offset`
    #: or a `reload_tilt_map` landing mid-cycle does not re-frame the installed
    #: plan, because :func:`release_state_from_meta` reads the frame from HERE.
    #: Re-reading the node's live correction instead
    #: would step the commanded tilt by the whole delta on one 25 ms knot, at a
    #: seam whose two halves would then disagree by ~1.8 mm of cup lever.
    levelling_correction: Optional[np.ndarray] = None
    #: Worst per-channel velocity disagreement at a splice seam (mm/s and rev/s).
    #: Zero for a single window.  Recorded rather than gated: ``validate_cycle``
    #: on the spliced whole is the authority on whether the seam is executable,
    #: and it measures the Hermite the emitter will actually sample.
    seam_vel_mismatch: Optional[Tuple[float, float]] = None
    #: Which knots :attr:`report`'s ``peak_*`` fields describe, as ``[a, b)``.
    #:
    #: ``None`` means **the whole plan**, which is the case for every window
    #: :func:`plan_cycle` produces and for :func:`extend`'s output (its head is
    #: unchanged, so the head's own peaks are still true and are merged back in
    #: exactly — every ``peak_*`` is a maximum, so the max of the two halves' maxima
    #: IS the whole plan's).
    #:
    #: A construction path that re-gates only PART of the plan (see
    #: :func:`_gate_joined`) carries a RANGE, and it must: the ungated portion's
    #: report describes trajectory a bounded gate never re-checked, and merging
    #: it in would report a stale number as live — a peak above a limit sitting
    #: next to ``ok=True``, which is the worst kind of diagnostic.  So the
    #: re-gated range's peaks are reported alone and this field says so, and a
    #: consumer that needs a whole-plan peak must ask for one
    #: (``validate_cycle`` on the plan) rather than assume.
    report_range_knots: Optional[Tuple[int, int]] = None
    #: Per-stage wall time, seconds, keyed :data:`STAGE_KEYS`.  ``None`` when the
    #: meta did not come from :func:`plan_cycle` directly (a spliced/joined meta).
    #:
    #: **Attribution, added 2026-09-06.** When a solve blows out — 1655.1 /
    #: 2021.2 / 2158.9 ms on the UH-3 attempt against ~200 ms nominal — the
    #: single ``plan_wall_s`` number cannot say WHERE, so every hypothesis has to
    #: be tested by re-running the whole thing offline (and two of the 2026-09-06
    #: hypotheses were withdrawn on measurements taken below the knee).  The
    #: split makes the next slow solve attribute itself from its own log line.
    #:
    #: **The shape is not what the name suggests, and it was measured on the
    #: first run of this field (2026-09-06, venv, idle box):**
    #:
    #: ===========  =====  ======  =====  =======  =====  =======
    #: window        qp     tilt    dec     val     cont   total
    #: ===========  =====  ======  =====  =======  =====  =======
    #: 0.6 s LAUNCH   3.3     1.2    1.1     63.0    0.5    69.1
    #: 1.0 s LANDING  6.7     3.9    2.6    117.7    0.6   131.5
    #: 1.4 s STEADY  11.2     4.2    3.7    163.9    0.7   183.6
    #: ===========  =====  ======  =====  =======  =====  =======
    #:
    #: **``validate_cycle`` is ~89 % of the solve and the QP is ~6 %** — so
    #: "the planner is slow" has always meant "the GATE is slow".  That is not a
    #: complaint (the gate walks every knot of a seven-channel plan through the
    #: kinematics, and it is the reason nothing unexecutable reaches the wire).
    #:
    #: **UNDER STARVATION THE SHAPE INVERTS, which is what makes the split a
    #: DIAGNOSTIC and not just a curiosity** (measured on the same day, three
    #: busy cores of six, ``--load 3``):
    #:
    #: ===================  ========  ======  =====  =======  ========
    #: arm                     qp      tilt    dec     val     total
    #: ===================  ========  ======  =====  =======  ========
    #: uncapped pool, rep 1  **2256.4**   21.7    3.7    636.7    2920.3
    #: uncapped pool, rep 2       6.7   18.6    3.7    609.1     641.1
    #: capped, rep 1             10.4    4.8    4.9    186.4     208.3
    #: capped, rep 2              7.8    6.3    5.0    195.6     216.7
    #: ===================  ========  ======  =====  =======  ========
    #:
    #: The gate inflates ~3x under starvation (186 → 609-637 ms) — bad but
    #: proportionate.  The QP inflates **~200x** on the rep that has no warm
    #: start (10 → 2256 ms), because that is where the densest run of tiny numpy
    #: calls lives and every one of them pays a scheduler round trip.  So the
    #: read is:
    #:
    #: * ``val`` large, ``qp`` small — the NORMAL shape, just a big window.
    #: * ``qp`` comparable to or larger than ``val`` — **thread-pool
    #:   starvation**: check the ``blas threads:`` line and ``load1``.
    #:
    #: (Note the warm start: rep 2's ``qp`` is 6.7 ms even uncapped, because it
    #: re-uses rep 1's factorisation.  A cold QP is the exposed one.)
    #:
    #: The five keys **sum to** ``plan_wall_s`` by construction — ``cont`` is the
    #: residual, so nothing can hide between the stages.
    stage_wall_s: Optional[Dict[str, float]] = None

    # ── convenience views ──

    @property
    def t_release_s(self) -> Optional[float]:
        return self.releases[-1].t_s if self.releases else None

    @property
    def release_vel_mm_s(self) -> Optional[np.ndarray]:
        return self.releases[-1].vel_mm_s if self.releases else None

    @property
    def release_site_mm(self) -> Optional[np.ndarray]:
        return self.releases[-1].site_mm if self.releases else None

    @property
    def t_catch_s(self) -> Optional[float]:
        return self.catches[0].t_s if self.catches else None

    @property
    def catch_k(self) -> int:
        return self.catches[0].knot if self.catches else -1

    @property
    def catch_site_mm(self) -> Optional[np.ndarray]:
        return self.catches[0].site_mm if self.catches else None

    @property
    def catch_vel_mm_s(self) -> Optional[np.ndarray]:
        return self.catches[0].vel_mm_s if self.catches else None

    @property
    def arm_lead_s(self) -> Optional[float]:
        return self.catches[0].arm_lead_s if self.catches else None

    @property
    def runway_margin_rev(self) -> Optional[float]:
        return self.catches[0].runway_margin_rev if self.catches else None


def detach_knots(cup_cfg=None) -> int:
    """Knots after a release that carry that ball's detach-cone equalities.

    ``cup_cycle``'s ``n_detach`` is THE number — the QP pins the cup's
    acceleration DIRECTION at knots ``k_rel+1 .. k_rel+n_detach`` so the ball
    already out of the cup gets no lateral shove — and every consumer of it in
    this module and above reads it through here rather than keeping a copy.
    """
    return int(cc.CupCycleConfig.n_detach if cup_cfg is None
               else getattr(cup_cfg, 'n_detach', cc.CupCycleConfig.n_detach))


def release_state_at_knot(plan: CyclePlan, meta: CycleMeta, k: int,
                          cfg=None) -> CycleState:
    """The POST-RELEASE boundary condition at the release sitting on knot ``k``.

    :func:`state_at_knot` is the seed for an ordinary interior knot, and it is
    deliberately ``post_release=False`` / ``detach_axis=None`` — no ball leaves
    the cup in the middle of a carry.  At a RELEASE knot that is the wrong
    state: a ball leaves there, and a window seeded without the cone would
    re-solve the very knots whose acceleration direction the ball's flight
    depends on (1.126 m/s² of off-axis specific force against an original
    4.4e-16 — see :func:`_refuse_splice_into_a_detach_cone`).

    So this is the interior spelling of :func:`release_state_from_meta`, and
    the same floats: the mark on knot ``k`` supplies the site, the take-off
    velocity, ``g`` and the detach axis (the direction the ball actually left
    along), the plan's knot ``k`` supplies pose and hand, and the levelling
    frame is the one the PLAN was built in (row E8's in-flight rule).  At the
    TERMINAL knot it equals :func:`release_state_from_meta` field for field,
    which is how the two stay one path rather than two spellings.

    ``ValueError`` when knot ``k`` carries no release: a caller asking for a
    post-release seed where no ball left is a caller that has mislaid a mark.
    """
    k = int(k)
    dt = float(meta.dt)
    mark = None
    for m in meta.releases:
        if int(round(float(m.t_s) / dt)) == k:
            mark = m
            break
    if mark is None:
        raise ValueError(
            'knot %d carries no release (this plan releases at knots %s) — '
            'only a knot a ball actually leaves the cup on has a post-release '
            'state to hand on'
            % (k, [int(round(float(m.t_s) / dt)) for m in meta.releases]))
    return CycleState(
        pose=plan.pose[k].copy(),
        pose_vel=plan.pose_vel[k].copy(),
        pose_accel=np.zeros(6),
        hand_rev=float(plan.hand_rev[k]),
        hand_vel_rps=float(plan.hand_vel_rps[k]),
        detach_axis=tg.cup_axis(float(mark.tilt[0]), float(mark.tilt[1])),
        post_release=True,
        cup_pos_mm=np.asarray(mark.site_mm, dtype=float).copy(),
        cup_vel_mm_s=np.asarray(mark.vel_mm_s, dtype=float).copy(),
        cup_accel_mm_s2=_G_MM_S2.copy(),
        levelling_correction=meta.levelling_correction,
    )


def release_state_from_meta(meta: CycleMeta, plan: CyclePlan,
                            cfg=None) -> CycleState:
    """The :class:`CycleState` the NEXT window chains from, exactly.

    Windows abut at a release instant, and "exactly" is load-bearing: the cup
    position, velocity and acceleration are carried across as the same floats the
    finished window's terminal equality pinned (site, take-off velocity, ``g``),
    not re-derived from the pose through the inverse map.  Round-tripping instead
    would cost ~1e-11 mm — harmless on its own, but it would make the two windows'
    shared knot merely *close*, and :func:`extend`'s seam check exists precisely
    to notice when they are not the same.

    ``detach_axis`` is the throw tilt's cup axis: the direction the ball actually
    left along, which is what the next window's detach-cone rows are written
    against.

    The levelling frame is carried across too (row E8): the next window is
    planned in the frame this one was BUILT in, not in whatever correction the
    node holds by the time it is called.  That is C-LEVEL-1's in-flight rule, and
    it is also what keeps the seam exact — both halves apply the same matrix to
    the same ``detach_axis``-derived tilt, so :func:`_seam_check` sees 0.0.

    This is the TERMINAL case of :func:`release_state_at_knot` — it checks that
    the window really does end at a release and then asks for the state at the
    last knot.  One path: an interior splice landing on a release knot gets the
    same floats through the same code.
    """
    if not meta.releases:
        raise ValueError(
            "meta carries no release: only a window that ENDS at a release can "
            "be chained from (kind=%r)" % meta.kind)
    mark = meta.releases[-1]
    if abs(mark.t_s - meta.duration_s) > 1e-9:
        raise ValueError(
            "the chained release must be the window's terminal one (release at "
            "%.4f s, window ends at %.4f s)" % (mark.t_s, meta.duration_s))
    return release_state_at_knot(plan, meta, int(plan.n_knots) - 1, cfg)


def is_release_terminal(meta: CycleMeta) -> bool:
    """True when the plan's LAST knot is a release — i.e. it ends in MOTION.

    :data:`LAUNCH` and :data:`STEADY` are release-terminal by construction;
    :data:`LANDING` and :data:`SETTLE` end at rest, and a :data:`JOINED` /
    :data:`SPLICED` plan inherits whichever its last window was.  The test is
    on the release INSTANT rather than on ``meta.kind`` so a spliced meta that
    carries a spent release in its middle is not mistaken for one.
    """
    return bool(meta.releases
                and abs(float(meta.releases[-1].t_s) - float(meta.duration_s))
                <= 1e-9)


# ─────────────────────────────────────────────────────────────────────────────
# Configuration builders
# ─────────────────────────────────────────────────────────────────────────────

def build_realize_config(limits, *, banking: bool = True,
                         z_float: Optional[bool] = None,
                         z_band_mm: Optional[float] = None) -> cr.RealizeConfig:
    """A :class:`cup_realize.RealizeConfig` whose tilt caps follow ``limits``.

    ``cup_realize.TILT_ACCEL_LIMIT_DEFAULT_RAD_S2`` is derived from the SHIPPED
    leg acceleration limit (``JB_TRAJ_LEG_ACC_LIMIT_MMPS2``, 5000 mm/s²) at import
    time, through the lever ``TILT_ACCEL_LEVER_MM``.  A unified sitting raises the
    session limits at session start — that is the settled pattern, and the Phase-1
    gate itself runs at leg-acc 3000 — so the shipped constant is the WRONG cap
    twice over: at a lowered session limit the banking schedule is shaped for
    accelerations the session forbids and ``validate_cycle`` refuses cycles that
    a correctly-shaped schedule would have flown; at a raised one the schedule is
    needlessly smooth and gives away tilt authority the session paid for.  Neither
    is a safety failure — ``validate_cycle`` is still the gate — but both are
    silent, and the cap is a *derived* number with a live input, so it is
    re-derived here from the same expression and the same lever.

    The tilt-JERK cap (C-CUP-3, the widen loop's second exit condition) is
    derived the same way from ``limits.leg_jerk_mmps3``, for the same reason and
    with a stronger one on top: the session leg-jerk limit is the number
    ``validate_cycle`` refuses ``LIMIT_JERK`` against, so a schedule shaped for
    the SHIPPED jerk limit while the session runs at another is shaped against a
    bound nothing enforces.  One map, one lever, two limits.

    The jerk cap built here is the STATIC-lever one — the cap for a cycle with no
    vertical cup motion.  ``cup_realize.tilt_schedule`` re-derives it per call
    from the cup plan in hand (``cup_realize._tilt_jerk_lever_mm``), because the
    product-rule content the third difference picks up off a moving lever arm is
    a property of the cycle, not of the geometry.  The session limit still
    enters exactly here, and exactly once.
    """
    accel_cap = (cr.TILT_ACCEL_BUDGET_FRACTION * float(limits.leg_acc_mmps2)
                 / cr.TILT_ACCEL_LEVER_MM)
    jerk_cap = (cr.TILT_ACCEL_BUDGET_FRACTION * float(limits.leg_jerk_mmps3)
                / cr.TILT_ACCEL_LEVER_MM)
    kwargs = dict(banking_enabled=bool(banking),
                  tilt_accel_limit_rad_s2=accel_cap,
                  tilt_jerk_limit_rad_s3=jerk_cap)
    if z_float is not None:
        kwargs['z_float_enabled'] = bool(z_float)
    if z_band_mm is not None:
        kwargs['z_band_mm'] = float(z_band_mm)
    return cr.RealizeConfig(**kwargs)


def build_cup_config(**overrides) -> cc.CupCycleConfig:
    """A :class:`cup_cycle.CupCycleConfig` boxed to the slider-reachable cup band.

    ``CupCycleConfig``'s own ``z_min_m`` / ``z_max_m`` defaults (0.45 / 1.10) are
    the sim planner's and are far wider than the slider can reach; its docstring
    says the caller MUST override them.  Left at the defaults the realisation
    saturates the stroke clamp at most knots and the gate refuses.  These come
    from the config through :data:`_CUP_Z_BOTTOM_M` / :data:`_CUP_Z_TOP_M`, so a
    change to the stroke or the prime rev ripples here instead of drifting.

    **The runway floor is the STROKE floor, not the box floor**, and the two are
    different numbers on purpose.  ``catch_runway_z_floor_m`` is defined by
    ``cup_cycle`` as "cup z with the slider at the BOTTOM of its stroke" — the
    height below which there is no slider left to decelerate into — and that is
    :data:`_CUP_Z_BOTTOM_M` (0.6796 m), the realisation of
    ``feasibility.HAND_STROKE_MIN_REV`` (0.0 rev), which is the same floor
    ``validate_cycle``'s runway pass measures against.  The POSITION box floor is
    one :data:`_CUP_Z_INSET_M` above it, because a knot is kept off the clamp;
    feeding that inset number as the runway floor made the analytic gate believe
    10 mm less runway existed than the gate downstream of it allows, i.e. two
    layers disagreeing about the same physical stop.  Corrected 2026-09-04.
    """
    kwargs = dict(z_min_m=_CUP_Z_BOTTOM_M + _CUP_Z_INSET_M,
                  z_max_m=_CUP_Z_TOP_M - _CUP_Z_INSET_M,
                  catch_runway_z_floor_m=_CUP_Z_BOTTOM_M,
                  catch_runway_enabled=True)
    kwargs.update(overrides)
    return cc.CupCycleConfig(**kwargs)


# ─────────────────────────────────────────────────────────────────────────────
# The timing twins
# ─────────────────────────────────────────────────────────────────────────────

def _knot_at_or_after(t_s: float, dt: float, n: int) -> int:
    return int(min(max(0, int(math.ceil(t_s / dt - 1e-9))), n - 1))


#: Bisections used to solve the hand-velocity zero inside one span.  40 halvings
#: of a 0.025 s span resolve it to 2.3e-14 s — float noise against a millisecond
#: consumer — and cost 40 scalar ``hand_at`` calls, run at most twice per plan.
_ZERO_CROSSING_BISECTIONS = 40


def _zero_crossing_s(plan: CyclePlan, k0: int, k1: int) -> float:
    """Time of the hand-velocity zero between knots ``k0`` and ``k1``.

    Solved on the plan's OWN curve, not interpolated between the two knot
    velocities.  The hand channel is a cubic Hermite, so its velocity is a
    QUADRATIC in the span parameter and the endpoints do not determine where
    inside the span it crosses: a straight line through them is only as good as
    the curvature is small.  That held while the hand left the catch from REST
    (the quadratic is nearly linear near a stationary point) and stopped holding
    when C-CUP-2's contact floor gave the cup a hover-then-dive: MEASURED
    (2026-09-20, ``scratchpad/probe_u7f.py``) on the reference 1.4 s STEADY, the
    linear estimate put the reversal at t = 0.580719 s where the plan's own hand
    velocity is still **+1.197 rev/s**, i.e. ~2 ms early.

    A quadratic whose endpoint values have opposite signs has exactly ONE root
    between them (two roots inside a bracket would leave the endpoints with the
    SAME sign), so bisection on the accessor converges to the root the callers
    mean — the first one forward for :func:`plan_stroke_clear_s`, the last one
    backward for :func:`plan_arm_lead_s`.  Both callers only reach this function
    after finding a strict sign change; the same-sign branch is kept total and
    falls back to the linear estimate the endpoints do support.

    Both consumers are REPORTING fields (``stroke_clear_s`` / ``arm_lead_s`` on
    the service response); nothing on the leg or hand command path reads them, so
    this changes a published number by ~2 ms and no commanded motion at all.
    """
    v0 = float(plan.hand_vel_rps[k0])
    v1 = float(plan.hand_vel_rps[k1])
    t0, t1 = float(plan.t[k0]), float(plan.t[k1])
    if v0 == v1:
        return t1
    if v0 * v1 > 0.0:                      # no bracketed sign change to solve
        frac = v0 / (v0 - v1)
        return t0 + max(0.0, min(1.0, frac)) * plan.dt
    lo, hi, sign0 = t0, t1, (1.0 if v0 > 0.0 else -1.0)
    for _ in range(_ZERO_CROSSING_BISECTIONS):
        mid = 0.5 * (lo + hi)
        v_mid = float(plan.hand_at(mid)[1])
        if v_mid == 0.0:
            return mid
        if (1.0 if v_mid > 0.0 else -1.0) == sign0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


#: Margin added past the hand's planned zero-velocity instant when answering
#: "when is the stroke clear?" — the announcement's measured earliness against
#: the physical release. Formerly ``hand_stroke.ARM_SUPPRESS_MARGIN_S``; that
#: module (and the reactive-arm model it sized this for) is deleted at R1, so
#: the value is now standalone here. Unchanged: 0.040 s.
_ARM_SUPPRESS_MARGIN_S = 0.040


def plan_stroke_clear_s(plan: CyclePlan, t_release_s: float, *,
                        eps_rps: float = _HAND_REST_EPS_RPS,
                        margin_s: float = _ARM_SUPPRESS_MARGIN_S
                        ) -> Optional[float]:
    """When the plan's own hand motion is clear of this release.

    Pre-R1 this answered "when can a scheduled command no longer land inside a
    live throw stroke?" by MODELLING the legacy firmware stroke engine's
    deceleration from the announced release velocity, via ``hand_stroke``
    (deleted at R1 with the reactive-arm/stroke-engine path it modelled).
    Under unified mode there never was a stroke engine: the hand's motion is in
    the plan, sampled from the same clock as the platform, so the answer is a
    fact about the plan rather than a model of a device — this function is
    unchanged by that deletion.

    **Definition.** The first instant at or after ``t_release_s`` at which the
    planned hand velocity reaches zero — its first stationary point, found on the
    knot grid and linearly interpolated inside the span that brackets it — plus
    ``margin_s`` (:data:`_ARM_SUPPRESS_MARGIN_S`).

    Returns ``None`` when the plan carries no such instant — the common case for a
    window that ENDS at its release, where the deceleration belongs to the next
    window.  A caller that needs the number for a terminal release reads it off
    the joined plan (:func:`extend`), which does contain it.
    """
    n = int(plan.n_knots)
    k0 = _knot_at_or_after(float(t_release_s), plan.dt, n)
    if abs(float(plan.hand_vel_rps[k0])) <= eps_rps:
        return float(plan.t[k0]) + float(margin_s)
    for k in range(k0 + 1, n):
        v = float(plan.hand_vel_rps[k])
        if abs(v) <= eps_rps:
            return float(plan.t[k]) + float(margin_s)
        if v * float(plan.hand_vel_rps[k - 1]) < 0.0:
            return _zero_crossing_s(plan, k - 1, k) + float(margin_s)
    return None


def plan_arm_lead_s(plan: CyclePlan, t_catch_s: float, *,
                    eps_rps: float = _HAND_REST_EPS_RPS) -> Optional[float]:
    """How long before touch-down the plan's hand motion into the catch begins.

    Pre-R1 this answered "how much lead does a REACTIVE catch arm need before
    its event, or the Teensy refuses the dispatch?" via ``hand_stroke`` (deleted
    at R1 with the reactive-arm path it modelled). Under unified mode nothing is
    armed — the catch stroke is already in the plan — so the question is the one
    a consumer actually still needs answered: **how long before touch-down does
    the plan's hand motion into the catch begin?**  That is the window during
    which the hand is committed, and it is what a suppression or possession
    consumer has to respect. Unchanged by the R1 deletion.

    **Definition.** ``t_catch_s`` minus the start of the contiguous run of moving
    hand that contains the touch-down — i.e. the last instant strictly before the
    catch at which the planned hand velocity was zero, interpolated inside its
    span.  When the hand is moving from the window start (no such instant), the
    motion began at the window start and the lead is ``t_catch_s`` itself.
    Returns ``None`` only when the catch time is not inside the plan.
    """
    n = int(plan.n_knots)
    t_catch = float(t_catch_s)
    if not 0.0 <= t_catch <= plan.total_duration + 1e-12:
        return None
    k_c = min(int(t_catch / plan.dt), n - 1)
    for k in range(k_c, 0, -1):
        v = float(plan.hand_vel_rps[k])
        if abs(v) <= eps_rps:
            return t_catch - float(plan.t[k])
        if v * float(plan.hand_vel_rps[k - 1]) < 0.0:
            return t_catch - _zero_crossing_s(plan, k - 1, k)
    if abs(float(plan.hand_vel_rps[0])) <= eps_rps:
        return t_catch - float(plan.t[0])
    return t_catch


# ─────────────────────────────────────────────────────────────────────────────
# Announcement
# ─────────────────────────────────────────────────────────────────────────────

def announcement_fields(meta: CycleMeta, t_release_wall_s: float) -> dict:
    """The physics fields of the self-``ThrowAnnouncement``, from the PLAN.

    Same six keys, same units and same frame (global mm, mm/s, seconds) as
    ``toss_release.build_announcement_fields``, so every downstream consumer —
    the tracker's correlation, possession, suppression — is unchanged.  The
    difference is where the numbers come from: the legacy builder computes them
    from a ``ReleaseState`` it derived from the goal, this one reads them off the
    trajectory that will actually be executed.

    ``landing_position`` is the BALLISTIC landing of the planned release state,
    not the nominated target: with the release velocity pinned by hard equality to
    ``takeoff_velocity(site, target, T)`` under the same 9806 mm/s² gravity that
    ``ballistics_bc`` uses, the two agree to float precision — and if they ever
    stop agreeing, the announcement must carry where the ball is going, not where
    it was asked to go.  ``ballistics_bc`` is the one gravity source on both
    sides, imported rather than re-derived.

    ``t_release_wall_s`` is the ABSOLUTE clock instant of the release (the plan's
    install time plus :attr:`CycleMeta.t_release_s`).
    """
    if not meta.releases:
        raise ValueError("meta carries no release to announce (kind=%r)"
                         % meta.kind)
    mark = meta.releases[-1]
    site = np.asarray(mark.site_mm, dtype=float)
    vel = np.asarray(mark.vel_mm_s, dtype=float)
    return dict(
        initial_position=site,
        initial_velocity=vel,
        predicted_tof_sec=float(mark.flight_s),
        landing_position=ballistics_bc.position_at(site, vel, mark.flight_s),
        landing_velocity=ballistics_bc.arrival_velocity(vel, mark.flight_s),
        landing_time_s=float(t_release_wall_s) + float(mark.flight_s),
    )


# ─────────────────────────────────────────────────────────────────────────────
# Planning
# ─────────────────────────────────────────────────────────────────────────────

def _require(goals: CycleGoals, kind: str, *names) -> None:
    missing = [n for n in names if getattr(goals, n) is None]
    if missing:
        raise ValueError("kind %r needs %s on the goal" % (kind, ', '.join(missing)))


def _events_for(kind: str, goals: CycleGoals):
    """``(events, settle_site_m)`` in SI for ``plan_window``."""
    has_throw, has_catch, _ = _KIND_SHAPE[kind]
    events = []
    settle_m = None
    if has_catch:
        _require(goals, kind, 'catch_site_mm', 'catch_vel_mm_s')
        # ``vel`` stays the RAW arrival: the QP projects it onto the held axis
        # itself (one ratio, the stroke's), so the projection is stated once,
        # where the rows that use it are written.
        axis = (None if goals.hold_tilt is None
                else tg.cup_axis(float(goals.hold_tilt[0]),
                                 float(goals.hold_tilt[1])))
        events.append(cc.CatchEvent(
            ball_id=int(goals.ball_id), t_s=goals.catch_time_s(),
            site=_vec3(goals.catch_site_mm, 'catch_site_mm') / 1000.0,
            vel=_vec3(goals.catch_vel_mm_s, 'catch_vel_mm_s') / 1000.0,
            axis=axis))
    if has_throw:
        _require(goals, kind, 'throw_site_mm', 'throw_target_mm', 'flight_s')
        events.append(cc.ThrowEvent(
            ball_id=int(goals.ball_id) + 1, t_s=float(goals.period_s),
            site=_vec3(goals.throw_site_mm, 'throw_site_mm') / 1000.0,
            target=_vec3(goals.throw_target_mm, 'throw_target_mm') / 1000.0,
            flight_s=float(goals.flight_s)))
    else:
        site = goals.settle_site_mm
        if site is None:
            site = goals.catch_site_mm
        if site is None:
            raise ValueError(
                "kind %r ends at REST and has no catch to default from, so "
                "settle_site_mm is required" % kind)
        settle_m = _vec3(site, 'settle_site_mm') / 1000.0
    return events, settle_m


#: The stages :attr:`CycleMeta.stage_wall_s` splits ``plan_wall_s`` into, in the
#: order they run.  ``cont`` ("the rest of the construction") is the RESIDUAL —
#: ``CyclePlan.from_realized``, the release/catch marks, the argument checks and
#: the config builds — so the five always sum to ``plan_wall_s`` exactly and no
#: time can hide between them.
STAGE_KEYS: Tuple[str, ...] = ('qp', 'tilt', 'dec', 'val', 'cont')


def format_stage_wall_ms(stage_wall_s: Optional[Dict[str, float]]) -> str:
    """``qp=181.2 tilt=3.4 dec=8.1 val=6.0 cont=1.3 ms``, or ``''``.

    The operator-facing spelling of :attr:`CycleMeta.stage_wall_s`.  Empty string
    when the meta carries no split, so a caller can concatenate unconditionally.
    """
    if not stage_wall_s:
        return ''
    return ' '.join('%s=%.1f' % (k, 1e3 * float(stage_wall_s.get(k, 0.0)))
                    for k in STAGE_KEYS) + ' ms'


def merge_stage_wall(a: Optional[Dict[str, float]],
                     b: Optional[Dict[str, float]],
                     extra_cont_s: float = 0.0) -> Optional[Dict[str, float]]:
    """Sum two splits stage-by-stage (a JOINED plan is two solves plus a join).

    ``extra_cont_s`` is the join's own cost and lands in ``cont``, which keeps
    the sum-to-``plan_wall_s`` invariant true for :func:`extend` as well.
    ``None`` when neither side carries a split.
    """
    if a is None and b is None:
        return None
    out = {k: 0.0 for k in STAGE_KEYS}
    for src in (a, b):
        for k in STAGE_KEYS:
            out[k] += float((src or {}).get(k, 0.0))
    out['cont'] += float(extra_cont_s)
    return out


def plan_cycle(kind: str, goals: CycleGoals, state: CycleState,
               limits, geom, *,
               cup_cfg: Optional[cc.CupCycleConfig] = None,
               realize_cfg: Optional[cr.RealizeConfig] = None,
               warm_start: Optional['cc.SolverState'] = None
               ) -> Tuple[CyclePlan, CycleMeta]:
    """Plan ONE window and gate it.  ``(CyclePlan, CycleMeta)``, or a refusal.

    Runs ``plan_window → tilt_schedule → decompose → CyclePlan.from_realized →
    validate_cycle`` and raises :class:`CycleInfeasible` if any of them refuses.
    Every refusal carries the layer's own code as the subcode, so an operator
    reading one outcome string knows whether the cup trajectory did not exist
    (``CATCH_RUNWAY``), the tilt aim was out of range (``TILT_PIN``) or the
    realised motion breached the machine (``LIMIT_JERK``, ``HAND_STROKE``).

    ``kind`` is one of :data:`LAUNCH` / :data:`STEADY` / :data:`LANDING` /
    :data:`SETTLE`. ``state.post_release`` is the CALLER'S, and the check on it
    is one-way: a state may not claim a release the kind cannot have had (that
    disagreement means the caller believes something about the ball the planner
    does not), but a post-release KIND may be planned from a state that did NOT
    follow a release. That case is real — a :data:`SETTLE` or :data:`LANDING`
    issued at ``MODE_NEW`` off a terminal hold — and it is not a formality:
    ``post_release`` is what decides whether ``cup_cycle`` assembles the detach-
    cone equalities, which pin the acceleration DIRECTION of the first knots so a
    ball leaving the cup gets no lateral shove. Asserting it off a hold pins
    those knots for a ball that does not exist (see
    :class:`cup_cycle.CupState`), which on a lateral carry forbids the cup from
    accelerating sideways out of rest at all.

    ``limits`` drives BOTH the gate and — through :func:`build_realize_config`
    when ``realize_cfg`` is not supplied — the banking schedule's acceleration
    cap, so a session limit change reshapes the plan rather than only re-judging
    it.
    """
    t_wall = time.perf_counter()
    if kind not in _KIND_SHAPE:
        raise ValueError("unknown window kind %r (expected one of %s)"
                         % (kind, ', '.join(KINDS)))
    has_throw, has_catch, post_release = _KIND_SHAPE[kind]
    if bool(state.post_release) and not post_release:
        raise ValueError(
            "kind %r requires state.post_release=%s, got %s — the two describe "
            "the same window and must agree"
            % (kind, post_release, bool(state.post_release)))
    if not float(goals.period_s) > 0.0:
        raise ValueError("period_s must be > 0, got %r" % (goals.period_s,))
    # The two attitude fields are checked per kind, like the site fields: one
    # window can HOLD an attitude through a catch or END at one, never both —
    # a slew inside a held line is the thing the held line exists to forbid.
    if goals.hold_tilt is not None and goals.rest_tilt is not None:
        raise ValueError(
            "give hold_tilt OR rest_tilt, not both: a window that holds the "
            "receive attitude through its catch cannot also slew to another "
            "one, and the slew is what breaks the held line.")
    if goals.hold_tilt is not None and kind != LANDING:
        raise ValueError(
            "hold_tilt is a %s field: only a window that catches and then rests "
            "can hold the receive attitude (a release needs its own take-off "
            "tilt). Got kind %r." % (LANDING, kind))
    if goals.rest_tilt is not None and kind != SETTLE:
        raise ValueError(
            "rest_tilt is a %s field: only a window that just moves and stops "
            "can choose the attitude it stops at. Got kind %r." % (SETTLE, kind))

    cup_cfg = build_cup_config() if cup_cfg is None else cup_cfg
    # A held or chosen attitude turns BANKING off: the banking objective points
    # the cup down the apparent-gravity field, which is a second, disagreeing
    # answer to the question the attitude field has already answered.
    rcfg = (build_realize_config(limits, banking=(bool(goals.banking_enabled)
                                                  and goals.hold_tilt is None
                                                  and goals.rest_tilt is None))
            if realize_cfg is None else realize_cfg)

    events, settle_m = _events_for(kind, goals)
    state0 = state.to_cup_state(rcfg)

    # Per-stage attribution (2026-09-06). See CycleMeta.stage_wall_s: one
    # `plan_wall_s` cannot say WHERE a blown-out solve went. (Measured on the
    # first run of this field: the gate, not the QP, is ~89 % of it.)
    stages: Dict[str, float] = {}
    t_stage = time.perf_counter()
    try:
        cup = cc.plan_window(events, state0, cup_cfg,
                             period_s=float(goals.period_s),
                             settle_site=settle_m, warm_start=warm_start,
                             holds_ball_at_start=(kind == LAUNCH
                                                  or bool(goals.holds_ball)))
    except cc.CupCycleInfeasible as exc:
        raise CycleInfeasible(exc.reason, [str(exc)])
    finally:
        stages['qp'] = time.perf_counter() - t_stage

    plan, meta = _realize(kind, cup, goals, state, limits, geom, rcfg,
                          t_wall=t_wall, stages=stages)
    return plan, meta


def _throw_tilt_for(cup, max_tilt_deg: float) -> np.ndarray:
    """The terminal cup attitude: the take-off direction, or LEVEL at rest.

    ``tilt_to_throw`` maps a zero-magnitude velocity to ``(0, 0)``, which is what
    a ``takeoff_vel`` of zeros (a window ending at rest) means — the cup is held
    upright over a seated ball.  So there is no branch for that; the sentinel does
    the work.  See :class:`cup_cycle.CupCyclePlan` on why the sentinel is zeros.

    **An aim past the usable cone is REFUSED here, not clamped.**  The ball
    detaches up the cup's symmetry axis, so the lateral component of the throw is
    delivered by the tilt; ``tilt_to_throw`` SATURATES past ``max_tilt_deg``, and
    a saturated aim is a ball that lands somewhere other than where it was aimed —
    silently, with no error anywhere.  ``toss_release`` set the precedent for the
    aimed tier ("gate the aim, don't rely on the clamp") and the plan carries the
    same obligation for the aimed unified rungs.  The CATCH pin keeps the clamp,
    deliberately: a partially-nulled fast arrival is a worse catch, not a wrong
    destination.
    """
    v = np.asarray(cup.takeoff_vel, dtype=float).reshape(3)
    speed = float(np.linalg.norm(v))
    if speed > 0.0:
        angle_deg = float(np.degrees(np.arccos(
            np.clip(v[2] / speed, -1.0, 1.0))))
        if angle_deg > float(max_tilt_deg) * (1.0 + 1e-9):
            raise CycleInfeasible(TILT_PIN, [bound_msg(
                'throw aim', angle_deg, '>', max_tilt_deg, unit='deg',
                knob='tilt_geometry.MAX_TILT_DEG', digits=3,
                tail=('the ball leaves along the cup axis, so a tilt past the '
                      'usable cone SATURATES and lands the throw off target '
                      'without reporting anything — move the target closer or '
                      'lengthen the flight'))])
    return np.asarray(tg.tilt_to_throw(v, max_tilt_deg=max_tilt_deg),
                      dtype=float)


def _tilt_to_gravity(tilt_xy, correction) -> np.ndarray:
    """PLAN-frame ``(rx, ry)`` → the GRAVITY frame.  ``correction=None`` ⇒ a copy.

    The inverse half of :func:`_tilts_to_plan`; see it for why the pair exists
    and for what the ``rz`` projection costs.  This is the C-LEVEL-1.E direction
    (``R_gravityᵀ @ R_commanded``) used INTERNALLY rather than on the wire: the
    seed pose is a commanded, plan-frame quantity, and every other input to
    ``tilt_schedule`` — the ballistic throw aim, the observed catch arrival, the
    apparent-gravity field the banking objective tracks — is gravity-referenced.
    Handing the schedule a plan-frame pin beside gravity-frame pins is exactly
    the mixed-frame interpolation this row exists to close.
    """
    tilt = np.asarray(tilt_xy, dtype=float).reshape(2)
    if correction is None:
        return tilt.astype(float, copy=True)
    pose = np.zeros(6)
    pose[3:5] = tilt
    return np.asarray(levelling.uncorrect_pose(pose, correction),
                      dtype=float)[3:5]


def _tilts_to_plan(tilts, correction) -> np.ndarray:
    """GRAVITY-frame tilt series ``(n, 2)`` → the PLAN frame.  ``None`` ⇒ a copy.

    One :func:`levelling.correct_pose` per knot, on a position-free 6-vector —
    ``correct_pose`` rewrites the rotation only and copies position through
    untouched, so a zero position is not a claim about anything.

    **Why the whole SERIES and not just the pins.**  ``tilt_schedule``'s interior
    is the banking solution, ``tilt_to_receive(g − a_cup)`` — an *apparent
    gravity* direction, i.e. as gravity-referenced as the pins are.  Shifting
    only the endpoints would leave the interior a fraction of a degree off the
    field it is meant to track, and the schedule's whole job is that the seated
    ball feels no lateral specific force.

    **The ``rz`` projection, and why it is free here.**  ``R_gravity @ R_tilt``
    of two pure-tilt rotations carries a second-order ``rz`` term
    (``|offset × tilt| / 2``).  ``decompose`` pins ``rz`` to 0 by construction —
    the cup is a surface of revolution and the realisation has no yaw to give —
    so that term has nowhere to go, and this function drops it.

    MEASURED over the whole regime (``/tmp/probe_level.py``, 2026-09-06: the
    11.663 mrad session offset × every aim from 0 to the 12° ceiling, 17 azimuths
    each):

    * worst ``|rz|`` discarded — **1.221e-3 rad**;
    * worst change in the commanded cup AXIS from dropping it —
      **1.276e-4 rad** (0.0073°), i.e. **91× smaller** than the 11.663 mrad this
      row corrects, and 0.005 mm of landing at the 0.5 m apex;
    * worst departure of the ``(rx, ry)`` pair from the plain additive shift
      ``tilt − offset`` — **4.266e-5 rad**, confirming BCH's prediction that the
      whole first-order difference between the matrix composition and the
      additive one lives in ``rz`` (``a × b`` of two pure-tilt vectors is pure
      ``z``), so the projection costs third-order terms only.

    The sweep is pinned by
    ``tests/motion/test_unified_cycle.py::test_the_plan_frame_shift_drops_only_the_rz_second_order_term``
    — a numeric claim no test measures is a claim that rots.
    """
    arr = np.asarray(tilts, dtype=float)
    out = arr.astype(float, copy=True)
    if correction is None:
        return out
    pose = np.zeros(6)
    for k in range(out.shape[0]):
        pose[3:5] = out[k]
        out[k] = np.asarray(levelling.correct_pose(pose, correction),
                            dtype=float)[3:5]
    return out


def _start_tilt_for(state: CycleState) -> Optional[np.ndarray]:
    """The tilt knot 0 must open at — the ATTITUDE half of the seed state.

    A plan's knot 0 must equal the machine's commanded state in EVERY channel:
    position, velocity, hand AND tilt.  ``start_tilt`` is how the tilt channel
    says so, and there are two ways to know the answer:

    * **After a release** (the chained case) the previous window's terminal tilt
      is recovered from ``detach_axis``: ``tilt_to_throw`` is the exact inverse
      of ``cup_axis`` inside the 12° ceiling and every detach axis in this stack
      was produced by it, so the round trip is exact.  Needed because at a
      release the cup is in free fall, the apparent-gravity field ``g − a_cup``
      is exactly ZERO and the banking objective is degenerate there.
    * **Otherwise** the state's own pose carries it: ``decompose`` writes the
      schedule's ``(rx, ry)`` straight into ``pose[3:5]`` (nothing else touches
      those two channels — ``rz`` is pinned to 0), so ``pose[3:5]`` IS the tilt
      the machine is being commanded to hold, in the same frame the schedule
      produces.  A ``CycleState`` seeded from the live commanded state therefore
      answers the question directly.

    **The failure mode this second branch closes (MEASURED 2026-09-06).**  Until
    it existed a NEW window planned from a machine at rest got ``None``, so knot
    0 was left to the banking schedule.  Knot 0 is not an anchor, so the
    accel-bounded smoother blends it toward the terminal pin and it comes out
    tilted even though the raw banking value at ``a_cup = 0`` is exactly level.
    On the UH-3 carry (``KIND_SETTLE``, banking on, 60 mm lateral, 1.4 s, settle
    z ``SETTLE_CUP_Z_MM``, parked hand −0.038 rev, session limits
    250/3000/150000) that was **3.0727° on knot 0** against a LEVEL held pose —
    +3.5325 mm of centroid x through the 744.3 mm ``CUP_TILT_CENTER_Z_MM`` lever
    — and ``trajectory_node._install_continuity_ok`` refused the install
    ``STALE_STATE: leg position drift 0.1519 rev > 0.0600``.  The bench saw the
    same defect at a marginally different seed (2.53°, **0.1248 rev** against the
    same 0.06 bound); the two agree on the lever to four figures (0.0494 rev per
    degree of knot-0 tilt), which is what identifies them as one mechanism.
    Nothing downstream refuses the tilted plan on its own merits —
    ``validate_cycle`` passes both — so the guard is this pin or the install
    gate, and the install gate can only say no.

    With the pin, knot 0 is the seed's tilt EXACTLY (drift 0.0000 rev), whether
    that is level or the ~0.65° the levelling map leaves standing: the pin
    carries what the machine is at, it does not assume level.  A seed tilted past
    the 12° ceiling raises out of ``tilt_schedule`` as ``TILT_PIN`` rather than
    being silently clamped — the ``toss_release`` "gate the aim, don't rely on
    the clamp" precedent, and the honest answer for a pose the cup geometry
    cannot express.

    ⚠ **THE TWO BRANCHES ANSWER IN TWO DIFFERENT FRAMES** (named 2026-09-06,
    contract row E8; this function's behaviour is unchanged).  ``detach_axis`` is
    the direction a ball physically left along, so the post-release branch is
    **GRAVITY**-referenced.  ``state.pose`` is the machine's COMMANDED pose, i.e.
    the **PLAN** frame ``R_gravity @ R_request`` that ``trajectory_node``'s
    C-LEVEL-1 ingests produce.  That disagreement, carried into ``tilt_schedule``
    beside gravity-referenced release/catch pins, IS the levelling defect E8
    closes — and it is not fixed here, because a caller needs the raw seed pin
    (``trajectory_node._install_continuity_ok`` compares against the commanded
    pose, in the plan frame).  :func:`_realize` normalises the two into the
    gravity frame at the one place the frames meet; read its docstring before
    calling this from anywhere new.
    """
    if not state.post_release:
        return np.asarray(state.pose, dtype=float).reshape(6)[3:5].copy()
    if state.detach_axis is None:
        return None
    return np.asarray(tg.tilt_to_throw(state.detach_axis), dtype=float)


def _realize(kind, cup, goals, state, limits, geom, rcfg, *, t_wall,
             stages: Optional[Dict[str, float]] = None):
    """Stages 2-4 plus the gate: tilt schedule, decomposition, plan, validate.

    ``stages`` is the caller's per-stage wall-time accumulator (see
    :attr:`CycleMeta.stage_wall_s`); it is filled in place and left alone when
    ``None``.

    **THE LEVELLING FRAME LIVES HERE — contract row E8** (added 2026-09-06,
    ``ros_ws/docs/levelling_frame.md``).  This is the single point at which the
    gravity frame the cycle *aims* in and the plan frame the cycle *commands* in
    meet, and the whole tilt series crosses in one step:

    1. the seed pin is normalised into the gravity frame, so ``tilt_schedule``
       sees ONE frame — the seed, the receive pin, the throw pin and the banking
       objective all gravity-referenced.  Only the ``post_release=False`` branch
       of :func:`_start_tilt_for` needs it: that branch reads the COMMANDED
       ``state.pose`` (plan frame), while the post-release branch is derived from
       ``detach_axis``, a direction a ball physically left along, and is already
       gravity-referenced.  Which branch ran is exactly ``state.post_release``,
       so it is read here rather than pushed into that function — whose raw,
       plan-frame answer step 4 also needs;
    2. ``tilt_schedule`` runs unchanged and produces a gravity-referenced series;
    3. :func:`_tilts_to_plan` re-expresses the whole series into the plan frame;
    4. knot 0 is re-pinned to the seed's own float, exactly.

    **Why here and not later.**  Before ``decompose``, so the pose, its
    derivatives, the lever-arm shift and the slider are all computed from ONE
    tilt series — a correction applied to ``pose[3:5]`` afterwards would leave
    the declared velocities finite-differenced from a series the positions no
    longer describe, which is the "desyncs the wire's own derivatives"
    disqualifier C-LEVEL-2 already refuses for a per-knot lookup.  And before
    ``validate_cycle``, so the gate measures the object that ships:
    plan == emitted == gated.

    **What it was before.**  MEASURED 2026-09-06 (bag ``2026-09-06_19-*``, and
    ``/tmp/probe_level.py`` on the same offset): knot 0 was pinned in the PLAN
    frame off the seed while the release pin was GRAVITY-frame, so
    ``tilt_schedule`` interpolated between endpoints in two different frames.
    A LAUNCH from a levelled prepare pose commanded exactly mechanical zero at
    release — a platform physically **+11.663 mrad** off gravity, a **0.6682°**
    tilt step across the window, and the ball thrown in **−y**.  The measured
    bag agrees: +9.9…+10.3 mrad of lean at release, −9 mrad of launch error in
    −y on 7/7 throws, 9–38 mm of lateral drift, rim strikes.

    **The lever-arm residual this placement leaves — PRE-EXISTING and UNCHANGED,
    stated so it is not re-discovered as a regression.**  ``decompose`` reads the
    tilt it is handed for the cup lever arm (``shift = arm · cup_axis_xy``), so
    the compensation is computed at the COMMANDED tilt while the platform
    physically holds the gravity-frame one, and the cup opening lands
    ``|arm| · |correction|`` from the site.  MEASURED (``/tmp/probe_level.py``,
    2026-09-06, the 0.5 m-apex LAUNCH, ``arm`` = 115.7 mm at the 860 mm release):
    **1.349379 mm** in −y — and the number is **identical before and after this
    fix, to 1e-9 mm**, because before it the *centroid* was right and the
    *attitude* was 11.663 mrad wrong, which lands the physical cup in exactly the
    same place.  This fix removes the attitude error (23.3 mm of ballistic drift
    at that apex → 0.000) and neither adds to nor subtracts from the lever term.

    Closing the lever term as well means feeding ``decompose`` the GRAVITY tilt
    for the geometry while writing the PLAN tilt into ``pose[3:5]`` — a
    ``cup_realize`` change, and the reason it is not taken here is that one frame
    downstream of ``decompose`` is the more valuable invariant:
    ``cup_state_from_platform`` (the exact inverse of this position map),
    :func:`_seam_check` and ``trajectory_node._install_continuity_ok`` all read
    ``pose[3:5]`` beside ``pose[:2]`` and would then be reading two frames.  (The
    legacy toss path does not carry this residual — ``toss_release`` computes its
    shift from the gravity-referenced aim and ``trajectory_node`` corrects only
    the rotation afterwards — so a future reader comparing the two paths at
    ~1.35 mm is looking at this, and at a real difference.)
    """
    correction = state.levelling_correction
    recv = (np.asarray(tg.tilt_to_receive(
        np.asarray(goals.catch_vel_mm_s, dtype=float),
        max_tilt_deg=rcfg.max_tilt_deg), dtype=float)
            if goals.catch_vel_mm_s is not None else np.zeros(2))
    throw_tilt = _throw_tilt_for(cup, rcfg.max_tilt_deg)
    start_tilt = _start_tilt_for(state)
    if start_tilt is not None and not state.post_release:
        # Plan frame → gravity frame.  See step 1 above for why only this branch.
        start_tilt = _tilt_to_gravity(start_tilt, correction)
    rest_slew = False
    if goals.hold_tilt is not None:
        # The HELD receive attitude: one constant at every knot, which is what
        # the QP's held-axis rows assumed when they slaved the cup's lateral
        # channel to the stroke.  A schedule that slews inside the window would
        # walk the centroid through the 744.3 mm lever under a line the QP
        # believes is fixed, so there is nothing to schedule here — only to pin.
        hold = np.asarray(goals.hold_tilt, dtype=float).reshape(2)
        recv = hold
        throw_tilt = hold
        if start_tilt is None:
            raise CycleInfeasible(TILT_PIN, [
                "a held-attitude catch needs a seed that names its tilt, and "
                "this one does not (a post-release seed carries only its detach "
                "axis). The PRE-TILT REST is what puts the machine at the held "
                "attitude; plan the catch from its terminal state."])
        seam = float(np.hypot(*(start_tilt - hold)))
        if seam > HOLD_TILT_SEAM_TOL_RAD:
            raise CycleInfeasible(TILT_PIN, [bound_msg(
                'seed attitude off the held receive attitude (%.4f°, %.4f°) ='
                % (np.degrees(hold[0]), np.degrees(hold[1])),
                np.degrees(seam), '>', np.degrees(HOLD_TILT_SEAM_TOL_RAD),
                'deg', digits=4,
                tail="the platform must ALREADY be at the held attitude when "
                     "the dive starts — that is the PRE-TILT REST's job. "
                     "Closing the gap inside the window walks the centroid "
                     "%.2f mm through the %.1f mm lever under a line the QP "
                     "holds fixed"
                     % (float(tg.CUP_TILT_CENTER_Z_MM) * np.sin(seam),
                        float(tg.CUP_TILT_CENTER_Z_MM)))])
    elif goals.rest_tilt is not None:
        # The attitude-bearing REST: ``_throw_tilt_for`` answers LEVEL for a
        # zero take-off (the cup is held upright over a seated ball), which is
        # right for every REST but the two this field exists for — the pre-tilt
        # and the decay.  Overridden at that one point, and the slew between the
        # seed's attitude and this one is the smoothstep.
        throw_tilt = np.asarray(goals.rest_tilt, dtype=float).reshape(2)
        rest_slew = True
    t_stage = time.perf_counter()
    try:
        # Only `tilt_schedule` is guarded: TILT_PIN means "the aim is outside the
        # cup's usable cone", and laundering a frame-conversion bug into that code
        # would send an operator to move the target when the defect is here.  The
        # shift is inside the `finally` so its cost lands in the `tilt` stage,
        # which is where a reader looking for it will look.
        try:
            tilts = cr.tilt_schedule(cup, recv, throw_tilt, rcfg,
                                     start_tilt=start_tilt,
                                     rest_slew=rest_slew)
        except ValueError as exc:
            raise CycleInfeasible(TILT_PIN, [str(exc)])
        tilts = _tilts_to_plan(tilts, correction)
        if (start_tilt is not None and correction is not None
                and not state.post_release):
            # ``start_tilt is not None`` is load-bearing, not belt-and-braces:
            # writing the seed back onto a knot 0 that ``tilt_schedule`` was NOT
            # asked to pin leaves knot 0 on the seed while knot 1 follows the
            # smoother, i.e. a manufactured step at the very seam this whole
            # function exists to keep continuous — measured as LIMIT_JERK rather
            # than as the STALE_STATE the un-pinned path is supposed to produce.
            #
            # THE CONTINUITY PIN SURVIVES EXACTLY.  The transpose round trip is
            # exact as a ROTATION, but knot 0 is not a rotation claim — it is the
            # assertion "this plan opens at the float the emitter is already
            # streaming", which `trajectory_node._install_continuity_ok` measures
            # in leg revolutions at 0.06 rev.  So the seed's own value is written
            # back rather than recovered: 0.0000 rev of drift, by construction,
            # not by a tolerance.  (The post-release branch needs no write-back:
            # its pin comes from `detach_axis`, which never left the gravity
            # frame, so its plan-frame image is a single forward application and
            # both sides of the chained seam apply the SAME matrix to the SAME
            # float.)
            tilts[0] = np.asarray(state.pose, dtype=float).reshape(6)[3:5]
    finally:
        if stages is not None:
            stages['tilt'] = time.perf_counter() - t_stage
    t_stage = time.perf_counter()
    realized = cr.decompose(cup, tilts, rcfg)
    if stages is not None:
        stages['dec'] = time.perf_counter() - t_stage
    # C-CUP-2: the contact window is a fact the QP established about the ball;
    # ``decompose`` only reshapes the trajectory and does not carry it, so it is
    # passed across explicitly here.  Drop it and the gate goes vacuous.
    plan = CyclePlan.from_realized(
        realized, contact_knots=getattr(cup, 'contact_knots', None))
    meta = _meta_for(kind, plan, cup, goals, tilts, recv, throw_tilt,
                     limits, geom, t_wall=t_wall, stages=stages,
                     correction=correction)
    return plan, meta


def _meta_for(kind, plan, cup, goals, tilts, recv, throw_tilt, limits, geom, *,
              t_wall, stages: Optional[Dict[str, float]] = None,
              correction=None):
    t_stage = time.perf_counter()
    report = fz.validate_cycle(plan, limits, geom)
    if stages is not None:
        stages['val'] = time.perf_counter() - t_stage
    if not report.ok:
        raise CycleInfeasible(report.code, report.reasons, report=report)

    has_throw, _, post_release = _KIND_SHAPE[kind]
    releases: List[ReleaseMark] = []
    catches: List[CatchMark] = []
    if has_throw:
        t_rel = float(plan.total_duration)
        releases.append(ReleaseMark(
            t_s=t_rel,
            site_mm=_vec3(goals.throw_site_mm, 'throw_site_mm'),
            vel_mm_s=np.asarray(cup.takeoff_vel, dtype=float) * 1000.0,
            flight_s=float(goals.flight_s),
            target_mm=_vec3(goals.throw_target_mm, 'throw_target_mm'),
            tilt=np.asarray(throw_tilt, dtype=float),
            stroke_clear_s=plan_stroke_clear_s(plan, t_rel)))
    if int(cup.catch_k) >= 0:
        t_catch = goals.catch_time_s()
        catches.append(CatchMark(
            t_s=t_catch, knot=int(cup.catch_k),
            site_mm=_vec3(goals.catch_site_mm, 'catch_site_mm'),
            vel_mm_s=_vec3(goals.catch_vel_mm_s, 'catch_vel_mm_s'),
            arm_lead_s=plan_arm_lead_s(plan, t_catch),
            runway_margin_rev=_runway_margin_rev(plan, t_catch, limits)))

    plan_wall_s = time.perf_counter() - t_wall
    if stages is not None:
        # `cont` is the residual, so the five keys sum to plan_wall_s EXACTLY —
        # anything not attributed to a named stage (CyclePlan.from_realized, the
        # marks, the config builds, the argument checks) shows up here rather
        # than vanishing.  Clamped at 0 because perf_counter is monotonic but the
        # arithmetic is not exact.
        stages['cont'] = max(0.0, plan_wall_s - sum(
            float(stages.get(k, 0.0)) for k in STAGE_KEYS if k != 'cont'))

    return CycleMeta(
        kind=kind, n_knots=int(plan.n_knots), dt=float(plan.dt),
        duration_s=float(plan.total_duration),
        releases=tuple(releases), catches=tuple(catches),
        report=report, plan_wall_s=plan_wall_s,
        stage_wall_s=(dict(stages) if stages is not None else None),
        tilts=np.asarray(tilts, dtype=float),
        receive_tilt=np.asarray(recv, dtype=float),
        throw_tilt=np.asarray(throw_tilt, dtype=float),
        stroke_clear_s=(plan_stroke_clear_s(plan, 0.0) if post_release
                        else None),
        takeoff_vel_mps=np.asarray(cup.takeoff_vel, dtype=float),
        warm_start=getattr(cup, 'warm_start', None),
        cup_plan=cup, goals=goals, levelling_correction=correction)


def _runway_margin_rev(plan: CyclePlan, t_catch: float, limits) -> float:
    """Slider travel left below the catch, minus what stopping there needs.

    The same expression ``feasibility.validate_cycle``'s runway pass uses, read at
    the INTERPOLATED touch-down rather than at ``catch_k·dt``: the QP pins the cup
    by equality at the interpolated instant, and reading the knot instead measures
    the cup mid-approach (``sim/cycle_gate.py`` documents 33.7 mm of phantom error
    from exactly that substitution).  Reported, never gated — the gate already ran.
    """
    rev_c, vel_c = plan.hand_at(float(t_catch))
    available = float(rev_c) - fz.HAND_STROKE_MIN_REV
    needed = (float(vel_c) ** 2 / (2.0 * float(limits.hand_acc_limit_rps2))
              + fz.CATCH_RUNWAY_MARGIN_REV)
    return available - needed


def plan_launch(goals, state, limits, geom, **kw):
    """:data:`LAUNCH` — from rest to a release.  See :func:`plan_cycle`.

    Release-terminal (:func:`is_release_terminal` is True on the result).
    """
    return plan_cycle(LAUNCH, goals, state, limits, geom, **kw)


def plan_steady(goals, state, limits, geom, **kw):
    """:data:`STEADY` — release → catch → release.  See :func:`plan_cycle`.

    Release-terminal (:func:`is_release_terminal` is True on the result).
    """
    return plan_cycle(STEADY, goals, state, limits, geom, **kw)


def plan_landing(goals, state, limits, geom, **kw):
    """:data:`LANDING` — release → catch → rest.  See :func:`plan_cycle`."""
    return plan_cycle(LANDING, goals, state, limits, geom, **kw)


def plan_settle(goals, state, limits, geom, **kw):
    """:data:`SETTLE` — release → rest, no catch.  See :func:`plan_cycle`."""
    return plan_cycle(SETTLE, goals, state, limits, geom, **kw)


# ─────────────────────────────────────────────────────────────────────────────
# Splicing
# ─────────────────────────────────────────────────────────────────────────────

def _shift_contact(window, shift: int, n_knots: int):
    """C-CUP-2's window(s) moved by ``shift`` knots and clipped to ``[0, n-1]``.

    ``window`` is ``None``, a bare ``(k0, k1)`` pair, or a tuple of one or more
    inclusive ``(k0, k1)`` ranges. Both shapes reach this function: a
    :class:`CyclePlan`'s own ``contact_knots`` is already normalised to the
    tuple-of-ranges form, but a ``cup_cycle.CupCyclePlan`` (the re-solved TAIL
    a splice reads ``contact_knots`` from directly, before it is ever wrapped
    in a ``CyclePlan``) still carries the bare-pair shape — so a bare pair is
    normalised to a one-tuple here too, the same duck-typing
    ``cycle_plan._checked_contact_knots`` does. Every range is shifted and
    clipped independently and a range that vanishes under the clip is dropped;
    the surviving ranges stay sorted because the input was sorted and the
    shift/clip is monotone. Returns ``None`` when nothing of it survives.

    Every path that re-indexes a :class:`CyclePlan` (extend, head, tail,
    splice) goes through here or through :func:`_join_contact`, because a
    dropped or mis-shifted window silently disarms the gate — the failure
    C-CUP-2 exists to prevent.
    """
    if not window:
        return None
    if not isinstance(window[0], (tuple, list)):
        window = (window,)
    out = []
    for k0, k1 in window:
        k0 = int(k0) + int(shift)
        k1 = int(k1) + int(shift)
        k0 = max(k0, 0)
        k1 = min(k1, int(n_knots) - 1)
        if k0 <= k1:
            out.append((k0, k1))
    return tuple(out) if out else None


def _join_contact(win_a, win_b):
    """The contact window(s) of two concatenated plans (either may be
    ``None``), already on the JOINT clock.

    CONCATENATES the head's ranges and the tail's ranges, merging only where
    two ranges TOUCH or OVERLAP — never drops, never hulls across a gap. A
    chained LAUNCH+STEADY has a window per event (launch: seed → release;
    steady: touch-down → next release) with the whole ballistic flight between
    them, and hulling across that gap gates the RELEASE DIVE that sits between
    them — measured −48 366 mm/s² on the chained fixture, 7× the floor — and
    refuses every chain, for a ball that is in the air. Keeping only the later
    window (the pre-tuple behaviour) instead left the EARLIER window's gate
    silently vacuous on exactly the plans this contract most needs to cover —
    the defect the tuple-of-ranges field closes. This is the same re-gate
    discipline as before (``extend`` re-gates only the new tail and the splice
    seam because **the head was already gated when it was built and is
    carried bit for bit**) — it now just carries every window forward instead
    of only the last one.
    """
    if not win_a:
        return win_b
    if not win_b:
        return win_a
    merged = sorted(list(win_a) + list(win_b))
    out = [merged[0]]
    for k0, k1 in merged[1:]:
        pk0, pk1 = out[-1]
        if k0 <= pk1 + 1:
            out[-1] = (pk0, max(pk1, k1))
        else:
            out.append((k0, k1))
    return tuple(out)


def _seam_velocity(cup_joined, tilts_joined, k_s: int, limits):
    """``(pose_vel, hand_vel_rps)`` at seam knot ``k_s`` of the JOINED series, or
    ``(None, None)`` when there is no joined cup track to derive them from.

    **Why the seam knot's velocity has to be re-derived** (MEASURED 2026-09-23,
    R4 U4; ``scratchpad/probe_r4_seam_vel_channels.py``).  ``_concat_plans``
    keeps the HEAD's velocity at the seam and drops the tail's (``plan_b``'s knot
    0), and ``decompose`` computes the FD-borne part of every channel —
    ``tilt_dot``, ``dz_dot`` and the ``arm·axis_dot`` term inside the xy and
    slider rates — with :func:`cup_realize._knot_derivative`, a CENTRAL
    difference inside a series.  On a ``splice_at`` the head's value at that knot
    was therefore differenced against a knot belonging to the tail the splice
    DISCARDS: on the R4 250 mm hop it came out at ``ry = −0.0100 rad/s`` while
    the joined tilt series rises at ``+0.0062`` (left) and ``+0.0008`` (right)
    rad/s — the wrong SIGN for the curve it is attached to — and the emitter's
    Hermite then jumps by **402.1 mm/s² in x and 2.916 rad/s² in tilt** at that
    one knot, 0.56× the whole tilt accel budget and the 5th largest jump of 142
    knots.  Re-derived here it is 98.7 mm/s² and 1.4 rad/s².

    This is the value the retired ``replan_tail`` (deleted at R4, 2026-09-24)
    always installed at ITS seam ("its position is pinned to the head's but its
    velocity legitimately moves, because its forward neighbour is new") — that
    path re-decomposed the joint cup+tilt series and took knot ``k_s`` from it.
    Same quantity here, over a 3-knot
    window: ``_knot_derivative`` is central at the middle knot of three, so the
    middle knot of ``[k_s−1, k_s, k_s+1]`` is bit-for-bit the full-series value,
    and the cost is one ``decompose`` over three knots instead of over the plan.

    **Only a TRUNCATED head is stale, and that is why this is not applied to a
    pure ``extend``** (measured 2026-09-23, both directions).  A ``splice_at``
    head is ``_head_view``'s truncation, so its last knot was differenced against
    knots the splice throws away — that knot's velocity describes a trajectory
    that no longer exists.  A pure ``extend`` discards nothing: its head's last
    knot carries ``_knot_derivative``'s second-order ONE-SIDED stencil over a
    series that genuinely ends there, which is a valid estimate, and at a
    rest-terminal or release-terminal seam it is the one that answers the
    boundary condition exactly (measured: EXACTLY zero velocity at a rest seam,
    where the joined central difference manufactures 0.041 mm/s across the
    stop-then-start kink, and 0.046 mm/s at a STEADY+STEADY release knot whose
    velocity IS the throw).  The splice seams' staleness is 1.565 mm/s and
    0.0136 rad/s — 34× larger, and the only case with a discarded neighbour.
    Re-deriving unconditionally also SILENTLY REPAIRS a seam velocity a caller
    injected, which is what ``test_the_bounded_extend_gate_still_refuses_a_defect_AT_the_seam``
    exists to catch; the gate must refuse such a seam, not smooth it.

    The config: ``decompose`` reads only ``active_z_mm``, ``cup_z_base_mm``,
    ``slider_rev_zero_mm``, ``slider_stroke_mm``, ``z_band_mm`` and
    ``z_float_enabled`` — none of which :func:`build_realize_config` derives from
    its ``banking`` argument, and no caller in this repo passes a custom
    ``realize_cfg`` to ``plan_cycle`` — so rebuilding it from ``limits`` here
    reproduces the one both halves were decomposed with exactly.

    ``cup_joined is None`` means a caller joined plans this module did not
    produce (``CycleMeta.cup_plan`` is ``Optional`` for exactly that case, and
    ``_concat_cup`` already answers ``None`` there).  There is then no joined
    series to difference and the seam keeps the head's velocity — the retired
    ``replan_tail`` used to refuse on this same absence, which is not a refusal
    here because a cup-less ``extend`` is a legitimate call.
    """
    if cup_joined is None:
        return None, None
    a, b = k_s - 1, k_s + 2
    win = cc.CupCyclePlan(
        pos=cup_joined.pos[a:b], vel=cup_joined.vel[a:b],
        acc=cup_joined.acc[a:b], jerk=cup_joined.jerk[a:b - 1],
        t=cup_joined.t[a:b] - cup_joined.t[a], dt=cup_joined.dt,
        catch_k=-1, takeoff_vel=cup_joined.takeoff_vel, warm_start=None)
    realized = cr.decompose(win, np.asarray(tilts_joined, dtype=float)[a:b],
                            build_realize_config(limits))
    return realized.pose_vel[1].copy(), float(realized.slider_vel_rev_s[1])


def _concat_plans(plan_a: CyclePlan, plan_b: CyclePlan, catch_k: int,
                  seam_vel=(None, None)) -> CyclePlan:
    """``plan_a`` then ``plan_b`` with ``plan_b``'s duplicate first knot dropped.

    ``seam_vel`` is ``(pose_vel, hand_vel_rps)`` for the seam knot — see
    :func:`_seam_velocity` for why the inherited pair is stale there.  Knots
    BELOW the seam are untouched, which is what keeps the head bit-identical.
    """
    n_a = int(plan_a.n_knots)
    n_joint = n_a + int(plan_b.n_knots) - 1
    pose_vel = np.vstack([plan_a.pose_vel, plan_b.pose_vel[1:]])
    hand_vel = np.concatenate([plan_a.hand_vel_rps, plan_b.hand_vel_rps[1:]])
    seam_pv, seam_hv = seam_vel
    if seam_pv is not None:
        pose_vel[n_a - 1] = seam_pv
        hand_vel[n_a - 1] = seam_hv
    return CyclePlan(
        pose=np.vstack([plan_a.pose, plan_b.pose[1:]]),
        pose_vel=pose_vel,
        hand_rev=np.concatenate([plan_a.hand_rev, plan_b.hand_rev[1:]]),
        hand_vel_rps=hand_vel,
        dt=plan_a.dt, catch_k=catch_k,
        # b's knot j sits at n_a - 1 + j on the joint clock (its knot 0 is a's
        # last knot, dropped as the duplicate).
        contact_knots=_join_contact(
            _shift_contact(plan_a.contact_knots, 0, n_joint),
            _shift_contact(plan_b.contact_knots, n_a - 1, n_joint)))


def _concat_cup(cup_a, cup_b, dt, catch_k):
    """The two cup tracks on the joint clock, or ``None`` if either is missing."""
    if cup_a is None or cup_b is None:
        return None
    n = cup_a.pos.shape[0] + cup_b.pos.shape[0] - 1
    return cc.CupCyclePlan(
        pos=np.vstack([cup_a.pos, cup_b.pos[1:]]),
        vel=np.vstack([cup_a.vel, cup_b.vel[1:]]),
        acc=np.vstack([cup_a.acc, cup_b.acc[1:]]),
        jerk=np.vstack([cup_a.jerk, cup_b.jerk]),
        t=np.arange(n, dtype=float) * dt, dt=dt,
        catch_k=catch_k, takeoff_vel=cup_b.takeoff_vel,
        warm_start=None)


def _seam_check(plan_a: CyclePlan, plan_b: CyclePlan) -> Tuple[float, float]:
    """Refuse a seam whose two sides do not describe the same machine state.

    Positions only — and that is the whole point.  The two halves are produced by
    the SAME pure ``decompose`` from the same cup state and the same pinned tilt,
    so a position disagreement above float noise means the chain was built wrong
    (a stale state, a missing ``start_tilt``, a different config), and splicing it
    anyway would emit a step on six legs inside one 25 ms knot.  Velocities are
    NOT gated here: both sides finite-difference the tilt series from different
    neighbourhoods, so they disagree by the finite difference's own truncation
    even when the chain is perfect.  ``validate_cycle`` on the spliced whole is
    the authority on whether that seam is executable, because it measures the
    Hermite the emitter will actually sample.  The measured disagreement is
    returned so it can be recorded.
    """
    d_pose = np.abs(plan_a.pose[-1] - plan_b.pose[0])
    d_hand = abs(float(plan_a.hand_rev[-1]) - float(plan_b.hand_rev[0]))
    worst_mm = float(np.max(d_pose[:3]))
    worst_rad = float(np.max(d_pose[3:]))
    if worst_mm > _SEAM_POS_TOL_MM or d_hand > _SEAM_POS_TOL_REV \
            or worst_rad > _SEAM_POS_TOL_MM:
        raise CycleInfeasible(CHAIN_DISCONTINUITY, [
            bound_msg('seam pose gap', max(worst_mm, worst_rad), '>',
                      _SEAM_POS_TOL_MM, unit='mm/rad', digits=9,
                      tail=('hand gap %.3e rev (bar %.1e); the two windows do '
                            'not share their boundary knot — chain the second '
                            'from release_state_from_meta of the first'
                            % (d_hand, _SEAM_POS_TOL_REV)))])
    d_pv = float(np.max(np.abs(plan_a.pose_vel[-1] - plan_b.pose_vel[0])))
    d_hv = abs(float(plan_a.hand_vel_rps[-1]) - float(plan_b.hand_vel_rps[0]))
    return d_pv, d_hv


def _joined_correction(meta_a: CycleMeta, meta_b: CycleMeta):
    """The levelling frame a splice inherits, or a refusal if the halves disagree.

    Row E8.  Two windows spliced across different levelling frames put the whole
    correction into one 25 ms knot — 11.7 mrad of commanded tilt, ~1.8 mm of cup
    lever through the 744.3 mm tilt-centre arm — which is precisely the step
    :func:`_seam_check` exists to refuse.  It WOULD refuse it (the two terminal
    poses differ by that much), but by then the operator is reading a millimetre
    gap and not the reason for it, so the frame is checked by identity first and
    the message names the cause.  ``plan_b`` is chained through
    :func:`release_state_from_meta`, which copies the frame off ``meta_a``, so
    the only way to reach the refusal is to have built the second window from
    somewhere else.
    """
    ca = meta_a.levelling_correction
    cb = meta_b.levelling_correction
    if ca is None and cb is None:
        return None
    same = (ca is not None and cb is not None
            and np.array_equal(np.asarray(ca, dtype=float),
                               np.asarray(cb, dtype=float)))
    if not same:
        raise CycleInfeasible(CHAIN_DISCONTINUITY, [
            'the two windows were planned in DIFFERENT levelling frames '
            '(%s vs %s) — chain the second from release_state_from_meta of the '
            'first, which carries the frame across; splicing across a frame '
            'change puts the whole correction on one knot'
            % ('none' if ca is None else 'a correction',
               'none' if cb is None else 'a correction')])
    return ca


def extend_gate_range(plan_a: CyclePlan, plan_b: CyclePlan) -> Tuple[int, int]:
    """``(k_from, n_gated)`` — the knot range :func:`extend`'s gate visits.

    Exposed so a test can assert the bound without timing anything: on a chain of
    equal windows this is CONSTANT per extend, which is what makes a constant
    beat's install cost constant too.
    """
    n_a, n_b = int(plan_a.n_knots), int(plan_b.n_knots)
    k_from = max(0, (n_a - 1) - _VALIDATE_STENCIL_KNOTS)
    return k_from, (n_a + n_b - 1) - k_from


def _gate_view(plan: CyclePlan, k_from: int) -> CyclePlan:
    """A :class:`CyclePlan` over ``plan``'s knots ``[k_from, end]``, no copy.

    numpy slices are VIEWS and ``CyclePlan.__init__`` keeps them, so this costs
    one finiteness scan over the retained range and nothing else.  The curve on
    the retained span is bit-identical — ``_locate``/``_hermite`` read only the
    enclosing knot pair, so every sample inside the view reads exactly the floats
    the full plan would have handed the same sample.
    """
    ck = int(plan.catch_k)
    return CyclePlan(pose=plan.pose[k_from:], pose_vel=plan.pose_vel[k_from:],
                     hand_rev=plan.hand_rev[k_from:],
                     hand_vel_rps=plan.hand_vel_rps[k_from:], dt=plan.dt,
                     catch_k=(ck - k_from if ck >= k_from else -1),
                     contact_knots=_shift_contact(
                         plan.contact_knots, -k_from,
                         int(plan.n_knots) - k_from))


def _merged_peaks(head: 'fz.FeasibilityReport', tail: 'fz.FeasibilityReport'):
    """``tail``'s verdict carrying the whole plan's peaks.

    Every ``peak_*`` field is a MAXIMUM over the sampled path, so the maximum of
    the two halves' maxima IS the whole plan's — exactly, not approximately.  The
    verdict (``ok``/``code``/``reasons``) comes from ``tail`` alone because the
    head's was OK by construction (a window that failed its own gate raised
    ``CycleInfeasible`` and never became a ``plan_a``), which :func:`extend`
    asserts rather than assumes.
    """
    return dataclasses.replace(
        tail,
        peak_leg_vel_mmps=max(head.peak_leg_vel_mmps, tail.peak_leg_vel_mmps),
        peak_leg_acc_mmps2=max(head.peak_leg_acc_mmps2,
                               tail.peak_leg_acc_mmps2),
        peak_leg_jerk_mmps3=max(head.peak_leg_jerk_mmps3,
                                tail.peak_leg_jerk_mmps3),
        peak_leg_ext_mm=max(head.peak_leg_ext_mm, tail.peak_leg_ext_mm),
        peak_step_rev=max(head.peak_step_rev, tail.peak_step_rev),
        peak_hand_rev=max(head.peak_hand_rev, tail.peak_hand_rev),
        peak_hand_vel_rps=max(head.peak_hand_vel_rps, tail.peak_hand_vel_rps),
        peak_hand_acc_rps2=max(head.peak_hand_acc_rps2,
                               tail.peak_hand_acc_rps2),
        peak_hand_step_rev=max(head.peak_hand_step_rev,
                               tail.peak_hand_step_rev))


def _needs_whole_plan_gate(plan: CyclePlan) -> bool:
    """True when a RANGE gate would ask a different question than a whole-plan one.

    Exactly one case, and it is about the hand-stroke FLOOR.
    ``feasibility._cycle_stroke_floor`` lowers that floor to the plan's own FIRST
    knot when the hand is parked below the homed zero — a statement about the
    whole plan — so a view starting mid-plan would silently re-derive a STRICTER
    floor from its own first knot and could refuse a plan the whole-plan gate
    accepts.  Tested by the condition that triggers it rather than by reaching
    into that private derivation.

    It does not hold on any shape the coordinator installs: the shipped settle
    site sits ~10 mm ABOVE the homed zero.  So this is a correctness backstop,
    not a live path.
    """
    return float(plan.hand_at(0.0)[0]) < fz.HAND_STROKE_MIN_REV


def _gate_from_knot(plan: CyclePlan, k_from: int, limits, geom):
    """``validate_cycle`` over ``[k_from, end]``, with the refusal's clock named."""
    if k_from <= 0:
        return fz.validate_cycle(plan, limits, geom)
    rep = fz.validate_cycle(_gate_view(plan, k_from), limits, geom)
    if rep.ok:
        return rep
    # The view has its OWN clock starting at knot `k_from`, so every `t=` in the
    # refusal is that much earlier than the plan's. Say so at the front rather
    # than let an operator hunt for a defect at 0.006 s that is really at
    # 0.581 s. The CODE is untouched — guards match on that.
    return dataclasses.replace(rep, reasons=(
        ["times below are on the re-gated range's own clock, which starts at "
         'knot %d = %.4f s of the plan (the head before it was gated when it was '
         'built and is carried bit for bit)'
         % (k_from, k_from * float(plan.dt))] + list(rep.reasons)))


def _gate_joined(joined: CyclePlan, plan_a: CyclePlan, plan_b: CyclePlan,
                 meta_a: CycleMeta, limits, geom):
    """Re-gate a splice over the SEAM AND THE NEW WINDOW, inheriting the head.

    **Why not the whole plan.**  ``validate_cycle`` is ~89 % of a solve and is
    linear in the knot count, so gating the whole joined plan makes every extend
    on a ring dearer than the last: 81 → 137 → 193 → … knots, one window at a
    time, forever.  A constant beat whose install cost grows every beat is not a
    constant beat, and the install lead is the number the firmware-offload work
    will be designed against, so it has to be FLAT.

    **Why it is sound.**  ``plan_a`` was gated as a whole when it was produced
    (``plan_cycle`` raises ``CycleInfeasible`` on a failing report, and a joined
    ``plan_a`` came through this same function), and ``plan_b`` was gated as a
    whole on its own clock by its own ``plan_cycle`` call — the argument this
    module already makes about the catch, applied to every pass.  What NEITHER
    gate has seen is the SEAM: the per-knot step from ``k_seam − 1`` into
    ``k_seam``, the hand span across it, and the leg-jerk difference through it.
    So the range starts :data:`_VALIDATE_STENCIL_KNOTS` before the seam, which is
    derived from those stencils, and runs to the end.

    **What is NOT inherited.**  The peaks: they are maxima, so
    :func:`_merged_peaks` recombines them exactly, and the range they describe is
    the union (see below).

    **A TIGHTENED LIMIT does not retro-judge the head, and that is deliberate**
    (a consequence of inheriting the verdict, so it is stated rather than
    discovered).  If ``set_limits`` lowers a bound mid-session to something the
    already-streaming head violates, the whole-plan gate would have refused the
    next extend; this one accepts it, and judges only the NEW window against the
    new limits.  That is the right answer on this path: the head is ALREADY ON THE
    WIRE and cannot be un-emitted, and refusing the extend of a release-terminal
    plan does not make the machine safer — it re-opens the supersede cliff (see
    :func:`is_release_terminal`): a release-terminal plan streamed past its
    terminal knot ends in a throw commanded to a dead stop that no firmware
    guard sees.  A tightened limit therefore applies from
    the next window, which is the first window it can apply to.  Pinned by
    ``test_a_tightened_limit_applies_from_the_NEXT_window_not_retroactively``.

    **The one case that falls back to the whole plan** is a joined plan whose
    FIRST knot is parked below the homed zero.  ``_cycle_stroke_floor`` lowers the
    hand-stroke floor to that knot for the WHOLE plan, and a view starting at the
    seam would silently re-derive a STRICTER floor from its own first knot — a
    different question, and one that could refuse a ring the full gate accepts.
    Rather than reach into that private derivation, the condition that triggers it
    is tested directly and the whole plan is gated when it holds.  It does not
    hold on any shape the coordinator installs (the shipped settle sits ~10 mm
    ABOVE the homed zero), so this is a correctness backstop, not a live path.
    """
    if not (meta_a.report is not None and meta_a.report.ok) \
            or _needs_whole_plan_gate(joined):
        # Defensive: a head whose own verdict is missing or not OK has nothing to
        # inherit (unreachable from plan_cycle/extend), and the parked-floor case
        # asks a different question. Either way, gate the whole thing.
        return fz.validate_cycle(joined, limits, geom), None
    k_from, _n = extend_gate_range(plan_a, plan_b)
    tail = _gate_from_knot(joined, k_from, limits, geom)
    if not tail.ok:
        return tail, None
    # ── The merged range, and it is NOT always the whole plan ─────────────────
    # ``meta_a`` may itself carry a RANGE — a chain that was re-planned before it
    # was extended is the ordinary case on a ring, one re-plan and one extend per
    # beat.  Merging a range report's peaks and then labelling the result
    # "whole plan" under-reports: MEASURED (2026-09-07) on a six-window ring
    # re-planned at ``k_s = 320`` and then extended once, ``peak_leg_jerk_mmps3``
    # came out 75 356 against a whole-plan 95 101 mm/s³ — **20.8 % low, wearing a
    # whole-plan label**, which is a diagnostic that lies in the dangerous
    # direction.
    #
    # The union is contiguous by construction, so it is an interval and not a set:
    # a re-plan's range runs to ``plan_a``'s END, and this gate's range starts one
    # stencil BEFORE the seam, which is ``plan_a``'s last knot.  So the two overlap
    # and the union is ``[meta_a's start, n_joined)``.
    merged = _merged_peaks(meta_a.report, tail)
    prior = meta_a.report_range_knots
    rng = None if prior is None else (int(prior[0]), int(joined.n_knots))
    return merged, rng


def extend(plan_a: CyclePlan, meta_a: CycleMeta,
           plan_b: CyclePlan, meta_b: CycleMeta,
           limits, geom, *,
           head_truncated: bool = False) -> Tuple[CyclePlan, CycleMeta]:
    """Concatenate two windows at their shared release knot.  Re-gates the whole.

    ``plan_b`` must have been planned from ``release_state_from_meta(meta_a,
    plan_a)`` — that is what makes its knot 0 the SAME machine state as
    ``plan_a``'s terminal knot rather than merely a nearby one; :func:`_seam_check`
    refuses anything else.  The duplicate knot is dropped, ``plan_a`` survives
    **bit for bit below the seam** (a caller can therefore splice repeatedly
    without the head drifting), and the SEAM AND THE NEW WINDOW are re-validated.

    **The seam knot itself is bit-identical in POSITION and re-derived in
    VELOCITY** (2026-09-23, R4 U4).  ``_seam_velocity`` carries the measurement
    and the mechanism; this is why it is allowed.  A splice knot is
    ``executor.WIRE_READ_KNOTS`` knots ahead of the emitter BY CONSTRUCTION:
    ``install_segment`` re-reads the clock after the solve and refuses
    ``SPLICE_TOO_LATE`` unless ``k_s > k_wire``, where ``k_wire`` already carries
    that reserve, so knot ``k_s``'s velocity has NOT been handed to the wire and
    may still change.  Everything the wire has read — every knot below ``k_s`` —
    is untouched, and the seam's POSITION is pinned (``_seam_check``), so nothing
    the machine is already executing moves.  ``head_truncated`` is what asks for
    it, and ONLY ``splice_at`` with a discarded tail passes it: a pure ``extend``
    joins at a boundary knot whose one-sided velocity is valid — and at a rest or
    release terminal it is the boundary condition exactly — so that path is
    bit-identical, deliberately (see ``_seam_velocity`` for both measurements).

    **The gate range is bounded, and that is what keeps a ring's beat constant.**
    ``validate_cycle`` is ~89 % of a solve and linear in the knot count, so
    re-gating the whole joined plan made every extend on a chain dearer than the
    last.  Both halves were already gated whole by their own ``plan_cycle`` calls;
    what neither had seen is the seam.  So the range runs from
    :data:`_VALIDATE_STENCIL_KNOTS` knots before the seam to the end — a constant
    ``n_b + 1`` knots per extend, whatever the chain depth — and the head's
    verdict is inherited with its peaks merged back in, so ``meta.report`` still
    describes the whole plan exactly.  :func:`_gate_joined` carries the argument
    and the one fall-back case; :func:`extend_gate_range` exposes the range.

    The joined ``catch_k`` is the FIRST catch on the joint clock, because
    ``CyclePlan`` carries one and ``validate_cycle``'s runway pass reads that one.
    Every constituent window's own catch was already gated by its own
    ``plan_cycle`` call, so nothing goes unchecked; ``meta.catches`` carries them
    all, re-based onto the joint clock.

    The joined plan inherits ``plan_b``'s terminal shape, so
    :func:`is_release_terminal` of the joined meta is what matters: joining a
    LANDING onto a LAUNCH makes the joined plan rest-terminal (the pair ends at
    rest), while joining a STEADY keeps it release-terminal.
    """
    if abs(float(plan_a.dt) - float(plan_b.dt)) > 1e-12:
        raise ValueError("cannot splice plans on different knot grids (%r, %r)"
                         % (plan_a.dt, plan_b.dt))
    t_wall = time.perf_counter()
    correction = _joined_correction(meta_a, meta_b)
    seam = _seam_check(plan_a, plan_b)

    n_a = int(plan_a.n_knots)
    offset = float(plan_a.total_duration)
    catch_k = (int(plan_a.catch_k) if int(plan_a.catch_k) >= 0
               else (int(plan_b.catch_k) + n_a - 1 if int(plan_b.catch_k) >= 0
                     else -1))
    # The joined cup track and tilt series are built BEFORE the gate, because the
    # seam knot's velocity is re-derived from them (:func:`_seam_velocity`) and the
    # gate has to measure the Hermite the emitter will actually sample.  The meta
    # below reuses these two rather than rebuilding them.
    cup_joined = _concat_cup(meta_a.cup_plan, meta_b.cup_plan, float(plan_a.dt),
                             catch_k)
    tilts_joined = np.vstack([meta_a.tilts, meta_b.tilts[1:]])
    joined = _concat_plans(
        plan_a, plan_b, catch_k,
        seam_vel=(_seam_velocity(cup_joined, tilts_joined, n_a - 1, limits)
                  if head_truncated else (None, None)))
    # The seam and the new window, not the whole plan — see `_gate_joined` for
    # why that is both sound and necessary (a ring's install cost must be flat).
    report, report_range = _gate_joined(joined, plan_a, plan_b, meta_a, limits,
                                        geom)
    if not report.ok:
        raise CycleInfeasible(report.code, report.reasons, report=report)

    releases = tuple(list(meta_a.releases)
                     + [dataclasses.replace(m, t_s=m.t_s + offset)
                        for m in meta_b.releases])
    catches = tuple(list(meta_a.catches)
                    + [dataclasses.replace(m, t_s=m.t_s + offset,
                                           knot=m.knot + n_a - 1)
                       for m in meta_b.catches])
    # Re-measure the twins on the JOINT clock: a release that had no
    # deceleration inside its own window has one now.
    releases = tuple(dataclasses.replace(
        m, stroke_clear_s=plan_stroke_clear_s(joined, m.t_s)) for m in releases)
    catches = tuple(dataclasses.replace(
        m, arm_lead_s=plan_arm_lead_s(joined, m.t_s)) for m in catches)

    join_wall_s = time.perf_counter() - t_wall

    return joined, CycleMeta(
        kind=JOINED, n_knots=int(joined.n_knots), dt=float(joined.dt),
        duration_s=float(joined.total_duration),
        releases=releases, catches=catches, report=report,
        plan_wall_s=(meta_a.plan_wall_s + meta_b.plan_wall_s + join_wall_s),
        # The joined split is the two windows' splits plus the join's own cost,
        # so the coordinator's LAUNCH+LANDING log line attributes itself the same
        # way a single window's does.  `join_wall_s` is read ONCE and shared with
        # `plan_wall_s` above: two separate perf_counter reads put ~3 us between
        # them and broke the sum-to-plan_wall_s invariant the split's
        # trustworthiness rests on.
        stage_wall_s=merge_stage_wall(meta_a.stage_wall_s, meta_b.stage_wall_s,
                                      join_wall_s),
        tilts=tilts_joined,
        receive_tilt=(meta_a.receive_tilt if meta_a.catches
                      else meta_b.receive_tilt),
        throw_tilt=meta_b.throw_tilt,
        stroke_clear_s=meta_a.stroke_clear_s,
        takeoff_vel_mps=meta_b.takeoff_vel_mps,
        warm_start=meta_b.warm_start,
        cup_plan=cup_joined,
        goals=meta_b.goals, seam_vel_mismatch=seam,
        levelling_correction=correction,
        # `None` (whole plan) unless the head ALREADY carried a range — see
        # `_gate_joined`. A ring re-plans and extends once per beat, so a joined
        # meta inheriting a re-planned head is the ordinary case, not an edge one.
        report_range_knots=report_range)


def splice_knot(meta: CycleMeta, t_now_s: float, lead_s: float) -> int:
    """First knot at or after ``t_now_s + lead_s`` — the earliest safe splice.

    Exposed so a caller can decide whether a re-plan is worth attempting (and can
    bound how many it runs) WITHOUT this module holding any state: the replan
    policy — "plan at commit, then at most N bounded catch-side re-plans" — is the
    coordinator's, and a counter living here would be a second, invisible copy of
    it.
    """
    return _knot_at_or_after(float(t_now_s) + float(lead_s), float(meta.dt),
                             int(meta.n_knots))


def state_at_knot(plan: CyclePlan, meta: CycleMeta, k: int) -> CycleState:
    """The boundary condition a NEW window is planned from at interior knot ``k``.

    This is the state the retired ``replan_tail`` always spliced its tail from,
    named so that :func:`splice_at` uses the SAME one rather than a second
    spelling of it (there is one splice seed in this module, and this is it).

    Every channel comes from the plan's or the cup track's OWN floats, never from
    a round trip through the platform: ``pose``/``hand`` off the knot,
    ``cup_pos_mm``/``cup_vel_mm_s``/``cup_accel_mm_s2`` off ``meta.cup_plan`` (the
    cup acceleration is not recoverable from a pose, and it is what the QP's
    start-of-window rows treat as exact — see :class:`CycleState`).

    ``post_release=False`` / ``detach_axis=None``: no ball leaves the cup mid-
    carry, so the window starting here carries no detach cone.  That is exactly
    why the caller must refuse a ``k`` inside an existing release's cone — the
    rows would be re-solved away under a ball already in the air.
    :func:`splice_at` does, via :func:`_refuse_splice_into_a_detach_cone` (the
    bound the retired ``replan_tail`` used to apply directly).

    ``levelling_correction`` is the frame the PLAN was built in (row E8's
    in-flight rule): a re-level between the install and this call must not
    re-frame a plan the emitter is already streaming.
    """
    cup0 = meta.cup_plan
    if cup0 is None:
        raise CycleInfeasible(REPLAN_WINDOW, [
            "no source cup track on the meta — only a plan produced by "
            "plan_cycle carries one, and a splice is solved from the cup "
            "state it holds"])
    k = int(k)
    return CycleState(
        pose=plan.pose[k].copy(), pose_vel=plan.pose_vel[k].copy(),
        pose_accel=np.zeros(6), hand_rev=float(plan.hand_rev[k]),
        hand_vel_rps=float(plan.hand_vel_rps[k]),
        detach_axis=None, post_release=False,
        cup_pos_mm=cup0.pos[k] * 1000.0,
        cup_vel_mm_s=cup0.vel[k] * 1000.0,
        cup_accel_mm_s2=cup0.acc[k] * 1000.0,
        levelling_correction=meta.levelling_correction)


def _refuse_splice_into_a_detach_cone(meta: CycleMeta, k_s: int, dt: float,
                                      n: int, cup_cfg=None) -> None:
    """Refuse ``k_s`` when it cuts into the equalities of a ball already in flight.

    The window a splice seed opens is solved with ``post_release=False`` (see
    :func:`state_at_knot`), so knots ``k_rel+1 .. k_rel+n_detach`` of any release
    the head already carries would be re-solved WITHOUT their detach-cone rows —
    a lateral shove delivered to a ball that has left the cup, MEASURED at
    1.126 m/s² of off-axis specific force against an original 4.4e-16
    (2026-09-04, ``/tmp/probe_f4b.py``; the retired ``replan_tail`` carried two
    copies of this bound, which this function now is, alone).
    ``validate_cycle`` cannot see it: what is left is a perfectly smooth track.

    Two bounds, both inherited from ``replan_tail``:

    * ``k_s <= n_detach`` — the cone of the release THIS plan itself follows,
      which sits at knots ``1..n_detach``.  Unconditional, as it has always been.
    * ``k_s <= k_rel + n_detach`` for a release the head carries at ``k_rel``.

    The second is taken over releases with ``k_rel < k_s`` only, and the
    difference from ``replan_tail``'s original version is load-bearing: there
    the tail RE-CREATES a terminal release, so the bound excludes the terminal
    knot; here the segment is a different skill and excludes nothing — except
    the ``k_rel == k_s`` case,
    which is :func:`extend`'s own chain (the segment was seeded by
    :func:`release_state_from_meta`, so it carries the cone itself and
    :func:`_seam_check` is the authority).  Refusing that case would refuse every
    ordinary LAUNCH+LANDING join, which ``splice_at`` at ``k_s = n-1`` is.
    """
    n_detach = detach_knots(cup_cfg)
    if k_s <= n_detach:
        raise CycleInfeasible(REPLAN_WINDOW, [
            bound_msg('splice knot', k_s, '<=', n_detach, knob='lead_s',
                      digits=0, limit_label='detach knots',
                      tail=('knots 1..%d carry the detach-cone equalities of the '
                            'release this window follows and the new window is '
                            'solved without them; raise lead_s so the splice '
                            'lands after them' % n_detach))])
    head_release_knots = [k for k in
                          (int(round(float(r.t_s) / dt)) for r in meta.releases)
                          if k < k_s]
    if head_release_knots:
        k_rel = max(head_release_knots)
        if k_s <= k_rel + n_detach:
            raise CycleInfeasible(REPLAN_WINDOW, [
                bound_msg('splice knot', k_s, '<=', k_rel + n_detach,
                          knob='lead_s', digits=0,
                          limit_label='release knot %d + detach knots' % k_rel,
                          tail=('the plan throws at knot %d and knots %d..%d '
                                'carry that ball\'s detach-cone equalities; the '
                                'new window is solved with neither, so the '
                                'splice has to land after them'
                                % (k_rel, k_rel + 1, k_rel + n_detach)))])


def _head_view(plan: CyclePlan, meta: CycleMeta,
               k_s: int) -> Tuple[CyclePlan, CycleMeta]:
    """``plan``/``meta`` truncated to knots ``[0, k_s]`` — the part a splice keeps.

    The arrays are numpy SLICES (views), so the head costs no copy and stays bit
    for bit what the emitter has already been handed.

    Two fields are deliberately NOT carried across, and both are about cost
    rather than trajectory:

    * ``plan_wall_s``/``stage_wall_s`` are zeroed.  :func:`extend` SUMS them, so
      inheriting the source plan's would make every splice report the whole
      chain's cumulative solve time as its own — and the number
      ``splice_at``'s callers use it for is the INSTALL deadline (how long this
      solve took, against the lead), where a cumulative figure is simply wrong.
    * ``cup_plan.warm_start`` is dropped, matching :func:`_concat_cup`.

    ``report``/``report_range_knots`` ARE carried: the head's verdict is the
    source plan's, which is what :func:`_gate_joined` inherits so that a splice
    re-gates only the seam and the new window.  Its ``peak_*`` fields describe
    the SOURCE plan — including the tail this splice discards — so a spliced
    meta's peaks can be ABOVE what the spliced plan actually reaches.  That is
    the conservative direction (never below), and buying the exact number back
    would cost a full re-gate of the head, which is the one thing the bounded
    gate range exists to avoid.
    """
    n_head = k_s + 1
    dt = float(plan.dt)
    ck = int(plan.catch_k)
    head = CyclePlan(pose=plan.pose[:n_head], pose_vel=plan.pose_vel[:n_head],
                     hand_rev=plan.hand_rev[:n_head],
                     hand_vel_rps=plan.hand_vel_rps[:n_head], dt=dt,
                     catch_k=(ck if 0 <= ck <= k_s else -1),
                     contact_knots=_shift_contact(plan.contact_knots, 0,
                                                  n_head))
    t_head = k_s * dt
    # Half a knot of tolerance on the release instant for the same float reason
    # the retired `replan_tail` documented at its nominated-catch bound: a
    # mark's `t_s` is re-based by float addition in `extend` and `k_s * dt` is a
    # different arithmetic, so an exact `<=` would drop a terminal release by
    # 1e-16 s.
    releases = tuple(m for m in meta.releases
                     if float(m.t_s) <= t_head + 0.5 * dt)
    catches = tuple(m for m in meta.catches if int(m.knot) <= k_s)
    cup = meta.cup_plan
    cup_head = None if cup is None else cc.CupCyclePlan(
        pos=cup.pos[:n_head], vel=cup.vel[:n_head], acc=cup.acc[:n_head],
        jerk=cup.jerk[:k_s], t=cup.t[:n_head], dt=cup.dt,
        catch_k=(int(cup.catch_k) if 0 <= int(cup.catch_k) <= k_s else -1),
        takeoff_vel=cup.takeoff_vel, warm_start=None)
    head_meta = CycleMeta(
        kind=meta.kind, n_knots=n_head, dt=dt, duration_s=t_head,
        releases=releases, catches=catches, report=meta.report,
        plan_wall_s=0.0, tilts=np.asarray(meta.tilts)[:n_head],
        receive_tilt=meta.receive_tilt, throw_tilt=meta.throw_tilt,
        stroke_clear_s=meta.stroke_clear_s,
        takeoff_vel_mps=meta.takeoff_vel_mps, warm_start=meta.warm_start,
        cup_plan=cup_head, goals=meta.goals,
        levelling_correction=meta.levelling_correction,
        seam_vel_mismatch=meta.seam_vel_mismatch,
        report_range_knots=meta.report_range_knots, stage_wall_s=None)
    return head, head_meta


def splice_at(plan: CyclePlan, meta: CycleMeta, k_s: int,
              seg_plan: CyclePlan, seg_meta: CycleMeta,
              limits, geom) -> Tuple[CyclePlan, CycleMeta]:
    """Join ``seg_plan`` onto ``plan`` at INTERIOR knot ``k_s``.

    This is :func:`extend` generalised from "the plan's last knot" to "any knot
    the wire has not read yet", and it is implemented AS ``extend``: the head is
    the source plan truncated to ``[0, k_s]`` (:func:`_head_view`), and the join,
    the duplicate-knot drop, the bounded re-gate and the mark re-basing are the
    existing ones.  There is deliberately no second concatenation and no second
    gate range in this module — ``k_s == n_knots - 1`` reduces to exactly today's
    ``extend``, bit for bit, which is what makes that claim testable.

    ``seg_plan``'s knot 0 must BE the source plan's knot ``k_s`` — plan it from
    ``state_at_knot(plan, meta, k_s)`` and it is; :func:`_seam_check` refuses
    anything else with :data:`CHAIN_DISCONTINUITY`, because a seam that is merely
    close is a position step on six legs inside one 25 ms knot.

    Refusals, all :data:`REPLAN_WINDOW` except the seam's:

    * ``k_s < 1`` — there would be no head to keep.
    * ``k_s > n - 1`` — that knot is not on this plan.
    * a ``k_s`` inside a detach cone (:func:`_refuse_splice_into_a_detach_cone`).

    The returned meta's kind is :data:`SPLICED`.  Everything else about it is
    ``extend``'s: the marks of the discarded tail are gone, ``seg_meta``'s are
    re-based by ``k_s·dt``, and ``levelling_correction`` is the head's (the two
    halves must agree — ``_joined_correction`` refuses otherwise, which is row
    E8's in-flight rule).
    """
    n = int(plan.n_knots)
    dt = float(plan.dt)
    k_s = int(k_s)
    if k_s < 1:
        raise CycleInfeasible(REPLAN_WINDOW, [
            bound_msg('splice knot', k_s, '<', 1, knob='lead_s',
                      digits=0, tail='there would be no head to keep')])
    if k_s > n - 1:
        raise CycleInfeasible(REPLAN_WINDOW, [
            bound_msg('splice knot', k_s, '>', n - 1, knob='lead_s', digits=0,
                      limit_label='last knot',
                      tail=('the plan runs [0, %d]; there is no knot %d to '
                            'splice onto' % (n - 1, k_s)))])
    _refuse_splice_into_a_detach_cone(meta, k_s, dt, n)
    head, head_meta = _head_view(plan, meta, k_s)
    # `k_s < n - 1` is exactly "this splice DISCARDS a tail", which is what makes
    # the head's last-knot velocity stale (:func:`_seam_velocity`).  At
    # `k_s == n - 1` nothing is discarded and this call stays today's `extend`,
    # bit for bit — the claim the paragraph above makes testable.
    joined, joined_meta = extend(head, head_meta, seg_plan, seg_meta,
                                 limits, geom,
                                 head_truncated=(k_s < n - 1))
    return joined, dataclasses.replace(joined_meta, kind=SPLICED)
