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
from jugglebot.motion.trajectory import hand_stroke as hs
from jugglebot.motion.trajectory import tilt_geometry as tg
from jugglebot.motion.trajectory.cycle_plan import CyclePlan
from jugglebot.motion.trajectory.hand_stroke import LINEAR_GAIN_REV_PER_M
from jugglebot.outcome_detail import bound_msg

# ── Window kinds ─────────────────────────────────────────────────────────────
LAUNCH = 'launch'
STEADY = 'steady'
LANDING = 'landing'
SETTLE = 'settle'
JOINED = 'joined'        #: the output of :func:`extend`
REPLANNED = 'replanned'  #: the output of :func:`replan_tail`

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
                + float(hw.JB_OP_HAND_CATCH_PRIME_REV) / LINEAR_GAIN_REV_PER_M)

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
#: ``HAND_NOT_PARKED`` gate measures against (``|pos| <= HAND_PARK_BAND_REV``,
#: 0.5 rev) and what the firmware's ``hand_source`` settle band is centred on.
#: But it is 10 mm BELOW this module's own cup box — the box is inset by
#: :data:`_CUP_Z_INSET_M` at both ends — so a window asked to settle there is
#: refused ``SETTLE_SITE`` before it plans (MEASURED 2026-09-04, the shipped
#: chained LAUNCH+LANDING at session limits: *"settle site z 0.6796 m is outside
#: the cup box [0.6896, 0.9846] m"*).
#:
#: So the settle is the parked height CLAMPED UP into the box: 689.6 mm =
#: **0.3162 rev**, which is inside ``HAND_PARK_BAND_REV`` with 37 % of the band to
#: spare, and is a state the NEXT cycle's LAUNCH can also be planned FROM (probed
#: at 689.60 / 690.0 / 692.0 / 695.0 mm — all ACCEPT), so a session's cycle N+1
#: starts where cycle N stopped.  Written as ``max`` rather than as the box floor
#: so it collapses to the true park the moment the box reaches it.
#:
#: NOT inside the firmware's ±0.10 rev ``hand_source`` settle band (that would
#: need 682.8 mm, further out of the box still).  Nothing depends on it being:
#: the latch switch is refused while the setpoint output is armed regardless
#: (``hand_source.cpp:60``), so it is never attempted from inside a session.
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
#: (``LINEAR_GAIN_REV_PER_M``, 31.62 rev/m), because a cup-z disagreement is what
#: reaches the hand channel.  The ROTATION channels are compared against the mm
#: bar too: they come from the PINNED tilt series and not from the solve, so
#: their gap is exactly zero, and 1e-3 rad still catches the failure this check
#: exists for — a missing ``start_tilt`` leaves 1.4e-2 rad (0.80°, 1.586 mm of
#: centroid), which
#: ``tests/motion/test_unified_cycle.py::test_the_start_tilt_pin_is_what_closes_the_seam``
#: measures.
_SEAM_POS_TOL_MM = _SEAM_MARGIN * _SEAM_FEAS_TOL_M * 1000.0
_SEAM_POS_TOL_REV = _SEAM_MARGIN * _SEAM_FEAS_TOL_M * LINEAR_GAIN_REV_PER_M

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
    #: Cup site the window comes to rest at.  Defaults to the catch site for a
    #: LANDING (the cup stops where it caught, which is the seat the ball is
    #: already in); REQUIRED for a SETTLE, which has no catch to default from.
    settle_site_mm: Optional[np.ndarray] = None
    ball_id: int = 0

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
    slider_mm = (float(hand_rev) / LINEAR_GAIN_REV_PER_M * 1000.0
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
            * LINEAR_GAIN_REV_PER_M)


def cup_z_for_hand_rev(hand_rev: float, cfg=None) -> float:
    """Cup-opening world z (mm) a LEVEL cup sits at for slider ``hand_rev``.

    The exact inverse of :func:`hand_rev_for_cup_z`; the pair exists so a caller
    that has one of the two never has to restate the map to get the other.
    """
    cfg = cr.RealizeConfig() if cfg is None else cfg
    slider_mm = (float(hand_rev) / LINEAR_GAIN_REV_PER_M * 1000.0
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
    slider_dot = float(hand_vel_rps) / LINEAR_GAIN_REV_PER_M * 1000.0
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
                pv, float(self.hand_vel_rps) / LINEAR_GAIN_REV_PER_M * 1000.0,
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
    #: — the plan's own answer to ``hand_stroke.stroke_clear_time``.  ``None``
    #: when the plan carries no knots after the release (the deceleration is in
    #: the next window); see :func:`plan_stroke_clear_s`.
    stroke_clear_s: Optional[float] = None


@dataclasses.dataclass(frozen=True)
class CatchMark:
    """One touch-down on a plan's own clock."""

    t_s: float                        #: INTERPOLATED touch-down, not ``knot·dt``
    knot: int                         #: ``catch_k`` — the knot at/just before it
    site_mm: np.ndarray
    vel_mm_s: np.ndarray              #: the BALL's arrival velocity
    #: Lead before touch-down at which the plan's hand catch motion begins — the
    #: plan's own answer to ``hand_stroke.required_arm_lead_s``.  See
    #: :func:`plan_arm_lead_s`.
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
    #: per-plan one.  On a JOINED or REPLANNED meta it is ``meta_b``'s, so a
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
    #: :func:`replan_tail` reads the exact cup state at the splice knot from.
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
    #: plan, because :func:`release_state_from_meta` and :func:`replan_tail` both
    #: read the frame from HERE.  Re-reading the node's live correction instead
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
    #: A :func:`replan_tail` output carries a RANGE, and it must: a re-plan
    #: REPLACES the tail, so the source report's peaks describe trajectory that no
    #: longer exists.  Merging them would report a deleted tail's number as live —
    #: a peak above a limit sitting next to ``ok=True``, which is the worst kind of
    #: diagnostic.  So the re-gated range's peaks are reported alone and this field
    #: says so, and a consumer that needs a whole-plan peak must ask for one
    #: (``validate_cycle`` on the plan) rather than assume.
    report_range_knots: Optional[Tuple[int, int]] = None
    #: Per-stage wall time, seconds, keyed :data:`STAGE_KEYS`.  ``None`` when the
    #: meta did not come from :func:`plan_cycle` (a ``replan_tail``).
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
    return CycleState(
        pose=plan.pose[-1].copy(),
        pose_vel=plan.pose_vel[-1].copy(),
        pose_accel=np.zeros(6),
        hand_rev=float(plan.hand_rev[-1]),
        hand_vel_rps=float(plan.hand_vel_rps[-1]),
        detach_axis=tg.cup_axis(float(mark.tilt[0]), float(mark.tilt[1])),
        post_release=True,
        cup_pos_mm=np.asarray(mark.site_mm, dtype=float).copy(),
        cup_vel_mm_s=np.asarray(mark.vel_mm_s, dtype=float).copy(),
        cup_accel_mm_s2=_G_MM_S2.copy(),
        levelling_correction=meta.levelling_correction,
    )


def is_release_terminal(meta: CycleMeta) -> bool:
    """True when the plan's LAST knot is a release — i.e. it ends in MOTION.

    :data:`LAUNCH` and :data:`STEADY` are release-terminal by construction;
    :data:`LANDING` and :data:`SETTLE` end at rest, and a :data:`JOINED` /
    :data:`REPLANNED` plan inherits whichever its last window was.  The test is
    on the release INSTANT rather than on ``meta.kind`` so a spliced meta that
    carries a spent release in its middle is not mistaken for one.
    """
    return bool(meta.releases
                and abs(float(meta.releases[-1].t_s) - float(meta.duration_s))
                <= 1e-9)


def latest_supersede_time_s(meta: CycleMeta) -> float:
    """Last plan time ``τ`` at which a frame may still be emitted from THIS plan.

    ``math.inf`` for a rest-terminal plan (there is nothing to hand over to, so
    there is no deadline); ``duration_s − dt`` for a release-terminal one.  A
    caller streaming past this instant emits a frame that LIES about the next
    knot.

    **The mechanism.**  ``KnotEmitter.frame`` samples the plan at ``τ``, ``τ+dt``
    and ``τ+2·dt`` and puts the ``τ+dt`` sample on the wire as the u1 knot and
    its exact velocities (``hand_next_vel_rps``, ``vel_next_mm_s``) — which is
    what the firmware's Mode-1 Hermite uses as its segment ENDPOINT velocity
    under ``HAS_V1``.  ``CyclePlan.state_at`` / ``hand_at`` clamp at
    ``t >= total_duration`` to the terminal HOLD: final position, **zero** twist,
    **zero** hand rate.  For a rest-terminal plan that clamp is the truth.  For a
    release-terminal one it is not: the plan ends mid-throw at full speed, so at
    ``τ = duration − dt`` the ``τ+dt`` sample lands exactly on the clamp and the
    emitted endpoint velocity collapses from the release velocity to 0.
    Everything upstream still agrees with itself, so nothing rejects the frame.
    Nothing downstream can either: the u0/u1 POSITIONS are correct, only the
    endpoint velocity is wrong, and no wire gate, pump gate or firmware clamp
    looks at v1 at all.

    **MEASURED** (2026-09-04, the reference 0.6 s LAUNCH, ``/tmp/probe_f1_cliff.py``,
    run twice with identical output): terminal hand knot velocity **93.011 rev/s**;
    at ``τ = duration − dt = 0.575 s`` the emitted ``hand_next_vel_rps`` is
    **0.0**, one float ulp earlier it is 93.011.  Reconstructing that segment with
    ``v1 = 0`` moves the firmware's Hermite by up to ``|h11|·T·Δv`` =
    **0.3445 rev = 10.90 mm** of slider mid-segment, and steps the transmitted
    ``vel_ff`` by 93 rev/s — both inside every guard on the path
    (``MAX_LEAD_HAND_REV`` 2.0 rev, ``HAND_VELFF_LIMIT_RPS`` 300 rev/s).  The same
    clamp zeroes ``vel_next_mm_s`` for the legs; on these fixtures the platform is
    nearly still at the release so that half is small, but the mechanism is the
    same one and it scales with the terminal twist.

    **What a caller does with it.**  Install the NEXT plan (``extend``'s output,
    or the next window) at or before this instant.  A session that cannot is
    better off streaming a rest-terminal plan: the deadline exists because the
    plan's own terminal knot is a lie about a trajectory that continues, and the
    fix is to make it continue.
    """
    if not is_release_terminal(meta):
        return math.inf
    return float(meta.duration_s) - float(meta.dt)


# ─────────────────────────────────────────────────────────────────────────────
# Configuration builders
# ─────────────────────────────────────────────────────────────────────────────

def build_realize_config(limits, *, banking: bool = True,
                         z_float: Optional[bool] = None,
                         z_band_mm: Optional[float] = None) -> cr.RealizeConfig:
    """A :class:`cup_realize.RealizeConfig` whose tilt-accel cap follows ``limits``.

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
    """
    accel_cap = (cr.TILT_ACCEL_BUDGET_FRACTION * float(limits.leg_acc_mmps2)
                 / cr.TILT_ACCEL_LEVER_MM)
    kwargs = dict(banking_enabled=bool(banking),
                  tilt_accel_limit_rad_s2=accel_cap)
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


def _zero_crossing_s(plan: CyclePlan, k0: int, k1: int) -> float:
    """Time of the hand-velocity zero between knots ``k0`` and ``k1`` (linear)."""
    v0 = float(plan.hand_vel_rps[k0])
    v1 = float(plan.hand_vel_rps[k1])
    if v0 == v1:
        return float(plan.t[k1])
    frac = v0 / (v0 - v1)
    return float(plan.t[k0]) + max(0.0, min(1.0, frac)) * plan.dt


def plan_stroke_clear_s(plan: CyclePlan, t_release_s: float, *,
                        eps_rps: float = _HAND_REST_EPS_RPS,
                        margin_s: float = hs.ARM_SUPPRESS_MARGIN_S
                        ) -> Optional[float]:
    """The plan's own twin of ``hand_stroke.stroke_clear_time``.

    ``hand_stroke``'s version answers "when can a scheduled command no longer land
    inside a live throw stroke?" by MODELLING the legacy firmware stroke engine's
    deceleration from the announced release velocity.  Under unified mode there is
    no stroke engine: the hand's motion is in the plan, sampled from the same
    clock as the platform, so the answer is a fact about the plan rather than a
    model of a device.

    **Definition.** The first instant at or after ``t_release_s`` at which the
    planned hand velocity reaches zero — its first stationary point, found on the
    knot grid and linearly interpolated inside the span that brackets it — plus
    ``margin_s``.  That is the same margin ``hand_stroke`` applies and for the
    same reason (the announcement's measured earliness against the physical
    release), which is why it is imported rather than restated.

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
    """The plan's own twin of ``hand_stroke.required_arm_lead_s``.

    ``hand_stroke``'s version answers "how much lead does a REACTIVE catch arm
    need before its event, or the Teensy refuses the dispatch?".  Under unified
    mode nothing is armed — the catch stroke is already in the plan — so the
    question becomes the one a consumer actually still needs answered: **how long
    before touch-down does the plan's hand motion into the catch begin?**  That is
    the window during which the hand is committed, and it is what a suppression
    or possession consumer has to respect.

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
        events.append(cc.CatchEvent(
            ball_id=int(goals.ball_id), t_s=goals.catch_time_s(),
            site=_vec3(goals.catch_site_mm, 'catch_site_mm') / 1000.0,
            vel=_vec3(goals.catch_vel_mm_s, 'catch_vel_mm_s') / 1000.0))
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

    **A release-terminal plan carries a streaming deadline**:
    :func:`latest_supersede_time_s` is the last ``τ`` at which a frame off this
    plan still tells the truth about the next knot, because the emitter's u1
    sample runs one ``dt`` ahead of ``τ`` and the plan's terminal clamp reports a
    HOLD the throw is not doing.  Install the next window at or before it.
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

    cup_cfg = build_cup_config() if cup_cfg is None else cup_cfg
    rcfg = (build_realize_config(limits, banking=bool(goals.banking_enabled))
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
                             settle_site=settle_m, warm_start=warm_start)
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
    t_stage = time.perf_counter()
    try:
        # Only `tilt_schedule` is guarded: TILT_PIN means "the aim is outside the
        # cup's usable cone", and laundering a frame-conversion bug into that code
        # would send an operator to move the target when the defect is here.  The
        # shift is inside the `finally` so its cost lands in the `tilt` stage,
        # which is where a reader looking for it will look.
        try:
            tilts = cr.tilt_schedule(cup, recv, throw_tilt, rcfg,
                                     start_tilt=start_tilt)
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
    plan = CyclePlan.from_realized(realized)
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

    Release-terminal, so it carries the :func:`latest_supersede_time_s` deadline.
    """
    return plan_cycle(LAUNCH, goals, state, limits, geom, **kw)


def plan_steady(goals, state, limits, geom, **kw):
    """:data:`STEADY` — release → catch → release.  See :func:`plan_cycle`.

    Release-terminal, so it carries the :func:`latest_supersede_time_s` deadline.
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

def _concat_plans(plan_a: CyclePlan, plan_b: CyclePlan, catch_k: int) -> CyclePlan:
    """``plan_a`` then ``plan_b`` with ``plan_b``'s duplicate first knot dropped."""
    return CyclePlan(
        pose=np.vstack([plan_a.pose, plan_b.pose[1:]]),
        pose_vel=np.vstack([plan_a.pose_vel, plan_b.pose_vel[1:]]),
        hand_rev=np.concatenate([plan_a.hand_rev, plan_b.hand_rev[1:]]),
        hand_vel_rps=np.concatenate([plan_a.hand_vel_rps,
                                     plan_b.hand_vel_rps[1:]]),
        dt=plan_a.dt, catch_k=catch_k)


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
                     catch_k=(ck - k_from if ck >= k_from else -1))


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
    plan does not make the machine safer — it re-opens the supersede cliff
    (:func:`latest_supersede_time_s`), which ends in a throw commanded to a dead
    stop that no firmware guard sees.  A tightened limit therefore applies from
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
           limits, geom) -> Tuple[CyclePlan, CycleMeta]:
    """Concatenate two windows at their shared release knot.  Re-gates the whole.

    ``plan_b`` must have been planned from ``release_state_from_meta(meta_a,
    plan_a)`` — that is what makes its knot 0 the SAME machine state as
    ``plan_a``'s terminal knot rather than merely a nearby one; :func:`_seam_check`
    refuses anything else.  The duplicate knot is dropped, ``plan_a`` survives
    **bit for bit** in the head (a caller can therefore splice repeatedly without
    the head drifting), and the SEAM AND THE NEW WINDOW are re-validated.

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
    :func:`latest_supersede_time_s` of the joined meta is the streaming deadline
    that matters: joining a LANDING onto a LAUNCH retires the launch's deadline
    (the pair ends at rest and returns ``inf``), while joining a STEADY moves it
    forward by that window.
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
    joined = _concat_plans(plan_a, plan_b, catch_k)
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
        tilts=np.vstack([meta_a.tilts, meta_b.tilts[1:]]),
        receive_tilt=(meta_a.receive_tilt if meta_a.catches
                      else meta_b.receive_tilt),
        throw_tilt=meta_b.throw_tilt,
        stroke_clear_s=meta_a.stroke_clear_s,
        takeoff_vel_mps=meta_b.takeoff_vel_mps,
        warm_start=meta_b.warm_start,
        cup_plan=_concat_cup(meta_a.cup_plan, meta_b.cup_plan, joined.dt,
                             catch_k),
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


def replan_tail(plan: CyclePlan, meta: CycleMeta, t_now_s: float,
                new_catch_site_mm, new_catch_vel_mm_s, limits, geom, *,
                lead_s: float,
                new_catch_t_s: Optional[float] = None,
                cup_cfg: Optional[cc.CupCycleConfig] = None,
                realize_cfg: Optional[cr.RealizeConfig] = None,
                warm_start: Optional['cc.SolverState'] = None
                ) -> Tuple[CyclePlan, CycleMeta]:
    """Re-solve the CATCH-SIDE tail of a committed cycle against a new landing.

    The replan policy (owner, 2026-08-29) is *plan at commit + bounded catch-side
    re-plans*, not a receding horizon: when the tracker moves its landing estimate,
    only the tail between now and the catch is re-solved, and **whatever the plan
    ends on stays fixed** so the beat does not move.  That is what this function
    implements, and it is why the tail window is planned with the SAME terminal
    boundary condition as the plan it replaces.

    **There are exactly two such boundary conditions, and both are supported.**
    A release-terminal plan (:data:`LAUNCH`, :data:`STEADY`, or a splice of one)
    ends mid-throw, and its tail is re-solved as a :data:`STEADY` window holding
    the old release (site, target, flight time) fixed.  A REST-terminal plan
    (:data:`LANDING`, :data:`SETTLE`, and the chained ``LAUNCH + LANDING`` the
    coordinator actually installs — rest-terminal *by design*, because that is
    the fix for the release-terminal cliff :func:`latest_supersede_time_s`
    documents) ends stopped over a seated ball, and its tail is re-solved as a
    :data:`LANDING` window whose settle site is pinned to **the plan's own
    terminal rest**.  Refusing the second shape would have made the
    replan policy dead code on the only shape the machine flies: every tracker
    landing update on a shipped install would have been answered
    ``REPLAN_WINDOW``.

    Mechanically: the splice knot ``k_s`` is the first knot at or after
    ``t_now + lead_s``; the head ``[0, k_s)`` is carried across **bit for bit**
    on all four channels; the tail is a fresh window solved from the cup state at
    ``k_s`` (``post_release=False`` — no ball leaves the cup mid-carry, so there
    is no detach cone to honour there) with the new catch and the old terminal;
    the cup track and the tilt series are spliced and decomposed as ONE series
    (see the comment at the splice); and the NEW TAIL AND THE SPLICE SEAM go back
    through ``validate_cycle``.

    **The re-gate range is bounded, and on a ring it has to be.**  The gate is
    ~89 % of a solve and linear in the knot count, so a whole-plan re-gate made
    every catch re-plan dearer than the last: MEASURED 399 ms on a 137-knot chain,
    already past the 0.30 s commit lead, which means the install lands behind the
    emitter and the continuity guard answers ``STALE_STATE``.  The head is carried
    bit for bit and was gated when its plan was installed, so the range starts
    :data:`_VALIDATE_STENCIL_KNOTS` knots before the splice and runs to the end.
    Consequently ``meta.report``'s ``peak_*`` describe THAT RANGE and not the whole
    plan, and :attr:`CycleMeta.report_range_knots` says so — deliberately NOT
    merged with the source plan's peaks, because a re-plan replaces the tail and
    those peaks describe trajectory that no longer exists.

    **WHICH catch is re-aimed, on a chain.**  A joined plan carries one catch per
    window and only one of them is live, so the caller NAMES the instant it means
    through ``new_catch_t_s`` and every catch-side bound below is taken against
    THAT one.  Left unnamed, the target is ``meta.catches[0]`` — permanently the
    FIRST catch on a joined meta, which from the second chained window onward is
    a touch-down that happened a beat ago.

    Every refusal is a :data:`REPLAN_WINDOW`: no source cup track (the plan was
    not produced by this module), no goals to inherit the banking and the ball id
    from, a splice knot with no usable tail — at or past the NOMINATED catch
    (there is nothing left to re-aim), before knot 1 (there would be no head),
    inside a detach cone (``k_s <= n_detach``, and on EVERY shape also
    ``k_s <= k_release + n_detach`` for each release the plan carries before its
    terminal — see the comments at those checks), skipping over a nearer catch
    (the tail holds exactly one touch-down, so a nominated catch that is not the
    first one after the splice would erase the ones between), or within three
    knots of the end (a window that short is not a trajectory) — or a
    ``new_catch_t_s`` outside the tail's own window.  There is deliberately no
    path out of here that is not a ``CycleInfeasible``: a bare ``ValueError``
    would carry no subcode and escape every guard matching on this module's
    outcome.

    **The TERMINAL RELEASE is preserved as an EVENT, and the chain depends on
    it.**  On the release-terminal branch the tail is planned with the original
    mark's ``(site, target, flight)`` and a ``period_s`` that puts the throw back
    on the plan's own last knot, so the release INSTANT and the whole release
    event survive bit for bit: MEASURED (2026-09-07,
    ``/tmp/probe_uh7_planner.py``, run twice with identical output) on a joined
    LAUNCH(0.6) + STEADY(1.4) spliced at ``k_s = 28``, ``|Δ t_s| = 0.0``,
    ``|Δ site_mm| = 0.0``, ``|Δ vel_mm_s| = 0.0``, ``|Δ tilt| = 0.0`` and
    ``|Δ total_duration| = 0.0`` — all exact equalities, not approximations, so
    the BEAT a chain is built on cannot drift through a re-plan.  The REALISED
    terminal knot is re-solved and agrees to ``1.1e-13 mm`` of pose,
    ``5.9e-13 rev`` of slider and ``1.9e-11 mm`` of cup position; only
    ``pose_vel[-1]`` moves measurably (``0.032 mm/s`` translational,
    ``2.7e-4 rad/s`` rotational), and that is ``decompose``'s ONE-SIDED finite
    difference at the last knot reading a re-solved neighbour rather than a
    release that moved.  :func:`release_state_from_meta` reads the SPLICED plan's
    own terminal, so the next chained window's seam is exact either way.

    **The replan envelope is narrower than the splice rule, and the gate is what
    says so.**  Measured on the 1.4 s reference cycle (2026-09-04, banking on,
    session limits 250/3000/150000): a splice at knot 4 validates at 50 891 mm/s³
    of leg jerk, at knot 12 it refuses with ``LIMIT_JERK`` (186 215) and at knot
    20 with ``LIMIT_ACC`` (4233 mm/s²) — and it does so for a re-plan to the SAME
    catch site, so the cause is not the re-aim.  It is the tilt schedule: the seam
    pin plus the catch pin sit closer together in the shorter tail window than the
    accel-bounded smoother can join under its cap (5.615 rad/s² produced against a
    3.196 cap at knot 20 — the "when the pins are too close together the cap is
    what gives" case ``cup_realize._accel_bounded_schedule`` documents), and the
    tilt curvature arrives at the legs through the 744.3 mm lever.  Every one of
    those is a LOUD refusal from the canonical gate, so the failure mode is a lost
    re-plan and never a bad plan; widening the envelope is a tilt-smoother change
    and belongs to whoever owns that, not to a carve-out here.
    """
    t_wall = time.perf_counter()
    cup0 = meta.cup_plan
    if cup0 is None:
        raise CycleInfeasible(REPLAN_WINDOW, [
            "no source cup track on the meta — only a plan produced by "
            "plan_cycle/replan_tail carries one, and the tail is solved from "
            "the cup state it holds"])
    if meta.goals is None:
        raise CycleInfeasible(REPLAN_WINDOW, [
            "no goals on the meta — only a plan produced by "
            "plan_cycle/replan_tail carries them, and the tail inherits its "
            "banking decision and its ball id from them (kind=%r)" % meta.kind])
    # WHICH BOUNDARY CONDITION the tail has to end on — and "TERMINAL" is the
    # load-bearing word in both branches.  A JOINED plan can carry a release in
    # its MIDDLE (launch, then the landing that catches it); that release is
    # already spent, so holding it "fixed" would fix nothing.  The predicate is
    # on the release INSTANT for exactly that reason (see `is_release_terminal`),
    # and the plan below is rest-terminal on the shipped chained install.
    release_terminal = is_release_terminal(meta)

    n = int(plan.n_knots)
    dt = float(plan.dt)
    k_s = splice_knot(meta, t_now_s, lead_s)
    # ── WHICH catch this re-plan is aiming at, as a knot ─────────────────────
    # A JOINED plan carries one catch per window and only ONE of them is LIVE.
    # ``meta.catch_k`` is permanently the FIRST of them — ``extend`` pins it
    # there because ``CyclePlan`` carries a single ``catch_k`` and
    # ``validate_cycle``'s runway pass reads that one — so on a chain it names a
    # touch-down that happened a beat ago.  Bounding the splice against it
    # refused EVERY landing update from the second chained window onward with
    # "the catch is inside the committed head", while the catch the update was
    # actually talking about was still most of a second in the future.  So the
    # caller NAMES the catch it is re-aiming through ``new_catch_t_s`` and the
    # bound is against that one.
    #
    # The knot comes from the MARK the nomination names, not from
    # ``floor(t/dt)`` — and it has to, because the two disagree.  ``cup_cycle``
    # takes ``k_td = floor(catch_time_s / dt)`` on the WINDOW's own clock and
    # ``extend`` then re-bases the index by ``+ n_a - 1``; recomputing the floor
    # on the JOINED clock is a different arithmetic on different floats.
    # MEASURED (2026-09-07): the ring's second catch is at 2.6 s with
    # ``dt = 0.025``, and ``2.6/0.025`` is exactly ``104.0`` while the window's
    # own ``0.6/0.025`` is ``23.999999999999996`` — floor 23, re-based to knot
    # **103**.  One knot of disagreement, entirely float representation, and it
    # made this bound refuse the very re-plan it was written to allow.  A
    # nomination within half a knot of a mark IS that mark; only a free
    # nomination (no mark at that instant) falls back to the floor, and for that
    # case the nominated-catch window bound below already guarantees
    # ``knot > k_s``, which is all this is used for.
    if new_catch_t_s is None:
        catch_k = int(meta.catch_k)
    else:
        named = next((m for m in meta.catches
                      if abs(float(m.t_s) - float(new_catch_t_s)) <= 0.5 * dt),
                     None)
        catch_k = (int(named.knot) if named is not None
                   else int(math.floor(float(new_catch_t_s) / dt)))
    if k_s < 1:
        raise CycleInfeasible(REPLAN_WINDOW, [
            bound_msg('splice knot', k_s, '<', 1, knob='lead_s',
                      digits=0, tail='there would be no head to keep')])
    # ── The detach cone is not re-solvable, so a splice cannot cut into it ──
    # A window that FOLLOWS a release carries hard equalities at knots
    # 1..n_detach pinning the cup's acceleration DIRECTION to the axis the ball
    # left along, so the ball in flight gets no lateral shove off the cup lip.
    # The tail is solved with ``post_release=False`` (nothing leaves the cup
    # mid-carry), which is right for the tail and fatal for those knots: any of
    # them that lands INSIDE the re-solved tail is re-solved without its cone
    # row.  MEASURED (2026-09-04, /tmp/probe_f4b.py, run twice identically): a
    # splice at k_s = 1 on a 3.24°-aimed throw leaves 1.126 m/s² of off-axis
    # specific force at knot 2 (0.686 at 1.62°, 0.276 level) where the original
    # plan had 4.4e-16 — a lateral kick delivered to a ball already in the air,
    # invisible to ``validate_cycle`` because the cup track is perfectly smooth.
    # The bound is one knot tighter than the measurement strictly needs: at
    # k_s == n_detach the tail's own start-acceleration equality happens to pin
    # knot n_detach to the value it already had, so the residual survives (0.0
    # measured).  That is an ACCIDENT of the start pin — nothing states it, the
    # QP only holds it to ``feas_tol``, and one knot of replan envelope is not
    # worth resting the ball's flight path on it.
    n_detach = int(cc.CupCycleConfig.n_detach if cup_cfg is None
                   else getattr(cup_cfg, 'n_detach', cc.CupCycleConfig.n_detach))
    if k_s <= n_detach:
        raise CycleInfeasible(REPLAN_WINDOW, [
            bound_msg('splice knot', k_s, '<=', n_detach, knob='lead_s',
                      digits=0, limit_label='detach knots',
                      tail=('knots 1..%d carry the detach-cone equalities of the '
                            'release this window follows and the tail is solved '
                            'without them; raise lead_s so the splice lands '
                            'after them' % n_detach))])
    # ── The SAME rule, applied to every release the plan carries MID-PLAN ─────
    # "Mid-plan" means every release that is not the plan's TERMINAL one, and the
    # bound is unconditional — it used to be skipped whenever the plan ended at a
    # release (`if not release_terminal`), on the argument that such a plan's only
    # release IS its terminal.  That is true of a single window and FALSE of every
    # chain: a joined LAUNCH + STEADY ends at a release AND carries the launch's
    # spent release at knot 24 of 81, so the guard skipped exactly the shape UH-7
    # flies.  MEASURED (2026-09-07, /tmp/probe_uh7_planner.py, run twice with
    # identical output) on that join at session limits: a re-plan at
    # ``t_now = 0.00`` / ``lead = 0.30`` (k_s = 12) and at ``t_now = 0.30``
    # (k_s = 24) were both ACCEPTED and both returned ``releases = [2.0]`` — the
    # 0.6 s throw silently deleted from a plan the emitter is streaming.
    #
    # The tail is re-solved with `post_release=False` and with at most the
    # plan's TERMINAL throw event, so a splice landing on or before a mid-plan
    # release would (a) re-solve that ball's detach-cone rows away — the same
    # lateral-shove-to-a-ball-in-flight defect the check above measures at
    # 1.126 m/s² — and (b) ERASE the release itself, because the tail has no
    # second throw to put back.  One bound closes both on every shape: the splice
    # must land strictly after the LAST mid-plan release and after its cone.
    #
    # The terminal release is excluded because the tail RE-CREATES it (the
    # release-terminal branch below pins the same throw event at the same
    # terminal knot); including it would refuse every re-plan of every
    # release-terminal plan, since its knot is ``n - 1`` by definition.
    k_terminal = n - 1
    mid_release_knots = [k for k in
                         (int(round(float(r.t_s) / dt)) for r in meta.releases)
                         if k < k_terminal]
    if mid_release_knots:
        k_rel = max(mid_release_knots)
        if k_s <= k_rel + n_detach:
            raise CycleInfeasible(REPLAN_WINDOW, [
                bound_msg('splice knot', k_s, '<=', k_rel + n_detach,
                          knob='lead_s', digits=0,
                          limit_label='release knot %d + detach knots' % k_rel,
                          tail=('the plan throws at knot %d and knots %d..%d '
                                'carry that ball\'s detach-cone equalities; the '
                                'tail is re-solved with neither, so the splice '
                                'has to land after them'
                                % (k_rel, k_rel + 1, k_rel + n_detach)))])
    # Only when NOTHING was nominated: then the sole catch this function knows
    # about is ``meta.catches[0]``, and a splice past it means the caller is
    # simply too late — ``lead_s`` is the knob and "catch knot" is the message.
    # When a catch WAS nominated the same bound is enforced one check later and
    # more precisely, by the nominated-catch window (``t_catch_full`` must reach
    # ``(k_s + 1)·dt``, which is exactly ``knot > k_s``): applying THIS one to a
    # chain refused every landing update from the second chained window onward,
    # because ``meta.catch_k`` is pinned to the first — spent — catch forever.
    if new_catch_t_s is None and catch_k >= 0 and k_s >= catch_k:
        raise CycleInfeasible(REPLAN_WINDOW, [
            bound_msg('splice knot', k_s, '>=', catch_k, knob='lead_s',
                      digits=0, limit_label='catch knot',
                      tail=('the catch is inside the committed head at '
                            't_now=%.3f s + lead %.3f s; nothing is left to '
                            're-aim' % (float(t_now_s), float(lead_s))))])
    if n - k_s < 4:
        raise CycleInfeasible(REPLAN_WINDOW, [
            bound_msg('tail knots', n - k_s, '<', 4, knob='lead_s', digits=0,
                      tail='a tail that short is not a trajectory')])

    goals = meta.goals
    mark = meta.releases[-1] if release_terminal else None
    period_tail = (n - 1 - k_s) * dt
    # The catch time is read off the META's mark, not off ``goals.catch_time_s()``
    # — on a SPLICED plan (a joined pair, or an earlier re-plan) the mark has been
    # re-based onto the joint clock and the goal's own field has not, so the two
    # differ by a whole window and only one of them means anything here.
    if new_catch_t_s is not None:
        t_catch_full = float(new_catch_t_s)
    elif meta.catches:
        t_catch_full = float(meta.catches[0].t_s)
    else:
        raise CycleInfeasible(REPLAN_WINDOW, [
            "the plan has no catch to move and none was nominated — pass "
            "new_catch_t_s to place one"])
    # ── The nominated catch must land INSIDE the tail, and bounding it here is
    # what keeps the refusal a refusal.  Out of range, the time reaches
    # ``_events_for`` as a tail-clock catch that is negative or past the
    # terminal throw, and ``cup_cycle`` raises a bare ``ValueError`` — which is
    # not a ``CycleInfeasible``, carries no subcode, and so escapes every guard
    # matching on this module's outcome.  MEASURED (2026-09-04,
    # /tmp/probe_f3_f4_f11_f15.py): ``new_catch_t_s = 1.45`` on the 1.4 s
    # reference cycle at k_s = 4 leaks "events must be ordered by
    # non-decreasing t_s"; ``0.05`` at k_s = 8 leaks "catch at t=-0.1500 s is
    # outside the window [0, 1.2000)".  Both are true and both are the wrong
    # exception.
    #
    # The window is ``[(k_s + 1)·dt, (n − 1)·dt)``: the lower end because
    # ``cup_cycle`` refuses a catch on the tail's knot 0 (it has no jerk support
    # there), the upper because the plan's terminal — the throw, or the rest —
    # sits at ``(n − 1)·dt`` and a catch may not reach it.  The BOUND is the same
    # on both shapes; only the name in the message follows the branch.
    t_catch_lo = (k_s + 1) * dt
    t_catch_hi = (n - 1) * dt
    if not (t_catch_lo <= t_catch_full < t_catch_hi):
        raise CycleInfeasible(REPLAN_WINDOW, [
            bound_msg('nominated catch', t_catch_full,
                      '<' if t_catch_full < t_catch_lo else '>=',
                      t_catch_lo if t_catch_full < t_catch_lo else t_catch_hi,
                      unit='s', knob='new_catch_t_s', digits=4,
                      limit_label=('tail knot 1 at'
                                   if t_catch_full < t_catch_lo
                                   else ('the terminal throw at'
                                         if release_terminal
                                         else 'the terminal rest at')),
                      tail=('the tail runs [%.4f, %.4f) s on the plan clock '
                            '(splice knot %d of %d); a catch outside it is not '
                            'a catch this window can make'
                            % (t_catch_lo, t_catch_hi, k_s, n - 1)))])
    # ── One catch per tail, so a NEARER one may not be skipped over ───────────
    # The same class as the mid-plan-release bound above, one event over.  The
    # tail is a single STEADY (catch → throw) or LANDING (catch → rest) window,
    # so it holds back exactly ONE touch-down.  A chained plan carries one catch
    # per window; nominating a LATER one while an earlier one still sits at or
    # after the splice would drop that earlier catch out of a plan the emitter is
    # already streaming — the cup simply would not be there for a ball already in
    # the air — and ``validate_cycle`` cannot see it, because what is left is a
    # perfectly smooth track.  A caller that nominates the LIVE catch (the first
    # still ahead of the splice, which is what
    # ``trajectory_node._replan_cycle_from_target`` selects) satisfies this by
    # construction; this is the bound that says so out loud.  It runs LAST of the
    # catch-side bounds so that a nomination which is not even inside the tail
    # keeps the more specific `nominated catch` message and its knob.
    #
    # NOTHING this module can currently BUILD reaches it, and that is a property
    # worth writing down rather than rediscovering: every kind that carries a
    # catch is entered FROM a release, so on any chain the releases interleave
    # the catches and the mid-plan-release bound above already forces the splice
    # past the release preceding the second catch — which is past the first.
    # This is therefore a structural backstop for a future kind carrying two
    # catches, pinned by
    # `test_replan_tail_refuses_a_nomination_that_would_ERASE_a_nearer_catch`,
    # which reaches it on a hand-built meta and asserts the subsumption on a real
    # one.
    skipped = [m for m in meta.catches if k_s <= int(m.knot) < catch_k]
    if skipped:
        raise CycleInfeasible(REPLAN_WINDOW, [
            'the nominated catch at %.4f s (knot %d) is not the FIRST one after '
            'the splice: catch knot(s) %s still sit ahead of splice knot %d and '
            'the tail carries exactly one touch-down, so re-solving would erase '
            'them — re-aim the LIVE catch (the first still ahead of '
            't_now + lead_s) instead'
            % (float(t_catch_full), catch_k,
               ', '.join(str(int(m.knot)) for m in skipped), k_s)])
    # ── The terminal the tail has to end on, one branch each ─────────────────
    # Release-terminal: the throw goals come from the release MARK, for the same
    # reason the catch time did — on a spliced plan the mark is on the joint
    # clock and the goal's fields are not.
    #
    # Rest-terminal: the settle site is the one the plan being replaced was
    # PINNED to — ``goals.settle_site_mm`` — falling back to the source cup
    # track's last knot only when the goal carries none (a LANDING may leave it
    # implicit, where ``_events_for`` defaults it to the catch site).
    #
    # WHY THE PIN AND NOT THE REALISED KNOT, which is the more obvious reading of
    # "the plan's own terminal".  ``SETTLE_CUP_Z_MM`` **is** the cup box's floor:
    # it is ``max(parked height, box floor)`` and the parked height is 10 mm
    # BELOW the floor, so the shipped settle site sits exactly ON the boundary.
    # The QP holds its terminal position equality only to ``feas_tol``, so the
    # realised knot lands on whichever side of that boundary the solve happens to
    # finish — MEASURED on the shipped chained install (2026-09-05,
    # /tmp/probe_rest_replan.py, run twice identically): realised terminal
    # 689.59999999997694 mm against a 689.6 mm pin, i.e. **2.3e-11 mm BELOW** it,
    # and ``cup_cycle._gate_settle_site``'s inclusive ``z_min <= z <= z_max`` then
    # refuses ``SETTLE_SITE`` on a site the machine is physically already resting
    # at.  Every landing update of every shipped cycle would have been refused by
    # a rounding residual.  The PIN cannot do that: it is the number the previous
    # solve's own gate already accepted.
    if release_terminal:
        tail_goals = CycleGoals(
            period_s=period_tail,
            throw_site_mm=np.asarray(mark.site_mm, dtype=float),
            throw_target_mm=np.asarray(mark.target_mm, dtype=float),
            flight_s=float(mark.flight_s),
            catch_site_mm=_vec3(new_catch_site_mm, 'new_catch_site_mm'),
            catch_vel_mm_s=_vec3(new_catch_vel_mm_s, 'new_catch_vel_mm_s'),
            catch_frac=None, catch_t_s=t_catch_full - k_s * dt,
            banking_enabled=bool(goals.banking_enabled),
            ball_id=int(goals.ball_id))
    else:
        tail_goals = CycleGoals(
            period_s=period_tail,
            catch_site_mm=_vec3(new_catch_site_mm, 'new_catch_site_mm'),
            catch_vel_mm_s=_vec3(new_catch_vel_mm_s, 'new_catch_vel_mm_s'),
            catch_frac=None, catch_t_s=t_catch_full - k_s * dt,
            banking_enabled=bool(goals.banking_enabled),
            ball_id=int(goals.ball_id),
            settle_site_mm=(
                _vec3(goals.settle_site_mm, 'settle_site_mm')
                if goals.settle_site_mm is not None
                else np.asarray(cup0.pos[n - 1], dtype=float) * 1000.0))

    rcfg = (build_realize_config(limits, banking=bool(goals.banking_enabled))
            if realize_cfg is None else realize_cfg)
    cup_cfg = build_cup_config() if cup_cfg is None else cup_cfg

    # The cup state at the splice knot, exact — the plan's own numbers, not a
    # round trip through the platform (whose acceleration is not recoverable).
    tail_state = CycleState(
        pose=plan.pose[k_s].copy(), pose_vel=plan.pose_vel[k_s].copy(),
        pose_accel=np.zeros(6), hand_rev=float(plan.hand_rev[k_s]),
        hand_vel_rps=float(plan.hand_vel_rps[k_s]),
        detach_axis=None, post_release=False,
        cup_pos_mm=cup0.pos[k_s] * 1000.0,
        cup_vel_mm_s=cup0.vel[k_s] * 1000.0,
        cup_accel_mm_s2=cup0.acc[k_s] * 1000.0)

    # STEADY (catch → release) for a release-terminal plan, LANDING (catch →
    # rest) for a rest-terminal one — the two kinds whose shape is "a catch, then
    # the terminal this plan already has".  `settle_m` is None for STEADY, so the
    # `plan_window` call below is unchanged on that branch.
    events, settle_m = _events_for(STEADY if release_terminal else LANDING,
                                   tail_goals)
    try:
        cup_tail = cc.plan_window(events, tail_state.to_cup_state(rcfg), cup_cfg,
                                  period_s=period_tail, settle_site=settle_m,
                                  warm_start=warm_start)
    except cc.CupCycleInfeasible as exc:
        raise CycleInfeasible(exc.reason, [str(exc)])

    recv = np.asarray(tg.tilt_to_receive(tail_goals.catch_vel_mm_s,
                                         max_tilt_deg=rcfg.max_tilt_deg),
                      dtype=float)
    throw_tilt = _throw_tilt_for(cup_tail, rcfg.max_tilt_deg)
    # Row E8: the tail is scheduled in the frame the PLAN was built in, read off
    # its own meta rather than from the node's live correction — a re-level or a
    # `reload_tilt_map` between the install and this re-plan must not re-frame a
    # plan the emitter is already streaming (C-LEVEL-1's in-flight rule).  The
    # splice pin `meta.tilts[k_s]` is a plan-frame knot, so it is un-corrected on
    # the way in and the tail is re-corrected on the way out; knot `k_s` is then
    # written back exactly, which is what keeps the head bit-identical.
    correction = meta.levelling_correction
    try:
        tail_tilts = cr.tilt_schedule(
            cup_tail, recv, throw_tilt, rcfg,
            start_tilt=_tilt_to_gravity(meta.tilts[k_s], correction))
        tail_tilts = _tilts_to_plan(tail_tilts, correction)
        if correction is not None:
            tail_tilts[0] = np.asarray(meta.tilts[k_s], dtype=float)
    except ValueError as exc:
        raise CycleInfeasible(TILT_PIN, [str(exc)])

    # Splice the CUP track and the TILT series, then run ``decompose`` ONCE over
    # the joint pair — rather than decomposing the tail alone and stacking the
    # two pose arrays.  ``decompose``'s knot velocities are finite differences of
    # the series it is handed, so a tail decomposed alone carries a ONE-SIDED
    # difference at its first knot while the head carries a CENTRED one at the
    # knot before, and the emitter's Hermite over that span then reconstructs a
    # curve neither half asked for.  Measured on this exact splice (2026-09-04):
    # peak leg jerk 454 947 mm/s³ decomposing the tail alone versus 186 215 over
    # the joint series at the same splice knot — a factor of 2.4, for free.
    #
    # The head stays BIT-IDENTICAL through all four channels even so, and the
    # ``start_tilt`` pin above is what buys that: the joint series' knot ``k_s``
    # carries the same cup position and the same tilt the head already had, so
    # every finite-difference stencil that reaches from a head knot into it reads
    # exactly what it read before.  Without the pin, the head's LAST knot
    # velocity would move, i.e. a knot the emitter may already have sent.
    tilts = np.vstack([meta.tilts[:k_s], tail_tilts])
    cup_joined = cc.CupCyclePlan(
        pos=np.vstack([cup0.pos[:k_s], cup_tail.pos]),
        vel=np.vstack([cup0.vel[:k_s], cup_tail.vel]),
        acc=np.vstack([cup0.acc[:k_s], cup_tail.acc]),
        jerk=np.vstack([cup0.jerk[:k_s], cup_tail.jerk]),
        t=np.arange(n, dtype=float) * dt, dt=dt,
        catch_k=k_s + int(cup_tail.catch_k),
        takeoff_vel=cup_tail.takeoff_vel, warm_start=cup_tail.warm_start)
    # ── The head is carried VERBATIM, and that is a construction not a hope ────
    # Decomposing the JOINT series (above) is right for the seam, but it does NOT
    # leave the head bit-identical, and the difference is invisible until you look
    # for it.  ``cup_realize._knot_derivative`` is a CENTRAL difference inside a
    # series and a second-order ONE-SIDED difference at its ends, and it is applied
    # to the tilt-rate term.  A plan built by ``extend`` therefore carries, at every
    # PRIOR seam knot, a velocity that came from a one-sided difference (that knot
    # was the END of the half it was decomposed in).  Re-decomposing the whole
    # series makes those knots interior, so they pick up a CENTRAL difference and
    # move — MEASURED (2026-09-07) 4.41e-2 mm/s at knots 24, 80 and 136 of a
    # six-window ring, every one of them OUTSIDE the re-gated range.
    #
    # The gate impact is nil at that magnitude, but the INVARIANT the bounded gate
    # rests on — "every knot that can change is inside the range" — was false as
    # written, and an invariant that is only true by accident of magnitude is not
    # one.  So the head's four channels are taken from the LIVE PLAN, and the
    # re-decomposed series supplies knots ``k_s`` onward.  ``k_s`` itself is a tail
    # knot: its position is pinned to the head's (the ``start_tilt`` pin above) but
    # its velocity legitimately moves, because its forward neighbour is new — and
    # it sits inside the gate range, which is exactly where it belongs.
    realized = cr.decompose(cup_joined, tilts, rcfg)
    spliced = CyclePlan(
        pose=np.vstack([plan.pose[:k_s], realized.pose[k_s:]]),
        pose_vel=np.vstack([plan.pose_vel[:k_s], realized.pose_vel[k_s:]]),
        hand_rev=np.concatenate([plan.hand_rev[:k_s],
                                 realized.slider_rev[k_s:]]),
        hand_vel_rps=np.concatenate([plan.hand_vel_rps[:k_s],
                                     realized.slider_vel_rev_s[k_s:]]),
        dt=realized.dt, catch_k=int(getattr(realized, 'catch_k', -1)))
    # ── Re-gate the NEW TAIL and the splice seam, not the whole plan ──────────
    # ``validate_cycle`` is ~89 % of a solve and LINEAR in the knot count, and on
    # a ring the plan grows a window per beat — so a whole-plan re-gate made every
    # catch re-plan dearer than the last.  MEASURED (2026-09-07): 399 ms on a
    # 137-knot chain, already past the 0.30 s ``_CYCLE_REPLAN_LEAD_S``, so the
    # splice knot the tail was solved for is behind the emitter by the time the
    # install is attempted and the continuity guard answers ``STALE_STATE``.  From
    # the second chained window on, EVERY landing update would have died that way
    # — the replan policy dead on the ring, which is exactly the failure the
    # live-catch selection above exists to end.
    #
    # The head ``[0, k_s)`` is carried BIT FOR BIT and was gated when the plan it
    # came from was installed, so the range starts one stencil before the splice
    # (:data:`_VALIDATE_STENCIL_KNOTS`, derived from ``validate_cycle``'s own
    # differences) and runs to the end.  That covers everything the splice
    # actually changed: the new tail, the per-knot step and hand span across the
    # splice, and the leg-jerk difference through it — the acceleration AT knot
    # ``k_s`` moves when the splice happens, because ``_locate`` puts that sample
    # at ``s = 0`` of the NEW span.  The new catch's runway is inside the range
    # too (``spliced.catch_k >= k_s`` by construction).
    #
    # The induction holds over repeated re-plans: every knot of the head was gated
    # by SOME pass — a knot before an earlier splice by that plan's own gate, one
    # after it as part of that re-plan's tail range.
    # ── ONE KNOT WIDER THAN THE EXTEND PATH, and the asymmetry is real ────────
    # On an EXTEND the seam knot's position AND velocity are bit-identical (it is
    # ``plan_a``'s terminal knot, carried through ``_concat_plans`` untouched), so
    # a view from ``k_seam − 1`` recomputes every difference that can have moved.
    # On a SPLICE the knot at ``k_s`` legitimately moves: its position is pinned
    # but its VELOCITY is not, because its forward neighbour is new.  That changes
    # the Hermite acceleration at ``s = 0`` of the span ``[k_s − 1, k_s]``, and the
    # leg-jerk difference across the sub-sample boundary ``(k_s − 2 | k_s − 1)``
    # therefore reads one new value — a difference a view starting at ``k_s − 1``
    # never forms, because that sample is its first.  One more knot is the whole
    # cost, and it keeps the range's own claim ("every difference that can have
    # moved is recomputed") true on this path as well.
    k_gate = (0 if _needs_whole_plan_gate(spliced)
              else max(0, k_s - _VALIDATE_STENCIL_KNOTS - 1))
    report = _gate_from_knot(spliced, k_gate, limits, geom)
    if not report.ok:
        raise CycleInfeasible(report.code, report.reasons, report=report)

    t_catch_new = k_s * dt + tail_goals.catch_time_s()
    # ── The marks the BIT-IDENTICAL HEAD still owns are carried, on both shapes ─
    # Every release and catch at a knot before ``k_s`` describes trajectory this
    # splice did not touch, so it describes the spliced plan unchanged — same
    # instant, same clock, same site.  Carrying them is what keeps
    # `announcement_fields` / `_accept_cycle` able to name the throw this plan
    # made (dropping them would report a cycle that never threw), and on a CHAIN
    # it is also what keeps the two bounds above meaningful across REPEATED
    # re-plans: they are computed from ``meta.releases`` / ``meta.catches``, so a
    # meta that forgot the launch's release would let the NEXT re-plan splice
    # into that ball's detach cone.
    #
    # One honest caveat, unchanged from the rest-terminal branch this generalises:
    # a carried mark keeps the ``stroke_clear_s`` / ``arm_lead_s`` it was measured
    # with, and those twins can reach a few knots PAST ``k_s`` into re-solved
    # trajectory.  Nothing in production reads a carried mark's twin, on either
    # branch: ``_accept_cycle`` takes ``stroke_clear_s`` from ``releases[-1]`` and
    # the catch twin from the mark still ahead of now, which is the new one.
    # ``releases[-1]`` is freshly measured on the RELEASE-TERMINAL branch (the new
    # terminal mark below re-runs ``plan_stroke_clear_s`` on the spliced plan); on
    # the REST-terminal branch it is a carried mark like any other, and carries a
    # pre-splice value — which is correct there, because a rest-terminal plan's
    # last release and its whole deceleration sit inside the bit-identical head.
    # So re-measuring would be motion nobody asked for.  Say it here rather than
    # leave a reader to assume they are fresh on both branches.
    if release_terminal:
        mark = meta.releases[-1]
        t_rel = float(spliced.total_duration)
        releases = tuple(meta.releases[:-1]) + (ReleaseMark(
            t_s=t_rel, site_mm=mark.site_mm, tilt=throw_tilt,
            vel_mm_s=np.asarray(cup_tail.takeoff_vel, dtype=float) * 1000.0,
            flight_s=mark.flight_s, target_mm=mark.target_mm,
            stroke_clear_s=plan_stroke_clear_s(spliced, t_rel)),)
    else:
        # Every release a rest-terminal plan carries sits in the head by the same
        # bound, so the whole tuple survives.
        releases = meta.releases
    catches = tuple(m for m in meta.catches if int(m.knot) < k_s) + (CatchMark(
        t_s=t_catch_new, knot=k_s + int(cup_tail.catch_k),
        site_mm=tail_goals.catch_site_mm, vel_mm_s=tail_goals.catch_vel_mm_s,
        arm_lead_s=plan_arm_lead_s(spliced, t_catch_new),
        runway_margin_rev=_runway_margin_rev(spliced, t_catch_new, limits)),)

    return spliced, CycleMeta(
        kind=REPLANNED, n_knots=int(spliced.n_knots), dt=dt,
        duration_s=float(spliced.total_duration),
        releases=releases, catches=catches, report=report,
        # The peaks describe the RE-GATED RANGE, not the whole plan — see the
        # field. `None` when the parked-floor fallback gated the whole thing.
        report_range_knots=(None if k_gate <= 0 else (k_gate, int(spliced.n_knots))),
        plan_wall_s=time.perf_counter() - t_wall,
        tilts=tilts, receive_tilt=recv, throw_tilt=throw_tilt,
        stroke_clear_s=meta.stroke_clear_s,
        takeoff_vel_mps=np.asarray(cup_tail.takeoff_vel, dtype=float),
        warm_start=cup_tail.warm_start, cup_plan=cup_joined,
        levelling_correction=correction,
        # Goals for the WHOLE spliced plan, on the whole plan's clock — so a
        # second re-plan reads a period and a catch time that describe what it is
        # actually holding, not the tail's.
        goals=dataclasses.replace(
            tail_goals, period_s=float(spliced.total_duration),
            catch_frac=None, catch_t_s=t_catch_new,
            # Release-terminal: the settle site is inert (the window ends at a
            # throw) and the ORIGINAL is carried so nothing is lost.
            # Rest-terminal: it is the terminal this splice was actually pinned
            # to, and a second re-plan re-reads the realised track anyway.
            settle_site_mm=(goals.settle_site_mm if release_terminal
                            else tail_goals.settle_site_mm)))
