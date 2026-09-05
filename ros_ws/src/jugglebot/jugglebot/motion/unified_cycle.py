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

Plan: ``plans/active/unified-7dof-planner.md`` § 4 Phase 4.
"""

from __future__ import annotations

import dataclasses
import math
import time
from typing import List, Optional, Tuple

import numpy as np

import jugglebot.hardware_config as hw
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
#: ``SINGULAR``, ``SETTLE_SITE``, ``ACC_BOX``, ``INFEASIBLE``), a
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

    # ── constructors ──

    @classmethod
    def at_rest(cls, pose, hand_rev, cfg=None) -> 'CycleState':
        """A stationary platform + slider: the LAUNCH boundary condition."""
        pose = np.asarray(pose, dtype=float).reshape(6)
        return cls(pose=pose, pose_vel=np.zeros(6), pose_accel=np.zeros(6),
                   hand_rev=float(hand_rev), hand_vel_rps=0.0,
                   detach_axis=None, post_release=False,
                   cup_pos_mm=cup_state_from_platform(pose, hand_rev, cfg),
                   cup_vel_mm_s=np.zeros(3), cup_accel_mm_s2=np.zeros(3))

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
    #: Worst per-channel velocity disagreement at a splice seam (mm/s and rev/s).
    #: Zero for a single window.  Recorded rather than gated: ``validate_cycle``
    #: on the spliced whole is the authority on whether the seam is executable,
    #: and it measures the Hermite the emitter will actually sample.
    seam_vel_mismatch: Optional[Tuple[float, float]] = None

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

    try:
        cup = cc.plan_window(events, state0, cup_cfg,
                             period_s=float(goals.period_s),
                             settle_site=settle_m, warm_start=warm_start)
    except cc.CupCycleInfeasible as exc:
        raise CycleInfeasible(exc.reason, [str(exc)])

    plan, meta = _realize(kind, cup, goals, state, limits, geom, rcfg,
                          t_wall=t_wall)
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


def _start_tilt_for(state: CycleState) -> Optional[np.ndarray]:
    """The seam pin for a window that follows a release, else ``None``.

    ``tilt_to_throw`` is the exact inverse of ``cup_axis`` inside the 12° ceiling,
    and every detach axis in this stack was produced by it, so the round trip
    recovers the tilt the previous window ended at.
    """
    if not state.post_release or state.detach_axis is None:
        return None
    return np.asarray(tg.tilt_to_throw(state.detach_axis), dtype=float)


def _realize(kind, cup, goals, state, limits, geom, rcfg, *, t_wall):
    """Stages 2-4 plus the gate: tilt schedule, decomposition, plan, validate."""
    recv = (np.asarray(tg.tilt_to_receive(
        np.asarray(goals.catch_vel_mm_s, dtype=float),
        max_tilt_deg=rcfg.max_tilt_deg), dtype=float)
            if goals.catch_vel_mm_s is not None else np.zeros(2))
    throw_tilt = _throw_tilt_for(cup, rcfg.max_tilt_deg)
    start_tilt = _start_tilt_for(state)
    try:
        tilts = cr.tilt_schedule(cup, recv, throw_tilt, rcfg,
                                 start_tilt=start_tilt)
    except ValueError as exc:
        raise CycleInfeasible(TILT_PIN, [str(exc)])
    realized = cr.decompose(cup, tilts, rcfg)
    plan = CyclePlan.from_realized(realized)
    meta = _meta_for(kind, plan, cup, goals, tilts, recv, throw_tilt,
                     limits, geom, t_wall=t_wall)
    return plan, meta


def _meta_for(kind, plan, cup, goals, tilts, recv, throw_tilt, limits, geom, *,
              t_wall):
    report = fz.validate_cycle(plan, limits, geom)
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

    return CycleMeta(
        kind=kind, n_knots=int(plan.n_knots), dt=float(plan.dt),
        duration_s=float(plan.total_duration),
        releases=tuple(releases), catches=tuple(catches),
        report=report, plan_wall_s=time.perf_counter() - t_wall,
        tilts=np.asarray(tilts, dtype=float),
        receive_tilt=np.asarray(recv, dtype=float),
        throw_tilt=np.asarray(throw_tilt, dtype=float),
        stroke_clear_s=(plan_stroke_clear_s(plan, 0.0) if post_release
                        else None),
        takeoff_vel_mps=np.asarray(cup.takeoff_vel, dtype=float),
        warm_start=getattr(cup, 'warm_start', None),
        cup_plan=cup, goals=goals)


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


def extend(plan_a: CyclePlan, meta_a: CycleMeta,
           plan_b: CyclePlan, meta_b: CycleMeta,
           limits, geom) -> Tuple[CyclePlan, CycleMeta]:
    """Concatenate two windows at their shared release knot.  Re-gates the whole.

    ``plan_b`` must have been planned from ``release_state_from_meta(meta_a,
    plan_a)`` — that is what makes its knot 0 the SAME machine state as
    ``plan_a``'s terminal knot rather than merely a nearby one; :func:`_seam_check`
    refuses anything else.  The duplicate knot is dropped, ``plan_a`` survives
    **bit for bit** in the head (a caller can therefore splice repeatedly without
    the head drifting), and the joined plan is re-validated as one.

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
    seam = _seam_check(plan_a, plan_b)

    n_a = int(plan_a.n_knots)
    offset = float(plan_a.total_duration)
    catch_k = (int(plan_a.catch_k) if int(plan_a.catch_k) >= 0
               else (int(plan_b.catch_k) + n_a - 1 if int(plan_b.catch_k) >= 0
                     else -1))
    joined = _concat_plans(plan_a, plan_b, catch_k)
    report = fz.validate_cycle(joined, limits, geom)
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

    return joined, CycleMeta(
        kind=JOINED, n_knots=int(joined.n_knots), dt=float(joined.dt),
        duration_s=float(joined.total_duration),
        releases=releases, catches=catches, report=report,
        plan_wall_s=(meta_a.plan_wall_s + meta_b.plan_wall_s
                     + (time.perf_counter() - t_wall)),
        tilts=np.vstack([meta_a.tilts, meta_b.tilts[1:]]),
        receive_tilt=(meta_a.receive_tilt if meta_a.catches
                      else meta_b.receive_tilt),
        throw_tilt=meta_b.throw_tilt,
        stroke_clear_s=meta_a.stroke_clear_s,
        takeoff_vel_mps=meta_b.takeoff_vel_mps,
        warm_start=meta_b.warm_start,
        cup_plan=_concat_cup(meta_a.cup_plan, meta_b.cup_plan, joined.dt,
                             catch_k),
        goals=meta_b.goals, seam_vel_mismatch=seam)


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
    (see the comment at the splice); and the whole plan goes back through
    ``validate_cycle``.

    Every refusal is a :data:`REPLAN_WINDOW`: no source cup track (the plan was
    not produced by this module), no goals to inherit the banking and the ball id
    from, a splice knot with no usable tail — at or past the catch (there is
    nothing left to re-aim), before knot 1 (there would be no head), inside a
    detach cone (``k_s <= n_detach``, and on a rest-terminal plan also
    ``k_s <= k_release + n_detach`` for the release the plan carries in its
    middle — see the comments at those checks), or within three knots of the end
    (a window that short is not a trajectory) — or a ``new_catch_t_s`` outside
    the tail's own window.  There is deliberately no path out of here that is not
    a ``CycleInfeasible``: a bare ``ValueError`` would carry no subcode and escape
    every guard matching on this module's outcome.

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
    catch_k = int(meta.catch_k)
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
    # ── The SAME rule, applied to a release the plan carries in its MIDDLE ────
    # Only a rest-terminal plan can have one (on a release-terminal plan the last
    # release IS the terminal, and any earlier one is separated from the splice
    # by a whole catch).  The chained LAUNCH + LANDING the coordinator installs is
    # exactly that shape: its launch release sits at knot 24 of 81, and knots
    # 25..24+n_detach carry that ball's detach-cone equalities.  The tail is
    # solved with `post_release=False` and no throw event at all, so a splice
    # landing on or before those knots would (a) re-solve the cone rows away —
    # the same lateral-shove-to-a-ball-in-flight defect the check above measures —
    # and (b) ERASE the release itself, since a LANDING window has no throw to
    # put back.  One bound closes both: the splice must land strictly after the
    # last release AND after its cone.
    if not release_terminal and meta.releases:
        k_rel = max(int(round(float(r.t_s) / dt)) for r in meta.releases)
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
    if catch_k >= 0 and k_s >= catch_k:
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
    try:
        tail_tilts = cr.tilt_schedule(cup_tail, recv, throw_tilt, rcfg,
                                      start_tilt=meta.tilts[k_s])
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
    spliced = CyclePlan.from_realized(cr.decompose(cup_joined, tilts, rcfg))
    report = fz.validate_cycle(spliced, limits, geom)
    if not report.ok:
        raise CycleInfeasible(report.code, report.reasons, report=report)

    t_catch_new = k_s * dt + tail_goals.catch_time_s()
    if release_terminal:
        mark = meta.releases[-1]
        t_rel = float(spliced.total_duration)
        releases = (ReleaseMark(
            t_s=t_rel, site_mm=mark.site_mm, tilt=throw_tilt,
            vel_mm_s=np.asarray(cup_tail.takeoff_vel, dtype=float) * 1000.0,
            flight_s=mark.flight_s, target_mm=mark.target_mm,
            stroke_clear_s=plan_stroke_clear_s(spliced, t_rel)),)
    else:
        # Every release this plan carries sits in the BIT-IDENTICAL head (the
        # guard above refuses a splice that does not clear the last one and its
        # detach cone), so the marks describe the spliced plan unchanged — same
        # instants, same clock, same site.  Carrying them is what keeps
        # `announcement_fields` / `_accept_cycle` able to name the throw this
        # plan made; dropping them would report a cycle that never threw.
        releases = meta.releases
    catches = (CatchMark(
        t_s=t_catch_new, knot=k_s + int(cup_tail.catch_k),
        site_mm=tail_goals.catch_site_mm, vel_mm_s=tail_goals.catch_vel_mm_s,
        arm_lead_s=plan_arm_lead_s(spliced, t_catch_new),
        runway_margin_rev=_runway_margin_rev(spliced, t_catch_new, limits)),)

    return spliced, CycleMeta(
        kind=REPLANNED, n_knots=int(spliced.n_knots), dt=dt,
        duration_s=float(spliced.total_duration),
        releases=releases, catches=catches, report=report,
        plan_wall_s=time.perf_counter() - t_wall,
        tilts=tilts, receive_tilt=recv, throw_tilt=throw_tilt,
        stroke_clear_s=meta.stroke_clear_s,
        takeoff_vel_mps=np.asarray(cup_tail.takeoff_vel, dtype=float),
        warm_start=cup_tail.warm_start, cup_plan=cup_joined,
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
