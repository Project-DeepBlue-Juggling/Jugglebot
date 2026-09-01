"""Whole-cycle 7-DoF gate — the Phase-1 sim acceptance for the unified planner.

Plan: ``plans/active/unified-7dof-planner.md`` § 4 Phase 1 ("Sim validation", the
phase gate) and § 1 owner resolution 1, which is what this file resolves: the
Rung-3 collision between "the slam is the seat's runway" and every cleanup of the
whole-cycle sim catch dropping MAKE to 0/12.

**What it drives.** The PRODUCTION chain, end to end, with nothing re-implemented::

    cup_cycle.plan_window   →  one cup trajectory for a whole cycle (QP)
    cup_realize.tilt_schedule  →  where the cup points, per knot
    cup_realize.decompose      →  6-DoF pose + slider, per knot
    cycle_plan.CyclePlan.from_realized
    feasibility.validate_cycle →  the 7-channel gate

Nothing in this file plans, realises or validates anything itself; it selects
cycles, runs them through those five calls, and scores the result.  That is the
point — a harness that re-derived any of them would be gating a different
machine than the one Phase 2 ships.

**The acceptance authority is the KINEMATIC-CAPTURE model** (owner, 2026-08-29).
MuJoCo contact runs ADVISORY-ONLY and is reported, never gated on.  Grounds, from
the plan: ``sim/toss_gate.py`` already deliberately avoids MuJoCo contact as the
low-fidelity element; hardware catches are already smooth (standing bench fact);
and the Rung-3 P2 failure was a fragile equilibrium of exactly that contact model.
The hardware ladder (Phase 5) is the seating authority.

**The bands** (``core_clean``, mirroring ``toss_gate``'s vocabulary):

* ``validate_ok`` — ``validate_cycle`` returns ``OK`` at the session limits.
* ``capture_ok`` — the cup opening arrives at the requested catch site within
  :data:`CAPTURE_TOL_MM` at the catch instant.  (``v_match`` is measured and
  reported but **deferred**, exactly as ``toss_gate`` defers it: the cup QP
  matches catch velocity SOFTLY at ``catch_slider_vel_ratio = 0.7``, so a 0.15
  band would fail every cycle for a designed reason, not a defect.)
* ``seat_ok`` — the carry criterion.  A seated ball feels the specific force
  ``g − a_cup``; it stays in the cup while that vector lies inside the cup's cone
  about the cup axis.  Measured from the catch knot to the release as the angle
  between the cup axis and apparent-up, over the knots where the field is strong
  enough to matter (:data:`SEAT_FORCE_FLOOR_MPS2` — in free fall the ball is
  weightless and its orientation relative to the cup is physically irrelevant,
  so including those knots would score noise).
* ``no_slam`` — **no ceiling-overshoot slam anywhere en route to the catch.**
  The slider never rises above its catch-prime ceiling
  (``JB_OP_HAND_CATCH_PRIME_REV``) at any knot before the catch.  This is the
  band that answers Rung-3 directly: the P2 sim catch obtained its deceleration
  room by overshooting the ceiling and slamming back down, and the runway
  constraint is supposed to have made that unnecessary *by construction*.
* ``runway_active`` — the runway requirement is read back from the planner
  (:func:`cup_cycle.catch_runway_requirement`) and the catch really does leave
  that much stroke under it, reported as ``runway_margin_m``.  A boolean echo of
  ``catch_runway_enabled`` would pass just as happily on a program whose row had
  stopped being assembled, which is the vacuity this band exists to prevent.

**Two required comparisons** (plan § 4 Phase 1: "reproduce its output bit-comparably
at zero banking, then improve on it with banking on"):

1. ``parity_max_abs`` — the zero-banking chain reproduces
   ``sim/juggle_tilt.py::realize_tilted`` knot for knot.  Asserted at 0.0.
2. ``lateral_impulse_banked`` vs ``lateral_impulse_level`` — with the SAME pins on
   both arms, banking must lower the lateral specific-force impulse
   ``∫|(g − a_cup) × cup_axis| dt`` the ball actually feels.  Matched pins are
   load-bearing: the receive tilt is a physics boundary condition, not a banking
   choice, so scoring a banked arm with a receive pin against a level arm without
   one measures the pin, not the banking.

**What is REPORTED and never gated:** the verdict at the SHIPPED session limits
(``TrajectoryLimits.from_config``, leg jerk 30000 mm/s³) alongside the gating
verdict at the catch-capable session limits this gate runs (leg jerk 150000,
``reload_gate``/``toss_gate`` parity).  Those two disagree, structurally, and the
disagreement is an owner decision rather than something for a harness to tune
past — see :data:`_SHIPPED_LIMIT_NOTE`.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
import sys
import time

import numpy as np

# Single path bootstrap (repo root, ros_ws pkg, config/generated);
# see sim/_paths.py.  Runnable entry scripts only — library modules under
# sim/ never touch sys.path.
_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
from sim._paths import bootstrap_paths  # noqa: E402
bootstrap_paths()

from jugglebot import hardware_config as hw                        # noqa: E402
from jugglebot.motion.geometry import StewartGeometry              # noqa: E402
from jugglebot.motion.trajectory import cup_cycle as cc            # noqa: E402
from jugglebot.motion.trajectory import cup_realize as cr          # noqa: E402
from jugglebot.motion.trajectory import feasibility as fz          # noqa: E402
from jugglebot.motion.trajectory import tilt_geometry as tg        # noqa: E402
from jugglebot.motion.trajectory.cycle_plan import CyclePlan       # noqa: E402
from jugglebot.motion.trajectory.limits import TrajectoryLimits    # noqa: E402
from sim.gate_common import (                                      # noqa: E402
    KNOT_DT_S, NEUTRAL_POSE, REACH_ENVELOPE_MM, ViewerClosed, attach_viewer,
)

# ---------------------------------------------------------------------------
# Constants and thresholds
# ---------------------------------------------------------------------------

#: The SLIDER-reachable cup-z window (m).  ``cup_cycle``'s own docstring requires
#: the caller to override ``z_min_m``/``z_max_m`` to this: with its 0.45/1.10
#: defaults the realisation saturates the stroke clamp at 31 of 41 knots and the
#: gate refuses (measured 2026-08-30, ``tests/motion/test_validate_cycle.py``).
#: The operating band [0, JB_OP_HAND_CATCH_PRIME_REV] maps to slider 20…335 mm,
#: i.e. cup z 679.6…994.6 mm at level; the inset covers the tilt drop (~6 mm).
CUP_Z_LO_M = 0.690
CUP_Z_HI_M = 0.985

#: Cup opening height (m) at the release.  NOT the middle of the band: the launch
#: is made by the slider alone under the z = 170 pin, so what sets the peak cup-z
#: acceleration is the stroke available BELOW the release point.  Measured sweep
#: (2026-09-01): at throw z 0.78 the runway is 90 mm and flight 0.7 s already
#: needs 4280 rev/s²; at 0.86 the runway is 170 mm and 0.8 s fits in 3256; above
#: 0.88 the program refuses outright because the pre-launch dip no longer fits
#: under the ceiling.  0.86 is the measured optimum, not a round number.
THROW_CUP_Z_M = 0.86

#: Cup opening height (m) at the catch.
CATCH_CUP_Z_M = 0.83

#: Cycle period (s) — the WP3 demo point.
PERIOD_S = 1.4
#: Catch instant as a fraction of the period.
CATCH_FRAC = 0.55

#: Arrival velocity (m/s) of the caught ball, world frame.
CATCH_VEL_MPS = (0.10, -0.05, -2.50)

#: Cup-opening position tolerance (mm) at the catch instant.  Matches
#: ``toss_gate``'s ``preposition_tol_mm``; the cup QP pins the catch position by
#: hard equality, so anything above numerical noise here is a real defect.
CAPTURE_TOL_MM = 2.0

#: Cup cone half-angle (deg) for the seating criterion.  ``MAX_TILT_DEG`` is the
#: usable ceiling from the bb Rung-0 tilt-tracking characterisation and is the
#: only cup-cone number this codebase has; reusing it keeps ONE source of truth
#: rather than inventing a second cup geometry here.
SEAT_CONE_DEG = float(tg.MAX_TILT_DEG)

#: Specific-force magnitude (m/s²) below which the seating angle is not scored.
#: In free fall ``g − a_cup`` is zero, the ball is weightless, and the angle
#: between the cup axis and a vanishing field is numerically arbitrary AND
#: physically irrelevant.  5 % of g.
SEAT_FORCE_FLOOR_MPS2 = 0.5

#: Lead time (s) before touch-down at which the ADVISORY column releases its ball.
#: Long enough that the spawn is comfortably above the floor plane (cup z 0.83 m
#: with a −2.5 m/s arrival back-propagates to 1.14 m at 0.30 s), short enough that
#: the ball spends most of the cycle where the cup is.
_BALL_SPAWN_LEAD_S = 0.30

#: Session leg limits this gate runs — ``reload_gate``/``toss_gate`` parity.
_SESSION_LEG_VEL_MMPS = 250.0
_SESSION_LEG_ACC_MMPS2 = 3000.0
_SESSION_LEG_JERK_MMPS3 = 150000.0

_SHIPPED_LIMIT_NOTE = (
    "REPORTED, NOT GATED. validate_shipped is the same plan judged at "
    "TrajectoryLimits.from_config(hw) (leg jerk 30000 mm/s^3). It disagrees with "
    "the gating verdict STRUCTURALLY, not marginally: under the z=170 pin the "
    "cup's z channel carries the whole launch at up to 1433 m/s^3 of jerk, and "
    "ANY platform tilt leaks sin(tilt) of that into the centroid xy through the "
    "744.3 mm rotation-centre lever arm. Measured 2026-09-01 on the demo cycle "
    "with a CONSTANT tilt and a motionless schedule: 0 deg -> 17005 mm/s^3, "
    "2 deg -> 16784, 3.5 deg -> 28970, 4 deg -> 33339 (over), 12 deg -> 107815. "
    "So the shipped 30000 mm/s^3 leg-jerk limit admits a banked cycle only below "
    "about 3.5 deg of tilt, and that is a session-limit decision for the owner, "
    "not something a harness may tune past. The FD jerk is mesh-CONVERGED "
    "(16967 at 2 samples/knot, 17035 at 32), so it is the trajectory's real jerk "
    "and not a reconstruction artifact.")

_DEFERRED_NOTE = (
    "v_match is measured and reported but NOT gated, mirroring toss_gate's own "
    "deferral of the same criterion. The cup QP matches catch velocity SOFTLY at "
    "catch_slider_vel_ratio = 0.7 by design, so the achieved match sits near 0.3 "
    "BY CONSTRUCTION; a 0.15 band would fail every cycle for a designed reason.")

_MUJOCO_NOTE = (
    "ADVISORY ONLY (plan owner resolution 1, 2026-08-29). Contact make/drop "
    "counts are reported and never feed 'passed'. The Rung-3 P2 failure was a "
    "fragile equilibrium of this contact model; the kinematic-capture bands are "
    "the sim authority and the Phase-5 hardware ladder is the seating authority.")


def _mean_finite(values) -> float:
    """Mean of the finite entries, NaN when there are none.

    ``np.nanmean`` warns on an all-NaN slice, which is the NORMAL case here (the
    advisory column off leaves every carry fraction NaN) — and a warning that
    fires on normal operation is a warning nobody reads.
    """
    finite = [float(v) for v in values if np.isfinite(v)]
    return float(np.mean(finite)) if finite else float('nan')


def _pass_threshold(n: int) -> int:
    """Exact-integer ``ceil(0.9 n)`` — ``toss_gate``'s threshold, unchanged."""
    return (9 * n + 9) // 10


# ---------------------------------------------------------------------------
# Configuration and per-cycle record
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class CycleGateConfig:
    """Knobs for :class:`CycleGate`."""

    #: Explicit point list; ``None`` uses :func:`default_grid`.
    points: list = None
    #: Trials per point.  The planning chain is fully DETERMINISTIC, so one trial
    #: says everything the planner can say; >1 only re-runs the MuJoCo column.
    trials_per_point: int = 1
    seed: int = 0
    #: Catch-capable session limits (reload-gate / toss-gate parity).
    leg_vel_mmps: float = _SESSION_LEG_VEL_MMPS
    leg_acc_mmps2: float = _SESSION_LEG_ACC_MMPS2
    leg_jerk_mmps3: float = _SESSION_LEG_JERK_MMPS3
    #: Run the MuJoCo advisory column.  Never affects ``passed``.
    contact_diag: bool = True
    #: Run the headline flight-time sweep.
    flight_sweep: bool = True
    report_path: str = None


@dataclasses.dataclass
class CycleTrialResult:
    """One cycle, scored.  Frozen shape; NaN for unavailable, never ``None``."""

    idx: int
    point_id: str
    flight_s: float
    throw_xy_mm: tuple
    catch_xy_mm: tuple
    displacement_mm: float
    banking: bool

    accepted: bool
    reject_code: str = None

    # ── gating bands ──
    validate_ok: bool = False
    capture_ok: bool = False
    seat_ok: bool = False
    no_slam: bool = False
    runway_active: bool = False
    core_clean: bool = False

    # ── measured, reported ──
    validate_code: str = ''
    validate_shipped_code: str = ''
    peak_leg_vel_mmps: float = float('nan')
    peak_leg_acc_mmps2: float = float('nan')
    peak_leg_jerk_mmps3: float = float('nan')
    peak_leg_ext_mm: float = float('nan')
    peak_hand_rev: float = float('nan')
    peak_hand_vel_rps: float = float('nan')
    peak_hand_acc_rps2: float = float('nan')
    peak_hand_step_rev: float = float('nan')
    capture_dist_mm: float = float('nan')
    v_match: float = float('nan')            # DEFERRED, see _DEFERRED_NOTE
    max_seat_angle_deg: float = float('nan')
    seat_scored_knots: int = 0
    pre_catch_hand_max_rev: float = float('nan')
    runway_margin_m: float = float('nan')
    max_tilt_deg: float = float('nan')
    tilt_rate_max_rad_s: float = float('nan')
    tilt_accel_max_rad_s2: float = float('nan')
    lateral_impulse_banked: float = float('nan')
    lateral_impulse_level: float = float('nan')
    banking_gain_frac: float = float('nan')
    parity_max_abs: float = float('nan')
    solver_iters: int = -1

    # ── MuJoCo advisory ──
    mj_makes: int = -1
    mj_drops: int = -1
    mj_held_at_end: bool = False
    mj_carry_held_frac: float = float('nan')

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        d['throw_xy_mm'] = list(self.throw_xy_mm)
        d['catch_xy_mm'] = list(self.catch_xy_mm)
        return d


# ---------------------------------------------------------------------------
# The grid
# ---------------------------------------------------------------------------

#: Flight times (s) in the BINDING band — every one of these must come out
#: ``core_clean`` for the gate to pass.  Capped at 0.8 s because that is the
#: measured ceiling under the 3500 rev/s² hand-acceleration limit with z pinned
#: (see :func:`max_plannable_flight_s`), not because 0.9 is uninteresting.
_BINDING_FLIGHTS_S = (0.50, 0.60, 0.70, 0.80)

#: Flight times run and reported but NOT gated — they are over the hand cap by
#: physics, and the gate says so rather than pretending otherwise.  Same shape as
#: ``toss_gate``'s ``hardware_marginal_flight`` advisory band.
_ADVISORY_FLIGHTS_S = (0.90,)

#: Catch-site displacements (mm) from the throw site, inside the sim-established
#: reliable reach envelope (``gate_common.REACH_ENVELOPE_MM`` = 80).  0 is the
#: same-site cycle; 40 and 80 are the two-pose re-pose cases — the carry that the
#: split architecture plans as nothing (plan § 1, consequence 2).
_DISPLACEMENTS_MM = (0.0, 40.0, 80.0)

#: Direction of each displacement (unit xy).  Two directions so the displaced
#: cases do not all stroke the same legs — the 2026-08-15 "Y not rougher"
#: finding was that x and y load the leg set differently.
_DISPLACEMENT_DIRS = ((1.0, 0.0), (0.0, 1.0), (0.707, 0.707))


def _point_id(point: dict) -> str:
    return "f%.2f_dx%+.0f_dy%+.0f_%s" % (
        point['flight_s'], point['catch_xy_mm'][0] - point['throw_xy_mm'][0],
        point['catch_xy_mm'][1] - point['throw_xy_mm'][1],
        'bank' if point['banking'] else 'level')


def default_grid() -> list:
    """The pinned whole-cycle set.  Literals, not a generator over a sweep.

    Structure: every binding flight at zero displacement (the demo column), then
    every displacement in every direction at the demo flight (the re-pose column),
    then the advisory flights.  Each is run with banking ON — the banking-OFF arm
    is not a separate point, it is the matched comparison run inside every trial.
    """
    throw_xy = (0.0, 0.0)
    points = []
    for f in _BINDING_FLIGHTS_S:
        points.append(dict(flight_s=f, throw_xy_mm=throw_xy,
                           catch_xy_mm=throw_xy, banking=True, advisory=False))
    for d in _DISPLACEMENTS_MM:
        if d == 0.0:
            continue
        for ux, uy in _DISPLACEMENT_DIRS:
            points.append(dict(
                flight_s=0.60, throw_xy_mm=throw_xy,
                catch_xy_mm=(throw_xy[0] + d * ux, throw_xy[1] + d * uy),
                banking=True, advisory=False))
    for f in _ADVISORY_FLIGHTS_S:
        points.append(dict(flight_s=f, throw_xy_mm=throw_xy,
                           catch_xy_mm=throw_xy, banking=True, advisory=True))
    return points


# ---------------------------------------------------------------------------
# The production chain, and the metrics over it
# ---------------------------------------------------------------------------

def _cup_config() -> cc.CupCycleConfig:
    """The cup QP config this gate plans with — runway ON, slider-reachable box."""
    return cc.CupCycleConfig(z_min_m=CUP_Z_LO_M, z_max_m=CUP_Z_HI_M,
                             catch_runway_z_floor_m=CUP_Z_LO_M,
                             catch_runway_enabled=True)


def plan_cycle(point: dict, cfg: cc.CupCycleConfig = None):
    """``plan_window`` on one grid point.  Raises :class:`cc.CupCycleInfeasible`."""
    cfg = _cup_config() if cfg is None else cfg
    throw_pos = np.array([point['throw_xy_mm'][0] / 1000.0,
                          point['throw_xy_mm'][1] / 1000.0, THROW_CUP_Z_M])
    # Land the ball back where it was thrown from: a steady-state cycle.
    throw_target = throw_pos.copy()
    flight_s = float(point['flight_s'])
    v_take = cc.takeoff_velocity(throw_pos, throw_target, flight_s)
    catch_pos = np.array([point['catch_xy_mm'][0] / 1000.0,
                          point['catch_xy_mm'][1] / 1000.0, CATCH_CUP_Z_M])
    events = [
        cc.CatchEvent(ball_id=0, t_s=PERIOD_S * CATCH_FRAC, site=catch_pos,
                      vel=np.array(CATCH_VEL_MPS)),
        cc.ThrowEvent(ball_id=1, t_s=PERIOD_S, site=throw_pos,
                      target=throw_target, flight_s=flight_s),
    ]
    # The cycle starts JUST AFTER a throw: the achieved state IS the release
    # state, and the detach axis is that release's cup axis.
    state0 = cc.CupState(pos=throw_pos, vel=v_take, acc=cc.GRAVITY,
                         detach_axis=None)
    return cc.plan_window(events, state0, cfg, period_s=PERIOD_S)


def _pins(cup):
    """``(receive_tilt, throw_tilt)`` — the two ball-frame boundary conditions."""
    recv = np.asarray(tg.tilt_to_receive(np.asarray(CATCH_VEL_MPS)), dtype=float)
    throw = np.asarray(tg.tilt_to_throw(np.asarray(cup.takeoff_vel)), dtype=float)
    return recv, throw


def touchdown_time_s() -> float:
    """The catch instant on the cycle clock — the time the QP pins the cup to."""
    return PERIOD_S * CATCH_FRAC


def cup_state_at(cup, t: float):
    """``(pos, vel)`` of the cup at ``t``, from the plan's own piecewise cubic.

    ``cup.catch_k`` is ``floor(t_catch/dt)``, the knot BEFORE touch-down whenever
    the catch falls between knots — and the QP pins the cup's position by equality
    at the interpolated instant, not at that knot.  Scoring the catch at the knot
    therefore reads the cup mid-approach: measured 33.7 mm of "capture error" on a
    cycle whose actual error at touch-down is 8.7e-12 mm.  The whole difference is
    20 ms of a 2.4 m/s descent.
    """
    dt = float(cup.dt)
    k = max(0, min(int(np.floor(t / dt)), cup.jerk.shape[0] - 1))
    d = t - k * dt
    j = cup.jerk[k]
    pos = (cup.pos[k] + cup.vel[k] * d + 0.5 * cup.acc[k] * d ** 2
           + j * d ** 3 / 6.0)
    vel = cup.vel[k] + cup.acc[k] * d + 0.5 * j * d ** 2
    return pos, vel


def first_carry_knot(cup) -> int:
    """First knot at or after touch-down — where the ball is actually in the cup.

    The carry begins when the ball lands, not at ``cup.catch_k``.  At the knot
    before touch-down the cup is still descending to meet the ball faster than
    free fall (measured ``a_z = −13.5 m/s²``), so the apparent-gravity field there
    points the other way and a seating score that included it read 176.9° on a
    carry that is otherwise inside 1.5°.
    """
    return int(min(int(np.ceil(touchdown_time_s() / float(cup.dt))),
                   cup.pos.shape[0] - 1))


def lateral_impulse(cup, tilts) -> float:
    """``∫|(g − a_cup) × cup_axis| dt`` (m/s) — the lateral specific force the ball feels.

    The physical score for banking.  Unnormalised on purpose: near the release the
    apparent-gravity field passes through zero, and a normalised (angle-only)
    metric would weight that weightless instant as heavily as the 50 m/s² catch.
    """
    g = np.array([0.0, 0.0, -float(hw.GRAVITY_MPS2)])
    total = 0.0
    for k in range(cup.acc.shape[0]):
        total += float(np.linalg.norm(
            np.cross(g - cup.acc[k], tg.cup_axis(*tilts[k]))))
    return total * float(cup.dt)


def seat_angles(cup, tilts):
    """``(max_angle_deg, n_scored)`` between the cup axis and apparent-up over the carry.

    Scored from the catch knot to the release, and only where ``|g − a_cup|``
    exceeds :data:`SEAT_FORCE_FLOOR_MPS2`: in free fall the ball is weightless, so
    the angle there is neither meaningful nor a seating risk.
    """
    g = np.array([0.0, 0.0, -float(hw.GRAVITY_MPS2)])
    worst = 0.0
    scored = 0
    for k in range(first_carry_knot(cup), cup.acc.shape[0]):
        f = g - cup.acc[k]
        mag = float(np.linalg.norm(f))
        if mag < SEAT_FORCE_FLOOR_MPS2:
            continue
        up = -f / mag
        ax = tg.cup_axis(*tilts[k])
        c = float(np.clip(np.dot(ax, up), -1.0, 1.0))
        worst = max(worst, float(np.degrees(np.arccos(c))))
        scored += 1
    return worst, scored


def realize_tilted_parity(cup, realized) -> float:
    """Max |Δ| (mm) between the production chain and ``juggle_tilt.realize_tilted``.

    The plan's comparison 1.  Driven on the SAME cup knots and the SAME level
    tilt, so any difference is a difference in the realisation itself.  Imported
    lazily: this is the only place the gate reaches into a ``sim`` runner, and it
    is a comparison target, not a dependency of the chain under test.
    """
    from sim.juggle_tilt import realize_tilted           # noqa: PLC0415
    worst = 0.0
    for k in range(cup.pos.shape[0]):
        pose, slider = realize_tilted(cup.pos[k, :2], float(cup.pos[k, 2]),
                                      0.0, 0.0)
        worst = max(worst, float(np.max(np.abs(pose - realized.pose[k]))))
        worst = max(worst, abs(slider - float(realized.slider_mm[k])))
    return worst


def max_plannable_flight_s(step_s: float = 0.05, hi_s: float = 1.50) -> dict:
    """THE HEADLINE NUMBER: the longest flight this planner can serve, z pinned.

    Sweeps flight time at the grid's own geometry and reports the largest value
    whose realised cycle both plans and passes ``validate_cycle`` with peak hand
    acceleration inside ``JB_TRAJ_HAND_ACC_LIMIT_RPS2``.

    Why the number exists at all: under the ``z = 170`` centroid pin the SLIDER
    alone makes the launch, so peak cup-z acceleration is set at the release and
    scales with take-off speed.  The hand's 3500 rev/s² cap is therefore what
    bounds flight time, and no planner knob moves it — the cup ACCELERATION box
    added in WP4 refuses the cycle rather than finding a gentler one (measured
    2026-09-01: a 110 m/s² box at flight 0.8 s is infeasible, because the launch
    genuinely needs more than that in the stroke available below the release).
    The lever that DOES move it is the release height, i.e. how much slider travel
    is left underneath the release point.
    """
    limits = _session_limits()
    geom = StewartGeometry()
    hand_cap = float(limits.hand_acc_limit_rps2)
    rows = []
    best = float('nan')
    f = 0.35
    while f <= hi_s + 1e-9:
        f = round(f, 3)
        point = dict(flight_s=f, throw_xy_mm=(0.0, 0.0), catch_xy_mm=(0.0, 0.0),
                     banking=False, advisory=True)
        row = {'flight_s': f}
        try:
            cup = plan_cycle(point)
        except cc.CupCycleInfeasible as exc:
            row.update(planned=False, reject_code=exc.reason,
                       hand_acc_rps2=float('nan'), validate_code='')
            rows.append(row)
            f += step_s
            continue
        rcfg = cr.RealizeConfig(banking_enabled=False)
        tilts = cr.tilt_schedule(cup, np.zeros(2), np.zeros(2), rcfg)
        realized = cr.decompose(cup, tilts, rcfg)
        rep = fz.validate_cycle(CyclePlan.from_realized(realized), limits, geom)
        row.update(planned=True, reject_code=None,
                   hand_acc_rps2=float(rep.peak_hand_acc_rps2),
                   cup_acc_z_mps2=float(np.abs(cup.acc[:, 2]).max()),
                   validate_code=rep.code)
        if rep.code == fz.OK and rep.peak_hand_acc_rps2 <= hand_cap:
            best = f
        rows.append(row)
        f += step_s
    return {'max_flight_s': best, 'hand_acc_cap_rps2': hand_cap,
            'throw_cup_z_m': THROW_CUP_Z_M, 'period_s': PERIOD_S,
            'runway_below_release_m': THROW_CUP_Z_M - CUP_Z_LO_M,
            'z_pinned_mm': float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM), 'rows': rows}


def _session_limits(cfg: CycleGateConfig = None) -> TrajectoryLimits:
    cfg = CycleGateConfig() if cfg is None else cfg
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=cfg.leg_vel_mmps, leg_acc_mmps2=cfg.leg_acc_mmps2,
        leg_jerk_mmps3=cfg.leg_jerk_mmps3)


# ---------------------------------------------------------------------------
# The gate
# ---------------------------------------------------------------------------

class CycleGate:
    """Runs :func:`default_grid` (or ``cfg.points``) through the production chain."""

    def __init__(self, cfg: CycleGateConfig = None):
        self.cfg = CycleGateConfig() if cfg is None else cfg
        self.geom = StewartGeometry()
        self.limits = _session_limits(self.cfg)
        self.shipped_limits = TrajectoryLimits.from_config(hw)
        self.viewer = None
        self._plant = None

    # ── plant (advisory column only) ──
    @property
    def plant(self):
        """Lazily built MuJoCo plant.  Only the advisory column touches it."""
        if self._plant is None:
            from sim.plant.mujoco_plant import MuJoCoPlant      # noqa: PLC0415
            self._plant = MuJoCoPlant(geom=self.geom, control_dt=KNOT_DT_S,
                                      contact_carry=True)
        return self._plant

    # ── one cycle ──
    def run_trial(self, idx: int, point: dict) -> CycleTrialResult:
        d_xy = (float(point['catch_xy_mm'][0] - point['throw_xy_mm'][0]),
                float(point['catch_xy_mm'][1] - point['throw_xy_mm'][1]))
        base = dict(idx=idx, point_id=_point_id(point),
                    flight_s=float(point['flight_s']),
                    throw_xy_mm=tuple(float(v) for v in point['throw_xy_mm']),
                    catch_xy_mm=tuple(float(v) for v in point['catch_xy_mm']),
                    displacement_mm=float(np.hypot(*d_xy)),
                    banking=bool(point['banking']))
        cup_cfg = _cup_config()
        try:
            cup = plan_cycle(point, cup_cfg)
        except cc.CupCycleInfeasible as exc:
            return CycleTrialResult(accepted=False, reject_code=exc.reason, **base)

        recv, throw_tilt = _pins(cup)
        rcfg = cr.RealizeConfig(banking_enabled=bool(point['banking']))
        tilts = cr.tilt_schedule(cup, recv, throw_tilt, rcfg)
        realized = cr.decompose(cup, tilts, rcfg)
        plan = CyclePlan.from_realized(realized)

        rep = fz.validate_cycle(plan, self.limits, self.geom)
        rep_ship = fz.validate_cycle(plan, self.shipped_limits, self.geom)

        # ── capture: does the cup opening reach the requested site on time? ──
        catch_site_m = np.array([point['catch_xy_mm'][0] / 1000.0,
                                 point['catch_xy_mm'][1] / 1000.0, CATCH_CUP_Z_M])
        p_td, v_td = cup_state_at(cup, touchdown_time_s())
        capture_dist_mm = float(np.linalg.norm(p_td - catch_site_m)) * 1000.0
        v_ball = np.asarray(CATCH_VEL_MPS, dtype=float)
        v_match = float(np.linalg.norm(v_td - v_ball)
                        / max(np.linalg.norm(v_ball), 1e-9))

        # ── no ceiling-overshoot slam en route to the catch ──
        # Every knot up to and including the landing knot: the Rung-3 failure was
        # the slider buying its deceleration room by rising past the prime and
        # slamming back, so the window is the whole APPROACH, not just the seat.
        k_c = first_carry_knot(cup)
        prime = float(hw.JB_OP_HAND_CATCH_PRIME_REV)
        pre = realized.slider_rev[:k_c + 1]
        pre_max = float(np.max(pre)) if pre.size else float('nan')

        # ── seating over the carry ──
        seat_deg, seat_n = seat_angles(cup, tilts)

        # ── the two required comparisons ──
        lcfg = cr.RealizeConfig(banking_enabled=False)
        level_tilts = cr.tilt_schedule(cup, recv, throw_tilt, lcfg)
        lat_bank = lateral_impulse(cup, tilts)
        lat_level = lateral_impulse(cup, level_tilts)
        # Parity is measured on the LEVEL-pin zero-banking arm, which is the
        # realize_tilted-equivalent baseline the plan names.
        flat = cr.tilt_schedule(cup, np.zeros(2), np.zeros(2), lcfg)
        parity = realize_tilted_parity(cup, cr.decompose(cup, flat, lcfg))

        d1 = np.diff(tilts, axis=0)
        d2 = tilts[:-2] - 2.0 * tilts[1:-1] + tilts[2:] if len(tilts) > 2 else None

        validate_ok = bool(rep.code == fz.OK)
        capture_ok = bool(capture_dist_mm <= CAPTURE_TOL_MM)
        seat_ok = bool(seat_n > 0 and seat_deg <= SEAT_CONE_DEG)
        no_slam = bool(np.isfinite(pre_max) and pre_max <= prime)
        # The runway band is a MEASURED fact, not a restatement of the config: read
        # the planner's own requirement back and check the catch really leaves that
        # much stroke under it.  A boolean echo of ``catch_runway_enabled`` would
        # pass just as happily on a program whose row had stopped being assembled,
        # which is the vacuity this whole band exists to prevent.
        floor_m, need_m = cc.catch_runway_requirement(
            np.asarray(CATCH_VEL_MPS, dtype=float), cup_cfg)
        runway_margin_m = float(CATCH_CUP_Z_M - floor_m - need_m)
        runway_active = bool(cup_cfg.catch_runway_enabled
                             and runway_margin_m >= -1e-12)

        res = CycleTrialResult(
            accepted=True, reject_code=None,
            validate_ok=validate_ok, capture_ok=capture_ok, seat_ok=seat_ok,
            no_slam=no_slam, runway_active=runway_active,
            core_clean=bool(validate_ok and capture_ok and seat_ok and no_slam
                            and runway_active),
            validate_code=rep.code, validate_shipped_code=rep_ship.code,
            peak_leg_vel_mmps=float(rep.peak_leg_vel_mmps),
            peak_leg_acc_mmps2=float(rep.peak_leg_acc_mmps2),
            peak_leg_jerk_mmps3=float(rep.peak_leg_jerk_mmps3),
            peak_leg_ext_mm=float(rep.peak_leg_ext_mm),
            peak_hand_rev=float(rep.peak_hand_rev),
            peak_hand_vel_rps=float(rep.peak_hand_vel_rps),
            peak_hand_acc_rps2=float(rep.peak_hand_acc_rps2),
            peak_hand_step_rev=float(rep.peak_hand_step_rev),
            capture_dist_mm=capture_dist_mm, v_match=v_match,
            max_seat_angle_deg=seat_deg, seat_scored_knots=seat_n,
            pre_catch_hand_max_rev=pre_max, runway_margin_m=runway_margin_m,
            max_tilt_deg=float(np.degrees(np.linalg.norm(tilts, axis=1)).max()),
            tilt_rate_max_rad_s=float(np.linalg.norm(d1, axis=1).max()
                                      / cup.dt) if len(d1) else 0.0,
            tilt_accel_max_rad_s2=(float(np.hypot(d2[:, 0], d2[:, 1]).max())
                                   / cup.dt ** 2) if d2 is not None else 0.0,
            lateral_impulse_banked=lat_bank, lateral_impulse_level=lat_level,
            banking_gain_frac=float((lat_level - lat_bank) / lat_level)
            if lat_level > 0 else float('nan'),
            parity_max_abs=parity,
            solver_iters=int(cup.warm_start.iters) if cup.warm_start else -1,
            **base)

        if self.cfg.contact_diag:
            mj = self._mujoco_advisory(cup, realized)
            res.mj_makes = mj['makes']
            res.mj_drops = mj['drops']
            res.mj_held_at_end = mj['held_at_end']
            res.mj_carry_held_frac = float(mj.get('carry_held_frac', float('nan')))
        return res

    # ── advisory MuJoCo column ──
    def _mujoco_advisory(self, cup, realized) -> dict:
        """Step the planned cycle in MuJoCo with a ball on a matched arrival arc.

        ADVISORY ONLY — see :data:`_MUJOCO_NOTE`.  Never raises into the gate: a
        MuJoCo failure returns the sentinel row rather than taking the run down,
        because a column that cannot fail the gate must not be able to abort it.
        """
        out = {'makes': 0, 'drops': 0, 'held_at_end': False}
        try:
            plant = self.plant
            plant.reset(NEUTRAL_POSE)
            plant.command(plant.pose_to_extensions(NEUTRAL_POSE))
            plant.command_hand(float(realized.slider_mm[0]))
            for _ in range(40):
                plant.step(KNOT_DT_S)

            # The ball flies a ballistic arc that passes through the planned catch
            # site at the planned instant with the planned arrival velocity.  It is
            # released on a SHORT LEAD rather than back-propagated to t = 0: the
            # full back-propagation puts the spawn 152 mm BELOW the floor plane
            # (measured), where the ground geom owns the ball before the cup ever
            # sees it — an advisory column that silently reported 0 makes because
            # its ball was underground would be worse than no column at all.
            t_td = touchdown_time_s()
            t_spawn = max(0.0, t_td - _BALL_SPAWN_LEAD_S)
            k_spawn = int(round(t_spawn / float(cup.dt)))
            tau = t_td - k_spawn * float(cup.dt)
            v_arr = np.asarray(CATCH_VEL_MPS, dtype=float)
            g_vec = np.array([0.0, 0.0, float(hw.GRAVITY_MPS2)])
            p_catch, _ = cup_state_at(cup, t_td)
            v_spawn = v_arr + g_vec * tau
            p_spawn = p_catch - v_arr * tau - 0.5 * g_vec * tau ** 2

            substeps = max(1, int(round(KNOT_DT_S / plant.timestep)))
            n_knots = realized.pose.shape[0]
            # A loss of contact in the LAST few knots is the launch — the cycle
            # ends at a release, and counting that as a drop would score the
            # planner's own throw as a failure.  ``cup_cycle``'s detach block is
            # ``n_detach`` knots long; one extra knot covers the ramp into it.
            launch_k = n_knots - (int(cc.CupCycleConfig().n_detach) + 1)
            caught = False
            lost = False
            carry_knots = 0
            held_knots = 0
            k_land = first_carry_knot(cup)
            for k in range(n_knots):
                plant.command(plant.pose_to_extensions(realized.pose[k]))
                plant.command_hand(float(realized.slider_mm[k]))
                if k == k_spawn:
                    plant.spawn_ball(p_spawn * 1000.0, v_spawn * 1000.0)
                for _ in range(substeps):
                    plant.step(plant.timestep)
                    if k < k_spawn:
                        continue
                    if not caught and plant.check_and_capture():
                        caught = True
                        out['makes'] += 1
                bs = plant.get_ball_state()
                if k_land <= k < launch_k:
                    carry_knots += 1
                    if bs is not None and bs.held:
                        held_knots += 1
                if caught and not lost and k < launch_k:
                    if bs is not None and not bs.held:
                        lost = True
                        out['drops'] += 1
                        out['drop_t_s'] = float(k) * float(cup.dt)
                if self.viewer is not None:
                    self.viewer.sync()
            bs = plant.get_ball_state()
            out['held_at_end'] = bool(bs is not None and bs.held)
            out['spawn_z_mm'] = float(p_spawn[2] * 1000.0)
            out['carry_held_frac'] = (float(held_knots) / carry_knots
                                      if carry_knots else float('nan'))
        except ViewerClosed:
            raise
        except Exception as exc:                       # advisory: never gate on it
            out['error'] = '%s: %s' % (type(exc).__name__, exc)
        return out

    # ── run + summarise ──
    def run(self) -> dict:
        points = (default_grid() if self.cfg.points is None
                  else list(self.cfg.points))
        results = []
        idx = 0
        for point in points:
            for _ in range(max(1, int(self.cfg.trials_per_point))):
                results.append(self.run_trial(idx, point))
                idx += 1
        return self._summarise(points, results)

    def _summarise(self, points, results) -> dict:
        by_point = {}
        for r in results:
            by_point.setdefault(r.point_id, []).append(r)

        rows = []
        binding_ok = True
        any_binding = False
        for point in points:
            pid = _point_id(point)
            rs = by_point.get(pid, [])
            n = len(rs)
            clean = sum(1 for r in rs if r.core_clean)
            thr = _pass_threshold(n) if n else 0
            row = {
                'point_id': pid, 'flight_s': float(point['flight_s']),
                'throw_xy_mm': list(point['throw_xy_mm']),
                'catch_xy_mm': list(point['catch_xy_mm']),
                'advisory': bool(point.get('advisory', False)),
                'n': n, 'core_clean': clean, 'pass_threshold': thr,
                'passed': bool(n and clean >= thr),
            }
            if not row['advisory']:
                any_binding = True
                binding_ok = binding_ok and row['passed']
            rows.append(row)

        accepted = [r for r in results if r.accepted]
        # Invariants: every accepted cycle must satisfy the structural bands, and
        # the zero-banking parity must be EXACT.  These are separate from the
        # per-point threshold so a systemic break cannot hide behind 9-of-10.
        parity_worst = max((r.parity_max_abs for r in accepted), default=float('nan'))
        parity_ok = bool(accepted) and parity_worst == 0.0
        banked = [r for r in accepted if r.banking]
        banking_wins = sum(1 for r in banked
                           if r.lateral_impulse_banked < r.lateral_impulse_level)
        banking_ok = bool(banked) and banking_wins == len(banked)
        runway_ok = all(r.runway_active for r in accepted) and bool(accepted)
        slam_free = all(r.no_slam for r in accepted)

        invariants_ok = bool(parity_ok and banking_ok and runway_ok and slam_free)
        passed = bool(rows and any_binding and binding_ok and invariants_ok)

        def _worst(attr, default=float('nan')):
            vals = [getattr(r, attr) for r in accepted
                    if np.isfinite(getattr(r, attr))]
            return float(max(vals)) if vals else default

        report = {
            'gate': 'cycle',
            'passed': passed,
            'binding_band_passed': bool(any_binding and binding_ok),
            'no_binding_points': not any_binding,
            'parity_exact': parity_ok,
            'parity_worst_abs': float(parity_worst),
            'banking_beats_level': banking_ok,
            'banking_wins': banking_wins,
            'banking_arms': len(banked),
            'runway_active_everywhere': runway_ok,
            'worst_runway_margin_m': min((r.runway_margin_m for r in accepted
                                          if np.isfinite(r.runway_margin_m)),
                                         default=float('nan')),
            'slam_free_everywhere': bool(slam_free),
            'trials': len(results),
            'accepted': len(accepted),
            'core_clean': sum(1 for r in results if r.core_clean),
            'rejected': [{'point_id': r.point_id, 'reject_code': r.reject_code}
                         for r in results if not r.accepted],
            'worst_leg_vel_mmps': _worst('peak_leg_vel_mmps'),
            'worst_leg_acc_mmps2': _worst('peak_leg_acc_mmps2'),
            'worst_leg_jerk_mmps3': _worst('peak_leg_jerk_mmps3'),
            'worst_hand_vel_rps': _worst('peak_hand_vel_rps'),
            'worst_hand_acc_rps2': _worst('peak_hand_acc_rps2'),
            'worst_hand_rev': _worst('peak_hand_rev'),
            'worst_capture_dist_mm': _worst('capture_dist_mm'),
            'worst_seat_angle_deg': _worst('max_seat_angle_deg'),
            'worst_tilt_deg': _worst('max_tilt_deg'),
            'worst_tilt_accel_rad_s2': _worst('tilt_accel_max_rad_s2'),
            'required_leg_vel_mmps': _worst('peak_leg_vel_mmps', 0.0) * 1.15,
            'required_leg_acc_mmps2': _worst('peak_leg_acc_mmps2', 0.0) * 1.15,
            'required_leg_jerk_mmps3': _worst('peak_leg_jerk_mmps3', 0.0) * 1.15,
            'shipped_limit_verdicts': sorted({r.validate_shipped_code
                                              for r in accepted}),
            'shipped_limit_note': _SHIPPED_LIMIT_NOTE,
            'deferred_criteria': ['v_match'],
            'deferred_note': _DEFERRED_NOTE,
            'contact_diagnostic': {'gating': False, 'note': _MUJOCO_NOTE,
                                   'makes': sum(max(0, r.mj_makes)
                                                for r in accepted),
                                   'drops': sum(max(0, r.mj_drops)
                                                for r in accepted),
                                   'held_at_end': sum(1 for r in accepted
                                                      if r.mj_held_at_end),
                                   'mean_carry_held_frac': _mean_finite(
                                       [r.mj_carry_held_frac
                                        for r in accepted]),
                                   'ran': bool(self.cfg.contact_diag)},
            'points': rows,
            'config': dataclasses.asdict(self.cfg),
            'thresholds': {
                'capture_tol_mm': CAPTURE_TOL_MM,
                'seat_cone_deg': SEAT_CONE_DEG,
                'seat_force_floor_mps2': SEAT_FORCE_FLOOR_MPS2,
                'hand_catch_prime_rev': float(hw.JB_OP_HAND_CATCH_PRIME_REV),
                'cup_z_box_m': [CUP_Z_LO_M, CUP_Z_HI_M],
                'throw_cup_z_m': THROW_CUP_Z_M,
                'reach_envelope_mm': REACH_ENVELOPE_MM,
                'session_leg_vel_mmps': self.cfg.leg_vel_mmps,
                'session_leg_acc_mmps2': self.cfg.leg_acc_mmps2,
                'session_leg_jerk_mmps3': self.cfg.leg_jerk_mmps3,
                'tilt_accel_limit_rad_s2': cr.TILT_ACCEL_LIMIT_DEFAULT_RAD_S2,
            },
            'results': [r.to_dict() for r in results],
        }
        if self.cfg.flight_sweep:
            report['flight_headline'] = max_plannable_flight_s()
        return report


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------

def run_gate(cfg: CycleGateConfig = None, viewer_speed: float = None) -> dict:
    """Run the gate, write the JSON report, return it."""
    cfg = CycleGateConfig() if cfg is None else cfg
    gate = CycleGate(cfg)
    t0 = time.time()
    try:
        if viewer_speed is not None:
            attach_viewer(gate, viewer_speed, tag='cycle_gate')
        report = gate.run()
    except ViewerClosed:
        print('[cycle_gate] viewer closed by the operator — no report written.')
        raise SystemExit(130)
    finally:
        if gate.viewer is not None:
            gate.viewer.close()
    report['wall_s'] = time.time() - t0

    path = cfg.report_path
    if path is None:
        out_dir = os.path.join(_repo_root, 'temp', 'reports')
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, 'cycle_gate_seed%d_n%d.json'
                            % (cfg.seed, cfg.trials_per_point))
    with open(path, 'w') as fh:
        json.dump(report, fh, indent=2)
    report['report_path'] = path
    return report


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--trials-per-point', '--trials', dest='trials',
                   type=int, default=1)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--flight', type=float, default=None,
                   help='single-point run: flight time (s)')
    p.add_argument('--displacement', type=float, default=None,
                   help='single-point run: catch displacement (mm, +x)')
    p.add_argument('--no-diag', action='store_true',
                   help='skip the advisory MuJoCo column')
    p.add_argument('--no-sweep', action='store_true',
                   help='skip the flight-time headline sweep')
    p.add_argument('--viewer', action='store_true')
    p.add_argument('--viewer-speed', type=float, default=1.0)
    p.add_argument('--report', default=None)
    args = p.parse_args(argv)

    points = None
    if (args.flight is None) != (args.displacement is None):
        p.error('--flight and --displacement must be given together')
    if args.flight is not None:
        points = [dict(flight_s=args.flight, throw_xy_mm=(0.0, 0.0),
                       catch_xy_mm=(args.displacement, 0.0),
                       banking=True, advisory=False)]

    cfg = CycleGateConfig(points=points, trials_per_point=args.trials,
                          seed=args.seed, contact_diag=not args.no_diag,
                          flight_sweep=not args.no_sweep,
                          report_path=args.report)
    rep = run_gate(cfg, viewer_speed=args.viewer_speed if args.viewer else None)

    for row in rep['points']:
        tag = 'ADVISORY' if row['advisory'] else 'binding'
        verdict = 'PASS' if row['passed'] else 'FAIL'
        print('[cycle_gate]   %s: core_clean %d/%d %s  (%s)'
              % (row['point_id'], row['core_clean'], row['n'],
                 verdict if not row['advisory'] else '----', tag))
    for r in rep['rejected']:
        print('[cycle_gate]   REJECTED %s [%s]'
              % (r['point_id'], r['reject_code']))
    print('[cycle_gate] %s  binding-band %s  parity %s (worst %.3g)  '
          'banking-beats-level %s (%d/%d)  slam-free %s  (v_match deferred)'
          % ('PASS' if rep['passed'] else 'FAIL',
             'PASS' if rep['binding_band_passed'] else 'FAIL',
             'EXACT' if rep['parity_exact'] else 'BROKEN',
             rep['parity_worst_abs'],
             'YES' if rep['banking_beats_level'] else 'NO',
             rep['banking_wins'], rep['banking_arms'],
             'YES' if rep['slam_free_everywhere'] else 'NO'))
    print('[cycle_gate] trials %d  accepted %d  core_clean %d  '
          'shipped-limit verdicts %s'
          % (rep['trials'], rep['accepted'], rep['core_clean'],
             rep['shipped_limit_verdicts']))
    print('[cycle_gate] worst: leg vel %.1f acc %.1f jerk %.0f | hand vel %.1f '
          'acc %.1f rev %.3f | capture %.3f mm  seat %.2f deg  tilt %.2f deg'
          % (rep['worst_leg_vel_mmps'], rep['worst_leg_acc_mmps2'],
             rep['worst_leg_jerk_mmps3'], rep['worst_hand_vel_rps'],
             rep['worst_hand_acc_rps2'], rep['worst_hand_rev'],
             rep['worst_capture_dist_mm'], rep['worst_seat_angle_deg'],
             rep['worst_tilt_deg']))
    diag = rep['contact_diagnostic']
    print('[cycle_gate] MuJoCo ADVISORY (never gating): makes %d  drops %d  '
          'held_at_end %d  mean carry-held %.2f  ran %s'
          % (diag['makes'], diag['drops'], diag['held_at_end'],
             diag['mean_carry_held_frac'], diag['ran']))
    if 'flight_headline' in rep:
        fh = rep['flight_headline']
        print('[cycle_gate] HEADLINE: max plannable flight %s s under the %.0f '
              'rev/s^2 hand cap, z pinned at %.0f mm, release cup z %.2f m '
              '(%.3f m of slider runway below it)'
              % (fh['max_flight_s'], fh['hand_acc_cap_rps2'], fh['z_pinned_mm'],
                 fh['throw_cup_z_m'], fh['runway_below_release_m']))
    print('[cycle_gate] required limits: vel %.0f acc %.0f jerk %.0f  '
          '(wall %.1fs)  → %s'
          % (rep['required_leg_vel_mmps'], rep['required_leg_acc_mmps2'],
             rep['required_leg_jerk_mmps3'], rep['wall_s'], rep['report_path']))
    return 0 if rep['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
