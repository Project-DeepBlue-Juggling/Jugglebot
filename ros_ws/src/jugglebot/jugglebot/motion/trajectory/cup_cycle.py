"""Whole-cycle cup planner — the dense-QP port of ``plan_cup_cycle``.

WHAT THIS IS
------------
One cup cycle (just-after-throw → catch → next throw) planned in task space as
a **convex quadratic program** over the cup's Cartesian jerk, solved in-process
with numpy alone. It is the production port of
``sim/juggle_planner/juggle_planner.py::plan_cup_cycle`` (a CasADi/IPOPT
program) required by ``plans/active/unified-7dof-planner.md`` Phase 1: this
package is pure-Python / numpy-only / Python 3.8 by architectural rule, and
CasADi is not importable here.

The formulation is a *transcription*, not a re-derivation. Same triple-integrator
rollout, same objective weights, same hard equalities (throw pos/vel/acc==g,
detach collinearity, interpolated catch position), same jerk and workspace
boxes. Parity against the frozen CasADi reference is the contract and is pinned
by ``tests/motion/test_cup_cycle.py`` (T-U2) against the committed fixtures in
``tools/probes/data/cup_cycle_qp_refs.npz``.

WHY A GOLDFARB–IDNANI DUAL ACTIVE SET
-------------------------------------
Phase 0 decision 1 (2026-08-30,
``logbook/2026-08-30-unified-7dof-planner-phase0-probes.md``) **binds** this
module to a Goldfarb–Idnani dual active-set solver. The obvious alternative —
solve the equality-constrained KKT system, add every violated box to the working
set, repeat — was measured to **cycle**: working sets grew to rank 37 of 53,
``cond(K)`` reached ~1e23, and it returned trajectories **124 m** from the
reference *without raising*. A silently wrong cup trajectory is the worst
available failure mode on this path. G–I is monotone in the dual objective, so
it terminates finitely by construction, and every solve here runs
``np.linalg.solve`` (the measured case set never needed an ``lstsq`` fallback,
so none is offered — a fallback that silently absorbs a singular working set is
the same silent-wrongness hazard in another coat).

Finite termination is not the same as a *correct* answer, so **every solve is
verified before it is returned** (:func:`_verify`): a 0.40 s grid point found
while building T-U1 drove the working set to 38 of 48 rows, made ``NᵀH⁻¹N``
numerically singular, and terminated with every inequality satisfied and an
equality residual of **121** — the identical silent-wrongness shape, reached
through conditioning rather than through cycling. IPOPT refuses that same
cycle, so it is genuinely infeasible; the QP now refuses it too, loudly.

Two conditioning details are load-bearing, both measured in Phase 0:

* **The tilted detach cross-product is rank 2.** Written literally as three
  ``cross(a − g, axis) == 0`` rows it is a rank-deficient equality block that
  makes the KKT matrix singular. It is enforced instead as **two orthonormal
  rows perpendicular to the detach axis**, verified equivalent to the literal
  cross-product form to ≤ 3.3e-16.
* **Equality rows are 2-norm normalised.** Without it ``cond(K)`` runs 1e5–1e7,
  driven by the ``dt³/6`` coefficients in the interpolated-catch rows. This is
  what keeps ``np.linalg.solve`` sufficient.

THE CATCH-RUNWAY CONSTRAINT (the one addition over the sim planner)
-------------------------------------------------------------------
Owner resolution 1 (2026-08-29, plan § 4 Phase 1): the catch knot must leave
``≥ v² / (2·a_hand_max)`` of slider stroke below it, plus a margin, so the
deceleration room after the seat is *planned* rather than obtained by overshoot.
``v`` is the **target** catch speed (``catch_slider_vel_ratio × |v_ball,z|``), a
parameter — so the bound stays linear and the program stays a convex QP.

It is applied in two places, and the split is deliberate:

1. **A hard linear inequality row on ``pos[catch_k, 2]``** inside the QP — a
   genuinely free variable, so this shapes the trajectory.
2. **An exact analytic gate on the interpolated touch-down height**, checked
   before the solve. *Surfaced explicitly*: the catch-position equality pins all
   three components of the cup position at touch-down to ``catch_pos``, so at
   that instant the runway requirement has no freedom left to shape — it is a
   **refusal** on the requested catch, not a shaper. Raising
   :class:`CupCycleInfeasible` with the numbers is strictly better than letting
   the same fact reappear as a dual-unbounded step deep inside the solver.

The geometry arrives through explicit cfg fields
(:attr:`CupCycleConfig.catch_runway_z_floor_m` — the cup z when the slider is at
the bottom of its stroke — and :attr:`CupCycleConfig.catch_runway_margin_m`),
never by importing platform geometry here; the floor defaults to ``z_min_m``,
which is already documented in the sim planner as the caller's slider-reachable
range. ``a_hand_max`` defaults to the owner-signed 3500 rev/s² divided by the
package's existing slider gain (see :data:`HAND_MAX_DECEL_MPS2`).

The whole constraint is disableable (``catch_runway_enabled = False``) so exact
legacy parity against the CasADi reference stays assertable. **WP3's
``validate_cycle`` re-checks the runway post hoc with the ACHIEVED catch
velocity** — this bound uses the target, which the soft velocity match only
approaches.

EVENT-TIMELINE API
------------------
Owner resolution 3 (2026-08-29): the entry point is :func:`plan_window`, which
takes an ordered list of :class:`ThrowEvent` / :class:`CatchEvent` over a horizon
window. v1 callers pass **exactly one throw and one catch**, on which parity with
``plan_cup_cycle`` is asserted; 3-ball later becomes data on the timeline rather
than an interface rewrite. :func:`plan_cup_cycle` is a thin
signature-compatible wrapper over it so the WP4 sim-parity harness can drive
either planner interchangeably.

Units are SI (m, s) throughout — the ballistics are natural in SI and this
mirrors the sim planner and Kai Ploeger's original. Callers in the mm-based
trajectory stack convert at the boundary.

Pure Python + numpy, plus the package's own ``hand_stroke`` for the slider
gain. No ROS2, no repo-root / ``controller`` / ``sim`` imports.
"""

from __future__ import annotations

import dataclasses
from typing import Optional, Sequence

import numpy as np

from jugglebot.motion.trajectory.hand_stroke import LINEAR_GAIN_REV_PER_M

#: Ballistics gravity — the SAME value the sim planner uses, so the ported
#: take-off velocities are bit-comparable. NOT the tracker's 9.81.
GRAVITY = np.array([0.0, 0.0, -9.806])

#: Owner-signed hand acceleration LIMIT (Phase 0 decision 4, 2026-08-30):
#: 3500 rev/s², under the C-HAND-2 authority bound of 3925.5 rev/s².
HAND_ACC_LIMIT_RPS2 = 3500.0

#: Default ``a_hand_max`` for the catch runway (m/s²) ≈ 110.70 — the hand
#: acceleration limit converted through the package's existing slider gain
#: (``hand_stroke.LINEAR_GAIN_REV_PER_M``, derived from the firmware spool
#: geometry and equal to the generated ``TEENSY_LINEAR_GAIN``). Imported rather
#: than restated: a second spelling of the same 31.617 rev/m is a number that
#: drifts, and ``hand_stroke`` is already the module every other consumer of the
#: gain reaches for.
HAND_MAX_DECEL_MPS2 = HAND_ACC_LIMIT_RPS2 / LINEAR_GAIN_REV_PER_M


class CupCycleInfeasible(RuntimeError):
    """The requested cycle has no feasible cup trajectory.

    A ``RuntimeError`` subclass on purpose: ``plan_cup_cycle``'s contract is
    "raises on infeasibility" (``opti.solve()`` raises ``RuntimeError``), so
    existing ``except RuntimeError`` callers keep working, while new callers can
    catch this precisely and read :attr:`reason`.
    """

    def __init__(self, message: str, reason: str = 'INFEASIBLE'):
        super().__init__(message)
        self.reason = reason


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class CupCycleConfig:
    """Knobs for :func:`plan_window` (SI units).

    Fields up to ``solver_print`` mirror ``sim.juggle_planner.PlannerConfig``
    **name for name and default for default** — the parity contract is that the
    two configs are interchangeable. The rationale for each of those knobs lives
    in the sim dataclass and is not duplicated here; only what this port adds is
    documented at length.
    """

    dt: float = 0.025                       # planner timestep (s); 40 Hz
    max_jerk_z: float = 6000.0              # |cup z jerk| (m/s³) — slider
    max_jerk_xy: float = 300.0              # |cup xy jerk| (m/s³) — platform
    lateral_accel_weight: float = 3.0
    n_detach: int = 2
    throw_coast_knots: int = 0
    throw_coast_weight: float = 0.0
    catch_vel_ratio: float = 0.7            # LATERAL (xy) match ratio
    catch_vel_weight: float = 200.0
    catch_slider_vel_ratio: float = 0.7     # VERTICAL (z) match ratio — slider
    catch_slider_vel_weight: float = 600.0
    catch_dwell_pre: int = 0
    catch_dwell_post: int = 0
    catch_dwell_weight: float = 0.0
    workspace_xy_m: float = 0.15            # |cup xy − centre| bound (m)
    z_min_m: float = 0.45
    z_max_m: float = 1.10
    solver_print: bool = False              # accepted, unused (no NLP here)

    # ---- catch runway (owner resolution 1, 2026-08-29) --------------------
    #: Hard runway bound ON. Set False for exact legacy parity against the
    #: CasADi reference (that program has no runway constraint).
    catch_runway_enabled: bool = True
    #: Cup z (m, world) with the slider at the BOTTOM of its stroke. ``None``
    #: falls back to ``z_min_m``, which the sim planner already documents as
    #: "the caller MUST override this to the SLIDER's reachable range".
    catch_runway_z_floor_m: Optional[float] = None
    #: Extra stroke reserved below the computed deceleration distance (m).
    catch_runway_margin_m: float = 0.020
    #: Deceleration authority used for ``v²/(2a)`` (m/s²).
    catch_runway_decel_mps2: float = HAND_MAX_DECEL_MPS2

    # ---- cup acceleration boxes (WP4, 2026-09-01) -------------------------
    #: Optional ``|acc_z| <= max_acc_z`` bound on the cup (m/s²). ``None`` = OFF,
    #: which is the default and what keeps T-U1/T-U2 parity with the CasADi
    #: reference exact (that program has no acceleration box either).
    #:
    #: **Why it exists.** The program already boxes cup *jerk* on both channels but
    #: nothing bounds cup *acceleration*, and under the ``z = 170`` centroid pin the
    #: SLIDER alone generates the whole launch acceleration. WP3 measured the
    #: consequence: peak cup-z acceleration is set at the release and scales with
    #: take-off speed — 140.2 m/s² (4432 rev/s²) at flight 0.8 s versus 43.2 m/s²
    #: (1367 rev/s²) at 0.5 s — so the 3500 rev/s² hand cap, not any planner knob,
    #: is what decides the admissible flight time. Lowering ``max_jerk_z`` is not a
    #: substitute: it does not move the peak at all until it goes low enough to
    #: make the program infeasible outright (measured 2026-09-01 — 6000 through
    #: 2000 all leave the peak at 1433 m/s³, 500 refuses).
    #:
    #: Acceleration is **affine in the jerk decision** (``acc = Ca·s0 + Aa·x``), so
    #: this is a pair of linear box rows per knot — the program stays a convex QP
    #: and the Goldfarb–Idnani solver handles them exactly as it handles the jerk
    #: and workspace boxes. It must admit ``|g|``: the release equality pins
    #: ``acc == g``, so a bound below 9.81 m/s² is infeasible by construction and is
    #: refused with that reason rather than as a dual-unbounded step.
    max_acc_z: Optional[float] = None
    #: Optional ``|acc_x|, |acc_y| <= max_acc_xy`` bound on the cup (m/s²). ``None``
    #: = OFF. The lateral counterpart, whose budget is the LEG acceleration limit
    #: rather than the hand's: lateral cup acceleration is made by the platform.
    max_acc_xy: Optional[float] = None

    # ---- solver ----------------------------------------------------------
    max_iter: int = 300
    tol: float = 1e-9
    #: Post-solve feasibility bar, in the QP's row-normalised units. The solver
    #: REFUSES any point whose equality residual or constraint violation exceeds
    #: it. Measured separation on this problem family (2026-08-30): a converged
    #: solve lands at 1e-13…1e-12, while the one grid point that drifted off the
    #: equality manifold landed at 1.2e+2 — five orders of headroom either side.
    #: See :func:`_solve_qp` for why the bar exists at all.
    feas_tol: float = 1e-7


# The runway knobs are the only fields a sim ``PlannerConfig`` will not have.
# Reading them through ``getattr`` is what makes the wrapper genuinely drop-in
# for the WP4 harness: handed the sim config, this planner runs the legacy
# constraint set exactly, rather than crashing or inventing a default.
_RUNWAY_DEFAULTS = {
    'catch_runway_enabled': False,
    'catch_runway_z_floor_m': None,
    'catch_runway_margin_m': 0.020,
    'catch_runway_decel_mps2': HAND_MAX_DECEL_MPS2,
    'max_acc_z': None,
    'max_acc_xy': None,
    'max_iter': 300,
    'tol': 1e-9,
    'feas_tol': 1e-7,
}


def _cfg(cfg, name):
    """Read ``name`` off ``cfg``, defaulting the fields a sim config lacks."""
    if name in _RUNWAY_DEFAULTS:
        return getattr(cfg, name, _RUNWAY_DEFAULTS[name])
    return getattr(cfg, name)


# ---------------------------------------------------------------------------
# Timeline events + window state
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class ThrowEvent:
    """A release on the window timeline.

    ``site`` is the cup position at release (m), ``target`` the world position
    the ball should land at ``flight_s`` later. ``t_s`` is measured from the
    window start.
    """

    ball_id: int
    t_s: float
    site: np.ndarray
    target: np.ndarray
    flight_s: float


@dataclasses.dataclass
class CatchEvent:
    """A touch-down on the window timeline.

    ``site`` / ``vel`` are the OBSERVED incoming ball's touch-down position and
    velocity (m, m/s) — i.e. ``ballistic_touchdown``'s output. ``t_s`` is
    measured from the window start.
    """

    ball_id: int
    t_s: float
    site: np.ndarray
    vel: np.ndarray


@dataclasses.dataclass
class CupState:
    """Cup boundary condition at the window start.

    ``pos``/``vel``/``acc`` are the ACHIEVED cup state (m, m/s, m/s²) — passing
    the achieved state is what makes the re-plan closed-loop. ``detach_axis`` is
    the cup's symmetry (up) axis in the world frame for the ball released at the
    window start; it belongs here rather than on a :class:`ThrowEvent` because
    that release happened BEFORE this window and only its axis survives into it.
    ``None`` (or world +z) is the flat cup, for which the detach constraint
    reduces exactly to ``acc_xy == 0``.

    ``post_release`` says whether this window actually FOLLOWS a release, and it
    is what decides whether the detach-cone equalities are assembled at all. It
    defaults to ``True`` because that is the only case v1 had: the steady-state
    window starts the instant a ball leaves the cup, so knots ``1..n_detach``
    must keep the cup's acceleration collinear with ``detach_axis`` or the ball
    gets a lateral shove on its way out. A window that does NOT follow a release
    — a launch from rest, or the tail half of a mid-cycle re-plan — has no ball
    leaving, and imposing those rows there is not conservative: it pins the
    *acceleration direction* of the first two knots for no physical reason, which
    on a launch forbids the cup from accelerating laterally out of rest at all.
    Set it False for those windows.
    """

    pos: np.ndarray
    vel: np.ndarray
    acc: np.ndarray
    detach_axis: Optional[np.ndarray] = None
    post_release: bool = True


@dataclasses.dataclass
class SolverState:
    """Working set + diagnostics from one solve; feed back as ``warm_start``.

    ``key`` fingerprints the problem shape (``n_steps``, ``dt``, inequality
    count). A warm start whose key does not match the new problem is ignored
    silently — the shape changed, so the indices mean nothing.
    """

    active: tuple
    key: tuple
    iters: int


@dataclasses.dataclass
class CupCyclePlan:
    """One planned cup cycle, sampled on the ``dt`` grid (SI units).

    The first eight fields are ``sim.juggle_planner.CupCyclePlan``'s, in the
    same order with the same shapes, so downstream code duck-types across both
    planners: ``pos``/``vel``/``acc`` are ``(n_steps+1, 3)``, ``jerk`` is
    ``(n_steps, 3)``, ``t`` the per-knot time (s) from the cycle start,
    ``catch_k`` the knot index at/just before the catch, ``takeoff_vel`` the
    planned release velocity. ``warm_start`` is an additive trailing field the
    sim dataclass does not have (default ``None``), carrying the solver working
    set for the next cycle.

    **The two sentinels for a window that lacks an event** (see
    :func:`plan_window`, which accepts 0 or 1 of each):

    * ``catch_k == -1`` — no catch in this window. Every consumer in the tree
      already tests ``0 <= catch_k`` before using it (``cup_realize.
      tilt_schedule``'s catch pin, ``feasibility.validate_cycle``'s runway
      pass), so the sentinel needs no new branches downstream.
    * ``takeoff_vel == [0, 0, 0]`` — no throw in this window (the terminal
      condition is rest, not a release). ZEROS rather than ``None`` because the
      one consumer of this field is ``tilt_geometry.tilt_to_throw``, which maps
      a zero-magnitude velocity to the LEVEL tilt ``(0, 0)`` — exactly the right
      terminal cup attitude for a plan that ends at rest — whereas ``None``
      would raise a ``TypeError`` inside ``np.asarray`` at every existing call
      site. A caller that must distinguish "no throw" from "a throw with zero
      take-off velocity" cannot: the latter is not a throw.
    """

    pos: np.ndarray
    vel: np.ndarray
    acc: np.ndarray
    jerk: np.ndarray
    t: np.ndarray
    dt: float
    catch_k: int
    takeoff_vel: np.ndarray
    warm_start: Optional[SolverState] = None

    def sample(self, t_rel: float):
        """Cubic (jerk-constant) interpolation of the cup state at ``t_rel``.

        Clamps to the cycle window. Returns ``(pos, vel, acc)`` 3-vectors (SI).
        Identical body to the sim dataclass's, so the two sample the same.
        """
        n = self.jerk.shape[0]
        t_rel = max(0.0, min(float(t_rel), n * self.dt))
        k = min(int(t_rel / self.dt), n - 1)
        dtau = t_rel - k * self.dt
        p = (self.pos[k] + self.vel[k] * dtau + 0.5 * self.acc[k] * dtau ** 2
             + (1.0 / 6.0) * self.jerk[k] * dtau ** 3)
        v = self.vel[k] + self.acc[k] * dtau + 0.5 * self.jerk[k] * dtau ** 2
        a = self.acc[k] + self.jerk[k] * dtau
        return p, v, a


# ---------------------------------------------------------------------------
# Ballistics (local, SI)
# ---------------------------------------------------------------------------

def takeoff_velocity(throw_pos, target_pos, flight_s, g=GRAVITY):
    """Ballistic launch velocity (m/s): ``v = (target − throw)/T − 0.5 g T``.

    Deliberately local rather than reused from ``ballistics_bc`` /
    ``toss_release``: those work in **mm** against the platform's release
    geometry, and this module is a self-contained SI port whose numbers must
    match the sim planner's to the last bit for the parity contract. Mixing the
    unit systems inside one QP assembly is exactly the class of error the
    parity fixtures could not catch.
    """
    throw_pos = np.asarray(throw_pos, dtype=float)
    target_pos = np.asarray(target_pos, dtype=float)
    return (target_pos - throw_pos) / flight_s - 0.5 * np.asarray(g, float) * flight_s


# ---------------------------------------------------------------------------
# Triple-integrator coefficient maps
# ---------------------------------------------------------------------------

_MAPS = {}
_MAPS_CAP = 32          # bounded so a long-lived node cannot grow this forever


def _integrator_maps(n_steps: int, dt: float):
    """Affine maps of pos/vel/acc knots onto ``(jerk vars, [p0, v0, a0])``.

    Returns ``(Ap, Av, Aa, Cp, Cv, Ca)`` with ``Ax`` of shape ``(n+1, n)`` (the
    jerk contribution, per axis) and ``Cx`` of shape ``(n+1, 3)`` (the initial-
    state contribution). The recursion is the sim planner's exact-polynomial
    Euler step, so ``pos = Cp @ [p0,v0,a0] + Ap @ jerk`` reproduces its rollout
    exactly.

    Cached on ``(n_steps, dt)`` — the only two things the coefficients depend on.
    (The catch knot and the detach mode change the CONSTRAINT rows, not these
    maps, so they are deliberately not part of the key.)
    """
    key = (int(n_steps), float(dt))
    hit = _MAPS.get(key)
    if hit is not None:
        return hit
    n = int(n_steps)
    Ap, Av, Aa = (np.zeros((n + 1, n)) for _ in range(3))
    Cp, Cv, Ca = (np.zeros((n + 1, 3)) for _ in range(3))
    Cp[0], Cv[0], Ca[0] = [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]
    for k in range(n):
        Ap[k + 1] = Ap[k] + Av[k] * dt + 0.5 * Aa[k] * dt ** 2
        Ap[k + 1, k] += dt ** 3 / 6.0
        Av[k + 1] = Av[k] + Aa[k] * dt
        Av[k + 1, k] += 0.5 * dt ** 2
        Aa[k + 1] = Aa[k]
        Aa[k + 1, k] += dt
        Cp[k + 1] = Cp[k] + Cv[k] * dt + 0.5 * Ca[k] * dt ** 2
        Cv[k + 1] = Cv[k] + Ca[k] * dt
        Ca[k + 1] = Ca[k]
    if len(_MAPS) >= _MAPS_CAP:
        _MAPS.clear()
    _MAPS[key] = (Ap, Av, Aa, Cp, Cv, Ca)
    return _MAPS[key]


def _axis_row(n: int, axis: int, coeffs: np.ndarray) -> np.ndarray:
    """A ``3n`` row touching only ``axis``'s jerk block (variables are
    ``[jerk_x(0..n-1), jerk_y(0..n-1), jerk_z(0..n-1)]``)."""
    row = np.zeros(3 * n)
    row[axis * n:(axis + 1) * n] = coeffs
    return row


def _perpendicular_basis(axis: np.ndarray):
    """Two orthonormal vectors spanning the plane perpendicular to ``axis``.

    ``cross(v, axis) == 0`` is rank 2; enforcing ``u1·v == 0`` and ``u2·v == 0``
    is exactly equivalent and full rank. Writing the three literal cross-product
    rows instead makes the KKT matrix singular (Phase 0, decision 1).
    """
    u = np.asarray(axis, dtype=float)
    u = u / np.linalg.norm(u)
    seed = (np.array([1.0, 0.0, 0.0]) if abs(u[0]) < 0.9
            else np.array([0.0, 1.0, 0.0]))
    u1 = np.cross(u, seed)
    u1 = u1 / np.linalg.norm(u1)
    u2 = np.cross(u, u1)
    return u1, u2


def _is_level(detach_axis) -> bool:
    """The sim planner's level test, verbatim in effect: ``None`` or world +z."""
    return (detach_axis is None
            or np.allclose(np.asarray(detach_axis, dtype=float),
                           [0.0, 0.0, 1.0], atol=1e-9))


def _as_site(value, name: str) -> np.ndarray:
    """A finite ``(3,)`` position, or a ``ValueError`` naming the field."""
    if value is None:
        raise ValueError(
            "%s is required for a window with no throw: the terminal condition "
            "is REST at a pinned site, and there is no event to read it from"
            % name)
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.shape != (3,):
        raise ValueError("%s must be a 3-vector, got shape %s"
                         % (name, np.shape(value)))
    if not np.all(np.isfinite(arr)):
        raise ValueError("%s must be finite, got %r" % (name, arr.tolist()))
    return arr


def _gate_settle_site(settle: np.ndarray, cfg) -> None:
    """Refuse a rest site outside the workspace boxes, WITH the numbers.

    The terminal position equality and the knot-``n`` workspace box rows are
    both hard, so a settle site outside the box makes the program infeasible —
    but infeasible in the worst available way: the dual step runs to infinity a
    hundred iterations deep and the operator gets ``INFEASIBLE`` with no clue
    which of thirty knobs is wrong. Same reasoning as the analytic runway gate.
    """
    xy_box = float(_cfg(cfg, 'workspace_xy_m'))
    z_min = float(_cfg(cfg, 'z_min_m'))
    z_max = float(_cfg(cfg, 'z_max_m'))
    if abs(settle[0]) > xy_box or abs(settle[1]) > xy_box:
        raise CupCycleInfeasible(
            "settle site xy (%.4f, %.4f) m is outside the +/-%.4f m workspace "
            "box the same window's knots are held inside"
            % (settle[0], settle[1], xy_box), reason='SETTLE_SITE')
    if not z_min <= settle[2] <= z_max:
        raise CupCycleInfeasible(
            "settle site z %.4f m is outside the cup box [%.4f, %.4f] m the "
            "same window's knots are held inside"
            % (settle[2], z_min, z_max), reason='SETTLE_SITE')


# ---------------------------------------------------------------------------
# QP assembly
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class _Program:
    """The assembled QP: ``min ½xᵀHx + fᵀx  s.t.  Aeq x = beq, Cᵀx ≥ bc``."""

    H: np.ndarray
    f: np.ndarray
    Aeq: np.ndarray
    beq: np.ndarray
    C: np.ndarray
    bc: np.ndarray
    n_steps: int
    catch_k: int
    takeoff_vel: np.ndarray


def _assemble(state0: CupState, throw: Optional[ThrowEvent],
              catch: Optional[CatchEvent],
              period_s: float, cfg,
              settle_site: Optional[np.ndarray] = None) -> _Program:
    """Transcribe one window into a dense convex QP.

    Decision variables are the cup's per-axis jerk over ``n_steps`` knots,
    stacked axis-major. Every term below has a one-to-one counterpart in
    ``sim.juggle_planner.plan_cup_cycle``; the only addition is the catch-runway
    inequality row.

    ``throw`` and ``catch`` are each optional (see :func:`plan_window`). The
    three switches are structural and are the whole of the generalisation:

    * **no throw** → the terminal 9 rows pin ``pos == settle_site``, ``vel == 0``
      and ``acc == 0`` instead of the release triple (site / take-off velocity /
      ``acc == g``);
    * **no catch** → the catch-position equality, the soft velocity match, the
      dwell term and the runway row are all absent, and ``catch_k`` is ``-1``;
    * **not post-release** → the detach-cone equalities are absent.

    Row ORDER is preserved exactly (terminal, then detach, then catch), and each
    block is emitted under the same expression it always was, so a window with a
    throw + a catch + a post-release start assembles the byte-identical program
    v1 assembled. That is what keeps the T-U2 parity fixtures exact.
    """
    dt = float(_cfg(cfg, 'dt'))
    n = int(round(period_s / dt))
    if n < 1:
        raise ValueError(f"period_s {period_s} is shorter than one dt ({dt})")
    Ap, Av, Aa, Cp, Cv, Ca = _integrator_maps(n, dt)

    s0 = np.array([np.asarray(state0.pos, float),
                   np.asarray(state0.vel, float),
                   np.asarray(state0.acc, float)])          # (3 state, 3 axis)
    cp, cv, ca = Cp @ s0, Cv @ s0, Ca @ s0                  # (n+1, 3 axis)
    nvar = 3 * n
    H = np.zeros((nvar, nvar))
    f = np.zeros(nvar)

    # ---- objective: mean-square acceleration (+ the lateral extra) ---------
    lat_w = float(_cfg(cfg, 'lateral_accel_weight'))
    for a in range(3):
        w = (1.0 + (lat_w if a < 2 else 0.0)) / n
        sl = slice(a * n, (a + 1) * n)
        H[sl, sl] += 2.0 * w * (Aa.T @ Aa)
        f[sl] += 2.0 * w * (Aa.T @ ca[:, a])

    n_detach = int(_cfg(cfg, 'n_detach'))
    catch_pos = None
    z_ratio = float(_cfg(cfg, 'catch_slider_vel_ratio'))
    k_td = -1

    if catch is not None:
        # ---- the catch, at the INTERPOLATED touch-down time ----------------
        catch_time_s = float(catch.t_s)
        k_td = max(0, min(int(np.floor(catch_time_s / dt)), n - 1))
        d_t = catch_time_s - k_td * dt

        # A catch inside the detach block is RANK-DEFICIENT, not merely awkward.
        # The detach equalities pin the acceleration at knots 1..n_detach, and
        # their jerk support is the leading variables 0..n_detach-1; the
        # interpolated catch-position row reaches only 0..k_td. With
        # k_td <= n_detach those n_detach + 1 equality rows (per axis, on the
        # level path) live on a support too small to carry them, the KKT matrix
        # is singular, and the solve is meaningless. Refuse here with the numbers
        # rather than let it surface as a singular working set deep inside the
        # solver — the same reasoning as the analytic runway gate.
        #
        # With NO detach rows (``post_release`` False) that argument is empty, but
        # the floor does not drop to zero: at ``k_td == 0`` the catch-position row
        # is ``(d_t³/6)·e_0`` alone, which vanishes identically when the catch
        # lands exactly on knot 0 and would then be a zero row divided by a zero
        # norm in the scaling below. So the floor is ``k_td >= 1`` there.
        if state0.post_release and k_td <= n_detach:
            raise CupCycleInfeasible(
                "catch at t=%.3fs falls inside the %d-knot detach block — the "
                "catch-position equality is rank-deficient against the detach "
                "rows" % (catch_time_s, n_detach), reason='CATCH_TOO_EARLY')
        if not state0.post_release and k_td <= 0:
            raise CupCycleInfeasible(
                "catch at t=%.3fs falls on the window's first knot — the "
                "catch-position equality has no jerk support there"
                % catch_time_s, reason='CATCH_TOO_EARLY')

        e_td = np.zeros(n)
        e_td[k_td] = 1.0
        r_ptd = (Ap[k_td] + Av[k_td] * d_t + 0.5 * Aa[k_td] * d_t ** 2
                 + (d_t ** 3 / 6.0) * e_td)
        c_ptd = cp[k_td] + cv[k_td] * d_t + 0.5 * ca[k_td] * d_t ** 2
        r_vtd = Av[k_td] + Aa[k_td] * d_t + 0.5 * d_t ** 2 * e_td
        c_vtd = cv[k_td] + ca[k_td] * d_t

        # Velocity-matched catch, split by morphology axis (soft, as in the sim).
        catch_vel = np.asarray(catch.vel, dtype=float)
        lat_ratio = float(_cfg(cfg, 'catch_vel_ratio'))
        v_target = np.array([lat_ratio * catch_vel[0], lat_ratio * catch_vel[1],
                             z_ratio * catch_vel[2]])
        v_weights = [float(_cfg(cfg, 'catch_vel_weight'))] * 2 \
            + [float(_cfg(cfg, 'catch_slider_vel_weight'))]
        for a in range(3):
            sl = slice(a * n, (a + 1) * n)
            H[sl, sl] += 2.0 * v_weights[a] * np.outer(r_vtd, r_vtd)
            f[sl] += 2.0 * v_weights[a] * (c_vtd[a] - v_target[a]) * r_vtd

        # Lateral DWELL at the catch (opt-in; disabled by default in both planners).
        dwell_w = float(_cfg(cfg, 'catch_dwell_weight'))
        dwell_pre = int(_cfg(cfg, 'catch_dwell_pre'))
        dwell_post = int(_cfg(cfg, 'catch_dwell_post'))
        catch_pos = np.asarray(catch.site, dtype=float)
        if dwell_w > 0.0 and (dwell_pre or dwell_post):
            for k in range(max(0, k_td - dwell_pre),
                           min(n + 1, k_td + dwell_post + 1)):
                for a in range(2):
                    sl = slice(a * n, (a + 1) * n)
                    H[sl, sl] += 2.0 * dwell_w * np.outer(Ap[k], Ap[k])
                    f[sl] += 2.0 * dwell_w * (cp[k, a] - catch_pos[a]) * Ap[k]

    # Throw COAST (opt-in; soft pull of the post-throw vertical accel to 0).
    # Gated on ``post_release`` because the accel it pulls to zero is the one
    # left over from the release the window FOLLOWS — on a launch-from-rest
    # window there is no such release and the term would just bias the first
    # knots of an ordinary acceleration profile toward zero. Both planners ship
    # it off (``throw_coast_weight = 0``), so no existing solve is affected.
    coast_knots = int(_cfg(cfg, 'throw_coast_knots'))
    coast_w = float(_cfg(cfg, 'throw_coast_weight'))
    if state0.post_release and coast_knots > 0 and coast_w > 0.0:
        sl = slice(2 * n, 3 * n)
        for i in range(1, min(coast_knots, n) + 1):
            H[sl, sl] += 2.0 * coast_w * np.outer(Aa[i], Aa[i])
            f[sl] += 2.0 * coast_w * ca[i, 2] * Aa[i]

    # ---- hard equalities ---------------------------------------------------
    rows, rhs = [], []
    if throw is not None:
        throw_pos = np.asarray(throw.site, dtype=float)
        v_takeoff = takeoff_velocity(throw_pos, np.asarray(throw.target, float),
                                     float(throw.flight_s))
        for a in range(3):                   # throw: pos, vel, acc == g
            rows.append(_axis_row(n, a, Ap[n])); rhs.append(throw_pos[a] - cp[n, a])
            rows.append(_axis_row(n, a, Av[n])); rhs.append(v_takeoff[a] - cv[n, a])
            rows.append(_axis_row(n, a, Aa[n])); rhs.append(GRAVITY[a] - ca[n, a])
    else:
        # Terminal REST. The three rows per axis are the same three rows in the
        # same order — position, velocity, acceleration — so a caller reading
        # ``Aeq`` by index sees one shape, not two. ``acc == 0`` (not ``g``) is
        # the point: at rest the cup is HELD, and the ball, if any, stays seated.
        v_takeoff = np.zeros(3)
        settle = _as_site(settle_site, 'settle_site')
        _gate_settle_site(settle, cfg)
        for a in range(3):
            rows.append(_axis_row(n, a, Ap[n])); rhs.append(settle[a] - cp[n, a])
            rows.append(_axis_row(n, a, Av[n])); rhs.append(0.0 - cv[n, a])
            rows.append(_axis_row(n, a, Aa[n])); rhs.append(0.0 - ca[n, a])

    if state0.post_release:
        if _is_level(state0.detach_axis):
            for i in range(1, n_detach + 1):     # level: acc_xy == 0
                for a in range(2):
                    rows.append(_axis_row(n, a, Aa[i]))
                    rhs.append(0.0 - ca[i, a])
        else:
            u1, u2 = _perpendicular_basis(state0.detach_axis)
            for i in range(1, n_detach + 1):     # tilted: (acc − g) ∥ axis, rank 2
                for u in (u1, u2):
                    row = np.zeros(nvar)
                    b = 0.0
                    for a in range(3):
                        row[a * n:(a + 1) * n] = u[a] * Aa[i]
                        b -= u[a] * (ca[i, a] - GRAVITY[a])
                    rows.append(row); rhs.append(b)

    if catch is not None:
        for a in range(3):                   # interpolated catch position
            rows.append(_axis_row(n, a, r_ptd))
            rhs.append(catch_pos[a] - c_ptd[a])

    Aeq = np.array(rows)
    beq = np.array(rhs)
    scale = np.linalg.norm(Aeq, axis=1)      # 2-norm row scaling; x unchanged
    Aeq = Aeq / scale[:, None]
    beq = beq / scale

    # ---- inequalities: jerk boxes, workspace boxes, catch runway -----------
    # Column order is preserved from the Phase 0 reference implementation (all
    # upper sides, then all lower sides, then the one-sided runway) because the
    # G-I most-violated selection breaks ties by index.
    box_rows, box_const, box_lo, box_hi = [], [], [], []
    jerk_xy = float(_cfg(cfg, 'max_jerk_xy'))
    jerk_z = float(_cfg(cfg, 'max_jerk_z'))
    for a in range(3):
        lim = jerk_z if a == 2 else jerk_xy
        for k in range(n):
            row = np.zeros(nvar)
            row[a * n + k] = 1.0
            box_rows.append(row); box_const.append(0.0)
            box_lo.append(-lim); box_hi.append(lim)
    xy_box = float(_cfg(cfg, 'workspace_xy_m'))
    z_min = float(_cfg(cfg, 'z_min_m'))
    z_max = float(_cfg(cfg, 'z_max_m'))
    # Knot 0 is the caller's ``pos0`` — a constant, not a variable, so bounding
    # it can only be vacuous or unsatisfiable. The sim planner writes the
    # constraint anyway (CasADi folds it away); here it is skipped, which is the
    # one structural difference and is invisible for any pos0 inside the box.
    for k in range(1, n + 1):
        for a in range(3):
            lo, hi = (z_min, z_max) if a == 2 else (-xy_box, xy_box)
            box_rows.append(_axis_row(n, a, Ap[k]))
            box_const.append(cp[k, a])
            box_lo.append(lo); box_hi.append(hi)
    BR = np.array(box_rows)
    BC = np.array(box_const)
    BLO = np.array(box_lo)
    BHI = np.array(box_hi)
    cols = [-BR.T, BR.T]
    rhss = [BC - BHI, BLO - BC]

    if catch is not None and _cfg(cfg, 'catch_runway_enabled'):
        floor_m, need_m = catch_runway_requirement(catch_vel, cfg)
        # The exact analytic gate: the catch-position equality pins the cup's
        # touch-down height to catch_pos[2], so the runway there is decided by
        # the REQUEST, not by the solver. Refuse with numbers rather than as a
        # dual-unbounded step 200 iterations deep.
        headroom = float(catch_pos[2]) - floor_m
        if headroom < need_m - 1e-12:
            raise CupCycleInfeasible(
                "catch runway: touch-down at z=%.4f m leaves %.1f mm above the "
                "slider floor z=%.4f m, but %.1f mm is needed to decelerate "
                "%.3f m/s at %.1f m/s^2 (incl. %.1f mm margin)"
                % (catch_pos[2], headroom * 1e3, floor_m, need_m * 1e3,
                   abs(z_ratio * catch_vel[2]),
                   float(_cfg(cfg, 'catch_runway_decel_mps2')),
                   float(_cfg(cfg, 'catch_runway_margin_m')) * 1e3),
                reason='CATCH_RUNWAY')
        # The shaping half: the same bound on the catch KNOT's height, which is
        # a free variable — EXCEPT at k_td == 0, where that knot IS the caller's
        # ``pos0``. ``Ap[0]`` is identically zero, so the row is a constant that
        # can only be vacuous or unsatisfiable, exactly as for the workspace box
        # skipped at k = 0 above (and it would additionally divide by a zero
        # column norm in the scaling below). The analytic gate just above already
        # covers the requirement there exactly, because the catch-position
        # equality has pinned that height anyway. The detach gate above already
        # refuses every ``k_td == 0`` for any ``n_detach >= 0``, so this is a
        # structural guard on the assembly rather than a reachable branch — it is
        # what keeps the column scaling below safe if that gate is ever relaxed.
        if k_td > 0:
            cols.append(_axis_row(n, 2, Ap[k_td])[:, None])
            rhss.append(np.array([floor_m + need_m - cp[k_td, 2]]))

    # ---- optional acceleration boxes (default OFF; see CupCycleConfig) ------
    # Appended as their OWN block after every pre-existing column so that with the
    # boxes off not one column index moves — which is what makes the T-U1/T-U2
    # parity fixtures bit-identical, and what keeps a warm start's active-set
    # indices meaning the same constraint (the same discipline the runway row
    # follows). With them on the shape changes, and ``plan_window``'s warm-start
    # key already carries ``C.shape[1]``, so a stale set cannot be misapplied.
    acc_z = _cfg(cfg, 'max_acc_z')
    acc_xy = _cfg(cfg, 'max_acc_xy')
    if acc_z is not None or acc_xy is not None:
        g_z = abs(float(GRAVITY[2]))
        if acc_z is not None and float(acc_z) < g_z:
            raise CupCycleInfeasible(
                "max_acc_z %.3f m/s^2 is below |g| = %.3f: the release equality "
                "pins the cup's acceleration to g exactly, so no trajectory can "
                "satisfy both. Raise the box or drop it."
                % (float(acc_z), g_z), reason='ACC_BOX')
        arows, aconst, alo, ahi = [], [], [], []
        for k in range(1, n + 1):
            for a in range(3):
                lim = acc_z if a == 2 else acc_xy
                if lim is None:
                    continue
                arows.append(_axis_row(n, a, Aa[k]))
                aconst.append(ca[k, a])
                alo.append(-float(lim))
                ahi.append(float(lim))
        AR = np.array(arows)
        AC = np.array(aconst)
        cols.extend([-AR.T, AR.T])
        rhss.extend([AC - np.array(ahi), np.array(alo) - AC])

    C = np.hstack(cols)
    bc = np.concatenate(rhss)
    cscale = np.linalg.norm(C, axis=0)       # unit columns: violations compare
    C = C / cscale
    bc = bc / cscale

    return _Program(H=H, f=f, Aeq=Aeq, beq=beq, C=C, bc=bc, n_steps=n,
                    catch_k=k_td, takeoff_vel=v_takeoff)


def catch_runway_requirement(catch_vel, cfg):
    """``(z_floor_m, required_runway_m)`` for the catch-runway constraint.

    ``required = (catch_slider_vel_ratio · |v_ball,z|)² / (2 · a) + margin`` —
    the TARGET catch speed, so the bound is a constant and the program stays a
    convex QP. WP3's ``validate_cycle`` re-checks post hoc with the ACHIEVED
    velocity, which the soft vertical match only approaches.

    Exposed publicly because the tests, WP3 and any caller sizing a catch height
    must all use one expression of the requirement.
    """
    v_tgt = abs(float(_cfg(cfg, 'catch_slider_vel_ratio'))
                * float(np.asarray(catch_vel, dtype=float)[2]))
    decel = float(_cfg(cfg, 'catch_runway_decel_mps2'))
    if decel <= 0.0:
        raise ValueError("catch_runway_decel_mps2 must be > 0")
    floor = _cfg(cfg, 'catch_runway_z_floor_m')
    floor_m = float(_cfg(cfg, 'z_min_m')) if floor is None else float(floor)
    need_m = v_tgt * v_tgt / (2.0 * decel) + float(
        _cfg(cfg, 'catch_runway_margin_m'))
    return floor_m, need_m


# ---------------------------------------------------------------------------
# Goldfarb–Idnani dual active set
# ---------------------------------------------------------------------------

def _verify(prog: _Program, x: np.ndarray, feas_tol: float, active, iters: int):
    """Refuse ``x`` unless it actually satisfies the program.

    NOT belt-and-braces. Measured on this problem family (2026-08-30): a 0.40 s
    cycle that **IPOPT also refuses** drove the working set to 38 of 48 rows,
    ``M = NᵀH⁻¹N`` became numerically singular, ``np.linalg.solve`` returned a
    finite ``r`` anyway, and the search direction stopped lying in the null space
    of the equality rows. The loop then terminated "successfully" with **every
    inequality satisfied** and an equality residual of **121** — a cup
    trajectory 121 units off the constraint manifold, returned without a word.
    Inequality feasibility alone does not detect this; the equality residual is
    the only witness.

    So every solve is verified before it is returned, and a point that fails is
    a refusal. A genuinely infeasible request and a numerically degenerate one
    are not distinguished here, deliberately: from the caller's seat both mean
    "no trustworthy plan for this cycle", and Phase 4 handles a refusal by
    holding the last good plan. Silently returning the point would be the
    failure mode Phase 0 was run to prevent.

    Which refusal path such a case takes is NOT stable: that same cycle refuses
    through the unbounded dual step under the venv interpreter and through this
    gate under ``/usr/bin/python3`` — identical Python 3.8.10 and numpy 1.24.4,
    differing only in BLAS. Both are refusals, which is all that is contracted;
    no caller should branch on which one fired.
    """
    eq_res = float(np.abs(prog.Aeq @ x - prog.beq).max()) if prog.Aeq.size else 0.0
    ineq_res = float(-(prog.C.T @ x - prog.bc).min()) if prog.C.size else 0.0
    if not np.all(np.isfinite(x)) or eq_res > feas_tol or ineq_res > feas_tol:
        raise CupCycleInfeasible(
            "QP solution failed verification (equality residual %.3e, worst "
            "constraint violation %.3e, bar %.1e) after %d iterations with a "
            "working set of %d — refused rather than returned"
            % (eq_res, ineq_res, feas_tol, iters, len(active)),
            reason='UNVERIFIED')


def _solve_qp(prog: _Program, warm: Optional[SolverState], max_iter: int,
              tol: float, feas_tol: float):
    """Minimise ``½xᵀHx + fᵀx`` s.t. ``Aeq x = beq`` and ``Cᵀx ≥ bc``.

    A Goldfarb–Idnani dual active set (Phase 0 decision 1 binds this module to
    it). Start from the equality-constrained KKT solution — dual feasible with
    an empty inequality working set — then repeatedly pick a violated
    inequality and drive it to feasibility, dropping blocking constraints on
    partial steps. The dual objective increases monotonically, so no working set
    repeats and termination is finite.

    ``warm`` biases only the *choice* of which violated constraint to admit
    next (previously-active ones first). Any violated constraint is a legal
    choice, so the optimum is unchanged and the finite-termination argument is
    untouched — a warm start here buys iterations, never a different answer.

    Raises :class:`CupCycleInfeasible` on an unbounded dual step (genuine
    infeasibility), on iteration exhaustion, or when the converged point fails
    :func:`_verify` — never returns an unverified x, because a silently
    infeasible cup trajectory is the failure mode Phase 0 was run to prevent.
    """
    H, f, Aeq, beq, C, bc = (prog.H, prog.f, prog.Aeq, prog.beq, prog.C, prog.bc)
    nvar, n_eq = H.shape[0], Aeq.shape[0]
    H_inv = np.linalg.inv(H)
    kkt = np.zeros((nvar + n_eq, nvar + n_eq))
    kkt[:nvar, :nvar] = H
    kkt[:nvar, nvar:] = Aeq.T
    kkt[nvar:, :nvar] = Aeq
    try:
        z0 = np.linalg.solve(kkt, np.concatenate([-f, beq]))
    except np.linalg.LinAlgError as exc:
        raise CupCycleInfeasible(
            "QP infeasible: the equality-constrained KKT system is singular "
            "(%s) — the %d equality rows are rank-deficient for this cycle, so "
            "there is no starting point to iterate from" % (exc, n_eq),
            reason='SINGULAR')
    x = z0[:nvar]
    u = -z0[nvar:]                       # multipliers: Hx + f − N u = 0

    n_mat = Aeq.T.copy()                 # working-set normals (eq rows first)
    hn_mat = H_inv @ Aeq.T
    active = []
    preferred = [j for j in (warm.active if warm is not None else ())
                 if 0 <= j < C.shape[1]]
    iters = 0
    while iters < max_iter:
        viol = C.T @ x - bc
        p = -1
        for j in preferred:              # warm-start bias (see docstring)
            if viol[j] < -tol and j not in active:
                p = j
                break
        if p < 0:
            p = int(np.argmin(viol))
        if viol[p] > -tol:
            _verify(prog, x, feas_tol, active, iters)
            return x, tuple(active), iters
        n_p = C[:, p]
        hn_p = H_inv @ n_p
        u_p = 0.0
        while iters < max_iter:          # dual iteration on this constraint
            iters += 1
            m_mat = n_mat.T @ hn_mat
            try:
                r = np.linalg.solve(m_mat, n_mat.T @ hn_p)
            except np.linalg.LinAlgError as exc:
                raise CupCycleInfeasible(
                    "QP infeasible: the working-set system NᵀH⁻¹N went singular "
                    "(%s) while admitting inequality %d at working-set size %d "
                    "— refused rather than solved through a degenerate set"
                    % (exc, p, len(active)), reason='SINGULAR')
            z_dir = hn_p - hn_mat @ r
            nz = float(n_p @ z_dir)
            slack = float(n_p @ x) - bc[p]
            t_full = (-slack) / nz if nz > 1e-14 else np.inf
            blockers = [(u[n_eq + j] / r[n_eq + j], j) for j in range(len(active))
                        if r[n_eq + j] > 1e-12]
            t_drop, blk = min(blockers) if blockers else (np.inf, -1)
            step = min(t_drop, t_full)
            if not np.isfinite(step):
                raise CupCycleInfeasible(
                    "QP infeasible: unbounded dual step admitting inequality "
                    "%d (working set size %d)" % (p, len(active)))
            if np.isfinite(t_full):
                x = x + step * z_dir
            u = u - step * r
            u_p += step
            if t_full <= t_drop:         # full step: p joins the working set
                n_mat = np.column_stack([n_mat, n_p])
                hn_mat = np.column_stack([hn_mat, hn_p])
                u = np.append(u, u_p)
                active.append(p)
                break
            u = np.delete(u, n_eq + blk)  # partial step: drop the blocker
            n_mat = np.delete(n_mat, n_eq + blk, axis=1)
            hn_mat = np.delete(hn_mat, n_eq + blk, axis=1)
            active.pop(blk)
    raise CupCycleInfeasible(
        "QP did not converge in %d iterations (working set size %d) — the "
        "solution is NOT verified feasible and is deliberately not returned"
        % (max_iter, len(active)))


# ---------------------------------------------------------------------------
# Public entry points
# ---------------------------------------------------------------------------

def plan_window(events: Sequence, state0: CupState,
                cfg: Optional[CupCycleConfig] = None, *,
                period_s: Optional[float] = None,
                settle_site: Optional[np.ndarray] = None,
                warm_start: Optional[SolverState] = None) -> CupCyclePlan:
    """Plan the cup over one horizon window from an ordered event timeline.

    Parameters
    ----------
    events
        Ordered (non-decreasing ``t_s``) :class:`ThrowEvent` /
        :class:`CatchEvent` objects, times measured from the window start. **At
        most one throw and at most one catch** — every combination of 0-or-1 of
        each is accepted, and those four combinations are the four window kinds
        a session needs (see below). More than one of either still raises
        ``NotImplementedError``: the timeline shape is already the 3-ball
        interface, only the solver's constraint generation is not, and a silent
        single-ball approximation of a 3-ball request would be far worse than a
        refusal.
    state0
        Cup boundary condition at the window start, including the detach axis of
        the ball released just before it and — since the four-kind
        generalisation — ``post_release``, which says whether there WAS such a
        release. See :class:`CupState`.
    cfg
        :class:`CupCycleConfig` (or a ``sim.juggle_planner.PlannerConfig``, for
        which the runway constraint reads as disabled — see ``_RUNWAY_DEFAULTS``).
    period_s
        Window length (s). Defaults to the throw event's ``t_s``, and is
        therefore **required when there is no throw** — with no terminal release
        nothing else in the timeline knows where the window ends.
    settle_site
        Cup position (m) the window comes to REST at. Required when there is no
        throw, ignored when there is one.
    warm_start
        The previous cycle's :attr:`CupCyclePlan.warm_start`. Ignored unless the
        problem shape matches. NB the shape key fingerprints the INEQUALITY
        structure only, so a warm start can cross between window kinds with the
        same knot count and the same inequality block — which is harmless by
        construction: a warm start biases only *which* violated constraint is
        admitted next, so it buys iterations and can never change the answer
        (see :func:`_solve_qp`).

    The four window kinds, and what each one is for
    ----------------------------------------------
    ==================  ======  ======  =============  =========================
    kind                throw   catch   ``post_release``  terminal condition
    ==================  ======  ======  =============  =========================
    launch (from rest)    1       0        False        release
    steady                1       1        True         release
    landing               0       1        True         rest at ``settle_site``
    settle                0       0        True         rest at ``settle_site``
    ==================  ======  ======  =============  =========================

    A session is a CHAIN of these: windows abut at each release instant, where
    the terminal state of one is exactly the ``state0`` of the next (position =
    throw site, velocity = take-off, acceleration = ``g``, ``detach_axis`` = the
    throw tilt's cup axis, ``post_release`` = True). v1 expressed only the
    ``steady`` row, which is why a launch had to be faked as a steady cycle whose
    "previous release" never happened — and that fake is not benign: it pins the
    first two knots' acceleration direction against a detach axis for a ball that
    is not there.

    Returns a :class:`CupCyclePlan`. Raises :class:`CupCycleInfeasible`
    (a ``RuntimeError``) when no feasible trajectory exists — the same
    raise-on-infeasible contract as ``plan_cup_cycle``. See
    :class:`CupCyclePlan` for the ``catch_k == -1`` / ``takeoff_vel == 0``
    sentinels a window without the corresponding event returns.
    """
    if cfg is None:
        cfg = CupCycleConfig()
    throws = [e for e in events if isinstance(e, ThrowEvent)]
    catches = [e for e in events if isinstance(e, CatchEvent)]
    if len(throws) + len(catches) != len(events):
        raise TypeError("events must be ThrowEvent / CatchEvent instances")
    if len(throws) > 1 or len(catches) > 1:
        raise NotImplementedError(
            "plan_window handles at most one throw and one catch per window "
            "(got %d throws, %d catches)" % (len(throws), len(catches)))
    times = [float(e.t_s) for e in events]
    if any(b < a for a, b in zip(times, times[1:])):
        raise ValueError("events must be ordered by non-decreasing t_s")

    throw = throws[0] if throws else None
    catch = catches[0] if catches else None
    if period_s is not None:
        window_s = float(period_s)
    elif throw is not None:
        window_s = float(throw.t_s)
    else:
        raise ValueError(
            "period_s is required for a window with no throw: the window length "
            "is otherwise read off the terminal release, and there isn't one")
    if window_s <= 0.0:
        raise ValueError("window length must be > 0 (got %r)" % (window_s,))
    if catch is not None and not 0.0 <= float(catch.t_s) < window_s:
        raise ValueError(
            "catch at t=%.4f s is outside the window [0, %.4f)"
            % (catch.t_s, window_s))

    prog = _assemble(state0, throw, catch, window_s, cfg,
                     settle_site=settle_site)
    dt = float(_cfg(cfg, 'dt'))
    n = prog.n_steps
    key = (n, dt, prog.C.shape[1])
    warm = warm_start if (warm_start is not None
                          and warm_start.key == key) else None
    x, active, iters = _solve_qp(prog, warm, int(_cfg(cfg, 'max_iter')),
                                 float(_cfg(cfg, 'tol')),
                                 float(_cfg(cfg, 'feas_tol')))

    jerk = x.reshape(3, n).T
    Ap, Av, Aa, Cp, Cv, Ca = _integrator_maps(n, dt)
    s0 = np.array([np.asarray(state0.pos, float),
                   np.asarray(state0.vel, float),
                   np.asarray(state0.acc, float)])
    return CupCyclePlan(
        pos=Cp @ s0 + Ap @ jerk,
        vel=Cv @ s0 + Av @ jerk,
        acc=Ca @ s0 + Aa @ jerk,
        jerk=jerk,
        t=np.arange(n + 1) * dt,
        dt=dt,
        catch_k=prog.catch_k,
        takeoff_vel=prog.takeoff_vel,
        warm_start=SolverState(active=active, key=key, iters=iters))


def plan_cup_cycle(pos0, vel0, acc0,
                   throw_pos, throw_target, flight_s,
                   catch_time_s, catch_pos, catch_vel,
                   period_s,
                   cfg=None, detach_axis=None,
                   warm_start: Optional[SolverState] = None) -> CupCyclePlan:
    """``sim.juggle_planner.plan_cup_cycle``'s signature, solved by the QP.

    A thin wrapper over :func:`plan_window` so the WP4 sim-parity harness can
    drive either planner through one call site. Argument names, order and
    meaning are the sim function's; ``warm_start`` is the only addition and is
    keyword-with-default, so a positional call written against the sim planner
    works unchanged.

    With ``cfg.catch_runway_enabled`` False (and always when handed a sim
    ``PlannerConfig``) this reproduces the sim planner's constraint set exactly;
    with it True the catch-runway bound is added, so the two can differ — that
    is the intended difference, not a parity failure.
    """
    events = [
        CatchEvent(ball_id=0, t_s=float(catch_time_s),
                   site=np.asarray(catch_pos, dtype=float),
                   vel=np.asarray(catch_vel, dtype=float)),
        ThrowEvent(ball_id=1, t_s=float(period_s),
                   site=np.asarray(throw_pos, dtype=float),
                   target=np.asarray(throw_target, dtype=float),
                   flight_s=float(flight_s)),
    ]
    state0 = CupState(pos=np.asarray(pos0, dtype=float),
                      vel=np.asarray(vel0, dtype=float),
                      acc=np.asarray(acc0, dtype=float),
                      detach_axis=(None if detach_axis is None
                                   else np.asarray(detach_axis, dtype=float)))
    return plan_window(events, state0, cfg, period_s=float(period_s),
                       warm_start=warm_start)
