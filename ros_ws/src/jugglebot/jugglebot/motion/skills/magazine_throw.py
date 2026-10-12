"""The MAGAZINE throw — one formulaic, heave-assisted, rest-terminal segment.

Ball Butler's magazine sits ~2.4 m up and ~1.4 m away, further than any QP
window this stack plans (``segments.THROW`` tops out near the hand's own stroke
speed).  This module builds that one throw WITHOUT the cup QP: every channel of
the 7-channel :class:`~jugglebot.motion.trajectory.cycle_plan.CyclePlan` is a
closed-form **piecewise-LINEAR acceleration on the 25 ms knot grid**, i.e. a
C2 cubic spline.  That is exactly what the streamed lane can represent (the
emitter interpolates each span as a cubic Hermite from knot ``(p, v)``, which
reproduces a linear-acceleration cubic exactly) and exactly what
``feasibility.validate_cycle`` demands of the hand (``HAND_LIMIT_C2``).  The
gate is NOT bypassed: the finished plan is run through ``validate_cycle`` at the
limits the caller passed, and any refusal is raised as the same
:class:`~jugglebot.motion.unified_cycle.CycleInfeasible` the QP path raises, so
``executor.install_segment`` reports it unchanged.  Nothing here raises a
limit, a ceiling or a safety constant.

THE SHAPE (rest -> throw -> rest, one ball, sensor-confirmed in the cup)
-----------------------------------------------------------------------
0. WAIT — hold the seed rest so the release lands on the requested instant.
1. PRE — sine-acceleration rest-to-rest moves of every channel to the
   stroke-start state: the platform tilts to the throw attitude and offsets so
   the release lands ON the requested site; the hand drops to the stroke
   bottom (the ball stays seated: the cup's downward acceleration is far above
   the C-CUP-2 contact floor, which the gate checks over ``contact_knots``).
2. HEAVE — the platform accelerates (trapezoid, ``plat_ramp_spans`` ramps) to
   its release velocity and CRUISES through the release span.  Its z component
   is the heave ASSIST, and it is zero unless the hand alone cannot reach.
3. HAND — the deleted Teensy ``calcThrow`` shape on the grid: knots
   ``[0, a1, A..A | -D..-D, -delta, 0]``, A and D AT their caps, one
   transition span ``+A -> -D`` that contains the release, and the two free
   knot values ``a1`` / ``delta`` absorbing the grid quantisation so the
   hand-relative speed at release is exactly what the aim needs and the hand
   stops exactly.
4. RELEASE — inside the transition span, at the instant the cup's AXIAL
   acceleration reaches the ball's free-fall acceleration along the axis
   (``-g * a_z``, rigid-body cup).  The platform cruises there, so the
   instant is closed-form.
5. POST — after the post-release hold, the tilt returns to level, the empty
   hand returns to the rest site's park height, the platform returns to the
   rest site at the active z.  The plan ends at REST.

LEAST AGGRESSIVE, IN THIS ORDER
-------------------------------
* **Tilt**: the cup axis is the ballistic take-off direction
  (``tilt_to_throw``) whenever that is inside ``MAX_TILT_DEG`` — the platform
  then carries only the small lateral residual the repo cup map needs.  Past
  the ceiling the axis is CAPPED at it, in the take-off's vertical plane, and
  the platform's release velocity leans out to supply the lateral excess (the
  prototype's ``tilt_cap_deg`` mode) — the smallest tilt that still aims it.
  When that lateral excess alone exceeds the platform-speed budget
  (:attr:`MagazineConfig.max_platform_mm_s`) the throw is REFUSED
  (:data:`MAGAZINE_TILT`) — the tilt would have to exceed the ceiling, and a
  saturated tilt lands the ball somewhere it was not aimed.
* **Hand first**: the hand supplies the take-off speed along the cup axis up
  to its cap — the largest hand-relative release speed whose full stroke
  (bottom -> release -> stop at the decel cap) stays under
  ``HAND_STROKE_MAX_REV`` and under the session hand-velocity limit.
* **Platform the remainder only**: the platform's release velocity is
  ``V - a * hand`` — exactly zero while the hand alone suffices and the aim
  is inside the ceiling; past the hand cap it moves ALONG the axis (the heave
  assist), past the tilt ceiling it also leans out.

THE AIM USES RIGID-BODY CUP KINEMATICS (main-session adjudication 2026-10-13)
--------------------------------------------------------------------------
The platform is rigid and the IK rotates it about the platform centre
``pos + (0, 0, GEOM_INITIAL_HEIGHT_MM)`` (``motion/ik_solver.py``
``compute_leg_vectors``: ``platform_centre = pos + geom.init_height_vec``), whose
world z at the active pose is ``578.2 + 170.0 = 748.2`` mm =
``CUP_TILT_CENTER_Z_MM`` (``trajectory/shaping.py``, derived as exactly that
sum).  So the rigid cup opening is ``centre(pose) + a * L(slider)`` with the
centre RIDING with the heave, and the release site, velocity and instant
announced in the meta are the rigid ones.  The hand acts along the PHYSICAL
cup axis ``a``, which is the GRAVITY-frame aim; the plan's commanded tilts are
that aim in the PLAN frame (``unified_cycle._tilts_to_plan`` from the seed's
``CycleState.levelling_correction`` — the same conversion ``_realize`` applies,
contract C-LEVEL-1/2 row E8; knot 0 keeps the seed's own float).

The repo's ``unified_cycle.cup_state_from_platform`` (the inverse of
``cup_realize.decompose``) holds the tilt centre FIXED at that height — exact
at the active z, but under real heave at a tilt it reads platform z travel as
slider travel along the axis, ~``a_xy * u_z`` of lateral velocity off.
:class:`MagazineDesign` reports ``repo map minus rigid`` at the release
(``repo_minus_rigid_*``) for every plan, so the gap is visible, not designed
against.  ``validate_cycle`` is unchanged: it gates the pose+hand knots, which
are what streams.
"""

from __future__ import annotations

import dataclasses
import math
import time
from typing import Optional, Tuple

import numpy as np

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.trajectory import ballistics_bc
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import feasibility as fz
from jugglebot.motion.trajectory import throw_envelope as te
from jugglebot.motion.trajectory import tilt_geometry as tg
from jugglebot.motion.trajectory.cycle_plan import CyclePlan

#: The skill kind.  Defined HERE, not in ``segments.KINDS``: ``segments.py`` is
#: one of the admissible-box gated files, and this kind never enters a box.
MAGAZINE = 'MAGAZINE'

# ── refusal codes (raised as uc.CycleInfeasible(code, reasons)) ─────────────
MAGAZINE_NOT_AT_REST = 'MAGAZINE_NOT_AT_REST'   #: the seed is moving / post-release
MAGAZINE_UNREACHABLE = 'MAGAZINE_UNREACHABLE'   #: hand cap + platform budget cannot reach
MAGAZINE_TILT = 'MAGAZINE_TILT'                 #: aiming needs > MAX_TILT_DEG of tilt
MAGAZINE_WINDOW = 'MAGAZINE_WINDOW'             #: release requested before the shape fits
#: ``executor.install_segment``'s refusal for a MAGAZINE asked to splice onto a
#: moving plan — this throw is only ever planned from a stopped machine.
MAGAZINE_NEEDS_REST = 'MAGAZINE_NEEDS_REST'

G_MM_S2 = float(ballistics_bc.GRAVITY_MMS2)
MM_PER_REV = 1000.0 / float(hw.HAND_REV_PER_M)
_C_Z = float(tg.CUP_TILT_CENTER_Z_MM)
_REST_TOL = 1e-6          #: |velocity| below which a seed counts as stopped


@dataclasses.dataclass(frozen=True)
class MagazineTerminal:
    """The magazine throw's goal, in the PLAN frame (mm).

    ``t_release_s`` is the requested release instant — ABSOLUTE when handed to
    ``executor.install_segment`` (like ``ThrowTerminal.t_release_s``), segment-
    relative by the time :func:`plan_magazine` sees it.  The plan releases at
    the nearest knot-quantised instant at or after the earliest the shape
    allows; the actual instant is ``Segment.event_t_s``.
    """

    target_mm: np.ndarray
    flight_s: float
    t_release_s: float
    release_site_mm: np.ndarray
    rest_site_mm: np.ndarray

    def __post_init__(self):
        for name in ('target_mm', 'release_site_mm', 'rest_site_mm'):
            object.__setattr__(self, name, sg._vec3(getattr(self, name), name))
        if not float(self.flight_s) > 0.0:
            raise ValueError('flight_s must be > 0, got %r' % (self.flight_s,))
        if not float(self.t_release_s) > 0.0:
            raise ValueError('t_release_s must be > 0, got %r'
                             % (self.t_release_s,))


@dataclasses.dataclass(frozen=True)
class MagazineConfig:
    """Shape knobs.  None of these is a limit — the gate holds the limits."""

    #: Hand acceleration with the ball aboard (rev/s^2); ``None`` = the drive's
    #: ascent authority (``throw_envelope.ACCEL_AUTHORITY_RPS2``), never more
    #: than the session hand limit less :attr:`hand_acc_margin_rps2`.
    hand_acc_rps2: Optional[float] = None
    #: Kept off the session hand-acc limit so the Hermite round trip cannot
    #: read ``limit + 1 ulp`` (``HAND_LIMIT_ACC``).
    hand_acc_margin_rps2: float = 0.1
    hand_bottom_min_rev: float = 0.15     #: lowest stroke start (ball seated)
    hand_top_margin_rev: float = 0.03     #: kept under HAND_STROKE_MAX_REV
    plat_acc_mmps2: float = 4500.0        #: platform heave acceleration cap
    plat_ramp_spans: int = 2
    #: Platform speed budget at release (|lateral lean + heave|), mm/s.  The
    #: legs' session limits still gate it; this only bounds the search.
    max_platform_mm_s: float = 2000.0
    pre_s: float = 0.80
    gap_s: float = 0.05
    tilt_return_s: float = 0.50
    hand_return_s: float = 0.40
    plat_return_s: float = 0.70
    tail_s: float = 0.05
    max_tilt_deg: float = float(tg.MAX_TILT_DEG)


@dataclasses.dataclass(frozen=True)
class MagazineDesign:
    """The built plan plus the numbers that chose it (diagnostics + tests)."""

    plan: CyclePlan
    t_release_s: float
    release_site_mm: np.ndarray        #: rigid-body cup, read off the plan
    release_vel_mm_s: np.ndarray       #: rigid-body cup, read off the plan
    design_vel_mm_s: np.ndarray        #: the ballistic take-off it aims for
    throw_tilt: np.ndarray             #: (rx, ry) GRAVITY frame, rad
    plan_tilt: np.ndarray              #: (rx, ry) PLAN frame, rad
    tilt_deg: float                    #: the commanded (gravity-frame) tilt
    aim_deg: float                     #: the take-off's angle from vertical
    hand_rel_rps: float                #: hand-relative speed at release
    hand_cap_rps: float
    heave_mm_s: float                  #: platform z velocity at release
    axis: np.ndarray                   #: physical (gravity-frame) cup axis
    plat_vel_mm_s: np.ndarray          #: platform velocity at release
    hand_top_rev: float
    k_release: int                     #: the transition knot (release in [k, k+1])
    #: Diagnostics: the repo cup map (fixed tilt centre) minus the rigid body,
    #: at the release, and the landing miss the repo map's release would
    #: imply at ``flight_s`` (mm).
    repo_minus_rigid_site_mm: np.ndarray
    repo_minus_rigid_vel_mm_s: np.ndarray
    repo_minus_rigid_landing_mm: float


# ── knot-grid primitives (piecewise-linear acceleration, integrated exactly) ─

def _integrate(acc, h, p0=0.0, v0=0.0):
    n = len(acc)
    p = np.empty(n)
    v = np.empty(n)
    p[0], v[0] = p0, v0
    for k in range(n - 1):
        v[k + 1] = v[k] + h * (acc[k] + acc[k + 1]) / 2.0
        p[k + 1] = p[k] + h * v[k] + h * h * (2.0 * acc[k] + acc[k + 1]) / 6.0
    return p, v


def _add_move(acc, k0, n_spans, distance, h):
    """Superpose a rest-to-rest sine-acceleration move of ``distance``."""
    if n_spans <= 0 or abs(distance) < 1e-15:
        return
    shape = np.sin(2.0 * np.pi * np.arange(n_spans + 1) / n_spans)
    shape[0] = shape[-1] = 0.0
    p, _ = _integrate(shape, h)
    acc[k0:k0 + n_spans + 1] += shape * (distance / p[-1])


def _plat_profile(speed, a_max, ramp, h):
    """Unit-scaled accel knots reaching ``speed`` with ``|acc| <= a_max``:
    a trapezoid (``ramp`` spans up, ``hold`` spans, ``ramp`` down), returned
    normalised so ``profile * u_i / speed`` is channel ``i``'s accel."""
    if speed <= 0.0:
        return np.zeros(1)
    hold = 0
    while speed / (h * (ramp + hold)) > a_max + 1e-9:
        hold += 1
    amp = speed / (h * (ramp + hold))
    up = np.linspace(0.0, amp, ramp + 1)
    return np.concatenate([up, np.full(hold, amp), up[::-1][1:]])


def _hand_stroke(v_rel, g_rps2, A, D, h):
    """``calcThrow`` on the grid: accel knots reaching ``v_rel`` (rev/s) at the
    instant the hand acceleration crosses ``-g_rps2`` inside the transition
    span, then stopping exactly.  Returns ``None`` when the grid cannot hold
    the shape at this speed (a free knot falls outside its cap)."""
    tau = h * (A + g_rps2) / (A + D)
    gain = A * tau - (A + D) * tau * tau / (2.0 * h)
    v1 = v_rel - gain
    n_a = max(1, int(math.ceil(v1 / (h * A) - 0.5)))
    a1 = v1 / h - A * (n_a - 0.5)
    n_d = max(1, int(math.ceil((v1 / h + A / 2.0) / D - 1.0)))
    delta = v1 / h + A / 2.0 - D * n_d
    if not (-1e-9 <= a1 <= A + 1e-9 and -1e-9 <= delta <= D + 1e-9):
        return None
    acc = np.concatenate([[0.0, a1], np.full(n_a, A), np.full(n_d, -D),
                          [-delta, 0.0]])
    kt = 1 + n_a
    p, v = _integrate(acc, h)
    up = (p[kt] + v[kt] * tau + A * tau * tau / 2.0
          - (A + D) * tau ** 3 / (6.0 * h))
    return dict(acc=acc, kt=kt, tau=tau, up_rev=up, total_rev=float(p[-1]))


def _hand_cap(g_rps2, A, D, h, top_rev, bottom_rev, vel_limit):
    """Largest hand-relative release speed whose stroke fits ``[bottom, top]``."""
    def fits(v):
        s = _hand_stroke(v, g_rps2, A, D, h)
        return s is not None and bottom_rev + s['total_rev'] <= top_rev
    lo, hi = 0.0, float(vel_limit)
    if fits(hi):
        return hi
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        if fits(mid):
            lo = mid
        else:
            hi = mid
    return lo


def _refuse(code, msg):
    raise uc.CycleInfeasible(code, [msg])


def _lever_mm(hand_rev):
    """Cup opening's distance from the platform centre along the cup axis
    (level: ``cup_z = centre_z + L``, the level half of the repo map)."""
    rc = cr.RealizeConfig()
    slider_mm = float(hand_rev) * MM_PER_REV + float(rc.slider_rev_zero_mm)
    return slider_mm + float(rc.cup_z_base_mm) - _C_Z


def _rev_for_lever(lever_mm):
    rc = cr.RealizeConfig()
    return (lever_mm - float(rc.cup_z_base_mm) + _C_Z
            - float(rc.slider_rev_zero_mm)) / MM_PER_REV


def rigid_cup_state(pose, hand_rev, axis) -> np.ndarray:
    """Rigid-body cup opening (mm): the IK's platform centre plus ``axis * L``.

    ``axis`` is the PHYSICAL cup axis (gravity frame — for an unlevelled
    machine simply ``cup_axis(rx, ry)`` of the pose).
    """
    p = np.asarray(pose, float).reshape(6)
    centre = np.array([p[0], p[1], p[2] + float(hw.GEOM_INITIAL_HEIGHT_MM)])
    return centre + np.asarray(axis, float) * _lever_mm(hand_rev)


def rigid_cup_velocity(pose_vel, hand_vel_rps, axis) -> np.ndarray:
    """Rigid-body cup velocity (mm/s) at CONSTANT attitude (zero tilt rate —
    the release span by construction)."""
    pv = np.asarray(pose_vel, float).reshape(6)
    return pv[:3] + np.asarray(axis, float) * float(hand_vel_rps) * MM_PER_REV


# ── the design ────────────────────────────────────────────────────────────────

def design_magazine(seed: uc.CycleState, terminal: MagazineTerminal, limits,
                    mcfg: Optional[MagazineConfig] = None) -> MagazineDesign:
    """Build (but do not gate) the magazine plan.  Raises ``CycleInfeasible``
    with a ``MAGAZINE_*`` code when no shape exists; :func:`plan_magazine` is
    the gated entry point."""
    mcfg = MagazineConfig() if mcfg is None else mcfg
    h = float(hw.JB_TRAJ_KNOT_DT_S)
    rc = cr.RealizeConfig()
    active_z = float(rc.active_z_mm)

    pv = np.asarray(seed.pose_vel, float).reshape(6)
    if (seed.post_release or float(np.max(np.abs(pv))) > _REST_TOL
            or abs(float(seed.hand_vel_rps)) > _REST_TOL):
        _refuse(MAGAZINE_NOT_AT_REST,
                'the magazine throw plans only from a stopped machine with the '
                'ball seated (seed |pose_vel| %.3g, hand %.3g rev/s, '
                'post_release=%s)' % (float(np.max(np.abs(pv))),
                                      float(seed.hand_vel_rps), seed.post_release))
    start_pose = np.asarray(seed.pose, float).reshape(6)
    start_hand = float(seed.hand_rev)
    corr = seed.levelling_correction

    site = terminal.release_site_mm
    target = terminal.target_mm
    T = float(terminal.flight_s)
    V = (target - site + np.array([0.0, 0.0, 0.5 * G_MM_S2 * T * T])) / T

    # Tilt: the take-off direction (gravity frame), capped at the ceiling in
    # the take-off's vertical plane; the platform leans out for the excess.
    speed = float(np.linalg.norm(V))
    ang = math.degrees(math.acos(max(-1.0, min(1.0, V[2] / speed))))
    aim = V
    if ang > mcfg.max_tilt_deg * (1.0 + 1e-9):
        c = math.radians(mcfg.max_tilt_deg)
        hz = V[:2] / float(np.linalg.norm(V[:2]))
        aim = np.array([hz[0] * math.sin(c), hz[1] * math.sin(c), math.cos(c)])
    tilt_g = np.asarray(tg.tilt_to_throw(aim, max_tilt_deg=mcfg.max_tilt_deg), float)
    tilt_p = uc._tilts_to_plan(tilt_g.reshape(1, 2), corr)[0]
    level_p = uc._tilts_to_plan(np.zeros((1, 2)), corr)[0]
    a = tg.cup_axis(float(tilt_g[0]), float(tilt_g[1]))    # physical axis
    az = float(a[2])

    # Rigid cup at constant tilt: v = u + a * s_dot.  Hand first (along a, up
    # to its cap), the platform supplies the remainder u = V - a * s_dot.
    A_lim = float(limits.hand_acc_limit_rps2) - mcfg.hand_acc_margin_rps2
    A = min(float(mcfg.hand_acc_rps2 if mcfg.hand_acc_rps2 is not None
                  else te.ACCEL_AUTHORITY_RPS2), A_lim)
    D = A_lim
    g_rps2 = G_MM_S2 * az / MM_PER_REV       # axial acc == -g a_z at release
    top_rev = float(fz.HAND_STROKE_MAX_REV) - mcfg.hand_top_margin_rev
    cap = _hand_cap(g_rps2, A, D, h, top_rev, mcfg.hand_bottom_min_rev,
                    float(limits.hand_vel_limit_rps) * (1.0 - 1e-3))
    s_need = float(V @ a)
    lean = V - a * s_need                         # 0 unless the tilt is capped
    if float(np.linalg.norm(lean)) > mcfg.max_platform_mm_s:
        _refuse(MAGAZINE_TILT,
                'the aim is %.2f deg from vertical; capped at %.1f deg the '
                'platform would have to lean out at %.0f mm/s (> %.0f mm/s '
                'budget) — lengthen flight_s or move the target'
                % (ang, mcfg.max_tilt_deg, float(np.linalg.norm(lean)),
                   mcfg.max_platform_mm_s))
    s_dot = min(s_need, cap * MM_PER_REV)
    u = V - a * s_dot
    if float(np.linalg.norm(u)) > mcfg.max_platform_mm_s:
        _refuse(MAGAZINE_UNREACHABLE,
                'needs a %.0f mm/s platform velocity beyond the hand cap '
                '%.2f m/s (> %.0f mm/s budget) for a %.2f m/s take-off'
                % (float(np.linalg.norm(u)), cap * MM_PER_REV / 1000.0,
                   mcfg.max_platform_mm_s, speed / 1000.0))
    hs = _hand_stroke(s_dot / MM_PER_REV, g_rps2, A, D, h)
    if hs is None:
        _refuse(MAGAZINE_UNREACHABLE,
                'the hand stroke has no grid shape at %.0f mm/s' % s_dot)
    p_unit = _plat_profile(float(np.linalg.norm(u)), mcfg.plat_acc_mmps2,
                           int(mcfg.plat_ramp_spans), h)
    u_norm = float(np.linalg.norm(u))
    n_pacc = len(p_unit) - 1

    # Timeline (knots).
    n_pre = int(round(mcfg.pre_s / h))
    n_gap = int(round(mcfg.gap_s / h))
    k_rel_min = n_pre + n_gap + max(n_pacc, hs['kt'])
    t_rel_min = k_rel_min * h + hs['tau']
    t_req = float(terminal.t_release_s)
    if t_req < t_rel_min - h / 2.0:
        _refuse(MAGAZINE_WINDOW,
                'release requested %.3f s after the origin; the shape needs at '
                'least %.3f s (pre %.2f s + stroke)' % (t_req, t_rel_min, mcfg.pre_s))
    n_wait = max(0, int(round((t_req - t_rel_min) / h)))
    k_t = n_wait + k_rel_min
    k_hs = k_t - hs['kt']
    k_ps = k_t - n_pacc
    k_hstop = k_hs + len(hs['acc']) - 1
    n_hold = uc.post_release_hold_knots()
    k_z_stop = k_t + 1 + n_pacc
    k_xy_stop = k_t + 1 + n_hold + n_pacc
    n_tr = int(round(mcfg.tilt_return_s / h))
    n_hr = int(round(mcfg.hand_return_s / h))
    n_pr = int(round(mcfg.plat_return_s / h))
    k_end = max(k_t + 1 + n_hold + n_tr, k_hstop + n_hr,
                max(k_z_stop, k_xy_stop) + n_pr) + int(round(mcfg.tail_s / h))
    n = k_end + 1

    acc = np.zeros((7, n))
    if u_norm > 0.0:
        for i in range(3):
            acc[i, k_ps:k_ps + n_pacc + 1] += p_unit * (u[i] / u_norm)
            k_dec = k_t + 1 + (0 if i == 2 else n_hold)
            acc[i, k_dec:k_dec + n_pacc + 1] -= p_unit * (u[i] / u_norm)
    acc[6, k_hs:k_hs + len(hs['acc'])] += hs['acc']

    # Stroke displacement up to the release instant (closed form in-span).
    t_rel = k_t * h + hs['tau']

    def disp(ch):
        p, v = _integrate(acc[ch], h)
        s = t_rel - k_t * h
        a0, a1 = acc[ch, k_t], acc[ch, k_t + 1]
        return p[k_t] + v[k_t] * s + a0 * s * s / 2.0 + (a1 - a0) * s ** 3 / (6.0 * h)

    dpl = np.array([disp(i) for i in range(3)])
    # Release pose: hand bottom chosen so the platform sits at the active z at
    # release when the stroke allows it (least leg travel); else clamped.
    rev_level = _rev_for_lever((float(site[2]) - _C_Z) / az)
    bottom = min(max(rev_level - hs['up_rev'], mcfg.hand_bottom_min_rev),
                 top_rev - hs['total_rev'])
    rev_rel = bottom + hs['up_rev']
    centre = site - a * _lever_mm(rev_rel)
    pose_rel = np.array([centre[0], centre[1],
                         centre[2] - float(hw.GEOM_INITIAL_HEIGHT_MM),
                         tilt_p[0], tilt_p[1], start_pose[5]])
    pose_stroke = pose_rel.copy()
    pose_stroke[:3] -= dpl

    k0 = n_wait
    for i in range(5):
        _add_move(acc[i], k0, n_pre, pose_stroke[i] - start_pose[i], h)
    _add_move(acc[6], k0, n_pre, bottom - start_hand, h)
    k_tr = k_t + 1 + n_hold
    _add_move(acc[3], k_tr, n_tr, level_p[0] - tilt_p[0], h)
    _add_move(acc[4], k_tr, n_tr, level_p[1] - tilt_p[1], h)
    rest = terminal.rest_site_mm
    end_xy = rest[:2]                       # level (physically): centre under the cup
    end_hand = _rev_for_lever(float(rest[2]) - _C_Z)
    start7 = np.concatenate([start_pose, [start_hand]])
    for i, goal, k_from in ((0, end_xy[0], k_xy_stop), (1, end_xy[1], k_xy_stop),
                            (2, active_z, k_z_stop)):
        p, _ = _integrate(acc[i], h, start7[i])
        _add_move(acc[i], k_from, n_pr, goal - p[-1], h)
    p, _ = _integrate(acc[6], h, start_hand)
    _add_move(acc[6], k_hstop, n_hr, end_hand - p[-1], h)

    P = np.empty((7, n))
    Vk = np.empty((7, n))
    for i in range(7):
        P[i], Vk[i] = _integrate(acc[i], h, start7[i])
    # Knot 0 is the seed's own float (the continuity pin), exactly.
    P[:6, 0] = start_pose
    P[6, 0] = start_hand
    plan = CyclePlan(pose=P[:6].T, pose_vel=Vk[:6].T, hand_rev=P[6],
                     hand_vel_rps=Vk[6], dt=h, contact_knots=((0, k_t),))

    pose_r, twist_r, _ = plan.state_at(t_rel)
    rev_r, rps_r = plan.hand_at(t_rel)
    if float(np.max(np.abs(twist_r[3:5]))) > 1e-9:
        # The rigid velocity below omits the tilt-rate term; the shape holds
        # the attitude through the release span, so this is a build defect.
        raise RuntimeError('magazine shape is tilting at the release (%r rad/s)'
                           % (twist_r[3:5],))
    site_r = rigid_cup_state(pose_r, rev_r, a)
    vel_r = rigid_cup_velocity(twist_r, rps_r, a)
    # Diagnostic: what the repo's fixed-tilt-centre map reads off the same knots.
    site_repo = uc.cup_state_from_platform(pose_r, rev_r)
    vel_repo = uc.cup_velocity_from_platform(pose_r, twist_r, rev_r, rps_r)
    land_repo = site_repo + vel_repo * T + np.array([0.0, 0.0, -0.5 * G_MM_S2 * T * T])
    return MagazineDesign(
        plan=plan, t_release_s=float(t_rel), release_site_mm=site_r,
        release_vel_mm_s=vel_r, design_vel_mm_s=V, throw_tilt=tilt_g,
        plan_tilt=np.asarray(tilt_p, float),
        tilt_deg=float(math.degrees(math.hypot(*tilt_g))),
        aim_deg=float(ang), hand_rel_rps=float(rps_r), hand_cap_rps=float(cap),
        heave_mm_s=float(twist_r[2]), axis=a,
        plat_vel_mm_s=np.asarray(twist_r[:3], float),
        hand_top_rev=float(np.max(P[6])), k_release=int(k_t),
        repo_minus_rigid_site_mm=site_repo - site_r,
        repo_minus_rigid_vel_mm_s=vel_repo - vel_r,
        repo_minus_rigid_landing_mm=float(np.linalg.norm(land_repo - target)))


def plan_magazine(seed: uc.CycleState, terminal: MagazineTerminal, cfg,
                  limits, geom=None, *, mcfg: Optional[MagazineConfig] = None
                  ) -> sg.Segment:
    """Plan the magazine throw into a gated, rest-terminal :class:`Segment`.

    ``cfg`` is the caller's ``segments.SegmentConfig`` (accepted so
    ``executor.install_segment`` calls this where it would call
    ``plan_segment``; the shape's own knobs are ``mcfg``).  The plan is gated
    by ``feasibility.validate_cycle`` at ``limits`` — the LIVE session limits —
    and a refusal raises ``uc.CycleInfeasible(report.code, ...)`` unchanged.

    The meta's ``kind`` is :data:`MAGAZINE` (no production code branches on
    ``CycleMeta.kind``): it is how ``executor.install_segment`` recognises a
    MAGAZINE record and refuses to splice ANY segment onto it, since the meta
    carries no ``cup_plan`` for ``unified_cycle.splice_at`` to read.
    """
    del cfg  # the QP holds (pre-release hold) do not apply: the heave cruises
    if limits is None:
        raise ValueError('plan_magazine needs the live session limits to gate '
                         'against; none were passed')
    geom = StewartGeometry() if geom is None else geom
    t_wall = time.perf_counter()
    d = design_magazine(seed, terminal, limits, mcfg)
    report = fz.validate_cycle(d.plan, limits, geom)
    if not report.ok:
        raise uc.CycleInfeasible(report.code, report.reasons, report=report)
    release = uc.ReleaseMark(
        t_s=d.t_release_s, site_mm=d.release_site_mm, vel_mm_s=d.release_vel_mm_s,
        flight_s=float(terminal.flight_s), target_mm=terminal.target_mm,
        tilt=d.throw_tilt,
        stroke_clear_s=uc.plan_stroke_clear_s(d.plan, d.t_release_s),
        pre_hold_knots=0)
    meta = uc.CycleMeta(
        kind=MAGAZINE, n_knots=int(d.plan.n_knots), dt=float(d.plan.dt),
        duration_s=float(d.plan.total_duration), releases=(release,),
        catches=(), report=report, plan_wall_s=time.perf_counter() - t_wall,
        tilts=np.asarray(d.plan.pose[:, 3:5], float),
        receive_tilt=np.zeros(2), throw_tilt=d.throw_tilt,
        takeoff_vel_mps=np.zeros(3),          # rest-terminal, like a SETTLE
        levelling_correction=seed.levelling_correction)
    # release_state stays None: the release is mid-span, so no QP window may
    # chain from it — the next skill plans from this segment's terminal REST.
    return sg.Segment(kind=MAGAZINE, plan=d.plan, meta=meta, splice_k=0,
                      event_t_s=d.t_release_s,
                      takeoff_vel_mm_s=d.release_vel_mm_s,
                      rest_site_mm=terminal.rest_site_mm, warm_start=None,
                      release_state=None)
