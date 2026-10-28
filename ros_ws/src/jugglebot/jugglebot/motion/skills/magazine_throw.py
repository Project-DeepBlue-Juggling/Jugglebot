"""The MAGAZINE throw — one formulaic, HAND-ONLY, rest-terminal segment.

Jugglebot throws its one ball back into Ball Butler's magazine.  This module
builds that throw WITHOUT the cup QP: every channel of the 7-channel
:class:`~jugglebot.motion.trajectory.cycle_plan.CyclePlan` is a closed-form
**piecewise-LINEAR acceleration on the 25 ms knot grid**, i.e. a C2 cubic
spline.  That is exactly what the streamed lane can represent (the emitter
interpolates each span as a cubic Hermite from knot ``(p, v)``, which
reproduces a linear-acceleration cubic exactly) and exactly what
``feasibility.validate_cycle`` demands of the hand (``HAND_LIMIT_C2``).  The
gate is NOT bypassed: the finished plan is run through ``validate_cycle`` at the
limits the caller passed, and any refusal is raised as the same
:class:`~jugglebot.motion.unified_cycle.CycleInfeasible` the QP path raises, so
``executor.install_segment`` reports it unchanged.  Nothing here raises a
limit, a ceiling or a safety constant.

THE PLATFORM IS STATIONARY THROUGH THE STROKE (owner decision, 2026-10-13)
------------------------------------------------------------------------
The first build (a7c1469a) let the legs heave the platform during the stroke
to supply the speed the hand cannot.  The owner withdrew that the same day:
"too risky and asking too much of the legs, for little benefit".  So the hand
supplies ALL of the take-off speed, along the tilted cup axis, and the legs
only (a) tilt and offset the platform BEFORE the stroke so the release lands on
the requested site at the requested attitude and (b) bring it back to level
rest AFTER the release.  A target the hand cannot reach is REFUSED, with the
speed it would need and the least-speed flight time, never flown faster.

THE SHAPE (rest -> throw -> rest, one ball, sensor-confirmed in the cup)
-----------------------------------------------------------------------
0. WAIT — hold the seed rest so the release lands on the requested instant.
1. PRE — sine-acceleration rest-to-rest moves of every channel to the
   stroke-start state: the platform tilts to the throw attitude and offsets so
   the release lands ON the requested site; the hand drops to the stroke
   bottom (the ball stays seated: the cup's downward acceleration is far above
   the C-CUP-2 contact floor, which the gate checks over ``contact_knots``).
2. HAND — the deleted Teensy ``calcThrow`` shape on the grid: knots
   ``[0, a1, A..A | -D..-D, -delta, 0]``, A and D AT their caps, one
   transition span ``+A -> -D`` that contains the release, and the two free
   knot values ``a1`` / ``delta`` absorbing the grid quantisation so the
   hand-relative speed at release is exactly what the aim needs and the hand
   stops exactly.  The platform HOLDS the release pose throughout.
3. RELEASE — inside the transition span, at the instant the cup's AXIAL
   acceleration reaches the ball's free-fall acceleration along the axis
   (``-g * a_z``).  The platform is still, so the instant is closed-form.
4. POST — after the post-release hold, the tilt returns to level, the empty
   hand returns to the rest site's park height, the platform returns to the
   rest site at the active z.  The plan ends at REST.

THE AIM
-------
* The cup axis IS the ballistic take-off direction (``tilt_to_throw``).  An
  aim past ``MAX_TILT_DEG`` (12°) is REFUSED (:data:`MAGAZINE_TILT`) — the
  refusal names the flight time that would bring it inside the ceiling (a
  longer flight is a steeper, slower take-off).
* The hand supplies the take-off speed along the axis.  Past the hand cap —
  the largest hand-relative release speed whose full stroke (bottom ->
  release -> stop at the decel cap) stays under ``HAND_STROKE_MAX_REV`` and
  under the session hand-velocity limit — the throw is REFUSED
  (:data:`MAGAZINE_UNREACHABLE`), naming the speed it needs, the cap, and the
  least-speed flight time for that target.

THE CUP IS THE RIGID BODY (main-session adjudication 2026-10-13)
----------------------------------------------------------------
The platform is rigid and the IK rotates it about the platform centre
``pos + (0, 0, GEOM_INITIAL_HEIGHT_MM)`` (``motion/ik_solver.py``
``compute_leg_vectors``: ``platform_centre = pos + geom.init_height_vec``), whose
world z at the active pose is ``578.2 + 170.0 = 748.2`` mm =
``CUP_TILT_CENTER_Z_MM`` (``trajectory/shaping.py``, derived as exactly that
sum).  So the rigid cup opening is ``centre(pose) + a * L(slider)``, and the
release site, velocity and instant announced in the meta are the rigid ones.
With the platform stationary at release the repo's
``unified_cycle.cup_velocity_from_platform`` reads the same release velocity
(the two maps differed only under heave); the site differs only when the hand
bottom is clamped off the active-z solution (a few mm at most).  The hand acts
along the PHYSICAL cup axis ``a``, which is the GRAVITY-frame aim; the plan's
commanded tilts are that aim in the PLAN frame (``unified_cycle._tilts_to_plan``
from the seed's ``CycleState.levelling_correction`` — the same conversion
``_realize`` applies, contract C-LEVEL-1/2 row E8; knot 0 keeps the seed's own
float).  ``validate_cycle`` is unchanged: it gates the pose+hand knots, which
are what streams.
"""

from __future__ import annotations

import dataclasses
import math
import time
from typing import Optional

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
MAGAZINE_UNREACHABLE = 'MAGAZINE_UNREACHABLE'   #: the hand cannot reach the speed
MAGAZINE_TILT = 'MAGAZINE_TILT'                 #: aiming needs > MAX_TILT_DEG of tilt
MAGAZINE_WINDOW = 'MAGAZINE_WINDOW'             #: release requested before the shape fits
#: ``executor.install_segment``'s refusal for a MAGAZINE asked to splice onto a
#: moving plan — this throw is only ever planned from a stopped machine.
MAGAZINE_NEEDS_REST = 'MAGAZINE_NEEDS_REST'

G_MM_S2 = float(ballistics_bc.GRAVITY_MMS2)
MM_PER_REV = 1000.0 / float(hw.HAND_REV_PER_M)
_C_Z = float(tg.CUP_TILT_CENTER_Z_MM)
_REST_TOL = 1e-6          #: |velocity| below which a seed counts as stopped
#: Flight times the refusal advice scans for the least-speed throw (s).
_FLIGHT_SCAN_S = (0.30, 2.00, 0.01)


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
    pre_s: float = 0.80                   #: rest -> stroke-start move
    gap_s: float = 0.05                   #: settle between PRE and the stroke
    tilt_return_s: float = 0.50
    hand_return_s: float = 0.40
    plat_return_s: float = 0.70           #: xy/z offset back to the rest site
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
    hand_rel_rps: float                #: hand-relative speed at release
    hand_cap_rps: float
    axis: np.ndarray                   #: physical (gravity-frame) cup axis
    hand_top_rev: float
    k_release: int                     #: the transition knot (release in [k, k+1])


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


def takeoff_velocity(site_mm, target_mm, flight_s) -> np.ndarray:
    """The ballistic take-off (mm/s) from ``site`` landing on ``target`` after
    ``flight_s`` (drag-free, the same model the meta's release is read with)."""
    site = np.asarray(site_mm, float).reshape(3)
    target = np.asarray(target_mm, float).reshape(3)
    T = float(flight_s)
    return (target - site + np.array([0.0, 0.0, 0.5 * G_MM_S2 * T * T])) / T


def aim_deg(vel_mm_s) -> float:
    """The take-off's angle from vertical (deg)."""
    v = np.asarray(vel_mm_s, float).reshape(3)
    speed = float(np.linalg.norm(v))
    if speed <= 0.0:
        return 0.0
    return math.degrees(math.acos(max(-1.0, min(1.0, float(v[2]) / speed))))


def least_speed_flight(site_mm, target_mm, max_tilt_deg=float(tg.MAX_TILT_DEG)):
    """``(flight_s, speed_mm_s)`` of the slowest take-off from ``site`` that
    lands on ``target`` inside the tilt ceiling, scanned over
    :data:`_FLIGHT_SCAN_S`; ``None`` when no flight time aims inside it.
    Advice for the operator positioning the magazine, not a design input —
    the flight time flown is always the operator's ``flight_s``."""
    lo, hi, step = _FLIGHT_SCAN_S
    best = None
    for T in np.arange(lo, hi + step / 2.0, step):
        v = takeoff_velocity(site_mm, target_mm, T)
        if aim_deg(v) > max_tilt_deg:
            continue
        s = float(np.linalg.norm(v))
        if best is None or s < best[1]:
            best = (float(T), s)
    return best


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
    V = takeoff_velocity(site, target, T)
    speed = float(np.linalg.norm(V))
    ang = aim_deg(V)

    # Tilt: the cup axis IS the take-off direction; past the ceiling, refuse.
    if ang > mcfg.max_tilt_deg * (1.0 + 1e-9):
        best = least_speed_flight(site, target, mcfg.max_tilt_deg)
        advice = ('no flight time aims it inside the ceiling — move the target'
                  if best is None else
                  'a flight of %.2f s aims inside it at %.2f m/s'
                  % (best[0], best[1] / 1000.0))
        _refuse(MAGAZINE_TILT,
                'the aim is %.2f deg from vertical at flight_s %.2f, over the '
                '%.1f deg tilt ceiling; %s' % (ang, T, mcfg.max_tilt_deg, advice))
    tilt_g = np.asarray(tg.tilt_to_throw(V, max_tilt_deg=mcfg.max_tilt_deg), float)
    tilt_p = uc._tilts_to_plan(tilt_g.reshape(1, 2), corr)[0]
    level_p = uc._tilts_to_plan(np.zeros((1, 2)), corr)[0]
    a = tg.cup_axis(float(tilt_g[0]), float(tilt_g[1]))    # physical axis
    az = float(a[2])

    # Hand: the whole take-off speed along a, up to its cap.
    A_lim = float(limits.hand_acc_limit_rps2) - mcfg.hand_acc_margin_rps2
    A = min(float(mcfg.hand_acc_rps2 if mcfg.hand_acc_rps2 is not None
                  else te.ACCEL_AUTHORITY_RPS2), A_lim)
    D = A_lim
    g_rps2 = G_MM_S2 * az / MM_PER_REV       # axial acc == -g a_z at release
    top_rev = float(fz.HAND_STROKE_MAX_REV) - mcfg.hand_top_margin_rev
    cap = _hand_cap(g_rps2, A, D, h, top_rev, mcfg.hand_bottom_min_rev,
                    float(limits.hand_vel_limit_rps) * (1.0 - 1e-3))
    s_dot = float(V @ a)                      # == speed: a is V's direction
    if s_dot > cap * MM_PER_REV:
        best = least_speed_flight(site, target, mcfg.max_tilt_deg)
        advice = ('' if best is None else
                  '; the least-speed flight for this target is %.2f s at '
                  '%.2f m/s' % (best[0], best[1] / 1000.0))
        _refuse(MAGAZINE_UNREACHABLE,
                'needs a %.2f m/s hand-relative release at flight_s %.2f; the '
                'hand cap is %.2f m/s (stroke %.2f rev at %.0f/%.0f rev/s^2, '
                'the platform stationary)%s'
                % (s_dot / 1000.0, T, cap * MM_PER_REV / 1000.0,
                   top_rev - mcfg.hand_bottom_min_rev, A, D, advice))
    hs = _hand_stroke(s_dot / MM_PER_REV, g_rps2, A, D, h)
    if hs is None:
        _refuse(MAGAZINE_UNREACHABLE,
                'the hand stroke has no grid shape at %.0f mm/s' % s_dot)

    # Timeline (knots).
    n_pre = int(round(mcfg.pre_s / h))
    n_gap = int(round(mcfg.gap_s / h))
    k_rel_min = n_pre + n_gap + hs['kt']
    t_rel_min = k_rel_min * h + hs['tau']
    t_req = float(terminal.t_release_s)
    if t_req < t_rel_min - h / 2.0:
        _refuse(MAGAZINE_WINDOW,
                'release requested %.3f s after the origin; the shape needs at '
                'least %.3f s (pre %.2f s + stroke)' % (t_req, t_rel_min, mcfg.pre_s))
    n_wait = max(0, int(round((t_req - t_rel_min) / h)))
    k_t = n_wait + k_rel_min
    k_hs = k_t - hs['kt']
    k_hstop = k_hs + len(hs['acc']) - 1
    n_hold = uc.post_release_hold_knots()
    n_tr = int(round(mcfg.tilt_return_s / h))
    n_hr = int(round(mcfg.hand_return_s / h))
    n_pr = int(round(mcfg.plat_return_s / h))
    k_tr = k_t + 1 + n_hold                   # platform returns start here
    k_end = (max(k_tr + n_tr, k_hstop + n_hr, k_tr + n_pr)
             + int(round(mcfg.tail_s / h)))
    n = k_end + 1

    acc = np.zeros((7, n))
    acc[6, k_hs:k_hs + len(hs['acc'])] += hs['acc']
    t_rel = k_t * h + hs['tau']

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

    k0 = n_wait
    for i in range(5):
        _add_move(acc[i], k0, n_pre, pose_rel[i] - start_pose[i], h)
    _add_move(acc[6], k0, n_pre, bottom - start_hand, h)
    _add_move(acc[3], k_tr, n_tr, level_p[0] - tilt_p[0], h)
    _add_move(acc[4], k_tr, n_tr, level_p[1] - tilt_p[1], h)
    rest = terminal.rest_site_mm
    end_hand = _rev_for_lever(float(rest[2]) - _C_Z)
    start7 = np.concatenate([start_pose, [start_hand]])
    # Level (physically): the centre sits under the cup, so xy = the rest xy.
    for i, goal in ((0, float(rest[0])), (1, float(rest[1])), (2, active_z)):
        p, _ = _integrate(acc[i], h, start7[i])
        _add_move(acc[i], k_tr, n_pr, goal - p[-1], h)
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
    if float(np.max(np.abs(twist_r))) > 1e-9:
        # The release velocity below is the hand's alone; the shape holds the
        # platform still through the release span, so this is a build defect.
        raise RuntimeError('magazine shape has the platform moving at the '
                           'release (%r)' % (twist_r,))
    site_r = rigid_cup_state(pose_r, rev_r, a)
    vel_r = rigid_cup_velocity(twist_r, rps_r, a)
    return MagazineDesign(
        plan=plan, t_release_s=float(t_rel), release_site_mm=site_r,
        release_vel_mm_s=vel_r, design_vel_mm_s=V, throw_tilt=tilt_g,
        plan_tilt=np.asarray(tilt_p, float),
        tilt_deg=float(math.degrees(math.hypot(*tilt_g))),
        hand_rel_rps=float(rps_r), hand_cap_rps=float(cap), axis=a,
        hand_top_rev=float(np.max(P[6])), k_release=int(k_t))


def plan_magazine(seed: uc.CycleState, terminal: MagazineTerminal, cfg,
                  limits, geom=None, *, mcfg: Optional[MagazineConfig] = None
                  ) -> sg.Segment:
    """Plan the magazine throw into a gated, rest-terminal :class:`Segment`.

    ``cfg`` is the caller's ``segments.SegmentConfig`` (accepted so
    ``executor.install_segment`` calls this where it would call
    ``plan_segment``; the shape's own knobs are ``mcfg``).  The plan is gated
    by ``feasibility.validate_cycle`` at ``limits`` — the LIVE sitting limits —
    and a refusal raises ``uc.CycleInfeasible(report.code, ...)`` unchanged.

    The meta's ``kind`` is :data:`MAGAZINE` (no production code branches on
    ``CycleMeta.kind``): it is how ``executor.install_segment`` recognises a
    MAGAZINE record and refuses to splice ANY segment onto it, since the meta
    carries no ``cup_plan`` for ``unified_cycle.splice_at`` to read.
    """
    del cfg  # the QP's pre-release hold knobs do not apply to this shape
    if limits is None:
        raise ValueError('plan_magazine needs the live sitting limits to gate '
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
