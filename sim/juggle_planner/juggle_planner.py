"""Online per-throw task-space planner for the BB-led juggle demo.

Replaces the offline ``juggle_optimizer`` + open-loop ``TrajectoryPlayer`` with
an **online** planner re-solved once per throw, in the spirit of Kai Ploeger's
kinematic toss-juggling planner (``kinematic_planning_for_nball_toss_juggling``,
IEEE 9981678). Each cycle the cup's 3-D Cartesian trajectory (the hand opening)
is re-planned **against the observed incoming ball** and from the **achieved**
cup state, so tracking and throw errors self-correct instead of accumulating —
the failure mode that defeated the offline-trajectory architecture (see logbook
2026-06-27-throw-aim-band-limit-and-closed-loop-catch).

Decision space — the cup's Cartesian **jerk** over one cycle; a triple
integrator gives pos/vel/acc; the objective minimises mean-squared acceleration
(smoothness). Constraints encode the physical throw/catch invariants in task
space:

* **Throw** (end of cycle): cup position = throw point, cup velocity = the
  ballistic take-off velocity that lands the ball at the throw target one flight
  later, and cup acceleration = gravity (the cup is in free-fall at release, so
  the ball detaches cleanly with no spurious force — Kai's ``cacc == g``).
* **Catch** (mid cycle, at the *observed* ball's touch-down time): cup position
  = the predicted ball touch-down position, and the cup velocity just before is
  **collinear** with the ball's arrival velocity (velocity-matched catch).

Realisation onto our morphology is a separate concern (see
``cup_traj_to_platform_slider`` in :mod:`sim.juggle_demo`): a **level** platform
tracks the cup's lateral (xy) motion and the **slider** does the fast vertical
(z) stroke — chosen because the band-limit characterisation (2026-06-27) found
the slider tracks 1:1 to >4 m/s while the platform tracks only *smooth* lateral
motion well (-3 dB ~5 Hz, perfect statically within a ~150 mm workspace).

Units: SI (metres, seconds, m/s, m/s²) internally — the ballistics are natural
in SI and this mirrors Kai. Callers in the mm-based sim convert at the boundary.
Pure CasADi + NumPy; no MuJoCo, no ROS2.
"""
from __future__ import annotations

import dataclasses
from typing import Optional

import casadi as cas
import numpy as np

GRAVITY = np.array([0.0, 0.0, -9.806])


def ballistic_touchdown(pos, vel, touchdown_height):
    """When/where/with-what-velocity a ball reaches ``touchdown_height`` (falling).

    Mirrors Kai's ``get_touchdown``. ``pos``/``vel`` are the ball's current
    world state (m, m/s). Returns ``(t_td, pos_td, vel_td)`` — the time until
    touch-down, the world position there, and the velocity there. Solves the
    quadratic ``z(t) = touchdown_height`` for the **descending** root. Uses the
    module-level :data:`GRAVITY` throughout (z-component for the quadratic, full
    vector for the propagation) so the two stay consistent.
    """
    pos = np.asarray(pos, dtype=float)
    vel = np.asarray(vel, dtype=float)
    g = GRAVITY[2]
    vz = vel[2]
    disc = vz * vz / g**2 - 2.0 * (pos[2] - touchdown_height) / g
    if disc < 0.0:
        disc = 0.0
    t_td = -vz / g + np.sqrt(disc)
    pos_td = pos + vel * t_td + 0.5 * GRAVITY * t_td**2
    vel_td = vel + GRAVITY * t_td
    return float(t_td), pos_td, vel_td


def takeoff_velocity(throw_pos, target_pos, flight_s, g=GRAVITY):
    """Ballistic launch velocity from ``throw_pos`` to land at ``target_pos``
    after ``flight_s`` seconds (m/s). ``v = (target-throw)/T - 0.5 g T``."""
    throw_pos = np.asarray(throw_pos, dtype=float)
    target_pos = np.asarray(target_pos, dtype=float)
    return (target_pos - throw_pos) / flight_s - 0.5 * g * flight_s


def _cas_cross(a, b):
    """Cross product of two 3-vectors (CasADi MX columns). Mirrors Kai's
    ``cas_cross_product`` — used for the tilted-axis detach constraint."""
    return cas.vertcat(a[1] * b[2] - a[2] * b[1],
                       a[2] * b[0] - a[0] * b[2],
                       a[0] * b[1] - a[1] * b[0])


@dataclasses.dataclass
class CupCyclePlan:
    """One planned cup cycle, sampled on the ``dt`` grid (SI units).

    ``pos``/``vel``/``acc`` are ``(n_steps+1, 3)``; ``jerk`` is ``(n_steps, 3)``.
    ``t`` is the per-knot time (s) from the cycle start. ``catch_k`` is the knot
    index nearest the catch, ``takeoff_vel`` the planned release velocity.
    """
    pos: np.ndarray
    vel: np.ndarray
    acc: np.ndarray
    jerk: np.ndarray
    t: np.ndarray
    dt: float
    catch_k: int
    takeoff_vel: np.ndarray

    def sample(self, t_rel: float):
        """Cubic (jerk-constant) interpolation of the cup state at ``t_rel`` (s).

        Clamps to the cycle window. Returns ``(pos, vel, acc)`` 3-vectors (SI).
        """
        n = self.jerk.shape[0]
        t_rel = max(0.0, min(float(t_rel), n * self.dt))
        k = min(int(t_rel / self.dt), n - 1)
        dtau = t_rel - k * self.dt
        p = (self.pos[k] + self.vel[k] * dtau + 0.5 * self.acc[k] * dtau**2
             + (1.0 / 6.0) * self.jerk[k] * dtau**3)
        v = self.vel[k] + self.acc[k] * dtau + 0.5 * self.jerk[k] * dtau**2
        a = self.acc[k] + self.jerk[k] * dtau
        return p, v, a


@dataclasses.dataclass
class PlannerConfig:
    """Knobs for :func:`plan_cup_cycle` (SI units)."""
    dt: float = 0.025                       # planner timestep (s); 40 Hz
    # Jerk is bounded PER AXIS by morphology: the vertical (z) cup motion is the
    # SLIDER (fast, perfect tracking) so it gets a high bound; the lateral (xy)
    # motion is the band-limited PLATFORM so it gets a modest bound that keeps
    # the lateral content inside the platform's good-tracking band (~3 Hz).
    max_jerk_z: float = 6000.0              # |cup z jerk| (m/s³) — slider
    max_jerk_xy: float = 300.0             # |cup xy jerk| (m/s³) — platform
    # Extra penalty on the cup's LATERAL (xy) acceleration — keeps the platform
    # (which must supply the xy motion and is band-limited) moving smoothly so
    # it stays inside its good-tracking band. 0 = Kai's plain min-accel.
    lateral_accel_weight: float = 3.0
    # Number of knots after takeoff to keep the cup moving straight up the throw
    # axis (clean detach, no sideways shove) and before touchdown to align the
    # cup velocity with the ball (velocity-matched catch). Mirrors Kai.
    n_detach: int = 2
    # Throw COAST (the operator's decelerate-and-separate model). Kai detaches in
    # FREE-FALL (cup acc == gravity at release): the ball is then weightless
    # relative to the cup, so any lateral velocity it carries (from a ~15 mm
    # off-centre seat — the band-limit catch error) PERSISTS and the throw flings
    # it beyond the catch reach (the 2-ball divergence; logbook 2026-06-27
    # online-replanning, throw-amplification). Instead, COAST: hold the cup at
    # CONSTANT velocity (acc == 0) for a few knots across the throw so the ball is
    # pressed into the cup (normal force = m g) and FRICTION settles its lateral
    # kick + centres it BEFORE the cup decelerates past -g and it separates at the
    # (now clean) take-off velocity. >0 enables the coast over the first
    # ``throw_coast_knots`` knots after the throw; 0 = Kai's free-fall detach.
    # SOFT (a strong penalty pulling the post-throw vertical accel toward 0, not
    # a hard acc==0) because a hard coast keeps the cup rising into the slider
    # ceiling within the tight apex-1.3 vertical budget — infeasible. The soft
    # penalty flattens the deceleration just enough to press the ball in.
    throw_coast_knots: int = 0
    throw_coast_weight: float = 0.0         # penalty on post-throw vertical accel
    # Velocity-matched catch (concern 2) — split BY MORPHOLOGY AXIS (the
    # level-platform decoupling applied to the catch). The failure this fixes:
    # a uniform soft velocity penalty lets the smoothness cost win on the
    # vertical, so the cup near-stops at the bottom of the oval (~-2.5 m/s)
    # while the ball descends at ~-5.3 m/s — the ball punches past the
    # nearly-stopped cup and sinks below the seat radius (measured; logbook
    # 2026-06-27 online-replanning, catch re-diagnosis). The fix mirrors what
    # the old demo learned (its separate ``catch_slider_vel_ratio``):
    #   * VERTICAL (z) — the SLIDER is the fast, perfect actuator, so make it
    #     descend WITH the ball: a STRONG match toward
    #     ``catch_slider_vel_ratio`` × the ball's arrival vz. The cup rides the
    #     ball down and cushions it to a co-moving stop in the cup.
    #   * LATERAL (xy) — the PLATFORM is band-limited, so keep a SOFT, modest
    #     match toward ``catch_vel_ratio`` × the small lateral arrival velocity
    #     (a hard lateral match conflicts with the platform's modest jerk and
    #     makes the NLP infeasible — the original concern-2 rationale).
    # The vertical ratio < 1 still leaves a small closing velocity so the ball
    # seats rather than floating; the firmer weight is what the old uniform
    # penalty lacked. NOTE the weight is MODERATE, not maximal: a full vertical
    # match drives the cup down at the ball's full arrival speed, which on a
    # short slider slams the floor and ejects the ball — the seating sweet spot
    # is a cup descent ~60-70% of the ball's (the demo's realisation overrides
    # these for its specific pattern; see ``sim.juggle_online._plan_cycle``).
    catch_vel_ratio: float = 0.7            # LATERAL (xy) match ratio — platform
    catch_vel_weight: float = 200.0         # LATERAL match weight (soft)
    catch_slider_vel_ratio: float = 0.7     # VERTICAL (z) match ratio — slider
    catch_slider_vel_weight: float = 600.0  # VERTICAL match weight (moderate)
    # Lateral DWELL at the catch — the online-native port of the old demo's
    # closed-loop reach+dwell (logbook 2026-06-27, ``_catch_reach_xy``). The
    # per-cycle re-plan already supplies the REACH (the catch is the observed
    # ball's touch-down); this supplies the DWELL: hold the cup's xy AT the
    # catch over a window of knots so the platform SETTLES at the landing with
    # ~zero lateral velocity through the seat, instead of swiping past (a moving
    # cup passes the ball and never seats — the old demo's central catch lesson).
    # The slider still descends with the ball (vertical match) — the morphology
    # split applied to the catch. Soft (strong-weighted) so it stays feasible
    # against the carry back to the throw; the window is kept short so the carry
    # still has time (the online tempo's catch->throw dwell is tighter than the
    # old demo's). 0 knots / 0 weight disables the dwell.
    #
    # DISABLED BY DEFAULT: a window/weight sweep on the 2-ball runner (logbook
    # 2026-06-27 online-replanning, catch arc) found the dwell HURTS net — it
    # fixes the caught ball's co-motion but distorts the *throw* (the NLP
    # redistributes the trajectory to satisfy the xy-hold), and the binding
    # constraint turned out to be throw consistency, not catch co-motion. Every
    # dwell setting (pre/post 1-3, weight 400-1800) scored below no-dwell. Kept
    # as an opt-in knob because the lesson (settle-don't-swipe) is sound and a
    # future pattern with more catch->throw dwell time may benefit.
    catch_dwell_pre: int = 0                # knots before the catch to hold xy
    catch_dwell_post: int = 0               # knots after the catch to hold xy
    catch_dwell_weight: float = 0.0         # xy-hold weight (0 = off; ~1500 on)
    workspace_xy_m: float = 0.15            # |cup xy - centre| bound (m)
    # Cup z box (m, world). NOTE for integration: the demo's realisation MUST
    # override these (and the throw/catch heights) to the SLIDER's reachable
    # range, or the slider clamps and the cup can't follow the plan. At centroid
    # z=170 mm the cup reaches ~0.66–1.01 m (cup_z ≈ 0.66 m + slider stroke
    # 0–355 mm), AND the post-release rise (the cup overshoots ~0.12 m before it
    # can decelerate from a ~5 m/s throw) means the THROW release must sit mid-
    # range (~0.85 m), not at the top. /tmp probe_realize confirmed both: a
    # 0.45 m floor clamped 124 mm; a 0.80 m release overshot 1.04 m. Defaults
    # below are a permissive generic box for the unit tests; the demo narrows it.
    z_min_m: float = 0.45
    z_max_m: float = 1.10
    solver_print: bool = False


def plan_cup_cycle(pos0, vel0, acc0,
                   throw_pos, throw_target, flight_s,
                   catch_time_s, catch_pos, catch_vel,
                   period_s,
                   cfg: Optional[PlannerConfig] = None,
                   detach_axis=None) -> CupCyclePlan:
    """Plan one cup cycle (just-after-throw -> catch -> next throw), SI units.

    Parameters
    ----------
    pos0, vel0, acc0 : (3,) array
        Cup state at the cycle start (m, m/s, m/s²) — the ACHIEVED state, so the
        re-plan is closed-loop.
    throw_pos : (3,)
        Cup position at the end-of-cycle throw (m).
    throw_target : (3,)
        World position the thrown ball should land at (m) after ``flight_s``.
    flight_s : float
        Ball flight time for the outgoing throw (s).
    catch_time_s : float
        Time from the cycle start to the incoming ball's touch-down (s),
        from :func:`ballistic_touchdown` on the OBSERVED ball.
    catch_pos, catch_vel : (3,)
        Observed incoming ball's touch-down position and velocity (m, m/s).
    period_s : float
        Cycle duration (s) — the next throw is at ``period_s``.
    detach_axis : (3,) array-like, optional
        The cup's symmetry (up) axis at the **cycle-start** throw, in the world
        frame — the axis along which the just-thrown ball detaches (Rung 2a's
        tilt-aimed throw). Default ``None`` (and the level ``[0, 0, 1]``) is the
        flat cup: the detach constraint reduces EXACTLY to the original
        ``cacc_xy == 0`` (so the ``tilt = 0`` plan is byte-for-byte unchanged). A
        tilted axis constrains the detach to be collinear with it (see below).

    Frame
    -----
    The xy workspace box (``cfg.workspace_xy_m``) is centred on the world xy
    origin, so all positions (``pos0``, ``throw_pos``, ``catch_pos``) must be in
    a frame whose lateral centre is xy=0 (the demo's throw/catch sit at ±100 mm
    about the origin). ``solve()`` raises on infeasibility — the per-throw online
    loop must wrap it (fall back to the last good plan / a held pose) and should
    warm-start from the previous cycle's solution.
    """
    if cfg is None:
        cfg = PlannerConfig()
    dt = cfg.dt
    n_steps = int(round(period_s / dt))
    g = GRAVITY

    opti = cas.Opti()
    cjerk = opti.variable(3, n_steps)

    p_pos0 = opti.parameter(3, 1)
    p_vel0 = opti.parameter(3, 1)
    p_acc0 = opti.parameter(3, 1)
    opti.set_value(p_pos0, np.asarray(pos0, float).reshape(3, 1))
    opti.set_value(p_vel0, np.asarray(vel0, float).reshape(3, 1))
    opti.set_value(p_acc0, np.asarray(acc0, float).reshape(3, 1))

    # Triple-integrator rollout (Kai's exact-polynomial Euler step).
    cpos = p_pos0
    cvel = p_vel0
    cacc = p_acc0
    for k in range(n_steps):
        cpos = cas.horzcat(cpos, cpos[:, -1] + cvel[:, -1] * dt
                           + 0.5 * cacc[:, -1] * dt**2 + (1.0 / 6.0) * cjerk[:, k] * dt**3)
        cvel = cas.horzcat(cvel, cvel[:, -1] + cacc[:, -1] * dt
                           + 0.5 * cjerk[:, k] * dt**2)
        cacc = cas.horzcat(cacc, cacc[:, -1] + cjerk[:, k] * dt)

    # Objective: mean squared acceleration + lateral-accel smoothness.
    cost = cas.sum1(cas.sum2(cacc**2)) / n_steps
    if cfg.lateral_accel_weight > 0.0:
        cost = cost + cfg.lateral_accel_weight * cas.sum1(cas.sum2(cacc[0:2, :]**2)) / n_steps

    # ---- Throw (end of cycle) ----
    v_takeoff = takeoff_velocity(throw_pos, throw_target, flight_s)
    opti.subject_to(cpos[:, -1] == np.asarray(throw_pos, float).reshape(3, 1))
    opti.subject_to(cvel[:, -1] == v_takeoff.reshape(3, 1))
    opti.subject_to(cacc[:, -1] == g.reshape(3, 1))
    # Detach: the just-thrown ball (released at the cycle start) leaves cleanly
    # along the cup's symmetry axis when the contact force on it is purely AXIAL
    # — i.e. the net applied acceleration (cacc - g) is collinear with the cup
    # axis, so there is no sideways shove. Kai's invariant:
    # ``cross(cacc - g, hand_axis) == 0`` over the first ``n_detach`` knots.
    #   * LEVEL (``detach_axis`` None / world +z): hand_axis = [0,0,1], for which
    #     the cross reduces EXACTLY to ``cacc_xy == 0`` — kept as the literal
    #     original path so the tilt=0 plan is byte-for-byte unchanged.
    #   * TILTED (``detach_axis`` given): the ball is thrown along the TILTED cup
    #     axis, so the lateral take-off comes from the slider speed PROJECTED
    #     through the tilt (lateral = |v| sin θ) rather than from band-limited
    #     platform translation (the level-decoupling's wall). Constrain
    #     (cacc - g) collinear with that axis.
    level = detach_axis is None or np.allclose(detach_axis, [0.0, 0.0, 1.0], atol=1e-9)
    if level:
        for i in range(1, cfg.n_detach + 1):
            opti.subject_to(cacc[0:2, i] == 0.0)
    else:
        axis = np.asarray(detach_axis, float).reshape(3)
        axis = axis / np.linalg.norm(axis)
        g_col = g.reshape(3, 1)
        axis_col = axis.reshape(3, 1)
        for i in range(1, cfg.n_detach + 1):
            opti.subject_to(_cas_cross(cacc[:, i] - g_col, axis_col) == 0)

    # ---- Catch (interpolated touch-down time) ----
    k_td = int(np.floor(catch_time_s / dt))
    k_td = max(0, min(k_td, n_steps - 1))
    d_t = catch_time_s - k_td * dt
    cpos_td = (cpos[:, k_td] + cvel[:, k_td] * d_t + 0.5 * cacc[:, k_td] * d_t**2
               + (1.0 / 6.0) * cjerk[:, k_td] * d_t**3)
    opti.subject_to(cpos_td == np.asarray(catch_pos, float).reshape(3, 1))
    # Velocity-matched catch — split by morphology axis (see PlannerConfig).
    # Vertical (z): STRONG match so the slider descends WITH the ball; lateral
    # (xy): SOFT match so the band-limited platform's jerk stays feasible.
    catch_vel = np.asarray(catch_vel, float)
    cvel_td = cvel[:, k_td] + cacc[:, k_td] * d_t + 0.5 * cjerk[:, k_td] * d_t**2
    v_lat_target = (cfg.catch_vel_ratio * catch_vel[0:2]).reshape(2, 1)
    cost = cost + cfg.catch_vel_weight * cas.sumsqr(cvel_td[0:2] - v_lat_target)
    v_z_target = cfg.catch_slider_vel_ratio * float(catch_vel[2])
    cost = cost + cfg.catch_slider_vel_weight * (cvel_td[2] - v_z_target)**2

    # Lateral DWELL: hold the cup's xy at the catch over [k_td-pre, k_td+post]
    # so the platform settles at the observed landing (zero lateral velocity)
    # through the seat rather than swiping past (see PlannerConfig.catch_dwell_*).
    if cfg.catch_dwell_weight > 0.0 and (cfg.catch_dwell_pre or cfg.catch_dwell_post):
        catch_xy = np.asarray(catch_pos, float)[0:2].reshape(2, 1)
        for k in range(max(0, k_td - cfg.catch_dwell_pre),
                       min(n_steps + 1, k_td + cfg.catch_dwell_post + 1)):
            cost = cost + cfg.catch_dwell_weight * cas.sumsqr(cpos[0:2, k] - catch_xy)

    # Throw COAST (soft): pull the vertical accel toward 0 for the first
    # ``throw_coast_knots`` knots after the throw so the cup flattens (presses
    # the ball in -> friction settles its lateral kick) before decelerating. See
    # PlannerConfig.throw_coast_*. Vertical only — the lateral is already pinned
    # to 0 by the detach constraint.
    if cfg.throw_coast_knots > 0 and cfg.throw_coast_weight > 0.0:
        for i in range(1, min(cfg.throw_coast_knots, n_steps) + 1):
            cost = cost + cfg.throw_coast_weight * cacc[2, i]**2

    opti.minimize(cost)

    # ---- Limits ----
    opti.subject_to(opti.bounded(-cfg.max_jerk_xy, cas.vec(cjerk[0:2, :]), cfg.max_jerk_xy))
    opti.subject_to(opti.bounded(-cfg.max_jerk_z, cas.vec(cjerk[2, :]), cfg.max_jerk_z))
    for k in range(n_steps + 1):
        opti.subject_to(opti.bounded(-cfg.workspace_xy_m, cpos[0, k], cfg.workspace_xy_m))
        opti.subject_to(opti.bounded(-cfg.workspace_xy_m, cpos[1, k], cfg.workspace_xy_m))
        opti.subject_to(opti.bounded(cfg.z_min_m, cpos[2, k], cfg.z_max_m))

    opts = {'print_time': 0, 'ipopt.print_level': 5 if cfg.solver_print else 0,
            'ipopt.sb': 'yes'}
    opti.solver('ipopt', opts)
    sol = opti.solve()

    pos = np.asarray(sol.value(cpos)).T          # (n+1, 3)
    vel = np.asarray(sol.value(cvel)).T
    acc = np.asarray(sol.value(cacc)).T
    jerk = np.asarray(sol.value(cjerk)).T        # (n, 3)
    t = np.arange(n_steps + 1) * dt
    return CupCyclePlan(pos=pos, vel=vel, acc=acc, jerk=jerk, t=t, dt=dt,
                        catch_k=k_td, takeoff_vel=v_takeoff)
