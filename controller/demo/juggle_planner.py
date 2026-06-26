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
    # Velocity-matched catch (concern 2): the cup velocity at the catch is
    # encouraged toward ``catch_vel_ratio`` × the ball's arrival velocity — a
    # SOFT penalty, not a hard equality, because a hard collinearity/velocity
    # constraint conflicts with the band-limited platform's modest lateral jerk
    # and makes the NLP infeasible. The slider satisfies the (dominant,
    # vertical) component exactly; the platform approximates the small lateral.
    # 0.7 leaves a 30% closing velocity so the ball seats into the cup.
    catch_vel_ratio: float = 0.7
    catch_vel_weight: float = 200.0
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
                   cfg: Optional[PlannerConfig] = None) -> CupCyclePlan:
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
    # Detach straight up the throw axis (no sideways acceleration just after
    # release) — here the throw axis is world +z (level cup), so constrain the
    # lateral acceleration to gravity's (zero) for the first n_detach knots.
    for i in range(1, cfg.n_detach + 1):
        opti.subject_to(cacc[0:2, i] == 0.0)

    # ---- Catch (interpolated touch-down time) ----
    k_td = int(np.floor(catch_time_s / dt))
    k_td = max(0, min(k_td, n_steps - 1))
    d_t = catch_time_s - k_td * dt
    cpos_td = (cpos[:, k_td] + cvel[:, k_td] * d_t + 0.5 * cacc[:, k_td] * d_t**2
               + (1.0 / 6.0) * cjerk[:, k_td] * d_t**3)
    opti.subject_to(cpos_td == np.asarray(catch_pos, float).reshape(3, 1))
    # Velocity-matched catch (SOFT): cup velocity at the catch knot toward
    # ``catch_vel_ratio`` × the ball's arrival velocity. Soft so the band-
    # limited platform's lateral jerk doesn't make it infeasible (see field doc).
    catch_vel = np.asarray(catch_vel, float)
    cvel_td = cvel[:, k_td] + cacc[:, k_td] * d_t + 0.5 * cjerk[:, k_td] * d_t**2
    v_catch_target = (cfg.catch_vel_ratio * catch_vel).reshape(3, 1)
    cost = cost + cfg.catch_vel_weight * cas.sumsqr(cvel_td - v_catch_target)

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
