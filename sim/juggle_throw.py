"""Rung 2a — single-ball tilt-aimed throw, the throw primitive in isolation.

Phase 2 of plans/active/bb-online-juggle-tilt-rearchitecture.md. A focused
single-ball harness that validates the **throw** primitive on its own, OPEN-LOOP
(no catch), so the throw's accuracy can be characterised before it is closed into
the Rung-2b self-catch loop:

  throw = carry the seated ball up the TILTED cup axis -> detach along that axis

* **Tilt is the engine.** The ballistic take-off velocity that lands the ball on
  the target, ``v_takeoff = takeoff_velocity(throw_pos, target, flight)``, sets
  both the *speed* and the *aim*. The cup is tilted so its symmetry axis points
  ALONG ``v_takeoff`` (:func:`~sim.juggle_tilt.tilt_to_throw`), and the fast
  slider drives the cup up that axis — so the lateral take-off is delivered as
  ``slider_speed × sin(tilt)`` (Rung 0's well-tracked DOF) instead of by the
  band-limited platform translation that wrecked the level-decoupled throw.
* **Carry + detach (real contact, no shortcut).** The ball is seated in the cup
  (``spawn_in_hand``) at a dip below the throw, carried up along the tilted axis
  on a :func:`~controller.demo.juggle_planner.plan_cup_cycle` trajectory (the
  tilt ramped in during the carry, held constant through the release per Rung 0),
  then released by physics (``begin_physics_throw`` + stiff contact). A re-plan
  from the achieved cup state, with the **tilted-axis detach constraint**
  (``detach_axis`` = the tilted cup axis), carries the cup away so the just-thrown
  ball detaches collinear with the axis (Kai's ``cross(cacc - g, axis) == 0``).
* **Open-loop landing.** After release the ball flies freely; its landing at
  ``catch_z`` is both measured from the TRUE state (deterministic open-loop
  accuracy) and predicted from a window of §3-**tracking-noise** observations fed
  to a :class:`~sim.juggle_noise.BallisticEstimator` — the noisy landing the
  Rung-2b catch would actually reach for. (The §3 *BB-throw* noise does not apply
  here: the cup is the thrower, not the Ball Butler.)

Outputs a :class:`ThrowResult` with the landing error + take-off geometry, for
the probe to characterise across the target box and the cadence range and to
relate the throw scatter to the catch's ~60-80 mm reliable reach (Rung 1).

Pure-Python, no ROS2. SI internally; mm at the plant boundary.
"""
from __future__ import annotations

import dataclasses
import os
import sys

import numpy as np

_sim_dir = os.path.dirname(os.path.abspath(__file__))
_repo_root = os.path.dirname(_sim_dir)
for _p in (_sim_dir, _repo_root,
           os.path.join(_repo_root, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_repo_root, 'config', 'generated')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import mujoco

from controller.demo.juggle_planner import (
    GRAVITY, PlannerConfig, ballistic_touchdown, plan_cup_cycle, takeoff_velocity,
)
from sim.juggle_noise import BallisticEstimator, JuggleNoise, NoiseConfig
from sim.juggle_tilt import cup_axis, realize_tilted, tilt_to_throw, MAX_TILT_DEG
from sim.plant.mujoco_plant import MuJoCoPlant

CONTROL_DT = 0.025
CUP_Z_BASE_MM = 659.6
SLIDER_STROKE_MM = 355.0
# A ball whose centre is more than this far from the cup opening at the end of
# the recovery has cleanly separated (vs riding the cup out, "glued").
SEPARATION_MM = 120.0


@dataclasses.dataclass
class SingleThrowConfig:
    """One tilt-aimed open-loop throw. SI units unless noted (mm where stated)."""
    # ---- throw geometry / target ----
    throw_xy_m: "tuple[float, float]" = (0.0, 0.0)     # cup xy at release
    throw_z_m: float = 0.85                            # cup release height (world)
    target_xy_m: "tuple[float, float]" = (0.10, 0.0)   # desired landing xy
    catch_z_m: float = 0.70                            # landing/measure height
    flight_s: float = 0.60                             # CADENCE: time of flight
    # ---- carry / release realisation knobs ----
    dip_m: float = 0.16            # carry dip below the throw, along the tilted axis
    tilt_ramp_frac: float = 0.6    # ramp tilt over this fraction of the carry, then hold
    n_recovery_ticks: int = 10     # post-release ticks driven along the recovery plan
    obs_window: int = 6            # post-release ticks observed for the landing estimate
    max_tilt_deg: float = MAX_TILT_DEG
    # ---- noise (§3, on by default; only TRACKING noise applies to a cup throw) ----
    noise: NoiseConfig = dataclasses.field(default_factory=NoiseConfig)
    seed: int = 0
    settle_ticks: int = 70         # platform warm-up at the dip before spawn


@dataclasses.dataclass
class ThrowResult:
    separated: bool                # ball cleanly left the cup (vs rode it out)
    target_xy_m: "tuple[float, float]"
    landing_xy_m: "tuple[float, float]"          # TRUE ballistic landing (noise-free)
    observed_landing_xy_m: "tuple[float, float]"  # §3-noisy estimator prediction
    error_mm: float                # |true landing - target| (open-loop accuracy)
    observed_error_mm: float       # |noisy predicted landing - target|
    tilt_deg: float                # commanded throw tilt magnitude
    takeoff_speed_mps: float       # |v_takeoff| commanded
    lateral_takeoff_mps: float     # |v_takeoff_xy| commanded (= speed·sin tilt)
    cup_speed_at_release_mps: float  # |cup velocity| achieved at release
    flight_s: float

    @property
    def reach_mm(self) -> float:
        """Distance from the cup home (xy origin) to the TRUE landing — the reach
        the Rung-2b catch must translate to. (Rung 1: reliable to ~60-80 mm.)"""
        return float(np.hypot(*self.landing_xy_m) * 1000.0)

    @property
    def clean(self) -> bool:
        """A usable throw = it separated AND landed within the catch's reliable
        reach (the gate framing: the throw scatter is the catch's reach)."""
        return self.separated and self.error_mm <= 80.0


def _smoothstep(f: float) -> float:
    f = float(np.clip(f, 0.0, 1.0))
    return f * f * (3.0 - 2.0 * f)


class SingleThrowRunner:
    def __init__(self, cfg: SingleThrowConfig):
        self.cfg = cfg
        self.plant = MuJoCoPlant(control_dt=CONTROL_DT, contact_carry=True)
        self.site_id = mujoco.mj_name2id(self.plant.model, mujoco.mjtObj.mjOBJ_SITE,
                                         'hand_opening')
        self.noise = JuggleNoise(cfg.noise, seed=cfg.seed)
        self.est = BallisticEstimator(GRAVITY)
        self.viewer = None         # parity with the other runners (unused headless)

    def cup_world_m(self):
        return self.plant.data.site_xpos[self.site_id].copy()

    def cup_vel_world_m(self):
        res = np.zeros(6)
        mujoco.mj_objectVelocity(self.plant.model, self.plant.data,
                                 mujoco.mjtObj.mjOBJ_SITE, self.site_id, res, 0)
        return res[3:6].copy()

    def run(self) -> ThrowResult:
        cfg = self.cfg
        plant = self.plant
        plant.reset()
        plant.set_contact_stiffness(False)             # soft: carry

        throw_pos = np.array([cfg.throw_xy_m[0], cfg.throw_xy_m[1], cfg.throw_z_m])
        target = np.array([cfg.target_xy_m[0], cfg.target_xy_m[1], cfg.catch_z_m])
        v_takeoff = takeoff_velocity(throw_pos, target, cfg.flight_s)
        rx, ry = tilt_to_throw(v_takeoff, cfg.max_tilt_deg)
        axis = cup_axis(rx, ry)                        # cup symmetry axis (unit)
        tilt_deg = float(np.degrees(np.hypot(rx, ry)))

        pcfg = PlannerConfig()
        pcfg.z_min_m = CUP_Z_BASE_MM / 1000.0 + 0.005
        pcfg.z_max_m = (CUP_Z_BASE_MM + SLIDER_STROKE_MM) / 1000.0 - 0.005

        # Self-consistent periodic plan: starts at the throw (just released),
        # dips to a carry low point, returns to the throw at v_takeoff. We play
        # its carry half (catch_time -> period) to bring the cup UP to the throw.
        period = max(cfg.flight_s, 0.45)
        catch_time = period * 0.40
        carry_low = throw_pos - axis * cfg.dip_m
        carry_low[2] = max(carry_low[2], cfg.catch_z_m)
        carry_vel = np.array([0.0, 0.0, -2.0])
        plan = plan_cup_cycle(throw_pos, v_takeoff, GRAVITY.copy(), throw_pos, target,
                              cfg.flight_s, catch_time, carry_low, carry_vel, period,
                              pcfg, detach_axis=axis)
        carry_dur = period - catch_time

        def tilt_at(t_in_carry: float):
            s = _smoothstep(t_in_carry / max(cfg.tilt_ramp_frac * carry_dur, 1e-6))
            return rx * s, ry * s

        # ---- Warm up at the carry-low point (pre-tilted level), seat the ball ----
        pose0, slider0 = realize_tilted(carry_low[:2], carry_low[2], 0.0, 0.0)
        for _ in range(cfg.settle_ticks):
            plant.command(plant.pose_to_extensions(pose0))
            plant.command_hand(slider0)
            plant.step(CONTROL_DT)
        plant.ball_manager.ball(0).spawn_in_hand()

        # ---- Carry pre-roll: play the plan's carry, ramp the tilt in ----
        n_carry = int(round(carry_dur / CONTROL_DT))
        tc0 = float(plant.data.time)

        def carry_cmd(t_abs: float):
            cp, _, _ = plan.sample(catch_time + (t_abs - tc0))
            trx, tryy = tilt_at(t_abs - tc0)
            return realize_tilted(cp[:2], float(cp[2]), trx, tryy)

        for _ in range(n_carry):
            pose, _ = carry_cmd(float(plant.data.time))
            plant.command(plant.pose_to_extensions(pose))
            plant.step(CONTROL_DT,
                       hand_cmd_fn=lambda t: carry_cmd(t)[1],
                       plat_cmd_fn=lambda t: plant.pose_to_extensions(carry_cmd(t)[0]))

        cup_p = self.cup_world_m()
        cup_v = self.cup_vel_world_m()

        # ---- Release + re-plan the recovery from the achieved cup state, with
        # the TILTED-AXIS detach (the ball detaches collinear with the axis) ----
        plant.set_contact_stiffness(True)
        plant.begin_physics_throw(ball=0)
        try:
            planB = plan_cup_cycle(cup_p, cup_v, GRAVITY.copy(), throw_pos, target,
                                   cfg.flight_s, catch_time, carry_low, carry_vel,
                                   period, pcfg, detach_axis=axis)
        except Exception:
            planB = plan
        t0 = float(plant.data.time)

        def recover_cmd(t_abs: float):
            cp, _, _ = planB.sample(t_abs - t0)
            return realize_tilted(cp[:2], float(cp[2]), rx, ry)   # hold the tilt

        self.est.reset()
        for k in range(cfg.n_recovery_ticks):
            pose, _ = recover_cmd(float(plant.data.time))
            plant.command(plant.pose_to_extensions(pose))
            plant.step(CONTROL_DT,
                       hand_cmd_fn=lambda t: recover_cmd(t)[1],
                       plat_cmd_fn=lambda t: plant.pose_to_extensions(recover_cmd(t)[0]))
            # Observe the in-flight ball with §3 tracking noise (the loop's view).
            if k < cfg.obs_window:
                bs = plant.get_ball_state(ball=0)
                true_m = np.asarray(bs.position_mm, float) / 1000.0
                obs_m = self.noise.observe(true_m * 1000.0) / 1000.0
                self.est.add(float(plant.data.time), obs_m)

        # ---- Landing: TRUE (open-loop accuracy) + §3-noisy estimate (catch view) ----
        bs = plant.get_ball_state(ball=0)
        bp = np.asarray(bs.position_mm, float) / 1000.0
        bv = np.asarray(bs.velocity_mms, float) / 1000.0
        separated = (not bs.held) and (
            float(np.linalg.norm(bp - self.cup_world_m())) * 1000.0 > SEPARATION_MM)
        _, pos_td, _ = ballistic_touchdown(bp, bv, cfg.catch_z_m)
        landing = pos_td[:2]

        if self.est.n >= 2:
            pe, ve = self.est.estimate()
            _, pos_td_obs, _ = ballistic_touchdown(pe, ve, cfg.catch_z_m)
            obs_landing = pos_td_obs[:2]
        else:
            obs_landing = landing

        tgt_xy = np.array(cfg.target_xy_m, float)
        return ThrowResult(
            separated=separated,
            target_xy_m=(float(tgt_xy[0]), float(tgt_xy[1])),
            landing_xy_m=(float(landing[0]), float(landing[1])),
            observed_landing_xy_m=(float(obs_landing[0]), float(obs_landing[1])),
            error_mm=float(np.linalg.norm(landing - tgt_xy) * 1000.0),
            observed_error_mm=float(np.linalg.norm(obs_landing - tgt_xy) * 1000.0),
            tilt_deg=tilt_deg,
            takeoff_speed_mps=float(np.linalg.norm(v_takeoff)),
            lateral_takeoff_mps=float(np.linalg.norm(v_takeoff[:2])),
            cup_speed_at_release_mps=float(np.linalg.norm(cup_v)),
            flight_s=cfg.flight_s,
        )


def run_single_throw(cfg: "SingleThrowConfig | None" = None) -> ThrowResult:
    """Run one tilt-aimed open-loop throw headless; return the :class:`ThrowResult`."""
    return SingleThrowRunner(cfg or SingleThrowConfig()).run()
