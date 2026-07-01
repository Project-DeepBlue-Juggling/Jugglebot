"""Rung 2b — single-ball tilt-aimed self-catch loop (the MAKE-OR-BREAK gate).

Phase 3 of plans/active/bb-online-juggle-tilt-rearchitecture.md. Composes the
Rung-2a **throw** with the Rung-1 **catch** into the minimal closed loop — one
ball, thrown up and caught by the SAME cup, cycle after cycle, re-planned each
cycle from the achieved cup state + the noisy observed ball:

  cycle = carry the seated ball up the (near-vertical, tilt~=0) column axis
          -> release under stiff contact -> recover (tilted-axis detach)
          -> observe + translate-to-reach + tilt-to-receive + constant-decel
             seat -> (ball seated again)

This is the cleanest test of the plan's riskiest assumption: does tilt make the
throw->catch->throw loop **stable** (loop gain < 1) under real contact + the §3
noise, with **no 2-ball confound**? The gate question: does the throw AMPLIFY the
seat / cup-lateral state (the pre-tilt divergence, loop gain > 1), or does the
tilt architecture break the amplification?

**RESULT (2026-07-01): BREAK — the pure column self-catch does NOT sustain.**
The loop diverges within 0-3 cycles across every seed and recover strategy: the
catch reach amplifies each cycle (~8 mm -> ~50 -> ~110 -> ~210) past the catch's
~60-80 mm reliable reach -> drop. Two compounding, column-specific failure modes
(NOT the pre-tilt band-limit cascade) drive it — see the module Discussion in
`logbook/2026-07-01-rung2b-selfcatch-column-divergence.md`:

1. **Column separation singularity.** A caught (centred, spinless) ball on a pure
   vertical column throw does NOT cleanly detach: the free-fall detach constraint
   (cup acc == g) means cup and ball share the same acceleration on the same
   vertical line, so they ride together. The Rung-2a throw separated only because
   the freshly *spawned* ball carried incidental spin; a Rung-1-*caught* ball is
   spinless -> the primitives do not compose cleanly on a column.
2. **Loop gain > 1 via the reach.** When the ball does separate, the column
   throw's separation is chaotically sensitive to the ball's residual lateral
   state at release, so the small lateral motion from the catch reach / the
   stationary reposition is amplified into a large landing offset -> the catch
   must reach further -> more residual lateral motion -> the reach amplifies.

**Crucially, for a column the tilt is ~0, so the tilt mechanism (decoupling
lateral aim from the band-limited platform) never ENGAGES** — the column does not
test the tilt hypothesis at all; it exercises a degenerate case where tilt is
inactive. The harness is retained as the measurement apparatus + the head-to-head
evidence for the gate; the config exposes ``recover`` and ``stationary`` so the
operator can reproduce every variant behind the verdict.

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
from sim.juggle_catch import _quintic_step
from sim.juggle_noise import BallisticEstimator, JuggleNoise, NoiseConfig
from sim.juggle_tilt import (
    cup_axis, realize_tilted, tilt_to_receive, tilt_to_throw, MAX_TILT_DEG,
)
from sim.plant.mujoco_plant import MuJoCoPlant

CONTROL_DT = 0.025
CUP_Z_BASE_MM = 659.6
SLIDER_STROKE_MM = 355.0
SEAT_RADIUS_MM = 40.0
WORKSPACE_XY_M = 0.15
# A ball whose centre is more than this far from the cup opening after the
# recovery has cleanly separated (vs riding the cup out, "glued"). Lower than the
# open-loop throw harness's 120 mm because the self-catch recover keeps the cup
# nearer the ball (it must descend to catch it), so a clean throw clears ~60 mm.
SEPARATION_MM = 60.0
# The catch's characterised reliable reach (Rung 1). A landing/reach beyond this
# is where the catch starts to miss — the divergence threshold.
CATCH_REACH_MM = 80.0


def _smoothstep(f: float) -> float:
    f = float(np.clip(f, 0.0, 1.0))
    return f * f * (3.0 - 2.0 * f)


class _QuinticSeg:
    """Continuous min-jerk quintic segment (p0,v0,a0) -> (pf,vf,af) over T,
    sampleable at any sub-tick time so the moved cup carries real velocity (the
    Rung-2a stop-and-go lesson). Used only for the between-cycle reposition."""

    def __init__(self, p0, v0, a0, pf, vf, af, T):
        p0 = np.asarray(p0, float); v0 = np.asarray(v0, float); a0 = np.asarray(a0, float)
        pf = np.asarray(pf, float); vf = np.asarray(vf, float); af = np.asarray(af, float)
        self.T = float(T)
        self.c0, self.c1, self.c2 = p0, v0, a0 / 2.0
        h = pf - (self.c0 + self.c1 * T + self.c2 * T ** 2)
        g1 = vf - (self.c1 + 2.0 * self.c2 * T)
        g2 = af - 2.0 * self.c2
        M = np.array([[T ** 3, T ** 4, T ** 5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        c345 = np.linalg.solve(M, np.stack([h, g1, g2]))
        self.c3, self.c4, self.c5 = c345[0], c345[1], c345[2]

    def sample(self, t):
        t = float(np.clip(t, 0.0, self.T))
        p = (self.c0 + self.c1 * t + self.c2 * t ** 2 + self.c3 * t ** 3
             + self.c4 * t ** 4 + self.c5 * t ** 5)
        return p


@dataclasses.dataclass
class SelfCatchConfig:
    """One single-ball tilt-aimed self-catch run. SI units unless noted (mm)."""
    n_cycles: int = 12                 # how many throw->catch cycles to attempt
    # ---- column geometry (throw and catch co-located = a stationary column) ----
    throw_z_m: float = 0.85            # cup release height (mid slider range)
    throw_target_z_m: float = 0.70     # ballistic aim height (sets take-off speed)
    catch_z_m: float = 0.84            # cup catch height (Rung 1)
    flight_s: float = 0.60             # nominal time of flight
    stationary: bool = True            # True: reposition to a FIXED origin each
                                       # cycle (stationary column). False: throw a
                                       # column in-place from the caught position
                                       # (the cup drifts; isolates the reposition).
    recover: str = "plan"              # "plan": Rung-2a plan_cup_cycle recover
                                       # (validated, faithful). "retract": an axial
                                       # slider retract (explored; a knife-edge).
    # ---- carry / release realisation knobs (Rung 2a) ----
    dip_m: float = 0.16
    tilt_ramp_frac: float = 0.6
    n_recovery_ticks: int = 10
    n_hold_ticks: int = 10             # settle-hold at carry_low before each carry
    retract_dist_m: float = 0.06       # (recover="retract" only)
    retract_dur_s: float = 0.10
    # ---- catch realisation knobs (Rung 1) ----
    catch_ready_lift_m: float = 0.06
    catch_slider_vel_ratio: float = 0.95
    catch_start_lead_s: float = 0.16
    seat_decel_time_s: float = 0.16
    reach_settle_floor_s: float = 0.10
    # ---- noise (§3; only TRACKING noise applies — the cup, not the BB, throws) --
    noise: NoiseConfig = dataclasses.field(default_factory=NoiseConfig)
    seed: int = 0
    settle_ticks: int = 70             # platform warm-up before the seed spawn


@dataclasses.dataclass
class SelfCatchCycle:
    """Per-cycle metrics — the loop-gain trend the gate reads."""
    cyc: int
    in_off_start_mm: float             # in-cup lateral offset at the throw
    separated: bool                    # the throw cleanly left the cup
    caught: bool                       # seat confirmed
    held_at_end: bool                  # still seated when the catch window closed
    in_off_end_mm: float               # in-cup lateral offset after the catch
    reach_mm: float                    # how far the catch translated (from origin)
    reach_from_throw_mm: float         # landing distance from the throw xy
    seat_offset_mm: float              # transient offset at the seat instant (-1 if none)
    landing_xy_mm: "tuple[float, float]"
    takeoff_speed_mps: float


@dataclasses.dataclass
class SelfCatchResult:
    cycles: "list[SelfCatchCycle]"

    @property
    def sustained(self) -> int:
        """Consecutive cycles from the start that both caught AND held."""
        n = 0
        for c in self.cycles:
            if c.caught and c.held_at_end:
                n += 1
            else:
                break
        return n

    @property
    def reach_trend_mm(self) -> "list[float]":
        return [round(c.reach_mm, 1) for c in self.cycles]

    @property
    def diverged(self) -> bool:
        """The loop did not hold every attempted cycle (it dropped / broke early),
        or the catch reach ran past the catch's reliable reach somewhere — either
        is the loop-gain > 1 signature."""
        held_all = all(c.caught and c.held_at_end for c in self.cycles)
        return (not held_all) or any(c.reach_mm > CATCH_REACH_MM for c in self.cycles)


class SelfCatchRunner:
    def __init__(self, cfg: SelfCatchConfig):
        self.cfg = cfg
        self.plant = MuJoCoPlant(control_dt=CONTROL_DT, contact_carry=True)
        self.site_id = mujoco.mj_name2id(self.plant.model, mujoco.mjtObj.mjOBJ_SITE,
                                         'hand_opening')
        self.noise = JuggleNoise(cfg.noise, seed=cfg.seed)
        self.est = BallisticEstimator(GRAVITY)
        self.pcfg = PlannerConfig()
        self.pcfg.z_min_m = CUP_Z_BASE_MM / 1000.0 + 0.005
        self.pcfg.z_max_m = (CUP_Z_BASE_MM + SLIDER_STROKE_MM) / 1000.0 - 0.005
        self.viewer = None             # parity with the other runners (unused)

    def cup_world_m(self):
        return self.plant.data.site_xpos[self.site_id].copy()

    def cup_vel_world_m(self):
        res = np.zeros(6)
        mujoco.mj_objectVelocity(self.plant.model, self.plant.data,
                                 mujoco.mjtObj.mjOBJ_SITE, self.site_id, res, 0)
        return res[3:6].copy()

    def _ball_offset_mm(self):
        bs = self.plant.get_ball_state(ball=0)
        bp = np.asarray(bs.position_mm, float)
        return bp - self.cup_world_m() * 1000.0, bs

    def run(self) -> SelfCatchResult:
        cfg = self.cfg
        plant = self.plant
        plant.reset()
        plant.set_contact_stiffness(False)

        origin = np.array([0.0, 0.0])
        # Warm up (and seat the ball) at the SAME carry-low point the cycle-0 carry
        # will start from, derived from the config geometry (throw_z / target_z /
        # dip / axis) rather than a literal height — so the freshly seated ball sees
        # no step position command at cycle 0 under any retune (mirrors
        # sim/juggle_throw.py, which warms up at its computed carry_low). For the
        # default column config this evaluates to [0, 0, 0.70] as before.
        _tp0 = np.array([origin[0], origin[1], cfg.throw_z_m])
        _tg0 = np.array([origin[0], origin[1], cfg.throw_target_z_m])
        _axis0 = cup_axis(*tilt_to_throw(takeoff_velocity(_tp0, _tg0, cfg.flight_s),
                                         MAX_TILT_DEG))
        carry_low0 = _tp0 - _axis0 * cfg.dip_m
        carry_low0[2] = max(carry_low0[2], cfg.throw_target_z_m)

        # ---- Warm up at carry_low, seed ball 0 centred (spawn_in_hand). ----
        pose0, slider0 = realize_tilted(carry_low0[:2], carry_low0[2], 0.0, 0.0)
        for _ in range(cfg.settle_ticks):
            plant.command(plant.pose_to_extensions(pose0))
            plant.command_hand(slider0)
            plant.step(CONTROL_DT)
        plant.ball_manager.ball(0).spawn_in_hand()

        cycles: "list[SelfCatchCycle]" = []
        for cyc in range(cfg.n_cycles):
            off0, bs0 = self._ball_offset_mm()
            in_off_start = float(np.linalg.norm(off0[:2]))
            if not bs0.held:
                cycles.append(SelfCatchCycle(cyc, in_off_start, False, False, False,
                                             float('nan'), float('nan'), float('nan'),
                                             -1.0, (float('nan'), float('nan')), 0.0))
                break

            # Column throw/catch xy: fixed origin (stationary) or the caught xy (drift).
            base_xy = origin if cfg.stationary else self.cup_world_m()[:2]
            throw_pos = np.array([base_xy[0], base_xy[1], cfg.throw_z_m])
            target = np.array([base_xy[0], base_xy[1], cfg.throw_target_z_m])
            v_takeoff = takeoff_velocity(throw_pos, target, cfg.flight_s)
            rx, ry = tilt_to_throw(v_takeoff, MAX_TILT_DEG)
            axis = cup_axis(rx, ry)
            period = max(cfg.flight_s, 0.45)
            catch_time = period * 0.40
            carry_dur = period - catch_time
            clow = throw_pos - axis * cfg.dip_m
            clow[2] = max(clow[2], cfg.throw_target_z_m)
            carry_vel = np.array([0.0, 0.0, -2.0])

            self._reposition_to(clow, cyc, plant)
            self._carry_up(plant, throw_pos, target, v_takeoff, rx, ry, axis, clow,
                           carry_vel, period, catch_time, carry_dur)
            separated, land_true = self._release_recover(
                plant, throw_pos, target, v_takeoff, rx, ry, axis, clow, carry_vel,
                period, catch_time)
            caught, held, in_off_end, reach_mm, seat_off, last_td = self._catch(plant)

            throw_xy_mm = throw_pos[:2] * 1000.0
            land_mm = np.asarray(land_true[:2]) * 1000.0
            cycles.append(SelfCatchCycle(
                cyc=cyc, in_off_start_mm=in_off_start, separated=separated,
                caught=caught, held_at_end=held, in_off_end_mm=in_off_end,
                reach_mm=reach_mm,
                reach_from_throw_mm=float(np.linalg.norm(land_mm - throw_xy_mm)),
                seat_offset_mm=seat_off,
                landing_xy_mm=(float(land_mm[0]), float(land_mm[1])),
                takeoff_speed_mps=float(np.linalg.norm(v_takeoff))))
            if not (caught and held):
                break
        return SelfCatchResult(cycles=cycles)

    # ---- phase helpers -----------------------------------------------------
    def _reposition_to(self, clow, cyc, plant):
        """Smoothly move the held ball back to carry_low then settle-hold (cyc>0).
        cyc 0 is freshly seeded at carry_low (warmed + spawn_in_hand), so it goes
        straight to the carry — an extra hold there only perturbs the fresh seat."""
        if cyc == 0:
            return
        cup_p = self.cup_world_m(); cup_v = self.cup_vel_world_m()
        rep_dur = 0.20
        seg = _QuinticSeg(cup_p, cup_v, np.zeros(3), clow, np.zeros(3),
                          np.zeros(3), rep_dur)
        t0 = float(plant.data.time)

        def rep_cmd(t_abs):
            p = seg.sample(t_abs - t0)
            return realize_tilted(p[:2], float(p[2]), 0.0, 0.0)

        for _ in range(int(round(rep_dur / CONTROL_DT))):
            pose, _ = rep_cmd(float(plant.data.time))
            plant.command(plant.pose_to_extensions(pose))
            plant.step(CONTROL_DT, hand_cmd_fn=lambda t: rep_cmd(t)[1],
                       plat_cmd_fn=lambda t: plant.pose_to_extensions(rep_cmd(t)[0]))
        pose0h, sl0h = realize_tilted(clow[:2], clow[2], 0.0, 0.0)
        for _ in range(self.cfg.n_hold_ticks):
            plant.command(plant.pose_to_extensions(pose0h))
            plant.command_hand(sl0h)
            plant.step(CONTROL_DT)

    def _carry_up(self, plant, throw_pos, target, v_takeoff, rx, ry, axis, clow,
                  carry_vel, period, catch_time, carry_dur):
        """Rung-2a carry: play the plan_cup_cycle carry half, ramping the tilt in."""
        plant.set_contact_stiffness(False)
        plan = plan_cup_cycle(throw_pos, v_takeoff, GRAVITY.copy(), throw_pos, target,
                              self.cfg.flight_s, catch_time, clow, carry_vel, period,
                              self.pcfg, detach_axis=axis)
        n_carry = int(round(carry_dur / CONTROL_DT))
        tc0 = float(plant.data.time)

        def carry_cmd(t_abs):
            cp, _, _ = plan.sample(catch_time + (t_abs - tc0))
            s = _smoothstep((t_abs - tc0) / max(self.cfg.tilt_ramp_frac * carry_dur, 1e-6))
            return realize_tilted(cp[:2], float(cp[2]), rx * s, ry * s)

        for _ in range(n_carry):
            pose, _ = carry_cmd(float(plant.data.time))
            plant.command(plant.pose_to_extensions(pose))
            plant.step(CONTROL_DT, hand_cmd_fn=lambda t: carry_cmd(t)[1],
                       plat_cmd_fn=lambda t: plant.pose_to_extensions(carry_cmd(t)[0]))

    def _release_recover(self, plant, throw_pos, target, v_takeoff, rx, ry, axis,
                         clow, carry_vel, period, catch_time):
        """Release under stiff contact, drive the recover (tilted-axis detach),
        observe the in-flight ball for the catch estimator. Returns (separated,
        true ballistic landing at catch_z)."""
        cfg = self.cfg
        cup_p = self.cup_world_m(); cup_v = self.cup_vel_world_m()
        plant.set_contact_stiffness(True)
        plant.begin_physics_throw(ball=0)
        planB = None
        if cfg.recover == "plan":
            try:
                planB = plan_cup_cycle(cup_p, cup_v, GRAVITY.copy(), throw_pos, target,
                                       cfg.flight_s, catch_time, clow, carry_vel,
                                       period, self.pcfg, detach_axis=axis)
            except Exception:
                planB = None
        retract_target = cup_p - axis * cfg.retract_dist_m
        retract_target[2] = max(retract_target[2], cfg.throw_target_z_m - 0.05)
        retract_seg = _QuinticSeg(cup_p, cup_v, GRAVITY, retract_target,
                                  np.zeros(3), np.zeros(3), cfg.retract_dur_s)
        t0 = float(plant.data.time)
        self.est.reset()

        def recover_cmd(t_abs):
            if cfg.recover == "plan" and planB is not None:
                cp, _, _ = planB.sample(t_abs - t0)
                return realize_tilted(cp[:2], float(cp[2]), rx, ry)
            p = retract_seg.sample(t_abs - t0)
            return realize_tilted(p[:2], float(p[2]), rx, ry)

        for _ in range(cfg.n_recovery_ticks):
            pose, _ = recover_cmd(float(plant.data.time))
            plant.command(plant.pose_to_extensions(pose))
            plant.step(CONTROL_DT, hand_cmd_fn=lambda t: recover_cmd(t)[1],
                       plat_cmd_fn=lambda t: plant.pose_to_extensions(recover_cmd(t)[0]))
            bs = plant.get_ball_state(ball=0)
            true_m = np.asarray(bs.position_mm, float) / 1000.0
            obs_m = self.noise.observe(true_m * 1000.0) / 1000.0
            self.est.add(float(plant.data.time), obs_m)

        bs = plant.get_ball_state(ball=0)
        bp = np.asarray(bs.position_mm, float) / 1000.0
        bv = np.asarray(bs.velocity_mms, float) / 1000.0
        separated = (not bs.held) and (
            float(np.linalg.norm(bp - self.cup_world_m())) * 1000.0 > SEPARATION_MM)
        _, land_true, _ = ballistic_touchdown(bp, bv, cfg.catch_z_m)
        return separated, land_true

    def _catch(self, plant):
        """Rung-1 catch on the in-flight ball: translate-to-reach + tilt-to-receive
        + constant-decel seat. Mirrors sim/juggle_catch.py's per-tick seat state
        machine (see that module for the physics rationale). Returns
        (caught, held_at_end, in_off_end_mm, reach_mm, seat_offset_mm, last_pos_td)."""
        cfg = self.cfg
        plant.set_contact_stiffness(False)
        catch_z = cfg.catch_z_m
        ready_z = catch_z + cfg.catch_ready_lift_m
        cup_p = self.cup_world_m()
        cmd_xy = cup_p[:2].copy(); cmd_vxy = np.zeros(2); cmd_axy = np.zeros(2)
        cmd_tilt = np.zeros(2); cmd_vtilt = np.zeros(2); cmd_atilt = np.zeros(2)
        cmd_z = float(cup_p[2]); cmd_vz = float(self.cup_vel_world_m()[2]); cmd_az = 0.0
        frozen_xy = None; frozen_tilt = None; settle_v0 = None
        caught = False; seat_offset = None
        last_pos_td = np.array([0.0, 0.0, catch_z])
        tcatch0 = float(plant.data.time)
        n_catch = int(round((cfg.flight_s + 0.40) / CONTROL_DT))

        for _k in range(n_catch):
            t_abs = float(plant.data.time)
            bs = plant.get_ball_state(ball=0)
            true_m = np.asarray(bs.position_mm, float) / 1000.0
            obs_m = self.noise.observe(true_m * 1000.0) / 1000.0
            self.est.add(t_abs, obs_m)
            pos_e, vel_e = self.est.estimate()
            t_td, pos_td, vel_td = ballistic_touchdown(pos_e, vel_e, catch_z)
            t_remaining = t_td
            pos_td = np.array(pos_td, float)
            pos_td[:2] = np.clip(pos_td[:2], -WORKSPACE_XY_M, WORKSPACE_XY_M)
            tilt_rx, tilt_ry = tilt_to_receive(vel_td, MAX_TILT_DEG)

            if self.est.n < 4:
                pose, slider = realize_tilted(cmd_xy, cmd_z, cmd_tilt[0], cmd_tilt[1])
                plant.command(plant.pose_to_extensions(pose))
                plant.step(CONTROL_DT, hand_cmd_fn=lambda _t, s=slider: s,
                           plat_cmd_fn=lambda _t, p=pose: plant.pose_to_extensions(p))
                continue

            descending = t_remaining <= cfg.catch_start_lead_s
            if frozen_xy is None:
                frozen_xy = pos_td[:2].copy()
            if descending and frozen_tilt is None:
                frozen_tilt = np.array([tilt_rx, tilt_ry])
            reach_xy = frozen_xy
            reach_tilt = frozen_tilt if frozen_tilt is not None else np.array([tilt_rx, tilt_ry])
            last_pos_td = np.array([reach_xy[0], reach_xy[1], catch_z])

            cmd_xy, cmd_vxy, cmd_axy = _quintic_step(
                cmd_xy, cmd_vxy, cmd_axy, reach_xy, np.zeros(2), np.zeros(2),
                cfg.reach_settle_floor_s, CONTROL_DT)
            horizon_tilt = max(t_remaining, cfg.reach_settle_floor_s)
            cmd_tilt, cmd_vtilt, cmd_atilt = _quintic_step(
                cmd_tilt, cmd_vtilt, cmd_atilt, reach_tilt, np.zeros(2), np.zeros(2),
                horizon_tilt, CONTROL_DT)
            if t_remaining > 0.0:
                if not descending:
                    z_t, vz_t = ready_z, 0.0
                    hz = max(t_remaining - cfg.catch_start_lead_s, 3.0 * CONTROL_DT)
                else:
                    z_t = catch_z
                    vz_t = cfg.catch_slider_vel_ratio * float(vel_td[2])
                    hz = max(t_remaining, 3.0 * CONTROL_DT)
                (cmd_z,), (cmd_vz,), (cmd_az,) = _quintic_step(
                    [cmd_z], [cmd_vz], [cmd_az], [z_t], [vz_t], [0.0], hz, CONTROL_DT)
            else:
                if settle_v0 is None:
                    settle_v0 = min(cmd_vz, -1e-6)
                a_dec = -settle_v0 / cfg.seat_decel_time_s
                if cmd_vz < -1e-4:
                    new_vz = min(cmd_vz + a_dec * CONTROL_DT, 0.0)
                    cmd_z = cmd_z + 0.5 * (cmd_vz + new_vz) * CONTROL_DT
                    cmd_vz, cmd_az = new_vz, a_dec
                else:
                    cmd_vz, cmd_az = 0.0, 0.0

            pose, slider = realize_tilted(cmd_xy, cmd_z, cmd_tilt[0], cmd_tilt[1])
            plant.command(plant.pose_to_extensions(pose))
            plant.step(CONTROL_DT, hand_cmd_fn=lambda _t, s=slider: s,
                       plat_cmd_fn=lambda _t, p=pose: plant.pose_to_extensions(p))

            if not caught and plant.check_and_capture(ball=0):
                caught = True
                off, _ = self._ball_offset_mm()
                seat_offset = float(np.linalg.norm(off))

        final = plant.get_ball_state(ball=0)
        ball_m = np.asarray(final.position_mm, float) / 1000.0
        settled = (ball_m - self.cup_world_m()) * 1000.0
        held_at_end = bool(final.held)
        in_off_end = float(np.linalg.norm(settled[:2]))
        reach_mm = float(np.linalg.norm(last_pos_td[:2]) * 1000.0)
        return (caught, held_at_end, in_off_end, reach_mm,
                float(seat_offset) if seat_offset is not None else -1.0, last_pos_td)


def run_self_catch(cfg: "SelfCatchConfig | None" = None) -> SelfCatchResult:
    """Run one single-ball tilt-aimed self-catch loop headless; return the
    :class:`SelfCatchResult` (the per-cycle loop-gain trend)."""
    return SelfCatchRunner(cfg or SelfCatchConfig()).run()
