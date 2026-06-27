"""Online re-planning juggle runner — the Kai-style architecture.

Replaces the offline-trajectory + open-loop-player demo (`sim/juggle_demo.py`)
with an **online** loop: each throw, observe the in-flight ball, re-plan the
cup's Cartesian trajectory for the next cycle (`controller.demo.juggle_planner`)
from the ACHIEVED cup state, and realise it on the platform+slider via the
**level-platform decoupling** (centroid_xy = cup_xy, slider = cup_z − offset).
The cup traces a continuous carry oval (no stop/starts); the throw/catch use the
same contact physics (`contact_carry`) as the old demo. See logbook
2026-06-27-online-replanning-architecture-and-cup-bandlimit.

This is a WIP standalone runner (kept beside the old `juggle_demo.py` so its
tests stay green) — the new architecture under development.

STATUS (2026-06-27): the loop runs end-to-end (re-plan → realise → step) and the
realisation tracks the planned cup oval (de-risked: ~26 mm lateral RMS, see
logbook), but it does NOT yet catch (0/0). Known issues to resolve next:
  1. **Init/phasing** — the planner's cycle puts the throw at the boundary
     (cup at throw_pos moving UP at v_take). At cycle-0 start the cup is settled
     STATIC, so `begin_physics_throw` releases ball 0 at ~0 velocity and it
     falls. Fix: pre-roll the cup through the carry (catch→throw) so it arrives
     at the throw point moving up, OR initialise a running pattern (Kai's
     `initialize_balls_in_cascade`), spawning ball 1's arc timed from the
     established cup motion.
  2. **Catch seating** — with a ball riding the cup (contact carry) the catch
     contact + the velocity-matched seat need tuning (the de-risk tracked the
     cup WITHOUT a ball); verify the soft/stiff contact switch windows and the
     seat thresholds against this loop.
  3. **Throw release timing** — confirm `begin_physics_throw` fires exactly at
     the boundary instant where cacc≈gravity (clean detach), not before.
Architecture, realisation, and planner are validated; the above is focused
phasing/tuning work.

Pure-Python, no ROS2. SI internally in the planner; mm at the plant boundary.
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
    GRAVITY, PlannerConfig, ballistic_touchdown, plan_cup_cycle,
)
from sim.plant.mujoco_plant import MuJoCoPlant

CONTROL_DT = 0.025

# Realisation constants (from the band-limit static map, tools/probes/
# juggle_cup_bandlimit.py): at centroid pose z = Z_ACTIVE_MM the cup (hand
# opening) sits at cup_z_world ≈ CUP_Z_BASE_MM + slider_mm.
Z_ACTIVE_MM = 170.0
CUP_Z_BASE_MM = 659.6
SLIDER_STROKE_MM = 355.0


@dataclasses.dataclass
class OnlineJuggleConfig:
    separation_m: float = 0.10          # lateral throw↔catch separation
    throw_z_m: float = 0.85             # cup release height (mid slider range)
    catch_z_m: float = 0.70             # cup catch height (low)
    apex_m: float = 1.30                # ball apex above the throw
    duration_s: float = 20.0
    seed: int = 0


def _flight_and_speed(apex_m, dz_m):
    """Flight time + vertical launch speed for a ball thrown up ``apex_m`` above
    the release that lands ``dz_m`` below it (catch lower than throw)."""
    g = -GRAVITY[2]
    v_up = np.sqrt(2 * g * apex_m)                      # speed to reach apex
    t_up = v_up / g
    t_down = np.sqrt(2 * (apex_m + dz_m) / g)           # apex back down to catch
    return t_up + t_down, v_up


def realize(cup_m):
    """Cup Cartesian target (m, world) → (pose_6dof_mm, slider_mm), level platform."""
    cup_mm = np.asarray(cup_m, float) * 1000.0
    pose = np.array([cup_mm[0], cup_mm[1], Z_ACTIVE_MM, 0.0, 0.0, 0.0])
    slider = float(np.clip(cup_mm[2] - CUP_Z_BASE_MM, 0.0, SLIDER_STROKE_MM))
    return pose, slider


class OnlineJuggleRunner:
    def __init__(self, cfg: OnlineJuggleConfig):
        self.cfg = cfg
        self.plant = MuJoCoPlant(control_dt=CONTROL_DT, contact_carry=True)
        self.bm = self.plant.ball_manager
        self.site_id = mujoco.mj_name2id(self.plant.model, mujoco.mjtObj.mjOBJ_SITE,
                                         'hand_opening')
        self.period = None
        self.flight, self.throw_speed = _flight_and_speed(
            cfg.apex_m, cfg.throw_z_m - cfg.catch_z_m)
        # One throw and one catch per cup cycle; the cup period is the time
        # between throws. For a 2-ball pattern the cup period = flight / 1.648
        # (the existing pattern ratio) — keep it so the catch lands mid-cycle.
        self.period = self.flight / 1.648
        self.catch_offset = self.flight - self.period   # ≈ 0.405·... within cycle
        s = cfg.separation_m / 2.0
        # Throw on the +x side (cup moving up), catch on the −x side (cup down).
        self.throw_pos = np.array([+s, 0.0, cfg.throw_z_m])
        self.catch_pos = np.array([-s, 0.0, cfg.catch_z_m])
        self.captures = []
        self.drops = []

    # ---- cup state ----
    def cup_world_m(self):
        return self.plant.data.site_xpos[self.site_id].copy()

    def cup_vel_world_m(self):
        res = np.zeros(6)
        mujoco.mj_objectVelocity(self.plant.model, self.plant.data,
                                 mujoco.mjtObj.mjOBJ_SITE, self.site_id, res, 0)
        return res[3:6].copy()

    # ---- realisation drive ----
    def _drive(self, plan, t_rel):
        cp, _, _ = plan.sample(t_rel)
        return self.plant.pose_to_extensions(realize(cp)[0])

    def _slider_at(self, plan, t_rel):
        cp, _, _ = plan.sample(t_rel)
        return realize(cp)[1]

    def _plan_cycle(self, pos0, vel0, acc0, observed_ball_state):
        """Plan one cup cycle catching the observed in-flight ball."""
        pcfg = PlannerConfig()
        pcfg.z_min_m = (CUP_Z_BASE_MM) / 1000.0 + 0.005
        pcfg.z_max_m = (CUP_Z_BASE_MM + SLIDER_STROKE_MM) / 1000.0 - 0.005
        if observed_ball_state is not None:
            bpos, bvel = observed_ball_state
            t_td, pos_td, vel_td = ballistic_touchdown(bpos, bvel, self.cfg.catch_z_m)
            catch_time = min(max(t_td, 0.05), self.period - 0.05)
            catch_pos = pos_td
            catch_vel = vel_td
        else:
            catch_time = self.catch_offset
            catch_pos = self.catch_pos
            catch_vel = np.array([0.0, 0.0, -self.throw_speed])
        return plan_cup_cycle(pos0, vel0, acc0, self.throw_pos, self.catch_pos,
                              self.flight, catch_time, catch_pos, catch_vel,
                              self.period, pcfg)

    def run(self):
        plant = self.plant
        plant.reset()
        # Seat ball 0 in the cup at the throw point; spawn ball 1 in flight to be
        # caught at the first catch.
        pose0, slider0 = realize(self.throw_pos)
        for _ in range(80):
            plant.command(plant.pose_to_extensions(pose0)); plant.command_hand(slider0)
            plant.step(CONTROL_DT)
        self.bm.ball(0).spawn_in_hand()
        # Ball 1 back-propagated to be at catch_pos at t=catch_offset.
        t = self.catch_offset
        v_arr = np.array([0.0, 0.0, -self.throw_speed])
        v0 = v_arr - GRAVITY * t
        p0 = self.catch_pos - v0 * t - 0.5 * GRAVITY * t * t
        plant.spawn_ball(p0 * 1000.0, v0 * 1000.0, ball=1)

        held_ball, flight_ball = 0, 1
        # initial plan from the throw state
        v_take = (self.catch_pos - self.throw_pos) / self.flight - 0.5 * GRAVITY * self.flight
        plan = self._plan_cycle(self.throw_pos, v_take, GRAVITY.copy(),
                                (p0, v0))

        n_cycles = int(self.cfg.duration_s / self.period)
        t_global = 0.0
        for cyc in range(n_cycles):
            # --- throw at cycle start: release the held ball ---
            self.plant.set_contact_stiffness(True)
            plant.begin_physics_throw(ball=held_ball)
            held_ball, flight_ball = flight_ball, held_ball
            cycle_caught = False
            # drive the cycle
            n_ticks = int(round(self.period / CONTROL_DT))
            t0 = float(plant.data.time)
            for k in range(n_ticks):
                t_rel = float(plant.data.time) - t0
                # soft contact for the catch portion (after ~40% of cycle)
                self.plant.set_contact_stiffness(t_rel < 0.10)
                plant.command(self._drive(plan, t_rel))
                plant.step(CONTROL_DT,
                           hand_cmd_fn=lambda tt, p=plan, b=t0: self._slider_at(p, tt - b),
                           plat_cmd_fn=lambda tt, p=plan, b=t0: self._drive(p, tt - b))
                if plant.check_and_capture(ball=held_ball):
                    self.captures.append((t_global + t_rel, held_ball))
                    cycle_caught = True
                # drop detection
                bs = plant.get_ball_state(ball=flight_ball)
                if bs and not bs.held and bs.position_mm[2] < -200:
                    self.drops.append((t_global + t_rel, flight_ball))
            t_global += self.period
            # --- re-plan next cycle from achieved cup state + observed ball ---
            cup_p = self.cup_world_m()
            cup_v = self.cup_vel_world_m()
            fb = plant.get_ball_state(ball=flight_ball)
            obs = None
            if fb and not fb.held:
                obs = (np.asarray(fb.position_mm) / 1000.0,
                       np.asarray(fb.velocity_mms) / 1000.0)
            try:
                plan = self._plan_cycle(cup_p, cup_v, GRAVITY.copy(), obs)
            except Exception:
                pass   # keep last plan on solver failure
        return self.captures, self.drops


def run(cfg=None):
    cfg = cfg or OnlineJuggleConfig()
    return OnlineJuggleRunner(cfg).run()


if __name__ == '__main__':
    caps, drops = run()
    print(f"captures: {len(caps)}  drops: {len(drops)}")
    print(f"  {caps}")
