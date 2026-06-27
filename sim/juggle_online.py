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

STATUS (2026-06-27): the loop runs end-to-end and catches **5 / 0** (seed 0) —
up from 0, and better than the old offline demo's 2 (whose ≥30 headline test
xfails on the same wall). Logbook
`2026-06-27-online-juggle-throw-fix-catch-axis-split-band-limit-cascade`.

Two fixes landed:
  1. **Throw slam → cycle-0 re-plan.** The slam (ball launched ~16.8 m/s) was a
     pre-roll → main-loop handoff discontinuity: the carry's steep final z-rise
     lags the slider, so the pre-roll ends ~67 mm low (783 not 850 mm) and the
     main loop's t_rel=0 throw command jumps the cup up into the ball. FIX:
     re-plan cycle 0 from the ACHIEVED pre-roll state (every later cycle already
     does this), so cycle 0 is continuous — clean a_cup<−g separation at v_take,
     no slam, no clamp (separation is the operator's decelerate-and-separate).
  2. **Catch seating → axis-split velocity-match** (in the planner). The cup now
     descends MODERATELY with the ball (~−3.4 m/s); a uniform soft match
     near-stopped it (ball sank below the cup), a full match slammed the slider
     floor. See `_plan_cycle` and PlannerConfig.catch_slider_vel_*.

REMAINING WALL — sustained 2-ball: the pattern collapses to 1-ball. A centred
(pre-seated) ball grooves forever; a caught-then-thrown ball diverges. Root
cause (measured): the catch seats the ball ~15 mm off-centre (the platform's
band-limited lateral tracking error), and the contact-carry throw AMPLIFIES that
to a >150 mm landing error — beyond the ±150 mm reach → lost (loop gain >1). This
is the same band-limit cascade that xfails the old demo. Catch-side tuning
(dwell), the throw coast, a symmetric startup, smaller separation, lateral
smoothing, and lower apex were all ruled out — they distort the throw or are
unphysical. The fix needs a LESS POSITION-SENSITIVE THROW (a small
controlled-velocity nudge toward v_take in the stiff-throw window) or a
self-centring cup — scoped for a follow-up. The dwell/coast knobs are kept
disabled-by-default in PlannerConfig (opt-in; they didn't help here).

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
        # Nominal ballistic take-off / arrival velocities for the pattern.
        self.v_take = (self.catch_pos - self.throw_pos) / self.flight - 0.5 * GRAVITY * self.flight
        self.v_arrival = self.v_take + GRAVITY * self.flight
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
        # Catch velocity-match, tuned for THIS pattern's arrival speed (~5.3 m/s).
        # The slider (vertical) descends WITH the ball, but only MODERATELY: a
        # full match (cup at -5.3) slams the slider floor (only ~40 mm below the
        # catch) and ejects the ball; near-stop (cup ~-2.5) lets the ball punch
        # past. Empirical sweet spot is cup ~-3.4 m/s at the catch (z_ratio 0.7,
        # weight 600 → captures 5 vs 0-3 across the grid's extremes; /tmp probe_grid).
        pcfg.catch_vel_ratio = 0.85         # lateral (platform) — soft
        pcfg.catch_slider_vel_ratio = 0.7   # vertical (slider) — moderate
        pcfg.catch_slider_vel_weight = 600.0
        if observed_ball_state is not None:
            bpos, bvel = observed_ball_state
            t_td, pos_td, vel_td = ballistic_touchdown(bpos, bvel, self.cfg.catch_z_m)
            catch_time = min(max(t_td, 0.05), self.period - 0.05)
            catch_pos = pos_td
            catch_vel = vel_td
        else:
            catch_time = self.catch_offset
            catch_pos = self.catch_pos
            catch_vel = self.v_arrival
        return plan_cup_cycle(pos0, vel0, acc0, self.throw_pos, self.catch_pos,
                              self.flight, catch_time, catch_pos, catch_vel,
                              self.period, pcfg)

    def run(self):
        plant = self.plant
        plant.reset()
        # First plan (nominal catch) — used for the carry pre-roll and cycle 0.
        plan = self._plan_cycle(self.throw_pos, self.v_take, GRAVITY.copy(), None)

        # ---- Carry pre-roll: seat ball 0 at the catch point and drive the cup
        # along the plan's CARRY (catch→throw) so it arrives at the throw point
        # already moving UP at v_take. Without this the cup starts static and the
        # first begin_physics_throw releases ball 0 at ~0 velocity (it falls).
        cup0, slider0 = realize(self.catch_pos)
        for _ in range(80):
            plant.command(plant.pose_to_extensions(cup0)); plant.command_hand(slider0)
            plant.step(CONTROL_DT)
        self.bm.ball(0).spawn_in_hand()                 # seat ball 0 at catch_pos
        plant.set_contact_stiffness(False)              # soft: carry
        n_carry = int(round((self.period - self.catch_offset) / CONTROL_DT))
        tc0 = float(plant.data.time)
        for k in range(n_carry):
            tt = self.catch_offset + (float(plant.data.time) - tc0)
            plant.step(CONTROL_DT,
                       hand_cmd_fn=lambda t, p=plan, b=tc0: self._slider_at(p, self.catch_offset + (t - b)),
                       plat_cmd_fn=lambda t, p=plan, b=tc0: self._drive(p, self.catch_offset + (t - b)))

        # Ball 1 spawned in flight to arrive at catch_pos at t=catch_offset of
        # cycle 0 (the main loop starts now, cup at throw_pos moving up).
        t = self.catch_offset
        v0 = self.v_arrival - GRAVITY * t
        p0 = self.catch_pos - v0 * t - 0.5 * GRAVITY * t * t
        plant.spawn_ball(p0 * 1000.0, v0 * 1000.0, ball=1)

        # Re-plan cycle 0 from the ACHIEVED pre-roll cup state. The carry's steep
        # final z-rise to the throw lags the slider by ~67 mm; handing the lagging
        # cup to a plan that assumes it is exactly at the throw makes the cup SLAM
        # up to catch the plan (overshoot 1010 mm, launching the ball ~16 m/s).
        # Re-planning from the achieved state makes cycle 0 continuous — no slam —
        # exactly as every later cycle already is. The throw is slightly low/slow;
        # the per-cycle re-plan self-corrects.
        cup_p = self.cup_world_m()
        cup_v = self.cup_vel_world_m()
        fb0 = plant.get_ball_state(ball=1)
        obs0 = ((np.asarray(fb0.position_mm) / 1000.0,
                 np.asarray(fb0.velocity_mms) / 1000.0)
                if fb0 and not fb0.held else None)
        try:
            plan = self._plan_cycle(cup_p, cup_v, GRAVITY.copy(), obs0)
        except Exception:
            pass

        held_ball, flight_ball = 0, 1
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
