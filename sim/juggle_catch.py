"""Rung 1 — clean single catch (BB-reload), the catch primitive in isolation.

Phase 1 of plans/active/bb-online-juggle-tilt-rearchitecture.md. A focused,
single-ball harness that validates the **catch** primitive on its own, fed a
known-clean BB throw, before any throw or 2-ball confound is introduced:

  catch = slow translate-to-reach (position) + tilt-to-receive (orientation)

* **BB as a ball.** A ball is spawned with an initial ``(pos, vel)`` from a
  configurable launch point (the Ball Butler, placeable anywhere), nominally
  landing at a target in the workspace. The §3 **BB throw noise** perturbs the
  initial conditions, scattering the landing well beyond the tilt lever-arm — so
  the catch must *reach* the observed landing, not the nominal one.
* **Observe.** Every tick the ball position is observed with §3 **tracking
  noise**; a :class:`~sim.juggle_noise.BallisticEstimator` (a known-gravity
  least-squares arc fit) turns the noisy positions into a noise-averaged
  ``(pos, vel)`` and the ballistic touch-down ``(t_td, pos_td, vel_td)``.
* **Translate-to-reach.** The platform centroid is eased toward the predicted
  touch-down xy over the whole flight — a *slow* reach, inside the platform's
  good tracking band (the band limit only bit fast transients).
* **Tilt-to-receive.** The cup is tilted (``tilt_to_receive``) to align its axis
  with the observed arrival velocity (≤12°), so the ball drops straight down the
  cup; the Rung 0 lever-arm compensation (``realize_tilted``) keeps the tilted
  cup opening on the reached xy. The **slider** descends with the ball
  (velocity-matched) — the existing axis-split catch — then settles to rest.

Outputs a :class:`CatchResult` with the in-cup seat offset, for the probe to
characterise across the workspace and under the noise.

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
    GRAVITY, ballistic_touchdown, takeoff_velocity,
)
from sim.juggle_noise import JuggleNoise, NoiseConfig, BallisticEstimator
from sim.juggle_tilt import realize_tilted, tilt_to_receive, MAX_TILT_DEG
from sim.plant.mujoco_plant import MuJoCoPlant

CONTROL_DT = 0.025
WORKSPACE_XY_M = 0.15        # |cup xy - centre| reach bound (matches planner)
SEAT_RADIUS_MM = 40.0        # cup seat radius (mirrors ball.manager.SEAT_RADIUS_M)


# --------------------------------------------------------------------------
# Minimum-jerk (quintic) single-segment planner — one scalar channel from an
# achieved (p, v, a) to a target (p, v, a) over a horizon T. Re-solved each
# tick from the last commanded state so a slow, smooth reach emerges (and the
# horizon shrinks toward the catch). Band-safe: bounded jerk, no step commands.
# --------------------------------------------------------------------------
def _quintic_step(p, v, a, pf, vf, af, T, dt):
    """Advance one quintic channel by ``dt`` toward (pf, vf, af) over horizon T.

    Returns the (p, v, a) at ``dt`` along the minimum-jerk quintic from the
    current (p, v, a) to the boundary (pf, vf, af) at ``T``. ``T`` is floored to
    a few ticks so the end-game stays finite. Vectorises over array channels.
    """
    p = np.asarray(p, float); v = np.asarray(v, float); a = np.asarray(a, float)
    pf = np.asarray(pf, float); vf = np.asarray(vf, float); af = np.asarray(af, float)
    T = max(float(T), 3.0 * dt)
    c0, c1, c2 = p, v, a / 2.0
    h = pf - (c0 + c1 * T + c2 * T ** 2)
    g1 = vf - (c1 + 2.0 * c2 * T)
    g2 = af - 2.0 * c2
    M = np.array([[T ** 3, T ** 4, T ** 5],
                  [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                  [6 * T, 12 * T ** 2, 20 * T ** 3]])
    Minv = np.linalg.inv(M)
    rhs = np.stack([h, g1, g2])                      # (3, nchan)
    c345 = Minv @ rhs                                # (3, nchan)
    c3, c4, c5 = c345[0], c345[1], c345[2]
    t = dt
    p_n = c0 + c1 * t + c2 * t ** 2 + c3 * t ** 3 + c4 * t ** 4 + c5 * t ** 5
    v_n = c1 + 2 * c2 * t + 3 * c3 * t ** 2 + 4 * c4 * t ** 3 + 5 * c5 * t ** 4
    a_n = 2 * c2 + 6 * c3 * t + 12 * c4 * t ** 2 + 20 * c5 * t ** 3
    return p_n, v_n, a_n


@dataclasses.dataclass
class SingleCatchConfig:
    """One BB throw + one catch. SI units unless noted (mm where stated)."""
    # ---- BB throw geometry (the Ball Butler, placeable anywhere) ----
    landing_xy_m: "tuple[float, float]" = (0.0, 0.0)   # nominal touch-down xy
    catch_z_m: float = 0.84                             # cup catch height (world)
    # Launch point relative to the nominal touch-down (origin = target+offset).
    # Default: 150 mm in -x and 250 mm BELOW the catch — the BB lobs up-and-over
    # so the ball arrives descending at a modest (~6 deg) angle (within ≤12°).
    bb_origin_offset_m: "tuple[float, float, float]" = (-0.15, 0.0, -0.25)
    flight_s: float = 0.60                              # launch -> touch-down
    # ---- catch realisation knobs (tuned empirically; see the probe) ----
    # The slider does the vertical descend-with-the-ball (the axis-split catch):
    # the cup HOLDS a short ``ready_lift`` ABOVE catch_z, then descends so it
    # passes through catch_z at touch-down moving down at ``vel_ratio``·|arrival
    # vz| (velocity-matched — keeps the soft contact's closing speed small so the
    # ball does not punch through), then CONSTANT-decelerates to rest over
    # ``seat_decel_time_s``, carrying the seated ball to a co-moving stop. Starting
    # the descent from ABOVE catch_z (``ready_lift``) avoids an up-overshoot (a
    # quintic to catch_z with downward end-velocity from a level hold loops up
    # first). See the Rung 1 catch sweep (tools/probes/juggle_catch_offset.py).
    catch_ready_lift_m: float = 0.06       # hold this far above catch_z first
    catch_slider_vel_ratio: float = 0.95   # cup vz at catch_z = ratio·arrival vz
    catch_start_lead_s: float = 0.16       # begin the descent this far before t_td
    seat_decel_time_s: float = 0.16        # CONSTANT-decel-to-rest duration after t_td
    reach_settle_floor_s: float = 0.10     # fixed receding-horizon for xy/tilt reach
    # The observed apex->catch flight is short (~0.26 s after the n>=4 warm-up
    # gate), so the lateral reach runs on a FIXED ``reach_settle_floor_s`` (0.10 s)
    # receding horizon — each tick eases the cup ~one floor-horizon toward the
    # (frozen) landing, converging over the last few ticks. The XY target is frozen
    # on the FIRST post-warm-up tick (the first n>=4 ballistic fit) so the parked xy
    # doesn't jitter on the post-crossing back-extrapolation. The reach and the
    # slider descend OVERLAP (the descend starts ~one tick later): the flight is too
    # short for the reach to fully settle first, so it is the firm constant-decel
    # seat (below), not a parked-then-descend separation, that carries the off-centre
    # catches. The TILT target is frozen LATER (when descending) so the tilt RAMPS
    # through the seat (see below) instead of being parked fully-tilted early.
    # ---- noise (§3, on by default) ----
    noise: NoiseConfig = dataclasses.field(default_factory=NoiseConfig)
    seed: int = 0
    # ---- run control ----
    settle_ticks: int = 60                 # platform warm-up at home before spawn
    post_td_margin_s: float = 0.40         # extra sim time after t_td to settle


@dataclasses.dataclass
class CatchResult:
    caught: bool                           # seat confirmed at any point
    held_at_end: bool                      # ball still seated at the run end (stayed)
    in_cup_offset_mm: float                # SETTLED ball centre -> hand_opening (end)
    lateral_offset_mm: float               # the xy part of the settled offset
    vertical_offset_mm: float              # the z part of the settled offset
    seat_offset_mm: float                  # transient offset at the seat instant
    catch_time_s: float                    # sim time of the seat (or -1)
    reach_mm: float                        # how far the centroid translated to reach
    catch_tilt_deg: float                  # tilt magnitude used at the catch
    landing_xy_m: "tuple[float, float]"    # ACTUAL (noisy) touch-down xy observed
    nominal_landing_xy_m: "tuple[float, float]"

    @property
    def clean(self) -> bool:
        """A clean catch = seated AND still held AND settled inside the cup."""
        return (self.caught and self.held_at_end
                and self.in_cup_offset_mm <= SEAT_RADIUS_MM)


class SingleCatchRunner:
    def __init__(self, cfg: SingleCatchConfig):
        self.cfg = cfg
        self.plant = MuJoCoPlant(control_dt=CONTROL_DT, contact_carry=True)
        self.site_id = mujoco.mj_name2id(self.plant.model, mujoco.mjtObj.mjOBJ_SITE,
                                         'hand_opening')
        self.noise = JuggleNoise(cfg.noise, seed=cfg.seed)
        self.est = BallisticEstimator(GRAVITY)
        # viewer hooks parity with juggle_online (unused headless)
        self.viewer = None

    def cup_world_m(self):
        return self.plant.data.site_xpos[self.site_id].copy()

    def _ball_true_m(self):
        bs = self.plant.get_ball_state(ball=0)
        return np.asarray(bs.position_mm, float) / 1000.0, bs

    def run(self) -> CatchResult:
        cfg = self.cfg
        plant = self.plant
        plant.reset()
        plant.set_contact_stiffness(False)             # soft contact: catch/carry

        catch_z = cfg.catch_z_m
        nominal_xy = np.array(cfg.landing_xy_m, float)
        target = np.array([nominal_xy[0], nominal_xy[1], catch_z])

        # ---- Warm the platform up at home: cup at the catch height + ready-lift,
        # so the descent starts from ABOVE catch_z (no up-overshoot). ----
        ready_z = catch_z + cfg.catch_ready_lift_m
        pose0, slider0 = realize_tilted(np.array([0.0, 0.0]), ready_z, 0.0, 0.0)
        for _ in range(cfg.settle_ticks):
            plant.command(plant.pose_to_extensions(pose0))
            plant.command_hand(slider0)
            plant.step(CONTROL_DT)

        # ---- Compute + perturb the BB throw, then spawn at the (noisy) apex ----
        # A real Ball Butler launches from across the room; the ball only enters
        # OUR workspace on the DESCENT. Modelling the full rising arc would have
        # the ball pass up THROUGH the waiting cup and collide with it (a sim
        # artefact, target-dependent). So: compute the full notional launch (this
        # is what the §3 BB noise perturbs — 2% of the real throw speed/displacement
        # gives the realistic landing scatter), apply the noise, then SPAWN the
        # perturbed ball at its apex — descending, well above the cup.
        origin = target + np.array(cfg.bb_origin_offset_m, float)
        v0 = takeoff_velocity(origin, target, cfg.flight_s)
        displacement_mm = (target - origin) * 1000.0
        pos_n_mm, vel_n_mms = self.noise.perturb_throw(
            origin * 1000.0, v0 * 1000.0, displacement_mm)
        origin_n = pos_n_mm / 1000.0
        v0_n = vel_n_mms / 1000.0
        g = -GRAVITY[2]
        t_apex = v0_n[2] / g if v0_n[2] > 0 else 0.0
        apex_pos = origin_n + v0_n * t_apex + 0.5 * GRAVITY * t_apex ** 2
        apex_vel = v0_n + GRAVITY * t_apex                 # vz ≈ 0 at apex
        plant.spawn_ball(apex_pos * 1000.0, apex_vel * 1000.0, ball=0)

        # ---- Commanded cup state (the reach is planned from this) ----
        cmd_xy = np.array([0.0, 0.0]); cmd_vxy = np.zeros(2); cmd_axy = np.zeros(2)
        cmd_tilt = np.zeros(2); cmd_vtilt = np.zeros(2); cmd_atilt = np.zeros(2)
        cmd_z = ready_z; cmd_vz = 0.0; cmd_az = 0.0

        t0 = float(plant.data.time)
        n_ticks = int(round((cfg.flight_s + cfg.post_td_margin_s) / CONTROL_DT))
        caught = False
        catch_time = -1.0
        seat_offset = None                 # transient offset at the seat instant
        last_pos_td = target.copy()
        last_tilt = np.zeros(2)
        # Once the descent begins, FREEZE the reach (xy + tilt) target: after
        # touch-down ballistic_touchdown extrapolates the (now-passed) crossing
        # backwards and would jitter the converged cup. The prediction is mature
        # by the descent start, so the frozen target seats cleanly.
        frozen_xy = None
        frozen_tilt = None
        settle_v0 = None                   # cup vz captured at the catch_z crossing

        for _k in range(n_ticks):
            t_rel = float(plant.data.time) - t0
            # --- observe + estimate ---
            true_m, bs = self._ball_true_m()
            obs_m = self.noise.observe(true_m * 1000.0) / 1000.0
            self.est.add(t_rel, obs_m)
            pos_e, vel_e = self.est.estimate()
            # ballistic_touchdown returns the time-from-NOW to touch-down (the
            # estimate is at the latest observation, i.e. now), so t_td IS the
            # remaining flight — do NOT subtract t_rel.
            t_td, pos_td, vel_td = ballistic_touchdown(pos_e, vel_e, catch_z)
            t_remaining = t_td
            # Clamp the predicted landing to the reachable workspace so a noisy
            # early estimate can't fling the platform to a leg-saturating reach.
            pos_td = np.array(pos_td, float)
            pos_td[:2] = np.clip(pos_td[:2], -WORKSPACE_XY_M, WORKSPACE_XY_M)
            tilt_rx, tilt_ry = tilt_to_receive(vel_td, MAX_TILT_DEG)

            # --- warm-up guard: hold at home until the arc is well-observed ---
            # The first few samples give a degenerate (n<3) or noisy fit; reaching
            # on it is what flung the platform. Hold home, keep observing.
            if self.est.n < 4:
                pose, slider = realize_tilted(cmd_xy, cmd_z, cmd_tilt[0], cmd_tilt[1])
                plant.command(plant.pose_to_extensions(pose))
                plant.step(
                    CONTROL_DT,
                    hand_cmd_fn=lambda _t, s=slider: s,
                    plat_cmd_fn=lambda _t, p=pose: plant.pose_to_extensions(p))
                continue

            descending = t_remaining <= cfg.catch_start_lead_s
            # Freeze the XY reach target on the first post-warm-up tick (the first
            # n>=4 ballistic fit) so the parked xy is stable and doesn't jitter on
            # the post-crossing back-extrapolation. The TILT target is frozen later
            # (when descending), so the tilt RAMPS THROUGH the seat (see below)
            # rather than being held fully tilted before the ball arrives.
            if frozen_xy is None:
                frozen_xy = pos_td[:2].copy()
            if descending and frozen_tilt is None:
                frozen_tilt = np.array([tilt_rx, tilt_ry])
            reach_xy = frozen_xy if frozen_xy is not None else pos_td[:2]
            reach_tilt = frozen_tilt if frozen_tilt is not None else np.array([tilt_rx, tilt_ry])
            last_pos_td = np.array([reach_xy[0], reach_xy[1], catch_z])
            last_tilt = reach_tilt

            # --- plan the cup target for the next tick ---
            # XY reach: ease toward the frozen landing on a FIXED receding horizon
            # (``reach_settle_floor_s``). The observed flight is too short (~0.26 s)
            # for the reach to fully settle before the slider descend begins, so they
            # overlap; the cup converges onto the frozen xy over the last few ticks
            # and the firm constant-decel seat (below) carries the off-centre catches.
            horizon_reach = cfg.reach_settle_floor_s
            cmd_xy, cmd_vxy, cmd_axy = _quintic_step(
                cmd_xy, cmd_vxy, cmd_axy, reach_xy, np.zeros(2), np.zeros(2),
                horizon_reach, CONTROL_DT)
            # TILT: ramp to the collinear receive tilt completing AT t_td (not
            # before). A fully-tilted cup parked early presents an asymmetric rim
            # the descending ball glances off; ramping the tilt in keeps the cup
            # nearer level when the ball first enters, then aligns as it seats.
            horizon_tilt = max(t_remaining, cfg.reach_settle_floor_s)
            cmd_tilt, cmd_vtilt, cmd_atilt = _quintic_step(
                cmd_tilt, cmd_vtilt, cmd_atilt, reach_tilt,
                np.zeros(2), np.zeros(2), horizon_tilt, CONTROL_DT)
            # vertical (slider): the axis-split descend-with-the-ball.
            #   hold at ready_z -> pass catch_z velocity-matched -> CONSTANT-decel
            #   to rest. The settle is a CONSTANT deceleration (not a min-jerk to
            #   rest): a min-jerk decel-to-rest has a deceleration PEAK ~1.9x its
            #   average in the middle of the stroke, and that force spike ejects the
            #   ball UP off the cup (a soft-contact bounce) — the dominant catch
            #   failure under the §3 noise. A constant deceleration has no peak, so
            #   the contact force is steady and the ball rides the cup smoothly to
            #   rest. (See the Rung 1 catch sweep, tools/probes/juggle_catch_offset.py.)
            if t_remaining > 0.0:
                if not descending:
                    z_t, vz_t = ready_z, 0.0
                    hz = max(t_remaining - cfg.catch_start_lead_s, 3.0 * CONTROL_DT)
                else:
                    # APPROACH — ease to catch_z arriving velocity-matched at t_td.
                    z_t = catch_z
                    vz_t = cfg.catch_slider_vel_ratio * float(vel_td[2])
                    hz = max(t_remaining, 3.0 * CONTROL_DT)
                (cmd_z,), (cmd_vz,), (cmd_az,) = _quintic_step(
                    [cmd_z], [cmd_vz], [cmd_az], [z_t], [vz_t], [0.0], hz, CONTROL_DT)
            else:
                # Constant-deceleration settle: ramp the (downward) cup velocity
                # linearly to zero over ``seat_decel_time_s``, then hold at rest.
                if settle_v0 is None:
                    settle_v0 = min(cmd_vz, -1e-6)        # cup vz at the crossing
                a_dec = -settle_v0 / cfg.seat_decel_time_s   # >0 (upward decel)
                if cmd_vz < -1e-4:
                    new_vz = min(cmd_vz + a_dec * CONTROL_DT, 0.0)
                    cmd_z = cmd_z + 0.5 * (cmd_vz + new_vz) * CONTROL_DT
                    cmd_vz, cmd_az = new_vz, a_dec
                else:
                    cmd_vz, cmd_az = 0.0, 0.0              # at rest — hold

            pose, slider = realize_tilted(cmd_xy, cmd_z, cmd_tilt[0], cmd_tilt[1])
            plant.command(plant.pose_to_extensions(pose))
            plant.step(
                CONTROL_DT,
                hand_cmd_fn=lambda _t, s=slider: s,
                plat_cmd_fn=lambda _t, p=pose: plant.pose_to_extensions(p))

            if not caught and plant.check_and_capture(ball=0):
                caught = True
                catch_time = t_rel
                ball_m = self._ball_true_m()[0]
                seat_offset = float(np.linalg.norm((ball_m - self.cup_world_m()) * 1000.0))

        # ---- Settled offset + stay-seated check at the run end ----
        final = self.plant.get_ball_state(ball=0)
        ball_m = np.asarray(final.position_mm, float) / 1000.0
        settled = (ball_m - self.cup_world_m()) * 1000.0
        held_at_end = bool(final.held)
        if seat_offset is None:
            seat_offset = float(np.linalg.norm(settled))

        reach_mm = float(np.linalg.norm(last_pos_td[:2]) * 1000.0)
        return CatchResult(
            caught=caught,
            held_at_end=held_at_end,
            in_cup_offset_mm=float(np.linalg.norm(settled)),
            lateral_offset_mm=float(np.linalg.norm(settled[:2])),
            vertical_offset_mm=float(settled[2]),
            seat_offset_mm=seat_offset,
            catch_time_s=catch_time,
            reach_mm=reach_mm,
            catch_tilt_deg=float(np.degrees(np.linalg.norm(last_tilt))),
            landing_xy_m=(float(last_pos_td[0]), float(last_pos_td[1])),
            nominal_landing_xy_m=(float(nominal_xy[0]), float(nominal_xy[1])),
        )


def run_single_catch(cfg: "SingleCatchConfig | None" = None) -> CatchResult:
    """Run one BB throw + catch headless; return the :class:`CatchResult`."""
    return SingleCatchRunner(cfg or SingleCatchConfig()).run()
