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

# Single path bootstrap (repo root, ros_ws pkg, config/generated);
# see sim/_paths.py.  Runnable entry scripts only — library modules under
# sim/ never touch sys.path.
_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
from sim._paths import bootstrap_paths  # noqa: E402
bootstrap_paths()

import mujoco

from sim.juggle_planner.juggle_planner import (
    GRAVITY, ballistic_touchdown, takeoff_velocity,
)
from sim.juggle_noise import JuggleNoise, NoiseConfig, BallisticEstimator
from sim.juggle_tilt import realize_tilted, tilt_to_receive, MAX_TILT_DEG
from sim.plant.mujoco_plant import MuJoCoPlant

CONTROL_DT = 0.025
WORKSPACE_XY_M = 0.15        # |cup xy - centre| reach bound (matches planner)
SEAT_RADIUS_MM = 40.0        # cup seat radius (mirrors ball.manager.SEAT_RADIUS_M)
# Momentum-budget deceleration (× g) for the continuous velocity-matched catch:
# once the co-moving cup receives the ball it is arrested to rest over
# d = v²/(2·a). At 6 g even the fast (~4.9 m/s) real-BB arrival is arrested in
# ~60 mm — trivial for a hand with tens of g of authority — so the slider stays
# off both clamps (no ceiling slam, no floor slam) under the CONTINUOUS command
# (a fixed-TIME constant-decel would slam the floor on a fast arrival once the cup
# actually tracks it). See logbook/2026-07-03-catch-control-formulation-design-basis.md.
CATCH_ARREST_ACCEL_G = 6.0


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


class _ContinuousQuintic:
    """A minimum-jerk quintic ``(p, v, a) -> (pf, vf, af)`` over horizon ``T``,
    sampleable at ANY sub-tick time — the mechanism behind the co-design catch.

    The sim already steps the hand at 1 kHz: ``MuJoCoPlant.step`` samples
    ``hand_cmd_fn(t)`` EVERY 1 ms substep. The old catch handed it a CONSTANT
    per-40 Hz-tick position (the end-of-tick setpoint), so the ultra-stiff hand
    actuator jumped to that setpoint and SETTLED within the tick — near-zero
    velocity at the sample instant, unable to co-move with the descending ball.
    Commanding this quintic sampled at each substep's sim time instead makes the
    cup track the planned VELOCITY continuously, so it receives the ball on a
    *descending* cup. Same per-tick displacement, but a smooth velocity vs a
    jump-and-settle — the design-basis A/B measured **-2700 mm/s (continuous) vs
    -35 mm/s (constant)** at a -2.7 m/s target
    (``logbook/2026-07-03-catch-control-formulation-design-basis.md``).

    Distinct from :func:`_quintic_step` (which returns only the end-of-tick
    state): this keeps the coefficients so :meth:`pos` can be evaluated at any
    intra-tick time and :meth:`state` gives the next tick's start. Vectorises over
    channels. ``T`` is floored to ``3*dt`` (matching :func:`_quintic_step`) so the
    receding-horizon end-game stays finite.
    """

    def __init__(self, p, v, a, pf, vf, af, T, dt):
        p = np.atleast_1d(np.asarray(p, float)); v = np.atleast_1d(np.asarray(v, float))
        a = np.atleast_1d(np.asarray(a, float)); pf = np.atleast_1d(np.asarray(pf, float))
        vf = np.atleast_1d(np.asarray(vf, float)); af = np.atleast_1d(np.asarray(af, float))
        self.dt = float(dt)
        self.T = max(float(T), 3.0 * self.dt)
        T = self.T
        self.c0, self.c1, self.c2 = p, v, a / 2.0
        h = pf - (self.c0 + self.c1 * T + self.c2 * T ** 2)
        g1 = vf - (self.c1 + 2.0 * self.c2 * T)
        g2 = af - 2.0 * self.c2
        M = np.array([[T ** 3, T ** 4, T ** 5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        c345 = np.linalg.solve(M, np.stack([h, g1, g2]))
        self.c3, self.c4, self.c5 = c345[0], c345[1], c345[2]

    def pos(self, t):
        """Commanded position at sub-tick time ``t`` (clipped to ``[0, T]``)."""
        t = float(np.clip(t, 0.0, self.T))
        return (self.c0 + self.c1 * t + self.c2 * t ** 2 + self.c3 * t ** 3
                + self.c4 * t ** 4 + self.c5 * t ** 5)

    def state(self, t=None):
        """``(p, v, a)`` at time ``t`` (default ``dt``) — the next tick's start."""
        t = self.dt if t is None else t
        t = float(np.clip(t, 0.0, self.T))
        p = self.pos(t)
        v = (self.c1 + 2 * self.c2 * t + 3 * self.c3 * t ** 2
             + 4 * self.c4 * t ** 3 + 5 * self.c5 * t ** 4)
        a = 2 * self.c2 + 6 * self.c3 * t + 12 * self.c4 * t ** 2 + 20 * self.c5 * t ** 3
        return p, v, a


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
    # ---- optional REAL Ball Butler throw source (fast-catch fidelity) ----
    # By default the incoming ball is a SYNTHETIC gentle lob (``takeoff_velocity``
    # over ``flight_s`` from ``bb_origin_offset_m``), arriving ~2.5 m/s. Set
    # ``use_real_bb`` to spawn from the REAL ``BallButlerSim`` kinematics instead —
    # a fast, flat throw (~4.9 m/s vz at catch_z from the demo BB placement), the
    # arrival the hardware actually delivers. The §3 BB-throw noise + the apex-
    # spawn are applied identically to both sources. See the fast-catch probe
    # (tools/probes/juggle_fastcatch.py) and
    # logbook/2026-07-02-fast-catch-fidelity.md.
    use_real_bb: bool = False
    bb_position_mm: "tuple[float, float, float]" = (-872.0, -630.0, 1430.0)
    bb_yaw_offset_rad: "float | None" = None   # None -> aim from position at origin
    # ---- optional MANUAL launch (operator-driven, e.g. sim/juggle_bb_catch.py) ----
    # When set to a ``(pos_mm(3), vel_mms(3))`` pair the incoming ball spawns from
    # THAT operator-chosen release state instead of the synthetic/real-BB branch —
    # the landing is wherever the ballistics take it (an on-target preset, or a
    # deliberate near-miss). The SAME §3 perturb + apex-spawn are applied (identical
    # perturb/apex path), so the catch pipeline is unchanged. Default None -> the
    # synthetic / real-BB branches run byte-for-byte unchanged.
    manual_launch_mm: "tuple | None" = None
    # ---- catch realisation knobs (tuned empirically; see the probe) ----
    # The slider does the vertical descend-with-the-ball (the axis-split catch),
    # PHASE-MATCHED to the arrival (2026-07-02 fast-catch fidelity). From a
    # ``ready_lift`` ABOVE catch_z, the cup is driven so it arrives at catch_z
    # MOVING DOWN at ``vel_ratio``·|estimated arrival vz| EXACTLY at the predicted
    # touch-down ``t_td`` — i.e. the cup is CO-MOVING with the ball, not
    # holding-high-then-dropping-late, when the ball lands. This is the crux for a
    # FAST arrival (a real Ball Butler ~4.9 m/s): the old hold-high-then-descend
    # schedule reached catch_z too late and the fast ball punched through.
    #
    # The descent is commanded CONTINUOUSLY at the sub-tick rate (a per-tick
    # :class:`_ContinuousQuintic` sampled by ``hand_cmd_fn``/``plat_cmd_fn`` every
    # 1 ms substep) rather than as a CONSTANT per-40 Hz-tick position — otherwise
    # the ultra-stiff hand actuator jump-and-settles and can't actually co-move
    # (design basis 2026-07-03). Once the TRUE ball reaches catch_z the cup is
    # arrested to rest over a MOMENTUM BUDGET d = v²/(2·a) at
    # :data:`CATCH_ARREST_ACCEL_G` — a min-jerk quintic re-solved each tick, so
    # only its low-jerk onset executes (no mid-stroke deceleration peak to eject the
    # ball); and, unlike the old fixed-TIME decel, its travel is bounded so a fast
    # arrival can't slam the slider floor once the cup honestly tracks it.
    # ``ready_lift`` (0.15) gives room to build the matched descent speed;
    # ``vel_ratio`` 0.55 is the sweet spot (0.5-0.6). See the fast-catch probe
    # (tools/probes/juggle_fastcatch.py).
    catch_ready_lift_m: float = 0.15       # hold this far above catch_z first
    catch_slider_vel_ratio: float = 0.55   # cup vz at catch_z = ratio·arrival vz
    catch_start_lead_s: float = 0.16       # tilt-freeze lead before t_td
    seat_decel_time_s: float = 0.16        # RETAINED for config compat; the catch now
                                           # arrests via CATCH_ARREST_ACCEL_G (see above)
    reach_settle_floor_s: float = 0.10     # fixed receding-horizon for xy/tilt reach
    # The observed apex->catch flight is short (~0.26 s after the n>=4 warm-up
    # gate), so the lateral reach runs on a FIXED ``reach_settle_floor_s`` (0.10 s)
    # receding horizon — each tick eases the cup ~one floor-horizon toward the
    # (frozen) landing, converging over the last few ticks. The XY target is frozen
    # on the FIRST post-warm-up tick (the first n>=4 ballistic fit) so the parked xy
    # doesn't jitter on the post-crossing back-extrapolation. The reach and the
    # slider descend OVERLAP (the descend starts ~one tick later): the flight is too
    # short for the reach to fully settle first, so it is the momentum-budget seat
    # (below), not a parked-then-descend separation, that carries the off-centre
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

    def _spawn_incoming(self, target):
        """Compute + perturb the incoming throw, then spawn ball 0 at its apex.

        A real Ball Butler launches from across the room; the ball only enters
        OUR workspace on the DESCENT. Modelling the full rising arc would have the
        ball pass up THROUGH the waiting cup and collide with it (a sim artefact,
        target-dependent). So: compute the full notional launch, apply the §3 BB
        noise (2% of the throw speed/displacement — the realistic landing scatter),
        then SPAWN the perturbed ball at its apex — descending, well above the cup.

        Two throw sources, selected by ``cfg.use_real_bb`` (both perturbed +
        apex-spawned identically):

        * **synthetic** (default): a gentle ballistic lob via ``takeoff_velocity``
          over ``flight_s`` from ``bb_origin_offset_m`` — arrives ~2.5 m/s.
        * **real BB**: the actual ``BallButlerSim`` kinematics aimed at ``target``
          — a fast, flat throw (~4.9 m/s vz at catch_z from the demo placement),
          the arrival the hardware delivers. Imported lazily so the default path
          keeps its import surface unchanged.
        """
        cfg = self.cfg
        if cfg.manual_launch_mm is not None:
            # Operator-chosen (release_pos, release_vel): the manual-launch tool.
            # Flows through the identical perturb + apex-spawn path below.
            pos_mm = np.asarray(cfg.manual_launch_mm[0], float).reshape(3)
            vel_mms = np.asarray(cfg.manual_launch_mm[1], float).reshape(3)
            target_mm = target * 1000.0
            displacement_mm = target_mm - pos_mm
        elif cfg.use_real_bb:
            from sim.ball_butler.sim import BallButlerSim
            pos = np.asarray(cfg.bb_position_mm, float)
            yaw = (cfg.bb_yaw_offset_rad if cfg.bb_yaw_offset_rad is not None
                   else float(np.arctan2(-pos[1], -pos[0])))   # aim from BB at origin
            bb = BallButlerSim.from_hardware_config(pos, yaw)
            target_mm = target * 1000.0
            rel_pos_mm, rel_vel_mms, _ = bb.compute_release_state(target_mm)
            pos_mm = np.asarray(rel_pos_mm, float)
            vel_mms = np.asarray(rel_vel_mms, float)
            displacement_mm = target_mm - pos_mm
        else:
            origin = target + np.array(cfg.bb_origin_offset_m, float)
            v0 = takeoff_velocity(origin, target, cfg.flight_s)
            pos_mm = origin * 1000.0
            vel_mms = v0 * 1000.0
            displacement_mm = (target - origin) * 1000.0

        pos_n_mm, vel_n_mms = self.noise.perturb_throw(pos_mm, vel_mms, displacement_mm)
        origin_n = pos_n_mm / 1000.0
        v0_n = vel_n_mms / 1000.0
        g = -GRAVITY[2]
        t_apex = v0_n[2] / g if v0_n[2] > 0 else 0.0
        apex_pos = origin_n + v0_n * t_apex + 0.5 * GRAVITY * t_apex ** 2
        apex_vel = v0_n + GRAVITY * t_apex                 # vz ≈ 0 at apex
        self.plant.spawn_ball(apex_pos * 1000.0, apex_vel * 1000.0, ball=0)

    def run(self) -> CatchResult:
        cfg = self.cfg
        plant = self.plant
        plant.reset()
        # The catch runs entirely in the "soft" (now FIRM — see
        # ball.manager.CONTACT_SOFT_*) contact regime. ``set_contact_stiffness`` is
        # idempotent and the manager initialises to "soft", so a plain
        # ``set_contact_stiffness(False)`` is a NO-OP that leaves the ball+cup geoms
        # at their MJCF-authored (0.05, 2.0) default — the OLD 50 ms overdamped soft
        # that lets a fast (~4.9 m/s) arrival punch straight through the cup. This
        # is a CATCH-ONLY harness (no throw stroke ever flips the contact to stiff
        # and back), so nothing else writes the module constants. Toggle
        # stiff->soft once so the firm CONTACT_SOFT_* values are actually applied to
        # the geoms before the catch. See logbook/2026-07-02-fast-catch-fidelity.md.
        plant.set_contact_stiffness(True)
        plant.set_contact_stiffness(False)             # firm contact: catch/carry

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
        self._spawn_incoming(target)

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
        settling = False                   # latched once the TRUE ball reaches catch_z
        a_budget = CATCH_ARREST_ACCEL_G * (-GRAVITY[2])   # momentum-budget arrest decel

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
            v_arr = abs(float(vel_td[2]))         # estimated |arrival vz| at catch_z

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

            # --- plan the cup target for THIS tick, commanded CONTINUOUSLY ---
            # The sim steps the hand at 1 kHz; commanding a CONSTANT per-40 Hz-tick
            # position snaps the ultra-stiff actuator to the end-of-tick setpoint
            # and lets it settle (near-zero velocity at the sample instant), so the
            # cup cannot co-move with the ball. Build a per-tick quintic and SAMPLE
            # it every substep (hand_cmd_fn/plat_cmd_fn) instead, so the cup tracks
            # the planned VELOCITY — it receives the ball on a DESCENDING cup. See
            # logbook/2026-07-03-catch-control-formulation-design-basis.md.
            #
            # XY reach: ease toward the frozen landing on a FIXED receding horizon
            # (``reach_settle_floor_s``). The observed flight is too short (~0.26 s)
            # for the reach to fully settle before the slider descend begins, so they
            # overlap; the cup converges onto the frozen xy over the last few ticks
            # and the momentum-budget seat (below) carries the off-centre catches.
            seg_xy = _ContinuousQuintic(cmd_xy, cmd_vxy, cmd_axy, reach_xy,
                                        np.zeros(2), np.zeros(2),
                                        cfg.reach_settle_floor_s, CONTROL_DT)
            # TILT: ramp to the collinear receive tilt completing AT t_td (not
            # before). A fully-tilted cup parked early presents an asymmetric rim
            # the descending ball glances off; ramping the tilt in keeps the cup
            # nearer level when the ball first enters, then aligns as it seats.
            horizon_tilt = max(t_remaining, cfg.reach_settle_floor_s)
            seg_tilt = _ContinuousQuintic(cmd_tilt, cmd_vtilt, cmd_atilt, reach_tilt,
                                          np.zeros(2), np.zeros(2), horizon_tilt,
                                          CONTROL_DT)
            # VERTICAL (slider): the PHASE-MATCHED descend-with-the-ball. Drive the
            # cup to arrive at catch_z MOVING DOWN at ``vel_ratio``·|arrival vz| at
            # the predicted touch-down (co-moving with the ball, not holding high
            # then dropping late). Once the TRUE ball reaches catch_z (a determin-
            # istic crossing; LATCHED so an estimate jitter can't unlatch it),
            # arrest the co-moving cup to rest over a MOMENTUM BUDGET
            # d = v²/(2·a) at ``a_budget``. The old fixed-TIME constant-decel over
            # ``seat_decel_time_s`` travelled ~v·t/2, which under the now-honoured
            # continuous command would slam the slider FLOOR on a fast (~4.9 m/s)
            # arrival; the momentum budget bounds the arrest distance (~60 mm). The
            # arrest itself is a min-jerk quintic re-solved each tick, so only its
            # low-jerk onset executes — no mid-stroke deceleration peak ejects the
            # ball (the anti-ejection property the old constant-decel seat gave,
            # now via a smooth bounded-jerk onset).
            ball_z = true_m[2]
            if not settling and ball_z > catch_z + 0.002 and t_remaining > 1e-3:
                # APPROACH — arrive at catch_z moving down at ratio·|arrival vz| @ t_td.
                tgt_vz = -cfg.catch_slider_vel_ratio * v_arr
                hz = max(t_remaining, 3.0 * CONTROL_DT)
                seg_z = _ContinuousQuintic([cmd_z], [cmd_vz], [cmd_az], [catch_z],
                                           [tgt_vz], [0.0], hz, CONTROL_DT)
            else:
                # SETTLE — the ball reached catch_z: momentum-budget-decel the
                # co-moving cup to rest, then hold. Latched (``settling``) so we
                # never re-enter the approach on a post-crossing estimate wobble.
                settling = True
                if cmd_vz < -1e-4:
                    t_stop = -cmd_vz / a_budget
                    z_stop = cmd_z + 0.5 * cmd_vz * t_stop
                    seg_z = _ContinuousQuintic([cmd_z], [cmd_vz], [cmd_az], [z_stop],
                                               [0.0], [0.0], max(t_stop, 3.0 * CONTROL_DT),
                                               CONTROL_DT)
                else:
                    seg_z = _ContinuousQuintic([cmd_z], [0.0], [0.0], [cmd_z], [0.0],
                                               [0.0], 3.0 * CONTROL_DT, CONTROL_DT)

            t_tick0 = float(plant.data.time)

            def cmd_fn(t, sx=seg_xy, sz=seg_z, st=seg_tilt, t0=t_tick0):
                tau = t - t0
                xy = sx.pos(tau); z = float(sz.pos(tau)[0]); ti = st.pos(tau)
                return realize_tilted(xy, z, float(ti[0]), float(ti[1]))

            pose_now, _ = cmd_fn(t_tick0)
            plant.command(plant.pose_to_extensions(pose_now))
            plant.step(CONTROL_DT, hand_cmd_fn=lambda t, f=cmd_fn: f(t)[1],
                       plat_cmd_fn=lambda t, f=cmd_fn: plant.pose_to_extensions(f(t)[0]))

            cmd_xy, cmd_vxy, cmd_axy = seg_xy.state()
            (zc, vc, ac) = seg_z.state()
            cmd_z, cmd_vz, cmd_az = float(zc[0]), float(vc[0]), float(ac[0])
            cmd_tilt, cmd_vtilt, cmd_atilt = seg_tilt.state()

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
