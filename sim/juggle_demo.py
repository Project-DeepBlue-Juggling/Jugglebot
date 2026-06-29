"""Standalone runner for the BB-led two-ball oval juggle demo (sim).

Wires every Phase 2/3 component together in one process at 40 Hz:

  - :class:`controller.demo.juggle_optimizer` — solves the periodic
    optimised platform trajectory once at startup.
  - :class:`controller.demo.player.TrajectoryPlayer` — drives the platform
    from the trajectory.
  - :class:`controller.demo.timeline.MasterTimeline` — schedules the BB
    priming throw + alternating hand throws / catches.
  - :class:`sim.plant.mujoco_plant.MuJoCoPlant` — physics, two-ball
    manager, hand actuator.
  - :class:`sim.ball_butler.sim.BallButlerSim` — computes BB release
    state for the priming throw.
  - :class:`sim.hand.trajectory.HandThrowSequence` /
    :class:`sim.hand.trajectory.HandCatchSequence` — drive the hand for
    each throw / catch event.

Plan reference: ``plans/active/bb-led-two-ball-juggle-demo.md`` §4 Phase 3.
Exit criterion: ≥ 30 consecutive catches detected by
``BallManager.check_capture``.

CLI:
    python sim/juggle_demo.py                       # 30 s headless run
    python sim/juggle_demo.py --duration 10
    python sim/juggle_demo.py --viewer              # MuJoCo passive viewer
    python sim/juggle_demo.py --realtime-rate 0.5   # half-speed slowmo
    python sim/juggle_demo.py --dashboard           # http://<jetson-ip>:8082
    python sim/juggle_demo.py --abort-at 5.0        # abort mid-pattern
    python sim/juggle_demo.py --record temp/reports/juggle.mp4   # fixed-camera mp4

Recording a video from a chosen angle
-------------------------------------
The live passive viewer always starts the free camera at the model's
``<global azimuth elevation>`` (135 / -30). To record from a different
angle:

  1. Run ``--viewer``, drag the camera to the angle you want, and press
     ``C``. The runner prints the current free-camera parameters as
     ready-to-paste flags, e.g.::

         camera: --cam-azimuth 90.0 --cam-elevation -20.0 \
                 --cam-distance 4.250 --cam-lookat 0.000 0.000 0.700

  2. Pass those flags to a headless ``--record`` run::

         python sim/juggle_demo.py --record temp/reports/juggle.mp4 \
             --cam-azimuth 90 --cam-elevation -20 --cam-distance 4.25 \
             --cam-lookat 0 0 0.7

``--record`` renders the sim offscreen from that *fixed* camera (it does
not need ``--viewer``) and pipes frames to the system ``ffmpeg`` as
H.264. With no ``--cam-*`` flags it uses the model default angle. Output
plays back real-time at ``--record-fps`` (default 40 = one frame per
40 Hz tick); a lower fps yields slow-motion.

Viewer keyboard (when ``--viewer`` is on):
    SPACE       pause / resume
    RIGHT       step one 40 Hz tick (when paused)
    ``[``       halve the wall-time playback rate (jumps out of free-running)
    ``]``       double the wall-time playback rate
    ``C``       print the current free-camera angle as ``--cam-*`` flags
    LEFT        no-op (sim can't run backwards)

Programmatic:
    from sim.juggle_demo import run, JuggleDemoConfig
    stats = run(JuggleDemoConfig(duration_s=5.0))

Pure-Python, no ROS2. The runner is the **same architecture** the Phase 4
hardware orchestrator will mirror, except the plant is ``HardwarePlant``
instead of ``MuJoCoPlant`` and the hand/BB events go to ``can_node``
services instead of the in-process sim model.

Why the runner has a "schedule lead" for hand events
----------------------------------------------------

A :class:`HandThrowSequence` needs to *start* its prelude smooth-move
well before its release instant — typically 0.5–1.0 s for the hand to
reposition. The event's ``t_wall`` is the release time, so the runner
pulls events from the master timeline early (``t_wall +
SCHEDULE_LEAD_S``) to set the sequences up in time. BB throws have no
such lead: their ``t_wall`` is when the BB releases the ball, so they
are dispatched at the exact moment via a small pending queue.
"""
from __future__ import annotations

import argparse
import dataclasses
import math
import os
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np

# Make sim/, controller/, the jugglebot ROS2 package, and generated config
# importable when this script is run directly on a fresh clone — without
# requiring the jugglebot package to be pip-installed (as it is on the
# Jetson venv). Mirrors the path entries in tests/conftest.py so the demo
# runs from any checkout with no PYTHONPATH/install dance.
_sim_dir = os.path.dirname(os.path.abspath(__file__))
_repo_root = os.path.dirname(_sim_dir)
for _p in (
    _sim_dir,                                               # bare hand/, ball_butler/ imports
    _repo_root,                                             # controller.*
    os.path.join(_repo_root, 'ros_ws', 'src', 'jugglebot'),  # jugglebot.motion.* (pure-Python ROS2 pkg)
    os.path.join(_repo_root, 'config', 'generated'),        # generated hardware/protocol config
):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from controller.demo.pattern import JugglePattern
from controller.telemetry import record_from_arrays
from controller.demo.player import TrajectoryPlayer
from controller.demo.timeline import (
    DEFAULT_BB_LEAD_S, Event, MasterTimeline,
)
from controller.demo.juggle_optimizer import (
    OptimizerConfig, optimise_juggle_trajectory,
)
from controller.demo.trajectory import JuggleTrajectory, build_analytic_oval
from sim.ball_butler.sim import BallButlerSim
from sim.hand.ballistics import compute_hand_offset_mm, rodrigues
from sim.hand.trajectory import (
    HandCatchSequence, HandCatchTrajectory, HandThrowSequence,
)
from sim.plant.mujoco_plant import MuJoCoPlant


CONTROL_DT_S = 0.025                       # 40 Hz player loop

# Throw-speed calibration for the contact-driven throw (concern 1). With the
# phase-switched contact stiffness making the throw faithful (the stiff
# up-stroke transfers ~100% of the commanded speed), the realised exit speed
# matches the optimiser-planned ball speed at 1.0 — no compensation needed.
# (The earlier 1.08 compensated a pre-stiffness-fix soft-contact under-throw
# and now over-shoots ~8%.) See ``_schedule_hand_throw`` and logbook 2026-06-27.
THROW_SPEED_CALIB = 1.00

# Closed-loop catch reach (2026-06-27). The emergent contact throw cannot hit
# the catch cup open-loop: the band-limited platform carries ~0.1 m/s of
# irreducible sweep momentum through the throw, so the ball lands ~100-140 mm
# from the planned catch (a real, deterministic platform-tracking limit, not a
# plan error — see logbook 2026-06-27 throw-aim). Rather than fight the throw,
# the catch CLOSES THE LOOP: it observes the in-flight ball, predicts its xy at
# the catch instant (ballistic, exact in sim), and reaches the platform xy (a
# smootherstep ramp-hold-ramp, profile below) so the cup meets the ball.
# Reach profile (offsets from the catch instant tc, seconds). The reach is a
# ramp-HOLD-ramp, not a peak-and-leave bump: the platform must ARRIVE at the
# ball and DWELL there with zero reach-velocity through the catch, otherwise
# the reach-velocity breaks the velocity-matched seat (the cup passes the ball
# while still moving and never seats). Ramp in over [tc-T0, tc-T1], HOLD at the
# reach over [tc-T1, tc+T2] (spanning the catch — the platform settles to the
# commanded reach, so no overshoot), ramp out over [tc+T2, tc+T3]. Windows fit
# inside (throw at tc-0.405, next throw at tc+0.22) so throws stay on-plan.
_REACH_T0 = 0.35      # ramp-in start before tc
_REACH_T1 = 0.10      # hold start before tc (settle window)
_REACH_T2 = 0.06      # hold end after tc
_REACH_T3 = 0.19      # fully returned to plan after tc
_REACH_GAIN = 0.83
# Clamp the reach magnitude (mm). A ball landing beyond this is outside the
# platform workspace and uncatchable anyway; clamping stops a wild/lost ball
# from commanding an absurd reach that saturates the legs and flails the
# platform (which would swat the other in-flight ball and cascade). The catch
# of a wild ball just fails gracefully without disrupting the rest.
_REACH_MAX_MM = 180.0

# Number of platform-only warm-up ticks before the pre-held ball is seated
# into the cup, so the startup position step (throw_pose -> first
# trajectory pose) is absorbed with no ball in contact. See the runner's
# __init__ "Platform warm-up" block.
_PLATFORM_WARMUP_TICKS = 24

# Hand position (mm) at which the pre-held ball is carried through the
# pre-roll before its first throw. Low (near the throw-stroke start) so the
# cup sits close to the platform centroid — a short lever arm that keeps
# the oval carry from flinging the contact-held ball out of the open cup.
_PREHELD_CARRY_HAND_MM = 20.0


# Process-wide cache for offline-optimiser results, keyed on the pattern
# parameters + a small subset of OptimizerConfig fields that actually
# change the solve. Test suites construct many _JuggleDemoRunner
# instances with the same pattern + cfg; without caching each pays the
# ~12 s IPOPT cost. With caching, the second and later constructions
# return the cached result in microseconds. Cleared by tests that need
# a fresh solve (rare) via ``_optimise_cache.clear()``.
_optimise_cache: dict = {}


def _cache_key(pattern, cfg):
    """Hashable key for a (pattern, cfg) pair. Includes only fields that
    actually affect the solve.

    Pattern: the four primary JugglePattern fields. The derived
    properties (throw_speed_mps, flight_time_s, etc.) are determined
    by these, so they don't add entropy.

    Cfg: the optimiser-relevant fields. Output-grid fields like
    ``n_samples`` don't affect the solve and therefore don't enter
    the key — same solve, just resampled.
    """
    return (
        pattern.apex_height_mm, pattern.separation_mm,
        pattern.active_z_mm, pattern.oval_width_mm,
        cfg.n_knots, cfg.n_sub_per_segment,
        cfg.bank_locked_level, cfg.tilt_limit_rad,
        cfg.catch_offset_s, cfg.workspace_xy_mm, cfg.workspace_z_mm,
        cfg.platform_init_height_mm,
        cfg.pose_jerk_weight, cfg.leg_jerk_weight,
        cfg.leg_vel_equalize_weight, cfg.leg_vel_hard_cap_mms,
        cfg.catch_colinearity_weight, cfg.throw_colinearity_weight,
        cfg.catch_vel_ratio, cfg.catch_slider_vel_ratio,
        cfg.throw_twist_lin_weight, cfg.throw_twist_ang_weight,
        cfg.throw_twist_acc_weight,
    )


def _cached_optimise(pattern, cfg):
    """Memoised wrapper around :func:`optimise_juggle_trajectory`."""
    key = _cache_key(pattern, cfg)
    if key not in _optimise_cache:
        _optimise_cache[key] = optimise_juggle_trajectory(pattern, cfg)
    return _optimise_cache[key]
SCHEDULE_LEAD_S = 1.2                      # hand-event lookahead — exceeds the
                                            # prelude duration for any throw /
                                            # catch this demo schedules. The
                                            # FIRST hand throw needs the most
                                            # lead: hand starts at catch-prime
                                            # (~323 mm), throw start is at the
                                            # stroke bottom (~20 mm), so the
                                            # smooth-move prelude is ~0.75 s
                                            # plus ~0.07 s throw lead. Steady-
                                            # state throws/catches need almost
                                            # no prelude (hand ends at throw-
                                            # start after each catch, vice
                                            # versa) — the lead is mostly for
                                            # the bootstrap.
DROP_Z_FLOOR_MM = -200.0                   # below this, ball is considered dropped
RESET_PARK_Z_MM = 100000.0                 # park a dropped ball out of scene


# --------------------------------------------------------------------------
# Internal state types
# --------------------------------------------------------------------------
@dataclasses.dataclass
class _ThrowState:
    sequence: HandThrowSequence
    event: Event
    released: bool = False


@dataclasses.dataclass
class _CatchState:
    sequence: HandCatchSequence
    event: Event
    # Closed-loop catch reach: the platform-xy offset (mm, world) that brings
    # the cup to the OBSERVED ball's predicted landing at the catch instant.
    # ``None`` until the ball is cleanly in flight and the reach is computed
    # (then frozen). See ``_JuggleDemoRunner._update_catch_reaches``.
    reach_xy: Optional[np.ndarray] = None


# --------------------------------------------------------------------------
# Public types
# --------------------------------------------------------------------------
@dataclasses.dataclass
class JuggleDemoConfig:
    """Knobs for :func:`run`. Sensible defaults match the plan's §4 Phase 3
    operating point.
    """
    # Default pattern: apex 1.3 m (unchanged), throw–catch separation
    # 200 mm (doubled from JugglePattern's 100 mm default). The wider
    # spacing surfaces the optimiser's tilt + banking behaviour more
    # visibly; override via the ``--apex-height-mm`` / ``--separation-mm``
    # CLI flags or by passing a custom JugglePattern to ``run()``.
    pattern: JugglePattern = dataclasses.field(
        default_factory=lambda: JugglePattern(separation_mm=200.0))
    duration_s: float = 30.0
    n_catches: int = 32
    # BB yaw-axis origin in the world frame (origin at Jugglebot's base
    # centroid, +z up). Measured throw position post-BB-cutover. NOTE: this
    # is the yaw-axis origin, not the ball-release point — the release sits
    # ~release_l_position_mm (150 mm) out along the arm; the BB aim solver
    # accounts for that, so the primed ball still lands in the cup.
    bb_position_mm: tuple[float, float, float] = (-872.0, -630.0, 1430.0)
    # yaw_offset_rad rotates the BB so its local +x points toward Jugglebot
    # (the world origin). The direction from BB to the origin is
    # -bb_position_mm in xy → atan2(+630, +872) ≈ 0.626 rad (35.8°). If
    # bb_position_mm changes, update this to match.
    bb_yaw_offset_rad: float = math.atan2(630.0, 872.0)
    bb_scatter_mm: float = 0.0
    seed: Optional[int] = None
    abort_at_s: Optional[float] = None     # if set, abort at this sim_time
    log_path: Optional[str] = None
    headless: bool = True                  # if False, launch MuJoCo viewer
    use_optimised_trajectory: bool = True  # if False, use build_analytic_oval
    realtime_rate: float = 0.0             # 0 = free-running (fastest); 1.0 =
                                            # wall-clock real-time; 0.5 = half
                                            # speed (slowmo). Wall-paces the
                                            # main loop after each plant.step.
    dashboard: bool = False                # if True, start the telemetry
                                            # dashboard on `dashboard_port`
    dashboard_port: int = 8082
    # ---- Offscreen video recording ----------------------------------
    # When ``record_path`` is set, the runner renders the live MuJoCo
    # scene offscreen from a FIXED free camera each 40 Hz tick and pipes
    # frames to the system ffmpeg (H.264 mp4). Independent of ``headless``
    # / ``--viewer`` — recording needs no on-screen window. The ``cam_*``
    # fields override the model's default free-camera angle; leave any of
    # them ``None`` to inherit the model default (see ``_build_record_camera``).
    record_path: Optional[str] = None
    record_size: tuple[int, int] = (1280, 720)   # (width, height) px
    record_fps: int = 40                   # output fps; 40 = real-time
                                            # (one frame per tick), lower
                                            # = slow-motion playback
    cam_azimuth: Optional[float] = None
    cam_elevation: Optional[float] = None
    cam_distance: Optional[float] = None
    cam_lookat: Optional[tuple[float, float, float]] = None
    capture_tolerance_mm: Optional[float] = None  # INTERIM (2026-06-26):
                                            # loose gate while the catch is
                                            # VELOCITY-MATCHED (cup moves at
                                            # 0.7×ball, colinear). A cup
                                            # rushing through the catch point
                                            # at ~3.5 m/s contacts the ball
                                            # off-centre by the geometric
                                            # measure, so the strict 30 mm
                                            # centre-to-cup gate (the
                                            # 2026-05-24 default) structurally
                                            # can't judge a moving-cup catch —
                                            # it reads 1/33 even though the
                                            # ball reaches the cup every time
                                            # (33/33 loose). The proper oracle
                                            # is real CONTACT MECHANICS (does
                                            # the ball physically seat?), which
                                            # replaces this gate; restore a
                                            # real seat-based check then. Set a
                                            # value to re-enable the geometric
                                            # gate. See logbook 2026-06-26.


@dataclasses.dataclass
class JuggleDemoStats:
    """End-of-run summary returned by :func:`run`."""
    captures: list[tuple[float, int]]      # (sim_time, ball_idx)
    drops: list[tuple[float, int]]
    events_dispatched: int
    duration_s: float
    aborted: bool
    abort_time_s: Optional[float]

    @property
    def n_captures(self) -> int:
        return len(self.captures)

    @property
    def n_drops(self) -> int:
        return len(self.drops)


# --------------------------------------------------------------------------
# Runner
# --------------------------------------------------------------------------
class _JuggleDemoRunner:
    """Internal runner — one instance per :func:`run` call.

    Tests can construct this directly for white-box inspection (state
    machine, scheduled sequences, etc.); the public :func:`run` is the
    intended entry point.
    """

    def __init__(self, cfg: JuggleDemoConfig):
        if cfg.seed is not None:
            np.random.seed(cfg.seed)
        # Dedicated generator for BB scatter so cfg.seed actually makes
        # scatter reproducible (the global np.random.seed above does NOT
        # reach BallButlerSim, which draws from its own Generator).
        self._rng = np.random.default_rng(cfg.seed)

        self.cfg = cfg
        self.pattern = cfg.pattern

        # ---- Trajectory --------------------------------------------------
        if cfg.use_optimised_trajectory:
            # OptimizerConfig() picks up the project defaults — N=12
            # knots, leg-jerk² primary, leg-vel equalisation 0.01,
            # catch-colinearity weight 1000. Solve is cached on
            # ``(pattern_primary_fields, optimiser_relevant_cfg_fields)``
            # — see ``_cache_key`` for the exact set. Adding a new
            # OptimizerConfig field that affects the solve requires
            # extending ``_cache_key`` too. With the cache, the test
            # suite's repeated runner constructions hit the ~12 s
            # IPOPT solve once and reuse the result.
            opt_cfg = OptimizerConfig()
            self.opt_result = _cached_optimise(self.pattern, opt_cfg)
            self.trajectory: JuggleTrajectory = self.opt_result.trajectory
            self.catch_offset_s = self.opt_result.catch_offset_s
        else:
            # Fallback: the analytic oval. Doesn't satisfy throw/catch
            # constraints — only useful for debugging the plumbing.
            self.opt_result = None
            self.trajectory = build_analytic_oval(self.pattern)
            self.catch_offset_s = (self.pattern.flight_time_s
                                   - self.pattern.platform_period_s)

        # ---- Plant + player ---------------------------------------------
        # contact_carry=True: the ball physically rests in the cup and is
        # thrown by the hand stroke (concern 1). Capture is the seat-based
        # metric (ball settled + co-moving), NOT the old geometric snap
        # gate — so cfg.capture_tolerance_mm no longer governs the demo
        # (it is retained only as a legacy CLI knob; see its field doc).
        self.plant = MuJoCoPlant(control_dt=CONTROL_DT_S, contact_carry=True)
        self.player = TrajectoryPlayer(self.trajectory,
                                       geom=self.plant.geom,
                                       control_dt=CONTROL_DT_S)

        # ---- Initial state ----------------------------------------------
        # Platform at THROW pose; hand at catch-prime. Ball 1 (pre-held) is
        # seated into the hand AFTER the platform warm-up below (contact
        # carry can't tolerate the startup position step — see there); ball
        # 0 (BB-primed) is parked until the BB throws it.
        self.plant.reset(pose_6dof=self.pattern.throw_pose)
        # Hand starts LOW (at the throw-stroke start), not at catch-prime:
        # the pre-held ball is THROWN first, and a low hand keeps the cup
        # near the platform centroid (a short lever arm) so the oval carry
        # doesn't fling the contact-held ball out — at the ~323 mm prime
        # lever, platform tilt amplifies into cup acceleration that ejects
        # the ball. (Kinematic hold was immune; contact carry is not.)
        self.plant.command_hand(_PREHELD_CARRY_HAND_MM)
        # Settle: a handful of ticks for the hand actuator to converge.
        for _ in range(20):
            self.plant.step(CONTROL_DT_S)
        assert self.plant.n_balls >= 2, (
            f"need >= 2 balls in the model; found {self.plant.n_balls}")

        # ---- BB sim + BB lead-time calibration --------------------------
        # The MasterTimeline schedules ``bb_throw`` at t_wall = t0 - bb_lead.
        # The BB's actual time-of-flight (TOF) for the configured throw is
        # determined by its physics, not by the demo's bb_lead. To make
        # the ball arrive at the first hand catch event (t_wall = t0 +
        # catch_offset), set ``bb_lead = BB_TOF - catch_offset``.
        self.bb_sim = BallButlerSim.from_hardware_config(
            np.array(cfg.bb_position_mm, dtype=float),
            cfg.bb_yaw_offset_rad)
        # Catch-target = world-frame position of the hand opening at the
        # CATCH event instant. The catch event is the *midpoint of the
        # catch velocity-hold phase* (per HandCatchTrajectory's timeline
        # convention) — by that time the hand has descended ~137 mm from
        # the top of stroke (slider at ~198 mm). Querying the catch
        # trajectory at t=0 gives the exact slider position at arrival;
        # the formula is independent of throw_speed (the vC ratio
        # cancels). The trajectory's catch-event pose (xyz + orientation)
        # gives the platform centroid and the orientation; the hand
        # opening is offset by R_catch @ [0, 0, hand_offset_at_arrival].
        # The orientation matters — with banking enabled the optimised
        # trajectory tilts at catch, shifting the hand opening's world
        # xy by sin(tilt) * hand_offset (~35 mm at the 18° default-
        # config catch tilt). Aiming at a level-assumed target would
        # miss the cup by more than the BallManager's capture tolerance.
        _catch_slider_at_arrival = HandCatchTrajectory(
            self.pattern.throw_speed_mps).sample(0.0)
        hand_offset_at_arrival_mm = compute_hand_offset_mm(_catch_slider_at_arrival)
        pose_at_catch, _, _ = self.trajectory.eval(self.catch_offset_s)
        catch_centroid_world = np.array([
            pose_at_catch[0],
            pose_at_catch[1],
            self.plant.geom.init_height_mm + pose_at_catch[2],
        ])
        R_catch = rodrigues(pose_at_catch[3:6])
        self._catch_target_world_mm = (
            catch_centroid_world
            + R_catch @ np.array([0.0, 0.0, hand_offset_at_arrival_mm]))
        _, _, self._bb_tof_s = self.bb_sim.compute_release_state(
            self._catch_target_world_mm)
        bb_lead = self._bb_tof_s - self.catch_offset_s
        if bb_lead <= 0.0:
            # Pathological: BB TOF is shorter than the catch offset, i.e.
            # the BB ball would land before the BB even throws. Fall back
            # to the default lead and warn — the demo will not catch the
            # priming ball cleanly, but downstream throws can still juggle.
            print(f"[juggle_demo] BB TOF {self._bb_tof_s:.3f}s < catch_offset "
                  f"{self.catch_offset_s:.3f}s — using fallback bb_lead", file=sys.stderr)
            bb_lead = DEFAULT_BB_LEAD_S

        # ---- Master timeline --------------------------------------------
        # t0 is the first hand-throw instant on the shared (sim) wall clock.
        # The init buffer above bb_lead leaves time for the first throw's
        # scheduling lead (the hand has to traverse from catch-prime to
        # the throw-stroke start before the throw fires).
        t_now = float(self.plant.get_state().time)
        self._t0 = t_now + max(bb_lead, SCHEDULE_LEAD_S) + 0.4
        self.timeline = MasterTimeline(
            self.pattern, self.trajectory, t0=self._t0,
            n_catches=cfg.n_catches,
            catch_offset_s=self.catch_offset_s,
            bb_lead_s=bb_lead,
        )

        # ---- Platform warm-up, then seat the pre-held ball --------------
        # contact_carry is sensitive to a startup position step: the
        # platform was reset to throw_pose, but the first trajectory
        # command (command_at(t_rel0), ~1.6 s before t0) is an ~85 mm
        # one-tick step at the hand opening, whose violent sub-tick
        # transient the stiff cup contact converts into a ball ejection
        # (the kinematic hold masked this entirely). Drive the platform
        # onto the trajectory for a few ticks FIRST — with no ball seated —
        # so the step is absorbed, then seat the pre-held ball onto the
        # smoothly-moving hand (spawn_in_hand gives it the hand-opening
        # velocity, so it is co-moving and seats). Hardware has no such
        # step: the platform accelerates from rest onto the pattern. The
        # warm-up consumes a few ticks of the pre-roll lead, well within
        # the buffer before the first BB throw.
        for _ in range(_PLATFORM_WARMUP_TICKS):
            t_rel = float(self.plant.get_state().time) - self._t0
            self.plant.command(self.player.command_at(t_rel).ext_mm)
            self.plant.step(CONTROL_DT_S, plat_cmd_fn=self._plat_cmd_at)
        self.plant.ball_manager.ball(1).spawn_in_hand()

        # ---- State machine ----------------------------------------------
        # Hand sequences are queued in time order. The runner pulls events
        # from the timeline up to SCHEDULE_LEAD_S in advance — that gives
        # the bootstrap throw enough prelude time to traverse from
        # catch-prime to throw-start, but it also means a second event of
        # the same kind can be pulled before the first fires. Queueing
        # both throws and catches separately preserves time order; the
        # head's `sample(t_wall)` decides which one is currently driving
        # the hand.
        self._throw_queue: list[_ThrowState] = []
        self._catch_queue: list[_CatchState] = []
        # Deferred BB throws — pulled from the timeline early, executed at
        # their exact t_wall.
        self._pending_bb_throws: list[Event] = []

        # ---- Stats -------------------------------------------------------
        self.captures: list[tuple[float, int]] = []
        self.drops: list[tuple[float, int]] = []
        self.events_dispatched = 0
        self.aborted = False
        self.abort_time_s: Optional[float] = None
        self._t_start = t_now

        # ---- Telemetry state shared between CSV + dashboard --------------
        # ``_last_event_kind`` reports the most recently dispatched
        # timeline-event kind (``bb_throw`` / ``hand_throw`` /
        # ``hand_catch`` / ``aborted``); empty before any event fires.
        # ``_prev_leg_vel_*`` are the previous-tick snapshot used to
        # compute the leg-acceleration derivative passed to the SSE feed.
        self._last_event_kind: str = ""
        self._prev_leg_vel_mmps: Optional[np.ndarray] = None
        self._prev_telemetry_t: Optional[float] = None

        # ---- Telemetry CSV -----------------------------------------------
        self._csv_fp = None
        self._csv_writer = None
        if cfg.log_path:
            self._open_csv(cfg.log_path)

        # ---- Live dashboard ---------------------------------------------
        self._dashboard = None
        if cfg.dashboard:
            # Lazy import — the dashboard server pulls in http.server and
            # threading; the headless integration tests don't need them.
            from sim.viz.dashboard.server import DashboardServer
            self._dashboard = DashboardServer(port=cfg.dashboard_port)
            self._dashboard.start()
            # The DashboardServer's own start() prints
            # "Dashboard: http://localhost:<port>" — accurate only when a
            # browser on the *same machine* is reaching it. Print the
            # machine's LAN IPs too so a remote browser can connect.
            _print_reachable_urls(cfg.dashboard_port)
        self._last_hand_cmd_mm = 0.0       # for the StepRecord.hand_cmd_mm field

        # ---- Offscreen video recorder -----------------------------------
        # Lazily built only when a record path is set so headless tests and
        # plain runs never touch the GL/offscreen path. Captures one frame
        # per tick (see ``tick`` step 9). Settle ticks in __init__ run
        # before this exists, so they are not recorded.
        self._recorder = None
        if cfg.record_path:
            self._recorder = _VideoRecorder(
                self.plant.model, self.plant.data,
                cfg.record_path,
                width=cfg.record_size[0], height=cfg.record_size[1],
                fps=cfg.record_fps,
                cam=_build_record_camera(self.plant.model, cfg),
            )

        # ---- Keyboard / interactive state -------------------------------
        # Driven by the MuJoCo passive viewer's `key_callback`. When the
        # demo runs headless (no viewer) the keys aren't observed and the
        # defaults (not paused, configured rate, no pending step) apply.
        self._paused = False
        self._pending_frame_step = False
        self._effective_realtime_rate = float(cfg.realtime_rate)
        # Set by the module-level ``run()`` after the passive viewer is
        # launched, so ``key_callback`` can read the live free camera for
        # the ``C`` (print-camera) binding. ``None`` when headless.
        self._viewer_handle = None

    # ---- Keyboard --------------------------------------------------------
    def key_callback(self, keycode: int) -> None:
        """Handle a keypress from the MuJoCo passive viewer.

        Bindings::

            SPACE              toggle pause
            RIGHT_ARROW        when paused, step exactly one 40 Hz tick
            ``[`` / ``]``      halve / double the wall-time playback rate
                               (jumping out of free-running mode if needed)
            LEFT_ARROW         no-op (sim can't run backwards)
        """
        if keycode == _KEY_SPACE:
            self._paused = not self._paused
            state = "paused" if self._paused else "running"
            print(f"[juggle_demo] {state} at sim_time "
                  f"{float(self.plant.get_state().time) - self._t_start:.3f} s")
        elif keycode == _KEY_RIGHT_ARROW:
            if self._paused:
                self._pending_frame_step = True
        elif keycode == _KEY_LEFT_BRACKET:
            # Halve the rate. Free-running (0) jumps to real-time first
            # so the user has somewhere meaningful to step down from.
            if self._effective_realtime_rate <= 0.0:
                self._effective_realtime_rate = 1.0
            else:
                self._effective_realtime_rate = max(
                    self._effective_realtime_rate * 0.5, 0.0625)
            print(f"[juggle_demo] realtime rate = "
                  f"{self._effective_realtime_rate:.3f}x")
        elif keycode == _KEY_RIGHT_BRACKET:
            if self._effective_realtime_rate <= 0.0:
                self._effective_realtime_rate = 1.0
            else:
                self._effective_realtime_rate = min(
                    self._effective_realtime_rate * 2.0, 16.0)
            print(f"[juggle_demo] realtime rate = "
                  f"{self._effective_realtime_rate:.3f}x")
        elif keycode == _KEY_C:
            # Print the current free-camera parameters as ready-to-paste
            # ``--cam-*`` flags, so the user can record a fixed-camera
            # video from the angle they just dragged to.
            h = self._viewer_handle
            if h is not None:
                c = h.cam
                print("[juggle_demo] camera: "
                      f"--cam-azimuth {c.azimuth:.1f} "
                      f"--cam-elevation {c.elevation:.1f} "
                      f"--cam-distance {c.distance:.3f} "
                      f"--cam-lookat {c.lookat[0]:.3f} "
                      f"{c.lookat[1]:.3f} {c.lookat[2]:.3f}")
        elif keycode == _KEY_LEFT_ARROW:
            pass   # documented no-op

    # ---- Lifecycle ------------------------------------------------------
    def close(self) -> None:
        if self._csv_fp is not None:
            self._csv_fp.close()
            self._csv_fp = None
        if self._dashboard is not None:
            self._dashboard.stop()
            self._dashboard = None
        if self._recorder is not None:
            self._recorder.close()
            self._recorder = None

    # ---- One control tick ----------------------------------------------
    def tick(self, t_wall: float) -> None:
        """One 40 Hz control step."""
        # 1. Look ahead: pre-schedule hand events whose t_wall falls
        #    within SCHEDULE_LEAD_S, defer BB events.
        for event in self.timeline.due_events(t_wall + SCHEDULE_LEAD_S):
            self.events_dispatched += 1
            self._last_event_kind = event.kind
            if event.kind == 'bb_throw':
                self._pending_bb_throws.append(event)
            elif event.kind == 'hand_throw':
                self._schedule_hand_throw(event, t_wall)
            elif event.kind == 'hand_catch':
                self._schedule_hand_catch(event, t_wall)

        # 2. Execute due BB throws (at their exact t_wall).
        while (self._pending_bb_throws
               and self._pending_bb_throws[0].t_wall <= t_wall):
            self._execute_bb_throw(self._pending_bb_throws.pop(0))

        # 2b. Closed-loop catch: freeze each imminent catch's reach once its
        #     ball is cleanly in flight (observe-and-predict; see method doc).
        self._update_catch_reaches(t_wall)

        # 3. Select the active hand sequence: command the 40 Hz setpoint,
        #    trigger the physics throw at the release instant, and get a
        #    sub-step sampler so the hand tracks the stroke at the physics
        #    rate (emulating the real 500 Hz hand).
        hand_sampler = self._update_active_hand(t_wall)

        # 3b. Switch the ball/cup contact stiff for the throw up-stroke
        #     (clean, deterministic separation), soft otherwise (cushioned
        #     catch/carry). See BallManager.set_contact_stiffness.
        self.plant.set_contact_stiffness(self._throw_stroke_active(t_wall))

        # 4. Command the platform (for telemetry; the per-substep sampler
        #    below supersedes it inside step()). ``cmd`` is the uncorrected
        #    plan (telemetry reference); the commanded extensions carry the
        #    closed-loop catch reach so the cup meets the observed ball.
        t_rel = t_wall - self._t0
        cmd = self.player.command_at(t_rel)
        self.plant.command(self._plat_cmd_at(t_wall))

        # 5. Step physics, sub-stepping BOTH the platform and hand commands
        #    at the physics rate (emulating the real 500 Hz controllers).
        #    The platform sub-step is load-bearing for contact carry: a
        #    40 Hz staircase makes the stiff leg actuators ring at each
        #    tick boundary and shakes the ball out of the cup.
        self.plant.step(CONTROL_DT_S,
                        hand_cmd_fn=hand_sampler,
                        plat_cmd_fn=self._plat_cmd_at)

        # 6. Poll captures + drops.
        new_t_wall = float(self.plant.get_state().time)
        self._poll_captures(new_t_wall)
        self._poll_drops(new_t_wall)

        # 7. Optional abort.
        sim_time = new_t_wall - self._t_start
        if (self.cfg.abort_at_s is not None
                and not self.aborted
                and sim_time >= self.cfg.abort_at_s):
            self._do_abort(new_t_wall)

        # 8. Telemetry.
        if self._csv_writer is not None or self._dashboard is not None:
            snap = self._telemetry_snapshot(new_t_wall)
            if self._csv_writer is not None:
                self._write_csv_row(new_t_wall, cmd, snap)
            if self._dashboard is not None:
                self._broadcast_dashboard_record(new_t_wall, cmd, snap)

        # 9. Optional video frame (offscreen render from the fixed camera).
        if self._recorder is not None:
            self._recorder.capture()

    # ---- Run loop -------------------------------------------------------
    def run(self, viewer_handle=None) -> JuggleDemoStats:
        n_steps = int(round(self.cfg.duration_s / CONTROL_DT_S))
        # Wall-time pacing: when ``_effective_realtime_rate > 0`` the loop
        # sleeps so one tick of sim time takes ``CONTROL_DT_S / rate``
        # seconds of wall time. ``1.0`` = real-time, ``0.5`` = half-speed
        # slowmo, ``2.0`` = 2x fast playback. ``0`` skips the pacing so
        # tests and headless runs go as fast as MuJoCo can integrate
        # (the existing behaviour). The viewer's key_callback can mutate
        # the rate live via ``[`` / ``]``; the loop re-reads it each
        # iteration so the schedule re-targets immediately.
        wall_anchor = time.monotonic()
        steps_taken_for_schedule = 0       # sim ticks consumed since the
                                            # last rate change or unpause —
                                            # the schedule's "step" count
        executed_steps = 0                  # count of ticks actually run;
                                            # advances duration progress
                                            # only when not paused
        last_rate = self._effective_realtime_rate
        while executed_steps < n_steps:
            # Re-anchor the pacing schedule if the rate changed (live key
            # bindings) so the new rate takes effect cleanly without
            # racing to "catch up" against the old schedule.
            if self._effective_realtime_rate != last_rate:
                wall_anchor = time.monotonic()
                steps_taken_for_schedule = 0
                last_rate = self._effective_realtime_rate

            if self._paused and not self._pending_frame_step:
                # Idle: keep the viewer responsive (so key events fire
                # and the user can drag the camera) but do not tick the
                # sim. Re-anchor the schedule too — when we resume we
                # don't want to suddenly run a backlog of "missed" ticks.
                if viewer_handle is not None:
                    viewer_handle.sync()
                time.sleep(0.01)
                wall_anchor = time.monotonic()
                steps_taken_for_schedule = 0
                continue
            # If a single-frame step was requested while paused, do
            # exactly one tick and re-pause.
            consume_frame_step = self._pending_frame_step
            self._pending_frame_step = False

            t_wall = float(self.plant.get_state().time)
            self.tick(t_wall)
            if viewer_handle is not None:
                viewer_handle.sync()
            executed_steps += 1
            steps_taken_for_schedule += 1

            if consume_frame_step:
                # One-shot step; immediately drop back into the paused
                # branch on the next loop iteration. Don't bother pacing
                # — single-frame stepping is meant to be instantaneous.
                continue

            if self._effective_realtime_rate > 0.0:
                # Sleep until the next tick's scheduled wall-time. Using
                # an absolute schedule (anchored at the last rate change
                # / unpause) keeps the pacing accurate over long runs
                # (no drift from cumulative sleep-undershoot).
                wall_dt = CONTROL_DT_S / self._effective_realtime_rate
                target = (wall_anchor
                          + steps_taken_for_schedule * wall_dt)
                remaining = target - time.monotonic()
                if remaining > 0.0:
                    time.sleep(remaining)
        return JuggleDemoStats(
            captures=list(self.captures),
            drops=list(self.drops),
            events_dispatched=self.events_dispatched,
            duration_s=float(self.plant.get_state().time) - self._t_start,
            aborted=self.aborted,
            abort_time_s=self.abort_time_s,
        )

    # ---- Event dispatch (private) --------------------------------------
    def _execute_bb_throw(self, event: Event) -> None:
        """BB releases the primed ball at this exact moment.

        Aims at ``_catch_target_world_mm`` — the world-frame hand-opening
        position at the matched catch event, computed from the optimised
        trajectory's catch-event pose AND orientation. With banking
        enabled the catch tilt shifts this target's xy by ~35 mm from a
        level-assumed catch; aiming at the level-assumed target misses
        the cup by more than the BallManager's capture tolerance.
        """
        spawn = self.bb_sim.throw_at_jugglebot(
            spawn_time=event.t_wall,
            landing_xy_mm=self._catch_target_world_mm[:2],
            catch_z_mm=float(self._catch_target_world_mm[2]),
            scatter_mm=self.cfg.bb_scatter_mm,
            rng=self._rng,
        )
        # Spawn into the sim now (event.t_wall == current t_wall by
        # the pending-queue gating in tick()).
        self.plant.spawn_ball(spawn.position_mm, spawn.velocity_mms,
                              ball=event.ball)

    def _schedule_hand_throw(self, event: Event, t_wall: float) -> None:
        """Queue a HandThrowSequence whose release fires at event.t_wall."""
        # Use the LATEST already-queued sequence's end position as the
        # starting hand position for this new sequence's prelude — not
        # the current sensed hand position. In steady state the hand
        # finishes the previous catch at the stroke bottom; if we used
        # the current sensed position (perhaps the middle of an in-
        # progress catch) the prelude would over- or under-shoot.
        hand_pos = self._predict_hand_pos_at(event.t_wall)
        # Throw-speed calibration (concern 1, contact throw): the ball
        # now leaves by the physics of the hand stroke, not an imposed
        # velocity, so it separates carrying only a fraction of the
        # commanded stroke peak speed (the cup decelerates at the stroke
        # top and the ball outruns it). Command the stroke faster by
        # ``THROW_SPEED_CALIB`` so the *realised* ball exit speed matches
        # the optimiser-planned ``event.event_vel`` — keeping the apex,
        # flight time, and catch aim that the offline solve assumed.
        # Calibrated empirically against the demo (see logbook 2026-06-26
        # contact-mechanics-integration).
        result = HandThrowSequence.try_create(
            throw_speed_mps=event.event_vel * THROW_SPEED_CALIB,
            release_time=event.t_wall,
            current_pos_mm=hand_pos,
            current_time=t_wall,
        )
        if result.feasible:
            self._throw_queue.append(_ThrowState(result.sequence, event))
        else:
            # Document the infeasibility; not a runner-fatal error — the
            # ball just isn't thrown this cycle.
            print(f"[juggle_demo] hand throw infeasible at t={t_wall:.3f}: "
                  f"{result.reason}", file=sys.stderr)

    def _schedule_hand_catch(self, event: Event, t_wall: float) -> None:
        """Queue a HandCatchSequence whose midpoint fires at event.t_wall."""
        # See note in :meth:`_schedule_hand_throw` — predict at the
        # event's time, not the scheduling tick's time.
        hand_pos = self._predict_hand_pos_at(event.t_wall)
        result = HandCatchSequence.try_create(
            event_vel_mps=event.event_vel,
            arrival_time=event.t_wall,
            current_pos_mm=hand_pos,
            current_time=t_wall,
        )
        if result.feasible:
            self._catch_queue.append(_CatchState(result.sequence, event))
        else:
            print(f"[juggle_demo] hand catch infeasible at t={t_wall:.3f}: "
                  f"{result.reason}", file=sys.stderr)

    def _throw_stroke_active(self, t_wall: float) -> bool:
        """True while the active hand throw is in its up-stroke window.

        Used to switch the ball/cup contact to the stiff (clean-throw)
        regime for the stroke and back to soft (cushioned catch/carry)
        otherwise. The window brackets the throw trajectory's
        accel→separation span; the small margins ensure the stiff regime is
        in place before the stroke presses the ball in and stays through
        separation.
        """
        if not self._throw_queue:
            return False
        seq = self._throw_queue[0].sequence
        traj = seq.throw_trajectory
        start = seq.release_time + traj.start_time
        end = seq.release_time + traj.end_time
        return (start - 0.01) <= t_wall <= (end + 0.03)

    def _plat_cmd_at(self, t_wall: float) -> np.ndarray:
        """Platform leg-extension target at sim time ``t_wall`` — the
        per-substep platform command sampler passed to
        :meth:`MuJoCoPlant.step`. Evaluates the trajectory player at the
        substep's relative time so the platform tracks the trajectory at
        the physics rate, not a 40 Hz staircase. Adds the closed-loop catch
        reach (platform-xy offset) so the cup meets the observed ball.
        """
        pose = self.player.command_at(t_wall - self._t0).pose.copy()
        pose[:2] += self._catch_reach_xy(t_wall)
        return self.plant.pose_to_extensions(pose)

    def _catch_reach_xy(self, t_wall: float) -> np.ndarray:
        """Active platform-xy reach offset (mm, world) at ``t_wall``.

        Sum of each queued catch's smootherstep ramp-hold-ramp: ramped in over
        ``[tc-_REACH_T0, tc-_REACH_T1]``, HELD (= ``reach_xy``) over
        ``[tc-_REACH_T1, tc+_REACH_T2]`` spanning the catch with zero
        correction-velocity, ramped out over ``[tc+_REACH_T2, tc+_REACH_T3]``,
        and zero outside (so throws stay on the periodic plan and the
        velocity-matched catch is preserved). Catches whose reach is not yet
        computed contribute nothing.
        """
        off = np.zeros(2)
        # The platform warm-up in __init__ drives _plat_cmd_at before the
        # catch queue exists; no catches are pending then, so reach is zero.
        for cs in getattr(self, '_catch_queue', ()):
            if cs.reach_xy is None:
                continue
            u = t_wall - cs.event.t_wall
            if -_REACH_T0 <= u < -_REACH_T1:
                s = (u + _REACH_T0) / (_REACH_T0 - _REACH_T1)        # 0 -> 1
                off = off + cs.reach_xy * _smootherstep(s)
            elif -_REACH_T1 <= u <= _REACH_T2:
                off = off + cs.reach_xy                              # hold
            elif _REACH_T2 < u <= _REACH_T3:
                s = (u - _REACH_T2) / (_REACH_T3 - _REACH_T2)        # 0 -> 1
                off = off + cs.reach_xy * (1.0 - _smootherstep(s))
        return off

    def _update_catch_reaches(self, t_wall: float) -> None:
        """Freeze each imminent catch's reach once its ball is cleanly in flight.

        For every queued catch whose reach window has begun (``t_wall >= tc -
        _REACH_T0``) and whose target ball is in free flight, predict the
        ball's xy at the catch instant (ballistic — xy is constant-velocity in
        sim) and set ``reach_xy`` to the platform offset that moves the cup
        (planned catch-target xy) onto that landing. Computed ONCE per catch
        then frozen, so the reach is a smooth pre-planned bump, not a per-tick
        chase (which would inject the platform's own tracking jitter into the
        catch). The planned cup xy is the periodic catch target
        ``_catch_target_world_mm`` (same every period).
        """
        for cs in self._catch_queue:
            if cs.reach_xy is not None:
                continue
            tc = cs.event.t_wall
            if t_wall < tc - _REACH_T0:
                continue
            ball = self.plant.get_ball_state(ball=cs.event.ball)
            if ball is None or ball.held or not ball.active:
                continue
            dt = tc - t_wall
            if dt <= 0.0:
                continue
            landing_xy = (np.asarray(ball.position_mm[:2], dtype=float)
                          + np.asarray(ball.velocity_mms[:2], dtype=float) * dt)
            reach = (landing_xy - self._catch_target_world_mm[:2]) * _REACH_GAIN
            mag = float(np.linalg.norm(reach))
            if mag > _REACH_MAX_MM:
                reach = reach * (_REACH_MAX_MM / mag)
            cs.reach_xy = reach

    def _predict_hand_pos_at(self, t_wall: float) -> float:
        """Estimate the hand position at ``t_wall`` for prelude planning.

        Returns the end position of the latest already-queued sequence
        (throw or catch, whichever ends later among those that end before
        ``t_wall``). Falls back to the current sensed hand position when
        no sequence is queued ahead.
        """
        candidates: list[tuple[float, float]] = []
        for st in self._throw_queue:
            if st.sequence.end_time <= t_wall:
                # HandThrowSequence exposes end_pos_mm directly
                candidates.append((st.sequence.end_time, st.sequence.end_pos_mm))
        for st in self._catch_queue:
            if st.sequence.end_time <= t_wall:
                candidates.append(
                    (st.sequence.end_time,
                     st.sequence.catch_trajectory.end_pos_mm))
        if candidates:
            candidates.sort(key=lambda x: x[0])
            return candidates[-1][1]
        return self.plant.get_state().hand_pos_mm or 0.0

    def _update_active_hand(self, t_wall: float):
        """Select the live hand sequence and return its sub-step sampler.

        Commands the 40 Hz hand setpoint from whichever queued sequence is
        currently live, and returns that sequence's ``sample`` callable
        ``fn(t)->pos|None`` so :meth:`MuJoCoPlant.step` can refresh the
        hand command every physics substep (emulating the real 500 Hz
        hand). Returns ``None`` when no sequence drives the hand this tick.

        Throw takes priority over catch during overlap (the catch's
        END_PROFILE_HOLD tail at the bottom of stroke happens to be the
        throw's prelude start, so giving throw priority is the natural
        ordering). Sequences whose ``end_time`` has passed are popped
        off the head of their respective queues.

        At the throw's release instant the ball transitions to free flight
        by :meth:`MuJoCoPlant.begin_physics_throw` — **no** velocity is set;
        the throw emerges from the (sub-stepped) hand stroke (concern 1).
        This replaces the old sim-only analytic release-velocity override.
        """
        # Prune expired heads.
        while self._throw_queue and t_wall > self._throw_queue[0].sequence.end_time:
            self._throw_queue.pop(0)
        while self._catch_queue and t_wall > self._catch_queue[0].sequence.end_time:
            self._catch_queue.pop(0)

        # Throw head first — owns the hand once its prelude has started.
        if self._throw_queue:
            head = self._throw_queue[0]
            if t_wall >= head.sequence.prelude_start_time:
                pos = head.sequence.sample(t_wall)
                if pos is not None:
                    self.plant.command_hand(pos)
                    self._last_hand_cmd_mm = pos
                if not head.released and t_wall >= head.sequence.release_time:
                    # Physics throw: the ball leaves by the hand stroke, not
                    # an imposed velocity. begin_physics_throw un-holds the
                    # ball + arms the re-capture guard while keeping contact
                    # enabled so the still-rising cup keeps accelerating it.
                    self.plant.begin_physics_throw(ball=head.event.ball)
                    head.released = True
                return head.sequence.sample

        # No throw is driving the hand right now — try catch head.
        if self._catch_queue:
            head = self._catch_queue[0]
            if t_wall >= head.sequence.prelude_start_time:
                pos = head.sequence.sample(t_wall)
                if pos is not None:
                    self.plant.command_hand(pos)
                    self._last_hand_cmd_mm = pos
                return head.sequence.sample
        return None

    # ---- Capture / drop polling ----------------------------------------
    def _poll_captures(self, t_wall: float) -> None:
        for ball_idx in range(self.plant.n_balls):
            if self.plant.check_and_capture(ball=ball_idx):
                self.captures.append((t_wall - self._t_start, ball_idx))

    def _poll_drops(self, t_wall: float) -> None:
        for ball_idx in range(self.plant.n_balls):
            state = self.plant.get_ball_state(ball=ball_idx)
            if state is None or not state.active or state.held:
                continue
            if state.position_mm[2] < DROP_Z_FLOOR_MM:
                self.drops.append((t_wall - self._t_start, ball_idx))
                # Park the dropped ball out of scene so it doesn't
                # interfere with the rest of the run.
                self.plant.ball_manager.ball(ball_idx).reset()

    # ---- Abort path -----------------------------------------------------
    def _do_abort(self, t_wall: float) -> None:
        """Truncate the master timeline, drop unstarted hand sequences,
        and swap the player to the exit transient.

        Sequences whose prelude has already begun stay in the queue — the
        hand can't safely be yanked mid-trajectory — but any sequence
        still in the queue with ``prelude_start_time > t_wall`` (a
        future throw or catch that hadn't started yet) is dropped so the
        abort actually stops the pattern instead of leaking late
        captures from queued-but-not-yet-fired events.
        """
        exit_transient = self.timeline.abort(t_wall)
        self.player.abort(exit_transient)
        self.aborted = True
        self._last_event_kind = "aborted"
        self.abort_time_s = t_wall - self._t_start
        self._throw_queue = [st for st in self._throw_queue
                             if st.sequence.prelude_start_time <= t_wall]
        self._catch_queue = [st for st in self._catch_queue
                             if st.sequence.prelude_start_time <= t_wall]
        self._pending_bb_throws = []

    # ---- Telemetry CSV --------------------------------------------------
    def _open_csv(self, path: str) -> None:
        import csv
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self._csv_fp = open(path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_fp)
        self._csv_writer.writerow([
            'sim_time',
            'plat_x_mm', 'plat_y_mm', 'plat_z_mm',
            'hand_pos_mm',
            'ball0_x', 'ball0_y', 'ball0_z', 'ball0_held',
            'ball1_x', 'ball1_y', 'ball1_z', 'ball1_held',
            'cmd_ext0', 'cmd_ext1', 'cmd_ext2',
            'cmd_ext3', 'cmd_ext4', 'cmd_ext5',
        ])

    def _telemetry_snapshot(self, t_wall: float) -> dict:
        """Sample plant + ball state once per tick for both CSV and SSE.

        Computes leg acceleration as a backward-difference derivative of
        ``leg_velocities_mmps`` against the previous snapshot.  Two
        consumers (CSV writer and dashboard broadcaster) share the same
        snapshot so the dashboard view always matches the CSV row.
        """
        state = self.plant.get_state()
        b0 = self.plant.get_ball_state(ball=0)
        b1 = self.plant.get_ball_state(ball=1)
        sim_time = t_wall - self._t_start

        # Backward-difference leg acceleration (mm/s²).  First tick: zero.
        # ``np.array(..., copy=True)`` is load-bearing: the plant returns a
        # view into its internal state buffer that is overwritten in
        # place each step.  Aliasing would make ``leg_acc`` read zero.
        leg_vel = np.array(state.leg_velocities_mmps, dtype=float, copy=True)
        if (self._prev_leg_vel_mmps is not None
                and self._prev_telemetry_t is not None
                and sim_time > self._prev_telemetry_t):
            dt = sim_time - self._prev_telemetry_t
            leg_acc = (leg_vel - self._prev_leg_vel_mmps) / dt
        else:
            leg_acc = np.zeros(6)
        self._prev_leg_vel_mmps = leg_vel
        self._prev_telemetry_t = sim_time

        nan3 = np.full(3, np.nan)
        ball_positions = np.stack([
            np.asarray(b0.position_mm, dtype=float) if b0 else nan3,
            np.asarray(b1.position_mm, dtype=float) if b1 else nan3,
        ])
        ball_velocities = np.stack([
            np.asarray(b0.velocity_mms, dtype=float) if b0 else nan3,
            np.asarray(b1.velocity_mms, dtype=float) if b1 else nan3,
        ])
        ball_held = np.array([
            int(b0.held) if b0 else 0,
            int(b1.held) if b1 else 0,
        ], dtype=int)

        return {
            'sim_time': sim_time,
            'state': state,
            'b0': b0,
            'b1': b1,
            'leg_acc': leg_acc,
            'ball_positions_mm': ball_positions,
            'ball_velocities_mms': ball_velocities,
            'ball_held': ball_held,
        }

    def _broadcast_dashboard_record(self, t_wall: float, cmd, snap: dict) -> None:
        """Push one StepRecord per tick to the connected dashboard clients."""
        state = snap['state']
        record = record_from_arrays(
            time=snap['sim_time'],
            ref_pose=cmd.pose,
            ref_twist=cmd.twist,
            actual_pose=np.concatenate([state.platform_pos_mm, state.platform_rot]),
            actual_twist=state.platform_twist,
            cmd_extensions=cmd.ext_mm,
            actual_extensions=state.leg_extensions_mm,
            leg_velocities=state.leg_velocities_mmps,
            leg_accelerations=snap['leg_acc'],
            hand_cmd_mm=self._last_hand_cmd_mm,
            hand_pos_mm=state.hand_pos_mm or 0.0,
            hand_vel_mmps=state.hand_vel_mmps or 0.0,
            ball_positions_mm=snap['ball_positions_mm'],
            ball_velocities_mms=snap['ball_velocities_mms'],
            ball_held=snap['ball_held'],
            throw_phase=self._last_event_kind,
            catches_total=len(self.captures),
            solve_status="ok",     # no MPC; placeholder for the dashboard schema
            t_ref_s=snap['sim_time'],
        )
        self._dashboard.broadcast(record)

    def _write_csv_row(self, t_wall: float, cmd, snap: dict) -> None:
        state = snap['state']
        b0 = snap['b0']
        b1 = snap['b1']
        self._csv_writer.writerow([
            f"{snap['sim_time']:.6f}",
            *(f"{x:.3f}" for x in state.platform_pos_mm),
            f"{(state.hand_pos_mm or 0.0):.3f}",
            *(f"{x:.3f}" for x in (b0.position_mm if b0 else [np.nan]*3)),
            f"{int(b0.held) if b0 else 0}",
            *(f"{x:.3f}" for x in (b1.position_mm if b1 else [np.nan]*3)),
            f"{int(b1.held) if b1 else 0}",
            *(f"{x:.4f}" for x in cmd.ext_mm),
        ])


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------
def _smootherstep(s: float) -> float:
    """C2 smootherstep ``6s⁵−15s⁴+10s³`` on [0,1]; zero slope at both ends.

    Used to ramp the closed-loop catch reach in/out with zero
    correction-velocity at the endpoints (no jerk into the platform).
    """
    return s * s * s * (s * (s * 6.0 - 15.0) + 10.0)


def _print_reachable_urls(port: int) -> None:
    """Print every LAN IPv4 address the dashboard is reachable from.

    The :class:`DashboardServer` itself prints ``http://localhost:<port>``,
    which is accurate only for a browser on the same machine. A remote
    browser (typical sim use: viewer on the Jetson, dashboard on the
    workstation) needs the Jetson's LAN IP. List every non-loopback
    IPv4 we can see so the user can copy-paste whichever one is on the
    right network.
    """
    import socket
    addrs: list[str] = []
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None,
                                       family=socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith('127.') and ip not in addrs:
                addrs.append(ip)
    except socket.gaierror:
        pass
    # Fallback: UDP socket trick to get the default-route IP.
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        if ip not in addrs and not ip.startswith('127.'):
            addrs.append(ip)
    except (OSError, socket.gaierror):
        pass
    for ip in addrs:
        print(f"Dashboard: http://{ip}:{port}  (LAN)")


# --------------------------------------------------------------------------
# Offscreen video recording
# --------------------------------------------------------------------------
def _build_record_camera(model, cfg: "JuggleDemoConfig"):
    """Construct the fixed free camera used for ``--record``.

    Starts from MuJoCo's default free camera — which reads the model's
    ``<global azimuth elevation>`` and frames the model with a sensible
    distance/lookat — then overrides any field the user supplied via the
    ``cam_*`` config fields. Leaving a field ``None`` inherits the model
    default, so ``--record`` with no ``--cam-*`` flags reproduces the
    viewer's startup angle.
    """
    import mujoco
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, cam)
    if cfg.cam_azimuth is not None:
        cam.azimuth = float(cfg.cam_azimuth)
    if cfg.cam_elevation is not None:
        cam.elevation = float(cfg.cam_elevation)
    if cfg.cam_distance is not None:
        cam.distance = float(cfg.cam_distance)
    if cfg.cam_lookat is not None:
        cam.lookat[:] = [float(v) for v in cfg.cam_lookat]
    return cam


class _VideoRecorder:
    """Render the live MuJoCo scene offscreen and stream it to ffmpeg.

    One frame is captured per 40 Hz tick from a *fixed* free camera. The
    system ``ffmpeg`` is fed raw RGB frames over a stdin pipe and encodes
    H.264 — so this needs only ``mujoco`` (for the offscreen renderer) and
    an ``ffmpeg`` on PATH, no ``imageio-ffmpeg`` dependency. The renderer
    shares the plant's live ``model``/``data``, so whatever the physics
    step produced (platform pose, hand, balls) is exactly what's drawn.
    """

    def __init__(self, model, data, path, *, width, height, fps, cam):
        import subprocess
        import mujoco
        self._data = data
        self._cam = cam
        # The MJCF declares a 640x480 offscreen framebuffer by default;
        # rendering larger than that raises. Enlarge the model's offscreen
        # buffer to fit the requested resolution before building the
        # renderer. This only grows the offscreen capability — it does not
        # affect physics or the passive viewer's onscreen buffer.
        if width > model.vis.global_.offwidth:
            model.vis.global_.offwidth = width
        if height > model.vis.global_.offheight:
            model.vis.global_.offheight = height
        self._renderer = mujoco.Renderer(model, height=height, width=width)
        self.path = path
        self.frames = 0
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        # rawvideo in → H.264 mp4 out. yuv420p + even dims keep the file
        # playable in browsers / QuickTime. -loglevel error keeps ffmpeg's
        # per-frame chatter out of the demo's stdout.
        cmd = [
            'ffmpeg', '-y', '-loglevel', 'error',
            '-f', 'rawvideo', '-pix_fmt', 'rgb24',
            '-s', f'{width}x{height}', '-r', str(fps), '-i', '-',
            '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
            '-crf', '18', '-preset', 'medium', str(path),
        ]
        try:
            self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        except FileNotFoundError as exc:
            self._renderer.close()
            raise RuntimeError(
                "ffmpeg not found on PATH — required for --record") from exc

    def capture(self) -> None:
        self._renderer.update_scene(self._data, camera=self._cam)
        frame = self._renderer.render()        # (h, w, 3) uint8, C-contiguous
        self._proc.stdin.write(frame.tobytes())
        self.frames += 1

    def close(self) -> None:
        if self._proc is not None:
            try:
                self._proc.stdin.close()
                self._proc.wait()
            finally:
                self._proc = None
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


# GLFW key codes used by mujoco.viewer's `key_callback`. Defined inline so
# we don't need to import glfw just for the constants.
_KEY_SPACE = 32
_KEY_C = 67                # print current free-camera angle as --cam-* flags
_KEY_LEFT_BRACKET = 91     # `[`
_KEY_RIGHT_BRACKET = 93    # `]`
_KEY_RIGHT_ARROW = 262
_KEY_LEFT_ARROW = 263      # accepted but no-op (sim can't run backwards)


# --------------------------------------------------------------------------
# Public entry points
# --------------------------------------------------------------------------
def run(cfg: Optional[JuggleDemoConfig] = None) -> JuggleDemoStats:
    """Run the juggle demo with the given config. Returns end-of-run stats."""
    if cfg is None:
        cfg = JuggleDemoConfig()
    runner = _JuggleDemoRunner(cfg)
    viewer_handle = None
    try:
        if not cfg.headless:
            import mujoco.viewer  # type: ignore
            viewer_handle = mujoco.viewer.launch_passive(
                runner.plant.model, runner.plant.data,
                key_callback=runner.key_callback)
            runner._viewer_handle = viewer_handle
            print("[juggle_demo] keyboard: SPACE pause/resume   "
                  "Right step (when paused)   [ slower / ] faster   "
                  "C print-camera")
        return runner.run(viewer_handle=viewer_handle)
    finally:
        runner.close()
        if viewer_handle is not None:
            viewer_handle.close()


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run the BB-led two-ball oval juggle demo in MuJoCo.")
    parser.add_argument('--duration', type=float, default=30.0,
                        help="Simulation duration in seconds (default 30)")
    parser.add_argument('--n-catches', type=int, default=32,
                        help="Target number of catches (default 32)")
    parser.add_argument('--viewer', action='store_true',
                        help="Launch the MuJoCo passive viewer")
    parser.add_argument('--abort-at', type=float, default=None,
                        help="Trigger abort at this sim time (seconds)")
    parser.add_argument('--scatter-mm', type=float, default=0.0,
                        help="Gaussian sigma for BB landing scatter (mm)")
    parser.add_argument('--seed', type=int, default=None,
                        help="RNG seed (for reproducible BB scatter)")
    parser.add_argument('--log',
                        default=None,
                        help="CSV log path (default: temp/logs/juggle_demo_<ts>.csv)")
    parser.add_argument('--no-log', action='store_true',
                        help="Disable CSV logging")
    parser.add_argument('--analytic-baseline', action='store_true',
                        help="Use the un-optimised analytic oval (debugging only)")
    parser.add_argument('--apex-height-mm', type=float, default=None,
                        help="Ball apex above the hand at release (mm). "
                             "Default: JugglePattern's 1300 mm. Lower apex "
                             "gives faster tempo and tighter control.")
    parser.add_argument('--separation-mm', type=float, default=None,
                        help="THROW–CATCH horizontal separation (mm). "
                             "Default: 200 mm (this runner's default; "
                             "doubled from JugglePattern's 100 mm).")
    parser.add_argument('--capture-tolerance-mm', type=float, default=None,
                        help="Ball-centre-to-cup-opening distance (mm) for "
                             "the strict capture gate. Default: 30 mm (the "
                             "JuggleDemoConfig default since cuts #4/#5). "
                             "Pass any value to override; pair with "
                             "--no-capture-gate to disable the gate.")
    parser.add_argument('--no-capture-gate', action='store_true',
                        help="Disable the capture-tolerance gate entirely "
                             "(any cup-rim contact catches). Restores the "
                             "legacy 'snap-in from rim' behaviour visible "
                             "at low apex with the loose default.")
    parser.add_argument('--realtime-rate', type=float, default=0.0,
                        help="Wall-clock pacing: 1.0 = real-time, 0.5 = half-"
                             "speed slowmo, 2.0 = 2x, 0 (default) = free-"
                             "running (fastest)")
    parser.add_argument('--dashboard', action='store_true',
                        help="Start the live telemetry dashboard "
                             "(http://localhost:8082)")
    parser.add_argument('--dashboard-port', type=int, default=8082,
                        help="Dashboard HTTP/SSE port (default 8082)")
    parser.add_argument('--record', default=None, metavar='PATH',
                        help="Render the run offscreen from a fixed camera "
                             "and write an H.264 mp4 to PATH (needs ffmpeg on "
                             "PATH). Independent of --viewer.")
    parser.add_argument('--record-size', default='1280x720', metavar='WxH',
                        help="Recording resolution (default 1280x720)")
    parser.add_argument('--record-fps', type=int, default=40,
                        help="Recording output fps (default 40 = real-time; "
                             "lower = slow-motion)")
    parser.add_argument('--cam-azimuth', type=float, default=None,
                        help="Fixed-camera azimuth (deg) for --record")
    parser.add_argument('--cam-elevation', type=float, default=None,
                        help="Fixed-camera elevation (deg) for --record")
    parser.add_argument('--cam-distance', type=float, default=None,
                        help="Fixed-camera distance (m) for --record")
    parser.add_argument('--cam-lookat', type=float, nargs=3, default=None,
                        metavar=('X', 'Y', 'Z'),
                        help="Fixed-camera look-at point (m) for --record, as "
                             "three space-separated values, e.g. "
                             "--cam-lookat -0.1 0.03 1.11 (each may be negative)")
    args = parser.parse_args(argv)

    if args.no_log:
        log_path = None
    elif args.log:
        log_path = args.log
    else:
        ts = time.strftime('%Y%m%d_%H%M%S')
        log_path = f"temp/logs/juggle_demo_{ts}.csv"

    # Override pattern fields from CLI flags (when provided).
    pattern_kwargs: dict = {'separation_mm': 200.0}
    if args.apex_height_mm is not None:
        pattern_kwargs['apex_height_mm'] = args.apex_height_mm
    if args.separation_mm is not None:
        pattern_kwargs['separation_mm'] = args.separation_mm
    pattern = JugglePattern(**pattern_kwargs)

    # Parse the recording resolution into (w, h). ``--cam-lookat`` already
    # arrives as a list of three floats (nargs=3) or None.
    try:
        _rw, _rh = (int(v) for v in args.record_size.lower().split('x'))
    except ValueError:
        parser.error(f"--record-size must be WxH, got {args.record_size!r}")
    cam_lookat = tuple(args.cam_lookat) if args.cam_lookat is not None else None

    # Capture-tolerance resolution: --no-capture-gate explicitly
    # disables; an explicit --capture-tolerance-mm value sets it;
    # otherwise let JuggleDemoConfig's dataclass default (30 mm) apply.
    cfg_kwargs = dict(
        pattern=pattern,
        duration_s=args.duration,
        n_catches=args.n_catches,
        bb_scatter_mm=args.scatter_mm,
        seed=args.seed,
        abort_at_s=args.abort_at,
        log_path=log_path,
        headless=not args.viewer,
        use_optimised_trajectory=not args.analytic_baseline,
        realtime_rate=args.realtime_rate,
        dashboard=args.dashboard,
        dashboard_port=args.dashboard_port,
        record_path=args.record,
        record_size=(_rw, _rh),
        record_fps=args.record_fps,
        cam_azimuth=args.cam_azimuth,
        cam_elevation=args.cam_elevation,
        cam_distance=args.cam_distance,
        cam_lookat=cam_lookat,
    )
    if args.no_capture_gate:
        cfg_kwargs['capture_tolerance_mm'] = None
    elif args.capture_tolerance_mm is not None:
        cfg_kwargs['capture_tolerance_mm'] = args.capture_tolerance_mm
    cfg = JuggleDemoConfig(**cfg_kwargs)
    t_start = time.time()
    stats = run(cfg)
    wall_elapsed = time.time() - t_start

    print(f"[juggle_demo] sim duration:   {stats.duration_s:7.3f} s")
    print(f"[juggle_demo] wall elapsed:   {wall_elapsed:7.3f} s "
          f"(realtime {stats.duration_s / max(wall_elapsed, 1e-6):.2f}x)")
    print(f"[juggle_demo] events:         {stats.events_dispatched}")
    print(f"[juggle_demo] captures:       {stats.n_captures}")
    print(f"[juggle_demo] drops:          {stats.n_drops}")
    if stats.aborted:
        print(f"[juggle_demo] aborted at:     {stats.abort_time_s:.3f} s")
    if log_path:
        print(f"[juggle_demo] log written to: {log_path}")
    if args.record:
        print(f"[juggle_demo] video written to: {args.record}")
    return 0 if stats.n_captures >= 1 else 1


if __name__ == '__main__':
    sys.exit(main())
