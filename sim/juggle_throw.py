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
  on a :func:`~sim.juggle_planner.juggle_planner.plan_cup_cycle` trajectory (the
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

import argparse
import dataclasses
import os
import sys
import time

import numpy as np

# Single path bootstrap (repo root, ros_ws pkg, config/generated);
# see sim/_paths.py.  Runnable entry scripts only — library modules under
# sim/ never touch sys.path.
_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
from sim._paths import bootstrap_paths  # noqa: E402
bootstrap_paths()

from jugglebot import hardware_config as _hw  # noqa: E402

import mujoco

from sim.juggle_planner.juggle_planner import (
    GRAVITY, PlannerConfig, ballistic_touchdown, plan_cup_cycle, takeoff_velocity,
)
from sim.juggle_noise import BallisticEstimator, JuggleNoise, NoiseConfig
from sim.juggle_tilt import cup_axis, realize_tilted, tilt_to_throw, MAX_TILT_DEG
from sim.plant.mujoco_plant import MuJoCoPlant
# Viewer + offscreen-recording plumbing is shared with the two-ball runner (it
# needs only mujoco + ffmpeg, no CasADi). Importing juggle_online adds no heavy
# deps here — it pulls the same juggle_planner / juggle_tilt / plant modules this
# file already imports. See sim/juggle_online.py for the VideoRecorder rationale.
from sim.juggle_online import (
    VideoRecorder, build_record_camera,
    _KEY_SPACE, _KEY_C, _KEY_LEFT_BRACKET, _KEY_RIGHT_BRACKET,
    _KEY_RIGHT_ARROW, _KEY_LEFT_ARROW,
)

CONTROL_DT = 0.025
CUP_Z_BASE_MM = 659.6
# Slider travel [0, stroke] mm — DERIVED (see sim/juggle_tilt.py for why; it read
# a hardcoded 355.0 until 2026-08-21, 10.25 mm above what the plant will execute).
SLIDER_STROKE_MM = float(_hw.GEOM_HAND_STROKE_MM)
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
    # ---- viewer / offscreen recording (mirror sim/juggle_online.py; PURELY
    # additive — the headless run_single_throw() never reads these, so it stays
    # byte-identical. Only the module-level run()/main() below use them.) --------
    headless: bool = True          # if False, launch the MuJoCo passive viewer
    realtime_rate: float = 0.0     # wall-clock pacing: 0 = free-run, 1.0 = real-time
    record_path: "str | None" = None
    record_size: "tuple[int, int]" = (1280, 720)   # (width, height) px
    record_fps: int = 40           # output fps; 40 = real-time (1 frame/tick)
    cam_azimuth: "float | None" = None
    cam_elevation: "float | None" = None
    cam_distance: "float | None" = None
    cam_lookat: "tuple[float, float, float] | None" = None


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
        # ---- viewer / wall-clock pacing. Only the module-level run() attaches a
        # viewer/recorder and wraps plant.step to call _tick_boundary; the headless
        # run() never does, so run_single_throw() is byte-identical. ----
        self._recorder = None
        self._realtime_rate = float(cfg.realtime_rate)
        self._paused = False
        self._pending_step = False
        self._wall_anchor = None
        self._sched_steps = 0
        self._last_rate = self._realtime_rate

    def cup_world_m(self):
        return self.plant.data.site_xpos[self.site_id].copy()

    def cup_vel_world_m(self):
        res = np.zeros(6)
        mujoco.mj_objectVelocity(self.plant.model, self.plant.data,
                                 mujoco.mjtObj.mjOBJ_SITE, self.site_id, res, 0)
        return res[3:6].copy()

    # ---- viewer controls (mirrors sim/juggle_online.py) ----
    def key_callback(self, keycode: int) -> None:
        """SPACE pause/resume · RIGHT step one tick (paused) · ``[`` slower /
        ``]`` faster · ``C`` print current free-camera angle as --cam-* flags."""
        if keycode == _KEY_SPACE:
            self._paused = not self._paused
            print(f"[juggle_throw] {'paused' if self._paused else 'running'} "
                  f"at sim_time {float(self.plant.data.time):.3f} s")
        elif keycode == _KEY_RIGHT_ARROW:
            if self._paused:
                self._pending_step = True
        elif keycode in (_KEY_LEFT_BRACKET, _KEY_RIGHT_BRACKET):
            r = self._realtime_rate if self._realtime_rate > 0.0 else 1.0
            r = max(r * 0.5, 0.0625) if keycode == _KEY_LEFT_BRACKET else min(r * 2.0, 16.0)
            self._realtime_rate = r
            print(f"[juggle_throw] realtime rate = {r:.3f}x")
        elif keycode == _KEY_C:
            h = self.viewer
            if h is not None:
                c = h.cam
                print("[juggle_throw] camera: "
                      f"--cam-azimuth {c.azimuth:.1f} "
                      f"--cam-elevation {c.elevation:.1f} "
                      f"--cam-distance {c.distance:.3f} "
                      f"--cam-lookat {c.lookat[0]:.3f} "
                      f"{c.lookat[1]:.3f} {c.lookat[2]:.3f}")
        elif keycode == _KEY_LEFT_ARROW:
            pass   # documented no-op (sim can't run backwards)

    def _tick_boundary(self) -> None:
        """Called after every control tick (via the wrapped plant.step in the
        module-level run()): record a frame (if recording), keep the viewer
        responsive, honour pause / single-step, and pace to ``realtime_rate``.
        No-op when headless, not recording, and free-running."""
        if self._recorder is not None:
            self._recorder.capture()
        v = self.viewer
        if v is None and self._realtime_rate <= 0.0:
            return
        if v is not None and not v.is_running():
            self.viewer = None              # window closed → finish fast/headless
            return
        while v is not None and self._paused and not self._pending_step:
            v.sync()
            time.sleep(0.01)
            self._wall_anchor = None        # re-anchor the schedule on resume
            if not v.is_running():
                self.viewer = None
                return
        stepped_one_frame = self._pending_step
        self._pending_step = False
        if v is not None:
            v.sync()
        if stepped_one_frame:
            return                          # single-frame step is instantaneous
        rate = self._realtime_rate
        if rate > 0.0:
            if self._wall_anchor is None or rate != self._last_rate:
                self._wall_anchor = time.monotonic()
                self._sched_steps = 0
                self._last_rate = rate
            self._sched_steps += 1
            target = self._wall_anchor + self._sched_steps * (CONTROL_DT / rate)
            remaining = target - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)

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


def run(cfg: "SingleThrowConfig | None" = None, tail_timeout_s: float = 2.0) -> ThrowResult:
    """Run one throw WITH the MuJoCo passive viewer (``cfg.headless`` False) and/or
    an offscreen mp4 recorder (``cfg.record_path`` set); otherwise integrates
    headless at full speed. Returns the :class:`ThrowResult`.

    The gated ``SingleThrowRunner.run()`` loop is left untouched: instead of
    editing it, this wrapper wraps ``runner.plant.step`` so every physics tick
    drives a viewer/record "tick boundary" (sync + pace + frame capture). Because
    the wrapper is only installed here, the headless ``run_single_throw()`` path
    never sees it and stays byte-identical.

    ``SingleThrowRunner.run()`` returns ~10 recovery ticks after release — BEFORE
    the ball lands (the landing is extrapolated). For a watchable/recorded run we
    then keep stepping (holding the cup at its final recovered pose) until the ball
    centre passes below ``catch_z`` or ``tail_timeout_s`` elapses, so the operator
    sees the full parabola. This tail runs ONLY in viewer/record mode."""
    cfg = cfg or SingleThrowConfig()
    runner = SingleThrowRunner(cfg)
    viewer_handle = None
    try:
        if not cfg.headless:
            import mujoco.viewer  # type: ignore
            viewer_handle = mujoco.viewer.launch_passive(
                runner.plant.model, runner.plant.data,
                key_callback=runner.key_callback)
            runner.viewer = viewer_handle
            print("[juggle_throw] keyboard: SPACE pause/resume   RIGHT step "
                  "(when paused)   [ slower / ] faster   C print-camera   "
                  "(mouse-drag = camera)")
        if cfg.record_path:
            runner._recorder = VideoRecorder(
                runner.plant.model, runner.plant.data, cfg.record_path,
                width=cfg.record_size[0], height=cfg.record_size[1],
                fps=cfg.record_fps, cam=build_record_camera(runner.plant.model, cfg))
            print(f"[juggle_throw] recording to {cfg.record_path}")

        # Wrap plant.step so each sim step drives a viewer/record tick boundary,
        # WITHOUT editing the gated run() loop. Headless never installs this.
        _orig_step = runner.plant.step

        def _stepped(*args, **kwargs):
            _orig_step(*args, **kwargs)
            runner._tick_boundary()

        runner.plant.step = _stepped

        result = runner.run()

        # ---- Watchable landing tail (viewer/record ONLY — never headless). Hold
        # the cup at its final recovered pose (plant.step with no cmd fns leaves the
        # position actuators at the last recovery command) and keep stepping until
        # the ball falls below catch_z or the timeout, syncing/recording each tick.
        if viewer_handle is not None or runner._recorder is not None:
            catch_z_mm = cfg.catch_z_m * 1000.0
            for _ in range(max(1, int(round(tail_timeout_s / CONTROL_DT)))):
                if viewer_handle is not None and not viewer_handle.is_running():
                    break
                runner.plant.step(CONTROL_DT)      # hold last recovered pose
                bs = runner.plant.get_ball_state(ball=0)
                if bs is not None and bs.position_mm[2] < catch_z_mm:
                    break
        return result
    finally:
        if runner._recorder is not None:
            n = runner._recorder.frames
            runner._recorder.close()
            print(f"[juggle_throw] video written: {cfg.record_path} ({n} frames)")
        if viewer_handle is not None:
            viewer_handle.close()


def main(argv: "list[str] | None" = None) -> int:
    """CLI to watch/record a single tilt-aimed throw (mirrors the viewer/record
    flags of sim/juggle_online.py, plus the throw's target/height/cadence knobs)."""
    p = argparse.ArgumentParser(
        description="Run one tilt-aimed single-ball throw in MuJoCo (Rung 2a).")
    p.add_argument('--duration', type=float, default=2.0,
                   help="Max seconds of post-release landing tail to render, so "
                        "the full parabola is watchable (default 2)")
    p.add_argument('--viewer', action='store_true',
                   help="Launch the MuJoCo passive viewer (mouse = camera)")
    p.add_argument('--realtime-rate', type=float, default=None,
                   help="Wall-clock pacing: 1.0 = real-time, 0.5 = half-speed "
                        "slowmo, 2.0 = 2x, 0 = free-running (fastest). Default: "
                        "1.0 with --viewer, else 0.")
    p.add_argument('--seed', type=int, default=0,
                   help="§3 tracking-noise seed (only perturbs the OBSERVED "
                        "landing; the true throw is deterministic)")
    # ---- throw geometry / cadence ----
    p.add_argument('--target-x-mm', type=float, default=100.0,
                   help="Desired landing x (mm, cup-world). Default 100.")
    p.add_argument('--target-y-mm', type=float, default=0.0,
                   help="Desired landing y (mm, cup-world). Default 0.")
    p.add_argument('--throw-z-mm', type=float, default=None,
                   help="Cup release height (mm, cup-world). Default 850.")
    p.add_argument('--catch-z-mm', type=float, default=None,
                   help="Landing / measure height (mm, cup-world). Default 700.")
    p.add_argument('--flight-s', type=float, default=0.60,
                   help="Cadence: time of flight in seconds. Default 0.60.")
    # ---- recording (mirrors sim/juggle_online.py) ----
    p.add_argument('--record', default=None, metavar='PATH',
                   help="Render offscreen from a fixed camera and write an H.264 "
                        "mp4 to PATH (needs ffmpeg on PATH). Independent of --viewer.")
    p.add_argument('--record-size', default='1280x720', metavar='WxH',
                   help="Recording resolution (default 1280x720)")
    p.add_argument('--record-fps', type=int, default=40,
                   help="Recording output fps (default 40 = real-time; "
                        "lower = slow-motion)")
    p.add_argument('--cam-azimuth', type=float, default=None,
                   help="Fixed-camera azimuth (deg) for --record")
    p.add_argument('--cam-elevation', type=float, default=None,
                   help="Fixed-camera elevation (deg) for --record")
    p.add_argument('--cam-distance', type=float, default=None,
                   help="Fixed-camera distance (m) for --record")
    p.add_argument('--cam-lookat', type=float, nargs=3, default=None,
                   metavar=('X', 'Y', 'Z'),
                   help="Fixed-camera look-at point (m) for --record, three "
                        "space-separated values. Press C in --viewer to print "
                        "the current angle as these flags.")
    args = p.parse_args(argv)

    rate = args.realtime_rate
    if rate is None:
        rate = 1.0 if args.viewer else 0.0

    try:
        _rw, _rh = (int(v) for v in args.record_size.lower().split('x'))
    except ValueError:
        p.error(f"--record-size must be WxH, got {args.record_size!r}")

    kw: dict = dict(
        seed=args.seed,
        headless=not args.viewer, realtime_rate=rate,
        record_path=args.record, record_size=(_rw, _rh), record_fps=args.record_fps,
        cam_azimuth=args.cam_azimuth, cam_elevation=args.cam_elevation,
        cam_distance=args.cam_distance,
        cam_lookat=tuple(args.cam_lookat) if args.cam_lookat is not None else None,
        target_xy_m=(args.target_x_mm / 1000.0, args.target_y_mm / 1000.0),
        flight_s=args.flight_s,
    )
    if args.throw_z_mm is not None:
        kw['throw_z_m'] = args.throw_z_mm / 1000.0
    if args.catch_z_mm is not None:
        kw['catch_z_m'] = args.catch_z_mm / 1000.0
    cfg = SingleThrowConfig(**kw)

    r = run(cfg, tail_timeout_s=args.duration)
    tgt = tuple(round(v * 1000.0, 1) for v in r.target_xy_m)
    land = tuple(round(v * 1000.0, 1) for v in r.landing_xy_m)
    print(f"[juggle_throw] target {tgt} mm   true landing {land} mm   "
          f"error {r.error_mm:.1f} mm   separated {r.separated}   "
          f"tilt {r.tilt_deg:.2f} deg")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
