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

import argparse
import dataclasses
import os
import sys
import time
from pathlib import Path

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
from sim.juggle_tilt import realize_tilted
from sim.plant.mujoco_plant import MuJoCoPlant

CONTROL_DT = 0.025

# GLFW key codes for the MuJoCo passive viewer's key_callback (mirrors
# sim/juggle_demo.py so the two runners share the same viewer controls).
_KEY_SPACE = 32
_KEY_C = 67                # print current free-camera angle as --cam-* flags
_KEY_LEFT_BRACKET = 91     # `[`  — slower
_KEY_RIGHT_BRACKET = 93    # `]`  — faster
_KEY_RIGHT_ARROW = 262     # step one tick when paused
_KEY_LEFT_ARROW = 263      # accepted but no-op (sim can't run backwards)

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
    seed: int = 0                       # accepted for CLI parity; runner is
                                        # deterministic (no RNG) so it's unused
    headless: bool = True               # if False, launch the MuJoCo viewer
    realtime_rate: float = 0.0          # wall-clock pacing: 0 = free-running,
                                        # 1.0 = real-time, 0.5 = half-speed, etc.
    # ---- Offscreen video recording (mirrors sim/juggle_demo.py --record) ----
    # When ``record_path`` is set, render the live scene offscreen from a FIXED
    # free camera each tick and pipe frames to the system ffmpeg (H.264 mp4),
    # independent of ``headless`` / the viewer. ``cam_*`` override the model's
    # default free-camera angle; leave any None to inherit it.
    record_path: "str | None" = None
    record_size: "tuple[int, int]" = (1280, 720)   # (width, height) px
    record_fps: int = 40                # output fps; 40 = real-time (1 frame/tick)
    cam_azimuth: "float | None" = None
    cam_elevation: "float | None" = None
    cam_distance: "float | None" = None
    cam_lookat: "tuple[float, float, float] | None" = None


def _flight_and_speed(apex_m, dz_m):
    """Flight time + vertical launch speed for a ball thrown up ``apex_m`` above
    the release that lands ``dz_m`` below it (catch lower than throw)."""
    g = -GRAVITY[2]
    v_up = np.sqrt(2 * g * apex_m)                      # speed to reach apex
    t_up = v_up / g
    t_down = np.sqrt(2 * (apex_m + dz_m) / g)           # apex back down to catch
    return t_up + t_down, v_up


def realize(cup_m, rx: float = 0.0, ry: float = 0.0):
    """Cup Cartesian target (m, world) → (pose_6dof_mm, slider_mm).

    Level (``rx == ry == 0``) is the original level-platform decoupling and is
    byte-for-byte unchanged (the existing 2-ball runner relies on it). A non-zero
    tilt (Rung 2a) re-introduces orientation with the Rung-0 lever-arm
    compensation — it delegates to :func:`sim.juggle_tilt.realize_tilted`, which
    reduces EXACTLY to the level form at zero tilt (zero lateral shift, zero
    vertical drop), so this is the single realisation for both the level runner
    and the tilt-aimed throw/catch."""
    cup_m = np.asarray(cup_m, float)
    return realize_tilted(cup_m[:2], float(cup_m[2]), rx, ry)


# --------------------------------------------------------------------------
# Offscreen video recording — adapted verbatim from sim/juggle_demo.py's
# ``_build_record_camera`` / ``_VideoRecorder``. Duplicated (rather than
# imported) to keep this runner standalone — importing juggle_demo would pull
# in the CasADi offline optimiser. UNIFY into a shared module when juggle_demo
# is retired (the online runner is meant to replace it).
# --------------------------------------------------------------------------
def build_record_camera(model, cfg: "OnlineJuggleConfig"):
    """Fixed free camera for ``--record``: the model's default free camera with
    any user-supplied ``cam_*`` field overridden (None inherits the default)."""
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


class VideoRecorder:
    """Render the live MuJoCo scene offscreen and stream it to ffmpeg.

    One frame per control tick from a FIXED free camera; raw RGB is piped to
    the system ``ffmpeg`` (H.264 mp4) over stdin — needs only ``mujoco`` + an
    ``ffmpeg`` on PATH (no imageio-ffmpeg). Shares the plant's live model/data
    so the drawn frame is exactly the post-step state.
    """

    def __init__(self, model, data, path, *, width, height, fps, cam):
        import subprocess
        self._data = data
        self._cam = cam
        # The MJCF's default offscreen framebuffer is 640x480; rendering larger
        # raises. Grow the offscreen buffer to fit (no effect on physics or the
        # onscreen viewer) before building the renderer.
        if width > model.vis.global_.offwidth:
            model.vis.global_.offwidth = width
        if height > model.vis.global_.offheight:
            model.vis.global_.offheight = height
        self._renderer = mujoco.Renderer(model, height=height, width=width)
        self.path = path
        self.frames = 0
        Path(path).parent.mkdir(parents=True, exist_ok=True)
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
        # ---- viewer / wall-clock pacing (the module-level run() attaches the
        # passive viewer to self.viewer before calling run()) ----
        self.viewer = None
        self._recorder = None               # set by the module-level run()
        self._realtime_rate = float(cfg.realtime_rate)
        self._paused = False
        self._pending_step = False
        self._wall_anchor = None            # monotonic time the pacing schedule
        self._sched_steps = 0               # was anchored, and ticks since then
        self._last_rate = self._realtime_rate

    # ---- cup state ----
    def cup_world_m(self):
        return self.plant.data.site_xpos[self.site_id].copy()

    def cup_vel_world_m(self):
        res = np.zeros(6)
        mujoco.mj_objectVelocity(self.plant.model, self.plant.data,
                                 mujoco.mjtObj.mjOBJ_SITE, self.site_id, res, 0)
        return res[3:6].copy()

    # ---- viewer controls (mirrors sim/juggle_demo.py) ----
    def key_callback(self, keycode: int) -> None:
        """Handle a keypress from the MuJoCo passive viewer.

        SPACE pause/resume · RIGHT step one tick (paused) · ``[`` slower /
        ``]`` faster (jumps out of free-running first) · LEFT no-op.
        Camera is the viewer's built-in mouse drag/scroll.
        """
        if keycode == _KEY_SPACE:
            self._paused = not self._paused
            print(f"[juggle_online] {'paused' if self._paused else 'running'} "
                  f"at sim_time {float(self.plant.data.time):.3f} s")
        elif keycode == _KEY_RIGHT_ARROW:
            if self._paused:
                self._pending_step = True
        elif keycode in (_KEY_LEFT_BRACKET, _KEY_RIGHT_BRACKET):
            r = self._realtime_rate if self._realtime_rate > 0.0 else 1.0
            r = max(r * 0.5, 0.0625) if keycode == _KEY_LEFT_BRACKET else min(r * 2.0, 16.0)
            self._realtime_rate = r
            print(f"[juggle_online] realtime rate = {r:.3f}x")
        elif keycode == _KEY_C:
            # Print the current free-camera angle as ready-to-paste --cam-*
            # flags, so the user can record a fixed-camera video from the angle
            # they just dragged to (mirrors juggle_demo's C binding).
            h = self.viewer
            if h is not None:
                c = h.cam
                print("[juggle_online] camera: "
                      f"--cam-azimuth {c.azimuth:.1f} "
                      f"--cam-elevation {c.elevation:.1f} "
                      f"--cam-distance {c.distance:.3f} "
                      f"--cam-lookat {c.lookat[0]:.3f} "
                      f"{c.lookat[1]:.3f} {c.lookat[2]:.3f}")
        elif keycode == _KEY_LEFT_ARROW:
            pass   # documented no-op

    def _tick_boundary(self) -> None:
        """Called after every control tick: record a frame (if recording), keep
        the viewer responsive, honour pause / single-step, and pace to
        ``realtime_rate``. No-op when headless, not recording, and free-running
        (so tests / probes integrate at full speed)."""
        # Record one frame per tick (independent of the viewer / pacing — works
        # headless). Captured AFTER the physics step, so it's the post-step state.
        if self._recorder is not None:
            self._recorder.capture()
        v = self.viewer
        if v is None and self._realtime_rate <= 0.0:
            return
        if v is not None and not v.is_running():
            self.viewer = None              # window closed → finish fast/headless
            return
        # Pause spin: freeze the sim while keeping the viewer interactive.
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
            self._tick_boundary()
        self.bm.ball(0).spawn_in_hand()                 # seat ball 0 at catch_pos
        plant.set_contact_stiffness(False)              # soft: carry
        n_carry = int(round((self.period - self.catch_offset) / CONTROL_DT))
        tc0 = float(plant.data.time)
        for k in range(n_carry):
            tt = self.catch_offset + (float(plant.data.time) - tc0)
            plant.step(CONTROL_DT,
                       hand_cmd_fn=lambda t, p=plan, b=tc0: self._slider_at(p, self.catch_offset + (t - b)),
                       plat_cmd_fn=lambda t, p=plan, b=tc0: self._drive(p, self.catch_offset + (t - b)))
            self._tick_boundary()

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
                self._tick_boundary()
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
    """Run the online juggle. Launches the MuJoCo passive viewer when
    ``cfg.headless`` is False (mouse = camera; keyboard = pause/step/speed/`C`)
    and/or an offscreen mp4 recorder when ``cfg.record_path`` is set; otherwise
    integrates headless at full speed. Returns ``(captures, drops)``."""
    cfg = cfg or OnlineJuggleConfig()
    runner = OnlineJuggleRunner(cfg)
    viewer_handle = None
    try:
        if not cfg.headless:
            import mujoco.viewer  # type: ignore
            viewer_handle = mujoco.viewer.launch_passive(
                runner.plant.model, runner.plant.data,
                key_callback=runner.key_callback)
            runner.viewer = viewer_handle
            print("[juggle_online] keyboard: SPACE pause/resume   RIGHT step "
                  "(when paused)   [ slower / ] faster   C print-camera   "
                  "(mouse-drag = camera)")
        if cfg.record_path:
            runner._recorder = VideoRecorder(
                runner.plant.model, runner.plant.data, cfg.record_path,
                width=cfg.record_size[0], height=cfg.record_size[1],
                fps=cfg.record_fps, cam=build_record_camera(runner.plant.model, cfg))
            print(f"[juggle_online] recording to {cfg.record_path}")
        return runner.run()
    finally:
        if runner._recorder is not None:
            n = runner._recorder.frames
            runner._recorder.close()
            print(f"[juggle_online] video written: {cfg.record_path} ({n} frames)")
        if viewer_handle is not None:
            viewer_handle.close()


def main(argv: "list[str] | None" = None) -> int:
    """CLI mirroring sim/juggle_demo.py's viewer-relevant flags. The juggle_demo
    args that depend on offline-demo machinery (BB scatter/abort, capture gate,
    analytic baseline, dashboard, CSV log) are intentionally omitted — they have
    no analogue in the online runner; the throw/catch pattern + viewer flags do."""
    p = argparse.ArgumentParser(
        description="Run the online re-planning two-ball juggle in MuJoCo.")
    p.add_argument('--duration', type=float, default=20.0,
                   help="Simulation duration in seconds (default 20)")
    p.add_argument('--viewer', action='store_true',
                   help="Launch the MuJoCo passive viewer (mouse = camera)")
    p.add_argument('--realtime-rate', type=float, default=None,
                   help="Wall-clock pacing: 1.0 = real-time, 0.5 = half-speed "
                        "slowmo, 2.0 = 2x, 0 = free-running (fastest). Default: "
                        "1.0 with --viewer (so you can watch it), else 0.")
    p.add_argument('--seed', type=int, default=0,
                   help="RNG seed (accepted for parity; runner is deterministic)")
    p.add_argument('--apex-height-mm', type=float, default=None,
                   help="Ball apex above the throw (mm). Default 1300.")
    p.add_argument('--separation-mm', type=float, default=None,
                   help="THROW-CATCH lateral separation (mm). Default 100.")
    p.add_argument('--throw-z-mm', type=float, default=None,
                   help="Cup release height (mm, cup-world). Default 850.")
    p.add_argument('--catch-z-mm', type=float, default=None,
                   help="Cup catch height (mm, cup-world). Default 700.")
    # ---- recording (mirrors sim/juggle_demo.py) ----
    p.add_argument('--record', default=None, metavar='PATH',
                   help="Render offscreen from a fixed camera and write an "
                        "H.264 mp4 to PATH (needs ffmpeg on PATH). Independent "
                        "of --viewer.")
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
                   help="Fixed-camera look-at point (m) for --record, as three "
                        "space-separated values (each may be negative). Press C "
                        "in --viewer to print the current angle as these flags.")
    args = p.parse_args(argv)

    # Default pacing: real-time when viewing (so it's watchable), else free-run.
    rate = args.realtime_rate
    if rate is None:
        rate = 1.0 if args.viewer else 0.0

    try:
        _rw, _rh = (int(v) for v in args.record_size.lower().split('x'))
    except ValueError:
        p.error(f"--record-size must be WxH, got {args.record_size!r}")

    kw: dict = dict(
        duration_s=args.duration, seed=args.seed,
        headless=not args.viewer, realtime_rate=rate,
        record_path=args.record, record_size=(_rw, _rh), record_fps=args.record_fps,
        cam_azimuth=args.cam_azimuth, cam_elevation=args.cam_elevation,
        cam_distance=args.cam_distance,
        cam_lookat=tuple(args.cam_lookat) if args.cam_lookat is not None else None,
    )
    if args.apex_height_mm is not None:
        kw['apex_m'] = args.apex_height_mm / 1000.0
    if args.separation_mm is not None:
        kw['separation_m'] = args.separation_mm / 1000.0
    if args.throw_z_mm is not None:
        kw['throw_z_m'] = args.throw_z_mm / 1000.0
    if args.catch_z_mm is not None:
        kw['catch_z_m'] = args.catch_z_mm / 1000.0
    cfg = OnlineJuggleConfig(**kw)

    t_start = time.time()
    caps, drops = run(cfg)
    wall = time.time() - t_start
    print(f"[juggle_online] captures: {len(caps)}  drops: {len(drops)}")
    print(f"[juggle_online] sim {cfg.duration_s:.1f}s  wall {wall:.1f}s "
          f"(realtime {cfg.duration_s / max(wall, 1e-6):.2f}x)")
    if caps:
        print(f"[juggle_online] capture times: "
              f"{[round(t, 2) for t, _ in caps]}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
