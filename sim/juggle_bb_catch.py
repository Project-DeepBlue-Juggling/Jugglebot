"""Interactive Ball-Butler -> Jugglebot single-catch tool.

An operator-driven harness for **visually verifying that Jugglebot catches a Ball
Butler throw smoothly**. The operator sets the throw's launch POSITION and initial
VELOCITY VECTOR with keypresses, presses ``R`` to release, and watches Jugglebot
reach, tilt, and seat the ball in the MuJoCo viewer. Multiple throws per session
(adjust between throws). The §3 noise (2% BB throw + 0.5 mm tracking) is ON.

The catch itself is NOT re-implemented here — it reuses the validated Rung-1 catch
(:class:`sim.juggle_catch.SingleCatchRunner`, the phase-matched fast-capable seat
from the 2026-07-02 fast-catch-fidelity work). This tool only chooses the *throw*
(via the additive ``SingleCatchConfig.manual_launch_mm`` field, default-off) and
attaches the passive viewer by wrapping ``plant.step`` with a tick-boundary
closure — the same non-invasive pattern the single-ball CLIs use. It never edits
the catch controller, contact, or seat.

Launch / velocity model
-----------------------
* **Launch position** = the ball's release point (world mm). Init = the *real Ball
  Butler* release point for a throw from the demo placement ``(-872,-630,1430)`` mm
  aimed at ``(0,0,catch_z)`` — so the default preset lands at the catch point.
  (Spawning from the raw BB placement instead misses by ~120 mm; the release point
  is the point that lands on target — see the harness header comment.)
* **Velocity** = the ball's initial velocity (world mm/s). Init = that same real-BB
  release velocity (the preset). Nudge it to test near-misses.
* The landing is wherever the ballistics take it — the preset lands on the catch
  point; any nudge is a deliberate near-miss.

Keys (interactive ``--viewer``)
-------------------------------
    POSITION (launch point, +-20 mm):   i/k = +x/-x    l/j = +y/-y    u/o = +z/-z
    VELOCITY (+-75 mm/s):                f/h = +vx/-vx  t/g = +vy/-vy  v/b = +vz/-vz
    R = release (run one throw + catch)  P = reset launch to the preset
    SPACE pause/resume   RIGHT step (paused)   [ slower / ] faster   C print-camera
    (mouse-drag / scroll = camera)

(``v``/``b`` drive vz because the viewer key callback receives GLFW *physical* key
codes with no shift distinction, so ``r`` and ``R`` collide — ``R`` is reserved for
release.)

The letter controls were checked against MuJoCo's built-in viewer shortcuts: on
mujoco 3.2.3 the visualisation/render *flag* toggles are UI-only (``mjVISSTRING`` /
``mjRNDSTRING`` expose no key shortcuts), so the nudge letters do not clash with
them. MuJoCo's reserved *navigation* keys are Space (pause), Backspace (reset),
the arrows (step), Tab (UI), Esc (free-camera), ``-`` / ``=`` (speed) and the
digits 0-9 (geom/site groups) — the tool avoids those for its controls, though it
intentionally shares Space (pause) / arrows (step) with the viewer's own handling
for consistency with the other juggle sims. (Bindings can differ across MuJoCo
versions — confirm on-screen.)

Spawn visibility
----------------
By default the ball is spawned at the trajectory's APEX (descending, well above the
cup) — the proven, reliable path the headless catch relies on (avoids a rising arc
clipping the cup). ``--full-flight`` instead spawns from the release point so the
operator sees the ball fly the whole arc from the Butler; it works for the demo
placement (far/high, so the rising arc clears the cup) but NEAR/LOW launch points
can clip the cup on the way up — use apex (default) for those. ``--full-flight`` is
implemented purely at the harness level (an instance-level ``_spawn_incoming``
override); ``sim/juggle_catch.py`` gains only the one ``manual_launch_mm`` field.

Headless (``--throws N``) runs scripted throws (the preset, or ``--vx/--vy/--vz`` /
``--landing-x-mm/--landing-y-mm`` overrides) and prints each catch result — how the
parent + tests exercise this without a display.

Pure-Python, no ROS2. SI internally; mm at the plant boundary.
"""
from __future__ import annotations

import argparse
import dataclasses
import math
import os
import sys
import time

import mujoco
import numpy as np

_sim_dir = os.path.dirname(os.path.abspath(__file__))
_repo_root = os.path.dirname(_sim_dir)
for _p in (_sim_dir, _repo_root,
           os.path.join(_repo_root, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_repo_root, 'config', 'generated')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from sim.ball_butler.sim import BallButlerSim
from sim.juggle_catch import CONTROL_DT, SingleCatchConfig, SingleCatchRunner
from sim.juggle_noise import JuggleNoise
from sim.juggle_tilt import realize_tilted
# Reuse the viewer / recording plumbing verbatim (do NOT re-implement it).
from sim.juggle_online import (
    VideoRecorder, build_record_camera,
    _KEY_SPACE, _KEY_C, _KEY_LEFT_BRACKET, _KEY_RIGHT_BRACKET,
    _KEY_RIGHT_ARROW, _KEY_LEFT_ARROW,
)

# ---- The demo Ball Butler placement (world mm), aimed from across the room. ----
# Matches sim/juggle_catch.py's use_real_bb default + tools/probes/juggle_fastcatch.py.
DEMO_BB_PLACEMENT_MM = np.array([-872.0, -630.0, 1430.0])
DEMO_BB_YAW_RAD = math.atan2(630.0, 872.0)      # == atan2(-y, -x): aim at origin

# ---- GLFW physical key codes (uppercase-ASCII) for the operator controls. ----
_KEY_I, _KEY_J, _KEY_K, _KEY_L, _KEY_O, _KEY_U = 73, 74, 75, 76, 79, 85
_KEY_F, _KEY_G, _KEY_H, _KEY_T, _KEY_V, _KEY_B = 70, 71, 72, 84, 86, 66
_KEY_P, _KEY_R = 80, 82

POS_STEP_MM = 20.0        # launch-position nudge per keypress
VEL_STEP_MMS = 75.0       # velocity nudge per keypress (within the 50-100 mm/s band)

# Default --viewer camera (a good three-quarter view of Jugglebot + the catch). Any
# --cam-* CLI flag overrides its component; press C in the viewer to print a new angle.
DEFAULT_VIEW_AZIMUTH = -53.0
DEFAULT_VIEW_ELEVATION = -32.2
DEFAULT_VIEW_DISTANCE = 3.817
DEFAULT_VIEW_LOOKAT = (0.058, 0.0, 0.890)

# Release-state preview (drawn in the viewer's user scene while idle, before release).
BALL_R_M = 0.035          # ball radius (matches the MJCF ball geom)
ARROW_SCALE_S = 0.12      # velocity arrow length = |v| * this (s) -> a visible arrow
ARROW_WIDTH_M = 0.008     # velocity arrow shaft width


def _preset_launch(catch_z_m: float, target_xy_mm=(0.0, 0.0)):
    """The real-BB release ``(pos_mm, vel_mms)`` for a throw from the demo placement
    aimed at ``(target_xy, catch_z)`` — the on-target preset. Spawning from THIS
    release point lands at the catch point (spawning from the raw placement misses
    by ~120 mm)."""
    bb = BallButlerSim.from_hardware_config(DEMO_BB_PLACEMENT_MM, DEMO_BB_YAW_RAD)
    target_mm = np.array([float(target_xy_mm[0]), float(target_xy_mm[1]),
                          catch_z_m * 1000.0])
    rel_pos_mm, rel_vel_mms, _ = bb.compute_release_state(target_mm)
    return np.asarray(rel_pos_mm, float).copy(), np.asarray(rel_vel_mms, float).copy()


def _flight_window_s(pos_mm, vel_mms, catch_z_m: float, full_flight: bool) -> float:
    """A generous loop-length ``flight_s`` covering the launch -> catch_z descent.

    Only sets the number of catch-loop ticks (``flight_s + post_td_margin_s``); the
    catch timing is driven live by the ballistic estimator, so this just has to be
    long enough. Full-flight covers release -> apex -> catch_z; apex-mode covers
    apex -> catch_z (never shorter than the proven 0.60 default)."""
    g = 9.806
    pz = float(pos_mm[2]) / 1000.0
    vz = float(vel_mms[2]) / 1000.0
    if full_flight:
        # descending root of  pz + vz t - .5 g t^2 = catch_z
        a, b, c = -0.5 * g, vz, pz - catch_z_m
        disc = b * b - 4.0 * a * c
        if disc <= 0.0:
            return 0.60
        t = (-b - math.sqrt(disc)) / (2.0 * a)   # larger root (a < 0)
        return max(t + 0.15, 0.60)
    apex_z = pz + (vz * vz / (2.0 * g) if vz > 0.0 else 0.0)
    t_fall = math.sqrt(max(2.0 * (apex_z - catch_z_m) / g, 0.0))
    return max(t_fall + 0.10, 0.60)


@dataclasses.dataclass
class BBCatchConfig:
    catch_z_m: float = SingleCatchConfig.catch_z_m       # 0.84, the cup catch height
    seed: int = 0
    headless: bool = True
    throws: int = 1                       # headless: number of scripted throws
    realtime_rate: float = 0.0            # 0 = free-run, 1.0 = real-time, ...
    full_flight: bool = False             # spawn from the release point (visual)
    # ---- scripted-throw overrides (headless) ----
    target_xy_mm: "tuple[float, float]" = (0.0, 0.0)     # preset aim (landing)
    vx_mms: "float | None" = None
    vy_mms: "float | None" = None
    vz_mms: "float | None" = None
    # ---- offscreen recording (mirrors sim/juggle_online.py) ----
    record_path: "str | None" = None
    record_size: "tuple[int, int]" = (1280, 720)
    record_fps: int = 40
    cam_azimuth: "float | None" = None
    cam_elevation: "float | None" = None
    cam_distance: "float | None" = None
    cam_lookat: "tuple[float, float, float] | None" = None


class BBCatchTool:
    """Owns ONE :class:`SingleCatchRunner` (one plant) for the whole session, so the
    passive viewer can attach to it once. Per throw it sets the runner's
    ``manual_launch_mm`` from the operator's launch state and calls ``run()`` (which
    resets the plant, warms at ready, spawns, and catches). ``plant.step`` is wrapped
    with a tick-boundary closure so the viewer stays live / paced during the catch —
    the catch loop itself is untouched."""

    def __init__(self, cfg: BBCatchConfig):
        self.cfg = cfg
        base = SingleCatchConfig(
            catch_z_m=cfg.catch_z_m, seed=cfg.seed,
            landing_xy_m=(cfg.target_xy_mm[0] / 1000.0, cfg.target_xy_mm[1] / 1000.0))
        self.runner = SingleCatchRunner(base)
        self.plant = self.runner.plant

        # Operator-controlled launch state (the on-target preset by default).
        self.launch_pos_mm, self.vel_mms = _preset_launch(cfg.catch_z_m, cfg.target_xy_mm)
        if cfg.vx_mms is not None:
            self.vel_mms[0] = float(cfg.vx_mms)
        if cfg.vy_mms is not None:
            self.vel_mms[1] = float(cfg.vy_mms)
        if cfg.vz_mms is not None:
            self.vel_mms[2] = float(cfg.vz_mms)
        self._preset_pos = self.launch_pos_mm.copy()
        self._preset_vel = self.vel_mms.copy()

        # Viewer / wall-clock pacing state (mirrors sim/juggle_online.py).
        self.viewer = None
        self._recorder = None
        self._realtime_rate = float(cfg.realtime_rate)
        self._paused = False
        self._pending_step = False
        self._release_pending = False
        self._wall_anchor = None
        self._sched_steps = 0
        self._last_rate = self._realtime_rate
        self._throw_i = 0

        # Wrap plant.step ONCE so run()'s internal steps hit the tick boundary
        # (viewer sync + pacing + frame capture) without editing run()'s loop.
        self._orig_step = self.plant.step
        self.plant.step = self._wrapped_step

    # ---- viewer plumbing ---------------------------------------------------
    def _wrapped_step(self, *args, **kwargs):
        self._orig_step(*args, **kwargs)
        self._tick_boundary()

    def _tick_boundary(self) -> None:
        """After every control tick: capture a frame (if recording), keep the viewer
        responsive, honour pause / single-step, and pace to ``realtime_rate``. No-op
        when headless, not recording, and free-running. Copied from
        sim/juggle_online.py::OnlineJuggleRunner._tick_boundary."""
        if self._recorder is not None:
            self._recorder.capture()
        v = self.viewer
        if v is None and self._realtime_rate <= 0.0:
            return
        if v is not None and not v.is_running():
            self.viewer = None
            return
        while v is not None and self._paused and not self._pending_step:
            v.sync()
            time.sleep(0.01)
            self._wall_anchor = None
            if not v.is_running():
                self.viewer = None
                return
        stepped_one_frame = self._pending_step
        self._pending_step = False
        if v is not None:
            v.sync()
        if stepped_one_frame:
            return
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

    # ---- key handling ------------------------------------------------------
    def key_callback(self, keycode: int) -> None:
        p, v = self.launch_pos_mm, self.vel_mms
        if keycode == _KEY_I:
            p[0] += POS_STEP_MM; self._print_state()
        elif keycode == _KEY_K:
            p[0] -= POS_STEP_MM; self._print_state()
        elif keycode == _KEY_L:
            p[1] += POS_STEP_MM; self._print_state()
        elif keycode == _KEY_J:
            p[1] -= POS_STEP_MM; self._print_state()
        elif keycode == _KEY_U:
            p[2] += POS_STEP_MM; self._print_state()
        elif keycode == _KEY_O:
            p[2] -= POS_STEP_MM; self._print_state()
        elif keycode == _KEY_F:
            v[0] += VEL_STEP_MMS; self._print_state()
        elif keycode == _KEY_H:
            v[0] -= VEL_STEP_MMS; self._print_state()
        elif keycode == _KEY_T:
            v[1] += VEL_STEP_MMS; self._print_state()
        elif keycode == _KEY_G:
            v[1] -= VEL_STEP_MMS; self._print_state()
        elif keycode == _KEY_V:
            v[2] += VEL_STEP_MMS; self._print_state()
        elif keycode == _KEY_B:
            v[2] -= VEL_STEP_MMS; self._print_state()
        elif keycode == _KEY_R:
            self._release_pending = True          # the idle loop runs the throw
        elif keycode == _KEY_P:
            self.launch_pos_mm = self._preset_pos.copy()
            self.vel_mms = self._preset_vel.copy()
            print("[bb_catch] reset launch to preset")
            self._print_state()
        elif keycode == _KEY_SPACE:
            self._paused = not self._paused
            print(f"[bb_catch] {'paused' if self._paused else 'running'}")
        elif keycode == _KEY_RIGHT_ARROW:
            if self._paused:
                self._pending_step = True
        elif keycode in (_KEY_LEFT_BRACKET, _KEY_RIGHT_BRACKET):
            r = self._realtime_rate if self._realtime_rate > 0.0 else 1.0
            r = (max(r * 0.5, 0.0625) if keycode == _KEY_LEFT_BRACKET
                 else min(r * 2.0, 16.0))
            self._realtime_rate = r
            print(f"[bb_catch] realtime rate = {r:.3f}x")
        elif keycode == _KEY_C:
            h = self.viewer
            if h is not None:
                c = h.cam
                print("[bb_catch] camera: "
                      f"--cam-azimuth {c.azimuth:.1f} --cam-elevation {c.elevation:.1f} "
                      f"--cam-distance {c.distance:.3f} "
                      f"--cam-lookat {c.lookat[0]:.3f} {c.lookat[1]:.3f} {c.lookat[2]:.3f}")
        elif keycode == _KEY_LEFT_ARROW:
            pass

    # ---- throw + catch -----------------------------------------------------
    def _spawn_from_release(self, target):
        """Full-flight spawn: release the (perturbed) ball at the launch point so the
        operator sees the whole arc. Assigned to ``runner._spawn_incoming`` (a bound
        method of THIS tool) so ``run()``'s ``self._spawn_incoming(target)`` call
        reaches it. Mirrors juggle_catch._spawn_incoming's perturb path (identical
        §3 noise) but skips the apex conversion."""
        r = self.runner
        pos_mm = np.asarray(r.cfg.manual_launch_mm[0], float).reshape(3)
        vel_mms = np.asarray(r.cfg.manual_launch_mm[1], float).reshape(3)
        disp_mm = target * 1000.0 - pos_mm
        pos_n_mm, vel_n_mms = r.noise.perturb_throw(pos_mm, vel_mms, disp_mm)
        r.plant.spawn_ball(pos_n_mm, vel_n_mms, ball=0)

    def do_throw(self):
        """Run ONE manual throw + catch and return the :class:`CatchResult`."""
        r = self.runner
        r.cfg.manual_launch_mm = (self.launch_pos_mm.copy(), self.vel_mms.copy())
        r.cfg.flight_s = _flight_window_s(self.launch_pos_mm, self.vel_mms,
                                          self.cfg.catch_z_m, self.cfg.full_flight)
        # The runner is reused across throws: run() resets the PLANT but not the
        # estimator/noise, so reset them here for a correct, deterministic throw.
        r.est.reset()
        r.noise = JuggleNoise(r.cfg.noise, seed=self.cfg.seed + self._throw_i)
        if self.cfg.full_flight:
            r._spawn_incoming = self._spawn_from_release
        res = r.run()
        self._throw_i += 1
        return res

    def _warm_to_ready(self) -> None:
        """Hold the platform at the ready pose (level cup, catch_z + ready-lift) so
        the initial view shows Jugglebot poised before the first throw."""
        c = self.runner.cfg
        self.plant.reset()
        ready_z = c.catch_z_m + c.catch_ready_lift_m
        pose0, slider0 = realize_tilted(np.array([0.0, 0.0]), ready_z, 0.0, 0.0)
        for _ in range(c.settle_ticks):
            self.plant.command(self.plant.pose_to_extensions(pose0))
            self.plant.command_hand(slider0)
            self.plant.step(CONTROL_DT)

    # ---- release-state preview (viewer only) -------------------------------
    def _update_preview(self) -> None:
        """Draw the current release state in the viewer's user scene: a translucent
        ball at the launch point + an arrow along the initial velocity. Called each
        idle frame so it updates LIVE as the operator nudges the launch position /
        velocity, showing what the next R will throw."""
        v = self.viewer
        if v is None:
            return
        scn = v.user_scn
        p0 = (self.launch_pos_mm / 1000.0).astype(float)
        tip = (p0 + (self.vel_mms / 1000.0) * ARROW_SCALE_S).astype(float)
        eye = np.eye(3).flatten()
        mujoco.mjv_initGeom(
            scn.geoms[0], mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([BALL_R_M, 0.0, 0.0]), p0, eye,
            np.array([0.95, 0.35, 0.35, 0.45], np.float32))          # translucent ball
        mujoco.mjv_initGeom(
            scn.geoms[1], mujoco.mjtGeom.mjGEOM_ARROW,
            np.zeros(3), np.zeros(3), eye,
            np.array([0.25, 0.6, 1.0, 0.9], np.float32))             # velocity arrow
        mujoco.mjv_connector(
            scn.geoms[1], mujoco.mjtGeom.mjGEOM_ARROW, ARROW_WIDTH_M, p0, tip)
        scn.ngeom = 2

    def _clear_preview(self) -> None:
        if self.viewer is not None:
            self.viewer.user_scn.ngeom = 0

    # ---- reporting ---------------------------------------------------------
    def _print_state(self) -> None:
        p, v = self.launch_pos_mm, self.vel_mms
        print(f"[bb_catch] launch_pos_mm=({p[0]:.0f},{p[1]:.0f},{p[2]:.0f})  "
              f"vel_mms=({v[0]:.0f},{v[1]:.0f},{v[2]:.0f})  |v|={np.linalg.norm(v):.0f}  "
              f"mode={'full-flight' if self.cfg.full_flight else 'apex'}")

    def _print_result(self, res) -> None:
        p, v = self.launch_pos_mm, self.vel_mms
        print(f"[bb_catch] throw {self._throw_i}: "
              f"launch=({p[0]:.0f},{p[1]:.0f},{p[2]:.0f})mm "
              f"vel=({v[0]:.0f},{v[1]:.0f},{v[2]:.0f})mm/s")
        lx, ly = res.landing_xy_m
        print(f"[bb_catch]   {'CLEAN CATCH' if res.clean else 'MISS'}: "
              f"caught={res.caught} held={res.held_at_end} "
              f"in_cup_off={res.in_cup_offset_mm:.1f}mm reach={res.reach_mm:.1f}mm "
              f"landing_xy=({lx * 1000:.0f},{ly * 1000:.0f})mm "
              f"tilt={res.catch_tilt_deg:.1f}deg")

    @staticmethod
    def result_dict(res) -> dict:
        lx, ly = res.landing_xy_m
        return dict(
            caught=bool(res.caught),
            held=bool(res.held_at_end),
            in_off_mm=float(res.in_cup_offset_mm),
            reach_mm=float(res.reach_mm),
            landing_xy_mm=(float(lx * 1000.0), float(ly * 1000.0)),
        )


def _print_help() -> None:
    print("[bb_catch] keys — POSITION (launch pt, +-20 mm):  i/k = +x/-x   l/j = +y/-y   u/o = +z/-z")
    print("[bb_catch]        VELOCITY (+-75 mm/s):            f/h = +vx/-vx  t/g = +vy/-vy  v/b = +vz/-vz")
    print("[bb_catch]        R = release (throw + catch)      P = reset launch to preset")
    print("[bb_catch]        SPACE pause   RIGHT step (paused)   [ slower / ] faster   C print-camera   (mouse = camera)")


def run(cfg: "BBCatchConfig | None" = None):
    """Run the BB-catch tool. Interactive viewer when ``cfg.headless`` is False
    (press R to throw, adjust between throws); otherwise runs ``cfg.throws`` scripted
    throws headless. An offscreen mp4 recorder is attached when ``cfg.record_path``
    is set (captures throw frames). Returns a list of per-throw result dicts."""
    cfg = cfg or BBCatchConfig()
    tool = BBCatchTool(cfg)
    viewer_handle = None
    results: list = []
    try:
        if not cfg.headless:
            import mujoco.viewer  # type: ignore
            viewer_handle = mujoco.viewer.launch_passive(
                tool.plant.model, tool.plant.data, key_callback=tool.key_callback)
            tool.viewer = viewer_handle
            # Start at a good three-quarter view (CLI --cam-* overrides each field).
            cam = viewer_handle.cam
            cam.azimuth = cfg.cam_azimuth if cfg.cam_azimuth is not None else DEFAULT_VIEW_AZIMUTH
            cam.elevation = cfg.cam_elevation if cfg.cam_elevation is not None else DEFAULT_VIEW_ELEVATION
            cam.distance = cfg.cam_distance if cfg.cam_distance is not None else DEFAULT_VIEW_DISTANCE
            cam.lookat[:] = cfg.cam_lookat if cfg.cam_lookat is not None else DEFAULT_VIEW_LOOKAT
            viewer_handle.sync()
        if cfg.record_path:
            tool._recorder = VideoRecorder(
                tool.plant.model, tool.plant.data, cfg.record_path,
                width=cfg.record_size[0], height=cfg.record_size[1],
                fps=cfg.record_fps, cam=build_record_camera(tool.plant.model, cfg))
            print(f"[bb_catch] recording to {cfg.record_path}")

        if cfg.headless:
            for _ in range(max(cfg.throws, 1)):
                res = tool.do_throw()
                tool._print_result(res)
                results.append(BBCatchTool.result_dict(res))
            return results

        # ---- interactive session ----
        _print_help()
        tool._print_state()
        print("[bb_catch] Jugglebot at ready — press R to throw.")
        tool._warm_to_ready()
        while viewer_handle.is_running():
            if tool._release_pending:
                tool._release_pending = False
                tool._clear_preview()               # hide the preview during the flight
                res = tool.do_throw()
                results.append(BBCatchTool.result_dict(res))
                tool._print_result(res)
                if viewer_handle.is_running():
                    print("[bb_catch] holding result — adjust and press R for the next throw.")
            else:
                tool._update_preview()              # live release-state ball + velocity arrow
                viewer_handle.sync()
                time.sleep(0.02)
        return results
    finally:
        if tool._recorder is not None:
            n = tool._recorder.frames
            tool._recorder.close()
            print(f"[bb_catch] video written: {cfg.record_path} ({n} frames)")
        if viewer_handle is not None:
            viewer_handle.close()


def main(argv: "list[str] | None" = None) -> int:
    p = argparse.ArgumentParser(
        description="Interactive Ball-Butler -> Jugglebot single-catch tool. "
                    "Set the launch position + velocity with keys, press R to throw, "
                    "watch Jugglebot catch it. Headless (--throws N) for scripted runs.")
    p.add_argument('--viewer', action='store_true',
                   help="Launch the MuJoCo passive viewer (interactive; mouse = camera)")
    p.add_argument('--throws', type=int, default=1,
                   help="Headless: number of scripted throws to run (default 1). "
                        "Throw i uses seed (--seed + i) for deterministic distinct throws.")
    p.add_argument('--seed', type=int, default=0, help="Base §3-noise seed (default 0)")
    p.add_argument('--catch-z-mm', type=float, default=None,
                   help="Cup catch height (mm, cup-world). Default 840.")
    p.add_argument('--full-flight', action='store_true',
                   help="Spawn from the release point (see the whole arc) instead of "
                        "the apex. Reliable for the far/high demo BB; near/low launch "
                        "points may clip the cup on the rising arc — default is apex.")
    p.add_argument('--realtime-rate', type=float, default=None,
                   help="Wall-clock pacing: 1.0 = real-time, 0.5 = slowmo, 2.0 = 2x, "
                        "0 = free-run. Default: 1.0 with --viewer, else 0.")
    # ---- scripted-throw overrides ----
    p.add_argument('--vx', type=float, default=None, help="Override launch vx (mm/s)")
    p.add_argument('--vy', type=float, default=None, help="Override launch vy (mm/s)")
    p.add_argument('--vz', type=float, default=None, help="Override launch vz (mm/s)")
    p.add_argument('--landing-x-mm', type=float, default=None,
                   help="Aim the preset at this landing x (mm) instead of 0")
    p.add_argument('--landing-y-mm', type=float, default=None,
                   help="Aim the preset at this landing y (mm) instead of 0")
    # ---- recording (mirrors sim/juggle_online.py) ----
    p.add_argument('--record', default=None, metavar='PATH',
                   help="Render offscreen from a fixed camera and write an H.264 mp4 "
                        "(needs ffmpeg on PATH). Captures throw frames.")
    p.add_argument('--record-size', default='1280x720', metavar='WxH',
                   help="Recording resolution (default 1280x720)")
    p.add_argument('--record-fps', type=int, default=40,
                   help="Recording output fps (default 40 = real-time)")
    p.add_argument('--cam-azimuth', type=float, default=None, help="Fixed-camera azimuth (deg)")
    p.add_argument('--cam-elevation', type=float, default=None, help="Fixed-camera elevation (deg)")
    p.add_argument('--cam-distance', type=float, default=None, help="Fixed-camera distance (m)")
    p.add_argument('--cam-lookat', type=float, nargs=3, default=None, metavar=('X', 'Y', 'Z'),
                   help="Fixed-camera look-at (m). Press C in --viewer to print the current angle.")
    args = p.parse_args(argv)

    rate = args.realtime_rate
    if rate is None:
        rate = 1.0 if args.viewer else 0.0
    try:
        _rw, _rh = (int(v) for v in args.record_size.lower().split('x'))
    except ValueError:
        p.error(f"--record-size must be WxH, got {args.record_size!r}")

    target_xy = (args.landing_x_mm or 0.0, args.landing_y_mm or 0.0)
    kw: dict = dict(
        seed=args.seed, headless=not args.viewer, throws=args.throws,
        realtime_rate=rate, full_flight=args.full_flight, target_xy_mm=target_xy,
        vx_mms=args.vx, vy_mms=args.vy, vz_mms=args.vz,
        record_path=args.record, record_size=(_rw, _rh), record_fps=args.record_fps,
        cam_azimuth=args.cam_azimuth, cam_elevation=args.cam_elevation,
        cam_distance=args.cam_distance,
        cam_lookat=tuple(args.cam_lookat) if args.cam_lookat is not None else None,
    )
    if args.catch_z_mm is not None:
        kw['catch_z_m'] = args.catch_z_mm / 1000.0
    cfg = BBCatchConfig(**kw)

    t_start = time.time()
    results = run(cfg)
    wall = time.time() - t_start
    n_clean = sum(1 for r in results if r['caught'] and r['held']
                  and r['in_off_mm'] <= 40.0)
    print(f"[bb_catch] {len(results)} throw(s): {n_clean} clean  (wall {wall:.1f}s)")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
