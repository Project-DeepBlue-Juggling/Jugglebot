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

RE-PLAN (2026-07-01, ``oscillate=True``): the HONEST tilt test, STILL A BREAK.
--------------------------------------------------------------------------------
Per the operator's OPTION 1 re-plan, the ``oscillate=True`` mode shuttles a single
ball between two lateral points A and B (throw A->B, catch at B, throw B->A, ...)
so every throw is LATERAL and the commanded tilt is NON-ZERO (~1.4 deg for the
default +-20 mm on x) — i.e. tilt actually ENGAGES and does the lateral-aim work,
unlike the degenerate column. This is the faithful make-or-break test of the tilt
hypothesis on a NON-degenerate geometry.

**RESULT (2026-07-01): BREAK — the oscillation does NOT sustain either.** With
tilt engaged it diverges within 1-4 cycles at every separation (20-70 mm) and axis
(x / y / diagonal): the loop gain is > 1 (max sustained 4 of >= 10, at x-20). The
load-bearing distinction from the column: the in-cup SEAT offset stays SMALL
(~0.5-2 mm — tilt DOES keep the ball centred), but the LANDING amplifies (e.g.
x-40 seed 1: landing error
3.7 -> 89 -> 728 mm). Root cause: the tilt-aimed throw's landing is CHAOTICALLY and
DETERMINISTICALLY sensitive to the throw-ORIGIN pose — a 10 mm shift of the throw
point swings the landing ~40 mm (dLanding/dOrigin ~4, and up to ~11 with sign
changes at a real 1.8 deg tilt). This is the contact-detach knife-edge Rung 2a
flagged (the non-y-symmetric leg layout + soft->stiff release), which Rung 2a only
characterised from the ORIGIN and deferred; the oscillation throws from OFF-origin
points A/B, where it is loop-fatal. **Tilt fixes the band-limit (the pre-tilt
divergence's mechanism) but NOT this pose-chaos, which is the binding amplification
off-origin — so the tilt re-architecture, by itself, does not close the loop.**
See ``logbook/2026-07-01-rung2b-oscillation-tilt-engaged-diverges.md``.

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
from sim.juggle_catch import CATCH_ARREST_ACCEL_G, _ContinuousQuintic, _quintic_step
from sim.juggle_noise import BallisticEstimator, JuggleNoise, NoiseConfig
from sim.juggle_tilt import (
    cup_axis, realize_tilted, tilt_to_receive, tilt_to_throw, MAX_TILT_DEG,
)
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
# The momentum-budget arrest deceleration for the continuous catch lives in
# sim.juggle_catch (CATCH_ARREST_ACCEL_G), shared with the Rung-1 SingleCatch.


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
    # ---- oscillation geometry (Rung-2b RE-PLAN — the HONEST tilt test) ----------
    # The pure column above is a DEGENERATE test of the tilt hypothesis: a column
    # commands ~0 tilt, so the tilt mechanism (decoupling lateral aim from the
    # band-limited platform) never ENGAGES (logbook
    # 2026-07-01-rung2b-selfcatch-column-divergence.md). The oscillation instead
    # shuttles a single ball between two lateral points A and B: throw A->B, catch
    # at B, throw B->A, catch at A, ... Each throw is LATERAL (target != origin), so
    # the commanded tilt is NON-ZERO and tilt actually does the lateral-aim work.
    # The A<->B axis + separation are chosen inside the Rung-2a reliable box (the
    # column + 50 mm ring separate cleanly, <=33 mm; the +-100 mm asymmetry is
    # deferred to Rung 3), so this tests TILT, not the deferred asymmetry.
    oscillate: bool = False            # True: two-point A<->B oscillation (tilt on)
    # Default A<->B: +-20 mm on x (40 mm separation, tilt ~1.4 deg). Chosen so the
    # FIRST throw composes cleanly (lands ~3.7 mm from B, deep inside the catch
    # reach) yet tilt is clearly engaged; the loop then diverges by cyc 2-3 (the
    # BREAK). Across the full 20-70 mm sweep no separation sustains (max 4 of >=10);
    # the cyc-0 landing error is non-monotonic in separation (see the logbook).
    osc_point_a_m: "tuple[float, float]" = (-0.020, 0.0)  # cup xy of point A (world)
    osc_point_b_m: "tuple[float, float]" = (0.020, 0.0)   # cup xy of point B (world)
    recover: str = "plan"              # "plan": Rung-2a plan_cup_cycle recover
                                       # (validated, faithful). "retract": an axial
                                       # slider retract (explored; a knife-edge).
    # ---- release (throw) mode (Rung-2b re-architecture) ------------------------
    # "detach": the shipped contact-carry emergent throw (set stiff contact ->
    #   begin_physics_throw; the take-off velocity EMERGES from the hand stroke).
    #   This is the CONTACT-DETACH KNIFE-EDGE Rung 2a/2b characterised: a 10 mm
    #   throw-origin shift swings the landing ~40 mm (dLanding/dOrigin ~2.7), so
    #   the throw->catch->throw loop gain is > 1 and the loop DIVERGES (the BREAK).
    # "kinematic": the Rung-2b fix — cut the ball free at the cup's planned
    #   take-off velocity (Ball.ballistic_release), breaking hand contact so no
    #   residual push re-corrupts the launch. This is the CLEAN-SEPARATION limit
    #   (a ball leaving with no push departs at the cup velocity — Newton); it
    #   drops the open-loop knife-edge gain 2.7 -> 0.01, so the loop gain drops
    #   below 1. Pairs with the co-moving robust catch (see _catch). The "detach"
    #   default keeps the tilt=0 / Rung-1 / Rung-2a paths byte-identical.
    release: str = "detach"            # "detach" (shipped) | "kinematic" (Rung-2b fix)
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
    # ---- viewer / offscreen recording (mirror sim/juggle_online.py; PURELY
    # additive — the headless run_self_catch() never reads these, so it stays
    # byte-identical. Only the module-level run()/main() below use them.) --------
    headless: bool = True              # if False, launch the MuJoCo passive viewer
    realtime_rate: float = 0.0         # wall-clock pacing: 0 = free-run, 1.0 = real
    record_path: "str | None" = None
    record_size: "tuple[int, int]" = (1280, 720)   # (width, height) px
    record_fps: int = 40               # output fps; 40 = real-time (1 frame/tick)
    cam_azimuth: "float | None" = None
    cam_elevation: "float | None" = None
    cam_distance: "float | None" = None
    cam_lookat: "tuple[float, float, float] | None" = None


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
    # ---- oscillation-mode metrics (nan for the column path) ----
    target_xy_mm: "tuple[float, float]" = (float('nan'), float('nan'))  # nominal target
    landing_err_mm: float = float('nan')   # |landing - target| — the throw accuracy
    tilt_deg: float = 0.0                  # commanded throw tilt magnitude (0 = column)


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
    def in_off_trend_mm(self) -> "list[float]":
        """Per-cycle in-cup seat offset after the catch — the primary loop-gain
        trend (flat/decaying = loop gain < 1, growing = diverging)."""
        return [round(c.in_off_end_mm, 1) for c in self.cycles]

    @property
    def landing_err_trend_mm(self) -> "list[float]":
        """Per-cycle |landing - nominal target| — the throw accuracy trend (the
        oscillation loop-gain signature; nan for the column path)."""
        return [round(c.landing_err_mm, 1) for c in self.cycles]

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
        # ---- viewer / wall-clock pacing. Only the module-level run() attaches a
        # viewer/recorder and wraps plant.step to call _tick_boundary; the headless
        # run() never does, so run_self_catch() is byte-identical. ----
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
            print(f"[juggle_selfcatch] {'paused' if self._paused else 'running'} "
                  f"at sim_time {float(self.plant.data.time):.3f} s")
        elif keycode == _KEY_RIGHT_ARROW:
            if self._paused:
                self._pending_step = True
        elif keycode in (_KEY_LEFT_BRACKET, _KEY_RIGHT_BRACKET):
            r = self._realtime_rate if self._realtime_rate > 0.0 else 1.0
            r = max(r * 0.5, 0.0625) if keycode == _KEY_LEFT_BRACKET else min(r * 2.0, 16.0)
            self._realtime_rate = r
            print(f"[juggle_selfcatch] realtime rate = {r:.3f}x")
        elif keycode == _KEY_C:
            h = self.viewer
            if h is not None:
                c = h.cam
                print("[juggle_selfcatch] camera: "
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
        # The column co-locates throw+target at the origin (tilt~=0); the
        # oscillation throws from A toward B (a lateral throw -> tilt engages).
        if cfg.oscillate:
            _tp0 = np.array([cfg.osc_point_a_m[0], cfg.osc_point_a_m[1], cfg.throw_z_m])
            _tg0 = np.array([cfg.osc_point_b_m[0], cfg.osc_point_b_m[1], cfg.throw_target_z_m])
        else:
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

            if cfg.oscillate:
                # Two-point A<->B: cyc 0 throws A->B, cyc 1 throws B->A, ... — the
                # cup shuttles A<->B and every throw is LATERAL (tilt engages). The
                # throw ORIGIN is re-planned from the ACHIEVED caught xy (closed-loop
                # self-correction of the cup position error), except cyc 0 (freshly
                # seeded at A); the TARGET is the fixed nominal other point.
                pa = np.asarray(cfg.osc_point_a_m, float)
                pb = np.asarray(cfg.osc_point_b_m, float)
                to_xy = pb if (cyc % 2 == 0) else pa
                from_xy = pa if cyc == 0 else self.cup_world_m()[:2]
                throw_pos = np.array([from_xy[0], from_xy[1], cfg.throw_z_m])
                target = np.array([to_xy[0], to_xy[1], cfg.throw_target_z_m])
            else:
                # Column throw/catch xy: fixed origin (stationary) or caught xy (drift).
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
            target_mm = target[:2] * 1000.0
            cycles.append(SelfCatchCycle(
                cyc=cyc, in_off_start_mm=in_off_start, separated=separated,
                caught=caught, held_at_end=held, in_off_end_mm=in_off_end,
                reach_mm=reach_mm,
                reach_from_throw_mm=float(np.linalg.norm(land_mm - throw_xy_mm)),
                seat_offset_mm=seat_off,
                landing_xy_mm=(float(land_mm[0]), float(land_mm[1])),
                takeoff_speed_mps=float(np.linalg.norm(v_takeoff)),
                target_xy_mm=(float(target_mm[0]), float(target_mm[1])),
                landing_err_mm=float(np.linalg.norm(land_mm - target_mm)),
                tilt_deg=float(np.degrees(np.hypot(rx, ry)))))
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
        """Carry the seated ball up to the throw point, ramping the tilt in.

        * **detach** (shipped): play the ``plan_cup_cycle`` carry half. The cup
          arrives at the throw point MOVING at ``v_takeoff`` because the emergent
          contact throw imparts the cup's velocity to the ball.
        * **kinematic** (Rung-2b co-design): ``ballistic_release`` IMPOSES
          ``v_takeoff`` on the ball independent of the cup's velocity, so the
          carry need NOT build ``v_takeoff``. Lift the cup to the throw point
          ending at ~rest — this removes the ~120 mm post-release upward OVERSHOOT
          the ``v_takeoff`` carry incurred (and that the recover had to claw back),
          so the hand moves only as far as it needs to interact with the ball.
          Commanded continuously (sub-tick). The detach path stays byte-identical.
        """
        cfg = self.cfg
        if cfg.release == "kinematic":
            plant.set_contact_stiffness(False)
            cup_p = self.cup_world_m(); cup_v = self.cup_vel_world_m()
            seg = _ContinuousQuintic(cup_p, cup_v, np.zeros(3), throw_pos,
                                     np.zeros(3), np.zeros(3), carry_dur, CONTROL_DT)
            n_carry = int(round(carry_dur / CONTROL_DT))
            tc0 = float(plant.data.time)

            def carry_cmd(t_abs):
                cp = seg.pos(t_abs - tc0)
                s = _smoothstep((t_abs - tc0) / max(cfg.tilt_ramp_frac * carry_dur, 1e-6))
                return realize_tilted(cp[:2], float(cp[2]), rx * s, ry * s)

            for _ in range(n_carry):
                pose, _ = carry_cmd(float(plant.data.time))
                plant.command(plant.pose_to_extensions(pose))
                plant.step(CONTROL_DT, hand_cmd_fn=lambda t, f=carry_cmd: f(t)[1],
                           plat_cmd_fn=lambda t, f=carry_cmd:
                           plant.pose_to_extensions(f(t)[0]))
            return
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
        if cfg.release == "kinematic":
            # KINEMATIC (ballistic) release — the Rung-2b co-design recover. Cut
            # the ball free at the PLANNED take-off velocity, then run a MINIMAL
            # recover: the low-velocity-release carry leaves the cup ~at rest at
            # the throw point (no v_takeoff overshoot to claw back), so the cup
            # only eases up to the catch ready height while the ball climbs to its
            # apex. The ball is ballistic + contact-disabled after release, so its
            # landing is decoupled from the cup's recover motion (verified). See
            # logbook/2026-07-03-catch-control-formulation-design-basis.md.
            return self._release_recover_kinematic(
                plant, v_takeoff, rx, ry, cup_p, cup_v)
        # DETACH (shipped): the emergent contact-carry throw (knife-edge).
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

    def _release_recover_kinematic(self, plant, v_takeoff, rx, ry, cup_p, cup_v):
        """Minimal recover for the kinematic (co-design) throw.

        The low-velocity-release carry leaves the cup ~at rest at the throw point,
        so — unlike the ``v_takeoff`` carry, whose ~+2.7 m/s post-release rise the
        plan-recover had to arrest and fold into a ~250 mm down-and-up excursion —
        the cup only needs to ease continuously up to the catch ready height and
        level off while the ball climbs to its apex. ``ballistic_release`` cuts the
        ball free at ``v_takeoff`` with contact disabled, so the ball is ballistic
        and its landing is decoupled from this cup motion (the catch reads the
        achieved cup state and adapts). Observe the in-flight ball for the catch
        estimator; return ``(separated, true ballistic landing at catch_z)``.
        """
        cfg = self.cfg
        plant.set_contact_stiffness(False)
        plant.ballistic_release(v_takeoff * 1000.0, ball=0)
        ready_z = cfg.catch_z_m + cfg.catch_ready_lift_m
        target_p = np.array([cup_p[0], cup_p[1], ready_z])
        rec_dur = cfg.n_recovery_ticks * CONTROL_DT
        seg = _ContinuousQuintic(cup_p, cup_v, np.zeros(3), target_p, np.zeros(3),
                                 np.zeros(3), rec_dur, CONTROL_DT)
        t0 = float(plant.data.time)
        self.est.reset()

        def recover_cmd(t_abs):
            cp = seg.pos(t_abs - t0)
            # Ramp the tilt back to level as the cup settles (the throw tilt is no
            # longer needed once the ball is free; a level ready pose seats cleanly).
            s = 1.0 - _smoothstep((t_abs - t0) / max(rec_dur, 1e-6))
            return realize_tilted(cp[:2], float(cp[2]), rx * s, ry * s)

        for _ in range(cfg.n_recovery_ticks):
            pose, _ = recover_cmd(float(plant.data.time))
            plant.command(plant.pose_to_extensions(pose))
            plant.step(CONTROL_DT, hand_cmd_fn=lambda t, f=recover_cmd: f(t)[1],
                       plat_cmd_fn=lambda t, f=recover_cmd:
                       plant.pose_to_extensions(f(t)[0]))
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
        """Catch the in-flight ball: translate-to-reach + tilt-to-receive + a
        cushioned seat. Returns (caught, held_at_end, in_off_end_mm, reach_mm,
        seat_offset_mm, last_pos_td).

        Dispatches by ``cfg.release``:

        * **kinematic** (Rung-2b co-design): :meth:`_catch_continuous` — a
          CONTINUOUS velocity-matched descent commanded at the sub-tick rate, so
          the cup co-moves with the ball on a *descending* stroke (no ceiling
          overshoot; contact moving DOWN). See
          ``logbook/2026-07-03-catch-control-formulation-design-basis.md``.
        * **detach** (shipped, Rung-1): the fixed-schedule constant-decel seat
          below — a per-tick quintic to catch_z then a constant-decel-to-rest,
          each commanded as a CONSTANT per 40 Hz tick. BYTE-IDENTICAL to the
          original, so tilt=0 / Rung-1 / Rung-2a / the detach loop are unchanged.
        """
        cfg = self.cfg
        if cfg.release == "kinematic":
            return self._catch_continuous(plant)
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
            if frozen_xy is None:                       # freeze the reach on the first fit
                frozen_xy = pos_td[:2].copy()
            if descending and frozen_tilt is None:
                frozen_tilt = np.array([tilt_rx, tilt_ry])
            reach_xy = frozen_xy if frozen_xy is not None else pos_td[:2].copy()
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

    def _catch_continuous(self, plant):
        """Catch the in-flight ball with a CONTINUOUS velocity-matched descent —
        the Rung-2b co-design catch
        (``logbook/2026-07-03-catch-control-formulation-design-basis.md``).

        The shipped kinematic catch tried to co-move by commanding the cup a
        CONSTANT position per 40 Hz tick (targeting the ball's descent velocity),
        so the ultra-stiff hand actuator jump-and-settled and could only build
        downward runway by OVERSHOOTING to the stroke ceiling — the 93 ceiling
        ticks, the +9 mm/s UP contact, the ball seating ~145 mm above catch_z on a
        parked cup (the motion-quality review). Here the whole pose (xy, z, tilt)
        is commanded CONTINUOUSLY at the sub-tick rate (``hand_cmd_fn`` /
        ``plat_cmd_fn`` sample a per-tick :class:`_ContinuousQuintic` vs sim time),
        so the cup genuinely descends at the ball's velocity — it receives the ball
        on a DESCENDING cup, no ceiling overshoot. The seat confirms during the
        co-moving descent; a momentum-budget arrest (:data:`CATCH_ARREST_ACCEL_G`,
        d = v²/(2·a)) then decelerates the co-moving cup to rest well off both
        slider clamps. Once seated, hold a LEVEL pose so the ball rides stably.
        """
        cfg = self.cfg
        plant.set_contact_stiffness(False)
        catch_z = cfg.catch_z_m
        ready_z = catch_z + cfg.catch_ready_lift_m
        a_budget = CATCH_ARREST_ACCEL_G * (-GRAVITY[2])
        cup_p = self.cup_world_m()
        cmd_xy = cup_p[:2].copy(); cmd_vxy = np.zeros(2); cmd_axy = np.zeros(2)
        cmd_tilt = np.zeros(2); cmd_vtilt = np.zeros(2); cmd_atilt = np.zeros(2)
        cmd_z = float(cup_p[2]); cmd_vz = float(self.cup_vel_world_m()[2]); cmd_az = 0.0
        frozen_xy = None; frozen_tilt = None; settling = False
        caught = False; seat_offset = None; level_pose = None
        last_pos_td = np.array([0.0, 0.0, catch_z])
        n_catch = int(round((cfg.flight_s + 0.40) / CONTROL_DT))

        for _k in range(n_catch):
            t_abs = float(plant.data.time)
            bs = plant.get_ball_state(ball=0)
            true_m = np.asarray(bs.position_mm, float) / 1000.0
            obs_m = self.noise.observe(true_m * 1000.0) / 1000.0
            self.est.add(t_abs, obs_m)

            # Once seated, hold a LEVEL pose (the cup is at rest, so a constant
            # command is smooth) so the ball rides stably in the levelled cup.
            if level_pose is not None:
                pose, slider = level_pose
                plant.command(plant.pose_to_extensions(pose))
                plant.step(CONTROL_DT, hand_cmd_fn=lambda _t, s=slider: s,
                           plat_cmd_fn=lambda _t, p=pose: plant.pose_to_extensions(p))
                continue

            pos_e, vel_e = self.est.estimate()
            t_td, pos_td, vel_td = ballistic_touchdown(pos_e, vel_e, catch_z)
            t_remaining = t_td
            pos_td = np.array(pos_td, float)
            pos_td[:2] = np.clip(pos_td[:2], -WORKSPACE_XY_M, WORKSPACE_XY_M)
            tilt_rx, tilt_ry = tilt_to_receive(vel_td, MAX_TILT_DEG)
            v_arr = abs(float(vel_td[2]))          # estimated |arrival vz| at catch_z

            # Warm-up: hold (cup at rest) until the arc is well-observed.
            if self.est.n < 4:
                pose, slider = realize_tilted(cmd_xy, cmd_z, cmd_tilt[0], cmd_tilt[1])
                plant.command(plant.pose_to_extensions(pose))
                plant.step(CONTROL_DT, hand_cmd_fn=lambda _t, s=slider: s,
                           plat_cmd_fn=lambda _t, p=pose: plant.pose_to_extensions(p))
                continue

            descending = t_remaining <= cfg.catch_start_lead_s
            # Freeze the reach LATE (when descending): the clean throw's
            # early-flight fit mispredicts the landing, the near-touchdown fit is
            # accurate (the kinematic-release lesson).
            if frozen_xy is None and descending:
                frozen_xy = pos_td[:2].copy()
            if descending and frozen_tilt is None:
                frozen_tilt = np.array([tilt_rx, tilt_ry])
            reach_xy = frozen_xy if frozen_xy is not None else pos_td[:2].copy()
            reach_tilt = (frozen_tilt if frozen_tilt is not None
                          else np.array([tilt_rx, tilt_ry]))
            last_pos_td = np.array([reach_xy[0], reach_xy[1], catch_z])

            # --- per-tick continuous quintic segments (xy / tilt / z) ---
            seg_xy = _ContinuousQuintic(cmd_xy, cmd_vxy, cmd_axy, reach_xy,
                                        np.zeros(2), np.zeros(2),
                                        cfg.reach_settle_floor_s, CONTROL_DT)
            seg_tilt = _ContinuousQuintic(
                cmd_tilt, cmd_vtilt, cmd_atilt, reach_tilt, np.zeros(2), np.zeros(2),
                max(t_remaining, cfg.reach_settle_floor_s), CONTROL_DT)
            if not settling and true_m[2] > catch_z + 0.002 and t_remaining > 1e-3:
                # APPROACH: from the ready height, descend to arrive at catch_z
                # co-moving with the ball (down at ratio·|arrival vz|) at t_td.
                if not descending:
                    z_t, vz_t = ready_z, 0.0
                    hz = max(t_remaining - cfg.catch_start_lead_s, 3.0 * CONTROL_DT)
                else:
                    z_t = catch_z
                    vz_t = -cfg.catch_slider_vel_ratio * v_arr
                    hz = max(t_remaining, 3.0 * CONTROL_DT)
                seg_z = _ContinuousQuintic([cmd_z], [cmd_vz], [cmd_az], [z_t], [vz_t],
                                           [0.0], hz, CONTROL_DT)
            else:
                # ARREST: momentum-budget MIN-JERK decel of the co-moving cup to
                # rest over d = v²/(2·a) at CATCH_ARREST_ACCEL_G — the quintic is
                # re-solved each tick, so only its low-jerk ONSET executes (no
                # mid-stroke deceleration peak to eject the ball; gentle onset).
                # Latched (``settling``) so an estimate wobble can't re-enter the
                # approach.
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

            t0 = t_abs

            def cmd_fn(t, sx=seg_xy, sz=seg_z, st=seg_tilt, t0=t0):
                tau = t - t0
                xy = sx.pos(tau); z = float(sz.pos(tau)[0]); ti = st.pos(tau)
                return realize_tilted(xy, z, float(ti[0]), float(ti[1]))

            pose_now, _ = cmd_fn(t0)
            plant.command(plant.pose_to_extensions(pose_now))
            plant.step(CONTROL_DT, hand_cmd_fn=lambda t, f=cmd_fn: f(t)[1],
                       plat_cmd_fn=lambda t, f=cmd_fn: plant.pose_to_extensions(f(t)[0]))

            cmd_xy, cmd_vxy, cmd_axy = seg_xy.state()
            (zc, vc, ac) = seg_z.state()
            cmd_z, cmd_vz, cmd_az = float(zc[0]), float(vc[0]), float(ac[0])
            cmd_tilt, cmd_vtilt, cmd_atilt = seg_tilt.state()

            if not caught and plant.check_and_capture(ball=0):
                caught = True
                off, _ = self._ball_offset_mm()
                seat_offset = float(np.linalg.norm(off))
                level_pose = realize_tilted(cmd_xy, cmd_z, 0.0, 0.0)

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


def run(cfg: "SelfCatchConfig | None" = None) -> SelfCatchResult:
    """Run the self-catch loop WITH the MuJoCo passive viewer (``cfg.headless``
    False) and/or an offscreen mp4 recorder (``cfg.record_path`` set); otherwise
    integrates headless at full speed. Returns the :class:`SelfCatchResult`.

    The gated ``SelfCatchRunner.run()`` loop (and its phase helpers) is left
    untouched: instead of editing it, this wrapper wraps ``runner.plant.step`` so
    every physics tick drives a viewer/record "tick boundary" (sync + pace + frame
    capture). Because the wrapper is only installed here, the headless
    ``run_self_catch()`` path never sees it and stays byte-identical. The loop runs
    many cycles, so it is naturally watchable — no landing tail is needed."""
    cfg = cfg or SelfCatchConfig()
    runner = SelfCatchRunner(cfg)
    viewer_handle = None
    try:
        if not cfg.headless:
            import mujoco.viewer  # type: ignore
            viewer_handle = mujoco.viewer.launch_passive(
                runner.plant.model, runner.plant.data,
                key_callback=runner.key_callback)
            runner.viewer = viewer_handle
            print("[juggle_selfcatch] keyboard: SPACE pause/resume   RIGHT step "
                  "(when paused)   [ slower / ] faster   C print-camera   "
                  "(mouse-drag = camera)")
        if cfg.record_path:
            runner._recorder = VideoRecorder(
                runner.plant.model, runner.plant.data, cfg.record_path,
                width=cfg.record_size[0], height=cfg.record_size[1],
                fps=cfg.record_fps, cam=build_record_camera(runner.plant.model, cfg))
            print(f"[juggle_selfcatch] recording to {cfg.record_path}")

        # Wrap plant.step so each sim step drives a viewer/record tick boundary,
        # WITHOUT editing the gated run() loop. Headless never installs this.
        _orig_step = runner.plant.step

        def _stepped(*args, **kwargs):
            _orig_step(*args, **kwargs)
            runner._tick_boundary()

        runner.plant.step = _stepped
        return runner.run()
    finally:
        if runner._recorder is not None:
            n = runner._recorder.frames
            runner._recorder.close()
            print(f"[juggle_selfcatch] video written: {cfg.record_path} ({n} frames)")
        if viewer_handle is not None:
            viewer_handle.close()


def main(argv: "list[str] | None" = None) -> int:
    """CLI to watch/record the single-ball self-catch loop (mirrors the
    viewer/record flags of sim/juggle_online.py, plus the column / oscillation
    geometry knobs)."""
    p = argparse.ArgumentParser(
        description="Run the single-ball tilt-aimed self-catch loop in MuJoCo "
                    "(Rung 2b).")
    p.add_argument('--cycles', type=int, default=12,
                   help="Number of throw->catch cycles to attempt (default 12)")
    p.add_argument('--viewer', action='store_true',
                   help="Launch the MuJoCo passive viewer (mouse = camera)")
    p.add_argument('--realtime-rate', type=float, default=None,
                   help="Wall-clock pacing: 1.0 = real-time, 0.5 = half-speed "
                        "slowmo, 2.0 = 2x, 0 = free-running (fastest). Default: "
                        "1.0 with --viewer, else 0.")
    p.add_argument('--seed', type=int, default=0,
                   help="§3 tracking-noise seed (the only RNG in the loop)")
    # ---- geometry: column (default) vs two-point oscillation ----
    p.add_argument('--oscillate', action='store_true',
                   help="Two-point A<->B oscillation (every throw lateral, tilt "
                        "engaged) instead of the degenerate stationary column")
    p.add_argument('--separation-mm', type=float, default=40.0,
                   help="Oscillation A<->B separation along x (mm): A=(-s/2,0), "
                        "B=(+s/2,0). Default 40 (A=-20, B=+20). --oscillate only.")
    g = p.add_mutually_exclusive_group()
    g.add_argument('--stationary', dest='stationary', action='store_true',
                   help="Column mode: reposition to a FIXED origin each cycle "
                        "(default)")
    g.add_argument('--drift', dest='stationary', action='store_false',
                   help="Column mode: throw in-place from the caught xy (the cup "
                        "drifts; isolates the reposition)")
    p.set_defaults(stationary=None)
    # ---- release (throw) mode ----
    p.add_argument('--release', choices=('detach', 'kinematic'), default='detach',
                   help="Throw mode: 'detach' (shipped emergent contact throw, the "
                        "knife-edge -> BREAK) or 'kinematic' (Rung-2b fix: cut the "
                        "ball free at the planned take-off velocity -> loop gain < 1, "
                        "sustains). 'kinematic' also defaults the carry dip to 0.10 m "
                        "(the shorter carry that avoids the between-cycle slosh).")
    p.add_argument('--dip-m', type=float, default=None,
                   help="Carry dip depth (m) below the throw point. Default 0.16 for "
                        "'detach', 0.10 for 'kinematic' (override here).")
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

    # 'kinematic' uses the shorter carry dip (0.10) that avoids the between-cycle
    # slosh; --dip-m overrides. 'detach' keeps the shipped 0.16 default.
    dip_m = args.dip_m if args.dip_m is not None else (
        0.10 if args.release == 'kinematic' else 0.16)
    kw: dict = dict(
        n_cycles=args.cycles, seed=args.seed, oscillate=args.oscillate,
        release=args.release, dip_m=dip_m,
        headless=not args.viewer, realtime_rate=rate,
        record_path=args.record, record_size=(_rw, _rh), record_fps=args.record_fps,
        cam_azimuth=args.cam_azimuth, cam_elevation=args.cam_elevation,
        cam_distance=args.cam_distance,
        cam_lookat=tuple(args.cam_lookat) if args.cam_lookat is not None else None,
    )
    if args.stationary is not None:
        kw['stationary'] = args.stationary
    if args.oscillate:
        _s = args.separation_mm / 1000.0 / 2.0
        kw['osc_point_a_m'] = (-_s, 0.0)
        kw['osc_point_b_m'] = (+_s, 0.0)
    cfg = SelfCatchConfig(**kw)

    res = run(cfg)
    print(f"[juggle_selfcatch] sustained {res.sustained}/{cfg.n_cycles} cycles "
          f"({'oscillation' if cfg.oscillate else 'column'}, release={cfg.release})")
    print(f"[juggle_selfcatch] in-cup offset trend (mm): {res.in_off_trend_mm}")
    if cfg.oscillate:
        print(f"[juggle_selfcatch] landing-err trend (mm):   {res.landing_err_trend_mm}")
    print(f"[juggle_selfcatch] reach trend (mm):         {res.reach_trend_mm}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
