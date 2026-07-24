"""Headless, seeded reload-catch gate — the Phase-6 production-in-the-loop harness.

Drives the **actual** ``motion.trajectory`` planner + emitter (``planner.build_catch``
→ ``KnotEmitter`` → a real ``SetpointPump``) into the MuJoCo sim plant, throws a
ballistic ball under the §3 juggle noise, arms the **arm-and-forget** Teensy hand
mirror (``sim.hand.trajectory.HandCatchSequence``), and measures the Reload-gate
criteria across N seeded trials. This is the harness the plan's "production-in-the-
loop rule" requires: a sim pass transfers to hardware because every command flows
through the same planner/gate/pump the robot uses, and *every emitted knot is
accepted by a real SetpointPump* (re-asserted here, not assumed).

Reframed per the Phase-6 "hand-catch smoothness is a sim-fidelity work item": the
hardware arm-and-forget path already catches smoothly, so the sim must *reproduce*
that — a Python mirror of the Teensy catch generator (accelerate to
−CATCH_VEL_RATIO·v_ball = −0.6·v_ball per the config source of truth, a
short constant-velocity hold centred on arrival, constant decel) with a realistic
arm latency — not the bb sims' continuous sub-tick velocity-matched hand commands
(which never existed on hardware). The MuJoCo capture model is contact→kinematic-
hold, so a caught ball never separates; the gate's remaining work is (a) the
platform reaching *under* the observed ball with the receive tilt, (b) the hand
meeting the ball at a low relative velocity, and (c) the platform holding quiescent
while the ball seats.

Reload-gate criteria (plan § Hand-catch smoothness / Sim-gate definitions):
  * ≥ 18/20 catches under noise (2 % BB speed, 0.5 mm tracking) across 5 landing
    offsets within ±60 mm xy and arrival speeds 2.5–4.0 m/s;
  * |v_hand − v_ball| at first contact ≤ 15 % of |v_ball|;
  * no ball–cup separation > 10 ms after first contact (automatic under the
    contact→hold capture model — reported for completeness);
  * hold-phase platform centroid travel < 5 mm and tilt change < 1° from
    ``t_arrival + 0.15 s`` until the return move;
  * zero feasibility violations in accepted runs; all knots pump-accepted.

Run headless (default) — writes a JSON gate report to ``temp/reports/``. The full
20-run gate is ``--trials 20``; ``tests/sim/test_reload_gate.py`` runs a small-N
smoke in CI.

Pure-Python + MuJoCo; no ROS2. SI at the ball/hand interface, mm at the plant/
trajectory boundary (mirrors the rest of the runner).
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
import sys
import time

import numpy as np

_sim_dir = os.path.dirname(os.path.abspath(__file__))
_repo_root = os.path.dirname(_sim_dir)
for _p in (_repo_root, _sim_dir,
           os.path.join(_repo_root, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_repo_root, 'config', 'generated')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import (
    KnotEmitter, TrajectoryLimits, tilt_to_receive,
)
from jugglebot.motion.trajectory import ballistics_bc as bal
from jugglebot.motion.trajectory import planner
from jugglebot.motion.trajectory.feasibility import TrajectoryInfeasible
from jugglebot.motion.trajectory.shaping import cup_lateral_shift_mm

from controller.teensy_link.setpoint_pump import SetpointPump

from plant.mujoco_plant import MuJoCoPlant
from hand.trajectory import HandCatchTrajectory, HandCatchSequence
from juggle_noise import JuggleNoise, NoiseConfig, BallisticEstimator

# ── Sim/cup geometry constants (measured against sim/model/jugglebot.xml) ──────
# cup_opening_world_z = CUP_Z_BASE_MM + hand_pos_mm + (platform_z_stow − Z_ACTIVE).
# Probed 2026-07-08: neutral (z=170) + hand=0 → 659.6 mm; +1 mm hand → +1 mm cup;
# +1 mm platform z → +1 mm cup (see the logbook Design section).
CUP_Z_BASE_MM = 659.6
Z_ACTIVE_MM = 170.0                       # STOW-relative neutral platform z
GRAVITY_MMS2 = bal.GRAVITY_MMS2
KNOT_DT_S = 0.025
NEUTRAL_POSE = np.array([0.0, 0.0, Z_ACTIVE_MM, 0.0, 0.0, 0.0])

# The 5 landing offsets (mm xy) within ±60 mm, and 4 arrival speeds (m/s) in
# 2.5–4.0 — the 20-run gate is the 5×4 grid (one seed per cell).
_LANDING_OFFSETS_MM = (
    (0.0, 0.0), (45.0, 0.0), (-30.0, 25.0), (20.0, -35.0), (-35.0, -20.0),
)
_ARRIVAL_SPEEDS_MPS = (2.5, 3.0, 3.5, 4.0)

# Ball radius (mm) — from sim/model/jugglebot.xml ball_geom size (0.035 m).
# Informational: the ball-cup capture instant (contact→hold) is a coupled function
# of this radius + the co-moving cup, empirically ~+45 ms past centre-arrival; the
# hand hold is anchored via `capture_offset_s`, not a closed-form BALL_R/|v| lead.
BALL_R_MM = 35.0


@dataclasses.dataclass
class ReloadGateConfig:
    trials: int = 20
    seed: int = 0
    lead_s: float = 0.7                    # reach lead (plan install → arrival)
    settle_hold_s: float = float(hw.JB_TRAJ_CATCH_SETTLE_HOLD_S)   # 0.5
    # Catch-capable session limits (above the Phase-1 defaults; the gate reports the
    # measured requirement back to Phase 4). Clamped to the YAML ceilings.
    leg_vel_mmps: float = 250.0
    leg_acc_mmps2: float = 3000.0
    leg_jerk_mmps3: float = 150000.0
    # §3 noise (both ON by default; the gate must hold under it).
    bb_throw_noise_frac: float = 0.02
    tracking_noise_mm: float = 0.5
    # Robustness sweep (arm-time error / event-vel error) — 0 = nominal.
    arm_time_err_s: float = 0.0
    event_vel_err_frac: float = 0.0
    spawn_lead_s: float = 0.32             # ball spawned this long before arrival
    # Offset (s) placing the hand velocity-hold MIDPOINT (t=0, where the hand is at
    # its −CATCH_VEL_RATIO·v_ball = −0.6·v_ball matched speed) relative to nominal
    # arrival. This is the
    # empirically-swept COUPLED fixed point (2026-07-08): the −10 ms anchor holds the
    # ~14 ms velocity-hold so that the actual MuJoCo capture — which fires ~+16-20 ms
    # PAST nominal arrival under the co-moving-cup contact geometry (the ball descends
    # into the raised, moving cup before contacting the collision geom, and raising
    # the cup shifts capture earlier — hence coupled, not a closed-form BALL_R/|v|
    # lead) — lands nearest the hold window. See the logbook Discussion.
    capture_offset_s: float = -0.010
    report_path: str | None = None


# ── Acceptance thresholds (plan § Hand-catch smoothness) ──
VEL_MATCH_FRAC = 0.15          # |v_hand − v_ball| ≤ 15 % of |v_ball| at contact
HOLD_TRAVEL_MM = 5.0           # platform centroid travel over the hold window
HOLD_TILT_DEG = 1.0            # platform tilt change over the hold window
SEPARATION_MS = 10.0           # max post-contact ball–cup separation
GATE_PASS_FRACTION = 18.0 / 20.0
REACH_ENVELOPE_MM = 80.0       # sim-established reliable reach


@dataclasses.dataclass
class _CatchSetup:
    """The synthesised + observed catch inputs `build_catch` needs for one catch,
    computed independently of the plant state (so the same synthesis feeds the
    reset-from-neutral gate trial and the no-reset sequential catch)."""
    v_ball: float
    armed_v: float = 0.0
    catch_pose: np.ndarray | None = None
    rx: float = 0.0
    ry: float = 0.0
    reach_mm: float = 0.0
    cup_world_z: float = 0.0
    spawn_pos_n: np.ndarray | None = None
    spawn_vel_n: np.ndarray | None = None
    act_T: float = 0.0
    reject_code: str | None = None


@dataclasses.dataclass
class TrialResult:
    idx: int
    offset_mm: tuple
    arrival_speed_mps: float
    accepted: bool                 # build_catch feasible
    reject_code: str | None
    caught: bool
    held_at_end: bool
    vel_match_frac: float          # |v_hand − v_ball| / |v_ball| at contact
    hold_travel_mm: float
    hold_tilt_deg: float
    separation_ms: float
    reach_mm: float
    capture_rel_ms: float          # capture time − nominal arrival (ms; <0 = early)
    pump_rejects: int
    peak_leg_vel_mmps: float
    peak_leg_acc_mmps2: float
    peak_leg_jerk_mmps3: float
    clean: bool

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        d['offset_mm'] = list(self.offset_mm)
        return d


class _ViewerHook:
    """Optional interactive MuJoCo viewer for watching gate/sequential runs.

    Purely observational: attaches a passive viewer to the plant's model/data
    (which survive ``plant.reset`` — the plant resets MjData in place) and paces
    the sim to ``speed``× real time with wall-clock sleeps OUTSIDE the physics,
    so gate math, seeding, and PASS/FAIL are bit-identical to a headless run.
    Closing the viewer window stops the run cleanly via ``ViewerClosed``.

    Requires a display (``pip install mujoco`` ships the viewer). Lazy-imported
    so headless machines never touch it.
    """

    def __init__(self, plant, speed: float = 1.0):
        import mujoco.viewer  # lazy: needs a display only when actually used
        self._viewer = mujoco.viewer.launch_passive(plant.model, plant.data)
        self._speed = max(1e-3, float(speed))
        self._plant = plant
        self._wall0 = time.time()
        self._sim0 = float(plant.data.time)

    def sync(self):
        if not self._viewer.is_running():
            raise ViewerClosed()
        self._viewer.sync()
        # Pace sim time to speed× wall time (sleep the surplus, never rush).
        target_wall = self._wall0 + (float(self._plant.data.time) - self._sim0) / self._speed
        lag = target_wall - time.time()
        if lag > 0:
            time.sleep(lag)

    def close(self):
        try:
            self._viewer.close()
        except Exception:
            pass


class ViewerClosed(Exception):
    """Raised by _ViewerHook.sync() when the operator closes the viewer window."""


class ReloadGate:
    """Owns one MuJoCo plant + emitter + pump for the whole gate run."""

    def __init__(self, cfg: ReloadGateConfig):
        self.cfg = cfg
        self.geom = StewartGeometry()
        self.plant = MuJoCoPlant(geom=self.geom)
        self.viewer = None  # optional _ViewerHook, attached by main() on --viewer
        self.emitter = KnotEmitter(self.geom)
        self.limits = TrajectoryLimits.from_config(hw).with_session_limits(
            leg_vel_mmps=cfg.leg_vel_mmps, leg_acc_mmps2=cfg.leg_acc_mmps2,
            leg_jerk_mmps3=cfg.leg_jerk_mmps3)
        self.noise_cfg = NoiseConfig(bb_throw_noise_frac=cfg.bb_throw_noise_frac,
                                     tracking_noise_mm=cfg.tracking_noise_mm)

    # ---- geometry helpers -------------------------------------------------
    def _catch_pose(self, obs_landing_xy_mm, obs_v_arrival_mms, cup_world_z_mm):
        """The build_catch target pose from an OBSERVED landing + arrival velocity.

        Receive tilt from the arrival velocity (cup axis collinear); platform xy
        offset by −lever-arm shift so the *tilted* cup lands on the observed xy;
        platform z chosen so the cup opening (at the hand-at-arrival position) sits
        at ``cup_world_z_mm``."""
        rx, ry = tilt_to_receive(obs_v_arrival_mms)
        shift = cup_lateral_shift_mm(rx, ry, cup_z_mm=cup_world_z_mm)
        cx = float(obs_landing_xy_mm[0]) - float(shift[0])
        cy = float(obs_landing_xy_mm[1]) - float(shift[1])
        return np.array([cx, cy, Z_ACTIVE_MM, rx, ry, 0.0]), (rx, ry)

    # ---- catch synthesis (shared by run_trial and run_sequential) ---------
    def _prepare_catch(self, offset_mm, arrival_speed_mps: float,
                       seed: int) -> _CatchSetup:
        """Synthesise the thrown-ball spawn state (under §3 noise), observe it,
        and build the ``build_catch`` target pose — everything up to (not
        including) the planner call. Shared verbatim by both the standalone
        20-run gate (``run_trial``) and the sequential-catch mode
        (``run_sequential``); the only difference between the two is the planner
        ``state0`` (neutral vs the settled/returned plant state) and whether the
        plant is reset first."""
        cfg = self.cfg
        rng = np.random.default_rng(seed)
        noise = JuggleNoise(self.noise_cfg, seed=seed)

        v_ball = float(arrival_speed_mps)
        # Hand position at arrival (t=0, the velocity-hold midpoint) for THIS speed,
        # which sets the cup opening world-z the ball must arrive at.
        cat = HandCatchTrajectory(v_ball)
        hand_at_arrival_mm = cat.sample(0.0)
        cup_world_z = CUP_Z_BASE_MM + hand_at_arrival_mm       # at neutral platform z

        # TRUE arrival: at the landing offset, cup height, descending at v_ball with
        # a lateral component (so the receive tilt is exercised). Random azimuth.
        az = rng.uniform(0.0, 2.0 * np.pi)
        lat_frac = rng.uniform(0.05, 0.18)                     # keeps tilt < 12°
        vxy = v_ball * 1000.0 * lat_frac
        v_arr_true = np.array([vxy * np.cos(az), vxy * np.sin(az),
                               -v_ball * 1000.0 * np.sqrt(max(1 - lat_frac ** 2, 0.0))])
        landing_true = np.array([offset_mm[0], offset_mm[1], cup_world_z])

        # Synthesise the ball's SPAWN state a short spawn_lead before arrival and
        # apply the §3 BB throw noise THERE — the apex/descent-spawn model the bb sim
        # uses (juggle_bb_catch spawns at the apex with noise on). Applying the 2 %
        # noise at the full ground-launch release instead integrates a steep throw's
        # large velocity noise over the whole flight into a ~150 mm landing scatter
        # (unrealistic, uncatchable); the descent-spawn model scatters the landing
        # within the ~80 mm reliable envelope, as intended.
        spawn_lead = cfg.spawn_lead_s
        spawn_vel_ideal = v_arr_true - bal.G_VEC_MMS2 * spawn_lead
        spawn_pos_ideal = (landing_true - spawn_vel_ideal * spawn_lead
                           - 0.5 * bal.G_VEC_MMS2 * spawn_lead ** 2)
        disp = landing_true - spawn_pos_ideal
        spawn_pos_n, spawn_vel_n = noise.perturb_throw(
            spawn_pos_ideal, spawn_vel_ideal, disp)

        # ACTUAL arrival of the perturbed ball at the cup height (descending root).
        try:
            act_landing, act_v_arr, act_T = bal.arrival_state_at_z(
                spawn_pos_n, spawn_vel_n, cup_world_z, descending=True)
        except ValueError:
            return _CatchSetup(v_ball=v_ball, reject_code='BALLISTIC')

        # OBSERVE: fit a known-gravity ballistic arc to noisy observations over the
        # descent (the tracking-noise-averaging BallisticEstimator, NOT a raw finite
        # difference — differencing amplifies the 0.5 mm noise into m/s velocity
        # error), then propagate to the cup crossing → the estimate the planner
        # reaches for.
        est = BallisticEstimator(bal.G_VEC_MMS2)
        for k in range(7):
            tt = 0.02 + (act_T - 0.06) * k / 6.0
            est.add(tt, noise.observe(bal.position_at(spawn_pos_n, spawn_vel_n, tt)))
        p_est, v_est = est.estimate()
        try:
            obs_landing, obs_v_arr, _ = bal.arrival_state_at_z(
                p_est, v_est, cup_world_z, descending=True)
        except ValueError:
            return _CatchSetup(v_ball=v_ball, reject_code='BALLISTIC')

        # event_vel error (robustness sweep) perturbs the speed the hand is armed with.
        armed_v = v_ball * (1.0 + cfg.event_vel_err_frac)

        catch_pose, (rx, ry) = self._catch_pose(
            obs_landing[:2], obs_v_arr, cup_world_z)
        reach_mm = float(np.hypot(catch_pose[0], catch_pose[1]))

        return _CatchSetup(
            v_ball=v_ball, armed_v=armed_v, catch_pose=catch_pose, rx=rx, ry=ry,
            reach_mm=reach_mm, cup_world_z=cup_world_z, spawn_pos_n=spawn_pos_n,
            spawn_vel_n=spawn_vel_n, act_T=act_T)

    # ---- one trial (standalone gate — always reset, plan from neutral) ----
    def run_trial(self, idx: int, offset_mm, arrival_speed_mps: float,
                  seed: int) -> TrialResult:
        cfg = self.cfg
        s = self._prepare_catch(offset_mm, arrival_speed_mps, seed)
        if s.reject_code is not None:
            return self._reject(idx, offset_mm, s.v_ball, s.reject_code,
                                reach_mm=s.reach_mm)
        # PLAN via the production constructor (the single gate). A reject is loud.
        try:
            plan, report = planner.build_catch(
                (NEUTRAL_POSE, np.zeros(6), np.zeros(6)), s.catch_pose,
                cfg.lead_s, self.limits, self.geom, settle_hold_s=cfg.settle_hold_s)
        except TrajectoryInfeasible as e:
            return self._reject(idx, offset_mm, s.v_ball, e.code, reach_mm=s.reach_mm)
        return self._simulate(idx, offset_mm, s.v_ball, plan, report, s.reach_mm,
                              s.armed_v, s.spawn_pos_n, s.spawn_vel_n, s.act_T,
                              s.cup_world_z, s.rx, s.ry)

    def _simulate(self, idx, offset_mm, v_ball, plan, report, reach_mm, armed_v,
                  spawn_pos_n, spawn_vel_n, act_T, cup_world_z, rx, ry,
                  reset=True, run_full_plan=False):
        """Drive one catch through the plant. ``reset`` (default) resets+settles the
        plant at neutral first (the standalone gate); the sequential mode passes
        ``reset=False`` so successive catches run on the *same* plant state (the
        prior catch's settled/returned pose). ``run_full_plan`` extends the sim
        window to cover a return-to-neutral tail (so the platform is back at neutral
        for the next sequential catch), instead of stopping at the hold."""
        cfg = self.cfg
        plant, emitter = self.plant, self.emitter
        pump = SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV,
                            max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV)
        model_dt = plant.timestep
        substeps = max(1, int(round(KNOT_DT_S / model_dt)))

        # Settle the platform at neutral with the hand (re-)primed near the catch
        # start. ``reset=False`` (sequential) keeps the prior catch's plant state and
        # held ball — the platform is already near neutral from the prior return, and
        # re-priming the hand re-arms the arm-and-forget catch (the held ball rides up
        # with the hand, then is re-spawned as the next thrown ball below).
        if reset:
            plant.reset(NEUTRAL_POSE)
        plant.command(plant.pose_to_extensions(NEUTRAL_POSE))
        hand_prime = HandCatchTrajectory(armed_v).start_pos_mm
        plant.command_hand(hand_prime)
        for _ in range(60):
            plant.step(KNOT_DT_S)
            if self.viewer is not None:
                self.viewer.sync()

        t_install = plant.data.time
        arrival_sim = t_install + cfg.lead_s
        # Arm the hand (arm-and-forget). Anchor the velocity-hold midpoint (t=0) to
        # the SURFACE-contact instant — the ball surface meets the cup ~BALL_R/|v|
        # before its centre reaches the cup opening — so the hand is at its
        # velocity-matched hold speed when the ball first touches. A realistic
        # arm-time error is added on top for the robustness sweep.
        hand_arrival = arrival_sim + cfg.capture_offset_s + cfg.arm_time_err_s
        seq = HandCatchSequence(armed_v, hand_arrival, hand_prime)
        self._last_capture_rel_ms = float('nan')

        # Spawn the (already-noised) ball act_T before arrival so it reaches the cup
        # height at arrival_sim (ballistic — the spawn state carries the rest of the
        # flight exactly).
        spawn_at = arrival_sim - act_T

        run_until = max(seq.end_time, arrival_sim + cfg.settle_hold_s) + 0.05
        if run_full_plan:
            # Run through the whole assembled plan (including any return-to-neutral
            # tail) so the platform is settled at neutral for the next catch.
            plan_dur = sum(float(s.duration) for s in plan.segments)
            run_until = max(run_until, t_install + plan_dur + 0.05)
        caught = False
        v_match = np.nan
        separation_ms = 0.0
        contact_lost_since = None
        spawned = False
        n_frames = 0
        # Hold-quiescence sampling window: t_arrival + 0.15 s → end of hold.
        hold_start = arrival_sim + 0.15
        hold_end = arrival_sim + cfg.settle_hold_s
        hold_pos = []
        hold_rot = []

        while plant.data.time < run_until:
            # Platform setpoint updated at the 25 ms knot boundary (the streamed
            # SETPOINT rate); every emitted knot goes through the real pump.
            tau = plant.data.time - t_install
            frame = emitter.frame(plan, tau, n_frames)
            sp, reason = pump.build(frame, t_origin_us=int(n_frames * 25000))
            n_frames += 1
            plant.command(np.asarray(frame['ext_mm']))
            # Physics substeps: the hand runs at the HIGH rate the Teensy executes
            # its arm-and-forget catch profile (not the 25 ms platform knot rate), so
            # the ~14 ms velocity-hold is actually resolved at contact — sample the
            # analytic hand profile every substep.
            for _ in range(substeps):
                if plant.data.time >= run_until:
                    break
                hp = seq.sample(plant.data.time)
                if hp is not None:
                    plant.command_hand(hp)
                if not spawned and plant.data.time >= spawn_at:
                    plant.spawn_ball(spawn_pos_n, spawn_vel_n)
                    spawned = True
                plant.step(model_dt)
                if self.viewer is not None:
                    self.viewer.sync()

                if spawned and not caught and plant.check_and_capture():
                    caught = True
                    self._last_capture_rel_ms = (plant.data.time - arrival_sim) * 1000.0
                    # |v_hand − v_ball| at first contact (world frame; both are along
                    # the tilted cup axis, so this is the axial velocity mismatch).
                    bs = plant.get_ball_state()
                    v_ball_vec = bs.velocity_mms
                    v_hand_vec = plant.ball_manager.ball(0)._hand_site_velocity() * 1000.0
                    v_match = float(np.linalg.norm(v_ball_vec - v_hand_vec)
                                    / max(np.linalg.norm(v_ball_vec), 1e-6))
                if caught:
                    if not plant.get_ball_state().held:
                        if contact_lost_since is None:
                            contact_lost_since = plant.data.time
                        separation_ms = max(
                            separation_ms,
                            (plant.data.time - contact_lost_since) * 1000.0)
                    else:
                        contact_lost_since = None
                if hold_start <= plant.data.time <= hold_end:
                    st = plant.get_state()
                    hold_pos.append(st.platform_pos_mm.copy())
                    hold_rot.append(st.platform_rot.copy())

        held_at_end = bool(plant.get_ball_state().held) if plant.has_ball else False
        hold_travel = self._travel(hold_pos)
        hold_tilt = self._tilt_change_deg(hold_rot)

        clean = bool(
            caught and held_at_end
            and (not np.isnan(v_match)) and v_match <= VEL_MATCH_FRAC
            and hold_travel < HOLD_TRAVEL_MM and hold_tilt < HOLD_TILT_DEG
            and separation_ms <= SEPARATION_MS
            and pump.frames_rejected == 0
            and reach_mm <= REACH_ENVELOPE_MM)

        return TrialResult(
            idx=idx, offset_mm=tuple(offset_mm), arrival_speed_mps=v_ball,
            accepted=True, reject_code=None, caught=caught,
            held_at_end=held_at_end, vel_match_frac=float(v_match),
            hold_travel_mm=float(hold_travel), hold_tilt_deg=float(hold_tilt),
            separation_ms=float(separation_ms), reach_mm=reach_mm,
            capture_rel_ms=float(self._last_capture_rel_ms),
            pump_rejects=int(pump.frames_rejected),
            peak_leg_vel_mmps=float(report.peak_leg_vel_mmps),
            peak_leg_acc_mmps2=float(report.peak_leg_acc_mmps2),
            peak_leg_jerk_mmps3=float(report.peak_leg_jerk_mmps3),
            clean=clean)

    @staticmethod
    def _reject(idx, offset_mm, v_ball, code, reach_mm=0.0) -> TrialResult:
        return TrialResult(
            idx=idx, offset_mm=tuple(offset_mm), arrival_speed_mps=float(v_ball),
            accepted=False, reject_code=code, caught=False, held_at_end=False,
            vel_match_frac=float('nan'), hold_travel_mm=0.0, hold_tilt_deg=0.0,
            separation_ms=0.0, reach_mm=float(reach_mm), capture_rel_ms=float('nan'),
            pump_rejects=0,
            peak_leg_vel_mmps=0.0, peak_leg_acc_mmps2=0.0, peak_leg_jerk_mmps3=0.0,
            clean=False)

    @staticmethod
    def _travel(samples) -> float:
        if len(samples) < 2:
            return 0.0
        arr = np.asarray(samples)
        return float(np.max(np.linalg.norm(arr - arr[0], axis=1)))

    @staticmethod
    def _tilt_change_deg(rot_samples) -> float:
        if len(rot_samples) < 2:
            return 0.0
        arr = np.asarray(rot_samples)
        return float(np.degrees(np.max(np.linalg.norm(arr - arr[0], axis=1))))

    # ---- the full gate ----------------------------------------------------
    def run(self) -> dict:
        cfg = self.cfg
        results = []
        for i in range(cfg.trials):
            offset = _LANDING_OFFSETS_MM[i % len(_LANDING_OFFSETS_MM)]
            speed = _ARRIVAL_SPEEDS_MPS[i % len(_ARRIVAL_SPEEDS_MPS)]
            results.append(self.run_trial(i, offset, speed, seed=cfg.seed + i))
        return self._summarise(results)

    # ---- the sequential-catch mode (inherited item d) ---------------------
    def run_sequential(self, n_catches: int, seed: int) -> dict:
        """N catches back-to-back on ONE plant WITHOUT resetting between them.

        Each catch: reach → tilt-through-seat → settle-hold → (for every catch but
        the last) a ``hold_after=False`` return toward neutral; then the next thrown
        ball is spawned and a SECOND ``build_catch`` is planned *from the settled /
        returned plant state* (read off the plant, not reset to neutral). Proves the
        planner + plant repeat a catch from a realistic non-home start and re-quiesce
        each time.

        Scope caveat (documented in the logbook): this harness drives the planner and
        plant directly and does NOT go through ``trajectory_node``, so the node-level
        settle-bounded reach-freeze / freeze-release across repeated catches is NOT
        exercised here — that is covered by the Phase-5 node tests. This mode covers
        the planner + plant repetition only."""
        cfg = self.cfg
        catches = []
        for i in range(n_catches):
            offset = _LANDING_OFFSETS_MM[i % len(_LANDING_OFFSETS_MM)]
            speed = _ARRIVAL_SPEEDS_MPS[i % len(_ARRIVAL_SPEEDS_MPS)]
            s = self._prepare_catch(offset, speed, seed + i)
            if s.reject_code is not None:
                catches.append(self._reject(i, offset, s.v_ball, s.reject_code,
                                            reach_mm=s.reach_mm))
                continue
            last = (i == n_catches - 1)
            if i == 0:
                state0 = (NEUTRAL_POSE, np.zeros(6), np.zeros(6))
            else:
                # Plan the next catch from the ACTUAL settled/returned plant pose
                # (twist/accel ~0 after the return hold) — not a reset to neutral.
                st = self.plant.get_state()
                pose0 = np.concatenate([st.platform_pos_mm, st.platform_rot])
                state0 = (pose0, np.zeros(6), np.zeros(6))
            try:
                plan, report = planner.build_catch(
                    state0, s.catch_pose, cfg.lead_s, self.limits, self.geom,
                    settle_hold_s=cfg.settle_hold_s,
                    hold_after=last, neutral_pose=None if last else NEUTRAL_POSE)
            except TrajectoryInfeasible as e:
                catches.append(self._reject(i, offset, s.v_ball, e.code,
                                            reach_mm=s.reach_mm))
                continue
            catches.append(self._simulate(
                i, offset, s.v_ball, plan, report, s.reach_mm, s.armed_v,
                s.spawn_pos_n, s.spawn_vel_n, s.act_T, s.cup_world_z, s.rx, s.ry,
                reset=(i == 0), run_full_plan=True))
        return self._summarise_sequential(catches)

    def _summarise_sequential(self, catches) -> dict:
        quiescent = [c for c in catches
                     if c.hold_travel_mm < HOLD_TRAVEL_MM
                     and c.hold_tilt_deg < HOLD_TILT_DEG]
        return {
            'gate': 'reload_sequential',
            'n_catches': len(catches),
            'all_caught': all(c.caught for c in catches),
            'all_held': all(c.held_at_end for c in catches),
            'all_quiescent': len(quiescent) == len(catches),
            'worst_hold_travel_mm': float(max((c.hold_travel_mm for c in catches),
                                              default=0.0)),
            'worst_hold_tilt_deg': float(max((c.hold_tilt_deg for c in catches),
                                             default=0.0)),
            'catches': [c.to_dict() for c in catches],
            'config': dataclasses.asdict(self.cfg),
            'thresholds': {'hold_travel_mm': HOLD_TRAVEL_MM,
                           'hold_tilt_deg': HOLD_TILT_DEG},
        }

    @staticmethod
    def _core_clean(r) -> bool:
        """A 'core' catch: every Reload-gate criterion EXCEPT the hand-contact
        velocity-match (which is a light-scope deferral — see the logbook). Caught,
        held, hold-quiescent, no separation, pump-clean, reach in envelope."""
        return bool(
            r.caught and r.held_at_end
            and r.hold_travel_mm < HOLD_TRAVEL_MM and r.hold_tilt_deg < HOLD_TILT_DEG
            and r.separation_ms <= SEPARATION_MS and r.pump_rejects == 0
            and r.reach_mm <= REACH_ENVELOPE_MM)

    def _summarise(self, results) -> dict:
        n = len(results)
        accepted = [r for r in results if r.accepted]
        clean = [r for r in results if r.clean]
        core_clean = [r for r in results if self._core_clean(r)]
        caught = [r for r in results if r.caught]
        feas_violations = sum(1 for r in accepted if r.reject_code is not None)
        pump_rejects = sum(r.pump_rejects for r in results)
        # Required limits = the max measured leg peaks across ACCEPTED runs, with a
        # 1.15× headroom so the session limit isn't sitting exactly on the peak.
        req_vel = max((r.peak_leg_vel_mmps for r in accepted), default=0.0) * 1.15
        req_acc = max((r.peak_leg_acc_mmps2 for r in accepted), default=0.0) * 1.15
        req_jerk = max((r.peak_leg_jerk_mmps3 for r in accepted), default=0.0) * 1.15
        vmatch = [r.vel_match_frac for r in caught if not np.isnan(r.vel_match_frac)]
        threshold = int(round(GATE_PASS_FRACTION * n))
        # FULL gate (includes the vel-match criterion — DEFERRED per light-scope).
        passed = (len(clean) >= threshold
                  and feas_violations == 0 and pump_rejects == 0)
        # CORE gate (every criterion except the deferred vel-match): the software
        # actually validated this phase.
        core_passed = (len(core_clean) >= threshold
                       and feas_violations == 0 and pump_rejects == 0)
        return {
            'gate': 'reload',
            'passed': bool(passed),
            'core_passed': bool(core_passed),
            'deferred_criteria': ['vel_match_frac'],
            'deferred_note': (
                'Hand-contact velocity-match (|v_hand-v_ball| <= 15% at first '
                'contact) is a light-scope DEFERRAL: it floors at ~0.26 in this '
                'MuJoCo contact->instant-hold model because the Teensy velocity-hold '
                'window (~14 ms) is narrower than the achievable ball-cup capture '
                'timing alignment (~+-20 ms), so the hand is ~74% of the ball speed '
                'at capture. The cup axis IS correctly aligned (lateral and axial '
                'mismatch scale identically). See the logbook Open Questions.'),
            'trials': n,
            'clean': len(clean),
            'core_clean': len(core_clean),
            'caught': len(caught),
            'accepted': len(accepted),
            'threshold': threshold,
            'feasibility_violations_in_accepted': feas_violations,
            'total_pump_rejects': pump_rejects,
            'worst_vel_match_frac': float(max(vmatch)) if vmatch else None,
            'mean_vel_match_frac': float(np.mean(vmatch)) if vmatch else None,
            'worst_hold_travel_mm': float(max((r.hold_travel_mm for r in caught),
                                              default=0.0)),
            'worst_hold_tilt_deg': float(max((r.hold_tilt_deg for r in caught),
                                             default=0.0)),
            'worst_separation_ms': float(max((r.separation_ms for r in results),
                                             default=0.0)),
            'required_leg_vel_mmps': float(req_vel),
            'required_leg_acc_mmps2': float(req_acc),
            'required_leg_jerk_mmps3': float(req_jerk),
            'config': dataclasses.asdict(self.cfg),
            'thresholds': {
                'vel_match_frac': VEL_MATCH_FRAC, 'hold_travel_mm': HOLD_TRAVEL_MM,
                'hold_tilt_deg': HOLD_TILT_DEG, 'separation_ms': SEPARATION_MS,
                'reach_envelope_mm': REACH_ENVELOPE_MM,
            },
            'results': [r.to_dict() for r in results],
        }


def _attach_viewer(gate: 'ReloadGate', viewer_speed) -> bool:
    """Attach an interactive viewer to a constructed gate (``--viewer``).
    Returns True on success; on failure (headless machine, no GLFW) prints the
    reason and returns False so the caller proceeds headless."""
    if viewer_speed is None:
        return False
    try:
        gate.viewer = _ViewerHook(gate.plant, speed=viewer_speed)
        print(f"[reload_gate] viewer attached ({viewer_speed:g}x real time) — "
              "pacing sleeps sit outside the physics; results are bit-identical "
              "to a headless run. Close the window to stop.")
        return True
    except Exception as e:  # no display / viewer backend unavailable
        print(f"[reload_gate] viewer unavailable ({e}) — continuing headless.")
        return False


def run_gate(cfg: ReloadGateConfig, viewer_speed=None) -> dict:
    gate = ReloadGate(cfg)
    _attach_viewer(gate, viewer_speed)
    try:
        report = gate.run()
    except ViewerClosed:
        print("[reload_gate] viewer closed — run stopped by operator (no report).")
        raise SystemExit(130)
    finally:
        if gate.viewer is not None:
            gate.viewer.close()
    path = cfg.report_path
    if path is None:
        os.makedirs(os.path.join(_repo_root, 'temp', 'reports'), exist_ok=True)
        path = os.path.join(_repo_root, 'temp', 'reports',
                            f"reload_gate_seed{cfg.seed}_n{cfg.trials}.json")
    with open(path, 'w') as f:
        json.dump(report, f, indent=2)
    report['report_path'] = path
    return report


def _run_sequential_cli(cfg: ReloadGateConfig, n_catches: int, viewer_speed=None) -> int:
    """Run + report the sequential-catch mode; exit 0 iff every catch caught,
    held, and stayed hold-quiescent."""
    t0 = time.time()
    gate = ReloadGate(cfg)
    _attach_viewer(gate, viewer_speed)
    try:
        rep = gate.run_sequential(n_catches, seed=cfg.seed)
    except ViewerClosed:
        print("[reload_gate] viewer closed — run stopped by operator (no report).")
        raise SystemExit(130)
    finally:
        if gate.viewer is not None:
            gate.viewer.close()
    wall = time.time() - t0
    path = cfg.report_path
    if path is None:
        os.makedirs(os.path.join(_repo_root, 'temp', 'reports'), exist_ok=True)
        path = os.path.join(_repo_root, 'temp', 'reports',
                            f"reload_gate_sequential_n{n_catches}_seed{cfg.seed}.json")
    with open(path, 'w') as f:
        json.dump(rep, f, indent=2)
    ok = rep['all_caught'] and rep['all_held'] and rep['all_quiescent']
    print(f"[reload_gate] SEQUENTIAL {'PASS' if ok else 'FAIL'}  "
          f"n_catches {rep['n_catches']}  all_caught {rep['all_caught']}  "
          f"all_held {rep['all_held']}  all_quiescent {rep['all_quiescent']}  "
          f"worst hold travel {rep['worst_hold_travel_mm']:.3f} mm  "
          f"tilt {rep['worst_hold_tilt_deg']:.3f}°")
    for c in rep['catches']:
        print(f"[reload_gate]   catch {c['idx']}: caught {c['caught']}  "
              f"held {c['held_at_end']}  hold_travel {c['hold_travel_mm']:.3f} mm  "
              f"tilt {c['hold_tilt_deg']:.3f}°  capture_rel {c['capture_rel_ms']:.1f} ms")
    print(f"[reload_gate] (wall {wall:.1f}s) → {path}")
    return 0 if ok else 1


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Headless seeded reload-catch sim gate.")
    p.add_argument('--trials', type=int, default=20)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--lead-s', type=float, default=0.7)
    p.add_argument('--arm-time-err-s', type=float, default=0.0)
    p.add_argument('--event-vel-err-frac', type=float, default=0.0)
    p.add_argument('--sequential', type=int, default=0,
                   help="N: run N catches back-to-back on one plant (no reset "
                        "between catches) instead of the standalone gate.")
    p.add_argument('--viewer', action='store_true',
                   help="open an interactive MuJoCo viewer and pace to real time "
                        "(observational only — results are bit-identical to "
                        "headless; needs a display).")
    p.add_argument('--viewer-speed', type=float, default=1.0,
                   help="viewer pacing factor (0.25 = quarter-speed slow-mo; "
                        "only with --viewer).")
    p.add_argument('--report', default=None)
    args = p.parse_args(argv)
    viewer_speed = args.viewer_speed if args.viewer else None
    cfg = ReloadGateConfig(
        trials=args.trials, seed=args.seed, lead_s=args.lead_s,
        arm_time_err_s=args.arm_time_err_s,
        event_vel_err_frac=args.event_vel_err_frac, report_path=args.report)
    if args.sequential > 0:
        return _run_sequential_cli(cfg, args.sequential, viewer_speed=viewer_speed)
    t0 = time.time()
    rep = run_gate(cfg, viewer_speed=viewer_speed)
    wall = time.time() - t0
    print(f"[reload_gate] FULL {'PASS' if rep['passed'] else 'FAIL'}  "
          f"CORE {'PASS' if rep['core_passed'] else 'FAIL'} "
          f"(vel-match deferred)  "
          f"clean {rep['clean']}  core_clean {rep['core_clean']}/{rep['trials']} "
          f"(threshold {rep['threshold']})  "
          f"caught {rep['caught']}  accepted {rep['accepted']}  "
          f"feas_viol {rep['feasibility_violations_in_accepted']}  "
          f"pump_rejects {rep['total_pump_rejects']}")
    print(f"[reload_gate] worst vel-match {rep['worst_vel_match_frac']}  "
          f"hold travel {rep['worst_hold_travel_mm']:.2f} mm  "
          f"tilt {rep['worst_hold_tilt_deg']:.2f}°  "
          f"sep {rep['worst_separation_ms']:.1f} ms")
    print(f"[reload_gate] required limits: vel {rep['required_leg_vel_mmps']:.0f} "
          f"acc {rep['required_leg_acc_mmps2']:.0f} "
          f"jerk {rep['required_leg_jerk_mmps3']:.0f}  "
          f"(wall {wall:.1f}s)  → {rep['report_path']}")
    return 0 if rep['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
