"""Headless, seeded single-ball-toss gate — the Phase-2 production-in-the-loop
harness (``plans/active/single-ball-toss.md`` § Sim gate, Tier 8a).

Each trial drives the **actual** production stack end-to-end: the platform is
pre-positioned to the nominated catch pose through ``planner.build_move`` →
``KnotEmitter`` → a real ``SetpointPump`` (the go_to_pose path the toss FSM's
POSITIONING executes), the release state + self-announcement come from the
Phase-1 ``toss_release`` math (``compute_release_state`` /
``build_announcement_fields`` — the production math under test), the pre-tilt
catch plan is installed via ``planner.build_catch`` timed to the ANNOUNCED
landing (production: catch_coordinator pre-tilts on the announcement, before
any tracking exists), the hand throw stroke runs the Teensy mirror
``HandThrowSequence``, and the throw is realised as a seeded-noise kinematic
release of the in-plant ball. The post-release ball flies IN-PLANT; the gate
observes it through ``JuggleNoise.observe`` + the known-gravity
``BallisticEstimator`` (the mocap analogue) and arms the arm-and-forget
``HandCatchSequence`` ONCE from the propagated arrival — the estimate drives
exactly what the tracker drives in production: the hand catch timing, not the
platform (open-loop platform pivot).

**Gating capture model** — contact→kinematic-hold (``contact_carry=False``),
identical to ``sim/reload_gate.py``: the reload gate's Phase-6 reframe (and the
plan, § Sim gate) hold that *hardware catches are already smooth and the MuJoCo
contact model is the low-fidelity element*, so contact physics is deliberately
NOT load-bearing in the gating verdict. The kinematic release is therefore
``Ball.release(velocity_mms)`` — the mode-correct kinematic-hold ejector (same
mechanism as ``Ball.ballistic_release``: imposed velocity + contact cut +
re-capture guard). **Plan-wording delta, flagged**: the plan names
``Ball.ballistic_release``; that exact function runs verbatim in the NON-GATING
contact-physics diagnostic column (``--diag-release kinematic``), which is
contact-carried per its contract. See the Phase-2 logbook Discussion.

**Gate criterion (two binding bands, reported separately):**
  * the plan's band — every swept point with arrival speed 2-3 m/s
    (flights {0.55, 0.60} s ⇒ 12 points in the factored grid) needs
    ``core_clean >= 9/10``;
  * the orchestrator amendment (2026-07-25) — the T = 0.80 s points at
    z = 170 (all five xy positions) are ADDITIONALLY binding >= 9/10: the
    2-3 m/s band is hardware-marginal (< 0.7 s firmware kind-1 windup race,
    Phase-1 logbook Known limitations), so the hardware-relevant band must
    gate too.
``passed`` requires both bands, zero feasibility violations in accepted runs,
and zero pump rejects with every emitted frame ACCEPTED (the accepted==emitted
counter is the non-vacuous form of the invariant). Explicit-points/debug runs
whose grid intersects NEITHER band gate on the full band instead of vacuously
passing two empty ``all()``'s. "Cycle success" is ``core_clean`` — which
requires the arm-and-forget catch to have actually ARMED from tracking
(``catch_armed``; a ball falling into a statically-parked cup must not pass)
— the hand-contact velocity-match stays the reload gate's documented
light-scope deferral.

**Documented fidelity deltas** (see the Phase-2 logbook):
  * release-velocity noise (``release_vel_noise_frac`` = 1 %) is a PLACEHOLDER
    until Phase-5 T0 measures the real scatter — the gate re-runs with the
    measured value before any hardware catch attempt (plan § Sim gate). Unlike
    reload's descent-spawn noise, toss noise integrates over the WHOLE flight
    (landing σ ≈ 21-59 mm) onto an open-loop platform — the cup alone absorbs
    it; that is exactly what the gate measures.
  * the pre-position ``build_move`` omits the LeanShaper (hardware go_to_pose
    runs gain 0.6) — terminal pose identical, only the transit shape differs
    (the same delta reload_gate accepts on the catch side).
  * ``toss_release`` produces hardware-global numbers (ACTIVE cup plane
    809.08 mm) while the sim's probed cup plane differs by a ~40 mm-class
    constant frame delta. By design the launch VELOCITY is frame-invariant
    (it is what production commands), xy is shared, and every z delta is
    RECORDED (``release_frame_z_delta_mm``, ``announced_landing_err_ms``) —
    never reconciled. Do not inject sim-frame overrides into ``toss_release``.
  * z-sweep fixed-tracker-plane timing skew: production's ``ball_tracker_node``
    predicts landings at the FIXED active-z catch plane
    (``initial_height + active_z + hand_catch_offset``,
    ball_tracker_node.py:36-38), while this gate's estimator propagates to the
    nominated-z cup plane — so on the z-sweep points the hardware hand-arm
    timing will differ from the gate's by ~Δz/v_arrival, ≈ 7.6 ms at
    z = 200 / T = 0.80 (30 mm at ~3.9 m/s arrival). Well inside the ±20 ms
    capture alignment; recorded here, not modelled.

Run headless (default) — writes a JSON gate report to ``temp/reports/``. The
full factored sweep is 29 points × 10 trials = 290 gating trials (+ the
non-gating contact-physics diagnostic column); ``tests/sim/test_toss_gate.py``
runs a small-N smoke in CI.

Pure-Python + MuJoCo; no ROS2. SI at the ball/hand interface, mm at the plant/
trajectory boundary (mirrors ``reload_gate``).
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
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

import mujoco

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import (
    KnotEmitter, TrajectoryLimits, tilt_to_receive,
)
from jugglebot.motion.trajectory import ballistics_bc as bal
from jugglebot.motion.trajectory import planner
from jugglebot.motion.trajectory import toss_release
from jugglebot.motion.trajectory.feasibility import TrajectoryInfeasible
from jugglebot.motion.trajectory.shaping import cup_lateral_shift_mm

from controller.teensy_link.setpoint_pump import SetpointPump

from plant.mujoco_plant import MuJoCoPlant
from hand.trajectory import (
    HandCatchSequence, HandCatchTrajectory, HandThrowSequence, STROKE_MARGIN_MM,
)
from juggle_noise import JuggleNoise, NoiseConfig, BallisticEstimator

from gate_common import (
    CUP_Z_BASE_MM, HOLD_TILT_DEG, HOLD_TRAVEL_MM, KNOT_DT_S, NEUTRAL_POSE,
    SEPARATION_MS, VEL_MATCH_FRAC, Z_ACTIVE_MM, ViewerClosed, attach_viewer,
    tilt_change_deg, travel_mm,
)

# ── Sweep grid (plan § Sim gate; factored per the Phase-2 spec) ────────────────
_XY_POINTS_MM = ((0.0, 0.0), (60.0, 60.0), (60.0, -60.0),
                 (-60.0, 60.0), (-60.0, -60.0))
_FLIGHTS_S = (0.55, 0.60, 0.80, 0.95, 1.10)
_Z_SWEEP_MM = (140.0, 200.0)
_Z_SWEEP_FLIGHTS_S = (0.60, 0.80)

# The plan's 2-3 m/s binding band: 2.0 <= event_vel <= 3.0 m/s (the upper
# bound ⇔ flight <= 0.610 s; the lower bound is explicit so a slow-arrival
# debug point can never silently join the binding band).
_BINDING_SPEED_MIN_MPS = 2.0
_BINDING_SPEED_MPS = 3.0
# The orchestrator-amendment binding band: T = 0.80 s at z = 170 (all five xy).
_BINDING_T080_S = 0.80
# Flights below this are firmware-marginal on hardware (Phase-1 kind-1 windup).
_HW_MARGINAL_FLIGHT_S = 0.7

_HW_MARGINAL_NOTE = (
    "Flights < 0.7 s are firmware-marginal on hardware: the tracker-driven "
    "catch arm can land mid-throw-decel and the Teensy kind-1 windup budget "
    "silently drops it (lying ack) for flight <=~0.6 s at realistic pipeline "
    "latencies. The sim does not model this budget; T1 hardware floor is "
    "0.7 s. See logbook/2026-07-25-toss-phase1-action-sequencer-coordinator.md "
    "(Known limitations) and the plan risk register.")

_DEFERRED_NOTE = (
    'Hand-contact velocity-match (|v_hand-v_ball| <= 15% at first '
    'contact) is a light-scope DEFERRAL inherited from the reload gate: it '
    'floors at ~0.26 in this MuJoCo contact->instant-hold model because the '
    'Teensy velocity-hold window (~14 ms) is narrower than the achievable '
    'ball-cup capture timing alignment (~+-20 ms), so the hand is ~74% of the '
    'ball speed at capture. The cup axis IS correctly aligned (lateral and '
    'axial mismatch scale identically). See the reload-gate logbook Open '
    'Questions. 2026-07-25 2-trial calibration run showed worst 0.431 under '
    'the same contact->hold model.')


def default_grid(grid: str = 'factored'):
    """The sweep points as ``[(x, y, z, T), ...]``.

    ``factored`` (default): grid A — 5 xy poses × 5 flights at z = 170
    (25 points) + grid B — centre pose, z ∈ {140, 200} × flights {0.60, 0.80}
    (4 points) = 29 points. ``full``: the full 5 × 3 × 5 cross (75 points,
    post-T0 re-runs only).
    """
    pts = []
    if grid == 'full':
        for (x, y) in _XY_POINTS_MM:
            for z in (Z_ACTIVE_MM,) + _Z_SWEEP_MM:
                for T in _FLIGHTS_S:
                    pts.append((x, y, float(z), T))
        return pts
    for (x, y) in _XY_POINTS_MM:
        for T in _FLIGHTS_S:
            pts.append((x, y, Z_ACTIVE_MM, T))
    for z in _Z_SWEEP_MM:
        for T in _Z_SWEEP_FLIGHTS_S:
            pts.append((0.0, 0.0, float(z), T))
    return pts


def _point_id(point) -> str:
    x, y, z, T = point
    return f"x{x:g}_y{y:g}_z{z:g}_T{T:.2f}"


def _pass_threshold(n: int) -> int:
    """ceil(0.9·n) in exact integer math (0.9*10 float-rounds ABOVE 9)."""
    return (9 * n + 9) // 10


def _toss_catch_pose(obs_landing_xy_mm, obs_v_arrival_mms, cup_world_z_mm,
                     platform_z_mm):
    """The build_catch target pose from a landing + arrival velocity.

    Cloned from ``reload_gate.ReloadGate._catch_pose`` (reload_gate.py, ``_catch_pose``).
    The ONE deliberate difference: pose z = the **nominated** platform z
    (the toss sweeps z 170±30), not the fixed ``Z_ACTIVE_MM``. Pinned by
    ``test_corner_pose_announcement_crosses_into_accepted_build_catch``.
    """
    rx, ry = tilt_to_receive(obs_v_arrival_mms)
    shift = cup_lateral_shift_mm(rx, ry, cup_z_mm=cup_world_z_mm)
    cx = float(obs_landing_xy_mm[0]) - float(shift[0])
    cy = float(obs_landing_xy_mm[1]) - float(shift[1])
    return np.array([cx, cy, float(platform_z_mm), rx, ry, 0.0]), (rx, ry)


@dataclasses.dataclass
class TossGateConfig:
    trials_per_point: int = 10
    seed: int = 0
    grid: str = 'factored'            # 'factored' | 'full'
    points: list | None = None        # explicit [(x, y, z, T), ...] override
    prep_gap_s: float = 0.8           # catch-plan install → scheduled release
    settle_hold_s: float = float(hw.JB_TRAJ_CATCH_SETTLE_HOLD_S)   # 0.5
    preposition_duration_s: float = 1.5
    preposition_settle_s: float = 0.3
    preposition_tol_mm: float = 2.0
    # Catch-capable session limits (reload-gate parity).
    leg_vel_mmps: float = 250.0
    leg_acc_mmps2: float = 3000.0
    leg_jerk_mmps3: float = 150000.0
    # PLACEHOLDER until Phase-5 T0 measures the real release scatter
    # (plan § Sim gate): per-component Gaussian velocity σ = frac·|v_launch|.
    release_vel_noise_frac: float = 0.01
    tracking_noise_mm: float = 0.5
    # Robustness sweep (arm-time error / event-vel error) — 0 = nominal.
    arm_time_err_s: float = 0.0
    event_vel_err_frac: float = 0.0
    # Inherited from the reload gate's empirically-swept coupled anchor;
    # re-sweep at bring-up (toss arrival speeds run to 5.4 m/s vs reload's 4.0).
    capture_offset_s: float = -0.010
    # Observation feed (mocap-rate analogue on the real in-plant ball).
    obs_start_s: float = 0.05
    obs_period_s: float = 0.010
    obs_min_samples: int = 6
    # Non-gating contact-physics diagnostic column.
    contact_diag: bool = True
    diag_release: str = 'detach'      # 'detach' | 'kinematic'
    diag_trials: int = 4              # per diag flight point
    report_path: str | None = None


@dataclasses.dataclass
class _TossSetup:
    """Per-trial pure synthesis result (no plant): the production release state,
    the announced-arrival-armed hand speed, and the pre-tilt catch pose."""
    event_vel_mps: float = 0.0
    rs: "toss_release.ReleaseState | None" = None
    armed_v: float = 0.0
    catch_pose: np.ndarray | None = None
    rx: float = 0.0
    ry: float = 0.0
    cup_world_z_sim: float = 0.0
    ann_landing_vel_mms: np.ndarray | None = None
    reject_code: str | None = None


@dataclasses.dataclass
class TossTrialResult:
    idx: int
    point_id: str
    pose_mm: tuple                 # nominated (x, y, z)
    flight_time_s: float
    event_vel_mps: float
    seed: int
    accepted: bool                 # build_move + build_catch both feasible
    reject_code: str | None
    preposition_err_mm: float
    caught: bool
    held_at_end: bool
    vel_match_frac: float
    hold_travel_mm: float
    hold_tilt_deg: float
    separation_ms: float
    capture_rel_ms: float          # capture − predicted arrival (ms)
    announced_landing_err_ms: float  # capture − announced landing_time (ms)
    landing_err_mm: float          # |ball xy at capture − nominated xy|
    capture_dist_mm: float         # |ball − hand_opening site| at capture
    release_pos_err_xy_mm: float   # |sim release xy − announced release xy|
    release_frame_z_delta_mm: float  # sim release z − announced release z (R1)
    hand_timing_margin_ms: float   # catch downstroke start − throw end
    hand_arm_infeasible: bool
    catch_armed: bool              # the arm-and-forget HandCatchSequence armed
    pump_rejects: int              # pre-position pump + catch pump
    pump_frames_emitted: int       # frames offered to BOTH pumps
    pump_frames_accepted: int      # frames a pump returned a Setpoint for
    peak_leg_vel_mmps: float       # max over BOTH plans (move + catch)
    peak_leg_acc_mmps2: float
    peak_leg_jerk_mmps3: float
    hardware_marginal: bool        # flight < 0.7 s (firmware windup band)
    clean: bool
    core_clean: bool

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        d['pose_mm'] = list(self.pose_mm)
        return d


class TossGate:
    """Owns one MuJoCo plant + emitter for the whole gate run (the non-gating
    contact-physics diagnostic plant is constructed lazily)."""

    def __init__(self, cfg: TossGateConfig):
        self.cfg = cfg
        self.geom = StewartGeometry()
        self.plant = MuJoCoPlant(geom=self.geom)      # contact_carry=False (gating)
        self.viewer = None  # optional ViewerHook, attached by main() on --viewer
        self.emitter = KnotEmitter(self.geom)
        self.limits = TrajectoryLimits.from_config(hw).with_session_limits(
            leg_vel_mmps=cfg.leg_vel_mmps, leg_acc_mmps2=cfg.leg_acc_mmps2,
            leg_jerk_mmps3=cfg.leg_jerk_mmps3)
        self.noise_cfg = NoiseConfig(
            bb_throw_noise_frac=cfg.release_vel_noise_frac,
            tracking_noise_mm=cfg.tracking_noise_mm)
        self._site_id = mujoco.mj_name2id(
            self.plant.model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')
        self._diag_plant = None       # lazy MuJoCoPlant(contact_carry=True)

    # ---- pure per-trial synthesis (no plant) ------------------------------
    def _prepare_toss(self, point) -> _TossSetup:
        """Production release state + announcement-derived catch inputs for one
        ``(x, y, z_nom, T)`` cell — everything up to (not including) the planner
        calls. All ``toss_release`` calls run on DEFAULTS (hardware-frame; R1)."""
        cfg = self.cfg
        x, y, z, T = point
        try:
            rs = toss_release.compute_release_state((x, y, z), T)
        except ValueError:
            return _TossSetup(reject_code='RELEASE_STATE')
        if not toss_release.validate_event_vel(rs.event_vel_mps):
            return _TossSetup(event_vel_mps=rs.event_vel_mps,
                              reject_code='EVENT_VEL_BAND')
        # Announced landing velocity (the announcement field the hand is armed
        # from — production arms from the announcement, not from tracking).
        ann_landing_vel = bal.arrival_velocity(rs.launch_vel_mms, T)
        armed_v = (float(np.linalg.norm(ann_landing_vel)) / 1000.0) \
            * (1.0 + cfg.event_vel_err_frac)
        # Probed linear cup relation (gate_common.py, ``CUP_Z_BASE_MM`` probe comment) + the z_nom term the
        # toss adds: the platform holds the NOMINATED z, not Z_ACTIVE.
        cup_world_z_sim = (CUP_Z_BASE_MM
                           + HandCatchTrajectory(armed_v).sample(0.0)
                           + (z - Z_ACTIVE_MM))
        # Announced landing xy == nominated xy (stow_to_global adds only z).
        catch_pose, (rx, ry) = _toss_catch_pose(
            rs.catch_point_global_mm[:2], ann_landing_vel, cup_world_z_sim, z)
        return _TossSetup(
            event_vel_mps=rs.event_vel_mps, rs=rs, armed_v=armed_v,
            catch_pose=catch_pose, rx=rx, ry=ry,
            cup_world_z_sim=cup_world_z_sim,
            ann_landing_vel_mms=ann_landing_vel)

    # ---- one gating trial -------------------------------------------------
    def run_trial(self, idx: int, point, seed: int) -> TossTrialResult:
        cfg = self.cfg
        x, y, z, T = point
        s = self._prepare_toss(point)
        if s.reject_code is not None:
            return self._reject(idx, point, s.event_vel_mps, seed, s.reject_code)
        pose_nom = np.array([x, y, z, 0.0, 0.0, 0.0])
        # PLAN via the production constructors (the single gate). Rejects are loud.
        try:
            plan_move, report_move = planner.build_move(
                (NEUTRAL_POSE, np.zeros(6), np.zeros(6)), pose_nom,
                cfg.preposition_duration_s, self.limits, self.geom)
        except TrajectoryInfeasible as e:
            return self._reject(idx, point, s.event_vel_mps, seed, e.code)
        # The pre-tilt catch plan: state0 = the nominated (commanded) pose —
        # production plans from commanded state. Lead = prep_gap + T so the
        # plan's arrival instant equals the ANNOUNCED landing time.
        try:
            plan_catch, report_catch = planner.build_catch(
                (pose_nom, np.zeros(6), np.zeros(6)), s.catch_pose,
                cfg.prep_gap_s + T, self.limits, self.geom,
                settle_hold_s=cfg.settle_hold_s)
        except TrajectoryInfeasible as e:
            return self._reject(idx, point, s.event_vel_mps, seed, e.code)
        noise = JuggleNoise(self.noise_cfg, seed=seed)
        return self._simulate_toss(idx, point, s, plan_move, report_move,
                                   plan_catch, report_catch, noise, seed)

    def _simulate_toss(self, idx, point, s, plan_move, report_move,
                       plan_catch, report_catch, noise, seed) -> TossTrialResult:
        """One trial start-to-verdict on the gating (kinematic-hold) plant."""
        cfg = self.cfg
        x, y, z, T = point
        plant, emitter = self.plant, self.emitter
        model_dt = plant.timestep
        substeps = max(1, int(round(KNOT_DT_S / model_dt)))
        pose_nom_xyz = np.array([x, y, z])

        # 1. Neutral settle; hand at the throw-stroke start / catch-rest bottom
        # (20 mm — firmware ground truth: a completed catch leaves the hand at
        # stroke bottom; NOT reload's ~335 mm catch prime).
        plant.reset(NEUTRAL_POSE)
        plant.command(plant.pose_to_extensions(NEUTRAL_POSE))
        plant.command_hand(STROKE_MARGIN_MM)
        for _ in range(60):
            plant.step(KNOT_DT_S)
            if self.viewer is not None:
                self.viewer.sync()

        # 2. Seat the ball (sim analogue of "ball sourced by a prior Reload").
        plant.ball_manager.ball(0).spawn_in_hand()

        # 3. Pre-position through the production plan (pump A). Every knot of
        # the pre-position plan counts toward the pump-accepted invariant.
        pump_a = SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV,
                              max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV)
        t_move = plant.data.time
        move_until = t_move + cfg.preposition_duration_s + cfg.preposition_settle_s
        n_frames = 0
        # Non-vacuous pump invariant (accepted == emitted, pinned by the
        # smoke): count the frames a pump actually returned a Setpoint for,
        # not just the reject counter.
        pump_frames_emitted = 0
        pump_frames_accepted = 0
        pump_a_last_u0 = None          # last ACCEPTED pre-position knot (rev)
        while plant.data.time < move_until:
            tau = plant.data.time - t_move
            frame = emitter.frame(plan_move, tau, n_frames)
            sp_a, _ = pump_a.build(frame, t_origin_us=int(n_frames * 25000))
            pump_frames_emitted += 1
            if sp_a is not None:
                pump_frames_accepted += 1
                pump_a_last_u0 = sp_a.u0
            n_frames += 1
            plant.command(np.asarray(frame['ext_mm']))
            for _ in range(substeps):
                if plant.data.time >= move_until:
                    break
                plant.step(model_dt)
                if self.viewer is not None:
                    self.viewer.sync()
        st = plant.get_state()
        preposition_err_mm = float(np.linalg.norm(
            st.platform_pos_mm - pose_nom_xyz))

        # 5. Pre-tilt catch install + announcement (production ordering:
        # announce/arm BEFORE the throw). The announcement is stamped with the
        # SCHEDULED release time — the gate's absolute-sim-time analogue of the
        # coordinator's dispatch_time + event_delay.
        t_install = plant.data.time
        t_release = t_install + cfg.prep_gap_s
        ann = toss_release.build_announcement_fields(s.rs, throw_time_s=t_release)
        t_land_ann = float(ann['landing_time_s'])

        # 6. Arm the throw (hand). Prelude is zero-length (hand already at the
        # throw start); infeasible should be unreachable at prep_gap 0.8 s.
        res = HandThrowSequence.try_create(
            s.rs.event_vel_mps, t_release,
            current_pos_mm=STROKE_MARGIN_MM, current_time=t_install)
        if not res.feasible:
            return self._reject(idx, point, s.event_vel_mps, seed,
                                'HAND_THROW_BUDGET')
        throw_seq = res.sequence

        # 7. Main loop (pump B): stream the catch plan; chain the hand
        # throw→catch profiles; kinematic release; observe; arm-once; capture.
        pump_b = SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV,
                              max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV)
        est = BallisticEstimator(bal.G_VEC_MMS2)
        catch_seq = None
        released = False
        caught = False
        v_match = float('nan')
        separation_ms = 0.0
        contact_lost_since = None
        capture_rel_ms = float('nan')
        announced_landing_err_ms = float('nan')
        landing_err_mm = float('nan')
        capture_dist_mm = float('nan')
        release_pos_err_xy_mm = float('nan')
        release_frame_z_delta_mm = float('nan')
        hand_timing_margin_ms = float('nan')
        hand_arm_infeasible = False
        catch_armed = False
        pump_b_first_u0 = None
        t_arrival_pred = None
        next_obs_t = None
        hand_cmd_mm = float(STROKE_MARGIN_MM)
        hold_start = t_land_ann + 0.15
        hold_end = t_land_ann + cfg.settle_hold_s
        hold_pos = []
        hold_rot = []
        run_until = t_land_ann + cfg.settle_hold_s + 0.05
        n_frames = 0

        while plant.data.time < run_until:
            tau = plant.data.time - t_install
            frame = emitter.frame(plan_catch, tau, n_frames)
            sp_b, _ = pump_b.build(frame, t_origin_us=int(n_frames * 25000))
            pump_frames_emitted += 1
            if sp_b is not None:
                pump_frames_accepted += 1
                if pump_b_first_u0 is None:
                    pump_b_first_u0 = sp_b.u0
                    # Pre-position→catch pump HANDOFF continuity. Production
                    # runs ONE continuous pump across plans, so a jump between
                    # the last pre-position knot and the first catch knot
                    # would hit its per-step gate; the harness's two pumps
                    # each waive their first frame, so the crossing must be
                    # checked explicitly or a discontinuous handoff passes
                    # silently.
                    if pump_a_last_u0 is not None:
                        handoff_step = max(
                            abs(b - a) for a, b in
                            zip(pump_a_last_u0, pump_b_first_u0))
                        if handoff_step > float(hw.JB_OP_MAX_POSITION_STEP_REV):
                            print(f"[toss_gate] PUMP HANDOFF DISCONTINUITY "
                                  f"trial {idx} ({_point_id(point)}): first "
                                  f"catch knot jumps {handoff_step:.4f} rev "
                                  f"from the last pre-position knot (limit "
                                  f"{float(hw.JB_OP_MAX_POSITION_STEP_REV):.3f}"
                                  f" rev/frame) — failing the trial")
                            return self._reject(idx, point, s.event_vel_mps,
                                                seed, 'PUMP_HANDOFF_STEP')
            n_frames += 1
            plant.command(np.asarray(frame['ext_mm']))
            for _ in range(substeps):
                if plant.data.time >= run_until:
                    break
                t_now = plant.data.time
                # Hand at the high (substep) rate: throw profile while active,
                # then the armed arm-and-forget catch profile. The chain is
                # position-continuous (throw end 335 mm = catch start default).
                hp = throw_seq.sample(t_now)
                if hp is None and catch_seq is not None:
                    hp = catch_seq.sample(t_now)
                if hp is not None:
                    plant.command_hand(hp)
                    hand_cmd_mm = float(hp)
                # Kinematic release at the scheduled instant (mode-correct
                # ejector for contact_carry=False — see the module docstring).
                if not released and t_now >= t_release:
                    bs = plant.get_ball_state()
                    release_pos_sim = bs.position_mm.copy()
                    _, v_noisy = noise.perturb_throw(
                        s.rs.release_pos_global_mm, s.rs.launch_vel_mms,
                        np.zeros(3))
                    plant.release_ball(v_noisy)
                    released = True
                    release_pos_err_xy_mm = float(np.hypot(
                        release_pos_sim[0] - s.rs.release_pos_global_mm[0],
                        release_pos_sim[1] - s.rs.release_pos_global_mm[1]))
                    release_frame_z_delta_mm = float(
                        release_pos_sim[2] - s.rs.release_pos_global_mm[2])
                    next_obs_t = t_now + cfg.obs_start_s
                plant.step(model_dt)
                if self.viewer is not None:
                    self.viewer.sync()
                t_now = plant.data.time
                # Observe the REAL in-plant ball (mocap analogue) + one-shot arm.
                if released and next_obs_t is not None and t_now >= next_obs_t:
                    est.add(t_now,
                            noise.observe(plant.get_ball_state().position_mm))
                    next_obs_t += cfg.obs_period_s
                    if catch_seq is None and est.n >= cfg.obs_min_samples:
                        p_est, v_est = est.estimate()
                        try:
                            _, _, t_rem = bal.arrival_state_at_z(
                                p_est, v_est, s.cup_world_z_sim,
                                descending=True)
                        except ValueError:
                            pass    # degenerate early fit; retry next sample
                        else:
                            t_arrival_pred = t_now + t_rem
                            hand_arrival = (t_arrival_pred
                                            + cfg.capture_offset_s
                                            + cfg.arm_time_err_s)
                            chk = HandCatchSequence.try_create(
                                s.armed_v, hand_arrival, hand_cmd_mm,
                                current_time=max(t_now, throw_seq.end_time))
                            hand_arm_infeasible = not chk.feasible
                            catch_seq = HandCatchSequence(
                                s.armed_v, hand_arrival, hand_cmd_mm)
                            catch_armed = True
                            hand_timing_margin_ms = (
                                (hand_arrival
                                 + catch_seq.catch_trajectory.start_time)
                                - throw_seq.end_time) * 1000.0
                            run_until = max(catch_seq.end_time,
                                            t_land_ann + cfg.settle_hold_s
                                            ) + 0.05
                # Capture + metrics (ball position is still the raw contact
                # position on the capture substep — the hold teleport applies
                # from the NEXT substep, so capture_dist_mm is honest).
                if released and not caught and plant.check_and_capture():
                    caught = True
                    bs = plant.get_ball_state()
                    v_ball_vec = bs.velocity_mms
                    v_hand_vec = plant.ball_manager.ball(0) \
                        ._hand_site_velocity() * 1000.0
                    v_match = float(np.linalg.norm(v_ball_vec - v_hand_vec)
                                    / max(np.linalg.norm(v_ball_vec), 1e-6))
                    site_mm = plant.data.site_xpos[self._site_id] * 1000.0
                    capture_dist_mm = float(np.linalg.norm(
                        bs.position_mm - site_mm))
                    landing_err_mm = float(np.hypot(
                        bs.position_mm[0] - x, bs.position_mm[1] - y))
                    if t_arrival_pred is not None:
                        capture_rel_ms = (t_now - t_arrival_pred) * 1000.0
                    announced_landing_err_ms = (t_now - t_land_ann) * 1000.0
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

        held_at_end = bool(plant.get_ball_state().held) if plant.has_ball \
            else False
        hold_travel = travel_mm(hold_pos)
        hold_tilt = tilt_change_deg(hold_rot)
        pump_rejects = int(pump_a.frames_rejected + pump_b.frames_rejected)

        core_clean = bool(
            caught and held_at_end
            # Gate honesty (silent-dead-arm guard): the catch only counts when
            # the arm-and-forget HandCatchSequence actually ARMED from the
            # tracked flight, and armed feasibly — a ball dropping into a
            # statically-parked cup must not gate-pass.
            and catch_armed and not hand_arm_infeasible
            and hold_travel < HOLD_TRAVEL_MM and hold_tilt < HOLD_TILT_DEG
            and separation_ms <= SEPARATION_MS
            and pump_rejects == 0
            and preposition_err_mm <= cfg.preposition_tol_mm)
        clean = bool(core_clean and (not np.isnan(v_match))
                     and v_match <= VEL_MATCH_FRAC)

        return TossTrialResult(
            idx=idx, point_id=_point_id(point), pose_mm=(x, y, z),
            flight_time_s=T, event_vel_mps=float(s.event_vel_mps), seed=seed,
            accepted=True, reject_code=None,
            preposition_err_mm=preposition_err_mm,
            caught=caught, held_at_end=held_at_end,
            vel_match_frac=float(v_match),
            hold_travel_mm=float(hold_travel), hold_tilt_deg=float(hold_tilt),
            separation_ms=float(separation_ms),
            capture_rel_ms=float(capture_rel_ms),
            announced_landing_err_ms=float(announced_landing_err_ms),
            landing_err_mm=float(landing_err_mm),
            capture_dist_mm=float(capture_dist_mm),
            release_pos_err_xy_mm=float(release_pos_err_xy_mm),
            release_frame_z_delta_mm=float(release_frame_z_delta_mm),
            hand_timing_margin_ms=float(hand_timing_margin_ms),
            hand_arm_infeasible=bool(hand_arm_infeasible),
            catch_armed=bool(catch_armed),
            pump_rejects=pump_rejects,
            pump_frames_emitted=int(pump_frames_emitted),
            pump_frames_accepted=int(pump_frames_accepted),
            peak_leg_vel_mmps=float(max(report_move.peak_leg_vel_mmps,
                                        report_catch.peak_leg_vel_mmps)),
            peak_leg_acc_mmps2=float(max(report_move.peak_leg_acc_mmps2,
                                         report_catch.peak_leg_acc_mmps2)),
            peak_leg_jerk_mmps3=float(max(report_move.peak_leg_jerk_mmps3,
                                          report_catch.peak_leg_jerk_mmps3)),
            hardware_marginal=bool(T < _HW_MARGINAL_FLIGHT_S),
            clean=clean, core_clean=core_clean)

    @staticmethod
    def _reject(idx, point, event_vel_mps, seed, code) -> TossTrialResult:
        x, y, z, T = point
        return TossTrialResult(
            idx=idx, point_id=_point_id(point), pose_mm=(x, y, z),
            flight_time_s=T, event_vel_mps=float(event_vel_mps), seed=seed,
            accepted=False, reject_code=code,
            preposition_err_mm=float('nan'), caught=False, held_at_end=False,
            vel_match_frac=float('nan'), hold_travel_mm=0.0,
            hold_tilt_deg=0.0, separation_ms=0.0,
            capture_rel_ms=float('nan'),
            announced_landing_err_ms=float('nan'),
            landing_err_mm=float('nan'), capture_dist_mm=float('nan'),
            release_pos_err_xy_mm=float('nan'),
            release_frame_z_delta_mm=float('nan'),
            hand_timing_margin_ms=float('nan'), hand_arm_infeasible=False,
            catch_armed=False, pump_rejects=0,
            pump_frames_emitted=0, pump_frames_accepted=0,
            peak_leg_vel_mmps=0.0, peak_leg_acc_mmps2=0.0,
            peak_leg_jerk_mmps3=0.0,
            hardware_marginal=bool(T < _HW_MARGINAL_FLIGHT_S),
            clean=False, core_clean=False)

    # ---- the full gate ----------------------------------------------------
    def run(self) -> dict:
        cfg = self.cfg
        points = list(cfg.points) if cfg.points is not None \
            else default_grid(cfg.grid)
        results = []
        gi = 0
        for pt in points:
            for _ in range(cfg.trials_per_point):
                results.append(self.run_trial(gi, pt, seed=cfg.seed + gi))
                gi += 1
        diag = self._run_contact_diag() if cfg.contact_diag else None
        return self._summarise(points, results, diag)

    # ---- contact-physics diagnostic column (non-gating, § 6) --------------
    def _run_contact_diag(self, flights=None) -> dict:
        """The plan's contact-physics variant: centre pose × the sweep flights
        on a SECOND ``contact_carry=True`` plant, seat-based verdicts. Keeps
        measuring what the contact model predicts (the Rung-2a/2b knife-edge)
        so T0's measured scatter can be compared against both columns.
        **No pass/fail; never feeds ``passed``.**"""
        cfg = self.cfg
        if self._diag_plant is None:
            self._diag_plant = MuJoCoPlant(geom=self.geom, contact_carry=True)
        if flights is None:
            flights = _FLIGHTS_S
        rows = []
        i = 0
        for T in flights:
            for _ in range(cfg.diag_trials):
                rows.append(self._diag_trial(
                    (0.0, 0.0, Z_ACTIVE_MM, T), cfg.seed + 10000 + i))
                i += 1
        errs = [r['landing_err_mm'] for r in rows
                if r['caught'] and not math.isnan(r['landing_err_mm'])]
        return {
            'mode': cfg.diag_release,
            'trials': len(rows),
            'caught': sum(1 for r in rows if r['caught']),
            'pump_rejects': int(sum(r['pump_rejects'] for r in rows)),
            'landing_err_mm': {
                'worst': float(max(errs)) if errs else None,
                'mean': float(np.mean(errs)) if errs else None,
            },
            'rows': rows,
        }

    def _diag_trial(self, point, seed: int) -> dict:
        """One contact-carried trial (diagnostic column). Three substitutions
        vs the gating flow: (1) the release is ``begin_physics_throw`` under a
        stiff throw-stroke contact ('detach', velocity EMERGES) or the plan's
        named ``ballistic_release`` verbatim ('kinematic'); (2) the platform
        command is refreshed per SUBSTEP from the plan (the 25 ms knot
        staircase sloshes a contact-carried ball out of the cup —
        mujoco_plant.py:296-305; knots still stream through the pump for the
        invariant count); (3) the verdict is the seat metric."""
        cfg = self.cfg
        x, y, z, T = point
        plant = self._diag_plant
        emitter = self.emitter
        model_dt = plant.timestep
        substeps = max(1, int(round(KNOT_DT_S / model_dt)))
        s = self._prepare_toss(point)
        row = {'point_id': _point_id(point), 'flight_time_s': T, 'seed': seed,
               'accepted': False, 'reject_code': s.reject_code,
               'caught': False, 'held': False,
               'landing_err_mm': float('nan'), 'separated': False,
               'seat_offset_mm': float('nan'),
               'pump_rejects': 0}          # BOTH diag pumps (move + catch)
        if s.reject_code is not None:
            return row
        pose_nom = np.array([x, y, z, 0.0, 0.0, 0.0])
        try:
            plan_move, _ = planner.build_move(
                (NEUTRAL_POSE, np.zeros(6), np.zeros(6)), pose_nom,
                cfg.preposition_duration_s, self.limits, self.geom)
            plan_catch, _ = planner.build_catch(
                (pose_nom, np.zeros(6), np.zeros(6)), s.catch_pose,
                cfg.prep_gap_s + T, self.limits, self.geom,
                settle_hold_s=cfg.settle_hold_s)
        except TrajectoryInfeasible as e:
            row['reject_code'] = e.code
            return row
        row['accepted'] = True
        noise = JuggleNoise(self.noise_cfg, seed=seed)

        def _stream(plan, t0, until, pump, n0):
            """Knots through the pump; platform refreshed per substep from the
            plan (diag-only carry-fidelity aid)."""
            n = n0
            while plant.data.time < until:
                tau = plant.data.time - t0
                frame = emitter.frame(plan, tau, n)
                pump.build(frame, t_origin_us=int(n * 25000))
                n += 1
                plant.command(np.asarray(frame['ext_mm']))
                for _ in range(substeps):
                    if plant.data.time >= until:
                        break
                    pose_sub, _, _ = plan.state_at(plant.data.time - t0)
                    plant.command(plant.pose_to_extensions(pose_sub))
                    plant.step(model_dt)
            return n

        # Settle + seat (contact-carried: the ball physically rests in the cup).
        plant.reset(NEUTRAL_POSE)
        plant.set_contact_stiffness(False)
        plant.command(plant.pose_to_extensions(NEUTRAL_POSE))
        plant.command_hand(STROKE_MARGIN_MM)
        for _ in range(60):
            plant.step(KNOT_DT_S)
        plant.ball_manager.ball(0).spawn_in_hand()
        pump = SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV,
                            max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV)
        t_move = plant.data.time
        _stream(plan_move, t_move,
                t_move + cfg.preposition_duration_s + cfg.preposition_settle_s,
                pump, 0)

        t_install = plant.data.time
        t_release = t_install + cfg.prep_gap_s
        ann = toss_release.build_announcement_fields(s.rs, throw_time_s=t_release)
        t_land_ann = float(ann['landing_time_s'])
        res = HandThrowSequence.try_create(
            s.rs.event_vel_mps, t_release,
            current_pos_mm=STROKE_MARGIN_MM, current_time=t_install)
        if not res.feasible:
            row['reject_code'] = 'HAND_THROW_BUDGET'
            row['accepted'] = False
            row['pump_rejects'] = int(pump.frames_rejected)
            return row
        throw_seq = res.sequence
        t_stroke_start = t_release + throw_seq.throw_trajectory.start_time

        est = BallisticEstimator(bal.G_VEC_MMS2)
        catch_seq = None
        released = False
        stiffened = False
        caught = False
        contact_lost_since = None
        next_obs_t = None
        hand_cmd_mm = float(STROKE_MARGIN_MM)
        run_until = t_land_ann + cfg.settle_hold_s + 0.05
        pump2 = SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV,
                             max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV)
        n = 0
        while plant.data.time < run_until:
            tau = plant.data.time - t_install
            frame = emitter.frame(plan_catch, tau, n)
            pump2.build(frame, t_origin_us=int(n * 25000))
            n += 1
            plant.command(np.asarray(frame['ext_mm']))
            for _ in range(substeps):
                if plant.data.time >= run_until:
                    break
                t_now = plant.data.time
                pose_sub, _, _ = plan_catch.state_at(t_now - t_install)
                plant.command(plant.pose_to_extensions(pose_sub))
                hp = throw_seq.sample(t_now)
                if hp is None and catch_seq is not None:
                    hp = catch_seq.sample(t_now)
                if hp is not None:
                    plant.command_hand(hp)
                    hand_cmd_mm = float(hp)
                if cfg.diag_release == 'detach':
                    # Stiffen at the compressive up-stroke start; the take-off
                    # velocity EMERGES from the stroke at begin_physics_throw.
                    if not stiffened and t_now >= t_stroke_start:
                        plant.set_contact_stiffness(True)
                        stiffened = True
                    if not released and t_now >= t_release:
                        plant.begin_physics_throw()
                        released = True
                        next_obs_t = t_now + cfg.obs_start_s
                    if stiffened and t_now >= t_release + 0.1:
                        plant.set_contact_stiffness(False)
                        stiffened = False
                else:
                    # The plan-named function, verbatim, in its contract mode.
                    if not released and t_now >= t_release:
                        _, v_noisy = noise.perturb_throw(
                            s.rs.release_pos_global_mm, s.rs.launch_vel_mms,
                            np.zeros(3))
                        plant.ballistic_release(v_noisy)
                        released = True
                        next_obs_t = t_now + cfg.obs_start_s
                plant.step(model_dt)
                t_now = plant.data.time
                if released and next_obs_t is not None and t_now >= next_obs_t:
                    est.add(t_now,
                            noise.observe(plant.get_ball_state().position_mm))
                    next_obs_t += cfg.obs_period_s
                    if catch_seq is None and est.n >= cfg.obs_min_samples:
                        p_est, v_est = est.estimate()
                        try:
                            _, _, t_rem = bal.arrival_state_at_z(
                                p_est, v_est, s.cup_world_z_sim,
                                descending=True)
                        except ValueError:
                            pass
                        else:
                            hand_arrival = (t_now + t_rem
                                            + cfg.capture_offset_s
                                            + cfg.arm_time_err_s)
                            catch_seq = HandCatchSequence(
                                s.armed_v, hand_arrival, hand_cmd_mm)
                            run_until = max(catch_seq.end_time,
                                            t_land_ann + cfg.settle_hold_s
                                            ) + 0.05
                if released and not caught and plant.check_and_capture():
                    caught = True
                    bs = plant.get_ball_state()
                    row['landing_err_mm'] = float(np.hypot(
                        bs.position_mm[0] - x, bs.position_mm[1] - y))
                if caught:
                    if not plant.get_ball_state().held:
                        if contact_lost_since is None:
                            contact_lost_since = plant.data.time
                        if (plant.data.time - contact_lost_since) * 1000.0 \
                                > SEPARATION_MS:
                            row['separated'] = True
                    else:
                        contact_lost_since = None
        row['caught'] = bool(caught)
        bs = plant.get_ball_state()
        row['held'] = bool(bs.held)
        site_mm = plant.data.site_xpos[self._site_id] * 1000.0
        row['seat_offset_mm'] = float(np.linalg.norm(bs.position_mm - site_mm))
        row['pump_rejects'] = int(pump.frames_rejected + pump2.frames_rejected)
        return row

    # ---- summary ----------------------------------------------------------
    def _summarise(self, points, results, diag) -> dict:
        cfg = self.cfg
        n = len(results)
        accepted = [r for r in results if r.accepted]
        clean = [r for r in results if r.clean]
        core_clean = [r for r in results if r.core_clean]
        caught = [r for r in results if r.caught]
        feas_violations = sum(1 for r in accepted if r.reject_code is not None)
        pump_rejects = sum(r.pump_rejects for r in results)

        by_point = {}
        for r in results:
            by_point.setdefault(r.point_id, []).append(r)
        point_rows = []
        for pt in points:
            pid = _point_id(pt)
            x, y, z, T = pt
            rs_pt = by_point.get(pid, [])
            npt = len(rs_pt)
            # event_vel from the RECORDED trial rows (populated even on
            # EVENT_VEL_BAND rejects) — never recomputed here: a point whose
            # compute_release_state raised (RELEASE_STATE reject) would crash
            # the summary. Rows lacking it ⇒ ev = NaN, binding flags False.
            ev = float('nan')
            for r in rs_pt:
                if math.isfinite(r.event_vel_mps) and r.event_vel_mps > 0.0:
                    ev = float(r.event_vel_mps)
                    break
            thr = _pass_threshold(npt)
            n_core = sum(1 for r in rs_pt if r.core_clean)
            point_rows.append({
                'point_id': pid,
                'pose_mm': [x, y, z],
                'flight_time_s': T,
                'event_vel_mps': float(ev),
                # The plan's binding band: arrival speed 2-3 m/s.
                'binding_2_3_mps': bool(
                    math.isfinite(ev)
                    and _BINDING_SPEED_MIN_MPS <= ev <= _BINDING_SPEED_MPS),
                # The orchestrator-amendment binding band: T=0.80 s at z=170.
                # Membership is purely geometric (grid coordinates) — an
                # isfinite(ev) term here would let a fully-rejected point
                # silently leave the band and pass it vacuously; a rejected
                # binding point must instead FAIL pass_9_of_10 (core_clean 0).
                'binding_T080_z170': bool(
                    abs(T - _BINDING_T080_S) < 1e-9
                    and abs(z - Z_ACTIVE_MM) < 1e-9),
                'hardware_marginal_flight': bool(T < _HW_MARGINAL_FLIGHT_S),
                'n': npt,
                'caught': sum(1 for r in rs_pt if r.caught),
                'clean': sum(1 for r in rs_pt if r.clean),
                'core_clean': n_core,
                'pass_threshold': thr,
                'pass_9_of_10': bool(npt > 0 and n_core >= thr),
            })

        band_23 = [p for p in point_rows if p['binding_2_3_mps']]
        band_t080 = [p for p in point_rows if p['binding_T080_z170']]
        passed_23 = all(p['pass_9_of_10'] for p in band_23)
        passed_t080 = all(p['pass_9_of_10'] for p in band_t080)
        passed_full_band = bool(all(p['pass_9_of_10'] for p in point_rows))
        pump_frames_emitted = sum(r.pump_frames_emitted for r in results)
        pump_frames_accepted = sum(r.pump_frames_accepted for r in results)
        invariants_ok = (feas_violations == 0 and pump_rejects == 0
                         and pump_frames_accepted == pump_frames_emitted)
        no_binding_points = (not band_23) and (not band_t080)
        if no_binding_points:
            # Explicit-points/debug run intersecting NEITHER binding band:
            # two empty all()'s would vacuously PASS, so gate on the full
            # band instead. main() prints the loud NO BINDING POINTS banner.
            # A zero-trial run (empty cfg.points) can never PASS.
            passed = bool(results and passed_full_band and invariants_ok)
        else:
            passed = bool(passed_23 and passed_t080 and invariants_ok)

        req_vel = max((r.peak_leg_vel_mmps for r in accepted), default=0.0) * 1.15
        req_acc = max((r.peak_leg_acc_mmps2 for r in accepted), default=0.0) * 1.15
        req_jerk = max((r.peak_leg_jerk_mmps3 for r in accepted), default=0.0) * 1.15
        vmatch = [r.vel_match_frac for r in caught
                  if not np.isnan(r.vel_match_frac)]
        prepos = [r.preposition_err_mm for r in accepted
                  if not np.isnan(r.preposition_err_mm)]
        landerr = [r.landing_err_mm for r in caught
                   if not np.isnan(r.landing_err_mm)]
        annerr = [r.announced_landing_err_ms for r in caught
                  if not np.isnan(r.announced_landing_err_ms)]

        return {
            'gate': 'toss',
            'passed': passed,
            # The two binding bands, reported separately (per-band criterion
            # only; `passed` conjoins them with the invariants below).
            'passed_2_3_mps_band': bool(passed_23),
            'passed_T080_z170_band': bool(passed_t080),
            'passed_full_band': passed_full_band,      # advisory, all points
            'no_binding_points': bool(no_binding_points),
            'binding_note': (
                "passed = (every 2-3 m/s-band point core_clean >= ceil(0.9 n)) "
                "AND (every T=0.80 s @ z=170 point core_clean >= ceil(0.9 n)) "
                "AND zero feasibility violations AND zero pump rejects with "
                "every emitted pump frame accepted. The T=0.80 band is the "
                "2026-07-25 orchestrator amendment: the 2-3 m/s band is "
                "hardware-marginal, so the hardware-relevant band gates too. "
                "When the run's grid intersects NEITHER band "
                "(no_binding_points), passed falls back to passed_full_band "
                "AND the invariants — never a vacuous empty-band PASS."),
            'core_passed_note': ('criterion evaluated on core_clean; vel_match '
                                 'deferred'),
            'deferred_criteria': ['vel_match_frac'],
            'deferred_note': _DEFERRED_NOTE,
            'trials': n,
            'clean': len(clean),
            'core_clean': len(core_clean),
            'caught': len(caught),
            'accepted': len(accepted),
            'points': point_rows,
            'feasibility_violations_in_accepted': feas_violations,
            'total_pump_rejects': pump_rejects,
            'total_pump_frames_emitted': int(pump_frames_emitted),
            'total_pump_frames_accepted': int(pump_frames_accepted),
            'worst_vel_match_frac': float(max(vmatch)) if vmatch else None,
            'mean_vel_match_frac': float(np.mean(vmatch)) if vmatch else None,
            'worst_hold_travel_mm': float(max(
                (r.hold_travel_mm for r in caught), default=0.0)),
            'worst_hold_tilt_deg': float(max(
                (r.hold_tilt_deg for r in caught), default=0.0)),
            'worst_separation_ms': float(max(
                (r.separation_ms for r in results), default=0.0)),
            'worst_preposition_err_mm': float(max(prepos)) if prepos else None,
            'worst_landing_err_mm': float(max(landerr)) if landerr else None,
            'worst_announced_landing_err_ms': (
                float(max(annerr, key=abs)) if annerr else None),
            'required_leg_vel_mmps': float(req_vel),
            'required_leg_acc_mmps2': float(req_acc),
            'required_leg_jerk_mmps3': float(req_jerk),
            'hardware_marginal_note': _HW_MARGINAL_NOTE,
            'contact_diagnostic': diag,
            'config': dataclasses.asdict(cfg),
            'thresholds': {
                'vel_match_frac': VEL_MATCH_FRAC,
                'hold_travel_mm': HOLD_TRAVEL_MM,
                'hold_tilt_deg': HOLD_TILT_DEG,
                'separation_ms': SEPARATION_MS,
                'preposition_tol_mm': cfg.preposition_tol_mm,
            },
            'results': [r.to_dict() for r in results],
        }


def run_gate(cfg: TossGateConfig, viewer_speed=None) -> dict:
    gate = TossGate(cfg)
    attach_viewer(gate, viewer_speed, tag='toss_gate')
    try:
        report = gate.run()
    except ViewerClosed:
        print("[toss_gate] viewer closed — run stopped by operator (no report).")
        raise SystemExit(130)
    finally:
        if gate.viewer is not None:
            gate.viewer.close()
    path = cfg.report_path
    if path is None:
        os.makedirs(os.path.join(_repo_root, 'temp', 'reports'), exist_ok=True)
        path = os.path.join(
            _repo_root, 'temp', 'reports',
            f"toss_gate_seed{cfg.seed}_n{report['trials']}.json")
    with open(path, 'w') as f:
        json.dump(report, f, indent=2)
    report['report_path'] = path
    return report


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description="Headless seeded single-ball-toss sim gate.")
    p.add_argument('--trials-per-point', type=int, default=10)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--grid', choices=('factored', 'full'), default='factored')
    p.add_argument('--pose', default=None,
                   help="X,Y,Z single-point debug run (with --flight).")
    p.add_argument('--flight', type=float, default=None,
                   help="flight time (s) for the --pose single-point run.")
    p.add_argument('--release-noise-frac', type=float, default=0.01,
                   help="PLACEHOLDER release-velocity noise fraction until "
                        "Phase-5 T0 measures the real scatter.")
    p.add_argument('--arm-time-err-s', type=float, default=0.0)
    p.add_argument('--event-vel-err-frac', type=float, default=0.0)
    p.add_argument('--no-diag', action='store_true',
                   help="skip the non-gating contact-physics diagnostic column.")
    p.add_argument('--diag-release', choices=('detach', 'kinematic'),
                   default='detach')
    p.add_argument('--viewer', action='store_true',
                   help="open an interactive MuJoCo viewer and pace to real "
                        "time (observational only — results are bit-identical "
                        "to headless; needs a display).")
    p.add_argument('--viewer-speed', type=float, default=1.0)
    p.add_argument('--report', default=None)
    args = p.parse_args(argv)
    viewer_speed = args.viewer_speed if args.viewer else None
    points = None
    if args.pose is not None or args.flight is not None:
        if args.pose is None or args.flight is None:
            p.error("--pose and --flight must be given together")
        x, y, z = (float(v) for v in args.pose.split(','))
        points = [(x, y, z, float(args.flight))]
    cfg = TossGateConfig(
        trials_per_point=args.trials_per_point, seed=args.seed,
        grid=args.grid, points=points,
        release_vel_noise_frac=args.release_noise_frac,
        arm_time_err_s=args.arm_time_err_s,
        event_vel_err_frac=args.event_vel_err_frac,
        contact_diag=not args.no_diag, diag_release=args.diag_release,
        report_path=args.report)
    t0 = time.time()
    rep = run_gate(cfg, viewer_speed=viewer_speed)
    wall = time.time() - t0
    for row in rep['points']:
        if row['binding_2_3_mps'] or row['binding_T080_z170']:
            band = '2-3m/s' if row['binding_2_3_mps'] else 'T0.80@z170'
            print(f"[toss_gate]   {row['point_id']}: core_clean "
                  f"{row['core_clean']}/{row['n']} "
                  f"{'PASS' if row['pass_9_of_10'] else 'FAIL'}  "
                  f"(binding {band}"
                  f"{', hw-marginal' if row['hardware_marginal_flight'] else ''})")
    if rep['no_binding_points']:
        print("[toss_gate] *** NO BINDING POINTS — gating on full band *** "
              "(this run's grid intersects neither the 2-3 m/s band nor "
              "T=0.80 @ z=170; explicit-points/debug run)")
        print(f"[toss_gate] {'PASS' if rep['passed'] else 'FAIL'}  "
              f"full-band {'PASS' if rep['passed_full_band'] else 'FAIL'}  "
              f"(vel-match deferred)")
    else:
        print(f"[toss_gate] {'PASS' if rep['passed'] else 'FAIL'}  "
              f"2-3m/s band {'PASS' if rep['passed_2_3_mps_band'] else 'FAIL'}  "
              f"T0.80@z170 band "
              f"{'PASS' if rep['passed_T080_z170_band'] else 'FAIL'}  "
              f"full-band {'PASS' if rep['passed_full_band'] else 'FAIL'} "
              f"(advisory)  (vel-match deferred)")
    print(f"[toss_gate] clean {rep['clean']}  core_clean {rep['core_clean']}"
          f"/{rep['trials']}  caught {rep['caught']}  "
          f"accepted {rep['accepted']}  "
          f"feas_viol {rep['feasibility_violations_in_accepted']}  "
          f"pump_rejects {rep['total_pump_rejects']}")
    print(f"[toss_gate] worst vel-match {rep['worst_vel_match_frac']}  "
          f"hold travel {rep['worst_hold_travel_mm']:.2f} mm  "
          f"tilt {rep['worst_hold_tilt_deg']:.2f}°  "
          f"sep {rep['worst_separation_ms']:.1f} ms  "
          f"prepos {rep['worst_preposition_err_mm']} mm  "
          f"landing {rep['worst_landing_err_mm']} mm  "
          f"ann-landing {rep['worst_announced_landing_err_ms']} ms")
    print(f"[toss_gate] required limits: vel {rep['required_leg_vel_mmps']:.0f} "
          f"acc {rep['required_leg_acc_mmps2']:.0f} "
          f"jerk {rep['required_leg_jerk_mmps3']:.0f}  "
          f"(wall {wall:.1f}s)  → {rep['report_path']}")
    return 0 if rep['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
