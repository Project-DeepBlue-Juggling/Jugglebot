"""Skill-stack sim gate -- ``SkillExecutor`` on the MuJoCo plant (plan § 4 R2).

Replaces ``sim/cycle_gate.py`` and ``sim/unified_gate.py`` (both deleted the
same rung, 2026-09-12, skill-stack R2 Unit E): those gated the SUPERSEDED
FSM/unified-cycle stacks; this one drives the CURRENT one --
``jugglebot.motion.skills.executor.SkillExecutor`` walking a
``schedule.compile_columns`` columns pattern against the REAL chain::

    schedule.compile_columns                     the columns timeline: THROW/
        │                                         CATCH/REST skills on the
        │                                         shared wall clock
        ▼
    executor.SkillExecutor.tick()                 dispatches skills that are
        │                                         due, builds their terminals
        ▼
    executor.install_segment()                    plans one skill (segments.
        │                                         plan_segment -> unified_cycle)
        │                                         and splices/fresh-installs it
        │                                         onto the live PlanRecord
        ▼
    stream_chain: emitter -> pump -> wire -> mirror -> MuJoCoPlant
                                               (verbatim shared chain, moved out
                                               of unified_gate so this gate and
                                               the ported hand-lane tests share
                                               one copy)

**Capture model**: kinematic (``contact_carry=False``) -- MuJoCo contact
triggers the make, the carry is a kinematic hold, exactly the resolution
``sim/cycle_gate.py`` and ``sim/unified_gate.py`` both recorded (owner,
2026-08-29): hardware catches are already smooth and the MuJoCo contact model
is the low-fidelity element.

**Noise** (owner, 2026-09-12): the RELEASE is exact (``bb_throw_noise_frac``
0.0) and the OBSERVATION carries the 0.5 mm tracking noise.  R2 certifies the
software chain; the per-throw scatter of the machine's own 0.9 m self-toss is
unmeasured (the model's 2 % default is the Ball Butler's, a documented
placeholder) and is what the R3 sitting measures.  MEASURED 2026-09-12 (this
gate, 20 throws x 5 seeds, 300/5000/200k, hand 3500): the 100 mm hop uses the
whole 5000 mm/s^2 budget, so a landing scatter >= 0.5 % (~18 mm) is refused
by the gate before motion (0 drops in every case); 80 mm passes 3/5 seeds at
0.5 %; 60 mm passes 5/5 at 0.5 % and fails at 1 %.  That table is the R4/R5
separation-vs-scatter sizing input; ``--throw-noise-frac`` re-runs it.
**Learner**: OFF at R2 -- every THROW carries the identity
prior (``schedule.compile_columns``'s ``y_d=(zeros(2), flight_s)``), so
``executor._throw_terminal`` always aims at the commanded landing.  **Sites**:
perfect (``sites.columns_sites`` -- no site-calibration error modelled).

**One clock.**  The executor's wall clock IS ``plant.data.time`` -- no second
time source, so a `t_abs_s` comparison here means exactly what it means in
``install_segment``'s own docstring.

**Gate criterion (owner, 2026-09-12)**: 20 consecutive columns cycles (one
`n_throws=20` schedule -- 1 pre-existing catch + 19 matching catches = 20
scheduled catches) at apex 0.9 m, separation 100 mm, dwell 0.30 s, ZERO drops,
on five fixed seeds (0..4), at session limits 300/5000/200000 (leg), hand
3500 rev/s^2; the whole five-seed sweep under 5 minutes.

Run headless (default); writes a JSON report under ``temp/reports/``.
Pure Python + MuJoCo; no ROS2.

**On this module's length (628 lines, past the plan's ~500-line guideline):**
one gate now does what ``cycle_gate.py`` (single-cycle plan-domain scoring)
and ``unified_gate.py`` (grid sweep + chain streaming) split across two files
-- a LIVE multi-throw schedule (dispatch, install/splice, noise-driven
tracking, per-ball release/capture/removal bookkeeping) run through the same
real chain -- because a schedule has no fixed cycle count to score row by
row the way a grid sweep does; splitting the schedule loop from its
bookkeeping would be a second file with no seam a caller could use alone.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import os
import statistics
import sys
import time

import numpy as np

_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
from sim._paths import bootstrap_paths  # noqa: E402
bootstrap_paths()

import jugglebot.hardware_config as hw                            # noqa: E402
from jugglebot.motion.geometry import StewartGeometry             # noqa: E402
from jugglebot.motion.trajectory import KnotEmitter                # noqa: E402
from jugglebot.motion.trajectory import TrajectoryLimits           # noqa: E402
from jugglebot.motion.trajectory import ballistics_bc as bal       # noqa: E402
from jugglebot.motion import unified_cycle as uc                   # noqa: E402
from jugglebot.motion.skills import schedule as sk                 # noqa: E402
from jugglebot.motion.skills import sites                          # noqa: E402
from jugglebot.motion.skills import executor as ex                 # noqa: E402
from jugglebot.motion.skills.segments import (                     # noqa: E402
    CATCH, THROW, SegmentConfig,
)

from teensy_link.protocol import Setpoint                          # noqa: E402
from teensy_link.setpoint_pump import (                            # noqa: E402
    FLAG_HAS_HAND, FLAG_HAS_U1, FLAG_HAS_U2, FLAG_HAS_V1,
)

from sim.plant.mujoco_plant import MuJoCoPlant                     # noqa: E402
from sim.juggle_noise import BallisticEstimator, JuggleNoise, NoiseConfig  # noqa: E402
from sim import stream_chain                                       # noqa: E402
from sim.gate_common import ViewerClosed, attach_viewer             # noqa: E402

# ---------------------------------------------------------------------------
# Session limits -- read out of THIS SOURCE by regex
# (tests/ros/test_unified_cycle_bench.py::
#  test_the_session_limits_match_the_gates_that_planned_them), so this triple
# is the ONE place the R2 operating point's leg limits live for that bench.
# ---------------------------------------------------------------------------
_SESSION_LEG_VEL_MMPS = 300.0
_SESSION_LEG_ACC_MMPS2 = 5000.0
_SESSION_LEG_JERK_MMPS3 = 200000.0
_SESSION_HAND_ACC_RPS2 = 3500.0

#: Worst |mirror - plan| (rev) the HAND / LEG reconstruction bands allow --
#: carried over from ``sim/unified_gate.py`` (deleted this rung) verbatim, with
#: their provenance: a wire-quantisation bound for the hand (one float32
#: half-ulp at the ~10 rev operating point, MEASURED worst 4.766e-07 rev over
#: the full 27-point unified grid, 2026-09-05) and a measured IK-curvature
#: residual for the legs (a rev-space cubic through IK'd knots is not the IK of
#: a pose-space cubic in between; MEASURED worst 1.065e-05 rev on unified's
#: 60 mm ring, 2026-09-05; re-banded 2026-09-11 for the measured hand gain --
#: see ``sim/unified_gate.py``'s git history for the full derivation this gate
#: no longer carries).
MIRROR_TOL_HAND_REV = 4.0e-6
MIRROR_TOL_LEG_REV = 1.0e-4

#: 40 Hz knot grid (s) -- the fixed emitter/plan grid, never re-derived.
KNOT_DT_S = float(hw.JB_TRAJ_KNOT_DT_S)
#: Firmware tick (s) -- 500 Hz, shared with ``stream_chain.TICK_S``.
TICK_S = stream_chain.TICK_S
#: Tracking observation rate -- the QTM rate (plan § 3 noise model).
OBS_PERIOD_S = 0.005
#: How long the host keeps quiet after the last event, covering the firmware's
#: whole wind-down ladder plus room to remove the final unscheduled ball.
QUIET_TAIL_S = 0.5

_WANT_FLAGS = FLAG_HAS_U1 | FLAG_HAS_U2 | FLAG_HAS_HAND | FLAG_HAS_V1


# ---------------------------------------------------------------------------
# Config + result records
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class SkillsGateConfig:
    apex_m: float = 0.9
    separation_mm: float = 100.0
    dwell_s: float = 0.30
    n_throws: int = 20
    seeds: tuple = (0, 1, 2, 3, 4)
    leg_vel_mmps: float = _SESSION_LEG_VEL_MMPS
    leg_acc_mmps2: float = _SESSION_LEG_ACC_MMPS2
    leg_jerk_mmps3: float = _SESSION_LEG_JERK_MMPS3
    hand_acc_rps2: float = _SESSION_HAND_ACC_RPS2
    noise: NoiseConfig = dataclasses.field(
        default_factory=lambda: NoiseConfig(bb_throw_noise_frac=0.0,
                                            tracking_noise_mm=0.5))
    report_path: str = None


@dataclasses.dataclass
class SkillsTrialResult:
    """One seed's columns run, scored.  NaN for unavailable, never ``None``."""

    seed: int
    scheduled_catches: int = 0
    makes: int = 0
    drops: int = 0
    attempt_ended: bool = False
    end_code: str = ''
    installs_total: int = 0
    installs_accepted: int = 0
    plan_wall_s: tuple = ()
    pump_frames_emitted: int = 0
    pump_frames_accepted: int = 0
    pump_rejects: int = 0
    flags_seen: tuple = ()
    mirror_leg_worst_rev: float = float('nan')
    mirror_hand_worst_rev: float = float('nan')
    ticks: int = 0
    wall_s: float = float('nan')

    pump_clean: bool = False
    mirror_ok: bool = False
    flags_ok: bool = False
    all_installed: bool = False
    caught_all: bool = False
    no_drops: bool = False
    passed: bool = False

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        d['flags_seen'] = [hex(f) for f in self.flags_seen]
        return d


class _InstallCtx:
    """The one mutable box the installer closure needs (record, warm start,
    the seed used only for the very first fresh install, and the pending
    releases a THROW's acceptance schedules)."""

    def __init__(self, seed_rest: uc.CycleState):
        self.record = None
        self.seed_rest = seed_rest
        self.warm_start = None
        self.pending_releases = []
        self.plan_wall_s = []
        self.installs_total = 0
        self.installs_accepted = 0


# ---------------------------------------------------------------------------
# The gate
# ---------------------------------------------------------------------------

class SkillsGate:
    """Runs the shipped columns schedule through the whole chain, per seed."""

    def __init__(self, cfg: SkillsGateConfig = None):
        self.cfg = SkillsGateConfig() if cfg is None else cfg
        self.geom = StewartGeometry()
        self.limits = TrajectoryLimits.from_config(hw).with_session_limits(
            leg_vel_mmps=self.cfg.leg_vel_mmps,
            leg_acc_mmps2=self.cfg.leg_acc_mmps2,
            leg_jerk_mmps3=self.cfg.leg_jerk_mmps3,
            hand_acc_rps2=self.cfg.hand_acc_rps2)
        self.rcfg = uc.build_realize_config(self.limits)
        self.mm_to_rev = np.asarray(hw.GEOM_MM_TO_REV, dtype=float)
        self.seg_cfg = SegmentConfig()
        self.viewer = None
        self._plant = None

    @property
    def plant(self):
        """The gating plant -- kinematic capture authority (``contact_carry=
        False``): MuJoCo contact triggers the make, the carry is a kinematic
        hold rather than contact physics."""
        if self._plant is None:
            self._plant = MuJoCoPlant(geom=self.geom)
        return self._plant

    def _rest_state(self, site: 'sites.Site') -> uc.CycleState:
        """A stationary state whose cup opening sits at ``site``'s REST height.

        Same construction as ``unified_gate.UnifiedGate._rest_state`` /
        ``tests/motion/test_unified_cycle.py``'s ``_rest_state``: the pose and
        the slider are built through the level realisation by hand, so the
        inputs owe nothing to the forward map the planner is being gated on.
        """
        cfg = self.rcfg
        rest_mm = site.rest_site_mm()
        slider_mm = float(rest_mm[2]) - float(cfg.cup_z_base_mm)
        rev = ((slider_mm - float(cfg.slider_rev_zero_mm)) / 1000.0
               * hw.HAND_REV_PER_M)
        pose = np.array([float(rest_mm[0]), float(rest_mm[1]),
                         float(cfg.active_z_mm), 0.0, 0.0, 0.0])
        return uc.CycleState.at_rest(pose, rev, cfg)

    # ── one trial ─────────────────────────────────────────────────────────

    def run_trial(self, seed: int) -> SkillsTrialResult:
        cfg = self.cfg
        plant = self.plant
        geom = self.geom
        noise = JuggleNoise(cfg.noise, seed=seed)
        site0, site1 = sites.columns_sites(cfg.separation_mm)

        # 1. Settle the machine at rest at site 0 (ball A's home).
        rest0 = self._rest_state(site0)
        pose0 = np.asarray(rest0.pose, dtype=float)
        plant.reset(pose0)
        plant.command(plant.pose_to_extensions(pose0))
        plant.command_hand(stream_chain.slider_mm_of_rev(
            float(rest0.hand_rev), self.rcfg))
        for _ in range(40):
            plant.step(KNOT_DT_S)
            if self.viewer is not None:
                self.viewer.sync()
        plant.ball_manager.ball(0).spawn_in_hand()

        # 2. Spawn ball B (ball 1) already in flight: a nominal vertical
        # self-toss of the same apex, both ends at site 1's CATCH height (the
        # same symmetric-height assumption ``schedule.flight_s`` itself makes,
        # so the schedule's own tau/beta stay exact).  ``t0_abs_s`` is chosen
        # as spawn-time + beta so the schedule's "ball B released at t0-beta"
        # premise (``schedule.compile_columns``'s docstring) is exactly this
        # spawn -- no fast-forward needed, and the ball's TRUE physics landing
        # (MuJoCo gravity, deterministic) lands at spawn+t_f = t0+tau exactly.
        t_f = sk.flight_s(cfg.apex_m)
        beta = sk.beat_s(t_f, cfg.dwell_s)
        tau = sk.transit_s(t_f, cfg.dwell_s)
        v0_mm_s = math.sqrt(2.0 * bal.GRAVITY_MMS2 * cfg.apex_m * 1000.0)
        pos1 = site1.catch_site_mm()
        pos1n, vel1n = noise.perturb_throw(
            pos1, np.array([0.0, 0.0, v0_mm_s]), np.zeros(3))
        t_spawn = float(plant.data.time)
        plant.spawn_ball(pos1n, vel1n, ball=1)
        t0_abs_s = t_spawn + beta

        pattern = sk.Pattern(sites=(site0, site1), apex_m=cfg.apex_m,
                             dwell_s=cfg.dwell_s, n_throws=cfg.n_throws)
        sched = sk.compile_columns(pattern, t0_abs_s)
        scheduled_catches = sum(1 for s in sched.skills if s.kind == CATCH)
        t_land_final = t0_abs_s + (cfg.n_throws - 1) * beta + t_f
        ball_final = (cfg.n_throws - 1) % 2

        # 3. Per-ball tracking / hold-bookkeeping state.
        estimators = {0: BallisticEstimator(bal.G_VEC_MMS2),
                     1: BallisticEstimator(bal.G_VEC_MMS2)}
        airborne = {0: False, 1: True}
        next_obs_t = {0: None, 1: t_spawn}
        expect_held = {0: True, 1: False}
        lost_since = {0: None, 1: None}
        removed_final = [False]
        makes = [0]
        drops = [0]

        def tracker(ball_id):
            est = estimators[ball_id]
            if est.n < 3:
                return None
            p_est, v_est = est.estimate()
            try:
                pos, vel, t_rem = bal.arrival_state_at_z(
                    p_est, v_est, sites.CATCH_CUP_Z_MM, descending=True)
            except ValueError:
                return None
            return ex.Landing(pos_mm=np.asarray(pos, dtype=float),
                              vel_mm_s=np.asarray(vel, dtype=float),
                              t_land_abs_s=float(plant.data.time + t_rem))

        ictx = _InstallCtx(rest0)

        def installer(kind, terminal, t_now_s, *, ball_id):
            new_record, result, seg = ex.install_segment(
                ictx.record, ictx.seed_rest, kind, terminal, t_now_s,
                cfg=self.seg_cfg, limits=self.limits, geom=geom,
                warm_start=ictx.warm_start)
            ictx.installs_total += 1
            if result.accepted:
                ictx.record = new_record
                ictx.seed_rest = None
                ictx.warm_start = seg.warm_start
                ictx.installs_accepted += 1
                ictx.plan_wall_s.append(float(result.plan_wall_s))
                if kind == THROW:
                    ictx.pending_releases.append((
                        float(terminal.t_release_s), ball_id,
                        np.asarray(seg.takeoff_vel_mm_s, dtype=float),
                        np.asarray(terminal.site_mm, dtype=float),
                        np.asarray(terminal.target_mm, dtype=float)))
                elif kind == CATCH and terminal.then_throw is not None:
                    # A catch-with-throw carries its OWN release mark
                    # (`Segment.release_t_s` on the plan clock; the terminal's
                    # own `then_throw.t_release_s` is still the ABSOLUTE wall
                    # instant `SkillExecutor._catch_terminal` built it from --
                    # `install_segment` re-bases a COPY for the solve, never
                    # this object) -- the schedule's own `ThenThrow` folded
                    # this release onto the catch (`schedule.
                    # _fold_catch_throw_pairs`), so this is the ONLY place it
                    # is ever scheduled.
                    tt = terminal.then_throw
                    ictx.pending_releases.append((
                        float(tt.t_release_s), ball_id,
                        np.asarray(seg.takeoff_vel_mm_s, dtype=float),
                        np.asarray(tt.site_mm, dtype=float),
                        np.asarray(tt.target_mm, dtype=float)))
            return result

        executor = ex.SkillExecutor(sched, installer, tracker=tracker)

        # 4. Stream.
        emitter = KnotEmitter(geom)
        pump = stream_chain.make_pump()
        mirror = stream_chain.make_mirror(geom)
        t_end = t_land_final + QUIET_TAIL_S
        next_frame_t = plant.data.time
        frame_seq = 0
        frame_t0 = None
        latch_tau = 0.0
        emitted = accepted = 0
        flags_seen = set()
        worst_leg = worst_hand = 0.0
        ticks = 0
        t_wall = time.time()

        while plant.data.time < t_end:
            t = plant.data.time

            # Dispatch-check every TICK, not every 40 Hz frame: a skill's
            # window budget is measured from the ACTUAL install instant
            # (``install_segment``'s splice lead), so checking only at the
            # 25 ms frame grid wastes up to a whole knot of margin the
            # schedule's own ``LEAD_KNOTS`` already had to spend once on the
            # wire's lookahead -- a real node's dispatch loop is not tied to
            # the 40 Hz emitter cadence either.
            executor.tick(t)

            if t >= next_frame_t - 1e-9:
                record = ictx.record
                if record is not None:
                    # A fresh install starts a NEW record.t0_s (a splice never
                    # does -- install_segment carries it forward unchanged) --
                    # reset the frame counter to that origin so knot 0 samples
                    # at tau=0 exactly.
                    if frame_t0 is None or record.t0_s != frame_t0:
                        frame_t0 = record.t0_s
                        frame_seq = 0
                        next_frame_t = record.t0_s
                    # Sample the NOMINAL grid instant (seq*dt), never the raw
                    # physical clock: TICK_S (2 ms) does not divide dt (25 ms)
                    # evenly, so a frame gated on "t >= next_frame_t" can fire
                    # up to one tick late, and feeding that late instant to a
                    # HAND channel that is mid-jerk (e.g. right after a THROW's
                    # release) measurably moves u0 off the knot the mirror
                    # reconstructs against -- worst observed 6.2e-3 rev before
                    # this fix (2026-09-12), three orders above MIRROR_TOL_
                    # HAND_REV, entirely a sampling-instant artefact and not a
                    # property of the plan or the wire.
                    tau = frame_seq * record.plan.dt
                    if tau <= record.plan.total_duration + 1e-9:
                        frame = emitter.frame(record.plan, tau, frame_seq)
                        sp, _reason = pump.build(
                            frame, t_origin_us=int(frame_seq * 25000))
                        emitted += 1
                        if sp is not None:
                            accepted += 1
                            flags_seen.add(int(sp.flags))
                            stream_chain.latch(mirror, Setpoint.unpack(sp.pack()),
                                              t)
                            latch_tau = tau
                        frame_seq += 1
                next_frame_t += record.plan.dt if record is not None else KNOT_DT_S

            record = ictx.record
            if record is not None:
                st = plant.get_state()
                fb_rev = list(np.asarray(st.leg_extensions_mm) * self.mm_to_rev)
                fb_hand = stream_chain.rev_of_slider_mm(
                    float(st.hand_pos_mm), self.rcfg)
                cmd_pos, _cmd_vel, _ = mirror.tick(t, fb_rev)
                hand_out = mirror.tick_hand(t, fb_hand)
                plant.command(np.asarray(cmd_pos) / self.mm_to_rev)
                if hand_out is not None:
                    plant.command_hand(
                        stream_chain.slider_mm_of_rev(hand_out[0], self.rcfg))

                tau_phase = latch_tau + (t - mirror.base_timestamp)
                if 0.0 <= tau_phase <= record.plan.total_duration:
                    d_leg, d_hand = stream_chain.recon(
                        record.plan, mirror, tau_phase, geom, self.mm_to_rev)
                    worst_leg = max(worst_leg, d_leg)
                    worst_hand = max(worst_hand, d_hand)

            # Releases due (a THROW's takeoff, at the terminal's ABSOLUTE
            # instant) -- pop everything due, not just the head, so a tick
            # that straddles two close releases still fires both.
            while ictx.pending_releases and ictx.pending_releases[0][0] <= t:
                t_rel, b_id, vel_mm_s, site_mm, target_mm = \
                    ictx.pending_releases.pop(0)
                if plant.has_ball and plant.get_ball_state(b_id).held:
                    _, v_noisy = noise.perturb_throw(
                        site_mm, vel_mm_s, target_mm - site_mm)
                    plant.release_ball(v_noisy, ball=b_id)
                expect_held[b_id] = False
                lost_since[b_id] = None
                airborne[b_id] = True
                estimators[b_id].reset()
                next_obs_t[b_id] = t

            for b in (0, 1):
                if plant.check_and_capture(b):
                    makes[0] += 1
                    expect_held[b] = True
                    airborne[b] = False
                if expect_held[b]:
                    bs = plant.get_ball_state(b)
                    if not bs.held:
                        if lost_since[b] is None:
                            lost_since[b] = t
                            drops[0] += 1
                    else:
                        lost_since[b] = None
                if (airborne[b] and next_obs_t[b] is not None
                        and t >= next_obs_t[b]):
                    bs = plant.get_ball_state(b)
                    estimators[b].add(t, noise.observe(bs.position_mm))
                    next_obs_t[b] += OBS_PERIOD_S

            if not removed_final[0] and t >= t_land_final:
                plant.ball_manager.ball(ball_final).reset()
                airborne[ball_final] = False
                expect_held[ball_final] = False
                removed_final[0] = True

            plant.step(TICK_S)
            ticks += 1
            if self.viewer is not None:
                self.viewer.sync()

        pump_clean = bool(pump.frames_rejected == 0 and accepted == emitted
                          and emitted > 0)
        mirror_ok = bool(worst_leg <= MIRROR_TOL_LEG_REV
                        and worst_hand <= MIRROR_TOL_HAND_REV)
        flags_ok = bool(flags_seen == {_WANT_FLAGS})
        # A CATCH RE-SEND's refusal is a documented, non-fatal path
        # (``executor.SkillExecutor``'s own docstring: the committed catch
        # stands, which is strictly better than none, so the attempt
        # continues) -- installs_total/installs_accepted below counts every
        # resend attempt too and is reported for diagnosis, but the PASS
        # criterion is the one thing a resend refusal does NOT break.
        all_installed = bool(not executor.attempt_ended)
        caught_all = bool(makes[0] >= scheduled_catches > 0)
        no_drops = bool(drops[0] == 0)

        res = SkillsTrialResult(
            seed=seed, scheduled_catches=scheduled_catches, makes=makes[0],
            drops=drops[0], attempt_ended=bool(executor.attempt_ended),
            end_code=str(executor.end_code),
            installs_total=ictx.installs_total,
            installs_accepted=ictx.installs_accepted,
            plan_wall_s=tuple(ictx.plan_wall_s),
            pump_frames_emitted=emitted, pump_frames_accepted=accepted,
            pump_rejects=int(pump.frames_rejected),
            flags_seen=tuple(sorted(flags_seen)),
            mirror_leg_worst_rev=worst_leg, mirror_hand_worst_rev=worst_hand,
            ticks=ticks, wall_s=time.time() - t_wall,
            pump_clean=pump_clean, mirror_ok=mirror_ok, flags_ok=flags_ok,
            all_installed=all_installed, caught_all=caught_all,
            no_drops=no_drops)
        res.passed = bool(pump_clean and mirror_ok and flags_ok
                          and all_installed and caught_all and no_drops)
        return res

    # ── run + summarise ───────────────────────────────────────────────────

    def run(self) -> dict:
        t0 = time.time()
        results = [self.run_trial(s) for s in self.cfg.seeds]
        wall_s = time.time() - t0

        all_wall_ms = [w * 1000.0 for r in results for w in r.plan_wall_s]
        all_wall_ms.sort()

        def _pctl(vals, p):
            if not vals:
                return float('nan')
            k = min(len(vals) - 1, int(round(p * (len(vals) - 1))))
            return float(vals[k])

        report = {
            'gate': 'skills',
            'passed': bool(results and all(r.passed for r in results)),
            'seeds': list(self.cfg.seeds),
            'n_throws': self.cfg.n_throws,
            'apex_m': self.cfg.apex_m,
            'separation_mm': self.cfg.separation_mm,
            'dwell_s': self.cfg.dwell_s,
            'wall_s': wall_s,
            'plan_wall_ms': {
                'min': _pctl(all_wall_ms, 0.0),
                'p50': (float(statistics.median(all_wall_ms)) if all_wall_ms
                        else float('nan')),
                'max': _pctl(all_wall_ms, 1.0),
                'n': len(all_wall_ms),
            },
            'thresholds': {
                'mirror_tol_leg_rev': MIRROR_TOL_LEG_REV,
                'mirror_tol_hand_rev': MIRROR_TOL_HAND_REV,
                'session_leg_vel_mmps': self.cfg.leg_vel_mmps,
                'session_leg_acc_mmps2': self.cfg.leg_acc_mmps2,
                'session_leg_jerk_mmps3': self.cfg.leg_jerk_mmps3,
                'session_hand_acc_rps2': self.cfg.hand_acc_rps2,
            },
            'trials': [r.to_dict() for r in results],
        }
        path = self.cfg.report_path
        if path is None:
            out_dir = os.path.join(_repo_root, 'temp', 'reports')
            os.makedirs(out_dir, exist_ok=True)
            path = os.path.join(
                out_dir, 'skills_gate_%s.json'
                % time.strftime('%Y%m%dT%H%M%S'))
        with open(path, 'w') as fh:
            json.dump(report, fh, indent=2)
        report['report_path'] = path
        return report


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------

def run_gate(cfg: SkillsGateConfig = None, viewer_speed: float = None) -> dict:
    cfg = SkillsGateConfig() if cfg is None else cfg
    gate = SkillsGate(cfg)
    try:
        if viewer_speed is not None:
            attach_viewer(gate, viewer_speed, tag='skills_gate')
        report = gate.run()
    except ViewerClosed:
        print('[skills_gate] viewer closed by the operator -- no report written.')
        raise SystemExit(130)
    finally:
        if gate.viewer is not None:
            gate.viewer.close()
    return report


def _print_table(rep: dict) -> None:
    print('[skills_gate] %-6s %-8s %-6s %-6s %-6s %-10s %-10s %-9s'
          % ('seed', 'verdict', 'sched', 'makes', 'drops', 'mir_leg',
             'mir_hand', 'installs'))
    for t in rep['trials']:
        verdict = 'PASS' if t['passed'] else 'FAIL'
        print('[skills_gate] %-6d %-8s %-6d %-6d %-6d %-10.2e %-10.2e %d/%d'
              % (t['seed'], verdict, t['scheduled_catches'], t['makes'],
                 t['drops'], t['mirror_leg_worst_rev'],
                 t['mirror_hand_worst_rev'], t['installs_accepted'],
                 t['installs_total']))
    pw = rep['plan_wall_ms']
    print('[skills_gate] plan wall ms: min %.2f  p50 %.2f  max %.2f  (n=%d)'
          % (pw['min'], pw['p50'], pw['max'], pw['n']))
    print('[skills_gate] %s  (wall %.1f s over %d seed(s))  -> %s'
          % ('PASS' if rep['passed'] else 'FAIL', rep['wall_s'],
             len(rep['seeds']), rep.get('report_path')))


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--seeds', type=int, nargs='+', default=None,
                   help='override the default five seeds (0..4)')
    p.add_argument('--n-throws', type=int, default=None)
    p.add_argument('--viewer', action='store_true')
    p.add_argument('--no-viewer', action='store_true',
                   help='headless (the default; accepted for symmetry)')
    p.add_argument('--viewer-speed', type=float, default=1.0)
    p.add_argument('--report', default=None)
    p.add_argument('--separation-mm', type=float, default=None,
                   help='site separation (default: the operating point, 100 mm)')
    p.add_argument('--throw-noise-frac', type=float, default=None,
                   help='per-component release-velocity scatter as a fraction of '
                        'the release speed (default 0.0: an exact release; the '
                        'module docstring carries the separation-vs-scatter table)')
    args = p.parse_args(argv)

    cfg = SkillsGateConfig(report_path=args.report)
    if args.seeds is not None:
        cfg.seeds = tuple(args.seeds)
    if args.n_throws is not None:
        cfg.n_throws = args.n_throws
    if args.separation_mm is not None:
        cfg.separation_mm = float(args.separation_mm)
    if args.throw_noise_frac is not None:
        cfg.noise = NoiseConfig(bb_throw_noise_frac=float(args.throw_noise_frac),
                                tracking_noise_mm=cfg.noise.tracking_noise_mm)

    rep = run_gate(cfg, viewer_speed=(args.viewer_speed if args.viewer
                                      else None))
    _print_table(rep)
    return 0 if rep['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
