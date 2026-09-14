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

**On this module's length (1219 lines at R3, past the plan's ~500-line
guideline; 628 at R2):** one gate now does what ``cycle_gate.py``
(single-cycle plan-domain scoring) and ``unified_gate.py`` (grid sweep +
chain streaming) split across two files -- a LIVE multi-throw schedule
(dispatch, install/splice, noise-driven tracking, per-ball release/capture/
removal bookkeeping) run through the same real chain -- because a schedule
has no fixed cycle count to score row by row the way a grid sweep does;
splitting the schedule loop from its bookkeeping would be a second file with
no seam a caller could use alone. **R3 (plan § 4 R3 "Sim validation")** adds
the self-toss learner run (``SelfTossGateConfig``, ``run_self_toss_attempt``/
``run_self_toss_seed``/``run_learn``): it is a SECOND caller of the one
stream loop (``_stream_chain``, extracted from ``run_trial`` at this rung so
neither caller carries a second copy of it) and of the shared
tracker/installer closures (``_make_tracker`` / ``_make_installer``), not a
second gate file -- a self-toss run and a columns run differ only in their
schedule, limits and the R3-only learner/box/observer wiring, which is a
few hundred lines of genuinely new orchestration (cheap-reset attempts,
per-throw error scoring, the monotone-decay verdict) with nowhere smaller to
land than beside the loop it drives.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import os
import statistics
import sys
import tempfile
import time

import numpy as np

_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
from sim._paths import bootstrap_paths  # noqa: E402
bootstrap_paths()

import jugglebot.hardware_config as hw                            # noqa: E402
from jugglebot import ball_possession as bp                        # noqa: E402
from jugglebot.motion.geometry import StewartGeometry             # noqa: E402
from jugglebot.motion.trajectory import KnotEmitter                # noqa: E402
from jugglebot.motion.trajectory import TrajectoryLimits           # noqa: E402
from jugglebot.motion.trajectory import ballistics_bc as bal       # noqa: E402
from jugglebot.motion import unified_cycle as uc                   # noqa: E402
from jugglebot.motion.skills import admissible as adm              # noqa: E402
from jugglebot.motion.skills import learner as lr                  # noqa: E402
from jugglebot.motion.skills import memory as mem                  # noqa: E402
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


def _make_tracker(plant, ball_state: dict):
    """The one ``tracker(ball_id) -> Optional[ex.Landing]`` closure, shared by
    the columns run and the R3 self-toss run: a noise-averaged ballistic fit
    (``ball_state[ball_id]['estimator']``) projected to the catch plane.
    Factored out of ``run_trial`` so the two callers read one definition.

    ``t_rem`` (:func:`ballistics_bc.arrival_state_at_z`) is time-to-touchdown
    from the fit's OWN reference instant — the estimator's latest SAMPLE time
    (``BallisticEstimator.estimate()`` evaluates at ``Δt = 0`` there), not
    "now". ``t_land_abs_s`` must therefore be ``last_obs_t + t_rem``
    (``ball_state[ball_id]['last_obs_t']``, stamped in ``_stream_chain`` at
    every ``estimator.add()``), never ``plant.data.time + t_rem``: once
    sampling stops (the ball caught, ``bstate['airborne']`` False) the fit is
    frozen and ``t_rem`` is a FIXED offset from that frozen reference, so
    adding it to the advancing "now" drifts the predicted landing later by
    exactly the ticks elapsed since the last sample — R3-j probe 2026-09-13,
    ``sim/skills_gate.py --learn --policy B --seeds 0``: a self-toss LAUNCH
    throw whose catch fires ~25 ms before the fit's own predicted crossing
    (the ball arriving early under the +11% launch-speed plant bias) carried
    that 25 ms of drift across the ~94 ms from capture to
    ``executor.CAUGHT_WINDOW_S`` finalisation, reporting a flight 96 ms long
    (0.9548 s against a true ~0.859 s) -- the observed/commanded ratio for
    every LAUNCH throw in that run (1.11-1.23, non-constant) while every
    CHAINED throw, whose catch never actually closed before the schedule
    refused, read the correct ~1.11 (the plant bias alone).
    """

    def tracker(ball_id):
        bstate = ball_state[ball_id]
        if not bstate['airborne']:
            return None
        est = bstate['estimator']
        if est.n < 3:
            return None
        p_est, v_est = est.estimate()
        try:
            pos, vel, t_rem = bal.arrival_state_at_z(
                p_est, v_est, sites.CATCH_CUP_Z_MM, descending=True)
        except ValueError:
            return None
        t_ref = bstate.get('last_obs_t')
        if t_ref is None:
            t_ref = plant.data.time
        return ex.Landing(pos_mm=np.asarray(pos, dtype=float),
                          vel_mm_s=np.asarray(vel, dtype=float),
                          t_land_abs_s=float(t_ref) + float(t_rem))
    return tracker


def _make_installer(ictx: _InstallCtx, seg_cfg, limits, geom):
    """The one ``installer(kind, terminal, t_now_s, *, ball_id) ->
    InstallResult`` closure — ``ex.install_segment`` over ``ictx``'s live
    record, plus the pending-release bookkeeping every caller needs (a THROW
    or a CATCH's carried ``then_throw`` schedules the physical release the
    stream loop later pops).  Factored out of ``run_trial`` for the same
    reason as :func:`_make_tracker`: the R3 self-toss run needs the identical
    policy, not a second copy of it."""

    def installer(kind, terminal, t_now_s, *, ball_id):
        new_record, result, seg = ex.install_segment(
            ictx.record, ictx.seed_rest, kind, terminal, t_now_s,
            cfg=seg_cfg, limits=limits, geom=geom, warm_start=ictx.warm_start)
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
                tt = terminal.then_throw
                ictx.pending_releases.append((
                    float(tt.t_release_s), ball_id,
                    np.asarray(seg.takeoff_vel_mm_s, dtype=float),
                    np.asarray(tt.site_mm, dtype=float),
                    np.asarray(tt.target_mm, dtype=float)))
        return result
    return installer


# ---------------------------------------------------------------------------
# R3 — sim validation of the learner (plan § 4 R3 "Sim validation")
# ---------------------------------------------------------------------------
#
# Single site P1 self-toss, chained, injecting the MEASURED plant errors on
# top of the R2 noise model, driving the REAL SkillExecutor + learner +
# Memory + install chain through the ONE stream loop above
# (:meth:`SkillsGate._stream_chain`) -- no second copy of it.  Owner
# decisions 2026-09-13 (plan § 4 R3 "Sim validation" + the brief this rung was
# built from).

#: The R3 operating point's site: P1 = (-50, 0) mm -- the same site
#: ``sites.columns_sites(100.0)`` names ``P1`` (half of a 100 mm separation),
#: so the admissible box swept for site pair ``('P1', 'P1')`` applies without
#: a second site definition.
_SELF_TOSS_SEPARATION_MM = 100.0

#: The band a throw's landing error must enter within
#: :data:`SelfTossGateConfig.band_entry_throws` (plan § 4 R3): 20 mm lateral,
#: 20 ms of flight-time error.
XY_BAND_MM = 20.0
FLIGHT_BAND_S = 0.020

#: The monotone-decay rule's spread multiplier (owner decision 2026-09-13,
#: learner probe: false-fail 0.3-0.8% on converged noise) -- median(err,
#: throws 16-25) <= median(err, throws 6-15) + MONOTONE_K * s, s = 1.4826 *
#: MAD(err, throws 6-25).
MONOTONE_K = 1.3

#: The MEASURED plant release errors (owner decision 2026-09-13, provenance
#: ``logbook/2026-09-06`` UH-6: +5.2..+16.7% launch-speed bias, mean +10.9%,
#: rounded to the even +11% the decision states; the aim bias is the legacy
#: +8.5 mrad toward +y).  Applied to EVERY release in the R3 self-toss run,
#: on top of (never instead of) the R2 tracking-noise/exact-release model —
#: see :func:`_measured_release_bias` and its one call site in
#: :meth:`SkillsGate._stream_chain` (``release_bias``).
LAUNCH_SPEED_BIAS_FRAC = 0.11
LAUNCH_AIM_BIAS_RAD = 8.5e-3


def _measured_release_bias(vel_mm_s: np.ndarray) -> np.ndarray:
    """The machine's own measured release error: launch speed scaled by
    ``1 + LAUNCH_SPEED_BIAS_FRAC`` and the velocity rotated
    ``LAUNCH_AIM_BIAS_RAD`` about the platform x axis TOWARD +y.

    Sign convention (there being no ``y`` component to anchor one on a pure
    vertical self-toss): for ``vel_mm_s = (0, 0, v0)`` this returns
    ``y' = v0*sin(theta) > 0`` — i.e. toward +y, matching the decision's
    "+8.5 mrad +y" — and ``z' = v0*cos(theta)``, the physically tiny
    corresponding loss of vertical speed.  Reused for every throw regardless
    of its (learner-commanded) lateral component: the bias is a property of
    the LAUNCH, applied after the identity/learner command already picked a
    target, exactly where the R2 gate's own noise is applied.
    """
    v = np.asarray(vel_mm_s, dtype=float).reshape(3).copy()
    v *= (1.0 + LAUNCH_SPEED_BIAS_FRAC)
    c, s = math.cos(LAUNCH_AIM_BIAS_RAD), math.sin(LAUNCH_AIM_BIAS_RAD)
    y, z = float(v[1]), float(v[2])
    v[1] = y * c + z * s
    v[2] = -y * s + z * c
    return v


class _MemoryLearner:
    """Adapts ``memory.Memory`` to the ``learner`` duck-type ``SkillExecutor``
    wants (``command(x, y_d) -> u``, no ``cfg`` argument) by binding one
    ``learner.LearnerConfig`` for the run — the R3 hyperparameters are fixed
    per gate run, not re-supplied per throw."""

    def __init__(self, memory_obj: mem.Memory, cfg: lr.LearnerConfig):
        self._memory = memory_obj
        self._cfg = cfg

    def command(self, x, y_d):
        return self._memory.command(x, y_d, self._cfg)


def _make_observer(plant):
    """``observer(ball_id, t_abs_s) -> EVIDENCE_SEATED/EMPTY`` from the
    plant's own held state — the sim's ground truth, read live so the
    executor's precondition ladder and outcome capture run for real (plan §
    4 R3 build note: "the ladder, NO_BALL and NO_RELEASE run for real")."""

    def observer(ball_id, t_abs_s):
        bs = plant.get_ball_state(ball_id)
        held = bool(bs is not None and bs.held)
        return bp.EVIDENCE_SEATED if held else bp.EVIDENCE_EMPTY
    return observer


def _make_observations(observer):
    """``observations(t_abs_s) -> ex.Observations`` for a single-ball,
    single-site sim run: mocap/hand/level/mode are always fresh in MuJoCo (no
    staleness or mode change is modelled at R3), ``hand_at_seed`` AND
    ``hand_at_park`` are true by construction (the ACTIVATE park IS the
    schedule's own seed — see ``schedule.compile_self_toss``'s docstring, and
    Unit B's fresh-origin `REJECTED_HAND_NOT_PARKED` row), and
    ``ball_evidence`` is the live observer's answer for ball 0."""

    def observations(t_abs_s):
        return ex.Observations(
            mocap_fresh=True, hand_fresh=True, hand_at_seed=True,
            hand_at_park=True, levelled=True,
            ball_evidence=observer(0, t_abs_s), in_trajectory_mode=True)
    return observations


@dataclasses.dataclass
class SelfTossGateConfig:
    """The R3 sim-validation operating point (plan § 4 R3) — a SEPARATE
    config from :class:`SkillsGateConfig`: this run's limits (150 000 mm/s³
    leg jerk, matching ``config/generated/admissible_box.yaml``'s (P1, P1)
    sweep) differ from the columns gate's own R2 point (200 000), and its
    noise model adds the measured plant bias on top of the R2 knobs — reusing
    one dataclass would force one gate to carry a field the other never
    sets."""

    apex_m: float = 0.9
    dwell_s: float = 0.30
    leg_vel_mmps: float = 300.0
    leg_acc_mmps2: float = 5000.0
    leg_jerk_mmps3: float = 150000.0
    hand_acc_rps2: float = 3500.0
    noise: NoiseConfig = dataclasses.field(
        default_factory=lambda: NoiseConfig(bb_throw_noise_frac=0.0,
                                            tracking_noise_mm=0.5))
    learner_cfg: lr.LearnerConfig = dataclasses.field(
        default_factory=lr.LearnerConfig)
    xy_band_mm: float = XY_BAND_MM
    flight_band_s: float = FLIGHT_BAND_S
    band_entry_throws: int = 5
    #: The monotone rule's far window edge (throws 16-25) — band entry (5) +
    #: 20 more throws, plan § 4 R3.
    target_throws: int = 25
    #: Safety cap on cheap resets (owner decision 6) — far above what a
    #: converging learner needs; hitting it is itself a reportable finding.
    max_attempts: int = 60
    admissible_box_path: str = None
    report_path: str = None


def _load_admissible_boxes(path: str = None) -> list:
    """``List[AdmissibleBox]`` from ``config/generated/admissible_box.yaml``
    (or ``path``) — the sequence shape ``SkillExecutor.boxes`` wants
    (``admissible.select`` resolves ``(site pair, apex band)`` from it)."""
    if path is None:
        path = os.path.join(_repo_root, 'config', 'generated',
                            'admissible_box.yaml')
    return adm.load(path)


#: The ACTIVATE park is documented (and requested) as "hand 0 rev", which
#: sits EXACTLY on ``feasibility.HAND_STROKE_MIN_REV`` (0.0) — the workspace's
#: hard lower edge.  ``CycleState.at_rest``'s ``cup_pos_mm`` (the cup height
#: the QP chain actually seeds from, NOT the raw ``hand_rev`` field — see
#: below) round-trips hand_rev -> cup height -> hand_rev through two
#: independent floating-point paths (the forward map here, the realised
#: plan's inverse map in ``unified_cycle._realize``/``decompose``), and
#: MEASURED (2026-09-13, this rung's own probe) that round trip does not
#: land bit-exact at the origin: knot 0 reads a few ULP negative and
#: ``HAND_STROKE`` refuses "BELOW the homed zero" on EVERY attempt, at
#: t=0.000s, before any physical motion. A pure numerical artefact of
#: seeding exactly on a hard boundary, not a physical one and not the
#: learner's — so it is fixed here rather than reported as a criterion
#: failure. 1e-4 rev raises the seed's cup height by 0.033 mm (679.6 ->
#: 679.6033 mm, still "679.6 mm" to the precision every other reference to
#: this park state uses) and clears the measured few-ULP noise by nine
#: orders of magnitude — MEASURED to plan clean at 1e-5 rev already; this
#: keeps a wide margin rather than the minimum that happened to pass.
_PARK_HAND_REV_EPS = 1e-4


def _activate_park_state(rcfg, site) -> uc.CycleState:
    """The ACTIVATE park boundary condition (plan § 4 R3 carried note): hand
    at (a hair above) 0 rev — see :data:`_PARK_HAND_REV_EPS` — cup at
    ``rcfg``'s ``active_z_mm``/``slider_rev_zero_mm`` — MEASURED 679.6 mm
    (``schedule.FLOOR_LIFT_S``'s docstring) — 10 mm under the 689.6 mm
    planner floor, which is exactly why ``compile_self_toss`` opens on a
    REST that lifts off this state rather than a fresh-origin THROW planned
    from it directly."""
    pose = np.array([float(site.cup_mm[0]), float(site.cup_mm[1]),
                     float(rcfg.active_z_mm), 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, _PARK_HAND_REV_EPS, rcfg)


def _monotone_verdict(errs, band_entry_throws: int, target_throws: int,
                      k: float = MONOTONE_K):
    """The R3 monotone-decay rule (owner decision 2026-09-13): split throws
    ``band_entry_throws+1 .. target_throws`` (1-indexed: 6..25 at the
    defaults) into an early and a late half and require
    ``median(late) <= median(early) + k * s``, ``s = 1.4826 * MAD(the whole
    span)``.  ``errs`` is the per-throw error in throw order (0-indexed).
    Returns ``None`` — not evaluable — when fewer than ``target_throws``
    throws were collected, or when the window itself is empty (a config
    with ``band_entry_throws >= target_throws``, e.g. a small smoke run);
    the rule needs a non-empty window and computing on one would only
    divide by zero into a meaningless ``False``."""
    if len(errs) < target_throws or band_entry_throws >= target_throws:
        return None
    span = np.asarray(errs[band_entry_throws:target_throws], dtype=float)
    half = span.shape[0] // 2
    early, late = span[:half], span[half:]
    mad = float(np.median(np.abs(span - np.median(span))))
    s = 1.4826 * mad
    return bool(float(np.median(late)) <= float(np.median(early)) + k * s)


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
        ball_state = {
            0: dict(estimator=BallisticEstimator(bal.G_VEC_MMS2),
                   airborne=False, next_obs_t=None, expect_held=True,
                   lost_since=None, last_obs_t=None),
            1: dict(estimator=BallisticEstimator(bal.G_VEC_MMS2),
                   airborne=True, next_obs_t=t_spawn, expect_held=False,
                   lost_since=None, last_obs_t=None),
        }
        tracker = _make_tracker(plant, ball_state)
        ictx = _InstallCtx(rest0)
        installer = _make_installer(ictx, self.seg_cfg, self.limits, geom)
        executor = ex.SkillExecutor(sched, installer, tracker=tracker)

        # 4. Stream (the one loop -- ``_stream_chain``).
        t_end = t_land_final + QUIET_TAIL_S
        loop = self._stream_chain(
            plant=plant, geom=geom, rcfg=self.rcfg, noise=noise,
            executor=executor, ictx=ictx, ball_state=ball_state, t_end=t_end,
            final_removal=(t_land_final, ball_final))

        pump_clean = bool(loop['pump_rejects'] == 0
                          and loop['accepted'] == loop['emitted']
                          and loop['emitted'] > 0)
        mirror_ok = bool(loop['worst_leg'] <= MIRROR_TOL_LEG_REV
                        and loop['worst_hand'] <= MIRROR_TOL_HAND_REV)
        flags_ok = bool(loop['flags_seen'] == {_WANT_FLAGS})
        # A CATCH RE-SEND's refusal is a documented, non-fatal path
        # (``executor.SkillExecutor``'s own docstring: the committed catch
        # stands, which is strictly better than none, so the attempt
        # continues) -- installs_total/installs_accepted below counts every
        # resend attempt too and is reported for diagnosis, but the PASS
        # criterion is the one thing a resend refusal does NOT break.
        all_installed = bool(not executor.attempt_ended)
        caught_all = bool(loop['makes'] >= scheduled_catches > 0)
        no_drops = bool(loop['drops'] == 0)

        res = SkillsTrialResult(
            seed=seed, scheduled_catches=scheduled_catches,
            makes=loop['makes'], drops=loop['drops'],
            attempt_ended=bool(executor.attempt_ended),
            end_code=str(executor.end_code),
            installs_total=ictx.installs_total,
            installs_accepted=ictx.installs_accepted,
            plan_wall_s=tuple(ictx.plan_wall_s),
            pump_frames_emitted=loop['emitted'],
            pump_frames_accepted=loop['accepted'],
            pump_rejects=loop['pump_rejects'],
            flags_seen=tuple(sorted(loop['flags_seen'])),
            mirror_leg_worst_rev=loop['worst_leg'],
            mirror_hand_worst_rev=loop['worst_hand'],
            ticks=loop['ticks'], wall_s=loop['wall_s'],
            pump_clean=pump_clean, mirror_ok=mirror_ok, flags_ok=flags_ok,
            all_installed=all_installed, caught_all=caught_all,
            no_drops=no_drops)
        res.passed = bool(pump_clean and mirror_ok and flags_ok
                          and all_installed and caught_all and no_drops)
        return res

    # ── the one stream loop (shared with the R3 self-toss learner run) ─────

    def _stream_chain(self, *, plant, geom, rcfg, noise, executor, ictx,
                      ball_state, t_end, release_bias=None,
                      final_removal=None, stop_check=None):
        """The emitter/pump/wire/mirror/executor.tick loop (module
        docstring) -- run by every caller against a live ``PlanRecord``
        (``ictx``) and the executor dispatching one ``Schedule``.

        ``ball_state`` is ``{ball_id: {'estimator', 'airborne',
        'next_obs_t', 'expect_held', 'lost_since'}}`` -- the per-ball
        tracking/possession bookkeeping the SIM gate owns (the executor
        itself never touches MuJoCo). ``release_bias(vel_mm_s) ->
        vel_mm_s``, when given, is applied AFTER ``noise.perturb_throw`` at
        every physical release -- the R3 self-toss run's measured-plant
        bias (:func:`_measured_release_bias`); the R2 columns gate passes
        ``None`` and its own exact/noisy-tracking behaviour is unchanged.
        ``final_removal`` (``(t_land_final, ball_final)``) is the columns-
        only "remove the last unscheduled ball" step; ``None`` skips it.
        ``stop_check()``, when given, ends the loop as soon as it returns
        True (in addition to ``t_end``) -- the self-toss run's
        ``executor.done``, so a refused or dropped attempt does not stream
        out a whole ``t_end`` budget it no longer needs.

        Returns ``{'makes', 'drops', 'emitted', 'accepted', 'pump_rejects',
        'flags_seen' (a set), 'worst_leg', 'worst_hand', 'ticks', 'wall_s'}``.
        """
        emitter = KnotEmitter(geom)
        pump = stream_chain.make_pump()
        mirror = stream_chain.make_mirror(geom)
        next_frame_t = plant.data.time
        frame_seq = 0
        frame_t0 = None
        latch_tau = 0.0
        emitted = accepted = 0
        flags_seen = set()
        worst_leg = worst_hand = 0.0
        ticks = 0
        makes = 0
        drops = 0
        removed_final = final_removal is None
        t_wall = time.time()

        while plant.data.time < t_end and (stop_check is None
                                           or not stop_check()):
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
                    float(st.hand_pos_mm), rcfg)
                cmd_pos, _cmd_vel, _ = mirror.tick(t, fb_rev)
                hand_out = mirror.tick_hand(t, fb_hand)
                plant.command(np.asarray(cmd_pos) / self.mm_to_rev)
                if hand_out is not None:
                    plant.command_hand(
                        stream_chain.slider_mm_of_rev(hand_out[0], rcfg))

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
                bstate = ball_state[b_id]
                if plant.has_ball and plant.get_ball_state(b_id).held:
                    _, v_noisy = noise.perturb_throw(
                        site_mm, vel_mm_s, target_mm - site_mm)
                    if release_bias is not None:
                        v_noisy = release_bias(v_noisy)
                    plant.release_ball(v_noisy, ball=b_id)
                bstate['expect_held'] = False
                bstate['lost_since'] = None
                bstate['airborne'] = True
                bstate['estimator'].reset()
                bstate['next_obs_t'] = t

            for b, bstate in ball_state.items():
                if plant.check_and_capture(b):
                    makes += 1
                    bstate['expect_held'] = True
                    bstate['airborne'] = False
                if bstate['expect_held']:
                    bs = plant.get_ball_state(b)
                    if not bs.held:
                        if bstate['lost_since'] is None:
                            bstate['lost_since'] = t
                            drops += 1
                    else:
                        bstate['lost_since'] = None
                if (bstate['airborne'] and bstate['next_obs_t'] is not None
                        and t >= bstate['next_obs_t']):
                    bs = plant.get_ball_state(b)
                    noisy = noise.observe(bs.position_mm)
                    bstate['estimator'].add(t, noisy)
                    bstate['last_obs_t'] = t
                    bstate['next_obs_t'] += OBS_PERIOD_S

            if not removed_final and t >= final_removal[0]:
                plant.ball_manager.ball(final_removal[1]).reset()
                ball_state[final_removal[1]]['airborne'] = False
                ball_state[final_removal[1]]['expect_held'] = False
                removed_final = True

            plant.step(TICK_S)
            ticks += 1
            if self.viewer is not None:
                self.viewer.sync()

        return dict(makes=makes, drops=drops, emitted=emitted,
                   accepted=accepted, pump_rejects=int(pump.frames_rejected),
                   flags_seen=flags_seen, worst_leg=worst_leg,
                   worst_hand=worst_hand, ticks=ticks,
                   wall_s=time.time() - t_wall)

    # ── R3: the self-toss learner run (plan § 4 R3 "Sim validation") ───────
    #
    # A ``SkillsGate`` used for these methods must be constructed with a
    # ``SelfTossGateConfig`` (``self.cfg``), never a ``SkillsGateConfig`` --
    # ``run_learn`` below is the only constructor, and it never shares an
    # instance with ``run`` / ``run_trial``.

    def run_self_toss_attempt(self, *, site, n_throws: int, memory: mem.Memory,
                              learner_cfg: lr.LearnerConfig, boxes: list,
                              noise: JuggleNoise, throws_out: list
                              ) -> Tuple[str, dict, '_InstallCtx']:
        """One self-toss attempt, cold from the ACTIVATE park (owner decision
        6, "resets are cheap"): reset + spawn, ``schedule.compile_self_toss``,
        stream it through the one chain (:meth:`_stream_chain`) with the R3
        learner/box/observer/observations wired in.  ``throws_out`` accumulates
        every finalised :class:`~jugglebot.motion.skills.memory.Experience`
        this attempt produces (in schedule order); the memory itself is
        appended to as a side effect of ``on_experience`` (below), so it
        carries forward to the NEXT attempt even though the plant does not.

        Returns ``(end_code, loop_stats, ictx)`` -- ``end_code`` is ``''`` for
        an attempt that ran its whole schedule with no refusal.
        """
        cfg: SelfTossGateConfig = self.cfg
        plant = self.plant
        geom = self.geom
        park = _activate_park_state(self.rcfg, site)
        pose0 = np.asarray(park.pose, dtype=float)
        plant.reset(pose0)
        plant.command(plant.pose_to_extensions(pose0))
        plant.command_hand(stream_chain.slider_mm_of_rev(0.0, self.rcfg))
        for _ in range(40):
            plant.step(KNOT_DT_S)
            if self.viewer is not None:
                self.viewer.sync()
        plant.ball_manager.ball(0).spawn_in_hand()
        t0_abs_s = float(plant.data.time)

        pattern = sk.SelfTossPattern(site=site, apex_m=cfg.apex_m,
                                     dwell_s=cfg.dwell_s, n_throws=n_throws)
        sched = sk.compile_self_toss(pattern, t0_abs_s)

        ball_state = {0: dict(estimator=BallisticEstimator(bal.G_VEC_MMS2),
                             airborne=False, next_obs_t=None,
                             expect_held=True, lost_since=None,
                             last_obs_t=None)}
        tracker = _make_tracker(plant, ball_state)
        ictx = _InstallCtx(park)
        installer = _make_installer(ictx, self.seg_cfg, self.limits, geom)
        observer = _make_observer(plant)
        observations = _make_observations(observer)
        learner = _MemoryLearner(memory, learner_cfg)
        executor = ex.SkillExecutor(
            sched, installer, tracker=tracker, learner=learner, boxes=boxes,
            observer=observer, on_experience=lambda exp: (
                memory.append(exp), throws_out.append(exp)),
            observations=observations)

        # A generous, bounded timeout -- the real exit is ``executor.done``
        # (``stop_check``), so a refused or dropped attempt does not stream
        # out the whole schedule's worth of quiet tail it no longer needs.
        t_end = float(sched.skills[-1].t_abs_s) + 1.0
        loop = self._stream_chain(
            plant=plant, geom=geom, rcfg=self.rcfg, noise=noise,
            executor=executor, ictx=ictx, ball_state=ball_state, t_end=t_end,
            release_bias=_measured_release_bias,
            stop_check=lambda: executor.done)
        return str(executor.end_code), loop, ictx

    def run_self_toss_seed(self, seed: int, policy: str = 'A') -> dict:
        """One seed's whole R3 learner run: repeated cheap-reset attempts,
        collecting throws until ``cfg.target_throws`` releases have been
        observed (plan § 4 R3 / owner decision 2026-09-13).

        Policy **A**: single-throw attempts (``n_throws=1``, THROW -> CATCH ->
        REST) until the memory holds ``learner_cfg.k_min`` rows, then chained.
        Policy **B**: chained from the very first throw.  Both retain the
        memory across every reset -- only the plant and the executor are cold
        each attempt.
        """
        if policy not in ('A', 'B'):
            raise ValueError("policy must be 'A' or 'B', got %r" % (policy,))
        cfg: SelfTossGateConfig = self.cfg
        site = sites.columns_sites(_SELF_TOSS_SEPARATION_MM)[0]
        boxes = _load_admissible_boxes(cfg.admissible_box_path)
        noise = JuggleNoise(cfg.noise, seed=seed)
        tmp_dir = tempfile.mkdtemp(prefix='skills_gate_learn_seed%d_' % seed)
        memory = mem.Memory(os.path.join(tmp_dir, 'memory.csv'))
        assert len(memory) == 0, 'a fresh tmp path must be a cold memory'

        throws: list = []
        attempts = 0
        refusals = 0
        drops_total = 0
        makes_total = 0
        installs_total = installs_accepted = 0
        end_codes = []
        t_wall0 = time.time()

        while len(throws) < cfg.target_throws and attempts < cfg.max_attempts:
            remaining = cfg.target_throws - len(throws)
            if policy == 'A' and len(memory) < cfg.learner_cfg.k_min:
                n_throws = 1
            else:
                n_throws = max(1, remaining)
            attempts += 1
            end_code, loop, ictx = self.run_self_toss_attempt(
                site=site, n_throws=n_throws, memory=memory,
                learner_cfg=cfg.learner_cfg, boxes=boxes, noise=noise,
                throws_out=throws)
            end_codes.append(end_code)
            if end_code:
                refusals += 1
            drops_total += loop['drops']
            makes_total += loop['makes']
            installs_total += ictx.installs_total
            installs_accepted += ictx.installs_accepted

        t_f_nominal = sk.flight_s(cfg.apex_m)
        rows = []
        for i, exp in enumerate(throws):
            err_xy_mm = float(1000.0 * np.linalg.norm(exp.y[:2]))
            err_flight_ms = float(1000.0 * abs(float(exp.y[2]) - t_f_nominal))
            rows.append(dict(
                throw=i + 1, u=exp.u.tolist(), y=exp.y.tolist(),
                err_xy_mm=err_xy_mm, err_flight_ms=err_flight_ms,
                caught=bool(exp.caught), t_abs_s=float(exp.t_abs_s)))

        throws_to_band_xy = next(
            (r['throw'] for r in rows if r['err_xy_mm'] <= cfg.xy_band_mm),
            None)
        throws_to_band_flight = next(
            (r['throw'] for r in rows
             if r['err_flight_ms'] <= cfg.flight_band_s * 1000.0), None)
        entered_band = bool(
            throws_to_band_xy is not None
            and throws_to_band_xy <= cfg.band_entry_throws
            and throws_to_band_flight is not None
            and throws_to_band_flight <= cfg.band_entry_throws)

        monotone_xy = _monotone_verdict(
            [r['err_xy_mm'] for r in rows], cfg.band_entry_throws,
            cfg.target_throws)
        monotone_flight = _monotone_verdict(
            [r['err_flight_ms'] for r in rows], cfg.band_entry_throws,
            cfg.target_throws)

        return dict(
            seed=seed, policy=policy, throws=rows,
            n_throws_collected=len(rows), attempts=attempts,
            end_codes=end_codes, refusals=refusals, drops=drops_total,
            makes=makes_total, installs_total=installs_total,
            installs_accepted=installs_accepted,
            throws_to_band_xy=throws_to_band_xy,
            throws_to_band_flight=throws_to_band_flight,
            entered_band=entered_band,
            monotone_xy=monotone_xy, monotone_flight=monotone_flight,
            passed=bool(entered_band and bool(monotone_xy)
                       and bool(monotone_flight)),
            wall_s=time.time() - t_wall0)

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


def run_learn(cfg: SelfTossGateConfig = None, seeds=(0, 1, 2, 3, 4),
             policy: str = 'A') -> dict:
    """The R3 sim-validation run (plan § 4 R3 "Sim validation"): one
    ``SkillsGate`` built for the self-toss operating point, one seed at a
    time, writing the report JSON the brief this rung was built from asks
    for. Never shares the gate instance with :func:`run_gate` (columns)."""
    cfg = SelfTossGateConfig() if cfg is None else cfg
    gate = SkillsGate(cfg)
    t0 = time.time()
    seed_results = [gate.run_self_toss_seed(s, policy=policy) for s in seeds]
    wall_s = time.time() - t0
    report = {
        'gate': 'skills_learn',
        'policy': policy,
        'passed': bool(seed_results and all(r['passed'] for r in seed_results)),
        'seeds': list(seeds),
        'apex_m': cfg.apex_m,
        'dwell_s': cfg.dwell_s,
        'xy_band_mm': cfg.xy_band_mm,
        'flight_band_s': cfg.flight_band_s,
        'band_entry_throws': cfg.band_entry_throws,
        'target_throws': cfg.target_throws,
        'wall_s': wall_s,
        'seed_results': seed_results,
    }
    path = cfg.report_path
    if path is None:
        out_dir = os.path.join(_repo_root, 'temp', 'reports')
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(
            out_dir, 'skills_gate_learn_%s_%s.json'
            % (policy, time.strftime('%Y%m%dT%H%M%S')))
    with open(path, 'w') as fh:
        json.dump(report, fh, indent=2)
    report['report_path'] = path
    return report


def _print_learn_table(rep: dict) -> None:
    print('[skills_gate_learn] policy %s  apex %.2f m  dwell %.2f s  band '
          '%.0f mm / %.0f ms  entry<=%d throws  window<=%d throws'
          % (rep['policy'], rep['apex_m'], rep['dwell_s'], rep['xy_band_mm'],
             rep['flight_band_s'] * 1000.0, rep['band_entry_throws'],
             rep['target_throws']))
    print('[skills_gate_learn] %-6s %-8s %-10s %-10s %-9s %-9s %-9s %-6s %-6s'
          % ('seed', 'verdict', 'band_xy', 'band_flt', 'mono_xy', 'mono_flt',
             'attempts', 'drops', 'makes'))
    for r in rep['seed_results']:
        verdict = 'PASS' if r['passed'] else 'FAIL'
        print('[skills_gate_learn] %-6d %-8s %-10s %-10s %-9s %-9s %-9d %-6d %-6d'
              % (r['seed'], verdict, r['throws_to_band_xy'],
                 r['throws_to_band_flight'], r['monotone_xy'],
                 r['monotone_flight'], r['attempts'], r['drops'], r['makes']))
    print('[skills_gate_learn] %s  (wall %.1f s over %d seed(s))'
          % ('PASS' if rep['passed'] else 'FAIL', rep['wall_s'],
             len(rep['seeds'])))


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
    p.add_argument('--learn', action='store_true',
                   help='run the R3 sim-validation learner run (plan § 4 R3) '
                        'instead of the columns gate')
    p.add_argument('--policy', choices=('A', 'B'), default='A',
                   help='--learn only: A = single-throw attempts until '
                        'k_min rows then chained (the gated policy); B = '
                        'chained from the first throw (reported alongside)')
    args = p.parse_args(argv)

    if args.learn:
        lcfg = SelfTossGateConfig(report_path=args.report)
        if args.seeds is not None:
            seeds = tuple(args.seeds)
        else:
            seeds = (0, 1, 2, 3, 4)
        rep = run_learn(lcfg, seeds=seeds, policy=args.policy)
        _print_learn_table(rep)
        return 0 if rep['passed'] else 1

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
