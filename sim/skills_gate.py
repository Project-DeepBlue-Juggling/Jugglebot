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
prior (``schedule.compile_columns``'s ``y_d=(zeros(2), apex_m)``), so
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
land than beside the loop it drives. **R4 (Unit U7a)** generalises that SAME
learner run to a hop (``SelfTossGateConfig.pattern='hop'``, two sites
P1<->P2, :func:`_sites_for`) rather than adding a THIRD copy: the site
tuple and the ``pattern`` key are now parameters of
``run_self_toss_attempt``/``run_self_toss_seed``/``run_learn`` instead of a
single site hardcoded in each, which is a net SMALLER footprint than one
more near-duplicate run_*_seed method would have been (~35 net new lines
for the generalisation + the shared ``_wall_ms_stats`` percentile helper,
against several hundred for a parallel hop-only copy of the loop).
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
from types import SimpleNamespace

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
    FLAG_HAS_HAND, FLAG_HAS_U1, FLAG_HAS_U2, FLAG_HAS_V1, FLAG_HAS_V2,
)

from sim.plant.mujoco_plant import MuJoCoPlant                     # noqa: E402
from sim.juggle_noise import BallisticEstimator, JuggleNoise, NoiseConfig  # noqa: E402

#: ``skill_node._DISPATCH_LOOKAHEAD_S``, mirrored (2026-10-02): the node's
#: executors dispatch this far ahead of their tick (one 40 Hz orchestrator
#: tick + trajectory_node's ceil-snap of ``t_now`` onto the knot grid), so a
#: reserved fresh origin (``install_segment(..., reserve_fresh_lead=True)``)
#: never plans under the schedule's ``window_s``. Restated rather than
#: imported: ``skill_node`` pulls in rclpy, which this gate must not need.
_DISPATCH_LOOKAHEAD_S = 1.0 / 40.0 + float(hw.JB_TRAJ_KNOT_DT_S)
from sim import stream_chain                                       # noqa: E402
from sim.gate_common import ViewerClosed, attach_viewer             # noqa: E402

# ---------------------------------------------------------------------------
# Session limits -- read out of THIS SOURCE by regex
# (tests/ros/test_unified_cycle_bench.py::
#  test_the_session_limits_match_the_gates_that_planned_them), so this triple
# is the ONE place the R2 operating point's leg limits live for that bench.
# ---------------------------------------------------------------------------
# 300 -> 350 on 2026-10-04 (R5 sitting 3): the roomier columns geometry (apex
# 0.95, dwell 0.25, separation 125) is swept and flown at 350 mm/s; at 300 the
# one-ball trial's steady catch refused LIMIT_VEL at 300.2 mm/s.
_SESSION_LEG_VEL_MMPS = 350.0
_SESSION_LEG_ACC_MMPS2 = 5000.0
_SESSION_LEG_JERK_MMPS3 = 200000.0
# Owner, 2026-10-02: 3900 (was 3500). The fed columns rehearsal's same-site
# catch-and-throw fold peaked at 101 % of 3500 in BOTH feed layouts (the
# 2026-09-30 pass was the same knife edge landing on the right side), so the
# session cap moved to the YAML ceiling; config/hardware_config.yaml
# `hand_acc_limit_rps2` is 3900 since the same day and the box is re-swept at it.
_SESSION_HAND_ACC_RPS2 = 3900.0

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

#: HAS_V2 since C2FF (2026-09-14): the emitter sends the exact u2-knot velocities
#: on every frame. HAS_SCHED stays clear here -- the sim loop emits no knot
#: stamp, and the firmware mirror plays arrival-phase (legacy) frames.
_WANT_FLAGS = (FLAG_HAS_U1 | FLAG_HAS_U2 | FLAG_HAS_HAND | FLAG_HAS_V1
               | FLAG_HAS_V2)

#: How close (mm, xy) the platform's emitted cup opening must sit to the
#: columns Stop's last ball's landing xy at the instant it lands (owner
#: decision D2, 2026-09-30, R5 rescope, E1'): "the platform moves under the
#: landing ... so it strikes the held ball and comes to rest on the
#: platform". Both are the SAME site's xy by construction
#: (``sites.Site.throw_site_mm``/``catch_site_mm``/``rest_site_mm`` all share
#: one xy per site — only z differs by role), so this is a physical-chain
#: sanity check (real MuJoCo plant position vs. the schedule's own aim), not
#: a tight tolerance search: 25 mm is generous against the mm-scale mirror/
#: IK-curvature residuals this gate already bounds (``MIRROR_TOL_LEG_REV``
#: etc.) and the sub-mm xy noise the release model carries at R2/R5.
FINAL_CUP_XY_TOL_MM = 25.0


def _columns_final_removal(sched: 'sk.Schedule'):
    """The columns Stop's own last-ball bookkeeping (owner decision D2,
    ``schedule.compile_columns``'s cross-site shadow throw).

    Scans ``sched.skills`` for the ONE skill whose ``then_throw`` carries
    ``shadow_landing`` (set only for ``n_throws >= 2`` — see
    ``compile_columns``'s docstring: the Stop throw is always folded into a
    catch-with-throw, because every throw index that can be the Stop
    (``n - 1 >= 1``) always has a same-ball catch exactly one dwell before it
    to fold with — there is no standalone-THROW case to handle here).

    Returns ``(t_land_final_s, ball_final, target_xy_mm)``:

    * ``t_land_final_s`` — the wall-clock instant the shadow-landed ball
      reaches the catch plane: the carried release instant plus the
      schedule's OWN flight time (``sched.flight_s`` — the same number
      every throw in this schedule uses, since a columns throw's duration is
      the symmetric-height ballistic time and does not depend on which site
      it is aimed at, only on the pattern's apex).
    * ``ball_final`` — which MuJoCo ball index this is (the shadow throw's
      own ``ball_id``).
    * ``target_xy_mm`` — the xy (mm) it is aimed at: the OTHER site's
      ``catch_site_mm()[:2]`` (the site the ball it strikes is already
      resting at).

    Read off the schedule's own skills rather than re-derived from
    ``beta``/``n_throws`` arithmetic, so this stays correct however
    ``compile_columns`` schedules the Stop (plan § 0: one place this fact is
    computed). ``None`` when no skill carries the flag (``n_throws == 1``:
    nothing is held yet, so there is no Stop to check)."""
    for skill in sched.skills:
        tt = skill.then_throw
        if tt is not None and tt.shadow_landing:
            t_land_final_s = float(tt.t_release_abs_s) + float(sched.flight_s)
            target_xy_mm = np.asarray(tt.target.catch_site_mm(),
                                      dtype=float)[:2].copy()
            return t_land_final_s, int(skill.ball_id), target_xy_mm
    return None


# ---------------------------------------------------------------------------
# Config + result records
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class SkillsGateConfig:
    apex_m: float = 0.9
    separation_mm: float = 100.0
    dwell_s: float = 0.27
    n_throws: int = 20
    seeds: tuple = (0, 1, 2, 3, 4)
    leg_vel_mmps: float = _SESSION_LEG_VEL_MMPS
    leg_acc_mmps2: float = _SESSION_LEG_ACC_MMPS2
    leg_jerk_mmps3: float = _SESSION_LEG_JERK_MMPS3
    hand_acc_rps2: float = _SESSION_HAND_ACC_RPS2
    #: Which aim source the executor runs with (``executor.AIM_SOURCES``).
    #: ``tracker`` here, not the LIVE default (``schedule``): this gate's
    #: whole tracker-refine surface (``_make_tracker``, the RESEND
    #: assertions) only exists on that path, and an open-loop run is
    #: requested explicitly with ``--catch-aim-source schedule``.
    catch_aim_source: str = ex.AIM_TRACKER
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
    #: D2/E1' Stop check: ``|emitted cup xy - last ball's landing xy|`` (mm)
    #: at ``t_land_final`` — ``nan`` when there was no Stop to check
    #: (``n_throws == 1``). See :data:`FINAL_CUP_XY_TOL_MM`.
    final_cup_err_mm: float = float('nan')

    pump_clean: bool = False
    mirror_ok: bool = False
    flags_ok: bool = False
    all_installed: bool = False
    caught_all: bool = False
    no_drops: bool = False
    #: D2/E1': True when ``final_cup_err_mm <= FINAL_CUP_XY_TOL_MM`` (or
    #: vacuously True when there was no Stop to check).
    final_cup_ok: bool = False
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


#: The sample count below which this gate's tracker reports NOTHING — the
#: admission rule of the robot tracker it stands in for
#: (``tracking/flight_fit.BallisticFit``: ``min_samples=12`` surviving samples
#: spanning at least ``min_span_s=0.050``; at this gate's ``OBS_PERIOD_S``
#: of 5 ms, 12 samples ARE 55 ms of span, so one number carries both).
#:
#: It was 3 until 2026-09-21, and 3 samples is 10 ms of baseline: over the
#: 0.86 s flight of a 0.9 m self-toss that extrapolates the 0.5 mm
#: observation noise into a landing tens to >100 mm from the truth, and the
#: executor's live catch re-aim (``_resend_live_catch``) fires on it ~12 ms
#: after release. Measured (seed 0, policy A, 2026-09-21): the n=3 fit of the
#: chain's last throw put the landing at (27.5, -69.2) mm against a true
#: (-54.6, 8.2) mm, the re-aim was ACCEPTED, the NEXT tick's correction back
#: was REFUSED (``LIMIT_JERK``, peak leg jerk 481 688 mm/s³ — the banking-
#: saturation class of ``logbook/2026-09-16-banking-saturates-on-small-
#: lateral-offsets.md``) and the re-aim cap was then spent, so the cup dived
#: to the bogus aim: 113.4 mm from the ball laterally at the crossing and
#: only 1.3 mm off in z — a clean lateral miss, reported as ``caught=False``.
#: The robot cannot fail this way twice over: its fit refuses to exist below
#: 12 samples, and ``skill_node``'s ``learner_lateral_authority_mm`` (0
#: before 2026-09-21, 40 by launch default since) pins a tracker aim's
#: lateral to the schedule site when clamped
#: (``SkillExecutor._clamp_lateral_to_schedule``) — an authority this gate
#: deliberately leaves unset regardless, so the sim learner keeps its xy
#: correction.
_FIT_MIN_SAMPLES = 12


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
    refused, read the correct ~1.11 (the plant bias alone). (That 94 ms was the OLD window: ``CAUGHT_WINDOW_S`` became 0.35 s anchored on the OBSERVED landing on 2026-09-16, so the interval this drift accumulates over is now LONGER and the anchoring fix matters more, not less.)
    """

    def tracker(ball_id):
        bstate = ball_state[ball_id]
        if not bstate['airborne']:
            return None
        est = bstate['estimator']
        if est.n < _FIT_MIN_SAMPLES:
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
        # ``from_fit``: this estimator IS a batch parabola over the flight's
        # samples (the sim's stand-in for ``tracking/flight_fit.py``), which
        # is the class of estimate the learner accepts as an outcome.
        return ex.Landing(pos_mm=np.asarray(pos, dtype=float),
                          vel_mm_s=np.asarray(vel, dtype=float),
                          t_land_abs_s=float(t_ref) + float(t_rem),
                          from_fit=True)
    return tracker


class _AnnouncedStamp:
    """A ``builtin_interfaces/Time``-shaped stand-in (``.sec``/``.nanosec``)
    for one track's announced ``throw_time`` -- the ONLY thing
    ``ball_possession.ball_throw_time_s``/``match_announced_track`` read off
    a synthetic ``/balls`` record. Mirrors ``tests/ros/
    test_two_ball_association.py``'s ``_T`` so both layers build the same
    shape from the same float."""

    def __init__(self, t_s: float):
        self.sec = int(math.floor(t_s))
        self.nanosec = int(round((t_s - self.sec) * 1e9))


#: Mirrors the real tracker's terminal-track retention (``ball_possession``
#: module docstring: "the ~2 s the terminal track is retained") -- how long
#: a CAUGHT track still appears in the synthetic ``/balls`` snapshot fed to
#: the correlator, so the negative control (``legacy=True``) sees the same
#: stale-track overlap the 2026-10-02 mis-latch fed on.
_TRACK_RETENTION_S = 2.0


class _IdentityTracker:
    """Mints one tracker id per ANNOUNCED release -- never per schedule/
    physical ball -- and resolves every ``tracker(ball_id)`` read through
    the REAL correlator (``ball_possession.advance_correlation`` /
    ``flight_in_progress``), so a mis-association in THAT rule fails this
    gate the way it failed the 2026-10-02 sitting (U1, "The release
    identity"). Before this class, ``_make_tracker`` answered ``tracker
    (ball_id)`` straight off ground truth (``ball_state[ball_id]``) -- the
    association step the robot actually does never ran, so no seed could
    reproduce the mis-latch (``u1_two_ball_association.md`` § 3).

    ``physical_tracker(phys_ball_id) -> Optional[ex.Landing]`` is
    :func:`_make_tracker`'s own closure, reused verbatim -- this class only
    decides WHICH physical ball's fit answers WHICH schedule ball's read;
    the fit itself is not duplicated here.

    ``legacy=True`` is the negative control (owner ask, handoff § "What
    would make the gate faithful"): announcements carry no ``thrower``, so
    every latch resolves through the PRE-2026-10-04 exclusion rule
    (``advance_flight_latches``, one claimed set PER schedule ball) instead
    of the identity rule -- this must reproduce WINDOW_TOO_SHORT on fed
    columns, proving the gate can see the class the fix closes.
    """

    #: Lifecycle ints, matching ``jugglebot_interfaces/msg/BallState`` and
    #: ``tests/ros/test_two_ball_association.py``'s ``TBT, FLY, CAUGHT`` /
    #: ``ANN, CONF`` -- not load-bearing here (``ball_possession`` takes
    #: these as parameters, not literals), kept equal for readability only.
    TBT, FLY, CAUGHT = 0, 1, 2
    ANNOUNCED, CONFIRMED = 0, 1

    def __init__(self, physical_tracker, *, legacy: bool = False):
        self._physical_tracker = physical_tracker
        self._legacy = bool(legacy)
        self._next_id = 1
        self._tracks: dict = {}            # tid -> mutable state dict
        self.correlation: dict = {}        # schedule ball_id -> (FlightLatch,...)
        self._prev_records: list = []
        self._records_cache: list = []
        self._now_s = 0.0

    def announce(self, ball_id: int, t_release_s: float, *,
                 thrower: str = 'jugglebot') -> int:
        """Mint a new track for ONE announced release of schedule ``ball_id``
        and queue its correlation latch -- the sim's stand-in for
        ``skill_node._maybe_announce`` (our own throws) / ``_install_
        announced_reload`` (a Ball Butler feed), called at ACCEPTED INSTALL
        (a THROW, or a CATCH carrying a ``then_throw``), never on a re-send
        of the same release (:func:`_make_installer`'s ``on_release`` fires
        once per accepted ``(ball_id, t_release_s)`` pair -- a RE-send
        supersedes its own pending release, it does not re-announce it)."""
        tid = self._next_id
        self._next_id += 1
        self._tracks[tid] = dict(phys=int(ball_id), status=self.TBT,
                                 tracking=self.ANNOUNCED, caught_at=None,
                                 seen_airborne=False)
        preexisting = tuple(sorted(r.id for r in self._prev_records
                                   if r.status == self.FLY))
        latch = bp.FlightLatch(
            t_release_s=float(t_release_s), preexisting=preexisting,
            thrower=(None if self._legacy else bp.announced_source(thrower)))
        self._tracks[tid]['throw_time'] = float(t_release_s)
        self._tracks[tid]['source'] = bp.announced_source(thrower)
        self.correlation[int(ball_id)] = (
            tuple(self.correlation.get(int(ball_id), ())) + (latch,))
        return tid

    def _build_records(self) -> list:
        return [SimpleNamespace(
                    id=tid, status=tr['status'], tracking=tr['tracking'],
                    source=tr['source'], destination='jugglebot',
                    throw_time=_AnnouncedStamp(tr['throw_time']))
                for tid, tr in self._tracks.items()]

    def tick(self, now_s: float, ball_state: dict) -> None:
        """Advance every track's lifecycle from ``ball_state`` (the sim's
        ground truth: TBT -> IN_FLIGHT at its own announced ``throw_time``,
        ANNOUNCED -> CONFIRMED once that flight's estimator has a fit,
        IN_FLIGHT -> CAUGHT once the physical ball is no longer airborne),
        drop tracks CAUGHT longer than :data:`_TRACK_RETENTION_S`, then
        refine the correlation against the resulting snapshot.

        Call ONCE per sim tick, BEFORE ``executor.tick`` -- the correlator
        must see this instant's snapshot before the executor's own dispatch
        check reads :meth:`tracker`, exactly as a real ``/balls`` message
        precedes the node's dispatch on the same clock."""
        now_s = float(now_s)
        expired = []
        for tid, tr in self._tracks.items():
            if tr['status'] == self.TBT and now_s >= tr['throw_time'] - 1e-9:
                tr['status'] = self.FLY
            if tr['status'] == self.FLY:
                bstate = ball_state.get(tr['phys'])
                if bstate is not None:
                    if (tr['tracking'] != self.CONFIRMED
                            and bstate['estimator'].n >= _FIT_MIN_SAMPLES):
                        tr['tracking'] = self.CONFIRMED
                    # ``seen_airborne`` gates the CAUGHT transition on having
                    # actually observed flight at least once -- NOT merely
                    # "not airborne now". The physical release (``_stream_
                    # chain``'s ``pending_releases`` pop, which sets
                    # ``bstate['airborne'] = True``) and this track's own
                    # TBT -> FLY flip key on the SAME announced instant, but
                    # float accumulation over hundreds of 2 ms ticks can put
                    # them one tick apart (root-caused 2026-10-04 B1, probe
                    # ``probe_assoc6.py``: every track read "not airborne"
                    # on the tick it was minted FLY -- the ball had not
                    # physically left the hand yet -- and was immediately
                    # marked CAUGHT, so ``flight_in_progress`` never saw a
                    # single FLY/CONFIRMED track and NO run produced any
                    # learner row at all). Requiring one TRUE sighting first
                    # removes the race regardless of tick alignment.
                    if bstate['airborne']:
                        tr['seen_airborne'] = True
                    elif tr['seen_airborne']:
                        tr['status'] = self.CAUGHT
                        tr['caught_at'] = now_s
            if (tr['status'] == self.CAUGHT and tr['caught_at'] is not None
                    and now_s - tr['caught_at'] > _TRACK_RETENTION_S):
                expired.append(tid)
        for tid in expired:
            del self._tracks[tid]

        records = self._build_records()
        if self._legacy:
            self.correlation = {
                k: bp.advance_flight_latches(
                    records, robot_name='jugglebot', latches=v, now_s=now_s,
                    in_flight_status=self.FLY)
                for k, v in self.correlation.items()}
        else:
            self.correlation = bp.advance_correlation(
                records, robot_name='jugglebot', correlation=self.correlation,
                now_s=now_s, in_flight_status=self.FLY)
        self._prev_records = records
        self._records_cache = records
        self._now_s = now_s

    def tracker(self, ball_id: int):
        """``tracker(ball_id) -> Optional[ex.Landing]`` for ``SkillExecutor``
        -- the sim's stand-in for ``skill_node._tracker``: resolve the
        flight in progress through the real correlator, then answer with
        THAT track's bound physical ball's fit (never ground truth keyed
        directly on ``ball_id``)."""
        latches = self.correlation.get(int(ball_id), ())
        tid = bp.flight_in_progress(
            self._records_cache, latches=latches, now_s=self._now_s,
            in_flight_status=self.FLY, confirmed_tracking=self.CONFIRMED)
        if tid is None:
            return None
        tr = self._tracks.get(tid)
        if tr is None:
            return None
        return self._physical_tracker(tr['phys'])


def _make_installer(ictx: _InstallCtx, seg_cfg, limits, geom, *,
                    on_release=None):
    """The one ``installer(kind, terminal, t_now_s, *, ball_id) ->
    InstallResult`` closure — ``ex.install_segment`` over ``ictx``'s live
    record, plus the pending-release bookkeeping every caller needs (a THROW
    or a CATCH's carried ``then_throw`` schedules the physical release the
    stream loop later pops).  Factored out of ``run_trial`` for the same
    reason as :func:`_make_tracker`: the R3 self-toss run needs the identical
    policy, not a second copy of it.

    ``on_release(ball_id, t_release_s)``, when given, fires ONCE per
    newly-scheduled ``(ball_id, t_release_s)`` pair, at the SAME accepted
    install this function already treats as the release's own announcement
    (``skill_node._maybe_announce`` runs at accepted install too, 0.6-0.8 s
    before the physical release) -- never on a re-send of the same release
    (a re-send carries the identical ``t_release_s``, only a different
    aim). :class:`_IdentityTracker` binds this to ``announce``."""
    announced: set = set()

    def installer(kind, terminal, t_now_s, *, ball_id):
        new_record, result, seg = ex.install_segment(
            ictx.record, ictx.seed_rest, kind, terminal, t_now_s,
            cfg=seg_cfg, limits=limits, geom=geom, warm_start=ictx.warm_start,
            # trajectory_node's own rule (2026-10-02): `t_now_s` is the
            # executor's dispatch tick, so an event-bearing fresh origin
            # starts LEAD_S after it -- the window the robot plans, not
            # window + lead. The stream loop holds knot 0 until that origin
            # exactly as the robot's emitter does.
            reserve_fresh_lead=True)
        ictx.installs_total += 1
        if result.accepted:
            ictx.record = new_record
            ictx.seed_rest = None
            ictx.warm_start = seg.warm_start
            ictx.installs_accepted += 1
            ictx.plan_wall_s.append(float(result.plan_wall_s))
            src = (terminal if kind == THROW
                   else terminal.then_throw
                   if kind == CATCH and terminal.then_throw is not None
                   else None)
            if src is not None:
                # A re-send SUPERSEDES its own pending release: the same
                # (ball, release instant) may be installed up to 1 +
                # resend_max times, and only the LAST terminal is what the
                # machine will do. Appending kept the FIRST dispatch's
                # takeoff (planned for a release at the site) and released
                # the ball with it from the re-aimed cup, while the re-sent
                # takeoffs -- which carry the lateral fly-back a release off
                # the site needs (executor._catch_terminal, 2026-09-28) --
                # sat behind it unused (`held` already False). Harmless while
                # every release was at the site (identical takeoffs); it
                # walked the 2026-09-28 learner gate 50 mm off the site once
                # the release followed the catch.
                t_rel = float(src.t_release_s)
                ictx.pending_releases = [
                    pr for pr in ictx.pending_releases
                    if not (pr[1] == ball_id and abs(pr[0] - t_rel) < 1e-9)]
                ictx.pending_releases.append((
                    t_rel, ball_id,
                    np.asarray(seg.takeoff_vel_mm_s, dtype=float),
                    np.asarray(src.site_mm, dtype=float),
                    np.asarray(src.target_mm, dtype=float)))
                ictx.pending_releases.sort(key=lambda pr: pr[0])
                if on_release is not None:
                    key = (int(ball_id), round(t_rel, 6))
                    if key not in announced:
                        announced.add(key)
                        on_release(int(ball_id), t_rel)
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

#: The R4 hop operating point's separation -- P1/P2 = (+-125, 0) mm, matching
#: the 'hop' boxes swept into ``config/generated/admissible_box.yaml`` (site
#: pair ``(P1, P2)``/``(P2, P1)``, ``release_site_xy_mm``/``target_site_xy_mm``
#: stamped at exactly +-125.0 mm -- ``admissible.select`` refuses on a
#: mismatch, so this constant and the sweep's must agree to the mm).
_HOP_SEPARATION_MM = 250.0

#: Ball Butler's horizontal bearing at the feed site (R5 sitting 2,
#: 2026-10-02: not necessarily site 1 -- see `run_columns_attempt`'s own
#: swap), reused for the R5 columns
#: FEED trial (:meth:`SkillsGate.run_columns_attempt`'s ``feed_angle_deg``/
#: ``feed_speed_mmps`` branch): the (x, y) direction of the F-a/F-b probes'
#: reused real arrival vector ``(1058.0, 475.0, -5507.0)`` mm/s
#: (``scratchpad/probe_bbfed_columns.py``, ``bbfed_columns_probe.md``, R5
#: 2026-09-30 -- ~11.9 deg off vertical at that speed), so a FEED run's
#: synthetic arrival points the same way the bench's real Ball Butler does
#: instead of an arbitrary axis. Unit vector of the horizontal components
#: only; the trial scales it by ``feed_speed_mmps * sin(feed_angle_deg)``
#: and combines it with a vertical component
#: ``-feed_speed_mmps * cos(feed_angle_deg)`` (arrival is downward).
_COLUMNS_FEED_BEARING_XY = np.array([1058.0, 475.0])
_COLUMNS_FEED_BEARING_XY_UNIT = (
    _COLUMNS_FEED_BEARING_XY / np.linalg.norm(_COLUMNS_FEED_BEARING_XY))

#: The band a throw's landing error must enter within
#: :data:`SelfTossGateConfig.band_entry_throws` (plan § 4 R3): 20 mm lateral,
#: 42 mm of APEX error.
#:
#: 42 mm is the plan's original 20 ms of flight-time error, carried over
#: unchanged when the learner's outcome became an apex (2026-09-18): at the
#: 0.9 m operating point ``dh/dt_f = g·t_f/4 = 2.10 m/s``, so 0.020 s of
#: flight is 0.042 m of apex. Same physical tolerance, expressed in the
#: quantity the learner now commands.
XY_BAND_MM = 20.0
APEX_BAND_MM = 42.0

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
    staleness or mode change is modelled at R3), the hand-POSITION rows are
    gone (`REJECTED_HAND_NOT_PARKED` retired 2026-09-16 — the opening REST
    carries the hand home from wherever it is, and the seed reconciliation
    that replaced the refusal lives in the ROS node, which this gate does not
    run), and ``ball_evidence`` is the live observer's answer for ball 0."""

    def observations(t_abs_s):
        return ex.Observations(
            mocap_fresh=True, hand_fresh=True, levelled=True,
            ball_evidence=observer(0, t_abs_s), in_trajectory_mode=True)
    return observations


@dataclasses.dataclass
class SelfTossGateConfig:
    """The R3/R4 sim-validation operating point (plan § 4 R3, R4 Unit U7a) —
    a SEPARATE config from :class:`SkillsGateConfig`: this run's limits
    (leg jerk ``_SESSION_LEG_JERK_MMPS3`` = 200 000 since the R5 ramp was HELD on
    2026-09-30 and ``config/generated/admissible_box.yaml`` was re-swept at it —
    150 000 until then, when the fed columns rehearsal refused ``LIMIT_JERK`` at
    100-103 % of a cap the sitting no longer flies) share the columns gate's R2
    point, and its noise model adds the measured plant bias on top
    of the R2 knobs — reusing one dataclass would force one gate to carry a
    field the other never sets.

    ``pattern``/``separation_mm`` together pick :func:`_sites_for`'s site
    tuple: ``'self_toss'`` (the R3 default) is the single site P1 at
    ``_SELF_TOSS_SEPARATION_MM``; ``'hop'`` (R4) is the two sites P1/P2 at
    ``_HOP_SEPARATION_MM``, alternating release/target every throw
    (``schedule.OneBallPattern``). Both run through the SAME
    :meth:`SkillsGate.run_self_toss_attempt`/``run_self_toss_seed``/
    :func:`run_learn` — a hop is not a third copy of the loop, only a wider
    site tuple and a different ``pattern``/box lookup key
    (``executor.SkillExecutor._command_u`` reads ``self.schedule.pattern``,
    which ``schedule.compile_one_ball`` sets from the sites tuple's length)."""

    apex_m: float = 0.9
    dwell_s: float = 0.27
    #: 'self_toss' (1 site, P1) or 'hop' (2 sites, P1<->P2) -- see the class
    #: docstring. Validated by :func:`_sites_for`.
    pattern: str = 'self_toss'
    #: ``sites.columns_sites(separation_mm)`` -- the R3 default is
    #: ``_SELF_TOSS_SEPARATION_MM`` (100 mm); a hop run sets this to
    #: ``_HOP_SEPARATION_MM`` (250 mm) to match the swept 'hop' boxes.
    separation_mm: float = _SELF_TOSS_SEPARATION_MM
    leg_vel_mmps: float = _SESSION_LEG_VEL_MMPS
    leg_acc_mmps2: float = _SESSION_LEG_ACC_MMPS2
    leg_jerk_mmps3: float = _SESSION_LEG_JERK_MMPS3
    #: Follows the session constant (3900 since 2026-10-02, see its comment)
    #: for the same reason ``leg_jerk_mmps3`` follows its own: a stale literal
    #: here made every fed attempt restart on the 2026-09-30 rehearsal.
    hand_acc_rps2: float = _SESSION_HAND_ACC_RPS2
    #: Which aim source the executor runs with (``executor.AIM_SOURCES``).
    #: ``tracker`` here, not the LIVE default (``schedule``): this gate's
    #: whole tracker-refine surface (``_make_tracker``, the RESEND
    #: assertions) only exists on that path, and an open-loop run is
    #: requested explicitly with ``--catch-aim-source schedule``.
    catch_aim_source: str = ex.AIM_TRACKER
    noise: NoiseConfig = dataclasses.field(
        default_factory=lambda: NoiseConfig(bb_throw_noise_frac=0.0,
                                            tracking_noise_mm=0.5))
    learner_cfg: lr.LearnerConfig = dataclasses.field(
        default_factory=lr.LearnerConfig)
    xy_band_mm: float = XY_BAND_MM
    apex_band_mm: float = APEX_BAND_MM
    band_entry_throws: int = 5
    #: The monotone rule's far window edge (throws 16-25) — band entry (5) +
    #: 20 more throws, plan § 4 R3.
    target_throws: int = 25
    #: Safety cap on cheap resets (owner decision 6) — far above what a
    #: converging learner needs; hitting it is itself a reportable finding.
    max_attempts: int = 60
    admissible_box_path: str = None
    report_path: str = None
    #: R5 columns FEED trial (owner ask, 2026-09-30): when BOTH are set,
    #: :meth:`SkillsGate.run_columns_attempt` spawns ball 1 on an oblique
    #: arrival at the FEED site -- this many degrees off vertical, at this
    #: speed (mm/s), along :data:`_COLUMNS_FEED_BEARING_XY_UNIT`'s bearing
    #: -- instead of the R2 vertical self-toss spawn, and compiles the
    #: schedule via ``compile_columns(pattern, feed=LandingPrior(...))``,
    #: exactly as ``SkillNode._install_columns_schedule`` does for a real
    #: Ball Butler feed. Both ``None`` (the default) is today's vertical
    #: spawn, unchanged bit-for-bit; giving only one of the two is a
    #: ``ValueError`` (see that method). Ignored outside ``pattern ==
    #: 'columns'``.
    feed_angle_deg: float = None
    feed_speed_mmps: float = None
    #: R5 sitting 2 (owner decision, 2026-10-02): mirrors `skill_node.
    #: SkillNode`'s own `columns_feed_site` -- in a FEED trial (both of
    #: the above set), which named `columns_sites` site holds ball A and
    #: which is fed. 'P1' (the default) is the swapped layout
    #: `run_columns_attempt` implements unconditionally today: A at site
    #: 1 (P2), feed at site 0 (P1) -- site 0 is the site NEAREST Ball
    #: Butler along its feed bearing, so the incoming ball's descent
    #: never crosses A's own column. 'P2' is the OLD, un-swapped layout
    #: (A at site 0, feed at site 1) that let the two balls' mocap
    #: markers merge in flight on 4/4 attempts, 2026-10-02 sitting 2 --
    #: kept here only so this gate can still reproduce it. Ignored
    #: outside a FEED trial; any other value raises (no silent
    #: fallback -- this is a sim knob, not a launch parameter an
    #: operator can mistype).
    columns_feed_site: str = 'P1'
    #: R5 sitting 2 (owner decision, 2026-10-02): mirrors `skill_node.
    #: SkillNode`'s own `columns_feed_aim_toward_a_mm` -- in a FEED trial
    #: (both of the above set), ball B's synthetic arrival targets the
    #: FEED site walked this many mm toward the held ball A's site, not
    #: the feed site itself (`run_columns_attempt`'s own docstring: the
    #: swapped layout's UNDISPLACED feed catch refuses LIMIT_ACC at
    #: 101 %, the cup must reverse against Ball Butler's +x lateral
    #: arrival to match it at touch-down; 20 mm toward A clears at
    #: 80/83/73 % vel/acc/jerk). Ignored outside a FEED trial.
    #: 20 -> 10 mm on 2026-10-04, with skill_node's default (sitting 3: the
    #: 20 mm aim spent 17 of the 26 mm of column clearance on the first pass).
    feed_aim_toward_a_mm: float = 10.0
    #: B4 (R5 sitting 4, 2026-10-04): Ball Butler's own measured landing
    #: bias -- mirrors `skill_node.SkillNode`'s `columns_feed_bb_bias_mm`
    #: (schedule frame, mm; `bias = landing - request`). ``(0.0, 0.0)``
    #: (the default) is today's behaviour, unchanged: the ball spawns to
    #: land exactly where the schedule's own `LandingPrior` says it will
    #: (`aim_xy`, same as `feed_aim_toward_a_mm`'s own point) -- there is
    #: no "announced vs. physical" gap in this harness otherwise, since
    #: it builds its own synthetic announcement FROM the physical
    #: landing. A nonzero value opens exactly that gap, the other way
    #: round from the real fix: the `LandingPrior` fed to
    #: `compile_columns` (what the catch schedule plans against) stays at
    #: the UNBIASED `aim_xy` -- the request/announced point, same as a
    #: real Ball Butler's own reported belief -- while the ball is
    #: physically spawned to land `feed_bb_bias_mm` away from it, so this
    #: knob characterises the UNCORRECTED failure `columns_feed_bb_bias_mm`
    #: exists to cancel (R5 sitting 4: the probe read [+25.8, +27.3] on the L3 bag, A2's hand analysis (+29, +25); see
    #: `run_columns_attempt`'s own paragraph on this field for the exact
    #: split). Ignored outside a FEED trial.
    feed_bb_bias_mm: Tuple[float, float] = (0.0, 0.0)

    #: Which correlation rule :class:`_IdentityTracker` applies (2026-10-04,
    #: U1 "two-ball association" fix). ``'fixed'`` (the default): identity
    #: latches keyed on the announcement's own ``(thrower, throw_time)`` --
    #: the 2026-10-04 fix (``ball_possession.advance_correlation``).
    #: ``'head'``: the PRE-fix exclusion rule (``advance_flight_latches``,
    #: one claimed set PER schedule ball) -- the negative control that must
    #: reproduce WINDOW_TOO_SHORT on fed columns (U1 handoff § "Pass
    #: criteria"), proving this gate can see the class the fix closes.
    correlator: str = 'fixed'

    #: B2b (2026-10-04): run :meth:`SkillsGate.run_columns_1ball_seed`
    #: instead of :meth:`SkillsGate.run_columns_seed` -- ball A real, ball
    #: B a PHANTOM (``schedule.Pattern.phantom_balls=(1,)``, never
    #: spawned). Ignored outside ``pattern == 'columns'`` (CLI-enforced,
    #: ``main()``'s own ``p.error``, no silent fallback).
    one_ball: bool = False
    #: B2b only: ``True`` compiles the lead-in through
    #: ``schedule.compile_reload_columns`` (ball A fed externally, exactly
    #: as :meth:`SkillsGate.run_reload_attempt`'s own synthetic Ball
    #: Butler arrival) instead of ``compile_columns`` directly (ball A
    #: already resting at its site). Ignored outside ``one_ball``.
    one_ball_reload: bool = False

    #: B2 (R5 sitting 4): run :meth:`SkillsGate.run_columns_1ball_fed_seed`
    #: instead of :meth:`SkillsGate.run_columns_seed` -- the OTHER half of
    #: ``one_ball``: ball A a PHANTOM (``schedule.Pattern.
    #: phantom_balls=(0,)``, never spawned), ball B real and fed by Ball
    #: Butler (``feed_angle_deg``/``feed_speed_mmps`` both required, CLI-
    #: enforced). Mutually exclusive with ``one_ball`` (CLI-enforced).
    one_ball_fed: bool = False


def _correlator_is_legacy(cfg: 'SelfTossGateConfig') -> bool:
    """``cfg.correlator`` as the ``legacy`` bool :class:`_IdentityTracker`
    takes. Raises on anything but ``'fixed'``/``'head'`` -- no silent
    fallback to the fix (or to the negative control) on a typo."""
    if cfg.correlator not in ('fixed', 'head'):
        raise ValueError("cfg.correlator must be 'fixed' or 'head', got %r"
                         % (cfg.correlator,))
    return cfg.correlator == 'head'


def _sites_for(cfg: 'SelfTossGateConfig') -> tuple:
    """``cfg.pattern``'s site tuple -- ``('self_toss',)`` slices out just P1
    (the R3 single-site operating point, unchanged); ``'hop'`` (R4) keeps
    both P1 and P2, so ``schedule.OneBallPattern`` alternates release/target
    between them.  Both slice the SAME ``sites.columns_sites(cfg.separation_mm)``
    pair -- a hop's P1/P2 are the columns sites at the hop's own (wider)
    separation, not a second site definition (mirrors
    ``schedule.py``'s own ``columns_sites`` reuse for ``compile_columns``).
    Raises on any other ``pattern`` -- there is no site tuple to fall back to
    (plan § 0: no fallback modes)."""
    s0, s1 = sites.columns_sites(cfg.separation_mm)
    if cfg.pattern == 'hop':
        return (s0, s1)
    if cfg.pattern == 'self_toss':
        return (s0,)
    raise ValueError("cfg.pattern must be 'self_toss' or 'hop', got %r"
                     % (cfg.pattern,))


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
    planner floor, which is exactly why ``compile_one_ball`` opens on a
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


#: The substring :meth:`~jugglebot.motion.skills.executor.SkillExecutor.
#: _valid_tracked_landing` puts in ``ThrowReport.no_row_reason`` when it
#: refuses a tracker read whose landing time misses THIS release's own
#: scheduled landing by more than its band (``TRACKER-IDENTITY-REFUSED``,
#: ``executor.py:2253``/``2297``) -- the ONE place a wrong-ball read
#: surfaces WITHOUT the gate needing its own (ground-truth) comparison.
#: Probed empirically 2026-10-04 (B1, ``probe_assoc8.py``, ``--correlator
#: head`` on fed columns): every throw past the first reads this, with a
#: fitted-landing-vs-scheduled gap of 0.54-0.57 s -- one beat (0.579 s),
#: exactly U1's own finding, and NOT a ``WINDOW_TOO_SHORT`` end_code -- A1's
#: band gate (landed the same day as this unit) already intercepts the bad
#: READ downstream before an install ever sees it, so the hard refusal U1
#: diagnosed against the OLD code does not reproduce verbatim; this is the
#: layered defense working, and the substring below is what a wrong-ball
#: row looks like under TODAY's full stack.
_WRONG_BALL_REASON = "not this release's flight"


def _association_verdict(reports: list, end_codes: list,
                         beat_s: float) -> dict:
    """The two-ball-association pass criteria over one run (U1 handoff §
    "Pass criteria", read through :class:`_IdentityTracker`'s REAL
    correlator rather than ground truth):

    1. no refusal is ``WINDOW_TOO_SHORT`` -- the 2026-10-02 sitting's own
       symptom (a mis-latched release's window measures against the WRONG
       flight's landing) under the PRE-A1 stack;
    2. every LEARNER row's (``ThrowReport.row``) ``release_err_s`` is inside
       ``beat_s / 2`` -- a mis-latched row reads a WHOLE beat off (U1 § 2d:
       -0.519..-0.575 s against this run's own beat), well outside the
       genuine release lag (0.02-0.14 s) plus apex-fit scatter the bound
       otherwise tolerates;
    3. no row is refused for reading the WRONG release's flight
       (:data:`_WRONG_BALL_REASON`, A1's ``TRACKER-IDENTITY-REFUSED`` band,
       which is what a wrong-ball attribution looks like under today's full
       stack -- see that constant's own note)."""
    window_too_short = any('WINDOW_TOO_SHORT' in str(c) for c in end_codes)
    wrong_ball = [r for r in reports
                 if not r.row and _WRONG_BALL_REASON in (r.no_row_reason or '')]
    errs = [float(r.release_err_s) for r in reports
           if r.row and r.release_err_s is not None]
    bound = float(beat_s) / 2.0
    bad = [e for e in errs if abs(e) >= bound]
    return dict(
        window_too_short=bool(window_too_short),
        wrong_ball_rows=len(wrong_ball),
        release_err_checked=len(errs),
        release_err_max_s=(max(abs(e) for e in errs) if errs
                           else float('nan')),
        release_err_bound_s=bound,
        release_err_bad=len(bad),
        ok=bool((not window_too_short) and len(bad) == 0
                and len(wrong_ball) == 0))


def _wall_ms_stats(wall_s) -> dict:
    """``{'min', 'p50', 'p95', 'max', 'n'}`` (ms) over an iterable of
    per-install wall-clock seconds -- the sim-side twin of the Jetson's
    < 50 ms install gate.  ONE definition shared by the columns gate
    (``SkillsGate.run``) and every ``SelfTossGateConfig`` run
    (``run_self_toss_seed``, self-toss and R4 hop alike) rather than each
    computing its own percentile (plan § 0: "no second copy of any constant
    or timing")."""
    ms = sorted(float(w) * 1000.0 for w in wall_s)

    def _pctl(p):
        if not ms:
            return float('nan')
        k = min(len(ms) - 1, int(round(p * (len(ms) - 1))))
        return float(ms[k])

    return {
        'min': _pctl(0.0),
        'p50': (float(statistics.median(ms)) if ms else float('nan')),
        'p95': _pctl(0.95),
        'max': _pctl(1.0),
        'n': len(ms),
    }


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
        # The Stop (D2, E1'): the last throw is aimed at the OTHER site and
        # lands ON the ball already held there — read straight off the
        # schedule's own skills (:func:`_columns_final_removal`), not
        # re-derived arithmetic, so this stays correct however
        # ``compile_columns`` schedules it. ``n_throws == 1`` has no Stop
        # (nothing held yet) and falls back to the pre-E1' arithmetic — the
        # only throw's own landing, at its own site.
        shadow = _columns_final_removal(sched)
        if shadow is not None:
            t_land_final, ball_final, final_target_xy_mm = shadow
        else:
            t_land_final = t0_abs_s + (cfg.n_throws - 1) * beta + t_f
            ball_final = (cfg.n_throws - 1) % 2
            final_target_xy_mm = None

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
        executor = ex.SkillExecutor(
            sched, installer, tracker=tracker,
            catch_aim_source=self.cfg.catch_aim_source,
            dispatch_lookahead_s=_DISPATCH_LOOKAHEAD_S)

        # 4. Stream (the one loop -- ``_stream_chain``).
        t_end = t_land_final + QUIET_TAIL_S
        loop = self._stream_chain(
            plant=plant, geom=geom, rcfg=self.rcfg, noise=noise,
            executor=executor, ictx=ictx, ball_state=ball_state, t_end=t_end,
            final_removal=(t_land_final, ball_final),
            final_target_xy_mm=final_target_xy_mm)

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
        # D2/E1': the platform must be sitting under the last ball when it
        # lands on the one already held — ``None`` (n_throws == 1, nothing
        # held) is vacuously OK, nothing to check.
        final_cup_err_mm = float(loop.get('final_cup_err_mm', float('nan')))
        final_cup_ok = bool(final_target_xy_mm is None
                            or (not math.isnan(final_cup_err_mm)
                                and final_cup_err_mm <= FINAL_CUP_XY_TOL_MM))

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
            final_cup_err_mm=final_cup_err_mm,
            pump_clean=pump_clean, mirror_ok=mirror_ok, flags_ok=flags_ok,
            all_installed=all_installed, caught_all=caught_all,
            no_drops=no_drops, final_cup_ok=final_cup_ok)
        res.passed = bool(pump_clean and mirror_ok and flags_ok
                          and all_installed and caught_all and no_drops
                          and final_cup_ok)
        return res

    # ── the one stream loop (shared with the R3 self-toss learner run) ─────

    def _stream_chain(self, *, plant, geom, rcfg, noise, executor, ictx,
                      ball_state, t_end, release_bias=None,
                      final_removal=None, final_target_xy_mm=None,
                      stop_check=None, pending_spawn=None, identity=None):
        """The emitter/pump/wire/mirror/executor.tick loop (module
        docstring) -- run by every caller against a live ``PlanRecord``
        (``ictx``) and the executor dispatching one ``Schedule``.

        ``identity`` (:class:`_IdentityTracker` or ``None``): when given,
        ``identity.tick(t, ball_state)`` runs every sim tick BEFORE
        ``executor.tick(t)`` -- the correlator must see this instant's
        synthetic ``/balls`` snapshot before the executor's own dispatch
        check can read ``identity.tracker``, exactly as a real ``/balls``
        message precedes the node's dispatch on the same clock. ``None``
        (the R2/R4 callers that do not use the identity-aware tracker)
        changes nothing from before this parameter existed.

        ``ball_state`` is ``{ball_id: {'estimator', 'airborne',
        'next_obs_t', 'expect_held', 'lost_since'}}`` -- the per-ball
        tracking/possession bookkeeping the SIM gate owns (the executor
        itself never touches MuJoCo). ``release_bias(vel_mm_s) ->
        vel_mm_s``, when given, is applied AFTER ``noise.perturb_throw`` at
        every physical release -- the R3 self-toss run's measured-plant
        bias (:func:`_measured_release_bias`); the R2 columns gate passes
        ``None`` and its own exact/noisy-tracking behaviour is unchanged.

        ``pending_spawn`` (``(spawn_t_abs_s, position_mm, velocity_mms,
        ball_id)`` or ``None``): a ball not yet in the scene, spawned the
        first tick ``plant.data.time >= spawn_t_abs_s`` -- the R4 reload
        trial's own need (:meth:`run_reload_attempt`, root-caused
        2026-09-24: spawning at the caller's "now" instead of the ball's own
        physically-valid release instant put the spawn point 69.5 m under
        the world floor -- see ``_RELOAD_BALL_FLIGHT_LEAD_S``'s docstring).
        ``ball_state[ball_id]`` is armed (``airborne=True``, ``next_obs_t``
        reset, ``estimator`` reset) at the same instant, so nothing is
        "observed" of a ball that is not yet physically in the world. Every
        other caller passes ``None`` (spawn already happened before the
        loop starts, the pre-R4 behaviour, unchanged).
        ``final_removal`` (``(t_land_final, ball_final)``) is the columns-
        only "remove the last unscheduled ball" step; ``None`` skips it.
        ``final_target_xy_mm`` (mm, xy or ``None``), only meaningful with
        ``final_removal`` set: the D2/E1' Stop check (module docstring's
        columns Stop) -- at the removal instant, the PLANT's own emitted cup
        xy (:func:`~jugglebot.motion.unified_cycle.cup_state_from_platform`
        of the live ``PlantState``, not the plan's commanded knot: this is a
        physical-chain sanity check on the real MuJoCo pose, mirror included)
        is compared against it and the distance returned as
        ``'final_cup_err_mm'``. ``None`` (every caller but
        :meth:`run_trial` with ``n_throws >= 2``) skips the check —
        ``'final_cup_err_mm'`` stays out of the returned dict.
        ``stop_check()``, when given, ends the loop as soon as it returns
        True (in addition to ``t_end``) -- the self-toss run's
        ``executor.done``, so a refused or dropped attempt does not stream
        out a whole ``t_end`` budget it no longer needs.

        Returns ``{'makes', 'drops', 'emitted', 'accepted', 'pump_rejects',
        'flags_seen' (a set), 'worst_leg', 'worst_hand', 'ticks', 'wall_s',
        'final_cup_err_mm' (only when ``final_target_xy_mm`` is given)}``.
        """
        emitter = KnotEmitter(geom)
        pump = stream_chain.make_pump()
        mirror = stream_chain.make_mirror(geom)
        next_frame_t = plant.data.time
        frame_seq = 0
        frame_t0 = None
        latch_tau = 0.0
        #: The ``t0_s`` of the record the mirror's last frame was latched
        #: from: the reconstruction check compares the mirror against THAT
        #: plan's phase only (a fresh install changes ``record`` before its
        #: first frame reaches the mirror; a splice keeps ``t0_s`` and a
        #: bit-identical head, so it stays comparable).
        latch_t0 = None
        #: Monotone wire stamp across records (the robot stamps every frame
        #: with its own knot instant, never a per-plan counter that restarts).
        wire_seq = 0
        emitted = accepted = 0
        flags_seen = set()
        worst_leg = worst_hand = 0.0
        ticks = 0
        makes = 0
        drops = 0
        removed_final = final_removal is None
        final_cup_err_mm = float('nan')
        t_wall = time.time()

        while plant.data.time < t_end and (stop_check is None
                                           or not stop_check()):
            t = plant.data.time

            if pending_spawn is not None and t >= pending_spawn[0] - 1e-9:
                spawn_t, spawn_pos, spawn_vel, spawn_ball = pending_spawn
                plant.spawn_ball(spawn_pos, spawn_vel, ball=spawn_ball)
                bstate = ball_state[spawn_ball]
                bstate['airborne'] = True
                bstate['expect_held'] = False
                bstate['lost_since'] = None
                bstate['estimator'].reset()
                bstate['next_obs_t'] = t
                pending_spawn = None

            # The correlator's snapshot for THIS instant, before the
            # executor reads it (``identity`` docstring above).
            if identity is not None:
                identity.tick(t, ball_state)

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
                if record is not None and (frame_t0 is None
                                           or record.t0_s != frame_t0):
                    # A fresh install starts a NEW record.t0_s (a splice never
                    # does -- install_segment carries it forward unchanged) --
                    # re-grid the frame counter onto that origin so knot 0
                    # samples at tau=0 exactly. The first frame is the first
                    # grid instant AT OR AFTER now, which is BEFORE t0 for a
                    # reserved fresh origin (t_now + LEAD_S): those frames
                    # sample tau < 0, which a CyclePlan answers with knot 0 --
                    # the hold trajectory_node's emitter streams between the
                    # install and t0 (its `_emit_once` samples
                    # `state_at(t_k - t0)` on the absolute grid). Until
                    # 2026-10-02 this jumped straight to tau = 0 at the
                    # install instant, so a future origin's plan reached the
                    # mirror (t0 - t_install) EARLY.
                    frame_t0 = record.t0_s
                    frame_seq = int(math.ceil(
                        (t - record.t0_s) / record.plan.dt - 1e-9))
                    next_frame_t = record.t0_s + frame_seq * record.plan.dt
                if record is not None and t >= next_frame_t - 1e-9:
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
                        frame = emitter.frame(record.plan, tau, wire_seq)
                        sp, _reason = pump.build(
                            frame, t_origin_us=int(wire_seq * 25000))
                        emitted += 1
                        wire_seq += 1
                        if sp is not None:
                            accepted += 1
                            flags_seen.add(int(sp.flags))
                            stream_chain.latch(mirror, Setpoint.unpack(sp.pack()),
                                              t)
                            latch_tau = tau
                            latch_t0 = record.t0_s
                        frame_seq += 1
                    next_frame_t += record.plan.dt
                elif record is None:
                    next_frame_t += KNOT_DT_S

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
                # Negative phases (the pre-t0 hold) are checked too: the plan
                # answers knot 0 there, which is what the mirror must hold.
                if (latch_t0 == record.t0_s
                        and tau_phase <= record.plan.total_duration):
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
                if final_target_xy_mm is not None:
                    st_f = plant.get_state()
                    pose_f = np.concatenate(
                        [st_f.platform_pos_mm, st_f.platform_rot])
                    hand_rev_f = stream_chain.rev_of_slider_mm(
                        float(st_f.hand_pos_mm), rcfg)
                    cup_f = uc.cup_state_from_platform(pose_f, hand_rev_f,
                                                       rcfg)
                    final_cup_err_mm = float(np.linalg.norm(
                        np.asarray(cup_f[:2], dtype=float)
                        - np.asarray(final_target_xy_mm, dtype=float)))
                plant.ball_manager.ball(final_removal[1]).reset()
                ball_state[final_removal[1]]['airborne'] = False
                ball_state[final_removal[1]]['expect_held'] = False
                removed_final = True

            plant.step(TICK_S)
            ticks += 1
            if self.viewer is not None:
                self.viewer.sync()

        out = dict(makes=makes, drops=drops, emitted=emitted,
                  accepted=accepted, pump_rejects=int(pump.frames_rejected),
                  flags_seen=flags_seen, worst_leg=worst_leg,
                  worst_hand=worst_hand, ticks=ticks,
                  wall_s=time.time() - t_wall)
        if final_target_xy_mm is not None:
            out['final_cup_err_mm'] = final_cup_err_mm
        return out

    # ── R3: the self-toss learner run (plan § 4 R3 "Sim validation") ───────
    #
    # A ``SkillsGate`` used for these methods must be constructed with a
    # ``SelfTossGateConfig`` (``self.cfg``), never a ``SkillsGateConfig`` --
    # ``run_learn`` below is the only constructor, and it never shares an
    # instance with ``run`` / ``run_trial``.

    def run_self_toss_attempt(self, *, pattern_sites, n_throws: int,
                              memory: mem.Memory, learner_cfg: lr.LearnerConfig,
                              boxes: list, noise: JuggleNoise,
                              throws_out: list, reports_out: list = None
                              ) -> Tuple[str, dict, '_InstallCtx']:
        """One self-toss (``pattern_sites`` = ``(P1,)``) or hop
        (``pattern_sites`` = ``(P1, P2)``, R4) attempt, cold from the
        ACTIVATE park (owner decision 6, "resets are cheap"): reset + spawn
        at ``pattern_sites[0]``, ``schedule.compile_one_ball``, stream it
        through the one chain (:meth:`_stream_chain`) with the R3
        learner/box/observer/observations wired in.  ``throws_out`` accumulates
        every finalised :class:`~jugglebot.motion.skills.memory.Experience`
        this attempt produces (in schedule order); the memory itself is
        appended to as a side effect of ``on_experience`` (below), so it
        carries forward to the NEXT attempt even though the plant does not.
        ``reports_out``, when given, accumulates this attempt's
        ``executor.reports`` (one ``ThrowReport`` per finalised release,
        INCLUDING no-row releases) -- the two-ball-association pass criteria
        (U1 handoff § "Pass criteria") read ``release_err_s`` off these, not
        off ``Experience``, which does not carry it.

        The ``tracker`` wired into the executor goes through the REAL
        correlator (:class:`_IdentityTracker`, over :func:`_make_tracker`'s
        ground-truth fit), not ground truth directly -- `cfg.correlator`
        picks the identity rule (``'fixed'``, the 2026-10-04 fix) or the
        legacy exclusion rule (``'head'``, the negative control).

        Returns ``(end_code, loop_stats, ictx)`` -- ``end_code`` is ``''`` for
        an attempt that ran its whole schedule with no refusal.
        """
        cfg: SelfTossGateConfig = self.cfg
        plant = self.plant
        geom = self.geom
        park = _activate_park_state(self.rcfg, pattern_sites[0])
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

        pattern = sk.OneBallPattern(sites=tuple(pattern_sites), apex_m=cfg.apex_m,
                                    dwell_s=cfg.dwell_s, n_throws=n_throws)
        sched = sk.compile_one_ball(pattern, t0_abs_s)

        ball_state = {0: dict(estimator=BallisticEstimator(bal.G_VEC_MMS2),
                             airborne=False, next_obs_t=None,
                             expect_held=True, lost_since=None,
                             last_obs_t=None)}
        identity = _IdentityTracker(_make_tracker(plant, ball_state),
                                    legacy=_correlator_is_legacy(cfg))
        ictx = _InstallCtx(park)
        installer = _make_installer(ictx, self.seg_cfg, self.limits, geom,
                                    on_release=identity.announce)
        observer = _make_observer(plant)
        observations = _make_observations(observer)
        learner = _MemoryLearner(memory, learner_cfg)
        executor = ex.SkillExecutor(
            sched, installer, tracker=identity.tracker,
            catch_aim_source=self.cfg.catch_aim_source,
            learner=learner, boxes=boxes,
            observer=observer, on_experience=lambda exp: (
                memory.append(exp), throws_out.append(exp)),
            observations=observations,
            dispatch_lookahead_s=_DISPATCH_LOOKAHEAD_S)

        # A generous, bounded timeout -- the real exit is ``executor.done``
        # (``stop_check``), so a refused or dropped attempt does not stream
        # out the whole schedule's worth of quiet tail it no longer needs.
        t_end = float(sched.skills[-1].t_abs_s) + 1.0
        loop = self._stream_chain(
            plant=plant, geom=geom, rcfg=self.rcfg, noise=noise,
            executor=executor, ictx=ictx, ball_state=ball_state, t_end=t_end,
            release_bias=_measured_release_bias,
            stop_check=lambda: executor.done, identity=identity)
        if reports_out is not None:
            reports_out.extend(executor.reports)
        return str(executor.end_code), loop, ictx

    def run_self_toss_seed(self, seed: int, policy: str = 'A') -> dict:
        """One seed's whole R3/R4 learner run: repeated cheap-reset attempts,
        collecting throws until ``cfg.target_throws`` releases have been
        observed (plan § 4 R3 / owner decision 2026-09-13; R4 Unit U7a
        generalises this to ``cfg.pattern``'s site tuple, :func:`_sites_for`
        -- self-toss's single site is unchanged, a hop alternates P1/P2).

        Policy **A**: single-throw attempts (``n_throws=1``, THROW -> CATCH ->
        REST) until the memory holds ``learner_cfg.k_min`` rows, then chained.
        Policy **B**: chained from the very first throw.  Both retain the
        memory across every reset -- only the plant and the executor are cold
        each attempt.  Unchanged by ``pattern``: the memory's ``x`` carries
        the release site's xy (``executor._command_u``), so P1 and P2 rows
        sit ``>=`` 125 mm apart in ``x``-space against a 10 mm kernel
        bandwidth (``learner.LearnerConfig.h_x``) -- a hop's two sites learn
        two effectively separate corrections from the one shared memory file,
        no per-site bookkeeping needed here.
        """
        if policy not in ('A', 'B'):
            raise ValueError("policy must be 'A' or 'B', got %r" % (policy,))
        cfg: SelfTossGateConfig = self.cfg
        pattern_sites = _sites_for(cfg)
        boxes = _load_admissible_boxes(cfg.admissible_box_path)
        noise = JuggleNoise(cfg.noise, seed=seed)
        tmp_dir = tempfile.mkdtemp(prefix='skills_gate_learn_seed%d_' % seed)
        memory = mem.Memory(os.path.join(tmp_dir, 'memory.csv'))
        assert len(memory) == 0, 'a fresh tmp path must be a cold memory'

        throws: list = []
        reports: list = []
        attempts = 0
        refusals = 0
        drops_total = 0
        makes_total = 0
        installs_total = installs_accepted = 0
        end_codes = []
        plan_wall_s_all: list = []
        t_wall0 = time.time()

        while len(throws) < cfg.target_throws and attempts < cfg.max_attempts:
            remaining = cfg.target_throws - len(throws)
            if policy == 'A' and len(memory) < cfg.learner_cfg.k_min:
                n_throws = 1
            else:
                n_throws = max(1, remaining)
            attempts += 1
            end_code, loop, ictx = self.run_self_toss_attempt(
                pattern_sites=pattern_sites, n_throws=n_throws, memory=memory,
                learner_cfg=cfg.learner_cfg, boxes=boxes, noise=noise,
                throws_out=throws, reports_out=reports)
            end_codes.append(end_code)
            if end_code:
                refusals += 1
            drops_total += loop['drops']
            makes_total += loop['makes']
            installs_total += ictx.installs_total
            installs_accepted += ictx.installs_accepted
            plan_wall_s_all.extend(ictx.plan_wall_s)

        rows = []
        for i, exp in enumerate(throws):
            err_xy_mm = float(1000.0 * np.linalg.norm(exp.y[:2]))
            err_apex_mm = float(1000.0 * abs(float(exp.y[2]) - cfg.apex_m))
            rows.append(dict(
                throw=i + 1, u=exp.u.tolist(), y=exp.y.tolist(),
                err_xy_mm=err_xy_mm, err_apex_mm=err_apex_mm,
                caught=bool(exp.caught), t_abs_s=float(exp.t_abs_s)))

        throws_to_band_xy = next(
            (r['throw'] for r in rows if r['err_xy_mm'] <= cfg.xy_band_mm),
            None)
        throws_to_band_apex = next(
            (r['throw'] for r in rows
             if r['err_apex_mm'] <= cfg.apex_band_mm), None)
        entered_band = bool(
            throws_to_band_xy is not None
            and throws_to_band_xy <= cfg.band_entry_throws
            and throws_to_band_apex is not None
            and throws_to_band_apex <= cfg.band_entry_throws)

        monotone_xy = _monotone_verdict(
            [r['err_xy_mm'] for r in rows], cfg.band_entry_throws,
            cfg.target_throws)
        monotone_apex = _monotone_verdict(
            [r['err_apex_mm'] for r in rows], cfg.band_entry_throws,
            cfg.target_throws)

        beat = sk.beat_s(sk.flight_s(cfg.apex_m), cfg.dwell_s)
        assoc = _association_verdict(reports, end_codes, beat)

        return dict(
            seed=seed, policy=policy, pattern=cfg.pattern,
            separation_mm=cfg.separation_mm, throws=rows,
            n_throws_collected=len(rows), attempts=attempts,
            end_codes=end_codes, refusals=refusals, drops=drops_total,
            makes=makes_total, installs_total=installs_total,
            installs_accepted=installs_accepted,
            plan_wall_ms=_wall_ms_stats(plan_wall_s_all),
            throws_to_band_xy=throws_to_band_xy,
            throws_to_band_apex=throws_to_band_apex,
            entered_band=entered_band,
            monotone_xy=monotone_xy, monotone_apex=monotone_apex,
            association=assoc,
            passed=bool(entered_band and bool(monotone_xy)
                       and bool(monotone_apex) and assoc['ok']),
            wall_s=time.time() - t_wall0)

    # ── R5: the columns learner run (Unit G, plan § R5) ─────────────────────
    #
    # Both balls, one shared memory, the learner ON -- the SAME physical cold-
    # reset setup :meth:`run_trial` uses each attempt ("spawn B as today":
    # ball A seated at site 0, ball B already in flight toward site 1, t0
    # chosen so the schedule's own "ball B released at t0-beta" premise is
    # exact) but through :func:`compile_columns`'s LEARNER-COMMANDED throws
    # rather than R2's identity prior. NOT built on ``run_self_toss_attempt``/
    # ``_sites_for`` (the ONE-ball self-toss/hop generaliser): a columns
    # attempt needs BOTH sites live at once (``sk.Pattern``, not
    # ``sk.OneBallPattern``) and the Stop's cross-site shadow throw
    # (``compile_columns`` only), which the one-ball loop has no shape for.

    def run_columns_attempt(self, *, n_throws: int, memory: mem.Memory,
                            learner_cfg: lr.LearnerConfig, noise: JuggleNoise,
                            throws_out: list, reports_out: list = None,
                            phantom_a: bool = False
                            ) -> Tuple[str, dict, '_InstallCtx']:
        """One columns learner attempt, cold from a level rest holding ball
        A with ball B already airborne toward the FEED site (the SAME
        setup :meth:`run_trial` steps 1-2 use) -- vertically
        (``cfg.feed_angle_deg``/``cfg.feed_speed_mmps`` both ``None``, the
        default: A rests at site 0, B feeds site 1, unchanged bit-for-bit
        since before the FEED option) or, when both are set, on an oblique
        Ball Butler-like arrival compiled through ``compile_columns(pattern,
        feed=LandingPrior(...))`` instead of a free ``t0_abs_s`` -- see
        :data:`SelfTossGateConfig.feed_angle_deg`.

        **A FEED trial swaps which site holds A and which is fed, per
        `cfg.columns_feed_site` (R5 sitting 2, owner decision,
        2026-10-02), mirroring `skill_node.SkillNode._run_columns`'s own
        swap**: by default (``columns_feed_site='P1'``) A rests at site
        1, B feeds site 0 -- site 0 is the site NEAREST Ball Butler
        along its feed bearing, so B's descent never crosses A's own
        column (the OLD, un-swapped FEED trial, ``columns_feed_site=
        'P2'``, let the two balls' mocap markers merge in flight on 4/4
        attempts, 2026-10-02 sitting 2). The swapped
        layout's own cost: the feed catch must reverse the cup against
        Ball Butler's +x lateral arrival to match it at touch-down, so the
        UNDISPLACED feed catch refuses LIMIT_ACC at 101 % -- the synthetic
        arrival's target is therefore walked `cfg.feed_aim_toward_a_mm`
        toward site 1 (A's site), not site 0 itself. The vertical
        (non-FEED) spawn above is NOT swapped -- A still rests at site 0,
        B still feeds site 1, exactly as before the FEED option existed.
        The learner (:class:`_MemoryLearner`)
        replaces the identity prior; the box lookup is BYPASSED
        (``boxes=None``): the committed 'columns' box entries are
        EMPTY/NaN as swept for this rung (``config/generated/
        admissible_box.yaml``, gate hash stale against a concurrent planner
        sweep besides) -- with ``boxes=None`` the learner's raw command
        reaches the planner unclipped (``executor._command_u``: the
        box-refusal branch only runs when ``self.boxes is not None``), so a
        genuinely bad command is caught by the PLANNER's own refusal, not
        silently clipped into a box nobody swept. ``resend_max_per_catch=0``
        for this two-site pattern (owner decision 2026-09-29): a catch
        re-send releases from the RE-AIMED cup position
        (``executor._catch_terminal``), and the fly-back that needs carries
        a lateral offset the OTHER site's own throw was never sized for --
        ``_make_installer``'s pending-release note on why appending (not
        superseding) walked the learner gate 50 mm off-site once release
        followed catch.

        Every throw but the schedule's Stop (the cross-site shadow landing --
        bypasses the learner entirely and produces no experience row, see
        ``compile_columns``/``executor._finalise_outcome``) reaches
        ``on_experience``; ``throws_out`` accumulates them in schedule
        order, same contract as :meth:`run_self_toss_attempt`. ``reports_out``
        is this attempt's ``executor.reports``, same contract too (U1
        handoff's pass criteria read ``release_err_s`` off these).

        The tracker is :class:`_IdentityTracker` over ground truth, exactly
        as :meth:`run_self_toss_attempt` wires it -- THIS is the method the
        2026-10-02 mis-latch needed: two schedule balls, each announced
        while the other is mid-flight, is precisely the shape the old
        ground-truth stand-in could never exercise (U1 § 3). A FEED trial
        additionally announces ball 1's feed flight with ``thrower=
        'ball_butler'`` at its physical spawn instant -- the sim's stand-in
        for Ball Butler's own ``ThrowAnnouncement`` (``_install_announced_
        reload``'s real-system counterpart) -- the un-fed vertical spawn
        announces nothing for that first flight, matching the real system's
        "ball already in the air, no prior to gate against" case (U1
        handoff, "What A1 changed... accepted as before").

        Returns ``(end_code, loop_stats, ictx)`` -- ``end_code`` is ``''``
        for an attempt that ran its whole schedule with no refusal.

        ``phantom_a`` (R5 sitting 4, B2's ``columns_1ball_fed`` -- the other
        half of ``columns_1ball``: ball A phantom at ``a_site``, ball B
        real, fed, exactly as the FEED trial above): requires ``feed_mode``
        (``cfg.feed_angle_deg``/``cfg.feed_speed_mmps`` both set) -- a
        vertical spawn would have nothing real to catch at all. Ball A is
        never spawned in hand, ``ball_state[0]`` is the SAME inert shape
        :meth:`run_columns_1ball_attempt` gives its own phantom ball
        (``expect_held=False``, never watched for a drop/catch), and its
        release is never announced (the ``on_release`` filter below, that
        method's own mirror). ``pattern.phantom_balls=(0,)`` is the ONE
        enforcement point for everything else (no OUTCOME row, no tracker
        latch, no ball-evidence precondition, never a D3 survivor) --
        ``executor._register_outcome``'s own phantom guard already means
        ``throws_out``/``memory`` only ever see ball B's rows, so the
        verdict this feeds (:meth:`run_columns_1ball_fed_seed`) counts ball
        B alone with no extra filtering needed.
        """
        cfg: SelfTossGateConfig = self.cfg
        plant = self.plant
        geom = self.geom
        site0, site1 = sites.columns_sites(cfg.separation_mm)
        # R5 sitting 2 (2026-10-02): a FEED trial swaps which site holds A
        # and which is fed (mirrors `SkillNode._run_columns`'s own swap,
        # class docstring above); the vertical run keeps site 0 = A,
        # site 1 = feed, unchanged.
        feed_mode = not (cfg.feed_angle_deg is None
                        and cfg.feed_speed_mmps is None)
        # R5 sitting 2 (2026-10-02): `cfg.columns_feed_site` picks the
        # swap ONLY inside a FEED trial (`SelfTossGateConfig.
        # columns_feed_site`'s own docstring) -- the vertical (non-FEED)
        # spawn is never swapped, unchanged bit-for-bit.
        if not feed_mode:
            a_site, feed_site = site0, site1
        elif cfg.columns_feed_site == 'P1':
            a_site, feed_site = site1, site0
        elif cfg.columns_feed_site == 'P2':
            a_site, feed_site = site0, site1
        else:
            raise ValueError("cfg.columns_feed_site must be 'P1' or 'P2', "
                             'got %r' % (cfg.columns_feed_site,))

        if phantom_a and not feed_mode:
            raise ValueError(
                'phantom_a requires feed_mode (feed_angle_deg/feed_speed_mmps '
                'both set) -- a vertical, un-fed spawn leaves ball B with '
                'nothing real to catch')

        rest0 = self._rest_state(a_site)
        pose0 = np.asarray(rest0.pose, dtype=float)
        plant.reset(pose0)
        plant.command(plant.pose_to_extensions(pose0))
        plant.command_hand(stream_chain.slider_mm_of_rev(
            float(rest0.hand_rev), self.rcfg))
        for _ in range(40):
            plant.step(KNOT_DT_S)
            if self.viewer is not None:
                self.viewer.sync()
        # B2 (`columns_1ball_fed`): ball A is never physically there to
        # spawn when it is the phantom -- see `run_columns_1ball_attempt`'s
        # own phantom ball, never spawned either.
        if not phantom_a:
            plant.ball_manager.ball(0).spawn_in_hand()

        t_f = sk.flight_s(cfg.apex_m)
        beta = sk.beat_s(t_f, cfg.dwell_s)
        pos1 = feed_site.catch_site_mm()
        pattern = sk.Pattern(sites=(a_site, feed_site), apex_m=cfg.apex_m,
                             dwell_s=cfg.dwell_s, n_throws=n_throws,
                             phantom_balls=((0,) if phantom_a else ()))

        if not feed_mode:
            # R2's vertical self-toss spawn -- ball B already airborne
            # toward the feed site, landing back at its own launch point
            # after a full flight `t_f`. Unchanged bit-for-bit from before
            # the FEED option (owner ask, 2026-09-30).
            v0_mm_s = math.sqrt(2.0 * bal.GRAVITY_MMS2 * cfg.apex_m * 1000.0)
            pos1n, vel1n = noise.perturb_throw(
                pos1, np.array([0.0, 0.0, v0_mm_s]), np.zeros(3))
            t_spawn = float(plant.data.time)
            plant.spawn_ball(pos1n, vel1n, ball=1)
            t0_abs_s = t_spawn + beta
            sched = sk.compile_columns(pattern, t0_abs_s)
        else:
            # R5 FEED trial (owner ask, 2026-09-30): a Ball Butler-like
            # oblique arrival at the feed site, fed through
            # `compile_columns`'s `feed=` path exactly as `SkillNode.
            # _install_columns_schedule` compiles a real BB
            # announcement/tracker landing (`skill_node.py` ~3197/3321) --
            # `t0 = t_land - transit_s` is computed by `compile_columns`
            # itself, not chosen here.
            if cfg.feed_angle_deg is None or cfg.feed_speed_mmps is None:
                raise ValueError(
                    'feed_angle_deg and feed_speed_mmps must both be set, '
                    'or both left None (the default vertical spawn) -- got '
                    'feed_angle_deg=%r feed_speed_mmps=%r'
                    % (cfg.feed_angle_deg, cfg.feed_speed_mmps))
            angle = math.radians(cfg.feed_angle_deg)
            lateral_mm_s = cfg.feed_speed_mmps * math.sin(angle)
            land_vel_nom = np.array([
                _COLUMNS_FEED_BEARING_XY_UNIT[0] * lateral_mm_s,
                _COLUMNS_FEED_BEARING_XY_UNIT[1] * lateral_mm_s,
                -cfg.feed_speed_mmps * math.cos(angle)])
            # R5 sitting 2 (2026-10-02): the synthetic arrival targets the
            # feed site walked `feed_aim_toward_a_mm` TOWARD A's site
            # (`SkillNode._columns_feed_aim_site`'s own mirror) -- the
            # swapped layout's UNDISPLACED feed catch refuses LIMIT_ACC at
            # 101 % (class docstring above).
            a_minus_feed_xy = (np.asarray(a_site.cup_mm[:2], dtype=float)
                              - np.asarray(feed_site.cup_mm[:2], dtype=float))
            norm = float(np.linalg.norm(a_minus_feed_xy))
            aim_unit_xy = (a_minus_feed_xy / norm if norm > 1e-9
                          else np.zeros(2))
            aim_xy = (np.asarray(feed_site.cup_mm[:2], dtype=float)
                     + cfg.feed_aim_toward_a_mm * aim_unit_xy)
            land_pos_nom = np.array([aim_xy[0], aim_xy[1], float(pos1[2])])
            land_pos, land_vel = noise.perturb_throw(
                land_pos_nom, land_vel_nom, np.zeros(3))
            # B4 (R5 sitting 4, 2026-10-04): `cfg.feed_bb_bias_mm` opens the
            # gap between what the schedule BELIEVES (`land_pos`, fed to
            # `LandingPrior` below unchanged -- the request/announced
            # point) and where the ball is PHYSICALLY spawned to land --
            # the dataclass field's own docstring names which side of the
            # real `columns_feed_bb_bias_mm` fix this characterises
            # (the uncorrected failure, not the fix itself: nothing here
            # corrects the `LandingPrior`, mirroring the fact that this
            # harness has no skill_node-equivalent correction path).
            # Zero (the default) makes `physical_land_pos` bit-for-bit
            # `land_pos` -- no behaviour change.
            bias_x, bias_y = cfg.feed_bb_bias_mm
            physical_land_pos = land_pos + np.array([bias_x, bias_y, 0.0])

            # Backward-integrate through gravity to a physical spawn point
            # `t_f` (this pattern's own flight time) before touch-down --
            # this lands the ball at `t_spawn + t_f`, the SAME wall-clock
            # instant the vertical branch above lands at (its own
            # `t0_abs_s + transit_s == t_spawn + beta + transit_s ==
            # t_spawn + t_f`, since `beta == t_f - transit_s`), so a FEED
            # run keeps the vertical branch's own proven dispatch margin
            # for ball A's first THROW while landing ball B on an oblique
            # arrival instead of a vertical one. Same backward-integration
            # shape as `run_reload_attempt`'s synthetic BB announcement
            # (see `_RELOAD_BALL_FLIGHT_LEAD_S`'s docstring for the
            # floor-clearance reasoning); at this pattern's operating point
            # (apex 0.9 m) and BB's ~11.9 deg/5.6 m/s arrival the spawn
            # point is well clear of both the floor and the platform.
            g = bal.GRAVITY_MMS2
            flight_lead_s = t_f
            spawn_vel = np.array([land_vel[0], land_vel[1],
                                  land_vel[2] + g * flight_lead_s])
            spawn_pos = np.array([
                physical_land_pos[0] - land_vel[0] * flight_lead_s,
                physical_land_pos[1] - land_vel[1] * flight_lead_s,
                physical_land_pos[2] - land_vel[2] * flight_lead_s
                - 0.5 * g * flight_lead_s * flight_lead_s])
            t_spawn = float(plant.data.time)
            plant.spawn_ball(spawn_pos, spawn_vel, ball=1)
            t_land_abs_s = t_spawn + flight_lead_s
            feed = sk.LandingPrior(pos_mm=land_pos, vel_mm_s=land_vel,
                                   t_land_abs_s=t_land_abs_s)
            sched = sk.compile_columns(pattern, feed=feed)

        ball_state = {
            # B2 (`columns_1ball_fed`): the SAME inert shape
            # `run_columns_1ball_attempt` gives its own phantom ball --
            # `expect_held=False` means `_stream_chain` never watches it for
            # a drop or counts a capture for it, exactly like a phantom
            # ball's real cup is never physically occupied.
            0: dict(estimator=BallisticEstimator(bal.G_VEC_MMS2),
                   airborne=False, next_obs_t=None,
                   expect_held=not phantom_a,
                   lost_since=None, last_obs_t=None),
            1: dict(estimator=BallisticEstimator(bal.G_VEC_MMS2),
                   airborne=True, next_obs_t=t_spawn, expect_held=False,
                   lost_since=None, last_obs_t=None),
        }
        identity = _IdentityTracker(_make_tracker(plant, ball_state),
                                    legacy=_correlator_is_legacy(cfg))
        if feed_mode:
            # Ball Butler's own announcement of its feed throw -- physically
            # released the instant it enters the scene (the sim has no BB
            # launcher node to announce it any earlier); thrower='ball_butler'
            # makes this an IDENTITY latch no jugglebot release can ever
            # satisfy (`bp.match_announced_track`'s source check).
            identity.announce(1, t_spawn, thrower='ball_butler')

        def on_release(ball_id: int, t_release_s: float) -> None:
            # B2's mirror of `run_columns_1ball_attempt`'s own filter (the
            # gate-side `SkillNode._maybe_announce` equivalent): a
            # phantom's release is never announced, so it can never mint a
            # tracker id or latch a correlation.
            if int(ball_id) in pattern.phantom_balls:
                return
            identity.announce(ball_id, t_release_s)

        ictx = _InstallCtx(rest0)
        installer = _make_installer(ictx, self.seg_cfg, self.limits, geom,
                                    on_release=on_release)
        learner = _MemoryLearner(memory, learner_cfg)
        # ``_make_observer`` takes ``ball_id`` as an argument (unlike
        # ``_make_observations``, which hardcodes ball 0) -- it is already
        # correct for columns' two simultaneous balls without change. This
        # is the ONLY thing that latches ``Experience.caught`` (``_advance_
        # outcomes``: ``caught_seen`` never latches with ``observer=None``,
        # which every experience row read as ``caught=False`` before this —
        # probed 2026-09-30, this run, seed 0: makes=5/5, drops=0, every
        # row still read ``caught=False`` without this wire).
        observer = _make_observer(plant)
        executor = ex.SkillExecutor(
            sched, installer, tracker=identity.tracker,
            catch_aim_source=self.cfg.catch_aim_source,
            learner=learner, boxes=None, resend_max_per_catch=0,
            observer=observer,
            on_experience=lambda exp: (
                memory.append(exp), throws_out.append(exp)),
            dispatch_lookahead_s=_DISPATCH_LOOKAHEAD_S)

        # A generous, bounded timeout -- the real exit is ``executor.done``
        # (``stop_check``), matching ``run_self_toss_attempt``.
        t_end = float(sched.skills[-1].t_abs_s) + 1.0
        loop = self._stream_chain(
            plant=plant, geom=geom, rcfg=self.rcfg, noise=noise,
            executor=executor, ictx=ictx, ball_state=ball_state, t_end=t_end,
            stop_check=lambda: executor.done, identity=identity)
        if reports_out is not None:
            reports_out.extend(executor.reports)
        return str(executor.end_code), loop, ictx

    def run_columns_seed(self, seed: int) -> dict:
        """One seed's whole R5 columns learner run (Unit G, plan § R5):
        repeated cheap-reset attempts (:meth:`run_columns_attempt`),
        collecting throws until ``cfg.target_throws`` experience rows have
        been observed -- the same policy the R3/R4 ``run_self_toss_seed``
        loop uses (owner decision 6, "resets are cheap"), generalised to
        columns' two simultaneous balls. Every attempt is chained from the
        first throw (there is no policy-A single-throw stage here: a
        columns schedule needs ball B already in flight from the first
        instant, so there is no "cold, one throw" shape to gate on
        ``learner_cfg.k_min`` the way a single-ball self-toss has).

        Each attempt's schedule carries the columns Stop (owner decision D2)
        as its LAST throw, which produces no experience row (see
        :meth:`run_columns_attempt`) -- so an attempt requesting ``remaining``
        more rows asks for ``n_throws = remaining + 1`` (floored at 2, since
        a Stop needs a ball already held to land on).

        Reports the SAME apex/xy band-entry criterion
        ``run_self_toss_seed`` does (:data:`XY_BAND_MM`/:data:`APEX_BAND_MM`,
        ``_monotone_verdict``), PLUS the R5 gate's own shape (plan § R5:
        "five consecutive cycles then 30 catches" -- sim's stand-in, per the
        brief this rung was built from: run >= ``target_throws`` throws per
        seed and report the longest run of consecutive ``caught=True``
        experience rows, in collection order across every attempt (a cold
        reset between attempts does not reset the streak: the metric is
        "how many throws in a row did this seed's memory catch", not
        "within one uninterrupted schedule").
        """
        cfg: SelfTossGateConfig = self.cfg
        noise = JuggleNoise(cfg.noise, seed=seed)
        tmp_dir = tempfile.mkdtemp(
            prefix='skills_gate_columns_learn_seed%d_' % seed)
        memory = mem.Memory(os.path.join(tmp_dir, 'memory.csv'))
        assert len(memory) == 0, 'a fresh tmp path must be a cold memory'

        throws: list = []
        reports: list = []
        attempts_stats: list = []
        attempts = 0
        refusals = 0
        drops_total = 0
        makes_total = 0
        installs_total = installs_accepted = 0
        end_codes = []
        plan_wall_s_all: list = []
        t_wall0 = time.time()

        while len(throws) < cfg.target_throws and attempts < cfg.max_attempts:
            remaining = cfg.target_throws - len(throws)
            n_throws = max(2, remaining + 1)
            attempts += 1
            throws_before = len(throws)
            end_code, loop, ictx = self.run_columns_attempt(
                n_throws=n_throws, memory=memory, learner_cfg=cfg.learner_cfg,
                noise=noise, throws_out=throws, reports_out=reports)
            end_codes.append(end_code)
            if end_code:
                refusals += 1
            drops_total += loop['drops']
            makes_total += loop['makes']
            installs_total += ictx.installs_total
            installs_accepted += ictx.installs_accepted
            plan_wall_s_all.extend(ictx.plan_wall_s)
            attempts_stats.append(dict(
                attempt=attempts, n_throws=n_throws, end_code=end_code,
                makes=loop['makes'], drops=loop['drops'],
                throws_collected=len(throws) - throws_before))

        rows = []
        for i, exp in enumerate(throws):
            err_xy_mm = float(1000.0 * np.linalg.norm(exp.y[:2]))
            err_apex_mm = float(1000.0 * abs(float(exp.y[2]) - cfg.apex_m))
            rows.append(dict(
                throw=i + 1, u=exp.u.tolist(), y=exp.y.tolist(),
                err_xy_mm=err_xy_mm, err_apex_mm=err_apex_mm,
                caught=bool(exp.caught), t_abs_s=float(exp.t_abs_s)))

        throws_to_band_xy = next(
            (r['throw'] for r in rows if r['err_xy_mm'] <= cfg.xy_band_mm),
            None)
        throws_to_band_apex = next(
            (r['throw'] for r in rows
             if r['err_apex_mm'] <= cfg.apex_band_mm), None)
        entered_band = bool(
            throws_to_band_xy is not None
            and throws_to_band_xy <= cfg.band_entry_throws
            and throws_to_band_apex is not None
            and throws_to_band_apex <= cfg.band_entry_throws)

        monotone_xy = _monotone_verdict(
            [r['err_xy_mm'] for r in rows], cfg.band_entry_throws,
            cfg.target_throws)
        monotone_apex = _monotone_verdict(
            [r['err_apex_mm'] for r in rows], cfg.band_entry_throws,
            cfg.target_throws)

        longest_consecutive_catches = 0
        _run = 0
        for r in rows:
            if r['caught']:
                _run += 1
                longest_consecutive_catches = max(longest_consecutive_catches,
                                                  _run)
            else:
                _run = 0

        beat = sk.beat_s(sk.flight_s(cfg.apex_m), cfg.dwell_s)
        assoc = _association_verdict(reports, end_codes, beat)

        return dict(
            seed=seed, policy='columns', pattern=cfg.pattern,
            separation_mm=cfg.separation_mm, apex_m=cfg.apex_m,
            feed_angle_deg=cfg.feed_angle_deg,
            feed_speed_mmps=cfg.feed_speed_mmps,
            throws=rows, n_throws_collected=len(rows), attempts=attempts,
            attempts_stats=attempts_stats, end_codes=end_codes,
            refusals=refusals, drops=drops_total, makes=makes_total,
            installs_total=installs_total,
            installs_accepted=installs_accepted,
            plan_wall_ms=_wall_ms_stats(plan_wall_s_all),
            throws_to_band_xy=throws_to_band_xy,
            throws_to_band_apex=throws_to_band_apex,
            entered_band=entered_band,
            monotone_xy=monotone_xy, monotone_apex=monotone_apex,
            longest_consecutive_catches=longest_consecutive_catches,
            association=assoc,
            passed=bool(drops_total == 0
                       and longest_consecutive_catches >= min(
                           30, cfg.target_throws)
                       and assoc['ok']),
            wall_s=time.time() - t_wall0)

    # ── B2 (R5 sitting 4): columns_1ball_fed -- ball A PHANTOM, ball B ──────
    # real and fed by Ball Butler (the OTHER half of columns_1ball, below).

    def run_columns_1ball_fed_attempt(self, *, n_throws: int,
                                      memory: mem.Memory,
                                      learner_cfg: lr.LearnerConfig,
                                      noise: JuggleNoise, throws_out: list,
                                      reports_out: list = None
                                      ) -> Tuple[str, dict, '_InstallCtx']:
        """B2's ``columns_1ball_fed`` trial (R5 sitting 4): a thin wrapper
        over :meth:`run_columns_attempt`'s own ``phantom_a=True`` branch --
        ball B's FEED (Ball Butler's oblique arrival, ``cfg.feed_angle_deg``
        / ``cfg.feed_speed_mmps``) is not re-implemented here, because it is
        IDENTICAL whether or not ball A is real: same spawn physics, same
        tracker/identity wiring, same installer. Requires ``cfg.
        feed_angle_deg``/``cfg.feed_speed_mmps`` both set (CLI: ``--one-ball-
        fed`` refuses otherwise, see :func:`main`) -- see that method's
        ``phantom_a`` paragraph for why a vertical spawn cannot be phantom-A.
        """
        return self.run_columns_attempt(
            n_throws=n_throws, memory=memory, learner_cfg=learner_cfg,
            noise=noise, throws_out=throws_out, reports_out=reports_out,
            phantom_a=True)

    def run_columns_1ball_fed_seed(self, seed: int) -> dict:
        """One seed of B2's ``columns_1ball_fed`` trial
        (:meth:`run_columns_1ball_fed_attempt`), chained like
        :meth:`run_columns_seed` -- cheap resets until ``cfg.target_throws``
        of ball B's own experience rows have been collected (ball A
        produces none, by construction -- ``executor._register_outcome``'s
        phantom guard, same enforcement point :meth:`run_columns_1ball_seed`
        relies on for ball A in the mirror case).

        PASS criteria, the FEED trial's own shape (:meth:`run_columns_seed`)
        applied to ball B's rows, PLUS the phantom-specific checks
        :meth:`run_columns_1ball_seed` runs for its own phantom ball:
          - no attempt ends ``REJECTED_NO_BALL``/``ABORTED_NO_RELEASE`` (the
            two a phantom's empty hand could otherwise spuriously trip).
          - zero experience rows carry ``ball_id == 0`` -- A trains nothing.
          - drops == 0 and the usual longest-consecutive-catches /
            association verdicts, computed over ball B's rows (the only
            rows there are).
        """
        cfg: SelfTossGateConfig = self.cfg
        if cfg.feed_angle_deg is None or cfg.feed_speed_mmps is None:
            raise ValueError(
                'columns_1ball_fed requires feed_angle_deg and '
                'feed_speed_mmps both set -- ball B is the only real ball '
                'and it is always fed')
        noise = JuggleNoise(cfg.noise, seed=seed)
        tmp_dir = tempfile.mkdtemp(
            prefix='skills_gate_columns1ballfed_seed%d_' % seed)
        memory = mem.Memory(os.path.join(tmp_dir, 'memory.csv'))
        assert len(memory) == 0, 'a fresh tmp path must be a cold memory'

        throws: list = []
        reports: list = []
        attempts = 0
        refusals = 0
        drops_total = makes_total = 0
        end_codes: list = []
        plan_wall_s_all: list = []
        t_wall0 = time.time()

        while len(throws) < cfg.target_throws and attempts < cfg.max_attempts:
            remaining = cfg.target_throws - len(throws)
            # Floored at 3, not 2 (`run_columns_seed`'s own floor): ball B
            # (the real one here) is at ODD schedule index, and the Stop
            # (`compile_columns`'s own cross-site last throw) lands on ball
            # B whenever `n_throws` is EVEN -- at the floor of 2 this makes
            # ball B's ONLY throw the Stop, which carries no experience row
            # (`shadow_landing`), so the attempt nets ZERO new rows and the
            # loop never converges. `rows(n) = (n - 1) // 2` for either
            # parity (worked out from `compile_columns`'s own `ball_i =
            # i % 2` / stop-index arithmetic) is >= 1 once `n >= 3`, so 3
            # is the smallest floor that always makes progress.
            n_throws = max(3, remaining + 1)
            attempts += 1
            end_code, loop, ictx = self.run_columns_1ball_fed_attempt(
                n_throws=n_throws, memory=memory, learner_cfg=cfg.learner_cfg,
                noise=noise, throws_out=throws, reports_out=reports)
            end_codes.append(end_code)
            if end_code:
                refusals += 1
            drops_total += loop['drops']
            makes_total += loop['makes']
            plan_wall_s_all.extend(ictx.plan_wall_s)

        a_rows = [exp for exp in throws if int(exp.ball_id) == 0]
        b_rows = [exp for exp in throws if int(exp.ball_id) == 1]

        rows = []
        for i, exp in enumerate(b_rows):
            err_xy_mm = float(1000.0 * np.linalg.norm(exp.y[:2]))
            err_apex_mm = float(1000.0 * abs(float(exp.y[2]) - cfg.apex_m))
            rows.append(dict(
                throw=i + 1, err_xy_mm=err_xy_mm, err_apex_mm=err_apex_mm,
                caught=bool(exp.caught)))

        throws_to_band_xy = next(
            (r['throw'] for r in rows if r['err_xy_mm'] <= cfg.xy_band_mm),
            None)
        throws_to_band_apex = next(
            (r['throw'] for r in rows
             if r['err_apex_mm'] <= cfg.apex_band_mm), None)
        monotone_xy = _monotone_verdict(
            [r['err_xy_mm'] for r in rows], cfg.band_entry_throws,
            cfg.target_throws)
        monotone_apex = _monotone_verdict(
            [r['err_apex_mm'] for r in rows], cfg.band_entry_throws,
            cfg.target_throws)

        longest_consecutive_catches = 0
        _run = 0
        for r in rows:
            if r['caught']:
                _run += 1
                longest_consecutive_catches = max(longest_consecutive_catches,
                                                  _run)
            else:
                _run = 0

        beat = sk.beat_s(sk.flight_s(cfg.apex_m), cfg.dwell_s)
        assoc = _association_verdict(reports, end_codes, beat)
        _FORBIDDEN_PHANTOM_CODES = ('REJECTED_NO_BALL', 'ABORTED_NO_RELEASE')
        no_forbidden_refusal = not any(
            ec in _FORBIDDEN_PHANTOM_CODES for ec in end_codes)

        return dict(
            seed=seed, policy='columns_1ball_fed', pattern='columns',
            apex_m=cfg.apex_m, separation_mm=cfg.separation_mm,
            feed_angle_deg=cfg.feed_angle_deg,
            feed_speed_mmps=cfg.feed_speed_mmps,
            attempts=attempts, end_codes=end_codes, refusals=refusals,
            drops=drops_total, makes=makes_total,
            plan_wall_ms=_wall_ms_stats(plan_wall_s_all),
            throws=rows, n_throws_collected=len(rows),
            a_experience_rows=len(a_rows), b_throws=len(b_rows),
            throws_to_band_xy=throws_to_band_xy,
            throws_to_band_apex=throws_to_band_apex,
            monotone_xy=monotone_xy, monotone_apex=monotone_apex,
            longest_consecutive_catches=longest_consecutive_catches,
            association=assoc,
            passed=bool(no_forbidden_refusal
                       and drops_total == 0
                       and len(a_rows) == 0 and len(b_rows) > 0
                       and longest_consecutive_catches >= min(
                           30, cfg.target_throws)
                       and assoc['ok']),
            wall_s=time.time() - t_wall0)

    # ── B2b: the columns_1ball trial (ball A real, ball B a PHANTOM) ───────

    def run_columns_1ball_attempt(self, *, n_throws: int, memory: mem.Memory,
                                  learner_cfg: lr.LearnerConfig,
                                  noise: JuggleNoise, throws_out: list,
                                  reports_out: list = None,
                                  reload: bool = False
                                  ) -> Tuple[str, dict, '_InstallCtx', int]:
        """B2b's ``columns_1ball`` trial: ball A real at site 0, ball B a
        PHANTOM at site 1 (``schedule.Pattern.phantom_balls=(1,)``) --
        columns motion with one real ball (owner's framing, ``skill_node.
        SkillNode._run_columns_1ball``'s own docstring: "JB running columns
        with just a single ball, to test the vertical throws while the
        Platform zips from side to side"). Site pick is UNSWAPPED (A at
        site 0, B phantom at site 1) -- unlike :meth:`run_columns_attempt`'s
        FEED trial, nothing is ever fed at the phantom's site, so none of
        that swap's collision/aim-walk concerns apply (mirrors
        ``_run_columns_1ball``'s own site pick, which exists only so an
        operator sees the same two named sites an ordinary columns attempt
        would).

        **Ball B is never physically spawned** -- no ``plant.spawn_ball``
        call for ball 1 anywhere in this method, in either branch below. It
        stays parked (``BallManager.reset``'s default: far above the
        scene, ``held`` forever False) for the whole attempt. The schedule
        still carries ball B's own THROW/CATCH skills (``compile_columns``
        is byte-identical in its own MOTION output whether or not
        ``phantom_balls`` is set -- B2's own contract,
        ``schedule.Pattern.phantom_balls``'s docstring): the cup still
        transits to B's site and the hand still strokes there, so the legs
        see identical dynamics to a real two-ball columns attempt -- this
        method's own return value (below) lets the caller confirm those
        skills actually DISPATCHED as motion, not merely that the compiled
        schedule contains them.

        ``reload=False`` (the default) compiles the columns schedule
        directly, ball A assumed already resting at site 0 -- mirrors
        ``_run_columns_1ball``'s own non-reload branch and this gate's own
        :meth:`run_columns_attempt` non-FEED branch (the same full-beat
        dispatch margin). ``reload=True`` instead runs ball A through the
        SAME synthetic Ball-Butler arrival :meth:`run_reload_attempt` uses
        (``_RELOAD_ARRIVAL_ANGLE_DEG`` / ``_RELOAD_ARRIVAL_SPEED_MMPS``,
        backward-integrated to a physical spawn point), then hands the
        DECAY REST's end off into the columns tail via
        ``schedule.compile_reload_columns`` -- mirrors
        ``_run_columns_1ball``'s own ``reload=True`` branch, which calls
        ``_start_reload(..., kind='columns_1ball', columns_pattern=
        pattern)`` on the real node; the sim gate has no ``_start_reload``,
        so this inlines :meth:`run_reload_attempt`'s own synthetic-
        announcement shape against the one-ball-columns compiler instead of
        a second ``compile_reload`` call. ``lift_s`` is left at
        ``schedule.FLOOR_LIFT_S`` (the already-home case) -- the same choice
        :meth:`run_reload_attempt`'s own ``OneBallPattern`` makes implicitly
        (its default ``floor_lift_s``), since this gate always resets from
        a level rest with the hand at home.

        **The identity tracker never announces or latches ball B** -- the
        gate's OWN enforcement, parallel to ``SkillNode._maybe_announce``'s
        real one (``schedule.Schedule.is_phantom``'s docstring): the
        ``on_release`` closure below drops any release whose ``ball_id`` is
        in ``pattern.phantom_balls`` before it ever reaches
        :meth:`_IdentityTracker.announce`, so this gate's own
        correlation/association bookkeeping sees exactly what the real
        robot's tracker would -- never a track minted for a ball that was
        never thrown.

        Returns ``(end_code, loop_stats, ictx, b_skills_dispatched)`` --
        ``end_code`` is ``''`` for an attempt that ran its whole schedule
        with no refusal (the SAME contract :meth:`run_columns_attempt`
        returns, which also means every skill in the compiled schedule --
        A's and B's -- reached dispatch); ``b_skills_dispatched`` is the
        number of ball B's own THROW/CATCH skills the compiled schedule
        carries, counted off the schedule itself (not a second, separate
        tally), so the caller can assert "B's skills executed as motion"
        against the SAME number the executor actually dispatched.
        """
        cfg: SelfTossGateConfig = self.cfg
        plant = self.plant
        geom = self.geom
        site0, site1 = sites.columns_sites(cfg.separation_mm)
        # Unswapped: A at site 0, B (phantom) at site 1 -- see the method
        # docstring (no feed exists to collide with, unlike the FEED
        # trial's own swap).
        a_site, phantom_site = site0, site1
        pattern = sk.Pattern(sites=(a_site, phantom_site), apex_m=cfg.apex_m,
                             dwell_s=cfg.dwell_s, n_throws=n_throws,
                             phantom_balls=(1,))

        rest0 = self._rest_state(a_site)
        pose0 = np.asarray(rest0.pose, dtype=float)
        plant.reset(pose0)
        plant.command(plant.pose_to_extensions(pose0))
        plant.command_hand(stream_chain.slider_mm_of_rev(
            float(rest0.hand_rev), self.rcfg))
        for _ in range(40):
            plant.step(KNOT_DT_S)
            if self.viewer is not None:
                self.viewer.sync()

        pending_spawn = None
        if not reload:
            # Ball A assumed already held at site 0 -- mirrors
            # `run_columns_attempt`'s own non-FEED spawn.
            plant.ball_manager.ball(0).spawn_in_hand()
            t_f = sk.flight_s(cfg.apex_m)
            beta = sk.beat_s(t_f, cfg.dwell_s)
            # A full beat of margin before dispatch -- the same idiom
            # `run_columns_attempt` uses for its own vertical spawn, ample
            # for `launch_s` + `LEAD_S` + dispatch lookahead.
            t0_abs_s = float(plant.data.time) + beta
            # The phantom's first catch mirrors the FED start's aim point
            # and level pin (`sk.phantom_feed_prior`, the node's own
            # `_run_columns_1ball` does the same): at the nominal site it
            # refused LIMIT_JERK on every seed at the sitting-4 geometry
            # (2026-10-04) while the fed start passed 5/5.
            a_minus_ph_xy = (np.asarray(a_site.cup_mm[:2], dtype=float)
                             - np.asarray(phantom_site.cup_mm[:2], dtype=float))
            _norm = float(np.linalg.norm(a_minus_ph_xy))
            _unit = a_minus_ph_xy / _norm if _norm > 1e-9 else np.zeros(2)
            _aim_xy = (np.asarray(phantom_site.cup_mm[:2], dtype=float)
                       + cfg.feed_aim_toward_a_mm * _unit)
            aim_site = sites.Site(phantom_site.name, np.array(
                [_aim_xy[0], _aim_xy[1], float(phantom_site.cup_mm[2])]))
            sched = sk.compile_columns(
                pattern, feed=sk.phantom_feed_prior(pattern, aim_site, t0_abs_s))
            ball_state_0 = dict(
                estimator=BallisticEstimator(bal.G_VEC_MMS2), airborne=False,
                next_obs_t=None, expect_held=True, lost_since=None,
                last_obs_t=None)
        else:
            # Ball A is fed externally, exactly as `run_reload_attempt`'s
            # own synthetic Ball Butler arrival -- see the method docstring.
            angle = math.radians(self._RELOAD_ARRIVAL_ANGLE_DEG)
            speed = self._RELOAD_ARRIVAL_SPEED_MMPS
            land_pos_nom = np.asarray(a_site.catch_site_mm(), dtype=float)
            land_vel_nom = np.array([speed * math.cos(angle), 0.0,
                                     -speed * math.sin(angle)])
            land_pos, land_vel = noise.perturb_throw(
                land_pos_nom, land_vel_nom, np.zeros(3))
            t0_abs_s = float(plant.data.time)
            t_land_abs_s = t0_abs_s + self._RELOAD_ANNOUNCE_LEAD_S
            g = bal.GRAVITY_MMS2
            flight_lead_s = self._RELOAD_BALL_FLIGHT_LEAD_S
            spawn_vel = np.array([land_vel[0], land_vel[1],
                                  land_vel[2] + g * flight_lead_s])
            spawn_pos = np.array([
                land_pos[0] - land_vel[0] * flight_lead_s,
                land_pos[1] - land_vel[1] * flight_lead_s,
                land_pos[2] - land_vel[2] * flight_lead_s
                - 0.5 * g * flight_lead_s * flight_lead_s])
            spawn_t_abs_s = t_land_abs_s - flight_lead_s
            pending_spawn = (spawn_t_abs_s, spawn_pos, spawn_vel, 0)
            sched = sk.compile_reload_columns(
                land_pos, land_vel, t_land_abs_s, sk.FLOOR_LIFT_S, pattern,
                t0_abs_s)
            ball_state_0 = dict(
                estimator=BallisticEstimator(bal.G_VEC_MMS2), airborne=False,
                next_obs_t=None, expect_held=False, lost_since=None,
                last_obs_t=None)

        b_skills_dispatched = sum(
            1 for s in sched.skills
            if s.kind in (THROW, CATCH) and int(s.ball_id) == 1)

        ball_state = {
            0: ball_state_0,
            # Ball 1 (phantom) is never spawned -- this entry exists only
            # so `_stream_chain`'s `ball_state[b_id]` lookups (the
            # release-pop loop, the per-ball make/drop/observe loop) don't
            # KeyError on B's own scheduled releases: a parked ball's
            # `held` reads False forever, so `_stream_chain` never actually
            # calls `plant.release_ball`/counts a capture for it, and
            # `expect_held=False` here means it is never watched for a drop
            # either -- every field below stays physically inert.
            1: dict(estimator=BallisticEstimator(bal.G_VEC_MMS2),
                   airborne=False, next_obs_t=None, expect_held=False,
                   lost_since=None, last_obs_t=None),
        }
        identity = _IdentityTracker(_make_tracker(plant, ball_state),
                                    legacy=_correlator_is_legacy(cfg))

        def on_release(ball_id: int, t_release_s: float) -> None:
            # THE enforcement point (gate-side mirror of `SkillNode.
            # _maybe_announce`'s real one): a phantom's release is never
            # announced, so it can never mint a tracker id or latch a
            # correlation.
            if int(ball_id) in pattern.phantom_balls:
                return
            identity.announce(ball_id, t_release_s)

        ictx = _InstallCtx(rest0)
        installer = _make_installer(ictx, self.seg_cfg, self.limits, geom,
                                    on_release=on_release)
        learner = _MemoryLearner(memory, learner_cfg)
        observer = _make_observer(plant)
        executor = ex.SkillExecutor(
            sched, installer, tracker=identity.tracker,
            catch_aim_source=self.cfg.catch_aim_source,
            learner=learner, boxes=None, resend_max_per_catch=0,
            observer=observer,
            on_experience=lambda exp: (
                memory.append(exp), throws_out.append(exp)),
            dispatch_lookahead_s=_DISPATCH_LOOKAHEAD_S)

        # A generous, bounded timeout -- the real exit is `executor.done`
        # (`stop_check`), matching `run_columns_attempt`.
        t_end = float(sched.skills[-1].t_abs_s) + 1.0
        loop = self._stream_chain(
            plant=plant, geom=geom, rcfg=self.rcfg, noise=noise,
            executor=executor, ictx=ictx, ball_state=ball_state, t_end=t_end,
            stop_check=lambda: executor.done, identity=identity,
            pending_spawn=pending_spawn)
        if reports_out is not None:
            reports_out.extend(executor.reports)
        return str(executor.end_code), loop, ictx, b_skills_dispatched

    def run_columns_1ball_seed(self, seed: int, *, reload: bool = False
                               ) -> dict:
        """One seed of B2b's ``columns_1ball`` trial
        (:meth:`run_columns_1ball_attempt`), chained like
        :meth:`run_columns_seed` -- cheap resets until ``cfg.target_throws``
        of ball A's OWN experience rows have been collected (ball B
        produces none, by construction -- see that method's docstring).

        PASS criteria (``handoff_B2_sim_trial.md`` § "Pass criteria",
        confirmed against the REAL plant/hand rather than only the offline
        unit tests):
          - every attempt's ``end_code`` is ``''`` -- no refusal of any
            kind, in particular never ``REJECTED_NO_BALL``/
            ``ABORTED_NO_RELEASE`` (the two a phantom's empty hand could
            otherwise spuriously trip -- see ``executor.py::
            _register_outcome``'s and ``schedule.py::Schedule.is_phantom``'s
            docstrings for why they structurally cannot).
          - ball A caught every throw it produced a row for (``n``/``n``),
            with the same band/monotone-convergence verdicts
            :meth:`run_columns_seed` reports (computed over A's rows only).
          - ball B's own THROW/CATCH skills dispatched as motion at least
            once (``b_skills_dispatched``, summed across attempts).
          - zero experience rows carry ``ball_id == 1`` -- B trains nothing.
          - the usual association verdict (:func:`_association_verdict`).
        """
        cfg: SelfTossGateConfig = self.cfg
        noise = JuggleNoise(cfg.noise, seed=seed)
        tmp_dir = tempfile.mkdtemp(
            prefix='skills_gate_columns1ball_seed%d_' % seed)
        memory = mem.Memory(os.path.join(tmp_dir, 'memory.csv'))
        assert len(memory) == 0, 'a fresh tmp path must be a cold memory'

        throws: list = []
        reports: list = []
        attempts = 0
        refusals = 0
        drops_total = makes_total = 0
        b_skills_dispatched_total = 0
        end_codes: list = []
        plan_wall_s_all: list = []
        t_wall0 = time.time()

        while len(throws) < cfg.target_throws and attempts < cfg.max_attempts:
            remaining = cfg.target_throws - len(throws)
            n_throws = max(2, remaining + 1)
            attempts += 1
            end_code, loop, ictx, b_skills = self.run_columns_1ball_attempt(
                n_throws=n_throws, memory=memory, learner_cfg=cfg.learner_cfg,
                noise=noise, throws_out=throws, reports_out=reports,
                reload=reload)
            end_codes.append(end_code)
            if end_code:
                refusals += 1
            drops_total += loop['drops']
            makes_total += loop['makes']
            b_skills_dispatched_total += b_skills
            plan_wall_s_all.extend(ictx.plan_wall_s)

        b_rows = [exp for exp in throws if int(exp.ball_id) == 1]
        a_rows = [exp for exp in throws if int(exp.ball_id) == 0]
        a_caught = sum(1 for exp in a_rows if exp.caught)

        rows = []
        for i, exp in enumerate(a_rows):
            err_xy_mm = float(1000.0 * np.linalg.norm(exp.y[:2]))
            err_apex_mm = float(1000.0 * abs(float(exp.y[2]) - cfg.apex_m))
            rows.append(dict(
                throw=i + 1, err_xy_mm=err_xy_mm, err_apex_mm=err_apex_mm,
                caught=bool(exp.caught)))

        throws_to_band_xy = next(
            (r['throw'] for r in rows if r['err_xy_mm'] <= cfg.xy_band_mm),
            None)
        throws_to_band_apex = next(
            (r['throw'] for r in rows
             if r['err_apex_mm'] <= cfg.apex_band_mm), None)
        monotone_xy = _monotone_verdict(
            [r['err_xy_mm'] for r in rows], cfg.band_entry_throws,
            cfg.target_throws)
        monotone_apex = _monotone_verdict(
            [r['err_apex_mm'] for r in rows], cfg.band_entry_throws,
            cfg.target_throws)

        longest_consecutive_catches = 0
        _run = 0
        for r in rows:
            if r['caught']:
                _run += 1
                longest_consecutive_catches = max(longest_consecutive_catches,
                                                  _run)
            else:
                _run = 0

        beat = sk.beat_s(sk.flight_s(cfg.apex_m), cfg.dwell_s)
        assoc = _association_verdict(reports, end_codes, beat)
        _FORBIDDEN_PHANTOM_CODES = ('REJECTED_NO_BALL', 'ABORTED_NO_RELEASE')
        no_forbidden_refusal = not any(
            ec in _FORBIDDEN_PHANTOM_CODES for ec in end_codes)

        return dict(
            seed=seed, policy='columns_1ball', pattern='columns',
            reload=bool(reload), apex_m=cfg.apex_m,
            separation_mm=cfg.separation_mm,
            attempts=attempts, end_codes=end_codes, refusals=refusals,
            drops=drops_total, makes=makes_total,
            plan_wall_ms=_wall_ms_stats(plan_wall_s_all),
            a_throws=len(a_rows), a_caught=a_caught,
            b_experience_rows=len(b_rows),
            b_skills_dispatched=b_skills_dispatched_total,
            throws_to_band_xy=throws_to_band_xy,
            throws_to_band_apex=throws_to_band_apex,
            monotone_xy=monotone_xy, monotone_apex=monotone_apex,
            longest_consecutive_catches=longest_consecutive_catches,
            association=assoc,
            passed=bool(refusals == 0 and no_forbidden_refusal
                       and drops_total == 0
                       and len(a_rows) > 0 and a_caught == len(a_rows)
                       and len(b_rows) == 0
                       and b_skills_dispatched_total > 0
                       and assoc['ok']),
            wall_s=time.time() - t_wall0)

    # ── R4: the reload trial (Unit U3, brief step 5, owner decision D4) ────

    #: BB's arrival condition this gate spawns synthetically -- an external
    #: throw at 25 deg below horizontal, 4.8 m/s, toward `sites[0]`'s catch
    #: point (`brief_U3_reload_skills.md` step 5's own numbers).
    _RELOAD_ARRIVAL_ANGLE_DEG = 25.0
    _RELOAD_ARRIVAL_SPEED_MMPS = 4800.0
    #: How long BEFORE the arrival this gate spawns the ball, backward-
    #: integrated through gravity from the arrival condition (constant
    #: horizontal velocity, only the vertical component and the position
    #: move). This is NOT a claim about BB's own real flight time -- this
    #: gate has no BB node, so nothing here simulates a launcher or the
    #: `bb/throw_at_target` round trip, only the physical ARRIVAL
    #: `schedule.compile_reload` is built from (the synthetic announcement).
    #: Sized so `compile_reload`'s own window check clears with margin: an
    #: already-home hand needs PRE-TILT REST >= max(FLOOR_LIFT_S, PRETILT_S)
    #: + RELOAD_CATCH_WINDOW_S + 2·LEAD_S = 1.5 + 0.5 + 2·0.225 =~ 2.45 s
    #: (two leads since 2026-09-29: the REST's own dispatch lead, and the
    #: CATCH now dispatching only once that REST has ENDED)
    #: (`schedule` module constants), so the synthetic announcement arrives
    #: this many seconds before the ball does -- exactly mirroring BB's own
    #: announcement preceding its physical release by `throw_delay_s`.
    _RELOAD_ANNOUNCE_LEAD_S = 4.0

    #: How long BEFORE touch-down the ball is PHYSICALLY spawned into the
    #: MuJoCo scene -- deliberately NOT `_RELOAD_ANNOUNCE_LEAD_S`.
    #:
    #: ROOT-CAUSED 2026-09-24 (`scratchpad/probe_r4_sim_reload_capture.py`,
    #: seed 0): the two are different physical facts BB's own announcement
    #: keeps separate (`throw_delay_s` before the physical release, THEN the
    #: ball's own flight time) but this gate's first cut conflated -- it
    #: spawned the ball AT the announcement instant, backward-integrating
    #: `_RELOAD_ANNOUNCE_LEAD_S` (4.0 s) of free fall from the 25 deg /
    #: 4.8 m/s arrival. That arrival's OWN apex is only 1039.8 mm above the
    #: landing site's `CATCH_CUP_Z_MM` height (830 mm) -- `t_apex =
    #: |v_z_land| / g = 0.207 s` -- so the spawn height as a function of the
    #: backward time `dt` is `830 + 2028.568*dt - 4903*dt**2` (mm), which
    #: crosses the world floor (`sim/model/jugglebot.xml`'s `floor` plane,
    #: `pos="0 0 -0.082"`, an INFINITE MuJoCo collision half-space regardless
    #: of its finite visual `size`) at `dt = 0.668 s` and goes deeply negative
    #: past that -- at 4.0 s the spawn point was **69.5 m below the floor**.
    #: MuJoCo does not refuse an interpenetrating spawn; its contact solver
    #: reads the ~69 m penetration as a proportionally huge restoring force
    #: (`ball_geom`'s soft `solref="0.05 2.0"`) and rockets the ball to
    #: z ~= 1.31e6 mm (~1.3 km) by `t_land`, still climbing at ~328 m/s
    #: (measured, `scratchpad/probe_r4_sim_reload_capture.py`, seed 0: an
    #: isolated single-ball check the same session found the contact-driven
    #: acceleration already at ~1177x gravity 2 ms after spawn) -- the ball
    #: is nowhere near the cup at touch-down and no MuJoCo contact with any
    #: `hand_collision_*` geom is EVER seen (0 contact events across the
    #: whole attempt), hence `makes=0` and the `REJECTED_NO_BALL` at the
    #: first THROW (there is nothing seated to throw). This is independent
    #: of `_RELOAD_ANNOUNCE_LEAD_S`, which stays
    #: 4.0 s -- ONLY the announcement-to-landing schedule margin
    #: `compile_reload` needs; it is not a claim about the ball's own hang
    #: time either, and this gate's arrival condition cannot physically
    #: sustain 4.0 s of free flight from any above-ground point.
    #:
    #: 0.35 s clears the floor with ~940 mm of margin (`spawn_z(0.35 s) =
    #: 939.4 mm`, well inside the 0-666 mm window that keeps the spawn point
    #: above ground) while still landing ON the SAME announced arrival
    #: condition -- `run_reload_attempt` computes this spawn on its own
    #: (SHORTER) backward integration and injects it mid-stream via
    #: `_stream_chain`'s `pending_spawn`, at the simulated instant
    #: `t_land_abs_s - _RELOAD_BALL_FLIGHT_LEAD_S`, rather than at `t0_abs_s`.
    _RELOAD_BALL_FLIGHT_LEAD_S = 0.35

    def run_reload_attempt(self, *, site: 'sites.Site', noise: JuggleNoise,
                           n_throws: int = 1
                           ) -> Tuple[str, dict, '_InstallCtx']:
        """One R4 reload attempt (Unit U3, brief step 5): the platform
        starts at a level REST at ``site`` with the cup EMPTY (the bridge
        REST's own end state, ``SkillNode._start_reload``, is OUT OF SCOPE
        for this gate -- a level REST at a site is already well covered:
        `tests/motion/test_skills_schedule.py`'s `compile_reload` suite and
        `tests/ros/test_skill_node.py`'s reload tests pin it). A ball is
        spawned already in flight on a ballistic arc that arrives at
        ``site``'s catch point at ``_RELOAD_ARRIVAL_ANGLE_DEG`` /
        ``_RELOAD_ARRIVAL_SPEED_MMPS`` -- the SYNTHETIC ANNOUNCEMENT: those
        arrival physics (position, velocity, the wall-clock landing instant)
        ARE the announcement ``schedule.compile_reload`` is compiled from,
        exactly as ``SkillNode._on_announcement`` compiles it from BB's real
        one. The compiled schedule then streams through the SAME one chain
        (:meth:`_stream_chain`) every other trial in this file uses.

        Returns ``(end_code, loop_stats, ictx)`` -- ``end_code`` is ``''``
        for an attempt that ran its whole schedule with no refusal.
        """
        plant = self.plant
        geom = self.geom
        rest0 = self._rest_state(site)
        pose0 = np.asarray(rest0.pose, dtype=float)
        plant.reset(pose0)
        plant.command(plant.pose_to_extensions(pose0))
        plant.command_hand(stream_chain.slider_mm_of_rev(
            float(rest0.hand_rev), self.rcfg))
        for _ in range(40):
            plant.step(KNOT_DT_S)
            if self.viewer is not None:
                self.viewer.sync()

        angle = math.radians(self._RELOAD_ARRIVAL_ANGLE_DEG)
        speed = self._RELOAD_ARRIVAL_SPEED_MMPS
        land_pos_nom = np.asarray(site.catch_site_mm(), dtype=float)
        land_vel_nom = np.array([speed * math.cos(angle), 0.0,
                                 -speed * math.sin(angle)])
        land_pos, land_vel = noise.perturb_throw(
            land_pos_nom, land_vel_nom, np.zeros(3))

        # The synthetic announcement's OWN instant -- "now" to
        # `compile_reload`, exactly like `SkillNode._on_announcement`'s
        # `now = self.get_clock().now()...` at the moment the announcement
        # arrives.
        t0_abs_s = float(plant.data.time)
        t_land_abs_s = t0_abs_s + self._RELOAD_ANNOUNCE_LEAD_S

        # Backward-integrate the arrival condition through gravity to a
        # spawn point `_RELOAD_BALL_FLIGHT_LEAD_S` earlier on the SAME
        # parabola -- NOT `_RELOAD_ANNOUNCE_LEAD_S` (see that constant's
        # docstring: at this arrival condition 4.0 s of backward free fall
        # lands the spawn point 69.5 m under the world floor). The ball is
        # not spawned here -- `_stream_chain`'s `pending_spawn` injects it at
        # `spawn_t_abs_s`, once the sim clock actually reaches it, so the
        # PRE-TILT REST streams first exactly as it would waiting for BB's
        # real, later physical release.
        g = bal.GRAVITY_MMS2
        flight_lead_s = self._RELOAD_BALL_FLIGHT_LEAD_S
        spawn_vel = np.array([land_vel[0], land_vel[1],
                              land_vel[2] + g * flight_lead_s])
        spawn_pos = np.array([
            land_pos[0] - land_vel[0] * flight_lead_s,
            land_pos[1] - land_vel[1] * flight_lead_s,
            land_pos[2] - land_vel[2] * flight_lead_s
            - 0.5 * g * flight_lead_s * flight_lead_s])
        spawn_t_abs_s = t_land_abs_s - flight_lead_s
        pending_spawn = (spawn_t_abs_s, spawn_pos, spawn_vel, 0)

        pattern = sk.OneBallPattern(sites=(site,), apex_m=self.cfg.apex_m,
                                    dwell_s=self.cfg.dwell_s,
                                    n_throws=n_throws)
        sched = sk.compile_reload(land_pos, land_vel, t_land_abs_s, pattern,
                                  t0_abs_s)

        # `airborne=False` / `next_obs_t=None` until `pending_spawn` actually
        # fires -- there is no ball in the scene yet at `t0_abs_s` (BB has
        # only just announced), so nothing here should be "observed".
        ball_state = {0: dict(estimator=BallisticEstimator(bal.G_VEC_MMS2),
                             airborne=False, next_obs_t=None,
                             expect_held=False, lost_since=None,
                             last_obs_t=None)}
        tracker = _make_tracker(plant, ball_state)
        ictx = _InstallCtx(rest0)
        installer = _make_installer(ictx, self.seg_cfg, self.limits, geom)
        observer = _make_observer(plant)
        observations = _make_observations(observer)
        # The config's own aim source -- the LIVE default is AIM_TRACKER.
        # HISTORY (2026-09-23): this was hard-coded to AIM_SCHEDULE for one
        # session because `_make_tracker`'s fit of the then-4 s synthetic
        # flight aimed the catch at x = 3.06e5 mm; that flight was the
        # spawn-below-the-floor defect `_RELOAD_BALL_FLIGHT_LEAD_S` fixed, and
        # with the corrected 0.35 s flight the trial PASSES 5/5 seeds under
        # AIM_TRACKER with tracker re-aims accepted (7-8 installs per seed,
        # 2026-09-24 00:24). The executor now also bounds a tracker fit for an
        # externally announced ball in TIME against the announcement
        # (`executor.EXTERNAL_LANDING_TIME_BAND_S`), so an immature fit can
        # no longer move a reload catch's window.
        executor = ex.SkillExecutor(
            sched, installer, tracker=tracker,
            catch_aim_source=self.cfg.catch_aim_source,
            observer=observer, observations=observations,
            dispatch_lookahead_s=_DISPATCH_LOOKAHEAD_S)

        # A generous, bounded timeout -- the real exit is `executor.done`
        # (`stop_check`), same reason `run_self_toss_attempt` gives.
        t_end = float(sched.skills[-1].t_abs_s) + 1.0
        loop = self._stream_chain(
            plant=plant, geom=geom, rcfg=self.rcfg, noise=noise,
            executor=executor, ictx=ictx, ball_state=ball_state, t_end=t_end,
            release_bias=_measured_release_bias,
            stop_check=lambda: executor.done, pending_spawn=pending_spawn)
        return str(executor.end_code), loop, ictx

    def run_reload_seed(self, seed: int) -> dict:
        """One seed of the R4 reload gate (Unit U3, brief step 5): a SINGLE
        attempt -- no cheap-reset chaining like :meth:`run_self_toss_seed`,
        because this gate is about the reload CHOREOGRAPHY capturing the
        externally-thrown ball, not learner convergence (no learner/boxes
        are wired -- the pattern's continuation throws run open-loop, ``u =
        y_d``, the R2-and-earlier behaviour). PASS = the schedule's one
        reload CATCH is a make (kinematic capture) with zero pump rejects
        across the whole run and no attempt-ending refusal."""
        cfg: SelfTossGateConfig = self.cfg
        site = sites.columns_sites(_SELF_TOSS_SEPARATION_MM)[0]
        noise = JuggleNoise(cfg.noise, seed=seed)
        t_wall0 = time.time()
        end_code, loop, ictx = self.run_reload_attempt(site=site, noise=noise)
        pump_clean = bool(loop['pump_rejects'] == 0
                          and loop['accepted'] == loop['emitted']
                          and loop['emitted'] > 0)
        caught_all = bool(loop['makes'] >= 1)
        no_drops = bool(loop['drops'] == 0)
        passed = bool(pump_clean and caught_all and no_drops and not end_code)
        return dict(
            seed=seed, end_code=end_code, makes=loop['makes'],
            drops=loop['drops'], pump_rejects=loop['pump_rejects'],
            pump_frames_emitted=loop['emitted'],
            pump_frames_accepted=loop['accepted'],
            installs_total=ictx.installs_total,
            installs_accepted=ictx.installs_accepted,
            pump_clean=pump_clean, caught_all=caught_all, no_drops=no_drops,
            passed=passed, wall_s=time.time() - t_wall0)

    # ── run + summarise ───────────────────────────────────────────────────

    def run(self) -> dict:
        t0 = time.time()
        results = [self.run_trial(s) for s in self.cfg.seeds]
        wall_s = time.time() - t0

        all_wall_s = [w for r in results for w in r.plan_wall_s]

        report = {
            'gate': 'skills',
            'passed': bool(results and all(r.passed for r in results)),
            'seeds': list(self.cfg.seeds),
            'n_throws': self.cfg.n_throws,
            'apex_m': self.cfg.apex_m,
            'separation_mm': self.cfg.separation_mm,
            'dwell_s': self.cfg.dwell_s,
            'wall_s': wall_s,
            'plan_wall_ms': _wall_ms_stats(all_wall_s),
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
    """The R3/R4 sim-validation run (plan § 4 R3 "Sim validation"; R4 Unit
    U7a generalises it to ``cfg.pattern='hop'``; R5 Unit G to
    ``cfg.pattern='columns'``, :meth:`SkillsGate.run_columns_seed` -- both
    balls, the columns Stop, no ``policy`` (columns has no policy-A
    single-throw stage, see that method's docstring)): one ``SkillsGate``
    built for ``cfg``'s operating point, one seed at a time, writing the
    report JSON the brief this rung was built from asks for. Never shares
    the gate instance with :func:`run_gate` (the R2 identity-prior columns
    trial)."""
    cfg = SelfTossGateConfig() if cfg is None else cfg
    gate = SkillsGate(cfg)
    t0 = time.time()
    if cfg.pattern == 'columns' and cfg.one_ball:
        seed_results = [gate.run_columns_1ball_seed(s, reload=cfg.one_ball_reload)
                        for s in seeds]
    elif cfg.pattern == 'columns' and cfg.one_ball_fed:
        seed_results = [gate.run_columns_1ball_fed_seed(s) for s in seeds]
    elif cfg.pattern == 'columns':
        seed_results = [gate.run_columns_seed(s) for s in seeds]
    else:
        seed_results = [gate.run_self_toss_seed(s, policy=policy)
                        for s in seeds]
    wall_s = time.time() - t0
    report = {
        'gate': 'skills_learn',
        'policy': policy,
        'pattern': cfg.pattern,
        'passed': bool(seed_results and all(r['passed'] for r in seed_results)),
        'seeds': list(seeds),
        'apex_m': cfg.apex_m,
        'dwell_s': cfg.dwell_s,
        'separation_mm': cfg.separation_mm,
        'feed_angle_deg': cfg.feed_angle_deg,
        'feed_speed_mmps': cfg.feed_speed_mmps,
        'xy_band_mm': cfg.xy_band_mm,
        'apex_band_mm': cfg.apex_band_mm,
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


def run_reload_gate(cfg: SelfTossGateConfig = None,
                    seeds=(0, 1, 2, 3, 4)) -> dict:
    """The R4 reload sim gate (Unit U3, brief step 5): spawns a synthetic
    Ball Butler arrival (25 deg / 4.8 m/s toward ``sites[0]``) at seeds 0-4
    and runs ``schedule.compile_reload``'s schedule through the real install
    chain. PASS per seed = caught (kinematic capture) with zero pump
    rejects; the whole gate passes when every seed does. Reuses
    ``SelfTossGateConfig`` (the R3 self-toss operating point: apex 0.9 m,
    dwell 0.30 s, session limits 300/5000/200000/3500) -- the reload's own
    tail throws are kinematically ordinary self-toss throws at the SAME
    site (`compile_reload`'s own docstring), so this is the right box/limit
    point for them, not a third one."""
    cfg = SelfTossGateConfig() if cfg is None else cfg
    gate = SkillsGate(cfg)
    t0 = time.time()
    seed_results = [gate.run_reload_seed(s) for s in seeds]
    wall_s = time.time() - t0
    report = {
        'gate': 'skills_reload',
        'passed': bool(seed_results and all(r['passed']
                                            for r in seed_results)),
        'seeds': list(seeds),
        'apex_m': cfg.apex_m,
        'dwell_s': cfg.dwell_s,
        'arrival_angle_deg': SkillsGate._RELOAD_ARRIVAL_ANGLE_DEG,
        'arrival_speed_mmps': SkillsGate._RELOAD_ARRIVAL_SPEED_MMPS,
        'wall_s': wall_s,
        'seed_results': seed_results,
    }
    path = cfg.report_path
    if path is None:
        out_dir = os.path.join(_repo_root, 'temp', 'reports')
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(
            out_dir, 'skills_gate_reload_%s.json'
            % time.strftime('%Y%m%dT%H%M%S'))
    with open(path, 'w') as fh:
        json.dump(report, fh, indent=2)
    report['report_path'] = path
    return report


def _print_reload_table(rep: dict) -> None:
    print('[skills_gate_reload] arrival %.1f deg / %.1f m/s  apex %.2f m  '
          'dwell %.2f s'
          % (rep['arrival_angle_deg'], rep['arrival_speed_mmps'] / 1000.0,
             rep['apex_m'], rep['dwell_s']))
    print('[skills_gate_reload] %-6s %-8s %-6s %-6s %-13s %-9s %s'
          % ('seed', 'verdict', 'makes', 'drops', 'pump_rejects',
             'installs', 'end_code'))
    for r in rep['seed_results']:
        verdict = 'PASS' if r['passed'] else 'FAIL'
        print('[skills_gate_reload] %-6d %-8s %-6d %-6d %-13d %-9s %s'
              % (r['seed'], verdict, r['makes'], r['drops'],
                 r['pump_rejects'],
                 '%d/%d' % (r['installs_accepted'], r['installs_total']),
                 r['end_code'] or '-'))
    print('[skills_gate_reload] %s  (wall %.1f s over %d seed(s))  -> %s'
          % ('PASS' if rep['passed'] else 'FAIL', rep['wall_s'],
             len(rep['seeds']), rep.get('report_path')))


def _print_learn_table(rep: dict) -> None:
    print('[skills_gate_learn] pattern %s  policy %s  apex %.2f m  dwell '
          '%.2f s  separation %.0f mm  band %.0f mm xy / %.0f mm apex  '
          'entry<=%d throws  window<=%d throws'
          % (rep.get('pattern', 'self_toss'), rep['policy'], rep['apex_m'],
             rep['dwell_s'], rep.get('separation_mm', float('nan')),
             rep['xy_band_mm'], rep['apex_band_mm'], rep['band_entry_throws'],
             rep['target_throws']))
    print('[skills_gate_learn] %-6s %-8s %-10s %-10s %-9s %-9s %-9s %-6s %-6s '
          '%-9s %-9s'
          % ('seed', 'verdict', 'band_xy', 'band_apex', 'mono_xy', 'mono_apex',
             'attempts', 'drops', 'makes', 'plan_p50', 'longest'))
    for r in rep['seed_results']:
        verdict = 'PASS' if r['passed'] else 'FAIL'
        pw = r.get('plan_wall_ms')
        p50 = pw['p50'] if pw else float('nan')
        print('[skills_gate_learn] %-6d %-8s %-10s %-10s %-9s %-9s %-9d %-6d '
              '%-6d %-9.2f %-9s'
              % (r['seed'], verdict, r['throws_to_band_xy'],
                 r['throws_to_band_apex'], r['monotone_xy'],
                 r['monotone_apex'], r['attempts'], r['drops'], r['makes'],
                 p50, r.get('longest_consecutive_catches', '-')))
        assoc = r.get('association')
        if assoc is not None:
            print('[skills_gate_learn]   assoc=%s window_too_short=%s '
                  'wrong_ball_rows=%d release_err_max_s=%.4f (bound %.4f, '
                  'bad %d/%d)'
                  % ('OK' if assoc['ok'] else 'FAIL',
                     assoc['window_too_short'], assoc['wrong_ball_rows'],
                     assoc['release_err_max_s'], assoc['release_err_bound_s'],
                     assoc['release_err_bad'], assoc['release_err_checked']))
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
                   help='site separation (default: the operating point -- '
                        '100 mm columns/self_toss, 250 mm --learn --pattern hop)')
    p.add_argument('--apex-m', type=float, default=None,
                   help='pattern apex height, m above the catch plane '
                        '(default: the operating point, 0.9 m)')
    p.add_argument('--target-throws', type=int, default=None,
                   help='--learn only: throws to collect per seed (default: '
                        'the operating point, 25; R5 columns\' own gate '
                        'criterion wants >= 30, plan § R5)')
    p.add_argument('--throw-noise-frac', type=float, default=None,
                   help='per-component release-velocity scatter as a fraction of '
                        'the release speed (default 0.0: an exact release; the '
                        'module docstring carries the separation-vs-scatter table)')
    p.add_argument('--catch-aim-source', choices=ex.AIM_SOURCES,
                   default=ex.AIM_TRACKER,
                   help='where a CATCH is aimed from: tracker (the LIVE '
                        'default since 2026-09-18 and this gate\'s -- the '
                        'converged fit, with the schedule as its prior, plus '
                        'the refine path this gate asserts), schedule (open '
                        'loop from the commanded throw state), or '
                        'schedule_hand')
    p.add_argument('--learn', action='store_true',
                   help='run the R3/R4/R5 sim-validation learner run (plan '
                        '§ 4 R3; R4 Unit U7a; R5 Unit G) instead of the R2 '
                        'identity-prior columns gate')
    p.add_argument('--pattern', choices=('self_toss', 'hop', 'columns'),
                   default='self_toss',
                   help='--learn only: self_toss (R3, 1 site, P1), hop '
                        '(R4, 2 sites, P1<->P2, Unit U7a) or columns (R5, '
                        'both sites + the Stop, Unit G, --policy ignored)')
    p.add_argument('--policy', choices=('A', 'B'), default='A',
                   help='--learn only: A = single-throw attempts until '
                        'k_min rows then chained (the gated policy); B = '
                        'chained from the first throw (reported alongside)')
    p.add_argument('--reload', action='store_true',
                   help='run the R4 reload gate (Unit U3, brief step 5): a '
                        'synthetic Ball Butler arrival (25 deg / 4.8 m/s) '
                        'through schedule.compile_reload, instead of the '
                        'columns gate')
    p.add_argument('--feed-angle-deg', type=float, default=None,
                   help='--learn --pattern columns only: spawn ball 1 on an '
                        'oblique arrival at the feed site (R5 sitting 2, '
                        '2026-10-02: the site NEAREST Ball Butler, not '
                        "necessarily site 1 -- SkillsGate.run_columns_"
                        'attempt swaps which site holds A), this many '
                        'degrees off vertical, along the Ball Butler '
                        'bearing (SelfTossGateConfig.feed_angle_deg), '
                        "through compile_columns(feed=...) -- must be "
                        'given together with --feed-speed-mmps (default: '
                        "None, today's vertical spawn, unchanged)")
    p.add_argument('--feed-speed-mmps', type=float, default=None,
                   help='--learn --pattern columns only: the arrival speed '
                        '(mm/s) for --feed-angle-deg -- must be given '
                        'together with it')
    p.add_argument('--columns-feed-site', choices=('P1', 'P2'), default=None,
                   help='--learn --pattern columns, --feed-angle-deg only: '
                        'which named site is fed (SelfTossGateConfig.'
                        'columns_feed_site, default \'P1\' -- the swapped '
                        'layout: A at site 1, feed at site 0). \'P2\' is '
                        'the OLD, un-swapped layout (A at site 0, feed at '
                        'site 1), R5 sitting 2, 2026-10-02')
    p.add_argument('--feed-aim-toward-a-mm', type=float, default=None,
                   help='--learn --pattern columns, --feed-angle-deg only: '
                        'how far (mm) the synthetic arrival is walked from '
                        "the feed site TOWARD ball A's site "
                        '(SelfTossGateConfig.feed_aim_toward_a_mm, default '
                        '20.0 -- the swapped layout\'s undisplaced feed '
                        'catch refuses LIMIT_ACC at 101 %%, R5 sitting 2, '
                        '2026-10-02)')
    p.add_argument('--bb-bias-mm', nargs=2, type=float, default=None,
                   metavar=('BX', 'BY'),
                   help='--learn --pattern columns, --feed-angle-deg only '
                        '(B4, R5 sitting 4): SelfTossGateConfig.'
                        'feed_bb_bias_mm -- opens a gap between where the '
                        "schedule's own LandingPrior believes ball B will "
                        'land (unchanged, the request/announced point) and '
                        'where it is PHYSICALLY spawned to land, mirroring '
                        'the real Ball Butler bias skill_node.py\'s '
                        'columns_feed_bb_bias_mm cancels (default [0.0, '
                        '0.0]: no behaviour change; measured 2026-10-04: '
                        '[29.0, 25.0])')
    p.add_argument('--correlator', choices=('fixed', 'head'), default=None,
                   help='--learn only: which tracker-correlation rule '
                        '_IdentityTracker applies. \'fixed\' (the default): '
                        'identity latches keyed on the announcement\'s own '
                        '(thrower, throw_time) -- the 2026-10-04 two-ball-'
                        'association fix. \'head\': the pre-fix exclusion '
                        'rule -- the negative control that must reproduce '
                        'WINDOW_TOO_SHORT on fed columns.')
    p.add_argument('--dwell-s', type=float, default=None,
                   help='--learn only: override SelfTossGateConfig.dwell_s '
                        '(default 0.25). Use this to pin the dwell this run '
                        'plans at to the admissible box\'s OWN swept dwell '
                        '(config/generated/admissible_box.yaml\'s dwell_s) '
                        'when the two differ and a box-dwell mismatch '
                        'matters to the caller.')
    p.add_argument('--one-ball', action='store_true',
                   help='--learn --pattern columns only (B2b): run the '
                        'columns_1ball trial -- ball A real, ball B a '
                        'PHANTOM (schedule.Pattern.phantom_balls=(1,), '
                        'never spawned) -- instead of the two-real-ball '
                        'columns gate')
    p.add_argument('--one-ball-reload', action='store_true',
                   help='--one-ball only: ball A is fed externally through '
                        'schedule.compile_reload_columns (the same '
                        'synthetic Ball Butler arrival --reload uses for '
                        'self_toss/hop) instead of starting already held '
                        'at its site')
    p.add_argument('--one-ball-fed', action='store_true',
                   help='--learn --pattern columns only (B2, R5 sitting 4): '
                        'run the columns_1ball_fed trial -- the OTHER half '
                        'of --one-ball: ball A a PHANTOM (schedule.Pattern.'
                        'phantom_balls=(0,), never spawned), ball B real '
                        'and fed by Ball Butler. Requires --feed-angle-deg/'
                        '--feed-speed-mmps (ball B is always fed here); '
                        'mutually exclusive with --one-ball')
    args = p.parse_args(argv)

    if args.reload:
        rcfg = SelfTossGateConfig(report_path=args.report,
                                  catch_aim_source=args.catch_aim_source)
        seeds = tuple(args.seeds) if args.seeds is not None else (0, 1, 2, 3, 4)
        rep = run_reload_gate(rcfg, seeds=seeds)
        _print_reload_table(rep)
        return 0 if rep['passed'] else 1

    if args.learn:
        if (args.feed_angle_deg is None) != (args.feed_speed_mmps is None):
            p.error('--feed-angle-deg and --feed-speed-mmps must be given '
                     'together, or both left off')
        if (args.feed_angle_deg is not None and args.pattern != 'columns'):
            p.error('--feed-angle-deg/--feed-speed-mmps only apply to '
                     '--pattern columns')
        if args.one_ball and args.pattern != 'columns':
            p.error('--one-ball only applies to --pattern columns')
        if args.one_ball_reload and not args.one_ball:
            p.error('--one-ball-reload requires --one-ball')
        if args.one_ball and args.feed_angle_deg is not None:
            p.error('--one-ball and --feed-angle-deg/--feed-speed-mmps are '
                     'mutually exclusive -- B2b has no FEED variant')
        if args.one_ball_fed and args.pattern != 'columns':
            p.error('--one-ball-fed only applies to --pattern columns')
        if args.one_ball_fed and args.one_ball:
            p.error('--one-ball-fed and --one-ball are mutually exclusive '
                     '-- each is phantom on the OTHER ball')
        if args.one_ball_fed and args.feed_angle_deg is None:
            p.error('--one-ball-fed requires --feed-angle-deg/'
                     '--feed-speed-mmps -- ball B is always fed in this '
                     'trial')
        if args.bb_bias_mm is not None and args.feed_angle_deg is None:
            p.error('--bb-bias-mm only applies with --feed-angle-deg/'
                     '--feed-speed-mmps set')
        default_sep = (_HOP_SEPARATION_MM if args.pattern == 'hop'
                      else _SELF_TOSS_SEPARATION_MM)
        separation_mm = (float(args.separation_mm)
                         if args.separation_mm is not None else default_sep)
        lcfg = SelfTossGateConfig(report_path=args.report,
                                  catch_aim_source=args.catch_aim_source,
                                  pattern=args.pattern,
                                  separation_mm=separation_mm)
        if args.apex_m is not None:
            lcfg.apex_m = float(args.apex_m)
        if args.target_throws is not None:
            lcfg.target_throws = int(args.target_throws)
        if args.feed_angle_deg is not None:
            lcfg.feed_angle_deg = float(args.feed_angle_deg)
        if args.feed_speed_mmps is not None:
            lcfg.feed_speed_mmps = float(args.feed_speed_mmps)
        if args.columns_feed_site is not None:
            lcfg.columns_feed_site = str(args.columns_feed_site)
        if args.feed_aim_toward_a_mm is not None:
            lcfg.feed_aim_toward_a_mm = float(args.feed_aim_toward_a_mm)
        if args.bb_bias_mm is not None:
            lcfg.feed_bb_bias_mm = (float(args.bb_bias_mm[0]),
                                    float(args.bb_bias_mm[1]))
        if args.correlator is not None:
            lcfg.correlator = str(args.correlator)
        if args.dwell_s is not None:
            lcfg.dwell_s = float(args.dwell_s)
        if args.one_ball:
            lcfg.one_ball = True
        if args.one_ball_reload:
            lcfg.one_ball_reload = True
        if args.one_ball_fed:
            lcfg.one_ball_fed = True
        if args.seeds is not None:
            seeds = tuple(args.seeds)
        else:
            seeds = (0, 1, 2, 3, 4)
        rep = run_learn(lcfg, seeds=seeds, policy=args.policy)
        _print_learn_table(rep)
        return 0 if rep['passed'] else 1

    cfg = SkillsGateConfig(report_path=args.report,
                           catch_aim_source=args.catch_aim_source)
    if args.seeds is not None:
        cfg.seeds = tuple(args.seeds)
    if args.n_throws is not None:
        cfg.n_throws = args.n_throws
    if args.separation_mm is not None:
        cfg.separation_mm = float(args.separation_mm)
    if args.apex_m is not None:
        cfg.apex_m = float(args.apex_m)
    if args.throw_noise_frac is not None:
        cfg.noise = NoiseConfig(bb_throw_noise_frac=float(args.throw_noise_frac),
                                tracking_noise_mm=cfg.noise.tracking_noise_mm)

    rep = run_gate(cfg, viewer_speed=(args.viewer_speed if args.viewer
                                      else None))
    _print_table(rep)
    return 0 if rep['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
