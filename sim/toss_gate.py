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

**Gate criterion (binding bands, reported separately):**
  * the plan's band — every swept point with arrival speed 2-3 m/s
    (flights {0.55, 0.60} s ⇒ 12 points in the factored grid) needs
    ``core_clean >= 9/10``;
  * the orchestrator amendment (2026-07-25) — the T = 0.80 s points at
    z = 170 (all five xy positions) are ADDITIONALLY binding >= 9/10: the
    2-3 m/s band is hardware-marginal (< 0.7 s firmware kind-1 windup race,
    Phase-1 logbook Known limitations), so the hardware-relevant band must
    gate too;
  * the Tier-8b displaced band (single-ball-toss Phase 4) — in a ``--tier 8b``
    run the displaced grid's centre + 50 mm-ring points at T = 0.80 s /
    z = 170 (the bb Rung-2a reliable box: separated 10/10, landing error
    4.9-32.4 mm, re-hosted on the production stack) need
    ``core_clean >= ceil(0.9 n)``, reported as ``passed_8b_ring``. In a
    tier-8b run DISPLACED points leave the T = 0.80 band (superseded by this
    one), so the ADVISORY rings ({70, 100, cap} since Phase E) and the
    T = 0.95 spot checks stay ADVISORY; tier-8a runs are byte-identical (the
    band is empty there and the old membership logic is untouched).

    **Why the cap ring is advisory and not binding (Phase E, 2026-07-29).** The
    shipped displacement cap moved 70 → 150 mm, and the natural instinct is to
    make the 150 mm ring gate. It does not, for the same reason the contact
    column has never gated: the ball-lands-in-cup verdict at 150 mm is dominated
    by release scatter through a MuJoCo contact/capture model this gate's own
    docstring calls the low-fidelity element, and the release-noise magnitude is
    still the Phase-5 T0 PLACEHOLDER. Gating the shipped cap on a placeholder
    would make the cap an artefact of a number nobody has measured. What the
    advisory rings DO gate is the part sim is authoritative about: they run the
    full production pipeline, so their trials feed the invariant counters (zero
    feasibility violations, every emitted knot pump-accepted) that ``passed``
    conjoins. The catch RATE at the cap is the hardware ladder's job
    (``tests/hardware/session_anomaly_fixes.md`` § SECTION DISP), and the
    PLANNABILITY of the reach is `tools/probes/displaced_reach_frontier.py`'s.
``passed`` requires every binding band, zero feasibility violations in accepted runs,
and zero pump rejects with every emitted frame ACCEPTED (the accepted==emitted
counter is the non-vacuous form of the invariant). Explicit-points/debug runs
whose grid intersects NEITHER band gate on the full band instead of vacuously
passing two empty ``all()``'s. "Cycle success" is ``core_clean`` — which
requires the arm-and-forget catch to have actually ARMED from tracking
(``catch_armed``; a ball falling into a statically-parked cup must not pass)
— the hand-contact velocity-match stays the reload gate's documented
light-scope deferral.

**Tier 8b (``--tier 8b``, single-ball-toss Phase 4)** — the tilt-aimed
displaced throw→catch. ``toss_release.compute_release_state_tilted`` aims the
cup axis ALONG the ballistic A→B take-off (all lateral velocity comes from the
slider stroke projected through the throw tilt — never platform translation);
the pre-position ``build_move`` targets the swing-compensated PRE-TILT pose at
the throw site A; pump B streams a validated ``build_hold`` at A through the
prep gap and swaps to the ``build_catch`` reach A→B (lead = the flight time,
arrival = the announced landing) INSIDE THE SAME PUMP at the scheduled release
— the sim analogue of the production deferred-reach choreography (the stock
announcement pre-tilt would complete the A→B translate BEFORE release for
every toss flight, firing the throw from B with the receive tilt), with the
pump's own per-step gate checking the hold→reach crossing (no waived first
frame). The catch at B runs the unchanged 8a path. The non-gating detach diag
column additionally runs the ±{70, 100, 150} mm directional-asymmetry MAP
(``contact_diagnostic.asymmetry_map``): the Rung-2a glue/overshoot physics
lives in contact detach, which the gating column's imposed-velocity release
cannot reproduce — a kinematic-column map would be symmetric and meaningless.

**The throw site A is SWEPT here and read LIVE in production.** Since
single-ball-toss Phase E the coordinator sources A from the platform's live
commanded pose (``trajectory/commanded_position``), never from config; this gate
keeps ``--throw-site`` because a sweep has to *choose* A. The gate is therefore
faithful to the geometry and deliberately silent about the plumbing — the live
read has its own unit coverage (``tests/ros/test_toss_coordinator.py::
test_8b_throw_site_is_the_live_commanded_pose``).

**The 2026-07-25 asymmetry map is STALE VINTAGE — do not inherit it.** It was
measured on the pre-2026-07-26 machine (moving-rim catch at seat rate 0.07, the
pre-C-CATCH-1 arrival), so anything read off it — notably the +y-hemisphere
weakness at 70-100 mm — describes a plant that no longer exists. Re-run the map
rather than citing it.

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

# Single path bootstrap (repo root, ros_ws pkg, config/generated);
# see sim/_paths.py.  Runnable entry scripts only — library modules under
# sim/ never touch sys.path.
_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
from sim._paths import bootstrap_paths  # noqa: E402
bootstrap_paths()

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

from sim.plant.mujoco_plant import MuJoCoPlant
from sim.hand.trajectory import (
    HandCatchSequence, HandCatchTrajectory, HandThrowSequence, STROKE_MARGIN_MM,
)
from sim.juggle_noise import JuggleNoise, NoiseConfig, BallisticEstimator

from sim.gate_common import (
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


# The 8-direction ring (unit vectors): the Rung-2a reliable-box characterisation
# directions (axes + diagonals), used by both the Tier-8b binding grid and the
# ±{70, 100} mm asymmetry map.
_R2 = math.sqrt(0.5)
_RING_8DIR = ((1.0, 0.0), (_R2, _R2), (0.0, 1.0), (-_R2, _R2),
              (-1.0, 0.0), (-_R2, -_R2), (0.0, -1.0), (_R2, -_R2))
# Tier-8b binding displaced ring (Rung-2a reliable box: the column + the 50 mm
# ring, separated 10/10, landing error 4.9-32.4 mm — logbook 2026-06-30).
_TOSS_8B_RING_MM = 50.0
# Advisory displaced rings, extended 2026-07-29 (single-ball-toss Phase E) from
# {70} to {70, 100, cap}: 70 = the bb Rung-2a clean-box edge and the OLD shipped
# displacement cap (hardware-validated 11/11 at the 2026-07-27 T4 rungs), 100 and
# 150 = the operator-ordered working range. The cap ring is read from the config
# key so the gate and the shipped cap can never describe different machines.
_TOSS_8B_CAP_RING_MM = float(hw.JB_OP_TOSS_MAX_DISPLACEMENT_MM)
_TOSS_8B_ADVISORY_RINGS_MM = (70.0, 100.0, _TOSS_8B_CAP_RING_MM)
_TOSS_8B_FLIGHT_S = 0.80                    # the hardware band (§ 2.6 rules out <=0.61)
# Directional-asymmetry MAP radii + flights (§ 3.2; non-gating). The 150 mm radius
# is Phase E's; 0.60 s is kept as the Rung-2a characterisation flight even though
# the production FSM refuses 150 mm there (the closed-form reach bound is 108 mm
# at T = 0.60), because the map is a PHYSICS characterisation and the refusal is
# a separate, unit-tested gate — pinned by
# tests/ros/test_toss_sequencer.py::test_displacement_rejected, whose
# ((150, 0, 170), (0, 0), 0.60) leg asserts exactly that REJECTED_DISPLACEMENT.
# So the 150 mm / 0.60 s cells describe a state the FSM will not command; they
# are kept deliberately, as characterisation, not as evidence for a flight band.
_ASYMMETRY_RADII_MM = (70.0, 100.0, 150.0)
_ASYMMETRY_FLIGHTS_S = (0.60, 0.80)         # 0.60 = Rung-2a characterisation; 0.80 = hw band


def default_grid_8b(throw_site_xy=(0.0, 0.0), advisory_rings_mm=None):
    """The Tier-8b displaced grid as ``[(x, y, z, T), ...]`` — B positions
    relative to the throw site A (``throw_site_xy``): the centre (A) + the
    BINDING 50 mm ring (8 directions) + the ADVISORY rings
    (``advisory_rings_mm``, default {70, 100, cap}) + two advisory
    T = 0.95 column-displaced spot checks, all at z = 170 / T = 0.80 (the
    hardware band; short flights are infeasible at these displacements, § 2.6).

    Every ring runs the FULL gating pipeline (production ``build_catch`` +
    ``KnotEmitter`` + a real ``SetpointPump`` + the feasibility gate), so the
    advisory rings still contribute to the invariant counters — "does the
    production stack accept a 150 mm displaced toss end to end" is answered even
    though its catch RATE does not gate."""
    ax, ay = float(throw_site_xy[0]), float(throw_site_xy[1])
    rings = (_TOSS_8B_ADVISORY_RINGS_MM if advisory_rings_mm is None
             else tuple(float(r) for r in advisory_rings_mm))
    z = Z_ACTIVE_MM
    pts = [(ax, ay, z, _TOSS_8B_FLIGHT_S)]                      # centre = A (displacement 0)
    for ux, uy in _RING_8DIR:                                    # binding 50 mm ring
        pts.append((ax + _TOSS_8B_RING_MM * ux, ay + _TOSS_8B_RING_MM * uy,
                    z, _TOSS_8B_FLIGHT_S))
    for radius in rings:                                         # advisory rings
        for ux, uy in _RING_8DIR:
            pts.append((ax + radius * ux, ay + radius * uy, z, _TOSS_8B_FLIGHT_S))
    pts.append((ax + _TOSS_8B_RING_MM, ay, z, 0.95))            # advisory T=0.95 spots
    pts.append((ax, ay + _TOSS_8B_RING_MM, z, 0.95))
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
    tier: str = '8a'                  # '8a' co-located | '8b' tilt-aimed displaced
    throw_site_xy: tuple = (0.0, 0.0)  # Tier-8b throw site A (STOW xy); B = the grid point
    advisory_rings_mm: tuple | None = None  # Tier-8b advisory ring radii (None ⇒
                                      # _TOSS_8B_ADVISORY_RINGS_MM = {70, 100, cap})
    asymmetry_radii_mm: tuple | None = None  # asymmetry-map radii (None ⇒
                                      # _ASYMMETRY_RADII_MM = {70, 100, 150})
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
    # Tier-8b: the pre-position target is the swing-compensated PRE-TILT pose at
    # the throw site A (6-dof); the trial holds there through the prep gap and
    # swaps to the A->B reach at t_release. For 8a it is the level nominated pose
    # and hold_reach_swap is False (single pre-tilt catch installed at t_install).
    preposition_pose: np.ndarray | None = None
    hold_reach_swap: bool = False
    displacement_mm: float = 0.0
    pretilt_rx: float = 0.0
    pretilt_ry: float = 0.0


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
    # Tier-8b metrics (0.0 / NaN for 8a).
    tier: str = '8a'
    displacement_mm: float = 0.0   # |B_xy − A_xy| (the O1-cap headroom witness)
    pretilt_err_deg: float = float('nan')  # commanded vs achieved pre-tilt at A
    reach_lead_s: float = float('nan')     # A→B reach installed → arrival

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
        calls. All ``toss_release`` calls run on DEFAULTS (hardware-frame; R1).
        Tier 8b dispatches to :meth:`_prepare_toss_8b` (tilted release at A)."""
        cfg = self.cfg
        x, y, z, T = point
        if cfg.tier == '8b':
            return self._prepare_toss_8b(point)
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
            ann_landing_vel_mms=ann_landing_vel,
            preposition_pose=np.array([x, y, z, 0.0, 0.0, 0.0]),
            hold_reach_swap=False)

    def _prepare_toss_8b(self, point) -> _TossSetup:
        """Tier-8b synthesis: the TILTED release at the throw site A
        (``compute_release_state_tilted``, aim ALONG the A->B take-off — all
        lateral velocity comes from the slider stroke through the tilt), the
        swing-compensated PRE-TILT POSITIONING pose at A, and the catch pose at
        the displaced B from the announced (lateral) arrival. The loud
        ``ThrowTiltInfeasible`` clamp maps to a TILT_CLAMP reject (never a
        silently mis-aimed throw)."""
        cfg = self.cfg
        x, y, z, T = point
        try:
            rs = toss_release.compute_release_state_tilted(
                (x, y, z), T, throw_site_xy_mm=cfg.throw_site_xy)
        except toss_release.ThrowTiltInfeasible:
            return _TossSetup(reject_code='TILT_CLAMP')
        except ValueError:
            return _TossSetup(reject_code='RELEASE_STATE')
        if not toss_release.validate_event_vel(rs.event_vel_mps):
            return _TossSetup(event_vel_mps=rs.event_vel_mps,
                              reject_code='EVENT_VEL_BAND')
        ann_landing_vel = bal.arrival_velocity(rs.launch_vel_mms, T)
        armed_v = (float(np.linalg.norm(ann_landing_vel)) / 1000.0) \
            * (1.0 + cfg.event_vel_err_frac)
        cup_world_z_sim = (CUP_Z_BASE_MM
                           + HandCatchTrajectory(armed_v).sample(0.0)
                           + (z - Z_ACTIVE_MM))
        # Catch pose at the displaced B (the announced landing IS B's cup point);
        # the lateral arrival now yields a real receive tilt + swing shift.
        catch_pose, (rx, ry) = _toss_catch_pose(
            rs.catch_point_global_mm[:2], ann_landing_vel, cup_world_z_sim, z)
        return _TossSetup(
            event_vel_mps=rs.event_vel_mps, rs=rs, armed_v=armed_v,
            catch_pose=catch_pose, rx=rx, ry=ry,
            cup_world_z_sim=cup_world_z_sim,
            ann_landing_vel_mms=ann_landing_vel,
            preposition_pose=np.asarray(rs.pretilt_pose_stow, dtype=float),
            hold_reach_swap=True,
            displacement_mm=float(rs.displacement_mm),
            pretilt_rx=float(rs.tilt_rx), pretilt_ry=float(rs.tilt_ry))

    # ---- one gating trial -------------------------------------------------
    def run_trial(self, idx: int, point, seed: int) -> TossTrialResult:
        cfg = self.cfg
        x, y, z, T = point
        s = self._prepare_toss(point)
        if s.reject_code is not None:
            return self._reject(idx, point, s.event_vel_mps, seed, s.reject_code)
        pose_pre = np.asarray(s.preposition_pose, dtype=float)
        # PLAN via the production constructors (the single gate). Rejects are loud.
        # Pre-position to the pre-position pose (level nominated for 8a, the
        # swing-compensated pre-tilt pose at A for 8b).
        try:
            plan_move, report_move = planner.build_move(
                (NEUTRAL_POSE, np.zeros(6), np.zeros(6)), pose_pre,
                cfg.preposition_duration_s, self.limits, self.geom)
        except TrajectoryInfeasible as e:
            return self._reject(idx, point, s.event_vel_mps, seed, e.code)
        try:
            if s.hold_reach_swap:
                # Tier 8b: hold at A through the prep gap, then reach A->B over
                # the flight (lead = T; arrival = the announced landing). The
                # deferred-reach choreography — the stock announce pre-tilt would
                # complete the A->B translate BEFORE release for every flight.
                plan_hold = planner.build_hold(
                    (pose_pre, np.zeros(6), np.zeros(6)), self.limits, self.geom)
                plan_catch, report_catch = planner.build_catch(
                    (pose_pre, np.zeros(6), np.zeros(6)), s.catch_pose,
                    T, self.limits, self.geom, settle_hold_s=cfg.settle_hold_s)
            else:
                # Tier 8a: state0 = the nominated (commanded) pose — production
                # plans from commanded state. Lead = prep_gap + T so the plan's
                # arrival instant equals the ANNOUNCED landing time.
                plan_hold = None
                plan_catch, report_catch = planner.build_catch(
                    (pose_pre, np.zeros(6), np.zeros(6)), s.catch_pose,
                    cfg.prep_gap_s + T, self.limits, self.geom,
                    settle_hold_s=cfg.settle_hold_s)
        except TrajectoryInfeasible as e:
            return self._reject(idx, point, s.event_vel_mps, seed, e.code)
        noise = JuggleNoise(self.noise_cfg, seed=seed)
        return self._simulate_toss(idx, point, s, plan_move, report_move,
                                   plan_catch, report_catch, plan_hold,
                                   noise, seed)

    def _simulate_toss(self, idx, point, s, plan_move, report_move,
                       plan_catch, report_catch, plan_hold, noise, seed
                       ) -> TossTrialResult:
        """One trial start-to-verdict on the gating (kinematic-hold) plant."""
        cfg = self.cfg
        x, y, z, T = point
        plant, emitter = self.plant, self.emitter
        model_dt = plant.timestep
        substeps = max(1, int(round(KNOT_DT_S / model_dt)))
        preposition_xyz = np.asarray(s.preposition_pose[:3], dtype=float)

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
        # Pre-position error vs the commanded pre-position xyz (A's swing-
        # compensated pre-tilt pose for 8b, the level nominated pose for 8a).
        preposition_err_mm = float(np.linalg.norm(
            st.platform_pos_mm - preposition_xyz))
        # Commanded vs achieved pre-tilt at A (8b diagnostic; ~0 for level 8a).
        pretilt_err_deg = float(np.degrees(np.linalg.norm(
            st.platform_rot - np.array([s.pretilt_rx, s.pretilt_ry, 0.0]))))

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
        reach_installed_t = None       # 8b: sim time the A->B reach became active

        while plant.data.time < run_until:
            # Tier 8b: stream the hold at A through the prep gap, then swap to
            # the A->B reach at t_release INSIDE THE SAME PUMP — so the pump's
            # own per-step gate checks the hold->reach crossing (production runs
            # one continuous pump; no waived first frame at the swap). Tier 8a
            # streams the single pre-tilt catch from t_install throughout.
            if plan_hold is not None and plant.data.time < t_release:
                active_plan = plan_hold
                tau = plant.data.time - t_install
            else:
                active_plan = plan_catch
                tau = plant.data.time - (t_release if plan_hold is not None
                                         else t_install)
                if plan_hold is not None and reach_installed_t is None:
                    reach_installed_t = plant.data.time
            frame = emitter.frame(active_plan, tau, n_frames)
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
            clean=clean, core_clean=core_clean,
            tier=cfg.tier, displacement_mm=float(s.displacement_mm),
            pretilt_err_deg=float(pretilt_err_deg),
            reach_lead_s=(float(t_land_ann - reach_installed_t)
                          if reach_installed_t is not None else float('nan')))

    def _reject(self, idx, point, event_vel_mps, seed, code) -> TossTrialResult:
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
            clean=False, core_clean=False, tier=self.cfg.tier)

    # ---- the full gate ----------------------------------------------------
    def run(self) -> dict:
        cfg = self.cfg
        if cfg.points is not None:
            points = list(cfg.points)
        elif cfg.tier == '8b':
            points = default_grid_8b(cfg.throw_site_xy,
                                     cfg.advisory_rings_mm)
        else:
            points = default_grid(cfg.grid)
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
        out = {
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
        # Tier-8b only: the ±{70, 100} mm directional-asymmetry MAP — the
        # Rung-2a glue/overshoot physics lives in contact DETACH (the gating
        # column's imposed-velocity release cannot reproduce it). ADVISORY /
        # NON-GATING by construction (§ 3.2).
        if cfg.tier == '8b':
            out['asymmetry_map'] = self._run_asymmetry_map()
        return out

    def _run_asymmetry_map(self, dirs=None, radii=None, flights=None) -> dict:
        """The ±{70, 100, 150} mm directional-asymmetry MAP (Tier-8b, § 3.2) — 8
        directions × radii × flights {0.60, 0.80} on the
        contact_carry DETACH plant, ``diag_trials`` per cell. Per-cell metric:
        ``seated_n`` (post-catch seat: caught AND not a >SEPARATION_MS bounce-OUT)
        + landing error (mm) vs the commanded B. NOTE ``seated_n`` is honest to
        what it counts — it is NOT the Rung-2a THROW-detach 'glue' (ball never
        left the cup during the stroke); the imposed-velocity/detach release here
        exposes no per-trial throw-glue signal, so the map characterizes post-catch
        seat + landing-error-vs-B, not the throw asymmetry the Rung-2a table names.
        **Non-gating** — a characterisation for picking T4's displacement
        direction, NEVER a pass/fail (the contact model is the documented
        low-fidelity element; its hardware truth is T0/T4's measurand). Rung-2a
        reference: (0, ±100) glued 0/2; (−100, 0) err 134 mm; (+100, 0) 21.5 mm
        good; (−71, +71) 18.6 mm good. The dirs/radii/flights overrides exist for
        the CI smoke (a 1-cell slice)."""
        cfg = self.cfg
        if self._diag_plant is None:
            self._diag_plant = MuJoCoPlant(geom=self.geom, contact_carry=True)
        dirs = _RING_8DIR if dirs is None else dirs
        if radii is None:
            radii = (cfg.asymmetry_radii_mm if cfg.asymmetry_radii_mm is not None
                     else _ASYMMETRY_RADII_MM)
        flights = _ASYMMETRY_FLIGHTS_S if flights is None else flights
        ax, ay = float(cfg.throw_site_xy[0]), float(cfg.throw_site_xy[1])
        cells = []
        i = 0
        for ux, uy in dirs:
            for radius in radii:
                for T in flights:
                    bx, by = ax + radius * ux, ay + radius * uy
                    # BUGFIX 2026-07-29: `i` was constant across this
                    # comprehension (it advanced only AFTER it), so every trial
                    # in a cell ran the SAME seed and the map was really ONE
                    # trial replicated `diag_trials` times. The tell was
                    # `landing_err_mm_mean == landing_err_mm_worst` EXACTLY in
                    # every cell that had any seated trial. The
                    # `i += diag_trials` below shows the intent was always a
                    # distinct seed per trial. Enumerating the offset here
                    # restores it; nothing else changes, and the cell dict is
                    # unchanged.
                    #
                    # It does NOT explain the map's near-bimodality, and the
                    # first draft of this comment claimed it did. Measured on
                    # the POST-FIX re-run (temp/reports/
                    # toss_8b_phaseE_asymmetry_seed0.json, seed 0): 47 of 48
                    # cells still read 0/4 or 4/4, one reads 1/4. That is the
                    # documented low-fidelity contact model, not a seeding
                    # artefact — so a 0/4 cell is still NOT evidence of a 0 %
                    # direction, and the map stays ~1-bit-per-cell at n = 4.
                    # The directional evidence with real spread is the GATING
                    # column (8/10-10/10); see the runbook's § SECTION DISP.
                    trials = [self._diag_trial((bx, by, Z_ACTIVE_MM, T),
                                               cfg.seed + 20000 + i + k)
                              for k in range(cfg.diag_trials)]
                    i += cfg.diag_trials
                    caught = [r for r in trials if r['caught']]
                    seated = sum(1 for r in trials
                                 if r['caught'] and not r['separated'])
                    errs = [r['landing_err_mm'] for r in caught
                            if not math.isnan(r['landing_err_mm'])]
                    cells.append({
                        'direction': [round(ux, 3), round(uy, 3)],
                        'radius_mm': float(radius),
                        'flight_time_s': float(T),
                        'target_xy_mm': [round(bx, 2), round(by, 2)],
                        'n': len(trials),
                        'caught': len(caught),
                        # Post-catch SEAT count: caught AND not a >SEPARATION_MS
                        # contact-loss bounce-OUT (r['separated']). This is NOT the
                        # Rung-2a THROW-detach 'glue' (ball never left the cup during
                        # the stroke): the imposed-velocity/detach release here has no
                        # per-trial throw-glue signal to derive it from, so the cell
                        # honestly reports post-catch seat + landing-error-vs-B only.
                        'seated_n': int(seated),             # caught AND seated (post-catch)
                        'landing_err_mm_mean': (
                            float(np.mean(errs)) if errs else None),
                        'landing_err_mm_worst': (
                            float(max(errs)) if errs else None),
                    })
        return {
            'gating': False,
            'note': ('ADVISORY contact-detach characterisation (Rung-2a '
                     'glue/overshoot); NEVER feeds passed. Metric per cell: '
                     'seated_n (POST-CATCH seat: caught AND not a >SEPARATION_MS '
                     'bounce-OUT) + landing error vs the commanded B. seated_n is '
                     'NOT throw-detach glue (no per-trial throw-glue signal is '
                     'derivable from the detach release). Diag motion differs from '
                     'Rung-2a (hold-pretilt + production HandThrowSequence stroke '
                     'here, not plan_cup_cycle) — deltas vs the logbook table are '
                     'findings, not a replication study.'),
            'radii_mm': list(radii),
            'flights_s': list(flights),
            'cells': cells,
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
        pose_pre = np.asarray(s.preposition_pose, dtype=float)
        try:
            plan_move, _ = planner.build_move(
                (NEUTRAL_POSE, np.zeros(6), np.zeros(6)), pose_pre,
                cfg.preposition_duration_s, self.limits, self.geom)
            if s.hold_reach_swap:
                # Tier 8b: hold the pre-tilt at A through the prep gap, then
                # reach A->B over the flight (lead = T) — same deferred-reach
                # choreography as the gating column.
                plan_hold = planner.build_hold(
                    (pose_pre, np.zeros(6), np.zeros(6)), self.limits, self.geom)
                plan_catch, _ = planner.build_catch(
                    (pose_pre, np.zeros(6), np.zeros(6)), s.catch_pose,
                    T, self.limits, self.geom, settle_hold_s=cfg.settle_hold_s)
            else:
                plan_hold = None
                plan_catch, _ = planner.build_catch(
                    (pose_pre, np.zeros(6), np.zeros(6)), s.catch_pose,
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

        def _active(t):
            # Tier 8b: hold the pre-tilt at A until t_release, then the A->B
            # reach; Tier 8a: the single pre-tilt catch from t_install.
            if plan_hold is not None and t < t_release:
                return plan_hold, t_install
            return plan_catch, (t_release if plan_hold is not None else t_install)

        while plant.data.time < run_until:
            active_plan, plan_t0 = _active(plant.data.time)
            tau = plant.data.time - plan_t0
            frame = emitter.frame(active_plan, tau, n)
            pump2.build(frame, t_origin_us=int(n * 25000))
            n += 1
            plant.command(np.asarray(frame['ext_mm']))
            for _ in range(substeps):
                if plant.data.time >= run_until:
                    break
                t_now = plant.data.time
                sub_plan, sub_t0 = _active(t_now)
                pose_sub, _, _ = sub_plan.state_at(t_now - sub_t0)
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
            is_8b = (cfg.tier == '8b')
            displacement = float(np.hypot(x - float(cfg.throw_site_xy[0]),
                                          y - float(cfg.throw_site_xy[1])))
            point_rows.append({
                'point_id': pid,
                'pose_mm': [x, y, z],
                'flight_time_s': T,
                'event_vel_mps': float(ev),
                'displacement_mm': displacement,
                # The plan's binding band: arrival speed 2-3 m/s (Tier-8a only —
                # the 8b binding band is the displaced 50 mm ring below).
                'binding_2_3_mps': bool(
                    not is_8b and math.isfinite(ev)
                    and _BINDING_SPEED_MIN_MPS <= ev <= _BINDING_SPEED_MPS),
                # The orchestrator-amendment binding band: T=0.80 s at z=170
                # (Tier-8a only). Membership is purely geometric (grid
                # coordinates) — an isfinite(ev) term here would let a
                # fully-rejected point silently leave the band and pass it
                # vacuously; a rejected binding point must instead FAIL
                # pass_9_of_10 (core_clean 0).
                'binding_T080_z170': bool(
                    not is_8b and abs(T - _BINDING_T080_S) < 1e-9
                    and abs(z - Z_ACTIVE_MM) < 1e-9),
                # Tier-8b binding displaced band: the centre (A) + the 50 mm ring
                # at T=0.80 / z=170 (the Rung-2a reliable box). Purely geometric,
                # same no-vacuous-pass rationale. The 70 mm ring (displacement
                # ~70) and the T=0.95 spots stay ADVISORY (excluded here).
                'binding_8b_ring': bool(
                    is_8b and abs(T - _TOSS_8B_FLIGHT_S) < 1e-9
                    and abs(z - Z_ACTIVE_MM) < 1e-9
                    and displacement <= _TOSS_8B_RING_MM + 1e-6),
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
        band_8b = [p for p in point_rows if p['binding_8b_ring']]
        passed_23 = all(p['pass_9_of_10'] for p in band_23)
        passed_t080 = all(p['pass_9_of_10'] for p in band_t080)
        passed_8b_ring = all(p['pass_9_of_10'] for p in band_8b)
        passed_full_band = bool(all(p['pass_9_of_10'] for p in point_rows))
        pump_frames_emitted = sum(r.pump_frames_emitted for r in results)
        pump_frames_accepted = sum(r.pump_frames_accepted for r in results)
        invariants_ok = (feas_violations == 0 and pump_rejects == 0
                         and pump_frames_accepted == pump_frames_emitted)
        no_binding_points = (not band_23) and (not band_t080) and (not band_8b)
        if no_binding_points:
            # Explicit-points/debug run intersecting NO binding band: empty
            # all()'s would vacuously PASS, so gate on the full band instead.
            # main() prints the loud NO BINDING POINTS banner. A zero-trial run
            # (empty cfg.points) can never PASS.
            passed = bool(results and passed_full_band and invariants_ok)
        elif cfg.tier == '8b':
            # Tier 8b: the binding band is the displaced 50 mm ring (+ centre);
            # the 2-3 m/s / T080 8a bands do not apply.
            passed = bool(passed_8b_ring and invariants_ok)
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
            'tier': cfg.tier,
            'passed': passed,
            # The binding bands, reported separately (per-band criterion only;
            # `passed` conjoins the tier's binding band(s) with the invariants).
            'passed_2_3_mps_band': bool(passed_23),
            'passed_T080_z170_band': bool(passed_t080),
            'passed_8b_ring_band': bool(passed_8b_ring),   # Tier-8b binding band
            'passed_full_band': passed_full_band,      # advisory, all points
            'no_binding_points': bool(no_binding_points),
            'binding_note': (
                "Tier 8a: passed = (every 2-3 m/s-band point core_clean >= "
                "ceil(0.9 n)) AND (every T=0.80 s @ z=170 point core_clean >= "
                "ceil(0.9 n)) AND the invariants (zero feasibility violations, "
                "zero pump rejects, every emitted pump frame accepted). The "
                "T=0.80 band is the 2026-07-25 orchestrator amendment (the "
                "2-3 m/s band is hardware-marginal). Tier 8b: passed = (every "
                "centre + 50 mm-ring point at T=0.80/z=170 core_clean >= "
                "ceil(0.9 n)) AND the invariants; the 70 mm ring + T=0.95 spots "
                "stay ADVISORY. When the grid intersects NO binding band "
                "(no_binding_points), passed falls back to passed_full_band AND "
                "the invariants — never a vacuous empty-band PASS."),
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
    p.add_argument('--tier', choices=('8a', '8b'), default='8a',
                   help="'8b' runs the tilt-aimed displaced throw→catch: the "
                        "displaced binding grid (centre + 50 mm ring @ T=0.80) "
                        "plus the non-gating ±{70,100} mm asymmetry map.")
    p.add_argument('--throw-site', default=None,
                   help="Tier-8b throw site A as 'X,Y' STOW mm (default 0,0 = "
                        "the workspace centre); B is the swept grid point. NOTE "
                        "production reads A from the platform's live commanded "
                        "pose (single-ball-toss Phase E); here it is swept.")
    p.add_argument('--advisory-rings', default=None,
                   help="Tier-8b ADVISORY ring radii, comma-separated mm "
                        "(default 70,100,<toss_max_displacement_mm>).")
    p.add_argument('--asymmetry-radii', default=None,
                   help="directional-asymmetry MAP radii, comma-separated mm "
                        "(default 70,100,150).")
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
    throw_site = (0.0, 0.0)
    if args.throw_site is not None:
        throw_site = tuple(float(v) for v in args.throw_site.split(','))
    rings = (None if args.advisory_rings is None
             else tuple(float(v) for v in args.advisory_rings.split(',')))
    asym = (None if args.asymmetry_radii is None
            else tuple(float(v) for v in args.asymmetry_radii.split(',')))
    cfg = TossGateConfig(
        trials_per_point=args.trials_per_point, seed=args.seed,
        grid=args.grid, tier=args.tier, throw_site_xy=throw_site, points=points,
        advisory_rings_mm=rings, asymmetry_radii_mm=asym,
        release_vel_noise_frac=args.release_noise_frac,
        arm_time_err_s=args.arm_time_err_s,
        event_vel_err_frac=args.event_vel_err_frac,
        contact_diag=not args.no_diag, diag_release=args.diag_release,
        report_path=args.report)
    t0 = time.time()
    rep = run_gate(cfg, viewer_speed=viewer_speed)
    wall = time.time() - t0
    for row in rep['points']:
        if (row['binding_2_3_mps'] or row['binding_T080_z170']
                or row.get('binding_8b_ring')):
            band = ('8b-ring' if row.get('binding_8b_ring')
                    else '2-3m/s' if row['binding_2_3_mps'] else 'T0.80@z170')
            print(f"[toss_gate]   {row['point_id']}: core_clean "
                  f"{row['core_clean']}/{row['n']} "
                  f"{'PASS' if row['pass_9_of_10'] else 'FAIL'}  "
                  f"(binding {band}"
                  f"{', hw-marginal' if row['hardware_marginal_flight'] else ''})")
    if rep['no_binding_points']:
        print("[toss_gate] *** NO BINDING POINTS — gating on full band *** "
              "(this run's grid intersects no binding band; "
              "explicit-points/debug run)")
        print(f"[toss_gate] {'PASS' if rep['passed'] else 'FAIL'}  "
              f"full-band {'PASS' if rep['passed_full_band'] else 'FAIL'}  "
              f"(vel-match deferred)")
    elif rep['tier'] == '8b':
        print(f"[toss_gate] {'PASS' if rep['passed'] else 'FAIL'}  "
              f"8b-ring band "
              f"{'PASS' if rep['passed_8b_ring_band'] else 'FAIL'}  "
              f"full-band {'PASS' if rep['passed_full_band'] else 'FAIL'} "
              f"(advisory)  (vel-match deferred)")
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
