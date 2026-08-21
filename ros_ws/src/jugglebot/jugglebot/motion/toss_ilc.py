"""Per-goal critical-point ILC correction — the **layer 3** artifact.

Normative design: ``plans/active/critical-point-ilc.md`` — design constraints
2 (one apply point, nothing in the 40 Hz path), 3 (updated commands are
validated by the REAL gates, never by a linearised surrogate) and 5
(provenance-keyed, dormant on mismatch, byte-identical OFF); § "Sensitivity and
update law" for the per-goal keyed artifact and its Gate-1 key quantisation.
Read § Phase 2 before adding a caller: **where** this artifact may be evaluated
is the load-bearing half of the contract and is not inferable from these
functions. It is evaluated **exactly once per toss GOAL**, in
``reload_coordinator_node._toss_aim_for_goal`` (itself called only from
``_build_toss_cycle``) — never in the 40 Hz emitter, never per Hermite knot,
never in ``catch_coordinator``, and never on the reload path.
``tests/ros/test_toss_ilc_node.py::test_the_ilc_artifact_is_looked_up_in_exactly_one_scope``
is the structural enforcement of that, in the shape of C-TOSS-CAL-1's own D4
manifest.

WHAT THIS IS, AND WHAT IT IS NOT
--------------------------------
It is **layer 3** of the commanded-toss correction stack:

* Layer 0 — ``config/tilt_calibration.yaml`` (:mod:`jugglebot.motion.tilt_map`,
  C-LEVEL-2). INCLINOMETER-measured: *where is gravity at this pose?*
* Layer 1 — ``config/toss_calibration.yaml`` (:mod:`jugglebot.motion.toss_cal`,
  C-TOSS-CAL-1). BALL-measured, persistent, per-pose: *where does the ball
  actually go?* — a 2-DOF commanded **aim**.
* Layer 2 — the session trim (:mod:`jugglebot.toss_trim`), RAM-only, common-mode.
* **Layer 3 — this artifact.** BALL-measured, persistent, per-**goal** (pose AND
  flight time): a small **command-vector** correction learnt offline by
  trust-region ILC from mined per-toss records.

Layer 1 is the 2-DOF special case of layer 3 and the plan says so out loud — the
"absorb or keep" decision between them is **Phase 3's, on A/B evidence**, which
is exactly why the key quantisation below is the aim map's own 150 mm grid: an
ILC cell and an aim-map node must name the same physical pose or the comparison
cannot be made cell by cell.

Until then the two layers are ADDITIVE and cannot double-count, for the reason
the plan's § Relationship gives: v1 learns the residual *on top of* whatever is
applied, and every per-toss record carries the applied aim, so the fit is a
converging fixed point rather than a one-shot measurement.

THE COMMAND VECTOR, AND THE ONE CHANNEL THAT IS REFUSED
-------------------------------------------------------
:data:`ILC_CHANNELS` is ``('aim_rx', 'aim_ry', 'event_vel_trim')`` — the first
three columns of ``tests/hardware/ilc_fit_lib.U_CHANNELS``, in that order.

``release_timing_offset``, the fourth, is **REFUSED by name** from any artifact
(:data:`REFUSED_CHANNELS`). Not "unimplemented": Phase 1's conditioning screen
found its column of ``F`` structurally zero — in the model a dispatch shift is a
rigid translation of the release instant and every landing quantity is a
difference of instants that both carry it — so an artifact carrying one would be
asking this loader to command a channel nothing measured. The quantity a dispatch
shift *does* move (``release_time_err_ms``) is a scheduling error already owned
by ``toss_trim.release_latency_ms`` and by gate G-1.

WHERE EACH BOUND IS ENFORCED, AND WHY THERE IS NO SECOND COPY OF THE D7 CLAMP
-----------------------------------------------------------------------------
* **aim** — bounded per cell at PARSE time (:data:`ILC_AIM_MAX_RAD`), and at
  APPLY only through ``toss_cal.clamp_total_aim`` over ``map + ilc``, which is
  D7's single enforcement point. This module deliberately contains no aim clamp
  of its own: a second clamp is a second bound to drift, and the sum that
  actually needs bounding exists only at the node's apply seam. (Layer 2 is no
  longer in that sum — its aim estimator is MONITOR-ONLY since 2026-08-21,
  ``toss_trim.AIM_AUTHORITY``, contradiction C4.)
* **A clamp HIT at apply is a REFUSAL of the ILC contribution, not a
  truncation** — the node drops the whole ILC aim, WARNs, and re-composes
  ``map`` exactly as it would have without this layer. That mirrors
  ``ilc_fit_lib.admit_command``'s own convention (*"a truncated step is not the
  step that was solved for"*) and closes the plan's risk 5: a partially
  truncated correction desynchronises applied-u from recorded-u, and the learner
  would then be fitting against a command the machine never flew.
* **event_vel trim** — bounded per cell at PARSE time by
  :data:`ILC_SPEED_AUTHORITY` (±0.15, the owner's Gate-1 decision of
  2026-08-13, DEMOTED 2026-08-21 to an outer ceiling), and at APPLY by
  ``reload_coordinator_node._ilc_vel_trim_refusal`` — which runs
  ``toss_release.validate_event_vel`` AND ``throw_envelope.evaluate`` (contract
  C-HAND-3), because the wire band bounds nothing physical and the admissible
  trim is strongly flight-time-dependent (contradiction **C2**). Parse time is
  deliberately NOT made T-dependent: a cell's key carries a *quantised* flight
  time, so the honest gate is the one at apply, holding the goal's real T.
  Nothing else adds to this channel in production — ``toss_trim``'s
  ``speed_k_v`` is recorded but never applied, and RETIRED as of decision 5.

  ``toss_trim.SPEED_AUTHORITY`` (±0.10) **is not this bound and must not be
  changed to match it**: that constant bounds the SESSION trim, a different loop
  with a different update law, measurand and operator model.

**Provenance-gated dormancy (constraint 5).** ``requires.tilt_map_version``,
``requires.toss_cal_version``, ``requires.estimator_version`` and
``requires.model_config_identity`` are all REQUIRED. On any mismatch the
artifact **loads but stays DORMANT** — ``ilc_loaded = true``,
``ilc_applied = false``, loud WARN — mirroring C-TOSS-CAL-1's D3 pattern one
layer up. Root cause, the same one in a new frame: an ILC residual fitted with
aim map A underneath it double-counts aim map B's delta, and a sensitivity ``F``
computed against one geometry config points the update in the wrong direction
under another. ``toss_cal_version`` is compared against the version the machine
is **APPLYING**, not merely the one it loaded: a dormant layer 1 commands zero
aim, which is not the state the residual was fitted on top of.

**Validation is all-or-nothing and loud** (:func:`parse_toss_ilc`): a document
that fails any check raises :class:`TossIlcError` and **no** artifact is loaded,
so the machine degrades to exactly the pre-Phase-2 toss. There is no partial
load and no silent zero-fill — a half-trusted correction is indistinguishable at
the machine from a correct one until a ball misses.

**A key MISS is exactly zero, and never an interpolation.** Unlike the aim map,
which clamps a query into its hull and interpolates bilinearly, this artifact
answers only at cells it was fitted at. Two reasons, both structural rather than
conservative: the value is a whole command vector whose channels have different
units and different authorities, so "half way between two corrections" is not a
correction anybody solved for; and the corpora are sparse (3 goal cells on the
2026-08-12 mine), so a hull clamp would apply one cell's correction across the
entire workspace.

WHAT THIS MODULE MUST NOT IMPORT
---------------------------------
:mod:`jugglebot.motion.toss_cal` — ``tests/motion/test_toss_cal.py::
test_nothing_else_imports_the_loader`` pins ``reload_coordinator_node`` as the
aim map's *single owner*, and a second importer inside the package is a second
owner. :data:`ILC_AIM_MAX_RAD` is therefore stated here and **pinned equal to
``toss_cal.TOTAL_MAX_RAD`` by test** rather than imported, exactly as
``toss_trim`` states its own ``TRIM_MAX_RAD``. Same rule against
``tests/hardware/ilc_fit_lib`` (a test-tree module cannot be a production
dependency): every constant this module shares with the fit is pinned equal in
``tests/motion/test_toss_ilc.py``.

Pure Python — no ROS imports (consumed by ``reload_coordinator_node``, by
``tests/hardware/ilc_fit.py`` and by ``tests/motion`` with no ROS mocking at
all). ``ament_index_python`` is imported lazily and defensively inside the
resolver, exactly as ``tilt_map`` and ``toss_cal`` do.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, NamedTuple, Optional, Sequence, Tuple

import numpy as np

import yaml

import jugglebot.hardware_config as hw
from jugglebot.motion.tilt_map import find_repo_root
from jugglebot.motion.trajectory import ballistics_bc

__all__ = [
    'SCHEMA_VERSION',
    'ESTIMATOR_VERSION',
    'ILC_ENV',
    'ILC_CHANNELS',
    'REFUSED_CHANNELS',
    'ILC_AIM_MAX_RAD',
    'ILC_SPEED_AUTHORITY',
    'POSE_CELL_MM',
    'POSE_Z_CELL_MM',
    'FLIGHT_TIME_CELL_S',
    'AIM_UNIT',
    'VEL_UNIT',
    'MODEL_CONFIG_KEYS',
    'TossIlcError',
    'IlcCorrection',
    'IlcLookup',
    'ZERO_CORRECTION',
    'TossIlc',
    'model_config_identity',
    'goal_key',
    'parse_toss_ilc',
    'load_toss_ilc',
    'lookup',
    'artifact_version',
    'build_document',
    'toss_ilc_candidates',
    'resolve_toss_ilc_path',
]

#: The only schema version this build understands. A document carrying anything
#: else is refused rather than best-effort parsed, for C-TOSS-CAL-1's reason: the
#: fields a future version adds may change what the numbers *mean*, and applying
#: them anyway is a silent command error.
SCHEMA_VERSION = 1

#: Identity of the offline estimator chain whose output an artifact may be fitted
#: from — the mined arrival/backcast/whole-arc fits feeding
#: ``tests/hardware/ilc_fit_lib``. Written into ``requires.estimator_version`` by
#: the fit tool and compared here; a mismatch makes the artifact DORMANT.
#:
#: **Bump it whenever the definition of a mined error channel changes** —
#: ``flight_time_err_s``, ``arrival_dir_err_rad``, ``land_err_mm``, the whole-arc
#: lateral estimator, the ``coverage_asym_s`` refusal. Those choices change what
#: the fitted number *means*, and an artifact fitted under one and applied under
#: another is a plausible version string on a wrong command. ``/1`` is the
#: post-E-1 whole-arc estimator (2026-08-13).
ESTIMATOR_VERSION = 'ilc-critical-point/1'

#: Explicit-override env var. Authoritative when set: it is then the ONLY
#: candidate. See :func:`toss_ilc_candidates` — and note that pointing it at a
#: path that does not exist is the operator's per-launch OFF switch for the
#: Phase-3 A/B, which is why "no artifact" must stay a silent, zero-cost state.
ILC_ENV = 'JUGGLEBOT_TOSS_ILC'

#: THE per-cell command channels, in ``ilc_fit_lib.U_CHANNELS`` order. Order is
#: load-bearing: it is the order the fit's ``u`` array, every column of ``F`` and
#: every screen verdict use, and an artifact written in a different order would
#: put a radian in a dimensionless slot.
ILC_CHANNELS: Tuple[str, ...] = ('aim_rx', 'aim_ry', 'event_vel_trim')

#: Channels an artifact may NOT carry, and the reason each is refused. See the
#: module docstring — a refusal by name beats a silent drop, which would let an
#: operator believe a channel is being commanded when it is not.
REFUSED_CHANNELS: Dict[str, str] = {
    'release_timing_offset': (
        "its column of F is STRUCTURALLY zero (a dispatch shift is a rigid "
        "translation of the release instant, and every landing quantity is a "
        "difference of instants that both carry it), so nothing measured it — "
        "the real seam is toss_trim.release_latency_ms and gate G-1"),
}

#: Per-cell bound on the ILC **aim** magnitude ``hypot(aim_rx, aim_ry)``, rad.
#:
#: **This is deliberately the same number as ``toss_cal.TOTAL_MAX_RAD`` (1.0°)
#: and is pinned equal to it by test** — see the module docstring for why it is
#: restated rather than imported. It is NOT a second enforcement point for D7:
#: the total commanded aim is bounded once, at the node's apply seam, by
#: ``toss_cal.clamp_total_aim``. This bound exists at PARSE time for the reason
#: ``toss_cal._check_aim_magnitude`` exists: a stored cell past the authority is
#: a FIT fault (wrong sign convention, a units error, a fit on a degraded plant),
#: not a calibration, and it must be refused where it is written rather than
#: silently truncated where it is applied.
ILC_AIM_MAX_RAD = math.radians(1.0)

#: THE ILC's own accumulated ``event_vel_trim`` authority, dimensionless
#: (``k_v − 1``). **OWNER DECISION, 2026-08-13, Gate 1** — see
#: ``plans/active/critical-point-ilc.md`` § "Gate 1 CLOSED" and
#: ``tests/hardware/ilc_fit_lib.ILC_SPEED_AUTHORITY``, which carries the same
#: number and the full rail sweep.
#:
#: Root cause, not appeal to the memo: the measured corpus requires
#: ``event_vel_trim = −0.1076`` and two of three goal cells exceed ±0.10 on their
#: own, so ``toss_trim.SPEED_AUTHORITY`` cannot carry this loop. Widening ±0.15
#: is safe for three independent reasons: ``STROKE_TOP_REV`` is algebraically
#: velocity-independent, so a speed trim cannot walk the hand into its end stop;
#: ``validate_event_vel`` still gates every command at apply; and neither rail is
#: reachable anywhere in the sequencer's [0.55, 1.10] s flight-time band
#: (−15 % at T = 0.55 s is 2.30 m/s, 7.7× clear of the 0.3 m/s floor; +15 % at
#: T = 1.10 s is 6.21 m/s, 1.13× inside the 7.0 m/s ceiling — the binding side).
ILC_SPEED_AUTHORITY = 0.15

#: Artifact key: pose xy cell pitch, mm. **Deliberately the aim map's own grid**
#: (``tilt_cal_grid.build_axis(DEFAULT_BOX_MM = 150, nodes = 3)`` gives
#: [−150, 0, +150]), so an ILC cell and an aim-map node name the same physical
#: pose. That matters more than any resolution argument: Phase 3's absorb-or-keep
#: decision compares the two corrections cell by cell and cannot if their grids
#: disagree. Pinned equal to ``ilc_fit_lib.POSE_CELL_MM``.
POSE_CELL_MM = 150.0

#: Artifact key: nominated catch **z** cell, mm. NOT the xy pitch — quantising a
#: 170 mm ACTIVE plane onto a 150 mm grid would label it "150" and pool it with a
#: genuinely different plane. 10 mm is chosen against the miner's own refusal
#: tolerance (``toss_record_miner.PLANE_MISMATCH_TOL_MM`` = 5 mm), so two rows
#: sharing a z cell agree on the measurement plane to inside the tolerance the
#: miner would have refused them over. Pinned equal to
#: ``ilc_fit_lib.POSE_Z_CELL_MM``.
POSE_Z_CELL_MM = 10.0

#: Artifact key: commanded flight-time cell, s. A single-operating-point corpus
#: cannot tell a launch-speed GAIN from an OFFSET; the two hypotheses' corrections
#: diverge across a cell by ``|dv|·(eps/T)``, and holding that under the per-goal
#: noise-equivalent command gives ±28 ms. 50 ms cells (±25 ms) sit inside that
#: with margin and give 11 cells over the sequencer's [0.55, 1.10] s band. Pinned
#: equal to ``ilc_fit_lib.FLIGHT_TIME_CELL_S``.
FLIGHT_TIME_CELL_S = 0.050

#: The only ``units.aim`` this build understands. The aim rides
#: ``toss_release.aim_target_offset_mm`` as a commanded ANGLE, so a mm-valued
#: grid read as radians is a ~3000× error — the key is validated, never assumed.
AIM_UNIT = 'rad'

#: The only ``units.event_vel_trim`` this build understands: ``k_v − 1``, so 0.0
#: is today's machine and −0.10 is a 10 % slower launch. A ``percent``-valued
#: field read as a fraction is a 100× error in a channel that directly scales the
#: hand's throw velocity.
VEL_UNIT = 'dimensionless'

#: The ``hardware_config`` constants the PRODUCTION forward chain reads, and
#: therefore the constants a sensitivity ``F`` was computed through. Hashed into
#: :func:`model_config_identity`, which the artifact records and this loader
#: re-derives live.
#:
#: **Honest limit, stated rather than implied**: this identifies the *config* the
#: model ran on, not the model's *code*. A change to ``toss_release`` geometry
#: that leaves every constant alone will not move this hash — the record's
#: ``git_sha`` is the channel for that. What it does close is the failure this
#: layer actually invites: an artifact fitted at one release-height / cup-offset
#: geometry silently applied at another, where the same ``du`` points somewhere
#: else.
MODEL_CONFIG_KEYS: Tuple[str, ...] = (
    'GEOM_INITIAL_HEIGHT_MM',
    'GEOM_HAND_AXIS_BOTTOM_OFFSET_MM',
    'HAND_THROW_POS_M',
    'HAND_CATCH_OFFSET_MM',
    'TEENSY_TRAJ_MIN_EVENT_VEL_MPS',
    'TEENSY_TRAJ_MAX_EVENT_VEL_MPS',
)

#: Source-tree YAML — the file the fit tool's ``--write-artifact`` writes and the
#: file that is committed. ``None`` when this module runs from a deployment
#: outside the repo.
_REPO_ROOT = find_repo_root(__file__)
_SRC_ILC_YAML = (os.path.join(_REPO_ROOT, 'config', 'toss_ilc.yaml')
                 if _REPO_ROOT else None)


class TossIlcError(ValueError):
    """Any ILC-artifact schema, validation or lookup failure.

    Subclasses ``ValueError`` so a caller that already funnels malformed config
    through ``except ValueError`` keeps working. Layer 3 is **non-gating**, the
    same posture C-TOSS-CAL-1 takes: the loader's obligation on catching this is
    to load NO artifact and log loudly, never to refuse a toss. An absent or
    rejected artifact must produce exactly the pre-Phase-2 machine, including its
    rejection codes.
    """


class IlcCorrection(NamedTuple):
    """One cell's command-vector correction. Units: rad, rad, dimensionless."""
    aim_rx: float
    aim_ry: float
    event_vel_trim: float

    @property
    def aim_rad(self) -> Tuple[float, float]:
        return (self.aim_rx, self.aim_ry)

    @property
    def is_zero(self) -> bool:
        return (self.aim_rx == 0.0 and self.aim_ry == 0.0
                and self.event_vel_trim == 0.0)


#: The correction a MISS returns, and the one a disabled or dormant artifact
#: contributes: exactly zero in every channel, so the composed path is today's
#: machine bit for bit.
ZERO_CORRECTION = IlcCorrection(0.0, 0.0, 0.0)


class IlcLookup(NamedTuple):
    """The result of one per-goal lookup.

    ``hit`` false always carries ``correction is ZERO_CORRECTION``; ``key`` is
    reported either way, because "which cell did this goal quantise onto?" is the
    question an operator asks first when a correction did not apply.
    """
    correction: IlcCorrection
    key: Tuple[float, float, float, float]
    hit: bool


def model_config_identity() -> str:
    """``sha256`` prefix over :data:`MODEL_CONFIG_KEYS` + the ballistic gravity.

    Recomputed live at every provenance check rather than cached: it is ~10 µs of
    hashing at goal-build time, and a cached identity is one more thing that can
    be stale in exactly the situation the identity exists to detect.
    """
    payload = {name: float(getattr(hw, name)) for name in MODEL_CONFIG_KEYS}
    payload['GRAVITY_MMS2'] = float(ballistics_bc.GRAVITY_MMS2)
    digest = hashlib.sha256(
        json.dumps(payload, sort_keys=True).encode('utf-8')).hexdigest()
    return digest[:12]


def goal_key(x_mm: float, y_mm: float, z_mm: float, flight_time_s: float, *,
             pose_cell_mm: float = POSE_CELL_MM,
             z_cell_mm: float = POSE_Z_CELL_MM,
             flight_cell_s: float = FLIGHT_TIME_CELL_S
             ) -> Tuple[float, float, float, float]:
    """Quantise a goal onto its artifact cell — ``(x, y, z, T)``.

    **Byte-for-byte the arithmetic ``ilc_fit_lib.goal_key`` uses**, including the
    ``np.round`` half-to-even rule and the decimal rounding that follows it. The
    two must agree exactly: a cell the fit wrote under one rounding rule and this
    loader looks up under another is a permanent, silent MISS — the artifact
    loads, reports applied, and commands nothing. ``tests/motion/test_toss_ilc.py``
    pins them equal over a swept grid rather than by inspection.

    The xy cells are centred on multiples of :data:`POSE_CELL_MM` about the
    origin, i.e. on the aim map's own node values; z and the flight time get
    their own, finer cells.
    """
    for name, value in (('x_mm', x_mm), ('y_mm', y_mm), ('z_mm', z_mm),
                        ('flight_time_s', flight_time_s)):
        if not math.isfinite(float(value)):
            raise TossIlcError(
                'ILC goal key needs finite values, got {}={!r}'.format(
                    name, value))
    q = float(pose_cell_mm)
    qz = float(z_cell_mm)
    qt = float(flight_cell_s)
    if q <= 0.0 or qz <= 0.0 or qt <= 0.0:
        raise TossIlcError(
            'ILC key cell sizes must be positive, got ({}, {}, {})'.format(
                pose_cell_mm, z_cell_mm, flight_cell_s))
    return (round(float(np.round(float(x_mm) / q)) * q, 3),
            round(float(np.round(float(y_mm) / q)) * q, 3),
            round(float(np.round(float(z_mm) / qz)) * qz, 3),
            round(float(np.round(float(flight_time_s) / qt)) * qt, 4))


@dataclass(frozen=True)
class TossIlc:
    """A validated per-goal ILC correction table.

    ``cells`` maps a :func:`goal_key` tuple to an :class:`IlcCorrection`. It is
    a plain dict rather than a grid because the artifact is **sparse by
    construction**: a corpus visits the goals an operator actually threw, and
    every unvisited cell is a MISS returning exactly zero, not an interpolation.

    The key quantisation is carried in the document and re-stated here rather
    than assumed from the module constants, so an artifact written under a
    different quantisation is *detectable* — :func:`parse_toss_ilc` refuses it
    outright rather than silently looking it up on the wrong grid.
    """

    cells: Dict[Tuple[float, float, float, float], IlcCorrection]
    version: str
    requires_tilt_map_version: str
    requires_toss_cal_version: str
    requires_estimator_version: str
    requires_model_config_identity: str
    pose_cell_mm: float = POSE_CELL_MM
    pose_z_cell_mm: float = POSE_Z_CELL_MM
    flight_time_cell_s: float = FLIGHT_TIME_CELL_S
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        if not isinstance(self.cells, dict):
            raise TossIlcError(
                'TossIlc.cells must be a dict keyed by goal_key tuples, got '
                '{}'.format(type(self.cells).__name__))
        # Frozen and copied for the same reason ``TossCal`` freezes its arrays:
        # the table is a shared per-process single reference that every toss goal
        # reads, so one in-place write would silently re-command every subsequent
        # throw.
        object.__setattr__(self, 'cells', dict(self.cells))
        for key, correction in self.cells.items():
            if not (isinstance(key, tuple) and len(key) == 4):
                raise TossIlcError(
                    'TossIlc cell key must be a 4-tuple (x, y, z, T), got '
                    '{!r} — build via parse_toss_ilc()'.format(key))
            if not isinstance(correction, IlcCorrection):
                raise TossIlcError(
                    'TossIlc cell {!r} is not an IlcCorrection — build via '
                    'parse_toss_ilc()'.format(key))

    @property
    def n_cells(self) -> int:
        return len(self.cells)

    def provenance_mismatch(self, *, tilt_map_version: str,
                            toss_cal_version: str,
                            model_identity: Optional[str] = None
                            ) -> Optional[str]:
        """``None`` if this artifact may be APPLIED, else a one-line reason.

        Four independent gates, each closing a different double-count:

        * **tilt map** — an ILC residual fitted under layer 0 A double-counts
          layer 0 B's delta (C-TOSS-CAL-1's D3, one layer up).
        * **aim map** — ``toss_cal_version`` is compared against the version the
          machine is **APPLYING**, so a DORMANT layer 1 reads as ``''`` here. A
          dormant aim map commands zero aim, which is not the state this residual
          was fitted on top of; treating "loaded" as "applied" would re-learn a
          correction the machine is not making.
        * **estimator** — a correction fitted from one definition of
          ``flight_time_err_s`` applied under another is the R1 frame-error class
          with a plausible version string on it.
        * **model config** — ``F`` is computed through the production geometry
          constants; under a different geometry the same ``du`` points somewhere
          else.

        An **unknown** live version (empty string: no ``trajectory/status`` heard
        yet, or a build that publishes none) is a mismatch, deliberately. "I
        cannot verify what is underneath me" is not "the right thing is
        underneath me". Fail closed, loudly, and cost only precision.
        """
        want_tilt = str(self.requires_tilt_map_version or '')
        live_tilt = str(tilt_map_version or '')
        if want_tilt and live_tilt != want_tilt:
            return ('requires.tilt_map_version {!r} but the live tilt map is {}'
                    .format(want_tilt,
                            repr(live_tilt) if live_tilt else 'UNKNOWN'))
        if not want_tilt and live_tilt:
            return ('requires.tilt_map_version is empty (fitted with NO tilt '
                    'map) but the live tilt map is {!r}'.format(live_tilt))

        want_cal = str(self.requires_toss_cal_version or '')
        live_cal = str(toss_cal_version or '')
        if want_cal and live_cal != want_cal:
            return ('requires.toss_cal_version {!r} but the APPLIED aim map is '
                    '{} (an aim map that is loaded but DORMANT applies nothing '
                    'and reads as absent here)'
                    .format(want_cal, repr(live_cal) if live_cal else 'NONE'))
        if not want_cal and live_cal:
            return ('requires.toss_cal_version is empty (fitted with NO aim map '
                    'applied) but the machine is applying aim map {!r}'
                    .format(live_cal))

        if str(self.requires_estimator_version) != ESTIMATOR_VERSION:
            return ('requires.estimator_version {!r} but this build ships {!r}'
                    .format(self.requires_estimator_version, ESTIMATOR_VERSION))

        live_model = (model_config_identity() if model_identity is None
                      else str(model_identity))
        if str(self.requires_model_config_identity) != live_model:
            return ('requires.model_config_identity {!r} but this build\'s '
                    'forward-chain config hashes to {!r} — the sensitivity F '
                    'this correction was solved through is not the one this '
                    'machine would compute (keys: {})'
                    .format(self.requires_model_config_identity, live_model,
                            ', '.join(MODEL_CONFIG_KEYS)))
        return None


# ── validation helpers ───────────────────────────────────────────


def _mapping(value: Any, what: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise TossIlcError(
            '{} must be a mapping, got {}'.format(what, type(value).__name__))
    return value


def _require(data: Dict[str, Any], key: str, what: str) -> Any:
    if key not in data:
        raise TossIlcError('ILC artifact is missing {}'.format(what))
    return data[key]


def _finite(value: Any, what: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TossIlcError(
            '{} must be a number, got {!r}'.format(what, value))
    out = float(value)
    if not math.isfinite(out):
        raise TossIlcError('{} is not finite: {!r}'.format(what, value))
    return out


def _numeric(value: Any) -> Any:
    """Recursively coerce numbers to float for hashing; pass anything else through.

    Copied from ``toss_cal._numeric`` including both of its audit findings:
    ``170`` and ``170.0`` are the same artifact written by two serializers, and
    ndarrays are converted with ``tolist()`` and recursed rather than passed
    through ``default=str`` (numpy truncates its repr past 1000 elements, so two
    genuinely different tables could otherwise share a version).
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, np.ndarray):
        return _numeric(value.tolist())
    if isinstance(value, np.generic):
        item = value.item()
        return item if isinstance(item, bool) else _numeric(item)
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, (list, tuple)):
        return [_numeric(item) for item in value]
    if isinstance(value, dict):
        return {str(k): _numeric(v) for k, v in value.items()}
    return value


def artifact_version(data: Dict[str, Any]) -> str:
    """``"<captured.date>-<sha256(numbers)[:8]>"`` — the observable identity.

    **The hash covers every number the machine ACTS ON, and nothing else**: the
    schema ``version``, both ``units`` (they decide what the cells *mean*), the
    whole ``key`` quantisation block (it decides which goal reads which cell) and
    every cell's key and channel values. The provenance blocks (``captured``,
    ``requires``, ``fit``) are excluded.

    That rule is C-TOSS-CAL-1's own, restated: *two files whose applied numbers
    are identical report the same version; an edit to a single applied number
    changes it.* The units and the key block are inside the hash precisely
    because an edit to either changes the machine's behaviour without changing a
    single cell value.

    Callable before validation, so a malformed document still hashes
    deterministically.
    """
    if not isinstance(data, dict):
        data = {}
    captured = data.get('captured')
    date = ''
    if isinstance(captured, dict):
        # yaml.safe_load turns a bare `date: 2026-08-13` into datetime.date.
        date = str(captured.get('date', '')).strip()

    def _block(key: str) -> Dict[str, Any]:
        block = data.get(key)
        return block if isinstance(block, dict) else {}

    raw_cells = data.get('cells')
    cells: List[Any] = []
    if isinstance(raw_cells, (list, tuple)):
        for entry in raw_cells:
            if not isinstance(entry, dict):
                cells.append(_numeric(entry))
                continue
            cells.append([_numeric(entry.get('key'))]
                         + [_numeric(entry.get(name))
                            for name in ILC_CHANNELS])
    payload = json.dumps(
        {'version': data.get('version'),
         'units': _numeric(_block('units')),
         'key': _numeric(_block('key')),
         'cells': sorted(cells, key=json.dumps)},
        sort_keys=True, default=str)
    digest = hashlib.sha256(payload.encode('utf-8')).hexdigest()[:8]
    return '{}-{}'.format(date or 'undated', digest)


def _parse_cell(entry: Any, index: int, quant: Tuple[float, float, float]
                ) -> Tuple[Tuple[float, float, float, float], IlcCorrection]:
    cell = _mapping(entry, 'cells[{}]'.format(index))

    for name in sorted(cell):
        if name in REFUSED_CHANNELS:
            raise TossIlcError(
                'cells[{}] carries the REFUSED channel {!r}: {}. Remove it from '
                'the artifact — a channel this build will not command must not '
                'be written as though it will be'
                .format(index, name, REFUSED_CHANNELS[name]))
        if name not in ILC_CHANNELS and name != 'key':
            raise TossIlcError(
                'cells[{}] carries the unknown key {!r}; this build understands '
                '{} plus "key"'.format(index, name,
                                       list(ILC_CHANNELS)))

    raw_key = _require(cell, 'key', 'cells[{}].key'.format(index))
    if not isinstance(raw_key, (list, tuple)) or len(raw_key) != 4:
        raise TossIlcError(
            'cells[{}].key must be [x_mm, y_mm, z_mm, flight_time_s], got '
            '{!r}'.format(index, raw_key))
    coords = [_finite(v, 'cells[{}].key[{}]'.format(index, i))
              for i, v in enumerate(raw_key)]

    # The stored key must BE a cell of the document's own quantisation. A key
    # that is not on the grid could never be hit by a lookup, which would load
    # clean and command nothing — the exact silent failure a sparse table
    # invites, so it is refused where it is written.
    canonical = goal_key(coords[0], coords[1], coords[2], coords[3],
                         pose_cell_mm=quant[0], z_cell_mm=quant[1],
                         flight_cell_s=quant[2])
    if tuple(round(float(v), 4) for v in coords) != tuple(
            round(float(v), 4) for v in canonical):
        raise TossIlcError(
            'cells[{}].key {} is not a cell of this artifact\'s own key '
            'quantisation (pose {} mm / z {} mm / flight {} s) — it quantises '
            'onto {}, so no goal could ever look it up'
            .format(index, coords, quant[0], quant[1], quant[2],
                    list(canonical)))

    values = []
    for name in ILC_CHANNELS:
        values.append(_finite(_require(cell, name,
                                       'cells[{}].{}'.format(index, name)),
                              'cells[{}].{}'.format(index, name)))
    correction = IlcCorrection(values[0], values[1], values[2])

    mag = math.hypot(correction.aim_rx, correction.aim_ry)
    if mag > ILC_AIM_MAX_RAD:
        raise TossIlcError(
            'cells[{}] commands {:.4f}deg of ILC aim, past the {:.2f}deg '
            'authority — that is {:.0f} mm of commanded landing shift at '
            'h = 0.78 m, i.e. a FIT fault (wrong sign convention, a units '
            'error, or a fit on a degraded plant), not a calibration'
            .format(index, math.degrees(mag), math.degrees(ILC_AIM_MAX_RAD),
                    4 * 0.78 * 1000 * mag))
    if abs(correction.event_vel_trim) > ILC_SPEED_AUTHORITY:
        raise TossIlcError(
            'cells[{}] commands event_vel_trim {:+.4f}, past the +-{:.2f} ILC '
            'speed authority (ilc_fit_lib.ILC_SPEED_AUTHORITY, owner decision '
            '2026-08-13). This module REFUSES rather than clamping: widening a '
            'safety bound is an operator decision, and a silently clamped trim '
            'is not the correction that was solved for'
            .format(index, correction.event_vel_trim, ILC_SPEED_AUTHORITY))
    return canonical, correction


def parse_toss_ilc(data: Dict[str, Any]) -> TossIlc:
    """Validate a schema-v1 ILC artifact and build the :class:`TossIlc`.

    All-or-nothing: every failure mode raises :class:`TossIlcError` with a
    message naming the offending field, and nothing partial is returned.
    """
    doc = _mapping(data, 'ILC artifact document')

    raw_version = _require(doc, 'version', "the top-level 'version' key")
    if isinstance(raw_version, bool) or not isinstance(raw_version, int) \
            or raw_version != SCHEMA_VERSION:
        raise TossIlcError(
            'unsupported ILC artifact schema version {!r}; this build '
            'understands version {} only'.format(raw_version, SCHEMA_VERSION))

    units = _mapping(_require(doc, 'units', "the 'units' block"),
                     "the 'units' block")
    aim_unit = str(_require(units, 'aim', 'units.aim')).strip()
    if aim_unit != AIM_UNIT:
        raise TossIlcError(
            'units.aim is {!r}; this build applies the ILC aim as a commanded '
            'ANGLE and understands {!r} only (a mm-valued cell read as radians '
            'is a ~3000x error)'.format(aim_unit, AIM_UNIT))
    vel_unit = str(_require(units, 'event_vel_trim',
                            'units.event_vel_trim')).strip()
    if vel_unit != VEL_UNIT:
        raise TossIlcError(
            'units.event_vel_trim is {!r}; this build applies the trim as '
            'k_v - 1 and understands {!r} only (a percent-valued cell read as a '
            'fraction is a 100x error on the hand\'s throw velocity)'
            .format(vel_unit, VEL_UNIT))

    key_block = _mapping(_require(doc, 'key', "the 'key' block"),
                         "the 'key' block")
    quant = (
        _finite(_require(key_block, 'pose_cell_mm', 'key.pose_cell_mm'),
                'key.pose_cell_mm'),
        _finite(_require(key_block, 'pose_z_cell_mm', 'key.pose_z_cell_mm'),
                'key.pose_z_cell_mm'),
        _finite(_require(key_block, 'flight_time_cell_s',
                         'key.flight_time_cell_s'), 'key.flight_time_cell_s'),
    )
    shipped = (POSE_CELL_MM, POSE_Z_CELL_MM, FLIGHT_TIME_CELL_S)
    if quant != shipped:
        # Refused rather than honoured. Honouring it would look up the right
        # numbers on the wrong grid the moment the shipped quantisation moves —
        # and the Gate-1 argument for these cells (the aim map's own node
        # spacing, the miner's plane tolerance, the gain-vs-offset bound) is a
        # property of THIS build, not of the file.
        raise TossIlcError(
            'key quantisation {} does not match this build\'s {} — an artifact '
            'keyed on a different grid names different physical goals. Re-fit '
            'it, or land the quantisation change in jugglebot.motion.toss_ilc '
            'first (the constants carry their derivations)'
            .format(list(quant), list(shipped)))

    requires = _mapping(_require(doc, 'requires', "the 'requires' block"),
                        "the 'requires' block")
    for name, hint in (
            ('tilt_map_version',
             "use '' to declare 'fitted with NO tilt map loaded'"),
            ('toss_cal_version',
             "use '' to declare 'fitted with NO aim map applied'"),
            ('estimator_version', 'see toss_ilc.ESTIMATOR_VERSION'),
            ('model_config_identity',
             'see toss_ilc.model_config_identity()')):
        if name not in requires:
            raise TossIlcError(
                'ILC artifact is missing requires.{} — an ungateable artifact '
                'cannot be made dormant on a plant change (design constraint '
                '5), so it is refused rather than trusted. {}'
                .format(name, hint))
    tilt_req = str(requires['tilt_map_version'] or '')
    cal_req = str(requires['toss_cal_version'] or '')
    est_req = str(requires['estimator_version'] or '')
    model_req = str(requires['model_config_identity'] or '')
    if not est_req or not model_req:
        raise TossIlcError(
            'requires.estimator_version and requires.model_config_identity may '
            'not be empty — unlike the two map versions there is no "fitted '
            'without one" state for either')

    raw_cells = _require(doc, 'cells', "the 'cells' block")
    if not isinstance(raw_cells, (list, tuple)):
        raise TossIlcError(
            "the 'cells' block must be a list of per-goal entries, got "
            '{}'.format(type(raw_cells).__name__))
    if not raw_cells:
        raise TossIlcError(
            "the 'cells' block is empty — an artifact that corrects nothing is "
            'not the same object as no artifact, and shipping one would report '
            'ilc_applied=true while commanding zero. Delete the file instead')

    cells: Dict[Tuple[float, float, float, float], IlcCorrection] = {}
    for index, entry in enumerate(raw_cells):
        key, correction = _parse_cell(entry, index, quant)
        if key in cells:
            raise TossIlcError(
                'cells[{}] repeats the goal cell {} — two corrections for one '
                'goal is a fit that could not decide, and picking either here '
                'would be this loader deciding for it'.format(index,
                                                              list(key)))
        cells[key] = correction

    metadata = doc.get('captured') or {}
    metadata = dict(_mapping(metadata, "the 'captured' block"))

    return TossIlc(cells=cells,
                   version=artifact_version(doc),
                   requires_tilt_map_version=tilt_req,
                   requires_toss_cal_version=cal_req,
                   requires_estimator_version=est_req,
                   requires_model_config_identity=model_req,
                   pose_cell_mm=quant[0], pose_z_cell_mm=quant[1],
                   flight_time_cell_s=quant[2],
                   metadata=metadata)


def load_toss_ilc(path) -> TossIlc:
    """Read + validate an ILC artifact YAML file. Every failure is a
    :class:`TossIlcError`.

    Unreadable, non-YAML, empty and schema-invalid files all raise the same
    type: the caller's contract is identical in every case — load no artifact,
    log loudly, keep throwing exactly as the pre-Phase-2 machine did. An
    **absent** file is not this function's business; the loader decides that
    absence is silent before calling here.
    """
    try:
        with open(path, 'r') as handle:
            data = yaml.safe_load(handle)
    except OSError as exc:
        raise TossIlcError(
            'cannot read ILC artifact {}: {}'.format(path, exc))
    except yaml.YAMLError as exc:
        raise TossIlcError(
            'ILC artifact {} is not valid YAML: {}'.format(path, exc))
    if data is None:
        raise TossIlcError('ILC artifact {} is empty'.format(path))
    try:
        return parse_toss_ilc(data)
    except TossIlcError as exc:
        raise TossIlcError('{}: {}'.format(path, exc))


# ── path resolution ──────────────────────────────────────────────


def toss_ilc_candidates(
        environ: Optional[Mapping[str, str]] = None) -> Tuple[str, ...]:
    """The candidate paths for ``config/toss_ilc.yaml``, in order.

    ``$JUGGLEBOT_TOSS_ILC`` → **repo source tree** → ament share dir. Source
    tree before share for ``tilt_map``'s reason, inherited by ``toss_cal`` and
    now by this: the fit tool writes the source-tree file and the operator
    relaunches, so share-first would serve the previous ``colcon build``'s stale
    artifact — silently, with ``ilc_loaded`` still true and a plausible version.

    **The env override is authoritative: when it is set it is the ONLY
    candidate**, existing or not. Falling through would apply *a different
    correction than the operator named* while reporting success. It is also the
    Phase-3 A/B's per-launch OFF switch: point it at a path that does not exist
    and the treatment arm degrades to the baseline with no config edit, no
    codegen and no rebuild.

    The ament share dir appears only when ``ament_index_python`` is importable
    AND knows the package; the source-tree entry is absent for a deployment
    outside the repo. So this returns one, two or three entries and callers must
    not index it.
    """
    env = (environ if environ is not None else os.environ).get(ILC_ENV)
    if env:
        return (env,)

    candidates: List[str] = [_SRC_ILC_YAML] if _SRC_ILC_YAML else []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory('jugglebot'),
            'config', 'toss_ilc.yaml'))
    except Exception:
        # ament_index not importable (the pytest venv without ROS 2 sourced) or
        # the package is not on the ament index — the source tree is the answer.
        pass
    return tuple(candidates)


def resolve_toss_ilc_path(
        environ: Optional[Mapping[str, str]] = None) -> Optional[str]:
    """First existing entry of :func:`toss_ilc_candidates`, or ``None``.

    ``None`` means **no ILC correction exists**, which this layer defines as a
    silent, non-gating state producing exactly the pre-Phase-2 toss — so this
    returns rather than raises.
    """
    for candidate in toss_ilc_candidates(environ):
        if os.path.exists(candidate):
            return candidate
    return None


# ── lookup ───────────────────────────────────────────────────────


def lookup(toss_ilc: TossIlc, x_mm: float, y_mm: float, z_mm: float,
           flight_time_s: float) -> IlcLookup:
    """THE per-goal lookup: quantise the goal, return its cell or exact zero.

    ``x_mm`` / ``y_mm`` / ``z_mm`` are the goal's nominated catch pose in STOW
    mm — the frame ``goal_catch_xyz_stow_mm`` records and the frame the fit
    recovers its goals in — and ``flight_time_s`` is the RESOLVED commanded
    flight time.

    **A miss returns :data:`ZERO_CORRECTION` and ``hit = False``.** No hull
    clamp, no nearest neighbour, no interpolation — see the module docstring for
    why a sparse command-vector table must not invent values between its cells.

    A non-finite query raises :class:`TossIlcError` rather than returning a NaN
    correction: a NaN aim would become a NaN virtual target, a NaN launch
    velocity and a NaN ``event_vel`` on the wire, surfacing only as a downstream
    rejection after a goal has already claimed the platform.
    """
    key = goal_key(x_mm, y_mm, z_mm, flight_time_s,
                   pose_cell_mm=toss_ilc.pose_cell_mm,
                   z_cell_mm=toss_ilc.pose_z_cell_mm,
                   flight_cell_s=toss_ilc.flight_time_cell_s)
    correction = toss_ilc.cells.get(key)
    if correction is None:
        return IlcLookup(ZERO_CORRECTION, key, False)
    return IlcLookup(correction, key, True)


# ── document construction (used by the fit tool's writer) ────────


def build_document(cells: Sequence[Tuple[Sequence[float], Sequence[float]]], *,
                   tilt_map_version: str, toss_cal_version: str,
                   date: str, tool: str,
                   estimator_version: str = ESTIMATOR_VERSION,
                   model_identity: Optional[str] = None,
                   captured: Optional[Mapping[str, Any]] = None,
                   fit: Optional[Mapping[str, Any]] = None
                   ) -> Dict[str, Any]:
    """Build a schema-v1 artifact document from fitted cells. Pure; no I/O.

    ``cells`` is a sequence of ``(key4, u3)`` pairs — the goal key and the
    ``(aim_rx, aim_ry, event_vel_trim)`` correction in :data:`ILC_CHANNELS`
    order.

    **The writer lives here, in the production module, and not in the fit tool**,
    for the reason the record schema gives for living in the installed package:
    a writer that carries its own idea of the schema is a second schema, and the
    two drift silently — the artifact still loads, it just means something else.
    ``tests/hardware/ilc_fit.py`` calls this and then serialises the result.

    The returned document is deliberately **not** self-validated here: the caller
    round-trips it through :func:`parse_toss_ilc` before writing, so the exact
    bytes that reach the disk are the bytes this build's loader accepts.
    """
    entries = []
    for raw_key, raw_u in cells:
        key = [float(v) for v in raw_key]
        u = [float(v) for v in raw_u]
        if len(key) != 4 or len(u) != len(ILC_CHANNELS):
            raise TossIlcError(
                'build_document needs (key4, u{}) pairs, got key={} u={}'
                .format(len(ILC_CHANNELS), key, u))
        entry: Dict[str, Any] = {'key': key}
        for name, value in zip(ILC_CHANNELS, u):
            entry[name] = value
        entries.append(entry)

    captured_block: Dict[str, Any] = {'date': str(date), 'tool': str(tool)}
    if captured:
        captured_block.update({str(k): v for k, v in captured.items()})

    doc: Dict[str, Any] = {
        'version': SCHEMA_VERSION,
        'captured': captured_block,
        'requires': {
            'tilt_map_version': str(tilt_map_version or ''),
            'toss_cal_version': str(toss_cal_version or ''),
            'estimator_version': str(estimator_version),
            'model_config_identity': str(
                model_config_identity() if model_identity is None
                else model_identity),
        },
        'units': {'aim': AIM_UNIT, 'event_vel_trim': VEL_UNIT},
        'key': {'pose_cell_mm': POSE_CELL_MM,
                'pose_z_cell_mm': POSE_Z_CELL_MM,
                'flight_time_cell_s': FLIGHT_TIME_CELL_S},
        'cells': entries,
    }
    if fit:
        doc['fit'] = {str(k): v for k, v in fit.items()}
    return doc
