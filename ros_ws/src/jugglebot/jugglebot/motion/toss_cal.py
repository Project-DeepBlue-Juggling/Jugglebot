"""Pose-dependent toss AIM calibration map — contract **C-TOSS-CAL-1**.

Normative design: ``plans/active/toss-selftuning.md`` §§ 3.1, 3.2, 3.7 and
decisions D1–D4. Read § 3.1 before adding a caller: **where** this map may be
evaluated is the load-bearing half of the contract and is not inferable from
these functions. It is evaluated **exactly once per toss GOAL**, in
``reload_coordinator_node._build_toss_cycle`` — never in the 40 Hz emitter,
never per Hermite knot, never in ``catch_coordinator``, and never on the reload
path (a BallButler ball's aim belongs to BallButler). ``tests/motion/
test_toss_cal.py::test_the_map_is_looked_up_in_exactly_one_scope`` is the
structural enforcement of that (D4).

**What this map is, and what it is NOT.** It is *layer 1* of the three-layer
stack in § 3.1:

* Layer 0 — ``config/tilt_calibration.yaml`` (:mod:`jugglebot.motion.tilt_map`,
  C-LEVEL-2). INCLINOMETER-measured, answers "where is gravity at this pose?",
  applied at the six C-LEVEL-1 ingest sites inside ``trajectory_node``.
* **Layer 1 — this map.** BALL-measured, answers "where does the ball actually
  go?", applied as a deliberately non-level **commanded aim tilt** on the toss
  goal. It never touches :mod:`jugglebot.motion.levelling`, so C-LEVEL-1's "no
  second implementation" clause survives untouched (D1).
* Layer 2 — the session trim (RAM only, phase 2e).

The inclinometer senses gravity, so layer 0 nulls the *gravity-frame*
orientation error and nothing else — it is structurally blind to hand-axis
misalignment, cup/release asymmetry and platform residual motion, all of which
move the ball identically. Layer 1 is by construction the residual left after
layer 0, measured by a different sensor answering a different question, so there
is no shared parameter to double-count.

**Units, and why this build refuses anything but radians.** ``units.aim`` must
read ``rad``. The lateral landing offset of a vertical toss tilted θ from
vertical is ``b = 4·h·θ`` (§ F1) — an angle is therefore height-portable and one
map serves every ``throw_height_m``. SC-1 (the height ladder) may yet find the
residual is *fixed-mm* or *impulse*-shaped, in which case the map's unit
changes; a mm-valued grid silently read as radians is a ~3000× error, so the key
is validated rather than assumed.

**Sign/frame convention — the one that matters.** ``aim_rad`` stores the
**COMMANDED aim tilt** ``(rx, ry)`` in the production
``toss_release.TiltedReleaseState.tilt_rx``/``tilt_ry`` convention: the number
the machine tilts the platform by at release, NOT the measured plant bias. The
fit tool (phase 2c) owns turning measured landing errors into the commanded aim
that cancels them, sign included, and rung SC-0 measures that sign on hardware
rather than assuming it. Consequences worth stating plainly:

* Zero ⇒ vertical ⇒ **exactly today's machine**, bit for bit.
* The apply path needs no sign flip anywhere, so there is no runtime place for
  a sign to be wrong.
* Because the *applied* aim is recorded per toss (``map_aim_rad``), the fit is a
  converging fixed point rather than a one-shot measurement — captures do not
  have to run with the map uninstalled (§ 3.7 item 3).

**Provenance-gated dormancy (D3).** ``requires.tilt_map_version`` and
``requires.estimator_version`` are REQUIRED. On mismatch the map **loads but
stays DORMANT** — ``toss_cal_loaded = true``, ``toss_cal_applied = false``, loud
WARN — mirroring the ``TrajectoryStatus.tilt_map_loaded`` / ``tilt_map_version``
pattern C-LEVEL-2 already ships. Root cause: an aim residual fitted under tilt
map A double-counts tilt map B's delta. A re-``level`` does **not** invalidate
the map: it perturbs only the common mode, which is layer 2's job (§ 3.2).

**Validation is all-or-nothing and loud** (:func:`parse_toss_cal`): a document
that fails any check raises :class:`TossCalError` and **no** map is loaded, so
the machine degrades to exactly the pre-2b toss. There is deliberately no
partial load, no per-node repair and no silent zero-fill — a half-trusted aim
map is indistinguishable at the machine from a correct one until a ball misses,
and unlike a tilt residual an aim error is *the* thing that misses.

**Path resolution** (:func:`resolve_toss_cal_path`) copies
:mod:`jugglebot.motion.tilt_map` verbatim, including the repo-root *marker
search* rather than a fixed ``__file__`` walk — see
:func:`jugglebot.motion.tilt_map.find_repo_root` for why a fixed walk makes the
source-tree candidate unable to exist under ``colcon``, silently restoring the
stale-calibration trap. ``$JUGGLEBOT_TOSS_CAL`` is authoritative when set (it is
then the ONLY candidate): falling through would apply *a different calibration
than the operator named* while reporting a plausible version, which is strictly
worse than applying none.

Pure Python — no ROS imports (consumed by ``reload_coordinator_node`` and by
``tests/motion`` with no ROS mocking at all). ``ament_index_python`` is imported
lazily and defensively inside the resolver, exactly as ``tilt_map`` does.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Tuple

import numpy as np
import yaml

from jugglebot.motion.tilt_map import find_repo_root

__all__ = [
    'SCHEMA_VERSION',
    'ESTIMATOR_VERSION',
    'TOTAL_MAX_RAD',
    'TOSS_CAL_ENV',
    'AIM_UNIT',
    'TossCalError',
    'TossCal',
    'parse_toss_cal',
    'load_toss_cal',
    'lookup',
    'clamp_total_aim',
    'map_version',
    'toss_cal_candidates',
    'resolve_toss_cal_path',
]

#: The only schema version this build understands. A document carrying anything
#: else is refused rather than best-effort parsed: the fields a future version
#: adds may change what the numbers *mean*, and applying them anyway is a silent
#: aim error — the one class of error that ends in a dropped ball.
SCHEMA_VERSION = 1

#: Identity of the arrival-offset estimator whose output this map may be fitted
#: from. Written into ``requires.estimator_version`` by the phase-2c fit tool and
#: compared here; a mismatch makes the map DORMANT (D3). Bump it whenever the
#: definition of ``land_err_mm`` changes (fit plane rule, descending-band width,
#: lateral gate, minimum sample count) — those choices change what the fitted
#: number *means*, and a map fitted under one and applied under another is the
#: R1 frame-error class with a plausible version string on it.
ESTIMATOR_VERSION = 'arrival-offset/1'

#: THE aim authority bound, radians, as a **magnitude** ``hypot(rx, ry)``.
#: 1.0° — the design's ``TOTAL_MAX`` (§ 3.6 constants table, D7). Three
#: independent reasons it is this number and not a looser one:
#:
#: * it keeps the additive rotation-vector composition ``R_corr(level+tiltmap) ·
#:   R_aim`` inside C-LEVEL-2's documented 1°×1° regime entry (cross term
#:   1.523e-4 rad = 0.0087° = 0.48 mm of landing offset);
#: * it caps the cup-swing side effect at ``HAND_CATCH_OFFSET_MM · sin(1°)`` =
#:   **1.131 mm** (measured by ``/tmp/probe_toss_cal_aim.py``, 2026-08-11);
#: * at the working height it is ~55 mm of commanded landing shift
#:   (54.578 mm at a full 1° aim, h = 0.78 m, same probe — a SECANT, which is
#:   the right quantity for "what does saturating this clamp buy"; the
#:   derivative at zero aim is 54.5718 mm/deg, see D3 in
#:   ``toss_release.aim_target_offset_mm``) — 1.6× the 35 mm capture radius, so a
#:   saturated wrong-signed aim is guaranteed to miss and must never be
#:   exceeded by an unreviewed number.
#:
#: Enforced TWICE: per node at parse time (a stored node beyond it is a capture
#: fault, not a calibration) and again at APPLY (:func:`clamp_total_aim`), which
#: is where it becomes load-bearing once phase 2e's session trim adds to it.
TOTAL_MAX_RAD = math.radians(1.0)

#: The only ``units.aim`` this build understands — see the module docstring.
AIM_UNIT = 'rad'

#: Explicit-override env var. Authoritative when set: it is then the ONLY
#: candidate. See :func:`toss_cal_candidates`.
TOSS_CAL_ENV = 'JUGGLEBOT_TOSS_CAL'

#: Source-tree YAML — the file the phase-2c fit tool writes and the file that is
#: committed. ``None`` when this module runs from a deployment outside the repo.
_REPO_ROOT = find_repo_root(__file__)
_SRC_TOSS_YAML = (os.path.join(_REPO_ROOT, 'config', 'toss_calibration.yaml')
                  if _REPO_ROOT else None)


class TossCalError(ValueError):
    """Any toss-calibration schema, validation or lookup failure.

    Subclasses ``ValueError`` so a caller that already funnels malformed config
    through ``except ValueError`` keeps working. C-TOSS-CAL-1 is non-gating: the
    loader's obligation on catching this is to **not load a map** and log
    loudly, never to refuse a toss. An absent or rejected aim map must produce
    exactly the pre-2b machine, including its rejection codes.
    """


@dataclass(frozen=True)
class TossCal:
    """A validated commanded-aim map over an (x, y) grid at one capture height.

    ``rx_rad`` / ``ry_rad`` are indexed ``[iy][ix]`` — row selects ``y_mm``,
    column selects ``x_mm`` — matching the YAML nesting order and numpy's
    row-major convention. Getting this transposed is the obvious mistake and is
    invisible on a symmetric grid, which is why the unit tests use a
    deliberately **asymmetric** map.

    All four arrays are **copied and marked read-only** at construction: copied
    so a caller that hands in its own working arrays (the fit tool
    self-validating before it writes) does not find them frozen for the rest of
    its run; read-only because the map is a shared per-process single reference
    that every toss goal reads, so one in-place write would silently re-aim
    every subsequent throw.

    The structural invariants live here as well as in :func:`parse_toss_cal`,
    for the same reason ``TiltMap`` does it: :func:`lookup` divides by a cell
    width, so a hand-built 1-node axis would raise ``ZeroDivisionError`` — an
    exception type no caller is told to expect — instead of the
    :class:`TossCalError` every other failure raises.
    """

    x_mm: np.ndarray
    y_mm: np.ndarray
    rx_rad: np.ndarray
    ry_rad: np.ndarray
    version: str
    requires_tilt_map_version: str
    requires_estimator_version: str
    z_mm: Optional[float] = None    # capture height; provenance only
    anchor_aim_rad: Optional[Tuple[float, float]] = None   # 2e warm-start prior
    speed_k_v: Optional[float] = None                      # 2e speed trim prior
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        arrays = {}
        for name in ('x_mm', 'y_mm', 'rx_rad', 'ry_rad'):
            try:
                array = np.array(getattr(self, name), dtype=float)
            except (TypeError, ValueError) as exc:
                raise TossCalError(
                    f"TossCal.{name} is not a numeric array: {exc}") from exc
            array.flags.writeable = False
            arrays[name] = array
            object.__setattr__(self, name, array)

        for name in ('x_mm', 'y_mm'):
            axis = arrays[name]
            if axis.ndim != 1 or axis.size < 2:
                raise TossCalError(
                    f"TossCal.{name} must be 1-D with at least 2 nodes, got "
                    f"shape {axis.shape} — build via parse_toss_cal()")
            if not np.all(np.diff(axis) > 0.0):
                raise TossCalError(
                    f"TossCal.{name} must be strictly increasing — a repeated "
                    f"node is a zero-width cell")

        expected = (int(arrays['y_mm'].size), int(arrays['x_mm'].size))
        for name in ('rx_rad', 'ry_rad'):
            if arrays[name].shape != expected:
                raise TossCalError(
                    f"TossCal.{name} must be indexed [iy][ix] with shape "
                    f"{expected}, got {arrays[name].shape}")

    @property
    def shape(self) -> Tuple[int, int]:
        """``(n_y, n_x)`` — the aim grids' shape."""
        return (int(self.y_mm.size), int(self.x_mm.size))

    def provenance_mismatch(self, tilt_map_version: str) -> Optional[str]:
        """``None`` if this map may be APPLIED against ``tilt_map_version``,
        else a one-line reason (D3).

        An **unknown** live tilt-map version (empty string: no
        ``trajectory/status`` heard yet, or a build that publishes none) is a
        mismatch, deliberately. "I cannot verify which layer 0 is underneath
        me" is not "the right one is underneath me", and applying an aim
        residual on an unverified layer 0 is precisely the double-count D3
        exists to prevent. Fail closed, loudly, and cost only precision.
        """
        want_tilt = str(self.requires_tilt_map_version or '')
        live = str(tilt_map_version or '')
        if want_tilt and live != want_tilt:
            return ("requires.tilt_map_version {!r} but the live tilt map is "
                    "{}".format(want_tilt, repr(live) if live else 'UNKNOWN'))
        if not want_tilt:
            # A map that declares "no tilt map required" is only consistent with
            # a machine that has none loaded; otherwise the capture ran under a
            # layer 0 it did not record.
            if live:
                return (f"requires.tilt_map_version is empty (captured with NO "
                        f"tilt map) but the live tilt map is {live!r}")
        if str(self.requires_estimator_version) != ESTIMATOR_VERSION:
            return (f"requires.estimator_version "
                    f"{self.requires_estimator_version!r} but this build ships "
                    f"{ESTIMATOR_VERSION!r}")
        return None


# ── validation helpers ───────────────────────────────────────────


def _mapping(value: Any, what: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise TossCalError(
            f"{what} must be a mapping, got {type(value).__name__}")
    return value


def _require(data: Dict[str, Any], key: str, what: str) -> Any:
    if key not in data:
        raise TossCalError(f"toss calibration is missing {what}")
    return data[key]


def _axis(raw: Any, name: str) -> np.ndarray:
    """Validate one grid axis: 1-D, >= 2 nodes, finite, strictly increasing."""
    try:
        axis = np.asarray(raw, dtype=float)
    except (TypeError, ValueError) as exc:
        raise TossCalError(f"grid.{name} is not a numeric array: {exc}") from exc
    if axis.ndim != 1:
        raise TossCalError(
            f"grid.{name} must be a 1-D list, got shape {axis.shape}")
    if axis.size < 2:
        # A single-node axis has no cell to interpolate across; every query
        # would clamp onto that node, which is a constant bias masquerading as a
        # map. Refuse it rather than silently degrade to "one global aim".
        raise TossCalError(
            f"grid.{name} needs at least 2 nodes to interpolate, got "
            f"{axis.size}")
    if not np.all(np.isfinite(axis)):
        raise TossCalError(
            f"grid.{name} contains non-finite values: {axis.tolist()}")
    if not np.all(np.diff(axis) > 0.0):
        raise TossCalError(
            f"grid.{name} must be strictly increasing (ascending, no repeats), "
            f"got {axis.tolist()}")
    return axis


def _aim_grid(raw: Any, name: str, n_y: int, n_x: int) -> np.ndarray:
    """Validate one aim grid: shape ``(n_y, n_x)`` and finite. The authority
    bound is a MAGNITUDE and is checked across both axes together, in
    :func:`_check_aim_magnitude`."""
    try:
        grid = np.asarray(raw, dtype=float)
    except (TypeError, ValueError) as exc:
        raise TossCalError(
            f"aim_rad.{name} is not a numeric 2-D array (ragged rows or a "
            f"non-numeric entry?): {exc}") from exc
    if grid.shape != (n_y, n_x):
        raise TossCalError(
            f"aim_rad.{name} must be indexed [iy][ix] with shape "
            f"(len(grid.y_mm), len(grid.x_mm)) = ({n_y}, {n_x}), got "
            f"{grid.shape}")
    if not np.all(np.isfinite(grid)):
        bad = [(int(iy), int(ix))
               for iy, ix in zip(*np.nonzero(~np.isfinite(grid)))]
        raise TossCalError(
            f"aim_rad.{name} contains non-finite values at [iy][ix] {bad} — a "
            f"failed capture node must be resolved by the fit tool (kept at its "
            f"previous value and marked stale, § 3.7 item 7), never shipped as "
            f"NaN")
    return grid


def _check_aim_magnitude(rx: np.ndarray, ry: np.ndarray) -> None:
    """The authority bound, applied to ``hypot(rx, ry)`` per node.

    A magnitude, not a per-axis box: the physical quantity the bound protects is
    the commanded from-vertical aim, and a per-axis box would permit
    ``hypot = 1.414°`` at the corners — 41 % past the authority every downstream
    argument (composition regime, cup swing, landing shift) is sized on.
    """
    mag = np.hypot(rx, ry)
    if mag.size and float(np.max(mag)) > TOTAL_MAX_RAD:
        iy, ix = np.unravel_index(int(np.argmax(mag)), mag.shape)
        peak = float(mag[iy][ix])
        raise TossCalError(
            f"aim_rad node [iy={int(iy)}][ix={int(ix)}] commands "
            f"{math.degrees(peak):.4f}° of aim, past the {math.degrees(TOTAL_MAX_RAD):.2f}° "
            f"authority bound — that is {4 * 0.78 * 1000 * peak:.0f} mm of "
            f"commanded landing shift at h = 0.78 m, i.e. a capture fault (wrong "
            f"sign convention, a units error, or a fit on a degraded plant), not "
            f"a calibration")


# ── parse / load ─────────────────────────────────────────────────


def _numeric(value: Any) -> Any:
    """Recursively coerce numbers to float for hashing; pass anything else through.

    Copied from ``tilt_map._numeric``, including both of the audit findings that
    shaped it: ``170`` and ``170.0`` are the same calibration written by two
    serializers (so numbers are normalised before hashing, or the fit tool's
    re-emit churns the version and sends an operator hunting a difference that
    does not exist), and **ndarrays are converted with ``tolist()`` and
    recursed, never passed through ``default=str``** — an ndarray and a list of
    the same numbers hash differently, and numpy truncates its repr past 1000
    elements to ``[0.1, 0.2, ..., 0.9]``, so on a large grid two genuinely
    different maps could share a version. That is exactly the silent-wrong-map
    failure the version string exists to make impossible.
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
    return value


def map_version(data: Dict[str, Any]) -> str:
    """``"<captured.date>-<sha256(numbers)[:8]>"`` — the observable map identity.

    Published on ``toss/calibration_status``, carried in every toss record's
    ``toss_cal_version``, and read back by the phase-2c fit tool after it calls
    ``toss/reload_calibration`` — that readback is the hard guarantee that the
    node loaded the file the tool wrote.

    **The hash covers every number the machine ACTS ON, and nothing else.**
    That is: the schema ``version``, ``units.aim`` (it decides what the grids
    *mean*), both grid **axes**, both ``aim_rad`` grids, ``anchor.aim_rad`` and
    ``speed.k_v``. The provenance blocks (``captured``, ``requires``,
    ``jacobian``, ``stats``, and ``grid.z_mm`` / ``grid.orientation``) are
    excluded.

    This is one step wider than the design's literal "schema version, both axes,
    the bias grids" (§ 3.7), and deliberately so: ``anchor.aim_rad`` is the
    session trim's warm-start prior and ``speed.k_v`` the speed prior, so phase
    2e *acts on both*. Under the literal rule an edit to either would change the
    machine's behaviour without changing the version — precisely the property
    the version string exists to deny. The rule this implements is the design's
    own stated intent: *two files whose applied numbers are identical report the
    same version; an edit to a single applied node changes it.*

    Numbers are float-normalised before hashing (see :func:`_numeric`). The date
    prefix makes the string readable at a glance; the digest is what makes it
    identifying. Callable before validation, so a malformed document still
    hashes deterministically.
    """
    if not isinstance(data, dict):
        data = {}
    captured = data.get('captured')
    date = ''
    if isinstance(captured, dict):
        # yaml.safe_load turns a bare `date: 2026-08-11` into datetime.date.
        date = str(captured.get('date', '')).strip()

    def _block(key: str) -> Dict[str, Any]:
        block = data.get(key)
        return block if isinstance(block, dict) else {}

    grid = _block('grid')
    aim = _block('aim_rad')
    payload = json.dumps(
        {'version': data.get('version'),
         'unit': _block('units').get('aim'),
         'x_mm': _numeric(grid.get('x_mm')),
         'y_mm': _numeric(grid.get('y_mm')),
         'rx': _numeric(aim.get('rx')),
         'ry': _numeric(aim.get('ry')),
         'anchor': _numeric(_block('anchor').get('aim_rad')),
         'k_v': _numeric(_block('speed').get('k_v'))},
        sort_keys=True, default=str)
    digest = hashlib.sha256(payload.encode('utf-8')).hexdigest()[:8]
    return f"{date or 'undated'}-{digest}"


def _optional_pair(raw: Any, what: str) -> Optional[Tuple[float, float]]:
    if raw is None:
        return None
    try:
        pair = np.asarray(raw, dtype=float).reshape(2)
    except (TypeError, ValueError) as exc:
        raise TossCalError(f"{what} is not a 2-vector of numbers: {exc}") from exc
    if not np.all(np.isfinite(pair)):
        raise TossCalError(f"{what} contains non-finite values: {pair.tolist()}")
    if float(np.hypot(pair[0], pair[1])) > TOTAL_MAX_RAD:
        raise TossCalError(
            f"{what} magnitude {math.degrees(float(np.hypot(*pair))):.4f}° is "
            f"past the {math.degrees(TOTAL_MAX_RAD):.2f}° authority bound")
    return (float(pair[0]), float(pair[1]))


def parse_toss_cal(data: Dict[str, Any]) -> TossCal:
    """Validate a schema-v1 toss-calibration document and build the :class:`TossCal`.

    All-or-nothing: every failure mode raises :class:`TossCalError` with a
    message naming the offending field, and nothing partial is returned.
    """
    doc = _mapping(data, 'toss calibration document')

    raw_version = _require(doc, 'version', "the top-level 'version' key")
    if isinstance(raw_version, bool) or not isinstance(raw_version, int) \
            or raw_version != SCHEMA_VERSION:
        raise TossCalError(
            f"unsupported toss calibration schema version {raw_version!r}; this "
            f"build understands version {SCHEMA_VERSION} only")

    units = _mapping(_require(doc, 'units', "the 'units' block"),
                     "the 'units' block")
    unit = str(_require(units, 'aim', "units.aim")).strip()
    if unit != AIM_UNIT:
        raise TossCalError(
            f"units.aim is {unit!r}; this build applies the map as a commanded "
            f"aim ANGLE and understands {AIM_UNIT!r} only. A mm- or "
            f"impulse-valued grid read as radians is a ~3000x error — SC-1 "
            f"decides the unit, and a non-rad map needs a build that knows how "
            f"to apply it")

    # requires: REQUIRED whole, because D3's dormancy gate is the thing that
    # stops a map fitted under one layer 0 being applied on another. A map with
    # no provenance cannot be gated, and an ungateable map is refused rather
    # than trusted.
    requires = _mapping(_require(doc, 'requires', "the 'requires' block"),
                        "the 'requires' block")
    if 'tilt_map_version' not in requires:
        raise TossCalError(
            "toss calibration is missing requires.tilt_map_version — an aim "
            "residual fitted under tilt map A double-counts tilt map B's delta "
            "(D3), so the map it was captured under is mandatory provenance. "
            "Use '' to declare 'captured with NO tilt map loaded'")
    tilt_req = str(requires['tilt_map_version'] or '')
    est_req = str(_require(requires, 'estimator_version',
                           "requires.estimator_version") or '')

    grid = _mapping(_require(doc, 'grid', "the 'grid' block"), "the 'grid' block")
    x_mm = _axis(_require(grid, 'x_mm', "grid.x_mm"), 'x_mm')
    y_mm = _axis(_require(grid, 'y_mm', "grid.y_mm"), 'y_mm')

    z_mm = None
    if grid.get('z_mm') is not None:
        try:
            z_mm = float(grid['z_mm'])
        except (TypeError, ValueError) as exc:
            raise TossCalError(f"grid.z_mm is not a number: {exc}") from exc
        if not math.isfinite(z_mm):
            raise TossCalError(f"grid.z_mm is not finite: {z_mm}")

    aim = _mapping(_require(doc, 'aim_rad', "the 'aim_rad' block"),
                   "the 'aim_rad' block")
    n_y, n_x = int(y_mm.size), int(x_mm.size)
    rx = _aim_grid(_require(aim, 'rx', "aim_rad.rx"), 'rx', n_y, n_x)
    ry = _aim_grid(_require(aim, 'ry', "aim_rad.ry"), 'ry', n_y, n_x)
    _check_aim_magnitude(rx, ry)

    anchor_block = doc.get('anchor') or {}
    anchor = _optional_pair(_mapping(anchor_block, "the 'anchor' block")
                            .get('aim_rad'), 'anchor.aim_rad')

    speed_block = doc.get('speed') or {}
    k_v = _mapping(speed_block, "the 'speed' block").get('k_v')
    if k_v is not None:
        try:
            k_v = float(k_v)
        except (TypeError, ValueError) as exc:
            raise TossCalError(f"speed.k_v is not a number: {exc}") from exc
        if not math.isfinite(k_v):
            raise TossCalError(f"speed.k_v is not finite: {k_v}")

    metadata = doc.get('captured', {})
    if metadata is None:
        metadata = {}
    metadata = dict(_mapping(metadata, "the 'captured' block"))

    # TossCal.__post_init__ copies and freezes the four arrays — deliberately
    # not done here, so a caller's own arrays are never frozen behind its back.
    return TossCal(x_mm=x_mm, y_mm=y_mm, rx_rad=rx, ry_rad=ry,
                   version=map_version(doc),
                   requires_tilt_map_version=tilt_req,
                   requires_estimator_version=est_req,
                   z_mm=z_mm, anchor_aim_rad=anchor, speed_k_v=k_v,
                   metadata=metadata)


def load_toss_cal(path) -> TossCal:
    """Read + validate a toss-calibration YAML file. Every failure is a
    :class:`TossCalError`.

    Unreadable, non-YAML, empty and schema-invalid files all raise the same
    type: the caller's contract is identical in every case — do not load a map,
    log loudly, keep throwing exactly as the pre-2b machine did. An **absent**
    file is not this function's business; the loader decides that absence is
    silent (soft absence, non-gating) before calling here.
    """
    try:
        with open(path, 'r') as handle:
            data = yaml.safe_load(handle)
    except OSError as exc:
        raise TossCalError(f"cannot read toss calibration {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise TossCalError(
            f"toss calibration {path} is not valid YAML: {exc}") from exc
    if data is None:
        raise TossCalError(f"toss calibration {path} is empty")
    try:
        return parse_toss_cal(data)
    except TossCalError as exc:
        raise TossCalError(f"{path}: {exc}") from exc


# ── path resolution ──────────────────────────────────────────────


def toss_cal_candidates(
        environ: Optional[Mapping[str, str]] = None) -> Tuple[str, ...]:
    """The candidate paths for ``config/toss_calibration.yaml``, in order.

    ``$JUGGLEBOT_TOSS_CAL`` → **repo source tree** → ament share dir. Source
    tree before share for the reason ``tilt_map`` inverted the order: the
    acquisition tool *rewrites the source-tree file at runtime* and then asks the
    running node to reload it, so share-first would serve the previous build's
    stale calibration until the next ``colcon build`` — silently, with
    ``toss_cal_loaded`` still true and a plausible-looking version.

    **The env override is authoritative: when it is set it is the ONLY
    candidate**, existing or not. Falling through would apply *a different
    calibration than the operator named* while reporting success, which is
    strictly worse than applying none (C-TOSS-CAL-1 is non-gating, so applying
    none costs only precision).

    The ament share dir appears only when ``ament_index_python`` is importable
    AND knows the package — under the pytest venv it simply does not, and the
    source-tree entry is likewise absent for a deployment outside the repo. So
    this returns one, two or three entries and callers must not index it.
    """
    env = (environ if environ is not None else os.environ).get(TOSS_CAL_ENV)
    if env:
        return (env,)

    candidates: List[str] = [_SRC_TOSS_YAML] if _SRC_TOSS_YAML else []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory('jugglebot'),
            'config', 'toss_calibration.yaml'))
    except Exception:
        # ament_index not importable (the pytest venv without ROS 2 sourced) or
        # the package is not on the ament index — the source tree is the answer.
        pass
    return tuple(candidates)


def resolve_toss_cal_path(
        environ: Optional[Mapping[str, str]] = None) -> Optional[str]:
    """First existing entry of :func:`toss_cal_candidates`, or ``None``.

    ``None`` means **no aim calibration exists**, which C-TOSS-CAL-1 defines as
    a silent, non-gating state producing exactly the pre-2b toss — so this
    returns rather than raises.
    """
    for candidate in toss_cal_candidates(environ):
        if os.path.exists(candidate):
            return candidate
    return None


# ── lookup ───────────────────────────────────────────────────────


def _cell_index(axis: np.ndarray, value: float) -> int:
    """Index of the cell's lower node, clamped so ``idx + 1`` is always valid."""
    idx = int(np.searchsorted(axis, value, side='right')) - 1
    return min(max(idx, 0), int(axis.size) - 2)


def lookup(toss_cal: TossCal, x_mm: float, y_mm: float) -> Tuple[float, float]:
    """Bilinear commanded aim ``(rx, ry)`` in radians at ``(x_mm, y_mm)``.

    ``x_mm`` / ``y_mm`` are the goal's nominated catch pose xy in STOW mm — the
    same frame the grid was captured in.

    The query is **clamped into the calibrated hull first**, then interpolated,
    so a pose outside the grid gets the nearest hull point's aim and the map
    never extrapolates. C-LEVEL-2 states the reason and it applies with more
    force here: a wrong-signed edge extrapolation aims a throw *worse* than no
    map at all. Exact at every node (the weights collapse to 0.0/1.0 there, so a
    node query returns its stored value bit-identically).

    A non-finite query raises :class:`TossCalError` rather than returning NaN: a
    NaN aim would become a NaN virtual target, a NaN launch velocity and a NaN
    ``event_vel`` on the wire, surfacing only as a downstream rejection after a
    goal has already claimed the platform.
    """
    x = float(x_mm)
    y = float(y_mm)
    if not (math.isfinite(x) and math.isfinite(y)):
        raise TossCalError(
            f"toss calibration query must be finite, got (x={x_mm!r}, y={y_mm!r})")

    xs = toss_cal.x_mm
    ys = toss_cal.y_mm
    x = min(max(x, float(xs[0])), float(xs[-1]))
    y = min(max(y, float(ys[0])), float(ys[-1]))

    ix = _cell_index(xs, x)
    iy = _cell_index(ys, y)
    x0, x1 = float(xs[ix]), float(xs[ix + 1])
    y0, y1 = float(ys[iy]), float(ys[iy + 1])
    fx = (x - x0) / (x1 - x0)
    fy = (y - y0) / (y1 - y0)

    w00 = (1.0 - fx) * (1.0 - fy)
    w10 = fx * (1.0 - fy)
    w01 = (1.0 - fx) * fy
    w11 = fx * fy

    def _blend(grid: np.ndarray) -> float:
        return float(w00 * grid[iy][ix] + w10 * grid[iy][ix + 1]
                     + w01 * grid[iy + 1][ix] + w11 * grid[iy + 1][ix + 1])

    return (_blend(toss_cal.rx_rad), _blend(toss_cal.ry_rad))


def clamp_total_aim(rx: float, ry: float) -> Tuple[float, float, List[str]]:
    """Re-clamp the TOTAL commanded aim at APPLY. Returns ``(rx, ry, hits)``.

    The magnitude is clamped to :data:`TOTAL_MAX_RAD`, preserving direction — a
    per-axis clamp would rotate the aim, which is worse than shortening it.
    ``hits`` is the list of clamp names that bound (``['total_aim']`` or empty),
    recorded per toss in ``clamp_hits``.

    In phase 2b this is provably a no-op: parse time already bounds every node's
    magnitude and a bilinear blend of bounded vectors is a convex combination,
    hence bounded. It exists here anyway because D7 requires the total to be
    re-clamped **at apply, not only at update**, and phase 2e's session trim
    adds to this number — one enforcement point, landed with the seam it
    protects rather than bolted on beside it later.

    A non-finite input is refused rather than laundered: :func:`lookup` cannot
    produce one, so it would mean a caller invented an aim.
    """
    x, y = float(rx), float(ry)
    if not (math.isfinite(x) and math.isfinite(y)):
        raise TossCalError(
            f"commanded aim must be finite, got (rx={rx!r}, ry={ry!r})")
    mag = math.hypot(x, y)
    if mag <= TOTAL_MAX_RAD:
        return (x, y, [])
    scale = TOTAL_MAX_RAD / mag
    return (x * scale, y * scale, ['total_aim'])
