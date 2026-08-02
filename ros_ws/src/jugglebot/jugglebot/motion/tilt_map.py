"""Pose-dependent residual tilt map — the data half of contract C-LEVEL-2.

Normative document: ``ros_ws/docs/levelling_frame.md`` (contract **C-LEVEL-2**,
which *composes with* and never replaces C-LEVEL-1). Read it before adding a
caller: **where** this map may be evaluated is the load-bearing half of the
contract and is not inferable from these functions. In particular a per-knot or
emitter-side evaluation is forbidden — the map is evaluated **once per external
pose ingest**, keyed on the *uncorrected intent* pose.

**What a residual is.** The ``level`` routine measures the platform's tilt
against gravity at exactly one pose (the firmware ACTIVATE pose) and applies
that single offset everywhere. Any *pose-dependent* kinematic error is invisible
to it by construction. A residual is what is left over: with a fresh ``level``
correction loaded and **no map loaded**, the platform is commanded to the level
orientation at grid node ``(x, y, z_grid)`` and the inclinometer is read —
``residual = mean(raw_reading) + radians(inclinometer_offset_deg)``, the exact
``LevellingHandler`` formula. At the home node the residual is ≈ 0 by
construction.

**How it is applied.** ``combined = level_offset + bilinear(map, intent_xy)``,
fed to the *existing* single Rodrigues in :mod:`jugglebot.motion.levelling`
(:func:`~jugglebot.motion.levelling.correction_for_pose`), which keeps the
number of rotations composed onto a commanded pose at one. Additive
rotation-vector composition differs from the exact ``R(map) @ R(level)`` by
exactly ``|level_offset × residual| / 2``: **7.19e-5 rad at the measured
envelope** (0.78185° offset × 0.604° residual, orthogonal) but **1.523e-4 rad at
1° × 1°**, so the bound is stated with its regime in
``ros_ws/docs/levelling_frame.md`` § C-LEVEL-2 and is *not* a blanket
"sub-degree" guarantee. Nothing here enforces it — ``MAX_ABS_RESIDUAL_RAD``
alone permits 1.25e-3 rad.

**Outside the calibrated hull the query is clamped, never extrapolated.** A
wrong-signed edge extrapolation aims a throw *worse* than no map at all, and a
NaN would ride silently into the commanded rotation.

**Schema v1** (machine-written by the Phase-3 acquisition tool into the
committed ``config/tilt_calibration.yaml``)::

    version: 1
    captured:            # provenance, passed through to TiltMap.metadata
      date: 2026-08-02
      git_sha: ...
      tool: ...
      uptime_ms_first: ...
      uptime_ms_last: ...
      level_offset_rad: [.., ..]
      base_condition: "free text"
    grid:
      z_mm: 170.0
      orientation: level
      x_mm: [-150.0, ..., 150.0]     # strictly increasing, >= 2 nodes
      y_mm: [-150.0, ..., 150.0]     # strictly increasing, >= 2 nodes
    residual_rad:
      tx: [[...], ...]               # indexed [iy][ix]
      ty: [[...], ...]               # indexed [iy][ix]
    stats:                           # optional, advisory only
      ...

**Validation is all-or-nothing and loud** (:func:`parse_tilt_map`): a document
that fails any check raises :class:`TiltMapError` and **no** map is loaded, so
the machine degrades to exactly C-LEVEL-1 behaviour. There is deliberately no
partial load, no per-node repair and no silent zero-fill — a half-trusted map is
indistinguishable at the machine from a correct one until a ball misses.

**Path resolution** (:func:`resolve_tilt_map_path`) deliberately **inverts**
``motion/friction_ff_params.py``'s order. That loader resolves
``ament share → source tree`` because the friction constants are hand-tuned into
the committed YAML and a ``colcon build`` is the natural publish step. This one
resolves ``source tree → ament share``, because the Phase-3 acquisition tool
*rewrites the source-tree file at runtime* and then asks the running node to
reload it. Share-first would serve the previous build's stale calibration until
the next ``colcon build`` — silently, with ``tilt_map_loaded`` still true and a
``tilt_map_version`` that looks perfectly plausible. That is the exact trap
``friction_ff_params.py:24-33`` documents, and the acquisition workflow walks
into it every run.

Pure Python — no ROS imports (consumed by ROS nodes, and by ``tests/motion``
with no ROS mocking at all). ``ament_index_python`` is imported lazily and
defensively inside the resolver for the same reason it is there: the pytest venv
has no ROS 2 sourced.

Nothing in this module reads the map's *location* policy into its *content*
policy: :func:`resolve_tilt_map_path` answers "which file", and absence of every
candidate is **not** an error here — C-LEVEL-2 makes an absent map silent (soft
absence, non-gating) and only an *invalid* one loud. The resolver therefore
returns ``None`` rather than raising, unlike ``friction_ff_params``' fail-fast
``FileNotFoundError``.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Tuple

import numpy as np
import yaml

__all__ = [
    'SCHEMA_VERSION',
    'MAX_ABS_RESIDUAL_RAD',
    'TILT_MAP_ENV',
    'TiltMapError',
    'TiltMap',
    'parse_tilt_map',
    'load_tilt_map',
    'lookup',
    'map_version',
    'tilt_map_candidates',
    'resolve_tilt_map_path',
]

#: The only schema version this build understands. A document carrying anything
#: else is refused rather than best-effort parsed: the fields a future version
#: adds may change what the numbers *mean*, and applying them anyway is a
#: silent frame error.
SCHEMA_VERSION = 1

#: Sanity bound on any single residual node, radians (0.05 rad ≈ 2.87°). The
#: measured pose-dependent error is 0.04–0.6° (2026-07-28 extremity table), so a
#: node beyond this is a capture fault (stale level reference, a leg pinned on
#: its stroke clamp, a units error), not a calibration.
MAX_ABS_RESIDUAL_RAD = 0.05

#: Explicit-override env var. When set it is **authoritative** — see
#: :func:`tilt_map_candidates` for why it does not fall through.
TILT_MAP_ENV = 'JUGGLEBOT_TILT_CAL'

def _find_repo_root(start: str) -> Optional[str]:
    """Walk up from ``start`` to the Jugglebot repo root, or ``None``.

    **Deliberately a search, not a fixed ``__file__`` walk.**
    ``friction_ff_params.py`` uses a fixed five-level walk, which is correct
    only from the source checkout: ``colcon`` *copies* the package into
    ``ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot/motion/``,
    where the same five levels land on ``ros_ws/install/jugglebot`` — and
    ``setup.py`` installs config into ``share/jugglebot/config/``, so
    ``install/jugglebot/config/`` is a directory nothing ever creates.

    A fixed walk here would therefore make the source-tree candidate **unable to
    exist in production**, and the resolver would fall through to the ament share
    copy on every load — silently restoring exactly the stale-calibration trap
    this module inverted the resolution order to avoid: the acquisition tool
    rewrites the source file, calls reload, and the node re-loads the *previous
    build's* numbers while reporting ``tilt_map_loaded=true`` and a plausible
    version. (``friction_ff_params.py:24-33`` records that same defect as a
    shipped bug — the ``motor_guard`` startup crash of 2026-06-24 — reached from
    the other direction.)

    The marker is ``config/hardware_config.yaml`` beside a ``ros_ws/`` directory:
    both are committed, both are at the repo root, and neither exists at any
    other level of either tree. Returns ``None`` for a genuinely detached
    deployment (an install tree outside the repo), where the share copy IS the
    right answer and ``$JUGGLEBOT_TILT_CAL`` is the escape hatch.
    """
    current = os.path.dirname(os.path.abspath(start))
    while True:
        if (os.path.isdir(os.path.join(current, 'ros_ws'))
                and os.path.exists(os.path.join(current, 'config',
                                                'hardware_config.yaml'))):
            return current
        parent = os.path.dirname(current)
        if parent == current:
            return None
        current = parent


#: Source-tree YAML — the file the Phase-3 acquisition tool writes and the file
#: that is committed. ``None`` when this module is running from a deployment
#: outside the repo (see :func:`_find_repo_root`).
_REPO_ROOT = _find_repo_root(__file__)
_SRC_TILT_YAML = (os.path.join(_REPO_ROOT, 'config', 'tilt_calibration.yaml')
                  if _REPO_ROOT else None)


class TiltMapError(ValueError):
    """Any tilt-map schema, validation or lookup failure.

    Subclasses ``ValueError`` so a caller that already funnels malformed config
    through ``except ValueError`` keeps working, while a loader that wants to
    distinguish "this file is not a tilt map" from every other ``ValueError``
    can catch this type. C-LEVEL-2 is non-gating: the loader's obligation on
    catching this is to **not load a map** and log loudly, never to block.
    """


@dataclass(frozen=True)
class TiltMap:
    """A validated residual tilt map over an (x, y) grid at one capture height.

    ``tx_rad`` / ``ty_rad`` are indexed ``[iy][ix]`` — row index selects
    ``y_mm``, column index selects ``x_mm`` — matching the YAML nesting order
    and ``numpy``'s row-major convention. Getting this transposed is the
    obvious mistake and is invisible on a symmetric grid, which is why the
    unit tests use a deliberately **asymmetric** map.

    All four arrays are **copied and marked read-only** at construction. Copied
    because a caller that hands in its own working arrays (the Phase-3
    acquisition tool self-validating before it writes the YAML) must not find
    them frozen for the rest of its run. Read-only because the map is a shared,
    per-process single reference that every pose ingest reads, so one in-place
    write would silently re-frame every subsequent commanded pose.

    **The structural invariants live here, not only in :func:`parse_tilt_map`.**
    ``lookup`` divides by a cell width, so a hand-built map with a 1-node axis
    would raise ``ZeroDivisionError`` — an exception type no caller is told to
    expect — instead of the ``TiltMapError`` every other failure in this module
    raises. Enforcing them in ``__post_init__`` means the invariant travels with
    the type, whatever route built it.
    """

    x_mm: np.ndarray
    y_mm: np.ndarray
    tx_rad: np.ndarray
    ty_rad: np.ndarray
    version: str
    z_mm: Optional[float] = None   # capture height; provenance only, never keyed on
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        arrays = {}
        for name in ('x_mm', 'y_mm', 'tx_rad', 'ty_rad'):
            try:
                array = np.array(getattr(self, name), dtype=float)
            except (TypeError, ValueError) as exc:
                raise TiltMapError(
                    f"TiltMap.{name} is not a numeric array: {exc}") from exc
            array.flags.writeable = False
            arrays[name] = array
            object.__setattr__(self, name, array)

        for name in ('x_mm', 'y_mm'):
            axis = arrays[name]
            if axis.ndim != 1 or axis.size < 2:
                raise TiltMapError(
                    f"TiltMap.{name} must be 1-D with at least 2 nodes, got "
                    f"shape {axis.shape} — build via parse_tilt_map()")
            if not np.all(np.diff(axis) > 0.0):
                raise TiltMapError(
                    f"TiltMap.{name} must be strictly increasing — a repeated "
                    f"node is a zero-width cell")

        expected = (int(arrays['y_mm'].size), int(arrays['x_mm'].size))
        for name in ('tx_rad', 'ty_rad'):
            if arrays[name].shape != expected:
                raise TiltMapError(
                    f"TiltMap.{name} must be indexed [iy][ix] with shape "
                    f"{expected}, got {arrays[name].shape}")

    @property
    def shape(self) -> Tuple[int, int]:
        """``(n_y, n_x)`` — the residual grids' shape."""
        return (int(self.y_mm.size), int(self.x_mm.size))


# ── validation helpers ───────────────────────────────────────────


def _mapping(value: Any, what: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise TiltMapError(
            f"{what} must be a mapping, got {type(value).__name__}")
    return value


def _require(data: Dict[str, Any], key: str, what: str) -> Any:
    if key not in data:
        raise TiltMapError(f"tilt map is missing {what}")
    return data[key]


def _axis(raw: Any, name: str) -> np.ndarray:
    """Validate one grid axis: 1-D, >= 2 nodes, finite, strictly increasing."""
    try:
        axis = np.asarray(raw, dtype=float)
    except (TypeError, ValueError) as exc:
        raise TiltMapError(
            f"grid.{name} is not a numeric array: {exc}") from exc
    if axis.ndim != 1:
        raise TiltMapError(
            f"grid.{name} must be a 1-D list, got shape {axis.shape}")
    if axis.size < 2:
        # A single-node axis has no cell to interpolate across; every query
        # would clamp onto that node, which is a constant offset masquerading
        # as a map. Refuse it rather than silently degrade to C-LEVEL-1+bias.
        raise TiltMapError(
            f"grid.{name} needs at least 2 nodes to interpolate, got "
            f"{axis.size}")
    if not np.all(np.isfinite(axis)):
        raise TiltMapError(
            f"grid.{name} contains non-finite values: {axis.tolist()}")
    if not np.all(np.diff(axis) > 0.0):
        raise TiltMapError(
            f"grid.{name} must be strictly increasing (ascending, no repeats), "
            f"got {axis.tolist()}")
    return axis


def _residual_grid(raw: Any, name: str, n_y: int, n_x: int) -> np.ndarray:
    """Validate one residual grid: shape ``(n_y, n_x)``, finite, in bounds."""
    try:
        grid = np.asarray(raw, dtype=float)
    except (TypeError, ValueError) as exc:
        raise TiltMapError(
            f"residual_rad.{name} is not a numeric 2-D array (ragged rows or a "
            f"non-numeric entry?): {exc}") from exc
    if grid.shape != (n_y, n_x):
        raise TiltMapError(
            f"residual_rad.{name} must be indexed [iy][ix] with shape "
            f"(len(grid.y_mm), len(grid.x_mm)) = ({n_y}, {n_x}), got "
            f"{grid.shape}")
    if not np.all(np.isfinite(grid)):
        bad = [(int(iy), int(ix))
               for iy, ix in zip(*np.nonzero(~np.isfinite(grid)))]
        raise TiltMapError(
            f"residual_rad.{name} contains non-finite values at [iy][ix] "
            f"{bad} — a failed capture node must be resolved by the "
            f"acquisition tool, never shipped as NaN")
    peak = float(np.max(np.abs(grid))) if grid.size else 0.0
    if peak > MAX_ABS_RESIDUAL_RAD:
        raise TiltMapError(
            f"residual_rad.{name} peak |residual| {peak:.6f} rad exceeds the "
            f"{MAX_ABS_RESIDUAL_RAD} rad sanity bound "
            f"({math.degrees(MAX_ABS_RESIDUAL_RAD):.2f}°) — a residual that "
            f"large is a capture fault, not a calibration")
    return grid


# ── parse / load ─────────────────────────────────────────────────


def _numeric(value: Any) -> Any:
    """Recursively coerce numbers to float for hashing; pass anything else through.

    ``170`` and ``170.0`` are the same calibration written by two serializers.
    Non-numeric values are returned untouched so a malformed document still
    hashes deterministically (``map_version`` is callable before validation).
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, (list, tuple)):
        return [_numeric(item) for item in value]
    return value


def map_version(data: Dict[str, Any]) -> str:
    """``"<captured.date>-<sha256(content)[:8]>"`` — the observable map identity.

    Published on ``TrajectoryStatus.tilt_map_version`` (C-LEVEL-2's
    observability half, landed in Phase 2) so an operator can tell, from a
    running machine, *which* calibration is applied.

    The hash covers the **applied numbers only** — the schema ``version``, the
    two grid **axes**, and the ``residual_rad`` grids. Everything else is
    provenance and is deliberately excluded, including the two provenance fields
    that live *inside* the ``grid`` block: ``z_mm`` (carried for context, never
    keyed on) and ``orientation`` (free text). Two files whose applied numbers
    are identical must report the same version; an edit to a single node must
    change it.

    Numbers are **normalised to float before hashing**, so YAML's `170` and
    `170.0` — which `json.dumps` would otherwise render as different strings —
    do not churn the version. That churn is not hypothetical: the Phase-3 tool
    re-emits this file, and a version that changes when nothing was recalibrated
    sends an operator hunting a difference that does not exist.

    The date prefix makes the string readable at a glance; the digest is what
    makes it *identifying*.
    """
    if not isinstance(data, dict):
        data = {}
    captured = data.get('captured')
    date = ''
    if isinstance(captured, dict):
        # yaml.safe_load turns a bare `date: 2026-08-02` into datetime.date.
        date = str(captured.get('date', '')).strip()

    grid = data.get('grid') if isinstance(data.get('grid'), dict) else {}
    residual = (data.get('residual_rad')
                if isinstance(data.get('residual_rad'), dict) else {})
    payload = json.dumps(
        {'version': data.get('version'),
         'x_mm': _numeric(grid.get('x_mm')),
         'y_mm': _numeric(grid.get('y_mm')),
         'tx': _numeric(residual.get('tx')),
         'ty': _numeric(residual.get('ty'))},
        sort_keys=True, default=str)
    digest = hashlib.sha256(payload.encode('utf-8')).hexdigest()[:8]
    return f"{date or 'undated'}-{digest}"


def parse_tilt_map(data: Dict[str, Any]) -> TiltMap:
    """Validate a schema-v1 tilt-map document and build the :class:`TiltMap`.

    All-or-nothing: every failure mode raises :class:`TiltMapError` with a
    message naming the offending field, and nothing partial is returned. See
    the module docstring for the schema and ``ros_ws/docs/levelling_frame.md``
    § C-LEVEL-2 for why the validation set is exactly this one.
    """
    doc = _mapping(data, 'tilt map document')

    raw_version = _require(doc, 'version', "the top-level 'version' key")
    if isinstance(raw_version, bool) or not isinstance(raw_version, int) \
            or raw_version != SCHEMA_VERSION:
        raise TiltMapError(
            f"unsupported tilt map schema version {raw_version!r}; this build "
            f"understands version {SCHEMA_VERSION} only")

    grid = _mapping(_require(doc, 'grid', "the 'grid' block"), "the 'grid' block")
    x_mm = _axis(_require(grid, 'x_mm', "grid.x_mm"), 'x_mm')
    y_mm = _axis(_require(grid, 'y_mm', "grid.y_mm"), 'y_mm')

    z_mm = None
    if grid.get('z_mm') is not None:
        try:
            z_mm = float(grid['z_mm'])
        except (TypeError, ValueError) as exc:
            raise TiltMapError(f"grid.z_mm is not a number: {exc}") from exc
        if not math.isfinite(z_mm):
            raise TiltMapError(f"grid.z_mm is not finite: {z_mm}")

    residual = _mapping(_require(doc, 'residual_rad', "the 'residual_rad' block"),
                        "the 'residual_rad' block")
    n_y, n_x = int(y_mm.size), int(x_mm.size)
    tx = _residual_grid(_require(residual, 'tx', "residual_rad.tx"),
                        'tx', n_y, n_x)
    ty = _residual_grid(_require(residual, 'ty', "residual_rad.ty"),
                        'ty', n_y, n_x)

    metadata = doc.get('captured', {})
    if metadata is None:
        metadata = {}
    metadata = dict(_mapping(metadata, "the 'captured' block"))

    # TiltMap.__post_init__ copies and freezes the four arrays — deliberately
    # not done here, so a caller's own arrays are never frozen behind its back.
    return TiltMap(x_mm=x_mm, y_mm=y_mm, tx_rad=tx, ty_rad=ty,
                   version=map_version(doc), z_mm=z_mm, metadata=metadata)


def load_tilt_map(path) -> TiltMap:
    """Read + validate a tilt-map YAML file. Every failure is a ``TiltMapError``.

    Unreadable, non-YAML, empty and schema-invalid files all raise the same
    type: the caller's contract is identical in every case — do not load a map,
    log loudly, keep running under C-LEVEL-1. An **absent** file is not this
    function's business; the loader decides that absence is silent (soft
    absence, non-gating) before calling here.
    """
    try:
        with open(path, 'r') as handle:
            data = yaml.safe_load(handle)
    except OSError as exc:
        raise TiltMapError(f"cannot read tilt map {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise TiltMapError(f"tilt map {path} is not valid YAML: {exc}") from exc
    if data is None:
        raise TiltMapError(f"tilt map {path} is empty")
    try:
        return parse_tilt_map(data)
    except TiltMapError as exc:
        raise TiltMapError(f"{path}: {exc}") from exc


# ── path resolution ──────────────────────────────────────────────


def tilt_map_candidates(
        environ: Optional[Mapping[str, str]] = None) -> Tuple[str, ...]:
    """The candidate paths for ``config/tilt_calibration.yaml``, in order.

    ``$JUGGLEBOT_TILT_CAL`` → **repo source tree** → ament share dir. The
    inversion of ``friction_ff_params.py``'s order is the point; the module
    docstring says why.

    **The env override is authoritative: when it is set it is the ONLY
    candidate**, existing or not. Falling through to the source tree would load
    a *different calibration than the operator named* while reporting
    ``tilt_map_loaded=True`` and a plausible-looking ``tilt_map_version`` — the
    same silent-wrong-map failure the share-first order was rejected for, and
    strictly worse than loading nothing (C-LEVEL-2 is non-gating, so loading
    nothing costs only precision). A typo'd override therefore degrades to
    offset-only, loudly, instead of quietly applying the committed map.

    The ament share dir is appended only when ``ament_index_python`` is
    importable AND knows the package — under the pytest venv (no ROS 2 sourced)
    it simply does not appear, which is why callers must treat the returned
    tuple as the whole truth for logging rather than assuming three entries.
    The source-tree entry is likewise absent for a deployment outside the repo.
    So this can return one, two or three entries and callers must not index it.
    """
    env = (environ if environ is not None else os.environ).get(TILT_MAP_ENV)
    if env:
        return (env,)

    candidates = [_SRC_TILT_YAML] if _SRC_TILT_YAML else []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory('jugglebot'),
            'config', 'tilt_calibration.yaml'))
    except Exception:
        # ament_index not importable (the pytest venv without ROS 2 sourced) or
        # the package is not on the ament index — the source tree is the answer.
        pass
    return tuple(candidates)


def resolve_tilt_map_path(
        environ: Optional[Mapping[str, str]] = None) -> Optional[str]:
    """First existing entry of :func:`tilt_map_candidates`, or ``None``.

    ``None`` means **no calibration file exists**, which C-LEVEL-2 defines as a
    silent, non-gating state — so this returns rather than raises. Contrast
    ``friction_ff_params._resolve_yaml_path``, which fail-fasts with a
    ``FileNotFoundError``: its YAML is a hard runtime dependency, this one is a
    refinement.
    """
    for candidate in tilt_map_candidates(environ):
        if os.path.exists(candidate):
            return candidate
    return None


# ── lookup ───────────────────────────────────────────────────────


def _cell_index(axis: np.ndarray, value: float) -> int:
    """Index of the cell's lower node, clamped so ``idx + 1`` is always valid."""
    idx = int(np.searchsorted(axis, value, side='right')) - 1
    return min(max(idx, 0), int(axis.size) - 2)


def lookup(tilt_map: TiltMap, x_mm: float, y_mm: float) -> Tuple[float, float]:
    """Bilinear residual ``(tx, ty)`` in radians at ``(x_mm, y_mm)``.

    The query is **clamped into the calibrated hull first**, then interpolated —
    so a pose outside the grid gets the nearest hull point's residual and the
    map never extrapolates. Exact at every node (the interpolation weights
    collapse to 0.0/1.0 there, so a node query returns its stored value
    bit-identically).

    ``x_mm`` / ``y_mm`` are the **uncorrected intent** pose's position
    components, in mm, in the same frame the grid was captured in. A non-finite
    query raises :class:`TiltMapError` rather than returning NaN: NaN here would
    be negated straight into the commanded rotation and surface only downstream
    as a feasibility rejection, after a goal has already claimed the platform.
    """
    x = float(x_mm)
    y = float(y_mm)
    if not (math.isfinite(x) and math.isfinite(y)):
        raise TiltMapError(
            f"tilt map query must be finite, got (x={x_mm!r}, y={y_mm!r})")

    xs = tilt_map.x_mm
    ys = tilt_map.y_mm
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

    return (_blend(tilt_map.tx_rad), _blend(tilt_map.ty_rad))
