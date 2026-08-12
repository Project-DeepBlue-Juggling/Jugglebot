"""Pure fit core for the toss AIM calibration map — contract **C-TOSS-CAL-1**.

Normative design: ``plans/active/toss-selftuning.md`` §§ 3.6.2, 3.7 and 3.8, build
phase **2c**. This module is the hardware-free half of
``tests/hardware/toss_cal_fit.py`` (a thin CLI over it) and is imported unchanged
by ``tools/toss_cal_analyse.py`` and by ``tests/motion/test_toss_cal_fit.py``.
Nothing here opens a socket, launches a node, or imports ``rclpy`` — it reads a
JSONL corpus and returns numbers, so the whole fit runs in the ordinary pytest
suite.

WHAT THIS MODULE OWNS
---------------------
1. **The partition rule** (§ 3.7). A corpus is partitioned by
   :data:`PARTITION_KEYS` and a fit across two partitions is REFUSED without an
   explicit flag; the census is stamped into ``captured.partition_census``. This
   is what makes *"the 07-16 premises were measured on a degraded plant"*
   structural rather than remembered.
2. **The reduction** — one toss becomes one commanded-aim estimate
   (:func:`reduce_to_aim`), via the **production forward model**, not a
   re-derived one.
3. **The admission filter** — guards G1/G2/G3/G9 and the label-inclusion rule
   (§ 3.7 item 4), as far as one record can decide them.
4. **The gates that refuse a write** — flat field and ball-actually-flew (§ 3.8).
5. **The document** — home-referenced grid + anchor prior, validated through the
   production loader (:mod:`jugglebot.motion.toss_cal`) before it is written.

THE SIGN, AND THE ONE PLACE IT LIVES
------------------------------------
The map stores the **commanded aim** — the tilt the machine applies at release
(``toss_cal``'s module docstring). The corpus stores ``land_err_mm`` =
``land_xy − catch_point_global_mm[:2]`` (``toss_record.FIELDS``). Turning the
second into the first is this module's job, and it is done by inverting the
**production** aim→landing map:

    ``J(T, z) = d(aim_target_offset_mm) / d(rx, ry)``   evaluated at zero aim

:func:`aim_landing_jacobian` computes ``J`` by finite-differencing
``toss_release.aim_target_offset_mm`` — the very function
``reload_coordinator_node._toss_aim_for_goal`` uses to apply the map. There is
therefore **no hand-written sign anywhere in the fit**: if the apply path's
convention ever changed, the fit's would change with it in the same direction,
which is the only structural defence against the failure mode rung SC-0 exists to
catch (a sign flip aims the machine roughly twice as badly as no map).

That ``J`` is emphatically **not** a scaled identity. At h = 0.78 m::

    J = [[    0.0,  3126.5],
         [-3126.5,     0.0]]  mm/rad

i.e. ``S = [[0, 1], [-1, 0]]``, a 90° rotation: a tilt about **+rx** moves the
ball in **−y** and a tilt about **+ry** moves it in **+x** (right-hand rule on
the cup axis). An identity ``S`` — the intuitive guess — would correct an x
error with an x tilt and push the ball sideways instead. The design's insistence
that ``S`` be *measured* by SC-0 rather than assumed is vindicated by the
production geometry before a single ball flies.

**A deliberate deviation from the design text, with its root cause.** § 3.7
item 3 writes the reduction as

    ``b_i = map_aim_i + trim_aim_i + S⁻¹·land_err_i/(4·h_i)``   (a PLUS)

That sign does not close. With applied aim ``A`` and plant bias ``ψ`` the
landing error is ``land_err = J·(A + ψ)``, so ``J⁻¹·land_err = A + ψ`` and the
design's expression evaluates to ``2A + ψ`` — at ``A = 0`` it returns the plant
bias *uncancelled*, and commanding that as the next map DOUBLES the error and
diverges on every subsequent capture. The fixed point is the MINUS:

    ``b_i = A_i − J⁻¹·land_err_i``  ⇒  ``b = A − (A + ψ) = −ψ``

independent of ``A``, which is exactly the "captures do not have to run with the
map uninstalled" property § 3.7 item 3 is arguing for. :func:`reduce_to_aim`
implements the minus; ``tests/motion/test_toss_cal_fit.py``'s closed-loop test
injects a known bias into a replayed corpus, runs the REAL fit, and asserts sign
**and** magnitude come back — the strongest-test shape borrowed from
``tests/motion/test_tilt_cal_grid.py``.

The design's ``4·h`` gain is kept as a documented cross-check rather than as the
arithmetic: ``J``'s magnitude is 3126.5 mm/rad at h = 0.78 against ``4h`` =
3120.0, a 0.21 % difference carried by the ``Δz`` and drop terms the exact form
keeps. A test pins the two together at 1 %.

HOME-REFERENCING (§ 3.2), AND WHY THE ANCHOR IS NOT A CORRECTION
-----------------------------------------------------------------
``level`` is one int16-quantised SCL3300 sample with 1.2–1.7 mrad/axis of
session-to-session scatter, so an *absolute* aim map bakes one session's level
noise into every future session. The shipped grid is therefore
``M(P) = b(P) − mean(b(home anchors))`` — the stable mechanical part — and the
absolute home residual ships separately as ``anchor.aim_rad``, the **warm-start
prior for phase 2e's session trim**, never as a hard correction. With the trim
disabled (its default, and mandatory during a capture, preflight R4) the machine
applies only the home-referenced field, so the common mode stays uncorrected on
purpose: it is the part that is not a constant of the machine.

Consequence worth stating because it is easy to misread as a bug: on a
single-node corpus the shipped grid is all zeros by construction, and the whole
of the measured bias lives in ``anchor.aim_rad``. That is a correct SC-0/SC-1
capture, and the flat-field guard is what stops it being mistaken for a grid.
"""

from __future__ import annotations

import hashlib
import math
import os
from datetime import datetime
from typing import Any, Dict, Iterable, List, NamedTuple, Optional, Sequence, Tuple

import numpy as np
import yaml

import jugglebot.hardware_config as hw
from jugglebot import toss_record, toss_trim
from jugglebot.motion import toss_cal
from jugglebot.motion.trajectory.toss_release import aim_target_offset_mm
# The reduction, the Jacobian and the three admission filters MOVED into the
# production package in phase 2e (``jugglebot/toss_trim.py``) and are imported —
# never re-implemented — here. Root cause: the ONLINE session trim and this
# OFFLINE fit must admit and reduce a toss identically, or the map an operator
# promotes is fitted on a different population from the one the machine learnt
# on, and nothing would ever surface the difference. Same argument D11 makes for
# the labeller. This module keeps its own names so every caller, CLI and test is
# unchanged by the move.
from jugglebot.toss_trim import (          # noqa: F401  (re-exported)
    AIM_LABELS,
    APEX_SANITY_FRAC,
    FIT_RMS_MAX_MM,
    SPEED_SE_MAX,
    SPEED_TIMING_N_MIN,
    TAU_SE_MAX_MS,
    TIMING_POLL_DT_MS_MAX,
    TIMING_POLL_DT_MS_MIN,
    achieved_apex_m,
    achieved_flight_s,
    admit_for_aim,
    admit_for_speed,
    admit_for_timing,
    aim_landing_jacobian,
    applied_aim_rad,
    reduce_to_aim,
)
from jugglebot.toss_trim import _G, _is_pair, _num, _z_of   # noqa: F401

__all__ = [
    'TossFitError',
    'N_MIN', 'TRIM_FRACTION', 'THETA_ACC_RAD', 'NODE_SNAP_TOL_MM',
    'FIT_RMS_MAX_MM', 'APEX_SANITY_FRAC', 'AIM_LABELS', 'PARTITION_KEYS',
    'TIMING_POLL_DT_MS_MIN', 'TIMING_POLL_DT_MS_MAX', 'TAU_SE_MAX_MS',
    'SPEED_SE_MAX', 'TOOL_NAME',
    'load_corpus', 'partition_key', 'partition_census', 'census_lines',
    'select_partition',
    'aim_landing_jacobian', 'achieved_flight_s', 'achieved_apex_m',
    'applied_aim_rad', 'reduce_to_aim',
    'admit_for_aim', 'admit_for_timing', 'admit_for_speed',
    'home_index', 'node_of', 'NodeFit', 'fit_nodes', 'trimmed_mean',
    'AnchorVisit', 'anchor_visits', 'anchor_estimate',
    'flat_field_verdict', 'ball_flew_verdict',
    'sigma_land_mm', 'r_eff_mm', 'speed_fit', 'timing_fit', 'score_groups',
    'build_map_document', 'validate_map_document', 'dump_map_yaml',
    'diff_documents', 'write_target_path', 'sha256_of', 'synthetic_corpus',
    'default_estimator_block',
]


#: Any refusal to fit or to write. Carries the operator-facing reason.
#:
#: An **alias** of :class:`jugglebot.toss_trim.TossTrimError`, not a subclass:
#: the reduction and the admission filters moved into the production package in
#: phase 2e, and every ``except TossFitError`` here must keep catching what those
#: moved functions raise. A subclass would silently stop catching them — the
#: quietest possible way to turn a named refusal into a traceback out of a fit.
TossFitError = toss_trim.TossTrimError


TOOL_NAME = 'tests/hardware/toss_cal_fit.py'

# ── Thresholds ───────────────────────────────────────────────────────────────

#: Minimum admitted tosses for a node to be RE-FITTED (§ 3.7 item 2).
#: **PROVISIONAL** until the first corpus, in the convention
#: ``tests/hardware/tilt_cal_grid.py`` already uses. Design basis (F4): detecting
#: a bias ``β`` through the arrival offset needs ``n ≥ (2σ/β)²``, which at
#: σ = 20 mm is 16 tosses for β = 10 mm and 8 for β = 15 mm. 8 buys a ~15 mm
#: resolution per node, which is 43 % of the 35 mm capture radius — enough to see
#: a residual worth correcting, not enough to chase noise.
N_MIN = 8

#: Fraction trimmed from EACH tail before the per-node mean (§ 3.7 item 3).
#: A plain mean is dragged by a single rimshot or mis-fitted track; a median
#: throws away half the information at n = 8. The 10 % trim drops nothing at
#: n < 10 and one sample per tail from n = 10, which is the right behaviour at
#: this sample size.
TRIM_FRACTION = 0.10

#: θ_acc — the tilt map's own accuracy floor, 0.15° (§ 3.6 constants table).
#: Used by the flat-field guard: a map whose largest node is smaller than the
#: layer below it cannot be distinguished from that layer's noise.
THETA_ACC_RAD = math.radians(0.15)

#: How close a record's nominated catch xy must be to a grid node to COUNT as
#: that node. The capture tool commands node coordinates verbatim and the
#: declaration carries the same float, so a real node record matches to ~1e-12 mm
#: and 1.0 mm is pure headroom for JSON float round-tripping. It is deliberately
#: NOT generous: SC-3's verification poses are chosen OFF-node precisely so they
#: are an independent check, and snapping one into the nearest node would launder
#: the check into the fit it exists to test.
NODE_SNAP_TOL_MM = 1.0

#: The mandatory partition keys (§ 3.7). ``z_mm`` is derived from
#: ``goal_catch_xyz_stow_mm[2]``; every other entry is a record field.
PARTITION_KEYS: Tuple[str, ...] = (
    'tilt_map_version', 'bridge_fw_version', 'platform_fw_version',
    'toss_tier', 'z_mm', 'hand_odrive_config_sha',
)

# The re-derived timing gate (TIMING_POLL_DT_MS_MIN / _MAX / TAU_SE_MAX_MS /
# SPEED_SE_MAX / SPEED_TIMING_N_MIN) and the reduction's _G moved to
# jugglebot/toss_trim.py in phase 2e together with the admission filters that
# read them — see the import block above. They are re-exported unchanged.


# ── Corpus ───────────────────────────────────────────────────────────────────


def load_corpus(paths: Iterable[str]) -> List[Dict[str, Any]]:
    """Read one or more ``toss_record/1`` JSONL files into a list of records.

    Rows are decoded through the **production** :func:`toss_record.decode`, so a
    field this build does not know about is dropped exactly as every other
    consumer drops it, and a missing field arrives as ``None`` rather than as a
    ``KeyError`` three call frames later. Each row is stamped with
    ``_source_file`` (leading underscore: not a schema field, never encoded) so a
    refusal can name the file it came from.

    A row that fails to decode is REPORTED and skipped rather than aborting the
    load: a corpus is an instrument, and losing a sitting because one line was
    truncated by a killed writer is a worse failure than fitting the other 128.
    """
    out: List[Dict[str, Any]] = []
    for path in paths:
        with open(path, 'r') as handle:
            for lineno, line in enumerate(handle, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = toss_record.decode(line)
                except (ValueError, TypeError) as exc:
                    print('  SKIP {}:{}: {}'.format(path, lineno, exc))
                    continue
                rec['_source_file'] = path
                out.append(rec)
    return out


def sha256_of(path: str) -> str:
    """SHA-256 of a corpus file, for ``captured.source_files``.

    Every map write records its source list with per-file digests so a bad map is
    attributable and rollback-able node by node (§ 7 R3).
    """
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for chunk in iter(lambda: handle.read(65536), b''):
            digest.update(chunk)
    return digest.hexdigest()


def partition_key(rec: Dict[str, Any]) -> Tuple[Any, ...]:
    """The § 3.7 partition tuple for one record.

    A ``None`` component is preserved as ``None`` rather than collapsed onto a
    default — ``hand_odrive_config_sha`` ships nullable and § 8 is explicit that
    *"a null is a KNOWN GAP in the partition key, not an assumed match"*. Two
    records that agree on everything else and are both unknown therefore land in
    the same partition (they are equally unattributable), and
    :func:`select_partition` reports how many keys are unknown so the operator
    knows what the census cannot see.
    """
    values: List[Any] = []
    for key in PARTITION_KEYS:
        if key == 'z_mm':
            values.append(_z_of(rec))
        else:
            value = rec.get(key)
            values.append(None if value is None else str(value))
    return tuple(values)


def partition_census(records: Sequence[Dict[str, Any]]) -> Dict[Tuple[Any, ...], int]:
    """``{partition_key: n_records}`` — stamped into ``captured``."""
    census: Dict[Tuple[Any, ...], int] = {}
    for rec in records:
        key = partition_key(rec)
        census[key] = census.get(key, 0) + 1
    return census


def census_lines(census: Dict[Tuple[Any, ...], int]) -> List[str]:
    """Human-readable census, largest partition first."""
    lines = []
    for key, n in sorted(census.items(), key=lambda kv: (-kv[1], str(kv[0]))):
        parts = ['{}={}'.format(name, '?' if value is None else value)
                 for name, value in zip(PARTITION_KEYS, key)]
        lines.append('  n={:<4d} {}'.format(n, '  '.join(parts)))
    return lines


def select_partition(records: Sequence[Dict[str, Any]], *,
                     allow_cross: bool = False,
                     key: Optional[Tuple[Any, ...]] = None
                     ) -> Tuple[List[Dict[str, Any]], Dict[Tuple[Any, ...], int],
                                Optional[Tuple[Any, ...]], List[str]]:
    """Pick the partition to fit. Returns ``(rows, census, key, warnings)``.

    **Refuses a cross-partition fit** unless ``allow_cross`` is set: a corpus
    pooling a degraded and a restored plant yields a map that is the weighted
    average of two machines, and it looks *fine* — the residual mean is still a
    number and the sd only widens a little (§ 7 R3). The refusal is the whole
    point of the key list, so it raises :class:`TossFitError` naming every
    partition rather than quietly fitting the biggest one.

    With ``allow_cross`` the pooled set is returned and the census is stamped
    into ``captured`` regardless, so the crossing is recorded in the artefact
    rather than remembered by whoever ran the command.
    """
    census = partition_census(records)
    warnings: List[str] = []
    if not records:
        return [], census, None, warnings

    unknown = sum(1 for k in census for v in k if v is None)
    if unknown:
        missing = sorted({name for k in census
                          for name, v in zip(PARTITION_KEYS, k) if v is None})
        warnings.append(
            'partition keys UNKNOWN in this corpus: {} — a null key is a known '
            'gap, never an assumed match (§ 8). Records differing only in an '
            'unknown key are indistinguishable here.'.format(', '.join(missing)))

    if key is not None:
        rows = [r for r in records if partition_key(r) == key]
        if not rows:
            raise TossFitError(
                'no records in the requested partition {}'.format(key))
        return rows, census, key, warnings

    if len(census) == 1:
        only = next(iter(census))
        return list(records), census, only, warnings

    if not allow_cross:
        raise TossFitError(
            'refusing to fit across {} partitions — a map pooled over two '
            'plants is the weighted average of two machines and it LOOKS fine '
            '(the residual mean is still a number, the sd only widens a '
            'little). Census:\n{}\n'
            'Fit one partition (--partition-index N), or cross deliberately '
            'with --allow-cross-partition, which stamps the census into the '
            'map.'.format(len(census), '\n'.join(census_lines(census))))
    warnings.append(
        'CROSSING {} partitions by explicit request — the census is stamped '
        'into captured.partition_census.'.format(len(census)))
    return list(records), census, None, warnings


# ── Physics + admission: IMPORTED from jugglebot/toss_trim.py ────────────────
#
# ``aim_landing_jacobian`` / ``achieved_flight_s`` / ``achieved_apex_m`` /
# ``applied_aim_rad`` / ``reduce_to_aim`` and the three ``admit_for_*`` filters
# moved into the production package in phase 2e and are imported at the top of
# this file. The sign argument, the fixed-point derivation and the guard
# vocabulary now live in that module's docstring — read it there, once.
#
# Two things this move BUYS, both of which are the reason for it rather than
# tidiness: the online session trim and this offline fit can no longer drift
# apart about what a toss reduces to or which tosses are admissible; and
# ``reduce_to_aim``'s minus (the design text writes a plus, which diverges — see
# the 2c logbook section) exists in exactly one place for exactly one review.



# ── Grid geometry ────────────────────────────────────────────────────────────


def home_index(x_mm: Sequence[float], y_mm: Sequence[float]) -> Tuple[int, int]:
    """``(ix, iy)`` of the origin node, which MUST exist.

    Home-referencing is undefined without it (§ 3.2), and a grid that omits the
    origin would silently ship an absolute map under a home-referenced name — the
    one substitution the whole anchor-mean design exists to prevent.
    """
    try:
        ix = [round(float(v), 6) for v in x_mm].index(0.0)
        iy = [round(float(v), 6) for v in y_mm].index(0.0)
    except ValueError:
        raise TossFitError(
            'the grid must contain the origin node (0, 0): the map is '
            'HOME-REFERENCED (§ 3.2) and without a home node there is nothing '
            'to reference it to. Got x={} y={}'.format(list(x_mm), list(y_mm)))
    return ix, iy


def node_of(rec: Dict[str, Any], x_mm: Sequence[float], y_mm: Sequence[float],
            tol_mm: float = NODE_SNAP_TOL_MM) -> Optional[Tuple[int, int]]:
    """``(ix, iy)`` for a record's nominated catch pose, or ``None`` if it is not
    a node record (an SC-3 off-node check pose, or a pose from another grid)."""
    xyz = rec.get('goal_catch_xyz_stow_mm')
    if not xyz or len(xyz) < 2 or xyz[0] is None or xyz[1] is None:
        return None
    try:
        x, y = float(xyz[0]), float(xyz[1])
    except (TypeError, ValueError):
        return None
    dx = [abs(x - float(v)) for v in x_mm]
    dy = [abs(y - float(v)) for v in y_mm]
    ix, iy = int(np.argmin(dx)), int(np.argmin(dy))
    if dx[ix] > tol_mm or dy[iy] > tol_mm:
        return None
    return ix, iy


def trimmed_mean(values: Sequence[float],
                 fraction: float = TRIM_FRACTION) -> float:
    """Mean after dropping ``floor(fraction·n)`` samples from EACH tail.

    ``floor``, not ``round``: at n = 8 and 10 % that drops nothing, which is the
    correct behaviour — trimming 1 of 8 per tail throws away 25 % of a sample
    that N_MIN already calls marginal. From n = 10 it drops one per tail, which
    is where a single rimshot or mis-fitted track starts to matter.
    """
    vals = sorted(float(v) for v in values)
    if not vals:
        raise TossFitError('trimmed_mean of an empty sample')
    k = int(math.floor(float(fraction) * len(vals)))
    if 2 * k >= len(vals):
        k = 0
    kept = vals[k:len(vals) - k] if k else vals
    return float(sum(kept) / len(kept))


class NodeFit(NamedTuple):
    """One node's reduction. ``stale`` nodes are carried, never re-fitted."""
    ix: int
    iy: int
    n: int
    rx: float
    ry: float
    sd_rx: float
    sd_ry: float
    date_first: str
    date_last: str
    stale: bool
    source: str          # 'fit' | 'previous'


class AnchorVisit(NamedTuple):
    """One contiguous visit to the home node — the toss analogue of
    ``tilt_cal_grid.HomeAnchor``."""
    t_s: float
    rx: float
    ry: float
    n: int


def _previous_covers(previous: Optional[toss_cal.TossCal],
                     x_mm: Sequence[float], y_mm: Sequence[float]) -> bool:
    """Is ``previous`` defined on EXACTLY this grid, node for node?

    A previous map on a different grid cannot supply a thin node's value by
    index — ``[iy][ix]`` would silently read a different physical pose, which is
    the transposed-index class of error dressed up as a refresh. It could be
    *interpolated*, and that is refused for the reason D15 gives: interpolation
    invents calibration exactly where the machine had trouble.
    """
    if previous is None:
        return False
    if previous.shape != (len(y_mm), len(x_mm)):
        return False
    return (np.allclose(np.asarray(previous.x_mm, dtype=float),
                        np.asarray(x_mm, dtype=float), atol=1e-6)
            and np.allclose(np.asarray(previous.y_mm, dtype=float),
                            np.asarray(y_mm, dtype=float), atol=1e-6))


def _sd(values: Sequence[float]) -> float:
    if len(values) < 2:
        return float('nan')
    return float(np.std(np.asarray(values, dtype=float), ddof=1))


def _iso_date(t: Optional[float]) -> str:
    if t is None or not math.isfinite(float(t)):
        return ''
    return datetime.fromtimestamp(float(t)).strftime('%Y-%m-%d')


def fit_nodes(records: Sequence[Dict[str, Any]],
              x_mm: Sequence[float], y_mm: Sequence[float], *,
              n_min: int = N_MIN,
              previous: Optional[toss_cal.TossCal] = None,
              snap_tol_mm: float = NODE_SNAP_TOL_MM
              ) -> Tuple[Dict[Tuple[int, int], NodeFit],
                         Dict[Tuple[int, int], List[Dict[str, Any]]],
                         Dict[str, int]]:
    """Reduce an admitted corpus to one :class:`NodeFit` per grid node.

    Returns ``(fits, admitted_by_node, excluded_counts)``. ``fits`` carries an
    entry for every node of the grid; ``admitted_by_node`` is the per-node
    admitted rows (the analyser and the ball-flew guard both want them).

    **Thin nodes keep their PREVIOUS value (D15).** A node with fewer than
    ``n_min`` admitted tosses is emitted with ``stale=True`` and
    ``source='previous'``, carrying the previous map's number. It is never
    interpolated from its neighbours — that would invent calibration data at
    exactly the place the machine had trouble — and it never fails the whole
    write, because this map is an *incremental refinement* and refusing would
    block 24 good nodes because one had a thin week. A thin node with no
    previous value is emitted as zero and reported; :func:`ball_flew_verdict` is
    what refuses the write in that case.
    """
    admitted: Dict[Tuple[int, int], List[Dict[str, Any]]] = {}
    excluded: Dict[str, int] = {}
    for rec in records:
        node = node_of(rec, x_mm, y_mm, snap_tol_mm)
        ok, reason = admit_for_aim(rec)
        if not ok:
            excluded[reason] = excluded.get(reason, 0) + 1
            continue
        if node is None:
            excluded['off_node'] = excluded.get('off_node', 0) + 1
            continue
        admitted.setdefault(node, []).append(rec)

    fits: Dict[Tuple[int, int], NodeFit] = {}
    for iy in range(len(y_mm)):
        for ix in range(len(x_mm)):
            rows = admitted.get((ix, iy), [])
            n = len(rows)
            if n >= int(n_min):
                bx, by = [], []
                stamps = []
                for rec in rows:
                    b = reduce_to_aim(rec)
                    bx.append(b[0])
                    by.append(b[1])
                    stamps.append(rec.get('announce_throw_time_ros'))
                good = [t for t in stamps if t is not None]
                fits[(ix, iy)] = NodeFit(
                    ix=ix, iy=iy, n=n,
                    rx=trimmed_mean(bx), ry=trimmed_mean(by),
                    sd_rx=_sd(bx), sd_ry=_sd(by),
                    date_first=_iso_date(min(good) if good else None),
                    date_last=_iso_date(max(good) if good else None),
                    stale=False, source='fit')
                continue
            prev_rx, prev_ry = 0.0, 0.0
            source = 'never_measured'
            if _previous_covers(previous, x_mm, y_mm):
                prev_rx = float(previous.rx_rad[iy][ix])
                prev_ry = float(previous.ry_rad[iy][ix])
                source = 'previous'
            fits[(ix, iy)] = NodeFit(
                ix=ix, iy=iy, n=n, rx=prev_rx, ry=prev_ry,
                sd_rx=float('nan'), sd_ry=float('nan'),
                date_first='', date_last='', stale=True, source=source)
    return fits, admitted, excluded


def anchor_visits(records: Sequence[Dict[str, Any]],
                  x_mm: Sequence[float], y_mm: Sequence[float], *,
                  gap_s: float = 120.0) -> List[AnchorVisit]:
    """Group the admitted HOME-node records into contiguous visits, in time.

    A "visit" is a run of home-node tosses with no admitted non-home toss between
    them and no gap longer than ``gap_s``. This is the toss analogue of the tilt
    tool's interleaved home anchors, and the series is what discriminates arrival
    repeatability from a drifting plant across one capture — the number the
    anchor gate reports (§ 3.8) and the analyser plots.
    """
    hx, hy = home_index(x_mm, y_mm)
    rows = []
    for rec in records:
        ok, _ = admit_for_aim(rec)
        if not ok:
            continue
        t = rec.get('announce_throw_time_ros')
        if t is None:
            continue
        rows.append((float(t), node_of(rec, x_mm, y_mm), rec))
    rows.sort(key=lambda r: r[0])

    visits: List[AnchorVisit] = []
    run: List[Tuple[float, Dict[str, Any]]] = []

    def _flush():
        if not run:
            return
        bx, by = [], []
        for _, rec in run:
            b = reduce_to_aim(rec)
            bx.append(b[0])
            by.append(b[1])
        visits.append(AnchorVisit(t_s=run[0][0], rx=float(np.mean(bx)),
                                  ry=float(np.mean(by)), n=len(run)))
        run.clear()

    for t, node, rec in rows:
        if node != (hx, hy):
            _flush()
            continue
        if run and (t - run[-1][0]) > float(gap_s):
            _flush()
        run.append((t, rec))
    _flush()
    return visits


def anchor_estimate(visits: Sequence[AnchorVisit]
                    ) -> Tuple[Tuple[float, float], Tuple[float, float], int]:
    """``((rx, ry), (se_rx, se_ry), n_tosses)`` — the anchor MEAN and its se.

    The **mean over visits**, not over tosses, and deliberately: the quantity the
    reference must be robust to is *session drift between visits*, so a visit
    with 12 tosses must not outvote three visits with 4 each. Same choice
    ``tilt_cal_grid.anchor_mean_rad`` made, for the same reason.

    ``se`` is the scatter BETWEEN visits over √n_visits; with a single visit it
    is ``nan`` — one anchor cannot estimate its own repeatability, and reporting
    a zero there would read as a perfectly repeatable machine.
    """
    usable = [v for v in visits if v.n > 0]
    if not usable:
        raise TossFitError(
            'no admitted tosses at the home node — the map is home-referenced '
            '(§ 3.2), so a capture that never flew a ball at (0, 0) has no '
            'reference and cannot be written')
    rx = float(np.mean([v.rx for v in usable]))
    ry = float(np.mean([v.ry for v in usable]))
    if len(usable) < 2:
        se = (float('nan'), float('nan'))
    else:
        se = (float(np.std([v.rx for v in usable], ddof=1) / math.sqrt(len(usable))),
              float(np.std([v.ry for v in usable], ddof=1) / math.sqrt(len(usable))))
    return (rx, ry), se, sum(v.n for v in usable)


# ── Gates that refuse the write (§ 3.8) ──────────────────────────────────────


def flat_field_verdict(fits: Dict[Tuple[int, int], NodeFit],
                       home_ref: Tuple[float, float],
                       tol_rad: float = THETA_ACC_RAD) -> Tuple[bool, str]:
    """``(is_flat, message)`` — is every HOME-REFERENCED node smaller than θ_acc?

    A map whose largest node is under the accuracy floor of the layer beneath it
    is indistinguishable from that layer's noise: it buys nothing and costs a
    version bump, a redeploy and an operator's trust in the next real one. Unlike
    the tilt tool's namesake — which only WARNS, because a flat tilt field is the
    outcome that whole programme is trying to reach — this one **refuses the
    write** (§ 3.8), and the CLI's ``--allow-flat-field`` is the documented
    override, stamped into ``captured``.

    Note the guard is computed on the SHIPPED (home-referenced) numbers, so a
    large *common* bias with no spatial structure is correctly called flat: the
    common mode is the session trim's job (§ 3.2), and it lives in
    ``anchor.aim_rad``, not in the grid.
    """
    if not fits:
        return False, 'no nodes — flatness is undefined'
    mags = []
    for fit in fits.values():
        mags.append(math.hypot(fit.rx - float(home_ref[0]),
                               fit.ry - float(home_ref[1])))
    peak = max(mags)
    if peak >= float(tol_rad):
        return (False,
                'field varies: largest home-referenced node {:.4f}° >= θ_acc '
                '{:.3f}°'.format(math.degrees(peak), math.degrees(tol_rad)))
    return (True,
            'FLAT FIELD — every one of {} home-referenced nodes is under '
            'θ_acc ({:.4f}° peak vs {:.3f}°). A map this small is '
            'indistinguishable from the tilt map\'s own accuracy floor: it '
            'would change no landing by more than {:.1f} mm at h = 0.78 m, '
            'while costing a version bump every consumer has to re-verify. If '
            'the machine really is this well aimed, that is the RESULT — record '
            'it in the logbook and ship no map. Override with '
            '--allow-flat-field.'
            .format(len(fits), math.degrees(peak), math.degrees(tol_rad),
                    peak * 3126.53))


def ball_flew_verdict(fits: Dict[Tuple[int, int], NodeFit],
                      previous: Optional[toss_cal.TossCal]
                      ) -> Tuple[bool, str]:
    """``(ok, message)`` — did a ball actually fly?

    The direct analogue of the tilt-cal DISARMED capture that wrote a plausible
    all-zeros map because nothing moved (§ 3.8). Two refusals, with different
    root causes:

    * **Nothing flew anywhere.** Every node has zero admitted tosses. Whatever
      the corpus is, it is not a capture — writing would only churn the version.
      This fires even when a previous map exists, because there is no new
      information in the corpus at all.
    * **A node was never measured, ever.** Zero admitted tosses here AND no
      previous value to carry. D15's thin-node rule keeps a *previous* value; it
      cannot conjure one, and shipping a zero would put an unmeasured node into a
      bilinear blend with its measured neighbours, inventing calibration between
      them.

    A node that is merely THIN (some tosses, fewer than ``N_MIN``) is not a
    refusal — that is exactly the case D15 exists for.
    """
    if not fits:
        return False, 'no nodes'
    flew = [f for f in fits.values() if f.n > 0]
    if not flew:
        return (False,
                'BALL NEVER FLEW: not one of the {} grid nodes has a single '
                'admitted toss. This is the signature of a capture in which '
                'nothing actually happened — the toss analogue of the '
                '2026-07-15 DISARMED tilt capture that wrote a plausible '
                'all-zeros map. Check the wire was armed, the hand sensor was '
                'valid, and the mocap could see the ball over the cup (the '
                'reference sitting could NOT: 0/31 usable, § 10).'
                .format(len(fits)))
    never = [f for f in fits.values()
             if f.n == 0 and f.source == 'never_measured']
    if never:
        return (False,
                'refusing to write: {} node(s) have NO admitted toss in this '
                'corpus and NO previous value to carry — {} ({}). A thin node '
                'keeps its previous number (D15); a never-measured node has '
                'none, and shipping a zero would drag its measured neighbours '
                'toward zero through the bilinear blend, inventing calibration '
                'exactly where the machine has never been.'
                .format(len(never),
                        ', '.join('[ix={},iy={}]'.format(f.ix, f.iy)
                                  for f in never),
                        'no previous map is installed' if previous is None
                        else 'the previous map is on a different grid'))
    return True, '{}/{} nodes carry at least one admitted toss'.format(
        len(flew), len(fits))


# ── Population statistics (§ 3.7 stats block, § 6 discriminators) ────────────


def sigma_land_mm(records: Sequence[Dict[str, Any]]) -> Optional[float]:
    """Pooled per-axis sd of ``land_err_mm`` over admitted tosses, mm.

    THE number the pre-registered discriminators in § 6 are read against
    (σ_L ≥ 18 ⇒ chase variance; ≤ 12 with catches still failing ⇒ chase the seat;
    > 30 ⇒ stop). Each axis is centred on **its own** mean before pooling: a
    common bias is exactly what the map removes, and folding it into σ would make
    the *pre*-calibration scatter look larger than the plant's and the
    *post*-calibration improvement look like variance reduction it is not.
    """
    rows: List[Tuple[float, float]] = []
    for rec in records:
        ok, _ = admit_for_aim(rec)
        if not ok:
            continue
        err = rec.get('land_err_mm')
        if _is_pair(err):
            rows.append((float(err[0]), float(err[1])))
    if len(rows) < 3:
        return None
    arr = np.asarray(rows, dtype=float)
    centred = arr - arr.mean(axis=0)
    # Pooled per-axis sd: sum of squares over both axes, (n-1) dof per axis.
    return float(math.sqrt(float((centred ** 2).sum())
                           / (2.0 * (len(rows) - 1))))


def r_eff_mm(records: Sequence[Dict[str, Any]]) -> Optional[float]:
    """The capture radius the observed catch rate IMPLIES, mm.

    Inverts the Rayleigh model of F6 — ``P = 1 − exp(−R²/2σ²)`` ⇒
    ``R = σ·√(−2·ln(1 − P))`` — using the measured σ and the measured clean-catch
    rate. Compared against the geometric ``GEOM_HAND_RADIUS_MM`` = 35.0 it is a
    direct read on how much of the miss rate is NOT explained by arrival scatter:
    an ``R_eff`` well under 35 means balls that arrived inside the cup were not
    kept, which is the seat, not the aim (§ 6 discriminator 2).
    """
    sigma = sigma_land_mm(records)
    if sigma is None or sigma <= 0.0:
        return None
    labelled = [r for r in records if r.get('label') in AIM_LABELS]
    if not labelled:
        return None
    caught = sum(1 for r in labelled
                 if r.get('label') == toss_record.LABEL_CAUGHT)
    p = caught / float(len(labelled))
    if p <= 0.0 or p >= 1.0:
        return None
    return float(sigma * math.sqrt(-2.0 * math.log(1.0 - p)))


def speed_fit(records: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """The speed gain ``k_v`` (§ 3.6.1): ``k̂_v = √(mean(h_cmd / h_ach))``.

    Multiplies ``event_vel_mps`` so ``h_ach → h_cmd``. Returns the estimate, its
    standard error, ``n``, and whether the § 3.6.1 apply gate (n ≥ 5, se ≤ 1 %,
    deadband 1 %) is met. Safe by construction — ``STROKE_TOP_REV`` is
    algebraically velocity-independent, so a speed trim cannot move the hand
    toward its end stop.
    """
    ratios: List[float] = []
    excluded: Dict[str, int] = {}
    for rec in records:
        ok, reason = admit_for_speed(rec)
        if not ok:
            excluded[reason] = excluded.get(reason, 0) + 1
            continue
        T_cmd = _num(rec.get('flight_time_s'))
        T_ach = _num(rec.get('achieved_flight_s_mocap'))
        ratios.append((T_cmd / T_ach) ** 2)          # h ∝ T²
    out: Dict[str, Any] = {'n': len(ratios), 'k_v': None, 'se': None,
                           'applies': False, 'excluded': excluded}
    if len(ratios) < SPEED_TIMING_N_MIN:
        return out
    mean = float(np.mean(ratios))
    k_v = math.sqrt(mean) if mean > 0.0 else None
    if k_v is None:
        return out
    sd = float(np.std(np.asarray(ratios), ddof=1))
    se_mean = sd / math.sqrt(len(ratios))
    se_kv = se_mean / (2.0 * k_v)                     # d(√m)/dm = 1/(2√m)
    out.update({'k_v': k_v, 'se': se_kv,
                'applies': bool(se_kv <= SPEED_SE_MAX
                                and abs(k_v - 1.0) >= 0.01)})
    return out


def timing_fit(records: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """The release latency ``τ`` = median(departure edge − announced throw), ms.

    The gate is the re-derived one (:func:`admit_for_timing`), and the applied
    standard error is **quantisation-limited**: the measurand's dispersion on the
    reference bag is 20.51 ms against a poll-quantisation prediction of 20.50 ms,
    so the sample sd IS the cadence, and the honest ``se`` is
    ``sd/√n``. ``applies`` reports the § 3.6.1 gate (n ≥ 5, se ≤ 5 ms, deadband
    10 ms).

    Also returns ``uptime_trend_ms_per_h`` — the within-session residual-vs-uptime
    slope (D16). The design **refuses** a timing fit whose within-session trend
    exceeds the between-node signal; that comparison needs the node fit, so it is
    reported here and enforced by the caller, which has both.
    """
    shifts: List[float] = []
    uptimes: List[float] = []
    excluded: Dict[str, int] = {}
    for rec in records:
        ok, reason = admit_for_timing(rec)
        if not ok:
            excluded[reason] = excluded.get(reason, 0) + 1
            continue
        dep = _num(rec.get('t_departure_raw_ros'))
        ann = _num(rec.get('announce_throw_time_ros'))
        if dep is None or ann is None:
            excluded['malformed_edge'] = excluded.get('malformed_edge', 0) + 1
            continue
        shifts.append((dep - ann) * 1e3)
        up = _num(rec.get('uptime_ms_at_release'))
        uptimes.append(up / 3.6e6 if up is not None else float('nan'))
    out: Dict[str, Any] = {'n': len(shifts), 'tau_ms': None, 'se_ms': None,
                           'sd_ms': None, 'applies': False,
                           'uptime_trend_ms_per_h': None, 'excluded': excluded}
    if not shifts:
        return out
    arr = np.asarray(shifts, dtype=float)
    out['tau_ms'] = float(np.median(arr))
    if len(arr) >= 2:
        sd = float(np.std(arr, ddof=1))
        out['sd_ms'] = sd
        out['se_ms'] = sd / math.sqrt(len(arr))
    pairs = [(u, s) for u, s in zip(uptimes, shifts) if math.isfinite(u)]
    if len(pairs) >= 3:
        u = np.asarray([p[0] for p in pairs])
        s = np.asarray([p[1] for p in pairs])
        if float(np.ptp(u)) > 0.0:
            out['uptime_trend_ms_per_h'] = float(np.polyfit(u, s, 1)[0])
    if (out['se_ms'] is not None and len(arr) >= SPEED_TIMING_N_MIN
            and out['se_ms'] <= TAU_SE_MAX_MS and abs(out['tau_ms']) >= 10.0):
        out['applies'] = True
    return out


def score_groups(records: Sequence[Dict[str, Any]],
                 groups: Sequence[Tuple[str, Sequence[Dict[str, Any]]]]
                 ) -> List[Dict[str, Any]]:
    """A/B scoring rows, mirroring ``ball_arrival_offset.py``'s ``--group`` CLI.

    Scored on the **continuous** observables (σ_L, |mean land_err|, R_eff), never
    on the label alone: distinguishing 0.80 → 0.90 catch rate needs 196 tosses
    per arm (F4), which is 39 minutes of pure throwing per arm, while the arrival
    offset resolves a 15 mm bias in 8. The catch rate is reported because the
    operator wants to see it, and it is explicitly not the discriminator.
    """
    rows: List[Dict[str, Any]] = []
    for name, subset in groups:
        admitted = [r for r in subset if admit_for_aim(r)[0]]
        errs = np.asarray([r['land_err_mm'] for r in admitted], dtype=float) \
            if admitted else np.zeros((0, 2))
        labelled = [r for r in subset if r.get('label') in AIM_LABELS]
        caught = sum(1 for r in labelled
                     if r.get('label') == toss_record.LABEL_CAUGHT)
        rows.append({
            'name': name,
            'n_records': len(subset),
            'n_admitted': len(admitted),
            'mean_err_mm': [float(errs[:, 0].mean()), float(errs[:, 1].mean())]
                           if len(errs) else None,
            'sigma_mm': sigma_land_mm(subset),
            'r_eff_mm': r_eff_mm(subset),
            'catch_rate': (caught / float(len(labelled))) if labelled else None,
            'n_labelled': len(labelled),
        })
    return rows


# ── The document ─────────────────────────────────────────────────────────────


def build_map_document(x_mm: Sequence[float], y_mm: Sequence[float],
                       z_mm: float,
                       fits: Dict[Tuple[int, int], NodeFit],
                       anchor: Tuple[Tuple[float, float], Tuple[float, float], int],
                       *,
                       captured: Dict[str, Any],
                       requires: Dict[str, Any],
                       jacobian: Dict[str, Any],
                       speed: Optional[Dict[str, Any]] = None,
                       stats_extra: Optional[Dict[str, Any]] = None
                       ) -> Dict[str, Any]:
    """Assemble the schema-v1 ``config/toss_calibration.yaml`` document.

    **The grid ships HOME-REFERENCED** — every node is ``b(P) − anchor_mean`` —
    and the absolute anchor ships separately in ``anchor.aim_rad`` as phase 2e's
    warm-start prior (§ 3.2). A constant added to every reduced node, anchor
    included, therefore produces a byte-identical grid: that is the level-noise
    immunity the whole referencing scheme exists for, and it is pinned by a test.

    Numbers are rounded — aim to 9 dp (1 nrad, six orders below anything the
    machine can command), axes to 6 dp — purely so the committed YAML is
    diff-readable node by node and byte-stable across a re-fit that changes
    nothing. :func:`toss_cal.map_version` float-normalises before hashing, so the
    rounding cannot churn the version.
    """
    (a_rx, a_ry), (se_rx, se_ry), a_n = anchor
    n_x, n_y = len(x_mm), len(y_mm)
    rx_grid: List[List[float]] = []
    ry_grid: List[List[float]] = []
    sd_rx: List[List[Optional[float]]] = []
    sd_ry: List[List[Optional[float]]] = []
    n_grid: List[List[int]] = []
    dates: List[List[str]] = []
    stale_nodes: List[Dict[str, Any]] = []

    for iy in range(n_y):
        rx_row, ry_row, sdx_row, sdy_row, n_row, date_row = [], [], [], [], [], []
        for ix in range(n_x):
            fit = fits.get((ix, iy))
            if fit is None:
                raise TossFitError(
                    'build_map_document: no NodeFit for [ix={}, iy={}] — '
                    'fit_nodes emits every node of the grid, so a gap here '
                    'means the grid and the fits disagree'.format(ix, iy))
            rx_row.append(round(float(fit.rx) - float(a_rx), 9))
            ry_row.append(round(float(fit.ry) - float(a_ry), 9))
            sdx_row.append(None if not math.isfinite(fit.sd_rx)
                           else round(float(fit.sd_rx), 9))
            sdy_row.append(None if not math.isfinite(fit.sd_ry)
                           else round(float(fit.sd_ry), 9))
            n_row.append(int(fit.n))
            date_row.append('{}..{}'.format(fit.date_first or '-',
                                            fit.date_last or '-'))
            if fit.stale:
                stale_nodes.append({
                    'ix': ix, 'iy': iy,
                    'x_mm': round(float(x_mm[ix]), 6),
                    'y_mm': round(float(y_mm[iy]), 6),
                    'n': int(fit.n), 'source': fit.source,
                })
        rx_grid.append(rx_row)
        ry_grid.append(ry_row)
        sd_rx.append(sdx_row)
        sd_ry.append(sdy_row)
        n_grid.append(n_row)
        dates.append(date_row)

    stats: Dict[str, Any] = {
        'n_per_node': n_grid,
        'sd_rx_rad': sd_rx,
        'sd_ry_rad': sd_ry,
        'date_range': dates,
        'stale_nodes': stale_nodes,
        'failed_nodes': [],
    }
    if stats_extra:
        stats.update(stats_extra)

    doc: Dict[str, Any] = {
        'version': toss_cal.SCHEMA_VERSION,
        'captured': dict(captured),
        'requires': dict(requires),
        'units': {'aim': toss_cal.AIM_UNIT,
                  'height_scaling_exponent': 1.0,
                  'h_capture_m': None},
        'jacobian': dict(jacobian),
        'grid': {
            'z_mm': round(float(z_mm), 6),
            'orientation': 'level',
            'x_mm': [round(float(v), 6) for v in x_mm],
            'y_mm': [round(float(v), 6) for v in y_mm],
        },
        'aim_rad': {'rx': rx_grid, 'ry': ry_grid},
        'anchor': {
            'aim_rad': [round(float(a_rx), 9), round(float(a_ry), 9)],
            'n': int(a_n),
            'se_rad': [None if not math.isfinite(se_rx) else round(float(se_rx), 9),
                       None if not math.isfinite(se_ry) else round(float(se_ry), 9)],
        },
        'speed': dict(speed or {'k_v': None, 'se': None}),
        'stats': stats,
    }
    return doc


def validate_map_document(doc: Dict[str, Any]) -> toss_cal.TossCal:
    """Self-validate through the **production loader** before writing.

    Deliberately :func:`toss_cal.parse_toss_cal` rather than a re-implemented
    check list: the only property that matters is *"reload_coordinator_node will
    accept this file"*, and a second copy of the validation rules is a second
    thing to drift.
    """
    try:
        return toss_cal.parse_toss_cal(doc)
    except toss_cal.TossCalError as exc:
        raise TossFitError(
            'refusing to write — the map this fit produced would be REJECTED by '
            'reload_coordinator_node:\n  {}\n'
            'Likely causes, in order: (a) a node past the {:.2f}° authority '
            'bound, which is a capture fault (wrong sign convention, a units '
            'error, or a fit on a degraded plant) rather than a calibration; '
            '(b) a grid axis with fewer than 2 nodes; (c) a units key this '
            'build does not apply.'
            .format(exc, math.degrees(toss_cal.TOTAL_MAX_RAD)))


def dump_map_yaml(doc: Dict[str, Any]) -> str:
    """Serialise with a provenance header. Block style: one number per line makes
    ``git diff`` between two captures readable node by node, which is the whole
    reason this file is committed."""
    header = (
        '# Pose-dependent toss AIM calibration map — contract C-TOSS-CAL-1\n'
        '# (plans/active/toss-selftuning.md). MACHINE-WRITTEN by\n'
        '# {} — do not hand-edit: the fit tool\n'
        '# rewrites this file wholesale, and a hand-edited node has no\n'
        '# measurement behind it. aim_rad is the COMMANDED aim tilt and the\n'
        '# grid is HOME-REFERENCED; the absolute home residual lives in\n'
        '# anchor.aim_rad as the session trim\'s warm-start prior, NOT as a\n'
        '# correction. An ABSENT file is a silent, non-gating state.\n'
        .format(TOOL_NAME))
    return header + yaml.safe_dump(doc, default_flow_style=False,
                                   sort_keys=False)


def diff_documents(old: Dict[str, Any], new: Dict[str, Any]) -> Dict[str, Any]:
    """Node-by-node delta between two map documents — the ``--dry-run`` print and
    the analyser's map-vs-map invariance check (§ 3.8: max node delta ≤
    max(2·se, 0.05°))."""
    def _grid(doc, axis):
        return np.asarray(((doc.get('aim_rad') or {}).get(axis) or []),
                          dtype=float)
    out: Dict[str, Any] = {'shape_match': True, 'nodes': [], 'max_delta_rad': 0.0}
    orx, ory = _grid(old, 'rx'), _grid(old, 'ry')
    nrx, nry = _grid(new, 'rx'), _grid(new, 'ry')
    if orx.shape != nrx.shape or ory.shape != nry.shape or orx.size == 0:
        out['shape_match'] = False
        return out
    for iy in range(nrx.shape[0]):
        for ix in range(nrx.shape[1]):
            d_rx = float(nrx[iy][ix] - orx[iy][ix])
            d_ry = float(nry[iy][ix] - ory[iy][ix])
            mag = math.hypot(d_rx, d_ry)
            out['nodes'].append({'ix': ix, 'iy': iy, 'd_rx': d_rx,
                                 'd_ry': d_ry, 'mag_rad': mag})
            out['max_delta_rad'] = max(out['max_delta_rad'], mag)
    return out


def write_target_path(environ: Optional[Dict[str, str]] = None) -> str:
    """Where the map is written — the **first candidate of the production
    resolver**, never a walk of this file's own location.

    ``toss_cal.toss_cal_candidates()`` is the same list the node resolves, so the
    tool cannot write one file while the node reads another and both report
    success. When ``$JUGGLEBOT_TOSS_CAL`` is set it is authoritative and the only
    candidate, so this returns it: writing anywhere else would apply a different
    calibration than the operator named.
    """
    candidates = toss_cal.toss_cal_candidates(environ)
    if not candidates:
        raise TossFitError(
            'no toss-calibration candidate path exists — this tree does not '
            'look like the repo and $%s is unset' % toss_cal.TOSS_CAL_ENV)
    return candidates[0]


def synthetic_corpus(x_mm: Sequence[float], y_mm: Sequence[float], *,
                     plant_bias_rad,
                     applied_aim_rad_fn=None,
                     n_per_node: int = 10,
                     sigma_mm: float = 20.0,
                     z_mm: float = 170.0,
                     apex_m: float = 0.78,
                     seed: int = 20260811,
                     t0: float = 1786343595.0,
                     dt_s: float = 6.0,
                     label_fn=None) -> List[Dict[str, Any]]:
    """A replayable corpus with a **known** plant aim bias — the closed-loop
    driver for the phase-2c acceptance test and for the CLI's ``--self-check``.

    ``plant_bias_rad`` is either a ``(ψx, ψy)`` pair (constant field) or a
    callable ``(x_mm, y_mm) -> (ψx, ψy)`` (spatial field). ``applied_aim_rad_fn``
    is the aim the machine was commanding while the corpus was recorded — the
    lever that proves the reduction is a *fixed point*: a corpus captured with a
    map installed must reduce to the same answer as one captured without.

    **The landing error is generated by the FORWARD production model**::

        land_err = aim_target_offset_mm(A + ψ, T, z) + noise

    ``aim_target_offset_mm`` is the same function ``reload_coordinator_node``
    uses to apply the map, and :func:`reduce_to_aim` inverts it through a
    *separately computed* Jacobian. So the round trip exercises the real sign
    convention in both directions rather than restating one of them: if the fit's
    inverse disagreed with the apply path's forward map — the R1 failure that
    aims the machine twice as badly as no map — the recovered bias would come
    back with the wrong sign or on the wrong axis, and the test would say so.

    Noise is isotropic Gaussian on the landing point with per-axis
    ``sigma_mm``; 20 mm is the design's working assumption and 26 mm is what the
    2026-08-10 retest inverts to (F6).
    """
    rng = np.random.default_rng(seed)
    T = math.sqrt(8.0 * float(apex_m) / _G)
    records: List[Dict[str, Any]] = []
    t = float(t0)
    for iy, y in enumerate(y_mm):
        for ix, x in enumerate(x_mm):
            for k in range(int(n_per_node)):
                psi = (plant_bias_rad(x, y) if callable(plant_bias_rad)
                       else tuple(plant_bias_rad))
                applied = ((0.0, 0.0) if applied_aim_rad_fn is None
                           else tuple(applied_aim_rad_fn(x, y)))
                total = (applied[0] + psi[0], applied[1] + psi[1])
                offset = aim_target_offset_mm(total[0], total[1], T, float(z_mm))
                err = (float(offset[0]) + float(rng.normal(0.0, sigma_mm)),
                       float(offset[1]) + float(rng.normal(0.0, sigma_mm)))
                label = (label_fn(err) if label_fn is not None
                         else (toss_record.LABEL_CAUGHT
                               if math.hypot(*err) <= hw.GEOM_HAND_RADIUS_MM
                               else toss_record.LABEL_MISSED))
                rec = toss_record.blank_record()
                rec.update({
                    'toss_uid': 'syn-{}-{}-{}'.format(ix, iy, k),
                    'action': 'toss_continuous',
                    'announce_throw_time_ros': t,
                    'goal_catch_xyz_stow_mm': [float(x), float(y), float(z_mm)],
                    'flight_time_s': T,
                    'achieved_flight_s_mocap': T,
                    'apex_height_m': float(apex_m),
                    'toss_tier': '8a',
                    'tilt_map_version': 'synthetic-tilt/1',
                    # G5 (levelling) is enforced by the SHARED admission filter
                    # from phase 2e on, so a synthetic corpus has to model a
                    # correctly-levelled machine explicitly. Leaving these null
                    # would make every synthetic record refuse with
                    # ``no_gravity_correction`` — which is the guard working,
                    # not a fixture quirk.
                    'gravity_correction_loaded': True,
                    'tilt_map_applied': True,
                    'toss_cal_version': 'synthetic-toss-cal/1',
                    'bridge_fw_version': '10 (proto 5)',
                    'platform_fw_version': '2',
                    'map_aim_rad': [applied[0], applied[1]],
                    'trim_aim_rad': [0.0, 0.0],
                    'total_aim_rad': [applied[0], applied[1]],
                    'outcome': 'CAUGHT' if label == toss_record.LABEL_CAUGHT
                               else 'MISSED',
                    'throw_stroke_seen': True,
                    'ball_track_confirmed': True,
                    't_departure_raw_ros': t + 0.175,
                    'land_err_mm': [err[0], err[1]],
                    'land_err_norm_mm': math.hypot(*err),
                    'n_fit': 12,
                    'fit_rms_mm': 1.2,
                    'fit_sparse': False,
                    'sensor_valid_frac': 1.0,
                    'sensor_poll_dt_ms_median': 70.0,
                    'ball_held_stamp_wall_anchored': True,
                    'uptime_ms_at_release': int((t - t0) * 1e3) + 60000,
                    'label': label,
                    'label_source': 'synthetic',
                    'record_provenance': toss_record.PROV_BOTH,
                })
                records.append(rec)
                t += float(dt_s)
    return records


def default_estimator_block() -> Dict[str, Any]:
    """``requires.estimator`` — the arrival-offset estimator's own parameters.

    Pulled from ``tools/probes/ball_arrival_offset`` at call time rather than
    copied, so a change to the estimator's defaults shows up as a *version
    mismatch* in the artefact instead of as a silently different measurand.
    """
    import importlib.util
    here = os.path.dirname(os.path.abspath(__file__))
    probe = os.path.join(os.path.dirname(here), os.pardir, 'tools', 'probes',
                         'ball_arrival_offset.py')
    probe = os.path.normpath(probe)
    spec = importlib.util.spec_from_file_location('_bao_for_fit', probe)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return {
        'plane_mm_rule': 'GEOM_INITIAL_HEIGHT_MM + active_z + HAND_CATCH_OFFSET_MM',
        'band_mm': float(module.BAND_MM),
        'lateral_gate_mm': float(module.LATERAL_MM),
        'min_samples': int(module.MIN_SAMPLES),
    }
