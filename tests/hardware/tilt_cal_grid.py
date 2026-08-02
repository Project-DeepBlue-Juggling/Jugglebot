#!/usr/bin/env python3
"""Tilt-calibration grid capture — the C-LEVEL-2 acquisition tool (OPERATOR-run).

Drives the platform over an (x, y) grid at one operating height, dwells at each
node, reads the on-board inclinometer N times, and writes an interpolatable
**residual tilt map** to the source-tree ``config/tilt_calibration.yaml``. Then
it asks the running ``trajectory_node`` to reload the file, verifies the applied
version matches what it just wrote, and re-measures a handful of **off-node**
check poses to prove the interpolation actually removed the error.

Contract: ``ros_ws/docs/levelling_frame.md`` § **C-LEVEL-2**.
Plan: ``plans/active/tilt-calibration-grid.md`` (Phase 3 builds this; Phase 4's
rungs C0–C3 run it — ``tests/hardware/session_tilt_calibration.md`` is the
runbook and is the authority for what to do at the robot).

**What a residual is.** ``level`` measures the platform's tilt against gravity at
exactly one pose and applies that offset everywhere, so every *pose-dependent*
kinematic error is invisible to it. With a fresh ``level`` loaded and **no map
loaded**, this tool commands the level orientation at each node and computes::

    residual = mean(raw_reading) + radians(JB_OP_INCLINOMETER_OFFSET_DEG)

— the exact ``state_machine.LevellingHandler`` formula (``:455-460``; it is an
ADD, per axis, in radians). At the home node that is ≈ 0 *by construction*: it is
the pose ``level`` itself measured. A home node that is not ≈ 0 means the level
reference is stale and the capture is aborted rather than baked in.

**Safety posture — request-only, like ``traj_ramp_battery.py``.** This tool
issues ``trajectory/go_to_pose`` requests and reads services. It **never arms,
never changes control mode, never touches limits, never commands the hand**.
Arming, modes and E-STOP belong to the operator. If the platform is not armed in
TRAJECTORY the preflight refuses before anything is requested, and even if that
check were bypassed every move would come back ``WRONG_MODE`` / ``STALE_STATE``
and nothing would move.

**Every exit path returns the platform to the centre node first.** Aborts,
Ctrl-C and exceptions all route through the same return-to-centre; the platform
is never left parked at a raised displaced pose.

**Why the moves are deliberately slow.** ``--move-duration-s`` defaults to 3.0 s
and ``--lean-gain`` is an explicit ``0.0`` (lean shaping OFF, so the terminal
pose is the pure commanded one and nothing about the transit is inherited from
node config). Unshaped ±150 mm laterals are the guard-latch canary *at minimum
duration* — 0.94–1.08 rev deviation against a 1.0 rev latch
(``logbook/2026-07-17-wobble-latch-unshaped-traverse.md``) — but that was
measured at ~312 mm/s leg peaks. A 300 mm traverse over 3.0 s runs about a third
of that, which is what makes lean 0.0 safe here. Do not shorten the duration to
save time without re-reading that entry.

**Why reads are spaced (``--read-gap-s``).** The Murata SCL3300 is read **one
sample per trigger** (``Teensy_code_platform.ino:377-396``, ``getInclination()``)
in **mode 4**, whose inclination output carries a **10 Hz internal low-pass
filter**. Mode 4 is the SCL3300 library's default (``SCL3300.h:144``,
``scl3300_mode = 4``, applied by ``begin()``) and the firmware never calls
``setMode`` — it calls a bare ``inclinometer.begin()``
(``Teensy_code_platform.ino:692``). Back-to-back reads inside ~100 ms therefore
resample the *same* filtered state: the mean is fine but the spread collapses,
and the per-read sd — which gates the home node and sizes θ_acc — would report a
noise floor the sensor does not have. The default 0.15 s gap is just over one
filter period. Rung C0 pins the real number.

**Failed nodes are never shipped.** A node whose reads come back NaN (the tilt
service's failure signal — ``GetTiltReadingService`` has no ``success`` field) is
recorded as failed. ``--on-fail continue`` finishes the sweep so the operator
gets the full CSV and knows *which* nodes are bad, but **no map is written** and
the exit code is nonzero: interpolating a failed node from its neighbours would
invent calibration data at exactly the place the machine had trouble, and
C-LEVEL-2 forbids shipping NaN. Re-run (usually with a longer ``--dwell-s``).

Usage (operator; stack up, platform ARMED in TRAJECTORY, fresh ``level`` done)::

    # default capture — 5x5 over +/-150 mm at z=170, write + apply + verify
    python3 tests/hardware/tilt_cal_grid.py --base-condition "flat floor, no shims"

    # rehearsal: print the node order + ETA, make zero ROS calls
    python3 tests/hardware/tilt_cal_grid.py --dry-run

    # a map is already loaded and must come off before a capture
    python3 tests/hardware/tilt_cal_grid.py --force-uninstall --base-condition "..."

    # rung C2: re-verify the EXISTING map after a re-level, without recapturing
    python3 tests/hardware/tilt_cal_grid.py --verify-only

Runs under the **system python3.8 with ROS 2 sourced** (like every other script
in this directory) — ``source ros_ws/install/setup.bash`` first. It does NOT need
the project venv. The pure core (grid spec, residual math, document assembly, CSV
rows) imports no ROS and is unit-tested in ``tests/motion/test_tilt_cal_grid.py``;
``rclpy`` and the interface packages are imported inside :func:`run` only.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import sys
import time
from datetime import datetime
from typing import Any, Dict, List, NamedTuple, Optional, Sequence, Tuple

import numpy as np
import yaml

import jugglebot.hardware_config as hw
from jugglebot.motion import tilt_map

import _th_test_common as thc

# ── defaults ─────────────────────────────────────────────────────────────
#
# Every default marked PROVISIONAL is a placeholder until rung C0 of
# plans/active/tilt-calibration-grid.md measures it on hardware. C0 exists
# precisely because the SCL3300's noise floor and settling behaviour are
# unmeasured in this repo, and the probe-first rule forbids asserting a
# threshold before a probe pins it. Do not promote any of these to "the
# number" without a C0 measurement behind it.

DEFAULT_BOX_MM = 150.0
DEFAULT_NODES = 5
DEFAULT_Z_MM = 170.0
DEFAULT_DWELL_S = 1.0            # PROVISIONAL (C0 pins settle time)
DEFAULT_N_READS = 8              # PROVISIONAL (C0 pins reads-per-node)
DEFAULT_READ_GAP_S = 0.15        # PROVISIONAL (> 1/10 Hz sensor LPF period)
DEFAULT_MOVE_DURATION_S = 3.0
DEFAULT_CHECK_POSES = 6
DEFAULT_THRESHOLD_DEG = 0.15     # PROVISIONAL (C0 pins theta_acc)
DEFAULT_TIMEOUT_S = 5.0

#: Home-node gate. ``|residual(0,0)|`` must satisfy BOTH:
#:   * within ``HOME_NODE_SD_MULT x`` the per-read sd, floored by
#:     ``HOME_NODE_TOL_FLOOR_RAD`` — the floor exists because a quiet sensor
#:     makes ``3 x sd`` degenerate (the Teensy persistence quantum alone is
#:     1 mrad/axis), and a gate that false-aborts on a good level reference
#:     gets disabled by the third operator who hits it;
#:   * under ``HOME_NODE_ABS_CEIL_RAD`` unconditionally — so a *noisy* sensor
#:     cannot launder a genuinely stale level reference through a wide 3 x sd.
#: One-sided gates fail one of those two ways round; this one fails neither.
HOME_NODE_SD_MULT = 3.0
HOME_NODE_TOL_FLOOR_RAD = 0.0015   # ~0.086 deg  PROVISIONAL
HOME_NODE_ABS_CEIL_RAD = 0.005     # ~0.287 deg  PROVISIONAL

#: Capture wants a freshly power-cycled can-bridge Teensy (the uptime-lag
#: discipline). Warn, never refuse — the tilt read path is request/response over
#: the relay, not the hand-dispatch path the lag was measured on, so refusing
#: would cost a sitting over an effect that may not touch this measurement.
UPTIME_WARN_MS = 30 * 60 * 1000

#: A node needs at least this many finite reads to be usable at all (an sd needs
#: two), AND at least half of the requested reads — a node where most reads
#: failed is a node the machine was unhappy at, not a calibrated one.
MIN_GOOD_READS = 2

#: Dry-run ETA only: measured-ish per-read service round trip (3 relay retries
#: worst case). Never used for control.
EST_READ_S = 0.25

CSV_COLUMNS = (
    'iso', 't_s', 'phase', 'visit', 'ix', 'iy',
    'x_mm', 'y_mm', 'z_mm', 'read_index',
    'raw_tx_rad', 'raw_ty_rad', 'res_tx_rad', 'res_ty_rad',
    'ok', 'uptime_ms', 'note',
)

TOOL_NAME = 'tests/hardware/tilt_cal_grid.py'


class TiltCalError(RuntimeError):
    """An operator-facing refusal: the message is the whole error report."""


class Node(NamedTuple):
    """One grid node — ``ix``/``iy`` index ``x_mm``/``y_mm`` respectively."""

    ix: int
    iy: int
    x_mm: float
    y_mm: float


class ReadStats(NamedTuple):
    """Per-node reduction of N raw inclinometer reads.

    ``res_*`` carry the mounting offset already added (the LevellingHandler
    formula); ``raw_mean_*`` are the sensor means before it, kept because the
    analyser and any future re-derivation want the un-offset numbers.
    ``sd_*`` are **sample** standard deviations (ddof=1) over the finite reads —
    ddof=0 would understate the spread at the small N this tool uses.
    """

    n_ok: int
    n_total: int
    raw_mean_tx: float
    raw_mean_ty: float
    res_tx: float
    res_ty: float
    sd_tx: float
    sd_ty: float


# ── pure core: grid spec ─────────────────────────────────────────────────


def mounting_offset_rad() -> Tuple[float, float]:
    """``radians(JB_OP_INCLINOMETER_OFFSET_DEG)``, the LevellingHandler term.

    Read from the generated config module so a re-tune of
    ``jugglebot_operational.inclinometer_offset_deg`` flows here without a code
    edit — the same single source of truth ``state_machine.build_default_machine``
    wires into ``LevellingHandler``. Returned as a 2-tuple ``(tx, ty)``.
    """
    offset = hw.JB_OP_INCLINOMETER_OFFSET_DEG
    if len(offset) != 2:
        raise TiltCalError(
            'JB_OP_INCLINOMETER_OFFSET_DEG must be [tiltX, tiltY], got '
            '{!r}'.format(offset))
    return (math.radians(float(offset[0])), math.radians(float(offset[1])))


def parse_node_list(text: str, name: str) -> List[float]:
    """Parse ``--x``/``--y`` (comma- or space-separated mm) into a valid axis."""
    tokens = [t for t in text.replace(',', ' ').split() if t]
    try:
        values = [float(t) for t in tokens]
    except ValueError as exc:
        raise TiltCalError(
            '--{} is not a list of numbers: {}'.format(name, exc))
    return validate_axis(values, name)


def validate_axis(values: Sequence[float], name: str) -> List[float]:
    """Enforce the schema's axis invariants at the *tool* boundary.

    The loader enforces these too, but failing here means the operator is told
    before the platform moves for ten minutes rather than after.
    """
    axis = [float(v) for v in values]
    if len(axis) < 2:
        raise TiltCalError(
            'grid axis {} needs at least 2 nodes to interpolate, got {}'
            .format(name, len(axis)))
    if not all(math.isfinite(v) for v in axis):
        raise TiltCalError(
            'grid axis {} contains non-finite values: {}'.format(name, axis))
    if not all(b > a for a, b in zip(axis, axis[1:])):
        raise TiltCalError(
            'grid axis {} must be strictly increasing (ascending, no repeats), '
            'got {}'.format(name, axis))
    return axis


def build_axis(box_mm: float, nodes: int, name: str) -> List[float]:
    """``nodes`` evenly spaced values spanning ``[-box_mm, +box_mm]``.

    Rounded to 6 dp so the YAML carries ``-150.0`` rather than a float-noise
    tail; the axes are hashed into ``tilt_map_version`` after float
    normalisation, so a stable spelling keeps the version stable across runs
    that recalibrate nothing.
    """
    if nodes < 2:
        raise TiltCalError(
            '--nodes must be at least 2 (axis {}), got {}'.format(name, nodes))
    if not math.isfinite(box_mm) or box_mm <= 0.0:
        raise TiltCalError(
            '--box must be a positive half-width in mm, got {}'.format(box_mm))
    step = (2.0 * box_mm) / (nodes - 1)
    return validate_axis(
        [round(-box_mm + i * step, 6) for i in range(nodes)], name)


def home_index(x_mm: Sequence[float], y_mm: Sequence[float]) -> Tuple[int, int]:
    """Index of the ``(0, 0)`` home node, or refuse.

    The home node is not a nicety: it is the stale-level-reference gate, and
    C-LEVEL-2 defines the residual there as ≈ 0 *by construction*. A grid
    without it (``--nodes 4`` over a symmetric box, say) can still be captured
    arithmetically but ships with no evidence that the reference it is measured
    against was valid — which is the one failure that silently corrupts **every**
    node at once. Refuse instead.
    """
    tol = 1e-6
    ix = next((i for i, v in enumerate(x_mm) if abs(v) <= tol), None)
    iy = next((i for i, v in enumerate(y_mm) if abs(v) <= tol), None)
    if ix is None or iy is None:
        raise TiltCalError(
            'the home node (0, 0) must be one of the grid nodes — it is the '
            'stale-level-reference gate, and the residual there is 0 by '
            'construction. Use an odd --nodes over a symmetric --box, or put '
            '0 in both --x and --y. Got x={}, y={}'
            .format(list(x_mm), list(y_mm)))
    return (ix, iy)


def serpentine_order(x_mm: Sequence[float],
                     y_mm: Sequence[float]) -> List[Node]:
    """Boustrophedon sweep: rows of increasing y, alternating x direction.

    Minimises travel — a raster order would fly the full x span back at the end
    of every row, which on a 5x5/±150 grid is four extra 300 mm traverses.
    """
    order: List[Node] = []
    for iy, y in enumerate(y_mm):
        xs = list(enumerate(x_mm))
        if iy % 2:
            xs.reverse()
        for ix, x in xs:
            order.append(Node(ix=ix, iy=iy, x_mm=float(x), y_mm=float(y)))
    return order


def visit_order(x_mm: Sequence[float], y_mm: Sequence[float]) -> List[Node]:
    """The capture order: **home node first**, then the serpentine sweep.

    Home is measured first and exactly once — its reading both gates the level
    reference (before ten minutes of platform motion are spent on a capture that
    would be discarded) and populates its own grid node. Re-measuring it in its
    serpentine slot would produce two different numbers for one node and force
    an arbitrary choice between them.
    """
    hx, hy = home_index(x_mm, y_mm)
    home = Node(ix=hx, iy=hy, x_mm=float(x_mm[hx]), y_mm=float(y_mm[hy]))
    rest = [n for n in serpentine_order(x_mm, y_mm)
            if not (n.ix == hx and n.iy == hy)]
    return [home] + rest


def check_poses(x_mm: Sequence[float], y_mm: Sequence[float],
                count: int) -> List[Tuple[float, float]]:
    """``count`` off-node verification poses, strictly inside the grid hull.

    Cell centres only — the point of the verification pass is to measure what
    the *interpolation* does, and a node pose would measure the stored value
    instead (which is exact there by construction and proves nothing).

    Selection is deterministic and quadrant-round-robin, outermost cell first:
    deterministic so rung C2 can re-verify the very same poses after a re-level
    without recapturing, quadrant-balanced so six poses cannot all land in one
    corner, and outermost-first because the 2026-07-28 extremity table puts the
    largest residual (0.604°) at the corner and the smallest (0.041°) near the
    middle — the corner is where an interpolation error costs a ball.
    """
    if count <= 0:
        return []
    centres: List[Tuple[float, float]] = []
    for iy in range(len(y_mm) - 1):
        for ix in range(len(x_mm) - 1):
            centres.append(((float(x_mm[ix]) + float(x_mm[ix + 1])) / 2.0,
                            (float(y_mm[iy]) + float(y_mm[iy + 1])) / 2.0))
    if not centres:
        raise TiltCalError(
            'the grid has no interior cell to place a check pose in — a check '
            'pose must be off-node, so the grid needs at least 2x2 nodes')

    quadrants: List[List[Tuple[float, float]]] = [[], [], [], []]
    for cx, cy in centres:
        q = (0 if cx >= 0 else 1) + (0 if cy >= 0 else 2)
        quadrants[q].append((cx, cy))
    for bucket in quadrants:
        # Outermost first, then a stable tie-break so the order never depends
        # on dict/set iteration or floating-point ties.
        bucket.sort(key=lambda p: (-(p[0] ** 2 + p[1] ** 2), p[0], p[1]))

    chosen: List[Tuple[float, float]] = []
    rank = 0
    while len(chosen) < min(count, len(centres)):
        progressed = False
        for bucket in quadrants:
            if rank < len(bucket):
                chosen.append(bucket[rank])
                progressed = True
                if len(chosen) == min(count, len(centres)):
                    break
        if not progressed:
            break
        rank += 1
    return chosen


# ── pure core: residual math ─────────────────────────────────────────────


def residual_from_reads(reads: Sequence[Sequence[float]],
                        offset_rad: Tuple[float, float]) -> ReadStats:
    """Reduce N raw ``(tx, ty)`` reads to a residual + per-read spread.

    ``reads`` may contain NaN pairs — that is how ``get_platform_tilt`` reports a
    failed read (the service has no ``success`` field). A pair is kept only if
    **both** axes are finite: the bridge writes ``[nan, nan]`` together, and a
    half-finite pair would mean something has gone wrong that averaging should
    not smooth over.

    The residual is ``mean(raw) + radians(inclinometer_offset_deg)``, per axis,
    added — the exact ``LevellingHandler`` formula (``state_machine.py:455-460``).
    Getting this sign wrong inverts every node in the map and the machine would
    aim twice as badly as with no map at all, so it is pinned by
    ``tests/motion/test_tilt_cal_grid.py`` against the real generated constant.
    """
    # Materialise before iterating twice — a generator would report n_total = 0
    # after the comprehension had already consumed it, silently turning every
    # node into "0 reads requested" and passing node_is_good().
    pairs = list(reads)
    finite = [(float(a), float(b)) for a, b in pairs
              if math.isfinite(float(a)) and math.isfinite(float(b))]
    n_total = len(pairs)
    n_ok = len(finite)
    if n_ok == 0:
        nan = float('nan')
        return ReadStats(n_ok=0, n_total=n_total, raw_mean_tx=nan,
                         raw_mean_ty=nan, res_tx=nan, res_ty=nan,
                         sd_tx=nan, sd_ty=nan)
    txs = np.array([p[0] for p in finite], dtype=float)
    tys = np.array([p[1] for p in finite], dtype=float)
    sd_tx = float(np.std(txs, ddof=1)) if n_ok >= 2 else 0.0
    sd_ty = float(np.std(tys, ddof=1)) if n_ok >= 2 else 0.0
    raw_tx = float(np.mean(txs))
    raw_ty = float(np.mean(tys))
    return ReadStats(n_ok=n_ok, n_total=n_total,
                     raw_mean_tx=raw_tx, raw_mean_ty=raw_ty,
                     res_tx=raw_tx + offset_rad[0],
                     res_ty=raw_ty + offset_rad[1],
                     sd_tx=sd_tx, sd_ty=sd_ty)


def node_is_good(stats: ReadStats, n_reads_requested: int) -> bool:
    """Whether a node's reads support shipping it in a calibration."""
    if stats.n_ok < MIN_GOOD_READS:
        return False
    return stats.n_ok * 2 >= n_reads_requested


def home_node_verdict(stats: ReadStats) -> Tuple[bool, str]:
    """The stale-level-reference gate. Returns ``(ok, human message)``.

    Both axes are gated independently rather than on the vector magnitude: a
    stale reference usually leans on one axis, and a magnitude check dilutes a
    single-axis error by up to sqrt(2) against the same tolerance.
    """
    if stats.n_ok < MIN_GOOD_READS:
        return (False,
                'home node returned only {}/{} finite reads — the inclinometer '
                'is not answering reliably; fix that before capturing'
                .format(stats.n_ok, stats.n_total))
    problems = []
    for axis, res, sd in (('tx', stats.res_tx, stats.sd_tx),
                          ('ty', stats.res_ty, stats.sd_ty)):
        tol = max(HOME_NODE_SD_MULT * sd, HOME_NODE_TOL_FLOOR_RAD)
        if abs(res) > tol or abs(res) > HOME_NODE_ABS_CEIL_RAD:
            problems.append(
                '{}: |{:+.6f}| rad ({:+.4f} deg) vs tolerance {:.6f} rad '
                '(max({:.1f}x sd {:.6f}, floor {:.6f})), absolute ceiling '
                '{:.6f} rad'
                .format(axis, res, math.degrees(res), tol, HOME_NODE_SD_MULT,
                        sd, HOME_NODE_TOL_FLOOR_RAD, HOME_NODE_ABS_CEIL_RAD))
    if problems:
        return (False,
                'STALE LEVEL REFERENCE — the home node (0, 0) residual is not '
                '~0, which by C-LEVEL-2 it must be by construction. ' +
                '; '.join(problems) +
                '. Most likely: `level` was not run immediately before this '
                'capture, or it was run and then the process that applies it '
                'was relaunched (the correction is per-PROCESS). Re-run '
                '`level` from IDLE and start over. Capturing against a stale '
                'reference corrupts EVERY node identically and the map would '
                'look perfectly plausible.')
    return (True,
            'home node OK: residual ({:+.6f}, {:+.6f}) rad = ({:+.4f}, '
            '{:+.4f}) deg, per-read sd ({:.6f}, {:.6f}) rad'
            .format(stats.res_tx, stats.res_ty,
                    math.degrees(stats.res_tx), math.degrees(stats.res_ty),
                    stats.sd_tx, stats.sd_ty))


# ── pure core: map document ──────────────────────────────────────────────


def build_map_document(x_mm: Sequence[float], y_mm: Sequence[float],
                       z_mm: float,
                       results: Dict[Tuple[int, int], ReadStats],
                       captured: Dict[str, Any],
                       n_reads: int) -> Dict[str, Any]:
    """Assemble the schema-v1 document from the per-node reductions.

    Residuals are rounded to 9 dp (1 nrad — six orders below the sensor's own
    resolution) and axes to 6 dp, purely so the committed YAML is diff-readable
    and byte-stable across re-captures that change nothing. ``map_version``
    float-normalises before hashing, so the rounding cannot churn the version.

    Raises :class:`TiltCalError` naming every missing or unusable node rather
    than emitting NaN — C-LEVEL-2's validation refuses NaN at load, and a
    partial map is the one artefact that looks correct until a ball misses.
    """
    n_x, n_y = len(x_mm), len(y_mm)
    tx_grid: List[List[float]] = []
    ty_grid: List[List[float]] = []
    sd_tx_grid: List[List[float]] = []
    sd_ty_grid: List[List[float]] = []
    missing: List[str] = []

    for iy in range(n_y):
        tx_row, ty_row, sdx_row, sdy_row = [], [], [], []
        for ix in range(n_x):
            stats = results.get((ix, iy))
            if stats is None or not node_is_good(stats, n_reads):
                got = 0 if stats is None else stats.n_ok
                missing.append('({:+.1f}, {:+.1f}) [ix={}, iy={}]: {}/{} '
                               'usable reads'
                               .format(x_mm[ix], y_mm[iy], ix, iy, got,
                                       n_reads))
                tx_row.append(float('nan'))
                ty_row.append(float('nan'))
                sdx_row.append(float('nan'))
                sdy_row.append(float('nan'))
                continue
            tx_row.append(round(stats.res_tx, 9))
            ty_row.append(round(stats.res_ty, 9))
            sdx_row.append(round(stats.sd_tx, 9))
            sdy_row.append(round(stats.sd_ty, 9))
        tx_grid.append(tx_row)
        ty_grid.append(ty_row)
        sd_tx_grid.append(sdx_row)
        sd_ty_grid.append(sdy_row)

    if missing:
        raise TiltCalError(
            'refusing to write a calibration with {} unusable node(s):\n  {}\n'
            'A failed node is never interpolated from its neighbours — that '
            'would invent calibration data at exactly the place the machine '
            'had trouble — and NaN is refused at load by C-LEVEL-2. The CSV '
            'and _meta.json are still written, so re-run the capture (a longer '
            '--dwell-s is the usual fix) and check whether those nodes sit on '
            'a leg stroke clamp.'
            .format(len(missing), '\n  '.join(missing)))

    return {
        'version': tilt_map.SCHEMA_VERSION,
        'captured': dict(captured),
        'grid': {
            'z_mm': round(float(z_mm), 6),
            'orientation': 'level',
            'x_mm': [round(float(v), 6) for v in x_mm],
            'y_mm': [round(float(v), 6) for v in y_mm],
        },
        'residual_rad': {'tx': tx_grid, 'ty': ty_grid},
        'stats': {
            'n_reads': int(n_reads),
            'sd_tx_rad': sd_tx_grid,
            'sd_ty_rad': sd_ty_grid,
            'failed_nodes': [],
        },
    }


def validate_map_document(doc: Dict[str, Any]):
    """Self-validate through the **production loader** before writing.

    Deliberately ``tilt_map.parse_tilt_map`` rather than a re-implemented check
    list: the only property that matters is "the node will accept this file",
    and a second copy of the validation rules is a second thing to drift. Any
    rejection is re-raised as a :class:`TiltCalError` carrying the loader's own
    message plus the likely physical cause, because the operator reading it is
    standing at a robot, not at a schema.
    """
    try:
        return tilt_map.parse_tilt_map(doc)
    except tilt_map.TiltMapError as exc:
        raise TiltCalError(
            'refusing to write — the map this capture produced would be '
            'REJECTED by trajectory_node:\n  {}\n'
            'Likely causes, in order: (a) a stale level reference (was `level` '
            'run from IDLE immediately before the capture, in THIS process?) — '
            'that shifts every node by the same large amount; (b) a leg pinned '
            'on its stroke clamp at an extremity node, which corrupts that node '
            'only; (c) a units error if this tool was edited. The measured '
            'pose-dependent signal is 0.04-0.6 deg; the bound is {:.2f} deg.'
            .format(exc, math.degrees(tilt_map.MAX_ABS_RESIDUAL_RAD)))


def dump_map_yaml(doc: Dict[str, Any]) -> str:
    """Serialise the document with a provenance header comment.

    Block style throughout: one number per line makes ``git diff`` between two
    captures readable node-by-node, which is the whole point of committing this
    file.
    """
    header = (
        '# Pose-dependent residual tilt map — contract C-LEVEL-2\n'
        '# (ros_ws/docs/levelling_frame.md). MACHINE-WRITTEN by\n'
        '# {} — do not hand-edit: the acquisition tool\n'
        '# rewrites this file wholesale on every capture, and a hand-edited\n'
        '# node has no measurement behind it.\n'
        '# An ABSENT file is a silent, non-gating state (C-LEVEL-1 behaviour).\n'
        .format(TOOL_NAME))
    return header + yaml.safe_dump(doc, default_flow_style=False,
                                   sort_keys=False)


def write_target_path(environ: Optional[Dict[str, str]] = None) -> str:
    """Where to write the map — the FIRST candidate of the production resolver.

    Uses ``tilt_map.tilt_map_candidates`` rather than any walk of this file's
    own location. That is not a stylistic preference: the resolver's whole
    reason for inverting ``friction_ff_params``' order is that this tool writes
    the file the node then reloads, and a second, independent notion of "where
    the source tree is" is exactly how the two ends come to disagree while both
    report success. When ``$JUGGLEBOT_TILT_CAL`` is set it is authoritative and
    the only candidate, so this returns it — writing anywhere else would apply a
    different calibration than the operator named.
    """
    candidates = tilt_map.tilt_map_candidates(environ)
    if not candidates:
        raise TiltCalError(
            'no tilt-map path candidates exist: the repo source tree could not '
            'be located from the installed jugglebot package and ament does not '
            'know the package either. Set ${} to an explicit path.'
            .format(tilt_map.TILT_MAP_ENV))
    return candidates[0]


def looks_like_source_tree(path: str) -> bool:
    """Whether ``path`` sits in the repo (``<root>/config/x.yaml``, ``ros_ws``
    beside it) rather than in an ament share/install tree.

    A property of the returned path, checked as a courtesy so the operator is
    told *before* the capture rather than by the post-write version readback —
    which is the actual guarantee and catches this regardless.
    """
    root = os.path.dirname(os.path.dirname(os.path.abspath(path)))
    return os.path.isdir(os.path.join(root, 'ros_ws'))


# ── pure core: CSV rows ──────────────────────────────────────────────────


def csv_row(iso: str, t_s: float, phase: str, visit: int,
            ix: int, iy: int, x_mm: float, y_mm: float, z_mm: float,
            read_index: int, raw_tx: float, raw_ty: float,
            offset_rad: Tuple[float, float],
            uptime_ms: Optional[int], note: str = '') -> Dict[str, Any]:
    """One per-read CSV row. ``ix``/``iy`` are ``-1`` for verification poses.

    ``res_*`` are recorded per read (not only per node) so the analyser can
    recompute the reduction, and so a re-analysis is never hostage to this
    tool's averaging choices.
    """
    ok = math.isfinite(raw_tx) and math.isfinite(raw_ty)
    return {
        'iso': iso,
        't_s': round(float(t_s), 3),
        'phase': phase,
        'visit': int(visit),
        'ix': int(ix),
        'iy': int(iy),
        'x_mm': round(float(x_mm), 3),
        'y_mm': round(float(y_mm), 3),
        'z_mm': round(float(z_mm), 3),
        'read_index': int(read_index),
        'raw_tx_rad': raw_tx,
        'raw_ty_rad': raw_ty,
        'res_tx_rad': (raw_tx + offset_rad[0]) if ok else float('nan'),
        'res_ty_rad': (raw_ty + offset_rad[1]) if ok else float('nan'),
        'ok': 1 if ok else 0,
        'uptime_ms': '' if uptime_ms is None else int(uptime_ms),
        'note': note,
    }


def estimate_duration_s(n_nodes: int, n_checks: int, move_s: float,
                        dwell_s: float, n_reads: int,
                        read_gap_s: float) -> float:
    """Wall-clock ETA for the dry-run plan (includes return-to-centre)."""
    per_visit = move_s + dwell_s + n_reads * (EST_READ_S + read_gap_s)
    return (n_nodes + n_checks + 1) * per_visit


def git_sha() -> str:
    """HEAD SHA for provenance, or ``'unknown'`` — never raises."""
    try:
        out = subprocess.run(
            ['git', 'rev-parse', 'HEAD'],
            cwd=os.path.dirname(os.path.abspath(__file__)),
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=True)
        return out.stdout.decode().strip()
    except Exception:
        return 'unknown'


# ── CLI ──────────────────────────────────────────────────────────────────


class _Parser(argparse.ArgumentParser):
    """``ArgumentParser`` that explains the one error an operator will hit.

    ``--x -150,-75,0,75,150`` fails, because argparse only treats a leading
    ``-`` as a value when the token is a bare negative *number*, and a
    comma-separated list is not. The failure message ("expected one argument")
    gives no hint about the fix, and this is the tool's own documented usage
    form — so it is answered here rather than left to be rediscovered at the
    robot.
    """

    def error(self, message):
        if 'expected one argument' in message and (
                '--x' in message or '--y' in message):
            message += (
                "\n\nA node list that starts with a negative number needs the "
                "'=' form, or argparse reads it as another option:\n"
                "    --x=-150,-75,0,75,150      (NOT  --x -150,-75,0,75,150)\n"
                "A list starting with a positive number is fine either way.")
        super().error(message)


def build_parser() -> argparse.ArgumentParser:
    ap = _Parser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='Runbook: tests/hardware/session_tilt_calibration.md')
    grid = ap.add_argument_group('grid spec')
    grid.add_argument('--x', default=None,
                      help='explicit x node list in mm (comma/space separated, '
                           'strictly increasing), e.g. '
                           '--x=-150,-75,0,75,150. Use the "=" form when the '
                           'list starts negative. Overrides --box/--nodes.')
    grid.add_argument('--y', default=None,
                      help='explicit y node list in mm, e.g. --y=-150,0,150. '
                           'Overrides --box/--nodes.')
    grid.add_argument('--box', type=float, default=DEFAULT_BOX_MM,
                      help='half-width in mm of a symmetric square grid '
                           '(default %(default)s).')
    grid.add_argument('--nodes', type=int, default=DEFAULT_NODES,
                      help='nodes per axis for --box (default %(default)s; must '
                           'be odd so (0,0) is a node — the home gate).')
    grid.add_argument('--z', type=float, default=DEFAULT_Z_MM,
                      help='capture height in mm (default %(default)s). z is a '
                           'scalar by design: a z sweep is a config change.')

    timing = ap.add_argument_group('timing (PROVISIONAL defaults — rung C0 pins them)')
    timing.add_argument('--dwell-s', type=float, default=DEFAULT_DWELL_S,
                        help='settle time after the move completes, before the '
                             'first read (default %(default)s).')
    timing.add_argument('--n-reads', type=int, default=DEFAULT_N_READS,
                        help='sequential tilt reads per pose (default '
                             '%(default)s). Never concurrent — the relay is '
                             'serialized.')
    timing.add_argument('--read-gap-s', type=float, default=DEFAULT_READ_GAP_S,
                        help='gap between reads (default %(default)s). Must '
                             'exceed the SCL3300 10 Hz LPF period or the '
                             'per-read sd measures the filter, not the sensor.')
    timing.add_argument('--move-duration-s', type=float,
                        default=DEFAULT_MOVE_DURATION_S,
                        help='commanded duration for every go_to_pose (default '
                             '%(default)s — deliberately slow; see the module '
                             'docstring before shortening it).')
    timing.add_argument('--lean-gain', type=float, default=0.0,
                        help='lean gain passed explicitly on every move '
                             '(default %(default)s = OFF).')
    timing.add_argument('--timeout-s', type=float, default=DEFAULT_TIMEOUT_S,
                        help='per-service-call timeout (default %(default)s).')

    ver = ap.add_argument_group('verification')
    ver.add_argument('--check-poses', type=int, default=DEFAULT_CHECK_POSES,
                     help='off-node poses re-measured after the map is applied '
                          '(default %(default)s).')
    ver.add_argument('--threshold-deg', type=float,
                     default=DEFAULT_THRESHOLD_DEG,
                     help='PASS bound on each check pose residual magnitude '
                          '(default %(default)s — PROVISIONAL until C0).')
    ver.add_argument('--verify-only', action='store_true',
                     help='skip the capture: re-measure the check poses against '
                          'the ALREADY-LOADED map and report PASS/FAIL. Grid '
                          'axes are read from the loaded map file so the poses '
                          'are identical to the capture run (rung C2).')

    misc = ap.add_argument_group('workflow')
    misc.add_argument('--on-fail', choices=('continue', 'abort'),
                      default='continue',
                      help='node read failure policy (default %(default)s). '
                           'abort returns to the centre node first, then exits '
                           'nonzero.')
    misc.add_argument('--base-condition', default=None,
                      help='free text recorded in captured.base_condition, e.g. '
                           '"flat floor, no shims" or "2 deg shim under leg 1". '
                           'Prompted for if omitted.')
    misc.add_argument('--no-apply', action='store_true',
                      help='capture and write the CSV/meta, but do NOT write '
                           'the map, reload, or verify. The map is still '
                           'assembled and validated, and the report says '
                           'whether it WOULD be accepted. This is rung C0: a '
                           'read-noise probe must not leave a calibration '
                           'behind.')
    misc.add_argument('--force-uninstall', action='store_true',
                      help='if a map is loaded, move the file aside (timestamped '
                           '.bak) and reload so the capture starts with '
                           'tilt_map_loaded=false.')
    misc.add_argument('--dry-run', action='store_true',
                      help='print the node sequence and ETA; make ZERO ROS '
                           'calls and construct no ROS objects.')
    misc.add_argument('--yes', action='store_true',
                      help='skip the interactive operator confirmations '
                           '(rehearsal only — the confirmations are the fresh-'
                           'level and hand-quiescent preconditions).')
    misc.add_argument('--out-dir', default=None,
                      help='CSV/meta output directory (default temp/logs/).')
    return ap


def resolve_grid(args) -> Tuple[List[float], List[float]]:
    """Grid axes from ``--x/--y`` if given, else ``--box/--nodes``."""
    x_mm = (parse_node_list(args.x, 'x') if args.x
            else build_axis(args.box, args.nodes, 'x'))
    y_mm = (parse_node_list(args.y, 'y') if args.y
            else build_axis(args.box, args.nodes, 'y'))
    home_index(x_mm, y_mm)   # refuse early if (0, 0) is not a node
    return x_mm, y_mm


def print_plan(x_mm: Sequence[float], y_mm: Sequence[float], z_mm: float,
               order: Sequence[Node], checks: Sequence[Tuple[float, float]],
               args) -> None:
    print('Tilt-calibration grid — {}x{} nodes at z={} mm'
          .format(len(x_mm), len(y_mm), z_mm))
    print('  x_mm: {}'.format(list(x_mm)))
    print('  y_mm: {}'.format(list(y_mm)))
    print('  move={:.2f}s  dwell={:.2f}s  n_reads={}  read_gap={:.2f}s  '
          'lean_gain={}'.format(args.move_duration_s, args.dwell_s,
                                args.n_reads, args.read_gap_s, args.lean_gain))
    print('  visit order (home node FIRST, then serpentine):')
    for i, n in enumerate(order):
        tag = '  <- home node (stale-level gate)' if i == 0 else ''
        print('    {:>3d}. ({:+7.1f}, {:+7.1f}) [ix={}, iy={}]{}'
              .format(i + 1, n.x_mm, n.y_mm, n.ix, n.iy, tag))
    print('  verification check poses (off-node, cell centres):')
    for i, (cx, cy) in enumerate(checks):
        print('    {:>3d}. ({:+7.1f}, {:+7.1f})'.format(i + 1, cx, cy))
    eta = estimate_duration_s(len(order), len(checks), args.move_duration_s,
                              args.dwell_s, args.n_reads, args.read_gap_s)
    print('  ETA ~ {:.0f} s ({:.1f} min) including the return to centre'
          .format(eta, eta / 60.0))


# ── runtime (ROS) ────────────────────────────────────────────────────────


class _Runner:
    """Thin rclpy client: two services, three subscriptions, no state machine.

    Everything actuating goes through ``trajectory/go_to_pose``. There is no
    arming call, no mode change and no hand command anywhere in this class, and
    there must never be one — see the module docstring.
    """

    def __init__(self, node, args):
        from jugglebot_interfaces.srv import GoToPose, GetTiltReadingService
        from jugglebot_interfaces.msg import TrajectoryStatus, RobotState
        from diagnostic_msgs.msg import DiagnosticStatus
        from std_srvs.srv import Trigger

        self.node = node
        self.args = args
        self.status = None
        self.status_stamp = 0.0
        self.link_kv = {}
        self.uptime_first = None
        self.uptime_last = None
        self.pose_offset_rad = None

        self._GoToPose = GoToPose
        self._Tilt = GetTiltReadingService
        self._Trigger = Trigger

        self.cli_go = node.create_client(GoToPose, '/trajectory/go_to_pose')
        self.cli_tilt = node.create_client(GetTiltReadingService,
                                           '/get_platform_tilt')
        self.cli_reload = node.create_client(Trigger,
                                             '/trajectory/reload_tilt_map')
        node.create_subscription(TrajectoryStatus, '/trajectory/status',
                                 self._on_status, 10)
        node.create_subscription(DiagnosticStatus, '/link_status',
                                 self._on_link, 10)
        node.create_subscription(RobotState, '/robot_state',
                                 self._on_robot_state, 10)

    # -- subscriptions ----------------------------------------------------

    def _on_status(self, msg):
        self.status = msg
        self.status_stamp = time.time()

    def _on_link(self, msg):
        kv = {v.key: v.value for v in msg.values}
        self.link_kv = kv
        raw = kv.get('uptime_ms')
        if raw is not None:
            try:
                uptime = int(raw)
            except ValueError:
                return
            if self.uptime_first is None:
                self.uptime_first = uptime
            self.uptime_last = uptime

    def _on_robot_state(self, msg):
        data = list(getattr(msg, 'pose_offset_rad', []) or [])
        if len(data) >= 2:
            self.pose_offset_rad = [float(data[0]), float(data[1])]

    # -- plumbing ---------------------------------------------------------

    def spin(self, seconds: float) -> None:
        import rclpy
        end = time.time() + max(0.0, seconds)
        while True:
            remaining = end - time.time()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self.node, timeout_sec=min(0.05, remaining))

    def call(self, client, request, name: str):
        import rclpy
        if not client.wait_for_service(timeout_sec=self.args.timeout_s):
            raise TiltCalError(
                'service {} unavailable after {:.1f} s — is the stack up '
                '(ros2 launch jugglebot jugglebot_launch.py), and was '
                'jugglebot rebuilt?'.format(name, self.args.timeout_s))
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future,
                                         timeout_sec=self.args.timeout_s)
        result = future.result()
        if result is None:
            raise TiltCalError(
                'service {} did not answer within {:.1f} s'
                .format(name, self.args.timeout_s))
        return result

    def wait_for_status(self, timeout_s: float = 5.0):
        self.status = None
        deadline = time.time() + timeout_s
        while time.time() < deadline and self.status is None:
            self.spin(0.1)
        if self.status is None:
            raise TiltCalError(
                'no /trajectory/status message in {:.1f} s — trajectory_node is '
                'not running, or jugglebot_interfaces was not rebuilt (a stale '
                'install makes trajectory_node exit ~200 ms after launch). '
                'Build BOTH packages: colcon build --packages-select '
                'jugglebot_interfaces jugglebot'.format(timeout_s))
        return self.status

    # -- actions ----------------------------------------------------------

    def go_to(self, x_mm: float, y_mm: float, z_mm: float, label: str) -> float:
        """Request a level-orientation move; return the planned duration.

        The orientation is the identity quaternion — the *commanded* level
        orientation. What the platform physically does with it depends on the
        correction ``trajectory_node`` composes, which is the entire quantity
        being calibrated.
        """
        from geometry_msgs.msg import Pose, Point, Quaternion
        request = self._GoToPose.Request()
        request.pose = Pose(
            position=Point(x=float(x_mm), y=float(y_mm), z=float(z_mm)),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        request.duration_s = float(self.args.move_duration_s)
        request.lean_gain = float(self.args.lean_gain)
        response = self.call(self.cli_go, request, 'trajectory/go_to_pose')
        if not response.accepted:
            raise TiltCalError(
                'go_to_pose REJECTED for {} at ({:+.1f}, {:+.1f}, {:.1f}): '
                'code={} :: {}'.format(label, x_mm, y_mm, z_mm,
                                       response.code, response.message))
        return float(response.planned_duration_s)

    def read_tilt_once(self) -> Tuple[float, float]:
        """One sequential ``get_platform_tilt``. NaN pair on a failed read."""
        response = self.call(self.cli_tilt, self._Tilt.Request(),
                             'get_platform_tilt')
        data = list(response.tilt_xy)
        if len(data) < 2:
            return (float('nan'), float('nan'))
        return (float(data[0]), float(data[1]))

    def measure(self, x_mm: float, y_mm: float, z_mm: float, label: str,
                offset_rad: Tuple[float, float]
                ) -> Tuple[ReadStats, List[Tuple[float, float]]]:
        """Move, dwell, then N spaced sequential reads. Returns (stats, raws)."""
        planned = self.go_to(x_mm, y_mm, z_mm, label)
        self.spin(planned + self.args.dwell_s)
        raws: List[Tuple[float, float]] = []
        for i in range(self.args.n_reads):
            if i:
                self.spin(self.args.read_gap_s)
            raws.append(self.read_tilt_once())
        return residual_from_reads(raws, offset_rad), raws


def _fmt_deg(tx: float, ty: float) -> str:
    return '({:+.4f}, {:+.4f}) deg'.format(math.degrees(tx), math.degrees(ty))


def _magnitude_deg(tx: float, ty: float) -> float:
    return math.degrees(math.hypot(tx, ty))


def run(args) -> int:                                    # noqa: C901
    """The ROS half. Imports rclpy lazily so the pure core stays importable."""
    import rclpy

    offset_rad = mounting_offset_rad()
    out_dir = args.out_dir or os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__)))), 'temp', 'logs')
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(out_dir, 'tilt_cal_grid_{}.csv'.format(stamp))
    meta_path = os.path.join(out_dir, 'tilt_cal_grid_{}_meta.json'.format(stamp))

    # Bring the ROS context up under its own guard: _Runner's interface
    # imports are the likeliest early failure (a stale or unsourced install),
    # and leaking an initialised context plus a live node there would leave the
    # next run to inherit them.
    rclpy.init()
    try:
        node = rclpy.create_node('tilt_cal_grid')
        runner = _Runner(node, args)
    except ImportError as exc:
        rclpy.shutdown()
        raise TiltCalError(
            'interface import failed ({}). Did you run  colcon build '
            '--packages-select jugglebot_interfaces jugglebot  and  source '
            'ros_ws/install/setup.bash ?'.format(exc))
    except BaseException:
        rclpy.shutdown()
        raise

    started = time.time()
    rc = 0
    node_summaries: List[Dict[str, Any]] = []
    check_summaries: List[Dict[str, Any]] = []
    reached_motion = False
    z_mm = float(args.z)

    try:
        csv_file = open(csv_path, 'w', newline='')
    except BaseException:
        node.destroy_node()
        rclpy.shutdown()
        raise
    writer = csv.DictWriter(csv_file, fieldnames=list(CSV_COLUMNS))
    writer.writeheader()

    def emit(row: Dict[str, Any]) -> None:
        writer.writerow(row)
        csv_file.flush()

    def record(phase: str, visit: int, ix: int, iy: int, x: float, y: float,
               raws: Sequence[Tuple[float, float]], note: str = '') -> None:
        now_iso = datetime.now().isoformat(timespec='milliseconds')
        for k, (rtx, rty) in enumerate(raws):
            emit(csv_row(now_iso, time.time() - started, phase, visit, ix, iy,
                         x, y, z_mm, k, rtx, rty, offset_rad,
                         runner.uptime_last, note))

    try:
        # ── preflight ────────────────────────────────────────────────
        print('== preflight ==')
        status = runner.wait_for_status()
        runner.spin(1.0)   # let /link_status and /robot_state arrive too

        if runner.uptime_last is None:
            print('  ! no uptime_ms on /link_status yet — the can-bridge '
                  'heartbeat may not have arrived. Continuing.')
        else:
            hours = runner.uptime_last / 3600000.0
            flag = ' <-- LARGE' if runner.uptime_last > UPTIME_WARN_MS else ''
            print('  can-bridge uptime_ms = {} ({:.2f} h){}'
                  .format(runner.uptime_last, hours, flag))
            if runner.uptime_last > UPTIME_WARN_MS:
                print('    Tracking lag is known to grow with can-bridge '
                      'uptime (10 ms fresh -> ~240 ms at 30 h). Capture wants '
                      'a freshly power-cycled bridge; quote this number with '
                      'every result.')

        if status.mode != 'TRAJECTORY' or not status.streaming:
            raise TiltCalError(
                'platform is not armed into the streaming hold: '
                'trajectory/status mode={!r} streaming={}. This tool never '
                'arms and never changes mode — bring the machine to '
                'ACTIVE:TRAJECTORY with the 40 Hz hold stream first, then '
                're-run.'.format(status.mode, status.streaming))
        print('  mode=TRAJECTORY streaming=True')

        if not status.gravity_correction_loaded:
            raise TiltCalError(
                'trajectory/status gravity_correction_loaded=false — there is '
                'no level reference to measure residuals against, and every '
                'residual is DEFINED relative to it. Run `level` from IDLE '
                '(over /orchestrator_command, not the /activate service), then '
                're-run. Note the correction is per-PROCESS: a relaunch drops '
                'it silently.')
        print('  gravity_correction_loaded=True')

        if args.verify_only:
            if not status.tilt_map_loaded:
                raise TiltCalError(
                    '--verify-only re-measures an ALREADY-APPLIED map, but '
                    'trajectory/status tilt_map_loaded=false. Load a map '
                    '(capture one, or restore config/tilt_calibration.yaml and '
                    'call trajectory/reload_tilt_map) first.')
            print('  tilt_map_loaded=True version={}'
                  .format(status.tilt_map_version))
        else:
            if status.tilt_map_loaded and args.force_uninstall:
                target = write_target_path()
                if os.path.exists(target):
                    backup = '{}.{}.bak'.format(target, stamp)
                    os.rename(target, backup)
                    print('  moved {} -> {}'.format(target, backup))
                else:
                    print('  no file at {} to move aside'.format(target))
                response = runner.call(runner.cli_reload,
                                       runner._Trigger.Request(),
                                       'trajectory/reload_tilt_map')
                print('  reload: success={} :: {}'
                      .format(response.success, response.message))
                if not response.success:
                    raise TiltCalError(
                        'reload after --force-uninstall returned success=false: '
                        '{} (an absent file must UNLOAD and succeed)'
                        .format(response.message))
                status = runner.wait_for_status()
                if status.tilt_map_loaded:
                    raise TiltCalError(
                        'map is STILL loaded after --force-uninstall (version '
                        '{!r}) — do not capture: the loaded map would bake '
                        'itself into its own successor.'
                        .format(status.tilt_map_version))
                print('  tilt_map_loaded=False (uninstalled)')
            elif status.tilt_map_loaded:
                raise TiltCalError(
                    'a tilt map is loaded (version {!r}) and capture must run '
                    'with NO map: residuals are defined against the '
                    'single-offset reference, so capturing under a loaded map '
                    'bakes that map into its own successor and the error '
                    'compounds silently on every recapture. Re-run with '
                    '--force-uninstall to move the file aside and reload.'
                    .format(status.tilt_map_version))
            else:
                print('  tilt_map_loaded=False')

        if runner.pose_offset_rad is None:
            print('  ! /robot_state.pose_offset_rad not seen — recording the '
                  'level offset as "unknown" in the map metadata.')
        else:
            print('  level offset (Teensy-persisted, 1 mrad quantised) = '
                  '{} rad'.format(runner.pose_offset_rad))

        # Resolve (and sanity-check) the write target NOW, not after a
        # four-minute sweep. A detached deployment resolves the ament share
        # copy as its first candidate, and discovering that at write time costs
        # the operator the whole capture.
        if not args.verify_only and not args.no_apply:
            target = write_target_path()
            if not looks_like_source_tree(target) \
                    and not os.environ.get(tilt_map.TILT_MAP_ENV):
                raise TiltCalError(
                    'the resolved write target {} is not in the repo source '
                    'tree (no ros_ws/ beside its config/ directory). Writing '
                    'into an ament share/install tree would be silently undone '
                    'by the next colcon build. Set ${} if this is deliberate.'
                    .format(target, tilt_map.TILT_MAP_ENV))
            print('  write target {}'.format(target))

        # The home-node residual is ~0 by construction only at the pose `level`
        # itself measured — the firmware ACTIVATE pose, z = 170. At any other
        # height a nonzero home residual is a REAL pose-dependent residual, and
        # the gate below would misreport it as a stale level reference and send
        # the operator re-levelling over a cause that is not the cause.
        if not args.verify_only and abs(float(args.z) - DEFAULT_Z_MM) > 1e-6:
            print('  ! --z {} is not the ACTIVATE height {}: the home-node gate '
                  'assumes the level reference was measured at this pose. A '
                  'nonzero home residual here is REAL, not stale — read a gate '
                  'failure accordingly.'.format(args.z, DEFAULT_Z_MM))

        base_condition = args.base_condition
        if args.yes and not args.verify_only:
            print('  ! --yes: SKIPPING the fresh-`level` and hand-quiescent '
                  'confirmations on a LIVE capture. Those two capture '
                  'preconditions have no programmatic check — the home-node '
                  'gate catches only a grossly stale reference, and nothing '
                  'at all observes the hand.')
        if not args.yes:
            thc.safety_gate(
                'Was `level` run from IDLE in THIS session, with THIS running '
                'trajectory_node process (no relaunch since)?\n'
                '  Every residual below is measured relative to that '
                'reference. A stale one corrupts all of them identically and '
                'the map still looks plausible.', 'yes')
            thc.safety_gate(
                'Is the HAND quiescent (no hand move in flight, no ball op '
                'running)?\n'
                '  Tilt reads block the Platform-Teensy loop that streams hand '
                'moves.', 'yes')
            if base_condition is None:
                print('\nDescribe the base condition for the record (e.g. '
                      '"flat floor, no shims" / "2 deg shim under leg 1"):')
                base_condition = input('> ').strip()
        if not base_condition:
            base_condition = 'unspecified'

        # ── grid ─────────────────────────────────────────────────────
        if args.verify_only:
            map_path = tilt_map.resolve_tilt_map_path()
            if map_path is None:
                raise TiltCalError(
                    '--verify-only needs the loaded map file to reproduce the '
                    'same check poses, but no file exists at any candidate '
                    'path: {}'.format('; '.join(tilt_map.tilt_map_candidates())
                                      or '<none>'))
            loaded = tilt_map.load_tilt_map(map_path)
            x_mm = [float(v) for v in loaded.x_mm]
            y_mm = [float(v) for v in loaded.y_mm]
            z_mm = float(loaded.z_mm) if loaded.z_mm is not None else float(args.z)
            print('  check poses derived from {} (version {}, z={} mm)'
                  .format(map_path, loaded.version, z_mm))
            # The axes come from the map, but --check-poses comes from THIS
            # invocation — and rung C2a's whole premise is re-measuring the
            # same poses. Compare against the capture's own recorded argv.
            captured_args = loaded.metadata.get('args') or []
            if '--check-poses' in captured_args:
                try:
                    original = int(
                        captured_args[captured_args.index('--check-poses') + 1])
                except (IndexError, ValueError):
                    original = None
                if original is not None and original != args.check_poses:
                    print('  ! this map was captured with --check-poses {} but '
                          'you passed {}. Rung C2a compares the SAME poses; '
                          'pass --check-poses {} to reproduce them.'
                          .format(original, args.check_poses, original))
            order: List[Node] = []
        else:
            x_mm, y_mm = resolve_grid(args)
            order = visit_order(x_mm, y_mm)
        checks = check_poses(x_mm, y_mm, args.check_poses)

        results: Dict[Tuple[int, int], ReadStats] = {}
        failed_nodes: List[Tuple[int, int]] = []
        reached_motion = True

        # ── capture ──────────────────────────────────────────────────
        if order:
            print('\n== capture ({} nodes) =='.format(len(order)))
        for visit, nd in enumerate(order):
            label = 'node {}/{} ({:+.1f}, {:+.1f})'.format(
                visit + 1, len(order), nd.x_mm, nd.y_mm)
            try:
                stats, raws = runner.measure(nd.x_mm, nd.y_mm, z_mm, label,
                                             offset_rad)
            except TiltCalError as exc:
                print('  [FAIL] {} :: {}'.format(label, exc))
                record('capture', visit, nd.ix, nd.iy, nd.x_mm, nd.y_mm,
                       [(float('nan'), float('nan'))], note=str(exc)[:200])
                failed_nodes.append((nd.ix, nd.iy))
                # A home-node failure is fatal whatever --on-fail says: the
                # home node IS the stale-level gate, and `continue` would jump
                # past it and spend four more minutes capturing a grid that
                # build_map_document will refuse anyway for the missing node.
                # Aborting early is the entire reason home is measured first.
                if visit == 0 or args.on_fail == 'abort':
                    raise
                rc = 1
                continue

            record('capture', visit, nd.ix, nd.iy, nd.x_mm, nd.y_mm, raws)
            results[(nd.ix, nd.iy)] = stats
            # Appended per node, not in a trailing loop: on the home-gate abort
            # — the runbook's headline ABORT criterion — a trailing loop never
            # runs and _meta.json ships an empty 'nodes', exactly when the
            # operator was told to retain the per-node reduction.
            node_summaries.append({
                'ix': nd.ix, 'iy': nd.iy,
                'x_mm': nd.x_mm, 'y_mm': nd.y_mm,
                'res_tx_rad': stats.res_tx, 'res_ty_rad': stats.res_ty,
                'sd_tx_rad': stats.sd_tx, 'sd_ty_rad': stats.sd_ty,
                'n_ok': stats.n_ok, 'n_total': stats.n_total,
                'good': node_is_good(stats, args.n_reads),
            })
            good = node_is_good(stats, args.n_reads)
            print('  [{}] {} residual {} sd ({:.6f}, {:.6f}) rad  reads {}/{}'
                  .format('OK  ' if good else 'FAIL', label,
                          _fmt_deg(stats.res_tx, stats.res_ty),
                          stats.sd_tx, stats.sd_ty, stats.n_ok, stats.n_total))
            if not good:
                failed_nodes.append((nd.ix, nd.iy))
                if args.on_fail == 'abort':
                    raise TiltCalError(
                        '{} produced only {}/{} usable reads and --on-fail='
                        'abort'.format(label, stats.n_ok, stats.n_total))
                rc = 1

            if visit == 0:
                ok, message = home_node_verdict(stats)
                print('  home gate: {}'.format(message))
                if not ok:
                    raise TiltCalError(message)

        # ── write + apply ────────────────────────────────────────────
        if not args.verify_only:
            captured = {
                'date': datetime.now().strftime('%Y-%m-%d'),
                'git_sha': git_sha(),
                'tool': TOOL_NAME,
                'args': list(sys.argv[1:]),
                'uptime_ms_first': runner.uptime_first,
                'uptime_ms_last': runner.uptime_last,
                'level_offset_rad': (runner.pose_offset_rad
                                     if runner.pose_offset_rad is not None
                                     else 'unknown'),
                'level_offset_source': ('robot_state.pose_offset_rad '
                                        '(Teensy-persisted, 1 mrad quantised)'),
                'base_condition': base_condition,
            }
            doc = build_map_document(x_mm, y_mm, z_mm, results, captured,
                                     args.n_reads)
            validate_map_document(doc)
            expected_version = tilt_map.map_version(doc)

            if args.no_apply:
                # Rung C0: a read-noise probe must not leave a calibration
                # behind. The document is still assembled and validated, so the
                # operator learns the capture is sound without committing it.
                print('\n== apply SKIPPED (--no-apply) ==')
                print('  the map WOULD be accepted: version {} grid {}x{}'
                      .format(expected_version, len(x_mm), len(y_mm)))
                print('  nothing written, nothing reloaded, no verification '
                      'pass. Per-node sd is in the CSV and _meta.json.')
                checks = []
            else:
                target = write_target_path()
                if not looks_like_source_tree(target) \
                        and not os.environ.get(tilt_map.TILT_MAP_ENV):
                    raise TiltCalError(
                        'the resolved write target {} is not in the repo source '
                        'tree (no ros_ws/ beside its config/ directory). '
                        'Writing into an ament share/install tree would be '
                        'silently undone by the next colcon build. Set ${} if '
                        'this is deliberate.'
                        .format(target, tilt_map.TILT_MAP_ENV))
                with open(target, 'w') as handle:
                    handle.write(dump_map_yaml(doc))
                print('\n== apply ==')
                print('  wrote {}'.format(target))
                print('  expected version {}'.format(expected_version))

                response = runner.call(runner.cli_reload,
                                       runner._Trigger.Request(),
                                       'trajectory/reload_tilt_map')
                print('  reload: success={} :: {}'
                      .format(response.success, response.message))
                if not response.success:
                    raise TiltCalError(
                        'trajectory_node REJECTED the map this tool just '
                        'wrote: {}'.format(response.message))

                status = runner.wait_for_status()
                if not status.tilt_map_loaded:
                    raise TiltCalError(
                        'reload reported success but trajectory/status says '
                        'tilt_map_loaded=false — nothing is applied.')
                if status.tilt_map_version != expected_version:
                    raise TiltCalError(
                        'APPLIED THE WRONG FILE. trajectory_node reports '
                        'version {!r} but this tool wrote {!r} to {}. The node '
                        'resolved a different candidate path than this tool '
                        'wrote to — check ${} on both sides, and whether an '
                        'ament share copy is shadowing the source tree.'
                        .format(status.tilt_map_version, expected_version,
                                target, tilt_map.TILT_MAP_ENV))
                print('  applied and CONFIRMED: version {}'
                      .format(status.tilt_map_version))

        # ── verification pass ────────────────────────────────────────
        print('\n== verification ({} off-node check poses, threshold {:.3f} '
              'deg) =='.format(len(checks), args.threshold_deg))
        n_fail = 0
        for i, (cx, cy) in enumerate(checks):
            label = 'check {}/{} ({:+.1f}, {:+.1f})'.format(
                i + 1, len(checks), cx, cy)
            try:
                stats, raws = runner.measure(cx, cy, z_mm, label, offset_rad)
            except TiltCalError as exc:
                print('  [FAIL] {} :: {}'.format(label, exc))
                record('verify', i, -1, -1, cx, cy,
                       [(float('nan'), float('nan'))], note=str(exc)[:200])
                n_fail += 1
                check_summaries.append({
                    'x_mm': cx, 'y_mm': cy, 'verdict': 'FAIL',
                    'error': str(exc)[:200], 'n_ok': 0,
                })
                if args.on_fail == 'abort':
                    raise
                continue
            record('verify', i, -1, -1, cx, cy, raws)
            if not node_is_good(stats, args.n_reads):
                print('  [FAIL] {} only {}/{} usable reads'
                      .format(label, stats.n_ok, stats.n_total))
                n_fail += 1
                check_summaries.append({
                    'x_mm': cx, 'y_mm': cy, 'verdict': 'FAIL',
                    'error': 'only {}/{} usable reads'.format(
                        stats.n_ok, stats.n_total),
                    'n_ok': stats.n_ok,
                })
                continue
            magnitude = _magnitude_deg(stats.res_tx, stats.res_ty)
            verdict = 'PASS' if magnitude <= args.threshold_deg else 'FAIL'
            if verdict == 'FAIL':
                n_fail += 1
            print('  [{}] {} residual {} |r|={:.4f} deg  sd ({:.6f}, {:.6f})'
                  .format(verdict, label, _fmt_deg(stats.res_tx, stats.res_ty),
                          magnitude, stats.sd_tx, stats.sd_ty))
            check_summaries.append({
                'x_mm': cx, 'y_mm': cy,
                'res_tx_rad': stats.res_tx, 'res_ty_rad': stats.res_ty,
                'magnitude_deg': magnitude, 'verdict': verdict,
                'sd_tx_rad': stats.sd_tx, 'sd_ty_rad': stats.sd_ty,
                'n_ok': stats.n_ok,
            })
        if n_fail:
            rc = 1
            print('\n  {} of {} check poses FAILED the {:.3f} deg threshold.'
                  .format(n_fail, len(checks), args.threshold_deg))
        elif checks:
            print('\n  all {} check poses PASS.'.format(len(checks)))

    except TiltCalError as exc:
        print('\nABORT: {}'.format(exc), file=sys.stderr)
        rc = 2
    except KeyboardInterrupt:
        print('\nABORT: interrupted by operator.', file=sys.stderr)
        rc = 130
    finally:
        # Never park displaced — this runs on every path, including aborts.
        #
        # `except BaseException`, deliberately: a plain `except Exception` does
        # NOT catch KeyboardInterrupt, and the second Ctrl-C is the reflex when
        # a program does not die on the first. The return-to-centre blocks for
        # up to --timeout-s + the move, so that second interrupt lands squarely
        # inside this block; with `Exception` it propagated straight out of the
        # `finally`, skipping the artefact writes AND the shutdown, and left the
        # platform at a raised displaced pose with no message at all. Aborting
        # the return has to be LOUD — that is the whole contract of this block.
        if reached_motion:
            try:
                print('\nreturning to centre (0, 0, {:.1f})...'.format(z_mm))
                planned = runner.go_to(0.0, 0.0, z_mm, 'return to centre')
                runner.spin(planned + 0.5)
                print('  at centre.')
            except BaseException as exc:                 # noqa: BLE001
                print('  !! RETURN TO CENTRE FAILED: {}: {}\n'
                      '     The platform may be parked at a displaced pose — '
                      'bring it home manually before deactivating.'
                      .format(type(exc).__name__, exc), file=sys.stderr)
                rc = rc or 2
        try:
            meta = {
                'tool': TOOL_NAME,
                'git_sha': git_sha(),
                'argv': sys.argv,
                'started_iso': datetime.fromtimestamp(started).isoformat(),
                'ended_iso': datetime.now().isoformat(),
                'duration_s': round(time.time() - started, 1),
                'csv': csv_path,
                'uptime_ms_first': runner.uptime_first,
                'uptime_ms_last': runner.uptime_last,
                'level_offset_rad': runner.pose_offset_rad,
                'inclinometer_offset_deg': list(
                    hw.JB_OP_INCLINOMETER_OFFSET_DEG),
                'n_reads': args.n_reads,
                'dwell_s': args.dwell_s,
                'read_gap_s': args.read_gap_s,
                'move_duration_s': args.move_duration_s,
                'threshold_deg': args.threshold_deg,
                'verify_only': bool(args.verify_only),
                'no_apply': bool(args.no_apply),
                'nodes': node_summaries,
                'checks': check_summaries,
                'exit_code': rc,
            }
            with open(meta_path, 'w') as handle:
                json.dump(meta, handle, indent=2, default=str)
            csv_file.close()
            print('\nCSV  -> {}'.format(csv_path))
            print('meta -> {}'.format(meta_path))
            print('analyse with: python tools/tilt_cal_analyse.py --csv {}'
                  .format(csv_path))
        finally:
            # The rclpy context must come down even if the artefact write
            # failed or was interrupted, or the next run inherits a live node.
            node.destroy_node()
            rclpy.shutdown()
    return rc


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.verify_only and args.no_apply:
        print('ABORT: --verify-only and --no-apply are contradictory — '
              '--verify-only captures nothing and applies nothing already.',
              file=sys.stderr)
        return 2
    try:
        if args.verify_only:
            if args.dry_run:
                print('--verify-only --dry-run: check poses are read from the '
                      'loaded map at run time, so there is nothing to preview '
                      'offline.')
                return 0
        else:
            x_mm, y_mm = resolve_grid(args)
            order = visit_order(x_mm, y_mm)
            checks = check_poses(x_mm, y_mm, args.check_poses)
            if args.dry_run:
                print_plan(x_mm, y_mm, float(args.z), order, checks, args)
                print('\ndry-run: no ROS calls made, no ROS objects '
                      'constructed.')
                return 0
        # Inside the handler: run()'s own prologue (the mounting-offset read,
        # the ROS context bring-up) raises TiltCalError too, and an operator
        # standing at the robot should get this tool's ABORT line rather than a
        # traceback.
        return run(args)
    except TiltCalError as exc:
        print('ABORT: {}'.format(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
