#!/usr/bin/env python3
"""Tilt-calibration grid capture — the C-LEVEL-2 acquisition tool (OPERATOR-run).

Drives the platform over an (x, y) grid at one operating height, dwells at each
node, reads the on-board inclinometer N times, and writes an interpolatable
**residual tilt map** to the source-tree ``config/tilt_calibration.yaml``. Then
it asks the running ``trajectory_node`` to reload the file, verifies the applied
version matches what it just wrote, and re-measures a handful of **off-node**
check poses to prove the interpolation actually removed the error.

Contract: ``ros_ws/docs/levelling_frame.md`` § **C-LEVEL-2**.
Plan: ``plans/archived/2026-08-15 tilt-calibration-grid.md`` (Phase 3 builds this; Phase 4's
rungs C0–C3 run it — ``tests/hardware/session_tilt_calibration.md`` is the
runbook and is the authority for what to do at the robot).

**What a residual is — ANCHOR-MEAN HOME-REFERENCED (2026-08-10).** ``level``
measures the platform's tilt against gravity at exactly one pose and applies
that offset everywhere, so every *pose-dependent* kinematic error is invisible
to it. With ANY constant correction loaded and **no map loaded**, this tool
commands the level orientation at each node and measures::

    m_i = mean(raw_reading) + radians(JB_OP_INCLINOMETER_OFFSET_DEG)

— the exact ``state_machine.LevellingHandler`` formula (``:455-460``; an ADD,
per axis, in radians). The home pose is re-measured **k times through the
sweep** — at the start, after every ``--home-revisit-every`` non-home grid
visits, and at the end — and the shipped residual is referenced to the **MEAN
of those anchors**::

    M(P_i) := m_i − mean_over_anchors(m_home)

Writing the platform's true field as ``f(P)`` and the loaded correction as
``C_cap``, each measurement is ``m_i = f(P_i) − C_cap``, so
``M(P_i) = f(P_i) − f(home)``: the correction cancels EXACTLY, however stale,
as long as it is CONSTANT across the sweep.

**Why the MEAN, and not the first anchor or a time interpolation.** Two C0
captures on 2026-08-10 (``temp/logs/tilt_cal_grid_20260810_115343*`` and
``_120735*``) each completed all nine nodes cleanly and then re-measured home
**+1.81 mrad / +1.59 mrad on ty** against the start — reproducibly, with the
``/gravity_offset`` monitor silent and the ODrive forensics clean, i.e. *not*
a changed correction. The mechanism is OPEN, with two live candidates:
**arrival repeatability** (Jugglebot is hand-built with FDM-printed parts;
re-arriving at a pose after a workspace tour lands within ~1–2 mrad of tilt,
path-dependent hysteresis — the leading hypothesis, and consistent with two
identical sweeps reproducing the same offset) and **sensor warm-up / thermal
settling** of the SCL3300 after the ODrive power-cycle. The anchor series
recorded on every capture from here discriminates them for free: arrival
repeatability scatters about a mean, warm-up trends monotonically.

The mean is the right reference under BOTH. Under arrival repeatability it
averages per-arrival noise down as ``σ/sqrt(k)``; under a smooth time drift it
centres the error at ``±`` half the total drift instead of pinning it to one
end. Either residual is far inside the owner-stated 0.5 deg (8.7 mrad)
repeatability tolerance and the 0.15 deg ``θ_acc``, so a time interpolation
would buy nothing a future session could measure while making the map depend
on a mechanism nobody has yet identified. With zero anchor scatter the mean
reduces **bit-identically** to the single-home referencing it replaces
(test-pinned).

**The gate is a REPORT first.** Anchors are printed on every capture — the
per-anchor table, the per-axis peak-to-peak spread and the signed
start-to-end trend — because the series is the evidence that settles the
mechanism. A p-p spread above 0.002 rad WARNs prominently and does **not**
stop the capture. The capture aborts only on a spread above **0.0087 rad**
(0.5 deg, the owner-stated repeatability tolerance) or a single
**consecutive-anchor step above 0.005 rad** — a discrete event (a re-level, a
relaunch re-push, a mechanical snap) rather than the smooth wander the design
now tolerates. The retired tight statistical drift gate discarded two complete,
usable captures over ~1.8 mrad; the gate's abort-and-discard behaviour was the
bug, not the measurement. The causal ``/gravity_offset`` monitor is unchanged
and remains the detector for an actually-changed correction.

The earlier start-of-capture "STALE LEVEL REFERENCE" abort stays retired — it
was mistuned against physics (the level reference is ONE int16-quantised
SCL3300 sample with measured session scatter σ ≈ 1.2–1.7 mrad/axis; the
1.5 mrad floor sat inside that distribution and false-aborted 40–60 % of
healthy attempts). What screens the reference now: a ``|m_home|`` WARN at
0.010 rad (level grossly stale — harmless to a home-referenced map, worth
surfacing) and a hard abort at 0.05 rad. Verification check poses are scored
home-referenced too (home is re-measured first and subtracted), or a constant
stale level would fail every check pose of a perfectly valid capture.
Contract: ``levelling_frame.md`` § C-LEVEL-2.

**Safety posture — request-only, like ``traj_ramp_battery.py``.** This tool
issues ``trajectory/go_to_pose`` requests and reads services. It **never arms,
never changes control mode, never touches limits, never commands the hand**.
Arming, modes and E-STOP belong to the operator.

**The preflight's mode check is NOT a backstop, and this tool does not rely on
one.** ``mode != TRAJECTORY`` does come back ``WRONG_MODE``, but the mode is only
half the question: with the mode right and the **wire disarmed**
(``mpc_active=0``), ``go_to_pose`` is *accepted* — streaming-while-disarmed is
the legal pre-arm phase of the ARMING CONTRACT — the setpoints are emitted and
dropped, and the platform does not move. A whole capture then completes against
a stationary platform, every residual is the same number, and the tool writes,
applies and *verifies* a plausible all-zeros calibration that verifies perfectly
**because** nothing moved. That is not hypothetical: on 2026-07-15 a full ramp
battery ran exactly that way with zero warnings anywhere. So the disarmed wire is
guarded explicitly and in three layers — the ``/link_status`` ``mpc_active``
check (at preflight **and** re-checked from a fresh message before every node),
the ``DISARMED`` marker on every accepted ``go_to_pose`` response, and a
flat-field warning before the map is written. See § "pure core: the
disarmed-wire guard" below.

**Every exit path returns the platform to the centre node first.** Aborts,
Ctrl-C and exceptions all route through the same return-to-centre, so the
platform is not left parked at a raised displaced pose. **One case cannot honour
that and says so instead**: a wire that disarmed mid-sweep makes the return
itself un-executable — the move would be "accepted" and dropped like every other
— so the return is refused and the tool prints ``RETURN TO CENTRE FAILED … bring
it home manually``. Loudly failing to come home beats silently pretending to.

**Transit shaping — lean deferred to node config, duration deliberately slow.**
``--move-duration-s`` defaults to 3.0 s and ``--lean-gain`` defaults to
``-1.0`` = defer to the node's configured gain (ships 0.6, per
``GoToPose.srv``: negative ⇒ config; an EXPLICIT 0.0 forces lean OFF). Lean
shaping affects the TRANSIT only: the lean window w(s) is exactly zero at
s = 0 and s = 1 (``shaping.py`` clamped-smoothstep window), so the terminal
pose — the thing this tool measures — is the pure commanded pose at ANY gain.
An earlier default of an explicit 0.0 argued that lean off keeps the terminal
pose pure; that inverts under scrutiny (the terminal pose is pure regardless),
and shaped transits are the precedent-proven working point: all four ±150
corners were held on 2026-07-27 with lean 0.6
(``logbook/2026-07-28-anomaly-fixes-validation-sitting.md``). The slow
duration is the other half: unshaped ±150 mm laterals at MINIMUM duration are
the guard-latch canary — 0.94–1.08 rev deviation against a 1.0 rev latch at
~312 mm/s leg peaks (``logbook/2026-07-17-wobble-latch-unshaped-traverse.md``)
— while a 212 mm diagonal over 3.0 s runs ~71 mm/s mean. Do not shorten the
duration to save time without re-reading that entry.

**Faults are detected mid-node, not just at boundaries (2026-08-10).** On the
2026-08-09 C0 sitting, leg 0's ODrive latched SPINOUT_DETECTED 2.5 s into the
move to (−150, −150); the platform collapsed, 30 reads ran against it, and the
fault surfaced only at the NEXT node's boundary wire check, 6.1 s later. The
tool now consults the cached ``/link_status`` kv (callbacks already pump
during the gap spins — zero extra service calls) at arrival and after EVERY
read, aborts immediately naming the read index, and dumps forensics on any
abort: the full link kv, per-leg ODrive ``active_errors``/``disarm_reason``
RAW and DECODED (name table from ``jugglebot/can/odrive.py``), a
first-nonzero-error edge latch per leg, the last trajectory status, and
wall + uptime clocks — into the console AND ``_meta.json``, whose
``abort_reason`` field is now always written. Capture forensics BEFORE any
relaunch: the next launch's BOOT pre-flight auto-clears the drive-side error
record.

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

Usage (operator; stack up, platform ARMED in TRAJECTORY, a correction loaded —
fresh ``level`` RECOMMENDED, constancy through the sweep REQUIRED)::

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
from jugglebot.motion import workspace
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import pose_to_leg_lengths

# The ODrive error-bit name table, for decoding forensics dumps.
# ``jugglebot/can/odrive.py`` is the authority, but its package __init__
# pulls the jugglebot_interfaces message bindings and python-can — present on
# the robot and on this Jetson's test environment, absent on an off-robot
# checkout — so the pure core vendors the table as a fallback rather than
# make its import hostage to the ROS workspace.
# ``tests/motion/test_tilt_cal_grid.py`` pins vendored == authoritative
# whenever the real import is available, so drift is a test failure, not a
# silent wrong decode. 67108864 -> 'SPINOUT_DETECTED' is the 2026-08-09
# leg-0 code.
try:
    from jugglebot.can.odrive import ERROR_CODES as ODRIVE_ERROR_CODES
except Exception:                                        # noqa: BLE001
    # Vendored from ros_ws/src/jugglebot/jugglebot/can/odrive.py ERROR_CODES.
    ODRIVE_ERROR_CODES = {
        1: 'INITIALIZING',
        2: 'SYSTEM_LEVEL',
        4: 'TIMING_ERROR',
        8: 'MISSING_ESTIMATE',
        16: 'BAD_CONFIG',
        32: 'DRV_FAULT',
        64: 'MISSING_INPUT',
        256: 'DC_BUS_OVER_VOLTAGE',
        512: 'DC_BUS_UNDER_VOLTAGE',
        1024: 'DC_BUS_OVER_CURRENT',
        2048: 'DC_BUS_OVER_REGEN_CURRENT',
        4096: 'CURRENT_LIMIT_VIOLATION',
        8192: 'MOTOR_OVER_TEMP',
        16384: 'INVERTER_OVER_TEMP',
        32768: 'VELOCITY_LIMIT_VIOLATION',
        65536: 'POSITION_LIMIT_VIOLATION',
        16777216: 'WATCHDOG_TIMER_EXPIRED',
        33554432: 'ESTOP_REQUESTED',
        67108864: 'SPINOUT_DETECTED',
        134217728: 'BRAKE_RESISTOR_DISARMED',
        268435456: 'THERMISTOR_DISCONNECTED',
        1073741824: 'CALIBRATION_ERROR',
    }

import _th_test_common as thc

# ── defaults ─────────────────────────────────────────────────────────────
#
# Every default marked PROVISIONAL is a placeholder until rung C0 of
# plans/archived/2026-08-15 tilt-calibration-grid.md measures it on hardware. C0 exists
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

#: Mid-sweep home re-measures: one anchor after every N **non-home** grid
#: visits. 0 disables them (the start and end anchors always exist). The
#: default 4 gives ~3 anchors on the 3x3 probe grid and ~7 on the 5x5 default
#: — enough to see the shape of the home series without spending a third of
#: the sweep on it (each anchor costs one move + dwell + n_reads, ~9 s at the
#: C0 working point, so 5 mid-sweep anchors add ~45 s to a ~4 min capture).
DEFAULT_HOME_REVISIT_EVERY = 4

#: Anchor-series gates (2026-08-10). These REPLACE the tight statistical drift
#: gate, which compared start-vs-end home means against a read-noise-derived
#: tolerance (~0.75-0.94 mrad) and aborted BOTH complete C0 captures on
#: 2026-08-10 over a reproducible ~1.6-1.8 mrad ty offset — with the causal
#: /gravity_offset monitor silent and the forensics clean, i.e. the level
#: reference had NOT changed. Discarding a finished, usable capture over an
#: effect ~4.8x INSIDE the owner's repeatability tolerance was the bug.
#:
#: The mechanism is OPEN: platform arrival repeatability (hand-built, FDM
#: parts, path-dependent hysteresis on re-arriving at a pose) leads; SCL3300
#: warm-up / thermal settling fits too. The anchor series recorded on every
#: capture from here discriminates them, which is why the WARN is prominent
#: and non-blocking rather than absent.
#:
#: WARN: any axis's anchor peak-to-peak spread over 0.002 rad (0.115 deg) —
#: above the ~1 mrad read-noise floor of an N-read mean, so it reports real
#: pose-return variation and not sensor noise.
#: ABORT: p-p over 0.0087 rad = **0.5 deg, the owner-stated repeatability
#: tolerance** — beyond it the "which arrival is the reference" question stops
#: being a rounding error against the 0.15 deg theta_acc.
#: ABORT: any single CONSECUTIVE-anchor step over 0.005 rad — a discrete event
#: (re-level, relaunch re-push of the int16-mrad Teensy copy, a mechanical
#: snap), which is a different physical claim from smooth wander and stays
#: fatal at a threshold the smooth case does not reach.
ANCHOR_PP_WARN_RAD = 0.002       # 0.115 deg — prominent, NON-blocking
ANCHOR_PP_ABORT_RAD = 0.0087     # 0.5 deg — owner repeatability tolerance
ANCHOR_STEP_ABORT_RAD = 0.005    # a discrete event between two anchors

#: ``|m_home|`` sanity bounds. Under home-referencing a nonzero home
#: measurement is HARMLESS to the map (it cancels), so these are not
#: staleness gates: the WARN (0.010 rad = 0.57 deg, >= 5.9 sigma of measured
#: level scatter — essentially never a false fire) surfaces a level so stale
#: the sweep is physically tilted by more than the field being measured; the
#: ABORT (0.05 rad, aligned with tilt_map.MAX_ABS_RESIDUAL_RAD) is where the
#: second-order term |Δf x m_home|/2 = 2.6e-4 rad exceeds the composition
#: budget and something is genuinely broken.
HOME_REF_WARN_RAD = 0.010
HOME_REF_ABORT_RAD = 0.05

#: Per-node IK stroke-margin preflight: refuse any pose whose worst leg comes
#: within this margin of the Jetson/firmware hard extension bound
#: [LEG_HARD_MARGIN_MM, stroke - LEG_HARD_MARGIN_MM] = [5, 275] mm. Pure
#: numpy (ik_solver.pose_to_leg_lengths at identity orientation — the loaded
#: correction shifts extensions by <= ~2.2 mm at these magnitudes, well
#: inside this margin). The default 5x5/±150 grid's worst corrected margin is
#: +20.6 mm (ik-stroke analysis, 2026-08-10), so this refuses nothing the
#: plan intends while catching a --x/--y/--z excursion before any motion.
STROKE_PREFLIGHT_MARGIN_MM = 10.0

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

#: Phase tag for a MID-SWEEP home re-measure. Distinct from ``'capture'`` so
#: the analyser cannot average an anchor into the home grid node (that would
#: mix a start-of-sweep value with re-arrivals minutes later, exactly the
#: variation the anchors exist to measure) and distinct from ``'home_end'`` so
#: the final anchor stays identifiable in every historical CSV.
PHASE_HOME_ANCHOR = 'home_anchor'
PHASE_HOME_END = 'home_end'

TOOL_NAME = 'tests/hardware/tilt_cal_grid.py'


class TiltCalError(RuntimeError):
    """An operator-facing refusal: the message is the whole error report."""


class TiltCalStatusTimeout(TiltCalError):
    """``wait_for_status`` gave up waiting for an EXPECTED state.

    Distinct from the base class so a caller can attach its own remedy text —
    "the ament share copy is shadowing the source tree", "check $JUGGLEBOT_TILT_CAL
    on both sides" — to a timeout while any other failure still routes through
    the ordinary ABORT path unchanged. ``last_status`` is the final message seen,
    so the caller can name what the node actually reported rather than guessing.
    """

    def __init__(self, message: str, last_status: Any = None):
        super().__init__(message)
        self.last_status = last_status


class TiltCalFaultError(TiltCalError):
    """A RUN-level abort that must never be demoted to a failed node.

    The per-node/per-check ``try`` blocks convert a :class:`TiltCalError` into
    a failed *node* and, under the default ``--on-fail continue``, carry on.
    That is right for a flaky read and catastrophically wrong for a wire that
    disarmed, a guard fault that latched, or a correction that changed
    mid-sweep — every remaining node would measure a collapsed or re-referenced
    platform. Both handlers re-raise this subclass unconditionally.

    Precedent: 2026-08-09, leg-0 SPINOUT_DETECTED latched mid-move; 30 reads
    ran against a collapsed platform because nothing between the boundary
    checks could abort the run.
    """


class TiltRead(NamedTuple):
    """One inclinometer read with ITS OWN clocks.

    The 2026-08-09 CSVs stamped all 30 rows of a node with one ``iso`` and one
    ``uptime_ms`` captured after the read loop, which made intra-node fault
    correlation impossible from the CSV (the leg-0 collapse had to be timed
    from the launch log). ``t_wall`` feeds the ``iso`` column, ``t_mono`` the
    ``t_s`` column (monotonic — immune to wall-clock steps), ``uptime_ms`` is
    the can-bridge clock as cached at THIS read.
    """

    t_wall: float
    t_mono: float
    uptime_ms: Optional[int]
    tx: float
    ty: float


class Node(NamedTuple):
    """One grid node — ``ix``/``iy`` index ``x_mm``/``y_mm`` respectively."""

    ix: int
    iy: int
    x_mm: float
    y_mm: float


class HomeAnchor(NamedTuple):
    """One measurement of the home pose, timestamped within the capture.

    The capture interleaves k of these — start, every
    ``--home-revisit-every`` non-home grid visits, end — and the shipped
    reference is their MEAN. ``t_s`` is seconds from the run's monotonic
    start, matching the CSV ``t_s`` column, so an anchor row in the report can
    be found in the CSV without guesswork; it is what makes the series a
    *time* series and therefore able to separate a monotonic warm-up trend
    from arrival-to-arrival scatter.
    """

    t_s: float
    m_tx: float
    m_ty: float
    sd_tx: float
    sd_ty: float
    n_ok: int

    @classmethod
    def from_stats(cls, t_s: float, stats: ReadStats) -> 'HomeAnchor':
        return cls(t_s=float(t_s), m_tx=stats.res_tx, m_ty=stats.res_ty,
                   sd_tx=stats.sd_tx, sd_ty=stats.sd_ty, n_ok=stats.n_ok)

    def as_dict(self) -> Dict[str, Any]:
        """The metadata row shape (``captured.home_reference.anchors[i]``)."""
        return {
            't_s': round(float(self.t_s), 3),
            'm_tx_rad': float(self.m_tx),
            'm_ty_rad': float(self.m_ty),
            'sd_tx_rad': float(self.sd_tx),
            'sd_ty_rad': float(self.sd_ty),
            'n_ok': int(self.n_ok),
        }


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

    The home node is not a nicety: it is the **reference node** — every shipped
    residual is ``measured − m_home``, so a grid without ``(0, 0)``
    (``--nodes 4`` over a symmetric box, say) has nothing to reference against
    and cannot be captured under C-LEVEL-2's home-referenced definition at
    all. Refuse instead.
    """
    tol = 1e-6
    ix = next((i for i, v in enumerate(x_mm) if abs(v) <= tol), None)
    iy = next((i for i, v in enumerate(y_mm) if abs(v) <= tol), None)
    if ix is None or iy is None:
        raise TiltCalError(
            'the home node (0, 0) must be one of the grid nodes — the map is '
            'HOME-REFERENCED (every shipped residual is measured minus the '
            'home measurement), so a grid without (0, 0) has no reference. '
            'Use an odd --nodes over a symmetric --box, or put 0 in both --x '
            'and --y. Got x={}, y={}'
            .format(list(x_mm), list(y_mm)))
    return (ix, iy)


def centre_out_order(x_mm: Sequence[float],
                     y_mm: Sequence[float]) -> List[Node]:
    """Every grid node sorted centre-out: radius ascending, **corners LAST**.

    Replaced the boustrophedon serpentine (2026-08-10). The serpentine
    minimised travel but made visit 2 the far corner — on the 2026-08-09 C0
    sitting the first post-home move was the 212 mm diagonal to (−150, −150),
    the sweep's longest traverse and its highest-stroke pose, and the leg-0
    collapse there cost the whole capture with zero usable field data banked.
    Centre-out banks the inner (low-stroke) nodes before the corner poses, so
    a fault at an extremity loses the corners, not the capture. Costs more
    total travel; each move is still individually shaped and slow.

    Tie-break within a radius ring is CCW angle from +x then index — fully
    deterministic, never dict/float-order dependent.
    """
    nodes = [Node(ix=ix, iy=iy, x_mm=float(x), y_mm=float(y))
             for iy, y in enumerate(y_mm) for ix, x in enumerate(x_mm)]
    nodes.sort(key=lambda n: (round(n.x_mm ** 2 + n.y_mm ** 2, 6),
                              math.atan2(n.y_mm, n.x_mm), n.ix, n.iy))
    return nodes


def visit_order(x_mm: Sequence[float], y_mm: Sequence[float]) -> List[Node]:
    """The capture order: **home node first**, then centre-out, corners last.

    Home is measured first — that reading is the FIRST home anchor, and the
    shipped reference is the mean of all of them — and exactly once *as a grid
    node*: the mid-sweep revisits and the end-of-sweep re-measure return to
    the same pose, but those readings are anchors, never a second value for
    the node. (Averaging them into the node would fold the very arrival-to-
    arrival variation the anchors exist to measure into the node's residual.)
    """
    hx, hy = home_index(x_mm, y_mm)
    home = Node(ix=hx, iy=hy, x_mm=float(x_mm[hx]), y_mm=float(y_mm[hy]))
    rest = [n for n in centre_out_order(x_mm, y_mm)
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


def home_reference_verdict(stats: ReadStats) -> Tuple[str, str]:
    """Sanity-screen the home REFERENCE measurement. ``(verdict, message)``.

    ``verdict`` is ``'ok'`` / ``'warn'`` / ``'abort'``. Under home-referencing
    a nonzero ``m_home`` is HARMLESS to the map — it cancels out of every
    shipped residual — so this is **not** a staleness gate (the old one is
    retired; it false-aborted 40–60 % of healthy attempts because the level
    reference's own single-sample scatter is 1.2–1.7 mrad/axis against its
    1.5 mrad floor). What is left to screen:

    * too few finite reads ⇒ ``abort`` — no reference, no map;
    * ``|m_home| > 0.05 rad`` ⇒ ``abort`` — the second-order term
      ``|Δf × m_home|/2`` exceeds the composition budget there (2.6e-4 rad),
      and a platform 2.9° off level at its own levelling pose is broken, not
      stale;
    * ``|m_home| > 0.010 rad`` ⇒ ``warn`` — ≥ 5.9 σ of measured level scatter
      (never a false fire on a healthy path), harmless to the map (second
      order 5.3e-5 rad), but the sweep is physically tilted by more than the
      0.6° field it is measuring, and the operator should know why.

    Both axes screened independently: a gross error usually leans on one axis
    and a magnitude check dilutes it by up to sqrt(2).
    """
    if stats.n_ok < MIN_GOOD_READS:
        return ('abort',
                'home node returned only {}/{} finite reads — the inclinometer '
                'is not answering reliably; fix that before capturing'
                .format(stats.n_ok, stats.n_total))
    worst = max(abs(stats.res_tx), abs(stats.res_ty))
    detail = ('m_home ({:+.6f}, {:+.6f}) rad = ({:+.4f}, {:+.4f}) deg, '
              'per-read sd ({:.6f}, {:.6f}) rad'
              .format(stats.res_tx, stats.res_ty,
                      math.degrees(stats.res_tx), math.degrees(stats.res_ty),
                      stats.sd_tx, stats.sd_ty))
    if worst > HOME_REF_ABORT_RAD:
        return ('abort',
                'home reference |m_home| = {:.4f} rad ({:.2f} deg) exceeds '
                'the {:.3f} rad sanity ceiling — this is not a stale level, '
                'it is a platform ~{:.1f} deg off level at its own levelling '
                'pose (collapsed leg? units error?). {}. Diagnose before '
                'capturing anything.'
                .format(worst, math.degrees(worst), HOME_REF_ABORT_RAD,
                        math.degrees(worst), detail))
    if worst > HOME_REF_WARN_RAD:
        return ('warn',
                'home reference |m_home| = {:.4f} rad ({:.2f} deg) is above '
                'the {:.3f} rad WARN — the level reference is grossly stale. '
                'HARMLESS to the map (home-referencing cancels it; second-'
                'order residue <= 5.3e-5 rad at this bound is already priced '
                'in), but the sweep is running physically tilted by more than '
                'the field it measures. A fresh `level` would zero this. {}'
                .format(worst, math.degrees(worst), HOME_REF_WARN_RAD, detail))
    return ('ok', 'home reference OK: {}'.format(detail))


def _exact_mean(values: Sequence[float]) -> float:
    """Mean that is **bit-identical to the value itself** at zero scatter.

    ``sum(v)/k`` does not have that property in floating point: three copies
    of ``a`` sum to a rounded ``3a`` whose quotient by 3 can miss ``a`` by an
    ULP. That ULP would make the anchor-mean design fail to reduce exactly to
    the single-home referencing it replaces, and "the map changed in the last
    digit because we added a redundant measurement that agreed" is precisely
    the kind of unexplainable diff a machine-written, committed YAML must not
    produce. So identical inputs short-circuit, and everything else goes
    through ``math.fsum`` (exactly-rounded summation — no accumulation order
    dependence either).
    """
    items = [float(v) for v in values]
    if not items:
        raise TiltCalError('cannot average an empty anchor series')
    first = items[0]
    if all(v == first for v in items):
        return first
    return math.fsum(items) / len(items)


def usable_anchors(anchors: Sequence[HomeAnchor]) -> List[HomeAnchor]:
    """Anchors with enough finite reads to mean anything (``>= MIN_GOOD_READS``)."""
    return [a for a in anchors if a.n_ok >= MIN_GOOD_READS]


def anchor_mean_rad(anchors: Sequence[HomeAnchor]) -> Tuple[float, float]:
    """The shipped reference: the MEAN of every usable home anchor.

    ``M(P_i) = m_i − anchor_mean`` for every node. With one anchor, or with k
    anchors that agree exactly, this returns that value **bit-identically** —
    so the anchor-mean map reduces to the previous single-home map with no
    numeric change at all (see :func:`_exact_mean`).

    Why the mean rather than the first anchor or a time interpolation is
    argued in the module docstring: it is the right estimator under BOTH open
    mechanisms (arrival repeatability ⇒ σ/sqrt(k); smooth drift ⇒ centred at
    ± half the total), and the difference between them is far inside the
    0.5 deg repeatability tolerance and the 0.15 deg theta_acc.
    """
    good = usable_anchors(anchors)
    if not good:
        raise TiltCalError(
            'no usable home anchor ({} recorded, none with >= {} finite '
            'reads) — the map has no reference and cannot be written'
            .format(len(anchors), MIN_GOOD_READS))
    return (_exact_mean([a.m_tx for a in good]),
            _exact_mean([a.m_ty for a in good]))


def anchor_spread(anchors: Sequence[HomeAnchor]) -> Dict[str, Any]:
    """Peak-to-peak, trend and worst consecutive step of the home series.

    Pure reduction, no verdict — :func:`anchor_series_verdict` applies the
    thresholds and the report prints these numbers whatever the verdict is.
    ``trend`` is END minus START (signed, so a monotonic warm-up is legible as
    such); ``max_step`` is over CONSECUTIVE anchors, which is what separates a
    discrete event from smooth wander of the same total size.
    """
    good = usable_anchors(anchors)
    out: Dict[str, Any] = {'n_anchors': len(good), 'n_recorded': len(anchors)}
    for axis, values in (('tx', [a.m_tx for a in good]),
                         ('ty', [a.m_ty for a in good])):
        if not values:
            out[axis] = {'pp': float('nan'), 'trend': float('nan'),
                         'max_step': float('nan'), 'max_step_index': None}
            continue
        steps = [values[i + 1] - values[i] for i in range(len(values) - 1)]
        worst = max(range(len(steps)), key=lambda i: abs(steps[i])) \
            if steps else None
        out[axis] = {
            'pp': max(values) - min(values),
            'trend': values[-1] - values[0],
            'max_step': abs(steps[worst]) if worst is not None else 0.0,
            'max_step_signed': steps[worst] if worst is not None else 0.0,
            'max_step_index': worst,
        }
    return out


def anchor_table_lines(anchors: Sequence[HomeAnchor]) -> List[str]:
    """The per-anchor report table, one string per line (header included).

    Printed on EVERY capture — clean or not. The series is the evidence that
    will settle whether the ~1.6-1.8 mrad home offset seen on both 2026-08-10
    C0 captures is arrival repeatability or sensor warm-up, and evidence that
    is only printed when something fails is evidence nobody collects.
    """
    lines = ['    {:>3} {:>9} {:>12} {:>12} {:>10} {:>10} {:>6}'.format(
        '#', 't_s', 'm_tx_rad', 'm_ty_rad', 'sd_tx', 'sd_ty', 'n_ok')]
    for i, a in enumerate(anchors):
        lines.append(
            '    {:>3d} {:>9.1f} {:>+12.6f} {:>+12.6f} {:>10.6f} {:>10.6f} '
            '{:>6d}'.format(i + 1, a.t_s, a.m_tx, a.m_ty, a.sd_tx, a.sd_ty,
                            a.n_ok))
    return lines


def anchor_series_verdict(anchors: Sequence[HomeAnchor]) -> Tuple[str, str]:
    """``('ok' | 'warn' | 'abort', message)`` for the home-anchor series.

    Replaces the retired start-vs-end statistical drift gate. The thresholds
    are the module constants and their rationale is there; in one line: this
    gate no longer tries to detect "the reference moved at all", because the
    reference demonstrably moves ~1-2 mrad on a healthy machine and the map
    does not care. It detects (a) variation large enough to matter against the
    owner's 0.5 deg repeatability tolerance, and (b) a discrete step between
    two consecutive anchors, which is a re-level / relaunch / mechanical snap
    rather than the pose-return wander the design now absorbs.

    A WARN names BOTH open mechanisms deliberately: an operator who reads
    "arrival repeatability" as settled fact would stop recording the series,
    and the series is what discriminates them.
    """
    good = usable_anchors(anchors)
    if len(good) < 1:
        return ('abort',
                'no usable home anchor ({} recorded) — the capture has no '
                'reference to ship residuals against'.format(len(anchors)))
    if len(good) < len(anchors):
        bad = [i + 1 for i, a in enumerate(anchors)
               if a.n_ok < MIN_GOOD_READS]
        return ('abort',
                'home anchor(s) {} returned fewer than {} finite reads — the '
                'inclinometer stopped answering at the reference pose, so the '
                'anchor series cannot certify this capture'
                .format(bad, MIN_GOOD_READS))

    spread = anchor_spread(anchors)
    detail = '; '.join(
        '{}: p-p {:.6f} rad ({:.4f} deg), trend {:+.6f} rad ({:+.4f} deg), '
        'worst consecutive step {:.6f} rad'
        .format(axis, spread[axis]['pp'], math.degrees(spread[axis]['pp']),
                spread[axis]['trend'], math.degrees(spread[axis]['trend']),
                spread[axis]['max_step'])
        for axis in ('tx', 'ty'))
    header = '{} home anchor(s) over {:.0f} s — {}'.format(
        len(good), (good[-1].t_s - good[0].t_s), detail)

    over_pp = [axis for axis in ('tx', 'ty')
               if spread[axis]['pp'] > ANCHOR_PP_ABORT_RAD]
    if over_pp:
        return ('abort',
                'HOME ANCHOR SPREAD EXCEEDS THE REPEATABILITY TOLERANCE on {} '
                '— {}. The ceiling is {:.4f} rad (0.5 deg), the repeatability '
                'this machine is expected to hold; past it the map depends on '
                'WHICH arrival was used as the reference, which is no longer '
                'a rounding error against the {:.2f} deg theta_acc. Nothing '
                'was written. Check for a shifting base, a loose platform '
                'joint, or a leg that is not returning to its commanded '
                'position, then re-run.'
                .format('/'.join(over_pp), header, ANCHOR_PP_ABORT_RAD,
                        DEFAULT_THRESHOLD_DEG))

    over_step = [axis for axis in ('tx', 'ty')
                 if spread[axis]['max_step'] > ANCHOR_STEP_ABORT_RAD]
    if over_step:
        where = ', '.join(
            '{} between anchors {} and {} ({:+.6f} rad)'.format(
                axis, spread[axis]['max_step_index'] + 1,
                spread[axis]['max_step_index'] + 2,
                spread[axis]['max_step_signed'])
            for axis in over_step)
        return ('abort',
                'DISCRETE STEP IN THE HOME ANCHOR SERIES — {} exceeds the '
                '{:.4f} rad consecutive-step ceiling. {}. A step between two '
                'consecutive anchors is an EVENT, not the smooth arrival-to-'
                'arrival wander this capture tolerates: a re-level or relaunch '
                're-pushing the int16-mrad Teensy copy (the /gravity_offset '
                'monitor is the causal detector and should also have fired), '
                'or something mechanical letting go. Nothing was written; '
                'diagnose before re-running.'
                .format(where, ANCHOR_STEP_ABORT_RAD, header))

    over_warn = [axis for axis in ('tx', 'ty')
                 if spread[axis]['pp'] > ANCHOR_PP_WARN_RAD]
    if over_warn:
        return ('warn',
                'HOME ANCHOR SPREAD above the {:.4f} rad WARN on {} — {}. '
                'The capture is VALID and was written: every residual is '
                'referenced to the MEAN of these anchors, and this spread is '
                'well inside the 0.0087 rad (0.5 deg) repeatability tolerance '
                'and the {:.2f} deg theta_acc. Two mechanisms remain open — '
                'ARRIVAL REPEATABILITY of the hand-built FDM structure '
                '(path-dependent hysteresis on re-arriving at a pose; scatters '
                'about a mean) versus SENSOR WARM-UP / thermal settling of the '
                'SCL3300 (trends monotonically). The anchor series across '
                'captures will discriminate them — record this table with the '
                'session notes.'
                .format(ANCHOR_PP_WARN_RAD, '/'.join(over_warn), header,
                        DEFAULT_THRESHOLD_DEG))
    return ('ok', 'home anchor series OK — {}'.format(header))


def home_revisit_after_visits(n_nodes: int, revisit_every: int) -> List[int]:
    """Grid-visit indices after which a mid-sweep home anchor is inserted.

    Counting is in **non-home grid visits**: visit 0 of the order IS the home
    node (and is the first anchor), so an anchor lands after non-home visits
    ``N, 2N, 3N, …`` — which are grid indices ``N, 2N, 3N, …`` — while the
    last non-home visit is skipped because the end-of-sweep re-measure already
    anchors there. Two consecutive home measurements separated by nothing but
    a re-move would measure the same arrival twice, which is the one thing the
    series must not do.

    3x3 (9 nodes, 8 non-home) at N=4 ⇒ one mid-sweep anchor ⇒ 3 anchors total.
    5x5 (25 nodes, 24 non-home) at N=4 ⇒ five ⇒ 7 total. ``revisit_every <= 0``
    disables mid-sweep anchors entirely (start + end remain).
    """
    if revisit_every <= 0 or n_nodes <= 2:
        return []
    n_non_home = n_nodes - 1
    out: List[int] = []
    k = 1
    while k * revisit_every < n_non_home:
        out.append(k * revisit_every)
        k += 1
    return out


def decode_error_bits(mask: int) -> List[str]:
    """ODrive error bitmask → names, via the ``can/odrive.py`` table.

    Decomposes ``active_errors`` and ``disarm_reason`` alike (same bit
    vocabulary). Unknown bits are surfaced as a hex remainder rather than
    silently dropped — an undecodable bit is exactly the datum a forensics
    dump must not eat. ``67108864 → ['SPINOUT_DETECTED']`` is the 2026-08-09
    leg-0 code.
    """
    mask = int(mask)
    names: List[str] = []
    for bit in sorted(ODRIVE_ERROR_CODES):
        if mask & bit:
            names.append(ODRIVE_ERROR_CODES[bit])
            mask &= ~bit
    if mask:
        names.append('UNKNOWN_BITS_0x{:08X}'.format(mask))
    return names


# ── pure core: IK stroke-margin preflight ────────────────────────────────


_GEOMETRY: Optional[StewartGeometry] = None


def _stewart_geometry() -> StewartGeometry:
    global _GEOMETRY
    if _GEOMETRY is None:
        _GEOMETRY = StewartGeometry()
    return _GEOMETRY


def stroke_margin_problems(
        poses_mm: Sequence[Tuple[float, float]], z_mm: float,
        margin_mm: float = STROKE_PREFLIGHT_MARGIN_MM) -> List[str]:
    """Refusal lines for every pose too close to a leg extension bound.

    Pure-numpy IK (``ik_solver.pose_to_leg_lengths``, identity orientation —
    poses here are STOW-relative exactly like ``GoToPose``, and the loaded
    gravity correction moves extensions by ≤ ~2.2 mm at grid magnitudes, well
    inside the margin). A pose is refused when its worst leg's distance to the
    Jetson/firmware hard bound ``[LEG_HARD_MARGIN_MM, stroke −
    LEG_HARD_MARGIN_MM]`` = [5, 275] mm is under ``margin_mm``.

    This runs BEFORE any motion (and in ``--dry-run``): a node on a stroke
    clamp does not fail loudly at capture time — the firmware clamps per leg
    at 500 Hz and the node's reading is silently wrong. Sanity anchor: home
    ``(0, 0, 170)`` sits mid-stroke at ~154 mm on all six legs.

    Kinematic scope note: the 2026-08-09 leg-0 collapse was NOT a stroke
    event — leg 0 was the most-RETRACTED leg at (−150, −150) with ≥ 85 mm of
    margin (SPINOUT_DETECTED, a drive-side fault). This gate exists for the
    class, not that instance.
    """
    geom = _stewart_geometry()
    lo = float(workspace.LEG_HARD_MARGIN_MM)
    hi = float(geom.leg_stroke_mm) - float(workspace.LEG_HARD_MARGIN_MM)
    problems: List[str] = []
    seen = set()
    for x, y in poses_mm:
        key = (round(float(x), 6), round(float(y), 6))
        if key in seen:
            continue
        seen.add(key)
        ext = pose_to_leg_lengths(
            np.array([float(x), float(y), float(z_mm)], dtype=float),
            np.eye(3), geom)
        margins = np.minimum(ext - lo, hi - ext)
        worst = int(np.argmin(margins))
        if margins[worst] < margin_mm:
            problems.append(
                '({:+.1f}, {:+.1f}, {:.1f}): leg {} extension {:.1f} mm is '
                '{:.1f} mm from the [{:.0f}, {:.0f}] mm hard bound '
                '(< {:.0f} mm preflight margin); per-leg ext {}'
                .format(x, y, z_mm, worst, ext[worst], margins[worst],
                        lo, hi, margin_mm,
                        [round(float(e), 1) for e in ext]))
    return problems


# ── pure core: the disarmed-wire guard ───────────────────────────────────
#
# A capture on a DISARMED wire is the failure this section exists to make
# impossible. `trajectory/go_to_pose` ACCEPTS a move while mpc_active=0 by
# design — streaming-while-disarmed is the legal pre-arm phase of the ARMING
# CONTRACT, so the planning verdict is genuinely "accepted" — and the only
# marker is a suffix on `response.message`. The platform then does not move, the
# inclinometer reads the same stationary pose 25 times, every residual comes
# back identical, and the tool writes, applies and VERIFIES a plausible
# all-zeros calibration that verifies perfectly because nothing ever moved.
#
# Precedent: 2026-07-15, a full ramp battery "accepted" and fully emitted with
# mpc_active=0 — zero motion, zero warnings anywhere. That incident is why
# trajectory_node logs the suffix at all; this tool must not discard it.
#
# Three independent layers, because each one alone has a hole:
#   1. the /link_status kv check — cheapest and earliest, but goes stale between
#                                  messages, so it runs at preflight AND again
#                                  before every node and check pose;
#   2. the per-move suffix check — synchronous with the actual command, but it
#                                  only fires once the sweep is under way, and it
#                                  is derived from the same /link_status topic
#                                  (trajectory_node._wire_armed is a bare mirror
#                                  with no staleness timeout), so a stalled
#                                  publisher freezes layer 2 exactly as it
#                                  freezes layer 1 — hence layer 1 treats a
#                                  silent topic as an abort in its own right;
#   3. the flat-field check      — sees the SYMPTOM rather than the cause, so it
#                                  also catches a platform that failed to move
#                                  for a reason neither of the other two can
#                                  observe (armed at the Jetson, dead
#                                  downstream). WARNs; never refuses.


#: Residual spread below which a whole capture is judged "flat". A real field
#: varies 0.041°→0.604° across the workspace (2026-07-28 extremity table), i.e.
#: ~1e-2 rad end to end — two orders above this. A stationary platform read N
#: times varies only by sensor noise, which the 1 mrad Teensy quantum bounds
#: well under this too. PROVISIONAL like every other threshold here until C0/C1
#: measure a real field; it only ever raises a WARNING, never a refusal.
FLAT_FIELD_TOL_RAD = 1e-4


def response_reports_disarmed(message: Any) -> bool:
    """Does an ACCEPTED ``go_to_pose`` response say the wire is disarmed?

    Keys on the substring ``DISARMED``, which is what
    ``trajectory_node._wire_state_suffix`` appends (`` [wire DISARMED — setpoints
    not reaching the legs]``). Case-insensitive and substring-based on purpose:
    the exact wording is a log string that may be reworded, while the word
    itself is the contract. ``go_to_pose``'s only other message content is the
    accepted-move summary and the rejection codes, none of which carry it.
    """
    return 'DISARMED' in str(message or '').upper()


def wire_armed_verdict(link_kv: Optional[Dict[str, str]]) -> Tuple[bool, str]:
    """``(ok, message)`` for a cached ``/link_status`` key-value mapping.

    Reads the keys ``teensy_bridge_node._publish_link_status`` actually
    publishes, in their actual encodings (verified against that function, not
    assumed): ``mpc_active`` is ``str(int(bool))`` — ``'1'`` or ``'0'``;
    ``fault_state`` is a FaultState **enum name**, ``'NONE'`` when clear and
    ``'UNKNOWN'`` when no heartbeat has been seen; ``bridge_link`` is
    ``'UP'`` / ``'LOST'`` / ``'NO_HEARTBEAT'``.

    **Every one of the three keys is REQUIRED**, and an absent one is a failure
    rather than a pass. This runs against a cache that is empty until the first
    message arrives, so defaulting to "armed" would reintroduce exactly the hole
    it closes — and a bridge that publishes ``mpc_active`` publishes all three in
    the same message, so a missing companion key means this is not the publisher
    the tool thinks it is talking to, not that the field is optional.
    """
    kv = link_kv or {}
    if not kv:
        return (False,
                'no /link_status message seen at all — teensy_bridge_node is '
                'not running, or not publishing. This tool cannot confirm the '
                'wire is armed, and a capture on a disarmed wire writes a '
                'plausible all-zeros calibration (2026-07-15).')
    for key in ('mpc_active', 'fault_state', 'bridge_link'):
        if key not in kv:
            return (False,
                    "/link_status carries no {!r} key (saw: {}) — this is not "
                    'the bridge this tool expects; refusing to assume the wire '
                    'is armed.'.format(key, ', '.join(sorted(kv)) or '<empty>'))
    raw = kv.get('mpc_active')
    if str(raw).strip() != '1':
        return (False,
                'the wire is DISARMED (/link_status mpc_active={!r}). '
                'go_to_pose is ACCEPTED while disarmed — the platform would not '
                'move, every node would read the same stationary pose, and this '
                'tool would write and "verify" an all-zeros map. Arm via the '
                'orchestrator (auto-arm on ACTIVE entry) or  ros2 service call '
                '/set_setpoint_output std_srvs/srv/SetBool "{{data: true}}"  '
                'and re-run.'.format(raw))
    # 'UNKNOWN' is what the bridge publishes when it has seen NO heartbeat, so
    # anything that is not exactly 'NONE' is a refusal — a guard latch and an
    # absent heartbeat are both states in which the legs will not move.
    fault = str(kv['fault_state']).strip()
    if fault != 'NONE':
        return (False,
                'Teensy guard fault_state={} — leg output is suppressed even '
                'with mpc_active=1, so a capture would measure a platform that '
                'cannot move. Recover with  ros2 service call /clear_errors '
                'std_srvs/srv/Trigger  and re-run.'.format(fault))
    bridge_link = str(kv['bridge_link']).strip()
    if bridge_link != 'UP':
        return (False,
                'bridge_link={} — the Jetson↔Teensy UDP link is not up, so '
                'commanded setpoints are not reaching the legs.'
                .format(bridge_link))
    return (True,
            'wire ARMED: mpc_active=1 fault_state={} bridge_link={}'
            .format(fault, bridge_link))


def flat_field_verdict(results: Dict[Tuple[int, int], ReadStats],
                       tol_rad: float = FLAT_FIELD_TOL_RAD
                       ) -> Tuple[bool, str]:
    """``(is_flat, message)`` — is every node's residual the same number?

    The **signature of a capture that never moved**. If the platform stayed put,
    each node re-measures one stationary pose and the residuals differ only by
    sensor noise; a real field spans two orders more than ``tol_rad`` across the
    workspace.

    WARNS, never refuses, and this is deliberate. A flat field is also what a
    genuinely well-behaved machine would produce, and a *refusal* would make
    this tool unable to certify the one outcome the whole programme is trying to
    reach. The operator is told what flatness means and decides.
    """
    usable = [s for s in results.values() if s.n_ok >= MIN_GOOD_READS]
    if len(usable) < 2:
        return False, 'fewer than two usable nodes — flatness is undefined'
    spread_tx = max(s.res_tx for s in usable) - min(s.res_tx for s in usable)
    spread_ty = max(s.res_ty for s in usable) - min(s.res_ty for s in usable)
    if not (math.isfinite(spread_tx) and math.isfinite(spread_ty)):
        return False, 'non-finite residual spread — flatness is undefined'
    if max(spread_tx, spread_ty) > tol_rad:
        return (False,
                'field varies: spread tx {:.6f} rad, ty {:.6f} rad over {} '
                'nodes'.format(spread_tx, spread_ty, len(usable)))
    return (True,
            'FLAT FIELD — all {} node residuals agree within {:.6f} rad '
            '(spread tx {:.2e}, ty {:.2e}). A real residual field spans '
            '~1e-2 rad across this workspace. The usual cause is that the '
            'platform NEVER MOVED: every node re-measured one stationary pose. '
            'Check that the wire was armed for the whole sweep and that the '
            'commanded poses were actually reached before trusting this map.'
            .format(len(usable), tol_rad, spread_tx, spread_ty))


# ── pure core: map document ──────────────────────────────────────────────


def build_map_document(x_mm: Sequence[float], y_mm: Sequence[float],
                       z_mm: float,
                       results: Dict[Tuple[int, int], ReadStats],
                       captured: Dict[str, Any],
                       n_reads: int,
                       home_ref: Tuple[float, float]) -> Dict[str, Any]:
    """Assemble the schema-v1 document from the per-node reductions.

    **Shipped residuals are ANCHOR-MEAN HOME-REFERENCED**: every stored node
    value is ``measured − home_ref`` per axis, where ``home_ref`` is
    :func:`anchor_mean_rad` over the home anchors interleaved through the
    sweep. Any constant in the measurements — a stale level reference above
    all — cancels out of every node identically: adding a constant to every
    measured residual (anchors included) produces a byte-identical document.
    Bilinear interpolation commutes with the constant exactly, so no lookup
    or apply-side change accompanies this.

    The home node ships **exactly 0.0 only when the anchors agree exactly**
    (one anchor, or a perfectly repeatable machine — the previous design's
    "subtracts itself" case, preserved bit-identically by ``_exact_mean``).
    With real anchor scatter it ships the home node's own departure from the
    anchor mean — sub-mrad, correct, and *deliberately* not forced to zero:
    zeroing it would push that same scatter onto every other node instead of
    splitting it, and would hide the one number that says how repeatable the
    machine was during the sweep.

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
            tx_row.append(round(stats.res_tx - float(home_ref[0]), 9))
            ty_row.append(round(stats.res_ty - float(home_ref[1]), 9))
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
            'Likely causes, in order: (a) a leg pinned on its stroke clamp at '
            'an extremity node, which corrupts that node only (the IK '
            'preflight bounds this but cannot see a mechanical bind); (b) a '
            'units error if this tool was edited. A stale level reference is '
            'NOT a candidate: the residuals are home-referenced, so any '
            'constant reference error cancels out of every node. The measured '
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


def uninstall_plan(environ: Optional[Dict[str, str]] = None
                   ) -> Tuple[List[str], Optional[str]]:
    """``(paths_to_move_aside, env_override)`` for ``--force-uninstall``.

    Moving aside only the **source-tree** file is not enough to uninstall a map.
    ``setup.py`` installs ``config/tilt_calibration.yaml`` into
    ``share/jugglebot/config/`` whenever it exists at build time (a conditional
    ``data_files`` row), and the resolver's order is env → source tree → ament
    share. So once ANY ``colcon build`` has run with the file present, removing
    the source copy just falls through to the share copy: the reload succeeds,
    the OLD map stays loaded, and the capture is refused with a message naming
    neither the cause nor the remedy. Every existing candidate has to go.

    **An env-override candidate is never touched.** When ``$JUGGLEBOT_TILT_CAL``
    is set it is the *only* candidate, and it points at a file the operator
    named explicitly — possibly outside the repo, possibly the one artefact they
    are protecting. Renaming it on their behalf is a surprise this tool has no
    mandate for, so it is returned as ``env_override`` for the caller to refuse
    on, with instructions.
    """
    env = (environ if environ is not None else os.environ).get(
        tilt_map.TILT_MAP_ENV)
    if env:
        return [], env
    return ([path for path in tilt_map.tilt_map_candidates(environ)
             if os.path.exists(path)], None)


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
                        read_gap_s: float, n_home_revisits: int = 0) -> float:
    """Wall-clock ETA for the dry-run plan.

    The ``+ 3`` covers the three non-grid visits every full capture makes:
    the end-of-sweep home anchor, the verification home reference
    (home-referenced scoring), and the return to centre.
    ``n_home_revisits`` adds the MID-SWEEP anchors, which cost a full
    move + dwell + read set each — on the 5x5 default that is five extra
    visits, and an ETA that omitted them would understate the capture by
    ~45 s at the C0 working point.
    """
    per_visit = move_s + dwell_s + n_reads * (EST_READ_S + read_gap_s)
    return (n_nodes + n_checks + n_home_revisits + 3) * per_visit


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
                           'be odd so (0,0) is a node — the home reference '
                           'every other node is shipped against).')
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
    timing.add_argument('--lean-gain', type=float, default=-1.0,
                        help='lean gain passed on every move (default '
                             '%(default)s = defer to the node config gain, '
                             'ships 0.6 — the precedent-proven transit shape; '
                             'an EXPLICIT 0.0 forces lean OFF). Lean shapes '
                             'the TRANSIT only: the lean window is zero at '
                             'both endpoints, so the terminal pose measured '
                             'here is identical at any gain.')
    timing.add_argument('--timeout-s', type=float, default=DEFAULT_TIMEOUT_S,
                        help='per-service-call timeout (default %(default)s).')
    timing.add_argument('--home-revisit-every', type=int,
                        default=DEFAULT_HOME_REVISIT_EVERY,
                        help='insert a home re-measure (an ANCHOR) after every '
                             'N non-home grid visits (default %(default)s; 0 '
                             'disables mid-sweep anchors). The start and end '
                             'home measurements are always anchors, and every '
                             'shipped residual is referenced to the MEAN of '
                             'all of them — which averages down the ~1-2 mrad '
                             'arrival-to-arrival variation measured on '
                             '2026-08-10. Each anchor costs one full '
                             'move+dwell+read visit.')

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
                      help='if a map is loaded, move EVERY existing map '
                           'candidate aside (source tree AND any ament share '
                           'copy, each to a timestamped .bak; refuses a '
                           '$JUGGLEBOT_TILT_CAL-pointed file) and reload so '
                           'the capture starts with tilt_map_loaded=false.')
    misc.add_argument('--dry-run', action='store_true',
                      help='print the node sequence and ETA; make ZERO ROS '
                           'calls and construct no ROS objects.')
    misc.add_argument('--yes', action='store_true',
                      help='skip the interactive operator confirmations '
                           '(rehearsal only — the confirmations are the '
                           'constant-correction and hand-quiescent '
                           'preconditions).')
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
    lean_desc = ('{} (defer to node config, ships 0.6)'.format(args.lean_gain)
                 if args.lean_gain < 0 else str(args.lean_gain))
    print('  move={:.2f}s  dwell={:.2f}s  n_reads={}  read_gap={:.2f}s  '
          'lean_gain={}'.format(args.move_duration_s, args.dwell_s,
                                args.n_reads, args.read_gap_s, lean_desc))
    revisits = home_revisit_after_visits(len(order), args.home_revisit_every)
    print('  home anchors: {} total (start + {} mid-sweep + end) — the shipped '
          'reference is their MEAN'
          .format(len(revisits) + 2, len(revisits)))
    print('  visit order (home node FIRST, then centre-out — corners LAST):')
    for i, n in enumerate(order):
        tag = ('  <- home node (home reference, anchor 1 of {})'
               .format(len(revisits) + 2) if i == 0 else '')
        print('    {:>3d}. ({:+7.1f}, {:+7.1f}) [ix={}, iy={}]{}'
              .format(i + 1, n.x_mm, n.y_mm, n.ix, n.iy, tag))
        if i in revisits:
            print('         -> HOME ANCHOR ({:+7.1f}, {:+7.1f}) [phase {}]'
                  .format(0.0, 0.0, PHASE_HOME_ANCHOR))
    print('         -> HOME ANCHOR ({:+7.1f}, {:+7.1f}) [phase {}, '
          'end of sweep]'.format(0.0, 0.0, PHASE_HOME_END))
    print('  verification check poses (off-node, cell centres; scored '
          'home-referenced):')
    for i, (cx, cy) in enumerate(checks):
        print('    {:>3d}. ({:+7.1f}, {:+7.1f})'.format(i + 1, cx, cy))
    eta = estimate_duration_s(len(order), len(checks), args.move_duration_s,
                              args.dwell_s, args.n_reads, args.read_gap_s,
                              len(revisits))
    print('  ETA ~ {:.0f} s ({:.1f} min) including {} mid-sweep home anchor(s), '
          'the end-of-sweep anchor, the verification home reference and the '
          'return to centre'.format(eta, eta / 60.0, len(revisits)))


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
        from std_msgs.msg import Float64MultiArray
        from std_srvs.srv import Trigger

        self.node = node
        self.args = args
        self.status = None
        self.status_stamp = 0.0
        self.link_kv = {}
        self.link_stamp = 0.0
        # Monotonic count of /link_status messages consumed. `refresh_link` waits
        # for this to ADVANCE rather than sleeping a fixed interval, so a
        # mid-sweep wire check reads a message that provably post-dates the
        # request instead of whatever happened to be cached.
        self.link_seq = 0
        self.uptime_first = None
        self.uptime_last = None
        self.pose_offset_rad = None
        # Forensics state: last per-leg motor summary from /robot_state, plus a
        # FIRST-nonzero-error edge latch per leg. The latch matters because the
        # on-change DIAGNOSTIC feed can coalesce, and the abort-time snapshot
        # shows abort-time state, not fault-time state — the edge is the datum
        # the 2026-08-09 sitting only had because a bag happened to be rolling.
        self.motor_states = None
        self.motor_error_edge: Dict[int, Dict[str, Any]] = {}
        # Constancy monitor: /gravity_offset traffic. Any message between the
        # baseline mark (set when the sweep begins) and the last read violates
        # the constant-correction capture precondition, CAUSALLY observed.
        self.gravity_msg_count = 0
        self.gravity_baseline = None
        self.last_gravity = None

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
        node.create_subscription(Float64MultiArray, '/gravity_offset',
                                 self._on_gravity_offset, 10)

    # -- subscriptions ----------------------------------------------------

    def _on_status(self, msg):
        self.status = msg
        self.status_stamp = time.time()

    def _on_link(self, msg):
        kv = {v.key: v.value for v in msg.values}
        self.link_kv = kv
        self.link_stamp = time.time()
        self.link_seq += 1
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
        states = list(getattr(msg, 'motor_states', []) or [])
        if not states:
            return
        summary = []
        for i, ms in enumerate(states):
            entry = {
                'axis': i,
                'current_state': int(getattr(ms, 'current_state', 0)),
                'active_errors': int(getattr(ms, 'active_errors', 0)),
                'disarm_reason': int(getattr(ms, 'disarm_reason', 0)),
                'pos_estimate': float(getattr(ms, 'pos_estimate',
                                              float('nan'))),
                'vel_estimate': float(getattr(ms, 'vel_estimate',
                                              float('nan'))),
                'iq_measured': float(getattr(ms, 'iq_measured',
                                             float('nan'))),
                'bus_voltage': float(getattr(ms, 'bus_voltage',
                                             float('nan'))),
            }
            summary.append(entry)
            if ((entry['active_errors'] or entry['disarm_reason'])
                    and i not in self.motor_error_edge):
                self.motor_error_edge[i] = {
                    't_wall': time.time(),
                    'iso': datetime.now().isoformat(timespec='milliseconds'),
                    'uptime_ms': self.uptime_last,
                    'active_errors': entry['active_errors'],
                    'disarm_reason': entry['disarm_reason'],
                }
        self.motor_states = summary

    def _on_gravity_offset(self, msg):
        self.gravity_msg_count += 1
        self.last_gravity = {
            't_wall': time.time(),
            'iso': datetime.now().isoformat(timespec='milliseconds'),
            'data': [float(v) for v in list(getattr(msg, 'data', []) or [])],
        }

    # -- forensics ---------------------------------------------------------

    def forensics(self) -> Dict[str, Any]:
        """Everything already flowing into this process, stashed and decoded.

        Zero service calls — this is a pure snapshot of the caches, safe to
        take on any abort path including a dead link. Per-leg ODrive error
        masks are dumped RAW and DECODED (``decode_error_bits``): the
        2026-08-09 sitting's leg-0 code survived only because a rosbag was
        rolling, and the next launch's BOOT pre-flight auto-clears the
        drive-side record.
        """
        now = time.time()

        def _decorate(entry):
            out = dict(entry)
            out['active_errors_decoded'] = decode_error_bits(
                entry.get('active_errors', 0))
            out['disarm_reason_decoded'] = decode_error_bits(
                entry.get('disarm_reason', 0))
            return out

        status = self.status
        status_snapshot = None
        if status is not None:
            status_snapshot = {}
            for field in ('mode', 'streaming', 'gravity_correction_loaded',
                          'tilt_map_loaded', 'tilt_map_version'):
                status_snapshot[field] = getattr(status, field, None)
        return {
            'wall_iso': datetime.now().isoformat(timespec='milliseconds'),
            'uptime_ms_last': self.uptime_last,
            'link_kv': dict(self.link_kv),
            'link_age_s': (round(now - self.link_stamp, 3)
                           if self.link_stamp else None),
            'motor_states': ([_decorate(e) for e in self.motor_states]
                             if self.motor_states else None),
            'motor_error_edges': {str(leg): _decorate(edge)
                                  for leg, edge in
                                  sorted(self.motor_error_edge.items())},
            'last_traj_status': status_snapshot,
            'status_age_s': (round(now - self.status_stamp, 3)
                             if self.status_stamp else None),
            'pose_offset_rad': self.pose_offset_rad,
            'last_gravity_offset': self.last_gravity,
            'gravity_msgs_total': self.gravity_msg_count,
            'gravity_msgs_since_sweep_start': (
                self.gravity_msg_count - self.gravity_baseline
                if self.gravity_baseline is not None else None),
        }

    def nan_read(self) -> TiltRead:
        """A stamped NaN read for the failure-path CSV rows."""
        return TiltRead(time.time(), time.monotonic(), self.uptime_last,
                        float('nan'), float('nan'))

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

    def wait_for_status(self, timeout_s: float = 5.0, predicate=None,
                        expected: str = ''):
        """Wait for a ``/trajectory/status``; optionally for one that SATISFIES.

        ``/trajectory/status`` is published on a **5 Hz timer** (a 0.2 s period),
        so the first message that arrives after a ``reload_tilt_map`` call can
        easily have been composed *before* the reload took effect. Reading that
        one as the readback made every post-reload assertion a coin flip: a
        completed capture could abort with "APPLIED THE WRONG FILE" naming the
        version it had just correctly replaced, and ``--force-uninstall`` could
        report a map "STILL loaded" that was already gone. Both send the
        operator chasing a shadowing bug that does not exist, after a
        four-minute sweep.

        With a ``predicate`` this keeps consuming SUCCESSIVE messages until one
        satisfies it, and only the **timeout** is a failure. That is sound in
        both directions: the expected state is terminal (nothing else is
        changing the node's map underneath us), so "never appeared within
        ``timeout_s``" is a genuine failure and "appeared on message 3" is a
        genuine success. ``timeout_s`` defaults to 5.0 — 25 status periods.
        """
        self.status = None
        deadline = time.time() + timeout_s
        last = None
        while time.time() < deadline:
            self.spin(0.05)
            status = self.status
            if status is None:
                continue
            last = status
            if predicate is None or predicate(status):
                return status
        if last is None:
            raise TiltCalError(
                'no /trajectory/status message in {:.1f} s — trajectory_node is '
                'not running, or jugglebot_interfaces was not rebuilt (a stale '
                'install makes trajectory_node exit ~200 ms after launch). '
                'Build BOTH packages: colcon build --packages-select '
                'jugglebot_interfaces jugglebot'.format(timeout_s))
        raise TiltCalStatusTimeout(
            'trajectory/status never reported {} within {:.1f} s (last seen: '
            'tilt_map_loaded={} version={!r})'
            .format(expected or 'the expected state', timeout_s,
                    last.tilt_map_loaded, last.tilt_map_version), last)

    def refresh_link(self, timeout_s: float = 2.0) -> Tuple[Dict[str, str], bool]:
        """``(kv, fresh)`` — wait for a NEW ``/link_status``, then return it.

        ``fresh`` is False when none arrived inside ``timeout_s`` (the cached kv
        is returned regardless).

        **A stale cache cannot be covered by the per-move check**, which is why
        the caller aborts on it. The ``DISARMED`` suffix is synchronous with the
        command, but it is derived from the *same* topic and no more current:
        ``trajectory_node._wire_armed`` is a bare mirror of the last
        ``/link_status`` it received, with **no staleness timeout**. So if
        ``teensy_bridge_node`` dies or its 10 Hz timer stalls mid-sweep — exactly
        when setpoints stop reaching the Teensy — both this check and the
        per-move marker freeze at their last value and keep reporting ARMED.
        """
        seen = self.link_seq
        deadline = time.time() + timeout_s
        while time.time() < deadline and self.link_seq == seen:
            self.spin(0.05)
        return self.link_kv, self.link_seq != seen

    def assert_wire_armed(self, where: str) -> None:
        """Re-check the wire from a FRESH ``/link_status``; abort if it dropped.

        **Layer 1 repeated between nodes**, not layer 2: the preflight kv check
        happens once and a four-minute sweep gives a wire plenty of time to be
        disarmed under it (an E-STOP, a mode change, a guard latch). Layer 2 —
        the per-move ``DISARMED`` suffix — is a separate mechanism screened in
        :meth:`go_to`. Both exist because neither covers the other's hole.

        A silent ``/link_status`` is itself an abort. See :meth:`refresh_link`:
        the per-move marker mirrors the same topic with no staleness timeout, so
        a stalled publisher takes out *both* layers at once and the capture would
        run to completion against an unobservable wire. This cannot false-fire on
        jitter — ``/link_status`` is a 10 Hz timer, so the 2 s window is twenty
        consecutive missed publishes.
        """
        self.assert_correction_constant(where)
        kv, fresh = self.refresh_link()
        if not fresh:
            raise TiltCalError(
                'no /link_status message in 2.0 s before {} — teensy_bridge_node '
                'has stopped publishing (that topic is a 10 Hz timer, so this is '
                'twenty missed publishes, not jitter). The go_to_pose DISARMED '
                'marker mirrors the SAME topic with no staleness timeout, so it '
                'would also freeze reporting ARMED. Refusing to continue against '
                'a wire this tool can no longer observe.'.format(where))
        ok, message = wire_armed_verdict(kv)
        if not ok:
            raise TiltCalError(
                'wire check FAILED before {}: {}'.format(where, message))

    def mark_sweep_started(self) -> None:
        """Arm the constancy monitor: /gravity_offset traffic from here on is
        a capture-precondition violation. Called once, after the prompts and
        before the first commanded motion."""
        self.gravity_baseline = self.gravity_msg_count

    def assert_correction_constant(self, where: str) -> None:
        """The CAUSAL half of capture precondition 1 (constant correction).

        Any ``/gravity_offset`` message after :meth:`mark_sweep_started` means
        the reference changed mid-sweep — a re-level, or a relaunch re-pushing
        the Teensy-persisted (int16-mrad truncated) copy — and every node
        measured after it is referenced differently from those before. That is
        observed directly here, not inferred from the drift it happens to
        leave, so a zig-zag that nets to zero drift still aborts.
        """
        if self.gravity_baseline is None:
            return
        n = self.gravity_msg_count - self.gravity_baseline
        if n:
            raise TiltCalFaultError(
                'LEVEL REFERENCE CHANGED MID-SWEEP (detected {}): {} '
                '/gravity_offset message(s) arrived since the sweep began '
                '(last: {}). The capture precondition is a CONSTANT '
                'correction — nodes measured after the change are referenced '
                'differently from those before, which home-referencing cannot '
                'cancel. Nothing usable was captured; re-run from the start.'
                .format(where, n, self.last_gravity))

    def assert_cached_wire_ok(self, where: str) -> None:
        """Mid-node fault check against the CACHED ``/link_status`` kv.

        Zero extra service calls and zero waiting: the ``spin(read_gap_s)``
        between reads is already pumping ``_on_link``, so the cache tracks the
        10 Hz publisher within ~0.1 s. On the 2026-08-09 leg-0 collapse this
        check — at arrival, before read 0 — would have caught the fault ~1 s
        after the latch instead of 6.1 s and 30 garbage reads later at the
        next node boundary. Raises :class:`TiltCalFaultError` so the per-node
        ``try`` cannot demote a latched fault to a failed node.
        """
        self.assert_correction_constant(where)
        ok, message = wire_armed_verdict(self.link_kv)
        if not ok:
            raise TiltCalFaultError(
                'FAULT DETECTED {} — {}'.format(where, message))

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
        # Layer 2 of the disarmed-wire guard, and the authoritative one: this
        # marker travels on the response to THIS command, so unlike the
        # /link_status cache it cannot be stale. Discarding it is what let a
        # whole capture complete against a platform that never moved.
        if response_reports_disarmed(response.message):
            raise TiltCalFaultError(
                'go_to_pose was ACCEPTED for {} at ({:+.1f}, {:+.1f}, {:.1f}) '
                'but trajectory_node reports the wire is DISARMED :: {}\n'
                '  Acceptance here is a PLANNING verdict — the setpoints are '
                'emitted and dropped, so the platform does NOT move. Every node '
                'from here would re-measure one stationary pose and this tool '
                'would write, apply and "verify" an all-zeros calibration that '
                'passes because nothing moved (2026-07-15 precedent). Arm the '
                'wire and re-run.'.format(label, x_mm, y_mm, z_mm,
                                          response.message))
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
                ) -> Tuple[ReadStats, List[TiltRead]]:
        """Move, dwell, then N spaced sequential reads. Returns (stats, reads).

        Each read carries ITS OWN wall/monotonic/uptime stamps (the 2026-08-09
        CSVs shared one timestamp per node, which cost the fault forensics),
        and the cached-kv fault check runs at ARRIVAL and after EVERY read —
        a mid-move or mid-read guard latch aborts within one read of the
        cache seeing it, naming the read index, instead of surfacing at the
        next node boundary after a full set of garbage reads.
        """
        planned = self.go_to(x_mm, y_mm, z_mm, label)
        self.spin(planned + self.args.dwell_s)
        reads: List[TiltRead] = []
        try:
            self.assert_cached_wire_ok('on arrival at {}'.format(label))
            for i in range(self.args.n_reads):
                if i:
                    self.spin(self.args.read_gap_s)
                tx, ty = self.read_tilt_once()
                reads.append(TiltRead(time.time(), time.monotonic(),
                                      self.uptime_last, tx, ty))
                self.assert_cached_wire_ok(
                    'after read {} at {}'.format(i, label))
        except TiltCalFaultError as exc:
            # The stamped reads taken BEFORE the fault check fired are
            # primary evidence (on 2026-08-09 the collapsed-platform tilt
            # series was exactly what the analysis needed) — attach them so
            # the caller can write them to the CSV before re-raising.
            exc.reads = reads
            raise
        return (residual_from_reads([(r.tx, r.ty) for r in reads],
                                    offset_rad), reads)


def forensics_has_drive_error(fx: Dict[str, Any]) -> bool:
    """Does this snapshot actually contain a nonzero ODrive error/disarm code?

    Gates the "capture evidence BEFORE any relaunch" epilogue. That line used
    to print on EVERY abort, including the two 2026-08-10 C0 gate aborts whose
    forensics were entirely clean — telling an operator to preserve
    drive-side evidence that does not exist trains them to skip the line on
    the run where it matters, which is the one abort where the evidence is
    genuinely perishable (the next launch's BOOT pre-flight clears it).
    """
    for entry in (fx.get('motor_states') or []):
        if entry.get('active_errors') or entry.get('disarm_reason'):
            return True
    for edge in (fx.get('motor_error_edges') or {}).values():
        if edge.get('active_errors') or edge.get('disarm_reason'):
            return True
    return False


def print_forensics(fx: Dict[str, Any]) -> None:
    """Console rendering of a :meth:`_Runner.forensics` snapshot.

    Printed on EVERY abort, before the meta is written: the operator standing
    at the robot needs the decoded ODrive error names NOW (the next launch's
    BOOT pre-flight auto-clears the drive-side record), and the meta JSON is
    for the session that reads the artifacts later.
    """
    print('\n-- forensics ------------------------------------------------',
          file=sys.stderr)
    kv = fx.get('link_kv') or {}
    print('  link_kv: fault_state={} mpc_active={} bridge_link={} '
          'uptime_ms={} (age {} s)'.format(
              kv.get('fault_state'), kv.get('mpc_active'),
              kv.get('bridge_link'), kv.get('uptime_ms'),
              fx.get('link_age_s')), file=sys.stderr)
    for key in ('guard_fault_leg', 'live_deviation', 'max_dev_leg',
                'max_dev_value'):
        if key in kv:
            print('  link_kv: {}={}'.format(key, kv[key]), file=sys.stderr)
    states = fx.get('motor_states') or []
    for entry in states:
        if entry.get('active_errors') or entry.get('disarm_reason'):
            print('  leg {}: state={} active_errors={} {} disarm_reason={} '
                  '{} pos={:.3f} vel={:.3f} iq={:.2f} vbus={:.1f}'.format(
                      entry.get('axis'), entry.get('current_state'),
                      entry.get('active_errors'),
                      entry.get('active_errors_decoded'),
                      entry.get('disarm_reason'),
                      entry.get('disarm_reason_decoded'),
                      entry.get('pos_estimate', float('nan')),
                      entry.get('vel_estimate', float('nan')),
                      entry.get('iq_measured', float('nan')),
                      entry.get('bus_voltage', float('nan'))),
                  file=sys.stderr)
    if states and not any(e.get('active_errors') or e.get('disarm_reason')
                          for e in states):
        print('  motor_states: all {} axes error-clean at abort time'
              .format(len(states)), file=sys.stderr)
    edges = fx.get('motor_error_edges') or {}
    for leg, edge in edges.items():
        print('  FIRST-ERROR EDGE leg {}: {} active_errors={} {} '
              'disarm_reason={} {} (uptime_ms {})'.format(
                  leg, edge.get('iso'), edge.get('active_errors'),
                  edge.get('active_errors_decoded'),
                  edge.get('disarm_reason'),
                  edge.get('disarm_reason_decoded'),
                  edge.get('uptime_ms')), file=sys.stderr)
    print('  traj status: {} (age {} s)  pose_offset_rad: {}'.format(
        fx.get('last_traj_status'), fx.get('status_age_s'),
        fx.get('pose_offset_rad')), file=sys.stderr)
    if fx.get('gravity_msgs_since_sweep_start'):
        print('  /gravity_offset DURING SWEEP: {} message(s), last {}'.format(
            fx.get('gravity_msgs_since_sweep_start'),
            fx.get('last_gravity_offset')), file=sys.stderr)
    if forensics_has_drive_error(fx):
        print('  full snapshot lands in _meta.json (forensics key). CAPTURE '
              'EVIDENCE BEFORE ANY RELAUNCH: a nonzero ODrive error/disarm '
              'code is present above, and the next launch BOOT pre-flight '
              'auto-clears the drive-side record.', file=sys.stderr)
    else:
        print('  full snapshot lands in _meta.json (forensics key). No ODrive '
              'error or disarm code in this snapshot — the drives were clean '
              'at abort time, so nothing drive-side is perishable here.',
              file=sys.stderr)
    print('---------------------------------------------------------------',
          file=sys.stderr)


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
    started_mono = time.monotonic()
    rc = 0
    node_summaries: List[Dict[str, Any]] = []
    check_summaries: List[Dict[str, Any]] = []
    reached_motion = False
    z_mm = float(args.z)
    abort_reason: Optional[str] = None
    home_meta: Optional[Dict[str, Any]] = None

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
               reads: Sequence[TiltRead], note: str = '') -> None:
        # Per-read stamps, not one per node: iso from each read's own wall
        # clock, t_s from its monotonic clock, uptime_ms as cached AT that
        # read. The 2026-08-09 CSVs stamped all 30 rows post-loop, which made
        # intra-node fault timing unrecoverable from the artifact.
        for k, r in enumerate(reads):
            iso = datetime.fromtimestamp(r.t_wall).isoformat(
                timespec='milliseconds')
            emit(csv_row(iso, r.t_mono - started_mono, phase, visit, ix, iy,
                         x, y, z_mm, k, r.tx, r.ty, offset_rad,
                         r.uptime_ms, note))

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

        # Layer 1 of the disarmed-wire guard. mode=TRAJECTORY + streaming=True
        # says the Jetson is EMITTING; it says nothing about whether the bridge
        # is forwarding. Only /link_status mpc_active does, and it was already
        # subscribed and cached here for uptime_ms without ever being read.
        wire_ok, wire_message = wire_armed_verdict(runner.link_kv)
        if not wire_ok:
            raise TiltCalError(wire_message)
        print('  {}'.format(wire_message))

        if not status.gravity_correction_loaded:
            raise TiltCalError(
                'trajectory/status gravity_correction_loaded=false — capture '
                'precondition 1 (C-LEVEL-2) requires a correction LOADED and '
                'CONSTANT for the sweep. Home-referencing makes freshness '
                'optional, but "no correction at all" is not a constant '
                'correction: the map would later be applied on top of a live '
                'one this capture never saw. Run `level` from IDLE (over '
                '/orchestrator_command, not the /activate service), then '
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
                to_move, env_override = uninstall_plan()
                if env_override:
                    raise TiltCalError(
                        '--force-uninstall refuses to touch an env-pointed map. '
                        '${} is set to {}, which makes it the ONLY candidate — '
                        'and it names a file you chose, possibly outside this '
                        'repo. Either  unset {}  (the source tree and the ament '
                        'share copy then become the candidates this tool can '
                        'manage) or move that file aside yourself, then re-run.'
                        .format(tilt_map.TILT_MAP_ENV, env_override,
                                tilt_map.TILT_MAP_ENV))
                if not to_move:
                    print('  no candidate file exists to move aside (tried: {})'
                          .format('; '.join(tilt_map.tilt_map_candidates())
                                  or '<none>'))
                failures = []
                for path in to_move:
                    backup = '{}.{}.bak'.format(path, stamp)
                    try:
                        os.rename(path, backup)
                    except OSError as exc:
                        failures.append('{} ({})'.format(path, exc))
                        print('  ! could NOT move {} -> {}: {}'
                              .format(path, backup, exc))
                        continue
                    print('  moved {} -> {}'.format(path, backup))
                if failures:
                    print('  ! {} candidate(s) could not be moved; the reload '
                          'below will say whether one of them is still being '
                          'resolved.'.format(len(failures)))
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
                try:
                    status = runner.wait_for_status(
                        predicate=lambda s: not s.tilt_map_loaded,
                        expected='tilt_map_loaded=false')
                except TiltCalStatusTimeout as exc:
                    still = tilt_map.resolve_tilt_map_path()
                    raise TiltCalError(
                        'map is STILL loaded after --force-uninstall (version '
                        '{!r}) — do not capture: the loaded map would bake '
                        'itself into its own successor.\n'
                        '  This tool resolves {} as the current map file.\n'
                        '  The usual cause is an AMENT SHARE COPY: setup.py '
                        'installs config/tilt_calibration.yaml into '
                        'share/jugglebot/config/ whenever it exists at build '
                        'time, and the resolver falls through to it once the '
                        'source-tree file is gone. Remove or rename that copy '
                        '(or re-run colcon build with the source file absent), '
                        'then re-run. Candidates tried: {}.\n'
                        '  If the node runs with a different ${} than this '
                        'shell, they are resolving different files — check both '
                        'sides.'
                        .format(getattr(exc.last_status, 'tilt_map_version',
                                        '<unknown>'),
                                still if still else '<no existing candidate>',
                                '; '.join(tilt_map.tilt_map_candidates())
                                or '<none>',
                                tilt_map.TILT_MAP_ENV))
                print('  tilt_map_loaded=False (uninstalled)')
            elif status.tilt_map_loaded:
                raise TiltCalError(
                    'a tilt map is loaded (version {!r}) and capture must run '
                    'with NO map: a loaded map shapes the very poses being '
                    'measured, so capturing under it bakes that map into its '
                    'own successor and the error compounds silently on every '
                    'recapture (home-referencing cancels a constant offset, '
                    'not a pose-dependent one). Re-run with --force-uninstall '
                    'to move every map candidate aside (source tree and ament '
                    'share copy) and reload.'
                    .format(status.tilt_map_version))
            else:
                print('  tilt_map_loaded=False')

        if runner.pose_offset_rad is None:
            print('  ! /robot_state.pose_offset_rad not seen — recording the '
                  'level offset as "unknown" in the map metadata.')
        else:
            print('  level offset (bridge /robot_state cache; a fresh level '
                  'is full-float, only a relaunch re-push is 1 mrad '
                  'quantised) = {} rad'.format(runner.pose_offset_rad))

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

        # Under home-referencing a nonzero m_home is EXPECTED and harmless at
        # any height — it cancels out of every shipped residual. At z != 170
        # (the ACTIVATE pose `level` measures) m_home additionally contains a
        # REAL pose-dependent component, which is fine for the same reason;
        # only the |m_home| WARN/abort bounds and the anchor-series gates
        # apply. Say so, so a WARN at an unusual height reads correctly.
        if not args.verify_only and abs(float(args.z) - DEFAULT_Z_MM) > 1e-6:
            print('  ! --z {} is not the ACTIVATE height {}: expect a larger '
                  '|m_home| (it now contains a real pose-dependent term). '
                  'Harmless — the map is home-referenced — but read a '
                  '0.010 rad WARN with that in mind.'
                  .format(args.z, DEFAULT_Z_MM))

        base_condition = args.base_condition
        if args.yes and not args.verify_only:
            print('  ! --yes: SKIPPING the constancy and hand-quiescent '
                  'confirmations on a LIVE capture. Hand quiescence has no '
                  'programmatic check at all; correction constancy IS '
                  'machine-checked (mid-sweep /gravity_offset abort + the '
                  'home-anchor series), but only after motion has '
                  'been spent on the violation.')
        if not args.yes:
            thc.safety_gate(
                'CONSTANT-CORRECTION precondition: will the loaded gravity '
                'correction stay UNCHANGED for the whole sweep (no re-level, '
                'no relaunch, no /gravity_offset push mid-capture)?\n'
                '  The map is HOME-REFERENCED, so a stale-but-CONSTANT '
                'correction cancels exactly — freshness is RECOMMENDED (it '
                'keeps |m_home| small), not required. A correction that '
                'CHANGES mid-sweep corrupts every node measured after the '
                'change; the tool machine-checks this (mid-sweep '
                '/gravity_offset abort, plus the home-anchor series, which '
                'aborts on a discrete step between consecutive anchors).',
                'yes')
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

        # IK stroke-margin preflight — BEFORE any motion, every pose this run
        # will command (grid nodes, check poses, home). A pose on a stroke
        # clamp does not fail loudly at capture time: the firmware clamps per
        # leg at 500 Hz and the node's reading is silently wrong.
        preflight_poses = ([(nd.x_mm, nd.y_mm) for nd in order]
                           + [(cx, cy) for cx, cy in checks]
                           + [(0.0, 0.0)])
        stroke_problems = stroke_margin_problems(preflight_poses, z_mm)
        if stroke_problems:
            raise TiltCalError(
                'IK stroke-margin preflight REFUSED {} pose(s) — worst leg '
                'within {:.0f} mm of the [5, 275] mm hard extension bound:\n'
                '  {}\nShrink --box / adjust --x/--y/--z. No motion was '
                'commanded.'.format(len(stroke_problems),
                                    STROKE_PREFLIGHT_MARGIN_MM,
                                    '\n  '.join(stroke_problems)))
        print('  IK stroke preflight OK: every pose keeps >= {:.0f} mm of '
              'leg-extension margin ({} poses checked)'
              .format(STROKE_PREFLIGHT_MARGIN_MM, len(preflight_poses)))

        results: Dict[Tuple[int, int], ReadStats] = {}
        failed_nodes: List[Tuple[int, int]] = []
        anchors: List[HomeAnchor] = []
        # Mid-sweep home anchors: grid-visit indices after which the platform
        # returns to (0, 0) and re-measures. The start (visit 0) and the
        # end-of-sweep re-measure are anchors too, so this list is the
        # ADDITIONAL ones.
        revisit_after = set(home_revisit_after_visits(
            len(order), args.home_revisit_every))
        reached_motion = True
        # Arm the constancy monitor: any /gravity_offset message from here to
        # the last read is a capture-precondition violation and aborts.
        runner.mark_sweep_started()

        def measure_home_anchor(index: int, phase: str,
                                label: str) -> HomeAnchor:
            """Move home, re-measure, record the CSV rows, bank the anchor.

            Used for every anchor after the first (which is the home GRID
            visit). The rows are tagged ``home_anchor`` / ``home_end`` rather
            than ``capture`` so neither this tool nor the analyser can fold a
            re-arrival into the home node's own residual.
            """
            hx, hy = home_index(x_mm, y_mm)
            runner.assert_wire_armed(label)
            stats, reads = runner.measure(0.0, 0.0, z_mm, label, offset_rad)
            record(phase, index, hx, hy, 0.0, 0.0, reads)
            if stats.n_ok < MIN_GOOD_READS:
                raise TiltCalError(
                    '{} returned only {}/{} finite reads — the anchor series '
                    'is the map\'s reference, so a dead anchor cannot be '
                    'skipped over. Nothing was written; re-run (a longer '
                    '--dwell-s is the usual fix).'
                    .format(label, stats.n_ok, stats.n_total))
            t_s = (reads[0].t_mono - started_mono) if reads else (
                time.monotonic() - started_mono)
            anchor = HomeAnchor.from_stats(t_s, stats)
            anchors.append(anchor)
            print('  home anchor {}: m = {} sd ({:.6f}, {:.6f}) rad  '
                  'reads {}/{}'
                  .format(len(anchors), _fmt_deg(anchor.m_tx, anchor.m_ty),
                          anchor.sd_tx, anchor.sd_ty, stats.n_ok,
                          stats.n_total))
            return anchor

        # ── capture ──────────────────────────────────────────────────
        if order:
            print('\n== capture ({} nodes, {} home anchors) =='
                  .format(len(order), len(revisit_after) + 2))
        for visit, nd in enumerate(order):
            label = 'node {}/{} ({:+.1f}, {:+.1f})'.format(
                visit + 1, len(order), nd.x_mm, nd.y_mm)
            # DELIBERATELY outside the try below: that handler converts a
            # TiltCalError into a failed *node* and, under the default
            # --on-fail continue, carries on. A wire that disarmed mid-sweep is
            # never "this one node failed" — every remaining node would
            # re-measure the same stationary pose — so it must abort the run.
            runner.assert_wire_armed(label)
            try:
                stats, reads = runner.measure(nd.x_mm, nd.y_mm, z_mm, label,
                                              offset_rad)
            except TiltCalFaultError as exc:
                # A latched fault / disarmed wire / mid-sweep re-level is a
                # RUN-level abort, never "this one node failed" — demoting it
                # under --on-fail continue is how 30 reads ran against a
                # collapsed platform on 2026-08-09. Bank the partial reads
                # first: they are the tilt series at the fault edge.
                record('capture', visit, nd.ix, nd.iy, nd.x_mm, nd.y_mm,
                       getattr(exc, 'reads', None) or [runner.nan_read()],
                       note=str(exc)[:200])
                raise
            except TiltCalError as exc:
                print('  [FAIL] {} :: {}'.format(label, exc))
                record('capture', visit, nd.ix, nd.iy, nd.x_mm, nd.y_mm,
                       [runner.nan_read()], note=str(exc)[:200])
                failed_nodes.append((nd.ix, nd.iy))
                # A home-node failure is fatal whatever --on-fail says: the
                # home node IS the reference every other node is shipped
                # against, and `continue` would jump past it and spend four
                # more minutes capturing a grid that build_map_document will
                # refuse anyway for the missing node. Aborting early is the
                # entire reason home is measured first.
                if visit == 0 or args.on_fail == 'abort':
                    raise
                rc = 1
                continue

            record('capture', visit, nd.ix, nd.iy, nd.x_mm, nd.y_mm, reads)
            results[(nd.ix, nd.iy)] = stats
            # Appended per node, not in a trailing loop: on ANY mid-sweep
            # abort (a fault, the |m_home| sanity abort, later the drift
            # gate) a trailing loop never runs and _meta.json would ship an
            # empty 'nodes', exactly when the operator most needs the
            # per-node reduction retained.
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
                if visit == 0:
                    raise TiltCalError(
                        'home node produced only {}/{} usable reads — it is '
                        'the reference every other node is shipped against, '
                        'so the sweep cannot produce a map; aborting before '
                        'four minutes of motion reach the same refusal at '
                        'build time'.format(stats.n_ok, stats.n_total))
                if args.on_fail == 'abort':
                    raise TiltCalError(
                        '{} produced only {}/{} usable reads and --on-fail='
                        'abort'.format(label, stats.n_ok, stats.n_total))
                rc = 1

            if visit == 0:
                verdict, message = home_reference_verdict(stats)
                if verdict == 'warn':
                    print('  !! home reference WARN: {}'.format(message))
                else:
                    print('  home reference: {}'.format(message))
                if verdict == 'abort':
                    raise TiltCalError(message)
                # ANCHOR 1. The home grid visit doubles as the first anchor —
                # same move, same dwell, same read count as every other one.
                anchors.append(HomeAnchor.from_stats(
                    (reads[0].t_mono - started_mono) if reads
                    else (time.monotonic() - started_mono), stats))

            if visit in revisit_after:
                measure_home_anchor(
                    visit, PHASE_HOME_ANCHOR,
                    'home anchor {} (mid-sweep, after visit {}/{})'
                    .format(len(anchors) + 1, visit + 1, len(order)))

        # ── end-of-sweep home anchor + the anchor-series REPORT ──────
        # The anchors are the map's reference AND the record that settles
        # whether the home pose's ~1-2 mrad re-arrival offset (both 2026-08-10
        # C0 captures) is arrival repeatability or sensor warm-up. The report
        # prints on every capture; only a 0.5 deg spread or a 5 mrad
        # consecutive step aborts. The causal /gravity_offset monitor remains
        # the detector for a correction that actually changed. Runs for
        # --no-apply too — C0 is exactly where these statistics come from.
        home_ref = (0.0, 0.0)
        if not args.verify_only and order:
            hx, hy = home_index(x_mm, y_mm)
            if results.get((hx, hy)) is None:
                raise TiltCalError(
                    'the home node has no usable measurement — cannot '
                    'home-reference the map (this should have aborted at '
                    'visit 0).')
            print('\n== home anchor (end of sweep) ==')
            measure_home_anchor(len(order), PHASE_HOME_END,
                                'home anchor {} (end of sweep)'
                                .format(len(anchors) + 1))

            spread = anchor_spread(anchors)
            anchor_verdict, anchor_message = anchor_series_verdict(anchors)
            home_ref = anchor_mean_rad(anchors)
            print('\n== home anchor series ({} anchors, reference = their '
                  'MEAN) =='.format(len(anchors)))
            for line in anchor_table_lines(anchors):
                print(line)
            print('  anchor mean (the shipped reference) = {}'
                  .format(_fmt_deg(*home_ref)))
            if anchor_verdict == 'abort':
                home_meta = {
                    'anchors': [a.as_dict() for a in anchors],
                    'anchor_verdict': 'ABORT',
                    'referenced': 'anchor-mean',
                }
                raise TiltCalError(anchor_message)
            if anchor_verdict == 'warn':
                print('\n  !! {}'.format(anchor_message))
            else:
                print('  {}'.format(anchor_message))

            home_meta = {
                'referenced': 'anchor-mean',
                'anchors': [a.as_dict() for a in anchors],
                'anchor_mean_rad': [home_ref[0], home_ref[1]],
                'anchor_pp_rad': [spread['tx']['pp'], spread['ty']['pp']],
                'trend_rad': [spread['tx']['trend'], spread['ty']['trend']],
                'anchor_max_step_rad': [spread['tx']['max_step'],
                                        spread['ty']['max_step']],
                'anchor_verdict': anchor_verdict.upper(),
                # Kept for continuity with pre-anchor captures and the
                # analyser's older readers: the FIRST and LAST anchors under
                # their previous names. They are no longer the reference.
                'm_home_rad': [anchors[0].m_tx, anchors[0].m_ty],
                'sd_home_rad': [anchors[0].sd_tx, anchors[0].sd_ty],
                'm_home_end_rad': [anchors[-1].m_tx, anchors[-1].m_ty],
                'sd_home_end_rad': [anchors[-1].sd_tx, anchors[-1].sd_ty],
            }

        # ── write + apply ────────────────────────────────────────────
        if not args.verify_only:
            # The two capture preconditions were checked once, at preflight,
            # four minutes ago. Both can change UNDER a sweep — someone reloads
            # a map, a relaunch drops the per-process level correction — and
            # either one invalidates every residual just measured. Re-fetch a
            # FRESH status and refuse to write rather than shipping a map
            # captured under conditions that no longer held. Aborting here still
            # routes through the ordinary return-to-centre path, and the CSV and
            # _meta.json are still written, so nothing is lost but the map.
            status = runner.wait_for_status()
            if status.tilt_map_loaded:
                raise TiltCalError(
                    'a tilt map became LOADED during the sweep (version {!r}) — '
                    'refusing to write. A map shapes the poses being measured, '
                    'so a map that came up mid-capture makes some nodes mean '
                    'one thing and the rest another (a pose-dependent change; '
                    'home-referencing cancels only constants). Nothing was '
                    'written; re-run from a clean state (--force-uninstall).'
                    .format(status.tilt_map_version))
            if not status.gravity_correction_loaded:
                raise TiltCalError(
                    'gravity_correction_loaded went FALSE during the sweep — '
                    'refusing to write. The capture precondition is a '
                    'correction loaded and CONSTANT; a correction that '
                    'DISAPPEARED mid-sweep is the constancy violation in its '
                    'bluntest form (nodes before and after are referenced '
                    'differently). The usual cause is a trajectory_node '
                    'relaunch mid-capture (the correction is per-PROCESS). '
                    'Re-run `level` from IDLE and start the capture over.')

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
                'level_offset_source': ('robot_state.pose_offset_rad (bridge '
                                        'cache; fresh level = full float, '
                                        'relaunch re-push = 1 mrad quantised)'),
                'home_reference': home_meta,
                'base_condition': base_condition,
            }
            doc = build_map_document(x_mm, y_mm, z_mm, results, captured,
                                     args.n_reads, home_ref)
            validate_map_document(doc)
            expected_version = tilt_map.map_version(doc)

            # Layer 3 of the disarmed-wire guard: the SYMPTOM check. Layers 1
            # and 2 watch the wire this tool can see; this one watches the data
            # and so also catches a platform that failed to move for a reason
            # neither of them observes. WARN, never refuse — a genuinely flat
            # field is a legitimate (excellent) result, and refusing it would
            # make this tool unable to certify the best possible outcome.
            is_flat, flat_message = flat_field_verdict(results)
            if is_flat:
                print('\n  !! {}'.format(flat_message))

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

                # Poll for the EXPECTED version rather than trusting the first
                # status to arrive: at 5 Hz that message can have been composed
                # before the reload landed, which made this abort — after a
                # completed capture — a coin flip.
                try:
                    status = runner.wait_for_status(
                        predicate=(lambda s: s.tilt_map_loaded
                                   and s.tilt_map_version == expected_version),
                        expected='tilt_map_loaded=true version={!r}'
                                 .format(expected_version))
                except TiltCalStatusTimeout as exc:
                    last = exc.last_status
                    if last is not None and not last.tilt_map_loaded:
                        raise TiltCalError(
                            'reload reported success but trajectory/status says '
                            'tilt_map_loaded=false — nothing is applied.')
                    raise TiltCalError(
                        'APPLIED THE WRONG FILE. trajectory_node reports '
                        'version {!r} but this tool wrote {!r} to {}. The node '
                        'resolved a different candidate path than this tool '
                        'wrote to — check ${} on both sides, and whether an '
                        'ament share copy is shadowing the source tree '
                        '(setup.py installs one whenever the file exists at '
                        'build time). Candidates from here: {}.'
                        .format(getattr(last, 'tilt_map_version', '<unknown>'),
                                expected_version, target,
                                tilt_map.TILT_MAP_ENV,
                                '; '.join(tilt_map.tilt_map_candidates())
                                or '<none>'))
                print('  applied and CONFIRMED: version {}'
                      .format(status.tilt_map_version))

        # ── verification pass — HOME-REFERENCED scoring ──────────────
        # With the map applied, a check pose measures ≈ m_home even when the
        # map is perfect (the constant the capture correctly tolerated is
        # still in the loop), so home is re-measured FIRST and subtracted
        # from every check. Scoring the raw check residual would fail every
        # pose under a stale-but-constant level whose capture was valid.
        verify_ref = (0.0, 0.0)
        if checks:
            label = 'verification home reference'
            print('\n== verification home reference ==')
            runner.assert_wire_armed(label)
            ref_stats, ref_reads = runner.measure(0.0, 0.0, z_mm, label,
                                                  offset_rad)
            record('verify_home', -1, -1, -1, 0.0, 0.0, ref_reads)
            if not node_is_good(ref_stats, args.n_reads):
                raise TiltCalError(
                    'the verification home reference produced only {}/{} '
                    'usable reads — check poses cannot be scored without it'
                    .format(ref_stats.n_ok, ref_stats.n_total))
            verify_ref = (ref_stats.res_tx, ref_stats.res_ty)
            print('  m_home(verify) = {} — subtracted from every check pose'
                  .format(_fmt_deg(*verify_ref)))

        print('\n== verification ({} off-node check poses, threshold {:.3f} '
              'deg, home-referenced) =='.format(len(checks),
                                                args.threshold_deg))
        n_fail = 0
        for i, (cx, cy) in enumerate(checks):
            label = 'check {}/{} ({:+.1f}, {:+.1f})'.format(
                i + 1, len(checks), cx, cy)
            # Same placement rationale as the capture loop: outside the try, so
            # a mid-verification disarm aborts rather than scoring a PASS at a
            # pose the platform never reached.
            runner.assert_wire_armed(label)
            try:
                stats, reads = runner.measure(cx, cy, z_mm, label, offset_rad)
            except TiltCalFaultError as exc:
                record('verify', i, -1, -1, cx, cy,
                       getattr(exc, 'reads', None) or [runner.nan_read()],
                       note=str(exc)[:200])
                raise
            except TiltCalError as exc:
                print('  [FAIL] {} :: {}'.format(label, exc))
                record('verify', i, -1, -1, cx, cy,
                       [runner.nan_read()], note=str(exc)[:200])
                n_fail += 1
                check_summaries.append({
                    'x_mm': cx, 'y_mm': cy, 'verdict': 'FAIL',
                    'error': str(exc)[:200], 'n_ok': 0,
                })
                if args.on_fail == 'abort':
                    raise
                continue
            record('verify', i, -1, -1, cx, cy, reads)
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
            ref_tx = stats.res_tx - verify_ref[0]
            ref_ty = stats.res_ty - verify_ref[1]
            magnitude = _magnitude_deg(ref_tx, ref_ty)
            verdict = 'PASS' if magnitude <= args.threshold_deg else 'FAIL'
            if verdict == 'FAIL':
                n_fail += 1
            print('  [{}] {} home-ref residual {} |r|={:.4f} deg  raw {}  '
                  'sd ({:.6f}, {:.6f})'
                  .format(verdict, label, _fmt_deg(ref_tx, ref_ty),
                          magnitude, _fmt_deg(stats.res_tx, stats.res_ty),
                          stats.sd_tx, stats.sd_ty))
            check_summaries.append({
                'x_mm': cx, 'y_mm': cy,
                'res_tx_rad': stats.res_tx, 'res_ty_rad': stats.res_ty,
                'ref_tx_rad': ref_tx, 'ref_ty_rad': ref_ty,
                'verify_home_rad': list(verify_ref),
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
        abort_reason = str(exc)
        print('\nABORT: {}'.format(exc), file=sys.stderr)
        try:
            print_forensics(runner.forensics())
        except Exception as fx_exc:                       # noqa: BLE001
            print('  (forensics dump itself failed: {})'.format(fx_exc),
                  file=sys.stderr)
        rc = 2
    except KeyboardInterrupt:
        abort_reason = 'interrupted by operator (KeyboardInterrupt)'
        print('\nABORT: interrupted by operator.', file=sys.stderr)
        try:
            print_forensics(runner.forensics())
        except Exception as fx_exc:                       # noqa: BLE001
            print('  (forensics dump itself failed: {})'.format(fx_exc),
                  file=sys.stderr)
        rc = 130
    except SystemExit as exc:
        # thc.safety_gate declines via sys.exit(1). Without this handler the
        # meta records exit_code 0 / abort_reason None while the process
        # exits 1 — recreating the "forensically indistinguishable attempt"
        # class this change exists to close. Re-raised: the exit code is the
        # gate's, not ours.
        abort_reason = 'operator declined a safety gate (SystemExit {})'.format(
            exc.code)
        rc = exc.code if isinstance(exc.code, int) else 2
        raise
    except BaseException as exc:                          # noqa: BLE001
        # Anything else (OSError on the map write, an rclpy failure
        # mid-sweep, EOFError at a prompt): record WHY before propagating,
        # so the meta is never silent about a crash.
        abort_reason = 'unexpected {}: {}'.format(type(exc).__name__, exc)
        rc = rc or 3
        raise
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
            # A final cache snapshot is written on EVERY run, clean or not —
            # it costs nothing (pure cache read) and the clean-run baseline is
            # itself evidence when the next run aborts.
            try:
                _final_forensics = runner.forensics()
            except Exception:                             # noqa: BLE001
                _final_forensics = None
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
                'home_reference': home_meta,
                # ALWAYS present (None on a clean run): the 2026-08-09
                # header-only attempts were indistinguishable exit_code-2s
                # because the abort reason lived only on stderr.
                'abort_reason': abort_reason,
                'forensics': _final_forensics,
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
                problems = stroke_margin_problems(
                    [(nd.x_mm, nd.y_mm) for nd in order]
                    + [(cx, cy) for cx, cy in checks] + [(0.0, 0.0)],
                    float(args.z))
                if problems:
                    print('\n  !! IK stroke preflight WOULD REFUSE {} '
                          'pose(s):\n    {}'.format(
                              len(problems), '\n    '.join(problems)))
                    print('\ndry-run: no ROS calls made, no ROS objects '
                          'constructed.')
                    return 2
                print('  IK stroke preflight OK: every pose keeps >= '
                      '{:.0f} mm of leg-extension margin'
                      .format(STROKE_PREFLIGHT_MARGIN_MM))
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
