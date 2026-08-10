#!/usr/bin/env python3
"""Toss AIM-calibration acquisition — contract **C-TOSS-CAL-1**, build phase 2f
of ``plans/active/toss-selftuning.md`` § 3.8 (OPERATOR-run, at the robot).

Drives the four capture rungs of § 3.8 by sending ``TossContinuous`` goals and
**observing**.  It never arms, never changes control mode, never dispatches a
hand move, never publishes on a production topic.  Everything actuating is a
goal to ``/jugglebot/toss_continuous``; the safing ladder on every exit belongs
to ``reload_coordinator_node`` and is reached by **cancelling the goal**.

    # rehearsal: node order, toss count, ETA, BALL BUDGET — ZERO ROS calls
    python3 tests/hardware/toss_cal_grid.py --rung sc2 --dry-run

    # rung SC-0 — the sign/gain probe that BLOCKS every later rung
    python3 tests/hardware/toss_cal_grid.py --rung sc0 --base-condition "..."

    # score a captured rung from the MINED corpus (desk-side, no robot)
    python3 tests/hardware/toss_cal_grid.py --rung sc0 --score temp/probes/toss_records_*.jsonl

    # rung SC-2 — the 3x3 grid (refuses until SC-0 PASSed and SC-1 was scored)
    python3 tests/hardware/toss_cal_grid.py --rung sc2 --base-condition "..."

    # rung SC-3 — verification against the ALREADY-LOADED map
    python3 tests/hardware/toss_cal_grid.py --verify-only

Runbook: ``tests/hardware/session_phase8_toss_hardware.md`` and § 6 of the plan.
Fit + write: ``tests/hardware/toss_cal_fit.py`` (phase 2c).  Analysis:
``tools/toss_cal_analyse.py``.

WHY CAPTURE AND SCORING ARE TWO INVOCATIONS
-------------------------------------------
The **only admissible aim observable is** ``land_err_mm`` (D5), and its schema
origin is **M — mined**: it comes from the offline mocap descending-branch
estimator via ``tools/probes/toss_record_miner.py``.  Nothing in the live graph
produces it — ``TossResult.catch_error_mm`` is D5's named-forbidden observable
and ``BallState.landing_position`` is the same tracker Kalman filter one message
earlier (the finding 2e landed against the session trim, § 10).  So this tool
**cannot score its own rungs live**, and pretending otherwise would mean gating
a blocking rung on a forbidden observable.

Instead each capture writes an **UNSCORED** row into the rung ledger
(``temp/logs/toss_cal_rungs.json``) carrying the exact mine command, and a later
``--score`` invocation fills the verdict in.  ``--rung sc2`` refuses to spend 72
tosses until SC-0 carries a PASS and SC-1 has been scored — that is what makes
"SC-0 BLOCKS everything" structural rather than remembered.

THE GATES ARE THREE-VALUED, AND THEY REFUSE ON EVIDENCE
------------------------------------------------------
§ 3.8's gates are written as point comparisons — SC-0's "diagonal within ±25 %
of ``4h``", SC-3's "``|mean L|`` ≤ 10 mm at ≥5 of 6".  Measured against the
precision those rungs actually achieve, **the literal gates refuse a healthy
machine most of the time** (probe ``/tmp/probe_toss_sc_gates.py``, 2026-08-11,
20 000 trials per cell, at the design's own working figure σ = 20 mm/axis):

===========  ====================================  ================
rung          literal gate, PERFECT plant           evidence gate
===========  ====================================  ================
SC-0 n=5      **33.0 %** pass (σ=30: 10.1 %)        97.3 %
SC-3 n=5      **8.0 %** pass  (σ=30: 0.4 %)         99.9 %
SC-1 n=8      **0.2 %** of ladders decide units     unchanged
===========  ====================================  ================

A blocking gate that refuses a healthy plant two times in three does not protect
the map — it prevents the map from ever being captured, and the first thing an
operator does with a gate that always fails is disable it.  So every gate here
is **three-valued** and refuses only on evidence:

* **FAIL** — the 2·se interval EXCLUDES the design's bound.  Blocking.
* **PASS** — the 2·se interval is entirely INSIDE the bound.  Proven good.
* **INCONCLUSIVE** — the interval straddles it.  Reported with the ``n`` that
  would resolve it, and NOT treated as a pass by any later rung's precondition.

The design's point estimate is still printed on every line, and nothing the
gates exist to catch is weakened: at every σ tested, a **sign flip leaks 0 of
24 000 trials** under both forms, and a 2× gain error leaks 1.15 % at σ=20
(0.05 % at n=8) against the literal gate's 0 % — bought with a 3× reduction in
false refusals.  See § Phase 2f of
``logbook/2026-08-10-toss-selftuning-build.md``.

SC-1 IN PARTICULAR CANNOT DECIDE THE UNITS, AND SAYS SO UP FRONT
----------------------------------------------------------------
At the design's n = 8 and σ = 20 mm the exponent's 95 % CI has a median width of
**4.40** — three times the width of the whole ``[0.7, 1.3]`` rad band — and lands
inside a branch **0.2 %** of the time.  Even n = 32 at a 1.0° bias reaches only
14.3 %.  The branch table's escape hatch ("CI spanning two branches ⇒ store at
the working height only and WARN elsewhere") is therefore not the exception, it
is the expected outcome.  The rung still earns its 32 tosses: it produces the
**first honest σ_L on this machine**, which is the number every other gate's
resolvability — and 2e's whole trim-value question — turns on.  ``--dry-run``
prints the power calculation before the tosses are spent.

HOW AN AIM IS COMMANDED (SC-0)
------------------------------
``TossContinuous`` has no aim field: the aim is applied by
``reload_coordinator_node._build_toss_cycle`` from the persistent map, and the
map is therefore the ONLY aim authority.  SC-0's probe aims are commanded by
writing a **uniform probe map** through the production write target
(``toss_cal.toss_cal_candidates()[0]`` — no ``__file__`` walk anywhere), calling
``toss/reload_calibration`` and **reading the version back**.  The pre-rung file
is snapshotted first and **restored on every exit path**, including a second
Ctrl-C: a ±0.5° probe map left installed aims every later throw 27 mm off while
every log line reports a calibration as applied, which is the silent-wrong class
this whole plan is built to avoid.

NODE EXHAUSTION IS NOT A CAPTURE ABORT
--------------------------------------
A session that ends ``STOPPED_RELOAD_BUDGET`` has spent its ball supply at that
node (operator decision 4, § 9; D19).  The tool marks the node **thin/stale**,
logs the census, and **skips to the next node**.  § 3.7 item 7 already writes a
thin node correctly (previous value kept, ``stale: true``), so an exhausted node
costs one node's refresh rather than a sitting.

Runs under the **system python3.8 with ROS 2 sourced** — ``source
ros_ws/install/setup.bash`` first.  It does NOT need the project venv.  The pure
core (rung specs, refusals, gates, probe-map assembly, ledger, CSV rows) imports
no ROS and is unit-tested in ``tests/motion/test_toss_cal_grid.py``; ``rclpy``
and the interface packages are imported inside :func:`run` only.
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import shutil
import sys
import time
from datetime import datetime
from typing import Any, Dict, List, NamedTuple, Optional, Sequence, Tuple

_HERE = os.path.dirname(os.path.abspath(__file__))              # tests/hardware
_REPO = os.path.dirname(os.path.dirname(_HERE))                 # repo root
for _p in (_HERE, _REPO, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_REPO, 'config', 'generated')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np                                              # noqa: E402

import jugglebot.hardware_config as hw                          # noqa: E402
from jugglebot import toss_trim                                 # noqa: E402
from jugglebot.motion import toss_cal                           # noqa: E402

import toss_fit_lib as fit                                      # noqa: E402
# The grid geometry and the disarmed-wire verdict are IMPORTED from the tilt
# tool, not restated. § 3.7 says this map uses "the same axes as
# tilt_calibration.yaml", and the wire is literally the same wire with the same
# `/link_status` encodings — a second implementation of either is a second place
# for the node order or the DISARMED semantics to drift, and the tilt tool's
# version is the one with a hardware finding behind every branch.
import tilt_cal_grid as tcg                                     # noqa: E402
import _th_test_common as thc                                   # noqa: E402

TOOL_NAME = 'tests/hardware/toss_cal_grid.py'
ACTION_NAME = '/jugglebot/toss_continuous'

# ── defaults ─────────────────────────────────────────────────────────────
#
# Every default marked PROVISIONAL is a placeholder until the first capture
# measures it. The probe-first rule forbids asserting a threshold before a
# probe pins it, and the probe behind the gate constants is
# /tmp/probe_toss_sc_gates.py (2026-08-11) — quoted in the module docstring.

DEFAULT_BOX_MM = 150.0
DEFAULT_NODES = 3                # § 3.8: 3x3 FIRST; 5x5 only after one verifies
DEFAULT_Z_MM = 170.0
DEFAULT_N_PER_NODE = 8           # SC-1/SC-2 (§ 3.8)
DEFAULT_SC0_N = 5                # SC-0 (§ 3.8)
DEFAULT_SC3_N = 5                # SC-3 (§ 3.8)
DEFAULT_CHECK_POSES = 6          # SC-3: ">= 6 off-node check poses"
DEFAULT_THROW_HEIGHT_M = 0.78
DEFAULT_DWELL_S = float(hw.JB_OP_TOSS_SESSION_DWELL_DEFAULT_S)      # 6.0
DEFAULT_THROW_DELAY_S = 5.0      # toss_sequencer.DEFAULT_TOSS_THROW_DELAY_S
DEFAULT_MAX_RELOADS = int(hw.JB_OP_TOSS_SESSION_MAX_RELOADS)        # 3
DEFAULT_TIMEOUT_S = 5.0          # per service call
DEFAULT_HOME_REVISIT_EVERY = 4
DEFAULT_SIGMA_MM = 20.0          # the DESIGN's working figure, never measured
                                 # on this machine (2e's open row). Used for the
                                 # power REPORT only — every gate's se comes
                                 # from the data.
DEFAULT_CATCH_RATE = 0.75        # § 6's budget table assumption
DEFAULT_SC0_PROBE_DEG = 0.5      # § 3.8. Raising it sharpens the gain estimate
                                 # (se ∝ 1/δ) but displaces the landing by
                                 # 4h·δ = 54.6 mm at 1.0°, which is 68 % of the
                                 # 80 mm reach envelope: most probe tosses would
                                 # MISS and the rung would exhaust its reload
                                 # budget. 0.5° is 27 mm — catchable.

SC1_HEIGHTS_M = (0.45, 0.60, 0.78, 1.00)     # § 3.8

# The height-exponent branch table, VERBATIM from § 3.8. Ordered, and the first
# band the whole CI fits inside wins; a CI spanning two bands is the
# working-height-only branch (the hull-clamp doctrine applied to the height
# axis).
SC1_BRANCHES = (
    ('rad', (0.7, 1.3),
     'store the map in RADIANS — one map serves every throw_height_m'),
    ('dv', (0.3, 0.7),
     'release-impulse origin — store in Δv units'),
    ('mm', (-0.2, 0.2),
     'fixed-mm origin — store in mm and REFUSE off-capture-height application'),
)
SC1_UNITS_AMBIGUOUS = 'working_height_only'

# SC-0 accept bounds (§ 3.8), applied as EVIDENCE intervals — see the docstring.
SC0_DIAG_LO = 0.75
SC0_DIAG_HI = 1.25
SC0_OFFDIAG_FRAC = 0.30
# Interval half-width in standard errors. 2.0 is ~95 % at the pooled df this
# rung reaches (4 arms × (n−1) = 16 per axis at n = 5, t = 2.12) and is the same
# multiplier the SC-3 and SC-1 gates use, so one number governs every refusal.
GATE_K_SE = 2.0

# SC-3 gates (§ 3.8's table).
SC3_ACCURACY_MM = 10.0           # ~1.2x θ_acc's 8.2 mm floor
SC3_MIN_PASSING = 5              # of --check-poses
SC3_COMMON_MODE_MM = 6.0         # pooled |c|, must fit one session-trim authority

# Anchor-series gates — the tilt-cal ones VERBATIM (§ 3.8's table says so).
ANCHOR_PP_WARN_RAD = tcg.ANCHOR_PP_WARN_RAD
ANCHOR_PP_ABORT_RAD = tcg.ANCHOR_PP_ABORT_RAD
ANCHOR_STEP_ABORT_RAD = tcg.ANCHOR_STEP_ABORT_RAD

# R7: stricter than tilt_cal_grid on purpose — static inclinometer reads are
# uptime-insensitive, a TIMING bias is not (§ 3.8).
UPTIME_ABORT_MS = 30 * 60 * 1000

# Cadence arithmetic for the ETA (§ 6's budget table).
SETTLE_S = 2.80                  # toss_session.DEFAULT_SESSION_MISS_CLEANUP_S
GOAL_OVERHEAD_S = 4.0            # accept + go_home + verified arrival between goals
RELOAD_COST_S = 25.0             # § 6: "a drop costs ~25 s all-in"

# The interlude's own fence (D19). Named here because the tool's response to it
# is a NODE SKIP, not an abort.
NODE_EXHAUSTED_OUTCOME = 'STOPPED_RELOAD_BUDGET'

# TossContinuous feedback phases in which a cancel is DEFERRED to the cycle's own
# terminal, because the stroke is about to fire or the ball is airborne
# (reload_coordinator_node._toss_cancel_deferred). THROWING is listed because the
# tool cannot see the cutoff instant from outside — it reports "may be deferred".
CANCEL_DEFERRED_PHASES = ('THROWING', 'BALL_IN_FLIGHT', 'CATCHING', 'SETTLING')

RUNGS = ('sc0', 'sc1', 'sc2', 'sc3')

LEDGER_NAME = 'toss_cal_rungs.json'
LEDGER_SCHEMA = 'toss_cal_rungs/1'

VERDICT_PASS = 'PASS'
VERDICT_FAIL = 'FAIL'
VERDICT_INCONCLUSIVE = 'INCONCLUSIVE'
VERDICT_UNSCORED = 'UNSCORED'

CSV_COLUMNS = (
    'iso', 't_s', 'rung', 'goal_index', 'label', 'ix', 'iy',
    'x_mm', 'y_mm', 'z_mm', 'throw_height_m',
    'probe_rx_deg', 'probe_ry_deg',
    'toss_uid', 'cycle_index', 'outcome',
    'total_aim_rx_rad', 'total_aim_ry_rad',
    'map_aim_rx_rad', 'map_aim_ry_rad',
    'flight_time_s', 'catch_error_mm_fsm_diagnostic',
    'dwell_tilt_n', 'dwell_tilt_rx_rad', 'dwell_tilt_ry_rad',
    'uptime_ms', 'note',
)


class TossCalGridError(RuntimeError):
    """A refusal or an abort. Non-zero exit, nothing written."""


class TossCalGridFaultError(TossCalGridError):
    """A RUN-level fault (disarmed wire, a latched drive error, a lost link).

    A subclass so the per-node handler can re-raise it rather than demote it to
    "this one node failed" — the tilt-cal BLOCKING finding transposed: under
    ``--on-fail continue`` a demoted wire fault would let every remaining node
    run against a machine that cannot move.
    """


# ── pure core: rung specs ────────────────────────────────────────────────


class GoalSpec(NamedTuple):
    """One ``TossContinuous`` goal: a node, a height, an optional probe aim."""
    rung: str
    index: int
    label: str
    ix: int
    iy: int
    x_mm: float
    y_mm: float
    z_mm: float
    throw_height_m: float
    n_throws: int
    probe_aim_rad: Optional[Tuple[float, float]]   # None ⇒ leave the map alone
    is_anchor: bool = False

    @property
    def probe_deg(self) -> Tuple[float, float]:
        if self.probe_aim_rad is None:
            return (float('nan'), float('nan'))
        return (math.degrees(self.probe_aim_rad[0]),
                math.degrees(self.probe_aim_rad[1]))


def sc0_arms(probe_deg: float) -> List[Tuple[str, Tuple[float, float]]]:
    """§ 3.8's five SC-0 aim settings: ``(0,0)``, ``(±δ,0)``, ``(0,±δ)``.

    The zero arm is not decoration — it measures the plant bias ψ that the ±δ
    arms are differenced around, and it is the arm whose landing error the
    operator compares against a no-map baseline.
    """
    d = math.radians(float(probe_deg))
    return [
        ('zero', (0.0, 0.0)),
        ('rx+', (+d, 0.0)),
        ('rx-', (-d, 0.0)),
        ('ry+', (0.0, +d)),
        ('ry-', (0.0, -d)),
    ]


def off_node_poses(x_mm: Sequence[float], y_mm: Sequence[float],
                   count: int) -> List[Tuple[float, float]]:
    """``count`` off-node SC-3 check poses — **cell centres first, then edge
    midpoints**.

    ``tilt_cal_grid.check_poses`` is used verbatim for the cell centres (same
    deterministic quadrant-round-robin, same outermost-first ordering, same
    reason: a node pose measures the stored value, which is exact there by
    construction and proves nothing). It is EXTENDED here because § 3.8 asks for
    **≥6** off-node poses and a 3×3 grid — the grid § 3.8 also mandates for the
    first capture — has only **2×2 = 4 interior cells**. The two requirements
    are unsatisfiable together from cell centres alone.

    Edge midpoints are the right extension rather than a denser grid of
    centres: they sit off-node, inside the hull, and exercise the bilinear blend
    along exactly ONE axis, which is the case a transposed ``[iy][ix]`` grid
    passes at a cell centre and fails on an edge. Centres come first because
    they exercise both axes at once.
    """
    if count <= 0:
        return []
    chosen = list(tcg.check_poses(x_mm, y_mm, count))
    if len(chosen) >= count:
        return chosen[:count]
    edges: List[Tuple[float, float]] = []
    for iy, y in enumerate(y_mm):
        for ix in range(len(x_mm) - 1):
            edges.append(((float(x_mm[ix]) + float(x_mm[ix + 1])) / 2.0,
                          float(y)))
    for ix, x in enumerate(x_mm):
        for iy in range(len(y_mm) - 1):
            edges.append((float(x),
                          (float(y_mm[iy]) + float(y_mm[iy + 1])) / 2.0))
    # Outermost first, stable tie-break — never dependent on set iteration.
    edges.sort(key=lambda p: (-(p[0] ** 2 + p[1] ** 2), p[0], p[1]))
    for pose in edges:
        if len(chosen) >= count:
            break
        if pose not in chosen:
            chosen.append(pose)
    if len(chosen) < count:
        raise TossCalGridError(
            'this {}x{} grid offers only {} distinct off-node check pose(s), '
            'fewer than the {} requested. A check pose must be off-node (a node '
            'measures the stored value and proves nothing about the '
            'interpolation), so lower --check-poses or use a denser grid.'
            .format(len(x_mm), len(y_mm), len(chosen), count))
    return chosen


def rung_goals(rung: str, x_mm: Sequence[float], y_mm: Sequence[float],
               z_mm: float, args) -> List[GoalSpec]:
    """The ordered goal list for one rung. Pure — this is what ``--dry-run``
    prints and what the ETA and ball budget are computed from."""
    rung = str(rung)
    if rung == 'sc0':
        ix, iy = tcg.home_index(x_mm, y_mm)
        out = []
        for i, (name, aim) in enumerate(sc0_arms(args.sc0_probe_deg)):
            out.append(GoalSpec(
                rung, i, 'SC-0 {} aim ({:+.2f}, {:+.2f})deg'.format(
                    name, math.degrees(aim[0]), math.degrees(aim[1])),
                ix, iy, 0.0, 0.0, z_mm, float(args.throw_height_m),
                int(args.sc0_n), aim))
        return out
    if rung == 'sc1':
        ix, iy = tcg.home_index(x_mm, y_mm)
        # No probe map. SC-1 regresses the RAW landing error on height, so it
        # needs the applied aim to be ZERO — with a correcting map installed
        # |L| is the residual, not the plant bias, and the exponent would be
        # fitted to noise. That is enforced as a refusal
        # (:func:`sc1_baseline_verdict`) rather than by writing a zero map: a
        # rung that writes nothing has nothing to restore, and the operator's
        # remedy (--force-uninstall) is the one the escape-hatch table already
        # names.
        return [GoalSpec(rung, i, 'SC-1 h={:.2f} m'.format(h), ix, iy,
                         0.0, 0.0, z_mm, float(h), int(args.n_per_node), None)
                for i, h in enumerate(SC1_HEIGHTS_M)]
    if rung == 'sc2':
        order = tcg.visit_order(x_mm, y_mm)
        revisit = set(tcg.home_revisit_after_visits(
            len(order), int(args.home_revisit_every)))
        out: List[GoalSpec] = []
        hx, hy = tcg.home_index(x_mm, y_mm)
        for visit, nd in enumerate(order):
            out.append(GoalSpec(
                rung, len(out), 'node ({:+.1f}, {:+.1f})'.format(nd.x_mm, nd.y_mm),
                nd.ix, nd.iy, float(nd.x_mm), float(nd.y_mm), z_mm,
                float(args.throw_height_m), int(args.n_per_node), None))
            if visit in revisit:
                out.append(GoalSpec(
                    rung, len(out), 'home anchor (after visit {})'.format(visit + 1),
                    hx, hy, 0.0, 0.0, z_mm, float(args.throw_height_m),
                    int(args.n_per_node), None, is_anchor=True))
        out.append(GoalSpec(
            rung, len(out), 'home anchor (end of sweep)', hx, hy, 0.0, 0.0,
            z_mm, float(args.throw_height_m), int(args.n_per_node), None,
            is_anchor=True))
        return out
    if rung == 'sc3':
        checks = off_node_poses(x_mm, y_mm, int(args.check_poses))
        return [GoalSpec(rung, i, 'check {} ({:+.1f}, {:+.1f})'.format(
                             i + 1, cx, cy), -1, -1, float(cx), float(cy),
                         z_mm, float(args.throw_height_m), int(args.sc3_n), None)
                for i, (cx, cy) in enumerate(checks)]
    raise TossCalGridError('unknown rung {!r}; want one of {}'
                           .format(rung, ', '.join(RUNGS)))


def toss_count(goals: Sequence[GoalSpec]) -> int:
    return sum(int(g.n_throws) for g in goals)


def estimate_duration_s(goals: Sequence[GoalSpec], *, dwell_s: float,
                        throw_delay_s: float, catch_rate: float) -> float:
    """Wall-clock ETA. § 6's cadence arithmetic, spelled out.

    A goal costs ``throw_delay`` to the first release, ``dwell`` per subsequent
    cycle, one settle, and one inter-goal overhead (accept + the verified
    ``go_home`` between nodes). Drops are priced at § 6's measured ~25 s all-in.
    """
    total = 0.0
    for g in goals:
        n = max(1, int(g.n_throws))
        total += (float(throw_delay_s) + (n - 1) * float(dwell_s)
                  + SETTLE_S + GOAL_OVERHEAD_S)
    drops = toss_count(goals) * max(0.0, 1.0 - float(catch_rate))
    return total + drops * RELOAD_COST_S


def ball_budget(goals: Sequence[GoalSpec], *, max_reloads: int,
                catch_rate: float) -> Dict[str, Any]:
    """The BALL BUDGET line § 3.8 asks ``--dry-run`` to print.

    There is **no machine-side magazine fence** (D19): ``ball_butler_node`` has
    no ball-count field, so supply has no machine observability and none is
    invented. What can be stated honestly is the worst case the ONE machine-side
    fence permits, and the expected drops at an assumed catch rate.
    """
    n_goals = len(goals)
    tosses = toss_count(goals)
    return {
        'goals': n_goals,
        'tosses': tosses,
        'worst_case_reloads': int(max_reloads) * n_goals,
        'expected_drops': round(tosses * max(0.0, 1.0 - float(catch_rate)), 1),
        'max_reloads_per_goal': int(max_reloads),
        'catch_rate_assumed': float(catch_rate),
    }


# ── pure core: preflight refusals R1–R9 ──────────────────────────────────


class Refusal(NamedTuple):
    rid: str
    ok: bool
    message: str


def hand_sensor_verdict(valid_seen: bool, held_to_empty_seen: bool
                        ) -> Tuple[bool, str]:
    """R5 — the sensor is the loop's entire ground truth.

    A **static** read cannot see a stuck bit, so the refusal needs a live
    ``held → empty`` transition and not merely ``ball_held_valid``. That is the
    P2 preflight row of § 6 made machine-checked.
    """
    if not valid_seen:
        return (False,
                'R5 REFUSED: no /hand_telemetry sample with ball_held_valid=true '
                'was seen. UNKNOWN is not empty and is not held — with no ground '
                'truth every label this capture produces is unfounded.')
    if not held_to_empty_seen:
        return (False,
                'R5 REFUSED: ball_held_valid is true but no live held->empty '
                'transition was observed. A static read cannot see a stuck bit, '
                'and a stuck-held bit labels every toss CAUGHT. Lift the ball '
                'out of the cup and replace it while this tool watches, then '
                're-run.')
    return (True, 'R5 ok: sensor valid AND a live held->empty edge observed')


def sc1_baseline_verdict(applied: bool,
                         aim_at_home_rad: Optional[Tuple[float, float]],
                         tol_rad: float = 1e-9) -> Tuple[bool, str]:
    """SC-1 needs a ZERO applied aim at the centre node.

    The height ladder regresses ``log|L|`` on ``log h``, and ``|L|`` under an
    applied correction is the RESIDUAL, not the plant bias the exponent is a
    property of. A map that is loaded but DORMANT aims vertically and is
    therefore fine; a map that is APPLIED and non-zero is not.
    """
    if not applied:
        return (True, 'SC-1 baseline ok: no aim is applied (the plant bias is '
                      'measured directly)')
    if aim_at_home_rad is None:
        return (False,
                'SC-1 REFUSED: a toss map is APPLIED but this tool could not '
                'read its aim at the centre node, so it cannot tell whether the '
                'height ladder would measure the plant bias or a residual. '
                'Re-run with --force-uninstall.')
    if max(abs(float(v)) for v in aim_at_home_rad) <= tol_rad:
        return (True, 'SC-1 baseline ok: the applied map commands zero aim at '
                      'the centre node')
    return (False,
            'SC-1 REFUSED: the applied map commands ({:+.4f}, {:+.4f}) deg at '
            'the centre node. The height ladder would regress the RESIDUAL, not '
            'the plant bias, and the exponent it reports would be a property of '
            'the map rather than of the machine. Re-run with --force-uninstall.'
            .format(math.degrees(aim_at_home_rad[0]),
                    math.degrees(aim_at_home_rad[1])))


def preflight_refusals(obs: Dict[str, Any]) -> List[Refusal]:
    """The nine § 3.8 refusals, **all hoisted before anything moves**.

    Pure: ``obs`` is a plain dict of observations so every rung of the table is
    unit-testable without a robot. The tilt-cal audit found a write-target check
    running *after* a four-minute sweep; here R8 is evaluated in the same pass as
    R1.

    Returns every row, in order, evaluated — the caller prints the whole table
    and then raises on the first failure, so an operator sees which OTHER
    preconditions also need fixing before walking back to the robot.
    """
    out: List[Refusal] = []

    ok, message = tcg.wire_armed_verdict(obs.get('link_kv'))
    out.append(Refusal('R1', ok, 'R1 ' + ('ok: ' if ok else 'REFUSED: ') + message))

    gravity = bool(obs.get('gravity_correction_loaded'))
    out.append(Refusal(
        'R2', gravity,
        'R2 ok: gravity_correction_loaded=true' if gravity else
        'R2 REFUSED: gravity_correction_loaded=false — every toss would be '
        'REJECTED_NOT_LEVELLED. Run `level` from IDLE (over '
        '/orchestrator_command, not the /activate service); the correction is '
        'per-PROCESS, so a relaunch drops it silently.'))

    tilt = bool(obs.get('tilt_map_loaded'))
    out.append(Refusal(
        'R3', tilt,
        'R3 ok: tilt_map_loaded=true version={!r}'.format(
            obs.get('tilt_map_version')) if tilt else
        'R3 REFUSED: tilt_map_loaded=false. This aim map is DOWNSTREAM of the '
        'tilt map (D3) — capturing without one bakes pose-dependent tilt error '
        'into the aim bias, and the resulting map is invalid the moment a tilt '
        'map is loaded.'))

    trim_off = not bool(obs.get('toss_trim_enabled'))
    out.append(Refusal(
        'R4', trim_off,
        'R4 ok: toss_trim_enabled=false' if trim_off else
        'R4 REFUSED: toss_trim_enabled=true — the session trim is a RAM-only '
        'common-mode correction fitted online with no review, and letting it '
        'move during a capture contaminates the persistent map with it. '
        '  ros2 param set /reload_coordinator_node toss_trim_enabled false'))

    ok5, message5 = hand_sensor_verdict(bool(obs.get('sensor_valid_seen')),
                                        bool(obs.get('sensor_edge_seen')))
    out.append(Refusal('R5', ok5, message5))

    tier = str(obs.get('toss_tier') or '')
    tier_ok = tier == '8a'
    out.append(Refusal(
        'R6', tier_ok,
        'R6 ok: toss_tier=8a' if tier_ok else
        'R6 REFUSED: toss_tier={!r}, but the map is DEFINED at 8a (F2: an '
        'aim-corrected 8a toss is structurally an 8b toss with zero '
        'displacement). Set jugglebot_operational.toss_tier back to 8a and '
        'relaunch.'.format(tier)))

    uptime = obs.get('uptime_ms')
    if uptime is None:
        out.append(Refusal(
            'R7', False,
            'R7 REFUSED: no uptime_ms on /link_status — the can-bridge '
            'heartbeat has not arrived, so the ONE partition key that separates '
            'a fresh plant from a degraded one is unknown. Absence is not '
            'freshness.'))
    else:
        fresh = int(uptime) <= UPTIME_ABORT_MS
        out.append(Refusal(
            'R7', fresh,
            'R7 ok: uptime_ms={} ({:.2f} h)'.format(uptime, uptime / 3600000.0)
            if fresh else
            'R7 REFUSED: can-bridge uptime_ms={} ({:.2f} h) is past the {:.0f} '
            'min ceiling. The dispatch shift is +118-133 ms at ~16 h, and this '
            'map bakes in a TIMING bias — a capture on a warm bridge is a '
            'capture of the drift. Power-cycle the can-bridge Teensy and '
            're-arm.'.format(uptime, uptime / 3600000.0,
                             UPTIME_ABORT_MS / 60000.0)))

    target = obs.get('write_target')
    target_ok = bool(obs.get('write_target_ok'))
    out.append(Refusal(
        'R8', target_ok,
        'R8 ok: write target {}'.format(target or '<none needed by this rung>')
        if target_ok else
        'R8 REFUSED: {}'.format(obs.get('write_target_reason')
                                or 'the map write target is not usable')))

    confirmed = bool(obs.get('operator_confirmed'))
    via_yes = bool(obs.get('operator_yes'))
    out.append(Refusal(
        'R9', confirmed,
        ('R9 ok: operator confirmations SKIPPED by --yes on a LIVE capture — '
         'floor state, ball supply and E-STOP reach have NO machine check'
         if via_yes else 'R9 ok: operator confirmations taken') if confirmed
        else
        'R9 REFUSED: the operator confirmations (floor clear, ball supply '
        'staged, E-STOP in reach) were not given.'))
    return out


def write_target_verdict(environ: Optional[Dict[str, str]] = None
                         ) -> Tuple[bool, str, Optional[str]]:
    """R8 — ``(ok, path_or_none, reason)``.

    Resolved through ``toss_cal.toss_cal_candidates()[0]``, the production
    resolver's own first candidate, so the tool cannot write one file while the
    node reads another and both report success. **No ``__file__`` walk anywhere**
    — that was a tilt-cal Phase-2 finding.
    """
    try:
        target = fit.write_target_path(environ)
    except fit.TossFitError as exc:
        return (False, None, str(exc))
    env = (environ if environ is not None else os.environ).get(
        toss_cal.TOSS_CAL_ENV)
    if not env and not tcg.looks_like_source_tree(target):
        return (False, target,
                'the resolved write target {} is not in the repo source tree '
                '(no ros_ws/ beside its config/ directory). Writing into an '
                'ament share/install tree would be silently undone by the next '
                'colcon build. Set ${} if this is deliberate.'
                .format(target, toss_cal.TOSS_CAL_ENV))
    parent = os.path.dirname(target) or '.'
    if not os.path.isdir(parent):
        return (False, target,
                'the write target directory {} does not exist'.format(parent))
    if not os.access(parent, os.W_OK):
        return (False, target,
                'the write target directory {} is not writable'.format(parent))
    if os.path.exists(target) and not os.access(target, os.W_OK):
        return (False, target,
                'the write target {} exists and is not writable'.format(target))
    return (True, target, None)


def uninstall_plan(environ: Optional[Dict[str, str]] = None
                   ) -> Tuple[List[str], Optional[str]]:
    """``(paths_to_move_aside, env_override)`` for ``--force-uninstall``.

    **Every** existing candidate, source tree AND ament share. ``setup.py``
    installs ``config/toss_calibration.yaml`` into ``share/jugglebot/config/``
    whenever it exists at build time, and the resolver order is env → source →
    share, so moving only the source copy falls straight through to the share
    copy: the reload succeeds, the OLD map stays loaded, and the refusal names
    neither cause nor remedy. The tilt-cal review found exactly that, in a
    single-file version that could not actually uninstall.

    An env-override candidate is never touched — it names a file the operator
    chose, possibly outside the repo — and is returned for the caller to refuse
    on, with instructions.
    """
    env = (environ if environ is not None else os.environ).get(
        toss_cal.TOSS_CAL_ENV)
    if env:
        return ([], env)
    return ([path for path in toss_cal.toss_cal_candidates(environ)
             if os.path.exists(path)], None)


# ── pure core: the three-valued gates ────────────────────────────────────


class GateVerdict(NamedTuple):
    verdict: str                 # PASS | INCONCLUSIVE | FAIL
    headline: str
    detail: Dict[str, Any]

    @property
    def blocking(self) -> bool:
        return self.verdict == VERDICT_FAIL


def interval_verdict(point: float, se: float, lo: float, hi: float) -> str:
    """One scalar against a band, three-valued and evidence-first.

    PASS iff ``[point ± k·se]`` lies wholly inside ``[lo, hi]``; FAIL iff it lies
    wholly outside; INCONCLUSIVE when it straddles an edge. See the module
    docstring for why the design's point comparison is retained as a printed
    number rather than as the gate.
    """
    if not (math.isfinite(point) and math.isfinite(se)) or se < 0.0:
        return VERDICT_INCONCLUSIVE
    half = GATE_K_SE * float(se)
    a, b = point - half, point + half
    if a >= lo and b <= hi:
        return VERDICT_PASS
    if b < lo or a > hi:
        return VERDICT_FAIL
    return VERDICT_INCONCLUSIVE


def worst_verdict(verdicts: Sequence[str]) -> str:
    """FAIL dominates INCONCLUSIVE dominates PASS."""
    if any(v == VERDICT_FAIL for v in verdicts):
        return VERDICT_FAIL
    if any(v == VERDICT_INCONCLUSIVE for v in verdicts):
        return VERDICT_INCONCLUSIVE
    return VERDICT_PASS


def _mean_sd(values: Sequence[float]) -> Tuple[float, float, int]:
    arr = np.asarray([float(v) for v in values if math.isfinite(float(v))],
                     dtype=float)
    if arr.size == 0:
        return (float('nan'), float('nan'), 0)
    if arr.size == 1:
        return (float(arr[0]), float('nan'), 1)
    return (float(arr.mean()), float(arr.std(ddof=1)), int(arr.size))


def sc0_measure(arms: Dict[str, Sequence[Sequence[float]]], probe_rad: float,
                jacobian: np.ndarray) -> Tuple[np.ndarray, float, Dict[str, Any]]:
    """``(D, se, detail)`` from the five SC-0 arms.

    ``arms`` maps the arm name (``rx+`` / ``rx-`` / ``ry+`` / ``ry-`` / ``zero``)
    to that arm's list of ``land_err_mm`` pairs. ``D = J_pred⁻¹ · J_meas`` — the
    measured Jacobian expressed in the **predicted** basis.

    Why the ratio and not the raw matrix: § 3.8's accept test is written as
    "diagonal within ±25 % of the predicted ``4h``, off-diagonal < 30 % of the
    diagonal", which presumes ``S`` is a scaled identity. The 2c build measured
    it and it is **a 90° rotation** (``J = [[0, 3126.5], [−3126.5, 0]]``), so in
    the raw ``(Lx,Ly)×(rx,ry)`` basis the design's "diagonal" is the zero entry
    and its "off-diagonal" is the gain — the literal test is inverted on this
    plant. Expressed in the predicted basis the test means exactly what it says
    and reduces to the design's words whenever ``J_pred`` IS a scaled identity.
    """
    need = ('rx+', 'rx-', 'ry+', 'ry-')
    means: Dict[str, np.ndarray] = {}
    counts: Dict[str, int] = {}
    resid: List[float] = []
    for name in need:
        rows = [r for r in (arms.get(name) or []) if r is not None]
        if len(rows) < 2:
            raise TossCalGridError(
                'SC-0 arm {!r} has {} usable toss(es); the sign/gain probe needs '
                'at least 2 per arm to have a standard error at all'
                .format(name, len(rows)))
        pts = np.asarray([[float(r[0]), float(r[1])] for r in rows], dtype=float)
        means[name] = pts.mean(axis=0)
        counts[name] = int(pts.shape[0])
        resid.extend((pts - means[name]).ravel().tolist())
    # Pooled within-arm sd across BOTH axes and all four arms: the arms are
    # replicates of one measurement process, so pooling is what buys the df that
    # makes a 2·se interval meaningful at n = 5.
    dof = sum(counts[n] - 1 for n in need) * 2
    sigma = math.sqrt(sum(r * r for r in resid) / dof) if dof > 0 else float('nan')
    n_arm = min(counts[n] for n in need)
    col_rx = (means['rx+'] - means['rx-']) / (2.0 * probe_rad)
    col_ry = (means['ry+'] - means['ry-']) / (2.0 * probe_rad)
    J_meas = np.column_stack([col_rx, col_ry])
    D = np.linalg.solve(np.asarray(jacobian, dtype=float), J_meas)
    gain = float(np.linalg.norm(np.asarray(jacobian, dtype=float), 2))
    se = (math.sqrt(2.0) * sigma / math.sqrt(n_arm) / (2.0 * probe_rad) / gain
          if math.isfinite(sigma) and gain > 0.0 else float('nan'))
    detail = {
        'sigma_mm': sigma, 'n_per_arm': n_arm, 'dof': dof,
        'se_ratio': se, 'gain_mm_per_rad': gain,
        'probe_rad': float(probe_rad),
        'J_measured_mm_per_rad': J_meas.tolist(),
        'J_predicted_mm_per_rad': np.asarray(jacobian, dtype=float).tolist(),
        'D': D.tolist(),
        'arm_mean_mm': {k: v.tolist() for k, v in means.items()},
        'arm_n': counts,
    }
    zero = arms.get('zero') or []
    if zero:
        zx, _, _ = _mean_sd([r[0] for r in zero])
        zy, _, _ = _mean_sd([r[1] for r in zero])
        detail['zero_arm_mean_mm'] = [zx, zy]
        detail['zero_arm_n'] = len(zero)
    return D, se, detail


def sc0_verdict(D: np.ndarray, se: float, detail: Optional[Dict[str, Any]] = None
                ) -> GateVerdict:
    """§ 3.8's SC-0 accept test, three-valued (module docstring).

    Order matters: the SIGN is checked first and on its own, because a sign flip
    inverts every node and aims the machine roughly twice as badly as no map at
    all, and it is the ONE part of this rung the sample size can actually
    resolve (0 leaks in 24 000 probe trials at every σ tested).
    """
    D = np.asarray(D, dtype=float)
    diag = [float(D[0][0]), float(D[1][1])]
    off = [float(D[0][1]), float(D[1][0])]
    detail = dict(detail or {})
    detail.update({'diag': diag, 'off_diag': off, 'se_ratio': float(se)})

    # (1) SIGN — blocking, and excluded from zero at k·se.
    half = GATE_K_SE * float(se) if math.isfinite(se) else float('inf')
    signs = []
    for value in diag:
        if not math.isfinite(value):
            signs.append(VERDICT_INCONCLUSIVE)
        elif value - half > 0.0:
            signs.append(VERDICT_PASS)
        elif value + half < 0.0:
            signs.append(VERDICT_FAIL)
        else:
            signs.append(VERDICT_INCONCLUSIVE)
    sign_verdict = worst_verdict(signs)
    detail['sign_verdict'] = sign_verdict
    if sign_verdict == VERDICT_FAIL:
        return GateVerdict(
            VERDICT_FAIL,
            'SC-0 FAIL — the measured aim->landing SIGN is INVERTED against the '
            'production model (D = {:+.3f}, {:+.3f} ± {:.3f}). Do NOT capture a '
            'grid: a sign flip aims the machine roughly twice as badly as no map '
            'at all. Find the frame/plane/sign error first (plan § 7 R1 lists '
            'the four silent routes).'.format(diag[0], diag[1], se),
            detail)
    if sign_verdict == VERDICT_INCONCLUSIVE:
        return GateVerdict(
            VERDICT_INCONCLUSIVE,
            'SC-0 INCONCLUSIVE — the aim->landing gain is not distinguishable '
            'from ZERO at this precision (D = {:+.3f}, {:+.3f} ± {:.3f}). Either '
            'the platform did not actually aim, or the scatter swamps a {:.2f}° '
            'probe. Re-run with more tosses or a larger --sc0-probe-deg before '
            'spending a grid.'.format(diag[0], diag[1], se,
                                      math.degrees(detail.get('probe_rad', 0.0))),
            detail)

    # (2) GAIN and (3) OFF-DIAGONAL — reported always, blocking on evidence.
    gain_v = worst_verdict([interval_verdict(v, se, SC0_DIAG_LO, SC0_DIAG_HI)
                            for v in diag])
    bound = SC0_OFFDIAG_FRAC * min(abs(v) for v in diag)
    off_v = worst_verdict([interval_verdict(v, se, -bound, bound) for v in off])
    detail['gain_verdict'] = gain_v
    detail['offdiag_verdict'] = off_v
    detail['offdiag_bound'] = bound
    overall = worst_verdict([gain_v, off_v])
    headline = (
        'SC-0 {} — sign OK; gain D = ({:+.3f}, {:+.3f}) ± {:.3f} against the '
        '[{:.2f}, {:.2f}] band [{}]; off-diagonal ({:+.3f}, {:+.3f}) against '
        '±{:.3f} [{}]'
        .format(overall, diag[0], diag[1], se, SC0_DIAG_LO, SC0_DIAG_HI, gain_v,
                off[0], off[1], bound, off_v))
    if overall == VERDICT_INCONCLUSIVE:
        headline += ('\n  INCONCLUSIVE is not a pass: the plant obeys the '
                     'production model as far as this rung can see, but the '
                     '±25 % gain band is finer than {:.0f} tosses can resolve. '
                     'SC-2 may proceed on the SIGN, which is what SC-0 exists '
                     'to protect.'.format(4.0 * detail.get('n_per_arm', 0)))
    return GateVerdict(overall, headline, detail)


def sc0_resolvable_n(sigma_mm: float, probe_rad: float, gain_mm_per_rad: float,
                     half_band: float = 0.25) -> int:
    """Tosses per arm needed for the ±``half_band`` gain gate to be decidable.

    ``se = √2·σ/√n / (2δ) / gain`` and the gate needs ``k·se ≤ half_band``.
    Printed by ``--dry-run`` so the operator learns the rung's resolving power
    before spending it, not after.
    """
    if probe_rad <= 0.0 or gain_mm_per_rad <= 0.0 or half_band <= 0.0:
        return 0
    se_unit = math.sqrt(2.0) * float(sigma_mm) / (2.0 * probe_rad) / gain_mm_per_rad
    return int(math.ceil((GATE_K_SE * se_unit / half_band) ** 2))


def sc1_fit(points: Sequence[Tuple[float, float]]) -> Dict[str, Any]:
    """OLS of ``log|L|`` on ``log h`` with a 95 % CI on the slope.

    ``points`` is ``[(h_m, |mean L| mm), ...]`` — the magnitude of each height's
    MEAN landing error, not the mean of the magnitudes: ``E|L| > |E L|`` under
    noise, and using the biased one tilts every exponent toward zero.
    """
    rows = [(float(h), float(m)) for h, m in points
            if math.isfinite(float(h)) and float(h) > 0.0
            and math.isfinite(float(m)) and float(m) > 0.0]
    if len(rows) < 3:
        raise TossCalGridError(
            'SC-1 needs at least 3 heights with a positive mean landing error; '
            'got {}'.format(len(rows)))
    x = np.array([math.log(h) for h, _ in rows])
    y = np.array([math.log(m) for _, m in rows])
    xm, ym = float(x.mean()), float(y.mean())
    sxx = float(((x - xm) ** 2).sum())
    slope = float(((x - xm) * (y - ym)).sum() / sxx)
    intercept = ym - slope * xm
    resid = y - (intercept + slope * x)
    dof = len(rows) - 2
    s2 = float((resid ** 2).sum() / dof)
    se = math.sqrt(s2 / sxx) if sxx > 0.0 else float('inf')
    # Two-sided 95 % Student-t, df 1..4. Tabulated rather than pulled from scipy:
    # this file runs under the system python3.8 with ROS sourced, where scipy is
    # not guaranteed, and four constants are cheaper than a dependency.
    t_crit = {1: 12.706205, 2: 4.302653, 3: 3.182446, 4: 2.776445}.get(dof, 2.0)
    half = t_crit * se
    return {'slope': slope, 'ci_lo': slope - half, 'ci_hi': slope + half,
            'se': se, 'dof': dof, 'n_heights': len(rows), 't_crit': t_crit,
            'points': rows}


def sc1_units_verdict(fit_result: Dict[str, Any]) -> GateVerdict:
    """§ 3.8's branch table, VERBATIM, applied to the exponent's CI."""
    lo = float(fit_result['ci_lo'])
    hi = float(fit_result['ci_hi'])
    slope = float(fit_result['slope'])
    for name, (blo, bhi), text in SC1_BRANCHES:
        if lo >= blo and hi <= bhi:
            return GateVerdict(
                VERDICT_PASS,
                'SC-1 units = {} — exponent {:.3f}, 95 % CI [{:.3f}, {:.3f}] '
                'inside [{:.1f}, {:.1f}]: {}'
                .format(name, slope, lo, hi, blo, bhi, text),
                dict(fit_result, units=name))
    return GateVerdict(
        VERDICT_INCONCLUSIVE,
        'SC-1 units = {} — exponent {:.3f}, 95 % CI [{:.3f}, {:.3f}] spans more '
        'than one branch. Store at the WORKING HEIGHT ONLY and WARN when the map '
        'is applied at another throw_height_m (the hull-clamp doctrine on the '
        'height axis). This is the EXPECTED outcome at these sample sizes — see '
        'the module docstring; the rung\'s real product is the first honest '
        'sigma_L.'.format(SC1_UNITS_AMBIGUOUS, slope, lo, hi),
        dict(fit_result, units=SC1_UNITS_AMBIGUOUS))


def sc3_verdict(poses: Sequence[Dict[str, Any]], *,
                accuracy_mm: float = SC3_ACCURACY_MM,
                min_passing: int = SC3_MIN_PASSING,
                common_mode_mm: float = SC3_COMMON_MODE_MM) -> GateVerdict:
    """§ 3.8's verification gates over the off-node check poses.

    ``poses`` rows carry ``label``, ``mean_mm`` (2-vector), ``se_mm`` and ``n``.
    Each pose is three-valued against ``|mean L| ≤ accuracy_mm``; the rung FAILs
    when more than ``len(poses) − min_passing`` poses are FAIL, and the pooled
    common mode ``|c|`` is gated against ``common_mode_mm`` (it must fit inside
    one session-trim authority).
    """
    rows = []
    for pose in poses:
        mean = pose.get('mean_mm') or [float('nan'), float('nan')]
        mag = math.hypot(float(mean[0]), float(mean[1]))
        se = float(pose.get('se_mm', float('nan')))
        verdict = interval_verdict(mag, se, 0.0, float(accuracy_mm))
        rows.append({'label': pose.get('label'), 'mean_mm': list(mean),
                     'magnitude_mm': mag, 'se_mm': se,
                     'n': int(pose.get('n', 0)), 'verdict': verdict})
    n_fail = sum(1 for r in rows if r['verdict'] == VERDICT_FAIL)
    n_pass = sum(1 for r in rows if r['verdict'] == VERDICT_PASS)
    allowed_fail = max(0, len(rows) - int(min_passing))

    cx = [float(r['mean_mm'][0]) for r in rows if math.isfinite(r['magnitude_mm'])]
    cy = [float(r['mean_mm'][1]) for r in rows if math.isfinite(r['magnitude_mm'])]
    mx, _, nx = _mean_sd(cx)
    my, _, _ = _mean_sd(cy)
    c_mag = math.hypot(mx, my) if nx else float('nan')
    ses = [float(r['se_mm']) for r in rows if math.isfinite(float(r['se_mm']))]
    c_se = (math.sqrt(sum(s * s for s in ses)) / len(ses)) if ses else float('nan')
    common_v = interval_verdict(c_mag, c_se, 0.0, float(common_mode_mm))

    detail = {'poses': rows, 'n_fail': n_fail, 'n_pass': n_pass,
              'allowed_fail': allowed_fail, 'common_mode_mm': c_mag,
              'common_mode_se_mm': c_se, 'common_mode_verdict': common_v,
              'accuracy_mm': float(accuracy_mm)}

    if n_fail > allowed_fail or common_v == VERDICT_FAIL:
        return GateVerdict(
            VERDICT_FAIL,
            'SC-3 FAIL — {} of {} check poses EXCEED {:.1f} mm on the evidence '
            '(at most {} may), pooled common mode |c| = {:.2f} mm [{}]. Revert: '
            '  git checkout config/toss_calibration.yaml  and call '
            'toss/reload_calibration.'
            .format(n_fail, len(rows), accuracy_mm, allowed_fail, c_mag, common_v),
            detail)
    if n_pass >= int(min_passing) and common_v == VERDICT_PASS:
        return GateVerdict(
            VERDICT_PASS,
            'SC-3 PASS — {} of {} check poses PROVEN within {:.1f} mm, pooled '
            'common mode |c| = {:.2f} mm within {:.1f} mm'
            .format(n_pass, len(rows), accuracy_mm, c_mag, common_mode_mm),
            detail)
    return GateVerdict(
        VERDICT_INCONCLUSIVE,
        'SC-3 INCONCLUSIVE — no check pose EXCEEDS {:.1f} mm on the evidence, '
        'but only {} of {} are PROVEN inside it (need {}). The map is not '
        'refuted and is not certified: {} tosses per pose cannot resolve a '
        '{:.1f} mm bound against the measured scatter. Raise --sc3-n or accept '
        'the map on the point estimates, deliberately.'
        .format(accuracy_mm, n_pass, len(rows), min_passing,
                min((r['n'] for r in rows), default=0), accuracy_mm),
        detail)


def sc3_resolvable_n(sigma_mm: float, accuracy_mm: float = SC3_ACCURACY_MM
                     ) -> int:
    """Tosses per check pose needed for the ``accuracy_mm`` bound to be
    decidable (``k·se ≤ accuracy``)."""
    if accuracy_mm <= 0.0:
        return 0
    return int(math.ceil((GATE_K_SE * float(sigma_mm) / float(accuracy_mm)) ** 2))


# ── pure core: corpus reduction for the gates ────────────────────────────


def land_err_of(rec: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    """The ONE admissible aim observable, or ``None``.

    ``catch_error_mm`` is never consulted, in either its declared or its mined
    form: D5 forbids it (a dead-reckoned free-fall extrapolation, a scalar
    distance not a signed 2-vector, and defined only for CAUGHT balls). It is
    carried in the CSV as a DIAGNOSTIC so that forbidding it stays checkable.
    """
    value = rec.get('land_err_mm')
    if (isinstance(value, (list, tuple)) and len(value) == 2
            and all(isinstance(v, (int, float)) for v in value)
            and all(math.isfinite(float(v)) for v in value)):
        return (float(value[0]), float(value[1]))
    return None


def admitted_land_errors(records: Sequence[Dict[str, Any]]
                         ) -> Tuple[List[Dict[str, Any]], Dict[str, int]]:
    """Split a corpus into rows usable for a gate, with the refusal census.

    Admission is ``toss_trim.admit_for_aim`` — the SAME filter the offline fit
    and the online trim use (2e moved it there precisely so there is one). A
    second admission rule here would let a rung certify a plant the fit then
    refuses.
    """
    keep: List[Dict[str, Any]] = []
    census: Dict[str, int] = {}
    for rec in records:
        ok, reason = toss_trim.admit_for_aim(rec)
        if not ok:
            census[reason] = census.get(reason, 0) + 1
            continue
        err = land_err_of(rec)
        if err is None:
            census['no_land_err'] = census.get('no_land_err', 0) + 1
            continue
        keep.append(rec)
        census['admitted'] = census.get('admitted', 0) + 1
    return keep, census


def group_by_applied_aim(records: Sequence[Dict[str, Any]], arms:
                         Sequence[Tuple[str, Tuple[float, float]]],
                         tol_rad: float = 1e-5
                         ) -> Dict[str, List[Tuple[float, float]]]:
    """Bin admitted records onto the SC-0 arms by the aim they actually applied.

    Keyed on the record's own ``total_aim_rad`` rather than on capture order:
    a record knows what the machine commanded for it, and an ordering assumption
    silently mis-bins every toss after a dropped goal.
    """
    out: Dict[str, List[Tuple[float, float]]] = {name: [] for name, _ in arms}
    for rec in records:
        aim = toss_trim.applied_aim_rad(rec)
        err = land_err_of(rec)
        if aim is None or err is None:
            continue
        for name, want in arms:
            if (abs(aim[0] - want[0]) <= tol_rad
                    and abs(aim[1] - want[1]) <= tol_rad):
                out[name].append(err)
                break
    return out


def group_by_height(records: Sequence[Dict[str, Any]],
                    heights_m: Sequence[float], tol_m: float = 0.02
                    ) -> Dict[float, List[Tuple[float, float]]]:
    """Bin admitted records onto the SC-1 height ladder by their COMMANDED apex."""
    out: Dict[float, List[Tuple[float, float]]] = {float(h): [] for h in heights_m}
    for rec in records:
        apex = rec.get('goal_throw_height_m')
        if apex is None:
            apex = rec.get('apex_height_m')
        err = land_err_of(rec)
        if apex is None or err is None:
            continue
        for h in heights_m:
            if abs(float(apex) - float(h)) <= tol_m:
                out[float(h)].append(err)
                break
    return out


def group_by_pose(records: Sequence[Dict[str, Any]],
                  poses: Sequence[Tuple[float, float]],
                  tol_mm: float = 1.0) -> Dict[int, List[Tuple[float, float]]]:
    """Bin admitted records onto the SC-3 check poses by nominated catch xy."""
    out: Dict[int, List[Tuple[float, float]]] = {i: [] for i in range(len(poses))}
    for rec in records:
        goal = rec.get('goal_catch_xyz_stow_mm')
        err = land_err_of(rec)
        if not (isinstance(goal, (list, tuple)) and len(goal) >= 2) or err is None:
            continue
        for i, (px, py) in enumerate(poses):
            if (abs(float(goal[0]) - float(px)) <= tol_mm
                    and abs(float(goal[1]) - float(py)) <= tol_mm):
                out[i].append(err)
                break
    return out


def sigma_from_records(records: Sequence[Dict[str, Any]],
                       key_fn) -> Optional[float]:
    """Pooled within-group per-axis sd of ``land_err_mm`` — the first honest
    ``σ_L`` (SC-1's real product, § 3.8)."""
    groups: Dict[Any, List[Tuple[float, float]]] = {}
    for rec in records:
        err = land_err_of(rec)
        if err is None:
            continue
        groups.setdefault(key_fn(rec), []).append(err)
    resid: List[float] = []
    dof = 0
    for rows in groups.values():
        if len(rows) < 2:
            continue
        pts = np.asarray(rows, dtype=float)
        resid.extend((pts - pts.mean(axis=0)).ravel().tolist())
        dof += (pts.shape[0] - 1) * 2
    if dof <= 0:
        return None
    return math.sqrt(sum(r * r for r in resid) / dof)


# ── pure core: probe map ─────────────────────────────────────────────────


def probe_map_document(x_mm: Sequence[float], y_mm: Sequence[float],
                       z_mm: float, aim_rad: Tuple[float, float], *,
                       tilt_map_version: str, base_condition: str,
                       argv: Optional[Sequence[str]] = None,
                       flight_time_s: float = 0.7977,
                       date: Optional[str] = None) -> Dict[str, Any]:
    """A UNIFORM map that commands exactly ``aim_rad`` everywhere.

    Every node carries the same value, so ``toss_cal.lookup`` returns it
    bit-identically at every query in the hull and the clamp returns it outside —
    which is what makes an SC-0 arm a *commanded* aim rather than an interpolated
    approximation to one.

    ``captured.probe`` is stamped true, and ``base_condition`` says so in words:
    a probe map found in a git working tree must be recognisable as one at a
    glance, because it is a calibration nobody fitted.
    """
    rx, ry = float(aim_rad[0]), float(aim_rad[1])
    n_x, n_y = len(x_mm), len(y_mm)
    if n_x < 2 or n_y < 2:
        raise TossCalGridError(
            'a probe map needs at least a 2x2 grid; got {}x{}'.format(n_x, n_y))
    grid_rx = [[round(rx, 12)] * n_x for _ in range(n_y)]
    grid_ry = [[round(ry, 12)] * n_x for _ in range(n_y)]
    doc = {
        'version': toss_cal.SCHEMA_VERSION,
        'captured': {
            'date': date or datetime.now().strftime('%Y-%m-%d'),
            'git_sha': tcg.git_sha(),
            'tool': TOOL_NAME,
            'args': list(argv if argv is not None else sys.argv[1:]),
            'probe': True,
            'base_condition': 'SC-0 PROBE MAP ({:+.3f}, {:+.3f}) deg — NOT a '
                              'calibration, restored on exit. {}'
                              .format(math.degrees(rx), math.degrees(ry),
                                      base_condition or ''),
        },
        'requires': {
            'tilt_map_version': str(tilt_map_version or ''),
            'estimator_version': toss_cal.ESTIMATOR_VERSION,
        },
        'units': {'aim': toss_cal.AIM_UNIT, 'height_scaling_exponent': 1.0,
                  'h_capture_m': None},
        'jacobian': {
            'gain_mm_per_rad': float(np.linalg.norm(
                toss_trim.aim_landing_jacobian(flight_time_s, float(z_mm)), 2)),
            'source': 'production aim_target_offset_mm (probe map)',
        },
        'grid': {'z_mm': round(float(z_mm), 6), 'orientation': 'level',
                 'x_mm': [round(float(v), 6) for v in x_mm],
                 'y_mm': [round(float(v), 6) for v in y_mm]},
        'aim_rad': {'rx': grid_rx, 'ry': grid_ry},
        'anchor': {'aim_rad': [0.0, 0.0], 'n': 0, 'se_rad': [None, None]},
        'speed': {'k_v': None, 'se': None},
        'stats': {'n_per_node': [[0] * n_x for _ in range(n_y)],
                  'stale_nodes': [], 'failed_nodes': []},
    }
    fit.validate_map_document(doc)      # through the PRODUCTION loader
    return doc


# ── pure core: the rung ledger ───────────────────────────────────────────


def ledger_path(out_dir: str) -> str:
    return os.path.join(out_dir, LEDGER_NAME)


def load_ledger(path: str) -> Dict[str, Any]:
    """Read the ledger, or an empty one. A corrupt ledger is a REFUSAL, never a
    silent reset — "no SC-0 row" and "an unreadable SC-0 row" must not look the
    same to the rung that is about to spend 72 tosses on the difference."""
    if not os.path.exists(path):
        return {'schema': LEDGER_SCHEMA, 'rungs': {}}
    try:
        with open(path) as handle:
            data = json.load(handle)
    except (OSError, ValueError) as exc:
        raise TossCalGridError(
            'the rung ledger {} exists but could not be read ({}). Fix or delete '
            'it deliberately — a blocking precondition must never be satisfied '
            'by an unreadable file.'.format(path, exc))
    if not isinstance(data, dict) or 'rungs' not in data:
        raise TossCalGridError(
            'the rung ledger {} is not a {} document'.format(path, LEDGER_SCHEMA))
    return data


def save_ledger(path: str, data: Dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w') as handle:
        json.dump(data, handle, indent=2, sort_keys=True, default=str)


def record_rung(ledger: Dict[str, Any], rung: str, verdict: str, *,
                headline: str = '', detail: Optional[Dict[str, Any]] = None,
                extra: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    row = {
        'rung': rung,
        'verdict': verdict,
        'headline': headline,
        'iso': datetime.now().isoformat(timespec='seconds'),
        'git_sha': tcg.git_sha(),
        'detail': detail or {},
    }
    row.update(extra or {})
    ledger.setdefault('rungs', {})[rung] = row
    return row


def rung_precondition(ledger: Dict[str, Any], rung: str, *,
                      allow_5x5: bool = False, nodes: int = 3
                      ) -> Tuple[bool, str]:
    """Which earlier rungs must have been SCORED before ``rung`` may run.

    This is what makes "SC-0 BLOCKS everything" (D14) structural. SC-0 must be
    PASS **or INCONCLUSIVE**: INCONCLUSIVE means the sign is proven and only the
    ±25 % gain band is unresolved, which is exactly the part of the rung the
    sample size cannot decide — blocking on it would block every capture forever
    (module docstring). SC-0 FAIL and SC-0 UNSCORED both block.
    """
    rows = (ledger or {}).get('rungs', {})

    def _state(name: str) -> str:
        return str((rows.get(name) or {}).get('verdict') or 'MISSING')

    if rung in ('sc0', 'sc1'):
        return (True, '')
    if rung in ('sc2', 'sc3'):
        sc0 = _state('sc0')
        if sc0 == VERDICT_FAIL:
            return (False,
                    'SC-0 is recorded FAIL — the aim->landing sign is inverted. '
                    'Capturing a grid on an inverted sign spends a whole sitting '
                    'producing a map that aims the machine twice as badly as no '
                    'map. Fix the sign, re-run --rung sc0, then come back.')
        if sc0 in ('MISSING', VERDICT_UNSCORED):
            return (False,
                    'SC-0 is {} in the rung ledger. It BLOCKS every later rung '
                    '(D14): the sign must be MEASURED, not assumed from the '
                    'geometry. Run  --rung sc0  and then  --rung sc0 --score '
                    '<mined corpus>.'.format(sc0.lower()))
        if rung == 'sc2':
            sc1 = _state('sc1')
            if sc1 in ('MISSING', VERDICT_UNSCORED):
                return (False,
                        'SC-1 is {} in the rung ledger. It decides the map\'s '
                        'UNITS, and getting that wrong makes the map silently '
                        'invalid at any other throw_height_m. Run  --rung sc1  '
                        'and score it (an INCONCLUSIVE exponent is a legitimate '
                        'and expected result — it selects the working-height-only'
                        ' branch — but it must be MEASURED).'.format(sc1.lower()))
            if int(nodes) > 3 and not allow_5x5:
                sc3 = rows.get('sc3') or {}
                if str(sc3.get('verdict')) != VERDICT_PASS:
                    return (False,
                            'a {0}x{0} capture is {1} tosses and § 3.8 orders a '
                            '3x3 FIRST: the first capture\'s job is to find out '
                            'whether a per-node FIELD exists at all, and a 5x5 '
                            'is 200 tosses of possibly-flat data that does not '
                            'fit a comfortable sitting. Verify a 3x3 (--rung '
                            'sc3 => PASS) or pass --allow-5x5 deliberately.'
                            .format(int(nodes), int(nodes) ** 2 * 8))
    return (True, '')


# ── pure core: cancel disposition ────────────────────────────────────────


def cancel_disposition(phase: Optional[str]) -> Tuple[bool, str]:
    """``(deferred, operator text)`` for a Ctrl-C at ``phase`` — § 3.8's wording.

    The tool cannot see ``TOSS_CANCEL_CUTOFF_S`` from outside, so ``THROWING``
    is reported as *may be deferred*: the coordinator honours a cancel before
    the cutoff and defers after it, and claiming certainty either way would be a
    lie about an airborne ball.
    """
    text = str(phase or 'UNKNOWN')
    if text in CANCEL_DEFERRED_PHASES:
        qualifier = ('may be DEFERRED (the stroke may already have fired)'
                     if text == 'THROWING'
                     else 'DEFERRED to the catch terminal (~1.2 s)')
        return (True,
                'CANCEL requested — cycle is in {}: {}.\n'
                'A second Ctrl-C will NOT skip this wait. Ball is airborne.'
                .format(text, qualifier))
    return (False,
            'CANCEL requested — cycle is in {}: honoured NOW (nothing is '
            'airborne).'.format(text))


def node_exhausted(outcome: Optional[str]) -> bool:
    """Operator decision 4 / D19: a session that ended on the reload budget is a
    NODE skip, not a capture abort."""
    return str(outcome or '') == NODE_EXHAUSTED_OUTCOME


# ── pure core: CSV rows ──────────────────────────────────────────────────


def csv_row(iso: str, t_s: float, goal: GoalSpec, record: Dict[str, Any],
            uptime_ms: Optional[int], note: str = '') -> Dict[str, Any]:
    """One per-toss CSV row, appended **as the toss completes**.

    Never built in a trailing loop: on any mid-capture abort a trailing loop
    never runs, and the artefact would be empty exactly when the operator most
    needs it (§ 3.8's forensics rule, a tilt-cal finding).
    """
    def _pair(name, index):
        value = record.get(name)
        if isinstance(value, (list, tuple)) and len(value) > index:
            try:
                return float(value[index])
            except (TypeError, ValueError):
                return ''
        return ''

    probe = goal.probe_deg
    return {
        'iso': iso,
        't_s': round(float(t_s), 3),
        'rung': goal.rung,
        'goal_index': int(goal.index),
        'label': goal.label,
        'ix': int(goal.ix),
        'iy': int(goal.iy),
        'x_mm': round(float(goal.x_mm), 3),
        'y_mm': round(float(goal.y_mm), 3),
        'z_mm': round(float(goal.z_mm), 3),
        'throw_height_m': round(float(goal.throw_height_m), 4),
        'probe_rx_deg': '' if math.isnan(probe[0]) else round(probe[0], 5),
        'probe_ry_deg': '' if math.isnan(probe[1]) else round(probe[1], 5),
        'toss_uid': record.get('toss_uid', ''),
        'cycle_index': record.get('cycle_index', ''),
        'outcome': record.get('outcome', ''),
        'total_aim_rx_rad': _pair('total_aim_rad', 0),
        'total_aim_ry_rad': _pair('total_aim_rad', 1),
        'map_aim_rx_rad': _pair('map_aim_rad', 0),
        'map_aim_ry_rad': _pair('map_aim_rad', 1),
        'flight_time_s': record.get('flight_time_s', ''),
        # DIAGNOSTIC ONLY. D5 forbids it as an estimator input; it is carried so
        # that forbidding it stays checkable from the artefact.
        'catch_error_mm_fsm_diagnostic': record.get('catch_error_mm_fsm', ''),
        'dwell_tilt_n': record.get('dwell_tilt_n', ''),
        'dwell_tilt_rx_rad': _pair('dwell_tilt_rad', 0),
        'dwell_tilt_ry_rad': _pair('dwell_tilt_rad', 1),
        'uptime_ms': '' if uptime_ms is None else int(uptime_ms),
        'note': note,
    }


# ── CLI ──────────────────────────────────────────────────────────────────


def build_parser() -> argparse.ArgumentParser:
    ap = tcg._Parser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='Runbook: tests/hardware/session_phase8_toss_hardware.md')

    grid = ap.add_argument_group('grid spec')
    grid.add_argument('--x', default=None,
                      help='explicit x node list in mm (comma/space separated, '
                           'strictly increasing), e.g. --x=-150,0,150. Use the '
                           '"=" form when the list starts negative.')
    grid.add_argument('--y', default=None, help='explicit y node list in mm.')
    grid.add_argument('--box', type=float, default=DEFAULT_BOX_MM,
                      help='half-width in mm of a symmetric square grid '
                           '(default %(default)s).')
    grid.add_argument('--nodes', type=int, default=DEFAULT_NODES,
                      help='nodes per axis (default %(default)s — § 3.8 orders a '
                           '3x3 FIRST; must be odd so (0,0) is a node).')
    grid.add_argument('--z', type=float, default=DEFAULT_Z_MM,
                      help='catch-pose z in mm (default %(default)s).')
    grid.add_argument('--allow-5x5', action='store_true',
                      help='permit --nodes 5 before a 3x3 has verified (D14).')

    timing = ap.add_argument_group('timing (PROVISIONAL — the first capture pins them)')
    timing.add_argument('--n-per-node', type=int, default=DEFAULT_N_PER_NODE,
                        help='tosses per SC-1 height / SC-2 node (default '
                             '%(default)s).')
    timing.add_argument('--sc0-n', type=int, default=DEFAULT_SC0_N,
                        help='tosses per SC-0 arm (default %(default)s).')
    timing.add_argument('--sc3-n', type=int, default=DEFAULT_SC3_N,
                        help='tosses per SC-3 check pose (default %(default)s).')
    timing.add_argument('--sc0-probe-deg', type=float,
                        default=DEFAULT_SC0_PROBE_DEG,
                        help='SC-0 probe aim magnitude in degrees (default '
                             '%(default)s). se of the gain ratio scales as 1/δ; '
                             'the landing displacement is 4h·δ = 27 mm at 0.5°, '
                             '55 mm at 1.0° against an 80 mm reach envelope.')
    timing.add_argument('--throw-height-m', type=float,
                        default=DEFAULT_THROW_HEIGHT_M,
                        help='apex for SC-0/SC-2/SC-3 (default %(default)s).')
    timing.add_argument('--dwell-s', type=float, default=DEFAULT_DWELL_S,
                        help='session dwell (default %(default)s — operator '
                             'decision 3 keeps the 6.0 s cadence).')
    timing.add_argument('--throw-delay-s', type=float,
                        default=DEFAULT_THROW_DELAY_S,
                        help='delay from goal accept to the FIRST release '
                             '(default %(default)s).')
    timing.add_argument('--catch-vel-scale', type=float, default=0.0,
                        help='0 => config default (clamped [0.3, 1.5]).')
    timing.add_argument('--max-reloads', type=int, default=DEFAULT_MAX_RELOADS,
                        help='per-goal reload budget (default %(default)s). On '
                             'exhaustion the SESSION fails closed and this tool '
                             'marks the node thin/stale and SKIPS to the next.')
    timing.add_argument('--timeout-s', type=float, default=DEFAULT_TIMEOUT_S,
                        help='per-service-call timeout (default %(default)s).')
    timing.add_argument('--goal-timeout-s', type=float, default=0.0,
                        help='per-goal wall-clock ceiling; 0 => derived from the '
                             'cadence with a 3x margin.')
    timing.add_argument('--home-revisit-every', type=int,
                        default=DEFAULT_HOME_REVISIT_EVERY,
                        help='insert a home ANCHOR goal after every N SC-2 nodes '
                             '(default %(default)s; 0 disables). Every shipped '
                             'residual is referenced to the anchor MEAN.')
    timing.add_argument('--sensor-probe-timeout-s', type=float, default=90.0,
                        help='how long R5 waits for a live held->empty edge '
                             '(default %(default)s).')

    ver = ap.add_argument_group('verification')
    ver.add_argument('--check-poses', type=int, default=DEFAULT_CHECK_POSES,
                     help='off-node SC-3 poses (default %(default)s; § 3.8 asks '
                          'for >= 6).')
    ver.add_argument('--accuracy-mm', type=float, default=SC3_ACCURACY_MM,
                     help='SC-3 per-pose accuracy bound (default %(default)s).')
    ver.add_argument('--verify-only', action='store_true',
                     help='alias for --rung sc3: score the ALREADY-LOADED map at '
                          'the off-node check poses and write nothing.')

    misc = ap.add_argument_group('workflow')
    misc.add_argument('--rung', choices=RUNGS, default='sc0',
                      help='which § 3.8 rung to run (default %(default)s — the '
                           'first-ever capture starts there, it BLOCKS every '
                           'later rung, and a mistaken bare invocation then '
                           'spends the CHEAPEST rung rather than a 72-toss grid).')
    misc.add_argument('--score', action='append', default=[],
                      help='desk-side: score this rung from a MINED corpus '
                           '(JSONL path or glob, repeatable) and write the '
                           'verdict into the rung ledger. Makes ZERO ROS calls.')
    misc.add_argument('--sigma-mm', type=float, default=DEFAULT_SIGMA_MM,
                      help='assumed per-axis arrival scatter for the POWER '
                           'report only (default %(default)s — the design\'s '
                           'working figure, never measured on this machine). '
                           'Every gate\'s standard error comes from the data.')
    misc.add_argument('--assumed-catch-rate', type=float,
                      default=DEFAULT_CATCH_RATE,
                      help='catch rate used for the ETA and the ball budget '
                           '(default %(default)s, § 6\'s table).')
    misc.add_argument('--on-fail', choices=('continue', 'abort'),
                      default='continue',
                      help='per-goal failure policy (default %(default)s). A '
                           'DISARMED wire and a drive fault are RUN-level and '
                           'ignore this.')
    misc.add_argument('--base-condition', default=None,
                      help='free text recorded in the meta, e.g. "flat floor, '
                           'fresh bridge, BB magazine 12". Prompted for if '
                           'omitted.')
    misc.add_argument('--no-apply', action='store_true',
                      help='capture and write the CSV/meta, but never write a '
                           'map, never reload. SC-0 still needs its probe maps, '
                           'so --no-apply refuses --rung sc0.')
    misc.add_argument('--force-uninstall', action='store_true',
                      help='move EVERY existing map candidate aside (source tree '
                           'AND any ament share copy, each to a timestamped '
                           '.bak; refuses a $JUGGLEBOT_TOSS_CAL-pointed file) '
                           'and reload so the capture starts with '
                           'toss_cal_loaded=false.')
    misc.add_argument('--dry-run', action='store_true',
                      help='print the goal sequence, toss count, ETA, ball '
                           'budget and gate power; make ZERO ROS calls and '
                           'construct no ROS objects.')
    misc.add_argument('--yes', action='store_true',
                      help='skip the interactive operator confirmations (R9) '
                           'with a loud live-capture warning. It does NOT skip '
                           'R5 — the sensor edge is machine-checked ground '
                           'truth, not a confirmation.')
    misc.add_argument('--out-dir', default=None,
                      help='CSV/meta/ledger directory (default temp/logs/).')
    return ap


def resolve_grid(args) -> Tuple[List[float], List[float]]:
    x_mm = (tcg.parse_node_list(args.x, 'x') if args.x
            else tcg.build_axis(args.box, args.nodes, 'x'))
    y_mm = (tcg.parse_node_list(args.y, 'y') if args.y
            else tcg.build_axis(args.box, args.nodes, 'y'))
    tcg.home_index(x_mm, y_mm)      # refuse early if (0, 0) is not a node
    return x_mm, y_mm


def print_plan(rung: str, x_mm: Sequence[float], y_mm: Sequence[float],
               z_mm: float, goals: Sequence[GoalSpec], args) -> None:
    print('Toss AIM-calibration capture — rung {} at z={} mm'
          .format(rung.upper(), z_mm))
    print('  grid x_mm: {}'.format(list(x_mm)))
    print('  grid y_mm: {}'.format(list(y_mm)))
    print('  dwell={:.2f}s  throw_delay={:.2f}s  max_reloads={}  '
          'on_empty_cup=RELOAD  stop_on_miss=false'
          .format(args.dwell_s, args.throw_delay_s, args.max_reloads))
    print('  goal order:')
    for g in goals:
        tag = '  <- ANCHOR' if g.is_anchor else ''
        aim = ('' if g.probe_aim_rad is None else
               '  probe aim ({:+.3f}, {:+.3f}) deg'.format(*g.probe_deg))
        print('    {:>3d}. {:<44s} ({:+7.1f}, {:+7.1f}) h={:.2f} m  n={}{}{}'
              .format(g.index + 1, g.label, g.x_mm, g.y_mm, g.throw_height_m,
                      g.n_throws, aim, tag))
    budget = ball_budget(goals, max_reloads=args.max_reloads,
                         catch_rate=args.assumed_catch_rate)
    eta = estimate_duration_s(goals, dwell_s=args.dwell_s,
                              throw_delay_s=args.throw_delay_s,
                              catch_rate=args.assumed_catch_rate)
    print('  TOSS COUNT {} over {} goal(s)'
          .format(budget['tosses'], budget['goals']))
    print('  ETA ~ {:.0f} s ({:.0f} min) at an assumed {:.0%} catch rate '
          '(drops priced at {:.0f} s all-in)'
          .format(eta, eta / 60.0, args.assumed_catch_rate, RELOAD_COST_S))
    print('  BALL BUDGET: worst case {} reload(s) ({} per goal x {} goals); '
          'expected ~{:.0f} drop(s) at {:.0%}.'
          .format(budget['worst_case_reloads'], budget['max_reloads_per_goal'],
                  budget['goals'], budget['expected_drops'],
                  args.assumed_catch_rate))
    print('    Ball supply is OPERATOR-MANAGED — ball_butler_node has no '
          'ball-count or magazine field, so no machine fence exists (D19). '
          'max_reloads is the only automatic limit and it fails the SESSION '
          'closed; this tool then marks the node thin/stale and SKIPS to the '
          'next one.')

    # Power report: what this rung can and cannot decide, BEFORE it is spent.
    T = math.sqrt(8.0 * float(args.throw_height_m) / 9.80665)
    gain = float(np.linalg.norm(
        toss_trim.aim_landing_jacobian(T, float(z_mm)), 2))
    print('  GATE POWER at an assumed sigma = {:.0f} mm/axis (the design\'s '
          'working figure, NEVER measured on this machine):'
          .format(args.sigma_mm))
    if rung == 'sc0':
        need = sc0_resolvable_n(args.sigma_mm, math.radians(args.sc0_probe_deg),
                                gain)
        se = (math.sqrt(2.0) * args.sigma_mm / math.sqrt(max(1, args.sc0_n))
              / (2.0 * math.radians(args.sc0_probe_deg)) / gain)
        print('    gain ratio se ~ {:.3f} at n={} => the +-25 % band is '
              '{:.1f} se wide. Resolving it needs n >= {} PER ARM ({} tosses).'
              .format(se, args.sc0_n, 0.25 / se if se > 0 else float('inf'),
                      need, need * 5))
        print('    The SIGN — what this rung exists to protect — is resolved at '
              'n=5 (0 leaks in 24 000 probe trials).')
    elif rung == 'sc1':
        print('    the exponent CI is ~4.4 wide at n=8, sigma=20 (median over '
              '20 000 probe ladders) against a 0.6-wide rad band: the units '
              'decision lands INSIDE a branch ~0.2 % of the time. Expect '
              '{}; the rung\'s real product is the first honest sigma_L.'
              .format(SC1_UNITS_AMBIGUOUS))
    elif rung == 'sc3':
        need = sc3_resolvable_n(args.sigma_mm, args.accuracy_mm)
        print('    proving |mean L| <= {:.1f} mm needs n >= {} per check pose '
              '({} tosses); at n={} the gate can only REFUTE a bad map, not '
              'certify a good one.'
              .format(args.accuracy_mm, need, need * len(goals), args.sc3_n))
    else:
        print('    SC-2 has no gate of its own — the write rules and both '
              'write-refusing guards live in toss_cal_fit.py (phase 2c).')

    print('\n  AFTER this rung:')
    if rung == 'sc3':
        print('    1. python tools/probes/toss_record_miner.py --bag <bag> --jsonl')
        print('    2. python3 {} --rung sc3 --score <corpus.jsonl>'
              .format(TOOL_NAME))
        print('    3. KEEP or REVERT (§ 6 decision D-i): a FAIL means  git '
              'checkout config/toss_calibration.yaml  followed by  ros2 service '
              'call /toss/reload_calibration std_srvs/srv/Trigger.')
    if rung in ('sc0', 'sc1', 'sc2'):
        print('    1. python tools/probes/toss_record_miner.py --bag <bag> --jsonl')
        print('    2. python3 {} --rung {} --score <corpus.jsonl>'
              .format(TOOL_NAME, rung))
    if rung == 'sc2':
        print('    3. python tests/hardware/toss_cal_fit.py --corpus '
              '<corpus.jsonl> --dry-run   (then without --dry-run to write)')
        print('    4. FIRST HARDWARE APPLICATION OF A NEW MAP USES A '
              'DELIBERATELY DOUBLED BIAS ON ONE NODE (§ 5 P5.4):')
        print('         python3 {} --rung sc0 --sc0-probe-deg <|home bias| in '
              'deg> --sc0-n 5'.format(TOOL_NAME))
        print('       SC-0 IS the doubled-bias check, run at the fitted '
              'magnitude and with a gate on it. The design\'s literal wording '
              '("if doubling the bias doubles the error, the sign is wrong") is '
              'loose: with the CORRECT sign the error ladder over bias x0/x1/x2 '
              'is |b| -> ~0 -> |b| REVERSED, and with the wrong sign it is '
              '|b| -> 2|b| -> 3|b|. What discriminates is the DIRECTION, which '
              'is what SC-0 measures — one node, five tosses, a named verdict, '
              'instead of a whole grid.')
        print('    5. python3 {} --rung sc3        (the real verification pass)'
              .format(TOOL_NAME))


# ── desk-side scoring ────────────────────────────────────────────────────


def expand_corpus(patterns: Sequence[str]) -> List[str]:
    paths: List[str] = []
    for pattern in patterns:
        hits = sorted(glob.glob(pattern))
        if not hits:
            if os.path.exists(pattern):
                hits = [pattern]
            else:
                raise TossCalGridError(
                    'no corpus file matches {!r}'.format(pattern))
        paths.extend(hits)
    seen, out = set(), []
    for path in paths:
        if path not in seen:
            seen.add(path)
            out.append(path)
    return out


def score_rung(rung: str, records: Sequence[Dict[str, Any]], args,
               x_mm: Sequence[float], y_mm: Sequence[float]) -> GateVerdict:
    """Evaluate one rung's gate from a MINED corpus. Pure, no ROS."""
    admitted, census = admitted_land_errors(records)
    print('  admission census: {}'.format(
        ', '.join('{}={}'.format(k, v) for k, v in sorted(census.items()))
        or '<empty>'))
    if not admitted:
        return GateVerdict(
            VERDICT_INCONCLUSIVE,
            '{} UNSCORABLE — not one record survived admission. The dominant '
            'refusal above names why; `no_mocap_fit` means the descending '
            'branch was never tracked over the cup, which is a capture '
            'precondition this corpus fails (§ 10, the 2a finding).'
            .format(rung.upper()),
            {'census': census})

    T = math.sqrt(8.0 * float(args.throw_height_m) / 9.80665)
    jac = toss_trim.aim_landing_jacobian(T, float(args.z))

    if rung == 'sc0':
        arms = sc0_arms(args.sc0_probe_deg)
        grouped = group_by_applied_aim(admitted, arms)
        for name, _ in arms:
            print('    arm {:<5s} n={}'.format(name, len(grouped.get(name, []))))
        D, se, detail = sc0_measure(grouped, math.radians(args.sc0_probe_deg),
                                    jac)
        detail['census'] = census
        return sc0_verdict(D, se, detail)

    if rung == 'sc1':
        grouped = group_by_height(admitted, SC1_HEIGHTS_M)
        points = []
        for h in SC1_HEIGHTS_M:
            rows = grouped.get(float(h), [])
            if not rows:
                print('    h={:.2f} m: NO admitted tosses'.format(h))
                continue
            pts = np.asarray(rows, dtype=float)
            mean = pts.mean(axis=0)
            mag = float(math.hypot(mean[0], mean[1]))
            print('    h={:.2f} m: n={:<3d} mean L = ({:+7.2f}, {:+7.2f}) mm  '
                  '|L| = {:.2f} mm'.format(h, len(rows), mean[0], mean[1], mag))
            points.append((float(h), mag))
        sigma = sigma_from_records(
            admitted, lambda r: round(float(r.get('goal_throw_height_m') or 0.0), 3))
        result = sc1_fit(points)
        result['sigma_L_mm'] = sigma
        result['census'] = census
        verdict = sc1_units_verdict(result)
        if sigma is not None:
            print('    sigma_L (pooled within-height, per axis) = {:.2f} mm '
                  '-- the FIRST measurement of this number on this machine'
                  .format(sigma))
        return verdict

    if rung == 'sc3':
        checks = off_node_poses(x_mm, y_mm, int(args.check_poses))
        grouped = group_by_pose(admitted, checks)
        poses = []
        for i, (cx, cy) in enumerate(checks):
            rows = grouped.get(i, [])
            if not rows:
                poses.append({'label': '({:+.1f}, {:+.1f})'.format(cx, cy),
                              'mean_mm': [float('nan'), float('nan')],
                              'se_mm': float('nan'), 'n': 0})
                continue
            pts = np.asarray(rows, dtype=float)
            mean = pts.mean(axis=0)
            sd = float(pts.std(ddof=1).mean()) if pts.shape[0] > 1 else float('nan')
            poses.append({'label': '({:+.1f}, {:+.1f})'.format(cx, cy),
                          'mean_mm': [float(mean[0]), float(mean[1])],
                          'se_mm': (sd / math.sqrt(pts.shape[0])
                                    if math.isfinite(sd) else float('nan')),
                          'n': int(pts.shape[0])})
        verdict = sc3_verdict(poses, accuracy_mm=args.accuracy_mm)
        for row in verdict.detail['poses']:
            print('    [{:<12s}] {} |mean L| = {:.2f} +- {:.2f} mm  n={}'
                  .format(row['verdict'], row['label'], row['magnitude_mm'],
                          row['se_mm'], row['n']))
        return verdict

    # SC-2 has no gate of its own: the write rules and both write-refusing
    # guards belong to toss_cal_fit.py. What is scored here is the CENSUS —
    # whether every node has enough admitted tosses to be re-fitted at all.
    counts: Dict[Tuple[float, float], int] = {}
    for rec in admitted:
        goal = rec.get('goal_catch_xyz_stow_mm') or [None, None]
        try:
            key = (round(float(goal[0]), 1), round(float(goal[1]), 1))
        except (TypeError, ValueError, IndexError):
            continue
        counts[key] = counts.get(key, 0) + 1
    thin = [k for k in counts if counts[k] < fit.N_MIN]
    missing = [(float(x), float(y)) for y in y_mm for x in x_mm
               if (round(float(x), 1), round(float(y), 1)) not in counts]
    for key in sorted(counts):
        print('    node {} n={}{}'.format(key, counts[key],
                                          '   THIN' if counts[key] < fit.N_MIN
                                          else ''))
    detail = {'census': census, 'per_node': {str(k): v for k, v in counts.items()},
              'thin_nodes': [list(k) for k in thin],
              'never_flew_nodes': [list(k) for k in missing],
              'n_min': fit.N_MIN}
    if missing:
        return GateVerdict(
            VERDICT_FAIL,
            'SC-2 FAIL — {} node(s) have NO admitted toss at all: {}. A node '
            'that never flew cannot be written (a zero there is not neutral: '
            'the bilinear blend would drag its measured neighbours toward zero '
            'across half a cell).'.format(len(missing), missing),
            detail)
    if thin:
        return GateVerdict(
            VERDICT_INCONCLUSIVE,
            'SC-2 INCONCLUSIVE — {} node(s) below N_MIN={}: {}. They keep their '
            'PREVIOUS value marked stale:true (D15); the rest re-fit normally. '
            'Run toss_cal_fit.py --dry-run to see the node-by-node diff.'
            .format(len(thin), fit.N_MIN, thin), detail)
    return GateVerdict(
        VERDICT_PASS,
        'SC-2 PASS — every grid node carries >= N_MIN={} admitted tosses. Fit '
        'with toss_cal_fit.py.'.format(fit.N_MIN), detail)


def run_score(args) -> int:
    """``--score``: desk-side, ZERO ROS calls."""
    x_mm, y_mm = resolve_grid(args)
    paths = expand_corpus(args.score)
    records = fit.load_corpus(paths)
    print('== score rung {} =='.format(args.rung.upper()))
    print('  corpus: {} record(s) from {} file(s)'.format(len(records), len(paths)))
    verdict = score_rung(args.rung, records, args, x_mm, y_mm)
    print('\n{}'.format(verdict.headline))
    out_dir = args.out_dir or os.path.join(_REPO, 'temp', 'logs')
    path = ledger_path(out_dir)
    ledger = load_ledger(path)
    record_rung(ledger, args.rung, verdict.verdict, headline=verdict.headline,
                detail=verdict.detail,
                extra={'corpus': paths, 'n_records': len(records)})
    save_ledger(path, ledger)
    print('\nledger -> {}'.format(path))
    return 0 if verdict.verdict != VERDICT_FAIL else 1


# ── runtime (ROS) ────────────────────────────────────────────────────────


class _Runner:
    """Thin rclpy client: one action, two services, five subscriptions.

    There is no arming call, no mode change, no hand command and no production
    publisher anywhere in this class, and there must never be one. The single
    actuating primitive is a ``TossContinuous`` goal; the single safing
    primitive is a goal cancel plus ``trajectory/go_home``, which is the same
    service ``reload_coordinator_node`` itself calls to safe the machine.
    """

    def __init__(self, node, args):
        from rclpy.action import ActionClient
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from jugglebot_interfaces.action import TossContinuous
        from jugglebot_interfaces.msg import (HandTelemetryMessage, RobotState,
                                              TrajectoryStatus)
        from diagnostic_msgs.msg import DiagnosticStatus
        from geometry_msgs.msg import Point
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        self.node = node
        self.args = args
        self._TossContinuous = TossContinuous
        self._Trigger = Trigger

        self.status = None
        self.status_stamp = 0.0
        self.link_kv: Dict[str, str] = {}
        self.link_stamp = 0.0
        self.link_seq = 0
        self.uptime_first: Optional[int] = None
        self.uptime_last: Optional[int] = None
        self.cal_status: Dict[str, Any] = {}
        self.commanded_xyz: Optional[Tuple[float, float, float]] = None
        self.commanded_stamp = 0.0
        self.motor_states = None
        self.motor_error_edge: Dict[int, Dict[str, Any]] = {}

        # Hand sensor: R5's ground truth. `_held` tracks the debounced verdict
        # so a held->empty EDGE can be observed, which is what a static read
        # cannot do.
        self.sensor_valid_seen = False
        self.sensor_edge_seen = False
        self._held: Optional[bool] = None

        # Per-toss declarations. Consumed as they arrive; the CSV row is written
        # in the callback's caller, never in a trailing loop.
        self.records: List[Dict[str, Any]] = []
        self._record_uids: set = set()

        self.cli_reload = node.create_client(Trigger, '/toss/reload_calibration')
        self.cli_go_home = node.create_client(Trigger, '/trajectory/go_home')
        self.action = ActionClient(node, TossContinuous, ACTION_NAME)

        node.create_subscription(TrajectoryStatus, '/trajectory/status',
                                 self._on_status, 10)
        node.create_subscription(DiagnosticStatus, '/link_status',
                                 self._on_link, 10)
        node.create_subscription(RobotState, '/robot_state',
                                 self._on_robot_state, 10)
        node.create_subscription(HandTelemetryMessage, '/hand_telemetry',
                                 self._on_hand, 50)
        node.create_subscription(Point, '/trajectory/commanded_position',
                                 self._on_commanded, 10)
        node.create_subscription(String, '/toss/record', self._on_record, 20)
        node.create_subscription(
            String, '/toss/calibration_status', self._on_cal_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

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
            }
            summary.append(entry)
            if ((entry['active_errors'] or entry['disarm_reason'])
                    and i not in self.motor_error_edge):
                self.motor_error_edge[i] = dict(
                    entry, t_wall=time.time(), uptime_ms=self.uptime_last,
                    iso=datetime.now().isoformat(timespec='milliseconds'))
        self.motor_states = summary

    def _on_hand(self, msg):
        valid = bool(getattr(msg, 'ball_held_valid', False))
        if not valid:
            # UNKNOWN never collapses to a verdict (D13) and never advances the
            # edge latch: an invalid sample is not "empty".
            self._held = None
            return
        self.sensor_valid_seen = True
        held = bool(getattr(msg, 'ball_held', False))
        if self._held is True and held is False:
            self.sensor_edge_seen = True
        self._held = held

    def _on_commanded(self, msg):
        self.commanded_xyz = (float(msg.x), float(msg.y), float(msg.z))
        self.commanded_stamp = time.time()

    def _on_cal_status(self, msg):
        try:
            self.cal_status = json.loads(msg.data)
        except ValueError:
            pass

    def _on_record(self, msg):
        try:
            record = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(record, dict):
            return
        uid = str(record.get('toss_uid') or '')
        if uid and uid in self._record_uids:
            return
        if uid:
            self._record_uids.add(uid)
        self.records.append(record)

    # -- forensics ---------------------------------------------------------

    def forensics(self) -> Dict[str, Any]:
        """A pure cache snapshot — zero service calls, safe on any abort path
        including a dead link."""
        now = time.time()
        status = self.status
        snapshot = None
        if status is not None:
            snapshot = {name: getattr(status, name, None) for name in
                        ('mode', 'streaming', 'gravity_correction_loaded',
                         'tilt_map_loaded', 'tilt_map_version')}

        def _decorate(entry):
            out = dict(entry)
            out['active_errors_decoded'] = tcg.decode_error_bits(
                entry.get('active_errors', 0))
            out['disarm_reason_decoded'] = tcg.decode_error_bits(
                entry.get('disarm_reason', 0))
            return out

        return {
            'wall_iso': datetime.now().isoformat(timespec='milliseconds'),
            'uptime_ms_last': self.uptime_last,
            'link_kv': dict(self.link_kv),
            'link_age_s': (round(now - self.link_stamp, 3)
                           if self.link_stamp else None),
            'motor_states': ([_decorate(e) for e in self.motor_states]
                             if self.motor_states else None),
            'motor_error_edges': {str(leg): _decorate(edge) for leg, edge
                                  in sorted(self.motor_error_edge.items())},
            'last_traj_status': snapshot,
            'status_age_s': (round(now - self.status_stamp, 3)
                             if self.status_stamp else None),
            'toss_cal_status': dict(self.cal_status),
            'commanded_position_mm': (list(self.commanded_xyz)
                                      if self.commanded_xyz else None),
            'commanded_age_s': (round(now - self.commanded_stamp, 3)
                                if self.commanded_stamp else None),
            'hand_sensor': {'valid_seen': self.sensor_valid_seen,
                            'held_to_empty_seen': self.sensor_edge_seen,
                            'last_held': self._held},
            'records_seen': len(self.records),
        }

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
            raise TossCalGridError(
                'service {} unavailable after {:.1f} s — is the stack up '
                '(ros2 launch jugglebot jugglebot_launch.py), and were BOTH '
                'jugglebot_interfaces and jugglebot rebuilt?'
                .format(name, self.args.timeout_s))
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future,
                                         timeout_sec=self.args.timeout_s)
        result = future.result()
        if result is None:
            raise TossCalGridError('service {} did not answer within {:.1f} s'
                                   .format(name, self.args.timeout_s))
        return result

    def refresh_link(self, timeout_s: float = 2.0) -> Dict[str, str]:
        """Wait for a /link_status message that provably POST-DATES this call.

        Waiting on the sequence counter rather than sleeping a fixed interval is
        what makes the between-node wire check read fresh state instead of
        whatever happened to be cached before the last goal.
        """
        start_seq = self.link_seq
        end = time.time() + max(0.0, timeout_s)
        while time.time() < end and self.link_seq == start_seq:
            self.spin(0.05)
        return dict(self.link_kv)

    def assert_wire_armed(self, where: str) -> None:
        kv = self.refresh_link()
        ok, message = tcg.wire_armed_verdict(kv)
        if not ok:
            raise TossCalGridFaultError('{}: {}'.format(where, message))

    def assert_no_drive_fault(self, where: str) -> None:
        if self.motor_error_edge:
            legs = ', '.join(
                'leg {} {}'.format(leg, tcg.decode_error_bits(
                    edge.get('active_errors', 0) | edge.get('disarm_reason', 0)))
                for leg, edge in sorted(self.motor_error_edge.items()))
            raise TossCalGridFaultError(
                '{}: a leg ODrive latched an error during this capture ({}). '
                'Capture forensics BEFORE any relaunch — the next launch\'s BOOT '
                'pre-flight auto-clears the drive-side error record.'
                .format(where, legs))

    def wait_for_status(self, timeout_s: float = 5.0, predicate=None,
                        expected: str = ''):
        end = time.time() + max(0.0, timeout_s)
        while time.time() < end:
            self.spin(0.05)
            if self.status is not None and (predicate is None
                                            or predicate(self.status)):
                return self.status
        raise TossCalGridError(
            'no /trajectory/status{} within {:.1f} s — is trajectory_node up?'
            .format(' matching {}'.format(expected) if expected else '',
                    timeout_s))

    # -- the one actuating primitive --------------------------------------

    def send_session(self, goal: GoalSpec, *, dwell_s: float,
                     throw_delay_s: float, catch_vel_scale: float,
                     max_reloads: int, timeout_s: float,
                     drain_fn=None) -> Any:
        """Send ONE ``TossContinuous`` goal and wait for its result.

        ``drain_fn`` is called on every spin iteration so a per-toss row reaches
        the CSV **as that toss completes**, not when the goal does: a goal that
        aborts on toss 7 of 8 must still leave seven rows on disk.

        The first Ctrl-C cancels and prints whether the cancel is honoured now or
        DEFERRED because a ball is airborne; a second one is refused with the
        same message rather than being allowed to skip the wait.
        """
        import rclpy
        from geometry_msgs.msg import Point

        if not self.action.wait_for_server(timeout_sec=self.args.timeout_s):
            raise TossCalGridError(
                '{} action server unavailable after {:.1f} s. `ros2 action list` '
                'showing none of /jugglebot/{{reload,toss,toss_continuous}} means '
                'reload_coordinator_node raised ImportError on a stale '
                'jugglebot_interfaces — rebuild BOTH packages.'
                .format(ACTION_NAME, self.args.timeout_s))

        request = self._TossContinuous.Goal()
        request.catch_position = Point(x=float(goal.x_mm), y=float(goal.y_mm),
                                       z=float(goal.z_mm))
        request.throw_height_m = float(goal.throw_height_m)
        request.num_throws = int(goal.n_throws)
        request.dwell_time_s = float(dwell_s)
        request.throw_delay_s = float(throw_delay_s)
        request.catch_vel_scale = float(catch_vel_scale)
        # stop_on_miss FALSE + on_empty_cup RELOAD is the ONLY combination in
        # which the auto-reload interlude is reachable (§ 3.9): with
        # stop_on_miss true the MISSED cycle that dropped the ball has already
        # stopped the session one cycle earlier. A capture that stopped on every
        # miss would spend a sitting on re-sending goals.
        request.stop_on_miss = False
        request.on_empty_cup = 'RELOAD'
        request.max_reloads = int(max_reloads)

        phase = {'value': 'SESSION_CHECKING'}

        def _feedback(msg):
            try:
                phase['value'] = str(msg.feedback.phase)
            except AttributeError:
                pass

        send_future = self.action.send_goal_async(request,
                                                  feedback_callback=_feedback)
        rclpy.spin_until_future_complete(self.node, send_future,
                                         timeout_sec=self.args.timeout_s)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise TossCalGridError(
                '{} REJECTED the goal at accept for {}. A REJECTED_BUSY here '
                'means another ball op holds the one-ball-op claim; every other '
                'code names the goal field it refused.'
                .format(ACTION_NAME, goal.label))

        result_future = handle.get_result_async()
        deadline = time.time() + timeout_s
        cancelled = False
        while not result_future.done():
            if time.time() > deadline:
                if not cancelled:
                    print('  !! goal {} exceeded its {:.0f} s ceiling — '
                          'cancelling'.format(goal.label, timeout_s))
                    handle.cancel_goal_async()
                    cancelled = True
                    deadline = time.time() + 30.0
                    continue
                raise TossCalGridFaultError(
                    'goal {} did not terminate 30 s after a cancel — the '
                    'coordinator is not answering. STOP and inspect before '
                    'sending anything else.'.format(goal.label))
            try:
                rclpy.spin_once(self.node, timeout_sec=0.05)
                if drain_fn is not None:
                    drain_fn()
            except KeyboardInterrupt:
                if cancelled:
                    print('  (already cancelling — a second Ctrl-C cannot skip '
                          'this wait)')
                    continue
                deferred, text = cancel_disposition(phase['value'])
                print('\n{}'.format(text))
                handle.cancel_goal_async()
                cancelled = True
                deadline = time.time() + (30.0 if deferred else 15.0)
        wrapped = result_future.result()
        result = getattr(wrapped, 'result', wrapped)
        if cancelled:
            raise KeyboardInterrupt(
                'operator cancelled during {}'.format(goal.label))
        return result

    # -- safing ------------------------------------------------------------

    def return_to_centre(self, timeout_s: float = 8.0) -> None:
        """Re-centre via ``trajectory/go_home`` and CONFIRM arrival.

        ``go_home`` returns on the service ACK at plan-INSTALL, not on arrival —
        the same trap ``_recentre_and_verify`` documents — so this waits the
        profile out and then checks a FRESH ``/trajectory/commanded_position``.
        A CAUGHT cycle ends in ACTION_STAY, so after a corner node the routine
        state is "parked 150 mm off centre with a raised platform"; leaving it
        there is what this exists to prevent.
        """
        response = self.call(self.cli_go_home, self._Trigger.Request(),
                             'trajectory/go_home')
        if not getattr(response, 'success', False):
            raise TossCalGridError(
                'trajectory/go_home refused: {}'.format(
                    getattr(response, 'message', '')))
        self.commanded_stamp = 0.0
        end = time.time() + timeout_s
        while time.time() < end:
            self.spin(0.1)
            if self.commanded_stamp and self.commanded_xyz is not None:
                x, y, _ = self.commanded_xyz
                if math.hypot(x, y) <= 5.0:
                    return
        raise TossCalGridError(
            'go_home was accepted but /trajectory/commanded_position never '
            'reported the platform inside 5 mm of centre within {:.1f} s'
            .format(timeout_s))


# ── probe-map install / restore ──────────────────────────────────────────


class _ProbeMapSession:
    """Install probe maps at the production write target and ALWAYS restore.

    The snapshot is taken once, before the first probe map is written, and the
    restore runs from the ``finally`` of :func:`run` — including on a second
    Ctrl-C. A ±0.5° probe map left installed aims every later throw 27 mm off
    while every log line reports a calibration as applied; that is the
    silent-wrong class, and it is worth the bookkeeping.
    """

    def __init__(self, runner, target: str, stamp: str):
        self.runner = runner
        self.target = target
        self.stamp = stamp
        self.backup: Optional[str] = None
        self.existed = os.path.exists(target)
        self.armed = False
        self.installed_version: Optional[str] = None

    def snapshot(self) -> None:
        if self.armed:
            return
        if self.existed:
            self.backup = '{}.{}.probe-bak'.format(self.target, self.stamp)
            shutil.copy2(self.target, self.backup)
            print('  probe-map snapshot: {} -> {}'.format(self.target, self.backup))
        else:
            print('  probe-map snapshot: no map exists at {} (restore = delete)'
                  .format(self.target))
        self.armed = True

    def install(self, doc: Dict[str, Any]) -> str:
        self.snapshot()
        version = toss_cal.map_version(doc)
        with open(self.target, 'w') as handle:
            handle.write(fit.dump_map_yaml(doc))
        response = self.runner.call(self.runner.cli_reload,
                                    self.runner._Trigger.Request(),
                                    'toss/reload_calibration')
        if not getattr(response, 'success', False):
            raise TossCalGridError(
                'toss/reload_calibration REJECTED the probe map this tool just '
                'wrote: {}'.format(getattr(response, 'message', '')))
        end = time.time() + 5.0
        while time.time() < end:
            self.runner.spin(0.1)
            if self.runner.cal_status.get('toss_cal_version') == version:
                self.installed_version = version
                applied = self.runner.cal_status.get('toss_cal_applied')
                if not applied:
                    raise TossCalGridError(
                        'the probe map loaded but is DORMANT ({}). A dormant map '
                        'aims vertically, so every SC-0 arm would command the '
                        'same zero aim and the probe would measure nothing.'
                        .format(self.runner.cal_status.get('dormant_reason')))
                return version
        raise TossCalGridError(
            'reload reported success but /toss/calibration_status never '
            'reported version {!r} (saw {!r}). The node resolved a different '
            'candidate path than this tool wrote to — check ${} on both sides '
            'and whether an ament share copy is shadowing the source tree. '
            'Candidates from here: {}.'
            .format(version, self.runner.cal_status.get('toss_cal_version'),
                    toss_cal.TOSS_CAL_ENV,
                    '; '.join(toss_cal.toss_cal_candidates()) or '<none>'))

    def restore(self) -> None:
        if not self.armed:
            return
        try:
            if self.backup is not None:
                shutil.copy2(self.backup, self.target)
                os.remove(self.backup)
                print('  probe map RESTORED: {} put back'.format(self.target))
            elif os.path.exists(self.target):
                os.remove(self.target)
                print('  probe map REMOVED: {} deleted (none existed before)'
                      .format(self.target))
            self.runner.call(self.runner.cli_reload,
                             self.runner._Trigger.Request(),
                             'toss/reload_calibration')
        except BaseException as exc:                             # noqa: BLE001
            print('  !! PROBE MAP RESTORE FAILED: {}: {}\n'
                  '     A PROBE map may still be installed at {} — it is NOT a '
                  'calibration. Restore it by hand ({}), then call '
                  '  ros2 service call /toss/reload_calibration '
                  'std_srvs/srv/Trigger  before any further throwing.'
                  .format(type(exc).__name__, exc, self.target,
                          'git checkout config/toss_calibration.yaml'
                          if self.backup is None else
                          'cp {} {}'.format(self.backup, self.target)),
                  file=sys.stderr)
            raise
        finally:
            self.armed = False


def print_forensics(fx: Dict[str, Any]) -> None:
    print('\n-- forensics --', file=sys.stderr)
    for key in ('wall_iso', 'uptime_ms_last', 'link_age_s', 'status_age_s',
                'records_seen'):
        print('  {}: {}'.format(key, fx.get(key)), file=sys.stderr)
    print('  link_kv: {}'.format(fx.get('link_kv')), file=sys.stderr)
    print('  last_traj_status: {}'.format(fx.get('last_traj_status')),
          file=sys.stderr)
    print('  toss_cal_status: {}'.format(fx.get('toss_cal_status')),
          file=sys.stderr)
    print('  hand_sensor: {}'.format(fx.get('hand_sensor')), file=sys.stderr)
    edges = fx.get('motor_error_edges') or {}
    if edges:
        print('  ODRIVE ERROR EDGES (first nonzero per leg):', file=sys.stderr)
        for leg, edge in sorted(edges.items()):
            print('    leg {}: active={} disarm={} at {}'
                  .format(leg, edge.get('active_errors_decoded'),
                          edge.get('disarm_reason_decoded'), edge.get('iso')),
                  file=sys.stderr)
    else:
        print('  no ODrive error edges latched', file=sys.stderr)


def run(args) -> int:                                    # noqa: C901
    """The ROS half. Imports rclpy lazily so the pure core stays importable."""
    import rclpy

    out_dir = args.out_dir or os.path.join(_REPO, 'temp', 'logs')
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(out_dir, 'toss_cal_grid_{}_{}.csv'
                            .format(args.rung, stamp))
    meta_path = os.path.join(out_dir, 'toss_cal_grid_{}_{}_meta.json'
                             .format(args.rung, stamp))

    x_mm, y_mm = resolve_grid(args)
    z_mm = float(args.z)
    goals = rung_goals(args.rung, x_mm, y_mm, z_mm, args)

    ledger = load_ledger(ledger_path(out_dir))
    ok, why = rung_precondition(ledger, args.rung, allow_5x5=args.allow_5x5,
                                nodes=len(x_mm))
    if not ok:
        raise TossCalGridError('rung {} REFUSED: {}'.format(args.rung.upper(), why))

    rclpy.init()
    try:
        node = rclpy.create_node('toss_cal_grid')
        runner = _Runner(node, args)
    except ImportError as exc:
        rclpy.shutdown()
        raise TossCalGridError(
            'interface import failed ({}). Did you run  colcon build '
            '--packages-select jugglebot_interfaces jugglebot  and  source '
            'ros_ws/install/setup.bash ?'.format(exc))
    except BaseException:
        rclpy.shutdown()
        raise

    started = time.time()
    started_mono = time.monotonic()
    rc = 0
    abort_reason: Optional[str] = None
    goal_summaries: List[Dict[str, Any]] = []
    exhausted_nodes: List[Dict[str, Any]] = []
    reached_motion = False
    probe = None
    current_goal: Optional[GoalSpec] = None
    base_condition = args.base_condition

    try:
        csv_file = open(csv_path, 'w', newline='')
    except BaseException:
        node.destroy_node()
        rclpy.shutdown()
        raise
    writer = csv.DictWriter(csv_file, fieldnames=list(CSV_COLUMNS))
    writer.writeheader()

    def drain_records(goal: GoalSpec, note: str = '') -> int:
        """Append every record seen since the last drain — AS THEY COMPLETE."""
        n = 0
        while runner.records:
            record = runner.records.pop(0)
            writer.writerow(csv_row(
                datetime.now().isoformat(timespec='milliseconds'),
                time.monotonic() - started_mono, goal, record,
                runner.uptime_last, note))
            n += 1
        csv_file.flush()
        return n

    try:
        # ── preflight: ALL NINE REFUSALS, before anything moves ──────
        print('== preflight (rung {}) =='.format(args.rung.upper()))
        runner.wait_for_status()
        runner.spin(1.5)          # let /link_status, /toss/calibration_status land

        status = runner.status
        if status.mode != 'TRAJECTORY' or not status.streaming:
            raise TossCalGridError(
                'platform is not armed into the streaming hold: '
                'trajectory/status mode={!r} streaming={}. This tool never arms '
                'and never changes mode — bring the machine to ACTIVE:TRAJECTORY '
                'with the 40 Hz hold stream first.'
                .format(status.mode, status.streaming))
        print('  mode=TRAJECTORY streaming=True')

        # R5's edge needs the operator to move the ball; ask BEFORE the table so
        # the whole preflight can be evaluated in one pass.
        if not (runner.sensor_valid_seen and runner.sensor_edge_seen):
            print('  waiting up to {:.0f} s for a live hand-sensor held->empty '
                  'edge — LIFT THE BALL OUT OF THE CUP AND REPLACE IT NOW '
                  '(a static read cannot see a stuck bit)'
                  .format(args.sensor_probe_timeout_s))
            end = time.time() + float(args.sensor_probe_timeout_s)
            while time.time() < end and not runner.sensor_edge_seen:
                runner.spin(0.2)

        writes_a_map = any(g.probe_aim_rad is not None for g in goals)
        target_ok, target_path, target_reason = write_target_verdict()
        if not writes_a_map:
            # R8 still RESOLVES the target (so a broken deployment is named
            # before the tosses, not after), but a rung that writes nothing is
            # not refused for a target it will never touch.
            target_ok = True
            target_reason = None
        if not args.yes:
            thc.safety_gate(
                'LIVE CAPTURE about to send {} tosses over {} goal(s).\n'
                '  Floor clear of balls?  Ball supply staged in BallButler '
                '(operator-managed — NO machine fence exists)?  E-STOP in '
                'reach?  Nothing under the platform?'
                .format(toss_count(goals), len(goals)), 'yes')
            if base_condition is None:
                print('\nDescribe the base condition for the record (e.g. '
                      '"flat floor, fresh bridge, BB magazine 12"):')
                base_condition = input('> ').strip()
        else:
            print('  ! --yes: SKIPPING the operator confirmations on a LIVE '
                  'capture. Floor state, ball supply and E-STOP reach have NO '
                  'machine check whatsoever — --yes is a rehearsal affordance.')
        if not base_condition:
            base_condition = 'unspecified'

        obs = {
            'link_kv': runner.link_kv,
            'gravity_correction_loaded': bool(status.gravity_correction_loaded),
            'tilt_map_loaded': bool(status.tilt_map_loaded),
            'tilt_map_version': str(status.tilt_map_version or ''),
            'toss_trim_enabled': _param_true(runner, node,
                                             'toss_trim_enabled'),
            'sensor_valid_seen': runner.sensor_valid_seen,
            'sensor_edge_seen': runner.sensor_edge_seen,
            'toss_tier': str(hw.JB_OP_TOSS_TIER),
            'uptime_ms': runner.uptime_last,
            'write_target': target_path,
            'write_target_ok': target_ok,
            'write_target_reason': target_reason,
            # safety_gate sys.exit()s on a decline, so reaching here means the
            # gate was taken (or --yes waived it). R9 is still a row of the pure
            # table so the refusal is unit-testable in both states.
            'operator_confirmed': True,
            'operator_yes': bool(args.yes),
        }
        refusals = preflight_refusals(obs)
        for row in refusals:
            print('  {}'.format(row.message))
        failed = [row for row in refusals if not row.ok]
        if failed:
            raise TossCalGridError(
                '{} preflight refusal(s): {}. The FIRST is:\n  {}'
                .format(len(failed), ', '.join(r.rid for r in failed),
                        failed[0].message))

        if args.force_uninstall:
            to_move, env_override = uninstall_plan()
            if env_override:
                raise TossCalGridError(
                    '--force-uninstall refuses to touch an env-pointed map. ${} '
                    'is set to {}, which makes it the ONLY candidate and names a '
                    'file you chose. Unset it, or move that file aside yourself.'
                    .format(toss_cal.TOSS_CAL_ENV, env_override))
            for path in to_move:
                backup = '{}.{}.bak'.format(path, stamp)
                os.rename(path, backup)
                print('  moved {} -> {}'.format(path, backup))
            if not to_move:
                print('  no candidate file exists to move aside (tried: {})'
                      .format('; '.join(toss_cal.toss_cal_candidates()) or '<none>'))
            response = runner.call(runner.cli_reload, runner._Trigger.Request(),
                                   'toss/reload_calibration')
            print('  reload: success={} :: {}'.format(
                getattr(response, 'success', None),
                getattr(response, 'message', '')))

        print('  toss/calibration_status: {}'.format(
            json.dumps(runner.cal_status, sort_keys=True)))

        if args.rung == 'sc1':
            aim_at_home = None
            applied = bool(runner.cal_status.get('toss_cal_applied'))
            if applied:
                path = toss_cal.resolve_toss_cal_path()
                if path is not None:
                    try:
                        aim_at_home = toss_cal.lookup(
                            toss_cal.load_toss_cal(path), 0.0, 0.0)
                    except toss_cal.TossCalError:
                        aim_at_home = None
            ok1, message1 = sc1_baseline_verdict(applied, aim_at_home)
            print('  {}'.format(message1))
            if not ok1:
                raise TossCalGridError(message1)

        # ── the capture ──────────────────────────────────────────────
        goal_timeout = float(args.goal_timeout_s) or max(
            60.0, 3.0 * (args.throw_delay_s
                         + max(g.n_throws for g in goals) * args.dwell_s
                         + args.max_reloads * RELOAD_COST_S))
        print('\n== capture: {} goal(s), {} tosses, per-goal ceiling {:.0f} s =='
              .format(len(goals), toss_count(goals), goal_timeout))
        reached_motion = True
        if any(g.probe_aim_rad is not None for g in goals):
            probe = _ProbeMapSession(runner, target_path, stamp)

        for goal in goals:
            current_goal = goal
            # DELIBERATELY OUTSIDE the per-goal try below. That handler demotes a
            # TossCalGridError to a failed goal and, under the default --on-fail
            # continue, carries on. A wire that disarmed mid-capture is never
            # "this one node failed" — every remaining goal would be accepted,
            # nothing would move, and the tool would collect a plausible corpus
            # of tosses that never happened.
            runner.assert_wire_armed(goal.label)
            runner.assert_no_drive_fault(goal.label)
            try:
                if goal.probe_aim_rad is not None:
                    doc = probe_map_document(
                        x_mm, y_mm, z_mm, goal.probe_aim_rad,
                        tilt_map_version=obs['tilt_map_version'],
                        base_condition=base_condition,
                        flight_time_s=math.sqrt(8.0 * goal.throw_height_m / 9.80665))
                    version = probe.install(doc)
                    print('  probe map installed and CONFIRMED: {} aim '
                          '({:+.3f}, {:+.3f}) deg'
                          .format(version, *goal.probe_deg))

                print('\n  -> {} : {} toss(es) at ({:+.1f}, {:+.1f}, {:.0f}) '
                      'h={:.2f} m'.format(goal.label, goal.n_throws, goal.x_mm,
                                          goal.y_mm, goal.z_mm,
                                          goal.throw_height_m))
                result = runner.send_session(
                    goal, dwell_s=args.dwell_s,
                    throw_delay_s=args.throw_delay_s,
                    catch_vel_scale=args.catch_vel_scale,
                    max_reloads=args.max_reloads, timeout_s=goal_timeout,
                    drain_fn=lambda g=goal: drain_records(g))
            except TossCalGridFaultError:
                drain_records(goal, note='fault')
                raise
            except TossCalGridError as exc:
                print('  [FAIL] {} :: {}'.format(goal.label, exc))
                drain_records(goal, note=str(exc)[:200])
                goal_summaries.append({'label': goal.label, 'verdict': 'FAIL',
                                       'error': str(exc)[:200]})
                if args.on_fail == 'abort':
                    raise
                rc = 1
                continue

            n_rows = drain_records(goal)
            outcome = str(getattr(result, 'outcome', ''))
            summary = {
                'label': goal.label, 'rung': goal.rung, 'index': goal.index,
                'x_mm': goal.x_mm, 'y_mm': goal.y_mm, 'z_mm': goal.z_mm,
                'throw_height_m': goal.throw_height_m,
                'probe_aim_deg': (None if goal.probe_aim_rad is None
                                  else list(goal.probe_deg)),
                'requested_throws': goal.n_throws,
                'outcome': outcome,
                'throws_completed': int(getattr(result, 'throws_completed', 0)),
                'catches_confirmed': int(getattr(result, 'catches_confirmed', 0)),
                'reloads_used': int(getattr(result, 'reloads_used', 0)),
                'per_cycle_outcomes': list(getattr(result, 'per_cycle_outcomes', [])),
                'records_written': n_rows,
                'uptime_ms': runner.uptime_last,
            }
            # Appended per goal, not in a trailing loop: on ANY later abort a
            # trailing loop never runs and _meta.json would ship an empty list,
            # exactly when the operator most needs the per-goal census.
            goal_summaries.append(summary)
            print('     {} : {}/{} threw, {} caught, {} reload(s), {} record(s)'
                  .format(outcome, summary['throws_completed'], goal.n_throws,
                          summary['catches_confirmed'], summary['reloads_used'],
                          n_rows))

            if node_exhausted(outcome):
                # OPERATOR DECISION 4 / D19. Not an abort: § 3.7 item 7 writes a
                # thin node correctly (previous value, stale: true), so an
                # exhausted node costs one node's refresh, not a sitting.
                exhausted_nodes.append(summary)
                summary['node_state'] = 'thin/stale (reload budget exhausted)'
                print('     NODE EXHAUSTED — the reload budget ({}) is spent at '
                      'this node. Marking it THIN/STALE and SKIPPING to the '
                      'next; the capture continues. Restock BallButler when '
                      'convenient.'.format(args.max_reloads))
                continue

            if outcome not in ('COMPLETED', 'STOPPED_ON_MISS'):
                message = ('goal {} ended {} — that is a machine fault, and '
                           'repeating a fault over every remaining node is how '
                           'one fault becomes N'.format(goal.label, outcome))
                if args.on_fail == 'abort':
                    raise TossCalGridError(message)
                print('  !! {}'.format(message))
                rc = 1

    except TossCalGridError as exc:
        abort_reason = str(exc)
        print('\nABORT: {}'.format(exc), file=sys.stderr)
        try:
            print_forensics(runner.forensics())
        except Exception as fx_exc:                       # noqa: BLE001
            print('  (forensics dump itself failed: {})'.format(fx_exc),
                  file=sys.stderr)
        rc = 2
    except KeyboardInterrupt as exc:
        abort_reason = 'interrupted by operator ({})'.format(exc)
        print('\nABORT: interrupted by operator.', file=sys.stderr)
        try:
            print_forensics(runner.forensics())
        except Exception as fx_exc:                       # noqa: BLE001
            print('  (forensics dump itself failed: {})'.format(fx_exc),
                  file=sys.stderr)
        rc = 130
    except SystemExit as exc:
        # thc.safety_gate declines via sys.exit(1). Without this handler the meta
        # would record exit_code 0 / abort_reason None while the process exits 1,
        # recreating the "forensically indistinguishable attempt" class.
        abort_reason = 'operator declined a safety gate (SystemExit {})'.format(
            exc.code)
        rc = exc.code if isinstance(exc.code, int) else 2
        raise
    except BaseException as exc:                          # noqa: BLE001
        abort_reason = 'unexpected {}: {}'.format(type(exc).__name__, exc)
        rc = rc or 3
        raise
    finally:
        # `except BaseException`, deliberately: a plain `except Exception` does
        # NOT catch KeyboardInterrupt, and the second Ctrl-C is the reflex when a
        # program does not die on the first. Both the probe-map restore and the
        # return to centre block for seconds, so that second interrupt lands
        # squarely inside this block; with `Exception` it would propagate out of
        # the `finally`, skip the artefact write AND the shutdown, and leave the
        # platform parked at a raised displaced pose under a PROBE map, in
        # silence. Failing loudly is the whole contract of this block.
        if probe is not None:
            try:
                probe.restore()
            except BaseException as exc:                 # noqa: BLE001
                abort_reason = abort_reason or 'probe-map restore failed: {}'.format(exc)
                rc = rc or 2
        if reached_motion:
            try:
                print('\nreturning to centre (0, 0, {:.1f})...'.format(z_mm))
                runner.return_to_centre()
                print('  at centre.')
            except BaseException as exc:                 # noqa: BLE001
                print('  !! RETURN TO CENTRE FAILED: {}: {}\n'
                      '     The platform may be parked at a displaced pose — '
                      'bring it home manually before deactivating.'
                      .format(type(exc).__name__, exc), file=sys.stderr)
                rc = rc or 2
        try:
            # Anything the coordinator declared between the last drain and the
            # abort. On a Ctrl-C mid-goal those are the rows closest to the
            # event, which is exactly when they matter most.
            try:
                if current_goal is not None:
                    drain_records(current_goal, note='drained at exit')
            except Exception:                             # noqa: BLE001
                pass
            try:
                _final_forensics = runner.forensics()
            except Exception:                             # noqa: BLE001
                _final_forensics = None
            ledger = load_ledger(ledger_path(out_dir))
            record_rung(
                ledger, args.rung, VERDICT_UNSCORED,
                headline='captured {} goal(s); score it with  --rung {} --score '
                         '<mined corpus>'.format(len(goal_summaries), args.rung),
                extra={'csv': csv_path, 'meta': meta_path,
                       'exhausted_nodes': [n['label'] for n in exhausted_nodes],
                       'abort_reason': abort_reason})
            save_ledger(ledger_path(out_dir), ledger)

            meta = {
                'tool': TOOL_NAME,
                'rung': args.rung,
                'git_sha': tcg.git_sha(),
                'argv': sys.argv,
                'started_iso': datetime.fromtimestamp(started).isoformat(),
                'ended_iso': datetime.now().isoformat(),
                'duration_s': round(time.time() - started, 1),
                'csv': csv_path,
                'base_condition': base_condition,
                'grid': {'x_mm': list(x_mm), 'y_mm': list(y_mm), 'z_mm': z_mm},
                'uptime_ms_first': runner.uptime_first,
                'uptime_ms_last': runner.uptime_last,
                'toss_cal_status': dict(runner.cal_status),
                'goals': goal_summaries,
                'exhausted_nodes': exhausted_nodes,
                'settings': {
                    'n_per_node': args.n_per_node, 'sc0_n': args.sc0_n,
                    'sc3_n': args.sc3_n, 'sc0_probe_deg': args.sc0_probe_deg,
                    'dwell_s': args.dwell_s, 'throw_delay_s': args.throw_delay_s,
                    'max_reloads': args.max_reloads,
                    'throw_height_m': args.throw_height_m,
                },
                # ALWAYS present (None on a clean run): an abort that writes an
                # empty summary is the abort most in need of the data.
                'abort_reason': abort_reason,
                'forensics': _final_forensics,
                'exit_code': rc,
            }
            with open(meta_path, 'w') as handle:
                json.dump(meta, handle, indent=2, default=str)
            csv_file.close()
            print('\nCSV    -> {}'.format(csv_path))
            print('meta   -> {}'.format(meta_path))
            print('ledger -> {}'.format(ledger_path(out_dir)))
            print('\nNEXT: mine the bag, then score this rung:')
            print('  python tools/probes/toss_record_miner.py --bag <bag> --jsonl')
            print('  python3 {} --rung {} --score <corpus.jsonl>'
                  .format(TOOL_NAME, args.rung))
        finally:
            node.destroy_node()
            rclpy.shutdown()
    return rc


def _param_true(runner, node, name: str) -> bool:
    """Read a remote parameter on ``reload_coordinator_node``, fail-closed.

    An unreadable parameter is treated as ``True`` for R4 — "I could not check
    whether the session trim is on" is not "the session trim is off", and a trim
    running under a capture contaminates the persistent map with a RAM-only
    estimate nobody reviewed.
    """
    import rclpy
    from rcl_interfaces.srv import GetParameters
    client = node.create_client(GetParameters,
                                '/reload_coordinator_node/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=runner.args.timeout_s):
            print('  ! could not reach /reload_coordinator_node/get_parameters '
                  '— R4 fails CLOSED')
            return True
        request = GetParameters.Request()
        request.names = [name]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future,
                                         timeout_sec=runner.args.timeout_s)
        response = future.result()
        if response is None or not response.values:
            return True
        return bool(response.values[0].bool_value)
    finally:
        node.destroy_client(client)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.verify_only:
        args.rung = 'sc3'
    try:
        if args.score:
            if args.dry_run:
                print('--score is already desk-side and makes no ROS calls; '
                      '--dry-run adds nothing.')
            return run_score(args)
        if args.no_apply and args.rung == 'sc0':
            print('ABORT: --no-apply refuses --rung sc0. SC-0 commands its aims '
                  'through PROBE MAPS written to the production write target — '
                  'there is no other aim authority — so a run that writes '
                  'nothing measures nothing. The probe maps are restored on '
                  'every exit path.', file=sys.stderr)
            return 2
        x_mm, y_mm = resolve_grid(args)
        goals = rung_goals(args.rung, x_mm, y_mm, float(args.z), args)
        if args.dry_run:
            print_plan(args.rung, x_mm, y_mm, float(args.z), goals, args)
            # The rung precondition is a FILE read, not a ROS call, so the
            # rehearsal can answer "would this rung even be allowed to run?"
            # before the operator walks to the robot.
            out_dir = args.out_dir or os.path.join(_REPO, 'temp', 'logs')
            allowed, why = rung_precondition(
                load_ledger(ledger_path(out_dir)), args.rung,
                allow_5x5=args.allow_5x5, nodes=len(x_mm))
            if not allowed:
                print('\n  !! rung {} WOULD BE REFUSED: {}'
                      .format(args.rung.upper(), why))
                print('\ndry-run: no ROS calls made, no ROS objects constructed.')
                return 2
            print('\n  rung precondition OK (ledger {})'
                  .format(ledger_path(out_dir)))
            problems = tcg.stroke_margin_problems(
                [(g.x_mm, g.y_mm) for g in goals] + [(0.0, 0.0)], float(args.z))
            if problems:
                print('\n  !! IK stroke preflight WOULD REFUSE {} pose(s):\n    {}'
                      .format(len(problems), '\n    '.join(problems)))
                print('\ndry-run: no ROS calls made, no ROS objects constructed.')
                return 2
            print('\n  IK stroke preflight OK: every commanded pose keeps >= '
                  '{:.0f} mm of leg-extension margin'
                  .format(tcg.STROKE_PREFLIGHT_MARGIN_MM))
            print('\ndry-run: no ROS calls made, no ROS objects constructed.')
            return 0
        problems = tcg.stroke_margin_problems(
            [(g.x_mm, g.y_mm) for g in goals] + [(0.0, 0.0)], float(args.z))
        if problems:
            raise TossCalGridError(
                'IK stroke-margin preflight REFUSED {} pose(s) — worst leg '
                'within {:.0f} mm of the [5, 275] mm hard extension bound:\n  {}'
                '\nShrink --box / adjust --x/--y/--z. No goal was sent.'
                .format(len(problems), tcg.STROKE_PREFLIGHT_MARGIN_MM,
                        '\n  '.join(problems)))
        return run(args)
    except (TossCalGridError, fit.TossFitError, toss_cal.TossCalError) as exc:
        print('ABORT: {}'.format(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
