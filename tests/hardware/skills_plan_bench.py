#!/usr/bin/env python3
"""Bench driver: skill-stack R2's outstanding hardware gate (plan § 4 R2).

Measures per-skill PLAN TIME of ``trajectory/install_segment``, in the
LAUNCHED ``trajectory_node`` on the loaded Jetson (launch up, bag recording,
GUI up), over the real columns schedule at the operating point (apex 0.9 m,
separation 100 mm, dwell 0.30 s, session limits 300/5000/200000, hand 3500).

**THE ROBOT NEVER MOVES.** The launch runs with ``auto_arm:=false``; the
operator ACTIVATEs (legs + hand energised, holding) and never arms, so every
setpoint is dropped at the bridge (``trajectory_node._wire_state_suffix`` —
the ``[wire DISARMED ...]`` marker on an accepted response). The plan is
solved from the COMMANDED state, never the measured one (same argument as
``PlanCycle.srv``'s header), so a disarmed solve is the same solve an armed
one would be.

  python3 tests/hardware/skills_plan_bench.py --dry-run
  python3 tests/hardware/skills_plan_bench.py --rehearse --arm B --attempts 1 --n-throws 6
  python3 tests/hardware/skills_plan_bench.py --check
  python3 tests/hardware/skills_plan_bench.py --arm A --attempts 3 --label loaded

Runs under the SYSTEM python3 with ROS 2 sourced for a live run (like every
other rclpy tool in this directory) and under the project VENV for
``--rehearse``/``--dry-run``/``--check`` (those need ``numpy`` and the
``motion`` package but no ROS). The pure core (schedule/verdict/precondition
arithmetic, the request-field mirror, the synthetic tracker) imports no
``rclpy`` and no ``jugglebot_interfaces`` at module scope — only the pure
``jugglebot.motion.*`` / ``jugglebot.hardware_config`` packages, which are
plain Python source and need no colcon build. ``rclpy`` and
``jugglebot_interfaces`` are imported inside :func:`run` only, pinned by
``tests/ros/test_skills_plan_bench.py::test_the_pure_core_imports_no_ros_at_module_scope``.

ARM A vs ARM B
---------------
Both arms fly the SAME schedule (columns, apex 0.9 m, 100 mm separation,
0.30 s dwell) through the SAME install path. The synthetic tracker is the only
difference:

* **Arm A** — the tracker returns the SCHEDULE's own nominal landing exactly
  (zero jitter, forced regardless of ``--jitter-mm``). No catch is ever
  re-aimed, so this arm measures the plan-time floor: every skill installs
  exactly once.
* **Arm B** — the tracker adds a deterministic, per-attempt-seeded uniform
  jitter (default +/-3 mm, re-drawn every 0.1 s) to force catch RE-SENDS.
  Purpose: the re-send path is a SECOND install of an already-committed catch,
  at the general (unpinned) lead rather than the release-pinned handoff lead,
  and it is the shape most likely to blow the < 50 ms Jetson budget under
  load. Provenance: rehearsal 2026-09-13, 20 throws, +/-3 mm -> 39 re-send
  solves in one attempt (31 LIMIT_ACC + 8 LIMIT_JERK refusals, all
  non-fatal — the committed catch stands and the attempt continues).

WHAT THIS IS NOT
-----------------
It never arms, never opens the UDP link, never changes control mode, never
sets limits. Same request-only posture as ``unified_cycle_bench.py``. Its one
safety behaviour of its own is a REFUSAL (the P1-P7 preconditions below) plus
a live mid-run abort (see :data:`_ABORT_*`) that calls ``trajectory/hold`` the
instant the wire looks armed or an accepted response is missing its
``[wire DISARMED`` marker.

PRECONDITIONS (live; refuse ALL at once, never stop at the first)
-------------------------------------------------------------------
  P1  /trajectory/status fresh, mode == TRAJECTORY, streaming.
  P2  /robot_state fresh and is_homed.
  P3  wire DISARMED: /link_status mpc_active == '0' (and teensy_mpc_active ==
      '0' when present) — this gate must never move the robot.
  P4  session limits read back off /trajectory/status == 300/5000/200000.
  P5  no gravity correction loaded (the skill path has no pre-level yet).
  P6  no cycle plan active (the first install starts from the ACTIVATE hold).
  P7  trajectory/install_segment available (wait_for_service 3 s).

Exit codes: 0 = every gate PASS (or --check/--dry-run clean), 1 = any gate
FAIL, 2 = REFUSED (a precondition failed; nothing was requested).
"""

from __future__ import annotations

import argparse
import dataclasses
import csv
import json
import os

# The BLAS thread pool is capped to ONE thread before anything imports numpy,
# exactly as jugglebot_launch.py caps the planner nodes (`_planner_blas_env`).
# The pool's default six busy-spinning workers stall a small-numpy-call solve:
# MEASURED 2026-09-13, this driver's --rehearse on an idle Jetson, warm start
# carried: uncapped worst install 94.9 ms (solver windows to 68.1 ms on 3-9
# iterations); capped worst 31.8 ms (windows to 6.4 ms). The mechanism is
# jugglebot.motion.blas_threads' (the UH-3 E-STOP class). setdefault, so an
# explicit environment still wins.
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')
import math
import subprocess
import sys
import tempfile
import time
from datetime import datetime
from types import SimpleNamespace

_HERE = os.path.dirname(os.path.abspath(__file__))              # tests/hardware
_REPO = os.path.dirname(os.path.dirname(_HERE))                 # repo root
for _p in (_HERE, _REPO, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_REPO, 'config', 'generated')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np                                               # noqa: E402

import jugglebot.hardware_config as hw                            # noqa: E402
from jugglebot.motion import unified_cycle as uc                  # noqa: E402
from jugglebot.motion.geometry import StewartGeometry             # noqa: E402
from jugglebot.motion.trajectory import ballistics_bc as bal       # noqa: E402
from jugglebot.motion.trajectory.cup_realize import RealizeConfig  # noqa: E402
from jugglebot.motion.trajectory.limits import TrajectoryLimits    # noqa: E402
from jugglebot.motion.skills import admissible as adm              # noqa: E402
from jugglebot.motion.skills import executor as ex                # noqa: E402
from jugglebot.motion.skills import learner as lr                  # noqa: E402
from jugglebot.motion.skills import schedule as sc                # noqa: E402
from jugglebot.motion.skills import segments as sg                # noqa: E402
from jugglebot.motion.skills import sites as si                   # noqa: E402
from jugglebot.motion.skills.memory import Memory                  # noqa: E402

TOOL_NAME = 'tests/hardware/skills_plan_bench.py'

# ─────────────────────────────────────────────────────────────────────────────
# Wire constants mirrored from InstallSegment.srv (mirrored, not imported, so
# --dry-run / --rehearse work on a box with no colcon build; pinned to the
# .srv text by tests/ros/test_skills_plan_bench.py).
# ─────────────────────────────────────────────────────────────────────────────
KIND_THROW, KIND_CATCH, KIND_REST = 0, 1, 2
_WIRE_KIND = {sg.THROW: KIND_THROW, sg.CATCH: KIND_CATCH, sg.REST: KIND_REST}

#: The owner's R2 operating point (brief_common.md § 0, 2026-09-12).
APEX_M = 0.9
SEPARATION_MM = 100.0
DWELL_S = 0.30
SESSION_LEG_VEL_MMPS = 300.0
SESSION_LEG_ACC_MMPS2 = 5000.0
SESSION_LEG_JERK_MMPS3 = 200000.0
SESSION_HAND_ACC_RPS2 = 3500.0

#: The owner's R3 single-site self-toss operating point (brief_common.md § 0
#: "Owner decisions", 2026-09-13): site P1 = the SAME (-50, 0) mm point
#: ``columns_sites(100.0)`` names, apex/dwell/hand-acc unchanged from R2 — only
#: the leg jerk ceiling moves, to 150 000 mm/s^3 (the box swept in
#: ``config/generated/admissible_box.yaml`` for site pair ('P1', 'P1') is
#: swept at this jerk; a mismatch is caught loudly by ``admissible.
#: check_limits`` rather than silently clipping against the wrong envelope).
SELF_TOSS_SITE_X_MM = -50.0
SELF_TOSS_SITE_Y_MM = 0.0
SELF_TOSS_LEG_JERK_MMPS3 = 150000.0
#: Cold-start policy A (brief_common.md § "Owner decisions"): single-throw
#: attempts until the memory holds ``learner.LearnerConfig.k_min`` (2) rows.
#: One throw per rehearsal attempt is what lets ``--attempts 3`` (the default)
#: cross that floor and exercise the learner's post-prior command by the third
#: attempt, entirely offline.
SELF_TOSS_DEFAULT_N_THROWS = 1

#: The plan's own R3 sim-validation plant-error model (plan § R3 "Sim
#: validation"): +11% launch speed, +8.5 mrad aim. Injected here
#: ANALYTICALLY (:func:`biased_landing`, no MuJoCo) so ``--rehearse`` can
#: exercise the learner's command changes across attempts without a physics
#: engine — deterministic, no RNG, so the bias is exactly repeatable and
#: orthogonal to ``--jitter-mm``'s tracker noise (still layered on top for
#: arm B, exactly as it is for columns).
SELF_TOSS_SPEED_ERR_FRAC = 0.11
SELF_TOSS_AIM_ERR_MRAD = 8.5

#: R3-g FINDING (2026-09-13, this bench, ``--rehearse --pattern self-toss``
#: through ``install_segment`` unmodified), RESOLVED: ``schedule.
#: compile_self_toss``'s own opening REST — ``FLOOR_LIFT_S``, from the
#: ACTIVATE park (0, 0) to site P1's rest position (-50, 0) mm — refused
#: ``LIMIT_JERK`` at the R3 session limit at its original 1.0 s value (peak
#: 177 241 mm/s³ against 150 000; 205 051 mm/s³ at 1.2 s, a non-monotonic
#: solver artifact; CLEAN at 1.5 s and above). The SAME move at columns' R2
#: ceiling (200 000 mm/s³) is what the R2 gate's manual REST-pre already
#: accepts — so this was a NEW, R3-specific margin gap the lower jerk
#: ceiling opened up, not a defect in this driver. **Fix landed**:
#: ``schedule.FLOOR_LIFT_S`` is now 1.5 s (Finding A in the runsheet, now
#: RESOLVED) — the real ``skills/start_self_toss`` first call is no longer
#: expected to refuse on this segment. This bench still pre-positions self-
#: toss too (its OWN manual REST-pre, at this same 1.5 s window, mirroring
#: columns) rather than relying on ``compile_self_toss``'s own opening REST
#: — see ``rehearse_attempt``'s § 1b comment for why that still matters:
#: it masks the real cold-start move and the real ``_prelevel`` step from
#: this rehearsal.
SELF_TOSS_PRELIFT_S = 1.5

DEFAULT_ATTEMPTS = 3
#: ``None`` here (never a fixed literal) because the right default is
#: PATTERN-dependent — see the post-parse fixup in :func:`main` colocating
#: both defaults (:data:`DEFAULT_N_THROWS` for columns,
#: :data:`SELF_TOSS_DEFAULT_N_THROWS` for self-toss) at one read site.
DEFAULT_N_THROWS = 20
DEFAULT_JITTER_MM = 3.0
DEFAULT_MAX_AGE_S = 2.0
DEFAULT_SETTLE_S = 3.0
_TICK_HZ = 40.0
_TICK_PERIOD_S = 1.0 / _TICK_HZ
#: skill_node's own START_LEAD_S — one second of dispatch margin, not a
#: physical constant of the pattern.
_START_LEAD_S = 1.0

#: G1's bar — plan § 4 R2: "per-skill plan < 50 ms on the loaded Jetson".
G1_BAR_MS = 50.0
#: The wire-read budget, read off schedule.py rather than restated (plan § 0:
#: "every number carries provenance" — these three ARE the provenance).
_DT = float(hw.JB_TRAJ_KNOT_DT_S)
G2_HANDOFF_BUDGET_MS = (sc.HANDOFF_LEAD_KNOTS - sc.WIRE_READ_KNOTS) * _DT * 1e3
G2_UNPINNED_BUDGET_MS = (sc.LEAD_KNOTS - sc.WIRE_READ_KNOTS) * _DT * 1e3
#: G3 — plan § 5: max_emit_gap_ms p50 ~= 26, max < 40 ms (the UH-6 numbers).
G3_MAX_GAP_MS = 40.0

#: Pre-plan refusal codes: the install never reached a QP solve, so its
#: (near-zero) plan_wall_ms says nothing about solve cost.
_PRE_PLAN_REFUSAL_CODES = frozenset(('ABORTED', 
    'WRONG_MODE', 'GUARD_LATCHED', 'SERVICE_UNAVAILABLE', 'SERVICE_TIMEOUT',
    'WINDOW_TOO_SHORT'))

CSV_COLUMNS = ('attempt', 't_rel_s', 'kind', 'ball_id', 'accepted', 'code',
              'server_plan_ms', 'client_rtt_ms', 'dispatch_late_ms',
              'is_resend', 'is_preposition', 'splice_k', 'seeded_post_release',
              'budget_class', 'disarmed_marker', 'load1', 'mpc_active',
              'max_emit_gap_ms', 'message')


class BenchError(RuntimeError):
    """A refusal this tool minted itself (never a planner refusal)."""


# ─────────────────────────────────────────────────────────────────────────────
# Pure core — the schedule and its two splice budgets
# ─────────────────────────────────────────────────────────────────────────────

def build_schedule(*, n_throws: int, apex_m: float = APEX_M,
                   separation_mm: float = SEPARATION_MM,
                   dwell_s: float = DWELL_S, t0_abs_s: float = 0.0):
    sites = si.columns_sites(separation_mm)
    pattern = sc.Pattern(sites=sites, apex_m=apex_m, dwell_s=dwell_s,
                        n_throws=n_throws)
    return sc.compile_columns(pattern, t0_abs_s)


def default_self_toss_site() -> si.Site:
    """The R3 owner-decision site P1 — the module-level default, factored so
    every caller (``--dry-run``, ``--rehearse``, tests) names one site."""
    return si.Site('P1', np.array([SELF_TOSS_SITE_X_MM, SELF_TOSS_SITE_Y_MM,
                                   si.CATCH_CUP_Z_MM]))


def build_self_toss_schedule(*, n_throws: int, site: si.Site = None,
                             apex_m: float = APEX_M, dwell_s: float = DWELL_S,
                             t0_abs_s: float = 0.0):
    """R3's single-site self-toss schedule (``schedule.compile_self_toss``) —
    the self-toss counterpart of :func:`build_schedule`."""
    site = default_self_toss_site() if site is None else site
    pattern = sc.SelfTossPattern(site=site, apex_m=apex_m, dwell_s=dwell_s,
                                 n_throws=n_throws)
    return sc.compile_self_toss(pattern, t0_abs_s)


def kind_display(skill, *, is_preposition: bool = False,
                 is_resend: bool = False) -> str:
    """The six-way bucket G1's per-kind report groups on."""
    if is_preposition:
        return 'REST-pre'
    if is_resend:
        return 're-send'
    if skill.kind == sg.THROW:
        return 'THROW'
    if skill.kind == sg.CATCH:
        return 'CATCH+throw' if skill.then_throw is not None else 'CATCH'
    return 'REST'


def install_request_fields(kind: str, terminal) -> dict:
    """``{field_name: value}`` for ``InstallSegment.Request``, EXCLUDING
    ``kind``/``ball_id`` (the caller sets those — see :meth:`_Runner.install`).

    Mirrors ``skill_node.SkillNode._installer`` field for field, so the two
    request builders can never drift: ``tests/ros/test_skills_plan_bench.py``
    checks every name here against a real ``InstallSegment.Request`` and
    against ``skill_node``'s own source.
    """
    if kind == sg.THROW:
        return {
            't_event_s': float(terminal.t_release_s),
            'site_mm': [float(v) for v in terminal.site_mm],
            'target_mm': [float(v) for v in terminal.target_mm],
            'flight_s': float(terminal.flight_s),
        }
    if kind == sg.CATCH:
        out = {
            't_event_s': float(terminal.t_land_s),
            'site_mm': [float(v) for v in terminal.landing_mm],
            'landing_vel_mm_s': [float(v) for v in terminal.landing_vel_mm_s],
            'rest_site_mm': [float(v) for v in terminal.rest_site_mm],
        }
        tt = terminal.then_throw
        if tt is not None:
            out['t_release_s'] = float(tt.t_release_s)
            out['release_site_mm'] = [float(v) for v in tt.site_mm]
            out['target_mm'] = [float(v) for v in tt.target_mm]
            out['flight_s'] = float(tt.flight_s)
        return out
    return {
        't_event_s': float(terminal.t_rest_s),
        'rest_site_mm': [float(v) for v in terminal.rest_site_mm],
    }


def budget_class(splice_k: int, seeded_post_release: bool) -> str:
    if splice_k == 0:
        return 'fresh'
    if seeded_post_release:
        return 'handoff'
    return 'unpinned'


def is_solve(row: dict) -> bool:
    """False for a refusal that never reached a QP solve (plan § 4 R2's own
    filter): the pre-plan codes, and a STALE_STATE whose plan_wall_ms reads
    under 1 ms (a guard refusal, not a timed-out solve)."""
    if row.get('code') in _PRE_PLAN_REFUSAL_CODES:
        return False
    if row.get('code') == 'STALE_STATE':
        ms = row.get('server_plan_ms')
        if ms is None or float(ms) < 1.0:
            return False
    return True


def solves(rows) -> list:
    return [r for r in rows if is_solve(r)]


def print_dry_run(n_throws: int, *, pattern: str = 'columns',
                  site: si.Site = None, apex_m: float = APEX_M) -> None:
    if pattern == 'self-toss':
        site = default_self_toss_site() if site is None else site
        sched = build_self_toss_schedule(n_throws=n_throws, site=site,
                                         apex_m=apex_m)
        print('self-toss schedule: site=%s (%.1f, %.1f) mm apex=%.2f m '
              'dwell=%.2f s n_throws=%d' % (site.name, site.cup_mm[0],
                                            site.cup_mm[1], apex_m, DWELL_S,
                                            n_throws))
    else:
        sched = build_schedule(n_throws=n_throws)
        print('columns schedule: apex=%.2f m sep=%.0f mm dwell=%.2f s n_throws=%d'
              % (APEX_M, SEPARATION_MM, DWELL_S, n_throws))
    print('flight=%.4f s beat=%.4f s transit=%.4f s (%d skills)'
          % (sched.flight_s, sched.beat_s, sched.transit_s, len(sched.skills)))
    print('\n%-4s %-12s %-4s %-6s %10s %10s %-10s'
          % ('idx', 'kind', 'ball', 'site', 't_abs_s', 'dispatch', 'lead'))
    for i, sk in enumerate(sched.skills):
        pin = 'handoff(pinned)' if sk.lead_s == sc.HANDOFF_LEAD_S else 'unpinned'
        print('%-4d %-12s %-4d %-6s %10.4f %10.4f %-10s'
              % (i, kind_display(sk), sk.ball_id, sk.site.name, sk.t_abs_s,
                 sk.dispatch_s(), pin))
    print('\nsplice budgets (LEAD_KNOTS=%d HANDOFF_LEAD_KNOTS=%d '
          'WIRE_READ_KNOTS=%d dt=%.4f s):'
          % (sc.LEAD_KNOTS, sc.HANDOFF_LEAD_KNOTS, sc.WIRE_READ_KNOTS, _DT))
    print('  unpinned (general lead): %.1f ms' % (G2_UNPINNED_BUDGET_MS,))
    print('  handoff  (release-pinned lead): %.1f ms' % (G2_HANDOFF_BUDGET_MS,))
    print('\ngate criteria:')
    print('  G1 plan     : max server_plan_ms over every solve < %.1f ms '
          '(plan § 4 R2)' % (G1_BAR_MS,))
    print('  G2 splice   : zero SPLICE_TOO_LATE; handoff RTT < %.1f ms, '
          'unpinned RTT < %.1f ms' % (G2_HANDOFF_BUDGET_MS, G2_UNPINNED_BUDGET_MS))
    print('  G3 emitter  : max_emit_gap_ms < %.1f ms (plan § 5, the UH-6 '
          'numbers; SKIP in --rehearse)' % (G3_MAX_GAP_MS,))
    print('  G4 completion: no attempt ends early; every scheduled skill '
          'accepted (re-sends may refuse)')
    print('  G5 no motion: every accepted response carries [wire DISARMED; '
          'mpc_active never 1 (SKIP in --rehearse)')


# ─────────────────────────────────────────────────────────────────────────────
# Pure core — preconditions (live only; evaluated against a Snapshot)
# ─────────────────────────────────────────────────────────────────────────────

SET_LIMITS_CMD = (
    'ros2 service call /trajectory/set_limits '
    'jugglebot_interfaces/srv/SetTrajectoryLimits '
    '"{leg_vel_limit_mmps: %.1f, leg_acc_limit_mmps2: %.1f, '
    'leg_jerk_limit_mmps3: %.1f}"'
    % (SESSION_LEG_VEL_MMPS, SESSION_LEG_ACC_MMPS2, SESSION_LEG_JERK_MMPS3))

ACTIVATE_CMD = ("ros2 topic pub -t 3 -r 2 /orchestrator_command "
               "std_msgs/msg/String \"data: 'trajectory'\" (after Home + "
               "ACTIVATE)")


class Snapshot:
    """What the ROS half saw, flattened so the evaluator needs no ROS types."""

    def __init__(self, *, status_age_s=None, mode=None, streaming=None,
                 leg_vel=None, leg_acc=None, leg_jerk=None,
                 gravity_correction_loaded=None, cycle_active=None,
                 robot_state_age_s=None, is_homed=None,
                 mpc_active=None, teensy_mpc_active=None):
        self.status_age_s = status_age_s
        self.mode = mode
        self.streaming = streaming
        self.leg_vel = leg_vel
        self.leg_acc = leg_acc
        self.leg_jerk = leg_jerk
        self.gravity_correction_loaded = gravity_correction_loaded
        self.cycle_active = cycle_active
        self.robot_state_age_s = robot_state_age_s
        self.is_homed = is_homed
        self.mpc_active = mpc_active
        self.teensy_mpc_active = teensy_mpc_active


def _fresh(age, max_age_s):
    return age is not None and float(age) <= float(max_age_s)


def check_preconditions(snap: Snapshot, *, install_available: bool = True,
                        max_age_s: float = DEFAULT_MAX_AGE_S,
                        limit_tol: float = 1e-6) -> list:
    """P1-P7, in order. Every failure carries the command that fixes it —
    refuse ALL at once, never stop at the first (owner ask, dress-rehearsal
    rule)."""
    out = []

    out.append({
        'id': 'P1', 'name': 'trajectory/status fresh, TRAJECTORY, streaming',
        'ok': (_fresh(snap.status_age_s, max_age_s)
               and snap.mode == 'TRAJECTORY' and bool(snap.streaming)),
        'detail': ('age=%s mode=%r streaming=%r'
                   % (snap.status_age_s, snap.mode, snap.streaming)),
        'fix': ACTIVATE_CMD,
    })

    out.append({
        'id': 'P2', 'name': '/robot_state fresh and is_homed',
        'ok': _fresh(snap.robot_state_age_s, max_age_s) and bool(snap.is_homed),
        'detail': 'age=%s is_homed=%r' % (snap.robot_state_age_s, snap.is_homed),
        'fix': 'Home the robot, then ACTIVATE (runbook)',
    })

    p3_ok = (snap.mpc_active == '0'
             and (snap.teensy_mpc_active in (None, '0')))
    out.append({
        'id': 'P3', 'name': 'wire DISARMED (mpc_active == 0)',
        'ok': p3_ok,
        'detail': ('mpc_active=%r teensy_mpc_active=%r'
                   % (snap.mpc_active, snap.teensy_mpc_active)),
        'fix': ('this gate NEVER moves the robot — relaunch with '
                'auto_arm:=false and do not call /set_setpoint_output'),
    })

    lim_ok = all(
        got is not None and abs(float(got) - want) <= limit_tol * max(1.0, want)
        for got, want in ((snap.leg_vel, SESSION_LEG_VEL_MMPS),
                          (snap.leg_acc, SESSION_LEG_ACC_MMPS2),
                          (snap.leg_jerk, SESSION_LEG_JERK_MMPS3)))
    out.append({
        'id': 'P4', 'name': 'session limits == 300/5000/200000',
        'ok': bool(lim_ok),
        'detail': ('vel=%s acc=%s jerk=%s (want %.0f/%.0f/%.0f)'
                   % (snap.leg_vel, snap.leg_acc, snap.leg_jerk,
                      SESSION_LEG_VEL_MMPS, SESSION_LEG_ACC_MMPS2,
                      SESSION_LEG_JERK_MMPS3)),
        'fix': SET_LIMITS_CMD,
    })

    out.append({
        'id': 'P5', 'name': 'no gravity correction loaded',
        'ok': snap.gravity_correction_loaded is False,
        'detail': 'gravity_correction_loaded=%r' % (snap.gravity_correction_loaded,),
        'fix': ('the skill path has no pre-level yet (R3 item) — relaunch and '
                'do not run "level" this sitting'),
    })

    out.append({
        'id': 'P6', 'name': 'no cycle plan active',
        'ok': snap.cycle_active is False,
        'detail': 'cycle_active=%r' % (snap.cycle_active,),
        'fix': ('the first install must start from the ACTIVATE hold — '
                'relaunch, ACTIVATE, and do not run a prior cycle/skill'),
    })

    out.append({
        'id': 'P7', 'name': 'trajectory/install_segment available',
        'ok': bool(install_available),
        'detail': 'available=%r' % (bool(install_available),),
        'fix': ('source ~/Desktop/Jugglebot-skills/ros_ws/install/setup.bash '
                'in the launch terminal and rebuild both packages '
                '(colcon build --packages-select jugglebot_interfaces jugglebot)'),
    })
    return out


def print_checks(title: str, checks, *, stream=None) -> None:
    s = stream or sys.stdout
    print('\n%s' % (title,), file=s)
    for c in checks:
        tag = c.get('verdict') or ('OK' if c.get('ok') else 'REFUSED')
        print('  %-4s %-4s %s' % (c['id'], tag, c['name']), file=s)
        if c.get('detail'):
            print('         %s' % (c['detail'],), file=s)
        if tag in ('FAIL', 'REFUSED') and c.get('fix'):
            print('    fix: %s' % (c['fix'],), file=s)


# ─────────────────────────────────────────────────────────────────────────────
# Pure core — the synthetic tracker
# ─────────────────────────────────────────────────────────────────────────────

def build_nominal_landings(schedule) -> dict:
    """``{ball_id: [(pos_mm, t_land_abs_s), ...]}`` — the schedule's own
    catch sites, in dispatch order."""
    arrival = np.array([0.0, 0.0, -0.5 * bal.GRAVITY_MMS2 * schedule.flight_s])
    out = {}
    for sk in schedule.skills:
        if sk.kind == sg.CATCH:
            out.setdefault(sk.ball_id, []).append(
                (np.asarray(sk.site.catch_site_mm(), dtype=float), sk.t_abs_s))
    return out, arrival


def make_tracker(schedule, *, jitter_mm: float = 0.0, seed: int = 0,
                 now_fn=time.perf_counter):
    """Arm A (jitter_mm == 0) returns the nominal landing exactly. Arm B adds
    a deterministic uniform +/-jitter_mm offset in x/y, re-drawn every 0.1 s —
    forcing catch RE-SENDS, not modelling the tracker (rehearsal 2026-09-13:
    20 throws, +/-3 mm -> 39 re-send solves in one attempt)."""
    nominal, arrival = build_nominal_landings(schedule)
    rng = np.random.default_rng(seed)
    jit = {'t': -1e9, 'd': np.zeros(3)}

    def tracker(ball_id):
        t = now_fn()
        if jitter_mm > 0.0 and t - jit['t'] > 0.1:
            jit['t'] = t
            jit['d'] = np.array([rng.uniform(-jitter_mm, jitter_mm),
                                 rng.uniform(-jitter_mm, jitter_mm), 0.0])
        for pos, t_land in nominal.get(int(ball_id), ()):
            if t_land > t - 0.05:
                return ex.Landing(pos_mm=pos + jit['d'], vel_mm_s=arrival.copy(),
                                  t_land_abs_s=t_land)
        return None
    return tracker


def biased_landing(site_mm, target_mm, flight_s: float, *,
                   speed_frac: float = SELF_TOSS_SPEED_ERR_FRAC,
                   aim_mrad: float = SELF_TOSS_AIM_ERR_MRAD):
    """``(pos_mm, vel_mm_s, t_land_s)`` — the landing a throw commanded as
    ``site_mm -> target_mm`` over ``flight_s`` actually produces under the
    plan's R3 plant-error model (:data:`SELF_TOSS_SPEED_ERR_FRAC` /
    :data:`SELF_TOSS_AIM_ERR_MRAD`): launch speed ``speed_frac`` HIGH, aim
    tilted ``aim_mrad`` off the commanded direction (rotated toward +x — a
    fixed, deterministic azimuth, since the plan states only the error's
    MAGNITUDE, not a direction, and a fixed one keeps the rehearsal exactly
    repeatable). Crosses the same z the commanded flight targeted
    (``target_mm[2]``), via ``ballistics_bc.arrival_state_at_z`` — exact
    no-drag ballistics, the same boundary condition the QP itself is planned
    to satisfy, so the bias is physically grounded rather than an arbitrary
    offset. MEASURED (2026-09-13, this function, apex 0.9 m self-toss, site
    P1): landing offset −12.7 mm in x, flight +10.8% — the R3 gate's 30 mm
    reach band and 20 ms flight band (brief_common.md § 0) both have margin
    against it, so the bias alone should not itself refuse a throw; the
    LEARNER changing the command over attempts is what this exists to show.
    """
    site_mm = np.asarray(site_mm, dtype=float).reshape(3)
    target_mm = np.asarray(target_mm, dtype=float).reshape(3)
    v_nom = bal.launch_velocity(site_mm, target_mm, float(flight_s))
    speed = float(np.linalg.norm(v_nom))
    theta = float(aim_mrad) * 1e-3
    c, s = math.cos(theta), math.sin(theta)
    vx, vy, vz = v_nom
    v_dir = np.array([vx * c + vz * s, vy, -vx * s + vz * c])
    v_actual = v_dir / np.linalg.norm(v_dir) * speed * (1.0 + float(speed_frac))
    pos, vel, t_s = bal.arrival_state_at_z(site_mm, v_actual, float(target_mm[2]),
                                           descending=True)
    return pos, vel, float(t_s)


def _release_physics_of(kind, terminal):
    """``(site_mm, target_mm, flight_s, t_release_abs_s)`` for the ball
    ``terminal`` releases, or ``None`` when it releases nothing (a plain CATCH,
    a REST). Mirrors ``SkillNode._release_physics`` field for field — that
    method is ROS-node code and may not be imported here (the pure-core /
    no-ROS-at-module-scope split this file's own docstring states)."""
    if kind == sg.THROW:
        return (np.asarray(terminal.site_mm, dtype=float),
               np.asarray(terminal.target_mm, dtype=float),
               float(terminal.flight_s), float(terminal.t_release_s))
    if kind == sg.CATCH and terminal.then_throw is not None:
        tt = terminal.then_throw
        return (np.asarray(tt.site_mm, dtype=float),
               np.asarray(tt.target_mm, dtype=float),
               float(tt.flight_s), float(tt.t_release_s))
    return None


def make_self_toss_tracker(*, jitter_mm: float = 0.0, seed: int = 0,
                           now_fn=time.perf_counter):
    """The R3 self-toss rehearsal tracker, and the ``note_release`` callback
    that feeds it.

    Returns ``(tracker, note_release)``. ``note_release(ball_id, kind,
    terminal)`` must be called by the installer after every ACCEPTED install
    that releases a ball (the same read ``SkillNode._maybe_announce`` makes,
    minus its ROS side effects) — the tracker reports :func:`biased_landing`
    for whatever the LAST noted release commanded, plus arm B's jitter on top
    (the same composition :func:`make_tracker` uses for columns, so
    ``--arm``/``--jitter-mm`` mean the same thing for both patterns). Before
    any release is noted for a ball, the tracker returns ``None`` — same
    "no landing yet" contract as :func:`make_tracker`.
    """
    releases = {}
    rng = np.random.default_rng(seed)
    jit = {'t': -1e9, 'd': np.zeros(3)}

    def note_release(ball_id, kind, terminal):
        physics = _release_physics_of(kind, terminal)
        if physics is not None:
            releases[int(ball_id)] = physics

    def tracker(ball_id):
        physics = releases.get(int(ball_id))
        if physics is None:
            return None
        site_mm, target_mm, flight_s, t_release = physics
        pos, vel, t_s = biased_landing(site_mm, target_mm, flight_s)
        t = now_fn()
        if jitter_mm > 0.0 and t - jit['t'] > 0.1:
            jit['t'] = t
            jit['d'] = np.array([rng.uniform(-jitter_mm, jitter_mm),
                                 rng.uniform(-jitter_mm, jitter_mm), 0.0])
        return ex.Landing(pos_mm=pos + jit['d'], vel_mm_s=vel,
                          t_land_abs_s=t_release + t_s)

    return tracker, note_release


# ─────────────────────────────────────────────────────────────────────────────
# Pure core — driving a SkillExecutor and building CSV rows
# ─────────────────────────────────────────────────────────────────────────────

def _row(*, attempt: int, t_rel_s: float, kind: str, ball_id: int, res,
        is_resend: bool, is_preposition: bool, dispatch_late_ms,
        client_rtt_ms, disarmed_marker, load1, mpc_active,
        max_emit_gap_ms, expected_class=None) -> dict:
    return {
        'attempt': attempt, 't_rel_s': round(float(t_rel_s), 4), 'kind': kind,
        'ball_id': int(ball_id), 'accepted': bool(res.accepted),
        'code': str(res.code), 'server_plan_ms': float(res.plan_wall_s) * 1e3,
        'client_rtt_ms': client_rtt_ms, 'dispatch_late_ms': dispatch_late_ms,
        'is_resend': bool(is_resend), 'is_preposition': bool(is_preposition),
        'splice_k': int(res.splice_k),
        'seeded_post_release': bool(res.seeded_post_release),
        # A refused install carries no splice knot; it is classed by the budget
        # it WOULD have spent (expected_class), so a refused handoff is not read
        # against the unpinned budget (2026-09-13 sitting: it was).
        'budget_class': (budget_class(int(res.splice_k),
                                      bool(res.seeded_post_release))
                         if int(res.splice_k) >= 0 or expected_class is None
                         else expected_class),
        'disarmed_marker': disarmed_marker, 'load1': load1,
        'mpc_active': mpc_active, 'max_emit_gap_ms': max_emit_gap_ms,
        'message': str(res.message)[:200],
    }


def expected_class(skill, is_resend: bool) -> str:
    """The splice budget an install of ``skill`` spends, known before it is
    answered: a re-send splices at the general lead; a skill the schedule gave
    the handoff lead is pinned to the release before it; the launch THROW starts
    a fresh origin; anything else splices at the general lead."""
    from jugglebot.motion.skills.schedule import HANDOFF_LEAD_S
    if is_resend:
        return 'unpinned'
    if float(skill.lead_s) >= HANDOFF_LEAD_S - 1e-9:
        return 'handoff'
    return 'fresh' if skill.kind == sg.THROW else 'unpinned'


def drive_executor(exe, call_meta: list, *, run_t0: float, attempt: int,
                   now_fn, sleep_fn, abort_check=None) -> tuple:
    """Tick ``exe`` at 40 Hz until its schedule's tail + 0.4 s or it ends.

    ``call_meta`` is a list the installer closure appends ONE dict to per
    call (``client_rtt_ms``, ``disarmed_marker``, ``load1``, ``mpc_active``,
    ``max_emit_gap_ms``) — matched 1:1, in call order, against the entries
    ``exe.results`` gains during the same tick (the executor appends a result
    immediately after the installer returns, so the two lists stay in
    lockstep). ``abort_check()``, called after every tick, returns a non-empty
    reason to stop immediately (the live mid-run abort).

    Returns ``(rows, attempt_ended, end_code, abort_reason)``.
    """
    rows = []
    seen_idx = set()
    prev_results = len(exe.results)
    prev_meta = len(call_meta)
    t_end = max(s.t_abs_s for s in exe.schedule.skills) + 0.4
    abort_reason = ''
    while now_fn() < t_end and not exe.attempt_ended:
        t_tick = now_fn()
        exe.tick(t_tick)
        new_results = exe.results[prev_results:]
        new_meta = call_meta[prev_meta:]
        for (idx, skill, res), meta in zip(new_results, new_meta):
            is_resend = idx in seen_idx
            seen_idx.add(idx)
            dispatch_late_ms = (None if is_resend else
                                round((t_tick - skill.dispatch_s()) * 1e3, 3))
            rows.append(_row(
                attempt=attempt, t_rel_s=t_tick - run_t0,
                kind=kind_display(skill, is_resend=is_resend),
                ball_id=skill.ball_id, res=res, is_resend=is_resend,
                is_preposition=False, dispatch_late_ms=dispatch_late_ms,
                expected_class=expected_class(skill, is_resend), **meta))
        prev_results = len(exe.results)
        prev_meta = len(call_meta)
        if abort_check is not None:
            abort_reason = abort_check() or ''
            if abort_reason:
                break
        sleep_fn(_TICK_PERIOD_S)
    return rows, exe.attempt_ended, exe.end_code, abort_reason


# ─────────────────────────────────────────────────────────────────────────────
# The verdict
# ─────────────────────────────────────────────────────────────────────────────

def _pct(values, p):
    if not values:
        return float('nan')
    s = sorted(values)
    idx = min(len(s) - 1, int(round(p * (len(s) - 1))))
    return s[idx]


def _kind_stats_lines(rows) -> list:
    by_kind = {}
    for r in rows:
        by_kind.setdefault(r['kind'], []).append(float(r['server_plan_ms']))
    lines = []
    for k in ('REST-pre', 'THROW', 'CATCH+throw', 'CATCH', 'REST', 're-send'):
        vals = by_kind.get(k)
        if not vals:
            continue
        lines.append('    %-12s n=%-4d min=%6.2f p50=%6.2f p95=%6.2f max=%6.2f ms'
                     % (k, len(vals), min(vals), _pct(vals, 0.5),
                        _pct(vals, 0.95), max(vals)))
    return lines


def evaluate(rows: list, *, status_gaps: list = None, mpc_active_samples: list = None,
            attempts_meta: list = None, mode: str = 'live') -> list:
    """``[{'id','name','verdict','detail'}, ...]`` — G1-G5.

    ``attempts_meta`` is ``[{'ended_early': bool, 'end_code': str,
    'n_skills': int, 'n_dispatched': int}, ...]``, one per attempt, for G4.
    ``mode == 'rehearse'`` SKIPs G3/G5 (no emitter/wire to witness offline).
    """
    checks = []
    sv = solves(rows)

    # ── G1: plan time ────────────────────────────────────────────────────
    all_ms = [float(r['server_plan_ms']) for r in sv]
    if not all_ms:
        checks.append({'id': 'G1', 'name': 'plan time', 'verdict': 'SKIP',
                       'detail': 'no solves recorded'})
    else:
        worst = max(all_ms)
        detail_lines = ['worst %.2f ms over %d solves (bar %.1f ms)'
                        % (worst, len(all_ms), G1_BAR_MS)]
        detail_lines.extend(_kind_stats_lines(sv))
        checks.append({
            'id': 'G1', 'name': 'per-skill plan time < %.0f ms (plan § 4 R2)'
                                % (G1_BAR_MS,),
            'verdict': 'PASS' if worst < G1_BAR_MS else 'FAIL',
            'detail': '\n'.join(detail_lines),
        })

    # ── G2: splice budget ────────────────────────────────────────────────
    late = [r for r in rows if r['code'] == 'SPLICE_TOO_LATE']
    # The round trip is read against a budget only where one was SPENT: an
    # accepted install, or a SPLICE_TOO_LATE (the budget ran out). A planning
    # refusal installed nothing, so its round trip spent no splice budget.
    spent = [r for r in sv if (r['accepted'] or r['code'] == 'SPLICE_TOO_LATE')
             and r['client_rtt_ms'] is not None]
    handoff_rtt = [float(r['client_rtt_ms']) for r in spent
                  if r['budget_class'] == 'handoff']
    unpinned_rtt = [float(r['client_rtt_ms']) for r in spent
                   if r['budget_class'] == 'unpinned']
    hmax = max(handoff_rtt) if handoff_rtt else 0.0
    umax = max(unpinned_rtt) if unpinned_rtt else 0.0
    g2_ok = (not late and hmax < G2_HANDOFF_BUDGET_MS and umax < G2_UNPINNED_BUDGET_MS)
    checks.append({
        'id': 'G2', 'name': 'splice budget (zero SPLICE_TOO_LATE; handoff < '
                            '%.1f ms, unpinned < %.1f ms)'
                            % (G2_HANDOFF_BUDGET_MS, G2_UNPINNED_BUDGET_MS),
        'verdict': 'PASS' if g2_ok else 'FAIL',
        'detail': ('%d SPLICE_TOO_LATE; handoff max %.2f ms (margin %.2f), '
                  'unpinned max %.2f ms (margin %.2f)'
                  % (len(late), hmax, G2_HANDOFF_BUDGET_MS - hmax,
                     umax, G2_UNPINNED_BUDGET_MS - umax)),
    })

    # ── G3: emitter cadence ──────────────────────────────────────────────
    if mode == 'rehearse':
        checks.append({'id': 'G3', 'name': 'emitter cadence', 'verdict': 'SKIP',
                       'detail': 'no emitter offline'})
    elif not status_gaps:
        checks.append({'id': 'G3', 'name': 'emitter cadence', 'verdict': 'SKIP',
                       'detail': 'no /trajectory/status samples collected'})
    else:
        worst_gap = max(status_gaps)
        checks.append({
            'id': 'G3', 'name': 'max_emit_gap_ms < %.0f ms (plan § 5)'
                                % (G3_MAX_GAP_MS,),
            'verdict': 'PASS' if worst_gap < G3_MAX_GAP_MS else 'FAIL',
            'detail': 'p50=%.1f max=%.1f ms over %d samples'
                     % (_pct(status_gaps, 0.5), worst_gap, len(status_gaps)),
        })

    # ── G4: completion ───────────────────────────────────────────────────
    attempts_meta = attempts_meta or []
    ended_early = [a for a in attempts_meta if a['ended_early']]
    n_resend_accept = sum(1 for r in rows if r['is_resend'] and r['accepted'])
    n_resend_refuse = sum(1 for r in rows if r['is_resend'] and not r['accepted'])
    checks.append({
        'id': 'G4', 'name': 'completion (no early end; every scheduled skill '
                            'accepted, re-sends may refuse)',
        'verdict': 'FAIL' if ended_early else (
            'SKIP' if not attempts_meta else 'PASS'),
        'detail': ('%d/%d attempts ended early (%s); re-sends %d accepted / '
                  '%d refused'
                  % (len(ended_early), len(attempts_meta),
                     ', '.join(a['end_code'] for a in ended_early) or 'none',
                     n_resend_accept, n_resend_refuse)),
    })

    # ── G5: no motion ────────────────────────────────────────────────────
    if mode == 'rehearse':
        checks.append({'id': 'G5', 'name': 'no motion', 'verdict': 'SKIP',
                       'detail': 'no wire offline'})
    else:
        accepted = [r for r in rows if r['accepted']]
        unmarked = [r for r in accepted if not r.get('disarmed_marker')]
        mpc_active_samples = mpc_active_samples or []
        saw_armed = any(v == '1' for v in mpc_active_samples)
        g5_ok = (not unmarked) and (not saw_armed)
        checks.append({
            'id': 'G5', 'name': 'no motion (every accepted response carries '
                                '[wire DISARMED; mpc_active never 1)',
            'verdict': 'PASS' if g5_ok else 'FAIL',
            'detail': ('%d/%d accepted rows missing the marker; '
                      'mpc_active seen armed=%s over %d samples'
                      % (len(unmarked), len(accepted), saw_armed,
                         len(mpc_active_samples))),
        })

    return checks


def verdict_rc(checks) -> int:
    return 1 if any(c['verdict'] == 'FAIL' for c in checks) else 0


def git_head() -> str:
    try:
        return subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=_REPO,
            stderr=subprocess.DEVNULL).decode().strip()
    except Exception:                                    # noqa: BLE001
        return ''


# ─────────────────────────────────────────────────────────────────────────────
# --rehearse — offline, zero ROS
# ─────────────────────────────────────────────────────────────────────────────

def rehearse_attempt(attempt: int, *, n_throws: int, jitter_mm: float,
                     limits, geom, run_t0: float, now_fn=time.perf_counter,
                     sleep_fn=time.sleep, state=None, pattern: str = 'columns',
                     site: si.Site = None, learner=None, boxes=None,
                     on_experience=None, apex_m: float = APEX_M) -> tuple:
    """One rehearsed attempt, per ``probe_gate_rehearsal.py`` (verified
    2026-09-13). Returns ``(rows, meta)``.

    ``pattern='self-toss'`` (R3-g) runs ``schedule.compile_self_toss`` through
    the SAME install chain — warm start, epoch clock, learner/box hooks — with
    ``learner``/``boxes``/``on_experience`` wired into the
    :class:`~jugglebot.motion.skills.executor.SkillExecutor` exactly as
    ``SkillNode._svc_start_self_toss`` wires them, and
    :func:`make_self_toss_tracker`'s ``note_release`` recording every accepted
    release so the tracker can report :func:`biased_landing` for it. It does
    NOT skip the manual REST pre-position (§ 1b below) — this bench pre-
    positions self-toss too, at :data:`SELF_TOSS_PRELIFT_S`, so the
    rehearsal never exercises the real ``skills/start_self_toss`` path's OWN
    opening REST (``schedule.FLOOR_LIFT_S``) from the raw ACTIVATE park, nor
    the pre-level (``_prelevel``) that call runs first. See § 1b for why.
    """
    park = uc.CycleState.at_rest(
        np.array([0.0, 0.0, float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM), 0.0, 0.0, 0.0]),
        float(hw.JB_OP_HAND_ACTIVATE_POSITION_REV), RealizeConfig())

    state = {'rec': None} if state is None else state
    call_meta = []
    #: Set below, only for ``pattern='self-toss'``, once ``note_release`` is
    #: built — a mutable cell so ``installer`` (defined once, used by both
    #: patterns) has one call site for the hook rather than a second copy of
    #: the install body.
    on_release = {'fn': None}

    def installer(kind, terminal, t_now_s, ball_id=0):
        rec = state['rec']
        seed_rest = park if rec is None else None
        t_a = now_fn()
        # The QP warm start is carried exactly as trajectory_node carries it
        # (_segment_warm_start: passed to every install, replaced on accept).
        # Omitting it made every rehearsed solve cold (2026-09-13: QP tails of
        # 21-45 ms, and a 139 ms refusal, the node does not pay).
        new_rec, res, _seg = ex.install_segment(
            rec, seed_rest, kind, terminal, t_now_s,
            limits=limits, geom=geom, t_install_s=now_fn,
            warm_start=state.get('ws'))
        rtt_ms = (now_fn() - t_a) * 1e3
        # The node's plan_wall_ms is its callback's own wall time, refusals
        # included (trajectory_node._reject_segment times from entry);
        # install_segment itself reports 0.0 for a refused solve. So the
        # rehearsal reports the call's measured wall time — the same quantity.
        res = dataclasses.replace(res, plan_wall_s=rtt_ms / 1e3)
        if res.accepted:
            state['rec'] = new_rec
            state['ws'] = _seg.warm_start
            if on_release['fn'] is not None:
                on_release['fn'](ball_id, kind, terminal)
        call_meta.append({'client_rtt_ms': round(rtt_ms, 3),
                          'disarmed_marker': None, 'load1': os.getloadavg()[0],
                          'mpc_active': None, 'max_emit_gap_ms': None})
        return res

    rows = []

    # 1) Wait for the previous attempt's last segment to finish (an attempt
    #    that ended early leaves its plan still moving) — common to both
    #    patterns: the NEXT schedule's own opening skill must not splice onto
    #    a plan that has not reached its rest terminal yet.
    if state['rec'] is not None:
        while now_fn() < state['rec'].end_s + 0.05:
            sleep_fn(0.005)

    # 1b) The manual REST pre-position, from the live record exactly as
    #     trajectory_node plans it (a fresh origin at its terminal rest once
    #     it has ended; the park on the first attempt).
    #
    #     COLUMNS: the columns path has no floor lift/pre-level ported, so
    #     this bench supplies the same work-around the R2 gate sitting used
    #     — a 1.0 s lift.
    #
    #     SELF-TOSS: ``compile_self_toss`` also carries its own opening REST
    #     (``schedule.FLOOR_LIFT_S``, landed at 1.5 s — Finding A, RESOLVED:
    #     the original 1.0 s refused ``LIMIT_JERK`` at the R3 session limit,
    #     see the runsheet). Despite that, this bench ALSO manually
    #     pre-positions self-toss here, at :data:`SELF_TOSS_PRELIFT_S`
    #     (1.5 s) — so PLAINLY: this pre-position MASKS the real opening
    #     REST from a raw ACTIVATE park, and masks ``_prelevel`` entirely
    #     (`skills/start_self_toss` pre-levels the platform before compiling
    #     its schedule; this bench never calls it). After the manual
    #     pre-position lands, ``compile_self_toss``'s own opening REST is a
    #     trivial no-op-sized move (the platform is already there) and
    #     installs for free — so this rehearsal exercises the STEADY-STATE
    #     schedule, not the session's first-ever cold-start move. That gap
    #     is why the runsheet's dress rehearsal (§ 3) still runs the REAL
    #     ``skills/start_self_toss`` from a fresh ACTIVATE, not this bench.
    if pattern == 'self-toss':
        target_site = default_self_toss_site() if site is None else site
        rest_target_mm = target_site.rest_site_mm()
        prelift_s = SELF_TOSS_PRELIFT_S
    else:
        rest_target_mm = si.columns_sites(SEPARATION_MM)[0].rest_site_mm()
        prelift_s = 1.0
    t_a = now_fn()
    _new_rec, pre_res, _seg = ex.install_segment(
        state['rec'], park if state['rec'] is None else None, sg.REST,
        sg.RestTerminal(rest_site_mm=rest_target_mm, t_rest_s=t_a + prelift_s),
        t_a, limits=limits, geom=geom, t_install_s=now_fn,
        warm_start=state.get('ws'))
    rtt_ms = (now_fn() - t_a) * 1e3
    pre_res = dataclasses.replace(pre_res, plan_wall_s=rtt_ms / 1e3)
    rows.append(_row(attempt=attempt, t_rel_s=t_a - run_t0, kind='REST-pre',
                     ball_id=0, res=pre_res, is_resend=False,
                     is_preposition=True, dispatch_late_ms=None,
                     client_rtt_ms=round(rtt_ms, 3), disarmed_marker=None,
                     load1=os.getloadavg()[0], mpc_active=None,
                     max_emit_gap_ms=None))
    if not pre_res.accepted:
        return rows, {'ended_early': True,
                     'end_code': 'PRE_REST_' + pre_res.code,
                     'n_skills': 0, 'n_dispatched': 0}
    state['rec'] = _new_rec
    state['ws'] = _seg.warm_start
    end_s = state['rec'].end_s
    while now_fn() < end_s + 0.05:
        sleep_fn(0.005)

    # 2) the schedule, from t0 = now + START_LEAD_S.
    t0 = now_fn() + _START_LEAD_S
    if pattern == 'self-toss':
        site = default_self_toss_site() if site is None else site
        schedule = build_self_toss_schedule(n_throws=n_throws, site=site,
                                            apex_m=apex_m,
                                            t0_abs_s=t0)
        tracker, note_release = make_self_toss_tracker(
            jitter_mm=jitter_mm, seed=attempt, now_fn=now_fn)
        on_release['fn'] = note_release
    else:
        schedule = build_schedule(n_throws=n_throws, t0_abs_s=t0)
        tracker = make_tracker(schedule, jitter_mm=jitter_mm, seed=attempt,
                               now_fn=now_fn)
    exe = ex.SkillExecutor(schedule, installer, tracker=tracker,
                           learner=learner, boxes=boxes,
                           on_experience=on_experience)
    sched_rows, ended, end_code, _abort = drive_executor(
        exe, call_meta, run_t0=run_t0, attempt=attempt, now_fn=now_fn,
        sleep_fn=sleep_fn)
    rows.extend(sched_rows)
    return rows, {'ended_early': ended, 'end_code': end_code,
                 'n_skills': len(schedule.skills),
                 'n_dispatched': len(exe.dispatched)}


def _build_self_toss_context(args, limits):
    """``(site, learner, boxes, on_experience, memory)`` for ``--pattern
    self-toss`` — the R3 owner site, the admissible box loaded from
    ``config/generated/admissible_box.yaml`` (unclipped, with a printed note,
    when the file is missing or was swept under different limits — a bench
    rehearsal should still run, just without the box's safety net offline),
    and a FRESH, throwaway ``Memory`` under a temp directory (never the real
    session's ``temp/learn/<plant_id>/``, so re-running this bench can never
    perturb or be perturbed by a real hardware session's memory)."""
    site_x = SELF_TOSS_SITE_X_MM if args.site_x_mm is None else args.site_x_mm
    site_y = SELF_TOSS_SITE_Y_MM if args.site_y_mm is None else args.site_y_mm
    site = si.Site('P1', np.array([site_x, site_y, si.CATCH_CUP_Z_MM]))

    box_path = os.path.join(_REPO, 'config', 'generated', 'admissible_box.yaml')
    boxes = []
    if os.path.exists(box_path):
        try:
            loaded = adm.load(box_path)
            adm.check_limits(loaded, limits)
            boxes = loaded
        except adm.AdmissibleError as exc:
            print('note: admissible box at %s not usable (%s) -- learner '
                  'throws are REFUSED (NO_ADMISSIBLE_COMMAND: no box to '
                  'clip into)' % (box_path, exc))
    else:
        print('note: %s does not exist -- learner throws are REFUSED '
              '(NO_ADMISSIBLE_COMMAND: no box to clip into)' % (box_path,))

    tmp_dir = tempfile.mkdtemp(prefix='skills_bench_selftoss_')
    mem_path = os.path.join(tmp_dir, 'memory.csv')
    memory = Memory(mem_path)
    print('self-toss memory (fresh, this run only): %s' % (mem_path,))
    learner_cfg = lr.LearnerConfig()
    learner = SimpleNamespace(command=lambda x, y_d: memory.command(x, y_d, learner_cfg))

    def on_experience(exp):
        memory.append(exp)
        print('memory row %d: x=%s u=%s y=%s caught=%s'
              % (len(memory), exp.x.tolist(), exp.u.tolist(), exp.y.tolist(),
                 exp.caught))

    return site, learner, boxes, on_experience, memory


def run_rehearse(args) -> int:
    from jugglebot.motion import blas_threads as _blas
    n_threads, source = _blas.read_blas_threads()
    print(_blas.format_blas_line(n_threads, source))
    self_toss = args.pattern == 'self-toss'
    leg_jerk = SELF_TOSS_LEG_JERK_MMPS3 if self_toss else SESSION_LEG_JERK_MMPS3
    limits = TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=SESSION_LEG_VEL_MMPS, leg_acc_mmps2=SESSION_LEG_ACC_MMPS2,
        leg_jerk_mmps3=leg_jerk, hand_acc_rps2=SESSION_HAND_ACC_RPS2)
    geom = StewartGeometry()
    jitter = 0.0 if args.arm == 'A' else float(args.jitter_mm)
    if args.arm == 'A' and args.jitter_mm != DEFAULT_JITTER_MM:
        print('note: arm A forces jitter to 0.0 mm (the flag is ignored)')

    site = learner = boxes = on_experience = memory = None
    if self_toss:
        site, learner, boxes, on_experience, memory = _build_self_toss_context(
            args, limits)

    # The rehearsal runs on a WALL-CLOCK-SIZED clock: perf_counter shifted to the
    # epoch, so it stays monotonic. The live driver and skill_node schedule on the
    # ROS clock (~1.79e9 s), and a schedule that was exact near t = 0 was not
    # exact there (2026-09-13); a rehearsal near t = 0 could not see it.
    epoch = time.time() - time.perf_counter()

    def now_fn():
        return time.perf_counter() + epoch

    run_t0 = now_fn()
    shared = {'rec': None}
    all_rows, attempts_meta = [], []
    for a in range(args.attempts):
        rows, meta = rehearse_attempt(
            a, n_throws=args.n_throws, jitter_mm=jitter, limits=limits,
            geom=geom, run_t0=run_t0, now_fn=now_fn, state=shared,
            pattern=args.pattern, site=site, learner=learner, boxes=boxes,
            on_experience=on_experience, apex_m=args.apex_m)
        all_rows.extend(rows)
        attempts_meta.append(meta)
        mem_note = '' if memory is None else ' memory rows=%d' % (len(memory),)
        print('attempt %d: %d rows, ended_early=%s end_code=%s (%d/%d skills)%s'
              % (a, len(rows), meta['ended_early'], meta['end_code'],
                 meta['n_dispatched'], meta['n_skills'], mem_note))

    checks = evaluate(all_rows, attempts_meta=attempts_meta, mode='rehearse')
    print_checks('VERDICT (--rehearse, pattern %s, arm %s, jitter %.1f mm)'
                % (args.pattern, args.arm, jitter), checks)
    _write_outputs(args, all_rows, checks, label='rehearse',
                   preconditions=[], mode='rehearse')
    return verdict_rc(checks)


# ─────────────────────────────────────────────────────────────────────────────
# Live run — imports rclpy lazily
# ─────────────────────────────────────────────────────────────────────────────

class _Runner:
    """Thin rclpy client: one service, three subscriptions."""

    def __init__(self, node, timeout_s: float):
        from jugglebot_interfaces.msg import RobotState, TrajectoryStatus
        from jugglebot_interfaces.srv import InstallSegment
        from diagnostic_msgs.msg import DiagnosticStatus
        from std_srvs.srv import Trigger

        self.node = node
        self.timeout_s = timeout_s
        self._InstallSegment = InstallSegment
        self._Trigger = Trigger

        self.status = None
        self.status_stamp = None
        self.link_kv = {}
        self.link_stamp = None
        self.motor_states = None
        self.is_homed = None
        self.robot_state_stamp = None

        self.cli_install = node.create_client(InstallSegment,
                                              'trajectory/install_segment')
        self.cli_hold = node.create_client(Trigger, 'trajectory/hold')
        node.create_subscription(TrajectoryStatus, '/trajectory/status',
                                 self._on_status, 10)
        node.create_subscription(DiagnosticStatus, '/link_status',
                                 self._on_link, 10)
        node.create_subscription(RobotState, '/robot_state',
                                 self._on_robot_state, 10)

    def _on_status(self, msg):
        self.status, self.status_stamp = msg, time.time()

    def _on_link(self, msg):
        self.link_kv = {v.key: v.value for v in msg.values}
        self.link_stamp = time.time()

    def _on_robot_state(self, msg):
        self.motor_states = msg.motor_states
        self.is_homed = bool(msg.is_homed)
        self.robot_state_stamp = time.time()

    def spin(self, seconds: float) -> None:
        import rclpy
        end = time.time() + max(0.0, seconds)
        while True:
            remaining = end - time.time()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self.node, timeout_sec=min(0.02, remaining))

    def _age(self, stamp):
        return None if stamp is None else time.time() - stamp

    def snapshot(self) -> Snapshot:
        st = self.status
        return Snapshot(
            status_age_s=self._age(self.status_stamp),
            mode=None if st is None else str(st.mode),
            streaming=None if st is None else bool(st.streaming),
            leg_vel=None if st is None else float(st.leg_vel_limit_mmps),
            leg_acc=None if st is None else float(st.leg_acc_limit_mmps2),
            leg_jerk=None if st is None else float(st.leg_jerk_limit_mmps3),
            gravity_correction_loaded=(
                None if st is None else bool(st.gravity_correction_loaded)),
            cycle_active=None if st is None else bool(st.cycle_active),
            robot_state_age_s=self._age(self.robot_state_stamp),
            is_homed=self.is_homed,
            mpc_active=self.link_kv.get('mpc_active'),
            teensy_mpc_active=self.link_kv.get('teensy_mpc_active'))

    def latest_call_meta(self) -> dict:
        st = self.status
        return {
            'client_rtt_ms': None,       # filled by the caller around the call
            'disarmed_marker': None,     # filled by the caller from the response
            'load1': os.getloadavg()[0],
            'mpc_active': self.link_kv.get('mpc_active'),
            'max_emit_gap_ms': (None if st is None
                                else round(float(st.max_emit_gap_ms), 2)),
        }

    def install(self, kind, terminal, t_now_s, *, ball_id):
        """Mirrors ``skill_node.SkillNode._installer`` field for field."""
        import rclpy
        req = self._InstallSegment.Request()
        req.kind = _WIRE_KIND[kind]
        req.ball_id = int(ball_id)
        for k, v in install_request_fields(kind, terminal).items():
            setattr(req, k, v)

        if not self.cli_install.wait_for_service(timeout_sec=self.timeout_s):
            return (ex.InstallResult(False, 'SERVICE_UNAVAILABLE',
                                     'trajectory/install_segment unavailable',
                                     0.0), 0.0)
        future = self.cli_install.call_async(req)
        t_a = time.perf_counter()
        rclpy.spin_until_future_complete(self.node, future,
                                         timeout_sec=self.timeout_s)
        rtt_ms = (time.perf_counter() - t_a) * 1e3
        resp = future.result()
        if resp is None:
            return (ex.InstallResult(False, 'SERVICE_TIMEOUT',
                                     'trajectory/install_segment did not answer '
                                     'in %.1f s' % (self.timeout_s,), 0.0),
                    rtt_ms)
        res = ex.InstallResult(bool(resp.accepted), str(resp.code),
                               str(resp.message), float(resp.plan_wall_ms) / 1e3,
                               splice_k=int(resp.splice_k),
                               t0_s=float(resp.t0_mono),
                               event_t_s=float(resp.t_event_mono),
                               seeded_post_release=bool(resp.seeded_post_release))
        return res, rtt_ms

    def hold(self) -> str:
        import rclpy
        try:
            if not self.cli_hold.wait_for_service(timeout_sec=2.0):
                return 'trajectory/hold NOT AVAILABLE — stop the robot by hand'
            future = self.cli_hold.call_async(self._Trigger.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            resp = future.result()
            if resp is None:
                return 'trajectory/hold did not answer in 5 s'
            return 'held: %s' % (resp.message,)
        except Exception as exc:                          # noqa: BLE001
            return 'trajectory/hold raised: %s' % (exc,)


def live_attempt(attempt: int, runner: '_Runner', *, n_throws: int,
                 jitter_mm: float, run_t0: float, node) -> tuple:
    import rclpy

    def clock_now():
        return node.get_clock().now().nanoseconds / 1e9

    call_meta = []
    abort = {'reason': ''}

    def installer(kind, terminal, t_now_s, ball_id=0):
        if runner.link_kv.get('mpc_active') == '1' and not abort['reason']:
            abort['reason'] = 'mpc_active read 1 mid-run'
        if abort['reason']:
            # Once an abort has tripped, no further install reaches the node. An
            # install refusal also ends the executor's attempt, so a tick with
            # several skills due sends nothing more before the post-loop hold.
            call_meta.append({'client_rtt_ms': 0.0, 'disarmed_marker': None,
                              'load1': os.getloadavg()[0],
                              'mpc_active': runner.link_kv.get('mpc_active'),
                              'max_emit_gap_ms': None})
            return ex.InstallResult(False, 'ABORTED', abort['reason'], 0.0)
        result = runner.install(kind, terminal, t_now_s, ball_id=ball_id)
        res, rtt_ms = result
        marker = 'wire DISARMED' in res.message
        call_meta.append({'client_rtt_ms': round(rtt_ms, 3),
                          'disarmed_marker': (marker if res.accepted else None),
                          'load1': os.getloadavg()[0],
                          'mpc_active': runner.link_kv.get('mpc_active'),
                          'max_emit_gap_ms': runner.latest_call_meta()[
                              'max_emit_gap_ms']})
        # Mid-run abort fence #2 (owner spec): an ACCEPTED response missing the
        # marker means setpoints may be reaching the legs. Flagged here, the
        # instant it is known; the hold call itself is centralised in
        # live_attempt's post-loop handling so it never fires twice.
        if res.accepted and not marker and not abort['reason']:
            abort['reason'] = ("accepted response missing 'wire DISARMED': %s"
                               % (res.message,))
        return res

    def sleep_fn(period_s):
        runner.spin(period_s)

    def abort_check():
        if abort['reason']:
            return abort['reason']
        if runner.link_kv.get('mpc_active') == '1':
            abort['reason'] = 'mpc_active read 1 mid-run'
            return abort['reason']
        return ''

    status_gaps, mpc_samples = [], []

    def sample():
        st = runner.status
        if st is not None:
            status_gaps.append(float(st.max_emit_gap_ms))
        mpc_samples.append(runner.link_kv.get('mpc_active'))

    # An attempt that ended early leaves its last segment still moving; the next
    # pre-position must not splice onto it (2026-09-13: every attempt after an
    # early end refused its pre-position LIMIT_JERK or SPLICE_TOO_LATE for exactly
    # that). trajectory/status says how long the active plan has left.
    wait_deadline = clock_now() + 5.0
    while clock_now() < wait_deadline:
        st = runner.status
        if st is not None and float(st.plan_time_remaining_s) <= 0.0:
            break
        runner.spin(0.02)
        sample()

    sites = si.columns_sites(SEPARATION_MM)
    rows = []
    t_a = clock_now()
    pre_terminal = sg.RestTerminal(rest_site_mm=sites[0].rest_site_mm(),
                                   t_rest_s=t_a + 1.0)
    pre_res = installer(sg.REST, pre_terminal, t_a, ball_id=0)
    meta = call_meta[-1]
    rows.append(_row(attempt=attempt, t_rel_s=t_a - run_t0, kind='REST-pre',
                     ball_id=0, res=pre_res, is_resend=False,
                     is_preposition=True, dispatch_late_ms=None, **meta))
    if abort_check():
        runner.hold()
        return rows, {'ended_early': True, 'end_code': 'ABORT_' + abort['reason'],
                      'n_skills': 0, 'n_dispatched': 0}, status_gaps, mpc_samples
    if not pre_res.accepted:
        return rows, {'ended_early': True, 'end_code': 'PRE_REST_' + pre_res.code,
                      'n_skills': 0, 'n_dispatched': 0}, status_gaps, mpc_samples
    # The pre-position's own rest instant (t_a + 1.0, set above) is its end —
    # trajectory_node holds the live plan, so there is no local PlanRecord.end_s
    # to read back here (unlike --rehearse's in-process record).
    end_s = t_a + 1.0
    while clock_now() < end_s + 0.05:
        runner.spin(0.02)
        sample()

    t0 = clock_now() + _START_LEAD_S
    schedule = build_schedule(n_throws=n_throws, t0_abs_s=t0)
    tracker = make_tracker(schedule, jitter_mm=jitter_mm, seed=attempt,
                           now_fn=clock_now)
    exe = ex.SkillExecutor(schedule, installer, tracker=tracker)

    def abort_check_and_sample():
        sample()
        return abort_check()

    sched_rows, ended, end_code, abort_reason = drive_executor(
        exe, call_meta, run_t0=run_t0, attempt=attempt, now_fn=clock_now,
        sleep_fn=sleep_fn, abort_check=abort_check_and_sample)
    rows.extend(sched_rows)
    if abort_reason:
        print('!! ABORT: %s' % (abort_reason,))
        print('   %s' % (runner.hold(),))
        ended, end_code = True, 'ABORT_' + abort_reason
    return rows, {'ended_early': ended, 'end_code': end_code,
                 'n_skills': len(schedule.skills),
                 'n_dispatched': len(exe.dispatched)}, status_gaps, mpc_samples


def run_live(args) -> int:
    import rclpy

    rclpy.init()
    node = None
    rc = 0
    try:
        node = rclpy.create_node('skills_plan_bench')
        runner = _Runner(node, timeout_s=2.0)
        print('waiting %.1f s for telemetry...' % (args.settle_s,))
        runner.spin(args.settle_s)

        install_available = runner.cli_install.wait_for_service(timeout_sec=3.0)
        snap = runner.snapshot()
        pre = check_preconditions(snap, install_available=install_available,
                                  max_age_s=args.max_age_s)
        print_checks('PRECONDITIONS', pre)
        if any(not c['ok'] for c in pre):
            print('\nREFUSED: %d precondition(s) not met — nothing was '
                  'commanded.' % (sum(1 for c in pre if not c['ok'])),
                  file=sys.stderr)
            return 2

        jitter = 0.0 if args.arm == 'A' else float(args.jitter_mm)
        run_t0 = node.get_clock().now().nanoseconds / 1e9
        all_rows, attempts_meta = [], []
        all_gaps, all_mpc = [], []
        for a in range(args.attempts):
            rows, meta, gaps, mpc = live_attempt(
                a, runner, n_throws=args.n_throws, jitter_mm=jitter,
                run_t0=run_t0, node=node)
            all_rows.extend(rows)
            attempts_meta.append(meta)
            all_gaps.extend(gaps)
            all_mpc.extend(mpc)
            print('attempt %d: %d rows, ended_early=%s end_code=%s (%d/%d skills)'
                  % (a, len(rows), meta['ended_early'], meta['end_code'],
                     meta['n_dispatched'], meta['n_skills']))
            if meta['ended_early'] and str(meta['end_code']).startswith('ABORT_'):
                break

        checks = evaluate(all_rows, status_gaps=all_gaps,
                          mpc_active_samples=all_mpc,
                          attempts_meta=attempts_meta, mode='live')
        print_checks('VERDICT (arm %s, label %s)' % (args.arm, args.label),
                    checks)
        _write_outputs(args, all_rows, checks, label=args.label,
                       preconditions=pre, mode='live')
        rc = verdict_rc(checks)
        return rc
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


# ─────────────────────────────────────────────────────────────────────────────
# Output
# ─────────────────────────────────────────────────────────────────────────────

def _write_outputs(args, rows, checks, *, label, preconditions, mode) -> None:
    out_dir = args.out_dir or os.path.join(_REPO, 'temp', 'logs')
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(out_dir, 'skills_plan_bench_%s_%s_%s.csv'
                            % (args.arm, label, stamp))
    meta_path = csv_path[:-4] + '_meta.json'

    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        for r in rows:
            writer.writerow({k: r.get(k) for k in CSV_COLUMNS})

    with open(meta_path, 'w') as f:
        json.dump({
            'tool': TOOL_NAME, 'git_head': git_head(), 'mode': mode,
            'started': datetime.now().isoformat(timespec='seconds'),
            'args': {k: v for k, v in vars(args).items()},
            'limits': {'leg_vel': SESSION_LEG_VEL_MMPS,
                      'leg_acc': SESSION_LEG_ACC_MMPS2,
                      'leg_jerk': SESSION_LEG_JERK_MMPS3,
                      'hand_acc': SESSION_HAND_ACC_RPS2},
            'preconditions': preconditions, 'checks': checks,
            'csv': csv_path, 'n_rows': len(rows),
        }, f, indent=2, default=str)

    print('\ncsv  -> %s' % (csv_path,))
    print('meta -> %s' % (meta_path,))


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def run_skills_check(node) -> tuple:
    """Call ``skill_node``'s ``skills/check`` (item 8, ``SkillNode._svc_
    check``) and return ``(ok, message)`` — a SEPARATE service from this
    driver's own P1-P7 (which gate ``trajectory/install_segment``'s own
    preconditions on ``trajectory_node``): ``skills/check`` reports the R3
    precondition LADDER (mocap / hand / level / ball-evidence) plus the
    admissible-box / limits status on ``skill_node`` itself, so a dress
    rehearsal must see BOTH refusal surfaces in one pass (Workflow Rules:
    "make gates report every refusal at once"). ``rclpy`` is imported lazily —
    this function is only ever called from the live ``--check`` path.
    """
    import rclpy
    from std_srvs.srv import Trigger
    cli = node.create_client(Trigger, 'skills/check')
    if not cli.wait_for_service(timeout_sec=3.0):
        return False, 'skills/check unavailable -- is skill_node running?'
    future = cli.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    resp = future.result()
    if resp is None:
        return False, 'skills/check did not answer in 5 s'
    return bool(resp.success), str(resp.message)


def build_parser():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dry-run', action='store_true',
                    help='print the compiled schedule and gate criteria; ZERO '
                         'ROS calls, no planning')
    ap.add_argument('--rehearse', action='store_true',
                    help='OFFLINE rehearsal through the real planner; zero ROS')
    ap.add_argument('--check', action='store_true',
                    help='live preconditions only (this driver\'s P1-P7 plus '
                         'skill_node\'s skills/check, R3-g), exit 0/2')
    ap.add_argument('--pattern', choices=('columns', 'self-toss'),
                    default='columns',
                    help="'columns' (R2, default) or 'self-toss' (R3: one "
                         'site, the learner + memory in the loop)')
    ap.add_argument('--apex-m', type=float, default=APEX_M,
                    help='self-toss apex (m); default %.2f. The 2026-09-14 '
                         'apex ladder rehearses each rung (0.5-0.9 m) with '
                         'it -- the admissible box is selected by apex, so a '
                         'rung with no swept box is refused '
                         '(NO_ADMISSIBLE_COMMAND) by this bench\'s own '
                         'executor, the same check skill_node runs before '
                         'any motion' % APEX_M)
    ap.add_argument('--site-x-mm', type=float, default=None,
                    help='self-toss site x, platform frame mm (default %.1f, '
                         'the owner P1)' % (SELF_TOSS_SITE_X_MM,))
    ap.add_argument('--site-y-mm', type=float, default=None,
                    help='self-toss site y, platform frame mm (default %.1f)'
                        % (SELF_TOSS_SITE_Y_MM,))
    ap.add_argument('--arm', choices=('A', 'B'), default=None,
                    help='A = exact-landing tracker (no re-sends); B = jittered '
                         'tracker (forces re-sends). For self-toss, "exact" '
                         'means the plant-error bias only (biased_landing) — '
                         'arm B adds jitter ON TOP. Required for --rehearse '
                         'and a live run.')
    ap.add_argument('--attempts', type=int, default=DEFAULT_ATTEMPTS,
                    help='number of attempts (default %d)' % (DEFAULT_ATTEMPTS,))
    ap.add_argument('--n-throws', type=int, default=None,
                    help='throws per attempt (default %d for columns, %d for '
                         'self-toss -- cold-start policy A, one throw per '
                         'attempt until the memory crosses k_min)'
                        % (DEFAULT_N_THROWS, SELF_TOSS_DEFAULT_N_THROWS))
    ap.add_argument('--jitter-mm', type=float, default=DEFAULT_JITTER_MM,
                    help='arm B catch jitter (mm, default %.1f); arm A forces '
                         '0.0' % (DEFAULT_JITTER_MM,))
    ap.add_argument('--label', default=None,
                    help="output label (default 'loaded'; 'rehearse' is forced "
                         "in --rehearse)")
    ap.add_argument('--out-dir', default=None,
                    help='CSV/meta output directory (default temp/logs/)')
    ap.add_argument('--max-age-s', type=float, default=DEFAULT_MAX_AGE_S,
                    help='precondition freshness bound (s, default %.1f)'
                        % (DEFAULT_MAX_AGE_S,))
    ap.add_argument('--settle-s', type=float, default=DEFAULT_SETTLE_S,
                    help='seconds to collect telemetry before the checks '
                         '(default %.1f)' % (DEFAULT_SETTLE_S,))
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.n_throws is None:
        args.n_throws = (SELF_TOSS_DEFAULT_N_THROWS if args.pattern == 'self-toss'
                         else DEFAULT_N_THROWS)

    if args.dry_run:
        print_dry_run(args.n_throws, pattern=args.pattern, apex_m=args.apex_m)
        return 0

    if args.check:
        try:
            import rclpy
        except ImportError:
            print('ABORT: --check needs rclpy (source ROS 2 and run under the '
                  'system python3)', file=sys.stderr)
            return 2
        rclpy.init()
        node = None
        try:
            node = rclpy.create_node('skills_plan_bench_check')
            runner = _Runner(node, timeout_s=2.0)
            runner.spin(args.settle_s)
            install_available = runner.cli_install.wait_for_service(timeout_sec=3.0)
            pre = check_preconditions(runner.snapshot(),
                                      install_available=install_available,
                                      max_age_s=args.max_age_s)
            skills_ok, skills_msg = run_skills_check(node)
            pre.append({
                'id': 'S1', 'name': 'skills/check (R3 ladder + admissible box)',
                'ok': skills_ok, 'detail': skills_msg,
                'fix': ('fix the named row skills/check reports (mocap / hand '
                        '/ level / ball-evidence / box), or start skill_node '
                        'if the service is unavailable'),
            })
            print_checks('PRECONDITIONS (--check)', pre)
            return 0 if all(c['ok'] for c in pre) else 2
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()

    if args.arm is None:
        print('ABORT: --arm A|B is required', file=sys.stderr)
        return 2

    if args.rehearse:
        args.label = 'rehearse'
        return run_rehearse(args)

    if args.label is None:
        args.label = 'loaded'
    try:
        return run_live(args)
    except BenchError as exc:
        print('\nABORT: %s' % (exc,), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
