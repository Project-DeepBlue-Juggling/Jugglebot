#!/usr/bin/env python3
"""toss_trace_recorder — real-ordering trace recorder + offline checker for the
single-ball-toss choreography (Phase 3 of ``plans/active/single-ball-toss.md``).

**This script OBSERVES the graph; it never commands motion, never publishes on
any production topic, never calls any service.**  The goal trigger is the
operator's ``ros2 action send_goal`` (see the runbook,
``tests/hardware/session_phase8_toss_trace.md``).  The Phase-1 baseline this
trace validates is commit ``5447f03``
(``logbook/2026-07-25-toss-phase1-action-sequencer-coordinator.md``).

Why an rclpy subscriber and not ``ros2 topic echo``: echo gives false negatives
for high-rate RELIABLE topics on this Foxy box (memory
``reference_ros2_topic_echo_flaky_foxy``); the established repo pattern is the
``tests/hardware/th_*`` / ``tools/probes/*`` subscriber probes.

Subcommands::

    python3 tests/hardware/toss_trace_recorder.py record   [--out DIR] [--duration S]
            [--mocap-decimate N] [--rosout-all]
    python3 tests/hardware/toss_trace_recorder.py check    TRACE.jsonl
            (--dry | --reject) [--epsilon-ms 5.0] [--timeline]

``record`` needs the ROS 2 environment (system Python 3.8 with
``/opt/ros/foxy/setup.bash`` + ``ros_ws/install/setup.bash`` sourced — NOT the
project venv).  It writes ``temp/logs/toss_trace_<stamp>.jsonl`` plus a
``..._meta.json`` companion (git SHA, per-topic counts, observed QTM body
names, first/last can-bridge ``uptime_ms`` — the uptime-discipline artifact).

``check`` is pure stdlib (json/argparse only — no rclpy import on this path),
so it runs anywhere including the venv.  It evaluates the captured JSONL
against the Phase-3 ordering invariants (DT-1..DT-14 for the waived dry trace,
RJ-1..RJ-4 for the un-waived REJECTED_NO_BALL trace — IDs match the runbook and
the Phase-3 design spec) and prints one PASS/FAIL/AMBIGUOUS/SKIP line per
invariant.  Exit code 0 iff no FAIL.

**Timestamp honesty (RF-3)**: rows are stamped at callback execution; two
messages landing in the same executor wait-set cycle can be observed in either
order.  Every load-bearing gap in the choreography is >= 1 FSM tick (~50 ms) by
design; the checker treats observed gaps smaller than ``--epsilon-ms`` (default
5 ms) as AMBIGUOUS, never PASS — an inversion *larger* than epsilon is a hard
FAIL.  In-bundle sub-tick orderings (vel_scale / prime_dispatched vs armed) are
expected AMBIGUOUS-or-PASS and FAIL only on a >epsilon inversion.
"""

from __future__ import annotations

import argparse
import json
import re
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUT_DIR = REPO_ROOT / 'temp' / 'logs'

# ── Topic keys (row "topic" values; shared by record and check) ──────────────
T_PRIME_HOLD = 'catch/prime_hold'
T_PRIME_DISPATCHED = 'catch/prime_dispatched'
T_ARMED = 'catch/armed'
T_VEL_SCALE = 'catch/vel_scale'
T_ANNOUNCE = 'throw_announcements'
T_DYN_TARGET = 'catch/dynamic_target'
T_TARGET_FB = 'trajectory/target_feedback'
T_HAND = 'hand_telemetry'
T_TRAJ_STATUS = 'trajectory/status'
T_CONTROL_MODE = 'control_mode_topic'
T_BALLS = 'balls'
T_MOCAP = 'rigid_body_poses'
T_LINK = 'link_status'
T_TOSS_FB = '/jugglebot/toss/_action/feedback'
T_TOSS_STATUS = '/jugglebot/toss/_action/status'
T_RELOAD_FB = '/jugglebot/reload/_action/feedback'
T_RELOAD_STATUS = '/jugglebot/reload/_action/status'
T_ROSOUT = '/rosout'

# The self-announced toss's own destination — the owner of the predicted-ball
# track ball_prediction_node synthesises from the ThrowAnnouncement (see DT-14).
ROBOT_NAME = 'jugglebot'

# The choreography wires whose ordering the invariants govern (RJ-2 silence
# set; DT-11's "last choreography row" set).
CHOREOGRAPHY_TOPICS = (
    T_PRIME_HOLD, T_PRIME_DISPATCHED, T_ARMED, T_VEL_SCALE,
    T_ANNOUNCE, T_DYN_TARGET, T_TARGET_FB,
)

# /rosout default node filter (final dotted name component).
ROSOUT_NODE_FILTER = {
    'reload_coordinator_node', 'catch_coordinator_node', 'trajectory_node',
    'teensy_bridge_node', 'ball_tracker_node', 'orchestrator_node',
}

# ── Checker thresholds (derived from the production sources; see spec § 1) ───
STROKE_VEL_RPS = 40.0          # reload_coordinator_node._THROW_STROKE_VEL_RPS
PARK_BAND_REV = 0.5            # hand-parked band (toss_sequencer CHECKING)
ASCENT_POS_REV = 1.5           # DT-6: any pos_meas above this pre-stroke = ascent
RETRACT_BAND_REV = 1.0         # DT-11: post-retract "back in the bottom band"
EPISODE_GAP_S = 0.2            # stroke-episode grouping gap
TARGET_POS_TOL_MM = 30.0       # DT-7 pre-tilt target_pos envelope around (0,0,170)
STROKE_ONSET_TOL_S = 0.3       # DT-9 onset vs announced throw_time
MIN_TICK_GAP_S = 0.010         # DT-1/DT-2: below this the >=1-FSM-tick gap
                               # both invariants claim does not hold (FAIL)
CMD_POS_TOL_REV = 0.05         # cmd-echo "changed" threshold (pos_cmd, rev)
CMD_VEL_TOL_RPS = 0.5          # cmd-echo "changed" threshold (vel_ff_cmd, rev/s)
WINDOW_PRE_PAD_S = 0.5         # scan pad before the goal-accept evidence
WINDOW_END_PAD_S = 2.0         # scan pad after the terminal evidence (teardown)
FALLBACK_PRE_S = 15.0          # rosout-fallback window length before the outcome
EXPECT_HOLD_TO_ARM_MS = (40.0, 80.0)   # DT-1 informational expectation

# Log-line match strings (verified against the node sources 2026-07-25):
MSG_LATCH_ARMED = 'catch latch armed'            # trajectory_node (exact msg)
MSG_LATCH_DISARMED = 'catch latch disarmed'      # trajectory_node (exact msg)
MSG_PRETILT = 'pre-tilt target published from announcement'  # catch_coordinator
MSG_WAIVER = 'waiving the ball-possession'       # reload_coordinator WARN
CCN_PRIME_RE = re.compile(r'\bprim(?:ed|ing)\b', re.IGNORECASE)
OUTCOME_RE = re.compile(r'^Toss ([A-Z][A-Za-z0-9_()]*)')
TRANSIENT_FB_CODES = {'FROZEN', 'STALE_STATE'}

# rcl_interfaces/Log levels (Foxy)
LOG_WARN = 30

# action_msgs/GoalStatus
STATUS_ACCEPTED = 1
STATUS_EXECUTING = 2
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6
TERMINAL_STATUSES = (STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED)

# RJ-1: CHECKING's ordered gates — reject code → the unhealthy precondition
# wire (an earlier-code reject means the bench state is wrong; fix before the
# dry capture).  Order matches toss_sequencer.TossSequencer.step's CHECKING
# block followed by _step_checking (kept name-keyed, not line-keyed: a
# line-numbered reference rots on the first unrelated edit above it).
REJECT_WIRE_MAP = {
    'REJECTED_TIER': 'goal/config: JB_OP_TOSS_TIER must be 8a or 8b '
                     '(shipped default: 8b since 2026-07-28 — do NOT "fix" this '
                     'by reverting the default; a tier outside {8a, 8b} means the '
                     'YAML or the INSTALLED copy is wrong)',
    'REJECTED_CANT_MAKE_LEAD': 'goal: throw_delay_s below the 3.5 s floor',
    'REJECTED_FLIGHT_TIME': 'goal: flight_time_s outside [0.55, 1.10] s',
    'REJECTED_DISPLACEMENT': 'goal (Tier 8b only): |B - A| from the config throw '
                             'site exceeds the 70 mm cap, or exceeds the '
                             'closed-form reach bound for this flight time '
                             '(256 mm at T=0.80 s, 83 mm at T=0.55 s). Under 8b '
                             'this gate answers BEFORE the workspace box, so a '
                             'far-lateral goal reports DISPLACEMENT, not WORKSPACE',
    'REJECTED_TILT_CLAMP': 'goal (Tier 8b only): the aim required to reach B from '
                           'the throw site exceeds the platform tilt ceiling',
    'REJECTED_EVENT_VEL': 'goal: event_vel outside the Teensy [0.3, 7.0] m/s band',
    'REJECTED_WORKSPACE': 'goal: catch pose outside |x|,|y|<=150 mm / z 170+-50 mm. '
                          'Under Tier 8b from throw site (0,0) the LATERAL half is '
                          'unreachable (the 70 mm cap answers first) — check z',
    'REJECTED_WRONG_MODE': 'control_mode_topic is not TRAJECTORY '
                           '(robot not armed into the streaming hold)',
    'REJECTED_MOCAP_STALE': 'rigid_body_poses stale (QTM / mocap_node)',
    'REJECTED_NOT_STREAMING': 'trajectory/status streaming=False (emitter down)',
    'REJECTED_NOT_LEVELLED': 'trajectory_node holds no gravity correction: run '
                             '`level` (it is per-PROCESS and every relaunch '
                             'drops it — RobotState.levelling_complete reading '
                             'True proves nothing). Same code if trajectory/status '
                             'itself went silent for >1 s, so if `level` does not '
                             'clear it, check that trajectory_node is alive and '
                             'that jugglebot_interfaces was rebuilt',
    'REJECTED_HAND_STALE': 'hand_telemetry stale (Teensy TELEMETRY frames absent)',
    'REJECTED_HAND_NOT_PARKED': 'hand pos_meas outside the +-0.5 rev park band',
    'REJECTED_TRACK_ACTIVE': 'phantom tracker expectation destined jugglebot '
                             '(wait for expiry / restart tracker; see RF-5)',
    'REJECTED_BUSY': 'another ball-op goal is running',
}


# ═════════════════════════════════════════════════════════════════════════════
# check subcommand — pure stdlib (no rclpy import on this path)
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class Finding:
    id: str
    verdict: str            # PASS | FAIL | AMBIGUOUS | SKIP
    detail: str


@dataclass
class GoalWindow:
    goal_id: Optional[str]
    t_start: float
    t_end: float
    term_status: Optional[int]
    outcome: Optional[str]
    outcome_t: Optional[float]
    outcome_msg: str = ''
    source: str = 'status'   # 'status' | 'rosout-fallback'

    @property
    def pad_start(self) -> float:
        return self.t_start - WINDOW_PRE_PAD_S

    @property
    def pad_end(self) -> float:
        return self.t_end + WINDOW_END_PAD_S


def load_trace(path: Path) -> List[dict]:
    rows: List[dict] = []
    with open(path, 'r') as fh:
        for lineno, line in enumerate(fh, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError as exc:
                print('WARNING: skipping malformed JSONL line %d: %s'
                      % (lineno, exc), file=sys.stderr)
    rows.sort(key=lambda r: r.get('t', 0.0))
    return rows


def topic_rows(rows: List[dict], topic: str,
               t0: Optional[float] = None,
               t1: Optional[float] = None) -> List[dict]:
    out = []
    for r in rows:
        if r.get('topic') != topic:
            continue
        t = r.get('t', 0.0)
        if t0 is not None and t < t0:
            continue
        if t1 is not None and t > t1:
            continue
        out.append(r)
    return out


def rosout_rows(rows: List[dict], node: Optional[str] = None,
                contains: Optional[str] = None,
                t0: Optional[float] = None,
                t1: Optional[float] = None) -> List[dict]:
    out = []
    for r in topic_rows(rows, T_ROSOUT, t0, t1):
        d = r.get('d', {})
        if node is not None and d.get('node') != node:
            continue
        if contains is not None and contains not in d.get('msg', ''):
            continue
        out.append(r)
    return out


def outcome_lines(rows: List[dict]) -> List[Tuple[float, str, str]]:
    """All coordinator 'Toss <OUTCOME>' lines as (t, outcome, msg)."""
    out = []
    for r in rosout_rows(rows, node='reload_coordinator_node'):
        msg = r['d'].get('msg', '')
        m = OUTCOME_RE.match(msg)
        if m:
            out.append((r['t'], m.group(1), msg))
    return out


def find_goal_windows(rows: List[dict]) -> List[GoalWindow]:
    """Goal windows from /jugglebot/toss/_action/status transitions (primary),
    else from the coordinator's /rosout lines (fallback — spec § 4.4)."""
    first_seen: Dict[str, float] = {}
    terminal: Dict[str, Tuple[float, int]] = {}
    order: List[str] = []
    for r in topic_rows(rows, T_TOSS_STATUS):
        for gid, st in r.get('d', {}).get('goals', []):
            if st >= STATUS_ACCEPTED and gid not in first_seen:
                first_seen[gid] = r['t']
                order.append(gid)
            if st in TERMINAL_STATUSES and gid not in terminal:
                terminal[gid] = (r['t'], st)

    lines = outcome_lines(rows)
    windows: List[GoalWindow] = []
    if first_seen:
        used = set()
        last_t = rows[-1]['t'] if rows else 0.0
        for gid in order:
            t_start = first_seen[gid]
            t_end, st = terminal.get(gid, (last_t, None))
            outcome = None
            o_t = None
            o_msg = ''
            for i, (lt, oc, msg) in enumerate(lines):
                if i in used:
                    continue
                if t_start - 1.0 <= lt <= t_end + 3.0:
                    outcome, o_t, o_msg = oc, lt, msg
                    used.add(i)
                    break
            windows.append(GoalWindow(gid, t_start, t_end, st,
                                      outcome, o_t, o_msg, 'status'))
        return windows

    # Fallback: no status rows captured (status topic missed).
    waivers = rosout_rows(rows, node='reload_coordinator_node',
                          contains=MSG_WAIVER)
    for lt, oc, msg in lines:
        t_start = lt - FALLBACK_PRE_S
        for w in waivers:
            if 0.0 <= lt - w['t'] <= 120.0:
                t_start = min(t_start, w['t'] - 0.5)
                break
        if rows:
            t_start = max(t_start, rows[0]['t'])
        windows.append(GoalWindow(None, t_start, lt, None, oc, lt, msg,
                                  'rosout-fallback'))
    return windows


def select_window(windows: List[GoalWindow], mode: str) -> Tuple[Optional[GoalWindow], str]:
    """Pick the goal window matching the mode's expected outcome class.
    Returns (window, note)."""
    if not windows:
        return None, 'no goal windows found in the trace'
    if mode == 'dry':
        def match(w):
            return w.outcome is not None and (
                w.outcome == 'MISSED' or w.outcome.startswith('MISSED')
                or w.outcome == 'ABORTED_NO_RELEASE')
    else:
        def match(w):
            return w.outcome is not None and w.outcome.startswith('REJECTED')
    matching = [w for w in windows if match(w)]
    if len(windows) == 1:
        return windows[0], ''
    if len(matching) == 1:
        return matching[0], ('%d goal windows in trace; selected the one '
                             'matching --%s' % (len(windows), mode))
    if matching:
        return matching[-1], ('%d windows match --%s; selected the LAST — '
                              'prefer one goal per trace file'
                              % (len(matching), mode))
    return windows[-1], ('no window matches the --%s outcome class; '
                         'checking the last window (outcome=%s)'
                         % (mode, windows[-1].outcome))


def order_verdict(t_early: Optional[float], t_late: Optional[float],
                  eps_s: float,
                  min_gap_s: Optional[float] = None) -> Tuple[str, float]:
    """Ordering verdict for 'early must precede late' with the RF-3 epsilon
    band: |gap| < eps → AMBIGUOUS (never PASS); inversion > eps → FAIL;
    optional min_gap floor below which the ordering guarantee is void."""
    if t_early is None or t_late is None:
        return 'FAIL', float('nan')
    gap_ms = (t_late - t_early) * 1000.0
    eps_ms = eps_s * 1000.0
    if gap_ms <= -eps_ms:
        return 'FAIL', gap_ms
    if abs(gap_ms) < eps_ms:
        return 'AMBIGUOUS', gap_ms
    if min_gap_s is not None and gap_ms < min_gap_s * 1000.0:
        return 'FAIL', gap_ms
    return 'PASS', gap_ms


def combine(fid: str, subs: List[Tuple[str, str]]) -> Finding:
    verdicts = [v for v, _ in subs]
    details = [d for _, d in subs if d]
    if 'FAIL' in verdicts:
        v = 'FAIL'
    elif 'AMBIGUOUS' in verdicts:
        v = 'AMBIGUOUS'
    else:
        v = 'PASS'
    return Finding(fid, v, '; '.join(details))


def stroke_episodes(hand_rows: List[dict]) -> List[dict]:
    """Ascending-stroke episodes: vel_meas >= 40 rev/s with pos_meas > 0.5 rev
    (reload_coordinator's stroke-watch criterion), grouped by gaps > 0.2 s."""
    episodes: List[dict] = []
    last_t = None
    for r in hand_rows:
        d = r.get('d', {})
        if (d.get('vel_meas', 0.0) >= STROKE_VEL_RPS
                and d.get('pos_meas', 0.0) > PARK_BAND_REV):
            t = r['t']
            if last_t is None or (t - last_t) > EPISODE_GAP_S:
                episodes.append({'t_on': t, 't_ros_on': r.get('t_ros'),
                                 't_off': t, 'peak_vel': d.get('vel_meas', 0.0)})
            else:
                ep = episodes[-1]
                ep['t_off'] = t
                ep['peak_vel'] = max(ep['peak_vel'], d.get('vel_meas', 0.0))
            last_t = t
    return episodes


@dataclass
class TraceContext:
    rows: List[dict]
    win: GoalWindow
    eps_s: float
    # dry-trace anchors (None where absent)
    t_hold_true: Optional[float] = None
    t_hold_false: Optional[float] = None
    t_arm_true: Optional[float] = None
    t_arm_false: Optional[float] = None
    t_vel_scale: Optional[float] = None
    ann_row: Optional[dict] = None
    t_dispatch: Optional[float] = None
    dispatch_kind: str = 'none'          # 'throwing-fb' | 'stroke' | 'none'
    episodes: List[dict] = field(default_factory=list)
    rest_pos_cmd: Optional[float] = None
    rest_vel_cmd: Optional[float] = None
    no_release_variant: bool = False


def first_bool_edge(rows: List[dict], topic: str, value: bool,
                    t0: float, t1: float,
                    after: Optional[float] = None) -> Optional[float]:
    for r in topic_rows(rows, topic, t0, t1):
        if bool(r.get('d', {}).get('value')) == value:
            if after is not None and r['t'] < after:
                continue
            return r['t']
    return None


def build_context(rows: List[dict], win: GoalWindow, eps_s: float) -> TraceContext:
    ctx = TraceContext(rows=rows, win=win, eps_s=eps_s)
    p0, p1 = win.pad_start, win.pad_end
    ctx.t_hold_true = first_bool_edge(rows, T_PRIME_HOLD, True, p0, p1)
    if ctx.t_hold_true is not None:
        ctx.t_hold_false = first_bool_edge(rows, T_PRIME_HOLD, False, p0, p1,
                                           after=ctx.t_hold_true)
    ctx.t_arm_true = first_bool_edge(rows, T_ARMED, True, p0, p1)
    if ctx.t_arm_true is not None:
        ctx.t_arm_false = first_bool_edge(rows, T_ARMED, False, p0, p1,
                                          after=ctx.t_arm_true)
    vs = topic_rows(rows, T_VEL_SCALE, p0, p1)
    ctx.t_vel_scale = vs[0]['t'] if vs else None
    for r in topic_rows(rows, T_ANNOUNCE, p0, p1):
        d = r.get('d', {})
        if d.get('thrower_name') == 'jugglebot' and d.get('target_id') == 'jugglebot':
            ctx.ann_row = r
            break
    ctx.no_release_variant = (win.outcome == 'ABORTED_NO_RELEASE')

    # Hand cmd-echo resting values: last row in the 2 s before the window
    # (else the earliest in-window row before the PREPARE bundle).
    pre = topic_rows(rows, T_HAND, p0 - 2.0, win.t_start)
    if pre:
        ctx.rest_pos_cmd = pre[-1]['d'].get('pos_cmd')
        ctx.rest_vel_cmd = pre[-1]['d'].get('vel_ff_cmd')
    else:
        limit = ctx.t_hold_true if ctx.t_hold_true is not None else p1
        early = topic_rows(rows, T_HAND, win.t_start, limit)
        if early:
            ctx.rest_pos_cmd = early[0]['d'].get('pos_cmd')
            ctx.rest_vel_cmd = early[0]['d'].get('vel_ff_cmd')

    ctx.episodes = stroke_episodes(topic_rows(rows, T_HAND, p0, p1))

    # DT-5 dispatch-time proxy: the true set_hand_traj_cmd call instant is
    # UNOBSERVABLE (the dispatch leaves no log line and no topic trace).  The
    # coordinator publishes action feedback at the END of the FSM tick that
    # executes the dispatch (reload_coordinator_node._step_toss_sequence), and
    # that tick is the first whose phase is THROWING (toss_sequencer
    # ._enter_throwing returns PHASE_THROWING + ACTION_DISPATCH_THROW on the
    # same tick) — so the first phase=THROWING toss-feedback row bounds the
    # dispatch from above by ~1 tick.  A cmd-echo scan anchored at the
    # announcement is NOT a valid proxy: on an inverted trace it rediscovers
    # the already-changed echo as a positive gap (2026-07-25 audit finding).
    fb = topic_rows(rows, T_TOSS_FB, p0, p1)
    if win.goal_id is not None:
        scoped = [r for r in fb if r['d'].get('goal_id') == win.goal_id]
        if scoped:
            fb = scoped
    throwing = [r for r in fb if r['d'].get('phase') == 'THROWING']
    if throwing:
        ctx.t_dispatch, ctx.dispatch_kind = throwing[0]['t'], 'throwing-fb'
    elif ctx.episodes:
        ctx.t_dispatch, ctx.dispatch_kind = ctx.episodes[0]['t_on'], 'stroke'
    return ctx


# ── DT-1..DT-14 (dry waived trace) ───────────────────────────────────────────

def check_dt1(rows: List[dict], win: GoalWindow, ctx: TraceContext) -> Finding:
    """prime_hold True precedes armed True by >= ~1 tick."""
    if ctx.t_hold_true is None or ctx.t_arm_true is None:
        return Finding('DT-1', 'FAIL',
                       'missing edge: prime_hold True at %s, armed True at %s'
                       % (ctx.t_hold_true, ctx.t_arm_true))
    v, gap = order_verdict(ctx.t_hold_true, ctx.t_arm_true, ctx.eps_s,
                           min_gap_s=MIN_TICK_GAP_S)
    return Finding('DT-1', v,
                   'prime_hold(t=%.4f) -> armed(t=%.4f) gap %.1f ms '
                   '(expect %.0f-%.0f; <%.0f ms voids the earlier-wait-set '
                   'guarantee)' % (ctx.t_hold_true, ctx.t_arm_true, gap,
                                   EXPECT_HOLD_TO_ARM_MS[0],
                                   EXPECT_HOLD_TO_ARM_MS[1],
                                   MIN_TICK_GAP_S * 1000.0))


def check_dt2(rows: List[dict], win: GoalWindow, ctx: TraceContext) -> Finding:
    """armed True precedes the toss self-announcement by >= ~1 tick."""
    if ctx.ann_row is None:
        return Finding('DT-2', 'FAIL',
                       'no throw_announcements row with thrower_name='
                       'target_id=jugglebot in the window')
    if ctx.t_arm_true is None:
        return Finding('DT-2', 'FAIL', 'no armed True edge in the window')
    v, gap = order_verdict(ctx.t_arm_true, ctx.ann_row['t'], ctx.eps_s,
                           min_gap_s=MIN_TICK_GAP_S)
    return Finding('DT-2', v,
                   'armed(t=%.4f) -> announcement(t=%.4f) gap %.1f ms '
                   '(explicit >=1-tick gap by construction; <%.0f ms voids '
                   'it)' % (ctx.t_arm_true, ctx.ann_row['t'], gap,
                            MIN_TICK_GAP_S * 1000.0))


def check_dt3(rows: List[dict], win: GoalWindow, ctx: TraceContext) -> Finding:
    """vel_scale precedes armed True (in-bundle order; same-tick AMBIGUOUS ok)."""
    if ctx.t_vel_scale is None:
        return Finding('DT-3', 'FAIL', 'no catch/vel_scale row in the window')
    if ctx.t_arm_true is None:
        return Finding('DT-3', 'FAIL', 'no armed True edge in the window')
    v, gap = order_verdict(ctx.t_vel_scale, ctx.t_arm_true, ctx.eps_s)
    return Finding('DT-3', v,
                   'vel_scale(t=%.4f) -> armed(t=%.4f) gap %.1f ms '
                   '(in-bundle; AMBIGUOUS tolerated)'
                   % (ctx.t_vel_scale, ctx.t_arm_true, gap))


def check_dt4(rows: List[dict], win: GoalWindow, ctx: TraceContext) -> Finding:
    """Exactly ONE prime_dispatched belt stamp, before armed; no re-stamping."""
    subs: List[Tuple[str, str]] = []
    all_true = [r for r in topic_rows(rows, T_PRIME_DISPATCHED,
                                      win.pad_start, win.pad_end)
                if bool(r['d'].get('value'))]
    if len(all_true) != 1:
        subs.append(('FAIL', '%d prime_dispatched True rows in the window '
                             '(expected exactly 1; >1 = periodic re-stamping, '
                             "A2's failure mode)" % len(all_true)))
    if all_true:
        t_stamp = all_true[0]['t']
        if ctx.t_hold_true is not None and ctx.t_dispatch is not None:
            if not (ctx.t_hold_true - ctx.eps_s <= t_stamp
                    <= ctx.t_dispatch + ctx.eps_s):
                subs.append(('FAIL', 'stamp t=%.4f outside [prime_hold True, '
                                     'throw dispatch] = [%.4f, %.4f]'
                             % (t_stamp, ctx.t_hold_true, ctx.t_dispatch)))
        v, gap = order_verdict(t_stamp, ctx.t_arm_true, ctx.eps_s)
        subs.append((v, 'stamp(t=%.4f) -> armed gap %.1f ms '
                        '(in-bundle; AMBIGUOUS tolerated)' % (t_stamp, gap)))
    elif not subs:
        subs.append(('FAIL', 'no prime_dispatched True row'))
    return combine('DT-4', subs)


def check_dt5(rows: List[dict], win: GoalWindow, ctx: TraceContext) -> Finding:
    """Announcement precedes the THROWING transition (dispatch-time proxy,
    ~1 tick); no stroke-command evidence before the announcement.

    What is honestly proven: the true ``set_hand_traj_cmd`` call time is
    UNOBSERVABLE — the dispatch leaves no log line and no topic trace — so
    the positive ordering is checked against a proxy: the coordinator
    publishes action feedback at the end of the FSM tick that executes the
    dispatch, and that tick is the first whose phase is THROWING, so the
    first phase=THROWING toss-feedback row trails the dispatch by <= ~1
    tick.  Because the proxy trails, it alone cannot rule out a dispatch a
    few ms BEFORE the announcement inside that tick shadow; a negative scan
    (mirroring DT-8's structure) backs it: ANY hand cmd-echo change in
    [pad_start, announcement) is a hard FAIL."""
    if ctx.ann_row is None:
        return Finding('DT-5', 'FAIL', 'no toss announcement in the window')
    t_ann = ctx.ann_row['t']
    subs: List[Tuple[str, str]] = []
    # Negative scan: zero stroke-command evidence before the announcement.
    if ctx.rest_pos_cmd is not None:
        early = []
        for r in topic_rows(rows, T_HAND, win.pad_start, t_ann - ctx.eps_s):
            d = r.get('d', {})
            if (abs(d.get('pos_cmd', 0.0) - ctx.rest_pos_cmd) > CMD_POS_TOL_REV
                    or abs(d.get('vel_ff_cmd', 0.0) - (ctx.rest_vel_cmd or 0.0))
                    > CMD_VEL_TOL_RPS):
                early.append(r)
        if early:
            subs.append(('FAIL', 'hand cmd-echo changed BEFORE the '
                                 'announcement (%d row(s), first at t=%.4f, '
                                 'pos_cmd %.3f vs resting %.3f rev) — '
                                 'dispatch evidence precedes the announcement'
                         % (len(early), early[0]['t'],
                            early[0]['d'].get('pos_cmd', 0.0),
                            ctx.rest_pos_cmd)))
        else:
            subs.append(('PASS', 'no cmd-echo change in [pad_start, '
                                 'announcement)'))
    else:
        subs.append(('AMBIGUOUS', 'no resting cmd-echo baseline — the '
                                  'pre-announcement negative scan could not '
                                  'run'))
    # Positive ordering against the THROWING-transition proxy.
    if ctx.t_dispatch is None:
        if (ctx.no_release_variant
                and not any(v == 'FAIL' for v, _ in subs)):
            return Finding('DT-5', 'SKIP',
                           'NO_RELEASE variant with no THROWING feedback row '
                           'captured — no dispatch-time proxy to order '
                           'against (no cmd-echo change pre-announcement)')
        subs.append(('FAIL', 'no dispatch-time proxy (no phase=THROWING '
                             'feedback row, no stroke episode)'))
    else:
        v, gap = order_verdict(t_ann, ctx.t_dispatch, ctx.eps_s)
        subs.append((v, 'announcement(t=%.4f) -> THROWING-transition proxy '
                        '[%s](t=%.4f) gap %.1f ms (the throw is the LAST '
                        'commitment; the proxy trails the true '
                        'set_hand_traj_cmd instant, which is unobservable, '
                        'by <=1 tick)'
                     % (t_ann, ctx.dispatch_kind, ctx.t_dispatch, gap)))
    return combine('DT-5', subs)


def check_dt6(rows: List[dict], win: GoalWindow, ctx: TraceContext) -> Finding:
    """No auto-prime while prime_hold is True."""
    if ctx.t_hold_true is None:
        return Finding('DT-6', 'FAIL', 'no prime_hold True edge in the window')
    t1 = ctx.t_hold_false if ctx.t_hold_false is not None else win.pad_end
    subs: List[Tuple[str, str]] = []
    ccn_prime = [r for r in rosout_rows(rows, node='catch_coordinator_node',
                                        t0=ctx.t_hold_true, t1=t1)
                 if CCN_PRIME_RE.search(r['d'].get('msg', ''))]
    if ccn_prime:
        subs.append(('FAIL', '%d CCN prime line(s) inside the hold window; '
                             'first at t=%.4f: %r'
                     % (len(ccn_prime), ccn_prime[0]['t'],
                        ccn_prime[0]['d'].get('msg', '')[:80])))
    stamps = [r for r in topic_rows(rows, T_PRIME_DISPATCHED, ctx.t_hold_true, t1)
              if bool(r['d'].get('value'))]
    if len(stamps) > 1:
        subs.append(('FAIL', '%d prime_dispatched True rows inside the hold '
                             "window (only DT-4's single coordinator stamp "
                             'is allowed)' % len(stamps)))
    stroke_on = ctx.episodes[0]['t_on'] if ctx.episodes else t1
    ascent = [r for r in topic_rows(rows, T_HAND, ctx.t_hold_true,
                                    stroke_on - 0.05)
              if r['d'].get('pos_meas', 0.0) > ASCENT_POS_REV]
    if ascent:
        subs.append(('FAIL', 'hand ascended pre-stroke: pos_meas %.2f rev at '
                             't=%.4f (kind-3 ascent would launch the ball)'
                     % (ascent[0]['d'].get('pos_meas', 0.0), ascent[0]['t'])))
    if not subs:
        subs.append(('PASS', 'no CCN prime lines, single belt stamp, hand '
                             'stayed in the bottom band until the stroke'))
    return combine('DT-6', subs)


def check_dt7(rows: List[dict], win: GoalWindow, ctx: TraceContext) -> Finding:
    """Armed-announcement pre-tilt flows: >=1 pre-tilt log, the predicted-track
    dynamic_target STREAM all within TARGET_POS_TOL_MM of (0,0,170), then
    target_feedback accepted (source 'catch')."""
    subs: List[Tuple[str, str]] = []
    pretilt = rosout_rows(rows, node='catch_coordinator_node',
                          contains=MSG_PRETILT,
                          t0=win.pad_start, t1=win.pad_end)
    if not pretilt:
        subs.append(('FAIL', 'no "%s" rosout line' % MSG_PRETILT))
    t_ann = ctx.ann_row['t'] if ctx.ann_row is not None else win.t_start
    dyn = topic_rows(rows, T_DYN_TARGET, t_ann, win.pad_end)
    if not dyn:
        subs.append(('FAIL', 'no catch/dynamic_target after the announcement '
                             '(the predicted-track pre-position never fired)'))
    if dyn:
        # A self-announced toss seeds its own predicted-ball track, so the catch
        # pipeline STREAMS pre-position targets (not a single pre-tilt). Every
        # target must stay within tol of the nominated (0,0,170): a target that
        # wanders means a REAL ball pulled the reach off the nominated point —
        # which the old "exactly 1" count could not distinguish from the
        # intrinsic stream (see DT-14).
        off = []
        for r in dyn:
            tp = r['d'].get('target_pos', [None, None, None])
            try:
                if max(abs(tp[0] - 0.0), abs(tp[1] - 0.0),
                       abs(tp[2] - 170.0)) > TARGET_POS_TOL_MM:
                    off.append((r['t'], tp))
            except (TypeError, IndexError):
                off.append((r['t'], tp))
        if off:
            subs.append(('FAIL', '%d/%d dynamic_target(s) not within %.0f mm '
                                 'of (0,0,170) (first off at t=%.4f: %s) — the '
                                 'reach wandered off the nominated point'
                         % (len(off), len(dyn), TARGET_POS_TOL_MM,
                            off[0][0], off[0][1])))
        fb = topic_rows(rows, T_TARGET_FB, dyn[0]['t'] - ctx.eps_s, win.pad_end)
        accepted = [r for r in fb
                    if r['d'].get('accepted') and r['d'].get('source') == 'catch']
        hard_rejects = [r for r in fb
                        if not r['d'].get('accepted')
                        and r['d'].get('source') == 'catch'
                        and r['d'].get('code') not in TRANSIENT_FB_CODES]
        if not accepted:
            subs.append(('FAIL', 'no accepted=True source=catch '
                                 'target_feedback after the dynamic_target'))
        if hard_rejects:
            subs.append(('FAIL', 'non-transient catch feedback reject(s): %s'
                         % [r['d'].get('code') for r in hard_rejects]))
    if not subs:
        subs.append(('PASS', 'pre-tilt log + %d dynamic_target(s) all within '
                             '%.0f mm of (0,0,170) + accepted catch feedback'
                     % (len(dyn), TARGET_POS_TOL_MM)))
    return combine('DT-7', subs)


def check_dt8(rows: List[dict], win: GoalWindow, ctx: TraceContext) -> Finding:
    """Zero dynamic_target rows before the first armed True (CCN drops
    unarmed announcements)."""
    if ctx.t_arm_true is None:
        return Finding('DT-8', 'FAIL', 'no armed True edge in the window')
    early = topic_rows(rows, T_DYN_TARGET, win.pad_start,
                       ctx.t_arm_true - ctx.eps_s)
    if early:
        return Finding('DT-8', 'FAIL',
                       '%d dynamic_target row(s) before armed True (first at '
                       't=%.4f) — an unarmed announcement leaked through'
                       % (len(early), early[0]['t']))
    return Finding('DT-8', 'PASS',
                   'no dynamic_target before armed True (t=%.4f)'
                   % ctx.t_arm_true)


def check_dt9(rows: List[dict], win: GoalWindow, ctx: TraceContext) -> Finding:
    """Single stroke; onset at the scheduled release; stroke precedes
    BALL_IN_FLIGHT; no re-dispatch."""
    if ctx.no_release_variant:
        return Finding('DT-9', 'SKIP',
                       'NO_RELEASE variant — re-run for the MISSED capture')
    subs: List[Tuple[str, str]] = []
    if len(ctx.episodes) != 1:
        subs.append(('FAIL', '%d ascending stroke episodes (expected exactly '
                             '1; >1 = re-dispatch class — NEW evidence)'
                     % len(ctx.episodes)))
    if ctx.episodes:
        ep = ctx.episodes[0]
        if (ctx.ann_row is not None
                and ctx.ann_row['d'].get('throw_time') is not None
                and ep.get('t_ros_on') is not None):
            err = ep['t_ros_on'] - ctx.ann_row['d']['throw_time']
            if abs(err) > STROKE_ONSET_TOL_S:
                subs.append(('FAIL', 'stroke onset %.3f s from the announced '
                                     'throw_time (tolerance +-%.1f s)'
                             % (err, STROKE_ONSET_TOL_S)))
            else:
                subs.append(('PASS', 'onset %.0f ms from announced throw_time'
                             % (err * 1000.0,)))
        else:
            subs.append(('AMBIGUOUS', 'no announced throw_time / t_ros to '
                                      'check the onset against'))
        bif = [r for r in topic_rows(rows, T_TOSS_FB, win.pad_start, win.pad_end)
               if r['d'].get('phase') == 'BALL_IN_FLIGHT']
        if bif:
            v, gap = order_verdict(ep['t_on'], bif[0]['t'], ctx.eps_s)
            subs.append((v, 'stroke onset(t=%.4f) -> first BALL_IN_FLIGHT '
                            'feedback gap %.1f ms' % (ep['t_on'], gap)))
        else:
            subs.append(('FAIL', 'no BALL_IN_FLIGHT feedback row'))
        subs.append(('PASS', 'peak vel %.1f rev/s' % ep['peak_vel']))
    return combine('DT-9', subs)


def check_dt10(rows: List[dict], win: GoalWindow, ctx: TraceContext) -> Finding:
    """Feedback phases form the strict subsequence POSITIONING -> PREPARING ->
    THROWING -> BALL_IN_FLIGHT [-> CATCHING [-> SETTLING]]; CHECKING never
    appears; no regressions."""
    canonical = ['POSITIONING', 'PREPARING', 'THROWING', 'BALL_IN_FLIGHT',
                 'CATCHING', 'SETTLING']
    fb = topic_rows(rows, T_TOSS_FB, win.pad_start, win.pad_end)
    if win.goal_id is not None:
        scoped = [r for r in fb if r['d'].get('goal_id') == win.goal_id]
        if scoped:
            fb = scoped
    seq: List[str] = []
    for r in fb:
        ph = r['d'].get('phase')
        if not seq or seq[-1] != ph:
            seq.append(ph)
    subs: List[Tuple[str, str]] = []
    if 'CHECKING' in seq:
        subs.append(('FAIL', 'CHECKING appeared in feedback (it must '
                             'terminate-or-advance on tick 1)'))
    unknown = [p for p in seq if p not in canonical and p != 'CHECKING']
    if unknown:
        subs.append(('FAIL', 'unknown phase(s) %s' % unknown))
    idxs = [canonical.index(p) for p in seq if p in canonical]
    if any(b <= a for a, b in zip(idxs, idxs[1:])):
        subs.append(('FAIL', 'phase regression in %s' % seq))
    required = canonical[:3] if ctx.no_release_variant else canonical[:4]
    missing = [p for p in required if p not in seq]
    if missing:
        subs.append(('FAIL', 'missing required phase(s) %s (saw %s)'
                     % (missing, seq)))
    if not subs:
        subs.append(('PASS', 'phases %s' % ' -> '.join(seq)))
    return combine('DT-10', subs)


def check_dt11(rows: List[dict], win: GoalWindow, ctx: TraceContext) -> Finding:
    """Teardown order: armed False -> 'catch latch disarmed' -> ... ->
    prime_hold False LAST; hand retracts to the bottom band."""
    subs: List[Tuple[str, str]] = []
    if ctx.t_arm_false is None:
        subs.append(('FAIL', 'no armed False edge in the (padded) window'))
    disarm = [r for r in rosout_rows(rows, node='trajectory_node',
                                     t0=win.pad_start, t1=win.pad_end)
              if r['d'].get('msg', '').strip() == MSG_LATCH_DISARMED]
    if not disarm:
        subs.append(('FAIL', 'no "%s" rosout line' % MSG_LATCH_DISARMED))
    if ctx.t_arm_false is not None and disarm:
        v, gap = order_verdict(ctx.t_arm_false, disarm[0]['t'], ctx.eps_s)
        subs.append((v, 'armed False(t=%.4f) -> latch-disarmed log gap %.1f ms'
                     % (ctx.t_arm_false, gap)))
    if ctx.t_hold_false is None:
        subs.append(('FAIL', 'no prime_hold False edge in the (padded) window'))
    else:
        later = []
        for tp in CHOREOGRAPHY_TOPICS:
            for r in topic_rows(rows, tp, ctx.t_hold_false + ctx.eps_s,
                                win.pad_end):
                later.append((r['t'], tp))
        if later:
            subs.append(('FAIL', '%d choreography row(s) after prime_hold '
                                 'False (first: %s at t=%.4f) — prime_hold '
                                 'must be released LAST'
                         % (len(later), later[0][1], later[0][0])))
        else:
            subs.append(('PASS', 'prime_hold False (t=%.4f) is the last '
                                 'choreography row' % ctx.t_hold_false))
    hand = topic_rows(rows, T_HAND, win.pad_start, win.pad_end)
    if hand:
        final_pos = hand[-1]['d'].get('pos_meas', float('nan'))
        if not (final_pos <= RETRACT_BAND_REV):
            subs.append(('FAIL', 'hand did not retract: final pos_meas '
                                 '%.2f rev (expected <= %.1f)'
                         % (final_pos, RETRACT_BAND_REV)))
        else:
            subs.append(('PASS', 'hand retracted to %.2f rev' % final_pos))
    else:
        subs.append(('FAIL', 'no hand_telemetry rows in the window'))
    return combine('DT-11', subs)


def check_dt12(rows: List[dict], win: GoalWindow, ctx: TraceContext) -> Finding:
    """Exactly one truthful 'Toss MISSED' WARN (no diagnostic suffix) + exactly
    one waiver WARN at goal start."""
    if ctx.no_release_variant:
        return Finding('DT-12', 'SKIP',
                       'NO_RELEASE variant — re-run for the MISSED capture')
    subs: List[Tuple[str, str]] = []
    lines = [(t, oc, msg) for (t, oc, msg) in outcome_lines(rows)
             if win.pad_start <= t <= win.pad_end]
    missed = [(t, oc, msg) for (t, oc, msg) in lines if oc.startswith('MISSED')]
    if len(missed) != 1:
        subs.append(('FAIL', '%d MISSED-class outcome lines (expected exactly '
                             '1); all outcome lines: %s'
                     % (len(missed), [oc for _, oc, _ in lines])))
    else:
        t, oc, msg = missed[0]
        if msg.strip() != 'Toss MISSED':
            subs.append(('FAIL', 'outcome line %r is not the bare '
                                 '"Toss MISSED" (diagnostic fields should be '
                                 'NaN/absent on a dry trace)' % msg))
        else:
            subs.append(('PASS', 'one "Toss MISSED" at t=%.4f' % t))
    waivers = rosout_rows(rows, node='reload_coordinator_node',
                          contains=MSG_WAIVER,
                          t0=win.pad_start, t1=win.pad_end)
    if len(waivers) != 1:
        subs.append(('FAIL', '%d waiver WARN lines in the window (expected '
                             'exactly 1 at goal start)' % len(waivers)))
    else:
        lvl = waivers[0]['d'].get('level')
        if lvl is not None and int(lvl) != LOG_WARN:
            subs.append(('AMBIGUOUS', 'waiver line level=%s (expected WARN=30)'
                         % lvl))
        else:
            subs.append(('PASS', 'one waiver WARN at t=%.4f' % waivers[0]['t']))
    return combine('DT-12', subs)


def check_dt13(rows: List[dict], win: GoalWindow, ctx: TraceContext) -> Finding:
    """streaming=True + mode=TRAJECTORY throughout; control_mode TRAJECTORY;
    plan_kind move-then-hold for the positioning leg."""
    subs: List[Tuple[str, str]] = []
    ts = topic_rows(rows, T_TRAJ_STATUS, win.t_start, win.t_end)
    bad = [r for r in ts if not (r['d'].get('streaming')
                                 and r['d'].get('mode') == 'TRAJECTORY')]
    if bad:
        subs.append(('FAIL', '%d trajectory/status row(s) with streaming!='
                             'True or mode!=TRAJECTORY (first at t=%.4f: %s)'
                     % (len(bad), bad[0]['t'], bad[0]['d'])))
    elif not ts:
        subs.append(('FAIL', 'no trajectory/status rows in the window'))
    cm = topic_rows(rows, T_CONTROL_MODE, win.t_start, win.t_end)
    bad_cm = [r for r in cm if r['d'].get('value') != 'TRAJECTORY']
    if bad_cm:
        subs.append(('FAIL', '%d control_mode row(s) != TRAJECTORY (first: %r '
                             'at t=%.4f)' % (len(bad_cm),
                                             bad_cm[0]['d'].get('value'),
                                             bad_cm[0]['t'])))
    elif not cm:
        subs.append(('FAIL', 'no control_mode_topic rows in the window'))
    # plan_kind: the positioning leg drives 'move'. It is NOT expected to return
    # to 'hold' within the window — the self-announcement's predicted track keeps
    # the catch pipeline pre-positioning (an active streaming plan) through the
    # predicted flight, and the recenter-to-hold falls AFTER the trace window at
    # teardown. The load-bearing guarantee is streaming=True + mode=TRAJECTORY
    # throughout (the hard checks above); plan_kind is reported for the record.
    kinds = [(r['t'], r['d'].get('plan_kind')) for r in ts]
    move_ts = [t for t, k in kinds if k == 'move']
    if move_ts:
        tail = [k for t, k in kinds if t > move_ts[-1]]
        subs.append(('PASS', "positioning 'move' present; plan_kind stayed "
                             "under streaming control (tail=%r) — the 'hold' "
                             "recenter falls after the window"
                     % (tail[-1] if tail else 'move',)))
    else:
        subs.append(('AMBIGUOUS', "no 'move' plan_kind row observed — the "
                                  'null positioning move can complete between '
                                  '5 Hz status ticks'))
    return combine('DT-13', subs)


def check_dt14(rows: List[dict], win: GoalWindow, ctx: TraceContext) -> Finding:
    """Pollution guard. A *self-announced* toss always seeds its own predicted
    track: ball_prediction_node synthesises a destination=self BallState from the
    ThrowAnnouncement (status TO_BE_THROWN -> IN_FLIGHT on the announced
    schedule, tracking=0, NO mocap detection), so balls rows are intrinsic to the
    choreography, not pollution. The guard fails only on a REAL detection
    (tracking=1) or a foreign-destination track — a physical stray ball/marker in
    the volume, which is what would actually corrupt a dry trace."""
    b = topic_rows(rows, T_BALLS, win.pad_start, win.pad_end)
    real = []
    for r in b:
        for e in (r['d'].get('balls') or []):
            if int(e.get('tracking', 0)) != 0 or e.get('destination') != ROBOT_NAME:
                real.append((r['t'], e))
    if real:
        t0, e0 = real[0]
        return Finding('DT-14', 'FAIL',
                       '%d real-detection/foreign balls entr(y/ies) in the '
                       'window (first at t=%.4f: %s) — a physical stray '
                       'ball/marker polluted the dry trace, re-run (RF-6)'
                       % (len(real), t0, e0))
    if b:
        ids = sorted({e.get('id') for r in b for e in (r['d'].get('balls') or [])})
        return Finding('DT-14', 'PASS',
                       '%d balls row(s), all the intrinsic announcement-seeded '
                       'predicted track (tracking=0, destination=%s, id(s)=%s) '
                       '— no real detection' % (len(b), ROBOT_NAME, ids))
    return Finding('DT-14', 'PASS', 'no balls rows in the window')


# ── RJ-1..RJ-4 (un-waived reject trace) ──────────────────────────────────────

def check_rj1(rows: List[dict], win: GoalWindow, ctx: TraceContext) -> Finding:
    """Loud reject with the RIGHT code; action status ABORTED.  The
    REJECTED_NO_BALL code itself proves the earlier CHECKING gates (mode /
    streaming / mocap / LEVELLING / hand-fresh / hand-parked) all PASSED —
    including, since 2026-07-26, that trajectory_node was reporting a loaded
    gravity correction on a fresh trajectory/status."""
    subs: List[Tuple[str, str]] = []
    lines = [(t, oc, msg) for (t, oc, msg) in outcome_lines(rows)
             if win.pad_start <= t <= win.pad_end]
    nb = [(t, oc) for (t, oc, _) in lines if oc == 'REJECTED_NO_BALL']
    if len(nb) == 1:
        subs.append(('PASS', 'one "Toss REJECTED_NO_BALL" at t=%.4f — all '
                             'earlier CHECKING gates passed' % nb[0][0]))
    else:
        others = [oc for _, oc, _ in lines if oc != 'REJECTED_NO_BALL']
        hint = ''
        for oc in others:
            base = oc.split('(')[0]
            if base in REJECT_WIRE_MAP:
                hint += ' | %s -> %s' % (base, REJECT_WIRE_MAP[base])
        subs.append(('FAIL', '%d REJECTED_NO_BALL lines (expected exactly 1); '
                             'saw %s%s — an earlier-code reject means that '
                             'precondition wire is unhealthy; fix before the '
                             'dry capture' % (len(nb), others or 'none', hint)))
    if win.source == 'status':
        if win.term_status == STATUS_ABORTED:
            subs.append(('PASS', 'action status ABORTED'))
        else:
            subs.append(('FAIL', 'terminal action status %s (expected '
                                 'ABORTED=6)' % win.term_status))
    else:
        subs.append(('AMBIGUOUS', 'no action-status rows captured; terminal '
                                  'status unverified (rosout-fallback window)'))
    return combine('RJ-1', subs)


def check_rj2(rows: List[dict], win: GoalWindow, ctx: TraceContext) -> Finding:
    """Total choreography silence: nothing armed, nothing moved."""
    subs: List[Tuple[str, str]] = []
    noisy = []
    for tp in CHOREOGRAPHY_TOPICS:
        n = len(topic_rows(rows, tp, win.pad_start, win.pad_end))
        if n:
            noisy.append('%s x%d' % (tp, n))
    if noisy:
        subs.append(('FAIL', 'choreography traffic in the window: %s'
                     % ', '.join(noisy)))
    latch = [r for r in rosout_rows(rows, node='trajectory_node',
                                    t0=win.pad_start, t1=win.pad_end)
             if 'catch latch' in r['d'].get('msg', '')]
    if latch:
        subs.append(('FAIL', '%d "catch latch" rosout line(s) in the window'
                     % len(latch)))
    hand = topic_rows(rows, T_HAND, win.pad_start, win.pad_end)
    if hand and ctx.rest_pos_cmd is not None:
        moved = [r for r in hand
                 if abs(r['d'].get('pos_cmd', 0.0) - ctx.rest_pos_cmd)
                 > CMD_POS_TOL_REV]
        if moved:
            subs.append(('FAIL', 'hand cmd-echo changed at t=%.4f (pos_cmd '
                                 '%.3f vs resting %.3f rev)'
                         % (moved[0]['t'], moved[0]['d'].get('pos_cmd', 0.0),
                            ctx.rest_pos_cmd)))
    ts = topic_rows(rows, T_TRAJ_STATUS, win.pad_start, win.pad_end)
    left_hold = [r for r in ts if r['d'].get('plan_kind') not in ('hold', None)]
    if left_hold:
        subs.append(('FAIL', "plan_kind left 'hold': %r at t=%.4f (a reject "
                             'must move NOTHING)'
                     % (left_hold[0]['d'].get('plan_kind'), left_hold[0]['t'])))
    if not subs:
        subs.append(('PASS', 'zero choreography rows, no latch logs, hand '
                             'cmd-echo unchanged, plan_kind stayed hold'))
    return combine('RJ-2', subs)


def check_rj3(rows: List[dict], win: GoalWindow, ctx: TraceContext) -> Finding:
    """Waiver provably unset: zero waiver WARN lines in the window."""
    w = rosout_rows(rows, node='reload_coordinator_node', contains=MSG_WAIVER,
                    t0=win.pad_start, t1=win.pad_end)
    if w:
        return Finding('RJ-3', 'FAIL',
                       '%d waiver WARN line(s) in the window (first at '
                       't=%.4f) — the waiver was SET during the un-waived '
                       'capture' % (len(w), w[0]['t']))
    return Finding('RJ-3', 'PASS', 'no waiver WARN lines in the window')


def check_rj4(rows: List[dict], win: GoalWindow, ctx: TraceContext) -> Finding:
    """Zero action-feedback publishes (a first-tick terminal never feeds back)."""
    fb = topic_rows(rows, T_TOSS_FB, win.pad_start, win.pad_end)
    if win.goal_id is not None:
        scoped = [r for r in fb if r['d'].get('goal_id') == win.goal_id]
        if scoped or fb:
            fb = scoped if scoped else fb
    if fb:
        return Finding('RJ-4', 'FAIL',
                       '%d toss feedback row(s) in the window (first phase %r '
                       'at t=%.4f) — a reject must publish no feedback'
                       % (len(fb), fb[0]['d'].get('phase'), fb[0]['t']))
    return Finding('RJ-4', 'PASS', 'no toss feedback rows in the window')


DRY_CHECKS = [check_dt1, check_dt2, check_dt3, check_dt4, check_dt5,
              check_dt6, check_dt7, check_dt8, check_dt9, check_dt10,
              check_dt11, check_dt12, check_dt13, check_dt14]
REJECT_CHECKS = [check_rj1, check_rj2, check_rj3, check_rj4]


# ── timeline (--timeline; the human review artifact) ─────────────────────────

def hand_band(pos: float) -> str:
    if pos <= PARK_BAND_REV:
        return 'PARK'
    if pos <= ASCENT_POS_REV:
        return 'LOW'
    if pos >= 9.0:
        return 'TOP'
    return 'MID'


def build_timeline(rows: List[dict], win: GoalWindow,
                   ctx: TraceContext) -> List[Tuple[float, str, str]]:
    ev: List[Tuple[float, str, str]] = []
    for tp in CHOREOGRAPHY_TOPICS:
        for r in topic_rows(rows, tp, win.pad_start, win.pad_end):
            d = r['d']
            if tp == T_ANNOUNCE:
                desc = ('thrower=%s target=%s tof=%.3fs'
                        % (d.get('thrower_name'), d.get('target_id'),
                           d.get('predicted_tof_sec', float('nan'))))
            elif tp == T_DYN_TARGET:
                desc = 'target_pos=%s' % (d.get('target_pos'),)
            elif tp == T_TARGET_FB:
                desc = ('accepted=%s code=%s source=%s'
                        % (d.get('accepted'), d.get('code'), d.get('source')))
            else:
                desc = 'value=%s' % (d.get('value'),)
            ev.append((r['t'], tp, desc))
    for r in topic_rows(rows, T_TOSS_FB, win.pad_start, win.pad_end):
        ev.append((r['t'], 'toss/feedback', 'phase=%s' % r['d'].get('phase')))
    for r in topic_rows(rows, T_TOSS_STATUS, win.pad_start, win.pad_end):
        ev.append((r['t'], 'toss/status', '%s' % (r['d'].get('goals'),)))
    keep = (MSG_LATCH_ARMED, MSG_LATCH_DISARMED, MSG_PRETILT, MSG_WAIVER,
            'prime_hold raised', 'prime_hold released')
    for r in topic_rows(rows, T_ROSOUT, win.pad_start, win.pad_end):
        msg = r['d'].get('msg', '')
        if (any(k in msg for k in keep) or OUTCOME_RE.match(msg)
                or (r['d'].get('node') == 'catch_coordinator_node'
                    and CCN_PRIME_RE.search(msg))):
            ev.append((r['t'], 'rosout/%s' % r['d'].get('node'), msg[:100]))
    last_band = None
    last_kind = None
    for r in topic_rows(rows, T_HAND, win.pad_start, win.pad_end):
        d = r['d']
        band = hand_band(d.get('pos_meas', 0.0))
        if band != last_band:
            ev.append((r['t'], 'hand', 'band %s -> %s (pos %.2f rev, vel '
                       '%.1f rev/s)' % (last_band, band,
                                        d.get('pos_meas', 0.0),
                                        d.get('vel_meas', 0.0))))
            last_band = band
    for ep in ctx.episodes:
        ev.append((ep['t_on'], 'hand', 'STROKE onset (peak %.1f rev/s, ends '
                   't=%.4f)' % (ep['peak_vel'], ep['t_off'])))
    for r in topic_rows(rows, T_TRAJ_STATUS, win.pad_start, win.pad_end):
        k = r['d'].get('plan_kind')
        if k != last_kind:
            ev.append((r['t'], 'traj/status', 'plan_kind -> %s (streaming=%s '
                       'mode=%s)' % (k, r['d'].get('streaming'),
                                     r['d'].get('mode'))))
            last_kind = k
    for r in topic_rows(rows, T_BALLS, win.pad_start, win.pad_end):
        ev.append((r['t'], 'balls', 'POLLUTION: %s' % (r['d'],)))
    ev.sort(key=lambda e: e[0])
    return ev


def cmd_check(args: argparse.Namespace) -> int:
    path = Path(args.trace)
    if not path.exists():
        print('ERROR: trace file not found: %s' % path, file=sys.stderr)
        return 2
    rows = load_trace(path)
    mode = 'dry' if args.dry else 'reject'
    eps_s = args.epsilon_ms / 1000.0

    print('=' * 72)
    print('toss_trace_recorder check --%s' % mode)
    print('  trace: %s  (%d rows)' % (path, len(rows)))
    print('  epsilon: %.1f ms  (gaps < epsilon are AMBIGUOUS, never PASS; '
          'an inversion' % args.epsilon_ms)
    print('   larger than epsilon is a hard FAIL — recorder stamps are '
          'callback-entry')
    print('   times, same-wait-set arrivals can be observed in either order)')
    print('=' * 72)

    windows = find_goal_windows(rows)
    win, note = select_window(windows, mode)
    if win is None:
        print('FAIL: %s' % note)
        return 1
    if note:
        print('NOTE: %s' % note)
    print('goal window [%s]: goal_id=%s t=[%.4f, %.4f] terminal_status=%s '
          'outcome=%s' % (win.source, win.goal_id, win.t_start, win.t_end,
                          win.term_status, win.outcome))

    ctx = build_context(rows, win, eps_s)
    if mode == 'dry' and ctx.no_release_variant:
        print()
        print('*** NO_RELEASE variant — the throw dispatch was eaten '
              '(ERR_TIMEOUT class).')
        print('*** This is a VALID capture of a different path (record it as '
              'epidemic')
        print('*** evidence) — re-run for the MISSED capture.  Two '
              'consecutive')
        print('*** ABORTED_NO_RELEASE => stop; that is the epidemic '
              "investigation's")
        print('*** evidence, not a re-run situation.  (RF-4)')
    print()

    checks = DRY_CHECKS if mode == 'dry' else REJECT_CHECKS
    findings = [c(rows, win, ctx) for c in checks]
    counts = {'PASS': 0, 'FAIL': 0, 'AMBIGUOUS': 0, 'SKIP': 0}
    for f in findings:
        counts[f.verdict] = counts.get(f.verdict, 0) + 1
        print('%-6s %-10s %s' % (f.id, f.verdict, f.detail))
    print()
    print('summary: %d PASS, %d FAIL, %d AMBIGUOUS, %d SKIP'
          % (counts['PASS'], counts['FAIL'], counts['AMBIGUOUS'],
             counts['SKIP']))
    if counts['AMBIGUOUS']:
        print('AMBIGUOUS is acceptable ONLY on DT-3/DT-4 (same-tick bundle '
              'orderings);')
        print('anywhere else it needs a human read of the --timeline before '
              'sign-off.')

    if args.timeline:
        print()
        print('-' * 72)
        print('timeline (t relative to window start %.4f)' % win.t_start)
        print('-' * 72)
        for t, tag, desc in build_timeline(rows, win, ctx):
            print('%+10.3f s  %-28s %s' % (t - win.t_start, tag, desc))

    return 0 if counts['FAIL'] == 0 else 1


# ═════════════════════════════════════════════════════════════════════════════
# record subcommand — rclpy (system Python 3.8 with the ROS env sourced)
# ═════════════════════════════════════════════════════════════════════════════

def _time_to_sec(t: Any) -> float:
    return float(t.sec) + float(t.nanosec) * 1e-9


def _get_repo_sha() -> str:
    try:
        return subprocess.run(
            ['git', 'rev-parse', 'HEAD'], cwd=str(REPO_ROOT),
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            check=True).stdout.decode().strip()
    except Exception:
        return 'unknown'


def cmd_record(args: argparse.Namespace) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (DurabilityPolicy, QoSProfile,
                               ReliabilityPolicy)
    except ImportError as exc:
        print('ERROR: rclpy not importable (%s).\n'
              'The record subcommand needs the ROS 2 environment:\n'
              '  source /opt/ros/foxy/setup.bash\n'
              '  source ros_ws/install/setup.bash\n'
              'and the SYSTEM python3 (not the project venv).' % exc,
              file=sys.stderr)
        return 2
    try:
        from action_msgs.msg import GoalStatusArray
        from diagnostic_msgs.msg import DiagnosticStatus
        from rcl_interfaces.msg import Log
        from std_msgs.msg import Bool, Float64, String
        from jugglebot_interfaces.action import Reload, Toss
        from jugglebot_interfaces.msg import (
            BallStateArray, DynamicTargetCommand, HandTelemetryMessage,
            RigidBodyPoses, TargetFeedback, ThrowAnnouncement,
            TrajectoryStatus)
    except ImportError as exc:
        print('ERROR: interface import failed (%s).\n'
              'Did you `colcon build --packages-select jugglebot_interfaces '
              'jugglebot`\nand `source ros_ws/install/setup.bash`?' % exc,
              file=sys.stderr)
        return 2

    out_dir = Path(args.out) if args.out else DEFAULT_OUT_DIR
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    trace_path = out_dir / ('toss_trace_%s.jsonl' % stamp)
    meta_path = out_dir / ('toss_trace_%s_meta.json' % stamp)

    rclpy.init()

    class TossTraceRecorder(Node):
        """Observe-only recorder: subscribes only; never publishes, never
        calls a service."""

        def __init__(self):
            super().__init__('toss_trace_recorder')
            self.q = deque()          # thread-safe append/popleft
            self.counts: Dict[str, int] = {}
            self.rate_prev: Dict[str, int] = {}
            self.body_names: set = set()
            self.uptime_first: Optional[int] = None
            self.uptime_last: Optional[int] = None
            self.last_phase = '-'
            self.last_outcome = '-'
            self.last_hand: Optional[dict] = None
            self.last_traj: Optional[dict] = None
            self.mocap_arrivals = 0
            self.t0_perf = time.perf_counter()
            self.t0_wall = time.time()
            self.t0_ros = self.get_clock().now().nanoseconds * 1e-9

            status_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)

            self._mk(T_PRIME_HOLD, Bool, 50, self._d_bool)
            self._mk(T_PRIME_DISPATCHED, Bool, 50, self._d_bool)
            self._mk(T_ARMED, Bool, 50, self._d_bool)
            self._mk(T_VEL_SCALE, Float64, 50, self._d_float)
            self._mk(T_ANNOUNCE, ThrowAnnouncement, 50, self._d_announce)
            self._mk(T_DYN_TARGET, DynamicTargetCommand, 50, self._d_dyn)
            self._mk(T_TARGET_FB, TargetFeedback, 50, self._d_target_fb)
            self._mk(T_HAND, HandTelemetryMessage, 200, self._d_hand)
            self._mk(T_TRAJ_STATUS, TrajectoryStatus, 20, self._d_traj)
            self._mk(T_CONTROL_MODE, String, 20, self._d_string)
            self._mk(T_BALLS, BallStateArray, 50, self._d_balls)
            self._mk(T_MOCAP, RigidBodyPoses, 200, self._d_mocap)
            self._mk(T_LINK, DiagnosticStatus, 20, self._d_link)
            self._mk(T_TOSS_FB, Toss.Impl.FeedbackMessage, 50, self._d_fb)
            self._mk(T_TOSS_STATUS, GoalStatusArray, status_qos,
                     self._d_status)
            self._mk(T_RELOAD_FB, Reload.Impl.FeedbackMessage, 50, self._d_fb)
            self._mk(T_RELOAD_STATUS, GoalStatusArray, status_qos,
                     self._d_status)
            self._mk(T_ROSOUT, Log, 1000, self._d_rosout)

            self.create_timer(1.0, self._live_line)

        def _mk(self, topic, msgtype, qos, extractor):
            def cb(msg, _topic=topic, _ex=extractor):
                t = time.perf_counter()
                d = _ex(msg)
                if d is None:       # filtered (rosout) or decimated (mocap)
                    return
                n = self.counts.get(_topic, 0) + 1
                self.counts[_topic] = n
                self.q.append({
                    't': t,
                    't_ros': self.get_clock().now().nanoseconds * 1e-9,
                    'topic': _topic, 'n': n, 'd': d,
                })
            self.create_subscription(msgtype, topic, cb, qos)

        # extractors ---------------------------------------------------------
        @staticmethod
        def _d_bool(msg):
            return {'value': bool(msg.data)}

        @staticmethod
        def _d_float(msg):
            return {'value': float(msg.data)}

        @staticmethod
        def _d_string(msg):
            return {'value': str(msg.data)}

        @staticmethod
        def _d_announce(msg):
            return {
                'thrower_name': str(msg.thrower_name),
                'target_id': str(msg.target_id),
                'throw_time': _time_to_sec(msg.throw_time),
                'landing_time': _time_to_sec(msg.landing_time),
                'predicted_tof_sec': float(msg.predicted_tof_sec),
                'landing_position': [float(msg.landing_position.x),
                                     float(msg.landing_position.y),
                                     float(msg.landing_position.z)],
                'initial_velocity': [float(msg.initial_velocity.x),
                                     float(msg.initial_velocity.y),
                                     float(msg.initial_velocity.z)],
            }

        @staticmethod
        def _d_dyn(msg):
            return {
                'target_pos': [float(msg.target_pos.x),
                               float(msg.target_pos.y),
                               float(msg.target_pos.z)],
                'target_quat': [float(msg.target_quat.w),
                                float(msg.target_quat.x),
                                float(msg.target_quat.y),
                                float(msg.target_quat.z)],
                'target_vel': [float(msg.target_vel.x),
                               float(msg.target_vel.y),
                               float(msg.target_vel.z)],
                'arrival_time': float(msg.arrival_time),
            }

        @staticmethod
        def _d_target_fb(msg):
            return {'accepted': bool(msg.accepted), 'code': str(msg.code),
                    'reason': str(msg.reason),
                    'arrival_time': float(msg.arrival_time),
                    'source': str(msg.source)}

        def _d_hand(self, msg):
            d = {'pos_meas': float(msg.pos_meas),
                 'vel_meas': float(msg.vel_meas),
                 'pos_cmd': float(msg.pos_cmd),
                 'vel_ff_cmd': float(msg.vel_ff_cmd),
                 'tor_ff_cmd': float(msg.tor_ff_cmd),
                 'iq_meas': float(msg.iq_meas)}
            self.last_hand = d
            return d

        def _d_traj(self, msg):
            d = {'streaming': bool(msg.streaming), 'mode': str(msg.mode),
                 'plan_kind': str(msg.plan_kind), 'seq': int(msg.seq),
                 'last_rejection': str(msg.last_rejection),
                 # C-LEVEL-1.O. Recording it is what makes the levelling gate's
                 # cross-process wiring (orchestrator /gravity_offset →
                 # trajectory_node → coordinator CHECKING) visible in a real
                 # capture — mocked-ROS tests cannot see that ordering at all.
                 # getattr, unlike the coordinator's plain read: an observability
                 # tool must degrade rather than die mid-sitting if it is run
                 # against an interface build that predates the field.
                 'gravity_correction_loaded': bool(
                     getattr(msg, 'gravity_correction_loaded', False))}
            self.last_traj = d
            return d

        @staticmethod
        def _d_balls(msg):
            return {'n': len(msg.balls),
                    'balls': [{'id': int(b.id), 'status': int(b.status),
                               'destination': str(b.destination),
                               'tracking': int(b.tracking)}
                              for b in msg.balls]}

        def _d_mocap(self, msg):
            self.mocap_arrivals += 1
            for b in msg.bodies:
                self.body_names.add(str(b.name))
            if (self.mocap_arrivals - 1) % max(1, args.mocap_decimate) != 0:
                return None
            return {'bodies': [{'name': str(b.name),
                                'x': float(b.pose.pose.position.x),
                                'y': float(b.pose.pose.position.y),
                                'z': float(b.pose.pose.position.z)}
                               for b in msg.bodies],
                    'arrivals': self.mocap_arrivals}

        def _d_link(self, msg):
            kv = {v.key: v.value for v in msg.values}
            uptime = None
            if 'uptime_ms' in kv:
                try:
                    uptime = int(kv['uptime_ms'])
                except ValueError:
                    uptime = None
            if uptime is not None:
                if self.uptime_first is None:
                    self.uptime_first = uptime
                self.uptime_last = uptime
            keep = {k: kv[k] for k in
                    ('uptime_ms', 'mpc_active', 'teensy_mpc_active',
                     'time_synced', 'bus1_health', 'bus2_health') if k in kv}
            return {'uptime_ms': uptime, 'level': int(msg.level.hex(), 16)
                    if isinstance(msg.level, bytes) else int(msg.level),
                    'message': str(msg.message), 'kv': keep}

        def _d_fb(self, msg):
            d = {'goal_id': bytes(msg.goal_id.uuid).hex()[:8],
                 'phase': str(msg.feedback.phase)}
            self.last_phase = d['phase']
            return d

        @staticmethod
        def _d_status(msg):
            return {'goals': [[bytes(s.goal_info.goal_id.uuid).hex()[:8],
                               int(s.status)] for s in msg.status_list]}

        def _d_rosout(self, msg):
            name = str(msg.name).split('.')[-1]
            if not args.rosout_all and name not in ROSOUT_NODE_FILTER:
                return None
            m = str(msg.msg)
            if name == 'reload_coordinator_node' and OUTCOME_RE.match(m):
                self.last_outcome = m[:60]
            lvl = msg.level
            if isinstance(lvl, bytes):
                lvl = int.from_bytes(lvl, 'little')
            return {'node': name, 'level': int(lvl), 'msg': m}

        # live confirmation line --------------------------------------------
        def _live_line(self):
            def rate(topic):
                n = self.counts.get(topic, 0)
                r = n - self.rate_prev.get(topic, 0)
                self.rate_prev[topic] = n
                return r
            hand_rate = rate(T_HAND)
            mocap_rate = self.mocap_arrivals - self.rate_prev.get('_mocap', 0)
            self.rate_prev['_mocap'] = self.mocap_arrivals
            hp = (self.last_hand or {}).get('pos_meas', float('nan'))
            tj = self.last_traj or {}
            print('[rec %5.0fs] hand %3d Hz pos %6.2f rev | mocap %3d Hz | '
                  'stream=%s mode=%s plan=%s | phase=%s | hold/disp/armed/'
                  'ann/dyn = %d/%d/%d/%d/%d | balls %d | outcome: %s'
                  % (time.perf_counter() - self.t0_perf, hand_rate, hp,
                     mocap_rate, tj.get('streaming'), tj.get('mode'),
                     tj.get('plan_kind'), self.last_phase,
                     self.counts.get(T_PRIME_HOLD, 0),
                     self.counts.get(T_PRIME_DISPATCHED, 0),
                     self.counts.get(T_ARMED, 0),
                     self.counts.get(T_ANNOUNCE, 0),
                     self.counts.get(T_DYN_TARGET, 0),
                     self.counts.get(T_BALLS, 0),
                     self.last_outcome), flush=True)

    node = TossTraceRecorder()

    stop = threading.Event()
    fh = open(trace_path, 'w')

    def writer():
        while not stop.is_set():
            drained = False
            while True:
                try:
                    row = node.q.popleft()
                except IndexError:
                    break
                fh.write(json.dumps(row) + '\n')
                drained = True
            if drained:
                fh.flush()
            stop.wait(0.5)
        while True:                      # final drain
            try:
                row = node.q.popleft()
            except IndexError:
                break
            fh.write(json.dumps(row) + '\n')
        fh.flush()
        fh.close()

    wt = threading.Thread(target=writer, daemon=True)
    wt.start()

    print('toss_trace_recorder: recording to %s' % trace_path)
    print('  (observe-only: this process subscribes only — it never '
          'publishes, never calls services)')
    print('  Ctrl-C to stop.  Confirm the 1 Hz line shows hand ~100 Hz, '
          'mocap flowing,')
    print('  stream=True mode=TRAJECTORY, and hand pos within the park band '
          'BEFORE sending any goal.')

    deadline = (time.perf_counter() + args.duration) if args.duration else None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if deadline is not None and time.perf_counter() >= deadline:
                print('duration reached — stopping.')
                break
    except KeyboardInterrupt:
        print('\nCtrl-C — stopping.')
    finally:
        end_perf = time.perf_counter()
        end_wall = time.time()
        end_ros = node.get_clock().now().nanoseconds * 1e-9
        stop.set()
        wt.join(timeout=5.0)
        meta = {
            'trace': str(trace_path),
            'start': {'perf': node.t0_perf, 'wall': node.t0_wall,
                      'wall_iso': datetime.fromtimestamp(
                          node.t0_wall).isoformat(timespec='milliseconds'),
                      'ros': node.t0_ros},
            'end': {'perf': end_perf, 'wall': end_wall,
                    'wall_iso': datetime.fromtimestamp(
                        end_wall).isoformat(timespec='milliseconds'),
                    'ros': end_ros},
            'git_sha': _get_repo_sha(),
            'hostname': socket.gethostname(),
            'argv': sys.argv,
            'per_topic_counts': node.counts,
            'mocap_arrivals_full_rate': node.mocap_arrivals,
            'mocap_decimate': args.mocap_decimate,
            'observed_rigid_bodies': sorted(node.body_names),
            'uptime_ms_first': node.uptime_first,
            'uptime_ms_last': node.uptime_last,
            'rosout_filter': (sorted(ROSOUT_NODE_FILTER)
                              if not args.rosout_all else 'ALL'),
        }
        with open(meta_path, 'w') as mf:
            json.dump(meta, mf, indent=2)
        print('trace:  %s' % trace_path)
        print('meta:   %s' % meta_path)
        print('uptime_ms first/last: %s / %s  (quote these in the debrief — '
              'uptime discipline)' % (node.uptime_first, node.uptime_last))
        print('observed rigid bodies: %s' % sorted(node.body_names))
        node.destroy_node()
        rclpy.shutdown()
    return 0


# ═════════════════════════════════════════════════════════════════════════════

def main() -> int:
    p = argparse.ArgumentParser(
        description='Toss choreography real-ordering trace recorder + checker '
                    '(Phase 3, plans/active/single-ball-toss.md)')
    sub = p.add_subparsers(dest='cmd', required=True)

    pr = sub.add_parser('record', help='record a live trace (needs the ROS '
                                       'env; observe-only)')
    pr.add_argument('--out', default=None,
                    help='output dir (default: temp/logs/)')
    pr.add_argument('--duration', type=float, default=None,
                    help='stop after S seconds (default: run until Ctrl-C)')
    pr.add_argument('--mocap-decimate', type=int, default=10,
                    help='record 1-in-N rigid_body_poses rows (default 10; '
                         'a full-rate arrival counter is always kept)')
    pr.add_argument('--rosout-all', action='store_true',
                    help='disable the default /rosout node filter')

    pc = sub.add_parser('check', help='evaluate a captured JSONL against the '
                                      'Phase-3 invariants (pure stdlib)')
    pc.add_argument('trace', help='path to toss_trace_*.jsonl')
    g = pc.add_mutually_exclusive_group(required=True)
    g.add_argument('--dry', action='store_true',
                   help='waived dry-choreography trace (DT-1..DT-14)')
    g.add_argument('--reject', action='store_true',
                   help='un-waived REJECTED_NO_BALL trace (RJ-1..RJ-4)')
    pc.add_argument('--epsilon-ms', type=float, default=5.0,
                    help='observation-ambiguity band (default 5.0 ms)')
    pc.add_argument('--timeline', action='store_true',
                    help='print the merged ordered event timeline')

    args = p.parse_args()
    if args.cmd == 'record':
        return cmd_record(args)
    return cmd_check(args)


if __name__ == '__main__':
    sys.exit(main())
