"""skill_node — the skill-stack orchestrator shell (R2, Unit D2, plan § 2.4).

Ties the pure-Python ``motion.skills`` stack (schedule / segments / executor —
no ROS imports, ``motion/`` rule) to the ROS graph: tracks ``/balls`` landings
and the hand's live possession evidence, ticks a
:class:`~jugglebot.motion.skills.executor.SkillExecutor` at 40 Hz, and installs
each dispatched skill through a synchronous ``trajectory/install_segment``
call. ALL PLANNING happens inside ``trajectory_node`` (see that service's
header for why: only it holds the live commanded state); this node's whole job
is schedule bookkeeping and dispatch — a thin wrapper over a pure-Python
policy, exactly as the other ROS nodes in this package are.

``jugglebot/juggle`` (the ``Juggle`` action, R4 owner decision D3) is THE
start surface: one goal names a ``pattern`` (``self_toss`` | ``hop`` |
``columns`` — :data:`~jugglebot.motion.skills.schedule.Schedule.pattern`'s
own names) and an optional ``reload`` flag. Goal accept runs the shared
no-motion refusal ladder and compiles the schedule (``_start_pattern`` —
merges the R2/R3 columns-start and self-toss-start Trigger services
(deleted at R4) into one path); ``self_toss``/``hop`` also
pre-level the platform (``_prelevel``, the same gravity-level rest
``reload_coordinator_node._unified_prelevel`` brings the session-start floor
lift to) and wire the R3 precondition ladder (``executor.Observations`` /
``precondition_refusals``) through ``_observations``/``_ball_evidence``; a
``reload=True`` goal opens with the Ball Butler reload (``_start_reload``)
before its own throws. Cancelling the goal ends the current attempt
(``_stop_attempt``, the former stop-service body unchanged) — every
segment is rest-terminal (``segments``' invariant), so ending an attempt
needs no motion command of its own: whatever is streaming already ends at
rest, and a released ball's outcome keeps finalising after the attempt ends.
``skills/check`` reports every current ladder refusal plus the box/limits
status in one call, for a dress-rehearsal runsheet.
"""

from __future__ import annotations

import collections
import csv
import dataclasses
import math
import os
import threading
import time
from types import SimpleNamespace
from typing import Dict, List, Tuple

import numpy as np

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import String
from std_srvs.srv import Trigger
from jugglebot_interfaces.action import Juggle
from jugglebot_interfaces.msg import (BallButlerHeartbeat, BallStateArray,
                                      HandTelemetryMessage, RigidBodyPoses,
                                      ThrowAnnouncement, TrajectoryStatus)
from jugglebot_interfaces.srv import BallButlerThrow, GoToPose, InstallSegment

import jugglebot.hardware_config as hw
import jugglebot.protocol_config as proto
from jugglebot import ball_possession
from jugglebot.motion import blas_threads
from jugglebot.motion.skills import admissible as adm
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import learner as lr
from jugglebot.motion.skills.executor import (InstallResult, Landing,
                                              Observations, SkillExecutor,
                                              precondition_refusals)
from jugglebot.motion.skills.hand_launch import HandLaunchMonitor
from jugglebot.motion.skills.memory import Memory, memory_path
from jugglebot.motion.skills.schedule import (HOME_BAND_REV, LEAD_S,
                                              OneBallPattern, Pattern,
                                              PRETILT_S, compile_columns,
                                              compile_one_ball,
                                              compile_reload,
                                              compile_reload_wait,
                                              floor_lift_s, home_hand_bounds)
from jugglebot.motion.skills.segments import CATCH, REST, THROW
from jugglebot.motion.skills.sites import (CATCH_CUP_Z_MM, REST_CUP_Z_MM,
                                           REST_HAND_REV, Site, columns_sites)
from jugglebot.motion.tilt_map import find_repo_root
from jugglebot.motion.trajectory import ballistics_bc
from jugglebot.ball_possession import (
    FlightLatch, advance_flight_latches, flight_in_progress)

# BallStatus enum (BallState.msg): 0 = TO_BE_THROWN, 1 = IN_FLIGHT, 2 = CAUGHT.
# Restated (not imported from reload_coordinator_node — that module is ROS/FSM
# code and this one must stay independent of it) — see brief_common.md's
# "restate, don't import from the FSM node" instruction (R3-e2 item 6, same
# discipline applied here for item 3's correlation).
_BALL_STATUS_IN_FLIGHT = 1
# TrackingConfidence enum (BallState.msg): 1 = CONFIRMED (mocap-matched —
# physical airborne evidence; IN_FLIGHT status alone is time-based and proves
# nothing).
_BALL_TRACKING_CONFIRMED = 1

# The repo root, found the same way reload_coordinator_node._REPO_ROOT is
# (tilt_map.find_repo_root's MARKER walk, never a fixed __file__ depth — see
# that function's docstring for why a fixed walk breaks under colcon install).
# None for a genuinely detached deployment, where the admissible box / memory
# paths below cannot be resolved and a self-toss/hop Juggle goal refuses.
_REPO_ROOT = find_repo_root(__file__)
_ADMISSIBLE_BOX_PATH = (os.path.join(_REPO_ROOT, 'config', 'generated',
                                     'admissible_box.yaml')
                       if _REPO_ROOT else None)

# ``sk_seg`` kind string -> ``InstallSegment.Request`` wire constant. Built from
# the module's own KIND_* names below (not restated as bare ints), same
# discipline as trajectory_node's `_SEGMENT_KINDS`.
_WIRE_KIND = {THROW: InstallSegment.Request.KIND_THROW,
             CATCH: InstallSegment.Request.KIND_CATCH,
             REST: InstallSegment.Request.KIND_REST}

#: How long a synchronous `trajectory/install_segment` call waits for its
#: answer. Mirrors `reload_coordinator_node._call_plan_cycle`'s flat bound: the
#: service itself carries its own < 50 ms core / plan_wall_ms budget
#: (InstallSegment.srv), so this is a "the service died" backstop, not a
#: nominal-latency budget.
_SERVICE_WAIT_S = 2.0

#: SkillExecutor.tick cadence. The plan's dispatch-lead arithmetic
#: (`schedule._MIN_WINDOW_S`, `executor.LEAD_KNOTS`) is quantised to the 40 Hz
#: knot grid, so ticking at the same rate is the only choice that cannot itself
#: become the reason a dispatch lands late.
_TICK_HZ = 40.0

#: The Juggle action's `execute_callback` feedback publish rate (brief: "at
#: <= 5 Hz") — one order below the 40 Hz tick, plenty for a human-readable
#: throw_index/caught readout and no load on the action transport.
_FEEDBACK_HZ = 5.0

#: `_stop_attempt`'s bound on waiting for `_tick_lock` (finding 12, R3 audit,
#: 2026-09-13): a tick mid-`_dispatch` can itself be blocked inside
#: `_wait_future` on the install client, up to `_SERVICE_WAIT_S`, and
#: `_prelevel`'s own `go_to_pose` round trip carries the same bound again if
#: a tick is what triggered it -- `2 * _SERVICE_WAIT_S` covers both, plus one
#: tick period so an ordinary (non-blocked) in-progress tick has time to
#: finish on its own cadence.
_STOP_LOCK_WAIT_S = 2.0 * _SERVICE_WAIT_S + 1.0 / _TICK_HZ

#: The R3 precondition ladder's freshness windows (item 6, plan carried R3
#: note / `executor.Observations`). Restated, not imported, from
#: `reload_coordinator_node` — that module is FSM/ROS code slated for deletion
#: at R4 (module docstring), and these are the SAME physical facts (a topic's
#: publish rate / the sensor's own noise), not a second definition of them.
#: Restates `reload_coordinator_node._MOCAP_STALE_S`.
_MOCAP_STALE_S = 0.5
#: Restates `reload_coordinator_node._TRAJ_STATUS_STALE_S` (the topic
#: itself is 5 Hz; also the freshness window for `trajectory/commanded_position`,
#: same as `_live_commanded_position` there).
_TRAJ_STATUS_STALE_S = 1.0
#: The hand-telemetry staleness bound `_observations` and `tools/probes/
#: possession_replay.py::_sensor` both read (the FSM's own copy died with
#: `reload_coordinator_node.py` at R4, 2026-09-24 — this is now the one
#: statement of the value, not a restatement of a dead module's).
_HAND_STATE_STALE_S = 0.5

#: The session-start mocap-Platform-vs-commanded-position frame check (plan
#: `plans/archived/cup-contact-contract.md` § 1, 2026-09-18): on 2026-09-18 the
#: mocap `Platform` body sat +30 mm in y of the commanded platform position
#: with `Base` aligned to 1 mm -- re-aligning QTM to the base moved the
#: reported platform between +30 and -50 mm, i.e. a small base-alignment
#: error amplified over the base-to-platform lever. A learner with lateral
#: authority would spend its first throws "correcting" that as if it were a
#: miss, so lateral authority stays pinned until this offset is below
#: `_FRAME_CHECK_LIMIT_MM` at session start.
#:
#: FRAME/UNITS, established from the code before subtracting anything (not
#: assumed): `mocap_interface.py` (`base_frame_bodies` does not include
#: "Platform") publishes a non-base body's x/y UNCHANGED from the QTM stream
#: and shifts only z, by the base-to-platform transform
#: (`mocap_interface.py::MocapInterface.on_packet`, ~L396-417); `trajectory_node`
#: publishes `trajectory/commanded_position` in the SAME STOW/platform_start
#: frame. `reload_coordinator_node`'s own toss-arrival mocap cross-check
#: (`_TOSS_MOCAP_BODY_PARAM`, `_on_mocap`) compares exactly these two
#: UNCONVERTED for the same reason ("converting either side to global would
#: double-add GEOM_INITIAL_HEIGHT_MM") -- so x/y from the two topics subtract
#: directly here too, with no rotation/translation applied. z carries the
#: base-to-platform offset and is deliberately not part of this check (it is
#: a lateral-authority question, plan § 1).
#:
#: WHAT THE OFFSET IS (2026-09-22 sitting, analysed 2026-09-23,
#: `logbook/2026-09-23-cup-contact-first-sitting.md`): with QTM re-aligned to
#: the base the Platform body sat (-1.56, -8.53) mm from the command with a
#: 0.03 mm standard deviation over 77 s, invariant under relocating the base.
#: That is not alignment noise: 574.3 mm (the STOW height) x the levelling
#: pose offset (0.015, 0.002) rad = (8.6, 1.1) mm -- a lever arm. The IK
#: rotates the platform about its own centre (`ik_solver.py:161`), so the
#: software cannot produce it; the QTM Base-body frame and the machine's
#: base plane differ by a small fixed tilt, and a point ~600 mm up the base
#: axis appears ~8 mm sideways. The commanded position lives in the base
#: frame, the tracker's landings in the QTM frame, and the cup is physically
#: at command + offset in the QTM frame. So the SAME measured offset is
#: SUBTRACTED from every tracker landing (`_on_balls`, the one entry point
#: both the learner outcome and the catch aim read) -- a landing then means
#: "relative to where the cup really is", and the learner cannot "correct" a
#: frame offset into the throws. The check's limit is therefore a SANITY
#: bound on the alignment, not a precision target, and a STABILITY
#: requirement on the Platform body (`_FRAME_CHECK_PLAT_SPREAD_MM`): a
#: jittering or moving body is not a session-start number.
#: The window the Platform buffer is judged over, and the commanded
#: position's own at-rest window -- one clock (`time.perf_counter()`), one
#: window, for both.
_FRAME_CHECK_WINDOW_S = 1.0
#: Above this mean-offset norm the base alignment is wrong outright (the
#: 2026-09-18 +30..-50 mm cases), not a lever arm, and no subtraction is
#: trusted -- lateral authority is refused (plan § 1). Was 5.0 mm through
#: 2026-09-22, when it refused Block B on the 8.5 mm lever arm above.
_FRAME_CHECK_LIMIT_MM = 25.0
#: The Platform body's own bounding-box diagonal inside the window must stay
#: under this: the 2026-09-22 measurement scattered 0.03 mm (sd) at rest, so
#: anything past 2 mm is a moving platform, a flickering body definition or a
#: mis-tracked marker -- refuses to evaluate rather than subtract a bad
#: number from every landing.
_FRAME_CHECK_PLAT_SPREAD_MM = 2.0
#: The commanded position's bounding-box diagonal inside the window must
#: stay under this, or the check cannot tell a real platform offset from the
#: platform having moved mid-window -- refuses to evaluate, never a silent
#: pass (plan § 1: "measure ... platform at rest").
_FRAME_CHECK_REST_TOL_MM = 1.0
#: Minimum Platform mocap samples inside the window before the mean offset
#: is trusted -- one noisy sample is not a session-start measurement.
#: `rigid_body_poses` streams at `hw.TRACKING_MOCAP_DT_S` = 5 ms (200 Hz), so
#: a healthy 1.0 s window carries ~200 samples; 20 is a generous floor under
#: drops.
_FRAME_CHECK_MIN_SAMPLES = 20

#: the two-site pattern parameter defaults: apex 0.9 m and dwell 0.30 s are the
#: owner's R2 operating point (2026-09-12); separation 250 mm is the owner's
#: R4 target (plan § 0, 2026-09-16, BallButler experience; decision D2
#: 2026-09-23: "sweep 250 first, fly 250 first") and the ONLY separation the
#: hop boxes in config/generated/admissible_box.yaml are swept at — the GUI
#: relay sends 0 for every numeric goal field ("use the node's default"), so
#: this constant IS the separation a GUI-started hop flies.  A 100 mm hop has
#: no box and is refused NO_ADMISSIBLE_COMMAND before anything moves.
_DEFAULT_APEX_M = 0.9
_DEFAULT_SEPARATION_MM = 250.0
_DEFAULT_DWELL_S = 0.30
_DEFAULT_N_THROWS = 4

#: The columns schedule's first skill installs this long after a columns
#: Juggle goal is accepted — one second of margin for the operator's own dispatch latency,
#: not a physical constant of the pattern.
_START_LEAD_S = 1.0

#: the self-toss-pattern parameter defaults (R3 owner decisions,
#: brief_common.md § "Owner decisions"): one site, P1 = (-50, 0) mm — the SAME
#: point `columns_sites(100.0)`'s P1 names, so a box swept for site pair
#: ('P1', 'P1') at that xy (`config/generated/admissible_box.yaml`) matches
#: without a second geometry definition.
_DEFAULT_SITE_X_MM = -50.0
_DEFAULT_SITE_Y_MM = 0.0
_DEFAULT_SITE_NAME = 'P1'
#: A fresh id starts a cold memory (plan § 2.2) — `memory_path`'s own contract.
_DEFAULT_PLANT_ID = 'jugglebot'

#: R4 reload (owner decision D4, brief step 1b): the `bb/throw_at_target`
#: `throw_delay_s` this node asks for is floored here, never below it —
#: restated (not imported, node-boundary discipline) from
#: `ball_butler_node._DEFAULT_THROW_DELAY_S` = 2.5 s, BB's own pitch-axis
#: settle-time floor (that node's own docstring: a 60 deg pitch change takes
#: ~1.6 s of motion + ~0.5 s margin, and firing before it settles releases at
#: the wrong angle). In practice the opening REST + pre-tilt margin below
#: always clears this (the opening REST alone is >= `schedule.FLOOR_LIFT_S`
#: = 1.5 s), so the floor is a documented backstop, not what usually binds.
_BB_THROW_DELAY_FLOOR_S = 2.5

#: R4 reload: margin (s) added past BB's own reported `throw_delay_s +
#: predicted_tof_s` before `_on_tick` gives up waiting for the
#: `ThrowAnnouncement` and fails closed (`ABORTED_NO_ANNOUNCEMENT`, brief
#: step 1d). Covers scheduling/publish jitter on both ends of the round
#: trip — this node's own tick cadence (`1 / _TICK_HZ` = 0.025 s) is two
#: orders inside it, so 1.0 s is pure headroom, not a measured bound.
RELOAD_ANNOUNCE_TIMEOUT_S = 1.0

#: R4 reload C1 (2026-09-28, the 2026-09-27 22:37 sitting): how long
#: `_start_reload` waits for Ball Butler's OWN heartbeat to report ready
#: (IDLE, ball in hand, connected) before giving up. The sitting measured
#: `bb/reload`'s own `requestCheckBall()` -> CHECKING_BALL cycle at ~1.1 s
#: (22:47:28.84 -> 29.94); the old code asked to throw 5 ms after `bb/reload`
#: answered and got `THROW_REJECTED_BAD_STATE` on every attempt (the
#: firmware accepts a throw only from IDLE/TRACKING) -- this floor is
#: ~2.7x that measurement, not a guess.
RELOAD_BB_READY_TIMEOUT_S = 3.0

#: R4 reload, 2026-09-28 21:41 sitting: `bb/reload` is sent ONLY when Ball
#: Butler's own heartbeat says its hand is EMPTY (owner, 2026-09-28: the
#: firmware's RELOAD_CMD runs `requestCheckBall()`, a ~1.0-1.1 s CHECKING_BALL
#: cycle measured on every reload of both R4 sittings, and a ball already in
#: hand needs no check -- BB can feed as soon as its axes are in position).
#: When it IS sent, BB must physically fetch a ball, which no bag has ever
#: recorded (every 2026-09 reload found the ball already in hand), so this
#: wait is a ceiling, NOT a measurement -- it only lengthens how long the
#: platform rests level under the aim point, the safe place to wait.
RELOAD_BB_FETCH_TIMEOUT_S = 10.0

#: A Ball Butler heartbeat older than this is not evidence of BB's state (the
#: bridge publishes it at 10 Hz, measured 2026-09-28: 0.100 s median period),
#: so neither the "ball already in hand, skip bb/reload" decision nor the
#: "ready, fire" gate may read it.
_BB_HEARTBEAT_FRESH_S = 0.5

#: R4 reload C1: a fixed ceiling on Ball Butler's own predicted flight time,
#: for the `await_announcement` deadline `_fire_reload_throw` arms BEFORE
#: (not after, C2) the `bb/throw_at_target` round trip -- the real
#: `predicted_tof_s` is not known until that call returns, and the deadline
#: must already be armed before it is made. Every reload throw targets the
#: SAME catch cup from the SAME Ball Butler mount, so its ToF is bounded by
#: BB's own reachable-workspace geometry, not a per-call unknown -- test
#: fixtures and the 2026-09-27 sitting both measured ~0.7 s.
_RELOAD_TOF_CEILING_S = 1.5


def _wire_tilt_out(tilt):
    """``segments.CatchTerminal.hold_tilt`` / ``RestTerminal.tilt`` ->
    ``(is_set, [rx, ry])`` for ``InstallSegment.Request.{hold,rest}_tilt_set``
    / ``*_tilt_rad`` (R4 reload, Unit U3; the flag since the R4 audit,
    2026-09-24): ``None`` -> ``(False, [nan, nan])`` — the FLAG is the "no
    tilt" answer (rosidl zero-initialises the array, and a REST's own
    ``(0.0, 0.0)`` is a real target), the NaN is belt and braces. See
    ``trajectory_node._wire_tilt``, the one decode point."""
    if tilt is None:
        return False, [float('nan'), float('nan')]
    return True, [float(tilt[0]), float(tilt[1])]


def _mocap_aim_point_mm(site: Site, z_mm: float, offset_mm):
    """The inverse of `SkillNode._on_balls`'s subtraction (R4 reload, brief
    step 1b): a tracker LANDING has the measured mocap-to-schedule offset
    SUBTRACTED to reach the schedule frame, so an OUTGOING aim point —
    ``bb/throw_at_target``'s ``target_point_global_mm``, which Ball Butler
    resolves against the mocap graph — needs the SAME offset ADDED back.
    ``offset_mm`` is ``SkillNode._mocap_to_schedule_mm`` (``None`` before the
    first evaluable frame check — treated as ``(0, 0)``, the same fallback
    `_on_balls` uses)."""
    dx, dy = (0.0, 0.0) if offset_mm is None else (float(offset_mm[0]),
                                                    float(offset_mm[1]))
    return (float(site.cup_mm[0]) + dx, float(site.cup_mm[1]) + dy,
           float(z_mm))


@dataclasses.dataclass(frozen=True)
class FrameCheckResult:
    """The session-start mocap-Platform-vs-commanded-position offset (plan
    `cup-contact-contract.md` § 1) — the pure half of `SkillNode._frame_check`.

    ``evaluable=False`` means the check itself could not run; ``detail``
    names which input, and ``offset_mm``/``dx_mm``/``dy_mm`` are 0.0 and MUST
    NOT be read as "no offset", only as "unknown" — the caller treats
    cannot-evaluate as a refusal exactly like over-limit, never as a silent
    pass (fail-closed, plan § 1). ``evaluable=True`` means the three offset
    fields are real measurements and ``within_limit`` says whether
    ``offset_mm`` is at or under `_FRAME_CHECK_LIMIT_MM`. ``detail`` is
    always one complete, loggable sentence either way — the node logs it on
    every check, refusal or not (plan § 1: "always log the offset").
    ``plat_spread_mm`` is the Platform body's bounding-box diagonal inside
    the window (0.0 when not evaluable) — the stability the subtraction in
    `SkillNode._on_balls` rests on."""
    evaluable: bool
    within_limit: bool
    offset_mm: float
    dx_mm: float
    dy_mm: float
    detail: str
    plat_spread_mm: float = 0.0


def _frame_offset_check(platform_samples, commanded_samples, now, *,
                        window_s: float = _FRAME_CHECK_WINDOW_S,
                        limit_mm: float = _FRAME_CHECK_LIMIT_MM,
                        rest_tol_mm: float = _FRAME_CHECK_REST_TOL_MM,
                        min_samples: int = _FRAME_CHECK_MIN_SAMPLES,
                        plat_spread_tol_mm: float = _FRAME_CHECK_PLAT_SPREAD_MM
                        ) -> FrameCheckResult:
    """Pure: the mocap-Platform-vs-commanded xy offset over the last
    ``window_s`` (plan `cup-contact-contract.md` § 1; see the frame/units
    note on `_FRAME_CHECK_WINDOW_S` above this module's `SkillNode` class —
    x/y subtract directly, no transform).

    ``platform_samples`` / ``commanded_samples``: iterables of
    ``(mono_s, x_mm, y_mm)``, any order, on the SAME monotonic clock as
    ``now`` (`time.perf_counter()` in `SkillNode` — keeping them on one clock
    is the caller's job, not this function's). No ROS, no node state:
    `SkillNode._frame_check` is the only caller and owns the two buffers this
    reads.

    Refuses to evaluate (``evaluable=False``) rather than guess, on any of:
    no Platform sample ever seen; the freshest Platform sample older than
    ``window_s`` (the mocap graph, or that body specifically, went stale);
    fewer than ``min_samples`` Platform samples inside the window; no
    commanded-position sample ever seen or the freshest one older than
    ``window_s``; or the commanded position's bounding-box diagonal inside
    the window exceeding ``rest_tol_mm`` (the platform was still moving, so
    an "offset" measured against it is not the session-start number the
    contract means); or the Platform body's own bounding-box diagonal
    exceeding ``plat_spread_tol_mm`` (a body that moves or flickers inside
    the window cannot be subtracted from every landing — 2026-09-23)."""
    platform_samples = list(platform_samples)
    commanded_samples = list(commanded_samples)

    if not platform_samples:
        return FrameCheckResult(
            False, False, 0.0, 0.0, 0.0,
            'no mocap Platform body sample has ever been seen (check '
            '/rigid_body_poses carries a body named "Platform")')
    newest_plat_age_s = now - max(t for t, _, _ in platform_samples)
    if newest_plat_age_s > window_s:
        return FrameCheckResult(
            False, False, 0.0, 0.0, 0.0,
            'the mocap Platform body sample is stale (%.2f s old, window '
            '%.1f s)' % (newest_plat_age_s, window_s))
    plat_in_win = [(t, x, y) for t, x, y in platform_samples
                  if (now - t) <= window_s]
    if len(plat_in_win) < min_samples:
        return FrameCheckResult(
            False, False, 0.0, 0.0, 0.0,
            'only %d mocap Platform samples in the %.1f s window (need >= '
            '%d)' % (len(plat_in_win), window_s, min_samples))

    if not commanded_samples:
        return FrameCheckResult(
            False, False, 0.0, 0.0, 0.0,
            'no trajectory/commanded_position sample has ever been seen')
    newest_cmd_age_s = now - max(t for t, _, _ in commanded_samples)
    if newest_cmd_age_s > window_s:
        return FrameCheckResult(
            False, False, 0.0, 0.0, 0.0,
            'trajectory/commanded_position is stale (%.2f s old, window '
            '%.1f s)' % (newest_cmd_age_s, window_s))
    cmd_in_win = [(t, x, y) for t, x, y in commanded_samples
                 if (now - t) <= window_s]

    cmd_xs = [x for _, x, _ in cmd_in_win]
    cmd_ys = [y for _, _, y in cmd_in_win]
    cmd_spread_mm = math.hypot(max(cmd_xs) - min(cmd_xs),
                               max(cmd_ys) - min(cmd_ys))
    if cmd_spread_mm > rest_tol_mm:
        return FrameCheckResult(
            False, False, 0.0, 0.0, 0.0,
            'the commanded position moved %.2f mm inside the %.1f s window '
            '(limit %.1f mm) — not at rest' % (cmd_spread_mm, window_s,
                                               rest_tol_mm))

    plat_xs = [x for _, x, _ in plat_in_win]
    plat_ys = [y for _, _, y in plat_in_win]
    plat_spread_mm = math.hypot(max(plat_xs) - min(plat_xs),
                                max(plat_ys) - min(plat_ys))
    if plat_spread_mm > plat_spread_tol_mm:
        return FrameCheckResult(
            False, False, 0.0, 0.0, 0.0,
            'the mocap Platform body moved %.2f mm inside the %.1f s window '
            '(limit %.1f mm) — not a stable session-start offset (platform '
            'still moving, or the body is mis-tracked)'
            % (plat_spread_mm, window_s, plat_spread_tol_mm), plat_spread_mm)

    plat_x = sum(plat_xs) / len(plat_xs)
    plat_y = sum(plat_ys) / len(plat_ys)
    cmd_x = sum(cmd_xs) / len(cmd_xs)
    cmd_y = sum(cmd_ys) / len(cmd_ys)
    dx_mm = plat_x - cmd_x
    dy_mm = plat_y - cmd_y
    offset_mm = math.hypot(dx_mm, dy_mm)
    detail = ('mocap Platform is %+.1f mm (x %+.1f, y %+.1f) from the '
              'commanded position over %.1f s, body spread %.2f mm '
              '(limit %.1f mm)'
              % (offset_mm, dx_mm, dy_mm, window_s, plat_spread_mm, limit_mm))
    return FrameCheckResult(True, offset_mm <= limit_mm, offset_mm, dx_mm,
                            dy_mm, detail, plat_spread_mm)


class SkillNode(Node):
    """Schedule-driven skill dispatch shell (plan § 2.4)."""

    def __init__(self, robot_name: str = _DEFAULT_PLANT_ID):
        super().__init__('skill_node')

        self._robot_name = robot_name
        self._balls = {}  # ball_id (int) -> Landing, the tracker `SkillExecutor` reads
        # The latest `/balls` message's raw ball records (id/status/destination/
        # tracking), for ball-identity correlation (`_advance_correlation`) —
        # `_balls` above is the LANDING projection `latch_announced_ball` cannot
        # use (it needs `status`/`destination`/`tracking`, not just a landing).
        self._raw_balls = []
        # SCHEDULE ball_id -> a tuple of `ball_possession.FlightLatch`, ONE
        # PER ANNOUNCED RELEASE (never one per ball — see `_maybe_announce`).
        # `/balls` ids are the TRACKER's own; the schedule/executor speak in
        # schedule ball ids, and this is the one translation. Advanced by
        # `_advance_correlation`, read by `_tracker` through
        # `ball_possession.flight_in_progress`.
        self._correlation = {}
        # SCHEDULE ball_id -> the absolute wall-clock instant of the last
        # release announced for it — `_maybe_announce`'s de-duplication key
        # for a CATCH re-send (see that method's docstring).
        self._announced_release_s = {}
        # The hand's live possession evidence (ball_possession.EVIDENCE_*) —
        # the R3 ladder's `ball_evidence` field (item 6) AND outcome capture's
        # `observer` read (`_ball_evidence`), one value, one writer
        # (`_on_hand_telemetry`).
        self._possession_evidence = ball_possession.EVIDENCE_UNKNOWN
        # Debounced twin for the ladder's SEATED precondition (see _on_hand_telemetry).
        self._possession_evidence_stable = ball_possession.EVIDENCE_UNKNOWN
        self._executor = None  # a live SkillExecutor, or None between attempts
        # The last ACCEPTED install's future event (a THROW's release, or a
        # CATCH-with-throw's carried release) on the schedule's own wall
        # clock -- 0.0 when the streaming plan carries none (a plain CATCH,
        # a REST). Updated on every ACCEPTED install (`_installer`), read
        # and cleared by `_maybe_hold_pending_event` (Unit A, R3 first
        # sitting, 2026-09-13, L1): an ended attempt does not stop a
        # streaming release just because it ended -- every segment is
        # rest-terminal, not event-free.
        self._pending_event_mono = 0.0
        # Guards `_pending_event_mono`'s check-and-clear only (never the hold
        # round trip): `_stop_attempt` calls `_maybe_hold_pending_event` OUTSIDE
        # `_tick_lock` while `_on_tick` may be inside it on another thread.
        self._pending_lock = threading.Lock()
        # Guards `_correlation`'s read-modify-write. `_on_balls` (the
        # subscription group) and `_maybe_announce` (the tick, via
        # `_installer`) both rebind the latch queue and run on DIFFERENT
        # callback groups, so a concurrent announce could otherwise lose its
        # own latch — and a lost latch means the next flight is never
        # correlated at all. `_tracker` only reads the tuple and needs no lock.
        self._correlation_lock = threading.Lock()
        # The latest `trajectory/status`, for the self-toss/hop Juggle goal's
        # limits check (`_run_one_ball`) and the R3 ladder's
        # `levelled`/`in_trajectory_mode` fields (`_observations`, item 6).
        self._traj_status = TrajectoryStatus()
        # Perf-clock arrival stamps for the R3 ladder's freshness checks (item
        # 6) — one stamp per cached message, same monotonic domain
        # (`time.perf_counter()`) `reload_coordinator_node` uses for the
        # identical purpose (that module's `_*_mono` caches).
        self._traj_status_mono = 0.0
        self._mocap_mono = 0.0
        self._hand_telemetry_mono = 0.0
        self._hand_pos_meas = 0.0
        self._hand_pos_cmd = 0.0
        # The hand's launch-speed ratio source for `catch_aim_source` =
        # `schedule_hand` (`hand_launch.HandLaunchMonitor`): fed every
        # `/hand_telemetry` sample, read once per catch by the executor. Held
        # for the node's life (not per attempt) — a stroke's samples arrive
        # before the catch that asks about them, and the monitor's own
        # history bound is what keeps it small.
        self._hand_launch = HandLaunchMonitor()
        # `trajectory/commanded_position` — the pre-level move's own xy/z
        # (item 7; `_prelevel`), restated from
        # `reload_coordinator_node._on_commanded_position` /
        # `_live_commanded_position`'s cache-plus-freshness shape.
        self._commanded_pos_mm = None
        self._commanded_pos_mono = 0.0
        # Session-start mocap-vs-commanded frame check (`_frame_check`, plan
        # `cup-contact-contract.md` § 1): bounded, time-stamped
        # ``(mono_s, x_mm, y_mm)`` buffers of the mocap `Platform` body and
        # the commanded position, fed by `_on_mocap` / `_on_commanded_position`
        # respectively. `maxlen` bounds worst-case memory — no allocation
        # growth once full (`collections.deque`'s own contract) — sized well
        # above each topic's nominal rate over `_FRAME_CHECK_WINDOW_S`:
        # `rigid_body_poses` at `hw.TRACKING_MOCAP_DT_S` (200 Hz) and
        # `commanded_position` on the 5 Hz status timer.
        self._platform_mocap_xy = collections.deque(maxlen=400)
        self._commanded_xy_hist = collections.deque(maxlen=40)
        # The measured mocap-to-schedule offset ``(dx_mm, dy_mm)`` the last
        # evaluable frame check produced (`_frame_check_error`), or ``None``
        # when none has been measured yet: `_on_balls` subtracts it from every
        # tracker landing's xy, so a `Landing` the executor reads is already
        # "relative to where the cup really is" (see the frame-offset note on
        # `_FRAME_CHECK_WINDOW_S`). Never zeroed on a cannot-evaluate result
        # -- the last good measurement is better than none -- only replaced
        # by a fresh evaluable one.
        self._mocap_to_schedule_mm = None
        # `/link_status`'s `sched_refused` — the FIRMWARE's count of streamed
        # lanes it refused and is now HOLDING (`_on_link_status`), and the
        # value latched when the running attempt started. `hand_lane_refused`
        # is `live > baseline`: a counter that was already non-zero at the
        # start (a previous attempt's refusal) is history, not this attempt's.
        # -1 = never observed, which reads as "no refusal" and so keeps the
        # pre-2026-09-18 behaviour on a bridge that does not publish the key.
        self._sched_refused = -1
        self._sched_refused_mono = 0.0
        self._sched_refused_at_start = -1
        # Set by `_on_tick` when an END needs the machine HELD rather than
        # left to its rest tail (`executor.HAND_LANE_REFUSED`), cleared by
        # `_maybe_hold_pending_event` when it fires the hold — so the hold is
        # fired exactly once per attempt, not once per tick.
        self._force_hold = False
        # Which END code has already ARMED that hold, so the arming itself is
        # once per attempt too ('' = none; reset with the attempt).
        self._hold_forced_code = ''
        # R4 reload (C1, 2026-09-28 rework of brief step 1): the reload
        # context spanning the WHOLE reload wait — `None` outside one, else
        # a `SimpleNamespace(phase=..., pattern=..., boxes=..., memory=...,
        # deadline_mono=..., site=..., bridge=...)`.  Three phases, one
        # object mutated in place (never replaced — `is` identity checks
        # elsewhere rely on it):
        #   'await_bb_idle'     — armed by `_start_reload` once `bb/reload`
        #                         is accepted; `_maybe_fire_reload_throw`
        #                         (the tick) fires `bb/throw_at_target` the
        #                         moment BB's own heartbeat reports ready
        #                         (IDLE, ball in hand, connected) — NOT 5 ms
        #                         after `bb/reload`, which the 2026-09-27
        #                         22:37 sitting measured racing BB's own
        #                         ~1.1 s CHECKING_BALL cycle on every
        #                         attempt (`THROW_REJECTED_BAD_STATE` x5).
        #   'firing'            — `_maybe_fire_reload_throw` has started the
        #                         (blocking) `bb/throw_at_target` round trip;
        #                         a concurrent tick (this node's callback
        #                         group is reentrant) must not start a
        #                         second one.
        #   'await_announcement'— armed BEFORE `bb/throw_at_target` is
        #                         invoked (C2), so an announcement delivered
        #                         while that call is still in flight is
        #                         accepted, not read as "no reload awaiting
        #                         one" (the OLD code armed AFTER the call
        #                         returned and dropped exactly this).
        # Cleared by whichever of `_on_announcement` / `_check_reload_timeout`
        # / `_check_reload_lane_refused` / `_on_bb_throw_outcome` ends the
        # wait first. The bridge REST's own `SkillExecutor` routinely
        # retires itself (`_executor` -> `None`) WHILE this is still armed —
        # a one-skill schedule is "done" (fully dispatched) the instant it
        # is sent to `trajectory/install_segment`, long before it has
        # physically settled (`compile_reload_wait`'s only skill measured
        # 6.42 s of hand-homing in the field) — this is the state
        # `_refuse_if_running` reads so a second start cannot slip in during
        # that window, and the state `_check_reload_lane_refused` watches
        # `hand_lane_refused` against once the executor that would
        # ordinarily have caught it is already gone (C5b: the 2026-09-27
        # sitting's guard latch fired with NO `HAND_LANE_REFUSED` END ever
        # logged, because nothing was left ticking to produce one). Guarded
        # by `_reload_lock` (read on the tick's 40 Hz cadence, written from
        # the service thread, the tick, and the subscription's own callback
        # group).
        self._reload_ctx = None
        self._reload_lock = threading.Lock()
        # True from the moment `_on_announcement` swaps the reload schedule
        # in until that attempt ends (C3 widening, 2026-09-28): Ball
        # Butler's ThrowAnnouncement is published at DISPATCH (its Teensy's
        # single CMD_RESULT is terminal — `OK` only once the ball has been
        # released, `ABORTED_NOT_SETTLED` if it aborts at fire — so nothing
        # earlier could carry it), which means a schedule can already be
        # compiled for a throw the firmware then refuses or aborts. The
        # outcome relay is the only thing that can tell us the ball is not
        # coming; this flag keeps `_on_bb_throw_outcome` listening after the
        # ctx was popped. Cleared with the attempt (`_end_attempt`, the
        # tick's `done` branch) and re-armed by `_start_reload`.
        self._reload_throw_inflight = False
        # Ball Butler's own latest heartbeat (C1) — cached here, never
        # blocking: `_maybe_fire_reload_throw` reads it every tick, and
        # `_check_reload_timeout`'s `ABORTED_BB_NOT_READY` names whatever it
        # last saw. 0 = `BallButlerStates.BOOT`, the state before any
        # heartbeat has ever arrived — never "ready" by construction.
        self._bb_state = 0
        self._bb_ball_in_hand = False
        self._bb_connected = False
        self._bb_heartbeat_mono = 0.0

        # The Juggle action's own bookkeeping (R4 U5). `_goal_done_event` is
        # set by `_on_tick` the instant nothing is left running for the goal
        # in progress (`self._executor is None and self._reload_ctx is
        # None` — NOT merely `self._executor is None`, which is also true
        # mid-reload-wait between the bridge REST finishing and the real
        # reload schedule swapping in; see `_reload_ctx`'s comment above);
        # `_juggle_execute` waits on it instead of polling `self._executor`
        # directly, so it can never observe the bridge-wait gap as "done".
        # `_goal_end_code` is the last `SkillExecutor.end_code` the current
        # goal's attempt set, captured the moment `_on_tick` nulls
        # `self._executor` (the code is gone once that happens) — `''` means
        # the attempt ran to its own natural end (COMPLETED, plan § 0's
        # rest-terminal contract: nothing needs to "end" a schedule that
        # simply ran out). `_goal_outcome_lines` accumulates the executor's
        # own OUTCOME lines (`tick()`'s return) across the whole attempt, for
        # the goal result's `per_throw`/`throws`/`caught`. All three are
        # reset by `_start_pattern` at the start of every accepted goal.
        self._goal_done_event = threading.Event()
        self._goal_end_code = ''
        self._goal_outcome_lines = []

        self.declare_parameter('apex_m', _DEFAULT_APEX_M)
        self.declare_parameter('separation_mm', _DEFAULT_SEPARATION_MM)
        self.declare_parameter('dwell_s', _DEFAULT_DWELL_S)
        self.declare_parameter('n_throws', _DEFAULT_N_THROWS)
        self.declare_parameter('site_x_mm', _DEFAULT_SITE_X_MM)
        self.declare_parameter('site_y_mm', _DEFAULT_SITE_Y_MM)
        self.declare_parameter('plant_id', _DEFAULT_PLANT_ID)
        # Where a CATCH's aim comes from (owner decision 2026-09-18). The
        # LIVE default is `tracker`: the catch is aimed at the tracker's
        # CONVERGED ballistic fit, with the schedule's commanded landing as
        # the prior at dispatch, and no step of that order waits (the
        # 2026-09-15 NO_LANDING sitting was an aim that DEPENDED on a mocap
        # marker; since `ce6d603` the tracker confirms every flight, 22/22 on
        # 2026-09-17, and the release slips 0.019-0.137 s from its knot,
        # which only an observation can see). `schedule` and `schedule_hand`
        # stay selectable as the open-loop A/B arm — see
        # `executor.AIM_SOURCES` and `executor._catch_aim`.
        self.declare_parameter('catch_aim_source', ex.AIM_TRACKER)
        # Owner 2026-09-16 pinned this at 0 (the learner corrects FLIGHT
        # only) until the planner's small-lateral-offset banking defect was
        # fixed; owner 2026-09-21 lifted the pin once the cup-contact
        # contract landed banking that is amplitude-aware rather than a
        # scale-free 12 degree clamp (plans/archived/cup-contact-contract.md
        # § 6): the box admits +/-40 mm at 0.9 m, and the session-start
        # mocap-vs-commanded frame check (`_frame_check_error`) is what now
        # guards a live authority against an unverified mocap frame. mm per
        # axis, applied by SkillExecutor.lateral_authority_m; `:=0` re-pins
        # it for a sitting or a bench check. Owner 2026-09-23, after the
        # first unpinned sitting (Block B): 40 mm dropped balls on the 25-throw
        # chains at 0.9 m (the learner's y command reached +29 mm against a
        # 17.6 mm plant scatter — two drops in 51), 20 mm was stable at both
        # apexes — 20 mm is the launch default; the box still admits 40.
        self.declare_parameter('learner_lateral_authority_mm', 20.0)
        # How many re-solves one committed CATCH may spend re-aiming from later
        # fits (executor.resend_max_per_catch; 2 since 2026-09-18). 0 disables
        # re-aiming outright -- the A/B knob for a sitting: on 2026-09-18 16:16
        # 18 of 21 timing-only re-sends were refused LIMIT_JERK, each a solve.
        self.declare_parameter('catch_resend_max', 2)

        # ── the install client + tick timer share ONE reentrant group ──────
        # Fixed 2026-09-13 (found by reading, never exercised live): `main`
        # used to run a plain single-threaded `rclpy.spin`, and `_installer`
        # blocks the tick callback in `_wait_future`'s poll loop waiting for
        # the SAME node's client response — a response only the executor can
        # ever deliver. Under a single-threaded spin nothing is left to
        # process that response while the tick callback is busy-waiting for
        # it: every install times out, forever. Mirrors
        # `reload_coordinator_node`'s `_hold_cbg` / `trajectory_node`'s
        # `_hold_cbg` narrow-reentrant pattern — here the WHOLE install path
        # (client + timer) moves together, because the tick callback and the
        # client response are the two halves of the one blocking call that
        # needed unblocking, not two independent throughput concerns.
        self._cbgroup = ReentrantCallbackGroup()
        # Guards against the tick RE-ENTERING itself: a `ReentrantCallbackGroup`
        # lets the executor run the SAME timer callback again on another
        # thread while the first invocation is still blocked inside
        # `_wait_future` — that would start a second dispatch on top of one
        # already in flight. `acquire(blocking=False)` in `_on_tick` makes a
        # still-busy tick a no-op tick rather than a second install.
        self._tick_lock = threading.Lock()

        self._install_cli = self.create_client(
            InstallSegment, 'trajectory/install_segment',
            callback_group=self._cbgroup)
        # Shares the SAME reentrant group as the install client (item 7,
        # `_prelevel`): `_run_one_ball` runs on the node's default
        # (non-reentrant) service group and blocks in `_wait_future` waiting
        # for THIS client's response, exactly the shape `__init__`'s comment
        # above documents for the install client — a different group is what
        # lets the MultiThreadedExecutor process that response concurrently.
        self._go_to_pose_cli = self.create_client(
            GoToPose, 'trajectory/go_to_pose', callback_group=self._cbgroup)
        # Shares the SAME reentrant group too (Unit A, R3 first sitting,
        # 2026-09-13): `_maybe_hold_pending_event` blocks in `_wait_future`
        # for THIS client's response, called from `_on_tick` (already on the
        # group) and from `_stop_attempt` (the default group) -- the response
        # still needs the MultiThreadedExecutor to process it concurrently.
        self._hold_cli = self.create_client(
            Trigger, 'trajectory/hold', callback_group=self._cbgroup)
        # R4 reload (brief step 1b): the SAME reason every other blocking
        # client on this node shares `self._cbgroup` (see the comment on
        # `_install_cli` above) — `_start_reload` blocks in `_wait_future`
        # for both, from the self-toss/hop Juggle goal's own default
        # (non-reentrant) group.
        self._reload_cli = self.create_client(
            Trigger, 'bb/reload', callback_group=self._cbgroup)
        self._bb_throw_cli = self.create_client(
            BallButlerThrow, 'bb/throw_at_target',
            callback_group=self._cbgroup)

        self._announce_pub = self.create_publisher(
            ThrowAnnouncement, 'throw_announcements', 10)

        # ── every subscription gets its OWN callback group (finding 4, R3
        # audit, 2026-09-13) ────────────────────────────────────────────────
        # All subscriptions used to share the node's default MutuallyExclusive
        # group. `_run_one_ball` blocks in `_prelevel` (a `go_to_pose`
        # round trip whose wait can run past a second) on the SERVICE's own
        # default group, and rclpy Foxy's executor yields ready timers before
        # subscriptions -- so while `_prelevel` blocks, no subscription
        # callback runs, and the first `_on_tick` after it can read a
        # `trajectory/status` sample older than `_TRAJ_STATUS_STALE_S`,
        # aborting `ABORTED_MODE_CHANGED` before the opening REST even
        # dispatches. A dedicated non-default group per subscription (plus the
        # `MultiThreadedExecutor(num_threads=3)` in `main` below) lets them run
        # concurrently with a blocked service call instead of queuing behind
        # it.
        self._sub_cbgroup = MutuallyExclusiveCallbackGroup()
        self.create_subscription(BallStateArray, 'balls', self._on_balls, 10,
                                 callback_group=self._sub_cbgroup)
        self.create_subscription(
            HandTelemetryMessage, 'hand_telemetry', self._on_hand_telemetry, 10,
            callback_group=self._sub_cbgroup)
        self.create_subscription(
            TrajectoryStatus, 'trajectory/status', self._on_traj_status, 10,
            callback_group=self._sub_cbgroup)
        self.create_subscription(
            RigidBodyPoses, 'rigid_body_poses', self._on_mocap, 10,
            callback_group=self._sub_cbgroup)
        self.create_subscription(
            Point, 'trajectory/commanded_position',
            self._on_commanded_position, 10, callback_group=self._sub_cbgroup)
        # The bridge's own 10 Hz link diagnostic, for ONE key: `sched_refused`,
        # the firmware's count of streamed lanes it REFUSED and is now holding
        # (`executor.HAND_LANE_REFUSED`). Read HERE rather than routed through
        # `trajectory/status`: the counter is a BRIDGE fact that only this
        # node acts on, the decider is one hop from the publisher instead of
        # two (10 Hz, not the status timer's 5), and it needs no new field on
        # a typed interface — so a branch that flies tonight needs no
        # `jugglebot_interfaces` rebuild to be safe.
        self.create_subscription(
            DiagnosticStatus, 'link_status', self._on_link_status, 10,
            callback_group=self._sub_cbgroup)
        # R4 reload (brief step 1c): this node only ever PUBLISHED on
        # `throw_announcements` before (`_maybe_announce`, our own throws) —
        # this is its one SUBSCRIBER, for Ball Butler's announcement of a
        # reload throw. Own callback group, same reason as every other
        # subscription here (finding 4, R3 audit): `_on_announcement` must
        # run while `_start_reload` is blocked in a `bb/*` round trip on the
        # service's default group.
        self.create_subscription(
            ThrowAnnouncement, 'throw_announcements', self._on_announcement,
            10, callback_group=self._sub_cbgroup)
        # R4 reload C1 (2026-09-28): Ball Butler's own state, cached (never
        # blocking, same reason every other subscription here is on its own
        # group) for `_maybe_fire_reload_throw` to read every tick.
        self.create_subscription(
            BallButlerHeartbeat, 'bb/heartbeat', self._on_bb_heartbeat, 10,
            callback_group=self._sub_cbgroup)
        # R4 reload C3 (2026-09-28): the firmware's terminal CMD_RESULT for a
        # reload throw, relayed by `ball_butler_node._on_throw_result` — a
        # rejection ends the reload wait THIS tick instead of waiting out
        # `RELOAD_ANNOUNCE_TIMEOUT_S` for an announcement that a rejected
        # throw will never produce (the 22:37 sitting waited 4.6 s for
        # exactly this).
        self.create_subscription(
            String, 'bb/throw_outcome', self._on_bb_throw_outcome, 10,
            callback_group=self._sub_cbgroup)

        self.create_service(Trigger, 'skills/check', self._svc_check)

        # The Juggle action's OWN callback group (R4 owner decision D3, U5):
        # a goal accept (`_juggle_goal` -> `_start_pattern`) and an accepted
        # goal's `_juggle_execute` both block in `_wait_future` round trips
        # on the SAME clients the tick shares `self._cbgroup` with (see that
        # group's own comment above) -- a SEPARATE reentrant group (not
        # `self._cbgroup` itself) is what lets a goal accept still run --
        # and reject a concurrent second goal -- while an already-accepted
        # goal's execute callback is mid-wait on another thread. `main`'s
        # `MultiThreadedExecutor` grew a 4th thread for it (see that
        # function's comment).
        self._juggle_cbgroup = ReentrantCallbackGroup()
        self._juggle_action = ActionServer(
            self, Juggle, 'jugglebot/juggle',
            execute_callback=self._juggle_execute,
            goal_callback=self._juggle_goal,
            cancel_callback=self._juggle_cancel,
            callback_group=self._juggle_cbgroup)

        self.create_timer(1.0 / _TICK_HZ, self._on_tick,
                          callback_group=self._cbgroup)

        # One INFO line at start-up (`blas threads: N (source)`, WARN if the
        # pool is uncapped) — the runsheet's row 13 greps for it, and the
        # launch file's `_planner_blas_env` cap is otherwise invisible here.
        self._blas_threads, self._blas_source = blas_threads.check_blas_threads(
            self.get_logger(), 'skill_node')
        self.get_logger().info('skill_node ready')

    # ── tracking ──────────────────────────────────────────────────────────

    def _on_balls(self, msg):
        """Cache ``/balls`` landings, keyed by ball id, for the executor's
        tracker callable. Mirrors `catch_coordinator_node._msg_to_ball`'s
        field reads.

        THE one point where the tracker's (mocap) frame becomes the
        schedule's: the session-start offset `_mocap_to_schedule_mm` is
        subtracted from the landing xy here, so the executor's catch aim and
        the learner's outcome row both see a landing relative to the cup's
        real position (frame-offset note on `_FRAME_CHECK_WINDOW_S`). z and
        velocity are untouched (the offset is lateral, plan § 1)."""
        self._raw_balls = list(msg.balls)
        off = self._mocap_to_schedule_mm
        dx, dy = (0.0, 0.0) if off is None else (float(off[0]), float(off[1]))
        for b in msg.balls:
            t_land = float(b.time_at_land.sec) + float(b.time_at_land.nanosec) * 1e-9
            self._balls[int(b.id)] = Landing(
                pos_mm=np.array([b.landing_position.x - dx,
                                 b.landing_position.y - dy,
                                 b.landing_position.z], dtype=float),
                vel_mm_s=np.array([b.landing_velocity.x, b.landing_velocity.y,
                                   b.landing_velocity.z], dtype=float),
                t_land_abs_s=t_land,
                from_fit=bool(b.landing_from_fit))
        self._advance_correlation()

    def _now_s(self) -> float:
        """Now, in ROS epoch seconds — the clock the announcements' own
        ``throw_time`` is on, and therefore the only clock a "has this release
        happened?" comparison may use (`_advance_correlation`, `_tracker`)."""
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _advance_correlation(self) -> None:
        """Refine every live schedule-ball-id -> tracker-id correlation
        against the latest ``/balls`` snapshot (item 3, plan § 2.7).

        ``jugglebot.ball_possession`` owns the RULE — `latch_announced_ball`
        per announced release (shared with `reload_coordinator_node`'s FSM
        latch, plan discipline: one rule, not a lookalike copy) and
        `advance_flight_latches` over the per-release queue; this method owns
        the per-schedule-ball-id state the rule threads through."""
        now_s = self._now_s()
        with self._correlation_lock:
            for ball_id, latches in list(self._correlation.items()):
                self._correlation[ball_id] = advance_flight_latches(
                    self._raw_balls, robot_name=self._robot_name,
                    latches=latches, now_s=now_s,
                    in_flight_status=_BALL_STATUS_IN_FLIGHT)

    def _tracker(self, ball_id: int):
        """The executor's tracker callable — keyed by SCHEDULE ball id.

        `/balls` ids are the tracker's OWN (item 3): a schedule ball id must
        first be correlated to one via `_maybe_announce` / `_advance_correlation`
        before a landing can be returned, and then only for the flight
        ACTUALLY IN PROGRESS — the latest announced release that has happened,
        whose id is both IN_FLIGHT and CONFIRMED (a raw IN_FLIGHT status alone
        is time-based and proves nothing — see `_BALL_TRACKING_CONFIRMED`;
        `ball_possession.flight_in_progress` holds the whole rule and the
        reason an earlier latch is never a fallback)."""
        latches = self._correlation.get(int(ball_id))
        if not latches:
            return None
        tracker_id = flight_in_progress(
            self._raw_balls, latches=latches, now_s=self._now_s(),
            in_flight_status=_BALL_STATUS_IN_FLIGHT,
            confirmed_tracking=_BALL_TRACKING_CONFIRMED)
        if tracker_id is None:
            return None
        return self._balls.get(int(tracker_id))

    def _catch_resend_max(self) -> int:
        """`catch_resend_max`, clamped to 0..5 with a WARN outside it -- a
        bound on a cost (solves per catch), never a policy, so an out-of-range
        value is clamped rather than refused."""
        raw = int(self.get_parameter('catch_resend_max').value)
        cap = min(max(raw, 0), 5)
        if cap != raw:
            self.get_logger().warn('catch_resend_max %d outside 0..5 -- using %d'
                                   % (raw, cap))
        return cap

    def _catch_aim_source(self) -> str:
        """The validated ``catch_aim_source`` parameter. An unknown value
        falls back to the LIVE default (:data:`executor.AIM_TRACKER`) with an
        error logged, rather than refusing the attempt: a typo in a launch
        override must not leave the operator with no catch at all, and the
        default is the only value a session can fly without having chosen
        it — falling back to the OTHER arm of an A/B would fly a policy the
        runsheet does not name."""
        value = str(self.get_parameter('catch_aim_source').value)
        if value not in ex.AIM_SOURCES:
            self.get_logger().error(
                'catch_aim_source=%r is not one of %s — using %r'
                % (value, list(ex.AIM_SOURCES), ex.AIM_TRACKER))
            return ex.AIM_TRACKER
        return value

    def _launch_ratio(self, ball_id: int, t_release_abs_s: float):
        """The executor's ``launch_ratio`` callable: the MEASURED hand
        launch-speed ratio of the stroke ending at ``t_release_abs_s``, or
        ``None`` when the hand telemetry cannot vouch for one (the executor
        then keeps the theoretical aim). ``ball_id`` is unused — this hand
        throws one ball at a time, and the release INSTANT is what selects
        the stroke."""
        r = self._hand_launch.ratio(t_release_abs_s)
        self.get_logger().info(
            'hand launch ratio for the release at %.3f (ball %d): %s'
            % (t_release_abs_s, ball_id, self._hand_launch.last_reason))
        return r

    def _on_traj_status(self, msg) -> None:
        """Cache the latest `trajectory/status` for `_run_one_ball`'s
        limits check and the R3 ladder's `levelled`/`in_trajectory_mode`
        fields (`_observations`, item 6). Perf-stamped like every other
        freshness-gated cache in this node."""
        self._traj_status = msg
        self._traj_status_mono = time.perf_counter()

    def _on_mocap(self, msg) -> None:
        """`rigid_body_poses` freshness (the R3 ladder's `mocap_fresh`, item
        6 — is the mocap graph publishing AT ALL) PLUS a bounded buffer of
        the `Platform` body's xy (plan `cup-contact-contract.md` § 1, the
        session-start frame check) — the one per-body read this node makes,
        mirroring `reload_coordinator_node`'s toss positioning cross-check
        (`_on_mocap` there): store AS PUBLISHED, x/y unconverted (see the
        frame/units note on `_FRAME_CHECK_WINDOW_S`)."""
        now = time.perf_counter()
        self._mocap_mono = now
        for body in msg.bodies:
            if body.name == 'Platform':
                p = body.pose.pose.position
                self._platform_mocap_xy.append((now, float(p.x), float(p.y)))
                break

    def _on_commanded_position(self, msg) -> None:
        """`trajectory/commanded_position`: the platform's live commanded
        (x, y, z) in STOW mm — restated from
        `reload_coordinator_node._on_commanded_position` (same NaN-drop
        discipline: a poisoned pose must never seed a pre-level move).
        `_prelevel` (item 7) is the only reader."""
        p = (float(msg.x), float(msg.y), float(msg.z))
        if not all(math.isfinite(v) for v in p):
            self.get_logger().error(
                'trajectory/commanded_position %r is non-finite — DISCARDED'
                % (p,))
            return
        now = time.perf_counter()
        self._commanded_pos_mm = p
        self._commanded_pos_mono = now
        # Session-start frame check's own at-rest window (plan
        # `cup-contact-contract.md` § 1) — xy only, same buffer shape as
        # `_platform_mocap_xy`.
        self._commanded_xy_hist.append((now, p[0], p[1]))

    def _live_commanded_position(self, now: float):
        """The live commanded platform (x, y, z) mm, or ``None`` when absent
        or stale — fail-closed, same shape as
        `reload_coordinator_node._live_commanded_position`."""
        pos = self._commanded_pos_mm
        mono = self._commanded_pos_mono
        if pos is None or mono <= 0.0 or (now - mono) >= _TRAJ_STATUS_STALE_S:
            return None
        return pos

    def _on_link_status(self, msg) -> None:
        """`/link_status` -> `sched_refused` only (2026-09-18).

        The bridge publishes the can-bridge heartbeat's ``sched_refused`` as a
        ``KeyValue`` at 10 Hz; it counts the times the firmware REFUSED a
        streamed lane's promotion and LATCHED the hold instead — after which
        the guard measures the refused command against the encoder and the
        deviation grows with every knot the plan walks on
        (``executor.HAND_LANE_REFUSED``, and ``leg_interp.cpp:497-520`` /
        ``:1226-1228`` for the firmware's side of it).

        Absent or unparseable key -> the cache is left UNOBSERVED (-1), which
        reads as "no refusal": the alternative (treating an unreadable
        diagnostic as a refusal) would end every attempt on a bridge that
        simply has an older wire format, and the value is defence in depth
        behind the lane SIZING (`_opening_rest_period`), not the only thing
        keeping the hand inside the envelope.
        """
        for kv in getattr(msg, 'values', ()):
            if getattr(kv, 'key', '') != 'sched_refused':
                continue
            try:
                self._sched_refused = int(float(kv.value))
            except (TypeError, ValueError):
                return
            self._sched_refused_mono = time.perf_counter()
            return

    def _hand_lane_refused(self, now: float) -> bool:
        """Has the firmware refused a streamed hand lane since this attempt
        started? ``_sched_refused`` past the latched baseline, on a FRESH
        sample. Staleness is not itself a refusal — a stale `/link_status`
        means the bridge stopped publishing, which the ladder's own
        `in_trajectory_mode`/`levelled` rows already end the attempt on one
        level up (both ride `trajectory/status`, whose own publisher needs the
        same graph)."""
        if self._sched_refused_at_start < 0 or self._sched_refused < 0:
            return False
        if (self._sched_refused_mono <= 0.0
                or (now - self._sched_refused_mono) >= _TRAJ_STATUS_STALE_S):
            return False
        return self._sched_refused > self._sched_refused_at_start

    def _on_bb_heartbeat(self, msg) -> None:
        """Cache Ball Butler's own state (R4 reload C1, 2026-09-28) —
        `_maybe_fire_reload_throw` reads it every tick to decide when to
        fire `bb/throw_at_target`, and `_check_reload_timeout` names it in
        `ABORTED_BB_NOT_READY` on a timeout. Never blocks (no service calls
        here): same reason every other subscription on `_sub_cbgroup` is a
        plain cache update."""
        self._bb_state = int(getattr(msg, 'state', 0))
        self._bb_ball_in_hand = bool(getattr(msg, 'ball_in_hand', False))
        self._bb_connected = bool(getattr(msg, 'connected', False))
        self._bb_heartbeat_mono = time.perf_counter()
        # A reload that sent `bb/reload` must see BB LEAVE IDLE before an IDLE
        # heartbeat counts as "ready": the 2026-09-28 21:41 sitting fired the
        # throw ~20 ms after `bb/reload` answered, on a heartbeat still
        # reporting the IDLE from BEFORE the ball check began (the check's
        # first CHECKING_BALL heartbeat arrived 73-121 ms later, bag
        # 2026-09-28_21-41-10). Plain attribute write on the live ctx; the
        # tick reads it under `_reload_lock`.
        with self._reload_lock:
            ctx = self._reload_ctx
            if (ctx is not None and ctx.phase == 'await_bb_idle'
                    and self._bb_state != int(proto.BallButlerStates.IDLE)):
                ctx.bb_left_idle = True

    def _bb_heartbeat_fresh(self) -> bool:
        """True when the cached Ball Butler heartbeat is recent enough to act on."""
        return (self._bb_heartbeat_mono > 0.0
                and time.perf_counter() - self._bb_heartbeat_mono
                <= _BB_HEARTBEAT_FRESH_S)

    def _bb_state_name(self) -> str:
        """The cached `_bb_state` as a `BallButlerStates` member name, for
        `ABORTED_BB_NOT_READY(<name>, ...)` — mirrors
        `teensy_bridge_node._bb_outcome_text`'s own fallback shape for a
        value outside the enum."""
        try:
            return proto.BallButlerStates(int(self._bb_state)).name
        except ValueError:
            return 'UNKNOWN(%d)' % (int(self._bb_state),)

    def _on_bb_throw_outcome(self, msg) -> None:
        """`bb/throw_outcome` (R4 reload C3, 2026-09-28): the firmware's
        terminal CMD_RESULT for a reload throw, relayed by
        `ball_butler_node._on_throw_result` as a `String` whose LEADING
        TOKEN is the `BallButlerCommandOutcome` member name (that node's own
        `_bb_outcome_text` builds it as ``"NAME (axis=..., detail1=...)"``).

        Compared against the enum member NAME, never a substring of the
        sentence (plan § 0's precondition-refusal discipline) — a token
        that is not ``OK`` while this attempt is `await_announcement` ends
        it THIS tick: the 2026-09-27 22:37 sitting waited out the full
        `RELOAD_ANNOUNCE_TIMEOUT_S`-bounded deadline (4.6 s) for an
        announcement a `THROW_REJECTED_BAD_STATE` throw was never going to
        produce (BB never threw — `_on_announcement`'s own contract is that
        a `ThrowAnnouncement` only ever follows a real release), when the
        firmware had already told us the outcome.

        Reload throws are the only ones this node ever asks Ball Butler
        for, but `bb/throw_outcome` is a NODE-WIDE topic (calibration
        throws / a future consumer share it) — gating on
        ``phase == 'await_announcement'`` OR `_reload_throw_inflight` means a
        message this attempt never asked for is ignored, not misattributed.

        The second gate is the announced-then-refused case: `ball_butler_node`
        announces at dispatch (the Teensy's CMD_RESULT is TERMINAL — `OK`
        arrives only once the ball has left, so an announcement gated on it
        would come after the flight began and starve `compile_reload` of its
        pre-tilt lead), so on 2026-09-27 every rejected throw was announced
        first and refused 20 ms later. A schedule compiled from such an
        announcement is waiting at the receive attitude for a ball that is
        not coming; the outcome ends it through its rest tail."""
        token = str(getattr(msg, 'data', '')).split(' ', 1)[0]
        if token == proto.BallButlerCommandOutcome.OK.name:
            return
        with self._reload_lock:
            ctx = self._reload_ctx
            reloading = ((ctx is not None and ctx.phase == 'await_announcement')
                         or self._reload_throw_inflight)
        if not reloading:
            return
        end_code = 'REJECTED_BB(%s)' % (token,)
        self._end_attempt(end_code, end_code)

    def _on_hand_telemetry(self, msg):
        """Track the hand's live position and possession evidence — the same
        tri-state `reload_coordinator_node._on_hand_telemetry` feeds into
        `ball_possession.HandBallSensorSource`. `pos_meas`/`pos_cmd` and the
        perf stamp feed the R3 ladder's `hand_fresh` (`_observations`,
        item 6); `_possession_evidence` is unused at R2
        beyond logging but IS the R3 ladder's `ball_evidence` and outcome
        capture's `observer` read (`_ball_evidence`) — one value, two
        readers, stored directly here rather than standing up the source's
        full arrival/retention state machine for a value nothing else
        consumes."""
        self._hand_pos_meas = float(getattr(msg, 'pos_meas', 0.0))
        self._hand_pos_cmd = float(getattr(msg, 'pos_cmd', 0.0))
        self._hand_telemetry_mono = time.perf_counter()
        # The launch-ratio monitor's sample, stamped on the ROS WALL clock
        # (the clock the schedule's release instants live on — a perf stamp
        # would not be comparable with one). Arrival time, not a bridge
        # stamp: the ratio compares PEAKS of the two channels inside one
        # window, so the few ms of transport lag shifts both channels of the
        # same message identically and cannot bias the ratio.
        self._hand_launch.add_sample(
            self.get_clock().now().nanoseconds / 1e9,
            float(getattr(msg, 'vel_ff_cmd', 0.0)),
            float(getattr(msg, 'vel_meas', 0.0)))
        valid = bool(getattr(msg, 'ball_held_valid', False))
        raw = getattr(msg, 'ball_held_raw', None)
        if not valid or raw is None:
            self._possession_evidence = ball_possession.EVIDENCE_UNKNOWN
        else:
            self._possession_evidence = (
                ball_possession.EVIDENCE_SEATED if bool(raw)
                else ball_possession.EVIDENCE_EMPTY)
        # The DEBOUNCED bit feeds the precondition ladder (2026-09-16): the raw
        # bit is right for release/catch EDGES (the debounce lags a departing
        # ball by ~240 ms, ball_possession.py) but it chatters while the hand
        # moves — two of the sitting's three REJECTED_NO_BALL refusals were a
        # single raw-sample carry-flicker during the post-hold park motion with
        # the debounced bit True throughout. A fresh THROW asks "is a ball
        # seated", a state question the debounced bit answers.
        held = getattr(msg, 'ball_held', None)
        if not valid or held is None:
            self._possession_evidence_stable = ball_possession.EVIDENCE_UNKNOWN
        else:
            self._possession_evidence_stable = (
                ball_possession.EVIDENCE_SEATED if bool(held)
                else ball_possession.EVIDENCE_EMPTY)

    def _observations(self, t_abs_s: float) -> Observations:
        """The R3 precondition ladder's snapshot of the whole machine at one
        instant (item 6; fields per `executor.Observations`'s docstring).

        Staleness is judged on THIS node's own `time.perf_counter()` — the
        SAME monotonic domain every cache above is stamped in — never on
        ``t_abs_s`` (the schedule's own wall clock, read off
        `self.get_clock()` in `_on_tick`/`_svc_check`, which may carry a
        CAN-offset the perf-clock caches know nothing about).

        Fed to BOTH the executor's ``observations=`` and, via
        `_ball_evidence`, its ``observer=`` — one builder, no second copy of
        the evidence read.
        """
        now = time.perf_counter()
        mocap_fresh = (self._mocap_mono > 0.0
                       and (now - self._mocap_mono) < _MOCAP_STALE_S)
        status_fresh = (self._traj_status_mono > 0.0
                        and (now - self._traj_status_mono)
                        < _TRAJ_STATUS_STALE_S)
        levelled = bool(status_fresh
                        and self._traj_status.gravity_correction_loaded)
        in_trajectory_mode = bool(status_fresh
                                  and self._traj_status.mode == 'TRAJECTORY')
        hand_fresh = (self._hand_telemetry_mono > 0.0
                     and (now - self._hand_telemetry_mono)
                     < _HAND_STATE_STALE_S)
        # NO hand-POSITION predicate here (RETIRED 2026-09-16, owner
        # decision). `REJECTED_HAND_NOT_PARKED` used to be built from
        # `pos_meas` against `pos_cmd` AND against the ACTIVATE park, and on
        # 2026-09-16 it refused nine schedules at skill 0 on a hand that was
        # genuinely parked: the bridge's `pos_cmd` echo had gone stale at
        # +0.5639 rev across an ACTIVATE re-park while `pos_meas` read
        # +0.0001. The honest enforcement point for "the plan's hand seed is
        # not where the hand is" is the SEED, and it now RECONCILES rather
        # than refuses (`trajectory_node._cycle_start_state`) — the opening
        # REST carries the hand home — from ANY position, over a period SIZED
        # to the firmware's resume-and-follow envelope
        # (`_opening_rest_period` / `schedule.floor_lift_s`), which is the
        # part that was missing when a REST handed 9.63 rev E-STOPPED the
        # machine three times on 2026-09-18. (That day's first answer, a
        # blocking `/park_hand` before the pre-level plus a PARK-BAND
        # precondition, lasted one day: the band was measured against the
        # ACTIVATE park at 0.0 rev while a schedule's REST leaves the hand at
        # `sites.REST_HAND_REV` = 0.3071 rev, so it refused every attempt
        # after the first by construction.) The ladder still has no hand
        # POSITION row: homing is an ACTION, not a refusal. `hand_fresh`
        # stays, and now carries the sizing too — a window sized from a stale
        # encoder is the same defect as a seed reconciled against one.
        return Observations(
            mocap_fresh=mocap_fresh, hand_fresh=hand_fresh,
            levelled=levelled, ball_evidence=self._possession_evidence_stable,
            in_trajectory_mode=in_trajectory_mode,
            hand_lane_refused=self._hand_lane_refused(now))

    def _ball_evidence(self, ball_id: int, t_abs_s: float) -> str:
        """The executor's ``observer`` callable (item 6): the live possession
        evidence from the RAW sensor bit — one hand sensor, ``ball_id`` unused
        (there is exactly one cup). Raw, not debounced, on purpose: this read
        detects release and catch EDGES, and the debounce lags a departing
        ball by ~240 ms (ball_possession.py). The ladder's SEATED precondition
        reads the debounced twin instead (`_observations`, 2026-09-16)."""
        return self._possession_evidence

    # ── the installer callable SkillExecutor dispatches through ────────────

    def _installer(self, kind, terminal, t_abs_s, *, ball_id) -> InstallResult:
        req = InstallSegment.Request()
        req.kind = _WIRE_KIND[kind]
        req.ball_id = int(ball_id)
        if kind == THROW:
            req.t_event_s = float(terminal.t_release_s)
            req.site_mm = [float(v) for v in terminal.site_mm]
            req.target_mm = [float(v) for v in terminal.target_mm]
            req.flight_s = float(terminal.flight_s)
        elif kind == CATCH:
            req.t_event_s = float(terminal.t_land_s)
            req.site_mm = [float(v) for v in terminal.landing_mm]
            req.landing_vel_mm_s = [float(v) for v in terminal.landing_vel_mm_s]
            req.rest_site_mm = [float(v) for v in terminal.rest_site_mm]
            req.hold_tilt_set, req.hold_tilt_rad = _wire_tilt_out(terminal.hold_tilt)
            tt = terminal.then_throw
            if tt is not None:
                req.t_release_s = float(tt.t_release_s)
                req.release_site_mm = [float(v) for v in tt.site_mm]
                req.target_mm = [float(v) for v in tt.target_mm]
                req.flight_s = float(tt.flight_s)
        else:
            req.t_event_s = float(terminal.t_rest_s)
            req.rest_site_mm = [float(v) for v in terminal.rest_site_mm]
            req.rest_tilt_set, req.rest_tilt_rad = _wire_tilt_out(terminal.tilt)

        if not self._install_cli.wait_for_service(timeout_sec=_SERVICE_WAIT_S):
            self.get_logger().error('trajectory/install_segment unavailable')
            return InstallResult(False, 'SERVICE_UNAVAILABLE',
                                 'trajectory/install_segment unavailable', 0.0)
        resp = self._wait_future(self._install_cli.call_async(req))
        if resp is None:
            return InstallResult(
                False, 'SERVICE_TIMEOUT',
                'trajectory/install_segment did not answer in %.1f s'
                % (_SERVICE_WAIT_S,), 0.0)
        result = InstallResult(bool(resp.accepted), str(resp.code),
                               str(resp.message),
                               float(resp.plan_wall_ms) / 1e3,
                               splice_k=int(resp.splice_k),
                               t0_s=float(resp.t0_mono),
                               event_t_s=float(resp.t_event_mono),
                               seeded_post_release=bool(resp.seeded_post_release))
        # The release instant ON THE WIRE: a THROW's release IS its event
        # (`t_event_mono`); only a CATCH-with-throw carries a second event,
        # which rides `t_release_mono` (0.0 for every other shape — a plain
        # THROW included, so reading it here announced NOTHING for a
        # standalone throw: five single self-tosses on 2026-09-13 ended
        # NO_LANDING with a CONFIRMED track and a valid landing in /balls).
        release_mono = (float(resp.t_event_mono) if kind == THROW
                        else float(resp.t_release_mono))
        # Unit A (R3 first sitting, 2026-09-13, L1): the SAME instant, kept
        # as the streaming plan's own future event -- a THROW's release, or
        # a CATCH-with-throw's carried one. 0.0 (a plain CATCH, a REST)
        # clears it: that install carries no future event, so whatever an
        # EARLIER accepted install left pending is now stale (this one
        # superseded it on the wire). Only an ACCEPTED install updates this
        # -- a refusal changes nothing about what is actually streaming.
        if result.accepted:
            self._pending_event_mono = release_mono if release_mono > 0.0 else 0.0
        self._maybe_announce(kind, terminal, result, release_mono,
                             ball_id=ball_id)
        return result

    @staticmethod
    def _release_physics(kind, terminal):
        """``(release_abs_s, site_mm, target_mm, flight_s)`` for the ball
        ``terminal`` releases, or ``None`` when it releases nothing (a plain
        CATCH, a REST). ``release_abs_s`` is the ABSOLUTE wall-clock instant
        the terminal itself carries (the same field `_installer` puts on the
        wire), used by `_maybe_announce` to de-duplicate a CATCH re-send."""
        if kind == THROW:
            return (float(terminal.t_release_s),
                   np.asarray(terminal.site_mm, dtype=float),
                   np.asarray(terminal.target_mm, dtype=float),
                   float(terminal.flight_s))
        if kind == CATCH and terminal.then_throw is not None:
            tt = terminal.then_throw
            return (float(tt.t_release_s), np.asarray(tt.site_mm, dtype=float),
                   np.asarray(tt.target_mm, dtype=float), float(tt.flight_s))
        return None

    def _maybe_announce(self, kind, terminal, result: InstallResult,
                        t_release_mono: float, *, ball_id) -> None:
        """Publish a ``ThrowAnnouncement`` for every ACCEPTED install that
        releases a ball — a THROW, or a CATCH carrying ``then_throw`` (item 3).

        Same six physics fields, same units/frame, same ``thrower_name ==
        target_id == robot_name`` discipline as `reload_coordinator_node.
        _announce_unified`, so every downstream consumer (tracker correlation,
        possession, suppression) is unaffected by which node announced it. The
        release VELOCITY is not on the wire (`InstallSegment.srv` carries the
        commanded site/target/flight, not a realized velocity — there is no
        trim to un-trim here, unlike the FSM path), so it is recomputed from
        exactly those three fields via `ballistics_bc.launch_velocity` — the
        same boundary condition the segment itself was planned to satisfy.

        QUEUES a FRESH, unlatched `FlightLatch` for this release — it does
        NOT reset the schedule ball's correlation. The physical ball a
        self-toss re-throws gets a NEW tracker id each flight, so this
        release needs its own latch; but the next throw is announced at its
        DISPATCH, ~1.25 s before its release, while the PREVIOUS flight of the
        same schedule ball is still in the air — so a reset would throw away
        the correlation of the flight in progress and (via `preexisting`,
        which excludes exactly the airborne id) re-latch onto the NEXT
        flight's id the moment it goes IN_FLIGHT. That is how the 2026-09-16
        sitting's outcome rows came to hold the NEXT flight's landing
        (`logbook/2026-09-16-tracker-correlation-follows-the-flight-in-progress.md`).
        The latch carries the release on the ROS clock — the announcement's
        own `throw_time`, the same instant the tracker flips the ball to
        IN_FLIGHT — because `flight_in_progress` may only read a release that
        has already happened.

        **Exactly once per throw** (never on a CATCH re-send of the same
        carried release): a re-send calls this too, with a terminal whose
        carried `then_throw` is bit-identical to the one already announced
        (the executor's own `_u_cache` caches it by skill index — see
        `SkillExecutor._catch_terminal`'s docstring), so de-duplicating on the
        release's own absolute wall-clock instant is exact, not a heuristic.
        """
        physics = self._release_physics(kind, terminal)
        if physics is None or not result.accepted or t_release_mono <= 0.0:
            return
        release_abs_s, site_mm, target_mm, flight_s_ = physics
        if self._announced_release_s.get(int(ball_id)) == release_abs_s:
            return
        self._announced_release_s[int(ball_id)] = release_abs_s
        vel = ballistics_bc.launch_velocity(site_mm, target_mm, flight_s_)
        lv = ballistics_bc.arrival_velocity(vel, flight_s_)
        now_perf = time.perf_counter()
        now_ros = self.get_clock().now()
        delta_s = float(t_release_mono) - now_perf
        ann = ThrowAnnouncement()
        ann.header.stamp = now_ros.to_msg()
        ann.header.frame_id = 'world'
        ann.thrower_name = self._robot_name
        ann.target_id = self._robot_name
        ann.initial_position = Point(x=float(site_mm[0]), y=float(site_mm[1]),
                                     z=float(site_mm[2]))
        ann.initial_velocity = Vector3(x=float(vel[0]), y=float(vel[1]),
                                       z=float(vel[2]))
        ann.landing_position = Point(x=float(target_mm[0]), y=float(target_mm[1]),
                                     z=float(target_mm[2]))
        ann.landing_velocity = Vector3(x=float(lv[0]), y=float(lv[1]),
                                       z=float(lv[2]))
        ann.predicted_tof_sec = flight_s_
        ann.throw_time = (now_ros + rclpy.time.Duration(seconds=delta_s)).to_msg()
        ann.landing_time = (now_ros + rclpy.time.Duration(
            seconds=delta_s + flight_s_)).to_msg()
        # Ids already IN_FLIGHT at THIS release are phantoms, never our new
        # ball — the same hardening pass `latch_announced_ball` documents.
        # (A previous flight of THIS schedule ball is one of them, and keeps
        # its own latch below: this set is what stops the new release from
        # stealing it, now that the latch is no longer reset.)
        preexisting = tuple(sorted(int(b.id) for b in self._raw_balls
                                   if int(b.status) == _BALL_STATUS_IN_FLIGHT))
        t_release_ros_s = (float(now_ros.nanoseconds) * 1e-9) + delta_s
        with self._correlation_lock:
            self._correlation[int(ball_id)] = advance_flight_latches(
                self._raw_balls, robot_name=self._robot_name,
                latches=tuple(self._correlation.get(int(ball_id), ()))
                        + (FlightLatch(t_release_s=t_release_ros_s,
                                       preexisting=preexisting),),
                now_s=float(now_ros.nanoseconds) * 1e-9,
                in_flight_status=_BALL_STATUS_IN_FLIGHT)
        self._announce_pub.publish(ann)
        self.get_logger().info(
            'skill announced ball %d: release in %.3f s, |v| %.3f m/s'
            % (int(ball_id), delta_s, float(np.linalg.norm(vel)) / 1000.0))

    def _wait_future(self, future, timeout_s: float = _SERVICE_WAIT_S):
        """Poll a service future to completion. Mirrors
        `reload_coordinator_node._wait_future` (the MultiThreadedExecutor
        services it on another thread)."""
        deadline = time.perf_counter() + timeout_s
        while not future.done() and time.perf_counter() < deadline and rclpy.ok():
            time.sleep(0.005)
        if not future.done():
            return None
        try:
            return future.result()
        except Exception as exc:                                  # noqa: BLE001
            self.get_logger().error('install_segment call raised: %s' % (exc,))
            return None

    def _maybe_hold_pending_event(self, end_code: str) -> None:
        """``trajectory/hold`` once when the streaming plan still carries a
        future event (Unit A, R3 first sitting, 2026-09-13, L1): at
        1789304571.365 the executor ended ``ABORTED_NO_RELEASE``, but the
        last accepted install (a CATCH-with-throw) kept streaming -- the
        hand ran two more full strokes after END, the second at the
        session's own acceleration ceiling, and the guard latched.
        ``_stop_attempt``'s docstring says "whatever is streaming already ends
        at rest" -- true of every segment's TERMINAL, but a streaming
        segment can still carry a RELEASE ahead of it, and ending the
        attempt does nothing to that.

        Called from `_on_tick` (every tick while ``attempt_ended`` is True)
        and from `_stop_attempt` (an operator stop is a stop of the MACHINE,
        not only of dispatch) -- idempotent either way, because
        ``_pending_event_mono`` is cleared the moment a hold is attempted,
        so a later call with nothing left pending is a no-op.

        TWO reasons to hold, one op. The second is ``_force_hold``, set by
        `_on_tick` on an ``executor.HAND_LANE_REFUSED`` END (2026-09-18):
        there the streaming plan's rest tail is NOT a safe end, because the
        firmware is HOLDING the hand rather than following it and every knot
        the plan walks on widens the deviation the guard measures. The hold is
        the fix precisely because ``trajectory/hold`` installs a HAND-LESS
        plan: no further hand frame reaches the wire, so the refused command
        stops walking. ``_force_hold`` is consumed on read, so this stays
        once-per-attempt however many ticks call it.
        """
        # `_pending_event_mono` is on the WIRE's clock — `time.perf_counter()`
        # (`trajectory_node`'s `t0_mono`/`t_event_mono`, the same domain
        # `_maybe_announce` differences against) — NOT the ROS clock the
        # executor ticks on; comparing it to the tick's `now` (ROS epoch,
        # ~1.79e9) would read every pending event as already past.
        with self._pending_lock:
            forced, self._force_hold = self._force_hold, False
            pending = (self._pending_event_mono > 0.0
                       and self._pending_event_mono > time.perf_counter())
            if pending:
                self._pending_event_mono = 0.0
            if not (forced or pending):
                return
        why = ('the firmware is holding a REFUSED hand lane' if forced
               else '1 pending event(s) cancelled')
        if not self._hold_cli.wait_for_service(timeout_sec=_SERVICE_WAIT_S):
            self.get_logger().error(
                'END %s: trajectory/hold unavailable -- %s'
                % (end_code, why))
            return
        resp = self._wait_future(self._hold_cli.call_async(Trigger.Request()))
        if resp is None or not bool(resp.success):
            self.get_logger().error(
                'END %s: trajectory/hold failed (%s)'
                % (end_code, '' if resp is None else resp.message))
            return
        self.get_logger().info(
            'END %s: hold installed — %s' % (end_code, why))

    # ── the tick ────────────────────────────────────────────────────────

    def _end_attempt(self, end_code: str, message: str, *,
                     force_hold: bool = False) -> None:
        """End the current attempt with `end_code` (R4 reload C1/C3/C4/C5b,
        2026-09-28) — the ONE enforcement point every automatic reload
        refusal/timeout routes through, so `_goal_end_code` always reflects
        the true end instead of being silently overwritten by whatever the
        executor (if any) does next.

        Mirrors `_stop_attempt`'s two-branch shape (a live, not-yet-ended
        executor gets `attempt_ended`/`end_code` set, so `_on_tick`'s own
        capture — `self._goal_end_code = self._executor.end_code` once
        `done` — carries it through normally) but generalised to any code,
        not only an operator `STOPPED`, and to the case `_stop_attempt`
        does not need to handle: NO live executor at all (`_executor is
        None`, e.g. every reload timeout/refusal once the bridge REST has
        already retired itself — C5b's own finding, see `_reload_ctx`'s
        `__init__` comment) or one that already ended a moment earlier this
        same tick. In BOTH of those cases nothing will ever call `_on_tick`'s
        capture line again for this attempt, so `_goal_end_code` is set
        HERE, directly — the same thing the pre-C1 `_check_reload_timeout`
        already did for `ABORTED_NO_ANNOUNCEMENT`, now the general case.

        `force_hold`, like `_on_tick`'s own `HAND_LANE_REFUSED` branch: the
        streaming plan's rest tail is not a safe end when the firmware is
        HOLDING a refused hand lane rather than following it — every knot
        the plan walks on widens the deviation the guard measures, so the
        hold must fire regardless of whether a release happens to be
        pending too. `_maybe_hold_pending_event` is idempotent either way
        (called unconditionally below), so a `False` here still holds a
        genuinely pending release, exactly as an ordinary abort does.

        Takes `_tick_lock` first (same reason `_stop_attempt` does — this
        can race a tick mid-dispatch on the timer thread), bounded by
        `_STOP_LOCK_WAIT_S`; a failure to acquire is logged and the end
        proceeds anyway, same tradeoff `_stop_attempt` makes."""
        got_lock = self._tick_lock.acquire(timeout=_STOP_LOCK_WAIT_S)
        if not got_lock:
            self.get_logger().warning(
                'END %s: could not acquire the tick lock within %.1f s -- '
                'proceeding without it' % (end_code, _STOP_LOCK_WAIT_S))
        try:
            if self._executor is not None and not self._executor.attempt_ended:
                self._executor.attempt_ended = True
                self._executor.end_code = end_code
            else:
                self._goal_end_code = end_code
            if force_hold:
                with self._pending_lock:
                    self._force_hold = True
                # `_on_tick`'s own inline HAND_LANE_REFUSED branch arms
                # `_force_hold` again on the SAME tick if the executor above
                # was still alive (its `attempt_ended` check runs right
                # after this method returns, inside the SAME `_on_tick`
                # invocation that called `_check_reload_lane_refused`) --
                # `_hold_forced_code` is the "once per attempt" gate BOTH
                # paths read, so it must be set here too, or the hold fires
                # twice.
                self._hold_forced_code = end_code
            with self._reload_lock:
                self._reload_ctx = None
                self._reload_throw_inflight = False
        finally:
            if got_lock:
                self._tick_lock.release()
        self._maybe_hold_pending_event(end_code)
        self.get_logger().error('%s: %s' % (end_code, message))

    def _check_reload_timeout(self) -> None:
        """R4 reload fail-closed (C1, 2026-09-28): once `_reload_ctx.
        deadline_mono` has passed, the attempt ends and `_reload_ctx` is
        cleared (via `_end_attempt`) — the only action this takes, because
        BB's own countdown/state cannot be aborted from here (the ball may
        still come, or BB may still become ready) and the safe end is
        already streaming: the bridge REST's rest tail, holding level under
        the exact point BB was asked to aim at.

        Two phases, two codes, read off the SAME field: `await_bb_idle`
        (armed by `_start_reload`, waiting on BB's OWN heartbeat —
        `_maybe_fire_reload_throw`) times out `ABORTED_BB_NOT_READY`,
        naming BB's last-seen state and ball-in-hand bit — one physical
        fact, not a generic abort (the 2026-09-27 22:37 sitting's
        `THROW_REJECTED_BAD_STATE` x5 was exactly BB not being IDLE yet).
        `await_announcement` (armed once `bb/throw_at_target` is accepted)
        times out `ABORTED_NO_ANNOUNCEMENT`, unchanged from the pre-C1
        code. Never fires during `firing` — `_maybe_fire_reload_throw`'s own
        blocking round trip is still using this SAME `ctx`, and yanking it
        out from under that call would leave a throw request answered by
        nothing."""
        timed_out_phase = None
        with self._reload_lock:
            ctx = self._reload_ctx
            if (ctx is not None and ctx.phase != 'firing'
                    and time.perf_counter() >= ctx.deadline_mono):
                self._reload_ctx = None
                timed_out_phase = ctx.phase
        if timed_out_phase is None:
            return
        # `_end_attempt` also clears `_reload_ctx` (already `None` here —
        # harmless, `with self._reload_lock: self._reload_ctx = None` twice
        # is idempotent) and captures `_goal_end_code` the same way for
        # BOTH phases, so one call covers either.
        if timed_out_phase == 'await_bb_idle':
            self._end_attempt(
                'ABORTED_BB_NOT_READY(%s, ball_in_hand=%s)'
                % (self._bb_state_name(), bool(self._bb_ball_in_hand)),
                'Ball Butler never reported ready (IDLE, ball in hand, '
                'connected) within the deadline -- the platform is at rest '
                'under the aim point (the bridge REST\'s rest tail)')
        else:
            self._end_attempt(
                'ABORTED_NO_ANNOUNCEMENT',
                'no ThrowAnnouncement from ball_butler within the deadline '
                '-- BB\'s own countdown cannot be aborted and the ball may '
                'still come; the platform is at rest under the aim point '
                '(the bridge REST\'s rest tail), the safest place to wait')

    def _check_reload_lane_refused(self) -> None:
        """C5b (2026-09-28): the 2026-09-27 22:37 sitting's guard latch
        fired with NO `HAND_LANE_REFUSED` END ever logged. Root cause: the
        reload bridge (`compile_reload_wait`) is a ONE-skill schedule, and
        `SkillExecutor.done` is "every skill dispatched" — true the INSTANT
        the REST is sent to `trajectory/install_segment`, not once it has
        physically settled. `_on_tick` therefore nulls `self._executor` the
        same tick it dispatches (measured: the REST's own hand-homing move
        ran 6.42 s in the field), and `hand_lane_refused` is read from
        INSIDE `self._executor.tick`'s own `observations` callback — with
        no executor left, it is never read again for the rest of that move.
        An ordinary self-toss/hop schedule has THROW/CATCH/REST left to
        dispatch after its own opening REST, so this gap is unique to the
        reload bridge.

        Runs every tick while `_reload_ctx` is armed (ANY phase — the
        bridge REST or its rest tail is what is physically streaming for
        the whole reload wait, not only `await_bb_idle`), independent of
        `self._executor`: the same fail-closed fact `HAND_LANE_REFUSED`
        already names, checked at the one place still watching once the
        bridge's own executor has retired."""
        with self._reload_lock:
            reloading = self._reload_ctx is not None
        if not reloading or not self._hand_lane_refused(time.perf_counter()):
            return
        self._end_attempt(
            ex.HAND_LANE_REFUSED,
            'the firmware refused the streamed hand lane (sched_refused '
            'moved) during a reload wait and is HOLDING the hand -- '
            'holding the plan before the refused command walks into '
            'MAX_DEVIATION', force_hold=True)

    def _maybe_fire_reload_throw(self) -> None:
        """C1 (2026-09-28): fire `bb/throw_at_target` the moment Ball
        Butler's own heartbeat reports ready (IDLE, ball in hand,
        connected) — called from `_on_tick`, after the timeout/lane-refused
        checks above, so a `ctx` they just ended is never fired on.

        Marks `ctx.phase = 'firing'` under `_reload_lock` BEFORE releasing
        it and making the blocking `bb/throw_at_target` round trip in
        `_fire_reload_throw`: `_on_tick` re-enters on `_cbgroup`'s
        `ReentrantCallbackGroup` (the same reason the install client
        tolerates a concurrent tick), so without this a second concurrent
        tick would see the SAME `await_bb_idle` phase and fire a second
        throw while the first is still in flight."""
        with self._reload_lock:
            ctx = self._reload_ctx
            ready = (ctx is not None and ctx.phase == 'await_bb_idle'
                    and self._bb_state == int(proto.BallButlerStates.IDLE)
                    and self._bb_ball_in_hand and self._bb_connected
                    and self._bb_heartbeat_fresh()
                    and (ctx.bb_left_idle or not ctx.reload_sent))
            if not ready:
                return
            ctx.phase = 'firing'
        self._fire_reload_throw(ctx)

    def _still_current(self, ctx) -> bool:
        """True while ``ctx`` is still THE reload context -- the guard every
        blocking step of `_fire_reload_throw` re-checks before it ends the
        attempt, because the tick keeps running (ReentrantCallbackGroup) while
        that method waits on Ball Butler."""
        with self._reload_lock:
            return self._reload_ctx is ctx

    def _fire_reload_throw(self, ctx) -> None:
        """`bb/throw_at_target`, fired once by `_maybe_fire_reload_throw`
        (`ctx` already marked `firing`). Any failure below ENDS the attempt
        through `_end_attempt` (C4: this runs well after the bridge
        executor exists, exactly the "after the bridge executor exists"
        case that must never leave a live executor behind an already-
        decided outcome) rather than returning anything — there is no
        service response left to return to; the Juggle goal was ACCEPTED
        back when `_start_reload` armed `await_bb_idle`.

        `delay_s` reads `ctx.bridge`'s OWN end instant
        (`Skill.t_abs_s` — "the wall-clock instant the [REST reaches
        rest]", `schedule.Skill`'s own docstring) rather than
        `pattern.floor_lift_s` (the pre-C1 code's proxy): BB's own
        CHECKING_BALL cycle plus this call's own service round trip both
        elapse BETWEEN `_start_reload` arming `await_bb_idle` and this
        method running, so "now" here is not "now" at `_start_reload` --
        using the bridge's own absolute settle instant is the one number
        that stays correct regardless of how long that wait took."""
        now = self.get_clock().now().nanoseconds / 1e9
        bridge_end_abs = float(ctx.bridge.skills[-1].t_abs_s)
        delay_s = max(bridge_end_abs - now + PRETILT_S + LEAD_S,
                     _BB_THROW_DELAY_FLOOR_S)
        aim_x, aim_y, aim_z = _mocap_aim_point_mm(
            ctx.site, CATCH_CUP_Z_MM, self._mocap_to_schedule_mm)
        req = BallButlerThrow.Request()
        req.use_target_point = True
        req.target_name = self._robot_name
        req.target_point_global_mm = Point(x=aim_x, y=aim_y, z=aim_z)
        req.throw_delay_s = float(delay_s)

        if not self._bb_throw_cli.wait_for_service(timeout_sec=_SERVICE_WAIT_S):
            if self._still_current(ctx):
                self._end_attempt('ABORTED_BB_THROW_UNAVAILABLE',
                                  'reload refused: bb/throw_at_target unavailable')
            return

        # C2: armed BEFORE the call (not after `_wait_future` returns) so an
        # announcement delivered while the call is in flight is accepted,
        # not read as "no reload awaiting one" (`_on_announcement`'s pop-
        # first-then-check-None shape) -- the 2026-09-27 field bug this
        # closes armed AFTER the call returned, and the announcement (then
        # published synchronously inside `bb/throw_at_target`'s OWN service
        # handler, C2b) arrived first. `_RELOAD_TOF_CEILING_S` stands in for
        # the real `predicted_tof_s` (not known until the call below
        # returns) -- every reload throw targets the same catch cup from
        # the same BB mount, so a fixed ceiling on its own reachable-
        # workspace ToF is a closed form, not a guess threaded through a
        # second call.
        deadline_mono = (time.perf_counter() + delay_s + _RELOAD_TOF_CEILING_S
                         + RELOAD_ANNOUNCE_TIMEOUT_S)
        with self._reload_lock:
            ctx.phase = 'await_announcement'
            ctx.deadline_mono = deadline_mono

        throw_resp = self._wait_future(self._bb_throw_cli.call_async(req))
        if not self._still_current(ctx):
            # Overtaken while blocked: another path (a `sched_refused` bump
            # via `_check_reload_lane_refused`, a Stop, the timeout) already
            # ended THIS attempt. With the bridge executor retired,
            # `_end_attempt` would write `_goal_end_code` a second time and
            # the Juggle result would lose the reason that actually stopped
            # the machine (2026-09-28 audit). The hold that path armed stays.
            return
        if throw_resp is None:
            self._end_attempt(
                'ABORTED_BB_THROW_TIMEOUT',
                'reload refused: bb/throw_at_target did not answer in '
                '%.1f s -- ball_butler_node itself answered nothing (a Ball '
                'Butler firmware refusal would have answered REJECTED_BB)'
                % (_SERVICE_WAIT_S,))
            return
        if not bool(throw_resp.success):
            # Same INVARIANTS.md row `_start_reload`'s bb/reload refusal
            # surfaces: an external thrower's OWN refusal, verbatim.
            end_code = 'REJECTED_BB(%s)' % (throw_resp.message,)
            self._end_attempt(end_code, end_code)
            return
        self.get_logger().info(
            'reload: bb/throw_at_target OK (predicted_tof=%.3f s, '
            'delay=%.3f s) -- awaiting ThrowAnnouncement'
            % (throw_resp.predicted_tof_s, throw_resp.throw_delay_s))

    def _on_tick(self):
        """Dispatch everything due, then retire a finished executor.

        Guarded against RE-ENTRY: the install client and this timer share one
        `ReentrantCallbackGroup` (see `__init__`) so the client's response can
        be processed WHILE a tick is blocked in `_installer._wait_future` —
        the fix for the single-threaded deadlock this replaces. But that same
        reentrancy would let the executor run a SECOND tick on another thread
        while the first is still mid-install; `acquire(blocking=False)` makes
        an already-busy tick a no-op instead of a second dispatch.

        Retires on `executor.done`, not `attempt_ended` — a released ball's
        outcome can still be finalising after the attempt ends (plan § 2.7),
        and `done` is exactly `attempt_ended` when nothing is listening for
        outcomes (`on_experience is None`), so this is a strict generalisation
        of the R2 behaviour, not a change to it.

        Services the R4 reload (`_check_reload_timeout`, then
        `_check_reload_lane_refused`, then `_maybe_fire_reload_throw`)
        FIRST, unconditionally, in that order: all three read/write only
        `_reload_ctx` under `_reload_lock`, not `_executor`/`_tick_lock`, so
        none may be skipped when the tick lock is busy or `_executor` is
        `None` — exactly the state for most of a reload wait (see
        `_reload_ctx`'s `__init__` comment). Order matters only in that a
        `ctx` a timeout or a refused lane just ended must never also be
        fired on the same tick — each of the three re-reads `_reload_ctx`
        fresh, so an earlier one clearing it makes a later one a no-op.
        """
        self._check_reload_timeout()
        self._check_reload_lane_refused()
        self._maybe_fire_reload_throw()
        if not self._tick_lock.acquire(blocking=False):
            return
        try:
            if self._executor is not None:
                now = self.get_clock().now().nanoseconds / 1e9
                for line in self._executor.tick(now):
                    self.get_logger().info(line)
                    # The Juggle goal result's `per_throw`/`throws`/`caught`
                    # (R4 U5): every finalised release gets exactly one
                    # OUTCOME line (`executor._finalise_outcome`).
                    if 'OUTCOME' in line:
                        self._goal_outcome_lines.append(line)
                if self._executor.attempt_ended:
                    if (self._executor.end_code == ex.HAND_LANE_REFUSED
                            and not self._hold_forced_code):
                        # Armed once per attempt (the flag below), consumed by
                        # `_maybe_hold_pending_event` on the same call.
                        self._hold_forced_code = ex.HAND_LANE_REFUSED
                        with self._pending_lock:
                            self._force_hold = True
                    self._maybe_hold_pending_event(self._executor.end_code)
                if self._executor.done:
                    # Captured HERE (R4 U5) — `self._executor.end_code` is
                    # gone the instant the line below runs, and
                    # `_juggle_execute` reads `_goal_end_code` only after
                    # `_goal_done_event` fires (`_maybe_signal_goal_done`).
                    self._goal_end_code = (self._executor.end_code
                                           if self._executor.attempt_ended
                                           else '')
                    self._executor = None
                    with self._reload_lock:
                        self._reload_throw_inflight = False
        finally:
            self._tick_lock.release()
        self._maybe_signal_goal_done()

    def _maybe_signal_goal_done(self) -> None:
        """Set `_goal_done_event` the instant nothing is left running for the
        Juggle goal in progress (R4 U5) — `self._executor is None and
        self._reload_ctx is None`, NOT merely the former (see
        `_goal_done_event`'s `__init__` comment: the latter is also true
        mid-reload-wait, between the bridge REST finishing and the real
        reload schedule swapping in). Called at the end of every `_on_tick`
        that took the tick lock, so `_juggle_execute`'s wait can never miss
        the transition — including the one `_check_reload_timeout` alone
        causes (that function runs before the lock, in the SAME `_on_tick`
        call this method ends)."""
        with self._reload_lock:
            idle = self._executor is None and self._reload_ctx is None
        if idle:
            self._goal_done_event.set()

    # ── attempt lifecycle ──────────────────────────────────────────────

    def _refuse_if_running(self, response):
        """``response`` refused with the shared "already running" message, or
        ``None`` when no attempt is running (the caller may proceed).

        R4 reload (brief step 1): a reload wait counts as running too, even
        while `_executor` briefly reads `None` between the bridge REST's own
        `SkillExecutor` finishing (`done`, once its one skill dispatches —
        it releases nothing, so nothing keeps it alive) and the
        announcement swapping in the real one — the whole reason
        `_reload_ctx` exists (see its `__init__` comment)."""
        with self._reload_lock:
            reloading = self._reload_ctx is not None
        if self._executor is None and not reloading:
            return None
        response.success = False
        response.message = ('an attempt is already running — cancel the '
                            'jugglebot/juggle goal first')
        return response

    #: Pattern names the Juggle goal accepts — `schedule.Schedule.pattern`'s
    #: own set (R4 owner decision D1).
    _PATTERNS = ('self_toss', 'hop', 'columns')

    def _start_pattern(self, goal) -> SimpleNamespace:
        """The Juggle goal's shared no-motion refusal ladder + schedule
        compile (R4 owner decision D3, U5): merges the R2/R3
        the deleted columns-start and self-toss-start Trigger
        services (deleted — one path, plan § 0) into one method, branching
        only where the compilers differ. Called synchronously from the
        action's ``goal_callback`` (``_juggle_goal``), never from
        ``execute_callback`` — a refusal must reject the GOAL, not abort an
        already-accepted one.

        Returns ``SimpleNamespace(success, message)`` — the same shape the
        deleted Trigger services returned, so every existing assertion on
        ``.success``/``.message`` reads unchanged.
        """
        pattern = str(getattr(goal, 'pattern', ''))
        if pattern not in self._PATTERNS:
            return SimpleNamespace(
                success=False,
                message=('refused: pattern %r is not one of %s'
                         % (pattern, self._PATTERNS)))
        reload = bool(getattr(goal, 'reload', False))
        if pattern == 'columns' and reload:
            return SimpleNamespace(
                success=False,
                message=('columns refused: reload is not available for '
                         'columns until R5 (owner decision D3, 2026-09-23)'))
        refused = self._refuse_if_running(SimpleNamespace(success=None,
                                                          message=None))
        if refused is not None:
            return refused

        apex_m = (float(goal.apex_m) or float(self.get_parameter('apex_m').value))
        separation_mm = (float(goal.separation_mm)
                         or float(self.get_parameter('separation_mm').value))
        dwell_s = float(self.get_parameter('dwell_s').value)
        n_throws = (int(goal.num_cycles)
                   or int(self.get_parameter('n_throws').value))

        # Fresh bookkeeping for THIS goal (U5): the executor's OUTCOME lines
        # accumulate here across the whole attempt (`_on_tick`), for the
        # Juggle result's `per_throw`/`throws`/`caught`; `_goal_end_code` is
        # captured the moment `_on_tick` nulls `self._executor` (see that
        # field's `__init__` comment); the event gates `_juggle_execute`'s
        # wait. Reset here, before anything moves, so a refusal below never
        # touches a previous goal's still-unread result.
        self._goal_outcome_lines = []
        self._goal_end_code = ''
        self._goal_done_event.clear()

        if pattern == 'columns':
            return self._run_columns(apex_m, separation_mm, dwell_s, n_throws)
        return self._run_one_ball(pattern, apex_m, separation_mm, dwell_s,
                                  n_throws, reload)

    def _run_columns(self, apex_m: float, separation_mm: float,
                     dwell_s: float, n_throws: int) -> SimpleNamespace:
        """The ``columns`` branch of :meth:`_start_pattern` — the R2 body of
        the deleted columns-start Trigger service, unchanged."""
        # Columns has no opening REST to home a displaced hand (R4) — refuse
        # BEFORE t0 is read and before anything moves (`_hand_home_error`).
        home_err = self._hand_home_error()
        if home_err:
            msg = 'columns refused: %s' % (home_err,)
            self.get_logger().error(msg)
            return SimpleNamespace(success=False, message=msg)
        # Session-start mocap-vs-commanded frame check (plan
        # `cup-contact-contract.md` § 1) — read-only, before t0 is read and
        # before anything moves, beside `_hand_home_error` above.
        frame_err = self._frame_check_error()
        if frame_err:
            msg = 'columns refused: %s' % (frame_err,)
            self.get_logger().error(msg)
            return SimpleNamespace(success=False, message=msg)
        t0 = self.get_clock().now().nanoseconds / 1e9 + _START_LEAD_S
        try:
            sites = columns_sites(separation_mm)
            pattern = Pattern(sites=sites, apex_m=apex_m, dwell_s=dwell_s,
                             n_throws=n_throws)
            schedule = compile_columns(pattern, t0)
        except ValueError as exc:
            msg = 'columns schedule refused: %s' % (exc,)
            self.get_logger().error(msg)
            return SimpleNamespace(success=False, message=msg)
        # A new schedule reuses small ball ids: drop any flight latch an
        # earlier (possibly early-ended) attempt left for them, so a catch
        # with no in-schedule release can never read a previous attempt's
        # flight (audit, 2026-09-16).
        with self._correlation_lock:
            for _bid in {sk.ball_id for sk in schedule.skills}:
                self._correlation.pop(_bid, None)
        # Latch the firmware's refused-lane counter as this attempt's BASELINE
        # (`_hand_lane_refused`): a refusal an earlier attempt provoked is
        # history, and only a bump from here on ends this one.
        self._sched_refused_at_start = self._sched_refused
        self._force_hold = False
        self._hold_forced_code = ''
        self._executor = SkillExecutor(
            schedule, self._installer, tracker=self._tracker,
            catch_aim_source=self._catch_aim_source(),
            resend_max_per_catch=self._catch_resend_max(),
            launch_ratio=self._launch_ratio)
        msg = ('columns schedule compiled: %d skills, %d throws, t0=%.3f'
              % (len(schedule.skills), n_throws, t0))
        self.get_logger().info(msg)
        return SimpleNamespace(success=True, message=msg)

    def _live_limits(self):
        """The session limits `admissible.check_limits` judges the loaded box
        against: LEG limits off the latest `trajectory/status`
        (`leg_*_limit_*`), falling back to the YAML module default on a field
        that reads 0.0 — the message's own documented "field absent" sentinel
        (`TrajectoryStatus.msg`) — and the HAND accel cap, which the status
        message does not carry at all (no live topic publishes a session hand
        cap), so it is the static session default (`hw.
        JB_TRAJ_HAND_ACC_LIMIT_RPS2`) rather than a value read off the wire."""
        status = self._traj_status
        leg_vel = float(status.leg_vel_limit_mmps) or float(hw.JB_TRAJ_LEG_VEL_LIMIT_MMPS)
        leg_acc = float(status.leg_acc_limit_mmps2) or float(hw.JB_TRAJ_LEG_ACC_LIMIT_MMPS2)
        leg_jerk = float(status.leg_jerk_limit_mmps3) or float(hw.JB_TRAJ_LEG_JERK_LIMIT_MMPS3)
        return SimpleNamespace(leg_vel_mmps=leg_vel, leg_acc_mmps2=leg_acc,
                               leg_jerk_mmps3=leg_jerk,
                               hand_acc_limit_rps2=float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2))

    def _live_hand_rev(self):
        """The MEASURED hand position (rev) if `/hand_telemetry` is fresh, else
        ``None`` — fail-closed, the same cache-plus-freshness shape
        `_live_commanded_position` uses for the platform."""
        if (self._hand_telemetry_mono <= 0.0
                or (time.perf_counter() - self._hand_telemetry_mono)
                >= _HAND_STATE_STALE_S):
            return None
        return float(self._hand_pos_meas)

    def _opening_rest_period(self):
        """``(period_s, why_not)`` for the opening REST that HOMES THE HAND —
        the schedule's first window, sized to the hand's measured displacement
        from `sites.REST_HAND_REV`.

        THE ONE HOME (owner instruction, 2026-09-18: *"if the hand isn't where
        it needs to be at the start, it should smooth-move down to the start
        position before beginning the cycle"*). Every REST in a schedule leaves
        the hand at ``REST_HAND_REV`` (0.3071 rev, the cup at
        ``uc.SETTLE_CUP_Z_MM``), so that is the one position a schedule starts
        from and the one reference this sizes against. The bridge's ACTIVATE
        park (0.0 rev) stays what `/recover` parks to; a hand sitting there is
        simply 0.307 rev from home and gets a homing REST like any other
        displacement, with no special case. The predecessor of this method
        (`_park_hand`, one day old) measured the start against the PARK and so
        REFUSED every attempt after the first — the hand was exactly where the
        previous schedule's REST had correctly left it.

        WHY A SIZED WINDOW AND NOT A FASTER MOVE. On a healthy ARMED lane the
        hand follows the stream at full C2 speed (90 rev/s through a throw), so
        the ceiling here is not about following — it is about RESUMING. After a
        hand-less hold (every attempt END installs one) the firmware hand group
        holds its last knot and promotes a resumed lane only if the frame at
        the handover instant is within ``SCHED_RESUME_TOL_POS_HAND_REV`` =
        0.05 rev of the held state; the first frame reaches the wire ~60-100 ms
        after the plan's origin, so a lane that has moved further than that by
        then is REFUSED and the guard then measures the refused command against
        the encoder until MAX_DEVIATION E-STOPS the machine (2026-09-18, three
        times in one sitting). `schedule.floor_lift_s` bounds the move by the
        firmware's own two numbers for exactly this regime —
        ``JB_OP_GENTLE_MOVE_VEL_LIMIT_RPS`` (2.5 rev/s, the profiled park's
        rate) and ``RECOVER_SLEW_ACCEL_RPS2`` (5 rev/s², the slew ramp) — which
        leaves the deviation two orders inside ``MAX_DEVIATION_HAND_REV``
        (2.5 rev) and the lead clamp (2.0 rev) the whole way.

        Refuses on ONE fact only, and pre-motion: an unknown or stale hand
        position. A window sized from a stale encoder is the same defect as the
        seed reconciled against one, and there is nothing else here to guess
        with.
        """
        rev = self._live_hand_rev()
        if rev is None:
            return (0.0, 'hand_telemetry is stale or absent (no sample in '
                         '%.1f s) — the opening REST cannot be sized to home '
                         'a hand position nobody has read'
                         % (_HAND_STATE_STALE_S,))
        period = floor_lift_s(rev)
        peak_v, peak_a = home_hand_bounds(rev, period)
        self.get_logger().info(
            'opening REST homes the hand: %+.4f → %+.4f rev over %.2f s '
            '(peak <= %.2f rev/s, %.2f rev/s²)'
            % (rev, REST_HAND_REV, period, peak_v, peak_a))
        return (period, '')

    def _hand_home_error(self) -> str:
        """``''`` when the hand is within :data:`~jugglebot.motion.skills.
        schedule.HOME_BAND_REV` of `REST_HAND_REV`, else why not — the
        COLUMNS-only check.

        `compile_columns` has no opening REST yet (carried to R4): its first
        skill is a CATCH that splices onto the live plan, so there is no window
        in the schedule that could home a displaced hand and nothing for
        `_opening_rest_period` to size. Refusing pre-motion is therefore the
        honest answer for this path — the alternative is streaming a catch
        whose seed is 9 rev from the hand, which is the 2026-09-18 E-STOP.
        Self-toss does not use this: it HOMES instead.
        """
        rev = self._live_hand_rev()
        if rev is None:
            return ('hand_telemetry is stale or absent (no sample in %.1f s) '
                    '— the hand position cannot be checked'
                    % (_HAND_STATE_STALE_S,))
        if abs(rev - REST_HAND_REV) > HOME_BAND_REV:
            return ('the hand is at %+.4f rev, %.3f rev from the schedule home '
                    '(%+.4f rev, outside the %.2f rev band) and the columns '
                    'schedule has no opening REST to home it (R4) — run a '
                    'Juggle goal with pattern self_toss or hop, whose opening '
                    'REST homes the hand, or DEACTIVATE → ACTIVATE and re-level'
                    % (rev, abs(rev - REST_HAND_REV), REST_HAND_REV,
                       HOME_BAND_REV))
        return ''

    def _frame_check(self) -> FrameCheckResult:
        """Snapshot the two buffers and run the pure check
        (`_frame_offset_check`) — the node-side half of the session-start
        mocap-vs-commanded frame check (plan `cup-contact-contract.md`
        § 1). Read-only, no motion, safe to call from either start path
        before anything moves."""
        return _frame_offset_check(tuple(self._platform_mocap_xy),
                                   tuple(self._commanded_xy_hist),
                                   time.perf_counter())

    def _frame_check_error(self) -> str:
        """The session-start frame check (plan `cup-contact-contract.md`
        § 1): ALWAYS logs the offset — or why it could not be measured, so
        every sitting records it whether or not lateral authority is live —
        and returns ``''`` unless `learner_lateral_authority_mm` is nonzero
        AND the offset is over the limit or cannot be measured. Fail-closed:
        a cannot-evaluate result refuses exactly like an over-limit one,
        never a silent pass; a `REJECTED_FRAME_OFFSET:` message always
        carries the number (or names the missing input).

        `learner_lateral_authority_mm == 0` (an explicit re-pin, no longer
        the default since 2026-09-21) still runs the check and still logs
        it, but never refuses on it — lateral authority is pinned, so there
        is nothing here to protect the attempt from.

        An evaluable, within-limit result is also ADOPTED as the
        mocap-to-schedule correction `_on_balls` subtracts from every tracker
        landing from now on (`_mocap_to_schedule_mm`, 2026-09-23) — at any
        authority, since the learner rows and the OUTCOME lines are written
        at authority 0 too and must mean the same thing either way. An
        over-limit or cannot-evaluate result leaves the previous correction
        in place (a stale good number beats none) and says so."""
        result = self._frame_check()
        if result.evaluable:
            self.get_logger().info('frame check: %s' % (result.detail,))
        else:
            self.get_logger().info(
                'frame check: cannot evaluate — %s' % (result.detail,))
        if result.evaluable and result.within_limit:
            self._mocap_to_schedule_mm = (float(result.dx_mm),
                                          float(result.dy_mm))
            self.get_logger().info(
                'tracker landings are now corrected by (x %+.1f, y %+.1f) mm '
                '(mocap -> schedule frame): the OUTCOME lines, learner rows '
                'and catch aims measure from the cup\'s real position'
                % (-result.dx_mm, -result.dy_mm))
        else:
            prev = self._mocap_to_schedule_mm
            self.get_logger().warning(
                'tracker landing correction NOT updated (%s) — %s'
                % ('offset over limit' if result.evaluable
                   else 'cannot evaluate',
                   'no correction is applied' if prev is None else
                   'keeping the earlier (x %+.1f, y %+.1f) mm'
                   % (-prev[0], -prev[1])))
        authority_mm = float(
            self.get_parameter('learner_lateral_authority_mm').value)
        if authority_mm <= 0.0:
            return ''
        if not result.evaluable:
            return ('REJECTED_FRAME_OFFSET: cannot evaluate the mocap-vs-'
                    'command offset (%s) with learner_lateral_authority_mm='
                    '%.1f > 0 — fix the named input (a moving platform, a '
                    'stale or missing Platform body), or start with '
                    'learner_lateral_authority_mm:=0'
                    % (result.detail, authority_mm))
        if not result.within_limit:
            return ('REJECTED_FRAME_OFFSET: %s — an offset this large is a '
                    'wrong base alignment, not the lever arm: re-align QTM to '
                    'the base, or start with learner_lateral_authority_mm:=0'
                    % (result.detail,))
        return ''

    def _prelevel(self) -> str:
        """Bring the platform to gravity-level rest before compiling the R3
        self-toss schedule (item 7; plan R3 carried note, ``''`` on success
        else why not). Mirrors
        `reload_coordinator_node._unified_prelevel`: the
        session-start schedule is planned from the seed the machine is
        ALREADY holding, and an un-prelevelled seed refuses ``LIMIT_JERK`` at
        174 772 mm/s^3 against the 150 000 session limit (prelevelled: 19 773
        — probe 2026-09-13). A pure attitude move: the live commanded xy/z is
        held exactly, only the attitude comes to gravity-level (the E3
        ingest, `trajectory_node`'s `GoToPose` handler, turns an IDENTITY
        intent into the gravity-level counter-tilt — C-LEVEL-1)."""
        if not self._go_to_pose_cli.wait_for_service(timeout_sec=_SERVICE_WAIT_S):
            return 'trajectory/go_to_pose unavailable'
        live = self._live_commanded_position(time.perf_counter())
        if live is None:
            return ('trajectory/commanded_position is stale — the pre-level '
                    'move has no xy/z to hold and one that guessed a pose '
                    'would move the platform')
        req = GoToPose.Request()
        req.pose = Pose(position=Point(x=float(live[0]), y=float(live[1]),
                                       z=float(live[2])),
                        orientation=Quaternion())
        req.duration_s = 0.0
        resp = self._wait_future(self._go_to_pose_cli.call_async(req))
        if resp is None:
            return ('trajectory/go_to_pose did not answer in %.1f s'
                    % (_SERVICE_WAIT_S,))
        if not bool(resp.accepted):
            return ('the pre-level move was refused: %s (%s)'
                    % (resp.code, resp.message))
        self.get_logger().info(
            'platform pre-levelled to gravity-level over %.2f s before the '
            'self-toss schedule (identity intent, E3-corrected)'
            % (float(resp.planned_duration_s),))
        # go_to_pose returns at plan INSTALL; wait out the planned move so the
        # schedule compiles from a STOPPED, gravity-level machine.
        end_at = time.perf_counter() + float(resp.planned_duration_s)
        while rclpy.ok() and time.perf_counter() < end_at:
            time.sleep(min(1.0 / _TICK_HZ, max(0.0, end_at - time.perf_counter())))
        return ''

    def _run_one_ball(self, kind: str, apex_m: float, separation_mm: float,
                      dwell_s: float, n_throws: int,
                      reload: bool) -> SimpleNamespace:
        """The ``self_toss``/``hop`` branch of :meth:`_start_pattern` (R4,
        generalises the deleted self-toss-start Trigger service to
        ``kind in ('self_toss', 'hop')`` — one site for ``self_toss``, the
        two :func:`~jugglebot.motion.skills.sites.columns_sites` sites for
        ``hop``, same compiler either way
        (:func:`~jugglebot.motion.skills.schedule.compile_one_ball`)):
        THROW -> CATCH -> ... -> REST, the learner on, memory at
        ``temp/learn/<plant_id>/memory.csv``.

        Order: refuse fast on anything that does not need the platform to
        move (running / repo root / pattern / box+limits) BEFORE
        `_prelevel` (item 7) moves anything, then compile the schedule —
        pre-levelling is the last check before compilation, per the R3
        carried note ("a healthy launch rests the cup at the gravity-level
        counter-tilt").
        """
        if _ADMISSIBLE_BOX_PATH is None:
            msg = ('cannot find the repo root from %r — the admissible box '
                  '/ memory paths cannot be resolved' % (__file__,))
            self.get_logger().error(msg)
            return SimpleNamespace(success=False, message=msg)

        plant_id = str(self.get_parameter('plant_id').value)

        if kind == 'self_toss':
            site_x_mm = float(self.get_parameter('site_x_mm').value)
            site_y_mm = float(self.get_parameter('site_y_mm').value)
            # Finding 5, R3 audit (2026-09-13): the self-toss goal always
            # names the site 'P1', but the ADMISSIBLE BOX it was swept
            # against (`_ADMISSIBLE_BOX_PATH`, keyed on `(site.name,
            # target.name)`) carries no site xy at all — a box swept at the
            # default (-50, 0) mm site is silently applied to whatever xy
            # the caller passes. Refuse before anything else (no platform
            # motion, no box/pattern object built yet) rather than let an
            # off-box site reach `_command_u`'s clip with the wrong box.
            if (abs(site_x_mm - _DEFAULT_SITE_X_MM) > 1e-9
                    or abs(site_y_mm - _DEFAULT_SITE_Y_MM) > 1e-9):
                msg = (
                    'self-toss refused: site (%.3f, %.3f) mm != the swept '
                    'default (%.3f, %.3f) mm — the admissible box carries no '
                    'site xy, so a box swept at the default site cannot be '
                    'applied here' % (site_x_mm, site_y_mm, _DEFAULT_SITE_X_MM,
                                      _DEFAULT_SITE_Y_MM))
                self.get_logger().error(msg)
                return SimpleNamespace(success=False, message=msg)
            sites = (Site(_DEFAULT_SITE_NAME,
                          np.array([site_x_mm, site_y_mm, CATCH_CUP_Z_MM])),)
        else:  # 'hop'
            sites = columns_sites(separation_mm)

        try:
            pattern = OneBallPattern(sites=sites, apex_m=apex_m,
                                     dwell_s=dwell_s, n_throws=n_throws)
        except ValueError as exc:
            msg = '%s pattern refused: %s' % (kind, exc)
            self.get_logger().error(msg)
            return SimpleNamespace(success=False, message=msg)

        try:
            boxes = adm.load(_ADMISSIBLE_BOX_PATH)
            adm.check_limits(boxes, self._live_limits())
        except adm.AdmissibleError as exc:
            msg = 'admissible box refused: %s' % (exc,)
            self.get_logger().error(msg)
            return SimpleNamespace(success=False, message=msg)
        # No box covers the requested apex -> refuse BEFORE any motion
        # (before `_prelevel`), rather than let `_command_u` discover this
        # only once the learner is already asking for a command (the latent
        # defect this closes: a box swept for one apex silently reused at
        # another). One pair for self_toss (P1,P1); two for hop (each
        # direction the ball actually crosses, `compile_one_ball`'s own
        # release/target rule — `sites[i % n]` -> `sites[(i+1) % n]`).
        n_sites = len(sites)
        pairs = sorted({(sites[i % n_sites].name, sites[(i + 1) % n_sites].name)
                       for i in range(n_sites)})
        by_name = {s.name: s for s in sites}
        missing = []
        for pair in pairs:
            release_xy = (float(by_name[pair[0]].cup_mm[0]),
                         float(by_name[pair[0]].cup_mm[1]))
            target_xy = (float(by_name[pair[1]].cup_mm[0]),
                        float(by_name[pair[1]].cup_mm[1]))
            if adm.select(boxes, kind, pair, apex_m,
                         release_site_xy_mm=release_xy,
                         target_site_xy_mm=target_xy) is None:
                reason = adm.describe_miss(boxes, kind, pair, apex_m,
                                           release_site_xy_mm=release_xy,
                                           target_site_xy_mm=target_xy)
                missing.append('%r: %s' % (pair, reason))
        if missing:
            msg = ('%s refused: no admissible box covers site pair(s) -- %s'
                  % (kind, '; '.join(missing)))
            self.get_logger().error(msg)
            return SimpleNamespace(success=False, message=msg)

        # Finding 11, R3 audit (2026-09-13): built here, before `_prelevel`
        # actually moves the platform, and guarded — `memory_path` raises
        # `ValueError` on a bad `plant_id`, and `Memory(...)` can raise
        # `OSError` (unreadable file) or `csv.Error` (malformed rows); left
        # uncaught in a goal accept either kills the node AFTER the platform
        # has already moved.
        try:
            memory = Memory(memory_path(_REPO_ROOT, plant_id))
        except (ValueError, OSError, csv.Error) as exc:
            msg = 'memory refused: %s' % (exc,)
            self.get_logger().error(msg)
            return SimpleNamespace(success=False, message=msg)

        # The opening REST HOMES THE HAND, over a period sized to the hand's
        # measured displacement (`_opening_rest_period`). Read here, after
        # every no-motion refusal and before the first thing that moves, for
        # the same reason `_prelevel` is: its one refusal (a hand position
        # nobody has read) must fire with nothing having moved. Nothing
        # commands the hand between here and the compile — `_prelevel` is a
        # platform-attitude move with no hand track — so the measurement is
        # still the hand's position when the schedule streams.
        lift_s, lift_err = self._opening_rest_period()
        if lift_err:
            msg = '%s refused: %s' % (kind, lift_err)
            self.get_logger().error(msg)
            return SimpleNamespace(success=False, message=msg)
        pattern = dataclasses.replace(pattern, floor_lift_s=lift_s)

        # Session-start mocap-vs-commanded frame check (plan
        # `cup-contact-contract.md` § 1) — read-only, beside the opening-REST
        # (park-analogue) check above and before `_prelevel` moves anything.
        frame_err = self._frame_check_error()
        if frame_err:
            msg = '%s refused: %s' % (kind, frame_err)
            self.get_logger().error(msg)
            return SimpleNamespace(success=False, message=msg)

        prelevel_err = self._prelevel()
        if prelevel_err:
            msg = '%s refused: %s' % (kind, prelevel_err)
            self.get_logger().error(msg)
            return SimpleNamespace(success=False, message=msg)

        # R4 reload (owner decision D4, brief step 1): everything above is
        # shared with the ordinary self-toss/hop (same site(s)/pattern, same
        # box + memory) — only how the schedule gets COMPILED differs,
        # because the reload's real schedule cannot be built until Ball
        # Butler's announcement supplies the receive tilt. `reload` is the
        # Juggle goal's own flag (the deleted per-node reload parameter it replaced is
        # deleted — one path, plan § 0).
        if reload:
            return self._start_reload(
                SimpleNamespace(success=None, message=None), site=sites[0],
                pattern=pattern, boxes=boxes, memory=memory)

        now = self.get_clock().now().nanoseconds / 1e9
        try:
            # Compile once to read the opening REST's own dispatch lead (the
            # floor lift + Skill.lead_s arithmetic — schedule.FLOOR_LIFT_S /
            # _assign_leads), rather than restating it as a fixed margin: a
            # PROBE compile at t0=now puts the first dispatch `lead_s` (0.15 s
            # at the current grid) before `now`, so shifting t0 by exactly
            # that deficit plus one tick period of operational margin (the
            # call/dispatch latency `_START_LEAD_S` names for columns) lands
            # the real schedule's first dispatch at or after "now".
            probe = compile_one_ball(pattern, now)
            deficit = now - probe.skills[0].dispatch_s()
            t0 = now + max(deficit, 0.0) + (1.0 / _TICK_HZ)
            schedule = compile_one_ball(pattern, t0)
        except ValueError as exc:
            msg = '%s schedule refused: %s' % (kind, exc)
            self.get_logger().error(msg)
            return SimpleNamespace(success=False, message=msg)

        learner_cfg = lr.LearnerConfig()
        learner = SimpleNamespace(
            command=lambda x, y_d: memory.command(x, y_d, learner_cfg))

        # A new schedule reuses small ball ids: drop any flight latch an
        # earlier (possibly early-ended) attempt left for them, so a catch
        # with no in-schedule release can never read a previous attempt's
        # flight (audit, 2026-09-16).
        with self._correlation_lock:
            for _bid in {sk.ball_id for sk in schedule.skills}:
                self._correlation.pop(_bid, None)
        # Latch the firmware's refused-lane counter as this attempt's BASELINE
        # (`_hand_lane_refused`): a refusal an earlier attempt provoked is
        # history, and only a bump from here on ends this one.
        self._sched_refused_at_start = self._sched_refused
        self._force_hold = False
        self._hold_forced_code = ''
        self._executor = SkillExecutor(
            schedule, self._installer, tracker=self._tracker,
            catch_aim_source=self._catch_aim_source(),
            resend_max_per_catch=self._catch_resend_max(),
            launch_ratio=self._launch_ratio,
            learner=learner, boxes=boxes,
            lateral_authority_m=float(self.get_parameter(
                'learner_lateral_authority_mm').value) / 1000.0,
            observer=self._ball_evidence, observations=self._observations,
            on_experience=self._bind_on_experience(memory))
        msg = (
            '%s schedule compiled: %d skills, %d throws, plant_id=%r, '
            'memory rows=%d, t0=%.3f'
            % (kind, len(schedule.skills), n_throws, plant_id, len(memory), t0))
        self.get_logger().info(msg)
        return SimpleNamespace(success=True, message=msg)

    def _start_reload(self, response, *, site: Site, pattern: OneBallPattern,
                      boxes, memory: Memory):
        """R4 reload start path (owner decision D4; C1/C4 rework,
        2026-09-28): installs the opening-REST-only bridge
        (`schedule.compile_reload_wait` — homes the hand, holds level at
        ``site``, since the receive tilt is not yet known), asks Ball
        Butler to reload, then ARMS the `await_bb_idle` wait rather than
        asking for the throw inline (`_maybe_fire_reload_throw`, the tick,
        fires it once BB's own heartbeat reports ready — see `_reload_ctx`'s
        `__init__` comment for the full phase sequence and why: the
        2026-09-27 22:37 sitting measured BB's own CHECKING_BALL cycle at
        ~1.1 s while the old code asked to throw 5 ms after `bb/reload`
        answered, drawing `THROW_REJECTED_BAD_STATE` on every attempt).

        Called only from `_run_one_ball`, after every no-motion
        refusal, the box/memory build and `_prelevel` — see that method's
        R4 branch. ``site``/``pattern``/``boxes``/``memory`` are exactly
        what the ordinary self-toss path would compile with; the reload's
        own DECAY REST hands off into the SAME one-ball pattern
        (`schedule.compile_reload`'s docstring), so it needs the identical
        box/memory the tail throws will be judged and learned against.

        C4: once the bridge `SkillExecutor` below is constructed, the
        machine has a COMMITTED, already-ticking plan (the 40 Hz timer
        dispatches it independently of whether this goal is ever
        accepted) — a `bb/reload` refusal from here on must not REJECT the
        goal (the 2026-09-27 field bug: the orchestrator read "an attempt
        is already in progress?" while the REST streamed on with nothing
        tracking it, and its fresh-origin solve lag walked into
        MAX_DEVIATION). It ACCEPTS instead and ends the attempt through
        `_end_attempt`, exactly like every other in-flight reload
        refusal (`_fire_reload_throw`, `_check_reload_timeout`,
        `_check_reload_lane_refused`, `_on_bb_throw_outcome`) — one
        enforcement point, not a second copy of "reject vs end" logic per
        call site.
        """
        now = self.get_clock().now().nanoseconds / 1e9
        lift_s = float(pattern.floor_lift_s)
        try:
            # Same t0 idiom as the ordinary self-toss path just above (a
            # probe compile to read the bridge's own dispatch lead, then
            # shift so the real compile's first dispatch lands at/after
            # "now") — one schedule, two skills long, so restating it here
            # rather than threading a third compiler through that block.
            probe = compile_reload_wait(site, lift_s, now)
            deficit = now - probe.skills[0].dispatch_s()
            t0 = now + max(deficit, 0.0) + (1.0 / _TICK_HZ)
            bridge = compile_reload_wait(site, lift_s, t0)
        except ValueError as exc:
            # No motion has been committed yet -- an ordinary goal REJECT
            # is correct here (nothing for `_end_attempt` to end).
            response.success = False
            response.message = 'reload bridge schedule refused: %s' % (exc,)
            self.get_logger().error(response.message)
            return response

        with self._correlation_lock:
            self._correlation.pop(0, None)
        self._sched_refused_at_start = self._sched_refused
        self._force_hold = False
        self._hold_forced_code = ''
        self._executor = SkillExecutor(
            bridge, self._installer, tracker=self._tracker,
            catch_aim_source=self._catch_aim_source(),
            resend_max_per_catch=self._catch_resend_max(),
            launch_ratio=self._launch_ratio,
            observer=self._ball_evidence, observations=self._observations)

        # From HERE ON (C4): the bridge REST is a committed physical fact.
        # A refusal below ACCEPTS the goal and ends the attempt through
        # `_end_attempt`, never rejects it.
        #
        # `bb/reload` ONLY for an empty hand (2026-09-28 21:41 sitting, owner):
        # Ball Butler's RELOAD_CMD runs its ~1 s ball check, and a ball already
        # in hand needs none -- BB feeds as soon as its axes are in position.
        # The decision reads a FRESH heartbeat only; a stale or absent one is
        # not evidence of a ball, so it falls through to `bb/reload`.
        ball_ready = (self._bb_heartbeat_fresh() and self._bb_connected
                      and self._bb_ball_in_hand)
        if ball_ready:
            reload_sent = False
            ready_timeout_s = RELOAD_BB_READY_TIMEOUT_S
            head = ('reload: Ball Butler already holds a ball -- bb/reload '
                    'skipped (its ball check is for an empty hand)')
        else:
            if not self._reload_cli.wait_for_service(timeout_sec=_SERVICE_WAIT_S):
                msg = 'reload refused: bb/reload unavailable'
                self._end_attempt('ABORTED_BB_RELOAD_UNAVAILABLE', msg)
                response.success = True
                response.message = (
                    msg + ' -- the bridge REST already streaming is the safe '
                    'rest tail')
                return response
            reload_resp = self._wait_future(
                self._reload_cli.call_async(Trigger.Request()))
            if reload_resp is None:
                msg = ('reload refused: bb/reload did not answer in %.1f s'
                      % (_SERVICE_WAIT_S,))
                self._end_attempt('ABORTED_BB_RELOAD_TIMEOUT', msg)
                response.success = True
                response.message = (
                    msg + ' -- the bridge REST already streaming is the safe '
                    'rest tail')
                return response
            if not bool(reload_resp.success):
                # INVARIANTS.md `REJECTED_BB(<message>)`: an external thrower's
                # OWN refusal surfaces verbatim, not laundered into a generic
                # abort — `reload_resp.message` is BB's own diagnostic sentence
                # (`teensy_bridge_node._svc_bb_reload`), unedited.
                end_code = 'REJECTED_BB(%s)' % (reload_resp.message,)
                self._end_attempt(end_code, end_code)
                response.success = True
                response.message = (
                    end_code + ' -- the bridge REST already streaming is the '
                    'safe rest tail')
                return response
            reload_sent = True
            ready_timeout_s = RELOAD_BB_FETCH_TIMEOUT_S
            head = ('reload requested: Ball Butler reports %s -- bb/reload OK'
                    % ('an EMPTY hand' if self._bb_heartbeat_fresh()
                       else 'no fresh heartbeat'))

        # C1: do NOT ask bb/throw_at_target here — arm `await_bb_idle` and
        # let `_maybe_fire_reload_throw` (the tick) fire it once BB's own
        # heartbeat reports ready. `bridge`/`site` ride the ctx so
        # `_fire_reload_throw` can derive `delay_s` from the bridge's own
        # settle instant and the aim point from `site`, whenever it runs.
        # `reload_sent` makes the gate also require `bb_left_idle` (set by
        # `_on_bb_heartbeat`): after a `bb/reload` the IDLE that counts is
        # the one AFTER BB's own reload/check cycle, never the one before it.
        with self._reload_lock:
            self._reload_throw_inflight = False
            self._reload_ctx = SimpleNamespace(
                phase='await_bb_idle', pattern=pattern, boxes=boxes,
                memory=memory, site=site, bridge=bridge,
                reload_sent=reload_sent, bb_left_idle=False,
                deadline_mono=time.perf_counter() + ready_timeout_s)
        response.success = True
        response.message = (
            '%s -- awaiting Ball Butler ready (IDLE, ball in hand, connected) '
            'within %.1f s' % (head, ready_timeout_s))
        self.get_logger().info(response.message)
        return response

    def _on_announcement(self, msg: ThrowAnnouncement) -> None:
        """Ball Butler's `ThrowAnnouncement` for a reload throw (R4, brief
        step 1c) — the AUTHORITY the reload schedule is compiled from,
        never compiled before it arrives (`schedule.compile_reload`'s own
        docstring).

        Filters on `thrower_name == 'ball_butler'` so this never mistakes
        one of THIS node's own self-toss announcements for a reload
        (`_maybe_announce` publishes those with `thrower_name ==
        self._robot_name`, never `'ball_butler'`), and on `target_id ==
        self._robot_name` so an announcement not naming us is ignored. Pops
        `_reload_ctx` before doing anything else — an announcement that
        arrives with nothing awaiting it (already timed out, or we were
        never reloading) has nothing to act on and is logged, not acted on.
        """
        if (str(msg.thrower_name) != 'ball_butler'
                or str(msg.target_id) != self._robot_name):
            return
        with self._reload_lock:
            ctx = self._reload_ctx
            self._reload_ctx = None
        if ctx is None:
            self.get_logger().info(
                'reload: a ball_butler announcement for us arrived with no '
                'reload awaiting one (already timed out, or none was '
                'requested) -- ignored')
            return

        # THE JOIN: `_on_balls` subtracts the measured mocap-to-schedule
        # offset from a tracker LANDING to reach the schedule frame (see
        # that method's docstring) -- the announcement's landing is the
        # SAME physical fact (BB's own predicted touch-down), read off the
        # SAME mocap-tagged frame, so it gets the SAME correction. Velocity
        # is untouched, as `_on_balls` leaves it.
        off = self._mocap_to_schedule_mm
        dx, dy = (0.0, 0.0) if off is None else (float(off[0]), float(off[1]))
        landing_mm = np.array([msg.landing_position.x - dx,
                               msg.landing_position.y - dy,
                               msg.landing_position.z], dtype=float)
        landing_vel_mm_s = np.array([msg.landing_velocity.x,
                                     msg.landing_velocity.y,
                                     msg.landing_velocity.z], dtype=float)
        t_land_abs_s = (float(msg.landing_time.sec)
                        + float(msg.landing_time.nanosec) * 1e-9)
        throw_time_s = (float(msg.throw_time.sec)
                       + float(msg.throw_time.nanosec) * 1e-9)
        now = self.get_clock().now().nanoseconds / 1e9
        try:
            schedule = compile_reload(landing_mm, landing_vel_mm_s,
                                      t_land_abs_s, ctx.pattern, now)
        except ValueError as exc:
            self.get_logger().error(
                'reload: the announced landing could not be scheduled (%s) '
                '-- no reload schedule installed; the platform stays at '
                'rest under the aim point (the bridge REST is already '
                'streaming)' % (exc,))
            return

        # Same FlightLatch rule `_maybe_announce` applies to our own
        # releases (`advance_flight_latches`, ball_possession's one RULE) —
        # a fresh, unlatched latch for the reload's schedule ball id (0,
        # `compile_reload`'s own convention), not appended to whatever an
        # earlier attempt left (already dropped in `_start_reload`).
        preexisting = tuple(sorted(int(b.id) for b in self._raw_balls
                                   if int(b.status) == _BALL_STATUS_IN_FLIGHT))
        with self._correlation_lock:
            self._correlation[0] = advance_flight_latches(
                self._raw_balls, robot_name=self._robot_name,
                latches=(FlightLatch(t_release_s=throw_time_s,
                                     preexisting=preexisting),),
                now_s=self._now_s(), in_flight_status=_BALL_STATUS_IN_FLIGHT)

        learner_cfg = lr.LearnerConfig()
        learner = SimpleNamespace(
            command=lambda x, y_d: ctx.memory.command(x, y_d, learner_cfg))
        # The SkillExecutor SWAP (brief step 1c): the bridge's own executor
        # is done by now in every ordinary case (its one REST dispatched
        # ticks ago) — this still takes `_tick_lock` first, same reason
        # `_stop_attempt` does (finding 12, R3 audit): a tick mid-dispatch on
        # another thread must not read `_executor` half-swapped.
        got_lock = self._tick_lock.acquire(timeout=_STOP_LOCK_WAIT_S)
        if not got_lock:
            self.get_logger().warning(
                'reload: could not acquire the tick lock within %.1f s -- '
                'swapping the executor without it' % (_STOP_LOCK_WAIT_S,))
        try:
            self._sched_refused_at_start = self._sched_refused
            self._force_hold = False
            self._hold_forced_code = ''
            self._executor = SkillExecutor(
                schedule, self._installer, tracker=self._tracker,
                catch_aim_source=self._catch_aim_source(),
                resend_max_per_catch=self._catch_resend_max(),
                launch_ratio=self._launch_ratio,
                learner=learner, boxes=ctx.boxes,
                lateral_authority_m=float(self.get_parameter(
                    'learner_lateral_authority_mm').value) / 1000.0,
                observer=self._ball_evidence, observations=self._observations,
                on_experience=self._bind_on_experience(ctx.memory))
        finally:
            if got_lock:
                self._tick_lock.release()
        with self._reload_lock:
            self._reload_throw_inflight = True
        self.get_logger().info(
            'reload: ThrowAnnouncement received -- compiled the reload '
            'schedule (%d skills) and swapped the executor'
            % (len(schedule.skills),))

    def _bind_on_experience(self, memory: Memory):
        """``on_experience`` callable bound to ``memory`` (plan § 2.5 step 6):
        appends the row (file I/O, orchestrator thread only — never the 40 Hz
        emitter, plan § 0) and logs it, one line per throw."""
        def _on_experience(exp):
            # `Memory.append` REFUSES a row outside `APEX_RATIO_BAND`
            # (2026-09-16) — the executor drops such a row first, so this is
            # the belt-and-braces path, and it must never raise into the tick
            # loop: an exception here would surface as an executor fault and
            # end an attempt over a bookkeeping refusal.
            try:
                memory.append(exp)
            except ValueError as exc:
                self.get_logger().error(
                    'memory row REFUSED (not appended): %s — u=%s y=%s'
                    % (exc, exp.u.tolist(), exp.y.tolist()))
                return
            self.get_logger().info(
                'memory row appended: x=%s u=%s y=%s caught=%s'
                % (exp.x.tolist(), exp.u.tolist(), exp.y.tolist(), exp.caught))
        return _on_experience

    def _stop_attempt(self) -> str:
        """End the current attempt (the deleted stop-service Trigger
        service's body, unchanged — R4 U5 now reaches it from the Juggle
        action's ``cancel_callback``/``execute_callback`` pair,
        ``_juggle_cancel``/``_juggle_execute``, instead of a service
        handler). Every segment is rest-terminal, so whatever is streaming
        already ends at rest — but rest-terminal is not event-free: a
        streaming segment can still carry a RELEASE ahead of it (a THROW, or
        a CATCH-with-throw), and ending the attempt does nothing to that on
        its own. `_maybe_hold_pending_event` (Unit A) installs one
        `trajectory/hold` when that is the case; otherwise this issues no
        motion command of its own.

        Does NOT discard the executor: a released ball's outcome can still
        be finalising after the attempt ends (plan § 2.7, "outcomes keep
        finalising after the attempt ends") — clearing `_executor` here
        (the earlier behaviour) silently dropped that flight's memory row.
        `_on_tick` already retires on `executor.done`, not `attempt_ended`
        alone, so setting the two flags and leaving the executor live is
        enough: a stopped attempt's observable flight still produces its
        memory row, and `_refuse_if_running`'s existing `_executor is not
        None` check refuses a new start for as long as that finalisation is
        still pending.

        Takes `_tick_lock` first (finding 12, R3 audit, 2026-09-13, bounded
        by `_STOP_LOCK_WAIT_S`): without it, this can race a tick mid-
        `_dispatch` on the timer thread — this method sets `attempt_ended` /
        `end_code` from the action's thread while `_on_tick` may be about to
        overwrite them with a dispatch's own refusal code, or STOPPED may
        itself overwrite a genuine abort the same tick produced a moment
        earlier. A failure to acquire within the bound is logged and the stop
        proceeds anyway — a stop that never lands is worse than one that
        loses this one race.

        Returns ``SimpleNamespace(success=True, message=...)`` — the same
        shape the deleted Trigger service returned (always a "success" in
        that sense — a stop can be a no-op, but it never fails), so every
        existing assertion on ``.success``/``.message`` reads unchanged."""
        got_lock = self._tick_lock.acquire(timeout=_STOP_LOCK_WAIT_S)
        if not got_lock:
            self.get_logger().warning(
                'cancel: could not acquire the tick lock within %.1f s '
                '— proceeding without it' % _STOP_LOCK_WAIT_S)
        ended_now = False
        message = ''
        try:
            if self._executor is not None:
                if not self._executor.attempt_ended:
                    self._executor.attempt_ended = True
                    self._executor.end_code = 'STOPPED'
                    ended_now = True
                    message = (
                        'attempt stopped — the rest tail is already streaming')
                else:
                    message = (
                        'attempt already ended (end_code=%r) — stop is a '
                        'no-op' % self._executor.end_code)
            else:
                # R4 reload (brief step 1): a reload wait leaves `_executor`
                # `None` (the bridge REST's own executor is `done` well
                # before the announcement can arrive — see `_reload_ctx`'s
                # `__init__` comment), so it is not "no attempt" — clear it
                # so a new start is not refused by `_refuse_if_running`.
                # BB's own countdown cannot be aborted (the ball may still
                # come) and the platform is already resting at the aim
                # point (the bridge REST's rest tail), so there is nothing
                # else to command.
                with self._reload_lock:
                    was_reloading = self._reload_ctx is not None
                    self._reload_ctx = None
                message = (
                    'reload wait cancelled -- BB\'s own countdown cannot be '
                    'aborted and the ball may still come; the platform is '
                    'at rest under the aim point' if was_reloading else
                    'no attempt was running')
        finally:
            if got_lock:
                self._tick_lock.release()
        if ended_now:
            # An operator stop is a stop of the MACHINE, not only of
            # dispatch (Unit A) — the same hold `_on_tick` installs on an
            # abort, called here outside the tick lock so a blocked
            # `trajectory/hold` round trip cannot stall ticking.
            self._maybe_hold_pending_event('STOPPED')
        self.get_logger().info(message)
        return SimpleNamespace(success=True, message=message)

    # ── the Juggle action (R4 owner decision D3, U5) ────────────────────

    def _juggle_goal(self, goal_request):
        """``goal_callback``: accept iff `_start_pattern` compiles a
        schedule — the SAME no-motion refusal ladder the deleted
        the deleted columns-start/self-toss-start Trigger services
        ran. Runs on `_juggle_cbgroup`, not the tick/install pair's
        `_cbgroup`, so a second goal is rejected promptly even while a first
        goal's `_juggle_execute` is mid-wait on another thread."""
        result = self._start_pattern(goal_request)
        if not result.success:
            self.get_logger().error('jugglebot/juggle REJECTED: %s'
                                    % (result.message,))
            return GoalResponse.REJECT
        self.get_logger().info('jugglebot/juggle ACCEPTED: %s'
                               % (result.message,))
        return GoalResponse.ACCEPT

    def _juggle_cancel(self, goal_handle):
        """``cancel_callback``: always ACCEPTs — an operator cancel must
        always be able to end the running attempt (mirrors the deleted
        the deleted stop service's unconditional ``response.success = True``).
        Mirrors `reload_coordinator_node._cancel_callback`'s split: this
        callback only decides accept/reject; `_juggle_execute`'s wait loop
        is what actually calls `_stop_attempt` once it observes
        ``goal_handle.is_cancel_requested``."""
        return CancelResponse.ACCEPT

    def _juggle_execute(self, goal_handle):
        """``execute_callback``: wait for the attempt `_juggle_goal` already
        started (through `_start_pattern`) to finish, publishing feedback at
        <= `_FEEDBACK_HZ`, and return the `Juggle.Result`.

        Does NOT tick — the 40 Hz timer (`_on_tick`, `_cbgroup`) is the only
        dispatcher; this callback only waits on `_goal_done_event` (set by
        `_on_tick`'s `_maybe_signal_goal_done` the instant the attempt is
        truly idle — see that method's docstring for why "truly idle" is
        not just "`_executor is None`"). A cancel calls `_stop_attempt` (the
        deleted stop-service body) and then keeps waiting on the SAME
        event — a cancel does not itself end the attempt this tick (every
        segment is rest-terminal, `_stop_attempt`'s own docstring), it only
        starts the rest tail that will.
        """
        result = Juggle.Result()
        feedback = Juggle.Feedback()
        cancelled = False
        period_s = 1.0 / _FEEDBACK_HZ
        while not self._goal_done_event.wait(timeout=period_s):
            if goal_handle.is_cancel_requested and not cancelled:
                cancelled = True
                self._stop_attempt()
            if not rclpy.ok():
                break
            feedback.phase = 'RUNNING'
            feedback.throw_index = len(self._goal_outcome_lines)
            feedback.caught = sum(1 for line in self._goal_outcome_lines
                                  if 'caught=True' in line)
            goal_handle.publish_feedback(feedback)
        end_code = self._goal_end_code
        outcome = 'STOPPED' if end_code == 'STOPPED' else (end_code or 'COMPLETED')
        result.outcome = outcome
        result.success = bool(outcome == 'COMPLETED')
        result.throws = len(self._goal_outcome_lines)
        result.caught = sum(1 for line in self._goal_outcome_lines
                            if 'caught=True' in line)
        result.per_throw = list(self._goal_outcome_lines)
        if cancelled:
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _svc_check(self, request, response):
        """``skills/check`` (item 8): every current R3 precondition-ladder
        refusal at once, plus the admissible-box/limits status and the
        session-start frame check (`_frame_check_error`, plan
        `cup-contact-contract.md` § 1) — the dress-rehearsal runsheet calls
        this before any powered attempt (Rigor: "make gates report every
        refusal at once", plan Workflow Rules). Never moves the platform and
        never touches ``_executor``.
        """
        now = self.get_clock().now().nanoseconds / 1e9
        obs = self._observations(now)
        # `launch=True`: a self-toss attempt's opening THROW carries every
        # row this ladder has, so rehearsing with it reports the widest set.
        # The retired `fresh_origin=` argument is gone with the hand-position
        # row it gated (2026-09-16) — a fresh origin now RECONCILES its hand
        # seed in `trajectory_node._cycle_start_state` instead of refusing.
        codes = precondition_refusals(obs, launch=True)
        ok = True
        lines = []
        if codes:
            ok = False
            lines.append('ladder REFUSED: %s' % (', '.join(codes),))
        else:
            lines.append('ladder OK')
        # The hand's POSITION is REPORTED, never gated (2026-09-16): a hand
        # away from home is a thing the opening REST carries there, and the
        # rehearsal's job is to let the operator SEE it — and now also SEE the
        # period that homing will take, which is the one number a displaced
        # hand changes about the schedule (2026-09-18). Reported against
        # `sites.REST_HAND_REV`, the home a schedule actually starts from, not
        # against the bridge's ACTIVATE park: measuring the start against the
        # park is exactly what refused every attempt after the first. `pos_cmd`
        # is the bridge's own echo and is the channel that went stale in the
        # 2026-09-16 sitting — named next to `pos_meas` so a disagreement is
        # visible rather than inferred.
        lines.append(
            'hand pos_meas %+.4f rev (home %+.4f rev, cup %.1f mm), bridge '
            'echo pos_cmd %+.4f rev — REPORTED, not gated: the opening REST '
            'plans from the MEASURED hand and would take %.2f s to home it'
            % (self._hand_pos_meas, REST_HAND_REV, REST_CUP_Z_MM,
               self._hand_pos_cmd, floor_lift_s(self._hand_pos_meas)))
        # Session-start mocap-vs-commanded frame check (plan
        # `cup-contact-contract.md` § 1), same string and the same
        # fail-closed semantics as the start paths (`_run_columns`'s
        # `_hand_home_error`-adjacent check, `_run_one_ball`'s
        # `_opening_rest_period`-adjacent one): a refusal the operator would
        # meet at `start_*` must be reported by the dry-run path too (UH-3,
        # 2026-09-06 — "make gates report every refusal at once"), not
        # discovered only on the powered robot. `learner_lateral_authority_mm
        # == 0` (an explicit re-pin, no longer the default since 2026-09-21)
        # never refuses on this — nothing here needs protecting when
        # authority is pinned — so it is reported as an informational line
        # carrying the measured offset instead.
        frame_err = self._frame_check_error()
        if frame_err:
            ok = False
            lines.append(frame_err)
        else:
            frame_result = self._frame_check()
            authority_mm = float(
                self.get_parameter('learner_lateral_authority_mm').value)
            if authority_mm <= 0.0:
                lines.append(
                    'frame check: %s (informational — '
                    'learner_lateral_authority_mm=0)'
                    % (frame_result.detail if frame_result.evaluable
                       else 'cannot evaluate — %s' % (frame_result.detail,)))
            else:
                lines.append('frame check OK: %s' % (frame_result.detail,))
        if _ADMISSIBLE_BOX_PATH is None:
            ok = False
            lines.append('box REFUSED: cannot find the repo root from %r'
                         % (__file__,))
        else:
            try:
                boxes = adm.load(_ADMISSIBLE_BOX_PATH)
                adm.check_limits(boxes, self._live_limits())
            except adm.AdmissibleError as exc:
                ok = False
                lines.append('box REFUSED: %s' % (exc,))
            else:
                # Per (site pair, apex band) -- the same predicate `select`
                # judges a live throw by, not just "a box exists somewhere".
                by_pair: Dict[Tuple[str, str], List[Tuple[float, float]]] = {}
                for b in boxes:
                    by_pair.setdefault(b.site_pair, []).append(b.apex_band_m)
                parts = ['%s %s' % (p, ', '.join(
                            '%.2f-%.2f' % (lo, hi) for lo, hi in sorted(bands)))
                        for p, bands in sorted(by_pair.items())]
                check_apex_m = float(self.get_parameter('apex_m').value)
                pair = (_DEFAULT_SITE_NAME, _DEFAULT_SITE_NAME)
                default_xy = (_DEFAULT_SITE_X_MM, _DEFAULT_SITE_Y_MM)
                if adm.select(boxes, 'self_toss', pair, check_apex_m,
                              release_site_xy_mm=default_xy,
                              target_site_xy_mm=default_xy) is None:
                    ok = False
                    lines.append(
                        'box REFUSED: no box covers %r at apex %.3f m (%s)'
                        % (pair, check_apex_m, '; '.join(parts)))
                else:
                    lines.append('box OK: %s' % ('; '.join(parts),))
                # The HOP's boxes at the node's separation, both directions —
                # the pattern a GUI Start actually flies (R4 runsheet finding,
                # 2026-09-24: this line only ever checked self_toss, so a
                # missing hop box surfaced as a goal rejection, never here).
                sep_mm = float(self.get_parameter('separation_mm').value)
                try:
                    hop_sites = columns_sites(sep_mm)
                except ValueError as exc:
                    ok = False
                    lines.append('hop box REFUSED: %s' % (exc,))
                else:
                    for a, b in ((hop_sites[0], hop_sites[1]),
                                 (hop_sites[1], hop_sites[0])):
                        if adm.select(boxes, 'hop', (a.name, b.name),
                                      check_apex_m,
                                      release_site_xy_mm=a.cup_mm[:2],
                                      target_site_xy_mm=b.cup_mm[:2]) is None:
                            ok = False
                            lines.append(
                                'hop box REFUSED: no hop box covers %s->%s at '
                                'separation %.1f mm, apex %.3f m'
                                % (a.name, b.name, sep_mm, check_apex_m))
                        else:
                            lines.append('hop box OK: %s->%s at %.1f mm'
                                         % (a.name, b.name, sep_mm))
        site_x_mm = float(self.get_parameter('site_x_mm').value)
        site_y_mm = float(self.get_parameter('site_y_mm').value)
        if (abs(site_x_mm - _DEFAULT_SITE_X_MM) > 1e-9
                or abs(site_y_mm - _DEFAULT_SITE_Y_MM) > 1e-9):
            ok = False
            lines.append(
                'site REFUSED: (%.3f, %.3f) mm != the swept default '
                '(%.3f, %.3f) mm — the admissible box carries no site xy'
                % (site_x_mm, site_y_mm, _DEFAULT_SITE_X_MM,
                   _DEFAULT_SITE_Y_MM))
        else:
            lines.append('site OK')
        response.success = ok
        response.message = '; '.join(lines)
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SkillNode()
    # ── MultiThreadedExecutor, not plain spin ─────────────────────────────
    # `_installer` blocks the tick timer in `_wait_future`'s poll loop,
    # waiting for THIS node's own install-client response — a response only
    # the executor can deliver. Under `rclpy.spin` (single-threaded) the same
    # thread that is busy-waiting is the only thread that could ever process
    # it: every install times out, forever (found 2026-09-13 by reading, never
    # exercised live). Two threads is what unblocks it: the client and the
    # tick timer share a `ReentrantCallbackGroup` (`SkillNode.__init__`), and
    # a MultiThreadedExecutor with more than one thread lets the response be
    # processed on a thread OTHER than the one blocked in the tick. The tick
    # itself is guarded against re-entering its own dispatch (`_on_tick`'s
    # `_tick_lock`), so extra threads cannot start a second install.
    # A third thread (finding 4, R3 audit, 2026-09-13) is for the
    # subscriptions' own `MutuallyExclusiveCallbackGroup` (`SkillNode.
    # __init__`'s `_sub_cbgroup`): with only two threads, both could be
    # occupied by the reentrant install-client/tick pair while a blocked
    # service call (`_prelevel`) starves every subscription callback.
    # A fourth (R4 owner decision D3, U5) is for the Juggle action's OWN
    # `_juggle_cbgroup`: `_juggle_goal` (a goal accept) and `_juggle_execute`
    # (an accepted goal's wait/feedback loop) both need to run concurrently
    # with each other (a second goal must be rejectable while the first is
    # still executing) AND with the tick — without a fourth thread the three
    # existing threads could all be occupied (tick, a blocked install
    # response, a blocked subscription) while `_juggle_goal` waits for one to
    # free up, which would delay every goal accept behind whatever the other
    # three threads are doing.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
