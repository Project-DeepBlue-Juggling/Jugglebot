"""Teensy can-bridge ROS 2 node — UDP-sourced mirror of ``can_node.py``.

This is the can-bridge successor to ``can_node.py``: it exposes the
same observable surface (robot state, hand telemetry, link/fault health) but
sources everything from the can-bridge Teensy over the dedicated UDP link
(``teensy_link``) instead of socketcan. ``can_node`` was DELETED in
the SocketCAN decommission (2026-07-06; see
logbook/2026-07-06-phase13-socketcan-decommission.md) — the many
``can_node.py:NNN`` parity
citations throughout this file refer to the last pre-deletion revision in git
history (the capability mapping lives in
``ros_ws/docs/can-node-teensy-parity.md``). The bridge owns the
**production topic/service names** directly (legs/hand promoted off the
side-by-side ``/teensy/*`` namespace during the leg/hand cutover; BB + cone in a later cutover).

Safety invariants this node upholds (non-negotiable):

* **``mpc_active`` defaults to 0.** The J→T heartbeat carries ``flags=0``
  (``mpc_active`` clear) on every startup path. The ONLY 0→1 path is the
  ``/set_setpoint_output`` service's stream-then-arm pre-check
  (ARMING_CONTRACT A1; automatic on ACTIVE entry via the orchestrator, A2).
  The old ``~enable_setpoint_output`` boot-arm is retained but INERT (loud
  ERROR — it armed with zero preconditions, the arm-before-stream trap).
  While disabled, no ``Setpoint`` frame is ever sent and the Teensy will not
  enable leg output. There is no code path through ``__init__`` that sets
  ``mpc_active`` at all.
* **Never command a dead link.** The link health monitor (Commit 2) mirrors
  ``can_node._watchdog_check``'s deferred-stow latch
  (``logbook/2026-05-19-can-loss-fault-response-safety-inversion.md``): it does
  not command the Teensy while the link is down; a stow is deferred to confirmed
  reconnect.

Threading model: the :class:`TeensyLinkClient` RX thread decodes frames and
invokes our callbacks. Those callbacks are kept short — they only stash the
latest decoded frame under a lock. All ROS publishing happens on the executor
thread via timers, mirroring ``can_node``'s "poll on one thread, publish on a
timer" split. This respects the teensy_link callback contract ("keep them fast;
enqueue heavy work").

Build order: Commit 1 adds the read side (this file). Commit 2 adds the link
watchdog + deferred-stow latch. Commit 3 adds the gated setpoint downlink.
Commit 4 adds the RPC service surface. Each is a separate, test-gated commit.
"""

from __future__ import annotations

import hashlib
import math
import os
import struct
import threading
import time
from collections import deque, namedtuple

import quaternion  # numpy-quaternion; same library can_node uses for tilt→quat

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from jugglebot_interfaces.msg import (
    BallButlerHeartbeat,
    CatchEvent,
    CatchingConeHeartbeat,
    HandTelemetryMessage,
    MotorStateSingle,
    RobotState,
    SetMotorVelCurrLimitsMessage,
)
from jugglebot_interfaces.srv import (
    ODriveCommandService,
    SetHandTrajCmd,
    SetHandGains,
    SetFloat,
    SetString,
    ActivateOrDeactivate,
    GetTiltReadingService,
)
from jugglebot_interfaces.action import BallButlerThrowCmd, HomeMotors
from geometry_msgs.msg import Quaternion
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, SetBool

from teensy_link import (
    TeensyLinkClient,
    RpcClient,
    RpcServer,
    TimeOfDayServer,
    RpcMethod,
    RpcError,
    RpcTimeout,
    MsgType,
    LinkState,
    BusHealth,
    FaultState,
    HeartbeatT2J,
    Telemetry,
    Diagnostic,
    Profile,
    ConeFrame,
    CmdResultFrame,
    BbAxisEstimates,
    LegCmd,
    PlatformFrame,
    HandCmdEcho,
    HandSensor,
    CanErrors,
    BridgeTxDiag,
    BridgeIdentity,
    ClockDiag,
    CacheDiag,
    RingDiag,
)
from teensy_link import protocol as p
from teensy_link import rpc_args
from teensy_link.fault_logic import LinkLossLatch
from teensy_link.setpoint_pump import SetpointPump
from teensy_link.encoder_search import (
    EncoderSearch, AxisStatus, AXIS_STATE_ENCODER_INDEX_SEARCH,
)
from teensy_link.homing import (
    HomingMonitor, AxisStatus as HomingAxisStatus,
)
from teensy_link.activate import (
    ActivateMonitor, AxisStatus as ActivateAxisStatus,
)
from teensy_link.deactivate import (
    DeactivateMonitor, AxisStatus as DeactivateAxisStatus,
)
from jugglebot.can import catching_cone
from jugglebot.can import odrive
from jugglebot.can.motor_state import MotorStateTracker
import jugglebot.hardware_config as hw
import jugglebot.protocol_config as proto


#: /cache_diag WARN threshold on the per-axis encoder-cache age FLOOR
#: (``CacheDiag.age_min_us``), microseconds.
#:
#: The floor is what the bridge-temporal investigation is hunting: between two
#: ODrive ``get_encoder_estimate`` broadcasts the cache age ramps from ~0 to one
#: broadcast period (a few ms), so a healthy floor sits near zero at ANY uptime,
#: and a floor of tens or hundreds of ms means the cache is not being written.
#:
#: 20 ms is chosen from both ends rather than picked round. Below: it is several
#: times the broadcast period, so ordinary jitter, a missed frame or a late
#: telemetry tick cannot reach it. Above: the firmware's own feedback-staleness
#: guard (``canbridge_config.h MOTOR_FB_STALENESS_US``) fires at 150 ms and
#: suppresses commands, so this WARNs at roughly a seventh of the level at which
#: the bridge would already be defending itself — early enough to be a warning
#: rather than a post-mortem, which is the entire class this arc exists to close
#: ("command-latency drift is invisible until a session is already degraded").
#: The 290-340 ms lag S1 measured is an order of magnitude above it.
#:
#: ADVISORY ONLY: this sets a DiagnosticStatus level on an instrumentation
#: topic. Nothing gates, refuses or actuates on it.
CACHE_AGE_FLOOR_WARN_US = 20_000

#: ``CacheDiag`` age fields saturate here rather than wrapping (u32 max, ~71.6
#: min). Also what an axis that has never been cached reports — read
#: ``seen_mask`` before treating the rail as staleness.
CACHE_AGE_SATURATED_US = 0xFFFF_FFFF

#: /ring_diag WARN threshold on the CAN RX-ring LEAK — the number of frames the
#: ring is holding that its own ``_available`` counter does not know about
#: (``true_depth - avail_reported``, sampled immediately after the drain loop, so
#: ``_available`` is 0 by definition at that instant and the residue IS the leak).
#:
#: 2 rather than 0, and the choice is the whole calibration. The probe is taken
#: from task context while the CAN ISR can push concurrently, and the accessor's
#: read ordering makes that skew UNDER-reporting: a concurrent push advances
#: tail and ``_available`` together, so reading head/tail first and
#: ``_available`` last can only make the reported leak SMALLER than the truth,
#: never larger (see ``Circular_Buffer::probe``; the node's negative-leak clamp
#: is the same property from the other side). So a reported leak of 1 is already
#: a real stranded frame. The one over-reporting path is the full-ring overwrite
#: edge (the ISR's head advance straddling the h/t reads) — reachable only with
#: the ring at 256/256, i.e. long after any threshold fired. 2 keeps one frame
#: of margin against that edge while conceding nothing that matters: an analyst
#: must still treat 1 as real. The predicted signature is not marginal: a ratcheting leak
#: saturates the 256-slot ring, i.e. two orders of magnitude above this line.
#:
#: ADVISORY ONLY: this sets a DiagnosticStatus level on an instrumentation topic.
#: Nothing gates, refuses or actuates on it.
RING_LEAK_WARN_FRAMES = 2


# ── The alarmed command-latency monitor (2026-07-24 closure contract) ──
#
# WHAT IS BEING CLOSED. `logbook/2026-07-18-teensy-uptime-tracking-degradation.md`
# § Addendum (2026-07-24) makes closure conditional on TWO deliverables, not
# one: the fix, AND "a continuously-measured, alarmed end-to-end command-latency
# monitor (logged with uptime_ms)". The class is not "the ring leaked" — it is
# **latency drift is invisible until a session is already degraded**. Every
# measurement layer that class needs now exists (RING_DIAG's leak, CACHE_DIAG's
# age floor, the lead-clamp mask, all bagged beside uptime_ms) and all of them
# are still SILENT: a WARN level on a 1 Hz diagnostics topic is invisible unless
# somebody is watching that topic, and nobody watches a topic during a sitting.
# What follows is the LOUD half — the operator-facing alarm — and nothing else.
#
# ADVISORY, NEVER A GATE. Identical policy to BRIDGE_FW_CHECK and the
# install-skew check: this logs and it publishes a row, and it refuses,
# suppresses and faults exactly nothing. A latency monitor wired into the leg
# command path would convert a REPORTING gap into an outage, which is a strictly
# worse failure than the one being reported.

#: Cadence of the throttled latency-monitor WARNING while any condition holds.
#: Rate-EXACT off a monotonic timestamp rather than rclpy
#: ``throttle_duration_sec`` — same reason as _GUARD_LATCH_REPEAT_S (its
#: first-call bookkeeping can silently drop the initial repeat), plus this way
#: the throttle is observable from a test instead of living inside rcutils.
#: 30 s against a session measured in minutes-to-hours keeps the line
#: unmissable in a scrollback without drowning the leg/hand traffic.
_LATENCY_MONITOR_LOG_PERIOD_S = 30.0

#: How long a RING_DIAG / CACHE_DIAG verdict stays valid for the monitor. Both
#: frames arrive at 1 Hz, so 5 s is five missed windows. Past that the input is
#: STALE and stops being evaluated — a 1 Hz input that stopped arriving must not
#: keep asserting its last value, in either direction.
_LATENCY_MONITOR_INPUT_STALE_S = 5.0

#: Lead-clamp DUTY that raises the alarm: the fraction of sampled heartbeats
#: whose ``lead_clamp_mask`` was nonzero while setpoints were streaming.
#:
#: Calibrated on the SAME statistic this code computes (mask-any, streaming-
#: gated, trailing 10 s window with the sample floor below), re-measured across
#: the 2026-08-15 bags: a LEAKING FW 13 bridge at 4.04 h reads duty 0.4441 with
#: a worst window of 0.68 (fires), while FW 14 at 5.80 / 5.84 / 15.19 h reads
#: 0.0183 / 0.0198 / 0.0039 with worst windows 0.0667 / 0.0333 / 0.0200 (quiet).
#: 0.15 sits above every healthy worst-window and well below the degraded mean.
#: The margin is sample-count dependent and worth stating honestly: against a
#: 0.06 plant it is ~2.1 binomial SD at the 30-sample floor and ~3.8 SD at 100
#: samples — which is why the floor below is not left at 30.
#:
#: (Earlier drafts of this comment quoted a "healthy 0.00-0.06" band. That mixed
#: a whole-bag mask-any figure with a per-leg in-moves figure — two different
#: statistics — and only the mask-any numbers above back this threshold.)
#:
#: WHY DUTY AND NOT A LAG THRESHOLD. S1 method correction (c): the lead clamp
#: pins the commanded position to feedback, so a lag-only monitor reads
#: HEALTHIEST exactly when the clamp is doing the most damage. Duty is the
#: behavioural symptom that made every degradation visible late, and it is the
#: only one of the three inputs that works on ANY firmware — the other two need
#: FW >= 12 / >= 13 frames.
_CLAMP_DUTY_WARN = 0.15

#: Trailing window the duty is measured over. Long enough that a single clamped
#: move onset (which is normal, and is what a healthy sub-0.02 duty is largely
#: made of) cannot fire it; short enough that the alarm lands DURING the session
#: rather than after it, which is the entire point of the contract.
_CLAMP_DUTY_WINDOW_S = 10.0

#: Minimum samples in that window before the duty is evaluated at all — 5 s of
#: streaming at the 10 Hz heartbeat.
#:
#: 50 rather than 30, for two reasons that point the same way. Statistically, 30
#: samples put a 0.06 plant only ~2.1 binomial SD below the threshold, and 50
#: takes that to ~2.7. Physically, clamping CONCENTRATES at move onset (S1: 94 %
#: of clamping moves clamp within 100 ms of onset), so the first seconds of a
#: stream are exactly where the transient lives — evaluating from 3 s makes the
#: first verdict a move-onset detector. 5 s spans several onsets and amortises
#: them. The cost is a 2 s later first verdict against a 10 s window, which is
#: nothing on the timescale the contract cares about, and the healthy record
#: (worst window 0.0667) has margin at either setting.
_CLAMP_DUTY_MIN_SAMPLES = 50

#: The summary token published on /link_status, highest active condition first.
#: The order is CAUSAL, not severity: the ring leak is the earliest precursor
#: (it is the mechanism the S3 soak convicted), the cache-age floor is the
#: stale-feedback symptom downstream of it, and the clamp duty is the last and
#: most visible link in the same chain. Naming the most UPSTREAM active
#: condition points the operator at the one whose fix subsumes the others.
LATENCY_MONITOR_OK = 'OK'
LATENCY_MONITOR_RING_LEAK = 'RING_LEAK'
LATENCY_MONITOR_CACHE_AGE = 'CACHE_AGE'
LATENCY_MONITOR_CLAMP_DUTY = 'CLAMP_DUTY'

#: The same precedence as an ordinal, so the log throttle can tell an ESCALATION
#: (a newly-active, more upstream cause — the operator's next action changes)
#: from a de-escalation or a flap (it does not). An escalation bypasses the
#: throttle; nothing else does, which bounds the bypasses at one per step up the
#: chain rather than one per threshold crossing.
_LATENCY_MONITOR_RANK = {
    LATENCY_MONITOR_OK: 0,
    LATENCY_MONITOR_CLAMP_DUTY: 1,
    LATENCY_MONITOR_CACHE_AGE: 2,
    LATENCY_MONITOR_RING_LEAK: 3,
}


#: Bound on consecutive /robot_state freshness-gate skips (~0.5 s at the 100 Hz
#: timer). Rationale at the gate in _publish_robot_state: telemetry and
#: heartbeats come from separate firmware tasks, so "link up" does not prove
#: telemetry flows — an unbounded gate could silence the orchestrator's
#: fault-content channel in that double-failure.
_ROBOT_STATE_MAX_CONSEC_SKIPS = 50


# ── Config identity ────────────────────────────────────────────
def hardware_config_identity() -> str:
    """Identify the hardware_config module this PROCESS actually imported.

    Production is build-frozen: `jugglebot.hardware_config` resolves to the
    colcon-INSTALLED copy, not the repo source and certainly not
    `config/hardware_config.yaml`. Editing the YAML and relaunching therefore
    changes nothing, silently — the single most confusing failure mode on this
    robot ("I changed the gain and it did nothing"). Logging the resolved path,
    a content hash and the mtime at boot turns that into something an operator
    can check against a bag or a screenshot after the fact.

    Reads `hw.__file__` rather than recomputing a path so the answer is the
    module that is really bound, whatever PYTHONPATH did.

    Deliberately scoped to hardware_config only (the tuning surface).
    `friction_ff_params.py`'s env -> ament-share -> source-tree resolution
    order is a landed 2026-06-24 crash fix in motor_guard's import chain and is
    NOT touched here — see plans/parked/refactor-2026-07.md Phase 5 item 3.

    Never raises: a diagnostic must not be able to stop the bridge booting.
    """
    try:
        path = getattr(hw, '__file__', None)
        if not path:
            return 'hardware_config: UNKNOWN source (module has no __file__)'
        with open(path, 'rb') as f:
            digest = hashlib.sha256(f.read()).hexdigest()
        # %z, not a bare local timestamp: the whole point is reconciling a bag
        # or a screenshot against a specific artifact after the fact, and an
        # offset-less local time is ambiguous across a DST change or a
        # differently-configured box.
        stamp = time.strftime('%Y-%m-%dT%H:%M:%S%z',
                              time.localtime(os.path.getmtime(path)))
        return ('hardware_config: %s sha256=%s mtime=%s'
                % (path, digest[:16], stamp))
    except Exception as exc:  # noqa: BLE001 — identity is best-effort
        return ('hardware_config: identity unavailable (%s: %s)'
                % (type(exc).__name__, exc))


# ── Install-skew self-check ────────────────────────────────────
#
# THE INCIDENT THIS CLOSES (2026-08-14, S3 of bridge-temporal-trustworthiness).
# The operator flashed can-bridge FW 13, relaunched, and the conviction bag
# recorded no /ring_diag at all: `colcon build` had not taken effect, so this
# node ran from a two-day-old `install/` tree that predates the RING_DIAG
# subscribe. BRIDGE_FW_CHECK reported OK throughout — and it was RIGHT to,
# which is the whole problem: it compares the board's FW_VERSION against
# `rpc_args.EXPECTED_BRIDGE_FW_VERSION` read from the LIVE repo-root
# `teensy_link/` tree (injected on PYTHONPATH by the launch, deliberately NOT
# colcon-installed), so it is structurally blind to the currency of the NODE's
# own build. Firmware currency and host currency are two independent halves and
# only one of them had a detector.
#
# jugglebot_launch.py already carries the sibling check for the GENERATED CONFIG
# modules (`_install_drift`, link B) — but its scope is exactly
# `hardware_config.py` + `protocol_config.py`, so a stale node source with
# unchanged config passes it green. This check covers the node's own source.
#
# ADVISORY ONLY, same policy as BRIDGE_FW_CHECK and the launch banner: a stale
# install is a CONFUSION failure, not a danger one (build-frozen is the safe
# direction), and putting a hash comparison in front of the leg/hand command
# path would convert a reporting gap into an outage.
_SOURCE_TREE_REL = ('ros_ws', 'src', 'jugglebot', 'jugglebot',
                    'teensy_bridge_node.py')

#: The three verdicts, never collapsed. 'unknown' is NOT a synonym for clean —
#: a check that could not run must read differently from one that ran and
#: agreed, or the row manufactures reassurance (the launch banner's
#: PARTIAL-vs-OK rule, same reasoning).
INSTALL_SKEW_CLEAN = '0'
INSTALL_SKEW_STALE = '1'
INSTALL_SKEW_UNKNOWN = 'unknown'


def _repo_source_copy(running_path: str) -> str | None:
    """Locate the repo-source copy of this module, or None.

    Resolution: ``JUGGLEBOT_REPO`` override (EXCLUSIVE when set — a missing
    file under it yields None/unknown, never a fall-through to a different
    tree), else walk up from the RUNNING file.

    The walk is what makes this work under colcon: the running file is
    ``<repo>/ros_ws/install/jugglebot/lib/python3.N/site-packages/jugglebot/
    teensy_bridge_node.py``, so the first ancestor that carries
    ``ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py`` is the repo that
    produced this install. Anchoring on the compared file itself (rather than on
    `config/generate_config.py`, the launch's anchor) means a resolved root can
    never lack the file the verdict is about. Running live from the source tree
    resolves to the file itself, which is a true clean verdict, not a special
    case.

    The env override mirrors ``jugglebot_launch._repo_root`` so a worktree
    session can pin its own tree. Unlike the launch this does NOT fall back to
    the canonical ``/home/jetson/Desktop/Jugglebot``: for a deploy whose source
    tree is genuinely absent, comparing against whatever unrelated checkout sits
    at a hard-coded path would manufacture a skew verdict about a tree this
    process never came from. An honest ``unknown`` is the weaker claim and the
    correct one.
    """
    override = os.environ.get('JUGGLEBOT_REPO')
    if override:
        # The override is AUTHORITATIVE, never a hint: an operator who pinned a
        # worktree must get a verdict about THAT tree or no verdict at all. A
        # silent fall-through to the walk-up would verdict against a different
        # repo and log CLEAN — the same manufactured-reassurance class this
        # check exists to close (a typo'd override yields `unknown` naming the
        # override path, not a plausible-looking answer about the wrong tree).
        cand = os.path.join(override, *_SOURCE_TREE_REL)
        return cand if os.path.isfile(cand) else None
    roots = []
    here = os.path.dirname(os.path.abspath(running_path))
    while True:
        roots.append(here)
        parent = os.path.dirname(here)
        if parent == here:
            break
        here = parent
    for root in roots:
        cand = os.path.join(root, *_SOURCE_TREE_REL)
        if os.path.isfile(cand):
            return cand
    return None


def install_skew_verdict(running_path: str | None = None) -> tuple[str, str]:
    """Compare the RUNNING module's source against the repo-source copy.

    Returns ``(verdict, detail)`` where verdict is one of the three
    ``INSTALL_SKEW_*`` tokens and detail is the operator-facing explanation
    (both paths and their mtimes on a skew; the reason on ``unknown``).

    Content hash, not mtime: colcon copies the ``.py`` verbatim, so equal bytes
    is exactly "this process is running the code the repo says it is", while
    mtimes differ on every build and would false-positive constantly. The mtimes
    are REPORTED (never compared) because they are what tells the operator which
    side is behind, which is the difference between "run colcon build" and
    "your checkout is stale".

    Never raises: a diagnostic must not be able to stop the bridge booting.
    """
    try:
        running = os.path.abspath(running_path if running_path is not None
                                  else __file__)
        source = _repo_source_copy(running)
        if source is None:
            override = os.environ.get('JUGGLEBOT_REPO')
            if override:
                return (INSTALL_SKEW_UNKNOWN,
                        'JUGGLEBOT_REPO=%s carries no %s — the override is '
                        'authoritative, so no other tree was consulted — '
                        'install currency NOT checked' %
                        (override, os.path.join(*_SOURCE_TREE_REL)))
            return (INSTALL_SKEW_UNKNOWN,
                    'no repo source tree found from %s (deploy without a '
                    'checkout, or an unexpected install layout) — install '
                    'currency NOT checked' % running)
        with open(running, 'rb') as f_run:
            run_digest = hashlib.sha256(f_run.read()).hexdigest()
        with open(source, 'rb') as f_src:
            src_digest = hashlib.sha256(f_src.read()).hexdigest()
        if run_digest == src_digest:
            return (INSTALL_SKEW_CLEAN,
                    'running %s matches repo source %s (sha256=%s)'
                    % (running, source, run_digest[:16]))
        return (INSTALL_SKEW_STALE,
                'running %s (mtime %s) DIFFERS from repo source %s (mtime %s)'
                % (running, _mtime_stamp(running), source,
                   _mtime_stamp(source)))
    except Exception as exc:  # noqa: BLE001 — the check is best-effort
        return (INSTALL_SKEW_UNKNOWN,
                'install check errored (%s: %s) — install currency NOT checked'
                % (type(exc).__name__, exc))


def _mtime_stamp(path: str) -> str:
    """Local mtime with a UTC offset (see hardware_config_identity on why %z)."""
    try:
        return time.strftime('%Y-%m-%dT%H:%M:%S%z',
                             time.localtime(os.path.getmtime(path)))
    except OSError:
        return 'unknown'


# ── RING_DIAG delivery-lag normaliser (S3 residual (a)) ────────
#
# THE RESIDUAL THIS CLOSES. FW 13's RING_DIAG delivery-lag integral
# (``lag_now_us``) read 151-183 ms on the S3 conviction bag and CREPT ~0.35 ms/s
# across it, against a naive expectation of 129 ms — an absolute value the entry
# recorded as "the trend is the instrument, the absolute is not", with the creep
# explicitly unexplained (logbook/2026-08-14-s3-conviction-ring-leak-measured.md
# § "Why the residuals are recorded rather than resolved", residual (a)).
#
# THE MECHANISM, measured 2026-08-15 across four bags to <1 %: the FlexCAN
# hardware capture clock runs SLOW against ``micros64()``. Per window the ratio
# ``cap_span_us / window_us`` measures that rate directly, and it is strongly
# LOAD-DEPENDENT: ~237 ppm with no setpoint stream (bag 10-06-14) against
# ~582 ppm while streaming (bag 10-08-06), and the same step appears WITHIN one
# bag (00-44-59: windows 0-27 at ~0.117 us/frame, windows 28-38 at
# 0.34-0.46). So the integral's absolute value is (time since the last reseed) x
# (whatever the rate was doing over that time) and is NOT delivery latency: it
# exceeded the 135 ms one-lap physical cap on a plant whose leak was
# identically 0.
#
# WHY THE OBVIOUS CORRECTION IS A TAUTOLOGY, and is deliberately NOT what this
# implements. ``lag_now_us`` is, in the firmware, exactly
# ``(micros64 since seed) - (arrival clock since seed)``, and ``window_us`` /
# ``cap_span_us`` are the per-window advances of those SAME two clocks
# (can_buses.cpp::jb_lag_fold, telemetry.cpp::ring_diag_uplink_step). Therefore
#
#     lag_now - lag_prev  ==  window_us - cap_span_us     (identity, +/- one
#                                                          inter-frame gap of
#                                                          sampling offset)
#
# and subtracting the MEASURED per-window divergence from the measured
# per-window lag change yields ~0 for every input — a number that would read
# "healthy" through a fully-leaked ring. The two S3 numbers are the same number
# seen twice: the creep of 0.35 ms/s and the ratio of 230-665 ppm agree because
# they are algebraically one quantity, not because two instruments concurred.
# A real delay line is not exempt: stranding N frames means N frames' worth of
# capture time is NOT folded in that window, so a genuine lag step lands in
# ``window_us - cap_span_us`` identically to the clock artefact. Nothing in this
# pair of fields can separate them.
#
# WHAT SEPARATES THEM is a MODEL plus an independent channel. The artefact is
# proportional to folded frames and is present always; a delivery lag is
# proportional to stranded frames, is bounded by one 256-slot ring lap, and is
# visible in the OCCUPANCY channel (``true_depth`` vs ``avail_reported``), which
# is derived from the ring indices and not from any clock. So:
#
#   1. CALIBRATE the artefact rate ONLY on windows the instrument itself calls
#      clean (the jugglebot spot leak within RING_LEAK_WARN_FRAMES and NO fresh
#      advance of its leak high-water this window). On those windows no
#      stranding is accumulating, so the whole divergence is artefact, by the
#      same reasoning the 2026-08-15 forensics used when it read the rate off a
#      leak-free plant.
#   2. SUBTRACT ``rate x frames folded`` from the raw integral, PER WINDOW and
#      accumulated. A leaking window is EXCLUDED from (1) but still accumulated
#      in (2), which is precisely why a real lag step survives the correction
#      while the clock drift does not.
#
# THE RATE IS TRAILING, NOT POOLED — the correction's single most important
# property, and the one an earlier draft of this class got wrong. The artefact
# is load-dependent (~237 ppm quiet / ~582 ppm streaming, measured) while the
# FOLD RATE IS NOT: ``jb_lag_fold`` folds jugglebot-bus RX frames (ODrive
# broadcasts, ~1950 per window in EVERY bag, streaming or not) and the 500 Hz
# setpoint stream is TX, which is never folded. Both the ppm figure and the
# per-frame figure therefore step by the SAME factor across a load change
# (x2.46 and x2.47 in the two bags above) — per-frame and per-second are one
# normaliser up to the constant ~1950, and choosing between them buys nothing.
# What DOES matter is that a rate pooled since node start applies a quiet-plant
# 0.1215 us/frame to a streaming plant whose true rate is 0.3000: an
# under-correction of ~348 ppm which, over a segment running to the firmware's
# |lag| > 200 ms forced reseed (~344 s at 582 ppm), leaves ~120 ms of
# uncorrected artefact — i.e. it MANUFACTURES a one-lap ring delay (114-135 ms)
# in the row labelled "the row to trend", on a plant with leak identically 0.
# A trailing estimator over _LAG_CAL_TRAIL_WINDOWS bounds that residual by the
# estimator's own lag instead: ~348 ppm x half the trailing span ≈ 10 ms.
#
# Per-frame is kept as the INTERNAL unit because the divergence and the fold
# count arrive in the same frame and it degrades gracefully if the fold rate
# ever does move; it is NOT a load-independence claim. ``lag_corr_rate_ppm`` is
# published in the forensics' units as a REPORT.
#
# The raw ``lag_now_us`` row is published UNCHANGED beside the corrected one.
# The correction rests on a calibration this class performs live; a reader must
# always be able to see the number the firmware actually sent.

#: Trailing span of the rate estimator, in eligible windows. Long enough that
#: the ~11 us/window leak ratchet stays buried in the per-window noise (see
#: _LAG_CAL_MIN_WINDOWS) and cannot be tracked out as if it were clock rate;
#: short enough that a load change is followed within a minute, which bounds the
#: post-transition residual at ~10 ms rather than the ~120 ms a pooled estimator
#: accrues. Do NOT shorten below ~30.
_LAG_CAL_TRAIL_WINDOWS = 60

#: Eligible windows required before a rate is published at all.
#:
#: A NOISE argument, and the noise is real: ``window_us`` is emit-to-emit but
#: ``cap_span_us`` is FOLD-to-fold (telemetry.cpp differences ``lag.arrival_us``,
#: whose value is stamped at the last folded frame), so every window's
#: divergence carries +/- one inter-frame gap. Measured on the quiet bag
#: 10-06-14: sigma ~12 us on a ~229 us divergence (per-window rate spread
#: -0.2393 to +0.4756 us/frame around a 0.1215 mean). Averaging improves as
#: 1/sqrt(N), so 30 windows put the rate's own error near 2 us/window — an order
#: below the artefact it estimates. Below this bar the corrected row renders
#: 'n/a'; an uncalibrated correction must not render as a measured zero.
_LAG_CAL_MIN_WINDOWS = 30

#: Sanity bound on a single window's |window_us - cap_span_us|, as a fraction of
#: the window. The artefact is 230-670 ppm and a one-lap delay step is ~135 ms;
#: 1 % (10 ms in a 1 s window) is far above the former and is only reached by a
#: corrupt or saturated sample, which must not be allowed to set the rate.
_LAG_CAL_MAX_DIVERGENCE_FRAC = 0.01


class LagClockNormalizer:
    """Turn RING_DIAG's raw lag integral into a clock-artefact-free number.

    One instance per node, driven from the 1 Hz ``/ring_diag`` drain. Pure
    Python, no ROS — the whole point is that it is exercisable from a test
    without a bridge (see tests/ros/test_teensy_bridge_node_lag_normalizer.py).

    Contract, in three parts:

    * **Calibration** holds the last ``_LAG_CAL_TRAIL_WINDOWS`` windows the
      caller flagged clean and reports ``rate_us_per_frame`` once
      ``_LAG_CAL_MIN_WINDOWS`` have EVER been seen. TRAILING, not pooled: the
      artefact tracks load and a pooled rate would silently under-correct a
      streaming session by ~348 ppm (the header carries the arithmetic). It is
      never reset by a reseed — a reseed moves the LAG series' zero, not the
      crystal's rate — and it FREEZES rather than expiring when clean windows
      stop arriving, so the correction keeps running at the last rate the plant
      actually justified.
    * **Segmentation** restarts the corrected series at 0 whenever the raw
      series' reference moves (a reseed) or the node's view of it breaks (a
      ``seq`` gap = a dropped uplink frame, an unseeded arrival clock). Imputing
      across a hole would silently widen the frame count and under-correct.
    * **Correction** is accumulated PER WINDOW:
      ``corrected += (lag_now - lag_prev) - rate_now x lag_frames``. Per window
      and not as one subtraction over the whole segment, because the rate is
      time-varying: applying today's rate to an hour of history would
      re-introduce exactly the error the trailing estimator exists to avoid.
      Accumulation starts at the window where the rate first exists, so the
      value is growth since the later of (segment start, calibration).
    """

    def __init__(self):
        # Trailing calibration window: (divergence_us, frames, window_us) per
        # eligible sample, oldest evicted automatically.
        self._cal = deque(maxlen=_LAG_CAL_TRAIL_WINDOWS)
        # Cumulative count, for the warm-up gate ONLY — the estimate itself
        # never sees more than the trailing span.
        self._cal_seen = 0
        # Segment state.
        self._seq_prev: int | None = None
        self._prev_lag_us: int | None = None
        self._base_t_local_us = 0
        self._segment_frames = 0
        self._segment_t_local_us = 0
        self._corrected_us = 0.0

    @property
    def cal_windows(self) -> int:
        """Eligible windows currently BACKING the rate (saturates at the span)."""
        return len(self._cal)

    @property
    def rate_us_per_frame(self) -> float | None:
        """Calibrated artefact, microseconds of drift per folded frame."""
        if self._cal_seen < _LAG_CAL_MIN_WINDOWS or not self._cal:
            return None
        frames = sum(c[1] for c in self._cal)
        if frames <= 0:
            return None
        return sum(c[0] for c in self._cal) / float(frames)

    @property
    def rate_ppm(self) -> float | None:
        """The same calibration expressed as the forensics' ppm. Report only."""
        if self._cal_seen < _LAG_CAL_MIN_WINDOWS or not self._cal:
            return None
        span = sum(c[2] for c in self._cal)
        if span <= 0:
            return None
        return 1e6 * sum(c[0] for c in self._cal) / float(span)

    def update(self, *, seeded: bool, reseed_in_window: bool, seq: int,
               t_local_us: int, window_us: int, cap_span_us: int,
               lag_now_us: int, lag_frames: int, ring_clean: bool) -> dict:
        """Fold one RING_DIAG window in; return the rows to render.

        ``ring_clean`` is the caller's verdict that no stranding was
        accumulating in this window (the jugglebot SPOT leak within
        ``RING_LEAK_WARN_FRAMES`` and no fresh advance of its leak high-water).
        It gates CALIBRATION only — a dirty window is still accumulated into the
        correction, which is what lets a real delivery-lag step survive it.

        Returns a dict with ``corrected_us`` (float or None), ``state``,
        ``segment_us``, ``segment_frames``, and the rate reports.
        """
        # ── 1. Calibration ────────────────────────────────────────────────
        # Self-contained in this window: window_us and cap_span_us both describe
        # THIS window, so eligibility needs no continuity with the previous
        # sample and a dropped uplink frame costs one window, not the estimate.
        if (seeded and not reseed_in_window and ring_clean
                and window_us > 0 and cap_span_us > 0 and lag_frames > 0):
            div = int(window_us) - int(cap_span_us)
            if abs(div) <= _LAG_CAL_MAX_DIVERGENCE_FRAC * window_us:
                self._cal.append((div, int(lag_frames), int(window_us)))
                self._cal_seen += 1

        # ── 2. Segmentation ───────────────────────────────────────────────
        # Wrap-safe: seq is a u32 emitted-frame counter. Contiguity is the only
        # proof that this window's lag_frames covers ALL the frames folded since
        # the previous sample.
        contiguous = (self._seq_prev is not None
                      and self._prev_lag_us is not None
                      and ((int(seq) - self._seq_prev) & 0xFFFFFFFF) == 1)
        self._seq_prev = int(seq)

        if not seeded:
            # No reference exists on the wire, so none exists here either. The
            # previous lag is dropped so the first seeded window starts fresh.
            self._prev_lag_us = None
            self._segment_frames = 0
            self._corrected_us = 0.0
            return self._result(None, 'unseeded', 0, 0)

        rate = self.rate_us_per_frame
        if reseed_in_window or not contiguous:
            self._base_t_local_us = int(t_local_us)
            self._segment_frames = 0
            self._segment_t_local_us = 0
            self._corrected_us = 0.0
            state = 'segment_start'
        else:
            self._segment_frames += int(lag_frames)
            self._segment_t_local_us = int(t_local_us) - self._base_t_local_us
            # ── 3. Correction, accumulated at THIS window's rate ──────────
            # Nothing accumulates before the rate exists: a correction applied
            # with a rate that does not yet exist is not a smaller correction,
            # it is a raw drift wearing the corrected row's name.
            if rate is not None:
                self._corrected_us += ((int(lag_now_us) - self._prev_lag_us)
                                       - rate * int(lag_frames))
            state = 'ok'
        self._prev_lag_us = int(lag_now_us)

        if rate is None:
            return self._result(None, 'uncalibrated',
                                self._segment_t_local_us, self._segment_frames)
        return self._result(self._corrected_us, state,
                            self._segment_t_local_us, self._segment_frames)

    def _result(self, corrected, state, segment_us, segment_frames) -> dict:
        return {
            'corrected_us': corrected,
            'state': state,
            'segment_us': segment_us,
            'segment_frames': segment_frames,
            'rate_us_per_frame': self.rate_us_per_frame,
            'rate_ppm': self.rate_ppm,
            'cal_windows': self.cal_windows,
        }


# ── Constants ──────────────────────────────────────────────────
# Mirror of can_node._HEARTBEAT_TIMEOUT_S: the can-bridge link is declared lost
# after this long without a T→J heartbeat. 2 s matches the CAN watchdog so the
# two nodes agree on what "lost" means during the side-by-side window.
_HEARTBEAT_TIMEOUT_S = 2.0

# Cadence of the persistent "TEENSY GUARD LATCHED" reminder while a guard fault
# stays latched. The fault EDGE logs once (loud, detailed); this repeats every
# _GUARD_LATCH_REPEAT_S so a latched E-STOP stays unmissable in a long log
# instead of scrolling out of sight after a single line. Enforced off a
# monotonic last-log timestamp (rate-EXACT) rather than rclpy
# throttle_duration_sec, whose first-call bookkeeping can silently drop the
# initial repeat.
_GUARD_LATCH_REPEAT_S = 5.0

# HeartbeatJ2T.flags bit0 = mpc_active (guard ENABLED). Sending this set tells
# the Teensy the Jetson is driving setpoints. It MUST stay 0 unless the operator
# has explicitly enabled setpoint output (Commit 3).
_FLAG_MPC_ACTIVE = 0x1

# Staleness window for the leg_setpoint_echo GUI topic: if no setpoint frame has
# been ACCEPTED by the pump within this window, the echo publishes NOTHING — so a
# stopped stream reads as silence downstream (the GUI gaps the dashed Pos (cmd)
# series out), never a stale flatline at the last commanded value.
_SETPOINT_ECHO_STALE_S = 0.5

# HeartbeatT2J.flags bits (T→J), per the protocol SPEC.
_T2J_FLAG_TIME_SYNCED = 0x1
_T2J_FLAG_STOW_PENDING = 0x2
_T2J_FLAG_ALL_AXIS_HB_OK = 0x4
# bit3: firmware-side mpc_active (udp_protocol HeartbeatFlagsT2J.MPC_ACTIVE) —
# the arm-took verification bit. Surfaced on /link_status as 'teensy_mpc_active'
# (ARMING_CONTRACT A5) so a host-armed / firmware-not-armed split is visible.
_T2J_FLAG_MPC_ACTIVE = 0x8
# Bits 4-5 carry the cone (CAN2) BusHealth value (generated single source:
# p.HeartbeatT2JFlags.CONE_HEALTH_MASK / p.HEARTBEAT_CONE_HEALTH_SHIFT).
# BusHealth.UNKNOWN == 0, so a flash older than the cone-health uplink reads
# as UNKNOWN — surfacing it is backward-compatible with an unflashed bridge.
_T2J_CONE_HEALTH_MASK = 0x30
_T2J_CONE_HEALTH_SHIFT = 4
# Bits 8-13 carry the per-leg torque_ff ingest-clamp mask (bit 8+i = leg i clamped
# at UDP ingest on the last ACCEPTED setpoint) — generated single source:
# p.HeartbeatT2JFlags.TORQUE_CLAMP_MASK / p.HEARTBEAT_TORQUE_CLAMP_SHIFT.
# Reads 0 from a firmware older than the 2026-07-14 ingest clamp (bits unused
# there), so surfacing it is backward-compatible with an unflashed bridge.
_T2J_TORQUE_CLAMP_MASK = int(p.HeartbeatT2JFlags.TORQUE_CLAMP_MASK)
_T2J_TORQUE_CLAMP_SHIFT = int(p.HEARTBEAT_TORQUE_CLAMP_SHIFT)

# HandSensor.flags bits (T→J, hand ball-present sensor) — generated single
# source: p.HandSensorFlags. An older bridge never sends the frame
# (never-seen ⇒ UNKNOWN).
_HAND_SENSOR_FLAG_RAW_HELD = int(p.HandSensorFlags.RAW_HELD)
_HAND_SENSOR_FLAG_DEBOUNCED_HELD = int(p.HandSensorFlags.DEBOUNCED_HELD)
_HAND_SENSOR_FLAG_VALID = int(p.HandSensorFlags.VALID)
_HAND_SENSOR_FLAG_STALE = int(p.HandSensorFlags.STALE)
_HAND_SENSOR_FLAG_TIME_SYNCED = int(p.HandSensorFlags.TIME_SYNCED)

# Jetson-side RX-age window for HAND_SENSOR. The bridge sends a 1 Hz keepalive
# whenever no new good reply lands, so ~3 missed keepalives means the FRAME
# SOURCE died — not that the sensor went quiet. Measured against the Jetson's
# own monotonic clock at RX, NEVER against HandSensor.t_bridge_us (a foreign
# wall clock that can step). Without this gate a dead bridge or a dropped link
# would leave the free-running 100 Hz hand_telemetry timer republishing
# ball_held=True, ball_held_valid=True from cache forever.
_HAND_SENSOR_RX_FRESH_S = 3.0

# HeartbeatT2J.bb_flags bits (T→J, Ball Butler cutover).
_T2J_BB_FLAG_BALL_IN_HAND  = 0x1
_T2J_BB_FLAG_HEARTBEAT_SEEN  = 0x2
_T2J_BB_FLAG_HEARTBEAT_STALE = 0x4

# Axis layout (from the generated protocol): legs 0..5, hand = 6, NUM_AXES = 7.
_NUM_LEGS = p.NUM_LEGS
_HAND_AXIS = p.NUM_LEGS         # 6
_NUM_AXES = p.NUM_AXES          # 7

# Ball Butler ODrive axes (CAN1 node ids). NOT part of the 7-axis Telemetry
# frame — their pos/vel ride BB_AXIS_ESTIMATES and their diagnostics ride
# DIAGNOSTIC frames with these axis_ids. When live, they are appended to
# robot_state.motor_states as trailing entries [7]=pitch, [8]=hand (the
# can_node-era 9-axis layout the GUI charts/fault-viz index positionally).
_BB_PITCH_AXIS = proto.NODE_ID_BB_PITCH   # 7
_BB_HAND_AXIS = proto.NODE_ID_BB_HAND     # 8

# DIAGNOSTIC.flags bits, mirrored by hand from the generated udp_protocol spec.
# Deliberately not version-stamped: a stamped comment rots on every bump (this
# one still said "PROTOCOL_VERSION 4" after the 4→5 bump). The spec is the
# source of truth; these bits have been stable since they were introduced.
_DIAG_FLAG_HB_STALE = 0x1   # ODrive CAN heartbeat older than CAN_HEARTBEAT_TIMEOUT
_DIAG_FLAG_HB_SEEN = 0x2    # ODrive has heartbeated at least once this firmware boot
# BB-axis liveness windows for the robot_state append. BB diag is a fixed 1 Hz
# per axis (send_bb_diag), so 3 s = three missed frames; BB estimates ride the
# 100 Hz telemetry tick, so 0.5 s of silence means the stream (not the bus) died.
_BB_DIAG_FRESH_S = 3.0
_BB_EST_FRESH_S = 0.5

# "Already at STOW" band for the deactivate no-op guard: a leg is considered stowed
# when it is IDLE and within this of the STOW pose (DeactivateMonitor stow_rev = 0.0,
# so |pos| <= this). 0.2 rev sits just above the monitor's 0.15 arrival tolerance to
# allow the small foam-relaxed drift a leg takes the instant it IDLEs.
_STOW_POS_MAX_REV = 0.2

# ODrive undervoltage error/disarm bit (matches odrive.ERR_DC_BUS_UNDER_VOLTAGE).
_ERR_DC_BUS_UNDER_VOLTAGE = 512

# ODrive CLOSED_LOOP axis state (matches odrive.AXIS_STATES['CLOSED_LOOP']).
_AXIS_STATE_CLOSED_LOOP = 8

# ── ODrive error decode (F3) ──────────────────────────────────────────────
# The full 32-bit active_errors/disarm_reason masks reach this node on every
# DIAGNOSTIC frame, but until F3 nothing on the Jetson turned them back into
# names: robot_state collapsed them to booleans, the guard-latch hint printed
# only '(leg 3)', and the decode table (odrive.ERROR_CODES / odrive.error_names)
# had ZERO consumers in the launched node graph. These two helpers are the one
# formatting point shared by the three restored consumers (_guard_fault_leg_hint,
# _log_odrive_errors, _publish_robot_state) so the operator sees one shape in the
# shell, the bag and the GUI.
#
# BOTH MASKS, ALWAYS. The 2026-08-10 leg-0 spinout had active_errors == 0 and
# the truth (SPINOUT_DETECTED) only in disarm_reason: active_errors self-heals,
# disarm_reason is sticky until CLEAR_ERRORS. A decoder that reads one mask is
# blind to exactly the class of fault that already bit this robot.
_AXIS_LABELS = {
    _HAND_AXIS: 'hand',
    _BB_PITCH_AXIS: 'bb_pitch',
    _BB_HAND_AXIS: 'bb_hand',
}


def _axis_label(axis: int) -> str:
    """Human label for an ODrive axis id: 'leg 0'..'leg 5', hand, bb_pitch, bb_hand.

    Legs keep the SPACED 'leg N' form the pre-F3 guard hint and the frozen
    MAX_DEVIATION fallback already printed — the operator (and the existing
    tests) read that shape, and one hint string must not mix two leg spellings.
    """
    if 0 <= axis < _NUM_LEGS:
        return f'leg {axis}'
    return _AXIS_LABELS.get(axis, f'axis {axis}')


def _decode_axis_errors(active: int, disarm: int) -> str:
    """'active=[NAME,...] disarm=[NAME,...] 0x<active>/0x<disarm>'.

    Names come from ``odrive.error_names`` (unknown bits survive as one
    ``UNKNOWN(0x…)`` entry rather than being silently eaten). The raw hex pair
    is kept alongside the names so a bag/log line stays decodable even if the
    table later gains a bit this build did not know about.
    """
    act_names = odrive.error_names(int(active))
    dis_names = odrive.error_names(int(disarm))
    return (f'active=[{",".join(act_names)}] disarm=[{",".join(dis_names)}] '
            f'0x{int(active):X}/0x{int(disarm):X}')


# ── Safe-shutdown sequencing (Task 3.3: Ctrl-C must ALWAYS profiled-stow) ──
# on_shutdown runs an ordered, bounded, best-effort sequence: DISARM (mpc_active→0)
# → SETTLE → profiled DEACTIVATE. The settle exists because set_heartbeat_flags(0)
# only takes effect on the heartbeat thread's NEXT tick (≤ 1/HEARTBEAT_HZ away): the
# firmware REJECTS DEACTIVATE while mpc_active=1 (leg_deactivate.cpp guard), so
# firing the DEACTIVATE RPC immediately would race ahead of the flags=0 J→T
# heartbeat and get bounced with ERR_BUS_DOWN — leaving the legs standing. Waiting a
# couple of heartbeat periods lets the disarm land first. Bounded — never hangs.
_SHUTDOWN_DISARM_SETTLE_S = 2.0 / float(p.HEARTBEAT_HZ)   # ≈ 0.2 s at 10 Hz
# Cap the shutdown DEACTIVATE so a stalled descent can't blow the ~8 s teardown
# budget. A healthy profiled stow reaches STOW+IDLE in ~2-4 s; this only bounds the
# stall case (DeactivateMonitor declares FAILED at timeout_s, the poll loop breaks).
_SHUTDOWN_DEACTIVATE_TIMEOUT_S = 6.0

# ── One-call guard recovery (/recover, FIX 3) ──
# Recovery convergence gate: after trajectory_node reseeds its hold at the measured
# encoder, /recover (and the rerouted armed /clear_errors) clears the guard ONLY once
# the streamed u0 is within this of every leg — so the clear cannot re-trip the latch.
# Tightened 0.25 → 0.03 rev (2026-07-11 clear-errors jolt forensics): the old 0.25 was
# 2.5× the firmware 0.10 rev lead clamp, so a "converged" clear could still leave u0 up
# to 0.25 rev off the drifted encoder → the lead clamp SATURATED at re-enable and
# injected pos_gain × 0.10 = 40 × 0.10 = a 4 rev/s velocity step to the −10 A current
# rail on every leg (measured on both clear events). 0.03 rev is well under the lead
# clamp, so the firmware re-enable slew barely engages — the two fixes COMPOSE (Jetson
# minimises the residual; firmware bounds the worst case at the single choke point).
_RECOVER_U0_TOL_REV = 0.03
# Arming pre-check margin — a SEPARATE, deliberately looser gate than the recovery
# convergence one above. Kept at 0.25 rev: arming seeds trajectory_node's hold at the
# measured pose so u0 already sits ≈ the encoder, AND the firmware re-enable slew now
# bounds any residual at the arm edge too, so tightening this would only risk spurious
# arm rejections without a safety gain. (A quarter of the firmware 1.0 rev MAX_DEVIATION
# backstop — was half of the pre-2026-07-16 0.5 rev; the arm gate deliberately stays
# 0.25.) The recovery path does NOT reuse this — a diverged command cleared onto a
# latched guard is exactly what the tight 0.03 gate above must catch.
_ARM_U0_TOL_REV = 0.25
# Bounded block on trajectory_node's reseed reply. The bridge runs a
# MultiThreadedExecutor, so a bounded wait here (in a ReentrantCallbackGroup) is safe.
_RECOVER_RESEED_TIMEOUT_S = 2.0
# Window /recover WAITS for trajectory_node's PROFILED DESCENT to walk the streamed
# u0 down onto the frozen encoder (u0 collapses over ~0.5-3 s at the session limits,
# each 40 Hz knot inside the pump gate — NOT the old instant one-frame reseed). Bounds
# the wait so a descent that never converges (froze in place) refuses rather than
# blocking the executor thread indefinitely.
_RECOVER_VERIFY_TIMEOUT_S = 4.0
# Bounded re-descend retries: the profiled descent targets the encoder SAMPLED AT
# INSTALL time, but during suppression the leg keeps drifting (~0.1 rev, closing the
# frozen lead offset — the Event-1 −0.102 rev plateau). If the descent converges u0
# onto that stale snapshot while the LIVE encoder has moved on, the convergence verify
# (which compares u0 vs live telemetry) plateaus above tol. Each retry re-samples the
# now-current encoder and re-installs the descent, chasing the settling leg; refuse
# after this many so a genuinely stuck leg does not block the executor thread forever.
_RECOVER_MAX_RESEED_ATTEMPTS = 3
# Manual-recovery hint appended to every armed-recovery REFUSAL (F5, 2026-07-11). When
# converge-first cannot complete (reseed timed out / refused / the descent never
# converged onto a stuck leg), the guard is left latched — so the refusal must tell the
# operator the always-available escape: DISARM (set_setpoint_output false → mpc_active=0,
# which has no external dependency and cannot deadlock) and then the plain /clear_errors
# clears straight through. Without this, an operator facing a stuck leg is left guessing.
_MANUAL_RECOVERY_HINT = ("manual recovery: disarm with set_setpoint_output=false, then "
                         "/clear_errors clears directly")

# Decoded Platform-Teensy RobotState (relay read). Fields mirror
# Teensy_code_platform.ino RobotState (is_homed / levelling_complete / pose offset, rad).
RelayRobotState = namedtuple(
    "RelayRobotState",
    ["is_homed", "levelling_complete", "pose_offset_tiltX", "pose_offset_tiltY"])


def _decode_relay_robot_state(data: bytes) -> RelayRobotState:
    """Decode a 0x6E0 RobotState reply exactly as Teensy_code_platform.ino
    decodeStateCANMessage packs it: byte0 flags (bit0 is_homed, bit1 levelling),
    int16 LE pose*1000 about X (bytes 1-2) and Y (bytes 3-4).

    Bytes 5-6 of the same frame carry the Platform Teensy's FW_VERSION and are
    decoded separately by ``rpc_args.decode_platform_fw_version`` — deliberately
    NOT folded into this namedtuple; see ``_platform_fw_version`` in ``__init__``
    for why. Byte 7 is reserved."""
    if len(data) < 5:
        raise ValueError(f"RobotState reply too short: {len(data)} bytes")
    flags = data[0]
    x = int.from_bytes(data[1:3], "little", signed=True)
    y = int.from_bytes(data[3:5], "little", signed=True)
    return RelayRobotState(
        is_homed=bool(flags & 0x1),
        levelling_complete=bool(flags & 0x2),
        pose_offset_tiltX=x / 1000.0,
        pose_offset_tiltY=y / 1000.0)


def _tilt_to_quat(tilt_x: float, tilt_y: float) -> Quaternion:
    """Convert inclinometer tilt readings to a ROS Quaternion — byte-identical to
    ``can_node._tilt_to_quat`` (can_node.py:1667), so ``robot_state.pose_offset_quat``
    carries the SAME value can_node published.

    ``tilt_x`` / ``tilt_y`` are absolute tilt angles (radians), not deltas. The
    conversion is stateless (no accumulation drift)."""
    q_roll = quaternion.from_rotation_vector([-tilt_x, 0, 0])
    q_pitch = quaternion.from_rotation_vector([0, -tilt_y, 0])
    result = (q_roll * q_pitch).normalized()
    quat = Quaternion()
    quat.x = result.x
    quat.y = result.y
    quat.z = result.z
    quat.w = result.w
    return quat


def _enum_name(enum_cls, value: int) -> str:
    """Best-effort enum-member name for a wire value (falls back to ``v<n>``)."""
    try:
        return enum_cls(value).name
    except ValueError:
        return f"v{value}"


class _MpcCommandSetpointSource:
    """SUB-only connection to the MPC command PUB (tcp://127.0.0.1:5557).

    **The Jetson-relay→Teensy-side switch.** Previously this read motor_guard's
    *already-interpolated* 500 Hz telemetry on :5556 (the Jetson-relay path); it now reads
    the **40 Hz MPC command stream** on :5557 (``ipc.MPC_COMMAND_ADDR``,
    ``ipc.TOPIC_MPC_CMD``) — the same ``make_mpc_command`` dicts motor_guard
    consumes — so the Teensy does the 500 Hz interpolation (Teensy-side path). motor_guard
    leaves the leg path; its :5556 output simply goes unconsumed.

    Subscribes to the ``mpccmd`` topic specifically (not ``b''``): :5557 also
    carries the HardwarePlant fallback-enable message, which is not a setpoint.

    Deliberately does NOT reuse ``jugglebot.motion.ipc.BridgeIPC`` (that BINDS the
    command PUB on :5555, which ``motion_bridge_node`` already binds — two binds
    on one address fail) and does NOT import ``ipc`` at module scope (``ipc``
    imports zmq/msgpack eagerly; the address/topic literals below mirror
    ``ipc.MPC_COMMAND_ADDR`` / ``ipc.TOPIC_MPC_CMD``). Mirrors BridgeIPC's SUB
    tuning (RCVHWM=2 + drain-to-latest) and the msgpack wire format
    (``[topic, msgpack(dict)]``). zmq/msgpack are imported lazily so a read-only
    / setpoint-disabled bridge has no hard dependency on them.
    """

    def __init__(self, addr: str = 'tcp://127.0.0.1:5557',
                 topic: bytes = b'mpccmd'):   # = ipc.MPC_COMMAND_ADDR / TOPIC_MPC_CMD
        import zmq  # lazy — only when setpoint output is enabled
        self._zmq = zmq
        import msgpack
        self._msgpack = msgpack
        self._ctx = zmq.Context()
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.RECONNECT_IVL, 100)       # match BridgeIPC
        self._sub.setsockopt(zmq.RECONNECT_IVL_MAX, 200)
        self._sub.setsockopt(zmq.RCVHWM, 2)                # bounded queue
        self._sub.connect(addr)
        self._sub.setsockopt(zmq.SUBSCRIBE, topic)         # mpc_cmd frames only
        self._sub.setsockopt(zmq.RCVTIMEO, 0)              # non-blocking

    def recv_latest(self) -> dict | None:
        """Drain the SUB and return the latest MPC command dict, or None."""
        latest = None
        while True:
            try:
                frames = self._sub.recv_multipart(flags=self._zmq.NOBLOCK)
                latest = self._msgpack.unpackb(frames[1], raw=False)
            except self._zmq.Again:
                break
        return latest

    def close(self) -> None:
        try:
            self._sub.close()
            self._ctx.term()
        except Exception:  # noqa: BLE001
            pass


class TeensyBridgeNode(Node):
    """ROS 2 node bridging the can-bridge Teensy UDP link to production topics.

    Args:
        client: An optional already-constructed :class:`TeensyLinkClient`. When
            ``None`` (production), one is built from the node's parameters and
            started. Tests inject a loopback client paired with ``FakeTeensy``.
            When injected, the node does NOT take ownership — it will not stop
            the client on shutdown (the test fixture owns its lifecycle).
    """

    def __init__(self, client: TeensyLinkClient | None = None,
                 setpoint_source=None, boot_state_read: bool = True,
                 stow_on_shutdown: bool = True):
        super().__init__('teensy_bridge_node')

        # On a clean shutdown (Ctrl-C of the launch), profiled-stow the platform
        # before transport teardown — the Teensy-side analogue of can_node.on_shutdown's
        # _gently_move_to_setpoint(0.0, deactivating=True). Default True in
        # production; unit tests (which call on_shutdown against a FakeTeensy that
        # never completes the descent) pass False via _build_paired_node.
        self._stow_on_shutdown = bool(stow_on_shutdown)

        # ── Install-skew self-check ────────────────────────────
        # Runs FIRST, before the RX thread exists: _record_bridge_fw_version
        # renders this verdict from the RX thread, and the check must not be a
        # value that thread can observe half-written. Two file reads, once.
        # One log call per verdict, each at a FIXED severity on its own source
        # line — Foxy's rcutils logger caches severity per line and RAISES on a
        # flip (971d12c), so a single call site with a computed level would be a
        # crash waiting for the first stale install.
        self._install_skew, self._install_skew_detail = install_skew_verdict()
        if self._install_skew == INSTALL_SKEW_STALE:
            self.get_logger().error(
                'INSTALL_SKEW: FAIL — this node is NOT running the repo source. '
                + self._install_skew_detail + '. BRIDGE_FW_CHECK cannot see '
                'this: it compares the flashed firmware against the LIVE '
                'teensy_link tree, so it reads OK while this node runs a stale '
                'build (2026-08-14: an S3 conviction bag recorded no /ring_diag '
                'for exactly this reason). FIX: cd ros_ws && colcon build '
                '--packages-select jugglebot, then RELAUNCH. Nothing is '
                'refused — advisory only.')
        elif self._install_skew == INSTALL_SKEW_UNKNOWN:
            self.get_logger().warning(
                'INSTALL_SKEW: INCONCLUSIVE — ' + self._install_skew_detail
                + '. Treat this node build as UNVERIFIED; it is not evidence '
                'that the install is fresh.')
        else:
            self.get_logger().info(
                'INSTALL_SKEW: OK — ' + self._install_skew_detail)

        # ── Parameters ─────────────────────────────────────────
        self.declare_parameter('teensy_ip', p.TEENSY_IP)
        # SAFETY-CRITICAL: setpoint output is OFF by default. Wired in Commit 3.
        self.declare_parameter('enable_setpoint_output', False)
        self.declare_parameter('heartbeat_timeout_s', _HEARTBEAT_TIMEOUT_S)
        # which axes the encoder_search service runs index
        # search on. Default = all legs; set to e.g. [0] for the standalone-leg
        # bench rig (node 0 = axis 0).
        self.declare_parameter('encoder_search_axes', list(range(p.NUM_LEGS)))
        # which axes the home service homes (sequentially — the
        # firmware homes one axis at a time). Default = all legs + the HAND (axis 6),
        # matching can_node._home_robot (JUGGLEBOT_AXES = legs + hand). The hand homes
        # with Homing::HAND_* params after its gains are applied host-side; a leg-only
        # bench rig sets e.g. [0], which then excludes the hand automatically.
        self.declare_parameter('home_axes', list(range(p.NUM_LEGS)) + [_HAND_AXIS])
        # which axes the configure + activate services act on.
        # configure applies gains + vel/curr limits + POSITION/PASSTHROUGH (the
        # Teensy-side _setup_odrives, for the hand too — closes the "configure the hand"
        # parity gap). activate fires the TRAP_TRAJ move to the active pose (legs
        # only; ACTIVATE is rejected on the hand). Default = all legs + the hand;
        # set to e.g. [0] for the standalone-leg bench rig.
        self.declare_parameter('configure_axes', list(range(p.NUM_LEGS)) + [_HAND_AXIS])
        self.declare_parameter('activate_axes', list(range(p.NUM_LEGS)))
        self.declare_parameter('deactivate_axes', list(range(p.NUM_LEGS)))

        self._heartbeat_timeout_s = float(
            self.get_parameter('heartbeat_timeout_s').value)

        # ── Transport client ───────────────────────────────────
        if client is None:
            teensy_ip = str(self.get_parameter('teensy_ip').value)
            self._client = TeensyLinkClient(
                teensy_addr=(teensy_ip, p.PORT_STREAM))
            self._own_client = True
        else:
            self._client = client
            self._own_client = False
        # start() is idempotent — safe whether the client is fresh or injected.
        self._client.start()

        # Inbound RPC server + the wall-clock anchor (ADR-0008). The Teensy is
        # the time-sync master; we answer its TIME_OF_DAY_QUERY with our
        # CLOCK_REALTIME. Registered unconditionally — it is read-only and safe.
        self._rpc_server = RpcServer(self._client)
        self._tod = TimeOfDayServer(self._rpc_server)

        # Outbound RPC client (used by the service surface in Commit 4).
        self._rpc = RpcClient(self._client)

        # ── Hand config state (byte-identical to can_node) ────────────
        # The gains/limits the bridge applies to the HAND ODrive (axis 6). Seeded
        # from config defaults (odrive.DEFAULT_*), updated by set_hand_gains. Used by
        # the cold-start hand configure/home path (refuse-flash-defaults) + the
        # set_hand_gains service. Mirrors can_node.py:135-137.
        self._hand_gains = dict(odrive.DEFAULT_HAND_GAINS)
        self._hand_vel_limit = odrive.DEFAULT_VEL_CURR['hand_vel']
        self._hand_curr_limit = odrive.DEFAULT_VEL_CURR['hand_curr']
        # Latest sniffed hand command (HAND_CMD_ECHO) → hand_telemetry cmd fields.
        # Mirrors can_node._last_hand_cmd (can_node.py:159).
        self._last_hand_cmd = {'pos': 0.0, 'vel': 0.0, 'tor': 0.0}

        # ── Latest-frame state (written on RX thread, read on ROS thread) ──
        self._lock = threading.Lock()
        self._latest_telemetry: Telemetry | None = None
        # Freshness bookkeeping for the /robot_state honesty gate (see
        # _publish_robot_state). _telemetry_gen advances on every accepted
        # TELEMETRY frame; _robot_state_pub_gen records which generation was last
        # published, so equality means "nothing new arrived since". -1 seeds a
        # value no generation can equal, so the first real publish always goes.
        self._telemetry_gen = 0
        self._robot_state_pub_gen = -1
        self._robot_state_stale_skips = 0
        self._robot_state_consec_skips = 0
        self._latest_heartbeat: HeartbeatT2J | None = None
        # Generation counter on the heartbeat, same idea as _telemetry_gen: the
        # latency monitor samples lead_clamp_mask off the LATEST heartbeat at
        # 10 Hz, and without this it could not tell a fresh heartbeat from the
        # same one read twice. Counting a stale repeat would let a link that
        # died mid-clamp hold the duty at 1.0 forever.
        self._heartbeat_gen = 0
        # Latest HAND_SENSOR frame (hand ball-present switch) + the JETSON
        # MONOTONIC arrival time. None until the first frame: that is the
        # tri-state UNKNOWN, and it is also what an unflashed bridge (no Phase
        # 3/4 firmware) looks like forever. The arrival time is deliberately
        # host-monotonic — t_bridge_us is a foreign wall clock (§ Architecture).
        self._latest_hand_sensor: HandSensor | None = None
        self._latest_hand_sensor_mono = 0.0
        # Latest CAN_ERRORS frame (CAN3 wire-error + fault-confinement counters,
        # 1 Hz from the bridge). Read-only instrument: nothing gates on it — it
        # exists so the CAN3 error CLASS is visible from a rosbag instead of only
        # over a USB serial console (2026-07-29 CAN3 flap, layer 2). A bridge
        # older than FW 5 never sends it, so None is a normal steady state.
        self._latest_can_errors: CanErrors | None = None
        # Latest BRIDGE_TX_DIAG frame (per-bus CAN TX deferral/queue pressure +
        # hand_ops' per-stage exit tally, 1 Hz from the bridge). Read-only
        # instrument on the same terms as _latest_can_errors: cumulative
        # counters, so no arrival stamp is kept — a stale copy is a truthful
        # floor. A bridge older than FW 9 never sends it, so None is a normal
        # steady state and must render as such rather than as zeros.
        self._latest_bridge_tx_diag: BridgeTxDiag | None = None
        # Latest BRIDGE_IDENTITY frame + the version verdict derived from it.
        # The verdict is recorded once per CHANGE (see _record_bridge_fw_version)
        # rather than on every 1 Hz frame, so a skew logs once instead of 3600
        # times an hour; _bridge_fw_version_seen is what makes "same value again"
        # distinguishable from "first value".
        self._latest_bridge_identity: BridgeIdentity | None = None
        self._bridge_fw_version_seen: int | None = None
        # Last fault_state seen on the T→J heartbeat, for edge-triggered logging in
        # _publish_link_status. None = nothing seen yet (so the first NONE is silent).
        self._last_fault_state: int | None = None
        # Monotonic timestamp of the last persistent "GUARD LATCHED" reminder (see
        # _GUARD_LATCH_REPEAT_S). None while no fault is latched; anchored to the
        # fault edge so the first repeat lands _GUARD_LATCH_REPEAT_S after it.
        self._last_guard_latch_log_t: float | None = None
        self._latest_profile: Profile | None = None
        # Per-axis latest Diagnostic (one axis per frame on the wire), plus its
        # host-arrival time (monotonic). The arrival times gate the BB axes'
        # robot_state append (_build_bb_motor_states): BB diag is a fixed 1 Hz,
        # so "no BB diag for 3 s" means the uplink slot died and a frozen stash
        # must not keep the BB entries alive. Platform axes (0..6) don't read
        # their arrival times today (the Telemetry-gate covers them).
        self._latest_diag: dict[int, Diagnostic] = {}
        self._latest_diag_mono: dict[int, float] = {}

        # Catching cone state (cone uplink). Catch events are
        # DISCRETE — every one is queued and published exactly once (the
        # stash-latest pattern above would drop impacts that arrive between
        # timer ticks); heartbeats are state — latest wins. The connected
        # gate mirrors can_node: a cone heartbeat within the CAN timeout.
        self._cone_catch_queue: list = []   # [(decoded CatchEvent, host_arrival_us)]
        self._latest_cone_hb = catching_cone.ConeHeartbeat()
        self._cone_hb_received = False
        self._cone_last_hb_mono = 0.0
        self._cone_hb_timeout_s = proto.CC_HEARTBEAT_TIMEOUT_MS / 1000.0

        # Link-loss deferred-stow latch (the bridge's UDP-link watchdog — the
        # Jetson↔Teensy analog of can_node._watchdog_check; the CAN-side latch is
        # owned by the Teensy firmware). See teensy_link/fault_logic.py.
        self._link_latch = LinkLossLatch()
        self._last_link_lost = False  # edge detector for logging
        # CAN3 (Jugglebot core) bus-health edge detector for the firmware-validation cold-start
        # reconnect re-trigger. None until the first heartbeat; only a recovery from
        # a DEGRADED state (WARN/BUS_OFF → OK) fires the conservative re-read — the
        # boot UNKNOWN→OK first-connection is excluded (the boot read already ran).
        self._last_bus1_health = None
        # The CAN3-reconnect conservative re-read does bounded retries + sleeps
        # (~1.9 s worst case), so it runs on a short-lived daemon thread and never
        # stalls the 100 Hz publish (which shares _health_check's callback group;
        # audit 2026-06-29 MEDIUM). This guard keeps at most one re-read in flight
        # (the recovery edge is one-shot). See _read_cold_start_state_conservative_async.
        self._cold_start_reread_inflight = False

        # ── HAND_TRAJ_CMD ack accounting (host-side, no wire change) ──
        # 20d01e9 (2026-07-24) demoted catch_coordinator's arm-ack failure from
        # warning to debug because it was an expected epidemic (30/51 = 59 % that
        # session, while every attempt eventually armed). That was the right call
        # for log noise and the wrong one for forensics: at the default log level
        # the error code is now dropped entirely, so every rosbag recorded since
        # carries NO evidence the arm ack ever failed — and a bag is what survives
        # a sitting, ~/.ros/log is not. These three counters put the numbers back,
        # in every bag, against the CURRENT bridge (no firmware change).
        #
        # They also split the one distinction the message text cannot: a
        # Teensy-RETURNED status (the firmware ran hand_ops and refused) from a
        # HOST-SYNTHESISED RpcTimeout (no response arrived at all). RpcTimeout
        # subclasses RpcError and carries ERR_TIMEOUT, so BOTH render
        # 'HAND_TRAJ_CMD: ERR_TIMEOUT' — that pooling is exactly what the
        # 2026-08-01 recount had to unpick by hand.
        #
        # Plain ints, no lock: written on the service-callback thread, read by the
        # 10 Hz link_status timer. A torn increment costs one count on a forensic
        # instrument; a lock on the RPC path is not worth that.
        self._hand_traj_calls = 0
        self._hand_traj_fail_teensy = 0
        self._hand_traj_fail_host = 0

        # ── Heartbeat: ALWAYS mpc_active=0 at startup ──────────
        # flags=0 ⇒ mpc_active clear. set_heartbeat_flags(0) AFTER start_heartbeat
        # makes the pin STRUCTURAL: start_heartbeat is a no-op against an already-
        # running heartbeat thread (an injected, pre-started client), so without
        # this explicit clear a stale flags=1 could survive construction. The
        # defensive 0-write closes that — mpc_active=0 on every startup path.
        self._mpc_active = False
        self._client.start_heartbeat(hz=float(p.HEARTBEAT_HZ), flags=0)
        self._client.set_heartbeat_flags(0)

        # RX-thread queues MUST exist before any subscribe() below: the client's
        # RX thread is already live, so a frame arriving between subscribe() and
        # a later __init__ line would hit an unset attribute (startup race).
        self._bb_est_queue = []   # list of BbAxisEstimates, drained by timer
        # Latest BB estimate sample + host-arrival time, for the robot_state BB
        # append (pos/vel source). Kept separate from the queue: the queue is
        # drained (emptied) by _publish_bb_axis_estimates, so "latest" must not
        # depend on drain timing.
        self._latest_bb_est: BbAxisEstimates | None = None
        self._latest_bb_est_mono = 0.0
        # Same queue-then-drain shape for the Teensy's POST-CLAMP executed leg
        # command (LEG_CMD, 100 Hz), drained by _publish_leg_cmd_executed. Also
        # declared before the subscribe block for the same startup-race reason.
        self._leg_cmd_queue = []  # list of LegCmd, drained by timer
        # And again for the can-bridge's per-anchor clock diagnostics (CLOCK_DIAG,
        # ~1 per 30 s), drained by _publish_clock_diag. Same declare-before-
        # subscribe rule. An FW 10 board never sends 0x8F, so on the pre-flash
        # bridge this queue simply stays empty forever and /clock_diag records as
        # a silent topic — the visible-absence the record list explicitly blesses.
        self._clock_diag_queue = []  # list of ClockDiag, drained by timer
        # And once more for the encoder-cache freshness census (CACHE_DIAG,
        # 1 Hz), drained by _publish_cache_diag. Same declare-before-subscribe
        # rule. An FW <= 11 board never sends 0x91, so on a pre-flash bridge this
        # queue stays empty forever and /cache_diag records as a silent topic.
        self._cache_diag_queue = []  # list of CacheDiag, drained by timer
        # And once more for the CAN RX-ring true-occupancy census (RING_DIAG,
        # 1 Hz), drained by _publish_ring_diag. Same declare-before-subscribe
        # rule. An FW <= 12 board never sends 0x92, so on a pre-flash bridge this
        # queue stays empty forever and /ring_diag records as a silent topic.
        self._ring_diag_queue = []  # list of RingDiag, drained by timer
        # Node-side normaliser for RING_DIAG's delivery-lag integral (S3
        # residual (a)). Lives on the node rather than inside the drain because
        # its calibration and its segment base must survive across windows;
        # touched ONLY from _publish_ring_diag (executor thread), never the RX
        # thread, so it needs no lock.
        self._lag_norm = LagClockNormalizer()
        # Previous window's per-bus leak high-water, so a SINCE-BOOT maximum can
        # be turned into a THIS-WINDOW event (see _publish_ring_diag). None
        # until the first window: no baseline, so no advance is claimed.
        self._ring_leak_hwm_prev = None

        # ── Alarmed command-latency monitor (2026-07-24 closure contract) ──
        # Three inputs, all already measured and bagged, none of them previously
        # LOUD. The two diagnostics inputs are written by their own 1 Hz drains
        # and read by the 10 Hz link_status timer; all three are timer callbacks
        # in the node's default (mutually-exclusive) group, so they serialise and
        # need no lock. Each carries its own arrival stamp — a 1 Hz input that
        # stopped arriving must stop asserting its last value.
        self._lm_ring_leak = 0            # worst jugglebot/bb/cone leak, last window
        self._lm_ring_t = 0.0             # monotonic arrival of that verdict
        self._lm_cache_floor_us = 0       # worst per-axis cache age FLOOR, last window
        self._lm_cache_t = 0.0
        # (monotonic, clamped?) samples of lead_clamp_mask, trimmed to
        # _CLAMP_DUTY_WINDOW_S. Sampled ONLY while setpoints are streaming and
        # only once per heartbeat — see _latency_monitor_sample_clamp.
        self._lm_clamp_samples = deque()
        self._lm_last_hb_gen = None
        self._lm_last_sp_frames = None
        self._lm_last_log_t = None        # monotonic; None ⇒ the next onset logs at once
        # The condition the last log line NAMED (not the current one): an
        # escalation past it bypasses the throttle. Never reset to OK on
        # recovery — that would let a threshold flap re-log at 10 Hz.
        self._lm_logged_state = LATENCY_MONITOR_OK

        # ── BB loud command-outcome channel (CMD_RESULT relay) ──
        # One outstanding throw at a time (firmware is serialized → no correlation
        # token). The bb/throw action's execute_callback waits on _bb_throw_event;
        # the RX-thread _on_cmd_result handler stores the firmware outcome and sets
        # it. _bb_throw_active gates a second concurrent goal (rejected in
        # goal_callback). Guarded by _bb_throw_lock; the event lives outside it.
        self._bb_throw_lock = threading.Lock()
        self._bb_throw_event = threading.Event()
        self._bb_throw_active = False
        self._bb_throw_result = None   # (outcome:int, detail0:int, detail1:int)

        # ── Platform-Teensy relay replies (PLATFORM_FRAME) ──
        # The relay READ RPCs (TILT_READ / STATE_READ) only TRIGGER a Platform
        # reply; the reply arrives async as a PLATFORM_FRAME on the RX thread. A
        # relay read clears the latched reply for its can_id, sends the RPC, then
        # blocks on this condition for a fresh reply. Correlation is by
        # (can_id, dlc) — sound only if CAN3 SRX_DIS suppresses the bridge's own
        # 0x6E0 write echo (bench-probe gate; see _await_platform_reply).
        self._platform_reply_cv = threading.Condition()
        self._platform_replies = {}    # can_id -> (data: bytes, dlc: int)
        # Concurrency hardening: serialize whole relay round-trips. Two concurrent
        # relay ops (e.g. a boot/reconnect STATE_READ racing a home's STATE_WRITE)
        # would otherwise (a) clear + await the same can_id's latched reply and
        # cross-correlate, and (b) lose a read-modify-write update
        # (cache-vs-Platform-Teensy divergence). One lock around each op closes both.
        # Held only for the rare (~ms) relay round-trip, never on the 100 Hz path.
        # RLock (re-entrant) so the read-modify-write helpers (_write_is_homed /
        # _write_level_state / _clear_cold_start_state_on_reboot) can hold it across
        # their whole cache-read → wire-write → cache-update sequence — which itself
        # calls relay_write_robot_state (also lock-guarded) — fully closing the
        # lost-update, not just serializing the two wire writes.
        self._relay_lock = threading.RLock()

        # ── Cold-start state cache (relay-sourced is_homed/level/pose) ──
        # The Platform Teensy OWNS the persisted cold-start state; it shares the
        # ODrive supply so it "forgets when they forget" (there is no can-bridge
        # store and no invalidation rule). The bridge READS that state
        # via the relay STATE_READ at boot + on each confirmed CAN3 (UDP-link)
        # reconnect, CACHES it here, and surfaces is_homed / levelling_complete /
        # pose_offset on robot_state FROM THE CACHE — the 100 Hz publish path NEVER
        # does a CAN3 round-trip. The bridge is the SOLE writer and does
        # read-modify-write THROUGH this cache (a homing write preserves levelling +
        # pose, and vice versa). Guarded by self._lock (same as the telemetry cache;
        # written off the publish thread, read on it under a MultiThreadedExecutor).
        # The default is the CONSERVATIVE cold value (is_homed=False) so that, until
        # an authoritative read lands (or if every boot read fails), robot_state
        # forces a re-home — wasteful but SAFE; NEVER the reverse (which could skip
        # homing on an unhomed robot).
        self._cold_start_state = RelayRobotState(
            is_homed=False, levelling_complete=False,
            pose_offset_tiltX=0.0, pose_offset_tiltY=0.0)
        # True once an authoritative relay read (or a bridge-issued write) has set
        # the cache; surfaced on link_status for the operator (the powered bring-up sitting).
        self._cold_start_authoritative = False
        # Platform-Teensy FW_VERSION, read out of bytes 5-6 of the SAME 0x6E0
        # RobotState reply that feeds the cache above (see relay_read_robot_state).
        #   None = no authoritative read has landed yet (we know NOTHING)
        #   0    = the board ANSWERED and carries pre-versioning firmware — i.e. it
        #          has not been flashed since 2026-07-27. A definite verdict, not
        #          an absence: every older firmware zero-fills those bytes.
        #   >=1  = the board's own release number.
        # DELIBERATELY NOT A FIELD OF RelayRobotState: that namedtuple is rebuilt
        # from scratch at three conservative-fallback sites whose meaning is "the
        # ODrive references may be gone" (boot default, total-read-failure fallback,
        # REBOOT_ODRIVES clear). The Platform Teensy stays powered through all
        # three and its firmware cannot change without a flash, so folding the
        # version in would make a REBOOT_ODRIVES silently erase a known-good
        # version and raise a false skew alarm — and would make every future
        # fallback site one more place to remember to preserve it.
        self._platform_fw_version = None
        # encoder_search_complete is DERIVED (no wire field): is_homed OR a search
        # that completed THIS process. This in-session bit tracks the OR term; it is
        # sticky-True for the process once set (exact can_node parity — can_node only
        # ever sets encoder_search_complete True, never clears it mid-run).
        self._encoder_search_done_session = False
        # pose_offset_quat memo: _tilt_to_quat runs the (numpy-allocating) quaternion
        # composition only when the cached tilt CHANGES (it changes on a relay read,
        # not per publish) — can_node likewise computed it on-change, not per publish.
        # Touched only on the publish thread (the timer's MutuallyExclusiveCallbackGroup
        # serializes it), so no lock needed.
        self._pose_quat_key = None
        self._pose_quat_xyzw = (0.0, 0.0, 0.0, 1.0)

        # ── Firmware version validation ──────────────
        # Restores can_node's Get_Version handshake (_handle_get_version:474-495).
        # The firmware sweeps Get_Version + caches the raw versions; the bridge
        # pulls them via GET_AXIS_VERSIONS (a cheap UDP RPC reading a bridge-LOCAL
        # cache — NO CAN3 round-trip) and runs the EXISTING tested
        # MotorStateTracker.validate_group against EXPECTED_HW_VERSIONS, latching:
        #   firmware_validated=True  → orchestrator BOOT proceeds (un-gates the
        #                              is_homed skip, state_machine.py:228-235),
        #   mismatch string latched   → kept firmware_validated=False AND surfaced
        #                              on robot_state.error → orchestrator force-
        #                              FAULT (exact can_node:489-492 / 1085 parity).
        # Until validation lands, firmware_validated stays False (BOOT waits, then
        # FAULTs on BOOT_TIMEOUT_S — can_node's behaviour when versions never come).
        # _versions is used for record_version + validate_group + the per-axis fw
        # rendering (_odrive_fw_versions_str); the latched bools are read on the
        # publish path under self._lock. The tracker needs no lock: the version-poll
        # writer and the link_status reader share the node-default
        # MutuallyExclusiveCallbackGroup, so they never run concurrently.
        self._versions = MotorStateTracker()
        self._firmware_validated = False
        self._firmware_mismatch_error = None     # str once a mismatch is latched (sticky)
        # Get_Version's fourth byte (fw_unreleased) per axis, kept node-local —
        # the tracker stores only the (major, minor, rev) triple.
        self._fw_unreleased = {}

        # ── F3/C2: per-(axis, error-code) log throttle for _log_odrive_errors ──
        # A SECOND tracker, deliberately NOT self._versions. The tracker above is
        # bound to the firmware-version handshake (record_version/validate_group);
        # overloading it would put two unrelated lifetimes on one object and make
        # a future reset of either one silently clobber the other. All this
        # instance is used for is last_error_log_times() + error_log_throttle_sec
        # — the throttle scaffolding can_node used, which survived the port
        # unused. It is touched only from the 10 Hz _publish_link_status timer
        # (node-default MutuallyExclusiveCallbackGroup), so it needs no lock.
        self._error_log = MotorStateTracker()

        # ── Subscriptions to T→J streams (RX-thread callbacks) ──
        self._client.subscribe(int(MsgType.HEARTBEAT_T2J), self._on_heartbeat_t2j)
        self._client.subscribe(int(MsgType.TELEMETRY), self._on_telemetry)
        self._client.subscribe(int(MsgType.DIAGNOSTIC), self._on_diagnostic)
        self._client.subscribe(int(MsgType.PROFILE), self._on_profile)
        self._client.subscribe(int(MsgType.CONE_FRAME), self._on_cone_frame)
        self._client.subscribe(int(MsgType.BB_AXIS_ESTIMATES), self._on_bb_estimates)
        self._client.subscribe(int(MsgType.CMD_RESULT), self._on_cmd_result)
        self._client.subscribe(int(MsgType.PLATFORM_FRAME), self._on_platform_frame)
        self._client.subscribe(int(MsgType.HAND_CMD_ECHO), self._on_hand_cmd_echo)  # hand conduit
        self._client.subscribe(int(MsgType.HAND_SENSOR), self._on_hand_sensor)  # ball-present switch
        self._client.subscribe(int(MsgType.CAN_ERRORS), self._on_can_errors)    # CAN3 wire-error counters
        self._client.subscribe(int(MsgType.BRIDGE_TX_DIAG), self._on_bridge_tx_diag)      # TX pressure + hand stages
        self._client.subscribe(int(MsgType.BRIDGE_IDENTITY), self._on_bridge_identity)    # bridge FW identity
        self._client.subscribe(int(MsgType.LEG_CMD), self._on_leg_cmd)          # post-clamp executed leg cmd
        self._client.subscribe(int(MsgType.CLOCK_DIAG), self._on_clock_diag)    # per-anchor clock + interp occupancy
        self._client.subscribe(int(MsgType.CACHE_DIAG), self._on_cache_diag)    # encoder-cache age + CAN RX ring
        self._client.subscribe(int(MsgType.RING_DIAG), self._on_ring_diag)      # TRUE RX-ring occupancy + delivery lag

        # ── Publishers (production names — leg-side cutover) ──
        # Promoted from the /teensy/* namespace to the production topic names
        # can_node used: with USB-CAN gone (can_node out of the launch), the
        # dual-publisher risk that drove the /teensy/* namespacing
        # is moot, and the GUI / orchestrator / consumers subscribe to these
        # production names — so the rename RECONNECTS them to the bridge.
        self.robot_state_pub = self.create_publisher(
            RobotState, 'robot_state', 10)
        self._warn_if_robot_state_msg_is_stale()
        self.hand_telemetry_pub = self.create_publisher(
            HandTelemetryMessage, 'hand_telemetry', 10)
        self.link_status_pub = self.create_publisher(
            DiagnosticStatus, 'link_status', 10)
        self.profile_pub = self.create_publisher(
            DiagnosticStatus, 'profile', 10)
        # GUI observability: the last ACCEPTED leg setpoint u0 (6 floats, motor
        # revs), source-agnostic — trajectory_node and run_mpc.py both feed the
        # :5557 funnel this echoes. No ROS topic otherwise carries commanded leg
        # positions on the Teensy-side path (the GUI's old datasource,
        # leg_lengths_topic, only publishes while the dormant run_mpc/motor_guard
        # stack streams). See _publish_leg_setpoint_echo for the freshness gate.
        self.leg_setpoint_echo_pub = self.create_publisher(
            Float64MultiArray, 'leg_setpoint_echo', 10)

        # The Teensy's OWN report of what it actually commanded to the leg
        # ODrives: the 500 Hz interp ladder's output (axes[i].target_pos_rev /
        # target_vel_rps) AFTER the lead and stroke clamps, sampled at the 100 Hz
        # telemetry rate and stamped with the bridge wall clock (t_teensy_us,
        # time-synced to the Jetson). NOT the same thing as leg_setpoint_echo
        # above — that is the ACCEPTED u0 knot as it entered the :5557 funnel on
        # THIS box, i.e. the far end of the same wire. The pair is the point:
        #   /leg_setpoint_echo  → what the Jetson asked for, when it asked
        #   /leg_cmd_executed   → what the Teensy commanded, when it commanded it
        #   robot_state         → what the encoders did
        # Those three timelines are the three-way discriminator
        # logbook/2026-07-18-teensy-uptime-tracking-degradation.md needed and did
        # not have: without the middle one, a lag that grows with bridge uptime
        # (10 ms fresh → ~240 ms at 30 h) cannot be attributed to Jetson→Teensy
        # transport vs the interp ladder vs the ODrives, and the entry's Addendum
        # names "LEG_CMD not published" as the first of its telemetry gaps. It is
        # also the input to the alarmed end-to-end command-latency monitor that
        # closure Addendum requires (delivered and validated 2026-08-15 —
        # logbook/2026-08-15-fw14-validated-arc-closed.md; the plan that carried
        # it, bridge-temporal-trustworthiness P0 → P3, is archived).
        self.leg_cmd_executed_pub = self.create_publisher(
            JointState, 'leg_cmd_executed', 50)
        # (_leg_cmd_queue is initialized above, before the subscribe block, to
        # avoid a startup race with the already-live RX thread.)

        # The can-bridge's per-anchor wall-clock discipline sample plus the 500 Hz
        # interp ladder's fallback occupancy (CLOCK_DIAG 0x8F, FW 11), one message
        # per accepted time-of-day anchor: ~1 per 30 s in steady state, ~2 Hz
        # during the pre-first-anchor fast retry.
        #
        # WHY A TOPIC AND NOT A /link_status KeyValue ROW, unlike its 1 Hz
        # siblings can3_errors / bridge_tx_diag: those carry CUMULATIVE counters,
        # where re-rendering the latest value at 10 Hz loses nothing because the
        # operator reads them by DIFFERENCING two captures. This frame is the
        # opposite — a SERIES of independent per-anchor MEASUREMENTS whose whole
        # purpose is a fit over hours (the crystal's ppm and its thermal
        # coefficient, per plans/active/bridge-clock-frequency-discipline.md
        # Phase 1). On a KeyValue row every sample would land in the bag ~300
        # times and the analysis would begin by de-duplicating a string; on a
        # topic each sample is published exactly once, in order. That is the same
        # split already drawn between /profile (latest-wins cache) and
        # /bb/axis_estimates + /leg_cmd_executed (queue→drain series).
        #
        # DiagnosticStatus, matching /profile: this is firmware instrumentation,
        # and the KeyValue keys make the wire fields self-describing in the bag.
        # Its lack of a header stamp is not a gap here — the sample carries its
        # OWN x-axis in t_local_us (the bridge's raw micros64 crystal). A ROS
        # header stamp would be host-arrival time, and a wall stamp would be the
        # very quantity under measurement folded into its own time base.
        self.clock_diag_pub = self.create_publisher(
            DiagnosticStatus, 'clock_diag', 20)

        # The can-bridge's encoder-cache freshness census (CACHE_DIAG 0x91,
        # FW 12), one message per 1 s window. Same queue→drain series shape as
        # /clock_diag above, and for the same reason: each window is an
        # independent MEASUREMENT (a per-axis cache-age floor and peak reduced
        # from ~100 samples), and the conclusion is a trend across an hours-long
        # soak, so a latest-wins row would delete most of the evidence.
        #
        # WHAT IT ANSWERS. S1 (2026-08-12) put the uptime command-latency drift
        # inside the Teensy — RTT flat at 1-3 ms, interp deadline misses 0, heap
        # and CAN throughput flat, ODrives and the Jetson-side link exonerated —
        # leaving one fork: is axes[i].pos_timestamp_us (the encoder cache the
        # leg_interp lead clamp measures `fb` against) going stale with uptime,
        # or does the leg genuinely trail? /robot_state and /leg_cmd_executed
        # BOTH read that cache, so they move together under a stale cache and
        # cannot separate the two. age_min_us can: it is a floor, and a floor
        # that grows with uptime is a cache that is not being written. The
        # per-axis enc_frames counters then say WHOSE fault the stall is —
        # frames arriving through it (stale value on the wire) or not arriving
        # at all (the ODrive's broadcast, or per-axis loss on the bus).
        #
        # No header stamp, again deliberately: the sample carries its own x-axis
        # in t_local_us, which is the bridge's raw micros64 — i.e. its power-on
        # UPTIME, the independent variable of the whole investigation.
        self.cache_diag_pub = self.create_publisher(
            DiagnosticStatus, 'cache_diag', 20)

        # The can-bridge's CAN RX-ring TRUE-occupancy census (RING_DIAG 0x92,
        # FW 13), one message per 1 s window. Same queue-then-drain series shape
        # as /cache_diag and /clock_diag above, and for the same reason: each
        # window is an independent measurement reduced from ~1000 on-chip probes,
        # and the conclusion is a trend across an hours-long soak, so a
        # latest-wins row would delete most of the evidence.
        #
        # WHAT IT ANSWERS. S2 (2026-08-13) killed the encoder-cache AGE
        # hypothesis — the cache was FRESHER at 28 h than at 1.3 h — and left the
        # mechanism as ~100-150 ms bit-identical per-axis telemetry freezes that
        # the freshness-blind lead clamp amplifies. The surviving candidate is a
        # defect in the vendored FlexCAN_T4: events() pops the RX ring BEFORE its
        # NVIC_DISABLE_IRQ guard, so the CAN ISR's `_available++` races the
        # task-side `_available--` one-directionally, increments are swallowed,
        # and the bridge's drain loop exits believing the ring is empty while a
        # residue is stranded — making every later delivery that many frames old,
        # ratcheting with uptime, capped at the 256-slot ring (~114-135 ms).
        # NOTHING ALREADY ON THE WIRE CAN SEE THIS: getRXQueueCount() returns
        # `_available`, so /cache_diag's rx_depth_hwm and rx_cap_hits are computed
        # from the corrupted number itself and read healthy through a fully-leaked
        # ring. This topic carries the ring's TRUE occupancy, taken from head/tail
        # at the instant after the drain, beside the `_available` that the rest of
        # the firmware believes.
        #
        # No header stamp, again deliberately: the sample carries its own x-axis
        # in t_local_us, the bridge's raw micros64 — i.e. its power-on UPTIME,
        # which is the independent variable the leak is theorised to ratchet with.
        self.ring_diag_pub = self.create_publisher(
            DiagnosticStatus, 'ring_diag', 20)

        # ── Ball Butler (cutover, production names) ────
        # Intentional naming deviation from the earlier "all under /teensy/*"
        # convention: with USB-CAN removed, the dual-publisher risk that namespacing was
        # preventing is moot (can_node is gone for BB). The bridge inherits
        # the production names so the GUI / orchestrator / mocap_node /
        # throw_director see no name change across the cutover. (The leg/hand
        # topics + services followed onto production names in the leg/hand cutover.)
        self.bb_heartbeat_pub = self.create_publisher(
            BallButlerHeartbeat, 'bb/heartbeat', 10)

        # ── Catching cone (cone uplink, production names) ────
        # Same naming rationale as bb/*: can_node's cone path is dead — the
        # cone lives on the can-bridge's CAN2, which USB-CAN never sees — so
        # the bridge inherits the production names and the existing consumers
        # (catch_correlation_node, analysis tooling) see no change. CONE_FRAME
        # relays carry the raw CAN payloads; decode reuses the same
        # jugglebot.can.catching_cone helpers can_node used.
        self.catch_event_pub = self.create_publisher(
            CatchEvent, 'cone/catch_event', 10)
        self.cone_heartbeat_pub = self.create_publisher(
            CatchingConeHeartbeat, 'cone/heartbeat', 10)

        # Ball Butler ODrive diagnostics (CAN1 nodes 7=pitch, 8=hand). The bridge
        # emits these as DIAGNOSTIC frames with axis_id 7/8, stashed in
        # self._latest_diag[7|8] by _on_diagnostic; republished as a flat array.
        # Layout: [pitch_fet, pitch_motor, pitch_iq, pitch_state,
        #          hand_fet,  hand_motor,  hand_iq,  hand_state]  (degC, degC, A, enum).
        self.bb_odrive_pub = self.create_publisher(
            Float32MultiArray, 'bb/odrive_diag', 10)

        # Ball Butler high-rate pitch/hand encoder estimates (BB_AXIS_ESTIMATES,
        # CAN1 nodes 7/8) for during-throw diagnostics: launch angle vs commanded
        # pitch, hand launch speed vs commanded. Published as JointState
        # (name=[bb_pitch, bb_hand], position=rev, velocity=rev/s), stamped with
        # the bridge wall-clock at sample time. Every sample is queued in
        # _on_bb_estimates and drained on the executor thread by
        # _publish_bb_axis_estimates (publishing off the RX thread, per the
        # other RX callbacks' contract).
        self.bb_estimates_pub = self.create_publisher(
            JointState, 'bb/axis_estimates', 50)
        # (_bb_est_queue is initialized above, before the subscribe block, to
        # avoid a startup race with the already-live RX thread.)

        # ── RPC service surface (production names — leg/hand cutover) ──
        # ODrive control issued over the can-bridge link via RpcClient. The
        # can-bridge owns CAN3 — legs 0-5 AND the hand ODrive (axis 6). Leg ops
        # target legs/broadcast; the hand surface is registered below.
        # Promoted from /teensy/* to the production names can_node served
        # (encoder_search, odrive_command, set_motor_vel_curr_limits) so the
        # orchestrator's existing service clients reach the bridge; clear_errors,
        # reboot_odrives and home are new bridge ops, named bare for consistency.
        #
        # ── Cold-start verb callback group ────────────────────────
        # Every BLOCKING cold-start verb — the manual encoder_search / home /
        # configure / activate / deactivate services (each a multi-second _run_*
        # poll loop) AND the orchestrator-facing home_motors action /
        # activate_or_deactivate / get_platform_tilt / set_level_state below — runs
        # in ONE ReentrantCallbackGroup so a multi-second move never starves the
        # 100 Hz robot_state publish + heartbeat (those stay in the node's default
        # MutuallyExclusiveCallbackGroup, dispatched on OTHER MultiThreadedExecutor
        # threads). Without this, a blocking verb in the default group serializes with
        # — and stalls — the telemetry timers for the whole move. Reentrancy is safe
        # here: the firmware busy-rejects a second concurrent move and the orchestrator
        # drives strictly sequentially; the one remaining race (two home_motors goals)
        # is closed by _home_action_goal's in-progress guard. reboot_odrives /
        # odrive_command stay in the default group — they are quick single RPCs, not
        # multi-second moves. clear_errors is created LATER, in the /recover
        # ReentrantCallbackGroup, because the armed reroute (F1) blocks multi-second.
        self._coldstart_cbgroup = ReentrantCallbackGroup()

        self.create_service(Trigger, 'reboot_odrives', self._svc_reboot_odrives)
        self.create_service(Trigger, 'encoder_search', self._svc_encoder_search,
                            callback_group=self._coldstart_cbgroup)
        self.create_service(Trigger, 'home', self._svc_home,
                            callback_group=self._coldstart_cbgroup)
        self.create_service(Trigger, 'configure', self._svc_configure,
                            callback_group=self._coldstart_cbgroup)
        self.create_service(Trigger, 'activate', self._svc_activate,
                            callback_group=self._coldstart_cbgroup)
        self.create_service(Trigger, 'deactivate', self._svc_deactivate,
                            callback_group=self._coldstart_cbgroup)
        self.create_service(ODriveCommandService, 'odrive_command',
                            self._svc_odrive_command)
        self.create_subscription(
            SetMotorVelCurrLimitsMessage, 'set_motor_vel_curr_limits',
            self._sub_vel_curr_limits, 10)

        # ── Orchestrator-facing cold-start conduit ────────────────
        # Makes the bridge a drop-in for the retired can_node from the LOCKED
        # orchestrator_node's view — ZERO edits to orchestrator_node / state_machine
        # (behaviour parity, not orchestrator churn). Each of the
        # four wrappers registers the exact (name, type) interface the orchestrator
        # drives cold-start through and delegates to the bridge's existing verbs:
        #   home_motors (HomeMotors action)      → _do_home (homes legs + hand)
        #   activate_or_deactivate (srv)         → _run_activate + _run_configure /
        #                                          _run_deactivate
        #   get_platform_tilt (srv)              → relay_read_tilt (NaN-on-failure)
        #   set_level_state (Float64MultiArray)  → _write_level_state (STATE_WRITE)
        # Registered directly on the bridge (not a separate conduit node/module): the
        # bridge owns CAN3 + the RPC client + these verbs; there is no existing
        # actions/services module to join, and grouping four verbs into one only
        # because they land together would be cohesion-by-implementation-moment (a
        # future teensy_bridge_node split should cut along the relay / hand /
        # cold-start seams instead). The drift-guard test
        # (tests/ros/test_orchestrator_conduit_contract.py) pins this (name, type) set
        # to the orchestrator's declared clients so a future rename fails the suite,
        # not the bench.
        self._home_lock = threading.Lock()
        self._home_in_progress = False   # guards concurrent home_motors goals
        self._home_action = ActionServer(
            self, HomeMotors, 'home_motors',
            execute_callback=self._home_action_execute,
            goal_callback=self._home_action_goal,
            callback_group=self._coldstart_cbgroup)
        self.create_service(
            ActivateOrDeactivate, 'activate_or_deactivate',
            self._svc_activate_or_deactivate,
            callback_group=self._coldstart_cbgroup)
        self.create_service(
            GetTiltReadingService, 'get_platform_tilt',
            self._svc_get_platform_tilt,
            callback_group=self._coldstart_cbgroup)
        self.create_subscription(
            Float64MultiArray, 'set_level_state', self._sub_set_level_state, 10,
            callback_group=self._coldstart_cbgroup)

        # ── Hand command surface ──────────────────────────────────
        # The can-bridge owns CAN3, which carries the hand ODrive (axis 6), so it
        # restores the hand conduit can_node held (silently no-op'd against the
        # bridge until now — catch_coordinator's hand services were GAPs). Same ROS
        # names + srv types can_node registered (can_node.py:198-201), so
        # catch_coordinator_node reaches the bridge UNCHANGED: set_hand_state/gains
        # ride the relay-seam axis-6 allow-table; set_hand_traj_cmd/smooth_move_hand
        # ride the HAND_TRAJ_CMD RPC (byte-identical 0x6D0 → Platform Teensy).
        self.create_service(SetString, 'set_hand_state', self._svc_set_hand_state)
        self.create_service(SetHandTrajCmd, 'set_hand_traj_cmd', self._svc_set_hand_traj)
        self.create_service(SetFloat, 'smooth_move_hand', self._svc_smooth_move_hand)
        self.create_service(SetHandGains, 'set_hand_gains', self._svc_set_hand_gains)

        # ── Ball Butler services (production names, cutover) ────
        # The bb/* services formerly served by can_node. The firmware
        # gates each TX on bb_present(); we translate the ERR_BUS_DOWN that
        # gate produces into a silent-success for bb/calibrate to preserve
        # can_node._svc_bb_calibrate's HOMING semantics (allows homing to
        # complete without BB attached). The other two propagate the
        # error so an operator sees a failed reload/reset.
        self.create_service(Trigger, 'bb/reload',    self._svc_bb_reload)
        self.create_service(Trigger, 'bb/reset',     self._svc_bb_reset)
        self.create_service(Trigger, 'bb/calibrate', self._svc_bb_calibrate)

        # ── Ball Butler throw ACTION (replaces bb/send_throw_command) ──
        # The action awaits the firmware's terminal outcome (relayed back as a
        # CMD_RESULT CAN frame) instead of the bridge-side "frame queued" ack the
        # service returned. Its callbacks share a ReentrantCallbackGroup so a
        # result-wait in execute_callback never stalls the 100 Hz telemetry timers
        # (which run in the default group on other MultiThreadedExecutor threads),
        # and a second goal's goal_callback can run (and be rejected) while a throw
        # is still outstanding.
        self._bb_throw_cbgroup = ReentrantCallbackGroup()
        self._bb_throw_action = ActionServer(
            self, BallButlerThrowCmd, 'bb/throw',
            execute_callback=self._bb_throw_execute,
            goal_callback=self._bb_throw_goal,
            cancel_callback=self._bb_throw_cancel,
            callback_group=self._bb_throw_cbgroup)

        # ── Timers (publish on the executor thread) ────────────
        self.create_timer(0.01, self._publish_robot_state)     # 100 Hz (telem rate)
        self.create_timer(0.01, self._publish_hand_telemetry)  # 100 Hz
        self.create_timer(0.1, self._publish_link_status)      # 10 Hz (heartbeat rate)
        self.create_timer(0.1, self._publish_bb_heartbeat)     # 10 Hz BB (matches CAN1 rate)
        self.create_timer(0.01, self._drain_cone_catch_events) # 100 Hz cone events (cheap when idle)
        self.create_timer(0.1, self._publish_cone_heartbeat)   # 10 Hz cone (matches CAN2 rate)
        self.create_timer(0.01, self._publish_bb_axis_estimates)  # 100 Hz drain (BB pitch/hand est.)
        self.create_timer(0.01, self._publish_leg_cmd_executed)   # 100 Hz drain (Teensy post-clamp leg cmd)
        # 1 Hz is ample for a ~30 s (worst case 500 ms) anchor cadence, and keeps
        # a queue that is empty 99 % of the time off the 100 Hz timer group.
        self.create_timer(1.0, self._publish_clock_diag)          # 1 Hz drain (per-anchor clock diag)
        # Matches the frame's own 1 Hz emit cadence: at most one sample is ever
        # waiting, so the queue exists for thread hand-off, not for buffering.
        self.create_timer(1.0, self._publish_cache_diag)          # 1 Hz drain (encoder-cache census)
        # Matches the frame's own 1 Hz emit cadence, as with /cache_diag above.
        self.create_timer(1.0, self._publish_ring_diag)           # 1 Hz drain (RX-ring occupancy census)
        self.create_timer(0.5, self._publish_bb_odrive)        # 2 Hz BB ODrive diag (CAN1 ~1 Hz src)
        self.create_timer(1.0, self._publish_profile)          # 1 Hz (profile rate)
        self.create_timer(1.0, self._health_check)             # 1 Hz link watchdog
        self.create_timer(1.0, self._version_check_poll)       # 1 Hz firmware-version handshake (no-op once resolved)

        # ── Setpoint downlink (Commit 3) — STARTS DISARMED ────
        # The 40/500 Hz hot path. The SetpointPump (pure packing + per-step
        # safety gate) is always constructed (cheap), but the ZMQ source and the
        # ingest thread are created ONLY by _arm_setpoint_output's
        # stream-then-arm pre-check (ARMING_CONTRACT A1 — automatic on ACTIVE
        # entry via the orchestrator, or a manual /set_setpoint_output call).
        # While disarmed there is NO setpoint thread and the heartbeat keeps
        # mpc_active=0, so the Teensy will not enable leg output.
        #
        # LEG TORQUE FEEDFORWARD — independently gated (ships ENABLED since
        # the 2026-07-16 arming session)
        # (dynamics.torque_ff_enabled, shipped false). The pump is the SINGLE wire
        # enforcement point for it: clamp (TRUE Nm) → ramp → Kt wire scale. With the
        # flag off it packs torque_ff = zeros and never reads cmd['torque_Nm'], so
        # the frame is byte-identical to the pre-feature one. The ramp is expressed
        # in ACCEPTED FRAMES (deterministic, and a dropped frame lengthens it — the
        # safe direction), derived here from the ramp seconds and the fixed 40 Hz
        # knot spacing.
        _ff_ramp_frames = int(math.ceil(
            float(hw.DYNAMICS_TORQUE_FF_RAMP_S) / float(hw.JB_TRAJ_KNOT_DT_S)))
        self._sp_pump = SetpointPump(
            mm_to_rev=hw.GEOM_MM_TO_REV,
            num_legs=p.NUM_LEGS, max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV,
            torque_ff_enabled=bool(hw.DYNAMICS_TORQUE_FF_ENABLED),
            torque_ff_max_nm=float(hw.DYNAMICS_TORQUE_FF_MAX_NM),
            torque_wire_scale=float(hw.ODRIVE_LEG_TORQUE_WIRE_SCALE),
            torque_ff_ramp_frames=_ff_ramp_frames)
        if hw.DYNAMICS_TORQUE_FF_ENABLED:
            self.get_logger().warn(
                'LEG TORQUE FEEDFORWARD IS ENABLED '
                f'(clamp ±{hw.DYNAMICS_TORQUE_FF_MAX_NM} Nm true, wire scale '
                f'{hw.ODRIVE_LEG_TORQUE_WIRE_SCALE:.6f}, ramp {_ff_ramp_frames} frames '
                f'≈ {hw.DYNAMICS_TORQUE_FF_RAMP_S} s). '
                'Set dynamics.torque_ff_enabled=false + regenerate + colcon build to disable.')
        # ── leg_setpoint_echo stash (GUI observability) ────────
        # Written by _process_setpoint on the SETPOINT THREAD (the production
        # leg hot path) — the write is the absolute minimum under self._lock:
        # one tuple ref + one float + one int. NO message construction and NO
        # publishing happen on that thread; _publish_leg_setpoint_echo (100 Hz
        # robot_state timer, executor thread) does both, gated on freshness
        # (_SETPOINT_ECHO_STALE_S) and deduped by seq so each accepted 40 Hz
        # frame is echoed at most once.
        self._last_accepted_u0 = None        # tuple[float x6], motor revs
        self._last_accepted_u0_mono = 0.0    # time.monotonic() at accept
        self._last_accepted_u0_seq = 0       # bumps once per ACCEPTED frame
        # Timer-thread-only (the timer's MutuallyExclusiveCallbackGroup
        # serializes it — same argument as _pose_quat_key): no lock needed.
        self._echo_last_pub_seq = 0
        self._sp_source = None
        self._sp_thread = None
        self._sp_stop = threading.Event()
        # True while _run_deactivate is mid-descent (any entry point). The arm
        # pre-check refuses while set: an arm landing mid-descent would hand the
        # descending legs to the setpoint stream partway down (ARMING_CONTRACT
        # A1, review 2026-07-15). Written/cleared on the deactivate's executor
        # thread, read on the arm service's thread — plain bool, GIL-atomic.
        self._deactivate_in_progress = False
        # Serializes _arm_setpoint_output (audit 2026-07-15): the arm service
        # lives in the Reentrant group, so without this two overlapping arms
        # could both pass the mpc_active check and double-start the ingest.
        self._arm_lock = threading.Lock()
        # ARMING_CONTRACT A1 — the boot-arm path is REMOVED. Arming at __init__
        # ran ZERO preconditions: nothing can legitimately stream at BOOT (the
        # workspace gate refuses to seed at STOW), so `enable_setpoint_output:=true`
        # armed onto a silent :5557 and the firmware MPC-staleness watchdog
        # latched within one guard tick — every leg command then refused
        # (reconfirmed live 2026-07-15 21:32). The ONLY arming path is the
        # runtime service below, whose stream-then-arm pre-check is the
        # contract's single safe-to-arm gate. The parameter is retained but
        # inert so stale launch invocations fail loud, not weird.
        if bool(self.get_parameter('enable_setpoint_output').value):
            self.get_logger().error(
                "enable_setpoint_output:=true is INERT since the arming contract "
                "(2026-07-15): boot-arming had zero preconditions and self-E-STOPd "
                "(MPC_STALE) before any producer could stream. The bridge starts "
                "DISARMED; arming is runtime-only via /set_setpoint_output — "
                "automatic on ACTIVE entry (orchestrator auto-arm) or manual. "
                "Remove the launch arg.")
        # Injected source retained so runtime arming (set_setpoint_output) can reuse
        # it in tests instead of opening a real ZMQ SUB on :5557.
        self._injected_setpoint_source = setpoint_source

        # ── Runtime arming service (fixes the arm-before-stream trap) ──
        # The documented PRODUCTION arming flow (the launch parameter stays for
        # bench use). On true: require (a) Teensy link up + fresh heartbeat, (b) a
        # fresh mpccmd frame on :5557 within 0.5 s, (c) that frame's u0 within
        # _ARM_U0_TOL_REV (0.25 rev, a quarter of the firmware 1.0 rev MAX_DEVIATION backstop) of
        # every leg's live pos_estimate — THEN stream-then-arm (start the thread, set
        # mpc_active=1). This is the pattern validated in
        # tests/hardware/teensy_guard_validation.py: never arm before a matching
        # stream is confirmed flowing, so the Teensy never E-STOPs at the arm edge.
        # The recovery/arming callback group is created HERE (before the arm
        # service) so both can share it — see the /recover comment below for the
        # full rationale.
        self._recover_cbgroup = ReentrantCallbackGroup()
        # The arm handler blocks up to 0.5 s per attempt on the :5557 frame-wait;
        # in the node-default MutuallyExclusiveCallbackGroup that block would
        # SERIALIZE with (and starve) the 100 Hz robot_state / 10 Hz link_status
        # timers on every refused auto-arm retry (review 2026-07-15) — freezing
        # exactly the telemetry the producer needs to seed. Reentrant group keeps
        # the timers running through the wait.
        self.create_service(
            SetBool, 'set_setpoint_output', self._svc_set_setpoint_output,
            callback_group=self._recover_cbgroup)

        # ── One-call guard recovery (/recover, FIX 3) ──
        # The 2026-07-10 runaway left the streamed u0 diverged ~0.5+ rev from the
        # frozen encoder, so every bare /clear_errors re-latched MAX_DEVIATION within
        # one 10 Hz fault tick. /recover does the recovery in the ONLY order that
        # survives: reseed trajectory_node's hold at the measured encoder → VERIFY the
        # streamed u0 has collapsed to within _RECOVER_U0_TOL_REV (0.03 rev) of every
        # encoder → THEN CLEAR_ERRORS. Hosted HERE (not trajectory_node) because the bridge runs a
        # MultiThreadedExecutor and already blocks executor threads for multi-second
        # cold-start verbs, so the bounded block on the reseed future is safe;
        # trajectory_node runs a single-threaded rclpy.spin() executor where blocking
        # a callback on a cross-process future would deadlock its sole thread. The
        # service + the cross-process reseed client share a ReentrantCallbackGroup so
        # the reseed reply is dispatched on another executor thread while /recover
        # blocks on the future. (The group itself is created above, next to the
        # arm service that shares it.)
        self._reseed_client = self.create_client(
            Trigger, 'trajectory/reseed_from_measured',
            callback_group=self._recover_cbgroup)
        self.create_service(Trigger, 'recover', self._svc_recover,
                            callback_group=self._recover_cbgroup)
        # clear_errors shares the /recover ReentrantCallbackGroup (F1, 2026-07-11).
        # WHY not the node-default MutuallyExclusiveCallbackGroup: the armed bare
        # /clear_errors reroutes INLINE through _svc_recover, which blocks up to
        # _RECOVER_MAX_RESEED_ATTEMPTS × (reseed_timeout + verify_timeout) ≈ 18 s. In the
        # default group that block would SERIALIZE with — and starve — the 100 Hz
        # telemetry timers AND the set_setpoint_output disarm service, making the disarm
        # escape hatch unreachable for the whole block. In the reentrant group the reseed
        # reply dispatches on another executor thread and the disarm/telemetry callbacks
        # keep running, so the operator can always disarm out.
        self.create_service(Trigger, 'clear_errors', self._svc_clear_errors,
                            callback_group=self._recover_cbgroup)
        # Instance-level so tests can shorten the descent-convergence wait (a
        # never-converging descent otherwise blocks the test for the full timeout).
        self._recover_verify_timeout_s = _RECOVER_VERIFY_TIMEOUT_S

        # ── Cold-start boot read ─────────────────────
        # Read the Platform Teensy's persisted RobotState BEFORE construction
        # returns — i.e. before the executor spins and the first robot_state is
        # published — so the orchestrator never acts on a wrong is_homed. Bounded
        # (a few retries × _RELAY_READ_TIMEOUT_S); on total failure the cache stays
        # at the CONSERVATIVE default (is_homed=False → forces a re-home, SAFE).
        # Disabled in unit tests (boot_state_read=False) that don't wire a Platform
        # responder — they exercise _refresh_cold_start_state directly instead.
        if boot_state_read:
            self._boot_read_cold_start_state()

        peer = (self.get_parameter('teensy_ip').value
                if client is None else 'injected')
        self.get_logger().info(
            f"TeensyBridgeNode up — peer={peer} stream={p.PORT_STREAM} "
            f"rpc={p.PORT_RPC}, DISARMED (mpc_active=0; arming is runtime-only "
            f"via /set_setpoint_output — see ARMING_CONTRACT.md)")
        # Boot-time config identity, logged ONCE. Its own call site at a FIXED
        # severity: Foxy's rcutils logger caches severity per source line and
        # raises on a flip (971d12c). Pairs with jugglebot_launch.py's drift
        # banner — the launch says whether the INSTALLED copy is stale, this
        # says which file this process actually got.
        self.get_logger().info("config identity — " + hardware_config_identity())

    # ═══════════════════════════════════════════════════════════
    # RX-thread frame callbacks — keep these short (stash + return)
    # ═══════════════════════════════════════════════════════════

    def _on_heartbeat_t2j(self, msg_type, seq, payload, addr):
        try:
            hb = HeartbeatT2J.unpack(payload)
        except Exception:  # noqa: BLE001 — never let a bad frame kill the RX thread
            return
        with self._lock:
            self._latest_heartbeat = hb
            self._heartbeat_gen += 1

    def _on_telemetry(self, msg_type, seq, payload, addr):
        try:
            tm = Telemetry.unpack(payload)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._latest_telemetry = tm
            # Generation, not a timestamp: _publish_robot_state needs to know
            # whether the latch it is about to serialise is the SAME one it
            # already published, and a monotonic counter answers that exactly,
            # with no clock read and no threshold to tune. See the honesty gate
            # there. Unbounded in Python, so no wrap case exists.
            self._telemetry_gen += 1

    def _on_diagnostic(self, msg_type, seq, payload, addr):
        try:
            dg = Diagnostic.unpack(payload)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._latest_diag[int(dg.axis_id)] = dg
            self._latest_diag_mono[int(dg.axis_id)] = time.monotonic()

    def _on_profile(self, msg_type, seq, payload, addr):
        try:
            pr = Profile.unpack(payload)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._latest_profile = pr

    def _on_cone_frame(self, msg_type, seq, payload, addr):
        # Decode the relayed CAN payload here (cheap struct unpack) so the
        # drain timer publishes ready-made events; anything malformed or
        # unrecognised (future cone frame types) is dropped silently, matching
        # the other RX callbacks' never-kill-the-RX-thread contract.
        try:
            cf = ConeFrame.unpack(payload)
            data = bytes(cf.data[:cf.dlc])
            if cf.can_id == catching_cone.CATCH_EVENT_ID:
                evt = catching_cone.CatchEvent.from_can_frame(data)
                arrival_us = int(time.time() * 1e6)
                with self._lock:
                    q = self._cone_catch_queue
                    q.append((evt, arrival_us))
                    # Concurrency hardening: bound the queue if the 100 Hz drain stalls, same
                    # as the BB-estimates sibling (drop-oldest). Catches are discrete
                    # + rare, so 4000 (~unbounded in practice) only guards a stuck drain.
                    if len(q) > 4000:
                        del q[:len(q) - 4000]
            elif cf.can_id == catching_cone.HEARTBEAT_ID:
                hb = catching_cone.ConeHeartbeat.from_can_frame(data)
                with self._lock:
                    self._latest_cone_hb = hb
                    self._cone_hb_received = True
                    self._cone_last_hb_mono = time.monotonic()
        except Exception:  # noqa: BLE001
            return

    def _on_platform_frame(self, msg_type, seq, payload, addr):
        # RX-thread callback: a Platform-Teensy relay reply (CAN3 0x6E0 RobotState
        # / 0x7DE tilt) the bridge forwarded verbatim. Latch the latest reply by
        # can_id and wake any relay read blocked on it. Malformed frames dropped
        # (never kill the RX thread). Decode lives in the relay read methods so the
        # bridge stays decoupled from the Platform-Teensy byte layout.
        try:
            pf = PlatformFrame.unpack(payload)
        except Exception:  # noqa: BLE001
            return
        data = bytes(pf.data[:pf.dlc])
        with self._platform_reply_cv:
            self._platform_replies[int(pf.can_id)] = (data, int(pf.dlc))
            self._platform_reply_cv.notify_all()

    def _on_hand_cmd_echo(self, msg_type, seq, payload, addr):
        # RX-thread callback (hand conduit): the bridge sniffed the Platform Teensy's
        # Set_Input_Pos to the hand ODrive (axis 6) on CAN3. Decode the commanded
        # pos/vel_ff/tor_ff and stash them so _publish_hand_telemetry can echo them
        # (byte-identical to can_node._handle_hand_input_pos: <f h h> then vel/tor
        # ÷ INPUT_SCALE_HAND_VEL/_TOR). Malformed frames dropped (never kill the RX
        # thread). Stash under the shared lock; the publish timer reads it.
        try:
            ce = HandCmdEcho.unpack(payload)
            pos, vel, tor = rpc_args.decode_hand_cmd_echo(
                bytes(ce.data),
                proto.INPUT_SCALE_HAND_VEL, proto.INPUT_SCALE_HAND_TOR)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._last_hand_cmd = {'pos': pos, 'vel': vel, 'tor': tor}

    def _on_hand_sensor(self, msg_type, seq, payload, addr):
        # RX-thread callback (hand ball-present sensor): stash the decoded frame
        # AND the host monotonic arrival time — the arrival time is the only
        # thing that can tell a live bridge from a dead one (the frame's own
        # flags describe the bridge's SDO cache and say nothing about the link).
        # Malformed frames dropped (never kill the RX thread).
        try:
            hs = HandSensor.unpack(payload)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._latest_hand_sensor = hs
            self._latest_hand_sensor_mono = time.monotonic()

    def _on_can_errors(self, msg_type, seq, payload, addr):
        # RX-thread callback (CAN3 wire-error counters). Pure diagnostics: cached
        # under the lock and rendered into one /link_status row. No RX stamp is
        # kept — unlike HAND_SENSOR these counters are cumulative-since-boot, so a
        # stale copy is still a truthful floor rather than a misleading verdict,
        # and the operator differences them across an A/B anyway.
        # Malformed frames dropped (never kill the RX thread).
        try:
            ce = CanErrors.unpack(payload)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._latest_can_errors = ce

    def _on_bridge_tx_diag(self, msg_type, seq, payload, addr):
        # RX-thread callback (CAN TX-path pressure + hand-stage attribution).
        # Same contract as _on_can_errors: pure diagnostics, cached under the
        # lock, no arrival stamp (cumulative counters make a stale copy a
        # truthful floor). Malformed frames dropped (never kill the RX thread) —
        # and the except is broad ON PURPOSE: a short payload from a bridge
        # built against an older header raises struct.error, which is NOT a
        # ValueError, so a narrow clause would let it escape to the client's
        # per-callback handler and log a traceback at 1 Hz forever.
        try:
            d = BridgeTxDiag.unpack(payload)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._latest_bridge_tx_diag = d

    def _on_bridge_identity(self, msg_type, seq, payload, addr):
        # RX-thread callback (can-bridge firmware identity). Caches the frame and
        # runs the FW-version verdict; _record_bridge_fw_version is announce-on-
        # change, so this arriving at 1 Hz forever costs one log line per change.
        # Malformed frames dropped — see _on_bridge_tx_diag for why the except is
        # broad.
        try:
            bi = BridgeIdentity.unpack(payload)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._latest_bridge_identity = bi
        self._record_bridge_fw_version(int(bi.fw_version))

    # ── Can-bridge firmware-identity check ──
    # THE single capture + verdict point for the can-bridge's FW_VERSION, and it
    # sits inside the RX callback for the same reason _record_platform_fw_version
    # sits inside relay_read_robot_state: that is the only place holding the raw
    # frame, so a future second consumer of BRIDGE_IDENTITY cannot bypass the
    # check by reading the cache instead.
    #
    # WARN, NEVER REFUSE — identical policy to the Platform Teensy check
    # (ros_ws/docs/platform_fw_version.md). Refusing on skew would put a version
    # comparison in front of the hand/leg command path, i.e. it would convert a
    # reporting defect into an outage; and the skew that matters here is a
    # DIAGNOSTIC one (an FW 8 bridge simply sends no counters), so there is
    # nothing unsafe to protect against.
    #
    # DEVIATION, recorded deliberately: this is the ONLY rclpy log call made
    # from the RX thread in this node — every other _on_* callback is log-free,
    # decodes, stashes under the lock and returns, so a malformed-frame storm
    # can never turn into a logging storm on the frame-receive path. The
    # carve-out is justified by the announce-on-change guard below, which bounds
    # this to at most one line per VERSION CHANGE (realistically one per launch)
    # rather than one per frame. If a future field here ever logs per-frame,
    # that reasoning is void and the log belongs on the 10 Hz publish timer
    # instead.

    def _record_bridge_fw_version(self, version: int):
        """Log the can-bridge FW_VERSION verdict. Called on the RX thread at 1 Hz.

        Announce-on-change in BOTH directions: at 1 Hz an unconditional log would
        be 3600 lines an hour, and an operator who sees the same line every second
        stops reading it — which is how a detector dies.
        """
        previous = self._bridge_fw_version_seen
        if previous == version:
            return
        self._bridge_fw_version_seen = version
        expected = rpc_args.EXPECTED_BRIDGE_FW_VERSION
        if version == expected:
            self.get_logger().info(
                f'BRIDGE_FW_CHECK: OK — can-bridge Teensy reports v{version} '
                f'(expected v{expected}){self._install_skew_note()}')
            return
        # Skew. ERROR level with one greppable token, and the older/newer
        # direction named because they fail differently: an older board is
        # missing counters this session's conclusions may rest on, a newer one
        # means the host checkout is behind the flashed board.
        direction = ('OLDER than' if version < expected else 'NEWER than')
        self.get_logger().error(
            f'BRIDGE_FW_CHECK: FAIL — can-bridge Teensy reports v{version}, '
            f'{direction} the v{expected} this host tree expects. Commands are '
            f'NOT refused (the skew is reported, never enforced — the same '
            f'warn-never-refuse policy as ros_ws/docs/platform_fw_version.md), '
            f'but any bench conclusion that reads the bridge_tx_diag counters is '
            f'untrustworthy until the can-bridge is re-flashed.'
            f'{self._install_skew_note()}')

    def _install_skew_note(self) -> str:
        """The host half of the currency verdict, appended to BRIDGE_FW_CHECK.

        The two halves are reported in ONE line because they are one question —
        "is what is running what the repo says?" — and the 2026-08-14 S3 miss is
        precisely what happens when an operator reads the firmware half as an
        answer to both. Rendered on the OK line too: a green firmware verdict
        beside a silent host verdict is the shape of the original trap.
        """
        if self._install_skew == INSTALL_SKEW_STALE:
            # Phrased to read correctly after BOTH call sites' punctuation —
            # '(expected v13)' and 'until the can-bridge is re-flashed.'
            return (' BUT NOTE — install_skew=1: this NODE is running a stale '
                    'colcon install, so nothing it reports about the bridge is '
                    'trustworthy. ' + self._install_skew_detail)
        if self._install_skew == INSTALL_SKEW_UNKNOWN:
            return (' [install_skew=unknown — the node build could not be '
                    'checked against a repo source tree]')
        return ' [install_skew=0 — node build matches the repo source]'

    def _on_bb_estimates(self, msg_type, seq, payload, addr):
        # RX-thread callback: decode + queue every sample. The drain timer
        # publishes on the executor thread (never publish from the RX thread,
        # matching the other RX callbacks' contract). Malformed frames dropped.
        try:
            e = BbAxisEstimates.unpack(payload)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            q = self._bb_est_queue
            q.append(e)
            if len(q) > 4000:        # ~40 s at 100 Hz — bound if the drain stalls
                del q[:len(q) - 4000]
            # Latest-wins stash for the robot_state BB append (the queue above is
            # drained/emptied by the bb/axis_estimates publisher, so it can't
            # serve as "latest").
            self._latest_bb_est = e
            self._latest_bb_est_mono = time.monotonic()

    def _on_leg_cmd(self, msg_type, seq, payload, addr):
        # RX-thread callback: decode + queue every sample. Same contract as
        # _on_bb_estimates — the drain timer publishes on the executor thread,
        # NEVER this one (a rclpy publish here would put DDS work on the socket
        # RX path that also carries the 100 Hz telemetry and the RPC replies).
        # Malformed frames dropped.
        try:
            c = LegCmd.unpack(payload)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            q = self._leg_cmd_queue
            q.append(c)
            if len(q) > 4000:        # ~40 s at 100 Hz — bound if the drain stalls
                del q[:len(q) - 4000]

    def _on_clock_diag(self, msg_type, seq, payload, addr):
        # RX-thread callback: decode + queue every per-anchor clock sample. Same
        # contract as _on_leg_cmd — the drain timer publishes on the executor
        # thread, NEVER this one. Malformed frames dropped; the except is broad
        # for the reason spelled out on _on_bridge_tx_diag (a short payload raises
        # struct.error, which is not a ValueError).
        #
        # EVERY sample is queued, never latest-wins: anchors are ~30 s apart and
        # each one is an independent measurement, so dropping one to keep the
        # newest would delete a data point from a fit that has only ~120 of them
        # per hour.
        try:
            cd = ClockDiag.unpack(payload)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            q = self._clock_diag_queue
            q.append(cd)
            # ~17 h of anchors at the 30 s cadence, or ~17 min of the 500 ms fast
            # retry. Bounds a stalled drain without ever plausibly truncating a
            # real session (drop-oldest, matching the other queues).
            if len(q) > 2000:
                del q[:len(q) - 2000]

    def _on_cache_diag(self, msg_type, seq, payload, addr):
        # RX-thread callback: decode + queue every encoder-cache census window.
        # Identical contract to _on_clock_diag — queue here, publish on the
        # executor thread in the drain timer, never on this socket thread, which
        # also carries the 100 Hz telemetry and every RPC reply. Malformed frames
        # dropped, with the same broad except: a short payload raises
        # struct.error, which is NOT a ValueError, and an exception escaping an
        # RX callback poisons the shared thread.
        #
        # EVERY window is queued, never latest-wins. Each frame is a reduction
        # over ~100 samples that no longer exist anywhere else — dropping one to
        # keep the newest deletes a second of evidence from a soak whose entire
        # conclusion is a trend line.
        try:
            cd = CacheDiag.unpack(payload)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            q = self._cache_diag_queue
            q.append(cd)
            # ~1 h at the 1 Hz emit cadence. Bounds a stalled drain (the drain is
            # also 1 Hz, so in health this queue holds 0 or 1 entries) without
            # truncating anything a real session would produce. Drop-oldest,
            # matching the other queues.
            if len(q) > 3600:
                del q[:len(q) - 3600]

    def _on_ring_diag(self, msg_type, seq, payload, addr):
        # RX-thread callback: decode + queue every RX-ring census window.
        # Identical contract to _on_cache_diag / _on_clock_diag — queue here,
        # publish on the executor thread in the drain timer, never on this socket
        # thread, which also carries the 100 Hz telemetry and every RPC reply.
        # Malformed frames dropped, with the same broad except: a short payload
        # raises struct.error, which is NOT a ValueError, and an exception
        # escaping an RX callback poisons the shared thread.
        #
        # EVERY window is queued, never latest-wins. Each frame reduces ~1000
        # service-tick probes that exist nowhere else, and the verdict is a trend
        # across hours — dropping one to keep the newest deletes a second of the
        # only evidence there is.
        try:
            rd = RingDiag.unpack(payload)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            q = self._ring_diag_queue
            q.append(rd)
            # ~1 h at the 1 Hz emit cadence. Bounds a stalled drain (the drain is
            # also 1 Hz, so in health this queue holds 0 or 1 entries) without
            # truncating anything a real session would produce. Drop-oldest,
            # matching the other queues.
            if len(q) > 3600:
                del q[:len(q) - 3600]

    def _on_cmd_result(self, msg_type, seq, payload, addr):
        # RX-thread callback (loud command-outcome channel). Decode the relayed CMD_RESULT
        # CAN frame; if it's a THROW outcome and a throw goal is outstanding, hand
        # the (outcome, detail0, detail1) to the waiting bb/throw execute_callback
        # and wake it. Never publish / never block here (RX-thread contract); any
        # malformed or unsolicited frame is dropped silently.
        try:
            cf = CmdResultFrame.unpack(payload)
            data = bytes(cf.data[:cf.dlc])
            if len(data) < 6:
                return
            cmd_type = data[0]
            outcome = data[1]
            detail0 = int.from_bytes(data[2:4], 'little', signed=True)
            detail1 = int.from_bytes(data[4:6], 'little', signed=True)
        except Exception:  # noqa: BLE001
            return
        if cmd_type != int(proto.BallButlerCommandType.THROW):
            return  # only the throw consumes CMD_RESULT today
        with self._bb_throw_lock:
            if not self._bb_throw_active:
                return  # no waiter — stale/duplicate frame, drop
            self._bb_throw_result = (outcome, detail0, detail1)
        self._bb_throw_event.set()

    def _publish_bb_axis_estimates(self):
        """Drain queued BB pitch/hand estimates → /bb/axis_estimates (JointState).

        Each sample is stamped with the bridge wall-clock at sample time
        (e.t_bridge_us, time-synced to the Jetson), so per-sample timing is
        correct regardless of when this drain runs. position=rev, velocity=rev/s;
        name=[bb_pitch, bb_hand]. Pitch deg = 90 + 360*pos_rev (PitchAxis.h);
        ball speed = vel_rps * 2*pi*HAND_SPOOL_RADIUS_M.
        """
        with self._lock:
            batch = self._bb_est_queue
            self._bb_est_queue = []
        for e in batch:
            js = JointState()
            t_us = int(e.t_bridge_us)
            js.header.stamp.sec = t_us // 1_000_000
            js.header.stamp.nanosec = (t_us % 1_000_000) * 1000
            js.name = ['bb_pitch', 'bb_hand']
            js.position = [float(e.pitch_pos_rev), float(e.hand_pos_rev)]
            js.velocity = [float(e.pitch_vel_rps), float(e.hand_vel_rps)]
            self.bb_estimates_pub.publish(js)

    def _publish_leg_cmd_executed(self):
        """Drain queued LEG_CMD samples → /leg_cmd_executed (JointState).

        WHAT this is: the can-bridge Teensy's own report of what its 500 Hz
        interp ladder commanded to the leg ODrives — axes[i].target_pos_rev /
        target_vel_rps AFTER the lead and stroke clamps
        (telemetry.cpp::send_leg_cmd, sampled at the 100 Hz telemetry rate).
        It is NOT /leg_setpoint_echo, which is the ACCEPTED u0 knot at the
        Jetson end of the same wire.

        WHY it exists: it is the missing middle timeline of the
        commanded → executed → measured chain, so a bag can attribute the
        uptime-growing command lag of
        logbook/2026-07-18-teensy-uptime-tracking-degradation.md to
        Jetson→Teensy transport, the interp ladder, or the ODrives, instead of
        only observing the total. It is also the input the alarmed end-to-end
        latency monitor that entry's closure Addendum requires will consume.

        Each sample is stamped with the bridge wall-clock at sample time
        (c.t_teensy_us, time-synced to the Jetson), so per-sample timing is
        correct regardless of when this drain runs — and, critically, the
        stamp is the TEENSY's clock, which is what makes a Jetson-vs-Teensy
        one-way delay measurable at all. position=motor rev, velocity=rev/s,
        Jugglebot convention (positive = extension); name=[leg0..leg5].
        """
        with self._lock:
            batch = self._leg_cmd_queue
            self._leg_cmd_queue = []
        for c in batch:
            js = JointState()
            t_us = int(c.t_teensy_us)
            js.header.stamp.sec = t_us // 1_000_000
            js.header.stamp.nanosec = (t_us % 1_000_000) * 1000
            js.name = [f'leg{i}' for i in range(_NUM_LEGS)]
            js.position = [float(v) for v in c.cmd_pos_rev]
            js.velocity = [float(v) for v in c.cmd_vel_rps]
            self.leg_cmd_executed_pub.publish(js)

    def _publish_clock_diag(self):
        """Drain queued CLOCK_DIAG samples → /clock_diag (DiagnosticStatus).

        One message per accepted time-of-day anchor, each carrying the bridge's
        wall-clock discipline state at that anchor plus the 500 Hz interp
        ladder's fallback occupancy over the window since the previous one.

        WHAT MAKES IT USABLE: every sample is self-contained. ``t_local_us`` is
        the bridge's raw ``micros64()`` — the pure crystal, the only honest
        x-axis for a clock fit, since the wall clock is the measurand and it
        STEPS. ``jetson_wall_us`` is the anchor actually applied, so a consumer
        derives the measured offset itself (``jetson_wall_us - t_local_us``)
        rather than trusting a firmware subtraction, and can re-derive
        ``freq_ppb`` from consecutive samples. ``anchor_seq`` is what makes a
        LOST frame visible: this is one-shot-per-anchor on a lossy transport, so
        without it a dropped sample would silently widen an interval and corrupt
        a rate fit while looking like clean data.

        ``flags`` is rendered BEFORE the values it qualifies, and ``freq_ppb`` is
        rendered as ``n/a`` when FREQ_VALID is clear. That is deliberate: the
        wire carries 0 in that case, and a bare ``0`` reads as "no frequency
        error" — the exact opposite of "no measurement". Closing that misread at
        the render site is cheaper than trusting every future consumer to check
        a bit.

        An FW 10 board never sends 0x8F, so on the pre-flash bridge this drains
        nothing and the topic records EMPTY — the visible absence the record
        list's add-never-trim rule is built around, not an error.
        """
        with self._lock:
            batch = self._clock_diag_queue
            self._clock_diag_queue = []
        for cd in batch:
            flags = int(cd.flags)
            stepped = bool(flags & int(p.ClockDiagFlags.STEPPED))
            first = bool(flags & int(p.ClockDiagFlags.FIRST_ANCHOR))
            freq_valid = bool(flags & int(p.ClockDiagFlags.FREQ_VALID))
            msg = DiagnosticStatus()
            msg.name = 'teensy/clock_diag'
            msg.hardware_id = 'can_bridge_teensy'
            # A STEP outside the first anchor is the one state worth flagging: it
            # means the measured offset moved further in one interval than any
            # crystal can explain, i.e. a link gap or a host clock jump. The first
            # anchor of a session steps by construction and is not a problem.
            msg.level = (DiagnosticStatus.WARN
                         if (stepped and not first) else DiagnosticStatus.OK)
            msg.message = (
                f'seq={int(cd.anchor_seq)} rtt={int(cd.rtt_us)}us '
                f'err={int(cd.err_us)}us '
                f'freq={int(cd.freq_ppb) if freq_valid else "n/a"}ppb')
            msg.values = [
                # Read these first — they qualify everything below.
                KeyValue(key='stepped', value=str(int(stepped))),
                KeyValue(key='first_anchor', value=str(int(first))),
                KeyValue(key='freq_valid', value=str(int(freq_valid))),
                KeyValue(key='anchor_seq', value=str(int(cd.anchor_seq))),
                KeyValue(key='t_local_us', value=str(int(cd.t_local_us))),
                KeyValue(key='jetson_wall_us', value=str(int(cd.jetson_wall_us))),
                KeyValue(key='dt_local_us', value=str(int(cd.dt_local_us))),
                KeyValue(key='rtt_us', value=str(int(cd.rtt_us))),
                KeyValue(key='err_us', value=str(int(cd.err_us))),
                KeyValue(key='freq_ppb',
                         value=str(int(cd.freq_ppb)) if freq_valid else 'n/a'),
                KeyValue(key='interp_ticks', value=str(int(cd.interp_ticks))),
                KeyValue(key='recover_slew_ticks',
                         value=str(int(cd.recover_slew_ticks))),
                KeyValue(key='extrap_ticks', value=str(int(cd.extrap_ticks))),
            ]
            self.clock_diag_pub.publish(msg)

    def _publish_cache_diag(self):
        """Drain queued CACHE_DIAG windows → /cache_diag (DiagnosticStatus).

        One message per 1 s window, each carrying the can-bridge's per-axis
        ENCODER-CACHE AGE floor and peak (reduced on-chip from a sample per
        100 Hz telemetry tick) plus the CAN RX-ring occupancy and decode-discard
        counters that have existed on the Teensy since 2026-06-04 / 2026-07-05
        and were never uplinked.

        WHAT IT DECIDES. S1 (2026-08-12) localized the uptime command-latency
        drift to the Teensy and exonerated everything measurable around it —
        RTT flat at 1-3 ms, zero interp deadline misses, flat heap, flat CAN
        throughput, ODrives and the Jetson-side link cleared. Two candidates
        survived: the encoder cache the ``leg_interp`` lead clamp measures
        ``fb`` against is going stale with uptime, or the leg genuinely trails.
        ``/robot_state`` and ``/leg_cmd_executed`` both read that same cache, so
        under a stale cache they move together and look exactly like real lag.
        ``age_min_us`` breaks the tie, because it is a FLOOR: jitter, sampling
        luck and a late tick can raise a maximum but none of them can raise a
        minimum. A floor that grows with uptime confirms the stale cache; a
        floor that stays at the broadcast period while the lag grows refutes it.

        ``enc_frames_*`` then splits the confirming case in two. The S1 bag
        forensics found the per-axis cache VALUE stalling for 30-500 ms in a fat
        tail (9-18 % of refresh intervals > 30 ms on an aged bridge against 4.3 %
        fresh) while every AGGREGATE CAN RX counter stayed flat — which is not a
        contradiction, because an aggregate cannot see one axis of seven go
        quiet. Differenced across a stall window, a counter that kept ADVANCING
        means the frames arrived and the decode ran, so the frozen value came in
        over the wire; a counter that PAUSED means nothing arrived for that axis.
        Different faults, different owners, and no other field separates them.

        ``seen_mask`` is rendered FIRST and the ages are qualified by it. An
        axis that has never been cached (the single-leg bench rig, a dark hand
        ODrive) reports the u32 saturation rail, which is honest but reads
        identically to a catastrophically stale cache — and it must not raise
        the level, or a bench session would WARN continuously and teach the
        operator to ignore the row.

        An FW <= 11 board never sends 0x91, so on the pre-flash bridge this
        drains nothing and the topic records EMPTY — the visible absence the
        record list's add-never-trim rule is built around, not an error.
        """
        with self._lock:
            batch = self._cache_diag_queue
            self._cache_diag_queue = []
        for cd in batch:
            seen = int(cd.seen_mask)
            ages_min = [int(v) for v in cd.age_min_us]
            ages_max = [int(v) for v in cd.age_max_us]
            # Only axes that have ever been cached can be judged. An unseen axis
            # is absent, not stale.
            seen_idx = [i for i in range(len(ages_min)) if seen & (1 << i)]
            worst_floor = max((ages_min[i] for i in seen_idx), default=0)
            # Feed the alarmed latency monitor (the LOUD half of the 2026-07-24
            # contract). Deliberately the seen_mask-qualified floor, not the raw
            # one: an axis that has never been cached rides the u32 saturation
            # rail, and alarming on a dark hand ODrive at a single-leg bench rig
            # is how an alarm teaches the operator to ignore it. Last window in
            # the batch wins — this is a live state, not a tally.
            self._lm_cache_floor_us = worst_floor
            self._lm_cache_t = time.monotonic()
            msg = DiagnosticStatus()
            msg.name = 'teensy/cache_diag'
            msg.hardware_id = 'can_bridge_teensy'
            msg.level = (DiagnosticStatus.WARN
                         if worst_floor >= CACHE_AGE_FLOOR_WARN_US
                         else DiagnosticStatus.OK)
            msg.message = (
                f'seq={int(cd.seq)} uptime={int(cd.t_local_us) // 1000}ms '
                f'worst_age_floor={worst_floor}us '
                f'samples={int(cd.samples)}/{int(cd.window_us)}us')
            values = [
                # Read this first — it qualifies every age below.
                KeyValue(key='seen_mask', value=str(seen)),
                KeyValue(key='seq', value=str(int(cd.seq))),
                KeyValue(key='t_local_us', value=str(int(cd.t_local_us))),
                KeyValue(key='window_us', value=str(int(cd.window_us))),
                KeyValue(key='samples', value=str(int(cd.samples))),
                # The single number an operator watches during a soak.
                KeyValue(key='worst_age_floor_us', value=str(worst_floor)),
            ]
            # Per-axis rows, 0-5 legs and 6 the hand. Rendered as 'n/a' for an
            # axis outside seen_mask: the wire carries the saturation rail
            # there, and a bare 4294967295 in a bag reads as a measured age.
            for i in range(len(ages_min)):
                ok = bool(seen & (1 << i))
                values.append(KeyValue(
                    key=f'age_min_us_{i}',
                    value=str(ages_min[i]) if ok else 'n/a'))
            for i in range(len(ages_max)):
                ok = bool(seen & (1 << i))
                values.append(KeyValue(
                    key=f'age_max_us_{i}',
                    value=str(ages_max[i]) if ok else 'n/a'))
            # Per-axis get_encoder_estimate frames decoded AND cached, CUMULATIVE
            # since the bridge booted. Rendered RAW and differenced by the
            # consumer, matching the can3_errors / bridge_tx_diag counters — the
            # node holds no previous sample, so a node-side delta would silently
            # widen across a dropped frame, which is the failure `seq` exists to
            # make visible rather than to paper over.
            #
            # The 'n/a' discipline above deliberately does NOT extend here: for an
            # age, the wire's saturation rail on an unseen axis is a MISSING
            # measurement; for a frame count, 0 is a measurement — that axis
            # received nothing — and it is the single most diagnostic value the
            # field can carry. Blanking it would erase the answer.
            for i in range(len(cd.enc_frames)):
                values.append(KeyValue(
                    key=f'enc_frames_{i}', value=str(int(cd.enc_frames[i]))))
            values.extend([
                KeyValue(key='rx_depth_hwm_jb',
                         value=str(int(cd.rx_depth_hwm_jb))),
                KeyValue(key='rx_depth_hwm_bb',
                         value=str(int(cd.rx_depth_hwm_bb))),
                KeyValue(key='rx_depth_hwm_cone',
                         value=str(int(cd.rx_depth_hwm_cone))),
                KeyValue(key='rx_cap_hits_jb',
                         value=str(int(cd.rx_cap_hits_jb))),
                KeyValue(key='rx_cap_hits_bb',
                         value=str(int(cd.rx_cap_hits_bb))),
                KeyValue(key='rx_cap_hits_cone',
                         value=str(int(cd.rx_cap_hits_cone))),
                KeyValue(key='decode_short', value=str(int(cd.decode_short))),
                KeyValue(key='decode_bad_axis',
                         value=str(int(cd.decode_bad_axis))),
            ])
            msg.values = values
            self.cache_diag_pub.publish(msg)

    def _publish_ring_diag(self):
        """Drain queued RING_DIAG windows → /ring_diag (DiagnosticStatus).

        One message per 1 s window, each carrying the can-bridge's CAN RX-ring
        TRUE occupancy beside the ``_available`` count the rest of the firmware
        believes, the hardware FIFO overflow/warning counts, and two independent
        jugglebot-bus cross-checks (delivery lag off the FlexCAN hardware capture
        timestamp, and the hand-sensor SDO round-trip floor).

        WHAT IT DECIDES. S2 (2026-08-13) killed the encoder-cache AGE hypothesis
        — the cache read FRESHER at 28 h than at 1.3 h — leaving the observed
        mechanism as ~100-150 ms bit-identical per-axis telemetry freezes that the
        freshness-blind lead clamp amplifies into commanded stops. The surviving
        candidate cause is a concurrency defect in the vendored FlexCAN_T4:
        ``events()`` pops the RX ring BEFORE its ``NVIC_DISABLE_IRQ`` guard, so
        the CAN ISR's ``_available++`` races the unguarded task-side
        ``_available--`` one-directionally (the ISR preempts the task, never the
        reverse). Increments get swallowed, ``_available`` monotonically
        under-counts, and the bridge's drain loop stops with a residue still in
        the ring — after which every frame it delivers is that many frames old,
        ratcheting with uptime and capping at the 256-slot ring depth
        (~114-135 ms at jugglebot-bus rates).

        WHY A NEW TOPIC RATHER THAN A FIELD ON /cache_diag. ``getRXQueueCount()``
        returns ``_available``, so ``rx_depth_hwm`` and ``rx_cap_hits`` — the
        ring fields /cache_diag already publishes — are computed from the very
        number the race corrupts. They are blind to this failure by construction
        and would read perfectly healthy through a fully-leaked ring.

        THE HEADLINE ROW is ``leak_jb`` = ``true_depth_jb - avail_reported_jb``,
        rendered first. It is sampled immediately after the drain loop, where
        ``_available`` is 0 by definition (that is why the loop exited), so the
        residual true depth IS the leak rather than an inference from it.
        ``leak_hwm_jb`` is the same quantity maximised over every 1 kHz service
        tick since boot, so the verdict does not rest on the 1 Hz sample landing
        luckily.

        LEVEL. WARN when the leak exceeds ``RING_LEAK_WARN_FRAMES`` on any bus, or
        when any hardware FIFO overflow has been recorded. The overflow term is
        not redundant with the leak: a leaked frame is merely LATE, an overflowed
        one is GONE, and the second is both worse and — until FW 13's vendored
        patch counted it — completely invisible.

        ``flags`` is read BEFORE the lag fields, and the lag is rendered ``n/a``
        while unseeded: an unseeded 0 means no reference exists, not zero lag.
        Likewise ``sdo_rtt_count == 0`` blanks the three RTT rows rather than
        publishing a 0 µs round trip that never happened.

        THE LAG INTEGRAL IS NORMALISED HERE (S3 residual (a), closed
        2026-08-15). ``lag_now_us`` is published RAW and unchanged, but the row
        to read is ``lag_now_corrected_us``: the raw integral also accumulates
        the FlexCAN capture clock's drift against ``micros64()``, measured at
        **~237 ppm quiet and ~582 ppm streaming** — LOAD-dependent, while the
        fold rate that scales it is not — which is why it exceeded the 135 ms
        one-lap physical cap on a plant whose leak was identically 0. The caveat
        this replaces — "the trend is the measurement, the absolute is not" —
        now reads: **the trend AFTER the clock-ratio correction is the
        measurement; the RAW integral drifts at the measured 230-670 ppm and its
        absolute value is not a delivery lag.** ``LagClockNormalizer`` carries
        the derivation, including why subtracting the measured per-window
        divergence would be a tautology, why the rate is calibrated on leak-free
        windows, and why that calibration has to be TRAILING rather than pooled.

        An FW <= 12 board never sends 0x92, so on the pre-flash bridge this drains
        nothing and the topic records EMPTY — the visible absence the record
        list's add-never-trim rule is built around, not an error.
        """
        with self._lock:
            batch = self._ring_diag_queue
            self._ring_diag_queue = []
        for rd in batch:
            # Signed, and clamped at 0 for display only where a negative reading
            # is meaningless: the leak cannot be negative in the mechanism, and a
            # -1 here is the documented +/-1 probe race (see Circular_Buffer::probe)
            # rather than a measurement. Clamping keeps a race artefact from
            # rendering as a number an operator would try to explain.
            leaks = {}
            for bus in ('jb', 'bb', 'cone'):
                depth = int(getattr(rd, f'true_depth_{bus}'))
                avail = int(getattr(rd, f'avail_reported_{bus}'))
                leaks[bus] = max(0, depth - avail)
            leak_hwms = {bus: int(getattr(rd, f'leak_hwm_{bus}'))
                         for bus in ('jb', 'bb', 'cone')}
            overflows = {bus: int(getattr(rd, f'fifo_overflows_{bus}'))
                         for bus in ('jb', 'bb', 'cone')}
            flags = int(rd.flags)
            lag_seeded = bool(flags & int(p.RingDiagFlags.LAG_SEEDED))
            reseeded = bool(flags & int(p.RingDiagFlags.LAG_RESEED_IN_WINDOW))
            rtt_n = int(rd.sdo_rtt_count)

            worst_leak = max(max(leaks.values()), max(leak_hwms.values()))
            any_overflow = any(v > 0 for v in overflows.values())

            # ── SINCE-BOOT vs THIS WINDOW ────────────────────────────────
            # leak_hwm_* is a maximum since BOOT — can_buses_ring_probe never
            # resets it. That is right for a diagnostics LEVEL (a verdict may be
            # cumulative) and wrong for everything below, both of which have to
            # be live: an alarm keyed on a since-boot maximum latches for the
            # rest of the session, and a calibration gate keyed on one goes dark
            # for the rest of the session. So the high-water is used only where
            # it ADVANCES this window, which is an event, and the spot leak
            # carries the level test.
            hwm_prev = self._ring_leak_hwm_prev
            hwm_fresh = {bus: (hwm_prev is not None
                               and leak_hwms[bus] > hwm_prev[bus])
                         for bus in ('jb', 'bb', 'cone')}
            # No baseline on the first window after node start: an advance
            # cannot be observed, so none is claimed. A plant that is STILL
            # leaking shows it in the spot leak within a window or two (S3 read
            # 247 on the spot pair), and a plant that leaked before this node
            # existed is history, not a live alarm.
            self._ring_leak_hwm_prev = dict(leak_hwms)

            # Feed the alarmed latency monitor. LIVE, not cumulative: the spot
            # leak, plus the high-water only in the window it advances in (the
            # spot pair is one sample out of ~1000 service-tick probes, so a
            # short excursion can hide between two 1 Hz reads — but only in the
            # window it actually happened in). The LEAK only — the summary token
            # set has no name for a FIFO overflow, and inventing one here would
            # report a lost frame under a label that means a late one;
            # /ring_diag's own level still WARNs on the overflow.
            spot_leak = max(leaks.values())
            self._lm_ring_leak = (max(spot_leak, max(leak_hwms.values()))
                                  if any(hwm_fresh.values()) else spot_leak)
            self._lm_ring_t = time.monotonic()

            # Normalise the delivery-lag integral (S3 residual (a)). The
            # calibration gate is the JUGGLEBOT bus only — that is the bus the
            # arrival clock folds — and it reuses RING_LEAK_WARN_FRAMES so
            # "clean enough to calibrate on" and "clean enough not to WARN" are
            # one definition rather than two that can drift apart. The
            # high-water term is the FRESH-ADVANCE form for the reason above: a
            # single historical excursion must not bar calibration for the rest
            # of the boot, or the instrument goes dark exactly at the uptime it
            # is wanted at. A fresh advance within the threshold still bars the
            # window — a 2-frame stranding that persists is ~1 ms in a 1 s
            # window, four times the artefact being estimated — and costing an
            # occasional window is the cheap side of that trade. A jugglebot
            # FIFO overflow is deliberately NOT part of the gate: a frame lost
            # inside the peripheral is never folded, so the unwrap simply spans
            # it and the capture-vs-decode divergence is unbiased; a loss long
            # enough to threaten the 16-bit wrap forces a reseed, which the
            # normaliser already segments at.
            ring_clean = (leaks['jb'] <= RING_LEAK_WARN_FRAMES
                          and not hwm_fresh['jb'])
            lagc = self._lag_norm.update(
                seeded=lag_seeded, reseed_in_window=reseeded,
                seq=int(rd.seq), t_local_us=int(rd.t_local_us),
                window_us=int(rd.window_us), cap_span_us=int(rd.cap_span_us),
                lag_now_us=int(rd.lag_now_us), lag_frames=int(rd.lag_frames),
                ring_clean=ring_clean)
            lag_corr = ('n/a' if lagc['corrected_us'] is None
                        else str(int(round(lagc['corrected_us']))))

            msg = DiagnosticStatus()
            msg.name = 'teensy/ring_diag'
            msg.hardware_id = 'can_bridge_teensy'
            msg.level = (DiagnosticStatus.WARN
                         if (worst_leak > RING_LEAK_WARN_FRAMES or any_overflow)
                         else DiagnosticStatus.OK)
            msg.message = (
                f'seq={int(rd.seq)} uptime={int(rd.t_local_us) // 1000}ms '
                f'leak_jb={leaks["jb"]} leak_hwm_jb={leak_hwms["jb"]} '
                f'lag_now={int(rd.lag_now_us) if lag_seeded else "n/a"}us '
                f'lag_corr={lag_corr}us '
                f'probes={int(rd.probe_ticks)}/{int(rd.window_us)}us')
            values = [
                # THE CONVICTION ROW, first: everything else is corroboration.
                KeyValue(key='leak_jb', value=str(leaks['jb'])),
                KeyValue(key='leak_hwm_jb', value=str(leak_hwms['jb'])),
                KeyValue(key='true_depth_jb', value=str(int(rd.true_depth_jb))),
                KeyValue(key='avail_reported_jb',
                         value=str(int(rd.avail_reported_jb))),
                KeyValue(key='true_depth_hwm_jb',
                         value=str(int(rd.true_depth_hwm_jb))),
                KeyValue(key='seq', value=str(int(rd.seq))),
                KeyValue(key='t_local_us', value=str(int(rd.t_local_us))),
                KeyValue(key='window_us', value=str(int(rd.window_us))),
                KeyValue(key='probe_ticks', value=str(int(rd.probe_ticks))),
                KeyValue(key='flags', value=str(flags)),
            ]
            for bus in ('bb', 'cone'):
                values.extend([
                    KeyValue(key=f'leak_{bus}', value=str(leaks[bus])),
                    KeyValue(key=f'leak_hwm_{bus}', value=str(leak_hwms[bus])),
                    KeyValue(key=f'true_depth_{bus}',
                             value=str(int(getattr(rd, f'true_depth_{bus}')))),
                    KeyValue(key=f'avail_reported_{bus}',
                             value=str(int(getattr(rd, f'avail_reported_{bus}')))),
                    KeyValue(key=f'true_depth_hwm_{bus}',
                             value=str(int(getattr(rd, f'true_depth_hwm_{bus}')))),
                ])
            # Hardware-FIFO counters: cumulative since boot, rendered RAW and
            # differenced by the consumer (the can3_errors / bridge_tx_diag
            # idiom). These are ISR-counted and therefore exact, unlike every
            # `_available`-derived number this frame exists to distrust.
            for bus in ('jb', 'bb', 'cone'):
                values.extend([
                    KeyValue(key=f'fifo_overflows_{bus}',
                             value=str(overflows[bus])),
                    KeyValue(key=f'fifo_warns_{bus}',
                             value=str(int(getattr(rd, f'fifo_warns_{bus}')))),
                ])
            # Delivery lag, jugglebot bus. 'n/a' while unseeded: the wire carries
            # 0 there, and a bare 0 reads as a healthy measured lag rather than as
            # the absence of a reference to measure against.
            values.extend([
                KeyValue(key='lag_seeded', value=str(int(lag_seeded))),
                KeyValue(key='lag_now_us',
                         value=str(int(rd.lag_now_us)) if lag_seeded else 'n/a'),
                KeyValue(key='lag_hwm_us',
                         value=str(int(rd.lag_hwm_us)) if lag_seeded else 'n/a'),
                KeyValue(key='lag_frames', value=str(int(rd.lag_frames))),
                KeyValue(key='cap_span_us', value=str(int(rd.cap_span_us))),
                KeyValue(key='lag_reseeds', value=str(int(rd.lag_reseeds))),
                # ── The normalised lag (S3 residual (a)) ──
                # THIS is the row to trend, not lag_now_us above. 'n/a' whenever
                # the correction cannot be made honestly: unseeded (no
                # reference on the wire) or uncalibrated (fewer than
                # _LAG_CAL_MIN_WINDOWS leak-free windows seen), never a 0 that
                # would read as a measured zero lag — the same discipline the
                # unseeded lag rows above follow.
                #
                # IT IS THE GROWTH CHANNEL, not an absolute lag: the value is
                # accumulated since the segment base, and the firmware reseeds
                # routinely mid-session, so a delay line that SATURATED before
                # this segment began reads ~0 here. `leak_jb` is the
                # absolute-occupancy channel; read them together.
                KeyValue(key='lag_now_corrected_us', value=lag_corr),
                # Which of those cases produced the value: 'ok' (accumulating),
                # 'segment_start' (this window restarted the series — a reseed
                # or a dropped uplink frame, so the 0 is a definition not a
                # measurement), 'uncalibrated', 'unseeded'.
                KeyValue(key='lag_corr_state', value=lagc['state']),
                # The calibration itself, published so the correction is
                # auditable from the bag alone rather than only reproducible
                # from the code — and because the rate is TRAILING, its value
                # over time is the record of the load state the plant was in.
                # us/frame is the internal unit; ppm is the same estimate in the
                # forensics' units. Both move together across a load change (the
                # fold rate is load-invariant), so neither is "the
                # load-independent one" — see LagClockNormalizer.
                KeyValue(key='lag_corr_rate_us_per_frame',
                         value=('n/a' if lagc['rate_us_per_frame'] is None
                                else f"{lagc['rate_us_per_frame']:.4f}")),
                KeyValue(key='lag_corr_rate_ppm',
                         value=('n/a' if lagc['rate_ppm'] is None
                                else f"{lagc['rate_ppm']:.1f}")),
                # Windows currently BACKING the rate. It saturates at
                # _LAG_CAL_TRAIL_WINDOWS by design (the estimator is trailing,
                # not pooled), so a value below that on a long-running session
                # means clean windows are scarce — itself diagnostic.
                KeyValue(key='lag_corr_windows',
                         value=str(int(lagc['cal_windows']))),
                # The correction's two denominators: frames folded and decode
                # time elapsed since the segment base. They are how a reader
                # tells a long clean segment from a freshly restarted one. (The
                # correction is accumulated per window at each window's own
                # rate, so these reproduce it only when the rate held steady.)
                KeyValue(key='lag_corr_frames',
                         value=str(int(lagc['segment_frames']))),
                KeyValue(key='lag_corr_segment_us',
                         value=str(int(lagc['segment_us']))),
                # A reseed moves the lag series' zero, so this window is not
                # comparable with the previous one. Surfaced per-window because
                # the cumulative counter alone cannot say WHICH sample to segment.
                KeyValue(key='lag_reseed_in_window', value=str(int(reseeded))),
            ])
            # SDO round trip. Count first, and it blanks the rest: 0 means no
            # round trip closed (poller off, parked, or every request timed out),
            # and a 0 us round trip is not a thing that can happen.
            values.append(KeyValue(key='sdo_rtt_count', value=str(rtt_n)))
            for key in ('sdo_rtt_min_us', 'sdo_rtt_max_us', 'sdo_rtt_last_us'):
                values.append(KeyValue(
                    key=key,
                    value=str(int(getattr(rd, key))) if rtt_n else 'n/a'))
            msg.values = values
            self.ring_diag_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════
    # Read-side publishers (mirror can_node field-by-field)
    # ═══════════════════════════════════════════════════════════

    def _build_motor_states(self, telem: Telemetry,
                            diag: dict[int, Diagnostic]) -> list:
        """Build a NUM_AXES list of MotorStateSingle from telemetry + diagnostics.

        pos/vel come from the 100 Hz Telemetry frame; per-axis state, errors,
        currents, temps, and bus voltage/current come from the on-change
        Diagnostic frame for that axis. Fields the can-bridge link does not
        carry (procedure_result, trajectory_done) are left at their
        MotorStateSingle defaults — documented in the handoff. Mirrors the
        construction can_node feeds into RobotState.motor_states.
        """
        states = []
        for axis in range(_NUM_AXES):
            s = MotorStateSingle()
            # pos/vel are already in Jugglebot convention (positive = extension)
            # — the Teensy sign-corrects from ODrive convention before sending.
            s.pos_estimate = float(telem.pos_rev[axis])
            s.vel_estimate = float(telem.vel_rps[axis])
            d = diag.get(axis)
            if d is not None:
                s.current_state = int(d.axis_state)
                s.active_errors = int(d.active_errors)
                s.disarm_reason = int(d.disarm_reason)
                s.iq_setpoint = float(d.iq_setpoint)
                s.iq_measured = float(d.iq_measured)
                s.fet_temp = float(d.temp_fet)
                s.motor_temp = float(d.temp_motor)
                s.bus_voltage = float(d.bus_voltage)
                s.bus_current = float(d.bus_current)
            states.append(s)
        return states

    def _build_bb_motor_states(self, diag: dict[int, Diagnostic],
                               diag_mono: dict[int, float],
                               bb_est: BbAxisEstimates | None,
                               bb_est_mono: float,
                               now_mono: float) -> list:
        """Build the trailing BB entries for robot_state ([7]=pitch, [8]=hand), or [].

        Restores the can_node-era 9-axis motor_states layout the GUI indexes
        positionally (charts stores 7/8, BB fault dots). pos/vel come from the
        latest BB_AXIS_ESTIMATES sample (100 Hz, same cadence as this publish);
        everything else — state, errors, Iq, temps, bus V/I — from the 1 Hz BB
        DIAGNOSTIC stash (axis_ids 7/8).

        All-or-nothing, honest-silence gating (the leg_setpoint_echo
        philosophy: silence, never a stale flatline or a phantom zero):
          * BOTH axes must pass, else neither is appended — a lone surviving
            hand entry would land at index 7 and be read as the pitch motor
            (positional integrity beats partial data).
          * heartbeat_seen (flags bit1) must be set — a dark/absent BB axis
            still produces zero-filled diag frames; without this gate they
            would surface as a live all-zero motor.
          * heartbeat_stale (flags bit0) must be clear — a mid-session BB
            death freezes the last values; stale entries must vanish, not
            flatline. An ODrive with an ACTIVE error still heartbeats, so
            real faults remain visible (GUI dots) rather than gated away.
          * both the diag stash and the estimate sample must be recent
            (_BB_DIAG_FRESH_S / _BB_EST_FRESH_S) — a frozen stash from a dead
            uplink slot or a previous link session must not keep BB alive.
        """
        if bb_est is None or (now_mono - bb_est_mono) > _BB_EST_FRESH_S:
            return []
        states = []
        for axis in (_BB_PITCH_AXIS, _BB_HAND_AXIS):
            d = diag.get(axis)
            if d is None or (now_mono - diag_mono.get(axis, 0.0)) > _BB_DIAG_FRESH_S:
                return []
            flags = int(d.flags)
            if not (flags & _DIAG_FLAG_HB_SEEN) or (flags & _DIAG_FLAG_HB_STALE):
                return []
            s = MotorStateSingle()
            if axis == _BB_PITCH_AXIS:
                s.pos_estimate = float(bb_est.pitch_pos_rev)
                s.vel_estimate = float(bb_est.pitch_vel_rps)
            else:
                s.pos_estimate = float(bb_est.hand_pos_rev)
                s.vel_estimate = float(bb_est.hand_vel_rps)
            s.current_state = int(d.axis_state)
            s.active_errors = int(d.active_errors)
            s.disarm_reason = int(d.disarm_reason)
            s.iq_setpoint = float(d.iq_setpoint)
            s.iq_measured = float(d.iq_measured)
            s.fet_temp = float(d.temp_fet)
            s.motor_temp = float(d.temp_motor)
            s.bus_voltage = float(d.bus_voltage)
            s.bus_current = float(d.bus_current)
            states.append(s)
        return states

    def _publish_leg_setpoint_echo(self):
        """Echo the last ACCEPTED leg setpoint u0 (6 floats, motor revs) for the GUI.

        Rides the 100 Hz robot_state timer (executor thread) — the setpoint
        thread only stashes (see _process_setpoint); it NEVER publishes. Two
        gates keep the topic honest:
          * freshness — no accepted frame within _SETPOINT_ECHO_STALE_S ⇒
            publish nothing, so downstream sees silence (chart gap), not a
            stale flatline;
          * dedup — each accepted frame (seq) is published at most once, so
            the topic carries the real ~40 Hz stream, not 100 Hz repeats.
        """
        try:
            with self._lock:
                u0 = self._last_accepted_u0
                mono = self._last_accepted_u0_mono
                seq = self._last_accepted_u0_seq
            if u0 is None or seq == self._echo_last_pub_seq:
                return  # nothing accepted yet / this frame already echoed
            if (time.monotonic() - mono) > _SETPOINT_ECHO_STALE_S:
                return  # stream stopped — silence, never a stale flatline
            msg = Float64MultiArray()
            msg.data = [float(v) for v in u0]
            self.leg_setpoint_echo_pub.publish(msg)
            self._echo_last_pub_seq = seq
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Setpoint echo publish error: {e}",
                                    throttle_duration_sec=5.0)

    # Fields this node assigns on RobotState that a STALE jugglebot_interfaces
    # build will not have. Keep in step with _publish_robot_state's assignments.
    _ROBOT_STATE_REQUIRED_FIELDS = ('platform_fw_version', 'platform_fw_version_read')

    def _warn_if_robot_state_msg_is_stale(self):
        """Name a half-rebuilt tree AT LAUNCH, once, instead of at BOOT timeout.

        A stale ``jugglebot_interfaces`` build is the row-B deployment skip, and it
        is NOT self-announcing on this node the way it is on ``trajectory_node``.
        The difference is a ``try/except``: ``trajectory_node._publish_status`` has
        no handler, so rclpy re-raises out of ``spin()`` and the process EXITS ~200
        ms after launch (e36d60d) — impossible to miss. ``_publish_robot_state``
        catches its own exceptions (it must: it also drives the setpoint echo, and
        a 100 Hz publisher that dies on one bad frame would take the leg-command
        observability path down with it). So the same stale build here degrades to
        ONE throttled ERROR per 5 s while ``/robot_state`` silently stops at 100 Hz
        — and the orchestrator, which waits on ``robot_state``, then reports
        "Check power and CAN connections", routing the operator to the CAN bus for
        a pure deployment fault.

        This check closes that gap without adding a refusal: it does not raise, it
        does not gate publishing, it runs once at construction. Same principle as
        the Platform FW_VERSION check this phase adds — a skipped deployment step
        must NAME ITSELF, not present as a hardware fault.
        """
        # hasattr on a constructed instance, deliberately: it is the one probe that
        # is correct for BOTH a real rosidl message (a missing field is absent from
        # __slots__, so hasattr is False) and the dataclass stand-in in
        # tests/ros/conftest.py. Introspecting the class (__slots__ /
        # get_fields_and_field_types) is correct for only one of the two.
        try:
            probe = RobotState()
            missing = [f for f in self._ROBOT_STATE_REQUIRED_FIELDS
                       if not hasattr(probe, f)]
        except Exception as e:  # noqa: BLE001 — a probe must never block construction
            self.get_logger().warning(
                f'could not introspect RobotState for a staleness check: {e}')
            return
        if not missing:
            return
        self.get_logger().error(
            'INTERFACES_STALE: the installed jugglebot_interfaces RobotState is '
            f'missing {missing} — robot_state will STOP PUBLISHING (one throttled '
            '"Robot state publish error" per 5 s) and the orchestrator will stall '
            'in BOOT reporting a power/CAN fault that does not exist. This is a '
            'HALF-REBUILT TREE, not a hardware fault. Fix: colcon build '
            '--packages-select jugglebot_interfaces jugglebot && source '
            'install/setup.bash, then relaunch.')

    def _publish_robot_state(self):
        """Publish robot_state, mirroring can_node._publish_robot_state.

        Suppressed until the first Telemetry frame arrives so the topic never
        carries a misleading all-zero / all-IDLE snapshot before the link is up.

        Also drives the leg_setpoint_echo publish off this same 100 Hz timer
        (see _publish_leg_setpoint_echo) — deliberately BEFORE the telemetry
        suppression gate, since the echo has its own freshness gate.
        """
        try:
            # Echo first (before the telemetry gate below): the echo has its
            # own freshness gate and must publish even while robot_state is
            # suppressed pre-telemetry.  Inside this try as containment —
            # _publish_leg_setpoint_echo catches its own exceptions, but a
            # pathological escape (e.g. the attribute itself broken) must not
            # take robot_state publishing down with it.
            self._publish_leg_setpoint_echo()
            with self._lock:
                telem = self._latest_telemetry
                telem_gen = self._telemetry_gen
                hb = self._latest_heartbeat
                diag = dict(self._latest_diag)
                diag_mono = dict(self._latest_diag_mono)
                bb_est = self._latest_bb_est
                bb_est_mono = self._latest_bb_est_mono
                cold = self._cold_start_state            # immutable namedtuple
                fw_version = self._platform_fw_version   # int | None
                search_session = self._encoder_search_done_session
                fw_validated = self._firmware_validated
                fw_mismatch = self._firmware_mismatch_error
            if telem is None:
                return  # No real data yet — don't publish a phantom snapshot.

            # ── Honesty gate: never republish a latch that has not moved ──────
            # This 100 Hz timer and the ~100 Hz TELEMETRY arrival are
            # independent, so they beat: some periods carry two telemetry frames
            # and some carry none, and on the "none" periods the pre-gate code
            # re-serialised the SAME latch under a fresh header stamp. That
            # manufactured a duplicate sample which is indistinguishable, in a
            # bag, from the robot genuinely holding still — and it is why
            # measured "97 % identical consecutive samples" was an artefact of
            # this publisher rather than a property of the plant. Skipping the
            # duplicate makes the published series follow the TELEMETRY rate
            # instead of the timer, so the mean stays ~100 Hz while the
            # fabricated rows disappear.
            #
            # THE LINK-LOST CARVE-OUT IS NOT OPTIONAL. When the UDP link dies,
            # telemetry stops, so the generation freezes — and a gate on
            # freshness alone would then suppress /robot_state entirely, exactly
            # when has_fatal_can_error below is being raised BECAUSE the link
            # died. The orchestrator has no arrival-based watchdog on this topic
            # (its liveness input is the message CONTENT), so that would turn the
            # loudest failure the bridge can report into total silence — strictly
            # worse than the duplicate this gate exists to remove. While the link
            # is lost we therefore keep publishing at the full timer rate, which
            # is the pre-existing behaviour on precisely the path that needs it.
            #
            # Placed AFTER _publish_leg_setpoint_echo() above, deliberately: that
            # echo has its own freshness gate, feeds the GUI's quiescence
            # detector (which counts samples in a fixed window), and must keep
            # publishing even while robot_state is suppressed pre-telemetry.
            #
            # Non-telemetry inputs (diagnostics, heartbeat, BB, cold-start) can
            # change without a new telemetry frame; they are delayed by at most
            # one telemetry period (~10 ms) because telemetry arrives at 100 Hz
            # whenever the link is up — and when it is NOT up, the carve-out
            # above publishes them anyway.
            #
            # THE SUPPRESSION IS BOUNDED (50 consecutive skips ≈ 0.5 s): the
            # 10 ms claim above assumes telemetry flows whenever heartbeats do,
            # but the firmware emits them from SEPARATE tasks (task_telem vs
            # task_heartbeat). If task_telem wedges while heartbeats keep the
            # link "up", an unbounded gate would silence /robot_state — and the
            # orchestrator's fatal-fault inputs ride this topic's CONTENT, so
            # heartbeat-carried fault transitions would never arrive. Publishing
            # a (stale-telemetry) frame every 0.5 s keeps the dedup property for
            # the artifact this gate exists to remove while capping the fault
            # latency of the double-failure case.
            link_lost = bool(self._link_latch.link_lost)
            if (telem_gen == self._robot_state_pub_gen and not link_lost
                    and self._robot_state_consec_skips < _ROBOT_STATE_MAX_CONSEC_SKIPS):
                self._robot_state_stale_skips += 1
                self._robot_state_consec_skips += 1
                return
            self._robot_state_consec_skips = 0
            self._robot_state_pub_gen = telem_gen

            states = self._build_motor_states(telem, diag)
            # Append the BB axes ([7]=pitch, [8]=hand) when live — the GUI's
            # charts/fault-viz consume them positionally from robot_state
            # (can_node 9-axis parity). Gated to honest-silence when BB is
            # dark/stale (see _build_bb_motor_states); every leg/hand
            # computation below slices states[:_NUM_LEGS] or indexes 0..6,
            # so the trailing entries never feed the fatal-fault logic.
            states += self._build_bb_motor_states(
                diag, diag_mono, bb_est, bb_est_mono, time.monotonic())

            msg = RobotState()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.motor_states = states

            # Typed fault flags. The Teensy owns the fault state machine, so the
            # headline determination is its single-valued HeartbeatT2J.fault_state.
            # But fault_state is single-valued, so a higher-priority fault (e.g.
            # CAN_BUS_DOWN) can MASK a concurrent ODrive fault — which would make
            # robot_state under-report a real per-leg fault for the same hardware
            # state. So we also OR in the
            # raw per-leg fatal conditions can_node uses (active error on any leg,
            # or disarm-while-CLOSED_LOOP — can_node._handle_error:416-421), keeping
            # the comparison faithful WITHOUT re-running the Teensy's stateful
            # soft-reset machine on the Jetson (which fault_state already reports).
            fault_state = int(hb.fault_state) if hb is not None else 0
            # After the three-bus remap (ADR-0013) the on-wire
            # bus1_health slot carries the Jugglebot CORE bus (CAN3: 6 legs + hand) --
            # the bus whose BUS_OFF is fatal for the legs. (bus2_health is now Ball
            # Butler; the cone bus health is not yet on the uplink -- TODO (cone-uplink work).)
            core_bus_health = int(hb.bus1_health) if hb is not None else 0
            legs = states[:_NUM_LEGS]  # the can-bridge owns legs 0-5 (hand = platform Teensy)
            any_leg_active_err = any(s.active_errors != 0 for s in legs)
            # CROSS-AXIS disarm-while-CLOSED_LOOP (fault-parity hardening; exact
            # parity with fault_logic.py:117 `any_disarmed and any_cl` + can_node):
            # ANY leg disarmed while ANY leg still holds CLOSED_LOOP is fatal. The
            # previous per-leg conjunction (the SAME leg disarmed AND CLOSED_LOOP)
            # essentially never fired — a disarmed ODrive leaves CLOSED_LOOP almost
            # immediately — so the common real event (one leg drops torque while the
            # others keep holding) was missed by exactly the OR-term that exists to
            # un-mask a fault while fault_state carries a higher-priority code.
            any_leg_disarmed = any(s.disarm_reason != 0 for s in legs)
            any_leg_in_cl = any(
                s.current_state == _AXIS_STATE_CLOSED_LOOP for s in legs)
            disarm_while_cl = any_leg_disarmed and any_leg_in_cl
            # A firmware-version MISMATCH is a fatal ODrive condition (can_node set
            # fatal_error=True → has_fatal_odrive_error=True, can_node:491/1080). It
            # is a latched OR-term recomputed each publish (sticky for the session).
            # NOTE (audit 2026-06-29 LOW, intentional): because the latch is sticky,
            # this stays True even after a CLEAR_ERRORS, where can_node's
            # has_fatal_odrive_error (= fatal_error) would drop to False. The
            # divergence is deliberate — a wrong-firmware ODrive IS a hard config
            # error that must stay fatal until the firmware is fixed + the bridge
            # restarted (the plan's "latched OR-term" instruction); both behaviours
            # still force-FAULT via the sticky error string in robot_state.error.
            fw_mismatch_present = fw_mismatch is not None
            msg.has_fatal_odrive_error = (
                fault_state == int(FaultState.ODRIVE_FATAL)
                or any_leg_active_err or disarm_while_cl
                or fw_mismatch_present)
            # Safety hardening: surface a dead Jetson↔Teensy UDP link as a fatal
            # CAN error. The orchestrator's ONLY health input is robot_state; without
            # this, a lost link leaves it consuming FROZEN motor states stamped with
            # fresh clock times + healthy flags indefinitely (can_node coupled 2 s
            # silence → fatal_can_error; that coupling drove the 2026-05-19 stow work
            # and was dropped in the port). The LinkLossLatch never arms before the
            # first heartbeat, so this OR is boot-safe. Read outside self._lock (the
            # latch is a separate object, updated by the 1 Hz _health_check).
            # link_lost was read at the honesty gate above — reused rather than
            # re-sampled, so the flag published here cannot disagree with the
            # flag that decided this message would be published at all.
            msg.has_fatal_can_error = (
                fault_state == int(FaultState.CAN_BUS_DOWN)
                or core_bus_health == int(BusHealth.BUS_OFF)
                or link_lost)
            # Undervoltage: matches can_node, which sets undervoltage_error ONLY
            # from a BITWISE test on active_errors (can_node._handle_error:436);
            # disarm_reason==UV is used by can_node solely in its clear predicate,
            # never to ASSERT UV. active_errors/disarm are ODrive bitfields, so use
            # bitwise & (not ==), active_errors only.
            msg.has_undervoltage = any(
                s.active_errors & _ERR_DC_BUS_UNDER_VOLTAGE for s in legs)
            # firmware_validated (version handshake): latched by the GET_AXIS_VERSIONS
            # handshake (_version_check_poll → validate_group). False until the
            # bridge has pulled + validated the ODrive versions; un-gates the
            # orchestrator's is_homed skip (state_machine.py:228-235) when True.
            msg.firmware_validated = fw_validated

            # Human-readable error strings (logging/rosbag parity with can_node).
            errors = []
            # Firmware-version mismatch FIRST (can_node:1085-1086 order) — its
            # presence in robot_state.error force-FAULTs the orchestrator
            # (orchestrator_node.py:137-139), the parity stop for a wrong-firmware
            # ODrive. Recomputed each publish from the sticky latch.
            if fw_mismatch_present:
                errors.append(fw_mismatch)
            if msg.has_undervoltage:
                errors.append("Undervoltage detected. Was the E-stop hit?")
            if msg.has_fatal_odrive_error:
                errors.append(
                    f"Fatal ODrive issue (Teensy fault_state="
                    f"{_enum_name(FaultState, fault_state)}).")
                # F3/C3 — name the actual ODrive error(s) beside the coarse
                # fault_state string, so a bag or the GUI's error[0]-and-friends
                # readouts say WHICH axis and WHICH condition instead of leaving
                # every consumer to re-decode the raw bitfields.
                #
                # STRICTLY INSIDE THE FATAL BRANCH — non-negotiable.
                # orchestrator_node._tick force-FAULTs on ANY non-empty
                # robot_state.error[], so a decoded line appended for a
                # NON-fatal bit (a hand/BB axis error, a leg disarm with no leg
                # in CLOSED_LOOP) would manufacture a brand-new FAULT path out
                # of pure observability. The decode adds detail to a fault that
                # has already been declared; it never declares one.
                #
                # APPENDED, never prepended: state-minimap.js reads error[0] as
                # the headline, and that headline must stay the coarse cause.
                for axis in sorted(diag):
                    d = diag[axis]
                    a_err, d_err = int(d.active_errors), int(d.disarm_reason)
                    if a_err == 0 and d_err == 0:
                        continue
                    errors.append(
                        f"ODrive {_axis_label(axis)}: "
                        f"{_decode_axis_errors(a_err, d_err)}")
            if msg.has_fatal_can_error:
                # Distinguish the UDP-link-loss cause (safety hardening) from a CAN3
                # bus fault, so an operator sees WHY robot_state went fatal.
                errors.append(
                    "Teensy link lost (UDP) — telemetry is stale."
                    if link_lost else "Fatal CAN bus issue.")
            msg.error = errors

            # Teensy-persisted cold-start state — sourced from the
            # Platform Teensy's RobotState via the relay STATE_READ (cached at boot
            # + on CAN3 reconnect; the publish path is non-blocking — it reads only
            # the cache, never a CAN3 round-trip). Replaces the hardcoded
            # conservative defaults of the earlier side-by-side bring-up.
            is_homed = bool(cold.is_homed)
            msg.is_homed = is_homed
            msg.levelling_complete = bool(cold.levelling_complete)
            # encoder_search_complete is DERIVED, not independently persisted —
            # exact can_node parity (can_node.py:549-550 sets it True whenever
            # is_homed; homing requires a prior search, so is_homed ⇒ search done)
            # plus the in-session bit set on a successful /encoder_search.
            msg.encoder_search_complete = is_homed or search_session
            tilt_x = float(cold.pose_offset_tiltX)
            tilt_y = float(cold.pose_offset_tiltY)
            msg.pose_offset_rad = [tilt_x, tilt_y]
            # Recompute the quat only when the tilt changes (memo — see __init__);
            # the publish-thread serialization makes the bare reads/writes safe.
            key = (tilt_x, tilt_y)
            if key != self._pose_quat_key:
                q = _tilt_to_quat(tilt_x, tilt_y)
                self._pose_quat_key = key
                self._pose_quat_xyzw = (q.x, q.y, q.z, q.w)
            qx, qy, qz, qw = self._pose_quat_xyzw
            msg.pose_offset_quat = Quaternion(x=qx, y=qy, z=qz, w=qw)

            # Platform-Teensy firmware identity, out of the SAME 0x6E0 frame as
            # is_homed / levelling / pose above — one frame, one message.
            # Assigned as PLAIN ATTRIBUTES (never setattr-with-default), the
            # gravity_correction_loaded precedent from e36d60d: a stale
            # jugglebot_interfaces build must FAIL, not be papered over.
            # BUT NOTE THE LIMIT, and do not repeat the claim this comment used to
            # make: unlike trajectory_node (whose timer has no handler, so rclpy
            # re-raises out of spin() and the process exits ~200 ms after launch),
            # this function catches its own exceptions below. A stale build here
            # therefore yields ONE throttled ERROR per 5 s and a silently-dead
            # /robot_state — NOT a dead node. That gap is what
            # _warn_if_robot_state_msg_is_stale() closes, at construction, by name.
            # Explicit None test, not `fw_version or 0`: 0 is a MEANINGFUL value
            # here (a pre-versioning board) and the falsy-0 idiom would read as if
            # it were being treated as "missing".
            msg.platform_fw_version = 0 if fw_version is None else int(fw_version)
            msg.platform_fw_version_read = fw_version is not None

            self.robot_state_pub.publish(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Robot state publish error: {e}",
                                    throttle_duration_sec=5.0)

    def _hand_sensor_snapshot(self):
        """Cached HAND_SENSOR frame + the Jetson-side freshness verdicts.

        Returns ``(frame, rx_fresh, valid)``. ``frame`` is None until the first
        HAND_SENSOR arrives — tri-state UNKNOWN (boot, or a bridge running
        firmware older than the Phase 3/4 poller, which never sends one).

        ``valid`` is the Jetson-side gate and closes the hop the bridge cannot
        see: the frame's own ``VALID`` (a gated good SDO reply) and
        ``TIME_SYNCED`` (the bridge's wall anchor is set, so ``t_bridge_us``
        means something) flags must BOTH be set, AND the frame must have
        arrived within ``_HAND_SENSOR_RX_FRESH_S`` of now on the HOST monotonic
        clock. The publisher is a free-running timer over this cache, so
        without the RX-age term a dead link republishes a stale "ball held"
        forever.
        """
        with self._lock:
            hs = self._latest_hand_sensor
            rx_mono = self._latest_hand_sensor_mono
        if hs is None:
            return None, False, False
        rx_fresh = (time.monotonic() - rx_mono) <= _HAND_SENSOR_RX_FRESH_S
        flags = int(hs.flags)
        valid = bool(rx_fresh
                     and flags & _HAND_SENSOR_FLAG_VALID
                     and flags & _HAND_SENSOR_FLAG_TIME_SYNCED)
        return hs, rx_fresh, valid

    def _publish_hand_telemetry(self):
        """Publish hand_telemetry from axis 6 of the Telemetry frame.

        Mirrors can_node._publish_hand_telemetry: the MEASURED side from the
        Telemetry/Diagnostic cache, and (hand conduit) the COMMAND side (pos_cmd/
        vel_ff_cmd/tor_ff_cmd) from the last sniffed HAND_CMD_ECHO — the hand's
        commanded-vs-measured tracking-error diagnostic (catch-tuning) that was
        dropped to 0 on the bridge until now. Also carries the hand ball-present
        sensor as a TRI-STATE (ball_held is meaningless unless ball_held_valid).
        """
        try:
            with self._lock:
                telem = self._latest_telemetry
                diag = self._latest_diag.get(_HAND_AXIS)
                cmd = dict(self._last_hand_cmd)
            if telem is None:
                return
            msg = HandTelemetryMessage()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.pos_cmd = float(cmd['pos'])
            msg.vel_ff_cmd = float(cmd['vel'])
            msg.tor_ff_cmd = float(cmd['tor'])
            msg.pos_meas = float(telem.pos_rev[_HAND_AXIS])
            msg.vel_meas = float(telem.vel_rps[_HAND_AXIS])
            msg.iq_meas = float(diag.iq_measured) if diag is not None else 0.0
            # Ball-present sensor. Never-seen leaves the message defaults
            # (False/False/False + a zero stamp) — an unflashed bridge must not
            # look like a confident "no ball". A seen-but-not-valid frame keeps
            # its last-known bits and stamp (the "how stale, and what was it?"
            # diagnostic) with ball_held_valid False marking them untrustworthy.
            hs, _rx_fresh, hs_valid = self._hand_sensor_snapshot()
            msg.ball_held_valid = hs_valid
            if hs is not None:
                flags = int(hs.flags)
                msg.ball_held = bool(flags & _HAND_SENSOR_FLAG_DEBOUNCED_HELD)
                msg.ball_held_raw = bool(flags & _HAND_SENSOR_FLAG_RAW_HELD)
                # Bridge wall-clock, already Jetson-epoch microseconds (the
                # bridge is the time-sync master) — same conversion as
                # _publish_bb_axis_estimates, NO Jetson-side offset.
                t_us = int(hs.t_bridge_us)
                msg.ball_held_stamp.sec = t_us // 1_000_000
                msg.ball_held_stamp.nanosec = (t_us % 1_000_000) * 1000
            self.hand_telemetry_pub.publish(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Hand telemetry error: {e}",
                                    throttle_duration_sec=5.0)

    def _link_age_us(self) -> int | None:
        """Microseconds since the last T→J heartbeat, or None if never seen."""
        return self._client.time_since_last_t2j_heartbeat_us()

    # ═══════════════════════════════════════════════════════════
    # Link-loss watchdog + deferred-stow latch (Commit 2)
    # ═══════════════════════════════════════════════════════════

    def _health_check(self):
        """1 Hz link-loss watchdog — the Jetson↔Teensy analog of can_node's
        _watchdog_check. Drives the deferred-stow latch.

        Invariant (ported from logbook/2026-05-19-can-loss-fault-response-
        safety-inversion.md): on confirmed link loss, ARM the deferred-stow
        latch and do NOT command the Teensy (the firmware's own fault machine
        holds the legs safely while the link is down — and frames wouldn't be
        delivered anyway). On confirmed reconnect, the latch stays ``stow_pending``
        and the bridge SURFACES it on link_status for the operator /
        orchestrator. The bridge does NOT auto-execute a stow: there is no stow
        RPC until a future firmware release, and the Teensy already owns the profiled
        CAN-side stow. This is the "always stow on confirmed
        reconnect, never command a dead link" invariant, minus the executor the
        bridge does not have.
        """
        try:
            age_us = self._link_age_us()
            seen = age_us is not None
            stale = seen and age_us > self._heartbeat_timeout_s * 1e6
            self._link_latch.update(stale=bool(stale), seen=bool(seen))

            if self._link_latch.link_lost and not self._last_link_lost:
                self.get_logger().error(
                    "Teensy link LOST — deferred stow armed; NOT commanding "
                    "the Teensy while the link is down (firmware holds the legs).")
            elif not self._link_latch.link_lost and self._last_link_lost:
                if self._link_latch.stow_pending:
                    self.get_logger().error(
                        "Teensy link RESTORED — a mid-run loss occurred. "
                        "STOW PENDING: no stow RPC exists yet (future firmware); "
                        "operator/orchestrator must stow the platform. Surfaced "
                        "on link_status (bridge_stow_pending=1).")
                else:
                    self.get_logger().info("Teensy link RESTORED.")
                # Safety hardening: forget the pre-loss setpoint so the per-step
                # gate can't wedge after a firmware stow. During the loss the firmware
                # may have stowed the legs to ~0 rev, but the pump's _prev_pos still
                # holds the pre-loss active pose (~2.2 rev) — so the first
                # post-reconnect MPC frame would fail the 0.3 rev step gate FOREVER
                # (reject loop until a node restart). reset() drops _prev_pos so the
                # next frame is accepted; the firmware MAX_DEVIATION gate remains the
                # complementary command-vs-encoder layer (setpoint_pump's documented
                # two-layer design). SAFE — it only forgets the prior frame, never
                # relaxes a bound.
                self._sp_pump.reset()
                # Cold-start refresh: on a confirmed UDP-link reconnect, re-read the Platform
                # Teensy's cold-start state (it is the authoritative store; a
                # reconnect only triggers a re-read, never INFERS reference state).
                # Refresh off the publish path; KEEP the cached value if the read
                # fails — a Jetson↔Teensy link blip leaves the Platform Teensy (and
                # its references) powered, so a re-home would be wrong (can_node
                # passive last-known parity). Dispatched to a daemon thread
                # so the relay round-trip can't stall the 100 Hz publish (shared
                # callback group). Best-effort: if a conservative CAN3-recovery
                # re-read already holds the shared guard, this keep-stale read is
                # simply skipped — the conservative read is the stronger refresh.
                self._dispatch_cold_start_reread(
                    self._refresh_cold_start_state, 'reconnect', 'cs-udp-reread')
            self._last_link_lost = self._link_latch.link_lost

            # Firmware-validation precondition (audit 2026-06-29): CAN3-bus-health reconnect
            # re-trigger. The UDP-watchdog edge above does NOT fire for a CAN3-only
            # drop — if Jugglebot is disconnected while the Jetson + can-bridge stay
            # powered, the UDP link never drops, so the UDP-reconnect re-read
            # never runs and the cache could hold a STALE is_homed=True against a
            # de-referenced robot (the hole that goes LIVE once firmware validation ungates the
            # orchestrator's is_homed skip). bus1_health (CAN3, the Jugglebot core
            # bus) recovering to OK from a DEGRADED state (WARN/BUS_OFF) signals the
            # Platform Teensy may have lost + regained power (it shares the ODrive
            # supply), so re-read CONSERVATIVELY (retry,
            # then is_homed=False on failure — NOT keep-stale). UNKNOWN→OK (the boot
            # first-connection) is excluded: it is not a loss, and the __init__ boot
            # read already covered it (re-reading there could needlessly clobber a
            # good boot value with the conservative fallback).
            with self._lock:
                hb = self._latest_heartbeat
            cur_bus1 = int(hb.bus1_health) if hb is not None else None
            _degraded = (int(BusHealth.WARN), int(BusHealth.BUS_OFF))
            if (cur_bus1 == int(BusHealth.OK)
                    and self._last_bus1_health in _degraded):
                self.get_logger().warning(
                    "Jugglebot core bus health recovered to OK (was "
                    f"{_enum_name(BusHealth, self._last_bus1_health)}) — Jugglebot "
                    "may have power-cycled; conservative cold-start re-read.")
                # OFF the 1 Hz timer thread — the re-read can take ~1.9 s and must
                # not stall the 100 Hz publish (shared callback group; audit MEDIUM).
                dispatched = self._dispatch_cold_start_reread(
                    self._read_cold_start_state_conservative,
                    'can3_reconnect', 'cs-can3-reread')
                # Consume the recovery edge ONLY once the conservative re-read is
                # under way. If a same-tick UDP-reconnect keep-stale read won the
                # shared guard (dispatched=False), leave _last_bus1_health DEGRADED
                # so the edge re-fires next tick — the conservative read (is_homed=
                # False on failure) must never be permanently preempted by the
                # weaker keep-stale path.
                if dispatched:
                    self._last_bus1_health = cur_bus1
            elif cur_bus1 is not None:
                self._last_bus1_health = cur_bus1
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Health check error: {e}",
                                    throttle_duration_sec=5.0)

    # ═══════════════════════════════════════════════════════════
    # Firmware-version handshake — Get_Version → validate_group
    # ═══════════════════════════════════════════════════════════

    def _version_check_poll(self):
        """Pull the bridge's cached ODrive versions (GET_AXIS_VERSIONS) and run the
        tested validate_group, latching firmware_validated / the mismatch string.

        No-op once resolved (validated, or a mismatch latched). Runs OFF the publish
        path: GET_AXIS_VERSIONS reads a bridge-LOCAL cache (no CAN3 round-trip), so
        the RPC is a cheap UDP round-trip. A failed pull — old firmware
        (ERR_UNKNOWN_METHOD / ERR_NOT_IMPL) or the firmware sweep not yet complete —
        just retries next tick; the orchestrator's BOOT_TIMEOUT_S governs the
        give-up, exactly as can_node validated asynchronously as Get_Version replies
        arrived (can_node._handle_get_version:487-495)."""
        with self._lock:
            if self._firmware_validated or self._firmware_mismatch_error:
                return
        try:
            # GET_AXIS_VERSIONS is a bridge-LOCAL cache read (sub-ms RTT normally),
            # but this poll shares _publish_robot_state's callback group, so bound
            # the worst-case block on a degraded link (audit 2026-06-29 LOW): one
            # short timeout + one retry rather than the default budget. A miss just
            # retries next 1 Hz tick (BOOT_TIMEOUT governs the give-up).
            ok, msg, blob = self._call_rpc(
                RpcMethod.GET_AXIS_VERSIONS, timeout=0.3, retries=1)
        except Exception as e:  # noqa: BLE001 — never let a timer die
            self.get_logger().error(f"version check pull errored: {e}",
                                    throttle_duration_sec=5.0)
            return
        if not ok or not blob:
            # Old firmware (ERR_UNKNOWN_METHOD/ERR_NOT_IMPL) or sweep not done →
            # cannot validate yet; keep firmware_validated=False and retry.
            self.get_logger().warning(
                f"firmware version pull unavailable ({msg}) — cannot validate yet",
                throttle_duration_sec=10.0)
            return
        try:
            per_axis = rpc_args.decode_axis_versions_result(blob)
            for axis, raw in per_axis.items():
                (_proto, hw_product, hw_ver, hw_variant,
                 fw_major, fw_minor, fw_rev, unrel) = odrive.decode_get_version(raw)
                # unrel first, so an axis never renders '-?' once record_version
                # lands (the renderer is total either way).
                self._fw_unreleased[axis] = unrel
                self._versions.record_version(
                    axis, (fw_major, fw_minor, fw_rev),
                    (hw_product, hw_ver, hw_variant))
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"version decode errored: {e}",
                                    throttle_duration_sec=5.0)
            return
        if not self._versions.all_jugglebot_versions_received():
            return  # firmware still sweeping — try again next tick
        # All present-and-expected Jugglebot axes reported — make a single
        # determination (can_node._handle_get_version:487-495 parity).
        error = self._versions.validate_group(odrive.JUGGLEBOT_AXES, "Jugglebot")
        with self._lock:
            if error:
                self._firmware_mismatch_error = error   # sticky; forces FAULT
            else:
                self._firmware_validated = True
        versions = self._odrive_fw_versions_str()
        if error:
            self.get_logger().error(
                f"Jugglebot firmware check FAILED: {error} (fw {versions})")
        else:
            self.get_logger().info(
                "Jugglebot firmware check PASSED — all axes match expected "
                f"versions (fw {versions})")

    def _odrive_fw_versions_str(self) -> str:
        """Per-axis ODrive firmware rendering, ``axis:major.minor.rev-unreleased``.

        The fourth Get_Version byte is rendered unconditionally so a reported zero
        is distinguishable from a byte that was never surfaced (which renders
        ``-?``, never a fabricated ``-0``). Axes with no
        version read show ``?`` rather than being dropped — a partial bench rig
        legitimately sweeps fewer axes (only those that heartbeat).
        """
        parts = []
        for axis in odrive.JUGGLEBOT_AXES:
            fw = self._versions.firmware_versions.get(axis)
            if fw is None:
                parts.append(f'{axis}:?')
            else:
                unrel = self._fw_unreleased.get(axis)
                parts.append(f'{axis}:{fw[0]}.{fw[1]}.{fw[2]}'
                             f"-{'?' if unrel is None else unrel}")
        return ' '.join(parts)

    def _hand_ball_sensor_str(self) -> str:
        """``/link_status`` rendering of the hand ball sensor.

        Tri-state by construction: UNKNOWN is rendered explicitly (and
        ``never seen`` distinguishes "no frame has ever arrived" from "a frame
        arrived but could not be trusted") and is NEVER rendered as ``empty``
        — boot, staleness and an un-anchored bridge clock all mean "we don't
        know", not "no ball" (§ Architecture, normative).

        The raw ``get_gpio_states`` word rides along in hex because it is the
        Phase 7 step 2 gate's observable: a wrong-but-existing endpoint id
        answers with a plausible CONSTANT, so only watching bit 2 of this word
        move on both edges proves the id, the pin mode and the wiring.
        """
        hs, rx_fresh, valid = self._hand_sensor_snapshot()
        if hs is None:
            return 'unknown (never seen)'
        flags = int(hs.flags)
        if valid:
            state = 'held' if flags & _HAND_SENSOR_FLAG_DEBOUNCED_HELD else 'empty'
        elif not rx_fresh or flags & _HAND_SENSOR_FLAG_STALE:
            # Distinguish "the reading aged out" (either hop) from "the bridge
            # has a fresh frame it still won't vouch for" (e.g. no wall anchor).
            state = 'stale'
        else:
            state = 'unknown'
        # Lowercase hex to match the bridge console's %08lx — Phase 7 step 2
        # compares the two surfaces textually.
        return (f'{state} miss={int(hs.miss_count)} '
                f'raw=0x{int(hs.raw_states):08x}')

    def _can3_errors_str(self) -> str:
        """``/link_status`` rendering of the CAN3 wire-error counters.

        One compact row, because this is an INSTRUMENT rather than a status:
        the operator reads it by differencing two captures (poller on vs off)
        rather than by looking at any single value. It is the Jetson-side half
        of the discriminator table in
        ``logbook/2026-07-29-can3-bus-health-flap-hand-sensor-poller.md`` — the
        2026-07-29 flap could be SEEN in a bag (``bus1_health`` at 42.4 % WARN)
        but not EXPLAINED from one, because these counters existed only on the
        bridge's serial console.

        ``flt``/``sust`` are the two that matter at a glance: ``flt=1`` is an
        instantaneous error-passive reading (a transient, and normal), while
        ``sust=1`` means it has persisted past ``CAN_PASSIVE_SUSTAIN_US`` and
        the command gate is actually refusing CAN3 commands.
        """
        with self._lock:
            ce = self._latest_can_errors
        if ce is None:
            # Never-seen is a real, expected state: any bridge older than FW 5.
            return 'unknown (never seen)'
        return (f'flt={int(ce.flt_live)} sust={int(ce.flt_sustained)} '
                f'tec={int(ce.tec_live)} rec={int(ce.rec_live)} '
                f'tecInc={int(ce.tec_inc_sum)} recInc={int(ce.rec_inc_sum)} '
                f'ack={int(ce.ack_cnt)} crc={int(ce.crc_cnt)} '
                f'form={int(ce.form_cnt)} stuff={int(ce.stuff_cnt)} '
                f'bit0={int(ce.bit0_cnt)} bit1={int(ce.bit1_cnt)} '
                f'txctx={int(ce.err_tx_ctx)} rxctx={int(ce.err_rx_ctx)} '
                f'gated={int(ce.tx_gated)}')

    def _bridge_tx_diag_str(self) -> str:
        """``/link_status`` rendering of the bridge's TX-pressure counters.

        One compact row, an INSTRUMENT rather than a status — read by
        differencing two captures (e.g. with and without the 500 Hz leg stream),
        never by looking at a single value.

        ``defer`` is the count of sends whose ``FlexCAN_T4::write()`` returned
        -1. That is a DEFERRAL into the 64-slot software TX queue, drained by the
        TX-complete ISR ~0.1-1 ms later — **not** a dropped frame. This is what
        makes ``hand_ops`` returning ``ERR_TIMEOUT`` and
        ``catch_coordinator``'s "the ack lies, frames were observed transmitted
        after a failed ack" both true at once. The two paths that genuinely lose
        a frame are TX-queue overflow (a 65th pending entry overwrites the
        oldest) and the vendored ``events()`` drain defect; ``txq`` approaching
        64 is the observable for the first.

        ``hand`` is the per-stage exit tally the 2026-08-01 recount could not
        get: three of ``hand_traj_cmd``'s exits return an identical bare
        ``ERR_TIMEOUT``, so on the wire they were indistinguishable. ``ok`` is
        derived (calls minus every failure class) rather than counted, so the
        row is arithmetically self-checking.

        A ONE-OFF ``ok=-1`` is benign and expected: the firmware's six counters
        are copied by ``task_telem`` while the RPC task may be incrementing
        them, so a frame can catch ``calls`` from before an invocation and a
        failure counter from after it. A PERSISTENT negative is real signal —
        it means an exit path stopped incrementing ``calls``, i.e. the
        denominator no longer covers the failure classes. The arithmetic is
        deliberately NOT clamped at zero: clamping would erase exactly that
        second case while hiding nothing useful about the first.

        Total by construction: field reads and integer arithmetic only. A
        renderer that raises takes the WHOLE ``/link_status`` message down, not
        just its own row.
        """
        with self._lock:
            d = self._latest_bridge_tx_diag
        if d is None:
            # Never-seen is a real, expected state: any bridge older than FW 9.
            return 'unknown (never seen)'
        calls = int(d.hand_calls)
        rej = int(d.hand_rej_homing)
        down = int(d.hand_bus_down)
        pre1 = int(d.hand_pre1_fail)
        pre2 = int(d.hand_pre2_fail)
        traj = int(d.hand_traj_fail)
        return (f'defer jb={int(d.tx_deferred_jb)} bb={int(d.tx_deferred_bb)} '
                f'cone={int(d.tx_deferred_cone)} '
                f'txq jb={int(d.tx_q_hwm_jb)} bb={int(d.tx_q_hwm_bb)} '
                f'cone={int(d.tx_q_hwm_cone)} '
                f'hand calls={calls} ok={calls - rej - down - pre1 - pre2 - traj} '
                f'rej={rej} busdown={down} '
                f'pre1={pre1} pre2={pre2} traj={traj}')

    def _bridge_fw_version_str(self) -> str:
        """Human/runbook rendering of the can-bridge's reported ``FW_VERSION``.

        Three distinct verdicts, never collapsed: ``unknown (never seen)`` (no
        BRIDGE_IDENTITY frame has arrived — either a bridge older than FW 9 or
        no bridge at all), the number plus the protocol version when it matches,
        and an explicit ``(expected vN)`` suffix when it does not. The skew is
        spelled out in the row and not only in the log, because the log line is
        announce-on-change and an operator joining mid-session would otherwise
        never see it.

        ``proto`` is rendered so the wire byte has a read site at all. It cannot
        detect protocol skew — a mismatch makes ``decode_frame`` reject every
        frame in both directions, including this one, so a value shown here is
        by construction a value that already agreed. What it documents is which
        protocol the running build was COMPILED against, which is the question
        the 24608bb stale-object flash (a v4 binary announcing itself as FW 8)
        left an operator unable to answer from the Jetson side.
        """
        with self._lock:
            bi = self._latest_bridge_identity
        if bi is None:
            return 'unknown (never seen)'
        version = int(bi.fw_version)
        expected = rpc_args.EXPECTED_BRIDGE_FW_VERSION
        if version == expected:
            return f'{version} (proto {int(bi.protocol_version)})'
        return (f'{version} (SKEW — expected v{expected}, '
                f'proto {int(bi.protocol_version)})')

    def _hand_traj_acks_str(self) -> str:
        """``/link_status`` rendering of the HAND_TRAJ_CMD ack tally.

        An instrument, like ``can3_errors``: read by differencing two captures
        (or by dividing fail by calls over one sitting), not by looking at a
        single value. It exists because 20d01e9 demoted the arm-ack failure to
        DEBUG, which removed the error code from every rosbag recorded since —
        see the counter init in ``__init__`` for the full argument.

        ``fail_teensy`` vs ``fail_host`` is the load-bearing split. The bridge
        answering with a non-OK status means the firmware ran ``hand_ops`` and
        one of its three CAN sends was refused; nothing coming back at all means
        the RPC request or its response was lost on the UDP link. Both render as
        ``HAND_TRAJ_CMD: ERR_TIMEOUT`` in the log, which is precisely why the
        2026-08-01 recount had to separate them by hand.

        Total by construction — three int reads and integer arithmetic, no
        parsing, no attribute chase. A renderer that raises takes the WHOLE
        ``/link_status`` message down with it, not just its own row. There is no
        never-seen sentinel because the counters exist from construction: before
        the first hand command the honest reading is ``calls=0``, and zero calls
        is not the same claim as zero failures out of many.
        """
        calls = self._hand_traj_calls
        fail_teensy = self._hand_traj_fail_teensy
        fail_host = self._hand_traj_fail_host
        return (f'calls={calls} ok={calls - fail_teensy - fail_host} '
                f'fail_teensy={fail_teensy} fail_host={fail_host}')

    # ═══════════════════════════════════════════════════════════
    # Setpoint downlink (Commit 3) — gated, default disabled
    # ═══════════════════════════════════════════════════════════

    def _set_mpc_active(self, active: bool):
        """Set the mpc_active state AND the J→T heartbeat flag together.

        This is the ONLY place mpc_active becomes 1. The heartbeat flag tells the
        Teensy the Jetson is driving setpoints (the firmware's guard-ENABLE
        precondition). Reached only through _arm_setpoint_output's stream-then-arm
        pre-check (ARMING_CONTRACT A1 — the boot-arm path is removed).
        """
        if active and not self._mpc_active:
            # Safety hardening: on the 0→1 re-enable edge, forget any stale
            # prior setpoint so the per-step gate starts fresh (the pump may hold a
            # _prev_pos from a prior session/pose). Same rationale as the reconnect
            # reset; the firmware MAX_DEVIATION gate is the complementary layer.
            self._sp_pump.reset()
        self._mpc_active = bool(active)
        self._client.set_heartbeat_flags(_FLAG_MPC_ACTIVE if active else 0)
        # Enable is the safety-relevant transition (warning); disable is benign.
        # Two DISTINCT call sites, not one line behind a severity-switching bound
        # method: Foxy's rcutils logger caches severity per source line and raises
        # 'Logger severity cannot be changed between calls' on a flip — which
        # killed the node mid-arm (STANDBY) on 2026-07-31, triggering the
        # safe-shutdown stow.
        if active:
            self.get_logger().warning(
                f"mpc_active set to {int(self._mpc_active)} — setpoint output ENABLED.")
        else:
            self.get_logger().info(
                f"mpc_active set to {int(self._mpc_active)} — setpoint output disabled.")

    def _start_setpoint_output(self, setpoint_source=None):
        """Bring up the setpoint source + ingest thread and set mpc_active=1.

        Called ONLY from _arm_setpoint_output after its preconditions pass
        (ARMING_CONTRACT A1; the zero-precondition __init__ boot-arm is removed).
        ``setpoint_source`` may be injected (tests); otherwise a real
        ``_MpcCommandSetpointSource`` is created — a ZMQ SUB on the **:5557**
        command stream (``mpccmd``), which trajectory_node publishes and the
        Teensy interpolates. It is NOT motor_guard's :5556 telemetry: that was
        the pre-cutover Jetson-relay path, and motor_guard has not been on the
        leg path since (see the ``_MpcCommandSetpointSource`` docstring above,
        and the launch file — motor_guard stopped launching 2026-08-01).
        """
        if setpoint_source is not None:
            self._sp_source = setpoint_source
        else:
            self._sp_source = _MpcCommandSetpointSource()
        # Set mpc_active BEFORE starting the thread so the first tick it drains
        # is not silently dropped by the mpc_active gate (and so the Teensy is
        # told we are driving before any setpoint can flow).
        self._set_mpc_active(True)
        self._sp_stop.clear()
        self._sp_thread = threading.Thread(
            target=self._setpoint_loop, name="teensy_bridge_setpoint", daemon=True)
        self._sp_thread.start()

    def _wait_wire_disarmed(self, timeout_s: float = 1.0) -> bool:
        """Wait until the FIRMWARE confirms disarmed (HeartbeatT2J bit3 drops).

        ``_set_mpc_active(False)`` only STAGES flags=0 — the 10 Hz heartbeat
        thread puts it on the wire on its NEXT tick (≤100 ms), and the firmware's
        own state comes back one T2J heartbeat later. Any action that must land
        disarmed (the A3 DEACTIVATE, the disarm-then-clear fallback) would race
        ``s_mpc_active=1`` on the Teensy without this wait — the review-caught
        staging race, same class as the bench harness's teardown-disarm race.
        Returns True once the firmware reports disarmed (or no heartbeat exists
        to consult); False on timeout.
        """
        deadline = time.monotonic() + timeout_s
        while True:
            with self._lock:
                hb = self._latest_heartbeat
            if hb is None or not (int(hb.flags) & _T2J_FLAG_MPC_ACTIVE):
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.02)

    def _stop_setpoint_output(self):
        """Stop the setpoint thread, drop mpc_active on the wire, close the source.

        The disarm half of the runtime arming service. Mirrors on_shutdown's
        setpoint teardown (thread first, then flags=0 on the wire), but leaves the
        node otherwise live (no stow, no transport teardown) so the operator can
        re-arm later in the same session.
        """
        self._sp_stop.set()
        if self._sp_thread is not None and self._sp_thread.is_alive():
            self._sp_thread.join(timeout=1.0)
        self._sp_thread = None
        try:
            self._set_mpc_active(False)
        except Exception:  # noqa: BLE001
            self._mpc_active = False
        if self._sp_source is not None:
            # Ownership: an injected setpoint source belongs to the test fixture
            # (same convention as the injected client — see the __init__ docstring
            # around client ownership), so disarm must NOT close it; only close a
            # source the node itself opened.
            if self._sp_source is not self._injected_setpoint_source:
                try:
                    self._sp_source.close()
                except Exception:  # noqa: BLE001
                    pass
            self._sp_source = None

    def _svc_set_setpoint_output(self, request, response):
        """SetBool: arm (true) / disarm (false) the 40 Hz setpoint downlink."""
        if bool(request.data):
            ok, msg = self._arm_setpoint_output()
            response.success = ok
            response.message = msg
        else:
            self._stop_setpoint_output()
            response.success = True
            response.message = 'setpoint output disabled (mpc_active=0)'
        return response

    def _arm_setpoint_output(self) -> tuple:
        """Run the arming preconditions; on all-pass, stream-then-arm.

        Returns ``(ok, message)``. The message carries the reject reason on
        failure (surfaced in the service response AND logged) so an operator sees
        exactly which precondition blocked the arm.

        Serialized by ``_arm_lock``: the service moved to the Reentrant group
        (so its 0.5 s frame-wait can't starve the telemetry timers), which
        removed the implicit mutual exclusion — two overlapping arms (a manual
        operator call racing an orchestrator auto-arm retry) would both pass
        the ``_mpc_active`` check and double-start the ingest thread.
        """
        if not self._arm_lock.acquire(blocking=False):
            return False, 'an arm attempt is already in progress — retry'
        try:
            return self._arm_setpoint_output_locked()
        finally:
            self._arm_lock.release()

    def _arm_setpoint_output_locked(self) -> tuple:
        if self._mpc_active:
            return True, 'setpoint output already enabled'

        # (a) Teensy link up + a fresh heartbeat (never arm onto a dead/silent link).
        age_us = self._link_age_us()
        if age_us is None or age_us > self._heartbeat_timeout_s * 1e6:
            return False, ('Teensy link down / no fresh heartbeat — refuse to arm '
                           '(never command a dead link)')
        if not self._link_latch.command_allowed():
            return False, 'Teensy link latched lost — refuse to arm'

        # (a2) The Teensy guard must not already be faulted. A latched E-STOP
        # silently DISCARDS leg output, so without this check the arm reports
        # "setpoint output ENABLED (armed)" onto a robot that cannot move — the
        # 40 Hz stream flows, setpoints_sent climbs, setpoints_rejected stays 0,
        # and nothing happens. That exact false-success cost a bench session on
        # 2026-07-09. The other preconditions cannot catch it: they check link
        # freshness, stream freshness, and u0-vs-encoder, none of which depend on
        # the guard latch.
        with self._lock:
            hb = self._latest_heartbeat
        if hb is not None and int(hb.fault_state) != int(FaultState.NONE):
            name = _enum_name(FaultState, int(hb.fault_state))
            return False, (
                f'Teensy guard fault latched (fault_state={name}) — refuse to arm; '
                f'leg output is suppressed until it is cleared. Recover with: '
                f'ros2 service call /clear_errors std_srvs/srv/Trigger')

        # (a3) No deactivate mid-flight. An arm landing during the descent would
        # hand the descending legs to the setpoint stream partway down (and the
        # early-descent u0 can still be within the 0.25 rev tolerance, so
        # precondition (c) alone does not cover it).
        if self._deactivate_in_progress:
            return False, ('deactivate in progress — refuse to arm mid-descent; '
                           'retry once the stow completes')

        # (b) a fresh mpccmd frame on :5557 within 0.5 s (is trajectory_node up?).
        source, created_here = self._acquire_setpoint_source()
        if source is None:
            return False, 'could not open the :5557 setpoint source'
        try:
            frame = None
            deadline = time.time() + 0.5
            while time.time() < deadline:
                frame = source.recv_latest()
                if frame is not None:
                    break
                time.sleep(0.02)
            if frame is None:
                if created_here:
                    source.close()
                return False, ('no mpccmd frame on :5557 within 0.5 s — is '
                               'trajectory_node running and streaming?')

            # Derive u0 exactly as the production pump would (rejects a malformed
            # frame, and guarantees the frame we arm on is itself pump-acceptable).
            # torque_ff_enabled is mirrored so the ACCEPTANCE decision matches the
            # production pump's — with the FF on, a malformed torque_Nm is a reject,
            # and we must not arm on a frame the production pump would refuse.
            probe = SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV, num_legs=p.NUM_LEGS,
                                 torque_ff_enabled=bool(hw.DYNAMICS_TORQUE_FF_ENABLED))
            sp, reason = probe.build(frame, 0)
            if sp is None:
                if created_here:
                    source.close()
                return False, (f'mpccmd frame not pump-acceptable: '
                               f'{reason or "no position command"}')
            u0 = sp.u0

            # (c) u0 within _ARM_U0_TOL_REV of every leg's live pos_estimate.
            with self._lock:
                telem = self._latest_telemetry
            if telem is None:
                if created_here:
                    source.close()
                return False, 'no telemetry yet — cannot verify u0 vs encoder'
            ok, why = self._u0_within_encoder_tol(u0, telem.pos_rev,
                                                  _ARM_U0_TOL_REV)
            if not ok:
                if created_here:
                    source.close()
                return False, (f'{why} — non-finite or mismatched encoder — refuse '
                               'to arm (would risk MAX_DEVIATION E-STOP)')
        except Exception as e:  # noqa: BLE001 — arming must never crash the node
            if created_here:
                try:
                    source.close()
                except Exception:  # noqa: BLE001
                    pass
            return False, f'arming check error: {e}'

        # TOCTOU re-check: the frame-wait above is up to 0.5 s — a deactivate
        # that STARTED during it would fire its TRAP_TRAJ believing the wire
        # disarmed, and arming now would land the stream mid-descent (the exact
        # hazard precondition (a3) exists to prevent).
        if self._deactivate_in_progress:
            if created_here:
                try:
                    source.close()
                except Exception:  # noqa: BLE001
                    pass
            return False, ('deactivate started during the arm pre-check — '
                           'refuse to arm mid-descent; retry once the stow '
                           'completes')

        # All preconditions met — stream-then-arm using the SAME source the check
        # observed (so the ingest thread continues from the confirmed stream).
        self._start_setpoint_output(source)
        return True, ('setpoint output ENABLED (armed): fresh stream confirmed, '
                      f'u0 within {_ARM_U0_TOL_REV} rev of every encoder')

    def _u0_within_encoder_tol(self, u0, pos_rev, tol) -> tuple:
        """Return ``(ok, reason)``: every leg's ``u0`` within ``tol`` rev of its
        encoder ``pos_rev``. NaN-safe — ``not (d <= tol)`` rejects a non-finite
        encoder (``d > tol`` would PASS a NaN, since every NaN comparison is False).
        Tolerance-agnostic, so it is shared by the arming pre-check (``_ARM_U0_TOL_REV``,
        0.25 rev) and /recover's convergence verify (``_RECOVER_U0_TOL_REV``, the tighter
        0.03 rev) — each passes its own ``tol`` (FIX 3)."""
        for i in range(p.NUM_LEGS):
            d = abs(float(u0[i]) - float(pos_rev[i]))
            if not (d <= tol):
                return False, (f'leg {i} u0 {float(u0[i]):.3f} rev vs encoder '
                               f'{float(pos_rev[i]):.3f} rev = {d:.3f} rev '
                               f'(> {tol} rev)')
        return True, ''

    def _verify_streamed_u0_converged(self, tol, timeout_s) -> tuple:
        """Poll until the streamed u0 is within ``tol`` rev of every live encoder, or
        ``timeout_s`` elapses. Returns ``(ok, reason)`` (FIX 3).

        This WAITS for trajectory_node's profiled descent to converge — u0 walks down
        onto the encoder over ~0.5-3 s (each 40 Hz knot inside the pump gate), so a
        single sample would race the still-descending command; the poll re-reads until
        it lands. Sources u0 from ``self._sp_pump._prev_pos`` — the exact LAST frame
        the bridge sent to the Teensy, which is what the firmware MAX_DEVIATION guard
        compares against — rather than re-reading the :5557 stream, so it never races
        the live setpoint-ingest thread during armed recovery. Reuses the same NaN-safe
        per-leg comparison as the arming pre-check (``_u0_within_encoder_tol``).
        """
        deadline = time.monotonic() + timeout_s
        last = 'no setpoint frame built yet'
        while True:
            u0 = self._sp_pump._prev_pos      # last accepted u0 (None until first send)
            with self._lock:
                telem = self._latest_telemetry
            if telem is None:
                last = 'no telemetry — cannot verify u0 vs encoder'
            elif u0 is None:
                last = 'no setpoint frame built yet (is the stream armed + flowing?)'
            else:
                ok, why = self._u0_within_encoder_tol(u0, telem.pos_rev, tol)
                if ok:
                    return True, ''
                last = why
            if time.monotonic() >= deadline:
                return False, last
            time.sleep(0.02)

    def _svc_recover(self, req, res):
        """``/recover`` (Trigger, FIX 3): one-call recovery from a latched Teensy guard.

        The 2026-07-10 stuttering-stroke runaway left the streamed u0 diverged
        ~0.5+ rev from the frozen encoder, so every bare /clear_errors re-latched
        MAX_DEVIATION within one 10 Hz fault tick. This does the recovery in the ONLY
        order that survives, WITHOUT disarming (mpc_active stays 1 — stopping the
        stream would trip MPC_STALE at 250 ms):
          1. ask trajectory_node to install a PROFILED DESCENT that walks the
             commanded u0 down onto the frozen encoder
             (``trajectory/reseed_from_measured``);
          2. WAIT (bounded) for that descent to converge — poll the pump's last-built
             u0 until it is within ``_RECOVER_U0_TOL_REV`` (0.03 rev) of every live
             encoder — so the clear cannot re-trip OR jolt at re-enable;
          3. if it plateaus above tol (the descent converged onto a STALE encoder
             snapshot while the leg drifted on during suppression — the Event-1
             −0.102 rev plateau), RE-INSTALL onto the now-current encoder and re-wait,
             up to ``_RECOVER_MAX_RESEED_ATTEMPTS`` times;
          4. only THEN fire CLEAR_ERRORS, and report precisely.
        If the reseed client is UNAVAILABLE (trajectory_node down), converge-first is
        IMPOSSIBLE — there is no node to install the descent, and with the setpoint
        source gone no fresh stream can jolt at re-enable — so it CLEARS DIRECTLY as an
        explicit escape hatch (with a loud WARN) rather than stranding the operator with
        a latched guard (F5). It still REFUSES (leaving the guard latched) when a live
        trajectory_node refuses/times out or the descent never converges — it NEVER
        clears onto a diverged command; those refusals state the manual disarm→clear
        recovery. Also the shared converge-first sequence the armed bare ``/clear_errors``
        reroutes through.
        """
        last_why = 'descent not yet attempted'
        for _attempt in range(_RECOVER_MAX_RESEED_ATTEMPTS):
            # 1. Ask trajectory_node to (re-)install a profiled descent onto the CURRENT
            #    measured encoder. Re-sampling each attempt is load-bearing: a descent
            #    onto a stale snapshot leaves u0 off the live (drifted) encoder; a fresh
            #    reseed chases the settling leg (see _RECOVER_MAX_RESEED_ATTEMPTS).
            if not self._reseed_client.service_is_ready():
                # trajectory_node (the setpoint source) is DOWN — converge-first cannot
                # help (nothing to install the descent, and no live stream to jolt at
                # re-enable). Refusing here would strand the operator with a latched guard
                # and no reachable converge path, so CLEAR DIRECTLY as the escape hatch,
                # with a loud WARN (F5). This is the raw-clear behaviour the pre-reroute
                # bare /clear_errors always had, now scoped to exactly the case where
                # converge-first is impossible.
                self.get_logger().warning(
                    'trajectory/reseed_from_measured UNAVAILABLE (trajectory_node down) '
                    '— converge-first impossible; clearing errors DIRECTLY (escape hatch)')
                cok, cmsg, _ = self.teensy_clear_errors()
                if cok:
                    # The ODrive integrator re-wound gravity while the guard was
                    # latched; restart the torque-FF ramp so the feedforward
                    # re-enters over the configured window instead of stepping
                    # (review 2026-07-14 — /recover keeps mpc_active=1, so
                    # neither reset() trigger fires on this path).
                    self._sp_pump.restart_torque_ramp()
                res.success = cok
                res.message = (
                    ('reseed unavailable (trajectory_node down) — cleared DIRECTLY '
                     f'(escape hatch): {cmsg}') if cok else
                    ('reseed unavailable AND direct CLEAR_ERRORS failed: '
                     f'{cmsg} — {_MANUAL_RECOVERY_HINT}'))
                return res
            try:
                future = self._reseed_client.call_async(Trigger.Request())
                deadline = time.monotonic() + _RECOVER_RESEED_TIMEOUT_S
                while not future.done() and time.monotonic() < deadline:
                    time.sleep(0.02)
                if not future.done():
                    res.success = False
                    res.message = ('trajectory_node reseed timed out '
                                   f'(> {_RECOVER_RESEED_TIMEOUT_S:.0f} s) — guard left '
                                   f'latched; {_MANUAL_RECOVERY_HINT}')
                    return res
                reseed = future.result()
            except Exception as e:  # noqa: BLE001 — recovery must never crash the node
                res.success = False
                res.message = (f'reseed call error: {e} — guard left latched; '
                               f'{_MANUAL_RECOVERY_HINT}')
                return res
            if not getattr(reseed, 'success', False):
                # A reseed refusal (stale telemetry / not streaming) will not be cured
                # by retrying — refuse now rather than burning the attempt budget.
                res.success = False
                res.message = (f'trajectory_node reseed refused: '
                               f'{getattr(reseed, "message", "unknown")} — guard left '
                               f'latched; {_MANUAL_RECOVERY_HINT}')
                return res

            # 2. WAIT for the profiled descent to walk u0 onto every LIVE encoder BEFORE
            #    clearing (it collapses over ~0.5-3 s, not in one frame).
            ok, last_why = self._verify_streamed_u0_converged(
                _RECOVER_U0_TOL_REV, self._recover_verify_timeout_s)
            if ok:
                break
            # 3. Plateaued off the live encoder — the leg drifted during the descent.
            #    Loop to re-descend onto the now-current encoder (bounded retries).
        else:
            res.success = False
            res.message = (
                f'reseed done but the profiled descent did not converge u0 onto the '
                f'encoder within {self._recover_verify_timeout_s:.1f}s × '
                f'{_RECOVER_MAX_RESEED_ATTEMPTS} attempts ({last_why}) — refusing to '
                f'clear (a clear now would re-latch); guard left latched; '
                f'{_MANUAL_RECOVERY_HINT}')
            return res

        # 4. Safe to clear now: u0 is within tol of every encoder.
        cok, cmsg, _ = self.teensy_clear_errors()
        if not cok:
            res.success = False
            res.message = f'reseed+converge OK but CLEAR_ERRORS failed: {cmsg}'
            return res
        # Restart the torque-FF ramp: the velocity integrator re-wound gravity
        # during the latch + firmware recovery slew (which zeroes cmd_tor), and
        # /recover keeps mpc_active=1 so neither of reset()'s triggers fires on
        # this path — without this the feedforward returns as a full-magnitude
        # single-tick step at slew hand-back (review 2026-07-14). The ramp
        # begins at the clear, so the residual step at hand-back is bounded by
        # ramp_scale(t_slew) x FF ~= (0.3 s / 2 s) x 0.04 Nm ~= 0.1 A — noise.
        self._sp_pump.restart_torque_ramp()
        res.success = True
        res.message = ('recovered: reseeded hold at measured, streamed u0 within '
                       f'{_RECOVER_U0_TOL_REV} rev of every encoder, CLEAR_ERRORS '
                       'fired')
        return res

    def _acquire_setpoint_source(self) -> tuple:
        """Return ``(source, created_here)`` for the arming check.

        Reuses an injected source (tests) or an already-open one; otherwise opens a
        fresh ``_MpcCommandSetpointSource`` (SUB on :5557), flagged so a failed arm
        closes it (no leak). A source opened here becomes the production ingest
        source only on a successful arm (``_start_setpoint_output`` adopts it).
        """
        if self._sp_source is not None:
            return self._sp_source, False
        if self._injected_setpoint_source is not None:
            return self._injected_setpoint_source, False
        try:
            return _MpcCommandSetpointSource(), True
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"could not open :5557 setpoint source: {e}")
            return None, False

    def _setpoint_loop(self):
        """Dedicated thread: drain the 40 Hz MPC command → pack Teensy-side knots → gate → send."""
        while not self._sp_stop.is_set():
            try:
                cmd = self._sp_source.recv_latest()
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"Setpoint source error: {e}",
                                        throttle_duration_sec=5.0)
                cmd = None
            if cmd is not None:
                self._process_setpoint(cmd)
            else:
                # ~1 kHz idle poll (the MPC command stream arrives at 40 Hz; the
                # Teensy interpolates to 500 Hz from the knots).
                self._sp_stop.wait(0.001)

    def _process_setpoint(self, cmd: dict):
        """Pack one 40 Hz MPC command into a Teensy-side knot Setpoint frame, gate it, send.

        SAFETY gates, in order:
          1. mpc_active must be set (operator opt-in). Belt-and-suspenders — the
             ingest thread only runs when enabled, but a direct caller is gated
             here too.
          2. Never command a dead link (the deferred-stow latch's command gate).
          3. The SetpointPump's per-step clamp + NaN/short-vector rejection.
        Only a clean, accepted frame is transmitted.

        FULLY EXCEPTION-CONTAINED (safety hardening): this runs on the dedicated
        'teensy_bridge_setpoint' daemon thread, so one malformed command or a
        transient send error (e.g. ENETUNREACH during the pre-latch window) must
        NOT kill the thread while mpc_active stays 1 — that would turn a one-frame
        problem into a restart-only outage of the production leg path with
        misleading telemetry. build() already REJECTS (not raises) malformed input;
        the send is OSError-guarded (mirroring the heartbeat thread); the outer
        guard is the backstop so no unexpected error escapes to the loop.
        """
        try:
            if not self._mpc_active:
                return
            if not self._link_latch.command_allowed():
                return  # never command a dead link
            t_origin_us = int(time.time() * 1_000_000)
            sp, reason = self._sp_pump.build(cmd, t_origin_us)
            if reason is not None:
                self.get_logger().error(
                    f"Setpoint REJECTED (not sent): {reason}",
                    throttle_duration_sec=1.0)
                return
            if sp is None:
                return  # feedback-only telemetry — nothing to send
            # Stash the ACCEPTED u0 for the leg_setpoint_echo GUI topic.
            # "Accepted" = passed the pump's validation + per-step gate (the
            # pump REJECTS, it never clamps — an accepted u0 is the exact
            # motor-rev vector packed into the frame). Stashed before the send
            # so a transient send error doesn't retract the echo (the link
            # watchdog owns real outages). HOT-PATH MINIMUM: three plain
            # assignments under the lock — message construction + publishing
            # happen on the executor timer (_publish_leg_setpoint_echo), never
            # on this thread.
            with self._lock:
                self._last_accepted_u0 = sp.u0
                self._last_accepted_u0_mono = time.monotonic()
                self._last_accepted_u0_seq += 1
            try:
                self._client.send_stream(int(MsgType.SETPOINT), sp.pack())
            except OSError as e:
                # Transient link error (ENETUNREACH/EHOSTUNREACH before the peer
                # is up, or a mid-run drop). The link watchdog owns the stow; here
                # we just drop this frame and keep the thread alive.
                self.get_logger().error(
                    f"Setpoint send failed (link down?): {e}",
                    throttle_duration_sec=1.0)
        except Exception as e:  # noqa: BLE001 — never let one frame kill the setpoint thread
            self.get_logger().error(
                f"Setpoint processing error (contained): {e}",
                throttle_duration_sec=1.0)

    def _guard_fault_leg_hint(self) -> str:
        """Best-effort ' (leg 3: active=[…] disarm=[…] 0x…/0x…)' culprit suffix.

        HeartbeatT2J.fault_state carries the guard REASON but no axis, so the
        edge log names the axis from the latest per-axis Diagnostic cache: an
        axis with a non-zero active error or disarm reason is the likely
        culprit for an ODRIVE_FATAL/MAX_DEVIATION/OVERSPEED latch. That scan is
        the primary path and is correct for ODRIVE_FATAL.

        F3 widened it twice. (a) It now DECODES both masks to names instead of
        printing a bare '(leg 3)' — the axis alone told an operator nothing
        about whether they were looking at an under-voltage, a spinout or a
        thermistor, and the names were already sitting unused in
        odrive.ERROR_CODES. (b) It scans EVERY axis present in _latest_diag,
        not range(_NUM_LEGS): a hand (6) or Ball Butler (7/8) fault produced a
        completely empty hint before, so the loudest line the bridge prints
        stayed silent about the one axis that was broken. Multiple culprits are
        joined with '; ' in axis order.

        Fallback: a MAX_DEVIATION latch trips on command↔encoder divergence, not
        an ODrive error, so active_errors/disarm_reason are frequently zero (all
        three 2026-07-16 S4 latches had active_errors==0 → the diag scan found
        nothing and the hint was silently empty). The firmware freezes the FIRST
        leg to cross in HeartbeatT2J.max_dev_leg (0xFF = none since boot) with the
        trip deviation in max_dev_value — that frozen snapshot is the ground-truth
        attribution, so name the leg from it when the diag scan is empty and the
        active fault is MAX_DEVIATION.

        Returns '' when no leg-specific cause is identifiable (e.g. a bus-level
        CAN_BUS_DOWN or an MPC-staleness fault) so the message stays honest rather
        than guessing a leg.
        """
        with self._lock:
            diag = dict(self._latest_diag)
            hb = self._latest_heartbeat
        culprits = []
        for axis in sorted(diag):
            d = diag[axis]
            active, disarm = int(d.active_errors), int(d.disarm_reason)
            if active != 0 or disarm != 0:
                culprits.append(f'{_axis_label(axis)}: '
                                f'{_decode_axis_errors(active, disarm)}')
        if culprits:
            return f' ({"; ".join(culprits)})'
        if (hb is not None
                and int(hb.fault_state) == int(FaultState.MAX_DEVIATION)
                and int(hb.max_dev_leg) != 0xFF):
            return (f' (leg {int(hb.max_dev_leg)}, '
                    f'dev={hb.max_dev_value:+.3f} rev at trip)')
        return ''

    def _log_odrive_errors(self):
        """F3/C2 — throttled per-axis ODrive error logging (can_node parity).

        Ported from ``can_node._handle_error`` (``7c7f61b^:can_node.py:419-447``),
        whose throttle scaffolding survived the cutover unused in
        ``can/motor_state.py``. Without it an ODrive error was visible ONLY as a
        boolean on /robot_state and a raw bitfield in the bag: an operator
        watching the launch shell got no line at all unless the Teensy guard
        also latched.

        WHICH TIMER. Called from the 10 Hz ``_publish_link_status``. Not the
        100 Hz ``_publish_robot_state`` (ten times the scan for no extra
        information — the throttle floor is 10 s), and not the 1 Hz
        ``_health_check`` (a fresh error would wait up to a second behind the
        link watchdog's own work). 10 Hz makes the first line effectively
        immediate while the scan itself is nine dict lookups.

        ONE LINE PER AXIS, NOT PER CODE. can_node emitted a separate line per
        set bit, which turns a bus-wide undervoltage into six near-identical
        lines. The message here is axis-level (``active=[…] disarm=[…]``), so a
        per-code line would just repeat itself. Throttling stays per
        (axis, code) exactly as before: a line is emitted when ANY currently-set
        code on that axis is outside its 10 s window, and every set code is then
        stamped — so a NEW code appearing mid-window still logs immediately,
        which is the property the per-code throttle exists for.

        Stamps are deliberately NOT pruned when a code clears. active_errors
        self-heals, so a flapping bit with pruning would log at the full 10 Hz;
        keeping the stamp costs one stale float and bounds the worst case at one
        line per 10 s per axis, which is what the throttle is for.
        """
        try:
            with self._lock:
                diag = dict(self._latest_diag)
            now = time.time()
            throttle = self._error_log.error_log_throttle_sec
            for axis in sorted(diag):
                d = diag[axis]
                active, disarm = int(d.active_errors), int(d.disarm_reason)
                if active == 0 and disarm == 0:
                    continue
                both = active | disarm
                # Throttle keys: every KNOWN set bit, plus one aggregate key for
                # the unknown residue (which can never collide with a known
                # single-bit code, since the residue excludes known bits). The
                # residue is keyed too so a future firmware error code is
                # throttled like any other rather than logging every 100 ms.
                known_mask = 0
                codes = []
                for code in odrive.ERROR_CODES:
                    known_mask |= code
                    if both & code:
                        codes.append(code)
                unknown = both & ~known_mask
                if unknown:
                    codes.append(unknown)
                log_times = self._error_log.last_error_log_times(axis)
                due = [c for c in codes if now - log_times.get(c, 0.0) > throttle]
                if not due:
                    continue
                for c in codes:
                    log_times[c] = now
                self.get_logger().error(
                    f'ODrive error on {_axis_label(axis)}: '
                    f'{_decode_axis_errors(active, disarm)}')
        except Exception as e:  # noqa: BLE001 — observability must never break the timer
            self.get_logger().error(f'ODrive error logging failed: {e}',
                                    throttle_duration_sec=10.0)

    # ═══════════════════════════════════════════════════════════
    # The alarmed command-latency monitor (2026-07-24 closure contract)
    # ═══════════════════════════════════════════════════════════

    def _latency_monitor_sample_clamp(self, hb, hb_gen):
        """Fold one ``lead_clamp_mask`` observation into the duty window.

        TWO GATES, both load-bearing:

        * **One sample per HEARTBEAT.** ``_latest_heartbeat`` is a latest-value
          latch, so a 10 Hz timer reading it after the link dies would resample
          the same frame forever — pinning the duty at whatever the last frame
          said. The generation counter makes "new frame" a fact rather than an
          assumption.
        * **Only while setpoints are STREAMING**, detected as the pump's frame
          counter advancing since the previous tick. The lead clamp acts on the
          leg interpolator's setpoint stream; with no stream there is nothing to
          clamp, and counting those samples as "not clamped" would dilute the
          duty of a short move to nothing. Measured off ``frames_built`` rather
          than ``mpc_active`` deliberately: the question is whether frames are
          actually going out, not whether an arm flag is set.

        Sampled at the heartbeat rate — the rate at which the firmware reports
        the mask at all, and the rate at which ``/link_status`` has always
        published it, which is the series the mask-any duty figures backing
        ``_CLAMP_DUTY_WARN`` were re-measured on.
        """
        now = time.monotonic()
        samples = self._lm_clamp_samples
        while samples and (now - samples[0][0]) > _CLAMP_DUTY_WINDOW_S:
            samples.popleft()
        sp_frames = self._sp_pump.frames_built
        streaming = (self._lm_last_sp_frames is not None
                     and sp_frames > self._lm_last_sp_frames)
        self._lm_last_sp_frames = sp_frames
        fresh_hb = (hb is not None and hb_gen != self._lm_last_hb_gen)
        if fresh_hb:
            self._lm_last_hb_gen = hb_gen
        if streaming and fresh_hb:
            samples.append((now, int(hb.lead_clamp_mask) != 0))

    def _clamp_duty(self):
        """Clamped fraction over the trailing window, or None if too few samples."""
        samples = self._lm_clamp_samples
        if len(samples) < _CLAMP_DUTY_MIN_SAMPLES:
            return None
        clamped = sum(1 for _t, c in samples if c)
        return clamped / float(len(samples))

    def _latency_monitor_step(self, hb, hb_gen) -> str:
        """Sample, evaluate, and LOG the command-latency monitor. Returns the token.

        The 2026-07-24 Addendum's second deliverable, and the only part of it
        that was missing: every input here was already measured, already
        levelled on a diagnostics topic and already bagged beside ``uptime_ms``
        — and therefore still invisible during a sitting, which is the exact
        failure class ("latency drift is invisible until a session is already
        degraded") the contract exists to close.

        Precedence is CAUSAL (see the LATENCY_MONITOR_* tokens): ring leak, then
        cache-age floor, then clamp duty. A stale input (its 1 Hz frame stopped
        arriving, or the board is too old to send it) is SKIPPED rather than
        held or zeroed — it cannot raise the alarm and it cannot silence a
        different condition either.

        EVERY INPUT IS LIVE, which precedence makes load-bearing: because the
        ring is ranked first, a ring term keyed on a since-boot maximum would
        not merely latch its own alarm, it would mask CACHE_AGE and CLAMP_DUTY
        for the rest of the session. ``_publish_ring_diag`` therefore feeds the
        SPOT leak plus a high-water only in the window it advances in, and
        "continuously-measured" in the contract is read as "recovery is
        observable", not just "sampled often". The blind spot that leaves is deliberate and
        bounded: a pre-FW-13 board sends no RING_DIAG at all, and on such a
        board the clamp-duty input — which needs no firmware support beyond the
        heartbeat — is the one that still fires.

        ADVISORY. This sets no DiagnosticStatus level, gates nothing and
        actuates nothing; it logs, and it returns a token for one KeyValue.
        """
        self._latency_monitor_sample_clamp(hb, hb_gen)
        now = time.monotonic()
        ring_fresh = (self._lm_ring_t > 0.0
                      and (now - self._lm_ring_t) <= _LATENCY_MONITOR_INPUT_STALE_S)
        cache_fresh = (self._lm_cache_t > 0.0
                       and (now - self._lm_cache_t) <= _LATENCY_MONITOR_INPUT_STALE_S)
        duty = self._clamp_duty()
        uptime = ('n/a' if hb is None else f'{int(hb.uptime_ms)}')

        if ring_fresh and self._lm_ring_leak > RING_LEAK_WARN_FRAMES:
            state = LATENCY_MONITOR_RING_LEAK
            detail = (f'CAN RX-ring leak {self._lm_ring_leak} frames '
                      f'(warns above {RING_LEAK_WARN_FRAMES}) — frames are '
                      f'being DELIVERED LATE; every consumer of bridge '
                      f'telemetry is that far behind')
        elif cache_fresh and self._lm_cache_floor_us >= CACHE_AGE_FLOOR_WARN_US:
            state = LATENCY_MONITOR_CACHE_AGE
            detail = (f'encoder-cache age floor '
                      f'{self._lm_cache_floor_us / 1000.0:.0f} ms '
                      f'(warns at {CACHE_AGE_FLOOR_WARN_US / 1000.0:.0f} ms) — '
                      f'the feedback the lead clamp measures against is stale')
        elif duty is not None and duty > _CLAMP_DUTY_WARN:
            state = LATENCY_MONITOR_CLAMP_DUTY
            detail = (f'lead-clamp duty {duty:.2f} over the last '
                      f'{_CLAMP_DUTY_WINDOW_S:.0f} s of streaming '
                      f'(warns above {_CLAMP_DUTY_WARN:.2f}; a healthy bridge '
                      f'measures 0.004-0.02, a leaking one 0.44) — commanded '
                      f'motion is being pinned to stale feedback')
        else:
            state = LATENCY_MONITOR_OK
            detail = ''

        # Rate-EXACT throttle off a monotonic stamp. ONE call site, fixed
        # severity (Foxy's rcutils logger caches severity per source line and
        # raises if a line changes it). The stamp is NOT reset when the
        # condition clears: a duty hovering at the threshold would otherwise
        # re-log on every 10 Hz tick that crossed it.
        #
        # An ESCALATION speaks immediately. Without that, CLAMP_DUTY followed by
        # RING_LEAK — the symptom, then the cause that explains it and changes
        # what the operator should do — could sit silent for 30 s behind a
        # throttle keyed on "some condition is active". Keyed on RANK rather
        # than on inequality so a flap between two conditions cannot use the
        # same door repeatedly.
        if state != LATENCY_MONITOR_OK:
            escalated = (_LATENCY_MONITOR_RANK[state]
                         > _LATENCY_MONITOR_RANK[self._lm_logged_state])
            if (self._lm_last_log_t is None or escalated
                    or (now - self._lm_last_log_t) >= _LATENCY_MONITOR_LOG_PERIOD_S):
                self._lm_last_log_t = now
                self._lm_logged_state = state
                self.get_logger().warning(
                    f'LATENCY MONITOR {state} (uptime_ms={uptime}): {detail}. '
                    f'ADVISORY — nothing is gated or suppressed; see '
                    f'/ring_diag, /cache_diag and /link_status lead_clamp_mask '
                    f'for the underlying numbers.')
        return state

    def _publish_link_status(self):
        """Publish link_status as a DiagnosticStatus.

        Surfaces the Teensy's reported link/fault/bus health AND the bridge's
        own view of link liveness (heartbeat age) plus — critically — the
        ``mpc_active`` flag we are sending, so an operator can confirm at a
        glance that setpoint output is disabled.
        """
        try:
            with self._lock:
                hb = self._latest_heartbeat
                hb_gen = self._heartbeat_gen
            age_us = self._link_age_us()
            # The alarmed latency monitor's one evaluation point (it needs the
            # 10 Hz heartbeat this method already holds). Runs BEFORE the level
            # is decided and never touches it — advisory, by contract.
            latency_monitor = self._latency_monitor_step(hb, hb_gen)
            # F3/C2 — throttled per-axis ODrive error decode onto the node log.
            # Rides this 10 Hz timer (see _log_odrive_errors for why this one)
            # and, like the latency monitor, is ADVISORY: it reads the diag
            # cache and writes only to the logger, never to msg.level.
            self._log_odrive_errors()

            msg = DiagnosticStatus()
            msg.name = 'teensy/link'
            msg.hardware_id = 'can_bridge_teensy'

            # Bridge-side liveness view (independent of the Teensy's self-report).
            if age_us is None:
                bridge_link = 'NO_HEARTBEAT'
            elif age_us > self._heartbeat_timeout_s * 1e6:
                bridge_link = 'LOST'
            else:
                bridge_link = 'UP'

            teensy_link = (_enum_name(LinkState, hb.link_state)
                           if hb is not None else 'UNKNOWN')
            teensy_fault = (_enum_name(FaultState, hb.fault_state)
                            if hb is not None else 'UNKNOWN')

            # Level: ERROR on lost link or any non-NONE Teensy fault; OK otherwise.
            fault_active = hb is not None and int(hb.fault_state) != int(FaultState.NONE)

            # Announce guard fault-state EDGES on the node log. Until this landed the
            # firmware could latch an E-STOP in total silence: /link_status carried
            # fault_state but was not recorded in the rosbag, and nothing was logged,
            # so an operator saw only "the robot stopped responding" and DEACTIVATE
            # coming back ERR_BUS_DOWN. Edge-triggered (not per-tick) so a latched
            # fault does not spam the 5 Hz timer.
            if hb is not None:
                fs = int(hb.fault_state)
                if fs != self._last_fault_state:
                    if fs != int(FaultState.NONE):
                        leg_hint = self._guard_fault_leg_hint()
                        # For a MAX_DEVIATION latch the single culprit leg is not
                        # the whole story — a fast coordinated move loads every
                        # leg, so the raw per-leg deviation vector gives the
                        # multi-leg context (which OTHER legs were near the limit)
                        # that the frozen single-leg snapshot cannot. Never
                        # re-threshold MAX_DEVIATION_REV here (hand-authored
                        # firmware constant, no generated mirror, changing today).
                        dev_ctx = ''
                        if fs == int(FaultState.MAX_DEVIATION):
                            dev_ctx = (' live_dev=['
                                       + ','.join(f'{d:+.2f}'
                                                  for d in hb.live_deviation)
                                       + '] rev')
                        self.get_logger().error(
                            f'Teensy guard FAULT LATCHED: fault_state={teensy_fault}'
                            f'{leg_hint}{dev_ctx} — leg output is now SUPPRESSED '
                            f'and every leg command (incl. DEACTIVATE, which '
                            f'returns ERR_BUS_DOWN) will be refused. Recover with: '
                            f'ros2 service call /clear_errors std_srvs/srv/Trigger')
                        # Anchor the persistent-reminder cadence to this edge so the
                        # first repeat lands _GUARD_LATCH_REPEAT_S later, not on the
                        # very next 10 Hz tick.
                        self._last_guard_latch_log_t = time.monotonic()
                    elif self._last_fault_state is not None:
                        self.get_logger().info(
                            'Teensy guard fault cleared (fault_state=NONE) — leg '
                            'output re-enabled.')
                        self._last_guard_latch_log_t = None
                    self._last_fault_state = fs

            # Persistent reminder while the latch PERSISTS: the edge log above fires
            # once, so without this a latched E-STOP goes silent after one line. Emit
            # a short unmissable ERROR every _GUARD_LATCH_REPEAT_S until CLEAR_ERRORS.
            # Rate-EXACT off a monotonic timestamp (never throttle_duration_sec).
            if fault_active:
                now = time.monotonic()
                if (self._last_guard_latch_log_t is None
                        or now - self._last_guard_latch_log_t >= _GUARD_LATCH_REPEAT_S):
                    self._last_guard_latch_log_t = now
                    leg_hint = self._guard_fault_leg_hint()
                    self.get_logger().error(
                        f'TEENSY GUARD LATCHED ({teensy_fault}){leg_hint} — leg '
                        f'output suppressed; CLEAR_ERRORS required')
            if bridge_link in ('LOST', 'NO_HEARTBEAT') or fault_active:
                msg.level = DiagnosticStatus.ERROR
                msg.message = f'link={bridge_link} fault={teensy_fault}'
            elif teensy_link == 'DEGRADED':
                msg.level = DiagnosticStatus.WARN
                msg.message = 'link degraded'
            else:
                msg.level = DiagnosticStatus.OK
                msg.message = 'OK'

            # Surface the bridge's own deferred-stow latch (Commit 2). When
            # bridge_stow_pending=1 after a reconnect, a mid-run link loss
            # happened and the platform needs stowing (no auto-stow RPC yet).
            if self._link_latch.stow_pending:
                msg.level = DiagnosticStatus.ERROR
                if msg.message == 'OK':
                    msg.message = 'stow pending (mid-run link loss)'

            stats = self._client.stats
            values = [
                KeyValue(key='bridge_link', value=bridge_link),
                KeyValue(key='teensy_link', value=teensy_link),
                KeyValue(key='fault_state', value=teensy_fault),
                KeyValue(key='mpc_active', value=str(int(self._mpc_active))),
                KeyValue(key='bridge_link_lost',
                         value=str(int(self._link_latch.link_lost))),
                KeyValue(key='bridge_stow_pending',
                         value=str(int(self._link_latch.stow_pending))),
                # cold-start cache visibility (the powered bring-up sitting
                # confirms the boot read landed and the right is_homed surfaced).
                KeyValue(key='cold_start_is_homed',
                         value=str(int(self._cold_start_state.is_homed))),
                KeyValue(key='cold_start_authoritative',
                         value=str(int(self._cold_start_authoritative))),
                # /robot_state publishes that were SKIPPED because the
                # telemetry latch had not moved since the previous one (see the
                # honesty gate in _publish_robot_state). Cumulative since node
                # start; the consumer differences two samples. Nominal is a
                # steady trickle — the 100 Hz publish timer and the ~100 Hz
                # TELEMETRY arrival beat against each other, so a few skips per
                # second is the two rates staying honest, not a fault. A count
                # that climbs toward the full 100/s means telemetry has slowed or
                # stopped, which /link_status's own staleness rows explain.
                KeyValue(key='robot_state_stale_skips',
                         value=str(int(self._robot_state_stale_skips))),
                # Platform-Teensy firmware identity. On THIS box `ros2 topic echo`
                # gives false negatives on high-rate RELIABLE topics
                # (reference_ros2_topic_echo_flaky_foxy), and robot_state runs at
                # 100 Hz — so link_status at 10 Hz is the surface a bench operator
                # can actually read, and it is the one the runbook cites.
                KeyValue(key='platform_fw_version',
                         value=self._platform_fw_version_str()),
                # ODrive firmware per axis — distinct from the Platform-Teensy row
                # above. The firmware_validated bool only asserts group consistency,
                # so the actual triples (plus Get_Version's fourth byte) are the only
                # bench evidence of what the drives are running.
                KeyValue(key='odrive_fw_versions',
                         value=self._odrive_fw_versions_str()),
                # Hand ball-present sensor: tri-state word + miss count + the
                # RAW get_gpio_states word in hex. Rendered unconditionally
                # (never-seen reads 'unknown (never seen)') so an unflashed or
                # dead bridge is visible here rather than absent.
                KeyValue(key='hand_ball_sensor',
                         value=self._hand_ball_sensor_str()),
                # CAN3 wire-error instrument (bridge FW 5+). Renders
                # 'unknown (never seen)' against an older bridge rather than
                # vanishing, so a missing row never reads as a clean bus.
                KeyValue(key='can3_errors',
                         value=self._can3_errors_str()),
                # Hand-arm ack tally (host-side; works against ANY bridge, no
                # firmware support needed). 20d01e9 demoted the arm-ack failure
                # to DEBUG, so since 2026-07-24 a sitting's bag has carried no
                # trace of it — and the bag is what survives the session. Split
                # fail_teensy (the bridge answered and refused) from fail_host
                # (nothing came back) because the log text pools them.
                KeyValue(key='hand_traj_acks',
                         value=self._hand_traj_acks_str()),
                # Bridge TX-path pressure + the firmware-side half of the
                # hand-ack story (bridge FW 9+). hand_traj_acks above counts
                # what the HOST saw; this counts what the BRIDGE did, and only
                # this one can say WHICH of hand_ops' three sends refused.
                # Renders 'unknown (never seen)' against an older bridge rather
                # than vanishing, so a missing row never reads as no pressure.
                KeyValue(key='bridge_tx_diag',
                         value=self._bridge_tx_diag_str()),
                # Can-bridge firmware identity, beside platform_fw_version
                # above — three Teensys share this bench and until now only one
                # of them said which build it was running. Warn-never-refuse:
                # a skew shows up here and in a BRIDGE_FW_CHECK log line, and
                # is never enforced.
                KeyValue(key='bridge_fw_version',
                         value=self._bridge_fw_version_str()),
                # The HOST half of the same currency question, deliberately
                # adjacent to the firmware half: '0' clean, '1' this node is
                # running a stale colcon install, 'unknown' the check could not
                # run. A bare token so a bag consumer can compare it exactly.
                #
                # WHY A PERSISTENT ROW AND NOT ONLY THE STARTUP LOG. The verdict
                # is logged once, at construction; /rosout recording races node
                # startup, and an operator (or an analysis script) joining a
                # bag mid-session would otherwise have no way to tell a fresh
                # node from the stale one that produced the 2026-08-14 S3 bag.
                # Same argument as bridge_fw_version's above.
                KeyValue(key='install_skew', value=self._install_skew),
                # The token alone cannot distinguish "source absent" from
                # "check errored", and on a '1' it cannot say WHICH side is
                # behind — which is the difference between rebuilding and
                # updating the checkout. The detail is static per process, so
                # it costs one string per publish and makes the bag
                # self-explanatory without the log.
                KeyValue(key='install_skew_detail',
                         value=self._install_skew_detail),
                # THE LATENCY MONITOR'S SUMMARY (2026-07-24 contract): the
                # highest active condition, 'OK' when clean. One token, in the
                # same message as uptime_ms below, so a bag or a GUI has a
                # single field to trend against bridge uptime instead of having
                # to join three diagnostics topics to notice a degrading
                # session. Published UNCONDITIONALLY (unlike uptime_ms, which
                # needs a heartbeat) — the monitor's clamp-duty input is exactly
                # the one that matters when heartbeats are patchy, and a row
                # that vanishes reads as no data, not as OK.
                #
                # The individual numbers behind the token are deliberately NOT
                # repeated here: leak, cache floor and lead_clamp_mask are all
                # already bagged (on /ring_diag, /cache_diag and this very
                # message), so the duty is reconstructible offline from the
                # sampling rule in _latency_monitor_sample_clamp.
                #
                # ADVISORY: it never touches msg.level. /link_status's level is
                # the link/fault channel the orchestrator reads, and a latency
                # advisory must not be able to colour it.
                KeyValue(key='latency_monitor', value=latency_monitor),
                # How the wall-clock ANCHOR the Teensy time-syncs to was stamped
                # (bridge-temporal-trustworthiness P2). 'kernel-midpoint' =
                # (kernel RX t2 + pre-send userspace t3)/2, which cancels this
                # node's processing delay out of the firmware's stamp + rtt/2.
                # 'userspace' = the pre-P2 t3-only stamp, i.e. the anchor carries
                # a +processing/2 bias again — a fallback that MUST be visible in
                # the bag rather than inferred. 'unknown' until the Teensy's
                # first TIME_OF_DAY_QUERY (boot, then every ~30 s).
                KeyValue(key='tod_stamp_mode',
                         value=self._tod.stamp_mode),
                # stamp_mode reports only the LAST query; a guard that fires
                # intermittently (e.g. NTP slew) leaves the mode reading
                # 'kernel-midpoint' while some anchors were userspace-stamped —
                # this count is the only artefact that reveals the flapping.
                KeyValue(key='tod_implausible_rx_stamps',
                         value=str(self._tod.implausible_rx_stamps)),
                KeyValue(key='setpoints_sent',
                         value=str(self._sp_pump.frames_built)),
                KeyValue(key='setpoints_rejected',
                         value=str(self._sp_pump.frames_rejected)),
                # Leg torque FF (ships ON since 2026-07-16). torque_ff_ramp is the live 0→1 ramp
                # multiplier — the operator watches this climb during the first arming
                # (tests/hardware/session_torque_ff.md, step S3). It reads 0 whenever
                # the feature is off, so a 0 here on an armed robot means "no FF", full
                # stop, with no ambiguity.
                KeyValue(key='torque_ff_enabled',
                         value=str(int(self._sp_pump.torque_ff_enabled))),
                KeyValue(key='torque_ff_ramp',
                         value=f'{self._sp_pump.torque_ff_ramp_scale():.3f}'),
                KeyValue(key='setpoints_without_ff',
                         value=str(self._sp_pump.frames_without_ff)),
                KeyValue(key='heartbeat_age_ms',
                         value=('n/a' if age_us is None else f'{age_us / 1000.0:.0f}')),
                KeyValue(key='rx_frames', value=str(stats.rx_frames)),
                KeyValue(key='tx_frames', value=str(stats.tx_frames)),
                KeyValue(key='crc_errors', value=str(stats.crc_errors)),
                KeyValue(key='decode_errors', value=str(stats.decode_errors)),
            ]
            if hb is not None:
                values += [
                    KeyValue(key='bus1_health',
                             value=_enum_name(BusHealth, hb.bus1_health)),
                    KeyValue(key='bus2_health',
                             value=_enum_name(BusHealth, hb.bus2_health)),
                    # Cone (CAN2) health rides HeartbeatT2J.flags bits 4-5; an
                    # old flash leaves them 0 = UNKNOWN (BusHealth's zero value).
                    KeyValue(key='bus3_health',
                             value=_enum_name(BusHealth,
                                              (int(hb.flags) & _T2J_CONE_HEALTH_MASK)
                                              >> _T2J_CONE_HEALTH_SHIFT)),
                    KeyValue(key='uptime_ms', value=str(int(hb.uptime_ms))),
                    KeyValue(key='time_synced',
                             value=str(int(bool(hb.flags & _T2J_FLAG_TIME_SYNCED)))),
                    KeyValue(key='teensy_stow_pending',
                             value=str(int(bool(hb.flags & _T2J_FLAG_STOW_PENDING)))),
                    # A5 — the firmware's own arm state (bit3). Normally mirrors
                    # the host 'mpc_active' KeyValue one heartbeat late; a
                    # persistent split means the arm/disarm never took on the wire.
                    KeyValue(key='teensy_mpc_active',
                             value=str(int(bool(hb.flags & _T2J_FLAG_MPC_ACTIVE)))),
                    # Leg guard-deviation diagnostics (2026-07-10 forensics). Per-leg
                    # live deviation (u0-encoder, the MAX_DEVIATION guard quantity) +
                    # the lead-clamp bitmask, plus the frozen latch-event snapshot
                    # (which leg crossed + dev/u0/encoder at the trip). Recorded on
                    # /link_status so a future stutter/latch bag is self-diagnosing.
                    KeyValue(key='lead_clamp_mask',
                             value=str(int(hb.lead_clamp_mask))),
                    # Per-leg torque_ff ingest-clamp mask (firmware backstop,
                    # 2026-07-14): bit i = leg i's |torque_ff| was clamped to
                    # TORQUE_FF_FIRMWARE_CLAMP_WIRE_NM at UDP ingest on the last
                    # accepted setpoint. Rides HeartbeatT2J.flags bits 8-13 (no
                    # wire-size change); 0 from a pre-clamp firmware. Nonzero
                    # means a producer exceeded the 0.25 wire-Nm backstop — the
                    # pump clamp (0.1451 wire-Nm at the 2026-07-15 scale) should always bind first, so
                    # any nonzero value here is a torque-path bug to chase.
                    KeyValue(key='torque_clamp_mask',
                             value=str((int(hb.flags) & _T2J_TORQUE_CLAMP_MASK)
                                       >> _T2J_TORQUE_CLAMP_SHIFT)),
                    KeyValue(key='live_deviation',
                             value=','.join(f'{d:.4f}' for d in hb.live_deviation)),
                    KeyValue(key='max_dev_leg',
                             value=('none' if int(hb.max_dev_leg) == 0xFF
                                    else str(int(hb.max_dev_leg)))),
                    KeyValue(key='max_dev_value', value=f'{hb.max_dev_value:.4f}'),
                    KeyValue(key='max_dev_u0', value=f'{hb.max_dev_u0:.4f}'),
                    KeyValue(key='max_dev_enc', value=f'{hb.max_dev_enc:.4f}'),
                    # Direct culprit-leg field for rosbags/GUI: the frozen
                    # max_dev_leg snapshot BUT gated on an ACTIVE MAX_DEVIATION
                    # latch, so downstream consumers don't have to re-derive the
                    # gating. max_dev_leg alone persists after /clear_errors (it
                    # is 'last latch since boot'), which would read as a live
                    # culprit forever; this key is '' unless a MAX_DEVIATION latch
                    # is currently held.
                    KeyValue(key='guard_fault_leg',
                             value=(str(int(hb.max_dev_leg))
                                    if (int(hb.fault_state)
                                        == int(FaultState.MAX_DEVIATION)
                                        and int(hb.max_dev_leg) != 0xFF)
                                    else '')),
                ]
            msg.values = values
            self.link_status_pub.publish(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Link status publish error: {e}",
                                    throttle_duration_sec=5.0)

    def _publish_bb_odrive(self):
        """Publish /bb/odrive_diag: BB pitch(7)/hand(8) ODrive temps, current, state.

        The can-bridge sends these on CAN1 as DIAGNOSTIC frames with axis_id 7/8,
        which _on_diagnostic stashes in self._latest_diag. Flat-array layout (8
        floats): [pitch_fet, pitch_motor, pitch_iq_meas, pitch_state,
                  hand_fet,  hand_motor,  hand_iq_meas,  hand_state]. Suppressed
        until at least one BB ODrive frame has arrived (avoids phantom zeros).
        """
        with self._lock:
            dp = self._latest_diag.get(7)
            dh = self._latest_diag.get(8)
        if dp is None and dh is None:
            return
        def quad(d):
            if d is None:
                return [float('nan')] * 4
            return [float(d.temp_fet), float(d.temp_motor),
                    float(d.iq_measured), float(d.axis_state)]
        msg = Float32MultiArray()
        msg.data = quad(dp) + quad(dh)
        self.bb_odrive_pub.publish(msg)

    def _publish_profile(self):
        """Publish profile (firmware instrumentation) as DiagnosticStatus."""
        try:
            with self._lock:
                pr = self._latest_profile
            if pr is None:
                return
            msg = DiagnosticStatus()
            msg.name = 'teensy/profile'
            msg.hardware_id = 'can_bridge_teensy'
            # interp deadline misses are the headline health signal.
            msg.level = (DiagnosticStatus.WARN
                         if pr.interp_deadline_misses > 0
                         else DiagnosticStatus.OK)
            msg.message = (f'interp_misses={pr.interp_deadline_misses} '
                           f'heap={pr.free_heap_bytes}B')
            msg.values = [
                KeyValue(key='free_heap_bytes', value=str(pr.free_heap_bytes)),
                KeyValue(key='interp_deadline_misses',
                         value=str(pr.interp_deadline_misses)),
                KeyValue(key='interp_max_jitter_us',
                         value=str(pr.interp_max_jitter_us)),
                KeyValue(key='udp_rtt_us', value=str(pr.udp_rtt_us)),
                KeyValue(key='udp_jitter_us', value=str(pr.udp_jitter_us)),
                KeyValue(key='can1_util_pct', value=f'{pr.can1_util_x100 / 100.0:.1f}'),
                KeyValue(key='can2_util_pct', value=f'{pr.can2_util_x100 / 100.0:.1f}'),
                KeyValue(key='can3_util_pct', value=f'{pr.can3_util_x100 / 100.0:.1f}'),
                KeyValue(key='can1_rx', value=str(pr.can1_rx)),
                KeyValue(key='can1_tx', value=str(pr.can1_tx)),
                KeyValue(key='can2_rx', value=str(pr.can2_rx)),
                KeyValue(key='can2_tx', value=str(pr.can2_tx)),
                KeyValue(key='can3_rx', value=str(pr.can3_rx)),
                KeyValue(key='can3_tx', value=str(pr.can3_tx)),
                KeyValue(key='cpu_pct_x100',
                         value=','.join(str(c) for c in pr.cpu_pct_x100)),
            ]
            self.profile_pub.publish(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Profile publish error: {e}",
                                    throttle_duration_sec=5.0)

    # ═══════════════════════════════════════════════════════════
    # RPC service surface (Commit 4)
    # ═══════════════════════════════════════════════════════════

    def _call_rpc(self, method, args=b"", *, timeout=None, retries=None):
        """Issue an RPC; return (success, message, result_blob).

        Blocks on the calling thread until the response arrives (decoded on the
        RX thread) or the timeout × retries budget expires. RpcError/RpcTimeout
        are caught and reported as (False, message).

        This is the single choke point every outbound RPC passes through, which
        is why the HAND_TRAJ_CMD ack tally lives here rather than in
        ``teensy_hand_traj_cmd`` (two service handlers reach that method) or in
        ``catch_coordinator_node`` (which publishes no ``/link_status``).
        """
        hand_traj = int(method) == int(RpcMethod.HAND_TRAJ_CMD)
        if hand_traj:
            # HAND_TRAJ_CMD is in NON_IDEMPOTENT_METHODS (teensy_link/rpc.py),
            # so retries is forced to 0: one counted call is exactly one wire
            # attempt and one counted failure is exactly one failed wire attempt,
            # with no silent re-dispatch underneath inflating either number.
            self._hand_traj_calls += 1
        try:
            result = self._rpc.call(int(method), args,
                                    timeout=timeout, retries=retries)
            return True, 'OK', result
        except RpcError as e:
            if hand_traj:
                # RpcTimeout subclasses RpcError and carries ERR_TIMEOUT, so both
                # branches render the identical 'HAND_TRAJ_CMD: ERR_TIMEOUT'
                # string — isinstance is the ONLY discriminator between "the
                # bridge answered and refused" (fail_teensy: hand_ops ran, a
                # can_jugglebot_send returned false) and "nothing came back at
                # all" (fail_host: the request or the response was lost). Those
                # two point at completely different halves of the system.
                if isinstance(e, RpcTimeout):
                    self._hand_traj_fail_host += 1
                else:
                    self._hand_traj_fail_teensy += 1
            return False, self._annotate_rpc_error(str(e)), b""

    def _annotate_rpc_error(self, message: str) -> str:
        """Disambiguate ERR_BUS_DOWN, which the firmware overloads three ways.

        ``leg_deactivate.cpp:deactivate_allowed`` (and the activate/relay/hand
        analogues) return ERR_BUS_DOWN for a WARN/BUS_OFF bus, for
        ``fault_can_bus_down()``, AND for ``fault_guard_mode() == ESTOP`` — a
        latched guard on a perfectly healthy bus. The bare status therefore sends
        an operator hunting a CAN fault that does not exist (2026-07-09: a
        MAX_DEVIATION E-STOP surfaced only as "DEACTIVATE rejected:
        ERR_BUS_DOWN"). Append the live fault_state so the real cause is named at
        the point of failure.
        """
        if 'ERR_BUS_DOWN' not in message:
            return message
        with self._lock:
            hb = self._latest_heartbeat
        if hb is None:
            return message
        fs = int(hb.fault_state)
        if fs != int(FaultState.NONE):
            return (f'{message} — NOTE: ERR_BUS_DOWN also covers a latched guard '
                    f'E-STOP, and fault_state={_enum_name(FaultState, fs)} is '
                    f'currently latched. The bus is likely fine. Clear it with: '
                    f'ros2 service call /clear_errors std_srvs/srv/Trigger')
        return (f'{message} — guard is not faulted (fault_state=NONE), so this is a '
                f'genuine bus condition (CAN3 WARN/BUS_OFF or down)')

    # ── Tested node methods (one per RpcMethod) — the reusable surface ──
    # Arg encoding (rpc_args, codegen-hoisted) + RpcClient call. ROS service
    # wrappers for the arg-bearing per-axis ops await new .srv types; the
    # encodings are fully covered by tests/ros/test_teensy_bridge_node_rpc.py.

    def teensy_set_axis_state(self, axis, state):
        return self._call_rpc(RpcMethod.SET_AXIS_STATE,
                              rpc_args.encode_set_axis_state(axis, state))

    def teensy_set_controller_mode(self, axis, ctrl, input_mode):
        return self._call_rpc(RpcMethod.SET_CONTROLLER_MODE,
                              rpc_args.encode_set_controller_mode(axis, ctrl, input_mode))

    def teensy_set_vel_curr_limits(self, axis, vel_limit, curr_limit, **kw):
        return self._call_rpc(RpcMethod.SET_VEL_CURR_LIMITS,
                              rpc_args.encode_set_vel_curr_limits(axis, vel_limit, curr_limit),
                              **kw)

    def teensy_set_pos_gain(self, axis, pos_gain):
        return self._call_rpc(RpcMethod.SET_POS_GAIN,
                              rpc_args.encode_set_pos_gain(axis, pos_gain))

    def teensy_set_vel_gains(self, axis, vel_gain, vel_int_gain):
        return self._call_rpc(RpcMethod.SET_VEL_GAINS,
                              rpc_args.encode_set_vel_gains(axis, vel_gain, vel_int_gain))

    def teensy_set_absolute_position(self, axis, position):
        return self._call_rpc(RpcMethod.SET_ABSOLUTE_POSITION,
                              rpc_args.encode_set_absolute_position(axis, position))

    def teensy_clear_errors(self, axis=rpc_args.AXIS_ALL):
        return self._call_rpc(RpcMethod.CLEAR_ERRORS,
                              rpc_args.encode_clear_errors(axis))

    def teensy_reboot(self, axis=rpc_args.AXIS_ALL):
        return self._call_rpc(RpcMethod.REBOOT_ODRIVES,
                              rpc_args.encode_reboot(axis))

    def teensy_encoder_search(self, axis=rpc_args.AXIS_ALL):
        return self._call_rpc(RpcMethod.ENCODER_SEARCH,
                              rpc_args.encode_encoder_search(axis))

    def teensy_home(self, axis=rpc_args.AXIS_ALL):
        return self._call_rpc(RpcMethod.HOME, rpc_args.encode_home(axis))

    def teensy_activate(self, axis=rpc_args.AXIS_ALL):
        return self._call_rpc(RpcMethod.ACTIVATE, rpc_args.encode_activate(axis))

    def teensy_deactivate(self, axis=rpc_args.AXIS_ALL):
        return self._call_rpc(RpcMethod.DEACTIVATE, rpc_args.encode_deactivate(axis))

    def teensy_hand_traj_cmd(self, args: bytes):
        """HAND_TRAJ_CMD: forward a pre-built 8-byte 0x6D0 payload (from
        rpc_args.encode_hand_traj_cmd / encode_smooth_move_hand). The firmware sends
        the CLOSED_LOOP + POSITION/PASSTHROUGH preamble then forwards it on the
        firmware-owned 0x6D0 id (aborting if a preamble send fails)."""
        return self._call_rpc(RpcMethod.HAND_TRAJ_CMD, args)

    def teensy_sdo_read(self, axis, endpoint):
        """SDO_READ — FIRE-AND-FORGET. The returned blob is ALWAYS empty: no SDO
        reply reaches the Jetson, and axis 6 is rejected outright. Not a way to
        read a register back — see rpc_args.encode_sdo_read for why."""
        return self._call_rpc(RpcMethod.SDO_READ,
                              rpc_args.encode_sdo_read(axis, endpoint))

    def teensy_sdo_write(self, axis, endpoint, value):
        return self._call_rpc(RpcMethod.SDO_WRITE,
                              rpc_args.encode_sdo_write(axis, endpoint, value))

    # ── Platform-Teensy relay ──
    # The relay reads TRIGGER a Platform-Teensy reply over CAN3 (0x7DE tilt /
    # 0x6E0 RobotState); the firmware forwards the reply verbatim as a
    # PLATFORM_FRAME the bridge correlates by (can_id, dlc). These methods are the
    # tested mechanism; the cold-start cache + orchestrator conduit source robot_state.is_homed/levelling/pose +
    # get_platform_tilt from them. NOTE(bench): the (can_id, dlc) discriminator is
    # only sound if CAN3 SRX_DIS is set so the bridge's own 0x6E0 STATE_WRITE is
    # not looped back as a reply, and the await timeout must exceed the measured
    # Platform reply latency — both gated on a bench probe before hardware trust.
    _RELAY_READ_TIMEOUT_S = 0.5

    # Cold-start boot read: a few bounded retries so a momentarily
    # not-yet-synced CAN3 / link gets a second chance before the conservative
    # is_homed=False fallback. Worst-case construction delay when the Platform is
    # unresponsive ≈ 3 × _RELAY_READ_TIMEOUT_S + 2 × _BOOT_STATE_READ_RETRY_S ≈
    # 1.9 s (each attempt's STATE_READ is ACKed fast but no PLATFORM_FRAME reply
    # arrives, so the await runs to timeout), and more if the UDP link itself is
    # down (each attempt then also hits the RpcClient timeout × retries). This runs
    # synchronously in __init__ before the executor spins; the bound is acceptable
    # for a deliberate cold-start step (the legs are not yet moving).
    _BOOT_STATE_READ_ATTEMPTS = 3
    _BOOT_STATE_READ_RETRY_S = 0.2
    # Reboot cold-start clear: the Platform Teensy stays powered through
    # an ODrive reboot, so a dropped clear would leave a STALE is_homed=True against
    # rebooted (de-referenced) ODrives — the dangerous direction. Retry the clear a
    # few times to harden that durability.
    _REBOOT_CLEAR_ATTEMPTS = 3
    _REBOOT_CLEAR_RETRY_S = 0.2

    def _await_platform_reply(self, can_id, expected_dlc, timeout):
        """Block (calling thread) for a fresh PLATFORM_FRAME on ``can_id`` with the
        expected dlc. Returns the raw reply bytes, or None on timeout. The caller
        clears the latch BEFORE sending the trigger RPC so a stale reply can't
        satisfy a new read."""
        deadline = time.monotonic() + timeout
        with self._platform_reply_cv:
            while True:
                entry = self._platform_replies.get(int(can_id))
                if entry is not None and entry[1] == expected_dlc:
                    return entry[0]
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._platform_reply_cv.wait(remaining)

    def relay_read_tilt(self, timeout=None):
        """TILT_READ: read the Platform-Teensy inclinometer. Returns
        (ok, message, (tiltX, tiltY)) in radians; tilt is None on failure."""
        timeout = self._RELAY_READ_TIMEOUT_S if timeout is None else timeout
        can_id = proto.CAN_ID_PLATFORM_TILT_READING
        with self._relay_lock:   # serialize the whole relay round-trip
            with self._platform_reply_cv:
                self._platform_replies.pop(int(can_id), None)
            ok, msg, _ = self._call_rpc(RpcMethod.TILT_READ)
            if not ok:
                return False, msg, None        # fail-fast (e.g. ERR_BUS_DOWN)
            data = self._await_platform_reply(can_id, expected_dlc=8, timeout=timeout)
            if data is None:
                return False, 'relay tilt read: no Platform reply within timeout', None
            tiltX, tiltY = struct.unpack('<ff', data[:8])
            return True, 'OK', (tiltX, tiltY)

    def relay_read_robot_state(self, timeout=None):
        """STATE_READ: read the Platform-Teensy RobotState. Returns
        (ok, message, RelayRobotState | None). Decodes the 0x6E0 reply exactly as
        Teensy_code_platform.ino decodeStateCANMessage packs it."""
        timeout = self._RELAY_READ_TIMEOUT_S if timeout is None else timeout
        can_id = proto.CAN_ID_PLATFORM_STATE_UPDATE
        with self._relay_lock:   # serialize the whole relay round-trip
            with self._platform_reply_cv:
                self._platform_replies.pop(int(can_id), None)
            ok, msg, _ = self._call_rpc(RpcMethod.STATE_READ)
            if not ok:
                return False, msg, None        # fail-fast (e.g. ERR_BUS_DOWN)
            data = self._await_platform_reply(can_id, expected_dlc=8, timeout=timeout)
            if data is None:
                return False, 'relay state read: no Platform reply within timeout', None
            self._record_platform_fw_version(data)
            return True, 'OK', _decode_relay_robot_state(data)

    # ── Platform-Teensy firmware-identity check ──
    # THE single capture + verdict point for the Platform Teensy's FW_VERSION.
    # It sits in relay_read_robot_state (the only place holding the raw 0x6E0
    # bytes) rather than in _refresh_cold_start_state, so EVERY successful state
    # read refreshes it — a future second caller cannot bypass the check by not
    # going through the cache layer. Same reasoning as the stroke-window gate
    # living inside _arm_hand_catch rather than at its call site.

    def _record_platform_fw_version(self, data: bytes):
        """Capture the Platform Teensy's FW_VERSION from a 0x6E0 reply and log the
        verdict. Called with _relay_lock held (lock order _relay_lock → _lock,
        same as _write_is_homed)."""
        version = rpc_args.decode_platform_fw_version(data)
        with self._lock:
            previous = self._platform_fw_version
            self._platform_fw_version = version
        expected = rpc_args.PLATFORM_FW_VERSION_EXPECTED
        if version == expected:
            # Only announce OK on a CHANGE (incl. the first read), so a reconnect
            # storm cannot spam the log with good news.
            if previous != version:
                self.get_logger().info(
                    f'PLATFORM_FW_CHECK: OK — Platform Teensy reports v{version} '
                    f'(expected v{expected})')
            return
        # Skew. ERROR level, one greppable token, and the un-versioned case named
        # explicitly because that is the un-flashed-board signature.
        if version == rpc_args.PLATFORM_FW_VERSION_UNVERSIONED:
            detail = ('PRE-VERSIONING firmware (no FW_VERSION) — this board has '
                      'NOT been flashed since 2026-07-27')
        else:
            detail = f'v{version}'
        self.get_logger().error(
            f'PLATFORM_FW_CHECK: FAIL — Platform Teensy reports {detail}, host '
            f'tree expects v{expected}. Hand commands are NOT refused (the skew '
            f'is reported, never enforced — ros_ws/docs/platform_fw_version.md), '
            f'but bench results from Teensy_code_platform/ are not trustworthy until the '
            f'Platform Teensy is re-flashed.')

    def _platform_fw_version_str(self) -> str:
        """Human/runbook rendering of the cached Platform-Teensy FW_VERSION.

        Three distinct verdicts, never collapsed: ``unknown`` (no authoritative
        read — a CAN3/relay problem, which also forces a re-home), ``0
        (PRE-VERSIONING)`` (the board answered and is un-flashed), or the number.
        """
        version = self._platform_fw_version
        if version is None:
            return 'unknown'
        if version == rpc_args.PLATFORM_FW_VERSION_UNVERSIONED:
            return '0 (PRE-VERSIONING)'
        return str(version)

    def relay_write_robot_state(self, is_homed, levelling_complete,
                                pose_offset_tiltX=0.0, pose_offset_tiltY=0.0):
        """STATE_WRITE: write the whole Platform-Teensy RobotState (the bridge is
        the sole writer; the firmware re-encodes the 0x6E0 frame). No reply —
        returns (ok, message) from the synchronous RPC ack. Serialized with the relay
        reads via _relay_lock so the STATE_WRITE read-modify-write
        cannot interleave with a concurrent STATE_READ (lost update / cache skew)."""
        with self._relay_lock:
            ok, msg, _ = self._call_rpc(
                RpcMethod.STATE_WRITE,
                rpc_args.encode_state_write(is_homed, levelling_complete,
                                            pose_offset_tiltX, pose_offset_tiltY))
            return ok, msg

    # ── Cold-start state cache: read at boot + on reconnect, write on home/reboot ──
    # The Platform Teensy owns the persisted cold-start state. These methods keep
    # self._cold_start_state in sync with it: a relay
    # read REFRESHES the cache (off the publish path); a relay write does
    # read-modify-write THROUGH the cache so a homing write preserves levelling +
    # pose, and vice versa. The 100 Hz publish path only reads the cache.

    def _refresh_cold_start_state(self, reason: str) -> bool:
        """Read the Platform Teensy RobotState via the relay and refresh the cache.

        Runs the relay read WITHOUT holding self._lock (it blocks for the CAN3
        round-trip), then swaps the new value into the cache under the lock. On
        success the cache + the authoritative flag are updated; on FAILURE the
        cache is LEFT UNCHANGED (keep the last authoritative value — can_node's
        passive "last-known-state until a fresh frame" semantics; never downgrade a
        good read on a transient hiccup). Returns True iff a fresh state was read.

        NOT for the boot path's first read — see _boot_read_cold_start_state, which
        applies the conservative is_homed=False fallback when there is no prior
        authoritative value to keep.
        """
        try:
            ok, msg, state = self.relay_read_robot_state()
        except Exception as e:  # noqa: BLE001 — never let a relay read crash a timer
            self.get_logger().error(
                f"cold-start read ({reason}) errored: {e}",
                throttle_duration_sec=5.0)
            return False
        if not ok or state is None:
            self.get_logger().warning(
                f"cold-start read ({reason}) failed: {msg} — keeping cached state",
                throttle_duration_sec=5.0)
            return False
        with self._lock:
            self._cold_start_state = state
            self._cold_start_authoritative = True
            if state.is_homed:
                # can_node.py:549-550 parity: homing requires a prior search, so a
                # read showing is_homed LATCHES encoder_search_complete True. Because
                # this bit is sticky (never cleared, incl. on reboot — can_node
                # leaves encoder_search_complete set), the DERIVED
                # encoder_search_complete = is_homed OR _encoder_search_done_session
                # is monotonic-True once homed/searched, exactly as can_node's field.
                self._encoder_search_done_session = True
        self.get_logger().info(
            f"cold-start state ({reason}): is_homed={int(state.is_homed)} "
            f"levelling={int(state.levelling_complete)} "
            f"pose=({state.pose_offset_tiltX:.4f},{state.pose_offset_tiltY:.4f})")
        return True

    def _boot_read_cold_start_state(self) -> bool:
        """Boot read: refresh the cold-start cache before the first publish, with a
        few bounded retries and the CONSERVATIVE-on-total-failure fallback. Returns
        True iff an authoritative read landed."""
        return self._read_cold_start_state_conservative('boot')

    def _read_cold_start_state_conservative(self, reason: str) -> bool:
        """Refresh the cold-start cache with bounded retries; on TOTAL failure fall
        back to the CONSERVATIVE cold value (is_homed=False / levelling=False /
        pose=0). Forcing a re-home is wasteful but SAFE; the reverse (a stale
        is_homed=True after the references may have been lost) could skip homing on
        an unhomed / de-referenced robot. Returns True iff an authoritative read
        landed.

        Used by the boot read (reason='boot') AND the CAN3-bus-health reconnect
        re-read (reason='can3_reconnect', firmware-validation precondition). NOTE the deliberate
        asymmetry vs _refresh_cold_start_state (the UDP-watchdog reconnect path),
        which KEEPS the stale cache on a failed read: a Jetson↔Teensy link blip does
        NOT imply the Platform Teensy lost power (its references are intact), so a
        re-home there would be wrong — can_node passive last-known parity. A CAN3
        bus-health recovery (WARN/BUS_OFF→OK) DOES imply the Jugglebot supply may
        have cycled (the Platform Teensy shares the ODrive supply), so a failed
        re-read must NOT keep a possibly-stale is_homed=True."""
        for attempt in range(self._BOOT_STATE_READ_ATTEMPTS):
            if self._refresh_cold_start_state(reason):
                return True
            if attempt + 1 < self._BOOT_STATE_READ_ATTEMPTS:
                time.sleep(self._BOOT_STATE_READ_RETRY_S)
        with self._lock:
            self._cold_start_state = RelayRobotState(
                is_homed=False, levelling_complete=False,
                pose_offset_tiltX=0.0, pose_offset_tiltY=0.0)
            self._cold_start_authoritative = False
        self.get_logger().warning(
            f"cold-start {reason} read failed after "
            f"{self._BOOT_STATE_READ_ATTEMPTS} attempts — defaulting to "
            "is_homed=False (conservative; forces a re-home).")
        # A MISSING answer must be as findable as a WRONG one: the same
        # PLATFORM_FW_CHECK token, so one grep of launch.log returns all three
        # verdicts (OK / FAIL / UNKNOWN) rather than two of them. Note this is
        # NOT the un-flashed signature — an un-flashed board answers, with 0.
        # No read landing at all means the relay itself is broken, which the
        # is_homed=False fallback above already makes loud on its own.
        if self._platform_fw_version is None:
            self.get_logger().error(
                'PLATFORM_FW_CHECK: UNKNOWN — no authoritative RobotState read '
                'has landed, so the Platform Teensy firmware version cannot be '
                'confirmed either way. Fix the relay/CAN3 read first; do not '
                'read this as evidence of a stale flash.')
        return False

    def _dispatch_cold_start_reread(self, fn, reason: str, thread_name: str) -> bool:
        """Run a cold-start re-read (``fn(reason)``) OFF the calling timer thread.

        BOTH cold-start re-read triggers fire from the 1 Hz _health_check timer,
        which shares the node's default MutuallyExclusiveCallbackGroup with the
        100 Hz _publish_robot_state — so a synchronous relay round-trip would stall
        the robot_state stream (the CAN3 conservative re-read up to ~1.9 s: audit
        2026-06-29 MEDIUM; the same off-thread fix extends to the
        UDP-watchdog reconnect re-read, ~0.5 s single round-trip). Dispatch to a
        short-lived daemon thread: it touches only self._cold_start_state /
        _cold_start_authoritative / _encoder_search_done_session under self._lock,
        and the relay RPC + _await_platform_reply primitives are thread-safe (the
        RX thread already feeds them).

        A SINGLE _cold_start_reread_inflight guard keeps at most one re-read of
        EITHER kind running at once — this deliberately prevents the CONSERVATIVE
        (CAN3-recovery) and the KEEP-STALE (UDP-reconnect) reads from racing the
        same cache, where the weaker keep-stale read could land its is_homed=True
        write LAST and resurrect a stale reference after a power-cycle. Returns
        True iff a re-read was started; False iff one was already in flight. The
        CAN3 caller uses the return to AVOID consuming its recovery edge when it
        loses the shared guard, so the conservative re-read re-fires next tick
        rather than being permanently preempted by a keep-stale read."""
        with self._lock:
            if self._cold_start_reread_inflight:
                return False
            self._cold_start_reread_inflight = True

        def _run():
            try:
                fn(reason)
            except Exception as e:  # noqa: BLE001 — never let the worker thread die silently
                self.get_logger().error(
                    f"async cold-start re-read ({reason}) errored: {e}",
                    throttle_duration_sec=5.0)
            finally:
                with self._lock:
                    self._cold_start_reread_inflight = False

        threading.Thread(target=_run, daemon=True, name=thread_name).start()
        return True

    def _write_is_homed(self, is_homed: bool):
        """Persist is_homed via STATE_WRITE, read-modify-write THROUGH the cache so
        levelling_complete + pose_offset are preserved (the bridge is the sole
        writer — a homing write must not clobber a prior levelling result, and vice
        versa). Updates the cache on success. Returns (ok, message).

        The whole read→wire-write→cache-update runs under _relay_lock so a
        concurrent _write_level_state cannot read the same stale cache
        and land its wire write last, clobbering is_homed on the Platform Teensy —
        the RMW is atomic across writers, not just the two wire writes serialized."""
        with self._relay_lock:
            with self._lock:
                cs = self._cold_start_state
            ok, msg = self.relay_write_robot_state(
                is_homed=bool(is_homed),
                levelling_complete=cs.levelling_complete,
                pose_offset_tiltX=cs.pose_offset_tiltX,
                pose_offset_tiltY=cs.pose_offset_tiltY)
            if ok:
                with self._lock:
                    # _replace on the CURRENT cache (re-read) preserves any concurrent
                    # levelling update; only is_homed is changed here.
                    self._cold_start_state = self._cold_start_state._replace(
                        is_homed=bool(is_homed))
                    self._cold_start_authoritative = True
        return ok, msg

    def _write_level_state(self, levelling_complete, tiltX, tiltY):
        """Persist the levelling result (levelling_complete + pose_offset) via
        STATE_WRITE, read-modify-write THROUGH the cache so is_homed is preserved (the
        bridge is the sole writer — a levelling write must not clobber a prior homing
        result, and vice versa). The orchestrator set_level_state subscriber's persist
        path; byte-parity with can_node._sub_set_level_state → _update_teensy_state
        (which merged into last_known_state, so is_homed carried through). Updates the
        cache on success. Returns ``(ok, message)``.

        Whole RMW under _relay_lock — see _write_is_homed; the
        symmetric case is a concurrent homing write being clobbered here."""
        with self._relay_lock:
            with self._lock:
                cs = self._cold_start_state
            ok, msg = self.relay_write_robot_state(
                is_homed=cs.is_homed,                 # preserve (read-modify-write)
                levelling_complete=bool(levelling_complete),
                pose_offset_tiltX=float(tiltX),
                pose_offset_tiltY=float(tiltY))
            if ok:
                with self._lock:
                    # _replace on the CURRENT cache (re-read) preserves any concurrent
                    # is_homed update; only levelling + pose are changed here.
                    self._cold_start_state = self._cold_start_state._replace(
                        levelling_complete=bool(levelling_complete),
                        pose_offset_tiltX=float(tiltX),
                        pose_offset_tiltY=float(tiltY))
                    self._cold_start_authoritative = True
        return ok, msg

    def _clear_cold_start_state_on_reboot(self):
        """REBOOT_ODRIVES shared-hook step 2 (cold-start clear): clear is_homed +
        levelling_complete + pose_offset on the Platform Teensy — the ODrives lose
        their references on reboot, so all three are cleared together (mirrors
        can_node.py:1559-1565). Retried (the Platform Teensy stays powered through
        the reboot, so a dropped clear would leave a dangerous stale is_homed=True).
        The cache is set to the cleared value regardless (the safe local view).
        Returns (ok, message).

        The retried wire clears + the cache reset run under _relay_lock so a
        concurrent homing/levelling write cannot land between a
        successful clear and the cache reset and resurrect a stale is_homed=True."""
        ok, msg = False, 'no attempt'
        with self._relay_lock:
            for attempt in range(self._REBOOT_CLEAR_ATTEMPTS):
                ok, msg = self.relay_write_robot_state(
                    is_homed=False, levelling_complete=False,
                    pose_offset_tiltX=0.0, pose_offset_tiltY=0.0)
                if ok:
                    break
                if attempt + 1 < self._REBOOT_CLEAR_ATTEMPTS:
                    time.sleep(self._REBOOT_CLEAR_RETRY_S)
            with self._lock:
                self._cold_start_state = RelayRobotState(
                    is_homed=False, levelling_complete=False,
                    pose_offset_tiltX=0.0, pose_offset_tiltY=0.0)
                self._cold_start_authoritative = True
                # Clear the in-session encoder-search bit too: a reboot resets the ODrive
                # MCUs, which lose their INCREMENTAL-ENCODER INDEX (not pre-calibrated to
                # flash — operator-confirmed 2026-07-02), so a re-encoder-search is
                # REQUIRED before the next home. The derived
                #   encoder_search_complete = is_homed OR _encoder_search_done_session
                # must therefore go False after a reboot (is_homed is already cleared
                # above). can_node.py:1552-1566 did NOT clear this — a LATENT BUG that only
                # bit once the orchestrator drove homing AUTOMATICALLY after a reboot
                # (orchestrator-driven auto-home): HomingHandler skipped encoder-search on the stale True and
                # homed on an un-indexed encoder → ODRIVE_FATAL → FAULT→BOOT→HOMING loop
                # (hardware, 2026-07-02; see logbook 2026-07-02-canbridge-reboot-encoder-
                # search-clear). Clearing it re-runs encoder-search on the next cold-start,
                # exactly as a fresh launch does. Deliberate divergence from can_node's
                # literal behaviour: the reboot must clear EVERYTHING the ODrive loses —
                # references (is_homed/levelling/pose) AND the encoder index.
                self._encoder_search_done_session = False
        return ok, msg

    # ── Encoder index search (Jetson-side orchestration) ──
    # The firmware ENCODER_SEARCH RPC is stubbed (ERR_NOT_IMPL); encoder index
    # search is an ODrive-autonomous axis state, so we orchestrate it from here
    # over the implemented SET_AXIS_STATE primitive + the telemetry/diagnostic
    # cache. The pure sequencing lives in teensy_link/encoder_search.py
    # (unit-tested); this method is the I/O loop around it. Homing (the firmware move) is
    # the part that must live in firmware (no per-leg motion RPC).

    def _encoder_axis_status(self, axes):
        """Build per-axis ``AxisStatus`` for the search state machine from the
        latest telemetry (pos → finite) + diagnostic (axis_state, active_errors)
        cache. An axis with no diagnostic yet is omitted — the state machine
        treats a missing axis as "no fresh status", aging it toward its timeout.
        """
        out = {}
        with self._lock:
            telem = self._latest_telemetry
            diag = dict(self._latest_diag)
        if telem is None:
            return out
        for axis in axes:
            d = diag.get(int(axis))
            if d is None:
                continue
            out[int(axis)] = AxisStatus(
                axis_state=int(d.axis_state),
                pos_finite=math.isfinite(float(telem.pos_rev[int(axis)])),
                active_errors=int(d.active_errors))
        return out

    def _run_encoder_search(self, axes, *, poll_dt=0.05):
        """Drive encoder index search on ``axes`` over the can-bridge link.

        Synchronous: blocks the calling (executor) thread until the search
        finishes — acceptable for a deliberate cold-start op; the RX + heartbeat
        threads keep the link alive and the telemetry cache fresh throughout, and
        the heartbeat stays ``mpc_active=0`` (no setpoint output). Returns
        ``(ok, message)``.
        """
        axes = [int(a) for a in axes]
        if not axes:
            return False, "no axes configured for encoder search"
        es = EncoderSearch(axes, timeout_s=2.0 * hw.JB_OP_ENCODER_SEARCH_TIMEOUT_S)
        # Belt-and-suspenders wall-clock bound; the state machine self-terminates
        # via its own per-axis timeout × (retries + 1) well inside this.
        hard_deadline = time.monotonic() + es.timeout_s * (es.max_retries + 1) + 5.0
        self.get_logger().info(f"encoder search: starting on axes {axes}")
        while True:
            now = time.monotonic()
            res = es.step(now, self._encoder_axis_status(axes))
            for axis in res.clear_errors:
                self.teensy_clear_errors(axis)
            for axis in res.set_search:
                self.teensy_set_axis_state(axis, AXIS_STATE_ENCODER_INDEX_SEARCH)
            if res.done:
                break
            if now > hard_deadline:
                self.get_logger().error("encoder search: hard deadline exceeded")
                break
            time.sleep(poll_dt)
        if es.done and not es.failed:
            msg = f"encoder search complete on axes {es.succeeded}"
            self.get_logger().info(msg)
            return True, msg
        parts = [f"axis {a}: {r}" for a, r in es.failed.items()]
        msg = "encoder search FAILED — " + "; ".join(parts)
        if es.succeeded:
            msg += f" (succeeded: {es.succeeded})"
        self.get_logger().error(msg)
        return False, msg

    # ── Homing (firmware move, Jetson-side observation) ──
    # Unlike encoder search, the homing *move* runs autonomously in the can-bridge
    # HOME handler (no per-leg motion RPC; the velocity-limited move-to-hardstop
    # must live in firmware). The Jetson fires HOME (fire-and-monitor) and watches
    # the telemetry + diagnostic cache for completion via HomingMonitor (pure,
    # unit-tested). The firmware homes one axis at a time, so axes are homed
    # sequentially.

    def _homing_axis_status(self, axis):
        """Build the :class:`HomingAxisStatus` for ``axis`` from the latest
        telemetry (pos) + diagnostic (axis_state, active_errors) cache, or None
        if no diagnostic has arrived yet."""
        with self._lock:
            telem = self._latest_telemetry
            d = self._latest_diag.get(int(axis))
        if telem is None or d is None:
            return None
        return HomingAxisStatus(
            axis_state=int(d.axis_state),
            pos_rev=float(telem.pos_rev[int(axis)]),
            active_errors=int(d.active_errors),
            homing_result=int(d.homing_result))   # authoritative outcome from the HomingResult uplink

    def _run_home(self, axes, *, poll_dt=0.05):
        """Home ``axes`` over the can-bridge link, one axis at a time.

        Synchronous: blocks the calling (executor) thread until homing finishes —
        acceptable for a deliberate cold-start op; the RX + heartbeat threads keep
        the link alive and the telemetry cache fresh throughout, and the heartbeat
        stays ``mpc_active=0`` (no setpoint output). Returns ``(ok, message)``.
        """
        axes = [int(a) for a in axes]
        if not axes:
            return False, "no axes configured for homing"
        # Per-axis observer timeout BACKSTOP. MUST sit BELOW the firmware per-axis
        # hard timeout (Homing::MOTOR_TIMEOUT_S = 30 s). Since the HomingResult uplink
        # landed (see logbook/2026-07-05-canhub-hardening-18a-homing-result-uplink.md),
        # the observer trusts the firmware's uplinked HomingResult, so a firmware abort —
        # including its own 30 s timeout — is reported as HOMING_FAILED directly; this
        # host timeout only catches a leg that never resolves at all (e.g. a lost
        # RUNNING→terminal transition). A real home completes in a few seconds, so 20 s
        # is generous and still below the firmware's 30 s.
        timeout_s = min(20.0, float(hw.HOMING_MOTOR_TIMEOUT_S) - 5.0)
        succeeded, failed = [], {}
        # The firmware briefly rejects a HOME (ERR_REJECTED, busy) while it finishes
        # the PREVIOUS axis: axis_state reads IDLE during STOP_SETTLE — before
        # set_absolute_position completes and s_phase returns to IDLE (~10-20 ms) —
        # and the observer (no longer gating on position; the legs relax off a foam
        # stop, 2026-06-26) can fire the next axis's HOME inside that window. Retry
        # across ticks until the firmware accepts it (the principled form of the
        # post-TX delays can_node used). Bounded so a genuinely-stuck axis fails.
        _MAX_REJECT_RETRIES = 20   # × poll_dt ≈ 1 s; busy window is ~10-20 ms
        self.get_logger().info(f"homing: starting on axes {axes} (sequential)")
        for axis in axes:
            is_hand = (axis == _HAND_AXIS)
            home_ref = abs(float(
                hw.HOMING_HAND_ABS_POS_REV if is_hand else hw.HOMING_LEG_ABS_POS_REV))
            # The hand (axis 6) homes with the SAME firmware move-to-hardstop
            # (Homing::HAND_* params) but its PID gains must be applied FIRST —
            # byte-identical to can_node._home_robot_steps' HAND branch
            # (_set_hand_gains() then the move). Refuse-flash-defaults: abort the
            # sequence if a gain write fails rather than drive the hand into a
            # hardstop on flash-default gains (can_node._set_hand_gains raised).
            if is_hand:
                gok, gmsg = self._apply_hand_gains()
                if not gok:
                    failed[axis] = gmsg
                    self.get_logger().error(f"homing: hand gain apply failed — {gmsg}")
                    break
            mon = HomingMonitor([axis], home_ref_rev=home_ref, timeout_s=timeout_s)
            hard_deadline = time.monotonic() + timeout_s + 5.0
            home_accepted = False     # firmware has ACCEPTED the HOME for this axis
            reject_retries = 0
            while True:
                now = time.monotonic()
                st = self._homing_axis_status(axis)
                res = mon.step(now, {axis: st} if st is not None else {})
                # Fire on the monitor's request, then keep re-firing until accepted.
                if res.set_home or not home_accepted:
                    ok, msg, _ = self.teensy_home(axis)
                    if ok:
                        home_accepted = True
                    elif 'REJECTED' in msg.upper() and reject_retries < _MAX_REJECT_RETRIES:
                        reject_retries += 1   # transient busy — retry next tick
                    else:
                        # Non-transient failure (bus down / bad axis), or retries
                        # exhausted (firmware stuck busy).
                        failed[axis] = f"HOME rejected: {msg}"
                        mon = None
                        break
                if mon is None or res.done:
                    break
                if now > hard_deadline:
                    self.get_logger().error(f"homing: hard deadline on axis {axis}")
                    failed[axis] = "hard deadline exceeded"
                    mon = None
                    break
                time.sleep(poll_dt)
            if mon is not None:
                succeeded.extend(mon.succeeded)
                failed.update(mon.failed)
            if axis in failed:
                # Abort the sequence on the first failure (matches can_node
                # _home_robot, which returns False on any motor's failure).
                break
        if failed:
            parts = [f"axis {a}: {r}" for a, r in failed.items()]
            msg = "homing FAILED — " + "; ".join(parts)
            if succeeded:
                msg += f" (succeeded: {succeeded})"
            self.get_logger().error(msg)
            return False, msg
        msg = f"homing complete on axes {succeeded}"
        self.get_logger().info(msg)
        return True, msg

    def _apply_hand_gains(self):
        """Apply the hand PID gains to axis 6 (before HOME(6) and in _run_configure).
        Byte-identical to can_node._set_hand_gains: SET_POS_GAIN then SET_VEL_GAINS
        from self._hand_gains. Refuse-flash-defaults — returns ``(False, reason)`` if
        a gain RPC fails, so the caller aborts rather than home/configure the hand on
        flash-default gains (the safety can_node held; _set_hand_gains raised there).
        Returns ``(ok, message)``."""
        g = self._hand_gains
        ok1, m1, _ = self.teensy_set_pos_gain(_HAND_AXIS, g['pos_gain'])
        ok2, m2, _ = self.teensy_set_vel_gains(_HAND_AXIS, g['vel_gain'], g['vel_int_gain'])
        if ok1 and ok2:
            return True, "hand gains applied"
        failed = [nm for ok, nm in ((ok1, 'hand.pos_gain'), (ok2, 'hand.vel_gains')) if not ok]
        return False, ("hand gain write failed on CAN: " + ", ".join(failed)
                       + " — hand may be on flash defaults")

    # ── Configure + activate (Teensy-side cold-start) ──
    # The Teensy-side path has only one leg-motion path (the gated 40 Hz setpoint stream),
    # so the can_node `_setup_odrives_steps` (gains/limits/mode) and
    # `_gentle_move_steps` (move to active pose) have no equivalent until here.
    # `_run_configure` is the pure-config _setup_odrives analogue; `_run_activate`
    # fires the firmware TRAP_TRAJ activate op + observes it. `/home` calls
    # `_run_configure` at completion (the operator's "set after every homing").

    def _run_configure(self, axes):
        """Apply the Teensy-side cold-start config to ``axes`` (the _setup_odrives
        analogue): per-leg position/velocity gains + vel/curr limits +
        POSITION/PASSTHROUGH controller mode. Idempotent and motion-free — it does
        NOT change axis_state or command a position (that is /activate's job), so it
        is safe to run after homing (legs IDLE at the hardstop) AND again after
        activate (legs CLOSED_LOOP holding the active pose, switching TRAP_TRAJ →
        PASSTHROUGH for the interp). Returns ``(ok, message)``.
        """
        axes = [int(a) for a in axes]
        if not axes:
            return False, "no axes configured for configure"
        leg_axes = [a for a in axes if a < p.NUM_LEGS]
        do_hand = _HAND_AXIS in axes
        ctrl = proto.ODRIVE_CONTROL_MODES['POSITION']
        inp = proto.ODRIVE_INPUT_MODES['PASSTHROUGH']
        failed = []
        self.get_logger().info(f"configure: applying gains/limits/PASSTHROUGH on axes {axes}")
        for axis in leg_axes:
            for name, (ok, m, _) in (
                ("pos_gain",
                 self.teensy_set_pos_gain(axis, hw.ODRIVE_LEG_POS_GAINS[axis])),
                ("vel_gains",
                 self.teensy_set_vel_gains(axis, hw.ODRIVE_LEG_VEL_GAINS[axis],
                                           hw.ODRIVE_LEG_VEL_INT_GAINS[axis])),
                ("vel_curr_limits",
                 self.teensy_set_vel_curr_limits(axis, hw.ODRIVE_LEG_VEL_LIMIT_RPS,
                                                 hw.ODRIVE_LEG_CURR_LIMIT_A)),
                ("controller_mode",
                 self.teensy_set_controller_mode(axis, ctrl, inp)),
            ):
                if not ok:
                    failed.append(f"axis {axis} {name}: {m}")
        if do_hand:
            # Configure the HAND (axis 6) — the hand half of can_node's cold-start
            # _setup_odrives (per the can_node<->Teensy parity audit): hand PID gains
            # (refuse-flash-defaults) + hand vel/curr limits + POSITION/PASSTHROUGH. Gains + limits
            # come from self._hand_gains / self._hand_vel_limit / self._hand_curr_limit
            # (config defaults, updatable via set_hand_gains / set_motor_vel_curr_limits).
            gok, gmsg = self._apply_hand_gains()
            if not gok:
                failed.append(f"axis {_HAND_AXIS} {gmsg}")
            for name, (ok, m, _) in (
                ("vel_curr_limits",
                 self.teensy_set_vel_curr_limits(_HAND_AXIS, self._hand_vel_limit,
                                                 self._hand_curr_limit)),
                ("controller_mode",
                 self.teensy_set_controller_mode(_HAND_AXIS, ctrl, inp)),
            ):
                if not ok:
                    failed.append(f"axis {_HAND_AXIS} {name}: {m}")
        if failed:
            msg = "configure FAILED — " + "; ".join(failed)
            self.get_logger().error(msg)
            return False, msg
        msg = f"configure complete on axes {axes}"
        self.get_logger().info(msg)
        return True, msg

    def _activate_axis_status(self, axes):
        """Build per-axis ``ActivateAxisStatus`` for the activate observer from the
        latest telemetry (pos/vel) + diagnostic (axis_state, active_errors) cache.
        An axis with no diagnostic yet is omitted (aged toward its timeout)."""
        out = {}
        with self._lock:
            telem = self._latest_telemetry
            diag = dict(self._latest_diag)
        if telem is None:
            return out
        for axis in axes:
            d = diag.get(int(axis))
            if d is None:
                continue
            out[int(axis)] = ActivateAxisStatus(
                axis_state=int(d.axis_state),
                pos_rev=float(telem.pos_rev[int(axis)]),
                vel_rps=float(telem.vel_rps[int(axis)]),
                active_errors=int(d.active_errors))
        return out

    def _deactivate_axis_status(self, axes):
        """Build per-axis ``DeactivateAxisStatus`` for the deactivate observer from
        the latest telemetry (pos/vel) + diagnostic (axis_state, active_errors)
        cache. An axis with no diagnostic yet is omitted (aged toward its timeout)."""
        out = {}
        with self._lock:
            telem = self._latest_telemetry
            diag = dict(self._latest_diag)
        if telem is None:
            return out
        for axis in axes:
            d = diag.get(int(axis))
            if d is None:
                continue
            out[int(axis)] = DeactivateAxisStatus(
                axis_state=int(d.axis_state),
                pos_rev=float(telem.pos_rev[int(axis)]),
                vel_rps=float(telem.vel_rps[int(axis)]),
                active_errors=int(d.active_errors))
        return out

    def _run_activate(self, axes, *, poll_dt=0.05):
        """Fire the firmware ACTIVATE (TRAP_TRAJ move to the active pose) and
        observe it to completion. A single configured axis fires that leg; any
        larger set fires ``AXIS_ALL`` (every PRESENT leg, parallel even-rise). The
        ActivateMonitor watches each leg reach its active-pose target + settle.

        Synchronous: blocks the calling (executor) thread for the ~seconds the move
        takes; the RX + heartbeat threads keep the link + telemetry cache alive and
        the heartbeat stays ``mpc_active=0`` (no setpoint output). Returns
        ``(ok, message)``.

        Precondition: a prior ``/configure`` (gains/limits/mode). Without it the
        legs run on flash-default gains and the move fails safe (never reaches the
        target → ActivateMonitor timeout → FAILED).
        """
        axes = [int(a) for a in axes]
        if not axes:
            return False, "no axes configured for activate"
        targets = {a: float(hw.JB_OP_ACTIVATE_POSITION_REVS[a]) for a in axes}
        # Single configured axis → that leg only; otherwise AXIS_ALL (all present
        # legs in parallel — the platform rises straight up, no tilt).
        fire_axis = axes[0] if len(axes) == 1 else rpc_args.AXIS_ALL
        # Footgun guard: a partial multi-leg subset fires AXIS_ALL (every PRESENT
        # leg), so legs NOT in activate_axes are still driven but unobserved. Only
        # [single leg] or the full leg set are coherent; warn on anything else.
        if len(axes) > 1 and set(axes) != set(range(p.NUM_LEGS)):
            self.get_logger().warning(
                f"activate_axes={axes} is a partial subset — ACTIVATE(AXIS_ALL) "
                f"will move ALL present legs, but only {axes} are observed. Use a "
                f"single leg or the full leg set.")
        ok, msg, _ = self.teensy_activate(fire_axis)
        if not ok:
            return False, f"ACTIVATE rejected: {msg}"
        mon = ActivateMonitor(axes, targets)
        hard_deadline = time.monotonic() + mon.timeout_s + 5.0
        self.get_logger().info(f"activate: TRAP_TRAJ move to active pose on axes {axes}")
        while True:
            now = time.monotonic()
            res = mon.step(now, self._activate_axis_status(axes))
            if res.done:
                break
            if now > hard_deadline:
                self.get_logger().error("activate: hard deadline exceeded")
                break
            time.sleep(poll_dt)
        if mon.failed:
            parts = [f"axis {a}: {r}" for a, r in mon.failed.items()]
            msg = "activate FAILED — " + "; ".join(parts)
            if mon.succeeded:
                msg += f" (succeeded: {mon.succeeded})"
            self.get_logger().error(msg)
            return False, msg
        msg = f"activate complete on axes {mon.succeeded}"
        self.get_logger().info(msg)
        return True, msg

    def _legs_already_stowed(self, axes):
        """True iff every leg in ``axes`` is already IDLE and within
        ``_STOW_POS_MAX_REV`` of the STOW pose (``|pos| <= that``; STOW = 0.0 rev, the
        DeactivateMonitor convention) — i.e. the platform is already stowed, so a
        DEACTIVATE would be a redundant re-arm+lower. Fail-SAFE: a missing telemetry
        or per-axis diagnostic (cannot confirm) returns False, so the deactivate
        proceeds normally rather than silently skipping a real lower."""
        with self._lock:
            telem = self._latest_telemetry
            diag = dict(self._latest_diag)
        if telem is None:
            return False
        for axis in axes:
            d = diag.get(int(axis))
            if d is None:
                return False                                          # no diag → can't confirm
            if int(d.axis_state) != proto.ODRIVE_STATES['IDLE']:
                return False                                          # a leg is not IDLE
            pos = float(telem.pos_rev[int(axis)])
            # NaN-safe: pos_rev is NaN until the encoder is ready / after an encoder
            # fault (a faulted active leg drops to IDLE reporting NaN). `abs(nan) > x`
            # is False, so a bare `> _STOW_POS_MAX_REV` would MISCLASSIFY a NaN leg as
            # stowed and skip a needed lower — leaving the platform UP. Treat non-finite
            # as "cannot confirm" → not stowed (fail-safe), mirroring _encoder_search_axis_status.
            if not math.isfinite(pos) or abs(pos) > _STOW_POS_MAX_REV:
                return False                                          # extended or unknown
        return True

    def _run_deactivate(self, axes, *, poll_dt=0.05, timeout_s=None):
        """Fire the firmware DEACTIVATE (TRAP_TRAJ controlled lower to STOW, then
        IDLE) and observe it to completion. A single configured axis fires that
        leg; any larger set fires ``AXIS_ALL`` (every PRESENT leg, parallel
        even-descent). The DeactivateMonitor watches each leg reach IDLE (the
        firmware's clean completion — not the foam-unreliable resting position).

        Synchronous: blocks the calling (executor) thread for the ~seconds the
        descent takes; the RX + heartbeat threads keep the link + telemetry cache
        alive and the heartbeat stays ``mpc_active=0`` (no setpoint output).
        Returns ``(ok, message)``.

        ``timeout_s`` overrides the DeactivateMonitor per-run budget (default None →
        the monitor's own default). The shutdown-stow path passes a tighter budget
        so a stalled descent can't blow the ~8 s teardown window.

        Precondition: the legs are holding the active pose in CLOSED_LOOP (a prior
        /activate). Without it the move fails safe (an errored or absent leg is
        rejected by the firmware; a stalled leg → DeactivateMonitor timeout).
        """
        axes = [int(a) for a in axes]
        if not axes:
            return False, "no axes configured for deactivate"
        self._deactivate_in_progress = True
        try:
            return self._run_deactivate_inner(axes, poll_dt=poll_dt,
                                              timeout_s=timeout_s)
        finally:
            self._deactivate_in_progress = False

    def _run_deactivate_inner(self, axes, *, poll_dt=0.05, timeout_s=None):
        # ARMING_CONTRACT A3 — disarm-before-stow, in-process. This is the ONE
        # place the ordering is airtight (same thread, synchronous), and it covers
        # every deactivate entry point: the orchestrator's ACTIVE→IDLE, the direct
        # /deactivate service, and the shutdown stow. The producer keeps streaming
        # through the descent (A4 keeps the mode published) — once disarmed the
        # Teensy ignores the stream and its staleness watchdog is inert
        # (s_mpc_active gates it), so the still-flowing frames are harmless. The
        # firmware's reject-DEACTIVATE-while-armed gate remains as the backstop,
        # not the mechanism (its refusal used to strand the platform un-stowed —
        # Sharp Edge #6, observed 2026-07-09).
        if self._mpc_active:
            self.get_logger().warning(
                "deactivate requested while armed — disarming first "
                "(ARMING_CONTRACT A3: disarm-before-stow)")
            self._stop_setpoint_output()
            # The disarm is only STAGED until the next 10 Hz heartbeat tick —
            # firing DEACTIVATE immediately would race s_mpc_active=1 on the
            # Teensy and be rejected. Wait for the firmware's arm-took bit
            # (T2J bit3) to confirm the disarm landed.
            if not self._wait_wire_disarmed():
                self.get_logger().error(
                    "disarm did not confirm on the wire (T2J bit3 still set "
                    "after 1 s) — proceeding; the firmware rejects DEACTIVATE "
                    "if truly armed (loud failure, no motion)")
        # Already-at-STOW no-op guard: if every target leg is already IDLE and within
        # _STOW_POS_MAX_REV of STOW, the platform is stowed — a DEACTIVATE would
        # needlessly re-arm CLOSED_LOOP (a jolt) only to lower nothing, then IDLE. Skip
        # it and report success. (This also short-circuits a redundant stow-on-shutdown
        # when the platform was already brought down.)
        if self._legs_already_stowed(axes):
            self.get_logger().info(
                f"deactivate: axes {axes} already at STOW (IDLE, |pos| <= "
                f"{_STOW_POS_MAX_REV} rev) — skipping the leg move")
            # Only the redundant LEG re-arm+lower is skipped; DEACTIVATE still
            # de-energises the HAND (axis 6) to preserve can_node's "idle ALL
            # JUGGLEBOT_AXES" parity (so e.g. a stow-on-shutdown with legs already down
            # but the hand armed still drops the hand). Best-effort, as in the full path.
            hok, hmsg, _ = self.teensy_set_axis_state(_HAND_AXIS, proto.ODRIVE_STATES['IDLE'])
            if not hok:
                self.get_logger().warning(
                    f"deactivate (already stowed): hand IDLE failed — {hmsg}")
            return True, f"already at STOW (axes {axes})"
        # Single configured axis → that leg only; otherwise AXIS_ALL (all present
        # legs in parallel — the platform lowers straight down, no tilt).
        fire_axis = axes[0] if len(axes) == 1 else rpc_args.AXIS_ALL
        # Footgun guard: a partial multi-leg subset fires AXIS_ALL (every PRESENT
        # leg), so legs NOT in deactivate_axes are still driven but unobserved.
        if len(axes) > 1 and set(axes) != set(range(p.NUM_LEGS)):
            self.get_logger().warning(
                f"deactivate_axes={axes} is a partial subset — DEACTIVATE(AXIS_ALL) "
                f"will move ALL present legs, but only {axes} are observed. Use a "
                f"single leg or the full leg set.")
        ok, msg, _ = self.teensy_deactivate(fire_axis)
        if not ok:
            return False, f"DEACTIVATE rejected: {msg}"
        mon = (DeactivateMonitor(axes, timeout_s=float(timeout_s))
               if timeout_s is not None else DeactivateMonitor(axes))
        hard_deadline = time.monotonic() + mon.timeout_s + 5.0
        self.get_logger().info(
            f"deactivate: TRAP_TRAJ lower to STOW + IDLE on axes {axes}")
        while True:
            now = time.monotonic()
            res = mon.step(now, self._deactivate_axis_status(axes))
            if res.done:
                break
            if now > hard_deadline:
                self.get_logger().error("deactivate: hard deadline exceeded")
                break
            time.sleep(poll_dt)
        # Idle the HAND (axis 6) too: can_node's deactivate de-energised ALL
        # JUGGLEBOT_AXES (legs via the TRAP_TRAJ→IDLE above; hand via
        # SET_AXIS_STATE(IDLE), can_node.py:1541-1543). ACTIVATE/DEACTIVATE are
        # rejected on axis 6 (leg-specific cold-start moves), so the hand is idled
        # explicitly here. Best-effort — a failure is logged but does not fail the
        # leg deactivate result (per the can_node<->Teensy parity audit).
        hok, hmsg, _ = self.teensy_set_axis_state(_HAND_AXIS, proto.ODRIVE_STATES['IDLE'])
        if not hok:
            self.get_logger().warning(f"deactivate: hand IDLE failed — {hmsg}")
        if mon.failed:
            parts = [f"axis {a}: {r}" for a, r in mon.failed.items()]
            msg = "deactivate FAILED — " + "; ".join(parts)
            if mon.succeeded:
                msg += f" (succeeded: {mon.succeeded})"
            self.get_logger().error(msg)
            return False, msg
        msg = f"deactivate complete on axes {mon.succeeded}"
        self.get_logger().info(msg)
        return True, msg

    # ── ROS service handlers (existing-type subset) ───────────

    def _svc_clear_errors(self, req, res):
        # 2026-07-11 clear-errors jolt: WHILE ARMED (mpc_active=1) the guard is actively
        # gating leg output, so a bare clear onto a diverged command re-enables output
        # with u0 off the drifted encoder → the pos_gain × lead velocity kick to the
        # current rail. Route the armed bare /clear_errors through the SAME converge-first
        # sequence as /recover (reseed → verify u0 on the encoder → clear) so no clear can
        # re-trip or jolt — there is no raw armed escape hatch (operator decision).
        # WHEN NOT ARMED (mpc_active=0 — e.g. the benign boot-time MPC_STALE latch: output
        # is not being evaluated, no setpoint is streaming), there is nothing to converge
        # and no jolt is possible, so clear DIRECTLY as before (a reseed would refuse —
        # not streaming — and needlessly block the clear).
        if self._mpc_active:
            res = self._svc_recover(req, res)
            if res.success:
                return res
            # ARMING_CONTRACT defense-in-depth: /recover's converge-first sequence
            # needs a live seeded stream to reseed onto. Armed-with-no-healthy-
            # stream is structurally unreachable under the contract (A1 refuses to
            # arm without one; A3 disarms before the stream legitimately stops),
            # but when reached anyway the old behaviour was a hard dead end:
            # clear reroutes to recover → recover needs a reseed → reseed refused
            # → still latched, forever ("trajectory_node reseed refused",
            # 2026-07-15 21:32). Disarming makes the direct clear safe: at
            # mpc_active=0 the firmware's guard terms are inert and the output
            # gate is off, so the clear can neither re-latch nor jolt — the legs
            # stay position-held by the ODrives. Re-arming afterwards must pass
            # the full A1 pre-check again.
            prior = res.message
            self.get_logger().error(
                f"armed /clear_errors: recover failed ({prior}) — falling back "
                f"to disarm + direct clear (safe: guard terms inert at "
                f"mpc_active=0; re-arm via /set_setpoint_output)")
            self._stop_setpoint_output()
            # The no-jolt property of this fallback depends on the clear landing
            # AFTER the firmware sees the disarm — a clear racing s_mpc_active=1
            # is exactly the 2026-07-11 jolt class. Refuse rather than race.
            if not self._wait_wire_disarmed():
                res.success = False
                res.message = (f'recover failed ({prior}); disarm did not '
                               f'confirm on the wire (T2J bit3 still set) — '
                               f'REFUSING the direct clear (it could jolt). '
                               f'Wire is disarming; retry /clear_errors.')
                return res
            ok, msg, _ = self.teensy_clear_errors()
            res.success = ok
            res.message = (f'recover failed ({prior}); disarmed and cleared '
                           f'directly — {msg}. Wire is DISARMED; re-arm via '
                           f'/set_setpoint_output when a stream is live.')
            return res
        ok, msg, _ = self.teensy_clear_errors()
        res.success = ok
        res.message = msg
        return res

    def _reboot_odrives(self):
        """REBOOT_ODRIVES shared ordered hook (mirrors can_node._reboot_odrives_
        steps). Step 1 — the bounded watchdog-suppression latch — is owned elsewhere
        (see logbook/2026-06-30-canbridge-phase6-reboot-latch.md) and out of scope
        here. Step 2 (cold-start clear): after firing the reboot, clear the
        persisted cold-start state on the Platform Teensy (the ODrives lose their
        references on reboot). Returns (ok, message)."""
        ok, msg, _ = self.teensy_reboot()
        # Step 2: clear is_homed/levelling/pose together. Unconditional on the
        # reboot result (parity with can_node, which always cleared) — clearing is
        # the SAFE direction (a re-home), never the reverse.
        cs_ok, cs_msg = self._clear_cold_start_state_on_reboot()
        if not cs_ok:
            self.get_logger().error(
                f"reboot: clearing cold-start state failed: {cs_msg}")
            msg = f"{msg}; WARNING cold-start clear failed: {cs_msg}"
        return ok, msg

    def _svc_reboot_odrives(self, req, res):
        ok, msg = self._reboot_odrives()
        res.success = ok
        res.message = msg
        return res

    def _svc_encoder_search(self, req, res):
        # Jetson-side orchestration over SET_AXIS_STATE (the firmware
        # ENCODER_SEARCH RPC remains stubbed). Scope via the encoder_search_axes
        # parameter (default all legs; [0] for the standalone-leg bench rig).
        axes = list(self.get_parameter('encoder_search_axes').value or [])
        ok, msg = self._run_encoder_search(axes)
        if ok:
            # Cold-start state: track the in-session search-done bit so the DERIVED
            # encoder_search_complete = is_homed OR within-session-search-done
            # (can_node parity — see _publish_robot_state). Sticky-True for the run.
            with self._lock:
                self._encoder_search_done_session = True
        res.success = ok
        res.message = msg
        return res

    def _do_home(self):
        """Home the configured axes (legs + hand), persist the is_homed RESULT, and
        (on success) apply the Teensy-side cold-start config — the shared body behind BOTH
        the manual ``/home`` Trigger service AND the orchestrator-facing
        ``home_motors`` action. One definition, so the two entry points can
        never drift. Returns ``(ok, message)``.

        Scope via the ``home_axes`` parameter (default all legs + the hand; ``[0]``
        for the standalone-leg bench rig). The firmware HOME handler runs the
        velocity-limited move-to-hardstop autonomously; ``_run_home`` drives +
        observes it (applying the hand gains before HOME(6)).

        Persisting is_homed = the home RESULT (read-modify-write through the cache so
        levelling/pose are preserved) is the safety crux, EXACT can_node.py:847 parity
        (``_update_teensy_state({'is_homed': success})`` writes ``success``, i.e.
        False on a FAILED home too): a failed re-home of an already-homed robot MUST
        clear is_homed, or a stale is_homed=True would let the orchestrator skip
        homing on an unhomed robot (state_machine.py:238-239). A failed persist
        surfaces a warning but does not change the home result (a missed persist →
        next boot re-homes, the SAFE direction). On success, re-apply the cold-start
        config (gains/limits may have been changed in/since a prior session), scoped
        to the homed axes; a configure failure surfaces but does not undo homing.

        SINGLE-FLIGHT GUARDED HERE (the real enforcement point): the cold-start
        ReentrantCallbackGroup lets the /home Trigger service AND the home_motors
        action reach this concurrently, and two _run_home sequences would interleave
        HOME RPCs + race _write_is_homed. Guarding _do_home itself (not just
        _home_action_goal) covers action-vs-action, action-vs-/home, AND /home-vs-
        /home; a concurrent caller gets (False, "home already in progress").
        """
        with self._home_lock:
            if self._home_in_progress:
                return False, "home already in progress"
            self._home_in_progress = True
        try:
            axes = list(self.get_parameter('home_axes').value or [])
            ok, msg = self._run_home(axes)
            w_ok, w_msg = self._write_is_homed(ok)
            if not w_ok:
                self.get_logger().error(
                    f"home: persisting is_homed={int(ok)} failed: {w_msg}")
                msg = f"{msg}; WARNING persist is_homed failed: {w_msg}"
            if ok:
                cfg_ok, cfg_msg = self._run_configure(axes)
                ok = ok and cfg_ok
                msg = f"{msg}; {cfg_msg}"
            return ok, msg
        finally:
            with self._home_lock:
                self._home_in_progress = False

    def _svc_home(self, req, res):
        # the manual /home Trigger service shares _do_home() with
        # the orchestrator's home_motors action — one home+persist+configure body.
        ok, msg = self._do_home()
        res.success = ok
        res.message = msg
        return res

    def _svc_configure(self, req, res):
        # apply the Teensy-side cold-start config (gains + vel/curr limits
        # + POSITION/PASSTHROUGH) to the configure_axes. Run after homing (auto via
        # /home) and again before arming / after activate (TRAP_TRAJ → PASSTHROUGH).
        axes = list(self.get_parameter('configure_axes').value or [])
        ok, msg = self._run_configure(axes)
        res.success = ok
        res.message = msg
        return res

    def _svc_activate(self, req, res):
        # fire the firmware TRAP_TRAJ move to the active pose +
        # observe completion. Scope via activate_axes (default all legs; [0] for
        # the standalone-leg bench rig). Precondition: a prior /configure.
        axes = list(self.get_parameter('activate_axes').value or [])
        ok, msg = self._run_activate(axes)
        res.success = ok
        res.message = msg
        return res

    def _svc_deactivate(self, req, res):
        # fire the firmware TRAP_TRAJ controlled lower to the STOW
        # pose + IDLE on arrival (the Teensy-side analogue of can_node deactivate),
        # then observe completion. Scope via deactivate_axes (default all legs;
        # [0] for the standalone-leg bench rig). Precondition: legs at the active
        # pose (a prior /activate).
        axes = list(self.get_parameter('deactivate_axes').value or [])
        ok, msg = self._run_deactivate(axes)
        res.success = ok
        res.message = msg
        return res

    # ── Hand command surface ──────────────────────────────────────
    # Byte-identical ports of can_node's hand services (can_node.py:782-829,
    # 1626-1661). Jetson-side validation matches can_node exactly (reject invalid
    # before a byte hits CAN3); the RPCs ride the relay-seam axis-6 allow-table
    # (state/gains) or the HAND_TRAJ_CMD 0x6D0 conduit (traj/smooth-move).

    def _svc_set_hand_state(self, req, res):
        # Set the hand ODrive axis state (IDLE / CLOSED_LOOP) directly on axis 6.
        # Reject an UNKNOWN state string Jetson-side: can_node passed the string to
        # encode_set_state (which KeyErrors on a bad key → caught as failure); we do
        # the same map + reject explicitly. SET_AXIS_STATE on axis 6 rides the
        # relay-seam allow-table.
        try:
            state = str(req.data)
            if state not in proto.ODRIVE_STATES:
                res.success = False
                res.message = f"Unknown hand state '{state}'"
                return res
            ok, m, _ = self.teensy_set_axis_state(_HAND_AXIS, proto.ODRIVE_STATES[state])
            res.success = ok
            res.message = f"Hand state set to {state}" if ok else m
        except Exception as e:  # noqa: BLE001
            res.success = False
            res.message = str(e)
        return res

    def _svc_set_hand_traj(self, req, res):
        # Arm a timed catch trajectory (SetHandTrajCmd: event_delay, event_vel,
        # traj_type). Validate exactly as can_node._send_hand_traj_cmd, compute the
        # ABSOLUTE wall_time_ms Jetson-side (now + event_delay), forward the 8-byte
        # 0x6D0 payload via HAND_TRAJ_CMD (the firmware sends the CLOSED_LOOP +
        # POSITION/PASSTHROUGH preamble then forwards it, aborting on a preamble
        # failure). The firmware does NOT re-stamp the deadline — an absolute
        # deadline is immune to Jetson→bridge→CAN3 transit jitter.
        try:
            event_delay = float(req.event_delay)
            event_vel = float(req.event_vel)
            traj_type = int(req.traj_type)
            if event_delay <= 0:
                raise ValueError(f"Invalid event delay: {event_delay}")
            if (event_vel < hw.TEENSY_TRAJ_MIN_EVENT_VEL_MPS
                    or event_vel > hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS):
                raise ValueError(f"Invalid event velocity: {event_vel}")
            if traj_type not in (0, 1, 2):
                raise ValueError(f"Invalid trajectory type: {traj_type}")
            wall_time_ms = int(time.time() * 1000) + int(event_delay * 1000)
            args = rpc_args.encode_hand_traj_cmd(traj_type, event_vel, wall_time_ms)
            ok, m, _ = self.teensy_hand_traj_cmd(args)
            res.success = ok
            res.message = "Hand trajectory set." if ok else m
        except Exception as e:  # noqa: BLE001
            res.success = False
            res.message = str(e)
        return res

    def _svc_smooth_move_hand(self, req, res):
        # Prime the hand to a target rev (SetFloat: data). Validate range exactly as
        # can_node._smooth_move_hand, forward the discriminator-3 0x6D0 payload.
        try:
            target_rev = float(req.data)
            if target_rev < 0 or target_rev > odrive.HAND_MOTOR_MAX_POSITION:
                raise ValueError(f"Invalid target: {target_rev:.3f} rev")
            args = rpc_args.encode_smooth_move_hand(target_rev)
            ok, m, _ = self.teensy_hand_traj_cmd(args)
            res.success = ok
            res.message = f"Smooth-move hand to {target_rev:.3f} rev" if ok else m
        except Exception as e:  # noqa: BLE001
            res.success = False
            res.message = str(e)
        return res

    def _svc_set_hand_gains(self, req, res):
        # Set hand PID gains (SetHandGains: pos_gain, vel_gain, vel_integrator_gain)
        # + remember them (the cold-start hand config/home path reapplies them).
        # Mirrors can_node._svc_set_hand_gains (SET_POS_GAIN then SET_VEL_GAINS on
        # axis 6, both riding the relay-seam allow-table), with ONE intentional
        # hardening: the remembered self._hand_gains is updated only on a successful
        # write (can_node cached unconditionally). This never caches gains that
        # failed to reach the ODrive — otherwise _apply_hand_gains would later
        # re-send stale values as if applied.
        try:
            pos_gain = float(req.pos_gain)
            vel_gain = float(req.vel_gain)
            vel_int_gain = float(req.vel_integrator_gain)
            ok1, m1, _ = self.teensy_set_pos_gain(_HAND_AXIS, pos_gain)
            ok2, m2, _ = self.teensy_set_vel_gains(_HAND_AXIS, vel_gain, vel_int_gain)
            if ok1 and ok2:
                self._hand_gains = {'pos_gain': pos_gain, 'vel_gain': vel_gain,
                                    'vel_int_gain': vel_int_gain}
                res.success = True
                res.message = (f"Hand gains set: pos={pos_gain}, vel={vel_gain}, "
                               f"vel_int={vel_int_gain}")
            else:
                res.success = False
                res.message = "; ".join(m for ok, m in ((ok1, m1), (ok2, m2)) if not ok)
        except Exception as e:  # noqa: BLE001
            res.success = False
            res.message = str(e)
        return res

    def _svc_odrive_command(self, req, res):
        cmd = req.command
        if cmd == 'clear_errors':
            # ONE canonical clear path (review 2026-07-15): route through
            # _svc_clear_errors unconditionally, so this conduit — the path the
            # ORCHESTRATOR's 'clear_errors' actually takes — shares the armed
            # converge-first reroute AND the recover-failed disarm+direct-clear
            # fallback. Before this, the fallback existed only on the Trigger
            # service, and the production command path could dead-end forever in
            # the armed-no-stream state (reseed refused → still latched → repeat).
            # res is duck-type compatible (success/message).
            return self._svc_clear_errors(req, res)
        elif cmd == 'reboot_odrives':
            # Route through the shared hook so this path ALSO clears the cold-start
            # state (cold-start clear) — the orchestrator reboots via odrive_command.
            ok, msg = self._reboot_odrives()
        else:
            ok, msg = False, f'Unknown command: {cmd}'
        res.success = ok
        res.message = msg
        return res

    # ═══════════════════════════════════════════════════════════
    # Orchestrator-facing cold-start conduit
    # ═══════════════════════════════════════════════════════════
    # Thin wrappers exposing the exact (name, type) interfaces the LOCKED
    # orchestrator drives cold-start through, delegating to the bridge's existing
    # verbs. Zero edits to orchestrator_node / state_machine.

    def _home_action_goal(self, goal_request):
        """Fast-reject a concurrent home_motors goal for a clean action REJECT. The
        AUTHORITATIVE single-flight guard lives in _do_home() (so it also covers the
        /home Trigger path); this is only the action-side optimization so a redundant
        goal is rejected up front rather than accepted-then-aborted. PEEK ONLY — it
        does not claim the guard (that happens in _do_home), so a goal that slips past
        this peek is still safely rejected by _do_home. Mirrors bb/throw's guard."""
        with self._home_lock:
            if self._home_in_progress:
                self.get_logger().warn(
                    'home_motors: rejecting goal — a home is already in progress')
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _home_action_execute(self, goal_handle):
        """home_motors action: the orchestrator's HomingHandler drives this
        (ActionClient(HomeMotors, 'home_motors'), orchestrator_node.py:61). Wraps the
        shared _do_home() (single-flight guarded; home legs + hand → persist is_homed →
        configure) and returns HomeMotors.Result(success) — byte-parity with
        can_node._action_home (can_node.py:843-861: home → persist → succeed/abort →
        Result(success)). The firmware HOME-axis-6 support + hand gains landed with
        the hand conduit, so this homes the hand too (parity with _home_robot_steps). A
        concurrent home (guard held) returns (False, …) → the goal aborts."""
        result = HomeMotors.Result()
        try:
            ok, msg = self._do_home()
            result.success = ok
            if ok:
                self.get_logger().info(f"home_motors: {msg}")
                goal_handle.succeed()
            else:
                self.get_logger().error(f"home_motors FAILED: {msg}")
                goal_handle.abort()
            return result
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"home_motors error: {e}")
            result.success = False
            goal_handle.abort()
            return result

    def _svc_activate_or_deactivate(self, req, res):
        """activate_or_deactivate: the orchestrator's ActiveHandler / LevellingHandler
        drive this (ActivateOrDeactivate client, orchestrator_node.py:51-52; command=
        'activate'|'deactivate'). Dispatches to _run_activate / _run_deactivate on the
        configured axes — the analogue of can_node._svc_activate_or_deactivate
        (can_node.py:765-780).

        ACTIVATE folds a _run_configure AFTER the move (parity requirement per the
        can_node<->Teensy parity audit): _run_activate ends the legs in TRAP_TRAJ
        holding the active pose;
        _run_configure switches them to POSITION/PASSTHROUGH so run_mpc.py's 40 Hz
        interp is the sole setpoint source (can_node ended PASSTHROUGH; the bridge's
        _run_activate alone ends TRAP_TRAJ). Order is activate-then-configure —
        configure is safe post-move (legs CLOSED_LOOP holding pose, motion-free; see
        _run_configure). A configure failure fails the result (legs at pose but not
        interp-ready is a real failure for run_mpc), mirroring _svc_home's fold. The
        configure scopes to activate_axes (the legs that moved; the hand is not part of
        ACTIVATE — it is rejected on axis 6)."""
        cmd = str(req.command)
        if cmd == 'activate':
            axes = list(self.get_parameter('activate_axes').value or [])
            ok, msg = self._run_activate(axes)
            if ok:
                cfg_ok, cfg_msg = self._run_configure(axes)
                ok = ok and cfg_ok
                msg = f"{msg}; {cfg_msg}"
            res.success = ok
            res.message = f"Platform activated. {msg}" if ok else msg
        elif cmd == 'deactivate':
            axes = list(self.get_parameter('deactivate_axes').value or [])
            ok, msg = self._run_deactivate(axes)
            res.success = ok
            res.message = f"Platform deactivated. {msg}" if ok else msg
        else:
            res.success = False
            res.message = f"Invalid command: {cmd}"
        return res

    # Tilt-read retry (orchestrator conduit): bounded retries on a failed/out-of-range relay read,
    # porting can_node's tilt robustness (can_node._tilt_reading_steps: resend on
    # timeout, retry on out-of-range, NaN after exhaustion — per the can_node<->Teensy parity audit).
    _TILT_READ_ATTEMPTS = 3

    def _svc_get_platform_tilt(self, req, res):
        """get_platform_tilt: the orchestrator's LevellingHandler drives this
        (GetTiltReadingService client, orchestrator_node.py:57-58; the level_get_tilt
        phase). Reads the Platform-Teensy inclinometer via the relay TILT_READ (relay-seam
        relay_read_tilt) with bounded retry + a validity bound, and returns the
        NaN-on-failure shape the orchestrator already consumes (orchestrator_node.py:
        194-206: NaN in tilt_xy → operation_result=False → LevellingHandler → FAULT).
        Byte-parity with can_node._svc_get_tilt (can_node.py:749-763) + the
        _tilt_reading_steps validity bound (|tilt| ≤ JB_OP_MAX_VALID_TILT_RAD; see the can_node<->Teensy parity audit)."""
        try:
            tx = ty = None
            for _ in range(self._TILT_READ_ATTEMPTS):
                ok, msg, tilt = self.relay_read_tilt()
                if ok and tilt is not None:
                    cand_x, cand_y = float(tilt[0]), float(tilt[1])
                    if (abs(cand_x) <= hw.JB_OP_MAX_VALID_TILT_RAD
                            and abs(cand_y) <= hw.JB_OP_MAX_VALID_TILT_RAD):
                        tx, ty = cand_x, cand_y
                        break
                    self.get_logger().warning(
                        f"tilt read out of range ([{cand_x:.3f}, {cand_y:.3f}] rad) "
                        "— retrying")
                else:
                    self.get_logger().warning(f"tilt read failed: {msg} — retrying")
            if tx is None:
                # Signal failure with NaN so the orchestrator doesn't mistake it for
                # real data (can_node._svc_get_tilt parity).
                res.tilt_xy = [float('nan'), float('nan')]
                res.tilt_quat = Quaternion(w=float('nan'))
                self.get_logger().error("get_platform_tilt: read failed (NaN returned)")
            else:
                res.tilt_xy = [tx, ty]
                res.tilt_quat = _tilt_to_quat(tx, ty)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"get_platform_tilt error: {e}")
            res.tilt_xy = [float('nan'), float('nan')]
            res.tilt_quat = Quaternion(w=float('nan'))
        return res

    def _sub_set_level_state(self, msg):
        """set_level_state: the orchestrator's LevellingHandler publishes
        [levelling_complete_flag, tiltX, tiltY] on the level_persist_state phase
        (orchestrator_node.py:76-77, 323-330). Persist it to the Platform Teensy via
        _write_level_state (relay STATE_WRITE, read-modify-write preserving is_homed).
        Byte-parity with can_node._sub_set_level_state (can_node.py:1052-1064)."""
        try:
            data = list(msg.data)
            if len(data) < 3:
                self.get_logger().warning(
                    f"set_level_state: expected [flag, tiltX, tiltY], got {data}")
                return
            levelling_complete = bool(data[0])
            tilt_x, tilt_y = float(data[1]), float(data[2])
            ok, wmsg = self._write_level_state(levelling_complete, tilt_x, tilt_y)
            if ok:
                self.get_logger().info(
                    f"Level state persisted: complete={levelling_complete}, "
                    f"offset=[{tilt_x:.4f}, {tilt_y:.4f}] rad")
            else:
                self.get_logger().error(f"set_level_state persist failed: {wmsg}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"set_level_state error: {e}")

    # ═══════════════════════════════════════════════════════════
    # Ball Butler (cutover — replaces can_node bb/*)
    # ═══════════════════════════════════════════════════════════

    def _publish_bb_heartbeat(self):
        """Publish bb/heartbeat (10 Hz) from the BB fields on HeartbeatT2J.

        Mirror of can_node._publish_bb_heartbeat: same ROS message, same field
        semantics. ``connected`` is the firmware's bb_present() predicate
        (heartbeat_seen && !heartbeat_stale), so a stale BB shows connected=
        False without the bridge needing its own staleness clock.

        Suppressed until the first HeartbeatT2J arrives so the topic never
        carries a misleading all-zero / state=BOOT snapshot before the link
        is up (mirroring _publish_robot_state's "no phantom snapshot" rule).
        """
        try:
            with self._lock:
                hb = self._latest_heartbeat
            if hb is None:
                return
            seen   = bool(hb.bb_flags & _T2J_BB_FLAG_HEARTBEAT_SEEN)
            stale  = bool(hb.bb_flags & _T2J_BB_FLAG_HEARTBEAT_STALE)
            msg = BallButlerHeartbeat()
            msg.connected    = bool(seen and not stale)
            msg.ball_in_hand = bool(hb.bb_flags & _T2J_BB_FLAG_BALL_IN_HAND)
            msg.state        = int(hb.bb_state)
            msg.state_data   = int(hb.bb_state_data)
            msg.yaw_deg      = float(hb.bb_yaw_deg)
            msg.pitch_deg    = float(hb.bb_pitch_deg)
            msg.hand_pos_mm  = float(hb.bb_hand_mm)
            self.bb_heartbeat_pub.publish(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"BB heartbeat publish error: {e}",
                                    throttle_duration_sec=5.0)

    def _bb_present(self) -> bool:
        """Local view of the firmware's bb_present() — used by bb/calibrate's
        skip-if-absent semantics. False before the first HeartbeatT2J or
        whenever BB has gone stale."""
        with self._lock:
            hb = self._latest_heartbeat
        if hb is None:
            return False
        return (bool(hb.bb_flags & _T2J_BB_FLAG_HEARTBEAT_SEEN)
                and not bool(hb.bb_flags & _T2J_BB_FLAG_HEARTBEAT_STALE))

    # ═══════════════════════════════════════════════════════════
    # Catching cone (cone uplink — mirrors can_node's
    # _handle_catch_event / _publish_cone_heartbeat field-by-field)
    # ═══════════════════════════════════════════════════════════

    def _drain_cone_catch_events(self):
        """Publish every queued catch event on cone/catch_event (100 Hz drain).

        Timestamp semantics mirror can_node._handle_catch_event: when the cone
        is time-synced, the frame's low-32 µs timestamp (latched in the cone's
        piezo ISR) is reconstructed into a full wall-clock instant against the
        host's clock at UDP arrival; when NOT synced the cone's counter is
        local-only and not comparable to wall time — fall back to host time so
        header.stamp / catch_time are at least a valid recent instant.
        Consumers must still check time_synced before treating catch_time as
        impact-truth.
        """
        with self._lock:
            if not self._cone_catch_queue:
                return
            queued = self._cone_catch_queue
            self._cone_catch_queue = []
        for evt, arrival_us in queued:
            try:
                msg = CatchEvent()
                msg.header.frame_id = 'catching_cone'
                if evt.time_synced:
                    catch_us = catching_cone.reconstruct_catch_time_us(
                        evt.catch_time_us_low32, arrival_us)
                    # Defensive: a host-clock jump (NTP step, manual `date`)
                    # between the last time-sync and this decode can place the
                    # reconstructed high bits in the wrong wrap window. Catches
                    # are necessarily within ms of UDP arrival; flag anything
                    # beyond ~1 s as a likely artefact.
                    delta_us = abs(catch_us - arrival_us)
                    if delta_us > 1_000_000:
                        self.get_logger().warning(
                            f"Catch seq {evt.sequence}: reconstructed time "
                            f"{delta_us / 1e6:.2f} s from host now — possible "
                            "host clock jump or stale time-sync.",
                            throttle_duration_sec=5.0)
                    stamp = rclpy.time.Time(nanoseconds=catch_us * 1000).to_msg()
                    msg.header.stamp = stamp
                    msg.catch_time = stamp
                else:
                    host_stamp = self.get_clock().now().to_msg()
                    msg.header.stamp = host_stamp
                    msg.catch_time = host_stamp
                msg.sequence = evt.sequence
                msg.time_synced = evt.time_synced
                msg.retrigger_suppressed = evt.retrigger_suppressed
                self.catch_event_pub.publish(msg)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"Catch event publish error: {e}")

    def _publish_cone_heartbeat(self):
        """Publish cone/heartbeat (10 Hz) from the latest relayed CONE_HEARTBEAT.

        Mirror of can_node._publish_cone_heartbeat: ``connected`` requires a
        cone heartbeat within CC_HEARTBEAT_TIMEOUT_MS, measured on the host
        monotonic clock from UDP arrival. Publishes connected=False defaults
        before the first heartbeat (matching can_node, whose consumers rely on
        the topic being alive to display "cone disconnected").
        """
        try:
            with self._lock:
                hb = self._latest_cone_hb
                received = self._cone_hb_received
                last_mono = self._cone_last_hb_mono
            connected = (received
                         and (time.monotonic() - last_mono) < self._cone_hb_timeout_s)
            msg = CatchingConeHeartbeat()
            msg.connected = connected
            msg.state = int(hb.state)
            msg.state_data = hb.state_data
            msg.sync_rms_us = hb.sync_rms_us
            msg.last_catch_seq = hb.last_catch_seq
            msg.ms_since_last_catch = hb.ms_since_last_catch
            msg.time_synced = hb.time_synced
            msg.have_any_catch = hb.have_any_catch
            self.cone_heartbeat_pub.publish(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Cone heartbeat publish error: {e}")

    # Extra wait (s) past throw_time before the action times out a result. Covers
    # the wind-up + release + the streamer completing and the firmware emitting OK.
    _BB_THROW_RESULT_MARGIN_S = 5.0

    @staticmethod
    def _bb_outcome_text(outcome, detail0, detail1):
        """Human-readable CMD_RESULT outcome (name + decoded details)."""
        try:
            name = proto.BallButlerCommandOutcome(int(outcome)).name
        except ValueError:
            name = f"UNKNOWN(0x{int(outcome):02x})"
        axis = {0: 'YAW', 1: 'PITCH', 2: 'BOTH'}.get(int(detail0), 'n/a')
        return f"{name} (axis={axis}, detail1={int(detail1)})"

    def _bb_throw_goal(self, goal_request):
        """Accept a throw goal only if none is outstanding (firmware is serialized).

        A speed==0 goal is an aim/track command (the firmware routes it to
        requestTracking — no throw, no CMD_RESULT); it does NOT consume the
        single-throw slot, so it is always accepted and completes immediately on
        dispatch (see execute_callback). Only real throws (speed>0) are gated.

        Runs in the action's ReentrantCallbackGroup, so it can reject a second
        throw promptly while execute_callback is still awaiting the first result.
        """
        if float(goal_request.throw_speed) == 0.0:
            return GoalResponse.ACCEPT
        with self._bb_throw_lock:
            if self._bb_throw_active:
                self.get_logger().warn(
                    'bb/throw: rejecting goal — a throw is already outstanding')
                return GoalResponse.REJECT
            self._bb_throw_active = True
            self._bb_throw_result = None
            self._bb_throw_event.clear()
        return GoalResponse.ACCEPT

    def _bb_throw_cancel(self, goal_handle):
        """Reject cancellation — a throw in flight has no firmware abort path."""
        return CancelResponse.REJECT

    def _bb_throw_execute(self, goal_handle):
        """Dispatch the throw and await the firmware's terminal CMD_RESULT.

        The BB_THROW RPC acks at the can-bridge (frame queued to CAN1), NOT at BB,
        so success/reject is learned only from the relayed CMD_RESULT — set on the
        RX thread by _on_cmd_result, which wakes the _bb_throw_event we wait on here.
        Times out (rather than hanging) if BB never answers.
        """
        req = goal_handle.request
        result = BallButlerThrowCmd.Result()

        # Aim/track command (speed 0): the firmware routes it to requestTracking —
        # no throw, no CMD_RESULT to await — so dispatch and complete immediately,
        # matching the retired service's fire-and-forget tracking behaviour. Does
        # not touch the single-throw slot (see goal_callback).
        if float(req.throw_speed) == 0.0:
            try:
                args = rpc_args.encode_bb_throw(
                    req.yaw_angle_rad, req.pitch_angle_rad, 0.0, req.throw_time)
            except Exception as e:  # noqa: BLE001
                result.success = False
                result.outcome = int(proto.BallButlerCommandOutcome.REJECTED)
                result.message = f"Aim arg encode failed: {e}"
                goal_handle.abort()
                return result
            ok, m, _ = self._call_rpc(RpcMethod.BB_THROW, args)
            result.success = ok
            result.outcome = int(proto.BallButlerCommandOutcome.OK if ok
                                 else proto.BallButlerCommandOutcome.REJECTED)
            result.message = ("Aim/track command dispatched (no throw)." if ok
                              else f"Aim dispatch failed (BB unreachable?): {m}")
            if ok:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result

        try:
            try:
                args = rpc_args.encode_bb_throw(
                    req.yaw_angle_rad, req.pitch_angle_rad,
                    req.throw_speed, req.throw_time)
            except Exception as e:  # noqa: BLE001 (Python-side range check failure)
                result.success = False
                result.outcome = int(proto.BallButlerCommandOutcome.REJECTED)
                result.message = f"Throw arg encode failed: {e}"
                goal_handle.abort()
                return result

            # Dispatch. A failed RPC means the bridge couldn't queue the frame
            # (BB not present) — no CMD_RESULT will ever come, so abort now.
            ok, m, _ = self._call_rpc(RpcMethod.BB_THROW, args)
            if not ok:
                result.success = False
                result.outcome = int(proto.BallButlerCommandOutcome.REJECTED)
                result.message = f"Throw dispatch failed (BB unreachable?): {m}"
                goal_handle.abort()
                return result

            # Await the firmware's terminal outcome (set on the RX thread).
            timeout_s = max(float(req.throw_time), 0.0) + self._BB_THROW_RESULT_MARGIN_S
            if not self._bb_throw_event.wait(timeout=timeout_s):
                result.success = False
                result.outcome = int(proto.BallButlerCommandOutcome.TIMEOUT)
                result.message = (
                    f"No CMD_RESULT within {timeout_s:.1f}s — firmware silent "
                    f"(BB detached, or a terminal point missed an emit).")
                self.get_logger().warn(f'bb/throw: {result.message}')
                goal_handle.abort()
                return result

            with self._bb_throw_lock:
                outcome, detail0, detail1 = self._bb_throw_result
            result.outcome = int(outcome)
            result.detail0 = int(detail0)
            result.detail1 = int(detail1)
            result.message = self._bb_outcome_text(outcome, detail0, detail1)
            if int(outcome) == int(proto.BallButlerCommandOutcome.OK):
                result.success = True
                self.get_logger().info(f'bb/throw: {result.message}')
                goal_handle.succeed()
            else:
                result.success = False
                self.get_logger().warn(f'bb/throw: {result.message}')
                goal_handle.abort()
            return result
        finally:
            with self._bb_throw_lock:
                self._bb_throw_active = False

    def _svc_bb_reload(self, req, res):
        """bb/reload — BB_RELOAD RPC. ERR_BUS_DOWN surfaces as failure (the
        operator wants to know if BB isn't there for a reload attempt)."""
        ok, m, _ = self._call_rpc(RpcMethod.BB_RELOAD, b"")
        res.success = ok
        res.message = "Reload command sent." if ok else f"Reload failed: {m}"
        return res

    def _svc_bb_reset(self, req, res):
        """bb/reset — BB_RESET RPC. ERR_BUS_DOWN surfaces as failure."""
        ok, m, _ = self._call_rpc(RpcMethod.BB_RESET, b"")
        res.success = ok
        res.message = "Reset command sent." if ok else f"Reset failed: {m}"
        return res

    def _svc_bb_calibrate(self, req, res):
        """bb/calibrate — BB_CALIBRATE_LOC RPC. If BB is not present, return
        success=True silently (mirror can_node._svc_bb_calibrate). The state
        machine's HOMING phase uses this service; the silent-skip lets
        homing complete without BB attached (the original can_node
        behaviour). When BB IS present we forward the RPC's status.
        """
        if not self._bb_present():
            res.success = True
            res.message = "No BB heartbeat received — calibration skipped"
            return res
        ok, m, _ = self._call_rpc(RpcMethod.BB_CALIBRATE_LOC, b"")
        res.success = ok
        res.message = "Calibrate command sent." if ok else f"Calibrate failed: {m}"
        return res

    def _sub_vel_curr_limits(self, msg):
        """Apply leg AND hand vel/current limits over the can-bridge link.

        Mirrors can_node._sub_vel_curr_limits. The can-bridge owns CAN3 — legs 0-5
        AND the hand ODrive (axis 6) — so it applies the HAND limits too (parity
        hardening): the earlier "hand limits ignored" note was a can_node parity
        regression, false since the relay-seam axis-6 allow-table (the node already sends
        SET_VEL_CURR_LIMITS to axis 6 in _run_configure). On a successful hand write we
        also update the cached self._hand_vel_limit/_hand_curr_limit so a later
        _run_configure re-applies the operator's update, not the config default
        (parity with the self._hand_gains cache). Short-timeout RPCs keep a topic
        callback from stalling the executor on a dead link.
        """
        if msg.legs_vel_limit > 0 and msg.legs_curr_limit > 0:
            for axis in range(p.NUM_LEGS):
                ok, m, _ = self.teensy_set_vel_curr_limits(
                    axis, msg.legs_vel_limit, msg.legs_curr_limit,
                    timeout=0.2, retries=0)
                if not ok:
                    self.get_logger().error(
                        f"set_vel_curr_limits leg {axis} failed: {m}",
                        throttle_duration_sec=2.0)
                    break
        # Hand (axis 6): the can-bridge owns it on CAN3 (relay-seam allow-table).
        if msg.hand_vel_limit > 0 and msg.hand_curr_limit > 0:
            ok, m, _ = self.teensy_set_vel_curr_limits(
                _HAND_AXIS, msg.hand_vel_limit, msg.hand_curr_limit,
                timeout=0.2, retries=0)
            if ok:
                self._hand_vel_limit = float(msg.hand_vel_limit)
                self._hand_curr_limit = float(msg.hand_curr_limit)
            else:
                self.get_logger().error(
                    f"set_vel_curr_limits hand failed: {m}",
                    throttle_duration_sec=2.0)

    # ═══════════════════════════════════════════════════════════
    # Shutdown
    # ═══════════════════════════════════════════════════════════

    def _shutdown_stow(self):
        """Profiled controlled-lower to STOW + IDLE on a clean shutdown — the Teensy-side
        analogue of can_node.on_shutdown's ``_gently_move_to_setpoint(0.0,
        deactivating=True)`` (can_node.py:1693-1706). Best-effort and self-bounded:
        never raises and never hangs teardown. Guard *shape* follows can_node (skip
        when the bus is undrivable, else stow), scoped to the CAN3 bus-off /
        no-telemetry / stow-pending subset — a broader fatal-CAN (link-lost /
        CAN_BUS_DOWN) only makes the DEACTIVATE RPC fail fast (bounded by its
        timeout×retry budget, never a hang):
          * CAN3 core bus down / no telemetry → skip (can't drive a dead bus; the
            firmware's CAN3-loss deferred stow safes the platform on reconnect);
          * a deferred stow already pending → skip (the firmware completes it);
          * otherwise DEACTIVATE the configured legs (TRAP_TRAJ lower to STOW then
            IDLE; ``_run_deactivate`` idles the hand too), bounded by
            DeactivateMonitor's internal hard deadline so teardown can't hang.
        A leg that was never activated (not in CLOSED_LOOP) is rejected immediately
        by the firmware → clean no-op, no descent, no hang.
        """
        try:
            hb = self._latest_heartbeat
            core_bus = int(hb.bus1_health) if hb is not None else None   # CAN3
            if hb is None or core_bus == int(BusHealth.BUS_OFF):
                self.get_logger().warning(
                    "shutdown stow skipped — CAN3 core bus down/unseen; leaving the "
                    "firmware deferred-stow to safe the platform on reconnect.")
                return
            if int(hb.flags) & _T2J_FLAG_STOW_PENDING:
                self.get_logger().info(
                    "shutdown stow skipped — a deferred stow is already pending; the "
                    "firmware will complete it.")
                return
            axes = [int(a) for a in self.get_parameter('deactivate_axes').value]
            self.get_logger().info(
                f"shutdown: profiled stow (DEACTIVATE) on legs {axes} before teardown")
            ok, msg = self._run_deactivate(
                axes, timeout_s=_SHUTDOWN_DEACTIVATE_TIMEOUT_S)
            if ok:
                self.get_logger().info(f"shutdown stow complete — {msg}")
            else:
                self.get_logger().error(f"shutdown stow did not complete — {msg}")
                # If a guard E-STOP is latched, DEACTIVATE is IMPOSSIBLE (the firmware
                # bounces it ERR_BUS_DOWN) — leave the operator a loud, unmissable
                # final message rather than a quiet warning that scrolls away.
                self._warn_disarmed_but_standing_if_latched()
        except Exception as e:  # noqa: BLE001 — teardown must never raise
            self.get_logger().error(f"shutdown stow error (continuing teardown): {e}")

    def _warn_disarmed_but_standing_if_latched(self):
        """Loud final message when a latched guard made the shutdown DEACTIVATE
        impossible: the robot is DISARMED (mpc_active=0, no setpoint output — it will
        not move) but STILL STANDING at the active pose because the profiled stow
        could not run. A latched guard only clears with CLEAR_ERRORS, so tell the
        operator exactly that. No-op when no guard is latched (fault_state=NONE) — a
        non-guard stow failure (e.g. a stalled leg) gets the generic error above."""
        hb = self._latest_heartbeat
        fault = int(hb.fault_state) if hb is not None else int(FaultState.NONE)
        if fault == int(FaultState.NONE):
            return
        name = _enum_name(FaultState, fault)
        self.get_logger().error(
            "SHUTDOWN: ROBOT DISARMED BUT STILL STANDING — a latched Teensy guard "
            f"(fault_state={name}) made the profiled DEACTIVATE impossible. "
            "mpc_active is 0 (no setpoint output) so the platform will NOT move, but "
            "it is NOT stowed. CLEAR_ERRORS is required to release the guard: run "
            "'ros2 service call /clear_errors std_srvs/srv/Trigger' then re-run "
            "deactivate, OR lower the legs manually before removing power.")

    def on_shutdown(self):
        """Ordered, bounded, best-effort safe-shutdown, THEN transport teardown.

        Task 3.3: a ``ros2 launch`` Ctrl-C must ALWAYS profiled-stow the robot — no
        matter what mode it was in (armed / streaming / already idle).

        WHY THIS SEQUENCE LIVES HERE (not the orchestrator, not a launch
        OnShutdown): on a launch Ctrl-C, launch broadcasts SIGINT to EVERY node
        process at once and each runs its own ``main()`` finally. A cross-process
        orchestration (orchestrator → bridge service call) would race THIS node's
        executor teardown — an rclpy service call from a shutdown hook on a dying
        executor is the classic deadlock — and a launch OnShutdown handler runs in
        the launch process, which has no link to the robot at all. The bridge is the
        ONE process that owns BOTH the disarm (``_stop_setpoint_output``) and the
        profiled stow (``_run_deactivate``) as IN-PROCESS methods, and its RX +
        heartbeat daemon threads keep the UDP link alive through this teardown — so
        the sequence runs deterministically with NO dependency on any other
        process's executor. Guarantee: it runs to completion (or its bounded
        timeout) as long as this process reaches its ``finally``. Limit: if launch
        SIGKILLs the process before the DEACTIVATE completes, the firmware's own
        CAN3-loss deferred stow is the backstop (the descent is a firmware-side
        TRAP_TRAJ move that continues autonomously even if the link dies mid-way).

        Sequence — each step best-effort (loud on failure, continue), total ≲ 8 s:
          1. DISARM — ``_stop_setpoint_output`` stops the 40 Hz setpoint thread THEN
             drops mpc_active=0 on the WIRE (the in-process 'set_setpoint_output
             false'). Done FIRST so mpc_active reaches 0 before the emitter-stop
             stream silence can latch MPC_STALE (trajectory_node Sharp Edge #1).
          2. SETTLE — a bounded couple of heartbeat periods so the flags=0 J→T
             heartbeat is transmitted + registered before the DEACTIVATE RPC (the
             firmware rejects DEACTIVATE while mpc_active=1).
          3. STOW — profiled TRAP_TRAJ lower to STOW + IDLE (bounded), guarded for a
             dead bus / already-pending deferred stow; a latched guard makes this
             impossible → loud 'disarmed but standing, CLEAR_ERRORS required'.
          4. Transport teardown (only stops the client if we created it).
        """
        self.get_logger().info("Shutting down TeensyBridgeNode...")
        # 1. DISARM. _stop_setpoint_output stops the setpoint thread then drops
        # mpc_active on the WIRE (not just the local flag) — even for an injected
        # client whose heartbeat thread keeps running after on_shutdown. It is fully
        # internally guarded; wrap it so teardown never raises, and fall back to the
        # raw flag drop so mpc_active=0 is guaranteed on any path.
        try:
            self._stop_setpoint_output()
        except Exception as e:  # noqa: BLE001 — best-effort during teardown
            self.get_logger().error(
                f"shutdown disarm error (continuing to stow): {e}")
            try:
                self._set_mpc_active(False)
            except Exception:  # noqa: BLE001
                self._mpc_active = False
        # 2. SETTLE + 3. STOW over the still-live link (RX + heartbeat threads keep
        # the telemetry cache + DEACTIVATE RPC alive; MPC is quiesced above so no
        # setpoint fights the descent). The settle is gated on actually stowing so a
        # stow-disabled node (unit tests) tears down instantly.
        if self._stow_on_shutdown:
            # Prefer the firmware's own arm-took confirmation (T2J bit3) over a
            # fixed settle — the same staging race as A3: a delayed heartbeat
            # tick means a fixed sleep can still race s_mpc_active=1 (audit
            # 2026-07-15). Returns immediately when no heartbeat exists, so a
            # dead-link teardown cannot hang; the fixed settle remains as the
            # floor for the flags=0 heartbeat itself to go out.
            time.sleep(_SHUTDOWN_DISARM_SETTLE_S)
            self._wait_wire_disarmed()
            self._shutdown_stow()
        # 4. Transport teardown.
        try:
            self._tod.close()
            self._rpc_server.close()
            self._rpc.close()
        except Exception:  # noqa: BLE001
            pass
        if self._own_client:
            try:
                self._client.stop()
            except Exception:  # noqa: BLE001
                pass


def main(args=None):
    rclpy.init(args=args)
    node = TeensyBridgeNode()
    # MultiThreadedExecutor so the bb/throw action's result-wait (which can block
    # an executor thread for up to throw_time + margin in execute_callback) never
    # stalls the 100 Hz telemetry / heartbeat timers — they run on other threads.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
