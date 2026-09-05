#!/usr/bin/env python3
"""Bench driver: the SUB-CYCLE unified rungs (UH-3 carry, UH-5 throw), launch UP.

Drives the two rungs of the unified-7dof Phase 5 hardware ladder
(``tests/hardware/session_unified7_cycle_ladder.md``) that the shipped
``TossContinuous`` session cannot express, because that session only ever
installs a joined ``LAUNCH + LANDING`` and has no way to ask for a bare carry or
for a throw with no catch. This tool asks ``trajectory_node`` for exactly one
``PlanCycle`` window (two, for the throw's chain) and then watches.

  # UH-3 — banked carry with a SEATED BALL, 60 mm in +x, 1.4 s, and back
  python3 tests/hardware/unified_cycle_bench.py --rung carry --dx 60

  # UH-5 — planned throw at a 0.5 m apex, NO catch (the cup settles empty)
  python3 tests/hardware/unified_cycle_bench.py --rung throw --apex 0.5

  # print the request and the derived numbers; ZERO ROS calls, no robot
  python3 tests/hardware/unified_cycle_bench.py --rung throw --dry-run

Runs under the **system python3.8 with ROS 2 sourced** — ``source
/opt/ros/foxy/setup.bash`` then ``source ros_ws/install/setup.bash`` — like every
other rclpy tool in this directory, and NOT the project venv. The pure core
(request construction, the precondition evaluator, the verdict evaluator)
imports no ROS and is unit-tested in ``tests/ros/test_unified_cycle_bench.py``;
``rclpy`` and the interface package are imported inside :func:`run` only.

WHAT THIS IS NOT
----------------
**It is not the safety layer and it owns no E-stop.** The safety layer on this
path is (a) the operator's E-stop, in hand, and (b) the can-bridge's own guards
on the streamed hand lane — ``MAX_DEVIATION_HAND_REV`` 2.5 rev **armed** for
Phase 5 from UH-3 on (owner, 2026-09-05: ``hand7 arm`` on the Teensy console
joins the session-start checklist, and a trip is DATA — a fail-safe E-STOP
released by ``CLEAR_ERRORS``), ``MAX_LEAD_HAND_REV`` 2.0, the stroke clip
[0, 10.8] rev and the 345 rev/s overspeed E-STOP. This driver adds exactly one
safety behaviour of its own and it is a REFUSAL: it will not issue a request
unless the preconditions below hold. Once a plan is installed, stopping it is
the operator's E-stop or ``trajectory/go_home`` — not this process.

**It never opens the UDP link.** ``hand_stream_bench.py`` is the sole-owner
raw-UDP driver and requires the launch DOWN; this one is an ordinary ROS client
and requires the launch UP. Running them together would put two writers on one
link (`project_hardware_bench_facts`), so they are never both running.

**It never arms, never changes control mode, never sets limits, never switches
`hand_source`.** All four are the operator's, per
``ros_ws/docs/ARMING_CONTRACT.md`` and the Phase 4 latch finding (the firmware
refuses a ``hand_source`` switch while the setpoint output is armed, so a
session — and this driver — can only VERIFY it). Same request-only posture as
``traj_ramp_battery.py`` and ``tilt_cal_grid.py``.

PRECONDITIONS (all five are checked every run; a failure REFUSES and prints the
exact command that fixes it):

  P1  ``trajectory/status`` seen inside ``--max-age`` and reporting
      ``mode == TRAJECTORY`` with ``streaming`` true.
  P2  ``/robot_state`` seen inside ``--max-age`` — the same telemetry
      ``trajectory_node``'s own ``_robot_state_fresh`` gate reads, so a stale
      one earns ``STALE_STATE`` from the service anyway; refusing here names it.
  P3  the HAND is ENERGISED: ``/robot_state`` ``motor_states[6].current_state``
      == 8 (CLOSED_LOOP). An IDLE hand ignores every ``set_input_pos`` the
      streamed lane puts on the wire and looks EXACTLY like a perfect hold —
      the 2026-09-03 first-sitting failure. With the launch UP the bridge's own
      arming preamble brings axis 6 to CLOSED_LOOP + POSITION/PASSTHROUGH at
      ACTIVATE whenever the latch reads STREAMED (``teensy_bridge_node.py``
      ~:4205), so this check is normally a confirmation that ACTIVATE took.
  P4  the ``hand_source`` latch reads ``STREAMED`` on ``/link_status``
      (``KeyValue`` key ``hand_source``, values ``STREAMED`` /
      ``LEGACY_STROKE``, sourced from HeartbeatT2J flags bit 6). While LEGACY
      the firmware DISCARDS Setpoint index 6 — counted, but silent to the plan —
      so the platform would fly the whole window with a dead hand and a seated
      ball.
  P5  the session leg limits are the CATCH-CAPABLE set 250 / 3000 / 150000,
      read back off ``trajectory/status``. At the shipped 1000/5000/30000 every
      unified cycle reads ``LIMIT_JERK`` for a structural reason (the z-launch
      jerk leaks ``sin(tilt) x 744.3 mm`` into centroid xy), so this is a
      refusal with the ``ros2 service call`` line rather than a mystery
      rejection three seconds later.

OUTPUT
------
A per-tick CSV plus a ``_meta.json`` under ``temp/logs/``, and a PASS/FAIL/SKIP
verdict block on stdout. Exit 0 iff no FAIL. The CSV is what the sitting sends
back with the console log and the bag.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from datetime import datetime

_HERE = os.path.dirname(os.path.abspath(__file__))              # tests/hardware
_REPO = os.path.dirname(os.path.dirname(_HERE))                 # repo root
for _p in (_HERE, _REPO, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_REPO, 'config', 'generated')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

TOOL_NAME = 'tests/hardware/unified_cycle_bench.py'

# ─────────────────────────────────────────────────────────────────────────────
# Constants mirrored from elsewhere, each naming its source
# ─────────────────────────────────────────────────────────────────────────────

#: ``PlanCycle.srv``'s enums. Mirrored rather than imported so the pure core
#: stays importable without a colcon build; pinned to the .srv text by
#: ``tests/ros/test_unified_cycle_bench.py::test_the_mode_and_kind_enums_match_the_srv``.
MODE_NEW, MODE_EXTEND, MODE_REPLAN = 0, 1, 2
KIND_LAUNCH, KIND_STEADY, KIND_LANDING, KIND_SETTLE = 0, 1, 2, 3

#: The catch-capable session limits (leg vel mm/s, acc mm/s^2, jerk mm/s^3).
#: ``sim/cycle_gate.py`` and ``sim/unified_gate.py`` plan at these; plan § 4
#: owner decision 1 makes "unified mode rides session limits raised at the
#: sitting" the operating rule. The shipped launch defaults are
#: 1000 / 5000 / 30000, so this is a jerk RAISE and a velocity REDUCTION.
SESSION_LEG_VEL_MMPS = 250.0
SESSION_LEG_ACC_MMPS2 = 3000.0
SESSION_LEG_JERK_MMPS3 = 150000.0

SET_LIMITS_CMD = (
    'ros2 service call /trajectory/set_limits '
    'jugglebot_interfaces/srv/SetTrajectoryLimits '
    '"{leg_vel_limit_mmps: %.1f, leg_acc_limit_mmps2: %.1f, '
    'leg_jerk_limit_mmps3: %.1f}"'
    % (SESSION_LEG_VEL_MMPS, SESSION_LEG_ACC_MMPS2, SESSION_LEG_JERK_MMPS3))

#: Cup-opening heights the throw rung uses, mirroring
#: ``reload_coordinator_node._UNIFIED_THROW_CUP_Z_MM`` / ``_UNIFIED_CATCH_CUP_Z_MM``.
#: Deliberately module literals there ("they exist to be moved by UH-5/UH-6"),
#: so they are literals here too and are exposed as ``--throw-z`` / ``--catch-z``.
THROW_CUP_Z_MM = 860.0
CATCH_CUP_Z_MM = 830.0

#: Hand axis and its energised state. ``teensy_bridge_node._HAND_AXIS`` (= 6)
#: and ``protocol_config.ODRIVE_STATES['CLOSED_LOOP']`` (= 8).
HAND_AXIS = 6
AXIS_STATE_CLOSED_LOOP = 8

#: can-bridge FW 17 hand guards — ``canbridge_config.h``. Mirrored as literals
#: with the name beside them, the convention ``hand_stream_bench.py`` uses
#: (a C++ header is not importable and a second generator is not worth it);
#: pinned to the header by ``test_the_firmware_hand_guards_match_the_header``.
MAX_DEVIATION_HAND_REV = 2.5      # canbridge_config.h MAX_DEVIATION_HAND_REV
MAX_LEAD_HAND_REV = 2.0           # canbridge_config.h MAX_LEAD_HAND_REV
HAND_MOTOR_MAX_POSITION_REV = 10.8  # canbridge_config.h HAND_MOTOR_MAX_POSITION
                                    # (= GEOM_HAND_MOTOR_HARD_STOP_REVS)

#: The PLANNER's own slider ceiling — ``JB_OP_HAND_CATCH_PRIME_REV``, the top of
#: the operating band ``validate_cycle`` refuses outside with ``HAND_STROKE``.
#: Reported (never gated) beside the commanded peak so the operator can see how
#: much of the band a tier is using before the next one is asked for.
HAND_CATCH_PRIME_REV = 9.9594     # hardware_config JB_OP_HAND_CATCH_PRIME_REV

#: Slider revs per mm — ``LINEAR_GAIN_REV_PER_M`` / 1000, used only to print a
#: margin in millimetres beside one in revs.
REV_PER_MM = 31.65 / 1000.0

#: How much clearance the ENCODER must keep from the hard stop (rev). Sitting
#: two measured 10.4693 rev on a legacy stroke — 0.33 rev, ~10 mm — so the
#: default is set just below what the machine has actually been seen to reach,
#: and is a knob (``--metal-margin``) rather than a constant because the tier
#: ramp is what moves it.
DEFAULT_METAL_MARGIN_REV = 0.20

#: Gravity, mm/s^2 — ``hardware_config.GRAVITY_MMPS2``. Restated rather than
#: imported so the flight arithmetic works in ``--dry-run`` on a box with no
#: generated config; pinned by the offline test.
G_MM_S2 = 9806.0

#: Defaults. ``--period`` 1.4 s is the LANDING window the sim gate found
#: load-bearing (at 1.0 s the 50 mm ring refuses ``LIMIT_JERK`` at 155-198 k),
#: and it is what ``sim/unified_gate.py`` carries a carry at.
DEFAULT_PERIOD_S = 1.4
#: The LAUNCH window, ``reload_coordinator_node._UNIFIED_LAUNCH_WINDOW_S``.
DEFAULT_LAUNCH_WINDOW_S = 0.6
#: The chained SETTLE that makes the throw install REST-TERMINAL. Same 1.4 s.
DEFAULT_CHAIN_PERIOD_S = 1.4
#: 0.5 m apex — the "low tier, h <= 0.5 m" of T-H6/UH-5.
DEFAULT_APEX_M = 0.5


class BenchError(RuntimeError):
    """A refusal this tool minted itself (never a planner refusal)."""


# ─────────────────────────────────────────────────────────────────────────────
# Pure core — arithmetic
# ─────────────────────────────────────────────────────────────────────────────

def flight_for_apex_s(apex_m: float) -> float:
    """Time of flight (s) for a vertical self-toss to ``apex_m`` above release.

    ``T = sqrt(8h/g)`` — up and down, the SAME conversion
    ``TossContinuous.action``'s ``throw_height_m`` documents ("Converted once to
    flight time T = sqrt(8*h/g)"), restated here so the two tiers are the same
    number and a rung can be compared against a legacy toss at the same height.
    0.5 m => 0.6387 s.
    """
    if not apex_m > 0.0:
        raise BenchError('apex must be > 0 m, got %r' % (apex_m,))
    return math.sqrt(8.0 * float(apex_m) * 1000.0 / G_MM_S2)


def release_speed_for_flight_mm_s(flight_s: float) -> float:
    """Vertical take-off speed (mm/s) a flight of ``flight_s`` implies: ``gT/2``.

    Printed beside the plan's OWN ``release_vel_mm_s`` so the operator can see
    the QP agreeing with the ballistics before the ball leaves — the two are
    different computations of the same quantity and a disagreement is a finding.
    """
    return 0.5 * G_MM_S2 * float(flight_s)


# ─────────────────────────────────────────────────────────────────────────────
# Pure core — request construction
# ─────────────────────────────────────────────────────────────────────────────

def carry_request(cup_mm, *, dx_mm: float, dy_mm: float,
                  period_s: float = DEFAULT_PERIOD_S,
                  banking: bool = True) -> dict:
    """UH-3: ``MODE_NEW`` + ``KIND_SETTLE`` — a pure lateral re-pose of the CUP.

    ``cup_mm`` is the CURRENT cup opening (mm; xy platform-frame, z GLOBAL) —
    the forward map of the live pose and hand, :func:`cup_site_now` on the ROS
    side. The settle site is that point displaced in xy at the SAME z, so the
    ball is carried, not lifted.

    A ``SETTLE`` is the only rest-terminal kind, which is what makes it the
    carry: ``LAUNCH`` ends at a release and ``STEADY``/``LANDING`` need a catch
    this rung has no ball in the air for. The throw and catch fields are INERT
    for this kind (``unified_cycle._events_for`` reads neither) and are sent as
    zeros; a zero ``catch_vel_mm_s`` maps to a LEVEL receive tilt
    (``tilt_geometry.tilt_to_receive`` returns ``(0, 0)`` below 1e-9), so the
    zeros are a real no-op rather than a value that happens not to be read.
    """
    x, y, z = (float(cup_mm[0]), float(cup_mm[1]), float(cup_mm[2]))
    if not period_s > 0.0:
        raise BenchError('--period must be > 0 s, got %r' % (period_s,))
    return {
        'mode': MODE_NEW,
        'kind': KIND_SETTLE,
        'period_s': float(period_s),
        'throw_site_mm': [0.0, 0.0, 0.0],
        'throw_target_mm': [0.0, 0.0, 0.0],
        'flight_s': 0.0,
        'catch_site_mm': [0.0, 0.0, 0.0],
        'catch_vel_mm_s': [0.0, 0.0, 0.0],
        'catch_frac': 0.0,
        'settle_site_mm': [x + float(dx_mm), y + float(dy_mm), z],
        'banking_enabled': bool(banking),
        'chain': False,
        'chain_kind': 0,
        'chain_period_s': 0.0,
        'chain_catch_frac': 0.0,
        'lead_s': 0.0,
    }


def throw_request(cup_mm, *, flight_s: float,
                  period_s: float = DEFAULT_LAUNCH_WINDOW_S,
                  chain_period_s: float = DEFAULT_CHAIN_PERIOD_S,
                  throw_z_mm: float = THROW_CUP_Z_MM,
                  settle_z_mm: float = None,
                  banking: bool = True) -> dict:
    """UH-5: ``MODE_NEW`` + ``KIND_LAUNCH`` chained with ``KIND_SETTLE``.

    A vertical self-toss with **no catch**: the ball leaves at ``throw_z_mm``
    and the chained SETTLE brings the cup back down to rest while the ball flies
    away. The throw target is the throw site — the same steady-state self-toss
    the whole toss programme flies; an AIMED unified throw is out of scope until
    the aim-authority window is re-derived against ``tilt_geometry.MAX_TILT_DEG``
    (12 deg), which plan § 4 makes a Phase-5 prerequisite for UH-7.

    **The chain is a safety property, not a convenience.** A plan that ENDS at a
    release commands a hard STOP at the throw if it is streamed to its end:
    ``CyclePlan.hand_at(t >= duration)`` returns the terminal hold, so the frame
    at ``duration - dt`` carries ``hand_next_vel_rps = 0`` and that v1 is what
    the firmware's Hermite uses for the release segment itself. Measured on a
    0.6 s LAUNCH: true terminal hand velocity 93.011 rev/s emitted as 0.0. No
    guard fires on that; the throw is simply wrong. Chaining the settle in
    makes the FIRST installed plan rest-terminal, so there is no window in which
    a correctly planned launch is one late call away from a stopped throw.
    """
    x, y, _z = (float(cup_mm[0]), float(cup_mm[1]), float(cup_mm[2]))
    if not flight_s > 0.0:
        raise BenchError('--flight must be > 0 s, got %r' % (flight_s,))
    if not period_s > 0.0 or not chain_period_s > 0.0:
        raise BenchError('--launch-window and --chain-period must be > 0 s')
    settle_z = float(_z if settle_z_mm is None else settle_z_mm)
    return {
        'mode': MODE_NEW,
        'kind': KIND_LAUNCH,
        'period_s': float(period_s),
        'throw_site_mm': [x, y, float(throw_z_mm)],
        'throw_target_mm': [x, y, float(throw_z_mm)],
        'flight_s': float(flight_s),
        # Inert for LAUNCH and for the chained SETTLE alike (neither carries a
        # catch), but sent as the physically honest values rather than zeros so
        # a later `--chain-kind landing` needs no second spelling of them.
        'catch_site_mm': [x, y, CATCH_CUP_Z_MM],
        'catch_vel_mm_s': [0.0, 0.0, -release_speed_for_flight_mm_s(flight_s)],
        'catch_frac': 0.0,
        'settle_site_mm': [x, y, settle_z],
        'banking_enabled': bool(banking),
        'chain': True,
        'chain_kind': KIND_SETTLE,
        'chain_period_s': float(chain_period_s),
        'chain_catch_frac': 0.0,
        'lead_s': 0.0,
    }


def return_request(spec: dict) -> dict:
    """The carry's RETURN move: the same window with the displacement undone.

    Built from the outbound request rather than re-derived from a fresh cup
    read, so the machine provably goes back to where it started even if the
    outbound left a residual — a second forward-map read would carry that
    residual into the return and the pair would drift across repeats.
    """
    if spec['kind'] != KIND_SETTLE or spec['chain']:
        raise BenchError('return_request expects a bare SETTLE carry')
    out = dict(spec)
    out['settle_site_mm'] = list(spec.get('_origin_cup_mm')
                                 or spec['settle_site_mm'])
    return out


# ─────────────────────────────────────────────────────────────────────────────
# Pure core — preconditions
# ─────────────────────────────────────────────────────────────────────────────

class Snapshot:
    """What the ROS half saw, flattened so the evaluator needs no ROS types.

    Ages are SECONDS since the message arrived; ``None`` means never seen.
    """

    def __init__(self, *, status_age_s=None, mode=None, streaming=None,
                 leg_vel=None, leg_acc=None, leg_jerk=None, cycle_active=None,
                 robot_state_age_s=None, hand_axis_state=None,
                 hand_pos_rev=None, link_age_s=None, hand_source=None):
        self.status_age_s = status_age_s
        self.mode = mode
        self.streaming = streaming
        self.leg_vel = leg_vel
        self.leg_acc = leg_acc
        self.leg_jerk = leg_jerk
        self.cycle_active = cycle_active
        self.robot_state_age_s = robot_state_age_s
        self.hand_axis_state = hand_axis_state
        self.hand_pos_rev = hand_pos_rev
        self.link_age_s = link_age_s
        self.hand_source = hand_source


def _fresh(age, max_age_s):
    return age is not None and float(age) <= float(max_age_s)


def check_preconditions(snap: Snapshot, *, max_age_s: float = 2.0,
                        limit_tol: float = 1e-6) -> list:
    """Five checks. Returns ``[{'id','ok','detail','fix'}, ...]``, in order.

    Every failure carries the command that fixes it, because a refusal an
    operator has to go and look up is a refusal that gets bypassed.
    """
    out = []

    out.append({
        'id': 'P1', 'name': 'trajectory/status fresh, TRAJECTORY, streaming',
        'ok': (_fresh(snap.status_age_s, max_age_s)
               and snap.mode == 'TRAJECTORY' and bool(snap.streaming)),
        'detail': ('age=%s mode=%r streaming=%r'
                   % (_age(snap.status_age_s), snap.mode, snap.streaming)),
        'fix': ('is the launch up, and is the robot ACTIVATED? '
                'trajectory/plan_cycle refuses WRONG_MODE outside TRAJECTORY '
                'mode and STALE_STATE when the emitter is not seeded'),
    })

    out.append({
        'id': 'P2', 'name': '/robot_state fresh',
        'ok': _fresh(snap.robot_state_age_s, max_age_s),
        'detail': 'age=%s' % (_age(snap.robot_state_age_s),),
        'fix': ('no telemetry from the can-bridge — check /link_status; '
                'trajectory_node refuses STALE_STATE past its own 0.5 s bound'),
    })

    out.append({
        'id': 'P3', 'name': 'hand axis %d in CLOSED_LOOP' % (HAND_AXIS,),
        'ok': snap.hand_axis_state == AXIS_STATE_CLOSED_LOOP,
        'detail': ('motor_states[%d].current_state=%r (CLOSED_LOOP=%d)'
                   % (HAND_AXIS, snap.hand_axis_state, AXIS_STATE_CLOSED_LOOP)),
        'fix': ('the hand is NOT energised — an IDLE hand ignores every '
                'set_input_pos the streamed lane sends and looks exactly like '
                'a perfect hold. With the launch UP the bridge energises it at '
                'ACTIVATE whenever hand_source reads STREAMED, so this means '
                'ACTIVATE did not take (or the latch was LEGACY at ACTIVATE): '
                'deactivate, confirm hand_source, ACTIVATE again'),
    })

    out.append({
        'id': 'P4', 'name': 'hand_source latch STREAMED',
        'ok': (_fresh(snap.link_age_s, max_age_s)
               and snap.hand_source == 'STREAMED'),
        'detail': ('/link_status hand_source=%r (age=%s)'
                   % (snap.hand_source, _age(snap.link_age_s))),
        'fix': ('while LEGACY_STROKE the firmware DISCARDS Setpoint index 6 — '
                'counted, silent to the plan — so the platform flies the whole '
                'window with a dead hand. The latch cannot be switched while '
                'the setpoint output is armed, so: bring the launch DOWN, run  '
                'python tests/hardware/hand_stream_bench.py --source-only '
                'streamed  (expect "hand_source -> STREAMED: OK"), then launch '
                'and ACTIVATE again'),
    })

    lim_ok = all(
        got is not None and abs(float(got) - want) <= limit_tol * max(1.0, want)
        for got, want in ((snap.leg_vel, SESSION_LEG_VEL_MMPS),
                          (snap.leg_acc, SESSION_LEG_ACC_MMPS2),
                          (snap.leg_jerk, SESSION_LEG_JERK_MMPS3)))
    out.append({
        'id': 'P5', 'name': 'catch-capable session limits',
        'ok': bool(lim_ok),
        'detail': ('vel=%s acc=%s jerk=%s (want %.0f / %.0f / %.0f)'
                   % (snap.leg_vel, snap.leg_acc, snap.leg_jerk,
                      SESSION_LEG_VEL_MMPS, SESSION_LEG_ACC_MMPS2,
                      SESSION_LEG_JERK_MMPS3)),
        'fix': ('at the shipped 1000/5000/30000 every unified cycle reads '
                'LIMIT_JERK — the z-launch jerk leaks sin(tilt) x 744.3 mm '
                'into centroid xy. Raise them:\n      ' + SET_LIMITS_CMD),
    })
    return out


def _age(a):
    return 'never' if a is None else '%.2f s' % (float(a),)


# ─────────────────────────────────────────────────────────────────────────────
# Pure core — the verdict
# ─────────────────────────────────────────────────────────────────────────────

def evaluate(rung: str, plan: dict, trace: list, *,
             legacy_duration_s=None,
             metal_margin_rev: float = DEFAULT_METAL_MARGIN_REV) -> list:
    """Score one rung. ``[{'id','name','verdict','detail'}, ...]``.

    ``plan`` is the accepted ``PlanCycle`` response flattened to a dict; ``trace``
    is the per-tick rows the run recorded (dicts with ``t``, ``hand_cmd_rev``,
    ``hand_enc_rev``, ``hand_vel_rps``, ``hand_axis_state``, ``cycle_active``).
    ``verdict`` is PASS / FAIL / SKIP, matching ``hand_stream_bench.py``.

    Nothing here scores the BALL. The by-eye cup watch is the operator's and is
    what T-H5 actually turns on; the release-velocity check is bag-measured
    afterwards. This scores what the machine can witness about itself.
    """
    checks = []
    ticks = [r for r in trace if r.get('hand_cmd_rev') is not None]

    checks.append({
        'id': 'V1', 'name': 'plan accepted',
        'verdict': 'PASS' if plan.get('accepted') else 'FAIL',
        'detail': '%s: %s' % (plan.get('code'), plan.get('message')),
    })

    wall = plan.get('plan_wall_ms')
    if wall is None:
        checks.append({'id': 'V2', 'name': 'plan wall time', 'verdict': 'SKIP',
                       'detail': 'no plan_wall_ms in the response'})
    else:
        # The owner's SPLIT budget: core <= 50 ms, total <= 250 ms. A chained
        # install is two solves and its measured warm cost is 424 ms — recorded
        # as a deviation, not waived — so the chained rung is judged against
        # that and reported either way rather than failed on a known number.
        bar = 500.0 if plan.get('chained') else 250.0
        checks.append({
            'id': 'V2', 'name': 'plan wall time <= %.0f ms' % (bar,),
            'verdict': 'PASS' if float(wall) <= bar else 'FAIL',
            'detail': '%.1f ms%s' % (float(wall),
                                     ' (chained: two solves)'
                                     if plan.get('chained') else ''),
        })

    # ── V3: margin to METAL, measured on the ENCODER ────────────────────────
    # Scored on the encoder and not on the commanded peak, because the commanded
    # peak cannot reach metal: `validate_cycle` already refuses a plan outside
    # the operating band [0, JB_OP_HAND_CATCH_PRIME_REV] (9.9594 rev) with
    # HAND_STROKE, and the firmware clips every setpoint to [0, 10.8] after
    # that. A verdict on the command would restate two gates and could never
    # fail. What CAN reach metal is the physical slider overshooting its
    # command, and sitting two measured exactly that: encoder 10.4693 rev
    # against the 10.8 hard stop — 0.33 rev, 10 mm — on a legacy stroke at
    # --event-vel 3.0. The commanded margin is reported beside it, never gated.
    peak = plan.get('hand_peak_rev')
    encs = [float(r['hand_enc_rev']) for r in ticks
            if r.get('hand_enc_rev') is not None]
    cmd_note = ('' if peak is None else
                '; commanded peak %.4f (%.4f to metal, %.4f to the planner '
                'band ceiling)' % (float(peak),
                                   HAND_MOTOR_MAX_POSITION_REV - float(peak),
                                   HAND_CATCH_PRIME_REV - float(peak)))
    if not encs:
        checks.append({'id': 'V3', 'name': 'hand stayed clear of metal',
                       'verdict': 'SKIP',
                       'detail': 'no encoder ticks%s' % (cmd_note,)})
    else:
        worst_enc = max(encs)
        margin = HAND_MOTOR_MAX_POSITION_REV - worst_enc
        checks.append({
            'id': 'V3',
            'name': ('encoder stayed >= %.2f rev clear of the %.1f rev hard '
                     'stop' % (metal_margin_rev, HAND_MOTOR_MAX_POSITION_REV)),
            'verdict': 'PASS' if margin >= metal_margin_rev else 'FAIL',
            'detail': ('worst encoder %.4f rev, margin %.4f rev (%.1f mm)%s'
                       % (worst_enc, margin, margin / REV_PER_MM, cmd_note)),
        })

    if not ticks:
        checks.append({'id': 'V4', 'name': 'hand tracking', 'verdict': 'SKIP',
                       'detail': 'no telemetry ticks recorded'})
        checks.append({'id': 'V5', 'name': 'hand stayed energised',
                       'verdict': 'SKIP', 'detail': 'no telemetry ticks'})
    else:
        # |cmd - enc| against the firmware's own E-STOP band. NOT
        # velocity-compensated: this reads the 10-45 ms-stale telemetry cache,
        # so at speed it over-reads by the latency and is a CEILING on the true
        # residual, never a floor. A number under the band is therefore real
        # evidence; a number over it needs the [hand7] console line to
        # adjudicate, which is why the detail names the worst tick's time.
        worst, worst_t = 0.0, None
        # A tick whose ENCODER is missing cannot be scored, and scoring it as
        # 0.0 rev of deviation is the sitting-one failure in miniature: a check
        # that reads "perfect" because it read nothing. Counted and, if that is
        # ALL of them, the verdict is SKIP rather than a vacuous PASS.
        scored = 0
        for r in ticks:
            enc = r.get('hand_enc_rev')
            if enc is None:
                continue
            scored += 1
            d = abs(float(r['hand_cmd_rev']) - float(enc))
            if d > worst:
                worst, worst_t = d, r.get('t')
        if not scored:
            checks.append({
                'id': 'V4', 'name': 'hand tracking', 'verdict': 'SKIP',
                'detail': ('%d ticks, NONE carrying an encoder reading — '
                           'nothing to score' % (len(ticks),)),
            })
        else:
            checks.append({
                'id': 'V4',
                'name': 'worst |cmd-enc| under MAX_DEVIATION_HAND_REV %.1f'
                        % (MAX_DEVIATION_HAND_REV,),
                'verdict': 'PASS' if worst <= MAX_DEVIATION_HAND_REV else 'FAIL',
                'detail': ('%.4f rev at t=%s over %d of %d ticks (CEILING — '
                           'telemetry-stale, not velocity-compensated)'
                           % (worst,
                              'n/a' if worst_t is None else '%.2f s' % worst_t,
                              scored, len(ticks))),
            })
        # A MISSING axis state is not evidence that the hand was energised. It
        # is what a six-axis `/robot_state` looks like, and treating it as
        # benign is exactly how sitting one's `hold` row passed against an IDLE
        # axis. Anything that is not CLOSED_LOOP — including nothing at all —
        # fails this check.
        idle = [r for r in ticks
                if r.get('hand_axis_state') != AXIS_STATE_CLOSED_LOOP]
        unknown = sum(1 for r in idle if r.get('hand_axis_state') is None)
        checks.append({
            'id': 'V5', 'name': 'hand stayed in CLOSED_LOOP for every tick',
            'verdict': 'PASS' if not idle else 'FAIL',
            'detail': ('%d of %d ticks not CLOSED_LOOP (%d of them reported NO '
                       'axis state at all)' % (len(idle), len(ticks), unknown)
                       if idle else '%d ticks, all CLOSED_LOOP' % (len(ticks),)),
        })

    saw_cycle = any(bool(r.get('cycle_active')) for r in trace)
    # "Finished" must be POSITIVE evidence: the last tick reported cycle_active
    # and reported it FALSE. A tick with no status at all (None) says nothing
    # about whether the window ended, and reading it as "ended" would call a
    # cycle that is still installed a clean finish.
    last = trace[-1].get('cycle_active') if trace else None
    ended = last is False or last == 0
    checks.append({
        'id': 'V6', 'name': 'the cycle ran and finished',
        'verdict': 'PASS' if (saw_cycle and ended) else
                   ('FAIL' if saw_cycle else 'SKIP'),
        'detail': ('cycle_active seen=%s, cleared at the end=%s (last tick '
                   'reported %r)' % (saw_cycle, bool(ended), last)),
    })

    if rung == 'carry':
        if legacy_duration_s is None:
            checks.append({
                'id': 'V7', 'name': 'T-H5: re-pose duration <= legacy GoToPose',
                'verdict': 'SKIP',
                'detail': ('the legacy comparison could not be computed offline '
                           '(motion package not importable here) — compute it '
                           'from planner.build_move at lean_gain 0 desk-side'),
            })
        else:
            dur = plan.get('duration_s')
            checks.append({
                'id': 'V7', 'name': 'T-H5: re-pose duration <= legacy GoToPose',
                'verdict': ('SKIP' if dur is None else
                            ('PASS' if float(dur) <= float(legacy_duration_s)
                             else 'FAIL')),
                'detail': ('unified %.3f s vs legacy minimal-feasible %.3f s '
                           '(lean_gain 0). The OTHER half of T-H5 — no visible '
                           'ball disturbance — is the operator\'s eye, not this '
                           'number.'
                           % (float(dur or 0.0), float(legacy_duration_s))),
            })
    if rung == 'throw':
        rv = plan.get('release_vel_mm_s')
        want = plan.get('_expected_release_mm_s')
        if rv is None or want is None:
            checks.append({'id': 'V7', 'name': 'release velocity vs ballistics',
                           'verdict': 'SKIP', 'detail': 'no release velocity'})
        else:
            got = float(rv[2])
            err = abs(got - float(want)) / max(1e-9, float(want))
            checks.append({
                'id': 'V7', 'name': 'planned release vz within 5 % of gT/2',
                'verdict': 'PASS' if err <= 0.05 else 'FAIL',
                'detail': ('plan %.1f mm/s vs ballistics %.1f mm/s (%.2f %%). '
                           'This compares the PLAN with the arithmetic; the '
                           'T-H6 check that matters — ACHIEVED release velocity '
                           'within 5 %% of plan — is bag-measured afterwards '
                           'from /balls (tracker) against /throw_announcements'
                           % (got, float(want), 100.0 * err)),
            })
    return checks


def verdict_rc(checks) -> int:
    """0 iff no FAIL — ``toss_trace_recorder.py``'s rule."""
    return 1 if any(c['verdict'] == 'FAIL' for c in checks) else 0


# ─────────────────────────────────────────────────────────────────────────────
# Offline helpers that need the motion package (imported lazily, best-effort)
# ─────────────────────────────────────────────────────────────────────────────

def cup_site_now(pose6, hand_rev):
    """Cup opening (mm; xy platform-frame, z GLOBAL) for a pose + slider.

    ``unified_cycle.cup_state_from_platform`` — the exact inverse of the
    decomposition the planner realises with, so the site this driver asks to
    settle at is expressed in the frame the planner solves in. Imported lazily
    so ``--dry-run`` works without a colcon build.
    """
    from jugglebot.motion import unified_cycle as uc
    return [float(v) for v in uc.cup_state_from_platform(pose6, float(hand_rev))]


def legacy_move_duration_s(pose6, target_pose6, leg_limits):
    """Minimal feasible ``GoToPose`` duration for the same re-pose, lean OFF.

    T-H5's bar. ``planner.build_move`` with ``duration_s=None`` runs the
    duration-stretch loop and returns the shortest T the feasibility gate
    accepts, with ``shaper=None`` — which IS ``lean_gain = 0``, the identity.
    Returns ``None`` (never raises) if the package will not import or the pose
    is infeasible: this is a REPORTED comparison, not a gate, and a missing
    number must not stop a rung.
    """
    try:
        import numpy as np
        from jugglebot.motion.trajectory.planner import build_move
        from jugglebot.motion.trajectory.limits import TrajectoryLimits
        from jugglebot.motion.geometry import StewartGeometry
        import jugglebot.hardware_config as hw
        lim = TrajectoryLimits.from_config(
            hw, leg_vel_mmps=leg_limits[0], leg_acc_mmps2=leg_limits[1],
            leg_jerk_mmps3=leg_limits[2])
        state0 = (np.asarray(pose6, dtype=float), np.zeros(6), np.zeros(6))
        plan, _report = build_move(state0, np.asarray(target_pose6, dtype=float),
                                   None, lim, StewartGeometry(), shaper=None)
        return float(plan.duration)
    except Exception:                              # noqa: BLE001 — advisory only
        return None


# ─────────────────────────────────────────────────────────────────────────────
# Printing
# ─────────────────────────────────────────────────────────────────────────────

def print_request(spec: dict, *, prefix: str = '') -> None:
    print('%srequest:' % (prefix,))
    for k in ('mode', 'kind', 'period_s', 'throw_site_mm', 'throw_target_mm',
              'flight_s', 'catch_site_mm', 'catch_vel_mm_s', 'catch_frac',
              'settle_site_mm', 'banking_enabled', 'chain', 'chain_kind',
              'chain_period_s', 'chain_catch_frac', 'lead_s'):
        v = spec[k]
        if isinstance(v, list):
            v = '[' + ', '.join('%.3f' % f for f in v) + ']'
        print('%s  %-18s %s' % (prefix, k, v))


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
# The ROS half
# ─────────────────────────────────────────────────────────────────────────────

CSV_COLUMNS = ('t', 'phase', 'cycle_active', 'plan_time_remaining_s',
               'cycle_plan_wall_ms', 'cycle_hand_peak_rev',
               'cycle_hand_peak_vel_rps', 'cycle_supersede_deadline_s',
               'hand_cmd_rev', 'hand_enc_rev', 'hand_vel_rps', 'hand_iq_a',
               'hand_axis_state', 'ball_held', 'ball_held_valid',
               'leg_vel_limit_mmps', 'leg_jerk_limit_mmps3', 'hand_source',
               'last_rejection')


class _Runner:
    """Thin rclpy client: one service, four subscriptions, no state machine."""

    def __init__(self, node, args):
        from jugglebot_interfaces.msg import (HandTelemetryMessage, RobotState,
                                              TrajectoryStatus)
        from jugglebot_interfaces.srv import PlanCycle
        from diagnostic_msgs.msg import DiagnosticStatus
        from geometry_msgs.msg import Pose

        self.node = node
        self.args = args
        self._PlanCycle = PlanCycle

        self.status = None
        self.status_stamp = None
        self.link_kv = {}
        self.link_stamp = None
        self.motor_states = None
        self.robot_state_stamp = None
        self.hand = None
        self.pose = None

        self.cli_plan = node.create_client(PlanCycle, '/trajectory/plan_cycle')
        node.create_subscription(TrajectoryStatus, '/trajectory/status',
                                 self._on_status, 10)
        node.create_subscription(DiagnosticStatus, '/link_status',
                                 self._on_link, 10)
        node.create_subscription(RobotState, '/robot_state',
                                 self._on_robot_state, 10)
        node.create_subscription(HandTelemetryMessage, '/hand_telemetry',
                                 self._on_hand, 50)
        node.create_subscription(Pose, '/trajectory/commanded_pose',
                                 self._on_pose, 10)

    # -- subscriptions ----------------------------------------------------

    def _on_status(self, msg):
        self.status, self.status_stamp = msg, time.time()

    def _on_link(self, msg):
        self.link_kv = {v.key: v.value for v in msg.values}
        self.link_stamp = time.time()

    def _on_robot_state(self, msg):
        self.motor_states = msg.motor_states
        self.robot_state_stamp = time.time()

    def _on_hand(self, msg):
        self.hand = msg

    def _on_pose(self, msg):
        self.pose = msg

    # -- plumbing ---------------------------------------------------------

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
        ms = self.motor_states
        hand_state = None
        if ms is not None and len(ms) > HAND_AXIS:
            hand_state = int(ms[HAND_AXIS].current_state)
        return Snapshot(
            status_age_s=self._age(self.status_stamp),
            mode=None if st is None else str(st.mode),
            streaming=None if st is None else bool(st.streaming),
            leg_vel=None if st is None else float(st.leg_vel_limit_mmps),
            leg_acc=None if st is None else float(st.leg_acc_limit_mmps2),
            leg_jerk=None if st is None else float(st.leg_jerk_limit_mmps3),
            cycle_active=None if st is None else bool(st.cycle_active),
            robot_state_age_s=self._age(self.robot_state_stamp),
            hand_axis_state=hand_state,
            hand_pos_rev=(None if ms is None or len(ms) <= HAND_AXIS
                          else float(ms[HAND_AXIS].pos_estimate)),
            link_age_s=self._age(self.link_stamp),
            hand_source=self.link_kv.get('hand_source'))

    def row(self, t0: float) -> dict:
        st, ms, h = self.status, self.motor_states, self.hand
        hand_state = (int(ms[HAND_AXIS].current_state)
                      if ms is not None and len(ms) > HAND_AXIS else None)
        return {
            't': round(time.time() - t0, 4),
            'phase': None if st is None else st.plan_kind,
            'cycle_active': None if st is None else bool(st.cycle_active),
            'plan_time_remaining_s': (None if st is None
                                      else round(st.plan_time_remaining_s, 4)),
            'cycle_plan_wall_ms': None if st is None else st.cycle_plan_wall_ms,
            'cycle_hand_peak_rev': None if st is None else st.cycle_hand_peak_rev,
            'cycle_hand_peak_vel_rps': (None if st is None
                                        else st.cycle_hand_peak_vel_rps),
            'cycle_supersede_deadline_s': (None if st is None
                                           else st.cycle_supersede_deadline_s),
            'hand_cmd_rev': None if h is None else h.pos_cmd,
            'hand_enc_rev': None if h is None else h.pos_meas,
            'hand_vel_rps': None if h is None else h.vel_meas,
            'hand_iq_a': None if h is None else h.iq_meas,
            'hand_axis_state': hand_state,
            'ball_held': None if h is None else bool(h.ball_held),
            'ball_held_valid': None if h is None else bool(h.ball_held_valid),
            'leg_vel_limit_mmps': None if st is None else st.leg_vel_limit_mmps,
            'leg_jerk_limit_mmps3': (None if st is None
                                     else st.leg_jerk_limit_mmps3),
            'hand_source': self.link_kv.get('hand_source'),
            'last_rejection': None if st is None else st.last_rejection,
        }

    # -- the one service call ---------------------------------------------

    def plan(self, spec: dict):
        import rclpy
        if not self.cli_plan.wait_for_service(timeout_sec=self.args.timeout_s):
            raise BenchError(
                'trajectory/plan_cycle unavailable after %.1f s — is the launch '
                'up, and were BOTH jugglebot_interfaces and jugglebot rebuilt? '
                '(colcon build --packages-select jugglebot_interfaces jugglebot)'
                % (self.args.timeout_s,))
        req = self._PlanCycle.Request()
        for k, v in spec.items():
            if k.startswith('_'):
                continue
            setattr(req, k, v)
        future = self.cli_plan.call_async(req)
        rclpy.spin_until_future_complete(self.node, future,
                                         timeout_sec=self.args.timeout_s)
        resp = future.result()
        if resp is None:
            raise BenchError('trajectory/plan_cycle did not answer within '
                             '%.1f s' % (self.args.timeout_s,))
        return resp


def _flat(resp, spec) -> dict:
    """A ``PlanCycle.Response`` flattened for :func:`evaluate` and the meta file."""
    return {
        'accepted': bool(resp.accepted),
        'code': str(resp.code),
        'message': str(resp.message),
        't0_mono': float(resp.t0_mono),
        't_release_mono': float(resp.t_release_mono),
        't_catch_mono': float(resp.t_catch_mono),
        'release_vel_mm_s': [float(v) for v in resp.release_vel_mm_s],
        'release_terminal': bool(resp.release_terminal),
        'supersede_deadline_mono': float(resp.supersede_deadline_mono),
        'duration_s': float(resp.duration_s),
        'plan_wall_ms': float(resp.plan_wall_ms),
        'hand_peak_rev': float(resp.hand_peak_rev),
        'hand_peak_vel_rps': float(resp.hand_peak_vel_rps),
        'chained': bool(spec.get('chain')),
    }


def run(args) -> int:                                            # noqa: C901
    """The ROS half. Imports rclpy lazily so the pure core stays importable."""
    import rclpy

    out_dir = args.out_dir or os.path.join(_REPO, 'temp', 'logs')
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(out_dir, 'unified_cycle_bench_%s_%s.csv'
                            % (args.rung, stamp))
    meta_path = csv_path[:-4] + '_meta.json'

    rclpy.init()
    node = None
    csv_f = None
    rc = 0
    try:
        try:
            node = rclpy.create_node('unified_cycle_bench')
            runner = _Runner(node, args)
        except ImportError as exc:
            raise BenchError(
                'interface import failed (%s). Did you run  colcon build '
                '--packages-select jugglebot_interfaces jugglebot  and  source '
                'ros_ws/install/setup.bash ?' % (exc,))

        print('waiting %.1f s for telemetry...' % (args.settle_s,))
        runner.spin(args.settle_s)

        snap = runner.snapshot()
        pre = check_preconditions(snap, max_age_s=args.max_age_s)
        print_checks('PRECONDITIONS', pre)
        if any(not c['ok'] for c in pre):
            print('\nREFUSED: %d precondition(s) not met — nothing was '
                  'commanded.' % (sum(1 for c in pre if not c['ok'])),
                  file=sys.stderr)
            return 2

        # The cup site the rung is built around: the forward map of the LIVE
        # commanded pose and the LIVE hand. Read once, so the outbound and the
        # return share one origin.
        pose6, pose_note = live_pose(runner.pose, args.pose)
        hand_rev = snap.hand_pos_rev
        # The forward map is the default; each override replaces ONE component,
        # so `--z-mm` alone raises the rung without also moving it in xy.
        cup = cup_site_now(pose6, hand_rev)
        for idx, override in enumerate((args.cup_x, args.cup_y, args.z_mm)):
            if override is not None:
                cup[idx] = float(override)
        print('\npose %s: [%.2f, %.2f, %.2f] mm'
              % (pose_note, pose6[0], pose6[1], pose6[2]))
        print('cup opening now: [%.2f, %.2f, %.2f] mm (hand %.4f rev)'
              % (cup[0], cup[1], cup[2], hand_rev))

        specs = _build_specs(args, cup)
        legacy = None
        if args.rung == 'carry':
            target = list(pose6)
            target[0] += args.dx
            target[1] += args.dy
            legacy = legacy_move_duration_s(
                pose6, target,
                (snap.leg_vel, snap.leg_acc, snap.leg_jerk))
            print('legacy GoToPose minimal-feasible duration for the same '
                  're-pose at lean_gain 0: %s'
                  % ('unavailable' if legacy is None else '%.3f s' % legacy))

        csv_f = open(csv_path, 'w', newline='')
        writer = csv.DictWriter(csv_f, fieldnames=CSV_COLUMNS)
        writer.writeheader()

        t0 = time.time()
        trace = []
        plans = []
        for idx, (label, spec) in enumerate(specs):
            print('\n── %s ─────────────────────────────────────────' % (label,))
            print_request(spec, prefix='  ')
            if idx:
                # Re-check between moves: the first one may have left the
                # machine somewhere the second cannot be planned from, and a
                # refusal named here beats a planner refusal three lines on.
                re_pre = check_preconditions(runner.snapshot(),
                                             max_age_s=args.max_age_s)
                if any(not c['ok'] for c in re_pre):
                    print_checks('PRECONDITIONS (before %s)' % (label,), re_pre)
                    raise BenchError('preconditions lapsed before %s' % (label,))
            resp = runner.plan(spec)
            flat = _flat(resp, spec)
            flat['label'] = label
            if args.rung == 'throw':
                flat['_expected_release_mm_s'] = \
                    release_speed_for_flight_mm_s(args.flight)
            plans.append(flat)
            print('  accepted=%s code=%s' % (flat['accepted'], flat['code']))
            print('  message: %s' % (flat['message'],))
            if not flat['accepted']:
                raise BenchError('plan_cycle REFUSED %s: %s'
                                 % (flat['code'], flat['message']))
            print('  duration=%.3f s  plan_wall=%.1f ms  hand_peak=%.4f rev  '
                  'hand_peak_vel=%.2f rev/s'
                  % (flat['duration_s'], flat['plan_wall_ms'],
                     flat['hand_peak_rev'], flat['hand_peak_vel_rps']))
            print('  release_terminal=%s  t_release_mono=%.4f  '
                  'release_vel=[%.1f, %.1f, %.1f] mm/s'
                  % (flat['release_terminal'], flat['t_release_mono'],
                     flat['release_vel_mm_s'][0], flat['release_vel_mm_s'][1],
                     flat['release_vel_mm_s'][2]))
            if flat['release_terminal']:
                print('  !! RELEASE-TERMINAL install — it MUST be superseded '
                      'before its deadline or the release segment is commanded '
                      'to a stop. This driver does not supersede. STOP.')

            watch = flat['duration_s'] + args.tail_s
            deadline = time.time() + watch
            while time.time() < deadline:
                runner.spin(0.02)
                row = runner.row(t0)
                trace.append(row)
                writer.writerow(row)
                csv_f.flush()
            print('  watched %.2f s, %d ticks' % (watch, len(trace)))

        checks = evaluate(args.rung, plans[-1], trace,
                          legacy_duration_s=legacy,
                          metal_margin_rev=args.metal_margin)
        # A multi-move rung is only as good as its worst move, so the earlier
        # plans are scored too rather than being trusted because the run reached
        # the end.
        for p in plans[:-1]:
            for c in evaluate(args.rung, p, [], legacy_duration_s=legacy,
                              metal_margin_rev=args.metal_margin):
                if c['verdict'] != 'SKIP':
                    c = dict(c)
                    c['id'] = '%s/%s' % (p['label'], c['id'])
                    checks.insert(0, c)
        print_checks('VERDICT — rung %s' % (args.rung.upper(),), checks)
        rc = verdict_rc(checks)

        with open(meta_path, 'w') as f:
            json.dump({
                'tool': TOOL_NAME, 'rung': args.rung,
                'started': datetime.now().isoformat(timespec='seconds'),
                'args': {k: v for k, v in vars(args).items()},
                'cup_mm': cup, 'pose6': list(pose6), 'hand_rev': hand_rev,
                'legacy_duration_s': legacy,
                'preconditions': pre, 'plans': plans, 'checks': checks,
                'csv': csv_path, 'ticks': len(trace),
            }, f, indent=2, default=str)

        print('\ncsv  -> %s' % (csv_path,))
        print('meta -> %s' % (meta_path,))
        print('the bag is the launch\'s own (record:=true) — send it with these.')
        return rc
    except BenchError as exc:
        print('\nABORT: %s' % (exc,), file=sys.stderr)
        return 2
    finally:
        if csv_f:
            csv_f.close()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def quat_to_rotvec(qx, qy, qz, qw):
    """``geometry_msgs/Quaternion`` -> the ``(rx, ry, rz)`` rotation vector (rad).

    The pose convention this package plans in is a rotation VECTOR, not Euler
    angles — ``tilt_geometry.cup_axis(rx, ry)`` reads the first two components
    directly — and ``trajectory_node`` publishes the same sample as a quaternion
    on ``trajectory/commanded_pose``. This is the inverse, so the cup site is
    built in the frame the planner solves in rather than in a second one that
    happens to agree at level.
    """
    import numpy as np
    q = np.array([float(qx), float(qy), float(qz), float(qw)], dtype=float)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return [0.0, 0.0, 0.0]
    q = q / n
    if q[3] < 0.0:                      # canonical hemisphere: shortest rotation
        q = -q
    s = float(np.linalg.norm(q[:3]))
    if s < 1e-12:                       # identity, and the axis is undefined
        return [0.0, 0.0, 0.0]
    angle = 2.0 * math.atan2(s, float(q[3]))
    return [float(angle * q[i] / s) for i in range(3)]


def live_pose(pose_msg, override=None):
    """``(pose6, note)`` — the live commanded pose, or the ``--pose`` override.

    Used ONLY to build the cup site and the legacy-duration comparison. The plan
    itself is seeded inside ``trajectory_node`` from the live commanded state,
    which is the whole reason the planning happens there and not here: a caller
    planning from a 5 Hz sample would seed a window from a pose up to 200 ms
    stale and the install-continuity guard would refuse it by construction.

    ``None`` for the message with no override is an ERROR rather than a default.
    ``trajectory/commanded_pose`` is published ONLY while the emitter is
    streaming and is SILENT otherwise, deliberately ("publishing a
    stale/None-derived one would let a consumer site a throw at a pose the
    machine is not holding"), so inventing a home pose here would site a rung at
    a place the machine is not.
    """
    if override:
        parts = [float(v) for v in str(override).split(',')]
        if len(parts) != 6:
            raise BenchError('--pose needs six comma-separated values '
                             '(x,y,z,rx,ry,rz), got %d' % (len(parts),))
        return parts, 'from --pose'
    if pose_msg is None:
        raise BenchError(
            'no trajectory/commanded_pose seen — it is published only while the '
            'emitter is streaming, so either the robot is not ACTIVATED or the '
            'topic is not up. Pass --pose x,y,z,rx,ry,rz to override.')
    p = pose_msg.position
    rot = quat_to_rotvec(pose_msg.orientation.x, pose_msg.orientation.y,
                         pose_msg.orientation.z, pose_msg.orientation.w)
    tilt_deg = math.degrees(math.hypot(rot[0], rot[1]))
    note = 'live (tilt %.3f deg)' % (tilt_deg,)
    if tilt_deg > 0.5:
        note += ' — NOT level; both rungs are planned from rest and a settled ' \
                'machine is level, so check this before trusting the cup site'
    return [float(p.x), float(p.y), float(p.z)] + rot, note


def _build_specs(args, cup):
    """``[(label, request-dict), ...]`` for the rung."""
    if args.rung == 'carry':
        out = carry_request(cup, dx_mm=args.dx, dy_mm=args.dy,
                            period_s=args.period, banking=not args.no_banking)
        out['_origin_cup_mm'] = list(cup)
        back = return_request(out)
        specs = [('carry out (+%.0f, +%.0f mm)' % (args.dx, args.dy), out)]
        if not args.no_return:
            specs.append(('carry back to origin', back))
        return specs
    spec = throw_request(cup, flight_s=args.flight,
                         period_s=args.launch_window,
                         chain_period_s=args.chain_period,
                         throw_z_mm=args.throw_z,
                         banking=not args.no_banking)
    return [('throw (flight %.3f s, apex %.2f m)'
             % (args.flight, args.apex), spec)]


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def build_parser():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rung', choices=('carry', 'throw'), required=True,
                    help='carry = UH-3 (banked carry, seated ball); '
                         'throw = UH-5 (planned throw, no catch)')
    ap.add_argument('--dry-run', action='store_true',
                    help='print the request and the derived numbers; ZERO ROS '
                         'calls, no ROS objects constructed')

    g = ap.add_argument_group('carry (UH-3)')
    g.add_argument('--dx', type=float, default=60.0,
                   help='carry displacement in +x (mm, default 60)')
    g.add_argument('--dy', type=float, default=0.0,
                   help='carry displacement in +y (mm, default 0)')
    g.add_argument('--period', type=float, default=DEFAULT_PERIOD_S,
                   help='carry window duration (s, default %.1f — the sim '
                        'gate found 1.4 load-bearing; 1.0 refused LIMIT_JERK '
                        'on the 50 mm ring)' % (DEFAULT_PERIOD_S,))
    g.add_argument('--no-return', action='store_true',
                   help='do not plan the return move back to the origin')

    t = ap.add_argument_group('throw (UH-5)')
    t.add_argument('--apex', type=float, default=DEFAULT_APEX_M,
                   help='apex above the release plane (m, default %.2f). '
                        'T-H6 caps the rung at 0.5 m.' % (DEFAULT_APEX_M,))
    t.add_argument('--flight', type=float, default=None,
                   help='flight time (s) — overrides --apex')
    t.add_argument('--launch-window', type=float,
                   default=DEFAULT_LAUNCH_WINDOW_S,
                   help='LAUNCH window duration (s, default %.1f)'
                        % (DEFAULT_LAUNCH_WINDOW_S,))
    t.add_argument('--chain-period', type=float, default=DEFAULT_CHAIN_PERIOD_S,
                   help='chained SETTLE duration (s, default %.1f)'
                        % (DEFAULT_CHAIN_PERIOD_S,))
    t.add_argument('--throw-z', type=float, default=THROW_CUP_Z_MM,
                   help='release cup height (mm global, default %.1f)'
                        % (THROW_CUP_Z_MM,))

    c = ap.add_argument_group('common')
    c.add_argument('--no-banking', action='store_true',
                   help='plan with banking OFF. A Phase-0 A/B arm, NOT an '
                        'operating point: zero banking cannot plan a steady '
                        'cycle at session limits (measured LIMIT_VEL 529 mm/s '
                        'against a 250 cap) and the node warns about it.')
    c.add_argument('--z-mm', type=float, default=None,
                   help='override the cup z the rung runs at (mm global); '
                        'default is the live forward-mapped cup height')
    c.add_argument('--cup-x', type=float, default=None,
                   help='override the cup x (mm, platform frame)')
    c.add_argument('--cup-y', type=float, default=None,
                   help='override the cup y (mm, platform frame)')
    c.add_argument('--pose', default=None,
                   help='override the live pose as x,y,z,rx,ry,rz (mm/rad)')
    c.add_argument('--settle-s', type=float, default=2.0,
                   help='seconds to collect telemetry before the checks (2.0)')
    c.add_argument('--tail-s', type=float, default=1.5,
                   help='seconds to keep watching after the plan ends (1.5)')
    c.add_argument('--max-age-s', type=float, default=2.0,
                   help='precondition freshness bound (s, default 2.0)')
    c.add_argument('--timeout-s', type=float, default=10.0,
                   help='per-service-call timeout (s, default 10 — a chained '
                        'install is two solves and measured 424 ms warm)')
    c.add_argument('--metal-margin', type=float,
                   default=DEFAULT_METAL_MARGIN_REV,
                   help='encoder clearance required from the %.1f rev hard '
                        'stop (rev, default %.2f — sitting two measured '
                        '10.4693 rev on a legacy stroke)'
                        % (HAND_MOTOR_MAX_POSITION_REV,
                           DEFAULT_METAL_MARGIN_REV))
    c.add_argument('--out-dir', default=None,
                   help='CSV/meta output directory (default temp/logs/)')
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.flight is None:
        args.flight = flight_for_apex_s(args.apex)
    else:
        args.apex = (args.flight ** 2) * G_MM_S2 / 8000.0
    if args.rung == 'throw' and args.apex > 0.5 + 1e-9:
        print('note: apex %.3f m is above the T-H6 low tier (0.5 m). The rung '
              'is defined at h <= 0.5 m; a higher one needs the owner.'
              % (args.apex,), file=sys.stderr)

    if args.dry_run:
        cup = [float(args.cup_x or 0.0), float(args.cup_y or 0.0),
               float(args.z_mm if args.z_mm is not None else 689.6)]
        print('dry-run: no ROS calls made, no ROS objects constructed.')
        print('cup opening assumed: [%.2f, %.2f, %.2f] mm '
              '(--cup-x/--cup-y/--z-mm; live it is the forward map of the '
              'commanded pose + hand)' % tuple(cup))
        if args.rung == 'throw':
            print('apex %.3f m -> flight %.4f s -> release vz %.1f mm/s (gT/2)'
                  % (args.apex, args.flight,
                     release_speed_for_flight_mm_s(args.flight)))
        for label, spec in _build_specs(args, cup):
            print('\n── %s ──' % (label,))
            print_request(spec, prefix='  ')
        print('\nsession limits this rung requires: %.0f / %.0f / %.0f'
              % (SESSION_LEG_VEL_MMPS, SESSION_LEG_ACC_MMPS2,
                 SESSION_LEG_JERK_MMPS3))
        print('  ' + SET_LIMITS_CMD)
        return 0

    try:
        return run(args)
    except BenchError as exc:
        print('ABORT: %s' % (exc,), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
