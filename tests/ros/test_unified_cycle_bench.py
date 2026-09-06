"""Offline tests for ``tests/hardware/unified_cycle_bench.py`` — the Phase 5
sub-cycle bench driver (UH-3 carry, UH-5 throw).

Scope, and why it stops where it does. The driver is deliberately split the way
every other rclpy tool in ``tests/hardware/`` is split: a PURE CORE (request
construction, the precondition evaluator, the verdict evaluator, the ballistics)
that imports no ROS, and a thin ``run()`` that imports ``rclpy`` and the
interface package lazily and does nothing but subscribe, call one service and
write rows. Only the pure core is tested — the ROS half is the part a hardware
sitting exercises, and a mocked test of it would assert that the mocks were
called, which is not evidence about a robot.

Several of these tests are DRIFT PINS rather than behaviour tests: the driver
mirrors ``PlanCycle.srv``'s mode/kind enums, the can-bridge's hand guard
constants, the generated hard stop and the coordinator's plan budget / service
wait as module literals (a ``.srv`` and a C++ header are not importable from the
pure core, and importing ``reload_coordinator_node`` would drag rclpy into it —
which ``test_the_pure_core_imports_no_ros_at_module_scope`` forbids), so each
mirrored number is pinned to the file it came from. A mirror that silently
drifts is the failure mode this file exists to make impossible.

Lives under ``tests/ros/`` rather than ``tests/motion/`` because the driver's
subject is the ROS service contract, and the mocked-ROS conftest here is what
makes ``jugglebot_interfaces`` importable for the enum pin.
"""

from __future__ import annotations

import ast
import os
import re
import sys

import pytest

_TESTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO = os.path.dirname(_TESTS)
_HW_DIR = os.path.join(_TESTS, 'hardware')
if _HW_DIR not in sys.path:
    sys.path.insert(0, _HW_DIR)

import unified_cycle_bench as ucb  # noqa: E402


# ═════════════════════════════════════════════════════════════════════════════
# Drift pins — every mirrored constant against the file it was copied from
# ═════════════════════════════════════════════════════════════════════════════

_SRV = os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot_interfaces', 'srv',
                    'PlanCycle.srv')
_CANBRIDGE_CFG = os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot',
                              'Teensy_code_canbridge', 'canbridge_config.h')


def _srv_enums():
    """``{'MODE_NEW': 0, ...}`` parsed out of the .srv text."""
    out = {}
    with open(_SRV) as f:
        for line in f:
            m = re.match(r'\s*uint8\s+(MODE_\w+|KIND_\w+)\s*=\s*(\d+)', line)
            if m:
                out[m.group(1)] = int(m.group(2))
    return out


def test_the_mode_and_kind_enums_match_the_srv():
    """The driver's mirrored enums ARE ``PlanCycle.srv``'s.

    They are mirrored (not imported) so ``--dry-run`` works on a box with no
    colcon build — which is the whole point of a rehearsal mode. That trade is
    only safe while something checks the copy: a silently renumbered kind would
    make the carry rung ask for a LAUNCH, i.e. a throw, with a ball in the cup.
    """
    e = _srv_enums()
    assert e, 'parsed no enums out of %s' % (_SRV,)
    assert (ucb.MODE_NEW, ucb.MODE_EXTEND, ucb.MODE_REPLAN) == (
        e['MODE_NEW'], e['MODE_EXTEND'], e['MODE_REPLAN'])
    assert (ucb.KIND_LAUNCH, ucb.KIND_STEADY, ucb.KIND_LANDING,
            ucb.KIND_SETTLE) == (e['KIND_LAUNCH'], e['KIND_STEADY'],
                                 e['KIND_LANDING'], e['KIND_SETTLE'])


def test_the_request_dicts_name_only_real_srv_fields():
    """Every key the driver sets is a field ``PlanCycle.Request`` actually has.

    ``_Runner.plan`` does ``setattr(req, k, v)`` over the dict, and on a ROS
    message ``setattr`` of an unknown name raises — at the robot, mid-sitting,
    after the preconditions passed. Checking the names here moves that failure
    to the desk.
    """
    from jugglebot_interfaces.srv import PlanCycle
    req = PlanCycle.Request()
    cup = [0.0, 0.0, 689.6]
    for spec in (ucb.carry_request(cup, dx_mm=60.0, dy_mm=0.0),
                 ucb.throw_request(cup, flight_s=0.64)):
        for key in spec:
            if key.startswith('_'):
                continue
            assert hasattr(req, key), (
                '%r is not a PlanCycle.Request field' % (key,))


def test_the_firmware_hand_guards_match_the_header():
    """``MAX_DEVIATION_HAND_REV`` / ``MAX_LEAD_HAND_REV`` are the FW 17 numbers.

    ``MAX_DEVIATION_HAND_REV`` is V4's threshold — the band whose breach is a
    firmware E-STOP — and a stale copy would score a real hardware trip as a
    PASS, which is the one direction that matters. ``MAX_LEAD_HAND_REV`` is
    printed in the runbook and the driver's own docstring as the number that
    bounds a mid-stroke coast (6.4-7.4 rev raw, clamped to 2.0), and sitting two
    measured 1.9847 rev against it — 99.2 %.
    """
    with open(_CANBRIDGE_CFG) as f:
        text = f.read()
    for name, value in (('MAX_DEVIATION_HAND_REV', ucb.MAX_DEVIATION_HAND_REV),
                        ('MAX_LEAD_HAND_REV', ucb.MAX_LEAD_HAND_REV)):
        m = re.search(r'^constexpr\s+float\s+%s\s*=\s*([0-9.]+)f?\s*;' % name,
                      text, re.M)
        assert m, 'no %s in %s' % (name, _CANBRIDGE_CFG)
        assert float(m.group(1)) == pytest.approx(value), name


def test_the_hard_stop_and_gravity_match_the_generated_config():
    """The stroke clip and g are the generated numbers, not remembered ones.

    ``HAND_MOTOR_MAX_POSITION`` is ``GEOM_HAND_MOTOR_HARD_STOP_REVS`` on the
    firmware side (10.8), and g feeds the flight arithmetic the throw rung's
    release velocity is judged against.
    """
    import jugglebot.hardware_config as hw
    assert ucb.HAND_MOTOR_MAX_POSITION_REV == pytest.approx(
        float(hw.GEOM_HAND_MOTOR_HARD_STOP_REVS))
    assert ucb.G_MM_S2 == pytest.approx(float(hw.GRAVITY_MMPS2))
    assert ucb.HAND_CATCH_PRIME_REV == pytest.approx(
        float(hw.JB_OP_HAND_CATCH_PRIME_REV))


def test_the_firmware_clip_sits_BEYOND_the_measured_metal():
    """The clip is a zero-margin alias of the hard stop — and the stop moved.

    2026-09-06, operator, on the machine: stroke 352 mm, bottom -0.107 rev, top
    **10.701 rev**. ``HAND_MOTOR_MAX_POSITION`` is
    ``Geometry::HAND_MOTOR_HARD_STOP_REVS`` with no margin subtracted, so the
    firmware clips setpoints to a position **0.099 rev (3.1 mm) past the metal**
    — and no firmware guard can see the resulting stall: the deviation guard is
    COMMAND-relative (encoder and setpoint agree once the slider is jammed at the
    clip) and the lead clamp anchors the setpoint to the encoder rather than
    refusing it.

    So V3 scores against the MEASURED stop. This pins the gap rather than the
    driver's copy of it: when FW 18 pulls the clip back below the metal, this
    test is what says so.
    """
    gap = ucb.HAND_MOTOR_MAX_POSITION_REV - ucb.HAND_METAL_REV_MEASURED
    assert gap == pytest.approx(0.099, abs=5e-4)
    assert gap > 0.0, (
        'the clip is no longer past the metal — if FW 18 landed, retire this '
        'test and V3 can score against the clip again')
    assert '2026-09-06' in ucb.HAND_METAL_PROVENANCE
    assert '352' in ucb.HAND_METAL_PROVENANCE


def test_the_chained_plan_bar_is_the_COORDINATORS_release_budget():
    """V2's FAIL bar mirrors ``reload_coordinator_node._UNIFIED_PLAN_BUDGET_S``.

    Restated as a literal because importing the coordinator would drag rclpy into
    the pure core, which ``test_the_pure_core_imports_no_ros_at_module_scope``
    forbids; pinned here instead, on the same argument as every other mirror in
    this file. It is load-bearing in one direction: a copy that drifted HIGH
    would pass a rung the production session cannot fly.
    """
    from jugglebot import reload_coordinator_node as rcn
    assert ucb.UNIFIED_PLAN_BUDGET_MS == pytest.approx(
        float(rcn._UNIFIED_PLAN_BUDGET_S) * 1000.0)
    assert ucb.CHAINED_PLAN_ADVISORY_MS < ucb.UNIFIED_PLAN_BUDGET_MS
    assert ucb.SINGLE_PLAN_BAR_MS < ucb.CHAINED_PLAN_ADVISORY_MS


def test_the_service_timeout_is_not_longer_than_the_machine_ever_waits():
    """``--timeout-s`` must not outlast the coordinator's own client wait.

    A bench that waits longer than ``_SERVICE_WAIT_S`` calls a solve "answered"
    that a production session would already have abandoned — with the plan
    installed and streaming either way.
    """
    from jugglebot import reload_coordinator_node as rcn
    default = ucb.build_parser().parse_args(['--rung', 'carry']).timeout_s
    assert default == pytest.approx(float(rcn._SERVICE_WAIT_S))


def test_the_stage_split_note_explains_cont_on_a_JOINED_install():
    """`cont` is documented as a residual and on a join it is a third gate pass.

    ``unified_cycle.STAGE_KEYS`` calls it "the rest of the construction", which
    is honest for one window (a millisecond or two) and misleading for a joined
    install, where it is dominated by the join's own ``validate_cycle`` over the
    concatenated plan — 247 ms measured, 99.7 % of the join. An operator reading
    "residual" goes looking for overhead that is not there.
    """
    note = ucb.stage_wall_note(
        'installed qp=181.2 tilt=3.4 dec=8.1 val=247.9 cont=247.0 ms')
    assert 're-validate' in note
    assert 'cont ~= val' in note
    assert ucb.stage_wall_note('REJECTED_CYCLE_INFEASIBLE(HAND_STROKE: ...)') == ''
    assert ucb.stage_wall_note(None) == ''


def test_the_session_limits_match_the_gates_that_planned_them():
    """250 / 3000 / 150000 — the same triple ``sim/cycle_gate.py`` plans at.

    Read out of the gate source rather than restated, because P5 REFUSES a
    sitting whose limits differ from these: a copy that drifted from the gate's
    would refuse the very limits the sim evidence was produced at.
    """
    src = open(os.path.join(_REPO, 'sim', 'unified_gate.py')).read()
    got = {}
    for key, name in (('vel', '_SESSION_LEG_VEL_MMPS'),
                      ('acc', '_SESSION_LEG_ACC_MMPS2'),
                      ('jerk', '_SESSION_LEG_JERK_MMPS3')):
        m = re.search(r'^%s\s*=\s*([0-9.]+)' % name, src, re.M)
        assert m, name
        got[key] = float(m.group(1))
    assert got['vel'] == pytest.approx(ucb.SESSION_LEG_VEL_MMPS)
    assert got['acc'] == pytest.approx(ucb.SESSION_LEG_ACC_MMPS2)
    assert got['jerk'] == pytest.approx(ucb.SESSION_LEG_JERK_MMPS3)


def test_the_pure_core_imports_no_ros_at_module_scope():
    """``rclpy`` and ``jugglebot_interfaces`` are imported INSIDE functions only.

    An AST check, not a source-text one: the property is "no ROS import runs at
    import time", and that is what makes ``--dry-run`` a true rehearsal on a box
    with a stale colcon build (the case an operator hits when the interfaces
    changed and they have not rebuilt yet).
    """
    src = open(os.path.join(_HW_DIR, 'unified_cycle_bench.py')).read()
    tree = ast.parse(src)
    banned = ('rclpy', 'jugglebot_interfaces', 'diagnostic_msgs', 'std_msgs',
              'geometry_msgs')
    for node in tree.body:                      # module scope ONLY
        names = []
        if isinstance(node, ast.Import):
            names = [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            names = [node.module or '']
        for n in names:
            assert not any(n == b or n.startswith(b + '.') for b in banned), (
                '%s is imported at module scope' % (n,))


# ═════════════════════════════════════════════════════════════════════════════
# The live pose
# ═════════════════════════════════════════════════════════════════════════════

class _Q:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class _P:
    def __init__(self, x=0.0, y=0.0, z=170.0):
        self.x, self.y, self.z = x, y, z


class _PoseMsg:
    def __init__(self, position=None, orientation=None):
        self.position = position or _P()
        self.orientation = orientation or _Q()


def test_the_quaternion_inverts_to_the_planners_rotation_VECTOR():
    """Not Euler angles — ``tilt_geometry.cup_axis(rx, ry)`` reads a rotvec.

    Round-tripped against the forward map rather than against hand-written
    numbers, because the failure this guards is a convention mismatch that is
    invisible at level and grows with tilt: the cup site would be built in one
    frame and solved in another, and the rung would carry the ball somewhere
    other than where the runbook said.
    """
    import math as _m
    import numpy as np
    assert ucb.quat_to_rotvec(0.0, 0.0, 0.0, 1.0) == [0.0, 0.0, 0.0]
    assert ucb.quat_to_rotvec(0.0, 0.0, 0.0, 0.0) == [0.0, 0.0, 0.0]
    for rx, ry, rz in ((0.10, 0.0, 0.0), (0.0, -0.21, 0.0),
                       (0.05, 0.09, -0.03), (0.0, 0.0, 0.4)):
        vec = np.array([rx, ry, rz], dtype=float)
        ang = float(np.linalg.norm(vec))
        axis = vec / ang
        q = list(axis * _m.sin(ang / 2.0)) + [_m.cos(ang / 2.0)]
        got = ucb.quat_to_rotvec(*q)
        assert got == pytest.approx([rx, ry, rz], abs=1e-12)
    # The negative-w hemisphere is the SAME rotation and must not come back as
    # its 2*pi complement.
    q = [0.0, 0.0, 0.0, -1.0]
    assert ucb.quat_to_rotvec(*q) == pytest.approx([0.0, 0.0, 0.0], abs=1e-12)


def test_a_missing_commanded_pose_is_an_ERROR_not_a_default_home():
    """Silence on ``trajectory/commanded_pose`` means "not streaming", by design.

    The node publishes it ONLY while the emitter runs, deliberately, because a
    stale one would let a consumer site a throw at a pose the machine is not
    holding. Substituting a home pose here would do exactly that — and would do
    it in the one state (not activated) where the operator most needs to be told.
    """
    with pytest.raises(ucb.BenchError) as exc:
        ucb.live_pose(None)
    assert '--pose' in str(exc.value)
    pose, note = ucb.live_pose(None, override='1,2,3,0,0,0')
    assert pose == [1.0, 2.0, 3.0, 0.0, 0.0, 0.0] and note == 'from --pose'
    with pytest.raises(ucb.BenchError):
        ucb.live_pose(None, override='1,2,3')


def test_a_tilted_live_pose_is_used_but_FLAGGED():
    """A settled machine is level, so a tilt here is a fact worth surfacing.

    Both rungs are planned from rest and every window's tilt schedule ends
    level, so a non-level commanded pose means something else is going on. It is
    not a refusal — the cup site is still computed correctly from it — but it is
    named in the line the operator reads before the ball goes in.
    """
    pose, note = ucb.live_pose(_PoseMsg(_P(10.0, -4.0, 170.0)))
    assert pose == pytest.approx([10.0, -4.0, 170.0, 0.0, 0.0, 0.0])
    assert 'NOT level' not in note
    import math as _m
    tilted = _PoseMsg(_P(), _Q(x=_m.sin(0.05 / 2.0), w=_m.cos(0.05 / 2.0)))
    pose, note = ucb.live_pose(tilted)
    assert pose[3] == pytest.approx(0.05)
    assert 'NOT level' in note


def test_the_tilt_line_names_the_LEVELLING_CORRECTION_when_one_is_applied():
    """The frame is the whole answer, and it runs the opposite way to the obvious.

    ``trajectory/commanded_pose`` publishes ``_intent_orientation``, which
    REMOVES the levelling correction — so a plan built WITH the correction reads
    0.000 deg here, and a non-zero reading is the correction leaking back out of
    a plan that was seeded (C-LEVEL-1 forbids correcting a seed). Measured
    2026-09-06: offset [0.0126, 0.0011] rad = 0.725 deg, reported as 0.652 deg.

    So on a machine that IS levelled the honest line says "expected", and the
    "NOT level" warning is reserved for the case where no correction is applied
    and a real tilt is the only explanation left. Telling an operator to go
    hunting for a tilt that is the levelling map doing its job is how a rung
    gets abandoned for a healthy reading.
    """
    import math as _m
    tilted = _PoseMsg(_P(), _Q(x=_m.sin(0.0114 / 2.0), w=_m.cos(0.0114 / 2.0)))

    _pose, note = ucb.live_pose(tilted, correction_applied=True,
                                tilt_map_version='2026-08-10-3bf7964f')
    assert 'LEVELLING CORRECTION' in note and 'Expected' in note
    assert '2026-08-10-3bf7964f' in note
    assert 'NOT level' not in note

    _pose, note = ucb.live_pose(tilted, correction_applied=False)
    assert 'NOT level' in note and 'real tilt' in note

    # Status never seen: say so, rather than guessing either way.
    _pose, note = ucb.live_pose(tilted, correction_applied=None)
    assert 'was not read' in note

    # A LEVEL pose is never explained at all, correction or no correction.
    _pose, note = ucb.live_pose(_PoseMsg(_P()), correction_applied=True)
    assert 'LEVELLING CORRECTION' not in note and 'NOT level' not in note


# ═════════════════════════════════════════════════════════════════════════════
# Ballistics
# ═════════════════════════════════════════════════════════════════════════════

def test_flight_for_apex_is_the_actions_own_conversion():
    """``T = sqrt(8h/g)`` — the same map ``TossContinuous.throw_height_m`` uses.

    Pinned so a unified rung at "0.5 m" and a legacy toss at "0.5 m" are the
    same throw. T-H6's acceptance is outcome PARITY with the legacy tier at the
    same site, and parity is meaningless if the two tiers mean different heights.
    """
    assert ucb.flight_for_apex_s(0.5) == pytest.approx(0.63868108, abs=1e-8)
    # Round trip through the release speed: v = gT/2, apex = v^2/(2g) = h.
    for h in (0.2, 0.5, 0.78):
        t = ucb.flight_for_apex_s(h)
        v = ucb.release_speed_for_flight_mm_s(t)
        assert (v ** 2) / (2.0 * ucb.G_MM_S2) == pytest.approx(h * 1000.0)


def test_a_nonpositive_apex_is_refused_not_nan():
    with pytest.raises(ucb.BenchError):
        ucb.flight_for_apex_s(0.0)
    with pytest.raises(ucb.BenchError):
        ucb.flight_for_apex_s(-1.0)


# ═════════════════════════════════════════════════════════════════════════════
# Request construction
# ═════════════════════════════════════════════════════════════════════════════

_CUP = [12.0, -5.0, 689.6]


def test_the_carry_is_a_PURE_LATERAL_settle():
    """UH-3 moves the cup sideways at a FIXED z, and takes no chain.

    The z equality is the rung: a carry that also lifts is not the thing T-H5
    watches a seated ball through. The chain being absent is the other half —
    a SETTLE is already rest-terminal, so there is no release-terminal cliff to
    chain away from.
    """
    spec = ucb.carry_request(_CUP, dx_mm=60.0, dy_mm=-20.0)
    assert spec['mode'] == ucb.MODE_NEW
    assert spec['kind'] == ucb.KIND_SETTLE
    assert spec['chain'] is False
    assert spec['settle_site_mm'] == [72.0, -25.0, 689.6]
    assert spec['settle_site_mm'][2] == pytest.approx(_CUP[2])
    assert spec['banking_enabled'] is True
    # The throw/catch fields are INERT for this kind and are sent as zeros; a
    # zero catch velocity maps to a LEVEL receive tilt, so the zeros are a real
    # no-op rather than a value that happens not to be read.
    assert spec['throw_site_mm'] == [0.0, 0.0, 0.0]
    assert spec['catch_vel_mm_s'] == [0.0, 0.0, 0.0]
    assert spec['flight_s'] == 0.0


def test_the_carry_refuses_a_nonpositive_period():
    with pytest.raises(ucb.BenchError):
        ucb.carry_request(_CUP, dx_mm=60.0, dy_mm=0.0, period_s=0.0)


def test_banking_off_is_reachable_but_never_the_default():
    """`--no-banking` exists as the Phase-0 A/B arm, and is not the default.

    Zero banking cannot plan a steady cycle at session limits (measured
    LIMIT_VEL 529 mm/s against a 250 cap), so a banking-off default would refuse
    every rung; the node warns on the request for the same reason.
    """
    assert ucb.carry_request(_CUP, dx_mm=10.0, dy_mm=0.0)['banking_enabled']
    assert not ucb.carry_request(_CUP, dx_mm=10.0, dy_mm=0.0,
                                 banking=False)['banking_enabled']


def test_the_throw_chains_a_SETTLE_so_the_install_is_rest_terminal():
    """UH-5 installs LAUNCH + chained SETTLE as ONE plan.

    This is the cliff fix, and it is a safety property rather than a
    convenience: a plan that ENDS at a release commands a hard STOP at the throw
    if it is streamed to its end — the emitter's u1 sample lands on the terminal
    hold, so the release segment's transmitted v1 collapses (measured: 93.011
    rev/s emitted as 0.0) and NO guard fires, because the error is well inside
    both hand bands. Chaining makes the first installed plan rest-terminal, so
    the class is removed rather than raced. This driver supersedes nothing, so
    it must never install a release-terminal plan.
    """
    spec = ucb.throw_request(_CUP, flight_s=0.64)
    assert spec['kind'] == ucb.KIND_LAUNCH
    assert spec['chain'] is True
    assert spec['chain_kind'] == ucb.KIND_SETTLE
    assert spec['chain_period_s'] > 0.0
    assert spec['chain_kind'] != ucb.KIND_LAUNCH, (
        'a chained LAUNCH is refused by the service — a launch starts from '
        'REST and a chain starts post-release by construction')


def test_the_throw_is_a_SELF_toss_at_the_release_cup_height():
    """Target == site (vertical), at ``_UNIFIED_THROW_CUP_Z_MM``.

    An AIMED unified throw is out of scope until the aim-authority window is
    re-derived against ``MAX_TILT_DEG`` (12 deg) — a saturated aim lands the
    ball somewhere else and reports nothing — so nothing here invents one.
    """
    spec = ucb.throw_request(_CUP, flight_s=0.64)
    assert spec['throw_site_mm'] == spec['throw_target_mm']
    assert spec['throw_site_mm'][:2] == [_CUP[0], _CUP[1]]
    assert spec['throw_site_mm'][2] == pytest.approx(ucb.THROW_CUP_Z_MM)
    assert spec['settle_site_mm'][2] == pytest.approx(_CUP[2])


def test_the_return_move_uses_the_RECORDED_origin_not_a_fresh_read():
    """The carry goes back to where it started, exactly.

    Re-deriving the return from a second forward-map read would carry the
    outbound's residual into the return, and a repeated pair would then walk.
    """
    out = ucb.carry_request(_CUP, dx_mm=60.0, dy_mm=0.0)
    out['_origin_cup_mm'] = list(_CUP)
    back = ucb.return_request(out)
    assert back['settle_site_mm'] == _CUP
    assert back['kind'] == ucb.KIND_SETTLE
    assert back['period_s'] == out['period_s']


def test_the_return_helper_refuses_anything_but_a_bare_carry():
    with pytest.raises(ucb.BenchError):
        ucb.return_request(ucb.throw_request(_CUP, flight_s=0.64))


# ═════════════════════════════════════════════════════════════════════════════
# The settle-floor lift
#
# A rung's settle site is the live cup opening displaced in xy at the SAME z. Off
# a retracted hand that z is BELOW the planner's cup box, so `_gate_settle_site`
# refuses SETTLE_SITE before the QP runs. Measured 2026-09-06: hand -0.0380 rev
# ⇒ cup 678.68 mm ⇒ three refusals in a row before the operator reached for
# --z-mm. The driver now raises the site instead of minting a request the
# planner is guaranteed to refuse.
# ═════════════════════════════════════════════════════════════════════════════

def test_the_settle_floor_matches_the_planners_own():
    """689.6 mm is ``unified_cycle.SETTLE_CUP_Z_MM``, not a remembered number.

    A copy that drifted below the planner's floor would put the driver back to
    minting refusals; one that drifted above would lift further than it needs to
    and quietly change the height every carry rung is flown at.
    """
    from jugglebot.motion import unified_cycle as uc
    assert ucb.SETTLE_CUP_Z_MM == pytest.approx(float(uc.SETTLE_CUP_Z_MM))


def test_a_carry_off_a_RETRACTED_hand_is_lifted_to_the_floor():
    """The 2026-09-06 site, and the line the operator gets told."""
    cup, note = ucb.lift_cup_to_settle_floor([-0.09, 0.94, 678.68])
    assert cup[2] == pytest.approx(ucb.SETTLE_CUP_Z_MM)
    assert cup[:2] == pytest.approx([-0.09, 0.94])       # xy untouched
    assert '10.9 mm HIGHER' in note and '689.6' in note


def test_a_site_already_inside_the_box_is_untouched_and_silent():
    cup, note = ucb.lift_cup_to_settle_floor([12.0, -5.0, 700.0])
    assert cup == pytest.approx([12.0, -5.0, 700.0])
    assert note == ''


def test_an_explicit_z_override_is_HONOURED_below_the_floor_but_warned():
    """``--z-mm`` is an experiment, and silently moving it would hide a refusal.

    The operator asked for a height; they get that height, plus the prediction
    that the planner is about to refuse it. A driver that quietly corrected the
    number would make the refusal look like a planner bug.
    """
    cup, note = ucb.lift_cup_to_settle_floor([0.0, 0.0, 678.68], z_override=650.0)
    assert cup[2] == pytest.approx(650.0)
    assert 'BELOW the planner floor' in note and 'SETTLE_SITE' in note
    cup, note = ucb.lift_cup_to_settle_floor([0.0, 0.0, 678.68], z_override=690.0)
    assert cup[2] == pytest.approx(690.0) and note == ''


def test_the_return_move_settles_at_the_LIFTED_z_not_the_original():
    """The carry does not try to come back down below the floor.

    The lift happens ONCE, on the cup site both moves are built from, so the
    return's recorded origin is the lifted point by construction — the outbound
    and the return end at the same height and the pair can be repeated.
    """
    cup, _note = ucb.lift_cup_to_settle_floor([-0.09, 0.94, 678.68])
    out = ucb.carry_request(cup, dx_mm=60.0, dy_mm=0.0)
    out['_origin_cup_mm'] = list(cup)
    back = ucb.return_request(out)
    assert out['settle_site_mm'][2] == pytest.approx(ucb.SETTLE_CUP_Z_MM)
    assert back['settle_site_mm'][2] == pytest.approx(ucb.SETTLE_CUP_Z_MM)
    assert back['settle_site_mm'] == pytest.approx(list(cup))


# ═════════════════════════════════════════════════════════════════════════════
# The carry belt — the driver's own read of the planner's hand peak
#
# `plan_cycle` INSTALLS on accept, so the response's `hand_peak_rev` is the last
# chance anything on this side has to notice that a 60 mm sideways carry is about
# to be a 295 mm vertical stroke. It was exactly that on 2026-09-06, and every
# planner gate passed it (`cup_cycle._seed_relaxed_z_box` closed the cause).
# ═════════════════════════════════════════════════════════════════════════════

def test_the_cup_height_to_hand_map_matches_the_planners_own():
    """The mirrored gain and zero, against ``unified_cycle.hand_rev_for_cup_z``.

    The belt's bar is "the hand at the settle height", so a drifted gain or zero
    would move the bar itself — silently, and in the direction that matters
    (a low map reads every plan as excursive, a high one reads a slam as flat).
    """
    from jugglebot.motion import unified_cycle as uc
    for z in (679.6, 689.6, 700.0, 860.0, 984.6):
        assert ucb.hand_rev_for_cup_z(z) == pytest.approx(
            float(uc.hand_rev_for_cup_z(z)), abs=1e-4)


def test_the_carry_belt_refuses_a_plan_that_LIFTS_the_ball():
    """Settle + 1.0 rev is 32 mm of lift on a move that changes no height.

    The message is asserted in the operator's units, not the planner's: a
    refusal that only said "0.5 rev" would be a number nobody at the bench can
    picture against a ball sitting in a cup.
    """
    settle = ucb.hand_rev_for_cup_z(ucb.SETTLE_CUP_Z_MM)
    note = ucb.carry_excursion_refusal(settle + 1.0,
                                       [12.0, -5.0, ucb.SETTLE_CUP_Z_MM])
    assert note
    assert 'lift the hand 31.6 mm' in note
    assert 'REFUSING' in note
    assert '689.6' in note                       # the settle height it compared to


def test_the_carry_belt_passes_the_flat_carry():
    """Settle + 0.1 rev — 3 mm — is the rounding between request and solve.

    The bar has to admit that or the belt refuses every honest carry; the
    2026-09-06 plan lands ON the settle hand (0.0e+00 rev of excess), so this is
    already three millimetres of slack it does not use.
    """
    settle = ucb.hand_rev_for_cup_z(ucb.SETTLE_CUP_Z_MM)
    assert ucb.carry_excursion_refusal(settle + 0.1,
                                       [0.0, 0.0, ucb.SETTLE_CUP_Z_MM]) == ''
    assert ucb.carry_excursion_refusal(settle,
                                       [0.0, 0.0, ucb.SETTLE_CUP_Z_MM]) == ''
    # A hand BELOW the settle is not an excursion — a carry that ends lower than
    # it peaks is the normal shape when the rung was given `--z-mm`.
    assert ucb.carry_excursion_refusal(settle - 2.0,
                                       [0.0, 0.0, ucb.SETTLE_CUP_Z_MM]) == ''


def test_the_carry_belt_reads_the_REQUESTED_settle_height_not_the_constant():
    """``--z-mm`` moves the bar with it, or the belt refuses the experiment.

    The operator can fly the rung at any height inside the box; the bar is "flat
    relative to where this plan was told to settle", which is the only reading
    that survives an override.
    """
    high = ucb.SETTLE_CUP_Z_MM + 100.0
    settle_high = ucb.hand_rev_for_cup_z(high)
    assert ucb.carry_excursion_refusal(settle_high + 0.1,
                                       [0.0, 0.0, high]) == ''
    # ...and the same absolute peak IS a refusal against the lower settle.
    assert ucb.carry_excursion_refusal(settle_high + 0.1,
                                       [0.0, 0.0, ucb.SETTLE_CUP_Z_MM])


def test_the_2026_09_06_slam_would_have_been_refused_by_the_belt():
    """The measured failure, as the driver would have seen it in the response.

    ``hand_peak_rev`` 9.6482 against a 0.3162 rev settle: 295 mm of lift on a
    flat carry, accepted by every planner gate (0.31 rev of stroke headroom
    left). This is the case the belt exists for, so it is asserted with the
    number the bench actually produced rather than a synthetic one.
    """
    note = ucb.carry_excursion_refusal(9.6482, [0.0, 0.0, ucb.SETTLE_CUP_Z_MM])
    assert note and 'lift the hand 295' in note


# ═════════════════════════════════════════════════════════════════════════════
# The precondition evaluator
# ═════════════════════════════════════════════════════════════════════════════

def _good(**kw):
    base = dict(status_age_s=0.2, mode='TRAJECTORY', streaming=True,
                leg_vel=ucb.SESSION_LEG_VEL_MMPS,
                leg_acc=ucb.SESSION_LEG_ACC_MMPS2,
                leg_jerk=ucb.SESSION_LEG_JERK_MMPS3, cycle_active=False,
                robot_state_age_s=0.01,
                hand_axis_state=ucb.AXIS_STATE_CLOSED_LOOP,
                hand_pos_rev=0.3162, link_age_s=0.1, hand_source='STREAMED')
    base.update(kw)
    return ucb.Snapshot(**base)


def _by_id(checks):
    return {c['id']: c for c in checks}


def test_a_healthy_machine_passes_every_precondition():
    checks = ucb.check_preconditions(_good())
    assert [c['id'] for c in checks] == ['P1', 'P2', 'P3', 'P4', 'P5']
    assert all(c['ok'] for c in checks), _by_id(checks)


def test_P1_names_the_state_it_actually_found():
    """P1 has three ways to fail and the operator needs to know which one.

    ``mode='STANDBY'`` is the common one — the robot has not been activated yet
    — and it cost the 2026-09-06 sitting its first attempt against a generic fix
    line that listed all three conditions without saying which had failed.
    """
    standby = _by_id(ucb.check_preconditions(_good(mode='STANDBY')))['P1']
    assert standby['ok'] is False
    assert 'STANDBY' in standby['fix'] and 'ACTIVATE' in standby['fix']
    assert 'runbook step 7' in standby['fix']

    nostatus = _by_id(ucb.check_preconditions(_good(mode=None)))['P1']
    assert 'is the launch up' in nostatus['fix']

    notstreaming = _by_id(
        ucb.check_preconditions(_good(streaming=False)))['P1']
    assert 'NOT streaming' in notstreaming['fix']
    assert 'STALE_STATE' in notstreaming['fix']


@pytest.mark.parametrize('kw,failing', [
    (dict(status_age_s=None), 'P1'),
    (dict(status_age_s=99.0), 'P1'),
    (dict(mode='STANDBY'), 'P1'),
    (dict(streaming=False), 'P1'),
    (dict(robot_state_age_s=None), 'P2'),
    (dict(robot_state_age_s=5.0), 'P2'),
    (dict(hand_axis_state=1), 'P3'),          # IDLE
    (dict(hand_axis_state=None), 'P3'),
    (dict(hand_source='LEGACY_STROKE'), 'P4'),
    (dict(hand_source=None), 'P4'),
    (dict(link_age_s=99.0), 'P4'),
    (dict(leg_jerk=30000.0), 'P5'),           # the SHIPPED default
    (dict(leg_vel=1000.0), 'P5'),
    (dict(leg_acc=None), 'P5'),
])
def test_each_precondition_fails_on_its_own_symptom(kw, failing):
    """One symptom, one named refusal — nothing else moves.

    Parametrised rather than written out because the value is the ISOLATION:
    a check that also fires on a neighbour's symptom sends the operator to the
    wrong fix, which on this ladder means a launch cycle spent on the wrong
    hypothesis.
    """
    checks = _by_id(ucb.check_preconditions(_good(**kw)))
    assert checks[failing]['ok'] is False, checks[failing]
    for cid, c in checks.items():
        if cid != failing:
            assert c['ok'] is True, '%s also failed: %s' % (cid, c)


def test_an_IDLE_hand_is_refused_and_the_fix_says_why_it_looks_healthy():
    """P3's fix names the 2026-09-03 failure, because the symptom is silence.

    A de-energised hand ignores every ``set_input_pos`` the streamed lane sends
    and reads as a PERFECT hold — that is exactly how sitting one's ``hold`` row
    "passed" against an IDLE axis. An operator reading a bare "hand not
    CLOSED_LOOP" has no reason to distrust a flat encoder later.
    """
    c = _by_id(ucb.check_preconditions(_good(hand_axis_state=1)))['P3']
    assert c['ok'] is False
    assert 'IDLE' in c['fix'] and 'perfect hold' in c['fix']


def test_a_LEGACY_latch_refusal_carries_the_launch_down_recovery():
    """P4's fix is the whole recipe, because the latch cannot be moved from here.

    The firmware refuses a ``hand_source`` switch while the setpoint output is
    armed, so the only route is launch DOWN + ``hand_stream_bench.py
    --source-only streamed``. A refusal that just says "latch is LEGACY" leaves
    the operator to rediscover that.
    """
    c = _by_id(ucb.check_preconditions(_good(hand_source='LEGACY_STROKE')))['P4']
    assert c['ok'] is False
    assert 'hand_stream_bench.py --source-only streamed' in c['fix']
    assert 'DISCARDS' in c['fix']


def test_the_limits_refusal_hands_over_the_exact_service_call():
    c = _by_id(ucb.check_preconditions(_good(leg_jerk=30000.0)))['P5']
    assert c['ok'] is False
    assert 'ros2 service call /trajectory/set_limits' in c['fix']
    assert '150000.0' in c['fix'] and '250.0' in c['fix']
    assert ucb.SET_LIMITS_CMD in c['fix']


def test_the_freshness_bound_is_a_parameter_not_a_literal():
    """A slow box may need a wider window; the check must widen, not be skipped."""
    snap = _good(status_age_s=3.0, robot_state_age_s=3.0, link_age_s=3.0)
    assert not all(c['ok'] for c in ucb.check_preconditions(snap))
    assert all(c['ok'] for c in ucb.check_preconditions(snap, max_age_s=5.0))


# ═════════════════════════════════════════════════════════════════════════════
# The verdict evaluator
# ═════════════════════════════════════════════════════════════════════════════

def _plan(**kw):
    base = dict(accepted=True, code='OK', message='ok', duration_s=1.4,
                plan_wall_ms=180.0, hand_peak_rev=0.35,
                hand_peak_vel_rps=11.2, release_terminal=False,
                release_vel_mm_s=[0.0, 0.0, 3131.5], chained=False)
    base.update(kw)
    return base


def _trace(n=40, cmd=0.32, enc=0.32, state=ucb.AXIS_STATE_CLOSED_LOOP,
           active=True, ran_out=True, gap_ms=24.0):
    """A healthy run's trace.

    ``ran_out`` shapes the LAST row the way a finished cycle actually looks:
    ``plan_time_remaining_s`` at 0.0 with ``cycle_active`` STILL True. That is
    not a stylistic choice — ``cycle_active`` is a type test on the installed
    plan and is never cleared at expiry, so the pre-2026-09-06 shape (last row
    ``cycle_active=False``) was a run that had been SUPERSEDED, which is what
    this driver's own ``hold`` abort produces.
    """
    rows = [{'t': i * 0.05, 'hand_cmd_rev': cmd, 'hand_enc_rev': enc,
             'hand_vel_rps': 0.0, 'hand_axis_state': state,
             'max_emit_gap_ms': gap_ms,
             'plan_time_remaining_s': max(0.0, 1.4 - i * 0.05),
             'cycle_active': active} for i in range(n)]
    if rows:
        rows[-1]['plan_time_remaining_s'] = 0.0 if ran_out else 0.4
    return rows


def _v(checks):
    return {c['id']: c['verdict'] for c in checks}


def test_a_clean_carry_scores_all_PASS():
    checks = ucb.evaluate('carry', _plan(), _trace(), legacy_duration_s=1.5)
    assert ucb.verdict_rc(checks) == 0
    assert _v(checks) == {'V1': 'PASS', 'V2': 'PASS', 'V2b': 'PASS',
                          'V3': 'PASS', 'V4': 'PASS', 'V5': 'PASS',
                          'V6': 'PASS', 'V7': 'PASS'}


def test_a_refused_plan_fails_V1_and_carries_the_planners_own_words():
    checks = ucb.evaluate(
        'carry',
        _plan(accepted=False, code='REJECTED_CYCLE_INFEASIBLE',
              message='REJECTED_CYCLE_INFEASIBLE(LIMIT_JERK: 155123 > 150000)'),
        _trace(), legacy_duration_s=1.5)
    v = _by_id(checks)['V1']
    assert v['verdict'] == 'FAIL'
    assert 'LIMIT_JERK' in v['detail'], (
        'the planner refusal must reach the operator VERBATIM — the subcode is '
        'which layer said no')
    assert ucb.verdict_rc(checks) == 1


def test_tracking_past_the_deviation_band_FAILS_and_says_it_is_a_ceiling():
    """V4 is a CEILING on the residual, and the detail has to say so.

    The compare is against the 10-45 ms-stale telemetry cache with no velocity
    compensation, so at the throw's ~99 rev/s plateau it over-reads by pure
    latency. A number UNDER the band is therefore real evidence; a number over
    it needs the ``[hand7]`` console line to adjudicate, and an operator who
    does not know that will abort a healthy sitting.
    """
    checks = ucb.evaluate('carry', _plan(),
                          _trace(cmd=0.32, enc=0.32 - 3.0),
                          legacy_duration_s=1.5)
    c = _by_id(checks)['V4']
    assert c['verdict'] == 'FAIL'
    assert 'CEILING' in c['detail']
    ok = _by_id(ucb.evaluate('carry', _plan(), _trace(cmd=0.32, enc=0.32 - 2.4),
                             legacy_duration_s=1.5))['V4']
    assert ok['verdict'] == 'PASS'


def test_a_hand_that_dropped_out_of_CLOSED_LOOP_mid_run_FAILS():
    """The energised check is per TICK, not only at entry.

    Sitting one's whole failure was a hold that "passed" de-energised; a
    de-energisation partway through a stroke looks identical.
    """
    trace = _trace()
    trace[10]['hand_axis_state'] = 1
    assert _by_id(ucb.evaluate('carry', _plan(), trace,
                               legacy_duration_s=1.5))['V5']['verdict'] == 'FAIL'


def test_a_tick_with_NO_axis_state_is_not_evidence_that_the_hand_was_energised():
    """V5 fails closed on MISSING data, not just on a state that says IDLE.

    `hand_axis_state` is None whenever `/robot_state` carries six axes instead
    of seven — a shape the whole stack handles, and a shape this driver can meet
    mid-run. Excluding those ticks from the idle set (as this did until
    2026-09-05) makes "hand stayed energised" PASS on a run that never saw the
    axis at all, which is sitting one's failure with the evidence removed rather
    than the fault fixed.
    """
    trace = _trace()
    trace[10]['hand_axis_state'] = None
    c = _by_id(ucb.evaluate('carry', _plan(), trace, legacy_duration_s=1.5))['V5']
    assert c['verdict'] == 'FAIL'
    assert 'NO axis state' in c['detail'], c
    # Every tick blind is the loudest version of the same thing.
    blind = _trace(state=None)
    assert _by_id(ucb.evaluate('carry', _plan(), blind,
                               legacy_duration_s=1.5))['V5']['verdict'] == 'FAIL'


def test_ticks_with_no_ENCODER_reading_SKIP_V4_rather_than_scoring_zero():
    """A deviation of 0.0000 rev computed over nothing is not a passing hand.

    `ticks` is keyed on the COMMAND, so a run in which `/hand_telemetry` carried
    commands but no encoder would leave `worst` at its 0.0 initial value and
    read as the best tracking the bench has ever measured.
    """
    trace = [dict(r, hand_enc_rev=None) for r in _trace()]
    c = _by_id(ucb.evaluate('carry', _plan(), trace, legacy_duration_s=1.5))['V4']
    assert c['verdict'] == 'SKIP'
    assert 'NONE carrying an encoder' in c['detail'], c
    # A partially blind trace still scores, and says over how many ticks.
    part = _trace()
    for r in part[:10]:
        r['hand_enc_rev'] = None
    c = _by_id(ucb.evaluate('carry', _plan(), part, legacy_duration_s=1.5))['V4']
    assert c['verdict'] == 'PASS'
    assert 'over 30 of 40 ticks' in c['detail'], c


def test_a_last_tick_with_no_status_is_not_read_as_the_cycle_FINISHING():
    """V6 needs POSITIVE evidence that the window ran out.

    `plan_time_remaining_s` and `cycle_active` are both None on a tick where
    `trajectory/status` had not been seen, and neither says anything about
    whether the window ended. The cycle may still be installed — and this driver
    supersedes nothing, which is precisely why that shape has to be a FAIL.
    """
    trace = _trace()
    trace[-1]['cycle_active'] = None
    trace[-1]['plan_time_remaining_s'] = None
    c = _by_id(ucb.evaluate('carry', _plan(), trace, legacy_duration_s=1.5))['V6']
    assert c['verdict'] == 'FAIL'
    assert 'plan_time_remaining_s=None' in c['detail'], c


def test_V6_reads_a_STILL_INSTALLED_expired_cycle_as_the_clean_finish():
    """`cycle_active` is never cleared at expiry, so requiring False is a bug.

    It is `isinstance(active_plan, CyclePlan)` — and a REST-terminal cycle IS the
    streaming plan, holding its terminal pose until something supersedes it. The
    old V6 could therefore only PASS on a run that had been superseded, which for
    this driver means its own `hold` abort ran. Sitting three's UH-3 carry hit
    exactly that: the window expired cleanly and V6 said FAIL.
    """
    finished = _by_id(ucb.evaluate('carry', _plan(), _trace(ran_out=True),
                                   legacy_duration_s=1.5))['V6']
    assert finished['verdict'] == 'PASS'
    # SUPERSEDED (cycle_active False at the end) is NOT the clean finish — it is
    # the abort path, and it must not read as success.
    superseded = _trace(ran_out=True)
    superseded[-1]['cycle_active'] = False
    c = _by_id(ucb.evaluate('carry', _plan(), superseded,
                            legacy_duration_s=1.5))['V6']
    assert c['verdict'] == 'FAIL'
    assert 'SUPERSEDED' in c['detail']


def test_V3_scores_the_ENCODER_against_metal_not_the_commanded_peak():
    """The command cannot reach metal; the slider can.

    Two gates already bound the COMMAND — ``validate_cycle`` refuses a plan
    outside [0, JB_OP_HAND_CATCH_PRIME_REV] (9.9594 rev) with ``HAND_STROKE``,
    and the firmware clips every setpoint to [0, 10.8] after that — so a verdict
    on ``hand_peak_rev`` would restate two gates and could never fail. What CAN
    reach metal is the physical slider overshooting its command, and sitting two
    measured exactly that: encoder 10.4693 rev against the 10.8 hard stop, a
    0.33 rev margin, on a legacy stroke at --event-vel 3.0.

    The nominal UH-5 throw's commanded peak is 9.643 rev (probe, 2026-09-05,
    0.5 m apex at session limits) — 1.16 rev clear of metal — and must not fail
    this check, which the old "peak + one full lead clamp" formulation did.
    """
    # Sitting two's own number: PASSES at the default 0.20 rev margin, and is
    # visibly tight in the detail.
    c = _by_id(ucb.evaluate('throw', _plan(hand_peak_rev=9.643),
                            _trace(enc=10.4693)))['V3']
    assert c['verdict'] == 'PASS'
    # 0.2317 rev to the MEASURED 10.701 stop (it read 0.33 while the reference
    # was the firmware's 10.8 clip, which is 3.1 mm past the metal).
    assert '0.2317' in c['detail']
    assert '9.643' in c['detail'], 'the commanded margin is REPORTED beside it'
    # Into metal: FAIL.
    assert _by_id(ucb.evaluate('throw', _plan(hand_peak_rev=9.643),
                               _trace(enc=10.75)))['V3']['verdict'] == 'FAIL'
    # And the bar is a knob, because the tier ramp is what moves it.
    assert _by_id(ucb.evaluate('throw', _plan(hand_peak_rev=9.643),
                               _trace(enc=10.4693),
                               metal_margin_rev=0.5))['V3']['verdict'] == 'FAIL'


def test_V3_skips_with_no_encoder_ticks_but_still_reports_the_command():
    c = _by_id(ucb.evaluate('throw', _plan(hand_peak_rev=9.643), []))['V3']
    assert c['verdict'] == 'SKIP'
    assert 'commanded peak 9.6430' in c['detail']


def test_the_chained_install_is_judged_against_the_COORDINATORS_release_budget():
    """The chained bar is the number that decides whether a SESSION can fly.

    Past `_UNIFIED_PLAN_BUDGET_S` the production coordinator does not log a slow
    solve — it misses its release, the skew eats the 0.5 s grace, and the cycle
    terminalises ABORTED_NO_RELEASE with the ball in the air. So a joined install
    slower than that has proven the machine cannot run a session at these
    settings, whatever else the rung showed. The old bar was a bare 500 ms
    literal: under the budget that matters and over the cost that is normal.
    """
    # Sitting three's joined solves are 500-600 ms — PASS, and QUIET: that is
    # the cost this shape actually has, so it must not read as a deviation.
    quiet = _by_id(ucb.evaluate('throw', _plan(plan_wall_ms=560.0, chained=True),
                                []))['V2']
    assert quiet['verdict'] == 'PASS'
    assert 'ADVISORY' not in quiet['detail']
    # Past the advisory but under the budget: PASS, and says so out loud.
    c = _by_id(ucb.evaluate('throw', _plan(plan_wall_ms=850.0, chained=True),
                            []))['V2']
    assert c['verdict'] == 'PASS'
    assert 'ADVISORY' in c['detail']
    # Past the coordinator's budget: FAIL.
    assert _by_id(ucb.evaluate(
        'throw', _plan(plan_wall_ms=ucb.UNIFIED_PLAN_BUDGET_MS + 1.0,
                       chained=True), []))['V2']['verdict'] == 'FAIL'
    # A SINGLE window is still judged against the owner's 250 ms total.
    assert _by_id(ucb.evaluate('carry', _plan(plan_wall_ms=424.0),
                               []))['V2']['verdict'] == 'FAIL'


def test_V2b_FAILS_a_run_whose_solve_GAPPED_the_emitter():
    """The verdict that would have caught the 2026-09-06 UH-3 E-STOP.

    That run's solve evicted the 40 Hz emitter for 225-942 ms and latched the
    can-bridge's 250 ms setpoint watchdog — while every other verdict on it read
    healthy. 125 ms is five nominal ticks and half the watchdog.
    """
    ok = _by_id(ucb.evaluate('carry', _plan(), _trace(gap_ms=40.0),
                             legacy_duration_s=1.5))['V2b']
    assert ok['verdict'] == 'PASS'
    bad = _trace(gap_ms=26.0)
    bad[17]['max_emit_gap_ms'] = 804.0            # sitting three's own sp_age_ms
    c = _by_id(ucb.evaluate('carry', _plan(), bad, legacy_duration_s=1.5))['V2b']
    assert c['verdict'] == 'FAIL'
    assert '804.0' in c['detail']
    # A trace with no such column SKIPS rather than passing vacuously.
    blind = [dict(r, max_emit_gap_ms=None) for r in _trace()]
    assert _by_id(ucb.evaluate('carry', _plan(), blind,
                               legacy_duration_s=1.5))['V2b']['verdict'] == 'SKIP'


def test_the_release_velocity_check_compares_the_plan_with_the_ballistics():
    flight = ucb.flight_for_apex_s(0.5)
    want = ucb.release_speed_for_flight_mm_s(flight)
    good = _plan(release_vel_mm_s=[0.0, 0.0, want * 1.02])
    good['_expected_release_mm_s'] = want
    assert _by_id(ucb.evaluate('throw', good, _trace()))['V7']['verdict'] == 'PASS'
    bad = _plan(release_vel_mm_s=[0.0, 0.0, want * 1.20])
    bad['_expected_release_mm_s'] = want
    c = _by_id(ucb.evaluate('throw', bad, _trace()))['V7']
    assert c['verdict'] == 'FAIL'
    assert 'bag-measured' in c['detail'], (
        'the operator must not read this as the T-H6 achieved-velocity check'
    )


def test_a_missing_legacy_duration_SKIPS_rather_than_passing_T_H5():
    """An uncomputable comparison is a SKIP, never a silent PASS.

    T-H5's numeric half is "re-pose duration <= the legacy GoToPose duration for
    the same re-pose at lean_gain 0". If the offline computation did not run
    there is no bar, and a PASS there would be an unearned one.
    """
    c = _by_id(ucb.evaluate('carry', _plan(), _trace(),
                            legacy_duration_s=None))['V7']
    assert c['verdict'] == 'SKIP'
    assert 'build_move' in c['detail']


def test_the_legacy_duration_reads_total_duration_and_REPORTS_why_it_failed():
    """`TrajectoryPlan` has `total_duration`, not `duration`.

    On 2026-09-06 the bare `except Exception` turned that AttributeError into
    "motion package not importable here" and sent the operator to check their
    colcon build. The field name is fixed and the exception's repr now rides in
    the SKIP detail, so the next failure names itself.
    """
    from jugglebot.motion.trajectory.plan import TrajectoryPlan
    assert 'total_duration' in TrajectoryPlan.__slots__
    assert 'duration' not in TrajectoryPlan.__slots__
    # A failure inside the helper is RECORDED, and the SKIP quotes it.
    ucb.legacy_move_duration_s('not a pose', 'nor this', (1.0, 1.0, 1.0))
    assert ucb._LEGACY_DURATION_ERR, 'the reason was swallowed again'
    c = _by_id(ucb.evaluate('carry', _plan(), _trace(),
                            legacy_duration_s=None))['V7']
    assert ucb._LEGACY_DURATION_ERR[0] in c['detail']


def test_an_empty_trace_SKIPS_the_telemetry_checks_and_does_not_fail_them():
    checks = _by_id(ucb.evaluate('carry', _plan(), []))
    assert checks['V4']['verdict'] == 'SKIP'
    assert checks['V5']['verdict'] == 'SKIP'
    assert checks['V6']['verdict'] == 'SKIP'
    assert checks['V2b']['verdict'] == 'SKIP'
    assert ucb.verdict_rc(ucb.evaluate('carry', _plan(), [])) == 0


def test_a_window_still_RUNNING_at_the_last_tick_FAILS_V6():
    """The driver stopped watching while the window was still open.

    Either the watch was too short or the plan is longer than its response said,
    and this tool supersedes nothing — so the machine is still moving after the
    run ended, which is the shape that matters.
    """
    trace = _trace(ran_out=False)
    c = _by_id(ucb.evaluate('carry', _plan(), trace, legacy_duration_s=1.5))['V6']
    assert c['verdict'] == 'FAIL'
    assert 'ran out=False' in c['detail']


def test_verdict_rc_is_zero_iff_no_FAIL():
    assert ucb.verdict_rc([{'verdict': 'PASS'}, {'verdict': 'SKIP'}]) == 0
    assert ucb.verdict_rc([{'verdict': 'PASS'}, {'verdict': 'FAIL'}]) == 1


# ═════════════════════════════════════════════════════════════════════════════
# The CLI surface
# ═════════════════════════════════════════════════════════════════════════════

def test_dry_run_makes_no_ROS_calls_and_prints_both_rungs(capsys):
    """``--dry-run`` is the rehearsal: it must reach a printed request.

    Exercised for both rungs because the request builders diverge completely,
    and a rehearsal that only covers one of them is a rehearsal for one.
    """
    for argv in (['--rung', 'carry', '--dry-run', '--dx', '60'],
                 ['--rung', 'throw', '--dry-run', '--apex', '0.5']):
        assert ucb.main(argv) == 0
        out = capsys.readouterr().out
        assert 'no ROS calls made' in out
        assert 'settle_site_mm' in out
        assert ucb.SET_LIMITS_CMD in out


def test_flight_and_apex_are_inverse_on_the_command_line(capsys):
    """``--flight`` overrides ``--apex`` and back-computes the apex it implies.

    So a rung driven by a flight time still reports the height it will throw,
    which is what the T-H6 "h <= 0.5 m" advisory keys on.
    """
    ucb.main(['--rung', 'throw', '--dry-run', '--flight', '0.6386810803'])
    out = capsys.readouterr().out
    assert 'apex 0.500 m' in out


def test_the_rung_is_required():
    with pytest.raises(SystemExit):
        ucb.build_parser().parse_args([])
