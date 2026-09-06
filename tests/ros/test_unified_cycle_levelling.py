"""Contract row E8 at the NODE — the unified cycle's levelling frame.

WHAT THIS FILE DEFENDS
----------------------
``tests/motion/test_unified_cycle.py`` pins the frame *algebra*: given a
correction, ``unified_cycle._realize`` puts the release on gravity-level and
leaves knot 0 on the seed's own float.  Nothing there can see whether the NODE
ever hands the planner a correction, or whether it hands it the right one — and
that is precisely the half that was missing until 2026-09-06.  The bug was not a
wrong number anywhere; it was that ``grep -rn "levelling\\." `` over the cycle
machinery returned **nothing**, for four months, because the C-LEVEL-1
enumeration was phrased as "poses that enter" and a cycle has none.

So this file drives ``TrajectoryNode`` end to end and asserts on the INSTALLED
plan:

1. ``_cycle_start_state`` **builds** the correction for the seed — through
   ``correction_for_pose`` and ``_active_tilt_map()``, so the C-LEVEL-2 residual
   is in it and the dormancy gate is honoured — and carries it on the state.
2. A LAUNCH from the levelled prepare pose commands a release that is
   **gravity-level**, with **no physical tilt step** at knot 0.
3. The frame is carried through a CHAINED install and a REPLAN rather than
   re-read, which is C-LEVEL-1's in-flight rule.
4. The legacy ingest path is untouched.

The structural half — that a future cycle path which forgets the correction
fails loudly — is ``tests/ros/test_levelling_frame.py``'s manifest (rows
``build:E8`` / ``apply:E8`` / ``egress:E8`` and ``_CARRIED_BUILDS``).

Contract: ``ros_ws/docs/levelling_frame.md`` s "E8 — the unified cycle".
Measurements cited below are ``/tmp/probe_level.py`` (uncommitted, venv
interpreter, 2026-09-06), run more than once with identical output, against bag
``~/Desktop/rosbags/2026-09-06_19-*``.

ROS 2 is mocked by ``tests/ros/conftest.py``.  Nothing binds a port or touches
the filesystem outside ``tmp_path``, so the file is xdist-parallel-safe.
"""

from __future__ import annotations

import math
import time

import numpy as np
import pytest
import yaml

from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import Float64MultiArray, String

from jugglebot_interfaces.msg import MotorStateSingle, RobotState
from jugglebot_interfaces.srv import GoToPose, PlanCycle, SetTrajectoryLimits

import jugglebot.hardware_config as hw
from jugglebot.motion import levelling
from jugglebot.motion import tilt_map as tm
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.ik_solver import rotvec_to_rot_matrix
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.trajectory_node import TrajectoryNode


# ── The measured operating point ─────────────────────────────────────────────

#: The 2026-09-06 correction.  The bag's commanded prepare attitude was
#: rx = -11.663 mrad and the ingest writes ``rotvec(R) = [-tx, -ty, 0]``, so the
#: measured offset behind it is +0.011663 rad about x.
_OFFSET = (0.011663, 0.0)

_ACTIVATE_REV = list(hw.JB_OP_ACTIVATE_POSITION_REVS)

#: Session limits a unified sitting raises to at start (plan Phase 1, owner
#: decision 1) — the ones `sim/cycle_gate.py` and `tests/motion/test_unified_cycle`
#: run at.  Planning against the SHIPPED jerk would read LIMIT_JERK for a known
#: structural reason and test that fact instead of this wiring.
_SESSION_VEL, _SESSION_ACC, _SESSION_JERK = 250.0, 3000.0, 150000.0

_REST_CUP_Z_MM = 750.0
_THROW_CUP_Z_MM = 860.0

#: A vertical throw to a 0.5 m apex: ``T = 2*sqrt(2h/g)``.
_APEX_M = 0.5
_FLIGHT_S = 2.0 * math.sqrt(2.0 * _APEX_M / 9.806)

# An asymmetric residual grid — a transposed map is invisible on a symmetric
# one, and the whole point of a `build:` row (rather than a `store`) is that the
# residual at THIS pose is read, so the map must be able to disagree with itself.
_X_MM = [-150.0, 0.0, 150.0]
_Y_MM = [-150.0, 0.0, 150.0]
_TX = [[-0.0100, -0.0040, 0.0020],
       [-0.0080, 0.0000, 0.0085],
       [0.0010, 0.0060, 0.0140]]
_TY = [[0.0120, 0.0070, -0.0030],
       [0.0055, 0.0000, -0.0062],
       [-0.0020, -0.0090, -0.0150]]


class _CapturePub:
    def __init__(self):
        self.frames = []

    def send(self, msg):
        self.frames.append(msg)

    def close(self):
        pass


def _hand_rev_for_cup_z(cup_z_mm: float) -> float:
    """The slider rev whose LEVEL realisation puts the cup opening at ``cup_z_mm``.

    Built from the level relation ``cup_z = CUP_Z_BASE_MM + slider_mm`` rather
    than by inverting the forward map, so the fixture owes nothing to the map
    these tests exercise.
    """
    cfg = cr.RealizeConfig()
    slider_mm = float(cup_z_mm) - cfg.cup_z_base_mm
    return (slider_mm - cfg.slider_rev_zero_mm) / 1000.0 * cr.LINEAR_GAIN_REV_PER_M


_REST_HAND_REV = _hand_rev_for_cup_z(_REST_CUP_Z_MM)


def _robot_state(hand_rev=_REST_HAND_REV):
    """A SEVEN-axis robot_state: six legs at the ACTIVE pose plus the hand."""
    rs = RobotState()
    rs.motor_states = [MotorStateSingle(pos_estimate=float(_ACTIVATE_REV[i]))
                       for i in range(6)]
    rs.motor_states.append(MotorStateSingle(pos_estimate=float(hand_rev)))
    rs.is_homed = True
    return rs


def _node(offset=_OFFSET):
    """A seeded TRAJECTORY-mode node at session limits, emitter NOT started."""
    node = TrajectoryNode(command_pub_factory=_CapturePub, start_emitter=False)
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='TRAJECTORY'))
    req = SetTrajectoryLimits.Request()
    req.leg_vel_limit_mmps = _SESSION_VEL
    req.leg_acc_limit_mmps2 = _SESSION_ACC
    req.leg_jerk_limit_mmps3 = _SESSION_JERK
    node._svc_set_limits(req, SetTrajectoryLimits.Response())
    if offset is not None:
        node._on_gravity_offset(Float64MultiArray(data=list(offset)))
    return node


def _map_doc():
    return {
        'version': 1,
        'captured': {'date': '2026-08-10', 'git_sha': '3bf7964f',
                     'tool': 'unit-test', 'uptime_ms_first': 1000,
                     'uptime_ms_last': 2000,
                     'level_offset_rad': list(_OFFSET),
                     'base_condition': 'bench, unshimmed'},
        'grid': {'z_mm': 170.0, 'orientation': 'level',
                 'x_mm': list(_X_MM), 'y_mm': list(_Y_MM)},
        'residual_rad': {'tx': [list(r) for r in _TX],
                         'ty': [list(r) for r in _TY]},
    }


def _mapped_node(monkeypatch, tmp_path, offset=_OFFSET):
    path = tmp_path / 'tilt_calibration.yaml'
    path.write_text(yaml.safe_dump(_map_doc()))
    monkeypatch.setenv('JUGGLEBOT_TILT_CAL', str(path))
    node = _node(offset=offset)
    assert node._tilt_map is not None, 'fixture failed to load its own map'
    return node


def _prepare(node, x=0.0, y=0.0, z=170.0):
    """Leave the node HOLDING the levelled prepare pose, the way PREPARE does.

    A `go_to_pose` whose INTENT orientation is level; the E3 ingest corrects it,
    so the plan's terminal — and therefore what `_current_state` samples — is the
    PLAN-frame pose, tilted by the correction.  The origin is then rewound past
    the move so the machine is stationary at that pose, which is the state the
    coordinator issues `plan_cycle` from.
    """
    req = GoToPose.Request()
    req.pose = Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion())
    req.duration_s = 2.0
    req.lean_gain = 0.0
    assert node._svc_go_to_pose(req, GoToPose.Response()).accepted is True
    node._plan_t0 = time.perf_counter() - 10.0
    node._on_robot_state(_robot_state())
    return node


def _launch_req(period_s=0.6, chain=False):
    req = PlanCycle.Request()
    req.mode = req.MODE_NEW
    req.kind = req.KIND_LAUNCH
    req.period_s = period_s
    req.throw_site_mm = [0.0, 0.0, _THROW_CUP_Z_MM]
    req.throw_target_mm = [0.0, 0.0, _THROW_CUP_Z_MM]
    req.flight_s = _FLIGHT_S
    req.catch_site_mm = [0.0, 0.0, 830.0]
    req.catch_vel_mm_s = [0.0, 0.0, -2500.0]
    req.catch_frac = 0.0
    req.settle_site_mm = [0.0, 0.0, _REST_CUP_Z_MM]
    req.banking_enabled = True
    req.lead_s = 0.0
    if chain:
        req.chain = True
        req.chain_kind = req.KIND_LANDING
        req.chain_period_s = 1.0
        req.chain_catch_frac = _FLIGHT_S / 1.0
    return req


def _physical(pose6, correction):
    """The platform's attitude against GRAVITY for a commanded plan-frame pose.

    ``R_physical = R_gravity^T @ R_commanded`` — the exact inverse of what the
    ingest composed.  This is the frame the ball leaves in, and it is the only
    frame in which "level" means anything to a thrown ball.
    """
    return levelling.uncorrect_pose(np.asarray(pose6, dtype=float),
                                    correction)[3:6]


def _aim_mrad(pose6, correction):
    """Angle between the PHYSICAL cup axis and world +z, milliradians."""
    axis = rotvec_to_rot_matrix(_physical(pose6, correction)) @ np.array(
        [0.0, 0.0, 1.0])
    return 1e3 * float(np.arccos(np.clip(float(axis[2]), -1.0, 1.0))), axis


# ═════════════════════════════════════════════════════════════════════════════
# 1. The node BUILDS the correction and CARRIES it
# ═════════════════════════════════════════════════════════════════════════════

def test_the_cycle_seed_carries_a_correction_built_for_its_own_pose():
    """`_cycle_start_state` builds E8's correction, not the stored C-LEVEL-1 one.

    With no map loaded the two are numerically identical, which is exactly why
    the *next* test exists — this one pins the plumbing (a correction arrives at
    all, and it is the right matrix) and the mapped one pins that it was BUILT
    rather than fetched.
    """
    node = _prepare(_node())
    state, code, err = node._cycle_start_state(uc.LAUNCH)
    assert state is not None, (code, err)
    assert state.levelling_correction is not None
    assert np.allclose(state.levelling_correction,
                       levelling.correction_from_offset(*_OFFSET), atol=1e-15)


def test_the_correction_is_BUILT_at_the_seed_and_carries_the_map_residual(
        monkeypatch, tmp_path):
    """A `store` would apply the HOME node's residual to the whole workspace.

    This is the C-LEVEL-2 failure the manifest's `build:` / `store` distinction
    exists to hold, arriving at a new surface.  Seed the cycle at a DISPLACED
    pose whose map residual is non-zero, and the carried correction must differ
    from the node's stored `_gravity_correction` by exactly the residual at that
    pose — not at (0, 0), where the map is zero by construction.
    """
    node = _mapped_node(monkeypatch, tmp_path)
    _prepare(node, x=90.0, y=-60.0)
    state, code, err = node._cycle_start_state(uc.LAUNCH)
    assert state is not None, (code, err)

    seed_pose, _, _ = node._current_state()
    residual = tm.lookup(node._tilt_map, float(seed_pose[0]),
                         float(seed_pose[1]))
    assert not np.allclose(residual, 0.0), 'fixture pose has no residual to find'
    expected = levelling.correction_from_offset(_OFFSET[0] + residual[0],
                                                _OFFSET[1] + residual[1])
    assert np.allclose(state.levelling_correction, expected, atol=1e-15)
    # ...and it is NOT the stored single-offset correction: a `store` here would
    # pass every one-pose test and quietly ignore the calibration everywhere.
    assert not np.allclose(state.levelling_correction,
                           node._gravity_correction, atol=1e-9)


def test_an_unlevelled_node_holds_a_loaded_map_DORMANT(monkeypatch, tmp_path):
    """No level reference ⇒ identity, even with a map loaded.

    `_active_tilt_map()` is the dormancy gate (C-LEVEL-2 s "The map is gated on
    the level reference"): composing a residual onto the (0, 0) placeholder
    commands a rotation referenced to nothing — differently wrong, not less
    wrong.  Passing `self._tilt_map` directly instead of the accessor would
    re-create that at exactly this new surface, and neither the structural
    manifest (which keys on the call, not its arguments) nor a one-pose
    behavioural test would notice.
    """
    node = _mapped_node(monkeypatch, tmp_path, offset=None)
    assert node._tilt_map_loaded is True
    assert node._gravity_correction_loaded is False
    _prepare(node, x=90.0, y=-60.0)
    state, code, err = node._cycle_start_state(uc.LAUNCH)
    assert state is not None, (code, err)
    assert np.allclose(state.levelling_correction, np.eye(3), atol=1e-15)


def test_a_degraded_map_lookup_degrades_the_cycle_to_offset_only(monkeypatch,
                                                                 tmp_path):
    """A raising lookup must not kill the service — C-LEVEL-2's degrade path.

    `tilt_map.lookup` raises on a non-finite query.  Every other ingest catches
    it and falls back to the stored offset through the shared
    `_tilt_map_degraded`; this surface does the same, so a NaN reaches
    `feasibility` (which refuses it loudly) rather than taking the executor down
    inside a service callback.
    """
    node = _mapped_node(monkeypatch, tmp_path)
    # DISPLACED, so the degraded answer is numerically distinguishable from the
    # healthy one: at the home node the map is zero by construction and this test
    # would pass on a code path that never degraded at all.
    _prepare(node, x=90.0, y=-60.0)

    def _boom(*_a, **_kw):
        raise tm.TiltMapError('synthetic non-finite query')

    # Patched on `levelling`, not on `tilt_map`: `levelling.py` binds the lookup
    # directly (`from ...tilt_map import lookup as lookup_tilt_residual`), so
    # patching the source module is a silent no-op — the test would then assert
    # against a correction that was built normally.
    monkeypatch.setattr(levelling, 'lookup_tilt_residual', _boom)
    state, code, err = node._cycle_start_state(uc.LAUNCH)
    assert state is not None, (code, err)
    assert state.levelling_correction is node._gravity_correction
    assert node._tilt_map_degrade_logged is True


# ═════════════════════════════════════════════════════════════════════════════
# 2. The installed plan — the thing the bag measured
# ═════════════════════════════════════════════════════════════════════════════

def test_a_launch_from_the_levelled_prepare_pose_releases_gravity_level():
    """THE FIX, at the node: no tilt step at knot 0 and a level release.

    This is the bench observation, reproduced end to end through the real
    service.  MEASURED before the fix (bag ``2026-09-06_19-*``; offline twin
    ``/tmp/probe_level.py``): commanded prepare attitude -11.663 mrad (the
    correction, bit for bit), commanded release attitude mechanical zero, so the
    platform physically leaned **+11.663 mrad** at release — a **0.6682 deg**
    step the operator saw before every throw, **-9 mrad** of launch error in
    **-y** on 7/7 throws (the legacy path's bias is +8.5 mrad in +y — opposite
    sign, which is what identifies it as a frame error), and 9-38 mm of lateral
    drift into the rim.

    After: the physical aim at release is level to better than 0.03 mrad, and
    the ballistic drift ``4*h*sin(theta)`` goes 23.3 mm -> 0.000 mm at this apex.
    """
    node = _prepare(_node())
    correction = levelling.correction_from_offset(*_OFFSET)
    seed_pose, _, _ = node._current_state()
    # The seed really is the levelled prepare pose: tilted in the PLAN frame by
    # the correction, and level against gravity.
    assert np.allclose(seed_pose[3:5], [-_OFFSET[0], -_OFFSET[1]], atol=1e-9)
    assert np.allclose(_physical(seed_pose, correction), 0.0, atol=1e-9)

    resp = node._svc_plan_cycle(_launch_req(), PlanCycle.Response())
    assert resp.accepted is True, resp.message
    assert resp.code == feas.OK
    plan, meta, _t0 = node._cycle

    # Knot 0 is the machine, exactly — the install-continuity claim.
    assert np.array_equal(plan.pose[0][3:5], np.asarray(seed_pose)[3:5])
    # The release is LEVEL AGAINST GRAVITY...
    aim_mrad, axis = _aim_mrad(plan.pose[-1], correction)
    assert aim_mrad < 0.03
    # ...which means the COMMANDED attitude there is the correction itself.
    assert np.allclose(plan.pose[-1][3:5], [-_OFFSET[0], -_OFFSET[1]],
                       atol=1e-9)
    # No physical tilt step across the window at all.
    step_deg = float(np.degrees(np.linalg.norm(
        _physical(plan.pose[-1], correction)
        - _physical(plan.pose[0], correction))))
    assert step_deg < 2e-3
    # And the ballistic consequence, which is what the cup rim cares about.
    drift_mm = 4.0 * _APEX_M * 1e3 * math.sin(aim_mrad * 1e-3)
    assert drift_mm < 0.05


def test_the_same_launch_UNFRAMED_reproduces_the_bag(monkeypatch):
    """The failing half: strip the frame from the seed and the bag comes back.

    Deliberately monkeypatched at the STATE rather than at the node's offset,
    because zeroing the offset would also move the prepare pose and the two
    errors would cancel — which is exactly why the defect survived four months
    of level-machine testing.  The seed stays the levelled prepare pose; only
    the planner's knowledge of the frame is removed.
    """
    node = _prepare(_node())
    correction = levelling.correction_from_offset(*_OFFSET)
    real = TrajectoryNode._cycle_start_state

    def _unframed(self, kind):
        state, code, err = real(self, kind)
        if state is None:
            return state, code, err
        import dataclasses
        return dataclasses.replace(state, levelling_correction=None), code, err

    monkeypatch.setattr(TrajectoryNode, '_cycle_start_state', _unframed)
    resp = node._svc_plan_cycle(_launch_req(), PlanCycle.Response())
    assert resp.accepted is True, resp.message
    plan, _meta, _t0 = node._cycle

    assert np.allclose(plan.pose[-1][3:5], 0.0, atol=1e-9)   # mechanical zero
    aim_mrad, axis = _aim_mrad(plan.pose[-1], correction)
    assert aim_mrad == pytest.approx(11.663, abs=1e-3)
    assert axis[1] < 0.0, 'the bag threw in -y; so must the reproduction'
    step_deg = float(np.degrees(np.linalg.norm(
        _physical(plan.pose[-1], correction)
        - _physical(plan.pose[0], correction))))
    assert step_deg == pytest.approx(0.6682, abs=1e-3)


def test_the_chained_install_plans_both_windows_in_ONE_frame():
    """The SHIPPED install is LAUNCH+LANDING in one call — one frame, one seam.

    `release_state_from_meta` carries the frame off `meta_a`, so the chained
    window is planned in the frame the launch was BUILT in rather than in
    whatever the node holds a solve later.  Without that the two halves would
    disagree by the whole correction at their shared knot and
    `unified_cycle._joined_correction` (or `_seam_check` behind it) would refuse
    the install outright.
    """
    node = _prepare(_node())
    correction = levelling.correction_from_offset(*_OFFSET)
    resp = node._svc_plan_cycle(_launch_req(chain=True), PlanCycle.Response())
    assert resp.accepted is True, resp.message
    plan, meta, _t0 = node._cycle
    assert meta.kind == uc.JOINED
    assert meta.levelling_correction is not None
    assert np.allclose(meta.levelling_correction, correction, atol=1e-15)
    # Rest-terminal, and that rest is level AGAINST GRAVITY.
    assert np.allclose(_physical(plan.pose[-1], correction), 0.0, atol=3e-5)
    # Every knot's commanded attitude sits within the tilt the machine can hold;
    # the ceiling is checked on the GRAVITY aim, so the plan frame may exceed it
    # by the correction — see levelling_frame.md s "Consequences at the machine".
    worst = float(np.max(np.hypot(plan.pose[:, 3], plan.pose[:, 4])))
    assert worst <= math.radians(12.0) + float(np.hypot(*_OFFSET)) + 1e-9


def test_the_frame_is_recorded_on_the_installed_meta_not_re_read():
    """C-LEVEL-1's in-flight rule: a re-level does not re-frame a live plan.

    Re-reading the node's live correction from a continuation would step the
    commanded tilt by the whole delta at the splice — on knots the emitter is
    already streaming.  The plan carries its own frame instead, so a
    `/gravity_offset` arriving mid-cycle changes the NEXT install and nothing
    else.
    """
    node = _prepare(_node())
    built = levelling.correction_from_offset(*_OFFSET)
    assert node._svc_plan_cycle(_launch_req(chain=True),
                                PlanCycle.Response()).accepted is True
    _plan, meta, _t0 = node._cycle
    assert np.allclose(meta.levelling_correction, built, atol=1e-15)

    # A new offset lands mid-cycle. The node's stored correction moves...
    node._on_gravity_offset(Float64MultiArray(data=[0.05, -0.02]))
    assert not np.allclose(node._gravity_correction, built, atol=1e-6)
    # ...and the INSTALLED plan's frame does not.
    plan_after, meta_after, _t0 = node._cycle
    assert np.allclose(meta_after.levelling_correction, built, atol=1e-15)
    # A catch-side re-plan of that installed cycle keeps the plan's own frame
    # too — `replan_tail` reads `meta.levelling_correction`, never the node.
    # `t_now` sits after the launch's release and its detach cone — a splice
    # inside those knots is refused for a reason that has nothing to do with
    # frames (`REPLAN_WINDOW`), so the fixture lands past them.
    t_now, lead = 0.70, 0.10
    spliced, meta2 = uc.replan_tail(
        plan_after, meta_after, t_now,
        np.asarray(meta_after.catches[0].site_mm, dtype=float)
        + np.array([12.0, 6.0, 0.0]),
        np.asarray(meta_after.catches[0].vel_mm_s, dtype=float),
        node._limits, node._geom, lead_s=lead)
    assert meta2.levelling_correction is meta_after.levelling_correction
    k_s = uc.splice_knot(meta_after, t_now, lead)
    assert np.array_equal(spliced.pose[:k_s], plan_after.pose[:k_s])


# ═════════════════════════════════════════════════════════════════════════════
# 3. The legacy path is untouched
# ═════════════════════════════════════════════════════════════════════════════

def test_the_legacy_ingest_still_corrects_after_a_cycle_has_been_planned():
    """E3 keeps behaving exactly as C-LEVEL-1 says, cycle or no cycle.

    E8 adds a surface; it must not move one.  The `go_to_pose` ingest still
    corrects its own request's rotation once, and the plan it installs still
    ends at ``R_gravity @ R_request`` — the same assertion
    `test_levelling_frame.py::test_E3_go_to_pose_identity_lands_corrected_in_the_plan`
    makes, re-made here on a node that has just flown a unified cycle.
    """
    node = _prepare(_node())
    assert node._svc_plan_cycle(_launch_req(),
                                PlanCycle.Response()).accepted is True

    node._plan_t0 = time.perf_counter() - 10.0
    node._on_robot_state(_robot_state())
    req = GoToPose.Request()
    req.pose = Pose(position=Point(x=0.0, y=0.0, z=175.0),
                    orientation=Quaternion())
    req.duration_s = 2.0
    req.lean_gain = 0.0
    assert node._svc_go_to_pose(req, GoToPose.Response()).accepted is True
    plan = node._active_plan
    end = np.asarray(plan.state_at(plan.total_duration)[0])[3:6]
    expected = levelling.apply_gravity_correction(
        np.zeros(3), levelling.correction_from_offset(*_OFFSET))
    assert np.allclose(end, expected, atol=1e-12)


def test_a_pure_planner_call_without_a_node_is_unchanged():
    """`sim/` and `tests/motion` pass no correction and must be bit-identical.

    The sim gates (`sim/unified_gate.py`, `sim/cycle_gate.py`) have no
    gravity-offset concept at all — `levelling_frame.md` s Scope says so — so the
    default has to be a SHORT CIRCUIT, not an identity matrix threaded through a
    per-knot rotation round trip.  Asserted here as well as in
    `tests/motion/test_unified_cycle.py` because this file is the one a reader
    lands on when they ask "what did E8 change".
    """
    cfg = cr.RealizeConfig()
    pose = np.array([0.0, 0.0, cfg.active_z_mm, 0.0, 0.0, 0.0])
    state = uc.CycleState.at_rest(pose, _REST_HAND_REV, cfg)
    assert state.levelling_correction is None
    assert uc._start_tilt_for(state) is not None
    assert np.array_equal(uc._start_tilt_for(state), pose[3:5])
