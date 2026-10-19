"""skill_node in the robot frame (bb-calibration-one-attempt § Phase 3, 2026-10-19).

The robot frame is measured in software (mocap_node fits the robot's labelled
base markers and publishes the latched ``robot_frame``; ``p_world = R p_robot
+ t``). skill_node SNAPSHOTS it at the session-start frame check and carries
every world landing (tracker ``/balls``, Ball Butler announcement) world ->
robot frame, every Ball Butler aim robot -> world, and measures the Platform
offset in the robot frame. Pins:

* identity -- every output bit-for-bit what the pre-robot-frame code produced
  (the old formula restated in the test);
* rotated + translated QTM frame -- robot-frame quantities invariant;
* unknown frame -- ``REJECTED_ROBOT_FRAME_UNKNOWN`` at goal accept and from
  ``skills/check`` alongside the other ladder rows;
* freeze -- a ``robot_frame`` update mid-attempt changes nothing for it;
* staleness -- an old frame warns once and does not refuse.
"""

from __future__ import annotations

import math
import time
from unittest.mock import patch

import numpy as np
import pytest

from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import BallStateArray

import jugglebot.hardware_config as hw
from jugglebot import skill_node as sn
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import magazine_target as mt
from jugglebot.motion.skills.sites import Site
from jugglebot.robot_frame import RobotFrame
from tests.ros.conftest import _MockParameter
from tests.ros.test_skill_node import (_ball, _bb_announcement, _finish_bridge,
                                       _frame_ready, _node_with_client,
                                       _response, _robot_frame_msg,
                                       _self_toss_goal, _start_reload)


def _quat_from_matrix(R):
    return RobotFrame(R, np.zeros(3)).quaternion_xyzw()


def _rot(yaw_deg, roll_deg):
    cy, sy = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    cr, sr = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return Rz @ Rx


#: The brief's moved QTM frame: yaw 30 deg, roll 1 deg, origin (100, -50, 5).
_R_MOVED = _rot(30.0, 1.0)
_T_MOVED = np.array([100.0, -50.0, 5.0])
#: The 2026-09-22 lever-arm offset (robot frame).
_OFF = (-1.56, -8.53)


def _moved_frame():
    # Rebuild R from the quaternion exactly as the node will, so the test's
    # world images are computed through the SAME matrix the node holds.
    return RobotFrame.from_pose(_T_MOVED, _quat_from_matrix(_R_MOVED))


def _publish(node, frame: RobotFrame):
    node._on_robot_frame(_robot_frame_msg(tuple(frame.t.tolist()),
                                          frame.quaternion_xyzw()))


def _node_in(frame: RobotFrame, *, plat_robot_xy=_OFF, **kw):
    """A node whose robot frame is ``frame`` and whose Platform body sits at
    robot-frame ``(cmd + plat_robot_xy, stow height)``, fed as its WORLD
    image; the frame check is run so the snapshot and the offset are adopted."""
    node, client = _node_with_client(frame=False, robot_frame=False, **kw)
    _publish(node, frame)
    p_world = frame.to_world((plat_robot_xy[0], plat_robot_xy[1],
                              float(hw.GEOM_INITIAL_HEIGHT_MM)))
    _frame_ready(node, plat_x=float(p_world[0]), plat_y=float(p_world[1]),
                 plat_z_world=float(p_world[2]))
    assert node._frame_check_error() == ''
    return node, client


# ── identity: bit for bit ────────────────────────────────────────────────────

def test_identity_landing_and_announcement_are_bit_identical_to_the_old_formula():
    node, _c = _node_in(RobotFrame.IDENTITY)
    assert node._attempt_robot_frame.is_identity()
    assert node._mocap_to_schedule_mm == (-1.56, -8.53)
    dx, dy = node._mocap_to_schedule_mm
    x, y, z, vx, vy, vz = -51.37, 3.21, 830.4, 11.0, -7.0, -2500.0
    node._on_balls(BallStateArray(balls=[
        _ball(7, status=1, x=x, y=y, z=z, vx=vx, vy=vy, vz=vz, sec=10)]))
    landing = node._balls[7]
    old_pos = np.array([x - dx, y - dy, z], dtype=float)
    old_vel = np.array([vx, vy, vz], dtype=float)
    assert np.array_equal(landing.pos_mm, old_pos)
    assert np.array_equal(landing.vel_mm_s, old_vel)
    pos, vel = node._world_to_schedule((x, y, z), (vx, vy, vz))
    assert np.array_equal(pos, old_pos) and np.array_equal(vel, old_vel)


def test_identity_aim_point_and_frame_check_are_bit_identical():
    site = Site('P', np.array([-62.5, 0.0, sn.CATCH_CUP_Z_MM]))
    off, bias = (5.0, -3.0), (2.5, 1.25)
    old = (-62.5 + 5.0 - 2.5, 0.0 - 3.0 - 1.25, float(sn.CATCH_CUP_Z_MM))
    got = sn._mocap_aim_point_mm(site, sn.CATCH_CUP_Z_MM, off,
                                 frame=RobotFrame.identity(), bias_mm=bias)
    assert got == old
    assert sn._mocap_aim_point_mm(site, sn.CATCH_CUP_Z_MM, off) == (
        -62.5 + 5.0, -3.0, float(sn.CATCH_CUP_Z_MM))
    # The frame check through the identity snapshot == the pure check on the
    # published x/y (the pre-2026-10-19 call).
    node, _c = _node_in(RobotFrame.IDENTITY)
    via_node = node._frame_check()
    pure = sn._frame_offset_check(
        [(t, x, y) for t, x, y, _z in node._platform_mocap_xyz],
        tuple(node._commanded_xy_hist), time.perf_counter())
    assert (via_node.dx_mm, via_node.dy_mm) == (pure.dx_mm, pure.dy_mm)


def test_identity_magazine_plan_point_is_bit_identical(tmp_path):
    node, _c = _node_in(RobotFrame.IDENTITY)
    off = node._mocap_to_schedule_mm
    R = _rot(17.0, 0.0)
    origin = np.array([2000.0, -500.0, 100.0])
    p_base = [550.0, 300.0, 750.0]
    old = mt.base_to_plan_mm(p_base, R, origin, off)
    p_world = mt.base_to_world_mm(p_base, R, origin)
    new = (np.asarray(node._snapshot_frame().to_robot(p_world), dtype=float)
           - np.array([off[0], off[1], 0.0]))
    assert np.array_equal(new, old)


# ── a rotated + translated QTM frame: robot-frame quantities invariant ─────

def test_moved_frame_landing_comes_back_in_the_robot_frame():
    frame = _moved_frame()
    node, _c = _node_in(frame)
    # The Platform offset measured through the moved frame equals the one
    # measured under identity.
    assert node._mocap_to_schedule_mm == pytest.approx(_OFF, abs=1e-9)
    # A landing whose ROBOT-frame value (schedule frame + offset) is
    # (x, y, 830), given as its world image.
    want = np.array([-50.0, 12.0, 830.0])
    p_robot = want + np.array([_OFF[0], _OFF[1], 0.0])
    v_robot = np.array([120.0, -40.0, -2500.0])
    p_world = frame.to_world(p_robot)
    v_world = frame.vel_to_world(v_robot)
    node._on_balls(BallStateArray(balls=[
        _ball(3, status=1, x=p_world[0], y=p_world[1], z=p_world[2],
              vx=v_world[0], vy=v_world[1], vz=v_world[2], sec=10)]))
    np.testing.assert_allclose(node._balls[3].pos_mm, want, atol=1e-6)
    # Velocities rotate only -- no translation.
    np.testing.assert_allclose(node._balls[3].vel_mm_s, v_robot, atol=1e-6)
    assert not np.allclose(v_world, v_robot)


def test_moved_frame_announcement_landing_is_the_robot_frame_prior(tmp_path):
    frame = _moved_frame()
    node, _c = _node_with_client(response=_response(), frame=False,
                                 robot_frame=False)
    _publish(node, frame)
    p_world = frame.to_world((0.0, 0.0, float(hw.GEOM_INITIAL_HEIGHT_MM)))
    _frame_ready(node, plat_x=float(p_world[0]), plat_y=float(p_world[1]),
                 plat_z_world=float(p_world[2]))
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    # The BB aim is the WORLD image of (site + offset) -- offset ~0 here.
    req = node._bb_throw_cli.calls[0]
    aim_world = np.array([req.target_point_global_mm.x,
                          req.target_point_global_mm.y,
                          req.target_point_global_mm.z])
    np.testing.assert_allclose(frame.to_robot(aim_world),
                               [-50.0, 0.0, sn.CATCH_CUP_Z_MM], atol=1e-6)
    _finish_bridge(node)
    land_w = frame.to_world((-50.0, 0.0, sn.CATCH_CUP_Z_MM))
    vel_w = frame.vel_to_world((1200.0, 0.0, -2500.0))
    node._on_announcement(_bb_announcement(
        target_id=node._robot_name, landing_mm=land_w, landing_vel_mm_s=vel_w,
        throw_time_s=2.0, landing_time_s=2.7))
    catch = node._executor.schedule.skills[1]
    np.testing.assert_allclose(catch.landing_prior.pos_mm,
                               [-50.0, 0.0, sn.CATCH_CUP_Z_MM], atol=1e-6)


def test_moved_frame_bb_aim_is_the_world_image_of_site_plus_offset_minus_bias():
    frame = _moved_frame()
    site = Site('P', np.array([-125.0, 0.0, sn.CATCH_CUP_Z_MM]))
    got = np.array(sn._mocap_aim_point_mm(site, sn.CATCH_CUP_Z_MM, _OFF,
                                          frame=frame, bias_mm=(4.0, -2.0)))
    want_robot = np.array([-125.0 + _OFF[0] - 4.0, _OFF[1] + 2.0,
                           sn.CATCH_CUP_Z_MM])
    np.testing.assert_allclose(got, frame.to_world(want_robot), atol=1e-9)


# ── unknown frame refuses ────────────────────────────────────────────────────

def test_no_robot_frame_refuses_the_goal_at_any_authority(tmp_path):
    for authority in (0.0, 40.0):
        node, _c = _node_with_client(robot_frame=False)
        node.set_parameters([_MockParameter(
            authority, name='learner_lateral_authority_mm')])
        with patch.object(sn, '_REPO_ROOT', str(tmp_path)):
            resp = node._start_pattern(_self_toss_goal())
        assert resp.success is False
        assert ex.REJECTED_ROBOT_FRAME_UNKNOWN in resp.message, resp.message
        assert node._attempt_robot_frame is None
        assert node._executor is None


def test_no_robot_frame_is_reported_by_skills_check_with_the_ladder():
    node, _c = _node_with_client(robot_frame=False)
    node._mocap_mono = 0.0                         # mocap stale too
    resp = node._svc_check(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    ladder = [ln for ln in resp.message.split('\n')
              if ln.startswith('ladder REFUSED')]
    assert len(ladder) == 1, resp.message
    assert ex.REJECTED_MOCAP_STALE in ladder[0]
    assert ex.REJECTED_ROBOT_FRAME_UNKNOWN in ladder[0]
    assert node._observations(0.0).robot_frame_known is False
    _publish(node, RobotFrame.IDENTITY)
    assert node._observations(0.0).robot_frame_known is True


def _status(value=None):
    from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
    m = DiagnosticStatus()
    if value is not None:
        m.values = [KeyValue(key='robot_frame_settled', value=value)]
    return m


def test_settling_status_refuses_with_the_other_rows_and_clears(tmp_path):
    node, _c = _node_with_client()
    node._on_mocap_status(_status('0'))
    assert node._observations(0.0).robot_frame_settled is False
    resp = node._svc_check(Trigger.Request(), Trigger.Response())
    ladder = [ln for ln in resp.message.split('\n')
              if ln.startswith('ladder REFUSED')]
    assert len(ladder) == 1 and ex.REJECTED_ROBOT_FRAME_SETTLING in ladder[0]
    with patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        goal = node._start_pattern(_self_toss_goal())
    assert goal.success is False
    assert ex.REJECTED_ROBOT_FRAME_SETTLING in goal.message, goal.message
    assert 'Robot frame is settling' in goal.message
    node._on_mocap_status(_status('1'))              # flips back: cleared
    assert node._observations(0.0).robot_frame_settled is True
    resp = node._svc_check(Trigger.Request(), Trigger.Response())
    assert ex.REJECTED_ROBOT_FRAME_SETTLING not in resp.message


def test_settled_status_and_no_status_do_not_refuse():
    node, _c = _node_with_client()
    assert node._observations(0.0).robot_frame_settled is True   # no status
    node._on_mocap_status(_status('1'))
    assert node._observations(0.0).robot_frame_settled is True
    node._on_mocap_status(_status())                 # key absent: unchanged
    assert node._observations(0.0).robot_frame_settled is True


def test_absent_key_after_unsettled_keeps_unsettled():
    node, _c = _node_with_client()
    node._on_mocap_status(_status('0'))
    assert node._robot_frame_settled is False
    node._on_mocap_status(_status())                 # key absent: unchanged
    assert node._robot_frame_settled is False


def test_mid_attempt_unsettle_is_frozen_out_of_observations_and_snapshot():
    node, _c = _node_with_client()
    node._on_mocap_status(_status('0'))
    assert node._robot_frame_settled is False
    with patch.object(node, '_attempt_running', return_value=False):
        assert node._observations(0.0).robot_frame_settled is False
    with patch.object(node, '_attempt_running', return_value=True):
        assert node._observations(0.0).robot_frame_settled is True
        assert node._snapshot_robot_frame() == ''


def test_a_malformed_robot_frame_is_dropped_and_the_previous_kept():
    node, _c = _node_with_client()
    before = node._robot_frame
    node._on_robot_frame(_robot_frame_msg(quat_xyzw=(0.0, 0.0, 0.0, 0.0)))
    assert node._robot_frame is before


# ── freeze ───────────────────────────────────────────────────────────────────

def test_a_robot_frame_update_mid_attempt_does_not_move_its_landings(tmp_path):
    frame_a = _moved_frame()
    node, _c = _node_with_client(response=_response(), frame=False,
                                 robot_frame=False)
    _publish(node, frame_a)
    p_world = frame_a.to_world((0.0, 0.0, float(hw.GEOM_INITIAL_HEIGHT_MM)))
    _frame_ready(node, plat_x=float(p_world[0]), plat_y=float(p_world[1]),
                 plat_z_world=float(p_world[2]))
    resp = _start_reload(node, tmp_path)           # an attempt is running
    assert resp.success is True, resp.message
    snap = node._attempt_robot_frame
    frame_b = RobotFrame(frame_a.R, frame_a.t + np.array([50.0, 0.0, 0.0]))
    _publish(node, frame_b)
    assert node._robot_frame is not snap
    # A dry run mid-attempt does not re-snapshot either.
    node._svc_check(Trigger.Request(), Trigger.Response())
    assert node._attempt_robot_frame is snap
    want = np.array([-50.0, 0.0, 830.0])
    pw = frame_a.to_world(want)
    node._on_balls(BallStateArray(balls=[
        _ball(4, status=1, x=pw[0], y=pw[1], z=pw[2], sec=10)]))
    np.testing.assert_allclose(node._balls[4].pos_mm,
                               want - np.array([*node._mocap_to_schedule_mm,
                                                0.0]), atol=1e-6)


# ── staleness ────────────────────────────────────────────────────────────────

def test_an_old_robot_frame_warns_once_per_attempt_and_does_not_refuse():
    node, _c = _node_with_client()
    node._robot_frame_mono = time.perf_counter() - 11.0
    warnings = []
    node.get_logger().warning = lambda m: warnings.append(m)
    assert node._frame_check_error() == ''
    stale = [w for w in warnings if 'robot frame is' in w]
    assert len(stale) == 1, warnings
    assert 'the robot has not moved unless someone moved it' in stale[0]
    assert '11 s old' in stale[0]
