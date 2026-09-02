"""KnotEmitter tests (Phase 1) — the load-bearing pump-acceptance invariant.

The cross-cutting invariant of the whole MVP command path: **every frame the
emitter can produce is accepted by a real ``SetpointPump``** (the
production-in-the-loop rule). If this holds, the entire validated :5557 →
``SetpointPump`` → Teensy-Hermite chain transfers to hardware for free. Also
covers the FK seed roundtrip (activate revs → pose ≈ (0,0,170)), the ``motor_rev``
= ext × mm_to_rev convention, and byte-compatibility of the emitted frame via
the shared ipc ``_pack``.
"""

from __future__ import annotations

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import leg_lengths_to_pose, rot_matrix_to_rotvec
from jugglebot.motion import ipc
from jugglebot.motion.trajectory import (
    HoldPlan, KnotEmitter, TrajectoryLimits,
)
from jugglebot.motion.trajectory import planner

from teensy_link.setpoint_pump import SetpointPump

NEUTRAL = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])


def _geom():
    return StewartGeometry()


def _pump():
    return SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV,
                        max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV)


# ── FK seed roundtrip ─────────────────────────────────────────

def test_activate_revs_fk_roundtrip_is_neutral():
    geom = _geom()
    mm = np.asarray(hw.GEOM_MM_TO_REV)
    ext = np.asarray(hw.JB_OP_ACTIVATE_POSITION_REVS) / mm
    pos, rot, _ = leg_lengths_to_pose(ext, geom)
    rotvec = rot_matrix_to_rotvec(rot)
    assert np.allclose(pos, [0.0, 0.0, 170.0], atol=1e-3)
    assert np.allclose(rotvec, 0.0, atol=1e-6)


# ── motor_rev convention ──────────────────────────────────────

def test_emitter_motor_rev_equals_ext_times_mm_to_rev():
    geom = _geom()
    emit = KnotEmitter(geom)
    frame = emit.frame(HoldPlan(NEUTRAL), 0.0, 0)
    ext = np.asarray(frame['ext_mm'])
    mr = np.asarray(frame['motor_rev'])
    assert np.allclose(mr, ext * np.asarray(hw.GEOM_MM_TO_REV))
    # Hold at neutral ⇒ u0 == the activate revs (bumpless arm at the active pose).
    assert np.allclose(mr, hw.JB_OP_ACTIVATE_POSITION_REVS, atol=1e-3)


# ── The load-bearing invariant: pump accepts every emitted frame ──

def test_every_hold_frame_pump_accepted():
    geom = _geom()
    emit = KnotEmitter(geom)
    pump = _pump()
    plan = HoldPlan(NEUTRAL)
    for i in range(400):        # 10 s at 40 Hz
        sp, reason = pump.build(emit.frame(plan, i * 0.025, i), t_origin_us=i * 25000)
        assert reason is None, f"frame {i} rejected: {reason}"
        assert sp is not None
    assert pump.frames_built == 400
    assert pump.frames_rejected == 0


def test_every_move_frame_pump_accepted():
    """A profiled go-home move: every knot pump-accepted (no step-gate trip)."""
    geom = _geom()
    limits = TrajectoryLimits.from_config(hw)
    emit = KnotEmitter(geom)
    pump = _pump()
    state0 = (NEUTRAL + np.array([12., -8., 6., 0.0, 0.0, 0.0]),
              np.zeros(6), np.zeros(6))
    plan = planner.build_return_to_neutral(state0, NEUTRAL, 2.0, limits, geom)
    n = int(plan.total_duration / 0.025) + 60   # run past into the terminal hold
    for i in range(n):
        sp, reason = pump.build(emit.frame(plan, i * 0.025, i), t_origin_us=i * 25000)
        assert reason is None, f"move frame {i} rejected: {reason}"
    assert pump.frames_rejected == 0


def test_hold_frame_u0_within_arming_tolerance_of_seed():
    """The seed-then-hold u0 sits inside the 0.25 rev arming pre-check band."""
    geom = _geom()
    emit = KnotEmitter(geom)
    # Seed a hold from a measured pose (via FK of some plausible extensions).
    mm = np.asarray(hw.GEOM_MM_TO_REV)
    measured_rev = np.asarray(hw.JB_OP_ACTIVATE_POSITION_REVS)
    ext = measured_rev / mm
    pos, rot, _ = leg_lengths_to_pose(ext, geom)
    pose = np.concatenate([pos, rot_matrix_to_rotvec(rot)])
    frame = emit.frame(HoldPlan(pose), 0.0, 0)
    u0 = np.asarray(frame['motor_rev'])
    assert np.all(np.abs(u0 - measured_rev) < 0.25)


# ── Byte-compatibility via the shared ipc _pack ───────────────

def test_emitted_frame_packs_and_roundtrips():
    import msgpack
    geom = _geom()
    emit = KnotEmitter(geom)
    frame = emit.frame(HoldPlan(NEUTRAL), 0.0, 7)
    topic, blob = ipc._pack(ipc.TOPIC_MPC_CMD, frame)
    assert topic == ipc.TOPIC_MPC_CMD
    decoded = msgpack.unpackb(blob, raw=False)
    assert decoded['type'] == 'mpc_cmd'
    assert decoded['seq'] == 7
    # ndarrays become plain lists on the wire (the _ndarray_default handler).
    assert isinstance(decoded['motor_rev'], list) and len(decoded['motor_rev']) == 6
    assert isinstance(decoded['vel_mm_s'], list)
    # A real pump accepts the round-tripped (wire-form) frame too.
    sp, reason = _pump().build(decoded, t_origin_us=0)
    assert reason is None and sp is not None


# ═════════════════════════════════════════════════════════════
# T-U8 / T-R1 — legacy byte-identity vs the CAPTURED v5 fixtures, and the
# CyclePlan 7th-channel emission (2026-09-01, unified-7dof-planner Phase 2)
# ═════════════════════════════════════════════════════════════

import json as _json
import os as _os

_V5_EMITTER_FIXTURE = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)),
                                    'data', 'v5_emitter_frame_fixtures.json')


def _emitter_fixture():
    with open(_V5_EMITTER_FIXTURE) as f:
        return _json.load(f)


def _fixture_plan(case_name: str):
    """Recreate the exact plan a fixture case's recipe names."""
    if case_name == 'hold_neutral':
        return HoldPlan(np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0]))
    if case_name == 'move_return_to_neutral':
        state0 = (np.array([12.0, -8.0, 176.0, 0.0, 0.0, 0.0]),
                  np.zeros(6), np.zeros(6))
        limits = TrajectoryLimits.from_config(hw)
        return planner.build_return_to_neutral(
            state0, np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0]), 2.0,
            limits, StewartGeometry())
    raise AssertionError(f'unknown fixture case {case_name!r}')


def test_legacy_frames_byte_identical_to_v5_capture():
    """A plan with no hand track must emit BYTE-IDENTICAL msgpack to the
    pre-Phase-2 emitter — same keys, same key order, same float bits.

    The fixture (tests/motion/data/v5_emitter_frame_fixtures.json) was captured
    at commit 2aaaae1 (2026-09-01) by tools/probes/capture_v5_wire_fixtures.py
    against THIS checkout's generated config (gravity FF shipped ON) — it is
    READ-ONLY ground truth; if config/geometry changes, the capture script's
    provenance notes govern, not a regeneration.
    """
    fx = _emitter_fixture()
    assert fx['captured_at_commit'].startswith('2aaaae1')
    for case in fx['cases']:
        plan = _fixture_plan(case['case'])
        emit = KnotEmitter(_geom())
        for fr in case['frames']:
            frame = emit.frame(plan, float(fr['tau']), int(fr['seq']))
            topic, blob = ipc._pack(ipc.TOPIC_MPC_CMD, frame)
            assert (topic.decode() if isinstance(topic, bytes)
                    else topic) == fr['topic']
            assert blob.hex() == fr['msgpack_hex'], (
                f"{case['case']} tau={fr['tau']}: emitted bytes differ from "
                "the v5 capture")


def test_legacy_plan_emits_none_of_the_v6_keys():
    frame = KnotEmitter(_geom()).frame(HoldPlan(NEUTRAL), 0.0, 0)
    for key in ('vel_next_mm_s', 'hand_rev', 'hand_vel_rps', 'hand_next_rev',
                'hand_next2_rev', 'hand_next_vel_rps'):
        assert key not in frame


# ── CyclePlan hand emission (a REAL CyclePlan, the test_cycle_plan recipe) ──

from jugglebot.motion.ik_solver import (
    rotvec_to_rot_matrix, compute_jacobian, twist_to_leg_velocities,
)
from jugglebot.motion.trajectory.cycle_plan import CyclePlan

_CYCLE_DT = 0.025


def _cubic_channel(t, coeffs):
    a, b, c, d = coeffs
    return (a + b * t + c * t ** 2 + d * t ** 3,
            b + 2.0 * c * t + 3.0 * d * t ** 2)


def _cycle_plan(n: int = 17) -> CyclePlan:
    """Cubic-per-channel cycle plan near the neutral pose (test_cycle_plan's
    fixture recipe, kept small enough that every knot is pump-acceptable)."""
    t = np.arange(n, dtype=float) * _CYCLE_DT
    pose_coeffs = [
        (0.0, 40.0, -30.0, 8.0), (0.0, -25.0, 18.0, -5.0),
        (170.0, 6.0, -4.0, 1.0), (0.0, 0.05, -0.04, 0.01),
        (0.0, -0.03, 0.02, -0.006), (0.0, 0.0, 0.0, 0.0),
    ]
    pose = np.empty((n, 6))
    pose_vel = np.empty((n, 6))
    for i, c in enumerate(pose_coeffs):
        pose[:, i], pose_vel[:, i] = _cubic_channel(t, c)
    hand_rev, hand_vel = _cubic_channel(t, (5.0, 3.0, -2.0, 0.5))
    return CyclePlan(pose, pose_vel, hand_rev, hand_vel, _CYCLE_DT, catch_k=7)


def test_cycle_plan_emits_hand_keys_with_leg_lookahead_discipline():
    """hand_at sampled at (τ, τ+dt, τ+2dt) exactly like the legs' state_at, and
    vel_next_mm_s = J(pose@τ+dt)·twist@τ+dt — the exact u1-knot leg rates."""
    plan = _cycle_plan()
    geom = _geom()
    emit = KnotEmitter(geom)
    tau = 0.1
    frame = emit.frame(plan, tau, 4)
    h0 = plan.hand_at(tau)
    h1 = plan.hand_at(tau + _CYCLE_DT)
    h2 = plan.hand_at(tau + 2 * _CYCLE_DT)
    assert frame['hand_rev'] == pytest.approx(h0[0])
    assert frame['hand_vel_rps'] == pytest.approx(h0[1])
    assert frame['hand_next_rev'] == pytest.approx(h1[0])
    assert frame['hand_next_vel_rps'] == pytest.approx(h1[1])
    assert frame['hand_next2_rev'] == pytest.approx(h2[0])
    # vel_next: independent recomputation from the plan state at τ+dt.
    pose1, twist1, _ = plan.state_at(tau + _CYCLE_DT)
    pos1 = np.asarray(pose1[:3], dtype=float)
    rot1 = rotvec_to_rot_matrix(np.asarray(pose1[3:6], dtype=float))
    expect = twist_to_leg_velocities(twist1, pos1, rot1, geom,
                                     J=compute_jacobian(pos1, rot1, geom))
    assert np.allclose(np.asarray(frame['vel_next_mm_s']), expect, atol=1e-9)


def test_every_cycle_plan_frame_pump_accepted_with_hand_flags():
    """The load-bearing invariant, extended to the 7th channel: every frame a
    hand-carrying plan produces is accepted by a REAL pump and carries
    HAS_HAND|HAS_V1 with the hand values at index 6."""
    from teensy_link.setpoint_pump import FLAG_HAS_HAND, FLAG_HAS_V1
    plan = _cycle_plan()
    emit = KnotEmitter(_geom())
    pump = _pump()
    n = int(plan.total_duration / _CYCLE_DT) + 1
    for i in range(n):
        tau = i * _CYCLE_DT
        frame = emit.frame(plan, tau, i)
        sp, reason = pump.build(frame, t_origin_us=i * 25000)
        assert reason is None, f"cycle frame {i} rejected: {reason}"
        assert sp.flags & FLAG_HAS_HAND
        assert sp.flags & FLAG_HAS_V1
        assert sp.u0[6] == pytest.approx(plan.hand_at(tau)[0])
    assert pump.frames_rejected == 0


def test_out_reuse_scrubs_stale_hand_keys():
    """make_mpc_command's absent-when-None rule under out= reuse: a hand frame
    followed by a legacy frame into the SAME dict must leave no hand key
    behind, and the reused-dict legacy bytes must equal a fresh-dict build."""
    plan = _cycle_plan()
    hold = HoldPlan(NEUTRAL)
    emit = KnotEmitter(_geom())
    d: dict = {}
    frame = emit.frame(plan, 0.1, 0, out=d)
    assert frame is d and 'hand_rev' in d and 'vel_next_mm_s' in d
    frame = emit.frame(hold, 0.0, 1, out=d)
    for key in ('vel_next_mm_s', 'hand_rev', 'hand_vel_rps', 'hand_next_rev',
                'hand_next2_rev', 'hand_next_vel_rps'):
        assert key not in frame
    _, blob_reused = ipc._pack(ipc.TOPIC_MPC_CMD, frame)
    _, blob_fresh = ipc._pack(ipc.TOPIC_MPC_CMD,
                              KnotEmitter(_geom()).frame(hold, 0.0, 1))
    assert blob_reused == blob_fresh
