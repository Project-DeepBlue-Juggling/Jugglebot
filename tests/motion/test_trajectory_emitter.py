"""KnotEmitter tests (Phase 1) — the load-bearing pump-acceptance invariant.

The cross-cutting invariant of the whole MVP command path: **every frame the
emitter can produce is accepted by a real ``SetpointPump``** (the
production-in-the-loop rule). If this holds, the entire validated :5557 →
``SetpointPump`` → Teensy-Hermite chain transfers to hardware for free. Also
covers the FK seed roundtrip (activate revs → pose ≈ (0,0,170)), the ``motor_rev``
= ext × mm_to_rev convention (byte-identical to ``HardwarePlant``), and
byte-compatibility of the emitted frame via the shared ipc ``_pack``.
"""

from __future__ import annotations

import numpy as np

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
