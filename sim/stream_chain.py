"""The real chain: emitter -> pump -> wire -> firmware mirror (plan § 4 R2).

Extracted verbatim (2026-09-12, skill-stack R2 Unit E) from ``sim/unified_gate.py``
so ``sim/skills_gate.py`` and the ported hand-lane tests
(``tests/sim/test_skills_gate.py``) share ONE copy of the plumbing rather than a
second transcription drifting against the first::

    motion.trajectory.emitter.KnotEmitter.frame()          40 Hz
        │                     samples (tau, tau+dt, tau+2*dt) off a CyclePlan
        ▼
    teensy_link.setpoint_pump.SetpointPump.build()
        │                     the REAL bridge pump: per-channel step gates, the
        │                     all-or-nothing hand key rule, HAS_HAND / HAS_V1
        ▼
    Setpoint.pack() -> bytes -> Setpoint.unpack()
        │                     the REAL v6 wire (float32 lanes)
        ▼
    teensy_interp.TeensyLegInterp.tick() / .tick_hand()    500 Hz
                              the firmware mirror: 6 leg lanes (motor_guard's
                              validated ladder) + the FW 17 hand lane

Library module under ``sim/`` — no ``sys.path`` mutation (the import-style
contract, ``tests/sim/test_sim_import_style.py``); the firmware mirror is
loaded by FILE PATH exactly as ``unified_gate`` did, for the same reason: it
lives with the xref that validates it against ``motor_guard``
(``tools/probes/.../hermite_xref``), and a copy under ``sim/`` would be a
second thing to keep in step with ``leg_interp.cpp``.
"""

from __future__ import annotations

import importlib.util
import math
import os
import sys

import numpy as np

import jugglebot.hardware_config as hw
from jugglebot.motion.ik_solver import pose_to_leg_lengths, rotvec_to_rot_matrix
from jugglebot.motion.trajectory import KnotEmitter

from teensy_link.protocol import Setpoint
from teensy_link.setpoint_pump import (
    FLAG_HAS_HAND, FLAG_HAS_U1, FLAG_HAS_U2, FLAG_HAS_V1, SetpointPump,
)

#: Firmware interpolation tick (s) -- 500 Hz, ``leg_interp.cpp``'s ISR rate.
TICK_S = 0.002

_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_XREF_DIR = os.path.join(_repo_root, 'tools', 'probes',
                         'teensy_link_profiling', 'hermite_xref')


def _load_teensy_interp():
    if 'teensy_interp' in sys.modules:
        return sys.modules['teensy_interp']
    spec = importlib.util.spec_from_file_location(
        'teensy_interp', os.path.join(_XREF_DIR, 'teensy_interp.py'))
    module = importlib.util.module_from_spec(spec)
    sys.modules['teensy_interp'] = module
    spec.loader.exec_module(module)
    return module


ti = _load_teensy_interp()


def make_pump() -> SetpointPump:
    """A ``SetpointPump`` built from the SAME config chain the bridge uses.

    ``teensy_bridge_node`` passes ``max_step_hand_rev`` as
    ``JB_TRAJ_HAND_VEL_LIMIT_RPS x JB_TRAJ_KNOT_DT_S`` rather than leaving it on
    the module default; re-deriving it the same way here is what makes a pump
    reject in a caller of this module mean the same thing it would mean on the
    bridge.
    """
    return SetpointPump(
        mm_to_rev=hw.GEOM_MM_TO_REV,
        max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV,
        max_step_hand_rev=(float(hw.JB_TRAJ_HAND_VEL_LIMIT_RPS)
                           * float(hw.JB_TRAJ_KNOT_DT_S)),
        torque_ff_enabled=bool(hw.DYNAMICS_TORQUE_FF_ENABLED),
        torque_ff_max_nm=float(hw.DYNAMICS_TORQUE_FF_MAX_NM),
        torque_wire_scale=float(hw.ODRIVE_LEG_TORQUE_WIRE_SCALE),
        torque_ff_ramp_frames=int(math.ceil(
            float(hw.DYNAMICS_TORQUE_FF_RAMP_S)
            / float(hw.JB_TRAJ_KNOT_DT_S))))


class _DummyIPC:
    """No-op IPC so ``MotorGuard`` needs no ZMQ broker (``xref._DummyIPC``)."""

    def recv_all(self):
        return []

    def send_telemetry(self, msg):
        pass

    @property
    def seconds_since_last_recv(self):
        return 0.0

    def close(self):
        pass


def make_mirror(geom) -> 'ti.TeensyLegInterp':
    """The firmware mirror, with the stroke bounds the firmware actually ships.

    ``MotorGuard``'s ``_stroke_{min,max}_rev`` are the same numbers as
    ``canbridge_config.h``'s ``STROKE_{MIN,MAX}_REV`` -- pinned by
    ``tests/firmware/test_hermite_xref.py::test_firmware_stroke_bounds_match_motor_guard``
    -- so taking them from the guard keeps ONE source rather than a third copy.
    """
    import jugglebot.motion.motor_guard as mg              # noqa: PLC0415
    guard = mg.MotorGuard(geom=geom, ipc=_DummyIPC())
    return ti.TeensyLegInterp(guard._stroke_min_rev, guard._stroke_max_rev)


def latch(mirror, sp: Setpoint, t_latch: float) -> None:
    """Latch one DECODED wire frame into the mirror, flags and all."""
    has_u1 = bool(sp.flags & FLAG_HAS_U1)
    has_u2 = bool(sp.flags & FLAG_HAS_U2)
    has_v1 = bool(sp.flags & FLAG_HAS_V1)
    mirror.latch_setpoint(
        sp.u0[:6], sp.v0[:6], sp.accel[:6], sp.torque_ff[:6], t_latch,
        u1=(sp.u1[:6] if has_u1 else None),
        u2=(sp.u2[:6] if has_u2 else None),
        v1=(sp.v1[:6] if has_v1 else None))
    if sp.flags & FLAG_HAS_HAND:
        mirror.latch_hand(
            sp.u0[6], sp.v0[6], t_latch,
            u1=(sp.u1[6] if has_u1 else None),
            u2=(sp.u2[6] if has_u2 else None),
            v1=(sp.v1[6] if has_v1 else None),
            accel=sp.accel[6])


def slider_mm_of_rev(rev: float, cfg) -> float:
    """Hand motor rev -> sim slider mm (``cup_realize``'s relation, inverted)."""
    return (float(rev) / hw.HAND_REV_PER_M * 1000.0
            + float(cfg.slider_rev_zero_mm))


def rev_of_slider_mm(mm: float, cfg) -> float:
    return (float(mm) - float(cfg.slider_rev_zero_mm)) / 1000.0 \
        * hw.HAND_REV_PER_M


def recon(plan, mirror, tau: float, geom, mm_to_rev):
    """``(|Δleg|, |Δhand|)`` in rev between the mirror's RAW ladder and the plan."""
    pose_p, _, _ = plan.state_at(tau)
    rot = rotvec_to_rot_matrix(np.asarray(pose_p[3:6], dtype=float))
    ext = pose_to_leg_lengths(np.asarray(pose_p[:3], dtype=float), rot, geom)
    d_leg = float(np.max(np.abs(np.asarray(mirror.raw_pos)
                                - ext * np.asarray(mm_to_rev, dtype=float))))
    hr, _ = plan.hand_at(tau)
    return d_leg, abs(float(mirror.hand_raw_pos) - float(hr))


def hand_decay_probe(plan, geom) -> dict:
    """Cut the stream mid-stroke and watch the hand lane wind down.

    **Why this is a separate probe and not a column of the main run.**  Every
    plan shipped by a skill segment ENDS at rest, so the falling edge at plan
    expiry is a falling edge on a hand that is already stationary -- it
    observes nothing.  The firmware rule is about the other case: HAS_HAND
    falling while the hand is MOVING (a host gap, a backstop freeze to a
    hand-less plan), where holding Mode 1's ``s = 1`` endpoint would keep
    commanding it with ``vel_ff = v1``, i.e. hold-at-last-command from up to
    200 rev/s.  So the probe cuts the stream at the knot of peak hand speed and
    ticks the mirror forward with no further frames.

    Returns the observed ladder: the velocity at the moment of the cut, the age
    at which the lane leaves Mode 1, the age at which the velocity first reaches
    EXACTLY zero, whether it is monotone non-increasing in magnitude after the
    Taylor phase, and the total travel the wind-down adds.
    """
    emitter = KnotEmitter(geom)
    pump = make_pump()
    mirror = make_mirror(geom)
    dt = float(plan.dt)
    k_cut = int(np.argmax(np.abs(np.asarray(plan.hand_vel_rps))))
    k_cut = max(1, min(k_cut, int(plan.n_knots) - 3))

    hand_fb = float(plan.hand_rev[0])
    for seq in range(k_cut + 1):
        tau = seq * dt
        sp, _ = pump.build(emitter.frame(plan, tau, seq),
                           t_origin_us=int(seq * 25000))
        if sp is None:
            return {'ok': False, 'reason': 'pump rejected the pre-cut stream'}
        latch(mirror, Setpoint.unpack(sp.pack()), tau)
        hand_fb = float(sp.u0[6])
    t_cut = k_cut * dt
    v_at_cut = float('nan')
    pos_at_cut = None
    cmd_at_cut = None
    t_leave_mode1 = None
    t_zero = None
    peak_abs_after = 0.0
    monotone = True
    prev_abs = None
    trace = []
    cmd_last = 0.0
    lead_ticks_at_cut = int(mirror.hand_lead_clamp_ticks)
    t = t_cut
    while t <= t_cut + 0.40 + 1e-12:
        mirror.tick(t, [0.0] * 6)
        out = mirror.tick_hand(t, hand_fb)
        if out is None:
            return {'ok': False,
                    'reason': 'the hand lane went silent mid-probe (age %.4f s)'
                              % (t - mirror.hand_ts)}
        cmd_last = float(out[0])
        age = t - mirror.hand_ts
        v = float(mirror.hand_raw_vel)
        if pos_at_cut is None:
            pos_at_cut = float(mirror.hand_raw_pos)
            cmd_at_cut = cmd_last
            v_at_cut = v
        if t_leave_mode1 is None and int(mirror.hand_mode) != 1:
            t_leave_mode1 = age
        if int(mirror.hand_mode) != 1:
            if prev_abs is not None and abs(v) > prev_abs + 1e-9:
                monotone = False
            prev_abs = abs(v)
            peak_abs_after = max(peak_abs_after, abs(v))
            if t_zero is None and v == 0.0:
                t_zero = age
        trace.append((age, v, float(mirror.hand_raw_pos)))
        t += TICK_S
    return {
        'ok': True,
        'cut_knot': k_cut,
        'v_at_cut_rps': v_at_cut,
        'age_left_mode1_s': t_leave_mode1,
        'age_velocity_zero_s': t_zero,
        'decay_deadline_s': (ti.SEGMENT_T_S + ti.MAX_EXTRAP_DT_S
                             + ti.EXTRAP_DECAY_DT_S),
        'sampling_slack_s': TICK_S,
        'monotone_after_mode1': bool(monotone),
        'travel_after_cut_rev': float(trace[-1][2] - pos_at_cut),
        'clamped_travel_after_cut_rev': float(cmd_last - cmd_at_cut),
        'max_lead_hand_rev': ti.MAX_LEAD_HAND_REV,
        'lead_clamp_ticks': int(mirror.hand_lead_clamp_ticks) - lead_ticks_at_cut,
        'final_vel_rps': float(trace[-1][1]),
        'peak_abs_vel_after_mode1_rps': peak_abs_after,
    }
