"""KnotEmitter — samples a plan and assembles one ``make_mpc_command`` frame.

Each 40 Hz tick, the emitter samples the active plan at ``(tau, tau+dt, tau+2·dt)``
(``dt`` = 25 ms, the fixed firmware ``SEGMENT_T_S``), runs the IK chain on each
sample, and assembles the exact ``make_mpc_command`` field set ``HardwarePlant``
sent before it was removed 2026-09-01 (tag mpc-final) — so the whole validated
:5557 → ``SetpointPump`` → Teensy-Hermite chain accepts our frames unchanged
(the "emitter envelope is byte-compatible with ``HardwarePlant`` by
construction" invariant, which outlives the removal: the pump still enforces
that field set).

Field mapping (as ``controller/hardware_plant.py::command`` sent it, preserved
at tag mpc-final):
  * ``ext_mm``      = IK(pose@tau)                         — STOW-relative leg ext (mm)
  * ``pose_6dof``   = pose@tau                             — [x,y,z,rx,ry,rz] (mm, rad)
  * ``motor_rev``   = ext_mm × GEOM_MM_TO_REV              — **no stow offset** (0 ext = 0 rev),
    byte-for-byte the convention ``HardwarePlant`` used (hardware_plant.py:413
    at that tag). ``SetpointPump`` takes this verbatim as the Teensy-side
    ``u0`` knot.
  * ``vel_mm_s``    = J(pose) · twist@tau                  — leg extension rates (mm/s); the
    pump SAFETY-rejects a frame without a finite velocity, so this is always sent.
  * ``acc_mm_s2``   = J·accel + J̇·twist                    — leg extension accels (mm/s²);
    the Teensy Mode-1 Hermite ignores accel, but it is carried for HardwarePlant
    field-set parity (and used by the diagnostics peak tracker in later phases).
  * ``cmd_next_mm`` = IK(pose@tau+dt)                      — the u1 lookahead knot.
  * ``cmd_next2_mm``= IK(pose@tau+2·dt)                    — the u2 knot (sets the Hermite
    endpoint velocity v1 = (u2−u1)/T; C1 across segment joins).
  * ``torque_Nm``   = gravity (+ optional platform-inertia) feedforward — **TRUE Nm**,
    extension-positive, from :class:`~jugglebot.motion.torque_ff.LegTorqueFeedforward`.
    **Exact zeros unless ``dynamics.torque_ff_enabled`` is set** (shipped: true
    since 2026-07-16), in
    which case this field is byte-identical to the pre-feature ``np.zeros(6)``.
    The clamp / Kt wire scale / ramp-in are NOT applied here — they live at the single
    wire enforcement point, ``teensy_link/setpoint_pump.py``.

The 7th channel (2026-09-01, unified-7dof-planner Phase 2)
----------------------------------------------------------
A plan that carries a hand track — detected via the :class:`CyclePlan` contract's
``hand_at(t) → (rev, rev_s)`` surface — additionally emits, with the SAME
lookahead discipline as the legs (τ, τ+dt, τ+2·dt):

  * ``hand_rev`` / ``hand_vel_rps``  = ``hand_at(τ)``       — the u0/v0 hand knot
  * ``hand_next_rev`` / ``hand_next_vel_rps`` = ``hand_at(τ+dt)`` — u1 + exact v1
  * ``hand_next2_rev``               = ``hand_at(τ+2·dt)``  — the u2 knot
  * ``vel_next_mm_s`` = J(pose@τ+dt) · twist@τ+dt           — the legs' exact
    u1-knot extension rates for ``v1[0:6]``, reusing the Jacobian ``_ik(pose1)``
    already computes (one extra matrix-vector product; no extra IK).

Legacy plans (no ``hand_at``) emit NONE of the six new keys — the frame is
byte-identical to the pre-Phase-2 emitter (pinned against the captured v5
fixtures), and ``make_mpc_command``'s absent-when-None rule scrubs any stale
hand key from a reused ``out=`` dict when the active plan stops carrying a
hand track mid-stream.

Pure Python + numpy (imports ``jugglebot.motion.ipc.make_mpc_command`` for the
exact field set — a pure dict builder, no ZMQ/UDP transport). No repo-root imports.
"""

from __future__ import annotations

import numpy as np

from jugglebot.motion.ik_solver import (
    accel_to_leg_accels,
    compute_jacobian,
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
    twist_to_leg_velocities,
)
from jugglebot.motion.ipc import make_mpc_command
from jugglebot.motion.torque_ff import LegTorqueFeedforward

KNOT_DT_S = 0.025    # fixed 40 Hz knot spacing (firmware SEGMENT_T_S)


class KnotEmitter:
    """Stateless (per-frame) sampler: plan + time → ``make_mpc_command`` dict.

    Args:
        geom: :class:`StewartGeometry`.
        knot_dt_s: knot spacing (fixed 40 Hz; matches firmware ``SEGMENT_T_S``).
        torque_ff: injected :class:`LegTorqueFeedforward` (tests / ablation);
            built from ``hardware_config`` when None — which, in the shipped
            config, is the DISABLED feedforward that emits exact zeros.
    """

    def __init__(self, geom, knot_dt_s: float = KNOT_DT_S,
                 torque_ff: LegTorqueFeedforward | None = None):
        self._geom = geom
        self._dt = float(knot_dt_s)
        self._mm_to_rev = np.asarray(geom.mm_to_rev, dtype=float)
        self._torque_ff = (torque_ff if torque_ff is not None
                           else LegTorqueFeedforward(geom))

    def _ik(self, pose: np.ndarray):
        """pose → (ext_mm, pos, rot, J)."""
        pos = np.asarray(pose[:3], dtype=float)
        rot = rotvec_to_rot_matrix(np.asarray(pose[3:6], dtype=float))
        ext = pose_to_leg_lengths(pos, rot, self._geom)
        J = compute_jacobian(pos, rot, self._geom)
        return ext, pos, rot, J

    def frame(self, plan, tau: float, seq: int, *, out: dict | None = None
              ) -> dict:
        """Build one ``make_mpc_command`` dict for plan time ``tau`` (seconds).

        ``tau`` is the time since the active plan was installed. ``seq`` is the
        monotonic frame counter. Returns the dict (values are ndarrays; the ipc
        ``_pack`` default handler converts them to lists on the wire).
        """
        dt = self._dt
        pose0, twist0, accel0 = plan.state_at(tau)
        pose1, twist1, _ = plan.state_at(tau + dt)
        pose2, _, _ = plan.state_at(tau + 2.0 * dt)

        ext0, pos0, rot0, J0 = self._ik(pose0)
        ext1, pos1, rot1, J1 = self._ik(pose1)
        ext2, _, _, _ = self._ik(pose2)

        leg_vel = twist_to_leg_velocities(twist0, pos0, rot0, self._geom, J=J0)
        leg_acc = accel_to_leg_accels(accel0, twist0, pos0, rot0, self._geom,
                                      J=J0)
        motor_rev = ext0 * self._mm_to_rev

        # Feedforward torque (TRUE Nm, extension-positive). Exact zeros when the
        # feature is off, so the OFF path is byte-identical to the pre-feature
        # np.zeros(6). J0 and leg_acc are handed over so the FF re-uses the
        # Jacobian and leg accelerations we already paid for.
        torque_ff = self._torque_ff.compute(
            pos0, rot0, twist0, accel0, J=J0, leg_acc_mm_s2=leg_acc)

        # ── 7th channel: only for a plan that carries a hand track (the
        # CyclePlan contract). Legacy plans emit NO new keys — the None
        # defaults below make make_mpc_command scrub them from a reused out=
        # dict, so a stale hand key can never leak into a legacy frame.
        hand_kwargs = {}
        hand_at = getattr(plan, 'hand_at', None)
        if hand_at is not None:
            h0_rev, h0_vel = hand_at(tau)
            h1_rev, h1_vel = hand_at(tau + dt)
            h2_rev, _ = hand_at(tau + 2.0 * dt)
            # Exact leg v1: the u1-knot extension rates, one matvec on the J
            # already computed by _ik(pose1) (previously discarded).
            vel_next = twist_to_leg_velocities(twist1, pos1, rot1, self._geom,
                                               J=J1)
            hand_kwargs = dict(
                vel_next_mm_s=vel_next,
                hand_rev=float(h0_rev),
                hand_vel_rps=float(h0_vel),
                hand_next_rev=float(h1_rev),
                hand_next2_rev=float(h2_rev),
                hand_next_vel_rps=float(h1_vel),
            )

        return make_mpc_command(
            ext_mm=ext0,
            pose_6dof=np.asarray(pose0, dtype=float),
            motor_rev=motor_rev,
            vel_mm_s=leg_vel,
            acc_mm_s2=leg_acc,
            torque_Nm=torque_ff,
            seq=int(seq),
            cmd_next_mm=ext1,
            cmd_next2_mm=ext2,
            out=out,
            **hand_kwargs,
        )
