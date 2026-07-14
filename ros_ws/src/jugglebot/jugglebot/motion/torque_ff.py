"""Leg torque feedforward — gravity (+ optional platform inertia), in TRUE Nm.

This is the *producer* half of the leg torque-feedforward feature.  It turns a
planned platform state ``(pose, twist, accel)`` into the six motor torques that
would hold / drive the platform open-loop, so the ODrive's position+velocity
cascade only has to clean up model error instead of discovering the gravity load
by watching position error accumulate.

Where this sits in the chain
----------------------------
``TrajectoryPlan.state_at(τ)``
  → :class:`LegTorqueFeedforward` (here)      — TRUE Nm, extension-positive
  → ``make_mpc_command(torque_Nm=…)`` on :5557
  → ``controller.teensy_link.setpoint_pump.SetpointPump`` — **clamp, Kt wire
    scale, ramp** (the single safety enforcement point; see that module)
  → UDP ``Setpoint.torque_ff``
  → can-bridge ``leg_interp.cpp`` → ``encode_leg_setpoint`` (applies ``leg_sign``)
  → ODrive ``Set_Input_Pos.Torque_FF`` → ``iq = input_torque / torque_constant``

THE SIGN CONVENTION (the #1 risk in this feature)
-------------------------------------------------
Everything on the Jugglebot side of ``leg_sign`` is **extension-positive**:

  * A **positive leg force** (N) acts along the leg's extension direction.
  * A **positive motor torque** (Nm) is the torque that extends the leg
    (``conversions.leg_forces_to_motor_torques`` is a pure positive scale by the
    spool radius, so it preserves the sign).
  * Extending all six legs **raises the platform**.

Therefore, to hold the platform up against gravity, **every leg's feedforward
torque must be POSITIVE** at a level pose.  That is the physical fact
``tests/motion/test_leg_torque_ff.py::test_gravity_ff_sign_holds_platform_up``
pins, and it is the assertion that would catch a sign inversion before it reaches
1.2 kg of platform on six legs.

The firmware closes the loop on that convention: ``encode_leg_setpoint``
(``odrive_protocol.h:166-179``) applies ``leg_sign`` — a negation for leg axes —
to position, vel_ff **and** torque_ff alike, exactly as ``can_node`` did.  So a
positive wire torque becomes a negative ODrive torque, which drives ODrive
position negative, which is leg extension, which is platform-up.  The Jetson must
NOT pre-negate; it must emit extension-positive Nm and let the firmware invert.

What is (and is not) computed here
----------------------------------
  * **Gravity** (``torque_ff_gravity``, default ON when the master switch is on).
    ``W_support = −W_gravity``, decomposed by ``f = J^{−T}·W`` (implemented as
    ``np.linalg.solve(J.T, W)`` — never ``J.T @ W``), then force → torque.
  * **Platform inertia + reflected rotor inertia** (``torque_ff_platform_inertia``,
    default OFF).  Newton-Euler wrench from the planned acceleration, plus
    ``J_rotor · q̈_motor``.  Default-off because the can-bridge holds the last
    torque_ff undecayed through a stale-link window — see the YAML comment.
  * **NOT friction.**  The Stribeck friction FF is a separate, velocity-signed
    model that lives in ``friction_ff_params`` / ``motor_guard`` and needs the
    smooth low-velocity gate that a hard boost band once turned into a 5 Hz
    platform limit cycle (``logbook/2026-05-08-friction-ff-platform-limit-cycle.md``).
    It is deliberately out of scope here.
  * **NO clamp, NO Kt scale, NO ramp.**  Those are safety concerns and belong at
    the single wire enforcement point (``SetpointPump``), where they also cover
    the MPC's ``HardwarePlant`` producer.  This module emits an honest physical
    torque in TRUE Nm and nothing else.

All the heavy lifting is delegated to ``jugglebot.motion.dynamics`` — this module
adds the config gating, the reuse of the caller's already-computed Jacobian and
leg accelerations, and a non-finite backstop.

Pure Python + numpy.  No ROS2 dependency.
"""

from __future__ import annotations

import logging

import numpy as np

import jugglebot.hardware_config as hw
from jugglebot.motion.conversions import leg_forces_to_motor_torques
from jugglebot.motion.dynamics import (
    DynamicsParams,
    compute_gravity_wrench,
    compute_inertia_wrench,
)
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import accel_to_leg_accels, compute_jacobian

logger = logging.getLogger(__name__)

_NUM_LEGS = 6
_TWO_PI = 2.0 * np.pi


# Consecutive bad compute() calls over which a held feedforward decays to zero
# (0.5 s at the 40 Hz knot rate).
_DECAY_CALLS = 20

# Physical sanity bound on any single leg's feedforward torque (TRUE Nm).
# Gravity+inertia at every valid pose is <= ~0.041 Nm/leg; anything past 0.5 Nm
# out of this module is a numerical artifact, not physics.  Needed because
# np.linalg.solve does NOT raise on a merely ILL-CONDITIONED Jacobian — it
# returns huge *finite* forces that would sail through the isfinite check and
# sit on the downstream clamp (review 2026-07-14: a near-singular pose can
# reach this path via seeds that bypass the feasibility gate's cond check).
_SANE_MAX_NM = 0.5


class LegTorqueFeedforward:
    """Planned platform state → six TRUE-Nm, extension-positive motor torques.

    Args:
        geom: :class:`StewartGeometry` (spool radii, mm→rev).
        params: :class:`DynamicsParams`; loaded from config when None.
        enabled: master switch.  When False, :meth:`compute` returns **exact
            zeros** — the pre-feature behaviour, byte for byte.  Defaults to
            ``hw.DYNAMICS_TORQUE_FF_ENABLED`` (False in the shipped config).
        gravity: include the gravity-wrench term.
        platform_inertia: include the Newton-Euler platform-inertia wrench and
            the reflected rotor-inertia term.

    The three flags are AND-ed with the master switch, so
    ``enabled=False`` is always exactly zeros regardless of the others.
    """

    def __init__(self, geom: StewartGeometry,
                 params: DynamicsParams | None = None,
                 *,
                 enabled: bool | None = None,
                 gravity: bool | None = None,
                 platform_inertia: bool | None = None):
        self._geom = geom
        self._params = params if params is not None else DynamicsParams.from_config()
        self.enabled = bool(hw.DYNAMICS_TORQUE_FF_ENABLED
                            if enabled is None else enabled)
        self.gravity = bool(hw.DYNAMICS_TORQUE_FF_GRAVITY
                            if gravity is None else gravity)
        self.platform_inertia = bool(hw.DYNAMICS_TORQUE_FF_PLATFORM_INERTIA
                                     if platform_inertia is None else platform_inertia)
        # Rotor inertia referred to the motor shaft (kg·m²) — the reflected term.
        self._rotor_inertia = float(self._params.motor_rotor_inertia_kgm2)
        self._mm_to_rev = np.asarray(geom.mm_to_rev, dtype=float)
        # True when the config asks for *some* term; lets `compute` short-circuit.
        self._active = self.enabled and (self.gravity or self.platform_inertia)
        # Degradation state (review 2026-07-14): on a singular Jacobian or a
        # non-finite result, HOLD the last good torque and decay it linearly to
        # zero over _DECAY_CALLS calls, instead of stepping a live feedforward
        # to exact zeros in one tick.  Gravity is pose-continuous, so a bad
        # pose is transient and the held value stays physically right; the
        # decay bounds a *persistent* failure.  (A one-tick step to zero is the
        # same discontinuity class as the 2026-05-08 friction-FF limit cycle's
        # trigger, and the pump's reject path exists precisely to avoid it.)
        self._last_tau = np.zeros(_NUM_LEGS)
        self._bad_streak = 0

    # -- introspection (telemetry / tests) --------------------------------
    @property
    def active(self) -> bool:
        """True iff a non-zero feedforward can be produced."""
        return self._active

    def describe(self) -> str:
        if not self.enabled:
            return 'leg torque FF: OFF (torque_ff_enabled=false)'
        terms = []
        if self.gravity:
            terms.append('gravity')
        if self.platform_inertia:
            terms.append('platform-inertia+reflected')
        return 'leg torque FF: ON [' + ('+'.join(terms) if terms else 'no terms') + ']'

    # -- the computation ---------------------------------------------------
    def compute(self, pos: np.ndarray, rot: np.ndarray,
                twist: np.ndarray, accel: np.ndarray,
                *,
                J: np.ndarray | None = None,
                leg_acc_mm_s2: np.ndarray | None = None) -> np.ndarray:
        """Return the six feedforward motor torques (TRUE Nm, extension-positive).

        Args:
            pos: (3,) platform offset from the stow pose (mm).
            rot: (3,3) rotation matrix, platform → world.
            twist: (6,) ``[vx,vy,vz,wx,wy,wz]`` (mm/s, rad/s), base frame.
            accel: (6,) ``[ax,ay,az,αx,αy,αz]`` (mm/s², rad/s²), base frame.
            J: (6,6) Jacobian at this pose, if the caller already has it
                (the emitter does).  Recomputed when None.
            leg_acc_mm_s2: (6,) leg extension accelerations ``J·ẍ + J̇·ẋ``, if the
                caller already has them (the emitter does).  Only used by the
                reflected-rotor term; recomputed when None **and** that term is on.

        Returns a fresh ``(6,)`` float array.  Returns exact zeros when the
        feature is off, when the Jacobian is singular, or if the result is not
        finite (a NaN on the wire makes the firmware drop the WHOLE setpoint
        frame — ``leg_interp.cpp:164`` — which is strictly worse than no
        feedforward).
        """
        if not self._active:
            return np.zeros(_NUM_LEGS)

        if J is None:
            J = compute_jacobian(pos, rot, self._geom)

        # ── Wrench the LEGS must produce, in the base frame ───────────────
        # N and N·mm.  Sign: gravity is [0,0,−m·g], so the support wrench the
        # legs must apply is [0,0,+m·g] — pushing the platform UP.
        W_total = np.zeros(6)
        if self.gravity:
            W_total -= compute_gravity_wrench(rot, self._params)
        if self.platform_inertia:
            W_total += compute_inertia_wrench(rot, twist, accel, self._params)

        # ── Wrench → per-leg forces.  f = J^{−T}·W, i.e. solve(J.T, W).
        # NOT J.T @ W: the Jacobian maps twist → leg velocities, so virtual work
        # gives J^T·f = W and the forces come from a SOLVE, not a product.
        try:
            f_legs = np.linalg.solve(J.T, W_total)
        except np.linalg.LinAlgError:
            logger.warning('LegTorqueFeedforward: singular Jacobian — holding last '
                           'FF (decaying)')
            return self._degrade()

        # Force (N, extension-positive) → motor torque (Nm, extension-positive).
        # Pure positive scale by the spool radius ⇒ sign-preserving.
        tau = leg_forces_to_motor_torques(f_legs, self._geom)

        # ── Reflected rotor inertia: τ = J_rotor · q̈_motor ────────────────
        # Rides with the platform-inertia term: it is the same
        # acceleration-proportional class (and, per the 2026-07-13 plant ID,
        # J_eff ≈ 1× J_rotor, so the rotor is the DOMINANT inertia — omitting it
        # while keeping the platform wrench would be the wrong half).
        if self.platform_inertia and self._rotor_inertia > 0.0:
            if leg_acc_mm_s2 is None:
                leg_acc_mm_s2 = accel_to_leg_accels(accel, twist, pos, rot,
                                                    self._geom, J=J)
            q_ddot_rad_s2 = (np.asarray(leg_acc_mm_s2, dtype=float)
                             * self._mm_to_rev * _TWO_PI)
            tau = tau + self._rotor_inertia * q_ddot_rad_s2

        if (not np.all(np.isfinite(tau))
                or float(np.max(np.abs(tau))) > _SANE_MAX_NM):
            logger.warning('LegTorqueFeedforward: non-finite or implausible torque '
                           '(|tau|max > %.2f Nm — ill-conditioned Jacobian?) — '
                           'holding last FF (decaying)', _SANE_MAX_NM)
            return self._degrade()

        # Good tick: remember it (copy — callers may mutate) and clear the streak.
        self._last_tau = tau.copy()
        self._bad_streak = 0
        return tau

    def _degrade(self) -> np.ndarray:
        """Held-and-decaying feedforward for a bad tick (singular J / non-finite).

        Returns the last good torque scaled by a linear decay over
        ``_DECAY_CALLS`` consecutive bad calls, reaching exact zeros at the end.
        Never a one-tick step to zero mid-stream — see the constructor comment.
        """
        self._bad_streak += 1
        if self._bad_streak >= _DECAY_CALLS:
            return np.zeros(_NUM_LEGS)
        scale = 1.0 - (self._bad_streak / float(_DECAY_CALLS))
        return self._last_tau * scale
