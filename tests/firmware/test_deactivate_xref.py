"""Config-sanity + cross-file/protocol pins for the firmware DEACTIVATE op.

**Most of this file was retired in the Fable-5 hardening pass ([6]).** The
behavioural transcriptions of leg_deactivate.cpp's SETUP/COMMAND constants (checked
`== hw.*`, and a STOW target that compared one Python literal `0.0` against
another) were TAUTOLOGICAL/brittle. They are now SUPERSEDED by the compiled native
driver `tests/firmware/native/test_leg_deactivate.cpp`, which asserts the REAL
emitted CAN frames byte-for-byte — including `encode_leg_setpoint(i,
STOW_OFF_POSE_REV, …)` against the actual firmware header constant (so a firmware
edit to STOW_OFF_POSE_REV IS now caught, which the old literal-vs-literal test was
not) and the IDLE-on-arrival that distinguishes deactivate from activate.

What remains here is what a single-TU native test CANNOT see:
  * the Jetson-side RpcMethod id + rpc_args wire-encoding parity (a DIFFERENT
    module — controller.teensy_link, not leg_deactivate.cpp);
  * CONFIG-SANITY invariants among same-header constants (descent direction, stroke
    bound, gentle limit, timeout margin) — a pure-Python layer that runs even when
    g++ is absent and the native harness skips.
"""

from __future__ import annotations

import jugglebot.hardware_config as hw  # noqa: E402 (on sys.path via tests/conftest)

# STOW_OFF_POSE_REV is a firmware constant (canbridge_config.h = 0.0f). Kept here as
# a plain datum for the relationship checks below; the firmware's ACTUAL emitted
# STOW target is pinned by native/test_leg_deactivate.cpp against the real header.
FW_STOW_OFF_POSE_REV = 0.0


class FirmwareDeactivate:
    """Thin mirror of the leg_deactivate.cpp constants the surviving pins reference."""

    @staticmethod
    def traj_vel_limit() -> float:
        return hw.JB_OP_GENTLE_MOVE_VEL_LIMIT_RPS

    @staticmethod
    def target() -> float:
        return FW_STOW_OFF_POSE_REV

    @staticmethod
    def timeout_s() -> float:
        return hw.JB_OP_GENTLE_MOVE_TIMEOUT_S


# ── config-sanity: descent, gentle limit, stroke bound, timeout ───────────────

def test_deactivate_vel_limit_is_gentler_than_the_fast_limits():
    """The descent uses a limit GENTLER than the leg + fast TRAP vel limits — the
    lower must be as gentle as the rise. (The exact emitted value is pinned by
    native/test_leg_deactivate.cpp.)"""
    assert FirmwareDeactivate.traj_vel_limit() < hw.ODRIVE_LEG_VEL_LIMIT_RPS
    assert FirmwareDeactivate.traj_vel_limit() < hw.ODRIVE_TRAP_VEL_LIMIT_RPS


def test_stow_is_below_active_pose():
    """Deactivate is always a DESCENT: STOW is below every active-pose target, so the
    seed-then-TRAP_TRAJ ladder moves the legs DOWN into the off pose."""
    for i in range(6):
        assert FirmwareDeactivate.target() < hw.JB_OP_ACTIVATE_POSITION_REVS[i]


def test_stow_within_stroke():
    """The STOW target is a valid leg position at/within the bottom of the motor
    stroke — the firmware clip ([0,max]) is a no-op here."""
    assert 0.0 <= FirmwareDeactivate.target() < hw.GEOM_LEG_MOTOR_MAX_POSITION_REVS


def test_timeout_bounds_the_descent():
    """The op timeout must comfortably exceed the worst-case descent time: the full
    travel (active − STOW) at the gentle vel limit, plus accel ramps."""
    timeout = FirmwareDeactivate.timeout_s()
    travel = max(hw.JB_OP_ACTIVATE_POSITION_REVS[i] for i in range(6)) - FirmwareDeactivate.target()
    move_time = travel / FirmwareDeactivate.traj_vel_limit()   # ignore ramps (conservative)
    assert timeout > move_time * 2.0   # generous margin


# ── cross-file/protocol: RpcMethod id + rpc_args encoding (a DIFFERENT module) ──

def test_deactivate_rpc_method_id():
    """DEACTIVATE rides the next id after ACTIVATE (0x0022 → 0x0023) and is distinct
    from every other method (a duplicate id would silently misroute). This pins the
    Jetson-side protocol enum, which no leg_deactivate.cpp native test can see."""
    from controller.teensy_link import protocol as proto
    assert proto.RpcMethod.DEACTIVATE == 0x0023
    assert proto.RpcMethod.DEACTIVATE != proto.RpcMethod.ACTIVATE


def test_deactivate_arg_encoding_matches_activate_wire():
    """DEACTIVATE rides ArgAxisOnly, exactly like ACTIVATE — so encode_deactivate and
    encode_activate produce byte-identical wire payloads for the same axis (a
    Jetson-side rpc_args parity, not a firmware-behaviour concern)."""
    from controller.teensy_link import rpc_args
    for axis in (0, 3, rpc_args.AXIS_ALL):
        assert rpc_args.encode_deactivate(axis) == rpc_args.encode_activate(axis)
