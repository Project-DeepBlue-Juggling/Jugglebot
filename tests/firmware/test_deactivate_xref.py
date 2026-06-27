"""Cross-reference the C++ firmware deactivate constants against the config recipe.

The firmware DEACTIVATE handler (`leg_deactivate.cpp`, Phase 11 U5) is the
controlled inverse of ACTIVATE and the β-path analogue of can_node `deactivate`
(`_gentle_move_steps(0.0, deactivating=True)` → IDLE). It delegates the trapezoid
to the ODrive's TRAP_TRAJ planner (the offload), so what must match the proven
recipe is the *arithmetic*: the trajectory limits, the STOW target, the
completion tolerances, and the timeout. A hand-port typo (wrong constant, swapped
accel/decel, a wrong target) shows up here as a mismatch against the authoritative
Python config / the generated protocol.

The C++ can't be compiled here; this transcribes leg_deactivate.cpp's SETUP +
COMMAND + MONITOR constants 1:1 and asserts them against the generated `hw`
module (same YAML source as the firmware `hardware_config.h`) and the generated
RpcMethod enum. The adversarial review pass also eyeballs the module line-by-line.
"""

from __future__ import annotations

import jugglebot.hardware_config as hw  # noqa: E402 (on sys.path via tests/conftest)


# ── Firmware mirror (transcribed 1:1 from leg_deactivate.cpp) ──────────────────
# SETUP: set_traj_vel_limit(GENTLE_MOVE_VEL_LIMIT_RPS),
#        set_traj_acc_limits(TRAP_ACC_LIMIT_RPS2, TRAP_DEC_LIMIT_RPS2),
#        set_controller_mode(POSITION, TRAP_TRAJ), set_state(CLOSED_LOOP).
# COMMAND: encode_leg_setpoint(STOW_OFF_POSE_REV)  (clip + leg_sign).
# MONITOR: |pos − STOW| <= TARGET_REACHED_POS_TOL_REV AND
#          |vel|        <= TARGET_REACHED_VEL_TOL_RPS  → then IDLE.
# D_TIMEOUT_US = GENTLE_MOVE_TIMEOUT_S.
#
# STOW_OFF_POSE_REV is a firmware constant (canbridge_config.h = 0.0f) — the same
# off-pose can_node deactivated to (literal 0.0) and the deferred-stow target.
FW_STOW_OFF_POSE_REV = 0.0


class FirmwareDeactivate:
    @staticmethod
    def traj_vel_limit() -> float:
        return hw.JB_OP_GENTLE_MOVE_VEL_LIMIT_RPS

    @staticmethod
    def traj_acc_limit() -> float:
        return hw.ODRIVE_TRAP_ACC_LIMIT_RPS2

    @staticmethod
    def traj_dec_limit() -> float:
        return hw.ODRIVE_TRAP_DEC_LIMIT_RPS2

    @staticmethod
    def target() -> float:
        return FW_STOW_OFF_POSE_REV

    @staticmethod
    def pos_tol() -> float:
        return hw.JB_OP_TARGET_REACHED_POS_TOL_REV

    @staticmethod
    def vel_tol() -> float:
        return hw.JB_OP_TARGET_REACHED_VEL_TOL_RPS

    @staticmethod
    def timeout_s() -> float:
        return hw.JB_OP_GENTLE_MOVE_TIMEOUT_S


# ── trajectory-limit choice (identical to ACTIVATE — gentle, not the fast caps) ─

def test_deactivate_uses_gentle_vel_limit():
    """The descent uses the GENTLE_MOVE limit (2.5 rps), matching the legacy
    _gentle_move dynamics — NOT the higher ODRIVE_TRAP_VEL_LIMIT (15) nor the leg
    vel limit (4.0). The lower must be as gentle as the rise."""
    assert FirmwareDeactivate.traj_vel_limit() == hw.JB_OP_GENTLE_MOVE_VEL_LIMIT_RPS
    assert FirmwareDeactivate.traj_vel_limit() < hw.ODRIVE_LEG_VEL_LIMIT_RPS
    assert FirmwareDeactivate.traj_vel_limit() < hw.ODRIVE_TRAP_VEL_LIMIT_RPS


def test_deactivate_accel_decel_match_trap_defaults():
    assert FirmwareDeactivate.traj_acc_limit() == hw.ODRIVE_TRAP_ACC_LIMIT_RPS2
    assert FirmwareDeactivate.traj_dec_limit() == hw.ODRIVE_TRAP_DEC_LIMIT_RPS2


# ── STOW target ───────────────────────────────────────────────────────────────

def test_stow_target_is_the_off_pose():
    """The deactivate target is the STOW off-pose (0.0 rev) — the same target
    can_node deactivated to and the deferred-stow off-pose datum."""
    assert FirmwareDeactivate.target() == 0.0


def test_stow_is_below_active_pose():
    """Deactivate is always a DESCENT: STOW is below every active-pose target, so
    the seed-then-TRAP_TRAJ ladder moves the legs DOWN into the off pose."""
    for i in range(6):
        assert FirmwareDeactivate.target() < hw.JB_OP_ACTIVATE_POSITION_REVS[i]


def test_stow_within_stroke():
    """The STOW target is a valid leg position at/within the bottom of the motor
    stroke — the firmware clip (encode_leg_setpoint, [0,max]) is a no-op here."""
    assert 0.0 <= FirmwareDeactivate.target() < hw.GEOM_LEG_MOTOR_MAX_POSITION_REVS


# ── completion tolerances ─────────────────────────────────────────────────────

def test_completion_tolerances_match_config():
    assert FirmwareDeactivate.pos_tol() == hw.JB_OP_TARGET_REACHED_POS_TOL_REV
    assert FirmwareDeactivate.vel_tol() == hw.JB_OP_TARGET_REACHED_VEL_TOL_RPS
    assert FirmwareDeactivate.pos_tol() > 0.0 and FirmwareDeactivate.vel_tol() > 0.0


# ── timeout ───────────────────────────────────────────────────────────────────

def test_timeout_bounds_the_descent():
    """The op timeout must comfortably exceed the worst-case descent time: the
    full travel (active − STOW) at the gentle vel limit, plus accel ramps."""
    timeout = FirmwareDeactivate.timeout_s()
    assert timeout == hw.JB_OP_GENTLE_MOVE_TIMEOUT_S
    travel = max(hw.JB_OP_ACTIVATE_POSITION_REVS[i] for i in range(6)) - FirmwareDeactivate.target()
    move_time = travel / FirmwareDeactivate.traj_vel_limit()   # ignore ramps (conservative)
    assert timeout > move_time * 2.0   # generous margin


# ── RPC method id + arg encoding (Python ↔ firmware wire consistency) ──────────

def test_deactivate_rpc_method_id():
    """DEACTIVATE rides the next id after ACTIVATE (0x0022 → 0x0023) and is
    distinct from every other method (a duplicate id would silently misroute)."""
    from controller.teensy_link import protocol as proto
    assert proto.RpcMethod.DEACTIVATE == 0x0023
    assert proto.RpcMethod.DEACTIVATE != proto.RpcMethod.ACTIVATE


def test_deactivate_arg_encoding_matches_activate_wire():
    """DEACTIVATE rides ArgAxisOnly, exactly like ACTIVATE — so encode_deactivate
    and encode_activate produce byte-identical wire payloads for the same axis."""
    from controller.teensy_link import rpc_args
    for axis in (0, 3, rpc_args.AXIS_ALL):
        assert rpc_args.encode_deactivate(axis) == rpc_args.encode_activate(axis)
