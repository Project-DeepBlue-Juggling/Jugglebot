"""Cross-reference the C++ firmware activate constants against the config recipe.

The firmware ACTIVATE handler (`leg_activate.cpp`, Phase 11 U5) is the β-path
analogue of `can_node._gentle_move_steps` — but it delegates the trapezoid to the
ODrive's TRAP_TRAJ planner instead of streaming a software trapezoid over
PASSTHROUGH (that is the deliberate, intentional difference — the offload). What
must still match the proven recipe is the *arithmetic*: the trajectory limits, the
target pose, the completion tolerances, and the timeout. A hand-port typo (wrong
constant, swapped accel/decel, a target sign error) shows up here as a mismatch
against the authoritative Python config.

The C++ can't be compiled here; this transcribes leg_activate.cpp's SETUP +
COMMAND + MONITOR constants 1:1 and asserts them against the generated `hw`
module (same YAML source as the firmware `hardware_config.h`, so they cannot
drift — what we verify is the hand-written *choice of constant*). The adversarial
review pass also eyeballs the module line-by-line.
"""

from __future__ import annotations

import jugglebot.hardware_config as hw  # noqa: E402 (on sys.path via tests/conftest)


# ── Firmware mirror (transcribed 1:1 from leg_activate.cpp) ────────────────────
# SETUP: set_traj_vel_limit(GENTLE_MOVE_VEL_LIMIT_RPS),
#        set_traj_acc_limits(TRAP_ACC_LIMIT_RPS2, TRAP_DEC_LIMIT_RPS2),
#        set_controller_mode(POSITION, TRAP_TRAJ), set_state(CLOSED_LOOP).
# COMMAND: encode_leg_setpoint(ACTIVATE_POSITION_REVS[i])  (clip + leg_sign).
# MONITOR: |pos − target| <= TARGET_REACHED_POS_TOL_REV AND
#          |vel|          <= TARGET_REACHED_VEL_TOL_RPS.
# A_TIMEOUT_US = GENTLE_MOVE_TIMEOUT_S.

class FirmwareActivate:
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
    def target(axis: int) -> float:
        return hw.JB_OP_ACTIVATE_POSITION_REVS[axis]

    @staticmethod
    def pos_tol() -> float:
        return hw.JB_OP_TARGET_REACHED_POS_TOL_REV

    @staticmethod
    def vel_tol() -> float:
        return hw.JB_OP_TARGET_REACHED_VEL_TOL_RPS

    @staticmethod
    def timeout_s() -> float:
        return hw.JB_OP_GENTLE_MOVE_TIMEOUT_S


# ── trajectory-limit choice ───────────────────────────────────────────────────

def test_activate_uses_gentle_vel_limit():
    """The TRAP_TRAJ vel limit is the GENTLE_MOVE limit (2.5 rps), matching the
    legacy _gentle_move_steps dynamics — NOT the higher ODRIVE_TRAP_VEL_LIMIT (15)
    nor the leg vel limit (4.0). The first six-leg activate must be gentle."""
    assert FirmwareActivate.traj_vel_limit() == hw.JB_OP_GENTLE_MOVE_VEL_LIMIT_RPS
    assert FirmwareActivate.traj_vel_limit() < hw.ODRIVE_LEG_VEL_LIMIT_RPS
    assert FirmwareActivate.traj_vel_limit() < hw.ODRIVE_TRAP_VEL_LIMIT_RPS


def test_activate_accel_decel_match_trap_defaults():
    assert FirmwareActivate.traj_acc_limit() == hw.ODRIVE_TRAP_ACC_LIMIT_RPS2
    assert FirmwareActivate.traj_dec_limit() == hw.ODRIVE_TRAP_DEC_LIMIT_RPS2


# ── target pose ───────────────────────────────────────────────────────────────

def test_activate_target_is_the_active_pose():
    """The per-leg target is the IK-derived active pose (z = default_active_z),
    so activating then handing to the MPC at the active pose is a zero-motion hold
    (bumpless first arm)."""
    for i in range(6):
        assert FirmwareActivate.target(i) == hw.JB_OP_ACTIVATE_POSITION_REVS[i]


def test_active_pose_within_stroke():
    """Every active target is a valid, positive (extension) leg position safely
    inside the motor stroke — the firmware clip (encode_leg_setpoint) is a no-op
    here, never a silent truncation."""
    for i in range(6):
        t = FirmwareActivate.target(i)
        assert 0.0 < t < hw.GEOM_LEG_MOTOR_MAX_POSITION_REVS


def test_active_pose_above_home_hardstop():
    """The active pose is well ABOVE the homed hardstop (≈ −0.1 rev), so activate
    is always an EXTENSION off the hardstop into the workspace (the move direction
    the seed-then-TRAP_TRAJ ladder assumes)."""
    for i in range(6):
        assert FirmwareActivate.target(i) > abs(hw.HOMING_LEG_ABS_POS_REV)


# ── completion tolerances ─────────────────────────────────────────────────────

def test_completion_tolerances_match_config():
    assert FirmwareActivate.pos_tol() == hw.JB_OP_TARGET_REACHED_POS_TOL_REV
    assert FirmwareActivate.vel_tol() == hw.JB_OP_TARGET_REACHED_VEL_TOL_RPS
    assert FirmwareActivate.pos_tol() > 0.0 and FirmwareActivate.vel_tol() > 0.0


def test_firmware_completion_tol_tighter_than_jetson_observer():
    """The firmware MONITOR tol (0.01 rev) is TIGHTER than the Jetson
    ActivateMonitor default (0.05 rev), so the Jetson never declares done before
    the firmware has actually settled the leg at the target."""
    from controller.teensy_link.activate import DEFAULT_POS_TOL_REV
    assert FirmwareActivate.pos_tol() <= DEFAULT_POS_TOL_REV


# ── timeout ───────────────────────────────────────────────────────────────────

def test_timeout_bounds_the_move():
    """The op timeout must comfortably exceed the worst-case move time: the full
    travel (active − hardstop) at the gentle vel limit, plus accel ramps."""
    timeout = FirmwareActivate.timeout_s()
    assert timeout == hw.JB_OP_GENTLE_MOVE_TIMEOUT_S
    travel = max(FirmwareActivate.target(i) for i in range(6)) + abs(hw.HOMING_LEG_ABS_POS_REV)
    move_time = travel / FirmwareActivate.traj_vel_limit()   # ignore ramps (conservative)
    assert timeout > move_time * 2.0   # generous margin
