"""Config-sanity + cross-file pins for the firmware ACTIVATE op.

**Most of this file was retired in the Fable-5 hardening pass ([6]).** The
behavioural assertions that used to transcribe leg_activate.cpp's SETUP/COMMAND
constants and check them `== hw.*` were TAUTOLOGICAL: the mirror returned the same
`hw.*` value it compared against. They are now SUPERSEDED by the compiled native
driver `tests/firmware/native/test_leg_activate.cpp`, which asserts the REAL
emitted CAN frames byte-for-byte against the ODrive encoders (so the firmware's
actual choice of vel/acc limit, target pose, and completion tol is pinned by
running the real code, not by re-reading the constant).

What remains here is what a single-TU native test CANNOT see:
  * a CROSS-FILE pin of the firmware MONITOR tol against the Jetson observer
    constant (a different module);
  * CONFIG-SANITY invariants among same-header constants (active pose inside the
    stroke / above the hardstop, gentle-vs-fast limit ordering, timeout margin) —
    kept as a PURE-PYTHON layer that runs even when g++ is absent and the native
    harness skips.
"""

from __future__ import annotations

import jugglebot.hardware_config as hw  # noqa: E402 (on sys.path via tests/conftest)


class FirmwareActivate:
    """Thin mirror of the leg_activate.cpp constants the surviving pins reference.
    (The behavioural transcription moved to native/test_leg_activate.cpp.)"""

    @staticmethod
    def traj_vel_limit() -> float:
        return hw.JB_OP_GENTLE_MOVE_VEL_LIMIT_RPS

    @staticmethod
    def target(axis: int) -> float:
        return hw.JB_OP_ACTIVATE_POSITION_REVS[axis]

    @staticmethod
    def pos_tol() -> float:
        return hw.JB_OP_TARGET_REACHED_POS_TOL_REV

    @staticmethod
    def timeout_s() -> float:
        return hw.JB_OP_GENTLE_MOVE_TIMEOUT_S


# ── config-sanity: gentle-vs-fast limit ordering ──────────────────────────────

def test_activate_vel_limit_is_gentler_than_the_fast_limits():
    """The activate TRAP_TRAJ vel limit must be GENTLER than both the leg vel limit
    and the fast TRAP vel limit — the first six-leg activate must not be quick. (The
    exact value the firmware emits is pinned by native/test_leg_activate.cpp.)"""
    assert FirmwareActivate.traj_vel_limit() < hw.ODRIVE_LEG_VEL_LIMIT_RPS
    assert FirmwareActivate.traj_vel_limit() < hw.ODRIVE_TRAP_VEL_LIMIT_RPS


# ── config-sanity: the active pose is a valid extension target ────────────────

def test_active_pose_within_stroke():
    """Every active target is a valid, positive (extension) leg position safely
    inside the motor stroke — the firmware clip is a no-op here, never a silent
    truncation."""
    for i in range(6):
        t = FirmwareActivate.target(i)
        assert 0.0 < t < hw.GEOM_LEG_MOTOR_MAX_POSITION_REVS


def test_active_pose_above_home_hardstop():
    """The active pose is well ABOVE the homed hardstop (≈ −0.1 rev), so activate is
    always an EXTENSION off the hardstop into the workspace (the direction the
    seed-then-TRAP_TRAJ ladder assumes)."""
    for i in range(6):
        assert FirmwareActivate.target(i) > abs(hw.HOMING_LEG_ABS_POS_REV)


# ── cross-file: firmware tol vs the Jetson observer (a DIFFERENT module) ───────

def test_firmware_completion_tol_tighter_than_jetson_observer():
    """The firmware MONITOR tol (0.01 rev) is TIGHTER than the Jetson ActivateMonitor
    default (0.05 rev), so the Jetson never declares done before the firmware has
    settled the leg at the target. A single-TU native test cannot see the Python
    controller constant, so this cross-file pin stays here."""
    from controller.teensy_link.activate import DEFAULT_POS_TOL_REV
    assert FirmwareActivate.pos_tol() > 0.0
    assert FirmwareActivate.pos_tol() <= DEFAULT_POS_TOL_REV


# ── config/physics-sanity: the timeout bounds the worst-case move ─────────────

def test_timeout_bounds_the_move():
    """The op timeout must comfortably exceed the worst-case move time: the full
    travel (active − hardstop) at the gentle vel limit, plus accel ramps."""
    timeout = FirmwareActivate.timeout_s()
    travel = max(FirmwareActivate.target(i) for i in range(6)) + abs(hw.HOMING_LEG_ABS_POS_REV)
    move_time = travel / FirmwareActivate.traj_vel_limit()   # ignore ramps (conservative)
    assert timeout > move_time * 2.0   # generous margin
