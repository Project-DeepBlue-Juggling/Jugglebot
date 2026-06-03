"""Pure-Python port of the Teensy 500 Hz interpolator — the C++ translation target.

This module is a deliberately C++-shaped, scalar/per-leg-loop transcription of
the can-bridge firmware's `leg_interp.cpp`. It exists so the interpolator math
can be validated against the real `motor_guard.py` offline (see xref.py): the
C++ `leg_interp.cpp` mirrors THIS file line-for-line, so proving this matches
`motor_guard` proves the firmware port is correct (within float precision).

Ported from motor_guard.py's `_interpolate_and_send` method (the ~870-1049
method; the ladder proper is ~894-1048): the Hermite → Taylor → velocity-decay
ladder + lead-clamp + stroke-clamp. No numpy: lists + explicit per-leg loops,
exactly as the C++ does. Keep the two in sync. (Reference the method name, not
the line numbers, when they drift.)

NOTE: friction feedforward is intentionally NOT ported here — it is a separate
additive torque term (motor_guard `_compute_friction_ff_Nm`) that does not affect
the commanded *position* or *velocity*, which is what this ladder produces and
what the xref compares. Porting friction FF to the Teensy is a documented
follow-on (see handoff).
"""

from __future__ import annotations

NUM_LEGS = 6

# Constants ported 1:1 from motor_guard.py / canbridge_config.h.
SEGMENT_T_S = 0.025
MAX_EXTRAP_DT_S = 0.05
EXTRAP_DECAY_DT_S = 0.06
JERK_EMA_ALPHA = 0.3
MAX_LEAD_REV = 0.15


class TeensyLegInterp:
    """Scalar, per-leg port of the motor_guard interpolation ladder.

    The constructor takes the per-leg stroke clamp bounds (rev); these mirror the
    firmware's STROKE_MIN_REV / STROKE_MAX_REV and motor_guard's
    _stroke_min_rev / _stroke_max_rev.
    """

    def __init__(self, stroke_min_rev, stroke_max_rev):
        self.stroke_min = list(stroke_min_rev)
        self.stroke_max = list(stroke_max_rev)

        # Latched base state (per leg).
        self.base_pos = [0.0] * NUM_LEGS
        self.base_vel = [0.0] * NUM_LEGS
        self.base_accel = [0.0] * NUM_LEGS
        self.base_torque = [0.0] * NUM_LEGS
        self.jerk = [0.0] * NUM_LEGS
        self.base_timestamp = 0.0

        self.next_pos = None   # list[6] or None
        self.next2_pos = None  # list[6] or None

        # Jerk-estimation history.
        self.prev_accel = None  # list[6] or None
        self.prev_timestamp = 0.0

        # Outputs.
        self.cmd_pos = [0.0] * NUM_LEGS
        self.cmd_vel = [0.0] * NUM_LEGS
        self.cmd_torque = [0.0] * NUM_LEGS

    # ── Latch a new MPC setpoint (port of the relevant parts of _on_mpc_command) ──
    def latch_setpoint(self, u0, v0, accel, torque, t_latch,
                       u1=None, u2=None):
        """u0/u1/u2 in motor-rev, v0 in rev/s, accel in rev/s^2, torque in Nm.

        u1=None → no forward waypoint (Taylor fallback). u2 only meaningful with u1.
        """
        # Jerk EMA from consecutive accelerations (motor_guard._on_mpc_command).
        if self.prev_accel is not None:
            dt_mpc = t_latch - self.prev_timestamp
            if dt_mpc > 1e-6:
                for i in range(NUM_LEGS):
                    raw = (accel[i] - self.prev_accel[i]) / dt_mpc
                    self.jerk[i] = JERK_EMA_ALPHA * raw + (1.0 - JERK_EMA_ALPHA) * self.jerk[i]
        else:
            self.jerk = [0.0] * NUM_LEGS

        for i in range(NUM_LEGS):
            self.base_pos[i] = u0[i]
            self.base_vel[i] = v0[i]
            self.base_accel[i] = accel[i]
            self.base_torque[i] = torque[i]
        self.base_timestamp = t_latch

        self.next_pos = list(u1) if u1 is not None else None
        self.next2_pos = list(u2) if (u2 is not None and u1 is not None) else None

        self.prev_accel = list(accel)
        self.prev_timestamp = t_latch

    # ── 500 Hz tick (port of _interpolate_and_send) ───────────────────────────
    def tick(self, t_now, fb_pos):
        """Compute the per-leg commanded position/velocity for the current tick.

        fb_pos[6] is the latest motor feedback (rev), used by the lead clamp.
        Returns (cmd_pos, cmd_vel, cmd_torque), each a list[6].
        """
        dt = t_now - self.base_timestamp

        if self.next_pos is not None:
            # ── Mode 1: cubic Hermite between u0 and u1 ──
            T = SEGMENT_T_S
            s = dt / T
            if s > 1.0:
                s = 1.0
            s2 = s * s
            s3 = s2 * s
            h00 = 2.0 * s3 - 3.0 * s2 + 1.0
            h10 = s3 - 2.0 * s2 + s
            h01 = -2.0 * s3 + 3.0 * s2
            h11 = s3 - s2
            inv_T = 1.0 / T
            dh00 = (6.0 * s2 - 6.0 * s) * inv_T
            dh10 = 3.0 * s2 - 4.0 * s + 1.0
            dh01 = (-6.0 * s2 + 6.0 * s) * inv_T
            dh11 = 3.0 * s2 - 2.0 * s
            for i in range(NUM_LEGS):
                p0 = self.base_pos[i]
                p1 = self.next_pos[i]
                v0 = self.base_vel[i]
                if self.next2_pos is not None:
                    v1 = (self.next2_pos[i] - p1) / T
                else:
                    v1 = (p1 - p0) / T
                self.cmd_pos[i] = h00 * p0 + h10 * (T * v0) + h01 * p1 + h11 * (T * v1)
                self.cmd_vel[i] = dh00 * p0 + dh10 * v0 + dh01 * p1 + dh11 * v1

        elif dt <= MAX_EXTRAP_DT_S:
            # ── Mode 2: cubic Taylor extrapolation ──
            dt2 = dt * dt
            for i in range(NUM_LEGS):
                self.cmd_pos[i] = (self.base_pos[i]
                                   + self.base_vel[i] * dt
                                   + 0.5 * self.base_accel[i] * dt2
                                   + (1.0 / 6.0) * self.jerk[i] * (dt2 * dt))
                self.cmd_vel[i] = (self.base_vel[i]
                                   + self.base_accel[i] * dt
                                   + 0.5 * self.jerk[i] * dt2)

        else:
            # ── Mode 3: velocity decay to zero ──
            dt_b2 = MAX_EXTRAP_DT_S * MAX_EXTRAP_DT_S
            dt_over = dt - MAX_EXTRAP_DT_S
            decay_frac = 1.0 - dt_over / EXTRAP_DECAY_DT_S
            if decay_frac < 0.0:
                decay_frac = 0.0
            for i in range(NUM_LEGS):
                vel_b = (self.base_vel[i]
                         + self.base_accel[i] * MAX_EXTRAP_DT_S
                         + 0.5 * self.jerk[i] * dt_b2)
                pos_b = (self.base_pos[i]
                         + self.base_vel[i] * MAX_EXTRAP_DT_S
                         + 0.5 * self.base_accel[i] * dt_b2
                         + (1.0 / 6.0) * self.jerk[i] * (dt_b2 * MAX_EXTRAP_DT_S))
                if dt_over >= EXTRAP_DECAY_DT_S:
                    extra = vel_b * (EXTRAP_DECAY_DT_S * 0.5)
                else:
                    extra = vel_b * dt_over * (1.0 - dt_over / (2.0 * EXTRAP_DECAY_DT_S))
                self.cmd_pos[i] = pos_b + extra
                self.cmd_vel[i] = vel_b * decay_frac

        # torque_ff passes through (friction FF not ported — see module docstring).
        for i in range(NUM_LEGS):
            self.cmd_torque[i] = self.base_torque[i]

        # ── Lead clamp: never run more than MAX_LEAD_REV ahead of encoder ──
        for i in range(NUM_LEGS):
            pre = self.cmd_pos[i]
            dev = self.cmd_pos[i] - fb_pos[i]
            if dev > MAX_LEAD_REV:
                dev = MAX_LEAD_REV
            elif dev < -MAX_LEAD_REV:
                dev = -MAX_LEAD_REV
            self.cmd_pos[i] = fb_pos[i] + dev
            if self.cmd_pos[i] != pre:        # leg was lead-clamped → zero vel_ff
                self.cmd_vel[i] = 0.0

        # ── Stroke clamp: backstop against extrapolation past physical limits ──
        for i in range(NUM_LEGS):
            pre = self.cmd_pos[i]
            if self.cmd_pos[i] < self.stroke_min[i]:
                self.cmd_pos[i] = self.stroke_min[i]
            elif self.cmd_pos[i] > self.stroke_max[i]:
                self.cmd_pos[i] = self.stroke_max[i]
            if self.cmd_pos[i] != pre:        # clamped → zero vel_ff and torque
                self.cmd_vel[i] = 0.0
                self.cmd_torque[i] = 0.0

        return (list(self.cmd_pos), list(self.cmd_vel), list(self.cmd_torque))
