#!/usr/bin/env python3
"""Bit-for-bit cross-check: Teensy interpolator port vs. the real motor_guard.

Drives BOTH the production `MotorGuard._interpolate_and_send` and the C++-target
`TeensyLegInterp.tick` (teensy_interp.py) through identical inputs — a recorded
MPC command stream plus synthetic cases covering all three ladder modes — and
reports the maximum per-tick commanded-position divergence. Acceptance: < 1e-6 rev
(Phase 7 "done when").

Method (faithful-replay, mirrors the offline-replay discipline in
logbook/2026-05-20): motor_guard's wall clock is monkeypatched to a controllable
clock so the run is deterministic; the guard's interpolation base state is set
identically to the port's, isolating the ladder math (the actual port target)
from motor_guard's separate workspace/deviation/estop machinery (which stays on
the Jetson and is NOT ported).

Run:  python xref.py [recorded_mpc.csv]
      (defaults to the newest temp/logs/mpc_*.csv)
"""

from __future__ import annotations

import csv
import glob
import os
import sys
from unittest import mock

import numpy as np

# ── Repo paths ────────────────────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, "..", "..", "..", ".."))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_REPO, "ros_ws", "src", "jugglebot"))
sys.path.insert(0, os.path.join(_REPO, "config", "generated"))

import jugglebot.motion.motor_guard as mg          # noqa: E402
from jugglebot.motion.geometry import StewartGeometry  # noqa: E402
from jugglebot.motion.conversions import leg_velocities_to_motor_velocities  # noqa: E402
from teensy_interp import TeensyLegInterp, NUM_LEGS  # noqa: E402

TICK_DT = 0.002        # 500 Hz
MPC_DT = 0.025         # 40 Hz
SUBTICKS = 12          # ~24 ms of sub-ticks per MPC segment (< MPC_DT, so Hermite stays in-segment)


class _Clock:
    """Controllable perf_counter for deterministic replay."""
    def __init__(self):
        self.t = 0.0
    def perf_counter(self):
        return self.t


class _DummyIPC:
    """No-op IPC so MotorGuard needs no ZMQ broker."""
    def recv_all(self):
        return []
    def send_telemetry(self, msg):
        pass
    @property
    def seconds_since_last_recv(self):
        return 0.0
    def close(self):
        pass


def _make_guard():
    geom = StewartGeometry()
    guard = mg.MotorGuard(geom=geom, ipc=_DummyIPC())
    guard.mode = mg.GuardMode.ENABLED
    return guard, geom


def _copy_base_into_guard(guard, port, t_latch):
    """Make the guard's interpolation base identical to the port's."""
    guard._mpc_base_pos_rev = np.array(port.base_pos, dtype=float)
    guard._mpc_base_vel_rps = np.array(port.base_vel, dtype=float)
    guard._mpc_base_accel_rps2 = np.array(port.base_accel, dtype=float)
    guard._mpc_base_torque_Nm = np.array(port.base_torque, dtype=float)
    guard._mpc_base_jerk_rps3 = np.array(port.jerk, dtype=float)
    guard._mpc_next_pos_rev = (np.array(port.next_pos, dtype=float)
                               if port.next_pos is not None else None)
    guard._mpc_next2_pos_rev = (np.array(port.next2_pos, dtype=float)
                                if port.next2_pos is not None else None)
    guard._mpc_base_timestamp = t_latch
    guard._mpc_segment_T = 0.025
    guard._has_mpc_cmd = True


def _step_compare(guard, port, clock, t_now, fb):
    """Advance both to t_now with feedback fb; return max |pos| and |vel| diff."""
    clock.t = t_now
    guard._has_motor_fb = True
    guard._motor_fb_pos_rev = np.array(fb, dtype=float)
    guard._motor_fb_vel_rps = np.zeros(NUM_LEGS)
    guard._motor_fb_cur_A = np.zeros(NUM_LEGS)
    guard._motor_fb_timestamp = t_now
    guard._interpolate_and_send(TICK_DT)
    g_pos = guard._commanded_pos_rev.copy()
    g_vel = guard._commanded_vel_ff_rps.copy()

    t_pos, t_vel, _ = port.tick(t_now, fb)
    dpos = float(np.max(np.abs(g_pos - np.array(t_pos))))
    dvel = float(np.max(np.abs(g_vel - np.array(t_vel))))
    return dpos, dvel


def run_recorded_xref(csv_path):
    """Replay a recorded MPC cmd_ext stream through both interpolators."""
    guard, geom = _make_guard()
    mm_to_rev = np.array(geom.mm_to_rev)
    port = TeensyLegInterp(guard._stroke_min_rev, guard._stroke_max_rev)

    # Sanity: the firmware's embedded stroke bounds must equal the live guard's.
    # (legbridge_config.h STROKE_{MIN,MAX}_REV — checked in the pytest wrapper.)

    rows = []
    with open(csv_path) as f:
        for r in csv.DictReader(f):
            try:
                ext = [float(r[f"cmd_ext_{i}"]) for i in range(NUM_LEGS)]
                act = [float(r[f"actual_ext_{i}"]) for i in range(NUM_LEGS)]
            except (KeyError, ValueError):
                continue
            rows.append((ext, act))
    if len(rows) < 3:
        raise RuntimeError(f"{csv_path}: too few usable rows ({len(rows)})")

    max_dpos = max_dvel = 0.0
    clock = _Clock()
    with mock.patch.object(mg, "time", clock):
        prev_ext_rev = None
        for k in range(len(rows) - 2):
            t_latch = k * MPC_DT
            u0 = (np.array(rows[k][0]) * mm_to_rev).tolist()
            u1 = (np.array(rows[k + 1][0]) * mm_to_rev).tolist()
            u2 = (np.array(rows[k + 2][0]) * mm_to_rev).tolist()
            # MPC velocity: chord (u1-u0)/MPC_DT in mm/s → motor rev/s via the
            # production conversion (matches what hardware_plant would supply).
            vel_mm_s = ((np.array(rows[k + 1][0]) - np.array(rows[k][0])) / MPC_DT)
            v0 = np.array(leg_velocities_to_motor_velocities(vel_mm_s, geom)).tolist()
            accel = ([0.0] * NUM_LEGS if prev_ext_rev is None
                     else ((np.array(v0) - prev_ext_rev) / MPC_DT).tolist())
            torque = [0.0] * NUM_LEGS
            port.latch_setpoint(u0, v0, accel, torque, t_latch, u1=u1, u2=u2)
            _copy_base_into_guard(guard, port, t_latch)
            prev_ext_rev = np.array(v0)

            # Feedback = the recorded actual encoder position (realistic; exercises
            # the lead clamp). Both interpolators get the identical fb.
            fb = (np.array(rows[k][1]) * mm_to_rev).tolist()
            for s in range(1, SUBTICKS + 1):
                t_now = t_latch + s * TICK_DT
                dpos, dvel = _step_compare(guard, port, clock, t_now, fb)
                max_dpos = max(max_dpos, dpos)
                max_dvel = max(max_dvel, dvel)
    return max_dpos, max_dvel, len(rows)


def run_synthetic_xref():
    """Cover all three ladder modes + clamps with synthetic, deterministic inputs."""
    guard, geom = _make_guard()
    port = TeensyLegInterp(guard._stroke_min_rev, guard._stroke_max_rev)
    clock = _Clock()
    max_dpos = max_dvel = 0.0

    cases = []
    # (label, u1_present, u2_present, dt_list, big_jump)
    cases.append(("hermite_in_segment", True, True, [0.000, 0.005, 0.012, 0.020, 0.024], False))
    cases.append(("hermite_late_clamp_s1", True, True, [0.030, 0.040, 0.060], False))
    cases.append(("hermite_no_u2", True, False, [0.000, 0.010, 0.020], False))
    cases.append(("taylor", False, False, [0.000, 0.010, 0.030, 0.050], False))
    cases.append(("velocity_decay", False, False, [0.055, 0.070, 0.090, 0.110, 0.200], False))
    cases.append(("lead_clamp_big_jump", True, True, [0.005, 0.015, 0.024], True))

    base = [0.5 + 0.3 * i for i in range(NUM_LEGS)]
    with mock.patch.object(mg, "time", clock):
        for label, has_u1, has_u2, dts, big in cases:
            u0 = list(base)
            step = 0.4 if not big else 1.5    # big jump → lead clamp engages
            u1 = [u0[i] + step for i in range(NUM_LEGS)]
            u2 = [u1[i] + step for i in range(NUM_LEGS)]
            v0 = [(u1[i] - u0[i]) / MPC_DT for i in range(NUM_LEGS)]
            accel = [2.0 * (1 + i) for i in range(NUM_LEGS)]
            torque = [0.1 * (1 + i) for i in range(NUM_LEGS)]
            # Two latches so the jerk EMA has history (matches a running stream).
            port.latch_setpoint(u0, v0, [0.0] * NUM_LEGS, torque, 0.0,
                                u1=(u1 if has_u1 else None), u2=(u2 if has_u2 else None))
            port.latch_setpoint(u0, v0, accel, torque, MPC_DT,
                                u1=(u1 if has_u1 else None), u2=(u2 if has_u2 else None))
            _copy_base_into_guard(guard, port, MPC_DT)
            fb = list(u0)   # feedback at the segment base
            for dt in dts:
                t_now = MPC_DT + dt
                dpos, dvel = _step_compare(guard, port, clock, t_now, fb)
                max_dpos = max(max_dpos, dpos)
                max_dvel = max(max_dvel, dvel)
    return max_dpos, max_dvel


def _newest_csv():
    cands = sorted(glob.glob(os.path.join(_REPO, "temp", "logs", "mpc_*.csv")),
                   key=os.path.getmtime, reverse=True)
    return cands[0] if cands else None


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else _newest_csv()
    print("=== Teensy interpolator ⟷ motor_guard cross-check ===")
    syn_pos, syn_vel = run_synthetic_xref()
    print(f"synthetic : max |Δpos| = {syn_pos:.3e} rev   max |Δvel| = {syn_vel:.3e} rev/s")
    rec_pos = rec_vel = 0.0
    if csv_path and os.path.exists(csv_path):
        rec_pos, rec_vel, n = run_recorded_xref(csv_path)
        print(f"recorded  : {os.path.basename(csv_path)} ({n} rows)")
        print(f"            max |Δpos| = {rec_pos:.3e} rev   max |Δvel| = {rec_vel:.3e} rev/s")
    else:
        print("recorded  : (no temp/logs/mpc_*.csv found — synthetic only)")
    worst = max(syn_pos, rec_pos)
    ok = worst < 1e-6
    print(f"--- {'PASS' if ok else 'FAIL'}: worst |Δpos| = {worst:.3e} rev (threshold 1e-6) ---")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
