"""Offline timing helpers for bag analysis (BUG-134). Pure numpy, no bag, no ROS.

WHICH STAMP TO USE FOR TIMING ANALYSIS (the one place this is written down)
---------------------------------------------------------------------------
  /leg_cmd_executed header   bridge wall clock at the telemetry tick, the sample
                             time of the COMMAND. Use as is.
  Platform IMU t_wall_us     bridge wall clock via the Platform. Trust it only on
                             Platform FW >= 15, or when the capture's
                             flags.clock_suspect is false (BUG-132).
  /robot_state               NOT its `timestamp`: that is Jetson publish time,
                             12 to 14 ms after the tick. Use
                             robot_state_bridge_time() below. It still leaves the
                             encoder sample age (0 to 10 ms per axis, unexported
                             until can-bridge FW 50).
  /mocap_data .stamp         the envelope stamp the Tracker uses, via
                             mocap_time_on_bridge_clock() below.
  /rigid_body_poses          never the per-body stamps: they are raw packet
                             receive times and scatter 10 to 27 ms.
  /hand_telemetry            as /robot_state (appended bridge_stamp when valid).

Clock convention: /clock_diag err_us = Jetson minus bridge wall clock at the last
anchor, so a bridge stamp t_B is the Jetson instant t_B + err.
"""
from __future__ import annotations

import numpy as np

# ALIGNMENT constant for analysis, not a measured QTM latency. Measured 2026-11-20/22/23
# over seven captures: the /mocap_data envelope stamp runs 13.05 ms (sd 1.39 ms; orientation
# fit, position fit 13.3 ms sd 0.7) after the bridge-stamped /leg_cmd_executed once the
# bridge-to-Jetson clock error (err_us) is removed, from body motion against the FK of the
# executed command. It therefore includes QTM latency PLUS the Legs' own tracking lag
# behind the command (1.3 to 3 ms modelled; about 13 ms MEASURED on the bridge tick, 2026-11-27, BUG-151, and the pairing fallback below reads it about 5.6 ms short), so QTM's true latency against physical
# truth is only bounded to about 4 to 13 ms (a Ball Butler yaw cross-check disagrees).
# Do NOT apply it to the live Mocap stamp: the Ball Butler release constants and the
# bb_calibration lag advisory rest on the uncorrected stamp.
MOCAP_ENVELOPE_LATE_VS_BRIDGE_S = 0.013

SRC_NONE, SRC_FIELD, SRC_PAIRED = 0, 1, 2


def robot_state_bridge_time(stamp_s, log_s, bridge_stamp_s, bridge_valid,
                            cmd_stamp_s, cmd_log_s, max_pair_gap_s=0.004):
    """Best bridge tick time per /robot_state sample.

    Order: the message's own bridge_stamp where bridge_valid; else the stamp of the
    /leg_cmd_executed message nearest in bag log time if within max_pair_gap_s; else
    NaN. Never falls back to the publish stamp `stamp_s` (kept in the signature so
    callers pass the message as a whole; it is not used on purpose).

    Returns (t_bridge_s, source) with source codes SRC_NONE=0, SRC_FIELD=1, SRC_PAIRED=2.
    """
    log_s = np.asarray(log_s, float)
    n = len(log_s)
    t = np.full(n, np.nan)
    src = np.zeros(n, int)
    cmd_stamp_s = np.asarray(cmd_stamp_s, float)
    cmd_log_s = np.asarray(cmd_log_s, float)
    if len(cmd_log_s):
        o = np.argsort(cmd_log_s)
        cl, cs = cmd_log_s[o], cmd_stamp_s[o]
        idx = np.clip(np.searchsorted(cl, log_s), 1, max(len(cl) - 1, 1)) if len(cl) > 1 else np.zeros(n, int)
        if len(cl) > 1:
            left = np.abs(cl[idx - 1] - log_s) <= np.abs(cl[idx] - log_s)
            idx = np.where(left, idx - 1, idx)
        near = np.abs(cl[idx] - log_s) <= max_pair_gap_s
        t[near] = cs[idx][near]
        src[near] = SRC_PAIRED
    if bridge_stamp_s is not None and bridge_valid is not None:
        bs = np.asarray(bridge_stamp_s, float)
        ok = np.asarray(bridge_valid, bool) & np.isfinite(bs) & (bs > 0)
        t[ok] = bs[ok]
        src[ok] = SRC_FIELD
    return t, src


def mocap_time_on_bridge_clock(envelope_stamp_s, clock_err_s=0.0,
                               late_s=MOCAP_ENVELOPE_LATE_VS_BRIDGE_S):
    """Bridge-clock time of the physical state a Mocap frame reports (alignment).

    envelope_stamp_s is on the Jetson clock; clock_err_s is /clock_diag err_us / 1e6
    (Jetson minus bridge). Result = stamp - err - late: a stamp T with err 0.003 s maps
    to T - 0.016 s. Worked: T = 100.000, err = 0.0034 -> 99.9836.
    """
    return np.asarray(envelope_stamp_s, float) - clock_err_s - late_s
