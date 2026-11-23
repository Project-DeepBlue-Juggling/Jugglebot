#!/usr/bin/env python3
"""Offline check of a Platform IMU capture's wall-clock stamps (BUG-132).

WHAT IT MEASURES
----------------
How far the IMU's ``t_wall_us`` stamps are from the rest of the robot's clock,
using only the capture and the rosbag recorded alongside it. Two lags, both the
shift of the IMU gyro against the body angular rate of an FK orientation
(Kabsch-aligned, so no mount calibration is needed; correlation over the 3
axes, 2 ms grid then 0.2 ms refinement):

  lag1  IMU gyro vs the encoder FK orientation from ``/robot_state``, after
        RESTAMPING each ``/robot_state`` sample to the bridge stamp of the
        paired ``/leg_cmd_executed`` message (same telemetry tick; paired by
        nearest bag log time). This isolates the IMU clock (P) from the
        bridge/Teensy clock (B).
  lag2  IMU gyro vs the body rate of the ``/leg_cmd_executed`` FK orientation
        (bridge-stamped, the COMMAND). The Legs follow the command within a few
        ms, so a healthy IMU reads a small POSITIVE lag2; an IMU that leads
        the command is impossible, so a clearly negative lag2 means the IMU
        stamps are early.

WHY THE ENCODERS ARE RESTAMPED
------------------------------
``/robot_state`` carries a Jetson publish-time stamp that is 12 to 20 ms LATE
relative to the telemetry tick that produced the encoder values. Used as is,
it makes an on-time IMU look early by that amount. The paired
``/leg_cmd_executed`` stamp is the bridge's stamp of the same tick, so
copying it onto the encoder sample removes the publish delay. (The residual
encoder sample age, about 5.5 ms, is NOT removed; lag1 is read as is.)

SIGN CONVENTION
---------------
lag = -s, where s is the shift that, added to the IMU stamps, aligns it to the
reference. NEGATIVE = the IMU stamps EARLY (a sample is stamped before it was
taken), e.g. lag1 = -40 ms means every IMU timestamp is 40 ms too small.

OUTPUT
------
Prints a table and writes ``temp/probes/imu_clock_check/<stem>_<ts>.json``
with lag1 and its peak correlation r1, lag2 and r2, the mean
(``/robot_state`` stamp minus paired bridge stamp) in ms, the sliding-window
sd of lag1 (4 s windows, step 2 s, fixed rotation, r > 0.7), the capture's
``fw_version`` and ``flags.clock_suspect`` (None if the sidecar lacks it).
Verdict: PASS when |lag1| <= --max-abs-ms AND 0 <= lag2 <= +10 ms; FAIL
otherwise; INSUFFICIENT_MOTION when the capture's gyro rms (after median bias
removal and a 15 Hz low-pass, norm over 3 axes) is below ``MIN_GYRO_RMS_RPS``.
Exit 0 PASS, 1 FAIL, 2 not measurable.

MIN_GYRO_RMS_RPS = 0.02 rad/s (1.1 dps), measured 2026-11-23: the two resting
captures (20261120 C33B, AEDE) read 0.0003 and 0.0004 rad/s, the five moving
reference captures 0.064 to 0.151 rad/s. 0.02 sits ~50x above the resting floor
and ~3x below the weakest moving capture (572E), so a lag fitted above it is
driven by real rotation, not noise.

REFERENCE NUMBERS (measured 2026-11-23, this method, ms; negative = early)
--------------------------------------------------------------------------
  capture (platform FW 12-14)       lag1    lag2
  20261120 AEDF / AEE0 / AEE1    -2.2/-2.8/-3.4   +5.2/+3.6/+3.8
  20261123 4AFA                      -4.2           +6.8
  20261122 572E / 5730 / 5731   -48.8/-43.0/-40.0  -38.4/-33.2/-33.0
The three 2026-11-22 captures were taken after the Platform RX-queue delay
grew; the defect is fixed in Platform FW 15 (BUG-132).

Usage
-----
  python tools/probes/imu_clock_check.py --capture <stem|path> [--bag <dir>]
         [--max-abs-ms 5] [--captures-dir D] [--out-dir D]
Without --bag the bag under ~/Desktop/rosbags/ whose metadata.yaml time span
contains the capture is used. Run with the project venv (numpy, scipy,
mcap, mcap-ros2-support, pyyaml).
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))

MIN_GYRO_RMS_RPS = 0.02
LAG2_LO_MS = 0.0
LAG2_HI_MS = 10.0
DEFAULT_CAPTURE_DIRS = (
    os.path.join(REPO, "temp", "imu_capture"),
    os.path.expanduser("~/Desktop/Jugglebot/temp/imu_capture"),
)


# --------------------------------------------------------------------- pure pieces
def lowpass(x, fs, fc=15.0):
    from scipy.signal import butter, sosfiltfilt
    return sosfiltfilt(butter(4, fc / (fs / 2), output="sos"), x, axis=0)


def omega_body(t, R):
    """Body-frame angular rate at 2-sample-span midpoints. R: scipy Rotation array."""
    k = 2
    rv = (R[:-k].inv() * R[k:]).as_rotvec()
    dt = t[k:] - t[:-k]
    return 0.5 * (t[k:] + t[:-k]), rv / dt[:, None]


def kabsch(A, B):
    """Rotation Rk with Rk @ a ~ b for row samples."""
    H = A.T @ B
    U, _S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    return Vt.T @ np.diag([1.0, 1.0, d]) @ U.T


def _score(t, g, tm, w, s, R=None, mask=None):
    gi = np.stack([np.interp(tm - s, t, g[:, i], left=np.nan, right=np.nan) for i in range(3)], 1)
    ok = np.isfinite(gi).all(1) & np.isfinite(w).all(1)
    if mask is not None:
        ok = ok & mask
    if ok.sum() < 200:
        return np.nan, None
    Rk = R if R is not None else kabsch(gi[ok], w[ok])
    r = np.corrcoef((gi[ok] @ Rk.T).ravel(), w[ok].ravel())[0, 1]
    return r, Rk


def estimate_lag(t, g, tm, w, lo=-0.15, hi=0.10, coarse=0.002, fine=0.0002):
    """(lag_ms, peak_r, Rk). IMU gyro g(t) against reference rate w(tm).

    Shift s is added to the IMU stamps (IMU sample at stamp t is compared with the
    reference at t + s); lag_ms = -s * 1e3, so an IMU that stamps 40 ms EARLY gives -40.
    """
    grid = np.arange(lo, hi, coarse)
    sc = np.array([_score(t, g, tm, w, s)[0] for s in grid])
    if np.all(np.isnan(sc)):
        return float("nan"), float("nan"), None
    i = int(np.nanargmax(sc))
    g2 = np.arange(grid[i] - 1.5 * coarse, grid[i] + 1.5 * coarse + 1e-9, fine)
    out = [_score(t, g, tm, w, s) for s in g2]
    r2 = np.array([o[0] for o in out])
    j = int(np.nanargmax(r2))
    return -g2[j] * 1e3, float(r2[j]), out[j][1]


def window_lags(t, g, tm, w, s, Rk, win=4.0, step=2.0, half=0.04, rmin=0.7):
    """Per-window lag (ms) with the rotation held fixed; windows with r <= rmin dropped."""
    lags = []
    for a in np.arange(max(t[0], tm[0]), min(t[-1], tm[-1]) - win, step):
        mk = (tm >= a) & (tm < a + win)
        gr = np.arange(s - half, s + half, 0.001)
        rv = np.array([_score(t, g, tm, w, x, R=Rk, mask=mk)[0] for x in gr])
        if np.all(np.isnan(rv)):
            continue
        k = int(np.nanargmax(rv))
        if rv[k] > rmin:
            lags.append(-gr[k] * 1e3)
    return np.array(lags)


def verdict(lag1_ms, lag2_ms, max_abs_ms, gyro_rms_rps,
            min_rms=MIN_GYRO_RMS_RPS, lag2_lo=LAG2_LO_MS, lag2_hi=LAG2_HI_MS):
    """('PASS'|'FAIL'|'INSUFFICIENT_MOTION', exit_code, reason)."""
    if not np.isfinite(gyro_rms_rps) or gyro_rms_rps < min_rms:
        return "INSUFFICIENT_MOTION", 2, "gyro rms %.3f rad/s < %.3f" % (gyro_rms_rps, min_rms)
    if not (np.isfinite(lag1_ms) and np.isfinite(lag2_ms)):
        return "INSUFFICIENT_MOTION", 2, "a lag could not be measured"
    bad = []
    if abs(lag1_ms) > max_abs_ms:
        bad.append("|lag1| %.1f > %.1f ms" % (abs(lag1_ms), max_abs_ms))
    if not (lag2_lo <= lag2_ms <= lag2_hi):
        bad.append("lag2 %.1f outside [%.0f, %.0f] ms" % (lag2_ms, lag2_lo, lag2_hi))
    return ("FAIL", 1, "; ".join(bad)) if bad else ("PASS", 0, "")


# ------------------------------------------------------------------ data plumbing
def resolve_capture(arg, dirs=()):
    """(json_path, npz_path) from a stem or any path with/without extension."""
    base = arg[:-5] if arg.endswith(".json") else arg[:-4] if arg.endswith(".npz") else arg
    cands = [base] if os.path.dirname(base) else []
    for d in tuple(dirs) + DEFAULT_CAPTURE_DIRS:
        cands.append(os.path.join(d, base))
    for c in cands:
        if os.path.exists(c + ".npz"):
            return c + ".json", c + ".npz"
    raise SystemExit("capture not found: %s (looked in %s)" % (arg, cands))


def find_bag(t0, t1, root="~/Desktop/rosbags"):
    import yaml
    hits = []
    for md in sorted(glob.glob(os.path.join(os.path.expanduser(root), "*", "metadata.yaml"))):
        try:
            m = yaml.safe_load(open(md))["rosbag2_bagfile_information"]
            a = m["starting_time"]["nanoseconds_since_epoch"] / 1e9
            b = a + m["duration"]["nanoseconds"] / 1e9
        except Exception:
            continue
        if a <= t0 and t1 <= b:
            hits.append(os.path.dirname(md))
    if not hits:
        raise SystemExit("no bag under %s spans [%.1f, %.1f]; pass --bag" % (root, t0, t1))
    return hits[0]


def read_bag(bag, t0, t1):
    """robot_state rows [log_t, stamp, 6 pos] and leg_cmd_executed rows [log_t, stamp, 6 pos]."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    files = sorted(glob.glob(os.path.join(os.path.expanduser(bag), "*.mcap")))
    if not files:
        raise SystemExit("no .mcap in %s" % bag)
    st = lambda s: s.sec + s.nanosec * 1e-9
    df = DecoderFactory()
    dec = {}
    rs, cm = [], []
    for f in files:
        with open(f, "rb") as fh:
            for s, c, m in make_reader(fh).iter_messages(
                    topics=["/robot_state", "/leg_cmd_executed"],
                    start_time=int((t0 - 1) * 1e9), end_time=int((t1 + 1) * 1e9)):
                d = dec.get(s.id) or dec.setdefault(s.id, df.decoder_for(c.message_encoding, s))
                d = d(m.data)
                lt = m.log_time / 1e9
                if c.topic == "/robot_state":
                    rs.append([lt, st(d.timestamp)] + [x.pos_estimate for x in d.motor_states[:6]])
                else:
                    cm.append([lt, st(d.header.stamp)] + list(d.position)[:6])
    return np.array(rs, float), np.array(cm, float)


def restamp(r, c):
    """Copy onto each robot_state row the bridge stamp of the nearest-in-log-time cmd row.

    Returns (restamped rows, original stamp minus paired bridge stamp in seconds).
    """
    c = c[np.unique(c[:, 1], return_index=True)[1]]
    c = c[np.argsort(c[:, 0])]
    r = r[np.argsort(r[:, 0])]
    idx = np.clip(np.searchsorted(c[:, 0], r[:, 0]), 1, len(c) - 1)
    left_better = np.abs(c[idx - 1, 0] - r[:, 0]) <= np.abs(c[idx, 0] - r[:, 0])
    idx = np.where(left_better, idx - 1, idx)
    out = r.copy()
    out[:, 1] = c[idx, 1]
    return out, r[:, 1] - c[idx, 1]


def fk_rotations(revs):
    sys.path[:0] = [REPO, os.path.join(REPO, "ros_ws", "src", "jugglebot")]
    from jugglebot.motion.geometry import StewartGeometry
    from jugglebot.motion import ik_solver as ik, conversions as cv
    geom = StewartGeometry()
    ext = cv.revs_to_extensions_mm(revs, geom)
    out, guess = [], None
    for e in ext:
        try:
            p, Rm, _ = ik.leg_lengths_to_pose(e, geom, initial_guess=guess)
            guess = (p, Rm)
            out.append(Rm)
        except Exception:
            out.append(np.full((3, 3), np.nan))
    return np.array(out)


def ref_rate(rows):
    """(tm, body rate, 15 Hz low-pass at a nominal 100 Hz) of the FK orientation of
    rows [.., stamp, 6 revs]."""
    from scipy.spatial.transform import Rotation as Rot
    rows = rows[np.unique(rows[:, 1], return_index=True)[1]]
    Rm = fk_rotations(rows[:, 2:8])
    ok = np.isfinite(Rm).all((1, 2))
    rows, Rm = rows[ok], Rm[ok]
    tm, w = omega_body(rows[:, 1], Rot.from_matrix(Rm))
    return tm, lowpass(w, 100.0)


def load_gyro(npz):
    z = np.load(npz)
    t = z["t_wall_us"] / 1e6
    v = z["valid"]
    g = z["gyro_dps"].astype(float)[v]
    t = t[v]
    if len(t) < 200:  # empty or aborted capture: nothing to measure
        return t, np.zeros((len(t), 3)), float("nan")
    fs = 1.0 / np.median(np.diff(t))
    g = np.deg2rad(g - np.median(g, axis=0))
    return t, lowpass(g, fs), fs


# ------------------------------------------------------------------------- main
def run(capture, bag=None, max_abs_ms=5.0, captures_dir=None, out_dir=None):
    jpath, npath = resolve_capture(capture, dirs=(captures_dir,) if captures_dir else ())
    side = json.load(open(jpath)) if os.path.exists(jpath) else {}
    t, g, fs = load_gyro(npath)
    rms = float(np.sqrt(np.mean(np.sum(g ** 2, axis=1)))) if len(t) >= 200 else float("nan")
    res = {"capture": os.path.basename(npath)[:-4], "fw_version": (side.get("header") or {}).get("fw_version"),
           "clock_suspect": (side.get("flags") or {}).get("clock_suspect"),
           "gyro_rms_rps": rms, "max_abs_ms": max_abs_ms, "min_gyro_rms_rps": MIN_GYRO_RMS_RPS}
    lag1 = lag2 = r1 = r2 = float("nan")
    if rms >= MIN_GYRO_RMS_RPS:
        bag = bag or find_bag(t[0], t[-1])
        res["bag"] = os.path.abspath(os.path.expanduser(bag))
        rs, cm = read_bag(bag, t[0], t[-1])
        if len(rs) > 100 and len(cm) > 100:
            r_new, age = restamp(rs, cm)
            res["mean_robot_state_stamp_minus_bridge_ms"] = float(np.mean(age) * 1e3)
            tm, w = ref_rate(r_new)
            lag1, r1, Rk = estimate_lag(t, g, tm, w)
            wl = window_lags(t, g, tm, w, -lag1 / 1e3, Rk) if Rk is not None else np.array([])
            res["window_lag1_sd_ms"] = float(np.std(wl)) if len(wl) >= 3 else None
            res["window_lag1_n"] = int(len(wl))
            tm2, w2 = ref_rate(cm)
            lag2, r2, _ = estimate_lag(t, g, tm2, w2)
        else:
            res["bag_note"] = "too few /robot_state or /leg_cmd_executed messages in the capture window"
    v, code, why = verdict(lag1, lag2, max_abs_ms, rms)
    res.update(lag1_ms=lag1, r1=r1, lag2_ms=lag2, r2=r2, verdict=v, exit_code=code, reason=why)
    out_dir = out_dir or os.path.join(REPO, "temp", "probes", "imu_clock_check")
    os.makedirs(out_dir, exist_ok=True)
    op = os.path.join(out_dir, "%s_%s.json" % (res["capture"], time.strftime("%Y%m%dT%H%M%S")))
    json.dump(res, open(op, "w"), indent=1, sort_keys=True, default=float)
    res["out"] = op
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--capture", required=True)
    ap.add_argument("--bag")
    ap.add_argument("--max-abs-ms", type=float, default=5.0)
    ap.add_argument("--captures-dir")
    ap.add_argument("--out-dir")
    a = ap.parse_args(argv)
    r = run(a.capture, a.bag, a.max_abs_ms, a.captures_dir, a.out_dir)
    f = lambda k, fmt="%.1f": (fmt % r[k]) if isinstance(r.get(k), (int, float)) and np.isfinite(r[k]) else "n/a"
    print("%s fw=%s clock_suspect=%s gyro_rms=%.3f rad/s" % (r["capture"], r["fw_version"], r["clock_suspect"], r["gyro_rms_rps"]))
    sd = r.get("window_lag1_sd_ms")
    print("  lag1 %s ms (r=%s) | lag2 %s ms (r=%s) | rs-stamp minus bridge %s ms | window sd %s ms (n=%s)" % (
        f("lag1_ms"), f("r1", "%.3f"), f("lag2_ms"), f("r2", "%.3f"),
        f("mean_robot_state_stamp_minus_bridge_ms"),
        ("%.1f" % sd) if sd is not None else "n/a", r.get("window_lag1_n", "n/a")))
    print("  VERDICT %s %s -> %s" % (r["verdict"], r["reason"], r["out"]))
    return r["exit_code"]


if __name__ == "__main__":
    sys.exit(main())
