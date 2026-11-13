#!/usr/bin/env python3
"""Find Balls trapped under (or resting on) the Platform in a rosbag, and say
whether a Hand stall coincides.

WHY THIS EXISTS
---------------
BUG-84: the Hand stalls mid-dive at its ~70 A clamp, ~65 mm short of the
commanded bottom. The hypothesis is a Ball trapped under the Cup. The Tracker
cannot confirm it: a Ball that has hit the Platform and jostled about is dropped
as a Ball track (no ballistic fit), so ``/balls`` shows nothing, while the raw
unlabelled marker on ``/mocap_data`` is still there. The 2026-11-13 debrief
first read ``/balls`` and wrongly called the hypothesis refuted; the owner saw
the marker in the GUI replay. This tool reads the RAW markers.

WHAT IT DOES
------------
1. ``/mocap_data`` unlabelled markers (label == "") are moved into the Platform
   frame with the pose on ``/rigid_body_poses`` (position x,y as given, z + the
   stow height ``INITIAL_HEIGHT_MM``; checked 2026-11-13: the labelled
   "Platform - 3" marker is constant to 0.3 mm in this frame).
2. Markers belonging to a rigid body are dropped: the Hand's own unlabelled
   marker rides the axis at z = HAND_MARKER_Z0_MM + 32.567 mm/rev x rev (fitted
   2026-11-13 on bag 12-50-26, 1-6 rev bins; see ``hand_marker_local_z``), and
   anything occupying one 10 mm voxel for more than ``FIXTURE_FRAC`` of the bag
   is a fixture.
3. What is left is a candidate Ball. ``region_of`` (the one geometry function)
   says whether it is UNDER_HAND (inside the arm tube, below the Hand marker) or
   ON_PLATFORM (resting on the Platform top, beside the Cup). A candidate that
   stays within ``DWELL_RADIUS_MM`` for ``DWELL_MIN_S`` is a dwell.
4. Each dwell is traced back to the ``/balls`` track that came closest to it
   shortly before it appeared (the Ball it most likely was).
5. Hand stalls come from ``/hand_telemetry`` (or a ``temp/hand_capture`` CSV):
   |iq| at the clamp, or the command more than 1 rev from the position for
   more than 50 ms with the Hand not moving. A stall COINCIDES with a dwell when
   the dwell was in place at the stall time.

USAGE
-----
    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/trapped_ball_finder.py ~/Desktop/rosbags/2026-11-13_13-06-39
    python tools/trapped_ball_finder.py BAG --hand-csv temp/hand_capture/X.csv
    python tools/trapped_ball_finder.py BAG --window T0 T1   # epoch seconds
Writes ``temp/reports/trapped_ball_<bag>.txt`` and ``.json`` and prints the text.
Exit status 0 always (an analysis tool, not a gate). A full 30 min bag takes
a few minutes (the mocap stream is decimated by ``STRIDE`` before decoding).
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import sys
from collections import Counter
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Analysis parameters (named defaults; each justified from the three known
# events of 2026-11-13: launch 1 12:02:17.66 and 12:08:30.63, launch 3
# 13:15:22.69 -- see the report scratchpad fix-trapped.md / BUG-84 note).
# ---------------------------------------------------------------------------
STRIDE = 6                    # decode every 6th mocap frame (300 Hz -> 50 Hz)
INITIAL_HEIGHT_MM = 578.2     # config jugglebot_geometry.initial_height_mm (pose z is relative)
HAND_MM_PER_REV = 32.567      # config hand_mm_per_rev
HAND_MARKER_Z0_MM = -47.0     # Hand's own unlabelled marker, Platform-frame z at rev 0
HAND_MARKER_XY_MM = (0.0, -5.0)  # ... and its (x, y); measured (-1,-9) at rev 0, (0,-1) higher
HAND_MARKER_RADIAL_TOL_MM = 25.0
HAND_MARKER_AXIAL_TOL_MM = 25.0   # + speed term below
HAND_MARKER_LAG_S = 0.03      # mocap vs telemetry skew allowed when the Hand moves fast
# A Ball SEATED in the Cup rides with the Hand: Platform-frame (x,y) about (8,23), centre
# z = -25 + 32.567 mm/rev x rev (seen 2026-11-13 bag 13-06-39: 12.8 s dwell at (3,22,-25)
# with the Hand parked at rev 0, harmless). The trapped marker of event 3 sat at the same
# (x,y) but 80 mm BELOW where the Cup was (Hand stalled at 2.6 rev), so it is not IN_CUP.
CUP_SEAT_XY_MM = (8.0, 23.0)
CUP_SEAT_Z0_MM = -25.0
CUP_SEAT_RADIAL_TOL_MM = 20.0
CUP_SEAT_AXIAL_TOL_MM = 22.0
FIXTURE_FRAC = 0.15           # one 10 mm voxel held for > 15 % of frames = fixture
UNDER_R_MM = 95.0             # arm opening radius 70 + ~25 mm ball radius
UNDER_Z_MIN_MM = -250.0       # lowest believable resting height in the tube
PLATFORM_R_MM = 230.0         # plat_radius_mm 219.075 + margin
ON_PLATFORM_Z_MM = (-60.0, 80.0)  # a resting Ball's centre; excludes airborne Balls
DWELL_RADIUS_MM = 15.0        # "roughly stationary relative to the Platform"
DWELL_GAP_S = 0.25            # occlusion gap that does not end a dwell
DWELL_MIN_S = 0.30            # listed-dwell threshold: event 3's marker sat 0.8 s, event 2's 0.37 s;
                              # shorter passes are bounces (many, harmless)
STALL_DWELL_MIN_S = 0.08      # ... but AT a stall the bar is lower: event 1's marker sat only 0.13 s
                              # (22,3,-38) because the Hand freed itself in ~150 ms
ANCESTRY_LOOKBACK_S = 4.0     # how far before the dwell a Ball track may have ended
STALL_IQ_A = 55.0             # clamp is ~70 A; matches the debrief scan rule
STALL_IQ_MIN_S = 0.02
STALL_ERR_REV = 1.0           # |cmd - pos| ...
STALL_ERR_MIN_S = 0.05        # ... for longer than this ...
STALL_STOPPED_RPS = 2.0       # ... with the Hand not moving
STALL_MERGE_S = 1.0
COINCIDE_PAD_S = 0.05         # dwell must be in place at the stall instant (+/- this); 0.3 s admitted slow
                              # Ball-in-flight passes at the Apex

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPORT_DIR = os.path.join(REPO, "temp", "reports")


# ---------------------------------------------------------------------------
# Geometry (pure)
# ---------------------------------------------------------------------------
def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def to_platform_frame(world_pts: np.ndarray, pose: Sequence[float]) -> np.ndarray:
    """World markers (N,3) mm -> Platform frame. pose = (x, y, z_rel, qx, qy, qz, qw)."""
    R = quat_to_matrix(*pose[3:7])
    c = np.array([pose[0], pose[1], pose[2] + INITIAL_HEIGHT_MM])
    return (np.asarray(world_pts, float) - c) @ R


def hand_marker_local_z(rev: float) -> float:
    """Platform-frame z of the Hand's own marker at Hand position ``rev``."""
    return HAND_MARKER_Z0_MM + HAND_MM_PER_REV * rev


def region_of(local_xyz: Sequence[float], hand_rev: float) -> Optional[str]:
    """Where a marker (Platform frame, mm) sits relative to the Cup.

    "HAND_MARKER" - it is the Hand's own marker (to be dropped, never a Ball);
    "IN_CUP"      - a Ball seated in the Cup, moving with the Hand (harmless, never a dwell);
    "UNDER_HAND"  - inside the arm tube, below the Hand marker;
    "ON_PLATFORM" - resting on the Platform top outside the tube;
    None          - anywhere else (airborne, on the floor, on the Base...).
    """
    x, y, z = float(local_xyz[0]), float(local_xyz[1]), float(local_xyz[2])
    r_hand = np.hypot(x - HAND_MARKER_XY_MM[0], y - HAND_MARKER_XY_MM[1])
    if r_hand <= HAND_MARKER_RADIAL_TOL_MM and abs(z - hand_marker_local_z(hand_rev)) <= HAND_MARKER_AXIAL_TOL_MM:
        return "HAND_MARKER"
    if (np.hypot(x - CUP_SEAT_XY_MM[0], y - CUP_SEAT_XY_MM[1]) <= CUP_SEAT_RADIAL_TOL_MM
            and abs(z - (CUP_SEAT_Z0_MM + HAND_MM_PER_REV * hand_rev)) <= CUP_SEAT_AXIAL_TOL_MM):
        return "IN_CUP"
    r = np.hypot(x, y)
    if r <= UNDER_R_MM and UNDER_Z_MIN_MM <= z < hand_marker_local_z(hand_rev):
        return "UNDER_HAND"
    if r <= PLATFORM_R_MM and ON_PLATFORM_Z_MM[0] <= z <= ON_PLATFORM_Z_MM[1]:
        return "ON_PLATFORM"
    return None


# ---------------------------------------------------------------------------
# Detection (pure; synthetic-testable)
# ---------------------------------------------------------------------------
def find_dwells(frames: Sequence[Tuple[float, Sequence[Sequence[float]], float]],
                dwell_radius: float = DWELL_RADIUS_MM, gap_s: float = DWELL_GAP_S,
                min_s: float = DWELL_MIN_S, fixture_frac: float = FIXTURE_FRAC) -> List[dict]:
    """frames: [(t, [local xyz, ...], hand_rev)] of UNLABELLED markers.

    Returns dwell dicts {t0, t1, dur_s, region, local_mean, n}, sorted by t0.
    """
    n_frames = max(len(frames), 1)
    vox: Counter = Counter()
    for _t, pts, _h in frames:
        seen = {tuple(np.floor(np.asarray(p, float) / 10.0).astype(int)) for p in pts}
        vox.update(seen)
    fixtures = {v for v, n in vox.items() if n / n_frames > fixture_frac}

    open_: List[dict] = []
    done: List[dict] = []

    def close(c: dict) -> None:
        if c["t1"] - c["t0"] >= min_s:
            m = c["sum"] / c["n"]
            rc = c["regions"]
            done.append({"t0": c["t0"], "t1": c["t1"], "dur_s": c["t1"] - c["t0"],
                         "region": max(rc, key=rc.get), "under_frac": rc.get("UNDER_HAND", 0) / c["n"],
                         "local_mean": [float(v) for v in m], "n": c["n"]})

    for t, pts, rev in frames:
        for c in [c for c in open_ if t - c["t1"] > gap_s]:
            close(c)
        open_ = [c for c in open_ if t - c["t1"] <= gap_s]
        for p in pts:
            p = np.asarray(p, float)
            if tuple(np.floor(p / 10.0).astype(int)) in fixtures:
                continue
            reg = region_of(p, rev)
            if reg in (None, "HAND_MARKER", "IN_CUP"):
                continue
            best = None
            for c in open_:
                if np.linalg.norm(p - c["anchor"]) <= dwell_radius:
                    best = c
                    break
            if best is None:
                open_.append({"anchor": p, "t0": t, "t1": t, "regions": Counter({reg: 1}), "sum": p.copy(), "n": 1})
            else:
                best["t1"] = t
                best["regions"][reg] += 1
                best["sum"] += p
                best["n"] += 1
    for c in open_:
        close(c)
    return sorted(done, key=lambda d: d["t0"])


def find_stalls(hand: np.ndarray) -> List[dict]:
    """hand: rows (t, pos_cmd, pos_meas, vel_meas, iq_meas) with iq NaN where unknown."""
    if len(hand) == 0:
        return []
    t, cmd, pos, vel, iq = hand[:, 0], hand[:, 1], hand[:, 2], hand[:, 3], hand[:, 4]
    flags = []
    for mask, min_s in ((np.abs(np.nan_to_num(iq)) >= STALL_IQ_A, STALL_IQ_MIN_S),
                        ((np.abs(cmd - pos) > STALL_ERR_REV) & (np.abs(vel) < STALL_STOPPED_RPS), STALL_ERR_MIN_S)):
        i = 0
        while i < len(mask):
            if mask[i]:
                j = i
                while j + 1 < len(mask) and mask[j + 1]:
                    j += 1
                if t[j] - t[i] >= min_s:
                    k = i + int(np.argmax(np.abs(cmd[i:j + 1] - pos[i:j + 1])))
                    flags.append({"t": float(t[i]), "t_end": float(t[j]), "pos_rev": float(pos[k]),
                                  "cmd_rev": float(cmd[k]), "peak_iq_a": float(np.nanmax(np.abs(iq[i:j + 1]))) if np.isfinite(iq[i:j + 1]).any() else None})
                i = j + 1
            else:
                i += 1
    flags.sort(key=lambda s: s["t"])
    merged: List[dict] = []
    for s in flags:
        if merged and s["t"] - merged[-1]["t_end"] <= STALL_MERGE_S:
            m = merged[-1]
            m["t_end"] = max(m["t_end"], s["t_end"])
            if s["peak_iq_a"] is not None and (m["peak_iq_a"] is None or s["peak_iq_a"] > m["peak_iq_a"]):
                m["peak_iq_a"] = s["peak_iq_a"]
        else:
            merged.append(dict(s))
    return merged


def coincide(stalls: List[dict], dwells: List[dict], pad: float = COINCIDE_PAD_S) -> None:
    """Annotate each stall with the dwells in place at its time (in place).

    ``dwells`` should come from ``find_dwells(min_s=STALL_DWELL_MIN_S)``. s["trapped"]
    holds indices into that list; s["markers"] the dwell dicts themselves.
    """
    for s in stalls:
        s["trapped"] = [i for i, d in enumerate(dwells)
                        if d["region"] == "UNDER_HAND" and d["t0"] <= s["t"] + pad and d["t1"] >= s["t"] - pad]
        s["markers"] = [dwells[i] for i in s["trapped"]]


def trace_ball(dwell_world: Sequence[float], t0: float, balls: Sequence[Tuple[float, int, float, float, float, int]],
               lookback: float = ANCESTRY_LOOKBACK_S) -> Optional[dict]:
    """balls: rows (t, id, x, y, z, status) world mm. Closest approach of any
    track sample in [t0 - lookback, t0 + 0.3] to the dwell position."""
    best = None
    w = np.asarray(dwell_world, float)
    for t, bid, x, y, z, st in balls:
        if t < t0 - lookback or t > t0 + 0.3:
            continue
        d = float(np.linalg.norm(np.array([x, y, z]) - w))
        if best is None or d < best["dist_mm"]:
            best = {"ball_id": int(bid), "dist_mm": d, "dt_s": float(t - t0), "status": int(st),
                    "last_pos_world": [float(x), float(y), float(z)]}
    return best


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------
def _bag_file(bag: str) -> str:
    fs = sorted(glob.glob(os.path.join(os.path.expanduser(bag), "*.mcap")))
    if not fs:
        raise SystemExit("no .mcap under %s" % bag)
    return fs[0]


def read_bag(bag: str, window: Optional[Tuple[float, float]], stride: int = STRIDE):
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    df = DecoderFactory()
    hand: List[tuple] = []
    balls: List[tuple] = []
    frames: List[tuple] = []     # (t, local pts (K,3), world pts (K,3), pose)
    dec: Dict[int, object] = {}
    pose = None
    raw_pose = None
    n_mocap = 0
    kw = {}
    if window:
        kw = {"start_time": int(window[0] * 1e9), "end_time": int(window[1] * 1e9)}
    with open(_bag_file(bag), "rb") as fh:
        rd = make_reader(fh)
        for schema, ch, msg in rd.iter_messages(topics=["/mocap_data", "/rigid_body_poses", "/hand_telemetry", "/balls"], **kw):
            top = ch.topic
            t = msg.log_time / 1e9
            if top == "/rigid_body_poses":
                raw_pose = (schema, msg)
                continue
            if top == "/mocap_data":
                n_mocap += 1
                if (n_mocap - 1) % stride or raw_pose is None:
                    continue
            if top == "/balls" and msg.sequence % 2:
                continue
            f = dec.get(schema.id)
            if f is None:
                f = dec[schema.id] = df.decoder_for(ch.message_encoding, schema)
            d = f(msg.data)
            if top == "/hand_telemetry":
                hand.append((t, d.pos_cmd, d.pos_meas, d.vel_meas, d.iq_meas))
            elif top == "/balls":
                for b in d.balls:
                    if b.status != 0 and b.position.z < 1100:
                        balls.append((t, b.id, b.position.x, b.position.y, b.position.z, b.status))
            else:
                pschema, pmsg = raw_pose
                pf = dec.get(pschema.id)
                if pf is None:
                    pf = dec[pschema.id] = df.decoder_for("cdr", pschema)
                for b in pf(pmsg.data).bodies:
                    if b.name == "Platform":
                        p = b.pose.pose
                        pose = (p.position.x, p.position.y, p.position.z,
                                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
                if pose is None or not np.all(np.isfinite(pose)):
                    continue
                w = np.array([[m.position.x, m.position.y, m.position.z] for m in d.markers if not m.label])
                if len(w):
                    w = w[np.all(np.isfinite(w), axis=1)]
                frames.append((t, to_platform_frame(w, pose) if len(w) else np.zeros((0, 3)), w))
    return np.array(hand, float), balls, frames


def read_hand_csv(path: str) -> np.ndarray:
    """temp/hand_capture CSV -> rows (t_wall, cmd, pos, vel, iq) on the telemetry rows."""
    rows = []
    cmd = np.nan
    iq = np.nan
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            k = r["kind"]
            if k == "echo" or (r.get("cmd_pos_rev") not in (None, "")):
                try:
                    cmd = float(r["cmd_pos_rev"])
                except (TypeError, ValueError):
                    pass
            if r.get("iq_measured") not in (None, ""):
                iq = float(r["iq_measured"])
            if k == "telem":
                rows.append((float(r["t_jetson_wall_s"]), cmd, float(r["pos_rev"]), float(r["vel_rps"]), iq))
    a = np.array(rows, float)
    a[:, 1] = np.where(np.isnan(a[:, 1]), a[:, 2], a[:, 1])
    return a


def analyse(bag: str, hand_csv: Optional[str] = None, window: Optional[Tuple[float, float]] = None) -> dict:
    hand, balls, frames = read_bag(bag, window)
    stalls = find_stalls(read_hand_csv(hand_csv) if hand_csv else hand)
    ht = hand[:, 0] if len(hand) else np.zeros(0)
    rev = lambda t: float(np.interp(t, ht, hand[:, 2])) if len(ht) else 0.0
    cframes = [(t, loc, rev(t)) for t, loc, _w in frames]
    all_dwells = find_dwells(cframes, min_s=STALL_DWELL_MIN_S)
    dwells = [d for d in all_dwells if d["dur_s"] >= DWELL_MIN_S]
    # world position of each dwell = mean of the world points of the nearest local point
    wpos = {}
    for i, d in enumerate(all_dwells):
        best = None
        for (t, loc, w) in frames:
            if t < d["t0"] or t > d["t1"] or not len(loc):
                continue
            k = int(np.argmin(np.linalg.norm(loc - np.array(d["local_mean"]), axis=1)))
            best = w[k]
            break
        wpos[i] = best
        d["world_mean"] = [float(v) for v in best] if best is not None else None
        d["ball"] = trace_ball(best, d["t0"], balls) if best is not None else None
    coincide(stalls, all_dwells)
    return {"bag": os.path.basename(os.path.normpath(bag)), "n_frames": len(frames), "dwells": dwells, "stalls": stalls,
            "params": {k: v for k, v in globals().items() if k.isupper() and isinstance(v, (int, float, tuple))}}


def _ts(t: float) -> str:
    import datetime
    return datetime.datetime.fromtimestamp(t).strftime("%H:%M:%S.%f")[:-3]


def render(res: dict) -> str:
    L = ["Trapped-ball finder: bag %s (%d mocap frames analysed)" % (res["bag"], res["n_frames"]), ""]
    L.append("(a) Unlabelled-marker dwells (>= %.2f s, within %.0f mm), Platform frame mm:" % (DWELL_MIN_S, DWELL_RADIUS_MM))
    if not res["dwells"]:
        L.append("  none")
    for i, d in enumerate(res["dwells"]):
        b = d.get("ball")
        bs = ("Ball %d (closest approach %.0f mm, track sample %+.2f s from dwell start)" % (b["ball_id"], b["dist_mm"], b["dt_s"])) if b else "no Ball track near"
        L.append("  #%d %s %s..%s (%.2f s) at (%.0f,%.0f,%.0f) under-Hand %.0f%% of samples  <- %s" % (
            i, d["region"], _ts(d["t0"]), _ts(d["t1"]), d["dur_s"], *d["local_mean"], 100 * d["under_frac"], bs))
    L += ["", "(b) Hand stalls (|iq|>=%.0f A or |cmd-pos|>%.1f rev, stopped):" % (STALL_IQ_A, STALL_ERR_REV)]
    if not res["stalls"]:
        L.append("  none")
    for s in res["stalls"]:
        tr = s["trapped"]
        L.append("  %s..%s pos %.2f cmd %.2f peak iq %s A -> %s" % (
            _ts(s["t"]), _ts(s["t_end"]), s["pos_rev"], s["cmd_rev"],
            "%.1f" % s["peak_iq_a"] if s["peak_iq_a"] is not None else "n/a",
            ("TRAPPED MARKER COINCIDES: " + "; ".join("%s..%s (%.2f s) at (%.0f,%.0f,%.0f)%s" % (
                _ts(m["t0"]), _ts(m["t1"]), m["dur_s"], *m["local_mean"],
                (" <- Ball %d" % m["ball"]["ball_id"]) if m.get("ball") else "") for m in s["markers"]))
            if tr else "no trapped marker (cannot see)"))
    L.append("")
    return "\n".join(L)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("bag")
    ap.add_argument("--hand-csv", default=None)
    ap.add_argument("--window", nargs=2, type=float, metavar=("T0", "T1"), default=None)
    ap.add_argument("--out-dir", default=REPORT_DIR)
    a = ap.parse_args(argv)
    res = analyse(a.bag, a.hand_csv, tuple(a.window) if a.window else None)
    txt = render(res)
    os.makedirs(a.out_dir, exist_ok=True)
    base = os.path.join(a.out_dir, "trapped_ball_%s" % res["bag"])
    with open(base + ".txt", "w") as fh:
        fh.write(txt)
    with open(base + ".json", "w") as fh:
        json.dump(res, fh, indent=1, default=float)
    print(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
