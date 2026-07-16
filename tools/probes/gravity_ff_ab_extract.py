#!/usr/bin/env python3
"""A/B extraction + metrics for gravity-feedforward hardware sessions (rosbag pairs).

Reusable scenario harness (tools/probes rule): reads TWO mcap bags — a
feedforward-OFF baseline and a feedforward-ON session — with identical
treatment, prints every derived number (never hand-assemble derived tables),
and writes a metrics JSON to temp/probes/.

First use: the 2026-07-16 arming A/B (logbook/2026-07-16-gravity-ff-armed.md;
procedure tests/hardware/session_torque_ff.md) —
  BAG A (FF OFF): ~/Desktop/rosbags/2026-07-16_10-12-33/
  BAG B (FF ON):  ~/Desktop/rosbags/2026-07-16_10-17-45/
Pre-registered re-use: the "B.seq1 first-move iq outlier" follow-up (watch on
the next session) re-runs exactly this extraction + pos-delta move matching.

Key design points a future session should not rediscover:
- Match moves by PER-LEG POS-DELTA, never by move_seq index (bag A's seq 5 was
  a null move, silently desynchronizing index-based pairing).
- Compare hold currents only between IDENTICAL-APPROACH-HISTORY holds
  (post-activate vs post-activate): same-pose holds differ by up to ±1.5 A/leg
  with different approach history — dwarfing the 0.2-0.7 A sign signatures.
- At a static hold, total iq is FF-invariant (the velocity integrator re-pins
  it); only composition shifts. Equal hold currents A vs B is CORRECT.

Empirical facts baked into thresholds (probed 2026-07-16 before writing):
- vel_estimate is quantized at 0.01525879 rev/s LSB; hold-noise reaches
  ~0.09 rev/s (6 LSB), real motion is ~1-2.8 rev/s -> static gate 0.1 rev/s.
- The activation lift (stow -> active pose, 0 -> ~2.19 rev, trap-traj path)
  completes right around the arm instant, so the arm-edge check starts at
  lift quiescence, not blindly at arm-1 s.
- plan_kind stays 'move' for the whole 13-move battery; moves are segmented
  by move_seq transitions in /trajectory/diagnostics, with the actual motion
  window found from robot_state velocities.
- live_deviation values saturate at 0.10 = MAX_LEAD (rev) -> units are motor rev.

Run: source ~/Desktop/PDJ_venv/venv/bin/activate && python ff_ab_extract.py
"""
import json
import math
import os

import numpy as np
import yaml
from mcap_ros2.reader import read_ros2_messages

SCRATCH = os.path.dirname(os.path.abspath(__file__))
OUT_JSON = os.path.join(SCRATCH, "ff_ab_metrics.json")

BAGS = {
    "A": "/home/jetson/Desktop/rosbags/2026-07-16_10-12-33",
    "B": "/home/jetson/Desktop/rosbags/2026-07-16_10-17-45",
}
FF_LABEL = {"A": "FF OFF (baseline)", "B": "FF ON"}

N_LEGS = 6  # axis 6 = hand, excluded

# --- thresholds (all reported, none hidden) ---
VEL_STATIC_THRESH = 0.1    # rev/s: above = real motion; hold noise tops out ~0.09
HOLD_MIN_S = 5.0           # minimum static-window length
POSE_MATCH_TOL_REV = 0.02  # per-leg mean-pos tolerance for matching windows A<->B
QUIET_SUSTAIN_S = 0.3      # all-legs-quiet must persist this long to call lift done
SETTLE_FACTOR = 1.5        # settle threshold = 1.5x per-leg hold-noise floor
SETTLE_HOLD_S = 0.5        # must stay below threshold this long to count as settled
POST_MOVE_INT_S = 2.0      # integral window after move end
STATIC_MIN_AFTER_ARM_S = 3.05  # static windows for iq comparison start after arm+this
                               # (B's FF ramp is full at arm+2.0 s)


def extract_bag(path_dir):
    """Single-pass extraction of everything we need from one bag."""
    mcap_files = [f for f in os.listdir(path_dir) if f.endswith(".mcap")]
    assert len(mcap_files) == 1, mcap_files
    path = os.path.join(path_dir, mcap_files[0])

    meta = yaml.safe_load(open(os.path.join(path_dir, "metadata.yaml")))
    info = meta["rosbag2_bagfile_information"]
    t0_ns = info["starting_time"]["nanoseconds_since_epoch"]

    topic_counts = {
        t["topic_metadata"]["name"]: {
            "type": t["topic_metadata"]["type"],
            "count": t["message_count"],
        }
        for t in info["topics_with_message_count"]
    }

    d = {
        "path": path,
        "t0_ns": t0_ns,
        "duration_s": info["duration"]["nanoseconds"] / 1e9,
        "topic_counts": topic_counts,
        "ls_t": [], "ls_kv": [],
        "rs_t": [], "rs_pos": [], "rs_vel": [], "rs_iqm": [], "rs_iqs": [],
        "st_t": [], "st_plan": [], "st_mode": [], "st_stream": [],
        "st_rej": [], "st_seq": [],
        "dg_t": [], "dg_kv": [],
        "orch": [], "cmode": [], "orch_cmd": [],
        "ltd_t": [], "ltd_data": [],
        "se_t": [], "se_data": [],
    }

    for m in read_ros2_messages(path):
        t = (m.log_time_ns - t0_ns) / 1e9
        topic = m.channel.topic
        r = m.ros_msg
        if topic == "/link_status":
            kv = {x.key: x.value for x in r.values}
            kv["_fault_msg"] = r.message
            kv["_level"] = int(r.level)
            d["ls_t"].append(t)
            d["ls_kv"].append(kv)
        elif topic == "/robot_state":
            ms = r.motor_states
            d["rs_t"].append(t)
            d["rs_pos"].append([ms[i].pos_estimate for i in range(N_LEGS)])
            d["rs_vel"].append([ms[i].vel_estimate for i in range(N_LEGS)])
            d["rs_iqm"].append([ms[i].iq_measured for i in range(N_LEGS)])
            d["rs_iqs"].append([ms[i].iq_setpoint for i in range(N_LEGS)])
        elif topic == "/trajectory/status":
            d["st_t"].append(t)
            d["st_plan"].append(r.plan_kind)
            d["st_mode"].append(r.mode)
            d["st_stream"].append(bool(r.streaming))
            d["st_rej"].append(r.last_rejection)
            d["st_seq"].append(int(r.seq))
        elif topic == "/trajectory/diagnostics":
            kv = {x.key: x.value for x in r.values}
            d["dg_t"].append(t)
            d["dg_kv"].append(kv)
        elif topic == "/orchestrator_state":
            if not d["orch"] or d["orch"][-1][1] != r.data:
                d["orch"].append((t, r.data))
        elif topic == "/control_mode_topic":
            if not d["cmode"] or d["cmode"][-1][1] != r.data:
                d["cmode"].append((t, r.data))
        elif topic == "/orchestrator_command":
            d["orch_cmd"].append((t, r.data))
        elif topic == "/leg_torques_diagnostic":
            d["ltd_t"].append(t)
            d["ltd_data"].append(list(r.data))
        elif topic == "/leg_setpoint_echo":
            d["se_t"].append(t)
            d["se_data"].append(list(r.data))

    for k in ("rs_pos", "rs_vel", "rs_iqm", "rs_iqs"):
        d[k] = np.asarray(d[k], dtype=float)
    d["rs_t"] = np.asarray(d["rs_t"])
    d["se_t"] = np.asarray(d["se_t"])
    d["dg_t_arr"] = np.asarray(d["dg_t"])
    # Despiked speed: 5-sample median filter per leg kills the 1-2-sample
    # vel_estimate noise spikes (up to 0.153 rev/s) that otherwise fragment
    # static windows and pollute motion runs; real moves are 28+ samples.
    from scipy.ndimage import median_filter
    d["rs_speed_f"] = median_filter(np.abs(d["rs_vel"]), size=(5, 1))
    return d


def link_status_series(d, key, cast=float):
    ts, vs = [], []
    for t, kv in zip(d["ls_t"], d["ls_kv"]):
        if key in kv:
            try:
                ts.append(t)
                vs.append(cast(kv[key]))
            except ValueError:
                ts.pop()
    return np.asarray(ts), np.asarray(vs)


def live_deviation_matrix(d):
    ts, rows = [], []
    for t, kv in zip(d["ls_t"], d["ls_kv"]):
        s = kv.get("live_deviation", "")
        parts = s.split(",")
        if len(parts) == N_LEGS:
            try:
                rows.append([float(p) for p in parts])
                ts.append(t)
            except ValueError:
                pass
    return np.asarray(ts), np.asarray(rows)


def transitions(ts, vals):
    out = []
    prev = None
    for t, v in zip(ts, vals):
        if prev is None or v != prev:
            out.append((t, prev, v))
            prev = v
    return out


def find_arm_time(d):
    ts, vs = link_status_series(d, "mpc_active", int)
    for i in range(1, len(vs)):
        if vs[i - 1] == 0 and vs[i] == 1:
            return float(ts[i])
    return None


def quiet_runs(rt, speed_f, i_lo, i_hi):
    """Contiguous index runs in [i_lo,i_hi) where ALL legs despiked speed < thresh."""
    quiet = np.all(speed_f[i_lo:i_hi] < VEL_STATIC_THRESH, axis=1)
    runs = []
    start = None
    for j, q in enumerate(list(quiet) + [False]):
        if q and start is None:
            start = j
        elif not q and start is not None:
            runs.append((i_lo + start, i_lo + j))  # [incl, excl)
            start = None
    return runs


def lift_quiescence_time(d, t_from):
    """First time >= t_from where all legs stay quiet for QUIET_SUSTAIN_S.

    Uses UNFILTERED |vel| deliberately: the median filter declares quiet
    ~0.3 s early on the decaying lift tail, which leaks ~0.004 rev of
    residual lift creep into the arm-edge p2p. Raw gating is conservative
    (later onset) which is what a no-kick check wants.
    """
    rt = d["rs_t"]
    i0 = np.searchsorted(rt, t_from)
    for a, b in quiet_runs(rt, np.abs(d["rs_vel"]), i0, len(rt)):
        if rt[b - 1] - rt[a] >= QUIET_SUSTAIN_S:
            return float(rt[a])
    return None


def static_windows(d, t_min):
    """Global static windows (all legs |vel| < thresh for >= HOLD_MIN_S), t >= t_min."""
    rt = d["rs_t"]
    i0 = np.searchsorted(rt, t_min)
    wins = []
    for a, b in quiet_runs(rt, d["rs_speed_f"], i0, len(rt)):
        if rt[b - 1] - rt[a] < HOLD_MIN_S:
            continue
        sl = slice(a, b)
        # plan_kind at window midpoint
        tm = 0.5 * (rt[a] + rt[b - 1])
        pk = ""
        if d["st_t"]:
            k = max(0, np.searchsorted(np.asarray(d["st_t"]), tm) - 1)
            pk = d["st_plan"][k]
        wins.append({
            "t_start": float(rt[a]), "t_end": float(rt[b - 1]),
            "dur_s": float(rt[b - 1] - rt[a]),
            "plan_kind": pk,
            "pos_mean": d["rs_pos"][sl].mean(axis=0).tolist(),
            "pos_std": d["rs_pos"][sl].std(axis=0).tolist(),
            "vel_absmax": np.abs(d["rs_vel"][sl]).max(axis=0).tolist(),
            "iqm_mean": d["rs_iqm"][sl].mean(axis=0).tolist(),
            "iqm_std": d["rs_iqm"][sl].std(axis=0).tolist(),
            "iqs_mean": d["rs_iqs"][sl].mean(axis=0).tolist(),
            "iqs_std": d["rs_iqs"][sl].std(axis=0).tolist(),
            "n_samples": int(b - a),
        })
    return wins


def move_seq_segments(d):
    """(seq, t_start, t_end) from move_seq transitions in /trajectory/diagnostics."""
    segs = []
    prev = None
    for t, kv in zip(d["dg_t"], d["dg_kv"]):
        s = kv.get("move_seq")
        if s != prev:
            if prev is not None and prev != "0":
                segs[-1]["t_seg_end"] = t
            if s != "0":
                segs.append({"seq": int(s), "t_seg_start": t, "t_seg_end": None})
            prev = s
    if segs and segs[-1]["t_seg_end"] is None:
        segs[-1]["t_seg_end"] = d["dg_t"][-1]
    return segs


MOTION_MIN_RUN_S = 0.15   # real moves sustain |vel|>=0.1 for >=0.28 s; noise spikes are 1-2 samples
MOTION_MERGE_GAP_S = 0.3  # merge motion runs separated by less than this
MOTION_MIN_POS_REV = 0.01 # a qualifying run must displace some leg by at least this


def motion_runs(rt, speed_f, rp, i0, i1):
    """Merged, displacement-qualified motion runs within index range [i0,i1)."""
    moving = np.any(speed_f[i0:i1] >= VEL_STATIC_THRESH, axis=1)
    raw = []
    start = None
    for j, q in enumerate(list(moving) + [False]):
        if q and start is None:
            start = j
        elif not q and start is not None:
            raw.append([i0 + start, i0 + j])
            start = None
    # merge nearby runs
    merged = []
    for r in raw:
        if merged and rt[r[0]] - rt[merged[-1][1] - 1] < MOTION_MERGE_GAP_S:
            merged[-1][1] = r[1]
        else:
            merged.append(r)
    # qualify: duration + displacement
    out = []
    for a, b in merged:
        dur = rt[b - 1] - rt[a]
        disp = np.abs(rp[min(b, len(rt) - 1)] - rp[a]).max()
        if dur >= MOTION_MIN_RUN_S and disp >= MOTION_MIN_POS_REV:
            out.append((a, b))
    return out


def moves_list(d):
    """One entry per move_seq: motion window from velocities + realized peaks."""
    rt, rv, rp = d["rs_t"], d["rs_vel"], d["rs_pos"]
    dg_t = d["dg_t_arr"]
    moves = []
    for seg in move_seq_segments(d):
        t0s, t1s = seg["t_seg_start"], seg["t_seg_end"]
        i0, i1 = np.searchsorted(rt, t0s), np.searchsorted(rt, t1s)
        # realized peaks: diagnostics inside the seq window
        m = (dg_t >= t0s) & (dg_t < t1s)
        kvs = [d["dg_kv"][i] for i in np.where(m)[0]]
        def maxk(key):
            vals = []
            for kv in kvs:
                try:
                    v = float(kv.get(key, "nan"))
                    if not math.isnan(v):
                        vals.append(v)
                except ValueError:
                    pass
            return max(vals) if vals else None
        runs = motion_runs(rt, d["rs_speed_f"], rp, i0, i1)
        base = {
            "seq": seg["seq"],
            "t_seg_start": float(t0s), "t_seg_end": float(t1s),
            "n_motion_runs": len(runs),
            "peak_leg_vel_mmps": maxk("peak_leg_vel_mmps"),
            "peak_leg_acc_mmps2": maxk("peak_leg_acc_mmps2"),
            "realized_peak_leg_vel_mmps": maxk("realized_peak_leg_vel_mmps"),
            "realized_peak_leg_acc_mmps2": maxk("realized_peak_leg_acc_mmps2"),
            "realized_peak_leg_jerk_mmps3": maxk("realized_peak_leg_jerk_mmps3"),
        }
        if not runs:
            base["no_motion"] = True
            moves.append(base)
            continue
        a, b = runs[0]  # first qualifying run = THE commanded move
        t_m0, t_m1 = float(rt[a]), float(rt[b - 1])
        target = None
        if len(d["se_t"]):
            k = np.searchsorted(d["se_t"], t_m1 + 0.2) - 1
            if 0 <= k < len(d["se_data"]):
                target = [round(x, 4) for x in d["se_data"][k]]
        pos_delta = (rp[min(b, len(rt) - 1)] - rp[a]).tolist()
        base.update({
            "t_start": t_m0, "t_end": t_m1, "dur_s": t_m1 - t_m0,
            "no_motion": False,
            "pos_delta_rev": [round(x, 4) for x in pos_delta],
            "leg_setpoints_at_end_rev": target,
        })
        moves.append(base)
    return moves


def dev_hold_floor(dev_t, dev, wins):
    """Per-leg hold-noise floor: (median, p95) of |live_deviation| over static windows."""
    mask = np.zeros(len(dev_t), dtype=bool)
    for w in wins:
        mask |= (dev_t >= w["t_start"]) & (dev_t <= w["t_end"])
    if not mask.any():
        return [float("nan")] * N_LEGS, [float("nan")] * N_LEGS
    a = np.abs(dev[mask])
    return np.median(a, axis=0).tolist(), np.percentile(a, 95, axis=0).tolist()


def settle_times(pt, pd, t1, thresholds):
    """Per-threshold, per-leg time after t1 for |dev| to stay < thr for SETTLE_HOLD_S."""
    out = {}
    for name, thr_per_leg in thresholds.items():
        settle = []
        for leg in range(N_LEGS):
            thr = thr_per_leg[leg] if hasattr(thr_per_leg, "__len__") else thr_per_leg
            s = float("nan")
            below = np.abs(pd[:, leg]) < thr
            for j in range(len(pt)):
                if below[j]:
                    k = np.searchsorted(pt, pt[j] + SETTLE_HOLD_S)
                    if k > j and below[j:k].all():
                        s = float(pt[j] - t1)
                        break
            settle.append(s)
        out[name] = settle
    return out


def move_deviation_metrics(dev_t, dev, iq_t, iq, move, floor):
    t0, t1 = move["t_start"], move["t_end"]
    t_lim = move["t_seg_end"]
    out = {}
    m = (dev_t >= t0) & (dev_t <= t1)
    if m.any():
        out["dev_peak_abs"] = np.abs(dev[m]).max(axis=0).tolist()
        out["dev_mean_abs"] = np.abs(dev[m]).mean(axis=0).tolist()
    else:
        out["dev_peak_abs"] = out["dev_mean_abs"] = [float("nan")] * N_LEGS

    # settle time after arrival (capped at next move start), three thresholds:
    # 1.5x hold floor (as specified; floor is at dev quantization so this is
    # the strictest), plus absolute 0.0005 rev and 0.001 rev for robustness.
    post = (dev_t >= t1) & (dev_t <= min(t1 + 10.0, t_lim))
    pt, pd = dev_t[post], dev[post]
    st = settle_times(pt, pd, t1, {
        "floor1p5": [SETTLE_FACTOR * f for f in floor],
        "abs0p0005": 0.0005,
        "abs0p001": 0.001,
    })
    out["dev_settle_s"] = st["floor1p5"]
    out["dev_settle_0p0005_s"] = st["abs0p0005"]
    out["dev_settle_0p001_s"] = st["abs0p001"]
    out["settle_window_limit_s"] = float(min(t1 + 10.0, t_lim) - t1)

    m2 = (dev_t >= t1) & (dev_t <= min(t1 + POST_MOVE_INT_S, t_lim))
    if m2.sum() >= 2:
        out["dev_int_post2s"] = [
            float(np.trapz(np.abs(dev[m2][:, leg]), dev_t[m2])) for leg in range(N_LEGS)
        ]
    else:
        out["dev_int_post2s"] = [float("nan")] * N_LEGS

    mi = (iq_t >= t0) & (iq_t <= t1)
    if mi.any():
        out["iq_peak_abs"] = np.abs(iq[mi]).max(axis=0).tolist()
        out["iq_mean"] = iq[mi].mean(axis=0).tolist()
    else:
        out["iq_peak_abs"] = out["iq_mean"] = [float("nan")] * N_LEGS
    return out


def fmt6(v, w=8, p=4):
    return " ".join(f"{x:{w}.{p}f}" for x in v)


def analyze(tag, d):
    print(f"\n{'='*100}\nBAG {tag}  ({FF_LABEL[tag]})  {d['path']}")
    print(f"duration {d['duration_s']:.1f} s, t0_ns={d['t0_ns']}")

    print("\n-- topic inventory (nonzero) --")
    for name, v in sorted(d["topic_counts"].items(), key=lambda x: -x[1]["count"]):
        if v["count"]:
            print(f"  {v['count']:7d}  {name}  ({v['type']})")
    zero = [n for n, v in d["topic_counts"].items() if v["count"] == 0]
    print(f"  zero-count topics: {', '.join(sorted(zero))}")

    res = {"label": FF_LABEL[tag], "duration_s": d["duration_s"],
           "topic_counts": {k: v["count"] for k, v in d["topic_counts"].items()}}

    print("\n-- /orchestrator_state transitions --")
    for t, s in d["orch"]:
        print(f"  {t:8.2f} s  {s}")
    print("-- /control_mode_topic transitions --")
    for t, s in d["cmode"]:
        print(f"  {t:8.2f} s  {s!r}")
    print("-- /orchestrator_command --")
    for t, s in d["orch_cmd"]:
        print(f"  {t:8.2f} s  {s!r}")
    res["orchestrator_transitions"] = [[round(t, 2), s] for t, s in d["orch"]]
    res["control_mode_transitions"] = [[round(t, 2), s] for t, s in d["cmode"]]
    res["orchestrator_commands"] = [[round(t, 2), s] for t, s in d["orch_cmd"]]

    arm_t = find_arm_time(d)
    print(f"\n-- arm instant (mpc_active 0->1): t = {arm_t:.3f} s")
    res["arm_time_s"] = arm_t

    for key, cast in [("mpc_active", int), ("torque_ff_enabled", int),
                      ("setpoints_without_ff", int), ("torque_clamp_mask", int),
                      ("lead_clamp_mask", int), ("fault_state", str)]:
        ts, vs = link_status_series(d, key, cast)
        tr = transitions(ts, vs)
        print(f"-- /link_status {key} transitions --")
        for t, old, new in tr:
            print(f"  {t:8.2f} s  {old} -> {new}")
        res[f"ls_{key}_transitions"] = [[round(float(t), 2), str(old), str(new)] for t, old, new in tr]

    for key in ("setpoints_sent", "setpoints_rejected", "setpoints_without_ff",
                "crc_errors", "decode_errors"):
        ts, vs = link_status_series(d, key, int)
        final = int(vs[-1]) if len(vs) else None
        mx = int(vs.max()) if len(vs) else None
        print(f"-- /link_status {key}: final={final} max={mx}")
        res[f"ls_{key}_final"] = final
        res[f"ls_{key}_max"] = mx

    # torque_ff_ramp trajectory
    ts_r, vs_r = link_status_series(d, "torque_ff_ramp", float)
    res["ff_ramp"] = None
    if len(vs_r):
        nz = np.where(vs_r > 0.0)[0]
        full = np.where(vs_r >= 0.999)[0]
        print(f"-- torque_ff_ramp: max={vs_r.max():.3f}")
        if len(nz):
            t_first_nz = float(ts_r[nz[0]])
            t_full = float(ts_r[full[0]]) if len(full) else None
            print(f"   first >0 at t={t_first_nz:.2f} s"
                  + (f", reaches 1.000 at t={t_full:.2f} s (climb {t_full - t_first_nz:.2f} s)"
                     if t_full else ", NEVER reaches 1.0"))
            if arm_t is not None:
                print(f"   relative to arm: starts {t_first_nz - arm_t:+.2f} s"
                      + (f", full at {t_full - arm_t:+.2f} s" if t_full else ""))
            w = (ts_r >= t_first_nz - 0.3) & (ts_r <= (t_full if t_full else ts_r[-1]) + 0.3)
            print("   ramp samples (t,val): "
                  + " ".join(f"({t:.2f},{v:.3f})" for t, v in zip(ts_r[w], vs_r[w])))
            res["ff_ramp"] = {"t_first_nonzero_s": t_first_nz, "t_full_s": t_full,
                              "climb_s": (t_full - t_first_nz) if t_full else None,
                              "max": float(vs_r.max())}
        else:
            print("   ramp stayed 0 the whole bag")
            res["ff_ramp"] = {"t_first_nonzero_s": None, "t_full_s": None,
                              "climb_s": None, "max": float(vs_r.max())}

    if res["ff_ramp"] and res["ff_ramp"]["t_full_s"] is not None:
        ts_w, vs_w = link_status_series(d, "setpoints_without_ff", int)
        after = vs_w[ts_w >= res["ff_ramp"]["t_full_s"]]
        print(f"   setpoints_without_ff after full ramp: min={after.min()} max={after.max()}")
        res["setpoints_without_ff_after_full_ramp_max"] = int(after.max())

    print(f"\n-- /leg_torques_diagnostic: {len(d['ltd_t'])} messages")
    if d["ltd_t"]:
        arr = np.asarray(d["ltd_data"])
        print(f"   per-leg min: {fmt6(arr.min(axis=0))}")
        print(f"   per-leg max: {fmt6(arr.max(axis=0))}")
        res["leg_torques_diagnostic"] = {"count": len(d["ltd_t"]),
                                         "min": arr.min(axis=0).tolist(),
                                         "max": arr.max(axis=0).tolist()}
    else:
        res["leg_torques_diagnostic"] = {"count": 0}

    iqm, iqs = d["rs_iqm"], d["rs_iqs"]
    print(f"\n-- /robot_state current fields: BOTH present. iq_measured nonzero frac="
          f"{(np.abs(iqm) > 1e-6).mean():.3f}, iq_setpoint nonzero frac={(np.abs(iqs) > 1e-6).mean():.3f}")
    print(f"   iq_measured range [{iqm.min():.3f}, {iqm.max():.3f}] A; "
          f"iq_setpoint range [{iqs.min():.3f}, {iqs.max():.3f}] A")
    res["iq_fields"] = {"iq_measured_nonzero_frac": float((np.abs(iqm) > 1e-6).mean()),
                        "iq_setpoint_nonzero_frac": float((np.abs(iqs) > 1e-6).mean())}

    # --- A. arm-edge check ---
    # The activation lift (trap-traj) finishes right at the arm; start the
    # window at lift quiescence so we measure FF-kick, not the lift itself.
    rt, rp = d["rs_t"], d["rs_pos"]
    t_quiet = lift_quiescence_time(d, arm_t - 1.0)
    print(f"\n-- A. arm-edge check: arm={arm_t:.2f} s, lift quiescence at {t_quiet:.2f} s "
          f"({t_quiet - arm_t:+.2f} s vs arm)")
    w0, w1 = max(arm_t - 1.0, t_quiet), arm_t + 3.0
    m = (rt >= w0) & (rt <= w1)
    p2p = rp[m].max(axis=0) - rp[m].min(axis=0)
    print(f"   window [{w0:.2f}, {w1:.2f}] s: per-leg pos p2p (rev): {fmt6(p2p, 10, 6)}")
    print(f"   max p2p = {p2p.max():.6f} rev -> {'PASS (<=0.002: encoder noise, no kick)' if p2p.max() <= 0.002 else 'FAIL (>0.002 rev)'}")
    res["arm_edge"] = {"arm_t_s": arm_t, "lift_quiescence_s": t_quiet,
                       "window": [w0, w1], "pos_p2p_rev": p2p.tolist(),
                       "pass": bool(p2p.max() <= 0.002)}

    # For B: explicitly check the ramp window too
    if res["ff_ramp"] and res["ff_ramp"]["t_full_s"] is not None:
        r0, r1 = res["ff_ramp"]["t_first_nonzero_s"], res["ff_ramp"]["t_full_s"] + 0.5
        m = (rt >= r0) & (rt <= r1)
        p2p_r = rp[m].max(axis=0) - rp[m].min(axis=0)
        vmax_r = np.abs(d["rs_vel"][m]).max(axis=0)
        print(f"   FF-ramp window [{r0:.2f}, {r1:.2f}] s: pos p2p (rev): {fmt6(p2p_r, 10, 6)}")
        print(f"   FF-ramp window max |vel| (rev/s): {fmt6(vmax_r, 10, 4)}")
        print(f"   max p2p during ramp = {p2p_r.max():.6f} rev -> "
              f"{'PASS (no kick during ramp)' if p2p_r.max() <= 0.002 else 'FAIL'}")
        res["ramp_kick_check"] = {"window": [r0, r1], "pos_p2p_rev": p2p_r.tolist(),
                                  "vel_absmax_revps": vmax_r.tolist(),
                                  "pass": bool(p2p_r.max() <= 0.002)}

    # --- plan segments (coarse) ---
    segs = []
    if d["st_t"]:
        cur_kind, cur_start = d["st_plan"][0], d["st_t"][0]
        for t, k in zip(d["st_t"][1:], d["st_plan"][1:]):
            if k != cur_kind:
                segs.append({"kind": cur_kind, "t_start": round(cur_start, 2), "t_end": round(t, 2)})
                cur_kind, cur_start = k, t
        segs.append({"kind": cur_kind, "t_start": round(cur_start, 2), "t_end": round(d["st_t"][-1], 2)})
    print("\n-- /trajectory/status plan_kind segments --")
    for s in segs:
        print(f"  {s['t_start']:8.2f} -> {s['t_end']:8.2f} s  {s['kind']}")
    res["plan_segments"] = segs
    rejections = sorted({r for r in d["st_rej"] if r})
    print(f"-- /trajectory/status last_rejection values: {rejections or 'none'}")
    res["trajectory_rejections"] = rejections

    # --- B. static windows ---
    wins = static_windows(d, arm_t + STATIC_MIN_AFTER_ARM_S)
    print(f"\n-- B. static windows (all |vel|<{VEL_STATIC_THRESH} rev/s, >={HOLD_MIN_S:.0f} s, "
          f"t>=arm+{STATIC_MIN_AFTER_ARM_S:.2f}) --")
    for i, w in enumerate(wins):
        print(f"  H{i}: {w['t_start']:.2f}->{w['t_end']:.2f} s ({w['dur_s']:.1f} s, n={w['n_samples']}, plan={w['plan_kind']})")
        print(f"      pos_mean (rev):  {fmt6(w['pos_mean'])}")
        print(f"      iq_meas mean(A): {fmt6(w['iqm_mean'])}")
        print(f"      iq_meas std (A): {fmt6(w['iqm_std'])}")
        print(f"      iq_setp mean(A): {fmt6(w['iqs_mean'])}")
        print(f"      iq_setp std (A): {fmt6(w['iqs_std'])}")
    res["static_windows"] = wins

    # --- deviation floor + moves ---
    dev_t, dev = live_deviation_matrix(d)
    floor_med, floor_p95 = dev_hold_floor(dev_t, dev, wins)
    print(f"\n-- per-leg hold-noise floor |live_deviation| over static windows (rev): "
          f"median {fmt6(floor_med, 7, 4)} | p95 {fmt6(floor_p95, 7, 4)}")
    res["dev_hold_floor_median_rev"] = floor_med
    res["dev_hold_floor_p95_rev"] = floor_p95
    res["live_deviation_units_note"] = ("motor rev; saturates at 0.10 = MAX_LEAD lead-clamp ceiling")

    mv = moves_list(d)
    print(f"\n-- C. moves ({len(mv)}) --")
    for m_ in mv:
        if m_.get("no_motion"):
            print(f"  seq {m_['seq']}: NO MOTION in window {m_['t_seg_start']:.2f}-{m_['t_seg_end']:.2f} s "
                  f"(commanded peak_vel={m_['peak_leg_vel_mmps']}, realized={m_['realized_peak_leg_vel_mmps']} mm/s)")
            continue
        met = move_deviation_metrics(dev_t, dev, d["rs_t"], d["rs_iqm"], m_, floor_med)
        m_.update(met)
        print(f"  seq {m_['seq']:2d}: motion {m_['t_start']:.2f}->{m_['t_end']:.2f} s "
              f"(dur {m_['dur_s']:.2f} s; seg to {m_['t_seg_end']:.2f}; runs={m_['n_motion_runs']})")
        print(f"      pos delta (rev): {fmt6(m_['pos_delta_rev'])}")
        print(f"      realized peak vel/acc/jerk: {m_['realized_peak_leg_vel_mmps']} mm/s, "
              f"{m_['realized_peak_leg_acc_mmps2']} mm/s^2, {m_['realized_peak_leg_jerk_mmps3']} mm/s^3")
        print(f"      dev peak|.| (rev): {fmt6(m_['dev_peak_abs'])}")
        print(f"      dev mean|.| (rev): {fmt6(m_['dev_mean_abs'])}")
        print(f"      dev settle 1.5xfloor (s): {fmt6(m_['dev_settle_s'], 8, 2)}  [window limit {m_['settle_window_limit_s']:.1f} s]")
        print(f"      dev settle <0.0005rev(s): {fmt6(m_['dev_settle_0p0005_s'], 8, 2)}")
        print(f"      dev settle <0.001 rev(s): {fmt6(m_['dev_settle_0p001_s'], 8, 2)}")
        print(f"      D. int|dev| post-{POST_MOVE_INT_S:.0f}s (rev*s): {fmt6(m_['dev_int_post2s'])}")
        print(f"      iq peak|.| (A): {fmt6(m_['iq_peak_abs'])}")
        print(f"      iq mean    (A): {fmt6(m_['iq_mean'])}")
    res["moves"] = mv

    # --- E. anomalies ---
    anoms = []
    ts, vs = link_status_series(d, "setpoints_rejected", int)
    if len(vs) and vs.max() > 0:
        anoms.append(f"setpoints_rejected reached {int(vs.max())}")
    ts, vs = link_status_series(d, "torque_clamp_mask", int)
    if len(vs) and vs.max() > 0:
        anoms.append(f"torque_clamp_mask nonzero in {int((vs > 0).sum())} samples (max {int(vs.max())})")
    ts_f, vs_f = link_status_series(d, "fault_state", str)
    faults = sorted({v for v in vs_f if v not in ("NONE",)})
    if faults:
        anoms.append(f"fault_state values seen: {faults}")
    ts_l, vs_l = link_status_series(d, "lead_clamp_mask", int)
    lead_tr = transitions(ts_l, vs_l)
    post_arm_lead = [(round(float(t), 2), int(n)) for t, _, n in lead_tr
                     if arm_t is not None and t > arm_t + 0.5 and int(n) != 0]
    if post_arm_lead:
        anoms.append(f"lead_clamp engagements after arm (t, mask): {post_arm_lead}")
    if rejections:
        anoms.append(f"trajectory rejections: {rejections}")
    esc = max((float(kv.get("escalation_stop", 0)) for kv in d["dg_kv"]), default=0)
    crj = max((float(kv.get("consecutive_rejects", 0)) for kv in d["dg_kv"]), default=0)
    if esc > 0:
        anoms.append(f"escalation_stop max={esc}")
    if crj > 0:
        anoms.append(f"consecutive_rejects max={crj}")
    print("\n-- E. anomalies --")
    for a in anoms or ["(none)"]:
        print("  ", a)
    res["anomalies"] = anoms
    return res


def match_windows(ra, rb):
    """Match static windows A<->B by per-leg mean pos, in time order."""
    pairs = []
    used_b = set()
    for i, wa in enumerate(ra["static_windows"]):
        for j, wb in enumerate(rb["static_windows"]):
            if j in used_b:
                continue
            dp = np.abs(np.asarray(wa["pos_mean"]) - np.asarray(wb["pos_mean"]))
            if dp.max() < POSE_MATCH_TOL_REV:
                pairs.append((i, j, float(dp.max())))
                used_b.add(j)
                break
    return pairs


def main():
    data = {tag: extract_bag(p) for tag, p in BAGS.items()}
    results = {tag: analyze(tag, data[tag]) for tag in BAGS}

    print(f"\n{'='*100}\nMATCHED A/B COMPARISON\n{'='*100}")

    # --- within-bag repeatability yardstick: split the post-activate hold in half ---
    print("\n-- within-bag iq repeatability yardstick (split-half of first static window) --")
    yard = {}
    for tag in ("A", "B"):
        wins = results[tag]["static_windows"]
        if not wins:
            continue
        w = wins[0]
        dd = data[tag]
        rt = dd["rs_t"]
        tm = 0.5 * (w["t_start"] + w["t_end"])
        m1 = (rt >= w["t_start"]) & (rt < tm)
        m2 = (rt >= tm) & (rt <= w["t_end"])
        drift = dd["rs_iqm"][m2].mean(axis=0) - dd["rs_iqm"][m1].mean(axis=0)
        print(f"  {tag}: half2-half1 iq_measured drift (A): {fmt6(drift)}  (max|.| {np.abs(drift).max():.3f})")
        yard[tag] = drift.tolist()

    # --- matched static windows: sign-inversion check ---
    pairs = match_windows(results["A"], results["B"])
    print(f"\n-- matched static windows (pose within {POSE_MATCH_TOL_REV} rev/leg): "
          f"{[(f'A.H{i}', f'B.H{j}', round(dp,4)) for i,j,dp in pairs]}")
    cmp_holds = []
    for i, j, dpmax in pairs:
        wa = results["A"]["static_windows"][i]
        wb = results["B"]["static_windows"][j]
        da = np.asarray(wb["iqm_mean"]) - np.asarray(wa["iqm_mean"])
        sd = np.sqrt(np.asarray(wa["iqm_std"]) ** 2 + np.asarray(wb["iqm_std"]) ** 2)
        das = np.asarray(wb["iqs_mean"]) - np.asarray(wa["iqs_mean"])
        print(f"\n  A.H{i} ({wa['t_start']:.1f}-{wa['t_end']:.1f}s) vs B.H{j} "
              f"({wb['t_start']:.1f}-{wb['t_end']:.1f}s), max pose delta {dpmax:.4f} rev")
        print(f"    pos A (rev):           {fmt6(wa['pos_mean'])}")
        print(f"    pos B (rev):           {fmt6(wb['pos_mean'])}")
        print(f"    iq_meas A mean (A):    {fmt6(wa['iqm_mean'])}")
        print(f"    iq_meas B mean (A):    {fmt6(wb['iqm_mean'])}")
        print(f"    delta iq_meas B-A (A): {fmt6(da)}")
        print(f"    combined std (A):      {fmt6(sd)}")
        print(f"    delta iq_setp B-A (A): {fmt6(das)}")
        # Abort signature (S3, session_torque_ff.md): FF sign wrong => B sits
        # +0.2..0.7 A ABOVE A on EVERY leg. Anything else (scatter around zero,
        # mostly-negative deltas) is friction/approach-history hysteresis.
        if np.all(da > 0.15) and da.mean() > 0.2:
            verdict = "SIGN-INVERSION SIGNATURE (every leg B >> A)"
        elif not np.all(da > 0.15):
            verdict = ("NO sign-inversion signature (deltas not uniformly positive; "
                       f"leg-mean {da.mean():+.3f} A)")
        else:
            verdict = "MIXED - inspect"
        print(f"    sign-inversion check: leg-mean delta = {da.mean():+.4f} A, "
              f"n legs positive = {(da > 0).sum()}/6, max|delta| = {np.abs(da).max():.4f} A")
        print(f"      (abort signature = +0.2..0.7 A on EVERY leg) -> {verdict}")
        cmp_holds.append({
            "A_win": i, "B_win": j, "max_pose_delta_rev": dpmax,
            "A_range_s": [wa["t_start"], wa["t_end"]],
            "B_range_s": [wb["t_start"], wb["t_end"]],
            "iqm_mean_A": wa["iqm_mean"], "iqm_mean_B": wb["iqm_mean"],
            "delta_iqm_B_minus_A": da.tolist(), "combined_std": sd.tolist(),
            "delta_iqs_B_minus_A": das.tolist(),
            "verdict": verdict,
        })

    # --- matched moves (by move_seq) ---
    ma = {m["seq"]: m for m in results["A"]["moves"] if not m.get("no_motion")}
    mb = {m["seq"]: m for m in results["B"]["moves"] if not m.get("no_motion")}
    common = sorted(set(ma) & set(mb))
    print(f"\n-- matched moves by move_seq: A has {sorted(ma)}, B has {sorted(mb)}; comparing {common} --")
    cmp_moves = []
    agg = {k: [] for k in ["dev_peak_abs", "dev_mean_abs", "dev_settle_s",
                           "dev_settle_0p0005_s", "dev_settle_0p001_s",
                           "dev_int_post2s", "iq_peak_abs", "iq_mean"]}
    for k in common:
        a, b = ma[k], mb[k]
        pd_a = np.asarray(a["pos_delta_rev"])
        pd_b = np.asarray(b["pos_delta_rev"])
        same = np.abs(pd_a - pd_b).max() < 0.05
        print(f"\n  MOVE seq {k}: A {a['t_start']:.1f}s dur {a['dur_s']:.2f}s vs "
              f"B {b['t_start']:.1f}s dur {b['dur_s']:.2f}s | pos-delta match: {same} "
              f"| realized vel {a['realized_peak_leg_vel_mmps']} vs {b['realized_peak_leg_vel_mmps']} mm/s")
        if not same:
            print(f"    EXCLUDED from aggregates: pos deltas differ "
                  f"(A {a['pos_delta_rev']} vs B {b['pos_delta_rev']})")
        rows = {}
        for key, label in [("dev_peak_abs", "peak|dev| rev"), ("dev_mean_abs", "mean|dev| rev"),
                           ("dev_settle_s", "settle1.5f s"), ("dev_settle_0p0005_s", "settle5e-4 s"),
                           ("dev_settle_0p001_s", "settle1e-3 s"), ("dev_int_post2s", "int|dev| rev*s"),
                           ("iq_peak_abs", "peak|iq| A"), ("iq_mean", "mean iq A")]:
            va, vb = np.asarray(a[key], float), np.asarray(b[key], float)
            with np.errstate(invalid="ignore"):
                dd = vb - va
            print(f"    {label:15s} A: {fmt6(va)}")
            print(f"    {label:15s} B: {fmt6(vb)}")
            lm = float(np.nanmean(dd)) if not np.all(np.isnan(dd)) else float("nan")
            print(f"    {label:15s} B-A: {fmt6(dd)}   (leg-mean {lm:+.4f})")
            rows[key] = {"A": va.tolist(), "B": vb.tolist(), "delta": dd.tolist(),
                         "leg_mean_delta": lm}
            if same:
                agg[key].append(lm)
        cmp_moves.append({"seq": k, "pos_delta_match": bool(same),
                          "A_motion": [a["t_start"], a["t_end"]],
                          "B_motion": [b["t_start"], b["t_end"]], "metrics": rows})

    print("\n-- AGGREGATE over pos-delta-MATCHED moves (mean of leg-mean deltas, B-A) --")
    agg_out = {}
    for key, vals in agg.items():
        vals = [v for v in vals if not math.isnan(v)]
        agg_out[key] = float(np.mean(vals)) if vals else None
        print(f"    {key:20s}: {agg_out[key]:+.5f}" if vals else f"    {key:20s}: n/a")

    # paired settle summary with censoring accounting (settle windows are
    # censored at ~3 s by the next move in the battery)
    print("\n-- PAIRED settle-time summary over matched moves x legs (censoring-aware) --")
    settle_summary = {}
    for key in ("dev_settle_s", "dev_settle_0p0005_s", "dev_settle_0p001_s"):
        a_all, b_all = [], []
        n_both = n_a_only = n_b_only = n_neither = 0
        for cm in cmp_moves:
            if not cm["pos_delta_match"]:
                continue
            va = cm["metrics"][key]["A"]
            vb = cm["metrics"][key]["B"]
            for leg in range(N_LEGS):
                a_nan = va[leg] is None or (isinstance(va[leg], float) and math.isnan(va[leg]))
                b_nan = vb[leg] is None or (isinstance(vb[leg], float) and math.isnan(vb[leg]))
                if not a_nan and not b_nan:
                    n_both += 1
                    a_all.append(va[leg]); b_all.append(vb[leg])
                elif not a_nan:
                    n_a_only += 1
                elif not b_nan:
                    n_b_only += 1
                else:
                    n_neither += 1
        med_a = float(np.median(a_all)) if a_all else None
        med_b = float(np.median(b_all)) if b_all else None
        med_d = float(np.median(np.asarray(b_all) - np.asarray(a_all))) if a_all else None
        settle_summary[key] = {
            "n_both_settled": n_both, "n_A_only_settled": n_a_only,
            "n_B_only_settled": n_b_only, "n_neither": n_neither,
            "median_A_s": med_a, "median_B_s": med_b, "median_delta_B_minus_A_s": med_d,
        }
        print(f"    {key:20s}: both={n_both} A-only={n_a_only} B-only={n_b_only} neither={n_neither} "
              + (f"| median A {med_a:.2f} s, B {med_b:.2f} s, paired delta {med_d:+.2f} s"
                 if a_all else ""))

    out = {
        "generated_by": "ff_ab_extract.py",
        "bags": {t: BAGS[t] for t in BAGS},
        "thresholds": {
            "vel_static_thresh_revps": VEL_STATIC_THRESH,
            "hold_min_s": HOLD_MIN_S, "pose_match_tol_rev": POSE_MATCH_TOL_REV,
            "settle_factor": SETTLE_FACTOR, "settle_hold_s": SETTLE_HOLD_S,
            "post_move_integral_s": POST_MOVE_INT_S,
            "quiet_sustain_s": QUIET_SUSTAIN_S,
            "static_min_after_arm_s": STATIC_MIN_AFTER_ARM_S,
        },
        "per_bag": results,
        "within_bag_splithalf_iq_drift_A": yard,
        "matched_holds": cmp_holds,
        "matched_moves": cmp_moves,
        "aggregate_move_deltas_B_minus_A": agg_out,
        "paired_settle_summary": settle_summary,
    }

    def clean(o):
        if isinstance(o, float) and (math.isnan(o) or math.isinf(o)):
            return None
        if isinstance(o, dict):
            return {k: clean(v) for k, v in o.items()}
        if isinstance(o, (list, tuple)):
            return [clean(v) for v in o]
        if isinstance(o, (np.floating, np.integer)):
            return clean(o.item())
        if isinstance(o, np.bool_):
            return bool(o)
        return o

    with open(OUT_JSON, "w") as f:
        json.dump(clean(out), f, indent=1)
    print(f"\nWROTE {OUT_JSON}")


if __name__ == "__main__":
    main()
