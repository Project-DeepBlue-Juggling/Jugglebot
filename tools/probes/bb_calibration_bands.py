#!/usr/bin/env python3
"""Cross-sitting distributions and advisory bands for the BB (Ball Butler) calibration.

Reads a ``bb_calibration_replay.py`` output directory (``sweeps.csv``) and writes
``<replay_dir>/bands.md`` + ``<replay_dir>/bands.json``:

* per-SITTING (per bag) mean / SD / n of every calibration quantity, the cross-sitting
  distribution of the sitting means, and the within-sitting pooled SD;
* world quantities (yaw offset, axis point, base origin) grouped by QTM re-alignment EPOCH
  (an epoch boundary is a base-pose jump > ``EPOCH_JUMP_MM`` or > ``EPOCH_JUMP_DEG`` between
  consecutive base-seen sweeps), reported within-epoch vs across-epoch;
* kappa grouped by the KAPPA epochs of the 2026-10-14 investigation (10-10 / 10-11..10-13
  00:18 / 10-13 22:00 onward), reported within-epoch vs across-epoch;
* the Phase 1 advisory distances replayed in time order (delta-kappa vs the never-reset pool,
  delta-yaw and delta-axis vs the last accepted sweep IN THE SAME WORLD EPOCH), the proposed
  advisory bands with their derivation, and the fraction of intrinsic-good sweeps inside each;
* optionally (``--feeds``) the throw cross-check: each columns launch's calibration in force vs
  that launch's feed landing bias.

Only sweeps whose ``verdict_intrinsic`` is ACCEPTED are used: intrinsic quality, regardless of
the historical verdict - that is the point of the exercise.

Provenance: plans/archived/bb-calibration-one-attempt.md Phase 0 ("Cross-sitting distributions",
"Cross-check against throws", P7); logbook/2026-10-14-bb-calibration-one-attempt-investigation.md.
Pure Python + numpy; no ROS imports. ``summarise(rows) -> dict`` is the test seam.

Usage:
    python tools/probes/bb_calibration_bands.py <replay_dir> [--feeds <u4_feeds.csv>]
"""
from __future__ import annotations

import argparse
import csv
import datetime as _dt
import json
import math
import os
import sys
from collections import OrderedDict
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

# --- grouping rules --------------------------------------------------------------------------
EPOCH_JUMP_MM = 2.0      # base origin jump between consecutive sweeps that starts a new world epoch
EPOCH_JUMP_DEG = 0.05    # base heading jump that starts a new world epoch
# Kappa epochs (investigation addendum): bag-name prefixes, compared lexically.
KAPPA_EPOCHS = (("K0 (10-10)", "2026-10-10", "2026-10-11"),
                ("K1 (10-11..10-13 00:18)", "2026-10-11", "2026-10-13_22"),
                ("K2 (10-13 22:00 on)", "2026-10-13_22", "9999"))
BAG_UTC_OFFSET_H = 11.0  # bag names are local AEDT (UTC+11 from 2026-10-04)

# --- physics used to translate bands into "what a bump looks like" ----------------------------
BUMP_MM = 2.0                # a real knock of BB, at the marker radius
MARKER_RADIUS_MM = 143.0
# ~mean BB marker radius about the yaw axis (2 mm there ~= 0.8 deg)
TEMPLATE_HARD_CAP_MM = 1.5   # shipped value, bb_calibration.py, since 2026-10-17 (entry 2026-10-17-bb-calibration-residual-cap-1p5)
TEMPLATE_ADVISORY_MM = 1.0   # shipped value, bb_calibration.py, since 2026-10-17 (entry 2026-10-17-bb-calibration-residual-cap-1p5)

# Fidelity exclusions (replay fidelity.md): sweeps whose replayed verdict does NOT reproduce the
# node's logged verdict are left out of every band. Key (bag, sweep_idx) -> reason.
EXCLUDE = {
    ("2026-10-11_15-14-11", 1): "verdict mismatch: node logged CONSTELLATION_TOO_FEW_MOVING "
                                "(node-side stall), replay ACCEPTED; no logged measurements",
}

SCALARS = ("kappa_deg", "yaw_offset_deg", "lag_ms", "template_residual_mm", "base_residual_mm",
           "sigma_deg", "sweep_end_deg")
TRIPLES = ("axis_point_mm", "p_b_mm", "base_origin_mm")
WORLD_QUANTITIES = ("yaw_offset_deg", "axis_x", "axis_y", "axis_z", "base_x", "base_y", "base_z",
                    "base_heading_deg")
BASE_QUANTITIES = ("kappa_deg", "p_b_x", "p_b_y", "p_b_z")
INTRINSIC_QUANTITIES = ("lag_ms", "template_residual_mm", "base_residual_mm", "sigma_deg",
                        "sweep_end_deg")


# ============================================================================== parsing
def _f(x: Any) -> Optional[float]:
    if x is None:
        return None
    if isinstance(x, (int, float)):
        return None if (isinstance(x, float) and math.isnan(x)) else float(x)
    s = str(x).strip()
    if s in ("", "None", "nan", "NaN"):
        return None
    try:
        return float(s)
    except ValueError:
        return None


def _triple(x: Any) -> Optional[List[float]]:
    if x is None:
        return None
    if isinstance(x, (list, tuple)):
        vals = [float(v) for v in x]
    else:
        s = str(x).strip().strip("[]()")
        if not s:
            return None
        try:
            vals = [float(v) for v in s.replace(";", ",").split(",")]
        except ValueError:
            return None
    return vals if len(vals) == 3 else None


def parse_row(r: Dict[str, Any]) -> Dict[str, Any]:
    """Normalise one sweeps.csv / sweeps.json row (idempotent)."""
    out = dict(r)
    for k in SCALARS + ("t_start", "base_heading_deg", "pool_n_before", "logged_pool_n"):
        out[k] = _f(r.get(k))
    out["sweep_idx"] = int(float(r.get("sweep_idx", 0) or 0))
    for k in TRIPLES:
        out[k] = _triple(r.get(k))
    for name, key in (("axis", "axis_point_mm"), ("p_b", "p_b_mm"), ("base", "base_origin_mm")):
        t = out[key]
        for i, ax in enumerate("xyz"):
            out["%s_%s" % (name, ax)] = None if t is None else t[i]
    bs = str(r.get("base_seen", "")).strip().lower()
    out["base_seen"] = bs in ("true", "1", "yes") and out["base_origin_mm"] is not None
    return out


def load_rows(replay_dir: str) -> List[Dict[str, Any]]:
    with open(os.path.join(replay_dir, "sweeps.csv"), newline="") as fh:
        return [parse_row(r) for r in csv.DictReader(fh)]


# ============================================================================== grouping
def accepted(rows: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rr = [parse_row(r) for r in rows if str(r.get("verdict_intrinsic", "")).strip() == "ACCEPTED"
          and (str(r.get("bag")), int(float(r.get("sweep_idx", 0) or 0))) not in EXCLUDE]
    rr.sort(key=lambda r: (r["t_start"] or 0.0, r["sweep_idx"]))
    return rr


def kappa_epoch(bag: str) -> str:
    for name, lo, hi in KAPPA_EPOCHS:
        if lo <= bag < hi:
            return name
    return "K?"


def assign_world_epochs(rows: List[Dict[str, Any]]) -> None:
    """Tag each row with ``world_epoch`` (in place). Rows are time-ordered.

    A base-seen sweep opens a new epoch when its base origin moved > EPOCH_JUMP_MM or its
    heading > EPOCH_JUMP_DEG from the previous base-seen sweep. A sweep that did not see the
    base has no world reference: it gets its own epoch ``nobase:<bag>``.
    """
    n = 0
    prev = None
    for r in rows:
        if not r["base_seen"]:
            r["world_epoch"] = "nobase:%s" % r["bag"]
            continue
        if prev is not None:
            d = float(np.linalg.norm(np.subtract(r["base_origin_mm"], prev["base_origin_mm"])))
            dh = abs((r["base_heading_deg"] - prev["base_heading_deg"] + 180.0) % 360.0 - 180.0)
            if d > EPOCH_JUMP_MM or dh > EPOCH_JUMP_DEG:
                n += 1
        r["world_epoch"] = "W%d" % n
        prev = r


def _stats(v: Sequence[float]) -> Dict[str, Any]:
    a = np.asarray([x for x in v if x is not None], dtype=float)
    if a.size == 0:
        return {"n": 0}
    out = {"n": int(a.size), "mean": float(a.mean()), "min": float(a.min()),
           "max": float(a.max())}
    out["sd"] = float(a.std(ddof=1)) if a.size > 1 else None
    return out


def _pooled_sd(groups: Sequence[Sequence[float]]) -> Tuple[Optional[float], int]:
    ss, dof = 0.0, 0
    for g in groups:
        a = np.asarray([x for x in g if x is not None], dtype=float)
        if a.size > 1:
            ss += float(((a - a.mean()) ** 2).sum())
            dof += a.size - 1
    return (math.sqrt(ss / dof) if dof else None), dof


def _distribution(rows: List[Dict[str, Any]], q: str, sitting_key: str) -> Dict[str, Any]:
    groups = OrderedDict()  # type: OrderedDict
    for r in rows:
        if r.get(q) is None:
            continue
        groups.setdefault(r[sitting_key], []).append(r[q])
    per = OrderedDict((k, _stats(v)) for k, v in groups.items())
    means = [s["mean"] for s in per.values()]
    pooled, dof = _pooled_sd(list(groups.values()))
    return {"per_sitting": per, "cross_sitting": _stats(means),
            "within_sitting_pooled_sd": pooled, "within_dof": dof,
            "all_sweeps": _stats([x for g in groups.values() for x in g])}


def _epoch_split(rows: List[Dict[str, Any]], q: str, epoch_key: str,
                 sitting_key: str) -> Dict[str, Any]:
    """Within-epoch vs across-epoch: per epoch the distribution of sitting means; the pooled SD of
    sitting means about their epoch mean; and the distribution of epoch means."""
    by_ep = OrderedDict()  # type: OrderedDict
    for r in rows:
        if r.get(q) is not None:
            by_ep.setdefault(r[epoch_key], []).append(r)
    per_ep = OrderedDict()
    sitting_mean_groups = []
    for ep, rr in by_ep.items():
        d = _distribution(rr, q, sitting_key)
        sm = [s["mean"] for s in d["per_sitting"].values()]
        sitting_mean_groups.append(sm)
        per_ep[ep] = {"sweeps": d["all_sweeps"], "sitting_means": _stats(sm),
                      "within_sitting_pooled_sd": d["within_sitting_pooled_sd"]}
    pooled_sm, dof = _pooled_sd(sitting_mean_groups)
    sweep_pooled, sdof = _pooled_sd([[r[q] for r in rr] for rr in by_ep.values()])
    return {"per_epoch": per_ep,
            "within_epoch_sd_of_sitting_means": pooled_sm, "within_epoch_sitting_dof": dof,
            "within_epoch_sweep_sd": sweep_pooled, "within_epoch_sweep_dof": sdof,
            "across_epoch_means": _stats([v["sweeps"]["mean"] for v in per_ep.values()])}


# ============================================================================== advisories
def replay_advisories(rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """The Phase 1 advisory distances, in time order, over intrinsic-good sweeps only.

    dkappa_pool : kappa - median of every earlier kappa (pool kept, never reset)
    dkappa_epoch: kappa - median of earlier kappa in the same kappa epoch (None for the first)
    dyaw_last   : yaw - yaw of the previous sweep in the same WORLD epoch
    daxis_last  : |axis point - previous axis point| in the same world epoch (3-D, mm)
    dp_b_pool   : |p_b - median of earlier p_b| (BB position in base, mm)
    """
    pool_k, pool_pb = [], []  # type: List[float], List[List[float]]
    by_epoch_k = {}  # type: Dict[str, List[float]]
    last_world = {}  # type: Dict[str, Dict[str, Any]]
    out = []
    for r in rows:
        a = {"bag": r["bag"], "sweep_idx": r["sweep_idx"], "world_epoch": r["world_epoch"],
             "kappa_epoch": r["kappa_epoch"], "dkappa_pool": None, "dkappa_epoch": None,
             "dyaw_last": None, "daxis_last": None, "dp_b_pool": None,
             "first_in_world_epoch": r["world_epoch"] not in last_world}
        k = r.get("kappa_deg")
        if k is not None:
            if pool_k:
                a["dkappa_pool"] = k - float(np.median(pool_k))
            ek = by_epoch_k.setdefault(r["kappa_epoch"], [])
            if ek:
                a["dkappa_epoch"] = k - float(np.median(ek))
            pool_k.append(k)
            ek.append(k)
        if r.get("p_b_mm") is not None:
            if pool_pb:
                a["dp_b_pool"] = float(np.linalg.norm(np.subtract(
                    r["p_b_mm"], np.median(np.asarray(pool_pb), axis=0))))
            pool_pb.append(r["p_b_mm"])
        prev = last_world.get(r["world_epoch"])
        if prev is not None:
            a["dyaw_last"] = r["yaw_offset_deg"] - prev["yaw_offset_deg"]
            a["daxis_last"] = float(np.linalg.norm(np.subtract(r["axis_point_mm"],
                                                               prev["axis_point_mm"])))
        last_world[r["world_epoch"]] = r
        out.append(a)
    return out


def _kappa_vs_sweep_end(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Pooled within-kappa-epoch slope of kappa on sweep end (deg / deg), and the raw slope."""
    xs, ys = [], []
    for ep in {r["kappa_epoch"] for r in rows}:
        rr = [r for r in rows if r["kappa_epoch"] == ep and r.get("kappa_deg") is not None]
        if len(rr) < 3:
            continue
        x = np.array([r["sweep_end_deg"] for r in rr])
        y = np.array([r["kappa_deg"] for r in rr])
        xs.append(x - x.mean())
        ys.append(y - y.mean())
    x, y = np.concatenate(xs), np.concatenate(ys)
    b_within = float((x * y).sum() / (x * x).sum())
    r_within = float(np.corrcoef(x, y)[0, 1])
    allr = [r for r in rows if r.get("kappa_deg") is not None]
    xa = np.array([r["sweep_end_deg"] for r in allr])
    ya = np.array([r["kappa_deg"] for r in allr])
    b_all = float(np.polyfit(xa, ya, 1)[0])
    return {"slope_within_epoch_deg_per_deg": b_within, "r_within_epoch": r_within,
            "slope_pooled_raw_deg_per_deg": b_all, "r_raw": float(np.corrcoef(xa, ya)[0, 1]),
            "n": int(xa.size)}


# ============================================================================== summarise
def summarise(rows: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """Distributions over intrinsic-ACCEPTED rows. ``rows`` = raw or parsed sweeps.csv dicts."""
    rr = accepted(rows)
    assign_world_epochs(rr)
    for r in rr:
        r["kappa_epoch"] = kappa_epoch(r["bag"])
        # a world "sitting" is a bag inside one world epoch (10-10_14-10-50 spans a re-alignment)
        r["world_sitting"] = "%s|%s" % (r["bag"], r["world_epoch"])
    out = OrderedDict()  # type: Dict[str, Any]
    out["n_rows"] = len(list(rows))
    out["n_accepted_intrinsic"] = len(rr)
    out["n_sittings"] = len({r["bag"] for r in rr})
    out["world_epochs"] = OrderedDict()
    for r in rr:
        e = out["world_epochs"].setdefault(r["world_epoch"], {"bags": [], "n": 0,
                                                               "base_heading_deg": None})
        if r["bag"] not in e["bags"]:
            e["bags"].append(r["bag"])
        e["n"] += 1
        if r["base_heading_deg"] is not None:
            e["base_heading_deg"] = r["base_heading_deg"]
    out["kappa_epochs"] = OrderedDict()
    for r in rr:
        if r.get("kappa_deg") is not None:
            out["kappa_epochs"].setdefault(r["kappa_epoch"], []).append(r["bag"])
    for k in out["kappa_epochs"]:
        out["kappa_epochs"][k] = sorted(set(out["kappa_epochs"][k]))
    q = OrderedDict()
    for name in INTRINSIC_QUANTITIES + BASE_QUANTITIES:
        q[name] = _distribution(rr, name, "bag")
    for name in WORLD_QUANTITIES:
        q[name] = _distribution(rr, name, "world_sitting")
        q[name]["epochs"] = _epoch_split(rr, name, "world_epoch", "world_sitting")
    for name in ("kappa_deg", "sweep_end_deg", "lag_ms", "template_residual_mm",
                 "base_residual_mm", "p_b_x", "p_b_y", "p_b_z"):
        q[name]["kappa_epochs"] = _epoch_split(rr, name, "kappa_epoch", "bag")
    out["quantities"] = q
    out["kappa_vs_sweep_end"] = _kappa_vs_sweep_end(rr)
    out["advisories"] = replay_advisories(rr)
    out["_rows"] = rr
    return out


# ============================================================================== bands
def _abs_vals(adv: List[Dict[str, Any]], key: str, only_k_epoch: Optional[bool] = None) -> List[float]:
    return [abs(a[key]) for a in adv if a[key] is not None]


def _coverage(vals: Sequence[float], band: float) -> Tuple[int, int]:
    return sum(1 for v in vals if v <= band + 1e-12), len(vals)


def _ceil_to(x: float, step: float) -> float:
    return math.ceil(x / step - 1e-9) * step


def propose_bands(s: Dict[str, Any]) -> List[Dict[str, Any]]:
    """The band proposal. Rule (one, for every advisory): band = max(3 x the relevant
    cross-sitting SD, 1.25 x the largest observed intrinsic-good excursion), rounded up to a
    readable step. 3 SD so a fresh sitting is inside by construction; the 1.25 x max floor so
    the band covers ~all the recorded intrinsic-good sweeps (owner intent: none of the 23 false
    positives would have fired). Every band is then checked against what a REAL bump looks like.
    """
    q = s["quantities"]
    adv = s["advisories"]
    bands = []
    bump_deg = math.degrees(BUMP_MM / MARKER_RADIUS_MM)

    def band(name, unit, sd, sd_label, vals, step, bump, note, cap=None):
        mx = max(vals) if vals else float("nan")
        raw = max(3.0 * (sd or 0.0), 1.25 * mx)
        b = _ceil_to(raw, step)
        if cap is not None and b > cap:
            note = "%s; rule gave %.2f, clamped at the shipped advisory to %.2f" % (note, b, cap)
            b = cap
        cin, n = _coverage(vals, b)
        bands.append({"advisory": name, "unit": unit, "band": b, "sd": sd, "sd_label": sd_label,
                      "max_observed": mx, "coverage_in": cin, "coverage_n": n,
                      "bump_signature": bump, "bump_over_band": (bump / b if bump else None),
                      "note": note})

    k = q["kappa_deg"]
    ke = k["kappa_epochs"]
    # kappa vs pool: two readings - (a) the step is COVERED (pool spans epochs), (b) the step is
    # an EPOCH (compare within kappa epoch only).
    band("dkappa vs pool (step covered)", "deg", k["cross_sitting"]["sd"],
         "cross-sitting SD of sitting means, all epochs",
         _abs_vals(adv, "dkappa_pool"), 0.05, bump_deg,
         "pool = every earlier intrinsic-good kappa, never reset; median reference")
    band("dkappa vs epoch pool (step = epoch)", "deg", ke["within_epoch_sd_of_sitting_means"],
         "within-kappa-epoch SD of sitting means",
         _abs_vals(adv, "dkappa_epoch"), 0.05, bump_deg,
         "reference = median of earlier kappa in the same kappa epoch")
    yw = q["yaw_offset_deg"]["epochs"]
    band("dyaw vs last accepted (same world epoch)", "deg",
         yw["within_epoch_sd_of_sitting_means"],
         "within-world-epoch SD of sitting means",
         _abs_vals(adv, "dyaw_last"), 0.05, bump_deg,
         "first sweep after a QTM re-alignment has no reference (world epoch opens)")
    ax_sd = math.sqrt(sum((q[c]["epochs"]["within_epoch_sd_of_sitting_means"] or 0.0) ** 2
                          for c in ("axis_x", "axis_y", "axis_z")))
    band("daxis point vs last accepted (same world epoch)", "mm", ax_sd,
         "RSS of within-world-epoch SD of sitting means (x,y,z)",
         _abs_vals(adv, "daxis_last"), 0.25, BUMP_MM,
         "a 2 mm translation of BB moves the axis point 2 mm")
    pb_sd = math.sqrt(sum((q[c]["cross_sitting"]["sd"] or 0.0) ** 2
                          for c in ("p_b_x", "p_b_y", "p_b_z")))
    band("dp_b (BB in base) vs pool", "mm", pb_sd,
         "RSS of cross-sitting SD of sitting means (x,y,z)",
         _abs_vals(adv, "dp_b_pool"), 0.25, BUMP_MM,
         "frame-free position check; survives QTM re-alignment")
    tr = q["template_residual_mm"]
    band("template residual (advisory level; 1.5 mm hard cap)", "mm",
         tr["cross_sitting"]["sd"], "cross-sitting SD of sitting means",
         [r["template_residual_mm"] for r in s["_rows"]], 0.05, None,
         "absolute level, not a delta; band = advisory threshold",
         cap=TEMPLATE_ADVISORY_MM)
    lg = q["lag_ms"]
    lag_abs = [abs(r["lag_ms"]) for r in s["_rows"] if r["lag_ms"] is not None]
    band("|lag| (yaw source vs mocap)", "ms", lg["cross_sitting"]["sd"],
         "cross-sitting SD of sitting means", lag_abs, 1.0, None,
         "absolute level; LAG_AT_EDGE stays the hard check")
    return bands


# ============================================================================== throw cross-check
def _bag_start_unix(bag: str) -> float:
    t = _dt.datetime.strptime(bag[:19], "%Y-%m-%d_%H-%M-%S")
    return (t - _dt.datetime(1970, 1, 1)).total_seconds() - BAG_UTC_OFFSET_H * 3600.0


def throw_crosscheck(s: Dict[str, Any], all_rows: Sequence[Dict[str, Any]],
                     feeds_csv: str) -> Dict[str, Any]:
    """Per columns launch: the calibration in force (the node's LAST logged ACCEPTED sweep in the
    launch's bag) vs the feed landing bias (median miss_dx/miss_dy and land-minus-announced).

    Predicted landing shift of a calibration ERROR, at range R from the axis point:
      lateral (perpendicular to the throw) = R * yaw error;  translation = axis-point error.
    The error proxy is the in-force value minus the launch's own intrinsic mean (what the same
    launch's other sweeps say); between launches, the in-force kappa difference (frame-free).
    """
    parsed = [parse_row(r) for r in all_rows]
    bags = sorted({r["bag"] for r in parsed})
    starts = [(_bag_start_unix(b), b) for b in bags]
    with open(feeds_csv, newline="") as fh:
        feeds = list(csv.DictReader(fh))
    runs = OrderedDict()  # type: OrderedDict
    for f in feeds:
        td = _f(f.get("td"))
        if td is None:
            continue
        bag = None
        for t0, b in starts:
            if t0 <= td:
                bag = b
        runs.setdefault(f["run"], {"bag": set(), "feeds": []})
        runs[f["run"]]["bag"].add(bag)
        runs[f["run"]]["feeds"].append(f)
    acc_by_bag = {}  # type: Dict[str, List[Dict[str, Any]]]
    for r in s["_rows"]:
        acc_by_bag.setdefault(r["bag"], []).append(r)
    out = []
    for run, d in runs.items():
        bag = sorted(b for b in d["bag"] if b)[-1]
        logged = [r for r in parsed if r["bag"] == bag
                  and str(r.get("logged_outcome", "")).strip() == "ACCEPTED"]
        inforce = max(logged, key=lambda r: r["t_start"]) if logged else None
        own = acc_by_bag.get(bag, [])

        def med(c):
            v = [_f(f.get(c)) for f in d["feeds"]]
            v = [x for x in v if x is not None]
            return float(np.median(v)) if v else None
        ann = np.array([med("ann_land_x"), med("ann_land_y")])
        row = {"run": run, "bag": bag, "n_feeds": len(d["feeds"]),
               "miss_dx": med("miss_dx"), "miss_dy": med("miss_dy"),
               "land_minus_ann_x": med("land_minus_ann_x"),
               "land_minus_ann_y": med("land_minus_ann_y"),
               "in_force": None}
        if inforce is not None:
            ax = np.array(inforce["axis_point_mm"][:2])
            u = ann - ax
            rng = float(np.linalg.norm(u))
            u = u / rng
            v = np.array([-u[1], u[0]])
            mean_yaw = float(np.mean([r["yaw_offset_deg"] for r in own]))
            mean_ax = np.mean([r["axis_point_mm"] for r in own], axis=0)
            e_yaw = inforce["yaw_offset_deg"] - mean_yaw
            e_ax = np.asarray(inforce["axis_point_mm"]) - mean_ax
            miss = np.array([row["miss_dx"], row["miss_dy"]])
            row["in_force"] = {
                "sweep_idx": inforce["sweep_idx"], "yaw_offset_deg": inforce["yaw_offset_deg"],
                "kappa_deg": inforce["kappa_deg"], "axis_point_mm": inforce["axis_point_mm"],
                "pose_source": inforce.get("pose_source"),
                "pool_n": (inforce.get("logged_pool_n") if inforce.get("logged_pool_n")
                           is not None else inforce.get("pool_n_before")), "sweep_end_deg": inforce["sweep_end_deg"]}
            row.update({
                "range_mm": rng, "throw_dir_deg": math.degrees(math.atan2(u[1], u[0])),
                "n_own_intrinsic": len(own),
                "yaw_err_deg": e_yaw, "pred_lateral_from_yaw_mm": rng * math.radians(e_yaw),
                "axis_err_mm": float(np.linalg.norm(e_ax)),
                "pred_range_from_axis_mm": float(-e_ax[:2] @ u),
                "pred_lateral_from_axis_mm": float(-e_ax[:2] @ v),
                "bias_range_mm": float(miss @ u), "bias_lateral_mm": float(miss @ v)})
        out.append(row)
    # between-launch: frame-free kappa differences vs bias differences
    pairs = []
    for i in range(len(out)):
        for j in range(i + 1, len(out)):
            a, b = out[i], out[j]
            if a["in_force"] is None or b["in_force"] is None:
                continue
            dk = b["in_force"]["kappa_deg"] - a["in_force"]["kappa_deg"]
            rng = 0.5 * (a["range_mm"] + b["range_mm"])
            pairs.append({"a": a["run"], "b": b["run"], "dkappa_deg": dk,
                          "pred_lateral_mm": rng * math.radians(dk),
                          "dbias_range_mm": b["bias_range_mm"] - a["bias_range_mm"],
                          "dbias_lateral_mm": b["bias_lateral_mm"] - a["bias_lateral_mm"]})
    return {"launches": out, "pairs": pairs}


# ============================================================================== P7
def p7_prediction(s: Dict[str, Any]) -> Dict[str, Any]:
    q = s["quantities"]["kappa_deg"]
    sw = q["kappa_epochs"]["within_epoch_sweep_sd"]       # sweep-to-sweep, within kappa epochs
    sm = q["kappa_epochs"]["within_epoch_sd_of_sitting_means"]  # sitting-to-sitting, same epoch
    within = q["within_sitting_pooled_sd"]
    se3v3 = within * math.sqrt(2.0 / 3.0)
    return {"within_sitting_sd": within, "within_epoch_sweep_sd": sw,
            "within_epoch_sitting_mean_sd": sm,
            "se_diff_3v3_same_sitting": se3v3,
            "detectable_3v3_same_sitting_deg": 3.0 * se3v3,
            "detectable_3v3_across_sittings_deg": 3.0 * math.sqrt(se3v3 ** 2 + (sm or 0.0) ** 2),
            "detectable_single_sweep_vs_single_sweep_deg": 3.0 * math.sqrt(2.0) * within,
            "slope": s["kappa_vs_sweep_end"]}


# ============================================================================== report
def _fmt(x: Any, nd: int = 3) -> str:
    if x is None:
        return "-"
    if isinstance(x, float):
        return ("%." + str(nd) + "f") % x
    return str(x)


def render_md(s: Dict[str, Any], bands: List[Dict[str, Any]], p7: Dict[str, Any],
              cc: Optional[Dict[str, Any]], replay_dir: str, extra_md: str = "") -> str:
    L = []
    q = s["quantities"]
    L.append("# BB calibration: cross-sitting distributions and advisory bands\n")
    L.append("Replay dir `%s`; %d rows, **%d intrinsic-ACCEPTED sweeps over %d sittings (bags)**. "
             "Generated by `tools/probes/bb_calibration_bands.py`.\n"
             % (replay_dir, s["n_rows"], s["n_accepted_intrinsic"], s["n_sittings"]))
    L.append("Excluded for fidelity: " + "; ".join("%s #%d (%s)" % (k[0], k[1], v)
                                                  for k, v in EXCLUDE.items()) + "\n")
    L.append("## World epochs (QTM re-alignments: base jump > %.1f mm or > %.2f deg)\n"
             % (EPOCH_JUMP_MM, EPOCH_JUMP_DEG))
    L.append("| epoch | n | base heading (deg) | bags |\n|---|---|---|---|")
    for e, d in s["world_epochs"].items():
        L.append("| %s | %d | %s | %s |" % (e, d["n"], _fmt(d["base_heading_deg"], 3),
                                          ", ".join(d["bags"])))
    L.append("\nKappa epochs: " + "; ".join("%s: %d bags" % (k, len(v))
                                           for k, v in s["kappa_epochs"].items()) + "\n")
    L.append("## Distributions\n")
    L.append("Per quantity: cross-sitting = distribution of the per-sitting MEANS; within = pooled "
             "within-sitting SD. World quantities: sittings are bag x world epoch.\n")
    L.append("| quantity | sittings | cross mean | cross SD | cross min | cross max | within SD "
             "(dof) | within-epoch SD of sitting means | across-epoch SD of epoch means |")
    L.append("|---|---|---|---|---|---|---|---|---|")
    for name, d in q.items():
        cs = d["cross_sitting"]
        ep = d.get("epochs") or d.get("kappa_epochs")
        L.append("| %s | %d | %s | %s | %s | %s | %s (%d) | %s | %s |" % (
            name, cs["n"], _fmt(cs.get("mean")), _fmt(cs.get("sd")), _fmt(cs.get("min")),
            _fmt(cs.get("max")), _fmt(d["within_sitting_pooled_sd"]), d["within_dof"],
            _fmt(ep["within_epoch_sd_of_sitting_means"]) if ep else "-",
            _fmt(ep["across_epoch_means"].get("sd")) if ep else "-"))
    L.append("\n(World quantities use world epochs; kappa, sweep end, lag, residuals and p_b use "
             "kappa epochs.)\n")
    L.append("### Per sitting: kappa, yaw, sweep end, lag, template residual\n")
    L.append("| sitting | n | kappa mean (SD) | yaw mean (SD) | sweep end | lag ms | template mm | "
             "base mm |\n|---|---|---|---|---|---|---|---|")
    for bag, st in q["sweep_end_deg"]["per_sitting"].items():
        def ps(name, key=bag):
            p = q[name]["per_sitting"].get(key)
            return "-" if not p else "%s (%s)" % (_fmt(p["mean"]), _fmt(p.get("sd")))
        ys = [v for k2, v in q["yaw_offset_deg"]["per_sitting"].items() if k2.startswith(bag)]
        yaw = "; ".join("%s (%s)" % (_fmt(v["mean"]), _fmt(v.get("sd"))) for v in ys)
        L.append("| %s | %d | %s | %s | %s | %s | %s | %s |" % (
            bag, st["n"], ps("kappa_deg"), yaw, ps("sweep_end_deg"), ps("lag_ms"),
            ps("template_residual_mm"), ps("base_residual_mm")))
    L.append("\n### Kappa by kappa epoch\n")
    L.append("| epoch | sweeps | mean | sweep SD | sitting means SD | min | max |\n"
             "|---|---|---|---|---|---|---|")
    for ep, d in q["kappa_deg"]["kappa_epochs"]["per_epoch"].items():
        L.append("| %s | %d | %s | %s | %s | %s | %s |" % (
            ep, d["sweeps"]["n"], _fmt(d["sweeps"]["mean"]), _fmt(d["sweeps"].get("sd")),
            _fmt(d["sitting_means"].get("sd")), _fmt(d["sweeps"]["min"]),
            _fmt(d["sweeps"]["max"])))
    kv = s["kappa_vs_sweep_end"]
    L.append("\nKappa vs sweep end: within-kappa-epoch slope %s deg/deg (r = %s); raw pooled slope "
             "%s deg/deg (r = %s), n = %d.\n" % (
                 _fmt(kv["slope_within_epoch_deg_per_deg"], 4), _fmt(kv["r_within_epoch"], 2),
                 _fmt(kv["slope_pooled_raw_deg_per_deg"], 4), _fmt(kv["r_raw"], 2), kv["n"]))
    L.append("## Advisory bands (computed)\n")
    L.append("Rule: band = max(3 x SD, 1.25 x largest observed intrinsic-good value), rounded up. "
             "A 'bump' is %.0f mm at the %.0f mm marker radius = %.2f deg, or a 2 mm translation.\n"
             % (BUMP_MM, MARKER_RADIUS_MM, math.degrees(BUMP_MM / MARKER_RADIUS_MM)))
    L.append("| advisory | SD used | 3 x SD | max observed | band | coverage | bump / band | note |")
    L.append("|---|---|---|---|---|---|---|---|")
    for b in bands:
        L.append("| %s | %s %s (%s) | %s | %s | **%s %s** | %d/%d | %s | %s |" % (
            b["advisory"], _fmt(b["sd"]), b["unit"], b["sd_label"], _fmt(3 * (b["sd"] or 0)),
            _fmt(b["max_observed"]), _fmt(b["band"], 2), b["unit"], b["coverage_in"],
            b["coverage_n"], _fmt(b["bump_over_band"], 2), b["note"]))
    L.append("\n## P7 numbers\n")
    L.append("Within-sitting kappa SD %s deg; within-kappa-epoch sitting-mean SD %s deg. "
             "3-vs-3 in one sitting: SE of the difference %s deg, detectable (3 SE) %s deg; "
             "3-vs-3 across sittings in one epoch: %s deg; single sweep vs single sweep: %s deg.\n"
             % (_fmt(p7["within_sitting_sd"]), _fmt(p7["within_epoch_sitting_mean_sd"]),
                _fmt(p7["se_diff_3v3_same_sitting"]), _fmt(p7["detectable_3v3_same_sitting_deg"]),
                _fmt(p7["detectable_3v3_across_sittings_deg"]),
                _fmt(p7["detectable_single_sweep_vs_single_sweep_deg"])))
    if cc is not None:
        L.append("## Throw cross-check (computed)\n")
        L.append("| run | bag | feeds | in-force sweep | yaw (deg) | kappa | pose src | pool n | "
                 "miss dx / dy (mm) | land-ann dx / dy | bias range / lateral | yaw err (deg) "
                 "-> pred lateral | axis err (mm) -> pred range / lateral |")
        L.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
        for r in cc["launches"]:
            f = r["in_force"] or {}
            L.append("| %s | %s | %d | #%s | %s | %s | %s | %s | %s / %s | %s / %s | %s / %s | "
                     "%s -> %s | %s -> %s / %s |" % (
                         r["run"], r["bag"], r["n_feeds"], f.get("sweep_idx"),
                         _fmt(f.get("yaw_offset_deg")), _fmt(f.get("kappa_deg")),
                         f.get("pose_source"), _fmt(f.get("pool_n"), 0), _fmt(r["miss_dx"], 1),
                         _fmt(r["miss_dy"], 1), _fmt(r["land_minus_ann_x"], 1),
                         _fmt(r["land_minus_ann_y"], 1), _fmt(r.get("bias_range_mm"), 1),
                         _fmt(r.get("bias_lateral_mm"), 1), _fmt(r.get("yaw_err_deg")),
                         _fmt(r.get("pred_lateral_from_yaw_mm"), 1), _fmt(r.get("axis_err_mm"), 2),
                         _fmt(r.get("pred_range_from_axis_mm"), 1),
                         _fmt(r.get("pred_lateral_from_axis_mm"), 1)))
        L.append("\n| pair | d kappa in force (deg) | -> lateral (mm) | d bias range (mm) | "
                 "d bias lateral (mm) |\n|---|---|---|---|---|")
        for p in cc["pairs"]:
            L.append("| %s -> %s | %s | %s | %s | %s |" % (
                p["a"], p["b"], _fmt(p["dkappa_deg"]), _fmt(p["pred_lateral_mm"], 1),
                _fmt(p["dbias_range_mm"], 1), _fmt(p["dbias_lateral_mm"], 1)))
    if extra_md:
        L.append("\n" + extra_md.strip() + "\n")
    return "\n".join(L) + "\n"


def _jsonable(x: Any) -> Any:
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items() if k != "_rows"}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    if isinstance(x, set):
        return sorted(x)
    return x


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("replay_dir")
    ap.add_argument("--feeds", default=None,
                    help="columns analysis per-feed CSV (u4_feeds.csv) for the throw cross-check")
    ap.add_argument("--discussion", default=None,
                    help="markdown file appended verbatim to bands.md (the band reasoning)")
    a = ap.parse_args(argv)
    rows = load_rows(a.replay_dir)
    s = summarise(rows)
    bands = propose_bands(s)
    p7 = p7_prediction(s)
    cc = throw_crosscheck(s, rows, a.feeds) if a.feeds else None
    extra = open(a.discussion).read() if a.discussion else ""
    md = render_md(s, bands, p7, cc, a.replay_dir, extra)
    with open(os.path.join(a.replay_dir, "bands.md"), "w") as fh:
        fh.write(md)
    with open(os.path.join(a.replay_dir, "bands.json"), "w") as fh:
        json.dump(_jsonable({"summary": s, "bands": bands, "p7": p7, "throw_crosscheck": cc}),
                  fh, indent=1, default=str)
    sys.stdout.write(md)
    return 0


if __name__ == "__main__":
    sys.exit(main())
