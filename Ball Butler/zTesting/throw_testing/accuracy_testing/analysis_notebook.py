import marimo

__generated_with = "0.10.0"
app = marimo.App(width="full")


# ── Imports ──────────────────────────────────────────────────────────────────


@app.cell
def _():
    import marimo as mo
    return (mo,)


@app.cell
def _():
    import numpy as np
    from pathlib import Path
    from collections import defaultdict
    import matplotlib.pyplot as plt
    return Path, defaultdict, np, plt


# ── Configuration ────────────────────────────────────────────────────────────


@app.cell
def _():
    TOPIC = "/throw_announcements"
    THROW_ANNOUNCEMENT_MSGDEF = """\
std_msgs/Header header
string thrower_name
geometry_msgs/Point initial_position
geometry_msgs/Vector3 initial_velocity
string target_id
geometry_msgs/Point target_position
builtin_interfaces/Time throw_time
float64 predicted_tof_sec
"""
    FREQ = 300.0
    PLANE_Z = 750.0
    RELEASE_VZ_THRESHOLD = 1000.0
    RELEASE_VZ_FALLBACK = 500.0
    PRE_RELEASE_SPEED_FLOOR = 200.0
    RELEASE_SEARCH_BACK = 50
    return (
        FREQ,
        PLANE_Z,
        PRE_RELEASE_SPEED_FLOOR,
        RELEASE_SEARCH_BACK,
        RELEASE_VZ_FALLBACK,
        RELEASE_VZ_THRESHOLD,
        THROW_ANNOUNCEMENT_MSGDEF,
        TOPIC,
    )


# ── Target Loading (CSV or MCAP) ─────────────────────────────────────────────


@app.cell
def _(THROW_ANNOUNCEMENT_MSGDEF, TOPIC):
    def load_targets_from_csv(csv_path):
        """Load pre-extracted targets from a _targets.csv file.

        Handles CSVs with or without timestamp columns. Missing timestamp
        fields are set to None so downstream analysis can detect and skip them.
        """
        import csv

        records = []
        with open(csv_path, newline="") as f:
            for row in csv.DictReader(f):
                label = row["label"].strip()
                if label.startswith("New "):
                    try:
                        label = f"b{int(label.removeprefix('New '))}"
                    except ValueError:
                        pass

                def _opt_float(key):
                    val = row.get(key)
                    return float(val) if val is not None and val != "" else None

                records.append(
                    {
                        "label": label,
                        "target_id": row["target_id"],
                        "target_x": float(row["target_x"]),
                        "target_y": float(row["target_y"]),
                        "target_z": float(row["target_z"]),
                        "header_stamp": _opt_float("header_stamp"),
                        "throw_time": _opt_float("throw_time"),
                        "predicted_tof": _opt_float("predicted_tof_sec"),
                        "log_time": _opt_float("log_time"),
                    }
                )
        return records

    def extract_targets_from_mcap(mcap_path):
        """Extract throw announcements from a standalone .mcap file."""
        from mcap.reader import make_reader
        from rosbags.typesys import Stores, get_typestore, get_types_from_msg

        store = get_typestore(Stores.ROS2_HUMBLE)
        msg_type_name = "throw_interfaces/msg/ThrowAnnouncement"
        store.register(
            get_types_from_msg(THROW_ANNOUNCEMENT_MSGDEF, msg_type_name)
        )

        records = []
        with open(mcap_path, "rb") as f:
            reader = make_reader(f)
            for count, (_schema, _channel, message) in enumerate(
                reader.iter_messages(topics=[TOPIC])
            ):
                msg = store.deserialize_cdr(message.data, msg_type_name)
                records.append(
                    {
                        "label": f"b{count}",
                        "target_id": msg.target_id,
                        "target_x": msg.target_position.x,
                        "target_y": msg.target_position.y,
                        "target_z": msg.target_position.z,
                        "header_stamp": msg.header.stamp.sec
                        + msg.header.stamp.nanosec * 1e-9,
                        "throw_time": msg.throw_time.sec
                        + msg.throw_time.nanosec * 1e-9,
                        "predicted_tof": msg.predicted_tof_sec,
                        "log_time": message.log_time * 1e-9,
                    }
                )
        return records

    return extract_targets_from_mcap, load_targets_from_csv


# ── Trajectory Analysis Functions ────────────────────────────────────────────


@app.cell
def _(
    FREQ,
    PLANE_Z,
    PRE_RELEASE_SPEED_FLOOR,
    RELEASE_SEARCH_BACK,
    RELEASE_VZ_FALLBACK,
    RELEASE_VZ_THRESHOLD,
    defaultdict,
    np,
):
    def _normalize_label(label):
        text = label.strip()
        if text.startswith("New "):
            try:
                return f"b{int(text.removeprefix('New '))}"
            except ValueError:
                return text
        return text

    def load_qtm_trajectories(filepath):
        """Load QTM JSON and return {label: {values, frames}} and frequency."""
        import json

        with open(filepath) as f:
            qtm = json.load(f)
        freq = qtm["Timebase"]["Frequency"]
        trajectories = {}
        for marker in qtm["Markers"]:
            name = _normalize_label(marker["Name"])
            all_values, all_frames = [], []
            for part in marker["Parts"]:
                vals = np.array(part["Values"])
                if vals.size == 0:
                    continue
                start = part["Range"]["Start"]
                all_values.append(vals)
                all_frames.append(np.arange(start, start + len(vals)))
            if not all_values or name in trajectories:
                continue
            trajectories[name] = {
                "values": np.vstack(all_values),
                "frames": np.concatenate(all_frames),
            }
        return trajectories, freq

    def _find_release_frame(values, frames):
        if len(values) < 5:
            return None
        dt = 1.0 / FREQ
        vel_3d = np.diff(values[:, :3], axis=0) / dt
        speed = np.linalg.norm(vel_3d, axis=1)
        vz = np.diff(values[:, 2]) / dt
        peak_z_idx = np.argmax(values[:, 2])
        vz_search = vz[: min(peak_z_idx, len(vz))]
        if len(vz_search) == 0:
            return None
        above = np.where(vz_search > RELEASE_VZ_THRESHOLD)[0]
        if len(above) == 0:
            above = np.where(vz_search > RELEASE_VZ_FALLBACK)[0]
        if len(above) == 0:
            return None
        first_fast = above[0]
        search_start = max(0, first_fast - RELEASE_SEARCH_BACK)
        search_region = speed[search_start:first_fast]
        if len(search_region) > 0:
            slow = np.where(search_region < PRE_RELEASE_SPEED_FLOOR)[0]
            release_idx = (
                search_start + slow[-1]
                if len(slow) > 0
                else max(0, first_fast - 5)
            )
        else:
            release_idx = max(0, first_fast - 5)
        return frames[release_idx]

    def _find_landing(values, frames):
        z = values[:, 2]
        for i in range(len(z) - 1):
            if z[i] > PLANE_Z and z[i + 1] <= PLANE_Z:
                dz = z[i] - z[i + 1]
                frac = (z[i] - PLANE_Z) / dz if dz != 0 else 0.5
                p0, p1 = values[i, :3], values[i + 1, :3]
                return frames[i] + frac, p0 + frac * (p1 - p0)
        return None, None

    def run_analysis(targets, trajectories, excluded_labels):
        """Run full timing + spatial analysis.

        Handles targets with or without timestamp data. When timestamps are
        missing, only spatial analysis is performed (timing fields set to None).

        Returns (results, by_target, clock_offset, clock_msg).
        """
        excluded = {_normalize_label(x) for x in excluded_labels}
        traj = {k: v for k, v in trajectories.items() if k not in excluded}
        tgts = [t for t in targets if t["label"] not in excluded]

        # Check whether we have timestamp data for timing analysis
        _has_timing = (
            tgts
            and tgts[0].get("throw_time") is not None
            and tgts[0].get("predicted_tof") is not None
        )

        # Clock alignment via release-frame tie-points (only if timestamps exist)
        clock_offset = None
        clock_msg = "N/A (no timestamp data)"
        if _has_timing:
            offsets = []
            for t in tgts:
                if t["label"] not in traj:
                    continue
                d = traj[t["label"]]
                rf = _find_release_frame(d["values"], d["frames"])
                if rf is not None:
                    offsets.append(t["throw_time"] - rf / FREQ)

            if offsets:
                offsets_arr = np.array(offsets)
                clock_offset = float(np.median(offsets_arr))
                spread_ms = (
                    (np.max(offsets_arr) - np.min(offsets_arr)) * 1000
                )
                clock_msg = (
                    f"median offset = {clock_offset:.6f}s, "
                    f"spread = {spread_ms:.1f}ms across {len(offsets)} throws"
                )
            else:
                clock_msg = "No valid release frames found"

        # Per-throw analysis
        results = []
        for t in tgts:
            label = t["label"]
            if label not in traj:
                continue
            d = traj[label]
            release_frame = _find_release_frame(d["values"], d["frames"])
            landing_frame, landing_xyz = _find_landing(d["values"], d["frames"])
            if landing_frame is None:
                continue

            # Timing metrics (only if we have timestamps + clock alignment)
            landing_err_ms = None
            actual_tof = None
            release_err_ms = None
            tof_err_ms = None

            if _has_timing and clock_offset is not None:
                actual_landing = landing_frame / FREQ + clock_offset
                predicted_landing = t["throw_time"] + t["predicted_tof"]
                landing_err_ms = (actual_landing - predicted_landing) * 1000

                if release_frame is not None:
                    actual_tof = (landing_frame - release_frame) / FREQ
                    release_err_ms = (
                        release_frame / FREQ
                        + clock_offset
                        - t["throw_time"]
                    ) * 1000
                    tof_err_ms = (actual_tof - t["predicted_tof"]) * 1000

            # Spatial error (always computable if we have a landing)
            spatial_err = float(
                np.sqrt(
                    (landing_xyz[0] - t["target_x"]) ** 2
                    + (landing_xyz[1] - t["target_y"]) ** 2
                )
            )

            results.append(
                {
                    "label": label,
                    "target_id": t["target_id"],
                    "target_x": t["target_x"],
                    "target_y": t["target_y"],
                    "predicted_tof": t.get("predicted_tof"),
                    "actual_tof": actual_tof,
                    "tof_err_ms": tof_err_ms,
                    "release_err_ms": release_err_ms,
                    "landing_err_ms": landing_err_ms,
                    "spatial_err_mm": spatial_err,
                    "landing_x": float(landing_xyz[0]),
                    "landing_y": float(landing_xyz[1]),
                    "has_release": release_frame is not None,
                }
            )

        by_target = defaultdict(list)
        for r in results:
            by_target[r["target_id"]].append(r)

        return results, dict(by_target), clock_offset, clock_msg

    return load_qtm_trajectories, run_analysis


# ── Session Selection ────────────────────────────────────────────────────────


@app.cell
def _(Path, mo):
    # Discover sessions: a .json file with either a matching .mcap or _targets.csv
    # Searches the current directory and immediate subdirectories.
    _root = Path(".")
    _search_dirs = [_root] + [
        p for p in _root.iterdir() if p.is_dir() and not p.name.startswith(".")
    ]

    sessions = {}
    for _dir in _search_dirs:
        for _json_path in sorted(_dir.glob("*.json")):
            _stem = _json_path.stem
            _mcap = _json_path.with_suffix(".mcap")
            _csv = _json_path.with_name(f"{_stem}_targets.csv")
            if _mcap.exists() or _csv.exists():
                # Use subdir/stem as the display name to avoid collisions
                _label = (
                    f"{_dir.name}/{_stem}" if _dir != _root else _stem
                )
                sessions[_label] = {
                    "json": _json_path,
                    "mcap": _mcap if _mcap.exists() else None,
                    "csv": _csv if _csv.exists() else None,
                }

    mo.stop(
        not sessions,
        mo.md(
            "# Ball Butler Throw Accuracy Analysis\n\n"
            "**No sessions found.** Place `.json` (QTM) files with matching "
            "`.mcap` or `_targets.csv` files in this directory and reload."
        ),
    )

    session_dropdown = mo.ui.dropdown(
        options=list(sessions.keys()),
        value=list(sessions.keys())[0],
        label="Session",
    )

    # Build a description of what's available per session
    _details = []
    for _name, _paths in sessions.items():
        _has = []
        if _paths["csv"]:
            _has.append("csv")
        if _paths["mcap"]:
            _has.append("mcap")
        _details.append(f"- **{_name}**: {' + '.join(_has)}")

    mo.md(
        f"# Ball Butler Throw Accuracy Analysis\n\n"
        f"Found **{len(sessions)}** session(s):\n\n"
        + "\n".join(_details)
        + f"\n\n{session_dropdown}"
    )
    return session_dropdown, sessions


# ── Load Data ────────────────────────────────────────────────────────────────


@app.cell
def _(
    extract_targets_from_mcap,
    load_targets_from_csv,
    load_qtm_trajectories,
    mo,
    session_dropdown,
    sessions,
):
    mo.stop(session_dropdown.value is None)

    _sel = session_dropdown.value
    _paths = sessions[_sel]

    # Load targets: prefer CSV with timestamps, then MCAP extraction, then CSV without
    _target_source = ""
    _warning = ""
    targets = None

    if _paths["csv"]:
        _csv_targets = load_targets_from_csv(_paths["csv"])
        _has_timestamps = (
            _csv_targets
            and _csv_targets[0].get("throw_time") is not None
            and _csv_targets[0].get("predicted_tof") is not None
        )
        if _has_timestamps:
            targets = _csv_targets
            _target_source = f"CSV (`{_paths['csv'].name}`)"
        elif _paths["mcap"]:
            # CSV lacks timestamps — try MCAP extraction for full data
            try:
                targets = extract_targets_from_mcap(_paths["mcap"])
                _target_source = f"MCAP extraction (`{_paths['mcap'].name}`)"
            except Exception as _e:
                targets = _csv_targets
                _target_source = f"CSV (`{_paths['csv'].name}`)"
                _warning = (
                    f"\n\n**Warning:** CSV lacks timestamp columns and MCAP "
                    f"extraction failed ({type(_e).__name__}: {_e}). "
                    f"Timing analysis will not be available for this session."
                )
        else:
            targets = _csv_targets
            _target_source = f"CSV (`{_paths['csv'].name}`)"
            _warning = (
                "\n\n**Warning:** CSV lacks timestamp columns. "
                "Timing analysis will not be available for this session."
            )
    elif _paths["mcap"]:
        try:
            targets = extract_targets_from_mcap(_paths["mcap"])
            _target_source = f"MCAP extraction (`{_paths['mcap'].name}`)"
        except Exception as _e:
            mo.stop(
                True,
                mo.md(
                    f"**Error:** MCAP extraction failed: "
                    f"{type(_e).__name__}: {_e}"
                ),
            )
    else:
        mo.stop(True, mo.md("**Error:** No target data source found."))

    trajectories, _freq = load_qtm_trajectories(_paths["json"])

    mo.md(
        f"Loaded **{_sel}**: {len(targets)} throw announcements "
        f"(from {_target_source}), "
        f"{len(trajectories)} QTM trajectories ({_freq:.0f} Hz)"
        + _warning
    )
    return targets, trajectories


# ── Throw Exclusions ─────────────────────────────────────────────────────────


@app.cell
def _(mo, targets):
    exclude_selector = mo.ui.multiselect(
        options=[t["label"] for t in targets],
        label="Exclude throws",
        value=[],
    )

    mo.md(f"#### Exclude specific throws from analysis:\n\n{exclude_selector}")
    return (exclude_selector,)


# ── Run Analysis & Summary ───────────────────────────────────────────────────


@app.cell
def _(exclude_selector, mo, np, run_analysis, targets, trajectories):
    results, by_target, clock_offset, clock_msg = run_analysis(
        targets, trajectories, exclude_selector.value
    )

    mo.stop(not results, mo.md(f"**No results.** {clock_msg}"))

    # Determine what metrics are available
    _has_timing = any(r["landing_err_ms"] is not None for r in results)

    # Overall summary
    _n_rel = sum(1 for r in results if r["has_release"])
    _all_sp = [r["spatial_err_mm"] for r in results]

    _lines = []
    if _has_timing:
        _lines.append(f"**Clock alignment:** {clock_msg}")
    _lines.append(
        f"**Throws analysed:** {len(results)} "
        f"({_n_rel} with release detection)"
    )
    if _has_timing:
        _all_tof = [
            r["tof_err_ms"]
            for r in results
            if r["tof_err_ms"] is not None
        ]
        _all_land = [
            r["landing_err_ms"]
            for r in results
            if r["landing_err_ms"] is not None
        ]
        if _all_tof:
            _lines.append(
                f"**ToF error:** mean = {np.mean(_all_tof):+.1f} ms, "
                f"std = {np.std(_all_tof):.1f} ms"
            )
        if _all_land:
            _lines.append(
                f"**Landing time error:** "
                f"mean = {np.mean(_all_land):+.1f} ms, "
                f"std = {np.std(_all_land):.1f} ms"
            )
    _lines.append(
        f"**Spatial error:** mean = {np.mean(_all_sp):.1f} mm, "
        f"max = {np.max(_all_sp):.1f} mm"
    )

    # Per-target summary table
    if _has_timing:
        _header = (
            "| Target | Position (mm) | Throws | Land Err Mean (ms) "
            "| Land Err Std (ms) | Spatial Mean (mm) | Spatial Max (mm) |\n"
            "|--------|--------------|--------|--------------------|"
            "-------------------|-------------------|------------------|\n"
        )
        _rows = []
        for _tid in sorted(by_target.keys()):
            _rs = by_target[_tid]
            _le = [
                r["landing_err_ms"]
                for r in _rs
                if r["landing_err_ms"] is not None
            ]
            _sp = [r["spatial_err_mm"] for r in _rs]
            _le_mean = f"{np.mean(_le):+.1f}" if _le else "N/A"
            _le_std = f"{np.std(_le):.1f}" if _le else "N/A"
            _rows.append(
                f"| {_tid} | ({_rs[0]['target_x']:.0f}, "
                f"{_rs[0]['target_y']:.0f}) "
                f"| {len(_rs)} | {_le_mean} | {_le_std} "
                f"| {np.mean(_sp):.1f} | {np.max(_sp):.1f} |"
            )
    else:
        _header = (
            "| Target | Position (mm) | Throws "
            "| Spatial Mean (mm) | Spatial Max (mm) |\n"
            "|--------|--------------|--------"
            "|-------------------|------------------|\n"
        )
        _rows = []
        for _tid in sorted(by_target.keys()):
            _rs = by_target[_tid]
            _sp = [r["spatial_err_mm"] for r in _rs]
            _rows.append(
                f"| {_tid} | ({_rs[0]['target_x']:.0f}, "
                f"{_rs[0]['target_y']:.0f}) "
                f"| {len(_rs)} "
                f"| {np.mean(_sp):.1f} | {np.max(_sp):.1f} |"
            )

    mo.md(
        "## Summary\n\n"
        + "\n\n".join(_lines)
        + "\n\n### Per-Target Breakdown\n\n"
        + _header
        + "\n".join(_rows)
    )
    return by_target, results


# ── Per-Throw Results Table ──────────────────────────────────────────────────


@app.cell
def _(mo, results):
    def _fmt(val, fmt_str, scale=1):
        """Format a value with a format string, or return 'N/A' if None."""
        if val is None:
            return "N/A"
        return f"{val * scale:{fmt_str}}"

    _table = [
        {
            "Ball": r["label"],
            "Target": r["target_id"],
            "Pred ToF (ms)": _fmt(r["predicted_tof"], ".1f", 1000),
            "Actual ToF (ms)": _fmt(r["actual_tof"], ".1f", 1000),
            "ToF Err (ms)": _fmt(r["tof_err_ms"], "+.1f"),
            "Release Err (ms)": _fmt(r["release_err_ms"], "+.1f"),
            "Landing Err (ms)": _fmt(r["landing_err_ms"], "+.1f"),
            "Spatial Err (mm)": f"{r['spatial_err_mm']:.1f}",
        }
        for r in results
    ]

    mo.vstack(
        [
            mo.md("## Per-Throw Results"),
            mo.ui.table(_table),
        ]
    )
    return ()


# ── Plots ────────────────────────────────────────────────────────────────────


@app.cell
def _(by_target, mo, np, plt, results, session_dropdown):
    _tids = sorted(by_target.keys())
    _colors = plt.cm.Set1(np.linspace(0, 0.8, max(len(_tids), 1)))
    _cmap = {tid: _colors[i] for i, tid in enumerate(_tids)}
    _rng = np.random.default_rng(42)
    _has_timing = any(r["landing_err_ms"] is not None for r in results)

    if _has_timing:
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    else:
        fig, axes = plt.subplots(1, 1, figsize=(8, 7))
        axes = np.array([[axes, None], [None, None]])

    fig.suptitle(
        f"Ball Butler Analysis — {session_dropdown.value}",
        fontsize=14,
        fontweight="bold",
    )

    if _has_timing:
        # ── (a) Landing time error per throw ─────────────────────────────
        ax = axes[0, 0]
        for i, r in enumerate(results):
            if r["landing_err_ms"] is not None:
                ax.bar(
                    i,
                    r["landing_err_ms"],
                    color=_cmap[r["target_id"]],
                    edgecolor="white",
                    linewidth=0.5,
                )
        ax.axhline(0, color="black", linewidth=0.8)
        ax.set_xlabel("Throw index")
        ax.set_ylabel("Landing time error (ms)")
        ax.set_title("(a) Landing time: actual − predicted")
        for _tid in _tids:
            _idx = [
                i
                for i, r in enumerate(results)
                if r["target_id"] == _tid
            ]
            if _idx:
                ax.text(
                    np.mean(_idx),
                    ax.get_ylim()[1] * 0.9,
                    _tid.replace("volley_", "V"),
                    ha="center",
                    fontsize=8,
                    color=_cmap[_tid],
                    fontweight="bold",
                )

        # ── (b) ToF: predicted vs actual ─────────────────────────────────
        ax = axes[0, 1]
        for _tid in _tids:
            _rs = [
                r for r in by_target[_tid] if r["actual_tof"] is not None
            ]
            if not _rs:
                continue
            ax.scatter(
                [r["predicted_tof"] * 1000 for r in _rs],
                [r["actual_tof"] * 1000 for r in _rs],
                c=[_cmap[_tid]],
                s=50,
                label=_tid,
                edgecolors="black",
                linewidths=0.5,
                zorder=3,
            )
        _with_tof = [r for r in results if r["actual_tof"] is not None]
        if _with_tof:
            _all_tof_vals = [
                r["actual_tof"] * 1000 for r in _with_tof
            ] + [r["predicted_tof"] * 1000 for r in _with_tof]
            _lims = [min(_all_tof_vals) - 10, max(_all_tof_vals) + 10]
            ax.plot(
                _lims, _lims, "k--", linewidth=0.8, alpha=0.5, label="perfect"
            )
            ax.set_xlim(_lims)
            ax.set_ylim(_lims)
        ax.set_xlabel("Predicted ToF (ms)")
        ax.set_ylabel("Actual ToF (ms)")
        ax.set_title("(b) Time of flight: predicted vs actual")
        ax.legend(fontsize=8)
        ax.set_aspect("equal")

        # ── (c) Landing time error box plot by target ────────────────────
        ax = axes[1, 0]
        _box_data = [
            [
                r["landing_err_ms"]
                for r in by_target[_tid]
                if r["landing_err_ms"] is not None
            ]
            for _tid in _tids
        ]
        # Only plot if we have data in each group
        _valid_box = [d for d in _box_data if d]
        _valid_tids = [
            t for t, d in zip(_tids, _box_data) if d
        ]
        if _valid_box:
            _bp = ax.boxplot(
                _valid_box,
                tick_labels=_valid_tids,
                patch_artist=True,
                widths=0.5,
                medianprops=dict(color="black", linewidth=2),
            )
            for _patch, _tid in zip(_bp["boxes"], _valid_tids):
                _patch.set_facecolor(_cmap[_tid])
                _patch.set_alpha(0.7)
            for i, (_tid, _data) in enumerate(
                zip(_valid_tids, _valid_box)
            ):
                _x = _rng.normal(i + 1, 0.06, len(_data))
                ax.scatter(
                    _x,
                    _data,
                    c=[_cmap[_tid]],
                    s=25,
                    edgecolors="black",
                    linewidths=0.5,
                    zorder=3,
                )
        ax.axhline(0, color="black", linewidth=0.8)
        ax.set_ylabel("Landing time error (ms)")
        ax.set_title("(c) Landing time error by target")

        # ── (d) Spatial landing accuracy ─────────────────────────────────
        ax = axes[1, 1]
    else:
        # No timing data — just show spatial plot
        ax = axes[0, 0]

    for _tid in _tids:
        _rs = by_target[_tid]
        _c = _cmap[_tid]
        _tx, _ty = _rs[0]["target_x"], _rs[0]["target_y"]
        ax.scatter(
            _tx, _ty, c=[_c], s=150, marker="x", linewidths=3, zorder=5
        )
        _lx = [r["landing_x"] for r in _rs]
        _ly = [r["landing_y"] for r in _rs]
        ax.scatter(
            _lx,
            _ly,
            c=[_c],
            s=30,
            alpha=0.7,
            edgecolors="black",
            linewidths=0.5,
            zorder=3,
            label=_tid,
        )
        ax.annotate(
            "",
            xy=(_tx, _ty),
            xytext=(np.mean(_lx), np.mean(_ly)),
            arrowprops=dict(
                arrowstyle="-|>", color=_c, lw=1.5, shrinkA=3, shrinkB=5
            ),
        )
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title(
        "Spatial accuracy (× = target, dots = landings)"
        if not _has_timing
        else "(d) Spatial accuracy (× = target, dots = landings)"
    )
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    plt.tight_layout()

    # Render to PNG bytes for the download button
    import io as _io

    _buf = _io.BytesIO()
    fig.savefig(_buf, format="png", dpi=150, bbox_inches="tight")
    _buf.seek(0)

    _download = mo.download(
        _buf.getvalue(),
        filename=f"analysis_{session_dropdown.value}.png",
        mimetype="image/png",
        label="Save plot as PNG",
    )

    mo.vstack([fig, _download])
    return ()


if __name__ == "__main__":
    app.run()
