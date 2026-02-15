"""
Ball Butler Timing Analysis
============================
Analyses the timing of ball throws by comparing:
  - Predicted landing time (from ThrowAnnouncement: throw_time + predicted_tof)
  - Actual landing time (from QTM trajectory crossing the target plane)

Requires clock alignment between QTM (frame-based) and ROS (Unix time).
Uses the ball release event as a tie-point for alignment.

Usage:
    python timing_analysis.py <session.json> <session_targets.csv>

    # Exclude specific throws from analysis
    python timing_analysis.py <session.json> <session_targets.csv> --exclude b3 b7 b12

Dependencies:
    pip install numpy matplotlib
"""

import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


# ── Configuration ───────────────────────────────────────────────────────────

PLANE_Z = 750.0                 # Target plane height (mm)
RELEASE_VZ_THRESHOLD = 1000.0   # mm/s — ball clearly going up
RELEASE_VZ_FALLBACK = 500.0     # mm/s — lower fallback
PRE_RELEASE_SPEED_FLOOR = 200.0 # mm/s — "still in hand" speed cutoff
RELEASE_SEARCH_BACK = 50        # frames to look back from first fast frame


def normalize_label(label: str) -> str:
    """Normalize labels so both "New 0001" and "b1" map to "b1"."""
    text = label.strip()
    if text.startswith("New "):
        try:
            num = int(text.removeprefix("New "))
            return f"b{num}"
        except ValueError:
            return text
    if text.startswith("b"):
        return text
    return text


# ── Data Loading ────────────────────────────────────────────────────────────


def load_qtm_trajectories(filepath: Path, freq: float) -> dict:
    """Load QTM JSON and extract per-marker trajectory data.

    Returns:
        {marker_name: {'values': ndarray(N,4), 'frames': ndarray(N,)}}
    """
    with open(filepath) as f:
        qtm = json.load(f)

    assert qtm['Timebase']['Frequency'] == freq, (
        f"Expected {freq} Hz, got {qtm['Timebase']['Frequency']}"
    )

    trajectories = {}
    for marker in qtm['Markers']:
        name = normalize_label(marker['Name'])
        all_values, all_frames = [], []
        for part in marker['Parts']:
            vals = np.array(part['Values'])
            if vals.size == 0:
                continue
            start = part['Range']['Start']
            all_values.append(vals)
            all_frames.append(np.arange(start, start + len(vals)))
        if not all_values:
            continue
        if name in trajectories:
            print(f"Warning: duplicate label after normalization: {name} — keeping first")
            continue
        trajectories[name] = {
            'values': np.vstack(all_values),
            'frames': np.concatenate(all_frames),
        }

    return trajectories, qtm


def load_targets(filepath: Path) -> list[dict]:
    """Load the ROS target CSV with timestamps."""
    targets = []
    with open(filepath, newline='') as f:
        for row in csv.DictReader(f):
            targets.append({
                'label': normalize_label(row['label']),
                'target_id': row['target_id'],
                'target_x': float(row['target_x']),
                'target_y': float(row['target_y']),
                'target_z': float(row['target_z']),
                'header_stamp': float(row['header_stamp']),
                'throw_time': float(row['throw_time']),
                'predicted_tof': float(row['predicted_tof_sec']),
                'log_time': float(row['log_time']),
            })
    return targets


# ── Trajectory Analysis ─────────────────────────────────────────────────────


def find_release_frame(values: np.ndarray, frames: np.ndarray, freq: float) -> int | None:
    """Detect the release frame from Z-velocity profile.

    Looks for where Z-velocity first spikes upward (ball being thrown),
    then walks back to find where the ball was still stationary.
    """
    if len(values) < 5:
        return None

    dt = 1.0 / freq
    vel_3d = np.diff(values[:, :3], axis=0) / dt
    speed = np.linalg.norm(vel_3d, axis=1)
    vz = np.diff(values[:, 2]) / dt

    # Find peak Z (apex of trajectory)
    peak_z_idx = np.argmax(values[:, 2])
    search_end = min(peak_z_idx, len(vz))
    vz_search = vz[:search_end]

    if len(vz_search) == 0:
        return None

    # Find first frame where vz exceeds threshold
    above = np.where(vz_search > RELEASE_VZ_THRESHOLD)[0]
    if len(above) == 0:
        above = np.where(vz_search > RELEASE_VZ_FALLBACK)[0]
    if len(above) == 0:
        return None

    first_fast = above[0]

    # Walk back to find where speed was still low (ball in hand)
    search_start = max(0, first_fast - RELEASE_SEARCH_BACK)
    search_region = speed[search_start:first_fast]

    if len(search_region) > 0:
        slow_frames = np.where(search_region < PRE_RELEASE_SPEED_FLOOR)[0]
        if len(slow_frames) > 0:
            release_idx = search_start + slow_frames[-1]
        else:
            release_idx = max(0, first_fast - 5)
    else:
        release_idx = max(0, first_fast - 5)

    return frames[release_idx]


def find_landing_frame(values: np.ndarray, frames: np.ndarray, plane_z: float):
    """Find where the trajectory first crosses the target plane descending.

    Returns (landing_frame_interpolated, landing_xyz) or (None, None).
    """
    z = values[:, 2]
    for i in range(len(z) - 1):
        if z[i] > plane_z and z[i + 1] <= plane_z:
            dz = z[i] - z[i + 1]
            frac = (z[i] - plane_z) / dz if dz != 0 else 0.5
            landing_frame = frames[i] + frac
            p0 = values[i, :3]
            p1 = values[i + 1, :3]
            landing_xyz = p0 + frac * (p1 - p0)
            return landing_frame, landing_xyz

    return None, None


# ── Clock Alignment ─────────────────────────────────────────────────────────


def compute_clock_offset(targets, trajectories, freq):
    """Compute the QTM→ROS clock offset using release frames as tie-points.

    For each throw: offset = ros_throw_time - qtm_release_frame / freq
    Returns the median offset (robust to outliers).
    """
    offsets = []

    for t in targets:
        label = t['label']
        if label not in trajectories:
            continue

        d = trajectories[label]
        release_frame = find_release_frame(d['values'], d['frames'], freq)
        if release_frame is None:
            continue

        ros_throw = t['throw_time']
        qtm_release_s = release_frame / freq
        offset = ros_throw - qtm_release_s
        offsets.append(offset)

    if not offsets:
        print("Clock alignment failed: no valid release frames found after exclusions")
        return None

    offsets = np.array(offsets)
    median_offset = np.median(offsets)
    spread_ms = (np.max(offsets) - np.min(offsets)) * 1000

    print(f"Clock alignment: median offset = {median_offset:.6f}s, "
          f"spread = {spread_ms:.1f}ms across {len(offsets)} throws")

    return median_offset


# ── Main Analysis ───────────────────────────────────────────────────────────


def analyse(qtm_path: Path, csv_path: Path, exclude: list[str] | None = None):
    freq = 300.0
    trajectories, qtm_raw = load_qtm_trajectories(qtm_path, freq)
    targets = load_targets(csv_path)

    print(f"Loaded {len(trajectories)} QTM trajectories, {len(targets)} throw announcements")

    # Apply throw exclusions
    excluded = {normalize_label(x) for x in exclude} if exclude else set()
    if excluded:
        all_labels = set(trajectories.keys()) | {t['label'] for t in targets}
        n_traj_before = len(trajectories)
        n_targ_before = len(targets)
        trajectories = {k: v for k, v in trajectories.items() if k not in excluded}
        targets = [t for t in targets if t['label'] not in excluded]
        n_removed = (n_traj_before - len(trajectories)) + (n_targ_before - len(targets))
        if n_removed:
            print(f"  Excluded throws: {', '.join(sorted(excluded & all_labels))}")
        never_existed = excluded - all_labels
        if never_existed:
            print(f"  Warning: these labels were not found in the data: {', '.join(sorted(never_existed))}")
        print(f"  Remaining: {len(trajectories)} trajectories, {len(targets)} targets")

    # Clock alignment
    clock_offset = compute_clock_offset(targets, trajectories, freq)
    if clock_offset is None:
        return [], defaultdict(list), None

    # Analyse each throw
    results = []
    for t in targets:
        label = t['label']
        if label not in trajectories:
            print(f"  {label}: no QTM data — skipping")
            continue

        d = trajectories[label]
        release_frame = find_release_frame(d['values'], d['frames'], freq)
        landing_frame, landing_xyz = find_landing_frame(d['values'], d['frames'], PLANE_Z)

        if landing_frame is None:
            print(f"  {label}: no target plane crossing — skipping")
            continue

        # Actual landing time (in ROS epoch)
        actual_landing_ros = landing_frame / freq + clock_offset

        # Predicted landing time (from announcement)
        predicted_landing_ros = t['throw_time'] + t['predicted_tof']

        # Landing time error (always computable if we have a landing)
        landing_err_ms = (actual_landing_ros - predicted_landing_ros) * 1000

        # Release-dependent metrics (may be None if tracking started mid-flight)
        if release_frame is not None:
            actual_release_ros = release_frame / freq + clock_offset
            actual_tof = (landing_frame - release_frame) / freq
            release_err_ms = (actual_release_ros - t['throw_time']) * 1000
            tof_err_ms = (actual_tof - t['predicted_tof']) * 1000
        else:
            actual_tof = None
            release_err_ms = None
            tof_err_ms = None

        # Spatial landing error
        spatial_err = np.sqrt(
            (landing_xyz[0] - t['target_x'])**2 +
            (landing_xyz[1] - t['target_y'])**2
        )

        results.append({
            'label': label,
            'target_id': t['target_id'],
            'target_x': t['target_x'],
            'target_y': t['target_y'],
            'predicted_tof': t['predicted_tof'],
            'actual_tof': actual_tof,
            'tof_err_ms': tof_err_ms,
            'release_err_ms': release_err_ms,
            'landing_err_ms': landing_err_ms,
            'spatial_err_mm': spatial_err,
            'landing_x': landing_xyz[0],
            'landing_y': landing_xyz[1],
            'has_release': release_frame is not None,
        })

    # Print results table
    print(f"\n{'Ball':<6} {'target':<12} {'pred_tof':>9} {'actual_tof':>10} "
          f"{'tof_err':>8} {'release_err':>11} {'landing_err':>11} {'spatial':>8}")
    print(f"{'':6} {'':12} {'(s)':>9} {'(s)':>10} {'(ms)':>8} {'(ms)':>11} {'(ms)':>11} {'(mm)':>8}")
    print("─" * 88)

    for r in results:
        tof_str = f"{r['tof_err_ms']:>+7.1f}" if r['tof_err_ms'] is not None else "    N/A"
        rel_str = f"{r['release_err_ms']:>+10.1f}" if r['release_err_ms'] is not None else "       N/A"
        atof_str = f"{r['actual_tof']:>9.4f}" if r['actual_tof'] is not None else "      N/A"
        note = "" if r['has_release'] else "  (no release detected)"
        print(f"  {r['label']:<6} {r['target_id']:<12} {r['predicted_tof']:>8.4f}  "
              f"{atof_str}  {tof_str}  "
              f"{rel_str}  {r['landing_err_ms']:>+10.1f}  "
              f"{r['spatial_err_mm']:>7.1f}{note}")

    # Summary by target
    print(f"\n{'─'*88}")
    by_target = defaultdict(list)
    for r in results:
        by_target[r['target_id']].append(r)

    for tid in sorted(by_target.keys()):
        rs = by_target[tid]
        tof_errs = [r['tof_err_ms'] for r in rs if r['tof_err_ms'] is not None]
        land_errs = [r['landing_err_ms'] for r in rs]
        sp_errs = [r['spatial_err_mm'] for r in rs]
        print(f"\n  {tid} → ({rs[0]['target_x']:.0f}, {rs[0]['target_y']:.0f}) mm  [{len(rs)} throws]")
        if tof_errs:
            print(f"    ToF error:     mean={np.mean(tof_errs):+.1f}ms  std={np.std(tof_errs):.1f}ms")
        else:
            print(f"    ToF error:     N/A (no release detected)")
        print(f"    Landing time:  mean={np.mean(land_errs):+.1f}ms  std={np.std(land_errs):.1f}ms")
        print(f"    Spatial error: mean={np.mean(sp_errs):.1f}mm  max={np.max(sp_errs):.1f}mm")

    all_tof = [r['tof_err_ms'] for r in results if r['tof_err_ms'] is not None]
    all_land = [r['landing_err_ms'] for r in results]
    print(f"\n  OVERALL ({len(results)} throws, {len(all_tof)} with release detection):")
    if all_tof:
        print(f"    ToF error:     mean={np.mean(all_tof):+.1f}ms  std={np.std(all_tof):.1f}ms")
    print(f"    Landing time:  mean={np.mean(all_land):+.1f}ms  std={np.std(all_land):.1f}ms")

    return results, by_target, clock_offset


# ── Plotting ────────────────────────────────────────────────────────────────


def plot_results(results, by_target, clock_offset, session_number=""):
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    # Have the title indicate the session number
    fig.suptitle(f"Ball Butler Timing Analysis\n Session {session_number}", fontsize=14, fontweight="bold")

    target_ids = sorted(by_target.keys())
    colors = plt.cm.Set1(np.linspace(0, 0.8, len(target_ids)))
    tid_color = {tid: colors[i] for i, tid in enumerate(target_ids)}

    # ── (a) Landing time error per throw ────────────────────────────────
    ax = axes[0, 0]
    for i, r in enumerate(results):
        c = tid_color[r['target_id']]
        ax.bar(i, r['landing_err_ms'], color=c, edgecolor='white', linewidth=0.5)
    ax.axhline(0, color='black', linewidth=0.8, linestyle='-')
    ax.set_xlabel("Throw index")
    ax.set_ylabel("Landing time error (ms)")
    ax.set_title("(a) Landing time: actual − predicted")

    # Add target group labels
    for tid in target_ids:
        indices = [i for i, r in enumerate(results) if r['target_id'] == tid]
        if indices:
            mid = np.mean(indices)
            ax.text(mid, ax.get_ylim()[1] * 0.9, tid.replace('volley_', 'V'),
                    ha='center', fontsize=8, color=tid_color[tid], fontweight='bold')

    # ── (b) ToF: predicted vs actual ────────────────────────────────────
    ax = axes[0, 1]
    for tid in target_ids:
        rs = by_target[tid]
        rs_with_tof = [r for r in rs if r['actual_tof'] is not None]
        if not rs_with_tof:
            continue
        predicted = [r['predicted_tof'] * 1000 for r in rs_with_tof]
        actual = [r['actual_tof'] * 1000 for r in rs_with_tof]
        ax.scatter(predicted, actual, c=[tid_color[tid]], s=50, label=tid,
                   edgecolors='black', linewidths=0.5, zorder=3)

    # Perfect prediction line
    rs_with_tof = [r for r in results if r['actual_tof'] is not None]
    if rs_with_tof:
        all_tof = [r['actual_tof'] * 1000 for r in rs_with_tof] + [r['predicted_tof'] * 1000 for r in rs_with_tof]
        lims = [min(all_tof) - 10, max(all_tof) + 10]
        ax.plot(lims, lims, 'k--', linewidth=0.8, alpha=0.5, label='perfect')
        ax.set_xlim(lims)
        ax.set_ylim(lims)
    ax.set_xlabel("Predicted ToF (ms)")
    ax.set_ylabel("Actual ToF (ms)")
    ax.set_title("(b) Time of flight: predicted vs actual")
    ax.legend(fontsize=8)
    ax.set_aspect('equal')

    # ── (c) Landing time error box plot by target ───────────────────────
    ax = axes[1, 0]
    data_for_box = []
    labels_for_box = []
    for tid in target_ids:
        rs = by_target[tid]
        data_for_box.append([r['landing_err_ms'] for r in rs])
        labels_for_box.append(tid)

    bp = ax.boxplot(data_for_box, tick_labels=labels_for_box, patch_artist=True,
                    widths=0.5, medianprops=dict(color='black', linewidth=2))
    for patch, tid in zip(bp['boxes'], target_ids):
        patch.set_facecolor(tid_color[tid])
        patch.set_alpha(0.7)

    # Overlay individual points
    for i, (tid, data) in enumerate(zip(target_ids, data_for_box)):
        x = np.random.normal(i + 1, 0.06, len(data))
        ax.scatter(x, data, c=[tid_color[tid]], s=25, edgecolors='black',
                   linewidths=0.5, zorder=3)

    ax.axhline(0, color='black', linewidth=0.8, linestyle='-')
    ax.set_ylabel("Landing time error (ms)")
    ax.set_title("(c) Landing time error by target")

    # ── (d) Spatial landing accuracy ────────────────────────────────────
    ax = axes[1, 1]
    for tid in target_ids:
        rs = by_target[tid]
        c = tid_color[tid]

        # Plot target position
        tx, ty = rs[0]['target_x'], rs[0]['target_y']
        ax.scatter(tx, ty, c=[c], s=150, marker='x', linewidths=3, zorder=5)

        # Plot landing positions
        lx = [r['landing_x'] for r in rs]
        ly = [r['landing_y'] for r in rs]
        ax.scatter(lx, ly, c=[c], s=30, alpha=0.7, edgecolors='black',
                   linewidths=0.5, zorder=3, label=tid)

        # Arrow from mean landing to target
        mean_lx, mean_ly = np.mean(lx), np.mean(ly)
        ax.annotate("", xy=(tx, ty), xytext=(mean_lx, mean_ly),
                    arrowprops=dict(arrowstyle="-|>", color=c, lw=1.5,
                                    shrinkA=3, shrinkB=5))

    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title("(d) Spatial accuracy (× = target, dots = landings)")
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    plt.tight_layout()
    plt.show()


# ── Entry Point ─────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="Ball Butler timing analysis.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("session", type=Path, help="QTM JSON trajectory file")
    parser.add_argument("targets", type=Path, help="Target CSV file with timestamps")
    parser.add_argument(
        "--exclude",
        nargs="+",
        default=None,
        metavar="LABEL",
        help="Throw labels to exclude from analysis (e.g. b3 b7 b12)",
    )
    args = parser.parse_args()

    results, by_target, clock_offset = analyse(
        args.session, args.targets, exclude=args.exclude,
    )
    if not results:
        print("No results to plot. Exiting.")
        return
    # Extract session number from the session path for better plot titles (assume the filename is 'session<number>.json')
    session_number = args.session.stem.replace("session", "")
    plot_results(results, by_target, clock_offset, session_number=session_number)


if __name__ == "__main__":
    main()