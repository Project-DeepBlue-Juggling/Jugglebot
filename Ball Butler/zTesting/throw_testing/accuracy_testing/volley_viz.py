"""
Ball Butler Volley Throw Visualizer
====================================
Loads QTM JSON exports, stitches multiple recording sessions,
and renders all ball trajectories in an interactive 3D plot.

Optionally loads target CSVs to compute and display where each
trajectory intersects the target plane.

Usage:
    # 3D trajectories only
    python volley_viz.py session1.json session2.json

    # 3D trajectories + 2D target plane analysis (side-by-side)
    python volley_viz.py session1.json session2.json -t targets1.csv targets2.csv

    # Target plane analysis only (matplotlib, skip 3D)
    python volley_viz.py session1.json session2.json -t targets1.csv targets2.csv --no-3d

Dependencies:
    pip install pyvista numpy matplotlib
"""

import argparse
import csv
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib
import numpy as np
import pyvista as pv


# ── Configuration ───────────────────────────────────────────────────────────

GROUND_CONTACT_Z_MM = -10.0  # Truncate trajectory at first frame where Z <= this value
TRAJECTORY_OPACITY = 0.6
TRAJECTORY_LINE_WIDTH = 2.0
BACKGROUND_COLOR = "white"
GROUND_PLANE_COLOR = "lightgrey"
GROUND_PLANE_OPACITY = 0.3
TARGET_PLANE_COLOR = "lightblue"
TARGET_PLANE_OPACITY = 0.15
DEFAULT_TRAJECTORY_COLOR = [0.0, 0.8, 0.0]  # For throws with no target info

# Target plane analysis
TARGET_SPHERE_RADIUS = 15  # mm, for 3D target markers
INTERSECTION_SPHERE_RADIUS = 8  # mm, for 3D intersection markers
TARGET_DOT_SIZE = 80  # matplotlib scatter size (--no-3d mode)
INTERSECTION_DOT_SIZE = 30  # matplotlib scatter size (--no-3d mode)
TARGET_Z_TOLERANCE_MM = 5.0  # Targets within this tolerance of the mode Z are kept


# ── Data Structures ─────────────────────────────────────────────────────────


@dataclass
class TrajectorySegment:
    """A continuous tracked portion of a ball trajectory."""

    start_frame: int
    positions: np.ndarray  # Shape (N, 3) — X, Y, Z in mm
    residuals: np.ndarray  # Shape (N,) — QTM reconstruction residual


@dataclass
class BallTrajectory:
    """Full trajectory for one throw, possibly with tracking gaps."""

    name: str
    segments: list[TrajectorySegment] = field(default_factory=list)


@dataclass
class TargetInfo:
    """Target position for a single throw."""

    label: str
    target_id: str
    x: float
    y: float
    z: float


# ── Loading & Parsing ───────────────────────────────────────────────────────


def load_session(filepath: str | Path) -> list[BallTrajectory]:
    """Load a QTM JSON export and return a list of BallTrajectory objects."""
    filepath = Path(filepath)
    print(f"Loading {filepath.name}...")

    with open(filepath) as f:
        data = json.load(f)

    frequency = data["Timebase"]["Frequency"]
    print(f"  Frequency: {frequency} Hz")
    print(f"  Markers:   {len(data['Markers'])}")

    trajectories = []
    for marker in data["Markers"]:
        traj = BallTrajectory(name=marker["Name"])

        for part in marker["Parts"]:
            values = np.array(part["Values"])  # Shape (N, 4): X, Y, Z, residual
            if values.size == 0:
                continue

            segment = TrajectorySegment(
                start_frame=part["Range"]["Start"],
                positions=values[:, :3],
                residuals=values[:, 3],
            )
            traj.segments.append(segment)

        if traj.segments:
            trajectories.append(traj)

    print(f"  Loaded {len(trajectories)} trajectories with data")
    return trajectories


def load_targets(filepath: str | Path) -> list[TargetInfo]:
    """Load target positions from a CSV file."""
    filepath = Path(filepath)
    print(f"Loading targets from {filepath.name}...")

    targets = []
    with open(filepath, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            targets.append(
                TargetInfo(
                    label=row["label"],
                    target_id=row["target_id"],
                    x=float(row["target_x"]),
                    y=float(row["target_y"]),
                    z=float(row["target_z"]),
                )
            )

    print(f"  Loaded {len(targets)} target entries")
    return targets


def stitch_sessions(session_files: list[str | Path]) -> list[BallTrajectory]:
    """Load multiple sessions and combine them, renumbering markers to avoid
    name collisions (both sessions start at b0)."""
    all_trajectories = []
    offset = 0

    for filepath in session_files:
        session = load_session(filepath)
        for traj in session:
            # Renumber: extract the numeric part and add offset
            num = int(traj.name.removeprefix("b"))
            traj.name = f"b{num + offset}"
            all_trajectories.append(traj)
        offset += len(session)

    print(f"\nTotal trajectories after stitching: {len(all_trajectories)}")
    return all_trajectories


def stitch_targets(target_files: list[str | Path]) -> tuple[list[TargetInfo], dict[str, int]]:
    """Load multiple target CSVs and combine them, renumbering labels.

    Returns:
        all_targets: combined list of TargetInfo with renumbered labels
        label_to_session: {ball_label: session_index} (0-based)
    """
    all_targets = []
    label_to_session = {}
    offset = 0

    for session_idx, filepath in enumerate(target_files):
        targets = load_targets(filepath)
        for t in targets:
            num = int(t.label.removeprefix("b"))
            t.label = f"b{num + offset}"
            all_targets.append(t)
            label_to_session[t.label] = session_idx
        offset += len(targets)

    print(f"Total targets after stitching: {len(all_targets)}")
    return all_targets, label_to_session


# ── Colour Mapping ──────────────────────────────────────────────────────────


def build_color_map(targets: list[TargetInfo]) -> tuple[dict[str, np.ndarray], dict[str, str]]:
    """Build a rainbow colour map for target IDs and a label-to-target lookup.

    Returns:
        target_id_colors: {target_id: np.array([r, g, b])} with values 0–1
        label_to_target:  {ball_label: target_id}
    """
    # Get sorted unique target IDs for deterministic colour assignment
    unique_ids = sorted(set(t.target_id for t in targets))
    n = len(unique_ids)

    # Sample a rainbow (hsv) colourmap evenly
    cmap = matplotlib.colormaps.get_cmap("hsv").resampled(n + 1)  # +1 to avoid red→red wrap
    target_id_colors = {}
    for i, tid in enumerate(unique_ids):
        target_id_colors[tid] = np.array(cmap(i / max(n, 1))[:3])

    # Build label → target_id lookup
    label_to_target = {t.label: t.target_id for t in targets}

    return target_id_colors, label_to_target


# ── Trajectory Processing ───────────────────────────────────────────────────


def truncate_at_ground(
    trajectory: BallTrajectory,
    z_threshold: float = GROUND_CONTACT_Z_MM,
) -> BallTrajectory:
    """Truncate a trajectory at the first frame where Z <= z_threshold.
    Returns a new BallTrajectory (segments after ground contact are dropped,
    and the contact segment is trimmed)."""
    truncated_segments = []

    for seg in trajectory.segments:
        z_values = seg.positions[:, 2]
        below = np.where(z_values <= z_threshold)[0]

        if len(below) > 0:
            # Truncate at first ground contact (inclusive)
            cut = below[0] + 1
            truncated_segments.append(
                TrajectorySegment(
                    start_frame=seg.start_frame,
                    positions=seg.positions[:cut],
                    residuals=seg.residuals[:cut],
                )
            )
            break  # Ignore any segments after ground contact
        else:
            truncated_segments.append(seg)

    return BallTrajectory(name=trajectory.name, segments=truncated_segments)


# ── Target Plane Intersection ───────────────────────────────────────────────


def infer_target_plane_z(targets: list[TargetInfo], tolerance: float = TARGET_Z_TOLERANCE_MM) -> float:
    """Infer the target plane Z height from the targets.
    Uses the rounded mode, then reports outliers beyond tolerance."""
    z_values = np.array([t.z for t in targets])

    # Round to nearest mm to handle floating-point noise, then take mode
    z_rounded = np.round(z_values)
    unique, counts = np.unique(z_rounded, return_counts=True)
    mode_z = unique[np.argmax(counts)]

    n_outliers = np.sum(np.abs(z_values - mode_z) > tolerance)
    if n_outliers > 0:
        print(f"  Warning: {n_outliers} target(s) have Z outside {mode_z} ± {tolerance} mm and will be ignored")

    print(f"  Inferred target plane Z = {mode_z:.1f} mm")
    return float(mode_z)


def filter_targets_to_plane(
    targets: list[TargetInfo], plane_z: float, tolerance: float = TARGET_Z_TOLERANCE_MM
) -> list[TargetInfo]:
    """Keep only targets whose Z is within tolerance of the plane."""
    return [t for t in targets if abs(t.z - plane_z) <= tolerance]


def find_plane_intersection(
    trajectory: BallTrajectory, plane_z: float
) -> np.ndarray | None:
    """Find where a trajectory first crosses the target plane going downward.
    Returns [x, y] at the interpolated crossing point, or None if no crossing found."""
    for seg in trajectory.segments:
        z = seg.positions[:, 2]

        for i in range(len(z) - 1):
            # Descending crossing: Z[i] > plane_z and Z[i+1] <= plane_z
            if z[i] > plane_z and z[i + 1] <= plane_z:
                # Linear interpolation factor
                dz = z[i] - z[i + 1]
                if dz == 0:
                    t = 0.5
                else:
                    t = (z[i] - plane_z) / dz

                # Interpolate X and Y
                p0 = seg.positions[i]
                p1 = seg.positions[i + 1]
                crossing = p0 + t * (p1 - p0)
                return crossing[:2]  # [x, y]

    return None


def compute_all_intersections(
    trajectories: list[BallTrajectory], plane_z: float
) -> dict[str, np.ndarray]:
    """Compute plane intersections for all trajectories.
    Returns {label: np.array([x, y])} for trajectories that cross the plane."""
    intersections = {}
    n_missed = 0

    for traj in trajectories:
        xy = find_plane_intersection(traj, plane_z)
        if xy is not None:
            intersections[traj.name] = xy
        else:
            n_missed += 1

    print(f"  Plane intersections found: {len(intersections)} / {len(trajectories)}")
    if n_missed > 0:
        print(f"  ({n_missed} trajectories did not cross the target plane)")

    return intersections


# ── Cluster Identification & Mapping ────────────────────────────────────────


def cluster_intersections(
    intersections: dict[str, np.ndarray],
    n_clusters: int,
) -> dict[int, list[str]]:
    """Cluster intersection points using k-means.

    Returns:
        {cluster_index: [list of ball labels in that cluster]}
    """
    from sklearn.cluster import KMeans

    labels_list = list(intersections.keys())
    points = np.array([intersections[l] for l in labels_list])

    kmeans = KMeans(n_clusters=n_clusters, n_init=10, random_state=42)
    kmeans.fit(points)

    clusters: dict[int, list[str]] = {}
    for label, cluster_id in zip(labels_list, kmeans.labels_):
        clusters.setdefault(int(cluster_id), []).append(label)

    return clusters


def render_identification_plot(
    intersections: dict[str, np.ndarray],
    clusters: dict[int, list[str]],
    valid_targets: list[TargetInfo],
    plane_z: float,
) -> None:
    """Show a numbered plot for manual cluster-to-target identification.

    Clusters are numbered C0, C1, ... with labels at their centroids.
    Targets are numbered T0, T1, ... with labels at their positions.
    """
    import matplotlib.pyplot as plt

    # Deduplicate targets
    seen = {}
    for t in valid_targets:
        if t.target_id not in seen:
            seen[t.target_id] = t
    unique_targets = list(seen.values())

    # Assign distinct colours to clusters
    n_clusters = len(clusters)
    cmap = matplotlib.colormaps.get_cmap("tab20").resampled(max(n_clusters, 1))

    fig, ax = plt.subplots(figsize=(12, 12))

    # Plot each cluster with a different colour and label at centroid
    for ci, (cluster_id, members) in enumerate(sorted(clusters.items())):
        color = cmap(ci / max(n_clusters - 1, 1))[:3]
        pts = np.array([intersections[l] for l in members])
        ax.scatter(pts[:, 0], pts[:, 1], c=[color], s=20, alpha=0.5)

        # Centroid label
        cx, cy = pts.mean(axis=0)
        ax.annotate(
            f"C{ci}", (cx, cy),
            fontsize=14, fontweight="bold", color=color,
            ha="center", va="center",
            bbox=dict(boxstyle="round,pad=0.3", fc="white", ec=color, alpha=0.8),
            zorder=10,
        )

    # Plot targets with numbered labels
    for ti, t in enumerate(unique_targets):
        ax.scatter(t.x, t.y, c="red", s=120, marker="x", linewidths=2.5, zorder=11)
        ax.annotate(
            f"T{ti}", (t.x, t.y),
            fontsize=12, fontweight="bold", color="red",
            textcoords="offset points", xytext=(10, 10),
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="red", alpha=0.8),
            zorder=12,
        )

    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title(f"Cluster Identification — Z = {plane_z:.0f} mm\n"
                 f"Match each C# to a T# and fill in the mapping file")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

    return unique_targets


def save_mapping_template(
    clusters: dict[int, list[str]],
    unique_targets: list[TargetInfo],
    output_path: Path,
) -> None:
    """Save a JSON template for the user to fill in the cluster→target mapping.

    The file contains:
    - targets: list of {index, target_id, x, y} for reference
    - clusters: list of {index, centroid_x, centroid_y, target_index: null}
    - The user fills in target_index for each cluster
    """
    target_entries = []
    for ti, t in enumerate(unique_targets):
        target_entries.append({
            "index": ti,
            "label": f"T{ti}",
            "target_id": t.target_id,
            "x": round(t.x, 1),
            "y": round(t.y, 1),
        })

    cluster_entries = []
    for ci, (cluster_id, members) in enumerate(sorted(clusters.items())):
        # We don't have intersections here, so store member count
        cluster_entries.append({
            "index": ci,
            "label": f"C{ci}",
            "num_throws": len(members),
            "target_index": None,  # ← USER FILLS THIS IN
        })

    template = {
        "_instructions": (
            "For each cluster, set 'target_index' to the T# index of the "
            "target it corresponds to. For example, if cluster C3 should map "
            "to target T7, set target_index to 7 for the C3 entry."
        ),
        "targets": target_entries,
        "clusters": cluster_entries,
    }

    with open(output_path, "w") as f:
        json.dump(template, f, indent=2)

    print(f"\nMapping template saved to: {output_path}")
    print("Edit this file to set 'target_index' for each cluster, then re-run with --mapping")


def load_mapping(
    mapping_path: Path,
    clusters: dict[int, list[str]],
    valid_targets: list[TargetInfo],
) -> dict[str, str]:
    """Load a completed mapping file and return {ball_label: target_id}.

    The mapping file specifies which target each cluster corresponds to.
    """
    with open(mapping_path) as f:
        data = json.load(f)

    # Deduplicate targets in same order as save_mapping_template
    seen = {}
    unique_targets = []
    for t in valid_targets:
        if t.target_id not in seen:
            seen[t.target_id] = t
            unique_targets.append(t)

    # Build cluster_index → target_id from the mapping
    label_to_target = {}
    for entry in data["clusters"]:
        ci = entry["index"]
        ti = entry["target_index"]
        if ti is None:
            print(f"  Warning: cluster C{ci} has no target_index assigned — skipping")
            continue
        if ti < 0 or ti >= len(unique_targets):
            print(f"  Warning: cluster C{ci} has invalid target_index {ti} — skipping")
            continue

        target_id = unique_targets[ti].target_id

        # Find the members of this cluster
        # clusters dict is keyed by original k-means label; sorted order = ci
        sorted_keys = sorted(clusters.keys())
        if ci >= len(sorted_keys):
            print(f"  Warning: cluster C{ci} not found — skipping")
            continue
        original_key = sorted_keys[ci]
        for ball_label in clusters[original_key]:
            label_to_target[ball_label] = target_id

    n_mapped = len(label_to_target)
    n_targets = len(set(label_to_target.values()))
    print(f"  Mapping loaded: {n_mapped} throws → {n_targets} targets")
    return label_to_target


# ── Visualization Helpers ───────────────────────────────────────────────────


def build_polyline(points: np.ndarray) -> pv.PolyData:
    """Create a PyVista polyline from an (N, 3) array of points."""
    n = len(points)
    if n < 2:
        return None
    cells = np.concatenate([[n], np.arange(n)]).astype(np.int64)
    poly = pv.PolyData(points, lines=cells)
    return poly


def _get_traj_color(
    traj_name: str,
    target_id_colors: dict[str, np.ndarray] | None,
    label_to_target: dict[str, str] | None,
) -> list[float]:
    """Look up the colour for a trajectory from the target colour map."""
    if target_id_colors and label_to_target and traj_name in label_to_target:
        tid = label_to_target[traj_name]
        if tid in target_id_colors:
            return target_id_colors[tid].tolist()
    return DEFAULT_TRAJECTORY_COLOR


def _add_trajectories_to_plotter(
    plotter: pv.Plotter,
    processed: list[BallTrajectory],
    target_id_colors: dict[str, np.ndarray] | None = None,
    label_to_target: dict[str, str] | None = None,
) -> np.ndarray | None:
    """Add trajectory lines to a plotter. Returns stacked array of all points, or None."""
    all_points = []

    for traj in processed:
        color = _get_traj_color(traj.name, target_id_colors, label_to_target)

        for seg in traj.segments:
            if len(seg.positions) < 2:
                continue
            poly = build_polyline(seg.positions)
            if poly is not None:
                plotter.add_mesh(
                    poly,
                    color=color,
                    line_width=TRAJECTORY_LINE_WIDTH,
                    opacity=TRAJECTORY_OPACITY,
                    render_lines_as_tubes=False,
                )
                all_points.append(seg.positions)

    if not all_points:
        return None
    return np.vstack(all_points)


def _add_ground_plane(plotter: pv.Plotter, all_points: np.ndarray, z: float, pad: float) -> None:
    """Add a translucent ground plane."""
    x_range = (all_points[:, 0].max() - all_points[:, 0].min())
    y_range = (all_points[:, 1].max() - all_points[:, 1].min())
    x_center = (all_points[:, 0].max() + all_points[:, 0].min()) / 2
    y_center = (all_points[:, 1].max() + all_points[:, 1].min()) / 2

    ground = pv.Plane(
        center=(x_center, y_center, z),
        direction=(0, 0, 1),
        i_size=x_range + 2 * pad,
        j_size=y_range + 2 * pad,
    )
    plotter.add_mesh(
        ground,
        color=GROUND_PLANE_COLOR,
        opacity=GROUND_PLANE_OPACITY,
    )


def _add_target_plane(
    plotter: pv.Plotter,
    all_points: np.ndarray,
    plane_z: float,
    pad: float,
    valid_targets: list[TargetInfo] | None = None,
    target_id_colors: dict[str, np.ndarray] | None = None,
) -> None:
    """Add a translucent target plane and coloured target markers."""
    x_range = (all_points[:, 0].max() - all_points[:, 0].min())
    y_range = (all_points[:, 1].max() - all_points[:, 1].min())
    x_center = (all_points[:, 0].max() + all_points[:, 0].min()) / 2
    y_center = (all_points[:, 1].max() + all_points[:, 1].min()) / 2

    plane = pv.Plane(
        center=(x_center, y_center, plane_z),
        direction=(0, 0, 1),
        i_size=x_range + 2 * pad,
        j_size=y_range + 2 * pad,
    )
    plotter.add_mesh(
        plane,
        color=TARGET_PLANE_COLOR,
        opacity=TARGET_PLANE_OPACITY,
    )

    # Add target markers as coloured spheres
    if valid_targets and target_id_colors:
        seen = {}
        for t in valid_targets:
            if t.target_id not in seen:
                seen[t.target_id] = t
        for tid, t in seen.items():
            color = target_id_colors.get(tid, np.array([1.0, 0, 0])).tolist()
            sphere = pv.Sphere(radius=TARGET_SPHERE_RADIUS, center=(t.x, t.y, plane_z))
            plotter.add_mesh(sphere, color=color, opacity=0.9)



# ── Main Render Functions ───────────────────────────────────────────────────


def render_3d_only(
    trajectories: list[BallTrajectory],
    z_threshold: float = GROUND_CONTACT_Z_MM,
) -> None:
    """Render only the 3D trajectory view (no target data)."""
    processed = [truncate_at_ground(t, z_threshold) for t in trajectories]
    processed = [t for t in processed if any(len(s.positions) >= 2 for s in t.segments)]
    print(f"Rendering {len(processed)} trajectories...")

    plotter = pv.Plotter(window_size=(1600, 1000))
    plotter.set_background(BACKGROUND_COLOR)

    all_points = _add_trajectories_to_plotter(plotter, processed)
    if all_points is None:
        print("No trajectory data to render!")
        return

    pad = 0.1 * max((all_points[:, 0].max() - all_points[:, 0].min()), (all_points[:, 1].max() - all_points[:, 1].min()))
    _add_ground_plane(plotter, all_points, z_threshold, pad)

    plotter.add_axes(xlabel="X", ylabel="Y", zlabel="Z")
    plotter.add_text(
        f"Ball Butler — {len(processed)} trajectories",
        position="upper_left", font_size=12, color="black",
    )
    plotter.camera_position = "xz"
    plotter.reset_camera()

    print("Displaying — use mouse to orbit, scroll to zoom, middle-click to pan.")
    plotter.show()


def render_3d_with_targets(
    trajectories: list[BallTrajectory],
    valid_targets: list[TargetInfo],
    plane_z: float,
    target_id_colors: dict[str, np.ndarray],
    label_to_target: dict[str, str],
    z_threshold: float = GROUND_CONTACT_Z_MM,
) -> None:
    """Render 3D trajectories coloured by target, with target plane and markers."""

    processed = [truncate_at_ground(t, z_threshold) for t in trajectories]
    processed = [t for t in processed if any(len(s.positions) >= 2 for s in t.segments)]
    print(f"Rendering {len(processed)} trajectories...")

    plotter = pv.Plotter(window_size=(1600, 1000))
    plotter.set_background(BACKGROUND_COLOR)

    all_points = _add_trajectories_to_plotter(
        plotter, processed, target_id_colors, label_to_target,
    )
    if all_points is None:
        print("No trajectory data to render!")
        return

    pad = 0.1 * max((all_points[:, 0].max() - all_points[:, 0].min()), (all_points[:, 1].max() - all_points[:, 1].min()))
    _add_ground_plane(plotter, all_points, z_threshold, pad)
    _add_target_plane(plotter, all_points, plane_z, pad, valid_targets, target_id_colors)

    plotter.add_axes(xlabel="X", ylabel="Y", zlabel="Z")
    plotter.add_text(
        f"Ball Butler — {len(processed)} trajectories",
        position="upper_left", font_size=12, color="black",
    )
    plotter.camera_position = "xz"
    plotter.reset_camera()

    print("Displaying 3D view — close this window to open the target plane plot.")
    plotter.show()


def render_target_plane_matplotlib(
    targets: list[TargetInfo],
    intersections: dict[str, np.ndarray],
    plane_z: float,
    target_id_colors: dict[str, np.ndarray],
    label_to_target: dict[str, str],
    label_to_session: dict[str, int] | None = None,
) -> None:
    """Render the 2D target plane view in matplotlib (used with --no-3d)."""
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    # Marker style per session (cycles if >2 sessions)
    SESSION_MARKERS = ["o", "^", "s", "D", "v", "P"]

    fig, ax = plt.subplots(figsize=(10, 10))

    # Group intersections by target_id
    intersections_by_target: dict[str, list[np.ndarray]] = {}
    for label, xy in intersections.items():
        tid = label_to_target.get(label)
        if tid:
            intersections_by_target.setdefault(tid, []).append(xy)

    # Plot individual intersection points (low opacity), marker varies by session
    if intersections:
        for label, xy in intersections.items():
            tid = label_to_target.get(label)
            color = target_id_colors.get(tid, np.array([0.5, 0.5, 0.5])) if tid else np.array([0.5, 0.5, 0.5])
            session = label_to_session.get(label, 0) if label_to_session else 0
            marker = SESSION_MARKERS[session % len(SESSION_MARKERS)]
            ax.scatter(xy[0], xy[1], c=[color], s=INTERSECTION_DOT_SIZE, alpha=0.3,
                       marker=marker, zorder=2)

    # Plot average landing points (larger, opaque) and error arrows
    unique_targets = {}
    for t in targets:
        if t.target_id not in unique_targets:
            unique_targets[t.target_id] = t

    for tid, points in intersections_by_target.items():
        color = target_id_colors.get(tid, np.array([0.5, 0.5, 0.5]))
        pts = np.array(points)
        mean_x, mean_y = pts.mean(axis=0)

        # Average landing dot
        ax.scatter(
            mean_x, mean_y,
            c=[color], s=INTERSECTION_DOT_SIZE * 4, alpha=1.0,
            edgecolors="black", linewidths=0.5,
            zorder=4,
        )

        # Arrow from average landing to target
        if tid in unique_targets:
            t = unique_targets[tid]
            dx = t.x - mean_x
            dy = t.y - mean_y
            ax.annotate(
                "",
                xy=(t.x, t.y),               # arrow tip (target)
                xytext=(mean_x, mean_y),      # arrow tail (average landing)
                arrowprops=dict(
                    arrowstyle="-|>",
                    color=color * 0.7,         # slightly darker
                    lw=1.5,
                    shrinkA=5, shrinkB=5,
                ),
                zorder=3,
            )

    # Plot target positions (coloured ×, on top), deduplicated by target_id
    for tid, t in unique_targets.items():
        color = target_id_colors.get(tid, np.array([0.5, 0.5, 0.5]))
        ax.scatter(t.x, t.y, c=[color], s=TARGET_DOT_SIZE, marker="x", linewidths=2, zorder=5)

    # Legend with proxy artists
    # Determine how many sessions there are
    if label_to_session:
        n_sessions = max(label_to_session.values()) + 1
    else:
        n_sessions = 1

    legend_elements = [
        Line2D([0], [0], marker="x", color="grey", markeredgewidth=2,
               linestyle="None", markersize=10,
               label=f"Targets ({len(unique_targets)})"),
    ]
    for s in range(n_sessions):
        m = SESSION_MARKERS[s % len(SESSION_MARKERS)]
        legend_elements.append(
            Line2D([0], [0], marker=m, color="grey", markerfacecolor="grey",
                   linestyle="None", markersize=7, alpha=0.3,
                   label=f"Session {s + 1} landings"),
        )
    legend_elements.extend([
        Line2D([0], [0], marker="o", color="grey", markerfacecolor="grey",
               markeredgecolor="black", markeredgewidth=0.5,
               linestyle="None", markersize=10,
               label=f"Mean landing ({len(intersections_by_target)})"),
        Line2D([0], [0], color="grey", lw=1.5,
               label="Error (mean → target)"),
    ])
    ax.legend(handles=legend_elements)

    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title(f"Target Plane: Z = {plane_z:.0f} mm")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


# ── Entry Point ─────────────────────────────────────────────────────────────


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Ball Butler trajectory visualizer and target plane analysis.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
examples:
  # 3D view only
  python volley_viz.py session1.json session2.json

  # 3D + target plane
  python volley_viz.py session1.json session2.json -t targets1.csv targets2.csv

  # Step 1: Identify clusters — generates mapping template
  python volley_viz.py session1.json session2.json -t targets1.csv targets2.csv --identify

  # Step 2: Apply your mapping
  python volley_viz.py session1.json session2.json -t targets1.csv targets2.csv --mapping mapping.json

  # Target plane only (matplotlib, no 3D)
  python volley_viz.py session1.json session2.json -t targets1.csv targets2.csv --mapping mapping.json --no-3d
""",
    )
    parser.add_argument(
        "sessions",
        nargs="+",
        type=Path,
        help="QTM JSON trajectory files (one per recording session)",
    )
    parser.add_argument(
        "-t", "--targets",
        nargs="+",
        type=Path,
        default=None,
        help="Target CSV files (one per recording session, same order as sessions)",
    )
    parser.add_argument(
        "--identify",
        action="store_true",
        help="Show numbered cluster/target plot and save a mapping template JSON",
    )
    parser.add_argument(
        "--mapping",
        type=Path,
        default=None,
        help="Path to a completed cluster→target mapping JSON file",
    )
    parser.add_argument(
        "--no-3d",
        action="store_true",
        help="Skip the 3D view; show target plane in matplotlib instead",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # Validate files exist
    for f in args.sessions:
        if not f.exists():
            print(f"Error: File not found: {f}")
            sys.exit(1)

    if args.targets:
        for f in args.targets:
            if not f.exists():
                print(f"Error: File not found: {f}")
                sys.exit(1)

    # Load trajectories
    trajectories = stitch_sessions(args.sessions)

    if args.targets:
        # ── With targets ────────────────────────────────────────────────
        print("\n── Target Plane Analysis ──────────────────────────")
        all_targets, label_to_session = stitch_targets(args.targets)

        plane_z = infer_target_plane_z(all_targets)
        valid_targets = filter_targets_to_plane(all_targets, plane_z)

        processed = [truncate_at_ground(t) for t in trajectories]
        intersections = compute_all_intersections(processed, plane_z)

        # Determine unique target count (for clustering)
        unique_target_ids = sorted(set(t.target_id for t in valid_targets))
        n_unique_targets = len(unique_target_ids)

        if args.identify:
            # ── Identification mode: cluster, show plot, save template ──
            print(f"\n  Clustering into {n_unique_targets} groups...")
            clusters = cluster_intersections(intersections, n_unique_targets)
            unique_targets = render_identification_plot(
                intersections, clusters, valid_targets, plane_z,
            )

            template_path = Path("cluster_mapping.json")
            save_mapping_template(clusters, unique_targets, template_path)
            return

        if args.mapping:
            # ── Mapping mode: load user's cluster→target mapping ────────
            print(f"\n  Loading mapping from {args.mapping.name}...")
            clusters = cluster_intersections(intersections, n_unique_targets)
            label_to_target = load_mapping(args.mapping, clusters, valid_targets)
        else:
            # ── Default: use label-index-based mapping (may be inaccurate) ──
            _, label_to_target = build_color_map(valid_targets)

        target_id_colors, _ = build_color_map(valid_targets)

        if args.no_3d:
            render_target_plane_matplotlib(
                valid_targets, intersections, plane_z,
                target_id_colors, label_to_target, label_to_session,
            )
        else:
            # Show 3D first (blocking), then matplotlib after it's closed
            render_3d_with_targets(
                trajectories, valid_targets, plane_z,
                target_id_colors, label_to_target,
            )
            print("\nOpening target plane plot...")
            render_target_plane_matplotlib(
                valid_targets, intersections, plane_z,
                target_id_colors, label_to_target, label_to_session,
            )
    else:
        # ── No targets: 3D only with default colours ────────────────────
        if args.no_3d:
            print("Nothing to display: --no-3d requires --targets")
            sys.exit(1)
        render_3d_only(trajectories)


if __name__ == "__main__":
    main()