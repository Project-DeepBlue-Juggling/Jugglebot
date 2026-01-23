#!/usr/bin/env python3
"""
Rotation Axis Finder for Motion Capture Data

This script analyzes motion capture data from QTM (Qualisys Track Manager) to find
the axis of rotation for a rigid body rotating about a fixed axis.

Method:
Each marker traces a circular arc in 3D space. For a point rotating about a fixed axis:
1. The trajectory lies on a plane perpendicular to the rotation axis
2. The trajectory is a circular arc
3. The center of that circle lies on the rotation axis

We fit a circle to each marker's trajectory and find the common axis that best
explains all marker motions.
"""

import numpy as np
import pandas as pd
from scipy.optimize import minimize
from scipy.spatial.distance import cdist
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import warnings
import time

def load_qtm_tsv(filepath):
    """Load QTM TSV file and return marker data as a dictionary."""
    
    # Read header info
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    # Parse header
    header_info = {}
    for line in lines[:11]:
        parts = line.strip().split('\t')
        if len(parts) >= 2:
            header_info[parts[0]] = parts[1:]
    
    # Find the data start line (after MARKER_NAMES and column headers)
    # Column headers are on line 11 (0-indexed: line 10)
    df = pd.read_csv(filepath, sep='\t', skiprows=11, header=None)
    
    # Get marker names from header
    marker_names = header_info.get('MARKER_NAMES', [])
    
    # Build column names
    columns = ['Frame', 'Time']
    for name in marker_names:
        columns.extend([f'{name} X', f'{name} Y', f'{name} Z'])
    
    # Trim to match actual columns (there might be trailing empty column)
    df = df.iloc[:, :len(columns)]
    df.columns = columns
    
    return df, marker_names


def extract_marker_trajectories(df, marker_names):
    """
    Extract valid (non-zero) trajectories for each marker.
    Returns dict: marker_name -> Nx3 numpy array of positions
    """
    trajectories = {}
    
    for name in marker_names:
        x = df[f'{name} X'].values
        y = df[f'{name} Y'].values
        z = df[f'{name} Z'].values
        
        # Stack into Nx3 array
        positions = np.column_stack([x, y, z])
        
        # Filter out zero positions (missing data)
        # QTM uses exactly 0.000 for missing data
        valid_mask = ~((positions[:, 0] == 0) & 
                       (positions[:, 1] == 0) & 
                       (positions[:, 2] == 0))
        
        valid_positions = positions[valid_mask]
        
        if len(valid_positions) > 0:
            trajectories[name] = valid_positions
            
    return trajectories


def fit_plane(points):
    """
    Fit a plane to a set of 3D points using SVD.
    Returns: centroid, normal vector
    """
    centroid = np.mean(points, axis=0)
    centered = points - centroid
    
    # SVD - the normal is the singular vector with smallest singular value
    _, _, Vt = np.linalg.svd(centered)
    normal = Vt[-1]  # Last row of Vt
    
    return centroid, normal


def fit_circle_3d(points):
    """
    Fit a circle to 3D points that lie approximately on a plane.
    
    Returns:
        center: 3D center of the circle
        radius: radius of the circle
        normal: normal vector of the plane containing the circle
        residual: RMS error of fit
    """
    # First fit a plane to get the normal
    centroid, normal = fit_plane(points)
    
    # Project points onto the plane
    # Create local 2D coordinate system on the plane
    # Find two orthogonal vectors in the plane
    if abs(normal[0]) < 0.9:
        u = np.cross(normal, [1, 0, 0])
    else:
        u = np.cross(normal, [0, 1, 0])
    u = u / np.linalg.norm(u)
    v = np.cross(normal, u)
    v = v / np.linalg.norm(v)
    
    # Project points to 2D
    centered = points - centroid
    points_2d = np.column_stack([
        np.dot(centered, u),
        np.dot(centered, v)
    ])
    
    # Fit circle in 2D using algebraic method
    # Circle equation: (x-a)^2 + (y-b)^2 = r^2
    # Expanded: x^2 + y^2 - 2ax - 2by + (a^2 + b^2 - r^2) = 0
    # Let c = a^2 + b^2 - r^2
    # Then: -2ax - 2by + c = -(x^2 + y^2)
    
    x, y = points_2d[:, 0], points_2d[:, 1]
    A = np.column_stack([x, y, np.ones(len(x))])
    b = x**2 + y**2
    
    # Solve least squares
    result, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
    
    a_2d = result[0] / 2
    b_2d = result[1] / 2
    r_squared = result[2] + a_2d**2 + b_2d**2
    radius = np.sqrt(max(r_squared, 0))
    
    # Convert 2D center back to 3D
    center_3d = centroid + a_2d * u + b_2d * v
    
    # Calculate residual (RMS distance from circle)
    distances_to_center = np.sqrt((x - a_2d)**2 + (y - b_2d)**2)
    residual = np.sqrt(np.mean((distances_to_center - radius)**2))
    
    return center_3d, radius, normal, residual


def find_rotation_axis(trajectories):
    """
    Find the rotation axis from multiple marker trajectories.
    
    Each marker traces a circle around the axis. The axis passes through
    all circle centers and is parallel to all circle normals.
    
    For a rigid body rotating about a fixed axis:
    - Each marker traces a circle
    - All circle planes are perpendicular to the rotation axis
    - All circle centers lie ON the rotation axis
    
    Returns:
        axis_point: a point on the axis
        axis_direction: unit vector along the axis
        marker_results: dict with fit results for each marker
    """
    
    marker_results = {}
    centers = []
    normals = []
    weights = []  # Weight by number of points and inverse residual
    
    print("\nFitting circles to each marker trajectory:")
    print("-" * 60)
    
    for name, points in trajectories.items():
        if len(points) < 10:  # Need minimum points for reliable fit
            print(f"  {name}: Skipped (only {len(points)} points)")
            continue
            
        center, radius, normal, residual = fit_circle_3d(points)
        
        # Ensure consistent normal direction (all pointing same way)
        if len(normals) > 0 and np.dot(normal, normals[0]) < 0:
            normal = -normal
            
        marker_results[name] = {
            'center': center,
            'radius': radius,
            'normal': normal,
            'residual': residual,
            'n_points': len(points)
        }
        
        centers.append(center)
        normals.append(normal)
        
        # Weight by number of points and quality of fit
        weight = len(points) / (residual + 0.1)  # Add small value to avoid div by zero
        weights.append(weight)
        
        print(f"  {name}:")
        print(f"    Points: {len(points)}, Radius: {radius:.2f} mm")
        print(f"    Circle center: ({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f})")
        print(f"    Fit residual: {residual:.3f} mm")
    
    centers = np.array(centers)
    normals = np.array(normals)
    weights = np.array(weights)
    weights = weights / weights.sum()  # Normalize
    
    # The axis direction should be the average of all circle normals
    # (since each circle plane is perpendicular to the rotation axis)
    avg_normal = np.sum(normals * weights[:, np.newaxis], axis=0)
    avg_normal = avg_normal / np.linalg.norm(avg_normal)
    
    # The axis passes through all circle centers
    # Since all centers should lie on the axis, we use their weighted average
    avg_center = np.sum(centers * weights[:, np.newaxis], axis=0)
    
    # The axis direction is the average normal (perpendicular to all circle planes)
    axis_direction = avg_normal
    
    # Verify: check how well the centers lie on a line parallel to axis_direction
    # Project centers onto a plane perpendicular to axis to see scatter
    print("\n" + "-" * 60)
    print("Circle center alignment check:")
    
    # Calculate perpendicular distances from each center to the axis line
    for i, (name, result) in enumerate(marker_results.items()):
        center = result['center']
        # Vector from avg_center to this center
        v = center - avg_center
        # Component parallel to axis
        parallel = np.dot(v, axis_direction) * axis_direction
        # Perpendicular component (distance from axis)
        perp = v - parallel
        dist_from_axis = np.linalg.norm(perp)
        print(f"  {name}: {dist_from_axis:.3f} mm from axis line")
    
    return avg_center, axis_direction, marker_results


def find_axis_plane_intersection(axis_point, axis_direction, plane_point, plane_normal):
    """
    Find where the axis intersects a plane.
    
    Returns the intersection point, or None if axis is parallel to plane.
    """
    denom = np.dot(axis_direction, plane_normal)
    
    if abs(denom) < 1e-10:
        return None  # Axis parallel to plane
    
    t = np.dot(plane_point - axis_point, plane_normal) / denom
    intersection = axis_point + t * axis_direction
    
    return intersection


def analyze_rotation(df, marker_names):
    """Main analysis function."""
    
    # Extract trajectories
    trajectories = extract_marker_trajectories(df, marker_names)
    
    print(f"\nLoaded {len(trajectories)} markers with valid data:")
    for name, points in trajectories.items():
        print(f"  {name}: {len(points)} valid frames")
    
    # Find rotation axis
    axis_point, axis_direction, marker_results = find_rotation_axis(trajectories)
    
    print("\n" + "=" * 60)
    print("ROTATION AXIS RESULTS")
    print("=" * 60)
    
    print(f"\nAxis direction (unit vector):")
    print(f"  ({axis_direction[0]:.6f}, {axis_direction[1]:.6f}, {axis_direction[2]:.6f})")
    
    print(f"\nA point on the axis:")
    print(f"  ({axis_point[0]:.2f}, {axis_point[1]:.2f}, {axis_point[2]:.2f}) mm")
    
    # Find where axis passes through the approximate marker plane
    # Get all valid marker positions to define the plane
    all_points = np.vstack(list(trajectories.values()))
    plane_centroid, plane_normal = fit_plane(all_points)
    
    intersection = find_axis_plane_intersection(
        axis_point, axis_direction, plane_centroid, plane_normal
    )
    
    if intersection is not None:
        print(f"\nAxis intersection with marker plane:")
        print(f"  ({intersection[0]:.2f}, {intersection[1]:.2f}, {intersection[2]:.2f}) mm")
    
    # Calculate consistency metrics
    print("\n" + "-" * 60)
    print("Consistency check (normal alignment with axis):")
    for name, result in marker_results.items():
        dot_product = abs(np.dot(result['normal'], axis_direction))
        angle_deg = np.degrees(np.arccos(min(dot_product, 1.0)))
        print(f"  {name}: {angle_deg:.2f}° deviation from axis")
    
    # Calculate angular range
    print("\n" + "-" * 60)
    print("Rotation analysis:")
    
    for name, points in trajectories.items():
        if name not in marker_results:
            continue
            
        result = marker_results[name]
        center = result['center']
        normal = result['normal']
        
        # Project points to circle plane
        if abs(normal[0]) < 0.9:
            u = np.cross(normal, [1, 0, 0])
        else:
            u = np.cross(normal, [0, 1, 0])
        u = u / np.linalg.norm(u)
        v = np.cross(normal, u)
        
        centered = points - center
        angles = np.arctan2(np.dot(centered, v), np.dot(centered, u))
        
        # Unwrap angles and find range
        angles_unwrapped = np.unwrap(angles)
        angle_range = np.degrees(angles_unwrapped[-1] - angles_unwrapped[0])
        
        print(f"  {name}: {abs(angle_range):.1f}° rotation captured")
    
    return {
        'axis_point': axis_point,
        'axis_direction': axis_direction,
        'plane_intersection': intersection,
        'marker_results': marker_results
    }


def plot_analysis(trajectories, results, save_path=None):
    """
    Create a 3D visualization of the rotation axis analysis.
    
    Shows:
    - Marker trajectories (circular arcs)
    - Fitted circle centers
    - The rotation axis
    """
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Color palette for markers
    colors = ['#e41a1c', '#377eb8', '#4daf4a', '#984ea3', '#ff7f00']
    
    axis_point = results['axis_point']
    axis_direction = results['axis_direction']
    marker_results = results['marker_results']
    
    # Plot each marker trajectory
    for i, (name, points) in enumerate(trajectories.items()):
        color = colors[i % len(colors)]
        
        # Subsample if too many points (for cleaner visualization)
        if len(points) > 200:
            step = len(points) // 200
            points_plot = points[::step]
        else:
            points_plot = points
        
        # Plot trajectory
        ax.scatter(points_plot[:, 0], points_plot[:, 1], points_plot[:, 2], 
                   c=color, s=5, alpha=0.5, label=f'{name} trajectory')
        
        # Plot circle center if available
        if name in marker_results:
            center = marker_results[name]['center']
            ax.scatter([center[0]], [center[1]], [center[2]], 
                       c=color, s=150, marker='X', edgecolors='black', 
                       linewidths=1.5, zorder=5)
    
    # Plot the rotation axis
    # Extend axis line through the range of the data
    all_points = np.vstack(list(trajectories.values()))
    z_min, z_max = all_points[:, 2].min() - 20, all_points[:, 2].max() + 20
    
    # Calculate axis endpoints
    t_min = (z_min - axis_point[2]) / axis_direction[2] if abs(axis_direction[2]) > 1e-6 else -100
    t_max = (z_max - axis_point[2]) / axis_direction[2] if abs(axis_direction[2]) > 1e-6 else 100
    
    axis_start = axis_point + t_min * axis_direction
    axis_end = axis_point + t_max * axis_direction
    
    ax.plot([axis_start[0], axis_end[0]], 
            [axis_start[1], axis_end[1]], 
            [axis_start[2], axis_end[2]], 
            'k-', linewidth=3, label='Rotation axis', zorder=10)
    
    # Mark the axis point (intersection with marker plane)
    if results['plane_intersection'] is not None:
        pt = results['plane_intersection']
        ax.scatter([pt[0]], [pt[1]], [pt[2]], 
                   c='black', s=200, marker='o', zorder=10)
    
    # Draw fitted circles for each marker
    for i, (name, result) in enumerate(marker_results.items()):
        color = colors[i % len(colors)]
        center = result['center']
        radius = result['radius']
        normal = result['normal']
        
        # Create circle points
        theta = np.linspace(0, 2*np.pi, 100)
        
        # Create orthonormal basis in the circle plane
        if abs(normal[0]) < 0.9:
            u = np.cross(normal, [1, 0, 0])
        else:
            u = np.cross(normal, [0, 1, 0])
        u = u / np.linalg.norm(u)
        v = np.cross(normal, u)
        
        # Circle points
        circle_points = center + radius * (np.outer(np.cos(theta), u) + np.outer(np.sin(theta), v))
        
        ax.plot(circle_points[:, 0], circle_points[:, 1], circle_points[:, 2], 
                color=color, linestyle='--', linewidth=1.5, alpha=0.7)
    
    # Labels and formatting
    ax.set_xlabel('X (mm)', fontsize=12)
    ax.set_ylabel('Y (mm)', fontsize=12)
    ax.set_zlabel('Z (mm)', fontsize=12)
    ax.set_title('Rotation Axis Analysis\n(X = circle centers, dashed = fitted circles)', fontsize=14)
    
    # Create custom legend entries
    from matplotlib.lines import Line2D
    legend_elements = []
    for i, name in enumerate(trajectories.keys()):
        color = colors[i % len(colors)]
        legend_elements.append(Line2D([0], [0], marker='o', color='w', 
                                       markerfacecolor=color, markersize=8, label=name))
    legend_elements.append(Line2D([0], [0], marker='X', color='w', 
                                   markerfacecolor='gray', markersize=10, 
                                   markeredgecolor='black', label='Circle centers'))
    legend_elements.append(Line2D([0], [0], color='black', linewidth=3, label='Rotation axis'))
    
    ax.legend(handles=legend_elements, loc='upper left', fontsize=9)
    
    # Set equal aspect ratio for all axes
    max_range = np.array([
        all_points[:, 0].max() - all_points[:, 0].min(),
        all_points[:, 1].max() - all_points[:, 1].min(),
        all_points[:, 2].max() - all_points[:, 2].min()
    ]).max() / 2.0
    
    mid_x = (all_points[:, 0].max() + all_points[:, 0].min()) / 2
    mid_y = (all_points[:, 1].max() + all_points[:, 1].min()) / 2
    mid_z = (all_points[:, 2].max() + all_points[:, 2].min()) / 2
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    # Adjust viewing angle for better visualization
    ax.view_init(elev=25, azim=45)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\nPlot saved to: {save_path}")
    
    plt.show()
    
    return fig


if __name__ == "__main__":
    # Load the data
    filepath = "2026-01-09-Ball_Butler_Rotating_150_deg_trimmed.tsv"
    
    print("Loading QTM motion capture data...")
    df, marker_names = load_qtm_tsv(filepath)
    
    print(f"Loaded {len(df)} frames at 300 Hz")
    print(f"Markers: {marker_names}")
    
    start_time = time.time() * 1000  # milliseconds

    # Run analysis
    results = analyze_rotation(df, marker_names)
    
    # Also find intersection with horizontal plane at Z = 1725 (approx marker height)
    z_plane_point = np.array([0, 0, 1725])
    z_plane_normal = np.array([0, 0, 1])
    z_intersection = find_axis_plane_intersection(
        results['axis_point'], results['axis_direction'], z_plane_point, z_plane_normal
    )
    
    end_time = time.time() * 1000  # milliseconds
    print(f"\nAnalysis completed in {end_time - start_time:.2f} ms")

    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"\nRotation axis passes through the marker plane at:")
    print(f"  X = {results['plane_intersection'][0]:.2f} mm")
    print(f"  Y = {results['plane_intersection'][1]:.2f} mm") 
    print(f"  Z = {results['plane_intersection'][2]:.2f} mm")
    
    if z_intersection is not None:
        print(f"\nAxis intersection with Z = 1725 mm plane:")
        print(f"  X = {z_intersection[0]:.2f} mm")
        print(f"  Y = {z_intersection[1]:.2f} mm")
    
    print(f"\nAxis direction: ({results['axis_direction'][0]:.4f}, "
          f"{results['axis_direction'][1]:.4f}, {results['axis_direction'][2]:.4f})")
    
    # Calculate axis tilt from vertical
    vertical = np.array([0, 0, 1])
    tilt_angle = np.degrees(np.arccos(abs(np.dot(results['axis_direction'], vertical))))
    print(f"\nAxis tilt from vertical (Z): {tilt_angle:.2f}°")
    
    # Generate visualization
    trajectories = extract_marker_trajectories(df, marker_names)
    plot_analysis(trajectories, results, save_path=None)