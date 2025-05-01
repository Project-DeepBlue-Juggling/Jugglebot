#!/usr/bin/env python3
"""
Visualize a platform calibration KD-tree from a .joblib file.

- XY scatter of commanded positions
- Arrow directions = tilt direction (from rotation vector)
- Arrow colors = tilt magnitude
- (Optional) show pose errors as vectors
"""
import joblib
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# --- SETTINGS ---
folder_path = '/home/jetson/Desktop/Jugglebot/experimenting/platform_calibration/calibration_grid/'
file_name = 'platform_calibration_kdtree.joblib'
JOBLIB_FILE = folder_path + file_name
ROT_SCALE = 100.0                      # << should match build scale
SHOW_ERRORS = True                     # << toggle to see pose errors
ERROR_SCALE = 5.0                      # << error arrow magnification
FIGSIZE = (10, 8)

# ----------------

def load_calib(joblib_path):
    data = joblib.load(joblib_path)
    return data['features'], data['errors']

def decode_features(features):
    """Undo the feature scaling to get usable pose info."""
    xyz = features[:, :3]
    rotvec_scaled = features[:, 3:]
    rotvec = rotvec_scaled / ROT_SCALE
    return xyz, rotvec

def plot_calib(xyz, rotvec, errors):
    fig, ax = plt.subplots(figsize=FIGSIZE)

    # Compute rotation magnitudes (deg) and tilt directions
    tilt_mags = np.linalg.norm(rotvec, axis=1) * 180/np.pi  # degrees
    tilt_dirs = np.arctan2(rotvec[:,1], rotvec[:,0])         # radians: atan2(y, x)

    # Quiver plot: position → tilt direction
    q = ax.quiver(
        xyz[:,0], xyz[:,1],                     # positions
        np.cos(tilt_dirs), np.sin(tilt_dirs),    # tilt unit vectors
        tilt_mags,                              # color = tilt magnitude
        angles='xy', scale_units='xy', scale=20, cmap='viridis'
    )
    plt.colorbar(q, label='Tilt magnitude (deg)')

    if SHOW_ERRORS:
        # Plot error vectors (dx, dy only for simplicity)
        ax.quiver(
            xyz[:,0], xyz[:,1],
            errors[:,0]*ERROR_SCALE, errors[:,1]*ERROR_SCALE,
            color='red', angles='xy', scale_units='xy', scale=1,
            width=0.003, alpha=0.5, label='Pose errors (scaled)'
        )
        ax.legend()

    ax.set_aspect('equal')
    ax.set_xlabel('X position [mm]')
    ax.set_ylabel('Y position [mm]')
    ax.set_title('Platform Command Coverage and Tilt Directions')
    ax.grid(True)
    plt.tight_layout()
    plt.show()

def main():
    features, errors = load_calib(JOBLIB_FILE)
    xyz, rotvec = decode_features(features)
    plot_calib(xyz, rotvec, errors)

if __name__ == '__main__':
    main()
