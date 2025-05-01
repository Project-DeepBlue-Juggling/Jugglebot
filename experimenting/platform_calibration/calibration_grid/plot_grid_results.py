import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R
import os

import matplotlib.pyplot as plt

# Import the CSV file
# file_name = '180x180_grid.csv'
# file_name = '180x180_grid_with_ff_correction.csv'
# file_name = '160x160_grid_with_ff_correction.csv'
file_name = 'Happy face.csv'

# Get the current working directory
current_dir = os.path.dirname(os.path.abspath(__file__))
# Construct the full path to the CSV file
csv_file_path = os.path.join(current_dir, file_name)

arrow_length = 0.2

# Read CSV
df = pd.read_csv(csv_file_path)

# Calculate the average error between the commanded and measured positions
df['error_x'] = df['cmd_pos_x'] - df['meas_pos_x']
df['error_y'] = df['cmd_pos_y'] - df['meas_pos_y']
df['error_z'] = df['cmd_pos_z'] - df['meas_pos_z']
df['error_qx'] = df['cmd_ori_x'] - df['meas_ori_x']
df['error_qy'] = df['cmd_ori_y'] - df['meas_ori_y']
df['error_qz'] = df['cmd_ori_z'] - df['meas_ori_z']
df['error_qw'] = df['cmd_ori_w'] - df['meas_ori_w']

# Calculate the mean and standard deviation of the errors
mean_error_pos_error = np.mean(np.sqrt(df['error_x']**2 + df['error_y']**2 + df['error_z']**2))
mean_error_q_x = np.mean(np.sqrt(df['error_qx']**2))
mean_error_q_y = np.mean(np.sqrt(df['error_qy']**2))
mean_error_q_z = np.mean(np.sqrt(df['error_qz']**2))
mean_error_q_w = np.mean(np.sqrt(df['error_qw']**2))

print(f"Mean Position Error: {mean_error_pos_error:.4f} mm")
print(f"Mean Orientation Error (qx): {mean_error_q_x:.4f}")
print(f"Mean Orientation Error (qy): {mean_error_q_y:.4f}")
print(f"Mean Orientation Error (qz): {mean_error_q_z:.4f}")
print(f"Mean Orientation Error (qw): {mean_error_q_w:.4f}")

print(f"Max Position Error: {np.max(np.sqrt(df['error_x']**2 + df['error_y']**2 + df['error_z']**2)):.4f} mm")


# Function to plot 3D poses
def plot_poses_3d(ax, pos_prefix, ori_prefix, color, label):
    xs = df[f'{pos_prefix}_x']
    ys = df[f'{pos_prefix}_y']
    zs = df[f'{pos_prefix}_z']
    qxs = df[f'{ori_prefix}_x']
    qys = df[f'{ori_prefix}_y']
    qzs = df[f'{ori_prefix}_z']
    qws = df[f'{ori_prefix}_w']

    ax.scatter(xs, ys, zs, color=color, label=label, alpha=0.6)

    # Draw orientation arrows
    for x, y, z, qx, qy, qz, qw in zip(xs, ys, zs, qxs, qys, qzs, qws):
        r = R.from_quat([qx, qy, qz, qw])
        dir_vec = r.apply([0, 0, 1]) 
        ax.quiver(x, y, z, dir_vec[0], dir_vec[1], dir_vec[2], color=color, length=arrow_length, normalize=False)

# Function to plot 2D poses (x, y). No orientation shown on this plot
def plot_poses_2d(ax, pos_prefix, color, label):
    xs = df[f'{pos_prefix}_x']
    ys = df[f'{pos_prefix}_y']

    ax.scatter(xs, ys, color=color, label=label, alpha=0.6)

# 3D plot
fig_3d = plt.figure()
ax_3d = fig_3d.add_subplot(111, projection='3d')

plot_poses_3d(ax_3d, 'meas_pos', 'meas_ori', 'blue', 'Measured')
plot_poses_3d(ax_3d, 'cmd_pos', 'cmd_ori', 'red', 'Commanded')

ax_3d.set_xlabel('X')
ax_3d.set_ylabel('Y')
ax_3d.set_zlabel('Z')
ax_3d.legend()
ax_3d.set_title('Measured vs Commanded Poses')

# 2D plot
fig_2d = plt.figure()
ax_2d = fig_2d.add_subplot(111)
plot_poses_2d(ax_2d, 'meas_pos', 'blue', 'Measured')
plot_poses_2d(ax_2d, 'cmd_pos', 'red', 'Commanded')
ax_2d.set_xlabel('X')
ax_2d.set_ylabel('Y')
ax_2d.legend()
ax_2d.set_title('Measured vs Commanded Poses (2D)')

plt.show()