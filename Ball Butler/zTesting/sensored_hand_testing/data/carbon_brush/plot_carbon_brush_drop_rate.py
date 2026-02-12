#!/usr/bin/env python3
"""
Plot drop_rate_pct vs cycle from a carbon brush contact reliability test CSV file.

Usage:
    python plot_carbon_brush_drop_rate.py [csv_prefix]

Examples:
    python plot_carbon_brush_drop_rate.py
    python plot_carbon_brush_drop_rate.py carbon_brush_
"""

import sys
import glob
import os
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker

CM_PER_CYCLE = 49

def plot_drop_rate(csv_files, output_file=None, smooth_window=10):
    # Read and concatenate the CSVs, skipping comment lines
    dfs = [pd.read_csv(f, comment='#') for f in csv_files]

    # Offset cycle columns so each file continues from the previous
    for i in range(1, len(dfs)):
        offset = dfs[i - 1]['cycle'].iloc[-1]
        dfs[i]['cycle'] = dfs[i]['cycle'] + offset

    df = pd.concat(dfs, ignore_index=True)

    # Smooth the drop_rate_pct using a rolling mean
    df['drop_rate_pct'] = df['drop_rate_pct'].rolling(window=smooth_window, min_periods=1).mean()

    # Create the plot
    fig, ax = plt.subplots(figsize=(12, 6))

    ax.plot(df['cycle'], df['drop_rate_pct'], linewidth=0.5, color='#2563eb')

    ax.set_xlabel('Cycle', fontsize=12)
    ax.set_ylabel('Drop Rate (%)', fontsize=12)
    if smooth_window > 1:
        ax.set_title('Carbon Brush Contact Reliability Test\nDrop Rate vs Cycle\n(Smoothed over {} cycles)'.format(smooth_window), fontsize=14)
    else:
        ax.set_title('Carbon Brush Contact Reliability Test\nDrop Rate vs Cycle', fontsize=14)

    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, df['cycle'].max())
    ax.set_ylim(0, max(df['drop_rate_pct'].max() * 1.1, 0.1))

    # Format x-axis tick labels with commas
    ax.xaxis.set_major_formatter(mticker.FuncFormatter(lambda x, _: f"{int(x):,}" if x == int(x) else f"{x:,.0f}"))

    # Add secondary x-axis showing total distance travelled (offset below primary)
    ax2 = ax.secondary_xaxis(-0.12, functions=(
        lambda x: x * CM_PER_CYCLE / 100_000,  # cycles -> km
        lambda x: x * 100_000 / CM_PER_CYCLE,  # km -> cycles
    ))
    ax2.set_xlabel('Distance (km)', fontsize=12)
    ax2.xaxis.set_major_formatter(mticker.FuncFormatter(lambda x, _: f"{x:,.1f}"))  # type: ignore[attr-defined]

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Plot saved to: {output_file}")
    else:
        plt.show()


if __name__ == '__main__':
    # ===== VS Code IDE Configuration =====
    CSV_PREFIX = "carbon_brush_"
    SAVE_PLOT = False  # Set to True to save the plot, False to just display
    SMOOTH_FRAMES_WINDOW = 1  # How many frames to smooth over
    # ======================================

    if len(sys.argv) >= 2:
        CSV_PREFIX = sys.argv[1]

    # Find all matching CSVs in the script's directory, sorted by name
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pattern = os.path.join(script_dir, f"{CSV_PREFIX}*.csv")
    csv_files = sorted(glob.glob(pattern))

    if not csv_files:
        print(f"No CSV files found matching '{pattern}'")
        sys.exit(1)

    print(f"Found {len(csv_files)} CSV files: {[os.path.basename(f) for f in csv_files]}")

    if SAVE_PLOT:
        output_file = os.path.join(script_dir, f"{CSV_PREFIX}combined_plot.png")
    else:
        output_file = None

    plot_drop_rate(csv_files, output_file, smooth_window=SMOOTH_FRAMES_WINDOW)
