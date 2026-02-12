#!/usr/bin/env python3
"""
Plot drop_rate_pct vs cycle from a pogo-pin contact reliability test CSV file.

Usage:
    python plot_pogo_drop_rate.py <csv_file> [output_file]

Examples:
    python plot_pogo_drop_rate.py pogo_0004.csv
    python plot_pogo_drop_rate.py pogo_0004.csv my_plot.png
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker

def plot_drop_rate(csv_files, output_file=None, smooth_window=10):
    # Read and concatenate the CSVs, skipping comment lines
    # Read both CSVs
    dfs = [pd.read_csv(f, comment='#') for f in csv_files]
    # Offset the 'cycle' column in the second file so it continues from the first
    if len(dfs) > 1:
        last_cycle_first = dfs[0]['cycle'].iloc[-1]
        # If the second file's cycle starts at 0 or 1, offset it
        offset = last_cycle_first
        dfs[1]['cycle'] = dfs[1]['cycle'] + offset
    df = pd.concat(dfs, ignore_index=True)

    # Smooth the drop_rate_pct using a rolling mean
    df['drop_rate_pct'] = df['drop_rate_pct'].rolling(window=smooth_window, min_periods=1).mean()

    # Create the plot
    fig, ax = plt.subplots(figsize=(12, 6))

    ax.plot(df['cycle'], df['drop_rate_pct'], linewidth=0.5, color='#2563eb')

    # Add vertical lines and labels
    import textwrap
    # First vertical line: at the end of the first file's data
    n_first = len(pd.read_csv(csv_files[0], comment='#'))
    cycle_first_end = df.loc[n_first - 1, 'cycle']
    ax.axvline(x=cycle_first_end, color='red', linestyle='--', linewidth=1.2)
    label1 = (
        "Stopped due to noise, waited a few days, then applied highly conductive grease"
    )
    label1_lines = textwrap.wrap(label1, width=40)
    label1_wrapped = '\n'.join(label1_lines)
    ax.text(
        cycle_first_end, ax.get_ylim()[1]-10,
        label1_wrapped,
        color='red', rotation=0, va='bottom', ha='center', fontsize=10, backgroundcolor='white', clip_on=False
    )

    # Second vertical line: 56937 points into the second file (68337 into concatenated)
    idx_second = n_first + 56937 - 1
    if idx_second < len(df):
        cycle_second = df.loc[idx_second, 'cycle']
        ax.axvline(x=cycle_second, color='red', linestyle='--', linewidth=1.2)
        label2 = "Reapplied highly conductive grease"
        label2_lines = textwrap.wrap(label2, width=72)
        label2_wrapped = '\n'.join(label2_lines)
        ax.text(
            cycle_second, ax.get_ylim()[1],
            label2_wrapped,
            color='red', rotation=0, va='bottom', ha='center', fontsize=10, backgroundcolor='white', clip_on=False
        )

    ax.set_xlabel('Cycle', fontsize=12)
    ax.set_ylabel('Drop Rate (%)', fontsize=12)
    if smooth_window > 1:
        ax.set_title('Pogo-Pin Contact Reliability Test\nDrop Rate vs Cycle\n(Smoothed over {} cycles)'.format(smooth_window), fontsize=14)
    else:
        ax.set_title('Pogo-Pin Contact Reliability Test\nDrop Rate vs Cycle', fontsize=14)

    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, df['cycle'].max())
    ax.set_ylim(0, max(df['drop_rate_pct'].max() * 1.1, 0.1))

    # Format x-axis tick labels with commas
    ax.xaxis.set_major_formatter(mticker.FuncFormatter(lambda x, _: f"{int(x):,}" if x == int(x) else f"{x:,.0f}"))

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Plot saved to: {output_file}")
    else:
        plt.show()


if __name__ == '__main__':
    import os
    
    # ===== VS Code IDE Configuration =====
    DEFAULT_CSV = "2026-01-22 - 70k cycles.csv"
    SAVE_PLOT = False  # Set to True to save the plot, False to just display
    SMOOTH_FRAMES_WINDOW = 1 # How many frames to smooth over
    # ======================================
    
    if len(sys.argv) >= 2:
        csv_files = sys.argv[1:]
    else:
        # Use the directory of this script to find the default CSVs
        script_dir = os.path.dirname(os.path.abspath(__file__))
        csv_files = [
            os.path.join(script_dir, "2026-01-18 - 11400 cycles.csv"),
            os.path.join(script_dir, "2026-01-22 - 70k cycles.csv")
        ]
        print(f"No files specified, using defaults: {csv_files}")

    if SAVE_PLOT:
        # Generate output filename based on input CSVs
        base_name = os.path.splitext(os.path.basename(csv_files[-1]))[0]
        output_file = f"{base_name}_plot.png"
    else:
        output_file = None

    plot_drop_rate(csv_files, output_file, smooth_window=SMOOTH_FRAMES_WINDOW)
