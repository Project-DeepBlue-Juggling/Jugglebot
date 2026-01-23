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


def plot_drop_rate(csv_file, output_file=None, smooth_window=10):
    # Read the CSV, skipping comment lines
    df = pd.read_csv(csv_file, comment='#')

    # Smooth the drop_rate_pct using a rolling mean
    df['drop_rate_pct'] = df['drop_rate_pct'].rolling(window=smooth_window, min_periods=1).mean()
    
    # Create the plot
    fig, ax = plt.subplots(figsize=(12, 6))
    
    ax.plot(df['cycle'], df['drop_rate_pct'], linewidth=0.5, color='#2563eb')
    
    ax.set_xlabel('Cycle', fontsize=12)
    ax.set_ylabel('Drop Rate (%)', fontsize=12)
    ax.set_title('Pogo-Pin Contact Reliability Test\nDrop Rate vs Cycle\n(Smoothed over {} cycles)'.format(smooth_window), fontsize=14)
    
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, df['cycle'].max())
    ax.set_ylim(0, max(df['drop_rate_pct'].max() * 1.1, 0.1))
    
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
    SMOOTH_FRAMES_WINDOW = 200 # How many frames to smooth over
    # ======================================
    
    if len(sys.argv) >= 2:
        csv_file = sys.argv[1]
    else:
        # Use the directory of this script to find the default CSV
        script_dir = os.path.dirname(os.path.abspath(__file__))
        csv_file = os.path.join(script_dir, DEFAULT_CSV)
        print(f"No file specified, using default: {csv_file}")
    
    if len(sys.argv) > 2:
        output_file = sys.argv[2]
    elif SAVE_PLOT:
        # Generate output filename based on input CSV
        base_name = os.path.splitext(csv_file)[0]
        output_file = f"{base_name}_plot.png"
    else:
        output_file = None
    
    plot_drop_rate(csv_file, output_file, smooth_window=SMOOTH_FRAMES_WINDOW)
