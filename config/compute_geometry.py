#!/usr/bin/env python3
"""
compute_geometry.py — Stewart platform geometry calculator.

Computes base node positions, platform node positions, and theoretical leg 
lengths from the parametric inputs in hardware_config.yaml.

The node positions stored in hardware_config.yaml were originally computed
by this algorithm (via robot_geometry.py's build_platform() method).
If the physical platform dimensions ever change, re-run this script to
regenerate the node positions.

Usage:
    python config/compute_geometry.py               # verify stored values
    python config/compute_geometry.py --update      # update hardware_config.yaml

The --update flag replaces the base_nodes_mm and init_plat_nodes_mm arrays
in hardware_config.yaml with freshly computed values. It does NOT update
init_leg_lengths_mm (those are measured experimentally, not computed).

Stewart platform node placement using paired-angle convention:
    - Base: 6 nodes placed on a circle of radius base_radius_mm, positioned
    using three angular parameters (gamma0, gamma1, gamma2) derived from
    base_small_angle_deg.
    - Platform: 6 nodes placed on a circle of radius plat_radius_mm, using
    three angular parameters (lambda0, lambda1, lambda2) derived from
    plat_small_angle_deg and plat_x_axis_offset_deg.
"""

import math
import sys
from pathlib import Path

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
HW_YAML = SCRIPT_DIR / "hardware_config.yaml"


def compute_base_nodes(base_radius: float, base_small_angle_deg: float):
    """Compute the 6 base node positions from radius and angular parameter.

    Angular convention (from the original Onshape sketch):
        gamma2 = base_small_angle_deg     — angle between close base nodes
        gamma1 = 120 - gamma2             — angle between far base nodes
        gamma0 = 210 - gamma2 / 2         — offset from horizontal
    """
    deg_to_rad = math.pi / 180.0
    gamma2 = base_small_angle_deg
    gamma0 = 210.0 - gamma2 / 2.0
    gamma1 = 120.0 - gamma2

    nodes = []
    for i in range(6):
        first = int(math.floor(i / 2))
        second = int(math.floor((i + 1) / 2))
        angle_deg = gamma0 + gamma1 * first + gamma2 * second
        x = base_radius * math.cos(angle_deg * deg_to_rad)
        y = base_radius * math.sin(angle_deg * deg_to_rad)
        nodes.append([x, y, 0.0])
    return nodes


def compute_plat_nodes(plat_radius: float, plat_small_angle_deg: float,
                       plat_x_axis_offset_deg: float):
    """Compute the 6 platform node positions from radius and angular parameters.

    Angular convention:
        lambda1 = plat_small_angle_deg       — angle between close platform nodes
        lambda2 = 120 - lambda1              — angle between far platform nodes
        lambda0 = plat_x_axis_offset_deg     — offset from x-axis (measured in Onshape)
    """
    deg_to_rad = math.pi / 180.0
    lambda1 = plat_small_angle_deg
    lambda2 = 120.0 - lambda1
    lambda0 = plat_x_axis_offset_deg

    nodes = []
    for i in range(6):
        first = int(math.floor(i / 2))
        second = int(math.floor((i + 1) / 2))
        angle_deg = lambda0 + lambda1 * first + lambda2 * second
        x = plat_radius * math.cos(angle_deg * deg_to_rad)
        y = plat_radius * math.sin(angle_deg * deg_to_rad)
        nodes.append([x, y, 0.0])
    return nodes

def compute_theoretical_leg_lengths(base_nodes, plat_nodes, initial_height: float):
    """Compute leg lengths from geometry (platform at home position, no ball-joint offset)."""
    lengths = []
    for i in range(6):
        dx = plat_nodes[i][0] - base_nodes[i][0]
        dy = plat_nodes[i][1] - base_nodes[i][1]
        dz = initial_height  # plat z - base z (base z = 0)
        lengths.append(math.sqrt(dx * dx + dy * dy + dz * dz))
    return lengths


def round_list(vals, decimals=3):
    """Round a flat list or list-of-lists to given decimal places."""
    if isinstance(vals[0], list):
        return [[round(v, decimals) for v in row] for row in vals]
    return [round(v, decimals) for v in vals]


def max_diff(a, b):
    """Maximum element-wise difference between two (possibly nested) lists."""
    if isinstance(a[0], list):
        return max(abs(ai - bi) for row_a, row_b in zip(a, b) for ai, bi in zip(row_a, row_b))
    return max(abs(ai - bi) for ai, bi in zip(a, b))


def main():
    update_mode = "--update" in sys.argv

    if not HW_YAML.exists():
        print(f"Error: {HW_YAML} not found", file=sys.stderr)
        sys.exit(1)

    with open(HW_YAML, "r") as f:
        cfg = yaml.safe_load(f)

    geom = cfg["jugglebot_geometry"]

    # Compute from parametric inputs
    base_nodes = compute_base_nodes(geom["base_radius_mm"], geom["base_small_angle_deg"])
    plat_nodes = compute_plat_nodes(geom["plat_radius_mm"], geom["plat_small_angle_deg"],
                                    geom["plat_x_axis_offset_deg"])
    theoretical_legs = compute_theoretical_leg_lengths(base_nodes, plat_nodes,
                                                       geom["initial_height_mm"])

    # Round for display and storage
    base_nodes_r = round_list(base_nodes, 3)
    plat_nodes_r = round_list(plat_nodes, 3)

    # Compare with stored values
    stored_base = geom["base_nodes_mm"]
    stored_plat = geom["init_plat_nodes_mm"]

    base_err = max_diff(base_nodes_r, stored_base)
    plat_err = max_diff(plat_nodes_r, stored_plat)

    print("=== Stewart Platform Geometry Verification ===\n")

    print("Parametric inputs:")
    print(f"  base_radius_mm        = {geom['base_radius_mm']}")
    print(f"  plat_radius_mm        = {geom['plat_radius_mm']}")
    print(f"  base_small_angle_deg  = {geom['base_small_angle_deg']}")
    print(f"  plat_small_angle_deg  = {geom['plat_small_angle_deg']}")
    print(f"  plat_x_axis_offset_deg = {geom['plat_x_axis_offset_deg']}")
    print(f"  initial_height_mm     = {geom['initial_height_mm']}")
    print()

    print("Computed base_nodes_mm (rounded to 3 dp):")
    for i, node in enumerate(base_nodes_r):
        stored = stored_base[i]
        match = "OK" if node == stored else f"DIFF (stored: {stored})"
        print(f"  [{i}] {node}  {match}")
    print(f"  Max error: {base_err:.6f} mm")
    print()

    print("Computed init_plat_nodes_mm (rounded to 3 dp):")
    for i, node in enumerate(plat_nodes_r):
        stored = stored_plat[i]
        match = "OK" if node == stored else f"DIFF (stored: {stored})"
        print(f"  [{i}] {node}  {match}")
    print(f"  Max error: {plat_err:.6f} mm")
    print()

    print("Theoretical leg lengths (from geometry, NO ball-joint offset):")
    stored_legs = geom["init_leg_lengths_mm"]
    for i, stored in enumerate(stored_legs):
        print(f"  [{i}] computed: {theoretical_legs[i]:.3f} mm, stored: {stored} mm, "
              f"diff: {theoretical_legs[i] - stored:.3f} mm")
    print()
    print("NOTE: init_leg_lengths_mm are measured experimentally (Livestream #51 @ ~28min),")
    print("not computed from geometry. Differences are expected and correct.")
    print()

    if base_err < 0.001 and plat_err < 0.001:
        print("RESULT: Stored values match computed values. No update needed.")
    else:
        print(f"RESULT: Max base error = {base_err:.6f} mm, max plat error = {plat_err:.6f} mm")
        if update_mode:
            print("\n--update flag set. Updating hardware_config.yaml...")
            with open(HW_YAML, "r") as f:
                raw = f.read()

            # Replace base_nodes_mm block
            import re
            base_yaml = "  base_nodes_mm:\n"
            for node in base_nodes_r:
                base_yaml += f"    - [{node[0]}, {node[1]}, {node[2]}]\n"
            raw = re.sub(
                r"  base_nodes_mm:\n(?:    - \[.*?\]\n){6}",
                base_yaml, raw
            )

            # Replace init_plat_nodes_mm block
            plat_yaml = "  init_plat_nodes_mm:\n"
            for node in plat_nodes_r:
                plat_yaml += f"    - [{node[0]}, {node[1]}, {node[2]}]\n"
            raw = re.sub(
                r"  init_plat_nodes_mm:\n(?:    - \[.*?\]\n){6}",
                plat_yaml, raw
            )

            with open(HW_YAML, "w") as f:
                f.write(raw)
            print("Done. Run `python config/generate_config.py` to regenerate outputs.")
        else:
            print("Run with --update to update hardware_config.yaml.")


if __name__ == "__main__":
    main()
