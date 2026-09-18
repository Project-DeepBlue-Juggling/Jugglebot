"""The RETIRED 0.5° pitch sweep of the Ball Butler throw solver — test oracle only.

Until 2026-09-18 ``jugglebot.can.throw_ballistics.solve_throw_local`` and its sim
twin ``sim.ball_butler.sim._solve_throw`` both found the pitch minimising
horizontal landing velocity by sweeping the pitch range on a 0.5° grid.  The
closed-form ``_steepest_feasible_pitch`` replaced the sweep; this file keeps the
sweep, arithmetic untouched, so the tests can pin the replacement against it.
Never import this from production code.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple


def grid_pitch_solve(
    A: float, z_mm: float, l: float, d: float,
    pitch_min_rad: float, pitch_max_rad: float,
    v_max_mmps: float, h_max_mm: float, g_mmps2: float,
    pitch_step_deg: float = 0.5,
) -> Optional[Tuple[float, float, float, float]]:
    """``(pitch_rad, v_mmps, h_peak_mm, h_vel_mmps)`` of the grid argmin, or
    None where the sweep found no feasible pitch."""
    pitch_step_rad = math.radians(pitch_step_deg)
    n_steps = max(int((pitch_max_rad - pitch_min_rad) / pitch_step_rad), 1) + 1

    best = None  # (h_vel, pitch_rad, v_mmps, h_peak_mm)
    for i in range(n_steps):
        pitch_rad = pitch_min_rad + i * (pitch_max_rad - pitch_min_rad) / max(n_steps - 1, 1)
        cos_p = math.cos(pitch_rad)
        sin_p = math.sin(pitch_rad)
        if abs(cos_p) < 1e-6:
            continue
        R = A - l * cos_p + d
        if R <= 0:
            continue
        z_throw_mm = z_mm - l * sin_p
        sin_2p = 2.0 * sin_p * cos_p
        cos_2p = cos_p * cos_p - sin_p * sin_p
        denom = R * sin_2p - z_throw_mm * (1.0 + cos_2p)
        if denom <= 0:
            continue
        v_sq = g_mmps2 * R * R / denom
        if v_sq <= 0:
            continue
        v_mmps = math.sqrt(v_sq)
        if v_mmps > v_max_mmps:
            continue
        v_vert = v_mmps * sin_p
        h_peak_mm = (v_vert * v_vert) / (2.0 * g_mmps2) if v_vert > 0 else 0.0
        if h_peak_mm > h_max_mm:
            continue
        h_vel = v_mmps * cos_p
        if best is None or h_vel < best[0]:
            best = (h_vel, pitch_rad, v_mmps, h_peak_mm)

    if best is None:
        return None
    h_vel, pitch_rad, v_mmps, h_peak_mm = best
    return pitch_rad, v_mmps, h_peak_mm, h_vel
