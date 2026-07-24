#!/usr/bin/env python
"""Cup-tracking bandwidth characterisation for the juggle demo's actuators.

Characterises how well the two actuators that move the cup (hand opening) can
track a commanded sinusoidal CUP Cartesian trajectory:

  * the force-actuated **Stewart platform** (lateral / xy cup motion), and
  * the **hand slider** (vertical / z cup stroke).

For each it reports a lock-in Bode sweep (amplitude ratio + phase lag vs
frequency) plus a static-gain / workspace check. These numbers ground the
online cup planner's per-axis limits (``controller/demo/juggle_planner.py``):
the slider is fast + accurate (do the throw stroke there), the platform tracks
only smooth lateral motion within a ~150 mm workspace at the operating height.

Motivating logbook entry:
  logbook/2026-06-27-throw-aim-band-limit-and-closed-loop-catch.md
  (and its successor on the online-replanning architecture).

Outputs: printed to stdout (no files). Live observer of a headless MuJoCo
plant — no hardware, side-effect-free. Run from the repo root:
  python tools/probes/juggle_cup_bandlimit.py
"""
from __future__ import annotations

import os
import sys

import numpy as np

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_REPO, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_REPO, 'config', 'generated')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import mujoco
from sim.plant.mujoco_plant import MuJoCoPlant

CONTROL_DT = 0.025
OPERATING_Z_MM = 170.0            # pattern active_z (centroid STOW-relative)
SLIDER0_MM = 180.0


def _cup(plant, site_id):
    return plant.data.site_xpos[site_id].copy() * 1000.0   # mm world


def _settle(plant, pose, slider_mm, n=120):
    for _ in range(n):
        plant.command(plant.pose_to_extensions(np.asarray(pose, float)))
        plant.command_hand(slider_mm)
        plant.step(CONTROL_DT)


def _lockin_sweep(plant, site_id, axis, freqs, amp_mm, z0_mm):
    """axis 'platform_x' (centroid-x sinusoid) or 'slider_z' (slider sinusoid).
    Returns list of (f, amp_ratio, phase_lag_deg, achieved_peak_vel_mmps)."""
    out = []
    base_pose = [0, 0, z0_mm, 0, 0, 0]
    for f in freqs:
        w = 2 * np.pi * f
        _settle(plant, base_pose, SLIDER0_MM)
        t0 = float(plant.data.time)

        def plat_fn(tt):
            u = tt - t0
            if axis == 'platform_x':
                return plant.pose_to_extensions(
                    np.array([amp_mm * np.sin(w * u), 0, z0_mm, 0, 0, 0.]))
            return plant.pose_to_extensions(np.array(base_pose, float))

        def hand_fn(tt):
            u = tt - t0
            return SLIDER0_MM + (amp_mm * np.sin(w * u) if axis == 'slider_z' else 0.0)

        ts, a = [], []
        for _ in range(int(round((8.0 / f) / CONTROL_DT))):
            plant.step(CONTROL_DT, hand_cmd_fn=hand_fn, plat_cmd_fn=plat_fn)
            ts.append(float(plant.data.time) - t0)
            cup = _cup(plant, site_id)
            a.append(cup[0] if axis == 'platform_x' else cup[2])
        ts = np.array(ts)
        a = np.array(a) - np.mean(a)
        s, c = np.sin(w * ts), np.cos(w * ts)
        I, Q = np.mean(a * s), np.mean(a * c)
        B = 2 * np.sqrt(I**2 + Q**2)
        lag = -np.degrees(np.arctan2(Q, I))
        lag = lag + 360 if lag < -10 else lag
        out.append((f, B / amp_mm, lag, B * w))
    return out


def main():
    plant = MuJoCoPlant(control_dt=CONTROL_DT)
    site_id = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')

    print("STATIC platform-x tracking at z=170 (cmd centroid-x, settle, measure):")
    print("  cmd_x  centroid_x  cup_x  leg[min,max]")
    for cx in [40, 80, 100, 120, 150]:
        _settle(plant, [cx, 0, OPERATING_Z_MM, 0, 0, 0], SLIDER0_MM, n=200)
        st = plant.get_state()
        print(f"  {cx:4d}   {st.platform_pos_mm[0]:7.1f}   {_cup(plant, site_id)[0]:6.1f}  "
              f"[{st.leg_extensions_mm.min():.0f},{st.leg_extensions_mm.max():.0f}]")

    freqs = [0.8, 1.0, 1.6, 2.0, 3.0, 5.0, 8.0]
    print("\nPLATFORM lateral cup Bode at z=170, amp=80 mm (within workspace):")
    print("  freq  amp_ratio  lag(deg)  ach_peak_vel(mm/s)")
    for f, r, lag, v in _lockin_sweep(plant, site_id, 'platform_x', freqs, 80.0, OPERATING_Z_MM):
        print(f"  {f:4.1f}   {r:6.3f}    {lag:6.1f}     {v:6.0f}")

    print("\nSLIDER vertical cup Bode, amp=80 mm (around 180 mm):")
    print("  freq  amp_ratio  lag(deg)  ach_peak_vel(mm/s)")
    for f, r, lag, v in _lockin_sweep(plant, site_id, 'slider_z', freqs, 80.0, OPERATING_Z_MM):
        print(f"  {f:4.1f}   {r:6.3f}    {lag:6.1f}     {v:6.0f}")

    print("\nKey: slider tracks ~1:1 to >4 m/s (do the throw stroke there); "
          "platform lateral is static-exact within ~150 mm but -3 dB ~5 Hz, "
          "0.95/17deg at the 1.6 Hz cycle freq (smooth lateral positioning only).")


if __name__ == '__main__':
    main()
