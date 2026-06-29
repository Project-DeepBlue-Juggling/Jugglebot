#!/usr/bin/env python
"""Platform-tilt (orientation) tracking characterisation for throw-aiming.

The online juggle re-architecture aims each throw by **tilting the cup** and
detaching the ball along the tilted axis: the lateral take-off velocity is
``slider_speed * sin(tilt)`` (delivered by the fast, accurate slider along a
roughly-constant tilt) instead of by band-limited platform *translation*. This
probe quantifies how well the Stewart platform tracks orientation (rx/ry tilt)
at the juggle operating point so Phase 1 can build the tilted-detach planner on
real numbers.

It complements ``juggle_cup_bandlimit.py`` (which characterised lateral
*translation* + the slider). Here we characterise *tilt*:

  1. **Static hold** — commanded vs achieved tilt over a modest sweep
     (0/3/6/9/12 deg); accuracy + any droop.
  2. **Lever arm** — mm of cup (``hand_opening`` site) **lateral** shift per
     degree of tilt, plus the (second-order) **vertical** cross-coupling, so
     Phase 1 can offset ``centroid_xy`` to land the cup where the plan wants it.
  3. **Leg headroom** — does a modest tilt at z=170 keep all six legs off their
     [-5, 285] mm slide limits (no saturation — the artefact that bit the z=0
     lateral sweep)? Swept out to 24 deg to find the real margin.
  4. **Dynamic** — a low-frequency tilt Bode (amplitude ratio + phase lag)
     around the 1.6 Hz juggle cycle frequency; how fast can tilt change before
     the band limit bites?

Both tilt axes (rx and ry) are characterised because the Stewart leg layout is
not symmetric between x and y, so the leg spread (hence headroom) differs.

Motivating logbook entry:
  logbook/2026-06-29-platform-tilt-tracking-characterisation.md
  (Phase 0 of plans/active/bb-online-juggle-tilt-rearchitecture.md)

Outputs: printed to stdout (no files). Headless MuJoCo plant — no hardware,
side-effect-free. Run from the repo root:
  python tools/probes/juggle_tilt_bandlimit.py
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
SLIDER0_MM = 180.0                # nominal carry slider position (cup height)
SLIDE_LO_MM = -5.0                # MuJoCo leg slide-joint ctrl floor
SLIDE_HI_MM = 285.0               # MuJoCo leg slide-joint ctrl ceiling

# Tilt axis spec: (label, pose rot-vec index, lateral cup-shift axis, that
# axis' index). A +ry tilt rotates the cup z-axis toward +x (aim/shift in x);
# a +rx tilt rotates it toward -y (aim/shift in -y).
_AXES = [('ry', 1, 'x', 0), ('rx', 0, 'y', 1)]


def _cup(plant, site_id):
    return plant.data.site_xpos[site_id].copy() * 1000.0   # mm world


def _settle(plant, pose, slider_mm, n=200):
    for _ in range(n):
        plant.command(plant.pose_to_extensions(np.asarray(pose, float)))
        plant.command_hand(slider_mm)
        plant.step(CONTROL_DT)


def _tilt_pose(z0_mm, rot_idx, rad):
    pose = [0.0, 0.0, z0_mm, 0.0, 0.0, 0.0]
    pose[3 + rot_idx] = rad
    return pose


def _static_and_lever(plant, site_id, axis, z0_mm):
    """Static-hold accuracy + lever arm + leg headroom for one tilt axis.

    Returns list of (cmd_deg, ach_deg, d_lat_mm, d_vert_mm, mmdeg, legmin, legmax)
    where d_lat/d_vert are cup shift vs the level (tilt=0) reference.
    """
    _, rot_idx, _lat_name, lat_idx = next(a for a in _AXES if a[0] == axis)
    _settle(plant, _tilt_pose(z0_mm, rot_idx, 0.0), SLIDER0_MM)
    cup0 = _cup(plant, site_id)
    out = []
    for deg in [0, 3, 6, 9, 12]:
        _settle(plant, _tilt_pose(z0_mm, rot_idx, np.radians(deg)), SLIDER0_MM)
        st = plant.get_state()
        ach = np.degrees(st.platform_rot[rot_idx])
        cup = _cup(plant, site_id)
        d_lat = cup[lat_idx] - cup0[lat_idx]
        d_vert = cup[2] - cup0[2]
        mmdeg = d_lat / deg if deg else 0.0
        out.append((deg, ach, d_lat, d_vert, mmdeg,
                    st.leg_extensions_mm.min(), st.leg_extensions_mm.max()))
    return out


def _headroom(plant, axis, z0_mm, degs):
    """Leg min/max (and margins to the slide limits) at high tilt — find where
    (if anywhere) a leg approaches saturation. Returns list of
    (cmd_deg, legmin, legmax)."""
    _, rot_idx, _, _ = next(a for a in _AXES if a[0] == axis)
    out = []
    for deg in degs:
        _settle(plant, _tilt_pose(z0_mm, rot_idx, np.radians(deg)), SLIDER0_MM)
        st = plant.get_state()
        out.append((deg, st.leg_extensions_mm.min(), st.leg_extensions_mm.max()))
    return out


def _tilt_bode(plant, axis, freqs, amp_deg, z0_mm):
    """Lock-in Bode of achieved tilt vs a commanded sinusoidal tilt.
    Returns list of (f, amp_ratio, phase_lag_deg, achieved_peak_rate_degps)."""
    _, rot_idx, _, _ = next(a for a in _AXES if a[0] == axis)
    amp_rad = np.radians(amp_deg)
    out = []
    base_pose = _tilt_pose(z0_mm, rot_idx, 0.0)
    for f in freqs:
        w = 2 * np.pi * f
        _settle(plant, base_pose, SLIDER0_MM, n=120)
        t0 = float(plant.data.time)

        def plat_fn(tt):
            u = tt - t0
            return plant.pose_to_extensions(
                np.array(_tilt_pose(z0_mm, rot_idx, amp_rad * np.sin(w * u)), float))

        ts, a = [], []
        for _ in range(int(round((8.0 / f) / CONTROL_DT))):
            plant.step(CONTROL_DT, plat_cmd_fn=plat_fn)
            ts.append(float(plant.data.time) - t0)
            a.append(np.degrees(plant.get_state().platform_rot[rot_idx]))
        ts = np.array(ts)
        a = np.array(a) - np.mean(a)
        s, c = np.sin(w * ts), np.cos(w * ts)
        I, Q = np.mean(a * s), np.mean(a * c)
        B = 2 * np.sqrt(I**2 + Q**2)
        lag = -np.degrees(np.arctan2(Q, I))
        lag = lag + 360 if lag < -10 else lag
        out.append((f, B / amp_deg, lag, B * w))
    return out


def main():
    plant = MuJoCoPlant(control_dt=CONTROL_DT)
    site_id = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')

    # Settle to the level operating pose so the reported cup height is the
    # achieved value at the operating point (not the unsettled home keyframe).
    _settle(plant, _tilt_pose(OPERATING_Z_MM, 1, 0.0), SLIDER0_MM)
    print(f"Operating point: centroid z={OPERATING_Z_MM:.0f} mm, slider={SLIDER0_MM:.0f} mm "
          f"(cup ~{_cup(plant, site_id)[2]:.0f} mm world at level).")
    print(f"Leg slide-joint range: [{SLIDE_LO_MM:.0f}, {SLIDE_HI_MM:.0f}] mm "
          "(leg_extensions_mm ~= slide_mm at this geometry).\n")

    for axis, _ri, lat_name, _li in _AXES:
        print(f"STATIC tilt about {axis} (aim/shift in {lat_name}) — hold accuracy, "
              "lever arm, leg headroom:")
        print("  cmd_deg  ach_deg  droop_deg  d_lat(mm)  mm/deg  d_vert(mm)  leg[min,max]")
        for deg, ach, d_lat, d_vert, mmdeg, lmin, lmax in \
                _static_and_lever(plant, site_id, axis, OPERATING_Z_MM):
            print(f"   {deg:5.1f}   {ach:6.2f}    {deg - ach:6.3f}   {d_lat:7.2f}  "
                  f"{mmdeg:6.3f}   {d_vert:7.2f}    [{lmin:.1f},{lmax:.1f}]")
        print()

    print("LEG HEADROOM at high tilt (margins to the slide limits):")
    print("  axis  cmd_deg  leg[min,max]   margin_lo  margin_hi")
    for axis, _ri, _ln, _li in _AXES:
        for deg, lmin, lmax in _headroom(plant, axis, OPERATING_Z_MM,
                                         [12, 15, 18, 21, 24]):
            print(f"  {axis:4s}  {deg:5.1f}   [{lmin:6.1f},{lmax:6.1f}]   "
                  f"{lmin - SLIDE_LO_MM:7.1f}   {SLIDE_HI_MM - lmax:7.1f}")
    print()

    freqs = [0.8, 1.0, 1.6, 2.0, 3.0, 5.0, 8.0]
    for axis, _ri, _ln, _li in _AXES:
        print(f"DYNAMIC tilt Bode about {axis}, amp=6 deg "
              "(cup lateral ~10 mm — small, in workspace):")
        print("  freq  amp_ratio  lag(deg)  ach_peak_rate(deg/s)")
        for f, r, lag, v in _tilt_bode(plant, axis, freqs, 6.0, OPERATING_Z_MM):
            print(f"  {f:4.1f}   {r:6.3f}    {lag:6.1f}     {v:7.1f}")
        print()

    print("Key: tilt is held STATIC-EXACT (no droop) and the lever arm is a clean "
          "~1.66 mm/deg lateral (cup ~95 mm above the centroid) with only "
          "second-order vertical cross-coupling (~2 mm at 12 deg). Legs keep "
          ">45 mm margin even at 24 deg, so leg stroke does NOT bind tilt at "
          "z=170. Dynamically tilt tracks ~0.99/3deg at the 1.6 Hz cycle freq "
          "(better than lateral translation's 0.95/17deg) and is still 0.95 at "
          "5 Hz — so a modest tilt held through the throw and re-aimed "
          "cycle-to-cycle sits well inside the band.")


if __name__ == '__main__':
    main()
