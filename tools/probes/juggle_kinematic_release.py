"""Rung-2b kinematic-release knife-edge gain recipe (the make-or-break fix, open loop).

Measures the throw's ``dLanding/dOrigin`` sensitivity — the loop-gain the
throw->catch->throw self-catch loop inherits — for the two throw regimes, holding
everything else fixed (target (100,0) mm, throw origin swept +-10 mm, §3 noise OFF):

* **contact detach** (the shipped ``begin_physics_throw`` — the emergent throw):
  the take-off velocity EMERGES from the hand stroke, so it depends on the contact
  geometry at separation. A 10 mm origin shift swings the landing ~40 mm ->
  ``dLanding/dOrigin`` ~2.7 (worst one-sided ~6.6). Loop gain > 1 -> NO catch scheme
  stabilises the loop (the Rung-2b BREAK).
* **kinematic release** (the Rung-2b fix, ``ballistic_release``): cut the ball free
  at the cup's PLANNED take-off velocity, breaking hand contact so no residual push
  re-corrupts it. The clean-separation Newtonian limit -> the launch is INDEPENDENT
  of the in-cup seat offset -> ``dLanding/dOrigin`` ~0.01. Loop gain << 1 -> the loop
  can converge (the MAKE).

This is the ISOLATED recipe that ruled out contact tuning (every separating solref/
solimp config measured 2.4-16, all >> 1) and validated the kinematic release before
it was wired into the self-catch harness. Was ``/tmp/probe_kinematic_release.py``
(this session); promoted per CLAUDE.md's reusable-probe rule so the logbook
reference stays live. Headless MuJoCo plant, no hardware. Writes
``temp/probes/juggle_kinematic_release.csv``. Run from the repo root:

    python tools/probes/juggle_kinematic_release.py

See ``logbook/2026-07-01-rung2b-kinematic-release.md`` (Phase 3 / Rung 2b of
``plans/active/bb-online-juggle-tilt-rearchitecture.md``).
"""
from __future__ import annotations

import csv
import os
import sys

_repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_repo_root, os.path.join(_repo_root, 'sim'),
           os.path.join(_repo_root, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_repo_root, 'config', 'generated')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
import mujoco

from sim.plant.mujoco_plant import MuJoCoPlant
from sim.juggle_tilt import cup_axis, realize_tilted, tilt_to_throw, MAX_TILT_DEG
from controller.demo.juggle_planner import (
    plan_cup_cycle, takeoff_velocity, ballistic_touchdown, PlannerConfig, GRAVITY)

CONTROL_DT = 0.025
CUP_Z_BASE_MM = 659.6
SLIDER_STROKE_MM = 355.0


def _smoothstep(f):
    f = float(np.clip(f, 0.0, 1.0))
    return f * f * (3.0 - 2.0 * f)


def _run(origin_xy, mode, target_xy=(0.10, 0.0), throw_z=0.85, catch_z=0.70,
         flight=0.6, dip=0.16, tilt_ramp_frac=0.6, settle=70):
    """One throw from ``origin_xy`` toward ``target_xy``; return the ballistic
    landing xy (mm). ``mode`` in {'detach', 'kinematic'}.

    Carries the ball (contact-carry) up the tilted cup axis, then either lets it
    detach by physics (emergent) or cuts it free at the planned v_takeoff
    (kinematic). Contact-carry throughout, so both regimes share the same carry —
    only the RELEASE differs (the isolated variable).
    """
    plant = MuJoCoPlant(control_dt=CONTROL_DT, contact_carry=True)
    sid = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')

    def cup_vel():
        r = np.zeros(6)
        mujoco.mj_objectVelocity(plant.model, plant.data, mujoco.mjtObj.mjOBJ_SITE, sid, r, 0)
        return r[3:6].copy()

    throw_pos = np.array([origin_xy[0], origin_xy[1], throw_z])
    target = np.array([target_xy[0], target_xy[1], catch_z])
    v_takeoff = takeoff_velocity(throw_pos, target, flight)
    rx, ry = tilt_to_throw(v_takeoff, MAX_TILT_DEG)
    axis = cup_axis(rx, ry)

    pcfg = PlannerConfig()
    pcfg.z_min_m = CUP_Z_BASE_MM / 1000.0 + 0.005
    pcfg.z_max_m = (CUP_Z_BASE_MM + SLIDER_STROKE_MM) / 1000.0 - 0.005
    period = max(flight, 0.45)
    catch_time = period * 0.40
    carry_low = throw_pos - axis * dip
    carry_low[2] = max(carry_low[2], catch_z)
    carry_dur = period - catch_time
    plan = plan_cup_cycle(throw_pos, v_takeoff, GRAVITY.copy(), throw_pos, target,
                          flight, catch_time, carry_low, np.array([0.0, 0.0, -2.0]),
                          period, pcfg, detach_axis=axis)

    plant.reset()
    plant.set_contact_stiffness(False)
    pose0, slider0 = realize_tilted(carry_low[:2], carry_low[2], 0.0, 0.0)
    for _ in range(settle):
        plant.command(plant.pose_to_extensions(pose0))
        plant.command_hand(slider0)
        plant.step(CONTROL_DT)
    plant.ball_manager.ball(0).spawn_in_hand()

    tc0 = float(plant.data.time)

    def carry_cmd(t):
        cp, _, _ = plan.sample(catch_time + (t - tc0))
        s = _smoothstep((t - tc0) / max(tilt_ramp_frac * carry_dur, 1e-6))
        return realize_tilted(cp[:2], float(cp[2]), rx * s, ry * s)

    for _ in range(int(round(carry_dur / CONTROL_DT))):
        pose, _ = carry_cmd(float(plant.data.time))
        plant.command(plant.pose_to_extensions(pose))
        plant.step(CONTROL_DT, hand_cmd_fn=lambda t: carry_cmd(t)[1],
                   plat_cmd_fn=lambda t: plant.pose_to_extensions(carry_cmd(t)[0]))

    if mode == "kinematic":
        plant.ballistic_release(v_takeoff * 1000.0, ball=0)
    else:
        plant.set_contact_stiffness(True)
        plant.begin_physics_throw(ball=0)

    for _ in range(8):
        plant.step(CONTROL_DT)
    bs = plant.get_ball_state(0)
    bp = np.asarray(bs.position_mm, float) / 1000.0
    bv = np.asarray(bs.velocity_mms, float) / 1000.0
    _, pos_td, _ = ballistic_touchdown(bp, bv, catch_z)
    return pos_td[:2] * 1000.0, float(np.degrees(np.hypot(rx, ry)))


def _gain(mode):
    d = 10.0
    base, tilt = _run((0.0, 0.0), mode)
    pts = [(dx, dy, _run((dx / 1000.0, dy / 1000.0), mode)[0])
           for dx, dy in ((10, 0), (-10, 0), (0, 10), (0, -10))]
    g = max(np.linalg.norm(p - base) for _, _, p in pts) / d
    err = float(np.linalg.norm(base - np.array([100.0, 0.0])))
    return base, tilt, err, g, pts


def main() -> int:
    out_dir = os.path.join(_repo_root, 'temp', 'probes')
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, 'juggle_kinematic_release.csv')
    rows = []
    print("Rung-2b kinematic-release knife-edge gain — target (100,0) mm, origin +-10 mm, NOISE OFF")
    print("gate: dLanding/dOrigin < 1 => the throw->catch->throw loop can converge\n")
    for mode in ("detach", "kinematic"):
        base, tilt, err, g, pts = _gain(mode)
        verdict = "MAKE (< 1)" if g < 1.0 else "BREAK (> 1)"
        print(f"=== {mode} ===")
        print(f"  landing@origin {base.round(1)} mm  err {err:.1f} mm  tilt {tilt:.2f} deg")
        for dx, dy, p in pts:
            print(f"    origin({dx:+3},{dy:+3}) -> {p.round(1)}")
        print(f"  >> dLanding/dOrigin GAIN = {g:.2f}   [{verdict}]\n")
        rows.append(dict(mode=mode, land_x=round(base[0], 2), land_y=round(base[1], 2),
                         err_mm=round(err, 2), tilt_deg=round(tilt, 2), gain=round(g, 3)))
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f"wrote {csv_path}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
