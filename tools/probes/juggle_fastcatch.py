"""Fast-catch fidelity probe — seat a REAL fast Ball Butler throw in the Rung-1 catch.

What it characterises
---------------------
The shipped Rung-1 catch (``sim/juggle_catch.py``) was tuned for a gentle ~2.5 m/s
lob and could NOT seat the fast (~4.9 m/s vz), flat (~15° from vertical) arrival a
REAL Ball Butler delivers. Two coupled causes, both reproduced here:

  1. **Soft catch contact** — the old ``(0.05, 2.0)`` = 50 ms overdamped contact was
     too slow to arrest a fast ball inside the ~70 mm seat-escape window; the ball
     PLUNGED THROUGH the cup (held 0/6, ball ends ~1 m below).
  2. **Gentle-tuned seat** — the cup held high then descended too LATE, so it was
     not co-moving when the fast ball arrived (mistimed phase).

The fix (this probe validates the recipe before it was wired into the runner):

  * **Firm catch contact:** ``solref=(0.01, 1.0)``, ``solimp=(0.99, 0.999, 0.0005)``
    (a ~10 ms time constant) — arrests the fast arrival in-window, still cushioned
    for the gentle lob + the self-catch co-moving seat.
  * **Phase-matched descent seat:** from a larger ready-lift, drive the cup so it
    arrives at catch_z MOVING DOWN at ``ratio``·|arrival vz| EXACTLY at the predicted
    touch-down (a re-solved quintic completing at ``t_td``), then constant-decel to
    rest. ``ratio`` 0.55 (sweet spot 0.5–0.6; 0.7+ commands a descent the band-
    limited cup cannot track and it misses the phase). ``ready_lift`` 0.15 gives
    room to build the matched speed.

Verdict (below): SOFT plunges through at both speeds' fast end; FIRM + phase-match
seats the gentle lob AND the real BB, tight (~5–12 mm).

Motivating logbook: ``logbook/2026-07-02-fast-catch-fidelity.md``
Grounds test:       ``tests/sim/test_juggle_catch.py::test_fast_real_ball_butler_throw_is_seated``

Headless MuJoCo plant, no hardware. Writes ``temp/probes/juggle_fastcatch.csv``.
Run: ``python tools/probes/juggle_fastcatch.py`` (from the repo root).
"""
from __future__ import annotations

import csv
import math
import os
import sys

_repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_repo_root, os.path.join(_repo_root, 'sim')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
import mujoco

from sim.plant.mujoco_plant import MuJoCoPlant
from sim.ball_butler.sim import BallButlerSim
from sim.juggle_noise import NoiseConfig, JuggleNoise, BallisticEstimator
from sim.juggle_tilt import realize_tilted, tilt_to_receive, MAX_TILT_DEG
from sim.juggle_catch import _quintic_step
from controller.demo.juggle_planner import ballistic_touchdown, takeoff_velocity, GRAVITY

CONTROL_DT = 0.025
CATCH_Z = 0.84
WORKSPACE_XY_M = 0.15
# The demo Ball Butler placement (world mm), aimed from across the room at Jugglebot.
BB = BallButlerSim.from_hardware_config(np.array([-872., -630., 1430.]),
                                        math.atan2(630., 872.))

# Contact presets: SOFT = the OLD 50 ms catch contact (plunges a fast ball);
# FIRM = the shipped fast-catch contact (ball.manager.CONTACT_SOFT_* after 2026-07-02).
SOFT = ((0.05, 2.0), (0.99, 0.99, 0.001))
FIRM = ((0.01, 1.0), (0.99, 0.999, 0.0005))

# Shipped Rung-1 catch constants (sim/juggle_catch.py SingleCatchConfig).
READY_LIFT = 0.15
RATIO = 0.55
SEAT_DECEL_T = 0.16


def _set_catch_contact(plant, ref, imp):
    """Force the ball+cup geoms to a given contact regime (bypasses the manager's
    idempotent guard — the plain first set_contact_stiffness(False) is a no-op)."""
    bm = plant.ball_manager
    for g in [b._ball_geom_id for b in bm._balls] + list(bm._hand_geom_ids):
        plant.model.geom_solref[g][:2] = ref
        plant.model.geom_solimp[g][:3] = imp


def real_bb_apex(noise):
    """A REAL Ball Butler throw aimed at (0,0,catch_z), §3-perturbed, at its apex."""
    pos, vel, _ = BB.compute_release_state(np.array([0., 0., CATCH_Z * 1000]))
    disp = np.array([0., 0., CATCH_Z * 1000]) - np.asarray(pos, float)
    pos_n, vel_n = noise.perturb_throw(np.asarray(pos, float), np.asarray(vel, float), disp)
    return _to_apex(pos_n / 1000., vel_n / 1000.)


def gentle_lob_apex(noise):
    """A gentle synthetic lob (~2.5 m/s), like SingleCatchRunner's default, at apex."""
    target = np.array([0., 0., CATCH_Z]); origin = target + np.array([-0.15, 0., -0.25])
    v0 = takeoff_velocity(origin, target, 0.60)
    disp = (target - origin) * 1000.
    pos_n, vel_n = noise.perturb_throw(origin * 1000., v0 * 1000., disp)
    return _to_apex(pos_n / 1000., vel_n / 1000.)


def _to_apex(p0, v0):
    g = 9.806
    ta = v0[2] / g if v0[2] > 0 else 0.0
    ap = p0 + v0 * ta + 0.5 * GRAVITY * ta ** 2
    av = v0 + GRAVITY * ta
    return ap * 1000, av * 1000


def run_catch(spawn_fn, contact, ready_lift, ratio, seat_decel_t, seed):
    """One fast-catch: firm/soft contact + phase-matched descent seat (mirrors
    sim/juggle_catch.py::SingleCatchRunner.run's z-channel). Returns (held, offset_mm)."""
    ref, imp = contact
    plant = MuJoCoPlant(control_dt=CONTROL_DT, contact_carry=True)
    sid = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')
    noise = JuggleNoise(NoiseConfig(), seed=seed)
    est = BallisticEstimator(GRAVITY)
    plant.reset(); plant.set_contact_stiffness(False)
    _set_catch_contact(plant, ref, imp)
    ready_z = CATCH_Z + ready_lift
    pose0, sl0 = realize_tilted(np.array([0., 0.]), ready_z, 0., 0.)
    for _ in range(70):
        plant.command(plant.pose_to_extensions(pose0)); plant.command_hand(sl0); plant.step(CONTROL_DT)
    ap, av = spawn_fn(noise)
    plant.spawn_ball(ap, av, ball=0)

    cmd_xy = np.zeros(2); cmd_vxy = np.zeros(2); cmd_axy = np.zeros(2)
    cmd_tilt = np.zeros(2); cmd_vtilt = np.zeros(2); cmd_atilt = np.zeros(2)
    cmd_z = ready_z; cmd_vz = 0.0; cmd_az = 0.0
    est.reset(); t0 = float(plant.data.time)
    frozen_xy = None; settling = False; seat_v0 = None
    for _k in range(int(1.0 / CONTROL_DT)):
        true_m = np.asarray(plant.get_ball_state(0).position_mm, float) / 1000.
        est.add(float(plant.data.time) - t0, noise.observe(true_m * 1000.) / 1000.)
        if est.n < 4:
            pose, sl = realize_tilted(cmd_xy, cmd_z, cmd_tilt[0], cmd_tilt[1])
            plant.command(plant.pose_to_extensions(pose))
            plant.step(CONTROL_DT, hand_cmd_fn=lambda _t, s=sl: s,
                       plat_cmd_fn=lambda _t, p=pose: plant.pose_to_extensions(p))
            continue
        pe, ve = est.estimate()
        t_td, pos_td, vel_td = ballistic_touchdown(pe, ve, CATCH_Z)
        pos_td = np.array(pos_td, float); pos_td[:2] = np.clip(pos_td[:2], -WORKSPACE_XY_M, WORKSPACE_XY_M)
        v_arr = abs(float(vel_td[2]))
        rx, ry = tilt_to_receive(vel_td, MAX_TILT_DEG)
        if frozen_xy is None:
            frozen_xy = pos_td[:2].copy()
        cmd_xy, cmd_vxy, cmd_axy = _quintic_step(cmd_xy, cmd_vxy, cmd_axy, frozen_xy,
                                                 np.zeros(2), np.zeros(2), 0.10, CONTROL_DT)
        htilt = max(t_td, 0.10)
        cmd_tilt, cmd_vtilt, cmd_atilt = _quintic_step(cmd_tilt, cmd_vtilt, cmd_atilt,
                                                       np.array([rx, ry]), np.zeros(2), np.zeros(2), htilt, CONTROL_DT)
        # PHASE-MATCHED Z: arrive at catch_z moving down at ratio*v_arr @ t_td.
        ball_z = true_m[2]
        if not settling and ball_z > CATCH_Z + 0.002 and t_td > 1e-3:
            (cmd_z,), (cmd_vz,), (cmd_az,) = _quintic_step(
                np.array([cmd_z]), np.array([cmd_vz]), np.array([cmd_az]),
                np.array([CATCH_Z]), np.array([-ratio * v_arr]), np.array([0.0]),
                max(t_td, 3 * CONTROL_DT), CONTROL_DT)
        else:
            if not settling:
                settling = True; seat_v0 = min(cmd_vz, -1e-6)
            a_dec = -seat_v0 / seat_decel_t
            new_vz = min(cmd_vz + a_dec * CONTROL_DT, 0.0)
            cmd_z = cmd_z + 0.5 * (cmd_vz + new_vz) * CONTROL_DT
            cmd_vz, cmd_az = new_vz, a_dec
        pose, sl = realize_tilted(cmd_xy, cmd_z, cmd_tilt[0], cmd_tilt[1])
        plant.command(plant.pose_to_extensions(pose))
        plant.step(CONTROL_DT, hand_cmd_fn=lambda _t, s=sl: s,
                   plat_cmd_fn=lambda _t, p=pose: plant.pose_to_extensions(p))
    bs = plant.get_ball_state(0)
    cup = plant.data.site_xpos[sid].copy()
    d = float(np.linalg.norm(np.asarray(bs.position_mm, float) / 1000. - cup)) * 1000.
    return bool(bs.held), d


def _arrival_speed(spawn_fn):
    ap, av = spawn_fn(JuggleNoise(NoiseConfig(0.0, 0.0), seed=0))
    _, _, vel_td = ballistic_touchdown(np.asarray(ap) / 1000., np.asarray(av) / 1000., CATCH_Z)
    return abs(float(vel_td[2]))


def main():
    out_dir = os.path.join(_repo_root, 'temp', 'probes')
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, 'juggle_fastcatch.csv')
    n_seeds = 6
    print(f"Fast-catch fidelity — phase-matched descent (ready_lift={READY_LIFT}, "
          f"ratio={RATIO}), held/{n_seeds}\n")
    rows = []
    for label, fn in [("gentle-lob", gentle_lob_apex), ("real-BB", real_bb_apex)]:
        v = _arrival_speed(fn)
        print(f"### {label}  (arrival ~{v:.2f} m/s vz) ###")
        for cname, contact in [("soft", SOFT), ("firm", FIRM)]:
            res = [run_catch(fn, contact, READY_LIFT, RATIO, SEAT_DECEL_T, s) for s in range(n_seeds)]
            held = sum(int(h) for h, _ in res)
            offs = [round(d, 1) for _, d in res]
            print(f"  {cname:4s}: held {held}/{n_seeds}  offset_mm={offs}")
            for s, (h, d) in enumerate(res):
                rows.append(dict(spawn=label, arrival_vz_mps=round(v, 3), contact=cname,
                                 ready_lift=READY_LIFT, ratio=RATIO, seed=s,
                                 held=int(h), offset_mm=round(d, 1)))
        print()
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f"wrote {csv_path}")


if __name__ == '__main__':
    main()
