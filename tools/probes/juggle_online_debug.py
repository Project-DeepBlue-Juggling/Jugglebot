#!/usr/bin/env python
"""Debug harness for the online re-planning juggle runner (`sim/juggle_online.py`).

Two analyses for the open integration issues (see the throw-separation
re-diagnosis in logbook 2026-06-27-online-replanning-architecture-and-cup-
bandlimit and the runner's STATUS docstring):

  * **trajectory** — per-control-tick cup + ball x/z log over the first few
    cycles, to see the carry / throw / catch phases and where a ball goes.
  * **separation** — cup z-velocity/acceleration vs the ball's z-velocity
    through the FIRST throw, flagging where ``a_cup < -g``. This is how the two
    throw blockers were found: with the switched contact the cup *cohesively
    drags* the ball back down (ball reaches ~4.5 m/s up then is decelerated to 0
    WITH the cup at ~169 m/s² ≫ g); ``--stiff-always`` forces genuinely stiff
    contact, which makes the ball fly free (and catches it) — the A/B test that
    proved cohesion (not a "bounded slider") is the blocker. Stiff-always also
    exposes the SLAM (launch ~16.8 m/s ≫ planned 5) from the carry pre-roll
    leaving the cup low — the second thing to fix.

Usage (from the repo root):
  python tools/probes/juggle_online_debug.py                 # switched contact
  python tools/probes/juggle_online_debug.py --stiff-always  # A/B: stiff
  python tools/probes/juggle_online_debug.py --duration 2.2

Outputs to stdout. Headless MuJoCo plant, no hardware, side-effect-free.
"""
from __future__ import annotations

import argparse
import os
import sys
import types

import numpy as np

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_REPO, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_REPO, 'config', 'generated')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import mujoco
from sim.juggle_online import OnlineJuggleRunner, OnlineJuggleConfig

CONTROL_DT = 0.025


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=1.6)
    ap.add_argument('--stiff-always', action='store_true',
                    help="force stiff contact (disable the switch) for the A/B test")
    args = ap.parse_args()

    r = OnlineJuggleRunner(OnlineJuggleConfig(duration_s=args.duration))
    plant, sid = r.plant, r.site_id
    if args.stiff_always:
        plant.set_contact_stiffness(True)
        plant.set_contact_stiffness = lambda stiff: None    # no-op so the loop can't soften it

    rows = []
    _orig = plant.step

    def wrapped(self, dt, hand_cmd_fn=None, plat_cmd_fn=None):
        _orig(dt, hand_cmd_fn=hand_cmd_fn, plat_cmd_fn=plat_cmd_fn)
        cup = plant.data.site_xpos[sid].copy() * 1000.0
        b0 = plant.get_ball_state(ball=0)
        b1 = plant.get_ball_state(ball=1)
        rows.append((float(plant.data.time), cup.copy(),
                     np.asarray(b0.position_mm).copy(), float(b0.velocity_mms[2]), int(b0.held),
                     np.asarray(b1.position_mm).copy(), int(b1.held)))

    plant.step = types.MethodType(wrapped, plant)
    caps, drops = r.run()

    print(f"mode={'STIFF-ALWAYS' if args.stiff_always else 'switched'}  "
          f"captures={len(caps)} drops={len(drops)}  v_take_z={r.v_take[2]:.2f} m/s")

    print("\n-- trajectory (cup/ball x,z mm; h=held) --")
    print("  t      cup[x,z]      b0[x,z]h        b1[x,z]h")
    for i in range(0, len(rows), 3):
        t, cup, b0, _v0, h0, b1, h1 = rows[i]
        print(f"  {t:.3f}  [{cup[0]:5.0f}{cup[2]:5.0f}]  [{b0[0]:6.0f}{b0[2]:6.0f}]{h0}  "
              f"[{b1[0]:6.0f}{b1[2]:6.0f}]{h1}")

    # separation analysis around the first throw (held 1->0 on ball 0)
    thr = next((i for i in range(1, len(rows)) if rows[i - 1][4] == 1 and rows[i][4] == 0), None)
    if thr is None:
        print("\n-- no throw (ball 0 never un-held) --")
        return
    print(f"\n-- separation @ first throw (t={rows[thr][0]:.3f}); -g=-9.8 m/s² --")
    print("  t      cup_z  cup_vz  cup_az   ball_z  ball_vz  gap   note")
    for i in range(max(2, thr - 2), min(thr + 16, len(rows))):
        t, cup, b0, bvz, h, _b1, _h1 = rows[i]
        cz, bz = cup[2], b0[2]
        cvz = (cz - rows[i - 1][1][2]) / 1000.0 / CONTROL_DT
        cvz_p = (rows[i - 1][1][2] - rows[i - 2][1][2]) / 1000.0 / CONTROL_DT
        caz = (cvz - cvz_p) / CONTROL_DT
        note = "a_cup<-g" if caz < -9.8 else ""
        note += "  FLIES" if (bz - cz) > 50 else ""
        print(f"  {t:.3f}  {cz:5.0f}  {cvz:+5.2f}  {caz:+7.0f}   {bz:5.0f}  {bvz/1000:+5.2f}  "
              f"{bz-cz:4.0f}   {note}")


if __name__ == '__main__':
    main()
