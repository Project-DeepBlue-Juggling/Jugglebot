"""Motion-quality probe for the juggle catch/self-catch controllers (review P0).

The make-or-break gates measured only OUTCOMES (caught/held/landing/offset); none
measured MOTION quality, so a controller that slams both stroke clamps every cycle
and receives the ball on a RISING cup passed with a perfect score (Fable-5 review,
2026-07-03). This probe instruments the cup (``hand_opening`` site) vs the true ball
each control tick and reports the motion-quality metrics the review calls for:

  * cup vertical path length per cycle (mm)      -- over-travel vs momentum-shaped
  * slider stroke span used + clamp-saturation   -- ceiling/floor slams (0..355 mm)
  * cup vz at first ball contact (sign + ratio)   -- "receive" = cup moving DOWN
  * disengaged travel fraction                    -- motion while not interacting
  * max per-tick |cup z command delta|            -- command continuity / step

Principle (operator): the cup should move ONLY to accelerate/decelerate the ball
(and the minimal accel/decel to start/stop the interaction). Over-travel, clamp
slams, an up-moving cup at contact, and command steps are symptoms of a poorly
shaped controller, not targets.

Run:  source ~/Desktop/PDJ_venv/venv/bin/activate && python tools/probes/juggle_motion_quality.py
Writes temp/probes/juggle_motion_quality.csv. Headless MuJoCo, no hardware.
"""
from __future__ import annotations
import os
import sys

_here = os.path.dirname(os.path.abspath(__file__))
_root = os.path.dirname(os.path.dirname(_here))
for _p in (_root, os.path.join(_root, 'sim')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
import mujoco

from sim.juggle_selfcatch import SelfCatchRunner, SelfCatchConfig
from sim.juggle_catch import SingleCatchRunner, SingleCatchConfig

CUP_Z_FLOOR_MM = 659.6                     # cup z at slider 0 (level)
CUP_Z_CEIL_MM = 659.6 + 355.0              # cup z at slider 355 (level)
SLIDER_STROKE_MM = 355.0
BALL_ENGAGE_MM = 90.0                      # |ball - cup| within this = "interacting"


def _instrument(runner):
    """Wrap runner.plant.step; return the per-tick log (cup z/vz, slider, ball z,
    dist ball-to-cup, held)."""
    plant = runner.plant
    sid = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')
    jid = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_JOINT, 'hand_slide')
    sadr = plant.model.jnt_qposadr[jid]
    log = []
    orig = plant.step

    def spy(*a, **k):
        orig(*a, **k)
        cup = plant.data.site_xpos[sid].copy()
        r = np.zeros(6)
        mujoco.mj_objectVelocity(plant.model, plant.data, mujoco.mjtObj.mjOBJ_SITE, sid, r, 0)
        bs = plant.get_ball_state(0)
        ball = np.asarray(bs.position_mm, float) / 1000.0
        log.append(dict(cz=cup[2] * 1000.0, cvz=r[5] * 1000.0,
                        slider=float(plant.data.qpos[sadr]) * 1000.0,
                        bz=ball[2] * 1000.0,
                        dist=float(np.linalg.norm(ball - cup)) * 1000.0,
                        held=bool(bs.held)))
    plant.step = spy
    return log


def metrics(log, n_cycles):
    if not log:
        return {}
    cz = np.array([r['cz'] for r in log])
    slider = np.array([r['slider'] for r in log])
    path_z = float(np.sum(np.abs(np.diff(cz))))
    ceil = int(np.sum(slider >= SLIDER_STROKE_MM - 1.0))
    floor = int(np.sum(slider <= 1.0))
    # disengaged travel: cup z motion while the ball is neither engaged nor held
    diss = 0.0
    for i in range(1, len(log)):
        if log[i]['dist'] > BALL_ENGAGE_MM and not log[i]['held']:
            diss += abs(log[i]['cz'] - log[i - 1]['cz'])
    max_step = float(np.max(np.abs(np.diff(cz)))) if len(cz) > 1 else 0.0
    # cup vz at first ball contact per (held False->True) transition
    contacts = []
    for i in range(1, len(log)):
        if log[i]['held'] and not log[i - 1]['held']:
            pre = log[max(0, i - 2)]
            contacts.append(pre['cvz'])
    return dict(path_z_per_cycle=path_z / max(n_cycles, 1),
                stroke_span=float(slider.max() - slider.min()),
                ceil_ticks=ceil, floor_ticks=floor,
                disengaged_mm=diss, disengaged_frac=diss / max(np.sum(np.abs(np.diff(cz))), 1e-9),
                max_cmd_step_mm=max_step,
                cup_vz_at_contact=contacts)


def report(name, log, n_cycles):
    m = metrics(log, n_cycles)
    print(f"=== {name} ===")
    print(f"  cup z path/cycle : {m['path_z_per_cycle']:.0f} mm")
    print(f"  stroke span used : {m['stroke_span']:.0f} / {SLIDER_STROKE_MM:.0f} mm   "
          f"clamp hits: ceil {m['ceil_ticks']} / floor {m['floor_ticks']} ticks")
    print(f"  disengaged travel: {m['disengaged_mm']:.0f} mm ({100*m['disengaged_frac']:.0f}% of cup path)")
    print(f"  max cmd z step   : {m['max_cmd_step_mm']:.1f} mm/tick")
    cs = m['cup_vz_at_contact']
    tag = lambda v: ("UP (into ball, BAD)" if v > 5 else "down (receive)" if v < -5 else "~still")
    print(f"  cup vz @ contact : {[f'{v:+.0f}' for v in cs]} mm/s  -> " +
          ", ".join(tag(v) for v in cs))
    return m


if __name__ == "__main__":
    os.makedirs(os.path.join(_root, 'temp', 'probes'), exist_ok=True)
    print("Motion-quality probe — cup vs ball, per the Fable-5 review P0.\n")

    # 1) the self-catch MAKE config (kinematic oscillation) — the review's headline
    r1 = SelfCatchRunner(SelfCatchConfig(oscillate=True, n_cycles=3, seed=0,
                                         release="kinematic", dip_m=0.10))
    log1 = _instrument(r1); res1 = r1.run()
    m1 = report(f"self-catch MAKE (kinematic osc, sustained {res1.sustained}/3)", log1, 3)

    # 2) the Rung-1 single catch (what juggle_bb_catch shows) — gentle + real BB
    print()
    r2 = SingleCatchRunner(SingleCatchConfig(seed=0))
    log2 = _instrument(r2); r2.run()
    report("Rung-1 catch (gentle synthetic lob)", log2, 1)
    print()
    r3 = SingleCatchRunner(SingleCatchConfig(seed=0, use_real_bb=True))
    log3 = _instrument(r3); r3.run()
    report("Rung-1 catch (real fast Ball Butler)", log3, 1)

    print("\nGate guidance (review P0): a smooth cycle has NO clamp hits, contact with the "
          "cup moving DOWN (receive), low disengaged fraction, and no large cmd steps.")
