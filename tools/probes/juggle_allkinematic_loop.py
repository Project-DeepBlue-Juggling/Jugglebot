"""Rung-2b ALL-KINEMATIC self-catch loop — the de-risk before the hybrid.

Confirms, in isolation, that a KINEMATIC velocity release CLOSES the throw->catch->
throw loop, decoupled from the contact-carry catch's seat hand-off (which tripped up
the first naive hybrid swap). One consistent kinematic contact model, NO contact
hand-off:

* **throw** — kinematic velocity release (cut the ball free at the cup's planned
  take-off velocity; open-loop knife-edge gain ~0.01, see
  ``tools/probes/juggle_kinematic_release.py``);
* **catch** — kinematic capture (drive the cup to the noisy-observed predicted
  landing; snap the ball to the cup when it descends through catch_z).

Single ball shuttles A<->B (40 mm on x); the throw ORIGIN each cycle is the achieved
caught xy (closed-loop self-correction); §3 tracking noise on the observation.
Finding: sustains **12/12 on all 6 seeds** with a FLAT ~2.4 mm catch-reach-error
trend (loop gain << 1) — vs the contact-detach BREAK (diverges 1-4 cycles). This
de-risked the make-or-break before the faithful hybrid (contact-carry catch +
kinematic release) was built and shown to also sustain 12/12 (see
``sim/juggle_selfcatch.py`` ``release="kinematic"`` and
``tools/probes/juggle_selfcatch_loopgain.py``).

NB in kinematic capture the ball snaps to cup-centre, so the measured disturbance is
the CATCH REACH ERROR (cup-to-ball xy at capture), not a physical in-cup seat offset
— the honest all-kinematic disturbance; the real ~0.6 mm seat offset is a hybrid
concern (rejected there). Was ``/tmp/probe_allkinematic_loop.py`` (this session);
promoted per CLAUDE.md's reusable-probe rule. Headless MuJoCo plant, no hardware.
Writes ``temp/probes/juggle_allkinematic_loop.csv``. Run from the repo root:

    python tools/probes/juggle_allkinematic_loop.py [--seeds N] [--cycles N]

See ``logbook/2026-07-01-rung2b-kinematic-release.md`` (Phase 3 / Rung 2b of
``plans/active/bb-online-juggle-tilt-rearchitecture.md``).
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

_repo_root = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
from sim._paths import bootstrap_paths  # noqa: E402
bootstrap_paths()

import numpy as np
import mujoco

from sim.plant.mujoco_plant import MuJoCoPlant
from sim.juggle_tilt import cup_axis, realize_tilted, tilt_to_throw, MAX_TILT_DEG
from sim.juggle_noise import NoiseConfig, JuggleNoise, BallisticEstimator
from sim.juggle_selfcatch import _QuinticSeg
from sim.juggle_planner.juggle_planner import (
    plan_cup_cycle, takeoff_velocity, ballistic_touchdown, PlannerConfig, GRAVITY)

CONTROL_DT = 0.025
CUP_Z_BASE_MM = 659.6
SLIDER_STROKE_MM = 355.0
THROW_Z = 0.85
CATCH_Z = 0.70
FLIGHT = 0.6
DIP = 0.16
TILT_RAMP = 0.6
A = np.array([-0.02, 0.0])
B = np.array([0.02, 0.0])


def _smoothstep(f):
    f = float(np.clip(f, 0.0, 1.0))
    return f * f * (3.0 - 2.0 * f)


class Loop:
    def __init__(self, seed):
        self.plant = MuJoCoPlant(control_dt=CONTROL_DT, contact_carry=False)
        self.sid = mujoco.mj_name2id(self.plant.model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')
        self.noise = JuggleNoise(NoiseConfig(), seed=seed)
        self.est = BallisticEstimator(GRAVITY)
        self.pcfg = PlannerConfig()
        self.pcfg.z_min_m = CUP_Z_BASE_MM / 1000.0 + 0.005
        self.pcfg.z_max_m = (CUP_Z_BASE_MM + SLIDER_STROKE_MM) / 1000.0 - 0.005

    def cup(self):
        return self.plant.data.site_xpos[self.sid].copy()

    def cupv(self):
        r = np.zeros(6)
        mujoco.mj_objectVelocity(self.plant.model, self.plant.data,
                                 mujoco.mjtObj.mjOBJ_SITE, self.sid, r, 0)
        return r[3:6].copy()

    def _drive(self, seg, t0, n):
        def cmd(t_abs):
            p = seg.sample(t_abs - t0)
            return realize_tilted(p[:2], float(p[2]), 0.0, 0.0)
        for _ in range(n):
            pose, _ = cmd(float(self.plant.data.time))
            self.plant.command(self.plant.pose_to_extensions(pose))
            self.plant.step(CONTROL_DT, hand_cmd_fn=lambda t: cmd(t)[1],
                            plat_cmd_fn=lambda t: self.plant.pose_to_extensions(cmd(t)[0]))

    def carry_release(self, from_xy, to_xy):
        throw_pos = np.array([from_xy[0], from_xy[1], THROW_Z])
        target = np.array([to_xy[0], to_xy[1], CATCH_Z])
        v_takeoff = takeoff_velocity(throw_pos, target, FLIGHT)
        rx, ry = tilt_to_throw(v_takeoff, MAX_TILT_DEG)
        axis = cup_axis(rx, ry)
        period = max(FLIGHT, 0.45)
        catch_time = period * 0.40
        clow = throw_pos - axis * DIP
        clow[2] = max(clow[2], CATCH_Z)
        carry_dur = period - catch_time
        rseg = _QuinticSeg(self.cup(), self.cupv(), np.zeros(3), clow,
                           np.zeros(3), np.zeros(3), 0.20)
        self._drive(rseg, float(self.plant.data.time), int(round(0.20 / CONTROL_DT)))
        ph, sh = realize_tilted(clow[:2], clow[2], 0.0, 0.0)
        for _ in range(10):
            self.plant.command(self.plant.pose_to_extensions(ph))
            self.plant.command_hand(sh)
            self.plant.step(CONTROL_DT)
        plan = plan_cup_cycle(throw_pos, v_takeoff, GRAVITY.copy(), throw_pos, target,
                              FLIGHT, catch_time, clow, np.array([0., 0., -2.0]),
                              period, self.pcfg, detach_axis=axis)
        tc0 = float(self.plant.data.time)

        def cmd(t_abs):
            cp, _, _ = plan.sample(catch_time + (t_abs - tc0))
            s = _smoothstep((t_abs - tc0) / max(TILT_RAMP * carry_dur, 1e-6))
            return realize_tilted(cp[:2], float(cp[2]), rx * s, ry * s)
        for _ in range(int(round(carry_dur / CONTROL_DT))):
            pose, _ = cmd(float(self.plant.data.time))
            self.plant.command(self.plant.pose_to_extensions(pose))
            self.plant.step(CONTROL_DT, hand_cmd_fn=lambda t: cmd(t)[1],
                            plat_cmd_fn=lambda t: self.plant.pose_to_extensions(cmd(t)[0]))
        self.plant.release_ball(self.cupv() * 1000.0)     # KINEMATIC release
        return target[:2]

    def observe_predict(self, n_obs=5):
        self.est.reset()
        for _ in range(n_obs):
            self.plant.step(CONTROL_DT)
            bs = self.plant.get_ball_state(ball=0)
            obs = self.noise.observe(np.asarray(bs.position_mm, float)) / 1000.0
            self.est.add(float(self.plant.data.time), obs)
        pe, ve = self.est.estimate()
        _, td, _ = ballistic_touchdown(pe, ve, CATCH_Z)
        return td[:2]

    def catch(self, predicted_xy, target_xy):
        cup_p = self.cup(); cup_v = self.cupv()
        tgt = np.array([predicted_xy[0], predicted_xy[1], CATCH_Z])
        seg = _QuinticSeg(cup_p, cup_v, np.zeros(3), tgt, np.zeros(3), np.zeros(3), 0.28)
        t0 = float(self.plant.data.time)
        n_drive = int(round(0.28 / CONTROL_DT))
        holdpose, holdsl = realize_tilted(tgt[:2], CATCH_Z, 0.0, 0.0)
        last_free_xy = self.cup()[:2]
        for k in range(n_drive + 60):
            bs = self.plant.get_ball_state(ball=0)
            if bs.held:
                land_err = float(np.linalg.norm(last_free_xy - np.asarray(target_xy)) * 1000.0)
                return True, land_err, self.cup()[:2]
            bp = np.asarray(bs.position_mm, float) / 1000.0
            last_free_xy = bp[:2].copy()
            if k < n_drive:
                def cmd(t_abs):
                    pp = seg.sample(t_abs - t0)
                    return realize_tilted(pp[:2], float(pp[2]), 0.0, 0.0)
                pose, _ = cmd(float(self.plant.data.time))
                self.plant.command(self.plant.pose_to_extensions(pose))
                self.plant.step(CONTROL_DT, hand_cmd_fn=lambda t: cmd(t)[1],
                                plat_cmd_fn=lambda t: self.plant.pose_to_extensions(cmd(t)[0]))
            else:
                self.plant.command(self.plant.pose_to_extensions(holdpose))
                self.plant.command_hand(holdsl)
                self.plant.step(CONTROL_DT)
        return False, float('nan'), self.cup()[:2]

    def run(self, n_cycles=12):
        self.plant.reset()
        throw0 = np.array([A[0], A[1], THROW_Z])
        axis0 = cup_axis(*tilt_to_throw(
            takeoff_velocity(throw0, np.array([B[0], B[1], CATCH_Z]), FLIGHT), MAX_TILT_DEG))
        clow0 = throw0 - axis0 * DIP
        clow0[2] = max(clow0[2], CATCH_Z)
        pose0, sl0 = realize_tilted(clow0[:2], clow0[2], 0.0, 0.0)
        for _ in range(70):
            self.plant.command(self.plant.pose_to_extensions(pose0))
            self.plant.command_hand(sl0)
            self.plant.step(CONTROL_DT)
        self.plant.ball_manager.spawn_in_hand()
        cup_xy = A.copy()
        land_errs = []
        sustained = 0
        for cyc in range(n_cycles):
            to_xy = B if cyc % 2 == 0 else A
            self.carry_release(cup_xy, to_xy)
            pred = self.observe_predict()
            caught, land_err, cup_xy = self.catch(pred, to_xy)
            land_errs.append(round(land_err, 1))
            if not caught:
                break
            sustained += 1
        return sustained, land_errs


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seeds', type=int, default=6)
    ap.add_argument('--cycles', type=int, default=12)
    args = ap.parse_args(argv)
    out_dir = os.path.join(_repo_root, 'temp', 'probes')
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, 'juggle_allkinematic_loop.csv')
    print(f"ALL-KINEMATIC Rung-2b loop — A<->B 40 mm, {args.cycles} cycles, "
          f"kinematic capture+release, §3 noise\n")
    rows = []
    tot = []
    for s in range(args.seeds):
        sus, errs = Loop(s).run(args.cycles)
        tot.append(sus)
        print(f"  seed {s}: sustained={sus:2d}/{args.cycles}   landing-err trend(mm)={errs}")
        rows.append(dict(seed=s, sustained=sus, land_err_trend=errs))
    print(f"\n  mean sustained: {np.mean(tot):.1f}/{args.cycles}   "
          f"(contact-detach BREAK: diverges 1-4 cycles)")
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f"wrote {csv_path}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
