#!/usr/bin/env python3
"""Offline model: hand-ODrive PASSTHROUGH cascade, torque-FF on/off.

Answers, WITHOUT flashing anything, whether the planned hand torque
feedforward (`c2ff_spec.md`'s `tau_ff = K * J_HAND * 2*pi * a_cmd`) actually
closes the measured/commanded velocity gap the streamed lane opened when the
Platform Teensy's ~70% feedforward was retired.

Motivation: `logbook/...` (2026-09-14 skill-stack R3 apex-scoped findings —
see MEMORY `project_two_ball_skill_stack.md`) measured the streamed lane at
FF=0 landing 1.02-1.27x commanded peak velocity, 23-54 A peak iq, vs. the
retired Platform-Teensy era's 0.96-1.00x at 25-38 A. This probe builds the
same REAL self-toss plans `tools/probes/../.. scratchpad probe_hand_c2.py`
used (through `jugglebot.motion.skills.executor.install_segment`, the exact
path `trajectory_node` drives), resamples them the way the CURRENT can-bridge
firmware (FW21, off-knot arrival phase, no torque FF) and the FUTURE
knot-aligned scheduled lane would, and drives a discrete-time model of the
hand ODrive's own PASSTHROUGH position->velocity->torque cascade (gains read
from the flashed `odrive_pro_hand_config.json`) against a rigid-body hand+ball
plant, at K in {0, 0.7, 1.0} torque-FF gain.

Run from the repo root with the project venv active:
    python3 tools/probes/hand_cascade_ff_model.py

Outputs (timestamped, under temp/probes/hand_cascade_ff/):
    hand_cascade_ff_<UTC>.csv   -- one row per (apex, phase, K) case
    hand_cascade_ff_<UTC>.md    -- the same table plus the pre-registered
                                   comparison verdict, human-readable
"""
from __future__ import annotations

import csv
import datetime
import json
import math
import os
import sys

REPO = '/home/jetson/Desktop/Jugglebot-skills'
for _p in (os.path.join(REPO, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(REPO, 'config', 'generated'), REPO):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory.limits import TrajectoryLimits
from jugglebot.motion.trajectory.cycle_plan import _hermite as cp_hermite
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import sites as si

assert hw.__file__.startswith(REPO), 'picked up the wrong hardware_config: %s' % hw.__file__

DT = float(hw.JB_TRAJ_KNOT_DT_S)          # 0.025 s -- cup grid AND emitter tick
SEPARATION_MM = 100.0

# Given constants (U1 spec). Cross-checked against the generated config below.
J_HAND = 1.05e-5           # kg*m^2, hand rotor+slider reflected inertia
J_BALL = 2.41e-6           # kg*m^2, ball's contribution while held
KT_HAND = 0.0055133        # N*m/A

assert abs(hw.HAND_ENV_MEASURED_REFLECTED_INERTIA_KGM2 - J_HAND) < 1e-12
assert abs(hw.HAND_ENV_HAND_TORQUE_CONSTANT_NM_PER_A - KT_HAND) < 1e-12

GRAVITY_HOLD_CURRENT_A = float(hw.HAND_ENV_GRAVITY_HOLD_CURRENT_A)  # 1.5 A

ODRIVE_JSON = os.path.join(REPO, 'config', 'ODrive config Files',
                            'odrive_pro_hand_config.json')

TICK_HZ = 500.0            # can-bridge -> ODrive CAN setpoint update rate
SUB_HZ = 8000.0             # ODrive internal cascade rate (PASSTHROUGH)
SUB_DT = 1.0 / SUB_HZ

OUT_DIR = os.path.join(REPO, 'temp', 'probes', 'hand_cascade_ff')


# ─────────────────────────────────────────────────────────────────────────
# 0) ODrive gains -- read from the FLASHED config JSON, not hand-transcribed.
# ─────────────────────────────────────────────────────────────────────────

def load_odrive_gains():
    with open(ODRIVE_JSON) as fh:
        d = json.load(fh)
    ax = d['axis0']
    ctrl = ax['controller']['config']
    mot = ax['config']['motor']
    tau_max = min(float(mot['current_soft_max']) * KT_HAND,
                  float(ax['config']['torque_soft_max']))
    return dict(
        pos_gain=float(ctrl['pos_gain']),                       # 1/s
        vel_gain=float(ctrl['vel_gain']),                       # N*m / (rev/s)
        vel_integrator_gain=float(ctrl['vel_integrator_gain']), # N*m*s / rev
        vel_limit_rps=float(ctrl['vel_limit']),                 # rev/s
        current_soft_max_a=float(mot['current_soft_max']),
        torque_soft_max_nm=float(ax['config']['torque_soft_max']),
        tau_max_nm=tau_max,
        encoder_bandwidth_hz=float(ax['config']['encoder_bandwidth']),
        input_torque_scale=float(ax['config']['can']['input_torque_scale']),
        control_mode=int(ctrl['control_mode']),
        input_mode=int(ctrl['input_mode']),
    )


# ─────────────────────────────────────────────────────────────────────────
# 1) Build real self-toss attempts through install_segment (proven path,
#    lifted from scratchpad probe_hand_c2.py's build_attempt/hand_state_at).
# ─────────────────────────────────────────────────────────────────────────

def geom():
    return StewartGeometry()


def limits_r3():
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=300.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=150000.0,
        hand_acc_rps2=3500.0)


def rest_state(cup_mm, cfg=None):
    cfg = cr.RealizeConfig() if cfg is None else cfg
    slider_mm = float(cup_mm[2]) - cfg.cup_z_base_mm
    rev = (slider_mm - cfg.slider_rev_zero_mm) / 1000.0 * cr.HAND_REV_PER_M
    pose = np.array([cup_mm[0], cup_mm[1], cfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, cfg)


def build_attempt(apex_m, n_throws, geo, lim, t0=1789263419.5, dwell_s=0.30):
    site = si.columns_sites(SEPARATION_MM)[0]
    sched = sc.compile_self_toss(
        sc.SelfTossPattern(site=site, apex_m=apex_m, dwell_s=dwell_s,
                           n_throws=n_throws), t0_abs_s=t0)
    arrival = np.array([0.0, 0.0, -0.5 * 9806.0 * sched.flight_s])
    landings = [ex.Landing(pos_mm=sk.site.catch_site_mm(), vel_mm_s=arrival.copy(),
                            t_land_abs_s=float(sk.t_abs_s))
                for sk in sched.skills if sk.kind == sg.CATCH]

    clock = {'t': t0 - sc.FLOOR_LIFT_S - 1.0}

    def tracker(ball_id):
        for land in landings:
            if land.t_land_abs_s > clock['t'] - 0.05:
                return land
        return None

    state = {'record': None}
    events = []

    def installer(kind, terminal, t_now_s, ball_id=0):
        old_rec = state['record']
        seed = rest_state(site.rest_site_mm()) if old_rec is None else None
        new_rec, res, seg = ex.install_segment(
            old_rec, seed, kind, terminal, t_now_s, limits=lim, geom=geo)
        assert res.accepted, '%s: %s' % (res.code, res.message)
        events.append(dict(kind=kind, res=res, seg=seg, old_rec=old_rec,
                            new_rec=new_rec))
        state['record'] = new_rec
        return res

    execu = ex.SkillExecutor(sched, installer, tracker=tracker)
    t_end = max(sk.t_abs_s for sk in sched.skills) + 0.5
    t = clock['t']
    while t < t_end and not execu.attempt_ended:
        clock['t'] = t
        execu.tick(t)
        t += DT / 10.0

    assert not execu.attempt_ended, execu.end_code
    assert len(events) == len(sched.skills)
    throw0_t0 = events[1]['res'].t0_s
    chain_idx = max(i for i, e in enumerate(events) if e['res'].t0_s == throw0_t0)
    chain_record = events[chain_idx]['new_rec']
    return sched, events, chain_record


def hand_state_at(plan, t):
    """(pos rev, vel rev/s, acc rev/s^2) of the hand channel at plan time t."""
    t = float(t)
    if t >= plan.total_duration:
        return float(plan.hand_rev[-1]), 0.0, 0.0
    k, s = plan._locate(t)
    p, v, a = cp_hermite(float(plan.hand_rev[k]), float(plan.hand_vel_rps[k]),
                          float(plan.hand_rev[k + 1]), float(plan.hand_vel_rps[k + 1]),
                          plan.dt, s)
    return float(p), float(v), float(a)


# ─────────────────────────────────────────────────────────────────────────
# 2) Command streams: FW21-today (off-knot cubic Hermite frames, 500 Hz
#    sub-sampled, ZOH to the ODrive between ticks) and the knot-aligned
#    future lane. phi in {0.5, 0.0} x DT.
# ─────────────────────────────────────────────────────────────────────────

def command_stream(plan, dur, phi):
    """Concatenated (t_abs, p_cmd, v_cmd, a_cmd) covering [phi*DT, dur), one
    25 ms macro-frame at a time, each internally cubic-Hermite-interpolated
    (BCs = the PLAN's own state at the frame's two endpoints) and sampled at
    ~500 Hz -- this is what leg_interp.cpp's 500 Hz interpolator produces
    from a stream of 40 Hz frames, reused verbatim from scratchpad
    probe_hand_c2.py's firmware_cubic_step/resample_report."""
    out = []
    h = DT
    k = 0
    while True:
        tau_k = phi * DT + k * DT
        if tau_k + h > dur:
            break
        p0, v0, _ = hand_state_at(plan, tau_k)
        p1, v1, _ = hand_state_at(plan, tau_k + h)
        sub_t = np.arange(0.0, h - 1e-9, 1.0 / TICK_HZ)
        for st in sub_t:
            s = st / h
            p, v, a = cp_hermite(p0, v0, p1, v1, h, s)
            out.append((tau_k + st, p, v, a))
        k += 1
    return out


# ─────────────────────────────────────────────────────────────────────────
# 3) Ball-held state from the plan's own release/catch marks.
# ─────────────────────────────────────────────────────────────────────────

def held_schedule(meta):
    """Sorted (t_s, delta) events: -1 at a release (ball leaves), +1 at a
    catch (ball arrives). Held starts True (chain opens with a ball in
    hand)."""
    events = [(float(r.t_s), -1) for r in meta.releases]
    events += [(float(c.t_s), +1) for c in meta.catches]
    events.sort(key=lambda e: e[0])
    return events


def held_at(events, t):
    held = True
    for t_e, delta in events:
        if t_e <= t:
            held = (delta == +1)
        else:
            break
    return held


# ─────────────────────────────────────────────────────────────────────────
# 4) ODrive PASSTHROUGH cascade @ 8 kHz, ZOH between 500 Hz command ticks.
# ─────────────────────────────────────────────────────────────────────────

def simulate_cascade(cmd, gains, K, held_events, up_sign, t_release):
    """cmd: list of (t_abs, p_cmd, v_cmd(=vel_ff), a_cmd) at ~500 Hz.
    K: torque-FF gain (tau_ff = K * J_HAND * 2*pi * a_cmd), or None for the
    FW21-today case (pos+vel_ff, tau_ff hard-zeroed).
    Returns a dict of peak/at-release metrics."""
    pos_gain = gains['pos_gain']
    vel_gain = gains['vel_gain']
    ki = gains['vel_integrator_gain']
    vel_limit = gains['vel_limit_rps']
    tau_max = gains['tau_max_nm']
    bw = gains['encoder_bandwidth_hz']
    lpf_alpha = 1.0 - math.exp(-2.0 * math.pi * bw * SUB_DT)

    gravity_bias_nm = -up_sign * GRAVITY_HOLD_CURRENT_A * KT_HAND

    pos = float(cmd[0][1])
    vel_true = float(cmd[0][2])
    vel_est = vel_true
    integ = 0.0

    peak_vel_est = 0.0
    peak_vel_cmd = 0.0
    peak_iq = 0.0
    peak_pos_lag = 0.0
    v_est_at_release = None
    v_cmd_at_release = None

    n = len(cmd)
    for i in range(n):
        t_i, p_cmd, v_cmd, a_cmd = cmd[i]
        t_next = cmd[i + 1][0] if i + 1 < n else t_i + 1.0 / TICK_HZ
        n_sub = max(1, int(round((t_next - t_i) * SUB_HZ)))

        peak_vel_cmd = max(peak_vel_cmd, abs(v_cmd))
        peak_pos_lag = max(peak_pos_lag, abs(p_cmd - pos))

        tau_ff = 0.0 if K is None else K * J_HAND * 2.0 * math.pi * a_cmd

        for _ in range(n_sub):
            held = held_at(held_events, t_i)
            j_total = J_HAND + (J_BALL if held else 0.0)

            vel_sp = pos_gain * (p_cmd - pos) + v_cmd
            vel_sp = max(-vel_limit, min(vel_limit, vel_sp))
            err = vel_sp - vel_est
            tau_cmd = vel_gain * err + integ + tau_ff
            tau_cmd = max(-tau_max, min(tau_max, tau_cmd))
            integ += ki * err * SUB_DT

            alpha_rps2 = (tau_cmd + gravity_bias_nm) / (j_total * 2.0 * math.pi)
            vel_true += alpha_rps2 * SUB_DT
            pos += vel_true * SUB_DT
            vel_est += lpf_alpha * (vel_true - vel_est)

            iq = tau_cmd / KT_HAND
            peak_iq = max(peak_iq, abs(iq))
            peak_vel_est = max(peak_vel_est, abs(vel_est))
            peak_pos_lag = max(peak_pos_lag, abs(p_cmd - pos))

            t_i += SUB_DT
            if (v_est_at_release is None and t_release is not None
                    and t_i >= t_release):
                v_est_at_release = vel_est
                v_cmd_at_release = v_cmd

    return dict(peak_vel_est=peak_vel_est, peak_vel_cmd=peak_vel_cmd,
                peak_iq=peak_iq, peak_pos_lag=peak_pos_lag,
                v_est_at_release=v_est_at_release,
                v_cmd_at_release=v_cmd_at_release)


# ─────────────────────────────────────────────────────────────────────────
# 5) Main sweep.
# ─────────────────────────────────────────────────────────────────────────

CASES = [('K=0 (== FW21 today: pos+vel_ff, tau_ff=0)', None),
         ('K=0.7', 0.7),
         ('K=1.0', 1.0)]


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    gains = load_odrive_gains()
    geo = geom()
    lim = limits_r3()

    print('ODrive hand gains (from %s):' % os.path.basename(ODRIVE_JSON))
    for k, v in gains.items():
        print('  %-22s %s' % (k, v))
    print('gravity_hold_current_a=%.3f A -> %.5f N*m (HAND_ENV_GRAVITY_HOLD_CURRENT_A)'
          % (GRAVITY_HOLD_CURRENT_A, GRAVITY_HOLD_CURRENT_A * KT_HAND))

    rows = []
    for apex, n_throws in ((0.9, 4), (0.5, 4)):
        sched, events, record = build_attempt(apex, n_throws, geo, lim)
        plan = record.plan
        meta = record.meta
        held_events = held_schedule(meta)
        assert len(meta.releases) >= 1, 'no release marks on the chain plan'
        t_release0 = float(meta.releases[0].t_s)
        _, v_rel0, _ = hand_state_at(plan, t_release0)
        up_sign = 1.0 if v_rel0 >= 0 else -1.0

        # the throw with the largest |commanded release velocity| -- the
        # representative "the throw" release for this apex
        rel_speeds = [(float(r.t_s), abs(hand_state_at(plan, float(r.t_s))[1]))
                      for r in meta.releases]
        t_release_rep = max(rel_speeds, key=lambda x: x[1])[0]

        for phase_label, phi in (('off-knot (phi=0.5dt, FW21 today)', 0.5),
                                  ('knot-aligned (phi=0, future)', 0.0)):
            cmd = command_stream(plan, plan.total_duration, phi)
            for k_label, K in CASES:
                res = simulate_cascade(cmd, gains, K, held_events, up_sign,
                                        t_release_rep)
                ratio_peak = (res['peak_vel_est'] / res['peak_vel_cmd']
                              if res['peak_vel_cmd'] > 1e-9 else float('nan'))
                ratio_rel = (abs(res['v_est_at_release']) / abs(res['v_cmd_at_release'])
                             if res['v_cmd_at_release'] not in (None, 0.0) else float('nan'))
                row = dict(apex_m=apex, phase=phase_label, K=k_label,
                           peak_vel_cmd_rps=res['peak_vel_cmd'],
                           peak_vel_est_rps=res['peak_vel_est'],
                           peak_meas_cmd_ratio=ratio_peak,
                           v_cmd_at_release_rps=res['v_cmd_at_release'],
                           v_est_at_release_rps=res['v_est_at_release'],
                           release_meas_cmd_ratio=ratio_rel,
                           peak_iq_a=res['peak_iq'],
                           peak_pos_lag_rev=res['peak_pos_lag'])
                rows.append(row)
                print(('apex=%.1fm %-32s %-8s peak_meas/cmd=%.3f (peak_v_est=%.1f '
                       'peak_v_cmd=%.1f rps) release_meas/cmd=%.3f peak_iq=%.2fA '
                       'peak_pos_lag=%.4frev')
                      % (apex, phase_label, k_label, ratio_peak,
                         res['peak_vel_est'], res['peak_vel_cmd'], ratio_rel,
                         res['peak_iq'], res['peak_pos_lag']))

    stamp = datetime.datetime.utcnow().strftime('%Y%m%dT%H%M%SZ')
    csv_path = os.path.join(OUT_DIR, 'hand_cascade_ff_%s.csv' % stamp)
    fieldnames = list(rows[0].keys())
    with open(csv_path, 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print('wrote', csv_path)

    # ---- pre-registered comparison, off-knot (today's stream) rows only ----
    def sel(k_label):
        return [r for r in rows if r['K'] == k_label
                and r['phase'].startswith('off-knot')]

    k0 = sel('K=0 (== FW21 today: pos+vel_ff, tau_ff=0)')
    k07 = sel('K=0.7')
    k0_ratios = [r['peak_meas_cmd_ratio'] for r in k0]
    k07_ratios = [r['peak_meas_cmd_ratio'] for r in k07]

    supported = (min(k0_ratios) >= 1.02) and all(r <= 1.02 for r in k07_ratios)
    # per spec: K=0 lands inside-or-above 1.02, K=0.7 lands at <=1.00+-0.02
    supported = (min(k0_ratios) >= 1.02) and (max(k07_ratios) <= 1.02)

    md_path = os.path.join(OUT_DIR, 'hand_cascade_ff_%s.md' % stamp)
    with open(md_path, 'w') as f:
        f.write('# Hand cascade + torque-FF offline model\n\n')
        f.write('CSV: `%s`\n\n' % csv_path)
        f.write('| apex | phase | K | peak meas/cmd | peak v_est | peak v_cmd | '
                'release meas/cmd | peak iq (A) | peak pos lag (rev) |\n')
        f.write('|---|---|---|---|---|---|---|---|---|\n')
        for r in rows:
            f.write('| %.1f | %s | %s | %.3f | %.1f | %.1f | %.3f | %.2f | %.4f |\n'
                    % (r['apex_m'], r['phase'], r['K'], r['peak_meas_cmd_ratio'],
                       r['peak_vel_est_rps'], r['peak_vel_cmd_rps'],
                       r['release_meas_cmd_ratio'], r['peak_iq_a'],
                       r['peak_pos_lag_rev']))
        f.write('\n## Pre-registered comparison (off-knot stream, matching FW21 today)\n\n')
        f.write('Measured streamed lane, FF=0 (2026-09-13): peak meas/cmd 1.02-1.27, '
                'peak iq 23-54 A, ~128 rev/s commanded.\n\n')
        f.write('Platform-Teensy era (~70%% FF): 0.96-1.00, 25-38 A.\n\n')
        f.write('K=0 peak meas/cmd range: %.3f-%.3f\n\n' % (min(k0_ratios), max(k0_ratios)))
        f.write('K=0.7 peak meas/cmd range: %.3f-%.3f\n\n' % (min(k07_ratios), max(k07_ratios)))
        f.write('**Verdict: %s**\n' % ('SUPPORTED' if supported else 'NOT SUPPORTED'))
    print('wrote', md_path)

    print()
    print('VERDICT: %s' % ('SUPPORTED' if supported else 'NOT SUPPORTED'))
    print('  K=0   off-knot peak meas/cmd range: %.3f - %.3f (want >= 1.02)'
          % (min(k0_ratios), max(k0_ratios)))
    print('  K=0.7 off-knot peak meas/cmd range: %.3f - %.3f (want <= 1.00+-0.02)'
          % (min(k07_ratios), max(k07_ratios)))
    print()
    print('SKIPPED (optional, time-boxed): Platform-style trapezoidal-accel '
          'throw from git show 1e2c0c9^:.../Trajectory.h -- not modelled.')


if __name__ == '__main__':
    main()
