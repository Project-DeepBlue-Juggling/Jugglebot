#!/usr/bin/env python3
"""Leg-plant system-ID from recorded bench stroke CSVs — offline, no hardware.

WHAT THIS CHARACTERISES
-----------------------
Two things that the bench gain ladder never measured, both recoverable from
telemetry that ``bench_leg_sysid.py --mode strokes`` already writes:

1. **The plant** — effective inertia ``J_eff``, Coulomb friction ``tau_c`` and
   viscous friction ``b``, by regressing the rigid-body torque balance

       2*pi*J_eff*a  =  Kt*iq  -  tau_c*sgn(v)  -  b*v

   on the (a, v, iq) triples in each stroke CSV. ``a`` is the centred
   difference of a lightly-smoothed ``vel_rps``. Only clearly-moving samples
   (|v| > V_MOVING) are used, so ``sgn(v)`` is well-defined and the stiction
   regime does not pollute the kinetic-friction fit.

   ``J_eff`` is the number the whole cascade rests on: the velocity-loop
   crossover is ``omega_v = vel_gain / (2*pi*J_eff)`` and the position-loop
   crossover is ``omega_p = pos_gain``. Their ratio is what actually decides
   whether a gain point rings. It had never been measured — the methodology
   carried an unsourced "J_eff ~10x rotor" guess, which this refutes.

2. **The tracking error budget** — decomposing ``err(t) = cmd_u0 - encoder``
   into a delay-shaped term, an inertial term, a friction term and a residual:

       err  ~=  tau_d*v  +  k_a*a  +  k_f*sgn(v)  +  c

   The point of the decomposition is to show how much of the "tracking error"
   is even *attackable by feedback*. The delay term is not: it is gain-
   independent, and at the harness's sample-and-hold cadence a large slice of
   it is the METRIC (``cmd_u0`` is the knot at the START of the segment the
   firmware is interpolating THROUGH, so it lags the executed command by up to
   one knot of travel).

MOTIVATED BY
------------
``logbook/2026-07-13-leg-plant-id-and-the-units-bug.md`` — the session that
ended the leg gain hunt. Run against the Stage-1 stroke batteries in
``temp/probes/bench_sysid_20260712_222801/`` and
``temp/probes/bench_sysid_20260713_091748/``.

THE iq SIGN-FRAME TRAP (do not "fix" this fit)
----------------------------------------------
A code-read says the firmware reports ``iq`` in a DIFFERENT frame from ``pos``/``vel``:
``can_buses.cpp:85-86`` runs pos/vel through ``leg_sign()`` (which NEGATES for legs, giving
Jugglebot's extension-positive convention), while ``:92`` stores ``iq_measured`` RAW in the
ODrive frame. Taken at face value that implies an extending torque should read as NEGATIVE
iq, which would make this regression's ``J_eff`` and ``tau_c`` come out negative.

They do not. **The recorded data is unambiguous and it wins**: over the 2026-07-13 stroke
battery, corr(iq, accel) = +0.39 and corr(iq, sign(vel)) = +0.81; during a +36 rev/s^2
extension the leg draws +0.89 A, during -36 rev/s^2 it draws -0.82 A. So on this rig
**positive iq = extending torque**, in the same frame as pos/vel — the ODrive's own motor/
encoder direction calibration evidently inverts the raw sign back again. Both fitted
quantities come out POSITIVE and ``tau_c`` independently reproduces the 2026-04-27 friction
bench's 1.094 A, which a flipped frame could not do.

Do not "correct" this from the code-read alone. If you port this probe to the assembled
robot's legs, RE-VERIFY the sign empirically first (the correlations above are the check) —
the per-axis direction calibration is a property of each ODrive, not of the firmware.

CAVEATS (read before trusting a number)
---------------------------------------
* ``a`` is a numerical derivative of a 100/250 Hz velocity signal, so the
  J_eff fit is noise-limited (R^2 ~0.7-0.9 is normal and healthy here). It is
  good to maybe +/-20%, which is plenty to settle a 10x hypothesis and to size
  a feedforward, but it is NOT a precision inertia measurement.
* ``Kt`` matters directly (J_eff scales with it). The repo has TWO values:
  ``hardware_config.yaml``'s measured 0.0624 Nm/A (bench fit, R^2=0.994) and
  the ODrive's own ``motor.torque_constant = 0.0551``. This probe uses the
  measured one and reports the sensitivity. Reconcile them before shipping any
  torque feedforward.
* The bench leg is UNLOADED. Gravity, inter-leg coupling and hand reaction
  forces are all absent, so the friction/inertia terms here are a floor, not
  the loaded-robot budget.

USAGE
-----
    python tools/probes/bench_leg_plant_id.py <run_dir> [<run_dir> ...]
    python tools/probes/bench_leg_plant_id.py --all      # every bench_sysid_* run

Writes a JSON summary to ``temp/probes/plant_id_<stamp>.json`` and prints the
tables. Read-only with respect to the input runs.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from datetime import datetime

import numpy as np

_REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                           '..', '..'))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

# Motor torque constant, Nm per ODrive-reported Amp (hardware_config.yaml:60 -- bench fit
# R^2=0.994). NOTE this figure is UNDER RE-MEASUREMENT (2026-07-13): the historical fit may be
# biased HIGH by stiction, and the ODrive's own configured value disagrees (see below).
# J_eff and tau_c[Nm] scale linearly with this; tau_c[A] and the cascade table do NOT.
KT_NM_PER_A = 0.0624
# What is ACTUALLY FLASHED on the ODrives (odrive_pro_leg_config.json:152). This is ODrive's
# default 8.27/Kv formula, = 0.0551 for a 150 Kv motor. The drive divides every commanded torque
# by THIS number to get Iq -- so it, not the true Kt, sets the effective velocity-loop gain.
KT_ODRIVE_CONFIG = 0.055133331567049026
# Datasheet/rotor inertia (hardware_config.yaml:55) -- the baseline J_eff is compared to.
J_ROTOR_KGM2 = 2.75e-4
# Bench-leg geometry (single_leg_test.py:106, measured).
MM_PER_REV = 71.5708
# Below this speed sgn(v) is meaningless and stiction dominates -- excluded from the fit.
V_MOVING_RPS = 0.3
# Boxcar width for the velocity pre-smooth before differentiating.
SMOOTH_N = 5
# Production leg gain triples worth reporting bandwidths for.
GAIN_POINTS = ((40, 0.20), (70, 0.35), (90, 0.45), (110, 0.50))


def _load(path):
    """(t, v, iq, cmd, pos) from a bench stroke CSV, finite rows only."""
    d = np.genfromtxt(path, delimiter=',', names=True)
    cols = {}
    for name, key in (('t', 't_s'), ('v', 'vel_rps'), ('iq', 'iq_A'),
                      ('cmd', 'cmd_rev'), ('pos', 'pos_rev')):
        if key not in (d.dtype.names or ()):
            return None
        cols[name] = np.asarray(d[key], dtype=float)
    m = np.ones_like(cols['t'], dtype=bool)
    for a in cols.values():
        m &= np.isfinite(a)
    if m.sum() < 50:
        return None
    return {k: a[m] for k, a in cols.items()}


def _accel(t, v):
    """Centred difference of a boxcar-smoothed velocity (rev/s^2)."""
    k = np.ones(SMOOTH_N) / float(SMOOTH_N)
    return np.gradient(np.convolve(v, k, mode='same'), t)


def _lstsq(A, b):
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    pred = A @ x
    ss = np.sum((b - b.mean()) ** 2)
    r2 = 1.0 - np.sum((b - pred) ** 2) / ss if ss > 0 else float('nan')
    return x, float(r2), pred


def plant_fit(files):
    """Regress 2*pi*J*a = Kt*iq - tau_c*sgn(v) - b*v  =>  fit iq on (a, sgn v, v, 1)."""
    A, B = [], []
    for p in files:
        d = _load(p)
        if d is None:
            continue
        a = _accel(d['t'], d['v'])
        sel = (np.abs(d['v']) > V_MOVING_RPS) & np.isfinite(a)
        sel[:10] = False
        sel[-10:] = False                       # edge effects of the boxcar
        if sel.sum() < 50:
            continue
        A.append(np.c_[a[sel], np.sign(d['v'][sel]), d['v'][sel], np.ones(sel.sum())])
        B.append(d['iq'][sel])
    if not A:
        return None
    A, B = np.vstack(A), np.concatenate(B)
    x, r2, _ = _lstsq(A, B)
    return {
        'n_samples': int(len(B)),
        'r2': r2,
        'J_eff_kgm2': float(x[0] * KT_NM_PER_A / (2.0 * np.pi)),
        'J_eff_over_J_rotor': float(x[0] * KT_NM_PER_A / (2.0 * np.pi) / J_ROTOR_KGM2),
        'tau_c_Nm': float(x[1] * KT_NM_PER_A),
        'tau_c_A': float(x[1]),
        'b_Nm_per_rps': float(x[2] * KT_NM_PER_A),
        'offset_A': float(x[3]),
    }


def error_budget(files):
    """Decompose err = cmd_u0 - encoder into delay / inertial / friction / residual."""
    A, B = [], []
    for p in files:
        d = _load(p)
        if d is None:
            continue
        a = _accel(d['t'], d['v'])
        sel = np.isfinite(a)
        sel[:10] = False
        sel[-10:] = False
        if sel.sum() < 50:
            continue
        A.append(np.c_[d['v'][sel], a[sel], np.sign(d['v'][sel]), np.ones(sel.sum())])
        B.append((d['cmd'] - d['pos'])[sel])
    if not A:
        return None
    A, B = np.vstack(A), np.concatenate(B)
    x, r2, pred = _lstsq(A, B)

    def rms_mm(z):
        return float(np.sqrt(np.mean(np.asarray(z) ** 2)) * MM_PER_REV)

    return {
        'n_samples': int(len(B)),
        'r2': r2,
        'total_rms_mm': rms_mm(B),
        'delay_term_rms_mm': rms_mm(A[:, 0] * x[0]),
        'tau_d_ms': float(x[0] * 1e3),
        'inertial_term_rms_mm': rms_mm(A[:, 1] * x[1]),
        'friction_term_rms_mm': rms_mm(A[:, 2] * x[2]),
        'const_offset_mm': float(x[3] * MM_PER_REV),
        'residual_rms_mm': rms_mm(B - pred),
    }


def bandwidths(j_eff):
    """Cascade crossovers implied by a measured J_eff. The RATIO is what decides ringing.

    ``omega_p = pos_gain`` [rad/s] -- the position loop outputs a VELOCITY setpoint, so it
    does not involve the torque constant at all.

    ``omega_v`` is subtler, and getting it wrong is easy. On ODrive 0.6.x the velocity
    controller output is TORQUE, and the drive then computes ``Iq = torque / torque_constant``
    using ITS OWN CONFIGURED value (KT_ODRIVE_CONFIG), which is NOT the motor's true Kt.
    (Operator-confirmed 2026-07-13: vel_gain is in Nm/(rev/s).) So the torque the motor
    ACTUALLY produces per unit velocity error is

        tau_per_e_v = vel_gain * (KT_TRUE / KT_ODRIVE_CONFIG)

    i.e. with a config value 11.7% LOW, the shipping velocity loop runs ~13% STIFFER than its
    nominal vel_gain implies. Hence:

        omega_v = vel_gain * (KT_TRUE / KT_ODRIVE_CONFIG) / (2*pi*J_eff)
                = vel_gain / (KT_ODRIVE_CONFIG * x0)          [x0 = d(iq)/d(accel), MEASURED]

    **omega_v is INVARIANT to the true Kt.** J_eff scales linearly with the assumed KT_TRUE and
    x0 scales as 1/KT_TRUE, so it cancels: the cascade conclusion holds no matter how the
    0.0551-vs-0.0624 Kt question lands. Only KT_ODRIVE_CONFIG (what is actually flashed on the
    drive) enters. This is why the cascade table is trustworthy while Kt is still open.

    Using ``vel_gain/(2*pi*J)`` -- i.e. forgetting the torque_constant division -- understates
    omega_v by 13% and is the error that produced the (now corrected) 20.2 Hz / ratio 3.17
    figures first published on 2026-07-13.
    """
    out = []
    for pg, vg in GAIN_POINTS:
        tau_per_e_v = vg * (KT_NM_PER_A / KT_ODRIVE_CONFIG)      # real Nm per (rev/s) of error
        w_v = tau_per_e_v / (2.0 * np.pi * j_eff)                # rad/s
        w_p = float(pg)                                          # rad/s (pos_gain is 1/s)
        out.append({'pos_gain': pg, 'vel_gain': vg,
                    'vel_loop_hz': w_v / (2.0 * np.pi),
                    'pos_loop_hz': w_p / (2.0 * np.pi),
                    'ratio_wv_over_wp': w_v / w_p})
    return out


def analyse_run(run_dir):
    """Group a run's stroke CSVs by pos_gain (from the filename) and fit each."""
    csvs = sorted(glob.glob(os.path.join(run_dir, 'stroke_p*.csv')))
    if not csvs:
        return None
    by_gain = {}
    for p in csvs:
        # stroke_p<gain>_<label>.csv
        tag = os.path.basename(p).split('_')[1]
        by_gain.setdefault(tag, []).append(p)
    res = {'run': os.path.basename(run_dir), 'n_csv': len(csvs), 'gains': {}}
    for tag in sorted(by_gain):
        res['gains'][tag] = {'plant': plant_fit(by_gain[tag]),
                             'error_budget': error_budget(by_gain[tag])}
    return res


def _print_run(res):
    print(f"\n=== {res['run']}  ({res['n_csv']} stroke CSVs) ===")
    for tag, r in res['gains'].items():
        pl, eb = r['plant'], r['error_budget']
        print(f"\n  [{tag}]")
        if pl:
            print(f"    PLANT  (N={pl['n_samples']}, R2={pl['r2']:.3f})")
            print(f"      J_eff = {pl['J_eff_kgm2'] * 1e4:.2f}e-4 kg m^2"
                  f"  = {pl['J_eff_over_J_rotor']:.2f} x J_rotor")
            print(f"      tau_c = {pl['tau_c_Nm']:.4f} Nm ({pl['tau_c_A']:.2f} A)"
                  f"   b = {pl['b_Nm_per_rps'] * 1e3:.2f} mNm/(rev/s)")
        if eb:
            # NOT a partition: these are RMS norms of CORRELATED regressors, so they
            # do not sum (in any norm) to the total. Read them as per-term contributions.
            print(f"    ERROR BUDGET  (N={eb['n_samples']}, R2={eb['r2']:.3f})"
                  f"  -- component RMS CONTRIBUTIONS; regressors are correlated,")
            print(f"                    so they do NOT sum to the total.")
            print(f"      total errRMS      = {eb['total_rms_mm']:6.2f} mm")
            print(f"      delay  tau_d*v    = {eb['delay_term_rms_mm']:6.2f} mm"
                  f"   (tau_d = {eb['tau_d_ms']:.1f} ms)  <- gain-independent")
            print(f"      inertial  k_a*a   = {eb['inertial_term_rms_mm']:6.2f} mm")
            print(f"      friction  k_f*sgn = {eb['friction_term_rms_mm']:6.2f} mm")
            print(f"      const offset      = {eb['const_offset_mm']:6.2f} mm")
            print(f"      residual          = {eb['residual_rms_mm']:6.2f} mm")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('run_dirs', nargs='*', help='bench_sysid_* run directories')
    ap.add_argument('--all', action='store_true',
                    help='analyse every temp/probes/bench_sysid_* run')
    args = ap.parse_args(argv)

    runs = list(args.run_dirs)
    if args.all or not runs:
        runs = sorted(glob.glob(os.path.join(_REPO_ROOT, 'temp', 'probes',
                                             'bench_sysid_*')))
    results = [r for r in (analyse_run(d) for d in runs) if r]
    if not results:
        print('No stroke CSVs found. Point me at a bench_sysid_* run directory.',
              file=sys.stderr)
        return 1

    for r in results:
        _print_run(r)

    # Pooled J_eff across every gain point of every run -> the cascade table.
    js = [g['plant']['J_eff_kgm2'] for r in results for g in r['gains'].values()
          if g['plant']]
    if not js:
        print('No usable plant fits (no CSV had >=50 clearly-moving samples). '
              'Cannot build the cascade table.', file=sys.stderr)
        return 1
    j_mean = float(np.mean(js))
    print(f"\n=== CASCADE BANDWIDTHS  (pooled J_eff = {j_mean * 1e4:.2f}e-4 kg m^2, "
          f"{j_mean / J_ROTOR_KGM2:.2f} x J_rotor, from {len(js)} fits) ===")
    bw = bandwidths(j_mean)
    for b in bw:
        print(f"  pos {b['pos_gain']:3d} / vel {b['vel_gain']:.2f}:"
              f"  vel-loop {b['vel_loop_hz']:5.1f} Hz"
              f"   pos-loop {b['pos_loop_hz']:5.1f} Hz"
              f"   ratio wv/wp = {b['ratio_wv_over_wp']:.2f}")
    # The per-fit scatter is the honest uncertainty on every row above -- cite THIS,
    # not a hand-assembled range, whenever the cascade table is quoted in a document.
    j_lo, j_hi = float(np.min(js)), float(np.max(js))
    pg0, vg0 = GAIN_POINTS[0]
    # Same corrected formula as bandwidths(): the real torque per unit velocity error is
    # vel_gain * (KT_TRUE / KT_ODRIVE_CONFIG), NOT vel_gain.
    tau_per_e_v = vg0 * (KT_NM_PER_A / KT_ODRIVE_CONFIG)
    wv_lo = tau_per_e_v / (2.0 * np.pi * j_hi)           # rad/s; big J -> low omega_v
    wv_hi = tau_per_e_v / (2.0 * np.pi * j_lo)
    print(f"  scatter: J_eff {j_lo * 1e4:.2f}-{j_hi * 1e4:.2f}e-4 "
          f"({j_lo / J_ROTOR_KGM2:.2f}-{j_hi / J_ROTOR_KGM2:.2f} x J_rotor)"
          f"  =>  at pos {pg0}/vel {vg0}: vel-loop "
          f"{wv_lo / (2 * np.pi):.1f}-{wv_hi / (2 * np.pi):.1f} Hz,"
          f" ratio {wv_lo / pg0:.2f}-{wv_hi / pg0:.2f}")

    out_dir = os.path.join(_REPO_ROOT, 'temp', 'probes')
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    out = os.path.join(out_dir, f'plant_id_{stamp}.json')
    with open(out, 'w') as f:
        json.dump({'kt_nm_per_a': KT_NM_PER_A, 'j_rotor_kgm2': J_ROTOR_KGM2,
                   'mm_per_rev': MM_PER_REV, 'v_moving_rps': V_MOVING_RPS,
                   'runs': results, 'pooled_j_eff_kgm2': j_mean,
                   'cascade': bw}, f, indent=1)
    print(f"\nWrote {os.path.relpath(out, _REPO_ROOT)}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
