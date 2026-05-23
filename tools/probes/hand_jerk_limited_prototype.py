#!/usr/bin/env python3
"""hand_jerk_limited_prototype.py — Phase 2 jerk-limited profile reference.

What this module is
-------------------
The **offline Python reference implementation** of the jerk-limited
throw/catch profile family designed in Phase 2 of
``plans/active/hand-trajectory-generator-overhaul.md``. Phase 3 ports this
algorithm into both the Teensy firmware (``Trajectory.h``) and the sim port
(``sim/hand/trajectory.py``) **in lockstep**; this module is the canonical
reference both ports must agree with.

It is *not yet* wired into ``sim/hand/`` — that wiring is Phase 3. Phase 2
keeps it under ``tools/probes/`` so the Phase 2 prototype, its tests, and
its validation plots can land without touching the production hand stack.

Profile family
--------------
Three segments, joined with C2 (position + velocity + acceleration
continuity) at every boundary:

  1. **Accel quintic** — rest at ``stroke_lo`` → cruise velocity ``v`` over
     ``T_a``. BCs: ``(pos, vel, accel) = (stroke_lo, 0, 0) → (·, v, 0)``.
     Closed form: ``q(τ) = 2·τ³ − τ⁴`` (derived in the Phase 2 logbook).
  2. **Cruise** — linear segment at constant velocity ``v`` over ``T_c``.
     Trivially C∞ within itself; C2 with the quintics because the quintic
     BCs match (vel = v, accel = 0).
  3. **Decel quintic** — cruise velocity ``v`` → rest at ``stroke_hi`` over
     ``T_d``. Time-reverse of the accel quintic. Closed form:
     ``r(τ) = 2·τ − 2·τ³ + τ⁴``.

The accel and decel quintics are SYMMETRIC (same ramp time, same peak
accel) — see the Phase 2 logbook Discussion section for the family
selection (quintic over 7-segment S-curve) and the symmetric-vs-asymmetric
tradeoff. Per-segment evaluation is closed form and cheap.

Parameterisation
----------------
Inputs (per the plan §4 Phase 2):

* ``cruise_vel`` (m/s, signed) — cruise velocity. Positive for throw
  (hand rising), negative for catch (hand descending).
* ``duration`` (s) — total profile time ``T = T_a + T_c + T_d``. Must be
  in ``[T_min, T_max]``; see :func:`feasibility`.
* ``stroke_lo``, ``stroke_hi`` (m) — boundary positions; profile spans the
  signed interval ``stroke_hi − stroke_lo``. Sign must match
  ``cruise_vel``.

Derived (per the symmetric-ramp identity ``d_a = d_d = v·T_a / 2``):

* ``T_a = T − stroke / cruise_vel``
* ``T_c = 2·stroke / cruise_vel − T``  (equivalently ``T − 2·T_a``)
* ``d_a = d_d = cruise_vel · T_a / 2``  (signed)
* ``d_c = cruise_vel · T_c``  (signed)

Peak quantities (analytic, per-segment maxima):

* ``|a|_max = 1.5 · |cruise_vel| / T_a``  (at ``τ = 1/2`` of each quintic)
* ``|j|_max = 6 · |cruise_vel| / T_a²``  (at ``τ = 0, 1`` of each quintic)
* ``|j_step|_at_cruise_joins = 6·|cruise_vel| / T_a²``  (jerk drops from
  ``±|j|_max`` at the end of a quintic ramp to 0 in the cruise — a single
  step-discontinuity per cruise join, magnitude equal to the in-ramp peak;
  the only jerk discontinuities in the whole profile)

Feasibility
-----------
A profile is feasible iff ``T_min(v, stroke, A_peak) ≤ T_max(v, stroke)``,
which requires ``v² ≤ stroke · A_peak / 1.5``. The maximum cruise velocity
at full effective stroke (0.315 m) and the configured peak accel
(``max_event_hand_accel_rps2 = 6000 rev/s²`` ≈ 189.7 m/s²) is
``v_max ≈ 6.31 m/s`` (see the Phase 2 logbook §Discussion for the math and
the tradeoff against the legacy generator's 7.0 m/s).

Outputs (run as a script)
-------------------------
``python tools/probes/hand_jerk_limited_prototype.py`` writes:
* ``temp/probes/hand_jerk_limited_prototype.png`` — pos/vel/accel/jerk
  time-series for three representative throws + a catch at v=4 m/s.
* ``temp/probes/hand_jerk_limited_summary.json`` — analytic vs measured
  peaks, boundary residuals, scaling vs T slack.

Plan:    plans/active/hand-trajectory-generator-overhaul.md  (Phase 2)
Logbook: logbook/2026-05-23-hand-generator-phase2-jerk-limited-design.md
Tests:   tests/sim/test_hand_jerk_limited_prototype.py
"""

from __future__ import annotations

import json
import math
import os
import sys
from dataclasses import dataclass

# --- self-contained path setup (probe convention) --------------------------
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(_THIS_DIR))
for _p in (_REPO_ROOT, os.path.join(_REPO_ROOT, "config", "generated")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# Read the binding bounds from the generated config (single source of truth).
import hardware_config as _hw  # noqa: E402

MAX_EVENT_HAND_ACCEL_RPS2 = _hw.TEENSY_TRAJ_MAX_EVENT_HAND_ACCEL_RPS2  # 6000
HAND_STROKE_M             = _hw.TEENSY_TRAJ_HAND_STROKE_M              # 0.355
STROKE_MARGIN_M           = _hw.TEENSY_TRAJ_STROKE_MARGIN_M            # 0.02
CATCH_VEL_RATIO           = _hw.TEENSY_TRAJ_CATCH_VEL_RATIO            # 0.6
_LINEAR_GAIN              = _hw.TEENSY_LINEAR_GAIN                     # rev/m (= 31.617…)

# Derived constants
_TOTAL_STROKE_M = HAND_STROKE_M - 2.0 * STROKE_MARGIN_M                   # 0.315 m
MAX_EVENT_HAND_ACCEL_MPS2 = MAX_EVENT_HAND_ACCEL_RPS2 / _LINEAR_GAIN      # ~189.77 m/s^2

OUTPUT_DIR = os.path.join(_REPO_ROOT, "temp", "probes")


# ---------------------------------------------------------------------------
# Feasibility envelope
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Feasibility:
    """Feasibility envelope for a (stroke, cruise_vel, accel_peak) triple.

    ``T_min`` is the shortest physically-realisable profile duration;
    ``T_max`` is the longest before the cruise segment vanishes. ``feasible``
    is False when ``v² > stroke · A_peak / 1.5`` (peak-accel-bounded
    quintic ramps don't fit in the stroke).
    """
    feasible: bool
    T_min: float | None
    T_max: float | None
    T_a_min: float | None        # ramp time at T = T_min
    v_max: float                 # max |cruise_vel| at this stroke + A_peak


def feasibility(stroke: float, cruise_vel: float,
                accel_peak_mps2: float = MAX_EVENT_HAND_ACCEL_MPS2) -> Feasibility:
    """Compute the feasibility envelope. Sign convention: stroke and
    cruise_vel must have the same sign (both >0 throw, both <0 catch)."""
    s = abs(stroke)
    v = abs(cruise_vel)
    v_max = math.sqrt(s * accel_peak_mps2 / 1.5)
    if v <= 0.0 or s <= 0.0 or stroke * cruise_vel < 0.0:
        return Feasibility(False, None, None, None, v_max)
    T_a_min = 1.5 * v / accel_peak_mps2
    T_min = s / v + T_a_min
    T_max = 2.0 * s / v
    if T_min > T_max + 1e-12:
        return Feasibility(False, None, None, None, v_max)
    return Feasibility(True, T_min, T_max, T_a_min, v_max)


# ---------------------------------------------------------------------------
# JerkLimitedProfile — the prototype
# ---------------------------------------------------------------------------
class JerkLimitedProfile:
    """Symmetric 3-segment quintic-linear-quintic, C2 at all joins.

    Parameters
    ----------
    stroke_lo, stroke_hi : float
        Boundary positions (m). ``end_pos − start_pos`` has the same sign as
        ``cruise_vel``: positive for throw (rising), negative for catch.
    cruise_vel : float
        Cruise velocity (m/s, signed).
    duration : float
        Total profile duration T (s). Must lie in
        ``[feasibility(stroke, cruise_vel).T_min,
        feasibility(stroke, cruise_vel).T_max]``.
    accel_peak_mps2 : float
        Peak-acceleration bound used for feasibility check. The profile's
        analytic peak |a| is ``1.5 · |cruise_vel| / T_a`` and is ≤ this
        bound whenever ``duration ≥ T_min``.
    """

    def __init__(self, stroke_lo: float, stroke_hi: float,
                 cruise_vel: float, duration: float,
                 accel_peak_mps2: float = MAX_EVENT_HAND_ACCEL_MPS2):
        stroke = stroke_hi - stroke_lo
        feas = feasibility(stroke, cruise_vel, accel_peak_mps2)
        if not feas.feasible:
            raise ValueError(
                f"Infeasible: stroke={stroke:.4f} m, cruise_vel={cruise_vel:.3f} m/s, "
                f"A_peak={accel_peak_mps2:.1f} m/s²; v_max at this stroke = {feas.v_max:.3f} m/s"
            )
        if not (feas.T_min - 1e-12 <= duration <= feas.T_max + 1e-12):
            raise ValueError(
                f"Duration {duration:.4f} s out of [T_min={feas.T_min:.4f}, "
                f"T_max={feas.T_max:.4f}] for v={cruise_vel:.3f} m/s, stroke={stroke:.4f} m"
            )

        # Solve segment durations (symmetric: T_a = T_d).
        s_over_v = stroke / cruise_vel
        T_a = duration - s_over_v          # solved from the linear system
        T_c = 2.0 * s_over_v - duration    # T - 2·T_a
        # Numerical guards (T_c may be tiny-negative at T = T_max boundary)
        T_a = max(T_a, 0.0)
        T_c = max(T_c, 0.0)
        T_d = T_a

        d_a = cruise_vel * T_a / 2.0       # signed
        d_c = cruise_vel * T_c             # signed
        d_d = cruise_vel * T_d / 2.0       # signed

        self._stroke_lo = stroke_lo
        self._stroke_hi = stroke_hi
        self._cruise_vel = cruise_vel
        self._duration = duration
        self._accel_peak_mps2 = accel_peak_mps2
        self._T_a, self._T_c, self._T_d = T_a, T_c, T_d
        self._d_a, self._d_c, self._d_d = d_a, d_c, d_d
        self._t_join_1 = T_a                # end of accel quintic
        self._t_join_2 = T_a + T_c          # end of cruise

    # ---- properties --------------------------------------------------------
    @property
    def stroke_lo(self) -> float: return self._stroke_lo
    @property
    def stroke_hi(self) -> float: return self._stroke_hi
    @property
    def cruise_vel(self) -> float: return self._cruise_vel
    @property
    def duration(self) -> float: return self._duration
    @property
    def T_a(self) -> float: return self._T_a
    @property
    def T_c(self) -> float: return self._T_c
    @property
    def T_d(self) -> float: return self._T_d

    @property
    def peak_accel_mps2(self) -> float:
        """Analytic peak |acceleration| — exact, at τ = 1/2 of each quintic."""
        if self._T_a == 0.0:
            return 0.0
        return 1.5 * abs(self._cruise_vel) / self._T_a

    @property
    def peak_jerk_mps3(self) -> float:
        """Analytic peak |jerk| within the quintic ramps — at τ = 0, 1."""
        if self._T_a == 0.0:
            return 0.0
        return 6.0 * abs(self._cruise_vel) / (self._T_a ** 2)

    @property
    def cruise_join_jerk_step_mps3(self) -> float:
        """Signed magnitude of the jerk step at each cruise join.

        At the end of the accel quintic, jerk = (d_a/T_a³)·q'''(1) =
        -12·d_a/T_a³ (signed); in the cruise it is 0. The step magnitude
        equals 12·|d_a|/T_a³ = 6·|v|/T_a² (same as ``peak_jerk_mps3`` — the
        peak jerk *is* this step's leading edge).
        """
        if self._T_a == 0.0:
            return 0.0
        return 12.0 * abs(self._d_a) / (self._T_a ** 3)

    # ---- sampling ----------------------------------------------------------
    def sample(self, t: float) -> tuple[float, float, float, float]:
        """Return (pos_m, vel_mps, accel_mps2, jerk_mps3) at time *t* (s).

        Clamped to rest at the start and end positions outside [0, T].
        """
        if t <= 0.0:
            return (self._stroke_lo, 0.0, 0.0, 0.0)
        if t >= self._duration:
            return (self._stroke_hi, 0.0, 0.0, 0.0)

        if t <= self._t_join_1:
            # Accel quintic: q(τ) = 2τ³ − τ⁴
            T = self._T_a
            tau = t / T
            q   = 2.0 * tau ** 3 - tau ** 4
            qp  = 6.0 * tau ** 2 - 4.0 * tau ** 3
            qpp = 12.0 * tau - 12.0 * tau ** 2
            qppp = 12.0 - 24.0 * tau
            pos = self._stroke_lo + self._d_a * q
            vel = (self._d_a / T)        * qp
            acc = (self._d_a / T ** 2)   * qpp
            jrk = (self._d_a / T ** 3)   * qppp
            return (pos, vel, acc, jrk)

        if t <= self._t_join_2:
            # Cruise: constant velocity
            tau = t - self._t_join_1
            pos = (self._stroke_lo + self._d_a) + self._cruise_vel * tau
            return (pos, self._cruise_vel, 0.0, 0.0)

        # Decel quintic: r(τ) = 2τ − 2τ³ + τ⁴
        T = self._T_d
        tau = (t - self._t_join_2) / T
        r   = 2.0 * tau - 2.0 * tau ** 3 + tau ** 4
        rp  = 2.0 - 6.0 * tau ** 2 + 4.0 * tau ** 3
        rpp = -12.0 * tau + 12.0 * tau ** 2
        rppp = -12.0 + 24.0 * tau
        base = self._stroke_lo + self._d_a + self._d_c
        pos = base + self._d_d * r
        vel = (self._d_d / T)        * rp
        acc = (self._d_d / T ** 2)   * rpp
        jrk = (self._d_d / T ** 3)   * rppp
        return (pos, vel, acc, jrk)


# ---------------------------------------------------------------------------
# Convenience factories
# ---------------------------------------------------------------------------
def solve_min_time(stroke: float, cruise_vel: float,
                   accel_peak_mps2: float = MAX_EVENT_HAND_ACCEL_MPS2) -> float:
    """T_min = stroke/cruise_vel + 1.5·|cruise_vel|/A_peak (always feasible
    when the cruise vel itself is feasible)."""
    return feasibility(stroke, cruise_vel, accel_peak_mps2).T_min


def make_throw(release_vel: float, *,
               stroke_lo: float | None = None,
               stroke_hi: float | None = None,
               duration: float | None = None,
               accel_peak_mps2: float = MAX_EVENT_HAND_ACCEL_MPS2) -> JerkLimitedProfile:
    """Throw factory. Defaults: full effective stroke, T = T_min."""
    if stroke_lo is None:
        stroke_lo = STROKE_MARGIN_M
    if stroke_hi is None:
        stroke_hi = STROKE_MARGIN_M + _TOTAL_STROKE_M
    if duration is None:
        duration = solve_min_time(stroke_hi - stroke_lo, release_vel, accel_peak_mps2)
    return JerkLimitedProfile(stroke_lo, stroke_hi, release_vel, duration,
                              accel_peak_mps2)


def make_catch(ball_vel: float, *,
               stroke_lo: float | None = None,
               stroke_hi: float | None = None,
               duration: float | None = None,
               accel_peak_mps2: float = MAX_EVENT_HAND_ACCEL_MPS2) -> JerkLimitedProfile:
    """Catch factory. Hand descends at -catch_vel_ratio·ball_vel; default
    stroke is the full effective stroke (top → bottom), default T = T_min."""
    if stroke_lo is None:
        stroke_lo = STROKE_MARGIN_M
    if stroke_hi is None:
        stroke_hi = STROKE_MARGIN_M + _TOTAL_STROKE_M
    cruise = -CATCH_VEL_RATIO * abs(ball_vel)   # negative (downward)
    stroke = stroke_lo - stroke_hi              # negative (down)
    if duration is None:
        duration = solve_min_time(stroke, cruise, accel_peak_mps2)
    # Catch travels stroke_hi → stroke_lo: start = high, end = low.
    return JerkLimitedProfile(stroke_hi, stroke_lo, cruise, duration,
                              accel_peak_mps2)


# ---------------------------------------------------------------------------
# Validation helpers (used by both the script and the tests)
# ---------------------------------------------------------------------------
def boundary_residuals(prof: JerkLimitedProfile) -> dict:
    """Sample at t=0, t=T, and on either side of the two cruise joins; report
    the residuals of the C2 boundary conditions."""
    p0 = prof.sample(0.0)
    pT = prof.sample(prof.duration)
    eps = 1e-9
    j1a = prof.sample(prof.T_a - eps)        # end of accel quintic
    j1b = prof.sample(prof.T_a + eps)        # start of cruise
    j2a = prof.sample(prof.T_a + prof.T_c - eps)  # end of cruise
    j2b = prof.sample(prof.T_a + prof.T_c + eps)  # start of decel quintic
    return {
        "start_pos_residual_m": p0[0] - prof.stroke_lo,
        "start_vel_residual_mps": p0[1],
        "start_accel_residual_mps2": p0[2],
        "end_pos_residual_m": pT[0] - prof.stroke_hi,
        "end_vel_residual_mps": pT[1],
        "end_accel_residual_mps2": pT[2],
        "accel_join1_pos_jump_m": j1b[0] - j1a[0],
        "accel_join1_vel_jump_mps": j1b[1] - j1a[1],
        "accel_join1_jump_mps2": j1b[2] - j1a[2],
        "accel_join2_pos_jump_m": j2b[0] - j2a[0],
        "accel_join2_vel_jump_mps": j2b[1] - j2a[1],
        "accel_join2_jump_mps2": j2b[2] - j2a[2],
    }


def sample_grid(prof: JerkLimitedProfile, dt: float = 1.0 / 500.0) -> dict:
    """Sample the profile on a fixed dt grid → arrays for plotting/asserts."""
    n = int(round(prof.duration / dt)) + 1
    ts = [i * dt for i in range(n)]
    pos, vel, acc, jrk = [], [], [], []
    for t in ts:
        p, v, a, j = prof.sample(t)
        pos.append(p); vel.append(v); acc.append(a); jrk.append(j)
    return {"t": ts, "pos": pos, "vel": vel, "acc": acc, "jrk": jrk}


# ---------------------------------------------------------------------------
# Script entry point — characterise and plot
# ---------------------------------------------------------------------------
def _summary_for(label: str, prof: JerkLimitedProfile) -> dict:
    bcs = boundary_residuals(prof)
    grid = sample_grid(prof)
    max_abs = lambda xs: max(abs(x) for x in xs)
    return {
        "label": label,
        "cruise_vel_mps": prof.cruise_vel,
        "duration_s": prof.duration,
        "T_a_s": prof.T_a,
        "T_c_s": prof.T_c,
        "T_d_s": prof.T_d,
        "stroke_lo_m": prof.stroke_lo,
        "stroke_hi_m": prof.stroke_hi,
        "analytic_peak_accel_mps2": prof.peak_accel_mps2,
        "analytic_peak_jerk_mps3": prof.peak_jerk_mps3,
        "cruise_join_jerk_step_mps3": prof.cruise_join_jerk_step_mps3,
        "sampled_peak_vel_mps": max_abs(grid["vel"]),
        "sampled_peak_accel_mps2": max_abs(grid["acc"]),
        "sampled_peak_jerk_mps3": max_abs(grid["jrk"]),
        "max_C2_join_accel_jump_mps2": max(
            abs(bcs["accel_join1_jump_mps2"]),
            abs(bcs["accel_join2_jump_mps2"]),
        ),
        "boundary_residuals": bcs,
    }


def main() -> int:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Representative profiles — three throws (T = T_min and one with slack)
    # plus a catch at v=4 m/s.
    feas = feasibility(_TOTAL_STROKE_M, 6.31)  # v_max at the configured bound
    profiles = [
        ("throw v=3.0 m/s, T=T_min",   make_throw(3.0)),
        ("throw v=5.0 m/s, T=T_min",   make_throw(5.0)),
        # Slack demo at v=3 (where the feasibility envelope is wide): T set
        # to the midpoint of [T_min, T_max] to lower peak accel and jerk.
        ("throw v=3.0 m/s, T=mid-envelope", make_throw(
            3.0, duration=0.5 * (feasibility(_TOTAL_STROKE_M, 3.0).T_min
                                 + feasibility(_TOTAL_STROKE_M, 3.0).T_max))),
        ("catch v_ball=4.0 m/s, T=T_min", make_catch(4.0)),
    ]
    summaries = [_summary_for(label, p) for label, p in profiles]

    # Plot
    fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=False)
    for label, prof in profiles:
        g = sample_grid(prof)
        axes[0].plot(g["t"], [x * 1000.0 for x in g["pos"]], label=label)
        axes[1].plot(g["t"], g["vel"], label=label)
        axes[2].plot(g["t"], g["acc"], label=label)
        axes[3].plot(g["t"], g["jrk"], label=label)
    axes[0].set_ylabel("position [mm]")
    axes[1].set_ylabel("velocity [m/s]")
    axes[2].set_ylabel("acceleration [m/s²]")
    axes[3].set_ylabel("jerk [m/s³]")
    axes[3].set_xlabel("time [s]")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    axes[2].axhline(MAX_EVENT_HAND_ACCEL_MPS2, color="k", ls=":", lw=0.6,
                    label=f"A_peak limit ({MAX_EVENT_HAND_ACCEL_MPS2:.1f} m/s²)")
    axes[2].axhline(-MAX_EVENT_HAND_ACCEL_MPS2, color="k", ls=":", lw=0.6)
    axes[0].set_title("Phase 2 jerk-limited profile prototype — bounded jerk, C2 at joins")
    fig.tight_layout()
    fig_path = os.path.join(OUTPUT_DIR, "hand_jerk_limited_prototype.png")
    fig.savefig(fig_path, dpi=110, metadata={"Software": "hand_jerk_limited_prototype"})
    plt.close(fig)

    out = {
        "module": "hand_jerk_limited_prototype",
        "max_event_hand_accel_rps2": MAX_EVENT_HAND_ACCEL_RPS2,
        "max_event_hand_accel_mps2": MAX_EVENT_HAND_ACCEL_MPS2,
        "effective_stroke_m": _TOTAL_STROKE_M,
        "v_max_full_stroke_mps": feas.v_max,
        "profiles": summaries,
    }
    json_path = os.path.join(OUTPUT_DIR, "hand_jerk_limited_summary.json")
    with open(json_path, "w") as fh:
        json.dump(out, fh, indent=2, sort_keys=True)
        fh.write("\n")

    print("=" * 70)
    print("hand_jerk_limited_prototype — Phase 2 reference implementation")
    print("=" * 70)
    print(f"  Peak-accel bound: {MAX_EVENT_HAND_ACCEL_RPS2:.0f} rev/s² "
          f"= {MAX_EVENT_HAND_ACCEL_MPS2:.2f} m/s²")
    print(f"  v_max at full stroke (0.315 m): {feas.v_max:.3f} m/s")
    for s in summaries:
        print(f"  {s['label']}:  T={s['duration_s']:.4f}s  "
              f"peak|a|={s['analytic_peak_accel_mps2']:.1f} m/s²  "
              f"peak|j|={s['analytic_peak_jerk_mps3']:.0f} m/s³  "
              f"C2-join-jump={s['max_C2_join_accel_jump_mps2']:.2e} m/s²")
    print(f"  Outputs → {OUTPUT_DIR}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
