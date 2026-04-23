"""Unit tests for ``sim.analysis.diagnose.analyse_oscillation``.

Covers the chatter-ratio computation, its sub-LSB magnitude floor, and
its large-dt outlier mask.  Each of these pieces was added to close a
W6 false-positive: the W6 hardware session reported "Oscillation
detected on all 6 legs" despite an operator-verified clean ODrive
pos_setpoint trace — the cause was sub-micron IPOPT float-rounding
noise in ``cmd_ext`` being counted as real sign changes, compounded
by occasional 149 ms inter-tick wall-clock gaps from the W5 scheduled
``gc.collect(2)``.  The fix in commit 3082ff5 introduced:

* ``_CHATTER_MIN_DELTA_MM = 0.01`` — deltas below this are treated as
  "no change", excluded from both numerator and denominator.
* 5×-median-dt mask on the amplitude-growing regression so a GC-gap
  delta doesn't inflate the envelope slope.

The fix was verified on the W6 CSV at fix time but not guarded by a
unit test.  This file is that unit test.

See ``logbook/2026-04-23-hot-loop-zero-allocation-contract.md``
(Post-W7 audit → chatter follow-up) for the full narrative.
"""

from __future__ import annotations

import numpy as np
import pytest

# Chatter analysis lives in the sim.analysis package, which depends on
# viz.telemetry.StepRecord.  Import via the same path diagnose.py uses
# (``from viz.telemetry import StepRecord``) — conftest sets up sim/
# on sys.path for the test collection.
from viz.telemetry import StepRecord
from analysis.diagnose import analyse_oscillation


# ---------------------------------------------------------------------------
# Fixture helpers
# ---------------------------------------------------------------------------

def _make_records(
    n: int,
    dt: float = 0.025,
    cmd_leg0_fn=None,
    actual_pose_z_fn=None,
    t_start: float = 0.0,
    dt_overrides: dict[int, float] | None = None,
) -> list[StepRecord]:
    """Build a list of ``StepRecord`` with scalar fields set to match
    what ``analyse_oscillation`` reads: per-leg ``cmd_ext_{0..5}`` and
    per-axis ``actual_pose_{x,y,z,rx,ry,rz}``.

    ``cmd_leg0_fn(i) -> float`` populates ``cmd_ext_0``; all other leg
    commands stay at 0.0 (which collapses to 0 chatter after floor).

    ``actual_pose_z_fn(i) -> float`` populates ``actual_pose_z``; other
    DoFs stay at 0.0.

    ``dt_overrides[i]`` applies to the gap BEFORE sample ``i`` (i.e.
    ``records[i].time - records[i-1].time``).  Used to inject
    scheduled-GC-style 149 ms gaps at specific indices.

    All other StepRecord fields default to 0.0 — they're not read by
    ``analyse_oscillation`` so leaving them at default is fine.
    """
    records: list[StepRecord] = []
    t = t_start
    for i in range(n):
        if i > 0:
            step_dt = (dt_overrides.get(i, dt)
                       if dt_overrides is not None else dt)
            t += step_dt
        rec = StepRecord()
        rec.time = t
        if cmd_leg0_fn is not None:
            rec.cmd_ext_0 = float(cmd_leg0_fn(i))
        if actual_pose_z_fn is not None:
            rec.actual_pose_z = float(actual_pose_z_fn(i))
        records.append(rec)
    return records


# ---------------------------------------------------------------------------
# Fixture (a): sub-LSB noise is filtered → detected == False
# ---------------------------------------------------------------------------

class TestSubLsbFloor:
    """Deltas below ``_CHATTER_MIN_DELTA_MM`` (10 µm) are treated as
    no-change.  A clean hold with only IPOPT float-rounding noise
    must NOT trip the chatter flag."""

    def test_all_sub_lsb_deltas_yield_zero_chatter(self):
        # cmd drifts by ±1 µm per sample — classic IPOPT float noise,
        # all below the 10 µm floor.  Every delta has a sign, every
        # other sign flips — pre-fix this was chatter ≈ 0.5.
        rng = np.random.default_rng(seed=42)
        sub_lsb_noise = rng.uniform(-1e-3, 1e-3, size=200)  # ±1 µm
        cumsum = np.cumsum(sub_lsb_noise)
        records = _make_records(
            n=len(cumsum),
            cmd_leg0_fn=lambda i: float(cumsum[i]),
        )
        result = analyse_oscillation(records)
        assert result['detected'] is False, (
            f"Sub-LSB noise should not trip chatter.  "
            f"per_leg_chatter={result['per_leg_chatter']}"
        )
        # Leg 0 specifically should show 0.0 chatter — all deltas got
        # floored out, leaving fewer than 2 non-zero signs.
        assert result['per_leg_chatter'][0] == 0.0
        # All other legs are flat-zero; their diffs are all zero so
        # chatter is also 0.
        for leg in range(1, 6):
            assert result['per_leg_chatter'][leg] == 0.0

    def test_constant_cmd_yields_zero_chatter(self):
        # Degenerate edge case: cmd never changes at all.  diff is all
        # zeros; the magnitude floor and the ``len(nonzero) < 2`` early
        # return both handle it.  No division by zero, no crash.
        records = _make_records(
            n=100,
            cmd_leg0_fn=lambda i: 154.5,
        )
        result = analyse_oscillation(records)
        assert result['detected'] is False
        assert all(c == 0.0 for c in result['per_leg_chatter'])


# ---------------------------------------------------------------------------
# Fixture (b): real oscillation above floor → detected == True
# ---------------------------------------------------------------------------

class TestGenuineOscillation:
    """Alternating deltas above the 10 µm floor MUST flip the chatter
    flag — the floor defends against noise, not against real motion."""

    def test_alternating_plus_minus_1mm_detected(self):
        # ±1 mm alternating deltas — 100× the magnitude floor.  Every
        # sign change counts; chatter ratio approaches 1.0.
        records = _make_records(
            n=100,
            cmd_leg0_fn=lambda i: 1.0 if i % 2 == 0 else 0.0,
        )
        result = analyse_oscillation(records)
        assert result['detected'] is True, (
            f"Alternating ±1 mm must trip chatter.  "
            f"per_leg_chatter={result['per_leg_chatter']}"
        )
        # Leg 0 chatter should be close to 1.0 (every consecutive
        # non-zero sign flips).
        assert result['per_leg_chatter'][0] > 0.9

    def test_genuinely_growing_amplitude_detected(self):
        # Alternating deltas whose magnitude grows linearly from 50 µm
        # to 5 mm.  Both the chatter and amplitude_growing flags
        # should trip.  The dt-outlier mask (fixture c) specifically
        # protects this regression from being polluted by the
        # scheduled-GC gap, so here we also check the mask leaves
        # genuine growth intact.
        def cmd_fn(i):
            amp = 0.05 + 5e-2 * i   # 50 µm -> ~5 mm across 100 samples
            return amp if i % 2 == 0 else 0.0
        records = _make_records(n=100, cmd_leg0_fn=cmd_fn)
        result = analyse_oscillation(records)
        assert result['detected'] is True
        assert result['amplitude_growing'] is True


# ---------------------------------------------------------------------------
# Fixture (c): large-dt outliers (scheduled-GC gaps) don't corrupt the
# amplitude-growing regression
# ---------------------------------------------------------------------------

class TestScheduledGcGapMask:
    """The 5×-median-dt mask ensures the one-per-30 s scheduled-GC
    149 ms gap doesn't inject a fake envelope spike into the
    amplitude-growing slope fit."""

    def test_single_large_gap_does_not_fake_amplitude_growth(self):
        # Steady alternating 2 mm deltas (clearly chatter, but flat
        # envelope — slope ≈ 0).  Inject one GC-sized dt gap (149 ms
        # ≈ 6× the median 25 ms).
        def cmd_fn(i):
            return 2.0 if i % 2 == 0 else 0.0
        records = _make_records(
            n=200,
            cmd_leg0_fn=cmd_fn,
            dt_overrides={100: 0.149},   # one 149 ms gap mid-run
        )
        result = analyse_oscillation(records)
        # Chatter still detected (magnitudes above floor, alternating):
        assert result['detected'] is True
        # Envelope slope stays ≈ 0 — the dt mask kept the GC-gap
        # large-delta from inflating it.  Without the mask the GC-gap
        # delta becomes a lone envelope spike that, post-polyfit,
        # reads as positive slope.  With the mask, slope stays below
        # the 0.001 threshold and amplitude_growing stays False.
        assert result['amplitude_growing'] is False, (
            "dt-outlier mask failed to suppress GC-gap delta; "
            "amplitude_growing incorrectly flagged True."
        )

    def test_multiple_gc_gaps_do_not_crash(self):
        # Four GC-sized gaps across 200 samples — matches the observed
        # cadence of ~1 per 30 s on hardware.  Just verifies the
        # function handles a realistic gap pattern without error.
        records = _make_records(
            n=200,
            cmd_leg0_fn=lambda i: 0.0,
            dt_overrides={50: 0.149, 100: 0.149, 150: 0.149, 180: 0.149},
        )
        result = analyse_oscillation(records)
        # Flat cmd → no chatter regardless of gaps.
        assert result['detected'] is False


# ---------------------------------------------------------------------------
# Boundary: very short record lists should return sentinel results
# ---------------------------------------------------------------------------

class TestBoundary:
    def test_fewer_than_10_records_returns_sentinel(self):
        records = _make_records(n=5, cmd_leg0_fn=lambda i: float(i))
        result = analyse_oscillation(records)
        assert result == {
            'detected': False,
            'per_leg_chatter': [],
            'per_dof_chatter': [],
        }

    def test_constant_time_fallback_does_not_crash(self):
        # Synthetic fixture where every record has identical time.
        # median_dt is 0 → the "fall back to no mask" branch must run
        # without ZeroDivision.
        records = _make_records(n=50, cmd_leg0_fn=lambda i: 0.0)
        for r in records:
            r.time = 0.0
        result = analyse_oscillation(records)
        assert result['detected'] is False
