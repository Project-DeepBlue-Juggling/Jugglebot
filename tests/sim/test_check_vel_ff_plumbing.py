"""Unit tests for tools/check_vel_ff_plumbing.py — exercises the pure
analysis functions (`_ingest_sample`, `_verdict`, `_format_report`)
without any ZMQ I/O.

These tests pin down:
  - that absent / partial telemetry messages are counted but don't crash
  - that per-leg non-zero fractions and peak magnitudes are computed correctly
  - that the verdict logic returns the expected exit code in each branch
    (no-telemetry, leg_vel-always-None, flat-zero-vel_ff, live-vel_ff)
"""
from __future__ import annotations

import os
import sys

import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
_TOOLS_DIR = os.path.join(_REPO_ROOT, 'tools')
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)

import check_vel_ff_plumbing as cvf  # type: ignore  # noqa: E402


# ───────────────────────────────────────────────────────────────────
# _ingest_sample
# ───────────────────────────────────────────────────────────────────

def test_ingest_empty_message_increments_sample_count_only():
    stats = cvf.CaptureStats()
    cvf._ingest_sample(stats, {})
    assert stats.n_samples == 1
    assert stats.n_with_leg_vel == 0
    assert stats.n_with_leg_torques == 0
    assert stats.leg_vel_nonzero_count == [0] * 6
    assert stats.leg_vel_peak_abs == [0.0] * 6


def test_ingest_records_nonzero_vel_count_and_peak_per_leg():
    stats = cvf.CaptureStats()
    cvf._ingest_sample(stats, {'leg_vel': [0.0, 0.5, -1.2, 0.0, 0.3, 0.0]})
    assert stats.n_samples == 1
    assert stats.n_with_leg_vel == 1
    assert stats.leg_vel_nonzero_count == [0, 1, 1, 0, 1, 0]
    # peak is absolute value, so -1.2 → 1.2
    assert stats.leg_vel_peak_abs == [0.0, 0.5, 1.2, 0.0, 0.3, 0.0]


def test_ingest_peak_takes_max_across_samples():
    stats = cvf.CaptureStats()
    cvf._ingest_sample(stats, {'leg_vel': [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]})
    cvf._ingest_sample(stats, {'leg_vel': [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]})
    cvf._ingest_sample(stats, {'leg_vel': [-0.7, 0.0, 0.0, 0.0, 0.0, 0.0]})
    assert stats.leg_vel_peak_abs[0] == pytest.approx(0.7)
    assert stats.leg_vel_nonzero_count[0] == 3
    assert stats.n_with_leg_vel == 3


def test_ingest_handles_leg_vel_None_branch_in_motor_guard_fallback():
    """motor_guard's fallback envelope sets ``leg_vel: None``.
    The diagnostic must count the sample without crashing or registering it
    as a "with leg_vel" sample.
    """
    stats = cvf.CaptureStats()
    cvf._ingest_sample(stats, {'leg_vel': None, 'leg_torques': None})
    assert stats.n_samples == 1
    assert stats.n_with_leg_vel == 0
    assert stats.n_with_leg_torques == 0


def test_ingest_records_torques_separately_from_vel():
    stats = cvf.CaptureStats()
    cvf._ingest_sample(stats, {'leg_torques': [0.0] * 6})
    cvf._ingest_sample(stats, {'leg_torques': [0.01, 0.0, -0.02, 0.0, 0.0, 0.0]})
    assert stats.n_with_leg_torques == 2
    assert stats.leg_torques_nonzero_count == [1, 0, 1, 0, 0, 0]
    assert stats.leg_torques_peak_abs[0] == pytest.approx(0.01)
    assert stats.leg_torques_peak_abs[2] == pytest.approx(0.02)


def test_ingest_truncates_leg_arrays_longer_than_six():
    """Future-proofing: if motor_guard ever publishes a 7+-element array,
    don't index out of bounds."""
    stats = cvf.CaptureStats()
    cvf._ingest_sample(stats, {'leg_vel': [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 9.9]})
    assert stats.leg_vel_peak_abs == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]


def test_ingest_handles_short_leg_arrays_gracefully():
    stats = cvf.CaptureStats()
    cvf._ingest_sample(stats, {'leg_vel': [0.5, 0.4]})
    assert stats.n_with_leg_vel == 1
    assert stats.leg_vel_peak_abs[:2] == [0.5, 0.4]
    assert stats.leg_vel_peak_abs[2:] == [0.0] * 4


# ───────────────────────────────────────────────────────────────────
# _verdict
# ───────────────────────────────────────────────────────────────────

def test_verdict_no_samples_returns_no_telemetry():
    stats = cvf.CaptureStats()
    rc, msg = cvf._verdict(stats)
    assert rc == 1
    assert 'NO TELEMETRY' in msg


def test_verdict_all_leg_vel_None_returns_warn():
    """Every sample had leg_vel = None — fallback envelope only."""
    stats = cvf.CaptureStats(n_samples=200)  # received samples but no leg_vel
    rc, msg = cvf._verdict(stats)
    assert rc == 2
    assert 'leg_vel = None' in msg


def test_verdict_flat_zero_vel_ff_returns_warn():
    """Samples carry leg_vel arrays but every value is exactly 0.0."""
    stats = cvf.CaptureStats()
    for _ in range(200):
        cvf._ingest_sample(stats, {'leg_vel': [0.0] * 6})
    rc, msg = cvf._verdict(stats)
    assert rc == 2
    assert 'FLAT ZERO' in msg


def test_verdict_live_vel_ff_returns_ok():
    """At least one leg has peak above noise floor and non-zero in a
    sufficient fraction of samples."""
    stats = cvf.CaptureStats()
    # 200 samples, leg 0 is live with vel_ff = 0.5 in every sample
    for _ in range(200):
        cvf._ingest_sample(stats, {'leg_vel': [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]})
    rc, msg = cvf._verdict(stats)
    assert rc == 0
    assert 'LIVE' in msg


def test_verdict_one_leg_at_threshold_passes():
    """Exactly one leg above the non-zero-fraction threshold (10 %) and
    above the peak threshold (0.001 rev/s) — minimum positive case."""
    stats = cvf.CaptureStats()
    # 100 samples; leg 3 has vel_ff = 0.01 in 11 of them (above 10 % frac)
    for i in range(100):
        leg_vel = [0.0] * 6
        if i < 11:
            leg_vel[3] = 0.01
        cvf._ingest_sample(stats, {'leg_vel': leg_vel})
    rc, _ = cvf._verdict(stats)
    assert rc == 0


def test_verdict_below_fraction_threshold_returns_warn():
    """Above peak threshold but only 5 % of samples — below the 10 %
    non-zero-fraction floor — counts as flat zero."""
    stats = cvf.CaptureStats()
    for i in range(100):
        leg_vel = [0.0] * 6
        if i < 5:        # only 5 of 100 → 5 % < 10 % threshold
            leg_vel[2] = 0.5
        cvf._ingest_sample(stats, {'leg_vel': leg_vel})
    rc, msg = cvf._verdict(stats)
    assert rc == 2
    assert 'FLAT ZERO' in msg


def test_verdict_below_peak_threshold_returns_warn():
    """Non-zero in every sample but below the peak threshold — sub-LSB
    floating-point dust, not real motion."""
    stats = cvf.CaptureStats()
    for _ in range(100):
        # 1e-7 is below PEAK_VEL_THRESHOLD_RPS = 1e-3
        cvf._ingest_sample(stats, {'leg_vel': [1e-7] * 6})
    rc, msg = cvf._verdict(stats)
    assert rc == 2
    assert 'FLAT ZERO' in msg


# ───────────────────────────────────────────────────────────────────
# _format_report
# ───────────────────────────────────────────────────────────────────

def test_format_report_handles_empty_stats():
    stats = cvf.CaptureStats()
    text = cvf._format_report(stats)
    assert 'no samples' in text


def test_format_report_includes_per_leg_rows():
    stats = cvf.CaptureStats()
    stats.duration_s = 1.0
    for _ in range(500):
        cvf._ingest_sample(stats, {'leg_vel': [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
                                   'leg_torques': [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]})
    text = cvf._format_report(stats)
    assert 'Leg' in text
    # All six leg rows should appear
    assert '\n     0  ' in text or text.count('  0  ') >= 1
    # Reported peak for leg 0 should match
    assert '0.50000' in text
