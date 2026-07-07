"""Unit tests for ``diagnose.summarise_trajectory_moves`` (Phase-4 /diagnose ext).

The Phase-4 limit-ramp workflow raises one leg limit ~1.5×, runs a scripted move
battery, and reviews ``/diagnose`` for tracking error + jerk headroom before
persisting the bump to YAML. This module pins the summariser that turns the
recorded ``/trajectory/diagnostics`` stream into a per-move ``peaks + headroom``
table: it must segment moves (by ``move_seq`` — a completed move stays
``plan_kind == 'move'`` for its whole terminal hold, so the ramp battery's
back-to-back moves share one unbroken ``'move'`` run and only ``move_seq`` marks the
per-move boundary), read the accumulated realized peak off the LAST sample of each
window, and compute % used vs the session limits in force (realized AND the
gate-authoritative predicted).
"""

from __future__ import annotations

from analysis.diagnose import summarise_trajectory_moves


def _sample(t, kind, *, rv=0.0, ra=0.0, rj=0.0,
            lv=100.0, la=400.0, lj=8000.0, gain=0.0,
            pv=0.0, pa=0.0, pj=0.0, seq=None):
    """One /trajectory/diagnostics KeyValue dict (values are strings on the wire)."""
    d = {
        't_s': t,
        'plan_kind': kind,
        'peak_leg_vel_mmps': f'{pv:.1f}',
        'peak_leg_acc_mmps2': f'{pa:.1f}',
        'peak_leg_jerk_mmps3': f'{pj:.0f}',
        'realized_peak_leg_vel_mmps': f'{rv:.1f}',
        'realized_peak_leg_acc_mmps2': f'{ra:.1f}',
        'realized_peak_leg_jerk_mmps3': f'{rj:.0f}',
        'limit_leg_vel_mmps': f'{lv:.1f}',
        'limit_leg_acc_mmps2': f'{la:.1f}',
        'limit_leg_jerk_mmps3': f'{lj:.0f}',
        'lean_gain': f'{gain:.2f}',
    }
    if seq is not None:
        d['move_seq'] = str(seq)
    return d


def test_no_samples_reports_unavailable():
    out = summarise_trajectory_moves([])
    assert out['available'] is False


def test_segments_two_moves_separated_by_hold():
    stream = [
        _sample(0.0, 'hold', seq=0),
        # move 1 — realized peaks ramp up (running maxima)
        _sample(0.2, 'move', rv=40.0, ra=200.0, rj=4000.0, seq=1),
        _sample(0.4, 'move', rv=80.0, ra=360.0, rj=6000.0, pj=6100.0, seq=1),
        _sample(1.0, 'hold', seq=1),
        # move 2 — lean A/B arm at gain 0.3
        _sample(1.2, 'move', rv=30.0, ra=150.0, rj=3000.0, gain=0.3, seq=2),
        _sample(1.6, 'move', rv=55.0, ra=300.0, rj=5200.0, gain=0.3, seq=2),
        _sample(2.2, 'hold', seq=2),
    ]
    out = summarise_trajectory_moves(stream)
    assert out['available'] is True
    assert out['num_moves'] == 2

    m1, m2 = out['moves']
    # Move window carries the LAST (accumulated) realized peak.
    assert m1['realized']['vel_mmps'] == 80.0
    assert m1['realized']['jerk_mmps3'] == 6000.0
    assert m1['t_start_s'] == 0.2 and m1['t_end_s'] == 0.4
    assert m1['lean_gain'] == 0.0
    assert m1['move_seq'] == '1'
    # Realized headroom: 6000/8000 jerk ⇒ 75% used.
    assert m1['used_pct']['jerk'] == 75.0
    assert m1['used_pct']['vel'] == 80.0
    assert m1['used_pct']['acc'] == 90.0
    # Gate-authoritative (predicted) jerk headroom uses the fine peak (6100/8000 =
    # 76.25 → 76.2 by Python's round-half-to-even), distinct from realized's 75.0.
    assert m1['used_pct_predicted']['jerk'] == 76.2
    # Move 2 is the lean arm.
    assert m2['lean_gain'] == 0.3
    assert m2['realized']['vel_mmps'] == 55.0


def test_segments_consecutive_moves_without_hold_via_move_seq():
    """The REAL ramp-battery shape: back-to-back moves with NO hold between them.

    A completed move's plan stays plan_kind='move' through its terminal hold, so the
    stream is one unbroken 'move' run; only move_seq marks the boundary. Without the
    move_seq split these two would merge into one window and move 1's peaks (80) would
    be lost (the LAST sample carries move 2's reset-then-climbed 55).
    """
    stream = [
        _sample(0.0, 'move', rv=40.0, ra=200.0, rj=4000.0, seq=1),
        _sample(0.2, 'move', rv=80.0, ra=360.0, rj=6000.0, seq=1),
        # move 2 installs — plan_kind is STILL 'move', realized peaks reset then climb.
        _sample(0.4, 'move', rv=5.0,  ra=30.0,  rj=500.0,  seq=2),
        _sample(0.6, 'move', rv=55.0, ra=300.0, rj=5200.0, seq=2),
    ]
    out = summarise_trajectory_moves(stream)
    assert out['num_moves'] == 2
    assert out['moves'][0]['realized']['vel_mmps'] == 80.0    # move 1 not lost
    assert out['moves'][0]['move_seq'] == '1'
    assert out['moves'][1]['realized']['vel_mmps'] == 55.0
    assert out['moves'][1]['move_seq'] == '2'


def test_go_home_after_move_is_its_own_row_move_keeps_true_peaks():
    """Audit fix (2026-07-08): a go_home / graceful-stop installs a kind=='move' plan
    that RESETS the realized peaks under plan_kind=='move'. The node now bumps
    move_seq on that non-follower install, so the stop is its own summariser row and
    the move's TRUE realized peaks survive (probe #6: true 80 was reported as 12).
    """
    stream = [
        _sample(0.0, 'move', rv=40.0, ra=200.0, rj=4000.0, seq=1),
        _sample(0.2, 'move', rv=80.0, ra=360.0, rj=6000.0, seq=1),   # move's true peak
        # go_home installs — plan_kind STILL 'move', realized peaks reset to ~zero,
        # but move_seq bumped (the fix) so this is a NEW window, not a merge.
        _sample(0.4, 'move', rv=3.0,  ra=15.0,  rj=200.0,  seq=2),
        _sample(0.6, 'move', rv=12.0, ra=60.0,  rj=900.0,  seq=2),   # go_home's own peaks
    ]
    out = summarise_trajectory_moves(stream)
    assert out['num_moves'] == 2
    assert out['moves'][0]['move_seq'] == '1'
    assert out['moves'][0]['realized']['vel_mmps'] == 80.0    # move's true peak, NOT 12
    assert out['moves'][1]['move_seq'] == '2'
    assert out['moves'][1]['realized']['vel_mmps'] == 12.0    # go_home's own row


def test_consecutive_holdless_moves_without_move_seq_merge_gracefully():
    """Older bags lacking move_seq degrade to plan_kind-only segmentation (merge)."""
    stream = [
        _sample(0.0, 'move', rv=80.0, ra=360.0, rj=6000.0),   # no seq
        _sample(0.4, 'move', rv=55.0, ra=300.0, rj=5200.0),   # no seq
    ]
    out = summarise_trajectory_moves(stream)
    assert out['num_moves'] == 1                               # merged, as documented


def test_trailing_move_without_closing_hold_is_flushed():
    stream = [
        _sample(0.0, 'hold', seq=0),
        _sample(0.2, 'move', rv=50.0, ra=250.0, rj=5000.0, seq=1),
        _sample(0.5, 'move', rv=70.0, ra=350.0, rj=7000.0, seq=1),
    ]
    out = summarise_trajectory_moves(stream)
    assert out['num_moves'] == 1
    assert out['moves'][0]['realized']['vel_mmps'] == 70.0
