"""Unit tests for the pure core of the tilt-calibration acquisition tool.

Tool: ``tests/hardware/tilt_cal_grid.py`` (operator-run rclpy CLI).
Contract: ``ros_ws/docs/levelling_frame.md`` § **C-LEVEL-2**.
Plan: ``plans/archived/tilt-calibration-grid.md`` § Phase 3.

The tool splits into a **pure core** — grid-spec generation, the residual
reduction, map-document assembly, CSV rows — and a thin rclpy runtime. Only the
core is tested here, and it is importable with no ROS at all: the runtime
imports ``rclpy`` and the interface packages *inside* :func:`tilt_cal_grid.run`
precisely so this file can exist.

This lives in ``tests/motion/`` rather than ``tests/ros/`` or ``tests/sim/``
because that is where every existing importer of a ``tests/hardware/`` script
already lives (``test_bench_sysid_logic.py``, ``test_bench_sysid_bridge.py``,
``test_kt_lib.py``), using the same ``_HW_DIR`` ``sys.path`` idiom.

Two tests carry more weight than the rest:

* :func:`test_residual_matches_the_real_levelling_handler` drives the **actual**
  ``state_machine.LevellingHandler`` phase and compares its output to the tool's
  reduction on the same reading. The residual is *defined* as that formula, so a
  restatement of it in an assertion would drift in lockstep with a bug. This
  fails if either side changes its sign, its constant, or its axis order.
* :func:`test_map_document_round_trips_through_the_production_loader` pushes the
  writer's own output through ``tilt_map.parse_tilt_map`` — the same function
  ``trajectory_node`` loads with. "The tool writes a valid map" is otherwise a
  claim about a schema doc rather than about the machine.

The grid fixtures are deliberately **asymmetric** (3 x-nodes vs 4 y-nodes,
unequal spacing, no two residuals equal) for the reason ``test_tilt_map.py``
gives: a transposed ``[iy][ix]`` index is invisible on a square symmetric grid.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest
import yaml

_HW_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'hardware')
if _HW_DIR not in sys.path:
    sys.path.insert(0, _HW_DIR)

import tilt_cal_grid as tcg  # noqa: E402

import jugglebot.hardware_config as hw  # noqa: E402
from jugglebot.motion import tilt_map  # noqa: E402


OFFSET = tcg.mounting_offset_rad()


# ── grid spec ────────────────────────────────────────────────────────────


def test_build_axis_box_form_spans_the_box_inclusively():
    assert tcg.build_axis(150.0, 5, 'x') == [-150.0, -75.0, 0.0, 75.0, 150.0]
    assert tcg.build_axis(100.0, 3, 'y') == [-100.0, 0.0, 100.0]


def test_build_axis_rejects_degenerate_specs():
    with pytest.raises(tcg.TiltCalError, match='at least 2'):
        tcg.build_axis(150.0, 1, 'x')
    with pytest.raises(tcg.TiltCalError, match='positive half-width'):
        tcg.build_axis(0.0, 5, 'x')
    with pytest.raises(tcg.TiltCalError, match='positive half-width'):
        tcg.build_axis(-10.0, 5, 'x')


def test_parse_node_list_accepts_commas_and_spaces():
    assert tcg.parse_node_list('-150,-75,0,75,150', 'x') == \
        [-150.0, -75.0, 0.0, 75.0, 150.0]
    assert tcg.parse_node_list('-100 0 100', 'y') == [-100.0, 0.0, 100.0]


def test_parse_node_list_rejects_non_monotonic_and_short_axes():
    with pytest.raises(tcg.TiltCalError, match='strictly increasing'):
        tcg.parse_node_list('0,100,50', 'x')
    with pytest.raises(tcg.TiltCalError, match='strictly increasing'):
        tcg.parse_node_list('0,0,100', 'x')       # a repeat is a 0-width cell
    with pytest.raises(tcg.TiltCalError, match='at least 2'):
        tcg.parse_node_list('0', 'x')
    with pytest.raises(tcg.TiltCalError, match='not a list of numbers'):
        tcg.parse_node_list('0,banana', 'x')


def test_home_index_locates_the_origin():
    assert tcg.home_index([-150.0, 0.0, 150.0], [-75.0, 0.0, 75.0]) == (1, 1)


def test_home_index_refuses_a_grid_without_the_origin():
    """An even --nodes over a symmetric box has no (0, 0) node.

    The home node is the REFERENCE node — every shipped residual is
    ``measured − m_home`` — so a grid without it has nothing to reference
    against and is refused rather than captured.
    """
    axis = tcg.build_axis(150.0, 4, 'x')
    assert 0.0 not in axis
    with pytest.raises(tcg.TiltCalError, match='home node'):
        tcg.home_index(axis, axis)


def test_centre_out_radii_never_decrease():
    """The whole point of centre-out: inner (low-stroke) data banks first.

    2026-08-09: the serpentine's visit 2 was the 212 mm far-corner diagonal,
    and the leg-0 collapse there cost the entire capture with zero field data
    banked. Radius-monotone ordering makes an extremity fault lose the
    corners, not the capture.
    """
    x = tcg.build_axis(150.0, 5, 'x')
    y = tcg.build_axis(150.0, 5, 'y')
    order = tcg.centre_out_order(x, y)
    radii = [math.hypot(n.x_mm, n.y_mm) for n in order]
    assert radii == sorted(radii)
    assert len(order) == 25
    assert len({(n.ix, n.iy) for n in order}) == 25


def test_centre_out_is_deterministic():
    x = tcg.build_axis(150.0, 5, 'x')
    y = tcg.build_axis(150.0, 5, 'y')
    assert tcg.centre_out_order(x, y) == tcg.centre_out_order(x, y)


def test_visit_order_measures_the_home_node_first_and_only_once():
    x = tcg.build_axis(150.0, 5, 'x')
    y = tcg.build_axis(150.0, 5, 'y')
    order = tcg.visit_order(x, y)
    assert (order[0].x_mm, order[0].y_mm) == (0.0, 0.0)
    assert len(order) == 25
    assert len({(n.ix, n.iy) for n in order}) == 25
    # Exactly one GRID visit to the origin (the end-of-sweep drift re-measure
    # is gate evidence, never a second value for the node).
    assert sum(1 for n in order if n.x_mm == 0.0 and n.y_mm == 0.0) == 1


def test_visit_order_leaves_the_corners_for_last():
    """Corners are the highest-stroke poses (worst-leg hard margin ~20 mm vs
    ~118 mm at home) — they carry the residual field's largest values AND its
    highest fault exposure, so they must come after the inner data is safe."""
    x = tcg.build_axis(150.0, 5, 'x')
    y = tcg.build_axis(150.0, 5, 'y')
    order = tcg.visit_order(x, y)
    last_four = {(n.x_mm, n.y_mm) for n in order[-4:]}
    assert last_four == {(-150.0, -150.0), (150.0, -150.0),
                         (-150.0, 150.0), (150.0, 150.0)}


def test_visit_order_indices_address_their_own_coordinates():
    """``ix``/``iy`` must index ``x_mm``/``y_mm`` — swapping them is silent on a
    square grid, so this uses different axis lengths."""
    x = [-10.0, 0.0, 10.0]
    y = [-20.0, -5.0, 0.0, 30.0]
    for node in tcg.visit_order(x, y):
        assert x[node.ix] == node.x_mm
        assert y[node.iy] == node.y_mm


# ── check poses ──────────────────────────────────────────────────────────


def test_check_poses_are_off_node_and_inside_the_hull():
    x = tcg.build_axis(150.0, 5, 'x')
    y = tcg.build_axis(150.0, 5, 'y')
    poses = tcg.check_poses(x, y, 6)
    assert len(poses) == 6
    for cx, cy in poses:
        assert x[0] < cx < x[-1]
        assert y[0] < cy < y[-1]
        assert cx not in x, 'a check pose on a node measures the stored value'
        assert cy not in y


def test_check_poses_are_deterministic():
    """Rung C2 re-verifies the *same* poses after a re-level without
    recapturing, so this ordering is load-bearing, not cosmetic."""
    x = tcg.build_axis(150.0, 5, 'x')
    y = tcg.build_axis(150.0, 5, 'y')
    assert tcg.check_poses(x, y, 6) == tcg.check_poses(x, y, 6)


def test_check_poses_cover_all_four_quadrants_before_doubling_up():
    x = tcg.build_axis(150.0, 5, 'x')
    y = tcg.build_axis(150.0, 5, 'y')
    first_four = tcg.check_poses(x, y, 4)
    quadrants = {((cx >= 0), (cy >= 0)) for cx, cy in first_four}
    assert len(quadrants) == 4


def test_check_poses_prefer_the_outermost_cells():
    """The 2026-07-28 table puts the largest residual at the corner, so that is
    where an interpolation error costs a ball."""
    x = tcg.build_axis(150.0, 5, 'x')
    y = tcg.build_axis(150.0, 5, 'y')
    first_four = tcg.check_poses(x, y, 4)
    for cx, cy in first_four:
        assert math.hypot(cx, cy) == pytest.approx(math.hypot(112.5, 112.5))


def test_check_poses_handles_zero_and_oversized_counts():
    x = tcg.build_axis(150.0, 3, 'x')
    y = tcg.build_axis(150.0, 3, 'y')
    assert tcg.check_poses(x, y, 0) == []
    # 2x2 = 4 cells; asking for more returns all of them, not a crash.
    assert len(tcg.check_poses(x, y, 99)) == 4


# ── residual math ────────────────────────────────────────────────────────


def test_mounting_offset_comes_from_the_generated_constant():
    expected = tuple(math.radians(d) for d in hw.JB_OP_INCLINOMETER_OFFSET_DEG)
    assert tcg.mounting_offset_rad() == expected


class _LevellingCtx:
    """Minimal stand-in for the orchestrator context LevellingHandler mutates."""

    def __init__(self, tilt_reading):
        self.tilt_reading = list(tilt_reading)
        self.pose_offset_rad = None
        self.control_mode = ''
        self.request = None
        self.operation_result = True

    def clear_commands(self):
        pass


def test_residual_matches_the_real_levelling_handler():
    """Drive the production handler and compare, rather than restating it.

    C-LEVEL-2 defines a residual as *the exact LevellingHandler formula*
    (``state_machine.py:455-460``). Asserting against a hand-written
    ``raw + radians(offset)`` here would pass just as happily if both this tool
    and the handler flipped the sign together, and a sign flip inverts every
    node in the map — aiming the machine roughly twice as badly as no map at all.
    """
    from jugglebot.state_machine import LevellingHandler

    handler = LevellingHandler(
        inclinometer_offset_deg=hw.JB_OP_INCLINOMETER_OFFSET_DEG, settle_s=0.0)
    handler._phase_idx = LevellingHandler._PHASES.index('level_get_tilt')

    raw = (0.01234, -0.04567)
    ctx = _LevellingCtx(raw)
    handler.execute(ctx)
    assert ctx.pose_offset_rad is not None, 'handler phase did not run'

    stats = tcg.residual_from_reads([raw], tcg.mounting_offset_rad())
    assert stats.res_tx == pytest.approx(ctx.pose_offset_rad[0], abs=1e-15)
    assert stats.res_ty == pytest.approx(ctx.pose_offset_rad[1], abs=1e-15)


def test_residual_adds_the_offset_it_does_not_subtract():
    """Belt and braces on the sign, with the real (negative) constants."""
    stats = tcg.residual_from_reads([(0.0, 0.0)], OFFSET)
    assert stats.res_tx == pytest.approx(OFFSET[0])
    assert stats.res_ty == pytest.approx(OFFSET[1])
    assert OFFSET[0] < 0.0 and OFFSET[1] < 0.0, (
        'the shipped offsets are negative; if that changes, confirm this test '
        'still distinguishes an add from a subtract')


def test_residual_averages_and_reports_sample_sd():
    stats = tcg.residual_from_reads([(1.0, 5.0), (3.0, 5.0)], (0.0, 0.0))
    assert stats.raw_mean_tx == pytest.approx(2.0)
    # Sample sd (ddof=1) = sqrt(2); the population sd would be 1.0. ddof=0
    # understates the spread at the small N this tool uses, and the spread
    # gates the home node.
    assert stats.sd_tx == pytest.approx(math.sqrt(2.0))
    assert stats.sd_ty == pytest.approx(0.0)
    assert (stats.n_ok, stats.n_total) == (2, 2)


def test_residual_drops_nan_reads_but_counts_them():
    nan = float('nan')
    stats = tcg.residual_from_reads(
        [(1.0, 1.0), (nan, nan), (3.0, 3.0), (nan, 2.0)], (0.0, 0.0))
    assert (stats.n_ok, stats.n_total) == (2, 4)
    assert stats.raw_mean_tx == pytest.approx(2.0)


def test_residual_with_every_read_failed_is_all_nan():
    nan = float('nan')
    stats = tcg.residual_from_reads([(nan, nan)] * 4, OFFSET)
    assert stats.n_ok == 0
    assert stats.n_total == 4
    assert math.isnan(stats.res_tx) and math.isnan(stats.res_ty)
    assert not tcg.node_is_good(stats, 4)


def test_residual_accepts_a_generator():
    """Regression: a generator was consumed by the finite-filter, leaving
    ``n_total = 0`` — which made ``node_is_good`` pass a node with no reads."""
    stats = tcg.residual_from_reads(((1.0, 1.0) for _ in range(4)), (0.0, 0.0))
    assert (stats.n_ok, stats.n_total) == (4, 4)


def test_node_is_good_needs_an_sd_and_half_the_requested_reads():
    def _stats(n_ok):
        return tcg.ReadStats(n_ok=n_ok, n_total=8, raw_mean_tx=0.0,
                             raw_mean_ty=0.0, res_tx=0.0, res_ty=0.0,
                             sd_tx=0.0, sd_ty=0.0)
    assert not tcg.node_is_good(_stats(1), 8)    # no sd from one sample
    assert not tcg.node_is_good(_stats(3), 8)    # under half
    assert tcg.node_is_good(_stats(4), 8)
    assert tcg.node_is_good(_stats(8), 8)


# ── home reference screen + end-of-sweep drift gate ─────────────────────
#
# The start-of-capture STALE LEVEL REFERENCE abort is RETIRED (2026-08-10).
# It compared |m_home| — whose dominant variance is the level reference's own
# single-sample scatter, measured 1.2-1.7 mrad/axis on 2026-08-09 — against a
# 1.5 mrad floor derived from 4.5 s of read noise, and false-aborted 40-60%
# of healthy attempts. Under home-referencing m_home CANCELS out of every
# shipped residual, so what remains is a tri-state sanity screen on the
# reference measurement plus a matched-footing drift gate at sweep end.


def _home_stats(res_tx, res_ty, sd, n_ok=30):
    return tcg.ReadStats(n_ok=n_ok, n_total=30, raw_mean_tx=0.0,
                         raw_mean_ty=0.0, res_tx=res_tx, res_ty=res_ty,
                         sd_tx=sd, sd_ty=sd)


def test_home_reference_accepts_the_residual_that_falsely_aborted_c0():
    """The 2026-08-09 abort case: m_home_tx = +1.6273 mrad, sd 0.50 mrad —
    ~1.0-1.4 sigma of the level path's real physics, killed by the old gate
    (missed its 1.5020 mrad tolerance by 8.3%). It must pass clean now."""
    verdict, message = tcg.home_reference_verdict(
        _home_stats(0.0016272979, -0.0011481, 0.0005))
    assert verdict == 'ok', message


def test_home_reference_warns_on_a_grossly_stale_level():
    verdict, message = tcg.home_reference_verdict(_home_stats(0.02, 0.0, 5e-4))
    assert verdict == 'warn'
    assert 'HARMLESS' in message
    assert 'level' in message


def test_home_reference_aborts_only_at_the_sanity_ceiling():
    verdict, message = tcg.home_reference_verdict(_home_stats(0.06, 0.0, 5e-4))
    assert verdict == 'abort'
    assert 'sanity ceiling' in message
    # Boundary: the WARN threshold itself is still ok-side of warn.
    assert tcg.home_reference_verdict(
        _home_stats(tcg.HOME_REF_WARN_RAD, 0.0, 5e-4))[0] == 'ok'
    assert tcg.home_reference_verdict(
        _home_stats(tcg.HOME_REF_WARN_RAD + 1e-4, 0.0, 5e-4))[0] == 'warn'


def test_home_reference_aborts_when_the_sensor_barely_answered():
    verdict, message = tcg.home_reference_verdict(
        _home_stats(0.0, 0.0, 0.0, n_ok=1))
    assert verdict == 'abort'
    assert 'finite reads' in message


# ── the home-ANCHOR series (2026-08-10) ─────────────────────────────────
#
# The tight statistical drift gate that briefly replaced the staleness abort
# is RETIRED. It compared the start and end home means against a
# read-noise-derived tolerance (0.87/0.94 mrad) and aborted BOTH complete C0
# captures on 2026-08-10 over a reproducible ~1.6-1.8 mrad ty offset, with the
# causal /gravity_offset monitor silent and the ODrive forensics clean — i.e.
# the correction had NOT changed. Discarding a finished, usable capture over
# an effect ~4.8x inside the owner's 0.5 deg repeatability tolerance was
# the bug. What replaces it: k home anchors interleaved through the sweep,
# the mean of them as the shipped reference, and gates sized to the
# repeatability tolerance rather than to read noise.


def _anchor(t_s, m_tx, m_ty, sd=5e-4, n_ok=30):
    return tcg.HomeAnchor(t_s=t_s, m_tx=m_tx, m_ty=m_ty, sd_tx=sd, sd_ty=sd,
                          n_ok=n_ok)


#: The C0 run-1 artifact, verbatim (temp/logs/tilt_cal_grid_20260810_115343*):
#: home measured at the start of the sweep and again 138 s later at its end.
#: The ty pair differs by +1.812 mrad — the number the retired gate aborted on.
_C0_RUN1_START = _anchor(4.6, 0.00012847090099471145, 0.0015778696825412158,
                         sd=5.485521847912701e-4)
_C0_RUN1_END = _anchor(132.9, 0.0006909304729026296, 0.003389884411325777,
                       sd=7.119917258411154e-4)


def test_anchor_mean_of_one_anchor_is_that_anchor_bit_identically():
    """The reduction to the previous design must be EXACT, not approximate."""
    mean = tcg.anchor_mean_rad([_C0_RUN1_START])
    assert mean[0] == _C0_RUN1_START.m_tx
    assert mean[1] == _C0_RUN1_START.m_ty


@pytest.mark.parametrize('k', [1, 2, 3, 4, 5, 6, 7, 8])
def test_zero_scatter_anchors_reduce_bit_identically_to_single_home(k):
    """k identical anchors must give back the value with NO last-digit drift.

    This is the property that lets the anchor-mean design ship as a
    refinement rather than a change: a perfectly repeatable machine produces
    exactly the map the single-home referencing produced. Naive ``sum/k``
    fails it — three copies of a float sum to a rounded ``3a`` whose quotient
    by 3 can miss ``a`` by an ULP, and a committed machine-written YAML that
    changes in its last digit because a redundant measurement AGREED is an
    unexplainable diff.
    """
    value_tx, value_ty = 0.0016272979, -0.0011481
    anchors = [_anchor(10.0 * i, value_tx, value_ty) for i in range(k)]
    mean = tcg.anchor_mean_rad(anchors)
    assert mean[0] == value_tx
    assert mean[1] == value_ty


def test_anchor_mean_is_the_arithmetic_mean_when_they_differ():
    anchors = [_anchor(0.0, 0.001, 0.002), _anchor(50.0, 0.003, 0.004),
               _anchor(100.0, 0.005, 0.000)]
    mean = tcg.anchor_mean_rad(anchors)
    assert mean[0] == pytest.approx(0.003)
    assert mean[1] == pytest.approx(0.002)


def test_anchor_mean_ignores_an_unusable_anchor_and_refuses_an_empty_series():
    anchors = [_anchor(0.0, 0.001, 0.002),
               _anchor(50.0, 99.0, 99.0, n_ok=1),      # no sd, not usable
               _anchor(100.0, 0.003, 0.004)]
    mean = tcg.anchor_mean_rad(anchors)
    assert mean[0] == pytest.approx(0.002)
    with pytest.raises(tcg.TiltCalError, match='no usable home anchor'):
        tcg.anchor_mean_rad([_anchor(0.0, 0.001, 0.002, n_ok=1)])


def test_anchor_spread_reports_pp_trend_and_the_worst_consecutive_step():
    """Trend is END minus START and signed; p-p is order-free; max_step is
    over CONSECUTIVE anchors — the three together are what separate a
    monotonic warm-up from scatter from a discrete event."""
    anchors = [_anchor(0.0, 0.0, 0.000), _anchor(30.0, 0.0, 0.003),
               _anchor(60.0, 0.0, 0.001), _anchor(90.0, 0.0, 0.002)]
    spread = tcg.anchor_spread(anchors)
    assert spread['ty']['pp'] == pytest.approx(0.003)
    assert spread['ty']['trend'] == pytest.approx(0.002)
    assert spread['ty']['max_step'] == pytest.approx(0.003)
    assert spread['ty']['max_step_index'] == 0
    assert spread['n_anchors'] == 4


def test_todays_reproducible_home_offset_no_longer_aborts():
    """THE regression this change exists for — run 1 of 2026-08-10, verbatim.

    Both C0 captures completed all nine nodes and were then thrown away by
    the drift gate over this exact pair (ty +1.812 mrad against a 0.865 mrad
    tolerance), with /gravity_offset silent and the forensics clean. It must
    now be shipped. It sits just UNDER the 2.0 mrad p-p WARN line, so the
    verdict is 'ok' — the WARN is sized above the observed effect deliberately
    (a warning that fires on every healthy capture is a warning nobody reads);
    a third anchor continuing this trend crosses it, which is exactly when the
    operator should start recording the series.
    """
    verdict, message = tcg.anchor_series_verdict(
        [_C0_RUN1_START, _C0_RUN1_END])
    assert verdict == 'ok', message
    spread = tcg.anchor_spread([_C0_RUN1_START, _C0_RUN1_END])
    assert spread['ty']['pp'] == pytest.approx(0.0018120147, abs=1e-9)
    assert spread['ty']['pp'] < tcg.ANCHOR_PP_WARN_RAD
    # ...and the same series is nowhere near either abort line.
    assert spread['ty']['pp'] < tcg.ANCHOR_PP_ABORT_RAD
    assert spread['ty']['max_step'] < tcg.ANCHOR_STEP_ABORT_RAD


def test_anchor_series_warns_above_the_pp_warn_but_does_not_abort():
    """A WARN must be prominent, NON-blocking, and name BOTH open mechanisms —
    an operator who reads one as settled stops recording the series, and the
    series is the only thing that discriminates them."""
    anchors = [_anchor(0.0, 0.0, 0.000), _anchor(60.0, 0.0, 0.0015),
               _anchor(120.0, 0.0, 0.0025)]
    verdict, message = tcg.anchor_series_verdict(anchors)
    assert verdict == 'warn'
    assert 'ARRIVAL REPEATABILITY' in message
    assert 'WARM-UP' in message
    assert 'discriminate' in message
    assert 'VALID' in message


def test_anchor_series_aborts_at_the_half_degree_repeatability_ceiling():
    """0.0087 rad = 0.5 deg is the OWNER-STATED repeatability tolerance, not a
    statistical bound: past it, which arrival was used as the reference stops
    being a rounding error against the 0.15 deg theta_acc."""
    assert tcg.ANCHOR_PP_ABORT_RAD == pytest.approx(math.radians(0.5),
                                                    abs=5e-5)
    # Ramped over three steps so no CONSECUTIVE step trips its own (5 mrad)
    # ceiling — this test is about the p-p line and nothing else.
    def _ramp(total):
        return [_anchor(60.0 * i, 0.0, total * i / 3.0) for i in range(4)]

    assert tcg.anchor_series_verdict(
        _ramp(tcg.ANCHOR_PP_ABORT_RAD - 1e-5))[0] == 'warn'
    verdict, message = tcg.anchor_series_verdict(
        _ramp(tcg.ANCHOR_PP_ABORT_RAD + 1e-5))
    assert verdict == 'abort'
    assert 'REPEATABILITY TOLERANCE' in message


def test_anchor_series_aborts_on_a_discrete_consecutive_step():
    """A 5 mrad step between two ADJACENT anchors is an event (re-level,
    relaunch re-push, mechanical snap), and stays fatal even though the total
    spread is well inside the 0.5 deg ceiling — the two gates make different
    physical claims."""
    anchors = [_anchor(0.0, 0.0, 0.0), _anchor(60.0, 0.0, 0.0055),
               _anchor(120.0, 0.0, 0.0056)]
    verdict, message = tcg.anchor_series_verdict(anchors)
    assert verdict == 'abort'
    assert 'DISCRETE STEP' in message
    assert 'anchors 1 and 2' in message
    # The same total spread reached SMOOTHLY (no single step over the limit)
    # is only a WARN — that is the whole distinction.
    smooth = [_anchor(60.0 * i, 0.0, 0.0056 * i / 4.0) for i in range(5)]
    assert tcg.anchor_series_verdict(smooth)[0] == 'warn'


def test_anchor_series_aborts_when_an_anchor_barely_answered():
    anchors = [_anchor(0.0, 0.0, 0.001), _anchor(60.0, 0.0, 0.0, n_ok=1)]
    verdict, message = tcg.anchor_series_verdict(anchors)
    assert verdict == 'abort'
    assert 'finite reads' in message
    assert tcg.anchor_series_verdict([])[0] == 'abort'


def test_anchor_table_lines_render_every_anchor_with_its_clock():
    lines = tcg.anchor_table_lines([_C0_RUN1_START, _C0_RUN1_END])
    assert len(lines) == 3                     # header + 2 rows
    assert 't_s' in lines[0] and 'n_ok' in lines[0]
    assert '+0.001578' in lines[1]
    assert '132.9' in lines[2]


# ── mid-sweep anchor scheduling ─────────────────────────────────────────


def test_revisit_schedule_for_the_3x3_probe_grid():
    """9 nodes = 8 non-home visits; at every 4 that is ONE mid-sweep anchor,
    so 3 anchors total (start + 1 + end)."""
    assert tcg.home_revisit_after_visits(9, 4) == [4]


def test_revisit_schedule_for_the_5x5_default_grid():
    """25 nodes = 24 non-home visits ⇒ five mid-sweep anchors ⇒ 7 total."""
    assert tcg.home_revisit_after_visits(25, 4) == [4, 8, 12, 16, 20]


def test_revisit_schedule_never_anchors_immediately_before_the_end_anchor():
    """The last non-home visit is skipped: the end-of-sweep re-measure already
    anchors there, and two home measurements separated by nothing but a
    re-move would sample the same arrival twice — which is precisely the
    variation the series exists to measure."""
    # 9 nodes, every 8 non-home visits: the only candidate IS the last one.
    assert tcg.home_revisit_after_visits(9, 8) == []
    assert 24 not in tcg.home_revisit_after_visits(25, 6)
    assert tcg.home_revisit_after_visits(25, 6) == [6, 12, 18]


def test_revisit_schedule_can_be_disabled():
    """0 keeps the start and end anchors and nothing else — the minimum the
    anchor-mean design degrades to, and still an improvement on one anchor."""
    assert tcg.home_revisit_after_visits(25, 0) == []
    assert tcg.home_revisit_after_visits(25, -1) == []


def test_revisit_every_defaults_to_four_on_the_cli():
    args = tcg.build_parser().parse_args([])
    assert args.home_revisit_every == tcg.DEFAULT_HOME_REVISIT_EVERY == 4
    assert tcg.build_parser().parse_args(
        ['--home-revisit-every', '0']).home_revisit_every == 0


# ── the forensics epilogue is conditional ───────────────────────────────


def test_forensics_epilogue_fires_only_on_a_real_drive_error():
    """The 'capture evidence BEFORE any relaunch' line used to print on EVERY
    abort — including the two 2026-08-10 gate aborts whose drives were clean.
    Telling an operator to preserve evidence that does not exist trains them
    to skip the line on the one abort where it is genuinely perishable."""
    clean = {'motor_states': [{'axis': 0, 'active_errors': 0,
                               'disarm_reason': 0}],
             'motor_error_edges': {}}
    assert tcg.forensics_has_drive_error(clean) is False
    assert tcg.forensics_has_drive_error({}) is False
    spun_out = {'motor_states': [{'axis': 0, 'active_errors': 0,
                                  'disarm_reason': 67108864}],
                'motor_error_edges': {}}
    assert tcg.forensics_has_drive_error(spun_out) is True
    # The edge latch alone is enough: the abort-time snapshot can be clean
    # while the fault-time edge is not (the 2026-08-09 shape).
    edge_only = {'motor_states': [{'axis': 0, 'active_errors': 0,
                                   'disarm_reason': 0}],
                 'motor_error_edges': {'0': {'active_errors': 67108864,
                                             'disarm_reason': 0}}}
    assert tcg.forensics_has_drive_error(edge_only) is True


def test_fault_error_is_a_tilt_cal_error_subclass():
    """The per-node handlers re-raise TiltCalFaultError and demote plain
    TiltCalError to a failed node — the hierarchy IS the contract."""
    assert issubclass(tcg.TiltCalFaultError, tcg.TiltCalError)
    assert not issubclass(tcg.TiltCalError, tcg.TiltCalFaultError)


# ── ODrive error decode (forensics) ─────────────────────────────────────


def test_decode_names_the_2026_08_09_spinout():
    assert tcg.decode_error_bits(67108864) == ['SPINOUT_DETECTED']


def test_decode_handles_zero_combined_and_unknown_bits():
    assert tcg.decode_error_bits(0) == []
    assert tcg.decode_error_bits(512 | 4096) == [
        'DC_BUS_UNDER_VOLTAGE', 'CURRENT_LIMIT_VIOLATION']
    # An undecodable bit is surfaced, never eaten — it is exactly the datum a
    # forensics dump must not lose.
    assert tcg.decode_error_bits(1 << 23) == ['UNKNOWN_BITS_0x00800000']
    assert tcg.decode_error_bits((1 << 23) | 512) == [
        'DC_BUS_UNDER_VOLTAGE', 'UNKNOWN_BITS_0x00800000']


def test_vendored_error_table_matches_the_authoritative_one():
    """The tool vendors ERROR_CODES (its package __init__ needs the ROS
    workspace); this pin makes drift a test failure instead of a silent
    wrong decode at the robot."""
    odrive = pytest.importorskip(
        'jugglebot.can.odrive',
        reason='jugglebot.can needs jugglebot_interfaces + python-can')
    assert tcg.ODRIVE_ERROR_CODES == odrive.ERROR_CODES


# ── IK stroke-margin preflight ───────────────────────────────────────────


def test_stroke_preflight_home_sits_mid_stroke():
    """Sanity anchor for the frame convention (ik_solver's module header says
    'offset from the active position'; its docstrings and every call site are
    stow-relative): (0, 0, 170) must land mid-stroke, not near a bound."""
    from jugglebot.motion.ik_solver import pose_to_leg_lengths
    ext = pose_to_leg_lengths(np.array([0.0, 0.0, 170.0]), np.eye(3),
                              tcg._stewart_geometry())
    assert all(100.0 < e < 200.0 for e in ext), ext


def test_stroke_preflight_passes_the_default_grid_and_checks():
    x = tcg.build_axis(150.0, 5, 'x')
    y = tcg.build_axis(150.0, 5, 'y')
    poses = ([(n.x_mm, n.y_mm) for n in tcg.visit_order(x, y)]
             + tcg.check_poses(x, y, 6) + [(0.0, 0.0)])
    assert tcg.stroke_margin_problems(poses, 170.0) == []


def test_stroke_preflight_refuses_an_overtravel_pose_naming_the_leg():
    problems = tcg.stroke_margin_problems([(0.0, 300.0)], 170.0)
    assert len(problems) == 1
    message = problems[0]
    assert '+300.0' in message
    assert 'leg 0' in message          # legs 0/3 hit 287.4 mm; 0 reported
    assert 'hard bound' in message


def test_stroke_preflight_margin_is_the_gate_not_the_hard_bound():
    """A pose INSIDE the hard bound but within 10 mm of it is refused: the
    firmware clamps per leg at 500 Hz and the node's reading would be
    silently wrong, never loudly rejected."""
    # (165, 165, 170): worst leg 267.4 mm — legal against the 275 mm hard
    # bound, but only 7.6 mm of margin (measured 2026-08-10, identity R).
    problems = tcg.stroke_margin_problems([(165.0, 165.0)], 170.0)
    assert problems, 'a <10 mm margin must be refused even though legal'
    # ...and a pose with >=10 mm margin passes: (160, 160) sits at 11.7 mm.
    assert tcg.stroke_margin_problems([(160.0, 160.0)], 170.0) == []


# ── map document ─────────────────────────────────────────────────────────


def _asymmetric_capture():
    """3 x-nodes, 4 y-nodes, unequal spacing, no two residuals equal."""
    x = [-100.0, -25.0, 150.0]
    y = [-150.0, -10.0, 30.0, 120.0]
    results = {}
    for ix in range(len(x)):
        for iy in range(len(y)):
            results[(ix, iy)] = tcg.ReadStats(
                n_ok=8, n_total=8, raw_mean_tx=0.0, raw_mean_ty=0.0,
                res_tx=0.001 * (ix + 1) + 0.0001 * iy,
                res_ty=-0.002 * (iy + 1) - 0.0003 * ix,
                sd_tx=1e-5, sd_ty=2e-5)
    return x, y, results


_CAPTURED = {
    'date': '2026-08-03',
    'git_sha': 'abc123',
    'tool': tcg.TOOL_NAME,
    'args': [],
    'uptime_ms_first': 1000,
    'uptime_ms_last': 240000,
    'level_offset_rad': [0.001, -0.002],
    'base_condition': 'unit test',
}


def test_map_document_round_trips_through_the_production_loader():
    """The writer's output must be acceptable to the loader the node uses.

    ``parse_tilt_map`` is the exact function ``trajectory_node._load_tilt_map``
    calls, so this is a statement about the machine and not about a schema doc.
    """
    x, y, results = _asymmetric_capture()
    doc = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8, (0.0, 0.0))
    parsed = tilt_map.parse_tilt_map(doc)

    assert parsed.shape == (len(y), len(x))
    assert list(parsed.x_mm) == x
    assert list(parsed.y_mm) == y
    assert parsed.z_mm == pytest.approx(170.0)
    # The version the node will publish on TrajectoryStatus.tilt_map_version is
    # the one the tool computes locally to verify the auto-apply readback.
    assert parsed.version == tilt_map.map_version(doc)

    # Every node is recoverable at its own coordinates (lookup is exact at nodes).
    for ix in range(len(x)):
        for iy in range(len(y)):
            tx, ty = tilt_map.lookup(parsed, x[ix], y[iy])
            assert tx == pytest.approx(results[(ix, iy)].res_tx, abs=1e-9)
            assert ty == pytest.approx(results[(ix, iy)].res_ty, abs=1e-9)


def test_map_document_is_indexed_iy_ix():
    """A transposed grid is invisible on a square capture and fatal on a real
    one, so the fixture is 3 x 4."""
    x, y, results = _asymmetric_capture()
    doc = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8, (0.0, 0.0))
    tx = doc['residual_rad']['tx']
    assert len(tx) == len(y)
    assert all(len(row) == len(x) for row in tx)
    for ix in range(len(x)):
        for iy in range(len(y)):
            assert tx[iy][ix] == pytest.approx(results[(ix, iy)].res_tx,
                                               abs=1e-9)


def test_map_document_refuses_an_unusable_node():
    x, y, results = _asymmetric_capture()
    results[(1, 2)] = tcg.ReadStats(n_ok=0, n_total=8, raw_mean_tx=float('nan'),
                                    raw_mean_ty=float('nan'),
                                    res_tx=float('nan'), res_ty=float('nan'),
                                    sd_tx=float('nan'), sd_ty=float('nan'))
    with pytest.raises(tcg.TiltCalError) as excinfo:
        tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8, (0.0, 0.0))
    message = str(excinfo.value)
    assert 'unusable node' in message
    # The refusal must name the node, in operator coordinates.
    assert '-25.0' in message and '30.0' in message
    assert 'never interpolated' in message


def test_map_document_refuses_a_missing_node():
    x, y, results = _asymmetric_capture()
    del results[(0, 0)]
    with pytest.raises(tcg.TiltCalError, match='unusable node'):
        tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8, (0.0, 0.0))


def test_validate_refuses_an_out_of_bound_residual_with_a_physical_hint():
    """> MAX_ABS_RESIDUAL_RAD is a capture fault, not a calibration — and the
    operator reading the refusal is standing at a robot, so it names causes."""
    x, y, results = _asymmetric_capture()
    results[(2, 3)] = results[(2, 3)]._replace(
        res_tx=tilt_map.MAX_ABS_RESIDUAL_RAD * 2.0)
    doc = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8, (0.0, 0.0))
    with pytest.raises(tcg.TiltCalError) as excinfo:
        tcg.validate_map_document(doc)
    message = str(excinfo.value)
    assert 'REJECTED' in message
    assert 'stale level reference' in message
    assert 'stroke clamp' in message


def test_validate_accepts_a_residual_at_the_measured_envelope():
    """0.604 deg is the largest residual the 2026-07-28 seed table measured; a
    real capture must not trip the sanity bound."""
    x, y, results = _asymmetric_capture()
    results[(2, 3)] = results[(2, 3)]._replace(res_tx=math.radians(0.604))
    doc = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8, (0.0, 0.0))
    assert tcg.validate_map_document(doc) is not None


def test_dumped_yaml_reloads_and_carries_a_do_not_edit_header():
    x, y, results = _asymmetric_capture()
    doc = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8, (0.0, 0.0))
    text = tcg.dump_map_yaml(doc)
    assert 'MACHINE-WRITTEN' in text
    assert 'C-LEVEL-2' in text
    reloaded = yaml.safe_load(text)
    assert tilt_map.map_version(reloaded) == tilt_map.map_version(doc)
    tilt_map.parse_tilt_map(reloaded)


def test_written_file_loads_through_load_tilt_map(tmp_path):
    x, y, results = _asymmetric_capture()
    doc = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8, (0.0, 0.0))
    path = tmp_path / 'tilt_calibration.yaml'
    path.write_text(tcg.dump_map_yaml(doc))
    loaded = tilt_map.load_tilt_map(str(path))
    assert loaded.version == tilt_map.map_version(doc)
    assert loaded.metadata['base_condition'] == 'unit test'


def test_map_version_is_stable_across_a_rewrite_of_identical_numbers():
    """The tool re-emits this file every run; a version that churns when
    nothing was recalibrated sends an operator hunting a difference that does
    not exist."""
    x, y, results = _asymmetric_capture()
    first = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8, (0.0, 0.0))
    later = dict(_CAPTURED, git_sha='different', uptime_ms_last=999999)
    second = tcg.build_map_document(x, y, 170.0, results, later, 8, (0.0, 0.0))
    assert tilt_map.map_version(first) == tilt_map.map_version(second)


def test_map_version_changes_when_a_single_node_changes():
    x, y, results = _asymmetric_capture()
    first = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8, (0.0, 0.0))
    results[(0, 1)] = results[(0, 1)]._replace(res_tx=0.00777)
    second = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8, (0.0, 0.0))
    assert tilt_map.map_version(first) != tilt_map.map_version(second)


# ── home-referencing math ────────────────────────────────────────────────
#
# The 2026-08-10 redesign: shipped residual = measured − m_home, so the
# loaded correction C_cap cancels EXACTLY (m_i = f(P_i) − C_cap ⇒
# M(P_i) = f(P_i) − f(home)). These tests pin the three properties the
# contract states: exact cancellation of a constant, map(home) == 0 exactly,
# and bilinear commuting with the constant.


def _capture_with_home():
    """An asymmetric grid that CONTAINS (0, 0), with a nonzero home residual
    (the realistic case: the level reference is never exactly zero)."""
    x = [-100.0, 0.0, 150.0]
    y = [-150.0, 0.0, 30.0, 120.0]
    results = {}
    for ix in range(len(x)):
        for iy in range(len(y)):
            results[(ix, iy)] = tcg.ReadStats(
                n_ok=8, n_total=8, raw_mean_tx=0.0, raw_mean_ty=0.0,
                res_tx=0.0011 * (ix + 1) + 0.00013 * iy,
                res_ty=-0.0019 * (iy + 1) - 0.00031 * ix,
                sd_tx=1e-5, sd_ty=2e-5)
    hx, hy = tcg.home_index(x, y)
    home = results[(hx, hy)]
    return x, y, results, (home.res_tx, home.res_ty)


def test_home_node_ships_exactly_zero():
    """Not approximately — EXACTLY. The home node subtracts itself, and float
    subtraction of identical values is exact, so the old gate's 'residual(home)
    ≈ 0 by construction' claim becomes '== 0.0 by arithmetic'."""
    x, y, results, home_ref = _capture_with_home()
    doc = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8, home_ref)
    hx, hy = tcg.home_index(x, y)
    assert doc['residual_rad']['tx'][hy][hx] == 0.0
    assert doc['residual_rad']['ty'][hy][hx] == 0.0


def test_a_stale_but_constant_correction_cancels_exactly():
    """THE worked example behind the redesign: add a constant to every
    measured residual — home included — exactly what a stale C_cap does (the
    2026-08-09 level scatter was 1.2-2.4 mrad between successive levels), and
    the shipped document must be BYTE-IDENTICAL, version included."""
    x, y, results, home_ref = _capture_with_home()
    clean = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8,
                                   home_ref)

    stale_shift = (0.0024, -0.0017)   # ~the worst observed level-to-level jump
    shifted = {
        key: stats._replace(res_tx=stats.res_tx + stale_shift[0],
                            res_ty=stats.res_ty + stale_shift[1])
        for key, stats in results.items()
    }
    shifted_ref = (home_ref[0] + stale_shift[0], home_ref[1] + stale_shift[1])
    stale = tcg.build_map_document(x, y, 170.0, shifted, _CAPTURED, 8,
                                   shifted_ref)

    assert stale['residual_rad'] == clean['residual_rad']
    assert tilt_map.map_version(stale) == tilt_map.map_version(clean)


def test_bilinear_lookup_commutes_with_the_home_constant():
    """No interpolation change accompanies home-referencing: a constant shift
    passes exactly through the bilinear weights, so lookups on the
    home-referenced map equal lookups on the raw map minus m_home."""
    x, y, results, home_ref = _capture_with_home()
    referenced = tilt_map.parse_tilt_map(
        tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8, home_ref))
    raw = tilt_map.parse_tilt_map(
        tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8,
                               (0.0, 0.0)))
    for qx, qy in [(-50.0, -60.0), (75.0, 15.0), (0.0, 0.0), (149.0, 119.0)]:
        r_tx, r_ty = tilt_map.lookup(referenced, qx, qy)
        m_tx, m_ty = tilt_map.lookup(raw, qx, qy)
        assert r_tx == pytest.approx(m_tx - home_ref[0], abs=1e-12)
        assert r_ty == pytest.approx(m_ty - home_ref[1], abs=1e-12)


def test_anchor_metadata_round_trips_through_the_production_parser(tmp_path):
    """The whole anchor series must survive write → load as metadata.

    ``tilt_map.parse_tilt_map`` passes the ``captured`` block through
    untouched (``tilt_map.py:462-470``), so the nested ``anchors`` list needs
    no loader change — this test is what says so, and what would fail if the
    loader ever started validating or flattening that block. The series is the
    evidence that discriminates arrival repeatability from sensor warm-up
    across captures; losing it at the write boundary would lose the answer.
    """
    x, y, results, home_ref = _capture_with_home()
    anchors = [tcg.HomeAnchor(t_s=4.6, m_tx=home_ref[0], m_ty=home_ref[1],
                              sd_tx=5.5e-4, sd_ty=7.0e-4, n_ok=30),
               tcg.HomeAnchor(t_s=68.2, m_tx=home_ref[0] + 3e-4,
                              m_ty=home_ref[1] + 9e-4, sd_tx=5.1e-4,
                              sd_ty=6.6e-4, n_ok=30),
               tcg.HomeAnchor(t_s=132.9, m_tx=home_ref[0] + 5.6e-4,
                              m_ty=home_ref[1] + 1.81e-3, sd_tx=7.1e-4,
                              sd_ty=8.7e-4, n_ok=30)]
    mean = tcg.anchor_mean_rad(anchors)
    spread = tcg.anchor_spread(anchors)
    captured = dict(_CAPTURED)
    captured['home_reference'] = {
        'referenced': 'anchor-mean',
        'anchors': [a.as_dict() for a in anchors],
        'anchor_mean_rad': list(mean),
        'anchor_pp_rad': [spread['tx']['pp'], spread['ty']['pp']],
        'trend_rad': [spread['tx']['trend'], spread['ty']['trend']],
        'anchor_verdict': 'OK',
    }
    doc = tcg.build_map_document(x, y, 170.0, results, captured, 8, mean)
    path = tmp_path / 'tilt_calibration.yaml'
    path.write_text(tcg.dump_map_yaml(doc))
    loaded = tilt_map.load_tilt_map(str(path))

    home = loaded.metadata['home_reference']
    assert home['referenced'] == 'anchor-mean'
    assert len(home['anchors']) == 3
    assert home['anchors'][2]['t_s'] == pytest.approx(132.9)
    assert home['anchors'][2]['m_ty_rad'] == pytest.approx(
        home_ref[1] + 1.81e-3)
    assert home['anchors'][0]['n_ok'] == 30
    assert home['anchor_mean_rad'] == pytest.approx(list(mean))
    assert home['trend_rad'][1] == pytest.approx(1.81e-3)
    assert home['anchor_pp_rad'][1] == pytest.approx(1.81e-3)


def test_anchor_mean_referencing_ships_the_mean_not_the_first_anchor(tmp_path):
    """The node grid must be shifted by the MEAN — checking the map, not just
    the helper, because the reference enters the document at one call site and
    a wrong argument there is silent (every node is merely offset, and the
    field's *shape* still looks right)."""
    x, y, results, first_home = _capture_with_home()
    anchors = [_anchor(0.0, first_home[0], first_home[1]),
               _anchor(130.0, first_home[0] + 0.0006, first_home[1] + 0.0018)]
    mean = tcg.anchor_mean_rad(anchors)
    doc = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8, mean)
    hx, hy = tcg.home_index(x, y)
    # The home node ships its own departure from the mean — NOT zero — and
    # that departure is exactly half the anchor gap here.
    assert doc['residual_rad']['ty'][hy][hx] == pytest.approx(-0.0009, abs=1e-9)
    parsed = tilt_map.parse_tilt_map(doc)
    tx, ty = tilt_map.lookup(parsed, x[0], y[0])
    assert ty == pytest.approx(results[(0, 0)].res_ty - mean[1], abs=1e-9)


# ── CSV schema ───────────────────────────────────────────────────────────


def test_csv_columns_are_stable():
    """The analyser reads these names; changing one silently breaks
    ``tilt_cal_analyse.py --csv`` on every historical capture."""
    assert tcg.CSV_COLUMNS == (
        'iso', 't_s', 'phase', 'visit', 'ix', 'iy',
        'x_mm', 'y_mm', 'z_mm', 'read_index',
        'raw_tx_rad', 'raw_ty_rad', 'res_tx_rad', 'res_ty_rad',
        'ok', 'uptime_ms', 'note',
    )


def test_csv_row_keys_match_the_declared_columns_exactly():
    row = tcg.csv_row('2026-08-03T00:00:00', 1.0, 'capture', 0, 1, 2,
                      10.0, 20.0, 170.0, 3, 0.01, 0.02, OFFSET, 1234)
    assert set(row) == set(tcg.CSV_COLUMNS)


def test_csv_row_applies_the_mounting_offset_to_the_res_columns():
    row = tcg.csv_row('t', 0.0, 'capture', 0, 0, 0, 0.0, 0.0, 170.0, 0,
                      0.01, 0.02, OFFSET, None)
    assert row['ok'] == 1
    assert row['res_tx_rad'] == pytest.approx(0.01 + OFFSET[0])
    assert row['res_ty_rad'] == pytest.approx(0.02 + OFFSET[1])
    assert row['uptime_ms'] == ''


def test_csv_row_marks_a_failed_read_and_does_not_launder_nan():
    nan = float('nan')
    row = tcg.csv_row('t', 0.0, 'capture', 0, 0, 0, 0.0, 0.0, 170.0, 0,
                      nan, nan, OFFSET, 55)
    assert row['ok'] == 0
    assert math.isnan(row['res_tx_rad'])
    assert math.isnan(row['res_ty_rad'])


def test_verification_rows_are_marked_off_grid():
    row = tcg.csv_row('t', 0.0, 'verify', 2, -1, -1, 37.5, 112.5, 170.0, 0,
                      0.0, 0.0, OFFSET, 1)
    assert (row['ix'], row['iy']) == (-1, -1)
    assert row['phase'] == 'verify'


def test_mid_sweep_anchor_rows_carry_their_own_phase_tag():
    """A mid-sweep home anchor sits at the home node's coordinates and index,
    so ONLY the phase tag distinguishes it from the home grid visit. Without a
    distinct tag the analyser would average a re-arrival into the home node's
    residual — folding in exactly the arrival-to-arrival variation the anchors
    exist to expose separately.
    """
    assert tcg.PHASE_HOME_ANCHOR == 'home_anchor'
    assert tcg.PHASE_HOME_END == 'home_end'
    assert tcg.PHASE_HOME_ANCHOR != 'capture'
    row = tcg.csv_row('t', 61.2, tcg.PHASE_HOME_ANCHOR, 4, 2, 2,
                      0.0, 0.0, 170.0, 0, 0.001, 0.002, OFFSET, 99)
    assert row['phase'] == 'home_anchor'
    assert (row['x_mm'], row['y_mm']) == (0.0, 0.0)
    assert row['t_s'] == pytest.approx(61.2)


def test_the_tool_tags_its_anchor_rows_with_the_phase_constants():
    """Structural: ``run()`` needs a robot, so pin that both anchor record
    sites pass the CONSTANTS rather than an inline string literal that could
    drift away from the analyser's phase filter — a drift whose only symptom
    would be a re-arrival quietly averaged into the home node's residual.
    """
    source = open(tcg.__file__.replace('.pyc', '.py')).read()
    calls = source.count('measure_home_anchor(')
    assert calls == 3, (
        'expected the definition plus the mid-sweep and end-of-sweep calls, '
        'found {}'.format(calls))
    assert 'PHASE_HOME_ANCHOR,' in source
    assert 'PHASE_HOME_END,' in source
    # ...and no bare literal at a record site.
    assert "record('home_anchor'" not in source
    assert "record('home_end'" not in source


# ── write target ─────────────────────────────────────────────────────────


def test_write_target_honours_the_authoritative_env_override(monkeypatch):
    """When $JUGGLEBOT_TILT_CAL is set it is the ONLY candidate, so the tool
    must write there — writing anywhere else would apply a different
    calibration than the operator named while reporting success."""
    monkeypatch.setenv(tilt_map.TILT_MAP_ENV, '/tmp/explicit_tilt_cal.yaml')
    assert tcg.write_target_path() == '/tmp/explicit_tilt_cal.yaml'


def test_write_target_is_the_first_production_candidate(monkeypatch):
    monkeypatch.delenv(tilt_map.TILT_MAP_ENV, raising=False)
    candidates = tilt_map.tilt_map_candidates()
    if not candidates:
        pytest.skip('no candidate paths in this environment')
    assert tcg.write_target_path() == candidates[0]


def test_source_tree_detection_distinguishes_a_share_tree():
    assert tcg.looks_like_source_tree(
        '/home/x/Jugglebot/config/tilt_calibration.yaml') is False
    assert tcg.looks_like_source_tree(
        '/opt/ros/install/jugglebot/share/jugglebot/config/'
        'tilt_calibration.yaml') is False


def test_estimate_duration_includes_the_three_non_grid_visits():
    """25 nodes + 6 checks + end anchor + verification home reference +
    return to centre = 34 visit-equivalents, before any mid-sweep anchor."""
    per_visit = 3.0 + 1.0 + 2 * (tcg.EST_READ_S + 0.15)
    assert tcg.estimate_duration_s(25, 6, 3.0, 1.0, 2, 0.15) == \
        pytest.approx(34 * per_visit)


def test_estimate_duration_counts_the_mid_sweep_home_anchors():
    """Each mid-sweep anchor is a FULL visit (move + dwell + reads). On the
    5x5 default that is five of them; an ETA that omitted them would
    understate a ~6 min capture by ~45 s at the C0 working point."""
    per_visit = 3.0 + 2.0 + 30 * (tcg.EST_READ_S + 0.15)
    revisits = tcg.home_revisit_after_visits(25, 4)
    assert len(revisits) == 5
    with_anchors = tcg.estimate_duration_s(25, 6, 3.0, 2.0, 30, 0.15,
                                           len(revisits))
    assert with_anchors == pytest.approx(39 * per_visit)
    assert with_anchors - tcg.estimate_duration_s(
        25, 6, 3.0, 2.0, 30, 0.15) == pytest.approx(5 * per_visit)


# ── CLI wiring ───────────────────────────────────────────────────────────


def test_defaults_match_the_documented_capture():
    args = tcg.build_parser().parse_args([])
    assert args.box == 150.0 and args.nodes == 5 and args.z == 170.0
    assert args.lean_gain == -1.0, (
        'lean must DEFER TO NODE CONFIG (ships 0.6) by default: the lean '
        'window is zero at both endpoints so the terminal pose is unaffected, '
        'and the shaped transit is the precedent-proven working point (all '
        'four ±150 corners held 2026-07-27 at lean 0.6). An explicit 0.0 '
        'would force lean OFF per GoToPose.srv.')
    assert args.on_fail == 'continue'
    assert args.check_poses == 6
    assert args.threshold_deg == pytest.approx(0.15)
    assert args.read_gap_s > 0.1, (
        'reads must be spaced beyond the SCL3300 10 Hz LPF period or the '
        'per-read sd measures the filter rather than the sensor')


def test_apply_is_the_default_and_no_apply_serves_rung_c0():
    assert tcg.build_parser().parse_args([]).no_apply is False
    assert tcg.build_parser().parse_args(['--no-apply']).no_apply is True


def test_verify_only_and_no_apply_are_refused_together(capsys):
    assert tcg.main(['--verify-only', '--no-apply']) == 2
    assert 'contradictory' in capsys.readouterr().err


def test_resolve_grid_prefers_explicit_lists_over_the_box_form():
    args = tcg.build_parser().parse_args(
        ['--x=-50,0,50', '--y=-60,0,60', '--box', '999'])
    x, y = tcg.resolve_grid(args)
    assert x == [-50.0, 0.0, 50.0]
    assert y == [-60.0, 0.0, 60.0]


def test_negative_node_list_without_the_equals_form_explains_itself(capsys):
    """``--x -150,...`` cannot work (argparse reads the leading '-' as an
    option) and its native error says only "expected one argument". This is the
    tool's own documented usage form, so the fix is in the message."""
    with pytest.raises(SystemExit):
        tcg.build_parser().parse_args(['--x', '-150,-75,0,75,150'])
    assert '--x=-150' in capsys.readouterr().err


def test_resolve_grid_refuses_an_even_node_count():
    args = tcg.build_parser().parse_args(['--nodes', '4'])
    with pytest.raises(tcg.TiltCalError, match='home node'):
        tcg.resolve_grid(args)


def test_return_to_centre_guard_catches_base_exception():
    """The return-to-centre guard must catch ``BaseException``, not ``Exception``.

    Structural, because ``run()`` cannot be exercised without a robot — and this
    is exactly where the one blocking defect of this phase lived. A plain
    ``except Exception`` does **not** catch ``KeyboardInterrupt``, and the second
    Ctrl-C is the reflex when a program does not die on the first. The
    return-to-centre blocks for up to ``--timeout-s`` plus the move, so that
    second interrupt lands inside the guard: with ``Exception`` it propagated out
    of the ``finally``, skipping the artefact writes and the rclpy shutdown, and
    left the platform parked at a raised displaced pose **with no message** —
    silently contradicting the one absolute safety claim the tool and its runbook
    both make.

    This test exists to stop that being "tidied" back to ``except Exception``.
    """
    source = open(tcg.__file__.replace('.pyc', '.py')).read()
    marker = 'RETURN TO CENTRE FAILED'
    assert marker in source
    # rindex, not index: the module docstring quotes this operator-facing string
    # when explaining the one case the return cannot honour (a wire disarmed
    # mid-sweep). The occurrence that matters is the `print` in the guard, which
    # is the LAST one — anchoring on the first made the test fail on a prose
    # mention rather than on the handler it exists to pin.
    guard = source[:source.rindex(marker)]
    handler = guard.rsplit('except ', 1)[1].split('\n')[0]
    assert handler.startswith('BaseException'), (
        'the return-to-centre handler must catch BaseException so a second '
        'Ctrl-C cannot skip it silently; found "except {}"'.format(handler))


def test_dry_run_makes_no_ros_calls(capsys, monkeypatch):
    """--dry-run must construct nothing: it is the rehearsal an operator runs
    on a machine that is not even powered."""
    def _explode(*_args, **_kwargs):
        raise AssertionError('dry-run imported/used rclpy')

    monkeypatch.setitem(sys.modules, 'rclpy', _explode)
    assert tcg.main(['--dry-run']) == 0
    out = capsys.readouterr().out
    assert 'home node (home reference' in out
    assert 'IK stroke preflight OK' in out
    assert 'no ROS calls made' in out
    assert 'ETA' in out


# ── the disarmed-wire guard ──────────────────────────────────────────────
#
# The BLOCKING class this section pins: `trajectory/go_to_pose` is ACCEPTED
# while the wire is disarmed (streaming-while-disarmed is the legal pre-arm
# phase of the ARMING CONTRACT), the platform does not move, every node
# re-measures one stationary pose, and the tool writes + applies + "verifies" an
# all-zeros calibration that verifies perfectly BECAUSE nothing moved. Precedent
# for the silent-no-op shape: the 2026-07-15 ramp battery.


def test_disarmed_marker_is_detected_in_an_accepted_response():
    """The real suffix `trajectory_node._wire_state_suffix` appends."""
    accepted = ('move accepted: target (x=150.0 y=0.0 z=170.0 mm) over 3.000s '
                '(lean_gain=0.00) [wire DISARMED — setpoints not reaching the '
                'legs]')
    assert tcg.response_reports_disarmed(accepted) is True


def test_a_normal_accepted_response_is_not_flagged():
    """No false positives, or the first operator to hit one disables the check."""
    accepted = ('move accepted: target (x=150.0 y=0.0 z=170.0 mm) over 3.000s '
                '(lean_gain=0.00)')
    assert tcg.response_reports_disarmed(accepted) is False
    assert tcg.response_reports_disarmed('') is False
    assert tcg.response_reports_disarmed(None) is False


def test_disarmed_detection_is_case_insensitive():
    """Keyed on the WORD, not the wording: the sentence around it is a log
    string that may be reworded, the marker itself is the contract."""
    assert tcg.response_reports_disarmed('accepted [wire disarmed]') is True


def _kv(**overrides):
    """A /link_status key-value mapping in the encodings the bridge publishes.

    Verified against `teensy_bridge_node._publish_link_status`, not assumed:
    `mpc_active` is `str(int(bool))`, `fault_state` is a FaultState enum NAME,
    `bridge_link` is 'UP'/'LOST'/'NO_HEARTBEAT'.
    """
    kv = {'mpc_active': '1', 'fault_state': 'NONE', 'bridge_link': 'UP',
          'uptime_ms': '12345'}
    kv.update(overrides)
    return kv


def test_wire_verdict_passes_on_an_armed_link():
    ok, message = tcg.wire_armed_verdict(_kv())
    assert ok is True
    assert 'ARMED' in message


@pytest.mark.parametrize('overrides,needle', [
    ({'mpc_active': '0'}, 'DISARMED'),
    ({'fault_state': 'MAX_DEVIATION'}, 'MAX_DEVIATION'),
    # 'UNKNOWN' is what the bridge publishes when it has seen NO heartbeat —
    # not a clear guard, and it must not read as one.
    ({'fault_state': 'UNKNOWN'}, 'UNKNOWN'),
    ({'bridge_link': 'LOST'}, 'bridge_link=LOST'),
    ({'bridge_link': 'NO_HEARTBEAT'}, 'NO_HEARTBEAT'),
])
def test_wire_verdict_refuses_every_unsafe_link_state(overrides, needle):
    ok, message = tcg.wire_armed_verdict(_kv(**overrides))
    assert ok is False
    assert needle in message


def test_wire_verdict_refuses_an_empty_or_partial_cache():
    """An absent key is a FAILURE, never a pass.

    This runs against a cache that is empty until the first message arrives.
    Defaulting to "armed" there would reintroduce the exact hole it closes —
    a preflight that passes because nothing has been heard yet.
    """
    ok, message = tcg.wire_armed_verdict({})
    assert ok is False and 'no /link_status' in message
    ok, message = tcg.wire_armed_verdict(None)
    assert ok is False

    # ALL THREE keys are required, not just mpc_active. A bridge that publishes
    # mpc_active publishes the other two in the same message, so a missing
    # companion means this is not the publisher the tool thinks it is reading —
    # and falling through on it would silently skip a real refusal (a latched
    # guard fault, or a downed UDP link).
    for key in ('mpc_active', 'fault_state', 'bridge_link'):
        partial = _kv()
        del partial[key]
        ok, message = tcg.wire_armed_verdict(partial)
        assert ok is False, '{} must be required'.format(key)
        assert key in message


# ── temporal health: the post-FW-14 replacement for the fresh-boot rule ──
#
# The rule these tests pin: on FW 14+ the question is "is the plant HEALTHY"
# (`latency_monitor`), on anything older — or anything unreadable — it is still
# "is the bridge young" (`uptime_ms`), because a pre-FW-14 board genuinely IS
# uptime-sensitive. `logbook/2026-08-15-fw14-validated-arc-closed.md`.


def _kv14(**overrides):
    """A /link_status mapping from a healthy FW 14 bridge."""
    kv = dict(bridge_fw_version='14 (proto 5)', latency_monitor='OK')
    kv.update(overrides)
    return _kv(**kv)


def test_bridge_fw_version_parses_every_shape_the_bridge_renders():
    """Three renderings, and only two of them carry a number.

    `_bridge_fw_version_str` emits `'14 (proto 5)'`, a SKEW form that still
    leads with the version, and `'unknown (never seen)'` when no BRIDGE_IDENTITY
    frame has arrived. The last one, and an absent key, must both come back as
    None — "I cannot tell" must never read as "current", because every caller
    turns None into "keep the old uptime protection".
    """
    assert tcg.bridge_fw_version_value({'bridge_fw_version': '14 (proto 5)'}) == 14
    assert tcg.bridge_fw_version_value(
        {'bridge_fw_version': '13 (SKEW — expected v14, proto 5)'}) == 13
    assert tcg.bridge_fw_version_value(
        {'bridge_fw_version': 'unknown (never seen)'}) is None
    assert tcg.bridge_fw_version_value({}) is None
    assert tcg.bridge_fw_version_value(None) is None


def test_temporal_health_passes_a_warm_fw14_bridge_on_the_monitor():
    """THE landmine this rework removes.

    A 15.2 h FW 14 bridge measured 20 ms of end-to-end lag — the same number a
    fresh one measures — so refusing it costs a sitting and protects nothing.
    """
    warm = 15 * 3600 * 1000
    verdict = tcg.temporal_health_verdict(_kv14(), warm)
    assert verdict.ok is True
    assert verdict.basis == 'health'
    assert 'latency_monitor=OK' in verdict.message


@pytest.mark.parametrize('token', ['RING_LEAK', 'CACHE_AGE', 'CLAMP_DUTY'])
def test_temporal_health_refuses_every_alarm_even_on_a_freshly_booted_bridge(token):
    """An alarm outranks youth. A 1-minute-old bridge that is ALREADY leaking is
    a broken plant, and the old gate would have waved it straight through."""
    verdict = tcg.temporal_health_verdict(
        _kv14(latency_monitor=token), 60 * 1000)
    assert verdict.ok is False
    assert verdict.basis == 'health'
    assert token in verdict.message


def test_temporal_health_keeps_the_uptime_refusal_for_pre_fw14_boards():
    """The real protection is PRESERVED where it is still real.

    Below FW 14 the RX ring still leaks ~40 ms/h, so the board is genuinely
    uptime-sensitive and the power-cycle is genuinely the remedy.
    """
    kv = _kv(bridge_fw_version='13 (SKEW — expected v14, proto 5)',
             latency_monitor='OK')
    warm = tcg.temporal_health_verdict(kv, 31 * 60 * 1000)
    assert warm.ok is False
    assert warm.basis == 'uptime'
    assert 'Power-cycle' in warm.message
    assert tcg.temporal_health_verdict(kv, 29 * 60 * 1000).ok is True


def test_temporal_health_falls_back_to_uptime_when_the_firmware_is_unknowable():
    """`'unknown (never seen)'` and an absent row are pre-FW-14 for our purposes.

    A bridge that sends no BRIDGE_IDENTITY frame is older than FW 9 or absent
    entirely; either way the tool cannot certify FW 14, so it must not act as
    though it had.
    """
    for kv in (_kv(bridge_fw_version='unknown (never seen)'), _kv()):
        verdict = tcg.temporal_health_verdict(kv, 31 * 60 * 1000)
        assert verdict.ok is False
        assert verdict.basis == 'uptime'


def test_temporal_health_falls_back_when_a_stale_ros_build_omits_the_monitor():
    """Current bridge, old NODE. The health read is unavailable through no fault
    of the firmware, so the old ceiling governs and the message says to rebuild
    — silently passing a plant you cannot see is the failure this whole arc was."""
    kv = _kv(bridge_fw_version='14 (proto 5)')
    verdict = tcg.temporal_health_verdict(kv, 31 * 60 * 1000)
    assert verdict.ok is False
    assert verdict.basis == 'uptime'
    assert 'Rebuild and relaunch' in verdict.message
    # ...and it still PASSES a young bridge, so a stale build is not a hard stop.
    assert tcg.temporal_health_verdict(kv, 60 * 1000).ok is True


def test_temporal_health_refuses_when_there_is_no_link_status_at_all():
    """Absence is not health — the same argument `wire_armed_verdict` makes."""
    for kv in ({}, None):
        verdict = tcg.temporal_health_verdict(kv, 60 * 1000)
        assert verdict.ok is False
        assert verdict.basis == 'no_link'


def test_temporal_health_refuses_a_pre_fw14_board_with_no_uptime_either():
    assert tcg.temporal_health_verdict(_kv(uptime_ms=None), None).ok is False


def test_worse_latency_monitor_follows_causal_precedence():
    """RING_LEAK > CACHE_AGE > CLAMP_DUTY > OK, and an unknown token beats them
    all — an unrecognised value must never be reduced away as 'fine'."""
    assert tcg.worse_latency_monitor('OK', 'CLAMP_DUTY') == 'CLAMP_DUTY'
    assert tcg.worse_latency_monitor('CLAMP_DUTY', 'CACHE_AGE') == 'CACHE_AGE'
    assert tcg.worse_latency_monitor('CACHE_AGE', 'RING_LEAK') == 'RING_LEAK'
    assert tcg.worse_latency_monitor('RING_LEAK', 'OK') == 'RING_LEAK'
    assert tcg.worse_latency_monitor('OK', 'WAT') == 'WAT'
    assert tcg.worse_latency_monitor(None, 'OK') == 'OK'
    assert tcg.worse_latency_monitor('OK', None) == 'OK'
    assert tcg.worse_latency_monitor(None, None) is None


def test_latency_monitor_tokens_match_the_producer():
    """XREF against `teensy_bridge_node`, which OWNS these strings.

    They are restated here rather than imported because these harnesses must
    import without `rclpy`. That is the drift risk this test closes: a rename in
    the producer silently turns every token comparison in the harnesses into a
    permanent "unknown token", which fails CLOSED but for the wrong reason.
    Source text, not an import, for the same no-rclpy reason.
    """
    node = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__)))),
        'ros_ws', 'src', 'jugglebot', 'jugglebot', 'teensy_bridge_node.py')
    source = open(node).read()
    assert "LATENCY_MONITOR_OK = '{}'".format(tcg.LATENCY_MONITOR_OK) in source
    for token in tcg.LATENCY_MONITOR_ALARMS:
        assert "LATENCY_MONITOR_{} = '{}'".format(token, token) in source, \
            '{} is no longer the producer\'s token'.format(token)
    # The rank order the worst-seen fold depends on, as the producer declares it.
    assert 'LATENCY_MONITOR_CLAMP_DUTY: 1' in source
    assert 'LATENCY_MONITOR_CACHE_AGE: 2' in source
    assert 'LATENCY_MONITOR_RING_LEAK: 3' in source


def _stats(res_tx, res_ty, n_ok=8, n_total=8, sd=1e-6):
    """A ReadStats with the raw means filled in consistently.

    `raw_mean_*` are the un-offset sensor means; the flat-field check reads only
    `res_*` and `n_ok`, but constructing a NamedTuple partially would drift the
    moment a field is added.
    """
    return tcg.ReadStats(res_tx=res_tx, res_ty=res_ty, sd_tx=sd, sd_ty=sd,
                         raw_mean_tx=res_tx - OFFSET[0],
                         raw_mean_ty=res_ty - OFFSET[1],
                         n_ok=n_ok, n_total=n_total)


def test_flat_field_is_detected_as_the_disarmed_signature():
    """Every node the same number ⇒ the platform never moved."""
    results = {(ix, iy): _stats(0.0021, -0.0013)
               for ix in range(3) for iy in range(3)}
    is_flat, message = tcg.flat_field_verdict(results)
    assert is_flat is True
    assert 'FLAT FIELD' in message
    assert 'NEVER MOVED' in message


def test_a_real_field_is_not_flagged_flat():
    """A real field spans ~1e-2 rad across the workspace — two orders above the
    tolerance — so the check must never fire on a good capture."""
    results = {}
    for ix in range(3):
        for iy in range(3):
            results[(ix, iy)] = _stats(0.001 * ix, -0.001 * iy)
    is_flat, _ = tcg.flat_field_verdict(results)
    assert is_flat is False


def test_flat_field_is_undefined_below_two_usable_nodes():
    """One node cannot be 'flat'; an unusable node must not be counted."""
    one = {(0, 0): _stats(0.0, 0.0)}
    assert tcg.flat_field_verdict(one)[0] is False
    plus_dead = dict(one)
    plus_dead[(1, 0)] = _stats(0.0, 0.0, n_ok=0)
    assert tcg.flat_field_verdict(plus_dead)[0] is False


def test_the_wire_check_runs_before_the_per_node_try_block():
    """Structural: `assert_wire_armed` must sit OUTSIDE the per-node try.

    `run()` needs a robot, so this is checked in the source. The placement is
    the whole point: the try below converts a `TiltCalError` into a failed
    *node* and, under the default `--on-fail continue`, carries on. A wire that
    disarmed mid-sweep is never "this one node failed" — every remaining node
    would re-measure the same stationary pose — so demoting it there would
    reinstate the bug this guard exists to close.
    """
    source = open(tcg.__file__.replace('.pyc', '.py')).read()
    marker = 'stats, reads = runner.measure(nd.x_mm, nd.y_mm, z_mm, label,'
    assert marker in source
    prologue = source[:source.index(marker)]
    tail = prologue.rsplit('runner.assert_wire_armed(', 1)[1]
    assert tail.count('try:') == 1, (
        'assert_wire_armed must be called before the per-node `try:`, so a '
        'mid-sweep disarm aborts the run instead of scoring one bad node')


def test_fault_errors_escape_both_demotion_handlers():
    """Structural: `except TiltCalFaultError: raise` must precede the
    `except TiltCalError` demotion in BOTH the capture loop and the verify
    loop. Without the re-raise, a latched guard fault raised INSIDE measure()
    (arrival/per-read cached checks, the go_to DISARMED marker) is demoted to
    a failed node under the default --on-fail continue — which is exactly how
    30 reads ran against a collapsed platform on 2026-08-09.
    """
    source = open(tcg.__file__.replace('.pyc', '.py')).read()
    markers = (
        'stats, reads = runner.measure(nd.x_mm, nd.y_mm, z_mm, label,',
        'stats, reads = runner.measure(cx, cy, z_mm, label, offset_rad)',
    )
    for marker in markers:
        assert marker in source, marker
        after = source[source.index(marker):]
        fault_pos = after.find('except TiltCalFaultError')
        demote_pos = after.find('except TiltCalError as exc:')
        assert fault_pos != -1 and demote_pos != -1, marker
        assert fault_pos < demote_pos, (
            'the TiltCalFaultError re-raise must precede the TiltCalError '
            'demotion handler after `{}`'.format(marker))


def test_go_to_checks_the_response_for_the_disarmed_marker():
    """Structural: every accepted `go_to_pose` is screened, in `go_to` itself.

    Screening at the single choke point rather than per call site is what makes
    the guarantee total — capture nodes, verification poses and the
    return-to-centre all route through this one method.
    """
    source = open(tcg.__file__.replace('.pyc', '.py')).read()
    body = source.split('def go_to(', 1)[1].split('\n    def ', 1)[0]
    assert 'response_reports_disarmed(response.message)' in body


# ── --force-uninstall: the ament-share shadow ────────────────────────────


def test_uninstall_plan_moves_every_existing_candidate(monkeypatch, tmp_path):
    """Source tree AND ament share — moving only the source file is a no-op.

    `setup.py` installs `config/tilt_calibration.yaml` into
    `share/jugglebot/config/` whenever it exists at build time, and the resolver
    order is env -> source tree -> share. So after ANY colcon build with the
    file present, removing just the source copy falls through to the share copy:
    the reload succeeds, the OLD map stays loaded, and the capture is refused
    with a message naming neither the cause nor the remedy.
    """
    src = tmp_path / 'src' / 'tilt_calibration.yaml'
    share = tmp_path / 'share' / 'tilt_calibration.yaml'
    absent = tmp_path / 'gone' / 'tilt_calibration.yaml'
    for path in (src, share):
        path.parent.mkdir(parents=True)
        path.write_text('x')

    monkeypatch.delenv(tilt_map.TILT_MAP_ENV, raising=False)
    monkeypatch.setattr(tilt_map, 'tilt_map_candidates',
                        lambda environ=None: (str(src), str(share),
                                              str(absent)))
    to_move, env_override = tcg.uninstall_plan()
    assert env_override is None
    assert to_move == [str(src), str(share)], \
        'every EXISTING candidate must be moved aside, not just the first'


def test_uninstall_plan_refuses_to_touch_an_env_pointed_file(monkeypatch,
                                                            tmp_path):
    """An operator's `$JUGGLEBOT_TILT_CAL` file is never renamed behind them.

    When the override is set it is the ONLY candidate, and it names a file the
    operator chose — possibly outside the repo, possibly the artefact they are
    protecting. The caller refuses with instructions instead.
    """
    named = tmp_path / 'my_calibration.yaml'
    named.write_text('x')
    monkeypatch.setenv(tilt_map.TILT_MAP_ENV, str(named))
    to_move, env_override = tcg.uninstall_plan()
    assert to_move == []
    assert env_override == str(named)
    assert named.exists()
