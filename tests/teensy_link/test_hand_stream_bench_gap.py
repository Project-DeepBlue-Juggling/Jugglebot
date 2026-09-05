"""Offline cover for the ``moving_gap`` stage of ``hand_stream_bench.py``.

The stage exercises FW 17's NORMATIVE falling-edge rule: when ``HAS_HAND``
falls while leg frames keep flowing, the hand lane runs its Hermite to the u1
endpoint, Taylor-extrapolates for ``MAX_EXTRAP_DT_S`` and then DECAYS the
velocity to zero over ``EXTRAP_DECAY_DT_S``, all off its OWN knot clock
(``leg_interp.cpp`` :701-748). Everything the bench concludes about that rule
is arithmetic on three firmware constants and a synthetic knot schedule — so
all of it is testable here, with no hardware and no link.

WHY THIS FILE EXISTS AT ALL. On 2026-09-04 the ``gap`` stage streamed a
constant Hold while its CSV logged a 1.0 rev re-entry that never happened: the
override was computed, consumed by the CSV and by the deviation belt, and never
passed to the frame builder. It failed silently because the phantom's deviation
sat *under* the stage's own belt, and it reported a plausible number while
measuring nothing (`logbook/2026-09-04-fw17-hand-ladder-sitting-two.md`
§ "The gap stage measured nothing"). That is stage LOGIC, and there was no
offline test of any of it. These are the tests that class of defect needs.

Lives in ``tests/teensy_link/`` beside the other Setpoint-source covers
(``test_replay_setpoint.py``, ``test_synthetic_setpoint.py``): the driver is a
Setpoint source and the property under test is a wire-level presence rule
(HAS_HAND falling and rising). The driver itself lives in ``tests/hardware/``,
which ``--ignore=tests/hardware`` keeps out of collection, so it goes on
``sys.path`` explicitly — the ``tests/motion/test_bench_sysid_logic.py`` idiom.
"""
from __future__ import annotations

import os
import re
import sys

import pytest

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_HW_DIR = os.path.join(_REPO, 'tests', 'hardware')
if _HW_DIR not in sys.path:
    sys.path.insert(0, _HW_DIR)

import hand_stream_bench as B  # noqa: E402

_FW_HEADER = os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot',
                          'Teensy_code_canbridge', 'canbridge_config.h')


def _fw_constants():
    with open(_FW_HEADER, 'r', encoding='utf-8') as fh:
        src = fh.read()
    return src


def _fw_float(src, name):
    m = re.search(r'constexpr\s+float\s+' + name + r'\s*=\s*([0-9.]+)f', src)
    assert m, f'{name} no longer defined as a float literal in canbridge_config.h'
    return float(m.group(1))


# ---------------------------------------------------------------------------
# The mirrored firmware constants — the whole verdict is arithmetic on these
# ---------------------------------------------------------------------------

def test_extrapolation_constants_match_the_firmware():
    """The driver mirrors MAX_EXTRAP_DT_S / EXTRAP_DECAY_DT_S by hand.

    They are not in the generated config — ``canbridge_config.h`` is their only
    source — and the moving_gap stage's coast prediction, its wind-down window
    and its default gap length are all derived from them. A firmware edit that
    silently changed either would leave the bench measuring against a stale
    model and reporting a confident wrong verdict, which is precisely the
    failure mode row 17 already shipped once.
    """
    src = _fw_constants()
    assert B.MAX_EXTRAP_DT_S == _fw_float(src, 'MAX_EXTRAP_DT_S')
    assert B.EXTRAP_DECAY_DT_S == _fw_float(src, 'EXTRAP_DECAY_DT_S')


def test_seg_t_matches_a_firmware_segment_span():
    """SEG_T is the knot cadence the hand lane's Mode 1 runs over.

    The header defines SEGMENT_T_S twice behind ``UNIFIED7_BENCH_BUILD``
    (0.010 for the retired 100 Hz probe variant, 0.025 for the shipped build),
    so this pins that the driver's value is one the firmware actually knows —
    and specifically the production one.
    """
    src = _fw_constants()
    vals = [float(v) for v in
            re.findall(r'constexpr\s+float\s+SEGMENT_T_S\s*=\s*([0-9.]+)f', src)]
    assert vals, 'SEGMENT_T_S no longer defined in canbridge_config.h'
    assert B.SEG_T == 0.025
    assert B.SEG_T in vals


def test_hand_guard_constants_bound_the_reentry_bar():
    """The re-entry bar must fire before any firmware guard can.

    Root cause it matters: the stage runs OBSERVE-FIRST, so
    MAX_DEVIATION_HAND_REV only counts an exceed tick — it does not E-STOP —
    and MAX_LEAD_HAND_REV silently absorbs anything under it. Nothing
    downstream refuses a re-entry step the driver got wrong, so the driver's
    own bar is the only refusal, and it is meaningless unless it is the
    tightest number in the chain.
    """
    src = _fw_constants()
    lead = _fw_float(src, 'MAX_LEAD_HAND_REV')
    dev = _fw_float(src, 'MAX_DEVIATION_HAND_REV')
    assert B.MG_REENTRY_BAR_REV < lead < dev
    # ... and under the pump's own hand step gate (200 rev/s x 25 ms = 5.0 rev)
    # and the reference layer's 0.80 margin below it (4.0 rev).
    from teensy_link.setpoint_pump import DEFAULT_MAX_STEP_HAND_REV
    assert B.MG_REENTRY_BAR_REV < 0.80 * DEFAULT_MAX_STEP_HAND_REV


def test_winddown_and_coast_are_the_documented_sums():
    assert B.HAND_WINDDOWN_S == pytest.approx(
        B.SEG_T + B.MAX_EXTRAP_DT_S + B.EXTRAP_DECAY_DT_S)
    assert B.HAND_COAST_S == pytest.approx(
        B.SEG_T + B.MAX_EXTRAP_DT_S + 0.5 * B.EXTRAP_DECAY_DT_S)
    assert B.HAND_WINDDOWN_S == pytest.approx(0.135)
    assert B.HAND_COAST_S == pytest.approx(0.105)


# ---------------------------------------------------------------------------
# hand_gap_target — the port of the firmware's Mode-1/2/3 ladder
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('v', [0.5, 2.0, -0.5, -3.0])
def test_mode1_is_exactly_linear(v):
    """With p1 = p0 + v.T and v0 == v1 the Hermite basis sums to s exactly.

    This is why the stage's prediction is exact rather than a fit: a
    constant-velocity ramp makes every cubic term vanish.
    """
    for g in (0.0, 0.001, 0.010, 0.020, B.SEG_T):
        pos, vel = B.hand_gap_target(1.0, v, g)
        assert pos == pytest.approx(1.0 + v * g, abs=1e-12)
        assert vel == pytest.approx(v, abs=1e-12)


@pytest.mark.parametrize('v', [0.5, 2.0, -0.5])
def test_ladder_is_continuous_at_every_mode_boundary(v):
    eps = 1e-9
    for g in (B.SEG_T, B.SEG_T + B.MAX_EXTRAP_DT_S, B.HAND_WINDDOWN_S):
        lo_p, lo_v = B.hand_gap_target(0.0, v, g - eps)
        hi_p, hi_v = B.hand_gap_target(0.0, v, g + eps)
        assert lo_p == pytest.approx(hi_p, abs=1e-7)
        assert lo_v == pytest.approx(hi_v, abs=1e-6)


@pytest.mark.parametrize('v', [0.5, 2.0, -0.5])
def test_velocity_decays_monotonically_to_exactly_zero(v):
    """The rule the firmware ships: never hold-at-last-command from up to
    200 rev/s. |vel| must be non-increasing and reach 0 at the wind-down."""
    prev = abs(v) + 1.0
    g = 0.0
    while g <= 0.30:
        _pos, vel = B.hand_gap_target(0.0, v, g)
        assert abs(vel) <= prev + 1e-12
        prev = abs(vel)
        g += 0.001
    assert B.hand_gap_target(0.0, v, B.HAND_WINDDOWN_S)[1] == pytest.approx(0.0, abs=1e-12)
    assert B.hand_gap_target(0.0, v, 5.0)[1] == 0.0


@pytest.mark.parametrize('v', [0.5, 2.0, -0.5, -3.0])
def test_total_coast_is_v_times_hand_coast_s_and_then_frozen(v):
    """THE prediction row 17b tests: the lane's own target advances exactly
    v x 0.105 rev past the last knot and then stops moving forever."""
    settled = B.hand_gap_target(0.0, v, B.HAND_WINDDOWN_S)[0]
    assert settled == pytest.approx(v * B.HAND_COAST_S, abs=1e-12)
    for g in (0.14, 0.25, 1.0, 60.0):
        assert B.hand_gap_target(0.0, v, g)[0] == pytest.approx(settled, abs=1e-12)


def test_no_decay_before_mode_3():
    """Below SEG_T + MAX_EXTRAP_DT_S (75 ms) the lane still runs at the knot
    velocity, so a gap that short cannot observe the decay at all — the
    justification for the >= 5 knot floor on --gap-knots."""
    v = 0.5
    for g in (0.030, 0.050, 0.074):
        pos, vel = B.hand_gap_target(0.0, v, g)
        assert vel == pytest.approx(v)
        assert pos == pytest.approx(v * g)


def test_g_at_or_below_zero_returns_the_knot_state():
    assert B.hand_gap_target(2.0, 0.5, 0.0) == (2.0, 0.5)
    assert B.hand_gap_target(2.0, 0.5, -1.0) == (2.0, 0.5)


# ---------------------------------------------------------------------------
# MovingGapPlan — which frames carry the hand channel
# ---------------------------------------------------------------------------

def test_default_schedule_indices_and_firmware_gap():
    plan = B.MovingGapPlan(2.0, 9)
    assert (plan.first, plan.last, plan.reentry) == (80, 88, 89)
    assert plan.fw_gap_s == pytest.approx(0.250)


def test_exactly_n_frames_are_withheld_and_the_reentry_carries_the_hand():
    plan = B.MovingGapPlan(2.0, 9)
    withheld = [i for i in range(200) if not plan.with_hand(i)]
    assert withheld == list(range(80, 89))
    assert len(withheld) == 9
    assert plan.with_hand(79) and plan.with_hand(89)
    assert plan.is_reentry(89)
    assert not plan.is_reentry(88) and not plan.is_reentry(90)


def test_the_shipped_default_actually_completes_the_wind_down():
    """The load-bearing property of the DEFAULT, not of the mechanism.

    9 withheld knots is a 250 ms firmware gap against a 135 ms wind-down, so
    the decay finishes with ~115 ms of frozen hold left to observe. A default
    that stopped short would make the stage unable to see the very rule it
    exists for — the row-17 failure, repeated with a moving hand.
    """
    plan = B.MovingGapPlan(2.0, 9)
    assert plan.fw_gap_s > B.HAND_WINDDOWN_S
    assert plan.fw_gap_s >= 1.8 * B.HAND_WINDDOWN_S
    # 5 knots is the floor; 4 leaves the lane mid-decay at the re-entry.
    assert B.MovingGapPlan(2.0, 5).fw_gap_s > B.HAND_WINDDOWN_S
    assert B.MovingGapPlan(2.0, 4).fw_gap_s <= B.HAND_WINDDOWN_S


def test_predicted_reentry_step_is_small_and_bounded():
    """v x (gap - coast) = 0.0725 rev = 2.29 mm at the shipped defaults."""
    plan = B.MovingGapPlan(2.0, 9)
    step = 0.5 * (plan.fw_gap_s - B.HAND_COAST_S)
    assert step == pytest.approx(0.0725)
    assert abs(step) < B.MG_REENTRY_BAR_REV
    assert abs(step) * B.MM_PER_REV < 3.0          # mm


def test_plan_refuses_a_degenerate_schedule():
    with pytest.raises(ValueError):
        B.MovingGapPlan(2.0, 0)
    with pytest.raises(ValueError):
        B.MovingGapPlan(2.0, -1)
    with pytest.raises(ValueError):
        # No hand-bearing frame ahead of the gap ⇒ the lane never latches and
        # there is nothing to decay.
        B.MovingGapPlan(0.0, 9)


# ---------------------------------------------------------------------------
# evaluate_moving_gap — the pass criteria, on synthetic traces
# ---------------------------------------------------------------------------

P0, V0 = 1.0, 0.5
PLAN = B.MovingGapPlan(2.0, 9)


def _trace(echo_fn, enc_offset=0.03, n=None):
    """One tuple per withheld tick: (g, echo, enc_ex, model)."""
    n = PLAN.n if n is None else n
    rows = []
    for k in range(1, n + 1):
        g = k * B.SEG_T
        model = B.hand_gap_target(P0, V0, g)[0]
        echo = echo_fn(g)
        rows.append((g, echo, echo - enc_offset, model))
    return rows


def _verdicts(rows, **kw):
    kw.setdefault('reentry_step_rev', 0.0725)
    kw.setdefault('lead_bit_ticks', 0)
    kw.setdefault('total_ticks', 400)
    res = B.evaluate_moving_gap(rows, P0, V0, PLAN.fw_gap_s,
                                kw.pop('reentry_step_rev'),
                                kw.pop('lead_bit_ticks'),
                                kw.pop('total_ticks'), **kw)
    return {c['id']: c for c in res}


def test_a_conforming_lane_passes_every_criterion():
    v = _verdicts(_trace(lambda g: B.hand_gap_target(P0, V0, g)[0]))
    assert [v[k]['verdict'] for k in ('G1', 'G2', 'G3', 'G4', 'G5')] == ['PASS'] * 5
    assert 'DECAY' in v['G1']['detail']


def test_no_wind_down_is_caught_and_NAMED():
    """The lane keeps extrapolating at the knot velocity — the failure mode
    the falling-edge rule was written to forbid, in its loudest form."""
    v = _verdicts(_trace(lambda g: P0 + V0 * g))
    assert v['G1']['verdict'] == 'FAIL'
    assert 'NO WIND-DOWN' in v['G1']['detail']
    assert v['G2']['verdict'] == 'FAIL'          # the target never froze


def test_hold_at_endpoint_is_caught_and_NAMED():
    """The other forbidden mode: Mode 1 pinned at s = 1 forever, still
    commanding the endpoint with vel_ff = v1. It freezes (so G2 passes) but at
    the WRONG place, which is exactly why the coast criterion is three-way and
    not a "did it stop?" check."""
    v = _verdicts(_trace(lambda g: P0 + V0 * B.SEG_T))
    assert v['G1']['verdict'] == 'FAIL'
    assert 'HOLD-AT-ENDPOINT' in v['G1']['detail']
    assert v['G2']['verdict'] == 'PASS'


def test_the_coast_tolerance_cannot_swallow_a_wrong_mode():
    """The bar is a quarter of the smallest inter-model separation, so a coast
    halfway between DECAY and HOLD-AT-ENDPOINT must still FAIL."""
    midpoint = P0 + 0.5 * (V0 * B.HAND_COAST_S + V0 * B.SEG_T)
    v = _verdicts(_trace(lambda g: midpoint))
    assert v['G1']['verdict'] == 'FAIL'


def test_a_coast_inside_the_tolerance_still_passes():
    off = 0.9 * B.MG_COAST_TOL_FLOOR_REV
    v = _verdicts(_trace(lambda g: B.hand_gap_target(P0, V0, g)[0] + off))
    assert v['G1']['verdict'] == 'PASS'


def test_encoder_divergence_from_the_decayed_target_fails_G3():
    v = _verdicts(_trace(lambda g: B.hand_gap_target(P0, V0, g)[0],
                         enc_offset=B.MG_TRACK_BAR_REV + 0.01))
    assert v['G3']['verdict'] == 'FAIL'


def test_an_oversized_reentry_step_fails_G4():
    rows = _trace(lambda g: B.hand_gap_target(P0, V0, g)[0])
    assert _verdicts(rows, reentry_step_rev=0.9)['G4']['verdict'] == 'FAIL'
    assert _verdicts(rows, reentry_step_rev=-0.9)['G4']['verdict'] == 'FAIL'


def test_lead_clamp_engagement_fails_G5_and_says_it_is_a_sample():
    v = _verdicts(_trace(lambda g: B.hand_gap_target(P0, V0, g)[0]),
                  lead_bit_ticks=1)
    assert v['G5']['verdict'] == 'FAIL'
    assert 'NOT a duty' in v['G5']['detail']


# --- SKIP paths: a criterion that cannot be judged must SAY SO -------------
# (Never a silent omission and never a default PASS — the row-17 lesson is
#  that a stage which measures nothing must not report a verdict.)

def test_a_gap_too_short_to_reach_mode_3_skips_the_coast_criterion():
    short = B.MovingGapPlan(2.0, 2)               # 75 ms — Mode 2 at best
    rows = [(k * B.SEG_T, P0 + V0 * k * B.SEG_T, P0 + V0 * k * B.SEG_T,
             B.hand_gap_target(P0, V0, k * B.SEG_T)[0]) for k in (1, 2)]
    res = {c['id']: c for c in B.evaluate_moving_gap(
        rows, P0, V0, short.fw_gap_s, 0.02, 0, 100)}
    assert res['G1']['verdict'] == 'SKIP'
    assert 'Mode 3' in res['G1']['detail']
    assert res['G2']['verdict'] == 'SKIP'


def test_no_echo_in_the_gap_skips_rather_than_passes():
    rows = [(k * B.SEG_T, None, None,
             B.hand_gap_target(P0, V0, k * B.SEG_T)[0]) for k in range(1, 10)]
    res = {c['id']: c for c in B.evaluate_moving_gap(
        rows, P0, V0, PLAN.fw_gap_s, 0.07, 0, 400)}
    assert res['G1']['verdict'] == 'SKIP'
    assert res['G3']['verdict'] == 'SKIP'


def test_a_stage_that_never_reentered_skips_G4():
    rows = _trace(lambda g: B.hand_gap_target(P0, V0, g)[0])
    res = {c['id']: c for c in B.evaluate_moving_gap(
        rows, P0, V0, PLAN.fw_gap_s, None, 0, 400)}
    assert res['G4']['verdict'] == 'SKIP'
    assert 'never re-entered' in res['G4']['detail']


def test_every_criterion_is_always_reported():
    """Five criteria in, five out, whatever the trace — an absent criterion
    reads as an absent problem."""
    for rows in ([], _trace(lambda g: B.hand_gap_target(P0, V0, g)[0])):
        res = B.evaluate_moving_gap(rows, P0, V0, PLAN.fw_gap_s, 0.07, 0, 400)
        assert [c['id'] for c in res] == ['G1', 'G2', 'G3', 'G4', 'G5']
        assert all(c['verdict'] in ('PASS', 'FAIL', 'SKIP') for c in res)
        assert all(c['detail'] for c in res)


def test_the_stage_has_a_deviation_belt_default():
    assert B._MAX_DEV_DEFAULT['moving_gap'] == 0.5
