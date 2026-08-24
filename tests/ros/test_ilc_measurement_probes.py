"""The Phase-0b / 0d measurement probes' pure decision logic.

Probes: ``tools/probes/hand_contact_softness.py`` (0b — is the catch impact
measurable on the hand drive channels?) and ``tools/probes/hand_sensor_settle.py``
(0d — the messy-catch score). Plan: ``plans/active/critical-point-ilc.md``
§§ Phase 0b, Phase 0d.

WHY THESE TESTS EXIST ALONGSIDE THE PROBES' ``--self-check``
------------------------------------------------------------
The self-checks are the instrument's own two-sided acceptance and they run
here, but a green self-check only means *the probe agrees with itself*. These
tests add the two things it cannot do for itself:

* **Mutation guards** — deliberately break a threshold or a rule and assert the
  self-check goes red. Without them a self-check that silently stopped
  discriminating would still print a clean count. This is the
  ``test_hand_sensor_replay.py`` pattern, for the same reason.
* **Pins on the DECISIONS** the phase reports — the outcome-selection rule, the
  score definition, the exact sign-test values — so a future edit that changes
  which pre-registered outcome the evidence selects is a red test rather than a
  quietly different verdict in a re-run.

Both probes are pure below their bag readers, so nothing here touches
``~/Desktop/rosbags``, ``mcap`` or ROS 2, and every case runs on a fresh clone.
"""

from __future__ import annotations

import importlib.util
import os

import pytest

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_PROBES = os.path.join(_REPO, 'tools', 'probes')


def _load(name):
    spec = importlib.util.spec_from_file_location(
        name, os.path.join(_PROBES, name + '.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='module')
def softness():
    return _load('hand_contact_softness')


@pytest.fixture(scope='module')
def settle():
    return _load('hand_sensor_settle')


# ── 0b — the contact-softness probe ───────────────────────────────────────────

def test_softness_self_check_is_clean(softness):
    """Twenty-eight cases, every DETECT twinned with a REFUSE."""
    assert softness.self_check() == 0


def test_softness_self_check_catches_a_relaxed_separation_ratio(softness,
                                                                monkeypatch):
    """Mutation guard on the criterion that decides the whole phase.

    Drop the required separation to 1.0x and the zero-impact synthetic — a series
    whose contact and control windows are identical up to noise — starts
    "separating". That is the exact failure the matched-control design exists to
    prevent, so it must go red here.
    """
    assert softness.self_check() == 0, 'baseline must be green before mutating'
    monkeypatch.setattr(softness, 'SEPARATION_RATIO_MIN', 1.0)
    assert softness.self_check() == 1


def test_softness_self_check_catches_a_disabled_significance_gate(softness,
                                                                  monkeypatch):
    """The other half of the criterion, guarded on the NARROWING side.

    Widening alpha alone cannot flip anything and that is by design — the ratio
    gate is independent, so a zero-impact series still fails on its ~1.0x ratio
    whatever the p-value threshold says. (The first draft of this test asserted
    the widening direction and passed for the wrong reason.) The direction that
    IS reachable is alpha -> 0: no p-value can then clear the bar, the 12 rev/s
    DETECT case stops separating, and the self-check must go red.
    """
    assert softness.self_check() == 0, 'baseline must be green before mutating'
    monkeypatch.setattr(softness, 'SEPARATION_ALPHA', 0.0)
    assert softness.self_check() == 1


def test_the_sign_test_is_exact(softness):
    """Hand-computable values, pinned rather than trusted to a library, because
    the 0b verdict turns on which side of ``SEPARATION_ALPHA`` a p-value lands.

    (The 17/19 and 9/19 pairs were once described here as "what the reference
    bag's within-cycle comparison produced". That provenance no longer holds —
    with the iq impulse referenced to a disjoint quiet-held baseline the
    reference bag reads 19/40 on iq and 29/40 on vel — so they stand purely as
    exact-arithmetic pins now.)"""
    assert softness.binomial_sf(19, 19) == pytest.approx(1.0 / 2 ** 19)
    # Exactly, not approximately: the p == 0.5 branch is one integer ratio.
    assert softness.binomial_sf(17, 19) == 191.0 / 2 ** 19
    assert softness.binomial_sf(0, 19) == pytest.approx(1.0)
    # 9/19 is BELOW the median (9.5), so more than half the mass sits at or
    # above it — not significant in any direction.
    assert softness.binomial_sf(9, 19) > 0.5


def test_the_sign_test_survives_a_pooled_corpus_sized_n(softness):
    """A CEILING, not a rounding nicety.

    The term-by-term float form raised ``OverflowError: int too large to convert
    to float`` from n = 1030: ``C(n, i)`` is an exact bigint while ``0.5 ** n``
    has already underflowed, so the product overflows. A pooled corpus reaches
    four figures of paired catches easily, and the failure would land in the
    middle of the pooled report with every per-bag verdict already printed —
    exit 1 and no headline.

    The n = 2000 value is checkable independently: P(X >= n/2) for a symmetric
    binomial is ``0.5 + P(X = n/2)/2``, and P(X = 1000) = 0.017841, so the
    answer must be 0.508920.
    """
    assert softness.binomial_sf(1000, 2000) == pytest.approx(0.5089195, abs=1e-6)
    assert softness.binomial_sf(0, 2000) == 1.0
    assert softness.binomial_sf(2001, 2000) == 0.0
    # The values the reference bag actually produced are untouched by the change.
    assert softness.binomial_sf(17, 19) == 0.0003643035888671875


def test_the_cadence_census_counts_TRANSITIONS_not_samples(softness):
    """The whole premise of 0b's first question.

    A field republished at 100 Hz from a cache that refreshes at 10 Hz has 100
    samples and 10 transitions per second. Counting samples would report the
    channel as fast and it is not.
    """
    n = 1000
    t = [i * 0.01 for i in range(n)]
    # iq changes once every 10 samples => 10 transitions/s at a 100 Hz rate.
    iq = [float(i // 10) for i in range(n)]
    series = softness.DriveSeries(t, [0.0] * n, iq, [0.0] * n)
    cen = softness.cadence_census(series, [(0.0, 10.0)], 'iq')
    assert cen['n_samples'] == 1000
    assert cen['n_pairs'] == 999
    assert cen['transitions_per_s'] == pytest.approx(10.0, abs=0.2)
    assert cen['duplicate_frac'] == pytest.approx(0.9, abs=0.02)


def test_the_duplicate_fraction_counts_only_pairs_it_actually_compared(
        softness):
    """MULTI-interval, which is how the census is really called: the contact
    census runs over one window per catch, 38 of them on the reference bag.

    The divisor used to be ``n_samples - 1`` — the pair count of ONE contiguous
    run — so every window boundary contributed a phantom pair the scan never
    made, and the reported duplicate fraction was biased upward by
    ``(n_windows - 1) / n_samples``. Small on 38 windows of 30 samples; wrong at
    any size, and wrong in the direction that makes a channel look MORE
    cache-duplicated than it is.
    """
    n = 400
    t = [i * 0.01 for i in range(n)]
    # A fresh value every sample: zero duplicates, whatever the windowing.
    series = softness.DriveSeries(t, [0.0] * n, [float(i) for i in range(n)],
                                  [0.0] * n)
    windows = [(k * 1.0, k * 1.0 + 0.1) for k in range(4)]   # 4 x 10 samples
    cen = softness.cadence_census(series, windows, 'iq')
    assert cen['n_windows'] == 4
    assert cen['n_samples'] == 40
    assert cen['n_pairs'] == 36            # 4 windows x 9 pairs, NOT 39
    assert cen['n_transitions'] == 36
    assert cen['duplicate_frac'] == pytest.approx(0.0)


@pytest.mark.parametrize('vel_sep,iq_sep,want', [
    (True, True, 'i'),
    (False, False, 'ii'),
    (True, False, 'iii'),
    (False, True, 'UNREGISTERED'),
])
def test_the_outcome_rule_is_the_pre_registered_one(softness, vel_sep, iq_sep,
                                                    want):
    """All four cells, including the one the plan does NOT enumerate.

    The plan lists (i)/(ii)/(iii); "iq separates, vel does not" is logically
    available and unlisted, and the probe must report it as unregistered rather
    than rounding it into the nearest box. Rounding an unenumerated result into a
    pre-registered one is how a pre-registration stops being one — and on this
    corpus the weaker control really does produce that cell, so it is not a
    hypothetical.
    """
    def verdict(sep):
        return {'separates': sep, 'median_ratio': 2.0 if sep else 1.0,
                'sign_test': {'n_wins': 9, 'n_pairs': 10, 'p_one_sided': 0.01},
                'spacing_ms_p50': 10.0}

    code, _headline, _reasons = softness.select_outcome(verdict(vel_sep),
                                                        verdict(iq_sep))
    assert code == want


def test_the_pooled_verdict_cannot_use_a_weaker_control_than_the_per_bag_one(
        softness):
    """Regression guard for a real defect found while running 0b on the
    reference bag: the pooled block pooled only the WITHIN-CYCLE pairs, so it
    announced outcome (i) directly beneath a per-bag CROSS-LABEL verdict of (ii).
    A pooled headline that silently reverts to a weaker control is worse than no
    pooled headline."""
    series, events = softness._synth(12.0)
    held = [(e - 3.0, e + 3.0) for e in events]
    anchors = {'caught': events[:6], 'missed': events[6:]}
    res = softness.analyse(series, events, events, held, anchors)
    pooled = softness.pool({'bag': res})
    assert pooled['verdict']['control_used'] == 'cross_label (pooled)'
    # The expectation is INDEPENDENT, not "whatever the per-bag pass said" —
    # that form passed whenever the two agreed, including when both were wrong.
    # This series injects the SAME impact into the caught and the missed
    # population, so the cross-label control sees no difference and the honest
    # answer is (ii). The defect's answer is (iii): the within-cycle control
    # compares the impact against a quiet window and separates on vel at 9.0x.
    assert res['verdict']['outcome'] == 'ii'
    assert pooled['verdict']['outcome'] == 'ii'
    assert res['within_cycle_control']['peak_abs_dvel']['separates'] is True


def _level_series(softness, t0=20.0, span=(10.0, 30.0), dt=0.01):
    """A drive series whose ``iq`` sits at three known LEVELS: 5 A through the
    contact window of a catch at ``t0``, 3 A through its within-cycle control,
    1 A everywhere else (which is where the guarded quiet-held span falls)."""
    n = int((span[1] - span[0]) / dt)
    t = [span[0] + i * dt for i in range(n)]
    iq = []
    for ti in t:
        if t0 - softness.CONTACT_PRE_S <= ti < t0 + softness.CONTACT_POST_S:
            iq.append(5.0)
        elif t0 - softness.CONTROL_LEAD_S <= ti < t0 - softness.CONTROL_LAG_S:
            iq.append(3.0)
        else:
            iq.append(1.0)
    return softness.DriveSeries(t, [0.0] * n, iq, [0.0] * n)


def test_the_iq_baseline_comes_from_a_span_that_is_neither_window(softness):
    """The reference cannot be one of the two things being compared.

    Both integrals used to subtract the CONTROL window's own median, and the
    median MINIMISES its own window's L1 deviation — so the control started every
    comparison at its own floor and could not lose. Here that recipe scores the
    control at exactly 0 A·s (``|3 - 3|``) and the contact at ``|5 - 3|``,
    turning a genuine 2x into a divide-by-zero the ratio reports as ``None``.

    Drawn from the guarded quiet-held population instead (1 A, disjoint from
    both windows by ``QUIET_GUARD_S``), the two windows are 4 A and 2 A off the
    same reference and the ratio is the honest 2.0.
    """
    series = _level_series(softness)
    guarded = softness.quiet_spans([(10.0, 30.0)], [20.0])
    assert softness.quiet_baseline(series, guarded, 20.0) == pytest.approx(1.0)
    # ...and it is genuinely outside both windows.
    for lo, hi in guarded:
        assert hi <= 20.0 - softness.QUIET_GUARD_S or lo >= 20.0 + softness.QUIET_GUARD_S

    pv = softness._paired_values(series, [20.0], guarded)
    assert pv['iq_c'][0] == pytest.approx(4.0 * 0.29, rel=1e-6)
    assert pv['iq_k'][0] == pytest.approx(2.0 * 0.29, rel=1e-6)
    assert softness.ratio(pv['iq_c'], pv['iq_k']) == pytest.approx(2.0)
    assert pv['n_no_baseline'] == 0
    # The old recipe, spelled out, so the difference is visible in the diff and
    # not just in prose: the control's own median annihilates its own integral.
    own = softness.median_in(series, 20.0 - softness.CONTROL_LEAD_S,
                             20.0 - softness.CONTROL_LAG_S, 'iq')
    assert own == pytest.approx(3.0)
    assert softness.iq_impulse(series, 20.0 - softness.CONTROL_LEAD_S,
                               20.0 - softness.CONTROL_LAG_S, own) == 0.0


def test_a_control_window_that_overlaps_another_contact_is_excluded(softness):
    """During a rattle the "no ball in the cup yet" control window contains a
    previous raw arrival's contact window — so it is measuring a contact.

    The bias runs conservative (an inflated control shrinks the ratio), which is
    why it never surfaced as a false positive; the pair still does not mean what
    the column header says, so it is dropped as a PAIR and counted.
    """
    series, events = softness._synth(12.0)
    held = [(e - 3.0, e + 3.0) for e in events]
    guarded = softness.quiet_spans(held, events)
    clean = softness._paired_values(series, events, guarded)
    assert clean['n_overlap_excluded'] == 0

    rattled = sorted(list(events) + [events[3] - 0.6])
    dirty = softness._paired_values(series, rattled, guarded)
    assert dirty['n_overlap_excluded'] == 1
    # Dropped on BOTH sides — half a pair would bias the population it stayed in.
    holes = [i for i, x in enumerate(dirty['dv_c']) if x is None]
    assert holes == [i for i, x in enumerate(dirty['dv_k']) if x is None]
    assert holes == [i for i, x in enumerate(dirty['iq_c']) if x is None]
    assert len(holes) == 1


def test_the_two_noise_floors_are_reported_separately(softness):
    """The static floor (hand still) is not the floor a MOVING hand must beat.

    Collapsing them is how contact windows came to look 8.8x "above noise" on
    the reference bag while failing every matched control — so the probe must
    keep both, and the static one must be the smaller.
    """
    series, events = softness._synth(0.0)
    nf = softness.noise_floor(series, [(events[0] - 3.0, events[0] - 1.0)])
    assert nf['static']['n'] > 0 and nf['moving']['n'] > 0
    assert nf['static']['n'] < nf['moving']['n']
    assert nf['static']['max'] < nf['moving']['max']
    assert 'at least' in nf['recipe']      # the recipe travels with the numbers


# ── 0d — the sensor-settle probe ──────────────────────────────────────────────

def test_settle_self_check_is_clean(settle):
    """Thirty-four cases, every MESSY twinned with a CLEAN."""
    assert settle.self_check() == 0


@pytest.mark.parametrize('bound_s', (0.1, 60.0))
def test_settle_self_check_catches_a_RESIZED_quick_drop_bound(settle,
                                                              monkeypatch,
                                                              bound_s):
    """Mutation guard on the ground truth itself, BOTH directions.

    ``QUICK_DROP_S`` = 1.5 s is what makes the reference bag's three known
    sub-second seat-then-leaves the messy population; move it and the corpus's
    labels move with it.

    * 0.1 s NARROWS it below the shortest measured quick-drop (0.569 s), so the
      known-messy case stops being ground-truth messy.
    * 60 s WIDENS it past the synthetic's 40 s clean possession, so the clean
      twin stops being clean and the score has nothing left to discriminate.

    The widening value has to clear 40 s specifically — an earlier draft used
    30 s, which changed no label at all and so passed while asserting nothing.
    Both bounds are ABSOLUTE rather than written relative to the constant they
    guard, which is the only way a guard can see a resize in both directions.
    """
    assert settle.self_check() == 0, 'baseline must be green before mutating'
    monkeypatch.setattr(settle, 'QUICK_DROP_S', bound_s)
    assert settle.self_check() == 1


def test_settle_self_check_catches_a_zero_threshold(settle, monkeypatch):
    """A score threshold of 0 calls every catch messy: perfect recall, useless
    instrument. The false-messy half of the outcome rule is what catches it."""
    assert settle.self_check() == 0, 'baseline must be green before mutating'
    monkeypatch.setattr(settle, 'MESSY_THRESHOLD', 0)
    assert settle.self_check() == 1


def test_the_score_is_the_RAW_flip_count(settle):
    """The score definition, pinned — and pinned to the RAW bit specifically.

    The debounced count within W >= the quick-drop bound is the ground-truth
    definition restated, so it separates perfectly and demonstrates nothing. The
    raw count is not implied by the label: this case is a ball that rattled and
    stayed, which the possession label calls clean and the score calls messy.
    """
    samples = settle._samples([(20.0, True, True), (20.3, False, True),
                               (20.5, True, True)])
    rows = settle.sweep(samples, settle.catches_with_truth(samples))
    assert rows[0]['truth_messy'] is False        # possession never lost
    assert settle.messy_score(rows[0], 1.0) == 2  # raw bit flipped twice
    assert rows[0]['deb_flips'][1.0] == 0         # debounce absorbed it


def test_an_invalid_span_never_mints_a_flip(settle):
    """D13 / C-POSSESS-1 § 2: UNKNOWN is not EMPTY.

    Letting an invalid sample act as a level would mint a flip out of a telemetry
    hiccup and inflate the very score being defined — and 2026-08-12_18-57-05 in
    this corpus is only 63 % valid, so the path is exercised by real data, not
    hypothetically.
    """
    samples = settle._samples([(20.0, True, True)], invalid=((20.2, 20.6),))
    rows = settle.sweep(samples, settle.catches_with_truth(samples))
    assert settle.messy_score(rows[0], 1.0) == 0


def test_an_unclosed_final_segment_is_not_a_quick_drop(settle):
    """"We stopped recording" is not "the ball fell out".

    Without this, every bag that ends shortly after a catch reports a phantom
    messy event — and short bags are most of this corpus.
    """
    samples = settle._samples([(119.0, True, True)])
    rows = settle.catches_with_truth(samples)
    assert rows[0]['possession_closed'] is False
    assert rows[0]['truth_messy'] is False


def test_the_debounce_lag_matcher_refuses_an_implausible_pairing(settle):
    """A raw edge the debounce NEVER confirmed must not be paired with the next
    catch's debounced edge.

    The unbounded matcher reported a 41-second "debounce lag" on the reference
    bag — a census inventing a fact. Unconfirmed raw edges are counted, not
    matched, and that population is exactly the flicker population the score is
    built on.
    """
    samples = settle._samples([(20.0, True, True), (20.3, False, True),
                               (20.5, True, True), (60.0, False, False)])
    lag = settle.debounce_lag(samples)
    assert lag['fall_ms']['n_unconfirmed'] >= 1
    if lag['fall_ms'].get('n'):
        assert lag['fall_ms']['max'] <= settle.MAX_DEBOUNCE_LAG_S * 1e3


def test_the_poll_census_reads_the_stamp_not_the_publish_rate(settle):
    """0d's first question. ``/hand_telemetry`` publishes at 100 Hz; the sensor
    is polled far slower, and the flip count is bounded by the poll. A census
    that counted messages would report 100 Hz and be wrong by the whole factor
    it exists to expose — 71 ms vs a configured 20 ms on the reference bag."""
    samples = settle._samples([(20.0, True, True)], dt=0.071)
    cad = settle.poll_cadence(samples)
    assert cad['p50_ms'] == pytest.approx(71.0, abs=1.0)
    assert cad['configured_ms'] == pytest.approx(20.0)


def test_a_corpus_with_no_messy_catch_refuses_to_select_an_outcome(settle):
    """A null is a legitimate result; a DEFAULT is not.

    Eight of the ten 2026-08-12 sittings carry zero or one catch and no messy
    event at all, so this is the common case in the real corpus, not an edge.
    """
    samples = settle._samples([(20.0, True, True)])
    rows = settle.sweep(samples, settle.catches_with_truth(samples))
    _scored, confusion = settle.classify(rows, 1.0)
    code, headline, _reasons = settle.select_outcome(
        confusion, settle.poll_cadence(samples))
    assert code is None
    assert 'UNDECIDABLE' in headline


def test_the_window_choice_is_derived_not_hardcoded(settle):
    """W comes from the sweep's plateau, and the rationale travels with it.

    A W nobody can re-derive is a magic number; the reference bag selects 0.75 s
    as the plateau's low end AND the smallest window that recalls all three known
    quick-drops. This asserted only ``w in W_SWEEP`` — true of eight values, so
    it could not see the window move — and the shipped default was 1.0 s while
    every documented number for this metric was measured at 0.75 s.
    """
    samples = settle._samples([(20.0, True, True), (20.6, False, False),
                               (50.0, True, True)])
    rows = settle.sweep(samples, settle.catches_with_truth(samples))
    w, why = settle.choose_window(rows)
    assert w == 0.75
    assert 'plateau' in why and 'recalls' in why
    assert settle.recall_at(rows, w) == (1, 1)


def test_the_shipped_window_is_the_one_the_metric_is_defined_at(settle):
    """0.75 s, everywhere the probe can be asked "what W?".

    The score's documented recall / false-clean / false-messy rates are measured
    at 0.75 s. The module shipped a 1.0 s default, so ``messy_score(row)`` with
    no window — and every bag whose corpus cannot size W, which is most of them —
    silently scored at a window the docs never describe.
    """
    assert settle.SHIPPED_W_S == 0.75
    assert settle.FALLBACK_W_S == settle.SHIPPED_W_S
    assert settle.SHIPPED_W_S in settle.W_SWEEP
    # The default really is that constant, on both surfaces that have one.
    samples = settle._samples([(20.0, True, True), (20.3, False, True),
                               (20.5, True, True)])
    rows = settle.sweep(samples, settle.catches_with_truth(samples))
    assert settle.messy_score(rows[0]) == rows[0]['raw_flips'][0.75]
    # A corpus with no messy catch cannot size W and must fall back to the
    # SHIPPED window rather than to a second, undocumented one.
    quiet = settle.analyse(settle._samples([(20.0, True, True)]))
    assert quiet['w_chosen_s'] == 0.75


def test_a_forced_window_outside_the_sweep_is_analysed_not_crashed(settle):
    """``--window-s 0.9`` raised a bare ``KeyError(0.9)`` from inside
    ``classify``: the value was never swept, so the per-catch lookup missed. And
    ``main`` catches only ``(IOError, OSError)``, so it escaped as a traceback
    with the per-bag reports already printed.

    Forcing a free-form W is a legitimate ask of a probe whose entire subject is
    that W is a choice, so the forced value JOINS the sweep.
    """
    samples = settle._samples([(20.0, True, True), (20.6, False, False),
                               (40.0, True, True), (80.0, False, False)])
    res = settle.analyse(samples, w=0.9)
    assert res['w_chosen_s'] == 0.9
    assert '0.9' in res['w_sweep']              # swept, not silently dropped
    assert res['confusion']['w_s'] == 0.9
    assert settle.pool({'a': res}, 0.9)['w_chosen_s'] == 0.9
    # W = 0 is degenerate but STATED. `w = w or chosen or DEFAULT` read it as
    # unset and reported a window nobody asked for.
    assert settle.analyse(samples, w=0.0)['w_chosen_s'] == 0.0


def test_a_drop_on_the_FIRST_post_arrival_poll_scores_messy(settle):
    """THE false-clean regression, in the one direction the outcome rule forbids.

    The flip census used to rescan the samples inside ``(t_catch, t_catch + W]``
    and seed its previous level from the FIRST sample in that window — so a
    transition between the last pre-window sample and the first in-window one was
    invisible. The window opens at the catch instant, so the invisible transition
    was the one on the first post-arrival poll: a ball that seats and reads EMPTY
    on the very next poll — the shortest quick-drop the sensor can express, and
    ground-truth MESSY by possession — scored 0 and was called CLEAN.

    Outcome (i) requires ``false_clean_rate == 0``; this bug could only ever
    manufacture false CLEANs.
    """
    samples = settle._samples([(20.0, True, True), (20.03, False, False)])
    rows = settle.sweep(samples, settle.catches_with_truth(samples))
    assert rows[0]['truth_messy'] is True
    assert rows[0]['possession_s'] == pytest.approx(0.071, abs=0.002)
    for w in settle.W_SWEEP:
        assert rows[0]['raw_flips'][w] >= settle.MESSY_THRESHOLD, w
    _scored, confusion = settle.classify(rows, settle.SHIPPED_W_S)
    assert confusion['false_clean'] == 0


def test_the_flip_census_is_the_labellers_own_edge_list(settle):
    """One definition of "a transition", shared with the robot.

    ``jugglebot.toss_record.edges`` is what ``label_from_sensor`` and the live
    ``HandBallSensorSource`` use, so counting off its output is what stops this
    probe and the robot drifting onto different notions of a flip — and it is
    what makes the invalid-sample rule (UNKNOWN is not EMPTY) automatic here
    rather than restated.
    """
    samples = settle._samples([(20.0, True, True), (20.3, False, True),
                               (20.5, True, True)])
    raw = settle.edge_times(samples, False)
    deb = settle.edge_times(samples, True)
    assert raw == sorted(raw) and len(raw) == 3     # rise, fall, rise
    assert len(deb) == 1                            # the debounce absorbed it
    t = settle.catches_with_truth(samples)[0]['t_catch']
    # (lo, hi]: the catch edge that OPENS the window is not a flip inside it.
    assert settle.flips_in(raw, t, t + 1.0) == 2
    assert settle.flips_in(raw, t - 1.0, t) == 1
