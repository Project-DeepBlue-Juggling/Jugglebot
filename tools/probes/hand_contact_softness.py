#!/usr/bin/env python3
"""Is the CATCH IMPACT measurable on the hand drive channels? — Phase 0b.

WHAT IT DOES
------------
For every labelled catch in a rosbag it extracts ``/hand_telemetry``
``vel_meas`` and ``iq_meas`` around the arrival instant and answers, in the
order ``plans/active/critical-point-ilc.md`` § Phase 0b fixes:

1. **The CADENCE CENSUS, first, before any noise question.** ``iq_meas`` reaches
   the bag through the on-change / 1 Hz DIAGNOSTIC path and is *republished at
   100 Hz from a cache*, so the publish rate is not the refresh rate. What counts
   is **distinct-value transitions per second**, inside vs outside the contact
   windows. ``vel_meas`` is censused the same way as the control.
2. **The TRANSIENT CENSUS**, against a stated noise floor and — the part that
   actually decides it — against **matched controls**: a same-duration window
   earlier in the same cycle, and (where the bag has announcements) the same
   window on a MISSED toss, where the hand ran the same choreography with no
   ball to catch.
3. **The pre-registered outcome** — (i), (ii) or (iii) — that the evidence
   selects, from an explicit rule (:func:`select_outcome`), not from prose.

WHY THE MATCHED CONTROLS ARE THE WHOLE INSTRUMENT
--------------------------------------------------
The naive comparison — contact-window ``peak |Δvel|`` against a noise floor
measured while the hand is still — makes this channel look *excellent*: on
``2026-08-10_16-30-44`` the median contact peak is 1.715 rev/s against a
static-floor p99 of 0.195 rev/s, an apparent 8.8x. That number is an artefact.
The hand is **not** still at contact: it is finishing a commanded catch stroke,
and ``vel_meas`` tracks ``vel_ff_cmd`` through that stroke to within ~1 %
(-12.136 vs -12.320, -28.186 vs -27.720, -42.978 vs -43.120 rev/s on three
consecutive samples of one catch). A floor measured on a stationary hand is the
wrong denominator for a moving one, and comparing against it is how a channel
that measures the *command* gets mistaken for one that measures the *ball*.

So the probe reports the naive comparison **and** the controls, and selects the
outcome on the controls. See the header of :func:`select_outcome`.

WHAT IT IMPORTS RATHER THAN RE-IMPLEMENTS (plan design constraint 1)
--------------------------------------------------------------------
The bag reader, the self-toss filter, the sensor edges and the catch LABEL all
come from ``tools/probes/toss_record_miner.py`` and the installed
``jugglebot.toss_record`` it imports — the labelling authority. This probe adds
exactly one thing the miner does not carry: a second, light decode pass over
``/hand_telemetry`` for the DRIVE fields (``vel_meas``, ``iq_meas``,
``vel_ff_cmd``), which are not part of ``SensorSample``. Nothing about "which
catch" or "was it caught" is decided here.

OUTPUTS
-------
``temp/probes/hand_contact_softness_<stamp>.json`` with ``--json``. Bags are
processed **sequentially** — this box has 8 GB and a full sitting's
``/hand_telemetry`` is ~70 k messages.

USAGE
-----
    python tools/probes/hand_contact_softness.py --bag 2026-08-10_16-30-44
    python tools/probes/hand_contact_softness.py --all --match 2026-08-12 --json
    python tools/probes/hand_contact_softness.py --self-check

Exit 0 = every requested bag analysed; 1 = a bag was unreadable or the
self-check failed.
"""

from __future__ import annotations

import argparse
import bisect
import glob
import json
import os
import sys
from datetime import datetime

_HERE = os.path.dirname(os.path.abspath(__file__))          # tools/probes
_REPO = os.path.dirname(os.path.dirname(_HERE))             # repo root
for _p in (_HERE, _REPO, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import toss_record_miner as miner                                # noqa: E402
from jugglebot.toss_record import edges                          # noqa: E402

DEFAULT_ROOT = miner.DEFAULT_ROOT
OUT_DIR = os.path.join(_REPO, 'temp', 'probes')

# ── Windows (every one MEASURED or derived, never guessed) ────────────────────

#: The contact window, relative to the sensor's RAW empty->held edge. The lead is
#: sized on the MEASURED poll cadence, not on a guess: the hand sensor polls at
#: p50 71 / p95 111 ms (re-verified below and in the toss-build 2a findings), so
#: the true seat instant lies up to one poll interval BEFORE the edge the bag
#: reports. 100 ms covers the p95 poll; 200 ms after covers the seat and settle.
CONTACT_PRE_S = 0.10
CONTACT_POST_S = 0.20

#: The matched control window: the SAME duration, 0.5-0.8 s earlier in the same
#: cycle. Same hand, same session, same commanded choreography, no ball in the
#: cup yet. Paired with its own contact window, so per-catch stroke-to-stroke
#: variation cancels.
CONTROL_LEAD_S = 0.80
CONTROL_LAG_S = 0.50

#: One nanosecond, the amount an upper bound is nudged by when an interval has
#: to be CLOSED. `DriveSeries.span` bisects LEFT on the upper bound, which is
#: right for back-to-back windows and wrong for "the whole bag".
_EPS_S = 1e-9

#: Quiet-segment guard for the noise floor: this far from ANY sensor edge.
QUIET_GUARD_S = 1.0
#: A sample is "hand quiescent" below this speed. The static floor uses it; the
#: moving floor does not.
QUIET_VEL_REV_S = 0.5

#: A ball landing in a rigid cup is a contact event of order 10-30 ms. To
#: *integrate* a transient of that duration a channel needs its samples spaced at
#: least ~3x finer, so this is the cadence a channel must beat to be able to
#: carry an impulse metric at all. It is a physical requirement, not a
#: convention — and it is REPORTED, not used as the outcome gate (the controls
#: are; see select_outcome).
IMPACT_DURATION_MS = 30.0
CADENCE_SPACING_MAX_MS = IMPACT_DURATION_MS / 3.0

#: A channel "separates" when its contact distribution beats its matched control
#: by this factor on the median AND the paired sign test rejects at 5 %.
SEPARATION_RATIO_MIN = 1.5
SEPARATION_ALPHA = 0.05

#: Relative slack on the cadence comparison. A channel refreshing at EXACTLY the
#: requirement (10.0 ms against 10.0) must not be called starved by a float
#: representation error one ulp wide — `sp > limit` on binary floats is a coin
#: toss at the boundary, and this test's whole job is to be reproducible.
_CADENCE_SLACK = 1.0 + 1e-9

OUTCOME_I = ('i', 'RESOLVABLE — the measured transient carries the contact, and '
                  'the metric ships with its floor and admission thresholds')
OUTCOME_II = ('ii', 'NOT RESOLVABLE in either channel — softness stays '
                    'modeled-surrogate-only in v1; a firmware-side high-rate '
                    'capture is the follow-on')
OUTCOME_III = ('iii', 'RESOLVABLE in vel_meas, cadence-starved in iq_meas — a '
                      'vel_meas-only metric carries v1')
OUTCOME_UNREGISTERED = (
    'UNREGISTERED', 'iq_meas separates but vel_meas does not — the MIRROR of '
                    'the plan\'s outcome (iii), which the plan does not '
                    'enumerate. Report it; do not force it into (i)/(ii)/(iii)')


# ── The drive series (pure; this is what the tests drive) ─────────────────────

class DriveSeries(object):
    """``/hand_telemetry`` drive channels on the announcement clock.

    Parallel lists rather than a list of tuples: every metric below is a scan
    over one or two channels inside a slice, and the slice bounds come from one
    bisect on ``t``.
    """

    def __init__(self, t, vel, iq, vel_ff):
        self.t = list(t)
        self.vel = list(vel)
        self.iq = list(iq)
        self.vel_ff = list(vel_ff)

    def __len__(self):
        return len(self.t)

    def span(self, lo, hi):
        """-> ``(a, b)`` index bounds for ``lo <= t < hi``."""
        return bisect.bisect_left(self.t, lo), bisect.bisect_left(self.t, hi)

    def channel(self, name):
        return getattr(self, name)


def _pct(xs, q):
    if not xs:
        return None
    return xs[min(len(xs) - 1, int(q * len(xs)))]


def describe(xs):
    """The five-number summary every metric is reported through."""
    xs = sorted(float(x) for x in xs)
    if not xs:
        return {'n': 0}
    return {'n': len(xs), 'min': xs[0], 'p25': _pct(xs, .25),
            'median': _pct(xs, .5), 'p75': _pct(xs, .75),
            'p95': _pct(xs, .95), 'p99': _pct(xs, .99), 'max': xs[-1]}


# ── 1. The cadence census ─────────────────────────────────────────────────────

def cadence_census(series, intervals, channel):
    """Distinct-value TRANSITIONS per second for ``channel`` over ``intervals``.

    The number the plan asks for, and it is deliberately not the sample count: a
    field republished at 100 Hz from a cache that refreshes at 1 Hz produces 100
    samples and 1 transition per second, and only the second number says whether
    the field can carry a transient.

    ``duplicate_frac`` is the same fact from the other side — the fraction of
    consecutive samples that carry an IDENTICAL value. It is reported because it
    is the one that bites an impulse integral: a duplicated sample aliases into
    the high-frequency band exactly where a contact transient would live.

    Its denominator is ``n_pairs``, the number of CONSECUTIVE-SAMPLE pairs
    actually compared, accumulated per interval. It used to be
    ``n_samples - 1``, which is the pair count of ONE contiguous run: over the
    38 contact windows of the reference bag that is 37 phantom pairs the scan
    never made, so the reported duplicate fraction was biased upward by
    ~(n_windows - 1)/n_samples. Small here, wrong everywhere.
    """
    vals = series.channel(channel)
    n_samples = n_trans = n_pairs = 0
    duration = 0.0
    gaps = []
    for lo, hi in intervals:
        a, b = series.span(lo, hi)
        if b - a < 2:
            continue
        duration += series.t[b - 1] - series.t[a]
        n_samples += b - a
        n_pairs += max(0, b - a - 1)
        last_change = None
        for i in range(a + 1, b):
            if vals[i] != vals[i - 1]:
                n_trans += 1
                if last_change is not None:
                    gaps.append(series.t[i] - last_change)
                last_change = series.t[i]
    gaps.sort()
    return {
        'channel': channel,
        'n_windows': len(intervals),
        'n_samples': n_samples,
        'n_pairs': n_pairs,
        'duration_s': duration,
        'n_transitions': n_trans,
        'transitions_per_s': (n_trans / duration) if duration > 0 else None,
        'duplicate_frac': (1.0 - float(n_trans) / n_pairs
                           if n_pairs > 0 else None),
        'spacing_ms_p50': (_pct(gaps, .5) * 1e3) if gaps else None,
        'spacing_ms_p95': (_pct(gaps, .95) * 1e3) if gaps else None,
    }


def quiet_spans(held_spans, all_edges, guard_s=None):
    """-> the ball-held intervals with a ``guard_s`` exclusion around EVERY
    sensor edge removed.

    The noise floor's population, and (since the audit fix below) the population
    the shared ``iq`` baseline is drawn from too. The guard is what makes it a
    floor rather than a second measurement of the contact: without it the samples
    either side of the arrival edge — the loudest ones in the bag — sit inside
    the "quiet" population and the floor absorbs the very transient it is the
    denominator for.

    ``guard_s`` defaults through a ``None`` sentinel rather than binding
    :data:`QUIET_GUARD_S` at definition time: a default bound at ``def`` time is
    invisible to ``monkeypatch.setattr``, so a mutation guard aimed at the
    constant would pass while asserting nothing.
    """
    guard_s = QUIET_GUARD_S if guard_s is None else guard_s
    blocked = sorted((e - guard_s, e + guard_s) for e in all_edges)
    out = []
    for lo, hi in held_spans:
        if hi <= lo:
            continue
        cursor = lo
        for blo, bhi in blocked:
            if bhi <= cursor or blo >= hi:
                continue
            if blo > cursor:
                out.append((cursor, min(blo, hi)))
            cursor = max(cursor, bhi)
            if cursor >= hi:
                break
        if cursor < hi:
            out.append((cursor, hi))
    return out


# ── 2. The transient metrics ──────────────────────────────────────────────────

def peak_abs_step(series, lo, hi, channel='vel'):
    """``max |x[i] - x[i-1]|`` — the plan's named candidate for ``vel_meas``."""
    vals = series.channel(channel)
    a, b = series.span(lo, hi)
    if b - a < 2:
        return None
    return max(abs(vals[i] - vals[i - 1]) for i in range(a + 1, b))


def peak_command_residual(series, lo, hi):
    """``max |vel_meas - vel_ff_cmd|``.

    The command-free view of the same window: it subtracts the commanded profile
    instead of differencing it away, so a disturbance the velocity loop absorbed
    into a tracking error shows here even when it never bends the velocity trace.
    If the ball is visible anywhere on this channel, it is visible here.
    """
    a, b = series.span(lo, hi)
    if b - a < 1:
        return None
    return max(abs(series.vel[i] - series.vel_ff[i]) for i in range(a, b))


def iq_impulse(series, lo, hi, baseline):
    """``∫ |iq - baseline| dt`` over the window, trapezoid, in A·s.

    The plan's named candidate for ``iq_meas``. ``baseline`` is supplied by the
    caller and is the SAME reference for the contact and the control window — an
    impulse measured against its own window's median would subtract the very
    excursion it is trying to measure.

    **AUDIT FIX 2026-08-12.** The caller used to pass the CONTROL window's own
    median, which is not a neutral reference: it minimises the control's
    deviation by construction, so the control integral is systematically the
    smaller of the two whatever the data says. Driven on null data (two windows
    identical up to noise) that recipe returned a ratio of ~1.08 with the paired
    sign test rejecting at 5 % — a "separation" manufactured by the baseline.
    The reference now comes from a DISJOINT span (:func:`quiet_baseline`), so
    neither window is privileged.
    """
    a, b = series.span(lo, hi)
    if b - a < 2:
        return None
    total = 0.0
    for i in range(a + 1, b):
        mid = 0.5 * (series.iq[i] + series.iq[i - 1])
        total += abs(mid - baseline) * (series.t[i] - series.t[i - 1])
    return total


def median_in(series, lo, hi, channel):
    a, b = series.span(lo, hi)
    if b - a < 1:
        return None
    return sorted(series.channel(channel)[a:b])[(b - a) // 2]


def quiet_baseline(series, guarded, t):
    """-> the ``iq`` reference BOTH windows of one catch are integrated against.

    The median ``iq`` over the guarded quiet-held span NEAREST ``t`` — the same
    population :func:`noise_floor` uses, i.e. ball-held and at least
    :data:`QUIET_GUARD_S` from any sensor edge, so it is disjoint from both the
    contact window and its within-cycle control by construction.

    Nearest rather than whole-bag because the ``iq`` level tracks the hand's own
    duty (holding a ball at a different platform height is a different current),
    and a session-wide median would import that drift into every integral as a
    constant offset. Spans are tried in order of distance so an empty one falls
    through to the next rather than costing the pair.

    -> ``None`` when the bag has no guarded quiet span at all; the caller then
    has no neutral reference and drops the pair rather than falling back to a
    window-owned median, which is the recipe this function replaced.
    """
    cands = []
    for lo, hi in guarded:
        if hi <= lo:
            continue
        d = 0.0 if lo <= t <= hi else min(abs(lo - t), abs(hi - t))
        cands.append((d, lo, hi))
    for _d, lo, hi in sorted(cands):
        base = median_in(series, lo, hi, 'iq')
        if base is not None:
            return base
    return None


def distinct_transitions(series, lo, hi, channel):
    vals = series.channel(channel)
    a, b = series.span(lo, hi)
    return sum(1 for i in range(a + 1, b) if vals[i] != vals[i - 1])


def noise_floor(series, intervals, quiet_vel=None):
    """The stated noise floor, with its recipe attached to the numbers.

    TWO floors, because one of them is a trap:

    * ``static`` — ``|Δvel|`` over samples where the hand is quiescent
      (``|vel| < quiet_vel`` at BOTH ends of the difference) inside ``intervals``
      (which the caller passes as ball-held, >= 1 s from any sensor edge). This is
      the sensor + quantisation floor.
    * ``moving`` — the same over ALL samples in ``intervals``, regardless of hand
      speed. This is the floor a transient must beat when you do not already know
      the hand was still.

    A contact happens with the hand MOVING, so ``moving`` is the honest floor and
    ``static`` is reported to make the size of the difference visible.

    ``quiet_vel`` defaults through a ``None`` sentinel — see :func:`quiet_spans`
    for why a ``def``-time default defeats a ``monkeypatch`` mutation guard.
    """
    quiet_vel = QUIET_VEL_REV_S if quiet_vel is None else quiet_vel
    static, moving = [], []
    for lo, hi in intervals:
        a, b = series.span(lo, hi)
        for i in range(a + 1, b):
            d = abs(series.vel[i] - series.vel[i - 1])
            moving.append(d)
            if (abs(series.vel[i]) < quiet_vel
                    and abs(series.vel[i - 1]) < quiet_vel):
                static.append(d)
    return {
        'recipe': ('|d(vel_meas)| between consecutive /hand_telemetry samples, '
                   'over ball-held spans at least {:.1f} s from ANY sensor edge; '
                   '"static" additionally requires |vel_meas| < {:.2f} rev/s at '
                   'both ends of the difference'.format(QUIET_GUARD_S,
                                                        quiet_vel)),
        'static': describe(static),
        'moving': describe(moving),
    }


# ── The paired sign test (exact, stdlib) ──────────────────────────────────────

def _comb(n, k):
    num = den = 1
    for i in range(k):
        num *= n - i
        den *= i + 1
    return num // den


def binomial_sf(k, n, p=0.5):
    """P(X >= k) for X ~ Binomial(n, p).

    The ``p == 0.5`` case — the only one the sign test ever uses — is computed as
    ONE exact integer ratio, ``sum(C(n, i)) / 2**n``, evaluated by Python's
    correctly-rounded ``int / int``.

    The term-by-term float form it replaces OVERFLOWED for ``n >= 1030``:
    ``C(n, i)`` is an exact bigint while ``0.5 ** n`` underflows to 0.0 at
    n = 1075, so the product is ``huge * 0.0`` long before that and raises
    ``OverflowError: int too large to convert to float``. A pooled corpus reaches
    n = 1030 paired catches easily — this is a real ceiling on the pooled
    verdict, not a theoretical one — and the failure would land in the middle of
    the pooled report with the per-bag verdicts already printed.
    """
    if n <= 0:
        return 1.0
    k = max(int(k), 0)
    if k > n:
        return 0.0
    if p == 0.5:
        return sum(_comb(n, i) for i in range(k, n + 1)) / (1 << n)
    return sum(_comb(n, i) * (p ** i) * ((1.0 - p) ** (n - i))
               for i in range(k, n + 1))


def sign_test(contact, control):
    """Paired one-sided sign test: does ``contact`` exceed its OWN control?

    Paired, not two-sample, and that is the point: catches happen at different
    phases of the stroke and at different commanded speeds, so an unpaired
    comparison is dominated by which catches happened to land where. Ties are
    dropped (the standard convention) and reported, because a channel that ties
    often is a channel that is not moving.
    """
    wins = ties = 0
    pairs = 0
    for a, b in zip(contact, control):
        if a is None or b is None:
            continue
        pairs += 1
        if a > b:
            wins += 1
        elif a == b:
            ties += 1
    effective = pairs - ties
    return {'n_pairs': pairs, 'n_wins': wins, 'n_ties': ties,
            'p_one_sided': binomial_sf(wins, effective) if effective else 1.0}


def ratio(contact, control):
    """median(contact) / median(control), or ``None`` if either is empty/zero."""
    c = describe([x for x in contact if x is not None])
    k = describe([x for x in control if x is not None])
    if not c.get('n') or not k.get('n') or not k['median']:
        return None
    return c['median'] / k['median']


# ── 3. The pre-registered outcome ─────────────────────────────────────────────

def channel_verdict(contact, control, cadence):
    """-> the per-channel verdict dict the outcome rule reads.

    ``separates`` is decided by the CONTROL comparison alone — median ratio
    >= 1.5 and a paired sign test that rejects at 5 %. ``cadence_starved`` is
    reported alongside but does NOT gate the verdict: a channel can be fast and
    blind (``vel_meas``) or slow and informative, and collapsing the two
    questions into one number is how the naive floor comparison went wrong.
    """
    test = sign_test(contact, control)
    r = ratio(contact, control)
    sp = cadence.get('spacing_ms_p50')
    return {
        'contact': describe([x for x in contact if x is not None]),
        'control': describe([x for x in control if x is not None]),
        'median_ratio': r,
        'sign_test': test,
        'separates': bool(r is not None and r >= SEPARATION_RATIO_MIN
                          and test['p_one_sided'] < SEPARATION_ALPHA),
        'spacing_ms_p50': sp,
        'cadence_starved': bool(
            sp is None or sp > CADENCE_SPACING_MAX_MS * _CADENCE_SLACK),
        'cadence_requirement_ms': CADENCE_SPACING_MAX_MS,
    }


def select_outcome(vel_verdict, iq_verdict):
    """-> ``(code, headline, reasons)``. The pre-registered rule, in code.

    Plan § Phase 0b enumerates three outcomes: (i) resolvable, (ii) resolvable in
    neither, (iii) resolvable in ``vel_meas`` but cadence-starved in ``iq_meas``.
    A fourth case is *logically available and not enumerated* — ``iq_meas``
    separating while ``vel_meas`` does not — so this function can return it, and
    returns it loudly rather than rounding it into one of the three. Forcing an
    unenumerated result into the nearest pre-registered box is how a
    pre-registration stops being one.
    """
    v, q = vel_verdict['separates'], iq_verdict['separates']
    reasons = [
        'vel_meas: median ratio {} vs control, sign test {}/{} wins, p={:.4g}'
        .format('n/a' if vel_verdict['median_ratio'] is None
                else '{:.2f}x'.format(vel_verdict['median_ratio']),
                vel_verdict['sign_test']['n_wins'],
                vel_verdict['sign_test']['n_pairs'],
                vel_verdict['sign_test']['p_one_sided']),
        'iq_meas: median ratio {} vs control, sign test {}/{} wins, p={:.4g}'
        .format('n/a' if iq_verdict['median_ratio'] is None
                else '{:.2f}x'.format(iq_verdict['median_ratio']),
                iq_verdict['sign_test']['n_wins'],
                iq_verdict['sign_test']['n_pairs'],
                iq_verdict['sign_test']['p_one_sided']),
        'cadence: vel_meas transitions {} ms apart, iq_meas {} ms apart '
        '(a {:.0f} ms contact needs <= {:.1f} ms to integrate)'.format(
            'n/a' if vel_verdict['spacing_ms_p50'] is None
            else '{:.1f}'.format(vel_verdict['spacing_ms_p50']),
            'n/a' if iq_verdict['spacing_ms_p50'] is None
            else '{:.1f}'.format(iq_verdict['spacing_ms_p50']),
            IMPACT_DURATION_MS, CADENCE_SPACING_MAX_MS),
    ]
    if v and q:
        code, headline = OUTCOME_I
    elif v and not q:
        code, headline = OUTCOME_III
    elif q and not v:
        code, headline = OUTCOME_UNREGISTERED
    else:
        code, headline = OUTCOME_II
    return code, headline, reasons


# ── Bag reading: the miner for labels, one light pass for the drive ───────────

def read_drive(path, log_minus_stamp_s):
    """-> :class:`DriveSeries` on the SAME clock the miner puts its edges on.

    A second pass over ``/hand_telemetry`` rather than an index-join against the
    miner's ``SensorSample`` list: the join would be by position, and two decodes
    that disagree by one message on a tie would silently shift every drive sample
    against its sensor bit. Re-deriving ``t`` from the header stamp is
    order-independent and cannot mis-align.

    The header-stamp-else-``log_time`` rule is **called, not restated**:
    ``miner.hand_sample_time`` is the one home for it (audit fix 2026-08-12 — it
    was a second copy here). If the two ever disagreed, every drive metric in
    this file would be measured at an instant the sensor edges are not at, and
    nothing would say so.
    """
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    files = sorted(glob.glob(os.path.join(path, '*.mcap')))
    if not files:
        raise IOError('no .mcap in {}'.format(path))
    rows = []
    for f in files:
        with open(f, 'rb') as fh:
            reader = make_reader(fh, decoder_factories=[DecoderFactory()])
            for _sch, _ch, msg, dec in reader.iter_decoded_messages(
                    topics=['/hand_telemetry']):
                stamp = (float(dec.timestamp.sec)
                         + float(dec.timestamp.nanosec) * 1e-9)
                t = miner.hand_sample_time(stamp, msg.log_time / 1e9,
                                           log_minus_stamp_s)
                rows.append((t, float(dec.vel_meas), float(dec.iq_meas),
                             float(dec.vel_ff_cmd)))
    if not rows:
        raise IOError('no /hand_telemetry in {}'.format(path))
    rows.sort()
    return DriveSeries([r[0] for r in rows], [r[1] for r in rows],
                       [r[2] for r in rows], [r[3] for r in rows])


# ── The per-bag analysis ──────────────────────────────────────────────────────

def analyse(series, arrivals, all_edges, held_spans, cross_label=None):
    """-> the whole evidence dict for one bag. PURE — no bag reader, no ROS.

    ``arrivals``    RAW empty->held instants (the miner's edges).
    ``all_edges``   every sensor transition, for the quiet-span guard.
    ``held_spans``  ball-held intervals, for the noise floor.
    ``cross_label`` optional ``{'caught': [t, ...], 'missed': [t, ...]}`` of
                    MATCHED-PHASE anchors — the same instant of the cycle on a
                    toss that was caught and one that was not. This is the only
                    control that varies the ball and holds everything else, so
                    when it exists it is the one quoted first.
    """
    contact = [(t - CONTACT_PRE_S, t + CONTACT_POST_S) for t in arrivals]
    control = [(t - CONTROL_LEAD_S, t - CONTROL_LAG_S) for t in arrivals]
    guarded = quiet_spans(held_spans, all_edges)

    out = {
        'n_arrivals': len(arrivals),
        'contact_window_s': [-CONTACT_PRE_S, CONTACT_POST_S],
        'control_window_s': [-CONTROL_LEAD_S, -CONTROL_LAG_S],
        'cadence': {
            'contact': {c: cadence_census(series, contact, c)
                        for c in ('vel', 'iq')},
            'quiet_held': {c: cadence_census(series, guarded, c)
                           for c in ('vel', 'iq')},
            # +_EPS_S so the LAST sample is inside the whole-bag interval:
            # `span` bisects LEFT on the upper bound, so `(t[0], t[-1])` stops
            # one sample short and the whole-bag census silently dropped the
            # final transition of every bag.
            'whole_bag': {c: cadence_census(
                series, [(series.t[0], series.t[-1] + _EPS_S)] if series.t
                else [], c)
                for c in ('vel', 'iq')},
        },
        'noise_floor': noise_floor(series, guarded),
    }

    paired = _paired_values(series, arrivals, guarded)
    # Carried on the result so `pool` re-derives the pooled verdict from the
    # per-catch VALUES rather than averaging per-bag verdicts. Computed here,
    # once, so the pooled numbers cannot be built from a different baseline or a
    # different overlap exclusion than the per-bag ones — which is what a second
    # call site next to `analyse_bag` used to allow.
    out['_paired'] = paired
    dv_c, dv_k = paired['dv_c'], paired['dv_k']
    res_c, res_k = paired['res_c'], paired['res_k']
    iq_c, iq_k, ntr = paired['iq_c'], paired['iq_k'], paired['ntr']
    out['n_control_overlap_excluded'] = paired['n_overlap_excluded']
    out['n_no_baseline'] = paired['n_no_baseline']

    out['within_cycle_control'] = {
        'peak_abs_dvel': channel_verdict(
            dv_c, dv_k, out['cadence']['contact']['vel']),
        'peak_command_residual': channel_verdict(
            res_c, res_k, out['cadence']['contact']['vel']),
        'iq_impulse': channel_verdict(
            iq_c, iq_k, out['cadence']['contact']['iq']),
    }
    out['iq_transitions_per_contact_window'] = describe(ntr)
    out['softness_thinning'] = _thinning(dv_c, ntr)

    if cross_label and cross_label.get('caught') and cross_label.get('missed'):
        out['_xlabel'] = cross_label_values(series, cross_label, guarded)
        out['cross_label_control'] = cross_label_verdicts(
            out['_xlabel'], out['cadence']['contact'])
        src = 'cross_label'
        vel_v = out['cross_label_control']['peak_abs_dvel']
        iq_v = out['cross_label_control']['iq_impulse']
    else:
        src = 'within_cycle'
        vel_v = out['within_cycle_control']['peak_abs_dvel']
        iq_v = out['within_cycle_control']['iq_impulse']
    code, headline, reasons = select_outcome(vel_v, iq_v)
    out['verdict'] = {'control_used': src, 'outcome': code,
                      'headline': headline, 'reasons': reasons}
    return out


def _thinning(peaks, n_transitions):
    """Does the ``iq_meas`` refresh THIN as the catch softens? (plan risk 2)

    Tertiles of contact severity vs the distinct-transition count in the same
    window. The plan predicts the on-change gate opens less often exactly in the
    soft regime the whole channel is wanted for; this measures it.
    """
    pairs = sorted((p, n) for p, n in zip(peaks, n_transitions) if p is not None)
    if len(pairs) < 6:
        return {'n': len(pairs)}
    third = len(pairs) // 3
    soft = [n for _p, n in pairs[:third]]
    harsh = [n for _p, n in pairs[-third:]]
    return {'n': len(pairs),
            'softest_third_iq_transitions': describe(soft),
            'harshest_third_iq_transitions': describe(harsh)}


def cross_label_values(series, anchors, guarded=()):
    """-> ``{'caught': {metric: [...]}, 'missed': {...}}`` raw metric values.

    Split out from :func:`cross_label_verdicts` so the POOLED verdict can
    concatenate the values across bags and re-derive the same verdict, instead of
    averaging per-bag verdicts (which would weight a 1-catch bag like a 38-catch
    one) or — worse — falling back to the weaker within-cycle control and
    contradicting the per-bag headline.

    This path is structurally immune to the within-cycle baseline defect — the
    CAUGHT and MISSED populations are treated identically, so a baseline that
    favours one window favours it on both sides and cancels — but it uses the
    same :func:`quiet_baseline` anyway, so the corpus carries ONE definition of
    "the current this catch is measured against" rather than two.
    """
    got = {}
    for label in ('caught', 'missed'):
        dv, iq = [], []
        for t in anchors[label]:
            clo, chi = t - CONTACT_PRE_S, t + CONTACT_POST_S
            base = quiet_baseline(series, guarded, t)
            dv.append(peak_abs_step(series, clo, chi, 'vel'))
            iq.append(iq_impulse(series, clo, chi, base) if base is not None
                      else None)
        got[label] = {'peak_abs_dvel': dv, 'iq_impulse': iq}
    return got


def cross_label_verdicts(values, cadence=None):
    """CAUGHT vs MISSED at the SAME phase of the cycle — the one control that
    varies the ball and holds the choreography fixed.

    Unpaired (a caught toss and a missed one are different trials), so the sign
    test does not apply; the verdict rests on the median ratio, and the sign-test
    slot is filled with the degenerate n=0 result so the shape stays uniform for
    :func:`select_outcome`.
    """
    cadence = cadence or {}
    out = {}
    for metric, chan in (('peak_abs_dvel', 'vel'), ('iq_impulse', 'iq')):
        c = [x for x in values['caught'][metric] if x is not None]
        m = [x for x in values['missed'][metric] if x is not None]
        dc, dm = describe(c), describe(m)
        r = (dc['median'] / dm['median']
             if dc.get('n') and dm.get('n') and dm['median'] else None)
        sp = (cadence.get(chan) or {}).get('spacing_ms_p50')
        out[metric] = {
            'contact': dc, 'control': dm, 'median_ratio': r,
            'sign_test': {'n_pairs': 0, 'n_wins': 0, 'n_ties': 0,
                          'p_one_sided': 1.0},
            'separates': bool(r is not None and r >= SEPARATION_RATIO_MIN),
            'spacing_ms_p50': sp,
            'cadence_starved': bool(
            sp is None or sp > CADENCE_SPACING_MAX_MS * _CADENCE_SLACK),
            'cadence_requirement_ms': CADENCE_SPACING_MAX_MS,
            'note': 'unpaired CAUGHT vs MISSED at matched phase; the sign test '
                    'does not apply, the median ratio decides',
        }
    return out


def held_spans_from(samples):
    """-> ball-held intervals, from the miner's own debounced edges."""
    rises, falls = edges(samples, debounced=True)
    merged = sorted([(t, True) for t in rises] + [(t, False) for t in falls])
    spans = []
    open_t = None
    for t, is_rise in merged:
        if is_rise:
            open_t = t
        elif open_t is not None:
            spans.append((open_t, t))
            open_t = None
    valid = [s for s in samples if s.valid]
    if open_t is not None and valid:
        spans.append((open_t, valid[-1].t))
    return spans


def cross_label_anchors(rows):
    """-> matched-phase anchors from the miner's LABELS, or ``None``.

    Both populations are anchored at ``landing_time + median(catch_dt)`` — the
    same instant of the cycle for a toss that was caught and one that was not.
    Anchoring the caught population on its own arrival edge instead would give
    the two populations different phase distributions, and the comparison would
    measure that difference rather than the ball.
    """
    caught = [r for r in rows
              if r.get('label') == 'CAUGHT' and r.get('t_catch_raw_ros')]
    missed = [r for r in rows if r.get('label') == 'MISSED']
    if not caught or not missed:
        return None
    dts = sorted(r['t_catch_raw_ros'] - r['announce_landing_time_ros']
                 for r in caught)
    med = dts[len(dts) // 2]
    return {
        'median_catch_dt_s': med,
        'caught': [r['announce_landing_time_ros'] + med for r in caught],
        'missed': [r['announce_landing_time_ros'] + med for r in missed],
    }


# ── Reporting ─────────────────────────────────────────────────────────────────

def _fmt(d, key='median', unit=''):
    if not d or not d.get('n'):
        return '-'
    return '{:.4g}{}'.format(d[key], unit)


def print_report(name, res):
    print('\n=== {}'.format(name))
    print('  arrivals analysed: {}'.format(res['n_arrivals']))
    if not res['n_arrivals']:
        print('  (no catches in this bag — nothing to census)')
        return
    print('\n  -- 1. CADENCE CENSUS (distinct-value transitions, not samples) --')
    print('  {:12} {:>10} {:>12} {:>10} {:>12} {:>12}'.format(
        'window', 'channel', 'trans/s', 'dup frac', 'spacing p50', 'spacing p95'))
    for win in ('contact', 'quiet_held', 'whole_bag'):
        for ch in ('vel', 'iq'):
            c = res['cadence'][win][ch]
            print('  {:12} {:>10} {:>12} {:>10} {:>12} {:>12}'.format(
                win, ch + '_meas',
                '-' if c['transitions_per_s'] is None
                else '{:.2f}'.format(c['transitions_per_s']),
                '-' if c['duplicate_frac'] is None
                else '{:.1%}'.format(c['duplicate_frac']),
                '-' if c['spacing_ms_p50'] is None
                else '{:.1f} ms'.format(c['spacing_ms_p50']),
                '-' if c['spacing_ms_p95'] is None
                else '{:.1f} ms'.format(c['spacing_ms_p95'])))
    iqw = res['iq_transitions_per_contact_window']
    print('  iq_meas distinct transitions per {:.0f} ms contact window: '
          'min {} med {} max {}'.format(
              (CONTACT_PRE_S + CONTACT_POST_S) * 1e3,
              _fmt(iqw, 'min'), _fmt(iqw, 'median'), _fmt(iqw, 'max')))
    th = res['softness_thinning']
    if th.get('n', 0) >= 6:
        print('  thinning check — softest third: {} iq transitions; '
              'harshest third: {}'.format(
                  _fmt(th['softest_third_iq_transitions']),
                  _fmt(th['harshest_third_iq_transitions'])))

    nf = res['noise_floor']
    print('\n  -- 2. NOISE FLOOR --')
    print('  recipe: {}'.format(nf['recipe']))
    for kind in ('static', 'moving'):
        d = nf[kind]
        print('    {:7} |dvel| n={} p50 {} p95 {} p99 {} max {} rev/s'.format(
            kind, d.get('n', 0), _fmt(d, 'median'), _fmt(d, 'p95'),
            _fmt(d, 'p99'), _fmt(d, 'max')))

    print('\n  -- 3. TRANSIENT, against MATCHED CONTROLS --')
    for src in ('within_cycle_control', 'cross_label_control'):
        if src not in res:
            continue
        print('  [{}]'.format(src))
        for metric, v in sorted(res[src].items()):
            print('    {:22} contact med {:>9}  control med {:>9}  '
                  'ratio {:>7}  sign {}/{} p={:.4g}  {}'.format(
                      metric, _fmt(v['contact']), _fmt(v['control']),
                      '-' if v['median_ratio'] is None
                      else '{:.2f}x'.format(v['median_ratio']),
                      v['sign_test']['n_wins'], v['sign_test']['n_pairs'],
                      v['sign_test']['p_one_sided'],
                      'SEPARATES' if v['separates'] else 'no separation'))

    ver = res['verdict']
    print('\n  -- VERDICT (control used: {}) --'.format(ver['control_used']))
    print('  PRE-REGISTERED OUTCOME ({}): {}'.format(
        ver['outcome'], ver['headline']))
    for r in ver['reasons']:
        print('    - {}'.format(r))


def pool(results):
    """Pooled verdict over every analysed bag.

    Pooling is on the per-catch METRIC VALUES, not on the per-bag verdicts: most
    sittings carry a handful of catches, and a majority vote over per-bag
    verdicts would weight a 1-catch bag equally with a 38-catch one.

    **The pooled verdict uses the SAME control the per-bag verdicts use.** An
    earlier draft pooled only the within-cycle pairs and so announced outcome (i)
    directly underneath a per-bag cross-label verdict of (ii) — the pooled
    headline silently reverting to the weaker control. Both controls are still
    reported; only the strongest available one selects the outcome.
    """
    dv_c, dv_k, res_c, res_k, iq_c, iq_k, ntr = [], [], [], [], [], [], []
    n_overlap = n_no_base = 0
    xl = {'caught': {'peak_abs_dvel': [], 'iq_impulse': []},
          'missed': {'peak_abs_dvel': [], 'iq_impulse': []}}
    for r in results.values():
        w = r.get('_paired')
        if w:
            dv_c += w['dv_c']; dv_k += w['dv_k']
            res_c += w['res_c']; res_k += w['res_k']
            iq_c += w['iq_c']; iq_k += w['iq_k']
            ntr += w['ntr']
            n_overlap += w.get('n_overlap_excluded', 0)
            n_no_base += w.get('n_no_baseline', 0)
        x = r.get('_xlabel')
        if x:
            for label in ('caught', 'missed'):
                for metric in ('peak_abs_dvel', 'iq_impulse'):
                    xl[label][metric] += x[label][metric]
    if not dv_c:
        return None
    cad = {'vel': {'spacing_ms_p50': _pooled_spacing(results, 'vel')},
           'iq': {'spacing_ms_p50': _pooled_spacing(results, 'iq')}}
    out = {
        'n_arrivals': len(dv_c),
        'n_control_overlap_excluded': n_overlap,
        'n_no_baseline': n_no_base,
        'within_cycle_control': {
            'peak_abs_dvel': channel_verdict(dv_c, dv_k, cad['vel']),
            'peak_command_residual': channel_verdict(res_c, res_k, cad['vel']),
            'iq_impulse': channel_verdict(iq_c, iq_k, cad['iq']),
        },
        'iq_transitions_per_contact_window': describe(ntr),
        'softness_thinning': _thinning(dv_c, ntr),
    }
    have_xl = bool(xl['caught']['peak_abs_dvel'] and xl['missed']['peak_abs_dvel'])
    if have_xl:
        out['cross_label_control'] = cross_label_verdicts(xl, cad)
        src = 'cross_label (pooled)'
        vel_v = out['cross_label_control']['peak_abs_dvel']
        iq_v = out['cross_label_control']['iq_impulse']
    else:
        src = 'within_cycle (pooled)'
        vel_v = out['within_cycle_control']['peak_abs_dvel']
        iq_v = out['within_cycle_control']['iq_impulse']
    code, headline, reasons = select_outcome(vel_v, iq_v)
    out['verdict'] = {'control_used': src, 'outcome': code,
                      'headline': headline, 'reasons': reasons}
    return out


def _pooled_spacing(results, channel):
    vals = [r['cadence']['contact'][channel]['spacing_ms_p50']
            for r in results.values()
            if r.get('n_arrivals') and r['cadence']['contact'][channel]
            .get('spacing_ms_p50') is not None]
    return sorted(vals)[len(vals) // 2] if vals else None


# ── Self-check (synthetic; the right answer is known by construction) ─────────

#: The synthetic command's period. It is EXACTLY the contact->control offset —
#: the two windows have the same duration and are shifted by
#: ``CONTROL_LEAD_S - CONTACT_PRE_S`` = ``CONTACT_POST_S + CONTROL_LAG_S`` =
#: 0.70 s at both ends — so the control window sees a commanded profile IDENTICAL
#: to the contact window's. (The second expression used to read
#: ``CONTROL_LAG_S - CONTACT_POST_S``, which is 0.30 s and is not an offset of
#: anything; the code was always right.) Without that the generator injects the
#: very confound the
#: probe exists to detect and the zero-impact case "separates" on the command
#: alone — which is exactly what the first draft of this self-check did.
_SYNTH_CMD_PERIOD_S = CONTROL_LEAD_S - CONTACT_PRE_S


def _synth(impact_rev_s, *, iq_period_s=0.033, dt=0.01, n_events=12,
           iq_impact_a=0.0, seed=1):
    """A synthetic drive series with a KNOWN answer.

    A PERIODIC commanded stroke the measurement follows exactly (so ``vel_meas``
    really is the command, the way the hardware behaves), a deterministic
    pseudo-noise floor, ``iq_meas`` refreshed on a fixed period (the
    cache-republish shape), and an injected impact of ``impact_rev_s`` at each
    event instant. With ``impact_rev_s = 0`` the contact and control windows are
    identical up to noise and the probe MUST report no separation; with a large
    impact it MUST find it. Both directions are checked, because an instrument
    that only ever says "resolvable" is not one.
    """
    t, vel, iq, vff = [], [], [], []
    events = [20.0 + 8 * _SYNTH_CMD_PERIOD_S * k for k in range(n_events)]
    rng = seed
    n = int(events[-1] / dt) + 400
    last_iq_t = -1e9
    iq_val = 0.0
    for i in range(n):
        ti = i * dt
        rng = (1103515245 * rng + 12345) % 2147483648
        noise = ((rng / 2147483648.0) - 0.5) * 0.08
        # A commanded stroke, periodic at exactly the contact->control offset:
        # down-ramp, up-ramp, then a DWELL at zero. The dwell is what gives the
        # static noise floor a population at all — without a quiescent stretch
        # the "hand is still" floor is empty, which is itself the honest answer
        # for a hand that never stops.
        phase = (ti % _SYNTH_CMD_PERIOD_S) / _SYNTH_CMD_PERIOD_S
        if phase < 0.4:
            cmd = -40.0 * phase / 0.4
        elif phase < 0.8:
            cmd = -40.0 * (1.0 - (phase - 0.4) / 0.4)
        else:
            cmd = 0.0
        v = cmd + noise
        for e in events:
            if e <= ti < e + 0.02:
                v += impact_rev_s
        if ti - last_iq_t >= iq_period_s:
            last_iq_t = ti
            iq_val = 0.05 * cmd + noise
            for e in events:
                if e <= ti < e + 0.05:
                    iq_val += iq_impact_a
        t.append(ti); vel.append(v); iq.append(iq_val); vff.append(cmd)
    return DriveSeries(t, vel, iq, vff), events


def self_check() -> int:
    """Two-sided by construction: every DETECT case has a REFUSE twin.

    The refuse twins are the load-bearing half. A probe that reports "resolvable"
    on a series where the only thing present is the command it already knows
    about is the exact failure this phase exists to avoid, so the zero-impact
    case must select outcome (ii) and does.
    """
    fails = []
    checks = [0]

    def check(name, got, want):
        checks[0] += 1
        if got != want:
            fails.append('{}: got {!r}, want {!r}'.format(name, got, want))

    def run(impact, iq_impact=0.0, iq_period=0.033):
        series, events = _synth(impact, iq_impact_a=iq_impact,
                                iq_period_s=iq_period)
        held = [(e - 3.0, e + 3.0) for e in events]
        return analyse(series, events, events, held)

    # REFUSE: no impact at all -> nothing to resolve, in either channel.
    r0 = run(0.0)
    check('a series with NO impact selects outcome (ii)',
          r0['verdict']['outcome'], 'ii')
    check('no impact -> vel does not separate',
          r0['within_cycle_control']['peak_abs_dvel']['separates'], False)
    check('no impact -> iq does not separate',
          r0['within_cycle_control']['iq_impulse']['separates'], False)

    # THE NEUTRAL-BASELINE CASE. Two windows drawn from the SAME distribution
    # must not produce a significant paired sign test — an instrument whose
    # reference is one of the two things being compared cannot deliver that,
    # because the median MINIMISES its own window's L1 deviation and the control
    # therefore starts the comparison at its own floor. The shared reference now
    # comes from a disjoint guarded quiet-held span (`quiet_baseline`).
    #
    # ``iq_period=0.05`` is load-bearing and it is not tuning: the sample grid is
    # 10 ms and the contact->control offset is 0.70 s, so a 50 ms refresh puts
    # the two windows on the SAME staircase alignment (14 refreshes per offset).
    # At the generator's default 33 ms — which the dt grid rounds to 40 ms, and
    # 0.70/0.04 = 17.5 — the two windows sample the staircase differently and the
    # null case carries a deterministic ~3 % offset with a significant sign test.
    # That residual is a property of the GENERATOR, not of the estimator: it is
    # present with either baseline recipe (measured 2026-08-12, 10/12 wins and
    # p = 0.0193 both ways), so it must not be read as evidence about the
    # baseline in either direction.
    null_iq = run(0.0, iq_period=0.05)['within_cycle_control']['iq_impulse']
    check('two statistically identical windows do NOT reject the sign test',
          null_iq['sign_test']['p_one_sided'] >= SEPARATION_ALPHA, True)
    check('...and their median ratio is ~1, not the baseline\'s artefact',
          0.9 <= null_iq['median_ratio'] <= 1.1, True)
    check('...on a real pair population, not a degenerate all-ties one',
          null_iq['sign_test']['n_pairs'] - null_iq['sign_test']['n_ties'] >= 8,
          True)

    # DETECT: a large velocity impact and no iq signature -> outcome (iii).
    r1 = run(12.0)
    check('a 12 rev/s velocity impact separates on vel',
          r1['within_cycle_control']['peak_abs_dvel']['separates'], True)
    check('a velocity-only impact selects outcome (iii)',
          r1['verdict']['outcome'], 'iii')

    # DETECT both -> outcome (i).
    r2 = run(12.0, iq_impact=8.0, iq_period=0.005)
    check('velocity AND current impacts select outcome (i)',
          r2['verdict']['outcome'], 'i')

    # The UNENUMERATED case is reported, not rounded into (i)/(ii)/(iii).
    r3 = run(0.0, iq_impact=8.0, iq_period=0.005)
    check('an iq-only impact is reported as UNREGISTERED',
          r3['verdict']['outcome'], 'UNREGISTERED')

    # Cadence census: an iq cache refreshed every 100 ms must census at ~10/s,
    # NOT at the 100 Hz sample rate. This is the whole point of the census.
    slow, events = _synth(0.0, iq_period_s=0.10)
    cen = cadence_census(slow, [(10.0, 60.0)], 'iq')
    check('a 100 ms iq cache censuses at ~10 transitions/s',
          9.0 <= cen['transitions_per_s'] <= 11.0, True)
    # ~90 %, and the arithmetic is checkable by eye: iq refreshes once every 10
    # samples at dt = 0.01, so 9 of every 10 consecutive pairs are duplicates.
    # The case used to be NAMED "~99 %" while asserting 0.85-0.95 — a name that
    # disagrees with its own bound is a name that will be believed over the code.
    check('the same stream censuses ~90 % duplicate samples',
          0.85 <= cen['duplicate_frac'] <= 0.95, True)
    cenv = cadence_census(slow, [(10.0, 60.0)], 'vel')
    check('vel_meas on the same stream censuses near the sample rate',
          cenv['transitions_per_s'] > 90.0, True)
    # The WHOLE-BAG census has to reach the last sample. `span` bisects LEFT on
    # the upper bound — right for back-to-back windows, wrong for "the whole
    # bag", where it silently dropped the final sample and its transition.
    tiny = DriveSeries([0.0, 0.01, 0.02], [0.0] * 3, [0.0, 1.0, 2.0], [0.0] * 3)
    wb = analyse(tiny, [], [], [])['cadence']['whole_bag']['iq']
    check('the whole-bag census reaches the LAST sample',
          (wb['n_samples'], wb['n_transitions']), (3, 2))

    # The exact sign test, against hand-computable values.
    check('sign test 17/19 one-sided p', round(binomial_sf(17, 19), 6), 0.000364)
    check('sign test 17/19 is EXACT, to the bit', binomial_sf(17, 19),
          191.0 / 2 ** 19)
    check('sign test 9/19 is not significant',
          binomial_sf(9, 19) > 0.5, True)
    check('sign test on a perfect sweep', binomial_sf(5, 5), 0.03125)
    # A pooled corpus reaches four figures of pairs, and the float form of this
    # function raised `OverflowError: int too large to convert to float` from
    # n = 1030 (an exact bigint C(n, i) multiplied by an underflowed 0.5**n) —
    # in the middle of the pooled report, with every per-bag verdict already
    # printed. n = 2000 is comfortably past that ceiling.
    check('a 2000-pair sign test returns instead of overflowing',
          round(binomial_sf(1000, 2000), 4), 0.5089)
    check('...and the degenerate ends stay exact',
          (binomial_sf(0, 2000), binomial_sf(2001, 2000)), (1.0, 0.0))

    # The pooled verdict must select on the SAME control the per-bag verdicts
    # use. Regression guard for a real defect found while running this probe on
    # the reference bag: pooling only the within-cycle pairs announced outcome
    # (i) directly beneath a per-bag cross-label verdict of (ii).
    series, events = _synth(12.0)
    held = [(e - 3.0, e + 3.0) for e in events]
    anchors = {'caught': events[:6], 'missed': events[6:]}
    # `analyse` now attaches `_paired` itself. It used to be a second call at
    # every call site, which is one place per site for the baseline and the
    # exclusion rule to differ between the per-bag and the pooled numbers.
    one = analyse(series, events, events, held, anchors)
    pooled = pool({'synthetic': one})
    check('the per-bag result carries its own paired values',
          bool(one.get('_paired')), True)
    check('the pooled verdict uses the cross-label control when one exists',
          pooled['verdict']['control_used'], 'cross_label (pooled)')
    check('the pooled and per-bag verdicts agree on one corpus',
          pooled['verdict']['outcome'], one['verdict']['outcome'])

    # A CONTROL WINDOW THAT OVERLAPS ANOTHER ARRIVAL'S CONTACT WINDOW is not a
    # control — during a rattle the "no ball in the cup yet" window is measuring
    # a contact. The bias runs conservative (it inflates the control), which is
    # why it never showed up as a false positive; the pair still does not mean
    # what the column says, so it is dropped and COUNTED.
    rattled = sorted(list(events) + [events[3] - 0.6])
    pv = _paired_values(series, rattled, quiet_spans(held, events))
    check('a control window overlapping another contact is excluded',
          pv['n_overlap_excluded'], 1)
    check('...as a PAIR, so neither population is left half-counted',
          [i for i, x in enumerate(pv['dv_c']) if x is None],
          [i for i, x in enumerate(pv['dv_k']) if x is None])
    check('...and the clean pairs all survive',
          sum(1 for x in pv['dv_c'] if x is not None), len(rattled) - 1)

    # A floor measured on a STILL hand is not the floor a moving one must beat —
    # the trap this probe exists to avoid, pinned as a shape.
    series, events = _synth(0.0)
    nf = noise_floor(series, [(events[0] - 3.0, events[0] - 1.0)])
    check('both floors are reported, separately',
          nf['static']['n'] > 0 and nf['moving']['n'] > 0, True)
    check('the static floor is the SMALLER one (it excludes the stroke)',
          nf['static']['max'] < nf['moving']['max'], True)

    for f in fails:
        print('FAIL  {}'.format(f))
    print('{}/{} self-check cases pass'.format(checks[0] - len(fails), checks[0]))
    return 1 if fails else 0


# ── CLI ───────────────────────────────────────────────────────────────────────

def analyse_bag(path, robot='jugglebot'):
    """-> ``(result, note)``. Raises ``IOError`` on an unreadable bag."""
    data = miner.read_bag(path, sensor_only=True)
    rows = miner.mine_bag(data, robot=robot)
    rises_raw, _falls_raw = edges(data.hand, debounced=False)
    rises_deb, falls_deb = edges(data.hand, debounced=True)
    all_edges = sorted(list(rises_deb) + list(falls_deb))
    series = read_drive(path, data.log_minus_stamp_s)
    anchors = cross_label_anchors(rows)
    res = analyse(series, list(rises_raw), all_edges,
                  held_spans_from(data.hand), anchors)
    n = len(data.hand)
    n_valid = sum(1 for s in data.hand if s.valid)
    res['sensor'] = {
        'n_samples': n, 'n_valid': n_valid,
        'valid_frac': (float(n_valid) / n) if n else 0.0,
        'n_catches_debounced': len(rises_deb),
        'n_catches_raw': len(rises_raw),
        'n_self_toss_rows': len(rows),
        'median_catch_dt_s': (anchors or {}).get('median_catch_dt_s'),
    }
    note = None
    if n_valid < n:
        note = ('ball_held_valid false on {}/{} samples ({:.1%}) — the edge '
                'rule SKIPS invalid samples, so those spans contribute no '
                'arrivals'.format(n - n_valid, n, 1.0 - res['sensor']['valid_frac']))
    return res, note


def control_overlaps_a_contact(arrivals, i):
    """Does catch ``i``'s CONTROL window overlap ANOTHER arrival's contact
    window?

    The control is a fixed 0.5-0.8 s before the arrival, and during a rattle (or
    a reload landing right before a catch) a previous RAW arrival's contact
    window lands inside it — so the "no ball in the cup yet" control is measuring
    a contact. The bias is conservative (it inflates the control and shrinks the
    ratio), which is why it was not visible as a false positive; it is still a
    pair that does not mean what the column header says, so it is dropped and
    counted rather than left in with a note.
    """
    klo, khi = arrivals[i] - CONTROL_LEAD_S, arrivals[i] - CONTROL_LAG_S
    for j, t in enumerate(arrivals):
        if j == i:
            continue
        if (t - CONTACT_PRE_S) < khi and (t + CONTACT_POST_S) > klo:
            return True
    return False


def _paired_values(series, arrivals, guarded=()):
    """The per-catch contact/control metric pairs, and the pairs REFUSED.

    ``guarded`` is the quiet-held population :func:`quiet_baseline` draws the
    shared ``iq`` reference from. It defaults to empty only so a caller that
    genuinely has no sensor spans still gets the velocity metrics; with no
    guarded span there is no neutral baseline, every ``iq`` pair is ``None``, and
    ``n_no_baseline`` says so rather than the corpus quietly carrying a
    window-owned reference again.
    """
    arrivals = list(arrivals)
    dv_c, dv_k, res_c, res_k, iq_c, iq_k, ntr = [], [], [], [], [], [], []
    n_overlap = n_no_base = 0
    for i, t in enumerate(arrivals):
        clo, chi = t - CONTACT_PRE_S, t + CONTACT_POST_S
        klo, khi = t - CONTROL_LEAD_S, t - CONTROL_LAG_S
        if control_overlaps_a_contact(arrivals, i):
            n_overlap += 1
            for lst in (dv_c, dv_k, res_c, res_k, iq_c, iq_k):
                lst.append(None)          # the PAIR goes, not just one half
            ntr.append(distinct_transitions(series, clo, chi, 'iq'))
            continue
        base = quiet_baseline(series, guarded, t)
        if base is None:
            n_no_base += 1
        dv_c.append(peak_abs_step(series, clo, chi, 'vel'))
        dv_k.append(peak_abs_step(series, klo, khi, 'vel'))
        res_c.append(peak_command_residual(series, clo, chi))
        res_k.append(peak_command_residual(series, klo, khi))
        iq_c.append(iq_impulse(series, clo, chi, base) if base is not None else None)
        iq_k.append(iq_impulse(series, klo, khi, base) if base is not None else None)
        ntr.append(distinct_transitions(series, clo, chi, 'iq'))
    return {'dv_c': dv_c, 'dv_k': dv_k, 'res_c': res_c, 'res_k': res_k,
            'iq_c': iq_c, 'iq_k': iq_k, 'ntr': ntr,
            'n_overlap_excluded': n_overlap, 'n_no_baseline': n_no_base}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bag', action='append', default=[],
                    help='bag directory name under --root (repeatable)')
    ap.add_argument('--all', action='store_true',
                    help='every bag under --root (sequentially — 8 GB box)')
    ap.add_argument('--match', default='',
                    help='with --all, only bags whose name contains this')
    ap.add_argument('--root', default=DEFAULT_ROOT)
    ap.add_argument('--robot', default='jugglebot')
    ap.add_argument('--json', action='store_true',
                    help='write a timestamped JSON report to temp/probes/')
    ap.add_argument('--self-check', action='store_true')
    args = ap.parse_args(argv)

    if args.self_check:
        return self_check()
    bags = list(args.bag)
    if args.all:
        bags += [os.path.basename(p)
                 for p in sorted(glob.glob(os.path.join(args.root, '*')))
                 if os.path.isdir(p) and args.match in os.path.basename(p)
                 and os.path.basename(p) not in bags]
    if not bags:
        ap.error('need --bag or --all (or --self-check)')

    results, rc = {}, 0
    for name in bags:
        try:
            res, note = analyse_bag(os.path.join(args.root, name), args.robot)
        except (IOError, OSError) as exc:
            print('ERROR  {}: {}'.format(name, exc))
            rc = 1
            continue
        if note:
            print('\nNOTE   {}: {}'.format(name, note))
        print_report(name, res)
        results[name] = res

    pooled = pool(results)
    if pooled:
        print('\n\n=== POOLED over {} bag(s), {} arrivals'.format(
            len(results), pooled['n_arrivals']))
        for src in ('within_cycle_control', 'cross_label_control'):
            if src not in pooled:
                continue
            print('  [{}]'.format(src))
            for metric, v in sorted(pooled[src].items()):
                print('    {:22} contact med {:>9}  control med {:>9}  '
                      'ratio {:>7}  sign {}/{} p={:.4g}  {}'.format(
                          metric, _fmt(v['contact']), _fmt(v['control']),
                          '-' if v['median_ratio'] is None
                          else '{:.2f}x'.format(v['median_ratio']),
                          v['sign_test']['n_wins'], v['sign_test']['n_pairs'],
                          v['sign_test']['p_one_sided'],
                          'SEPARATES' if v['separates'] else 'no separation'))
        iqw = pooled['iq_transitions_per_contact_window']
        print('  iq_meas distinct transitions per contact window: '
              'min {} med {} max {}'.format(
                  _fmt(iqw, 'min'), _fmt(iqw, 'median'), _fmt(iqw, 'max')))
        th = pooled['softness_thinning']
        if th.get('n', 0) >= 6:
            print('  thinning — softest third {} iq transitions, '
                  'harshest third {}'.format(
                      _fmt(th['softest_third_iq_transitions']),
                      _fmt(th['harshest_third_iq_transitions'])))
        print('\n  POOLED PRE-REGISTERED OUTCOME ({}) [control: {}]: {}'.format(
            pooled['verdict']['outcome'], pooled['verdict']['control_used'],
            pooled['verdict']['headline']))
        for r in pooled['verdict']['reasons']:
            print('    - {}'.format(r))

    if args.json and results:
        for r in results.values():
            r.pop('_paired', None)
            r.pop('_xlabel', None)
        os.makedirs(OUT_DIR, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        out = os.path.join(OUT_DIR,
                           'hand_contact_softness_{}.json'.format(stamp))
        with open(out, 'w') as fh:
            json.dump({'windows': {'contact_pre_s': CONTACT_PRE_S,
                                   'contact_post_s': CONTACT_POST_S,
                                   'control_lead_s': CONTROL_LEAD_S,
                                   'control_lag_s': CONTROL_LAG_S,
                                   'quiet_guard_s': QUIET_GUARD_S},
                       'criteria': {
                           'separation_ratio_min': SEPARATION_RATIO_MIN,
                           'separation_alpha': SEPARATION_ALPHA,
                           'impact_duration_ms': IMPACT_DURATION_MS,
                           'cadence_spacing_max_ms': CADENCE_SPACING_MAX_MS},
                       'bags': results, 'pooled': pooled}, fh, indent=1)
        print('\nJSON -> {}'.format(out))
    return rc


if __name__ == '__main__':
    sys.exit(main())
