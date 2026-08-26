"""THE cadence contract, pinned: **a goal the gates accept cannot abort.**

``tools/probes/cadence_rung_check.py`` is the probe; this file is the gate.  It
imports the probe rather than re-deriving anything, for the reason
``tools/probes/README.md`` gives for committing replay harnesses at all: a check
that lives only in a probe is a check nobody runs.

What it pins, in order of how much it costs to get wrong:

1. **accept-implies-flies over the whole ``(T, dwell, delay, aim)`` grid.**
   ``ABORTED_CANT_MAKE_RELEASE`` is minted in ``_step_preparing`` — the catch
   latch is UP, the announcement is out, the hand is committed — and its cleanup
   retracts the hand under a seated ball.  No goal the SESSION accepts may reach
   it from static arithmetic.  Until 2026-08-23 three published rungs of
   ``tests/hardware/session_cadence_ladder.md`` did exactly that, every cycle.
2. **the published ladder still flies** — every rung, chained and first-cycle,
   with a layer-3 artifact loaded and without.
3. **the pre-audit ladder still reds** — the regression the probe was written to
   find has to stay findable, or the probe has lost it.

Runtime note: the grid sweep drives two real FSMs at the FSM loop's measured
0.040 s period — ``toss_sequencer.NODE_LOOP_PERIOD_S``, imported by the probe,
never restated — over ~1500 grid points, which is a few seconds. Worth it for the
one class of failure that reaches the hand.

⚠ **It said "the real 0.02 s node tick" until 2026-08-26, and the probe advanced
by a local literal of that value** (audit finding B3). That is the SLEEP at the
bottom of ``_run_toss_cycle``, not what an iteration costs, and it is half what
``pre_dispatch_budget_s`` charges the same ladder. A probe whose clock runs at
half the rate the gate is charged at grants the machine lead it never has: driven
with the pre-D3 floors, the corrected probe reports **252** accept-implies-flies
violations where the old one reported **0** — including the exact
``ABORTED_CANT_MAKE_RELEASE`` at ``+0.120 s`` that bag ``2026-08-26_14-25-16``
produced twice with the hand committed.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[2]
_PROBE = _REPO / 'tools' / 'probes' / 'cadence_rung_check.py'


def _load_probe():
    """Import the probe by PATH — it lives outside any package, deliberately
    (``tools/probes/README.md``: probes are standalone scripts, not a library)."""
    spec = importlib.util.spec_from_file_location('cadence_rung_check', _PROBE)
    mod = importlib.util.module_from_spec(spec)
    sys.modules.setdefault('cadence_rung_check', mod)
    spec.loader.exec_module(mod)
    return mod


probe = _load_probe()


def test_accept_implies_flies_over_the_whole_grid():
    """THE contract. Empty list or the machine can abort with the hand
    committed."""
    violations = probe.grid_violations()
    assert violations == [], '\n'.join(
        'T={:.4f} dwell={:.4f} delay={:.4f} {} ilc_trim={}: {}'.format(
            T, dwell, delay, 'first' if aimed else 'chained', ilc, why)
        for T, dwell, delay, aimed, ilc, why in violations)


def test_every_published_rung_flies_chained_and_on_its_first_cycle():
    """The ladder is an operator runbook: an operator arms these numbers on a
    machine with a ball in the cup. Each rung is checked four ways — the session
    accept gate, the chained cycle (census-B1 skip), the first cycle (which
    commands the pre-positioning move and is granted the extra lead the node
    grants it), each with a layer-3 artifact loaded and without."""
    for name, h, dwell, delay in probe.LADDER:
        T = probe.flight_for_height(h)
        v = probe.ts.vertical_event_vel_mps(T)
        granted = max(float(delay),
                      probe.ts.min_throw_delay_for_release_s(v, True))
        for ilc in (False, True):
            assert probe.session_accepts(T, dwell, delay, ilc_trim=ilc) is None, (
                '{} rejected at the session gate (ilc_trim={})'.format(name, ilc))
        assert probe.run_cycle(T, delay, aimed=False)[0] == 'FLIES', (
            '{} does not fly on a chained cycle'.format(name))
        assert probe.run_cycle(T, granted, aimed=True)[0] == 'FLIES', (
            '{} does not fly on its first cycle'.format(name))


def test_the_pre_audit_ladder_still_reproduces_its_failure():
    """The 2026-08-22 finding, kept findable.

    Three rungs of the 78daf4b ladder could not throw a ball. What CHANGED on
    2026-08-23 is where they die: the accept-time floor now models the whole
    pre-dispatch sequence, so they are refused BEFORE anything is armed
    (``REJECTED_*``, nothing moved) instead of aborting at ``cycle_start +
    0.06 s`` with the catch latch up. That is the fix, and a test that only
    asserted "they still fail" would not see the difference."""
    failures = 0
    for name, h, dwell, delay in probe.LADDER_PRE_AUDIT:
        T = probe.flight_for_height(h)
        session = probe.session_accepts(T, dwell, delay)
        first = probe.run_cycle(T, delay, aimed=True)[0]
        if session is not None or first != 'FLIES':
            failures += 1
            # Whatever refuses it must refuse it EARLY.
            assert first == 'FLIES' or first.startswith('REJECTED_'), (
                '{} still dies mid-sequence: {}'.format(name, first))
    assert failures >= 3, (
        'the pre-audit ladder no longer reproduces its own finding')


def test_the_probe_reports_the_frontier_the_runbook_publishes():
    """The runbook's § 2.0 frontier table and this probe must agree, because the
    operator reads one and the gate runs the other.

    Pinned at the band FLOOR, which is where the frontier lives (it is monotone
    across the band). The two columns MEET there: the throw envelope refuses a
    negative speed trim at ``T = 0.4949``, so a loaded ILC artifact costs
    nothing at the fastest point."""
    T = float(probe.FLIGHT_TIME_MIN_S)
    disarmed = probe.fastest_at(T, ilc_trim=False)
    armed = probe.fastest_at(T, ilc_trim=True)
    # 54.3 / 0.4168 / 0.6101 until 2026-08-26. Owner decision D3 charges the
    # pre-dispatch sequence in the loop's measured PERIOD instead of its sleep
    # (+0.080 s on every delay floor), which costs 6.8 % of the frontier — the
    # cadence the old floor advertised was one the machine could not make, and
    # bag 2026-08-26_14-25-16 aborted two cycles proving it. The way back is a
    # cheaper tick, not a smaller floor.
    assert 60.0 / disarmed[2] == pytest.approx(50.6, abs=0.1)
    assert 60.0 / armed[2] == pytest.approx(50.6, abs=0.1)
    assert disarmed[0] == pytest.approx(0.4968, abs=5e-4)
    assert disarmed[1] == pytest.approx(0.6901, abs=5e-4)
