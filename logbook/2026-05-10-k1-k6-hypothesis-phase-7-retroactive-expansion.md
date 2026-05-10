---
title: K1–K6 reference contract — Phase 7 hypothesis retroactive expansion
type: feature
date: 2026-05-10
status: resolved
phase: "mpc-tier0-contracts — Phase 7"
related_plan: "mpc-tier0-contracts.md"
related_entries:
  - 2026-05-10-plant-interface-contract-phase-6-p3-p4-enforcement
  - 2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement
  - 2026-04-20-k1-k6-reference-feasibility-resolution
files_changed:
  - tests/sim/test_make_feasible_events.py
  - plans/active/mpc-tier0-contracts.md
  - logbook/2026-05-10-k1-k6-hypothesis-phase-7-retroactive-expansion.md
  - logbook/INDEX.md
commits:
  - <filled-after-commit>
subsystem:
  - controller
  - mpc
tags:
  - contract
  - testing
  - hypothesis
---

# K1–K6 reference contract — Phase 7 hypothesis retroactive expansion

## Summary

Phase 7 of the [mpc-tier0-contracts plan](../plans/active/mpc-tier0-contracts.md):
added four new hypothesis property tests to
[tests/sim/test_make_feasible_events.py](../tests/sim/test_make_feasible_events.py),
extending the K1–K6 property surface from two properties (the canonical
two-event K1–K6 round-trip and the no-stretch rejection contract) to six.
Each new property targets a single invariant explicitly:

| Property | Invariant | max_examples |
|----------|-----------|--------------|
| ``test_property_K4_min_span`` | K4 — output spans ≥ 50 ms; near-duplicate inputs are merged | 100 |
| ``test_property_K5_coincident_twist`` | K5 — coincident events with mismatched twists raise ``ValueError`` | 100 |
| ``test_property_K6_idempotence`` | K6 — ``_clamp_twist_in_place`` is idempotent and leaves angular components untouched | 150 |
| ``test_property_K1_K6_multi_event`` | K1–K6 (incl. K2/K3 cascade) hold for 3..6-event proposals — exercises the multi-segment cascade-shift at `target.py:511-526` | 150 |

All four pass at ``max_examples=1000`` (deep validation, run locally with
``--hypothesis-seed=0`` before commit).  Committed values match the existing
two properties' convention (sad-path = 100, canonical = 150).  Phase 8 will
introduce ``ci-fast`` / ``ci-deep`` profiles and shift the per-test override
to a profile-driven default.

## Motivation

The K1–K6 reference-feasibility contract
([REFERENCE_LAYER_CONTRACT.md](../controller/REFERENCE_LAYER_CONTRACT.md))
was the structural pattern that Plan 1 emulates.  Two hypothesis properties
landed alongside the original W1–W11 cycle: ``test_property_K1_K6_two_event_proposal``
covers the canonical round-trip on a single inter-event segment, and
``test_property_no_stretch_rejects_deterministically`` pins the catch-path
rejection contract.

The plan's Phase 7 audit identified four gaps:

1. **K4** had only one scenario test
   ([test_k4_drops_near_degenerate_event](../tests/sim/test_make_feasible_events.py))
   exercising a single fixed-time geometry; no property-level coverage of the
   50 ms boundary.
2. **K5** had only one scenario test
   ([TestScenario08_TwistInconsistentAcrossBoundary](../tests/sim/test_make_feasible_events.py))
   exercising one fixed pair of mismatched twists; no property-level coverage
   that *every* mismatched pair raises.
3. **K6** had no test at all of the idempotence property — only end-to-end
   coverage via the K1 anchor and the per-event clamp.  Idempotence is what
   lets every later stage of ``make_feasible_events`` re-clamp without altering
   already-compliant boundary conditions; without a property test, a
   regression that breaks idempotence (e.g., a future contributor swapping
   ``np.clip`` for a non-idempotent saturation) would only be caught by
   downstream tests if at all.
4. **Cascade-shift** logic at [target.py:511–526](../controller/target.py)
   was untested at the property level.  The single existing canonical
   property uses two events (one segment), so any regression in the
   multi-event shift cascade — e.g., a future change that forgets to
   propagate ``delta`` to events past index ``i+1`` — would slip through.

## Design

### Property 1: K4 minimum-span

**Strategy.** A 3-event proposal with two inter-event gaps ``T1`` and ``T2``:
``T1`` ∈ [1 ms, 100 ms] straddles the 50 ms K4 boundary; ``T2`` ≥ 50 ms
ensures the third event always survives.  The straddle is deliberate —
≈ half the examples exercise the merge path (``T1 < 50 ms``), ≈ half preserve
all three events (``T1 ≥ 50 ms``).

**Assertion.** Two parts:

1. ``_assert_K4(events)`` unconditionally — output spans ≥ 50 ms regardless
   of feasibility outcome.  K4 enforcement runs at
   [target.py:463–474](../controller/target.py) *before* K2/K3 stretching,
   so K4 holds even when stretching subsequently rejects.
2. When ``T1 < 50 ms``, the middle event MUST be dropped — assert
   ``len(events) <= 2``.  This pins the *merge* contract specifically, not
   just the post-merge K4 invariant.

### Property 2: K5 coincident-twist

**Strategy.** Two events at identical time ``t_offset`` ∈ [50 ms, 2 s], same
pose, different twists.  ``t_offset`` ≥ 50 ms ensures K1 anchor at ``t_now=0``
doesn't collapse with the proposal events.

**Critical detail (audit finding).** K5's check at
[target.py:435–443](../controller/target.py) runs *after* the K6 clamp at
[target.py:419–433](../controller/target.py).  Two raw twists that differ
only in their out-of-clamp linear components collapse to identical twists
post-clamp and are NOT a K5 violation.  The property uses
``hypothesis.assume(not np.allclose(a_clamped, b_clamped))`` to filter those
cases — the precondition of the property is "twists differ post-clamp", not
"twists differ pre-clamp".

**Assertion.** ``pytest.raises(ValueError, match="K5")`` — the implementation
raises (it does not return a tuple with a reason string), and the message
names the K5 invariant.

### Property 3: K6 idempotence

**Strategy.** A random twist (±300 mm/s linear, ±300 angular per the existing
``_twist_strat``) and a clamp value ∈ [0, 500] mm/s.  Both bounds chosen so
≈ half the examples trigger clipping (clamp < |twist|) and ≈ half are
no-ops (clamp ≥ |twist|).  Clamp = 0 is intentionally permitted to exercise
the "zero-out linear" edge case.

**Assertion.** Three parts:

1. ``np.testing.assert_array_equal(once, twice)`` — applying
   ``_clamp_twist_in_place`` twice yields the same result as applying it
   once.
2. ``np.all(np.abs(once[:3]) <= clamp + 1e-12)`` — linear components are
   within ±clamp after one application.
3. ``np.testing.assert_array_equal(once[3:], twist[3:])`` — angular
   components (3..5) are never modified, regardless of clamp value.

**Implementation reach.** Tests the canonical K6 enforcement helper
``_clamp_twist_in_place`` at [target.py:254–263](../controller/target.py)
directly (imported as a module-private from ``controller.target``), not
through the public ``make_feasible_events`` boundary.  This is intentional:
the helper is the canonical enforcement point per the contract, and testing
it directly catches regressions that an end-to-end call might silently
absorb (e.g., a double-clamp inside ``make_feasible_events`` would pass an
end-to-end idempotence check without exercising the helper's idempotence).

### Property 4: K1–K6 multi-event

**Strategy.** A composite ``_multi_event_proposal()`` strategy generates
N ∈ [3, 6] events with strictly increasing times.  Each adjacent gap is
drawn from ``[_K4_MIN_SPAN_S, 1.0]`` so the input is K4-clean by
construction; the property under test is that K1–K6 hold on *output* even
after multi-segment stretching cascades through these events.

**Assertion (mirrors line 154–159 of the existing two-event property).**
``reason is None`` ⇒ K1–K6 fully compliant via ``_assert_K1_K6_compliant``;
``reason is not None`` ⇒ a non-empty machine-readable string.  This is the
"pin both feasible AND rejection contract" form per Phase 7's clarifying
question — strictly stronger than feasible-only, zero extra cost.

**What this catches.** The cascade-shift logic at
[target.py:511–526](../controller/target.py) shifts every later event time
by ``delta`` when segment ``i`` is stretched.  A regression that forgets
to propagate ``delta`` past index ``i+1`` would break K4 (some span goes
below 50 ms) or K2/K3 (the implicit gap of the un-shifted events is shorter
than feasibility allows).  The two-event property has only one segment to
shift; this property exercises the cascade across 2..5 segments.

## Implementation

### tests/sim/test_make_feasible_events.py (modified)

**Imports** ([line 22](../tests/sim/test_make_feasible_events.py#L22) and
[line 24](../tests/sim/test_make_feasible_events.py#L24)) — added
``assume`` from hypothesis, plus ``_clamp_twist_in_place`` and
``_K4_MIN_SPAN_S`` from ``controller.target``.

**Composite strategy** ``_multi_event_proposal`` at
[line 140–160](../tests/sim/test_make_feasible_events.py#L140) — drawn-N
events with K4-compliant cumulative gaps, reusing ``_pose_strat`` and
``_twist_strat``.

**Four new property tests** at
[line 217–347](../tests/sim/test_make_feasible_events.py#L217) — committed
``max_examples`` per the audit answers (sad-path = 100, canonical = 150),
all with ``deadline=None`` and ``HealthCheck.too_slow`` suppressed to match
existing convention.

### plans/active/mpc-tier0-contracts.md (modified)

Phase 7 marked ``COMPLETE (2026-05-10)`` in both the summary table and the
detailed Phase 7 heading.

## Verification

### Default-profile run (committed values)

```bash
$ pytest tests/sim/test_make_feasible_events.py -v
…
28 passed in 28.10s
```

The +4 properties land cleanly on the existing 24 tests (2 properties + ~25
scenarios).

### Deep-profile validation (max_examples=1000, seed=0)

Each new property was temporarily bumped to ``max_examples=1000`` and
re-run with ``--hypothesis-seed=0``:

```bash
$ pytest tests/sim/test_make_feasible_events.py::test_property_K4_min_span \
        tests/sim/test_make_feasible_events.py::test_property_K5_coincident_twist \
        tests/sim/test_make_feasible_events.py::test_property_K6_idempotence \
        tests/sim/test_make_feasible_events.py::test_property_K1_K6_multi_event \
        -v --hypothesis-seed=0
…
4 passed in 147.13s
```

All four pass at deep depth — exit-criterion satisfied.  The committed
``max_examples`` values were then restored to 100/100/150/150.

### Full-suite regression

- **Before Phase 7:** 1180 / 1180 pass (Phase 6 baseline).
- **After Phase 7:** **1184 / 1184 pass** — the +4 delta is exactly the
  four new properties.  Zero regressions.

## Discussion

### Why test ``_clamp_twist_in_place`` directly instead of via ``make_feasible_events``

Two formulations of K6 idempotence were considered:

1. **Direct on the helper.** Import ``_clamp_twist_in_place`` and assert
   ``clamp(clamp(x)) == clamp(x)`` element-wise.  Tests the canonical
   K6 enforcement point directly.
2. **End-to-end via the boundary.** Pass an event whose twist is already at
   the clamp boundary through ``make_feasible_events``, then pass the result
   through again, and assert equality.  Tests "the public API is idempotent".

We took option 1 for two reasons.  First, the helper is the **canonical
enforcement point** in the implementation — ``make_feasible_events`` calls
``_clamp_twist_in_place`` as its single K6 mechanism (target.py:407, 424),
so a direct test pins the helper that the contract is enforced through.
Second, option 2 has a coverage gap: a future regression that
introduces a *second* clamp call inside ``make_feasible_events`` (e.g.,
defensive double-clamp at multiple stages) would still pass an end-to-end
idempotence check, while corrupting the helper's idempotence in a way that
breaks downstream invariants we haven't yet articulated.  Option 1 is the
strictly stronger property.

The trade-off is that option 1 imports a private symbol (``_clamp_twist_in_place``,
underscore-prefixed).  This is a minor style violation — Python is "consenting
adults" but tests reaching into module internals is brittle if the helper
gets renamed.  Mitigated by the fact that any rename of
``_clamp_twist_in_place`` should also update the contract document, which
is the canonical source of the helper's name; a grep for the symbol catches
the test's import.

### Why ``hypothesis.assume`` (not strategy-bounded twists) for the K5 precondition

K5's check runs *after* K6 clamp.  Two ways to ensure the property exercises
"genuinely mismatched post-clamp twists":

1. **Strategy-bounded.** Restrict ``_twist_strat`` for this test to ±42.5
   (the minimum possible v_clamp = 0.85 × 50).  Guarantees no clamping ⇒
   raw mismatch survives.
2. **assume()-filtered.** Generate freely, then filter by post-clamp
   equality.

Option 1 is brittle: the strategy bound depends on the v_max strategy's
minimum (50), and a future widening of the v_max range would silently
narrow the test's coverage.  Option 2 lets the strategy explore the full
±300 range and only filter at the precondition boundary — hypothesis's
shrinker still finds minimal counterexamples within that filtered space.
The cost is a few wasted examples (cases where post-clamp twists collapse
to identical) but hypothesis's
``HealthCheck.filter_too_much`` would surface this if the filter rate got
problematic; in practice the rate is well under 1% because random pairs
in ±300 collapse only when both have linear components in the same
sign-and-magnitude bin out-of-clamp.

This is the same pattern the existing two-event property uses for
infeasibility (``if reason is None ... else assert reason is non-empty``)
— let the strategy explore freely, branch on the precondition.

### Why the multi-event property uses K4-clean inputs

The plan's "Critical details" specified: "the strategy must produce strictly
increasing event times … then sort + add ``_K4_MIN_SPAN_S``."  We took this
literally — every adjacent gap in the multi-event proposal is ≥ 50 ms.  An
alternative would have been to mix in near-duplicate events to test K4 and
K1–K6 simultaneously.

We rejected the mixed strategy because it would conflate two failure
modes:

- **K4 merge bug** ⇒ output has < 50 ms span ⇒ ``_assert_K4`` fails.
- **Cascade-shift bug** ⇒ output K2/K3 violation in a non-stretched
  segment ⇒ ``_assert_K2_K3`` fails.

A failure on a mixed strategy would require diagnosis to attribute.  By
keeping the multi-event strategy K4-clean and letting ``test_property_K4_min_span``
own the merge-path coverage, each property tests one thing.  When a future
regression breaks one, the failing test names which contract was violated
without further investigation.

### Why no shrinking-friendly tweaks beyond the existing strategies

The existing strategies (``_pose_strat`` ∈ ±200, ``_twist_strat`` ∈ ±300,
``_T_strat`` ∈ [50 ms, 3 s], ``_vmax_strat`` ∈ [50, 300] mm/s, ``_tau_strat``
∈ [10, 150] ms) were tuned in the original W1 implementation and have
shipped two property tests for several months without complaint.  The
new properties reuse them verbatim where possible, with two narrow
additions:

- ``T1`` ∈ [1, 100] ms for K4 — straddles the 50 ms boundary on purpose.
- ``clamp`` ∈ [0, 500] mm/s for K6 idempotence — wider than ``_vmax_strat``
  to ensure the no-op path is exercised even when ``_vmax_strat`` is at its
  low end.

These bounds encode test-specific structural intent (K4 boundary straddle,
K6 wide clamp envelope) and are documented in the property docstrings.
Re-using ``_T_strat`` for K4 would have lost the boundary-straddle
property; widening ``_vmax_strat`` for K6 would have polluted unrelated
properties.  Narrow per-property additions are the right cost-tradeoff.

## Open Questions

- **Phase 8 profile-driven max_examples.** The committed values (100/100/
  150/150) match the existing convention (sad-path = 100, canonical = 150)
  per the audit answers.  Phase 8 introduces ``ci-fast`` (50) and
  ``ci-deep`` (1000) profiles; the per-test ``@settings(max_examples=…)``
  decorators should be replaced with profile-driven defaults at that point.
  Until then, the deep-validation gate is manual (``--hypothesis-seed=0``
  + temporary bump-and-revert, as documented in Phase 7's process gates).
- **K5 K6-clamp interaction surfaced as a contract clarification, not a
  bug.** During the audit gate, the K5-runs-after-K6-clamp ordering was
  noted but not flagged — the contract document
  ([REFERENCE_LAYER_CONTRACT.md:90–98](../controller/REFERENCE_LAYER_CONTRACT.md))
  says "two events at the same time MUST have identical twists", which
  is satisfied by post-clamp equality.  The K5 property's
  ``assume(not np.allclose(a_clamped, b_clamped))`` precondition encodes
  this contract reading.  If a future audit decides K5 should be
  pre-clamp (i.e., raw mismatch raises regardless of clamp collapse),
  the contract document needs a clarifying sentence and the property's
  precondition flips.  Captured here; not changed in Phase 7.
- **Cascade-shift property as a state machine?** The current multi-event
  property is a stateless property — it tests one ``make_feasible_events``
  call against random N-event proposals.  The scheduler's S5 contract
  ([scheduler-contract-phase-3](2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement.md))
  uses a ``RuleBasedStateMachine`` to exercise sequences of submit /
  replace / tick.  A similar state machine for ``make_feasible_events``
  could test "successive calls with overlapping cached event lists"
  — but ``make_feasible_events`` is stateless by design (no cross-call
  state), so a state machine would have to test consumer-side caching
  patterns (e.g., ``ZmqTargetSource``'s ``_cached_events``) instead.
  That's a Plan 2 surface (Tier 2 source-coverage), not Phase 7.
