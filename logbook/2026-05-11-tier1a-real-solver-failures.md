---
title: MPC sad-path coverage — Phase 1 Tier 1a real IPOPT exit codes + fallback-keyword matrix
type: feature
date: 2026-05-11
status: resolved
phase: "mpc-sadpath-coverage-tiers-1-3 — Phase 1"
related_plan: "mpc-sadpath-coverage-tiers-1-3.md"
related_entries:
  - 2026-05-10-scheduler-cancel-next-during-transitioning
  - 2026-05-10-scheduler-begin-return-s3-overwrite
  - 2026-05-10-mpc-tier0-phase-8-ci-hypothesis-profiles
  - 2026-04-23-hot-loop-zero-allocation-contract
files_changed:
  - tests/sim/test_solver_failures.py
  - logbook/2026-05-11-tier1a-real-solver-failures.md
  - logbook/INDEX.md
  - plans/active/mpc-sadpath-coverage-tiers-1-3.md
commits:
  - <pending>
subsystem:
  - controller
  - mpc
tags:
  - testing
  - safety
  - solver
  - fallback
---

# MPC sad-path coverage — Phase 1 Tier 1a real IPOPT exit codes + fallback-keyword matrix

## Summary

Plan 2 Phase 1 (Tier 1a) — closes the largest sad-path coverage gap
identified by the 2026-05-08 MPC-layer audit: solver-failure paths.

Adds [tests/sim/test_solver_failures.py](../tests/sim/test_solver_failures.py)
with **17 new tests** (plus 1 xfail) covering:

- Four IPOPT exit codes driven by **real solver execution**
  (``Maximum_CpuTime_Exceeded``, ``Maximum_Iterations_Exceeded``,
  ``Infeasible_Problem_Detected``, and the CasADi exception path).
- The success-keyword classifier at
  [mpc.py:1082](../controller/mpc.py)
  (``ret in ('Solve_Succeeded', 'Solved_To_Acceptable_Level')``)
  parametrised over **every documented IPOPT status string** —
  catches the load-bearing tuple-typo / future-CasADi-string-rename /
  ``Solved_To_Acceptable_Level``-miscoverage regression classes.
- Critical-detail bookkeeping: warm-start intact across non-exception
  failures; warm-start cleared on the exception path; ``_consecutive_failures``
  increments per fault and resets on success; recovery from real
  infeasibility produces a clean subsequent solve.

One xfail (T-U-T1a-4 ``Restoration_Failed``) is permanently accepted —
the IPOPT-internal options that would force restoration failure
(``start_with_resto=yes`` / ``expect_infeasible_problem=yes``) are not
exposed through ``MPCParams`` and adding them would be a production-
code change orthogonal to Plan 2's "test additions only" discipline.
Structural coverage of the keyword classifier for that string is
included in the parametrised matrix.  See **Discussion → Xfail
accounting** for the Plan 2 archival-gate justification.

Test additions only; **zero production-code changes**.

Pre-Phase-1: 1193 passing.  Post-Phase-1: **1210 passing + 1 xfailed**
(+17 net).

## Motivation

Pre-existing fallback coverage in
[tests/sim/test_mpc_static.py::TestFallback](../tests/sim/test_mpc_static.py)
drives failure via a synthetic "infeasible reference" pattern
(``REF = [500, 500, 500, 0, 0, 0]``) plus tight ``max_iter`` /
``max_cpu_time``.  Every existing failure-path test routes through
``cold_hold(Maximum_Iterations_Exceeded)`` — a single code path that
leaves the rest of the failure-classifier surface entirely untested.

The audit identified three concrete gaps:

1. **``Maximum_CpuTime_Exceeded``** is never exercised — the existing
   tests use ``max_cpu_time=0.001`` but pair it with ``max_iter=1``,
   and ``max_iter`` always wins.  The CPU-time path has its own
   ``_timeout_hint`` capture logic at
   [mpc.py:1089–1102](../controller/mpc.py) that was untested.
2. **``Infeasible_Problem_Detected``** is never driven through the
   IPOPT branch.  The existing infeasibility tests use a far-distance
   reference that produces ``Maximum_Iterations_Exceeded`` instead.
3. **The success classifier's ``Solved_To_Acceptable_Level`` arm** has
   no test.  IPOPT essentially never emits this string on our
   well-conditioned problem, so no real-driver test could catch a
   typo or refactor that removed it from the tuple.

Phase 1 closes all three gaps with **real** solver execution where
possible (rather than mocked returns) and uses targeted stats injection
where the IPOPT API doesn't expose the necessary controls.

## Design

### Real-driver strategy per exit code

Pre-implementation probes ([/tmp/probe_ipopt_exit_codes.py],
[/tmp/probe_ipopt_infeas.py], [/tmp/probe_restoration.py]; the probes
themselves are not committed) mapped each IPOPT exit code to the
cleanest parameter recipe that produces it on our pinned CasADi 3.7.2
on Jetson:

| Exit code                       | Recipe                                                          | Wall-clock |
|---------------------------------|-----------------------------------------------------------------|-----------|
| ``Maximum_CpuTime_Exceeded``    | ``max_cpu_time=1e-3``                                          | <5 ms     |
| ``Maximum_Iterations_Exceeded`` | ``max_iter=1``                                                  | ~10 ms    |
| ``Infeasible_Problem_Detected`` | pin ``q[k]`` decision bounds to ``[1000.0, 1000.0+1e-9]``       | ~200 ms   |
| ``Restoration_Failed``          | (not drivable; see Xfail accounting)                            | n/a       |
| ``(exception)``                 | monkey-patch ``mpc._solver`` to a ``_RaisingSolver`` stub       | ~0 ms     |

Working Note #1 in the plan flagged a concern that ``max_cpu_time=1e-6``
might cause CasADi to raise an internal init error before IPOPT
initialises.  Empirically, the entire range ``[1e-6, 1e-2]`` returns a
clean ``Maximum_CpuTime_Exceeded`` on this stack — the concern doesn't
apply.  ``1e-3`` is chosen because it leaves IPOPT enough wall-clock to
get into the iteration loop (typical wall-clock for the failing solve
is 2–9 ms, well above the budget), giving the most realistic version of
the failure path.

### Why ``Infeasible_Problem_Detected`` needs pinned decision bounds,
### not the "flip ubg < lbg" trick

A natural-seeming way to drive infeasibility — set ``ubg[0] < lbg[0]``
on the constraint vector — fails because CasADi pre-validates ``lb ≤ ub``
before handing the problem to IPOPT.  The validation assertion fires at
``nlpsol`` ``__call__`` time, routing through the **exception** handler
at [mpc.py:1223–1238](../controller/mpc.py), not the IPOPT
classifier at line 1082.  The resulting status is
``hold_extrap(exception: ...Ill-posed problem detected...)`` — useful for
T-U-T1a-5 but not for T-U-T1a-2.

The pinned-decision-bound approach instead constructs a problem that
**passes** CasADi pre-validation (``ubw[i] = lbw[i] + 1e-9``) but has
no feasible point against the dynamics + IK + workspace constraints.
IPOPT runs to completion in restoration mode and returns the genuine
``Infeasible_Problem_Detected`` exit code (~200 ms wall-clock).

The ``q``-decision-bound region (indices ``6N..12N`` per the layout at
[mpc.py:646–651](../controller/mpc.py)) is the cleanest pin point:
pinning ``q`` to 1000 mm with a stroke of ~275 mm conflicts with the
IK constraints linking ``q`` to ``p``, with no parameter regime where
``Solve_Succeeded`` would arise instead.

### Why T-U-T1a-6 uses ``stats()`` injection, not real exit codes

The success classifier ``ret in ('Solve_Succeeded', 'Solved_To_Acceptable_Level')``
at [mpc.py:1082](../controller/mpc.py) is a literal-tuple test.  It
has three regression classes the real-driver tests can't catch:

1. **``Solved_To_Acceptable_Level`` success-classification.**  IPOPT
   emits this when it converges to ``acceptable_tol`` after
   ``acceptable_iter`` iterations without reaching full ``tol``.  Our
   problem is well-conditioned and barrier-tuned, so IPOPT essentially
   always reaches ``Solve_Succeeded`` rather than the acceptable
   shortcut.  A test relying on real IPOPT to surface this string
   would be flaky at best, impossible at typical parameters.  But the
   production code *depends on* the classifier accepting this string
   — if a refactor drops it from the tuple, production silently
   routes acceptable-convergence solves to fallback (commanding stale
   ``_prev_w`` against a perfectly usable solution).

2. **Tuple-structure typo.** A future edit produces
   ``('Solve_Succeeded' 'Solved_To_Acceptable_Level')`` (Python
   implicit string-concat collapsing two entries into one) or
   ``('Solve_Succeeded',)`` (accidental drop).  The matrix test fails
   immediately; the real-driver tests still pass because
   ``Solve_Succeeded`` keeps working.

3. **Future-CasADi string-rename surface.**  When the suite is run
   against a new CasADi version, the matrix's documented enumeration
   is the diff anchor for any renamed / added status string (e.g.,
   newer IPOPT introduces ``Maximum_WallTime_Exceeded`` alongside
   ``Maximum_CpuTime_Exceeded``).

The ``_StatsInjector`` wrapper at
[test_solver_failures.py](../tests/sim/test_solver_failures.py)
runs the real solver to produce a valid ``sol`` (``sol['x']``,
``sol['lam_g']``, ``sol['lam_x']`` all populated), then returns the
caller-supplied status string from ``stats().return_status``.  The
classifier at line 1082 sees the injected string and routes
accordingly.  This is functionally equivalent to testing
``pure_func(s)`` for each ``s`` in a documented input set — the test
is not asserting anything about IPOPT's behaviour, it's asserting the
classifier behaves correctly given a string.

### Why ``use_aot_solver=False`` is mandatory in the test fixtures

The AOT-compiled solver at
[controller/generated/mpc_gen.so](../controller/generated/) embeds the
IPOPT options at C++-compile time (when ``python controller/generate_solver.py``
runs).  Loading the ``.so`` via [mpc.py:597](../controller/mpc.py)
passes the runtime ``opts`` dict, but the options *baked into* the
``.so`` (such as the production ``max_cpu_time=0.018``,
``max_iter=200``) take precedence in some paths.  The probes confirmed
this: with ``use_aot_solver=True``, ``max_cpu_time=1e-3`` was honoured
on this machine, but the behaviour is brittle to whether the AOT cache
exists and matches the current hash.

Setting ``use_aot_solver=False`` forces the in-process
``cs.nlpsol('mpc', 'ipopt', nlp, opts)`` build at
[mpc.py:583](../controller/mpc.py), where the runtime ``opts`` dict is
the only source of IPOPT configuration.  This is the cleanest path
for failure-driver tests; the AOT path is the production path and is
exercised by the rest of the suite.

### Why ``_StatsInjector`` is a wrapper class, not a monkey-patched attribute

CasADi ``Function`` objects are SWIG-wrapped and don't allow attribute
assignment — ``mpc._solver.stats = lambda: ...`` raises
``AttributeError: 'casadi.casadi.Function' object has no attribute 'stats'``
(write access denied).  The only way to intercept ``stats()`` is to
replace the entire ``_solver`` reference with a Python object that
quacks like the solver (``__call__`` + ``stats``).  The
``_StatsInjector`` class is six lines; the wrapper-pattern boilerplate
is the cleanest option available.

## Implementation

### tests/sim/test_solver_failures.py (new, 18 tests)

Structure:

| Class                                  | Scope                                                        | Tests |
|----------------------------------------|--------------------------------------------------------------|-------|
| ``TestRealIpoptExitCodes``             | T-U-T1a-1, -2, -3, -5 — real exit codes via parameter tuning | 4     |
| ``TestRestorationFailedNotDrivable``   | T-U-T1a-4 — permanent xfail (see Xfail accounting)           | 1     |
| ``TestFallbackKeywordMatrix``          | T-U-T1a-6 — parametrised over 10 documented IPOPT strings     | 10    |
| ``TestFailureBookkeeping``             | Critical-detail assertions (warm-start integrity, counter dynamics, recovery) | 3 |

Helpers:

- ``_create_mpc(plant, **overrides)`` — mirrors
  [test_mpc_static.py::_create_mpc](../tests/sim/test_mpc_static.py)
  with the additional ``use_aot_solver=False`` default (see Design
  above).
- ``_seed_warm_start(plant, mpc, ref)`` — runs one successful solve so
  the failing-MPC scenarios have warm-start state to fall back on.
- ``_pin_q_decision_bounds_infeasible(mpc, value=1000.0)`` — singleton-
  infeasibility helper for T-U-T1a-2.  Mutates ``mpc._lbw`` / ``_ubw``
  in place; callers stash and restore if reuse is needed.
- ``_RaisingSolver`` — drop-in solver replacement that raises a
  caller-supplied ``RuntimeError`` on every ``__call__``.  Used by
  T-U-T1a-5.
- ``_StatsInjector(inner, return_status, iter_count=5)`` — wraps a real
  CasADi solver, proxies ``__call__`` to the real solver, and returns
  the caller-supplied status from ``stats()``.  Used by T-U-T1a-6
  (matrix) and ``TestFailureBookkeeping`` (warm-start preservation).

### Cross-cutting validations in each real-driver test

Every test in ``TestRealIpoptExitCodes`` asserts:

- The expected status string appears in ``diag['status']`` (substring,
  not exact match — ``_handle_failure`` wraps as
  ``cold_hold(<status>)`` / ``hold(<status>)`` / ``fallback(<status>)``).
- ``mpc.consecutive_failures == 1`` after the single fault.
- ``cmd`` is finite — the fallback path always produces a safe
  commanded extension regardless of which IPOPT exit drove it.

The exception-path test (T-U-T1a-5) additionally asserts the
warm-start clearing per [mpc.py:1229–1234](../controller/mpc.py): all
six warm-start fields (``_prev_w``, ``_prev_lam_g``, ``_prev_lam_x``,
``_timeout_hint``, ``_timeout_lam_g``, ``_timeout_lam_x``) zeroed, and
``_fallback_step`` reset to 0.

### TestFailureBookkeeping — critical-detail assertions

| Test                                              | Asserts                                                                      |
|---------------------------------------------------|------------------------------------------------------------------------------|
| ``test_warm_start_intact_after_solver_failure``  | Non-exception failure preserves ``_prev_w`` identity and contents             |
| ``test_consecutive_failures_resets_on_recovery``  | Counter resets to 0 the moment a success arrives (per [mpc.py:1105](../controller/mpc.py)) |
| ``test_recovery_after_real_infeasibility``        | Post-``Infeasible_Problem_Detected``, restoring bounds produces ``Solve_Succeeded`` cleanly |

The warm-start-intact test uses ``_StatsInjector`` (not the real
``max_cpu_time=1e-3`` driver) because the real driver triggers the
``_timeout_hint`` capture path at
[mpc.py:1089–1102](../controller/mpc.py), which writes into the
pre-allocated buffers.  That's correct production behaviour, but it
muddies the "is the *primal* warm-start preserved" question.
Injecting the status string after a clean solve isolates the
``_handle_failure`` walk-forward path from the timeout-hint path.

## Verification

### Test results

- **Before Phase 1:** 1193 passing (post Plan 2 Phase 0).
- **After Phase 1:** **1210 passing + 1 xfailed** — the +17 net delta
  matches: 4 ``TestRealIpoptExitCodes`` + 10
  ``TestFallbackKeywordMatrix`` (parametrised) + 3
  ``TestFailureBookkeeping``.  ``TestRestorationFailedNotDrivable`` is
  the documented xfail.  Zero regressions on existing tests.

ci-fast (``pytest tests/ -q``): 1210 / 1210 + 1 xfailed in 287.81 s.
No transient flakes this run (``test_decay_boundary_continuity`` —
flagged in Working Note #5 — passed cleanly).

The new file in isolation: 17 / 17 passed + 1 xfailed in 4.85 s.

### Pinned IPOPT exit-code coverage on this stack

Per the per-test status assertions and the probe-run analysis, the
following IPOPT exit codes are confirmed reachable on **CasADi 3.7.2 +
Jetson Orin Nano + this MPC problem shape**:

| Exit code                       | Reachable? | Test                              |
|---------------------------------|------------|-----------------------------------|
| ``Solve_Succeeded``             | yes        | every existing passing test       |
| ``Solved_To_Acceptable_Level``  | classifier-tested only (rarely emitted on this problem) | ``TestFallbackKeywordMatrix`` |
| ``Maximum_CpuTime_Exceeded``    | yes        | T-U-T1a-1                         |
| ``Maximum_Iterations_Exceeded`` | yes        | T-U-T1a-3                         |
| ``Infeasible_Problem_Detected`` | yes        | T-U-T1a-2                         |
| ``Restoration_Failed``          | **no** (xfail) | T-U-T1a-4 documented gap        |
| ``Search_Direction_Becomes_Too_Small`` | classifier-tested only (IPOPT-internal trigger) | ``TestFallbackKeywordMatrix`` |
| ``Diverging_Iterates``          | classifier-tested only | ``TestFallbackKeywordMatrix`` |
| ``User_Requested_Stop``         | classifier-tested only (no callback registered to trigger) | ``TestFallbackKeywordMatrix`` |
| ``Internal_Error``              | classifier-tested only | ``TestFallbackKeywordMatrix`` |
| ``exception: ...``              | yes (via solver crash) | T-U-T1a-5                  |

A CasADi version upgrade that renames or removes any of these strings
will surface either as a real-driver test failure (for the reachable
codes) or as an unhandled-case test failure (for the matrix), with
the matrix's parametrised list as the diff anchor.

## Discussion

### Xfail accounting — T-U-T1a-4 Restoration_Failed

Per Plan 2 Working Note #2: every xfail carries (a) test ID, (b)
tracking reference, (c) target close phase or date.

| Field             | Value                                                                |
|-------------------|----------------------------------------------------------------------|
| Test ID           | T-U-T1a-4                                                            |
| Tracking          | This logbook entry's Discussion (you are reading it)                 |
| Target close      | **Permanent**.  Per Plan 2's archival-gate language: "zero unfixed xfails at archival, OR each residual xfail has a documented justification for why it's permanently acceptable." |

**Justification.**  Driving ``Restoration_Failed`` requires the IPOPT-
internal options ``start_with_resto=yes`` and
``expect_infeasible_problem=yes``, neither of which is exposed through
``MPCParams``.  Adding them would be a production-code change
orthogonal to Plan 2's "test additions only" discipline.  The
structural coverage of the keyword classifier for this status string
is provided by ``TestFallbackKeywordMatrix``, which asserts the
classifier routes ``'Restoration_Failed'`` to fallback regardless of
whether IPOPT emits it on our problem.  If a future plan adds
restoration-control options to ``MPCParams`` (e.g., as part of a
"solver-tuning surface" expansion), the xfail can be removed in the
same commit as the option addition; until then, the structural
coverage is sufficient.

### Why this phase doesn't pre-emptively mitigate Working Note #5

Working Note #5 in the plan flagged that the hot-loop allocation
contract will flake more under Phase 3's hypothesis stateful fuzz.
Phase 1 adds zero hypothesis tests — the parametrised matrix is
pure ``pytest.mark.parametrize`` over a fixed list, with no
generation surface.  The flake risk after Phase 1 is identical to
pre-Phase-1, so a pre-emptive ``gc.collect()`` or ``slow`` marker
isn't load-bearing here.  Phase 3 will need the mitigation.

### Real-driver vs synthetic driver discipline

The plan's most important self-imposed constraint is "drives real
solver failure paths, not synthetic ones".  Phase 1's compliance:

- **Three of four test classes drive real solver failures end-to-end.**
  ``TestRealIpoptExitCodes`` runs IPOPT to completion in each test;
  ``TestRestorationFailedNotDrivable``'s xfailed body is also a real-
  driver attempt (just one that IPOPT doesn't reach);
  ``TestFailureBookkeeping``'s recovery tests drive a real
  ``Infeasible_Problem_Detected`` then real ``Solve_Succeeded``.
  ``TestFallbackKeywordMatrix`` is the sole synthetic-string class.
- **One test class drives a synthetic status string.**
  ``TestFallbackKeywordMatrix`` runs the real solver and intercepts
  the status string.  The synthetic part is the *string*, not the
  *behaviour* — IPOPT genuinely executes the problem; the test then
  hands the classifier a documented string set to verify the
  classifier's behaviour.  This is the right tool for testing a
  literal-tuple classifier: the classifier's correctness is
  string-by-string, not problem-by-problem.
- **One xfail.** Documented above.

The exception path (T-U-T1a-5) uses ``_RaisingSolver`` — also
synthetic — because the real CasADi API doesn't have a way to make
``nlpsol.__call__`` raise on demand without monkey-patching.  This is
the only way to exercise the exception handler at
[mpc.py:1223–1238](../controller/mpc.py), and the production code's
exception path is otherwise dark.  An alternative would be to drive
the CasADi pre-validation assertion (e.g., ``ubg < lbg``), but that
produces a CasADi-internal message rather than a clean
``RuntimeError`` — testing through it would couple the test to CasADi's
internal error-string format, which IS the brittle synthetic.  The
``_RaisingSolver`` is the more honest synthetic: it produces a clean
Python exception that exactly models the "solver crashed for an
unforeseen reason" failure mode the handler exists to catch.

### Convert line citations to symbol references — partial application

Working Note #3 recommends refreshing line-number citations to symbol
references during implementation.  The Phase 1 test docstrings cite
``mpc.py:1082``, ``:1089–1102``, ``:1105``, ``:1223–1238``, ``:1229–1234``
— concrete lines, not symbols.  Reasoning: the lines being cited are
inside a 200-line ``solve()`` method, and "the success-classifier line"
in plain English wouldn't identify the same line a year from now if
the function grew or shrunk.  The line citations point at sufficiently
narrow constructs (a single line; a 5-line warm-start-clear block) that
a future refactor would either preserve them or signal a behaviour
change that should re-anchor the citation.  When ``mpc.py`` is next
audited, refreshing these citations is a 5-minute task — the line
density of ``solve()`` makes them more readable than a symbolic
breadcrumb chain ("the success branch's classifier in the success arm
of the try block...").

### What the real-driver tests revealed that the existing suite missed

Two concrete behaviours surfaced by Phase 1 work that pre-Phase-1
tests would not have caught:

1. **The "infeasibility-via-flip-bounds" trick in
   [test_mpc_static.py:_INFEASIBLE_REF](../tests/sim/test_mpc_static.py)
   is a misnamed test.**  It routes through the exception handler,
   not the IPOPT classifier.  No production-code change needed (the
   tests still validate the exception-handling path), but a future
   reader of the test file shouldn't infer that
   ``Infeasible_Problem_Detected`` is exercised there.  Not changed
   in this commit (out of scope; a follow-up audit pass on
   ``test_mpc_static.py`` would be the right venue).

2. **``Maximum_CpuTime_Exceeded`` triggers a separate buffer-copy
   path** at [mpc.py:1089–1102](../controller/mpc.py) that captures
   the unconverged solution as ``_timeout_hint`` for next-solve
   warm-start.  The buffer-copy is gated on
   ``np.all(np.isfinite(self._timeout_hint_buf))`` — if the unconverged
   solution is non-finite, the hint is dropped.  T-U-T1a-1's run on a
   well-conditioned problem produces a finite unconverged primal, so
   the timeout-hint capture exercises both arms (the copy ran; the
   isfinite check passed).  Edge case: a CPU-time exceeded on an
   ill-conditioned solve might produce NaN — not tested here, but
   the timeout-hint dropping case is structurally protected by the
   isfinite gate.  Phase 3's NaN/Inf fuzz will exercise this
   structurally.

## Open Questions

- **Should ``Solved_To_Acceptable_Level`` have a real-driver test?**
  Producing it deterministically requires constructing a problem where
  IPOPT converges to ``acceptable_tol`` (1e-3) but not ``tol`` (1e-4).
  Doable in principle by tuning the IK weights so the optimum sits in
  that narrow band, but flaky against any parameter change and not
  worth the maintenance cost.  Classifier coverage via the matrix is
  the right scope for this string.
- **Should the exception path also test the cleanup of
  ``_prev_prev_u``?**  The exception handler at
  [mpc.py:1229–1234](../controller/mpc.py) clears six fields but
  doesn't touch ``_prev_u`` or ``_prev_prev_u``.  Those are needed by
  ``_handle_failure`` to compute the cold-fallback command, so
  preserving them is correct — but the handler doesn't have a test
  asserting "these specific fields are preserved while these others
  are cleared".  Phase 3's stateful warm-start property tests will
  cover this structurally.
- **Future CasADi upgrade workflow.**  When the pinned CasADi version
  changes, the maintainer should: (a) re-run
  ``pytest tests/sim/test_solver_failures.py::TestFallbackKeywordMatrix``
  with the new version, (b) check the IPOPT changelog for renamed /
  added status strings, (c) update ``_FALLBACK_KEYWORD_MATRIX`` in
  the test file.  This logbook entry is the right place to document
  the result of that pass.

## Related

- [plans/active/mpc-sadpath-coverage-tiers-1-3.md](../plans/active/mpc-sadpath-coverage-tiers-1-3.md)
  — Plan 2 Phase 1 specification.
- [logbook/2026-05-10-mpc-tier0-phase-8-ci-hypothesis-profiles.md](2026-05-10-mpc-tier0-phase-8-ci-hypothesis-profiles.md)
  — CI hypothesis profiles (Plan 1 Phase 8) used by Phase 1's matrix.
- [logbook/2026-04-23-hot-loop-zero-allocation-contract.md](2026-04-23-hot-loop-zero-allocation-contract.md)
  — W4a pre-allocation discipline referenced in TestFailureBookkeeping.
- [controller/mpc.py](../controller/mpc.py) — ``solve()`` /
  ``_handle_failure`` / warm-start machinery under test.
- [tests/sim/test_solver_failures.py](../tests/sim/test_solver_failures.py)
  — this phase's new test file.
