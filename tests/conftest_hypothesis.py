"""Hypothesis profile registration for the test suite.

Imported by ``tests/conftest.py`` at collection time so all hypothesis
tests default to the ``ci-fast`` profile unless a different one is
selected via ``--hypothesis-profile=<name>`` on the pytest command line.

Profiles
--------

- ``ci-fast`` (default): ``max_examples=50``.  Per-PR runs; full pytest
  suite target < 5 minutes wall-clock.
- ``ci-deep``: ``max_examples=1000``.  Nightly runs; full suite target
  ~10 min wall-clock.  Run with ``pytest tests/ -q --hypothesis-profile=ci-deep``.
- ``dev``: ``max_examples=200``.  Convenience for local iteration when
  ``ci-fast`` is too coarse to chase a flaky shrink and ``ci-deep`` is
  too slow.

All profiles share ``deadline=None`` (hypothesis's default ``200ms``
deadline is too tight for the per-example work in this repo's quintic-
feasibility properties and the scheduler state-machine walks) and a
common health-check suppression set:

- ``HealthCheck.too_slow`` — quintic feasibility checks and CasADi
  evaluations are inherently slower than scalar property tests.
- ``HealthCheck.filter_too_much`` — the K5 coincident-twist property
  uses ``hypothesis.assume`` to filter post-clamp-collision cases;
  the filter rate stays well under the default 50% threshold but is
  suppressed defensively here so a borderline-rate strategy doesn't
  flake under shrinking pressure.  (The scheduler state machine
  excludes past-time events by strategy bounds and uses
  ``@precondition`` for state-dependent gating, neither of which
  triggers ``filter_too_much``.)

Per Plan 1 Phase 8, every hypothesis test in this repo relies on
profile-driven defaults — no per-test ``@settings(max_examples=N)``
overrides exist.  This means selecting ``--hypothesis-profile=ci-deep``
actually scales every property to 1000 examples.  See
``logbook/2026-05-10-mpc-tier0-phase-8-ci-hypothesis-profiles.md`` for
the rollout history.
"""
from hypothesis import HealthCheck, settings

_COMMON_KW = dict(
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow, HealthCheck.filter_too_much],
)

settings.register_profile("ci-fast", max_examples=50, **_COMMON_KW)
settings.register_profile("ci-deep", max_examples=1000, **_COMMON_KW)
settings.register_profile("dev", max_examples=200, **_COMMON_KW)

# Default profile when --hypothesis-profile is not passed.
settings.load_profile("ci-fast")
