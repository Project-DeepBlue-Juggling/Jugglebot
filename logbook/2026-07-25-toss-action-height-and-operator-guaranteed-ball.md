---
title: "Toss.action: throw-height goal + operator-guaranteed ball (drop the ball-evidence gate)"
type: feature
date: 2026-07-25
status: resolved
phase: "single-ball-toss Phase 5 prep (Toss.action interface for hardware)"
related_plan: single-ball-toss.md
subsystem: ros
tags: [kinematics, testing]
commits:
  - c3beaa2
files_changed:
  - ros_ws/src/jugglebot_interfaces/action/Toss.action
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/toss_release.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - config/hardware_config.yaml
  - tests/ros/test_toss_coordinator.py
  - tests/motion/test_toss_release.py
---

# Toss.action: throw-height goal + operator-guaranteed ball

## Summary

Two operator-facing changes to `Toss.action` ahead of the Phase-5 hardware
bring-up, on the operator's direction: **(C)** the goal now nominates a
**throw height** (`throw_height_m`, apex above release) instead of a flight time
— height is the juggling-relevant variable; and **(B)** the CHECKING
**ball-evidence gate defaults OFF** (`toss_require_ball_evidence: false`) — the
operator guarantees a ball is in the hand, and the software's unreliable
possession belief must not be able to block a legitimate throw.

## Motivation

- **C (height, not time).** A juggler thinks in throw *height*, not flight time;
  the two are related but not linearly (`h = g·T²/8`, so `h ∝ T²`). Exposing
  height at the goal makes the operator's `send_goal` and the Phase-5 runbook
  read in the natural variable.
- **B (drop the gate).** There is **no ball-in-cup sensor** — "possession" is
  only a tracker-derived belief (a plausible `CAUGHT`), and tracker verdicts are
  unreliable (the false-MISSED corruption the Phase-7 reload arc keeps hitting).
  A physically-loaded ball can therefore fail to register and spuriously
  `REJECTED_NO_BALL` a legitimate toss. Since the operator guarantees the ball,
  the belief must not gate.

## Design

- **C.** The internal physics variable stays `flight_time_s` everywhere
  downstream (sequencer, release math, sim gate); only the operator-facing
  **goal field** changes. A single tested conversion
  `toss_release.flight_time_from_height(h) = sqrt(8·h/g)` (inverse
  `apex_height_from_flight_time`) runs once at ingestion in
  `reload_coordinator_node._execute_toss`. The `0 => config default` sentinel
  resolves to the (unchanged) `toss_flight_time_default_s` (~0.784 m apex at
  0.8 s), kept internally as a flight time. This is the minimal faithful change
  — `flight_time_s` had two roles (goal field vs internal physics), and only the
  goal-field role moved (grep-verified: zero remaining `req/goal.flight_time_s`
  consumers; the sequencer/gate keep their internal `flight_time_s`).
- **B.** A config constant `JB_OP_TOSS_REQUIRE_BALL_EVIDENCE` (default `False` —
  it is a hardware fact: no sensor exists) gates the `NO_BALL` reject:
  `ball_seated = waiver OR possession OR not require`. When not required
  (default), CHECKING never rejects on ball evidence; the **hand-parked** gate
  (reliably known from hand telemetry — it guards the kind-0 absolute-position
  stroke hazard) still stands. A forgotten ball just fires an empty stroke —
  benign, a missed-reload no-op.

## Implementation

- `Toss.action`: `float64 flight_time_s` → `float64 throw_height_m`.
- `toss_release.py`: `flight_time_from_height` / `apex_height_from_flight_time`.
- `reload_coordinator_node._execute_toss`: read `throw_height_m`, validate
  (`_invalid_toss_goal_field` field-renamed), resolve default / convert to the
  internal `flight`; `_build_toss_observations` `ball_seated` gains the
  `or not hw.JB_OP_TOSS_REQUIRE_BALL_EVIDENCE` term.
- `config/hardware_config.yaml`: `toss_require_ball_evidence: false` (+ regen).
- Tests: goal-builder field rename; the gate-exercising coordinator tests
  (`no_ball`, waiver, possession→ball_seated) monkeypatch the constant `True`
  so they still test the gate; new `test_no_ball_does_not_reject_by_default`
  pins the shipped no-block behaviour; new `test_flight_time_from_height_roundtrip`
  pins the conversion (`0.8 s ⇔ 0.78448 m`, `h ∝ T²`).

## Discussion

**Why B is a config default, not a per-session waiver.** The operator explicitly
did not want a Phase-3-style waiver they must remember to set. A config constant
defaulting false makes the no-gate behaviour the *shipped* state, with a clean
hook (`true`) if a real ball-held sensor is ever wired into possession. The
possession *latch* mechanism is retained (it still updates on CAUGHT / clears on
release) — it is simply not gating by default, so a future sensor slots in
without re-architecting.

**The Capture R interaction (a consequence worth recording).** With the gate
default-off, the Phase-3 dry-trace **Capture R** (un-waived → `REJECTED_NO_BALL`)
no longer rejects by default — the un-waived toss proceeds past NO_BALL. Phase 3
is CLOSED and this does not touch its validated result; the Phase-3 runbook and
this config's comment note that reproducing Capture R now requires
`toss_require_ball_evidence: true`. This was surfaced during implementation
rather than discovered on a re-run.

**Why Option A for C (goal field only, config default stays a flight time).**
The operator's ask was to change the *action*, not the internal default. Keeping
`toss_flight_time_default_s` internal avoids churning the sequencer's
`DEFAULT_TOSS_FLIGHT_TIME_S` fallback and its drift-guard test, and the
`0 => default` sentinel resolving to a flight time is a defensible implementation
detail (documented). A follow-up can make the config default a height if wanted.

## Verification

- Toss subset (`pytest tests/ros/test_toss_coordinator.py
  tests/ros/test_toss_sequencer.py tests/ros/test_toss_integration.py
  tests/motion/test_toss_release.py -q`, 2026-07-25): **199 passed**.
- Full suite: `pytest tests/ -q`, run 2026-07-25 — **3399 passed, 3 xfailed in
  1512.67 s** (+2 vs the Phase-3 baseline = the two new tests).
- Grep-to-zero: no remaining `req/goal.flight_time_s` consumer; no GUI/launch
  reference to the old goal field; the internal `flight_time_s` (sequencer,
  release math, sim gate) is intact.

## Open Questions

- **Hardware needs `colcon build --packages-select jugglebot_interfaces
  jugglebot`** (the interface field + config both changed) before the Phase-5
  session — the Phase-5 runbook carries this in its preflight.
- Whether the result should also report `achieved_height_m` (currently
  `achieved_flight_s`, the directly-measured quantity) — deferred; the operator
  commands height but the measured diagnostic is time.
