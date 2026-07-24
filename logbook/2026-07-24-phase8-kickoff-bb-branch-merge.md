---
title: "Phase 8 kickoff: BB juggle branch merged; single-ball-toss plan created"
type: refactor
date: 2026-07-24
status: resolved
phase: "MVP trajectory bringup — Phase 8 (single-ball self-toss) kickoff"
related_plan: single-ball-toss.md
subsystem: sim
tags: [docs, testing]
commits:
  - 20fcc9e
  - d91a40e
files_changed:
  - "(merge) controller/demo/*, sim/juggle_*.py, sim/juggle_tilt.py, tools/probes/juggle_*, tests/sim/test_juggle_*, tests/sim/test_demo_*, 17 logbook entries, 3 plans"
  - plans/active/single-ball-toss.md
  - plans/active/mvp-trajectory-bringup.md
---

# Phase 8 kickoff: BB juggle branch merged; single-ball-toss plan created

## Summary

`demo/bb-led-two-ball-juggle` (the Jugglebot-bb worktree branch, last commit
2026-07-04) is merged into `mvp-trajectory-bringup`, landing the online-juggle
tilt-ladder primitives — throw-to-nominated-target (Rung 2a), catch-at-position
(Rung 1), the 12/12 kinematic-release self-catch loop (Rung 2b) — plus their
tests, probes, plans, and logbook history in the working branch. A new plan,
`plans/active/single-ball-toss.md`, expands MVP Phase 8 into a two-tier
`Toss.action` bring-up (8a toss-at-position on the level platform; 8b
tilt-aimed displaced throw→catch) that re-hosts these primitives on the
production trajectory stack behind a `reload_gate.py`-style sim gate. The
branch and its worktree are retired after the merge.

## Motivation

Phase 7 reload reached 15/19 catches (fourth sitting, 2026-07-24) — the catch
pipeline is hardware-proven, and single-ball tossing is next (MVP Phase 8). The
toss capability was already substantially built in sim on the BB demo branch,
but ~370 commits behind and driving the platform through sim-side kinematic
realisation instead of the production planner → emitter → pump stack the
hardware runs. Unifying the branches first (rather than porting piecemeal)
preserves the ladder's logbook/plan history, its 16 test files and 10 probe
harnesses, and ends a split-brain worktree workflow before two-ball work
starts.

## Changes

- **Merge** `demo/bb-led-two-ball-juggle` → `mvp-trajectory-bringup`
  (`20fcc9e`). 71 commits; conflict surface was two doc tables and one
  comment block:
  - `logbook/INDEX.md` — rows interleaved by date desc (65 HEAD + 13 demo;
    non-increasing order + link integrity verified by script);
  - `tools/probes/README.md` — table union;
  - `sim/hand/trajectory.py` — kept HEAD (both sides independently converged on
    `CATCH_VEL_RATIO = 0.6`; HEAD carries the provenance comment);
  - `sim/juggle_noise.py` — byte-identical on both sides.
- **New plan** `plans/active/single-ball-toss.md` — Phase 8 expansion:
  `Toss.action` goal = nominated catch state ⟨position, flight time⟩, tiered
  8a/8b, production-in-the-loop `sim/toss_gate.py`, hardware staging T0–T4.
- **Amended** `plans/active/mvp-trajectory-bringup.md` § Phase 8 + phase table
  to point at the expansion.

## Discussion

**Merge, not port.** The tip-to-tip conflict surface was measured before
choosing: only four files touched on both sides since the fork, two of them
identical or comment-only. MVP-side drift since the fork is almost purely
additive (+3565/−12 in `controller/`, +4966/−36 in `motion/`), so the ladder
code finds the APIs it imports. A port would have re-typed ~6,300 lines of sim
work and severed the rung logbook entries from the files they document.

**Action, not service, for the toss** (operator-confirmed 2026-07-24). A toss
is a multi-second choreography with phases, per-phase aborts, and a verdict —
the same shape as Reload, which already proved the action pattern on hardware
(phase feedback, cancellation semantics, latch ownership). A service returns
once and would bury THROWING/BALL_IN_FLIGHT/CATCHING progress and abort
handling in out-of-band topics.

**Two tiers instead of one.** The level-platform vertical toss (8a) and the
tilt-aimed displaced throw (8b) differ in risk class, not just difficulty: 8a's
catch is co-located and tracker-driven, absorbing release scatter; 8b inherits
the Rung-2b contact-detach knife-edge (landing-vs-origin sensitivity ≈2.7) as a
real hardware risk plus an unresolved ±100 mm directional asymmetry from
Rung 2a. Tiering lets hardware validate the forgiving case first while the gate
maps 8b's envelope in sim. Notably, 8a alone already covers the operator's
"vary catch (x, y, z) and throw height" ask by translating the platform to the
nominated point before a vertical toss.

**Sim throw stays a kinematic release.** The ladder reversed contact-detach →
kinematic release deliberately (operator-approved 2026-07-01); hardware catches
are visibly smooth while the sim contact model is the low-fidelity element, so
the gate would otherwise gate on sim artifacts. The real release scatter is
unknown — measuring it (T0 bench characterisation) is a hard prerequisite
before any hardware catch attempt, and the gate re-runs with the measured
noise.

## Verification

- Post-merge collection: 3156 tests collected, zero import errors.
- First full-suite run surfaced exactly one semantic conflict (all 10
  `test_reload_gate.py` failures shared it): `reload_gate.py:453` called the
  pre-rework single-ball `BallManager._hand_site_velocity()`; fixed in the
  merge commit by routing via `ball_manager.ball(0)` (the demo-side rework
  moved single-ball internals to `Ball`). Repo-wide grep: no other stale
  references to the old manager API.
- Full suite post-fix (`pytest tests/ -q`, run 2026-07-24): **3153 passed,
  3 xfailed, 198 warnings in 1310.76 s**. The 3 xfails = the pre-existing
  HEAD-side `test_solver_failures.py` T-U-T1a-4 `Restoration_Failed` xfail
  (the 1-xfail baseline unchanged since 2026-07-08) + the demo branch's own
  2 strict-xfails (offline-demo headline in `test_demo_juggle_sim.py`, ladder
  frontier in `test_juggle_selfcatch.py`), the latter documented in its plans.
- `logbook/INDEX.md` post-resolution: 128 rows, dates non-increasing, all 13
  demo-side entry links resolve to files present in the merged tree. 4 of the
  17 merged entries were never indexed on the demo branch (pre-existing gap,
  carried through: `2026-06-26-contact-mechanics-integration`,
  `2026-07-03-catch-control-formulation-design-basis`,
  `2026-07-03-motion-quality-review`,
  `2026-07-03-p2-selfcatch-reunification-tension`); row backfill is scheduled
  in the toss plan's Phase 0.

## Open Questions

- Coordinator hosting (new `toss_coordinator_node` vs. extending
  `reload_coordinator_node`) — deferred to Phase 1 implementation, decided by
  measured shared-client surface.
- Whether `catch_coordinator`'s correlation path is already thrower-agnostic
  for `thrower_name='jugglebot'` — verify in Phase 1 either way.
