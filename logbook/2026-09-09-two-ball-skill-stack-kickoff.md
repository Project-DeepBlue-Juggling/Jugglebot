---
title: "Two-ball skill stack arc opened — the plans board re-cut around the RAI juggling paper, eleven plans superseded, the workspace on its own branch"
type: refactor
date: 2026-09-09
status: in-progress
phase: "two-ball-skill-stack — R0 kickoff (board + workspace; code deletion follows as its own commit)"
related_plan: two-ball-skill-stack.md
files_changed:
  - plans/active/two-ball-skill-stack.md
  - plans/active/INDEX.md
  - plans/parked/INDEX.md
  - plans/archived/INDEX.md
  - plans/archived/unified-7dof-planner.md
  - plans/archived/critical-point-ilc.md
  - plans/archived/toss-selftuning.md
  - plans/archived/toss-pipelined-preamble.md
  - plans/archived/single-ball-toss.md
  - plans/archived/catch-robustness.md
  - plans/archived/bb-online-juggle-tilt-rearchitecture.md
  - plans/archived/mvp-trajectory-bringup.md
  - plans/archived/hand-trajectory-generator-overhaul.md
  - plans/archived/catch-reach-degenerate-overshoot.md
  - plans/archived/inertia-ratio-reconciliation.md
  - .claude/workflows/bb-tilt-phases.js (deleted)
  - .claude/workflows/mvp-phase-runner.js (deleted)
  - CLAUDE.md
  - config/generate_udp_protocol.py (path sweep) + 221 further files whose path-qualified plan references were re-pointed (comments, docstrings, runsheets, logbook prose)
  - logbook/2026-09-09-two-ball-skill-stack-kickoff.md
  - logbook/INDEX.md
subsystem:
  - motion
  - ros
  - sim
tags:
  - docs
  - safety
---

# Two-ball skill stack arc opened

Planning and board change only; no behaviour changes. The owner brought Lee,
Wang, Atkeson, Rizzi and Rojas, *Rapid On-Robot Learning for Dynamic
Manipulation Skills: Robot Juggling* (arXiv:2608.26800v2, `Related Research/`)
to the 2026-09-09 session and asked for a path from the current stack to
two-ball juggling on the paper's model. The analysis and the owner's five
decisions are recorded in `plans/active/two-ball-skill-stack.md` § 1; the
document's § 0 spells out the values (simplicity, elegance, coherence,
robustness, rigor) every agent on the arc is briefed against.

## Motivation

Three findings drove the re-cut, all in the plan's § 1.2 with provenance:

- **The beat is structural, not hardware.** Two balls in one hand need a beat
  of (flight + dwell)/2 — 0.63 s at a 1.0 m apex. The unified ring's floor is
  the 0.800 s chain floor plus a flight, 1.70 s at the same apex, and the joined
  LAUNCH+STEADY solve grew with the beat (171 knots, 1.15–3.3 s on 2026-09-09).
- **The cost is the gate, not the planner.** UH-3: `qp` 7–11 ms, `val`
  191–202 ms, `validate_cycle` 2.44 ms/knot flat. The paper puts limits inside
  the generator and precomputes the composability set offline; nothing gates on
  the beat path.
- **The learner in the paper is ~100 lines because the command is the desired
  outcome**, making the prior's Jacobian the identity. The critical-point ILC's
  finite-difference sensitivity chain, per-cell artifact and batch discipline
  exist only because its command was controller knobs. The owner accepted the
  replacement outright.

The hand-streaming pain of the last five sittings was diagnosed as a
migration artefact of two masters for one axis (the `hand_source` latch, the
arming step, homing's axis-6 mode, non-resetting counters); the streamed lane
itself is validated bit-exact and flew 6/7 catches on UH-6. Retiring the
Platform Teensy stroke engine therefore moves from the last rung to the first.

## Changes

1. **Workspace.** Branch `skill-stack` from `4b9876b` in a new worktree,
   `~/Desktop/Jugglebot-skills`, so the parallel firmware-validation session on
   `mvp-trajectory-bringup` is untouched (its uncommitted work is not on this
   branch by design).
2. **Plans board.** Eleven plans archived `superseded`, each with an archival
   note naming what survives and at which rung the rest retires (eight from
   active, three from parked); the three INDEX files updated in the same
   commit. The new plan added with its active row. Active board after: six
   rows (five retained plus the new plan); parked: five.
3. **Reference sweep.** Every path-qualified reference to a moved plan was
   re-pointed to `plans/archived/` — 222 tracked files at HEAD, 0 after
   (`git ls-files | xargs grep -lE 'plans/(active|parked)/(<moved>)\.md'`).
   The generated UDP-protocol artifacts carry the generator's comments, so
   `python config/generate_config.py` was re-run after sweeping the generator.
   Filenames are unchanged, so every `related_plan:` keeps resolving.
4. **Processes.** The two phase-runner workflows tied to superseded plans
   (`.claude/workflows/`) are deleted.
5. **CLAUDE.md.** One bullet naming the arc, its branch, its normative values
   section, and the supersessions.

Code deletion (the census-backed dead-layer list in the plan's § 6) is
deliberately a separate commit with its own entry, so the board change and the
first code removal roll back independently.

## Verification

Recorded at commit time in this entry's Outcome (the named tests for a board
change are `tests/sim/test_plans_index.py`, which pins every board in both
directions, and `tests/sim/test_logbook_front_matter.py` /
`test_logbook_search.py`; the sweep touched docstrings in `*.py`, so the
default gate runs as well).

## Outcome

Board re-cut committed on `skill-stack`. Verification (2026-09-09): `./run_tests.sh --full` in the
worktree — parallel phase **6987 passed, 9 skipped, 2 xfailed, 4 warnings in 449.83s (0:07:29)** (rc 0), serial phase 4 passed (rc 0), total 481 s, RESULT PASS;
the named board/logbook tests `pytest tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py
tests/sim/test_logbook_search.py -q` — 107 passed. Pre-commit `/audit --unstaged` found two BLOCKING
(the new plan lacked its active-INDEX row; two prose links in that INDEX still named moved plans), two
WARNING (the eleven archived rows had landed after the closing section instead of inside the table; two
archival notes cited plan sections by the wrong number) and two NOTE items (sweep count 222 not 221; two
beat-table cells carried rounding-chain drift) — all narrative, all applied before the second full gate.
Next: the R0 dead-layer deletion (plan § 6) as its own commit and entry.
