---
title: "Single-ball toss Phase 0: post-merge reconciliation (doc pointers + INDEX backfill)"
type: refactor
date: 2026-07-25
status: resolved
phase: "MVP trajectory bringup — Phase 8 / single-ball toss Phase 0 (post-merge reconciliation)"
related_plan: single-ball-toss.md
subsystem: docs
tags: [docs]
commits:
  - PENDING
files_changed:
  - logbook/INDEX.md
  - sim/JUGGLE_DEMO.md
  - plans/active/bb-led-two-ball-juggle-demo.md
  - plans/active/single-ball-toss.md
---

# Single-ball toss Phase 0: post-merge reconciliation

## Summary

Phase 0 of `plans/active/single-ball-toss.md` closes: the doc pointers the
2026-07-24 merge left dangling are reconciled, and the full suite is green as
the phase gate. Four ladder logbook entries that were never indexed on the
demo branch get their `logbook/INDEX.md` rows backfilled; `sim/JUGGLE_DEMO.md`
gains a status banner distinguishing the paused offline demo from the merged
online ladder work; `plans/active/bb-led-two-ball-juggle-demo.md` gains a
merged-location note; the toss plan's phase table marks Phase 0 COMPLETE.
No code changes.

This entry is part of the 2026-07-25 unattended workflow-orchestrated build
run (operator-authorized 2026-07-24) executing
`plans/active/PROMPT-single-ball-toss-software-run.md`.

## Changes

- **`logbook/INDEX.md`** — backfilled the 4 rows scheduled by the toss plan's
  Phase 0 (the pre-existing indexing gap carried through the merge):
  `2026-06-26-contact-mechanics-integration`,
  `2026-07-03-catch-control-formulation-design-basis`,
  `2026-07-03-motion-quality-review`,
  `2026-07-03-p2-selfcatch-reunification-tension`. Each entry was read in
  full before its row was written; statuses/phases are quoted from each
  entry's own front matter; the three 2026-07-03 rows land newest-first
  within the day so the same-day supersession chain (design-basis supersedes
  p2-tension, which follows motion-quality-review) reads top-down. 129 → 134
  rows in this commit (133 after the four backfills, +1 for this entry's own
  row); non-increasing date order and link integrity re-verified by script
  after each of the two insertions.
- **`sim/JUGGLE_DEMO.md`** — status banner under the title: the README
  describes the PAUSED offline demo (`sim/juggle_demo.py`, CasADi optimiser —
  paused, not deleted); the online tilt-ladder work merged 2026-07-24
  continues on the production stack under `plans/active/single-ball-toss.md`;
  two-ball authority remains `plans/active/bb-online-juggle-tilt-rearchitecture.md`;
  the demo branch referenced in §2 is merged and retired.
- **`plans/active/bb-led-two-ball-juggle-demo.md`** — merged-location note
  under the title: branch merged into `mvp-trajectory-bringup` (merge commit
  `20fcc9e`), branch + worktree retired, frontier pointers as above.
- **`plans/active/single-ball-toss.md`** — phase table: Phase 0
  `IN PROGRESS (2026-07-24 merge)` → `COMPLETE (2026-07-25)`.

## Discussion

The INDEX rows were drafted by a dedicated agent that read all four entries
end-to-end (per the run kickoff's "read each entry before summarizing it"),
and were verified against each entry's front matter before insertion; the
insertion itself ran as an anchor-asserting script rather than a hand edit
because the surrounding rows are multi-kilobyte single lines where a partial
paste is easy and invisible. The script re-checks the two INDEX invariants
the 2026-07-24 merge established (dates non-increasing, every link resolves)
after writing, so this commit cannot re-open them.

One draft error was caught in review: the bb-led plan note originally
referenced "§2's clone-the-demo-branch instructions", but §2 of that plan is
*Architecture* — the clone instructions live in `sim/JUGGLE_DEMO.md` §2. The
plan note now says "branch references throughout this plan are historical"
instead. The banner's §2 reference is correct for its own file.

Phase-table timing: the row flips to COMPLETE in the same commit that
completes the phase — the suite-green gate is cited below and the commit SHA
is backfilled here immediately after the commit exists, per the logbook
SHA-backfill convention.

## Verification

- INDEX integrity script (2026-07-25, post-insertion): 134 rows (129
  pre-change + 4 backfills + this entry's own row), dates non-increasing,
  all entry links resolve to files in the tree. Both insertion scripts ran
  the full ordering + link check after writing.
- `/audit --unstaged` (2026-07-25, pre-commit): no BLOCKING findings;
  1 WARNING (this entry's row-count claim was stale — written as 133 before
  this entry's own INDEX row made it 134; fixed here) and 1 NOTE
  (`sim/JUGGLE_DEMO.md` §2 still instructed checking out the retired demo
  branch — an inline historical marker was added). Both fixed-and-landed in
  this commit per the autonomous-runner policy.
- Full suite (phase gate) — `pytest tests/ -q`, run 2026-07-25: **3153
  passed, 3 xfailed, 198 warnings in 1325.20 s**. Same pass/xfail counts as
  the 2026-07-24 post-merge baseline (the 3 xfails are the documented
  T-U-T1a-4 + two demo-branch strict-xfails).
