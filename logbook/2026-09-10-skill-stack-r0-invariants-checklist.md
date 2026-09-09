---
title: "R0 invariant checklist: 88 rows, every enforcement point and pinning test named, before the FSM stack is hollowed out"
type: feature
date: 2026-09-10
status: resolved
phase: "two-ball-skill-stack — R0 invariant checklist (closes R0)"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/__init__.py
  - ros_ws/src/jugglebot/setup.py
  - plans/active/two-ball-skill-stack.md
  - logbook/INDEX.md
subsystem:
  - motion
tags:
  - docs
  - safety
---

# R0 invariant checklist

`motion/skills/INVARIANTS.md` is the net under the skill-stack arc: R1 deletes the
Platform Teensy stroke engine and R4 hollows out 13 k lines of FSM choreography, and
anything load-bearing living only inside those files is lost with them unless it is
written down first. Each row is one physical fact in one normative sentence, with the
single place it is enforced (`path::symbol`, or `firmware: file::fn`) and the single
test that fails without it. The rule at the top is the operative one: **a code deletion
at any rung must first check every row that names the deleted path**, and every such row
is either re-enforced in the same rung (`PORT@R<n>`) or declared not-an-invariant with
the reason (`RETIRE@R<n>`).

**88 rows across eight layers** — transport + firmware guards 17, wire frame + emitter 6,
planner / reference feasibility 13 (K1–K6 plus the cup-QP boxes and the catch runway),
levelling frame 6, possession + arming 10 (C-POSSESS-1, A1–A5), hand 8, catch / arrival /
reach 5, session-level refusals 23. **By fate: 48 KEEP, 29 PORT, 11 RETIRE.** Every RETIRE
row says why it is choreography rather than physics — tier gates, pipeline slots,
displacement caps and dwell floors describe the ring's shape, not the machine's.

The Gaps section carries 10 entries: six invariants with no pinning test or only adjacent
coverage (notably that the bridge is the sole axis-6 writer after R1, and the hand
deviation guard's armed trip, never observed on hardware), and four invariants stated in
two places that must be collapsed before they are ported — the tilt ceiling, the reach
bound, the 0.10 rev hand settle band (three statements of one number), and "is the machine
levelled?", where `RobotState.levelling_complete` still reads True in exactly the state
C-LEVEL-1.O's gate exists to refuse.

`jugglebot.motion.skills` is created here — empty but for a docstring, per plan § 2.2 —
and registered in `setup.py`, so the checklist sits at the destination it protects rather
than beside the code it is about to outlive.

## Verification

- `./run_tests.sh --full` (2026-09-10, worktree, tree = this unit + the R1 prompt): parallel **6780 passed,
  9 skipped, 1 xfailed in 331.15 s** (rc 0), serial 4 passed (rc 0), total 363 s — RESULT PASS, exit 0.
- Phase-end `/audit` (2026-09-10) over this unit plus the deletion commit's normative-doc notes: six narrative
  findings, all applied before commit — I-FW-1's cited native test does not drive the staleness path (row
  now `NONE — gap`, Gaps 5); the `ABORTED_NO_RELEASE` gap was false (`test_no_release_evidence_aborts` is a
  direct driver; row re-cited, gap replaced); the eight table headers were two cells short; the vocabulary
  sentence was truncated; two details in the R1 prompt (grep counts, a line reference) tightened. Docs-only
  fixes: the named tests re-run scoped afterwards (`tests/sim/test_plans_index.py`,
  `test_logbook_front_matter.py`, `test_logbook_search.py`; no test reads `INVARIANTS.md`).
