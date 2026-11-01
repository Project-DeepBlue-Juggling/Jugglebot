---
title: "JB-104: the pose planner's UNVERIFIED refusal names the worst-violated equality and inequality rows (reason unchanged; labels built lazily on the refusal path); box re-swept value-identical, gate_hash fe8c52ba88b4"
type: feature
date: 2026-11-01
status: complete
phase: "night-shift follow-up"
related_plan:
  - two-ball-oval.md
related_entries:
  - 2026-10-31-hold-ladder-sitting-the-hold-model-was-wrong.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - tests/motion/test_cup_cycle_row_labels.py
  - tests/motion/test_skills_admissible.py
  - config/generated/admissible_box.yaml
  - map/bug-pose-planner-reports-unverified-instead-of-a-named.md
  - logbook/INDEX.md
---

# JB-104: UNVERIFIED names the worst rows

**What.** At the feasibility edge the pose planner refused with
`REJECTED_CYCLE_INFEASIBLE(UNVERIFIED: QP solution failed verification ...)`, which tells the
operator nothing about what bound. JB-104 asked for a named `INFEASIBLE` with the binding row; the
Night Shift found that `cup_cycle._verify` deliberately does not distinguish an infeasible request
from a numerically degenerate one (2026-08-30, Phase 0: which refusal path fires is
BLAS-dependent). The owner chose JB-126 option 1: keep `reason='UNVERIFIED'` and that
non-distinction, and append the worst rows to the message.

**Change.** `_verify`'s refusal message now ends `— worst rows: equality <i>: <name> (residual
r); inequality <j>: <name> (violation v)`, each only when it exceeds the bar. `_Program` carries
layout bookkeeping recorded where the blocks are assembled (`eq_marks`, `ineq_names`,
`ineq_widths`, `held_knots`: a few tuple appends per assembly, no strings), and
`_eq_label` / `_ineq_label` / `_worst_rows_text` turn a row index into a name only inside the
refusal branch. Named exactly: release and terminal-rest rows (quantity, axis), catch position,
held-axis slaving, capture move, the pose planner's platform-z terminal rows; jerk and position
boxes (knot, axis, side), runway, contact-window floor, platform-z band, tilt cone side and floor,
touch-down skid. Named by block and row offset only: the three hold-velocity / post-release
acceleration blocks and the acceleration boxes. Anything inconsistent falls back to "equality row
i of m"; a labelling exception can never mask the refusal. Example:
`... refused rather than returned — worst rows: equality 10: post-release acceleration row 1
(residual 3.000e+01); inequality 190: knot 32 y position upper bound (violation 8.040e+00)`.

**Box.** `cup_cycle.py` is a gated file, so the edit re-hashes the admissible box
(961808ee2d20 -> fe8c52ba88b4). The 26 families of the 2026-10-31 sweep of record (its
`sweep_h0.sh` list, minus the two dropped separation-125 high-low families) were re-swept once
and each output diffed against the 2026-10-31 file below the `swept_at` / `gate_hash` lines:
all 26 IDENTICAL (a one-round copy of that `sweep_h0.sh`, scratchpad only, uncommitted; 2026-11-01 13:58-14:59, `xargs -P 6`, BLAS 1 thread), and the merged 82 cells equal the committed box in order (`yaml.safe_load` equality). The committed box keeps its 82 cells; only its header moved.

**Verification.**
- `pytest tests/motion/test_cup_cycle_row_labels.py` (8 tests: every labelled block checked against the assembled `Aeq` / `C` for throw+catch, held-axis capture and the pose planner; fallbacks; `_verify` names the row it was driven to break) -> 8 passed, 2026-11-01.
- Gate, 2026-11-01: `./run_tests.sh` -> 7915 passed, 50 skipped in 401.51 s; serial phase 5 passed (the planner budget tests included); exit 0. The hash pins (`tests/motion/test_skills_admissible.py`, two places) and `config/generated/admissible_box.yaml` header moved to fe8c52ba88b4 / 2026-11-01.
