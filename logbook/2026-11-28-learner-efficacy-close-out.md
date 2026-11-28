---
title: "Learner efficacy close-out: E4 retired at the 15 mm/s Hand floor, one-Memory rule and lateral-authority row in INVARIANTS, PWAY-2 Destination narrowed"
type: feature
date: 2026-11-28
status: resolved
phase: "learner-efficacy E7 (PWAY-2)"
related_plan: learner-efficacy.md
related_entries:
  - 2026-11-27-learner-efficacy-e1-launch-baseline.md
  - 2026-11-26-plant-model-lateral-law-per-release-kind.md
  - 2026-11-27-plant-model-process-noise-on-b.md
files_changed:
  - plans/active/learner-efficacy.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - map/pathway-learner-generalisation.md
  - map/bug-two-ball-columns-runs-end-on-a-missed-catch-of-ball-2-with.md
  - map/bug-first-flight-of-the-per-release-kind-plant-model-bug-138-and.md
  - map/idea-the-oval-on-the-shared-memory-compile-the-oval-on-compile.md
  - tools/memory_scratch.py
  - tests/sim/test_memory_scratch.py
subsystem:
  - learner
tags:
  - docs
---

# Learner efficacy close-out

**What was decided (owner, 2026-11-27/28).** E1 showed a settled, hand-placed ball launches with the same scatter as a running Throw: about 15 mm/s per axis, landing sd 14 to 17 mm. The Cup cannot move sideways against the Platform, the Hand is stiff and symmetric, no marker can go on the Hand, and the ball is fully taped. The owner accepts 15 mm/s as this Hand's floor, so **E4 is retired**. The scatter is angular (about 0.2 degrees), so a lower Apex lands tighter. The one unflown idea, a ball-orientation check (pen mark, 30 Attempts in each of two orientations), is recorded under E4's "what would reopen it" and not filed.

**What changed.**
- INVARIANTS: new row I-LEARN-2 (the lateral authority is a Learner bound, never an actuator bound; widening c_max is safe only while install-time `validate_cycle` and the Teensy `MAX_DEVIATION` guard stay on the path). I-LEARN-1's stale P5 clause now states the one-Memory rule (plant `jugglebot` always; new plant only when the hardware physically changes; experiments write to a scratch copy via `tools/memory_scratch.py`). I-BEAT-1's yaw clause points to IDEA-12. No invariant weakened.
- Plan: PE4 retired, PE7 complete, E2's status carries the rule, § 7 answered. The plan stays active until BUG-158 and BUG-78's trial have flown.
- PWAY-2's Destination narrowed: no bias for new configurations or yaw; carried Throws only as far as BUG-138.
- BUG-78 gets E1's result in its `why:` and a Progress line; its 15 mm trial flies after BUG-158.
- New tool `tools/memory_scratch.py <scratch>` (E2(c)): copies the main plant's Memory CSV byte for byte to a scratch plant and writes an EMPTY seed for it (sha256 null), not the main plant's seed. A copied seed makes the debrief merge (`tools/runtime_records.py`) refuse the scratch plant, because that plant has no tracked file; with the empty seed the merge archives the scratch rows under `temp/learn/<scratch>/` and the main Memory is untouched. Copy while no Attempt is running.
- Filed BUG-158 (first flight of BUG-138/139, Sitting row) and IDEA-31 (oval on the shared Memory needs `compile_oval`).

**Verification.** 2026-11-28, `flock /tmp/jugglebot-tests.lock ./run_tests.sh --full` on the combined change: parallel **11738 passed, 53 skipped, 2 xfailed in 944.80 s**, serial **16 passed in 61.05 s**, RESULT PASS. The audit's narrative fixes came after it; the readers of the edited files were then re-run (command and result in the commit message). The tool's importers: `grep -rn memory_scratch --include='*.py' .` finds only `tests/sim/test_memory_scratch.py`. On a copy of the live Memory the tool copied 5811 rows and the node's seed check (`runtime_paths._check_present`) read the scratch plant as kept, with no warning. Tests that name INVARIANTS.md only list its path (`test_night_shift_gate.py`); none quotes a row.
