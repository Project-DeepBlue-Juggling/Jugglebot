---
title: "/sitting-debrief brings a passed branch up to date and re-runs the audit, suite and gate before merging"
type: feature
date: 2026-11-07
status: complete
phase: night-shift
related_plan:
  - night-shift.md
files_changed:
  - .claude/commands/sitting-debrief.md
  - tools/night_shift_gate.py
  - tests/sim/test_sitting_debrief_protocol.py
  - tests/sim/test_night_shift_gate.py
  - plans/active/night-shift.md
subsystem:
  - night-shift
tags:
  - night-shift
  - merge-gate
  - sitting-debrief
---

# /sitting-debrief updates a passed branch before merging

**What.** Owner's decision, closing the open question in [2026-11-07-night-shift-out-of-reach-opened.md](2026-11-07-night-shift-out-of-reach-opened.md). A held branch the owner passed at a Sitting is now merged like this: bring `main` into a throwaway copy of the branch, audit that tip, run `./run_tests.sh --full` on it with no network, gate it with `--waive c`, merge. The remote branch stays as it is, because that is what flew. Later Items build on earlier ones, because the base is local `main`.

**Why.** The gate requires a branch to contain its base, and `main` moves many times a day, so a pass almost never merged. It stayed held until a Session merged it by hand, without the gate. A Sitting pass says the flown code works; only this says it still works beside what has landed since. Cost: about 10 minutes of suite per branch, plus an audit.

**A rule that had to change with it.** Section 0 said a conflict is never resolved by taking either side. A rehearsal on the real held branches showed every one conflicts on its own Item file, and often on `logbook/INDEX.md` and `docs/project-map.md`, so that rule would have held every branch. These three, and no others, now have a mechanical answer: `main`'s Item (it holds the Sitting result), both sides' INDEX rows, and regenerated pages. A conflict in anything else still holds the branch.

**Left as it was.** A branch that touches a forbidden path still fails condition (f); merging it is a day decision by the owner, named in the summary as not gated. The merged result is still not the code that flew, and no gate can make it so.

## Verification

- Rehearsal on real held branches (2026-11-07, a throwaway clone, the section's commands as written, with a stub in place of the suite and audit): BUG-77 conflicted on its Item, `logbook/INDEX.md` and `docs/project-map.md`; all resolved mechanically; `pytest tests/sim/test_map_items.py tests/sim/test_map_items_rated.py tests/sim/test_mapkit_docs.py tests/sim/test_logbook_front_matter.py tests/sim/test_plans_index.py` on the merged tree: 198 passed, 46 skipped; gate `MERGE (c waived)`; `git merge --ff-only` landed; the remote branch unchanged. BUG-67 conflicted on its Item only. A real debrief has not used this yet.
- Gate semantics (2026-11-07, `test_the_debriefs_update_then_gate_reaches_a_merge`): the night's results for the old tip hold, and fresh results on the updated tip merge with only (c) waived.
- One audit (`audit-reporter`, Sonnet, 2026-11-07), its behaviour findings applied after the rehearsal: `BASE` is kept in a file because shell variables do not survive between tool calls (empty, the audit would have reviewed an empty diff and passed); main's copy is taken by naming the commit, because in this merge direction `--ours` is the branch; section 3 starts on `main`; a red gate or rejected push names the unpushed merges and never resets them. The rehearsal ran before these edits; the gate and protocol tests below ran after them.
- Default gate, in the namespace, on 2a907b46 (2026-11-07, `tools/night_shift_nonet.sh ./run_tests.sh`): parallel 8980 passed, 59 skipped in 548.96 s; serial 7 passed in 28.05 s; `RESULT: PASS`. The run before it failed one test, its pattern for `git merge --abort`, which the audit's fix had correctly changed; fixed and re-run. An earlier run was stopped at about 71 % at the owner's request during a Sitting.
