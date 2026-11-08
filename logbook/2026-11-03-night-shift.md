---
title: "Night Shift 2026-11-03: 1 merged (BUG-53 commanded-position clock), 3 held (BUG-54 band rule as an effort-m slice; BUG-42 and BUG-2 carried, now with tickets), 6 question tickets, a BUG-57 Sitting section; the whole night ran on the subscription after the API credit ran out at launch"
type: investigation
date: 2026-11-03
status: complete
phase: "night-shift (third live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-03.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
  - report
---

# Night Shift 2026-11-03 (third live night)

**Counts.** There were 62 candidates; 15 Items were touched. Report: `docs/night-shift/2026-11-03.md`.

- **Merged 1:** BUG-53. The commanded-position freshness reads an injectable clock, the fixture freezes it, and no window was widened.
- **Held 3:**
  - BUG-54 on `night-shift/2026-11-03-BUG-54`. The owner's band rule is built; the gate said MERGE. It is held under the effort-m row of the dispatch table: the first slice, with the re-sweep still to do.
  - BUG-42 and BUG-2, carried from 2026-11-01. They now have their first tickets, TKT-29 and TKT-30.
- **Question tickets 6:** TKT-29 to TKT-34. Options (2026-11-03) were written on BUG-55, BUG-58 and FEAT-38.
- **Sitting sections 1:** BUG-57.
- **Rated 2:** BUG-54 and BUG-59.
- **Skipped 55**, each with a reason.

The BUG-7 Skipped reason is corrected. The 2026-11-02 report said its firmware is not in this repo, but `ros_ws/src/jugglebot/Teensy_code_platform/` holds it; it has no native test harness, so it is still out of reach offline.

**Nightly.** CLAIMED GREEN 7985/8080 passed, 0 failed, 0 errored, 0 xfailed, 95 skipped 2026-11-03T04:04:31+11:00.

**Auth.** The API attempt failed at 04:47 on its first call (HTTP 400, "Credit balance is too low", `2026-11-03.claude-api.json`) before doing any work. The runner resumed the same session on the Claude subscription (Opus), and the whole night ran there. Nothing was in flight at the switch.

## Protocol findings for the owner

- **The gate copy cannot import mapkit as written.** § 2g runs `python D/night_shift_gate.py` with `D` under `temp/`. Python puts the script's directory on `sys.path`, not the repo root, so the gate's `from mapkit.frontmatter import ...` fails (`ModuleNotFoundError`, exit 1). Tonight ran it as `PYTHONPATH=. python D/night_shift_gate.py ...`, which changes the environment and none of the gate's inputs. The same applies to any `python <file under temp/>` that imports mapkit. Fixing § 2g is a change to `.claude/` (gate-frozen at night), so it is the owner's.
- **A grep-built test list crossed the night's rules once.** The first BUG-53 scoped run took its file list from `grep -rln skill_node tests/`. That list included `tests/hardware/skills_plan_bench.py` and two `teensy_bridge_node` test files, which § 0 rule 2 keeps out of the night's command lines (the deny list matches command text, so a list built by substitution passes it). The run was stopped at once. It was replaced by a run over the five `test_skill_node*.py` files named explicitly, and the full gate covered the rest. No hardware was touched: they are offline unit tests the full gate runs anyway.
- **The deny list refused one `sed` edit** whose text named `tools/probes/`. It was redone with the Edit tool, as § 0 rule 2 intends.
- **The BUG-53 full gate failed once** (`test_no_avoid_words_in_live_docs`): the entry spelled the pattern name in prose. It was fixed in e7fde952, and that run is kept as `gate-JB-141-run1-avoid-word.log`.
- `night-shift/2026-11-01-BUG-31` is still on the remote although the owner merged its fix on 2026-11-01 (BUG-31 complete). It is not a Held row (its Item is complete), and the branch is left for the owner to delete.

## Verification

- BUG-53, `./run_tests.sh --full` on e7fde952, 2026-11-03, clone: 8023 passed, 53 skipped (parallel) plus 8 passed (serial), `EXIT=0`. The gate script (`origin/main` copy, `PYTHONPATH=.`) said `MERGE`, and the merge fast-forwarded and was pushed.
- BUG-54, `./run_tests.sh --full` on 24d18f8e, 2026-11-03, clone: 8030 passed, 53 skipped plus 8 passed (serial), `EXIT=0`. The gate script said `MERGE`, and the branch was held under the effort-m row and pushed at its gated tip.
- Map writes: each push followed `pytest -q tests/sim/test_map_items.py tests/sim/test_map_items_rated.py tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit_docs.py` (2026-11-03, the last run 176 passed, 39 skipped, together with `test_sitting_sections.py` and `test_question_tickets.py`).
