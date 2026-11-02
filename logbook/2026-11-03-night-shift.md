---
title: "Night Shift 2026-11-03: 1 merged (JB-141 commanded-position clock), 3 held (JB-143 band rule as an effort-m slice; JB-120 and JB-21 carried, now with tickets), 6 question tickets, a JB-147 Sitting section; the whole night ran on the subscription after the API credit ran out at launch"
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

- **Merged 1:** JB-141. The commanded-position freshness reads an injectable clock, the fixture freezes it, and no window was widened.
- **Held 3:**
  - JB-143 on `night-shift/2026-11-03-JB-143`. The owner's band rule is built; the gate said MERGE. It is held under the effort-m row of the dispatch table: the first slice, with the re-sweep still to do.
  - JB-120 and JB-21, carried from 2026-11-01. They now have their first tickets, JB-152 and JB-153.
- **Question tickets 6:** JB-152 to JB-157. Options (2026-11-03) were written on JB-145, JB-149 and JB-150.
- **Sitting sections 1:** JB-147.
- **Rated 2:** JB-143 and JB-151.
- **Skipped 55**, each with a reason.

The JB-26 Skipped reason is corrected. The 2026-11-02 report said its firmware is not in this repo, but `ros_ws/src/jugglebot/Teensy_code_platform/` holds it; it has no native test harness, so it is still out of reach offline.

**Nightly.** CLAIMED GREEN 7985/8080 passed, 0 failed, 0 errored, 0 xfailed, 95 skipped 2026-11-03T04:04:31+11:00.

**Auth.** The API attempt failed at 04:47 on its first call (HTTP 400, "Credit balance is too low", `2026-11-03.claude-api.json`) before doing any work. The runner resumed the same session on the Claude subscription (Opus), and the whole night ran there. Nothing was in flight at the switch.

## Protocol findings for the owner

- **The gate copy cannot import mapkit as written.** § 2g runs `python D/night_shift_gate.py` with `D` under `temp/`. Python puts the script's directory on `sys.path`, not the repo root, so the gate's `from mapkit.frontmatter import ...` fails (`ModuleNotFoundError`, exit 1). Tonight ran it as `PYTHONPATH=. python D/night_shift_gate.py ...`, which changes the environment and none of the gate's inputs. The same applies to any `python <file under temp/>` that imports mapkit. Fixing § 2g is a change to `.claude/` (gate-frozen at night), so it is the owner's.
- **A grep-built test list crossed the night's rules once.** The first JB-141 scoped run took its file list from `grep -rln skill_node tests/`. That list included `tests/hardware/skills_plan_bench.py` and two `teensy_bridge_node` test files, which § 0 rule 2 keeps out of the night's command lines (the deny list matches command text, so a list built by substitution passes it). The run was stopped at once. It was replaced by a run over the five `test_skill_node*.py` files named explicitly, and the full gate covered the rest. No hardware was touched: they are offline unit tests the full gate runs anyway.
- **The deny list refused one `sed` edit** whose text named `tools/probes/`. It was redone with the Edit tool, as § 0 rule 2 intends.
- **The JB-141 full gate failed once** (`test_no_avoid_words_in_live_docs`): the entry spelled the pattern name in prose. It was fixed in e7fde952, and that run is kept as `gate-JB-141-run1-avoid-word.log`.
- `night-shift/2026-11-01-JB-89` is still on the remote although the owner merged its fix on 2026-11-01 (JB-89 complete). It is not a Held row (its Item is complete), and the branch is left for the owner to delete.

## Verification

- JB-141, `./run_tests.sh --full` on e7fde952, 2026-11-03, clone: 8023 passed, 53 skipped (parallel) plus 8 passed (serial), `EXIT=0`. The gate script (`origin/main` copy, `PYTHONPATH=.`) said `MERGE`, and the merge fast-forwarded and was pushed.
- JB-143, `./run_tests.sh --full` on 24d18f8e, 2026-11-03, clone: 8030 passed, 53 skipped plus 8 passed (serial), `EXIT=0`. The gate script said `MERGE`, and the branch was held under the effort-m row and pushed at its gated tip.
- Map writes: each push followed `pytest -q tests/sim/test_map_items.py tests/sim/test_map_items_rated.py tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit_docs.py` (2026-11-03, the last run 176 passed, 39 skipped, together with `test_sitting_sections.py` and `test_question_tickets.py`).
