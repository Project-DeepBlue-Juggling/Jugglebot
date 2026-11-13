---
title: "Night Shift: a merge to main is bound to the gate's MERGE record (closeout C1)"
type: feature
date: 2026-11-13
status: complete
phase: "night-shift close-out (C1)"
subsystem: tools
files_changed:
  - tools/night_shift_gate.py
  - tools/night_shift_landed.py
  - tools/night_shift.sh
  - .claude/commands/night-shift.md
  - .claude/commands/sitting-debrief.md
  - tests/sim/test_night_shift_gate.py
  - tests/sim/test_night_shift_landed.py
  - tests/sim/test_night_shift_runner.py
---

# Night Shift: the merge is bound to the gate

## What changed and why

`tools/night_shift_gate.py` decided MERGE or HOLD, but nothing tied a push to `main` to that verdict. Four failures went through unseen: a merge after a HOLD, a merge with no gate run, a merge on an `EXIT=0` log written for another tip, and a merge commit edited by hand so content no gate saw rides in with a gated tip (an evil merge). Invariant: **every non-exempt commit that reaches origin/main during a night is covered by a MERGE verdict record.**

- **The gate's `--record [DIR]`** writes `DIR/<tip>.json` atomically (temp file, fsync, rename) for MERGE and HOLD alike: schema, verdict, item, branch, tip, base ref and sha, waived, reasons, the sha256 of the gate, the log and the audit, and `written_at`. DIR is `$NIGHT_SHIFT_VERDICTS`, else `~/.local/state/jugglebot/verdicts`. A MERGE whose record cannot be written prints HOLD. Without `--record`, every verdict is unchanged: the 135 existing gate tests pass unedited.
- **`tools/night_shift_landed.py`** is the single enforcement point. Over `head_before..origin/main`: a plain commit passes if every path is EXEMPT or if it is an ancestor of a MERGE tip. A merge passes if it is an ancestor of a MERGE tip (the re-gate's merge of `origin/main` into a branch), or if it carries no non-EXEMPT content of its own (`diff-tree --cc`) and one of its parents is a MERGE tip. EXEMPT is a literal list: `map/*`, `docs/night-shift/*`, `logbook/*`, and the six pages `mapkit gen` writes. A test runs `mapkit gen` to pin that list against its real outputs.
- **The runner** fetches and captures origin/main straight after claude and salvage, BEFORE `deploy_step`, because the deploy's reverts are the runner's own commits. It then runs the checker as extracted from `head_before` (so a push tonight cannot rewrite it), with exit 2, a timeout or a missing file all counting as ungated. If the night is otherwise OK, the flag goes into the report's Summary and Cost lines, the cost commit lands as usual, and the night ends `UNGATED_MERGE` (exit 1, ticker re-raised, the commit, author and paths in the detail). If the night failed some other way, the evidence is added to that status's detail. Either way `$REPORTS/ungated.hold` is written, and the next start is HELD until the owner deletes it. Nothing is reverted. `last_run.json` gains `landed` and `origin_after_claude`.
- **The protocol**: 2e is now the audit, with its narrative fixes committed first, and 2f is ONE full suite on that tip. This avoids the second full suite and the re-gate after audit fixes. 2g passes `--record "$NIGHT_SHIFT_VERDICTS"`. The cross-references were renumbered, along with the one in `sitting-debrief.md`.

Deviations from the design (`closeout-design.md` C1):
1. `CLAUDE.md` is NOT exempt, although `mapkit gen` writes its term block. The term block changes only with `GLOSSARY.md`, which the gate forbids, and `CLAUDE.md` is itself a gate-FORBIDDEN path. A test checks that no exempt path is forbidden.
2. The merge rule also accepts a merge that is an ancestor of a MERGE tip. Without that, every re-gated branch (protocol 2l) would be flagged.
3. The check runs right after claude for every post-claude outcome, not only after line 897. A night that crashes after an ungated push still writes the hold file.

Known gaps: a hand-forged record is believed (the follow-up replays the gate from the record). Commits a gated branch merged in from `main` are covered by its record. A merge that drops one side's change shows nothing under `--cc`. Two gated tips merged one after the other form a combination no suite ran on.

## Verification

- 2026-11-13, `python -m pytest tests/sim/test_night_shift_gate.py -q`: 140 passed in 249.53 s (135 unchanged + 5 new).
- 2026-11-13, `python -m pytest tests/sim/test_night_shift_runner.py tests/sim/test_night_shift_settings.py tests/sim/test_night_shift_preflight.py tests/sim/test_night_shift_report.py tests/sim/test_night_shift_candidate.py tests/sim/test_night_shift_deploy.py tests/sim/test_night_shift_landed.py tests/sim/test_sitting_debrief_protocol.py tests/sim/test_no_legacy_item_ids.py -q -n 4 --dist loadfile`: 272 passed in 188.55 s (15 landed-check tests, 5 runner tests new).
- 2026-11-13, `./run_tests.sh -q` from the closeout-c1 worktree (live_session clear): `RESULT: PASS`, parallel 660 s rc=0, serial 33 s rc=0, total 693 s.
