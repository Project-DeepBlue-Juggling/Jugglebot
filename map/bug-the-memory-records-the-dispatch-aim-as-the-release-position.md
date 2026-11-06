---
id: JB-194
kind: bug
title: The Memory records the dispatch aim as the release position, and a refused re-send leaves the release record at the refused aim
status: complete
created: '2026-11-06'
updated: '2026-11-06T08:48:29Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: subsystem
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- map/bug-two-ball-throws-land-about-9-mm-toward-the-partner-site-an.md
- logbook/2026-11-06-arrival-pull-and-hand-dropout-probes.md
- logbook/2026-11-06-memory-release-record-fixes.md
parent: learner-generalisation
filed_by: claude
---

The Memory row for a carried Throw is written once, at dispatch, from the aim then in force. A tracker re-aim afterwards moves the release the plan flies but not the row, and a refused re-send moves the bookkeeping while the plan keeps the old release. Found by the JB-192 probe; no command has been corrupted, because no Learner input reads release xy or the seat offset today.

## Findings (2026-11-06)

- On the P2 bag, `rel` equals the dispatch aim to 0.05 mm on all 98 carried rows, while 92 of them were re-aimed (73 RESEND, 19 BUMP) and the release flown differs by about 5 to 6 mm toward the partner.
- A refused re-send leaves `_carried_release_mm` and `_carried_catch_xy_mm` at the refused aim (`executor.py:4011` is written before the installer at `7198`; the refusal branch `7202-7219` does not restore). 23 of 98 rows had one. It skews the next Catch's dispatch prior, not the row.

## Proposed fix (not applied)

- F1: add `release_idx` to `_PendingOutcome` (`executor.py:1116`) and a helper `_refresh_pending_release(idx)`. Call it after each accepted re-aim (`_resend_live_catch` near `7219`, `_resend_hand_corrected_catch` near `6990`, the accepted branch of `commit_catch_bump` near `7375`). The row then records the Throw that flew.
- F2: snapshot `_carried_release_mm` and `_carried_catch_xy_mm` before `_catch_terminal` in both re-send paths and restore them when the re-send is refused.
- Neither changes a command, a Banking value or the admissibility margin of the installed plan, so neither shrinks the arrival pull of JB-192. F2 can change the next Catch's dispatch prior by at most the refused re-aim delta (35 mm), still under the same clamp and online admissibility check.
- Tests: an accepting and a refusing fake installer in `tests/motion/test_skills_executor.py`, an accepted BUMP in `test_skills_bump.py`, and the test at line 5190 rewritten to use the production path.

## Decision wanted

The owner's go-ahead, because F1 and F2 change what the Memory records and the next Catch's prior. Rows written before the fix carry the dispatch aim in `rel`; the cut-over date goes in the row docstring and the logbook.

## Progress

- 2026-11-06: Fixed 2026-11-06 (owner go-ahead): the pending row is refreshed after each accepted re-aim or bump, and a refused re-send restores the release record to the last accepted release. No command changes with the first; three tests fail on the parent and pass. Rows before this date hold the dispatch aim on a re-aimed Catch.
