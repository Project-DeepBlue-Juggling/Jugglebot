---
title: "JB-34: logbook open-entries backfill survey, 103 entries flipped to resolved, six items filed"
type: refactor
date: 2026-10-28
status: resolved
phase: "inbox-wall"
files_changed:
  - logbook/artifacts/2026-10-28-jb34-open-entries-survey.csv
  - logbook/artifacts/2026-10-28-jb34-synthesis.md
  - logbook/INDEX.md
  - "93 + 10 logbook entries (frontmatter status plus a closing line), see the survey CSV"
  - map/idea-logbook-open-entries-backfill.md
  - "map/ items JB-82 to JB-87 (filed), JB-73 (retitled), JB-76 (note)"
  - plans/active/two-ball-oval.md
  - docs/project-map.md
subsystem:
  - docs
tags:
  - logbook
  - project-map
---

## Summary

The logbook had 228 entries dated before 2026-10-21 whose status was `in-progress`, `tuned` or `resolved-with-open-items` (102), or that were resolved but still carried an open-questions or follow-up section (126). Seven survey agents read them (102 status-flagged entries in depth, 126 skimmed; 11 entries after 2026-10-20 are live work and were skipped) and each got a verdict. The discussion artefacts are `logbook/artifacts/2026-10-28-jb34-synthesis.md` and `logbook/artifacts/2026-10-28-jb34-open-entries-survey.csv`.

Verdicts after the eight evidence-backed corrections (228 rows): SUPERSEDED 99, CLOSED 68, STALE 29, LIVE 16, STILL-OPEN 16. The 16 STILL-OPEN entries group into 12 themes, and nothing tracked their residue.

## Outcome

- **Filed (6):** JB-82 (bug, teensy_bridge_node shutdown Stow vs the launch's 5 s SIGTERM, and the missing single-owner interlock), JB-83 (bug, CAN3 analog drive path hardware fault), JB-84 (bug, Ball Butler over-CAN flash calls the retired Jugglebot-skills worktree; the fix is in the BallButler repo), JB-85 (feature, kinematic calibration acceptance: flying frame check and the Fit's -z gravity assumption; Pathway learner-generalisation), JB-86 (bug, `RestTerminal.holds_ball` never reaches the node; Pathway two-ball-oval), JB-87 (feature, dead wire fields, literal heartbeat masks, docs nav).
- **Folded (3):** theme A into JB-73, retitled "Hand ODrive state can silently diverge from what the can-bridge commanded" (vel_ff stays the first instance, three findings appended); theme I as a dated note on JB-76 (mocap_node priority/pinning, `TRACKING_MAX_FRAMES` revisit) (JB-23 and JB-31, also named by the synthesis, were not touched: nothing new to add); theme J as one open-question line in `plans/active/two-ball-oval.md` (offline replica cup-seating-cone angle 16.5-21.7 degrees vs hardware 31.6-41.7 degrees).
- **Dropped (3):** H, K, L, nothing filed.
- **Flips:** 93 entries from `flips.csv` plus the 10 STILL-OPEN full-depth entries whose residue is now filed (closing line names the JB-n) = 103 entries set to `status: resolved`, each with one `> Closed 2026-10-28: ... (JB-34 survey).` line at the end; the same 103 `logbook/INDEX.md` rows carry the new status (104 matched rows, one entry has two). Two LIVE entries stay open on purpose: `2026-10-18-learner-generalisation-investigation` (closes with plan P5) and `2026-10-19-cup-seating-cone-refuses-columns-stroke-braking` (the amended cone rule was not confirmed flown).

## Decisions

- **`resolved`, never `superseded`.** `superseded` is not on the documented ladder (`logbook/README.md` status ladder); one precedent entry and a report colour exist. The closing line carries the word, so nothing is lost and no normative doc is edited.
- **H dropped (hand ball sensor tuning):** the sensor has run every Sitting since 2026-08 and its tuning rows live in `tests/hardware/session_hand_ball_sensor.md`. **K dropped (GUI producer-liveness test):** no GUI topic regression since July. **L dropped (unreconciled constants, watch items):** each is documented, bounded or harmless and none recurred.
- **E2 declined by the owner:** the `skills/check` live-state rehearsal is not filed. Only the `holds_ball` wire gap is (JB-86).
- **Fold, not file, for A:** it is the same class as JB-73 (ODrive-side Hand state the can-bridge cannot see); the cost is that JB-73 becomes an enumeration.
- **`resolved` on a handed-off entry means "residue handed to the named JB item", not "fixed":** the 10 STILL-OPEN entries and the four that point at JB-73 close with a line naming an item that is still `inbox`; the work is open there, and a grep for `status: resolved` must not be read as fixed.
- **Nothing after 2026-10-20 is touched**, since the nightly and live sessions work on those entries.

## Verification

- 2026-10-28, `pytest -q tests/sim/test_map_items.py tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit_docs.py tests/sim/test_plans_index.py` (after the map edits): 34 passed.
- 2026-10-28, `./run_tests.sh -q` (worktree): PASS, parallel 369 s + serial 23 s, total 392 s, both rc=0. The only edit after that run was this entry's Verification section; `pytest tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py tests/sim/test_glossary_avoid_words.py -q` re-run afterwards: 37 passed.
