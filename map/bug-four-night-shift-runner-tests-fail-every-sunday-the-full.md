---
id: BUG-107
kind: bug
title: 'Four Night Shift runner tests fail every Sunday: the full walk turns their expected NOTHING_NEW into OK'
status: inbox
created: '2026-11-15'
updated: '2026-11-14T17:47:41Z'
owners: []
priority: high
effort: xs
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links: []
parent: night-shift
filed_by: claude
---

Four tests in `tests/sim/test_night_shift_runner.py` fail on every Sunday, so the Sunday 04:00 nightly is RED and every Night Shift gate run on a Sunday fails condition (a). Found by the Night Shift of 2026-11-15: the nightly at 04:04 failed exactly these four, and the same test failed in the night clone at 04:50.

## Findings

- `tools/night_shift.sh:832` makes a run a FULL WALK when `${NIGHT_SHIFT_WEEKDAY:-$(date +%u)}` is 7 (closeout C4, commit ae4ae9a1, 2026-11-14).
- The newer tests pass `NIGHT_SHIFT_WEEKDAY=3` explicitly (`test_night_shift_runner.py:1327-1380`), but four older ones call `ns.run()` without it and then expect `NOTHING_NEW` on a second run with nothing changed. On a Sunday the second run is a full walk, so it ends `OK` instead.
- Failing: `test_first_run_then_nothing_new_then_change`, `test_post_run_snapshot_is_tomorrows_baseline`, `test_a_night_with_no_commit_of_its_own_gets_a_history_commit`, `test_a_rejected_push_heals_the_next_night_and_a_rerun_is_one_row`.
- Reproduced: `tools/night_shift_nonet.sh python -m pytest tests/sim/test_night_shift_runner.py -k first_run_then_nothing_new` fails at line 335 with `night_shift: OK ...` where `NOTHING_NEW` was expected (2026-11-15 04:50).

## Wanted

Pin the weekday in the `ns` fixture's default environment (`NIGHT_SHIFT_WEEKDAY=3`), so only the tests that ask for a Sunday get one. The file is on the Night Shift's never-list, so a day Session makes this change. Until it lands, no Night Shift branch can merge on a Sunday.
