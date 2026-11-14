---
title: "Night Shift runner housekeeping: it prunes merged branches, commits a run history, and commits under its own committer"
type: feature
date: 2026-11-14
status: complete
phase: "night-shift close-out"
subsystem: tools
files_changed:
  - tools/night_shift.sh
  - tests/sim/test_night_shift_runner.py
  - docs/night-shift/README.md
  - logbook/INDEX.md
---

## What

Close-out C2. Three things the owner used to do by hand or could not see.

- **Prune.** After a passing landed check and never-list scan (not a dry run, not a skipped
  check) the runner deletes each origin `night-shift/*` branch whose tip is an ancestor of
  `origin/main` (`git push --force-with-lease=<name>:<tip> origin :<name>`), at most 20 a night,
  and `git branch -d`s merged local ones. Only `night-shift/*` is looked at; tonight's salvage
  branch is kept. Names go to `last_run.json` `pruned`, failures to `prune_failed`, and a
  failure never fails the night. The never-list scan has already run, and a deleted branch is
  gone from the next night's pre-claude snapshot, so a pruned branch is never new or moved.
- **History.** `write_status` appends the whole summary to `$REPORTS/history.jsonl`;
  `docs/night-shift/history.jsonl` is rebuilt from it (one row per run id (start time and pid), later row
  wins) and rides in the cost commit, the stub commit, or its own `docs(night-shift): run
  history` commit (clean main only, else `history_commit` says why not). The row leaves out
  `report`, `candidates`, `result` and `history_commit`, shows `$HOME` as `~`, and redacts
  credential values. `docs/night-shift/*` is EXEMPT, and the commit comes after the landed check.
- **Committer.** `GIT_COMMITTER_NAME/EMAIL` are exported for the run, so the runner's, claude's
  and the subagents' commits have committer `Jugglebot Night Shift
  <night-shift@noreply.jugglebot.invalid>` and the author is still the clone's config. No
  `user.*` is written, and `night_shift_landed.py` is untouched.

## Why not the obvious alternative

Setting `user.name`/`user.email` in the clone would change the author too, and git has no
committer-only config key; the environment variables are the one mechanism that reaches every
child process. A committed row cannot hold the night's final status (the commit precedes
`write_status`), so it is provisional and the next rebuild replaces it from the local file.

## Verification

- 2026-11-14, `python -m pytest tests/sim/test_night_shift_runner.py tests/sim/test_night_shift_landed.py tests/sim/test_night_shift_settings.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py -q -n 2`: **286 passed in 89.92s**; with the two gated files, `test_live_session_guard.py` and `test_night_shift_gate.py`, 156 passed (65 s).

## Published detail (same day)

The repository is public and a failed night's `detail` can carry the tail of claude's or the preflight's stderr. The committed `history.jsonl` row therefore publishes `detail` only for an OK night; any other status gets a fixed "not published" text and the full row stays in the reports directory on the box. Verification (2026-11-14): `python -m pytest tests/sim/test_night_shift_runner.py tests/sim/test_night_shift_landed.py -q -n 3`: 126 passed.
