---
id: FEAT-60
kind: feature
title: 'Confirm the 2026-11-19 nightly ran the 59 GUI tests: skipped count at or under 60'
status: inbox
created: '2026-11-18'
updated: '2026-11-18T09:22:32Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: none
radius: local
area: tests-tooling
blocked_by: []
links: []
filed_by: claude
---

One-off check for the Night Shift of 2026-11-19. BUG-103 (commit b90f31b3, 2026-11-18) made 59 GUI tests find Node, so they should run at 04:00 for the first time.

## Check

- Read the `--nightly` line and the `--nightly-report` copy given to this night.
- Baseline: the 2026-11-18 run had 113 skipped.
- Pass: the 2026-11-19 run has 60 or fewer skipped, and no skip reason in the report mentions Node.
- Fail: more than 60 skipped, or any Node skip reason.
- No run (DEFERRED or a stale date): leave this Item open and say so in the report.

## On pass

- Add a `## Progress` line with the skipped count and set this Item complete. No code change.

## On fail

- Leave this Item open. Put the skipped count and the Node skip reasons in the night's report under the owner's questions.
- Do not edit `tools/nightly_*` or `tools/systemd/*`: the 04:00 unit's PATH is the likely cause and is frozen for the night.
