---
id: BUG-117
kind: bug
title: No test fails when the candidate build skips its merge abort after a conflict
status: inbox
created: '2026-11-18'
updated: '2026-11-18T15:51:23Z'
owners: []
priority: low
effort: s
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links: []
filed_by: claude
---

Removing the merge --abort at tools/night_shift_candidate.sh:137 fails no test in tests/sim/test_night_shift_candidate.py (checked 2026-11-18 during FEAT-58, on the file before and after that change). Wanted: a test in which one branch conflicts and a later branch is still tried, asserting the later branch merges cleanly and the candidate worktree holds no half-finished merge. The test file is on the Night Shift's frozen paths, so this needs a day Session.
