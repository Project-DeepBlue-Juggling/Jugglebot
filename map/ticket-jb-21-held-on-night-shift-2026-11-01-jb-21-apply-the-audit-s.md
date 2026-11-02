---
id: JB-153
kind: ticket
title: 'JB-21 held on night-shift/2026-11-01-JB-21: apply the audit''s 2 fixes and merge, apply the WARNING only, merge as is, or drop?'
status: complete
created: '2026-11-03'
updated: '2026-11-02T22:47:13Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: ball-butler
blocked_by: []
links: []
parent: bug-bb-throw-outcome-carries-no-throw-identity
type: question
---

JB-21 has been held on `night-shift/2026-11-01-JB-21` since 2026-11-01. The branch appends ` release_wall_s=<epoch>` to `bb/throw_outcome`, and `skill_node` drops a verdict whose release falls outside the window it asked for. The audit left two behaviour-affecting findings open, and no question was filed then.

The first finding is a WARNING. The window's 0.75 s upper bound is measured from a clock read taken before `wait_for_service` and the round trip. Once the round trip passes about 0.8 s, a genuine verdict is dropped, so the attempt ends by its deadline and a genuine `NOT_SETTLED` is never retried. The second is a NOTE: the window is recorded after `_reload_throw_inflight` goes True, and a quick Stop and restart can overlap two windows.

## Options

1. Apply both and merge: `WINDOW_AFTER_S` 2.5 s with a clock re-read after `wait_for_service`, the window recorded where the inflight flag is set, and node-level tests for each (a unit, then a re-gate).
2. Apply the WARNING only (2.5 s and the re-read) and merge. The NOTE's gap is a stale suffixed verdict in a sub-tick window.
3. Merge as is: a slow service round trip can drop a genuine verdict.
4. Drop: keep today's unsuffixed message. The Item's own trigger, a late verdict misattributed, has not fired on hardware.

Recommendation: 1. Both fixes are a few lines each. With the WARNING open, the branch is worse than main on a slow round trip.

## What each answer unblocks

- 1 or 2: the Night Shift applies the fixes, re-gates and merges, and JB-21 closes.
- 3: a re-gate on the merged tip and a merge.
- 4: the branch is deleted and JB-21 goes back to open, waiting on its trigger.

## Answer

I agree with your recommendation
