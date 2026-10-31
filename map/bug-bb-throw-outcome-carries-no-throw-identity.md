---
id: JB-21
kind: bug
title: bb/throw_outcome carries no throw identity
status: in_progress
created: '2026-10-24'
updated: '2026-10-31T18:45:04Z'
owners:
- night-shift
priority: medium
effort: s
hardware: none
owner: none
radius: subsystem
diagnosis: confirmed
area: ball-butler
blocked_by: []
links:
- plans/archived/skill-stack-dress-rehearsals-and-residue.md
parent: ball-butler
filed_by: claude
---

The `bb/throw_outcome` message does not say which Throw it reports (bb-self-cancel finding F2), so a late verdict can be misattributed. The fix is in `ball_butler_node` or the bridge relay. Trigger: a late verdict misattributed.

## Night Shift hold (2026-11-01)

The branch appends ` release_wall_s=<epoch>` to `bb/throw_outcome` and `skill_node` drops a verdict whose release falls outside the window it asked for. The audit's open WARNING: the window's 0.75 s upper bound is measured from a clock read before `wait_for_service` and the round trip, so a genuine verdict is dropped once the round trip exceeds about 0.8 s and the attempt ends by its deadline instead of same-tick. Suggested: upper bound 2.5 s and a clock re-read after the wait.

- A second NOTE: the window is recorded after `_reload_throw_inflight` goes True (a stale suffixed verdict in that gap takes the old path); a quick Stop and restart can overlap windows.
- The gate log also carried a main-side readability failure (JB-120 body), fixed on main the same hour.
- Entry: logbook/2026-11-01-jb21-bb-throw-outcome-identity.md.

## Progress

- 2026-11-01: Night Shift 2026-11-01: held on night-shift/2026-11-01-JB-21 (fix committed, Item closed on the branch): audit WARNING open on the release window bound; unblocked by the owner's call on it, then a re-gate (detail below)
