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

## Progress

- 2026-11-01: Night Shift 2026-11-01: held on night-shift/2026-11-01-JB-21 (fix committed with the Item closed on the branch): audit WARNING open, the release window's 0.75 s upper bound can drop a genuine verdict when the bb/throw_at_target round trip exceeds about 0.8 s (suggested 2.5 s and a clock re-read after wait_for_service), plus a fail-open gap before the window is recorded; the gate log also carried the main-side readability failure fixed in the same hour; unblocked by the owner's call on the window bound, then a re-gate
