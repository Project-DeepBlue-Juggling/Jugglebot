---
id: JB-21
kind: bug
title: bb/throw_outcome carries no throw identity
status: inbox
created: '2026-10-24'
updated: '2026-10-24T13:05:00Z'
owners: []
priority: medium
blocked_by: []
links:
- plans/archived/skill-stack-dress-rehearsals-and-residue.md
parent: ball-butler
filed_by: claude
---

The `bb/throw_outcome` message does not say which Throw it reports (bb-self-cancel finding F2), so a late verdict can be misattributed. The fix is in `ball_butler_node` or the bridge relay. Trigger: a late verdict misattributed.
