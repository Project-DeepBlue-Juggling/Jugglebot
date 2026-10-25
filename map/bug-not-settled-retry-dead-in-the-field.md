---
id: JB-25
kind: bug
title: NOT_SETTLED retry is dead in the field
status: inbox
created: '2026-10-24'
updated: '2026-10-24T13:05:00Z'
owners: []
priority: low
blocked_by: []
links:
- plans/archived/skill-stack-dress-rehearsals-and-residue.md
parent: ball-butler
filed_by: claude
---

The `NOT_SETTLED` retry never fires in the field; it fails closed as `REJECTED_BB(THROW_ABORTED_NOT_SETTLED)`. A real retry is design work (bb-self-cancel finding F1). Trigger: a Ball Butler verdict race observed.
