---
id: JB-86
kind: bug
title: 'RestTerminal.holds_ball never reaches the node: every REST is rebuilt as holding a ball'
status: inbox
created: '2026-10-28'
updated: '2026-10-28T07:26:57Z'
owners: []
priority: low
effort: s
hardware: none
owner: none
radius: subsystem
diagnosis: confirmed
area: skill-stack
blocked_by: []
links: []
parent: two-ball-oval
filed_by: claude
---

`RestTerminal.holds_ball` never reaches the node: `tests/ros/test_install_segment.py:1413-1417` excludes it by name, so every REST is rebuilt as holding a ball. That over-constrains an empty-cup REST's cup-contact floor. Conservative: it costs refusals, not safety.

- Not filed: the `skills/check` live-state rehearsal (owner declined).
- The closing-REST LIMIT_JERK is already JB-24.

Source entries: [logbook/2026-10-02-skill-stack-r5-sitting-2.md](logbook/2026-10-02-skill-stack-r5-sitting-2.md), [logbook/2026-10-06-skill-stack-r5-sitting-6-start-state-refusals.md](logbook/2026-10-06-skill-stack-r5-sitting-6-start-state-refusals.md), [logbook/2026-09-14-skill-stack-r3-apex-ladder-prep.md](logbook/2026-09-14-skill-stack-r3-apex-ladder-prep.md)
