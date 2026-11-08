---
id: BUG-29
kind: bug
title: 'RestTerminal.holds_ball never reaches the node: every REST is rebuilt as holding a ball'
status: inbox
created: '2026-10-28'
updated: '2026-11-06T23:52:42Z'
owners: []
priority: low
effort: s
hardware: robot
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
- The closing-REST LIMIT_JERK is already BUG-5.

Source entries: [logbook/2026-10-02-skill-stack-r5-sitting-2.md](logbook/2026-10-02-skill-stack-r5-sitting-2.md), [logbook/2026-10-06-skill-stack-r5-sitting-6-start-state-refusals.md](logbook/2026-10-06-skill-stack-r5-sitting-6-start-state-refusals.md), [logbook/2026-09-14-skill-stack-r3-apex-ladder-prep.md](logbook/2026-09-14-skill-stack-r3-apex-ladder-prep.md)

## Review (2026-11-07)

Re-rated `hardware: robot`. `holds_ball=False` removes the cup-contact floor from a REST entirely (`cup_cycle.py`, the contact window is `None`), and live schedules already send `False` (`schedule.py:2688`, `executor.py:5791`), so wiring it through changes flown motion. It must fly before it merges.

The wire default matters: rosidl defaults a new bool to `false`, which would drop the floor for any sender that omits the field, the same trap `InstallSegment.srv` records for `*_set`. Give the field a default of holding a ball (`bool rest_holds_ball true`) or invert its name.

Night Shift may build this on a branch. The gate holds it at (f), because `jugglebot_interfaces` is frozen: a day Session merges after the two-package build.
