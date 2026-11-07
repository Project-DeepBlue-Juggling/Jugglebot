---
id: JB-87
kind: feature
title: Retire dead wire fields and literal flag masks; put teensy-udp-protocol.md in the docs nav
status: in_progress
created: '2026-10-28'
updated: '2026-11-07T19:01:34Z'
owners: []
priority: low
effort: s
hardware: none
owner: none
radius: cross-cutting
area: can-bridge
blocked_by: []
links: []
filed_by: claude
---

Small hygiene residue, one chore item. The `.action` edit needs a colcon rebuild and a launch restart.

- Dead `suppress_announcement` field (`ball_butler_node.py:595,800`, `BallButlerThrowCmd.action:20`) names the deleted can_node.
- Literal 0x8 / 0x30 heartbeat masks at `teensy_bridge_node.py:809-816` instead of `p.HeartbeatT2JFlags`.
- `docs/teensy-udp-protocol.md` is missing from the `mkdocs.yml` nav.
- Rename `HAND_LANE_REFUSED` to `LANE_REFUSED`.
- `gpio_poll.cpp:71-74` still says PROVISIONAL.

Source entries: [logbook/2026-06-08-bb-cutover-bench.md](logbook/2026-06-08-bb-cutover-bench.md), [logbook/2026-06-29-canbridge-phase0-native-harness.md](logbook/2026-06-29-canbridge-phase0-native-harness.md), [logbook/2026-09-21-agent-skills-config-and-docs-link-contract.md](logbook/2026-09-21-agent-skills-config-and-docs-link-contract.md), [logbook/2026-10-13-skill-stack-r5-wave-2-after-sitting-7.md](logbook/2026-10-13-skill-stack-r5-wave-2-after-sitting-7.md)

## Split (2026-11-07)

Night Shift can merge: the `mkdocs.yml` nav entry and the two heartbeat masks (now at `teensy_bridge_node.py:815` and `:820`, named constants that should read `p.HeartbeatT2JFlags`).

Night Shift builds and holds: removing `suppress_announcement`, because `.action` is in `jugglebot_interfaces`.

A day Session: the `HAND_LANE_REFUSED` rename, which touches the frozen `INVARIANTS.md`. The next firmware commit: the `gpio_poll.cpp` comment. Close this Item when all four are done.

## Progress

- 2026-11-08: Night Shift 2026-11-08: the heartbeat masks and the docs nav entry are done; still open are the action field, the rename and the firmware comment
