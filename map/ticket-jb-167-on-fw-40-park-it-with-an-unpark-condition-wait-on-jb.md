---
id: JB-198
kind: ticket
title: 'JB-167 on FW 40: park it with an unpark condition, wait on JB-149''s watch, add a firmware edge heartbeat, or stop the firmware adopting a refused frame?'
status: complete
created: '2026-11-07'
updated: '2026-11-06T22:52:36Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: subsystem
area: skill-stack
blocked_by: []
links: []
parent: feature-install-the-host-hold-within-one-tick-of-a-firmware
type: question
---

You asked in JB-179 how this Item fits now that FW 40 is built. FW 40 has flown (JB-174 passed, lost 0) and did not change the race, but the race has not bitten since: the 2026-11-06 Sittings had about 37 STOPs and no guard latch the debriefs attribute to one. Full table in `## Findings (2026-11-07)` on JB-167.

## What changed the picture

The firmware already holds a stopped lane itself. The latch comes from the host plan still streaming frames the firmware refuses. At 50 to 78 rev/s the Hand's 2.5 rev guard fills in about 32 to 50 ms (arithmetic, not measured), and the host hold lands 110 to 210 ms after the END (measured on FW 38/39). No host-side speed-up beats that.

## Options

1. Park it: the guard stays the authority and a latch costs one CLEAR_ERRORS. Unpark on a LANE_REFUSED or guard latch within 300 ms of a STOP on FW 40 with `lost 0`.
2. Wait on JB-149's `gc_freeze` arm and its `gc_probe` watch before deciding. Touches nothing.
3. Firmware edge heartbeat on a STOP, about 1 ms to the host, plus a host change. It still leaves the hold's service time. Safety-bearing, for your Session.
4. Stop the firmware adopting a refused frame as its base command. Removes the race at its source but reverses the "fails loud" design, so the guard no longer trips on a walking plan. Offline replay first.

Publish on change is withdrawn: it saves about 50 ms on average against a 32 to 50 ms guard.

Recommendation: 1. If the unpark condition fires, take 3 first, since 4 weakens the guard.

## What each answer unblocks

- 1: the next night parks JB-167 with that condition.
- 2: it stays open, waiting on JB-149.
- 3 or 4: a safety-bearing plan for your Session, with a `## Sitting` section.

## Answer

Agreed. Park it until the unpark condition fires.
