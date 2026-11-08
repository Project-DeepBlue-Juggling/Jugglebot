---
id: TKT-41
kind: ticket
title: 'FEAT-40: publish on change saves at most 100 ms because the STOP arrives in a 10 Hz heartbeat: wait for FW 40, build it anyway, add a firmware edge heartbeat, shorten END to hold, or leave it?'
status: complete
created: '2026-11-06'
updated: '2026-11-06T12:01:39Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: feature-install-the-host-hold-within-one-tick-of-a-firmware
type: question
---

You agreed to "probe, then publish on change" in TKT-37. Tonight's read shows that advice was wrong: the firmware reports a STOP only in its 10 Hz heartbeat, so publishing on change saves at most 100 ms and cannot land the hold within one tick. The table is in Findings (2026-11-06) on FEAT-40.

## Options

1. Wait for FEAT-41's Sitting row: FW 40 may remove most of these STOPs, and the Item may then not be worth building.
2. Publish on change anyway: saves about 50 ms on average, needs no firmware, and flies at a Sitting.
3. Send a heartbeat at once on a STOP in firmware, plus 2: the STOP reaches the host in about a millisecond. Needs a flash and a Sitting, so it belongs to your Session.
4. Shorten the END to hold leg (110-210 ms), the largest term: first find out where that time goes. A night unit can read it from the code and the logs.
5. Leave it: the Teensy guard stays the authority.

Recommendation: 1, then 4 and 3 if the STOPs survive FW 40.

## What each answer unblocks

- 1: FEAT-40 waits on FEAT-41's row.
- 2 or 4: a night unit. 2 gets a Sitting row.
- 3: your Session.
- 5: FEAT-40 parks.

## Answer

FW 40 is built now. Check how this Item fits into the current architecture.
