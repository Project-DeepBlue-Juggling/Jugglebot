---
id: JB-154
kind: ticket
title: 'JB-145 Hand stall stop near a release: no grace for a fast upward Hand, that plus a C1-entry stop at the brake authority, fix the starvation instead, or model first?'
status: complete
created: '2026-11-03'
updated: '2026-11-02T22:49:45Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: can-bridge
blocked_by: []
links: []
parent: bug-hand-stall-stop-near-a-release-cannot-fit-the-stroke-the-20
type: question
---

JB-145 records that a scheduled stream running dry near a release cannot stop the Hand inside its stroke. The 20 ms grace plus the C2 stop overrun the 10.501 rev clip by 4-10 rev in the model. The trigger, a forward gap over 70 ms, happened three times on 2026-11-02 (JB-149).

Night Shift 2026-11-03 drafted the options into the Item's `## Options (2026-11-03)` section. All four are firmware or probe work for the owner's Session.

## Options

1. No grace for a fast upward Hand: the stop starts at the cover end.
2. Option 1 plus a C1-entry stop at the plan's brake authority (4500 rev/s^2).
3. Fix the starvation (JB-149) and leave the stop as is, with the JB-114 bumper as the backstop.
4. Model options 1 and 2 with `tools/probes/hand_stall_stop.py` first, then pick.

Recommendation: 4 then 2.

## What each answer unblocks

- 1 or 2: a firmware change with a native test, then a bench-hand Sitting.
- 3: JB-145 waits on JB-149.
- 4: a probe run (offline), whose result picks between 1 and 2.

## Answer

I agree with your recommendation
