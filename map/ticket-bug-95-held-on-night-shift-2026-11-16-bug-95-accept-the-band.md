---
id: TKT-76
kind: ticket
title: 'BUG-95 held on night-shift/2026-11-16-BUG-95: accept the band-REST design with a cap on its rebase slide, accept it uncapped, log the band case only, or drop?'
status: complete
created: '2026-11-16'
updated: '2026-11-16T06:46:24Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-a-long-rest-can-be-refused-splice-too-late-the-fresh
type: question
---

BUG-95 is confirmed offline: after a long bridge REST (a Hand far from rest, lift 3.2 s or 6.5 s), the Feed's PRE-TILT REST can be dispatched while the running record ends between the install lead and the splice lead, so it clamps onto the record's last Knot and a slow solve ends the Attempt SPLICE_TOO_LATE.

The fix on night-shift/2026-11-16-BUG-95 makes one function decide fresh vs splice at all three sites, and starts a REST in that band at the record's end from its terminal rest, so a slow solve rebases instead of ending the Attempt.

## Options

1. **Accept the design:** a day Session amends INVARIANTS.md (K1 and I-INSTALL-1 now contradict the code), adds a cap on the band REST's rebase slide (the audit's main finding: it is bounded only by the solve deadline, so the Feed Catch can move later than the +0.16 to 0.18 s ever measured), maps the new ValueError path to UNREACHABLE and tests the MAGAZINE case; then a Sitting flies a long-lift Feed.
2. **Accept the design without the cap:** the same, but a slow solve's slide stays uncapped and is measured at the Sitting.
3. **Smaller fix instead:** keep today's behaviour but log the band case, and decide after a Sitting shows how often it happens.
4. **Drop the branch.**

## What each answer unblocks

- 1 or 2: a day Session (INVARIANTS.md is gate-frozen, so the night cannot merge it), then a Sitting. A `## Sitting` section is written once you choose.
- 3: the next night builds the logging change instead.
- 4: BUG-95 goes back to open.

Recommendation: 1. The cap keeps the Feed Catch inside timing that has flown, and the contract text has to change before the code it describes.

## Answer

I agree with your recommendation
