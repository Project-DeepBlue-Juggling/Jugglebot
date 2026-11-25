---
id: BUG-142
kind: bug
title: Hand park reports already parked on a Hand that is Idle on its bottom stop
status: inbox
created: '2026-11-25'
updated: '2026-11-25T16:01:28Z'
owners: []
priority: medium
effort: xs
hardware: robot
owner: hands
radius: local
diagnosis: hypothesis
area: hand
blocked_by: []
links: []
filed_by: claude
night_skip: owner-session
night_skip_on: '2026-11-26'
night_skip_note: FEAT-1 phase 1 fixes it (the no-op requires Closed-loop); FEAT-1's plan waits on your decisions O1 to O7.
---

`_park_hand` reports "already parked" on position alone, within 0.10 rev of 0.0, and the Hand's bottom stop is at -0.10 rev, on that band's edge. Inferred from code while writing the FEAT-1 plan (2026-11-24); not seen on hardware.

## Findings

- A Hand that the stall guard Idled and that fell onto its stop can be reported parked and left Idle after a clear.
- The next Attempt would then stream to an Idle Hand, which ends in a MAX_DEVIATION Soft E-stop at the first long stroke.
- The HAND_STALL operator line says the clear's park re-energises the Hand, which this case contradicts.
- The FEAT-1 plan's first phase fixes it: the no-op requires Closed-loop, and the install gate refuses a Hand that is not engaged.

## Wanted

- Fixed by FEAT-1 phase 1, or sooner on its own: require Closed-loop for the "already parked" answer.
