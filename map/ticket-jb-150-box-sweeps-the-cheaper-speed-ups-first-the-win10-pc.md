---
id: JB-156
kind: ticket
title: 'JB-150 box sweeps: the cheaper speed-ups first, the Win10 PC over SSH, the PC by hand, or leave it?'
status: open
created: '2026-11-03'
updated: '2026-11-02T18:15:22Z'
owners: []
priority: low
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: feature-run-admissible-box-sweeps-on-the-win10-desktop-about-2-5-h
type: question
---

JB-150 records that an admissible-box re-sweep takes about 2.5 h on the Jetson and holds every core, so a sweep and a Sitting cannot share the box. The body proposes the Win10 desktop and lists cheaper speed-ups that need no PC.

The body says it is not scheduled. This ticket gives you one place to answer when you take it up.

## Options

1. Cheaper speed-ups first, as a Night Shift unit: compare a rotating sample of 3 families in round b instead of all 29 (rounds a and b have been identical on every sweep), and profile `tools/admissible_sweep.py`. This roughly halves the wall time with no new machine.
2. Sweep on the PC over SSH: Windows OpenSSH server, an LF checkout, pinned numpy, and a Jetson-side certification pass before a box is committed.
3. Sweep on the PC by hand (PowerShell or WSL2), with the result back by git, and the same certification pass.
4. Leave it until a sweep blocks a Sitting again.

Recommendation: 1 now, then 2 if the halved sweep still blocks Sittings. Option 1 needs only your yes. Options 2 and 3 also need the PC's CPU and core count, and your call on whether a Jetson certification pass is enough for the record.

## What each answer unblocks

- 1: a tools-only unit the Night Shift can build and gate.
- 2 or 3: a plan under `plans/` and the PC setup in your Session.
- 4: the Item stays inbox.
