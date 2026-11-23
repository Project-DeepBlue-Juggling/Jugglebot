---
id: IDEA-26
kind: idea
title: 'Platform IMU residue: unflown checks and the unmeasured lever arm'
status: inbox
created: '2026-11-23'
updated: '2026-11-23T15:48:50Z'
owners: []
priority: low
effort: m
hardware: robot
owner: hands
radius: local
area: platform-teensy
blocked_by: []
links:
- plans/archived/platform-imu.md
filed_by: claude
---

Left open when plans/archived/platform-imu.md was archived on 2026-11-23 (IDEA-20 closed). None blocks use of the captures.

- Lever arm: the IMU position is the CAD value (5, -115, 18.5) mm. The Jog moves of 2026-11-22 could not fit it; a slow pure-rotation sweep would.
- Unflown: a forced gap in a capture, and an Armed upload after a Fault.
- The Platform-frame chart labels are not yet checked in a browser.
- One whole 256-word block goes missing on the first pass of each 1 kHz upload at rest. The refill heals it; the cause is unknown.
- The wall-clock chain could be tightened (stamp sync frames nearer the receive interrupt).
- The encoder-versus-Mocap ring-frequency gap (5.4 against 4.4 to 4.7 Hz) is for the IMU analysis to settle.

## Feasibility (2026-11-24)

- What it would take: each bullet is its own small job, and all but one need the robot. The lever arm needs a slow pure-rotation sweep on the Platform. The forced gap and the Armed upload after a Fault are Sitting steps. The chart labels need eyes in a browser.
- The one offline part is the ring-frequency gap (5.4 Hz from the encoders against 4.4 to 4.7 Hz from Mocap): an analysis of captures already taken, with no code change on the robot path.
- Effort: m in total, s per bullet. Hardware: robot for four of six. Owner: hands for the Sitting steps.
- One risk: the missing 256-word block on the first pass of each 1 kHz upload has an unknown cause; the refill hides it, so a change elsewhere in the upload path could make it permanent without any test noticing.
