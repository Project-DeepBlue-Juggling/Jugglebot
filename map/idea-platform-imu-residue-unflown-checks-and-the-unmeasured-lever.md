---
id: IDEA-26
kind: idea
title: 'Platform IMU residue: unflown checks and the unmeasured lever arm'
status: inbox
created: '2026-11-23'
updated: '2026-11-23T04:01:32Z'
owners: []
priority: low
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
