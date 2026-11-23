---
id: BUG-135
kind: bug
title: Platform CAN receive queue overflows during the inclinometer re-probe stall
status: inbox
created: '2026-11-23'
updated: '2026-11-23T07:12:23Z'
owners: []
priority: low
effort: m
hardware: robot
radius: subsystem
diagnosis: confirmed
area: platform-teensy
blocked_by: []
links:
- logbook/2026-11-23-platform-imu-clock-early-rx-ring-leak.md
filed_by: claude
night_skip: frozen-path
night_skip_on: '2026-11-24'
night_skip_note: Platform firmware (frozen at the gate) and a flash; a day Session picks the acceptance filter or a sliced inclinometer probe.
---

The Platform's 256-slot CAN receive queue can overflow during the roughly 103 ms inclinometer re-probe stall at Attempt traffic rates. This is pre-existing and found while fixing BUG-132 (logbook entry 2026-11-23-platform-imu-clock-early-rx-ring-leak).

## Findings

- The stall only happens while the inclinometer is not ready. Up to about 720 frames can arrive in it, so the ring overwrites its oldest frames and drops them.
- Platform FW 15 does not change that. It makes it visible as `rx_ring_depth_max` 255 in the capture header.
- The sync clock is not affected, because 0x7DD is stamped in the interrupt.

## Proposed fix

- Hardware acceptance filter on the Platform. It must keep 0x6F0, the only reflash path (the USB port is dead), and every id the sketch tests, and the traffic counter changes meaning. Alternatively avoid the stall by probing the inclinometer in short slices.
