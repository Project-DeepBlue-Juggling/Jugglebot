---
id: PWAY-12
kind: pathway
title: 'Stream delivery: no Attempt lost to a host delay'
status: active
created: '2026-11-29'
updated: '2026-11-28T14:49:26Z'
owners: []
priority: high
effort: l
hardware: robot
owner: hands
radius: normative
area: can-bridge
blocked_by: []
links:
- map/bug-teensy-bridge-node-forwards-only-the-newest-waiting-knot.md
- map/feature-install-the-host-hold-within-one-tick-of-a-firmware.md
- map/bug-40-hz-setpoint-stream-starved-past-the-firmware-s-70-ms.md
- map/feature-solver-worker-on-sched-fifo-take-the-catch-solve-out-of-the.md
destination: 150 consecutive two-ball Attempts on the robot with none lost to a firmware STOP, a refused lane or a Hand MAX_DEVIATION latch caused by late delivery of the Knot stream, and a late frame can no longer latch the Guard.
---

Plan plans/active/stream-delivery-contract.md. Three layers: prevent (the forwarding path leaves the rclpy process), tolerate (FW 54 frames of 7 Knots, 4 committed) and fail soft (the firmware reports a STOP at once and the forwarder holds). Evidence and the design are in the plan and its logbook entry.
