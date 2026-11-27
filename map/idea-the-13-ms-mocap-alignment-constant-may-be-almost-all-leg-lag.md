---
id: IDEA-30
kind: idea
title: The 13 ms Mocap alignment constant may be almost all Leg lag, leaving little for Mocap latency
status: inbox
created: '2026-11-27'
updated: '2026-11-27T07:06:44Z'
owners: []
priority: low
effort: s
hardware: none
owner: none
radius: subsystem
area: tracking
blocked_by: []
links:
- bug-legs-trail-the-command-by-13-to-15-ms-on-2026-11-27-against
filed_by: claude
---

tools/probes/_bag_time.py aligns /mocap_data to the bridge clock with a 13.05 ms constant measured from body motion against the FK of the executed command. It was read as Mocap latency plus a modelled Leg lag of 1.3 to 3 ms. BUG-151 measured the Leg lag directly on 2026-11-27 at 13.2 ms (sd 0.5, 10 oval Attempts). If both numbers hold, almost none of the constant is Mocap latency (unchecked).

## Wanted

- Re-derive the split from one Sitting's bags that carry RobotState.bridge_stamp: Mocap against encoder FK (no Leg lag inside) and against command FK.
- Say what it means for the live path: the Ball Butler release constants and the bb_calibration lag advisory rest on the uncorrected Mocap stamp, and nothing here changes them without a decision.
