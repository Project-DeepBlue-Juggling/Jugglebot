---
id: BUG-136
kind: bug
title: Catching cone catch stamps could run 10 ms early per stranded receive frame
status: inbox
created: '2026-11-23'
updated: '2026-11-23T07:12:14Z'
owners: []
priority: low
effort: s
hardware: robot
radius: subsystem
diagnosis: hypothesis
area: firmware
blocked_by: []
links:
- logbook/2026-11-23-platform-imu-clock-early-rx-ring-leak.md
filed_by: claude
night_skip: frozen-path
night_skip_on: '2026-11-24'
night_skip_note: Catching cone firmware and a flash; the offline check is blurred by the 100 Hz drain (Findings 2026-11-24). Low priority.
---

The Catching cone firmware builds against the stock FlexCAN_T4 and stamps the 0x7DD sync at dispatch, the defect diagnosed on the Platform in BUG-132 (logbook entry 2026-11-23-platform-imu-clock-early-rx-ring-leak).

## Findings

- A leak is expected almost never: its bus carries only the 100 Hz beacon and the bridge counted 0 stranded frames in 4 h.
- One stranded frame would stamp every Catch 10 ms early until the cone reboots. That biases a measurement (catch timing), not a motion.
- The Wave camera adapter (ESP32) also syncs to 0x7DD and was not audited.

## Deciding measurement

- Bridge receive time minus the cone stamp on CATCH_EVENT should be 30 ms plus under 1 ms. Any excess is the delay, in 10 ms steps. The data is already logged.

## Proposed fix

- Same as Ball Butler: patched library, or stamp 0x7DD in a reception hook. Low priority.

## Findings (2026-11-24)

- The deciding measurement is less direct than the body says. `cone/catch_event` carries only the reconstructed cone time (`header.stamp` equals `catch_time`); the bridge's UDP arrival time is used to rebuild it but is not published.
- So the only arrival reference in a recording is the record time, which adds the 100 Hz drain (`_drain_cone_catch_events`, 0 to 10 ms) and DDS delivery on top of the 30 ms. That jitter is the size of the 10 ms step it looks for: a stranded frame would show as a shift of the whole distribution, readable over many Catches but not per Catch.
- A cleaner reading needs the arrival time published (an additive field or a log line in the bridge).
