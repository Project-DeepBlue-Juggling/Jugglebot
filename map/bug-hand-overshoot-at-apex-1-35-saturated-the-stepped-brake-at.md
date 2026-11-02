---
id: JB-147
kind: bug
title: Hand overshoot at Apex 1.35 saturated the stepped brake at 60 A and drove the Hand into the top stop (SPINOUT)
status: in_progress
created: '2026-11-02'
updated: '2026-11-02T18:15:07Z'
owners:
- jugglebot-3d
priority: high
effort: m
hardware: robot
owner: decision
radius: subsystem
diagnosis: hypothesis
area: hand
blocked_by: []
links: []
filed_by: jugglebot-3d
---

At the first Apex 1.35 Columns goal (2026-11-02 17:00 Sitting, throw 10), one launch overshot. The Hand reached 168 rev/s against a planned ~151, and the release step's brake saturated at the 60 A limit (-60.4 A). The Hand overran the planned top (10.16 rev) and the 10.501 clip to 10.83 rev and hit the top stop; the encoder slipped (SPINOUT_DETECTED). The Ball went to 1.65 m.

## Evidence

Across the ~190 launches from Apex 1.05 to 1.35 that Sitting, the Hand topped out at 10.16-10.23 rev with brake current -10 to -36 A; only this one exceeded them. The launch came straight off a Catch that dived to the stroke floor (0.03 rev). There was no stream gap, the Platform z was fixed at 170 mm, and the cause of the overshoot is not resolved by the 100 Hz `/hand_telemetry`.

## Why it matters

The stepped brake runs near the constant-deceleration ideal, so an overspeed has only the current headroom and about 0.34 rev of stroke above the planned top. The same margin shrinks as the Apex rises.

## Options

Raise `hand_curr_limit_a` to the owner's 70 A (configure pushes it over the GUI value); scale a top margin with release speed; capture the Hand at a higher rate to find the overshoot's cause; a firmware overspeed guard. Related: JB-144, JB-145, the JB-114 bumper.

## Sitting

- setup: armed, bb-fed
- why: re-fly the Apex 1.35 rung that overshot, now with the 70 A limit and the C-HAND-6 speed-scaled top margin (8ffeba66), and with `hand_capture` on (the launch default since 8b9f990c), so an overshoot is caught at full rate.
- do:
  - Fly Columns, Sep. 125, Apex 1.35, Throws 20, twice.
  - Stop at the first SPINOUT_DETECTED, MAX_DEVIATION or stream STOP.
- pass: no SPINOUT_DETECTED in either 20, and every launch's Hand top at or under the 10.501 rev clip.
- record: per launch the Hand top (rev), peak Hand speed and brake current; the capture CSV path of any launch whose top exceeds its planned top.
- minutes: 15

## Progress

- 2026-11-02: Landed 8ffeba66: 70 A code limit, stream STOP 4500, C-HAND-6 speed-scaled top margin (K 4.75e-5), plus the opt-in hand_capture:=true for diagnosing the overshoot. Cause of the overshoot still open; fly with the capture on.
