---
id: JB-147
kind: bug
title: Hand overshoot at Apex 1.35 saturated the stepped brake at 60 A and drove the Hand into the top stop (SPINOUT)
status: in_progress
created: '2026-11-02'
updated: '2026-11-03T01:50:06Z'
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
  - Fly Columns, Sep. 125, Apex 1.45, Throws 20.
  - Fly Columns, Sep. 125, Apex 1.50, Throws 20.
  - Skip Columns 1.25 and 1.40 until the JB-143 box lands (those cells have no landing room yet).
  - Stop at the first SPINOUT_DETECTED, MAX_DEVIATION, stream STOP, or Hand iq over 66 A.
- pass: no SPINOUT_DETECTED in any run, and every launch's Hand top at or under the 10.501 rev clip.
- record: per run the catches, the highest Hand top (rev), peak Hand speed, peak brake and launch current; the capture CSV of any launch whose top exceeds its planned top; whether the Hand torque feedforward (capture `cmd_tor_ff`) flat-lines at the 0.234 N m clamp at 1.45 or 1.50.
- minutes: 30

## Sitting result (2026-11-03)

- verdict: fail
- notes: At 1.35, the hand isn't using the full stroke. It looks like there's at least 3-4cm extra at the top of the stroke. One attempt ended with LANE_REFUSED. The last attempt ended with SPINOUT_DETECTED, despite no obvious cause for this to my eye (no visible collision with the end stop, no sudden sounds) At 1.45, the first attempt ended with a messy throw being uncatchable. Second ended with SPINOUT_DETECTED, for which it sounded like the hand hit the end-stop (or something?). Third attempt ended the same way, but with no audible collision between the hand and the end-stop At 1.50, the first attempt had a non-audible SPINOUT_DETECTED, while the second attempt ended with the same error but I could hear the collision between the hand and the end-stop. Third attempt ended the same as the second (audbily).

## Progress

- 2026-11-02: Landed 8ffeba66: 70 A code limit, stream STOP 4500, C-HAND-6 speed-scaled top margin (K 4.75e-5), plus the opt-in hand_capture:=true for diagnosing the overshoot. Cause of the overshoot still open; fly with the capture on.
