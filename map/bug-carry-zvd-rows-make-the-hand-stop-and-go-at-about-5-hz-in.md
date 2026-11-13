---
id: BUG-90
kind: bug
title: carry_zvd rows make the Hand stop and go at about 5 Hz in flight
status: inbox
created: '2026-11-13'
updated: '2026-11-13T05:57:34Z'
owners: []
priority: medium
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- map/feature-pre-release-hold-rung-release-from-a-moving-platform-hold-0.md
filed_by: claude
---

With `carry_zvd` on, the pose planner gives the Hand a stop-and-go velocity of about 5 Hz during the flight phase and a two-stage dive after the Catch. The operator saw it at the Sitting of 2026-11-13 (step 21: Hand and Platform moving in a stepped manner). It is in the command, not the transport: the wire and the firmware interpolator behave the same in both arms.

## Findings

- Offline, `test_cup_zvd._solve` at the test fixture reproduces it: Hand velocity turning points in the flight window, flag off versus on.
- Nothing came near a limit (Hand and Leg peaks, tracking error, current and emit gap match the flag-off arm); the plan only watched the HAND peak gates, which the wiggle passes.
- Hypothesis: under a tilted Platform, cup z moves the Platform laterally through the lever term, so a Hand wiggle is a cheap way to cancel the mode in the ZVD rows.

## Turning points in the flight window (offline, 80 mm/s prominence)

| Dwell | flag off | flag on | Hand v range off to on (mm/s) |
|---|---|---|---|
| 0.35 | 0 | 5 | -82..193 to -1121..2016 |
| 0.40 | 0 | 4 | -88..194 to -376..751 |
| 0.50 | 0 | 2 | -112..230 to -91..247 |

## Wanted

- A Hand velocity bound or smoothness term in the pose QP, or a ZVD-row formulation that does not trade Hand motion for Platform lateral mode cancellation.
- Landed as a strict xfail: `test_the_zvd_rows_do_not_make_the_hand_stop_and_go_in_flight` (Dwell 0.40, asserts on <= off); the passing control is `test_the_flag_off_hand_velocity_is_monotone_through_the_flight`. Remove the xfail when fixed. The planner is unchanged.
- The flag-on Dwell 0.35 must not fly until this is fixed.
