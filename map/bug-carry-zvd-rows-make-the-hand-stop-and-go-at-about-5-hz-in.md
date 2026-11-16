---
id: BUG-90
kind: bug
title: carry_zvd rows make the Hand stop and go at about 5 Hz in flight
status: in_progress
created: '2026-11-13'
updated: '2026-11-15T19:02:54Z'
owners:
- night-shift
priority: medium
effort: m
hardware: robot
owner: decision
radius: subsystem
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

## Options (2026-11-14)

1. **A Hand smoothness term with the ZVD rows on:** penalise Hand jerk across the flight window only when `carry_zvd` is set; the strict xfail judges it offline. Planner math, so an Opus unit; held for a Sitting.
2. **Take the Hand out of the ZVD lever:** the rows see the Platform's own lateral motion, not cup z through the tilt lever. A change to the I-PLAN-16 formulation, for a day Session.
3. **A hard bound:** no Hand velocity turning point in flight. Risks infeasible solves at short Dwell.
4. **Leave `carry_zvd` off** (the default) and park the ZVD arm of FEAT-32 until a Session takes it up.

Recommendation: 1. It is the smallest change the existing xfail can judge, and it leaves the flag-off plan byte for byte as now. Risk: a heavy weight could weaken the mode cancellation the rows exist for, so the Sitting compares the Platform residual too.

## Sitting

- setup: level, armed, bb-fed
- why: The branch adds a Hand acceleration cost (weight 100, flight window less 0.15 s at each end) read only with carry_zvd on, which offline removes the ~5 Hz Hand stop-and-go the flag produced in flight at the 2026-11-13 Sitting (step 21), but it has never flown.
- do:
  - Launch on the branch with `gc_freeze:=true` and the bag recording. After Homing, `ros2 param get /skill_node carry_zvd` reads false. Level, Activate, Ball Butler loaded, Cup empty.
  - Set a fresh plant: `ros2 param set /skill_node plant_id zvd-oval-<date>-C`, read it back with `ros2 param get /skill_node plant_id`.
  - `ros2 param set /skill_node carry_zvd true` (refused while a goal runs), read it back; the goal start line must show carry_zvd.
  - Dwell 0.40: `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: oval_1ball_fed, apex_m: 1.00, separation_mm: 150.0, dwell_s: 0.40, num_cycles: 32}" --feedback`. Empty the Cup after every ended Attempt; after clear_errors re-enter TRAJECTORY before the next goal.
  - Dwell 0.35 (`dwell_s: 0.35`, other keys the same) ONLY if 0.40 showed no ~5 Hz Hand stop-and-go.
  - Stand down: `ros2 param set /skill_node carry_zvd false` and `ros2 param set /skill_node plant_id jugglebot`, each read back.
- pass: with the flag on at Dwell 0.40, the Hand moves smoothly through each flight between the post-release brake and the Capture move (no ~5 Hz stop-and-go seen), no Leg LIMIT or RING refusal end, and Catches no fewer than the 2026-11-13 flag-on arm.
- record: per Dwell: Throws caught per Attempt and each end reason, whether the Hand stop-and-go is gone, install-line Leg peaks (offline about 275 mm/s and 3270 mm/s² at 0.40, 318 and 5130 at 0.35), any RING refusal text, the bag and launch log paths.
- minutes: 30
- branch: night-shift/2026-11-15-BUG-90

## Progress

- 2026-11-15: Night Shift 2026-11-15: TKT-69 option 1 built on night-shift/2026-11-15-BUG-90 but unfinished (unit cut off by API credit); test_cup_zvd passes, wider checks, suite and audit owed
- 2026-11-16: Night Shift 2026-11-16: held on night-shift/2026-11-15-BUG-90: checks finished and audited clean, Sitting section written; unblocked by a Sitting
