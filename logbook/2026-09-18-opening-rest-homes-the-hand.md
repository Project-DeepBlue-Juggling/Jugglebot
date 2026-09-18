---
title: "The opening REST homes the hand within the firmware's envelope — the schedule-side park that refused an off-band hand lasted one sitting"
type: investigation
date: 2026-09-18
status: done
phase: "two-ball-skill-stack — R3"
related_plan: two-ball-skill-stack.md
files_changed:
  - plans/active/two-ball-skill-stack.md (2026-09-18 item 2 rewritten; the carried "re-seed the streamed hand lane onto the park" item marked RESOLVED; R4 carried item (c) now also names the columns refusal)
  - ros_ws/docs/choreography.md (regenerated: `park_hand` clients → none, `link_status` → +skill_node)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md (new C-HAND-4: the homing envelope, one enforcement point, three tests; the `HAND_LANE_REFUSED` END row)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py (`HAND_LANE_REFUSED`, `Observations.hand_lane_refused` (default False), checked every tick beside `in_trajectory_mode`; `_force_hold` installs a hand-less hold on the END path, consumed on read)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py (`floor_lift_s(hand_rev_now)`; `_SHAPE_KV = 1.875`, `_SHAPE_KA = 6.0`; `HOME_HAND_VEL_LIMIT_RPS` / `HOME_HAND_ACC_LIMIT_RPS2`; `SelfTossPattern.floor_lift_s`, `compile_self_toss` opening-REST period)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/sites.py (`REST_HAND_REV = uc.hand_rev_for_cup_z(REST_CUP_Z_MM)`, the one named home)
  - ros_ws/src/jugglebot/jugglebot/skill_node.py (`_park_hand` and its `/park_hand` client deleted; `_opening_rest_period()` reads `_hand_pos_meas`, refuses only on stale/absent telemetry; `sched_refused` read directly off `/link_status`, baseline latched at compile; `_hand_home_error()` for columns)
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py (`/park_hand` contract block reworded to the operator/recovery statement; op itself unchanged)
  - tests/hardware/session_skills_r3.md (§ 7 rewritten: sized REST + log line, `/park_hand` as the operator op, what END HAND_LANE_REFUSED means)
  - tests/motion/test_skills_executor.py (refused-lane-ends-attempt-and-holds test; default-unchanged test)
  - tests/motion/test_skills_schedule.py (floor at home, velocity branch, acceleration branch, symmetry about home, schedule shift, below-floor refusal)
  - tests/motion/test_skills_segments.py (contract test: samples the planned hand lane at `floor_lift_s`'s own period for three seeds, asserts peak |v|/|a| and arrival at home)
  - tests/ros/test_skill_node.py (park tests deleted; ten new: sized REST at 9.6227 rev, default at home, ACTIVATE park not a special case, stale-hand refusal, the INFO line, columns refuses a displaced hand, columns starts at home, a `sched_refused` bump ends+holds once, a high baseline is history, an absent key changes nothing)
subsystem:
  - motion
  - ros
tags:
  - safety
  - performance
  - testing
---

## Symptom

Every attempt after the first refused, on an armed wire, with the hand exactly where the previous
schedule's own REST had left it:

> self-toss refused: the hand park was refused: hand park REFUSED — the hand is at +0.3063 rev
> (outside 0.1 rev of the park) but the wire is ARMED ... DEACTIVATE then ACTIVATE

Owner's ruling, verbatim: *"If the hand isn't where it needs to be at the start, it should
smooth-move down to the start position before beginning the cycle."*

## Diagnosis

Two "homes" 0.307 rev apart, checked against a 0.10 rev band: the schedule's own REST leaves the
hand at `uc.hand_rev_for_cup_z(uc.SETTLE_CUP_Z_MM)` = 0.3071 rev, while the bridge's `/park_hand`
(landed the same day, `2026-09-18-tracker-aim-carried-the-lateral-bias-park-race-splice-budget.md`
item 4 / Discussion b) measures against `HAND_ACTIVATE_POSITION_REV` = 0.0. A hand parked by REST 1
is 0.307 rev outside the 0.10 rev band by construction, so `/park_hand` refused on every second
attempt — the schedule kept landing the hand at the site its own rest wants, and the park kept
scoring that as displaced.

The same day's 13:25 sitting (see that entry's Diagnosis #2) independently confirms the resume
envelope this fix has to respect: latch 1 was the opening REST moving the hand > 0.05 rev inside
the firmware's ~60-100 ms handover window, so the resumed lane was refused and the hold latched — a
lane that starts slow enough is accepted and followed. The two failures are the same shape: a hand
move that is either the wrong size (too far from the axis's real home) or too fast at the handover
(more than the firmware's resume tolerance allows) gets refused; a move sized to the actual home and
bounded to the firmware's own envelope does not.

## Discussion

**Why home in the REST rather than park.** `/park_hand` moves the axis directly, and after any
hand-less hold the firmware's lane HOLDS its last knot and keeps commanding it — parking the axis
out from under a still-streamed lane reopens the exact MAX_DEVIATION gap the 13:25 fix (`lane_cleared`)
exists to prevent, just with the encoder moving instead of the plan. The opening REST is different in
kind: it is the schedule's own lane, so "homing" is nothing but shaping that lane's first segment from
the encoder-reconciled seed to the rest rev — a continuous resume, not a park racing a live command.
That is why unit 4's fix (`/park_hand(*, lane_cleared=…)` refusing an off-band hand on an armed wire)
was the right answer to the wrong question: it correctly stopped a park from moving the axis under a
live lane, but a schedule was never supposed to park at all — it should shape its own opening move.
The design that survives is: `/park_hand` stays exactly what the 13:25 entry built it as (the
operator/recovery op, armed-wire refusal intact, because that refusal is still correct FOR AN
OUT-OF-BAND PARK); a schedule never calls it, and homes itself instead.

**The envelope numbers.** The firmware accepts a resumed lane only if, at the handover instant, the
new frame evaluates within `SCHED_RESUME_TOL_POS_HAND_REV` = 0.05 rev of the held state
(`leg_interp.cpp:497-520`, `canbridge_config.h:426-428`); the first frame reaches the wire ~60-100 ms
after the plan's origin, so `½·a·(0.1 s)² ≤ 0.05 rev ⇒ a ≤ ~5 rev/s²` in that window — the firmware's
own `RECOVER_SLEW_ACCEL_RPS2`. `HOME_HAND_VEL_LIMIT_RPS` reuses the profiled park's own rate
(`JB_OP_GENTLE_MOVE_VEL_LIMIT_RPS`, 2.5 rev/s) rather than inventing a second number for the same
regime.

**Why `_SHAPE_KA = 6.0`, not the quintic 5.774.** The REST's rest-terminal settle is shaped by
`unified_cycle.plan_settle`'s own QP, not a textbook quintic. Probing `plan_segment(REST, …)` at the
R3 limits across six seeds gave realised shape factors k_v 1.50-1.53, k_a 5.68-5.95, flat in Δ and T.
The quintic bound (15/8 = 1.875 for k_v) sits 23% conservative and is kept; the quintic bound for
k_a (10/√3 = 5.774) sits BELOW the QP's measured max and would size the acceleration branch roughly
3% hot — so `_SHAPE_KA` is the measured max rounded up, not the textbook constant. Measuring first
and letting the fixed constant follow the measurement (rather than the other way around) is the same
discipline the codebase's empirical-probe convention asks for.

**Why `sched_refused` is read directly off `/link_status` in `skill_node`, not through a new
`/trajectory/status` field.** Routing it through `trajectory_node` needs a new field on a typed ROS2
interface, i.e. an interfaces rebuild before the branch is safe to fly, and adds a 5 Hz cache between
the publisher and the only consumer that acts on the value. Reading `/link_status` directly is one
hop at 10 Hz with no interface change — the deviation from the original design brief (which named the
`trajectory_node` route) is root-caused, not a shortcut: an absent key reads UNOBSERVED (-1, i.e. no
refusal) so an unreadable diagnostic can never end an attempt on its own, and the baseline is latched
at compile so a *previous* attempt's refusal can't retroactively end a new one.

**Why columns refuses at R4 rather than homing.** `compile_self_toss`'s skill 0 is the REST itself, so
there is a window whose lane the sizing can shape. `compile_columns`'s skill 0 is a CATCH that splices
onto an already-live plan — there is no window to home into and nothing for the sizing to act on.
Deleting the schedule-side park unconditionally (as this fix does for self-toss) would have left
columns streaming a catch from a seed up to 9 rev away with no guard at all, which is worse than the
refusal it replaces. `_hand_home_error()` therefore refuses columns pre-motion, outside a name band,
pointing at the self-toss path that does home — deferred to R4 rather than solved here, because
columns' own homing needs a design for shaping a catch-splice window, not a REST.

## Fix

1. **`sites.py`** — `REST_HAND_REV = uc.hand_rev_for_cup_z(REST_CUP_Z_MM)` = 0.30706 rev, the ONE
   named home; the bridge's ACTIVATE park (0.0 rev) is untouched and stays what `/recover` parks to.
2. **`schedule.py`** — `floor_lift_s(hand_rev_now)` = `max(FLOOR_LIFT_S, K_v·Δ/2.5, sqrt(K_a·Δ/5))`,
   pure, no clock; `HOME_HAND_VEL_LIMIT_RPS` = 2.5, `HOME_HAND_ACC_LIMIT_RPS2` = 5.0;
   `_SHAPE_KV = 1.875`, `_SHAPE_KA = 6.0` (measured, see Discussion). `SelfTossPattern.floor_lift_s`
   carries the sized period (default `FLOOR_LIFT_S`, refuses below it); `compile_self_toss` sizes
   skill 0 from it and every later dispatch instant follows.
3. **`skill_node.py`** — `_park_hand` and its `/park_hand` client deleted (both call sites);
   `_opening_rest_period()` reads `_hand_pos_meas` where the park used to run, refuses on ONE fact
   only (`hand_telemetry is stale or absent`), and logs
   `opening REST homes the hand: +9.6227 → +0.3071 rev over 6.99 s (peak <= 2.50 rev/s, 1.15 rev/s²)`.
   `sched_refused` is read off `/link_status` every tick with a compile-time baseline (Discussion).
   `_hand_home_error()` refuses columns on a displaced hand, filed against R4.
4. **`executor.py`** — `HAND_LANE_REFUSED` + `Observations.hand_lane_refused` (default False, so the
   sim gate and every pre-existing observer are bit-identical); checked every tick beside
   `in_trajectory_mode`, not at dispatch, because the refusal can happen mid-window. The END path
   installs a hand-less hold via `_force_hold` (consumed on read, once per attempt).
5. **`teensy_bridge_node.py`** — `/park_hand`'s contract block reworded: a streamed lane may home the
   hand only within the firmware's own resume and follow envelope; outside a disarm/arm edge, nothing
   else may move the hand. The op and its armed-wire refusal are otherwise untouched — they are still
   correct for the operator/recovery case.
6. **Docs** — `INVARIANTS.md` C-HAND-4 (envelope, enforcement point, tests) + the `HAND_LANE_REFUSED`
   END row; `choreography.md` regenerated; the plan's 2026-09-18 item 2 rewritten, the carried
   "re-seed the streamed lane" item marked RESOLVED (the lane already resumes from its held knot —
   that is what the sizing is for); `session_skills_r3.md` § 7 rewritten.
7. **Tests** — five `test_skill_node.py` park tests deleted, ten added; six added to
   `test_skills_schedule.py`; one contract test (×3 seeds) added to `test_skills_segments.py`; two
   added to `test_skills_executor.py`.

## Verification

| what | date, command | result |
|---|---|---|
| skill_node (park deleted, homing + refusal) | 2026-09-18, `pytest tests/ros/test_skill_node.py -q -p no:cacheprovider -p no:randomly` | **73/73 pass** (was 68; −5 park tests, +10) |
| schedule (floor_lift_s shape) | 2026-09-18, `pytest tests/motion/test_skills_schedule.py -q -p no:cacheprovider -p no:randomly` | **62/62 pass** (+6) |
| segments (planned hand-lane contract) | 2026-09-18, `pytest tests/motion/test_skills_segments.py -q -p no:cacheprovider -p no:randomly` | **18/18 pass** (+1 parametrised ×3) |
| executor (HAND_LANE_REFUSED end+hold) | 2026-09-18, `pytest tests/motion/test_skills_executor.py -q -p no:cacheprovider -p no:randomly` | **113/113 pass** (+2) |
| broad skills sweep | 2026-09-18, `pytest tests/motion/test_skills_executor.py tests/ros/test_trajectory_node.py tests/ros/test_teensy_bridge_node_recover.py tests/ros/test_install_segment.py tests/ros/test_skills_plan_bench.py tests/motion/test_skills_sites.py -q -p no:cacheprovider -p no:randomly` | **384/384 pass** |
| sim gate + sites | 2026-09-18, `pytest tests/sim/test_skills_gate.py tests/motion/test_skills_sites.py -q -p no:cacheprovider -p no:randomly` | **20 passed, 1 xfailed** (gate's pre-existing xfail) |
| full ros+motion sweep | 2026-09-18, `pytest tests/ros tests/motion/test_skills_schedule.py tests/motion/test_skills_executor.py tests/motion/test_skills_segments.py tests/motion/test_skills_sites.py -q -p no:cacheprovider -p no:randomly` | **3144 passed, 4 skipped, 2 failed in 213 s** — the 2 failures are `test_choreography_map.py`'s doc-drift pair, an artifact of regenerating `choreography.md` while that run collected; re-run after: `pytest tests/ros/test_choreography_map.py -q -p no:cacheprovider -p no:randomly` (2026-09-18): **30/30 pass**; `test_skill_node.py` re-run scoped: **73/73 pass** |

(2026-09-18, `./run_tests.sh --full`, log `temp/logs/hand_home_full_20260918.log`, the committed tree): **PASS — parallel 6305 passed, 9 skipped, 2 xfailed in 288.26 s; serial 6 passed in 19.05 s.**

NOT run, per brief for this unit: `./run_tests.sh`, the nightly ticker.

## Carried

- **Columns' own opening homing** (R4) — `compile_columns` refuses a displaced hand outright rather
  than shaping a homing move onto its catch-splice skill 0; needs a design for shaping that window,
  not a REST, before columns can start from a displaced hand instead of refusing.
- **The banking contract as the route to lateral authority.** The owner asked why the learner cannot
  simply tilt the platform to remove the tracker's +y lateral bias (the 13:25 entry's Diagnosis #1
  and its clamp). It can — `tilt_to_receive` is the channel — but that channel is pinned to zero
  lateral authority by the 2026-09-16 banking-saturation finding
  (`2026-09-16-banking-saturates-on-small-lateral-offsets.md`): any nonzero lateral offset during the
  dive clamps `tilt_to_receive` to its 12° limit regardless of the offset's size, so handing the
  learner that channel today would not remove the bias, it would saturate on it. The banking contract
  (defined only with seating force, carried from that entry) is what unpins the channel; a "fixed
  trim" shortcut is retired rather than pursued because it rides the exact same
  `tilt_to_receive`/dive planner path and would saturate the same way.
