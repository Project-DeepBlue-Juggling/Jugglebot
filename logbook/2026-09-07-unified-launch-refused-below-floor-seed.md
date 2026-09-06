---
title: "Four unified goals refused without a ball leaving the cup — all four were one number, where the hand was resting; and the two that named a displaced catch were refused for a reach the unified plan never performs"
type: investigation
date: 2026-09-07
status: in-progress
phase: "unified-7dof-planner — Phase 5 (UH-6, second attempt)"
related_plan: unified-7dof-planner.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - tests/ros/test_unified_launch_floor.py
  - tests/hardware/session_unified7_cycle_ladder.md
  - plans/active/unified-7dof-planner.md
  - plans/active/INDEX.md
  - logbook/2026-09-07-unified-launch-refused-below-floor-seed.md
  - logbook/INDEX.md
subsystem:
  - ros
  - motion
tags:
  - safety
  - testing
---

# The launch was refused for where the hand was parked, and the aim was never going to be flown

## Summary

The 2026-09-06 23:55 sitting sent four `TossContinuous` goals with `unified_cycle: true` and got four
refusals with no ball ever leaving the cup. Goal #1 was **byte-identical to a goal accepted twelve
hours earlier**. The four refusals are not four findings:

- **Three of them are one number.** The hand was resting at **−0.1087 rev** (cup 676.16 mm), **13.44 mm
  below the planner's usable cup floor** of 689.60 mm. A LAUNCH seeded there must slam the cup into the
  z-box inside one 25 ms knot, and the closed form `|a| = 6·(floor_rev − seed)/dt²` reproduces
  `HAND_LIMIT_ACC` **4077.2 / 4076.9 / 4063.2 rev/s²** exactly — inverting each one recovers the seed to
  four decimals. Throw height, window length, aim and the levelling correction all measured **0.0 effect**.
- **The fourth is a legacy fiction.** `REJECTED_DISPLACEMENT` charged a 100.2 mm goal against a 66.0 mm
  *platform-reach* bound at T 0.495 s — a bound on an A→B reach the unified plan never performs, because
  the plan owns the whole traverse.
- **And the aim was silently discarded on every displaced goal.** The unified path never aims
  (`throw_target_mm = throw_site_mm`), but the *legacy* displaced preamble keys on `tier`, not on the
  planner, so it ran unbranched: goal #4 commanded 2.384°, mocap held **+2.27°** at POSITIONING, the
  plan's own first knots tilted the machine straight back, and the ball would have gone vertically up
  with nothing in any channel saying the aim had been dropped.

⚠ **The half that did not refuse is the more dangerous half.** The 3500 rev/s² cap is crossed only at
seed **−0.0485 rev** (11.53 mm low). Every seed between there and the floor is **ACCEPTED** and flies a
~0.35 rev knot-1 step at ~14 rev/s with the launch cup arcing **past** its 860 mm release site. The park
the machine sat at at 12:05 the same day (−0.038 rev) was **0.33 mm on the passing side of that line**.

Fixed here, **NOT re-flown**: the session lifts the hand into the box before its first launch, refuses a
below-floor launch by name before any solve, and refuses a displaced unified goal at acceptance instead
of tilting the platform for an aim it will throw away.

## Symptoms

Bag `~/Desktop/rosbags/2026-09-06_23-55-35/`, launch log
`~/.ros/log/2026-09-06-23-55-35-*/launch.log`, records
`temp/logs/toss_records_20260906-235539-1494163.jsonl`, Teensy capture
`temp/logs/cycle_ladder_20260906_235525.log`.

| # | `throw_height_m` | `catch_position` | aim | Outcome |
|---|---|---|---|---|
| 1 | 0.5 | (0, 0, 170) — **byte-identical to an accepted goal 12 h earlier** | — | `REJECTED_CYCLE_INFEASIBLE(HAND_LIMIT_ACC: 4077.2 > 3500.0)` |
| 2 | 0.3 | (0, 0, 170) | — | `HAND_LIMIT_ACC: 4076.9` |
| 3 | 0.3 | (100, 0, 170) | — | `REJECTED_DISPLACEMENT` (\|B−A\| 100.2 mm vs a 66.0 mm bound at T 0.495 s) |
| 4 | 0.3 | (50, 0, 170) | 2.384° | `HAND_LIMIT_ACC: 4063.2` |

No throw was armed and no ball left the cup on any of the four.

## Diagnosis

### The floor, in closed form

`unified_cycle.SETTLE_CUP_Z_MM` = **689.6 mm** is the planner's usable cup floor: the cup box's bottom
(679.6 mm) plus a 10 mm inset. `cup_cycle._assemble` writes the z-box rows for knots **1..n** — knot 0 is
the SEED and is exempt — and `_seed_relaxed_z_box`'s carve-out (added 2026-09-06, documented at 3400 rev/s²
exposure against the 3500 cap) applies to **REST-terminal** windows only. A LAUNCH is release-terminal, so
it takes the CONFIGURED box. The QP is therefore *required* to bring the cup inside the floor inside one
knot, and the hand acceleration that takes is

```
|a| = 6·(floor_rev − seed) / dt²      floor_rev = 0.3161715 rev,  dt = 0.025 s
```

Probe `/tmp/probe_floor.py` + `/tmp/probe_lift.py` (2026-09-07, session limits 250/3000/150000, banking on):

| seed (rev) | cup (mm) | vs floor | knot-1 jump (rev) | \|a\| (rev/s²) | LAUNCH verdict |
|---|---|---|---|---|---|
| **−0.1087** (the sitting) | 676.162 | **−13.44 mm** | +0.4249 | **4078.8** | REFUSED `HAND_LIMIT_ACC` |
| −0.0485 (the cap crossing) | 678.066 | −11.53 mm | +0.3647 | **3500.8** | REFUSED, barely |
| **−0.038** (the 12:05 park) | 678.398 | −11.20 mm | +0.3542 | **3400.0** | **ACCEPTED**, cup peaks 875.7 mm |
| 0.0 (the homed zero) | 679.600 | −10.00 mm | +0.3162 | **3035.2** | inside the silent band |
| +0.4865 (yesterday's carries) | 694.987 | +5.39 mm | — | — | ACCEPTED, clean |

Inverting the three measured refusals recovers the seed: 4077.2 ⇒ −0.108537, 4076.9 ⇒ −0.108506,
4063.2 ⇒ −0.107078. The whole family is **one fact: where the hand was.** The 875.7 mm figure above is at
flight 0.5 s with no speed trim; the sitting's own goal parameters put the same overshoot at 886.13 mm.

**Why yesterday worked.** The bench carries had lifted the hand to **0.4865 rev** roughly 90 s before the
first goal, and every unified cycle's chained LANDING settles at exactly 0.3162 rev — so once a session is
running it re-seeds itself inside the box every cycle. The failure is only ever reachable at the *first*
launch of a session, off whatever park the machine happened to be left in.

**What is not the cause**, measured at 0.0 effect across the four goals: the aim, the E8 levelling
correction, the throw height, and the window length.

### The aim was never going to be flown

Three separate seams, all pointing the same way:

- `reload_coordinator_node.py:~6954` — the unified request pins `req.throw_target_mm = list(req.throw_site_mm)`.
  A vertical self-toss. There is no field on the wire an aim could travel in;
- `_build_toss_cycle` keys the displaced preamble on `tier`, and `_position_platform_for_toss` had **no
  unified branch**, so the platform physically tilted to the aim. `unified_cycle._throw_tilt_for` then
  returned level for the vertical target and the window tilted back. Mocap: **+2.27° held** against 2.384°
  commanded;
- `_toss_unified_throw_xy` returned `seq.catch_pose_stow_mm[:2]` = **B** while the platform stood at **A**,
  so the launch window silently absorbed the \|B−A\| traverse and the announcement described a throw from a
  place the machine was not.

The only geometric aim bound that exists is `tilt_geometry.MAX_TILT_DEG` = 12° (the 1° constants are the
ILC/calibration correction channel, not an aim ceiling); the sitting's aims were 20–40 % of it. So the
refusal was never about the aim being too large — it was that no aim can be served on this path at all.

## Discussion

### The gate that turns on a third of a millimetre, and the band on the other side of it

The tempting fix, on first reading the bag, is to raise the hand acceleration cap: 4077 against 3500 is
a 17 % overshoot on a machine whose E-STOP band is far above it. That is the fix that would have hurt
most, and the reason is the shape of the failure rather than its magnitude.

`|a| = 6·(floor_rev − seed)/dt²` is **linear in the seed and enormous in the coefficient**: 9600 rev/s²
per rev of deficit, or **304 rev/s² per millimetre** of cup height. The cap is crossed at a seed
11.53 mm below the floor. The park the machine had been sitting at twelve hours earlier was 11.20 mm
below it — **0.33 mm on the passing side**. So the cap is not a guard on this class at all; it is a
coin toss with a third-of-a-millimetre edge, and which side it lands on decides between a loud refusal
and a **silent** 0.35 rev knot-1 step with the launch cup arcing 16–26 mm past the release site it was
solved for. Raising the cap would have converted every one of those refusals into that silent throw.

That is what makes the rule **the floor and not the cap**. A guard keyed on the acceleration catches
only the goals that were already going to be refused; a guard keyed on the floor catches the ones that
were going to fly badly. The `HAND_BELOW_FLOOR` refusal quotes the acceleration anyway, because that
number is what makes the silent band legible to an operator — "11 mm low" reads as a rounding error
until it is followed by "3400 rev/s², which is under the cap and will be flown".

### Why the lift happens at PREPARE, not by resizing the window or floating the z box

Three candidates were on the table, and two of them were measured to nothing:

- **Lengthen the LAUNCH window** so the QP has more knots to climb through. Measured 0.0 effect: the
  jump is a knot-1 *boundary* condition, not a rate problem. Knot 1 must be in the box whatever the
  window's total duration, so a 1.2 s launch slams exactly as hard as a 0.6 s one.
- **Let the release-terminal window take the relaxed z box**, i.e. extend `_seed_relaxed_z_box`'s
  carve-out to every kind. This is the *right* long-term answer and it is now an owner item (see Open
  Questions), but it is a planner change on the release-terminal path — the path whose terminal knot is
  the throw — made without a bench, on the night before a sitting. The carve-out it would generalise was
  itself written three weeks ago against 3400 rev/s² of exposure; widening it blind is how a contract
  dies.
- **Lift the hand into the box before the session's first launch.** No planner change, no cap change, and
  the machine ends in a state the planner already documents as good: a 1.0 s `KIND_SETTLE` from
  −0.1087 rev lands exactly on `floor_rev` at a peak hand rate of **0.654 rev/s** and a peak hand
  acceleration of **2.55 rev/s²** — three orders below the cap the un-lifted launch broke — for a 115 ms
  solve.

The placement is the other half of the choice, and it is a scheduling argument. The LAUNCH trigger fires
`_UNIFIED_LAUNCH_LEAD_S` = **1.80 s** before the FSM's scheduled release, and all of it is spoken for by
the solve (`_UNIFIED_PLAN_BUDGET_S` 1.20 s plus the 0.6 s window). A 1.0 s move plus a service round trip
inside that lead pushes the release past `TOSS_RELEASE_GRACE_S` and aborts the cycle with the ball in the
air — trading a refused throw for an aborted one. So the lift is paid where `_unified_warm_planner`'s
cold-solve cost is already paid: at session start, before any cycle exists, with nothing armed and nothing
airborne, **and the release schedule is untouched**. It is repeated once per cycle before the cycle is
*built* — a no-op read on a healthy chain, live only for the cycles a MISS, a retry or a reload interlude
left somewhere the session did not choose.

**The lift verifies on the hand, not on the plan.** Sitting 1 (2026-09-04) cost a bench evening to the
converse: a stage that ran to its terminal hold over an IDLE axis reads as a pass on every channel except
the encoder. So a settle that installs cleanly and leaves the hand where it was is a **failed** lift here,
and it says so.

### The tolerance is not a softening, it is what stops the belt sitting on a knife edge

A floor rule with an exact `<` would have been wrong in the other direction, and the reason only appears
once the lift exists. **The settle site IS the floor** — the lift's, and every chained LANDING's — so a
hand that has just done exactly what it was told sits at 0.3162 rev with the measurement straddling the
line. An exact test refuses roughly every other cycle of a perfectly healthy session, in the name of a
hazard that is not there, and it does it *after* cycle 1 has thrown.

`_UNIFIED_FLOOR_TOL_REV` = 0.01 rev is derived from both ends and they agree: it is **7.7× the measured
hand hold error** (0.0013 rev = 0.041 mm flat over a 600 s hold, `session_unified7_hand_bringup.md` row 16,
2026-09-04), so a parked hand cannot cross it by noise; and it costs at most `6·0.01/dt²` = **96 rev/s²** of
knot-1 acceleration, **2.7 % of the 3500 cap**, against the 4078 (117 %) the sitting hit. It is **1/35th**
of the 0.354 rev step the silent band actually flies, so nothing in the class this closes can hide inside
it. Both numbers had to be true; a tolerance justified by only the noise argument would have been a
threshold with no budget behind it.

## Fix

**F1 — lift into the box before the first LAUNCH.** `_unified_floor_lift()` reads the seed from
`/hand_telemetry` (fresh, else UNKNOWN — never "at the park"), and if the cup is below
`SETTLE_CUP_Z_MM` installs ONE `MODE_NEW`/`KIND_SETTLE` window of `_UNIFIED_FLOOR_LIFT_S` = 1.0 s to
`(live commanded platform xy, 689.6 mm)` — a pure z move, because a lift that traversed would be a
platform motion nobody asked for taken with a ball in the cup. It waits for the window to reach its
terminal hold (`plan_time_remaining_s` on `trajectory/status`, floored by the plan's own
`t0_mono + duration_s`, bounded by `_UNIFIED_FLOOR_LIFT_WAIT_S` = 3.0 s), then **re-reads the hand** and
reports failure if it did not move. Called at session start (immediately after `_unified_warm_planner`)
and once per cycle before `_build_toss_cycle`. One INFO line names the lift and the acceleration the
un-lifted launch would have commanded. An UNACKED lift is **held** (`trajectory/hold`) before returning,
on the same reasoning `_call_plan_cycle`'s `dispatched` split records.

Both the lift and the belt compare against the floor with a **0.01 rev / 0.316 mm** tolerance
(`_UNIFIED_FLOOR_TOL_REV`) — see the Discussion for why an exact test would have been the worse bug.

**F2 — refuse a below-floor LAUNCH loudly, before any solve.** `_tick_unified_launch` re-reads the seed
before it builds the request and mints
`REJECTED_CYCLE_PLAN(HAND_BELOW_FLOOR: hand rests <cup> mm < the planner floor 689.6 mm [catch_position…];
N mm low at <seed> rev — the launch would open with a 6*Δ/dt^2 = X rev/s² hand step against the 3500 rev/s²
cap; lift: <verdict>)`. It fires on **every** seed below the floor, not only on the ones past the cap, and
an UNKNOWN seed is refused rather than guessed. The lift's own verdict rides inside the outcome so the
refusal and the reason the fix did not land are one string.

**F3 — aim honesty.** (a) A displaced `catch_position` on a unified goal (\|B−A\| > `_UNIFIED_COLOCATED_TOL_MM`
= 5 mm against the LIVE commanded platform xy) is refused at goal acceptance with
`REJECTED_UNIFIED_AIM_UNSUPPORTED(...)` — before the latch, before the warm-up, before the legacy preamble
tilts anything, and before the FSM's `REJECTED_DISPLACEMENT` can charge a reach bound that does not apply.
An UNKNOWN platform pose does **not** refuse here: `REJECTED_POSE_UNKNOWN` already owns that case one layer
down, and a second spelling of one fact is how a guard and its twin drift apart. (b)
`_position_platform_for_toss` gained a unified branch: the pre-position is always LEVEL, and a resolved aim
(the calibration map, ILC layer 3) is WARNed as discarded rather than commanded into the platform and
silently undone by the plan. (c) `_toss_unified_throw_xy` returns the **LIVE** commanded platform xy, so the
launch is planned and announced from where the machine actually is; with (a) in place B == A, but the
guarantee no longer depends on the upstream gate staying in place. (d) With (a) refusing first, a displaced
unified goal never reaches the FSM's displacement gate, so the operator reads why the goal cannot be flown
instead of a number about a move nobody was going to make.

## Verification

- Probe, run 2026-09-07: `python /tmp/probe_floor.py` — reproduces `SETTLE_CUP_Z_MM` 689.6,
  `floor_rev` 0.3161715, and inverts the three measured `HAND_LIMIT_ACC` refusals to seeds
  −0.108537 / −0.108506 / −0.107078 rev.
- Probe, run 2026-09-07: `OMP_NUM_THREADS=1 python /tmp/probe_lift.py` — the 1.0 s `KIND_SETTLE` from
  −0.1087 rev lands on 0.3161715 rev, peak 0.6536 rev/s and 2.546 rev/s², 115 ms solve; the same LAUNCH
  goals refuse at 4078.8 / 3500.8 rev/s² and **accept** at seed −0.038 with the cup peaking 875.67 mm
  against an 860 mm release site.
- `python -m pytest tests/ros/test_unified_launch_floor.py -q -p no:randomly` (run 2026-09-07):
  **15/15 pass in 1.50 s.**
- `python -m pytest tests/ros/test_unified_launch_floor.py tests/ros/test_toss_continuous_node.py
  tests/ros/test_toss_integration.py tests/ros/test_unified_cycle_levelling.py
  tests/ros/test_unified_cycle_bench.py tests/ros/test_toss_coordinator.py
  tests/ros/test_toss_sequencer.py tests/ros/test_levelling_frame.py tests/sim/test_plans_index.py
  tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py -q -p no:randomly`
  (run 2026-09-07): **797/797 pass in 107.80 s.**
- `/usr/bin/python3 -m py_compile reload_coordinator_node.py` (run 2026-09-07): clean — Python 3.8
  compatibility held.
- **Full gate** (2026-09-07, `./run_tests.sh --full`, parallel **6875 passed / 4 skipped / 2 xfailed in 559.09 s**, serial **4 passed in 27.20 s**, total 593 s, exit 0) — **GREEN**, on the tree carrying this entry's fixes, the 2026-09-06 unified-cycle fixes and the conftest OOM fix. Two earlier runs of this gate wedged at 27–44 min with an xdist worker OOM-killed (2.43 GB) — the cause was a `MagicMock` `rclpy.ok` in `tests/ros/conftest.py` recording millions of calls under `while rclpy.ok()` waits with a no-op `time.sleep`, latent since the mock existed; fixed in the same commit (see the 2026-09-07 entry).

⚠ **Known red, handed off rather than fixed here.** Seven tests in
`tests/ros/test_unified_cycle_integration.py` drive `_tick_unified_launch` directly on a node whose hand
sits at `_ready_node`'s 0.0 rev default — the homed zero, which is **10.00 mm below the planner floor**, i.e.
inside the silent band — with `_hand_telemetry_mono` stamped on the fake clock, so the seed reads UNKNOWN.
All seven now hit the new belt and assert no request was sent:
`test_the_launch_tick_waits_for_the_release_lead`, `test_the_unified_launch_carries_the_SESSION_SPEED_TRIM`,
`test_trim_zero_leaves_the_unified_request_BIT_IDENTICAL`, `test_a_REFUSED_trim_is_not_flown_by_the_unified_launch`,
`test_an_UNACKED_plan_call_HOLDS_the_machine_and_says_a_plan_may_be_running`,
`test_a_hold_that_did_not_land_is_REPORTED_not_swallowed`,
`test_a_chained_launch_is_what_the_coordinator_actually_asks_for`. The harness fix is two lines in that
file's own `_seq_state`, under `if unified:` — set `node._hand_pos_meas = uc.hand_rev_for_cup_z(uc.SETTLE_CUP_Z_MM)`
and `node._hand_telemetry_mono = time.perf_counter()`. It was not applied because that file was being edited
by a parallel session. **Separately and NOT caused by this change**, five `TrajectoryNode`-only tests in the
same file fail in isolation on this tree (`test_plan_cycle_extend_keeps_the_origin_and_the_head`,
`test_landing_updates_replan_the_shipped_cycle_and_are_bounded_at_two`,
`test_a_replan_of_the_shipped_cycle_keeps_it_rest_terminal`,
`test_a_landing_update_on_a_bare_launch_is_refused_without_a_solve`,
`test_the_velocity_term_passes_the_same_origin_re_installs`,
`test_dynamic_target_routes_to_the_cycle_replan_not_build_catch`) and the set that fails varies run to run —
they touch no coordinator code at all.

## Outcome

**NOT re-flown.** Everything above is offline: probes, the mocked-ROS suite and the bag. The next sitting
is the first evidence that the lift moves the real hand and that a lifted seed plans a launch on the
machine. What to watch for, in order:

1. one `KIND_SETTLE` at session start, ~1.0 s, hand ending at 0.3162 ± a few thousandths;
2. the LAUNCH that follows accepted at the ordinary ~200–500 ms solve;
3. no `HAND_BELOW_FLOOR` in any outcome — one means the lift did not land, and the outcome says why.

## Open Questions

1. **The per-knot escape floor, which would close the class rather than route around it.** The lift is
   choreography; the underlying asymmetry is that `_seed_relaxed_z_box` relaxes knot 1's z box for
   REST-terminal windows only, so a release-terminal LAUNCH from a low seed is required to slam. Its own
   docstring names the generalisation. **Owner item** — it is a planner change on the throw path and wants
   a bench, not a night before a sitting.
2. **The aim-authority re-derivation for aimed unified throws** (Phase 5). What an aimed unified cycle
   should mean — a tilted release inside `MAX_TILT_DEG` planned by the QP, versus a displaced target the
   ballistics solve for — is undecided, and `REJECTED_UNIFIED_AIM_UNSUPPORTED` is a placeholder for that
   decision, not a verdict on it.
3. **`REJECTED_DISPLACEMENT` under unified is still reachable in principle.** F3(a) refuses displaced
   unified goals earlier, so the fiction is no longer *charged*; the FSM gate itself is untouched and would
   still fire if a unified goal ever reached it with a displaced site. Deleting or unified-branching that
   bound in `toss_sequencer.py` is the tidy-up.
4. **`REJECT_WIRE_MAP` hygiene.** `tests/hardware/toss_trace_recorder.py` has no entry for
   `HAND_BELOW_FLOOR` or `UNIFIED_AIM_UNSUPPORTED`; nothing pins that map's completeness, so a trace read
   of the next sitting will show the codes without their operator hint.
5. Closed 2026-09-07: the xdist worker was OOM-killed (journalctl: 2026-09-06 23:13 `Killed process … anon-rss:2484468kB`; 2026-09-07 00:13 `anon-rss:860228kB`). Root cause: the process-lifetime `rclpy` stand-in in `tests/ros/conftest.py` used `MagicMock` for `ok/init/shutdown/spin_once`; twelve `while rclpy.ok():` waits in `reload_coordinator_node` under a no-op `time.sleep` against real deadlines recorded 2 `_Call` objects per call (2,000,005 per 1,000,000 calls), and `--dist loadfile` lands a file's whole hoard in one worker — `tests/ros/test_toss_coordinator.py` peaked at 1129 MB (170 MB after), identical on a clean HEAD worktree, so latent since the mock existed. Fix: plain functions for the four names + `tests/ros/test_ros_mock_hygiene.py` (7 tests; 6 fail against the old conftest) pinning that no conftest stand-in records its calls. Verified: `tests/ros/` under `-n 4 --dist loadfile` 2849 passed, worker peaks 205/231/166/180 MB vs 1206 MB before; the no-op-sleep spin (~9 s per tier-8b test) remains a follow-up.
