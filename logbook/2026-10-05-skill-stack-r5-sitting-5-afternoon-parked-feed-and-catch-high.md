---
title: "R5 sitting 5 (2026-10-05 afternoon): Ball 2 never caught because the cup arrived at the feed site with the ball, still sliding — the fed columns now start with a hop that leaves the cup parked under the feed, and every catch is taken high (930 mm)"
type: investigation
date: 2026-10-05
status: resolved
phase: "two-ball-skill-stack — R5"
related_plan: two-ball-skill-stack.md
related_entries:
  - 2026-10-05-skill-stack-r5-sitting-5-launch-limits-and-jam-anchor.md
  - 2026-10-04-skill-stack-r5-sitting-4.md
  - 2026-09-30-skill-stack-r5-sitting-1.md
files_changed:
  - config/generated/admissible_box.yaml
  - plans/active/two-ball-skill-stack.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/report.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/sites.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - sim/skills_gate.py
  - tests/hardware/session_skills_r5_sitting6.md
  - tests/motion/test_feed_lateral_miss_probe.py
  - tests/motion/test_migrate_memory_catch_plane.py
  - tests/motion/test_skills_admissible.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_report.py
  - tests/motion/test_skills_schedule.py
  - tests/motion/test_skills_sites.py
  - tests/ros/test_skills_plan_bench.py
  - tests/ros/test_skill_node.py
  - tests/sim/test_skills_gate.py
  - tools/migrate_memory_catch_plane.py
  - tools/probes/feed_lateral_miss.py
  - tools/probes/throw_outcome_bag_probe.py
---

# R5 sitting 5, afternoon: the parked feed catch and the high catch

## Summary

The owner flew sitting 5 after the morning fixes (bag `2026-10-05_16-42-57`, launch log
`~/.ros/log/2026-10-05-16-42-57-819222-jetson-137539/launch.log`). The hand-jam recovery passed
twice on the bench (detected fast, raised to 5.0 rev, the ball dropped through, lowered) — the
owner calls that objective complete. Ball 1's half (`columns_1ball`, 3 goals of 3/6/10 throws)
caught every real throw (1 + 3 + 5; the one MISSED is the Stop's deliberate landing on the held
ball), and its observed apexes read 0.954-0.971 m against the 0.95 command — the +14.5 mm a perfect
plant reads with the catch plane 30 mm below the release (see Discussion). Ball 2's half (`columns_1ball_fed`) caught nothing
cleanly in 21 runs, and the owner asked whether the platform could tilt toward Ball Butler's throw.

It could, but the data says the angle is not what differs. Of 21 fed runs, 13 never attempted the
catch (the planner refused the transit into it), 2 ended on Ball Butler's side, and 6 attempted.
The same sitting's three `columns_1ball` runs open with Ball Butler feeding Ball 1 into a cup that
is already PARKED: 3 of 3 caught, at the same 11-12° arrival. In the fed pattern the cup crosses
~130 mm in the last 0.2 s, arrives 15-20 mm past the catch point still sliding against the ball,
and meets it 31-64 mm off centre. Two owner decisions (2026-10-05 evening) follow: a **hop entry**
(Jugglebot starts at the feed site holding Ball 1, throws it across to Ball 1's site, and stays
parked for Ball 2) and **catch high** on every catch (catch plane 930 mm, was 830; the owner's ask
that the hand wait near the top and move the main part of its stroke only with a ball in it).

Also found: `tools/probes/feed_lateral_miss.py` (added in the sitting-4 fixes) matched pattern names
by substring, so it skipped every `columns_1ball_fed` goal — runsheet row 19 could never return a
bias, and `columns_feed_bb_bias_mm` stayed `[0, 0]` all sitting. Fixed and tested.

## What the owner reported

"The hand jam detection and recovery is working! I tried this twice … I consider this objective
complete. Ball 1's half of the columns pattern is working very nicely … Ball 2's half … is no
better than it was in the last sitting; the ball was not caught cleanly once, across the ~10-15
attempts that I ran. Is there any way we can have the platform tilt towards the ball being thrown
by BB to help its catch?" Then, on the proposal: "throw Ball1 from Ball2's site towards Ball1's
site, then re-orient to catch Ball2 … Elegant. I suggest we also keep the hand at the top of its
stroke between throws and catches; the hand should only move the main part of its stroke while
holding a ball." Asked how to scope the stroke change, the owner chose "catch high, all catches".

## Measured

**How the 21 `columns_1ball_fed` runs ended** (launch log, one goal each):

| End | Count | Detail |
|---|---|---|
| `LIMIT_ACC` at skill 1 (the feed CATCH splice) | 11 | peak leg acceleration 5010-5306 mm/s² vs 5000 — the 115 mm transit from Ball 1's site into the feed catch; the hand stayed at P2 |
| `REJECTED_COLUMNS_FEED_UNCATCHABLE` at accept | 2 | 5026.7-5026.8 mm/s², the same transit |
| `REJECTED_BB(THROW_ABORTED_NOT_SETTLED)` / BB throw service timeout | 1 / 1 | Ball Butler side |
| attempted: `MISSED_CATCH` / seated late then thrown wide / no release | 3 / 2 / 1 | no clean catch |

The one full `columns` run also ended `LIMIT_ACC` at skill 1.

**Parked cup vs arriving cup** (free-fall fits of the raw `/mocap_data` track, `Platform` body at
the arrival; probe `feed_lateral_miss.py`'s method; scratchpad `s5b/feeds_1ball.py`):

| | `columns_1ball` feeds (cup parked at P2) | `columns_1ball_fed` feeds (cup arriving at P1) |
|---|---|---|
| Caught | 3 of 3 (seated −0.03..+0.26 s) | 0 of 6 cleanly (2 seated at +0.24 s, after the hand's reversal) |
| Arrival angle off vertical | 11.5-11.8° | 10.7-13.2° |
| Ball-to-cup distance at the arrival | 23-30 mm | 31-64 mm |
| Cup travel in the last 0.2 s | ≤ 1.2 mm | 132-138 mm |
| Cup velocity at the arrival | ~0.02 m/s | 0.2-0.55 m/s, against the ball's +x |
| Relative horizontal speed | 1.15 m/s | 1.25-1.48 m/s |

The cup overshoot at the feed site is repeatable (body x −67.6 … −72.9 mm against the −52.5 catch
point, ±2.5 mm) and the platform swings ~±10 mm in y through the arrival — the same transit Ball 1's
catches fly, where the ball drops vertically and the catch follows the tracker.

**Ball Butler's feeds** (16 tracked, incl. the 11 where the cup stayed at P2): arrive on time
(free-fall crossing within ±0.03 s of the committed landing; the `TRACKER-IDENTITY-REFUSED …
−0.15 s` lines are unconverged fits, the same withdrawn reading as sitting 1), at
v ≈ (+1.0, +0.45, −5.7) m/s at the 830 plane, and land median (+15, +21) mm from the point they
were asked to hit, x drifting ~30 mm across the 7 minutes (early +8..+67, late −10..+18). The fixed
probe, against the cup at the arrival (n 5): median (+34.7, +29.2) mm, 0/5 seated before 0.19 s —
that number includes the cup's overshoot, so it does not carry over to a parked cup.

**The hand stroke, as flown** (`/hand_telemetry`, goal 17, one beat): the empty hand reaches the
top (~9.7 rev) after each release, holds ~0.1 s, then drops ~146-169 mm EMPTY and meets the ball at
~5 rev moving down ~1.6 m/s, carries it to the bottom (0.27 rev), throws, and climbs to the top
again after the release. Phantom strokes are full empty strokes.

**Catch-height probe** (real chain LAUNCH + STEADY×3, apex 0.95 m, separation 125 mm, dwell 0.27 s,
350/5000/200000, hand 3900; scratchpad `s6/catch_height_probe.py`, 2026-10-05):

| Catch z | Empty drop before contact | Cup at contact | Stroke with the ball | Leg acc / jerk | Hand acc |
|---|---|---|---|---|---|
| 830 | 169 mm | −1.61 m/s | 137 mm | 4133 / 159 920 | 3528 |
| 890 | 108 mm | −1.22 m/s | 197 mm | 4115 / 158 918 | 3533 |
| **930** | **68 mm** | **−0.96 m/s** | **237 mm** | 4073 / 157 709 | 3509 |
| 950 | 48 mm | −0.80 m/s | 251 mm (bottom lifts to 0.49 rev) | 4074 / 154 400 | 3513 |
| 970 | 28 mm | −0.61 m/s | 248 mm (bottom 1.18 rev) | 4069 / 154 064 | 3704 |

Release height at catch 930: 880 → `HAND_LIMIT_ACC` 4416 rev/s² > 3900; 900 → 4907; 920 → QP
infeasible. The climb after the release is the hand braking from 4.3 m/s; it cannot be shortened.

**Ball-ball clearance of the hop entry** (scratchpad `s6/clearance.py`: Ball 1 from P1 at 860 mm
to P2 at 930 mm over the columns flight; Ball 2 into P1 on each of the 16 measured feeds):
closest approach ~0.19 s after Ball 1's release, both at ~1.5 m. Feeds on target: 141-157 mm
between centres (gap 67-83 mm; balls are 74 mm). Bias corrected by the median, each feed keeping its
own scatter: median gap 84 mm, worst 29 mm. Uncorrected (as flown): the +67 mm feed touches (gap 0).

## Discussion

**Why not tilt toward Ball Butler.** Banking the receive attitude toward the arrival is the
planner's default for every catch (`tilt_geometry.tilt_to_receive`); the feed catch is the one
place it was pinned level, on 2026-10-02, because sitting 1's held-level block caught 13 of 14 feeds
at 11.9° and banking on and off inside the transit + dwell ran the leg jerk channel at ~1.7× its
cap. Today's control case repeats the first half: the arrival angle is the same in the caught and
the missed groups, so tilting would act on the term that does not differ. It is also expensive on
this machine: the cup sits ~0.66 m above the Platform body, so an 11° bank swings it ~130 mm
sideways, which the legs must cancel by translation inside a window that was already over its
acceleration cap in 13 of 21 runs. Kept as the fallback if parked catches bounce off the far wall.

**Why the hop entry, and what it costs.** One cup means Ball 1 must be in the air when Ball 2
arrives, so the feed catch always sits inside Ball 1's flight. In the columns timeline the gap from
Ball 1's release to Ball 2's landing is the transit, (T − dwell)/2 = 0.305 s, fixed by the pattern.
The only freedom is WHERE Ball 1 is released: from the feed site, the cup does not move between the
release and the feed, and every later skill is unchanged (Ball 1 still lands at its own site at
T). It removes the transit the 13 refusals were about, and puts the feed catch in the condition
that caught 3/3 today and 13/14 in sitting 1. Costs: (a) Ball 1's opening throw is a 125 mm cross
throw with no learner rows behind it — the memory's state x is the release site, so its rows would
pool with that site's vertical throws; it flies `u = y_d` with no memory row, the columns Stop's
precedent, and its first catch at P2 follows the tracker; (b) the two balls pass ~0.19 s after
the release, safe with the bias corrected (median gap 84 mm, worst 29 mm) and touching only with an
uncorrected +67 mm feed — the runsheet orders bias before two balls; (c) Ball Butler's aim walk
toward A (`columns_feed_aim_toward_a_mm`, which existed only because the transit catch reversed the
cup against BB's +x arrival) is dropped in hop mode — it would cost 10 mm of that clearance.
Considered and rejected: a higher first throw of Ball 1 so the cup crosses early and waits (needs a
~1.2-1.4 m apex outside every swept box, and the cup still transits); feeding Ball 2 at Ball 1's
site (Ball 1 is in the air above it).

**Catch high: the tradeoff accepted.** The empty drop before contact is not slack: C-CUP-2 limits
the cup's downward acceleration to 0.7 g in the 0.125 s before touch-down (so an early ball is
pressed into the cup), and reaching 1.6 m/s under that limit takes ~146-169 mm of descent. Catching
higher therefore means a slower cup at contact — 0.96 m/s at 930 instead of 1.61 — so the ball
strikes the cup ~0.65 m/s harder (Ball 2 ~4.6 m/s relative at the 930 plane, was ~4.1), in exchange
for ~237 mm of stroke with the ball in the cup instead of ~137. The owner chose it knowing that
(2026-10-05). 930 rather than higher: past ~940 mm the dwell, not the stroke, limits how deep the
ball is carried (the bottom lifts), and hand acceleration rises at 970.

**What catch high costs beyond the contact speed: tolerance to a throw that flies high.** Found by the full gate, after the decision: the plan bench's self-toss rehearsal injects R3's cold-start plant error (+11 % launch speed, ~+23 % apex) and its catch-and-re-throw now refuses from ~900 mm (`HAND_LIMIT_ACC` 4305 rev/s² at 900 and 0.95 m apex, QP-infeasible at 930; clean at 880 and below), while NOMINAL self-toss throws plan at 930 (7/7 installs through `install_segment`, hand 3600). The columns box says the same thing from the other side: its apex ceiling fell from 1.047 to 0.950 m. A ball arriving much faster than commanded has to be stopped and re-thrown inside the same 0.27 s dwell from a slower, higher contact, and the hand cap runs out first. Kept at 930 for sitting 6 because the runsheet flies only columns patterns on a warm, migrated memory (sitting 5's apex errors were ~±2 %, not +23 %) and the owner chose the high catch knowing it trades contact speed; the fallback that restores the old tolerance is 880 mm (one constant, a re-sweep, the sims). A cold-start session (fresh `plant_id`) or a self-toss sitting at 930 should expect refused catches on its first high throws.

**The learner's apex had to change definition with the plane.** The learner reads a throw's apex
as v_z²/2g at the catch-plane crossing — the height above the CATCH plane — while the command side
is a flight time to that plane (`flight_s(u)`), thrown from the RELEASE plane. With the two planes
30 mm apart a perfect plant already read 14.5 mm high; at 930 it would read 35 mm low, a 50 mm step
for every memory row. The observed apex is now the u the planner's own model would have needed
(`schedule.apex_from_crossing`: solve v_c = rise/T − gT/2, apex = gT²/8), so a perfect plant reads
y = u at any plane offset, and the 390 existing rows (all catch 830 / release 860) were migrated once
by the same formula (`tools/migrate_memory_catch_plane.py`, backup + marker).

**Why `sites.py` is now gated.** The admissible box's gate hash covered seven planner files but not
the module that defines the release and catch planes, so moving the catch plane would have left the
committed box looking valid. The class is the morning's: two sources of truth with nothing coupling
them.

## Fix

**Hop entry.** `schedule.Skill.entry_hop` (THROW only, exclusive with `shadow_landing`) and
`compile_columns(..., entry='transit'|'hop')`: under `'hop'` (requires a feed) THROW 0 releases at
the feed site `sites[1]` with target `sites[0]`; every other skill is identical to the transit
schedule. The executor flies an `entry_hop` throw at `u = y_d` with no learner and no box lookup
(the Stop's bypass), writes no memory row, reads `caught` normally and tags the OUTCOME line
`(hop entry, no learner row)`; `plan_columns_first_cycle` already planned THROW 0 + the feed catch
generically (a test now pins it for the hop). `skill_node`: parameter `columns_entry` (default
`hop`; `transit` keeps sitting 5's entry; anything else WARNs once and uses `hop`), read at goal
start for the fed `columns`/`columns_1ball_fed` paths (Ball Butler and tracker feeds): the bridge
REST holds at the feed site and Ball Butler aims at the un-walked feed site (`columns_feed_bb_bias_mm`
still applies). `sim/skills_gate.py --columns-entry {hop,transit}` (default `hop` on fed trials),
and its two-ball learn loop now sizes attempts for the hop's extra row-less throw (sized for the
Stop alone it collected 29 rows and then looped 2-throw attempts to the 60-attempt cap).

**Catch high.** `sites.CATCH_CUP_Z_MM` 830 → 930 (release 860 unchanged). New
`schedule.apex_from_crossing(vz, rise)`; `executor._observed_apex_m` uses it with the live plane
offset. `tools/migrate_memory_catch_plane.py` (backup, marker, refuses a second run, `--dry-run`)
migrates rows recorded at the old planes. `admissible._GATED_FILES` gains `skills/sites.py`. Probes:
`feed_lateral_miss.py` (pattern-name fix, plane from `sites` with `--plane-mm` for older bags,
`SEAT_DEADLINE_S` 0.19 → 0.20 — the hand's reversal after touch-down measured on the planner,
0.184 s at 830 and 0.197 s at 930) and `throw_outcome_bag_probe.py` (`--plane-mm`). Tests whose
fixtures sat at the retired R2/R3 operating points (apex 0.9 / 0.85, dwell 0.30, separation 100)
moved to the R5 point, where the machine flies and the box is swept.

**Box.** Re-swept at the 930 plane (gate `3c9533225417`). Hop boxes are byte-identical; the
self-toss boxes at the 0.45 and 0.55 m bands each WIDEN by one 10 mm landing cell (x [−30, +30] →
[−40, +40] and x/y [−40, +30] → [−40, +40] mm), the other three are unchanged; the columns band 0.925-0.975 keeps its lateral box (x [−4, +4], y [−10, +20] mm) but its apex
ceiling falls from 1.047 to 0.950 m (the next grid flight, 0.924 s, trips the 3900 hand cap at the
high catch), so the learner can no longer command a columns throw above 0.95; the 0.875 band, empty
at 830, now holds a box up to 0.992 m.

## Verification

- Box sweep at the 930 plane, BLAS 1 thread, three patterns in parallel
  (`scratchpad/s6/sweep_run_s6.sh`, sitting 3's arguments: dwell 0.27, 350/5000/200000, hand
  3900, columns separation 125): run S6a (2026-10-05 19:23-19:57, Unit 1's planes) and S6b
  (20:10-21:06, the fully merged tree, no gated file changed between them) are **bit-identical**
  for all three parts (`cmp`; md5 columns abdf5ee4…, hop 8a762b68…, single be4cd55e…); merged in the
  committed order → `config/generated/admissible_box.yaml` md5 5f5e7ce5ef18810d6e30cd3937dae50a,
  gate `3c9533225417`.
- Install path at the R5 point (`scratchpad/s6/install_probe.py R5 830 880 900 930 950`,
  2026-10-05): a 6-throw columns schedule through the real `executor.install_segment`, 8/8 installs
  at every catch height; at 930 worst leg 288 mm/s / 4194 mm/s² / 156 401 mm/s³ (CATCH), hand
  3654 rev/s². At the retired R2 point the final rest-terminal CATCH refuses from 900 mm up
  (218 174 mm/s³ at 900, 272 121 at 930) — the reason 14 tests moved to the R5 point.
- Fed sims, hop entry, 930 plane (`python sim/skills_gate.py --learn --no-viewer --pattern columns
  --apex-m 0.95 --separation-mm 125 --target-throws 30 --feed-angle-deg 11.9 --feed-speed-mmps
  5600 [--one-ball-fed] --columns-entry hop`, 2026-10-05,
  `temp/logs/skills_gate_columns{,_1ball_fed}_hop_930_20261005.log`): two-ball **PASS 5/5** (1
  attempt each, 33 makes, 0 drops, longest 30, association OK, 0 wrong-ball rows); one-ball fed
  **PASS 5/5** (6 attempts, 37 makes, 0 drops, longest 30). The same two-ball run at the 830 plane
  before the attempt-sizing fix (the hop agent's, 2026-10-05) failed 5/5 at the 60-attempt cap with
  0 drops — the accounting defect, not a catch failure.
- Memory migration dry run (`python tools/migrate_memory_catch_plane.py --memory
  temp/learn/jugglebot/memory.csv --cutoff-epoch 1791192914 --dry-run`, 2026-10-05): 390 of 390
  rows, mean y2 shift −15.06 mm (expected −14.5 mm at a −30 mm rise and 0.95 m apex).
- Scoped suites on the merged tree before the test moves (`pytest <the nine skill-stack files> -n 4
  --dist loadfile`, 2026-10-05, `temp/logs/scoped_merged_s6_20261005.log`): 34 failed, 635 passed —
  20 cleared by the new box (`test_skill_node.py` 209 passed), 14 by the R5-point moves
  (`pytest tests/motion/test_skills_executor.py tests/motion/test_skills_admissible.py
  tests/sim/test_skills_gate.py -q -n 4 --dist loadfile`, 2026-10-05: **280 passed**).
- Audit (`/audit --unstaged`, 2026-10-05): CLEAN on behaviour; two narrative findings applied (the
  self-toss box sentence above; the migration tool now names a degenerate row before failing
  closed).
- First full gate (`./run_tests.sh --full`, 2026-10-05, `temp/logs/gate_full_r5_sitting6_prep_20261005.log`): 2 failed, 6062 passed, 9 skipped, 1 xfailed — both the plan bench's self-toss rehearsal (`LIMIT_JERK`) at the high catch with R3's +11 % plant error (the Discussion's tolerance paragraph; probe `scratchpad/s6/selftoss_probe.py`: clean at 830-880 mm, refused at 900 and 930). Those two tests characterise R3's rehearsal and now freeze the 830 plane (`tests/ros/test_skills_plan_bench.py` 69 passed).
- Full gate after the freeze (`./run_tests.sh --full`, 2026-10-05, `temp/logs/gate_full_r5_sitting6_prep_20261005b.log`): **6064 passed, 9 skipped, 1 xfailed in 302.82 s, serial 6 passed** (exit 0). Edited afterwards: this entry's Verification text only.

## Open questions

- Ball 1's opening hop has no learner: where it lands at P2 is first data in sitting 6; a target-aware
  memory key (or a per-pair memory) is the follow-up if it is biased.
- Ball Butler's x drift (~30 mm over 7 minutes) is larger than a fixed bias can cancel.
- The sim gate PASSED fed columns 5/5 at the old entry (2026-10-04) while the robot caught 0/6: the
  sim's platform tracks perfectly and does not model the transit overshoot or the 13 refusals.
- The sitting-4 refusal-margin decision (landing-time jitter vs the 10 % margin) is still open for
  the later catches of the pattern.
