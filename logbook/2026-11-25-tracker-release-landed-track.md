---
title: "Tracker: a CONFIRMED track whose frozen-fit landing is 0.2 s past stops taking the marker, so the re-throw of the same ball confirms (BUG-70); steady Throws escaped only by a one-frame marker dropout in the Carry"
type: bugfix
date: 2026-11-25
status: in-progress
phase: ""
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/tracking/matcher.py
  - tests/tracking/test_matcher_flight_over.py
  - logbook/2026-11-25-tracker-release-landed-track.md
subsystem: [tracking]
tags: [safety, testing]
---

## Summary

BUG-70 reproduces on today's code. Replaying bag `2026-11-05_22-46-00` through the real `BallTracker` (`tools/probes/tracker_bag_replay.py`), 8 of 49 announced Throws never confirm (tracker ids 3, 12, 15, 16, 44, 47, 48, 49). In each case the previous Throw's CONFIRMED track keeps the ball's marker through the Catch, the Carry and the whole next flight. That track is associated before the ANNOUNCED gate runs, so the new Throw never sees the marker.

The fix is one rule in `matcher.py`, `BallTracker._flight_over`. A CONFIRMED announced ball stops taking markers once its published landing comes from a frozen flight fit and the clock is past that landing plus `caught_grace_s` (0.2 s). It then misses every frame and ends CAUGHT through the existing lifecycle, which is how steady-state old tracks already end. On the 2026-11-05 bag the two Throws that really flew under a held marker, ids 3 and 47, now confirm at +0.0016 s. The other six never-confirmed ids were never thrown into a flight: for id 15 the marker rises from the cup to z 910 and stops, which is outside this rule's reach and not a Tracker failure. Every Throw that already confirmed (41 of 41) keeps the same confirm frame and a bit-identical landing on every in-flight frame. On the recent bags (2026-11-18, 2026-11-20 and 2026-11-23, one each), every Throw confirmed promptly by the old code (+0.0003 to +0.003 s) keeps its confirm frame and an identical landing. The change also catches a milder form of the same failure that the brief did not anticipate. Under the old code 7 + 11 Throws confirmed late (+0.008 to +0.66 s), because the previous track held the marker past the release and only lost it during the flight. They now confirm on the first or second in-flight frame, and 9 of them carry a different landing (up to 17.6 ms and 54 mm while both are fitted), because their fit now holds the whole flight. That changes what those Catches aim at. See the Discussion.

## Symptoms

From the Item: Throw 3 of the first fed Attempt on 2026-11-05 got no learner row because its tracker id never reached CONFIRMED. The live launch log has no confirm line for ids 4, 12, 15 and 16, and a late one for id 3. The replay gives the same set except 3/4, where the outcome depends on timing.

## Diagnosis: why the steady Throws escaped

This was measured on the replay: `$S/out/bug70/handover_trace.py`, which logs per frame the old track's prediction, its nearest marker and that marker's distance, over the window 1793879346-1793879367. Results are in `trace_1105.csv`.

- **While the ball sits in the cup, the old filter's velocity is nonsense.** After the Catch the marker rests at z 705-711 mm. The filter's position stays 20-24 mm below the marker, while its velocity state reads -5.5 to -6.5 m/s. Gravity integrates into the velocity, and the measurement pins only the position (process noise 5, measurement noise 1).
- **Through the re-throw ramp the old filter's offset grows to only 40-42 mm** (2→3 handover, 300 Hz frames), which is under the 50 mm association gate. With an unbroken marker stream, the old track therefore follows the ramp into the next flight and keeps the marker for the whole flight (id 2 tracked 783 frames, about two flights).
- **Every steady handover (ids 4→5 through 10→11) had one frame in the Carry where the ball marker was missing.** On that frame the nearest eligible marker was 1074-1142 mm away (a static marker at z 1669), 28-340 ms before the next throw_time. On that frame the filter coasts at -6.3 m/s, and the marker is 151 mm and then 284 mm from its prediction on the next two frames it is seen (id 4). It never re-gates: fwm 1, 5, 9, then CAUGHT. The failing handovers (2→3, 11→12) had no such frame.

So the steady Throws escaped by luck: a one-frame marker dropout in the Carry (cause not checked; the ball hidden by the cup is a guess). The Item's hypothesis, a fast ramp out of the Carry, is withdrawn. The ramps are alike, and the dropout is what differs.

## Discussion

**Why this rule.** The release has to satisfy one invariant: no marker is ever withheld from a ball whose landing prediction can still change. That is the only way the change could move a Catch aim, and so the Platform. `_flight_over` requires four things:

1. the ball is announced;
2. its published landing is the fit's (`landing_from_fit`);
3. the fit is `frozen`: it has samples, and the marker has since been at or below `landing_z + 250 mm`, so no sample has been admitted since and the fit's landing is a fixed number;
4. `t > landing_time + caught_grace_s`.

Withholding the marker admits no sample, so the fit stays frozen and the condition holds on every later frame. The landing the ball publishes is identical before, during and after the release. As a physical check, a ball still in free flight 0.2 s past a converged fit's crossing would be about a metre below the landing plane.

The hole is deliberate. If the re-throw lifts the marker back above the freeze floor before landing + 0.2 s (a Carry shorter than about 0.2 s plus the ramp's rise time), the fit unfreezes before the rule fires, and the old behaviour stands. This is conservative: it never steals, and it can only fail to help. On the 2026-11-05 bag, landing to the next throw_time is 0.395-0.421 s. On the recent bags, the release precedes the next announced throw_time by 0.25-0.28 s (2026-11-23). On the two-ball bags of 2026-11-18 and 2026-11-20 the next announcement is often the other ball's, so that lead reads negative and does not measure the Carry. No old track there failed to release because of a re-rise, except ids 11, 22 and 48 on 2026-11-18, which never met the frozen-fit condition (not inspected).

**Alternatives rejected.**

- *Release when the announced successor reaches its throw_time.* This is the Item's first suggestion, and it couples tracks by announcement order, not by the ball. In a two-ball Pattern, the next announcement is the OTHER ball's Throw while this one is still in flight. The rule would then withhold a marker from a ball in flight whenever the two balls pass within 50 mm of each other's filter, which is exactly the failure the invariant forbids. It also needs a pairing of schedule balls to tracker ids, which only the skill node knows (`flight_in_progress`'s latches).
- *Run the ANNOUNCED gate before the CONFIRMED association.* That lets any new announcement take a marker from an in-flight ball inside its 200 mm+ gate. It is the same class of failure, and it is wider.
- *Use the announced landing time, not the fit's.* The announcement can be off by tens of ms (and was 25 % off in 2026-09). The fit is the ball's own measured crossing. Requiring `landing_from_fit` also means a track that never converged keeps the old behaviour.
- *Fix the filter's in-cup velocity divergence.* That would change every CONFIRMED ball's filter, with a much larger blast radius. It also would not close the class: a well-behaved filter follows a smooth ramp too, and nothing would still end a track at its Catch.

**Tradeoff accepted.** The old track now ends CAUGHT about 0.2 s + 10 frames after its landing in every handover, where before it ended when the marker happened to drop out, or rode the next flight. Its published landing is unchanged until then. The skill node stops answering for it 0.2 s after its landing rather than at the re-throw; nothing reads it then (see the walk below).

**What a live Catch reads, before and after.** `skill_node._tracker(schedule ball)` → `ball_possession.flight_in_progress`: the latch with the latest release at or before now, and only if that tracker id is IN_FLIGHT and CONFIRMED, never an earlier latch.

- *During Throw N's flight up to its Catch:* both before and after, Catch N reads track N. The rule cannot fire before landing + 0.2 s, and `test_release_never_moves_a_landing_prediction` pins that landing frame-identical.
- *Carry, from landing N + 0.2 s to throw N+1:* the latest latch is still N. Before, the skill node returned N's frozen landing. After, it returns the same frozen landing for the 10 release frames, then None (CAUGHT). Catch N's aim was frozen before its landing (`catch_freeze_s`). The learner row reads `last_fit_landing`, latched in flight. `_advance_outcomes` takes fit landings it has already seen. None of them needs N after its landing.
- *Throw N+1's flight, in a failing handover:* before, the latest latch is N+1 and N+1 is ANNOUNCED, so the answer is None. Catch N+1 dispatches on the prior (learned or schedule landing) and is never re-aimed, `_advance_release_evidence` gets no fit, and the Throw gets no learner row. Track N (riding the marker, with a polluted fit and a Kalman landing) is never read because it is an earlier latch. After: N+1 confirms on the first or second in-flight frame, so its Kalman landing exists (rank 3, below the prior) and its fit converges within about 12 samples (rank 1). Catch N+1 is then aimed and re-aimed from the fit, as it is on every steady Throw. **This is the one behaviour change a Catch sees.** The Throws that used to fly open-loop on the prior now get the same tracker re-aim as every other Throw.
- *Steady handovers (new track confirmed within 3 ms before):* confirm frame and landing are bit-identical before and after on all four bags, so nothing a Catch reads changes.
- *Late-confirmed handovers (+0.008 to +0.66 s before; 18 Throws on the two recent two-ball bags):* this is the partial form of BUG-70. The old track held the marker through the release and lost it somewhere in the flight. Before, the new track's fit started late, converged at +0.22 to +0.72 s or not at all by the dispatch instant (2 Throws had no tracker landing at announced landing - 0.478 s), and was built from the descent only. After, the track confirms at release and its fit converges as on any steady Throw. While both are fitted, the landings differ by up to 17.6 ms and 54 mm, and they agree to 0.1-1.4 ms and 1-6 mm at the last in-flight sample. **So for these Throws the Catch aim and the re-aims change**: they move to the same full-flight fit every steady Throw already uses. This is the decision criterion's edge. No *steady* Throw changes, but these Catches do see a different, and by construction better-informed, landing.

## Fix

`ros_ws/src/jugglebot/jugglebot/tracking/matcher.py`: `_associate_confirmed_balls` asks `_flight_over` before gating. A released ball skips the gating loop and falls into the existing miss branch (fwm += 1, position from the filter, landing refreshed from the still-frozen fit), and `_check_lifecycle` ends it CAUGHT at `missed_frames_to_lose`, because it is past landing + grace. The `_check_lifecycle` comment notes the release.

**One control cycle, written out.** The tracker publishes `/balls` per processed mocap frame. The skill executor runs at 40 Hz and reads the cached latest `/balls` through `_tracker`. Take a 25 ms executor tick inside landing N + [0.2, 0.25] s. Before, track N is IN_FLIGHT, CONFIRMED, with a frozen-fit landing L. After, on the first released frame, track N is IN_FLIGHT, CONFIRMED, with landing L (the same value) and fwm 1. About 10 frames later it is CAUGHT and the tick reads None. No Catch dispatch or re-aim is pending for N at that instant, so no setpoint changes. At throw N+1 + 3 ms, N+1 is CONFIRMED with a Kalman landing. The executor's next tick ranks it below the prior, so the dispatch is unchanged until the fit converges. The first re-aim from that fit is the same path every steady Throw already takes, and is bounded by the existing re-aim limits (`catch_resend_max`, the freeze window). The change introduces no step and no discontinuity of its own.

## Verification

- (2026-11-25, `python tools/probes/tracker_bag_replay.py ~/Desktop/rosbags/2026-11-05_22-46-00 --out-dir $S/out/bug70/before`, at `origin/main` 62d82803) PASS 41/49; never confirmed: ids 3, 12, 15, 16, 44, 47, 48, 49. BUG-70 still reproduces on main.
- (2026-11-25, `python -I $S/out/bug70/ab_replay.py <worktree> ~/Desktop/rosbags/2026-11-05_22-46-00 ab_1105.csv`: one pass, two trackers, B = `_flight_over` pinned False, which is exactly the pre-change code) Before: 41/49 confirmed, never [3, 12, 15, 16, 44, 47, 48, 49]. After: 43/49, never [12, 15, 16, 44, 48, 49]. Confirm latency p50/p90/max 0.0016/0.0024/0.0033 s both ways. For the 41 Throws confirmed both ways: max |d confirm| 0.000000 s, max |d landing_time| 0.000000 s, max |d landing_position| 0.0000 mm, 0 `landing_from_fit` differences. Id 43 drops out of `/balls` 45 frames (0.15 s) earlier after the change; its fit put its landing 0.18 s before the announced one, so those frames come after its own fit landing + 0.2 s.
- (2026-11-25, `$S/out/bug70/ann_window.py`, Throw 15) The eligible marker nearest the announced path rises from z 706 to 910 mm over +0.15 s and stops. No flight happened, so a never-confirmed id 15 is correct.
- (2026-11-25, same `ab_replay.py`, `~/Desktop/rosbags/2026-11-18_11-59-50`) 149 announced. Before: 129 confirmed, latency p50/p90/max 0.0017/0.0031/0.3901 s. After: 134 confirmed, max latency 0.0115 s. Of the 129 confirmed both ways, 122 are unchanged and 7 changed. All 7 were late before (confirm +0.0084 to +0.3901 s), and their max |d landing| is 0.046 s / 303.5 mm, which includes frames where the old code had only a Kalman landing.
- (2026-11-25, same, `~/Desktop/rosbags/2026-11-20_13-48-32`) 151 announced. Before: 135 confirmed, max latency 0.6642 s. After: 142 confirmed, max latency 0.0041 s. 124 of 135 are unchanged. 9 were late before (+0.0106 to +0.6642 s). Ids 96 and 149 keep their confirm and landing but leave `/balls` 9 and 13 frames earlier, after their own fit landing + 0.2 s, which their fit put before the announced landing.
- (2026-11-25, same, `~/Desktop/rosbags/2026-11-23_14-31-06`) 9 announced, 9 confirmed both ways, everything identical (0 s, 0 mm).
- (2026-11-25, `$S/out/bug70/frame_dump.py`, the 7 Throws with the largest change, from T-4 s to T+1.3 s) Over frames where both trackers have a fitted landing: max |dt| 0.0000 to 0.0176 s and |dpos| 0.0 to 53.9 mm. At the last in-flight sample they agree within 1.4 ms and 6 mm. The old code had no tracker landing at all at the dispatch instant for 2 of the 7.
- Not checked: whether the never-confirmed ids left on 2026-11-18 (15) and 2026-11-20 (9) also flew. On 2026-11-05 all six remaining never-confirmed ids were checked with `ann_window.py` (12, 16, 44, 48, 49, and 15 above): no marker follows the announced path after the release. These look like the end-of-Attempt Throws BUG-114 describes, where the ball was already lost.
- (2026-11-25, `flock /tmp/jugglebot-tests.lock python -m pytest -q tests/tracking/ tests/ros/test_ball_tracker_flight_fit.py tests/ros/test_ilc_measurement_probes.py tests/ros/test_ball_tracker_gate.py tests/ros/test_ball_tracker_frame_gap.py tests/ros/test_ball_possession.py tests/ros/test_ball_tracker_announced_motion.py tests/ros/test_outcome_detail.py tests/sim/test_night_shift_settings.py`) **509 passed in 45.42 s**. The readers came from `grep -rln matcher tests/`.
- New tests: `tests/tracking/test_matcher_flight_over.py`, 6 tests on a synthetic ball at 300 Hz (the bag's rate). With `released = False` forced (the pre-change behaviour), 5 of 6 fail (2026-11-25): both re-throw ramps never confirm Throw 1, the landed track never ends, and the in-flight checks fail on the unconfirmed re-throws. Two weakened rules were checked as mutations. Releasing from 0.3 s before landing fails both `test_a_ball_in_flight_never_loses_its_marker` cases. Dropping the frozen and grace conditions fails 4 tests, including `test_release_never_moves_a_landing_prediction`. Restored, 6/6 pass.

## Outcome

This is held on branch `offline/bug70` and is not live until merged and `colcon build`. At the next launch, if merged: a Throw whose previous Throw's track would have kept the marker now confirms at release. Its Catch is aimed and re-aimed from its own fit instead of the prior, and it gets a learner row. Old tracks end CAUGHT about 0.2 s plus 10 frames after their own fit landing, instead of at a random dropout or never. Promptly confirmed Throws are unchanged. Status stays `in-progress` until a Sitting shows the rescued Throws catching.
