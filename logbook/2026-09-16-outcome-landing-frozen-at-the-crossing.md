---
title: "The outcome landing is frozen at the crossing — the learner was being taught the next flight"
type: bugfix
date: 2026-09-16
status: done
phase: "two-ball-skill-stack — R3"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py (new _consider_landing — freeze at the crossing, pre-crossing guard, band at admission; _PendingOutcome.best_lead_s / rejected_flight_s / n_rejected / t_next_release_s; _next_release; _bound_by_next_release; OUTCOME_NEXT_RELEASE_EPS_S; the band drop named at finalise)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/memory.py (FLIGHT_RATIO_BAND + flight_in_band — the one definition; a row outside it is dropped on LOAD and refused on APPEND)
  - tools/probes/outcome_landing_replay.py (new — replays /balls against the written memory rows: old pick vs new pick per throw)
  - tests/motion/test_skills_executor.py (7 new tests; the wild-landing test rewritten — a diverged estimate now writes NO row)
  - tests/motion/test_learner.py (band unit + append/load guards, on the sitting's real rows)
  - tests/ros/test_skill_node.py (the stop-mid-flight test now feeds the in-flight sample the tick loop takes)
  - temp/learn/README_QUARANTINE_20260916.md (which files/rows are contaminated, and the mv command)
  - plans/active/two-ball-skill-stack.md
  - logbook/2026-09-16-outcome-window-and-computed-catch-deferral.md (superseded-in-part note)
  - tests/hardware/session_skills_r3_apex_ladder.md (quarantine step before the next sitting)
---

## What / Why

The 2026-09-16 16:22 apex-ladder sitting (log
`temp/logs/launch_r2gate_20260916_1622.log`, bag `2026-09-16_16-22-22`) wrote
learner rows that cannot describe a throw. `armB-090` attempt 1 (t0
1789540415.456, commanded flight `u[2]` = 0.8569 s, four chained throws ~1.15 s
apart) recorded **observed flights of 2.2317, 2.2310 and 1.1554 s** — a 0.9 m
toss flies 0.857 s and cannot fly longer than ~0.86 s from any release the box
admits. The learner did exactly what those rows told it to: attempt 2 (3 rows)
commanded 0.788 s, attempt 3 (4 rows) 0.686 s, the operator logged "very low
throws" and "platform wobble", and attempt 3 ended `ABORTED_NO_RELEASE` with
the ball never leaving the cup. `armA-090` attempts 2–3 shrank the same way,
and the 14:16 sitting's rows (0.70/0.75/0.63/0.77/0.90 s for 0.6387/0.6996 s
commands) were already partly contaminated.

Two mechanisms, one shape: **the row's landing observation was never closed.**

## Discussion

### The contamination mechanism

`_advance_outcomes` set `pend.best_landing = landing` on EVERY tick where the
tracker returned an estimate that (a) post-dated the release and (b) sat
outside `OUTCOME_GUARD_S` of its own predicted crossing — `abs(...)`, so a
sample taken well AFTER its own crossing counted too. The last estimate before
the row finalised therefore won, with no test that it described the flight the
row is about. Three things then lined up:

* **The window got wider that morning.** The same-day fix
  ([2026-09-16-outcome-window-and-computed-catch-deferral](2026-09-16-outcome-window-and-computed-catch-deferral.md))
  moved `finalise_at` from a point sample at `t_land_scheduled + 0.15 s` to
  `CAUGHT_LAND_DEFER_CAP_S + CAUGHT_WINDOW_S` = up to **+0.70 s** past the
  scheduled landing, to catch a late-settling seat. At the R3 chained
  operating point the ball is re-thrown ~0.30 s after its catch, so the row
  now stayed open across the NEXT release. That change is right and stays; it
  widened the exposure, it did not create it.
* **The tracker holds one continuous track.** The 2026-09-15 tracker fix (all
  markers, a 200 mm announced gate) made a continuous same-id track across
  flights the norm, so after the catch that id's estimate is about the next
  flight — or, while the ball sits in the cup, about "now".
* **In-flight estimates were themselves a beat late.** The replay (below)
  shows the contaminated rows are NOT only post-catch samples: for every throw
  but the last of each attempt, the estimates taken *while the ball was in the
  air* already predicted `t_land ≈ release + flight + beat` (ratios 2.21–2.99
  of the command). That is a tracker-side anomaly this entry does not fix and
  the "Open" section names.

### Why the freeze, and why a band as well

Climbing one level (CLAUDE.md): the class is *"an estimate that is not about
this flight was allowed to become this flight's outcome"*, and it has three
members — the next flight's landing, a held ball's "lands now", and a diverged
filter. One enforcement point closes all three: **the row's observation is
admitted only while the ball is still in the air, and the estimate must itself
be physical.** `_consider_landing` is now the single gate, with four tests
(previous-flight, pre-crossing guard, physical band, frozen-at-the-crossing),
and `_advance_outcomes` no longer decides anything.

The band (`memory.FLIGHT_RATIO_BAND` = `(0.5, 1.6)` × the COMMANDED flight)
is derived from physics, not from scatter: flight is `t = 2v/g` and apex
`h = g t²/8`, so a ratio in flight IS the ratio in release speed and its
square in apex. This plant's measured 25 %-fast throw (apex 1.38 m on a 0.9 m
command) is exactly `r = 1.238`. Replayed over both sittings, the estimates
taken while the ball was genuinely in flight span `r = 0.97 .. 1.53` (n = 22)
and the contaminated rows `r = 2.21 .. 2.99` (n = 21) — a factor of 1.45
between the clusters with nothing in between. 1.6 clears the largest genuine
ratio, admits a plant throwing 60 % fast, and still rejects every contaminated
row by ≥ 38 %; 0.5 is an apex a quarter of the commanded one, i.e. a failed
release rather than a throw to learn from.

**The band alone would not have been enough, and neither would the freeze.**
A ball sitting in the cup has its landing predicted at ~now, so that
contaminant's ratio is `beat / commanded` — 1.348 at this operating point
(`armB-090` row 3: 1.1554 s against 0.8569 s), inside *any* band wide enough to
admit this plant, and arbitrarily close to 1 for a shorter beat. Only "the ball
has landed, stop looking" separates that class. Conversely the freeze alone
leaves the in-flight-but-a-beat-late estimates, which the band rejects. Both,
or neither works.

### What was ruled out

* **Filtering on the tracker id instead** (require a fresh track per flight).
  The replay shows the ids do change per flight on these bags, and the rows
  were contaminated anyway — the estimate was wrong *within* the correct
  track. An id-freshness rule would have looked like a fix and changed nothing.
* **"Prefer the estimate with the smallest lead to its own crossing"** — my
  first implementation of "closest to the crossing". Rejected *by the replay*:
  the tracker's first estimates after release predict a crossing almost
  immediately (the ball is barely above the plane and slow), so the smallest
  lead in a flight is usually that opening garbage. It turned three genuine
  rows into 0.02–0.05 s flights. The last admissible estimate wins instead,
  and lateness is judged by the PHYSICAL band, not by vantage point. This is
  also why the band is tested at ADMISSION and not only at finalise: an
  accepted "lands 30 ms from now" estimate would otherwise pull the freeze
  instant 30 ms past the release and close the row there.
* **Narrowing `CAUGHT_WINDOW_S` back down.** That would trade a measured catch
  verdict (5 of 7 seats on 09-16 landed later than 0.15 s) for a landing
  verdict, i.e. fix one defect by reinstating another. The window stays; what
  it may no longer do is reach past the next release.
* **Dropping `caught=False` rows from the learner's memory.** Plan § 2.5
  step 6 says the observed `y` is appended as one row, and the paper's memory
  is over landings regardless of possession (§ 2A); nothing in the plan
  conditions a row on the catch. Left alone deliberately — an uncaught throw
  still *landed somewhere*, which is exactly what the command learner models.
  No change to the update law.

## Fix

1. **`_consider_landing` (new, `executor.py`)** is the only path by which a
   row acquires a landing. Five tests, in order: the landing post-dates this
   release; the sample is at least `OUTCOME_GUARD_S` BEFORE the crossing it
   predicts (was `abs()`, which admitted post-crossing samples); `t_abs_s` is
   before `_landing_instant` — past which the row is **frozen**; the
   estimate's own crossing is before this ball's next release (an in-flight
   sample can still point past it, which the freeze alone does not catch);
   and the implied flight is inside `FLIGHT_RATIO_BAND` × the commanded
   flight. The last survivor wins. A refused out-of-band estimate is remembered
   (`rejected_flight_s`, `n_rejected`) only so the dropped row can name its
   reason: `no row: observed flight X s outside [a, b] of commanded Y s`.
2. **The verdict window can no longer swallow the next release.**
   `_register_outcome` records this ball's next scheduled release
   (`_next_release`, the mirror of `_previous_release`, so `_outcome_window`
   stays a pure function of the row) and `_bound_by_next_release` pulls
   `finalise_at` back to `t_next_release − OUTCOME_NEXT_RELEASE_EPS_S`
   (0.010 s), floored at `t_open` so a crowded schedule cannot invert the
   window. `_landing_instant` — the freeze instant — takes the same bound
   from the same helper (`_next_release_bound`), so on a CROWDED schedule (a
   re-release before the scheduled landing) the freeze and the verdict close
   stop at one instant instead of leaving a gap in which the next flight's
   estimate is still admissible. Nothing a catch needs is lost: the window's only business past the
   landing is the SEATED latch, and the ball is in the cup for that whole
   interval. The chained rows still read `caught=True` (replay, and a new
   chained executor test).
3. **`memory.py` owns the band** (`FLIGHT_RATIO_BAND`, `flight_in_band`) — one
   definition, imported by the executor. A row outside it is **dropped on
   load** with a warning (so a contaminated file already on disk cannot poison
   the next sitting) and **refused on append** (`ValueError`), so the
   executor's own check is belt and braces rather than the only guard.
4. **Quarantine**: `temp/learn/README_QUARANTINE_20260916.md` names the
   contaminated files and rows and gives the `mv` command. Nothing deleted.

### Replay — what the new rule would have written

`tools/probes/outcome_landing_replay.py` (new, reusable) reconstructs each
written row's landing pick from the bag's `/balls` stream, using the
executor's own `_consider_landing`. Full table:
`temp/probes/outcome_landing_replay_20260916.txt`. Extract:

| plant | att | thr | commanded | y old (row) | y new | admitted? |
|---|---|---|---|---|---|---|
| armB-090 | 1 | 1 | 0.8569 | 2.2317 | — | NO ROW |
| armB-090 | 1 | 2 | 0.8569 | 2.2310 | — | NO ROW |
| armB-090 | 1 | 3 | 0.8569 | 1.1554 | 1.1368 | yes |
| armB-090 | 2 | 1 | 0.7879 | 0.9953 | 0.9045 | yes |
| armB-090 | 3 | 1 | 0.6855 | 0.8691 | 0.7577 | yes |
| armA-090 | 1 | 1..2 | 0.8569 | 2.0538 / 2.1206 | — | NO ROW |
| armA-090 | 1 | 3 | 0.8569 | 1.0597 | 1.0627 | yes |
| armA-050 | 1 | 1..3 | 0.6387 | 0.7013 / 0.7544 / 0.6255 | 0.6886 / 0.7891 / 0.6223 | yes |
| armA-050 | 2 | 1..2 | 0.5988 | 1.5250 / 1.6022 | — | NO ROW |

Across both sittings: **43 rows written, 21 of which the new rule refuses
outright**; the 22 it admits span `y/u` = 0.974 .. 1.528, against 0.979 ..
2.985 as written. The old rule is reproduced to 0–12 ms on every row, which is
what licenses the comparison.

**Same commit — the ladder's SEATED precondition reads the DEBOUNCED bit.** Two of the sitting's three `REJECTED_NO_BALL` refusals (1789540058.4, 1789540480.2) were a single raw-sample carry-flicker during the post-hold park motion with the debounced `ball_held` True throughout (the third, 1789539829.9, was a genuinely empty cup at session start). `skill_node` now keeps two evidence values: the raw bit for the executor's release/catch-edge observer (the debounce lags a departing ball by ~240 ms) and a debounced twin `_possession_evidence_stable` for `Observations.ball_evidence`; test `test_the_ladder_reads_the_debounced_bit_and_the_observer_the_raw_one`.

## Verification

* Scoped suite (2026-09-16,
  `pytest tests/motion/test_skills_executor.py tests/motion/test_learner.py
  tests/ros/test_skill_node.py tests/sim/test_skills_gate.py
  tests/ros/test_skills_plan_bench.py -q`): **268 passed, 1 xfailed in
  40.34 s**.
* Replay (2026-09-16, `python tools/probes/outcome_landing_replay.py --bag
  ~/Desktop/rosbags/2026-09-16_16-22-22 --bag ~/Desktop/rosbags/2026-09-16_14-16-38
  --date 20260916`): 43 rows, 22 admitted (`y/u` 0.974–1.528), 21 dropped; old
  rule reproduced within 12 ms on all 43.
* Post-audit (2026-09-16): the FREEZE now shares the next-release bound with
  the verdict close (`_next_release_bound`, one definition of the 0.010 s
  margin), and an estimate whose own crossing lands at or after the next
  release is refused outright — swept tick by tick across a crowded schedule
  (`t_sched` 10.0 s, next release 9.5 s, a 15.0 s estimate offered from t = 0)
  in `test_a_crowded_schedule_freezes_at_the_next_release_not_the_schedule`.
  Re-running the replay with the bound modelled leaves both figures unchanged
  (22 admitted / 21 dropped, `y/u` 0.974–1.528).
* Docs gate (2026-09-16, `pytest tests/sim/test_plans_index.py
  tests/sim/test_logbook_front_matter.py -q`): see the run recorded in the
  commit for this entry.

(2026-09-16, `./run_tests.sh --full`, log `temp/logs/outcome2_full2_20260916.log`, the final tree incl. the audit follow-ups and the debounced NO_BALL fix): **PASS — parallel 6220 passed, 9 skipped, 2 xfailed in 288.43 s; serial 6 passed in 19.13 s.**

## Open

* **The tracker's in-flight landing estimate ran ~one beat late on every
  chained throw but the last** (ratios 2.21–2.99 of the command, from samples
  taken before the scheduled crossing). The executor now refuses those rows,
  so the learner is safe, but a chained sitting will produce a memory row only
  for the last throw of each attempt until this is understood — and the same
  estimate feeds `catch_aim_source=tracker`, which is why the live default
  stays `schedule`. Next step: replay `/balls` against `/mocap_data` ground
  truth (`tools/probes/throw_outcome_bag_probe.py` already fits the parabola)
  for one chained attempt and ask what `time_at_land` is being computed from.
* The band's upper edge (1.6) sits only ~5 % above the largest genuine ratio
  measured (1.528). If the hand's torque-FF landing moves the plant's bias
  again, re-measure before assuming the band is still slack.
