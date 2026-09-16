---
title: "Outcome verdict taken over a window after the OBSERVED landing; the legacy catch re-aim no longer fights a skill-stack plan"
type: bugfix
date: 2026-09-16
status: done
phase: "two-ball-skill-stack — R3"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py (CAUGHT_WINDOW_S 0.15 → 0.35 (owner ruling) measured; new CAUGHT_LEAD_S / CAUGHT_LAND_DEFER_CAP_S; _outcome_window; _PendingOutcome.caught_seen latch; _finalise_outcome reads the latch, not a fresh sample)
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py (_cycle_owner + _OWNER_CYCLE/_OWNER_SEGMENT plumbed through _install; _SEGMENT_OWNED refusal in _on_dynamic_target before any solve; _reject_cycle demotes REPLAN_WINDOW to a throttled WARN)
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py (_cycle_stroke_floor grants the dive tolerance to a seed AT the homed zero, not only below it)
  - config/generated/admissible_box.yaml (re-swept: gate_hash only)
  - tools/probes/caught_window_bag_probe.py (new — the landing→SEATED delay from an MCAP bag)
  - tools/probes/README.md (register the new probe)
  - tests/motion/test_skills_executor.py (8 window tests)
  - tests/motion/test_validate_cycle.py (4 stroke-floor tests)
  - tests/ros/test_unified_cycle_integration.py (4 ownership / log-level tests)
  - sim/skills_gate.py, tests/sim/test_skills_gate.py (dated note: the R3-j drift narrative's "~94 ms" was the OLD window)
  - tests/hardware/session_skills_r3_apex_ladder.md
  - plans/active/two-ball-skill-stack.md
---

## Superseded in part (2026-09-16, same day)

Defect 1's widened verdict window stands, but the window it widened was also
the row's LANDING-observation window, and at the chained operating point it
reached past the ball's next release — the learner was then taught the next
flight (2.23 s flights for an 0.857 s command). The landing observation now
freezes at the crossing and the window is bounded by the next release:
[2026-09-16-outcome-landing-frozen-at-the-crossing](2026-09-16-outcome-landing-frozen-at-the-crossing.md).

## What / Why

Three defects from the 2026-09-16 R3 sitting (log
`temp/logs/launch_r2gate_20260916_1416.log`, bag `2026-09-16_14-16-38`), two
attempts of four chained self-tosses. The operator caught 5/5 singles; the
learner was told it had caught 1.

**1. The `caught` verdict was a point sample at the wrong instant.**
`_finalise_outcome` read the possession observer ONCE, at
`t_land_scheduled + CAUGHT_WINDOW_S` (0.15 s). The plant throws ~8 % fast, so
the ball arrived +0.06..+0.20 s after the schedule said it would, and the
seated verdict then debounces on top of that — the sample landed on a cup the
ball had not reached yet. Four of the five rows read `caught=False` on catches
that happened. The verdict is now LATCHED (`_PendingOutcome.caught_seen`) by
any SEATED reading inside a window taken around the OBSERVED landing
(`_outcome_window`), and `finalise_at` follows the observed landing rather than
the scheduled one. The "no landing → no row" rule is untouched.

**2. 160 `REPLAN_WINDOW` ERROR lines had nothing to do with the executor.**
Every catch-with-throw dispatch was followed by 10–20 `plan_cycle refused:
REPLAN_WINDOW` ERRORs at ~10 ms spacing. The premise this unit was briefed on —
that the executor was PROBING the planner, re-calling `install_segment` every
tick until the splice cleared the previous throw's detach cone — is wrong, and
the log says so plainly: **every `install_segment` in the sitting was accepted
on its first call** (skill 2 splice_k=20 at 309.381, skill 3 splice_k=58 at
310.304, skill 4 splice_k=95 at 311.247, skill 5 splice_k=133 at 312.167). The
refusals are on `trajectory/plan_cycle`, the superseded FSM path — the caller is
the legacy reactive catch chain, re-aiming the skill stack's own plan. There was
no deferral loop to compute, so none was built; the fix is an ownership guard.
See the timeline below.

**3. `_cycle_stroke_floor` refused a seed of EXACTLY 0.0 rev.** Flagged and
left unfixed by `logbook/2026-09-16-hand-park-refusal-retired-rest-homes-the-hand.md`;
closed here. The dive tolerance was granted only to a seed strictly BELOW the
homed zero, so 0.0001 rev and −0.002 rev both passed and 0.0 — the one value
the machine parks at, ±0.0002 rev on the encoder — refused `HAND_STROKE`
("hand position −0.000 rev outside [0.000, 9.959]") on the cubic's own interior
curvature. The comparison is now strict, so the parked case and the sub-zero
case are one case.

## The measured seated delay

`tools/probes/caught_window_bag_probe.py` — the delay from the tracker's own
descending crossing of the 830 mm catch plane (`/balls`, linearly interpolated)
to the first debounced SEATED sample on `/hand_telemetry`
(`ball_held_valid and ball_held`; 100 Hz, `ball_held` is already debounced).
Both series on their own header stamps, the same Unix epoch the schedule runs
on. `ball_held_valid` never went False in any catch window in either bag.

| bag | n | per-throw delay (ms) | max | median |
|---|---|---|---|---|
| `2026-09-16_14-16-38` | 7 | +112.0, +41.3, −47.5 ⚠, +191.2, +122.3, −28.6, +122.3 | +191.2 | +112.0 |
| `2026-09-15_18-51-37` | 22 | −40.2 .. −49.3 (tight) | −40.2 | −44.5 |

⚠ **The two measurements disagree on exactly one row, and it is the one the
window turns on.** An independent pass over the same bags reported 09-16 throw 3
landing at 311.9977 and seating **+281.9 ms** later; the committed probe reports
that track crossing at 311.3665 with a −47.5 ms "delay" and no crossing near
311.998 at all. The committed probe takes only the FIRST descending crossing per
`/balls` track, so a track the tracker keeps across two flights loses its second,
real landing — that is the more likely error, and it is now recorded as a known
limitation in the probe's own docstring (with the fix named and deliberately not
applied, so the numbers here stay this version's output). **Everything else
agrees to the millisecond**: the +191.2 ms maximum and all 22 rows of 09-15.

Two facts fall out, and both are now constants:

* **The delay is not one-signed.** On 09-15 the sensor seated 39–48 ms BEFORE
  the interpolated crossing on all 22 throws — physically ordinary (the crossing
  estimates a plane the cup rim reaches first), and a window that OPENED at the
  landing would have scored every one of them a miss. Hence
  `CAUGHT_LEAD_S = 0.10` (twice the largest early seat, and still ~0.6 s inside
  the empty-cup interval that precedes every arrival, so it cannot read the
  previous ball).
* **`CAUGHT_WINDOW_S = 0.35`** (owner ruling 2026-09-16). I first set 0.25,
  covering the largest delay the probe reproduces (+191.2 ms) and deliberately
  EXCLUDING the +281.9 ms row as a bobble — `ball_held` went True, held ~150 ms
  and went False again, which read as a ball that never settled. **That reading
  was wrong, and the operator's own note is the evidence**: armA-050 is logged
  "worked", every ball caught, and the throw AFTER that row was itself caught,
  so the ball never reached the floor. The False ~150 ms later is the NEXT
  throw's RELEASE, not a drop. A catch that settles late is a catch. 0.35 s
  clears the +281.9 ms pairing by ~70 ms and the probe's own +191.2 ms by
  159 ms. The 0.15 s this replaces was too short for five of the seven 09-16
  throws.

`CAUGHT_LAND_DEFER_CAP_S = 0.35` bounds how far a tracker estimate may push
`finalise_at` (~1.8× the largest late crossing measured), so a diverged filter
cannot hold a learner row open forever. **It does NOT move with the window**
(asked at the ruling): the cap is a tracker-trust bound on how late the OBSERVED
LANDING may move the anchor, the window is a sensor bound on how long the SEAT
may take after it. Worst-case finalise latency is their sum, **0.70 s**, still
inside the ~0.95 s beat — the only deadline the memory has.

**Replay of the sitting's five rows under the new rule** (scheduled landings
from the log's `CATCH-AIM … t_land`, observed landings and seats from the probe):

| attempt | throw | sched | observed | seated | was | now |
|---|---|---|---|---|---|---|
| armA-050 | 1 | 310.133 | 310.1957 | 310.3083 | True | **True** |
| armA-050 | 2 | 311.072 | 311.2051 | 311.2482 | False | **True** |
| armA-050 | 3 | 312.011 | 311.9977 | 312.2797 | False | **True** (the late seat; at 0.25 s it would have been False) |
| armA-060 | 1 | 341.838 | 341.9068 | 342.0298 | False | **True** |
| armA-060 | 2 | 342.837 | 343.0330 | 343.0050 | False | **True** (needs the LEAD: the seat is 28 ms before the crossing) |

**All four False rows flip to True and the True row stays True — 5/5, which is
what the operator recorded.** Row 3 is the one the window width decides: the bag
shows `ball_held` True from 312.2797 for ~16 samples (~150 ms), then False from
~312.4383 until throw 4's catch at 313.2002. So the ball did not remain seated —
but it was re-thrown, not dropped, because throw 4 flew and was caught. At
0.35 s that row reads `caught=True`; at 0.25 s it read False. The two
`no landing estimate was ever observed` drops (armA-050 throw 4, armA-060
throw 3) each gain 0.10–0.16 s of tracker time and MAY now produce a row; that
is not determinable from the log, because what the executor's own `tracker`
callable returned in that extra interval is not recorded.

## The REPLAN_WINDOW timeline — reconstructed

Attempt 1, skill 2 (CATCH-with-throw), t0 of the active plan = 308.964
(the launch THROW, a fresh origin), dt = 0.025 s:

| t (abs) | what |
|---|---|
| 309.315 | executor dispatches skill 2 |
| 309.381 | `install_segment CATCH ball 0: splice_k=20, plan 63.1 ms` — **ACCEPTED, first call** |
| 309.395 | `ball_tracker_node`: "Ball 2 announced by 'jugglebot', throw in 1.04s" |
| 309.519 | `trajectory/plan_cycle is the superseded FSM install path` — WARN-once: the FIRST plan_cycle call of the session |
| 309.520 … 309.841 | 28 `REPLAN_WINDOW` refusals, splice knot walking 34 → 46 against "release knot 58" |

**Which plan's numbering, and why 24 knots apart.** Both numbers are on the
ACTIVE plan's clock, whose `t0` is the launch THROW's fresh origin. Knot 58 =
1.450 s is the release the catch-with-throw segment CARRIES — the throw skill 2
installed at 309.381, whose release the executor's own log line puts at
309.315 + 1.048 = 310.363 ≈ t0 + 58·dt. Knot 34 ≈ 0.850 s is where a splice
`LEAD_S` = 0.150 s ahead of ~309.52 lands. So the re-aim is trying to cut the
plan 24 knots (0.60 s) BEFORE a release that is already committed and already
has a ball scheduled to leave on it — `replan_tail` refuses every such splice at
`k_s <= k_rel + n_detach` (`n_detach` = 2, hence the "+ detach knots 60"),
because the re-solved tail would drop that ball's detach-cone equalities.

**The caller.** `catch_coordinator_node` publishes `catch/dynamic_target` off
the tracker, and `trajectory_node._on_dynamic_target` routes any such update to
`_replan_cycle_from_target` whenever `self._cycle is not None` — which a
skill-stack plan is, because `install_segment` installs a `CyclePlan` through
the same record. The announcement the skill stack itself publishes is what wakes
the chain. 89 refusals in attempt 1, 71 in attempt 2.

**They were NOT full solves.** `replan_tail` checks its two cone bounds before
the QP, and the refusal spacing (5–25 ms, mean ~10 ms) is the publisher's
100 Hz, not a solve's. That corrects the brief's estimate of 10–60 ms each. What
they cost is 160 needless service callbacks on the single-threaded executor
`install_segment` shares, and an unreadable log.

**The fix is ownership, not arithmetic.** `_install` now records
`_cycle_owner` under the same lock as the record it describes
(`_OWNER_SEGMENT` for `install_segment`, `_OWNER_CYCLE` for the three
`plan_cycle` modes, cleared with the record on any non-cycle install), and
`_on_dynamic_target` refuses a segment-owned plan `_SEGMENT_OWNED` before the
reach-freeze ladder and before any solve. The feedback still goes out on
`trajectory/target_feedback` so `catch_coordinator`'s accept/reject correlation
keeps working. The legacy path is bit-for-bit unchanged. Separately,
`_reject_cycle` now logs `REPLAN_WINDOW` as a **throttled WARN** (5 s) and keeps
ERROR for every other code: a reactive caller can hit that one refusal at its
publish rate through no fault of its own, and the plan it asked about is fine.

## `SPLICE_TOO_LATE` on armA-060 skill 4 — report only

The attempt ended `SPLICE_TOO_LATE`: "the solve took 0.147 s … budget 0.117 s
from dispatch". **Removing the wasted replans would not have saved it.** The
last `plan_cycle` refusal of that attempt is at **342.450**; the dispatch is at
**342.942** and the install lands at 343.098. Zero refused replans ran in the
0.49 s before that solve, so there was no contention from them to remove. The
solve is simply 2.3× the attempt's own median (that attempt's other
`install_segment` solves: 59.9, 35.0, 68.4, 64.0 ms). Cause unexplained — a
harder QP at knot 101, or sitting load (bag recording + GUI) — and left open. It
is the same class as the 135× solve inflation under load recorded for UH-3.

**Audit follow-ups (same day).** The SEATED latch is attributed to exactly ONE open row per tick — the one whose landing instant (`min(t_sched, t_obs)`, `_landing_instant`) is nearest the sampled tick — because the cup sensor is ball-blind and a columns beat (~0.58 s) is shorter than a row's worst-case window (0.80 s); four tests pin it. The bridge writes the hand-park echo only when ACTIVATE fired AXIS_ALL (a single-axis bench activate never touches the hand).

## Verification

- (2026-09-16) `pytest tests/motion/test_skills_executor.py
  tests/motion/test_trajectory_feasibility.py tests/motion/test_validate_cycle.py
  tests/motion/test_validate_cycle_vectorised.py tests/ros/test_skill_node.py
  tests/ros/test_install_segment.py tests/sim/test_skills_gate.py
  tests/ros/test_unified_cycle_integration.py tests/ros/test_trajectory_node.py
  tests/ros/test_catch_coordinator_node.py tests/ros/test_reload_integration.py
  tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py -q` →
  **631 passed, 1 xfailed in 127.16 s** (the xfail predates this change).
  `./run_tests.sh` was NOT run in this unit (out of its brief); the changed
  paths are under `controller`-adjacent `motion/`, so the pre-commit gate for
  this work is `./run_tests.sh --full`.
- (2026-09-16) `python tools/admissible_sweep.py --site-pairs both
  --single-apex 0.5 0.6 0.7 0.8 0.9 --leg-jerk 150000`, 269.4 s → **every box
  identical to the previous file**; only `swept_at` and `gate_hash`
  (`d38f76690bc1` → `37d68192b0e1`) changed. The `feasibility.py` edit changes
  `gate_hash` by construction (it is `sha256(feasibility.py + segments.py)`), so
  the re-sweep is mandatory and its result is the proof the gate did not move.
- (2026-09-16) `python tools/probes/caught_window_bag_probe.py
  ~/Desktop/rosbags/2026-09-16_14-16-38 ~/Desktop/rosbags/2026-09-15_18-51-37`
  → **09-16: n = 7, max +191.2 ms, median +112.0 ms; 09-15: n = 22, max
  −40.2 ms, median −44.5 ms** (~40 min wall for both bags; background it). It
  **agrees with the independent table on 28 of 29 rows to the millisecond** and
  disagrees on 09-16 throw 3 — see the ⚠ under the delay table, and the probe's
  own "Known limitation". The disagreement does not move `CAUGHT_WINDOW_S`,
  which the ruling set above the larger of the two readings.
- **Hardware:** NOT flown. The next sitting is
  `tests/hardware/session_skills_r3_apex_ladder.md`.

(2026-09-16, `./run_tests.sh --full`, log `temp/logs/park_outcome_full2_20260916.log`, the final tree of both same-day units plus the audit follow-ups): **PASS — parallel 6198 passed, 9 skipped, 2 xfailed in 295.88 s; serial 6 passed in 19.40 s.**

## Discussion

**The briefed hypothesis for defect 2 did not survive the log.** It was a good
hypothesis — the executor DOES have a retry-next-tick path for an unaimed catch,
and a deferral that probed the planner would look exactly like this in a log.
But `install_segment` and `plan_cycle` have separate log prefixes, and every one
of the 160 lines says `plan_cycle refused` while every `install_segment` line in
the sitting says accepted. Building the computed deferral anyway would have been
a correct-looking change to a path that never ran, and it would have left the
real caller hammering the planner. The general lesson is the cheap one: the
refusal's own service name was in the log line the whole time.

**Why refuse at the node and not in the planner.** The tempting fix is to make
`replan_tail` cheaper or to snap the re-aim's splice past the cone the way
`_snap_to_release` does for a segment. Both are wrong here, because the request
is not a tight fit that failed — the skill stack aims its own catches from the
landing the throw was COMMANDED to achieve (`AIM_SCHEDULE`, owner decision
2026-09-15, adopted precisely because the tracker produced no marker at all on
09-15). A tracker-driven re-aim of a skill-stack interval is a request from a
layer with no authority over it, and honouring it would drag the cup off the aim
the learner's own command is about to be scored against. Refusing it by
OWNERSHIP says that; refusing it by geometry would silently start accepting the
moment the geometry happened to fit.

**Why the verdict is a latch and not a longer-lived sample.** The obvious
alternative was to keep the point sample and just move it later. It cannot work
on a chain: by any instant late enough to cover the debounce, a catch-and-throw
has often already re-thrown the ball, so the cup at that instant says nothing
about whether the catch happened. The latch is also one-way for that reason —
the ball leaving again must not retract a catch that occurred.

**Accepted tradeoff.** A row now finalises up to 0.70 s after its scheduled
landing (`CAP + WINDOW`) instead of 0.15 s. The learner gets its feedback later
by up to 0.45 s. At a ~0.95 s beat that is under a beat, so a row still lands
before the next throw of the same ball is commanded, which is the only deadline
the memory actually has.
