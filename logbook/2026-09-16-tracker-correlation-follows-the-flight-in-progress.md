---
title: "The correlation now follows the flight in progress — the host was reading the NEXT announced ball, not the tracker being a beat late"
type: bugfix
date: 2026-09-16
status: done
phase: "two-ball-skill-stack — R3"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/ball_possession.py (new flight-latch layer — FlightLatch, advance_flight_latches, flight_in_progress, RELEASE_LATCH_EPS_S, MAX_FLIGHT_LATCHES; latch_announced_ball itself unchanged)
  - ros_ws/src/jugglebot/jugglebot/skill_node.py (_correlation is now a per-RELEASE latch queue, not one latch per schedule ball; _maybe_announce queues instead of resetting; _advance_correlation / _tracker go through the new layer; new _now_s and _correlation_lock)
  - tools/probes/outcome_landing_replay.py (replays the CORRELATION too — read_bag/_Snap/per_id_streams/served_stream; the new side no longer hand-picks an id)
  - tests/ros/test_skill_node.py (4 correlation tests adapted to the queue; 4 new — flight in progress, the move to the next flight, an ended flight yields nothing, a second announcement does not reset)
  - tests/ros/test_ball_possession.py (6 new pure-layer tests on the measured bag instants; the grep-proof test now pins the flight-latch symbols)
---

## What / Why

`logbook/2026-09-16-outcome-landing-frozen-at-the-crossing.md` closed the
learner contamination but left an Open item: *"the tracker's in-flight landing
estimate ran ~one beat late on every chained throw but the last"* (ratios
2.21–2.99 of the command), with `catch_aim_source=tracker` unusable as a
result. That attribution was wrong, and the bag says so in one line.

The tracker was never late. **The host was reading the wrong tracker id** —
the id of the NEXT announced flight, correlated before it had been thrown.

Measured, bag `2026-09-16_16-22-22`, `armB-090` attempt 1 (`/throw_announcements`
and `/balls`, all instants +1789540000):

| announced | release | tracker id | goes IN_FLIGHT | its pre-computed landing |
|---|---|---|---|---|
| 416.861 | 417.361 | **48** | 417.372 | 418.218 |
| 417.262 | 418.511 | **49** | 418.514 | 419.368 |
| 418.439 | 419.661 | **50** | 419.680 | 420.518 |
| 419.576 | 420.836 | **51** | 420.844 | 421.693 |

A chained CATCH+THROW announces its carried release at DISPATCH, ~1.25 s
ahead, so announcement *n+1* lands while flight *n* is in the air. Two defects
then compounded, both in `skill_node`, neither in `tracking/`:

1. `_maybe_announce` **reset** `_correlation[ball_id]` at every announcement —
   one latch per schedule ball, and a self-toss re-throws the same schedule
   ball every beat. The latch for the flight in the air was simply discarded.
2. The fresh latch's `preexisting` set — "ids already IN_FLIGHT are phantoms" —
   **excludes exactly the airborne id**. So the only id the new latch could
   ever resolve to was the next flight's, and it resolved the instant that id
   went IN_FLIGHT (418.514 for id 49), while flight 48's outcome row was still
   open until 418.918.

Flight 48's row therefore observed id 49's landing: y = 419.59 − 417.361 =
**2.2317 s** against an 0.8569 s command. Replayed over both sittings, the
**21 contaminated rows are exactly the 21 rows whose old pick read
`served_id + 1`** — the next announced ball — and no others (table below).

## Discussion

### Why the correlation layer, and not the matcher or the announcement

`tracking/matcher.py` is correct as written and was checked directly:
`handle_announcement` minting a new id per announcement is the right model (one
id per flight), and `_check_throw_times` flips TO_BE_THROWN → IN_FLIGHT only at
`current_time >= ball.throw_time`, both sides on the ROS clock
(`ball_tracker_node` passes `get_clock().now()`, the announcement carries
`throw_time` on the same clock). The bag confirms it: id 49 flips at 418.514
for a 418.511 release — a 3 ms skew, not a beat. So:

* **Queueing the announcement inside the matcher** (publish it as a distinct
  TO_BE_THROWN id only once its throw_time arrives) was rejected: the
  announcement's whole purpose *before* the release is to seed the 200 mm
  expected-position gate that confirms the ball
  (`logbook/2026-09-15-tracker-all-markers-gated-to-expected-ball.md`). Delaying
  the id delays confirmation of the very flight we are trying to follow.
* **Announcing the carried throw later** was rejected: the lead is what the
  tracker needs, it is a wire/timing change to the one path R3 flew clean, and
  it would leave the host's rule still wrong for any two-ball schedule.
* The defect is in the host's translation from schedule ball id to tracker id,
  so that is where the fix goes — no wire change, no firmware, no timing.

### The rule, one level up

The class is *"the correlation answered a question about a different flight"*,
and it has three members: the next flight (a reset latch re-latching early),
the previous flight (an earlier latch used as a fallback after the current one
ends), and an unreleased flight (a ball reported IN_FLIGHT by some other
path). One invariant closes all three: **correlate per RELEASE, never per
ball, and never read a release that has not happened.**

`ball_possession` gains the enforcement point (`FlightLatch`,
`advance_flight_latches`, `flight_in_progress`) next to the rule it already
owned; `latch_announced_ball` is untouched and still the per-announcement rule,
so the FSM path is unaffected (91 passed, below). The flight in progress is the
latch with the **latest release that has passed**, and only if its id is
IN_FLIGHT **and** CONFIRMED. There is deliberately **no fallback to an earlier
latch**: a release only happens once the previous flight of that schedule ball
is over, so an earlier latch's landing describes a finished flight — the
contamination itself. "No landing yet" is the honest answer.

Two details that are load-bearing rather than incidental:

* **Sibling exclusion.** At 419.680 both id 49 (caught, marker still visible,
  so still IN_FLIGHT) and id 50 (just released) are destination-tagged and
  IN_FLIGHT, and `latch_announced_ball` prefers the *first* tagged candidate it
  sees — which is 49. Each latch therefore excludes the ids its siblings have
  claimed, or release *n+1* would latch flight *n*'s id (a differently-shaped
  instance of the same class). Tested on those instants.
* **`RELEASE_LATCH_EPS_S` = 50 ms.** The tracker and the node read the same
  announced `throw_time` on two clock reads (3 ms apart, measured), so a latch
  must not refuse its own id because the reads straddle the instant. 50 ms is
  ~4 % of the shortest beat this stack schedules (~1.15 s), so it can never
  admit the next flight early.

`preexisting` keeps its old job and gains a second one: now that the latch is
not reset, it is what stops a new release from stealing the airborne id.

### What this changes about the 2026-09-16 freeze and band

Nothing is withdrawn. The freeze at the crossing, the next-release bound and
`FLIGHT_RATIO_BAND` all stay: they are the executor's defence against an
estimate that is not about this flight, and the correlation is the node's. The
band in particular still earns its place — a **held** ball's landing is
predicted at ~now whatever id you read it from, and `armB-090` att 1 throw 3
replays at y/u = 1.33 for exactly that reason (its successor wrote no row, so
this probe cannot bound its window; live, the schedule-derived
`_next_release` does). What changes is that the band is no longer doing the
work of the correlation: before the fix it had to reject 21 of 43 rows; after
it, all 43 are inside it.

### What was ruled out at the read

**Returning a CAUGHT ball's frozen landing** (the pre-fix `_tracker` checked
only CONFIRMED, never the status). Dropped: a ball in the cup with its marker
visible has its landing predicted at ~now, and after the marker is lost the
value is a dead-reckoned freeze. Neither is an observation of a flight, and
the catch aim has no use for either.

## Fix

1. **`ball_possession`** — `FlightLatch(t_release_s, announced_id, untagged,
   preexisting)`, one per announced release;
   `advance_flight_latches(...)` resolves every *released* latch in release
   order with sibling exclusion, leaves a future one untouched, and keeps the
   newest `MAX_FLIGHT_LATCHES` (4 — two flights can overlap at most);
   `flight_in_progress(...)` returns the id of the latest released latch when
   it is IN_FLIGHT and CONFIRMED, else `None`.
2. **`skill_node`** — `_correlation[ball_id]` is now a tuple of latches;
   `_maybe_announce` **appends** one carrying the announcement's own ROS-clock
   `throw_time` (the same instant the tracker flips the ball) instead of
   resetting; `_advance_correlation` and `_tracker` go through the new layer;
   `_now_s()` is the single ROS-epoch clock read both use. A new
   `_correlation_lock` guards the read-modify-write: `_on_balls` (subscription
   group) and `_maybe_announce` (tick group) both rebind the queue, and a lost
   append would leave the next flight uncorrelated for its whole flight.
   `_tracker` only reads the tuple and takes no lock.
3. **`tools/probes/outcome_landing_replay.py`** — the new side now replays the
   correlation as well (the bag's own `/throw_announcements` → latches →
   `flight_in_progress` → the served estimate stream), so it can no longer
   flatter or convict the executor with a hand-picked id. That hand-pick is
   why its first run blamed the tracker.

### Replay — before and after

`python tools/probes/outcome_landing_replay.py --bag
~/Desktop/rosbags/2026-09-16_16-22-22 --bag ~/Desktop/rosbags/2026-09-16_14-16-38
--learn temp/learn/_quarantine_20260916 --date 20260916` (the 20260916 memories
are quarantined, hence the `--learn` path). Full table:
`temp/probes/outcome_landing_replay_20260916_correlated.txt`.

| | as written | freeze+band only (2026-09-16 am) | freeze+band+correlation |
|---|---|---|---|
| rows admitted | 43/43 | **22**/43 | **43**/43 |
| y/u span | 0.979 .. 2.985 | 0.974 .. 1.528 | **0.829 .. 1.530** |

The 21 rows the morning's rule had to drop are precisely the 21 whose old pick
read the next announced id (`served_id + 1`); under the corrected correlation
they read 0.83 .. 1.14 of the command. Extract:

| plant | att | thr | commanded | old id → y | in-progress id → y |
|---|---|---|---|---|---|
| armB-090 | 1 | 1 | 0.8569 | 49 → 2.2317 | **48 → 1.1423** |
| armB-090 | 1 | 2 | 0.8569 | 50 → 2.2310 | **49 → 1.0020** |
| armA-090 | 1 | 1 | 0.8569 | 23 → 2.0538 | **22 → 0.9357** |
| armA-050 | 2 | 1 | 0.5988 | 2 → 1.5250 | **1 → 0.8434** |
| armB-070 | 1 | 2 | 0.7557 | 42 → 1.9621 | **41 → 0.6328** |

## Verification

* Scoped suite (2026-09-16, `pytest tests/ros/test_skill_node.py
  tests/ros/test_ball_possession.py tests/ros/test_ball_tracker_gate.py
  ros_ws/src/jugglebot/jugglebot/tracking/tests
  tests/motion/test_skills_executor.py tests/sim/test_skills_gate.py
  tests/ros/test_skills_plan_bench.py -q`): **476 passed, 1 xfailed, 1 failed
  in 41.11 s** — the one failure is
  `test_start_self_toss_is_refused_with_no_status_received_yet`, which is NOT
  from this change and belongs to a parallel session's uncommitted
  `config/hardware_config.yaml` edit: the generated launch defaults are now
  `JB_TRAJ_LEG_{VEL,ACC,JERK}` = 300 / 5000 / 150000 (verified 2026-09-16 by
  importing `config/generated/hardware_config.py`), i.e. exactly the swept
  admissible box, so the box no longer refuses at the YAML defaults and the
  later `trajectory/commanded_position is stale` refusal fires first. That
  refusal ladder is untouched by this change.
* FSM latch path unaffected (2026-09-16, `pytest
  tests/ros/test_reload_coordinator_node.py tests/ros/test_reload_integration.py
  -q`): **91 passed, 3 skipped in 3.61 s**.
* Replay (2026-09-16, command above): **43 rows, 43 admitted, y/u 0.829 ..
  1.530**, against 22 admitted before the correlation was modelled.

(2026-09-16, `./run_tests.sh --full`, log `temp/logs/ladder_close_full3_20260916.log`, the final tree of this phase — ladder close-out, new launch defaults, per-release correlation, lateral-authority guard, audit follow-ups): **PASS — parallel 6233 passed, 9 skipped, 2 xfailed in 297.12 s; serial 6 passed in 18.98 s.**

## Open

* **`catch_aim_source=tracker` has no remaining known defect** — the reason the
  live default stayed `schedule` was this anomaly. It has still never been
  flown, so the default is unchanged here; flying it is an owner decision and
  wants its own ladder row.
* The **held-ball ratio** (a caught ball's landing predicted at ~now) is still
  the one contaminant the band alone cannot separate at a short beat. The
  executor's schedule-derived next-release bound is what closes it live; the
  replay cannot see it for the last throw of an attempt.
* `armB-090` att 1 throw 3 replays at y/u = 1.33 for that reason. If a future
  sitting's last-throw rows cluster near the band's upper edge, suspect this
  before the plant.
