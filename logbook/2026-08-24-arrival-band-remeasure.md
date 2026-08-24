---
title: The post-FW-14 arrival band collapsed, and the half everyone expected to pay was already bought
type: investigation
date: 2026-08-24
status: resolved
related_plan: toss-selftuning.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/ball_possession.py
  - ros_ws/src/jugglebot/jugglebot/toss_session.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/toss_record.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/reload_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/hand_stroke.py
  - config/hardware_config.yaml
  - tests/ros/test_ball_possession.py
  - tests/ros/test_toss_session.py
  - tests/motion/test_toss_record.py
  - tests/motion/test_hand_stroke.py
  - tools/probes/toss_record_miner.py
subsystem:
  - ros
  - sensor
tags:
  - cadence
  - possession
  - measurement
---

# The post-FW-14 arrival band collapsed, and the half everyone expected to pay was already bought

## Summary

`tests/hardware/session_cadence_ladder.md` § 3.1 — one of the two measurements
that gate rung R3 — asked for the empty→held sensor edge's offset from the
announced landing to be re-taken over ≥ 30 catches on the post-FW-14 plant. The
shipped band (`ARRIVAL_BAND_MIN_S = 0.137`, `ARRIVAL_BAND_MAX_S = 0.80`) was cut
from +137…+798 ms, median +399, n = 35 over three 2026-08-10 bags, while the
can-bridge uptime-dependent dispatch shift was +54…+133 ms. FW 14 cut that shift
to 10–20 ms on 2026-08-15.

**The band collapsed, measured offline over four post-FW-14 bags, n = 33 announced
catches: +87.6 … +554.7 ms, median +183.9.** The ceiling fell 243 ms and the
median 215 ms. Constants moved `0.137 → 0.087` and `0.80 → 0.56`, and four
derived consumers followed from those two names — with one exception the runbook
did not anticipate and one economic surprise that is the reason this entry has a
Discussion.

No new sitting was needed: the corpus already existed.

## Measurement

Every bag under `~/Desktop/rosbags/` from `2026-08-14_18-58-24` (the first
post-FW-14 capture) onward was mined; 20 were candidates, and only four carry
`/throw_announcements` with announced self-tosses.

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
python tools/probes/toss_record_miner.py --bag <bag> --sensor-only --jsonl
```

Per row, the runbook's exact recipe: `t_catch_raw_ros − announce_landing_time_ros`
— the RAW empty→held edge (zero debounce) against the announced landing, both on
the ROS clock.

| bag | n | min | p5 | median | p95 | max | bridge FW | uptime |
|---|---|---|---|---|---|---|---|---|
| 2026-08-18_18-42-19 | 11 | 0.132 | 0.137 | 0.177 | 0.301 | 0.373 | 14 | 95.8 h |
| 2026-08-20_21-51-39 | 5 | 0.138 | 0.139 | 0.173 | 0.273 | 0.274 | 15 | 47.3 h |
| 2026-08-21_10-11-42 | 3 | 0.183 | 0.193 | 0.284 | 0.528 | 0.555 | 15 | 59.7 h |
| 2026-08-23_19-14-54 | 14 | 0.088 | 0.101 | 0.189 | 0.239 | 0.266 | 15 | 116.7 h |
| **POOLED (CAUGHT)** | **33** | **0.0876** | **0.1168** | **0.1839** | **0.3195** | **0.5547** | 14 + 15 | 47–117 h |

Mean 0.2027, sd 0.0849. Every row all 33 offsets, in ms:

```
87.6 108.6 122.2 132.1 137.7 141.8 141.9 154.8 163.3 164.2 169.2 170.8 172.7
173.8 176.8 182.9 183.9 193.3 196.1 196.8 198.0 200.9 208.7 217.1 219.1 225.3
228.1 265.8 271.0 273.6 283.8 373.1 554.7
```

**Per-bag scatter is small where n is not.** The three bags with n ≥ 5 have
medians of 177 / 173 / 189 ms — a 16 ms spread across two bridge FW versions and
70 hours of uptime range. The n = 3 bag's 284 ms median is the only outlier and
is small-sample noise on the longest flights in the corpus. There is no
plant-pooling problem here; pooling is what makes the ceiling defensible.

### Eligibility, verified per bag rather than by date

`bridge_fw_version` is carried on every mined row (from `/link_status`), and all
four bags read FW 14 or 15 — post-fix by construction, not by filename. Of the 20
candidates: 12 have no announced self-toss at all (bench captures), three
2026-08-18 bags carry no `/hand_telemetry`, and `2026-08-15_00-42-12` is
**truncated** — it dies in the mcap CDR reader (`struct.error: unpack_from
requires a buffer of at least 88 bytes … actual 84`). It carries no announcements
either way, so nothing was lost to it.

### Physical plausibility, checked before touching a constant

The runbook's stop condition was an implausibly small minimum. The expected
composition is *ball-settle physics + 20 ms poll quantisation + 10–20 ms
dispatch*, and the corpus is consistent with it: median +184 ms is dominated by
the ball riding the catch stroke down to the seat before the beam breaks, and the
minimum +87.6 ms sits at the corpus's **shortest** flight (0.549 s), where the
stroke is shortest. Nothing arrived before its announced landing.

**The one sub-88 ms datum is not a catch.** A 34th row carries an edge at
**+45.4 ms** — `sensor_edge_count 4`, held for 122 ms, labelled BOUNCED: a rim
graze, not a seated arrival. It is reported here and in `ARRIVAL_BAND_MIN_S`'s own
comment so a future reader finds it already accounted for.

### The offset is flight-dependent, and the corpus says so

| flight s | n | min ms | median ms | max ms |
|---|---|---|---|---|
| 0.549 | 2 | 88 | 98 | 109 |
| 0.639 | 2 | 138 | 140 | 142 |
| 0.700 | 3 | 169 | 217 | 266 |
| 0.756 | 2 | 173 | 222 | 271 |
| 0.798 | 3 | 163 | 164 | 184 |
| 0.903 | 16 | 122 | 188 | 373 |
| 0.989 | 4 | 196 | 222 | 284 |
| 1.069 | 1 | 555 | 555 | 555 |

Pearson r = 0.49, slope ≈ 320 ms of offset per second of flight. **Both rows above
+300 ms sit at flights longer than any published ladder rung.** Inside the
ladder's own envelope (flight ≤ 0.798 s) the observed ceiling is +271 ms.

### Agreement with the report-only 2026-08-23 reading

`ARRIVAL_BAND_MAX_S`'s previous comment carried a report-only reading of the same
2026-08-23 bag: +46.5 … +267.5 ms, median +184.7, n = 15. This re-mine of that
bag gives +45.4 … +265.8, median +184.0, n = 15 — agreement to **1–2 ms**, which
is a different edge-timestamp anchoring, not a disagreement. That reading was
correct and, as its own text said, too thin to ship from.

## Discussion

### Why 0.56 and 0.087, and what the margins are for

The originals' sizing policy is stated in their own comments and was applied
unchanged: **the floor is the datum** (`+137 ms` shipped as `0.137`, deliberately
not rounded), **the ceiling is the datum ceiled to the next 10 ms** (`+798 ms`
shipped as `0.80`, "so the constant is a bound rather than a datum"). Applied to
this corpus that is `+87.6 ms → 0.087` (floored to the millisecond so it remains
a true lower bound) and `+554.7 ms → 0.56` (+5.3 ms).

That margin is thin in absolute terms and it is deliberate, for a reason that is
about failure modes rather than about matching the old policy:

* **Too small is the expensive direction.** `CATCH_CONFIRM_WINDOW_S` *is*
  `ARRIVAL_BAND_MAX_S`, and it is the FSM's terminal deadline. A real catch whose
  seat edge lands past it mints MISSED on a good catch — which with
  `stop_on_miss: true` ends the sitting, and with `on_empty_cup: RELOAD` asks
  BallButler to throw a second ball at a full cup. That is why the constant is a
  BOUND over the population and not a percentile.
* **Too large is not free either.** It keeps `DEFAULT_SESSION_MISS_CLEANUP_S`
  longer than the teardown needs on every missed cycle, and — the reason this
  re-measure was worth booking at all — it keeps C-POSSESS-1 § 3.4 clause C.2's
  amputation threshold high, which is what stands between the deferred R6 fork
  and a usable ARRIVAL verdict.
* **Inflating the bound past the datum would destroy the thing that makes it
  auditable.** A future reader can re-derive `0.56` from the corpus in one line.
  A padded `0.60` would leave them unable to tell datum from padding, which is
  exactly how a measured constant becomes a chosen one.

**What the margin does NOT cover, stated rather than implied.** The C-HAND-3
flight ceiling is 1.1485 s; the longest flight measured here is 1.069 s at n = 1;
the fitted slope would put ≈ +580 ms there. So **more samples at long flights is
the one thing that can move this number again** — not another sitting at the
ladder's own flights, where 0.56 already leaves > 2× headroom over the observed
+271 ms.

### Why the CAUGHT-only minimum, and not the corpus minimum

`ARRIVAL_BAND_MIN_S`'s only production consumer is
`toss_session.DEFAULT_SESSION_DWELL_MARGIN_S`, and the quantity it models is *the
earliest instant a possession verdict for cycle N can exist*. The +45.4 ms
rimshot did produce a verdict, so it is a fair datum for that sentence read
literally — but the handoff it sizes is the one from a CAUGHT cycle, and no catch
in 33 has seated before +87.6 ms. Sizing the margin on the graze would put it
42 ms below the earliest instant a CAUGHT verdict has ever existed: the
fail-**open** direction for the one thing the number models, in exchange for
nothing (see below — it is inert at every rung either way).

### The economic surprise: the floor re-measure bought nothing

This is the non-obvious tradeoff, and it inverts what § 3.1 expected.

`toss_session.handoff_margin_s` is
`max(dwell_margin_s, hand_stroke.catch_park_reentry_s(v, scale))`. The runbook
sized the expected saving against a park term of **0.1204 s** at the R0–R3 flight
— under the old 0.137 s arrival term — so it expected R0–R3 to gain and R4+ to
gain nothing. Measured against the **shipped property** rather than against that
figure, by calling `TossSessionSequencer.required_dwell_s` directly
(`/tmp/probe_arrival_band_rungs.py`, run 2026-08-24):

| rung | flight | park (ILC armed) | req_dwell before | after | park (aim disarmed) | req_dwell before | after |
|---|---|---|---|---|---|---|---|
| R0 | 0.7977 | 0.1416 | 5.1416 | 5.1416 | 0.1204 | 5.1370 | 5.1204 |
| R1 | 0.7977 | 0.1416 | 3.6416 | 3.6416 | 0.1204 | 3.6370 | 3.6204 |
| R2 | 0.7977 | 0.1416 | 2.5416 | 2.5416 | 0.1204 | 2.5370 | 2.5204 |
| R3 | 0.7977 | 0.1416 | 1.0416 | 1.0416 | 0.1204 | 1.0370 | 1.0204 |
| R4 | 0.6059 | 0.1861 | 0.6361 | 0.6361 | 0.1582 | 0.6082 | 0.6082 |
| R5 | 0.5029 | 0.2081 | 0.6781 | 0.6781 | 0.1903 | 0.6603 | 0.6603 |
| R5-prime | 0.5029 | 0.2081 | 0.6481 | 0.6481 | 0.1903 | 0.6303 | 0.6303 |

**On the binding column — a layer-3 speed trim possible, which is the column the
ladder publishes its floors against — not one rung moves.** `floor_event_vel_mps`
judges the floor at the slowest release the apply seam could command, which
lengthens the catch stroke, which lifts the park term to 0.1416 s at R0–R3. That
is already above the *old* 0.137 s arrival term, so `dwell_margin_s` had stopped
being the max()'s winner before this re-measure ran. On the aim-disarmed column
R0–R3 gain exactly **16.6 ms** each — 0.7 % of R3's 2.298 s period — and R4/R5
gain nothing.

So the ladder's own framing ("where the re-measure DOES pay is R0–R3") was right
about the direction and about a term that had already been swallowed. **The value
of this measurement is entirely in the ceiling.**

### What the ceiling actually bought

* `CATCH_CONFIRM_WINDOW_S` **0.80 → 0.56** in both `toss_sequencer` and
  `reload_sequencer` (derived; no edit of their own).
* `DEFAULT_SESSION_MISS_CLEANUP_S` **2.84 → 2.60 s** — 240 ms off every missed
  cycle the session continues past.
* **C.2's amputation threshold falls from a 0.800 s cycle period to 0.560 s**, and
  the boundary tracks the next landing again above `BAND_MAX + lead` = 0.760 s
  instead of 1.000 s. The deferred **R6 fork (period 0.7529 s) stops amputating
  entirely**: its window closed at +0.5529 against a +0.800 ceiling and now closes
  at +0.5600 against a +0.560 one, so `_band_watched_out` passes and ARRIVAL can
  answer at that cadence for the first time. The ladder's § 3.1 box predicted
  ≈ 0.27 s from the thin 2026-08-23 reading; the wider corpus says 0.56 s, which
  still clears R6 by 193 ms.
* One window gets *narrower*: the R5′ clamp pin (period 0.9849 s) now closes at
  +0.7849 instead of +0.8000, because `b − lead` dominates once the ceiling drops
  under `period − lead`. That is 230 ms above the new measured ceiling, so no band
  tail is lost — the window still outlasts the evidence, which is the invariant
  that matters.

### Reconciliation with the 2026-08-23 clamp change (98e9e7b)

`98e9e7b` replaced `b − arrival_lead_s` with
`arrival_boundary_t(a, b, lead) = max(b − lead, min(a + ARRIVAL_BAND_MAX_S, b))`
and added C.2. Its own commit message named this re-measure as the reason it
landed first: *"moving a constant is safe only once every consumer reads it the
same way."* That held. Re-checked against the new value:

* The clamp consumes `ARRIVAL_BAND_MAX_S` as the **earlier ball's claim on the
  boundary** and as C.2's **watched-out test**. Shrinking it therefore *relaxes*
  C.2 (fewer UNKNOWNs) and *shortens* the earlier ball's claim only in the period
  band `[BAND_MAX, BAND_MAX + lead)` — where the surviving `b − lead` is still far
  above the measured ceiling. Nothing in the new semantics fights the smaller
  number; the collapsed band is what that fix was built to absorb.
* The uncommitted `ros_ws/docs/ball_possession_contract.md` in the tree carries a
  2026-08-24 audit correction to the same clause's reachability table (R5′ pin
  loses nothing, R6 loses 47.1 ms, thresholds 1.000 s vs 0.800 s). Its *reasoning*
  survives the re-measure unchanged; its *numbers* are all functions of
  `ARRIVAL_BAND_MAX_S` and now need re-deriving at 0.56. That document is being
  edited by another session and was deliberately not touched here — it is the
  first follow-up.
* The C-POSSESS-1.C abutment work of 2026-08-21 (§ Phase H of
  `logbook/2026-08-21-ilc-primary-foldin.md`) re-derived
  `CATCH_CONFIRM_WINDOW_S` from `ARRIVAL_BAND_MAX_S`. Verified still derived, in
  both sequencers, by identity (`is`, not `==`) in
  `test_the_catch_confirm_deadline_clears_the_band_it_has_to_outlast`.

### The consumer that does NOT follow, and the one that is a literal copy

§ 3.1 says "update the two names and nothing else — every consumer follows". Two
of the four named consumers break that promise, in opposite ways.

1. **The interlude's seat-edge band wait does not follow at all.**
   `ReloadCoordinatorNode._wait_out_seat_edge_band` waits
   `hw.JB_BD_ARRIVAL_WINDOW_S` (**1.50 s**, a `hardware_config.yaml` knob) past
   the previous cycle's scheduled landing — not `ARRIVAL_BAND_MAX_S`. Its
   docstring claimed *"deriving the wait from it rather than choosing a number is
   what makes the pending post-FW14 band re-measure shrink this wait too"*, and
   that claim is **wrong about the mechanism**: a change to the Python constant
   cannot reach a YAML value. What the re-measure did was widen the margin the
   wait already carried, from 1.9× the old ceiling to **2.7×** the new one. The
   wait was left at 1.50 s deliberately — the same knob also sizes the live
   ARRIVAL search window and the reload budget, so shrinking it is a separate
   decision with its own blast radius — and the false sentence was replaced with
   what is actually true.

2. **The dwell margin IS a re-typed literal, in the YAML.**
   `config/hardware_config.yaml: toss_session_dwell_margin_s: 0.137` is the value
   the node passes in at runtime; `toss_session.DEFAULT_SESSION_DWELL_MARGIN_S` is
   only the no-config fallback. Updating the Python constant alone would have
   changed the fallback and left the running machine at 0.137. The duplication is
   safe rather than merely present because
   `test_local_constants_match_generated_config` pins the two equal — that test
   red is exactly how this was found — but the runbook's "nothing else" is wrong
   about this one consumer, and the YAML comment now says so.

No other production literal copy of either band value exists: every other
occurrence of `0.137` / `0.80` / `+798` in `ros_ws/src/` is provenance prose or an
unrelated 0.80 s *flight time*.

### Why the boundary tests were re-keyed instead of re-numbered

Nine tests across four files pinned scenarios at the R5′ clamp pin (0.9849 s) and
the R6 fork (0.7529 s), because those were the reachable cadences where C.1 and
C.2 bite at a 0.800 s ceiling. At 0.56 both rungs walk out of both clauses, so
every one of those premises failed. Typing a fresh pair of rung numbers would
have scheduled the identical rot for the next re-measure, so each clause's
scenario period is now **derived from its own arithmetic** —
`BAND_MAX + lead/2` for C.1, `BAND_MAX × 0.9` for C.2 — along with the in-sliver
arrival instants.

That makes both periods **synthetic**: at the collapsed ceiling C.1 needs a
0.157 s dwell and C.2 a ~0.001 s dwell against a 0.487 s C-HAND-1 hand floor.
This is not a weakening. Both clauses are invariants over the *window*, not over
the cadence — `_band_watched_out`'s own docstring says so — and an
`arrival_window_s` configured under the band still reaches C.2 from a direction
cadence no longer can. The honest headline is that **the collapsed ceiling moved
the C.1/C.2 defect classes out of any physically schedulable cadence.**

One test needed more than re-keying.
`test_no_accepted_session_starts_a_cycle_inside_the_live_catch_stroke` existed to
kill a mutation reducing `handoff_margin_s` to `dwell_margin_s`, and it did that
by sweeping flights across a crossover that sat mid-band. At 0.087 s the crossover
moved to ≈ 1.13 s of flight — past every published rung — leaving the arrival
branch alive only at the band's long-flight end. The sweep now varies the
**margin** as well as the flight (the shipped 0.087 s and a 0.30 s margin above
every park value in the band), so both branches of the `max()` stay live and a
collapse in **either** direction reds a row.

## Changes

* `ball_possession.py`: `ARRIVAL_BAND_MAX_S 0.80 → 0.56`,
  `ARRIVAL_BAND_MIN_S 0.137 → 0.087`, both with full (date, n, band) provenance,
  the rimshot datum, the flight-dependence caveat and the "inert at every rung"
  note. In-file prose that had become false arithmetic on the constants
  (`BAND_MAX + lead = 1.000 s`, "below a 0.800 s cycle period") re-derived.
* `config/hardware_config.yaml`: `toss_session_dwell_margin_s 0.137 → 0.087`,
  PROVISIONAL flag retired, the literal-copy hazard documented; regenerated via
  `python config/generate_config.py`.
* `toss_session.py`, `toss_sequencer.py`, `toss_record.py`,
  `reload_coordinator_node.py`, `reload_sequencer.py`, `hand_stroke.py`: prose
  that stated now-false numbers, and the two "pending post-FW14 re-measure"
  promises, brought to the landed state. The seat-edge wait's false derivation
  claim replaced.
* Tests re-keyed as described; the miner's own `self_check` band cases derived
  from `toss_record.ARRIVAL_BAND_MAX_S`.

## Verification

* Miner self-check (`python tools/probes/toss_record_miner.py --self-check`, run
  2026-08-24): **63/63 self-check cases pass**.
* Scoped, every test file that reads `ball_possession` or its consumers
  (`python -m pytest -q -p no:randomly tests/ros/test_ball_possession.py
  tests/ros/test_toss_session.py tests/ros/test_toss_sequencer.py
  tests/ros/test_reload_sequencer.py tests/ros/test_reload_coordinator_node.py
  tests/ros/test_toss_continuous_node.py tests/ros/test_toss_coordinator.py
  tests/ros/test_hand_sensor_replay.py tests/motion/test_toss_record.py
  tests/motion/test_hand_stroke.py tests/motion/test_cadence_rung_check.py
  tests/ros/test_toss_record_miner.py tests/motion/test_toss_cal_fit.py`, run
  2026-08-24): **917 passed in 132.79 s**.
* THE GATE (`./run_tests.sh`, run 2026-08-24): **PASS — 5758 passed, 3 skipped,
  parallel 262 s + serial 11 s, total 273 s** (the serial phase is empty by the
  2026-08-01 carve-out; `run_tests.sh` says so). Not committed — this change-set
  is handed off for review.

## Open items

* `ros_ws/docs/ball_possession_contract.md` § 3.4's reachability table and its
  1.000 s / 0.800 s thresholds are all functions of `ARRIVAL_BAND_MAX_S` and need
  re-deriving at 0.56. Left untouched because another session holds that file.
* `tests/hardware/session_cadence_ladder.md` § 3.1 needs its status flipped and
  the measured band recorded. Left untouched for the same reason; the exact
  replacement text is banked for the cross-recording pass.
* **This change needs `colcon build --packages-select jugglebot`** before it is
  live on the robot: the `jugglebot` package is installed, and the YAML-derived
  `JB_OP_TOSS_SESSION_DWELL_MARGIN_S` is read from the installed copy.
* Whether `JB_BD_ARRIVAL_WINDOW_S` (1.50 s, now 2.7× the ceiling) should be
  trimmed is now a live question rather than a promise made by a docstring. It is
  not free: the same knob sizes the live ARRIVAL search window and the reload
  interlude budget.
* More samples at flights above 1.0 s are the only thing that can move the
  ceiling again. The corpus has one.
