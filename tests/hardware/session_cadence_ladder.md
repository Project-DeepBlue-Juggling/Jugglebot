# Hardware Session — the CADENCE LADDER (R0 → R5-prime)

**Plan**: `plans/active/toss-selftuning.md` § 11 (the timing census) and
`plans/active/critical-point-ilc.md` (operator decision 3, 2026-08-21)
**Action**: `jugglebot/toss_continuous` (`TossContinuous.action`)
**Depends on**: `session_anomaly_fixes.md` § SECTION CONT green at the shipped
6.0 s dwell. This file starts from that baseline and walks the dwell DOWN.
**Goal**: reach the tuning-phase operating point — **dwell 0.66 s at a 0.31 m
throw (flight 0.5029 s), cycle period 1.163 s, 51.6 throws/min — with or without
an armed aim** — one bench sitting per rung, with a measurable gate and a stated
ABORT at each.

> ### ⚠ THE OPERATING POINT MOVED TWICE. Read this before booking a sitting.
>
> **2026-08-22.** This file published **dwell 0.49 s / delay 0.35 s / 60.4
> throws/min**, and *that goal never threw a ball* — nor did R4 or R5 as they
> were published. Every rung's `throw_delay_s` was checked against the **kind-0
> dispatch budget** (`hand_stroke.min_throw_event_delay_s`, 0.28–0.34 s), which
> is the right floor for the dispatch and is not the floor the machine enforces:
> `toss_sequencer._step_preparing` re-checks that same budget against the lead
> REMAINING after CHECKING, POSITIONING and the PREPARE ladder have elapsed. The
> old numbers cleared the first floor and died on the second, every cycle, at
> `cycle_start + 0.06 s`, with the hand retracting under a seated ball. The file
> then republished every rung twice — a LEVEL pair and an AIMED pair 0.38 s
> apart — and reported a frontier of 54.3 / 40.4 throws/min.
>
> **2026-08-23 — the split is GONE and the aimed number is 34 % better.** Both
> items that gap named are now closed:
>
> 1. `trajectory/commanded_pose` publishes the commanded ORIENTATION alongside
>    the position, so `_toss_already_positioned` can verify a pre-tilt pose and
>    the census-B1 skip fires on an aimed chain. That was the entire LEVEL/AIMED
>    gap. **One pair per rung again**, and the frontier is **54.3 throws/min
>    whether an aim is armed or not** (was 40.4 aimed).
> 2. The accept-time delay floor now models the whole pre-dispatch sequence
>    (`toss_sequencer.min_throw_delay_for_release_s`, imported by BOTH gates).
>    It buys no cadence — it buys an honest refusal: a goal these gates accept
>    **cannot** die `ABORTED_CANT_MAKE_RELEASE`, and the probe asserts that over
>    the whole `(T, dwell, delay, aim)` grid.
>
> The operator's decision of 2026-08-21 ("dwell 0.49 s, ~61 throws/min") was
> taken on the pre-audit numbers. **It cannot be honoured as stated** — 61
> throws/min is not reachable on this build — and the corrected ladder below is
> what the operator ACCEPTED on 2026-08-23 in its place.

The C-HAND-3 band FLOOR is flight 0.4949 s, and the goal parameters here sit a
deliberate 8 ms of flight time above it, which buys clearance from a cliff — see
the warning under the ladder table in § 2.

> **⚡ THIS SESSION FIRES REAL THROWS, FASTER EACH RUNG.** By R4 the machine
> throws and catches roughly once a second and does not stop between cycles; by
> R5-prime the hand is in motion for all but ~0.17 s of every cycle. On any miss the ball
> lands on the floor. Clear the area, keep clear of the hand's stroke path and the landing
> zone, and keep the E-STOP in reach the entire time.
>
> **Read § "A cancel is always deferred" before the first rung.** Your stop
> button behaves differently at these cadences than it does at 6.0 s.

## Roles & safety framing

- **The operator (Harrison) runs every robot-actuating command below.** The
  implementing session prepared these commands and the PASS/ABORT criteria, and
  verifies read-only.
- **If your physical intuition disagrees with any framing here, that is
  load-bearing signal — say so before proceeding.** Cadence work compresses the
  window in which a wrong framing is cheap.
- **REBOOT the can-bridge Teensy before every sitting** (standing rule, see
  `memory/project_reload_action_catch_latch.md`). Log `uptime_ms` with every
  measurement in this file — the uptime-lag arc is closed by FW 14, and logging
  it is how a regression stays visible.
- One rung per sitting. Do not skip a rung because the previous one was clean:
  each rung's gate is the evidence the NEXT one's assumptions rest on.

---

## 0. What the ladder is walking down, and what it cannot reach

The turnaround floor is **hand-stroke geometry**, not policy. Between a landing
and the next release the hand must, in series and without overlap (C-HAND-1):

    (t_vel_catch + t_dec_catch)(v_armed)   ball contact -> rest at 0 rev
  + SMOOTH_MOVE_MIN_DURATION_S  (0.050)    the next command's prelude floor
  + SAFETY_GAP_S                (0.020)    the Teensy :642 budget gap
  + (t_acc + t_vel)(v_throw)               the next throw's windup

`hand_stroke.min_turnaround_dwell_s` is that expression, and
`toss_session.required_dwell_s` refuses any dwell below it. Evaluated over the
whole C-HAND-3 admitted band (probe against the tree, 2026-08-22; `catch/vel_scale`
at its 0.9 default):

| flight `T` | apex | v_release | catch tail | throw windup | **hand floor** | cycle period | throws/min |
|---|---|---|---|---|---|---|---|
| **0.4949** (band floor) | 0.304 m | 2.4400 | 0.2702 | 0.1469 | **0.4871** | **0.982** | **61.1** |
| 0.60 | 0.445 | 2.9530 | 0.2233 | 0.1213 | 0.4146 | 1.015 | 59.1 |
| 0.80 (nominal) | 0.788 | 3.9308 | 0.1677 | 0.0912 | 0.3289 | 1.129 | 53.1 |
| 1.00 | 1.229 | 4.9097 | 0.1343 | 0.0730 | 0.2773 | 1.277 | 47.0 |
| 1.1485 (band ceiling) | 1.620 | 5.6368 | 0.1170 | 0.0636 | **0.2505** | 1.399 | 42.9 |

> **⚠ The last two columns are HYPOTHETICAL, not achievable.** They are what the
> cadence would be if the hand floor were the only floor — which it was until the
> pre-dispatch sequence was measured on 2026-08-22, and is not any more. The
> achievable number is in § 2.0's frontier table (**54.3 throws/min** at best,
> aim armed or not, against the 61.1 this row implies). The hand-geometry columns
> to their left are correct and still worth having: they are what a firmware
> change to `calcCatch` would move, and nothing else in this file is.

**Three consequences to state plainly before the first sitting:**

1. **A 0.25 s dwell is unreachable at EVERY admitted flight time.** The minimum
   over the band is 0.2505 s, at the very ceiling (apex 1.62 m), and it rises
   from there. The true 0.25 s dwell is rung R6 and it is a **DEFERRED FIRMWARE
   FORK** — it needs a Platform Teensy flash changing `calcCatch`'s geometry.
   **It is not being built.** Operator decision 3, 2026-08-21.
2. **The dwell is the wrong operator variable.** Cycle *period* is nearly flat
   across the band and bottoms at the FLOOR of the flight band, whichever floor
   is binding. Ask for a throws/min number, not a dwell — and get there by making
   the FLIGHT short, not the dwell. (The achievable bottom is 1.105 s, § 2.0, not
   the 0.982 s this table's hypothetical column shows.)
3. **The hand floor is no longer what binds ANYWHERE — the PRE-DISPATCH
   SEQUENCE is.** The table above is the hand's own geometry and it is correct,
   but `required_dwell_s` is `max(throw_delay + handoff_margin, hand_floor)`, and
   since the delay floor grew the sequence cost the first term wins **at every
   admitted flight time, by at least 0.12 s**: the delay must clear the dispatch
   budget **plus** 0.080 s of chained sequence, and the handoff margin then adds
   the hand's park re-entry on top. At the target flight that makes the smallest
   legal dwell **0.605 s** (0.636 s with an ILC artifact loaded), against a
   0.487 s hand floor. The hand is no longer the limit; the software sequence is.
   `test_the_hand_floor_is_dominated_by_the_plumbing_term` pins that dominance,
   so if it ever inverts again the suite says so. See § 2.0.

### The firmware question the census left open: `t7`, not `t8` — VERIFIED

The census flagged one item to verify before trusting the table above: whether
the C-HAND-1 no-overlap floor is written against `t7` (hand at rest at 0 rev) or
`t8 = t7 + END_PROFILE_HOLD` (0.10 s). **If `t8`, every number above rises by
100 ms**. That no longer moves the operating point — since 2026-08-22 the hand
floor is not what binds (§ 0 consequence 3) — and it would not move it even then:
the hand floor at the band floor would rise from 0.4871 s to 0.5871 s against a
plumbing term of 0.6101 s, so the plumbing would still bind and R5-prime's 0.66 s
dwell would still be legal (its own hand floor would be 0.5805 s).

**What it WOULD do is eat the dominance argument.** The plumbing term's worst-case
margin over the hand floor drops from **0.1230 s to 0.0230 s** (both at
`T = 0.4949`, probe 2026-08-23) — and that margin is what makes it safe to leave
`hand_floor_dwell_s` on the untrimmed release speed while the two floors either
side of it moved to the fail-closed one (0.0715 s of worst-case trim sensitivity
would no longer fit inside it). `test_the_hand_floor_is_dominated_by_the_plumbing_term`
asserts the 0.12 s margin, so a `t8` firmware change reds the suite rather than
silently inverting an argument. That is the whole reason the six line references
below are re-verified after any `Trajectory.h` flash.

**Verified in the shipped `Teensy_code_platform/Trajectory.h`, 2026-08-22: it is
`t7`.** No adjustment. The evidence:

- `buildCatch()` emits `tA[4] = {t4, t5, t6, t7}` (`:266`), and `buildSegment`
  terminates on `end = tA[3] - start` (`:230`) — so the kind-1 catch profile's
  last sample is at `t7`, with no end hold;
- `buildThrow()` emits `tA[4] = {0, t1, t2, t3}` (`:250`) — likewise;
- `t8` is assigned once (`:210`) and read in exactly ONE place: `buildCommand()`'s
  `tA[9]` and its `while (t < t8)` loop (`:291`, `:304`). `buildCommand` is the
  kind-2 `makeFull` builder, and its own comment records that **no live host
  dispatches kind 2**.

If a future firmware change gives kind-0 or kind-1 an end hold, **this table is
wrong by 100 ms and the ladder stops at R4.** Re-verify these six line
references after any `Trajectory.h` flash.

---

## 1. Standing session settings for every rung

```bash
# Every rung, without exception, until the sensor package is bench-validated:
#   stop_on_miss : true      one miss ends the sitting
#   on_empty_cup : STOP      never RELOAD
#   num_throws   : 5         first run of any rung
```

**`stop_on_miss: true` and `on_empty_cup: STOP` are NOT defaults you may relax
for convenience.** The census's single most dangerous change is lowering the
dwell *without* the possession work, because that combination leaves a fail-open
`ball_seated` gate, labels every good cycle `BOUNCED`, and routes good cycles
into the auto-reload interlude — **which asks BallButler to throw a second ball
at a cup that already holds one.** The possession work (census D1/D3/D4/D6)
landed 2026-08-21 and is desk-verified, but it has **not been validated on the
bench**. Until a rung has confirmed it on real hardware, `RELOAD` stays off.

Record for every rung: `uptime_ms`, `iq_brake_min_a`, the per-cycle
`dwell_s`, and the toss-record JSONL (`temp/logs/toss_records_<session>.jsonl`).
Score the **miner**, not the console — see § 4.

---

## 2. The ladder

Each rung is a bench sitting. Run `num_throws = 5` first; only repeat at a
higher count once the rung's gate is met.

### 2.0 What the rungs are checked against, and what condemned the old ones

**Every rung below is driven through BOTH real FSMs, to the tick, until the
cycle either dispatches a throw or dies.** The check is a committed probe:

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
python tools/probes/cadence_rung_check.py            # every rung, four ways
python tools/probes/cadence_rung_check.py --solve    # the per-rung floors
python tools/probes/cadence_rung_check.py --frontier # the reachable frontier
python tools/probes/cadence_rung_check.py --grid     # accept-implies-flies
```

It constructs the `TossSessionSequencer` and asserts the accept gate passes,
then constructs the cycle `TossSequencer`, ticks it at the node's real
`_TICK_S` (0.02 s), feeds it the fastest node behaviour that is legal, and
asserts it reaches **`ACTION_DISPATCH_THROW`** — on the CHAINED cycle and on the
FIRST cycle of the sitting, with an ILC artifact loaded and without, i.e. four
ways per rung. `--grid` additionally sweeps the whole `(T, dwell, delay, aim)`
space for **accept-implies-flies** violations. Re-run it after any change to
`toss_session`, `toss_sequencer`, `hand_stroke` or the coordinator's tick.

**It is also a test now** — `tests/motion/test_cadence_rung_check.py` imports the
probe and reds the suite if any published rung stops flying or the grid finds a
violation, so this table cannot drift away from the machine again without the
default gate saying so.

> **Why that last word matters.** Until 2026-08-22 this section claimed every
> rung "has been checked against the real FSMs", and the check stopped at
> *"CHECKING reaches POSITIONING"*. `ABORTED_CANT_MAKE_RELEASE` is minted in
> `_step_preparing`, **after** POSITIONING — so the check could not see the gate
> that refuses a fast rung, and three rungs shipped that abort every cycle.
> Run against the pre-audit table, the probe reports:
>
> | rung | session | LEVEL chain | AIMED chain |
> |---|---|---|---|
> | R0–R3 | ACCEPT | FLIES | FLIES |
> | R4 (dwell 0.75, delay 0.55) | ACCEPT | FLIES | **ABORTED_CANT_MAKE_RELEASE** |
> | R5 (dwell 0.60, delay 0.40) | ACCEPT | **ABORTED_CANT_MAKE_RELEASE** | **ABORTED_CANT_MAKE_RELEASE** |
> | R5-prime (dwell 0.49, delay 0.35) | **REJECTED_DWELL** | **ABORTED_CANT_MAKE_RELEASE** | **ABORTED_CANT_MAKE_RELEASE** |
>
> The probe keeps that table as `LADDER_PRE_AUDIT`, deliberately: a probe that
> can no longer reproduce the finding has lost it.
>
> **Against the 2026-08-23 tree those same rungs still fail — and they fail
> EARLIER.** The accept-time floor now models the whole sequence, so R5 and
> R5-prime are `REJECTED_THROW_DELAY` at the session gate and every remaining
> failure is a `REJECTED_CANT_MAKE_LEAD` at CHECKING: refused before anything is
> armed, with the reject message naming the dispatch budget, the sequence cost
> and which of the two sequences it charged. Not one of them reaches
> `ABORTED_CANT_MAKE_RELEASE` any more. That is the fix, stated as a diff rather
> than as a claim.

**The LEVEL / AIMED split published here on 2026-08-22 is GONE, and it was worth
0.38 s of throw delay per cycle.** It existed because the census-B1 positioning
skip (`_toss_already_positioned`) returned `False` on **every tilted release**,
by construction: `trajectory/commanded_position` carried position and nothing
else, so the node could not verify the platform was already at a pre-tilt pose
and had to command the move. An armed aim — ILC layer 3, the calibration map, or
Tier 8b displacement — makes every release tilted, so every ILC sitting paid
`min_move_duration_s` (0.20) + `TOSS_POSITION_SETTLE_PAD_S` (0.20) + ticks to
traverse zero millimetres and re-command a tilt it was already holding.

`trajectory/commanded_pose` now carries the same sampled plan state WITH its
orientation, in the INTENT frame (`trajectory_node._intent_orientation` inverts
the C-LEVEL-1 correction before publishing, so a consumer compares against what
it is about to request rather than against a gravity-corrected version of it).
The check got STRICTER, not looser — a tilted release must now PROVE the platform
holds its tilt, where before it could not be asked — and the discriminator became
per-CYCLE:

- **the FIRST cycle of a sitting commands the move** (the platform is not yet at
  the pre-tilt pose). The node grants that one cycle the extra lead and logs one
  WARN line naming the raise; it does not kill the sitting;
- **every chained cycle takes the skip**, because a CAUGHT toss ends in
  `ACTION_STAY` holding the pose it threw and caught from.

**The reachable frontier** (`--frontier`, 2026-08-23; monotone across the whole
C-HAND-3 band, fastest at the floor):

| aim | fastest period | throws/min | at `T` | `throw_delay_s` | `dwell_time_s` |
|---|---|---|---|---|---|
| DISARMED | 1.105 s | **54.3** | 0.4949 | 0.4168 | 0.6101 |
| ILC artifact LOADED | 1.105 s | **54.3** | 0.4949 | 0.4168 | 0.6101 |

The two meet at the band floor because the throw envelope refuses a negative
speed trim there, so a loaded artifact costs nothing at the fastest point. Away
from the floor it costs ~1.5 throws/min: the session judges every derived floor
at the **slowest release the ILC's apply seam could command**, because every one
of those floors rises as the release slows and a negative trim used to raise the
cycle's floor *after* the session had accepted the goal.

Those are knife edges — zero clearance, and the PREPARE bundle's three
synchronous service round-trips are charged as free. The published rungs below
sit back from them.

### 2.1 The rungs

`throw_delay_s` matters as much as `dwell_time_s` (the floor is
`max(throw_delay + handoff_margin(T), hand_floor(T))`, and the DEFAULT 5.0 s
delay makes even R1 illegal), and `throw_height_m` has a **cliff** — see the
warning under the table.

**ONE pair per rung again** (2026-08-23). Every pair below is legal with a
layer-3 artifact LOADED as well as without it — the stricter of the two cases,
chosen because this runbook exists to arm ILC-primary and a two-column table is
a way to pick the wrong column at 2 am.

| rung | `throw_height_m` | ⇒ flight `T` | `dwell_time_s` | `throw_delay_s` | period | throws/min |
|---|---|---|---|---|---|---|
| R0 | 0.78 | 0.7977 | 5.60 | 5.00 | 6.398 | 9.4 |
| R1 | 0.78 | 0.7977 | 4.10 | 3.50 | 4.898 | 12.3 |
| R2 | 0.78 | 0.7977 | 3.00 | 2.40 | 3.798 | 15.8 |
| R3 | 0.78 | 0.7977 | 1.50 | 0.90 | 2.298 | 26.1 |
| R4 | 0.45 | 0.6059 | 0.65 | 0.45 | 1.256 | 47.8 |
| R5 | 0.31 | 0.5029 | 0.70 | 0.47 | 1.203 | 49.9 |
| **R5-prime** | **0.31** | **0.5029** | **0.66** | **0.44** | **1.163** | **51.6** |

Every cell above is `PUBLISHED LADDER: all rungs FLY` from
`tools/probes/cadence_rung_check.py`, run 2026-08-23 — four ways per rung
(session accept; chained cycle; first cycle; each with the ILC trim possible and
not) — and `tests/motion/test_cadence_rung_check.py` reds the suite if that stops
being true.

**One variable changes per rung.** R0→R3 walk the DWELL down at a fixed 0.78 m
throw; R4 and R5 shorten the FLIGHT (which is where the cadence actually comes
from — see § 0 consequence 2); R5-prime tightens the dwell at the R5 flight.

**R5 is SLOWER in dwell than R4 and still faster in cadence, and that is real.**
R4's dwell is 0.65 s and R5's is 0.70 s — the dwell goes *up* between them,
because the kind-0 dispatch budget rises as the flight shortens (0.310 s at
`T = 0.6059`, 0.334 s at `T = 0.5029`) and the delay carries it. The cadence
still improves (1.256 → 1.203 s period) because the flight shrinks faster than
the dwell grows. This is § 0 consequence 2 arriving with teeth: **period is the
operator variable, dwell is not.**

**What each rung clears its floors by** (probe, 2026-08-23; the ILC-loaded floors,
which are the binding ones):

| rung | delay floor | clearance | dwell floor | clearance |
|---|---|---|---|---|
| R4 | 0.4113 | 38.7 ms | 0.6361 | 13.9 ms |
| R5 | 0.4280 | 42.0 ms | 0.6781 | 21.9 ms |
| R5-prime | 0.4280 | 12.0 ms | 0.6481 | 11.9 ms |

> **⚠ `throw_height_m: 0.31`, NOT 0.30 — there is a cliff at 0.3005 m.** The
> census quotes the operating point's apex as "0.30 m", which is a rounded
> DISPLAY of the band floor and not a goal parameter. A goal of `0.30` converts
> to `T = 0.49472 s`, which is **0.17 ms below** `throw_envelope.MIN_FLIGHT_TIME_S`
> (0.494882) — so the cycle dies `REJECTED_THROW_ENVELOPE(ARM_WINDOW: …)` at
> CHECKING, having built the whole per-goal state first. Verified by probe,
> 2026-08-22: `0.300` refuses, `0.301` admits.
>
> `0.31` is used here instead of `0.301` deliberately: at `0.301` the goal sits
> 0.7 ms above a cliff whose position depends on `catch/vel_scale` and on the
> C-HAND-2 headroom reservation, so any operator adjustment to either would
> silently start refusing the rung. `0.31` costs **0.35 throws/min** (52.96 vs
> 53.31 level; 39.66 vs 39.85 aimed — probe, 2026-08-22) and buys 8 ms of
> clearance. If a sitting needs the last third of a throw per minute, take it
> from the DWELL — but read § 2.0 first: the floor at this flight is 0.620 s
> level / 1.000 s aimed, **not** the 0.487 s hand floor, and R5-prime already
> sits only 9.7 ms above it.

### R0 — baseline (no change)

| | |
|---|---|
| **goal** | `throw_height_m: 0.78`, `dwell_time_s: 5.60`, `throw_delay_s: 5.0` |
| **must have landed** | nothing — shipped defaults |
| **measurable gate** | 5/5 CAUGHT; `peak ≤ 10.39 rev`; `dip_below_x3 ≤ 0.10 rev`; `trunc = −`; `seeds = 0` |
| **watch for** | reference plant health. Record `uptime_ms` and `iq_brake_min_a` — every later rung is scored against this sitting |
| **PASS** | all five gate rows met |
| **ABORT** | any HAND row outside `session_anomaly_fixes.md` § PASS/ABORT. Do **not** proceed to R1 on a degraded plant — every rung below inherits it |

### R1 — the free rung (no code change)

| | |
|---|---|
| **goal** | `throw_height_m: 0.78`, `dwell_time_s: 4.10`, **`throw_delay_s: 3.5`** |
| **must have landed** | nothing. But **the delay is NOT optional**: leave it at the 5.0 default and the floor is 5.137 s, so a 4.10 s dwell is `REJECTED_DWELL`. The census's "no code needed" is about the CODE, not about the goal |
| **measurable gate** | 5/5 CAUGHT; per-cycle `dwell_s ∈ [4.10, 4.4]`; **`dwell_tilt_n ≥ 1`** |
| **watch for** | the first rung where the landing→next-cycle handoff is visibly the binding term. Note where the possession verdict lands relative to the scheduled landing — that number is what census A3 re-based the margin on |
| **PASS** | 5/5, and the achieved dwell inside the band |
| **ABORT** | any `ABORTED_CANT_MAKE_RELEASE`, or an achieved dwell above 4.4 s (a handoff that runs long here will run longer below) |

### R2 — 3.0 s

| | |
|---|---|
| **goal** | `throw_height_m: 0.78`, `dwell_time_s: 3.00`, `throw_delay_s: 2.4` |
| **must have landed** | census **A1/A2** — the delay floors became derived (landed 2026-08-22). At this flight the delay floor is **0.281 s**, so a 2.4 s delay clears it 8.5× |
| **measurable gate** | 5/5 CAUGHT; **zero** `ABORTED_CANT_MAKE_RELEASE`; announce→landing lead logged on every cycle |
| **watch for** | `TOSS_MIN_ANNOUNCE_LEAD_S` (2.5 s, WARN-only) starts firing. **Confirm it is inert**: for a level 8a the pre-tilt target is the already-held pose, and since census E5 `catch/pretilt_hold` suppresses CCN's platform target entirely. If the WARN coincides with ANY commanded platform motion, stop and report |
| **PASS** | 5/5 with zero release-window aborts |
| **ABORT** | any `ABORTED_CANT_MAKE_RELEASE` — it means the sequence budget is larger than modelled and R3 is not safe to attempt |

### R3 — 1.5 s ⚠ the first rung where a GOOD cycle can be mislabelled

| | |
|---|---|
| **goal** | `throw_height_m: 0.78`, `dwell_time_s: 1.50`, `throw_delay_s: 0.90` |
| **must have landed** | census **D1** (retention clamped to the announced next release), **D2** (arrival window clamped), **D3** (`ball_held_raw` feeds the live evidence query), **D4** (the seat-edge band wait), **D6** (our own latched ball excluded from `track_active`). All landed 2026-08-21. **AND both pre-R3 measurements — § 3.** |
| **measurable gate** | 5/5 CAUGHT **and** 5/5 records labelled `CAUGHT` (not `BOUNCED`) by `tools/probes/toss_record_miner.py`; zero `REJECTED_TRACK_ACTIVE`; **zero reload interludes** |
| **watch for** | the console and the corpus can now disagree. **Score the miner, not the console.** A cycle the operator saw caught and the record calls `BOUNCED` is a D1 regression, not a bad catch. Start scoring the channel-disagreement log here too (§ 4) |
| **PASS** | 5/5 on BOTH the live verdict and the mined label |
| **ABORT** | any `BOUNCED` label on a cycle the operator saw caught; any interlude; any `REJECTED_NO_BALL` on a cycle that had a ball. **Each of these is the fail-open class — stop the sitting, do not retry at a lower dwell** |

### R4 — 0.65 s level / 1.00 s aimed, and the flight comes down

| | |
|---|---|
| **goal** | `throw_height_m: 0.45`, `dwell_time_s: 0.65`, `throw_delay_s: 0.45` |
| **must have landed** | census **B1** (the no-op positioning move is skipped), **B3** (`_TICK_S` 0.05 → 0.02), **B6** (record + trim off the cycle thread), **A3** (the dwell margin re-based on the sensor arrival edge, 0.137 s), **E5** (`catch/pretilt_hold` raised on EVERY cycle). All landed 2026-08-22. **AND** the B1 orientation surface + the accept-floor redesign, landed 2026-08-23 |
| **measurable gate** | 5/5 CAUGHT; `dip_below_x3 ≤ 0.10 rev` on **every** cycle; `sensor_held_at_dispatch = true` on all 5; **log the per-cycle `dispatch → catch-stroke-end` gap** |
| **watch for** | (a) a mis-ordered `prime_hold`/`armed` toggle shows up HERE as a `dip_below_x3` excursion — the two now toggle at ~0.8 Hz and that has never been run; (b) E1/E2 conflict — **any platform motion inside `arrival ± [0.30, 0.50] s` is a stop**; (c) B1 — see the box below, and read it before diagnosing anything |
| **PASS** | 5/5 with every `dip_below_x3` inside band and a positive stroke gap on every cycle |
| **ABORT** | any `dip_below_x3 > 0.10 rev`; any commanded platform motion inside the settle/freeze window; **any negative `dispatch → catch-stroke-end` gap** |

> **⚠ B1: `POSITIONING skipped` should appear on EVERY cycle but the first —
> aim armed or not.** This box replaced a 2026-08-22 version that said the
> opposite ("it NEVER appears on an aimed chain, and that is correct"), which was
> true of the build that could not verify an orientation and is not true of this
> one.
>
> `_toss_already_positioned` has two conditions now: a FRESH
> `trajectory/commanded_pose`, and a match on position AND orientation within
> tolerance (17.5 mm per axis, 2.71 mrad per rotvec component — the tilt whose
> landing drift equals that same 17.5 mm at the tallest admitted throw). A
> chained cycle matches both trivially: the previous cycle's `ACTION_STAY` left
> the platform holding the pose it threw and caught from, and this cycle
> recomputes the identical pre-tilt from the identical (catch pose, flight, aim).
>
> Diagnose it this way:
>
> - **cycle 1 has no `POSITIONING skipped`, and one WARN line saying
>   `toss throw_delay raised … for this cycle`** → expected on every sitting that
>   starts from a level platform (i.e. every aimed one). Nothing to chase; the
>   cadence you asked for starts at cycle 2.
> - **cycles 2..N have no `POSITIONING skipped`** → *now* it is the finding:
>   the platform is drifting off the throw pose, `trajectory/commanded_pose` is
>   stale (or the running `trajectory_node` predates the topic — check
>   `ros2 topic list | grep commanded_pose`), or the aim is changing per cycle.
> - **the raise WARN appears on a cycle other than the first** → the previous
>   cycle did not end in `ACTION_STAY` (a MISS's `go_home`, a reload interlude's
>   recentre) or something else moved the platform. Read the WARN's pose.
> - **`POSITIONING skipped` on a cycle whose aim you believe is armed, with no
>   preceding cycle at that aim** → **stop the sitting.** Either the release was
>   not tilted after all or the guard has regressed; either way the aim is not
>   flying.

### R5 — 0.70 s level / 1.08 s aimed at the TARGET flight ⚠ the machine stops being quiescent

> **⚠ THIS RUNG'S NUMBERS DEVIATE FROM THE CENSUS, DELIBERATELY. Read why.**
>
> The census specifies R5 as **dwell 0.40 s at `T ≥ 0.64 s`**, sized against the
> hand floor alone (0.3932 s at that flight). **The shipped floors REFUSE it.**
> At `T = 0.64` the derived `:642` dispatch budget is **0.3037 s**, so even the
> old plumbing floor `0.3037 + 0.137 = 0.4407 s` was above 0.40. Verified against
> the real FSM (probe, 2026-08-22): that goal returns `REJECTED_THROW_DELAY`.
>
> A 0.40 s dwell is not reachable at ANY flight on this build once the
> pre-dispatch sequence is counted (§ 2.0): the smallest legal dwell over the
> whole C-HAND-3 band is **0.471 s** (0.482 s with an ILC artifact loaded), at
> `T ≈ 1.145` near the band ceiling, where the cadence is 37.1 throws/min
> (`--frontier`, 2026-08-23). Chasing 0.40 s costs cadence
> rather than buying it — § 0 consequence 2, arriving in practice.
>
> So R5 keeps its PURPOSE — the first sitting at the target flight, the rung
> where F1/F6/F7 must have landed because the machine is no longer quiescent —
> and takes numbers the machine admits, with a deliberate margin above the floor
> for a flight nothing has been flown at continuously: **0.70 s dwell, 21.9 ms
> above its 0.678 s floor, and a 0.47 s delay 42.0 ms above its 0.428 s one.**

| | |
|---|---|
| **goal** | `throw_height_m: 0.31`, `dwell_time_s: 0.70`, `throw_delay_s: 0.47` — 49.9 throws/min |
| **must have landed** | census **F1** (the miss-cleanup floor re-derived from COMPLETION rather than service acks), **F6** (pipelining, OR a demonstrated verdict path with ≤ 0.10 s latency), **F7** (invariant S5 re-argued **in writing** — at this dwell the "quiescent wait" is under 0.2 s and the reactive catch path is effectively always live). **None of these has landed. R5 is NOT reachable today.** |
| **measurable gate** | 20/20 CAUGHT across 4 sessions; measured `landing → next release` within 20 ms of the modelled floor; sustained **49.9 (level) / 37.9 (aimed) throws/min** |
| **watch for** | the C-HAND-1 no-overlap margin, per cycle. **A negative `dispatch → catch-stroke-end` gap is an abort-the-sitting event, not a data point.** Also: this is the first sitting at a ~0.50 s flight, so re-check the § 0 table's short-flight column against what the hand actually does before R5-prime tightens the dwell on top of it. And log `landing → hand back inside the park band` — the model says 0.190 s at this flight, and `handoff_margin_s` is now sized on it |
| **PASS** | 20/20 and the timing within 20 ms of model |
| **ABORT** | any negative stroke gap; any miss (`stop_on_miss` handles it, but debrief before retrying); **any `REJECTED_HAND_NOT_PARKED` on a cycle whose previous catch was good** — that is the handoff margin being too small, not a machine fault |

### R5-prime — the operating point ⭐

| | |
|---|---|
| **goal** | `throw_height_m: 0.31`, `dwell_time_s: 0.66`, `throw_delay_s: 0.44` |
| **flight / apex** | 0.5029 s / 0.31 m — 8 ms above the C-HAND-3 band FLOOR |
| **must have landed** | everything through R5 |
| **measurable gate** | **cycle period 1.163 s ⇒ 51.6 throws/min**, sustained, with or without an armed aim |
| **watch for** | this rung is TIGHT: against the ILC-loaded floors the dwell clears by **11.9 ms** and the delay by **12.0 ms** (probe, 2026-08-23). Log the stroke gap on every cycle and treat the sitting as instrumentation, not as a demo. Do not round any of the three numbers down, and do not round the height down either (see the cliff warning in § 2) |
| **PASS** | sustained 51.6 throws/min with a positive stroke gap on every cycle |
| **ABORT** | as R5 |

**This is the maximum-throughput operating point of the machine as built —
51.6 throws/min, whether or not an aim is armed.** The absolute frontier is 54.3
(§ 2.0), and this rung is that frontier with ~12 ms of clearance on both floors
and 8 ms of flight-time clearance, for a combined cost of 2.7 throws/min.

> **This is NOT the 60.4 throws/min this file once claimed, and the gap is not
> the hand.** The hand floor at this flight is 0.487 s and it is not what binds:
> the binding term is `throw_delay + handoff_margin`, where the delay must cover
> the kind-0 dispatch budget **plus the whole pre-dispatch sequence** (§ 2.0).
> The two things that WOULD have moved it both landed on 2026-08-23 and are why
> the aimed column of this file disappeared:
>
> 1. **`trajectory/commanded_pose` publishes the orientation**, so
>    `_toss_already_positioned` verifies a pre-tilt pose and the census-B1 skip
>    fires on an aimed chain. That was the whole 0.38 s gap — aimed sittings went
>    from 39.7 to 51.6 throws/min, a 30 % improvement, and it is the entire
>    cadence content of that day's work.
> 2. **The accept-time delay floor models the sequence it is measured against.**
>    It bought no cadence and was never going to; what it bought is that a goal
>    these gates accept cannot abort mid-sequence with the hand committed, which
>    is what would have caught this file's own numbers a day earlier.
>
> Anything faster than the 54.3 frontier is R6, and R6 is a firmware fork that is
> not being built.

### ~~R6 — 0.25 s~~ — DEFERRED FIRMWARE FORK, DO NOT BUILD

Needs a Platform Teensy flash changing `calcCatch`'s geometry, and with it a
re-derivation of C-HAND-3's `ARM_WINDOW` bound, `hand_stroke.HandStrokeModel`,
`sim/hand/trajectory.py`, and **every stroke landmark** (`x2` / `x3` / `x5`) that
the tilt map and the catch tuning were validated at. Operator decision 3,
2026-08-21: not being built.
---

## 2.9 Command reference — every field of `jugglebot/toss_continuous`

The rungs above give three numbers. This section is what those three numbers sit
inside, so an operator can type the command without reading the `.action` file.
Every default and every bound below is read from
`ros_ws/src/jugglebot_interfaces/action/TossContinuous.action`, the numerics gate
`reload_coordinator_node._invalid_toss_session_goal_field`, and the generated
`hardware_config` — never from memory.

### The template

```bash
ros2 action send_goal --feedback jugglebot/toss_continuous \
  jugglebot_interfaces/action/TossContinuous \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0},
    throw_height_m: 0.31,
    num_throws: 5,
    dwell_time_s: 0.66,
    throw_delay_s: 0.44,
    catch_vel_scale: 0.0,
    stop_on_miss: true,
    on_empty_cup: 'STOP',
    max_reloads: 0}"
```

`--feedback` is worth the noise at these cadences: the feedback carries
`cycle_index`, the live cycle's phase and a running `catches_confirmed`, which is
the only way to watch a sitting degrade before its terminal.

### Every field

| field | type / units | omitted or `0` means | validated range | refused as |
|---|---|---|---|---|
| `catch_position` | `geometry_msgs/Point`, mm, **STOW-relative platform frame** (`z: 170.0` is the ACTIVE plane) | `(0, 0, 0)` — a real request, **not** a default. `z: 0` is 170 mm below the active plane and dies. Always state it | finite; `abs(x), abs(y)` within `toss_workspace_xy_mm` (160.0), `abs(z - 170)` within `TOSS_Z_BAND_MM` | non-finite ⇒ `REJECTED_BAD_GOAL(catch_position.x)` / `.y` / `.z`; outside the box ⇒ `REJECTED_WORKSPACE` |
| `throw_height_m` | `float64`, apex above the release plane | the config default FLIGHT `JB_OP_TOSS_FLIGHT_TIME_DEFAULT_S` = 0.8 s (about a 0.784 m apex) — the flight is the default, not a height | finite, non-negative; converted once by `flight_time_from_height`, then gated by the derived C-HAND-3 envelope | negative or non-finite ⇒ `REJECTED_BAD_GOAL(throw_height_m)`; outside the envelope ⇒ `REJECTED_THROW_ENVELOPE(<bound>: ...)`, which names the bound it broke |
| `num_throws` | `int32` | `0` ⇒ `REJECTED_NUM_THROWS`. There is **no** default | 1 to `JB_OP_TOSS_SESSION_MAX_THROWS` = 20, inclusive | `REJECTED_NUM_THROWS` |
| `dwell_time_s` | `float64`, previous SCHEDULED landing to next release | `JB_OP_TOSS_SESSION_DWELL_DEFAULT_S` = 6.0 s | finite, non-negative, and at least `required_dwell_s` = `max(throw_delay + handoff_margin, hand_floor)` — see § 0 | negative or non-finite ⇒ `REJECTED_BAD_GOAL(dwell_time_s)`; under the floor ⇒ `REJECTED_DWELL` |
| `throw_delay_s` | `float64`, goal-accept to the FIRST release | `DEFAULT_TOSS_THROW_DELAY_S` = 5.0 s, which puts the dwell floor at 5.137 s — **even R1 is illegal at the default** | finite, non-negative, and at least `min_throw_delay_s` = the kind-0 dispatch budget **plus the pre-dispatch sequence** (0.361 s at the 0.80 s flight; 0.428 s at the R5 flight with an ILC artifact loaded) | negative or non-finite ⇒ `REJECTED_BAD_GOAL(throw_delay_s)`; under the floor ⇒ `REJECTED_THROW_DELAY` |
| `catch_vel_scale` | `float64`, multiplier on the armed catch speed | `JB_OP_CATCH_VEL_SCALE_DEFAULT` = 0.9 | finite, non-negative; catch_coordinator clamps to `[0.3, 1.5]` | negative or non-finite ⇒ `REJECTED_BAD_GOAL(catch_vel_scale)`. Outside the clamp is logged and clamped, never refused |
| `stop_on_miss` | `bool` | **`true`** — the IDL default, and load-bearing: an omitted field must mean STOP | n/a | n/a. It governs the `MISSED` class only; every `REJECTED_*`/`ABORTED_*` stops the session regardless of it |
| `on_empty_cup` | `string` | `"STOP"` — and anything that is not exactly `"RELOAD"` resolves to STOP (empty, misspelt, an older client's unset field) | `STOP` or `RELOAD` | never refused; an unreadable value fails closed to STOP |
| `max_reloads` | `int32` | `JB_OP_TOSS_SESSION_MAX_RELOADS` = 3 | non-negative | negative ⇒ `REJECTED_BAD_GOAL(max_reloads)` |

**`REJECTED_BAD_GOAL(<field>)` names the field.** That is the point of the code:
the numerics gate runs before anything is built or installed, and the log line
carries every value it was given, so a typo is diagnosed from one console line.

**Two things that change what the rungs mean have no goal field at all** and are
read from the generated config: the tier (`JB_OP_TOSS_TIER`, `8b` as shipped —
8a and 8b differ in whether the throw site is the live commanded pose) and
whether layer 3 may correct the aim (`JB_OP_TOSS_ILC_ENABLED`, `false` as
shipped). Neither is settable per goal; both are worth reading back before a
sitting.

### One worked example per rung

Only the fields that CHANGE are shown; everything else stays as the template.
`catch_position` is `{x: 0.0, y: 0.0, z: 170.0}` throughout — the ladder walks
cadence, never the catch site.

| rung | what the operator varies | the line |
|---|---|---|
| **R0** | the cadence pair only | `throw_height_m: 0.78, num_throws: 5, dwell_time_s: 5.60, throw_delay_s: 5.00` |
| **R1** | dwell **and** delay — the delay is not optional (at the 5.0 default the floor is 5.137 s, so 4.10 is `REJECTED_DWELL`) | `throw_height_m: 0.78, num_throws: 5, dwell_time_s: 4.10, throw_delay_s: 3.50` |
| **R2** | dwell + delay | `throw_height_m: 0.78, num_throws: 5, dwell_time_s: 3.00, throw_delay_s: 2.40` |
| **R3** | dwell + delay | `throw_height_m: 0.78, num_throws: 5, dwell_time_s: 1.50, throw_delay_s: 0.90` |
| **R4** | **the FLIGHT comes down** as well as the pair | `throw_height_m: 0.45, num_throws: 5, dwell_time_s: 0.65, throw_delay_s: 0.45` |
| **R5** | the flight again, at the target | `throw_height_m: 0.31, num_throws: 5, dwell_time_s: 0.70, throw_delay_s: 0.47` |
| **R5-prime** | the dwell tightens at the R5 flight | `throw_height_m: 0.31, num_throws: 5, dwell_time_s: 0.66, throw_delay_s: 0.44` |

`stop_on_miss: true`, `on_empty_cup: 'STOP'` and `max_reloads: 0` are the
standing settings of § 1 and are **not** varied by any rung.
`catch_vel_scale: 0.0` (the config default 0.9) is likewise fixed, and not for
convenience: it is a FLOOR term — the catch is armed at `event_vel x scale` and
the catch tail is inversely proportional to it, so a slower catch is a LONGER
turnaround and changing it invalidates every dwell in the table above.

Repeat a rung at a higher count once its gate is met by raising `num_throws`
alone. R5's gate asks for 20/20 across four sessions, i.e. four goals at
`num_throws: 5`, not one at 20 — the per-session terminal is part of what is
being scored.


## 3. The two measurements that must happen BEFORE R3

Neither is an argument. Both are measurements, and R3 does not run until they
exist.

### 3.1 Re-measure the sensor arrival band, post-FW 14 ⬜ NOT DONE

**What**: the empty→held edge's offset from the announced landing, over ≥ 30
catches. Today's constants are `ARRIVAL_BAND_MIN_S = 0.137` and
`ARRIVAL_BAND_MAX_S = 0.80` (measured +137…+798 ms, median +399, n = 35 over
three 2026-08-10 bags).

**Why it must be re-taken**: that band was captured when the can-bridge
uptime-dependent dispatch shift was **+54…+133 ms**. FW 14 cut it to **10–20 ms**
(`logbook/2026-08-15-fw14-validated-arc-closed.md`). The band is expected to
COLLAPSE, and four separate constants shrink with it because they are all derived
from those two names: `DEFAULT_SESSION_DWELL_MARGIN_S`, `CATCH_CONFIRM_WINDOW_S`,
`DEFAULT_SESSION_MISS_CLEANUP_S`, and the interlude's seat-edge band wait.

> **One of those four has a FLOOR under it now, and it will not follow the band
> all the way down.** `toss_session.handoff_margin_s` is
> `max(dwell_margin_s, hand_stroke.catch_park_reentry_s(v, scale))` — the verdict
> can only arrive once the hand is ALSO back inside the park band, and that
> second term is hand geometry, not sensor latency. It is **0.190 s** at the
> R5-prime flight against today's 0.137 s arrival term, so the park term is
> already the binding one at every rung from R4 down, and a collapsing arrival
> band does not move the dwell floor there at all. Where the re-measure DOES pay
> is R0–R3 (park re-entry 0.120 s at the 0.7977 s flight, still under 0.137) and
> in the other three constants. Size the expected saving accordingly before
> booking the sitting.

**How**: mine any fresh bag — no new sitting is needed if one already exists at
FW 14 with ≥ 30 announced catches. The band is
`t_catch_raw_ros − announce_landing_time_ros` per row: the RAW empty→held edge
(zero debounce, which is why the RAW field and not `t_catch_deb_ros`) against the
announced landing, both on the ROS clock. Both fields are already in the corpus —
there is no new instrumentation to add and no bespoke miner flag.

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
python tools/probes/toss_record_miner.py --bag <fresh-bag> --sensor-only --jsonl \
    > /tmp/band.jsonl
python - <<'EOF'
import json, statistics
d = [json.loads(l) for l in open('/tmp/band.jsonl')]
b = [r['t_catch_raw_ros'] - r['announce_landing_time_ros'] for r in d
     if r.get('t_catch_raw_ros') and r.get('announce_landing_time_ros')]
print('n=%d  min=%.3f  median=%.3f  max=%.3f' %
      (len(b), min(b), statistics.median(b), max(b)))
EOF
```

`--sensor-only` skips the mocap/tracker pass, which this measurement does not
need and which costs minutes.

**Outcome**: if the band collapses, update `ARRIVAL_BAND_MIN_S` /
`ARRIVAL_BAND_MAX_S` **and nothing else** — every consumer follows. R4 gets
materially cheaper. If it does NOT collapse, say so: the margin is `PROVISIONAL`
precisely because this is unresolved, and confirming the old band is a result.

> **A fifth consumer joined the ceiling on 2026-08-23**, and it is the reason
> that re-measure is worth booking rather than deferring: the ARRIVAL window's
> boundary between adjacent cycles is now `ball_possession.arrival_boundary_t`,
> which reads `ARRIVAL_BAND_MAX_S` directly (C-POSSESS-1 § 3.4 clauses C.1/C.2).
> A collapsing ceiling therefore buys **cadence headroom** as well as the four
> constants above: the boundary stops eating this ball's band at any cycle period
> under `ARRIVAL_BAND_MAX_S + arrival_lead_s`, which is 1.000 s today and would
> fall to about 0.47 s on the 2026-08-23 capture's +267.5 ms ceiling — moving the
> R6 fork's 0.7529 s period from "the band cannot be watched out" to "clear". No
> rung on the table above is affected either way; R6 is.

### 3.2 The 71 ms measured poll cadence against the configured 20 ms ⬜ NOT DIAGNOSED

**What**: `jugglebot_ball_detect.check_interval_ms` is **20** (50 Hz nominal).
The measured poll cadence is **~71 ms** — a 3.5× gap with **no diagnosis**
(`plans/active/toss-selftuning.md` § Open findings).

**Why it is a prerequisite and not a footnote**: at the R5-prime dwell that gap
is **11 % of the whole dwell** on the level column (71 ms of 0.63 s), and it sits directly under the `ball_seated`
precondition. A gate whose sample interval is 3.5× what its config says is a gate
whose staleness bound nobody has actually established.

**This is a CAN-BRIDGE INVESTIGATION and it is NOT built here.** It belongs with
the bridge's polling path, not with the cadence work — the census flags it, this
runbook carries it, and neither resolves it. Open it as its own investigation
before R3 (`/investigate`), and record the finding here.

---

## 4. Aim-channel truth — read before scoring any rung

**`arrival_dir` is the PRIMARY and ONLY aim residual driving the ILC update.**
`land_err_x` / `land_err_y` are **MONITOR-ONLY**: masked out of the aim update in
the weight matrix, never Q-arbitrated against `arrival_dir`, and optionally
reported bias-corrected through the fitted parity `b(z)` profile.

This is a resolution **by decision**, not by measurement. The H2 fixtured-ball
centroid capture was RETIRED by the operator on 2026-08-21 — conventional markers
on the ball would corrupt the very trackable surface being measured — and the
mechanism reading (platform-frame occlusion near the bottom of the stroke) is
consistent with the arc's own data: the bias is large at z ≈ 880 mm (catch-plane
height) and vanishes by z ≈ 1880 mm, parity-EVEN.

`arrival_dir` is whole-arc and therefore bias-immune; the plane-level residual is
not.

**ABSOLUTE centering closes through CATCH OUTCOMES.** An arrival-direction loop
converges the ball onto the cup the *arc* points at; a constant registration
offset between the mocap frame and the physical cup is invisible to it. The
penalty loop is the ground truth for "centered on the cup", and a residual
~10 mm registration bias against the 35 mm capture radius is tolerable and
visible in the penalty trend.

**The standing validation is the per-toss channel-disagreement log**
(`ilc_fit_lib.channel_disagreement`), and it has exactly two readings:

- the `arrival_dir`-driven loop converges **while** the plane residual holds the
  known `b(z)` profile shape ⇒ **the model is confirmed**;
- catch rate plateaus **with** a converged aim ⇒ **the decision is wrong**, the
  loop has centred on the measurement's cup rather than the world's, and C3
  re-opens.

Score it on every rung from R3, alongside the miner's labels. It costs nothing —
it is already in the record — and it is the only evidence that will distinguish
those two outcomes.

> The ball's physical radius is an operator-supplied measured constant (74 mm
> diameter, calipers) and is already in `hardware_config.yaml`. There is no
> operator line item for it in this runbook.

---

## 5. A cancel is always deferred at these dwells — census F4

`TOSS_CANCEL_CUTOFF_S = 0.25`. Cancels arriving later than
`t_release − 0.25 s` are **DEFERRED** to the FSM's own terminal, because
aborting a catch mid-flight drops a ball on the robot.

**At the ladder's dwells that cutoff is comparable to the entire dwell.** From
R5 down, a cancel is *always* deferred: your stop button gains **one full cycle
of latency**, and the machine will complete the throw and the catch in progress
before it stops.

This is expected and correct, and it is stated here rather than left to be
discovered mid-sitting. Two consequences:

- **The cancel button is not the E-STOP.** For anything that needs to stop NOW,
  use the E-STOP. The cancel is a graceful-completion request.
- The node logs one line per deferral naming the phase. If you press cancel and
  the machine keeps throwing, read that line before pressing anything else — it
  is telling you the ball is committed.

---

## 6. Related runbooks

- `tests/hardware/session_anomaly_fixes.md` — § SECTION CONT, the shipped-cadence
  baseline this ladder starts from, and the § PASS/ABORT HAND rows every rung's
  gate reuses.
- `tests/hardware/session_phase8_toss_hardware.md` — the single-toss T-rungs.
  A cadence failure that is really a THROW failure belongs there.
- `tests/hardware/session_hand_ball_sensor.md` — the possession sensor. The
  pre-R3 band re-measure (§ 3.1) reads its instrumentation.
