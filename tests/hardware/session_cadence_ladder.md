# Hardware Session — the CADENCE LADDER (R0 → R5-prime)

**Plan**: `plans/active/toss-selftuning.md` § 11 (the timing census) and
`plans/active/critical-point-ilc.md` (operator decision 3, 2026-08-21)
**Action**: `jugglebot/toss_continuous` (`TossContinuous.action`)
**Depends on**: `session_anomaly_fixes.md` § SECTION CONT green at the shipped
6.0 s dwell. This file starts from that baseline and walks the dwell DOWN.
**Goal**: reach the tuning-phase operating point — **dwell 0.49 s at a 0.31 m
throw (flight 0.5029 s), cycle period 0.993 s, 60.4 throws/min** — one bench
sitting per rung, with a measurable gate and a stated ABORT at each.

The operator's decision names this point as "dwell 0.49 s at flight 0.4949 s,
~61 throws/min". That is the C-HAND-3 band FLOOR, and it is the right target;
the goal parameters here sit a deliberate 8 ms of flight time above it, which
costs 0.5 throws/min and buys clearance from a cliff — see the warning under the
ladder table in § 2.

> **⚡ THIS SESSION FIRES REAL THROWS, FASTER EACH RUNG.** By R4 the machine
> throws and catches roughly once a second and does not stop between cycles; by
> R5-prime it is quiescent for under 0.1 s per cycle. On any miss the ball
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

**Three consequences to state plainly before the first sitting:**

1. **A 0.25 s dwell is unreachable at EVERY admitted flight time.** The minimum
   over the band is 0.2505 s, at the very ceiling (apex 1.62 m), and it rises
   from there. The true 0.25 s dwell is rung R6 and it is a **DEFERRED FIRMWARE
   FORK** — it needs a Platform Teensy flash changing `calcCatch`'s geometry.
   **It is not being built.** Operator decision 3, 2026-08-21.
2. **The dwell is the wrong operator variable.** Cycle *period* is nearly flat
   across the band and bottoms at 0.982 s. Ask for "≥ 60 throws/min", not
   "0.25 s dwell" — and get there by making the FLIGHT short, not the dwell.
3. **R5-prime clears its floor by 3.0 ms.** Dwell 0.49 s against a 0.4870 s
   required floor at the rung's own flight. That is why every rung from R4 logs
   the per-cycle `dispatch → catch-stroke-end` gap, and why a negative one
   aborts the sitting.

### The firmware question the census left open: `t7`, not `t8` — VERIFIED

The census flagged one item to verify before trusting the table above: whether
the C-HAND-1 no-overlap floor is written against `t7` (hand at rest at 0 rev) or
`t8 = t7 + END_PROFILE_HOLD` (0.10 s). **If `t8`, every number above rises by
100 ms** and the minimum achievable dwell becomes 0.351 s — which would put the
0.49 s operating point 97 ms out of reach (floor 0.5871 s at the band floor).

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

**Every rung's goal parameters are given, and every one has been checked against
the real FSMs** — probe, 2026-08-22: construct the `TossSessionSequencer` and
assert `START_CYCLE`, AND construct the cycle `TossSequencer` and assert its
CHECKING pass reaches POSITIONING. Both, because the two refuse for different
reasons and a goal can clear one and die on the other.

They are not suggestions. `throw_delay_s` matters as much as `dwell_time_s`
(the floor is `max(throw_delay + 0.137, hand_floor(T))`, and the DEFAULT 5.0 s
delay makes even R1 illegal), and `throw_height_m` has a **cliff** — see the
warning under the table.

| rung | `throw_height_m` | `dwell_time_s` | `throw_delay_s` | ⇒ flight `T` | required floor | period | throws/min |
|---|---|---|---|---|---|---|---|
| R0 | 0.78 | 5.60 | 5.0 | 0.7977 | 5.137 | 6.398 | 9.4 |
| R1 | 0.78 | 4.10 | 3.5 | 0.7977 | 3.637 | 4.898 | 12.3 |
| R2 | 0.78 | 3.00 | 2.4 | 0.7977 | 2.537 | 3.798 | 15.8 |
| R3 | 0.78 | 1.50 | 0.90 | 0.7977 | 1.037 | 2.298 | 26.1 |
| R4 | 0.45 | 0.75 | 0.55 | 0.6059 | 0.687 | 1.356 | 44.3 |
| R5 | 0.31 | 0.60 | 0.40 | 0.5029 | 0.537 | 1.103 | 54.4 |
| **R5-prime** | **0.31** | **0.49** | **0.35** | **0.5029** | **0.4870** | **0.993** | **60.4** |

**One variable changes per rung.** R0→R3 walk the DWELL down at a fixed 0.78 m
throw; R4 and R5 shorten the FLIGHT (which is where the cadence actually comes
from — see § 0 consequence 2); R5-prime tightens the dwell at the R5 flight.

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
> silently start refusing the rung. `0.31` costs **0.5 throws/min** (60.4 vs
> 60.9) and buys 8 ms of clearance. If a sitting needs the last half-throw per
> minute, take it from the DWELL (0.487 is the true floor at this flight), not
> from the height.

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

### R4 — 0.75 s, and the flight comes down

| | |
|---|---|
| **goal** | `throw_height_m: 0.45`, `dwell_time_s: 0.75`, `throw_delay_s: 0.55` |
| **must have landed** | census **B1** (the no-op positioning move is skipped), **B3** (`_TICK_S` 0.05 → 0.02), **B6** (record + trim off the cycle thread), **A3** (the dwell margin re-based on the sensor arrival edge, 0.137 s), **E5** (`catch/pretilt_hold` raised on EVERY cycle). All landed 2026-08-22 |
| **measurable gate** | 5/5 CAUGHT; `dip_below_x3 ≤ 0.10 rev` on **every** cycle; `sensor_held_at_dispatch = true` on all 5; **log the per-cycle `dispatch → catch-stroke-end` gap** |
| **watch for** | (a) a mis-ordered `prime_hold`/`armed` toggle shows up HERE as a `dip_below_x3` excursion — the two now toggle at ~0.75 Hz and that has never been run; (b) E1/E2 conflict — **any platform motion inside `arrival ± [0.30, 0.50] s` is a stop**; (c) B1: confirm the POSITIONING skip is actually firing (`POSITIONING skipped` in the node log). If it is NOT firing on a level chain, the platform is drifting off B and **that is the finding**, not a nuisance |
| **PASS** | 5/5 with every `dip_below_x3` inside band and a positive stroke gap on every cycle |
| **ABORT** | any `dip_below_x3 > 0.10 rev`; any commanded platform motion inside the settle/freeze window; **any negative `dispatch → catch-stroke-end` gap** |

### R5 — 0.60 s at the TARGET flight ⚠ the machine stops being quiescent

> **⚠ THIS RUNG'S NUMBERS DEVIATE FROM THE CENSUS, DELIBERATELY. Read why.**
>
> The census specifies R5 as **dwell 0.40 s at `T ≥ 0.64 s`**, sized against the
> hand floor alone (0.3932 s at that flight). **The shipped floors REFUSE it.**
> At `T = 0.64` the derived `:642` dispatch budget is **0.3037 s**, so the
> plumbing floor is `0.3037 + 0.137 = 0.4407 s` — above 0.40. Verified against
> the real FSM (probe, 2026-08-22): that goal returns `REJECTED_THROW_DELAY`.
>
> A 0.40 s dwell IS reachable, but only at `T ≥ ~1.0 s` — a 1.23 m apex, cycle
> period 1.40 s, **42.9 throws/min**. That is *slower than R5-prime*, and it is
> the census's own consequence 3 arriving in practice: **the dwell is the wrong
> operator variable.** Chasing 0.40 s costs cadence rather than buying it.
>
> So R5 keeps its PURPOSE — the first sitting at the target flight, the rung
> where F1/F6/F7 must have landed because the machine is no longer quiescent —
> and takes numbers the machine admits: **dwell 0.60 s at the band floor**,
> 54.4 throws/min, with **113 ms of dwell margin** above the 0.4870 s floor for
> a first sitting at a flight nothing has been flown at continuously.

| | |
|---|---|
| **goal** | `throw_height_m: 0.31`, `dwell_time_s: 0.60`, `throw_delay_s: 0.40` |
| **must have landed** | census **F1** (the miss-cleanup floor re-derived from COMPLETION rather than service acks), **F6** (pipelining, OR a demonstrated verdict path with ≤ 0.10 s latency), **F7** (invariant S5 re-argued **in writing** — at this dwell the "quiescent wait" is under 0.1 s and the reactive catch path is effectively always live). **None of these has landed. R5 is NOT reachable today.** |
| **measurable gate** | 20/20 CAUGHT across 4 sessions; measured `landing → next release` within 20 ms of the modelled floor; sustained **54.4 throws/min** |
| **watch for** | the C-HAND-1 no-overlap margin, per cycle. **A negative `dispatch → catch-stroke-end` gap is an abort-the-sitting event, not a data point.** Also: this is the first sitting at a ~0.50 s flight, so re-check the § 0 table's short-flight column against what the hand actually does before R5-prime tightens the dwell on top of it |
| **PASS** | 20/20 and the timing within 20 ms of model |
| **ABORT** | any negative stroke gap; any miss (`stop_on_miss` handles it, but debrief before retrying) |

### R5-prime — the operating point ⭐

| | |
|---|---|
| **goal** | `throw_height_m: 0.31`, `dwell_time_s: 0.49`, `throw_delay_s: 0.35` |
| **flight / apex** | 0.5029 s / 0.31 m — 8 ms above the C-HAND-3 band FLOOR |
| **must have landed** | everything through R5 |
| **measurable gate** | **cycle period 0.993 s ⇒ 60.4 throws/min**, sustained |
| **watch for** | the floor is 0.4870 s, so the goal clears it by **3.0 ms**. Log the stroke gap on every cycle and treat the sitting as instrumentation, not as a demo. `throw_delay_s: 0.35` is 15 ms above its own 0.3346 s floor — do not round it down, and do not round the height down either (see the cliff warning in § 2) |
| **PASS** | sustained 60+ throws/min with a positive stroke gap on every cycle |
| **ABORT** | as R5 |

**This is the maximum-throughput operating point of the machine as built.** The
true optimum is dwell 0.4870 s at the C-HAND-3 band floor itself — **61.1
throws/min** — and this rung is that optimum with 3.0 ms of dwell clearance and
8 ms of flight-time clearance, for a combined cost of 0.7 throws/min. Anything
faster is R6, and R6 is a firmware fork that is not being built.

### ~~R6 — 0.25 s~~ — DEFERRED FIRMWARE FORK, DO NOT BUILD

Needs a Platform Teensy flash changing `calcCatch`'s geometry, and with it a
re-derivation of C-HAND-3's `ARM_WINDOW` bound, `hand_stroke.HandStrokeModel`,
`sim/hand/trajectory.py`, and **every stroke landmark** (`x2` / `x3` / `x5`) that
the tilt map and the catch tuning were validated at. Operator decision 3,
2026-08-21: not being built.

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

### 3.2 The 71 ms measured poll cadence against the configured 20 ms ⬜ NOT DIAGNOSED

**What**: `jugglebot_ball_detect.check_interval_ms` is **20** (50 Hz nominal).
The measured poll cadence is **~71 ms** — a 3.5× gap with **no diagnosis**
(`plans/active/toss-selftuning.md` § Open findings).

**Why it is a prerequisite and not a footnote**: at the R5-prime dwell that gap
is **14 % of the whole dwell**, and it sits directly under the `ball_seated`
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
