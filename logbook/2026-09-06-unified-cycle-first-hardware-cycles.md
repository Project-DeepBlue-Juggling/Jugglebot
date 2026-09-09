---
title: "First unified cycles on hardware — six catches of seven, every one of them a feedforward catch into a parked cup; the throw left ~11 % fast, the replan gate never opened, and the whole cycle flew one levelling frame short"
type: investigation
date: 2026-09-06
status: resolved
phase: "unified-7dof-planner — Phase 5 (UH-3 / UH-5 / UH-6 first cycles)"
related_plan: unified-7dof-planner.md
files_changed:
  - ros_ws/docs/levelling_frame.md
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - tests/hardware/unified_cycle_bench.py
  - tests/hardware/session_unified7_cycle_ladder.md
  - tests/motion/test_unified_cycle.py
  - tests/ros/test_levelling_frame.py
  - tests/ros/test_unified_cycle_levelling.py
  - tests/ros/test_unified_cycle_bench.py
  - tests/ros/test_unified_cycle_integration.py
  - plans/archived/unified-7dof-planner.md
  - plans/active/INDEX.md
  - logbook/2026-09-06-unified-cycle-first-hardware-cycles.md
  - logbook/INDEX.md
subsystem:
  - motion
  - ros
  - tracking
tags:
  - safety
  - testing
  - performance
  - kinematics
  - docs
---

# First unified cycles — the machine juggled, and neither the catch nor the frame was what it looked like

## Summary

**The unified 7-DoF planner flew a whole cycle on hardware for the first time**, on the evening of
2026-09-06, after the morning's `colcon build` of `ce14e2f`. Three rungs:

- **UH-3 PASS** — two carries with a seated ball, `--dx 60` and `--dx 0 --dy 60`, both accepted and
  executed, the ball never disturbed. The refusal chain that ate the morning is closed.
- **UH-5 PASS** — three planned throws at `--apex 0.5`, smooth, guard ARMED and never tripped.
- **UH-6 FLOWN** — four `toss_continuous` goals: G0 refused `REJECTED_MOCAP_STALE` (QTM was not up),
  then **G1 3/3 CAUGHT, G2 3/3 CAUGHT, G3 one MISSED → `STOPPED_ON_MISS`**. **Six of seven caught.**

**And the catch quality FAILS its own criterion.** T-H6 asks for release-velocity parity with the
legacy path; the planned catch asks for the cup to be moving at **0.7 × the ball's arrival speed** at
contact (`cup_cycle.py:225`, `catch_slider_vel_ratio`). Measured across all seven cycles, the ball
arrives at **3961–4161 mm/s** onto a hand doing **−232…+4 mm/s** — a ratio of **0.001–0.059**. The
receive stroke had already finished, **172–185 ms** before the ball got there, and the cup was parked
at 0.313 rev when the ball hit it. **Every catch this sitting was a feedforward catch into a
stationary cup**, and it worked six times because the cup is a good bucket, not because the plan
caught anything.

Two convictions, both closed in this commit, **neither re-flown**:

1. **The throw left too fast and the closed loop that exists to absorb that never opened.** Three
   independent estimators put the release **+5.2…+16.7 %** over plan (mocap parabola, mean +10.9 %;
   flight time +10…+20 %; peak hand rev/s × empirical cup gain +12…+19 %). The ball therefore came
   home **146–195 ms** after the plan expected it. `catch/dynamic_target` — the tracker→planner replan
   channel built for exactly this — published **zero messages all sitting**, because the coordinator's
   `open_loop` short-circuit is raised by the legacy pre-tilt hold and has no unified branch.
2. **The unified cycle was flying one levelling frame short.** `CyclePlan` never passed through
   `levelling.correct_pose` — zero `levelling.*` calls existed anywhere in the cycle machinery — and
   the contract's own manifest never enumerated `plan_cycle`. It is a **drop, not a double-count**,
   and worse than a uniform offset: knot 0's tilt pin was in the **plan** frame while every other pin
   (release, catch, banking) was **gravity** frame, so `tilt_schedule` interpolated across two frames.
   Measured: the commanded release attitude was **exactly mechanical zero**, the platform physically
   at **+9.9…+10.3 mrad** at the instant of release, the ball leaving **−9 mrad into −y on 7/7
   throws** — the *opposite sign* to the legacy path's known +8.5 mrad +y bias — for **9–38 mm** of
   lateral drift over the flight, rim strikes, and the 20–181 ms of bounce that pollutes every
   sensor-side catch estimator.

The host is exonerated: `blas threads: 1` on all three planner nodes, `max_emit_gap_ms` p50 **26.0**
max **35.4** with **zero** samples over 40, `interp_deadline_misses` 0, jitter ≤ 3 µs, zero TX
deferrals. **The 2026-09-06-morning E-STOP class did not recur.**

---

## Symptoms

The sitting ran on `ce14e2f` plus a `colcon build`. Artefacts: the Teensy capture
`temp/logs/cycle_ladder_20260906_193029.log`; two MCAP bags under `~/Desktop/rosbags/2026-09-06_19-*`
(a relaunch at ~19:32 split them); driver CSV/meta at
`temp/logs/unified_cycle_bench_{carry_193247,carry_193313,throw_193345,throw_193400,throw_193407}*`;
miner output under `temp/probes/`.

**Before the rungs.** P5 refused once at the shipped limits — `set_limits 250/3000/150000` fixed it,
exactly as the runbook's precondition 11 says. One relaunch. The very first `toss_continuous` goal
(G0) was refused `REJECTED_MOCAP_STALE` because QTM had not been started.

| Rung | Invocation | Outcome |
|---|---|---|
| **UH-3** | `--rung carry --dx 60`, then `--dx 0 --dy 60` | **ACCEPTED and executed, both.** Plan **213–222 ms** (qp 7–11, val 191–202, cont 1–2), `load1` 3.7–4.7. Hand **flat** — peak = the settle at **0.4949 rev**, driver printing *"0.0 mm above"*. V4 read 0.486 rev at t = 0.25 s on the first carry (the telemetry-stale ceiling) and 0.0049 rev on the second. **V6 FAIL on every rung** and **V7 SKIP** — both driver defects, see Diagnosis. |
| **UH-5** | `--rung throw --apex 0.5`, ×3 | **PASS.** Joined LAUNCH+SETTLE, 2.0 s / 81 knots. Plan **508 / 547 / 571 ms** (val 240–274, cont 237–268). **V2 FAIL** against the driver's bare 500 ms bar — a driver defect, not a machine one. Commanded hand peak **9.6432 rev @ 99.01 rev/s**; encoder worst **9.7591–9.8032 rev**. Planned release `vz` **3131.5 mm/s**. |
| **UH-6** | `toss_continuous`, `throw_height 0.5`, `num_throws 3`, `dwell 6`, `delay 5`, `unified_cycle true` | **FLOWN.** G0 `REJECTED_MOCAP_STALE`; **G1 3/3 CAUGHT** (plan 505 / 593 / 500 ms, `load1` 4.6–5.4); **G2 3/3 CAUGHT** (606 / 585 / 530 ms, `load1` 5.4–6.0); **G3 one MISSED → `STOPPED_ON_MISS`** (500 ms, `load1` 7.4). Guard **ARMED** throughout, **never tripped**. |
| **UH-7** | — | **Not run.** It needs the steady-chain hand-off, which still does not exist. |

**Every UH-6 cycle was a fresh joined solve from rest.** No `EXTEND` fired at this cadence: the 6 s
dwell lets each cycle come to rest, so the coordinator plans `LAUNCH + LANDING` from scratch every
time. The runbook's older claim that cycles 2 and 3 are ~200 ms *extensions* of cycle 1 is wrong and
is corrected in this commit.

**The operator's three observations**, all of which turned out to be the load-bearing evidence:

- *"The catches are rough — the ball lands before or after the receive motion."*
- *"There's a small tilt step just before every throw."*
- *"The balls are thrown slightly backwards; they hit the top of the hand axis."*

---

## Diagnosis

### 1. The catch is feedforward, and the ball is late — four estimators agree on the sign

Ball arrival minus the plan's own catch instant, per cycle:

| Estimator | Offset | What it includes |
|---|---|---|
| Mocap parabola × cup track | **+146…+195 ms** | the clean one — geometry only |
| Hand `iq` impact | **+162…+256 ms** | + the impulse's rise time |
| `ball_held` sensor edge | **+192…+382 ms** | + **20–181 ms of rim bounce** |
| FSM `catch_dt` | **+198…+385 ms** | + the same bounce, + FSM latency |

**The hand is ALWAYS early.** The spread between the first row and the last three is not
disagreement — it is the rim bounce (§ 5) entering every sensor-side estimator and none of the
geometric one. Decomposed, the lateness is **departure lag +50…+66 ms** plus **flight excess
+139…+315 ms**, and the two close on the total to **4–5 ms**.

The ordering, every cycle without exception:

```
plan's catch instant  →  receive stroke completes  +172…185 ms
                      →  cup idle                    +0…79 ms
                      →  ball actually arrives     +146…195 ms
```

### 2. What the cup was doing when the ball hit it

The plan **executed correctly** — this is not a tracking failure:

| Quantity | Measured | Design |
|---|---|---|
| Cup velocity at the **planned** catch instant | **−1831…−2158 mm/s** | −2199 mm/s (**83–98 %**) |
| Cup velocity at the **actual** contact | **−232…+4 mm/s** | — |
| Ball arrival speed | **3961–4161 mm/s** | — |
| `catch_slider_vel_ratio` achieved | **0.001–0.059** | **0.7** |
| Closing mismatch | **3730–4157 mm/s** | ~4.0–4.4× design speed, **~17× the energy** |
| Catch runway consumed | **2–7.4 mm** | 120 mm planned (3.78 rev) |

`pos_cmd` sits flat at **0.313 rev** while `pos_meas` punches down **0.05–0.19 rev in ~11 ms** at
**+13.8…+19.9 A** — the signature of an external force driving the slider, i.e. the ball landing on
a parked cup and the position loop fighting back. The runway the QP reserves as a hard constraint,
the whole reason `catch_runway_decel_mps2` exists, went **unused**.

One record field is misleading and is fixed in this commit: the session record's `catch_knobs` for a
unified session reported `catch_vel_ratio 0.6 / catch_vel_scale 0.9`. Those are the **legacy Teensy
catcher's** knobs. The operative number under unified mode is `catch_slider_vel_ratio` **0.7**, and
nothing was reporting it.

### 3. The throw left too fast — three methods, one sign

| Method | Excess over plan |
|---|---|
| Mocap parabola fit | **+5.2…+16.7 %** (mean **+10.9 %**) |
| Flight time | **+10…+20 %** |
| Peak hand rev/s × empirical cup gain | **+12…+19 %** |

Achieved flight **0.766–0.837 s** against **0.6387 s** planned; apex rise **556–682 mm** against the
**500 mm** asked. A ball that leaves ~11 % fast comes home late by roughly the same fraction of the
flight — which is the **+139…+315 ms** flight-excess term of § 1, arriving from the throw side.

### 4. The replan gate never opened — traced, not inferred

`/catch/dynamic_target` carried **0 messages** for the entire sitting. The trace:

- `catch_coordinator_node.py:~870-871` computes
  `open_loop = ((pretilt_hold or JB_OP_RELOAD_PLATFORM_OPEN_LOOP) and catch_armed and
  announcement_seen)`. All three terms were **True**.
- `pretilt_hold` is raised by `reload_coordinator_node.py:~6321-6323` with **no unified branch** — it
  is the legacy reload choreography's hold, and a unified session raises it too.
- Under `open_loop` the coordinator calls `_republish_pretilt()`, a **no-op** since `_pretilt_cmd` was
  nulled at `~:678`. The legacy path's equivalent suppression is covered by `_publish_toss_reach`,
  deliberately off for unified at `~:5793` — so the unified path has **both** publishers suppressed
  and no third one.
- **The consumer was ready the whole time**: `trajectory_node:~3087` bypasses `catch_armed` under
  unified mode and `~3145` routes a dynamic target to `_replan_cycle_from_target`. The gap was one
  line upstream of a fully-built path.

**Why the suite was green.** `tests/ros/test_unified_cycle_integration.py:~2398` exercises the replan
route but **never calls `_on_pretilt_hold(True)`** — a coordinator in a state a session never reaches.

**A second defect sat on the same path** and would have bitten the moment the first was fixed:
`trajectory_node:~3204-3205` wrote `target_pos.x/y` straight into `catch_site_mm`, but `target_pos` is
the **platform centroid** the coordinator already lever-corrected and `catch_site_mm` is the **cup** —
a **~13.5 mm** bias at a 12° receive tilt, larger than the movement threshold a replan is gated on.

**And a replan would not have saved this sitting anyway**: the tracker was blind for most of each
flight (§ 9). Each replan is a **~230 ms** in-line solve (759 ms under load), which is its own reason
to gate it on real movement rather than on every tracker frame.

### 5. The frame drop — measured on the machine, reproduced offline

**`CyclePlan` never passed `levelling.correct_pose`.** The legacy path corrects at ingest
(`trajectory_node._pose_from_msg:~2497`); `grep -rn "levelling\."` over the whole cycle machinery
returned **nothing** for the entirety of Phases 1–4, and the contract's own enforcement list —
`_LEVELLING_MANIFEST` in `tests/ros/test_levelling_frame.py`, the C-LEVEL-1 pin — **never enumerated
`plan_cycle`**, so nothing failed.

It is a **drop**, and worse than a uniform offset because it was not uniform. `_start_tilt_for`
(`unified_cycle.py:~1317`) pinned knot 0 to the **seed's own** `pose[3:5]` — a **plan-frame** quantity
— while the release pin, the catch pin and the banking field are **gravity-frame** by construction
(`tilt_to_throw` of a ballistic take-off velocity, `tilt_to_receive` of an observed arrival,
`tilt_to_receive` of `g − a_cup`). `tilt_schedule` then interpolated **between two frames**.

Measured, tilt map `2026-08-10-3bf7964f`:

| | Commanded | Physical |
|---|---|---|
| Prepare attitude (knot 0) | **−11.663 mrad** — bit-for-bit the levelling correction | level |
| Release attitude | **exactly mechanical zero** (\|rx\| < 0.03 mrad) | **+9.9…+10.3 mrad, az −90°** |
| Ball launch direction | — | **−9 mrad mean, into −y, 7/7 throws** |

The legacy path's known aim bias is **+8.5 mrad into +y** — the **opposite sign**, which is what
makes this a frame error rather than a calibration drift. The predicted knot-0→release tilt step is
**0.671°**; the measured step is **+11.67 mrad = +0.669° about +x**. That is the operator's *"small
tilt step before every throw"*, to three digits.

Downstream: **9–38 mm** of lateral drift over the flight, rim strikes on arrival, and the **20–181 ms
of bounce** that contaminates every sensor-side catch estimator in § 1.

**UH-5 is the control.** It has no levelled prepare pose, so it showed **no step** — and the **same
un-levelled release**, +11.5…+11.8 mrad. Both halves are consistent with a single missing correction
rather than with two independent faults.

One field to distrust: the session record's `tilt_map_applied: true` asserts the map was **LOADED**,
not that it was **APPLIED**. It was true and meaningless.

### 6. The throws themselves — and the tightest number of the sitting

Encoder overshoot past the commanded 9.6432 rev peak: **+0.101…+0.220 rev**, **uncorrelated with peak
velocity**. Worst encoder peak **9.8699 rev** = **0.089 rev (2.8 mm)** below the 9.9594 rev band
ceiling — **the tightest margin of the sitting**. Worst hand deviation **1.1262 rev** = 45 % of the
2.5 rev band, 56 % of `MAX_LEAD_HAND_REV`, a clear improvement on sitting two's legacy-stroke 1.9847.
Aim **−9 mrad into −y**.

### 7. Host and link — clean, and the `cont` mystery resolved

`blas threads` **1** on all three planner nodes; setpoint age **8–22 ms**; `max_emit_gap_ms` p50
**26.0** / max **35.4** with **zero** samples > 40; `interp_deadline_misses` **0**; interp jitter
**≤ 3 µs**; **0** TX deferrals; `link=1 fault=0`.

**`cont` is not overhead.** It is the stage split's residual, and on a joined install it is the join's
**third full `validate_cycle` pass** over the concatenated plan — **247.18 ms, 99.7 % of the join**.
`validate_cycle` itself measured **2.44 ms/knot, flat across nine solves**. That is why `cont ≈ val`
on every UH-5 and UH-6 line, and it is expected.

### 8. The `[hand7]` deltas are NON-EVIDENCE

The console showed `sent` advancing at exactly 500 Hz per stage with `lead 0`, `dev_over 0`,
`unseen 0`, `stale 0`, `dev_max 10.9794`. **None of that is a clean bill of health.** The FW 18
counter-gate defect (`leg_interp.cpp:684` omits `s_output_enabled`) makes those counters exclude the
ticks that actually transmitted, and `dev_max 10.9794` is dead boot-cumulative history no reset short
of a Teensy reboot clears. The block proves one thing this sitting — `sent` says the lane ran.

### 9. Anomalies, recorded

- **QTM binds the flying ball to the stale `Catching Cone` rigid body ~60–80 ms after release**, on
  **5 of 7 throws** (`CONFIRMED = 0`), while the raw marker track stays complete. Memory says that
  body was disabled on 2026-08-29; **it is back**. This is why a replan would not have rescued this
  sitting: the tracker is blind for most of every flight.
- **Seven phantom `human_throw` balls** (tracks 7–13) minted from static reflectors near the Ball
  Butler, each with a full lifecycle, all before the goal that MISSED.
- **Empirical cup travel from the bag: 34.0–34.5 mm/rev** (two independent estimators) against the
  planner's **31.628** and the owner's bench **32.57**. The bench number explains ~3 % of the throw
  excess; the bag's estimate would explain all of it, and the two disagree. Open question 1.
- **Every learned correction was inactive**: `speed_bias_applied 1.0`, `ilc_vel_trim 0.0 no_artifact`,
  `toss_cal_loaded false`, aim 0 — while the ILC's own independent fit for this plant is
  `event_vel_trim = −0.1076`. The trim that would have removed most of the throw excess existed and
  was not in the loop.
- **A `> 2.0 s` solve is a lying-ack hazard.** On a client timeout the coordinator logs *"no plan
  installed, nothing was commanded"* (`~:7136`) while `trajectory_node` **installs anyway**
  (`~:4080`). Fixed in this commit.
- `achieved_flight_s_fsm` read **0.041 s / 0.138 s** — nonsense — on two cycles.
- **`Toss loop OVERRAN` on every cycle**, worst host-loop stall **325 ms**. Nothing gapped the wire,
  but the FSM loop is not keeping its period.
- `/cache_diag decode_bad_axis` ≈ **493 700 and climbing at ~14/s** — an ongoing decode fault nobody
  has attributed.
- One **86 ms** encoder cache-age spike on leg 5.
- **95 of 191 record fields null** until the miner runs. Expected.

---

## Discussion

### (a) The catch never had a chance, and that is two independent failures stacked

It is tempting to read *"6 of 7 caught"* as a pass with rough edges. It is not. A planned catch means
the cup is **moving down at 0.7 × the ball's speed** when they meet, so the impulse is small and the
runway absorbs the rest. What happened is the opposite: a **3730–4157 mm/s** closing mismatch onto a
**parked** cup — ~4× the design speed, ~17× the design energy — absorbed by 2–7.4 mm of slider and a
20 A spike. The ball stayed in because a cup is a bucket. **That is not the mechanism under test, so
the headline number cannot be read as validating it.**

The lateness decomposes into two terms, both faults, neither the planner's arithmetic:

- **Open loop.** The throw left ~11 % fast, so the ball came home late. The plan is a feedforward
  object: it commits to a catch instant at solve time.
- **Closed loop.** The answer to *"the ball is not where the plan said"* is `catch/dynamic_target` →
  `REPLAN`, and that channel published nothing — held shut by one conjunct in one boolean.
  **Every other piece of the path was built, tested and correct**: `trajectory_node` bypasses
  `catch_armed` under unified mode, the route to `_replan_cycle_from_target` is live, and
  `replan_tail` was generalised to rest-terminal plans on 2026-09-05 *specifically* so it would not
  be dead code. `open_loop`'s `pretilt_hold` term is a legacy reload concept a unified session raises
  by accident of shared choreography.

The test is the sharper lesson. `test_unified_cycle_integration` covers the replan route thoroughly —
it just never calls `_on_pretilt_hold(True)`, so it drives a coordinator in a state a real session is
*never* in. **A test that exercises a path in a state the machine never occupies is a test of a
hypothetical.** The fix carries a companion test that raises the hold first.

And plainly: **a fully working replan would not have rescued this sitting**, because QTM was binding
the ball to a stale rigid body for most of each flight. Two independent faults on one path is why
"6 of 7" happened at all — feedforward was the only mechanism actually running.

### (b) The frame drop, and why a contract with three parts still missed it

`ros_ws/docs/levelling_frame.md` is a proper contract — normative document, enumerated manifest, and a
test that fails when the two disagree — enforced for four months. It missed the unified cycle
completely.

**The enumeration was of the wrong kind of thing.** Every row E1–E7 is *a pose coming in*: "an
external pose entered through this converter, so correct it here." **A cycle has no pose coming in.**
`PlanCycle.Request` carries cup **sites** and **velocities** — positions, which the correction never
touches by rule — and the attitude is *derived inside the planner* from ballistics: `tilt_to_throw`
of the take-off velocity, `tilt_to_receive` of the observed arrival, `tilt_to_receive` of the
apparent-gravity field `g − a_cup`. Nothing on the request looks like an orientation, so nothing
tripped the question the contract knew how to ask. Restated as step 0 of the doc's "adding a new pose
surface" checklist:

> The enumeration is not *"poses that enter"*. It is **"commanded rotations that leave"**.

The second half of the lesson is the *shape*. A uniform missing correction would be a constant
attitude offset — visible, boring, probably caught in sim. What shipped was **half a correction**:
knot 0 in the plan frame beside release, catch and banking pins in the gravity frame, with a smoother
between them. The observable is a **step** — 0.669° over the pre-throw window — which reads like a
controller transient and not at all like a frame error. The operator saw it and called it a small
tilt step. Uniform, he would have seen nothing and the ball would still have flown 23 mm off.

Both halves are pinned now: the manifest gains `build:E8` / `egress:E8` / `apply:E8`, and
`test_every_apply_has_a_build_in_the_same_scope` resolves cross-scope pairings through
`_CARRIED_BUILDS` **and checks the E-numbers match**, so a build and an apply from different rows can
no longer satisfy each other; `test_carried_builds_names_only_live_manifest_rows` stops the excuse
table outliving the rows it excuses.

### (c) Why the correction rides the state and the meta, not a `plan_cycle` kwarg

The obvious implementation is a `correction=` kwarg on `plan_cycle`. It was written and **rejected**,
for two reasons that only appear past the first call:

- **`extend` and `replan_tail` would be uncorrected.** They do not go through `plan_cycle` — they
  build from a previous plan's meta. A kwarg reaches a session's first window and nothing after it,
  so a chained or replanned cycle would fly the exact bug this entry is about, *intermittently*, only
  on the windows that chained.
- **Re-deriving per window disagrees at the seam.** The correction is a function of pose, and the
  pose at a seam is not the pose at the origin, so two windows re-deriving independently pick up the
  **tilt map's gradient** as a discontinuity at exactly the knot where continuity is load-bearing.

So it is **built once per cycle, at the seed** (`trajectory_node._cycle_start_state` calls
`levelling.correction_for_pose(self._gravity_offset, self._active_tilt_map(), pose)`, `TiltMapError`
degrading rather than raising) and then **carried**: in on `CycleState.levelling_correction`, out on
`CycleMeta.levelling_correction`, chained by `release_state_from_meta`, read off the meta by
`replan_tail`. `_joined_correction` **refuses a splice whose halves disagree**, turning "the frames
drifted apart across a join" from a silent wrong throw into a loud refusal.

That the build and the apply now live in **different scopes** is exactly why the manifest needed
`_CARRIED_BUILDS` and the E-number check: every previous row built and applied inside one function,
and the machinery that verified that could not express this shape at all.

### (d) Why the announcement carries the nominal velocity and the command carries the trim

The session trim is now honoured on the unified launch (owner, 2026-09-06): the ILC fitted
**−10.76 %** for this plant, which is most of the throw excess.

Wiring it raised a question easy to get backwards. `_unified_cycle_request(vel_trim=)` scales the
**commanded take-off** by `(1 + trim)`, leaves `flight_s` and `throw_site` alone, and re-derives the
target through `ballistics_bc.position_at`. But `_announce_unified` **divides the trim back out** and
announces the **nominal** velocity. That looks like discarding information. It is not:

- **The command is about the machine.** The trim corrects *this plant's* tendency to over-deliver, so
  the number the hand is told must be the corrected one.
- **The announcement is about the ball.** The catch coordinator, the tracker's expectations and the
  session record all care what the ball will *do* — and the trim exists precisely because the ball
  does the nominal thing when the machine is commanded the trimmed thing. Announcing the trimmed
  number would tell every consumer the ball is 10.76 % slow, which is what the trim prevents.

It also **mirrors legacy**, where `release` is uncorrected and `event_vel` carries the trim, so a
reader who knows one path reads the other correctly. Backwards, it would have made the catch worse in
a way the throw telemetry would have called correct.

The channel matters too, and the old one is a trap: `speed_bias_applied` (`SessionTrim.speed_gain`)
has been **retired to monitor-only since 2026-08-21**. The live channel is the ILC artifact's
`aim['ilc_vel_trim']` — loaded at node start behind a `toss_ilc_enabled` param and a provenance
match, bounded ±0.15, gated by `_ilc_vel_trim_refusal`. **A sitting sets the trim by dropping the
fitted artifact**, not by turning a knob.

### (e) The driver told the operator wrong things, three times

Three of the sitting's "failures" were the bench driver, not the machine:

- **V6 FAILed on every rung** on the wrong predicate. Corrected to `plan_time_remaining_s ≤ 0 AND
  cycle_active` — `cycle_active` is a *type test on the installed plan* and stays true after the
  window ends (a cycle that has run out is still the plan holding the pose), so watching it go false
  is watching for something that never happens.
- **V2 FAILed at a bare 500 ms**, a bar sized for a single window. A **joined** install is two solves
  plus the join's own re-validate; 500–600 ms is healthy. The bar is now `UNIFIED_PLAN_BUDGET_MS`
  **1200** (the coordinator's `_UNIFIED_PLAN_BUDGET_S`, pinned as a literal because the driver must
  not import ROS at module scope, with a drift test holding the two together), **ADVISORY** at 700 ms,
  the 250 ms owner budget kept for a single window.
- **V7 SKIPped on a typo.** Now reads `plan.total_duration` and prints `repr(exc)`, so the next typo
  announces itself.

A fourth was right but scored against the wrong metal. V3 now uses `HAND_METAL_REV_MEASURED`
**10.701** — the owner's bench number — with provenance, not the firmware clip. **The clip 10.8 is
0.099 rev (3.2 mm) PAST the metal**, and no firmware guard can see a stall in that gap: the deviation
guard compares encoder to command (which agree once the slider is jammed at the clip) and the lead
clamp *anchors* the setpoint to the encoder rather than refusing it. New **V2b** watches
`max_emit_gap_ms ≤ 125` — half the watchdog — so the morning's E-STOP class gets an in-band warning
instead of a post-mortem.

**A refusal the operator cannot trust is worse than no check** — the same argument as the morning's
inverted "NOT level" warning. Three FAILs the operator is told to ignore is how a real FAIL gets
ignored.

### (f) The geometry measurement, and why it is a separate change

The owner measured the hand on the bench: **stroke 352 mm, bottom −0.107 rev, top 10.701 rev ⇒ 32.57
mm/rev**. The planner uses **31.628**. That is **+2.98 %** of hand travel the machine has and the
model does not, straight into the throw as excess release velocity.

The wrong part is not a typo, it is a **fudge factor**: `linear_gain_factor: 1.035`, commented
*"Multiplier on linear gain ('just 'cuz' factor)"*. And the geometry key beside it,
`hand_stroke_mm: 344.75`, is documented as *"a measured fact"* and then **derived in its own
comment** — `(10.8 + 0.1) × 31.6284 = 344.7496`. It is an inference from the old hard-stop anchor.

**This is deliberately NOT fixed here.** The blast radius crosses control, sim, geometry and docs
(enumerated in the plan's new § "Hand geometry correction") and it moves numbers on paths this sitting
did not fly. Two findings make that concrete: the flown `x2 = 5.9138 rev` is physically **192.6 mm**,
not the 187.0 the model believes; the catch prime `9.9594 rev` is physically **324.4 mm**, not 315.
**The robot has flown those positions successfully for months at the wrong nominal**, so the
correction must be paired with a **bench re-validation of the legacy release and catch points before
it ships** — and landing it inside a commit whose job is a frame drop and a replan gate would make
both unbisectable. It is also only **~3 %** of a **~11 %** excess: fixing it will not close the throw
error, and shipping it as though it would makes the *next* measurement unreadable.

### (g) Host jitter is exonerated; the firmware buffer is sequenced after the planner fixes

The morning's E-STOP made the Jetson's scheduling the prime suspect for anything time-shaped. The
evening's numbers close it: `max_emit_gap_ms` p50 **26.0** ms, max **35.4** ms, **zero** over 40;
`interp_deadline_misses` **0**; jitter **≤ 3 µs**; zero TX deferrals; setpoint age 8–22 ms;
`blas threads: 1` on all three planner nodes. **The wire was clean all evening.** The catch is
146–195 ms late against a stream whose worst hiccup was 35 ms — two orders of magnitude apart, and no
amount of scheduling work moves it.

The long-term shape — **a firmware knot buffer, whole-plan upload, so the Jetson's scheduling is off
the wire's critical path entirely** — is still the right destination, and the owner has sequenced it
**after** the planner fixes (2026-09-06). The root-cause reason, not the schedule one: uploading a
whole plan makes the plan **harder to change mid-flight**, and this sitting's headline finding is that
the plan *must* change mid-flight. Building the buffer before the closed loop works would optimise the
wrong end and make the replan path harder to land afterwards.

### (h) The QTM blindness is the one thing the Jetson cannot fix

For **5 of 7** throws QTM bound the flying ball to a stale `Catching Cone` rigid body **60–80 ms after
release** — the ball stopped existing as a ball for the rest of the flight, which is exactly the
window a catch replan reads. The raw marker track is complete throughout, so this is project
configuration, not markers or occlusion; the seven phantom `human_throw` balls from static Ball-Butler
reflectors are the other half of the same class. Memory records the Cone body being disabled on
2026-08-29. **It is back.**

Neither is fixable from the Jetson side, and both are now runbook preconditions (2c) rather than
surprises. Until they are, the replan path cannot be evaluated even with its gate open — which is the
honest reason the replan fix here is *not* claimed to be validated by anything but tests.

---

## Fix

Two independent fix sets land together, in one commit, because both are on the path the next sitting
flies.

### A — the levelling frame: contract row E8

1. **One application point, in the planner, between `tilt_schedule` and `decompose`**
   (`unified_cycle._realize`). The seed's knot-0 pin is taken **plan → gravity** first
   (`_tilt_to_gravity` → `levelling.uncorrect_pose`, on the `post_release=False` branch **only** — the
   chained case is in free fall and the banking objective is degenerate there), the schedule is then
   built entirely **single-frame**, and the finished series is taken **gravity → plan** as a whole
   (`_tilts_to_plan`, one `levelling.correct_pose` per knot on a position-free 6-vector). Knot 0 is
   **written back into the seed's own float**, gated on `start_tilt is not None` — a bug the
   companion test suite caught before it flew.
2. **The correction is built once, at the seed** — `trajectory_node._cycle_start_state` calls
   `levelling.correction_for_pose(self._gravity_offset, self._active_tilt_map(), pose)`, with
   `TiltMapError` routed to the existing degraded path rather than raised.
3. **It is carried, not re-derived** — `CycleState.levelling_correction` in,
   `CycleMeta.levelling_correction` out; `release_state_from_meta` chains it; `replan_tail` reads it
   off the meta; **`_joined_correction` refuses a splice whose halves disagree.** Rationale in
   Discussion (c).
4. **The contract** (`ros_ws/docs/levelling_frame.md`, **+200 lines**): a new
   § *"E8 — the unified cycle, and how it escaped the enumeration for four months"*; an E-table row
   *"E8 | `trajectory/plan_cycle` — the unified cycle's tilt schedule (`CyclePlan`) | built in
   `_cycle_start_state`, applied in `unified_cycle._tilts_to_plan` (via `_realize`) |
   `unified_cycle.plan_cycle` / `extend` / `replan_tail` — not a `planner.build_*` entry at all"*;
   Enforcement rows; a **step 0** in "adding a new pose surface" that asks the E8 question first; and
   the doc's stale *"three kinds"* corrected to **four**.
5. **The manifest** gains `build:E8` / `egress:E8` / `apply:E8`;
   `test_every_apply_has_a_build_in_the_same_scope` now resolves through `_CARRIED_BUILDS` **and
   checks the E-numbers match**; new `test_carried_builds_names_only_live_manifest_rows`.

**Measured (probe, 2026-09-06, on the sitting's own offset):**

| | before | after |
|---|---|---|
| Knot 0 commanded | **−11.663 mrad** | −11.663 mrad |
| Release commanded | **0** (mechanical zero) | **−11.663 mrad** (= the correction) |
| Release physical | **+11.663 mrad about −y** | **0.000000 mrad** |
| Knot 0 → release step | **0.6682°** | **0.0000°** |
| Lateral drift at a 0.5 m apex (`4·h·sinθ`) | **23.325 mm** | **0.000 mm** |
| Knot-0 continuity drift | — | **0, exact** |

`correction=None` and an identity correction are **bit-identical to pre-fix**, so nothing that does
not carry a map changes at all.

**Residuals documented, not introduced.** The `rz` projection is worst **1.221e-3 rad**, of which the
part that reaches the cup axis is **1.276e-4 rad** — **91× below the error just fixed**. The cup lever
arm is **1.349 mm**, identical before and after to **1e-9 mm**; closing it needs a `cup_realize`
change and was **declined** for this commit (Open Question 10).

Files: `unified_cycle.py` **+303/−12**, `trajectory_node.py` **+71/−4** (in `_cycle_start_state`),
`levelling_frame.md` **+200**, `tests/motion/test_unified_cycle.py` **+383** (11 tests),
`tests/ros/test_levelling_frame.py` **+95/−9**, NEW `tests/ros/test_unified_cycle_levelling.py`
**530 lines** (10 tests).

### B — the catch path, the trim, the timeout hazard, the driver and the runbook

6. **The replan gate opens under unified mode.** `catch_coordinator._on_balls`'s `open_loop` gains
   `and not self._unified_mode`.
7. **A movement gate on the published target**, `_UNIFIED_TARGET_MOVE_MM = **5.0**` — applied to the
   **published** `target_pos`, keyed on `ball_id` so **a new ball's first estimate is never filtered**,
   and cleared on the disarm edge. A replan is a ~230 ms in-line solve; it must be spent on real
   movement.
8. **The cup/centroid lever is applied at the consumer.** `trajectory_node._replan_cycle_from_target`
   now sets `catch_site.xy = target_pos.xy + HAND_CATCH_OFFSET_MM · cup_axis(quat)` — **the
   coordinator's own lever** (`64.78 mm` ⇒ **13.4685 mm** at 12°, `+ry→+x`, `+rx→−y`), the exact
   inverse of the compensation the message already carries. Using the planner's height-aware lever
   instead would be **17.8 mm** at 830 mm and land the cup **4.35 mm off the ball**; the two coincide
   at nominal because `GEOM_INITIAL_HEIGHT + ACTIVE_Z = 744.3 = CUP_TILT_CENTER_Z_MM`.
9. **The lying-ack hazard is closed.** `reload_coordinator._call_plan_cycle` returns
   `(response, dispatched)`; `_SERVICE_WAIT_S` is **2.0 s**, which is *below* the measured 2.02–2.16 s
   solves, so an UNACKED call is a real possibility. On UNACKED, `_hold_after_unacked_plan()` calls
   **`trajectory/hold` first**, then mints a terminal
   `REJECTED_PLAN_SERVICE(UNACKED: … a plan MAY have been installed — held/STILL STREAMING)` at ERROR.
   A genuinely UNAVAILABLE service keeps the honest *"nothing was commanded"*. `EXTEND` gets the same
   treatment.
10. **The session speed trim is honoured on the unified launch** — `_unified_vel_trim(state)` reads
    the ILC artifact's `aim['ilc_vel_trim']` block; `_unified_cycle_request(vel_trim=)` scales the
    commanded take-off by `(1 + trim)` with `flight_s` and `throw_site` unchanged and the target
    re-derived via `ballistics_bc.position_at`; wired into **launch AND extend**. `_announce_unified`
    divides the trim back out and announces the **nominal ball velocity** — rationale in Discussion
    (d). Probe numbers in that section.
11. **The record stops lying about the catch knob.** `catch_knobs(unified=)` reports the legacy keys
    as `None` and names `catch_slider_vel_ratio` **0.7**.
12. **The driver**: V6 = `plan_time_remaining_s ≤ 0 AND cycle_active`; V2 against
    `UNIFIED_PLAN_BUDGET_MS` **1200** with an **ADVISORY** at **700** and the **250** ms single-window
    budget retained; new **V2b** `max_emit_gap_ms ≤ 125` (half the watchdog, CSV column added);
    `--timeout-s 2.0` pinned to `_SERVICE_WAIT_S`; V7 fixed (`plan.total_duration`, `repr(exc)` in the
    SKIP); **V3 scores against `HAND_METAL_REV_MEASURED` 10.701** with provenance (sitting two's
    10.4693 ⇒ 0.2317 rev / 7.3 mm of margin) and the clip-past-metal / no-guard-sees-a-stall comment;
    a `stage_wall_note` explaining why `cont ≈ val` on a join.
13. **The runbook**: precondition **2c** (start QTM first; disable the `Catching Cone` body; mask the
    Ball Butler's reflectors), the UH-5 encoder numbers (9.76–9.87, ~0.8 rev to the **measured** metal,
    clip watch), the *"cycles 2 and 3 are extensions"* claim corrected, and the
    `cycle_active` / `plan_time_remaining_s` and `cont` notes.

---

## Verification

- **Levelling scoped gate** (2026-09-06, `python -m pytest tests/motion/test_unified_cycle.py
  tests/ros/test_levelling_frame.py tests/ros/test_unified_cycle_levelling.py -q`) — **306 passed in
  55.98 s**.
- **Both fix sets together** (2026-09-06, the same command widened to agent B's files) — **485 passed
  in 51.21 s**.
- **Fix set B's own scoped run** (2026-09-06) — **624 passed / 1 failed in 47.93 s**; the single
  failure was agent A's file **mid-edit** and is superseded by the 485-pass combined run above.
  `py_compile` clean on every touched module; both `--dry-run` rungs exit **rc 0**.
- **Unified sim gate** (2026-09-06, `python sim/unified_gate.py --no-viewer`) — **PASS in 73.1 s**:
  SET 1 `core_clean` **26/26**, SET 2 beat **EXACT**, **0** lead-clamp ticks.
- **Cycle gate** (2026-09-06, `python sim/cycle_gate.py`) — **PASS**, **11/11**, parity **EXACT**.
- **Levelling probe** (2026-09-06, `/tmp/probe_level.py`, the sitting's own gravity offset and map
  `2026-08-10-3bf7964f`) — the before/after table in § Fix A; `correction=None` and identity
  **bit-identical** to pre-fix.
- **Trim probe** (2026-09-06) — trim −0.1076 at 860 mm ⇒ ratio 0.8924, Δtarget z **−131.9 / −189.9 /
  −337.6 mm** at flight 0.5 / 0.6 / 0.8 s, direction unchanged, **trim 0 bit-identical**.
- **Hardware artefacts**, all 2026-09-06: `temp/logs/cycle_ladder_20260906_193029.log` (Teensy
  console); `~/Desktop/rosbags/2026-09-06_19-*` (two MCAP bags, split by the ~19:32 relaunch);
  `temp/logs/unified_cycle_bench_{carry_193247,carry_193313,throw_193345,throw_193400,throw_193407}*`
  (driver CSV + meta); miner output under `temp/probes/`.
- **Full gate** (2026-09-07, `./run_tests.sh --full`, parallel **6875 passed / 4 skipped / 2 xfailed in 559.09 s**, serial **4 passed in 27.20 s**, total 593 s, exit 0) — **GREEN**, on the tree carrying this entry's fixes, the 2026-09-07 launch-floor fixes and the conftest OOM fix. Two earlier runs of this gate wedged at 27–44 min with an xdist worker OOM-killed (2.43 GB) — the cause was a `MagicMock` `rclpy.ok` in `tests/ros/conftest.py` recording millions of calls under `while rclpy.ok()` waits with a no-op `time.sleep`, latent since the mock existed; fixed in the same commit (see the 2026-09-07 entry).

**Load-flake note.** The replan/extend test family in `tests/ros/test_unified_cycle_integration.py` is
**wall-clock sensitive by construction** — under two concurrent suites it reported a `STALE_STATE`
hand drift of **9.17 rev** and failed 1 of 7, then passed **8/8** on a clean worktree. This is the
τ-overrun class already recorded on 2026-09-06 (that entry's Open Question 9). **The gate runs alone
by rule**, which is the condition these numbers were taken under. Do not widen the tolerances.

---

## Outcome

**UH-3 PASS. UH-5 PASS. UH-6 FLOWN — and its catch quality FAILS T-H6.** UH-7 was not run.

The machine did the thing: a whole cycle, platform and hand on one plan and one clock, throw and catch
and settle, six times of seven with a ball. The wire never hiccuped, the guard was armed and never
tripped, the morning's E-STOP class did not recur, and the tightest hardware margin of the evening
(2.8 mm of encoder to the band ceiling) held.

**But the catch that was validated is not the catch that was designed.** T-H6's release-velocity
criterion fails on every method that measures it — the throw left **+5.2…+16.7 %** fast — and the
catch's own design ratio missed by two orders of magnitude (**0.001–0.059** against **0.7**), because
the receive stroke completed 172–185 ms before the ball arrived and the cup was parked when it did.
Six catches happened; **zero planned catches happened.**

**Both convictions are fixed in this commit and NEITHER IS RE-FLOWN.** The frame drop is closed at the
root, with the contract extended to the class of surface that hid it; the replan gate is open, the
target lever applied at the consumer, the movement gate in, the trim wired, the lying-ack hazard
closed, the driver honest and the runbook carrying the QTM preconditions. **None of that is
hardware-verified.** The next sitting is the verification — and it needs the QTM project fixed first,
because until the `Catching Cone` body is disabled and the Ball Butler's reflectors masked the tracker
is blind for most of every flight and the replan path cannot be evaluated at all.

**Re-fly 2026-09-07:** with the E8 levelling fix, the replan wiring and the launch-floor lift all in
place, UH-6 flew cleanly (operator report; no bag analysed yet) — catch quality against T-H6's
release-velocity criterion remains to be measured from the next bag.

---

## Withdrawn claims

- **"Unified mode gates the dynamic target."** The analysis agent's first hypothesis for the silent
  replan channel. **WITHDRAWN on trace:** `trajectory_node:~3087` explicitly *bypasses* `catch_armed`
  under unified mode and `~3145` routes to `_replan_cycle_from_target`. The consumer was ready; the
  blockage is one conjunct in the coordinator's `open_loop`, raised by a legacy pre-tilt hold with no
  unified branch. **Superseded by:** § Diagnosis 4.
- **"There is a release/catch height asymmetry worth ~50 ms."** **WITHDRAWN on measurement:** it is
  **~9 ms**, which is inside the noise of every estimator in § Diagnosis 1 and explains nothing about
  a 146–195 ms lateness.
- **"The mocap track is unfittable."** **WITHDRAWN:** a labelling artefact. The raw marker track is
  complete; what breaks is QTM's *binding* of the ball to a stale rigid body (§ Diagnosis 9), and the
  parabola fits cleanly once the labelling is read correctly. This matters, because "unfittable"
  would have removed the sitting's cleanest catch-timing estimator.
- **"Cycles 2 and 3 are extensions of the first plan, ~200 ms."** The runbook said so.
  **WITHDRAWN on the bag:** at this dwell every cycle comes to rest, so every cycle is a fresh
  **joined** LAUNCH+LANDING solve — 500–606 ms, no `EXTEND` fired all sitting. Corrected in the
  runbook in this commit.
- **"`hand_stroke_mm: 344.75` is a measured fact."** The YAML comment has said so since 2026-08-18 —
  44 hours. **WITHDRAWN:** the same comment then *derives* it, `(10.8 + 0.1) × 31.6284 = 344.7496`.
  It is an inference from the old hard-stop anchor, not a measurement. The bench measurement taken
  2026-09-06 is **352 mm**, with the bottom at −0.107 rev and the top at 10.701 rev ⇒ **32.57
  mm/rev**. The correction is its own planned change (Open Question 4).

---

## Open Questions

1. **Where is the rest of the throw excess?** The bench gain correction (32.57 vs 31.628 mm/rev) is
   **+2.98 %** of an excess measured at **+5.2…+16.7 %** (mean +10.9 %). Two independent bag
   estimators put the *empirical* cup travel at **34.0–34.5 mm/rev**, which would explain all of it —
   but that disagrees with the bench measurement by 4–6 %, and one of the two is wrong. Until it is
   resolved, the ILC trim is compensating for a mechanism nobody has named. **Do not ship the geometry
   correction as though it closes this** (Discussion (f)).
2. **QTM — operator action, and a hard precondition.** Disable the `Catching Cone` rigid body and mask
   the Ball Butler's reflectors before the next sitting. With them live the tracker is blind for most
   of every flight (5/7 throws) and mints phantom balls, so **the replan fix cannot be evaluated at
   all**. Runbook precondition 2c.
3. **The FW 18 bundle** (owner, 2026-09-06). Five items, one flash:
   (a) the hand clip becomes **stop − 0.2 rev** with the YAML stop set to the **measured 10.701**
   (today's clip 10.8 is 3.2 mm past the metal, in a gap no guard can see);
   (b) **homing restores** `POSITION`/`PASSTHROUGH` and the shipped velocity/current limits on axis 6
   (`leg_homing.cpp:195` leaves it in `VELOCITY`/`VEL_RAMP`, where `CLOSED_LOOP` alone still swallows
   the stream, invisible to every telemetry gate);
   (c) the `lead` / `dev_over` counters gain the missing **`s_output_enabled`** term, so they stop
   being unfalsifiable (§ Diagnosis 8);
   (d) a **`hand7 reset`** verb, so the counters can be zeroed without a Teensy reboot;
   (e) **`MPC_STALE` → `SETPOINT_STALE`** — 158 sites across 60 files, driven by one generator entry.
4. **The hand geometry correction is its own planned change** (owner, 2026-09-06). Measurement,
   blast radius, the flown-position audit and the acceptance criterion are enumerated in
   `plans/archived/unified-7dof-planner.md` § "Hand geometry correction". **The acceptance is a bench
   re-validation of the legacy release and catch points before it ships** — the machine has been
   flying `x2 = 5.9138 rev` (physically **192.6 mm**, not 187.0) and a catch prime of `9.9594 rev`
   (physically **324.4 mm**, not 315) successfully for months.
5. **2.8 mm.** The worst encoder peak this sitting, **9.8699 rev**, sits **0.089 rev** below the
   9.9594 rev band ceiling, with an overshoot past the commanded peak of **+0.101…+0.220 rev** that is
   **uncorrelated with peak velocity** — so it is not a simple coast term and there is no model for it.
   This is the tightest number of the sitting and the first thing to watch if the throw height rises.
6. **`achieved_flight_s_fsm` read 0.041 s and 0.138 s** on two cycles. Physically impossible; the
   field is being computed from the wrong pair of timestamps. Harmless today because nothing consumes
   it, which is exactly why it will still be wrong when something does.
7. **`decode_bad_axis` ≈ 493 700 and climbing at ~14/s** on `/cache_diag`. Unattributed. It has been
   noted since Phase 3 sitting two at ~2/s; it is now 7× that rate and nobody has found the source.
8. **`Toss loop OVERRAN` on every cycle**, worst host-loop stall **325 ms**. It gapped nothing this
   sitting (`max_emit_gap_ms` max 35.4 ms), but a 325 ms stall in the FSM loop is 13 emitter periods
   of margin the design does not know it is spending.
9. **The replan/extend test family is wall-clock load-sensitive** (§ Verification). It belongs to the
   τ-overrun class already tracked; the fix is to freeze every τ source in those tests, not to widen a
   tolerance. The gate runs alone, so it is not currently a gate risk.
10. **The cup lever-arm residual, 1.349 mm.** Identical before and after the E8 fix to 1e-9 mm, so E8
    neither caused nor closed it. Closing it needs a `cup_realize` change and was declined here to keep
    the frame fix bisectable. It is ~10× smaller than the error E8 removed and ~10× larger than the
    `rz` projection residual, which makes it the next one in the queue.
11. **The firmware knot buffer / whole-plan upload is the long-term shape, and it is sequenced AFTER
    the planner fixes** (owner, 2026-09-06). **Host jitter is exonerated for this sitting** — the wire
    was clean end to end — so the buffer is not a fix for anything observed here. The root-cause reason
    to sequence it late: a whole-plan upload makes the plan harder to change mid-flight, and this
    sitting's headline finding is that the plan **must** change mid-flight.
