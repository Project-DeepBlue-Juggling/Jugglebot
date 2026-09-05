---
title: "Third FW 17 hand sitting — the falling-edge decay lands bit-exact, the hold's 3 mm deviation turns out to be the clip floor, and the armed trip is closed unobserved"
type: investigation
date: 2026-09-05
status: resolved
phase: "unified-7dof-planner — Phase 3 (third hand bring-up sitting)"
related_plan: unified-7dof-planner.md
files_changed:
  - tests/hardware/session_unified7_hand_bringup.md
  - plans/active/unified-7dof-planner.md
  - plans/active/INDEX.md
  - logbook/2026-09-04-hand-bench-moving-gap-stage.md
  - logbook/2026-09-05-fw17-hand-ladder-sitting-three.md
  - logbook/INDEX.md
subsystem:
  - can
  - tools
tags:
  - safety
  - testing
  - docs
---

# Third FW 17 hand sitting — the decay rule confirmed, and two reframings

## Summary

Successor to
[`2026-09-04-fw17-hand-ladder-sitting-two`](2026-09-04-fw17-hand-ladder-sitting-two.md).
Four carried items, all discharged, in one 5-minute bench block (13:56–14:01,
launch down, `hand_stream_bench.py` the sole UDP owner, hand `axis_state ≡ 8`
throughout, `fault ≡ 0`, no guard trip, no abort).

**The headline is that FW 17's NORMATIVE falling-edge decay rule is CONFIRMED
on hardware, bit-exact against the closed-form prediction.** Row 17b's
`moving_gap` took a 250 ms knot gap mid-ramp at 0.500 rev/s; the firmware's own
target coasted **+0.05250 rev (1.66 mm)** past the last knot and froze —
against `v·0.105 = +0.05250` predicted, `|coast − DECAY| = 0.00000`, four times
the tolerance away from the forbidden hold-at-endpoint mode and six times away
from no-wind-down. G1–G5 all PASS, re-derived independently from the CSV.

**This project also finally holds a bracketed `[hand7]` capture**
(`temp/logs/hand7_console_20260905_135613.log`, 313 blocks at 1 Hz, opened
before the first row and closed after the last — sitting two's open question 7).
Every stage's counter deltas are in it, and they are clean: `lead 0`,
`dev_over 0`, `unseen 0`, `stale 0` on **every** stage and across the whole
sitting.

Two things came out that did not go in, and both are reframings rather than
results (§ Discussion):

1. **The 30 s hold's 0.0995 rev "deviation" was the firmware's `[0, 10.8]`
   stroke clip, not tracking error.** The hand was parked at −0.0976 rev; the
   firmware clipped the transmitted setpoint to exactly 0.0 and the hand went
   there. Both belts read the difference as deviation. Real tracking against
   the transmitted target was 0.0003 rev.
2. **`hand7 arm` WAS issued** — the console shows `guard=ARMED` from ≈13:59:25
   to the end, so row 18's restraint stage ran **armed**, with zero nuisance
   trips at a 0.2515 rev peak. But the operator has **closed row 18's arming
   half permanently** on thermal grounds, so the E-STOP itself has still never
   fired on hardware. Whether Phase 5 flies armed was an owner decision at the
   time of writing; **taken the same day: ARMED from UH-3 on, a trip is data**
   ([`2026-09-05-unified-7dof-phase5-prep.md`](2026-09-05-unified-7dof-phase5-prep.md)).

## Results, row by row

Times are from the console capture (`[guard] mpc_active` / `sp_age_ms` edges,
anchored on the `script` header; the monitor ran without `-f time`, so there
are no per-line stamps). They cross-check against each stage CSV's filename to
≤ 1 s. The operator's transcript times are shell-prompt times — i.e. the *end*
of the preceding stage — which is why they read 20–90 s earlier.

### Row 19(b) — v6 hand stream against the LEGACY latch → **PASS**

`--source-only legacy` (established 13:38), then `--stage hold --duration 3
--no-source-switch --close-loop`, streaming 13:56:47–13:56:50.
`hand_stream_hold_20260905_135648.csv`, 120 rows.

**`discard_legacy` delta = 222**, 0 → 222 across the bracket, and it never
moves again for the remaining 275 console blocks. **It counts FRAMES**:
`leg_interp.cpp:286-289` increments once per accepted Setpoint frame whose
`HAS_HAND` flag is stripped, inside the frame decode, not per 500 Hz tick.
Predicted **223** = 103 pre-arm hold-pump frames (the driver's own
`ARMED — streaming (hold thread sent 103 frames)` line — *that* is where the
operator's "103 frames" comes from, not the stage) + 120 stage frames
(3.0 s × 40 Hz). Observed 222, one short; the same window logged +3
`seq_gaps`, so a single lost datagram covers it. A tick-counting counter would
have read ≈2 788, so the frames-vs-ticks question the runbook left open is
settled.

Three independent proofs the hand channel never reached the wire:

| Surface | Reading | Why it is proof |
|---|---|---|
| `[hand7] sent` | delta **0** | The lane transmitted nothing at all. |
| CSV `echo_rev` | **empty on all 120 rows** | No `HAND_CMD_ECHO` was emitted, because none was TXed. |
| CSV `enc_rev` | span **0.00030 rev = 0.009 mm** | With `axis_state ≡ 8` the hand was ENERGISED and did not move — which is the whole point of running this row energised. |

Driver belt max 0.0010 rev (0.03 mm); legs unaffected; cachediag 7 windows,
0 episodes, worst −1.0.

### 30 s STREAMED hold → PASS, with the clip finding

Streaming 13:57:31–13:58:01, `hand_stream_hold_20260905_135732.csv`, 1200 rows.
Driver belt max **0.0995 rev (3.15 mm)**; cachediag 34 windows, 0 episodes,
worst deficit −11 (on leg 2). See § Discussion 1 — the deviation is the clip.
`[hand7]` delta: `sent` +15 091 (= 500 Hz × 30.2 s), everything else 0.

### Row 17b — `moving_gap`, the falling-edge decay → **PASS (G1–G5)**

`--stage moving_gap --duration 10 --close-loop`, streaming 13:58:40–13:58:50,
`hand_stream_moving_gap_20260905_135840.csv`, 400 rows, `lead_mask ≡ 0`.
Falling edge at knot 80 (t = 2.000 s), 9 knots withheld (CSV rows 80–88 read
the literal `withheld`), firmware gap 250 ms nominal / **249.4 ms measured**,
knot velocity **+0.500 rev/s**. Re-derived from the CSV; every number matches
the driver's own verdict.

| # | Criterion | Re-derived | Verdict |
|---|---|---|---|
| G1 | coast nearest & within tol of DECAY | **+0.05250 rev (1.66 mm)** at g = 225 ms vs DECAY +0.05250 / HOLD +0.01250 / NONE +0.11250, tol ±0.0100. `\|coast − DECAY\| = 0.00000` | **PASS** |
| G2 | target frozen past the 135 ms wind-down | 4 samples, all **+1.04007**, span **0.00000 rev** | **PASS** |
| G3 | encoder tracks the DECAYED target | worst `\|enc+vel·age − model\|` **0.0214 rev = 0.68 mm** (bar 0.100), at g = 125 ms, encoder *ahead* of the target | **PASS** |
| G4 | re-entry step bounded | **+0.0725 rev = 2.29 mm** vs predicted +0.0725, bar ±0.25; echo moved +1.04007 → +1.12453 on the next knot | **PASS** |
| G5 | `lead_clamp_mask` bit 6 never set | 0 of 400 sampled ticks; `[hand7] lead` delta 0 | **PASS** |

**The three-mode ladder is legible in the echo column**, which is what makes
this a confirmation rather than a fit: +0.0125/knot through Mode 2 (constant
velocity), +0.0085 then +0.0065 as Mode 3's decay bites, then bit-identical
from g = 150 ms. The closed form is exact here because the pump hard-zeroes
`accel[6]`, so the firmware's jerk EMA is identically 0 and every cubic term
vanishes — a disagreement would have convicted the firmware, not the model.

**One thing the five criteria do not cover, and it matters:** `[hand7] sent`
advanced by **exactly 500 in every full second of the stage, including the
second containing the gap**. The lane keeps transmitting its decayed target
across the falling edge rather than going quiet — the other half of the
normative rule, and nobody had checked it. Encoder excursion across the
withheld window was +0.0587 rev (1.86 mm) against the target's +0.0525
(1.66 mm): 0.2 mm of momentum overshoot, which is also what G3's 0.68 mm
worst-sample is.

Driver belt max 0.0591 rev (1.87 mm); cachediag 13 windows, 0 episodes, worst
deficit 0.0.

Independently reproduced from the CSV through the driver's own scorer
(2026-09-05, `python /tmp/probe_row17b.py`, which calls `evaluate_moving_gap()`
and `hand_gap_target()` off the recorded rows): gap **249.44 ms**, coast
**+0.052500**, freeze span **0.000000**, tracking **0.021418**, re-entry
**+0.072500** — all five, G1–G5 PASS. (The 249.44 is 0.56 ms under the CSV's
nominal 250.0 because `mg_origin` is stamped *after* `send_stream()` while the
row's `t` is stamped before it; inverting `pred_rev` through `hand_gap_target`
recovers that send latency exactly, and it shows up independently as a flat
`v0 × 0.56 ms` bias in `pred_rev`.)

### Row 18 — held-rotor deviation, re-flown ARMED → observe half PASS; arming half CLOSED BY OPERATOR DECISION

`--stage triangle --tri-speed 0.2 --tri-span 0.5 --close-loop`, streaming
14:00:19–14:00:51, `hand_stream_triangle_20260905_140020.csv`, 1200 rows.
**`guard=ARMED` for the whole stage.**

Peak residual **0.2515 rev = 7.95 mm at t = 10.30 s** (driver belt; the raw
`|cmd − enc|` at that instant is 0.2505, from cmd +1.0938, enc +0.8434) = **10.1 % of `MAX_DEVIATION_HAND_REV` 2.5** and 12.6 % of
`MAX_LEAD_HAND_REV` 2.0. Sitting two's peak was 0.1965 rev, so 28 % larger and
the same shape: 1 s worst bins climb 0.035 → 0.133 over t = 0–10 s, spike to
0.2515, then collapse to 0.018–0.043 for the remaining 19 s as the operator
releases. `|cmd − echo|` max 0.0203 rev, mean 0.0015 — the firmware target
tracked the streamed plan with no hand clip and no hand lead clamp.
`[hand7]` deltas `lead 0 dev_over 0 unseen 0 stale 0`, `sent` +15 091.

**The `−42`-frame cachediag episode is NOT the hand and NOT the restraint
peak.** It is on **leg 5** (`deficit_5 = −42.0`, `deficit_6 = −0.0`) in the
1.01 s window ending at stage t = 3.172 s. The hand's own worst deficit in the
whole stage was −5. And it coincides *exactly* with the sitting's only
`lead_mask` engagement — 52 rows reading **32 = 0x20 = bit 5, the LEG-5 lead
clamp**, in two runs spanning t = 2.05–3.725 s; bit 6 (hand) never set. So the
picture is coherent: leg 5's encoder broadcast dropped ~420 ms of frames, the
age-capped leg lead clamp engaged against the resulting anchor, and the hand
lane was untouched. `rx_depth_hwm_jb` flat at 9/256 and `cap_hits` 0 throughout,
so it is not the ring. Drops occurred without any restraint too (the
un-restrained 30 s hold logged leg 2 −11 and leg 5 −5), which is why this reads
as the known stationary per-axis class of `plans/active/leg-bus-frame-drops.md`
rather than a restraint artefact — but it landed inside the restraint window and
is 4× larger than any other, so the alternative is not excluded (§ Open
Questions).

### Close-out — two refusals and a hand-park

At 14:00:51 the hand sat at **+1.056 rev**, in neither rest band, so
`--source-only legacy` was **REFUSED twice** (`ERR_REJECTED`, the driver naming
the gate: *not settled at a rest position — retract [−0.20, +0.10] or
catch-prime 9.96 ± 0.10*).

**The driver's bands are correct and need no reconciliation.**
`hand_source.cpp:42-47` computes `at_retract` as
`pos >= Homing::HAND_ABS_POS_REV − HAND_SETTLE_BAND_REV` and
`pos <= JBOp::HAND_RETRACT_REV + HAND_SETTLE_BAND_REV`, i.e.
`[−0.1 − 0.1, 0.0 + 0.1] = [−0.20, +0.10]`, and `at_prime` as
`|pos − 9.9594| ≤ 0.10`, with `HAND_SETTLE_BAND_REV = 0.10`
(`canbridge_config.h:278`) and the two rest constants from
`config/generated/hardware_config.h:104,153-154`. +1.056 rev is outside both,
so the refusal is exactly right.

Recovery, launch down: at ≈14:01:04 the operator idled the hand motor with the
ODrive GUI, then backdrove it by hand to **+0.0689 rev**, and the switch to
`--source-only legacy` succeeded at ≈14:01:20. `[hand7]` finished
`src=LEGACY guard=ARMED lane=active`.

## Discussion

Two reframings; the rest of the sitting confirmed what was predicted and needs
no argument.

### 1. The hold's 3 mm "deviation" is the stroke clip, and it is a structural bias in the guard

The 30 s hold reported `max |cmd − (enc+vel·age)| = 0.0995 rev (3.15 mm)` —
77× the 600 s hold's 0.0013 rev, against a hand that was demonstrably
energised and undisturbed. That is a large enough jump to look like a
regression, and it is not one.

The CSV settles it in one line: **`cmd_rev ≡ −0.09764` and `echo_rev ≡
0.00000` on all 1200 rows, with `echo − cmd = +0.09764` exactly.** The hand had
been parked at −0.0977 rev by the legacy retract; the firmware's hand lane
clips its transmitted setpoint to `[0, HAND_MOTOR_MAX_POSITION]`
(`leg_interp.cpp:824-827`), so a command below zero goes on the wire as
literally 0.0, and the recovery slew walked the hand there — `|enc| < 0.005`
by t = 0.125 s, then +0.00003 ± 0.0003 rev for 29.9 s. The next stage's
baseline read −0.0001 → +0.0000, which is the same fact from the other side.

So the hand tracked its *actual* commanded target to **0.0003 rev = 0.01 mm**,
four times better than the 600 s hold. What both belts measured instead was the
gap between the *host's* command and the *firmware's* clipped one.

The part worth carrying is not the 3 mm; it is that **the firmware's own guard
residual has the same blind spot, and it is 1:1**. `leg_interp.cpp:792` computes
`dev = h_pos − fb_ex` from the **raw pre-clip** `h_pos`, so the console
`dev_last` sat at −0.0965…−0.0988 for the entire stage: a permanent residual
equal to the amount the command sits below the floor, on a perfectly tracking
axis. Here that is 3.9 % of the 2.5 rev band and harmless. It scales linearly,
and an armed guard would E-STOP a healthy hand if a producer ever commanded
2.5 rev below the floor.

Why record it as a first-frame *behaviour* rather than fix it: the clip is
deliberate and load-bearing (it is the metal, and clipping in the lane is what
zeroes the feedforward at the stop — the leg rule), and computing `dev`
pre-clip is equally deliberate (Phase 0 Decision 4 wants the residual to
measure what the *interpolator* asked for, so that a clamp cannot hide a
diverging command). Making `dev` post-clip would silence a real class of
failure to remove a benign one. The correct answer is the operating rule —
**never park-and-hold a streamed hand below 0** — plus knowing what the
symptom looks like so nobody spends a sitting on it. It is now in the runbook's
row 13.

### 2. The arm was taken; the trip was not; and that gap is now a Phase 5 decision

The operator's account had row 18's arming half as not run, carried again. The
console says otherwise: `guard=observe → ARMED` at ≈13:59:25, with the
`interp_hand7_console` handler's own out-of-cadence status echo at capture line
2339 marking the exact tick, and `ARMED` holds to the last block. The restraint
stage therefore ran **armed**, and § Observe-then-arm step 2 — "an observe-only
guard left in place indefinitely is a guard that does not exist" — is
discharged.

What was *not* discharged is the trip. The armed lane carried a restrained
0.2515 rev peak with **zero** `dev_over` ticks, which is a real and useful
result (no nuisance trip at 10 % of the band), but the E-STOP path from the
tick verdict through `fault_machine.cpp:418-433` to a latched
`FaultState::MAX_DEVIATION` has still never executed on hardware. The operator
stopped on motor temperature and has ruled the test out permanently.

**That call is right, and the arithmetic says so.** Reaching 2.5 rev of command
error at `pos_gain` 35 needs roughly ten times the restraint displacement
observed here — about 79 mm of command travel against a stalled rotor, at
saturated current the whole way. It is a deliberate thermal event by
construction, and there is no cheap version of it: the guard is sized for a
*runaway command*, and the only bench way to synthesise one is to stall the
plant. Repeating it to watch a latch we can read the source of is a poor trade
against a hand ODrive.

So the question is no longer "how do we observe the trip" but "**do we fly
Phase 5 armed with the trip unobserved**", and that is the owner's. The
recommendation, stated by failure mode rather than by authority:

- **Recommended: run the unified rungs ARMED, treat any trip as data.** What it
  prevents: a mis-planned or mis-timed hand knot driving the slider through its
  travel at up to 200 rev/s with nothing to stop it. `MAX_LEAD_HAND_REV` 2.0
  clamps the *command*, it does not stop the axis; `MAX_DEVIATION_HAND_REV` is
  the only thing that does. The cost of a false trip is bounded and known: a
  fail-safe E-STOP with the output gated, recovered by `CLEAR_ERRORS` and a
  re-arm whose recovery slew is already specified and already observed. The
  residual record now supports it — `dev_over` delta **0** across the whole
  sitting, carry-speed residuals **0.02–0.06 rev**, restrained peak
  **0.2515 rev**, worst excursion of any kind **1.1494 rev** (the manual park),
  all against 2.5. The known tight case is the stroke's 1.9847 rev, and the
  legacy stroke is not what Phase 5's rungs fly.
- **Alternative: stay observe-first.** What it prevents: exactly one thing —
  an aborted stroke's by-design trip costing a `CLEAR_ERRORS` mid-ladder, which
  sitting two already predicted and priced. What it costs: the ball-bearing
  rungs (UH-3 onward) fly with no hand deviation guard at all.

**Owner decision, pending.** Do not read this entry as having taken it.

**Taken later the same day**, after this write-up: the owner decided ARMED from
UH-3 on, with a trip treated as data — recorded in
[`2026-09-05-unified-7dof-phase5-prep.md`](2026-09-05-unified-7dof-phase5-prep.md)
and in the plan's Phase 5 block.

## Verification

Artifacts, all 2026-09-05, all from the operator:

- `temp/logs/hand7_console_20260905_135613.log` — the bracketed `[hand7]`
  capture, 313 blocks, 13:56:13 → 14:01:26. **Primary deliverable**; every
  counter delta in this entry comes from it.
- `temp/logs/hand_stream_hold_20260905_135648.csv` (+ `_cachediag`) — row 19(b).
- `temp/logs/hand_stream_hold_20260905_135732.csv` (+ `_cachediag`) — the 30 s
  STREAMED hold, the clip finding.
- `temp/logs/hand_stream_moving_gap_20260905_135840.csv` (+ `_cachediag`) —
  row 17b.
- `temp/logs/hand_stream_triangle_20260905_140020.csv` (+ `_cachediag`) —
  row 18's restraint run.

Analysis was done with throwaway probes under `/tmp/probe_*.py` (console
parser, per-stage counter differencing, an independent G1–G4 re-derivation from
`pred_rev`/`echo_rev`/`enc_rev`, and the cachediag/lead-mask correlation);
nothing was committed and no code was changed.

Doc gate — this close-out changed only documents (the runbook § Results, the
plan's Phase 3/5 blocks and its row in `plans/active/INDEX.md`, the `moving_gap`
entry's status and Outcome, this entry, and its row in `logbook/INDEX.md`):

- (2026-09-05, `python -m pytest tests/sim/test_plans_index.py
  tests/sim/test_logbook_search.py tests/sim/test_logbook_front_matter.py -q`,
  **108 passed in 0.70 s**).

## Outcome

**All four items carried out of sitting two are discharged**, and Phase 3's
record is complete:

| Carried item | Outcome |
|---|---|
| Row 17's decay half (row 17b `moving_gap`) | **PASS** — G1–G5, coast bit-equal to `v·0.105` |
| Row 19(b), v6 stream vs LEGACY | **PASS** — `discard_legacy` +222, `sent` +0 |
| Row 18's arming half | **CLOSED BY OPERATOR DECISION** — arm taken, trip unobserved; Phase 5 policy **RESOLVED the same day (owner): ARMED from UH-3 on, a trip is data** |
| A bracketed row-11c capture | **EXISTS** — `temp/logs/hand7_console_20260905_135613.log`, per-stage deltas in the runbook |

Whole-sitting `[hand7]`: `sent` **+35 222**, `discard_legacy` **+222**,
`unseen` **0**, `stale` **0**, `lead` **0**, `dev_over` **0**, `dev_max`
unmoved at the previous night's boot-cumulative 10.9794. The three streamed
stages' `sent` deltas sum to exactly the whole-sitting figure, so every hand
frame the firmware transmitted belongs to a stage and nothing leaked between
them. Validity: `[diag]` shows only `(link, fault)` = `(1, NONE)` while the
driver held the link and `(3, LINK_LOST)` when it did not — never
`MAX_DEVIATION`, `MPC_STALE`, `MOTOR_FB_STALE` or `ODRIVE_FATAL`; jugglebot bus
`err/rec/tec/defer/txq` all **0 → 0**; `drain_cap` 0; `[cantx] defer_by_class`
unmoved (only the pre-existing bb-bus `timesync=1970`); heap flat 2216–2416.

**The FW 18 counter-gate defect is still real and still unfixed** — it simply
did not bite, because `lead` / `dev_over` are *exceed* counters and the
idle-window residual never reached 2.0 / 2.5 rev. It came closest during the
close-out manual park at **1.1494 rev = 57 % of `MAX_LEAD_HAND_REV`**. Park a
streamed-latched hand from further away and `lead` starts ratcheting on a lane
that transmits nothing, which is exactly how sitting two banked 18 725 209.

## Open Questions

1. **The armed MAX_DEVIATION trip has never fired on hardware, and the only
   test written to fire it is closed.** Phase 5 arming policy was an owner
   decision pending at the time of writing — recommendation and failure modes in
   § Discussion 2. **RESOLVED the same day (owner, 2026-09-05): ARMED from UH-3
   on, and a trip is data**
   ([`2026-09-05-unified-7dof-phase5-prep.md`](2026-09-05-unified-7dof-phase5-prep.md)).
   What stays open is the underlying fact: the trip path is still unexercised,
   so the first one to fire on hardware is also its first test.
2. **The `−42`-frame cachediag episode on LEG 5.** Almost certainly the known
   stationary per-axis encoder-frame drop class
   (`plans/active/leg-bus-frame-drops.md`): it is on leg 5, not the hand
   (`deficit_6 = −0.0`), it coincides exactly with the sitting's only leg-5
   lead-clamp engagement, and comparable drops occurred with no restraint at
   all (leg 2 −11 in the 30 s hold). But it landed inside the restraint window
   and is 4× larger than anything else in the sitting, so a bus-load coupling
   (a current-saturated hand on the shared 48 V) is not excluded by this
   artefact. Row 18 will not be re-run, so the discriminator has to come from
   the drops plan's own workstream, not from another restraint block.
3. **Who idled axis 6 at ≈14:01:04?** Closed: the operator idled the hand via
   the ODrive GUI. `hand_stream_bench.py` has no IDLE-on-exit path (its only
   `SET_AXIS_STATE` is `--close-loop`'s 1 → 8) and `--source-only` is exempt
   from the energisation half entirely, so the bench driver was never the
   mechanism — it was always the operator's manual intervention. Phase 5 now
   has it written down: with launch DOWN, deactivate → idle via the ODrive
   GUI → push by hand → `--source-only legacy`.
4. **The launch-UP latch-return recovery does not exist yet.** With the launch
   down, a hand outside the settle bands is recovered by idling it and pushing
   it by hand. **With the launch up the hand is CLOSED_LOOP and not
   backdrivable, so that route is gone** — and a unified session leaves the
   hand at 0.3162 rev, outside `[−0.20, +0.10]` and unreachable from inside the
   QP's cup box (Phase 5 precondition (c)). The remaining routes are: disarm →
   drive the hand IDLE (the sitting-one "IDLE-on-exit" owner item, still open)
   → park → switch; a streamed retract with the output disarmed; or a bridge
   reboot, which boots LEGACY. One of these needs to be chosen and written into
   the Phase 5 runbook *before* the first unified rung, not discovered at the
   end of one.
5. **The retract-band reconciliation is CLOSED, not open** — recorded here so
   nobody re-opens it. Driver `[−0.20, +0.10]` ≡ firmware
   `[HAND_ABS_POS_REV − 0.10, HAND_RETRACT_REV + 0.10]`; prime 9.9594 ± 0.10;
   `HAND_SETTLE_BAND_REV = 0.10`. No inconsistency.
6. **Minor, carried forward unchanged:** `decode_bad_axis` on the jugglebot bus
   climbed **+622 over 312 s = 1.99/s**, the same steady rate sitting two
   flagged (its open question 10) and still unexplained, still moving no other
   counter. And the capture has **no per-line timestamps** — add `-f time` to
   the monitor command next sitting and the stage-boundary recovery step
   disappears.
