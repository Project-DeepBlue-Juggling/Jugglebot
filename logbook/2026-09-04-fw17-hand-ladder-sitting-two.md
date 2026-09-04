---
title: "Second FW 17 hand bring-up sitting — the energisation fix holds, a gap stage that measured a fiction, and a stroke that quietly got worse"
type: investigation
date: 2026-09-04
status: resolved
phase: "unified-7dof-planner — Phase 3 (second hand bring-up sitting)"
related_plan: unified-7dof-planner.md
files_changed:
  - tests/hardware/hand_stream_bench.py
  - tests/hardware/session_unified7_hand_bringup.md
  - plans/active/unified-7dof-planner.md
  - logbook/2026-09-04-fw17-hand-ladder-sitting-two.md
  - logbook/INDEX.md
subsystem:
  - can
  - ros
  - tools
tags:
  - safety
  - testing
  - performance
  - docs
---

# Second FW 17 hand bring-up sitting — the fix holds, the instrument does not

## Summary

Successor to
[`2026-09-04-fw17-hand-sitting-unflashed-idle-axis`](2026-09-04-fw17-hand-sitting-unflashed-idle-axis.md),
which found FW 17 blameless for the first sitting and fixed the two artifacts
that had produced its failures. The operator re-ran the ladder with that
driver gate in place, in three blocks: **bench 09:50–10:03** (launch down),
**launch-up 10:17–10:24**, and **bench + launch-up 14:47–15:09**.

**The energisation fix worked, and it is closed twice over.** `axis_state == 8`
on every row of every stage across the whole sitting (13 745 rows in the
morning block alone), `fault == 0`, `lead_mask == 0` everywhere. The
2026-09-03 de-energised class is shut. The independent second proof is the
600 s hold: `echo_rev ≡ cmd_rev` **bit-identical on all 24 000 rows**, which
only a live lane produces.

**Nine ladder rows PASS** (12, 13, 14, 16, 17's re-entry half, 19(a), 19(c),
20, 21). Three of them — **12, 19(c) and 21** — were closed by a **forensic
pass over the bags and the first console capture, after the sitting had ended**
(Symptoms rows 12, 19(c) and 21; Diagnosis § "The first console capture ever
taken"). **The owner declared Phase 3 COMPLETE on 2026-09-04 on that record.**
Four things came out of the sitting that did not go in:

1. **The gap stage had been measuring a fiction.** The driver computed
   `hand_override` for the CSV column and the deviation belt and then never
   passed it to `frame()`, so the re-entry never reached the wire — and the
   stage still printed *"31.67 mm"* against a runbook criterion reading *"ONE
   bounded ~32 mm catch-up move"*. It produced almost exactly the number that
   makes an operator tick the box. **Fixed here**, with a positive
   echo-movement check that is downstream of the wire.
2. **Two new findings on the stroke row, which must stay separate.** The
   morning's 2.17 ms timing bias did **not** reproduce in the afternoon
   (6.2× smaller, same form) — which **resolves its cause** and withdraws the
   firmware age-correction candidate. Independently, the stroke's tracking
   **degraded**: deviation 0.5387 → 1.2348 rev against a 1.5 rev belt, 18 %
   margin left. That second one is real, open, and not the timing bias.
3. **A drop-rate regression I asserted mid-sitting is WITHDRAWN** — the
   historical baseline's denominator is 6 and the driver's counter is 7.
   Normalised, nothing is significant.
4. **The number that governs the arming decision is NOT the one that looks
   alarming.** The first `[hand7]` line ever captured reads `dev_max=10.9794`
   — and that is **benign boot-cumulative history**, fully explained below and
   dead by construction. The real number is the **afternoon stroke's
   1.9847 rev of worst `|cmd − enc|`: 99.2 % of `MAX_LEAD_HAND_REV` and
   79.4 % of the 2.5 rev `MAX_DEVIATION_HAND_REV` band sitting three arms**,
   against 1.2067 rev on the morning stroke at the same `--event-vel`.

Row 18's arming half was **not run: the operator stopped because motor
temperature rose very quickly under restraint.** That was correct, and the
runbook was wrong to imply the test is cheap (Discussion 3). And **the console
record — the sitting's actual deliverable — still does not bracket a single
ladder row**: the first-ever capture was opened at **15:39**, after the last
block closed at 15:09, so it carries boot-cumulative history and the row-12 /
row-21 latch flips and **no per-row delta at all** (Discussion 4). New runbook
row **11c** makes the bracketed capture a precondition rather than advice.

## Symptoms

### The ladder, row by row

| Row | Test | Verdict |
|---|---|---|
| 12 | Source switch + refusals | **PASS** (closed post-sitting from the console capture + the driver's gate diagnosis) |
| 13 | T-H1 streamed hold | **PASS** (afternoon, the real 600 s) |
| 14 | T-H2a slow triangle | **PASS**, with a recorded ripple caveat |
| 15 | T-H2b stroke replay | **mechanism PASSES; a NEW defect appeared** |
| 16 | T-H3a host step refusal | **PASS**, console evidence only |
| 17 | T-H3c gap re-entry | VOID morning (driver bug); **re-entry half PASS** afternoon; decay half **UNOBSERVED** |
| 18 | T-H3b held-rotor deviation | **first half only** — arming half stopped on motor temperature |
| 19(a) | `set_hand_traj_cmd` under STREAMED | **PASS** |
| 19(b) | v6 hand stream against LEGACY | **NOT RUN** (mis-invoked) |
| 19(c) | `/set_hand_source` while ARMED | **PASS** (closed post-sitting; the over-determination dissolves twice) |
| 20 | Close-out validity sweep | **PASS** on every criterion the bag carries |
| 21 | Close-out state | **PASS** (the console capture carries the close-out `src=LEGACY guard=observe`) |

### Row 12 — the source switch, closed on three real state changes

**PASS**, closed post-sitting. Row 12 exists to prove the `hand_source` gates
*run*, and the trap the runbook now warns about is the idempotent-OK
short-circuit at `hand_source.cpp:56` — a re-assert of the current mode returns
OK **before** the mpc_active, freshness and settle gates and therefore proves
nothing. All three requests here were genuine **state changes**, so every gate
below `:56` actually executed:

| Request | Hand position | Result |
|---|---|---|
| STREAMED → LEGACY | +0.0000 rev (in the retract band) | **OK** |
| LEGACY → STREAMED | +0.9999 rev (**out of band**) | **REFUSED** |
| LEGACY → STREAMED | +0.0000 rev (in band) | **OK** |

The refusal carried the driver's own gate diagnosis, naming the position
against both bands: *`pos +1.000 rev vs retract [-0.20, +0.10] or catch-prime
9.96 ± 0.10`*. That is the row's whole point — the firmware returns one opaque
`ERR_REJECTED` by design and the driver disambiguates from its own caches.

The 15:39–15:43 console capture independently records **five** latch flips
across the window (row 12's pair, row 19's pair, and row 21's close-out),
ending at `src=LEGACY`.

### Row 13 — the 600 s hold

Run at 120 s in the morning (short **again** — the same shortfall as
2026-09-03) and at the real 600 s in the afternoon. **PASS.**

- `dev_max` **0.0013 rev = 0.041 mm**, and **FLAT over the full 10 minutes**:
  trend **+1.26e-09 ± 8.2e-10 rev/s**, t = 1.54, not significant.
- That is **385×** under the driver's own belt and **1920×** under the 2.5 rev
  `MAX_DEVIATION_HAND_REV` guard the *second* sitting arms against.
- Encoder net drift over 10 minutes: **−0.00006 mm**.
- `echo_rev ≡ cmd_rev` bit-identical on all 24 000 rows.

### Row 14 — the slow triangle

**PASS.** Tracking error flat to **0.8 %** across six 10 s blocks, worst
**1.92 mm**, hitting the same vertex every 8 s.

Caveat recorded rather than dismissed: a **~6.3 Hz, ±0.73 mm velocity
ripple**. It is **real motion**, not estimator noise — correlation
**r = 0.872** against the differentiated encoder — and it is **not a
turnaround artefact**, because the ripple rate is the same near a vertex and
away from one.

### Row 16 — the host step refusal

**PASS, on console evidence only.** `pump refused=True reason='hand step
6.0000 rev > 5.0 limit'`, twice, independently. The 5.0 rev boundary run
returned `refused=False` — the gate is `>`, so exactly-at-limit is accepted.
This is the first live confirmation of the Phase 2 derivation chain in the
shipped pump: **200 rev/s × 0.025 s = 5.0 rev**.

### Row 17 — the gap stage, void then passing

The morning run was **VOID**: the driver defect below meant the re-entry
never reached the wire. The evidence is unambiguous — `echo_rev` **bit-frozen
at 0.00018 for all 1200 rows**, total encoder excursion **0.0168 mm**, against
a *logged* 1.0 rev step.

After the fix, the same stage in the afternoon:

- **31.71 mm in one move**, peak **19.86 rev/s**;
- overshoot **+0.0835 mm (0.26 %)**, **one** sign change, no oscillation;
- the echo moved **within 25 ms** — the CSV's resolution floor, i.e. as fast
  as this instrument can see.

**The DECAY half remains UNOBSERVED**, and this is structural, not an
omission. The hand was at rest when the gap opened (max |vel| **0.073 rev/s**,
encoder span **6.6 µm** across the gap window), so there was no velocity to
decay. The gap stage **rides a Hold**, so **no current stage can exercise the
falling-edge decay at all** — that needs a gap taken while the hand is moving.

### Row 18 — the restraint curve, and where it stopped

The observe-half curve is good evidence of grow-then-recover: elevated error
with a positive median bias across **t = 10–30 s**, peak **0.1965 rev at
t = 28.25 s**, and a clean return to baseline from **t = 30 s**.

But that peak is **7.9 % of the 2.5 rev threshold**, so **the tick verdict was
never exercised**. The arming half — `hand7 arm`, then restrain until the
E-STOP latches — **was not run**: the operator stopped because **motor
temperature rose very quickly under restraint**. See Discussion 3; this is
recorded as sound judgement, not as an omission.

### Row 19 — three halves, three different outcomes

**(a) PASS** (10:17 bag). `hand_traj_acks = calls=1 ok=0 fail_teensy=1` with
**all five wire-visible `hand_ops` exit counters at 0**. `ERR_HAND_SOURCE` is
deliberately the one exit that does **not** ride the frame
(`telemetry.cpp:392-396`), so that exact pattern — one `fail_teensy`, zero of
everything visible — is **uniquely its fingerprint**.

Stronger, and worth having: axis 6 stayed **IDLE for 20 s after** the refused
call, and `hand_ops`' first CAN send is `set_state(CLOSED_LOOP)`
(`hand_ops.cpp:118-124`). So the refusal **provably preceded any CAN
side-effect**, exactly as `hand_ops.cpp:84-88` claims by construction.

**(b) NOT RUN.** Mis-invoked as a single command
(`--source-only legacy --stage hold ...`). `--source-only` switches the latch
and **returns immediately** (`hand_stream_bench.py:647-650`), so the hold half
never ran. It also has **no bag surface even in principle** — `discard_legacy`
is console-only.

**(c) PASS**, closed post-sitting on the **15:41 bag**
(`~/Desktop/rosbags/2026-09-04_15-41-04`), which is a different and much
cleaner armed window than the 15:08 one the mid-sitting read used. The
over-determination that made the earlier read useless **dissolves twice over**:

- **Structurally.** `hand_source_request` orders its gates bad-args →
  idempotent-OK (`:56`) → **`mpc_active` (`:60`)** → heartbeat (`:68`) → ts
  (`:71`) → staleness (`:73`) → **settle (`:74`)**. The arming gate
  short-circuits **before the settle gate is ever read**, so a refusal while
  armed is attributable to `:60` whatever the hand's position.
- **Empirically, and this is the stronger half.** Over the armed window
  **15:42:30.245 → 15:43:15.941 (45.597 s)** the hand's `pos_meas` spanned
  **[−0.000219, +0.000271] rev** — **0.27 % of the 0.3 rev retract band**,
  nearest-edge margin **0.099729 rev** — with `|vel|` max **0.207 rev/s**
  against the 0.5 rev/s limit and a worst inter-sample gap of **18.36 ms**
  against the 150 ms staleness cap. Replaying the firmware's own
  `hand_settled_at_rest()` predicate sample by sample gives **0 failures /
  4580 armed samples** and **0 / 14053 whole-bag**. The settle gate would have
  **passed**; only the arming gate can have refused.

Latch and clamp corroborate: `hand_source` reads **STREAMED in 459/459 armed
samples** (the latch demonstrably did not move while armed — which is the
refusal's observable consequence), `lead_clamp_mask` **0 in 459/459**, and bit
6 is never set anywhere in the bag.

**Caveat, recorded rather than glossed:** rosbag2 does not record *services*,
and the bridge node logs no arm/refusal line, so the `ERR_REJECTED` **return
byte itself is not captured**. The refusal is inferred from the latch not
moving — a necessary consequence of it, but not a direct capture. That is the
same observability hole as the unlogged arm refusals below, and it is why the
row stayed open until the bag was replayed.

### Row 20 — the close-out sweep

**PASS on every criterion the bag carries**: `leak_*` and `leak_hwm_*` all **0**
across 401 `/ring_diag`; `interp_deadline_misses` **0** and
`interp_max_jitter_us` max **2 µs** across 402 `/profile`; `latency_monitor`
**OK 4029/4029**; `tx_deferred` jb delta **0**; `bridge_fw_version`
**17 (proto 6)**; `BRIDGE_FW_CHECK: OK`; `lead_clamp_mask` **0** throughout,
**including bit 6**.

**The console capture corroborates it from the other side**, across all 264
blocks and with the bus-health fields no bag carries:

- **Jugglebot bus clean and constant**: `err=0 rec=0 tec=0 defer=0 txq=0
  capHit=0 gated=0`, `hwm=9`.
- **`[cantx] defer_by_class`: zero leg and zero hand deferrals** — the FW 10
  mailbox raise still holding under the 7-frame burst. (`timesync=1961` is the
  absent BB bus, not this one.)
- **`flt=passive` on the bb bus is a STICKY historical mark, not a live
  fault** — BB is absent (`ack=617`), every one of its counters is constant
  across the 264 blocks, and `tecNow` decays 56 → 5 → 0 within the capture.
- `heap` flat at **2408**; `seq_gaps` **+30 over 267 s**; `[axes] fresh=7/7` in
  every block with no error marks.
- One thing that is not clean and is not new: **`decode_bad_axis` climbs
  steadily at ~2.0/s** (119 730 over ~16.6 h of uptime, same rate throughout).
  Benign, unexplained, carried to Open Questions.

### Row 21 — the close-out, closed by the console capture

**PASS.** The 10:17 session closed to LEGACY correctly, and the 15:39–15:43
capture carries the close-out state directly, which no bag can:
**`[hand7] src=LEGACY guard=observe lane=idle`** on the final line, and
`guard=observe` on **every one of the 264 `[hand7]` blocks** in the file —
i.e. `hand7 arm` was never issued at any point (consistent with row 18's
arming half not being run). The robot left the sitting on the LEGACY path, one
`hand_source` switch away, exactly as Phase 5 expects.

This is also the concrete demonstration of why row 11c exists: the row-21
verdict was *unobtainable* until a console file existed, and it took a capture
opened half an hour after the ladder to supply it.

## Diagnosis

### The first console capture ever taken, and the alarming number on it

`temp/logs/hand7_console_20260904_153911.log` (15:39:11 → 15:43:38, 264
`[hand7]` blocks) is the first `[hand7]` record this project has ever held. Its
opening line reads (one line on the console, wrapped here):

```
[hand7] src=STREAMED guard=observe lane=idle sent=531218 discard_legacy=0
        unseen=0 stale=0 lead=18725209 dev_over=18725059 dev_last=0.0461
        dev_max=10.9794 dev_cmd=10.8161 dev_fb=-0.1632
```

**18.7 million lead-clamp ticks and 18.7 million deviation-exceed ticks, on a
lane reading `lane=idle` with the guard in observe.** Taken at face value that
is the runbook's row-15 hard-abort condition met eighteen million times over.
It is instead **benign boot-cumulative history, and every part of it
reconciles.**

**Mechanism.** `s_hand_active` (`leg_interp.cpp:439`) is a **latch**, and it is
cleared at exactly one site: the output-enable **rising edge**
(`leg_interp.cpp:479-484`). When the 2026-09-03 de-energised stroke aborted
mid-run, the lane stayed latched with no further arm to clear it, so the
deviation and lead census at `leg_interp.cpp:684` kept evaluating a **frozen
Mode-3 command** against the live encoder at 500 Hz until the next arm, ~10 h
later.

Four independent reconciliations, none of which needed the others:

- **The tick count is the wall-clock gap.** `lead / 500 = 37 450 s`. The last
  frame of the 2026-09-03 stroke lands 23:27:25 and the next arm is
  09:50:38 the following morning — **37 393 s**. The 57 s residual is **0.15 %**,
  and is itself accounted for: the hand travelled 9.96 → −0.11 rev between two
  step runs while the lane sat frozen, so the residual briefly re-entered the
  band.
- **`dev_cmd = 10.8161` is derivable in closed form.** The pump always sends
  `accel = 0` (`teensy_link/setpoint_pump.py:710`), so with `a0 = jk = 0` the
  Mode-3 formula (`leg_interp.cpp:734-748`) collapses to
  `u1 + (MAXEXT + DECAY/2)·v1 = u1 + 0.08·v1`. At the `HandStrokeModel(3.0)`
  stroke phase `t = 0.075126 s` — `u1 = 4.1630`, `v1 = 83.165` — that is
  **10.8162**, matching to float32 rounding.
- **`sent` matches the streamed exposure.** `sent / 500 = 1062.4 s` against
  **1060.6 s** summed from every stage CSV: **0.17 %**.
- **Nothing near 10.8161 ever reached the wire.** `dev_cmd` is captured
  **PRE-clip** (`leg_interp.cpp:792-799`), above both the lead clamp (`:814`)
  and the stroke clip (`:824`). The proof is the number itself:
  **10.8161 > `HAND_MOTOR_MAX_POSITION` 10.8**, which the clip cannot emit by
  construction. And `sent` did not advance a single frame across the whole
  window.

**The `/link_status` reading is consistent, not contradictory.** Row 20's
`lead_clamp_mask == 0` in 4029/4029 samples and this 18.7 M `lead` are
different quantities: the mask is an **instantaneous last-tick snapshot**
sampled at 10 Hz, `lead` is a **cumulative exceed-tick count**. The exceed
window closed at 09:50:38, before either bag started.

### The counters are gated wrong, and that is ours — a real contract/enforcement mismatch

`lead` and `dev_over` are **documented as throw-DUTY counters** —
`canbridge_config.h:256-259` ("non-zero duty during a throw is a
hard-abort-the-sitting signal"), `leg_interp.cpp:196` ("THE lead-duty counter
(nonzero in a throw ⇒ abort sitting)"), and the runbook's row-15 rule. But
their gate is `s_hand_active && hand_source_streamed()`
(`leg_interp.cpp:684`), plus `hts != 0` at `:758`. **`s_output_enabled` is not
in it.** They therefore count ticks on which the firmware **transmitted
nothing**.

The consequence is not cosmetic: **the runbook's "non-zero `lead` ⇒ HARD
ABORT" rule is unfalsifiable as written.** After any aborted or ended stage the
counter reads non-zero forever, so the operator can never satisfy it and can
never fail it either — the exact shape of a safety rule that has quietly
stopped being a rule.

**Minimal fix, and it needs a reflash:** add `out_en` to the gate at
`leg_interp.cpp:814-815` (the lead-duty increment) and `:792-799` (the
deviation census), or add a second, output-enabled-only counter pair beside the
existing ones and make *that* the duty read. Either way this is **sitting-three
/ FW 18 work**, not a same-session fix — nothing on this board changes without
another lockstep-class flash event. The runbook's row-15 rule is amended in the
meantime (Fix).

### There is no reset short of a reboot, so sitting three must difference

`interp_hand7_console` (`leg_interp.cpp:1213-1223`) accepts exactly three
forms — `hand7 arm`, `hand7 observe`, and bare `hand7` — and **none of them
resets a counter**. `interp_reset()` (`:1032`) zeroes them at `:1044` but has
**no runtime caller** (the file says so at `:248` and `:463`). `CLEAR_ERRORS`
does not touch them; `REBOOT_ODRIVES` reboots the ODrives, not the Teensy. And
`dev_max` / `dev_cmd` / `dev_fb` are **boot-cumulative high-waters that never
decay**, so a single aborted stage poisons them for the rest of the uptime —
which is precisely what happened here.

**Method for sitting three, stated so it is not re-derived at the bench:** the
lockstep flash **is** the reboot. Take the **first post-flash `[hand7]` line as
the true zero**, difference every counter across every stage, and **never read
an absolute**. That is what row 11c's "paste the line between rows" instruction
buys, and this sitting is the demonstration of what it costs to skip.

### Arming will not trip on any of this — by construction, and measured

The obvious fear on reading an 18.7 M `dev_over` is that `hand7 arm` at sitting
three would fire the E-STOP on the first fault-task poll. It cannot.

`fault_machine.cpp:409-411` computes `hand_dev_new = (hand_dev_over !=
s_hand_dev_over_prev)` and updates `s_hand_dev_over_prev`
**unconditionally — above the gate** at `:418-421`. That ordering exists
precisely so a disarmed, observing or LEGACY window cannot bank a stale delta
that fires on the first armed poll. The trip also freezes the
**trip-dedicated** `interp_hand_dev_trip_*` trio (`:428-430`), never the
boot-cumulative `dev_max` — the 2026-09-02 review fix.

Proven empirically as well: the operator's **15:42 armed window held
`mpc_active = 1` with `src=STREAMED` for 46 s** and **not one of these counters
moved a tick** (`sent`, `lead`, `dev_over`, `dev_max` are bit-identical on all
264 blocks of the capture).

### The gap stage measured nothing, and said the right number while doing it

`tests/hardware/hand_stream_bench.py` computed `hand_override` in the gap
branch, and **used it twice** — for the CSV's `cmd_rev` column and for the
deviation belt — but **never passed it to `frame()`** at the streaming build
call. The stage therefore streamed a **constant Hold** while its own CSV
logged a re-entry that never happened, and the belt compared the phantom
against itself.

Two properties made it silent, and both matter more than the typo:

- **The phantom's deviation is `|gap_delta|`, and the stage's own belt is
  `|gap_delta| + 0.5`.** The fiction sits *under* every bar in the loop by
  construction. No amount of tightening any existing check would have caught
  it.
- **The reported number was right.** The stage printed *"31.67 mm"*; the
  runbook's criterion reads *"ONE bounded ~32 mm catch-up move"*. The
  displacement was computed host-side from intent, so it was always going to
  land on the expected value.

Confirmation from the morning data is total: echo bit-frozen at
**0.00018 rev** for all 1200 rows, **0.0168 mm** of encoder excursion, under a
logged 1.0 rev step.

### Row 15, finding (1) — the timing bias did not reproduce, and that resolves it

The morning stroke reproduced the bias the runbook's row-15 discriminator was
written for: **8 moving samples**, `recon_mm / analytic_velocity` =
**2.168 ± 0.085 ms** across a **6.01×** velocity span; linear fit
**A = +0.064 mm / B = +2.120 ms** — the position term is ~0, so it is a
**pure fixed time offset**.

The afternoon stroke has **the same form and 6.2× less of it**: a tight core
of four samples at **0.3492 ± 0.0016 ms** across a 2.1× span **spanning both
accel and decel**, fit **A = 0.000 mm / B = +0.3492 ms**.

That non-reproduction is the whole diagnosis. Row 15 named two candidates and
could not separate them:

- a residual **host↔bridge wall-sync offset**, or
- the firmware **`interp_last_tick_us()` age correction under-correcting**
  (`telemetry.cpp:275-277`).

**A firmware age-correction is deterministic code on an unflashed board. It
would give the same offset twice. It did not.** The evidence therefore points
at the wall-sync offset and **against** the firmware candidate.

One caveat, recorded because it is the only thing that could rescue the
firmware candidate: this argument requires the bridge **not** to have been
rebooted between the two runs, and **no CSV column carries uptime**, so it
could not be checked. (An `uptime_ms` column would have settled it outright —
carried to Open Questions.)

On the bar itself: **every scored afternoon sample was ≤ 1.85 mm**, a pass on
the mechanism the 3.25 mm bound was derived for.

### Row 15, finding (2) — a real stroke tracking degradation, NEW and open

**Keep this separate from finding (1). They are different measurements and
they moved in opposite directions.**

The same stage, identical knots, went **0.5387 → 1.2348 rev** of deviation
against a **1.5 rev** belt — **18 % margin left, down from 64 %**.

Two explanations were checked and **ruled out**:

- **The starting offset.** The afternoon started **closer** to zero after the
  firmware recovery slew (encoder **+0.00008** vs **−0.00211** at stroke
  onset), and `cmd_rev` is **bit-identical** between the two runs. A worse
  start cannot be the cause of a worse result when the start was better.
- **The telemetry-age term.** The belt is `|cmd − (enc + vel·age)|`, and the
  afternoon's age was **larger** — which *subtracts*. And over any uniform age
  in [0, 30 ms] the floor-to-floor ratio is still **2.60×**, so no age
  assumption erases the growth.

Two **age-free** measures confirm it independently:

- **Encoder crossing-time lag**, averaged over 11 position levels:
  **+12.25 → +18.11 ms (+5.86 ms)**.
- **End-of-stroke overshoot past the endpoint**: **3.68 → 16.13 mm (4.38×)**,
  with return velocity **−6.12 → −11.62 rev/s** and settle time
  **750 → 1075 ms**.

**Hypothesis, explicitly NOT established: stage ordering.** The morning stroke
followed a hold and a triangle, i.e. an already-exercised hand; the afternoon
stroke was the **first streaming stage of its block**, on a hand that had sat
de-energised. Plausible (lubrication, thermal, ODrive state), unproven, and
easy to test by re-ordering.

**And one part of it is not the plant at all.** The afternoon **firmware
target** (`echo_rev`) itself overshot the endpoint by **+0.1678 rev
(5.31 mm)** versus the morning's **+0.13 mm** — on identical knots, an
identical 25 ms grid, with `lead_mask = 0`. So roughly **one third of the
encoder overshoot was COMMANDED, inside the FW 17 hand lane.** The sitting's
worst reconstruction sample, **6.55 mm**, is driven by *this*, **not** by the
timing bias of finding (1). Do not merge the two.

### Row 15, finding (3) — the number that actually governs the arming decision

**This supersedes the 10.98 as the number sitting three has to reason about,
and it is the only one of the three row-15 findings that constrains a safety
decision.**

The same degradation, read against the **guard constants** rather than the
driver's belt:

| Stroke | worst `\|cmd − enc\|` | of `MAX_LEAD_HAND_REV` 2.0 | of `MAX_DEVIATION_HAND_REV` 2.5 | peak encoder (stop 10.8) |
|---|---|---|---|---|
| 09:56 | **1.2067 rev** | 60.3 % | 48.3 % | 10.0759 rev |
| 14:47 | **1.9847 rev** | **99.2 %** | **79.4 %** | **10.4693 rev** |

Same `--event-vel`, identical knots. The afternoon stroke sat **0.0153 rev off
the lead clamp** and used **four fifths of the band the deviation guard trips
on**, and the encoder overshot to within **0.33 rev of the hard stop**.

**Caveat, stated because it cuts both ways and neither correction is small:**
this is the 40 Hz CSV's *raw* `|cmd − enc|`, not the firmware's
velocity-compensated residual. Compensation **shrinks** it (the guard subtracts
`vel·age`); 40 Hz sampling of a 500 Hz quantity **undersamples the peak**, so
the true worst tick is higher than what the CSV shows. The honest read is
"same order, sign of the error unknown" — which is exactly why it is a
budgeting number and not a verdict.

**Two consequences for sitting three, both operational:**

1. **Armed, expect the 2.5 rev band to be tight on strokes — budget for a
   trip.** A trip is not evidence the guard is wrong; at 79.4 % of the band on
   a *nominal* stroke, one is a reasonable outcome and the recovery path
   (`--clear-errors`, re-arm with the bring-up, watch the recovery slew) is
   already written into row 18.
2. **Armed, an aborted stroke WILL fire the guard, by design.** The lane
   decays on its own knot clock, so the residual passes 2.5 rev within ~30 ms
   of the last knot and needs a `CLEAR_ERRORS` to recover. That is the
   falling-edge rule working, not a fault — but it means an abort is no longer
   a free action once the guard is armed.

### The drop-rate scare, normalised away

Mid-sitting I asserted a drop-rate regression to the operator: **5.3 % of
windows with an episode against a 1.6 % 2026-08-30 baseline, ~3.3×**, and
flagged a **67-frame drought on axis 6** (the hand) as concerning.

**Both claims are withdrawn**, on a denominator check that should have come
first:

- **The 2026-08-30 baseline is LEGS-ONLY.** Its own arm-table means divide by
  **6** (0.78 × 6 = 4.68 ≈ the quoted 4.65 f/s), and its qualification 3
  records **four ax-6 episode windows in arm B**, which is impossible under an
  any-of-7 count.
- **The driver's counter is all-7** (`hand_stream_bench.py:887-890`).

Normalised both ways, nothing survives:

| Comparison | This sitting | 2026-08-30 | p |
|---|---|---|---|
| legs-only | 29/601 = **4.83 %** | 2/124 = **1.61 %** | 0.162 |
| all-7 | 32/601 = **5.32 %** | 6/124 = **4.84 %** | 1.000 |

Continuous statistics agree it is a **normal plant**: **3.48 drops/s over
6 legs** (2026-08-30 arms 4.65 / 2.14 / 4.19 / 0.00; historical band 1.8–5.0),
and per-axis-window **−0.580 ± 5.126** against a **−3.6 ± 10.3** reference.

And **axis 6 is the best-behaved axis** over the 600 s soak — lowest net loss
(**0.446 %**), lowest sd (**3.590**), joint-lowest episode count (**3**) —
about a **13× per-unit-time reduction** against the very arm that raised the
worry.

Episodes are **stationary**: 60 s bin counts
`[0, 3, 4, 4, 4, 3, 4, 4, 0, 2, 4]`, index of dispersion **0.778**
(*under*-dispersed). No uptime ratchet. `rx_depth_hwm_jb` flat at **9/256**
for all 601 windows.

**One genuinely new and unexplained thing** came out of that sweep, and it
belongs to `plans/active/leg-bus-frame-drops.md`, not to this entry: **axis
5's five drop events are periodic at ~134.5 s** (135.2 / 135.2 / 134.3 /
133.2), with axis 4 weakly at ~130–140 s. That is far too slow for a bus or
ISR mechanism and looks **ODrive-scheduler-side**.

## Discussion

### 1. A test that reports the expected number while measuring nothing

The gap bug is the canonical instance of a class this project keeps meeting:
**a stage that computes its verdict from host-side intent cannot fail
honestly.** Every observable the gap stage judged itself against —
`cmd_rev`, the deviation belt, the printed displacement — was derived from the
same local variable whose journey to the wire was broken. The stage was
internally consistent and externally empty.

It is worse than a silent no-op, because the failure mode is *confirmatory*:
the phantom deviation sits under the stage's own belt **by construction**
(`|gap_delta|` vs `|gap_delta| + 0.5`), and the number it prints —
"31.67 mm" — is almost exactly the "~32 mm" the runbook told the operator to
look for. A test that lands on the expected value while measuring nothing is
strictly more dangerous than one that fails, and this is the second sitting in
a row where a *pass* was the problem (2026-09-03's hold stage certified a dead
axis).

**The echo check is the fix because it is downstream of the wire, not because
it is one more assertion.** Under STREAMED the hand echo re-sources from
`axes[6].target_pos_rev` and is emitted **only when the interp actually
TXed**, so a re-entry that reached the firmware **must** move it. Adding a
tighter belt, a stricter tolerance, or a second host-side cross-check would
have caught none of this. The general rule, worth carrying to the next stage
written for this ladder: **every stage needs at least one observable the host
cannot fabricate.**

Note also what the first sitting's entry already recorded and what this one
sharpens: the echo proves the lane *computed and dispatched*, and only the
encoder is evidence about the **plant**. Both halves are needed, and the gap
stage had neither.

### 2. Two withdrawn hypotheses, and what withdrew them

Both were **mine**, and both were asserted to the operator **before** the
check that killed them.

**The drop-rate regression** died on **normalisation**: the historical
baseline counts six axes, the driver's counter counts seven, and the entire
"3.3×" was that denominator. Normalised either way, p = 0.162 and p = 1.000 —
and the axis I had flagged as concerning (the hand) turned out to be the
**best-behaved** one in the soak. The lesson is narrow and general: **a ratio
against a historical baseline is worthless until the baseline's denominator is
verified.** The tell was available before the alarm — the baseline's own
qualification 3 records four ax-6 episode windows, which cannot exist in an
any-of-7 count.

**The firmware age-correction candidate** for the 2.17 ms timing bias died on
**non-reproduction**, which is the cleanest way any hypothesis can die here:
**deterministic code on an unflashed board cannot give two different
answers.** Same board, same image, same stage, same knots — 2.168 ms in the
morning and 0.3492 ms in the afternoon, both with a zero position term. That
is not a firmware constant; it is a clock relationship that was re-established
between the runs. Recording the *form* (constant ms across velocity, zero
position term) mattered more than recording the magnitude: it is the form that
identified the class, and the magnitude change that eliminated one member of
it.

The honest residual is stated in the Diagnosis and carried below: this
argument assumes no bridge reboot between the runs, and **no CSV column
carries uptime**, so it could not be verified. That is a gap in the
instrument, not a gap in the reasoning — and it is exactly the kind of gap
that `enc_age` (added this commit for the sibling finding) was added to close.

### 3. Why the operator stopping row 18 was correct

The runbook asks the operator to `hand7 arm`, then restrain the slider until
`dev_over` counts past 2.5 rev. The operator stopped after the observe half
because **motor temperature rose very quickly under restraint.**

That was the right call, and the procedure was wrong to imply the test is
cheap. **Holding the rotor until 2.5 rev of position error accumulates at
`pos_gain` 35 saturates current BY CONSTRUCTION.** The whole test *is* a
deliberate stall; the heating is the mechanism, not a side effect of doing it
badly. A written step that says "repeat the restraint briefly" reads as a
30-second task and is in fact a controlled thermal event on a 48 V motor.

It is also **properly the second sitting's step in the plan's own terms**: the
guard ships **observe-first**, and the runbook's § Observe-then-arm names
`hand7 arm` as sitting two's decision. Stopping did not skip a step; it
declined to pull a later sitting's step forward into this one.

This is a case of **physical intuition overriding a written procedure**, which
this project treats as load-bearing signal. The procedure is the artifact that
should change.

### 4. The console record is the sitting's actual deliverable, and it arrived half an hour late

Two sittings have now produced **no console capture that brackets a ladder
row**. That is not a housekeeping complaint — **every counter this ladder
judges is console-only**: `lead`, `dev_max`, `dev_over`, `dev_last`,
`dev_cmd`, `dev_fb`, `unseen`, `stale`, `discard_legacy`, `sent`, `src=` /
`lane=` / `guard=`, and `[cantx]`'s `defer jb` / `txq jb`. None of it reaches a
CSV, a UDP uplink, or a bag.

**A capture does now exist** — `hand7_console_20260904_153911.log`, the first
ever — and what it did and did not settle is the sharpest possible argument for
row 11c:

- **What it closed, and could close nothing else:** row **21**'s
  `src=LEGACY guard=observe` close-out state, and row **12**'s latch flips.
  Both are *state* reads, true at any instant, so a capture taken at any point
  after the ladder answers them.
- **What it could not close, because the ladder had already ended:** every
  **delta**. `sent`, `lead`, `dev_over`, `dev_max` are bit-identical across all
  264 blocks, so rows 13–19 get **nothing** from it. Two hundred and sixty-four
  identical lines is not a record of a sitting; it is a photograph of the
  moment after one.
- **What it revealed that no one was looking for:** the 18.7 M counter history
  and, through it, the counter-gating defect above. That find is a **bonus of
  capturing at all**, not a substitute for capturing *on time*.

The concrete cost is still visible: **the observe-first `dev_max` baseline that
sitting three arms the 2.5 rev guard against exists only as a CSV-derived
substitute** (0.0013 rev / 0.041 mm) — and the capture's own `dev_max` is the
poisoned 10.9794, which is worse than no number at all if read naively. That
substitute is good, flat over 10 minutes with a statistically insignificant
trend, but it is a *reconstruction* of the number the guard's arming decision
needs, not the number itself. Rows 16, 18 and 19(b) likewise lost evidence they
had no other channel for.

That is why **row 11c is a precondition row, not advice**, and why it is
placed at row 6 in practice ("start the capture when you open the monitor").
Two mechanisms were considered and one rejected on a concrete ground rather
than taste: PlatformIO's own `-f log2file` filter exists in 6.1.19, but the
command body runs under `fs.cd(project_dir)` and the filter opens a
**relative** `logs/`, landing the log **inside the firmware source tree** —
against this repo's temp/-only runtime-artifact rule. `script -f` flushes on
every write (so the log survives an abnormal end) **and keeps the monitor
interactive**, which row 18 requires for `hand7 arm` / `hand7 observe`. A
plain `> file` redirect would have cost the interactivity.

### 5. `auto_arm` defaults true, so the operator does not own the armed edge

Row 19(c) needs the arming gate to be the *only* reason `/set_hand_source` is
refused, and the **15:08** window gave neither half of that. The arm was the
**orchestrator's**, 41 ms after `streaming ENABLED` (`auto_arm` defaults true —
`jugglebot_launch.py:287-288`, `orchestrator_node.py:64`,
`state_machine.py:506`); a manual `/set_setpoint_output true` therefore
normally returns *"already enabled"*, and the operator never controls the armed
edge at all. And the refusal it produced was over-determined: the hand at
1.0455 rev was outside both settle bands, so `hand_source.cpp:74` would have
refused armed or disarmed. Meanwhile both `/set_hand_source true` "successes"
in that window were **idempotent no-ops at `hand_source.cpp:56`** — second in
the chain, before the arming gate and all four telemetry gates, so they
executed **zero gates**. (That idempotent-OK shape is exactly why row 12 now
mandates establishing LEGACY first.)

**The 15:41 bag settled the row anyway, and the reason is worth stating**: the
over-determination is a property of *that* window, not of the test. In the
15:41 window the hand was inside the retract band by a factor of ~370 (Symptoms
19(c)), so the settle gate would have **passed** — and even if it would not
have, `mpc_active` at `:60` short-circuits **above** `:74`, so the refusal is
attributable to the arming gate on the gate ordering alone. Two independent
arguments, either one sufficient. **`auto_arm:=false` remains worth having**
for a sitting-three re-run — it makes the armed edge the operator's and removes
the need to reason about which window was which — but it is a convenience now,
not a blocker.

**A real observability defect found in passing**, worth its own line because it
can burn an unattended run: `_arm_setpoint_output`'s docstring claims reject
reasons are *"surfaced in the service response AND logged"*. **Arm refusals are
never logged** — there is a single unrelated `get_logger()` call in the whole
range. So an orchestrator retry loop can burn its **10-attempt budget
silently** before FAULTing, with nothing in the log saying why. The same hole
is what made 19(c) an *inference* rather than a capture: rosbag2 records no
services, and with nothing logged there is no other channel for a refusal.

### 6. The review fix that prevented a real incident, before anyone knew it would

`leg_interp.cpp:479-484` — clear the hand lane on the output-enable rising
edge, **before** `latch_from_staging()` at `:486` — landed on 2026-09-02 as a
Phase 3 review fix. It looked like tidiness: a stale lane should not survive
into a new session.

This sitting is the incident it prevented, and the counter history is the
proof. Through the overnight window the lane sat **latched and frozen at a
Mode-3 command of 10.8161 rev**, ~10.9 rev away from where the hand actually
was. Without that clear, the 09:50 arm would have run `latch_from_staging()`
on a **re-animated** lane holding that frozen command — and the output-enable
edge is exactly where the **recovery slew** begins, so the firmware would have
walked a **powered** hand toward 10.8161 rev at ≤ 1 rev/s. That is a commanded
excursion into the end stop on the first arm of the day, with the guard in
observe and nothing else in the chain positioned to refuse it.

Two things are worth taking from this. First, **it argues for the review
discipline in the strongest available form**: nobody predicted an abort leaving
a lane latched across a 10-hour gap, and the fix was justified at the time on
structure alone — "a latch must have exactly one clear site, and it must be the
session edge". Structural reasoning bought a hazard nobody had enumerated.
Second, **do not weaken it.** The obvious "simplification" — clearing the lane
lazily, or on the falling edge, or inside `latch_from_staging` — puts the clear
back on the wrong side of the slew and re-opens exactly this path. The clear
belongs above `:486`, and this entry is the reason.

## Fix

**`tests/hardware/hand_stream_bench.py`**

- **The gap stage's `hand_override` now reaches `frame()`.** The one-line root
  cause, documented at the call site with the measured cost (echo frozen at
  0.00018 rev for 1200 rows, 0.0168 mm of encoder excursion, against a logged
  1.0 rev step) so the next reader sees why the line is load-bearing.
- **A class-closing positive check.** After the re-entry knot the firmware's
  hand echo **must move ≥ `GAP_ECHO_MOVE_REV` (0.05 rev ≈ 1.6 mm) within
  `GAP_ECHO_DEADLINE_S` (0.25 s)** or the stage aborts. Both constants are
  derived in-file rather than picked:
  - **MOVE** — the gap stage rides a Hold, so the pre-re-entry echo is
    **static by construction**; 0.05 rev is 5 % of the default 1.0 rev
    re-entry: unmissable if the knot arrived, unreachable if it did not. The
    arming is skipped for a re-entry too small to be distinguishable
    (`|gap_delta| > 4 × GAP_ECHO_MOVE_REV`).
  - **DEADLINE** — uplink ≤ ~1.5 ms + one 25 ms knot to Mode-1 onto the
    re-entry + the first 500 Hz tick (≤ 2 ms) + one telemetry tick
    (≤ 10 ms at 100 Hz) + downlink ≤ ~1.5 ms ≈ **40 ms honest worst case**.
    0.25 s is ten knots, ~6× that, so the check **can only fire on a re-entry
    that never happened, never on a slow one**.
  - The abort message is diagnostic, not just loud: it names the two
    distinguishable causes and how to tell them apart from `[hand7]`
    (`src=STREAMED lane=active` with `sent=` climbing ⇒ the driver dropped the
    override again; `src=LEGACY` with `discard_legacy` climbing ⇒ the latch,
    not the driver).
- **An `enc_age` column in the stage CSV.** The belt is
  `|cmd − (enc + vel·age)|`, whose `vel·age` term is worth ~0.7 rev at the
  stroke's ~95 rev/s plateau against a 1.5 rev bar — so **without the column
  the belt is not reconstructible from its own CSV**, and an analyst has to
  *solve* for age rather than read it. That gap is precisely what blocked
  attributing finding (2) above. Empty (not 0.0) when no telemetry is cached,
  on the same convention as `echo_rev` / `recon_mm`.
- **`t_local_us` carried on the CacheDiag rows** (and into the companion CSV).
  The `t` column is the *driver's* monotonic arrival stamp and carries UDP
  delivery smear, so a drop episode could not be placed on the bridge's own
  timebase — the one the stage CSV's echoes are stamped in. Carrying the
  frame's own stamp makes an episode alignable against the stroke it belongs
  to.

**`tests/hardware/session_unified7_hand_bringup.md`**

- **New row 11c — a CAPTURED console for the whole sitting**, started at row 6
  and left running to row 21, with the verified command
  (`script -f … -c "pio device monitor …"`) and the two rejected alternatives
  and *why* (Discussion 4). It enumerates every console-only counter, states
  that none of it reaches a CSV/uplink/bag, and requires pasting the current
  `[hand7]` line between rows — counters are boot-cumulative, so **an
  un-bracketed row has no value at all**.
- **Numbered `11c`, NOT a renumber.** Rows 12–21 are cited **by number** from
  `logbook/2026-09-02-unified-7dof-planner-phase3-fw17-hand-lane.md:156,223`.
  Same reasoning that produced 11b last sitting, same failure class as the
  plan-filename convention in CLAUDE.md.
- **Row 6** now points at 11c's command instead of a bare `pio device monitor`,
  so the boot banner and rows 9–10 land in the same capture as the ladder.
- **Row 12 now requires establishing LEGACY first.** `hand_source.cpp:56`
  returns OK on `source == s_source` *before* the mpc_active, freshness and
  settle gates, so asserting `streamed` while already STREAMED proves nothing
   — the idempotent-OK false pass this sitting hit.
- **Row 13's 600 s is justified, not asserted** — three separate arguments:
  `dev_max` is an **extreme-value** statistic and it is the number sitting
  three arms the guard against, so a 5×-short window systematically
  under-estimates the tail; the drop-episode criterion needs enough windows to
  have a rate (128 vs ~600); and `defer jb` / `txq jb` are **rare-event**
  counters whose `0` is only as strong as the exposure behind it.
- **Row 15 gains the `recon_mm / analytic_velocity` discriminator** (constant
  across velocity ⇒ clock/age bias, not passable as wall-sync noise; scaling
  with velocity ⇒ the Hermite term the 3.25 mm bar was derived for), plus the
  warning that **`recon_mm` carries information on ~8 of 1200 rows**, so any
  median or p90 of the column reads 0.000 and is meaningless.
- **Row 17 gains the positive echo-movement evidence requirement** and names
  the driver bug that made it necessary.
- Rows 13/14/16/18/19 now say explicitly that their `[hand7]` deltas are read
  from **row 11c's capture — this row's evidence of record**, and the
  Artifacts section names the capture file as the sitting's primary
  deliverable.
- **The row-15 hard-abort rule is AMENDED, in Safety and in the row** — the
  counter-gating defect above makes *"`lead` must stay 0"* unfalsifiable as
  written. Until the firmware fix lands, the rule reads: **difference `lead`
  across the stage; a non-zero absolute is uninformative** and specifically is
  *not* grounds to abort. The rule's teeth are unchanged where it matters — a
  non-zero **delta** during a throw is still a hard abort — and the amendment
  is dated and attributed so a future reader sees it is a stopgap, not a
  relaxation.
- **A § Results section**, one row per ladder row, recording this sitting's
  verdicts (PASS / CARRIED) and naming this entry as the canonical record. It
  states what carries to sitting three and why: row 18's arming half
  (thermal — the operator stopped correctly, and it is sitting three's step by
  the doc's own § Observe-then-arm), row 19(b) (not run), and row 17's decay
  half (structurally untestable with the current stages).

**Not fixed here, and it cannot be:** the `lead` / `dev_over` output-enable
gating defect is a **firmware** change and therefore **FW 18 / sitting-three
work** — the fix is three tokens, the deployment is another lockstep-class
flash. Recording it as a named defect with its fix site is the whole of what
this session can do about it.

**`plans/active/unified-7dof-planner.md`** — Phase 3 recorded **COMPLETE
2026-09-04** (owner's declaration) in the § 3 status table, the phase heading,
the § 5 hardware-ladder rows, and a close-out paragraph covering both sittings.

No firmware change. No reflash. No host-node change.

## Verification

- (2026-09-04, `python -m py_compile tests/hardware/hand_stream_bench.py`,
  **exit 0**).
- (2026-09-04, `python -m pytest tests/sim/test_plans_index.py
  tests/sim/test_logbook_search.py tests/sim/test_logbook_front_matter.py -q`,
  **108 passed in 0.68 s**).
- Row-15 finding (3), the arming-decision number, reproduced from the stage
  CSVs: (2026-09-04, `python /tmp/probe_stroke_lead_margin.py`,
  **09:56 worst `|cmd − enc|` 1.2067 rev / peak enc 10.0759 rev; 14:47
  1.9847 rev = 99.2 % of `MAX_LEAD_HAND_REV` 2.0 and 79.4 % of
  `MAX_DEVIATION_HAND_REV` 2.5 / peak enc 10.4693 rev**). One-off probe,
  uncommitted per `tools/probes/README.md`.
- Re-run of the same three suites **after** this close-out's doc edits (this
  entry, the runbook § Results + row-15 amendment, and the plan's Phase 3
  rows): (2026-09-04, `python -m pytest tests/sim/test_plans_index.py
  tests/sim/test_logbook_search.py tests/sim/test_logbook_front_matter.py -q`,
  **108 passed in 0.68 s**).
- Pre-commit gate: (2026-09-04, `./run_tests.sh`, **6392 passed / 4 skipped in
  257.52 s parallel + empty serial phase, total 270 s — PASS**). `--full`
  deliberately not run: the owner directive scopes it to `controller/`/`sim/`
  diffs and this change touches neither (`tests/hardware/` + narrative only).
  Runbook row 1's `./run_tests.sh --full` before sitting three stands separately
  and is the operator's.

**Test-coverage honesty.** **No pytest test reads `hand_stream_bench.py` or
the runbook.** The triples above establish that the file still imports and
that the logbook/plan-index surface is intact; they do **not** exercise the new
gap check, the `enc_age` column, or any runbook row. **The real gate for these
two files is the bench sitting itself** — which is exactly why the gap fix is
written as a loud, self-explaining abort with a remedy in the message rather
than as a silent tightening.

## Outcome

The 2026-09-03 de-energised class is **closed**, twice over, and FW 17's hand
lane behaved correctly on every row that produced usable evidence: a 10-minute
hold flat at 0.041 mm of worst-tick deviation, a clean triangle, a live
confirmation of the 5.0 rev pump derivation, a bounded single-move gap
re-entry, an `ERR_HAND_SOURCE` refusal proven to precede any CAN side-effect,
and a spotless close-out sweep.

**Ladder state:** rows **12, 13, 14, 16, 17 (re-entry half), 19(a), 19(c), 20,
21 PASS**; **CARRIED to sitting three**: row 18's arming half (thermal — the
operator stopped correctly), row 19(b) (not run), and row 17's **decay half**,
which is untestable with the current stages.

**The owner declared Phase 3 COMPLETE on 2026-09-04** on that record — the
lockstep flash is discharged, the streamed hand lane is validated on real
hardware, and what remains is scheduled into sitting three rather than blocking
the phase. `plans/active/unified-7dof-planner.md` carries the close-out.

**The number that governs the arming decision, stated once more because it is
the thing a future reader must not miss:** it is **not** the 10.9794 `dev_max`
on the console line — that is dead boot-cumulative history, gated wrong, and
structurally incapable of tripping an arm (Diagnosis §§ 2–4 above). It is the
**afternoon stroke's 1.9847 rev of worst `|cmd − enc|` — 99.2 % of
`MAX_LEAD_HAND_REV` and 79.4 % of the 2.5 rev deviation band**, on a nominal
stroke at the same `--event-vel` where the morning read 1.2067 rev, with the
encoder overshooting to within **0.33 rev of the 10.8 rev hard stop**. Sitting
three should **arm expecting a trip**, and should know that once armed an
**aborted** stroke fires the guard by design (the lane decays, the residual
passes 2.5 rev within ~30 ms of the last knot) and needs `CLEAR_ERRORS`.

The sitting also produced three things worth more than the passes: **a stage
that had been measuring a fiction, now fixed with a downstream check**; **a
real, unexplained tracking degradation on the stroke** whose guard-relative
form is the number above; and, from the first `[hand7]` capture this project
has ever held, **a documented duty counter that does not measure duty** —
`lead` and `dev_over` count ticks on which the firmware transmitted nothing, so
the runbook's hard-abort rule was unfalsifiable as written. None of the three
would have surfaced from the pass/fail verdicts alone.

## Withdrawn claims

- **[2026-09-04, mid-sitting] Claimed a drop-rate regression** — 5.3 % of
  windows with an episode against a 1.6 % 2026-08-30 baseline (~3.3×), with a
  67-frame drought on axis 6 flagged as concerning. Asserted to the operator
  before the normalising check.
  **WITHDRAWN:** the 2026-08-30 baseline is **legs-only** (its arm-table means
  divide by 6 — 0.78 × 6 = 4.68 ≈ the quoted 4.65 f/s; its qualification 3
  records four ax-6 episode windows in arm B, impossible under an any-of-7
  count) while the driver's counter is **all-7**
  (`hand_stream_bench.py:887-890`). Normalised: legs-only 29/601 = 4.83 % vs
  2/124 = 1.61 %, **p = 0.162**; all-7 32/601 = 5.32 % vs 6/124 = 4.84 %,
  **p = 1.000**. Continuous statistics agree (3.48 drops/s over 6 legs, inside
  the 1.8–5.0 historical band; per-axis-window −0.580 ± 5.126 vs a
  −3.6 ± 10.3 reference), episodes are stationary and *under*-dispersed
  (index 0.778), `rx_depth_hwm_jb` flat at 9/256 across all 601 windows — and
  **axis 6 is the best-behaved axis in the soak** (net loss 0.446 %, sd 3.590,
  3 episodes; ~13× better per unit time than the arm that raised the worry).
  **No regression is demonstrated.**
  **Superseded by:** Diagnosis § "The drop-rate scare, normalised away".

- **[2026-09-04, morning] Kept the firmware `interp_last_tick_us()`
  age-correction (`telemetry.cpp:275-277`) live as a candidate cause of the
  2.17 ms one-sided reconstruction bias**, on the strength of the morning
  block alone (8 moving samples, 2.168 ± 0.085 ms across a 6.01× velocity
  span, fit A = +0.064 mm / B = +2.120 ms).
  **WITHDRAWN:** the bias **did not reproduce**. The afternoon block has the
  same form — constant ms across velocity, zero position term — at
  **0.3492 ± 0.0016 ms** (A = 0.000 mm / B = +0.3492 ms), **6.2× smaller**,
  across a 2.1× span covering both accel and decel. **Deterministic firmware
  on an unflashed board would give the same offset twice.** The evidence points
  at a residual host↔bridge wall-sync offset. *Caveat:* this requires no bridge
  reboot between the runs, and no CSV column carries uptime, so it could not be
  checked.
  **Superseded by:** Diagnosis § "Row 15, finding (1)".

## Open Questions

Closed since the sitting, so a future reader does not re-open them: rows **12**,
**19(c)** and **21** (Symptoms); the alarming `[hand7]` counter values, which
are fully explained and cannot trip an arm (Diagnosis §§ "The first console
capture ever taken", "Arming will not trip"); and *"is `auto_arm:=false`
required for row 19(c)"* — it is not, though it remains worth having
(Discussion 5).

1. **THE ARMING-DECISION NUMBER — the 2.5 rev band will be tight, budget for a
   trip.** The afternoon stroke's worst `|cmd − enc|` was **1.9847 rev =
   99.2 % of `MAX_LEAD_HAND_REV` 2.0 and 79.4 % of `MAX_DEVIATION_HAND_REV`
   2.5**, against **1.2067 rev** on the morning stroke at the same
   `--event-vel`; the encoder overshot to **10.4693 rev** against the 10.8 rev
   hard stop. This **supersedes the 10.9794 `dev_max`** as the number sitting
   three's `hand7 arm` decision rests on. Caveat carried with it: the 40 Hz
   CSV's raw residual is coarser than the firmware's velocity-compensated one
   (which shrinks it) and undersamples a 500 Hz quantity (which grows it).
   Second-order consequence to expect rather than debug: **armed, an aborted
   stroke fires the guard by design** — the lane decays and the residual passes
   2.5 rev within ~30 ms of the last knot — and needs `CLEAR_ERRORS`.
2. **`lead` / `dev_over` are gated wrong — a FIRMWARE fix, FW 18.** Both are
   documented as throw-**duty** counters (`canbridge_config.h:256-259`,
   `leg_interp.cpp:196`, runbook row 15) but their gate
   (`leg_interp.cpp:684`, `s_hand_active && hand_source_streamed()`, plus
   `hts != 0` at `:758`) omits `s_output_enabled`, so they count ticks on which
   nothing was transmitted. Fix: add `out_en` at `:792-799` / `:814-815`, or
   add an output-enabled-only counter pair and make that the duty read. Needs a
   reflash; the runbook rule is amended as a stopgap in the meantime.
3. **There is no runtime reset for the `[hand7]` counters** — the console
   handler (`leg_interp.cpp:1213-1223`) takes only `arm` / `observe` / bare
   `hand7`; `interp_reset()` (`:1032`) zeroes them but has no runtime caller;
   `CLEAR_ERRORS` does not touch them and `REBOOT_ODRIVES` reboots the ODrives,
   not the Teensy. **Method, not a blocker:** the lockstep flash *is* the
   reboot — take the first post-flash line as the true zero, difference across
   every stage, never read an absolute. A `hand7 reset` verb would be a
   one-line addition to the same FW 18 change as item 2.
4. **The stroke deviation growth is OPEN** — 0.5387 → 1.2348 rev against a
   1.5 rev belt (**18 % margin**), confirmed by two age-free measures
   (crossing-time lag +12.25 → +18.11 ms; end-of-stroke overshoot
   3.68 → 16.13 mm). Starting offset and telemetry age are **ruled out**. The
   **stage-ordering** hypothesis (exercised hand vs first stage on a hand that
   sat de-energised) is unestablished and is the cheapest next test: re-run
   the stroke both ways in one block. Item 1 is this growth read against the
   guard constants; they are the same phenomenon, not two.
5. **The commanded 5.31 mm firmware-target overshoot**, inside the FW 17 hand
   lane, on identical knots and an identical 25 ms grid with `lead_mask = 0`
   (+0.1678 rev vs the morning's +0.13 mm). About a third of the encoder
   overshoot was *commanded*. This is a firmware-lane question, distinct from
   item 4's plant question.
6. **The falling-edge decay has never been observed.** The gap stage rides a
   Hold, so **no current stage can exercise it** — the hand is at rest when
   the gap opens (max |vel| 0.073 rev/s, 6.6 µm of encoder span). Needs a new
   stage that takes the gap **while the hand is moving**, which is also the
   only way to test the normative rule Phase 3 shipped. **Carried to sitting
   three** and recorded as such in the runbook's § Results.
7. **A BRACKETED `[hand7]` capture still does not exist.** Row 11c is landed
   and a capture finally happened, but at 15:39 — after the last block closed
   at 15:09 — so it carries state and history and **no per-row delta**
   (Discussion 4). Sitting three is still the first that can produce the
   observe-first residual record from the counters themselves rather than the
   CSV-derived substitute.
8. **No CSV column carries bridge uptime**, which is what prevented closing the
   timing-bias argument outright (Withdrawn claim 2). An `uptime_ms` column
   would have settled it from the artifact alone.
9. **Arm refusals are never logged** — `_arm_setpoint_output`'s docstring
   claims reject reasons are surfaced *and* logged; only the service response
   carries them. An orchestrator retry loop can burn its 10-attempt budget
   silently before FAULTing. The same hole is why row 19(c)'s `ERR_REJECTED`
   is an **inference** from the latch not moving rather than a capture:
   rosbag2 records no services, and nothing is logged.
10. **`decode_bad_axis` on the jugglebot bus climbs steadily at ~2.0/s** —
    119 730 over ~16.6 h of uptime in the capture, at the same rate across all
    264 blocks. A steady benign source (a node id ≥ 7 on the jb bus), constant
    and self-consistent, **unexplained**. Not a blocker for any row, and it
    moves no other counter (`short = 0`, `err = 0`, `hwm` flat at 9), but a
    steady 2 Hz of discarded frames has an owner somewhere.
11. **Axis 5's ~134.5 s drop periodicity** (135.2 / 135.2 / 134.3 / 133.2, with
    axis 4 weakly at ~130–140 s) — too slow for a bus or ISR mechanism, looks
    ODrive-scheduler-side. **Belongs to `plans/active/leg-bus-frame-drops.md`,
    not to this entry.**
12. **The four owner items carried from the first sitting's entry** remain open:
    `--close-loop` as the default; `SET_VEL_CURR_LIMITS` on hand arming;
    IDLE-on-exit; and the homing trap (CLOSED_LOOP + VELOCITY/VEL_RAMP, which
    no telemetry gate can distinguish from a stream-ready hand).
