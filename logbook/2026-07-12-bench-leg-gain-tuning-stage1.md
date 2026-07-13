---
title: Bench-leg gain-tuning Stage 1 — bench system-ID, the overturned 110 winner, and the motion-excited-vibration instrument gap the operator's ears exposed
type: investigation
date: 2026-07-12
status: superseded  # 2026-07-13: accuracy knee + Stage-2 recommendation retracted (units bug)
superseded_by: 2026-07-13-leg-plant-id-and-the-units-bug.md
phase: "S4 gain-retune (Stage 1, bench) — CLOSED"
related_plan: leg-gain-tuning-methodology.md
files_changed:
  - tests/hardware/bench_leg_sysid.py
  - tests/hardware/sysid_lib.py
  - tests/motion/test_bench_sysid_bridge.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/platformio.ini
  - ros_ws/src/jugglebot/Teensy_code_canbridge/README.md
commits:
  - 1e03348
  - 0a271f8
  - 30cc786
  - 8cbf4f0
  - db05670
  - 0182a11
subsystem:
  - can
  - motion
tags:
  - safety
  - performance
  - testing
  - dynamics
---

# Bench-leg gain-tuning Stage 1 — bench system-ID, the overturned 110 winner, and the instrument gap the operator's ears exposed

## Summary

Stage 1 of the two-stage fast-motion gain retune (`plans/active/leg-gain-tuning-methodology.md`,
the "Fast-motion tier"): use the acceptable-loss 7th bench leg — same motor/ODrive/actuator
class as a platform leg, driven only via the can-bridge Teensy on CAN3 (Path BRIDGE) — to
(a) measure the leg-servo plant, (b) climb an escalate-until-unstable gain ladder, and
(c) settle the S4 servo-vs-structure question by reproducing (or failing to reproduce) the
`2026-07-10` ~6 Hz stutter on an isolated leg with the platform removed.

The day ran as four analysis rounds, each analysed by parallel Opus tracks and **each claim
adversarially verified against the raw CSVs and source** before it was trusted. The headline
outcomes: the unloaded bench leg **never reproduced the robot's ~6 Hz limit cycle in any
regime** — not on the ladder, not in sustained tracking, not even riding the lead clamp at
46 %; the first "winner" the ladder produced (**110/0.50/0.72**) was **overturned** — first by
the operator hearing motion-excited high-frequency vibration the quiescent + settle instruments
structurally could not see, then confirmed by a chirp peak/DC damping estimate of only
ζ≈0.49–0.52 at the 17.5 Hz crossover; and the bench envelope resolved to **pos 40 clean /
pos 70 accuracy knee / pos 90 aggressive-but-bounded edge / pos 110+ over the line**. The
recommended Stage-2 robot start is **70 / 0.35 / 0.56** with `MAX_LEAD` re-pinned to
`4.0/pos_gain = 0.057`, gated by a **loaded S4 replay on the real robot under v3 firmware**
(first at production 40, then the candidate). Six commits landed: one harness-unblock fix, one
big Run-A harness + Run-B bench-firmware upgrade, three feature/fix commits from the analysis
rounds, and a same-day follow-up.

> **⚠️ SUPERSEDED 2026-07-13 — read this before trusting any number below.** The **"accuracy
> knee"** and the **Stage-2 recommendation** in this entry are **retracted**. The stroke-error
> table was in **milli-revolutions, not millimetres** (`bench_leg_sysid.py:2548` scaled by `1e3`
> instead of the bench leg's `mm_per_rev` = 71.5708) — a **~14× inflation**. The real tracking error at production
> gains is **0.74 mm RMS**, and **0.054 mm at the catch instant** — the metric the spec is judged
> on — so **no gain change is warranted** and production `40 / 0.20 / 0.32` stands (`MAX_LEAD` stays
> 0.10). A plant-ID on this session's
> own CSVs further shows `J_eff` ≈ `J_rotor` (refuting the "~10× rotor" guess) and a production
> cascade of `ω_v/ω_p` ≈ 3 — already healthy. **The gain hunt is closed**; the effort moved to
> feedforward.
>
> **What remains valid and load-bearing:** the **stability envelope**, the **servo-vs-structure
> discriminant** (the 6 Hz did not reproduce on the isolated bench leg in any regime), the
> **friction describing-function** finding, the **measurement-ceiling lessons**, the
> **instrument-gap lesson** (§c — which this correction only reinforces), and every **harness and
> firmware fix**. See `logbook/2026-07-13-leg-plant-id-and-the-units-bug.md`.

**Probe data (cite, do not reproduce):** `temp/probes/bench_sysid_20260712_*/` —
round 1 (afternoon, stock v3, 40 Hz knots): `152819 / 153007 / 153054`;
round 2 (evening, bench-firmware cut-over mid-sequence): `175734 / 175857 / 180258 / 180335`;
round 3 (surveys + winner chirp/track): `190303 / 190334 / 190432 / 190502 / 190809 / 190912`;
round 4 (strokes battery + vel_int de-confound): `222801 / 223056` (+ `223152 / 223542`).

## Symptoms

- **Parent problem (context).** The `2026-07-10` evening S4 session showed a sustained
  **5.9–6.1 Hz leg-servo limit cycle** on all six legs (2nd harmonic ~12.3 Hz), speed-independent
  (corr 0.06 over 37 moves), ≈ `pos_gain/2π = 40/6.28 = 6.37 Hz`, with the ODrives actively
  driving it (iq slamming −3.8 → +5.9 A cycle-by-cycle) and a legitimate `MAX_DEVIATION` latch.
  See `logbook/2026-07-10-s4-stutter-guard-forensics-recovery-stack.md`. That entry left the
  loop-vs-structure question explicitly open ("the one question this bag cannot settle — single
  gain point") and handed it to a bench session. This is that session.
- **Immediate blocker.** The bench harness kept false-aborting a third time —
  *"output not engaged — tracked 0.0750 of 0.2190 rev"* — even though the operator watched the
  leg track three position steps cleanly in the ODrive GUI.
- **Mid-arc.** At the ladder-winner gains the operator could **see and hear** high-frequency
  vibration during motion that every instrument on the harness reported as clean.
- **Late.** The sustained-stroke battery latched `MAX_DEVIATION` at **every** gain point, and
  the spacemouse teleop mode grabbed the wrong stick when two were plugged in.

## Diagnosis

### 1. The false-abort was a stale-baseline arithmetic bug, not transport (commit `1e03348`)

Transport was exonerated *first*, by a loopback before touching the harness: exactly 40.0 Hz
frame cadence, monotonic seqs, zero drops, sustained heartbeat. The abort was a baseline
mismatch inside `_verify_output_engaged`:

1. the bench leg homed to **−0.069 rev, below the firmware stroke floor** `STROKE_MIN_REV[0]=0.0709`,
   so the settle frames were clamped *up* toward the floor while the harness still held the
   stale pre-settle sample;
2. the engagement-probe target was computed as `bounds.clamp(home + 0.03) = 0.15`, silently
   inflating the intended 0.03 rev nudge to **0.219 rev** measured from that stale home;
3. the commanded delta used the stale baseline while the encoder delta used the live position →
   a fictitious `track_frac ≈ 0.34` → abort.

Fix: **all baselines are now live encoder** (`engagement_probe_target(enc0, …)` clamped on the
high side only; `_enter_hold_at_center` re-samples after the settle). This unblocked the harness
after two earlier same-day fixes on the same abort class (`2b4b507` startup-latch-clear +
stream-then-arm; `80d8f36` armed-settle-before-approach-ramp).

### 2. Round 1 — four verified verdicts on the first clean data

The first clean `pos_steps / --mode all / ladder / chirp` runs (`152819 / 153007 / 153054`)
were analysed by four Opus tracks, each adversarially verified:

- **The ladder stop at pos_gain 90 was TABLE EXHAUSTION, not a ceiling.** `stop_reason=ladder_top`,
  `guard_recoveries_used=0` — the hard-coded 5-rung table (`sysid_lib.py` default_ladder) simply
  ran out of rungs. **Observability was broken three ways** so the *real* ceiling was unmeasurable:
  (i) iq was on-change-aliased (the stock DIAGNOSTIC frame fires only on iq-setpoint change >0.5 A
  or a 1 Hz heartbeat → per-rung |iq| peaks 3.2/2.9/8.2/2.4/8.8 A, non-monotone); (ii) the CSV was
  decimated to the 40 Hz knot rate (Nyquist 20 Hz) even though the UDP telemetry ran at 100 Hz;
  (iii) the ladder co-climbed `vel_int = pos_gain/125` and `vel_gain` *with* `pos_gain`, so the
  clean zeta climb (0.702 → 0.826) tracked the **velocity loop**, not pos_gain in isolation.
- **The chirp gain of ~0.5 at 0.02 rev is REAL Coulomb-friction small-signal attenuation, not an
  estimator bug.** Four independent tracks converged: coherence 0.96–1.0 across 1–7 Hz; the
  encoder physically rotates half the commanded 2.86 mm p-p; large steps (0.07–0.28 rev) reach DC
  gain 1.01–1.03 while the tiny chirp is friction-limited to ~0.5 (a 2× gain gap between large- and
  small-signal excitation is by definition a nonlinearity). The `2026-04-27` friction bench
  (`tau_c = 1.094 A`) is corroborated within 10–40 %.
- **The robot's ~6 Hz did NOT reproduce.** Every step/ladder tail settles dead-flat within ~4
  frames at the encoder quantization floor; there is no sustained AC energy to fit a ring
  frequency to across rungs, so the servo-vs-structure discriminant was **unanswerable from this
  data** — arrive-and-settle stimuli are the wrong excitation class. The robot fails during
  *sustained* ~2 rev/s strokes with the lead clamp engaged.
- **The honest chirp ceiling of 8 Hz is KNOT-bound** (`knot_stream_top_freq = (1/0.025)/5 = 8 Hz`),
  not telemetry-bound — enough to resolve the 6 Hz pos-loop question but not a structural mode
  above it. Closed-loop bandwidth ≈ `pos_gain/2π` (40 → 6.4 Hz, 90 → 14.3 Hz), cross-checked
  against the S4 forensics and `docs/knot-rate-analysis.md`.

### 3. Round 1 → Run-A harness + Run-B bench firmware (commit `0a271f8`)

The Round-1 verdicts drove a coordinated upgrade so the *next* runs could see what the first
could not:

- **100 Hz CSV logging** (one row per *received* telemetry frame + a lead-clamp column),
  **servo-limited step ramps** (0.9× lead margin = 3.6 rev/s, so rise measures the loop not the
  ramp), a **multi-amplitude chirp** (`--chirp-amps` 0.02/0.06/0.12 + Welch H1 + per-bin
  coherence + mean-removal), an **extended ladder** (rungs 110/130/155/180/210; `vel_int` frozen
  at 0.72 above 90; `vel_gain` candidates scaled `√(pg/90)`; a per-rung quiescent-hold buzz gate;
  revert-to-BASELINE + stop-on-first-latch at pg ≥ 130), a new **`--mode track`**
  sustained-tracking discriminant, and a `--knot-hz {40,100}` knob.
- **A `BENCH_SYSID_BUILD` firmware variant** (default OFF, operator-gated, **never** flashed to
  the assembled robot): `TELEM_RATE_HZ` 100 → 250, `SEGMENT_T_S` 0.025 → 0.010 (100 Hz knots),
  axis-0 DIAGNOSTIC forced every telemetry tick (250 Hz un-gated iq). **The flag-OFF production
  binary is byte-identical** (`firmware.hex` sha256 `2d504300…` unchanged pre/post edit).
- **5 review findings fixed pre-commit.** The HIGH: frame steps were fixed rev/frame constants
  whose velocity scaled with `--knot-hz` (0.09 rev/frame = 9.0 rev/s at 100 Hz, past the 4.0
  vel_limit) → replaced with velocity-anchored sizing (identity at 40 Hz, velocity binds at 100 Hz).

### 4. Round 2 — knot rate is a MEASUREMENT change, not a control change

The evening runs (`175734 / 175857 / 180258 / 180335`, operator flashing the bench variant
mid-sequence) resolved several confounds:

- **Knot rate does not change loop dynamics.** Same-gain, ladder pos_gain-90 ζ = **0.826 at
  40 Hz knots vs 0.816 at 100 Hz knots** — identical within noise. What the 100 Hz/250 Hz build
  changed was the **instrument** (chirp ceiling 8 → 20 Hz, 250 Hz iq/vel) and the **actuation
  texture** (100 Hz knots → less staircase in the 500 Hz Hermite command — the most likely source
  of the operator's "some configs less smooth" observation).
- **The 18:03 `--mode all` suite ran at BASELINE 40/0.2/0.32, not the ladder winner.** The
  working assumption (carried in the orchestrator's mid-stream context and echoed by one Round-2
  track) was that it characterized the winner. The parallel gains-and-loop track plus its
  adversarial verify **refuted this and code-confirmed the mechanism** at
  `bench_leg_sysid.py:1800` — `_bringup_closed_loop` unconditionally re-applies `BASELINE_GAINS`,
  so `pos_steps`/`chirp` can *only ever* measure the production loop. The apparent ζ 0.65 → 0.87
  "improvement" vs yesterday was the instrument (250 Hz telemetry resolving a 47 ms rise
  yesterday saw with ~2 samples), not the loop.
- **No limit cycle in any of the 10 track runs.** The only discrete sub-drive peak *moves with
  drive frequency* (2.4 Hz at f_drive 0.80; 5.2 Hz = 3× at f_drive 1.75) — a forced clipping
  harmonic, not a self-excited fixed-frequency cycle. The 39–41 Hz peaks are the 40 Hz knot
  sample-and-hold staircase (96–98 % removed by a 15 Hz LP = logging artifact). The `p55
  clamp_frac = 0.000` anomaly is genuine (V-shaped clamp engagement; raw cmd-pos peaks 0.1014 rev).
- **v2/v3 clamp context.** The S4 6 Hz ran on **v2** firmware (vel_ff zeroed at clamp engage,
  MAX_LEAD 0.15); **v3** keeps vel_ff (MAX_LEAD 0.10). On v3 the bench leg **rode the clamp to
  clamp_frac 0.456 with no limit cycle** — consistent with v3's vel_ff-keep having removed the S4
  aggravator *on the unloaded leg*, but this does **not** certify the loaded 6-leg robot.

### 5. Round 3 — the 110/0.50/0.72 winner is overturned

The operator's ears started this round; the numbers finished it (surveys `190303/190334`, winner
chirp/track `190809/190912`):

- **Effective damping is ζ ≈ 0.49–0.52 at the 17.5 Hz crossover** (chirp peak/DC ratio, coherence
  1.00) — far below the step-fit ζ 0.63–0.78 and below the 0.7 target. **Step fits overstate
  damping** because ramp-rate-limited steps never excite the 15–19 Hz resonance the chirp reveals.
- **ζ is non-reproducible:** 0.78 on the first ladder → 0.66–0.68 across three repeats tonight.
- **Sustained motion at these gains carries 3.7× baseline 20–45 Hz iq energy** (2.17× vel, verifier-reproduced),
  proven **closed-loop, not the knot staircase** (the 94–106 Hz staircase velocity band is equal
  between p110 and p40; the 20–45 Hz excess is not).
- **iq railed >9 A for 1.26 % of the 4.4 rev/s track** (current saturation, `clamp_engaged_frac=0`),
  where the identical track at production gains never rails.
- **pos_gain is the proven buzz driver** (monotone quiescent vel-RMS; onset at 130) **but only
  conditional on vel_int = 0.72**, which was frozen at all eight survey points — pos_gain and
  vel_int are confounded.

The quiescent-hold gate and the arrive-and-settle steps **structurally miss motion-excited
vibration**; the operator's hearing was the only instrument that saw it until the HF detector was
built.

### 6. Round 3 — the survey/teleop/strokes tooling (commits `30cc786`, `8cbf4f0`)

- `30cc786` added **`--rungs 'pg:vg:vint,…'`** (explicit single-candidate rungs through the same
  quiescent-buzz → step → onset pipeline, in a *survey* mode that records each verdict and
  **continues** rather than backing off), **`--gains`** (an override triple applied at bringup so
  a run's gain state is unambiguous), and **`--quiescent-secs`** (a soak override for thermal-onset
  checks).
- `8cbf4f0` added the **motion-excited HF onset detector** (windowed 20–45 Hz vel/iq band-RMS,
  knot-staircase notched, `|iq|` rail duties, advisory flag), **`--mode strokes`** (a min-jerk
  amplitude × peak-vel battery + asymmetric throw/catch + a sustained S4-class sequence), and
  **`--mode teleop`** (spacemouse-Z → leg position, slew-limited, live gain reconfiguration).
  Adversarial review confirmed **3 HIGH, all fixed pre-commit**: the near-rest gain-swap gate ran
  at *enqueue* time while the deferred worker's RPCs land up to ~1.5 s later mid-yank → **stick
  freeze while an apply is in flight**; a guard latch during an in-flight apply could interleave /
  lose the BASELINE revert → **a dedicated apply-lock + latch epoch make BASELINE provably the last
  gain write**; stroke HF was diluted ~0.7× by the two 0.4 s stationary holds → **per-moving-window
  metrics**.

### 7. Round 4 — strokes battery + vel_int de-confound (the decisive round; `db05670`, `0182a11`)

- **The sustained-stroke triple-latch was a HARNESS BUG, not a gain instability** (`db05670`). The
  sustained sequence alternates center+A → center−A (travel **2A**) but timed every swing with
  `T` computed for distance **A** → commanded peak `1.875·2A/T = 7.6 rev/s` at A=1.0/v=3.8, past
  the 4.0 vel_limit. Infeasible **by construction**: the leg fell ~0.5–0.7 rev behind the streamed
  u0 and latched `MAX_DEVIATION` at **every** gain point (dev ≈ −0.53/−0.66/−0.68 rev, `|dev|>0.5`
  firing ~0.78 s in), the third latch exhausting the recovery budget. Fix: size the sustained spec
  by the 2A swing distance and scale each segment's duration by distance; plus a **series-level
  velocity gate** (`series_peak_velocity`) that loudly skips any stroke commanding >5 % over the
  cap — **closing the whole bug class** (a future generator sizing error becomes one skipped stroke,
  not a guard-budget-burning triple latch).
- **The two baseline HF onsets are commanded-content FALSE POSITIVES.** They occur only on the
  99 ms strokes, which carry 10–56× more in-band commanded energy than long strokes, and their
  velHF **falls** with gain (0.173 → 0.127 → 0.063) — the *anti-vibration* signature of a stiffer
  loop tracking a fast short move better. **Real vibration grows with gain on long strokes** at
  cmdHF ≈ 0 (iqHF to **5.6× baseline at pos 90**, e.g. 0.217 → 1.209 A on a 345 ms stroke). The
  clean discriminator is the **pos-to-pos gain delta on identical stroke geometry**, where the
  commanded content cancels exactly — not a per-sample residual.
- **The accuracy knee is pos 70.** Battery mean errRMS 10.3 → **6.9 mm (−33 %)** from pos 40 → 70,
  while pos 90 buys only **~1 % more** accuracy for 2.4× iq churn and stronger real vibration.
  > **⚠️ 2026-07-13 — THE UNITS HERE ARE WRONG AND THIS CONCLUSION IS RETRACTED.** These values
  > are **milli-revolutions**, not millimetres: `bench_leg_sysid.py:2548` printed `err_rms * 1e3`
  > under an `mm` label, but `err_rms` is in **revolutions** (`cmd_rev` − `pos_rev`). The correct
  > scale is the bench leg's `mm_per_rev` = **71.5708**, so the figures are **0.74 → 0.49 mm**, ~14× smaller. The −33 %
  > ratio survives (scale-free); the *decision* does not — a 0.25 mm gain is irrelevant against the
  > operator's ±1 mm spec, and production `40/0.20/0.32` already passes it. See
  > `logbook/2026-07-13-leg-plant-id-and-the-units-bug.md`.
- **pos 130 is dead at all three vel_int {0.72, 0.55, 0.40}** — hair-trigger and intermittent
  (ζ 0.690/null/0.647; buzz non-monotone in vel_int, fired only on the middle rung at 1 % over the
  gate; a rung that logged "unstable" live sat at oscillation-score 0.494 offline, just under the
  0.50 gate; and across the same-evening round-3 surveys 130/0.45 buzzed in one run and was clean
  in the other while 130/0.50 buzzed in both — the round-4 de-confound then found the 0.50/vint-0.72
  point buzz-clean but step-unstable). **The boundary itself is the finding** —
  vel_int is not a clean rescue knob at 130.
- **Two harness fixes:** the teleop spacemouse device-selection bug — two mice, **five identical
  "3Dconnexion Universal Receiver" hidraw interfaces**, so `list_available_devices()` (a
  supported-model *catalog*) and name matching both fail — fixed with a **wiggle probe** (open each
  interface ~3 s, pick the one that reports the stick the operator wiggles) (`db05670`); and a
  `--rungs` **CSV filename collision** that silently overwrote 2 of 3 de-confound step CSVs (all
  three points shared pos 130 / vel 0.50) — filenames now carry the full triple (`0182a11`).

## Discussion

### (a) Hypotheses withdrawn mid-arc, and how the adversarial verify caught them

Three explanations died during the arc. All three would have shipped a wrong number if the
adversarial verify stage had been skipped.

- **"The 18:03 `--mode all` suite characterized the tuned winner 110/0.50/0.72."** This was the
  orchestrator's own mid-stream reading, and one Round-2 track (track-forensics rec #5) repeated
  it. It is wrong: the harness's `_bringup_closed_loop` unconditionally re-applies BASELINE before
  every `pos_steps`/`chirp` stage, so those stages can only measure 40/0.2/0.32. The verify stage
  caught it two ways — a direct code read at `bench_leg_sysid.py:1800`, and an independent data
  tell (the 18:03 quiescent tails sit *below even the pos-90 floor*, i.e. a loop softer than 90,
  consistent with 40, inconsistent with 110). Had this stood, the "beautiful" 18:03 step/chirp
  suite would have been mislabelled as the winner's characterization and a genuinely thin winner
  (a single ladder center-step) would have looked thoroughly validated.
- **"110/0.50/0.72 is the bench winner."** The ladder produced it as the highest rung that cleared
  the ζ ≥ 0.7 *step* gate. That gate is systematically optimistic: ramp-limited steps never excite
  the 15–19 Hz resonance, so their fit reads ζ 0.63–0.78 while the chirp peak/DC (which does excite
  it) reads ζ ≈ 0.5. The operator's ear was the first correct instrument; the chirp confirmed the
  mechanism. The lesson generalises: **a damping figure is only as honest as whether the stimulus
  reaches the resonance you care about.**
- **"The chirp shows a stiff loop with a corner near ~15 Hz."** A 2nd-order LS fit *rails* to that
  because the measured magnitude **rises** (0.43 → 1.08) into the ceiling instead of rolling off —
  impossible for a linear low-pass. The rise tracks peak velocity crossing the stiction knee
  (`omega_s = 0.251 rev/s`): it is a **friction describing-function** artifact of a 2.86 mm p-p
  sweep with torque_ff = 0, not a loop corner. (The verify stage also flagged the specific LS
  ζ = 0.485 figure as fragile/ill-conditioned — the robust estimate is the peak/DC 0.49–0.52,
  which reproduces exactly and is what this entry cites.)

### (b) Why the tooling took the shapes it did

- **Explicit `--rungs` survey semantics beat extending the auto-ladder.** The auto-ladder's high
  rungs sit on a zeta-constant diagonal (`vel_gain = 0.45·√(pg/90)`, so pos_gain *and* vel_gain
  step together) and back off on the first unstable point. Attributing the pos-130 buzz needs the
  **off-diagonal** points (130/0.50 vs 110/0.55) and needs the survey to **continue past an
  unstable point** so outcomes can be *compared across points* — the auto-ladder can express
  neither. A survey mode that records-and-continues is a different experiment (comparison), not a
  taller optimisation.
- **vel_int was frozen, then de-confounded.** Freezing `vel_int = 0.72` above pos 90 during the
  *ceiling hunt* was deliberate: ratio-scaling it (`pos_gain/125`) during current saturation
  manufactures integrator windup that reads as a `MAX_DEVIATION` latch and pollutes the pos_gain
  signal. But freezing it also **confounds** pos_gain with vel_int, so the follow-up
  de-confound survey (130 × {0.72, 0.55, 0.40}) was necessary to show 130 is dead *regardless* of
  vel_int — a frozen knob for a clean climb, then swept once to close the attribution.
- **The HF detector is advisory, not an abort.** It computes real-time band-RMS thresholds
  calibrated on one bad point, and it demonstrably false-positives on short (<150 ms) strokes from
  commanded content alone. Wiring it to abort would kill valid aggressive strokes on a metric that
  cannot yet tell commanded content from vibration by itself. The firmware guards (`MAX_DEVIATION`,
  MPC-staleness, current rail) remain the hard safety backstop; the detector's job is to **surface
  the onset to the operator**, whose ear remains the arbiter, and to leave a raw number the
  gain-delta analysis can trust after the fact.

### (c) The instrument-gap lesson

The through-line of the whole day is that **quiescent-hold and arrive-and-settle metrics are blind
to motion-excited vibration by construction** — the vibration only lives under sustained motion,
which those stimuli never sustain. The instruments reported 110/0.50/0.72 clean; the operator
heard and saw it fail. Only after the operator's judgment redirected the effort was the missing
instrument (the motion-excited HF detector) built, and only then did the numbers (3.7× iq HF, ζ≈0.5,
1.26 % rail) catch up to the ear. This is a direct, live instance of the CLAUDE.md rule that
physical intuition disagreeing with the framing is *load-bearing signal* — it saved the project
from committing a gain set that passes every automated gate and rings on the robot. The durable
defence is baked into the harness now: every *moving* stage carries the HF metric, and future
"winner" judgments key off the pos-to-pos gain delta on identical geometry, not the settle fit.

## Fix

Six commits, all bench-scoped; no production config value changed (the committed leg gains remain
the Level-1 `40/0.20/0.32`).

- **`1e03348`** — live-encoder baselines in `_verify_output_engaged` / `_enter_hold_at_center`
  (kills the false engagement abort; +4 regression tests incl. a numeric replay of the operator's
  exact console numbers). No firmware change.
- **`0a271f8`** — Run-A harness upgrades (100 Hz logging, servo-limited step ramps, multi-amplitude
  chirp + Welch/coherence, extended ladder with frozen vint + quiescent buzz gate + baseline-revert,
  `--mode track`, `--knot-hz`) + the **`BENCH_SYSID_BUILD`** bench-only firmware variant (250 Hz
  telemetry, 100 Hz knots, un-gated axis-0 iq; flag-OFF binary sha256-proven byte-identical;
  never-flash-to-robot warnings in README + config + platformio). 5 review findings fixed pre-commit.
- **`30cc786`** — `--rungs` gap-fill survey (never-abort survey mode), `--gains` bringup override,
  `--quiescent-secs` soak; bringup docstring now states the always-reapply-BASELINE behaviour.
- **`8cbf4f0`** — motion-excited HF onset detector, `--mode strokes` battery, `--mode teleop`;
  3 HIGH review findings fixed pre-commit (enqueue-vs-apply gate race → stick freeze; latch-vs-worker
  race → apply-lock + latch epoch; stroke HF hold-dilution → moving windows).
- **`db05670`** — sustained-stroke 2A sizing fix + series-level velocity gate (closes the class) +
  teleop device wiggle-probe.
- **`0182a11`** (same-day follow-up) — vel_int in the ladder CSV filename so `--rungs` points sharing
  (pos, vel_gain) no longer overwrite each other.

## Verification

Full suite `pytest tests/ -q`, all run 2026-07-12, each with the single pre-existing xfail:

| commit | scope | full-suite result |
|---|---|---|
| `1e03348` | live-encoder baselines | **2502 passed, 1 xfailed** in 576.36 s |
| `0a271f8` | Run-A harness + bench firmware | **2531 passed, 1 xfailed** in 590.43 s |
| `30cc786` | `--rungs` / `--gains` / `--quiescent-secs` | **2541 passed, 1 xfailed** in 647.35 s |
| `8cbf4f0` | HF detector / strokes / teleop | **2572 passed, 1 xfailed** in 586.74 s |
| `db05670` | sustained 2A fix + series gate + device probe | **2575 passed, 1 xfailed** in 586.48 s |
| `0182a11` | vint in ladder CSV filename | **2575 passed, 1 xfailed** in 603.85 s |

Firmware, flag OFF: `pytest tests/firmware/ -q` 165 passed; `test_leg_interp` 16 cases / 145
assertions; `pio run -e teensy41` SUCCESS and **hash-identical** to the pre-edit production build.
Flag ON: `g++ -DBENCH_SYSID_BUILD=1` native build passes the same 16/145 at `SEG_T=0.010`;
`pio run -e teensy41_bench_sysid` SUCCESS. All harness stage bodies are pure-function unit-tested
in `tests/motion/test_bench_sysid_bridge.py` (123 passed at `db05670`/`0182a11`); the armed
streaming / guard-recovery / RPC-worker / spacemouse I/O paths are validated statically + by
dry-run only — **operator bench validation of those paths remains the next step.**

## Outcome

> **⚠️ 2026-07-13 — THE STAGE-2 RECOMMENDATION BELOW IS RETRACTED.** The "accuracy knee" that
> motivated it was a **rev→mm units bug** (~14× — see the Diagnosis §7 callout and *Withdrawn
> claims*). The real tracking error at production gains is **0.74 mm RMS** — and **0.054 mm at the
> catch instant**, the metric the spec is judged on — already inside the
> operator's ±1 mm spec, so **no gain change is warranted**: production `40 / 0.20 / 0.32` stands
> and `MAX_LEAD` stays at 0.10. A plant-ID run on this session's own CSVs further shows the
> production cascade was already healthy (`J_eff` ≈ `J_rotor`, `ω_v/ω_p` ≈ 3), and that the
> remaining error is dominated by transport delay and the two switched-off feedforward terms —
> not by anything a feedback gain can reach. **The gain hunt is closed**; the effort moved to
> feedforward. The bench *envelope* (below) remains valid as a stability-envelope result.
> See `logbook/2026-07-13-leg-plant-id-and-the-units-bug.md`.

- **Bench envelope (unloaded leg, Path BRIDGE, v3 lineage):** **pos 40 clean / pos 70 accuracy
  knee / pos 90 aggressive-but-bounded edge / pos 110+ over the line.**
  (2026-07-13: read "accuracy knee" as *"where the mis-scaled error metric flattened"* — the
  envelope's *stability* boundaries stand; its *accuracy* framing does not.)
- **Servo-vs-structure discriminant:** the ~6 Hz limit cycle did **not** reproduce on the isolated
  bench leg in *any* regime — ladder, sustained tracking, or deliberate clamp-riding at 46 %. Given
  the v2→v3 clamp-behaviour change (vel_ff kept, MAX_LEAD 0.10), **the S4 signature may already be
  fixed on the robot** by v3 alone; the unloaded bench cannot certify a loaded, coupled, 6-leg
  system, so **the robot replay decides.**
- ~~**Stage-2 robot recommendation:** start at **pos 70 / vel 0.35 / vel_int 0.56** (the RUN-A pt70
  triple = the accuracy knee), with **`MAX_LEAD` re-pinned to `4.0/pos_gain = 0.057`**…~~
  **RETRACTED 2026-07-13.** Its sole quantitative basis was the mis-scaled accuracy knee. **No gain
  change is warranted:** production `40 / 0.20 / 0.32` meets the ±1 mm spec (0.054 mm at the catch
  instant, ~5× inside), and
  `MAX_LEAD` stays **0.10** (= `4.0/pos_gain` at pos 40 — the already-shipped value, so no firmware
  change either). The **loaded S4 replay survives, reframed**: it is now a *regression test of the
  v3 firmware fix at production gains* (does the 6 Hz reproduce on v3 at all?), **not** a
  gain-selection experiment — there is no candidate to escalate to.
- **Operator authorization:** bench speeds may go to **2.5 m/s (~35 rev/s)** on the acceptable-loss
  leg. The binding link is the firmware **vel_ff pass-through cap of 3.5 rev/s**
  (`LEAD_CLAMP_VELFF_LIMIT_RPS`) — a bench-flag bump of that cap under `BENCH_SYSID_BUILD` is the
  unlock, not the config vel_limit alone.

## Open Questions

- **Does the ~6 Hz survive on the LOADED coupled 6-leg platform under v3?** The unloaded bench
  cannot answer this — it is the dominant transfer risk and the single missing piece between the
  proven bench envelope and a production gain. Resolved by the loaded S4 replay acceptance gate.
- **vel_int's independent role in the pos-130 buzz** is still not isolated below 130 — pos_gain is
  the proven driver conditional on vel_int = 0.72, and the 130 de-confound shows 130 dead at all
  three vel_int, but whether a lower vel_int rescues pos 90/110 is untested.
- **The true audible tone frequency is unresolved:** at 250 Hz telemetry any component above
  125 Hz aliases; the resolvable 26–40 Hz iq excess could be the genuine resonance or an alias of a
  higher tone. Disambiguating needs a kHz iq/encoder capture, an accelerometer, or a microphone.
- **The pos-130 boundary intermittency** (buzzed yesterday, clean tonight; both runs at center
  1.5 rev) is presumed thermal but unconfirmed — a controlled repeat with a temperature log and a
  center-position sweep would separate thermal from position effects.
- **No real S4-class sustained datapoint exists yet** — the sustained entry self-latched at all
  gains until `db05670`; the err/HF surface for a genuine back-to-back sustained stroke awaits a
  re-run of the fixed harness.
- **Remaining Stage-1 bench items:** the by-ear spacemouse teleop session (now that device
  selection is fixed), the real sustained datapoint, the high-speed envelope up to 2.5 m/s (needs
  the vel_ff-cap unlock), and the thermal/positional-intermittency probe.

## Withdrawn claims

- **2026-07-13 — "The accuracy knee is pos 70 (battery mean errRMS 10.3 → 6.9 mm, −33 %)."**
  Retracted as a *decision basis*. The values are **milli-revolutions**, not millimetres:
  `bench_leg_sysid.py:2548` printed `err_rms * 1e3` under an `mm` label, but `err_rms` is in
  **revolutions** (`cmd_rev` − `pos_rev`; the manifest field is correctly named
  `track_err_rms_rev`). The correct scale is the bench leg's `mm_per_rev` = **71.5708**
  (`single_leg_test.py:106`), giving **0.74 → 0.49 → 0.48 mm** for pos 40/70/90 — **~14× smaller**. The −33 % ratio survives (ratios are scale-free); the
  magnitude does not. Against the operator's ±1 mm spec — judged **at the catch instant**, where
  production measures **0.054 mm median / 0.192 mm worst** — a 0.25 mm improvement is irrelevant, and
  production `40/0.20/0.32` already passes by ~5×. **Why the four rounds of adversarial verification
  missed it:** every check was *internal to the number* (right arrays, right windows, right
  exclusions, honest pos-to-pos delta — all true); nobody asked whether **10 mm RMS was physically
  plausible on a leg with a 280 mm stroke**. It was internally consistent and externally absurd.
  Fixed with a named, unit-tested enforcement point (`sysid_lib.format_stroke_error`).
  See `logbook/2026-07-13-leg-plant-id-and-the-units-bug.md`.
- **2026-07-13 — "Stage-2 robot recommendation: pos 70 / vel 0.35 / vel_int 0.56, `MAX_LEAD` 0.057."**
  Retracted. Its sole quantitative basis was the mis-scaled accuracy knee above. No gain change is
  warranted; `MAX_LEAD` stays 0.10 (already shipped). The loaded S4 replay survives as a **v3
  firmware regression test at production gains**, not a gain-selection experiment.
- **2026-07-12 — "The 18:03 `--mode all` suite measured the ladder winner 110/0.50/0.72."**
  Retracted. It ran at BASELINE 40/0.2/0.32; `_bringup_closed_loop` re-applies BASELINE before every
  `pos_steps`/`chirp` stage (`bench_leg_sysid.py:1800`), independently corroborated by the 18:03
  quiescent tails sitting below even the pos-90 floor. The ζ 0.65 → 0.87 "improvement" was the
  250 Hz instrument, not a loop change. (Originated in the orchestrator's mid-stream context and
  Round-2 track-forensics recommendation #5; overturned by the gains-and-loop track + its verify.)
- **2026-07-12 — "110/0.50/0.72 is the bench operating point / winner."** Retracted. Effective
  damping at the 17.5 Hz crossover is ζ ≈ 0.49–0.52 (chirp peak/DC, coherence 1.00), not the step-fit
  0.63–0.78; ζ is non-reproducible (0.78 → 0.66–0.68); sustained motion carries 3.7× baseline 20–45 Hz
  iq energy and rails 1.26 % at 4.4 rev/s. Overturned by operator perception first, then Round-3.
- **2026-07-12 — "The chirp shows a stiff loop with a ~15 Hz corner / resonance peak."** Retracted.
  The rising magnitude into the ceiling is a friction describing-function artifact of a 2.86 mm p-p
  sweep with torque_ff = 0; no linear corner is extractable. The specific LS ζ = 0.485 figure was
  flagged fragile/ill-conditioned by the verify stage — the robust damping figure is the peak/DC
  0.49–0.52.
