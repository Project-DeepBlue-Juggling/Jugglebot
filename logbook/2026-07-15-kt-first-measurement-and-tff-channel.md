---
title: First Kt measurement lands at 0.0577 — neither candidate; the torque_ff channel settled in sign and scale; the static-hold mode's signal was structurally zero
type: investigation
date: 2026-07-15
status: in-progress
phase: "Leg feedforward — Kt measured (confirmation run PENDING), tff channel settled, firmware ingest clamp built (flash PENDING)"
related_plan: leg-gain-tuning-methodology.md
files_changed:
  - tests/hardware/kt_bench_test.py
  - tests/hardware/kt_lib.py
  - tests/motion/test_kt_lib.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - config/hardware_config.yaml
  - config/generate_udp_protocol.py
  - controller/teensy_link/protocol.py
  - controller/teensy_link/setpoint_pump.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/firmware/native/test_leg_interp.cpp
  - tests/teensy_link/test_protocol_codec.py
  - tests/ros/test_teensy_bridge_node_read.py
  - tests/hardware/session_torque_ff.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/README.md
  - docs/teensy-udp-protocol.md
commits:
  - 10de03c
subsystem:
  - motion
  - can
tags:
  - dynamics
  - safety
  - testing
---

# First Kt measurement: 0.0577 — neither candidate. The tff channel settled. The static-hold mode measured nothing, by construction.

## Summary

The operator ran the first real bench sessions of the Kt experiment (2026-07-14 evening): three
`--mode torque_ff_check` runs (leg **inverted** — contraction lifts the load — masses free-hanging,
untouched) and four `--mode kt` runs (0.5/1.0/1.5/2.25 kg). Every run "failed" at the harness level;
**every run's data was recoverable**, and together they settle three questions and expose one
structural design error:

1. **Kt = 0.0577 ± 0.0012 Nm/A (provisional)** — *neither* prior candidate. From the four-mass
   friction-cancelling traverse data, fit against the operator's **actual** masses (the harness had
   mislabelled every run "1.00 kg" — see the bug list): slope −1.937 ± 0.041 A/kg, **R² = 0.99909**,
   intercept +0.010 A (≈ 0 — the internal-coherence check passing). ODrive's configured 0.0551 is
   excluded at **+2.1σ (low)**; the historical 0.0624 at **−3.8σ (high)**. Both wrongnesses are in
   the directions predicted on 2026-07-14: the nameplate `8.27/Kv` was never calibrated, and the old
   position-hold fit was stiction-inflated. **Provisional** because the masses were
   operator-recalled, not weighed, and the (stiction-confounded) hold-current differences sit
   higher, at ~0.063–0.070 — the confirmation run with weighed masses settles it.
2. **The torque_ff channel is settled, in sign and scale.** Positive wire `torque_ff` produces
   **extension** torque on the bench drive — **matching the production chain expectation; no
   negation anywhere** (three runs, ≥10σ). Decay-corrected channel gain **≈ 18.2 A/Nm vs the
   18.14 expected** from `1/torque_constant` (±15–20 %). The previous session's "MUST NEGATE /
   −21 A/Nm" verdict is fully dead: the injection path is **byte-identical** between harness
   versions (0 changed lines), and the −21 was **drift-aliasing** — a monotonic rung ladder at a
   fixed 3.2 s cadence makes time and torque perfectly collinear (`slope_alias = 160 × drift`), so
   the −0.13 A/s needed to fake −21 A/Nm was amply supplied by that run's hand-contamination
   drift (up to −0.29 A/s within single windows).
3. **The static-hold Mode 2's steady-state signal is exactly ZERO, by construction.** At a settled
   hold the velocity integrator forces `pos = cmd`, where torque balance pins total iq at the load
   value — a constant feedforward produces **no steady-state iq shift at all**. What the runs
   measured was the *transient* sampled at ~1.3 s of decay (locked-state re-absorption
   τ ≈ 2–10 s). Mode 2 is rebuilt around **edge-capture** (below).
4. **A friction-band load-transfer transient poisons the first ~7 s of any static hold**: amplitude
   ≈ τ_c (1.2–1.4 A, **load-independent** across a 2× load range), τ ≈ 3.1–3.5 s — the integrator
   absorbing the load tranche static friction carried after the approach. A 15 s pre-soak kills it.

Also landed: the **operator-approved firmware ingest clamp** (torque_ff bounded to ±0.25 wire-Nm at
UDP ingest, per-leg `torque_clamp_mask` on heartbeat flags bits 8–13 — no wire-size change, no
version bump; adversarial review verdict SHIP, zero findings; **dormant until flashed**), and a
rebuild of the Kt harness fixing every failure the sessions exposed.

## Diagnosis

### 1. The Kt fit (recovered from raw CSVs — the harness persisted nothing)

The four kt-mode runs aborted before stage-end serialization (`stages: []` in every manifest), but
the raw 250 Hz traverse CSVs survived. Steady windows re-selected with the orchestrator's recovery gate (|v−0.6| < 10 %,
mid-stroke, smoothed v) give n = 47–159 per traverse vs the starved detector's 23–63
(the harness's LANDED selector, which uses a wider position margin, keeps n = 70–168 on the
same CSVs — two different gates, both quoted in this entry; do not read them as one):

| actual mass | iq up (extend) | iq down (retract) | avg = −τ_g/Kt | half-diff = τ_f/Kt |
|---|---|---|---|---|
| 0.50 kg | +0.162 ± 0.107 A | −2.162 ± 0.079 A | −1.000 | 1.162 |
| 1.00 kg | −0.586 ± 0.095 A | −3.146 ± 0.079 A | −1.866 | 1.280 |
| 1.50 kg | −1.566 ± 0.056 A | −4.228 ± 0.065 A | −2.897 | 1.331 |
| 2.25 kg | −2.849 ± 0.047 A | −5.876 ± 0.051 A | −4.363 | 1.514 |

Fit `iq_avg = slope·m + b`: slope **−1.9365 ± 0.0412 A/kg**, b = +0.010 A, **R² = 0.99909** ⇒
`Kt = g·r/|slope|` = **0.05768 ± 0.00123 Nm/A** (bench spool r = 11.391 mm).

- vs ODrive configured 0.055133: **+2.1σ** — marginally excluded, low.
- vs historical 0.0624: **−3.8σ** — excluded, high (the stiction-bias prediction, confirmed).
- vs lever-arm-corrected 0.0633 and datasheet 0.0637: −4.6σ / −4.9σ.

**Bonus finding: friction grows with load.** The half-difference (`τ_f/Kt`) rises 1.16 → 1.51 A
over 0.5 → 2.25 kg — load-dependent bearing/bushing friction. It does **not** bias the Kt fit
(friction cancels within each mass's up/down pair whatever its magnitude), but it is a real datum:
the Stribeck friction model has no load term. At light load the half-diff (1.16 A) lands at the top
of the known τ_c band (0.88–1.22 A) — the built-in cross-check passing.

**The honest tension:** hold-current *differences* across the tff runs' masses give ~1.6–1.8 A/kg
(→ Kt ≈ 0.063–0.070, central ~0.065), vs the traverse fit's 1.94 A/kg (→ 0.0577). The traverse method is
structurally superior — friction cancels exactly, whereas static holds park anywhere inside a
±τ_c-wide band — but the discrepancy is why the YAML keeps 0.0624 until the **weighed-mass
confirmation run**. (Updating `motor_kt_nm_per_a` will auto-correct the FF wire scale to
`0.055133/0.0577 = 0.956` — the double-correction-proof design doing its job.)

### 2. The tff channel: three "failed" runs that settle everything

All three inverted-rig `torque_ff_check` runs returned NO CONCLUSION — **the gate doing its job on
a confounded instrument**, not bad data:

- **The transient** (H1, CONFIRMED with a mechanism correction): residuals vs the settled line are
  near-identical across runs — slot 0 ≈ +1.0 A, slot 1 ≈ +0.39 A, slot 2 ≤ 0.013 A. Single-exp
  fits: τ = 3.05–3.50 s, amplitude 1.17–1.36 A, explaining 88–94 % of residual variance. The
  amplitude is **load-independent and ≈ τ_c** — this is the friction-band **load transfer**
  completing (the integrator absorbing the tranche static friction carried after the approach),
  *not* wind-up toward the full hold current as first guessed. Simulation: no rung *re-ordering*
  rescues the runs (0/5040 permutations pass the gate); a **+15 s pre-soak** does (bias
  ≤ 0.2 A/Nm).
- **Settled-rung slopes are load-INDEPENDENT**: +8.38/+8.60/+8.46 A/Nm (t = 11–27) once the two
  transient rungs are dropped — the earlier "slope falls with load" reading was an artifact of the
  transient interacting with SEM-weighted fitting. All three runs are **post-hoc recoverable** and
  pass the existing gate on the settled rungs.
- **But +8.5 is not the channel gain** — it is the transient sampled at ~1.3 s of age. Fitting the
  full 250 Hz records with an exponential re-absorption model: **G = 15.8–21.1, weighted
  ≈ 18.2 A/Nm** (τ_locked ≈ 8–10 s), residuals at the noise floor. **The channel gain matches
  `1/torque_constant` = 18.14 within ±15–20 %.**
- **Sign settled**: positive wire tff → positive raw iq = extension on this drive, ≥10σ, three
  runs — the production chain (`extension-positive Jetson → firmware leg_sign negation → drive`)
  is **correct as built**. The manifests' `extension_iq_sign = −1` fields are wrong-way-round **rig
  bookkeeping artifacts**: the harness inferred "holding = extension," false on an inverted rig
  (fixed — the flag is now explicit and required).
- **The 20:55 session is discarded for sign/scale purposes**: its hold current (0.57 A vs 1.43–1.62
  expected free-hanging) confirms the documented hand-support; both members of its "clean in-band
  pair" were load-contaminated; equal successive decrements (−0.451/−0.401 A) are the signature of
  linear time-drift, not a torque response; and a transient model explains only 25 % of its residual
  variance (vs 88–94 % tonight).

### 3. Mode 2's structural flaw, and the edge-capture rebuild

At a settled static hold, the loop's steady-state response to a constant torque_ff is **exactly
zero** — the integrator re-establishes `pos = cmd`, where the total motor torque is pinned by the
load. Any steady-state-mean design measures re-absorption dynamics, not the channel. (The
orchestrator's first redesign — constant tff on constant-velocity traverses — was **refuted in
review** by the same physics: during motion the re-absorption is *faster* (τ = vel_gain/vel_int =
0.625 s), retaining only ~17 % of the signal.)

The rebuilt Mode 2 measures the channel where it is honest — **at the edges, before the integrator
responds**: settle gate (pos within ±0.3 mrev AND |d(iq)/dt| < 0.05 A/s over 3 s) → 15 s pre-soak →
square-wave tff (±0.010/0.020/0.035 Nm, half-period 1.75 s, ≥10 cycles, all inside the 0.045 Nm
static-friction band) → matched-filter edge location (shared median lag per hold — a
polarity-directed per-edge filter locks onto noise and inflates null jumps, caught by the
implementing agent's own tests) → per-edge jump with exponential decay-correction → slope =
jump/(2X), trimmed, with a paired-edge drift statistic. Verdict gate: t ≥ 3 ∧ edge-consistency
CV ≤ 20 % ∧ no drift; **drift is tested before the precise-null test** (drift can fake a null).
Synthetic end-to-end: a known 18.14 A/Nm channel is recovered ±1; a null channel → precise-null;
a drifting trace → NO CONCLUSION.

### 4. The Mode-1 harness failure chain — one root cause, four symptoms

Verified in code and against the manifests: the **starved steady-window gate** (accepting 3–7 % of
genuinely steady records — its accel sub-gate rejected nearly everything on the noisy 250 Hz
velocity estimate) marked every mass FAILED → the park path invoked a bring-up helper that runs
RPCs plus a hard 0.3 s sleep **while armed and streaming nothing** → guaranteed `MPC_STALE` latch →
the park's own re-arm then tripped over the latch → **"CANNOT PARK"**. That chain explains the
missing second reps, the latches, and the park failures in one stroke. Separately: `--hold-mass`
(a Mode-2 flag) was silently ignored by Mode 1, so **every kt run was labelled 1.00 kg** regardless
of the hung mass; and the 2.75 kg approach aborts reported `NONE @ nan` because the over-current
latch's identity was swallowed when no telemetry frame had landed yet.

**Fixes landed** (all unit-tested; 135 kt_lib tests): parse-time mode/flag validation (a Mode-2
flag with `--mode kt` is now a REFUSAL naming the bug it prevents); the steady selector reworked
(smoothed-v gate, position margin, accel sub-gate deleted — n = 70–168 on the real CSVs with
the landed selector; the recovery gate in §1 kept 47–159 — different gates, both real); a
keep-alive flat-knot pump wraps every armed between-stream gap; the park re-arm does the full
stream-then-arm dance (with an armed-healthy fast path that never drops the load); abort reasons
survive an all-NaN telemetry snapshot and include the budget-vs-peak context; manifests persist
per-traverse results **immediately** (an abort can no longer cost the data) and record the
operator-entered actual mass; `--rig-orientation {normal,inverted}` is **required** (no default);
`channel_live` is tri-state (None on no-conclusion — the old sentinel recorded a *dead channel* on
runs that prove it live); and the conditioning floor on light masses is downgraded to a warning —
the 0.5 kg point's empirical performance (on-line at R² 0.999) refuted the hard floor.

### 5. The firmware ingest clamp (operator-approved; review verdict SHIP)

`leg_interp.cpp` ingest now bounds |torque_ff[i]| to **±0.25 wire-Nm** beside the existing NaN
gate — **clamp, don't reject** (an oversized torque with valid pos/vel is a torque-path bug;
dropping the frame would starve the interp into a staleness E-STOP mid-motion). The constant flows
YAML → codegen → the generated Teensy header. Observability: a per-leg **`torque_clamp_mask`** on
heartbeat flags bits 8–13 (free bits — **no wire-size change, no version bump**), decoded by the
Python protocol and mirrored on `/link_status` beside `lead_clamp_mask`. The three-layer chain is
now: pump clamp 0.1325 wire-Nm (binds first) → firmware backstop 0.25 → ODrive 10 A current clamp.
Native g++ tests 17 cases/168 assertions; both pio envs build; 210 firmware/parser/bridge tests
pass; independent verification by the orchestrator (builds re-run, codegen idempotency proven,
mask arithmetic checked: 16128 = bits 8–13). **Dormant until the operator flashes** — noted in the
session doc.

## Discussion

### (a) Drift-aliasing: the instrument lesson of this arc

A monotonic parameter ladder at a fixed cadence makes the swept parameter and time perfectly
collinear — any drift in the measured quantity aliases directly into the fitted slope at
`slope_alias = Δt_cadence/Δparam × drift`. The 20:55 session manufactured a **wrong-signed,
plausible-magnitude channel gain** (−21 A/Nm ≈ −1.17 × the true gain) out of a −0.13 A/s drift,
and it survived a first analysis pass because the *magnitude* looked physical. The settled defence,
now in the harness: measurements that toggle (square-wave edges) rather than sweep monotonically;
a repeated reference rung as a drift audit; and a paired-edge drift statistic in the verdict gate.
This is the third instrument-class lesson of the week (units enforcement, magnitude sanity checks,
now drift-aliasing) and, like the others, it is baked into code rather than prose.

### (b) Two orchestrator hypotheses died in review — by design

The traverse-A/B redesign was refuted by the physics lens (the during-motion integrator absorbs
~83 % of a constant FF — kinetic-friction cancellation was right but irrelevant), and the
"47 % transmission / load-dependent slope" reading was refuted by the time-order lens (the
steady-state is zero; the slope was decay-sampled transient; the load trend was a weighting
artifact). Both would have shipped a wrong instrument or a wrong number. The pattern that keeps
working: pre-register the hypothesis, hand it to an adversarial lens with the raw data, and let
the numbers decide. It has now killed five confidently-held wrong readings in two days.

### (c) What "failed run" means after tonight

Every single hardware run of the past two sessions "failed" at the harness level, and every single
one produced decisive data. The failures were *instrument* failures — starved gates, mislabelled
masses, swallowed diagnostics, a structurally-zero measurement design — while the robot, the drive,
and the physics behaved perfectly. The harness rework's deepest change is therefore not any single
fix but the posture: **persist everything immediately, label everything with ground truth
(operator-entered mass, rig orientation), and never let a verdict outrun its data quality.**

## Verification

- `pytest tests/motion/test_kt_lib.py -q` (2026-07-15): **135 passed**. Scoped `tests/motion/`:
  **658 passed**. Firmware + parser + bridge-read: **210 passed**. Native g++ leg_interp:
  **17 cases / 168 assertions**.
- `pio run -e teensy41` and `-e teensy41_bench_sysid` (2026-07-15): both **SUCCESS** (re-run
  independently by the orchestrator).
- Codegen idempotency: `generate_config.py` + `generate_udp_protocol.py` re-run → **zero diff**.
- Both harness dry-runs exit 0 with the required `--rig-orientation`; all mode/flag refusals
  verified live; the mocked end-to-end recovers Kt 0.0577 and slope 18.09 A/Nm from synthetic
  hardware.
- Full suite `pytest tests/ -q`, run 2026-07-15 as the pre-commit gate: **2772 passed, 1 xfailed** in 604.87 s (the single pre-existing xfail).

## Outcome

- **Kt = 0.0577 ± 0.0012 Nm/A (provisional)** — supersedes both prior candidates as the working
  value, **pending the weighed-mass confirmation run**. YAML keeps 0.0624 until then (documented).
- **The torque_ff channel is validated end-to-end on hardware**: sign correct as built (no
  negation), gain = 1/torque_constant within ±15–20 %. The gravity-FF sign chain is confirmed by
  measurement, not just by energy-balance analysis.
- **The Kt harness is rebuilt** around the failure modes two real sessions exposed; Mode 2 is now
  edge-capture; every run persists its data immediately.
- **The firmware ingest clamp is built and reviewed (SHIP)** — flash pending.
- **Next bench sitting (one session)**: weigh the masses → flash (ingest clamp + BENCH_SYSID_BUILD)
  → `--mode kt --rig-orientation <actual> --masses <weighed>` with 2 reps → edge-capture
  `torque_ff_check`. That seals Kt (auto-correcting the FF wire scale via YAML) and unblocks the
  gravity-FF arming session (`tests/hardware/session_torque_ff.md`).

## Open Questions

- **Weighed masses.** The dominant systematic on Kt = 0.0577 is mass accuracy (2 % mass error ≈
  1.5 % Kt error — the same order as the 0.0551-vs-0.0577 distinction).
- **The hold-current tension** (~0.065, range 0.063–0.070, from stiction-confounded holds vs
  0.0577 from friction-cancelled traverses) — expected to dissolve with weighed masses + 2 reps; if it does
  not, something is unmodelled.
- **Load-dependent friction** (half-diff 1.16 → 1.51 A over 0.5 → 2.25 kg) — real, unmodelled in
  the Stribeck fit; matters if a friction FF is ever revisited at high platform loads.
- **The bench drive's own configured torque_constant was never read back** — the edge-capture gain
  (≈18.2) is consistent with 0.055133, so the earlier "1/21.3 = 0.047" speculation is resolved,
  but the odrivetool read-back in the session doc's S0 remains worth doing for the six platform
  drives.
- **The firmware clamp has never run on hardware** — flash + a bench sighting (mask stays 0 on a
  normal battery; goes nonzero under a deliberately oversized bench frame) is the remaining step.

## Withdrawn claims

- **2026-07-14 — "SIGN: a POSITIVE torque_ff RETRACTS the leg; the gravity FF implementer MUST
  negate" (harness verdict, 20:55 run).** Retracted. Drift-aliasing on a monotonic fixed-cadence
  ladder plus documented hand-contamination; the injection path was byte-identical across harness
  versions; three clean inverted-rig runs settle the sign the other way at ≥10σ. **No negation —
  the production chain is correct as built.**
- **2026-07-14 (orchestrator) — "the in-band slope is ~21.3 A/Nm, ~17 % above expected."**
  Retracted with the same root cause (both members of that pair were load-contaminated).
- **2026-07-15 (orchestrator) — "the settled slope +8.48 A/Nm means ~47 % transmission" and "the
  slope decreases with load."** Both retracted: the static steady-state is exactly zero, the +8.48
  was the transient at ~1.3 s of decay (decay-corrected gain ≈ 18.2), and the load trend was a
  weighting artifact of the transient.
- **2026-07-15 (orchestrator) — the constant-tff traverse A/B redesign.** Refuted in review before
  implementation: during-motion re-absorption (τ = 0.625 s) retains only ~17 % of the signal.
  Replaced by edge-capture.

## Related

- `logbook/2026-07-14-kt-reconciliation-and-gravity-ff.md` — the arc this continues.
- `logbook/2026-07-13-leg-plant-id-and-the-units-bug.md` — the session that opened the feedforward
  chapter (and the units/magnitude instrument lessons this entry's drift-aliasing lesson joins).
- `logbook/2026-04-27-friction-feedforward-bench-validation.md` — the stiction-biased 0.0624 this
  measurement supersedes (pending confirmation).
- `tests/hardware/session_torque_ff.md` — the gravity-FF arming procedure this unblocks.
