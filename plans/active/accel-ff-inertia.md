# Plan: Inertia-aware acceleration feedforward for lateral platform moves

**Status:** PROPOSED — Phase 0 ready to start (created 2026-07-16, from the
accel-FF exploration: machinery map + coordinated-inertia measurement + design)
**Branch:** mvp-trajectory-bringup
**Roles:** operator runs all hardware/flash steps; Claude preps + verifies read-only
**Prereq arcs:** gravity torque_ff SHIPPED ENABLED (2026-07-16, ff81a4d); Kt = 0.0570
SHIPPED; MAX_DEVIATION guard 1.0 rev + ODrive vel_limit 12 rev/s + ceilings opened
(2026-07-16, see logbook/2026-07-16-max-deviation-guard-tracking-lag.md).

---

## Recommendation (headline)

**What to build now (ordered):**
1. **Unblock and enable the platform-inertia torque_ff term that is already coded and config-gated off.** It is the tracking-FF lever the operator actually wants. The analytic accel already flows to the producer (`emitter.py:106-107`); `compute_inertia_wrench` + the reflected-rotor scalar are already computed inside `LegTorqueFeedforward.compute` (`torque_ff.py:206-207, 229-235`), both gated by the single flag `torque_ff_platform_inertia` (`hardware_config.yaml:160`). There is **no new dataflow to write** on the trajectory path.
2. **Firmware flash sitting (bundled, one trip):** the stale-hold torque decay (`leg_interp.cpp:403`) is a **hard prerequisite** — without it an accel-proportional term keeps pushing at full magnitude while `cmd_vel` decays to zero through a stale-link window → open-loop divergence. Fold in the stroke-clamp square-pulse fix (`leg_interp.cpp:447`) and decide the conditional items (VELFF_CAP raise, clamp resizes, ISR interpolation) from the Phase-0 arithmetic BEFORE the trip.
3. **Size at the rigid-body model first (~3.3-4.1e-4/leg = the shipped default), NOT the raw measured 7.7e-4.** The shipped wrench+rotor default already IS the rigid-body sizing — flipping the flag needs zero constant change. The measured number's role is to (a) validate direction/structure (lat>z 1.68×, heavy legs leg-2-in-x / leg-0-in-y — both reproduced by the CAD M_q) and (b) bound an optional scale sweep. Roughly HALF the measured heaviness is friction/viscous/velocity-loop-bandwidth load an accel FF must NOT chase; over-sizing to 7.7e-4 injects torque the loop doesn't need and also blows the pump clamp (0.248 vs 0.15 TRUE Nm at 3000 mm/s²).
4. **Pre-registered A/B on the lateral battery** mirroring the gravity-FF arc, reusing `tools/probes/gravity_ff_ab_extract.py`, with abort signatures for the 2026-05-08 limit-cycle class.

**What NOT to build:**
- **Do NOT build the dynamics-aware feasibility gate (mechanism b) first.** It is the "contract one level up" (today `feasibility.py` checks only kinematic peak_vel/acc/jerk/step, verified at the `# Decide the code` block ~feasibility.py:252) and it is genuinely valuable later — but it needs a *hardware-validated torque predictor*, which is exactly what the accel-FF A/B produces. Build the predictor (a), then reuse it as the gate's oracle. There is also no acute feasibility failure forcing it now: the evening post-fix bag (guard 1.0, vel_limit 6→12) ran the full 22-move battery to 235 mm/s with zero latches.
- **Do NOT add knot-phase lead.** It grows the u0−enc quantity the guard watches (pre-registered counterproductive).
- **Do NOT touch the frozen leg gains** (pos 40 / vel 0.20 / vel_int 0.32). If the null-result branch fires, the residual is velocity-loop bandwidth — a gains/architecture question that belongs to the replanner direction, not this plan.
- **Do NOT trajectory-shape (reduce accel) for smoothness** — that makes moves slower, against the operator's goal.

---

## Context

The operator is about to run lots of fast **lateral** platform moves (ball reloading, single-ball tossing). Hypothesis: inertia-aware work will make lateral moves less jerky. Two completed investigations settle the design:

**Machinery is already built.** The platform-inertia acceleration feedforward is fully wired end-to-end and gated OFF by one config flag — it is not missing code:
- `emitter.py:89` — `pose0,twist0,accel0 = plan.state_at(tau)` gives the analytic 6-DoF platform accel from the quintic segment.
- `emitter.py:106-107` — `accel0` and `leg_acc` are handed straight to `LegTorqueFeedforward.compute`.
- `torque_ff.py:206-207` — `if self.platform_inertia: W_total += compute_inertia_wrench(rot,twist,accel,params)` (full Newton–Euler incl. the 2026-07-14 transport-moment fix, `dynamics.py:221-230`).
- `torque_ff.py:229-235` — reflected-rotor term `τ = J_rotor·q̈` **rides under the same `platform_inertia` flag** (0.000275 kg·m²).
- Gate: `dynamics.torque_ff_platform_inertia: false` (`hardware_config.yaml:160` → `hardware_config.py:37`), AND-gated at `torque_ff.py:135` and `hardware_plant.py:134`.

**Measured coordinated-move inertia (2026-07-16 rosbags, n=243 leg-move fits, SG 90 ms, R²>0.5):** effective per-leg J_eff median **7.7e-4 kg·m² = 2.8× the bench rotor (2.75e-4)**. Direction-dependent: z 5.5e-4, x 9.1e-4, y 9.4e-4, rx 7.1e-4 → **lateral/z = 1.68×** (robust across filter widths and R² thresholds). Load concentrates on specific legs: **leg 2 (0-idx) heaviest in x** (median 12.7e-4, up to 18.9–19.9e-4 in the cleanest fast moves), **leg 0 heaviest in y** (13.3e-4, up to 16.4e-4).

**First-principles cross-check** (M_q = J⁻ᵀ M_x J⁻¹ from the 1.2 kg CAD platform): predicts median **3.3e-4** (1.2× rotor), vx 4.78 / vy 4.60 / vz 3.07e-4, ratio 1.53×. **Structure matches** (same direction ordering, same heavy legs); **magnitude is ~2× below** measured. The excess is stiction/viscous/gravity-drift aliasing into the accel coefficient (R²~0.5) — i.e. **rigid-body ≈ 3–5e-4 is the LOWER bound (true inertia); measured 5–10e-4 is the UPPER bound (inertia + non-inertial load)**. The 2026-07-16 forensics "5–20× rotor" was a velocity-loop-**lag** number, not clean inertia; reconciled here to 2–5× clean.

**Operator hypothesis is confirmed by both data and physics: lateral moves genuinely carry ~1.68× the per-leg effective inertia of z moves, concentrated on 2–3 legs.** That accel-proportional load is what the velocity loop currently lags on; the lag integrates into position deviation that engages the firmware lead clamp (MAX_LEAD 0.10 rev) — and lead-clamp engagement removing position-P authority is the discontinuity that reads as "jerky."

**Clamp headroom** (all verified): gravity term is 0.013–0.041 TRUE Nm/leg (0.23–0.72 A). Layer-1 Jetson pump clamp `torque_ff_max_nm=0.15` TRUE Nm (`hardware_config.yaml:170`) binds FIRST = 0.1451 wire-Nm = 2.63 A, with a construction ceiling of 0.30 TRUE Nm (`setpoint_pump.py:204`). Firmware ingest clamp 0.25 wire-Nm ≈ 4.53 A (`hardware_config.h:46`). ODrive soft-max 10 A ≈ 0.55 wire-Nm (position loop loses all authority above this). Ramp `torque_ff_ramp_s=2.0` (`hardware_config.yaml:214`), shared with gravity. Wire scale 0.96725; sign settled and hardware-verified (+18.35 A/Nm, NO Jetson negation).

**FF sizing vs the target band** (lateral 300–800 mm/s, accel 1000–5000 mm/s²; accel-FF = 2π·J·a):

| leg accel | rigid J (3.3e-4) | measured J (7.7e-4) | rotor-only (2.75e-4) |
|---|---|---|---|
| 1000 mm/s² | 0.029 Nm / 0.5 A | 0.069 Nm / 1.2 A | 0.025 Nm |
| 3000 mm/s² | 0.088 Nm / 1.5 A | 0.207 Nm / 3.6 A | 0.074 Nm |
| 5000 mm/s² | 0.147 Nm / 2.6 A | 0.344 Nm / 6.0 A | 0.123 Nm |

**Combined with the live gravity term at 3000 mm/s²:** rigid 0.088+0.041 = **0.129 TRUE Nm < 0.15 clamp (FITS, no clamp change)**; measured 0.207+0.041 = 0.248 TRUE Nm > 0.15 (would need the pump clamp raised toward ~0.28, still under the 0.30 ceiling). This is the concrete reason to ship rigid-body sizing first.

---

## Findings that shape the design

1. **The trajectory path emits gravity-only today** despite the inertia term being fully coded — flipping `torque_ff_platform_inertia: true` (+ codegen + colcon build) is the entire "enable" step, gated behind the firmware fix below.
2. **Firmware stale-hold is a hard blocker.** `leg_interp.cpp:403` holds `cmd_tor = s_base_torque` (flat ZOH) AFTER the Mode-3 velocity-decay block (375–401). For quasi-static gravity that is correct; for an accel-proportional term it diverges (torque keeps pushing while velocity decays through a stale window). Must decay torque_ff alongside `cmd_vel` in Mode 2/3.
3. **Stroke-clamp square-pulse (same class as the 2026-05-08 limit cycle).** `leg_interp.cpp:447` zeroes `cmd_tor` on stroke-clamp engagement → a leg at a stroke limit sees FF as 0→full→0 square pulses. Fold into the same firmware sitting.
4. **torque_ff is ZOH across the 25 ms knot** (`leg_interp.cpp:403`); only cmd_pos (Hermite) and cmd_vel (Hermite derivative) are interpolated. A fast accel term becomes a 40 Hz staircase. ISR-side interpolation of the reflected-rotor scalar from the Hermite's analytic 2nd derivative is the pre-planned fallback (needs only `MOTOR_ROTOR_INERTIA_KGM2`, already at `hardware_config.h:37`); the platform wrench stays Jetson-side ZOH (too heavy for the 500 Hz ISR).
5. **The shipped default IS the rigid-body sizing.** wrench (CAD, ~1.3e-4/leg platform) + rotor (2.75e-4) ≈ 3.3–4.1e-4, matching M_q — so the first hardware trial needs no constant override.
6. **VELFF_CAP couples to the lateral speed target.** `LEAD_CLAMP_VELFF_LIMIT_RPS = 3.5 rev/s` (`hardware_config.h`, `leg_interp.cpp:50,428-429`). While the lead clamp is engaged, catch-up velocity is bounded to Kp·MAX_LEAD + VELFF_CAP = 40·0.10 + 3.5 = **7.5 rev/s ≈ 529 mm/s**, below the operator's 800 mm/s target and far below the new 12 rev/s vel_limit. Accel FF's job is to prevent clamp engagement; VELFF_CAP is the belt-and-braces if it engages anyway at speed.
7. **The feasibility gate is kinematic-only** (`feasibility.py` decides LIMIT_VEL / LIMIT_ACC / LIMIT_JERK / STEP_BOUND at the `# Decide the code` block; no torque/tracking term). Making it dynamics-aware is mechanism (b), deferred.

---

## Phase 0 — Jetson-side sizing + firmware-item decision (NO hardware, NO firmware)

**Goal:** decide the exact firmware bundle and the sizing before the flash trip, using only offline compute.

- **T0.1** Write a scratch harness (`tools/probes/` if reused, `/tmp` if one-off) that drives `LegTorqueFeedforward.compute` (or `compute_full_feedforward_torques`) over the 22-move lateral battery poses at the first-target dynamics and prints, per leg per move: peak platform-inertia torque, combined gravity+inertia torque, and whether it exceeds the 0.15 / 0.25 / 0.55 clamp layers. Confirm the term reproduces the measured **structure** (lat>z, heavy leg = leg 2 in x / leg 0 in y).
- **T0.2** Sign pre-flight (offline): confirm a commanded +accel on a known lateral move produces the expected-sign per-leg torque delta, on the same extension-positive convention as the verified gravity term. **Gate:** sign matches the gravity chain (`test_gravity_ff_sign_holds_platform_up` convention).
- **T0.3** Decide firmware conditionals from T0.1 arithmetic:
  - Pump-clamp raise? Rigid sizing at 3000 mm/s² fits 0.15 → **NO** for the first target. Record the accel at which combined demand hits 0.15 (rigid: ~3400 mm/s²).
  - Firmware ingest-clamp (0.25 wire-Nm) resize? Binds ~3760 mm/s² (measured J) / ~8770 mm/s² (rigid J) → **NO** for rigid first target.
  - VELFF_CAP raise? Decide against the session speed plan (needed only if lateral moves will run >529 mm/s AND ever engage the clamp).
- **Gate 0:** a one-page sizing memo (in the plan's implementation report) fixing (i) sizing = rigid-body default, no constant change; (ii) the exact firmware bundle for Phase 1; (iii) the pre-registered A/B criteria (below). Checkpoint with operator before the flash trip.

**Files:** new scratch probe; read-only reference to `dynamics.py`, `torque_ff.py`, `hardware_config.yaml`.

---

## Phase 1 — Firmware flash sitting (ONE trip, bundled)

**Goal:** land every firmware change the term needs, plus decided conditionals, in a single flash. See the dedicated firmware bundle. Operator runs the flash.

- **T1.1 (REQUIRED) Stale-hold torque decay.** In `leg_interp.cpp`, make `cmd_tor` decay alongside `cmd_vel` in the Mode-2/Mode-3 extrapolation/decay path (the block at 375–401 that decays `cmd_vel`), instead of the flat hold at line 403. Mirror the velocity `decay` factor so torque and velocity go to zero together through a stale-link window. Update the stale line-refs in `hardware_config.yaml:164` and `torque_ff.py:346-347` (they cite the old `leg_interp.cpp:366`).
- **T1.2 (REQUIRED) Stroke-clamp square-pulse.** Replace the hard `cmd_tor[i] = 0.0f` at `leg_interp.cpp:447` with a treatment that does not inject a 0→full→0 discontinuity when the stroke clamp engages (e.g. hold-then-decay, or scale torque with the clamped position rate). Keep the recovery-slew zeroing at :519 (a genuine recovery ramp).
- **T1.3 (CONDITIONAL, per Gate 0) Firmware ingest-clamp resize** (`TORQUE_FF_FIRMWARE_CLAMP_WIRE_NM`, `hardware_config.h:46`) — only if sizing/pump-clamp raised past ~0.24 TRUE Nm.
- **T1.4 (CONDITIONAL, per Gate 0) VELFF_CAP raise** (`LEAD_CLAMP_VELFF_LIMIT_RPS`, firmware `hardware_config.h`) toward the 12 rev/s vel_limit — **carefully**: this constant was set LOW deliberately by the 2026-07-10 MAX_DEVIATION-runaway forensics (`leg_interp.cpp:9-13,406-415`); any raise must be re-validated against the stutter it was set to break.
- **T1.5 (CONDITIONAL, only if Phase 2 shows HF) ISR-side torque interpolation** of the reflected-rotor scalar from the Hermite d²/dt² (`MOTOR_ROTOR_INERTIA_KGM2` already present). Do NOT build speculatively.
- **Firmware verification (bench, operator-run):** force a stale-link window and confirm torque decays with velocity (T1.1); drive a leg to a stroke limit and confirm no square-pulse torque (T1.2). **Gate 1:** both behaviours confirmed on the bench before any platform run.

---

## Phase 2 — Enable, arm, and pre-registered A/B (hardware, operator-run)

- **T2.1 Flip the flag.** `dynamics.torque_ff_platform_inertia: true` (`hardware_config.yaml:160`) → `python config/generate_config.py` → stage regenerated artifacts → `pytest tests/ -q` → `cd ros_ws && colcon build --packages-select jugglebot`. The 2.0 s `torque_ff_ramp_s` provides the smooth dormant→armed ramp on the same wire as gravity (shared clamp/ramp). This is the "ship dormant-then-arm" pattern.
- **T2.2 Static-hold zero-motion check** (mirror the gravity-FF arm): at a fixed pose, arming must produce zero platform motion (accel=0 → inertia term=0, so the delta is exactly zero by construction; verify).
- **T2.3 Pre-registered A/B** — see the validation protocol. Run the 22-move lateral battery FF-off then FF-on, matched approach history, extract per-leg accel-phase peak deviation + lead-clamp engagement + act_std via `tools/probes/gravity_ff_ab_extract.py` (extended for accel-phase windows). **Gate 2:** ship-enabled only if the pre-registered success criterion is met and no abort signature fired; otherwise revert the flag (disarm = the logged act) or run the single pre-registered scale-up sweep.
- **T2.4 Tripwire test** mirroring `test_shipped_config_has_the_feature_on`: a config test asserting the shipped state (on or dormant, whichever Gate 2 decides) so disarming becomes the deliberate, logged act.

---

## Phase 3 — (DEFERRED to a separate plan) Dynamics-aware feasibility gate

Mechanism (b), the "contract one level up." Today `feasibility.py` gates only kinematic limits. A dynamics-aware gate would reject/reshape trajectories whose predicted combined gravity+inertia torque exceeds clamp headroom or whose predicted tracking deviation would approach MAX_LEAD. **Explicitly out of scope for this plan** — it reuses the Phase-2-validated torque predictor as its oracle and should be built only after the predictor is hardware-trusted. No acute failure forces it (guard 1.0 + vel_limit 12 runs the battery clean).

---

## Verification

- After every `*.py`/`*.yaml` edit: `pytest tests/ -q` (full suite is the pre-commit gate); config edits follow edit→`generate_config.py`→stage→test→commit.
- Firmware: PlatformIO build/compile for the can-bridge Teensy; the T1.1/T1.2 bench behaviours are the functional gate.
- The Phase-2 A/B is the load-bearing hardware verification; its criteria and abort signatures are pre-registered (see validation protocol) before the run, gravity-FF-arc style.
- `/investigate` or `/diagnose` on the A/B bags; logbook entry with a real Discussion section (hypothesis: accel FF cuts heavy-leg deviation; what a null means; the rigid-vs-measured sizing tradeoff).
- **After FF lands: re-run the `19-32-03` retime A/B and decide the `retime_model` re-enable** — the retiming model shipped OFF 2026-07-17 because its honest lean-traverse durations exceed the *tracking* envelope (dev 0.45→0.73 rev); acceptance = model-timed traverse deviation ≤ legacy-timed + day noise. See `logbook/2026-07-17-retime-model-tracking-envelope.md` (Open Questions).

## Out of scope

- Leg gain changes (pos 40 / vel 0.20 / vel_int 0.32 FROZEN per `plans/active/leg-gain-tuning-methodology.md`).
- Knot-phase lead (grows the u0−enc quantity the guard watches — counterproductive).
- The Stribeck friction FF model (`friction_ff_params.py`, motor_guard) — separate chain, deliberately out of scope.
- The MPC path (`hardware_plant.py`) — dormant on this branch; it also passes `skip_reflected_inertia=True` and zero-STEPS on LinAlgError (known-open, doubly gated). Enabling the inertia term there is a separate item.
- Dynamics-aware feasibility gating (Phase 3, separate plan).
- Trajectory shaping / accel reduction for smoothness (makes moves slower, against the goal).

## Notes / carried caveats

- `tests/hardware/traj_ramp_battery.py` is the A/B battery vehicle — referenced read-only, operator-run, NEVER edited by this work.
- Absolute J_eff is filter-sensitive (5.2e-4@70 ms → 15.7e-4@210 ms); trust STRUCTURE (lat/z 1.68×, per-leg ordering), treat magnitudes as ±40%. This is why sizing keys off the rigid-body model, not the raw measured number.
- Reflected rotor rides under the same flag as the wrench (`torque_ff.py:229`); per the 2026-07-13 plant ID J_eff ≈ 1× J_rotor, so rotor is the dominant term — omitting it would be the wrong half.

---

## Firmware flash bundle (one sitting)

**REQUIRED (block enabling the term at all):**

1. **Stale-hold torque decay** — `leg_interp.cpp:403` currently `for(i) cmd_tor[i] = s_base_torque[i];` is a flat ZOH placed AFTER the Mode-3 velocity-decay block (375–401). Make `cmd_tor` decay with the same `decay` factor as `cmd_vel` through the extrapolation/stale window (dt > MAX_EXTRAP 0.05 s). Rationale: an accel-proportional term must not keep pushing at full magnitude while commanded velocity decays to zero → open-loop divergence. Correct for static gravity, wrong for inertia — this is THE gate. Also fix the stale line-refs pointing at the old `leg_interp.cpp:366` in `hardware_config.yaml:164` and `torque_ff.py:346-347`.

2. **Stroke-clamp square-pulse fix** — `leg_interp.cpp:447` `if (cmd_pos[i] != pre) { cmd_vel[i]=0; cmd_tor[i]=0; }` zeroes torque on stroke-clamp engagement, so a leg oscillating at a stroke limit sees FF as 0→full→0 square pulses — the 2026-05-08 limit-cycle discontinuity class. Replace with hold-then-decay or position-rate-scaled torque. (Leave the recovery-slew zeroing at :519 — that is a genuine recovery ramp.)

**CONDITIONAL (decide at Gate 0 from Phase-0 arithmetic; default = SKIP for the rigid-body first target):**

3. **Firmware ingest-clamp resize** — `TORQUE_FF_FIRMWARE_CLAMP_WIRE_NM = 0.25f` (`hardware_config.h:46`, ≈4.53 A). Binds at ~3760 mm/s² (measured J) / ~8770 mm/s² (rigid J). Raise ONLY if the Jetson pump clamp is also raised past ~0.24 TRUE Nm (i.e. only if sizing to measured inertia in the upper band). NOT needed for rigid sizing ≤3000 mm/s².

4. **VELFF_CAP raise** — `LEAD_CLAMP_VELFF_LIMIT_RPS = 3.5f` (firmware `hardware_config.h`; consumed `leg_interp.cpp:50,428-429`). While the lead clamp is engaged, catch-up velocity is capped at Kp·MAX_LEAD + VELFF_CAP = 40·0.10 + 3.5 = **7.5 rev/s ≈ 529 mm/s** — below the operator's 800 mm/s lateral target and far below the new 12 rev/s vel_limit. Raise toward ~8–10 rev/s ONLY if lateral moves will run >529 mm/s AND could engage the clamp. **CAUTION:** this constant was set LOW deliberately by the 2026-07-10 MAX_DEVIATION-runaway forensics (`leg_interp.cpp:9-13,406-415`) — any raise must be re-validated against the stutter it was set to break. Independent of the FF term; bundle it only because it is the same flash and the same higher-speed sessions.

5. **ISR-side torque interpolation** — compute the reflected-rotor scalar in the 500 Hz ISR from the Hermite's analytic 2nd derivative (the ISR already differentiates the Hermite for cmd_vel), replacing the 40 Hz ZOH staircase for the rotor component. Needs only `MOTOR_ROTOR_INERTIA_KGM2 = 0.000275f` — **already present at `hardware_config.h:37`**. The platform-inertia wrench (Jᵀ + tensor) stays Jetson-side ZOH (too heavy for the ISR). **Build ONLY if Phase-2 A/B shows a real HF/act_std rise from the staircase** — do not build speculatively.

**Already flashed (no action):** torque_ff ingest clamp (0.25 wire-Nm, `leg_interp.cpp:56,192-201`), ZOH torque hold, MAX_DEVIATION guard 1.0 rev, overspeed E-STOP 16.5 rev/s.

**Bench verification before any platform run:** (T1.1) force a stale-link window → confirm torque decays with velocity; (T1.2) drive a leg to a stroke limit → confirm no square-pulse torque. These two are the firmware gate.

---

## Pre-registered A/B validation protocol

**Harness:** extend `tools/probes/gravity_ff_ab_extract.py` (pos-delta move matching; identical-approach-history holds only — the ±1.5 A/leg hysteresis dwarfs small FF signatures, so only matched-approach moves are comparable) to window on the **accel phase** of each move and emit per-leg peak deviation, lead-clamp engagement count, and act_std.

**Battery:** the 22-move lateral battery (2z / 3x / 3y / 3rx) that the evening post-fix bag (`2026-07-16_17-38-15`) already ran clean. Vehicle = `tests/hardware/traj_ramp_battery.py` (referenced read-only, operator-run, NEVER edited). Run FF-off then FF-on back-to-back at a FIXED limit step, matched approach history. First step: vel 200 / acc ~660 mm/s² (the level the afternoon latched at, now clean). Second step: open toward acc ~3000 mm/s² as the operator ramps session limits, where the effect is largest.

**Battery/power:** run on a consistent supply state / warm-soak, same discipline as the gravity-FF arc — the friction/hysteresis noise floor is battery- and warmth-sensitive. Do not compare a cold FF-off run to a warm FF-on run.

**Primary metric:** per-leg PEAK position deviation (u0−enc) on the accel phase of each fast lateral move, and lead-clamp engagement count (crossings of MAX_LEAD 0.10 rev), focused on the **heavy legs** (leg 2 in x, leg 0 in y).
**Secondary:** act_std of iq / motor current on the accel phase (HF/smoothness proxy — the 2026-05-08 limit-cycle metric); realized-vs-commanded velocity deficit.

**Success criterion (pre-registered):** FF-on reduces heavy-leg accel-phase peak deviation by **≥30%** at matched dynamics on the fast lateral moves, **without act_std increasing on any leg**. (The 30% target corresponds to the ~0.03 rev predicted removal at 3000 mm/s² rigid sizing being a meaningful fraction of the ~0.08 rev baseline that approaches the 0.10 rev clamp — see risks/magnitude.)

**Abort signatures (any → immediate disarm: flag off → generate_config → colcon → relaunch):**
- (i) act_std rises >2× on any leg (limit-cycle onset — the 2026-05-08 signature: act_std 21→862 µm).
- (ii) any 5 Hz spectral peak in iq / motor current (limit cycle).
- (iii) any NEW MAX_DEVIATION guard latch that the FF-off pass did not produce.
- (iv) FF-on peak deviation WORSE than FF-off on any leg at matched dynamics (sign error).

**Which battery to decide ship-enabled vs dormant:** the first step (vel 200 / acc 660) is the safety screen; the higher-accel step is the efficacy read. Ship enabled only if the higher-accel step meets the criterion cleanly.

**Pre-registered null-result meaning (mirror the gravity-FF dynamic-A/B null discipline):** a null at rigid-body sizing means either the accel-proportional load is not the dominant deviation source at these dynamics (consistent with "half the measured heaviness is non-inertial"), OR the rigid sizing is too low. **Decision criterion, pre-registered so the pivot is planned not sunk-cost:** a null at rigid sizing triggers exactly ONE scale-up sweep toward the measured upper bound (~2×, staying under the pump clamp / raising it toward the 0.30 ceiling if needed). A null there too → **ship DORMANT** (the term is not the smoothness lever at these dynamics; do not add wire torque for no benefit), log it, and escalate the residual jerkiness to the velocity-loop-bandwidth / replanner track (frozen gains, out of this plan's scope). A null is a legitimate, publishable outcome — not a failure to be rescued.

---

## Risks (ranked)

Ranked, with mitigations:

**1. Limit-cycle hazard (2026-05-08 canonical, HIGHEST).** Any hard/discontinuous gate on a fast-changing FF term → 5 Hz limit cycle (act_std 21→862 µm). The accel term is fast-varying within a segment, so it is more exposed than the quasi-static gravity term. *Mitigation:* the term ships behind the smooth 2.0 s `torque_ff_ramp_s`; `compute_inertia_wrench` is branch-free / C∞ in pose·twist·accel; the ONLY discontinuities are the firmware stroke-clamp zeroing (`leg_interp.cpp:447`, fixed in the flash bundle) and the ZOH staircase (monitored; ISR fallback ready). No hard velocity-band gate anywhere. Abort signature (i)/(ii) in the A/B catches onset.

**2. Stale-hold divergence (HIGH — the reason the firmware fix is a hard prereq).** Without T1.1, a link stall mid-accel leaves `cmd_tor` pushing at full magnitude while `cmd_vel` decays to zero (`leg_interp.cpp:403` flat ZOH after the decay block) → open-loop divergence → guard trip or worse. *Mitigation:* T1.1 is a blocking gate; the term is NOT enabled (Phase 2) until the bench confirms torque decays with velocity through a forced stale window.

**3. Clamp-saturation square-pulses (MEDIUM).** If combined gravity+accel demand oscillates across the pump-clamp boundary mid-accel-phase, the min/max truncation could chatter. *Mitigation:* the truncation itself is smooth (min/max, not a zero-step), and rigid-body sizing keeps combined demand at 3000 mm/s² to 0.129 TRUE Nm < the 0.15 clamp (FITS with margin). Measured sizing (0.248 > 0.15) would saturate — hence rigid-body first. If the clamp is raised for the upper band, size so p90 combined demand stays below it, not straddling it.

**4. ZOH staircase at high jerk (MEDIUM).** The 40 Hz torque staircase injects HF the loop must absorb; worse for the fast accel term than for gravity. *Mitigation:* monitor act_std / iq spectrum in the A/B (secondary metric); the ISR-side interpolation of the reflected-rotor scalar (needs only the already-present `MOTOR_ROTOR_INERTIA_KGM2`) is the pre-planned, empirically-gated fallback — build only if HF actually rises. Bounded risk.

**5. Sign error (LOW likelihood, HIGH consequence).** A sign flip makes the FF add the load during accel (drives the leg the wrong way → doubles deviation → guard trip). *Mitigation:* sign is settled and hardware-verified for the gravity term on the SAME wire and convention (+18.35 A/Nm, no Jetson negation, `test_gravity_ff_sign_holds_platform_up`); the inertia wrench uses the same `solve(Jᵀ,W)→leg_forces_to_motor_torques` extension-positive path. T0.2 offline sign pre-flight + abort signature (iv) in the A/B are the two nets.

**6. Over-sizing to the measured 7.7e-4 (LOW, but the methodology trap).** ~half the measured heaviness is friction/viscous/velocity-loop-bandwidth load an accel FF must not chase; sizing to it injects torque the loop doesn't need (overshoot, HF) and blows the clamp. *Mitigation:* rigid-body default first; measured is the pre-registered scale-sweep upper bound only, tuned by the A/B tracking result, never assumed.

**7. VELFF_CAP raise re-introducing the 2026-07-10 stutter (LOW, only if T1.4 taken).** The cap was set low deliberately to break the MAX_DEVIATION runaway. *Mitigation:* T1.4 is conditional, and any raise is re-validated against the stutter signature before relying on it.

---

## The operator's hypothesis, answered

**Yes — inertia-aware work should make fast lateral moves less jerky, and the mechanism is (a) tracking feedforward, specifically the platform-inertia torque_ff term — NOT trajectory shaping and NOT a feasibility gate.**

**Why the hypothesis is right:** the measurement confirms lateral moves carry ~1.68× the per-leg effective inertia of z moves (z 5.5e-4 vs lateral 9.2e-4 kg·m²), concentrated on 2–3 specific legs (leg 2 in x up to ~19e-4, leg 0 in y up to ~16e-4), and the CAD-derived M_q predicts the same ordering (1.53×) and the same heavy legs — so it is physics, not a fitting artifact. That accel-proportional load is exactly what the velocity loop currently lags on: with vel_gain 0.20 Nm/(rev/s) and an integrator time constant (~vel_gain/vel_int ≈ 0.6 s) that is long relative to a ~0.15 s accel phase, the proportional term carries the load with a velocity deficit v_err ≈ τ/vel_gain, which integrates into position deviation. That deviation approaching the 0.10 rev lead clamp — and the clamp removing position-P authority — is the discontinuity that reads as "jerky." Feeding the accel torque forward supplies it directly so the loop doesn't have to lag for it.

**Which mechanism — tracking FF, not the others:**
- **Tracking FF (build this):** removes ~0.03 rev (rigid sizing) to ~0.08 rev (measured sizing) of peak deviation on the heavy legs at 3000 mm/s², keeping them out of the lead clamp. This is the operator's lever.
- **Trajectory shaping (no):** would reduce accel → slower moves, against the goal.
- **Dynamic feasibility gate (later):** the "contract one level up" (today `feasibility.py` checks only kinematic limits) — valuable, but it needs the tracking-FF torque model as its hardware-validated oracle, and no acute feasibility failure forces it now (the battery runs latch-free). Build it after, reusing the FF predictor.
- **Knot-phase lead (no):** grows the u0−enc quantity the guard watches.

**Honest caveat — expect a PARTIAL, accel-scaled improvement, not a cure.** Roughly half the measured "heaviness" (7.7e-4 measured vs 3.3e-4 rigid-body) is friction/viscous/velocity-loop-bandwidth load an accel FF cannot and should not cancel. The improvement is largest at high accel and subtle at the currently-tested ~235 mm/s / ~660 mm/s² (~5× smaller, ~0.006–0.016 rev). If, after the A/B, residual jerkiness persists at low accel, the limiting factor is velocity-loop bandwidth (vel_gain 0.20, frozen) — a gains/architecture question that belongs to the replanner direction, not this FF work. That is the pre-registered null branch.

---

## Related

- `logbook/2026-07-16-max-deviation-guard-tracking-lag.md` — the forensics + guard/vel_limit arc that motivated this plan.
- `logbook/2026-07-16-gravity-ff-armed.md` — the gravity-FF arc whose arming/A-B pattern this plan mirrors.
- `plans/active/leg-gain-tuning-methodology.md` — FF-before-feedback; gains frozen.
- Inertia measurement bags: `~/Desktop/rosbags/2026-07-16_{10-12-33,10-17-45,13-17-58,13-34-58,17-38-15}`.
  The evening bag (17-38-15) doubles as the guard-1.0 + vel_limit-6.0 hardware validation:
  limits ramped vel 100→200→280 mm/s (acc 400→660, jerk 8000), 22 move segments,
  ZERO latches, peak realized leg vel 3.34 rev/s (~235 mm/s).
- Measurement method + numbers: n=243 leg-move fits, SG-90 ms derivative, R²>0.5;
  the one-off analysis scripts lived in the session scratchpad (volatile /tmp) — the
  durable numbers are in this plan's Context; the re-derivation recipe is one line
  (fit Kt·iq = c0 + cv·v + ca·a per leg per move window; J_eff = ca/2π).
