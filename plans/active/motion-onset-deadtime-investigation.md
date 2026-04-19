# Motion-Onset Dead-Time — Investigation & Fix Plan

**Status:** Proposal, awaiting diagnosis/fix-plan approval
**Related logbook:** [2026-04-18-hold-fighting-motion-onset-jitter.md](../../logbook/2026-04-18-hold-fighting-motion-onset-jitter.md) (Failure A — unresolved)
**Primary CSV:** `mpc_20260418_164119.csv` (vel-ff ON) + `mpc_20260418_181020.csv` (vel-ff OFF A/B partner)
**Author note:** This document is a research deliverable, NOT a fix. Do not touch control-loop code until diagnosis and fix plan are approved.

---

## 0. Symptom Recap

From the hold-fighting-motion-onset-jitter logbook entry (Move 1, vel-ff ON):

| Observation | Value |
|---|---|
| cmd_ext ramp-onset | t = 0.39 s |
| First measurable motor motion | t = 0.51 s |
| **Onset dead-time** | **~120 ms** |
| Leap at t = 0.54 s (all 6 legs simultaneously) | 2 mm in one 25 ms tick (50–100 mm/s) |
| Peak cmd-vs-actual lag | 12 mm at t = 0.71 s |
| actual_ext jumps ≥ 2 mm clustered 0.54–1.25 s | **39** (all 6 legs) |
| Leg-velocity swings within single ticks | 0 → 130 → −30 mm/s |

A/B result (vel-ff OFF): the 39-jump pattern is reproduced identically, shifted ~0.6 s later. **MPC and velocity feedforward are ruled out** as the source — the cause lies below the `HardwarePlant → motor_guard → ODrive` interface.

---

## 1. Verified Pipeline Facts (before diagnosis)

Before enumerating candidates, these facts were verified by re-reading the current code (not inferred from stale config):

1. **Legs run in `POSITION + PASSTHROUGH` at runtime.**
   - [can_node.py:634](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L634) (homing path, `_setup_odrives_steps`) and [can_node.py:1476-1477](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L1476-L1477) (re-assertion in `_gentle_move_steps`) both call `encode_set_controller_mode(axis_id, 'POSITION', 'PASSTHROUGH')`.
   - **Therefore:** the flashed ODrive config value `"input_mode": 5` (TRAP_TRAJ) in [config/ODrive config Files/odrive_pro_leg_config.json:182](../../config/ODrive%20config%20Files/odrive_pro_leg_config.json#L182) is overridden at boot and does **not** apply to running motion.
   - **Therefore:** `"input_filter_bandwidth": 20.0` at [line 181](../../config/ODrive%20config%20Files/odrive_pro_leg_config.json#L181) is inert — it only takes effect under `POS_FILTER` mode (per ODrive firmware semantics).

2. **Leg PID gains (current, post-iteration-3):**
   | Leg | pos_gain | vel_gain | vel_int_gain |
   |---|---|---|---|
   | 0, 2, 3, 5 | 40 | 0.20 | 0.32 |
   | 1, 4 | 30 | 0.20 | 0.24 |
   Source: [hardware_config.yaml:198-200](../../config/hardware_config.yaml#L198-L200). Ratio `pos_gain : vel_int_gain ≈ 125:1`.

3. **motor_guard Hermite interpolation is C¹ continuous at a hold→ramp transition.**
   - [motor_guard.py:703-742](../../ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py#L703-L742).
   - At `cmd_next_mm == cmd_cur` (hold), Hermite degenerates to `pos(s) = p0`. Velocity at segment end (`v1`) equals the MPC-provided `cmd_vel[1]`, which is continuous with the next segment's `v0`.
   - **No C¹ discontinuity** is introduced at the ramp-onset boundary.

4. **Encoder-LSB dead-band (commit abc4a8e) is already deployed** in `HardwarePlant.command()` and suppresses sub-LSB command jitter during hold (~8.56 μm per leg).
   - This fix targets hold-phase cosmetics only — it does not affect motion-onset.

5. **CLOSED_LOOP entry seeds `pos_setpoint` at current encoder position** and does not perform any mechanical seating ([can_node.py:628-631](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L628-L631), [can_node.py:1490-1492](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L1490-L1492)). The integrator is zeroed at entry.

These rule out: TRAP_TRAJ planning lag, `input_filter_bandwidth` lag, Hermite discontinuity, encoder-LSB quantization, and any possibility that the dead-band fix interferes with motion-onset.

---

## 2. Diagnosis — Candidate Mechanisms

For each candidate, I give:
- **Mechanism** — the physical or algorithmic cause
- **Predicted signature** — what it would produce in the CSV
- **Fit to observed data** — consistent, partial, or ruled out
- **Distinguishing evidence** — the test that separates it from its neighbours

### 2.1 Static friction (stiction) breakaway

**Mechanism.** Ballscrew + belt + motor bearings have static friction greater than kinetic friction. During hold, the pos-error integrator is near zero (ODrive holding encoder-stable). As `cmd_ext` begins to ramp, `pos_err = cmd − actual` grows linearly; motor current rises as `pos_gain · pos_err` (plus vel-int wind-up). Nothing moves until motor torque exceeds the breakaway threshold, at which point the mechanism snaps free and the pre-built-up error drives a fast catch-up.

**Predicted signature.**
- All legs share the same dead-time window (they share the same cmd cadence and similar μ_s).
- Breakaway magnitude proportional to how fast `pos_err` builds up (i.e. cmd-vel during onset).
- First-ever motion after a long rest is worst; repeated motion within seconds is milder.
- Motor current during the dead-time is a rising ramp that suddenly drops when the mechanism breaks free.

**Fit to observed data.** **Strong fit.** All 6 legs leap together in a 25 ms tick with 50–100 mm/s instant velocity — classic stick-slip release. The 39 jumps clustered 0.54–1.25 s are consistent with the mechanism cycling between stuck and moving while the cmd velocity is still low.

**Distinguishing evidence.** Two ways to confirm:
1. Run an "N-repeat" move (idle → move → idle → move, with identical trajectories, back-to-back). Dead-time should shrink on repeat 2+ as the mechanism stays "warm."
2. Compare motor `iq_measured` during the dead-time window to its value just after breakaway. A ramp-up-then-dip pattern is stiction; a monotonic rise is not.

**Already-available data.** The Apr 18 entry did not publish `iq_measured` traces. The CSV columns include `cmd_vel`/`cmd_ext`/`actual_ext` per leg but I did not confirm `iq_measured` is logged at 500 Hz by motor_guard. If it is in the rosbag (`leg_lengths_topic`), a plot through t = 0.39–0.55 s would be decisive.

### 2.2 Mechanical backlash (coupler / cable / belt lost-motion)

**Mechanism.** Each leg has a coupler between motor and ballscrew and a cable pulley. Under load reversal (or starting from rest against a load), the mechanism takes up lost motion before the ballscrew output moves. The leg-gain-tuning methodology explicitly notes this ([plan, Gotchas line 178](leg-gain-tuning-methodology.md)): *"Backlash eats the first ~0.2 mm of any step."*

**Predicted signature.**
- Dead-time depends on the **direction** of the last motion versus the new motion.
- A full back-and-forth of greater than the backlash magnitude should eliminate the dead-time on the *second* pass.
- Dead-time is position-independent (same mid-stroke as near endpoints).
- Once motion starts, the mechanism tracks normally — no post-breakaway catch-up overshoot.

**Fit to observed data.** **Partial fit.** Backlash alone would produce a smooth (slow) take-up, not a synchronised 2 mm leap at 50–100 mm/s. The Apr 18 entry explicitly ranks it as *"fits the onset stick-slip release burst, but does not explain ongoing hold fighting"* (entry lines 116–117). The fast leap signature is more consistent with stiction; backlash most likely contributes additively (stiction + backlash in series).

**Distinguishing evidence.**
- Manually back-drive each leg with the hand and measure coupler/cable slack (mm) before entering CLOSED_LOOP. If slack > ~0.1 mm, backlash is a substantive contributor.
- Run a "direction-reversal" A/B: two consecutive moves in the same direction vs. two in opposite directions. If same-direction move-2 has no dead-time but reversal-move has full dead-time, backlash is dominant.

**Already-available data.** Mechanical slack measurement is not in any log — this is a hardware measurement, not a software detection. The hold-fighting entry's Iteration-3 quiescent session (`mpc_20260418_222358.csv` onwards) may have "warm" repeat-move data; would need to re-run diagnose.py with a new motion-onset detector to confirm.

### 2.3 PID position-error build-up lag (pure control-side)

**Mechanism.** Even with zero friction and zero backlash, a PID cascaded loop responds to a cmd-ramp with an inherent lag proportional to `1 / crossover_frequency`. With `pos_gain = 30–40` (turns/rev-err → rev/s) and `vel_gain = 0.2` (Nm/(rev/s) — rough order), the closed-loop bandwidth is approximately `vel_gain · pos_gain / (2π · rotor_inertia_effective)`. With the platform mass projected into leg-rotor-inertia, this puts bandwidth at ~10–30 Hz, i.e. a time-constant of 5–15 ms.

**Predicted signature.**
- Dead-time scales with `1 / bandwidth`, order ~15–30 ms, not 120 ms.
- Actual position tracks cmd with a *continuous* lag, not a dead-zone followed by a leap.
- Dead-time is the same for the 1st move of a session and the 10th.

**Fit to observed data.** **Poor fit.** The predicted lag is an order of magnitude shorter than 120 ms, and the "flat then leap" signature is not what a pure linear-lag plant produces.

**Distinguishing evidence.** Control-side lag cannot produce a dead-time followed by a leap without a nonlinearity. If we instrument `pos_err` directly and see it rising linearly from cmd-onset, then a step drop at breakaway, the nonlinearity is a friction-type dead-zone, not linear lag.

### 2.4 motor_guard interpolator internal quantization

**Mechanism.** `_interpolate_and_send` rounds, clamps, or snaps to quantized values in a way that holds `pos_setpoint` at the hold-value until some threshold is crossed.

**Predicted signature.**
- Dead-time shows up as a constant `pos_setpoint` in the motor_guard output trace while `cmd_ext` is climbing.
- Would be identical across all legs (shared code path).

**Fit to observed data.** **Likely ruled out** by the A/B test. With vel-ff disabled, the motor_guard would still quantize the same way; the 39-jump pattern is identical, just shifted in time — which is consistent with both the stiction hypothesis (triggered later because cmd crosses breakaway later) and with a quantization bug. *But:* the Hermite formula has no explicit quantization at this boundary; the input values are float32 throughout until the CAN frame where the position is sent as float32 IEEE 754.

**Distinguishing evidence.** Log `commanded_pos_rev` (the exact value motor_guard sends over CAN) at 500 Hz through a motion-onset. If it ramps smoothly starting at cmd-onset, quantization is not the cause. If it steps in 1-LSB-per-frame increments or holds flat, quantization is implicated.

**Already-available data.** The CSV exposes `cmd_ext` (MPC → plant) but may not expose the 500 Hz `pos_setpoint` sent by motor_guard. Adding that to the telemetry stream would decide this candidate cleanly.

### 2.5 PID vel-integrator wind-up lag

**Mechanism.** A variant of 2.3 that specifically targets `vel_int_gain`. At entry, `vel_integrator` = 0. The integrator accumulates `vel_err = vel_cmd − vel_actual`; for a stuck motor, this grows as `vel_int_gain · vel_cmd · t`. If breakaway torque requires a large integrator contribution, the dead-time is `τ_breakaway / (vel_int_gain · vel_cmd_at_breakaway)`.

**Predicted signature.**
- Strongly dependent on the `vel_int_gain` value.
- A/B with higher `vel_int_gain` should shrink the dead-time.
- But with current `vel_int_gain = 0.24–0.32`, the integrator winds up ~quickly — if the torque needed is, say, 0.5 Nm and `vel_cmd = 1 rev/s`, integrator needs ~60–80 ms to produce 0.5 Nm. Not negligible vs. 120 ms.

**Fit to observed data.** **Partial fit.** Plausibly contributes; but not a pure explanation because the observed leap happens in a single 25 ms tick, not as a ramp-out of the integrator.

**Distinguishing evidence.** Doubling `vel_int_gain` (transiently, for a test move) should reduce dead-time proportionally *if* wind-up is the bottleneck. If dead-time is unchanged, the bottleneck is mechanical (stiction/backlash), not integrator speed.

### 2.6 Cmd quantization at `cmd_next_mm == cmd_cur` boundary (Hermite input)

**Mechanism.** Hermite's C¹ continuity depends on the MPC supplying `cmd_next_mm`, `cmd_next2_mm`, and accurate `cmd_vel`. If the MPC emits `cmd_next_mm = cmd_cur` during hold (and only transitions `cmd_next` in the tick when the ramp starts), the Hermite segment for that tick sees `(p0=p1=hold, v0=0, v1=new_v)`, which produces a cubic that rises slowly from `s=0` rather than ramping linearly from t=0.

**Predicted signature.**
- Lag of up to one full MPC period (25 ms), not 120 ms.
- Would appear as a one-tick "soft start" rather than a dead-zone.

**Fit to observed data.** **Too small** to account for 120 ms on its own; can contribute at most ~25 ms. Not the dominant mechanism.

**Distinguishing evidence.** If cmd_ext in the CSV shows a clean 25 ms-tick linear ramp from t = 0.39 s, Hermite is faithful; any sub-tick curvature at the boundary would be visible on the 500 Hz pos_setpoint trace.

### 2.7 ODrive `input_pos` scheduling latency / CAN frame batching

**Mechanism.** Six CAN frames at ~1 ms each on a 1 Mbps bus gives ~6 ms per MPC tick; under CAN-queue contention or a dropped-frame recovery, this could balloon. But even with 10× contention, that's 60 ms, not 120 ms.

**Predicted signature.** Per-leg lag, not synchronised. Bursts and recoveries visible in `can_node` rx/tx diagnostics.

**Fit to observed data.** **Poor fit.** All 6 legs leap in the *same* 25 ms tick, which argues against per-leg CAN scheduling jitter.

**Distinguishing evidence.** Check CAN drop counters / `attempt_restore()` triggers in the session log. If there were drops at t = 0.39 s, we'd see them; if not, CAN is exonerated.

### 2.8 Ranking of candidates

| Candidate | Fit | Ranking |
|---|---|---|
| 2.1 Stiction breakaway | **Strong** | **#1 (dominant)** |
| 2.2 Mechanical backlash | Partial (additive to 2.1) | #2 (contributing) |
| 2.5 Vel-integrator wind-up | Partial | #3 (contributing, control-side) |
| 2.6 Hermite hold→ramp boundary | Small (≤25 ms) | #4 (minor) |
| 2.3 Pure PID bandwidth lag | Poor | Ruled out as dominant |
| 2.4 Motor_guard quantization | Unlikely | Ruled out pending pos_setpoint log |
| 2.7 CAN scheduling | Poor | Ruled out by synchronicity |

**Working hypothesis.** Primary driver is stiction breakaway (2.1), with backlash (2.2) additive, and integrator wind-up (2.5) as a secondary control-side contribution. The "flat then leap" signature is a stick-slip release, and the 6-leg synchronicity reflects shared cmd-cadence + similar μ_s.

---

## 3. Fix Options — Ranked by Risk × Benefit

Proposals are ranked lowest-risk-first. Several can be combined.

### 3.1 Instrumentation-only: add motion-onset detector to diagnose.py

**Change.** Add `analyse_motion_onset()` to [sim/analysis/diagnose.py](../../sim/analysis/diagnose.py) that, for each cmd-ramp-onset (detected when `|d(cmd_ext)/dt|` crosses a threshold after a hold), measures `onset_latency_ms` = first sample where `|actual_ext − hold_pos| > 0.5 mm`, per leg.

**Expose.** `motion_onset_latency_ms` (per-leg + median across legs), `ramp_to_leap_magnitude_mm` (the catch-up magnitude), `leap_synchronisation_window_ms` (time spread across 6 legs).

**Risk.** ~0 — purely a detector, no control-loop touch. Landing this first unlocks A/B for every fix below.

**Benefit.** Without this metric, no proposed fix can be quantitatively validated. Currently diagnose.py has `actual_jumps` (2 mm threshold) which detects the leap but not the preceding silence (see [sim/analysis/diagnose.py:372-402](../../sim/analysis/diagnose.py#L372-L402)).

**Recommendation.** Ship this first, before any fix below. Re-run the Apr 18 CSV through it to establish the current `onset_latency_ms` baseline (expected ~120 ms from the entry).

### 3.2 Mechanical: coupler/cable backlash takeup + pre-tension procedure

**Change.** Before tuning any control gains, perform a hardware backlash audit:
- Remove platform load; for each leg, manually back-drive and measure lost-motion at the platform side (mm) vs. encoder counts.
- If lost-motion > 0.05 mm on any leg, identify the source (coupler set-screws, cable slack, pulley backlash) and tighten / replace / pre-tension.
- Document measurements in `logbook/`.

**Risk.** Low — physical inspection, reversible.

**Benefit.** Sets a known mechanical baseline. Until backlash is characterized, every software fix is fighting an unmeasured nonlinearity. If backlash is ≥ 0.2 mm, no software fix can cleanly close the dead-time (the mechanism must traverse backlash before it can track).

**Recommendation.** Do this **before** trying control-side fixes. It's also the only way to get clean data for Level-2 step-response tuning (gain-tuning methodology explicitly calls this out — Gotchas line 178).

### 3.3 Software: pre-move backlash-seating micro-pulse

**Change.** In `_gentle_move_steps` or `HardwarePlant`, when a cmd transitions from hold (no commanded motion for ≥N ticks) to a ramp, emit a one-tick counter-pulse of ±50 μm followed immediately by the intended motion. The counter-pulse pre-seats the mechanism against the direction of travel, taking up backlash before the "real" motion begins.

**Risk.** Medium — introduces a brief (25 ms) intentional motion before every move. Visible as a small jolt. Must be carefully bounded.

**Benefit.** Eliminates the mechanical contribution (2.2) without touching PID tuning. Isolates stiction (2.1) as the remaining term. Compatible with any future control-architecture change.

**Recommendation.** Only attempt if mechanical audit (3.2) confirms backlash > 0.05 mm and mechanical fixes aren't practical (e.g. couplers are best available). Implement as a feature flag so it can be A/B tested.

### 3.4 Control: vel-integrator pre-load at cmd-onset

**Change.** When the MPC transitions from hold to move (`|cmd_vel|` rises above threshold), pre-load each ODrive's `vel_integrator` with a value equal to the expected steady-state current needed for the commanded velocity. Send via ODrive's `vel_integrator_set` command (if supported over CAN) or by biasing the first `vel_ff` value to include the breakaway term.

**Risk.** Medium-high — wrong pre-load value causes initial overshoot; too-aggressive pre-load can cause hold-fighting to reappear. Interacts with Iteration-3 gain tuning which was already conservative.

**Benefit.** Directly shortens the wind-up component of dead-time (2.5). Does *not* address stiction (2.1) or backlash (2.2).

**Recommendation.** Defer until after 3.1 (instrumentation) and 3.2 (mechanical audit) are complete. If the residual dead-time after mechanical fixes is still >40 ms, revisit.

### 3.5 Control: breakaway-torque feedforward (Stribeck-style)

**Change.** Add a friction-compensation term to the torque feedforward path that, when `|cmd_vel| > 0.5 mm/s` and `|actual_vel| < 0.1 mm/s` (i.e. commanded but not yet moving), adds a constant torque equal to measured stiction breakaway for each leg. Once the leg is moving (`|actual_vel| > 0.5 mm/s`), the term decays to the kinetic-friction value.

**Risk.** High — requires per-leg measurement of `τ_stiction` and `τ_kinetic`; if the threshold is set wrong, causes audible hold-fighting (the Iteration-2/3 problem). Needs a `feedforward.apply_friction` feature flag.

**Benefit.** Directly targets the dominant mechanism (2.1). If tuned correctly, can fully eliminate the dead-zone.

**Recommendation.** Medium-to-long-term. Requires per-leg friction identification (a dedicated single-leg bench test). Do **not** implement blindly; mis-tuned friction FF is the most common cause of limit-cycle oscillation on machines that otherwise work fine.

### 3.6 Control: tune per-leg `pos_gain` UP (inverse of iteration-3)

**Change.** Iteration 3 of hold-fighting tuning *reduced* `pos_gain` on legs 1, 4 from 40 to 30 to suppress hold noise. A higher `pos_gain` would build up position error faster during a ramp-onset, shortening the dead-time. A staged exploration: try `pos_gain = 50, 60, 80` and see if onset-latency drops.

**Risk.** High — directly trades against hold-fighting, which Iteration 3 worked to suppress. Any increase in `pos_gain` risks reintroducing the hold-fighting Failure B.

**Benefit.** If `pos_gain · vel_int_gain` is the rate-limiting term and not mechanical friction, this is a one-line fix.

**Recommendation.** **Do not** attempt without mechanical audit (3.2) first. The control-system analysis suggests stiction dominates, which means raising `pos_gain` won't help — it will only increase hold noise while leaving the dead-zone intact.

### 3.7 Ranking summary

| Option | Risk | Benefit | Precondition | Sequence |
|---|---|---|---|---|
| 3.1 Motion-onset detector in diagnose.py | ~0 | Enables all A/B validation | None | **Ship first** |
| 3.2 Mechanical backlash audit | Low | Decisive for 2.2 | Hardware access | **Do before any software fix** |
| 3.3 Pre-move seating micro-pulse | Medium | Eliminates 2.2 only | 3.1 + 3.2 | Optional after 3.2 |
| 3.4 Vel-integrator pre-load | Medium-high | Shortens 2.5 | 3.1 + 3.2 | Optional |
| 3.5 Stribeck friction feedforward | High | Eliminates 2.1 | Per-leg friction ID bench test | Long-term |
| 3.6 Raise `pos_gain` | High | Small (only 2.5) | 3.1 + 3.2; unlikely to help | **Avoid** until mechanical is ruled in/out |

---

## 4. Recommended Test Battery

All tests should be runnable from the Jetson with the project venv (`source ~/Desktop/PDJ_venv/venv/bin/activate`). Match the format of the Level-1 gain-tuning methodology in [leg-gain-tuning-methodology.md](leg-gain-tuning-methodology.md).

### 4.1 Baseline re-measurement (before any fix)

**Purpose:** Establish motion-onset baseline using the new detector from 3.1.

**Procedure:**
1. Land the `analyse_motion_onset()` detector in diagnose.py (purely additive).
2. Re-run `python sim/analysis/diagnose.py --latest` on `mpc_20260418_164119.csv` and confirm `motion_onset_latency_ms ≈ 120` for Move 1.
3. Record baseline: `{per_leg_latency_ms[6], median_latency_ms, leap_magnitude_mm, leap_sync_window_ms}`.

**Acceptance:** detector is stable (≤5 ms repeatability on back-to-back runs of the same CSV).

### 4.2 Mechanical audit (before any software fix)

**Purpose:** Quantify backlash per leg.

**Procedure:** Manual per-leg measurement with platform removed:
1. Power IDLE. Attach dial indicator to platform-end of one leg.
2. Hand back-drive the ballscrew in one direction until indicator moves; reverse direction; measure lost-motion before indicator moves back.
3. Repeat 3× per leg; record `backlash_mm[6]`.

**Acceptance:** max per-leg backlash documented. If any leg ≥ 0.1 mm, tighten couplers / pre-tension cable / replace as needed; re-measure.

### 4.3 Warm vs. cold onset A/B (distinguishes 2.1 from 2.2)

**Purpose:** Separate stiction from backlash.

**Procedure:**
- Cold run: 10-minute idle, then the standard Apr 18 move sequence. Measure onset-latency on Move 1.
- Warm run: execute a "warm-up" trajectory (3× 5 mm steps) immediately before the same move sequence. Measure onset-latency on Move 1.

**Expected result:**
- If onset-latency drops substantially (e.g. 120 ms → 40 ms) on warm run, stiction dominates.
- If onset-latency unchanged, backlash dominates.
- Mixed result indicates both are contributing.

**CSV artifacts:** two sessions named `mpc_YYYYMMDD_HHMMSS_cold.csv` / `..._warm.csv`; compare via `compare_sessions.py`.

### 4.4 Direction-reversal A/B (isolates 2.2)

**Purpose:** Isolate backlash by comparing same-direction vs. reversed moves.

**Procedure:**
- Run A: `pose1 → pose2 → pose1` (same axis, reverses direction each move). Measure onset-latency on each move.
- Run B: `pose1 → pose2 → pose2 (no-op hold) → pose2_plus` (continues in same direction). Measure onset-latency on 2nd move.

**Expected result:**
- If onset-latency is high on every reversal in A but low on the continuation in B, backlash dominates.
- If onset-latency is high on every first-move regardless of direction, stiction dominates.

### 4.5 Iq-trace decisive test (confirms 2.1 vs. 2.5)

**Purpose:** Separate stiction (hardware nonlinearity) from integrator wind-up (software linear lag).

**Procedure:** Requires logging `iq_measured` at 500 Hz through motion-onset (check if the rosbag `leg_lengths_topic` or an equivalent topic already carries this; if not, add it for this test only).

**Expected result:**
- **Stiction:** `iq` ramps linearly during dead-time, then **drops** when mechanism breaks free.
- **Integrator wind-up alone:** `iq` ramps smoothly and continues ramping through breakaway (no drop).

### 4.6 Validation runs for each proposed fix

Once one or more fixes from §3 are applied, re-run the baseline (4.1) and compare via `compare_sessions.py`. Target metrics:

| Metric | Baseline | Target after fix | Stretch |
|---|---|---|---|
| median `motion_onset_latency_ms` | ~120 | ≤ 40 | ≤ 20 |
| leap magnitude on first tick after onset | 2.0 mm | ≤ 0.5 mm | ≤ 0.2 mm |
| `actual_jumps` count in 0–2 s window | 39 | ≤ 10 | 0 |
| hold-phase `act_std_um` (regression check) | 3–11 μm | ≤ 15 μm | unchanged |

The last row is a **regression guard** — any fix that shortens onset at the cost of hold-fighting is unacceptable (it reverses Iteration 3).

---

## 5. Prior-Art Review

### 5.1 What prior logbook iterations have tried

| Iteration | Entry | What was tried | What improved | What remained |
|---|---|---|---|---|
| 2026-03-30 | velocity-feedforward-oscillation | Fixed vel-ff semantic mismatch (`cmd - q_cur` → `cmd - prev_cmd`) | Violent oscillation (147–194 mm z-swing) eliminated; <0.5 mm tracking | Motion-onset untouched |
| 2026-04-17 | mpc-fallback-cmd-sawtooth-stutter | Walk-forward fallback along `prev_w`, rate-limited | Off-Active multi-axis stutter eliminated | Chronic solve-time overrun still triggers fallback |
| 2026-04-18 (a) | mpc-overhead-spikes-fallback-bursts | Bounded FK Newton-Raphson + plant-track extrapolation in hold | 30 ms FK spikes eliminated | IPOPT solve-time overrun deferred |
| 2026-04-18 (b) | hold-fighting-motion-onset-jitter (Iter 1–3) | Per-leg gains; dropped legs 1, 4 to 30/0.24 | Hold-stdev 14–25 μm → 3–11 μm (4× improvement) | **Motion-onset dead-time + 39-leap pattern unchanged** |
| 2026-04-19 | dead-band fix (abc4a8e) | Sub-LSB command dead-band in HardwarePlant | Hold-phase HF jitter suppressed | Motion-onset untouched |

**Conclusion:** No prior fix has targeted motion-onset. The gain-tuning iterations addressed only hold-phase fighting (Failure B in the hold-fighting entry); Failure A (motion-onset) was explicitly deferred.

### 5.2 What the Level-2/3 tuning methodology would buy us here

From [leg-gain-tuning-methodology.md](leg-gain-tuning-methodology.md):

- **Level 2 (step-response tuning)** fits rise-time, overshoot, settling for a closed-loop PID on a 5 mm step. It measures bandwidth, not friction. The methodology's own Gotchas section acknowledges *"Backlash eats the first ~0.2 mm of any step"* and tells the operator to use ≥5 mm steps to work around it — **it does not detect or compensate for backlash.** It also does not measure stiction breakaway.
- **Level 3 (system ID + loop shaping)** fits a second-order plant from a chirp. A second-order model **cannot represent** a dead-zone or Coulomb friction; the identification would smear the nonlinearity into the linear fit and produce a suboptimal controller.

**Therefore:** running Level 2 or Level 3 procedures on the current hardware will *not* fix the motion-onset dead-time. Those procedures assume the plant is linear within the operating region, which the observed "flat then leap" signature contradicts. The mechanical audit (3.2) and a friction-specific identification (precondition for 3.5) are the right tools, not generic PID tuning.

### 5.3 Why the vel-ff A/B and dead-band fix did not close this

- **Vel-ff A/B (Apr 18):** vel-ff adds a velocity feed-forward term that anticipates `vel_cmd`, providing torque before position error builds up. This helps with a linear plant but **cannot drive torque above breakaway when the commanded velocity is below stiction threshold**. Hence identical 39-jump pattern in both A/B arms, shifted in time because cmd-crossing of breakaway happens later.
- **Dead-band fix abc4a8e:** suppresses sub-LSB jitter during hold. Motion-onset happens when the cmd *exits* hold, at which point cmd-delta exceeds the dead-band by design. The fix is inert during onset.

---

## 6. Open Questions & Alignment Asks

Before any fix from §3 is implemented, I want alignment on:

1. **Hypothesis agreement.** Do you buy the ranking in §2.8 (stiction #1, backlash #2, integrator wind-up #3)? If not, which candidate is under-weighted?

2. **Mechanical-first sequencing.** I'm proposing 3.1 (detector) → 3.2 (mechanical audit) → pick-next-fix-based-on-audit-result. Is the mechanical audit (3.2) tractable on the current hardware (i.e. can you actually measure per-leg backlash)?

3. **Iq logging availability.** Is `iq_measured` already logged at 500 Hz in the rosbag `leg_lengths_topic`? If not, should I add it as a precondition for 4.5?

4. **Risk tolerance on 3.3 (pre-move seating pulse).** A 50 μm counter-pulse before every hold→move transition is a visible hack. Is that acceptable if mechanical backlash turns out to be unavoidable, or is the policy "fix it mechanically or not at all"?

5. **Friction identification bench test.** Is there appetite for a dedicated single-leg test harness (possibly under [tests/hardware/single_leg_test.py](../../tests/hardware/single_leg_test.py)) that measures per-leg `τ_stiction` and `τ_kinetic` via slow-ramp-to-breakaway? This is a precondition for 3.5 and also valuable input to Level 3 loop-shaping.

Awaiting approval on diagnosis + fix plan before writing any code.

---

## 7. Appendix — Related Files & Line Numbers

- [ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py:703-742](../../ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py#L703-L742) — Hermite interpolation formula
- [ros_ws/src/jugglebot/jugglebot/can_node.py:591-641](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L591-L641) — `_setup_odrives_steps` (first-boot PASSTHROUGH switch)
- [ros_ws/src/jugglebot/jugglebot/can_node.py:1440-1562](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L1440-L1562) — `_gentle_move_steps` (activation path)
- [ros_ws/src/jugglebot/jugglebot/can_node.py:703-723](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L703-L723) — `_send_position_target` (CAN encoding)
- [controller/hardware_plant.py](../../controller/hardware_plant.py) — ZMQ bridge, dead-band (abc4a8e)
- [config/hardware_config.yaml:198-200](../../config/hardware_config.yaml#L198-L200) — current per-leg gains
- [sim/analysis/diagnose.py:372-402](../../sim/analysis/diagnose.py#L372-L402) — `actual_jumps` detector (threshold 2 mm)
- [logbook/2026-04-18-hold-fighting-motion-onset-jitter.md](../../logbook/2026-04-18-hold-fighting-motion-onset-jitter.md) — primary prior-art entry, Failure A section
- [plans/active/leg-gain-tuning-methodology.md](leg-gain-tuning-methodology.md) — Level-2/3 procedures, Gotchas
