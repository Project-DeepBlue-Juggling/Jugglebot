---
title: Leg ODrive PID Gain Tuning Methodology
status: active
owner: harrison
created: 2026-04-18
last_updated: 2026-07-12
related_logbook:
  - 2026-04-18-hold-fighting-motion-onset-jitter.md
  - 2026-04-19-leg1-pose-dependent-hold-twitch.md
  - 2026-07-10-s4-stutter-guard-forensics-recovery-stack.md
  - 2026-07-12-bench-leg-gain-tuning-stage1.md
related_config:
  - config/hardware_config.yaml → jugglebot_odrive_defaults.leg_pos_gains / leg_vel_gains / leg_vel_int_gains
related_code:
  - ros_ws/src/jugglebot/jugglebot/can_node.py::_set_leg_gains
  - ros_ws/src/jugglebot/jugglebot/can/odrive.py::DEFAULT_LEG_GAINS
---

> **2026-04-20 status:** Level-1 converged with all six legs uniform at the
> original flash baseline **40/0.20/0.32**. The per-leg reductions Iteration-3
> introduced (legs 1 and 4 at 30/0.24) were **reversed** after they were
> shown to cause a pose-dependent hold-phase limit cycle at (0,−100,200).
> See the updated "Outcome for the hardware-bringup stage" section at the
> bottom of this document before opening a new tuning round — specifically,
> the Level-1 procedure as originally written tunes against a single pose
> and missed a failure mode that only shows up at extreme off-axis poses.
> Future rounds should extend the test battery to the extremes **before**
> reducing any gain from the uniform baseline.

# Leg ODrive PID Gain Tuning Methodology

## Purpose

Per-leg ODrive position-loop gains (`pos_gain`, `vel_gain`, `vel_integrator_gain`)
are now configurable via `config/hardware_config.yaml` and applied over CAN at
boot by `can_node._set_leg_gains()`. This document describes how to pick the
numbers that go in the YAML.

Three levels are described, in increasing order of rigour and time cost. Pick
the level that matches the claim you want to defend:

- Level 1 — empirical A/B → "this gain makes leg N quieter at hold without
  hurting tracking"
- Level 2 — step-response → "this gain gives leg N a 45 ms rise time and 8 %
  overshoot to a 5 mm position step"
- Level 3 — system ID + loop shaping → "this gain gives leg N a 60° phase
  margin at a crossover frequency 10× its mechanical resonance"

## Context: why per-leg gains?

The six Jugglebot legs are mechanically near-identical but not truly identical.
`mm_to_rev` already varies per leg (`config/hardware_config.yaml:125`), reflecting
build-to-build differences in spool diameter and thread engagement. Bearings,
couplers, and cable routing add further leg-specific compliance and friction.
A single uniform gain vector that works for the stiffest leg will over-drive
the softest leg and produce the exact audible motor-fighting signature
described in `logbook/2026-04-18-hold-fighting-motion-onset-jitter.md`.

Per-leg gains let us push each leg as hard as its mechanics will tolerate,
without sacrificing any leg to the weakest member.

## Assumptions and invariants

Across all three levels:

- `MAX_LEAD_REV = 0.15 rev` clamp in `motor_guard.py` remains in place as the
  hard safety floor. No amount of gain mis-tuning can cause a runaway faster
  than this clamp allows.
- Workspace/position limits in `can_node._send_position_target` remain in
  place. A leg cannot be driven outside its stroke.
- All gain edits go through `config/hardware_config.yaml`, then
  `python config/generate_config.py`, then a ROS2 rebuild. Never set gains
  live via ODriveGUI as a durable configuration — those values are lost on
  flash re-program and are invisible to the codebase.
- Keep `min(pos_gain) / max(pos_gain) ≥ 0.4` across the six legs. Larger
  asymmetry drives platform twist on straight-line moves (leg N lags → pose
  tilts during transit).
- Keep the ratio `pos_gain : vel_integrator_gain ≈ 125 : 1` unless you have a
  specific reason to break it (Level 2 or 3 tuning). This ratio preserves the
  outer-to-inner-loop bandwidth relationship that the ODrive firmware assumes.

## Level 1 — Empirical A/B on the outlier

Use when: you have an observed hold-phase jitter outlier and want to bring
it into line with the quietest leg. Cost: ~30 min per leg. Defensibility:
low — you're tuning against a single operating point and a single metric.

### Procedure

1. **Baseline measurement.** With all six legs uniform (current default
   `40.0 / 0.20 / 0.32`), run one Move 1 (Active → z=220, 15 s) and hold for
   5+ s at the target. In the resulting `temp/logs/mpc_*.csv`, compute
   `actual_ext_i` stdev per leg over the last 3 s at hold. That stdev in
   micrometres is the hold-quiescence metric. Rank the six legs by it.
2. **Target.** The quietest leg sets the floor — call it μ_best. Any leg with
   `actual_ext stdev > 1.5 × μ_best` is an outlier to tune.
3. **Halve → test → go between or halve again.**
   - Try `pos_gain × 0.5` on the outlier. Keep all other legs unchanged.
   - Edit `config/hardware_config.yaml`, run `python config/generate_config.py`,
     rebuild ROS2 (`cd ros_ws && colcon build --packages-select jugglebot &&
     source install/setup.bash`).
   - Re-run Move 1.
   - If hold stdev drops toward μ_best and tracking RMS does not rise more than
     1.5× on the pure-Z move: good direction.
   - If halving overshoots (new stdev < μ_best *but* tracking RMS or final
     settling error grows materially): try `pos_gain × 0.75` instead.
   - If halving didn't help at all: `pos_gain` wasn't the problem on this leg.
     Move to `vel_integrator_gain` (halve it independently, keep `pos_gain`
     at baseline).
   - `vel_gain` stays constant unless both outer and integrator tuning fail —
     that's a signal the inner loop bandwidth is wrong, which is rare.
4. **Ratio rule.** When you drop `pos_gain` by factor α, drop
   `vel_integrator_gain` by the same α. This keeps the integrator wind-up
   proportional to the now-sluggish position loop.
5. **Stop condition.** All six legs within 1.5× of μ_best on hold stdev AND
   aggregate tracking RMS within 10 % of the pre-tuning value on every move
   in the recommended test battery (pure-Z up, pure-Z down, pure-X, pure-Y,
   diagonal).
6. **Record.** Final values get committed to `config/hardware_config.yaml`;
   the tuning session gets a logbook entry with before/after hold-stdev
   numbers, final gain values, and the `/diagnose --compare` pre/post
   comparison.

### Failure modes and what they look like

| Symptom | Diagnostic | Fix |
|---|---|---|
| Platform drifts in gravity; sluggish start of motion | Tracking RMS up, final settling > 1 mm | Position loop too soft — back off gain reduction |
| Sudden 5–10 mm correction long after move ends | Integrator wind-up under load | Reduce `vel_integrator_gain` first, before touching `pos_gain` |
| Platform twists on straight-line move | `min_pos_gain / max_pos_gain < 0.4` between legs | Tighten the asymmetry — raise the weakest leg or lower the stiffest |
| Hold stdev improved but a new leg now fights | Re-ranked outlier | Expected — iterate |

### Test battery for Level 1 acceptance

Every round of gain changes must pass all of these before the new values
are committed:

1. `Active → z=220, hold 5 s` (long Z-up)
2. `z=220 → Active, hold 5 s` (long Z-down)
3. `Active → x=50, hold 5 s` (pure X)
4. `Active → y=50, hold 5 s` (pure Y)
5. `Active → (30, 40, 200, 0.05, 0.05, 0), hold 5 s` (diagonal with tilt)
6. **`Active → (0, −100, 200, 0, 0, 0), hold 5 s`** (extreme Y offset —
   added 2026-04-20 because this pose exposed a gain-margin failure that
   none of the five near-centre moves above could see; see
   `logbook/2026-04-19-leg1-pose-dependent-hold-twitch.md`)
7. **`Active → (100, 100, 200, 0, 0, 0), hold 5 s`** (extreme diagonal
   corner — companion to (6); also stresses IPOPT solve-time budget)

Pass criteria: per-leg hold stdev within 1.5× of μ_best on **every move
including the extreme-pose moves 6 and 7**, no error-severity flags from
`/diagnose`, no motor errors in the rosbag.

> **Cautionary note — extreme-pose tests are non-negotiable.** The
> Level-1 procedure as originally written (without moves 6 and 7) converged
> cleanly with legs 1 and 4 at reduced 30/0.24 gains, but that "converged"
> state had a 60× hold-phase asymmetry waiting at (0,−100,200). Gain
> margin has a pose-dependent component — the load distribution, and thus
> the effective phase margin of the closed-loop, changes with platform
> pose via the CoM offset and the Jacobian row norm. Do not declare a gain
> set "done" without exercising the full workspace.

## Level 2 — Step-response tuning

Use when: you need a defensible number for a logbook entry, a paper, or a
spec document. Cost: ~2 h per leg. Defensibility: medium — you're tuning
against a standardised input and a small set of well-known metrics.

### Procedure

1. **Switch the leg under test to TRAP_TRAJ control mode** (via CAN:
   `encode_set_controller_mode(axis_id, 'POSITION', 'TRAP_TRAJ')`). Other
   legs stay in PASSTHROUGH under normal operation or are disarmed — the
   point is to isolate one leg's dynamics.
2. **Support the platform mechanically** so the leg under test sees
   approximately its operational load (the platform + gravity share allocated
   to that leg at home pose). Do not run this test with the platform in free
   hang — the load is wrong.
3. **Command a 5 mm step** on `input_pos` using a Python script that talks
   to the leg directly over CAN. 5 mm is large enough to exit the noise
   floor but small enough to stay in the linear regime.
4. **Capture `pos_estimate` at 500 Hz** from the rosbag `/leg_lengths_topic`.
5. **Fit:**
   - `t_rise` (10 % → 90 % of step height)
   - `t_settle_2%` (time until error stays within ±2 % of final)
   - `% overshoot` (peak deviation past final, as fraction of step height)
6. **Targets for a compliant Stewart leg:** `t_rise ≈ 30–50 ms`,
   `overshoot ≈ 5–10 %`, `t_settle_2% ≈ 100 ms`. These come from the
   mechanical-bandwidth-to-control-bandwidth separation required for
   Stewart platforms at this scale; confirm with a small pilot before
   using them as hard targets.
7. **Binary-search `pos_gain`** to hit `t_rise` and `overshoot` targets:
   - Too slow rise → raise `pos_gain`
   - Too much overshoot → lower `pos_gain` OR raise `vel_gain`
8. **Add `vel_integrator_gain`** until final error goes to zero within
   `t_settle_2% × 2`. Increase by 50 % per trial; watch for late oscillation
   (the signature of too-high integrator gain).
9. **Repeat per leg.** Legs with different `mm_to_rev` will settle on
   different gains — that's the expected outcome.
10. **Record** each leg's fitted rise/overshoot/settle alongside its final
    gains in the logbook entry.

### Gotchas

- **Load matters.** A leg tuned with no platform weight will oscillate when
  the platform is added. A leg tuned at home pose may be soft at extension
  extremes.
- **Backlash eats the first ~0.2 mm of any step.** Use a 5 mm step or larger
  to keep backlash a small fraction of the signal.
- **The first move after a long rest has higher stiction than subsequent
  moves.** Do three warmup steps before collecting data.
- **TRAP_TRAJ itself imposes a velocity/accel profile.** This is the right
  mode for PID step response (versus PASSTHROUGH which gives an instantaneous
  step) because it isolates tracking performance from setpoint slew. If you
  want to characterise the PID against an instantaneous step, use
  PASSTHROUGH and a jog command — but be ready for the clamp on leg stroke
  to fire if the response is unstable.

## Fast-motion tier (Level-2f) — dynamic-tracking damping (two-stage: bench → Jugglebot)

**Added 2026-07-10; rewritten 2026-07-11 into a two-stage bench-first methodology.**
Levels 1–3 as written above all optimise **hold** behaviour (hold-phase stdev,
single-pose quiescence). The 2026-07-10 ~6 Hz stutter proved a second, orthogonal
regime that Level-1 does not cover: **fast-motion tracking under raised session
limits**. The Level-1 tier (`40 / 0.20 / 0.32`) is quiet at hold but its velocity
loop is too lightly damped to track an aggressive vertical stroke — the position
loop rings at its own bandwidth (`pos_gain/(2π)` = 6.37 Hz), the ODrive current
surges and brakes cycle-by-cycle, and on the biggest/fastest stroke the accumulated
command-vs-encoder lead crosses the 0.5-rev `MAX_DEVIATION` guard and latches. See
`logbook/2026-07-10-s4-stutter-guard-forensics-recovery-stack.md` and the on-robot
session protocol `tests/hardware/session_gain_retune.md`.

Use when: a gain set that passed the Level-1 HOLD battery produces an audible/visible
limit cycle, braking-current oscillation, lead-clamp engagement, or a spurious
`MAX_DEVIATION` latch on a fast stroke at raised limits.

**Why two stages.** The original single-stage plan tuned the *real Jugglebot legs*
directly (the `session_gain_retune.md` 3-point sweep). That is slow (every gain point
costs a YAML→codegen→colcon→relaunch cycle) and, more importantly, it pushes the real
legs into the ring/latch regime repeatedly to find the edge of stability. We now have a
**7th leg** — same motor (ODrive D6374-150Kv), same ODrive Pro, same ballscrew/spool
class as the six platform legs, but a **shorter stroke** and mounted on an
acceptable-loss bench rig with the platform's brake resistor attached. The bench leg
lets us do the *aggressive* work — system-ID, an escalate-until-unstable gain ladder,
and the loop-vs-structure discriminant — on hardware we are willing to break, then
transfer a **conservative, derated** result to the real robot with a short confirmation
pass. Stage 1 finds the answer; Stage 2 lands it safely.

### Target (both stages)

**Closed-loop damping ratio ζ ≥ 0.7 (≤ ~5 % overshoot) at the position-loop bandwidth.**
On the fixed fast vertical stroke (z 170→250 at 156/660/10500) the tier is met when:

- there is **no braking-current oscillation** (no negative-iq surge/collapse cycling);
- **position lag < 0.05 rev** with ≥2× margin below `MAX_LEAD_REV` (0.10 rev,
  `canbridge_config.h:178`), so the interp lead clamp never engages (`lead_clamp_mask`
  stays 0 on `/link_status`);
- the stroke is subjectively smooth (no ~6 Hz buzz) and the guard never latches.

Tighter than Level-2's 5–10 % overshoot target — the dynamic regime must ring *less*,
not just settle, because a sustained ring is what accumulates the lead that trips the
guard. From step-response overshoot, `ζ = −ln(OS)/√(π²+ln²OS)`; `OS ≤ 5 %` ⇒ `ζ ≥ 0.69`.

---

## STAGE 1 — bench leg (aggressive): system-ID, escalate-until-unstable, discriminate

The bench leg is acceptable-loss hardware. Push it. The goal of Stage 1 is to produce
three things the real robot cannot cheaply give: (a) a **measured plant + closed-loop
bandwidth** for the leg servo, (b) an **aggressively-optimised gain triple** found by
climbing until instability onset and backing off, and (c) a **definitive loop-vs-
structure verdict** for the 6 Hz ring — obtained by reproducing (or failing to
reproduce) the ring on the *isolated* leg, which has the same servo but **no platform
structure**. That last point is the scientific payoff the bench uniquely enables: the
on-robot `pos_gain`-tracking test can only *infer* loop-vs-structure; the bench can
*remove the platform entirely* and settle it.

### Bench leg — verified facts and hard limits (respect these)

MEASURED from the bench harnesses and config (cite when you set a cap):

- **Geometry:** one motor rev extends the bench leg **71.5708 mm**
  (`tests/hardware/single_leg_test.py:106`), spool radius ≈ 11.39 mm. The platform legs
  are ~70.5 mm/rev (`hardware_config.yaml:233`, `mm_to_rev[0]=0.01418`) — a ~1.5 %
  geometry difference, so the bench leg is *structurally near-identical, not identical*.
- **Stroke (SHORTER than a platform leg):** the bench leg's full mechanical travel is
  **3.0 rev** (`cogging_bench_test.py:126`, `HARD_POSITION_CAP_REV`), i.e. ≈ 214.7 mm,
  vs a platform leg's ≈ 3.97 rev / 280 mm (`hardware_config.yaml:182` `leg_stroke_mm`,
  firmware `STROKE_MAX_REV[0]=3.90`). **Operator-confirmed 2026-07-11:** the Phase-11
  values are the true limits — ≈3.4 rev physical, 3.30 rev software ceiling
  (`logbook/2026-06-24-phase11-bench-cutover.md`, U3-iv). **The 3.0 rev hard cap is
  retained DELIBERATELY** (operator: "a little extra margin won't hurt given the testing
  we're going to be doing"). `teensy_setpoint_bench.py:85`'s 3.90-rev figure is the
  *production* leg-0 clamp, not a bench measurement (see the Stage-1c stroke-safety
  bullet); crashing the leg into an end-stop can damage the ballscrew/cable/motor.
- **Velocity ceiling:** **35 rev/s = 2.5 m/s** of leg-end velocity
  (`cogging_bench_test.py:105-110`), enabled by the borrowed brake resistor (2026-04-27).
  This is the bench *survival* cap — **do not use it as the tuning velocity.** Reproduce
  the ring at **Jugglebot's operating point (~2 rev/s peak, the stutter regime)**, not
  the bench's max, so the identified plant/gains are relevant to the real robot.
- **Acceleration ceiling:** **250 rev/s²** (`cogging_bench_test.py:137-148`),
  brake-resistor-aware; achievable inside the 10 A current limit (`τ=J·α`, ~7 A at
  250 rev/s²).
- **Current limit:** **10 A** (`cogging_bench_test.py:117`) — same as a platform leg
  (`hardware_config.yaml:287`, `leg_curr_limit_a`).
- **Power:** the bench 21 V supply is back-EMF-marginal above ~1 rev/s
  (`logbook/2026-06-24-phase11-bench-cutover.md`, U3-iv). For fast-stroke gain work at
  ~2 rev/s **use the 48 V PSU + brake resistor.**

### Stage 1a — system-ID (do this first, on Path BRIDGE)

**Single-path reality (operator-confirmed 2026-07-11):** direct CAN on the Jetson has been
wound down entirely — the Jetson no longer sees CAN, and there is no USB-CAN adapter. The
bench leg is reachable **only** as CAN3 node 0 behind the can-bridge Teensy over the UDP
`teensy_link` protocol (**Path BRIDGE**). So Stage 1a **and** Stage 1b both run on the
bridge, with the honest degradations below. The former "Path DIRECT" (socketcan straight to
`input_pos`/`input_vel`) is **retired** and kept only as a one-line historical note in the
Topology section; the direct primitives are unrunnable here.

Identify the plant before touching gains. Order matters: inner loop out.

1. **Current-loop sanity.** The current loop is an ODrive-firmware kHz loop, not a user
   gain — you only confirm it is healthy, not tune it. On Path BRIDGE there is no raw
   `TORQUE/PASSTHROUGH` torque-step primitive from the harness; instead confirm health
   indirectly from the DIAGNOSTIC `iq_measured` trace during the position-step ramps (a
   clean, non-oscillating current that tracks the commanded acceleration). A dedicated
   torque-step needs a bridge RPC that does not exist today — treat current-loop health as
   a *precondition*, not a Stage-1a measurement.
2. **Velocity-loop step responses — INFERRED, not measured.** The bridge has no raw
   `input_vel` step, so `vel_gain` / `vel_int_gain` are **inferred from the position-step
   overshoot** (step 3): a lightly-damped velocity loop shows up as position-step overshoot
   and post-step velocity ripple. This is the largest Stage-1a degradation vs the retired
   direct path (which could isolate the velocity loop) — document the inference in the
   winner's logbook entry.
3. **Position-loop step + sine-sweep frequency response (bridge knot stimuli).** Streaming
   `POSITION/PASSTHROUGH` knots through the firmware 500 Hz Hermite:
   (a) **"steps" become 25 ms-quantised knot ramps.** The bridge cannot issue an
   instantaneous step; the harness ramps each 0.07–0.28 rev (5–20 mm) step at a per-knot
   increment **under the 0.10 rev lead clamp** (`bench_leg_sysid.py` sizes it at 0.5× the
   clamp = 0.05 rev/frame = 2.0 rev/s, Jugglebot's operating point — so the lead clamp never
   engages and `lead_clamp_mask` stays 0). Fit rise / overshoot / settle of the leg tracking
   that fast ramp (→ closed-loop damping); it approximates a step because the 500 Hz interp
   is well above the plant. **The step's apparent bandwidth is ramp-duration-limited, not a
   loop measurement:** `0.35/rise` on this fixed 2.0 rev/s ramp reads the *ramp*, not the
   loop (a ~6 Hz pos loop's ~58 ms rise is comparable to the 50–150 ms ramp of the smaller
   steps), so the manifest flags `f_bw_ramp_limited` with its ceiling `~1/(0.8·ramp_dur)`;
   take the honest loop bandwidth from the chirp below, **not** the step.
   (b) **chirp via knot streaming.** A log chirp is streamed as 40 Hz position knots,
   amplitude-bounded by the stroke, the vel/accel kinematic caps, **and the lead clamp**
   (so the sweep keeps the same 2× lead margin the steps have and the streamed knots stay a
   faithful lock-in reference — the vel cap alone leaves it riding the 0.10 rev clamp) in
   the harness. The
   top frequency is **bounded first by the measured telemetry rate and the 40 Hz knot rate**
   — the harness measures the TELEMETRY per-axis rate + irregularity over a warmup window
   BEFORE any stimulus and sets `f1 = min(request, telemetry bound, knot-stream bound)`. At
   ~100 Hz telemetry / 40 Hz knots that is **≈8 Hz** — enough to resolve the 6 Hz pos-loop
   question (`pos_gain/(2π)`) but **not** the 30–80 Hz structural band the retired direct
   path could reach. Estimate the closed-loop frequency response over the honest band; the
   mechanical `ω_n` (20–80 Hz) is **out of the bridge's honest reach** — a Level-3 note, not
   a Stage-1a deliverable on this path.

**The harness:** `tests/hardware/bench_leg_sysid.py` (`--path bridge`, the default and only
runnable path) implements all of the above; its CAN-free logic — onset/divergence detectors,
the gain-escalation ladder, the stroke-cap-bounded + lead-clamp-sized knot stimulus
generators, the telemetry-rate estimator, and the guard-latch backoff state machine — lives
in `tests/hardware/sysid_lib.py` and is unit-tested in `tests/motion/test_bench_sysid_logic.py`
+ `tests/motion/test_bench_sysid_bridge.py`. Gains apply **session-only over CAN3 via the
`SET_POS_GAIN`/`SET_VEL_GAINS` RPCs** (`controller/teensy_link/rpc_args.py`), re-applied per
point — instant, RAM-only, no rebuild. The ladder **disarms (`mpc_active=0`) across the two
RPC gain calls**, because the blocking RPCs (up to ~1.5 s worst-case with retries) can
otherwise straddle the 250 ms MPC-staleness window while armed and latch the guard;
disarming suppresses the firmware staleness check across the gap, then it re-arms to stream.
Run `--dry-run` first to see the full bounded plan with no socket opened.

### Stage 1b — escalate-until-unstable gain ladder (aggressive, auto-backoff)

Climb `pos_gain` in an explicit ladder; at each rung, add just enough velocity-loop
damping to hold `ζ ≥ 0.7`; stop when instability can no longer be damped out. `vel_int`
follows the `pos_gain : vel_int ≈ 125 : 1` ratio rule unless a rung's step response says
otherwise.

| rung | `pos_gain` | pos-loop BW `/(2π)` | `vel_int` (=pos/125) | `vel_gain` search |
|---|---|---|---|---|
| 0 | 25 | 3.98 Hz | 0.20 | {0.20 … 0.45} for ζ≥0.7 |
| 1 | 40 (baseline) | 6.37 Hz | 0.32 | {0.20 … 0.50} |
| 2 | 55 | 8.75 Hz | 0.44 | {0.30 … 0.60} |
| 3 | 70 | 11.1 Hz | 0.56 | {0.35 … 0.75} |
| 4 | 90 | 14.3 Hz | 0.72 | {0.45 … 0.90} |

**Instability-onset criteria (any ⇒ back off immediately):**

- sustained (non-decaying or growing) oscillation in `pos_estimate` on a step;
- `iq` limit-cycle / braking-current cycling (negative-iq surge/collapse);
- step overshoot > ~15 % (`ζ < 0.55`) that the available `vel_gain` cannot pull back;
- `vel_estimate` limit cycle at hold or after the step;
- audible ~6 Hz+ buzz.

**Auto-backoff (Path BRIDGE):** on any onset criterion the harness reverts to the last
stable triple, **disarms** (`mpc_active=0` — the firmware output gate closes in ≈one fault
step), and logs the rung. The **firmware safety machine is the backstop and must not be
fought**: the per-leg `MAX_DEVIATION` E-STOP (0.5 rev) and the MPC-staleness E-STOP stay
armed underneath the harness. When a stimulus is too hot the guard *latches*
(`fault_state != NONE` on the T→J heartbeat); the harness **detects the latch, backs the
ladder off to last-good, disarms, and recovers via a `CLEAR_ERRORS` RPC** (the firmware
re-enable slew makes that gentle) — with a bounded recovery budget so it never keeps
climbing into a guard it cannot satisfy (`sysid_lib.GuardLatchBackoff`). Only a **recoverable**
fault (feedback-stale / link-lost — the two the firmware self-clears when feedback/link
returns) is watched, not fought; the **latching** guard E-STOPs (MPC-staleness /
`MAX_DEVIATION` / motor-overspeed all set the sticky `s_estop_latched`, released only by
`CLEAR_ERRORS`) drive the back-off + `CLEAR_ERRORS` recovery above — MPC-staleness is not
merely watched, because the firmware does not self-clear it; a fatal fault (ODRIVE_FATAL /
CAN-bus-down) cedes authority so the firmware deferred-stow runs uncontested.
**Stroke-limited stimuli (harness-owned):** the firmware `STROKE_MAX_REV[0]=3.90` is the
*production* clamp and does **NOT** protect the 3.0-rev bench leg, so **the harness owns the
3.0-rev cap** — every step/chirp fits the 3.0-rev stroke, accel ≤ 250 rev/s², current
≤ 10 A, and the knot ramps stay under the 0.10 rev lead clamp. **Stop the ladder** when
either (i) no `vel_gain` at the next rung holds ζ ≥ 0.7 (the pos_gain ceiling — record it),
or (ii) the closed-loop bandwidth comfortably clears the 40 Hz interp / stutter regime with
ζ ≥ 0.7 margin (you have enough; do not chase the ceiling for its own sake). The **winning
bench triple** = highest `pos_gain` that clears the Stage-1c ring test with ζ ≥ 0.7 and the
smallest tracking lag.

### Stage 1c — loop-vs-structure discriminant (the bench's unique payoff)

Reproduce the 6 Hz ring on the bench leg under the **same 40 Hz position-interp drive**
Jugglebot uses (the can-bridge path — the ring reproduction needs the exact production interp
drive, so use the fixed-gain `teensy_setpoint_bench.py` β-knot source here rather than
`bench_leg_sysid.py`'s sweep), at **Jugglebot-equivalent gains (`40 / 0.20 / 0.32`)**, on the
fixed fast stroke sized to ~2 rev/s peak:

- **Ring PRESENT on the isolated bench leg** ⇒ the ring is the **servo loop** — it
  reproduces with the same leg servo and **no platform structure at all**, so the
  platform is exonerated. Tune it out with the Stage-1b ladder. As a confirmation, sweep
  `pos_gain {25,40,55}` on the bench and verify the ring frequency **tracks** `pos_gain/
  (2π)` (≈ 4.0 / 6.4 / 8.8 Hz) — the same discriminant the on-robot session runs, but now
  with the platform removed.
- **Ring ABSENT on the isolated bench leg** at `40/0.20/0.32` ⇒ the ring **needs the
  platform** — it is structural / inter-leg-coupling, not the leg servo. **Gains are not
  the lever.** Stop; hand off to the interpolant / lead-clamp / mechanical-resonance path
  (2026-07-10 forensics Rank 3/4). Do not persist any gain change.

INFERRED caveat: the bench leg's reflected inertia differs slightly from a loaded
platform leg, so a ~6.37 Hz ring on Jugglebot may appear at ~6.0–6.3 Hz on the bench —
still unambiguously the `pos_gain/(2π)` loop signature, not a coincidence.

---

## STAGE 1 — RESULTS (2026-07-12 bench session)

**Added 2026-07-12.** The first full Stage-1 session ran on the bench leg over Path BRIDGE across
four analysis rounds, each analysed by parallel Opus tracks with **every load-bearing claim
adversarially verified against the raw CSVs and source**. Full narrative, the withdrawn hypotheses,
and the commit list are in `logbook/2026-07-12-bench-leg-gain-tuning-stage1.md`. The load-bearing
outcomes:

### Envelope verdict (unloaded bench leg, v3 lineage)

- **pos 40** — clean everywhere (production baseline).
- **pos 70** — the accuracy knee (see the err-vs-HF table below).
- **pos 90** — aggressive-but-bounded edge; the first **real** motion-excited vibration onsets
  (iqHF to 5.6× baseline on fast long strokes) but still recoverable.
- **pos 110+** — over the line. The ladder's 110/0.50/0.72 "winner" was **overturned** (below);
  pos 130 is dead at all vel_int {0.72, 0.55, 0.40} — hair-trigger and intermittent, i.e. the
  boundary itself, not a vel_int-rescuable point.

### Discriminant outcome — the S4 ~6 Hz did NOT reproduce on the isolated leg

The unloaded bench leg **did not reproduce the S4 5.9–6.1 Hz limit cycle in any regime** — not on
the escalate-until-unstable ladder, not in `--mode track` sustained tracking, and not even
riding the lead clamp to `clamp_frac = 0.456`. Arrive-and-settle stimuli are the wrong excitation
class (tails settle dead-flat within ~4 frames); the robot fails only during *sustained* ~2 rev/s
strokes with the clamp engaged. **v2/v3 context:** the S4 cycle ran on **v2** firmware (vel_ff
zeroed at clamp engage, MAX_LEAD 0.15); the bench ran on the **v3** lineage (vel_ff kept, MAX_LEAD
0.10), which suppressed the aggravator on the *unloaded* leg. So the S4 signature **may already be
fixed on the robot by v3 alone** — but a single unloaded leg cannot certify a gravity-loaded,
inter-leg-coupled 6-leg platform. **The loaded S4 replay on the robot is the deciding test**
(Stage-2 acceptance gate).

### Chirp friction finding + multi-amplitude protocol

The chirp gain of ~0.5 at a 0.02 rev (2.86 mm p-p) amplitude is **real Coulomb-friction
small-signal attenuation, not an estimator artifact** (coherence 0.96–1.0; large steps reach DC
gain 1.01–1.03; four analysis tracks converged; the `2026-04-27` friction bench `tau_c = 1.094 A`
is corroborated within 10–40 %). The measured magnitude *rises* into the ceiling (not a linear
rolloff) as peak velocity crosses the stiction knee `omega_s = 0.251 rev/s` — a friction
describing-function, **not a loop corner**. Amendment to the chirp protocol: sweep **≥2 amplitudes**
(`--chirp-amps`, default 0.02/0.06/0.12 rev) as the definitive friction-vs-linear test — friction
predicts gain rising toward 1 with amplitude, a linear plant predicts amplitude-independence.

### Measurement-ceiling lessons (fold into any future bench session)

- **The honest chirp ceiling is KNOT-bound at 8 Hz** on 40 Hz knots (`knot_stream_top_freq =
  (1/0.025)/5`), not telemetry-bound; the 100 Hz-knot bench build lifts it to ~20 Hz.
- **Stock iq is on-change-aliased** (the DIAGNOSTIC frame fires only on iq-setpoint change >0.5 A
  or 1 Hz), so peak/hold iq is unmeasurable without the bench firmware's un-gated 250 Hz axis-0
  iq. Distrust any single-run iq peak trend on stock firmware.
- **Log at the telemetry rate, not the knot rate** — the stock CSV was decimated to 40 Hz
  (Nyquist 20 Hz) while 100 Hz telemetry was already on the wire; the Run-A harness now writes one
  row per received frame.
- **Knot rate does not change loop dynamics** (pos-90 ζ = 0.826 at 40 Hz knots vs 0.816 at
  100 Hz) — it changes the *instrument* (chirp ceiling, iq resolution) and the *actuation texture*
  (less 500 Hz-Hermite staircase). Do not read a knot-rate ζ change as a control-gain change.

### Err-vs-HF trade (RUN-A strokes battery, unloaded)

| gain point | mean errRMS | iq HF churn (mean) | verdict |
|---|---|---|---|
| pos 40 (0.20 / 0.32) | 10.3 mm | 0.27 A (1.0×) | clean everywhere |
| **pos 70 (0.35 / 0.56)** | **6.9 mm (−33 %)** | 0.51 A (1.81×) | **accuracy knee** |
| pos 90 | 6.8 mm (−1 % vs 70) | 0.66 A (2.4×); peak 1.21 A (5.6×) on a long stroke | aggressive edge, real vibration |
| pos 110 (0.50 / 0.72) | — | motion iq 3.7×; ζ≈0.5 @17.5 Hz; rail 1.26 % | over the line (overturned) |

Accuracy saturates at pos 70; pos 90 buys ~1 % more for 2.4× churn and stronger real onsets. On
all 60 completed sym/asym points the current rail (>9 A) and lead clamp never bound unloaded.

### Protocol amendments adopted 2026-07-12

**Two-firmware reality.** Two firmware images now exist for the can-bridge Teensy:

- **stock production v3** — flashed on the assembled robot; 40 Hz knots, 100 Hz telemetry,
  on-change-gated iq.
- **`BENCH_SYSID_BUILD`** (pio env `teensy41_bench_sysid`) — bench-only; 100 Hz knots, 250 Hz
  telemetry, axis-0 DIAGNOSTIC forced every telemetry tick (250 Hz un-gated iq). Zero wire-format
  change; the flag-OFF production binary is **sha256-proven byte-identical**.
  **NEVER flash the bench variant to the assembled robot** (its 100 Hz knots exceed the Jetson
  50 Hz production compute ceiling for the full stack; README + `canbridge_config.h` +
  `platformio.ini` all warn). **After any bench session, re-flash stock production v3 before the
  robot is driven.**

**Mode inventory now.** `bench_leg_sysid.py --mode` = `pos_steps | chirp | ladder | track |
strokes | teleop` (`all` = pos_steps+chirp). Knobs: `--rungs 'pg:vg:vint,…'` (explicit
single-candidate survey, never-abort), `--gains 'pg:vg:vint'` (bringup override so a run's gain
state is unambiguous), `--quiescent-secs` (thermal-onset soak), `--knot-hz {40,100}`,
`--fast-iq` (the 250 Hz iq buzz gate, requires the bench firmware), `--track-gains`,
`--stroke-amps`/`--stroke-vels`, `--teleop-device` (substring/hidraw-path pre-selection).

**Reading rules (how to read a bench run without being fooled).**

1. **Judge real vibration off the pos-to-pos gain DELTA on identical stroke geometry** — the
   commanded content cancels exactly. A velHF that *falls* with gain is the anti-vibration
   signature (a stiffer loop tracking a fast move better); a velHF/iqHF that *grows* with gain at
   cmdHF≈0 is real gain-induced vibration.
2. **Distrust any HF onset on a stroke shorter than ~150 ms** — short strokes carry 10–56× more
   in-band *commanded* energy than long ones and produce false positives; confirm with the gain
   delta before trusting the onset.
3. **Step-fit ζ overstates damping** — ramp-rate-limited steps never excite the 15–19 Hz
   resonance. The **honest damping figure is the chirp peak/DC ratio** (ζ≈0.49–0.52 at the 110
   crossover), not the step fit (0.63–0.78).

### Remaining Stage-1 bench items

- **By-ear spacemouse teleop session** (device selection is fixed — wiggle-probe when several
  identical receivers enumerate).
- **A real S4-class sustained datapoint** — the sustained-stroke entry self-latched at all gains
  from a harness sizing bug until it was fixed + gated; re-run for the genuine back-to-back err/HF
  surface.
- **High-speed envelope up to the operator-authorized 2.5 m/s (~35 rev/s)** on the acceptable-loss
  leg — the binding link is the firmware vel_ff pass-through cap `LEAD_CLAMP_VELFF_LIMIT_RPS =
  3.5 rev/s`; a bench-flag bump of that cap under `BENCH_SYSID_BUILD` is the unlock, not the config
  vel_limit alone.
- **Thermal/positional intermittency probe** — the pos-130 boundary buzzed one sitting and was
  clean the next (both at center 1.5 rev); a controlled repeat with a temperature log and a
  center-position sweep would separate thermal from position effects.

---

## STAGE 2 — Jugglebot transfer (conservative): derate, then confirm

Stage 1 found the answer on hardware we could break. Stage 2 lands it on the real robot
with the *minimum* on-robot exposure to the ring/latch regime. **Do not port the bench
winner verbatim** — derate it first, because part of the plant does not transfer.

### What transfers 1:1, and what does not

**Transfers (same motor + actuator):** the **current loop** (ODrive-firmware kHz loop)
and the **velocity-loop dynamics** — same D6374-150Kv motor, same ODrive Pro, same rotor
inertia `J_rotor = 2.75e-4 kg·m²` (`hardware_config.yaml:55`), same 10 A limit. The
Stribeck friction shape also generalises (the constant load offset does not — bench cable
preload vs platform gravity; see `hardware_config.yaml:160`).

**Does NOT transfer — the *position-loop plant* sees a different load:**

1. **Reflected load inertia.** A bench leg is unloaded; each platform leg additionally
   reflects a share of the ~1.2 kg platform+throw-axis mass (`hardware_config.yaml:43`,
   `platform_mass_kg`). INFERRED, crude vertical-in-phase estimate: reflected platform
   inertia per leg ≈ `(M/6)·(lead/2π)²` ≈ `0.2 kg × (0.0114 m)²` ≈ **2.6e-5 kg·m² ≈
   0.09× J_rotor**. So the *raw inertia bump* bench→Jugglebot is **modest (~≤10 % of
   rotor inertia)** — the shared rotor (+ shared leg-carriage) dominates, and the
   velocity/position-loop plant transfers *well*. (If the leg-mechanism reflected inertia
   is as large as the friction-FF work suggested — "J_eff ~10× rotor",
   `logbook/2026-04-27-friction-feedforward-bench-validation.md` — the platform share is
   <1 % and the transfer is even better.) **Direction matters:** higher load inertia
   *lowers* the velocity-loop bandwidth `ω_v = vel_gain/J`, which *erodes* the position
   loop's phase margin — so the same gains ring **slightly more** on Jugglebot, not less.
   The transfer must therefore *reclaim* that margin.
2. **Inter-leg structural coupling.** A single bench leg has none; on the platform,
   moving one leg reacts through the shared platform back onto the other five. In
   differential (non-in-phase) modes this adds effective inertia/stiffness the bench never
   sees.
3. **Gravity preload + pose-dependent phase margin.** Gravity shifts the integrator
   operating point per pose; MEASURED, the effective phase margin is **pose-dependent even
   at uniform gains** — the 2026-04-19 hold-twitch investigation (revert landed 04-20)
   found a 60× hold-phase asymmetry under NON-uniform per-leg gains (leg 1 at 30/0.24),
   resolved by restoring uniformity, with a ~2× residual pose-dependence remaining even
   at uniform gains (`logbook/2026-04-19-leg1-pose-dependent-hold-twitch.md`; Outcome
   section below). That residual is a non-transferring effect on the same order as the
   raw inertia delta — which is why the Stage-2 verify keeps the extreme-pose HOLD
   battery non-negotiable.

### Derating rule

Because Jugglebot has *more* effective inertia + coupling than the bench (points 1–2),
the bench winner will have *slightly less* phase margin on the robot. To reclaim it,
apply a **one-notch conservative derate** to the bench winner `(pos_gain*, vel_gain*,
vel_int*)` before the first on-robot stroke — **either**:

- **reduce `pos_gain` by ~10–15 %** (lowers the position-loop crossover, restoring
  margin), keeping the 125:1 `vel_int` ratio; **or**
- **raise `vel_gain` by ~15–20 %** (more velocity-loop damping, raising `ω_v/ω_p`),

whichever the Stage-1b step data says costs less tracking lag. The derate is small
*because the inertia delta is small* (point 1) — this is a nudge to buy margin, not a
retune. Then confirm on-robot; if the ring survives the derate, apply the *other* knob one
notch and re-verify (a single A/B, not a fresh sweep).

### Reduced on-robot confirmation (the old 3-point sweep shrinks to a verify pass)

Stage 1 already ran the discriminate-then-damp sweep on the bench. On the real robot you
do **not** repeat it — you *verify* the derated winner. Three checks, in
`session_gain_retune.md` (now its **Phase B**):

> **Concrete Stage-2 spec (2026-07-12, from the Stage-1 results above).** The candidate start
> is **pos_gain 70 / vel_gain 0.35 / vel_int 0.56** — the bench pt70 accuracy knee, taken as-is
> because starting at the *unloaded* knee is itself the derate (the loaded platform reflects
> ~1/6 of platform+payload inertia + gravity + inter-leg coupling, which erodes phase margin, so
> the same gains ring *slightly more* on the robot). `vel_int` is the **ratio-rule 0.56
> (`70/125`), NOT the bench-frozen 0.72** — the frozen value was a bench-only hedge against
> current-saturation windup; on the robot the integrator is always partially wound up holding
> gravity, so 0.72 (2.25× production) invites windup → overshoot/latch. **Re-pin
> `MAX_LEAD_REV = 4.0/pos_gain = 0.057`** (`canbridge_config.h`) so max lead again commands
> exactly vel_limit and the clamp stays meaningful at the stiffer loop.
>
> **Acceptance gate — the loaded S4 replay (the single deciding test).** Re-run the exact S4
> excitation that produced the original limit cycle (large sustained ~2 rev/s vertical spacemouse
> strokes), record a bag, and re-run the `2026-07-10` S4 analysis pipeline. Run it **first at
> production `40/0.20/0.32` under v3** (does the 6 Hz reproduce on v3 at all, now that vel_ff is
> kept? — the unloaded bench says it may already be gone), **then at the candidate**, escalating
> the candidate in stages **40 → 55 → 70** with a per-step bag + analysis, starting at low stroke
> amplitude.
> - **PASS** = no 5.9–6.1 Hz / ~12.3 Hz spectral peak on any leg; reduced tracking lag vs
>   production; no guard latches / E-STOPs; quiescent hold-current ripple bounded; the integrator
>   not saturating on gravity transients; catch-move overshoot < 15 %.
> - **ABORT** on any limit-cycle peak reappearing, any `MAX_DEVIATION` / MPC-staleness latch,
>   audible buzz/whine, or overshoot > 15 %. On abort, apply the other derate knob one notch
>   (`vel_gain` +15–20 %) and re-verify — **not** push the ring regime harder on the real legs.
>
> The three checks below are the per-candidate verify that rides on top of this gate.

1. **Ring-suppression stroke (×1):** apply the derated triple (YAML → codegen → colcon →
   relaunch), arm via `/orchestrator_command activate`, run the single fast stroke
   z 170→250→170 at 156/660/10500. PASS = no braking-current oscillation, max
   `live_deviation` < 0.05 rev, `lead_clamp_mask` stays 0, no ~6 Hz buzz, no
   `MAX_DEVIATION` latch. (This is the *only* time the real legs enter the stutter regime,
   and they now recover via `/recover` if it latches — the derate makes a latch unlikely.)
2. **Extreme-pose HOLD battery (non-negotiable cross-tier guard):** the Level-1 moves 6 &
   7 at (0,−100,200) and (100,100,200) — per-leg hold stdev within 1.5× of the quietest
   leg. A fast-motion winner that reintroduces the pose-dependent hold twitch is **not** a
   winner. The two tiers share one YAML gain vector, so they must be *jointly* satisfied;
   if a single uniform vector cannot satisfy both, escalate to Level 3 (loop shaping) or
   gain-scheduling (Open questions).
3. **If either fails:** apply the other derate knob one notch and re-run (single A/B). If
   neither the ring nor the hold twitch can be jointly cleared, that is the Level-3 /
   gain-scheduling trigger — *not* a reason to push the ring-regime harder on the real
   legs.

### Registered fast-motion gains

| date | stage | pos_gain | vel_gain | vel_int_gain | measured ζ / ring | notes |
|---|---|---|---|---|---|---|
| 2026-07-12 | bench (S1) | 70 | 0.35 | 0.56 | no 6 Hz ring on the unloaded bench in any regime; the 110 rung gave ζ≈0.5 @17.5 Hz and was overturned | accuracy knee (errRMS 10.3→6.9 mm, −33 %); pos 90 the aggressive edge; pos 110+ over the line. See `logbook/2026-07-12-bench-leg-gain-tuning-stage1.md` |
| — | robot (S2) | 70 (start) | 0.35 (start) | 0.56 (start) | TBD | **candidate, not yet verified** — the bench pt70 triple, derated by starting at the unloaded knee; MAX_LEAD re-pinned 0.057; pending the loaded S4 replay gate |

The **bench (S1)** row is the unloaded-leg result; the **robot (S2)** row is a *candidate start*,
**not** a validated production gain — it is verified only when the loaded S4 replay (below) passes.
Until the robot (S2) row is validated, the committed gains remain the Level-1 HOLD tier
(`40 / 0.20 / 0.32`, `hardware_config.yaml:319-321`), which is known to stutter on fast strokes at
raised limits — so **fast-motion S4 ramping stays gated on this tier converging** (see runbook
§ S4b).

### Topology & drive paths (how the bench leg connects, and which path each stage uses)

The bench ODrive **replaces Jugglebot entirely on CAN3**. **One drive path — Path BRIDGE —
now serves every stage** (operator-confirmed 2026-07-11):

> **Historical note (Path DIRECT, retired 2026-07-11):** the `single_leg_test.py` /
> `cogging_bench_test.py` harnesses used to open socketcan directly over a USB-CAN adapter
> for clean instantaneous steps/chirps + instant gain-setting. Direct CAN on the Jetson has
> been **wound down entirely** — the Jetson no longer sees CAN and there is no USB-CAN
> adapter — so that path is unrunnable here and is retained only for the record.
> `bench_leg_sysid.py --path direct` refuses live runs (accepting only `--dry-run` for the
> historical plan view). Everything below runs on Path BRIDGE.

**Path BRIDGE — bench ODrive on CAN3 through the can-bridge Teensy, ALL stages (1a, 1b, 1c,
2).** This reproduces Jugglebot's exact 40 Hz interp + lead-clamp drive **and** exposes the
v3 live-deviation telemetry. Topology facts:

- **Node-id:** the firmware treats CAN3 node ids **0..5 as legs** and 6 as the hand
  (`canbridge_config.h:84-85`). Present the bench ODrive as **node 0** — all historical
  bench work used node 0, and the per-index config arrays (`LEG_POS_GAINS[i]`,
  `STROKE_MIN/MAX_REV[i]`, `mm_to_rev[i]`, `odrive_expected_versions.axis_i`) are keyed by
  node index, so node 0 inherits leg-0's config. **Operator-confirmed 2026-07-11: the
  bench ODrive is at node 0, and its firmware (0.6.11 per the ODrive GUI) matches the
  Jugglebot ODrive Pros — the cold-start version sweep should pass.**
- **1-of-6 presence gating:** `leg_present(i) == axes[i].heartbeat_seen` (latched-once,
  `logbook/2026-06-24-...`, U1). Legs 1–5 absent ⇒ the interp setpoint TX, the stow
  descent, and the `MAX_DEVIATION` loop **skip them**; the deferred-stow reconnect
  predicate scopes to present legs (`all_present_legs_fresh`). So the bridge runs
  correctly on a subset-populated CAN3 (this was the Phase-11 U1 fix). The bus-partner
  presence gate (`BUS_PARTNER_STALENESS_US = 5 s`, `canbridge_config.h:212`) is per-bus —
  the bench ODrive's heartbeats keep CAN3 "present" so RPCs/sends are not refused.
- **Gains apply via `_run_configure`** (`teensy_bridge_node.py:2626`), which calls the
  `SET_POS_GAIN`/`SET_VEL_GAINS` RPCs from generated config (`teensy_bridge_node.py:2647-
  2650`); these forward a CAN frame to the ODrive and are **RAM-only, session-only**
  (`rpc.cpp:167-174`; no `save_configuration`). There is **no runtime ROS service that
  sets leg gains live** (only `set_hand_gains`, axis 6). So the production gain-apply loop
  is YAML → `generate_config` → `colcon build` → relaunch, applied at the next
  `_run_configure` (home/`/configure`/post-activate).
  **This is the Stage-1 gain-apply path:** the same RPCs are reachable Python-side without a
  rebuild — `controller/teensy_link/rpc.py:RpcClient.call` + `rpc_args.encode_set_pos_gain
  / encode_set_vel_gains` (mirrored by `teensy_bridge_node.teensy_set_pos_gain`, `:2041`) —
  and `bench_leg_sysid.py --path bridge` uses exactly these to set the bench leg's gains over
  CAN3 instantly, **re-applied per ladder point** (gated on `jugglebot_commands_allowed`,
  i.e. a fresh CAN3 partner heartbeat). Caveat: a subsequent orchestrator
  `configure`/`activate` re-applies the *YAML* gains, so during a Stage-1 session drive the
  bench-leg stimuli with `bench_leg_sysid.py` (or the synthetic β-knot source), **not** the
  orchestrator.
- **Stroke safety (CRITICAL):** the firmware `STROKE_MAX_REV[0]=3.90` is the *production*
  clamp and does **NOT** protect the shorter bench leg (`logbook/2026-06-24-...`, U3-iv).
  On Path BRIDGE the ≤3.0-rev bench cap **must live in the driver**, not the firmware.
  - **Stage 1a/1b (system-ID + ladder): `tests/hardware/bench_leg_sysid.py --path bridge`
    OWNS the 3.0-rev cap** (`--stroke-cap` default `3.0`, raising it needs `--confirm-stroke`
    and never exceeds 3.30). Every knot is clamped to `[margin, 3.0−margin]` before it
    leaves the harness, every step/chirp is stroke- **and** kinematic-capped, and each knot
    ramp stays under the 0.10 rev lead clamp. Run `--dry-run` to see the full bounded plan.
  - **Stage 1c (ring reproduction at fixed `40/0.20/0.32`): the synthetic β-knot driver
    `tests/hardware/teensy_setpoint_bench.py`** (`--axis 0 --center … --close-loop`).
    **Beware: its OWN built-in stroke clamp is the production `STROKE_MAX_REV[0]=3.900413`
    (`teensy_setpoint_bench.py:85`), hardcoded, no CLI override — it provides NO 3.0-rev
    backstop.** Until it grows one, keep it safe by choosing `--center`/`--amplitude` so
    `center + amplitude ≤ 3.0` with margin (prefer it over the full trajectory stack, whose
    platform-pose IK could extend leg 0 past the bench end-stop). `--close-loop` applies the
    `hardware_config` gains before arming.
- **Observability:** the v3 `/link_status` KeyValue fields at 10 Hz — `live_deviation`
  (per-leg u0−encoder, rev), `lead_clamp_mask`, and the frozen latch snapshot
  (`max_dev_leg`/`_value`/`_u0`/`_enc`) — plus `/diagnose` on the rosbag (braking-current
  trace, ring FFT/Lomb-Scargle offline). This telemetry exists **only on Path BRIDGE**; it
  is what makes Stage 1c and the Stage-2 verify measurable.

**Setup for the CAN3 swap (Path BRIDGE):** disconnect all six platform legs + hand + the
Platform Teensy from CAN3; connect **only** the bench ODrive (node 0) to CAN3. Ensure
**120 Ω termination at both physical ends** of the CAN3 stub (operator-handled and
confirmed sorted, 2026-07-11 — cabling/termination is the operator's domain).
Power the bench ODrive from the **48 V PSU
with the brake resistor attached**; the can-bridge Teensy is already powered from Jetson
5 V. With the Platform Teensy gone there is **no `is_homed` cold-start persistence, no
tilt/levelling, no `STATE_READ`** — none are needed for gain tuning; home the bench leg
fresh each session via the firmware `HOME(0)` RPC (`bench_leg_sysid.py --home`, or the
standalone `teensy_home_bench.py`).

**Recommendation:** run **every stage (1a, 1b, 1c, 2) on Path BRIDGE** — it is the only path
on this Jetson (direct CAN wound down 2026-07-11). System-ID therefore runs on interp-shaped
stimuli (workable, less clean — the 500 Hz interp is well above the plant, so a fast knot
ramp approximates a step): steps become 25 ms-quantised knot ramps under the lead clamp, the
velocity-loop step response is **inferred from position-step overshoot** rather than measured
directly, and the chirp top frequency is bounded to ≈8 Hz by the telemetry + 40 Hz knot rate
(enough for the 6 Hz pos-loop question, not the 30–80 Hz structural band). Run
`bench_leg_sysid.py --path bridge --dry-run` first to review the full bounded plan; the v3
`/link_status` telemetry (`live_deviation`, `lead_clamp_mask`) is the measurable that makes
Stage 1c and the Stage-2 verify honest.

## Level 3 — System ID + loop shaping

Use when: Level 2 hits a ceiling, or when you want explainable reproducible
gain selection as part of a spec. Cost: ~1 day per leg. Defensibility:
high — you're deriving gains from a measured plant model with specified
margins.

### Procedure

1. **Command a multisine chirp** into `input_pos` from 0.5 Hz to 80 Hz. 80 Hz
   comfortably exceeds the 40 Hz MPC rate and captures structural resonance.
   The chirp should have no DC component and amplitude small enough to stay
   linear (1–2 mm peak-to-peak is a reasonable starting point).
2. **Capture `pos_estimate` at 500 Hz and motor current at the fastest
   available rate.** Both are needed for the full plant model.
3. **Fit a second-order plant model per leg:**

   ```
   G(s) = ω_n² / (s² + 2ζω_n·s + ω_n²)
   ```

   Expected characteristics: ω_n in the 20–80 Hz range (mechanical
   resonance), ζ in the 0.1–0.4 range (moderately underdamped). Large
   inter-leg variance in ω_n is a signal of mechanical unit-to-unit
   variation and explains Level 1 outlier behaviour.
4. **Synthesise a PID** with:
   - Phase margin ≥ 60°
   - Gain margin ≥ 6 dB
   - Crossover frequency ≈ 10× the mechanical resonance of the softest leg
     (not the average — protect the weakest member)
5. **Verify on an independent trajectory** (not the chirp): command a real
   juggling-style move and measure tracking RMS, peak tracking error,
   disturbance rejection (apply a small mechanical tap during hold and
   measure settling).
6. **Iterate** until margins hold across all six legs simultaneously.

### When Level 3 becomes necessary

- You're making a claim in a paper, spec, or customer-facing document about
  dynamic performance
- You want to predict how the platform will behave under disturbances
  that weren't in your test battery (e.g., ball impact during a catch)
- Level 2 tuning leaves you with gains that pass the step-response criterion
  but fail in practice on full trajectories — this usually means the plant
  is higher-order than second-order and Level 2's targets were too loose
- You want to add a feedforward path based on the fitted plant inverse

## Recommendation for the current hardware-bringup stage

Stay at Level 1 until at least one of:

- Level 1 converges (per-leg gains committed, hold-phase fighting measured
  down to within 1.5× of μ_best across all six legs)
- Level 1 hits a ceiling (can't get leg N below some floor without hurting
  tracking; floor is above μ_best by more than 1.5×)
- A juggling-timing claim starts depending on gain-level behaviour

If Level 1 converges and stays converged across a week of test sessions,
we probably never need Level 2 for the current juggling-accuracy budget.
If Level 1 hits a ceiling, escalate to Level 2 for the leg in question
only — don't rebuild Level 2 across all six legs unless needed.

## Outcome for the hardware-bringup stage (2026-04-20)

**Level 1 converged at uniform 40/0.20/0.32 across all six legs** —
matching the original ODrive-flash baseline. Per-leg reductions turned
out to be a false optimum.

### Timeline

- **Iteration 1 (2026-04-18):** legs 1 and 3 identified as worst hold-phase
  outliers at neutral-pose hold. Leg 1 dropped to 20/0.16 (aggressive halving).
  Improved neutral-pose stdev but under-damped at low load.
- **Iteration 2 (2026-04-18/19):** leg 1 compromised to 30/0.24, leg 4 added
  at 30/0.24 (2nd-worst at z=220). Passed all five near-centre moves in the
  test battery. Declared converged.
- **Iteration 3 reversal (2026-04-20):** session `mpc_20260419_135251.csv`
  exposed a **60× hold-phase asymmetry** on leg 1 at pose (0,−100,200), a
  pose not covered by Iteration 1/2 testing. `act_std` 437 µm vs 7 µm on the
  quietest leg. Kinematic + static-force analysis (see
  `logbook/2026-04-19-leg1-pose-dependent-hold-twitch.md`) showed legs 1 and
  2 were in mathematically identical roles at that pose with near-equal
  load — meaning the 60× difference could only be a software asymmetry
  introduced by the per-leg gain vector, not a hardware difference. Revert
  of leg 1 → 40/0.32 produced a **16× stdev drop** (437 → 27.5 µm) at the
  same pose, validated in session `mpc_20260420_160401.csv`. Leg 4, held at
  30/0.24 as an in-experiment control, reproduced the same pose-dependent
  signature (50 µm stdev, 10× asymmetry) — confirming mechanism.
- **2026-04-20 leg-4 revert:** leg 4 → 40/0.32. Validated in session
  `mpc_20260420_182945.csv`: leg 4 stdev 50 → 36 µm at (0,−100,200);
  overall asymmetry ratio 60× → 8.5× relative to the original Iteration-2
  configuration. All six legs now uniform 40/0.20/0.32.

### Lessons

1. **Tuning against hold-phase stdev at a single pose is insufficient.**
   The hold-phase metric is stationary *per pose* but the load distribution
   (and thus effective phase margin) varies across the workspace via CoM
   offset and Jacobian geometry. A gain value that is "just soft enough" at
   neutral pose can be "over the edge" at an extreme pose.
2. **When a leg's observed noise is high, the first move is to check
   whether the YAML gives that leg a different gain from its peers.** We
   spent substantial effort looking for hardware explanations (leg-specific
   stiction, backlash, mechanical asymmetry) for a signal that turned out
   to be entirely produced by our own YAML.
3. **In-experiment controls are cheap and high-value.** Leg 4 was
   deliberately left at 30/0.24 during the leg-1 revert test. That control
   survived the A/B and reproduced the signature at a different leg,
   upgrading the result from "plausible fix" to "mechanism confirmed."
4. **"Per-leg" as a tool is right; "per-leg by trial-and-error at one
   pose" is wrong.** The per-leg gain plumbing remains valuable for Level-2
   or Level-3 work that actually measures plant asymmetry. It should not
   be driven by single-pose stdev differences alone.

### Revised Level-1 entry criteria (for any future round)

Do not touch any leg's gain unless all five of the following are true for
that leg, measured across the expanded test battery (including moves 6 and
7 at extreme poses):

1. Hold-phase stdev > 1.5× median across the test battery, in aggregate,
   not at one pose.
2. No other leg shows the same stdev at the same pose (rules out
   pose-level effects masquerading as leg-level).
3. The leg's observed stdev is above the mechanical-bandwidth floor of ~5
   µm (below that, you are tuning against measurement noise).
4. The motion-onset dead-time investigation
   (`plans/archived/2026-05-08 motion-onset-deadtime-investigation.md`) is either
   resolved or has a known contribution to the leg's observed stdev that
   you can subtract.
5. You have a written hypothesis about **why** this leg is different that
   goes beyond "its stdev is higher." Without a mechanism, the revert
   risk is too high.

If any of these fail, do not open a per-leg gain change. Leave the YAML
uniform.

## Open questions for later

- Does leg 1 + leg 3 being the worst pair correlate with a specific
  manufacturing batch or assembly position in the hex? If so, we can
  predict outliers from the physical leg serial numbers rather than
  discovering them at each recommission. (**Updated 2026-04-20:** less
  compelling now — at uniform 40/0.32, no leg stands out by more than
  ~2× at the problem pose. The signal this question was chasing was
  mostly Iteration-2's own asymmetry.)
- Would a simple online adaptation (e.g., auto-tune the integrator from
  hold-phase stdev during the first 5 s at Active) remove the need to
  commit gain numbers to YAML at all? Explore in Phase 5+ once we have
  a baseline Level 1 result.
- How do the gains need to change when catching a ball (brief high
  transient load)? Level 3 would let us predict this; Level 1 cannot.
- **New 2026-04-20:** Is gain-scheduling across the workspace worth the
  complexity? The (0,−100,200) data suggests phase margin is
  pose-dependent even at uniform gains (the residual 36 µm on leg 4 and
  58 µm on leg 1 at that pose, vs sub-15 µm on the other legs, is a hint).
  Before pursuing: run the expanded 7-move battery at uniform gains and
  see whether the residual asymmetry is material relative to the motion-
  onset dead-time (currently ~100–200 ms, orders of magnitude larger in
  tracking-error terms).
