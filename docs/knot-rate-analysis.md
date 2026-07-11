# The knot-rate sweet spot for Jugglebot's trajectory stream

**Question (operator, 2026-07-11):** *"I wasn't questioning 40 Hz as the stutter
mechanism — the 2026-07-10 forensics already exonerated it. I'm asking whether 40 Hz
is the ideal rate for our system as a whole. Too fast or too slow would both be
detrimental — there must be a sweet spot. How would we estimate the best knot-rate?"*

**Scope:** READ-ONLY analysis. Builds the quantitative trade-off model for *this*
system, grounded in the code and the measured 2026-07-10 forensics numbers. Produces
(a) a sweet-spot criterion as a function of system parameters, (b) the number for
today and for the post-retune system, (c) the measurements that would sharpen it, and
(d) an honest "is it worth changing" verdict with the cost.

> **Terminology.** "Knot rate" = the rate at which `trajectory_node`'s emitter thread
> publishes `SETPOINT` knots on :5557 (one knot per `SEGMENT_T_S`). Today that is a
> **hard 40 Hz / 25 ms**, pinned three places that must stay coherent: the Jetson
> emitter period (`trajectory_node._emitter_loop` derives it from
> `hw.JB_TRAJ_KNOT_DT_S`, `trajectory_node.py:429-432`), the config
> (`config/hardware_config.yaml:372` `knot_dt_s: 0.025`; `config/generated/hardware_config.py:149`),
> and the firmware Hermite segment time (`canbridge_config.h:116` `SEGMENT_T_S = 0.025f`,
> consumed at `leg_interp.cpp:36`). It is **not** the CAN setpoint rate — see §1.

---

## 0. Bottom line

**40 Hz is well-chosen and sits in a narrow feasible band. Leave it.**

> **Update 2026-07-11 — the compute ceiling is now MEASURED, not inferred.** The
> emit-gap-vs-rate sweep (§10) drove the production `KnotEmitter.frame` +
> `TargetFollower.follow` pipeline with the recorded S3 stick stream at
> {25,30,40,50,60,80} Hz on this Jetson. The would-gap fraction first exceeds 1 %
> **at 50 Hz** (default session tier); 40 Hz is the highest rate that stays clean
> (would-gap 0.02 %, **+6.7 ms p99 compute margin**). This *confirms* the inferred
> 45-50 Hz ceiling — the measured value lands right at its top edge — and the
> verdict below is unchanged.

- The band is bracketed by a **catch/timed-target timing floor (~20-40 Hz)** below and a
  **Jetson chase-follower compute ceiling — MEASURED at 50 Hz (§10)** above. 40 Hz sits
  high-center, with a **measured 6.7 ms p99** compute margin in the SpaceMouse/CATCH regime.
- Every *downward* pressure (interpolation fidelity, C1 join step, teleop latency) is
  already deep in diminishing returns at 40 Hz — 22 µm of interp error, 6 mm/s of join
  step, 25-50 ms of latency, all invisible or comfortably inside spec. The planned C2
  firmware interpolant *zeroes* the join step, removing the one downward pressure with
  any real magnitude.
- The single *binding* upward pressure is Jetson per-tick compute in the chase-follower
  regime (frame IK ×3 + chase replan ≈ 18-20 ms p99 against the 25 ms period). Raising
  the rate crosses this ceiling and reintroduces emit gaps — the exact S3 failure class
  that publish-first + the chase clamp just fixed.
- Independent sanity check: 40 Hz = **6.25× the measured 6.4 Hz servo bandwidth**,
  squarely inside the industrial 4-10× setpoint-streaming band. The retune raises the
  servo bandwidth but does **not** move the sweet spot up, because the Jetson compute
  ceiling (not the servo) is what caps the useful rate.

**Verdict:** no change. If a future retune session shows the chase-follower regime
gapping, the cheaper lever is to reduce the chase replan cost, not to change the rate.
The only rate change worth considering is a *downward* nudge to ~30-33 Hz to buy compute
headroom — and only if measured gapping justifies the firmware/xref re-validation cost.

---

## 1. What drops out immediately: the 500 Hz decoupling

The knot rate is **not** the rate at which the legs are commanded. The Teensy interp ISR
(`leg_interp.cpp:210` `interp_isr`, started at `leg_interp.cpp:467` on a 500 Hz
`IntervalTimer`) reconstructs a smooth command *between* knots via cubic Hermite and
**transmits a fresh CAN3 setpoint every 500 Hz tick regardless of when the next knot
arrives** (`leg_interp.cpp:390-406`). Consequences, all MEASURED / code-confirmed:

- **CAN3 load is knot-rate-independent.** 500 Hz × 6 legs ≈ 39 % of 1 Mbps
  (logbook `2026-07-10-...:60-63`), fixed at any knot rate. **CAN3 drops out of the
  calculus entirely.**
- **The Teensy interp ISR is knot-rate-independent.** Period 2000 µs
  (`canbridge_config.h:69-70`, `INTERP_RATE_HZ = 500`); per tick it does 6 legs of
  Hermite float math + lead/stroke clamp + 6 FlexCAN mailbox writes. On a 600 MHz
  Cortex-M7 with single-cycle FPU the float work is ~1-2 µs and the mailbox writes
  dominate at maybe ~10-20 µs total — **~1 % of the 2000 µs budget** (INFERRED; the live
  telemetry to confirm is `interp_deadline_misses()` / `interp_max_jitter_us()`,
  `leg_interp.cpp:476-478`, expected near-zero). **The Teensy is nowhere near binding.**

So the *only* firmware coupling to the knot rate is **coherence of `SEGMENT_T_S`** (and
the extrapolation windows `MAX_EXTRAP_DT_S = 50 ms`, `EXTRAP_DECAY_DT_S = 60 ms`,
`canbridge_config.h:117-118`) with the emitter period. Changing the rate means changing
those and re-validating — that is a *cost*, not a runtime constraint (see §9).

The rest of the model is therefore a **Jetson-side + servo-side** problem, not a bus or
firmware-throughput problem.

---

## 2. Downward pressures (want a **higher** rate / shorter T)

All four favor shrinking `T`. The table gives each pressure's value at today's
T = 25 ms, its scaling in T, and its verdict. `T` is the knot period; rate = 1/T.

| # | Pressure | Value at T = 25 ms | Scaling | Binding? |
|---|---|---|---|---|
| D1 | Interp reconstruction error (Hermite vs true quintic) | **22 µm** (MEASURED) | ∝ T³ | No — invisible at any plausible rate |
| D2 | C1 velocity-FF join step | **6.1 mm/s** = 4 % of peak (MEASURED) | ∝ T | No — small, and **C2 zeroes it** |
| D3 | Teleop input→wire latency | **1-2 knots = 25-50 ms** | ∝ T | No — below perception |
| D4 | Catch/timed arrival quantization | **±T/2 = ±12.5 ms** vs ±25 ms spec | ∝ T | **Sets the floor** |

### D1 — Interpolation fidelity ∝ T³, anchored at 22 µm

The forensics measured the Teensy cubic-Hermite reconstruction tracking the true quintic
to **22 µm** at 40 Hz (logbook `2026-07-10-...:57-60`). The T³ scaling comes from the
*source* of the error: the Hermite endpoint slope at u1 is a **finite difference**
`v1 = (u2 − u1)/T` (`leg_interp.cpp:313`), not the true velocity. A cubic Hermite with
*exact* endpoint slopes would have O(T⁴) error; the finite-difference slope injects an
O(T²) slope error that propagates to an O(T³) position error dominated by the segment
jerk. So:

```
e_interp(T) ≈ 22 µm · (T / 25 ms)³
```

- At 80 Hz (T = 12.5 ms): **~2.8 µm**.  At 20 Hz (T = 50 ms): **~176 µm** = 0.18 mm.

Even the 20 Hz value is an order of magnitude under the ±3 mm pose spec. **D1 is a
non-constraint across the whole [20, 80] Hz range** — raising the rate to chase interp
fidelity buys nothing you can measure at the platform. (HYPOTHESIZED exponent: T³ is the
model; the 22 µm anchor is MEASURED. If a rate change is ever contemplated, confirm the
exponent with the emit-gap/fidelity sweep in §8a — but the conclusion is robust to
anything between T² and T⁴.)

### D2 — C1 join step ∝ T, and C2 deletes it

At each knot boundary the Hermite's end slope (finite-difference `v1`) disagrees with the
next segment's start slope (true twist velocity `v0`), producing a velocity-feedforward
discontinuity of **6.1 mm/s** (MEASURED, "matching the analytic 0.5·T·a_peak",
logbook `2026-07-10-...:58-60`). This is linear in T: at 20 Hz it doubles to ~12 mm/s, at
80 Hz it halves to ~3 mm/s. At 4 % of peak velocity it is already minor.

**The calculus-changer:** the deferred **C2 / quintic firmware interpolant** (make the
firmware match acceleration too, or carry an explicit `v1`, making the join C1-exact)
drives this step to **exactly 0** — noted as "worthwhile polish only" in the forensics
(logbook `2026-07-10-...:63`). Once C2 lands, D2 vanishes *at every rate*, so it can never
be a reason to raise the knot rate. This matters for the verdict: the largest-magnitude
downward pressure is about to be removed by a change that is orthogonal to the rate.

### D3 — Teleop latency = 1-2 knots

Publish-first ordering (`trajectory_node._emit_once`, `trajectory_node.py:456-468`;
follower-cadence `§4.1`) samples and publishes each knot from the plan installed on the
*prior* tick, then replans for the *next* tick. Net operator-input→wire latency is
**1 knot** structural + up to **1 knot** for the replan to land = **25-50 ms** at 40 Hz,
scaling linearly with T. The plan accepts this explicitly ("imperceptible for teleop",
follower-cadence `§4.1`, `§7 Q2`) and the S3 re-fly confirmed it ("worked perfectly",
follower-cadence RESOLUTION). Human motor perception of teleop lag is ~50-100 ms, so 40 Hz
is already under threshold. Halving to 80 Hz (12.5-25 ms) is imperceptibly better; halving
the rate to 20 Hz (50-100 ms) starts to be felt. **D3 is a soft floor around 20-25 Hz.**

### D4 — Catch/timed arrival quantization — this is the real floor

The emitter samples the plan only at knot instants, so a timed/catch target's arrival can
only be *expressed* on the wire to within **±T/2** of its true arrival time. The S5 spec
is **±25 ms** end-to-end (`mvp_bench_runbook.md:390-402`; `mvp-trajectory-bringup.md:875,
880`). The plan's own framing: "knot quantisation ≤ one 25 ms knot"
(`mvp-trajectory-bringup.md:875`). At 40 Hz, quantization consumes **±12.5 ms** — *half*
the budget — leaving ±12.5 ms for emitter jitter + mocap latency + servo lag. That is the
**tightest downward constraint and it sets the floor**:

```
T/2 + (other timing error) ≤ 25 ms   ⇒   T ≤ 2·(25 ms − ε_other)
```

- With ε_other ≈ 12 ms (mocap + servo lag), **T ≤ 26 ms ⇒ rate ≥ ~38 Hz** — i.e. 40 Hz
  is essentially the *minimum* rate that meets the catch spec with any margin.
- If you insist only on ±T/2 ≤ 25 ms alone (no other budget), the hard floor is
  **T ≤ 50 ms ⇒ rate ≥ 20 Hz**.

So the catch spec is why you cannot drop the rate very far. It, not fidelity, is the
downward-binding constraint.

---

## 3. Upward pressures (want a **lower** rate / longer T)

| # | Pressure | Value at T = 25 ms | Scaling | Binding? |
|---|---|---|---|---|
| U1 | Jetson per-tick compute (chase-follower regime) | **~18-20 ms p99** vs 25 ms period | cost grows, period shrinks | **Sets the ceiling** |
| U2 | UDP / SetpointPump per-knot overhead | sub-ms | ∝ rate | No |
| U3 | Servo closed-loop bandwidth (command above ~5-10× BW is filtered) | 40 Hz = 6.25× the 6.4 Hz BW | fixed by servo, not rate | No today; loosens after retune |
| U4 | Firmware SEG_T coherence + interp ISR | ISR ~1 % of budget | ISR rate-independent | No (a *change cost*, not a limit) |

### U1 — The Jetson chase-follower compute ceiling (the binding constraint)

The emitter loop is single-threaded: each tick it runs `_emit_once` (build frame →
publish → replan), then sleeps to the next absolute deadline
(`trajectory_node.py:435-449`). Publish-first protects the *latency* of this tick's knot
from this tick's replan, but the **throughput** still requires the total per-tick cost to
fit inside the period `T`, or the deadline slips and an emit gap appears. Per-tick cost
depends on the mode:

- **Frame build (pre-publish, every mode):** `KnotEmitter.frame` runs the IK chain
  **3×** — at τ, τ+T, τ+2T (`emitter.py:74-76`, `_ik` at `emitter.py:53-59`, each call
  does `pose_to_leg_lengths` + `compute_jacobian`). INFERRED cost **~3-5 ms**: the
  analytic gate profiled at ~377 ms for 200 samples with ~90 % in the per-sample Jacobian
  chain (`mvp-trajectory-bringup.md:710, 738-739`) ⇒ ~1.7 ms per scalar IK+Jacobian ⇒
  ~5 ms for 3. (Cheap to replace with a measured number — §8c.)
- **Chase replan (post-publish, SpaceMouse/CATCH only):** `follow()` p99 **14.9 ms**
  measured on the S3 replay (follower-cadence RESOLUTION `§Validation`;
  `mvp-trajectory-bringup.md:493`); runbook budget "should stay ≲ 20 ms"
  (`mvp_bench_runbook.md:348`). This is the fast `validate_follow` gate (~1.5-4 ms accept,
  `mvp-trajectory-bringup.md:742`) plus the per-sample α-interval chase math over an
  ~85-knot horizon (`chase.py` header: "~85 knots ≈ 9 ms at 2.0 s").
- **Waypoint / HOLD mode:** frame build only, no continuous replan — `follow_block` was
  **0** during the S4 TRAJECTORY session (logbook `2026-07-10-...:65`). ~3-5 ms/tick,
  huge headroom.

So per-tick cost, worst regime (SpaceMouse/CATCH):

```
C(T) ≈ frame_IK(~4 ms, fixed)  +  chase_replan(~15 ms p99 at T=25 ms)  ≈ 18-20 ms p99
```

**Gapping condition: C(T) > T.** At T = 25 ms, C ≈ 19 ms < 25 → no gap, ~6 ms p99 margin.
This matches the MEASURED history:

- **Old inline-replan emitter (S3):** the 6-iteration reject path cost p50 41 / p99 58 ms
  (follower-cadence `§Corrections`) — *longer than the 25 ms period*, so every infeasible
  tick blew the budget → emit gaps climbed **25 → 62 → 78 → 146 ms** and the robot latched
  (follower-cadence `§1.4`).
- **After publish-first + chase clamp (S4 / S3 re-fly):** emit gaps "flat at 25 ms"
  (logbook `2026-07-10-...:60-64`); hold-mode Phase-1/2 session-max gaps 42.27 ms and
  56.60 ms (`mvp-trajectory-bringup.md:577, 639`) — well inside the 250 ms staleness
  window but already showing the non-RT Jetson's jitter tail.

**Where does raising the rate start gapping?** Two effects compound: the period `T`
shrinks *and* the chase cost grows (validate_follow samples at the knot rate, so a
fixed-duration horizon has more knots — ~0.1 ms/knot from the chase.py figure). Solving
`C(T) = T` with frame_IK fixed and chase ≈ 15 ms + (rate-growth):

- If chase cost were fixed at 19 ms → crossover T = 19 ms → **~52 Hz**.
- With the cost growing as the horizon gains knots, crossover pulls in to **~45-50 Hz**.

**So the publish-first pipeline starts gapping in the chase-follower regime around
45-50 Hz.** Today's 40 Hz is ~20 % below that — a *thin* p99 margin (a GC pause or a fat
replan tail can still gap occasionally even at 40 Hz; that tail is exactly why the old
follower died). This is the ceiling. It is a **Jetson property**, independent of the
servo or the firmware.

> **MEASURED (2026-07-11, §10) — the inferred 45-50 Hz crossover is confirmed at 50 Hz.**
> Both compounding effects are directly observed on this Jetson: frame IK is flat at
> **p50 ~2.5 ms / p99 ~3.0 ms** (rate-independent, *below* the 3-5 ms inferred here), and
> `follow()` p99 grows monotonically with rate — 12.5 → 13.6 → 15.4 → 20.3 → 23.1 →
> 27.5 ms at 25/30/40/50/60/80 Hz (default tier) — exactly the "fixed-duration horizon
> gains knots as kdt shrinks" mechanism. The total (frame+follow) p99 crosses the period
> between 40 Hz (18.3 ms < 25 ms, +6.7 ms margin) and 50 Hz (23.2 ms > 20 ms, −3.2 ms),
> and the would-gap fraction first exceeds 1 % at **50 Hz**. The ~19 ms p99 estimate used
> in `C(T)` above is close: measured total p99 at 40 Hz is 18.3 ms.

### U2 — UDP / pump overhead

Each knot is one `SetpointPump` build (step-bound check + struct pack) + one UDP frame
(~73 B). At 40 Hz that is 40 frames/s, sub-millisecond of the per-tick budget, folded
into U1's frame cost. Scales ∝ rate but negligible until well past the U1 ceiling.
**Non-binding.**

### U3 — Servo bandwidth: the "commands above ~5-10× BW are filtered anyway" pressure

The leg position loop is a low-pass plant. Its closed-loop bandwidth today is
`pos_gain/(2π) = 40/6.283 =` **6.37 Hz** (MEASURED coincidence with the ~6 Hz stutter
limit cycle, logbook `2026-07-10-...:48-49`; `session_gain_retune.md:28-29`; the S4b sweep
predicts ring ≈ 4.0 / 6.4 / 8.8 Hz at pos_gain 25 / 40 / 55,
`leg-gain-tuning-methodology.md:256`). Command spectral content **above** the servo
bandwidth is attenuated by the plant — streaming knots faster than the servo can act on
injects detail the legs cannot follow, and (worse) any spectral energy the interpolation
puts *near* the 6.4 Hz bandwidth is exactly what fed the limit cycle. So the servo
bandwidth sets a *soft* upper-usefulness bound on the command rate:

- **Today:** 40 Hz / 6.4 Hz = **6.25×**. Right in the industrial 4-10× band (see §7).
- **Post-retune:** if the retune lifts the bandwidth to ~10-15 Hz (pos_gain toward 55+
  gives ~8.8 Hz by the pos_gain/2π proxy; a real closed-loop −3 dB fit could reach 10-15 Hz
  — HYPOTHESIZED, to be measured, §8b), then 40 Hz becomes only 2.7-4× the bandwidth,
  brushing the *low* edge of the industrial band. That is the one effect that *could*
  argue for a modest rate increase — but it runs straight into the U1 compute ceiling and
  is not supported by any downward pressure needing it (see §6).

U3 is not binding today. It is the reason the sweet spot doesn't want to be *much* higher
than 40 Hz even if compute were free: past ~10× bandwidth (~64 Hz today) the extra knots
are filtered away.

### U4 — Firmware SEG_T + interp ISR

Covered in §1: the ISR is ~1 % utilized and rate-independent. The only coupling is
`SEGMENT_T_S` coherence, which is a *re-validation cost* (§9), not a runtime limit.

---

## 4. The sweet-spot criterion

Collecting §2 and §3, the feasible rate lives in a band, and within it the optimum is the
point where the marginal benefit of shrinking T (fidelity/latency/timing) equals the
marginal compute cost of shrinking T.

**Floor (max of the downward constraints):**

```
rate ≥ max(
  catch-timing:   1 / (2·(spec − ε_other))     ≈ 38 Hz   (spec 25 ms, ε_other ~12 ms)   ← binding
  fidelity:       (22µm / tol)^(1/3) / 25ms     ≈ 24 Hz   (tol 100 µm)
  teleop latency: 1 / lat_max                    ≈ 20 Hz   (lat_max ~50 ms)
)
⇒ floor ≈ 38-40 Hz  (or ~20 Hz if you spend the entire catch budget on quantization)
```

**Ceiling (min of the upward constraints):**

```
rate ≤ min(
  Jetson compute:  1 / (frame_IK + chase_p99)/margin   ≈ 45-50 Hz (chase regime)   ← binding
  servo usefulness: ~10× servo_BW                        ≈ 64 Hz today / 100-150 Hz post-retune
)
⇒ ceiling ≈ 45-50 Hz  (chase-follower regime); ~100 Hz in waypoint/hold mode
```

**Criterion, stated plainly:**

> Pick the **lowest** knot rate that satisfies the catch/timed-target timing spec (D4)
> with margin, because every downward pressure other than D4 is already deep in
> diminishing returns, while the upward compute cost (U1) is steeply binding just above
> 40 Hz. Do **not** raise the rate to chase fidelity or latency — those are already
> invisible; you would only spend compute margin and cross the gap ceiling. The optimum
> is the floor of the feasible band, set by D4, with a compute-headroom check against U1.

The two binding constraints — catch-timing floor (D4, ~38-40 Hz) and chase-compute
ceiling (U1, ~45-50 Hz) — **nearly touch**. The feasible band is narrow (roughly 40-50 Hz
with full catch margin, widening to 20-50 Hz if you relax catch margin) and 40 Hz sits at
its efficient edge: the lowest rate that keeps full catch-timing margin.

---

## 5. The number, today and post-retune

**Today: ~40 Hz is the sweet spot (keep it).**
- Floor (catch spec, full margin): ~38-40 Hz.
- Ceiling (chase-follower compute): **MEASURED 50 Hz** (§10), **+6.7 ms p99 headroom at
  40 Hz** (measured total per-tick p99 18.3 ms vs the 25 ms period).
- Servo cross-check: 6.25× the 6.4 Hz bandwidth — inside the industrial band.
- A *defensible downward nudge* to **~30-33 Hz** exists: it buys chase-follower compute
  headroom (period 30-33 ms vs ~19 ms cost → comfortable), keeps fidelity fine
  (T³: ~40-50 µm), keeps latency fine (30-66 ms), and keeps catch quantization inside spec
  (±15-16 ms < ±25 ms) — at the cost of a larger C1 join step (~8 mm/s, until C2 zeroes it)
  and thinner catch margin. This is the *only* rate move with a positive first-order case,
  and it is small.

**Post-retune: the sweet spot stays ~40-50 Hz — the retune does not move it up.**
- The retune raises the *servo* bandwidth (U3) toward ~10-15 Hz, which *loosens* the
  servo-usefulness ceiling to ~100-150 Hz. But the **binding ceiling is Jetson compute
  (U1), which the retune does not touch** — it stays ~45-50 Hz in the chase regime.
- If the servo genuinely reaches the top of the retune range (~15 Hz), 40 Hz falls to
  ~2.7× bandwidth, brushing the low edge of the 4-10× industrial band. That is the *only*
  post-retune argument to raise toward ~50-60 Hz — and it is gated on first reducing the
  chase replan cost (to lift U1), because at 15 Hz bandwidth the pure downward pressures
  still don't need more than 40 Hz.
- The C2 firmware interpolant (independent of the retune) removes D2 entirely, further
  weakening any case to raise the rate.

**Net: 40 Hz remains optimal before and after the retune.** The retune widens the
servo-side headroom that was never binding; it does not create a need for more knots.

---

## 6. Why "faster servo ⇒ faster knots" is a trap here

It is tempting to reason: retune lifts servo bandwidth → industrial rule says stream at
4-10× bandwidth → therefore raise the knot rate. That inference fails on this system
because the two ceilings are set by *different subsystems*:

- The **servo-usefulness** ceiling (U3) is a property of the leg plant. The retune moves
  it up.
- The **compute** ceiling (U1) is a property of the Jetson's chase-follower cost. The
  retune leaves it exactly where it is (~45-50 Hz).

You cannot spend servo headroom you can't compute. Raising the knot rate to satisfy "4×
of a 15 Hz servo" (=60 Hz) crosses the ~50 Hz compute ceiling and reintroduces the S3 emit
gaps — trading a servo-margin nicety for the *exact* availability failure the last two
sessions eliminated. The correct order of operations, if catch-timing accuracy ever
demands it, is: **first reduce the chase replan cost (raise U1), then consider the rate** —
never rate-first.

---

## 7. Industrial sanity check

Rule of thumb for setpoint/reference streaming into a servo loop: **stream at 4-10× the
closed-loop bandwidth** — high enough that the command quantization, not the plant, is
irrelevant to the response; not so high that you waste bus/compute on detail the plant
filters.

- **Today:** 40 Hz / 6.4 Hz = **6.25×** — dead center of the band. This is an independent
  confirmation that 40 Hz was well-chosen: it was arrived at from the MPC-fine-step
  heritage (`SEGMENT_T_S` = the "nominal MPC fine step", `canbridge_config.h:116`), yet it
  lands exactly where the servo-bandwidth rule would put it.
- **Post-retune (BW ~10-15 Hz):** the band shifts to 40-150 Hz. 40 Hz = 2.7-4×, brushing
  the low edge. Mildly under-sampled by the rule, but fine given the weak downward
  pressures; the honest fix if it matters is a *modest* bump to ~50 Hz *after* U1 is
  relieved, not a jump to 60-100 Hz.

The industrial rule and this system's first-principles band agree: 40 Hz is a good number,
and the useful range is tens of Hz, not hundreds.

---

## 8. Measurements that would sharpen the estimate

Ordered by value. The model's two soft spots are the U1 compute ceiling (inferred crossover
~45-50 Hz) and the post-retune servo bandwidth (hypothesized 10-15 Hz).

**(a) Emit-gap-vs-rate sweep, offline on the Jetson — highest value. — ✅ DONE 2026-07-11, see §10.**
Drove the production `KnotEmitter.frame` + `TargetFollower.follow` pipeline with the
recorded S3 stick stream at {25, 30, 40, 50, 60, 80} Hz on this Jetson, per-tick REAL
wall-clock timing, `knot_dt_s` pinned to 1/rate. **Result: the inferred ~45-50 Hz ceiling
is now MEASURED at 50 Hz** — see §10 for the full table, method, and the sharper
accept-restricted (worst-regime) would-gap curve.

**(b) Servo closed-loop bandwidth from the S4b retune data.**
The S4b `pos_gain` sweep {25, 40, 55} already yields ring frequencies {4.0, 6.4, 8.8} Hz.
Add a small chirp (Level-3 `leg-gain-tuning-methodology.md:288-303`) or a step-response fit
(Level-2) pre/post retune to get the true closed-loop −3 dB bandwidth. **This anchors the
industrial 4-10× band and tells you how much servo headroom the retune actually buys** — i.e.
whether the post-retune case for ~50 Hz in §5/§6 is real or moot.

**(c) Micro-benchmark `KnotEmitter.frame()` on the Jetson — cheap.**
Time `emitter.frame()` (3× `_ik`) directly to replace the inferred 3-5 ms (from the
377 ms/200-sample analytic profiling) with a measured pre-publish cost. Sharpens the U1
ceiling and would reveal the easy win of not computing `compute_jacobian` for the
u1/u2 samples that only need `pose_to_leg_lengths` (`emitter.py:75-76` discards J1/J2).

**(d) Teensy interp ISR headroom — confirmation only.**
Read `interp_deadline_misses()` / `interp_max_jitter_us()` (`leg_interp.cpp:476-478`, on
the profiling path) over a session. Expected near-zero; confirms the ISR has margin for a
`SEGMENT_T_S` change and that the 6 CAN mailbox writes fit the 2000 µs tick. Rate-independent,
so this only de-risks a *change*, it doesn't move the sweet spot.

---

## 9. Is changing the rate worth it? — verdict and cost

**Cost of any rate change** (from `mvp-trajectory-bringup.md:1138-1141`, "Configurable knot
rate" deferred):

1. **Firmware `SEGMENT_T_S`** change (and the extrapolation windows
   `MAX_EXTRAP_DT_S` / `EXTRAP_DECAY_DT_S`, `canbridge_config.h:116-118`) — a compile-time
   constant today. The plan's preferred path is a **per-frame segment-duration field in
   `SetpointPayload`** (protocol bump via `generate_udp_protocol.py`; staleness/extrap
   windows scale from the live value) so the rate stops being baked in — a protocol
   version bump touching all four generated copies + the xlang wire-hash pin.
2. **Re-validate the hermite_xref harness** at the new T: `teensy_interp.py` ↔
   `motor_guard.py` must still match to 0.0 rev (the invariant the whole interp port rests
   on, `leg_interp.cpp:4-8`), and the Teensy float32-vs-float64 residual re-checked.
3. **Re-run the bench battery** (S1-S8, `mvp_bench_runbook.md`) — every phase's timing and
   gap acceptance was validated at 40 Hz.

**Verdict by direction:**

- **Raise above 40 Hz: NOT worth it.** Gains are negligible (fidelity 22 → <10 µm invisible;
  latency 25 → 17 ms imperceptible; catch quantization ±12.5 → ±10 ms — already inside spec).
  Cost is real: crosses the ~45-50 Hz Jetson compute ceiling in the chase-follower regime →
  reintroduces the S3 emit-gap failure class, *plus* the firmware/xref/bench re-validation
  above. **Negative expected value.**
- **Lower to ~30-33 Hz: marginally defensible, not currently justified.** Buys chase-follower
  compute headroom; keeps everything inside spec; slightly worse catch margin and (until C2)
  a larger join step. Worth it **only if** measured evidence (§8a, or a live session) shows
  the chase-follower regime gapping at 40 Hz. Even then, the cheaper first lever is to
  **reduce the chase replan cost** (raise U1) — e.g. the `emitter.py:75-76` unused-Jacobian
  win, or a coarser validate decimation — not to pay the rate-change re-validation cost.
- **Keep 40 Hz: the recommendation.** It is the lowest rate that holds full catch-timing
  margin, it sits ~20 % under the compute ceiling, and it is 6.25× the servo bandwidth —
  the industrial sweet spot. The 2026-07-10 forensics already established the *rate* was
  never the problem (the servo gains were); this analysis adds that the rate is also close
  to *optimal*, not merely acceptable.

**The one thing to watch:** the chase-follower p99 compute margin at 40 Hz is thin (~6 ms).
If the post-retune sessions or Phase-6/7 catch work push the chase cost up (longer horizons,
heavier catch replans), the binding ceiling (U1) drops toward 40 Hz and the *first* symptom
will be `follow_block_max_ms` on `/trajectory/diagnostics` climbing toward the 25 ms period.
That is the signal to act — and the action is to cut the replan cost, with a downward rate
nudge to ~30 Hz as the fallback, not to raise the rate.

---

## 10. MEASURED — the emit-gap-vs-rate sweep (2026-07-11)

This section replaces the §3 U1 / §8a *inferred* ~45-50 Hz ceiling with a measured
number. It is the highest-value measurement the analysis called for, and it **confirms**
the verdict: the chase-follower compute ceiling is **50 Hz**, 40 Hz is the highest rate
that stays clean, and no rate change is warranted.

### 10.1 Method

Drove the **production** `KnotEmitter.frame` (pre-publish, 3× IK) + `TargetFollower.follow`
(post-publish chase replan) pipeline — imported unmodified from the main tree
`ros_ws/src/jugglebot` — over the recorded **S3 SpaceMouse stick stream**
(`pose_cmd.csv`, 2026-07-09, all-`SPACEMOUSE`, node-faithful quat→rotvec conversion),
mirroring `trajectory_node._emit_once`'s **publish-first** ordering exactly:

```
per virtual tick (clock advances by T = 1/rate):
  tau = now − t0
  frame_cost  = time( KnotEmitter.frame(plan, tau, seq) )      # pre-publish, every tick
  follow_cost = time( follower.follow(state0, target, limits) ) # post-publish, on a fresh target
  install res.plan (t0 = now) for the NEXT tick to sample
  total = frame_cost + follow_cost
  would_gap = total > T          # the tick overruns → emitter schedule slips → emit gap
```

- **Real wall-clock timing** (`perf_counter`) around each production call, on this Jetson,
  under **Python 3.8.10** — the same ROS2-Foxy interpreter `trajectory_node` runs under
  (production-faithful), numpy 1.24.4, governor `schedutil`. SOLO run, no other load.
- **`knot_dt_s` pinned to 1/rate per rate** (via `dataclasses.replace` on
  `TrajectoryLimits`) AND the `KnotEmitter` built with `knot_dt_s = 1/rate`, so BOTH the
  emitter's (τ, τ+dt, τ+2dt) sampling AND the gate's knot-step sampling
  (`feasibility.py` `n_knots = floor(dur/kdt)+2`, both `validate` and `validate_follow`)
  move to the swept rate — capturing the compounding "shorter period *and* more knots" effect.
- **Window:** the S3 incident worst-regime slice **[500, 632] s** (132 s, 13 200 stick rows
  — the aggressive stick-flinging that deadlocked the old follower). This is the binding
  chase-follower regime the U1 ceiling is about; the neutral head/tail of the 642 s file
  would only dilute the would-gap fraction with parked ticks. (A calibration pass on the
  tighter [569, 625] incident window reproduced 40/60/80 Hz within ~1 pp.)
- **Two would-gap fractions reported:** *all-tick* (over every emitter tick, the honest
  duty-cycle number) and *accept-tick* (restricted to real `build_follow` chase replans —
  the undiluted worst-regime signal; parked-stick ticks deadband and return in ~0.1 ms).
- Harness: `scratchpad/knotrate/emit_gap_sweep.py`; raw JSON `out/inc_default.json`,
  `out/inc_s4.json`; consolidated `out/emit_gap_sweep_results.csv`.

### 10.2 Results (window [500,632] s, per-tick REAL timing)

`frame` = `KnotEmitter.frame` cost (ms); `follow` = `TargetFollower.follow` cost (ms);
`total p99` = per-tick frame+follow p99 (ms); `margin` = period − total p99 (ms, negative =
p99 tick overruns); `gap-all` / `gap-accept` = would-gap fraction over all ticks / accept
ticks.

**Default session tier (100 / 400 / 8000):**

| Rate | T (ms) | frame p50/p99 | follow p50/p99 | total p99 | margin | gap-all | gap-accept |
|-----:|-------:|--------------:|---------------:|----------:|-------:|--------:|-----------:|
| 25 | 40.0 | 2.42 / 3.09 | 0.11 / 12.47 | 15.63 | **+24.4** | 0.00 % | 0.00 % |
| 30 | 33.3 | 2.45 / 2.94 | 0.12 / 13.59 | 16.80 | **+16.5** | 0.00 % | 0.00 % |
| **40** | **25.0** | **2.44 / 2.95** | **0.12 / 15.39** | **18.30** | **+6.7** | **0.02 %** | **0.00 %** |
| 50 | 20.0 | 2.42 / 2.97 | 0.11 / 20.27 | 23.16 | **−3.2** | 4.04 % | 24.51 % |
| 60 | 16.7 | 2.43 / 3.05 | 0.11 / 23.11 | 26.07 | −9.4 | 6.72 % | 44.26 % |
| 80 | 12.5 | 2.44 / 3.04 | 0.12 / 27.47 | 30.51 | −18.0 | 11.50 % | 87.70 % |

**S4 heavier tier (156 / 660 / 10500):**

| Rate | T (ms) | frame p50/p99 | follow p50/p99 | total p99 | margin | gap-all | gap-accept |
|-----:|-------:|--------------:|---------------:|----------:|-------:|--------:|-----------:|
| 25 | 40.0 | 2.48 / 3.04 | 0.12 / 10.65 | 13.66 | +26.3 | 0.00 % | 0.00 % |
| 30 | 33.3 | 2.44 / 2.95 | 0.11 / 11.00 | 13.92 | +19.4 | 0.00 % | 0.00 % |
| **40** | **25.0** | **2.43 / 2.94** | **0.11 / 12.53** | **15.42** | **+9.6** | **0.02 %** | **0.00 %** |
| 50 | 20.0 | 2.44 / 2.98 | 0.11 / 15.00 | 17.92 | +2.1 | 0.41 % | 2.70 % |
| 60 | 16.7 | 2.45 / 2.97 | 0.11 / 16.23 | 19.10 | −2.4 | 2.28 % | 17.51 % |
| 80 | 12.5 | 2.45 / 3.01 | 0.11 / 19.76 | 22.66 | −10.2 | 8.55 % | 78.83 % |

Standalone `KnotEmitter.frame()` micro-benchmark (4 000 calls, hold+move mix):
default **p50 2.59 / p99 2.90 ms**, s4 **p50 2.66 / p99 3.16 ms** — matches the in-replay
frame cost and is **below** the 3-5 ms inferred in §3.

### 10.3 What the numbers say

1. **The ceiling is 50 Hz (default tier).** The would-gap fraction first exceeds 1 % at
   50 Hz (all-tick 4.04 %, accept-tick 24.51 %), and the total per-tick p99 first exceeds
   the period at 50 Hz (23.2 ms > 20 ms). This **confirms the inferred 45-50 Hz band** —
   the measured value sits right at its top edge.
2. **40 Hz is clean, with the predicted margin.** would-gap 0.02 % (all) / 0.00 % (accept),
   total p99 18.3 ms vs the 25 ms period = **+6.7 ms** — matching the §3 "~6 ms p99 margin"
   estimate almost exactly. 40 Hz is the highest swept rate that stays under 1 %.
3. **Both compounding effects are directly observed.** `frame` cost is flat (~2.5 ms p50,
   rate-independent — it is 3× IK regardless of rate), while `follow` p99 grows
   monotonically with rate (12.5 → 27.5 ms, default) because a shorter `knot_dt` puts more
   knots in the fixed-duration horizon that `validate_follow` / `chase` sample — exactly the
   §3 U1 mechanism. This is why the ceiling pulls *in* from the naive "period-only" ~52 Hz.
4. **`follow` p50 is ~0.1 ms because the worst regime is deadband-dominated.** Even in the
   incident window ~80 % of ticks deadband against the converged chase target and return
   immediately; the *cost* lives entirely in the p99 tail of the active-chase ticks — which
   is precisely why the **accept-restricted** would-gap is the sharp signal (24.5 % of chase
   replans already gap at 50 Hz vs 4.0 % of all ticks).
5. **The "heavier regime" intuition is inverted — a measured surprise.** The S4 tier
   (higher vel/acc/jerk) is *cheaper*, not heavier: its `follow` p99 is lower at every rate
   (12.5 vs 15.4 ms at 40 Hz) and it gaps *less* (ceiling ~50-60 Hz vs 50 Hz). Higher limits
   let `build_follow` accept on the first `validate` more often (fewer stretch iterations)
   and let the chase reach further per tick (shorter-horizon plans → fewer knots to sample).
   **So the default session tier is the binding one**; a limit ramp does not erode the
   compute ceiling — it slightly relaxes it.

### 10.4 Verdict — re-confirmed

The measurement **confirms** the analysis. Keep 40 Hz:

- It is ~20 % below the **measured** 50 Hz compute ceiling, with a **measured +6.7 ms p99
  margin** in the worst (SpaceMouse/CATCH, default-tier) regime.
- Raising to 50 Hz would put ~1-in-4 chase replans over the period (accept-gap 24.5 %) —
  reintroducing the exact S3 emit-gap failure class publish-first + the chase clamp fixed,
  on top of the firmware `SEGMENT_T` / xref / bench re-validation cost (§9). Confirmed
  negative expected value.
- The one caveat from §9 stands and is now quantified: the 40 Hz p99 margin is +6.7 ms;
  if Phase-6/7 catch work pushes the chase cost up, watch `follow_block_max_ms` climbing
  toward the 25 ms period, and cut the chase replan cost first (the `emitter.py:75-76`
  unused-Jacobian win — `frame` discards J1/J2 yet still computes them) before touching
  the rate. The S4 result shows a rate *ramp* is not the lever; replan-cost reduction is.

---

### Appendix — key constants and their sources

| Quantity | Value | Source |
|---|---|---|
| Knot rate / period | 40 Hz / 25 ms | `hardware_config.yaml:372`; `hardware_config.py:149`; `canbridge_config.h:116`; `emitter.py:42` |
| Interp reconstruction error | 22 µm @ 40 Hz | logbook `2026-07-10-...:57-60` (MEASURED) |
| C1 join vel-FF step | 6.1 mm/s = 4 % peak ≈ 0.5·T·a_peak | logbook `2026-07-10-...:58-60` (MEASURED) |
| Servo bandwidth | 6.37 Hz = 40/(2π) | logbook `2026-07-10-...:48`; `session_gain_retune.md:28-29` (MEASURED) |
| Leg gains / vel limit | 40 / 0.20 / 0.32 ; 4.0 rev/s | `hardware_config.yaml:319-321, 286` |
| Chase `follow()` p99 | 14.9 ms (S3 replay) | follower-cadence RESOLUTION; `mvp-trajectory-bringup.md:493` (MEASURED) |
| Fast `validate_follow` | ~1.5-4 ms accept | `mvp-trajectory-bringup.md:742` (MEASURED) |
| Old reject-path cost | p50 41 / p99 58 ms | follower-cadence `§Corrections` (MEASURED) |
| Frame IK ×3 (`KnotEmitter.frame`) | **p50 ~2.5 / p99 ~3.0 ms** | §10 sweep 2026-07-11 (MEASURED; *below* the 3-5 ms inferred) |
| Chase-compute ceiling (would-gap > 1 %) | **50 Hz** (default tier) | §10 sweep 2026-07-11 (MEASURED; confirms inferred 45-50 Hz) |
| Total per-tick p99 @ 40 Hz | **18.3 ms** (+6.7 ms margin) | §10 sweep 2026-07-11 (MEASURED) |
| Emit gaps (post publish-first) | flat 25 ms | logbook `2026-07-10-...:60-64` (MEASURED) |
| Emit gaps (old inline, S3) | 25→62→78→146 ms | follower-cadence `§1.4` (MEASURED) |
| CAN3 setpoint load | 500 Hz × 6 ≈ 39 % of 1 Mbps, rate-independent | logbook `2026-07-10-...:60-63`; `leg_interp.cpp:390-406` |
| Interp ISR | 500 Hz, 2000 µs period, ~1 % util | `canbridge_config.h:69-70`; `leg_interp.cpp:210-467` (util INFERRED) |
| Catch/timed spec | ±25 ms; quant ±T/2 = ±12.5 ms | `mvp_bench_runbook.md:390-402`; `mvp-...:875` |
| Extrap windows | 50 ms / 60 ms | `canbridge_config.h:117-118` |
| Rate-change cost | SEG_T + per-frame field + xref + S1-S8 | `mvp-trajectory-bringup.md:1138-1141` |
