---
title: Content-freshness contract on the can-bridge lead clamp (DRAFT PROPOSAL)
created: 2026-08-14
status: superseded   # premise disproved by the RX-ring delay-line localisation; salvage re-homed (see Archival note)
completed: 2026-08-15
owner: harrison
last_updated: 2026-08-15
related_logbook:
  - 2026-08-12-s1-aged-bridge-isolation-teensy-internal.md
  - 2026-07-18-teensy-uptime-tracking-degradation.md
  - 2026-07-16-max-deviation-guard-tracking-lag.md
  - 2026-08-12-fw12-cache-diag-instrumentation.md
  - 2026-08-12-fault-machine-staleness-guard-atomic-read.md
related_config:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h → MAX_LEAD_REV
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h → MAX_DEVIATION_REV
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h → MOTOR_FB_STALENESS_US
related_code:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp::interp_isr
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp::evaluate_guard
  - ros_ws/src/jugglebot/Teensy_code_canbridge/axis_state.h::snapshot_pos_vel
archived: 2026-08-15
---

# Content-freshness contract on the can-bridge lead clamp

> **STATUS: DRAFT PROPOSAL — owner review required. Nothing here is
> implemented.** This document proposes a change to the *live leg control
> path*: the lead clamp is the last command authority before the leg ODrives,
> and no Jetson-side layer sits below it. No code, firmware or configuration
> may be written against this document until the owner has ruled on
> §&nbsp;11 (Decisions required). Two of those decisions change the safety
> envelope and one of them is a measurement that could invalidate the whole
> design.
>
> **Frontmatter note.** `status: proposed` is outside the controlled plan
> vocabulary in `DOCUMENTATION_GUIDE.md` § 2.6 (`active | completed |
> superseded`). It is used deliberately to mark a document that describes work
> nobody has approved yet, as distinct from work in flight. Two resolutions are
> available and the choice belongs to the owner: promote this document to
> `active` on approval and leave the vocabulary alone, or add `proposed` to the
> guide's vocabulary first and ripple it through
> `DOCUMENTATION_GUIDE.md` § 2.6 and § 3. The second is the one the
> contracts-over-patches rule prefers — change the normative document *before*
> the practice drifts. `plans/active/INDEX.md` already carries one
> out-of-vocabulary status (`reference`), so the drift has a precedent and that
> is an argument for closing it, not for repeating it.

> **SUPERSEDED AS POSED — 2026-08-14. Do not review §§ D1–D5 as written; do not
> implement.** This document's premise — that the fault is a **content-freeze
> anchor pathology** (§ 1.1) — has been overturned by the S2 endgame and the
> 2026-08-14 RX-ring concurrency audit
> (`logbook/2026-08-14-ring-audit-available-leak-delay-line.md`). The root cause
> is a **delay line, not a freeze**: FlexCAN_T4's `events()` pops the RX ring
> before its `NVIC_DISABLE_IRQ` guard, so the consumer's non-atomic
> `_available--` races the CAN ISR's `_available++` **one-directionally**, the
> counter monotonically under-counts, and every delivery is `D` frames late with
> `D` ratcheting on uptime and capped at one ring lap (256 slots ≈ 114–135 ms).
> The ring's own full test **proves duplicates and stale re-reads impossible** —
> frames arrive **exactly once, in order, late**.
>
> **Two consequences, either of which alone is disqualifying:**
>
> 1. **A content-change detector cannot see delayed-moving values.** CF-1's
>    bit-identical anchor-triple test keys on *content not changing*; under a
>    pure delay the content changes on schedule — it is simply **old**. The
>    detector is looking for the wrong invariant. Relatedly, ~**97 %** of the
>    § 1.1 "bit-identical per-axis freezes" turned out to be a **Jetson-side
>    artifact** (`/robot_state`'s latest-wins latch republished by a 100 Hz
>    ROS-clock timer with no staleness gate), so the population the detector was
>    designed against is largely not a Teensy phenomenon at all.
> 2. **§ 11's D3 — the measurement this document itself named as able to
>    invalidate the design — was run, and it did.** There is **no viable
>    `T_FREEZE_MIN`**: margin against the worst healthy-plant hold at speed is
>    **0.49× at 30 ms and 0.82× at 50 ms** (worst healthy hold 40.1 ms observed /
>    60.7 ms conservative, at 0.79 rev/s commanded). Encoder quantisation is
>    **ruled out** as the false-positive mechanism (effective quantum 2⁻¹⁹ rev),
>    and the § 3 rest/motion floor works as designed — the design fails for a
>    different reason: the 0.100 rev clamp budget is consumed in **40 ms** at
>    2.5 rev/s, while a healthy plant already holds values 40–60 ms at speed, so
>    **no threshold both spares a healthy plant and fires before the anchor is a
>    full budget stale**. The detector **detects but does not protect**.
>
> **The two salvageable pieces, named so they are not lost:**
>
> - **The velocity-extrapolated anchor** (`pos + vel·Δt`) — no detector, no
>   threshold, and on the fresh plant it cuts anchor-error p95 **0.129 →
>   0.064 rev** and the fraction of freezes exceeding the clamp budget **0.160 →
>   0.000** (n = 25). This is the right *shape* for any post-fix clamp hardening,
>   and it replaces §§ 3–4's whole detector-plus-fallback structure rather than
>   sitting inside it.
> - **The T = 100 ms / 0.67 rev/s content-hold threshold as a REPORTING
>   criterion** (§ 7.3's monitor, i.e. the arc's P3 alarm input): **0.28/min
>   healthy vs 4.75/min aged, 17×**. Useless as protection, well-calibrated as an
>   alarm.
>
> **Status stays `proposed`, and D5 is untouched.** Rework is **deferred until
> after the FW 14 fix's validation soak**, which is what will show whether any
> clamp hardening is still warranted at all — if the delay line is the whole
> fault, the commanded stops go away with it and there may be nothing left to
> harden. § 9's framing was right and is now load-bearing: this document is a
> *mitigation of the consequence*, and the consequence may not survive the
> cause's removal. §§ 2, 5, 6 and 10 (the enforcement-point enumeration, the
> interaction analysis, the ISR access discipline, and the verified backstop
> chain) remain accurate and are worth keeping whatever replaces the rest.
> **Wire-id note (2026-08-14):** § 7's "new `MsgType` 0x92 (next free)" is STALE — 0x92 was claimed by FW 13's `RING_DIAG` the same day. Any rework that lifts § 7's wire design must renumber to the next free id (0x93+), or recorded `RING_DIAG` bags will decode as the other message.

## 1. Context — the failure this closes, stated as failure modes

### 1.1 What was measured

S1 (`logbook/2026-08-12-s1-aged-bridge-isolation-teensy-internal.md`, bridge
aged 62.9–63.1 h) localized the uptime-growing command-latency drift to the
can-bridge Teensy: only the Teensy reboot restored the fresh-boot lag class
(15.8 ms at 27 s uptime against 290–340 ms at 63 h), with the Jetson transport,
the ODrives, the Teensy→ODrive TX path, RX wire errors, the heap, the interp
tick and CAN throughput all exonerated by in-bag data. The surviving mechanism
signature was a **per-axis encoder-cache refresh-stall tail**: the median
refresh interval is a constant 10.0–10.2 ms in every bag, and only the tail
degrades — 9.15 / 17.75 / 16.43 % of intervals exceed 30 ms when aged against
**4.29 %** fresh, p95 80–130 ms against 29.7 ms, max ~500 ms. Frames keep
arriving on time; the value inside them stalls.

S2 (onset forensics, run 2026-08-13) then killed the obvious follow-on
hypothesis and sharpened the mechanism:

- **The cache-AGE hypothesis is dead.** `pos_timestamp_us` age was *fresher* at
  28 h than at 1.3 h, and stayed ≤ 11 ms p95 straight through the freezes.
- **The mechanism is a bit-identical per-axis telemetry freeze.** `pos_rev`,
  `vel_rps` and `iq_measured` freeze **as a unit**, bit-for-bit, for
  ~100–150 ms, while `pos_timestamp_us` keeps advancing on every write and
  frames keep flowing.
- **An identical-firmware reflash cleared it**, which also serves as the clean
  reboot-only re-test S1 owed (S1's Arm C was an FW 10 → 11 flash, not a bare
  power-cycle).

> S2 has no logbook entry as at 2026-08-14. Every S2 number quoted in this
> document is therefore currently unverifiable from a committed artefact, and
> the S2 entry is a hard prerequisite for approving this proposal — the
> cite-with-the-triple rule exists exactly so a contract cannot be built on an
> uncommitted measurement.

### 1.2 Why that becomes a *motion* fault

The lead clamp (`leg_interp.cpp:458-468`) is:

```cpp
const float fb = axes[i].pos_rev;          // single-word atomic read
float dev = cmd_pos[i] - fb;
if (dev > LEAD)       { dev = LEAD;  clamp_mask |= (uint8_t)(1u << i); }
else if (dev < -LEAD) { dev = -LEAD; clamp_mask |= (uint8_t)(1u << i); }
cmd_pos[i] = fb + dev;
```

It is **memoryless** and it checks **no freshness of any kind**. Anchored on a
frozen `fb`, the executed command is pinned at `frozen ± MAX_LEAD_REV` and
stops advancing entirely, for as long as the freeze lasts. Relative to where
the leg actually is — ahead of the frozen anchor by `v · t_freeze` — a pinned
command is a commanded **stop**, and once `v · t_freeze > MAX_LEAD_REV` it is a
commanded **reversal**. At 2.5 rev/s the clamp saturates 40 ms into a freeze,
so a 100–150 ms freeze produces 60–110 ms of commanded stop.

The arithmetic that follows is the whole story: **eleven such moves compound
into the 283 ms end-to-end lag measured at 28 h uptime.** The clamp converted a
telemetry-content fault into a motion fault, and the S1 clamp-onset statistics
confirm the loop is stale from the first tick rather than pinned by mechanical
binding — 94 % of clamping moves clamp within 100 ms of onset and **30 of 49
moves are already clamped at motion onset** on the aged plant, against **0 of
24** fresh. Per-leg clamp duty in moves: 0.739 / 0.655 / 0.443 aged against
**0.058** fresh.

### 1.3 Why the existing guards cannot see it

| Guard | What it tests | Why it misses a content freeze |
|---|---|---|
| `MOTOR_FB_STALE` (`fault_machine.cpp:388-404`) | `micros64() − pos_timestamp_us > 150 ms` | The timestamp is written on **every** `write_pos_vel` call regardless of whether the value changed. Measured cache age stayed ≤ 11 ms p95 through the freezes. |
| `MAX_DEVIATION` (`fault_machine.cpp:361-376`) | `\|u0 − axes[i].pos_rev\| > 1.0 rev` | Reads the same frozen anchor, so it misreads in the same direction. Its own source comment already concedes the gap: *"MAX_DEVIATION above only catches a freeze while the command is moving AWAY from the frozen encoder; a HOLD freeze (cmd ≈ frozen enc) would otherwise go unnoticed and fly blind."* |
| `MOTOR_OVERSPEED` (`fault_machine.cpp:344-347`) | `\|axes[i].vel_rps\| > 16.5 rev/s` | `vel_rps` freezes bit-identically **with** `pos_rev`. A frozen velocity cannot trip an overspeed test. |
| Lead clamp (`leg_interp.cpp:458-468`) | `\|cmd − fb\| ≤ 0.10 rev` | The subject of this document. |

**Timestamp-freshness cannot close this class.** That is the finding that
forces a new invariant rather than a tightened threshold: a timestamp says when
a write happened, and every one of these guards needs to know whether the
*content* is still informative.

### 1.4 The class

> **A feedback-anchored command authority must never trust an anchor whose
> CONTENT is not evolving consistently with commanded motion.**

That sentence is the contract. Everything below is its definition, its single
enforcement point, its fallback policy and its test.

## 2. Climbing one level — every feedback-anchored authority on the leg path

A grep of `axes[i].pos_rev` across the can-bridge firmware returns **nine**
read sites in six functions. Enumerating them is what turns a clamp patch into
a contract, and it also shows why the enforcement point can still be singular.

| # | Site | Authority | Rate | Blind to content-staleness? | In scope |
|---|---|---|---|---|---|
| 1 | `leg_interp.cpp:460` | Lead clamp — the primary | 500 Hz ISR | **Yes** | **Yes** |
| 2 | `leg_interp.cpp:480` | Stroke-clamp non-finite fallback anchor | 500 Hz ISR | Yes | **Yes** (same function) |
| 3 | `leg_interp.cpp:557` | Recovery-slew re-clamp | 500 Hz ISR | Yes | **Yes** (same function) |
| 4 | `leg_interp.cpp:502` | Recovery-slew re-baseline at the output-enable edge | edge-triggered | Yes | Discussed, §&nbsp;5.2 |
| 5 | `leg_interp.cpp:519` | Recovery-slew pin during a cold-start move | 500 Hz ISR | Yes | Discussed, §&nbsp;5.2 |
| 6 | `leg_interp.cpp:697` | `interp_begin_stow` descent base | one-shot | Yes | Out, §&nbsp;9 |
| 7 | `fault_machine.cpp:368` | `MAX_DEVIATION` E-STOP | 10 Hz | Yes | Analysed, §&nbsp;5.1 — **no change proposed** |
| 8 | `leg_activate.cpp:187,228`, `leg_deactivate.cpp:185,227` | Cold-start move planners | one-shot | Yes | Out, §&nbsp;9 |
| 9 | `Teensy_code_canbridge.ino:145` | `live_deviation` telemetry | 10 Hz | Yes | Out — but see §&nbsp;7.3 |

Sites 1–3 are the 500 Hz command authority and are the contract's scope. They
are three reads of the same quantity inside one ISR, so routing all three
through one accessor keeps the enforcement point singular in the sense the
house model uses the term — one derivation, several consumers, exactly as
`axis_state.h` describes `leg_present()` as *"the single enforcement point for
the present-axis contract"*.

**The class is not confined to the firmware.** `motor_guard.py` carries the
same structural pair on the dormant MPC path — a lead clamp
(`motor_guard.py:1049-1053`, `MAX_LEAD_REV = 0.15`) and a deviation guard
(`motor_guard.py:569-581`, `MAX_DEVIATION_REV = 0.5`) both anchored on the same
`_motor_fb_pos_rev` cache, both memoryless, both content-blind. It is out of
scope because it is parked (`refactor-2026-07.md` Phase 3) and not launched, but
it is named here for one reason: **reviving the MPC chain without applying this
contract would reintroduce the defect on the revived path.** That obligation
belongs with the revival, and this sentence is where a future session will find
it.

## 3. The invariant

Stated normatively, in the K-contract style. `CF` = content freshness.

### CF-1 — Definition of a content-stale anchor

For leg axis *i*, let the **anchor triple** be the three single-word cache
fields `axes[i].pos_rev`, `axes[i].vel_rps` and `axes[i].iq_measured`, compared
as raw 32-bit patterns (integer equality on the bit representation, not
floating-point `==` — so `-0.0f` and `+0.0f` are distinguished and a `NaN`
compares equal to itself).

Let `t_change(i)` be the monotonic time of the most recent tick at which any
member of the anchor triple differed from its value at the preceding tick.

The anchor for axis *i* is **content-stale** at tick *t* iff **both**:

- **(a) Frozen:** `t − t_change(i) ≥ T_FREEZE_MIN`, and
- **(b) Contradicted by demand:** the *net* commanded displacement since
  `t_change(i)` satisfies `|cmd_pos_i(t) − cmd_pos_i(t_change(i))| ≥ D_TRIG`.

**Why the triple and not `pos_rev` alone.** S2 measured `pos`, `vel` and `iq`
freezing as a unit. Requiring all three closes the one physical confusion a
position-only test cannot: a leg mechanically bound against an obstruction also
stops moving, but it does **not** stop dithering in the low bits, and its
`iq_measured` under a standing position error is large and noisy. A
bit-identical `iq_measured` is not something a loaded, energised axis produces.
The triple test therefore discriminates *telemetry frozen* from *leg stopped*,
which is precisely the discrimination the fallback policy in §&nbsp;4 needs to
be safe.

**Why (b), and why it is not a velocity threshold.** The prompt-level candidate
definition gated on `|cmd_vel| > threshold`. A *net commanded displacement*
gate is strictly better for three reasons:

1. **Rest is lawful with no threshold to tune.** A stationary leg legitimately
   repeats its position for arbitrarily long. Its net commanded displacement is
   ~0, so (b) is never met and CF-1 never fires — regardless of how long the
   anchor repeats, and regardless of leg velocity noise.
2. **It is immune to the Mode-1 `s`-clamp defect.** `leg_interp.cpp:394-411`
   saturates `s` at 1.0, at which point `cmd_pos` freezes at `p1` while
   `cmd_vel` **holds at `v1`** (`dh11(1) = 1`, every other basis derivative is
   0). `cmd_vel` is therefore not a trustworthy proxy for "the command is
   moving"; the command's own increment is, by construction. See §&nbsp;5.3.
3. **It closes the low-speed quantisation-aliasing window analytically.**
   Conditions (a) and (b) together imply a minimum commanded velocity of
   `D_TRIG / T_FREEZE_MIN` before CF-1 can fire at all. At the proposed values
   that is **0.67 rev/s**, at which a healthy anchor advances ~2.5 × 10⁻³ rev
   per nominal broadcast period — roughly four orders of magnitude above
   float32 epsilon at a typical leg position of ~5 rev (~5 × 10⁻⁷ rev). Aliasing
   would require the drive's own `pos_estimate` quantum to exceed 1/400 rev.
   **That is a measurement, not an assumption** — see Decision D3.

### CF-2 — Release

The content-stale condition for axis *i* clears on the **first tick at which
any member of the anchor triple changes**. Release is edge-triggered on
content, never on a timer, and never on `pos_timestamp_us`.

### CF-3 — Bounded fallback authority

While axis *i* is content-stale, the command authority of §&nbsp;4 applies, and
the emitted command shall satisfy, at every tick and unconditionally:

```
|cmd_emitted_i − axes[i].pos_rev| ≤ MAX_LEAD_REV + DR_MAX_REV
```

This is the contract's single safety number. It is the quantity a reviewer
should check every implementation line against, and it is what §&nbsp;10's risk
register is written in terms of.

### CF-4 — Bounded duration, escalating into an existing path

Content-staleness on any present leg persisting for `T_FREEZE_MAX` shall raise
the existing **`MOTOR_FB_STALE`** condition, which is already a *recoverable,
non-latching* output suppression (`fault_machine.cpp:427,456` — deliberately
excluded from the E-STOP latch). No new fault state is introduced. The
`MOTOR_FB_STALE` condition so raised shall clear on CF-2 release, not on a
timestamp test — a content-raised suppression that cleared on the timestamp
rule would clear instantly, because the timestamps never went stale, and the
guard would flap at 500 Hz.

### CF-5 — Loud, never silent

Every content-stale episode shall be counted per axis and uplinked
(§&nbsp;7). A contract whose engagement is invisible degrades into an unowned
behaviour change; the whole reason this defect survived four weeks and three
firmware revisions is that the clamp's engagement was visible only as a
one-byte instantaneous mask with no occupancy census behind it.

### Proposed constants — all PROVISIONAL

| Name | Proposed | Derivation | Pinned by |
|---|---|---|---|
| `T_FREEZE_MIN` | **30 ms** | Above the *healthy* refresh p95 (29.7 ms, Arm C) so a healthy tail does not declare; below the observed freeze band (100–150 ms) with margin. | D3 |
| `D_TRIG` | **0.02 rev** | 20 % of `MAX_LEAD_REV`. With `T_FREEZE_MIN` it sets the 0.67 rev/s rest/motion boundary. | D3 |
| `T_FREEZE_MAX` | **120 ms** | Below `MOTOR_FB_STALENESS_US` (150 ms) so the two staleness bounds nest rather than race; above the freeze band's lower edge so a typical freeze is ridden out rather than escalated. | D1, D2 |
| `DR_MAX_REV` | **0.20 rev** | The credit ceiling. Buys 80 ms of coverage at 2.5 rev/s. **This is the safety-envelope decision** — see D1. | **D1** |
| `DR_DECAY_RPS` | **1.0 rev/s** | Reuses `RECOVER_SLEW_VEL_RPS`, a constant already validated on hardware as a tolerable command-recovery rate (2026-07-11 clear-errors jolt forensics). | D1 |

**None of these are derived from a first-principles bound on the fault.** The
nominal ODrive `get_encoder_estimate` broadcast period is *unconfirmed*: the
FW 12 documentation claims ~272 frames/s/axis (≈3.7 ms) and the S1 entry flags
it as not independently confirmed, while the measured 10.0–10.2 ms median is
the 100 Hz `/robot_state` sampling floor and therefore an upper bound on the
true period, not a measurement of it. FW 12's `enc_frames[7]` counter measures
it directly on first flight and must be read before these values are frozen.

## 4. Fallback policy — the design decision

Three policies were considered for what the clamp does while its anchor is
content-stale. All three are stated in terms of CF-3's bound.

### 4.1 Option A — HOLD

*Freeze the executed command at its last emitted value while content-stale;
release on the first anchor change.*

**Failure-mode analysis.**

- **It does not remove the commanded stop.** S1 measured 30 of 49 moves already
  clamped *at motion onset*. At onset the last emitted command is ≈ the leg's
  position, so holding it is exactly a commanded stop — the same 100 ms halt,
  reached by a different route. HOLD removes only the *reversal* (the command
  walking backward toward `frozen ∓ LEAD`), not the halt.
- **The catch-up on release is unchanged from today.** On CF-2 release the
  anchor jumps to truth. If the leg obeyed the hold, the trajectory has advanced
  by up to `v · t_freeze` (0.375 rev at 2.5 rev/s over 150 ms) while the leg did
  not, so `dev` is large, the clamp re-engages, and the command steps to
  `fb + LEAD` — a `pos_gain × LEAD = 40 × 0.10 = 4 rev/s` P-term demand plus up
  to `VELFF_CAP = 3.5 rev/s` of feedforward. That is the same harsh catch-up the
  current behaviour produces, bounded by the same numbers.
- **It is the safest option under a simultaneous mechanical bind**, because it
  never increases command authority: CF-3 holds with `DR_MAX_REV = 0`.

**Verdict: rejected as the primary policy.** It buys the reversal only, at the
cost of leaving the dominant symptom — the commanded halt, and therefore the
283 ms lag — entirely in place. It is retained as the **fallback-of-the-fallback**
(D1's conservative endpoint): setting `DR_MAX_REV = 0` reduces the recommended
design to HOLD exactly, which makes HOLD a one-constant rollback rather than a
separate code path.

### 4.2 Option B — COAST (recommended, in the dead-reckoned-anchor form)

*Keep advancing the command along the trajectory while content-stale, with the
clamp band **shifted** onto a dead-reckoned anchor rather than widened.*

The mechanism, stated exactly:

```
while content-stale:
    dr_offset_i += (cmd_pos_i(t) - cmd_pos_i(t-1))        # the command's OWN increment
    dr_offset_i  = clamp(dr_offset_i, ±DR_MAX_REV)
    fb_eff       = axes[i].pos_rev + dr_offset_i          # dead-reckoned anchor
on release (CF-2):
    dr_offset_i decays toward 0 at DR_DECAY_RPS           # band returns, no step
clamp as today, against fb_eff instead of axes[i].pos_rev
```

**Why this shape and not "unclamped" or "a widened bound".** Three properties
fall out of it that neither alternative has:

1. **The clamp keeps its full authority.** The band stays exactly `2 ×
   MAX_LEAD_REV` wide at all times; only its centre moves. "Unclamped" discards
   the authority entirely; "widened" halves the resolution of the guard in both
   directions, including the direction the command is *not* travelling.
2. **It is transparent by construction while the anchor is uninformative.**
   `dr_offset` accrues at exactly the rate the command advances, so
   `cmd_pos − fb_eff` stays frozen at whatever deviation held at the freeze
   onset. The clamp therefore neither engages nor releases spuriously during a
   freeze; it simply stops being a factor, which is the correct behaviour for a
   guard whose input has gone uninformative.
3. **Trajectory reversal during a freeze is handled with no extra logic**,
   because `dr_offset` is a signed displacement rather than a widening.

**Failure-mode analysis.**

- **A genuine runaway or bind coinciding with a freeze is the real risk.** If
  the leg is bound while the anchor is frozen, the command advances by up to
  `DR_MAX_REV` past where the clamp would otherwise have held it, and the
  ODrive's position error reaches `MAX_LEAD_REV + DR_MAX_REV = 0.30 rev` at the
  proposed value → `pos_gain × 0.30 = 12 rev/s` of P-term demand, which is
  exactly the drive's configured `vel_limit`. **This raises the worst-case
  commanded-velocity demand under a bind-plus-freeze coincidence from 4 rev/s
  today to a vel_limit-saturated 12 rev/s.** That is the single largest safety
  consequence of this proposal and it is Decision D1.
- **It is bounded in four independent ways**, three of which are freeze-immune:
  `DR_MAX_REV` (position), `T_FREEZE_MAX` → `MOTOR_FB_STALE` (time), the
  **stroke clamp** (`leg_interp.cpp:471-486` — absolute `STROKE_MIN/MAX_REV`
  bounds that read no anchor at all, and that zero `vel_ff` and `torque_ff`
  when they engage), and the ODrive's own `vel_limit` and 10 A current limit.
  See §&nbsp;10.2.
- **The bind-versus-freeze discrimination is CF-1's job**, and the
  `iq_measured` term in the anchor triple is what does it: a bound leg under a
  standing position error cannot present a bit-identical current measurement.
  The residual exposure is a bind that *begins* inside a freeze window, which
  no anchor-side test can see.
- **The release transient is bounded by rate, not by size.** `dr_offset` decays
  at `DR_DECAY_RPS` rather than collapsing, so the command is drawn back into
  the nominal band over ≤ 200 ms at the proposed constants, with the clamp's
  authority returning monotonically. A collapse-to-zero would produce a command
  step of up to `DR_MAX_REV` — which is benign in the *fault* case (the step
  moves the command back toward where the leg actually is, reducing the drive's
  position error) and harmful only in the *bind* case, so the decay exists
  specifically to bound the bind case.

**Verdict: recommended.**

### 4.3 Option C — FAULT-FAST

*Treat content-staleness beyond `T` as `MOTOR_FB_STALE` immediately.*

**Failure-mode analysis.**

- **On an aged plant it would trip more or less continuously.** 9.15–17.75 % of
  refresh intervals exceed 30 ms when aged, and the freeze band is 100–150 ms.
  A `T` low enough to protect the move onset (≤ 40 ms at 2.5 rev/s) fires on
  every one of those.
- **Each trip is expensive in motion terms.** `MOTOR_FB_STALE` sets
  `allow = false` → `interp_set_output_enabled(false)` → the 500 Hz stream stops
  and the ODrives hold their last setpoint. On re-enable the recovery slew
  (`leg_interp.cpp:501-505`) re-baselines to the live encoder and ramps at
  `RECOVER_SLEW_VEL_RPS = 1.0 rev/s`. So each trip costs a commanded stop *plus*
  a 1 rev/s ramp back — strictly worse for tracking than today's clamp pin,
  which at least holds position rather than surrendering it.
- **It is correct-loud but operationally unusable as a primary policy**, and
  that judgement is the honest reading of the numbers rather than a preference.

**Verdict: rejected as the primary policy; adopted as the terminating arm.**
CF-4 uses exactly this path at `T_FREEZE_MAX`, where the argument reverses: a
freeze longer than the coast budget genuinely *is* a loss of feedback authority,
and surrendering the output is then the correct response rather than an
overreaction. Reusing `MOTOR_FB_STALE` rather than inventing a fault state also
means the escalation inherits an already-validated recovery path.

### 4.4 The recommendation in one sentence

**Adopt Option B in the dead-reckoned-anchor form, terminating in Option C at
`T_FREEZE_MAX`, with Option A reachable as a one-constant rollback
(`DR_MAX_REV = 0`).**

## 5. Interactions to analyse before implementation

### 5.1 `MAX_DEVIATION` under a frozen anchor

**What it actually compares** (`fault_machine.cpp:361-376`):

```cpp
const float u0  = interp_base_pos(i);   // the LATCHED SETPOINT BASE, pre-clamp
const float enc = axes[i].pos_rev;      // the SAME cache the lead clamp reads
const float dev = u0 - enc;
if (fabsf(dev) > MAX_DEVIATION_REV) { estop = true; ... }
```

Three consequences, and the third is the one that de-risks the whole proposal:

1. **It reads the same frozen anchor, so it misreads in the same direction.**
   Under a freeze it measures a deviation that is not physically present.
2. **It compares `u0`, not the emitted command.** `u0` is the raw streamed
   trajectory knot from the Jetson, which **no option in §&nbsp;4 modifies**.
   Therefore *this contract does not change `MAX_DEVIATION`'s exposure at all* —
   its trip behaviour under a freeze is bit-identical before and after. This is
   worth stating loudly because the intuitive worry ("coasting commands will
   trip the deviation guard") is simply false: the guard never saw the clamped
   command in the first place.
3. **The one genuine interaction is a timing race**, and it constrains
   `T_FREEZE_MAX`. `MAX_DEVIATION` trips when `u0` has advanced
   `MAX_DEVIATION_REV = 1.0 rev` past the frozen anchor, i.e. after
   `≈ 1.0 / v_traj` seconds. `T_FREEZE_MAX` must fire first, or a content
   freeze escalates into a **latched** E-STOP requiring an operator
   `CLEAR_ERRORS` instead of a recoverable suppression. At `T_FREEZE_MAX =
   120 ms` this holds for `v_traj < 8.3 rev/s`, which covers the S1-measured
   move band (~2.5 rev/s) with a 3× margin but **not** the drive's 12.0 rev/s
   `vel_limit`. The guard also runs at `FAULT_TASK_HZ = 10 Hz`, adding up to
   100 ms of detection latency in its own favour. **No change to
   `MAX_DEVIATION` is proposed** — the constraint is discharged by choosing
   `T_FREEZE_MAX`, which is the cheaper and less invasive lever. Decision D2.

### 5.2 The recovery slew on re-enable

Site 4 (`leg_interp.cpp:502`) re-baselines `s_recover_pos[i]` to
`axes[i].pos_rev` on the `s_output_enabled` false→true edge. Under CF-4 that
edge is exactly a CF-2 release, so by construction the anchor has just changed
and is informative — no change required, but the ordering is load-bearing and
the implementation must preserve it (the release must be observable to the
fault task before the output gate re-opens, not after).

Site 5 (`leg_interp.cpp:519`) pins `s_recover_pos[i]` to the live encoder every
tick during a cold-start move. A freeze during a cold-start move pins the slew
state to a stale value — but the MPC leg TX is suppressed throughout that
window (`coldstart` gates the emit at `leg_interp.cpp:579`), so nothing reaches
the wire from this path and the pin re-converges on the next informative tick.
No change required; named so the next reader does not have to re-derive it.

Site 3 (`leg_interp.cpp:557`) re-runs the lead clamp on the slewed command.
This **must** route through the same anchor accessor as site 1, or the slew's
re-clamp will fight the dead-reckoned band and re-pin the command the primary
clamp just released.

### 5.3 The Mode-1 `s`-clamp's `vel_ff` hold — scoped OUT, and named

`leg_interp.cpp:394-411` saturates `s = dt / SEG_T` at 1.0. At `s = 1`:
`h01(1) = 1` and every other position basis function is 0, so `cmd_pos = p1`
and freezes; while `dh11(1) = 1` and every other velocity basis derivative is
0, so `cmd_vel = v1` and **holds**. The pair is internally inconsistent: the
ODrive is told to hold at `p1` and simultaneously to move at `v1`.

This is not a rare condition. Setpoints arrive at 40 Hz and `SEG_T` is 25 ms,
so `s` saturates on the jitter tail of essentially every segment — where the
hold is approximately *right*, since the trajectory really is near `v1` there.
It is badly wrong only for long gaps, and Mode 2/3 (extrapolation/decay) cannot
rescue it because they are gated on `!s_has_next`: with `u1` present, a stream
gap sits in the saturated `s`-clamp for up to `MPC_CMD_STALENESS_US = 250 ms`
rather than falling through to the decay ladder.

**Scoped out, deliberately, with a positive reason rather than an appeal to
boundaries.** This contract is *immunised* against it by CF-1(b) and the
Option-B accrual both keying on the command's own position increment rather
than on `cmd_vel`: with `cmd_pos` frozen at `p1`, the net displacement gate is
never met and `dr_offset` never accrues, which is the correct behaviour. The
`s`-clamp defect therefore neither corrupts this contract nor is corrupted by
it, and it has a different enforcement point (the Hermite branch, not the
clamp). Folding it in would make the single-enforcement-point property false
for both. **It is named here as a sibling work item** and should carry its own
logbook entry; the recommended sequencing is that it lands *before* the bench
validation of §&nbsp;8, so that the injected-freeze battery is not measuring two
defects at once.

### 5.4 The hand axis — same cache, different consumer

`axes[HAND_AXIS]` (index 6) is written by the same `decode_into_cache` path and
is therefore exposed to the identical freeze. It is **not** in scope, and the
reason is structural rather than jurisdictional: a grep of `hand_ops.cpp`
returns **zero** reads of `pos_rev`. The hand has no feedback-anchored 500 Hz
command authority in the bridge — its moves are dispatched as one-shot
trajectory commands — so there is no clamp to make content-aware.

The hand's exposure is real but lives elsewhere: the possession and arrival
judgements on the Jetson read hand position through the same cache, and
`logbook/2026-08-10-sensor-truth-possession.md` already sized its ARRIVAL
window around an uptime-dependent lag. Those consumers are out of scope for
this document and should be revisited when the root-cause arc closes.

### 5.5 `MOTOR_OVERSPEED` — a second blind guard, no change proposed

`fault_machine.cpp:344-347` tests `|axes[i].vel_rps| > MAX_MOTOR_VEL_RPS`.
`vel_rps` freezes bit-identically with `pos_rev`, so during a freeze the
overspeed guard is blind in the same way. This is named for completeness of the
class enumeration (§&nbsp;2). No change is proposed: the freeze-immune backstops
of §&nbsp;10.2 — the stroke clamp and the ODrive's own `vel_limit` — bound the
same hazard without an anchor.

## 6. Enforcement point and ISR access discipline

### 6.1 One function, three call sites, all inside the 500 Hz ISR

The contract's canonical enforcement point is a single static function in
`leg_interp.cpp`:

```cpp
// Returns the anchor the lead clamp shall use for leg i this tick, and
// updates the per-axis content-freshness state. THE single enforcement point
// for the content-freshness contract (plans/archived/lead-clamp-content-freshness.md).
static float clamp_anchor(uint8_t i, float cmd_pos_now);
```

It replaces the three bare reads at `leg_interp.cpp:460`, `:480` and `:557`.
Sites 4, 5, 6, 8 and 9 keep their bare reads; §&nbsp;2 records why.

### 6.2 Per-axis state

| Field | Type | Words | Purpose |
|---|---|---|---|
| `s_anchor_pos[i]`, `s_anchor_vel[i]`, `s_anchor_iq[i]` | `uint32_t` (bit patterns) | 3 | Previous tick's anchor triple, compared as integers |
| `s_anchor_change_us[i]` | `uint32_t` | 1 | Monotonic µs of the last content change (u32 is sufficient: every use is a bounded difference well under the ~71 min wrap, and unsigned subtraction is wrap-correct) |
| `s_cmd_at_change[i]` | `float` | 1 | `cmd_pos` at the last content change — CF-1(b)'s reference |
| `s_dr_offset[i]` | `float` | 1 | The dead-reckoned offset |
| `s_content_stale` | `uint8_t` bitmask | (shared) | Current per-leg state, published for telemetry |

Six words per leg × 6 legs = **36 words (144 B)** of new file-static state. The
prompt's "a couple of words per axis" is an underestimate and this document
records the real figure; it remains negligible against the Teensy 4.1's RAM and
adds no dynamic allocation.

Per-tick cost: three 32-bit loads, three integer compares, one float subtract
and compare, one conditional accumulate, per leg. No division, no
transcendental, no branch on floating-point classification.

### 6.3 The seqlock must NOT be used from the ISR — and this is a live hazard

`axis_state.h` provides `snapshot_pos_vel()`, a seqlock reader with a retry
loop. **It is unsafe to call from `interp_isr`, and calling it there would hang
the bridge.**

- `write_pos_vel()` runs on `task_can_rx`, a FreeRTOS *task*
  (`Teensy_code_canbridge.ino:504`), and bumps `seq` odd → writes → bumps even.
- `interp_isr` runs at NVIC `priority(16)`, deliberately **above** the FreeRTOS
  syscall ceiling of 32 (`leg_interp.cpp:632-658`), so it preempts every task
  unconditionally.
- If the ISR preempts the writer between the two `seq` bumps and then enters
  `snapshot_pos_vel`'s `while ((s0 & 1) || (s0 != s1))`, the loop **cannot
  terminate**: it waits for a writer that cannot run because the ISR holds the
  core. Result: a hard hang, ended by the watchdog.

Today this hazard is avoided only by *who happens to call it* — the sole caller
is `telemetry.cpp`, on `task_telem`. That is an accident of the current call
graph, not a stated rule. **This proposal includes adding that rule as a
comment at `axis_state.h`'s `snapshot_pos_vel` definition**, since the contract
would otherwise be one careless refactor away from a hang.

### 6.4 The access discipline the detector actually uses

`clamp_anchor` reads the three fields as **three independent single-word
volatile loads**, exactly as the existing clamp reads `pos_rev` today. Each
load is atomic on Cortex-M7; what is *not* guaranteed is that the three come
from the same writer generation — the ISR can land between the writer's stores
and read a skewed mix.

**Skew is one-directional and fails safe.** A skewed read can only make the
detector see a *change* that did not atomically occur; it can never make it see
a *freeze* that did not occur, because seeing a freeze requires all three words
to match the previous tick, which a partial write cannot manufacture. A
spurious change releases the content-stale state (CF-2), returning the clamp to
its present-day behaviour — the conservative direction. This is the argument
that makes the bare-load discipline correct here and it must be preserved
verbatim as a code comment.

No new IRQ-masked region, no new critical section, no FreeRTOS call — the
ISR's above-the-syscall-ceiling contract (`leg_interp.cpp:646-655`) is
preserved unchanged, as the determinism rule requires.

## 7. Telemetry — loud, never silent

### 7.1 Per-axis occupancy census (firmware)

Following the FW 11 occupancy idiom already in `leg_interp.cpp:101-125` —
**cumulative since boot, never cleared by a reader, the emitter differences two
consecutive reads** — which is what makes it race-free without masking:

| Counter | Type | Meaning |
|---|---|---|
| `s_content_stale_ticks[i]` | `uint32_t` | Ticks spent content-stale, per leg → duty when differenced against `s_tick_count` |
| `s_content_stale_episodes[i]` | `uint32_t` | CF-1 declarations → mean episode length = ticks / episodes |
| `s_content_stale_max_us[i]` | `uint32_t` | Longest episode since boot → the tail |
| `s_dr_offset_max_x1000[i]` | `int32_t` | Peak `dr_offset` seen, in milli-rev → the CF-3 margin, measured |
| `s_content_escalations[i]` | `uint32_t` | CF-4 escalations into `MOTOR_FB_STALE` |

Each is a single naturally-aligned word with the ISR as sole writer, so no
PRIMASK guard is needed — the same justification the existing occupancy
counters carry.

### 7.2 Wire surface

Everything below is authored in **`config/generate_udp_protocol.py`** — the
single source for `MsgType`, `PROTOCOL_VERSION` (currently 5) and every payload
layout — and generated into `config/generated/udp_protocol.{h,py}` plus the
delivered firmware copy `Teensy_code_canbridge/udp_protocol.h`. **No generated
file is hand-edited.** The conformance gates are
`tests/firmware/test_udp_protocol_xlang.py`
(`test_committed_cpp_matches_generator`, `test_committed_python_matches_generator`,
`test_delivered_copies_match_canonical`, `test_protocol_version_frozen`,
`test_wire_layout_frozen`) and `tests/firmware/test_bridge_fw_version_xref.py`.

**(a) The at-a-glance signal — one `HeartbeatT2J` flag bit, 10 Hz.**
`lead_clamp_anchor_stale`, sitting beside the existing `lead_clamp_mask` and
`live_deviation[6]` fields the heartbeat already carries
(`Teensy_code_canbridge.ino:138-150`). The flags field is `u32` with **bits 6,
7 and 14–31 free** (0–3 link/stow/heartbeat/mpc, 4–5 cone bus health, 8–13
torque-clamp mask). This costs **no payload-size change and no
`PROTOCOL_VERSION` bump** — the documented reason the field is `u32` on purpose,
and the precedent set by `TORQUE_CLAMP_MASK`. It renders into `/link_status`
alongside the existing `max_dev_*` rows (`teensy_bridge_node.py:3468-3502`),
with the standing caution that a renderer which raises takes the whole
`/link_status` message down.

**(b) The census — decision required between two shapes.** The §&nbsp;7.1 arrays
are ~5 × 6 × 4 B of per-axis counters and do not fit in a flag bit.

| Shape | Cost | Failure mode on a version skew |
|---|---|---|
| **Extend `CACHE_DIAG` 0x91** (already 1 Hz, already per-axis `u32` arrays, already published to `/cache_diag`, already *the* cache-freshness frame) | Changes a **flown** payload's size (129 B, `static_assert`-pinned); `test_wire_layout_frozen` must be updated deliberately | **Loud** — a mismatched decoder fails to unpack rather than staying quiet |
| **New `MsgType` 0x92** (next free above `CACHE_DIAG`) | A new id and a new topic; mirrors the 0x91 precedent, which itself opened a new block because 0x81–0x8F is full | **Silent** — an unaware decoder ignores the unknown id, so a stale Jetson looks healthy |

The **recommendation is (b)-new-id**, on the additive discipline that the
0x8D / 0x8E / 0x8F / 0x91 sequence has established four times: additive means no
`PROTOCOL_VERSION` bump, an unaware decoder ignores the id, and a never-seen
frame renders **empty rather than erroring**. The loud-failure argument for
extending 0x91 is genuinely attractive and is recorded rather than dismissed —
but it trades a *decode outage* for a *diagnostic gap*, and a decode outage on
`/cache_diag` would take out the instrument the root-cause arc is currently
depending on. Either way the `FW_VERSION` bump is confirmed on the
`BRIDGE_IDENTITY` frame, never inferred from a healthy link.

`CLOCK_DIAG` 0x8F is deliberately **not** the home, despite already carrying the
interp occupancy census (`interp_ticks` / `recover_slew_ticks` /
`extrap_ticks`): it is emitted once per accepted time-of-day anchor (~30 s in
steady state), which is far too coarse for an episode counter whose whole
purpose is to resolve 100–150 ms events.

**(c) Bagging is not optional.** The new topic and `/link_status` must both be
in the launch file's record list **in the same commit**. The
`/profile`-not-bagged gap that S1 had to work around is the precedent, and P0
of the parent arc already had to close the equivalent gap for
`/leg_cmd_executed`.

### 7.3 Alarm criterion for the P3 monitor

S1's method correction (c) is the binding constraint here and it must be quoted
into the monitor's design: *"P0's `/leg_cmd_executed` saturates as a
transport/execution discriminator exactly when the clamp pins"* — once the
clamp is engaged the executed command is a deterministic function of the stale
encoder, so **a lag-only monitor reads healthiest precisely when the loop is
most degraded**. The same trap applies to `live_deviation`
(`Teensy_code_canbridge.ino:145`), whose under-reporting under a pinned clamp
S1 formally withdrew a prior claim over.

Proposed alarm inputs, in priority order:

1. **`s_content_escalations` delta > 0 in any window** — a CF-4 escalation is
   never normal. Alarm immediately.
2. **Per-leg content-stale duty over a 10 s window** exceeding a threshold
   baselined from a *fresh* plant, not guessed. The fresh-plant reference
   already exists in kind: lead-clamp duty 0.058 fresh against 0.443–0.739
   aged.
3. **`s_content_stale_max_us` growth across a session**, tagged with
   `uptime_ms` per the arc's standing rule.

Every sample carries `uptime_ms`. The threshold is set from measurement, not
from a guess, per the P3 phase's own wording.

## 8. Bench validation plan

### 8.1 Proving the fallback without waiting for the fault

The fault is uptime-dependent and takes 16–24 h to reproduce, which is far too
slow a loop to validate a control-path change against. Three vehicles, in
increasing cost:

**(1) Native host harness — the regression gate.** `tests/firmware/native/`
compiles the firmware for the host with `g++` via `build.py` (no CMake; hash-cached
artefacts in `temp/firmware_native/`), driven by the pytest wrapper
`tests/firmware/test_native_firmware.py` and pre-built by `run_tests.sh:209-220`.
It carries **no marker**, so it runs in the default per-commit gate — which
`pyproject.toml` mandates for firmware natives as part of the hardware-safety
surface that is never demoted to `nightly`.

The seam already exists and is exactly the one this contract needs:
`test_leg_interp.cpp` **`#include`s `leg_interp.cpp`** rather than linking it, so
a test can reach the file statics and invoke the `static interp_isr()` directly.
The clamp math is already pinned there by
`TEST_CASE("lead clamp: bounds position, KEEPS vel_ff (capped), sets the clamp
mask")` (`test_leg_interp.cpp:77`), which sets `axes[0].pos_rev` by hand and
asserts the emitted `target_pos_rev` — i.e. **the fixture already controls the
anchor**, and a driven-freeze case is an extension of an existing pattern rather
than new machinery. `test_fault_machine.cpp:446` and `:535` do the same for
`MAX_DEVIATION` and its latch.

**Two gaps in the existing coverage that this work should close**, both found
while mapping the harness:

- `teensy_link/fault_logic.py` — the Python host mirror pinned to
  `native/fault_golden.json` — contains **no deviation logic at all**.
  `MAX_DEVIATION` is asserted only by the compiled test, never by the golden
  mirror, so the mirror cannot catch a divergence in the guard this contract's
  CF-4 escalation nests inside.
- `MAX_DEVIATION_REV = 1.0` is a hand-authored firmware constant with **no YAML
  key and no drift-test pin** — neither `tests/firmware/test_config_drift.py`
  nor `test_firmware_build_pins.py` covers it. Every constant this contract
  introduces should be pinned where those are not, so the nesting argument of
  §&nbsp;5.1 cannot silently rot.

**(2) Offline replay of the S1/S2 bags.** Feed a recorded aged-plant
`/leg_setpoint_echo` + `/robot_state` pair through the native interpolator and
compare the emitted command under present-day and contract behaviour. This is
the production-faithful-replay convention, and it answers the one question the
synthetic fixture cannot: *does the contract engage on the real fault, at the
real onsets?* A reusable harness belongs in `tools/probes/`, outputs under
`temp/probes/`.

**(3) On-hardware injection — the proof.** A compile-time-gated debug path in
`can_buses.cpp::decode_into_cache` that, for a nominated axis and duration,
**re-writes the cached values unchanged with a fresh timestamp**:
`write_pos_vel(a, a.pos_rev, a.vel_rps, micros64())`. Re-writing rather than
skipping the write is essential — skipping would also stall
`pos_timestamp_us` and trip the *existing* `MOTOR_FB_STALE` at 150 ms,
reproducing a different fault from the one under test.

**Two hard constraints on the injection build**, both derived from this
project's own history rather than from caution in the abstract:

- It is **compile-time gated and OFF in the shipping image** — the
  bench-sysid-firmware rule ("never flash the bench-sysid firmware to the
  robot") exists because a diagnostic build reaching the robot is a real,
  already-experienced failure.
- The injection build **must be identifiable on the wire**, via a flag on the
  `BRIDGE_IDENTITY` frame. The FW 9 → 10 → 11 → 12 sequence has established
  four times over that wire-invisible firmware means *a healthy link is not
  evidence of which build is aboard*. An injection build that cannot be
  distinguished from the shipping build by telemetry is exactly the hazard that
  discipline exists to prevent.

### 8.2 Regression tests

- **Native harness (default gate):** CF-1 declaration boundary in both
  directions (29 ms does not declare, 31 ms does); rest lawfulness (anchor
  repeating for 5 s at zero commanded displacement never declares); the
  `s`-clamp interaction (`cmd_pos` frozen at `p1` with `cmd_vel = v1` never
  declares); CF-2 edge release on each of the three triple members
  independently; CF-3 bound asserted on every emitted sample of every case;
  CF-4 escalation timing against `MOTOR_FB_STALENESS_US`; `dr_offset` sign
  correctness across a trajectory reversal mid-freeze; `DR_MAX_REV = 0`
  reducing exactly to present-day behaviour (the rollback path is a *tested*
  path, not a hoped-for one).
- **Codec round-trip** for the new `MsgType`, plus the standing
  wire-compatibility property (unknown id ignored, `PROTOCOL_VERSION`
  unchanged), following the `test_protocol_codec.py` pattern.
- **Launch-file assertion** that the new topic is in the record list.
- Ephemeral ports and `tmp_path` only, per the parallel-by-default rule.
- `./run_tests.sh --full` before any hardware sitting and at phase closure.

### 8.3 Acceptance criteria

| # | Condition | Criterion |
|---|---|---|
| A1 | **Fresh plant, injection off** | Zero measurable behaviour change across a standard battery: per-leg lead-clamp duty, end-to-end lag, and deviation p95 all inside the run-to-run band of the FW 12 fresh baseline. **And** the content-stale occupancy counters read ≈ 0 — proving the contract is *dormant* on a healthy plant rather than merely harmless. |
| A2 | **Injected 150 ms freezes at move onset, one axis** | No commanded stop — the emitted command's increment must not cross zero against the trajectory direction for the duration. The CF-3 bound `\|cmd − true encoder\| ≤ MAX_LEAD_REV + DR_MAX_REV` holds on every sample. No `MAX_DEVIATION` latch. Telemetry loud: episode counted, flag raised, `dr_offset_max` non-zero. |
| A3 | **Injected 300 ms freeze** (beyond `T_FREEZE_MAX`) | CF-4 escalates into `MOTOR_FB_STALE`; output suppression and the recovery slew behave as they do today; no latched E-STOP; escalation counter increments. |
| A4 | **Injected freeze with the leg mechanically restrained** | The bind-plus-freeze case of §&nbsp;4.2, run at low commanded velocity and with the current limit as the witness. This is the case D1 is about and it must be run *deliberately*, not left to chance. |
| A5 | **Aged plant (16–24 h soak), injection off** | The clamp-attributable share of the lag is removed. **Stated as a falsifiable prediction, not a promise:** if the mechanism model is right, aged end-to-end lag should approach the fresh class, because the commanded stops were the lag. If it does not, the model is wrong and this contract should be reconsidered rather than tuned. |

**A5 is not the arc's ≤ 20 ms criterion.** This contract is a *mitigation of the
consequence*, not a fix of the cause; the cause is owned by the root-cause arc
(§&nbsp;9). Conflating the two would let a successful mitigation close an
investigation that is still open.

## 9. Explicitly out of scope

- **The underlying stale-content root cause.** Owned by the FW 13 root-cause
  arc, which the S2 result narrowed to a binary — wire-side duplicate frames
  versus Teensy-side stale-ring re-reads, with FlexCAN_T4's ring bookkeeping
  the prime suspect and an RX-timestamp discriminator designed. This contract
  must not be presented, in any artefact, as having closed that arc; it makes
  the *consequence* survivable, which is a different claim.
- **The trajectory/status `max_emit_gap_ms` blind-monitor gap** — a sibling work
  item, named here so it is not lost.
- **The Mode-1 `s`-clamp `vel_ff` hold** (§&nbsp;5.3) — sibling, sequenced
  before §&nbsp;8's bench work.
- **The hand axis** (§&nbsp;5.4), **the cold-start move planners** and **the
  stow descent base** (§&nbsp;2, sites 6 and 8).
- **`MAX_DEVIATION`, `MOTOR_OVERSPEED` and `live_deviation`** — all confirmed
  blind to a content freeze (§§&nbsp;5.1, 5.5, 7.3), all deliberately left
  unchanged, all with the reason recorded rather than implied.
- **The shared UDP `drain_cap_hits` / `seq_gaps` counters**, still never
  uplinked (S1 Open Question 6) — deferred to P3.

## 10. Risk register

### 10.1 What this change could break

| Risk | Severity | Mitigation / bound |
|---|---|---|
| **Bind-plus-freeze coincidence** raises the worst-case P-term demand from 4 rev/s to a `vel_limit`-saturated 12 rev/s for up to `T_FREEZE_MAX` | **High — the headline risk** | `DR_MAX_REV` (D1); CF-4 escalation at 120 ms; the freeze-immune stroke clamp; the ODrive current limit; A4 tests it deliberately |
| **False CF-1 declaration at low commanded speed** from encoder quantisation opens the band during slow precision moves (catch approach, levelling) | **High** | The 0.67 rev/s analytic floor (§&nbsp;3, CF-1); **D3 measures it before implementation** |
| A `T_FREEZE_MAX` above the `MAX_DEVIATION` crossing time converts a freeze into a **latched** E-STOP at high trajectory speed | Medium | §&nbsp;5.1 constraint; D2 |
| Detector state diverges from the clamp it guards if a future refactor adds a fourth anchor read | Medium | Single accessor (§&nbsp;6.1); a native test that fails if any `axes[i].pos_rev` read appears in the clamp path |
| `snapshot_pos_vel` called from ISR context by a later change → hard hang | Medium | §&nbsp;6.3 comment; the rule stated in `axis_state.h` rather than left implicit |
| Telemetry added to a flown frame breaks an unaware decoder | Low | New `MsgType`, additive discipline (§&nbsp;7.2) |
| The contract masks the root cause and the FW 13 arc loses urgency | **Medium, and organisational rather than technical** | §&nbsp;9's framing; CF-5's loud telemetry makes residual freezes *more* visible, not less; A5 stated as falsifiable |

### 10.2 The actual backstop chain, verified

`CLAUDE.md` states: *"Leg-path safety authority is the Teensy-side
`MAX_DEVIATION` guard, in can-bridge firmware. … Nothing on the Jetson is in
that safety loop."* **Verified, and the verification is worse than the claim:**

- The can-bridge `MAX_DEVIATION` guard reads `axes[i].pos_rev` — bit-for-bit
  the same `volatile float` the lead clamp reads, neither through the seqlock.
- `Teensy_code_platform/` contains **zero** occurrences of `deviation` in any
  form. The platform Teensy has no deviation guard at all and is not in the leg
  loop.
- `motor_guard.py` is a parked fallback on the pre-cutover path and is not
  launched; its own guard is content-blind in the identical way (§&nbsp;2).

So **the backstop for this change lives on the same Teensy, reading the same
cache, and is subject to the same blindness.** There is no independent
anchor-side backstop anywhere in the system. What saves the design is that the
bounds which *are* independent are position- and current-domain rather than
anchor-domain — the table below.

The chain, in emit order, with anchor dependence marked:

| Layer | Bound | Reads the frozen anchor? |
|---|---|---|
| 1. Lead clamp | `±MAX_LEAD_REV` (+ `DR_MAX_REV` under this contract) | **Yes** — the subject |
| 2. **Stroke clamp** (`leg_interp.cpp:471-486`) | Absolute `STROKE_MIN/MAX_REV`; zeroes `vel_ff` and `torque_ff` on engagement | **No — freeze-immune** |
| 3. `vel_ff` cap | `±3.5 rev/s` | No |
| 4. `torque_ff` ingest clamp | `±0.25 wire-Nm` | No |
| 5. `MAX_DEVIATION` E-STOP, 10 Hz | `1.0 rev` on `u0` vs anchor, latched | **Yes** — misreads |
| 6. `MOTOR_FB_STALE`, 10 Hz | 150 ms timestamp age, recoverable | Timestamp only — **blind to content** today; CF-4 fixes this |
| 7. `MOTOR_OVERSPEED` E-STOP | `16.5 rev/s` on `vel_rps`, latched | **Yes** — misreads |
| 8. ODrive `vel_limit` / current limit | 12.0 rev/s / 10 A | **No — the only layer entirely independent of the Teensy cache** |

**Layers 2, 3, 4 and 8 are freeze-immune and bound every option in §&nbsp;4.**
That is the material de-risking finding: even a fully unbounded coast could not
command a leg outside its physical stroke, because the stroke clamp runs *after*
the lead clamp and reads no anchor at all. The proposal's residual exposure is
therefore confined to *velocity and current demand within the stroke*, which is
exactly what `DR_MAX_REV` and layer 8 bound.

## 11. Decisions required

**D1 — `DR_MAX_REV`, the safety-envelope decision.** The credit ceiling trades
coast coverage directly against the worst-case P-term sprint under a
bind-plus-freeze coincidence. `0.20 rev` covers 80 ms at 2.5 rev/s and permits
a `vel_limit`-saturated 12 rev/s demand; `0.05 rev` holds the demand to 6 rev/s
and covers only 20 ms; `0` reduces the design to Option A (HOLD) exactly. An
alternative parameterisation expresses the ceiling in the *velocity* domain —
`band_max = V_DEMAND_MAX / pos_gain`, with `pos_gain = 40` config-pinned — so
the knob is the physical quantity of concern and auto-adjusts if the gain ever
moves. Note that the 2026-07-10 forensics chose `MAX_LEAD_REV = 0.10`
specifically so `pos_gain × LEAD = 4.0` stayed below the then-`vel_limit`; that
limit has since moved 4.0 → 6.0 → 12.0, so the original constraint no longer
binds the way it did and the reasoning must be redone rather than inherited.
**This is a physical-intuition question about the machine and belongs to the
owner.**

**D2 — `T_FREEZE_MAX` against the `MAX_DEVIATION` crossing.** 120 ms is safe up
to 8.3 rev/s of trajectory velocity but not to the drive's 12.0 rev/s
`vel_limit` (§&nbsp;5.1). Options: accept it, on the grounds that measured moves
run at ~2.5 rev/s; lower `T_FREEZE_MAX`, at the cost of escalating on typical
freezes; or make it velocity-scaled. The third is more machinery for a case that
may never occur.

**D3 — the measurement that gates the whole design.** The false-positive rate
of a bit-identical detector at low commanded velocity is **unmeasured on this
plant**, and S1 formally withdrew its "27–58 % identical consecutive samples"
figure precisely because it pooled low-speed samples. Required before
implementation, and available offline from existing S1/S2 bags at no hardware
cost: **the distribution of bit-identical anchor-triple run-lengths against
`|v_cmd|` on a fresh plant.** If healthy run-lengths reach 30 ms at commanded
velocities near 0.67 rev/s, `T_FREEZE_MIN` and `D_TRIG` are wrong and the
detector needs a different discriminator. The same analysis should confirm the
true `get_encoder_estimate` broadcast period from FW 12's `enc_frames`, which
every constant in §&nbsp;3 is implicitly scaled against.

**D4 — the S2 logbook entry is a prerequisite.** Every S2 number in §&nbsp;1.1
is currently uncommitted. The contract must not be approved against an
unwritten measurement.

**D5 — plan-status vocabulary.** See the frontmatter note.

## 12. References

- `logbook/2026-08-12-s1-aged-bridge-isolation-teensy-internal.md` — the
  four-arm isolation, the exoneration chain, the refresh-stall tail table, the
  clamp-onset statistics, and method correction (c).
- `logbook/2026-07-18-teensy-uptime-tracking-degradation.md` — the parent
  investigation (**open**), the seven-bag lag table, and the 2026-07-24
  two-deliverable closure contract.
- `plans/archived/bridge-temporal-trustworthiness.md` — §§ S1 RESULTS, S2, P3, and
  the coupling insight. This document is a candidate component of **P3**, whose
  scope that plan deliberately left unspecified until S1 localized the drift.
- `logbook/2026-08-12-fw12-cache-diag-instrumentation.md` — `CACHE_DIAG` 0x91,
  the additive-`MsgType` discipline, and `enc_frames`.
- `logbook/2026-08-12-fault-machine-staleness-guard-atomic-read.md` — the most
  recent change to the guard CF-4 escalates into.
- `logbook/2026-07-16-max-deviation-guard-tracking-lag.md` — the
  `MAX_DEVIATION_REV` 0.5 → 1.0 raise, measured mid-curve; its envelope numbers
  remain contingent on an uptime-degraded plant.
- `controller/REFERENCE_LAYER_CONTRACT.md` — the house model for a contract
  document: normative invariants, one enforcement point, one test.
- `ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp:443-468` — the lead
  clamp and its 2026-07-10 forensics comment.
- `ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp:338-459` —
  `evaluate_guard`, containing `MAX_DEVIATION`, `MOTOR_FB_STALE`,
  `MOTOR_OVERSPEED` and the output gate.
- `ros_ws/src/jugglebot/Teensy_code_canbridge/axis_state.h` — the cache, the
  seqlock, and the present-axis contract this document's enforcement-point
  structure mirrors.

---

## Archival note (2026-08-15)

**Archived SUPERSEDED, never implemented. Nothing in this document ever reached
the leg control path, which is exactly what the draft's own review gate was for.**

**Why it closed.** Its premise — that the fault is a **content freeze** the
memoryless lead clamp mistakes for truth — was overturned by the 2026-08-14
RX-ring audit and then by measurement. The real mechanism was a **delay line**
(the FlexCAN_T4 `_available` one-way leak), under which content changes on
schedule and is merely *old*, so CF-1's bit-identical detector keys on the wrong
invariant; and ~97 % of the freeze population the detector was designed against
turned out to be a Jetson-side `/robot_state` latest-wins artefact, not a Teensy
phenomenon. The draft's own **D3** gate — the measurement it named as able to
invalidate itself — was run and did exactly that: there is no `T_FREEZE_MIN` that
both spares a healthy plant and fires before the anchor is a full clamp budget
stale. **The detector detects; it does not protect.**

**Why it is not merely deferred.** FW 14 removed the cause, and the consequence
went with it: end-to-end lag is 10–20 ms with **lead-clamp duty 0 on every move**
at 5.8 h and 15.2 h of uptime (`logbook/2026-08-15-fw14-validated-arc-closed.md`).
The § 9 framing was right — this document was a *mitigation of a consequence*, and
the consequence did not survive its cause's removal.

**Where the salvage lives: `plans/active/leg-bus-frame-drops.md`.** The validation
battery found a *different*, real input to the same amplifier — per-axis leg-bus
frame drops, gated by the 500 Hz setpoint stream rather than by uptime, present on
fresh firmware too. The crucial reframing that makes the salvage usable: under the
delay line the timestamps were **fresh** and only the content was stale, but a
genuine dropout means **no cache write happens at all**, so `pos_timestamp_us`
really does age — **a timestamp-age-aware clamp works there where a
content-freshness detector could not work here.** Carried forward:

- §§ 2, 5, 6 and 10 — the enforcement-point enumeration, the
  `MAX_DEVIATION`/stroke-clamp/`MOTOR_OVERSPEED` interaction analysis, the ISR
  access discipline (bare single-word loads only; `snapshot_pos_vel`'s seqlock
  retry **would hang the bridge** if called from `interp_isr`), and the verified
  backstop chain;
- the **velocity-extrapolated anchor** (`pos + vel·Δt`), measured on fresh-plant
  data at anchor-error p95 **0.129 → 0.064 rev** and over-budget freezes
  **0.160 → 0.000** (n = 25) — the right *shape* for any clamp hardening;
- the **T = 100 ms / 0.67 rev/s content-hold threshold as a REPORTING criterion**
  (0.28/min healthy vs 4.75/min aged, 17×), which shipped as calibration input to
  the P3 latency monitor.

**Two live cautions for any rework.** § 7's "new `MsgType` 0x92 (next free)" is
**stale** — 0x92 was claimed by FW 13's `RING_DIAG`; renumber to 0x93+ or recorded
bags will mis-decode. And the frontmatter's `status: proposed` was out of the
`DOCUMENTATION_GUIDE.md` § 2.6 vocabulary; it is resolved here by archiving as
`superseded`, which leaves the vocabulary question (D5) open and unforced.
