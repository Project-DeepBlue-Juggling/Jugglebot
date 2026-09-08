---
title: Hand geometry correction — the flown-position audit that withdrew "hand_stroke_mm is measured"
type: investigation
date: 2026-09-08
status: G2 landed on branch (hand-geometry-correction, 2a25b98) — G3 not flown
phase: hand-geometry-correction — G2
related_plan: hand-geometry-correction.md
files_changed:
  - config/hardware_config.yaml
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/toss_release.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - sim/hand/trajectory.py
  - sim/model/generate_mjcf.py
subsystem:
  - motion
  - sim
  - config
tags:
  - kinematics
  - safety
  - testing
---

## Summary

Read-only audit (G1 of `hand-geometry-correction.md`, no code/config/test/doc
edited). The owner measured the hand's cable-drum stroke on the bench
(2026-09-06: 352 mm between hard stops, bottom stop −0.107 rev, top stop
10.701 rev) and asked whether `linear_gain_factor` or `hand_spool_radius_m`
carries the resulting +2.97 % gain error, and whether `hand_stroke_mm`
(documented as "a measured fact") should become the new 352 mm. The audit
answers both, classifies all 24 config/code sites the gain touches by
whether the change moves a **commanded rev** or only a **label**, and writes
the operator bench checklist (G3) the owner requires flown *before* G2 ships.
This entry carries the G1 findings (folded in here from the session
scratchpad per the plan — see the full 24-row table under "The
flown-position table (full)" below) plus G2's software-correction status.

## Symptoms

`teensy_trajectory.linear_gain_factor: 1.035` and
`jugglebot_geometry.hand_stroke_mm: 344.75` were both long-standing constants;
the latter's own YAML comment called it "the hand's PHYSICAL travel between
hard stops {mm} — a measured fact," derived in-comment as
`(10.8 + 0.1) × 31.6284 = 344.7496`. The 2026-09-06 bench measurement (352 mm
stop-to-stop) disagreed with that "measured fact" by 2.1 %, and the gain it
implies (32.5685 mm/rev) disagreed with the shipped gain (31.6284 mm/rev) by
2.97 % — which is also, within measurement noise, the excess release velocity
the unified-cycle first-hardware-cycles sitting had already flagged
(2026-09-06 bag: every throw 11–15 % too fast).

## Diagnosis

**The gain.** From the bench measurement:

```
span   = 10.701 − (−0.107)      = 10.808 rev
mm/rev = 352.0 / 10.808         = 32.5685 mm/rev
rev/m  = 10.808 / 0.352         = 30.7045 rev/m
```

against the shipped `LINEAR_GAIN = linear_gain_factor / (2π·hand_spool_radius_m)
= 1.035 / (2π·0.00521) = 31.6172 rev/m`. **The machine has 2.97 % more travel
per commanded rev than the model believes**, and every commanded rev/s
produces 2.97 % more linear speed — the direct source of the excess release
velocity.

**Which knob is wrong.** `hand_spool_radius_m: 0.00521` is a measured part
(the physical spool) and should not move. `linear_gain_factor: 1.035` is
documented only as a "just 'cuz" fudge — the wrong part. Solving for the
factor that reproduces the measurement at the measured radius:

```
factor = 30.70454 × 2π × 0.00521 = 1.005125  →  1.0051 (4 dp)
```

The residual against the exact measurement is 0.0008 mm/rev, three orders of
magnitude below the measurement's own ~1 mm (0.3 %) resolution — 4 dp is the
right precision, not false precision.

**What 1.0051 physically means, and why it convicts the old 1.035.**
`factor / (2π·r)` is algebraically `1/(2π·r_eff)`, so `r_eff = r/factor`:

- **new:** `r_eff = 5.21 / 1.005125 = 5.183 mm` — 0.027 mm (0.51 %) under the
  bare 5.21 mm spool, the sign and rough magnitude expected of a cable that
  does not wrap on the bare drum (about half a cable diameter).
- **old:** `1.035` implies `r_eff = 5.034 mm` — 3.4 % under the drum, which
  is physically implausible for this cable. **That is the tell that 1.035 was
  never a geometry constant.** 1.0051 is a measured cable/spool calibration;
  1.035 was a fudge wearing a gain factor's clothes.

**`hand_stroke_mm` — withdrawn premise.** The owner's premise that this key
is "a measured fact" does not survive: its own YAML comment derives it
arithmetically from the *old*, wrong gain and the *old* hard-stop reading,
which makes it an inference, not a measurement. Two candidate numbers exist
and they are not the same thing:

| Reading | Definition | Value |
|---|---|---|
| stop-to-stop | bottom stop → top stop, 10.808 rev | **352.000 mm** (direct 2026-09-06 measurement) |
| above encoder zero | rev 0 → top stop, 10.701 rev | **348.524 mm** = 10.701 × 32.5685 |

They differ by the 0.107 rev (3.48 mm) of travel that exists *below* the
firmware's homing zero (the firmware homes onto the bottom stop and sets
encoder zero 0.1 rev above it — the 2026-09-06 −0.107 rev bottom-stop reading
independently confirms that offset). **Decision: `hand_stroke_mm: 352.0`**
(the honest stop-to-stop measurement, the key's own docstring's definition,
and 352.0's direct like-for-like successor to the old 344.75, which was also
stop-to-stop). The 3.48 mm difference is a **frame** question, already owned
by the 20 mm frame divergence documented at `cup_realize.py:76-89` — folding
it into the stroke number would bury it again.

**The MJCF clip is anchored at encoder zero, not stop-to-stop.**
`sim/model/jugglebot.xml`'s `hand_slide` joint range and `act_hand`
ctrlrange are single-sided `[0, hand_stroke_mm/1000]` from encoder zero. If
that consumer runs against the stop-to-stop 352.0, the modelled hand gets
3.48 mm of travel above zero the machine does not have. **Decision (D2):**
`generate_mjcf.py` should emit the derived travel-above-encoder-zero
(`hard_stop_rev / gain = 10.701 / 30.703768 = 0.348524 m`) instead of the raw
`hand_stroke_mm` key, making the clip a *consequence* of the measured stop
rather than a second inference riding the same key.

**The flown-position table — the governing question is "does the commanded
rev move?"** With `cup_z_level = 679.6 + rev × mm_per_rev` (679.6 mm is a
fixed, gain-independent anchor: `CUP_Z_BASE_MM` 659.6 + `SLIDER_REV_ZERO_MM`
20.0), a constant stored **in rev** is a label change under the gain
correction; a constant stored **in mm/cup-z** is a **command** change
(−2.888 % in rev, since 1/1.029748 − 1 = −0.02888). Twenty-four sites were
classified by file:line — full table below, “The flown-position table
(full)”. The rows that matter:

| Position | Stored as | Commanded rev moves? |
|---|---|---|
| x2 legacy release, x3 catch prime / stroke top, x5 legacy catch | rev (via `hand_stroke_m` basis) | **LABEL only, conditional on re-basing `hand_stroke_m`** (see below) |
| `SETTLE_CUP_Z_MM` (689.6 mm) | cup-z mm | COMMAND −0.0091 rev = −0.30 mm |
| `_UNIFIED_THROW_CUP_Z_MM` (860.0) / `_UNIFIED_CATCH_CUP_Z_MM` (830.0) | cup-z mm | COMMAND −5.37 mm / −4.47 mm unless re-set to the physically-flown 865.37 / 834.47 (**owner decision D1**) |
| `TILT_ACCEL_LIMIT_DEFAULT_RAD_S2` | derived | BEHAVIOUR −1.96 % (tighter cap) |
| cup operating band | derived | BEHAVIOUR +2.97 % (runway grows) |
| `HAND_THROW_OFFSET_MM` (58.044) | derived, NOT hardcoded as the owner's note assumed | BEHAVIOUR +5.564 mm (release plane was 5.56 mm too low) |
| `HAND_MAX_DECEL_MPS2` | derived | BEHAVIOUR +2.97 %; rev/s² limit (owner-signed) unchanged |
| coast ladder v-axis | measured table | see below — a decision, not a mechanical edit |

### The flown-position table (full)

All 24 sites the gain touches, classified by file:line — folded in here from the G1 session scratchpad (`geometry_audit.md`, not a repo file) per the plan, so this entry is the only durable copy. Rows 6/7 (D1),
15/16/23 (owned by the FW 18 unit, already landed on `mvp-trajectory-bringup`) and 19 (`hand_stroke_mm` 344.75 -> 352.0) reflect the decisions this correction actually landed with; the table otherwise records the audit's as-measured state.

Gain constants used throughout: old 31.6284 mm/rev (31.6172 rev/m), new
**32.5693 mm/rev** (30.7038 rev/m), ratio **1.029748**.

### Table

| # | Position | file:line | Stored as | Value | mm @ OLD | mm @ NEW | Commanded rev moves? |
|---|---|---|---|---|---|---|---|
| 1 | `x2` legacy release (`HAND_THROW_POS_M`) | `config/generated/` (derived, `generate_config.py`); consumed `toss_release.py:55` | m (profile) | 0.187044 m = **5.9138 rev** | 187.04 | **192.61** | **LABEL** if `hand_stroke_m` is re-based (recommended); **COMMAND −0.171 rev** if not |
| 2 | `x3` stroke top / catch prime | `hand_stroke.py:65-73` (`STROKE_TOP_REV`); YAML override `hardware_config.yaml:559` | rev (override) + m (derived) | **9.9594 rev** | 315.00 | **324.37** | **LABEL** if re-based; **COMMAND −0.288 rev** if not |
| 3 | `x5` legacy catch point | derived in `generate_config.py` | m (profile) | ~6.129 rev | ~193.8 | **~199.6** | same as x2 |
| 4 | `JB_OP_HAND_CATCH_PRIME_REV` | `hardware_config.yaml:559` | **rev** | 9.9594 | 315.00 | **324.37** | **LABEL** — it is already in rev |
| 5 | `SETTLE_CUP_Z_MM` | `unified_cycle.py:196-198` | cup z mm (derived: `679.6 + 10.0` inset) | **689.6 mm** | rev 0.31618 | rev **0.30704** | **COMMAND −0.0091 rev = −0.30 mm** (hand settles 0.30 mm lower; was physically 689.90, becomes 689.60) |
| 6 | `_UNIFIED_THROW_CUP_Z_MM` | `reload_coordinator_node.py:523` | cup z mm | **860.0** | rev 5.7037 | rev **5.5390** | **COMMAND −0.1648 rev = −5.37 mm.** Physically the machine has been throwing from **865.37 mm**. Leaving the literal at 860 moves the flown throw site DOWN 5.37 mm |
| 7 | `_UNIFIED_CATCH_CUP_Z_MM` | `reload_coordinator_node.py:524` | cup z mm | **830.0** | rev 4.7552 | rev **4.6178** | **COMMAND −0.1374 rev = −4.47 mm.** Physically flown at **834.47 mm** |
| 8 | `rest_cup_z` = catch − 80 | `reload_coordinator_node.py:7294` | cup z mm | 750.0 | rev 2.2258 | rev **2.1615** | **COMMAND −0.064 rev = −2.09 mm** (follows #7) |
| 9 | `_CUP_Z_TOP_MM` | `cup_realize.py:145-147` | derived from prime rev | **994.60** | — | **1003.97** | **LABEL** (derived); but it drives #10 and #11 |
| 10 | `TILT_ACCEL_LIMIT_DEFAULT_RAD_S2` | `cup_realize.py:148-153` | derived | **5.3262** rad/s² | — | **5.2220** | **BEHAVIOUR −1.96 %** — the planner's tilt-accel cap tightens (lever 469.4 → 478.7 mm). Owner's "−2 %" confirmed |
| 11 | cup operating band `_CUP_Z_TOP_M − _CUP_Z_BOTTOM_M` | `unified_cycle.py:155-157` | derived | 315.0 mm | — | **324.37 mm** | **BEHAVIOUR +2.97 %** — the runway/box grows. Owner's "runway +3 %" confirmed. Box lands `[0.6896, 0.9940]` m instead of `[0.6896, 0.9846]` |
| 12 | `HAND_THROW_OFFSET_MM` | `toss_release.py:55-56` = `GEOM_HAND_AXIS_BOTTOM_OFFSET_MM` (−129.0) + `HAND_THROW_POS_M`×1000 | derived mm | **58.044** | — | **63.608** | **BEHAVIOUR +5.564 mm** — the ball's release plane in the ballistics model rises 5.56 mm, i.e. it has been 5.56 mm too LOW for the life of the constant. **NOT hardcoded** — the owner's blast-radius note says "hardcoded"; it is in fact derived, and `HAND_THROW_POS_M` (0.187044, generated: `generate_config.py`, x2 in metres) is a function of `hand_stroke_m`, **not** of the gain. So it moves *because of the re-base*: 0.187044 × 1.029748 = 0.192608 m. It follows automatically on regenerate |
| 13 | `HAND_MAX_DECEL_MPS2` | `cup_cycle.py:141` (`HAND_ACC_LIMIT_RPS2 / LINEAR_GAIN_REV_PER_M`) | derived | **110.699** m/s² | — | **113.993** | **BEHAVIOUR +2.97 %** — the catch-runway decel budget grows; the runway constraint (`cup_cycle.py:246,296` → `feasibility`) admits slightly more. The rev/s² limit (3500, owner-signed) is unchanged, so the *machine* is unchanged; only the model's m/s² label of it was wrong |
| 14 | `CATCH_RUNWAY_MARGIN_REV` | `feasibility.py:984-985` | derived (`0.020 m × gain`) | **0.63234 rev** | — | **0.61408** | **COMMAND −0.018 rev** — a 20 mm margin is now correctly 0.614 rev instead of 0.632 |
| 15 | `PEAK_LIMIT_REV` | `throw_envelope.py:118-120` | rev | 10.8 − 0.2 = **10.60** | 335.3 | — | **COMMAND, but owned by the FW 18 unit** — becomes 10.701 − 0.2 = **10.501 rev**. Not this unit's edit |
| 16 | smooth-move excursion ceiling | `hardware_config.yaml:1276` (`smooth_move_excursion_margin_rev: 0.2`) + firmware `SMOOTH_MOVE_POS_CEIL_REV` | rev | 10.8 − 0.2 = **10.60** | 335.3 | — | **COMMAND, owned by the FW 18 unit** (and it needs a FLASH). Gain-independent |
| 17 | coast ladder v-axis | `hardware_config.yaml` `hand_throw_envelope.measured_coast_rev`, loaded `throw_envelope.py:219-266` | (m/s, rev) pairs | top rung 4.436 m/s | — | true speed was **4.568 m/s** | **See § "The coast ladder" below — a decision, not a mechanical edit** |
| 18 | `TOP_RUNG_ACHIEVED_DECEL_RPS2` | `throw_envelope.py:270-274` | derived (`(v·gain)²`) | — | — | ×**0.9431** | **BEHAVIOUR −5.7 %** (v_rev² scales as gain²). Feeds the aliasing budget; check the report line |
| 19 | `hand_stroke_mm` (geometry) | `hardware_config.yaml:368` | mm | 344.75 | — | **352.0** | **MODEL ONLY** — consumers are the GUI render and the MJCF joint range/ctrlrange, per the key's own comment. No commanded rev |
| 20 | `HAND_STROKE_MIN_REV` = 0.0, `HAND_STROKE_MAX_REV` = prime | `feasibility.py:932-933` | **rev** | 0.0 / 9.9594 | 0 / 315.0 | 0 / **324.37** | **LABEL** — both in rev. `feasibility.py` owns the operating-band gate; there is no separate zone module |
| 21 | `HAND_HOMED_REST_FLOOR_REV`, `HAND_PARK_BAND_REV` (0.5), `HAND_HARD_STOP_REV` | `hand_stroke.py:158,172`; `feasibility.py:956,976` | **rev** | — | — | — | **LABEL** — pure rev arithmetic off `HOMING_HAND_ABS_POS_REV` (−0.1) |
| 22 | `_UNIFIED_FLOOR_TOL_REV` = 0.01 → `_UNIFIED_FLOOR_TOL_MM` | `reload_coordinator_node.py:787-791` | rev → derived mm | 0.01 rev | 0.316 mm | **0.326 mm** | **LABEL** — the commanded tolerance is in rev |
| 23 | `G4_STROKE_PEAK_MAX_REV` | `toss_trim.py:444-457` = `HARD_STOP_REV − SMOOTH_MOVE_EXCURSION_MARGIN_REV` | rev | 10.6 | — | — | **FW 18 unit** — gain-independent |
| 24 | `hand_rev_for_cup_z` / `cup_z_for_hand_rev` | `unified_cycle.py:433-465`; tilted inverse `:385-430`; realisation `cup_realize.py:742,789` | functions | — | — | — | **THE map.** Every cup-z → rev command in the tree goes through these. Three test-local re-spellings exist and must follow: `tests/hardware/unified_cycle_bench.py:377`, `tests/ros/test_unified_cycle_levelling.py:110`, `tests/ros/test_unified_cycle_integration.py:87` |


**The one decision that decides most of the table: is `hand_stroke_m` a
measured stroke or a tuned profile basis?** `hardware_config.yaml:1192-1217`
already says explicitly it is "NOT the physical stroke … a TUNED PROFILE
PARAMETER whose units happen to be metres," and the firmware computes
`x2/x3/x5` as `(hand_stroke_m − 2·stroke_margin_m) × LINEAR_GAIN`. Holding
`hand_stroke_m: 0.355` while correcting the gain drops x3 from 9.9594 to
9.6719 rev (−9.35 mm) and x2 from 5.9138 to 5.7429 rev — moving the entire
legacy calibration set the machine has flown successfully for months.
**Re-basing beat the naive gain-only fix.** Solving for the basis that
preserves x3 in rev:

```
total_stroke_new = 9.959403 / 30.703768 = 0.324371 m
hand_stroke_m     = 0.324371 + 2 × 0.020 = 0.3643707   (0.355 → 0.3643707, 7 dp)
```

Seven decimals hold the x3 rev landmark to 3e-7, inside the 1e-6 tolerance
`tests/motion/test_hand_stroke.py:141` already asserts. Under the re-base,
**positions in rev are bit-identical** and a given `event_vel` produces
2.97 % less rev/s — which *is* the excess-velocity correction, cleanly
isolated from the stroke geometry. Independent confirmation: six places in
the tree pin a legacy rev literal (`STROKE_TOP_REV`, the x2/x3/x5 triple, the
catch-prime-equals-stroke-top test, `RELEASE_POS_REV`, the probe's
`_GATE_EXPECT`, and the coast ladder's own zero reference) — **every one
stays green under the re-base and goes red without it.** That is the
codebase's own guards agreeing the re-base is the correct reading.

**Three hand-typed mirrors the sweeps found** (not in the owner's original
blast-radius list, found only by two `grep -rn` sweeps — `rg` is not
installed on this box):

1. **`sim/hand/trajectory.py:53` `HAND_STROKE_M = 0.355`** — a hand-typed
   mirror, not imported from config. **The most dangerous missed edit**: miss
   it and the sim's x3 silently forks to 9.6717 rev while the planner's stays
   9.9594, with no cross-check in `tests/sim/test_hand_trajectory.py` to catch
   the divergence.
2. **`ros_ws/.../toss_sequencer.py:751` `HAND_THROW_RELEASE_OFFSET_MM =
   58.044`** — a literal pinned copy of `toss_release.HAND_THROW_OFFSET_MM`,
   and the actual hardcoded copy the owner's blast-radius note meant (its
   named target is in fact derived, not hardcoded) → moves to 63.608.
3. **`tests/firmware/test_hand_smooth_move_xref.py:373`** — a second
   hand-typed `_X3` expression using the raw old constants; its *value*
   survives the re-base (still 9.9594) so its eight dependent tests stay
   green either way, but the literals inside are a lie once the config
   changes and must be updated for honesty.

**MJCF frame — closed, and it forces owner decision D2** (see above).

**`_hand_prime_mm` moves even though it looks gain-invariant.**
`sim/plant/mujoco_plant.py:152-161`'s `_hand_prime_mm = 20 + x3_m × 1000` is
invariant to the gain alone but not to the re-base (`x3_m` itself moves
0.315 → 0.324371): `335.000 → 344.371 mm`. `tests/sim/test_hand.py:99` pins
335.0 and must be re-pinned.

**Coast ladder (row 17) — leave it, with a comment, not a rescale.**
`hand_throw_envelope.measured_coast_rev`'s x-axis is the **commanded**
`event_vel` (the YAML provenance block: "every other throw in the same bag at
the IDENTICAL commanded 4.436 m/s coasts 0.18-0.23"); its y-axis is
`measured peak − x3`, valid **only because the re-base holds x3 at 9.9594
rev** (holding `hand_stroke_m` at 0.355 instead would silently invalidate
every rung's zero reference). After the correction, a given commanded `v`
produces 2.888 % less rev/s and coast scales as `v_rev²`, so the ladder
**over-predicts** coast by ~5.7 % of its own value — fail-safe, costing 0.013
rev (0.4 mm) of conservatism at the top rung against 0.171 rev of residual
headroom. Re-scaling a measured safety table (contract C-HAND-3) by desk
algebra to recover that 0.4 mm was rejected — the ceiling was moved to
measurement precisely to avoid exactly that move.

**The ILC trim interaction — a real hazard, not bookkeeping.**
`event_vel_trim = −0.1076` (already clamped against `SPEED_AUTHORITY`) is a
multiplier on commanded `event_vel`, fitted from measured throws, and it is
currently absorbing the excess the gain error causes. After the correction
the same commanded `event_vel` produces 2.97 % less physical speed, so:

```
k_v_new = (1 − 0.1076) × 1.029748 = 0.91895  →  event_vel_trim ≈ −0.0810
```

**If the trim is not re-fitted in the same commit, the first throw after G2
reads ~3 % slow** — the correction and the stale trim would compound instead
of cancelling, misread as "the geometry fix broke the throw." Re-fit, not
re-capture: the corpus is raw measured throws; the geometry error lives in
the model mapping command to prediction. Preferred route: re-run
`tests/hardware/ilc_fit.py` on the existing corpus with the corrected config
(`ilc_fit_lib.py` computes the trim sensitivity "through the production
chain," so if it reads `LINEAR_GAIN_REV_PER_M` the corrected trim falls out
with no new bench time).

## Discussion

**Why the re-base beat the naive gain-only fix.** The naive fix — correct
`linear_gain_factor` and leave `hand_stroke_m` alone — is a smaller diff but
moves the entire legacy calibration set (x2/x3/x5, catch prime) by 2.888 %
in commanded rev, on positions "the machine has been flying successfully for
months" (owner's words), and silently invalidates the coast ladder's zero
reference (its y-axis is defined relative to x3). The re-base costs one more
YAML key and a longer comment, but isolates the correction to exactly the
intended effect — `event_vel → rev/s` drops 2.97 %, nothing else moves in
rev — and is independently endorsed by six pre-existing test pins that go
red without it. Textbook "climb one level of abstraction before you fix":
the naive fix patches the symptom, the re-base fixes the class.

**Why `hand_stroke_mm` is not simply set to 348.524 mm (D2's cheap
alternative).** That would recreate exactly the
derived-number-wearing-a-measurement's-clothes defect this correction exists
to remove — the same failure class that made 344.75 wrong in the first
place. The audit chose the more honest, slightly more invasive route: 352.0
stays a direct measurement, and the MJCF generator derives its own
frame-correct number from the hard-stop rev and the gain.

**Five tradeoffs accepted, all as owner-flippable assumptions (not made
unilaterally):**

- **D1** — `_UNIFIED_THROW/CATCH_CUP_Z_MM` (860/830): keep the round numbers
  and accept the unified lane's throw/catch sites moving down 5.37/4.47 mm on
  hardware, or re-set to the physically-flown 865.37/834.47 and lose the
  round numbers. Orchestrator's assumption (2026-09-06/08): re-express as the
  physically-flown 865.37/834.47 mm rather than move hardware. Alternative:
  keep 860/830 and accept the 5 mm move.
- **D2** — MJCF clip: emit the derived travel-above-zero (348.524 mm) from
  the generator, vs. writing 348.524 directly into `hand_stroke_mm`.
  Orchestrator's assumption: the derived route (preserves the honest
  stop-to-stop measurement). Alternative: the cheaper direct write.
- **D3** — re-base `hand_stroke_m` at all, vs. holding 0.355 and letting
  x2/x3/x5 drop 2.888 % in rev. Orchestrator's assumption: re-base.
  Alternative: hold 0.355 (a coherent position, but it moves the legacy
  calibration set).
- **D4** — coast ladder v-axis: leave the measured table untouched
  (~5.7 % conservative afterwards) vs. rescale it by 1.029748 to recover the
  0.4 mm of ceiling. Orchestrator's assumption: leave it — the conservatism
  costs 0.4 mm, the rescale opens a safety contract (C-HAND-3) by desk
  algebra. Alternative: rescale.
- **D5** — `sim/cycle_gate.py`'s 0.690/0.985 m literals: follow the re-base
  to 0.6896/0.9940, or stay frozen as Phase-1 numbers. Orchestrator's
  assumption: follow, to preserve the cross-check `unified_cycle.py:164`
  already claims against them. Alternative: freeze and rewrite the claim
  instead of the literals.

All five are recorded as **assumptions pending owner confirmation** in
`plans/active/hand-geometry-correction.md`, not as decided facts. They landed
in G2 (2a25b98, software-only, branch `hand-geometry-correction`) — that
commit IS the future G2 commit this section originally described in the
future tense; it has landed but **not flown**: no commanded hand position has
changed on hardware, because the branch has not been flashed or merged, and
G3 (bench re-validation) must fly and pass before it merges to
`mvp-trajectory-bringup`. A sixth assumption, D6, was found afterward
(G2a-2) and is recorded alongside these five.

## Verification

No pytest ran as part of G1 — it is a read-only audit; every file:line cited
was read directly, not inferred, and the arithmetic above was run in the
project venv interactively (not captured as a script). No config, code, or
test file was edited in G1.

**First full gate on the branch** (2026-09-08 12:31–12:39, `./run_tests.sh --full` in the worktree):
parallel **38 failed / 6921 passed / 9 skipped / 3 xfailed in 482.61 s**, serial 4 passed, exit 1.
All 38 were pinned literals in twelve files the G2 units had not run (cadence rung check 7, toss
continuous 6, unified launch floor 5, toss session 5, hand-stroke timeline probe 5, catch
coordinator 3, unified cycle bench 2, demo juggle sim/optimizer 1+1, ball possession 1,
validate-cycle 1, cup-cycle 1), in three families, each verified as the mechanical consequence of
the correction and updated with the derivation at the site: the hand-floor dwell terms scale with
the corrected gain (`HAND_MAX_DECEL` 110.70 → 113.99 mm/s² lifts `hand_floor_dwell_s` a few ms,
which pushes cadence rung R4's ILC-loaded floor past its old margin — 50.2 → 55.8 throws/min where
the pre-correction table read 50.6 → 56.3); the predicted chain sites moved with the catch plane
(+5.76 mm projected through the tilt); and the worked-example release/catch heights moved with
the two planes. No tolerance widened, no assertion deleted, nothing unexplained by geometry. The
twelve files alone (2026-09-08, `pytest <the twelve files> -q`): **755 passed / 5 xfailed in
417.53 s**. Four of those five xfails are NEW and deliberate — pins the unit could not attribute to
geometry and refused to widen: the cadence ladder's **R4** rung now clears its ILC-loaded dwell
floor by **−2 ms** (two tests; the fix is a re-cut of `tests/hardware/session_cadence_ladder.md`'s
R4 operating point, the critical-point-ilc plan's), and two MuJoCo demo tests —
`test_demo_juggle_optimizer.py::test_optimiser_converges` (IPOPT now takes **636** iterations
against a 200 ceiling, still converges) and `test_demo_juggle_sim.py`'s short run (**1 of 2**
catches land, reproducible at 8 s and 15 s, so not a timing-boundary miss). Both are
deterministic and correlated with the geometry-derived demo targets; the diagnosis is NOT clear,
so they are `xfail(strict=True)` for a short investigation rather than silently re-pinned — the
demo lane is sim-only and off the hardware path, but a sim catch that stopped landing after a
geometry change is exactly the kind of signal that must not be tidied away. Open item 7 below.

**Full gate after the sweep** (2026-09-08 14:43–14:51, `./run_tests.sh --full` in the worktree):
parallel **6955 passed / 9 skipped / 7 xfailed in 462.50 s**, serial **4 passed in 25.92 s**, total
494 s, **exit 0**. The seven xfails are the two pre-existing ones, D6, and the four of open item 7.
**G2 software** (all 2026-09-08, worktree `~/Desktop/Jugglebot-geometry`, branch
`hand-geometry-correction`): G2a-1 `pytest tests/motion/test_throw_envelope.py -q` **39 passed**,
`tests/sim/test_hand_throw_decel_ff.py` **22 passed**, `tests/motion/test_hand_stroke.py`
**25 passed**, `tests/sim/test_hand_trajectory.py` **224 passed**; G2a-2 the combined set
(`test_toss_release`, `test_reload_coordinator_node`, `test_toss_sequencer`,
`test_unified_cycle_integration`, `test_toss_coordinator`, `test_gui_geometry`, `test_mjcf_drift`,
`test_unified_gate`, `test_cycle_gate`, `test_hand_throw_decel_xref`, `test_ilc_fit`, `-q`):
**926 passed / 3 skipped / 1 xfailed in 124.88 s** (the xfail is D6); G2a-3
`pytest tests/firmware/test_platform_fw_version_xref.py tests/firmware/test_bridge_fw_version_xref.py -q`
**15 passed**, `tests/motion/test_toss_ilc.py` **165 passed**, `tests/ros/test_toss_ilc_node.py`
**42 passed**. The ILC re-fit: `ilc_fit_lib.fit_corpus` on the committed 19-row corpus at the
corrected config reproduces **−0.107592** (the fit's sensitivity is 2/g, pure ballistics — the gain
never enters), so the audit's pre-registered fallback applied: rescaled by the measured gain ratio
31.6172 / 30.7038 = 1.02975 to **−0.081044**, `admit_command()` True at every goal cell.

**`MIRROR_TOL_LEG_REV` tightened 5e-4 -> 3e-4** (`sim/unified_gate.py:335`,
B-N2, 2026-09-08 audit finding). Measured (2026-09-08,
`python sim/unified_gate.py --no-viewer`): worst HONEST leg residual over the
full 27-point grid (SET 1 single-toss + SET 2 two-pose ring) is **2.837e-05
rev** — 10.6x below the new 3e-4 band, real headroom preserved — while the
ring's fault footprint is **4.068e-04 rev**, now 1.36x OVER the new band,
restoring the non-vacuity the old 5e-4 band had lost (the fault footprint
used to sit comfortably inside it). 3e-4 is the tightest round number that
clears both the >=10x honest-headroom floor and the fault-exceeds-band
requirement — the honest worst already spends over a tenth of that margin,
and a smaller round number (1e-4) would leave under 4x headroom.

**Audit fixes (2026-09-08).** One phase audit over the G2 commit (one BLOCKING, seven WARNING,
four NOTE findings, all applied on this branch). The blocking one was the G3 runsheet itself: it
told the operator a physical cup-z reading should move between Part A and Part B, when the
physical height at 9.9594 rev is a property of the mechanism and only the model's label moves —
the merge gate was inverted, and is now stated as the model-vs-ruler gap closing. Code changes:
the bench and sim twins of the unified cup-z heights follow production (865.37 / 834.47, pinned
bench == production); one generated source for the hand's travel — `HAND_TRAVEL_ABOVE_ZERO_MM`
(stop / gain, the metal, for the sim plant's clip and the MJCF) and `HAND_WIRE_CLIP_TRAVEL_MM`
((stop − clip margin) / gain = 342.0 mm, the furthest position the wire will command, for
`cup_realize`'s clamp and the four sim juggle planners) — the realizer/sim parity test caught the
two bounds disagreeing by 6.5 mm while the fixes were in flight; the three knife-edge bounds made
deterministic (the timeline probe pins its sampling phase and asserts exact values). Two bounds
in the sim gate's decay probe were found to be below what the wire can represent and were
restated at the wire's own quanta rather than widened: the deadline comparisons add the 1 µs
timestamp quantum, and the lead-clamp comparison adds one float32 ULP at the command's
magnitude (every Setpoint and Telemetry field is float32; the old `+1e-9` had passed only on the
rounding sign, which flipped when the cup-z re-expression moved the cut). The probe now exposes
the lead over the frozen encoder and the encoder-minus-command offset at the cut so the clamp's
real invariant is what is asserted. Runs (2026-09-08, worktree): the fixer's seventeen-file set
**575 passed / 3 xfailed** (one failure, the lead-clamp bound, then fixed);
`pytest tests/sim/test_unified_gate.py tests/motion/test_cup_realize.py -q` **64 passed in 16.45 s**.

**Full gate after audit fixes** (2026-09-08 19:02–19:10, `./run_tests.sh --full` in the worktree):
parallel **6957 passed / 9 skipped / 7 xfailed in 463.32 s**, serial **4 passed in 26.08 s**,
total 494 s, **exit 0**.

## Open items

1. **G3 (bench re-validation) has not been flown.** The owner's acceptance
   criterion is explicit: re-validate the legacy x2/x3 release and catch
   points on hardware *before* the correction ships, so a post-change
   measurement is attributable to the gain fix alone. Runsheet:
   `tests/hardware/session_hand_geometry_revalidation.md`.
2. **The five owner decisions (D1–D5) above are orchestrator assumptions,
   not confirmations.** G2 must not ship until the owner has either
   confirmed them or flipped them.
3. **This correction closes ~3 points of an ~11 % measured throw excess.**
   After it lands the machine will still throw ~8 % fast; the remainder
   (candidate mechanisms: accel-phase feedforward, the `INERTIA_RATIO` mass
   model, release timing) is unexamined here and stays open. Do not ship
   this as "the throw fix."
4. **The Platform Teensy needs a flash** (Arduino IDE only — the pio image
   is CAN-MUTE) because its compiled hand constants move; its firmware
   version goes 4 → 5. Coordinate with the FW 18 (can-bridge) flash, which is
   happening on the same bench sitting.
5. **The ILC `event_vel_trim` re-fit is a G2 obligation in the same commit**
   as the gain/stroke correction, not a follow-up — see Discussion above.
   Done in G2 (−0.081044, staged, see Verification); the re-fit is a
   gain-ratio rescale, not a corpus re-fit, because the fit's sensitivity is
   pure ballistics and never sees the gain.
6. **D6 — the C-HAND-2 open-loop undershoot ceiling** (G2a-2). The corrected
   gain raises the worst open-loop commanded undershoot at the band floor to
   0.629 rev against the 0.60 rev `test_wire_quantisation_cannot_produce_a_visible_undershoot`
   operationalises for `ros_ws/docs/hand_decel_feedforward.md`'s C-HAND-2.
   The test is `xfail(strict=True)`, the ceiling is NOT raised: a landed safety
   contract changes document-first and by the owner. Recorded on the plan.
7. **Four xfails the gate sweep could not attribute to geometry** (G2a-4):
   the cadence ladder's R4 rung clears its ILC-loaded dwell floor by −2 ms
   (two tests in `test_cadence_rung_check.py`; fix = re-cut the R4 operating
   point in `tests/hardware/session_cadence_ladder.md`, critical-point-ilc's
   plan), and two MuJoCo demo tests — the optimiser's iteration count (636 vs
   a 200 ceiling, still converging) and one of two demo catches no longer
   landing (reproducible, not a timing boundary). Sim-only, off the hardware
   path, diagnosis unclear — a short investigation before they are re-pinned.
