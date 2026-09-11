---
title: Hand geometry correction — the measured linear gain replaces the "just 'cuz" factor
created: 2026-09-08
status: superseded   # 2026-09-11 — measurement absorbed into two-ball-skill-stack.md R1 (owner decision); see the Archival note below
archived: 2026-09-11
related_plan: unified-7dof-planner.md
related_logbook:
  - 2026-09-06-unified-cycle-first-hardware-cycles.md
  - 2026-09-05-unified-7dof-planner-phase4-unified-cycle-mode.md
related_config:
  - config/hardware_config.yaml → teensy_trajectory.linear_gain_factor (1.035 → 1.0051)
  - config/hardware_config.yaml → teensy_trajectory.hand_stroke_m (0.355 → 0.3643707)
  - config/hardware_config.yaml → jugglebot_geometry.hand_stroke_mm (344.75 → 352.0)
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/hand_stroke.py::LINEAR_GAIN_REV_PER_M
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py::_CUP_Z_TOP_MM
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py::HAND_MAX_DECEL_MPS2
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py::HAND_THROW_RELEASE_OFFSET_MM
  - sim/hand/trajectory.py::LINEAR_GAIN_FACTOR
---

# Plan — Hand geometry correction

## Archival note (2026-09-11)

Superseded by `two-ball-skill-stack.md` (owner decision 2026-09-11). The G2 build on branch
`hand-geometry-correction` (worktree `~/Desktop/Jugglebot-geometry`, 70 files) targets the FSM toss
stack and the Platform Teensy stroke engine, both retired by the skill stack: R0 already deleted five of
the test files it touches, and R1 deletes `hand_stroke.py`, `Trajectory.h` and the ILC. The branch is
therefore **not merged**; it and its worktree stay as the record of G1/G2 and the worktree is removed
once R1 lands.

**What survives is the measurement, and it lands at R1** as one measured key,
`hand_mm_per_rev`, from which the generated `HAND_REV_PER_M` is derived; `linear_gain_factor` and
`hand_spool_radius_m` are deleted rather than re-valued (one number per physical fact, no fudge knob —
plan § 0). The only firmware consumer of the old factor was `Trajectory.h`, which R1 deletes; the sim
plant's slider gain reads the new key.

| Reading | Top | Bottom | Span | mm/rev |
|---|---|---|---|---|
| 2026-09-06 (owner, bench) | 10.701 rev | −0.107 rev | 10.808 rev | 32.5685 |
| 2026-09-11 (owner, manual slide, encoder read) | 10.691 rev | −0.118 rev | 10.809 rev | 32.5654 |

The two agree on the scale to 0.01 %; the 0.011 rev shift at both ends is homing-zero variation (about
0.36 mm), a fact the rest floor and the clip bands must tolerate. R1 adopts the mean, **32.567 mm/rev
(30.705 rev/m)**, with both readings in the config comment.

Of the five assumptions: **D1** survives by construction (skill-stack sites are stated in true
millimetres); **D2** (the MJCF hand-joint clip is travel above encoder zero, ≈ 348.2–348.5 mm, not the
stop-to-stop 352 mm) becomes an R2 sim item; **D3** dies with the stroke engine; **D4** (the coast
ladder's v-axis left untouched, reading ≈ 5.7 % conservative) is carried as a note on the admissible
sweep's inputs; **D5** dies with `sim/cycle_gate.py` at R2. **G3 is replaced by one row in R1's flash
runbook**: on the streamed lane, jog the hand to each hard stop and read the encoder — expect a span of
10.81 ± 0.01 rev with the ends near −0.11 / 10.69 rev.

The ≈ 3 % gain error is about a quarter of the measured +11 % launch-speed excess; the learner owns the
rest. The filename is unchanged (DOCUMENTATION_GUIDE § 2.6); every reference keeps resolving.

**Parent plan:** [unified-7dof-planner.md](unified-7dof-planner.md) § "Hand
geometry correction" (owner, 2026-09-06). **Branch:** `mvp-trajectory-bringup`.
**Status: NOT FLOWN.** G1 done; G2 blocked on G3 and five owner decisions.
Deliberately **not** in the Phase 5 fix commit — pairing it with a frame fix and
a replan gate would make all three unbisectable.

## The measurement
Owner, bench, 2026-09-06: stroke **352 mm** between hard stops, bottom
**−0.107 rev**, top **10.701 rev** ⇒ `352.0/10.808` = **32.5685 mm/rev**
(**30.7045 rev/m**) against the planner's 31.6284 / 31.6172 — 2.97 % of travel
the machine has and the model does not.

## Decisions

1. **The wrong knob is `linear_gain_factor`, not `hand_spool_radius_m`.** The
   radius is measured; the factor was a "just 'cuz" fudge.
   **`linear_gain_factor: 1.035 → 1.0051`** (= 30.70454 × 2π × 0.00521, 4 dp;
   residual 0.0008 mm/rev). Its meaning changes from fudge to **measured
   cable/spool calibration**: `r_eff = r/factor` = **5.183 mm** against the
   5.21 mm bare spool — half a cable diameter, which is what a cable that does
   not wrap on the bare drum looks like. The old 1.035 implied 5.034 mm, 3.4 %
   under the drum: physically implausible, and the tell it was never geometry.

2. **`hand_stroke_m` is re-based so every commanded rev is preserved:
   `0.355 → 0.3643707`.** This key is the throw profile's *basis*, not a stroke
   (`hardware_config.yaml:1192-1217`); `x3 = (hand_stroke_m − 2·stroke_margin_m) ×
   LINEAR_GAIN`. Hold 0.355 and x3 drops 9.9594 → 9.6719 rev, x2 5.9138 → 5.7429 —
   the entire legacy calibration the machine has flown for months. Re-basing keeps
   **positions in rev bit-identical** while a given `event_vel` produces **2.97 %
   less rev/s**, which *is* the correction. Seven decimals hold x3 to 3e-7 rev.
   Five pinned rev literals stay green *because of* the re-base, and the measured
   coast ladder — whose y-axis is `peak − x3` — stays valid only under it.

3. **`stroke_margin_m` stays 0.02** — it is also `cup_realize.SLIDER_REV_ZERO_MM`
   (20 mm), the cup-z anchor at rev 0; moving it moves the cup.

4. **`hand_stroke_mm: 344.75 → 352.0`** — the honest stop-to-stop measurement,
   replacing an inference `(10.8 + 0.1) × 31.6284` documented as "a measured
   fact". Travel **above encoder zero** is a different number, 348.524 mm
   (10.701 rev); the 3.48 mm gap is the firmware's homing offset (decision D2).

5. **The coast ladder is not touched** (comment only). Its v-axis is the
   *commanded* `event_vel` (`hardware_config.yaml:1449`: "the IDENTICAL commanded
   4.436 m/s"), so afterwards it over-predicts coast by ~5.7 % — **fail-safe**,
   worth 0.013 rev (0.4 mm) at the top rung against 0.171 rev of residual
   headroom. Re-scaling a measured safety table by desk algebra opens C-HAND-3.

6. **`hand_motor_hard_stop_revs` (10.8 → 10.701), the smooth-move margin and
   `PEAK_LIMIT_REV` (10.6 → 10.501) belong to the FW 18 unit.** G2 asserts FW 18
   landed first and stops if it did not.

## Interfaces

| Surface | Effect |
|---|---|
| Commanded **rev** positions (x2/x3/x5, catch prime, park, floors) | **unchanged** (that is the re-base) |
| Commanded **rev/s** for a given `event_vel` | **−2.97 %** — the fix; durations +2.97 % |
| Cup-z-keyed sites (`_UNIFIED_THROW/CATCH_CUP_Z_MM`, `SETTLE_CUP_Z_MM`) | commanded rev moves; hardware moves **−5.37 / −4.47 / −0.30 mm** unless the literals are re-set — **owner decision D1** |
| `TILT_ACCEL_LIMIT_DEFAULT_RAD_S2` | 5.3262 → **5.2220** (−1.96 %) |
| Cup operating band | 315.0 → **324.37 mm** (+2.97 %) |
| `HAND_MAX_DECEL_MPS2` | 110.699 → **113.993** (+2.97 %); rev/s² limit unchanged |
| `HAND_THROW_OFFSET_MM` | 58.044 → **63.608** — the ballistics release plane rises 5.56 mm |
| Firmware | **FLASH REQUIRED** (Arduino IDE only; pio image is CAN-MUTE). BallButler unaffected — own spool; regenerate `--no-external` |

## Phases

### G1 — Audit. **DONE 2026-09-08.**
Gain derived, `hand_stroke_mm` decided, all 24 commanded-vs-label positions
classified with file:line, blast radius and pinned tests enumerated, bench
checklist written, ILC interaction analysed. Deliverable `geometry_audit.md`
(session scratchpad; fold into the G2 logbook entry) — it carries the full
derivation and the G2 handoff. Three hardcoded literals the owner's blast-radius
list did not have: `sim/hand/trajectory.py:53 HAND_STROKE_M = 0.355` (a
hand-typed mirror whose omission silently forks sim from planner),
`toss_sequencer.py:751 HAND_THROW_RELEASE_OFFSET_MM = 58.044` (the real hardcoded
copy), `test_hand_smooth_move_xref.py:373`'s hand-typed `_X3`.

### G2 — Software correction. **BLOCKED on G3 and on D1-D5.**
One commit: YAML (3 keys + comment rewrites) → `generate_config.py --no-external`
→ `sim/model/generate_mjcf.py` → 4 hand-edited code literals → 9 hand-edited test
literals → docs → the ILC trim re-fit. Full gate (`./run_tests.sh --full` —
`sim/` is touched) plus `/audit --unstaged`; then the flash, with FW 18.

**In the same commit or not at all: the ILC `event_vel_trim` re-fit**
(−0.1076 → ≈ −0.0810). It currently absorbs the excess this correction removes;
leave it and the first throw after G2 is ~3 % **slow**, which reads as "the
geometry fix broke the throw". Re-fit on the existing corpus — no new capture.

### G3 — Bench re-validation. **OPERATOR. Must precede G2 shipping.**
Owner's acceptance criterion, explicit: re-validate legacy `x2` and `x3`
**before** the correction ships, so the post-change measurement is readable.
Dry-run the offline scorer first (`toss_trace_recorder.py check … --dry`), then:
top stop reads **10.701 ± 0.02 rev**, bottom **−0.107** (STOP if off by > 0.05);
catch prime settles 9.9594 ± 0.10 rev with cup z near **1003.97 mm** (not 994.6 —
the independent confirmation of 32.57 mm/rev); legacy kind-0 throw releases at
5.9138 rev, cup z **872.21 mm**; then a two-or-three-tier baseline of commanded
`event_vel` vs achieved release velocity vs ball apex. Abort: peak > 10.60 rev
(10.501 post-FW-18), `dip_below_x3` > 0.100 rev, end-stop contact, hand fault.
Repeat after G2. Full checklist in the G1 audit § (e).

## Acceptance

- G3 flown and passed, **before** G2 lands.
- Full gate green at `--full`, with the (date, command, result) triple.
- Commanded rev at x2/x3/x5/prime **bit-identical** across the change
  (`test_hand_stroke.py::test_catch_prime_equals_the_stroke_top` is the guard).
- Post-G2 bench: rev positions unchanged, release velocity **−2.97 %**, mm/cup-z
  readings agree with the model first time; ILC trim re-fitted, inside its band.

## What this does NOT do
**~3 points of an ~11 % measured throw excess.** After it the machine still
throws ~8 % fast. Do not ship it as "the throw fix" — the remainder is Open
Question 1 of the sitting's entry, and conflating the two makes the next
measurement unreadable.

## Open owner decisions (all block G2)

- **D1** `_UNIFIED_THROW/CATCH_CUP_Z_MM` (`reload_coordinator_node.py:523-524`):
  keep the round 860/830 and let the unified lane's sites move down 5.37/4.47 mm
  on hardware, or re-set to 865.37/834.47 and preserve the flown geometry? The
  lane passed its ladder on 2026-09-04, so this is a live behaviour change.
- **D2** MJCF clip frame: emit the derived travel-above-zero (0.348524 m) from
  `generate_mjcf.py` and re-point `tests/sim/test_hand.py:56-57`, or write 348.524
  into `hand_stroke_mm`? The latter is cheaper and re-creates the exact
  derived-number-as-measurement defect this plan removes.
- **D3** Confirm decision 2 (the re-base) explicitly. **D4** Confirm decision 5.
- **D5** `sim/cycle_gate.py`'s 0.690 / 0.985 m literals — frozen Phase-1 numbers,
  or follow to 0.6896 / 0.9940?
