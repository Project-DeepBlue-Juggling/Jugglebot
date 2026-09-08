---
title: Hand geometry correction — the measured linear gain replaces the "just 'cuz" factor
created: 2026-09-08
status: active
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

**Parent plan:** [unified-7dof-planner.md](unified-7dof-planner.md) § "Hand
geometry correction" (owner, 2026-09-06). **Branch:** `mvp-trajectory-bringup`
for the G3 baseline sitting; the software build lives on branch
`hand-geometry-correction` in the worktree `~/Desktop/Jugglebot-geometry`, so
the operator can take the legacy baseline on the current branch before
switching over — see the runsheet.
**Status 2026-09-08: G1 DONE. G2 IN PROGRESS** on branch
`hand-geometry-correction` (worktree `~/Desktop/Jugglebot-geometry`).
**G3 NOT FLOWN.** Deliberately **not** in the Phase 5 fix commit — pairing it
with a frame fix and a replan gate would make all three unbisectable.

## The measurement
Owner, bench, 2026-09-06: stroke **352 mm** between hard stops, bottom
**−0.107 rev**, top **10.701 rev** ⇒ `352.0/10.808` = **32.5685 mm/rev**
(**30.7045 rev/m**) against the planner's 31.6284/31.6172 — 2.97 % of travel
the machine has and the model does not.

## Decisions

1. **The wrong knob is `linear_gain_factor`, not `hand_spool_radius_m`** (the
   radius is measured; the factor was a "just 'cuz" fudge).
   **`linear_gain_factor: 1.035 → 1.0051`** (4 dp; residual 0.0008 mm/rev).
   Meaning changes from fudge to **measured cable/spool calibration**:
   `r_eff = r/factor` = **5.183 mm** vs. the 5.21 mm bare spool — half a
   cable diameter. The old 1.035 implied 5.034 mm, 3.4 % under the drum:
   physically implausible, the tell it was never geometry.

2. **`hand_stroke_m` is re-based so every commanded rev is preserved:
   `0.355 → 0.3643707`** (full rationale under assumption D3 below). This key
   is the throw profile's *basis*, not a stroke (`hardware_config.yaml:1192-1217`);
   holding 0.355 instead drops x3 9.9594 → 9.6719 rev, x2 5.9138 → 5.7429 — the
   entire legacy calibration the machine has flown for months.

3. **`stroke_margin_m` stays 0.02** — also `cup_realize.SLIDER_REV_ZERO_MM`
   (20 mm), the cup-z anchor at rev 0; moving it moves the cup.

4. **`hand_stroke_mm: 344.75 → 352.0`** — the honest stop-to-stop measurement,
   replacing an inference `(10.8 + 0.1) × 31.6284` documented as "a measured
   fact". Travel **above encoder zero** is a different number, 348.524 mm
   (10.701 rev); the 3.48 mm gap is the firmware's homing offset (assumption D2).

5. **The coast ladder is not touched** (comment only; full rationale under
   assumption D4 below) — its v-axis is the *commanded* `event_vel`, so
   afterwards it over-predicts coast by ~5.7 %, fail-safe.

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
| Firmware | **FLASH REQUIRED** (Arduino IDE only, pio is CAN-MUTE); BallButler unaffected (own spool) — regenerate `--no-external` |

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

### G2 — Software correction. **IN PROGRESS on branch `hand-geometry-correction`
(worktree `~/Desktop/Jugglebot-geometry`); merge to `mvp-trajectory-bringup`
BLOCKED on G3 flying/passing and D1-D5 being confirmed or flipped.**
Built on its own branch, not `mvp-trajectory-bringup`, so the operator can
take the G3 baseline sitting on the current branch first (runsheet's Part A /
Part B). One commit: YAML (3 keys + comments) → `generate_config.py
--no-external` → `sim/model/generate_mjcf.py` → 4 hand-edited code literals →
9 hand-edited test literals → docs → the ILC trim re-fit. Full gate
(`./run_tests.sh --full` — `sim/` touched) plus `/audit --unstaged`.

**G2 items, both required:** (1) **Platform Teensy flash, FW 4 → 5**
(Arduino IDE only — `pio` image is CAN-MUTE); its compiled hand constants
move, so the hand runs the old geometry until this lands — coordinate with
the FW 18 (can-bridge) flash on the same sitting. (2) **ILC `event_vel_trim`
re-fit** (−0.1076 → ≈ −0.0810), same commit or not at all — it currently
absorbs the excess this correction removes, so skipping it makes the first
post-G2 throw read ~3 % **slow**, misread as "the fix broke the throw."
Re-fit on the existing corpus, no new capture.

### G3 — Bench re-validation. **OPERATOR. NOT FLOWN. Must precede G2 merging.**
Owner's acceptance criterion: re-validate legacy `x2`/`x3` **before** the
correction ships, so the post-change measurement is readable. Runsheet:
`tests/hardware/session_hand_geometry_revalidation.md` — Part A baselines on
`mvp-trajectory-bringup` (current branch, after FW 18); Part B switches to
the `hand-geometry-correction` worktree, flashes Platform FW 5, swaps in the
re-fitted ILC artifact, repeats; Part C is the pass/fail call. Key checks:
top stop **10.701 ± 0.02 rev**, bottom **−0.107**; catch prime **9.9594 ±
0.10 rev**, cup z ≈**1003.97 mm** (not 994.6); x2 release **5.9138 rev**, cup
z **872.21 mm**; then a two/three-tier commanded-vs-achieved velocity and
apex baseline. Abort: peak > 10.60 rev (10.501 post-FW-18), `dip_below_x3` >
0.100 rev, end-stop contact, hand fault. Full checklist in the G1 logbook
entry § (e) / `geometry_audit.md`.

## Acceptance

- G3 flown and passed, **before** G2 lands.
- Full gate green at `--full`, with the (date, command, result) triple.
- Commanded rev at x2/x3/x5/prime **bit-identical** across the change
  (`test_hand_stroke.py::test_catch_prime_equals_the_stroke_top` is the guard).
- Post-G2 bench: rev unchanged, release velocity **−2.97 %**, mm/cup-z agree
  with the model first time; ILC trim re-fitted, inside its band.

## What this does NOT do
**~3 points of an ~11 % measured throw excess.** After it the machine still
throws ~8 % fast — do not ship it as "the throw fix"; the remainder stays
open and conflating the two makes the next measurement unreadable.

## Assumptions pending owner confirmation (all block merging G2)

Software on `hand-geometry-correction` is built against these five
orchestrator assumptions (2026-09-06/08), not owner-confirmed facts — each
flippable, with its alternative named.

- **D1** `_UNIFIED_THROW/CATCH_CUP_Z_MM` (`reload_coordinator_node.py:523-524`)
  **re-set to the physically-flown 865.37/834.47 mm.** Alternative: keep the
  round 860/830 and accept the unified lane's sites moving down 5.37/4.47 mm
  on hardware (the lane passed its ladder on 2026-09-04, so this is a live
  behaviour change either way).
- **D2** MJCF clip frame **emits the derived travel-above-zero (0.348524 m)**
  from `generate_mjcf.py`, re-pointing `tests/sim/test_hand.py:56-57`.
  Alternative: write 348.524 straight into `hand_stroke_mm` — cheaper, but
  re-creates the derived-number-as-measurement defect this plan removes.
- **D3** `hand_stroke_m` **is re-based** (0.355 → 0.3643707) so commanded rev
  is preserved. Alternative: hold 0.355 and accept x2/x3/x5/prime dropping
  2.888 % in commanded rev — a coherent position, but it moves the legacy
  calibration set the machine has flown for months.
- **D4** the measured coast ladder's v-axis **is left untouched** (comment
  only; reads ~5.7 % conservative afterwards). Alternative: rescale it by
  1.029748 to recover 0.4 mm of ceiling — opens safety contract C-HAND-3 by
  desk algebra for a 0.4 mm gain.
- **D5** `sim/cycle_gate.py`'s 0.690/0.985 m literals **follow the re-base**
  to 0.6896/0.9940. Alternative: freeze them as Phase-1 numbers and rewrite
  `unified_cycle.py:164`'s "within 0.4 mm" claim instead.
- **D6** (found by G2a-2, 2026-09-08) the C-HAND-2 open-loop undershoot ceiling.
  The corrected gain raises the worst open-loop commanded undershoot at the
  band floor to **0.629 rev** against the **0.60 rev** that
  `tests/firmware/test_hand_throw_decel_xref.py::test_wire_quantisation_cannot_produce_a_visible_undershoot`
  operationalises for the C-HAND-2 clause of `ros_ws/docs/hand_decel_feedforward.md`.
  The test's own closed-loop argument (4.3 % attenuation ⇒ ~0.025 rev real dip)
  says 0.65 would be safe, but a landed safety contract is not raised by an
  agent: the test is `xfail(strict=True)` naming the number, **not** widened.
  Owner's call: raise the ceiling (contract doc first, then the test) or reduce
  the feedforward. The xfail comes out with the decision.
