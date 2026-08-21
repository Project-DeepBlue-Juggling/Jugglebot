---
title: hand-command-continuity remediated and archived — two stale anchors that would have mis-scored a powered sitting, and the sweep that only checked the name
type: bugfix
date: 2026-08-21
status: resolved
phase: "hand-command-continuity — archival remediation"
related_plan: "hand-command-continuity.md"
files_changed:
  - plans/archived/hand-command-continuity.md
  - plans/active/INDEX.md
  - plans/archived/INDEX.md
  - plans/parked/INDEX.md
  - plans/parked/hand-trajectory-generator-overhaul.md
  - tests/hardware/session_anomaly_fixes.md
  - ros_ws/docs/hand_command_continuity.md
  - ros_ws/docs/platform_fw_version.md
  - docs/can_bridge/safety.md
  - teensy_link/rpc_args.py
  - sim/plant/mujoco_plant.py
  - sim/hand/trajectory.py
  - tests/sim/test_hand.py
  - experimenting/platform_calibration/measuring_leg_mapping/can_interface.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/hand_stroke.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/Teensy_code_platform/Trajectory.h
  - ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino
  - config/hardware_config.yaml
  - tools/probes/hand_stroke_timeline.py
  - tools/probes/hand_decel_authority.py
  - tools/probes/README.md
  - tests/motion/test_hand_stroke.py
  - tests/firmware/test_hand_smooth_move_xref.py
  - tests/ros/test_catch_coordinator_node.py
  - tests/sim/test_hand_trajectory.py
  - tests/sim/test_hand_throw_decel_ff.py
  - sim/juggle_demo.py
  - sim/juggle_online.py
  - sim/juggle_selfcatch.py
  - sim/juggle_throw.py
  - sim/juggle_tilt.py
  - docs/sim_mpc/control_loop.md
  - docs/sim_mpc/hand_and_ball.md
  - docs/sim_mpc/plant.md
  - logbook/2026-08-10-hand-drive-braking-clamp-diagnosis.md
  - logbook/2026-08-18-hand-end-stop-corrected.md
subsystem:
  - motion
  - sim
  - config
  - ros
  - tools
tags:
  - safety
  - docs
  - testing
---

# hand-command-continuity — remediation and archival

## Problem

A pre-archival review of `hand-command-continuity.md` returned **NOT READY**. The
engineering was complete — all eight phases shipped, flashed (Platform Teensy
**FW 3**, can-bridge **FW 15**) and validated on hardware to the throw envelope's
ceiling on 2026-08-21. What blocked it was bookkeeping that had gone stale, and
two items would have actively mis-scored a powered bench session:

1. **The runbook expected the wrong firmware.** `tests/hardware/session_anomaly_fixes.md`
   rows **FW-1** and **H4.0d** gave PASS as `Platform Teensy reports v2` /
   `platform_fw_version: 2`, and routed anything else to *"flashed from a
   different tree; `git pull` + re-flash"*. The board and
   `teensy_link/rpc_args.py::PLATFORM_FW_VERSION_EXPECTED` are both on **3**
   (2026-08-18, the hand end-stop correction). As written the runbook sends the
   operator to re-flash a **correctly flashed board** — and the same box that
   carries the row is the one that says *"a `FAIL … v<other>` means you flashed
   from a different tree"*, so there is nothing in the procedure to catch it.
2. **A stale-anchor defect in live sim code.** `sim/plant/mujoco_plant.py`'s
   `_hand_stroke_mm` was a hardcoded `355.0`, used at `command_hand` as the clip
   bound, while `config/hardware_config.yaml` has carried
   `hand_stroke_mm: 344.75` since 2026-08-18. **The simulated hand could be
   commanded 10.25 mm further than the real one can go** — verbatim the failure
   the 2026-08-18 correction exists to prevent, and named in that entry's own
   text.

Plus: C-HAND-1's obligation F still read *"NOT LIVE until the Platform Teensy is
flashed"* four weeks after the flash; seven logbook entries were still
`status: in-progress`, each gated on "deferred to the operator" and never
re-statused, three of them pointing `related_plan:` at a file deleted in
`07f2361`; three Phase-0 findings had no owner in any plan; and roughly forty
live numbers and firmware line references had drifted.

## Root cause

Two distinct mechanisms, worth separating because they need different guards.

**(a) A rename-driven sweep verifies the NAME, not the VALUE.** The 2026-08-18
end-stop correction renamed `hand_motor_max_position_revs` →
`hand_motor_hard_stop_revs` and swept the symbol *down to zero*, which its
logbook entry records. That claim is true and was read as stronger than it is:
the **11.1 rev value** was still live in two places a symbol-scoped grep cannot
see —

* ⚠ `experimenting/platform_calibration/measuring_leg_mapping/can_interface.py:90`,
  `_HAND_MOTOR_MAX_POSITION = 11.1`, used as the hand clip bound in a script that
  **actuates the real motor**: a declared travel limit **0.3 rev = 9.5 mm past
  metal**, surviving purely because the symbol name differs;
* `sim/plant/mujoco_plant.py`, the `355.0` above — the 2026-08-18 entry names the
  MJCF geometry path (`sim/model/generate_mjcf.py`) and misses the second,
  independent hardcoded copy in the plant, which is the one in the command path.

The claim is withdrawn, with both survivors and a method note, in
`logbook/2026-08-18-hand-end-stop-corrected.md` § *Withdrawn claims*.

**(b) A phase status written *before* a deferred hardware sitting is never
re-read after it.** Every one of the seven `in-progress` entries, the obligation-F
row, the "Needs two flashes to be fully live" line and the FW-1/H4.0d rows was
correct on the day it was written and became wrong at an event nobody was
watching for — the flash, or the sitting. The plan itself already carried two
"status corrected 2026-08-18" annotations for exactly this, on rows written the
same way. The pattern is not a lapse; it is what a deferred-to-the-operator gate
does by default.

## Fix

**Hard-gate items.**

1. FW-1, H4.0d, the DEPLOYMENT MATRIX row C, the stage-8 pre-flight, § CHECK
   HAND-7's header and H7.0 now expect **v3**, with `v2` given its own row
   (has the decel feedforward, predates the end-stop correction, so
   `smoothMoveMaxDuration()` is still 0.8005 s and **H4.10 mis-scores it**).
   `ros_ws/docs/platform_fw_version.md` now says explicitly that later releases
   are **not** restated in prose — read them off the `.ino` and
   `PLATFORM_FW_VERSION_EXPECTED` — because a prose copy is what rotted.
2. C-HAND-1 obligation F re-statused **LIVE**, with the evidence (flash confirmed
   on all six launches of the 2026-07-27 sitting; the velocity-continuous branch
   fired on 4 of 17 tosses). Its `0.775 rev … against the 11.1 rev overextension
   guard` is now `0.475 rev … against the 10.8 rev hard stop`.
3. `mujoco_plant`'s `_hand_stroke_mm` derived from `GEOM_HAND_STROKE_MM`;
   `_hand_prime_mm` derived from `HAND_STROKE_TOP_REV` (it carried the
   pre-Phase-3 `9.858 * 2π * 5.21` — the retired prime **and** the wrong gain,
   with no `LINEAR_GAIN_FACTOR`). `tests/sim/test_hand.py` derives the same
   expectation instead of pinning the literal. This closes known-issue item 11 of
   the runbook, which had no plan owner.
4. The three unowned Phase-0 findings — the catch **time-origin** divergence
   (0.498 rev = 15.75 mm, velocity-independent), the **20 mm absolute
   catch-height** placement, and the benign catch `end_time` gap — re-homed to
   `plans/parked/hand-trajectory-generator-overhaul.md` § 6 *Inherited findings*.
   Two things fixed in that file in the same edit: its § *Safety-critical
   invariants* instruction to keep `hand_stroke_mm` *"authoritative"* for
   over-extension detection, which now contradicts the deliberate 2026-08-18
   split and would have led a resumer to re-merge the two keys; and its
   resumption note's "re-taken against Platform FW 2".
5. The seven logbook entries closed out `resolved` / `tuned` with dated close-out
   sections that annotate rather than rewrite. The three dangling
   `related_plan: PROMPT-anomaly-fixes-orchestration.md` now point at
   `hand-command-continuity.md` (×2) and `catch-robustness.md` (the C-POSSESS-1
   owner).

**Mechanical sweep.** The retired **11.1 rev** anchor (→ 10.8; every headroom
figure in the plan's baseline table is 0.3 rev smaller than it was recorded as,
worst case 0.775 → **0.475 rev**); the band floor **0.55 → 0.4949 s** and
everything keyed to it (window 115 → **50 ms**, closure velocity 1.02/1.26 →
**1.5907 m/s**, floor headroom 15.9 → **18.0 ms**, `t_dec` span 94.5–47.4 →
**104.9–45.4 ms**, `catch/vel_scale` closing at **0.659** at the derived floor
and its shipped default 0.8 → **0.9**); `smoothMoveMaxDuration()` **0.8005 →
0.78964 s**; the continuity band mid-stroke **20.3 → 19.96 rev/s**, with the
binding bound flipped from the duration cap to the excursion clamp; Phase 5's
single `peak <= 10.060` gate replaced by the tiered criterion the runbook
actually uses. Every firmware line reference re-anchored, now citing the
**symbol** alongside the number.

**Archival.** `git mv` unchanged (no date prefix), `status: completed`,
`completed:`/`archived: 2026-08-21`, an Archival-note section, the row moved
between the two indexes, and **48 path-qualified inbound references across 28
files** re-pointed `plans/active/…` → `plans/archived/…` — including
`tools/probes/hand_stroke_timeline.py:1800`, which **prints that path to the
operator's terminal** during `--gate`.

## Audit fixes — same session

`/audit`'s reporter was run against the whole unstaged diff before the commit
(CLAUDE.md's ≥2-narrative-`*.md` / normative-doc gate). Six findings were verified
against source and fixed here; two of them are the kind this remediation exists
to catch.

1. **[HIGH] My own clip tightening created a silent saturation.** Dropping
   `command_hand`'s bound 355.0 → 344.75 mm left four sim planners still sized on
   a hardcoded `SLIDER_STROKE_MM = 355.0` (`sim/juggle_tilt.py`,
   `juggle_online.py`, `juggle_selfcatch.py`, `juggle_throw.py`), three of which
   derive a planner ceiling `pcfg.z_max_m = (CUP_Z_BASE_MM + SLIDER_STROKE_MM)/1000
   − 0.005` = **1.0096 m — 5.21 mm above what the plant will now execute**, with
   no warn and no raise. Measured: 1704 clipped `command_hand` calls at exactly
   355.000 mm in `tests/sim/test_juggle_catch.py` and 2894 in
   `test_juggle_bb_catch.py`. Impact today is ≤0.14 mm of `in_cup_offset_mm`
   against a 20 mm tolerance and everything stays green, so it was latent rather
   than a regression — but it is the same stale-literal class the change exists to
   remove, left half-fixed. All four now derive from `GEOM_HAND_STROKE_MM`, so the
   planner ceiling and the plant clip move together (`z_max_m` → 0.99935 m).
2. **[HIGH] A ~10× overstated end-stop margin, in an entry this diff already
   edits.** `logbook/2026-08-10-hand-drive-braking-clamp-diagnosis.md` reported the
   worst tier-8a stroke at 10.766 rev as **0.334 rev ≈ 10.6 mm** from the limit —
   against the *declared* 11.1. Against the measured 10.8 it is **0.034 rev ≈
   1.1 mm**, and the min/median/max become 10.5 / 3.2 / 1.1 mm rather than
   20.0 / 12.7 / 10.6. Every sibling entry in the sweep got a dated annotation;
   this one had been missed, and it is the entry that attributes the operator's
   audible end-stop bumps to exactly that margin. **The conclusion strengthens**:
   1.1 mm of remaining travel explains an audible bump far better than 10.6 mm
   did, and the median stroke was already sitting exactly on the 0.10 rev band.
3. **[MEDIUM] `sim/hand/trajectory.py:89` performed the very conflation the
   comment I added two files away warns against** — it wrote
   ``hand_stroke_mm = 355.0``, naming the *geometry* key (344.75 since
   2026-08-18) while meaning the *trajectory* one. Now names both.
4. **[MEDIUM] The published sim docs** still described the ~323 mm prime
   (`docs/sim_mpc/plant.md`, `control_loop.md`, `hand_and_ball.md`) and listed
   `HAND_STROKE_M = 0.355 m` as **"Physical hand stroke"** — the same conflation,
   in the layer a reader is most likely to trust. Corrected, plus two stale
   comments in `sim/juggle_demo.py`.
5. **[MEDIUM] A contradiction the sweep created.**
   `logbook/2026-08-18-hand-end-stop-corrected.md` § Deployment says *"Until both
   are flashed, the firmware still clamps at 11.1"* — true on 2026-08-18, and now
   contradicted by `docs/can_bridge/safety.md`. Annotated with the flashed state
   rather than rewritten, and the safety doc's FW-15 claim gained the qualifier it
   needed (FW 15 means something different on the parked clapboard branch, and an
   FW 14 board is wire-identical, so read `bridge_fw_version` rather than infer).
6. **[MEDIUM, surfaced not fixed] `sim/model/jugglebot.xml` is an unpinned stale
   generated artifact.** `sim/model/generate_mjcf.py` builds the hand joint
   `range` / `ctrlrange` from `geom['hand_stroke_mm']` and has not been re-run
   since 2026-08-18, so the committed model still says `0.355`. Nothing pins its
   freshness: `config/generate_config.py` does not emit it and `tests/sim/test_model.py`
   has no hand-range assertion. Regenerating moves sim geometry, so it is left for
   an owner decision; the plant's tighter clip is the conservative side of the
   skew and says so at the derivation.

The audit also **verified clean**, by measurement rather than inspection: that
`_hand_stroke_mm` has exactly two uses and no external readers; that the MJCF hand
actuator is a `<position>` with `gaintype=0`, so `ctrl` is an absolute metre
target and there is no divisor risk in the changed constant; that the module-level
`hardware_config` import adds no new path requirement (it was already a hard
transitive dependency through `motion/geometry.py`, and all 60 `MuJoCoPlant(`
sites run under `bootstrap_paths()` or `tests/conftest.py`); that the 12.3 mm
prime raise breaks nothing, because ball spawn/capture heights read the live
`hand_opening` site and 335.0 is exactly `HandCatchTrajectory`'s default
`start_pos_mm` — so it *removes* a prelude rather than creating one, which was the
stated intent; and that `_HAND_MOTOR_MAX_POSITION`'s only importer runs with
`ODRIVE_NUM = 0`, a leg, so the `axis_id == 6` branch is dead as configured and
the change is behaviour-neutral and conservative.

## Discussion

**Why `_hand_prime_mm` was fixed rather than re-homed, and why it landed on
335 mm and not 315 mm.** The brief allowed either. Fixing it changes sim
behaviour: the plant's park moves 322.7 → 335.0 mm, 12.3 mm, in every sim test
that primes before dropping a ball. It was fixed because the *within-sim* answer
is unambiguous even though the frame question is not: `sim/hand/trajectory.py`
already documents its own top of stroke as `STROKE_MARGIN_MM + TOTAL_STROKE_MM
≈ 335 mm`, and that is where its catch trajectory takes its first sample — so a
plant parked at 322.7 mm was not parked where the catch starts, which is the
whole point of a prime. Deriving it from `HAND_STROKE_TOP_REV` also makes it
move with codegen instead of by hand, which is what Phase 3 added that constant
for.

**315 mm was rejected, deliberately.** That is the firmware's own x3 (it homes
downward and measures from the physical bottom, so it has no 20 mm inset). Using
it would make the plant agree with the firmware and **disagree with the sim's own
catch trajectory** — the hand would jump 20 mm the moment a scripted catch
started. Choosing between the two frames *is* re-homed finding 2, and it moves
the sim's catch height, so it is not a drive-by. The comment at the derivation
says so and names the owning plan.

**Why the plan was corrected rather than frozen as-found.** An archived plan is
mostly a historical record, and the repo's rule is to annotate history rather
than edit it. The line drawn here: **live instructions were corrected**
(Phase-summary Gate and Status columns, risk-register mitigations, phase-step
text, every file:line pointer), while **measurements were annotated** (the
baseline table keeps its `peak` values and gains a note that the reference point
moved; the superseded 546/208 ms window table keeps its banner). The test is
whether a reader would *act* on the number. `bound the excursion against 11.1
rev` in the risk register is an instruction; `0.775 rev of headroom` in an
Outcome is a measurement against an anchor that has since moved.

**One premise in the review brief did not survive checking.** It listed the
`catch/vel_scale` pair *"0.45 closes it / 0.50 barely opens it"* as stale.
Re-derived against the shipped model: at the 0.55–0.56 s flight those are still
**−13.9 ms / +19.4 ms**, because that flight is HAND-1b's 0.38 m corner and it
did not move. What is stale is the surrounding claim that 0.55 s is the short end
of the band, and the default it is quoted against (0.8, now **0.9**). Both are
fixed, and the band-floor figure (**0.659 closes it**, 1.36× headroom against
1.78×) is added beside the still-correct pair rather than replacing it.

**What the fix does not close.** The generic version of failure (a) — a
value-scoped sweep after a rename — has no guard. `_HAND_MOTOR_MAX_POSITION` was
found by grepping the *literal* `11.1`, not by any test. The cheap standing habit
is: after any rename that also changes a value, sweep the **value** as a separate
pass; `git grep -n '11\.1'` over the tracked tree, excluding `logbook/`, surfaces
both survivors in seconds. (`rg` is not installed on this box and returns a
silent zero.) A stronger guard — asserting that no live module carries a hand
position bound not derived from `GEOM_HAND_MOTOR_HARD_STOP_REVS` — is possible
but would need a whitelist for the archived and experimental trees, and is not
proposed here.

## Verification

**Firmware byte-identical.** Both Platform Teensy files changed are
comment-only. `cd ros_ws/src/jugglebot/Teensy_code_platform && pio run`, run
2026-08-21, at HEAD and with the edits applied:

| | text | data | bss | `firmware.hex` md5 |
|---|---|---|---|---|
| HEAD | 102720 | 17088 | 25472 | `e9f4cac3cbba91de538806f73f14f43a` |
| edited | 102720 | 17088 | 25472 | `e9f4cac3cbba91de538806f73f14f43a` |

Identical. **No reflash is needed** and none is implied; no `FW_VERSION` was
touched.

**Codegen.** `python config/generate_config.py`, run 2026-08-21, after the
comment-only `hardware_config.yaml` edit: every generated artifact byte-identical
(`git status -s config/` shows only the YAML).

**Suite.** See the Outcome section below for the gate triple.

## Outcome

`./run_tests.sh --full`, run 2026-08-21 on the Jetson in the project venv,
**after the six audit fixes were staged**: **`RESULT: PASS`** — parallel phase
**5728 passed, 3 xfailed in 514.41 s** (rc=0), serial phase **9 passed in
41.36 s** (rc=0), total **556 s**. Total passed **5737**, xfail count unchanged
at 3.

An earlier `--full` in the same session read 491.24 s / 41.03 s. It is
deliberately NOT cited here: it predates the audit fixes, four of which are
`sim/` code and therefore in the `--full`-only tier, so it could not have
certified this diff. The only edit made after the run cited above is the
`files_changed` frontmatter list in this entry, covered by
`pytest tests/sim/test_logbook_front_matter.py tests/sim/test_plans_index.py
tests/sim/test_logbook_search.py -q` (run 2026-08-21: **100 passed**).

Platform firmware **byte-identical** after the comment-and-line-reference edits:
`pio run`, run 2026-08-21 — text **102720** / data **17088** / bss **25472**,
matching the 2026-08-18 baseline exactly. No `FW_VERSION` touched, **no reflash
implied**.

Import-order check on the four `sim/juggle_*.py` entry scripts, which install
their own path roots: `from jugglebot import hardware_config as _hw` sits AFTER
`bootstrap_paths()` in all four (lines 68/66, 92/90, 57/55, 49/47). `tests/sim/`
could not have caught a hoisted import, because `tests/conftest.py` installs the
roots first.

`--full` rather than the default gate for three reasons at once, all of them
CLAUDE.md triggers: this change touches `sim/` (`sim/plant/mujoco_plant.py`,
`sim/hand/trajectory.py`), it is a plan-phase closure, and the `nightly` tier is
where the sim/MPC coverage of those paths lives.

**No test was added, weakened, skipped or xfailed.** The only test *body* change
is `tests/sim/test_hand.py::test_command_to_prime`, which now derives its
expectation from `HAND_STROKE_TOP_REV` instead of pinning `9.858 * 2π * 5.21`,
and gained an assertion (`prime_mm == approx(335.0)`) rather than losing one.
Everything else in `tests/` is a docstring or a comment.

**The one behavioural change in the suite's reach**, stated plainly: the MuJoCo
plant now parks the hand at **335.0 mm** instead of 322.7 mm on `hand_to_prime()`,
which is used by `tests/sim/test_hand.py`, `test_ball.py`, `test_multiball.py`,
`test_scheduled_catch.py`, `test_mpc_dynamic.py` and `tests/sim/helpers.py`. Run
scoped before the gate (2026-08-21, `pytest tests/sim/test_hand.py
tests/sim/test_ball.py tests/sim/test_multiball.py tests/sim/test_scheduled_catch.py -q`):
**35 passed in 14.49 s**. The clip bound also tightened 355.0 → 344.75 mm; nothing
in the sim commands above 335 mm (`sim/hand/trajectory.py`'s top of stroke and
`juggle_optimizer`'s `top_of_stroke_mm` are both 335), so it has 9.75 mm of
headroom and is the conservative side of the skew against
`sim/model/jugglebot.xml`, which still carries a pre-split `0.355 m` joint range
because it has not been regenerated.

## Related

- Plan: `plans/archived/hand-command-continuity.md` (§ Archival note — 2026-08-21)
- Re-homed findings: `plans/parked/hand-trajectory-generator-overhaul.md` § 6
  *Inherited findings*
- Withdrawn claim: `logbook/2026-08-18-hand-end-stop-corrected.md`
  § *Withdrawn claims*
- Contracts touched: `ros_ws/docs/hand_command_continuity.md` (C-HAND-1),
  `ros_ws/docs/platform_fw_version.md` (C-PLATFW-1)
- The closures that made archival possible:
  `logbook/2026-08-18-hand-end-stop-corrected.md`,
  `logbook/2026-08-18-trunc-criterion-stroke-end.md`,
  `logbook/2026-08-21-envelope-flown-to-ceiling.md`
