---
title: Hand-stroke timeline probe — the post-throw dip measured, and the instrument taught to score a FIXED capture
type: investigation
date: 2026-07-26
status: tuned
phase: "Self-toss anomaly fixes — hand-command-continuity Phase 0"
related_plan: "hand-command-continuity.md"
files_changed:
  - tools/probes/hand_stroke_timeline.py
  - tools/probes/data/hand_stroke_timeline_gate_ref.jsonl
  - tools/probes/README.md
  - tools/README.md
  - tests/hardware/session_anomaly_fixes.md
  - plans/archived/hand-command-continuity.md
commits:
  - cabf3c6
subsystem:
  - motion
  - ros
  - sim
tags:
  - testing
  - docs
  - safety
  - dynamics
---

# Hand-stroke timeline probe — the post-throw dip measured, and the instrument taught to score a FIXED capture

## Summary

Phase 0 of `plans/archived/hand-command-continuity.md` lands
`tools/probes/hand_stroke_timeline.py`: a reusable, offline, read-only harness
that reconstructs the hand's commanded-vs-measured timeline around every throw in
a recorded session, and turns the operator-visible post-throw dip into numbers.
It reproduces all 25 rows of the plan's Context table from the reference trace
(worst delta 0.4 ms against a 20 ms tolerance) and characterises the defect across
the whole evidence base: **7 self-tosses, all truncated mid-stroke, the hand
pulled 10.7–55.3 mm below the stroke end, with as little as 0.775 rev = 24.5 mm of
headroom to the 11.1 rev end stop.** No production code changed in this phase.

The phase's second half is a *verdict* obligation, and that is where the real work
of this session went: a probe validated only against the broken shape scores a
working fix as FAILED at the bench. Three instrument defects of exactly that kind
were found and fixed, and the gated criteria were re-derived from measured
thresholds instead of from artefacts of the probe's own control flow.

## Symptoms

Every one of the seven 2026-07-25 self-tosses shows the same shape in
`hand_telemetry`:

- `pos_cmd` stops following the throw's deceleration ramp and **freezes** at the
  live encoder value, somewhere between 6.1965 and 7.7825 rev (the ramp should
  have run to `x3` = 9.9594 rev);
- the hand coasts on to 10.1653–10.3248 rev — 321.5–326.6 mm — leaving
  **0.775–0.935 rev** of headroom to the `hand_motor_max_position_revs = 11.1`
  overextension guard;
- the position loop then pulls back at −17.9 to −42.4 rev/s and the hand ends up
  **0.339–1.748 rev = 10.7–55.3 mm below `x3`** before recovering;
- the two BallButler reload tosses in the same sessions show none of it (no throw
  stroke to clobber), and the one `ABORTED_NO_RELEASE` toss shows no stroke and no
  catch descent at all.

The operator-visible symptom is a dip in the hand just before the catch. The
less-visible one matters more: the same freeze discards the throw's own decel
ramp, so the ball's departure conditions are set by the position loop's reaction
to a frozen setpoint at whatever instant the arm frame lands — a throw
**repeatability** defect, not only a cosmetic one.

## Diagnosis

Traced end to end in the firmware source. Any kind-0/1/2 `HAND_TRAJ_CMD` makes
`Teensy_code.ino` build a prelude from the **live encoder position**
(`:522`, `makeSmoothMove(activeTraj.x.front())`) whose quintic is seeded
`v = a = 0` (`Trajectory.h:242-301`; `current_hand_velocity` is declared `extern`
at `:47` and never read by the generator), then clear the whole packed queue
(`:539-540`) and pack prelude + main (`:543-546`). So a kind-1 hand-catch arm
landing inside the throw's 65 ms deceleration ramp discards the remaining decel
frames and replaces them with a **rest-to-rest** ramp computed from a position the
hand is passing through at ~120 rev/s.

The probe's independent corroboration is the seed detector: `makeSmoothMove`
starts at `start_rev = current_hand_position` *exactly*, so each detected
from-rest quintic seed is cross-checked against the live `pos_meas` of the
preceding samples. Every seed across the evidence base reads **0.0000 rev**
(worst 0.0001) from that live value — the direct fingerprint of "the queue was
re-seeded from the live encoder mid-stroke". Seed counts also reproduce the
`Arming hand catch` counts from the launch logs on 7/7 tosses that had a stroke.

Two supporting measurements the plan's window arithmetic needed:

- **A dispatch latency nothing compensates.** `teensy_bridge_node.py:3652` stamps
  the absolute event from the *bridge's own* `time.time()` plus a delay the caller
  computed from *its* own clock read, so all ROS service transit lands the event
  late and `JB_OP_TOSS_RELEASE_LATENCY_MS` ships `0.0`. Measured **+12.8 to
  +21.9 ms** by bag clock (median +19.0), and up to **+23.4 ms** through the jsonl
  trace path — a systematic 1.5–1.7 ms clock-path difference on the same physical
  toss. Consequence: a stroke-busy window of `t_release_announced + t_dec` expires
  *mid-ramp*, so Phase 1's margin is not optional slack.
- **`_MAX_ARM_DISPATCHES = 2` is honoured** (15 dispatches over 10 tosses, maximum
  2 per ball) — no separate defect there. Five of ten tosses dispatched only
  *once*, including the toss the Context table measures, so the dip is caused by
  *any* kind-0/1/2 command landing mid-stroke, not by the retry.

## Discussion

### The finding that mattered: the instrument had only ever been scored against the broken shape

Phase 0's deliverable is an *instrument*, and the plan gives it a two-sided job —
"the same command is both the pre-fix characterisation and the post-fix verdict".
The pre-fix half was solid and reproduced under independent re-derivation. The
post-fix half had never been exercised against anything but one `tof = 0.80`
synthetic, and every gated criterion turned out to be an artefact of the probe's
control flow rather than a measured threshold.

Verified by running the probe's own model of a *perfect* post-fix capture: the
`clean` synthetic printed `dip_bottom 9.9594 rev depth 0.020 rev = 0.6 mm` and
`pullback −0.3 rev/s`, while the operator runbook said PASS was `dip_bottom = -`
and ABORT was "any dip printed", and gave `pullback` two mutually contradictory
criteria inside the same table row. An operator following that table would have
scored a working fix as FAILED, routed it back to Phase 1, and burned a powered
sitting — which is worse than having no instrument at all, because a bad reading
carries authority a missing one does not.

The root cause is that `dip` is **peak-minus-bottom**. That is non-zero on *any*
capture that overshoots and settles, including the bounded overshoot plan Phase 4
step 2 explicitly makes the expected behaviour: the 10.60 rev synthetic reads
20.2 mm of "dip", which is *larger than the smallest real pre-fix defect*
(20.3 mm). No threshold on `dip` can separate those two populations.

**Decide-and-document fork: fix the runbook prose, or fix the measure?** Two of
the three reviewers proposed making the runbook rows numeric (`dip depth <= 0.10
rev`). That would have made the document self-consistent while leaving the
criterion duplicated in four places of prose (the probe docstring, the probe
README, the runbook table, the runbook's "must read `dip=-`" line) and still
gating on a quantity that mixes two different physical events. The chosen fix
instead adds `dip_below_x3` — how far under the stroke end the hand ended up — and
gates on that. Concrete failure modes this prevents that the prose fix does not:
(a) a legitimate Phase-4 overshoot of 0.64 rev scores ABORT under any `dip`
threshold tight enough to catch the 20.3 mm ball-34 defect, because the two
readings are numerically identical; (b) the four prose copies drift the moment one
is edited, which had already happened once in this plan (the runbook told the
operator PASS was the literal string "GATE PASS - 20/20 rows" while the probe
printed 24/24). `dip_below_x3` is a *different quantity*, not a re-tuned one: the
pre-fix population reads 0.339–1.748 rev and the healthy population reads
0.000–0.001 rev, with nothing in between.

The band is `<= 0.10` rev = 3.2 mm, chosen as the **same** allowance the plan
already grants for overshoot *above* `x3`, so the criterion is symmetric about the
stroke end rather than a free parameter. **Tradeoff accepted and stated in the
runbook:** the margin is ~100× on the healthy side but only **3.4×** on the
tightest real defect (ball 34, 0.339 rev). A post-fix reading near 0.2–0.3 rev is
therefore not a clean PASS, and the runbook says so rather than implying the band
has more room than it does.

`pullback` was demoted from a gate to a bounded report for the same reason, and
this is the one place where a reviewer's suggested number would have been wrong:
both reviewers proposed ABORT at −1.0 or −2.0 rev/s. Measured, a *healthy* settle
from the coasting peak reads −0.31 rev/s at 0.02 rev of overshoot, −1.58 at the
runbook's own 10.060 rev ceiling, and −10.03 at 10.60 rev. So −1.0 and −2.0 both
falsely abort. The criterion is now ABORT below **−5.0** rev/s and explicitly
*conditional on `peak` having passed first* — because once `peak <= 10.060` rev is
established, the settle velocity it can produce is bounded at about −1.6 rev/s,
which leaves 3.6× to the tightest pre-fix −17.9. The row ordering in the runbook
table is load-bearing, not cosmetic.

### A downward command is not the catch descent

`catch_desc` bounds *both* the truncation scan and the peak/pullback/dip window,
and it was identified as "the first sample whose commanded velocity is below
−1 rev/s". A `makeSmoothMove` **brake** from the coasting peak back to `x3` is
also downward — and plan Phase 4 step 3 *specifically charters that brake* for the
at-target-but-moving case. So Phase 4's own deliverable guarantees the spoofing
frame exists on every capture the probe must score, and this is not a
hypothetical: with the old predicate, a synthetic post-fix capture carrying a
brake at release+105 ms reports `catch_desc` **567 ms early** (at the brake,
release+130 ms), so the peak/pullback/dip window collapses from ~600 ms to ~130 ms
and `peak` — the row that guards the 11.1 rev end stop — reads **10.1298 rev
against a real 10.1588**, i.e. it under-reports the excursion.

`catch_desc` now requires the commanded position to actually travel 0.5 rev below
`x3`, then walks back to the onset of that descent's negative-velocity run. The
0.5 rev is anchored on measured geometry, not tuned: the armed catch runs
`x3` → `x5` = 3.833 rev, while the largest coast above `x3` across the seven
observed tosses is 0.365 rev (peaks 10.1653–10.3248 rev against `x3` 9.9594). So
the threshold sits **7.7×** clear of the descent it must accept and **1.4×** clear
of the largest brake it must reject. The brake is reported
separately as `first_neg_cmd`, so it is **visible** rather than silently
shadowing the event it is not — and the runbook treats an annotated
`first_neg_cmd` as REPORT-not-ABORT, since after Phase 4 lands it is expected.

### Why the gate needed a short-flight case

`--gate`'s fixed-shape branch only ran at `tof = 0.80`, where `_DIP_WINDOW_S =
0.6` binds *before* `catch_desc` — so neither `catch_desc` cap was individually
load-bearing and a future maintainer could delete one with the gate staying
green. A `short-flight` case at `FLIGHT_TIME_MIN_S = 0.55` was added, where the
descent begins inside the window. Mutation-verified: deleting the dip-window cap
now fails, with the clean short-flight capture reporting **3.83 rev** of dip
instead of 0.001.

Recorded honestly rather than papered over: the **truncation-scan** cap is still
not pinned by any case, because the 50 ms scan margin binds ahead of it in every
physically reachable flight. It is belt-and-braces, and no synthetic was contorted
to pretend otherwise. Claiming coverage that does not exist is how a gate stops
being trusted.

### The mirror is faithful — but its absolute frame is not the firmware's

Phase 0's other job was clearing `sim/hand/trajectory.py` as the gate Phase 4 will
be test-driven against. It is faithful to float32 round-off in every quantity
Phase 4 asserts (positions ≤4.4e-8 m, segment durations ≤3.5e-8 s,
`makeSmoothMove` duration ≤3.2e-8 s, mid-move position ≤6.3e-7 rev), and the two
divergences found — the catch time origin (−4.9…−9.7 ms, a velocity-independent
0.498 rev = 15.75 mm) and the catch `end_time` (+90…+95 ms, the
`END_PROFILE_HOLD_S` term) — both live in the catch timeline, which none of Phase
4's assertions touch. So the STOP condition correctly does not fire.

**A hypothesis withdrawn.** The Outcome had recorded the sim-vs-firmware absolute
position frames as differing by "a documented, intentional +20 mm — the sim
measures from the physical bottom, the firmware from the encoder zero at the
bottom of the effective stroke". The second clause does not survive the shipped
code. `Homing::HAND_SPEED_RPS = -3.0f` homes the hand **downward** into the bottom
hard stop and `Homing::HAND_ABS_POS_REV = -0.1f`, so encoder zero sits 3.16 mm —
not 20 mm — above the physical bottom. Two independent checks confirm it:
`hand_motor_max_position_revs = 11.1` is 351.08 mm above zero, which places the
overextension guard 0.76 mm *below* the top of the 355 mm stroke (coherent only if
zero is the physical bottom; were zero 20 mm up, the shipped guard would sit 16 mm
*past* the hard stop), and `generate_config.py:585` derives
`HAND_CATCH_OFFSET_MM` with no margin term.

This mattered because Phase 4 must convert the `[0, 11.1]` rev end-stop bound into
the sim's millimetres. Carrying the +20 mm across would have set the sweep test's
ceiling **20 mm = 0.63 rev too high** — at 371.08 mm, a physical point 16.8 mm
(**0.532 rev**) past the 11.1 rev overextension guard, which is itself only
0.76 mm below the top of the 355 mm stroke, and 16.1 mm above the top of the sim's
own modelled travel. On a system that has already reached 10.325 rev with
0.775 rev of headroom, the test would have passed and the profile would then have
been transcribed to `Trajectory.h` and flashed.
The plan now states the corrected frame, the one valid mapping
(`rev = sim_mm/1000 × LINEAR_GAIN`, no margin term), and carries the constraint
*inside Phase 4 step 2 and step 4* rather than only in the Phase-0 Outcome — a
fresh Phase-4 agent reads its own section first.

The 20 mm gap is real, though, just not a cancelling convention: the sim *centres*
the 315 mm stroke in the 355 mm travel (20…335 mm) while the firmware puts it at
the bottom (0…315 mm with 40 mm unused above). Recorded as a **second** candidate
contributor to the known sim-catch fidelity gap
(`project_hand_catch_hardware_smooth_sim_janky`), alongside the 15.75 mm timeline
offset: `sim/input/scripted.py:315-316` catches 20 mm higher than the host stack
believes it does.

### Findings verified and deliberately not acted on

- **A third host-side copy of the stroke model.** `StrokeModel` and
  `smooth_move_duration_s` duplicate `Trajectory.h`, in a plan whose Phase 1 step
  2 explicitly mandates one shared `motion/` helper so the model is not copied a
  third time. Acting now would pre-empt Phase 1's design. The probe instead
  carries a note at the formula stating it is the pre-Phase-4 rest-to-rest form
  and must move with it — because the `quintic_T_model_s` gate row replays a
  frozen *pre-fix* fixture and so structurally cannot notice the formula going
  stale after Phase 4 retires it.
- **The kind-3 clobber gap** (`Teensy_code.ino:472-475` returns before
  `packedMsgs.clear()`, so a `SAFE_ABORT` retract within 1e-6 rev of the live
  position does not clear an armed stroke). Left untouched deliberately: kind-3 is
  the only un-arm mechanism the Teensy offers, and moving that clear changes the
  clobber path. Handed to the operator.
- **A truncated bag** at `~/Desktop/rosbags/2026-06-08_22-25-46` (286 MB, footer
  decodes as a 6.3 TB record) — that session was killed mid-write and is
  unreadable by any mcap tool, not just this probe.

## Fix

No production code changed. `tools/probes/hand_stroke_timeline.py` and its
committed gate fixture are the deliverable; the rest is documentation.

**Instrument, this session:**

1. `dip_below_x3_rev` / `_mm` added and made the gated dip measure; `dip_depth_*`
   retained as a reported quantity because the plan's Context table quotes it.
2. `catch_desc` re-anchored on a descent that travels `_CATCH_DESC_BELOW_X3_REV =
   0.5` rev below `x3`, with a walk-back to the run onset; `first_neg_cmd`
   reported separately so a braking prelude is visible.
3. `--gate`'s fixed-shape branch grown from 2 to **4** cases (`clean`,
   `overshoot`, `short-flight` at `FLIGHT_TIME_MIN_S`, `braking-prelude`), each
   pinning `dip_below_x3_rev` and `catch_desc` against the descent onset the
   synthetic built. A `dip_below_x3_rev` row added to the Context-table branch
   (24 → 25 rows).
4. `load_bag` now **detects** `/rosout` instead of asserting bags cannot carry it,
   and reports the arm count when it is there. The old behaviour printed
   "bags carry no /rosout" at an operator whose bag had it.
5. `--json` output timestamped, matching `fk_convergence_bag_check.py` and
   `cone_bag_decode.py` — a fixed name destroyed the previous `CHECK HAND-n`
   analysis in the same sitting.

**Documentation:**

6. `tests/hardware/session_anomaly_fixes.md` § Section HAND: the PASS/ABORT table
   restated as seven ordered numeric rows; the recorder env corrected to system
   `python3` with the ROS env (**not** the venv — the recorder's own docstring and
   `session_phase8_toss_trace.md:100` both say so, and Section HAND contradicted
   both on the one mandatory capture step); the `/rosout` sentence given its true
   reason; the baseline table extended with the two gated columns and its `shift`
   column labelled as the bag-clock reading.
7. `plans/archived/hand-command-continuity.md`: Confirmation 2's frame claim
   corrected and the constraint pushed into Phase 4 steps 2 and 4; the second
   catch divergence named; the normative mirror bounds identified (the second
   pass's, not the first's — they differ by up to 16×, and the looser set would
   let a real few-1e-6-rev drift pass Phase 4's xref test); the window table
   labelled as nominal-armed-velocity with the runtime-`event_vel` obligation
   added to Phase 1 step 3; Phase 5's PASS text corrected off "no negative
   velocity" onto the numeric rows; a § Third-pass hardening subsection added.
8. `tools/README.md` and `tools/probes/README.md`: the two conventions this probe
   deviates from (the matplotlib `--preview` rule; "never write inside `tools/`")
   scoped **in the convention documents first**, per the repo's own contract rule,
   rather than left as carve-outs asserted inside the deviating file.

## Outcome

**Gate: PASS.** `python tools/probes/hand_stroke_timeline.py --gate` (run
2026-07-26): `GATE PASS — 25/25 rows within tolerance` plus `GATE PASS —
fixed-shape branch` (4 cases), exit 0. Worst instant delta 0.4 ms against a 20 ms
tolerance. The same gate forced onto the committed fixture alone — the fresh-clone
path — returns 25/25 with identical deltas and `arms=1`, and
`--emit-gate-fixture` regenerates that fixture **byte-identically** (285 of 5764
rows).

**Mutation-verified discriminating power** (run 2026-07-26, each mutation applied
to a copy of the module and `run_fixed_shape_gate` re-run):

| mutation | result |
|---|---|
| old `catch_desc` predicate (first negative commanded velocity) | **CAUGHT** — `braking-prelude` fails: `catch_desc` 2.13 vs 2.697 (567 ms early), and `peak` 10.1298 vs a real 10.1588 |
| `catch_desc` cap on the dip window removed | **CAUGHT** — `short-flight` reports `dip_below_x3` 3.833 rev vs 0.001 |
| `dip_below_x3 := dip_depth` (peak-minus-bottom) | **CAUGHT** — `overshoot` 0.639 rev, `braking-prelude` 0.199 rev |
| `catch_desc` cap on the truncation scan removed | **not caught** — recorded as belt-and-braces; the 50 ms margin binds first in every reachable flight |

**Error paths** (run 2026-07-26): a corrupt bag alongside a good trace prints one
`ERROR: … RecordLengthLimitExceeded` line, analyses the good input, exit 1; an
empty bag directory (the `SystemExit` path) likewise; `--emit-gate-fixture`
without `--trace` is an argparse error, exit 2.

**Full suite:** `pytest tests/ -q` (run 2026-07-26, on the Jetson): **3429 passed,
3 xfailed, 198 warnings in 1323.44 s (0:22:03)**. Identical pass/xfail counts to
the pre-change baseline at HEAD `cf58157` (3429 passed, 3 xfailed in
~1331–1341 s) — **no net count change**, because no test file was added, modified,
weakened, xfailed or skipped by this phase. The xfail count is unchanged at **3**.
Neither of the two order/load-flaky allocation-budget tests
(`test_hot_loop_allocation_contract`,
`test_t3b_h4_on_post_solve_allocates_within_budget`) failed, so no isolated re-run
was needed.

An earlier run of the same command the same day — **3429 passed, 3 xfailed in
1338.06 s (0:22:18)** — was superseded because a comment block in the probe was
edited after it started. Both runs agree exactly on counts.

The gate above was re-run once more after the final comment/docstring corrections
to the probe (`--gate` → both `GATE PASS` lines, exit 0). Those corrections are
comment text in a file the suite provably never loads: `pytest.ini` sets
`testpaths = ["tests"]` so `tools/` is not collected, and
`grep -rIl hand_stroke_timeline tests/` returns exactly one file — this plan's
runbook markdown — with **zero** `.py` hits.

**Deferred to the operator / later phases:**

- Whether the firmware's kind-1 catch should centre the velocity hold on the
  predicted arrival (as `calcCatch`'s own construction implies and `makeFull`
  honours) rather than start it there. Out of scope for this plan; the host stack
  is self-consistent with the firmware, so the sim mirror is the outlier.
- The kind-3 clobber gap's at-target-**and**-at-rest half, which needs the queue
  clear moved ahead of the early return — a change to the only un-arm mechanism
  the Teensy offers, and therefore an operator decision.
- All bench validation. `tests/hardware/session_anomaly_fixes.md` § Section HAND
  carries the capture requirement, the verdict command, the seven-row numeric
  PASS/ABORT table and the pre-fix baseline to score against; Phases 1, 2 and 4
  append their own `CHECK HAND-n` bodies under the same header. **Nothing in this
  phase needs a colcon build, a relaunch or a flash** — the probe is offline and
  read-only, and no `ros_ws/src` or `config/` file changed. Phase 4 will need a
  Platform Teensy flash (not the can-bridge).

## Related

- Plan: `plans/archived/hand-command-continuity.md` (Phase 0 — Outcome)
- Runbook: `tests/hardware/session_anomaly_fixes.md` § Section HAND
- Probe: `tools/probes/hand_stroke_timeline.py`, README row in
  `tools/probes/README.md`
- Sibling phases of the same run: `logbook/2026-07-25-fk-convergence-tolerance.md`,
  `logbook/2026-07-25-levelling-frame-enumeration.md`
## Close-out — 2026-08-21

**Status `in-progress` → `tuned`.** This entry sat `in-progress` for four weeks
after its own work was done, because it was gated on "the instrument has not
scored a real post-fix capture yet". It has, several times over:

* the probe scored the 2026-07-27 validation sitting (17 tosses, six bags, six
  traces) and its verdict is what `plans/archived/hand-command-continuity.md`
  Phase 5 records;
* `--gate` now runs inside the pytest suite
  (`tests/motion/test_hand_stroke_timeline_probe.py`) after it was found RED on
  the shipped tree — `headroom_to_limit_rev` was keyed to the retired 11.1 rev
  anchor and nothing tested it;
* both criterion defects the instrument carried have since been found and fixed
  (`logbook/2026-08-18-trunc-criterion-stroke-end.md`): the truncation scan is
  bounded on the commanded profile REACHING `x3` rather than on a wall clock,
  and the collapse threshold is profile-relative rather than an absolute
  10 rev/s.

**`tuned`, not `resolved`, deliberately.** Two of this entry's Phase-0 findings —
the catch time-origin divergence (0.498 rev = 15.75 mm) and the 20 mm absolute
catch-height placement — are real, unresolved, and now owned by
`plans/parked/hand-trajectory-generator-overhaul.md` § 6 *Inherited findings*.
They were unowned by any plan until that re-home. The derivations stay in
`plans/archived/hand-command-continuity.md` § Phase 0 — Outcome, Confirmation 2.

**Numbers in this entry that were re-anchored later.** Every end-stop headroom
figure here was measured against the *declared* 11.1 rev stop. The operator
measured metal contact at **10.8 rev** on 2026-08-18
(`logbook/2026-08-18-hand-end-stop-corrected.md`), so every headroom recorded
here is **0.3 rev larger than the truth** — the worst case was 0.475 rev, not
0.775. The `peak` measurements themselves are unchanged; only the reference
moved. Left as written per the annotate-never-edit rule.
