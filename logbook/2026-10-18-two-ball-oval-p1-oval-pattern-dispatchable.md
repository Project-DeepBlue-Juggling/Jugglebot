---
title: Two-ball oval P1 - the one-ball oval pattern is dispatchable, C-CUP-4 and closing-speed checks land re-based, rung 1 flies at dwell 0.65
type: feature
date: 2026-10-18
status: resolved
phase: "two-ball-oval — P1"
related_plan: two-ball-oval.md
related:
  - 2026-10-17-two-ball-oval-plan-written.md
  - 2026-10-13-skill-stack-r6-close-out.md
files_changed:
  - "compiler: ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py"
  - "executor: ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py"
  - "executor: ros_ws/src/jugglebot/jugglebot/motion/skills/bump.py"
  - "checks: ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py"
  - "checks: ros_ws/src/jugglebot/jugglebot/motion/trajectory/cycle_plan.py"
  - "checks: ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py"
  - "box: ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py"
  - "box: tools/admissible_sweep.py"
  - "box: config/generated/admissible_box.yaml (re-swept, every cell)"
  - "plan: plans/active/box-sweep-same-site-fidelity.md"
  - "plan: plans/active/INDEX.md"
  - "node/GUI/sim: ros_ws/src/jugglebot/jugglebot/skill_node.py"
  - "node/GUI/sim: ros_ws/gui/js/juggle-panel.js"
  - "node/GUI/sim: sim/skills_gate.py"
  - "tools: tools/probes/late_reaim/bump_replay.py (evidence carried)"
  - "tests (new): tests/motion/test_skills_oval.py"
  - "tests (new): tests/fixtures/admissible_box_pre_oval.yaml"
  - "tests (new): tests/fixtures/cup_catch_plans/ (fx_proto_capture, fx_proto_pose, fx_sep175, fx_dwell040 npz + README.md)"
  - "tests: tests/motion/test_validate_cycle.py"
  - "tests: tests/motion/test_skills_admissible.py"
  - "tests: tests/motion/test_skills_bump.py"
  - "tests: tests/motion/test_skills_executor.py"
  - "tests: tests/motion/test_unified_cycle.py"
  - "tests: tests/ros/test_skill_node.py"
  - "tests: tests/ros/test_gui_juggle_defaults.py"
  - "tests: tests/sim/test_skills_gate.py"
  - "docs: ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md"
  - "docs: ros_ws/src/jugglebot/jugglebot/motion/skills/sites.py (docstring)"
  - "docs: plans/archived/cup-contact-contract.md (one-line pointer to INVARIANTS.md only)"
  - "docs: plans/active/two-ball-oval.md"
  - "docs: tests/hardware/session_oval_rung1.md"
  - "docs: logbook/INDEX.md"
subsystem:
  - skill-stack
  - motion-planner
tags:
  - oval
  - c-cup-4
  - safety
  - testing
  - docs
---

# Two-ball oval P1 - the oval pattern is dispatchable

## Summary

Before today nothing on the live stack could dispatch a fixed-role oval. P1 makes the
one-ball oval (`oval_1ball`, `oval_1ball_fed`) a real pattern: the compiler folds
CATCH(C) -> carry -> THROW(T) into one window, the executor threads the release site T
through every place that used to read the catch site, two new realised-plan checks
(C-CUP-4 seating, catch closing speed) gate every pattern, the admissible box gained an
oval cell and a dwell key, and the node, GUI and sim gate know the label. The phase also
corrected the plan in four places where the measured production path disagreed with the
plan text: **rung 1 flies at dwell 0.65, not 0.50**; C-CUP-4 refuses at **30 degrees / 0.30 g**
(20 / 0.5 stays the P3 shaper target); closing speed refuses below **20 %** (not 50 %; 25 % at the first re-base, then 20 %, see the closing-speed section); the
workspace rule has **no 20 mm margin**. Rung 1 is cleared to fly once the build is on the
launch; P2 starts after its outcome is logged.

## What landed

**Compiler** (`motion/skills/schedule.py`, report_compiler.md). `ThenThrow.release_site`
(default `None` = the catch site, so every same-site fold is byte-unchanged) with
`release_site_or` (:404); `OneBallPattern.roles = 'alternate' | 'fixed'` (:1297, validated
:1304, fixed needs exactly two sites); `_append_one_ball_pattern(roles=)` (:1444) and both
callers (`compile_one_ball`, `compile_reload`) pass `pattern.roles`; `_fold_catch_throw_pairs(fixed_roles=)`
(:741) folds across sites; `Schedule.pattern = 'oval_1ball'` for fixed roles (`_one_ball_label`, :1321).
Finding: the hop was already a same-site fold, so the fixed-role fold is the only cross-site one. The fold gate is proven load-bearing: restoring the same-site-only gate (`other.site is sk.site`, schedule.py:770) fails 3 of 5 `-k fold` tests (orchestrator run, 2026-10-18), 16 pass with the new gate.
The compiler cannot supply the throw site to the planner: `segments._plan_catch_throw` takes it
from `ThrowAfterCatch.site_mm`, which only the executor builds, hence the executor work.

**Executor** (`motion/skills/executor.py`, design_executor_threading.md). ONE helper
`release_site_of(skill)` (:1160; `then_throw.release_site` if set else `skill.site`) and
`_is_carried` by identity. Every release-site read uses it: `_previous_release` (:1178),
`_catch_terminal` release xy and z (:2606; T exactly, landing-independent), the `_command_u`
site argument (learner `x[0:2]` and box key `(T, C)`), `_release_pos_mm(idx)` (:2758),
`_offset_flyback_mm(idx, y_d, u_apex)` (:2785; zero for a carried fold), `_release_offset_m` (:2830),
the new `_seat_offset_m` (:2842; learner `x[2:4]`, the catch offset, which never shifts the
release or the flyback), `_register_outcome`, `_outward_unit` (:2470; counts folded release sites,
so it no longer depends on THROW 0 being in the schedule). Extras the census found beyond the
plan's eight: the window's settle rest and the Stop 'after' rest move to T (the window is
rest-terminal and the cup is at T after release); the phantom-landing branch, the drop-ball tail
and the closing REST stay at C (a ball that is not carried does not translate); a terminal
`BUMP-CARRIED` fence (:5069) keeps the catch bump off a carried release; Stop mid-carry declines
the 'before' rest by name (`STOP_BEFORE_CARRIED`, :589 and :3807-3877). The 2026-09-28 docstring is
narrowed to "a cup that is not carried does not translate". `REJECTED_SITE_OUTSIDE_WORKSPACE`
(:367) joined the REJECTED family.

**Checks** (`motion/trajectory/feasibility.py`).
`CUP_SEATING_CONE` / `CUP_SEATING_FLOOR` (:134-137, thresholds 30 deg / 0.30 g), `_SEATING_EPS`
(:143, float-equality slack because the proto plan's QP rows land on 0.49999999999725 g),
`CATCH_CLOSING_SPEED` (:165, `CATCH_CLOSING_SPEED_MIN_FRAC` 0.20, `CATCH_MATCH_FRAC` 0.7 restated
because the gate does not import its planner), `CATCH_SKID` as a report field with no threshold.
`_cup_seating_check` (:1385), `_catch_checks` (:1464), `_catch_seating_reasons` (:1534), batched
cup geometry (`_cup_axes_batch`, `_cup_positions_batch`) because scalar calls cost 2.8 ms and broke the
10 ms `validate_cycle` budget (now 7.42 ms). Plumbing: `cycle_plan.CatchEvidence` (:94, knot, time,
velocity, axis, program) carried by `CyclePlan.catch_evidence` through `plan_window`, `_concat_plans`,
`_gate_view`, the splice head and `bump._plan_slice`; all catches carried, not just the first. Held-axis
catches are exempt from closing speed by name; missing evidence on a plan with a catch is refused by name.

**Box** (`motion/skills/admissible.py`, `tools/admissible_sweep.py`).
`PATTERNS` +`oval_1ball`, `oval` (:76); fed variants use the sibling box (the caller passes the label,
as `columns_1ball_fed` does); `dwell_s` is a required keyword of `select` (:345) and `describe_miss`
(:388), unstamped boxes match no dwell, `check_limits` lost its dwell kwarg; `check_site_workspace`
(:566; `|site xy| + box half-extent <= workspace_xy_m`, no margin constant). Sweep: `oval_sweep`
(`tools/admissible_sweep.py:1096`), `_oval_window` (:1069, compiles the production `OneBallPattern(roles='fixed')`
and reads the folded CATCH), `_folded_catch_cell` (:511, shared with the hop), `_cell_metrics` (:1051),
BLAS pinned to one thread in code. EVERY cell was re-swept (see Discussion).

**Node, GUI, sim** (report_node.md). `skill_node._PATTERNS` and `_CUP_RULE` rows (`oval_1ball` as
`hop`, `oval_1ball_fed` as `columns_1ball_fed`), `_run_one_ball` dispatch with `OneBallPattern(roles='fixed')`,
`_box_coverage_refusal` (:3802; one message reports a missing box AND an outside-workspace site at once),
`skills/check` prints the oval box line and the workspace line, `_stop_with_rest` logs
`STOP-BEFORE-CARRIED` once; `juggle-panel.js` two list entries, two `PATTERN_POINTS` rows, the fed oval
shows "needs an empty cup"; `sim/skills_gate.py` `_OVAL_SEPARATION_MM`, `_sites_for`, `--pattern oval_1ball`.

**Docs.** The plan's § 7 vocabulary was condensed to stay under 400 lines; its terms (T/C, carry, empty transit, sweep, banking leak, residual, C-CUP-4, skid, closing speed, pose-level carry planner, seam dz, parked feed catch, ramp) were folded into § 2 and the § 1.3 ledger rather than restored.

**Tests.** New `tests/motion/test_skills_oval.py` (compiler 8 test functions, executor 9);
checks' tests in `tests/motion/test_validate_cycle.py`; table-shaped tests grew in place in
`test_skills_admissible.py`, `tests/ros/test_skill_node.py`, `test_gui_juggle_defaults.py`
(`len(PATTERNS)` 5 -> 7), `tests/sim/test_skills_gate.py`. Recipes in the table below.

## Discussion

### Why this approach over alternatives

*One helper, not eight local patches.* The executor read the CATCH skill's site as the release
site in at least twenty places (design table rows 1-21). The class of failure is "any new reader of
`skill.site` silently wrongs a cross-site fold", so the fix is `release_site_of` plus deleting the
`site` parameter from `_release_pos_mm`, `_release_offset_m` and `_offset_flyback_mm` so no caller
CAN pass the catch site again. Same-site patterns take the identical branch (`release_site_of(s) is s.site`),
which is why hop/columns/self_toss tests are unchanged evidence.

*Fixed roles on the existing one-ball path, not a new compiler.* `OneBallPattern.roles` at the two
site/target lines makes `compile_reload` inherit the oval for free, so the fed variant needs no entry
code (`_start_reload(site=sites[0])`).

*Estimator = the realised plan's cup-opening acceleration* (centred second difference of the realised cup
opening position), not the QP's planned acceleration. `_cup_contact_floor_check` differentiates per channel
with one-sided Hermite spans, but the 3-D cup acceleration needs xy, a nonlinear function of pose, tilt and
slider, so there is no per-channel Hermite to differentiate. The two estimators disagree materially on the
same plan (18.7 deg vs 26.6 deg); the safety question is what the cup actually does, so the realised plan
decides (D-P1-2). The estimator is also what reproduced the probe's numbers to 0.01 deg.

*Closing-speed fixtures are committed data.* The -345 mm/s (ratio 0.114) and -1742 mm/s (0.575) plans
come from the wayfinder prototype's pose QP and cannot be regenerated from committed code, so they are npz
files under `tests/fixtures/cup_catch_plans/` with a README stating provenance (D-P1-3). The 175 mm and
dwell-0.40 cells, which planning now refuses by design, are stored the same way.

*Margin rule: the planner's own workspace check is the authority, so the accept rule mirrors it
exactly.* `|site xy| + box half-extent <= workspace_xy_m`, no margin constant. Decision 38's extra 20 mm
refused columns and hop at 125 mm (125 + 20 + 20 = 165 > 150) which fly inside the planner's WORKSPACE check;
125 + 20 = 145 passes (D-P1-10).

*Test-file placement.* The checks are every-pattern contracts and the executor/compiler cases are oval-only,
so the checks' tests live beside the `CUP_CONTACT_ACC` tests in `test_validate_cycle.py` and the fold/executor
cases in the new `test_skills_oval.py` (also parallel-agent file safety); table-shaped tests grew in place
rather than "extending the seven files" (D-P1-11).

*Pooling: no learner code change.* The learner key is the release xy and `h_seat` does not
separate patterns, so the cheapest correct separation is operational: a distinct `plant_id` for oval
sittings (D-P1-6). A target-xy term in the key was the alternative; it would change a contract used by every
pattern for a one-sitting need, and the executor's own `entry_hop` rationale already states that pooling
cross and vertical throws "would corrupt both".

### What was ruled out

- **The probe's 18.7 deg, withdrawn (trigger a).** The wayfinder probe measured the rung-1 cell at 18.68 deg /
  0.837 g through a raw `plan_launch` / `plan_steady` chain. The box unit's first production sweep read
  **32.1 deg** for the same cell. The diag32 agent bisected it to ONE element: the production
  `SegmentConfig.pre_release_hold_s = 0.100 s` (4 knots, passed as `CycleGoals.pre_release_hold_knots` by
  `segments._plan_catch_throw`), which the probe's chain never sets. With the hold, the platform must be
  stationary from k51 so the QP squeezes the 150 mm carry into k36-k50 (0.375 s) and brakes at 5.4 m/s^2 against
  `a_z + g` = 0.82 g (atan = 33.6 deg). Setting only `pre_release_hold_s = 0` reproduces 18.68 deg exactly; the
  post-release hold, seed, launch period, knot count and offsets are irrelevant. So the 18.7 deg describes a
  hold-less plan that nothing dispatches, and the plan text "~17.8 / passes" was not the flown program.
  The hold is the R4 release-overshoot fix (platform still moving +70..+100 mm/s at ball separation under
  45 ms delay + 30 ms lag, measured on the hop 2026-09-29). It is KEPT: an oval-only hold of 0 would
  reintroduce that overshoot with no release-velocity-aware aim until P3's seam-dz work. The lever is
  dwell: with the default hold, 0.50 -> 32.1, 0.60 -> 21.5, **0.65 passes**, 0.70 -> 14.9 deg / 0.889 g;
  apex 0.85-1.0 all fail at 0.50 (34.6 / 32.1 / 30.9), so apex has little leverage; a one-knot hold (0.02 s)
  still reads 20.4. Decision 16's 0.50 is amended to 0.65; the ladder is 0.65 -> 0.50 -> 0.40 -> 0.30 and
  P3's shaper is what earns 0.50 back (D-P1-7). Re-bisected after the re-base to the 30 deg cone, the minimum
  passing dwell is 0.51 with a 1 deg margin, 0.65 has 12 deg; 0.65 was chosen for margin (table in Verification).
- **"Columns passes trivially" / every-pattern 20 deg / 0.5 g, withdrawn (trigger a).** The checks as
  specified refused 13 windows that fly today. Applied at 20 deg / 0.5 g: the 250 mm hop with a landing
  offset (28.7 deg, 0.320 g mid-carry at td+5..td+7, closing 0.35); the reload's held-axis feed catch
  (20.0-20.2 deg at touch-down + 1; a stationary cup holding tilt theta reads residual theta by
  construction); the held-attitude LANDING windows at the n-2 rest-approach knot (0.479 g). Columns itself
  passes. The re-base to **30 deg / 0.30 g on every pattern** (D-P1-8) is above the hop's 28.7 and the floor
  below the hop's 0.320; 20 deg / 0.5 g is kept as the P3 shaper TARGET in the constants' comment. The
  175 mm cell (21.4 deg) is therefore no longer refused by the CHECK; decision 57 (175 mm out) stands
  as an owner decision on its own authority.
- **Aligning the seating knot range with C-CUP-2's span mask** (knots catch_k+1 .. k1-1) to rescue the
  refused windows: tried, reverted, **removed nothing**. No window's worst knot is the release-adjacent knot:
  throw windows peak at td+1 (held, 20.0-20.2), td+5 / td+7 (hop 0.320 g / 28.7 deg; 22.0 deg) mid-carry; LANDING
  windows already clipped to n-2; the rung-1 fixture was unchanged. A second variant (k1-2 for windows with
  no release) cleared the LANDING floor refusals (0.479 g is a rest-terminal-tail artefact) but the held
  LANDING cone still read 20.6 deg. Not applied.
- **A `matched: bool` flag on the catch evidence** to scope CATCH_CLOSING_SPEED to catches whose program
  matches: considered, not sufficient. The axial-only LEVEL-pin match is itself a soft term, so even
  matched-only still refuses the flown columns feed at 0.42-0.48. What survived is the narrow `program`
  field, used only for the held-axis exemption (closed vocabulary, constructor error on a typo).
- **A shared `plant_id`** (decision 44) and a pooled learner: see Tradeoffs.
- **Keeping "columns cells byte-unchanged"** (the plan's Box wording): impossible, see below.

### 50 % -> 25 % -> 20 % closing speed, with the achieved-ratio distribution (trigger a)

Decision 48 was written from the STEADY catch (p50 0.58): refuse below 50 % of the 0.7x match target.
The planner's match is a soft term, and at columns' ~0.28 s runway it lands 0.42-0.50 of the target:
committed columns cells 0.44-0.48, the flown BB feed 0.42 (32 consecutive catches at sitting 8), hop
landing 0.47-0.49, self-toss 0.48, hop carry 0.349, p50 over 479 windows 0.58 (p5 0.527); held-axis
catches read ~0 (-0.006) because the cup is stationary at touch-down by design. The failure the check
exists to close is the near-hard capture-span catch at 0.114. A 50 % line cut straight through the flown
columns program. 25 % sat below the lowest flown ratio then recorded (0.349) with margin and above 0.114. The sitting-6 (2026-10-06) high catch at the historical 930 mm plane then proved lower: arrival −5520 mm/s, realised −960 vs target −3864 mm/s = 0.248; `tests/ros/test_install_segment.py` replays it (pinned at 930 mm on purpose), and the first re-base to 25 % refused that replay. The second step is 20 %: below 0.248 with margin, 1.75× above 0.114; the oval's catch reads 0.533. Held-axis catches are exempt by program (D-P1-9). Skid has no threshold: 100 mm/s covered
every measured cell and is a reading reference only (measured p50 8, p95 232, max 298 mm/s over all windows,
so a 100 mm/s refusal would have refused flown windows).

### The byte-unchanged box premise vs the gate hash (trigger b)

The plan promised "columns cells byte-unchanged". `feasibility.py` is in the box gate hash (a gated file of the box),
so adding the checks necessarily changes every cell's `gate_hash` stamp; the committed box would also be
stale to `test_the_committed_box_is_swept_at_the_launch_defaults_and_the_live_gate`. The honest invariant is
"the pre-existing cells' VALUES are unchanged". So EVERY cell is re-swept in the same commit (single / hop /
columns / oval) and `test_the_oval_resweep_changed_only_the_stamp_and_added_one_oval_cell` compares the new
file to `tests/fixtures/admissible_box_pre_oval.yaml` (HEAD's file) and fails if any pre-existing cell's
values differ; only `gate_hash` / `swept_at` may. One sweep invocation cannot reproduce the committed 19 cells
(it was three invocations with per-family args plus a merge, 2026-10-09), so the oval is its own fourth
invocation at dwell 0.65 and the four files are merged in order single, hop, columns, oval. If a value had
moved it would have been the new checks acting on committed families: stop and report (outcome in Verification).

### The extra executor sites (trigger c)

The plan listed eight sites. The census found more, and each of them is a place where "release site" and
"catch site" differ only for the oval: the window's settle rest and the Stop 'after' rest (cup is at T after
release; resting at C would fly a 150 mm empty transit inside the 0.3 s settle tail, which a diagnostic run proved
is a second blocker: `LIMIT_VEL` 447.8 mm/s on the segment tail until the rest site is T), `_outward_unit` (must
count folded release sites and not depend on THROW 0 being scheduled), and a bump fence. Deliberately NOT moved:
the phantom-landing branch (exists only in `columns_1ball_fed`, no previous release to thread), the drop-ball
tail (the survivor's throw is stripped, the ball is caught at C and not carried) and the closing REST (the last
catch is unfolded and held at C). One rule decides each: "a cup that is not carried does not translate".

### Stop-decline (trigger c)

A Stop that lands mid-carry used to offer a 'before' REST 50 ms before release. For a carried fold that REST
would be a loaded segment translating outside the C-CUP-4-checked window. On a carried fold the Stop offers only the 'after' rest: the executor declines 'before' BY NAME
(`STOP-BEFORE-CARRIED`), logs it once, the throw completes, the cup rests at T, and the ball is NOT caught (the same as
hop's Stop today). A held catch at C for the oval exit (decision 51's REST+CATCH pair) is a separate unit, not landed in
P1 (D-P1-5). Alternative considered: planning a loaded 'before' REST to T. Rejected: it adds an unchecked
loaded-translation path in the first phase that exists to check that path.

### The `plant_id` pooling (trigger b)

Decision 44 said "shared `plant_id`, rows keyed by the release xy". With `columns_sites(150)` the oval's
T = P1 = (-75, 0) has exactly the `x[0:2]` of every columns and self_toss P1 row, and `h_seat` (1.0 m) does
not separate them, so oval rows (target C, aim about C) would be fitted together with columns rows (target P1).
The owner chose a separate `plant_id` node parameter for oval sittings and no learner code change; the caveat
"two patterns sharing a release xy share rows" stays recorded as `I-LEARN-1`. This is an operational control
with no code guard: a forgotten `plant_id` pools silently. The runsheet row 12a sets it.

### Tradeoffs accepted

- **Thin margins.** Closing ratio: the oval reads 0.533 against 0.20 (fine) but the HOP carry sits 1.3 deg and
  0.02 g inside the 30 deg / 0.30 g check (28.7 deg / 0.320 g). Columns' own margin is unmeasured; the sim-gate columns
  point at dwell 0.25 or 0.30 is refused while 0.27 passes. A planner change that costs the hop a degree will refuse the hop, by
  design and by name. P3's QP rows exist to move the real margin to the 20 deg / 0.5 g target.
- **The 175 mm cell** is no longer refused by the check (21.4 deg < 30), so decision 57 (175 mm out) now rests on
  the owner's decision alone, not on a code refusal. If the owner ever wants 175 mm, the box row and the ladder
  are the only gates.
- **Pre-release hold kept** costs the oval 100 ms of a 500 ms carry and forces dwell 0.65 for the first rung,
  a slower cycle than the plan wanted, in exchange for not reintroducing the measured +x release overshoot.
- **Duplicated geometry in `feasibility.py`** (`_cup_axes_batch`, `_cup_positions_batch` copy
  `tg.cup_axis` / `uc.cup_state_from_platform`, `CATCH_MATCH_FRAC` restated): forced by the 10 ms budget and the
  gate-does-not-import-the-planner layering, and pinned equal to 1e-9 by tests rather than imported.
- **No seat-bit carry probe** was written (runsheet row 29 states it); the sitting reads seating from the bag by hand.
- **Rest-terminal tail artefact left unfixed**: the 0.479 g reading at n-2 of a window with no release clears only
  with a k1-2 end, and is not needed once the check is at 0.30 g.
- **self_toss box CHANGED (owner ruling 2026-10-18, accepted as is).** per-cell `landing_xy_m` old -> new ([x lo, hi], [y lo, hi], m): apex band 0.45-0.55 [-0.03,0.03],[-0.03,0.03] -> [-0.02,0.02],[-0.02,0.02]; 0.55-0.65 [-0.04,0.03],[-0.04,0.03] -> [-0.03,0.03],[-0.02,0.02]; 0.65-0.75 (cell 3) [-0.04,0.03],[-0.04,0.03] -> [-0.04,0.04],[-0.03,0.03]; 0.75-0.85 [-0.04,0.03],[-0.04,0.03] -> [-0.03,0.03],[-0.02,0.02]; 0.85-0.95 [-0.04,0.03],[-0.04,0.03] -> [-0.03,0.03],[-0.02,0.02]. Most edges shrank by one grid step; cell 3 (apex band 0.65-0.75) x upper went 0.03 -> 0.04, a WIDENING, also accepted by the owner (one grid step; the sweep tool gained the carry-window metric path and the settle-at-release-site rule since the 2026-10-09 box). Hop (8) and columns (6) cells are value-identical to the snapshot. Mechanism of the shrinkage: the
  sweep's same-site offset cell plans a loaded cup FLYBACK from the offset landing to the throw site; its lateral
  acceleration at the first post-touch-down knot (vertical support only 0.70 g) reads ~1 deg of C-CUP-4 residual
  per mm of offset (20 mm -> 21, 30 mm -> 30, 40 mm -> 38; cone 30), while the executor never dispatches that
  flyback (a same-site catch releases at the caught xy and shifts the AIM, `_offset_flyback_mm`, 2026-09-28 rule).
  Operational cost: a self_toss catch landing more than ~20-30 mm off now ends NO_ADMISSIBLE_COMMAND where it
  used to re-plan (on cells whose edge shrank). Follow-up: `plans/active/box-sweep-same-site-fidelity.md`. Pinned by
  `test_oval_resweep_pins_hop_columns_identical_self_toss_changed_one_oval_cell`.

### Open questions

- Does the real cup hold a 30 deg residual? The 55 deg cone is the cup's physical limit and 30 deg is the
  contract's check; rung 1's seat bit is the first measurement of how much of that margin the ball really uses.
- The sweep's `BOX_WORST_METRICS.seating_deg` for a PASSING oval cell reads the standalone catch at C (2.2 deg),
  not the carry, because the extended plan's `meta.report` carries no seating fields. The verdict is right; the
  logged number understates. Not fixed in P1.
- The sweep models a same-site translation (cup flyback) that the executor replaces with an aim shift, for every
  same-site family; hop/columns escaped only because their boxes are +-20 mm. Is the oval's carried cell affected?
  Filed as `plans/active/box-sweep-same-site-fidelity.md`.
- `tools/probes/late_reaim/bump_replay.py` was updated to carry evidence but not run.
- Whether the closing-speed definition (ratio against the soft 0.7x target, or interpolated touch-down) should be
  tightened once P3's QP rows make the match harder.

## Test recipes

Fail-before evidence is quoted from the owning agent's report. Executor cases (reported in-session, no file): written first and run against the unedited executor, 9 failed / 16 passed; after the edits 25/25 pass.

| test id | driver | fixture | fail-before evidence |
|---|---|---|---|
| `test_skills_oval.py::test_fixed_roles_throw_from_T_to_C_and_catch_at_C[n in 2,3,5 x one_ball/reload]` | `compile_one_ball` / `compile_reload` with `roles='fixed'` | `columns_sites(150)` | `pytest tests/motion/test_skills_oval.py -q` before implementation: 15 failed, 1 passed (`TypeError: __init__() got an unexpected keyword argument 'roles'`) |
| `::test_fixed_roles_label_the_schedule_oval_1ball`, `::test_fixed_roles_fold_one_window_per_cycle[2,3,6]`, `::test_same_site_folds_leave_release_site_none[1,2]`, `::test_roles_must_be_a_known_value`, `::test_fixed_roles_need_exactly_two_sites`, `::test_roles_default_is_alternate`, `::test_alternate_hop_site_sequence_is_unchanged` | compiler fold, label and validation | as above | same run (`AttributeError: 'OneBallPattern' object has no attribute 'roles'`); counterfactual gate toggle RUN by the orchestrator 2026-10-18: with `schedule.py` line 770 restored to `(other.site is sk.site)`, `pytest tests/motion/test_skills_oval.py -q -k fold` gives 3 failed / 2 passed (`test_fixed_roles_fold_one_window_per_cycle[2\|3\|6]` at an `AssertionError` on the skill count); 16 passed with the new gate |
| `::test_oval_carried_throw_releases_at_T_exactly_and_flies_nothing_back` | apex-gain executor harness, fixed roles, catch offset (0, +20, 0), `u_apex` 0.75 | `columns_sites(150)`, n_throws 2 | red: `assert [75.0, 20.0, 860.0] == [-75.0, 0.0, 860.0]` (release at the caught xy at C, not T) |
| `::test_oval_learner_state_is_release_xy_plus_catch_offset`, `::test_oval_forward_prior_queries_the_row_state`, `::test_oval_box_key_is_T_then_C` | `_offset_dispatch`, fake learner recording `predict`, spy on `adm.select` | fixed-role schedule | red: T2 x was `[0.075, 0.0, 0.0, 0.02]` vs expected -0.075; T3 box key `('P2','P2')` with release xy (75, 0) |
| `::test_oval_next_catch_prior_flies_from_T`, `::test_oval_carried_window_and_stop_rest_at_T`, `::test_oval_outward_unit_counts_folded_release_sites` | `_predicted_landing`, `_hand_corrected_landing`, `_catch_terminal`, `stop_terminals`; schedule with THROW 0 removed | fixed-role schedule | red: T4 prior built from the caught xy (`[0.0, -23.34, ...]` vs x 175.05); T5 rest at C `[75.0, 0.0, 693.5]`; T6 outward unit `None` |
| `::test_stop_mid_carry_declines_the_before_rest_by_name`, `::test_bump_never_runs_on_a_carried_fold` | `stop_terminals` mid-carry; `_catch_bump_decide` with bump enabled | fixed-role schedule | red: Stop-decline `AttributeError: ... 'stop_before_declined'`; T8 `'WITHIN-TOLERANCE' == 'BUMP-CARRIED'` |
| `test_validate_cycle.py::test_catch_closing_speed_refuses_the_capture_span_near_hard_catch` | `validate_cycle` on the stored capture plan | `fx_proto_capture.npz` (ratio 0.114) | with the checks wired off, plumbing present: 7 failed, `AssertionError: ('OK', [])` |
| `::test_catch_closing_speed_passes_the_pose_arm`, `::test_a_plan_with_a_catch_but_no_evidence_is_refused_by_name` | as above; plan rebuilt without evidence | `fx_proto_pose.npz` (0.575); rung-1 window | red: ratio `None` / plan accepted |
| `::test_a_held_axis_catch_is_exempt_from_the_closing_speed_by_name`, `::test_an_unknown_catch_program_is_a_constructor_error` | capture fixture relabelled `held_axis`; typo program | `fx_proto_capture.npz` | written with the re-base after the refusals survey (no separate red run recorded) |
| `::test_rung1_holdless_raw_chain_reads_the_pinned_values` | `_rung1_window()` regenerated from production (plan_launch + 3 plan_steady + extend) | columns_sites(150), apex 0.95, dwell 0.50 hold-less raw chain | red: readings `None`; pins 18.68 deg (+-0.05), 0.837 g, ratio 0.533, skid 9.3 |
| `::test_rung1_through_the_production_segment_path_refuses_the_cone` | `segments.plan_segment` CATCH + then_throw released at P1, default `SegmentConfig()` | dwell 0.50 | the box unit's first sweep returned `CUP_SEATING_CONE` 32.1 deg; passes as a refusal pin: code CONE, 32.1 deg, 0.82 g, one reason at knot 49 |
| `::test_the_p3_target_fixtures_pass_the_rebased_cone[fx_sep175,fx_dwell040]` | stored production windows | 21.44 / 24.41 deg, az_g > 0.5 | red, cone x2: `('OK', [])` before the check; at 20 deg they were refused (original pins), now pins of the P3 target |
| `::test_cup_seating_floor_reason_is_appended_beside_contact_acc`, `::test_the_unbumped_rung1_window_is_clear_of_the_floor` | rung-1 + 10 mm cosine pose-z bump under loose limits | rung-1 window | red: bump accepted (NOT caught by `CUP_CONTACT_ACC`) |
| `::test_a_columns_catch_carry_throw_window_passes_both_checks`, `::test_catch_match_frac_matches_the_planners_own`, `::test_batched_cup_geometry_is_the_planners_own` | production columns segment; constant pin; batched vs scalar geometry to 1e-9 | columns_sites(150) | guard pins (pass on landing) |
| `test_skills_admissible.py::test_select_requires_dwell_s`, `::test_select_refuses_a_cell_swept_at_another_dwell_and_describe_miss_names_dwell_s`, `::test_select_picks_the_cell_at_the_live_dwell_among_coexisting_dwells`, `::test_select_treats_an_unstamped_box_as_matching_no_dwell`, `::test_check_limits_no_longer_takes_a_dwell_and_passes_cells_at_mixed_dwells`, `::test_vocabulary_includes_the_oval_labels`, `::test_site_workspace_*` | HEAD `admissible.py` swapped in temporarily | boxes stamped `_DWELL_S` 0.27 | `pytest tests/motion/test_skills_admissible.py -q -k "dwell or workspace or vocabulary or select"`: 18 failed, 4 passed (`TypeError` unexpected kw `dwell_s`; `ValueError pattern must be one of ('self_toss','hop','columns')`); after: 15 passed |
| `::test_site_workspace_columns_and_hop_operating_points_pass`, `::test_the_workspace_margin_constant_is_gone` | 125 + 20 = 145 <= 150 | columns / hop operating points | node unit: 125 + 20 + 20 = 165 refused six hop-at-250 tests before the margin was dropped |
| `::test_the_oval_resweep_changed_only_the_stamp_and_added_one_oval_cell` | committed YAML vs `tests/fixtures/admissible_box_pre_oval.yaml` | the 19 pre-existing cells | red until the re-sweep landed (no oval cell; stale `gate_hash`) |
| `test_skill_node.py::test_oval_site_outside_workspace_is_refused_by_name_through_accept` | real accept path, half-extent 0.06 | oval box | fails with the check neutered (`assert True is False`, goal accepted) |
| `::test_oval_1ball_*`, `::test_oval_at_exact_workspace_equality_is_accepted`, `::test_accept_reports_a_missing_box_and_an_outside_site_together`, `::test_oval_refuses_a_box_swept_at_another_dwell_naming_dwell_s`, `::test_check_prints_the_oval_box_line_and_the_workspace_result`, `::test_hop_at_the_125_plus_20_operating_point_passes_the_workspace_rule` | real accept path / `skills/check` | `_good_box_path(oval=...)` | node report only; accept-path pins |
| `::test_stop_mid_carry_logs_why_only_the_after_rest_was_offered` | fake executor with `stop_before_declined` | n/a | fails with the log disabled (`assert any(...)` false) |
| `tests/sim/test_skills_gate.py::test_oval_1ball_schedule_is_every_throw_from_T_and_every_catch_at_C`, `::test_a_small_oval_learner_run_flies_the_choreography_and_keys_memory_on_T` | schedule; real learner run | gate cfg `oval_1ball` | the learner run failed with 15 x `NO_ADMISSIBLE_COMMAND` until the oval cell was swept (nothing mocked) |

## Xfail accounting

None. The unit reports record no `xfail` added and none marked to hide a failure: every refusal the checks
produced on a flown window was resolved by a measured re-base or classified as synthetic test geometry and
re-baselined with its reading recorded (report_refusals.md Task 5), not deferred.

## Verification

Baseline: merge `0f49e876` (skill-stack `4cd33254` + plan `488bec36`); gate on that tree, run 2026-10-17
by the peer session, `./run_tests.sh`: **parallel 6717 passed / 9 skipped (393 s) + serial 4 passed, PASS**
(log `temp/logs/gate_oval_merge_20261017.log`).

- Final-tree gate (`./run_tests.sh`, 2026-10-18): `./run_tests.sh --full` (every tier, nightly included), run 2026-10-18 on the final tree: **parallel 6792 passed, 9 skipped, 1 failed in 308 s; serial 7 passed in 28 s** (`temp/logs/gate_full_p1b_20261018.log`). The one failure, `tests/ros/test_replay_overview.py::test_a_pass_still_exiting_for_an_older_version_neither_reports_computing_nor_blocks_the_requeue`, is a thread race in the GUI replay overview (untouched by P1; the same file raised a PytestUnhandledThreadExceptionWarning in the first run) and passes 3 of 3 re-runs in isolation (11 passed each) - recorded as a load flake, not a regression. The earlier full run on the pre-20 % tree (`temp/logs/gate_full_p1_20261018.log`) was 6787 passed, 5 failed: the five were the oval cell vs the node-default-dwell test (now exempted by design), the three sitting-6 replays at closing ratio 0.248 (the 20 % re-base) and one harness-geometry feed announcement (moved to the feed site)
- Hypothesis depth (`pytest ... --hypothesis-profile=ci-deep` on the files changed, 2026-10-18): `pytest tests/motion/test_unified_cycle.py -q --hypothesis-profile=ci-deep --hypothesis-seed=0` (the only changed test file that uses hypothesis), run 2026-10-18: **87 passed, 2 skipped in 3.22 s** (`temp/logs/cideep_unified_cycle_20261018.log`)
- Serial budget tests (`pytest tests/motion/test_unified_cycle_budget.py tests/motion/test_validate_cycle_budget.py tests/motion/test_solver_worker_lateness.py -q`, 2026-10-18): `pytest tests/motion/test_unified_cycle_budget.py tests/motion/test_validate_cycle_budget.py tests/motion/test_solver_worker_lateness.py -q`, run 2026-10-18 after the last code edit, box idle: **4 passed in 8.65 s**; also green inside both full gates' serial tails (7 passed)
  (checks unit's earlier run: 4 passed; `validate_cycle` on the 57-knot reference min 7.42 ms against the 10 ms budget, seating +0.30 ms, catch +0.25 ms)
- Box re-sweep (four invocations, each run twice, `cmp` per file, BLAS one thread; merged single, hop, columns, oval): sweep 2026-10-18 `temp/sweeps/sweep_P1.sh` (four invocations: single/hop/columns at dwell 0.27, oval at 0.65 - 150 mm, apex 0.95 +/- 0.025 - all at 350/5000/200000/hand 3900, BLAS one thread), each run twice, `cmp` IDENTICAL for all four; merged single->hop->columns->oval; gate_hash 606b53e3912f (first merge 20436164b087 at the 25 % closing threshold; re-swept after the 20 % re-base with every cell identical, four files cmp IDENTICAL again), swept_at 2026-10-18, 20 cells, merged-file md5 5e2ea1b3ba8a1b75718c78b5d3035096; hop and columns cells value-identical to the pre-oval snapshot, self_toss cells changed (see Tradeoffs)
- Minimum passing dwell for the rung-1 cell, the bisection (apex 0.95, sep 150, 350/5000/200k, hand 3900, zero offset, production default hold, cone 30, floor 0.30, closing 0.25 (the 25 % setting the bisection ran under; recorded as run, not re-run at 20 %); dwell snaps to the 25 ms knot grid so values step in groups):

  | dwell (s) | verdict | max residual (deg) | min a_z+g (g) |
  |---|---|---|---|
  | 0.50 | FAIL CUP_SEATING_CONE | 32.11 | 0.820 |
  | 0.51-0.53 | OK | 28.99 | 0.831 |
  | 0.54-0.55 | OK | 26.20 | 0.841 |
  | 0.56-0.58 | OK | 23.60 | 0.851 |
  | 0.59-0.60 | OK | 21.51 | 0.860 |
  | 0.61-0.63 | OK | 19.55 | 0.868 |
  | 0.64-0.65 | OK | 17.81 | 0.876 |

  Minimum passing dwell at the 30 deg cone is **0.51**, with only 1.0 deg of margin at zero offset (the +-20 mm
  offsets read about 1.5 deg more, so 0.51 would shrink the box). The committed cell stays at 0.65: 17.8 deg, 12 deg
  of margin. Closing ratio 0.533 at every dwell, skid 41.6 mm/s (33.4 at 0.50). 
- `colcon build --packages-select jugglebot` (interfaces unchanged): `cd ros_ws && colcon build --packages-select jugglebot` run 2026-10-18 after the last code edit: **1 package finished, exit 0** (`temp/logs/colcon_p1b_20261018.log`); no .msg/.srv/.action changed, so jugglebot_interfaces was not rebuilt

Docs-only tests that read the paths this entry touches: `tests/sim/test_plans_index.py` (plan board) and
`tests/sim/test_logbook_front_matter.py` (this entry's front matter, `related_plan` resolved by bare filename).
The checks unit's measured numbers (survey of 496 gated windows, scratchpad data not committed) are the evidence for the
re-base; they are not a test.

## Outcome

P1 landed: the oval pattern compiles, folds, dispatches, is box-gated at dwell 0.65 and C-CUP-4-checked at
30 deg / 0.30 g, with closing speed refused below 20 %. Rung 1 (`tests/hardware/session_oval_rung1.md`) is
cleared to fly after a full launch restart (new `skill_node` / executor build, GUI service restart and hard
refresh) with `dwell_s` set to 0.65 and a separate `plant_id`. P2 starts after rung 1's outcome is logged.
