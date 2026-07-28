---
title: Tier 8b becomes the shipped toss default — and the four operator documents the flip quietly falsified
type: feature
date: 2026-07-28
status: in-progress
phase: "Self-toss anomaly fixes — toss_tier 8b default"
related_plan: "catch-reach-degenerate-overshoot.md"
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.h
  - config/generated/hardware_config.py
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - tests/ros/test_toss_sequencer.py
  - tests/ros/test_toss_coordinator.py
  - tests/hardware/toss_trace_recorder.py
  - tests/hardware/session_phase8_toss_hardware.md
  - tests/hardware/session_anomaly_fixes.md
  - plans/active/catch-reach-degenerate-overshoot.md
  - logbook/INDEX.md
commits:
  - 9247a33
subsystem:
  - config
  - ros
  - motion
tags:
  - safety
  - testing
  - docs
---

# Tier 8b becomes the shipped toss default — and the four operator documents the flip quietly falsified

## Summary

`jugglebot_operational.toss_tier` flips `"8a" → "8b"`, making the tilt-aimed
displaced throw→catch the shipped default. This is an **operator decision** taken on
this phase's own hardware evidence — 11/11 displaced throws accepted at the 2026-07-27
T4 rung, out to the 70 mm cap, with the deferred A→B reach firing correctly on every
one — and the code it enables has been in the tree since Phase 4 (2026-07-25). The
two drift guards that pinned the old default were re-pointed and mutation-checked.

The larger part of the work was **not** the flip. A tier is not a value: it selects
which of two shipped choreographies every toss goal runs. Changing the default
therefore silently rewrote what four operator-facing documents describe, and three of
those documents would have mis-routed a bench operator — one of them into reverting
this very commit mid-sitting. Those repairs ship **in the same commit as the flip**,
because a commit whose diff line is `toss_tier: "8b"` must not leave a live runbook
telling the operator that rung T3 is Tier 8a.

## Motivation

Two root causes, and they are different in kind.

**(a) A shipped config default must not silently disagree with the constant the FSM
ships.** `toss_tier` has exactly one production read —
`reload_coordinator_node.py:1228` — which passes the value straight into the
`TossSequencer` constructor. If the generated constant and the FSM's serviceable set
drift apart, the machine either refuses every toss goal or silently serves an
unimplemented tier, and nothing in the repo notices. `test_tier_constant_matches_config`
is the single enforcement point for that invariant, and a flip that leaves it pinned
to the retired value converts it from a guard into a failing test somebody will
"fix" by weakening.

**(b) A document that no longer matches the machine is worse than no document,
because the operator acts on it.** This is the class the 2026-07-27 run close-out
spent a whole phase eliminating ("a criterion that fires on CORRECT behaviour is
worse than no criterion"). The flip re-created it in four places at once, and none of
them is reachable by grepping the symbol `toss_tier` — they describe the *tier* in
prose. That is why the implementer's stale-doc sweep, which grepped only the symbol,
found none of them and three independent reviewers found all of them.

## Design

**What the tier actually selects**, walked one cycle at a time, because "8a vs 8b" is
not a label:

| | Tier 8a | Tier 8b |
|---|---|---|
| CHECKING | tier, lead, flight band, `event_vel`, workspace box | **plus** the `\|B − A\| ≤ 70 mm` cap, the closed-form reach bound, and a tilt clamp — all **before** the workspace box |
| POSITIONING | platform **level** at the nominated catch `(x, y)` | platform **tilted** at the config throw site `A = (0, 0)` |
| PREPARING | prime, `catch/prime_hold`, arm | **plus** `catch/pretilt_hold` raised, suppressing the stock announcement pre-tilt |
| THROWING | vertical stroke | **tilt-aimed** stroke |
| BALL_IN_FLIGHT | — | the deferred **A→B platform reach** publishes at `t_release` |

**Direction of risk, stated correctly.** The CHECKING gate gets strictly *tighter*: a
goal accepted at `|x| ≤ 150 mm` under 8a now refuses above `70 mm`, so on that path
the machine can only refuse more. **The choreography does not get smaller**, and it
would be wrong to claim the change "can only cause the machine to refuse more goals,
never to command a larger motion" — 8b tilts the platform, suppresses the stock
pre-tilt and puts a platform reach *into ball flight*, none of which 8a does. Both
halves are true and only stating both is honest; the runbook says so in those words.

**Discontinuity check** on the one plan the flip newly puts in the command path. The
deferred A→B reach is installed at release from a pose held open-loop since
POSITIONING, so its seed twist is ≈0 and it is a plain rest-to-rest quintic — no C²
discontinuity at the seam, and C-CATCH-1 bounds any unrequested excursion. With the
shipped `_CATCH_TILT_THROUGH_RATE_RADPS = 0.0` it manufactures no arrival twist at
all. Measured directly on 2026-07-27: `ZSEAT-4`'s Tier-8b rows read `rx`/`ry` span
`0.0000°` and position span `0.0000 mm` over `[release − 0.10 s, release]` on 11/11,
first `catch/dynamic_target` at `+0.013…+0.050 s`, never negative.

**Blast radius.** `reload_coordinator_node.py` holds four `TIER_8B` branches (`:1231`,
`:1472`, `:1551`, `:1590`) and every one is on the toss path — the reload action never
reads the tier, so the shipping reload choreography is untouched. No Teensy sketch
reads `TOSS_TIER` (0 hits across all three firmware trees, excluding the generated
header), so **no flash**. Nothing at 40 Hz, no feedforward path, no plan-install rate,
no park-band threshold, and nothing on the 0x6D0 conduit or the kind-3 clobber path.

## Discussion

### Re-pin the guard, or widen it to the serviceable set?

The sitting entry's own Open Question 1 recommended **widening**
`test_tier_constant_matches_config` to `JB_OP_TOSS_TIER in (TIER_8A, TIER_8B)`, on the
reasoning that re-pinning "fires again the next time the operator flips it". That
recommendation is **retracted** (§ Withdrawn claims of the sitting entry), and this
entry is where the reasoning lives.

A set-membership assertion cannot detect **drift** — it detects only an
*unimplemented* tier. That is already covered twice: the FSM rejects it at runtime
(`toss_sequencer.step`'s `if self.tier not in (TIER_8A, TIER_8B): _reject('TIER')`)
and `test_toss_goal_rejections_via_execute[tier-…]` drives `'9z'` through the
generated config. So widening would have duplicated an existing check while silently
dropping the **only** check on *which* tier ships — and which tier ships changes what
the machine physically does on every toss goal. The clause "it fires again the next
time the operator flips it" describes the guard **working**: a default that changes
commanded motion should cost one deliberate, reviewed edit. Concrete failure mode the
widened version admits: someone reverts the YAML to `8a` while chasing an unrelated
bench problem, the full suite stays green, and the robot ships the wrong choreography
with every artefact in the repo claiming otherwise.

### The plan brief's hedge about the WORKSPACE row was empirically wrong

The phase brief said to rebuild the `workspace` rejection row from "displacement cap
70 mm from throw site (0,0), per-axis box `TOSS_XY_LIMIT_MM = 150`", implying a
lateral construction. **That is impossible.** With `A = (0, 0)`, displacement ≤ 70
forces `|x|, |y| ≤ 70 < 150`, so no goal can pass the cap and violate the box
laterally — the cap answers first (`toss_sequencer.py:665`, strictly before the box at
`:678`). The box's lateral half is **structurally unreachable** through `_execute_toss`
under 8b, and the `z` band is the only reachable WORKSPACE branch. This was found by
driving the real `_execute_toss` over a coordinate sweep before either assertion was
written, not by reading the gate — which is the point of the probe-first rule. It is
also the second time in this run that a plan-author hedge turned out false on ground
truth.

Given the `z` band is the only option, the row is built at `x = 60, z = 300` rather
than `x = 0, z = 300`. **Why a live displacement matters**: a zero-displacement row
reaches the same branch while proving strictly less. If the cap ever collapsed toward
zero and began refusing every real displaced goal, a zero-displacement row would still
pass and report the WORKSPACE branch as healthy while it had become unreachable for
any goal a human would send. At `x = 60` the 8b gates genuinely run and pass first.

The lateral half keeps its coverage at FSM level in
`test_workspace_precheck_rejected`, which drives `(200,0,170)`, `(0,−200,170)` and
`(0,0,225)`. Note precisely: that test is **8a-pinned**, not tier-agnostic — it builds
`TossSequencer` without a tier and rides the dataclass default `tier: str = TIER_8A`.
8a is the only tier where the lateral branch is reachable, so the placement is right,
but the docstring saying "tier-agnostic" was wrong and is corrected.

### A new row, not a re-labelled one

The existing `workspace` parametrize row could have been re-labelled to assert
`REJECTED_DISPLACEMENT` on its original `x = 200` goal. It was not. A parametrize id
reading `[workspace-REJECTED_DISPLACEMENT]` is a permanent trap for the next reader,
who will reasonably assume the workspace gate mints DISPLACEMENT. Keeping the
`workspace` id honest and **adding** a `displacement` id also makes the suite delta
unambiguously `+1` rather than a rename plus an add. The `displacement` row is the
tier detector (it goes red at 8a, where `x = 200` mints WORKSPACE); the `workspace`
row is a tier-invariant coverage anchor. Both roles are stated in the docstring so
neither is mistaken for the other.

### Four documents the flip falsified — and why they ship in this commit

Three reviewers working from independent starting points converged on the same defect
class, and it is the one the code-side contract cannot protect: the tier lives in
prose, not in a symbol.

1. **`session_phase8_toss_hardware.md` rung T3 (HIGH, two lenses independently).**
   T3 says "catch at the four ±60 mm corners (throw = catch site, **Tier 8a**)" and
   its PASS rationale is "this exercises the platform pre-positioning (the POSITIONING
   move to the nominated x, y)". Traced against the flipped tree: 60 mm is inside the
   70 mm cap and inside the reach bound at `T ≈ 0.70 s` (≈171 mm, jerk-bound), the
   required aim ≈1.4° is far under the clamp, so the goal is **ACCEPTED as an 8b
   displaced throw**. `_toss_positioning_xyz` then commands `release.pretilt_pose_stow`
   — the platform pre-tilts at `(0, 0)` and never makes the move the rung exists to
   test. Worse, T3's `(0, +60)` corner fires an un-gated 8b displaced throw into the
   `+y` hemisphere the Phase-4 asymmetry map flags weak, at `T ≈ 0.70 s`, below the
   `T ≥ 0.80 s` that **T4 itself stipulates** for displaced throws — under a rung
   labelled 8a. The failure mode is a landing bias, i.e. a real ball on the floor, and
   the operator scoring `< 3/5` would route to "platform pre-positioning", a subsystem
   that never ran. Fixed with a decision banner (run T0–T3 on an 8a build, or run 8b
   and skip the `+y` corner), a tier-sensitivity warning on T3 itself, and a rewrite
   of T4's now-false "LAST; gated" / "enable 8b in config" framing.

2. **`toss_trace_recorder.py`'s `REJECT_WIRE_MAP` (all three lenses).** It told the
   operator `'REJECTED_TIER': 'goal/config: JB_OP_TOSS_TIER must be 8a'`. The
   implementer declined to fix it, reasoning that the string had been wrong since
   Phase 4 so this change does not falsify it. **That reasoning does not survive**:
   before the flip the config *was* 8a, so following the hint was a no-op; after it,
   following the hint reverts a shipped operator decision mid-sitting, and the only
   thing that would catch it is the next full-suite run. Fixed here — one entry — and
   `REJECTED_DISPLACEMENT` / `REJECTED_TILT_CLAMP` added, because DISPLACEMENT just
   became reachable by default and the 2026-07-27 sitting already logged 3× of it with
   no decoder hint. `REJECTED_POSITION` is still missing and stays with the tracked
   `REJECT_WIRE_MAP` hygiene follow-up; adding two codes the new default makes
   reachable is not the same as pre-empting that phase.

3. **`plans/active/catch-reach-degenerate-overshoot.md` Phase 4 (two lenses).** Its
   status cell still said "**STILL OPEN: nobody has asked the operator whether they
   SAW the balls touch the cup and leave** — decays by the hour", answered on
   2026-07-28; and "Score any CAP-WORK replay from a **clean 8a worktree**", which
   stops being obtainable from `HEAD` the moment this commit lands. Both annotated,
   the latter now naming `6641400` explicitly.

4. **`session_anomaly_fixes.md`'s EXECUTED safety box (narrative lens).** Not caused
   by the flip, but in the same commit's blast radius and safety-ranked above it — see
   the sibling commit's discussion.

### The instrument would have scored a correct capture as a failure

The implementer's handoff prescribed
`toss_trace_recorder.py check <trace> --reject --timeline` as the analysis command for
the new rejection rows, describing RJ-1 as "exactly the ordered-CHECKING-gates check".
**It is not.** RJ-1 hard-requires *exactly one* `REJECTED_NO_BALL` line in the window
(`check_rj1`: `if len(nb) == 1: PASS else: FAIL`). A correct `REJECTED_DISPLACEMENT`
capture yields `len(nb) == 0` and prints `FAIL: 0 REJECTED_NO_BALL lines (expected
exactly 1) … an earlier-code reject means that precondition wire is unhealthy; fix
before the dry capture` — with an **empty hint**, since DISPLACEMENT had no wire-map
entry. The operator would chase a non-existent precondition fault or abort the
sitting.

This is precisely the "instrument validated only against the broken case" trap, and it
was caught before transcription. The runbook does **not** prescribe `--reject` for
these rows; it scores them off the node's own outcome line (ground truth, no
instrument) and carries an explicit banner explaining why `--reject` is wrong *here*
and still correct for its own no-ball row. The section is two-sided by construction:
`TIER-A`/`TIER-B` are the goals that must be **flagged**, `TIER-C`/`TIER-D`/`TIER-E`
the goals that must be **accepted**, and every threshold is a measured number
(`< 0.02°`, `< 0.2 mm`, `+0.000…+0.100 s`, `50–75 mm`) rather than an artefact of the
instrument's control flow.

### Two rows that are NOT what the handoff proposed

The handoff's `TIER-C` was labelled "(NO MOTION): send a Toss goal at catch
`(0.0, 0.0, 170.0)`", PASS = "the outcome is neither `REJECTED_TIER` nor
`REJECTED_DISPLACEMENT`". Traced through `step`'s CHECKING block under 8b: tier ok,
lead ok, flight band ok, displacement `= hypot(0,0) = 0` so no DISPLACEMENT, tilt
exactly level so no TILT_CLAMP, `event_vel ≈ 3.93 m/s` in band, `|z − 170| = 0` so no
WORKSPACE, `A` xy `= (0,0)` so no second WORKSPACE. **CHECKING advances to
POSITIONING and the goal throws.** The label promises a static row, the machine
executes a full stroke with a possibly-unloaded hand and an unprepared operator, and
the PASS criterion is satisfied *by that throw* — nothing in the check flags the
surprise. Rewritten to run **un-armed**, so it refuses on a precondition code and the
"the envelope gates passed" property is read from the reject code rather than from a
throw.

Separately, the co-located `B == A` case under 8b **has never run on hardware**: all
11 validated T4 throws were displaced. At zero displacement the release state is
bitwise identical to 8a (`toss_release.py`'s documented degenerate identity), but the
8b *wrapper* still runs — `catch/pretilt_hold` is raised unconditionally at PREPARE
for 8b (`:1472`, no displacement predicate) and a 0 mm deferred reach still publishes
at `t_release`. That is the shape **every toss row in every section above** sends, so
`TIER-D` scores it and runs before any of them are re-scored. `/catch/pretilt_hold`
had to be added to the shared recording list for it — a stranded gate is unrecoverable
from a bag that lacked the topic.

## Implementation

- `config/hardware_config.yaml` — `toss_tier: "8a" → "8b"`, plus a rewrite of the
  comment block, which still said Tier 8b "is REJECTED_TIER until the plan's Phase 4
  lands behind this same key". The block now describes both serviceable tiers, states
  that anything outside `{8a, 8b}` is `REJECTED_TIER`, and records the default's
  change history. Comment-only: regeneration after the edit produced zero change in
  any generated consumer, which is also the proof that YAML comments do not propagate.
- Six generated consumers — one value line each, verified as the deterministic codegen
  output of the edited YAML (regenerated twice; second run left the tree clean).
- `tests/ros/test_toss_sequencer.py::test_tier_constant_matches_config` — re-pinned to
  `TIER_8B`, with the default's change history recorded inline so a future reader
  meeting this guard red can tell from the failure site which value is intended.
  `TIER_8A` remains a live import (`:1049`), so no dead import.
- `tests/ros/test_toss_coordinator.py::test_toss_goal_rejections_via_execute` — new
  `displacement` row (`x = 200`, asserts `REJECTED_DISPLACEMENT`); `workspace` row
  re-pointed to `(x = 60, z = 300)` and still asserting `REJECTED_WORKSPACE`.
  Parametrize rows **13 → 14**, net `+1` test (a row was added, none renamed or
  removed — so the suite delta is unambiguous).
- `tests/hardware/toss_trace_recorder.py` — `REJECT_WIRE_MAP`: `REJECTED_TIER` hint
  corrected to `{8a, 8b}` with an explicit "do NOT fix this by reverting the default";
  `REJECTED_DISPLACEMENT` and `REJECTED_TILT_CLAMP` added; `REJECTED_WORKSPACE`
  qualified with the lateral-unreachability note.
- `tests/hardware/session_phase8_toss_hardware.md` — § TIER decision banner, T3
  tier-sensitivity warning, T4 gate/no-op rewrite.
- `tests/hardware/session_anomaly_fixes.md` — new **§ SECTION TIER** (`TIER-PREREQ`,
  `TIER-A`, `TIER-B`, `TIER-C`, `TIER-D`, `TIER-E`); `/catch/pretilt_hold` added to
  the shared recording list.
- `plans/active/catch-reach-degenerate-overshoot.md` — Phase 4 cell annotated, Phase 4
  detail section with an Outcome paragraph added.

**Mutation check (the anti-vacuity gate).** With the YAML scratch-flipped back to
`8a` and config regenerated, `pytest tests/ros/test_toss_coordinator.py
tests/ros/test_toss_sequencer.py -q` → **2 failed, 183 passed**:
`test_tier_constant_matches_config` (`assert '8a' == '8b'`) and
`test_toss_goal_rejections_via_execute[displacement-REJECTED_DISPLACEMENT]`. Restore
verified byte-identical, re-run green at 185. The `[workspace-REJECTED_WORKSPACE]` row
stays green on both tiers **by design** — the `z` band is tier-agnostic, so it is the
stable coverage anchor while the `displacement` row is the tier detector.

## Verification

<TRIPLE>

Scoped runs, all 2026-07-28 on the Jetson in the project venv:

- `pytest tests/ros/test_toss_coordinator.py tests/ros/test_toss_sequencer.py -q` →
  **185 passed in 5.91 s**, against the sitting entry's recorded 184-passed 8a
  baseline — delta exactly `+1`, the added parametrize row, zero failures.
- `pytest tests/ros/ -q` → **1291 passed in 72.76 s** (full ROS-side blast radius of
  the config flip).

**Coverage traced, not assumed.** `grep -rln TOSS_TIER tests/ …` returns three files:
the two `tests/ros/` guards, which read the key, and `tests/hardware/toss_trace_recorder.py`,
which names it only in an operator hint string and is not collected by pytest — so the
recorder edit in this commit has **no automated coverage**, which is stated rather than
papered over. `REJECT_WIRE_MAP` has no test pinning its contents; the only consumer is
`check_rj1`'s hint construction, and the two added keys are string values in a dict
literal. The markdown edits (`plans/`, `tests/hardware/*.md`) are read by no test at
all; only `logbook/` is parsed, by `sim/analysis/logbook_search.py` via
`tests/sim/test_logbook_search.py` — and that suite's known blind spots (it skips
`INDEX.md` outright via `_SKIP_FILES`, and silently `continue`s past any entry whose
front matter lacks a `title`) mean a green result there is **not** evidence this
entry's front matter or the INDEX row are well-formed. Both were checked by eye.

**No hardware verification yet.** Every claim about how 8b behaves at the machine is
inherited from the 2026-07-27 T4 rung, and that evidence is **displaced-only**. The
co-located path, the deployment itself, and the two rejection gates are unvalidated
until § SECTION TIER runs.

## Outcome

The flip is landed and the drift guards detect drift again. What is **deferred to the
operator**, in the order the runbook executes it:

1. **`colcon build --packages-select jugglebot` + relaunch.** Until then the robot
   runs Tier 8a while every artefact says 8b, invisibly. `TIER-PREREQ` is a direct
   read of the installed file and is the first row of the next sitting. **No firmware
   flash**, no interfaces rebuild, no config regeneration.
2. **§ SECTION TIER**, rows `TIER-PREREQ` → `TIER-E`. Two refusal rows (no motion),
   one un-armed accept row (no motion), two throwing rows — of which `TIER-D` is the
   first hardware run of the co-located 8b path.
3. **`session_phase8_toss_hardware.md` § TIER decision** — choose an 8a build for
   T0–T3, or run 8b and skip T3's `(0, +60)` corner. Record which.

## Open Questions

1. **The sibling BallButler repo is dirtied by regenerating config here.**
   `config/generate_config.py` mirrors a generated header into
   `~/Desktop/BallButler/ball_butler_main/hardware_config.h`, and that repo's copy is
   currently ~54 insertions / 3 deletions behind (`MOTOR_KT_NM_PER_A` 0.0624→0.057,
   the whole `TORQUE_FF_*` block, and now `TOSS_TIER`). Pre-existing drift; a different
   git repo, so nothing here can or should commit it. The operator should know that
   regenerating config in Jugglebot dirties BallButler and that BallButler's committed
   header is many changes stale.
2. **`REJECTED_POSITION` is still absent from `REJECT_WIRE_MAP`.** Left with the
   tracked hygiene follow-up in `logbook/2026-07-25-toss-rejected-not-levelled.md`
   § Follow-ups, which also proposes a drift guard for the map. Worth scheduling: the
   map has no test pinning it against the FSM's reject codes, so it will rot again.
3. **`plans/active/single-ball-toss.md` still records 8b as shipping behind
   `JB_OP_TOSS_TIER='8a'`** (lines 186, 294) and its T4 instruction at line 330 ("Set
   `JB_OP_TOSS_TIER=8b` for the session") is now a no-op. None of it is misleading —
   they are historical Phase-4 records — but that plan's Phase-4 status cell wants a
   note now that the tier is the default and decision (d) has re-scoped the ±150 mm
   request into a full 8b programme. Out of this phase's scope; not touched.
4. **Is the 70 mm cap the right default now that 8b is?** Decision (d) orders raising
   it as part of the displaced-throw programme. The cap is currently the intersection
   of the 80 mm reach envelope with the Rung-2a clean box; raising it needs its own
   plan and its own gate, and is deliberately untouched here.

## Related

- `logbook/2026-07-28-anomaly-fixes-validation-sitting.md` — the sitting whose T4 rung
  is the evidence for this default, and whose § Discussion → *Operator testimony and
  decisions* records the operator's decision (d) that this flip serves.
- `plans/active/catch-reach-degenerate-overshoot.md` § Phase 4 — the plan phase this
  lands under.
- `plans/active/single-ball-toss.md` § Phase 4 — where Tier 8b itself was implemented
  (2026-07-25).
- `logbook/2026-07-25-toss-rejected-not-levelled.md` § Follow-ups — owns the remaining
  `REJECT_WIRE_MAP` hygiene.
- `tests/hardware/session_anomaly_fixes.md` § SECTION TIER — the bench rows.
- `tests/hardware/session_phase8_toss_hardware.md` § TIER — the T0–T4 tier decision.
