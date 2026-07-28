---
title: Self-toss anomaly fixes — Phase E (displaced throws to ±150 mm)
type: feature
date: 2026-07-25
status: in-progress
phase: "Phase E"
related_plan: "single-ball-toss.md"
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/reload_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/toss_release.py
  - ros_ws/docs/catch_reach_envelope.md
  - sim/toss_gate.py
  - tools/probes/displaced_reach_frontier.py
  - tools/probes/README.md
  - tests/ros/test_trajectory_node.py
  - tests/ros/test_toss_sequencer.py
  - tests/ros/test_toss_coordinator.py
  - tests/ros/test_toss_integration.py
  - tests/ros/test_reload_sequencer.py
  - tests/ros/test_reload_coordinator_node.py
  - tests/motion/test_toss_release.py
  - tests/sim/test_toss_gate.py
  - tests/hardware/session_anomaly_fixes.md
  - tests/hardware/toss_trace_recorder.py
  - plans/active/single-ball-toss.md
  - plans/active/PROMPT-anomaly-fixes-orchestration.md
commits:
  - ad80ff0
subsystem:
  - ros
  - motion
  - config
tags:
  - safety
  - testing
  - docs
  - kinematics
---

# Self-toss anomaly fixes — Phase E (displaced throws to ±150 mm)

## Summary

The Tier-8b displaced throw gets its ordered working range: the throw site `A` is
now the platform's **live commanded pose** instead of a config constant, the catch
reach envelope is re-scoped by contract **C-REACH-1** to bound *unrequested drift*
about the *declared* catch point rather than silently also capping *requested*
reach, the displacement cap is re-based on its own evidence at **150 mm**, and a
CAUGHT toss **stays** at its catch pose so sessions can chain. A fifth change
landed at finalize from an independent review: the new reload centred-ness gate's
tolerance had **zero** budget for the reload's own pre-tilt swing and so admitted
the exact state it exists to refuse. The phase ships with one **known limitation**
— chaining is refused *at* the cap — documented rather than papered over.

## Motivation

Four Tier-8b goals on 2026-07-27 (bag `2026-07-27_16-07-30`, nominal displacements
141 / 127 / 113 / 113 mm, run under an uncommitted 250 mm cap override) were **all
four refused**: `catch target 146/131/117/117 mm from the armed hold pose exceeds
the 80 mm reach envelope`. 4/4.

The refusal is not a tuning problem. `trajectory_node`'s catch reach envelope
exists to bound how far a *tracker refinement* may drag the platform from where the
catch was planned — **unrequested** motion. But it was centred on the pose held at
arming, and the Tier-8b deferred `A → B` reach is a **requested** motion published
at `t_release`. So the envelope was doing two jobs with one number, and the second
job's verdict could only ever arrive **after the irreversible commitment**, with
the ball airborne and unsavable. That is a whole class of failure, not one bad
constant — which is why it got a contract rather than a wider radius.

## Design

**1. `A` is the live commanded pose, and it is never defaulted.**
`trajectory_node` publishes `trajectory/commanded_position`
(`geometry_msgs/Point`, 5 Hz, only while seeded+streaming).
`toss_throw_site_mm` is **retired** and `compute_release_state_tilted`'s
`throw_site_xy_mm` became a **required** argument — one enforcement point, a
`TypeError` on omission and an explicit `ValueError` on `None`. No fresh read ⇒
`REJECTED_POSE_UNKNOWN`, placed strictest-first inside the 8b block because every
other displaced gate is a function of `|B − A|` and is meaningless without `A`.

**2. Contract C-REACH-1** (`ros_ws/docs/catch_reach_envelope.md`) re-centres the
envelope on the **declared** catch point `B`, published on `catch/reach_center` one
FSM tick before the arm raise and consumed-and-cleared by the next `arm_catch` call
of any kind. The 80 mm radius does not move — drift is bounded exactly as tightly,
about the place the catch is actually supposed to happen. The reload declares
nothing and stays byte-identical.

**3. The cap is `jugglebot_operational.toss_max_displacement_mm` = 150 mm**,
ctor-resolved, re-based on its own evidence now that it can no longer inherit the
envelope's.

**4. A CAUGHT toss STAYS** (`ACTION_STAY` = the RECENTER ladder minus `go_home` —
not-calling-`go_home`, no new mechanism). Every not-caught terminal keeps
`SAFE_ABORT` unchanged.

**5. (finalize) The reload centred-ness tolerance is
`envelope − hand_catch_offset·sin(MAX_TILT_DEG)` = 66.53 mm**, not the bare 80 mm
envelope radius.

## Discussion

### The convergent finding I could not fix, and shipped as a documented limitation

Two of the three independent reviewers — a physics lens and a contract lens,
starting from different places — landed on the **same** defect. That convergence is
the strongest signal the panel produces, so I verified it first, in-process against
the production objects rather than by reading.

It is real. The catch **deliberately** parks the platform *centroid* outside `B` so
the *cup* lands *on* `B` (`centroid = landing − hand_catch_offset · platform_z`).
Measured through the production chain at `B = (−150, 0, 170)`, `T = 0.80 s`: the cup
ends at exactly `(−150.000, 0)` — bit-on `B` — and the centroid at `(−153.100, 0)`.
`trajectory/commanded_position` publishes `_current_state()[0][:3]`, the
**centroid**. So the next goal reads `A = −153.10`, and driving the *real*
`TossSequencer` from there:

| next goal `B` | verdict |
|---|---|
| `(0, 0, 170)` (back to centre) | `REJECTED_DISPLACEMENT` (153.10 > cap 150) |
| `(−150, 0, 170)` | `REJECTED_WORKSPACE` (\|ax\| 153.10 > `TOSS_XY_LIMIT_MM` 150) |
| `(−100, 0, 170)` | `REJECTED_WORKSPACE` |

The offset is `hand_catch_offset · sin(receive tilt)` = `64.78 · sin(2.73°)` =
3.10 mm at `T = 0.80` (4.05 mm at 0.70), i.e. **2.07 % of the displacement**, so it
crosses the box at `|B| ≈ 147 mm` and crosses the cap on the return leg at any
radius ≳ 147 mm in **every** direction. Chaining — the headline capability — works
below ~146 mm and is refused at exactly the range the operator ordered.

**Root cause, stated precisely, because the reviewers' framing was diagnostically
right but overstated the consequence.** `compute_release_state_tilted` documents
`throw_site_xy_mm` as the point the release **cup** sits at ("the tilted release xy
sits exactly AT nominal A"); the wire supplies a **centroid**. The two agree
exactly whenever the platform is level — which is every state the machine has ever
been in before `STAY` — and diverge by the cup swing when it is not. **This is not
an aim error**: `A` is *nominated*, not observed, and POSITIONING commands the
platform so the cup is at `A`, so the aim stays self-consistent whatever value is
nominated. It is a **gate-frame** error: the `±150 mm` planning box and the 150 mm
cap are nominal-frame bounds evaluated against a centroid-frame observation.

**Why I did not fix it.** Every candidate requires a decision that is not mine:

- **(a) publish or derive the cup xy** — the root-cause fix, but it needs the
  commanded *rotation*. The phase deliberately kept rotation off that wire because
  C-LEVEL-1's correction rewrites commanded rotation and never position, so a
  position-only 3-vector is identical in the corrected and uncorrected frames and
  cannot be double-corrected by a consumer that feeds it back as a request. The
  toss *does* feed it back (`A` → pre-tilt pose → `go_to_pose`), so a cup xy
  derived from the corrected rotation re-opens exactly that hazard. Reversing a
  documented implementer fork, unilaterally, at finalize, to close a
  non-hazardous capability gap is the wrong trade.
- **(b) widen the A-side box and the cap by one cup-swing allowance** — the
  frame uncertainty is **bidirectional** (`true ∈ [meas − 13.47, meas + 13.47]`),
  so the fail-closed reading *tightens* to 136.5 mm and strands harder, while the
  permissive reading *loosens* an evidence-based cap by 13.47 mm. Two fudge
  factors pointing opposite ways is the shape CLAUDE.md's "contracts over patches"
  rule exists to refuse.
- **(c) lower the cap to ~145 mm** — violates the operator's stated "at least
  ±150 mm" ask.

So: pinned by a test, written into the contract as residual 7, recorded as a plan
open question, and — the part that actually matters for the next sitting — the
**runbook was corrected so it cannot mis-route the operator**. As shipped for
review, runbook row DISP-6.2 told the operator that a `REJECTED_DISPLACEMENT` means
"the throw site did not follow the platform", which is the **opposite** of the
truth: the throw site followed the platform *too faithfully*, into the centroid
frame. DISP-5 commanded exactly the geometry that strands. An instrument that
scores correct work as a failure is worse than no instrument — it routes the work
back for rework and burns a powered sitting — so the ladder now carries a KNOWN
LIMITATION box with the measured numbers, a `go_home` between cap-magnitude
attempts, and disambiguation for both reject codes.

### The finding I did fix, because it was a live hazard

The regression lens flagged the new `REJECTED_NOT_CENTERED` gate's tolerance at
confidence LIKELY. Numerically it is **PROVEN**, and worse than claimed.

The reload declares no reach centre, so its arm raise centres the 80 mm envelope on
the **park**. The target that envelope then judges is *not* the catch point:
`_compute_catch_command` pulls the centroid back along the tilted platform-z by
`hand_catch_offset_mm`. `compute_catch_orientation` **clamps** at
`MAX_TILT_DEG = 12°`, and real BB arrivals are 18–40° off vertical — so the clamp
**saturates on every reload** and the shift is a *constant* 13.469 mm. Measured
across 18 / 25 / 30 / 40° arrivals: 13.469 mm every time, invariant.

With the tolerance set to the envelope radius there is **zero** budget for it:

| park (adverse direction) | gate admits | resulting excursion | envelope |
|---|---|---|---|
| 60 mm | yes | 73.47 mm | accepts |
| 66 mm | yes | 79.47 mm | accepts |
| **70 mm** | **yes** | **83.47 mm** | **REJECTS** |
| 79 mm | yes | 92.47 mm | REJECTS |

The rejection arrives *after* `ACTION_SEND_THROW` — `ball_butler` publishes the
announcement synchronously inside `bb/throw_at_target`, so BB's countdown has
started and the ball is unsavable. That is the DISP-7.1 "E-STOP the session"
condition, reached through a gate that said yes, and it is the *same class* as the
phase's headline failure one path over. Worse, the ladder walks straight into it:
DISP-3 catches at 70 mm, `STAY` parks there, and the next rung opens with a Reload.

Fixed by deriving the tolerance rather than reusing a number that merely looked
semantically right. It costs nothing in normal operation (a `go_home`'d platform
reads 0 mm) and a refusal moves nothing, so the direction of error is safe.

### What I refuted

- **`REACH_*` "pinned by the drift-guard test"** (physics, MEDIUM) — the claim in
  the comment was false, but I fixed it by making it *true* (adding the three
  assertions) rather than deleting it. This mattered more than it looks: at the old
  70 mm cap the closed-form bound never bound (`d_max ≥ 83.2 mm`), so a stale local
  copy was harmless. Phase E made the bound **live**, so an operator lowering
  `leg_jerk_limit_mmps3` to 15000 for a cautious sitting would have had a 150 mm
  goal at `T = 0.70` pass CHECKING (module bound 171.5 mm) while the real budget
  allowed 85.8 mm — and the deferred reach would then reject `TOO_FAST` at
  `t_release`. The phase created the hazard the missing pin protects against.
- **The seed-bug comment overclaimed.** It said the bug was "exactly why every
  published cell reads 0/4 or 4/4". The implementer's own **post-fix** artefact
  says otherwise: 47 of 48 cells still read 0/4 or 4/4, one reads 1/4. The bug's
  real tell was `err_mean == err_worst` *exactly*; the near-bimodality is the
  contact model and survives the fix. Corrected, because a future session reading
  that comment would treat a 0/4 cell as a real 0 % direction.
- **DISP-5.3's expected jerk** quoted the closed form's **platform-space** figure
  (17578) as the expected reading of `peak_leg_jerk`, a **leg-space** measurand.
  Re-aggregating the phase's own gate run (`temp/reports/toss_8b_phaseE_seed0.json`,
  80 trials at the 150 mm ring): mean 9371, max **9742** — 0.55×. An operator
  seeing 9700 against a row demanding 17578 would have read a 45 % shortfall as
  evidence the reach was not spanning `A → B`.
- **`streaming` freshness** (regression, MEDIUM) — the *behaviour* claim is
  correct (the reload's `streaming` is a sticky last-value while
  `platform_centered` is freshness-gated, so a dead `trajectory_node` mints a
  geometry code for a link fault), but the implementer scoped it out deliberately
  and with a written reason: freshness-gating it changes when the **shipping**
  reload refuses, on a path this phase promised to leave byte-identical. What was
  actually defective was the *comment* claiming an ordering guarantee the sticky
  value cannot deliver. Comment corrected, behaviour deferred, and DISP-7.3 now
  tells the operator to confirm `/trajectory/status` is live before believing a
  `NOT_CENTERED`.
- **The 8a declaration giving up the envelope's second job** (regression, MEDIUM)
  — verified as a real behaviour change, but it is a deliberate trade, not an
  oversight: the envelope existed to bound *drift*, never to verify *arrival*, and
  leaning on it for arrival was the coupling C-REACH-1 removes. Recorded as
  contract residual 6 rather than mechanised, because the suggested guard adds a
  mechanism to protect a property that was always accidental.
- **`throw_site_known: bool = True`** (regression, LOW) — PROVEN and fixed. It was
  fail-**open** in a dataclass family where every sibling defaults fail-**closed**
  with an explicit comment saying why, and it pairs with `throw_site_xy_mm`'s
  `(0.0, 0.0)`, so a caller omitting both would site the throw at the workspace
  origin and be believed — the exact phantom site this phase retired a config key
  to kill. Flipping the default immediately caught one test silently relying on it
  (`test_8b_stream_equals_8a_plus_reach_catch`), which is the finding validating
  itself.
- **The pending-declaration leak** (contract, LOW, NOT-PROVEN) — I did not
  construct it either, and the same executor congestion that drops one client's
  `arm_catch` calls generally drops the next one's. But the code genuinely has no
  clear other than a mode change, so it is written into the contract as residual 5
  with the cheap mechanism named if it ever needs closing.

### Decide-and-document forks

- **Envelope mechanism: declared centre, not "centre on the first accepted
  post-arm target".** On the shipping reload path the first target can be a corrupt
  tracker landing estimate (18 corrupt reload tracks in the 2026-07-27 capture;
  27–46 envelope WORKSPACE rejects per flight in the 2026-07-23 session). Under the
  alternative that corruption would **define its own envelope** and drag the
  platform to it — re-opening the exact drift failure the envelope exists to bound,
  on a path this phase was told to leave byte-identical.
- **New topics carry `geometry_msgs/Point`, not a `jugglebot_interfaces` type** —
  a split build of `jugglebot_interfaces` against `jugglebot` is a known *silent*
  failure mode on this machine. Net effect: `colcon build --packages-select
  jugglebot` alone is sufficient.
- **The reload seam closed by refusal, not auto-return** — an auto-`go_home`
  injects new commanded platform motion into a choreography no hardware session has
  ever run that way. The refusal costs the operator one `go_home` and costs the
  machine nothing.
- **DISP-2's demonstration pose moved `(150, −150) → (140, −140)`** — 150.0 is the
  *exact* value of `TOSS_XY_LIMIT_MM`, and the new A-side box check refuses above
  it. The rung tells the operator to drive there by hand, so the commanded pose is
  an arbitrary float: `(150.4, −149.7)` echoes as "~(150, −150)" and refuses
  `REJECTED_WORKSPACE`, a code the rung's ABORT column did not carry. `(140, −140)`
  keeps the whole demonstrative value (still a 198 mm displaced goal under the
  retired config site) with 10 mm of margin.

## Implementation

- `trajectory_node.py` — `trajectory/commanded_position` publisher (position-only,
  streaming-gated); `catch/reach_center` subscription; `_pending_reach_center`
  read-and-cleared in `_svc_arm_catch` before the idempotent early return;
  non-finite declarations dropped loudly (a NaN centre makes every
  `excursion > envelope` comparison False and silently **disables** the gate).
- `toss_sequencer.py` — `REJECTED_POSE_UNKNOWN` strictest-first; ctor-resolved
  `max_displacement_mm`; `ACTION_STAY`; `throw_site_known` now fail-closed.
- `reload_coordinator_node.py` — live-pose read at goal accept; `_publish_reach_center`;
  `_toss_stay`; `_platform_centered` on the new derived `_RELOAD_CENTERED_TOL_MM`.
- `reload_sequencer.py` — `REJECTED_NOT_CENTERED` after mocap/streaming and before
  `ACTION_PRIME_HAND`, so nothing moves and BB is never asked.
- `motion/trajectory/toss_release.py` — `throw_site_xy_mm` required.
- `tools/probes/displaced_reach_frontier.py` — new committed probe.

## Verification

**Full suite:** `python -m pytest tests/ -q`, run 2026-07-29 on the Jetson in the
project venv: **4140 passed, 3 xfailed, 196 warnings in 1409.60 s (0:23:29)**,
exit 0.

**Delta accounted, exactly.** Baseline at HEAD `956a4b8` was **4096 passed,
3 xfailed in 1428.61 s**. Net **+44**:

- **+40** from the implementer's new tests and parametrisations (measured against a
  detached baseline worktree: 4099 collected at HEAD vs 4139 after their work);
- **+4** from finalize, verified by `--collect-only`:
  `test_centered_tolerance_leaves_room_for_the_reload_pretilt_swing` (1) and
  `test_chaining_at_the_cap_is_refused_known_limitation` (3 params).

**xfail unchanged at 3** — no test was weakened, skipped or deleted;
`git diff | grep -cE '^\+.*(xfail|pytest\.mark\.skip|pytest\.skip)'` = 0. The five
removed `def test_` lines in the diff are all renames whose replacements carry a
superset of assertions.

**Scoped runs, all 2026-07-29 in the venv:**
- `python -m pytest tests/ros/ tests/motion/ tests/sim/test_toss_gate.py -q` →
  **2311 passed in 442.33 s**
- `python -m pytest tests/ros/test_toss_sequencer.py tests/ros/test_reload_coordinator_node.py tests/ros/test_reload_sequencer.py -q` →
  **229 passed in 4.09 s**

**Codegen determinism:** `python config/generate_config.py` run twice back to back,
`git status --porcelain` byte-identical after each. `jugglebot_operational` already
had its `HW_SECTIONS` row, and both new keys emit
(`JB_OP_TOSS_MAX_DISPLACEMENT_MM = 150.0`,
`JB_OP_TOSS_STAY_AT_POSE_ON_CAUGHT = True`) plus the three `.h` mirrors.

**Probe evidence** (both re-run and re-derived at finalize, not taken on trust):
`temp/probes/displaced_reach_frontier_2026-07-29_02-49-14.json` — the real
all-8-directions frontier is 125 mm at `T = 0.55`, 175 mm at 0.60, ~225 mm from
0.70 up, so the closed-form bound is conservative below `T ≈ 0.75 s` and
**optimistic** above it. `temp/reports/toss_8b_phaseE_seed0.json` — PASS,
`feas_viol 0`, `pump_rejects 0`, 55 770/55 770 knots accepted; rings 79/80 (50 mm),
78/80 (70), 78/80 (100), **76/80 (150, worst direction SW 8/10)**.

**NOT DEPLOYED.** `colcon build --packages-select jugglebot` **and** a relaunch —
the launch runs the INSTALLED copy, so until the rebuild the robot executes the
70 mm cap, the config throw site and the `go_home` CAUGHT terminal while the repo,
the tests and the runbook all say otherwise. Runbook row **DISP-0** greps the
installed copy for exactly this. **No firmware flash** (no sketch reads either new
constant) and **no `jugglebot_interfaces` rebuild** (both new topics carry
`geometry_msgs/Point`).

## Related

- Contract: `ros_ws/docs/catch_reach_envelope.md` (**C-REACH-1**)
- Plan: `plans/active/single-ball-toss.md` § Phase E
- Operator ladder: `tests/hardware/session_anomaly_fixes.md` § SECTION DISP
- Sibling commit: the `sim/toss_gate.py` asymmetry-map seed bugfix —
  `logbook/2026-07-29-toss-gate-asymmetry-seed.md`
