---
title: "Skill-stack R2 — skills, schedule, stream (sim first): the CATCH-with-throw segment, the release seam, a 22× gate, and the sizing that moved the operating point"
type: feature
date: 2026-09-12
status: resolved
phase: "two-ball-skill-stack — R2"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/{sites,schedule,segments,executor,admissible}.py (new) + INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py (state_at_knot, release_state_at_knot, splice_at, detach_knots)
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py (validate_cycle vectorised), cycle_plan.py (states_at/hands_at)
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py (_svc_install_segment), skill_node.py (new), launch, setup
  - ros_ws/src/jugglebot_interfaces/srv/InstallSegment.srv (new)
  - sim/skills_gate.py, sim/stream_chain.py (new); sim/cycle_gate.py, sim/unified_gate.py (deleted)
  - tools/admissible_sweep.py (new) → config/generated/admissible_box.yaml; tools/probes/{skills_sizing_sweep,skills_segment_sweep}.py (new)
  - tests/hardware/hand_stream_bench.py (--trip-guard), session_skill_stack_r1_flash.md (row 18)
  - tests/motion/test_skills_*.py, test_unified_cycle_splice.py, test_validate_cycle_vectorised.py, test_validate_cycle_budget.py (new); tests/ros/test_install_segment.py, test_skill_node.py (new); tests/sim/test_skills_gate.py (new); tests/sim/test_cycle_gate.py, test_unified_gate.py (deleted)
  - plans/active/two-ball-skill-stack.md
subsystem:
  - motion
  - sim
  - ros
tags:
  - safety
  - planner
---

# R2 — skills, schedule, stream (sim first)

## Outcome

R2's software is landed and its sim gate met: `sim/skills_gate.py` drives the real
skill core (`SkillExecutor` → `install_segment` → the vectorised gate → the real
emitter, pump, wire and firmware mirror) against the MuJoCo plant at the owner's
operating point — **20 of 20 scheduled catches on each of the five seeds, zero
drops, plan wall min 10.4 / p50 28.3 / max 83.3 ms over 405 accepted installs
(catch re-sends included), 63.1 s for the whole sweep** (`python sim/skills_gate.py --no-viewer`,
2026-09-12, apex 0.9 m, separation 100 mm, dwell 0.30 s, 300/5000/200k, hand
3500, exact release + 0.5 mm tracking noise). Every number below carries its
command and date; the one hardware gate ("per-skill plan < 50 ms on the loaded
Jetson, launch up, bag recording") is **outstanding** — no powered sitting this
rung — and has its own no-motion sitting before R3,
`tests/hardware/session_skills_r2_plan_gate.md`. The sim-side and idle-Jetson
twin is recorded below.

What landed, in one line each:

- `motion/skills/`: `Site`, `Skill`/`Pattern`/`Schedule` (`compile_columns`),
  `Segment`/`plan_segment` (THROW = LAUNCH + SETTLE; CATCH = LANDING, or STEADY +
  SETTLE when it carries the next same-site throw; REST = SETTLE — all
  rest-terminal, pinned for all three kinds), `install_segment` + `SkillExecutor`
  (the paper's orchestrator, pure), `AdmissibleBox`.
- `unified_cycle.state_at_knot` / `release_state_at_knot` / `splice_at`: an
  interior splice built on the ring's own `extend`; `replan_tail` and
  `release_state_from_meta` refactored onto them (one path each).
- `feasibility.validate_cycle` vectorised: **4.73 ms** on a 40-knot segment
  (was 103.2; 21.8×), **6.28 ms** on the 57-knot reference (was 160.2; 25.5×),
  verdicts and reason strings identical to the scalar gate and every peak
  within 1e-9 relative over a 148-call battery incl. near-threshold plans.
- `InstallSegment.srv` + `trajectory_node._svc_install_segment` (the skill
  stack's ONE install path; `PlanCycle` untouched for the FSM until R4) and
  `skill_node.py` (the schedule shell: `/balls` → landings, a 40 Hz tick on the
  ROS clock, `skills/start_columns` / `skills/stop`).
- `tools/admissible_sweep.py` → `config/generated/admissible_box.yaml` (per
  site pair: landing ±20 mm, flight 0.833–0.857 s; apex 0.95 m excluded by the
  hand cap on the catch arrival); the node-side refusal on a limits/gate mismatch.
- `sim/skills_gate.py` replaces `sim/cycle_gate.py` + `sim/unified_gate.py`
  (stream-chain helpers moved verbatim to `sim/stream_chain.py`; the three
  firmware hand-lane tests ported).
- R1-carried items closed: warm start (measured away, below) and
  `hand_stream_bench --trip-guard` (row 18 is now live-runnable).

## Discussion

Three of the rung's findings reframed what the plan said. Each is written here
because none of them can be reconstructed from the code alone.

### 1. The sizing grid was empty, and the hand — not the legs — is why

The plan's R2 probe (apex {1.0, 1.3, 1.5} m × separation {80, 100} mm × leg
jerk {30k, 60k, 100k}) produced **no feasible columns cell** with the real QP and
gate (`tools/probes/skills_sizing_sweep.py`, 2026-09-11, two runs, 396 rows,
0 diffs). Extending the axes to 800 / 8000 / 300k mm-space (probe-only, past the
administrative ceilings) and release heights 860–980 mm changed nothing at
≥ 1.0 m: the steady columns window needs **3756–3833 rev/s² of hand
acceleration against the 3500 session cap** at every separation (40–100 mm),
with banking on or off, z-float on or off, chain depth 1 or 3; and 1.3/1.5 m
need the release raised to 900/940 mm to launch at all, after which the
post-release stop overruns the 1004 mm stroke top. The plan's "release-height
lever" trades launch acceleration against stopping room inside one 324 mm
stroke; z-float does not move the peak hand acceleration at all (identical
peaks with it on and off). The old cycle gate's headline — 0.80 s is the
maximum plannable flight at 3500 rev/s² with z pinned — was the same fact.

Feasible cells begin at 0.8–0.9 m. The owner adopted **0.9 m / 100 mm / dwell
0.30 s / hand 3500 / legs 300-5000-200000** over the 1.0 m row (which needs
the hand at its 3900 C-HAND-2 ceiling with 2 % margin) and over 60 mm. Two
facts for the sittings: every feasible cell sits at the leg-jerk ceiling
(200k), so the session ramp of decision 4 is to the ceiling, not past the
30k default; and the R1 sitting's 150k **does** pass the adopted point on the
final segment form (146k peak) — see § 3.

### 2. The pinned segment shape was infeasible; the whole-window form is what flies

The plan pinned a rest-terminal CATCH (LANDING) with the next THROW spliced into
its tail. Built that way, `sim/skills_gate.py` made 0–2 of 20 catches: the
THROW spliced one knot after touch-down refuses `LIMIT_JERK` (260k at the
adopted point). `tools/probes/skills_segment_sweep.py` — the executor driven
with a perfect analytic tracker, no plant — reproduced it and then swept 480
cells (apex 0.9–1.0, separation 40–100, dwell 0.30–0.50, jerk to 300k, legs to
800/8000, hand to 3900): **0 feasible**, best case 212k against 200k. The
mechanism is the seam: the CATCH's runway decelerates the hand toward rest, and
a THROW re-solved from one knot after touch-down with `dwell − lead` left must
undo that profile and reverse into the launch. The continuous STEADY form the
sizing sweep had measured (catch and throw in one QP window) knows at the catch
that a throw follows and shapes the deceleration for it — and it passes at the
same point with 178k of 200k. That is plan § 1.1's last row made concrete: this
platform moves continuously through catch and throw. The owner amended § 2.2: a
CATCH skill may carry the next same-site throw (`Skill.then_throw`), planned as
STEADY + a settle tail, still rest-terminal; the standalone CATCH stays for the
last catch and the R4 reload.

### 3. The release seam: splice AT the release, seeded post-release

Verifying § 2 exposed a correctness gap in the splice core as first built: a
segment dispatched at `t_release − lead` splices exactly at the release knot,
and `state_at_knot` seeded it with `post_release=False` — which re-solves the
window without the detach cone the release earned, the 1.126 m/s² off-axis
shove `replan_tail` refuses. The rule adopted is the ring's own handoff
(`extend`'s contract): a splice landing inside a release's detach cone snaps to
the release knot and is seeded by `release_state_at_knot` (site, take-off
velocity, g, detach axis, the head's levelling frame). A second arithmetic
fact followed (Unit G's finding): the schedule subtracted the same lead the
splice added, leaving one knot of wire margin against a 26–34 ms solve, so the
robot path would have refused `SPLICE_TOO_LATE` on the first handoff while the
tests passed with an instantaneous solve. Resolution: a segment that follows a
release always splices at that release whatever its dispatch time; the
handoff dispatch lead is 8 knots (125 ms of solve budget after the wire's 3),
the general lead 6 knots (was 4: plan § 2.4 assumed ≤ 20 ms of plan; measured
26–34 ms), the catch freeze lead + one knot. Proved with a modelled solve
(`skills_segment_sweep.py --solve-ms`, 2026-09-12, eight cells, six throws):
0 and 50 ms install end to end with peaks bit-identical to the unmodelled run
(the leads move WHEN a segment is dispatched, never what it plans); 100 ms
installs every catch and the launch and refuses only the final REST (an
unpinned splice has the general budget of 3 knots = 75 ms, and the machine is
already at rest there); 130 ms refuses the first handoff naming its 125 ms
budget. The general lead stays at 6 knots rather than 8: the plan's own gate
is < 50 ms on the loaded Jetson, so 75 ms is 25 ms of margin over the
criterion the sitting must meet regardless, and 8 knots would cost every catch
re-send 50 ms of tracker freshness. Two re-send facts followed from the
sim gate: a re-send whose window from `now + lead` to touch-down would be
under the 4-knot floor is not solved at all (75 refused solves per 8 throws
before); and the handoff snap considers only releases the segment's own
event FOLLOWS — a CATCH re-send's carried throw sits in the head after the
catch, and snapping to it pulled the splice past the catch (every re-send
refused with a negative window). With both, ~4 re-sends per catch install
under the 0.5 mm tracking noise and the late ones refuse `LIMIT_ACC`/`JERK`
harmlessly (the committed catch stands).

### Smaller findings

- **I-PLAN-3 was never true of the shipped gate.** The row said "a plan the
  coarse gate accepts must pass a dense re-check"; the leg jerk is a finite
  difference whose denominator shrinks with the mesh, so refinement RAISES it
  (the 1.4 s steady cycle reads < 150k at 4 samples/knot and 229 537 at 16),
  in the scalar gate exactly as in the vectorised one. The row now pins the
  true, stronger form: every sample-maximum peak at the dense mesh is ≥ its
  coarse twin, the closed-form and knot-step peaks are equal, and a coarse
  accept that the dense mesh refuses on any code other than `LIMIT_JERK` fails.
- **Warm start (R1 item a) measured away.** Fresh process, chained
  LAUNCH+STEADY: cold 385.6 ms, warm 377.9, QP warm-started 373.6 — the cold
  penalty is ~8 ms and the QP warm start ~3 ms; 93 % of the call was three
  `validate_cycle` passes. The sitting's 2.2 s was that under load. The QP
  `SolverState` rides on the `Segment` (no cache); the vectorised gate is the fix.
- **Parity tolerance.** Bit-exactness would have forced the scalar summation
  order onto the batched path; the shaped-gate precedent's 1e-9 relative bound
  (measured reorder error ~1e-13) with verdict identity and a near-threshold
  battery is what `test_validate_cycle_vectorised.py` pins. Residual, stated in
  that file: a plan within ~1 ulp of a hard workspace/condition bound could
  classify differently — inherent to any re-expression, bounded by those
  bounds' physical margins.
- Interior splices at the OLD session point (250/3000/150k) refuse
  `LIMIT_JERK` (154 510 at k_s = 28 on the reference chain); the R3 sitting
  runs at the new point.
- `reload_coordinator_node`'s 0.60 s extend lead was sized on a 354–387 ms
  MODE_EXTEND; that solve is now ~5× cheaper. Left alone (a live-path lead on
  the FSM, which R4 deletes).
- The plan's § 3 R2 row said "deletes PlanCycle modes, ring machinery" while
  § 4 R4 and § 6 scheduled the same deletion for R4 — corrected: R2 deletes the
  two sim gates and the scalar gate loop; `PlanCycle` and the ring policy go at
  R4, and the ring primitives (`extend`, `_concat_plans`, `_gate_joined`,
  `_seam_check`) are reused by `splice_at`, not dead.

## Verification

- Sizing (STEADY form): `python tools/probes/skills_sizing_sweep.py --vel 250 400`
  run twice 2026-09-11 → 396 rows, 0 diffs; the frontier and clamped runs
  2026-09-12 (`temp/probes/skills_sizing_{frontier1,frontier2,clamped}.md`).
- Sizing (segment form, the table of record): `python tools/probes/
  skills_segment_sweep.py --apex 0.9 --sep 40 60 80 100 --dwell 0.30 --jerk
  200000 150000 --vel-acc 300/5000 --hand-acc 3500 --n-throws 6` (2026-09-12):
  all 8 cells install end to end; 100 mm peaks 266 / 4034 / 146k / 3399, plan
  ≤ 27.7 ms per install (idle Jetson).
- Vectorised gate: `pytest tests/motion/test_validate_cycle_vectorised.py
  tests/motion/test_validate_cycle_budget.py tests/motion/test_unified_cycle_budget.py -q`
  (2026-09-12): green; 40-knot min 4.73 ms / p50 4.76 (< 10 ms), reference
  cycle 6.28 / 6.31; `plan_steady` total min 24.5 ms of which gate 6.4 (26 %).
- Sim gate: `python sim/skills_gate.py --no-viewer` (2026-09-12, the run of
  record, re-sends live): PASS on seeds 0–4, 20/20 catches each, 0 drops,
  plan wall min 10.43 / p50 28.28 / max 83.27 ms (n = 405 accepted installs),
  63.1 s; ~80 accepted installs per seed of ~138 attempted (the refused ones are
  late re-aims the committed catch survives). Before re-sends were live the
  same criterion read max 59.48 ms, n = 128, 50.8 s. A first run of the same
  criterion the same day showed two installs of 2289 and 1712 ms on seed 4; a
  deterministic replay of seed 4 alone measured max 58.9 ms (QP 33.9 of it),
  so those were the box, not the solver — the loaded-Jetson gate exists for
  exactly that. The scatter table (separation × throw-velocity noise, 5 seeds):
  100 mm passes only at 0 %; 80 mm 3/5 at 0.5 %; 60 mm 5/5 at 0.5 %, fails at
  1 %; 2 % fails everywhere; every failure a CATCH refused before motion, 0
  drops. `pytest tests/sim/test_skills_gate.py -q`: 8 passed.
- Admissible sweep: `python tools/admissible_sweep.py` (2026-09-12, idle box):
  2.1 s wall (a first run under three concurrent agents took 302 s — load, not
  the sweep); two boxes (P1→P2, P2→P1), landing ±20 mm × ±20 mm, flight
  0.8327–0.8569 s, stamped 300/5000/200k, hand 3500, gate hash `8217b4a13ca7`.
  `pytest tests/motion/test_skills_admissible.py -q`: 24 passed.
- Bench driver: `pytest tests/teensy_link/test_hand_stream_bench_gap.py -q`
  (2026-09-12): 56 passed.
- Full gate: `./run_tests.sh --full` (2026-09-12, after the audit fixes, with the sim gate's smoke and the two new serial timing tests included): **6500 passed, 9 skipped, 1 xfailed** in 318.47 s (parallel) + **6 passed** in 19.06 s (serial); `RESULT: PASS`. Baseline at `67445f3` was 6365 / 9 / 1 + 4 serial, so the rung adds 135 parallel tests and 2 serial ones with no regression.

## Handoff (R3)

- **Outstanding hardware gate**: per-skill `cycle_plan_wall_ms` < 50 ms with the
  launch up and a bag recording — its own sitting before R3, `tests/hardware/session_skills_r2_plan_gate.md` (written 2026-09-13),
  dress-rehearsed on the loaded Jetson. Both launches need
  `colcon build --packages-select jugglebot_interfaces jugglebot` (new srv).
- The R3 sitting runs at 300 / 5000 / 200000 (jerk to the ceiling is a logged
  ramp per decision 4); 150k also passes the adopted point on the final form.
- INVARIANTS rows re-carried to R3 (the skill-side precondition ladder: no ball,
  ball unknown, hand stale, hand not parked, mocap stale, not levelled, mode
  changed) — the mechanisms exist, the operator-facing codes land with the
  first powered rung.
- The learner's throw command lives on the CATCH skill's `then_throw` (the
  first throw from rest is a THROW skill); the outcome capture is R3's.
