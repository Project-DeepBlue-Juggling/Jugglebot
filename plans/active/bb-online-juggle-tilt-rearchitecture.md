---
title: Online-juggle tilt re-architecture — aim throws by platform tilt (Kai-style), close the 2-ball loop
created: 2026-06-29
status: active
related_plan: bb-led-two-ball-juggle-demo.md
related_logbook:
  - 2026-06-27-online-replanning-architecture-and-cup-bandlimit
  - 2026-06-27-online-juggle-throw-fix-catch-axis-split-band-limit-cascade
  - 2026-06-27-throw-aim-band-limit-and-closed-loop-catch
---

# Online-juggle tilt re-architecture

## 1. Context

### Where we are

`sim/juggle_online.py` (+ `controller/demo/juggle_planner.py`) is an online,
per-throw, task-space planner (a faithful adaptation of Kai Ploeger's
`plan_throw`, IEEE 9981678 — repo cloned at
`/home/jetson/Desktop/kinematic_planning_for_nball_toss_juggling/`). It catches
**5 / 0** (seed 0) under full contact physics, beating the offline demo's 2, then
**collapses to a 1-ball shuffle**. Root cause (measured, logbook
`2026-06-27-online-juggle-throw-fix-catch-axis-split-band-limit-cascade`): the
catch seats the ball **~15 mm off-centre** (the platform's band-limited lateral
tracking error), and the **contact-carry throw amplifies that beyond the
±150 mm reach** → divergence (loop gain > 1).

### Why tilt is the fix

The current **level-platform decoupling** forces the lateral throw component to
come from the **band-limited platform velocity** — exactly the actuator that
can't track fast transients (−3 dB at ~5 Hz; the throw is a fast transient). Kai
instead **tilts each hand** and detaches the ball **along the tilted axis**, so
the throw's *speed* comes from the fast actuator (his hand / our slider) and the
*aim* comes from a roughly-constant tilt. On our morphology:

> lateral take-off velocity = `slider_speed × sin(tilt)` — e.g. 5 m/s × sin(11°)
> ≈ **0.95 m/s**, delivered by the perfect slider along a **held tilt**, with the
> platform never commanding fast lateral translation.

Crucially, the throw velocity becomes set by a *constant tilt + the perfect
slider* — **independent of small in-cup offsets** — so the throw stops amplifying
the catch error and the loop should converge. Caveat (the operator's instinct):
*changing* tilt quickly swings the cup through its lever arm (ω×r) and re-enters
the band limit, so tilt is held ~constant through the throw, not whipped.

We adopted Kai's **planner**; this plan re-adds the two Kai ingredients we
dropped — **throw tilt** and the **carry-inward catch** — while *keeping full
contact-physics fidelity* (an explicit operator decision: no kinematic-throw
shortcut). Jugglebot has **one cup**, so 2-ball is a "2-in-one-hand" oval —
harder than Kai's two-hand cascade; we bring it up as **columns first**, seeded
by **Ball Butler**, then ramp toward the oval.

### Goal

Sustained 2-ball juggling — **≥30 catches / 0 drops**, seeded by Ball Butler,
under faithful contact + real actuator band limits (so it transfers to
hardware), the cup tracing a continuous carry oval. The headline
`test_full_sim_juggle_reaches_target_catches` (currently `xfail(strict=True)`)
passes un-xfailed against the online runner.

### Explicit decisions already taken (do not relitigate without the operator)

- **Keep contact-physics fidelity** (B): no kinematic / controlled-velocity
  throw shortcut. Make tilt-aimed throws work under *real* contact.
- **Re-introduce platform tilt** (A) as the primary lateral-aiming mechanism;
  modest, ~constant through the throw.
- **Carry-inward catch** (C): tilt makes Kai's catch-outer→carry-inward→
  throw-inner geometry + firmer (toward collinear) catch feasible.
- **Columns → oval** (D): bring up columns first (lowest lateral demand), but
  **BB-seeded with the platform tilting to receive the BB throw cleanly**; the
  oval remains the end goal, reached by ramping the tilt + separation knobs.

## 2. The phased plan

Each phase is run by a **fresh Claude instance** (a workflow subagent) following
the cycle **implement → audit → commit (fetch-guarded push) → write the next
phase's prompt**. The workflow runs in **gated auto-chain** segments with human
go/no-go at two gates. See §4 for the workflow + gates.

---

### Phase 0 — Characterise tilt / orientation tracking  *(GATE after)*

**Goal.** Quantify how well the Stewart platform tracks orientation (rx/ry tilt)
so we know tilt is viable for throw-aiming and the planner has the numbers it
needs. We characterised lateral translation + the slider; orientation is the
missing piece.

**Approach.** New committed probe `tools/probes/juggle_tilt_bandlimit.py`
(mirror the structure of `juggle_cup_bandlimit.py`): at the operating point
(centroid z = 170 mm), command rx/ry tilt and measure —
1. **Static hold** — commanded vs achieved tilt at a sweep of modest angles
   (e.g. 0/3/6/9/12°); is the hold accurate, any droop?
2. **Dynamic** — a low-frequency tilt Bode (amplitude ratio + phase lag at the
   1.6 Hz cycle freq and a couple of points around it); how fast can tilt change
   before the band limit bites?
3. **Lever arm** — mm of cup (`hand_opening`) **lateral** shift per degree of
   tilt, and any **vertical** cross-coupling, so Phase 1 can compensate.
4. **Leg headroom** — does a modest tilt at z = 170 keep all legs off their
   stroke limits (no saturation, the artefact that bit the z = 0 lateral sweep)?

**Files.** `tools/probes/juggle_tilt_bandlimit.py` (new, committed — reusable-
probe rule), `tools/probes/README.md` (entry), a findings note (a logbook entry
`2026-06-29-platform-tilt-tracking-characterisation` *or* a section appended to
the online-replanning entry — implementer's call, but it must be a durable,
cited artefact with the numbers).

**Acceptance.** Committed probe + a findings summary stating: static-hold
accuracy, dynamic tilt tracking at the cycle freq, the lever-arm (mm/deg lateral
+ vertical cross-coupling), the recommended **max usable tilt magnitude and
rate**, and leg-headroom confirmation. The probe runs headless on the Jetson
venv and is side-effect-free.

**GATE (human).** Is tilt-tracking good enough — modest tilt held accurately,
lever-arm understood — to build Phase 1 on? *(Also the first real test that the
**workflow mechanics** work end-to-end on a low-risk phase.)*

---

### Phase 1 — Re-introduce tilt in planner + realisation

**Goal.** Plan and realise throws aimed via a modest platform tilt; the lateral
take-off velocity comes from `tilt × slider-speed`, not platform translation.

**Approach.**
- **Planner** (`controller/demo/juggle_planner.py`): add a **tilted-axis detach**
  constraint — Kai's `cross(cacc − g, hand_axis) == 0` for the first `n_detach`
  knots, where `hand_axis` is the *tilted* cup z-axis (not world +z). Aim the
  take-off velocity along that axis. Add the tilt (the cup-axis orientation) as a
  planner input, bounded to the Phase 0-characterised modest range. This
  generalises the current level (world-+z) detach; keep level as `tilt = 0`.
- **Realisation** (`sim/juggle_online.py::realize`): re-introduce orientation
  (rx/ry) into the pose (undo the *orientation* half of the level decoupling;
  keep the slider doing speed-along-axis). **Compensate the lever arm** from
  Phase 0 — the cup shifts laterally with tilt, so offset `centroid_xy` so the
  cup lands where the plan wants it.

**Files.** `controller/demo/juggle_planner.py`, `sim/juggle_online.py`,
`tests/sim/test_demo_juggle_planner.py` (new tilt-constraint tests — verify the
take-off velocity's lateral component equals `slider_speed × sin(tilt)` and the
detach is collinear with the tilted axis).

**Acceptance.** Planner produces tilted throws with the expected lateral take-off
component (verified numerically); realisation commands the tilt and compensates
the lever arm so the *cup* tracks the planned tilted trajectory in sim (measured,
e.g. a probe); planner unit tests pass + the new tilt tests. Default behaviour
with `tilt = 0` is unchanged (regression-safe). No gate — auto-continue.

---

### Phase 2 — Validate the tilt-aimed throw under real contact  *(GATE after — make-or-break)*

**Goal.** Confirm the tilt-aimed throw imparts the lateral velocity **cleanly and
consistently regardless of small in-cup offset**, and that this kills (or
substantially reduces) the divergence — under full contact (decision B).

**Approach.** Run the online runner with tilt-aimed throws + contact; re-measure
(reuse / extend the Phase-pre probes `/tmp/probe_throwchar`-style, promoted to
`tools/probes/` if reused): (a) per-cycle **launch-velocity consistency** and
**landing-offset-vs-in-cup-offset** (does the throw still amplify?); (b) the
**divergence** (catch count, does the pattern sustain longer, is loop gain < 1?).
Compare head-to-head with the pre-tilt baseline (5 catches, diverges). Tune the
tilt magnitude within the Phase 0 limits.

**Files.** measurement probe(s) under `tools/probes/` if reused; tilt-magnitude
tuning in `sim/juggle_online.py`; a logbook entry documenting the make-or-break
result (the comparison table + the verdict).

**Acceptance.** A measured comparison showing the throw amplification is reduced
(launch velocity now ~independent of in-cup offset) and the divergence is
reduced/eliminated (catch count up materially vs 5). The logbook entry states the
verdict plainly.

**GATE (human).** Did tilt actually kill the divergence? Go/no-go for Phase 3-4.
If **no**, stop — the hypothesis failed and we re-plan (this is the riskiest
assumption in the whole arc).

---

### Phase 3 — Carry-inward catch geometry (Kai-style)

**Goal.** With tilt available, adopt Kai's catch-outer → carry-inward →
throw-inner geometry + a firmer (toward collinear) catch, replacing today's
lateral-reversal crossing.

**Approach.** Re-geometry the pattern: catch at the **outer** lateral point
(cup moving inward), carry the ball **inward** (a `CARRY_DISTANCE` analog), throw
from the **inner** point — the cup moves consistently (no double reversal).
Firm the catch velocity-match toward Kai's **hard collinear** (now feasible
because tilt carries the lateral burden). Update the planner's catch constraints
+ the runner's pattern geometry.

**Files.** `controller/demo/juggle_planner.py` (catch constraints),
`sim/juggle_online.py` (pattern geometry), `tests/sim/test_demo_juggle_planner.py`.

**Acceptance.** The cup traces a carry-inward oval (no lateral reversal at the
catch); the catch seats more reliably (measured); catch count improves further.
No gate — auto-continue.

---

### Phase 4 — Closed-loop bring-up: columns → oval, BB-seeded

**Goal.** Sustained 2-ball juggling, brought up **first as columns** (lowest
lateral demand), **seeded by Ball Butler** with the platform **tilting to receive
the BB throw cleanly**, then ramped toward the ovular end goal.

**Approach.**
- **BB seeding.** Integrate `BallButlerSim` priming (as the old `juggle_demo`
  does): BB throws the first ball; the platform **tilts to receive** the incoming
  BB throw cleanly (tilt is exercised on the catch/priming side, not just the
  throw). Reuse the old demo's BB-lead-time calibration pattern.
- **Columns first.** Start the closed loop as columns — near-zero crossing,
  ball(s) ~straight up/down in fixed column(s), cup shuttling — the regime with
  the least lateral demand. Get to **≥30 catches / 0 drops** consistent.
- **Ramp to oval.** Parameterise tilt + separation so columns and oval are two
  ends of one knob; dial up toward the ovular end goal once columns sustain.
- **Tests.** In `tests/sim/test_demo_juggle_sim.py`: retarget
  `test_full_sim_juggle_reaches_target_catches` (T-I3, currently
  `xfail(strict=True)` at the ≥30-catch bar) to the online runner and **un-xfail
  it when it passes**; and re-point the already-passing smoke test
  `test_short_run_catches_the_bb_primed_ball_and_a_throw` (catches both balls
  once) at the online runner.

**Files.** `sim/juggle_online.py` (BB seeding + columns/oval parameterisation),
`tests/sim/test_demo_juggle_sim.py` (retarget headline cases),
`sim/juggle_demo.py` (BB priming reference — read, don't necessarily change),
logbook.

**Acceptance.** ≥30 catches / 0 drops sustained (columns first), BB-seeded with
tilt-to-receive; headline tests pass un-xfailed against the online runner; a
demonstrated path (parameter ramp) toward the oval. Final phase.

## 3. Cross-cutting constraints (every phase)

- **Venv.** `source ~/Desktop/PDJ_venv/venv/bin/activate` for all python/pytest.
- **Control-system rigor.** Analyse control/physics implications before changing
  the planner/realisation (CLAUDE.md). Tilt touches the throw kinematics —
  reason through one cycle before editing.
- **Tests.** `pytest tests/ -q` after `*.py`/`*.yaml` changes AND as the final
  pre-commit gate; cite the (date, command, result) triple. The flaky
  `test_hot_loop_allocation_contract` (see memory) passes in isolation — re-run
  isolated to confirm, never block on it; any *other* failure blocks the commit.
- **Reusable probes** → `tools/probes/` (committed) + README entry; one-offs →
  `/tmp`.
- **Logbook.** Each substantive phase gets a logbook entry with a real
  Discussion section; backfill the commit SHA after committing.
- **No backticks in `git commit -m`** (use heredoc/-F); `/audit --unstaged`
  before any commit touching a logbook/plan/normative .md.

## 4. The workflow + gates

Driven by the saved workflow `.claude/workflows/bb-tilt-phases.js`. It runs a
**segment** of phases (`args: {from, to}`), each phase as fresh agents:
**implement → audit (independent audit-reporter) → finalize** (apply fixes, full
`pytest tests/ -q` gate, commit with the Co-Authored-By trailer, **fetch-guarded
push**, write the next phase's handoff prompt). Push is fetch-guarded: if
`origin/demo/bb-led-two-ball-juggle` has diverged, the phase **does not push or
rebase** — it aborts with a PARALLEL-SESSION warning for the human.

**Gated auto-chain segments (operator's choice):**

| Invoke | Runs | Then |
|---|---|---|
| `args: {from: 0, to: 0}` | Phase 0 | **GATE** — tilt-tracking good enough? (+ did the workflow mechanics work?) |
| `args: {from: 1, to: 2}` | Phase 1 → Phase 2 | **GATE** — did tilt kill the divergence? (make-or-break) |
| `args: {from: 3, to: 4}` | Phase 3 → Phase 4 | done |

After each segment the workflow surfaces the commits, the gate assessment, and
the next phase's handoff prompt. The human reviews, then triggers the next
segment.

## 5. References

- Kai Ploeger, *Controlling the Cascade* (IEEE 9981678); repo at
  `/home/jetson/Desktop/kinematic_planning_for_nball_toss_juggling/` —
  `task_space_juggling.py` (`plan_throw`, `HAND_TILT`, tilted detach, carry-
  inward geometry, the online re-plan loop).
- Band-limit characterisation: `tools/probes/juggle_cup_bandlimit.py`, logbook
  `2026-06-27-online-replanning-architecture-and-cup-bandlimit`.
- The wall this re-architecture targets: logbook
  `2026-06-27-online-juggle-throw-fix-catch-axis-split-band-limit-cascade`.
- Offline demo's tilt/banking + BB priming reference: `sim/juggle_demo.py`,
  `controller/demo/juggle_optimizer.py`, plan `bb-led-two-ball-juggle-demo.md`.
