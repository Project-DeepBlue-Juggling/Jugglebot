---
title: MVP Phase 4 — Lean shaping + limit-ramp tooling (code-complete; ramp deferred)
type: feature
date: 2026-07-08
status: resolved
phase: "4"
related_plan: mvp-trajectory-bringup.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/shaping.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/planner.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/__init__.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot_interfaces/srv/GoToPose.srv
  - sim/analysis/diagnose.py
  - .claude/commands/diagnose.md
  - tests/motion/test_trajectory_shaping.py
  - tests/sim/test_diagnose_trajectory.py
  - tests/ros/test_trajectory_node.py
  - tests/ros/conftest.py
  - tests/hardware/traj_ramp_battery.py
  - tests/hardware/session_phase4_ramp.md
commits:
  - 2d3afa0
  - 6eb2c74
  - 7588fe6
  - 4b2c02e
subsystem:
  - motion
  - ros
tags:
  - feature
  - trajectory
  - shaping
  - lean
  - diagnostics
  - salvage
---

# MVP Phase 4 — Lean shaping + limit-ramp tooling (code-complete; ramp deferred)

## Summary

Phase 4 of `mvp-trajectory-bringup.md` is the **software** for the multi-session
hardware limit ramp: it adds the observability and the opt-in refinement the ramp
sessions use, but lands **no limit values** — the ramp itself is operator bench
work (edit → regenerate → commit `hardware_config.yaml` between sessions). Four
code-only deliverables:

1. **Lean-into-translation shaping** (`motion/trajectory/shaping.py`, new): a
   translation move superposes a small tilt `∝` its own lateral acceleration to
   keep a ball seated in the cup, with a cup-height-derived lever-arm xy
   compensation so the *cup* stays on the intended path. Ships **default-OFF**
   (`JB_TRAJ_LEAN_GAIN = 0.0`); a single hardware A/B (gain 0.0 vs 0.3) decides
   whether it is kept. Shaping runs **before** `feasibility.validate` — the
   canonical single-gate invariant — so the gate always measures the shaped leg
   peaks and the duration-stretch loop sizes the shaped motion.
2. **A per-call lean override on `GoToPose.srv`** (`float64 lean_gain`) so the
   A/B battery can flip the arm per move without touching config.
3. **Realized-peak observability**: `trajectory_node` tracks the *realized* leg
   vel/acc/jerk peaks from the actually-emitted knots (reset per move) alongside
   the gate's *predicted* peaks, and publishes them + the session limits + the
   active `lean_gain` on `trajectory/diagnostics`. A new `/diagnose` extension
   (`sim/analysis/diagnose.py::summarise_trajectory_moves`) segments a ramp
   session's rosbag into a per-move `peaks + headroom` table.
4. **The operator ramp harness**: `tests/hardware/traj_ramp_battery.py`
   (service-calls-only, operator-run) + the per-step protocol
   `tests/hardware/session_phase4_ramp.md`.

Software is complete and the full suite is green. **The limit ramp itself and the
lean A/B are DEFERRED** to operator bench sessions (per-step protocol above).

> **Salvage note.** This phase was implemented by a previous autonomous agent
> whose session hit the API limit mid-run, leaving all work **uncommitted and
> unverified** (tests never executed, self-audit never done). This session
> salvaged that diff: verified every load-bearing invariant independently against
> the reference geometry, ran the gates the predecessor never reached, added the
> missing logbook/INDEX/plan artefacts, and applied the audit. The Discussion
> records what the predecessor had right, what needed correcting, and every fork
> adopted from its work.

## Motivation

Phase 4's role in the plan is to raise the session leg vel/acc/jerk limits from
the deliberately-tiny Phase-1/2 defaults (100 mm/s, 400 mm/s², 8000 mm/s³) toward
the levels Phase 6 will publish as required for catching — one small step per
short operator session, validated by feel + `/diagnose`. That workflow needs three
things the generator did not yet have: a way to **see** how close each move got to
the limits (realized peaks + headroom), a **scripted battery** so each session is
identical and repeatable, and the **one opt-in refinement** the plan carries (lean
shaping) with a hardware A/B. The always-on smoothness mechanism remains the gate's
duration stretch; lean is a refinement A/B'd on top, not a dependency.

## Design

### Lean shaping (`shaping.py`) — physics, sign, and where it runs

For a translation move the platform accelerates; a ball in the cup feels an
inertial pseudo-force opposing that acceleration. Leaning the cup's up-axis toward
the effective-gravity-plus-inertia direction `(a_x, a_y, g)` nulls the in-cup
lateral force. For base lateral accel `a = (a_x, a_y)` and gain `k`:

- `rx(t) += −k·a_y(t)/g` (tilt about world +x)
- `ry(t) += +k·a_x(t)/g` (tilt about world +y)

A `+ry` swings the cup up-axis toward `+x`; a `−rx` swings it toward `+y`
(right-hand rule on the `[rx, ry, rz]` rotvec), so the cup leans **into** the
acceleration — verified by `test_lean_sign_convention_leans_into_acceleration`,
pinned to the physics rather than to prose (the sign was stated backwards in early
Jugglebot-bb exploration docs).

**Lever-arm compensation.** The cup opening rides a height `arm = cup_z −
CUP_TILT_CENTER_Z_MM` above the tilt centre, so a tilt swings it sideways by
`arm · cup_axis_xy(rx, ry)`. To keep the *cup* on the intended path the platform
**centroid** is offset by the negative of that swing (`centroid_xy = target_xy −
shift`) — the exact mirror of the reference `juggle_tilt.py::realize_tilted`
(`centroid_xy = cup_xy − shift_mm`). `CUP_TILT_CENTER_Z_MM = 744.3` and a nominal
`LEAN_CUP_Z_MM = 839.4` give `arm = +95.1 mm`, reproducing the reference's
`LEVER_ARM_MM_PER_DEG = 1.66` (`arm·sin 1° = 1.6597`). The port's `_cup_axis_xy`
(Rodrigues on +z) reproduces the reference `cup_axis[:2]` exactly; the geometry
test pins `+ry(1°)→+x` and `+rx(1°)→−y` at ≈1.66 mm each.

**Where it runs (the canonical invariant).** `planner._build_rest_move` wraps its
plan with `shaper.shape(plan)` **before** returning, so every candidate the
`build_move` duration-stretch loop hands to `validate` is already shaped, and the
node passes the shaper *into* `build_move` — there is no path that applies lean
after the gate. A gain ≤ 0 (or a `HoldPlan`) is the identity, so the default is a
byte-for-byte-unchanged unshaped move.

**Analytic self-consistency.** `_ShapedPlan.state_at` adds the lean contribution
in closed form: the tilt is `∝ accel`, its rate `∝ jerk`, its curvature `∝ snap`,
all from the quintic Hermite coefficients the segment already stores, and the
small-angle lever compensation collapses to `shift = (arm·k/g)·a`. So the shaped
`(pose, twist, accel)` are mutually consistent (the emitter's `vel_mm_s` matches
the position-knot slope — no feedforward mismatch), verified by finite difference
in `test_shaped_twist_accel_are_analytic_derivatives_of_shaped_pose`.

### Realized-peak observability + the `/diagnose` extension

`_track_realized_peaks` (called after each emitted frame) accumulates running
maxima of the emitted leg vel/acc and a 40 Hz knot-rate finite-difference jerk,
reset on every install. `trajectory/diagnostics` now carries the gate-**predicted**
peaks (fine-sampled, from the accepting report) *and* these **realized** peaks, the
session limits (so `/diagnose` computes headroom without the YAML), and the active
`lean_gain`. `summarise_trajectory_moves` segments the recorded stream on contiguous
`plan_kind == 'move'` windows and reads the accumulated realized peak off each
window's last sample, emitting a `move → {predicted, realized, limits, used_pct,
lean_gain}` table — the `Trajectory Moves` block the `/diagnose` report renders.

### The ramp harness

`traj_ramp_battery.py` is a read-mostly ROS2 client: it fires a fixed battery
(z 170→190→170, x ±20, y ±20, tilt rx ±3°, then one deliberately-infeasible
`duration_s = 0.05` that must come back `TOO_FAST`) through the public
`trajectory/*` services and prints each accept/reject. It touches limits **only**
via `--set-*` (the runtime ramp knob, itself ceiling-clamped by the service) and
never arms the bridge or moves anything directly — arming and the mode sequence
stay the operator's job; every value it sends is a move target or a `--set-*` the
operator typed, never a hardcoded limit. `session_phase4_ramp.md` is the per-step
protocol (raise ONE limit ~1.5× → battery → `/diagnose` review → operator PASS ⇒
persist to YAML; ABORT ⇒ revert the in-session limit to last-good), mirroring
`session_phase1_hold.md`.

## Implementation

`GoToPose.srv` gains one request field `float64 lean_gain` — the only interface
change, so `colcon build --packages-select jugglebot_interfaces jugglebot` is the
gate (the `tests/ros/conftest.py` mock was updated to match: default 0.0). **No
`hardware_config.yaml` change this phase** (`JB_TRAJ_LEAN_GAIN` and
`GRAVITY_MMPS2` both landed earlier), so there is no codegen gate — confirmed
unnecessary, not skipped: Phase 4 deliberately lands no limit values.

The commits split along the same rollback/blame boundaries the prior phases used:
motion layer (`shaping.py` + `planner` + `__init__` + `tests/motion/`), then the
ROS surface (`trajectory_node` + `GoToPose.srv` + `tests/ros/`), then the
diagnostics tooling + protocol (`sim/analysis/diagnose.py` + `.claude/commands/
diagnose.md` + `tests/sim/` + the two `tests/hardware/` files), then docs
(this entry + INDEX + plan Outcome).

## Verification

(date, command, result triples — re-runnable from the artefact alone)

- **New shaping tests** (`pytest tests/motion/test_trajectory_shaping.py -q`, run
  2026-07-08) = **14 passed** — the analytic-derivative helper vs `QuinticSegment.
  eval` (1e-9) and vs finite difference, the sign convention pinned to the physics,
  the lever-arm cross-check vs `juggle_tilt.py` (+ry→+x / +rx→−y at 1.66 mm/deg),
  gain-0/negative/HoldPlan identity, position-continuity at the ends, shaped
  `(pose, twist, accel)` self-consistency, the 5° tilt cap, `build_move`
  validating the shaped plan, and the load-bearing **every shaped knot is
  pump-accepted** invariant.
- **/diagnose summariser tests** (`pytest tests/sim/test_diagnose_trajectory.py
  -q`, run 2026-07-08) = **5 passed** — no-samples, two-moves-separated-by-hold
  with lean-A/B arm + realized *and* predicted headroom %, the **battery case**
  (back-to-back moves segmented by `move_seq`, no intervening hold — the
  correction below), graceful merge when `move_seq` is absent (old bags),
  trailing-move flush.
- **Node lean/observability tests** (`pytest tests/ros/test_trajectory_node.py
  -q`, run 2026-07-08) = **49 passed** (was 42; **+7**: default-off, per-call
  override installs a shaped plan, negative→config default, over-1 clamp,
  diagnostics publish realized peaks + limits + lean_gain + `move_seq`,
  realized-peak reset on install, `move_seq` increments per accepted move).
- **Full suite** (`pytest tests/ -q`, run 2026-07-08) = **2120 passed, 1 xfailed
  in 518.69 s** — 0 failed (the load-flaky `test_hot_loop_allocation_contract`
  passed in-suite this run). Baseline before Phase 4 (`pytest tests/ -q`,
  2026-07-08, post `e039cc0`): **2094 passed incl. the isolated-flaky test, 1
  xfailed**. Net **+26 passed**, fully the new Phase-4 tests (14 shaping + 5
  diagnose + 7 node) and nothing else; the 1 xfailed is unchanged.
- **Interface build** (`colcon build --packages-select jugglebot_interfaces
  jugglebot`, run 2026-07-08, **without** the venv) = **2 packages finished, 0
  errors** — `GoToPose.srv`'s new `lean_gain` field compiles.
- **Codegen determinism** (2026-07-08): no `config/hardware_config.yaml` change
  this phase, so no codegen gate — confirmed genuinely unnecessary (Phase 4 lands
  no limit values), not skipped.

## Discussion

CLAUDE.md makes the Discussion non-negotiable here: this was a salvage of an
unreviewed diff, and several reversible forks (the predecessor's and one of mine)
carry physics that future sessions must be able to reconstruct.

### The salvage — what the predecessor had right, and what I checked

The predecessor's diff was, on independent verification, **correct on every
load-bearing point** — a careful contribution, not a sketch. I re-derived each
against ground truth rather than trusting the prose:

- **Lever-arm sign (the one early docs got backwards).** Confirmed the
  compensation *subtracts* the cup swing (`pose[0:2] -= shift`), matching the
  reference `realize_tilted` (`centroid = cup_xy − shift`), and that `_cup_axis_xy`
  reproduces the reference `cup_axis[:2]` exactly (Rodrigues on +z). The arm is
  positive (+95.1 mm) and reproduces 1.66 mm/deg. Right.
- **Shaping-before-validate.** Traced `planner._build_rest_move` → `shaper.shape`
  → returned into the `validate` loop for both the min-feasible and explicit-
  duration paths; the node passes the shaper into `build_move` and applies lean
  nowhere else. The single-gate invariant holds. Right.
- **Boundary tilt in position.** `test_added_tilt_vanishes_at_segment_ends_and_
  hold` confirms position tilt = 0 at the ends (accel = 0). Right (with the
  tilt-rate caveat below, which the predecessor documented honestly).
- **Phase-2/3 guards.** The lean edits are surgically inserted between the existing
  WRONG_MODE / not-seeded / stale-telemetry / BUSY / install-continuity guards and
  `build_move`; none is regressed (the existing node tests still pass unchanged).

What was **missing** (not wrong) was everything past the code: the tests were never
run, the interface never colcon-built, and the logbook/INDEX/plan-Outcome artefacts
did not exist. This session supplied those.

### Correction — the `/diagnose` per-move summariser collapsed the whole battery into one row

The predecessor's `summarise_trajectory_moves` segmented moves on a `plan_kind !=
'move'` transition, and its test `test_segments_two_moves_separated_by_hold`
interleaved `'hold'` samples to make it pass. But **the real ramp battery never
issues a hold between moves** — `traj_ramp_battery.py` fires all 12 moves
back-to-back with a `time.sleep(settle_s)` — and a completed move's plan keeps
`plan.kind == 'move'` for its entire terminal-hold lifetime (`plan.py:70-72`,
`'hold' if not self.segments else 'move'`). So the battery is one unbroken
`'move'` run; the summariser reported `num_moves == 1`, keeping only the **last**
move's realized peak and discarding moves 1–11 — a silent gutting of the core
Phase-4 deliverable (the per-move review table that gates every limit bump). I
reproduced it (3 back-to-back moves → `num_moves: 1`). The realized peaks *do*
reset per install in the node, but nothing in the diagnostics stream marked that
boundary. **Fix (structural, the honest signal not a heuristic):** the node now
publishes a monotonic `move_seq` KeyValue, bumped once per accepted `go_to_pose`
(NOT per follower replan, so a spacemouse stream stays one window), and the
summariser breaks a window on a `move_seq` change *or* a `plan_kind` transition.
Streams lacking `move_seq` (older bags) degrade gracefully to the prior
plan_kind-only behaviour. This needed no interface change (`move_seq` is a generic
diagnostics KeyValue, no colcon rebuild). New tests pin the battery case and the
graceful-degrade case; the independent audit raised the identical WARNING and
suggested the identical install-id fix, which corroborated the design.

While there I also split the headroom into `used_pct` (realized) **and**
`used_pct_predicted` (gate-authoritative, fine-sampled) and pointed the
`/diagnose` doc at `used_pct_predicted.jerk` for the binding constraint — see the
next fork.

### Fork (adopted from the predecessor) — `lean_gain` semantics: 0.0 = OFF, negative = use config

A default-constructed ROS request has `lean_gain = 0.0`. The predecessor mapped
**0.0 → explicit lean OFF** and **negative → defer to `JB_TRAJ_LEAN_GAIN`**, with
`≥ 0` clamped to `[0, 1]`. The concrete failure this prevents: if 0.0 meant "use
config", then the day the config gain is ramped to 0.3 *every* legacy/bare caller
(and every default-constructed request in a test) would silently start leaning —
an invisible behaviour change with no call-site opt-in. Making 0.0 an explicit OFF
keeps bare callers deterministically lean-off forever and forces the A/B battery to
*name* the arm it wants (`--lean-gain 0.0` / `0.3`). The negative sentinel is the
one path to the config gain. Adopted; the field doc in `GoToPose.srv` states it
plainly. (NaN is treated like the sentinel — graceful, and the effective gain is
always echoed in the response message, so no silent surprise.)

### Fork (adopted) — a fixed nominal cup arm, not the per-pose height-aware arm

The reference computes the lever arm from the *live* cup height (`arm = cup_z −
744.3`, so it scales with the commanded z + hand slider and even flips sign below
the tilt centre). The shaper uses a **fixed nominal** arm (+95.1 mm at the ~840 mm
operating cup height). Rationale: lean tilt is capped at 5°, so the fixed-arm
residual is second order over the MVP's small z range, and the height-aware
per-pose arm is explicitly the Phase-6 `tilt_geometry.py` port's job (the plan says
so). Accepting the fixed arm keeps `shaping.py` a self-contained numpy module with
no dependence on the live commanded z. Adopted; flagged in Open Questions.

### Fork (adopted, with a caveat made explicit) — the tilt-rate boundary transient

The plan says "quintic boundary accelerations are zero, so the added tilt vanishes
smoothly at segment ends — continuity is preserved by construction." That is true
**in position** but not in velocity: a rest-to-rest quintic has **nonzero boundary
jerk** (jerk ∝ 60 at s=1), so the tilt-*rate* (`∝ jerk`) is nonzero at the segment
end and steps to zero across the segment→terminal-hold seam. The predecessor's
docstring states this honestly ("a bounded tilt-rate transient remains at the
ends"). The consequence: with lean ON, the orientation channel is C0 (position-
continuous) but not C1 at that one internal seam — a bounded leg-velocity step the
gate *measures* (dense sampling) and bounds, and which stays pump-accepted (the
pump gate is on position steps, not velocity). This is inherent to "tilt ∝ accel"
with a plain quintic; the only ways to kill it are a septic basis (rejected by the
plan) or a lean *window* that tapers to zero-rate at the ends (deferred). The
right call for a **default-OFF** refinement is to keep the simple form, bound the
transient with the gate, and make it an explicit A/B watch-item — which the session
protocol does ("if the A/B shows the transient as a visible/audible tick at move
start/end, stop and reconsider a windowed lean"). Note this is a within-one-plan
seam, **not** an install/replan discontinuity: `state_at` is continuous in `t`
inside the segment, so C2-across-installs (seeding the next plan from the old one's
sampled state) is unaffected.

### Fork (adopted, low-stakes) — the tilt cap scales derivatives by the instantaneous factor

When `|tilt| > 5°` the code scales the tilt *and* its time-derivatives by the same
instantaneous `scale = cap/mag`. Since `scale` varies with time, the scaled twist/
accel are not the exact derivatives of the scaled pose during a *bound* interval —
a feedforward inconsistency. It does not matter at the operating point: at gain 0.3
a 25 mm xy move tilts ≈0.7°, ~7× under the 5° cap, so the cap never binds; it is a
**safety clamp for pathological gains**, and the inconsistency lives only in a
regime that is never commanded. Adopted as-is; noted so a future session that
raises the cap or gain knows to revisit.

### Fork (mine) — `/diagnose` jerk headroom: realized is a knot-rate proxy; predicted is gate-authoritative

The summariser computes `used_pct` (headroom) from the **realized** peaks. For
vel/acc that is exact (the emitted `vel_mm_s`/`acc_mm_s2` are the analytic
wire values). For **jerk** the realized value is a 40 Hz knot-rate finite
difference of the emitted leg accel — coarser than the gate's fine-sampled
*predicted* jerk (200 samples/segment), so it systematically **under-measures** the
true peak. Since jerk is the binding constraint the ramp raises against, a reviewer
told to "ramp against `used_pct.jerk`" off the coarse metric could read more
headroom than the gate will actually grant (the audit reproduced a realized-vs-
predicted gap). The summariser now emits **both** `used_pct` (from the realized
peaks — authoritative for vel/acc, which are exact wire values) and a parallel
`used_pct_predicted` (from the gate's fine peaks), and `.claude/commands/
diagnose.md` points the reviewer at `used_pct_predicted.jerk` as the
gate-authoritative jerk headroom, keeping realized as a knot-rate cross-check. Two
clean dicts rather than a hybrid `max(realized, predicted)` — the audit floated the
hybrid, but mixing realized-vel with predicted-jerk in one dict is a worse
foot-gun than two explicitly-labelled ones. A test pins that `used_pct_predicted`
tracks the fine peak (6100/8000 → 76.2 %) distinct from realized (75.0 %).

## Open questions / next steps

- **The ramp itself is the deferred hardware work** (`session_phase4_ramp.md`):
  per-step, raise ONE limit ~1.5× via `trajectory/set_limits` → run
  `traj_ramp_battery.py` → `/diagnose --latest` review → operator PASS ⇒ persist
  the bump to `hardware_config.yaml` (edit → regenerate → commit **between**
  sessions, one commit per validated bump with the `/diagnose` numbers). Exit when
  the limits reach the Phase-6-published catch requirements (expected order: vel
  ~150–250 mm/s, acc ~1500–3000 mm/s² — Phase 6 provides the real numbers).
- **The lean A/B is deferred** (one session, gain 0.0 vs 0.3 on the xy moves): keep
  lean only if measured leg jerk drops **and** the motion looks/sounds calmer; else
  leave `lean_gain: 0.0` and log the null result. Watch the boundary tilt-rate
  transient (above).
- **Windowed lean** (deferred): if the boundary transient is audible/visible in the
  A/B, a taper that forces zero tilt-*rate* at the ends removes the C1 seam at the
  cost of a small tracking lag — worth an explicit decision, not an autonomous pick.
- **Per-pose height-aware lever arm** (Phase 6, `tilt_geometry.py`): replaces the
  fixed nominal arm once catch tilts (≤12°) exceed the small-angle regime where the
  fixed arm is second order.

## Related

- Plan: [`plans/active/mvp-trajectory-bringup.md`](../plans/active/mvp-trajectory-bringup.md) — Phase 4 detail + the "Kinetics-aware shaping" architecture section.
- [2026-07-08-mvp-phase3-spacemouse-streaming.md](2026-07-08-mvp-phase3-spacemouse-streaming.md) — the follower supersede that lifts the Phase-2 `BUSY` restriction this phase's `go_to_pose` still carries.
- [2026-07-07-mvp-phase2-waypoint-moves.md](2026-07-07-mvp-phase2-waypoint-moves.md) — the `validate` gate + `build_move` the shaper feeds, and the `GoToPose.srv` this phase extends.
- Reference geometry: `Jugglebot-bb/sim/juggle_tilt.py` (`realize_tilted`, `cup_lateral_shift_mm`) — the lever-arm derivation the shaper ports.
