---
title: MVP Trajectory Bringup — autonomous build run (Phases 1–7 software-complete)
type: feature
date: 2026-07-08
status: resolved
related_plan: mvp-trajectory-bringup.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/ (new package)
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/reload_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator.py
  - ros_ws/src/jugglebot/jugglebot/ball_butler_node.py
  - ros_ws/src/jugglebot_interfaces/ (GoToPose/SetTrajectoryLimits/TimedTarget srv, TrajectoryStatus/TargetFeedback msg, Reload.action, BallButlerThrow ext)
  - sim/reload_gate.py
  - sim/juggle_noise.py
  - sim/hand/trajectory.py
  - config/hardware_config.yaml
  - tests/hardware/mvp_bench_runbook.md (new)
commits:
  - 63031c3   # P1 code (config + package + node + arming) — first build-run commit (branch base is 3ea6d18, the plan-doc commit)
  - c0b31a9   # P1 audit-fix (2 WARNING + 7 NOTE)
  - 614820c   # P2 code (full gate + build_move + ROS surface)
  - f38153f   # P2 audit-fix (1 BLOCKING + 2 WARNING + 6 NOTE)
  - 5cac69b   # P3 code (fast follower gate + graceful stop + ROS surface)
  - 3605e86   # P3 audit-fix (4 WARNING + 6 NOTE)
  - 2d3afa0   # P4 code (lean shaping + observability + ramp harness)
  - 1c0f9c1   # P4 audit-fix (1 BLOCKING + 4 WARNING + 2 NOTE)
  - 62e9ea7   # P5 code (timed targets + catch feedback swap)
  - 22ed9cf   # P5 audit-fix (3 WARNING + 6 NOTE) + ci-deep backfill
  - 12c7ad1   # P6 code (catch trajectory + production-in-the-loop sim gate)
  - bf5b46e   # P6 audit-fix (3 WARNING + 3 NOTE) + sequential-catch mode
  - e2c5afe   # P7 code (reload sequencer FSM + coordinator node + integration test)
  - 836856d   # P7 audit-fix (5 BLOCKING choreography bugs + WARNING/NOTE)
subsystem:
  - motion
  - ros
  - sim
  - config
tags:
  - mvp-trajectory-bringup
  - trajectory
  - reload
  - safety
  - testing
  - meta
---

# MVP Trajectory Bringup — autonomous build run (Phases 1–7 software-complete)

## Summary

This is the closing meta-entry for the `mvp-trajectory-bringup` build run — the arc
that removed the compute-marginal MPC 40 Hz CasADi hot loop from the leg path (kept
dormant, source retained) and replaced it with a deliberately simple, gate-guarded
Jetson-side trajectory generator streaming waypoints to the already-validated
`SetpointPump` → can-hub-Teensy Hermite chain on ZMQ :5557.

Seven phases were implemented **autonomously**: a fresh Opus 4.8 phase agent per
phase (`/mvp-phase-runner`) under Fable 5 orchestration, each phase landing
software-complete with the full `pytest tests/ -q` suite green, then followed by an
**independent `/audit` round** that reported findings and a fix-and-land package. The
per-phase entries carry the detail; this entry compiles the cross-phase decision
digest, the audit-arc retrospective, and the consolidated bench-must-answer list. The
build run reached the exact boundary the plan drew: **all software for goals 1–4 is
merged and green; every hardware session is deferred** to the operator via
`tests/hardware/mvp_bench_runbook.md`.

Headline facts:

- **7 phases implemented + 7 independent audit rounds** (branch range
  `63031c3^..94d6336`, i.e. commits `63031c3` through `94d6336` inclusive).
- **1 phase-agent death salvaged + 2 auxiliary-agent interruptions resumed.** The
  Phase-5 audit-fixer and the Phase-7 auditor both hit API session limits mid-run and
  were resumed from their transcripts with context intact (each re-verified its own
  working-tree state before continuing). Phase 4's death is documented in-entry: a
  predecessor agent hit the API limit mid-run, leaving the diff uncommitted and
  unverified (tests never run, self-audit never done); the salvaging session
  independently re-derived every load-bearing invariant against the reference
  geometry, ran the gates, added the missing logbook/INDEX/plan artefacts, and applied
  the audit before landing.
- **Suite grew 1956 → 2274 (+318 tests)**, every increment fully accounted for by new
  tests only (no pre-existing test changed count; the single xfail is unchanged
  throughout).
- **ci-deep green twice**: the Phase-2-deferred backfill ran at Phase 5 (2164/1) and
  again at Phase 7 (2274/1).
- **All hardware sessions deferred** to `tests/hardware/mvp_bench_runbook.md` (the
  single consolidated operator runbook) — Phases 1–5/7 hardware and the Phase-4 limit
  ramp are operator bench work; Phase 6 was sim-only and its gate CORE-PASSES.

## Decision digest

The load-bearing section: every **decide+document fork** taken across the seven
Discussions and audit rounds, grouped by phase, one line each (`fork — choice —
why`). This is the map a future session uses to reconstruct why the code looks the way
it does without re-reading seven entries.

### Orchestrator-level (cross-phase)

- **Knot rate — fixed 40 Hz** (firmware `SEGMENT_T_S = 0.025 s` compile-time) — the
  validated MPC seam is byte-identical at 40 Hz and the 250 ms staleness E-STOP gives
  a 10× margin; configurable knot rate is explicitly Deferred.
- **`BUSY`-then-lift-in-5 sequencing** — the Phase-2 audit added a *temporary* `BUSY`
  restriction (moves accepted only from a hold) because interrupting an in-flight move
  needs a C2 supersede; it was **lifted for the timed path in Phase 5** once the fast
  gate made a mid-move C2 replan safe. `go_to_pose` keeps `BUSY` deliberately (it uses
  the analytic gate for shaped plans) — the asymmetry is tested.
- **Pending-stop retry over retargeted stop** (Phase 3) — the graceful stop is
  decel-in-place; a near-boundary outward seed can't stop-in-place, so the node
  retries from the decaying live state. The cleaner *retargeted* stop
  (`p1 = p0 + α·v0`, never overshoots outward) is deferred because it changes the
  stop's terminal pose (a behavioural change worth its own review).
- **Document-not-fix the ≤1.5 % jerk finite-difference bias** — under-measurement is
  bounded and made strictly conservative (300 samples + ×1.05 inflation on the
  follower gate) rather than replaced with a costlier third-difference; over-rejection
  is benign (keep last valid plan).
- **12° tilt clamp** (Phase 7 audit) — clamp the receive tilt at
  `tilt_geometry.MAX_TILT_DEG` rather than reject above it: 12° is both the
  sim-validated receive envelope AND the lead-time-feasible tilt, and a clamped tilt
  seats a ball better than a level cup, so clamp-don't-reject strictly improves seating
  and never blacklists.
- **Light-scope Phase 6 vel-match deferral** — the ≤15 %-first-contact criterion floors
  at ~0.26 and is inconsistent with the hardware-validated 0.6 hand design (a *designed*
  ~40 % first-contact mismatch); landed everything else, documented the criterion for
  hardware revisit.
- **`juggle_bb_catch.py` port deferred to Phase 8/9** — the interactive BB→Jugglebot
  demo is superseded by the headless `sim/reload_gate.py` for Phase 6; the bb file
  ports with the multi-ball work.
- **Audit-fix commits skip a second `/audit`** — each phase's fix-and-land package is
  self-audited + orchestrator spot-checked rather than re-audited, to avoid an infinite
  audit regress; the fix package's own tests are the gate.

### Phase 1 — streaming foundation

- `trajectory/status` as `DiagnosticStatus`, not the typed `TrajectoryStatus.msg` yet
  — the typed msg + migration land in Phase 2 (which needs an interface rebuild
  anyway), so Phase 1 needs **no** interface rebuild before a safety-sensitive first
  arming session.
- No auto-return-to-neutral on STANDBY entry (deviation from the plan) — seed a hold at
  the *measured* pose + require an explicit `go_home`; avoid un-commanded platform
  motion during the first-ever arming bring-up.
- Derive the arming `u0` through a throwaway `SetpointPump.build` — the pump owns the
  authoritative u0 convention; re-deriving inline is a second copy that can silently
  drift.
- Copy the quintic math, don't import `controller/` — keep `motion/` free of a live
  dependency on the dormant `controller/`; drift closed by a bit-for-bit xref test.
- Step-bound emitter freeze is a backstop, not the primary mechanism — the gate's
  duration-stretch + firmware MAX_DEVIATION are primary.

### Phase 2 — waypoint moves + full gate

- `GoToPose.code` is `string`, not the plan's `int32` — the string is the feasibility
  enum verbatim (no int↔string map to drift), and operator-legible (`LIMIT_ACC`, not
  `4`). Phase 5's `TimedTarget` follows suit.
- Stretch loop starts at the duration floor, not the pose-space pre-size — the exact
  `1/Tⁿ` leg-space scaling converges in one stretch; the pose-space pre-size is only an
  inexact approximation.
- Condition-number reachability check kept as defence-in-depth — no current in-stroke
  pose triggers it (`WORKSPACE` always fires first), but a future geometry/edge could;
  it costs one already-computed `np.linalg.cond`.
- `validate` computes all four peaks (no early-return on first limit failure) — the
  stretch loop needs the worst ratio across vel/acc/jerk/step in one pass.
- `trajectory/diagnostics` stays `DiagnosticStatus` (while `status` is typed) — the
  diagnostics payload is an evolving open-ended bag; `KeyValue` absorbs new keys with
  no interface rebuild (Phase 4 adds peak tracking).
- Two-commit split (pure-motion layer, then ROS surface) — rollback/blame granularity.
- **Audit BLOCKING** — mid-move `go_to_pose` install jumped `u0` back to a ~1.5 s-stale
  seed (the analytic gate runs while the emitter streams the OLD plan) → a permanent
  install-continuity guard (`STALE_STATE` on >0.06 rev drift) + the temporary `BUSY`
  restriction + validate-perf quick wins.

### Phase 3 — SpaceMouse streaming

- Fast gate is a **vectorised finite-difference** gate, not a decimated analytic gate —
  profiling killed the obvious "decimate the SVD" (SVD is ~9 %; the analytic Jacobian
  chain is ~90 %); the finite-difference gate is fully contained in the trajectory
  package with a **bit-identical** step bound, so a divergence can't corrupt the
  analytic service-path gate hardware moves depend on.
- Jerk conservatism (300 samples + ×1.05), not "accept the bias" — an under-reporting
  gate would accept an over-jerk plan at the low session limit; over-rejection just
  keeps the last valid plan.
- Follower runs in the emitter thread (drain-to-latest per 40 Hz tick), not a per-message
  callback or a third thread — natural rate-limit, single `_plan_lock` owner, no extra
  install races.
- Saturation is a **ray-clamp to the existing stroke envelope**, not a new pose box — a
  true geometric nearest-valid clamp would invent a new envelope semantic (a safety fork
  not to auto-pick); the ray-clamp tracks up to the stroke boundary using only the gate's
  existing enforcement.
- Graceful stop is decel-in-place and cannot stop a super-limit velocity (physical
  reality) — audit-corrected: even a gate-limited seed within the decel overshoot of a
  boundary is unstoppable-in-place, so the node uses a pending-stop retry.
- STANDBY-exit replaces catch-and-complete with the graceful stop — more faithful to
  "STANDBY silences the move" (stop in place, don't run on to the old target).

### Phase 4 — lean shaping + ramp tooling

- `lean_gain` semantics: **0.0 = explicit OFF, negative = use config** — prevents every
  bare/legacy caller silently starting to lean the day the config gain is ramped to 0.3.
- Fixed nominal cup arm (+95.1 mm ⇒ 1.66 mm/deg), not the per-pose height-aware arm —
  lean is capped 5°, so the fixed-arm residual is second order; the height-aware arm is
  the Phase-6 `tilt_geometry` port's job.
- Tilt-rate boundary transient accepted with an explicit caveat — position-continuous
  but velocity/accel-stepped at both seams; bounded by the gate, made an A/B watch-item;
  windowed lean deferred. (The earlier "C2-across-installs unaffected" claim was
  **withdrawn** — a shaped install on a hold steps the commanded velocity at t=0.)
- Tilt cap scales derivatives by the instantaneous factor — a feedforward inconsistency
  that lives only in a never-commanded pathological-gain regime.
- `/diagnose` jerk headroom emits **both** realized (knot-rate proxy) and predicted
  (gate-authoritative fine peak) — the reviewer ramps against `used_pct_predicted.jerk`,
  not the coarse realized value.
- **Audit BLOCKING** — a shaped `state_at(T)` seam bug (the terminal-hold branch fired
  at the gate's final grid sample) fabricated a 721,215 mm/s³ jerk spike that inflated
  shaped lateral moves 5–8×; fixing the boundary + bisecting the stretch overshoot
  brings gain-0.3 lateral moves to an honest **1.45×** the unshaped minimum.

### Phase 5 — timed targets

- **Supersede design: the fast `validate_follow` gate**, not guard+retry or a
  predicted-install seed — guard+retry rejects *every* supersede while moving (the
  377 ms analytic gate leaves the seed stale past the guard bound); a predicted seed
  trades a measurable drift for an unmeasurable one; only the fast gate keeps the seed
  fresh (drift ≪ the 0.06 rev guard bound) so the supersede is both *accepted* and
  *safe*.
- `hold_after`: True = hold-at-target, False = return-to-neutral — **not** fly-through:
  an unsuperseded fly-through leaves a nonzero end velocity that the implicit terminal
  hold snaps to zero = unbounded jerk with no wired successor.
- Rest-termination is **forced**, not optional — a nonzero-velocity final segment is a
  step discontinuity in commanded leg velocity; a nonzero arrival velocity always gets a
  decel-to-rest continuation, gated as part of the assembled plan.
- The catch path reuses `build_timed` (not a parallel catch builder yet) —
  everything through `planner` → the gate; the richer `build_catch` is Phase 6.
- One clock crossing; the catch path needs none — `perf_counter` is system-wide
  `CLOCK_MONOTONIC`, so only the ROS-clock `timed_target` arrival is converted.
- **Audit** — `max_timed_lead_s = 60 s` clock-domain guard (an epoch-magnitude "lead"
  drove a ~7e10-element `np.arange` → MemoryError that killed the node); CATCH added to
  `_MOTION_MODES` (graceful stop on both CATCH directions); reach-freeze releases after
  `arrival + settle` (was latched forever) with a distinct `FROZEN` feedback code.

### Phase 6 — catch trajectory + sim gate

- Gate is production-in-the-loop on the **main** sim plant, not the bb port — the main
  `MuJoCoPlant`/`jugglebot.xml`/`ball/manager.py` already support a single-ball catch,
  so a sim pass transfers with no port.
- Six-file diff-audit: the bb divergences are **multi-ball** features (Phases 8/9), NOT
  catch-critical — nothing ported but `juggle_noise.py`.
- **Did NOT change `CATCH_VEL_RATIO` 0.9→0.6** in Phase 6 — a shared sim file whose flip
  risks a red suite in unaudited hand tests, a hand-behaviour param gated behind hardware
  evidence, and the flip wouldn't change the light-scope outcome (0.6 makes ≤15 % *more*
  clearly unmeetable). Deferred as a HIGH open question; landed in Phase 7 (blast radius
  was in fact tiny — one symbolic test ref).
- `build_catch` gate is the fast `validate_follow` — matches the Phase-5 catch path,
  catch is never lean-shaped (shaping-blindness is a guard), step bound bit-identical.
- Tilt-through-seat residual rate small + fixed (0.07 rad/s) — not zero (a parked rim
  deflects the ball) and not large (would blow the ceiling / hold-quiescence); exposed
  as a `build_catch` parameter for hardware tuning.
- **Vel-match ≤15 % light-scope deferral** — root-caused as a sim contact→instant-hold
  artifact (the ~14 ms Teensy velocity-hold is narrower than the achievable ±20 ms
  capture-timing alignment; the cup axis IS aligned — the mismatch is a uniform ~0.74×
  magnitude), reinforced by the 0.6 hand design.

### Phase 7 — reload action

- The receive tilt stays in `catch_coordinator` (shipped in `target_quat`), **not** moved
  to `target_vel` — `target_vel` is always zero, and moving the tilt computation would be
  a cross-subsystem contract change to a hardware-proven node (a HARD-STOP fork) for zero
  functional gain; swap `build_timed → build_catch` in `trajectory_node` only.
- The reload coordinator **orchestrates only** — a second motion path (a coordinator
  commanding the platform directly) would bypass `feasibility.validate` and fight the
  hand arming; the reload is a *composition* of proven paths.
- Synthesized-message integration test, not rosbag replay — the recorded bags predate
  this pipeline (a replay validates a dead path) and there is no rosbag2 reader in the
  mocked CI; the test drives the **real** BallTracker + CatchCoordinator engines.
- `BallButlerThrow` gets `aim_only` as a third field, not a separate service — one
  handler with a `throw = not aim_only` branch reuses the world→BB-local + aim + IK chain
  exactly.
- **Audit — 5 BLOCKING choreography bugs** (see the retrospective below): `target_id='point'`;
  the announcement dropped during AIMING; the settle deadline omitting ToF; concurrent
  goals double-throwing; `FROZEN`/`STALE_STATE` latching `MISSED_INFEASIBLE`. Plus the 12°
  tilt clamp.

## Discussion — the audit-arc retrospective

**7 audit rounds, ~62 findings fixed, 7 of them BLOCKING** (Phase 2 ×1, Phase 4 ×1,
Phase 7 ×5). The audit gate earned its place: it is where the run's most dangerous
defects were caught, and the distribution of BLOCKINGs is itself the lesson.

**The pattern that mattered — mocked-ROS unit tests cannot see cross-process message
choreography.** Every phase's per-node tests ran against a *mocked* ROS graph, which is
the right tool for a node's internal logic but structurally blind to the ordering that
emerges when independent processes exchange real messages over time. Phase 7 made this
vivid: all five BLOCKING findings were cross-process choreography bugs sitting on the
*nominal* reload path, invisible to green per-node suites —

1. `_send_throw` omitted `target_name`, so BB announced `target_id='point'` and the whole
   catch pipeline (tracker → CatchCoordinator → the reload's own announcement filter)
   dropped the ball;
2. BB publishes the `ThrowAnnouncement` *synchronously inside* the `throw_at_target`
   handler — before the service returns, i.e. while the FSM is still AIMING — so the
   phase-gated `note_announcement` discarded it deterministically (re-gated on the throw
   being *commanded*);
3. the settle deadline treated RELEASE as landing, omitting the real 0.61–0.73 s
   time-of-flight, so a nominal catch read MISSED (now anchored on the announced landing,
   ROS→perf converted);
4. `_goal_callback` accepted concurrent goals and rclpy runs accepted goals concurrently
   → a double-throw (now REJECTs a second goal while a reload is active);
5. `FROZEN`/`STALE_STATE` catch feedback — *expected* on every real flight inside the
   reach-freeze window — latched `MISSED_INFEASIBLE` forever (now dropped; a later accept
   clears an earlier reject).

target_id routing, announcement ordering, ToF arithmetic, goal concurrency, and feedback
semantics were **all** only visible to an auditor tracing the real ordering across
processes. The fix package added **production-ordering tests** that pin those seams
(an announcement between the throw decision and the throw result is honored; the settle
deadline anchors on landing + confirm; a second goal is rejected while the first is
active), so the seams are now regression-guarded, not just patched.

**What was ruled out and why** (the digest is the full list; the load-bearing rejections):
the `target_vel` tilt-move was ruled out as a needless contract change to a proven node;
a rosbag-replay integration test was ruled out as validating a dead pipeline; a
`CATCH_VEL_RATIO` flip in Phase 6 was ruled out on rollback-granularity + it not changing
the outcome; a decimated-analytic fast gate was ruled out because the Jacobian chain (not
the SVD) is the cost and a batched-Jacobian divergence would corrupt the *analytic*
hardware gate; a geometric pose-box saturation clamp was ruled out as inventing a new
safety envelope autonomously; and a fly-through `hold_after` was ruled out because an
unsuperseded fly-through produces an unbounded-jerk terminal snap.

**Honest limits.** There is **no hardware validation yet** — every claim in this arc is
software/sim, and the runbook exists precisely because the interesting failures live on
the bench. **Contact quality has no sim criterion**: under MuJoCo's contact→instant-hold
capture model a caught ball can never separate, so the separation metric is vacuously 0
and "caught" is a geometric event. The core reload gate therefore validates
*platform-side* behaviour only (reach-under + quiescent hold, decisively resolved at
≤0.02 mm hold travel). **Phase 7b's two-consecutive-bounce-out abort is the operative
guard for contact quality** — the one place where the "prior sim smoothness was not real"
caveat is answered by hardware, not sim. The vel-match deferral does not weaken the gate
because the gate never claimed to validate contact quality in the first place.

## Verification

Final gate triples (the run's terminal state, Phase-7 audit-fix package):

- Full suite: `pytest tests/ -q` (run 2026-07-08) = **2274 passed, 1 xfailed in
  553.60 s** — 0 real failures.
- ci-deep: `pytest tests/ -q --hypothesis-profile=ci-deep` (run 2026-07-08) =
  **2274 passed, 1 xfailed, 198 warnings in 3024.70 s (0:50:24)**.

Baseline before Phase 1 (`pytest tests/ -q`, 2026-07-07): 1956 passed, 1 xfailed. Net
**+318 passed** across the arc, every increment accounted for by new tests only; the
single xfail is unchanged throughout. `colcon build --packages-select
jugglebot_interfaces jugglebot` finished clean at every phase that touched interfaces.

## Open Questions — the consolidated bench-must-answer list

The same list the runbook's "Open items" section carries; the bench answers these:

1. **Catch z-convention (744.3 mm world) + QTM-frame mapping** — hardware-UNVERIFIED
   until **7a** passes (aim-only, zero balls, zero JB motion). The z is the STOW height
   plus the STOW→ACTIVE lift (574.3 + 170.0). If 7a needs a correction, update
   `reload_sequencer.compute_catch_point_mm` and re-run the software gate.
2. **Vel-match criterion redefinition** with 7b/7c evidence — the ≤15 %-at-first-contact
   metric is inconsistent with the 0.6 hand design (a *designed* ~40 % first-contact
   mismatch, absorbing over the stroke); redefine it (measure over the seat stroke, or
   match to the 0.6 design) against hardware, not sim.
3. **Reach envelope vs offsets** — two nominal gate trials exceed the ≤80 mm reliable
   envelope (offset + lever shift + noise → 89/92 mm); still caught. Tighten the offset
   spec or widen the envelope with 7c evidence.
4. **Tilt clamp behaviour beyond 12° collinearity** — real BB arrivals are 18–40° off
   vertical and are now clamped at 12° (partial collinear seating, the hand absorbs the
   residual). Watch partial-tilt catches in 7c; a steep off-centre bounce-out is a
   seating-margin signal, not a gate reject.
5. **Hand-telemetry CAUGHT cross-check** — MVP `CAUGHT` is a tracker-id-correlated
   in-flight estimate (last-KF horizontal miss, not a settled rest position). Implement
   the stronger hand-telemetry rest cross-check **only if 7c shows false CAUGHTs**.
6. **Emitter jitter p95** — read it from the DEBUG install-latency logs during the motion
   sessions (the "~2 ms install window" was reworded to "single-digit ms typical,
   guard-bounded"); the guard, not the estimate, carries the safety claim.

## Related

- Operator runbook: [`tests/hardware/mvp_bench_runbook.md`](../tests/hardware/mvp_bench_runbook.md) — the single consolidated bench sequencing + checklists layer.
- Plan: [`plans/active/mvp-trajectory-bringup.md`](../plans/active/mvp-trajectory-bringup.md) — the full architecture + every phase Outcome + Deferred section.
- Phase entries: [P1](2026-07-07-mvp-phase1-streaming-foundation.md) · [P2](2026-07-07-mvp-phase2-waypoint-moves.md) · [P3](2026-07-08-mvp-phase3-spacemouse-streaming.md) · [P4](2026-07-08-mvp-phase4-shaping-ramp-tooling.md) · [P5](2026-07-08-mvp-phase5-timed-targets.md) · [P6](2026-07-08-mvp-phase6-catch-trajectory-sim-gate.md) · [P7](2026-07-08-mvp-phase7-reload-action.md).
- Phase-6 gate evidence (committed): `logbook/artifacts/2026-07-08-mvp-phase6/` — the five scored reload-gate JSONs (nominal + arm ±30 ms + event_vel ±10 %) with the deterministic regenerate recipe in the Phase-6 entry.
