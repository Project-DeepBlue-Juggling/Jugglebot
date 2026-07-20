---
title: RELOAD becomes a self-contained action — a catch-armed latch replaces the persistent CATCH mode (SHELL retired too; 809.08 cup-plane aim)
type: refactor
date: 2026-07-20
status: resolved
phase: "MVP trajectory bringup — Phase 7 reload: action-driven catch + arm_catch latch (retire CATCH & SHELL)"
related_plan: reload-action-catch-latch.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/state_machine.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/reload_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/mpc_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/motion_bridge_node.py
  - ros_ws/gui/js/state-minimap.js
  - tools/probes/gui_dom_probe.py
  - ros_ws/docs/control_modes.md
  - ros_ws/docs/safety.md
  - tests/hardware/session_phase7_reload.md
  - tests/hardware/mvp_bench_runbook.md
  - plans/active/mvp-trajectory-bringup.md
  - plans/active/reload-action-catch-latch.md
commits:
  - bdbd186
  - 4572925
  - caea3ec
  - 790e943
  - 5d5f4ae
subsystem:
  - ros
  - motion
tags:
  - trajectory
  - reload
  - catch
  - control-modes
  - refactor
---

# RELOAD as a self-contained action; CATCH & SHELL modes retired

## Summary

The operator questioned whether Jugglebot needs a bespoke persistent **CATCH** control
mode "just to catch a ball." It does not. The entire reactive-catch pipeline —
`ball_tracker_node`, the `CatchCoordinator` policy, `compute_catch_orientation` (the
receive tilt), and the hand prime/gains/arm services — already runs independently of
control mode. CATCH mode did exactly **one** load-bearing thing: it flipped a single gate
in `trajectory_node._on_dynamic_target` (`if self._current_mode != 'CATCH': return`) that
let the tracked-ball tilt targets reach `planner.build_catch` and actuate the platform.

So "RELOAD supersedes CATCH" meant: **raise that gate from the `jugglebot/reload` action
for its duration, via a catch-armed latch, instead of from a persistent mode the operator
has to hold.** The catch *mechanics* did not move; only their *trigger* did. Alongside,
the legacy **SHELL** mode (a 1:1 alias of GUI with no dedicated producer) was deleted, and
the Q1 catch-aim bug (aiming at the 744.3 mm platform centroid instead of the 809.08 mm
cup plane) was fixed as groundwork.

Landed as five commits on `mvp-trajectory-bringup`:

- `bdbd186` (groundwork) — aim BB at the **809.08 mm cup plane**, not the 744.3 mm
  centroid (the Q1 fix).
- `4572925` (Phase 0) — delete the **SHELL** mode (GUI supersedes it).
- `caea3ec` (Phase 1) — introduce the **catch-armed latch** on `trajectory_node`.
- `790e943` (Phase 2) — the `jugglebot/reload` action **owns** the latch + hand + platform
  (proactive prime, recenter, abort-retract).
- `5d5f4ae` (Phase 3) — delete the **CATCH** mode; the gate is now latch-only.

This entry (Phase 4) updates the docs, runbook, plan, and writes this record.

## Discussion

### Why a latch, not a persistent mode

A persistent mode is the wrong abstraction for a one-shot catch. A mode is *operator
state* the human sets and holds; catching a single thrown ball is a *bounded episode*
with a clear owner (the reload action) and a clear lifetime (throw-accept → seat →
recenter). Encoding "the operator intends to catch" as a mode has three costs the latch
avoids:

1. **The operator has to hold it, and remember to leave it.** Leaving a streaming mode
   while armed latches `MPC_STALE` within 250 ms (Sharp Edge #1); CATCH mode put a
   ball-catch on the wrong side of that trap. The reload action never changes mode — it
   runs *within* `ACTIVE:TRAJECTORY` (already streaming + motion-capable) and toggles the
   latch, so the streaming invariant is never disturbed.
2. **A mode implies a producer.** CATCH had none of its own — it reused the
   coordinator's mode-independent `/balls` handler and only gated the platform tilt. That
   is exactly a *gate*, not a *mode*: one boolean, not a member of the mode partition.
3. **A stray ball under a held mode arms the hand.** With catching keyed to a mode the
   operator holds, any tracked ball fires the hand. Keying the hand-arm to the same latch
   (via the `catch/armed` topic) means the hand actuates **only during a reload**.

### What was ruled out

- **The orchestrator-state form** (make "catching" a first-class state-machine state that
  the reload action drives). Rejected: it re-introduces the mode-partition problem (a new
  member of `ActiveMode`, new transition edges, new allowlist entries duplicated across
  seven files) for something that is a single gate. The latch is a private flag on the one
  node that owns the gate, with a `SetBool` service and a mirror topic — no cross-process
  mode choreography to keep in sync.
- **The LEVELLING stream-suspend pattern.** LEVELLING was the other "action-shaped"
  precedent — but it *suspends* the 40 Hz stream and hands the platform to the CAN node's
  trapezoidal profiles. RELOAD **cannot** copy that: the live catch is a *streaming*
  trajectory (`catch/dynamic_target` → `build_catch` → the emitter), so the stream must
  stay up throughout. RELOAD runs inside `ACTIVE:TRAJECTORY` and never leaves the stream
  set; only the freeze-reset + graceful-stop bookkeeping is re-keyed to the latch edges.
- **Deleting the CATCH motion-transition safety logic outright.** CATCH was *not* a clean
  delete (unlike SHELL). The surrounding `trajectory_node` logic — the graceful
  decel-to-rest on entering/leaving CATCH mid-move, and the `_catch_arrival_perf`
  reach-freeze-window reset — is safety code that had to be **re-keyed to the latch, not
  removed**. Phase 1 factored it into two shared helpers (`_reset_catch_reach_freeze`,
  `_install_graceful_stop`) called by *both* the (then still-present) mode transition and
  the new latch edges, so nothing broke mid-refactor; Phase 3 then removed the dead
  mode-keyed branches once the latch was the sole owner.

### The control-flow seam (Phase 1 analysis)

Per the CLAUDE.md "analyze one MPC cycle before editing" rule, the arm/disarm seams were
walked step-by-step: **arm** (False→True) mirrors entering CATCH — freeze reset + a
graceful stop if a move is in flight; the `dynamic_target` reach installs `build_catch`
through the unchanged fast `validate_follow` gate; the freeze window holds the committed
reach into the seat (bounded release once arrival+settle passes); **disarm** (True→False)
mirrors leaving CATCH — freeze reset + a graceful stop if mid-reach (the documented abort:
the reach is silenced, never run on to the catch target); recenter is the existing
`go_home` path. Every graceful stop is seeded C2 off `_current_state()`, so there is **no
command discontinuity** at either seam — bit-identical to the mode-transition stop it
replaced.

### The RELOAD-action upgrade (Phase 2) and the hand-ownership split

The action was upgraded from "orchestrate only, operator holds CATCH" to "own the platform
+ hand for the reload's duration." The FSM (`reload_sequencer`, pure Python) now emits
three node-executed actions:

- **PREPARE** (on throw-accept, AIMING→THROW_PENDING): proactively prime the hand to top
  (`JB_OP_HAND_CATCH_PRIME_REV = 9.858`) **and** raise the catch-armed latch. Priming
  proactively — rather than letting the coordinator prime reactively on the first
  catchable ball — stops the catch stroke from racing the ball with a high-jerk late
  prime.
- **RECENTER** (terminal CAUGHT): lower the latch, `go_home` (the hand keeps the ball, no
  retract).
- **SAFE_ABORT** (any not-caught terminal once prepared, or a cancel/timeout/shutdown
  early exit): retract the hand to bottom (`HOMING_HAND_ABS_POS_REV ≈ 0`), lower the
  latch, `go_home`.

**Hand ownership** was split for least risk: the reload action owns the *proactive* prime
and the abort *retract*; `catch_coordinator_node` keeps the *reactive fire*
(`set_hand_traj_cmd`, timed to the tracked ball) and the catch gains — but its hand-arm is
now **gated on the `catch/armed` latch** (the coordinator subscribes; the reload
coordinator publishes on the same PREPARE/RECENTER/SAFE_ABORT edges that drive
`trajectory/arm_catch`, so the two stay in lockstep). This keeps the proven reactive-fire
timing exactly where it was validated, and only changes its *enablement*.

The two platform-command sources — the action's `go_home` recenter and the reactive
`catch/dynamic_target` — are **temporally disjoint by construction**: the latch is raised
only between PREPARE and the terminal, and `go_home` runs only at the terminal after the
latch is lowered.

### The Q1 aim fix (809.08)

`reload_coordinator_node` had passed only `(initial_height, active_z)` to
`compute_catch_point_mm`, defaulting `landing_z_offset` to 0 → the reload aimed BB at the
platform **centroid** (744.3 mm). But the ball is caught by the hand **cup**, which sits
`HAND_CATCH_OFFSET_MM = 64.78` mm above the centroid, and the rest of the catch stack
(`throw_ballistics._DEFAULT_CATCH_HEIGHT_MM`, `ball_tracker` landing_z,
`catch_coordinator`) all use **809.08**. Aiming 64.78 mm low pushed the ball's true-plane
crossing off-centre (toward BB) and ate tilt/reach margin on every catch. The fix passes
`landing_z_offset_mm = HAND_CATCH_OFFSET_MM` so the aim lands on the cup plane. Note the
**744.3 mm** value survives elsewhere as `CUP_TILT_CENTER_Z_MM` — a *different* quantity
(the cup tilt lever-arm reference in `tilt_geometry`/`shaping`), not the aim point; those
constants are unchanged.

## Fix

- **Phase 0** (`4572925`): removed `SHELL` from the `ActiveMode` enum + command allowlist
  (`state_machine`), the `trajectory_node` / `mpc_bridge` / `motion_bridge` mode-sets +
  docstrings, the GUI `state-minimap` `SUBMODES`, `gui_dom_probe`, `control_modes.md`,
  `safety.md`, and the affected tests. No functional change (GUI covers it).
- **Phase 1** (`caea3ec`): added `_catch_armed` + the `trajectory/arm_catch` (`SetBool`)
  service; the `_on_dynamic_target` gate fires when the latch is armed (transitional:
  latch OR CATCH mode until Phase 3); the CATCH-entry/exit bookkeeping factored into the
  two shared helpers, called by both the mode transition and the latch edges.
- **Phase 2** (`790e943`): `reload_sequencer` drops the CATCH precondition/abort and
  re-keys `WRONG_MODE`/`MODE_CHANGED` to `RELOAD_CONTROL_MODE = TRAJECTORY`; adds the
  PREPARE / RECENTER / SAFE_ABORT actions + the `_prepared` gate. `reload_coordinator_node`
  gains the `trajectory/arm_catch`, `smooth_move_hand`, `trajectory/go_home` clients, the
  `catch/armed` publisher, and `_safe_on_early_exit`. `catch_coordinator_node` gates its
  hand prime/arm on `catch/armed`.
- **Phase 3** (`5d5f4ae`): removed `CATCH` / `_CATCH_MODE` from the enum, allowlist,
  `_DEFAULT_STREAM_MODES`, `_MOTION_MODES`, `mpc_bridge`, `motion_bridge`, GUI `SUBMODES`,
  `gui_dom_probe`; the `_on_dynamic_target` gate is now latch-only
  (`if not self._catch_armed: return`); the dead mode-keyed graceful-stop / freeze-reset
  branches removed (the latch edges are the sole owner).
- **Phase 4** (this commit): `control_modes.md` + `safety.md` drop the CATCH row and note
  the action + latch; `session_phase7_reload.md` rewritten (no "keep CATCH mode"; the
  action drives everything; 7a/7b/7c aim at 809.08; 7b static catch via the manual
  `catch/armed` + reach-latch-down split); `mvp_bench_runbook.md` S6/S7/S8 + Sharp Edge #3
  updated (744.3 → 809.08; no CATCH mode); `mvp-trajectory-bringup.md` § Reload sequence +
  § Phase 7 reframed; the plan marked implemented.

## Verification

- Phase 1 scoped gate (`pytest tests/ros tests/motion -q`, run 2026-07-21):
  **1634 passed in 338.34 s**.
- Phase 2 scoped gate (`pytest tests/ros tests/motion -q`, run 2026-07-21):
  **1648 passed in 337.81 s**.
- Phase 3 scoped gate (`pytest tests/ros tests/motion -q`, run 2026-07-21):
  **1641 passed in 344.23 s**.
- Phase 4 (docs only) scoped gate (`pytest tests/ros tests/motion -q`, run 2026-07-21):
  **1641 passed, 51 warnings in 348.74 s** — no code change, confirming no accidental
  regression.
- The orchestrator runs the full `pytest tests/ -q` as the final gate after all phases
  land.

## Open / deferred

- **Phase 5 hardware validation (operator-run)** — the updated Phase-7 sessions: aim-only
  frame+z check (809.08), static catch, full reload action (prime → throw → reactive catch
  → recenter), and each abort path (no-ball, no-announcement, cancel). See
  `tests/hardware/session_phase7_reload.md`.
</content>
