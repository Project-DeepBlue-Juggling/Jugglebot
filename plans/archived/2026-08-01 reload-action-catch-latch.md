# Plan — RELOAD as a self-contained action; retire CATCH & SHELL modes

**Status:** implemented, ARCHIVED 2026-08-01 (design approved 2026-07-20; software landed 2026-07-21 —
Phase 0 SHELL delete `4572925`, Phase 1 catch-armed latch `caea3ec`, Phase 2 action
owns latch+hand `790e943`, Phase 3 CATCH delete `5d5f4ae`, Phase 4 docs+logbook this
commit; Q1 cup-plane fix `bdbd186` landed as groundwork. Phase 5 hardware validation
is operator-run and deferred. Logbook: `logbook/2026-07-20-reload-action-catch-latch.md`.)
**Branch:** `mvp-trajectory-bringup`
**Supersedes the CATCH-mode framing in:** `plans/active/mvp-trajectory-bringup.md` § Phase 7,
`tests/hardware/session_phase7_reload.md`, `ros_ws/docs/control_modes.md`.

## Motivation (the reframe)

The operator questioned whether a bespoke persistent **CATCH** control mode is needed "just to
catch a ball." Investigation confirms it is not. The entire reactive-catch pipeline —
`ball_tracker_node`, the `CatchCoordinator` policy, `compute_catch_orientation` (the receive
tilt), and the hand prime/gains/arm services — **already runs independently of control mode**.
CATCH mode does exactly one load-bearing thing: it flips a single gate in
`trajectory_node._on_dynamic_target` (`if self._current_mode != 'CATCH': return`,
`trajectory_node.py:1979`) that lets the tracked-ball tilt targets reach `planner.build_catch`
and actuate the platform.

So **"RELOAD supersedes CATCH" means: raise that gate from the RELOAD action for its duration,
instead of from a persistent mode the operator has to hold.** The catch *mechanics* do not move;
only their *trigger* changes. A persistent mode is the wrong abstraction for a one-shot catch.

Separately, **SHELL** mode is a legacy carry-over. It is treated identically to **GUI** mode in
every code path (`_POSE_MODES`, `_FOLLOWER_MODES`, `_ACTIVE_MODES`); there is no dedicated SHELL
producer, and GUI mode fully covers it. It is a clean 1:1 delete.

## Approved design (operator-confirmed 2026-07-20)

RELOAD stays the existing **`jugglebot/reload` Action** (keeps phase feedback, cancellation, and
the pure-Python `reload_sequencer` FSM), upgraded to **own the platform for its duration** and run
from **ACTIVE** (armed + streaming — the live catch is a streaming trajectory, so RELOAD cannot
copy LEVELLING, which *suspends* the stream; it runs within `ACTIVE:TRAJECTORY` instead).

Sequence (operator's spec):
1. **CHECKING** — preconditions (ACTIVE, armed, streaming a hold, mocap fresh, BB IDLE); reload BB
   if the hand is empty.
2. **AIMING** — aim BB at the **cup plane** `GEOM_INITIAL_HEIGHT + ACTIVE_Z + HAND_CATCH_OFFSET =
   809.08 mm` (the Q1 fix, committed as groundwork before this refactor) and throw.
3. **On BB throw-accept** — proactively **prime the hand to top** (`JB_OP_HAND_CATCH_PRIME_REV =
   9.858`) so the catch stroke never races the ball with a high-jerk late prime, confirm the level
   catch-ready pose (via `go_to_pose`/`go_home` — available now that we're in TRAJECTORY, not a
   gated mode), and **raise the catch-armed latch**.
4. **BALL_IN_FLIGHT / CATCHING** — the latch lets `catch/dynamic_target → build_catch` fire; the
   existing reactive tilt aligns the cup to the measured arrival; the hand catches.
5. **SETTLING → CAUGHT** — lower the latch, **re-center** to level neutral.
6. **Abort (any reason)** — **retract the hand to bottom** (`HOMING_HAND_ABS_POS_REV ≈ 0 rev`) and
   **re-center**; lower the latch.

Hand ownership split (least-risk): the RELOAD action owns the **proactive prime** (step 3) and the
abort **retract**; `catch_coordinator_node` keeps the **reactive fire** (`set_hand_traj_cmd`, timed
to the tracked ball) and catch gains, but its hand-arm is **gated on the catch-armed latch** so it
never fires on a stray ball outside a reload.

CATCH and SHELL are **deleted** from the `ActiveMode` enum and every gate.

## Removal surface (from the 2026-07-20 investigation)

**Canonical mode enum:** `ActiveMode` in `state_machine.py:34-41`
(`STANDBY, TRAJECTORY, SPACEMOUSE, SHELL, GUI, CATCH`). The 6-mode list is duplicated and must
stay in sync at: `state_machine.py:589-590` (command allowlist), `trajectory_node.py:144-145`
(`_DEFAULT_STREAM_MODES`), `:152` (`_FOLLOWER_MODES`), `:162` (`_MOTION_MODES`), `:167`
(`_CATCH_MODE`), `mpc_bridge_node.py:57,59`, `motion_bridge_node.py:103`,
`ros_ws/gui/js/state-minimap.js:87` (`SUBMODES`), `tests/ros/test_state_machine.py:1125`.
(`mpc_bridge_node` is dropped from the MVP launch — dormant, but cleaned for grep-to-zero.)

**SHELL is a clean delete** — ~9 code sites + the allowlist + GUI `SUBMODES` + `docs/control_modes.md`
+ `docs/safety.md` + `tests/ros/test_state_machine.py` (5) + `tests/ros/test_orchestrator_node.py`
(1) + `tests/sim/test_zmq_target.py` label (soft). No functional breakage (GUI covers it).

**CATCH is NOT a clean delete** — its mode is one gate, but the surrounding `trajectory_node`
motion-transition safety logic is entangled and must be **re-keyed to the latch, not deleted**:
`_MOTION_MODES` membership, `entering_catch_mid_move` / `leaving_catch_mid_move` graceful
decel-to-rest stops, and the `_catch_arrival_perf` reach-freeze-window reset (`trajectory_node.py:
698-768, 1898-1922, 1967-2022`). `reload_sequencer` itself hard-depends on CATCH
(`:78, 195-198`) and must drop those checks.

## Phased plan

Each phase is full-`pytest tests/ -q`-gated and committed separately (rollback granularity).

### Phase 0 — Delete SHELL mode (clean, independent, first)
- Remove `SHELL` from `ActiveMode` (`state_machine.py:39`) + command allowlist (`:589`).
- Remove `SHELL` from `trajectory_node._DEFAULT_STREAM_MODES` / `_FOLLOWER_MODES`,
  `mpc_bridge_node._POSE_MODES`, `motion_bridge_node._ACTIVE_MODES`, and their docstrings.
- GUI: remove from `state-minimap.js` `SUBMODES` + header docstring + `tools/probes/gui_dom_probe.py`
  hardcoded `ACTIVE:SHELL`.
- Docs: drop the SHELL rows in `docs/control_modes.md`, `docs/safety.md`.
- Tests: `test_state_machine.py` (SHELL enum/transition/`_STREAM_MODES`), `test_orchestrator_node.py`
  (command iteration), `test_zmq_target.py` label → `gui`.
- **Verify** `grep -rn "SHELL" ros_ws/src tests/` drops to zero mode-references (only `$SHELL`/`.STEP`
  noise remains). Full pytest. Commit. `/log refactor`.

### Phase 1 — Introduce the catch-armed latch (the careful control-flow rewrite)
- Add a `trajectory/arm_catch` service (`SetBool` or a bespoke Trigger pair) + a `_catch_armed`
  flag on `trajectory_node`.
- Replace the gate at `_on_dynamic_target:1979` (`_current_mode != 'CATCH'`) with `not _catch_armed`.
- **Re-key the motion-transition safety logic** from CATCH entry/exit to latch raise/lower:
  the graceful decel-to-rest on arm/disarm mid-move, and the `_catch_arrival_perf` freeze reset.
  **Control-system analysis required before editing** (per CLAUDE.md): walk one catch cycle
  step-by-step — arm → dynamic_target reach → freeze window → seat → disarm → recenter — and
  confirm no discontinuity at the arm/disarm seams that CATCH entry/exit currently smooths.
- Keep the emitter streaming during the catch: RELOAD runs in `TRAJECTORY` (already in the stream
  set), so no stream-set change is needed — only the freeze-reset re-key.
- Tests: port `test_trajectory_node.py` CATCH-gate tests (`:1329, 1549, 1637-1667`) to the latch.
  Full pytest. Commit. `/log refactor` or `/investigate` if a control-flow subtlety surfaces.

### Phase 2 — Upgrade the RELOAD action to own the latch + hand + platform
- `reload_coordinator_node`: add clients for `trajectory/arm_catch`, `smooth_move_hand`,
  `go_to_pose`/`go_home`. Build on the **Q1 catch-point fix** (809.08, committed as groundwork).
- `reload_sequencer`: drop the CATCH precondition/abort (`:78, 195-198`); add PREPARE (prime + latch
  raise), RECENTER (success), SAFE_ABORT (retract + recenter) actions; the FSM emits them and the
  node executes. Precondition becomes ACTIVE + streaming (keep `NOT_STREAMING`).
- Gate `catch_coordinator_node`'s hand-arm on the latch (subscribe to reload phase / the arm signal).
- Tests: rewrite `test_reload_sequencer.py`, `test_reload_coordinator_node.py`,
  `test_reload_integration.py` off the CATCH assumption; add the new-action-sequence coverage.
  Full pytest. Commit. Logbook entry (feature) + `/audit`.

### Phase 3 — Delete CATCH mode
- Remove `CATCH` / `_CATCH_MODE` from `ActiveMode`, the allowlist, `_DEFAULT_STREAM_MODES`,
  `_MOTION_MODES`, `mpc_bridge_node`, `motion_bridge_node`, GUI `SUBMODES`, `gui_dom_probe.py`.
- Any remaining dead CATCH branches in `trajectory_node` (now that the latch owns the path) removed.
- **Verify** `grep -rn "CATCH" ...` leaves only mechanics (`build_catch`, `CATCH_EVENT`,
  `HAND_CATCH_OFFSET`, `JB_TRAJ_CATCH_*`, `PHASE_CATCHING`). Full pytest. Commit. `/log refactor`.

### Phase 4 — Docs + logbook + audit
- Rewrite `tests/hardware/session_phase7_reload.md` (no "operator keeps CATCH"; the action drives
  everything; 7a aim = 809.08). Update `docs/control_modes.md`, `docs/safety.md`,
  `plans/active/mvp-trajectory-bringup.md` § Phase 7, `mvp_bench_runbook.md`. `/audit --unstaged`.

### Phase 5 — Hardware validation (operator-run)
- New/updated Phase-7 session: aim-only frame+z check (809.08), static catch, full reload action
  (prime → throw → reactive catch → recenter), each abort path (no-ball, no-announcement, cancel).

## Risks / control-system notes
- **Phase 1 is the linchpin.** The `trajectory_node` catch motion-transition logic is safety code
  (graceful stops on mode change, reach-freeze). Re-keying it to the latch is a control-flow
  rewrite; analyze one full cycle before editing and add a test that the arm/disarm seams produce
  no command discontinuity.
- **Two platform-command sources during a reload** — the reload action (`go_to_pose`/`go_home` for
  pre-position/recenter) and the reactive catch (`dynamic_target`). They must be temporally
  disjoint: latch raised only for the flight window; pre-position before, recenter after. The
  action sequences them; verify no overlap in the FSM.
- **`catch_coordinator` hand-arm gating** — without CATCH as the implicit "operator intends to
  catch" signal, the hand must arm only during a reload (latch-gated), else it fires on any stray
  tracked ball.

## Reference
- Investigation: 2026-07-20 session (three read-only agents: LEVELLING template, CATCH/SHELL removal
  surface, catch-mechanics re-hosting). Q1 catch-point offset fix (809.08) committed as groundwork.

---

## Archival note (2026-08-01)

**Archived as completed.** Phases 0–4 landed in software on 2026-07-21
(`4572925` SHELL delete, `caea3ec` catch-armed latch, `790e943` action owns
latch + hand, `5d5f4ae` CATCH delete, `d25b7b8` docs; `bdbd186` cup-plane
groundwork). The Phase 5 hardware validation this plan deferred to the operator
has since been run — see `logbook/2026-07-23-phase7-reload-first-hardware-session.md`
and `logbook/2026-07-24-phase7-fourth-sitting-openloop-telemetry-ladders.md`
(15/19 caught; the reload platform pivoted to open-loop by config default).

Continuing catch work is tracked by `plans/active/mvp-trajectory-bringup.md`
and the hand-ball-sensor / single-ball-toss plans, not here. One known-stale
detail is deliberately left in place: the 9.858 rev hand prime cited near the
top of this plan was superseded by the rev-derived prime
(`logbook/2026-07-26-hand-prime-rev-derived.md`).

Moved out of `plans/active/` by the 2026-07 refactor programme
(`plans/active/refactor-2026-07.md` § Phase 1, item 5).
