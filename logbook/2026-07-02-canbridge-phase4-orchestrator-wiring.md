---
title: Can-bridge foundation Phase 4 — orchestrator wiring (home_motors action + activate_or_deactivate + get_platform_tilt + set_level_state) restores automated cold-start parity
type: feature
date: 2026-07-02
status: resolved
phase: "4"
related_plan: canbridge-foundation-coldstart-parity.md
related_entries:
  - 2026-07-01-canbridge-phase5-hand-conduit
  - 2026-06-30-canbridge-phase6-reboot-latch
  - 2026-06-29-canbridge-phase3-version-validated
  - 2026-06-29-canbridge-phase2-coldstart-relay-state
  - 2026-06-29-canbridge-phase1-platform-relay-seam
  - 2026-06-27-can-node-teensy-parity-audit
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/ros/conftest.py
  - tests/ros/test_orchestrator_conduit_contract.py
  - tests/ros/test_teensy_bridge_node_conduit.py
  - tests/ros/test_orchestrator_conduit_integration.py
  - ros_ws/docs/can-node-teensy-parity.md
  - plans/active/canbridge-foundation-coldstart-parity.md
  - logbook/INDEX.md
commits:
  - 34c1730
  - e98a6c9
  - TBD-sitting
subsystem:
  - ros
  - can
  - testing
tags:
  - cold-start
  - orchestrator
  - parity
  - levelling
---

## Summary

Phase 4 — the plan's **final** phase — makes the can-bridge (`teensy_bridge_node`) a
drop-in for the retired `can_node` from the **locked** `orchestrator_node`'s view,
restoring the **automated orchestrator-driven cold-start** the parity audit flagged
as broken. It wires the four interfaces the orchestrator drives cold-start through
and the bridge lacked, delegating each to the bridge's existing verbs — with **zero
edits** to `orchestrator_node` / `state_machine` (locked-decision #1):

1. **`home_motors` ActionServer(HomeMotors)** → the shared `_do_home()` (home legs +
   hand via Phase-5 firmware → persist `is_homed` → configure). The orchestrator's
   `HomingHandler` drives `ActionClient(HomeMotors, 'home_motors')`; the bridge
   previously served only a `home` **Trigger** service (wrong name + type), so
   `server_is_ready()==False` → homing faulted.
2. **`activate_or_deactivate` service (ActivateOrDeactivate)** → `_run_activate` +
   `_run_configure` (the PASSTHROUGH fold) / `_run_deactivate`. The bridge previously
   served split `activate`/`deactivate` **Trigger**s the orchestrator never called.
3. **`get_platform_tilt` service (GetTiltReadingService)** → `relay_read_tilt` with
   bounded retry + a validity bound + the **NaN-on-failure** shape the orchestrator
   already consumes (NaN → `operation_result=False` → `LevellingHandler` → FAULT).
4. **`set_level_state` subscriber (Float64MultiArray)** → new `_write_level_state`
   (relay STATE_WRITE, read-modify-write preserving `is_homed`). The gravity-offset
   persist was silently discarded before (no subscriber).

**ACTIVATE folds a `_run_configure`** so the legs end POSITION/PASSTHROUGH — interp-
ready for `run_mpc.py` as the sole setpoint source — closing audit rows 27/28
(`_run_activate` alone ends TRAP_TRAJ). All four wrappers (plus the pre-existing
blocking `encoder_search`/`home`/`configure`/`activate`/`deactivate` services) share
**one `ReentrantCallbackGroup`** so the multi-second moves never starve the 100 Hz
`robot_state` publish + heartbeat.

The interface parity is pinned by a **runtime drift-guard** contract test that
introspects both nodes' declared `(name, type)` sets — so a future orchestrator
rename fails the suite, not the bench.

This **completes the plan's implementation** — automated orchestrator-driven cold-
start parity is restored.

## Motivation

The `can_node`→Teensy parity audit's headline finding: the production
`jugglebot_launch.py` runs `orchestrator_node` **and** `teensy_bridge_node`
together, but the orchestrator drives cold-start through interfaces the bridge did
not provide (or reported negative). Phases 2/3 fixed the negative `robot_state`
fields (`is_homed`/`levelling_complete`/`pose_offset`/`encoder_search_complete` via
the relay; `firmware_validated` via the Get_Version handshake), and Phase 5 added
hand homing so `_do_home` homes legs + hand. But the four **caller-facing** seams —
the `home_motors` action, the `activate_or_deactivate` service, `get_platform_tilt`,
and `set_level_state` — remained gaps (matrix rows 6, 7, 24, 27, 28, 44, 45, 54,
60). The Phase-5 powered sitting confirmed the consequence and the fix's shape: with
no orchestrator wiring, `orchestrator_node` looped `BOOT→HOMING→FAULT` (its HOMING
handler could not drive the bridge's `/home`) until manual homing set `is_homed`,
then correctly `BOOT→IDLE`. So the skip-if-homed path + the Phase-2/3 chain already
worked end-to-end; Phase 4's job was narrow — make the HOMING/LEVELLING/ACTIVE
handlers actually **drive** the bridge instead of faulting.

## Design

### Registered directly on the bridge (not a separate `orchestrator_conduit.py`)

The plan sketched a new `orchestrator_conduit.py` module. The operator pushed back
(design checkpoint) and we register the four entities **directly on
`teensy_bridge_node`** instead, as `_svc_*`/handler methods next to their siblings.
Rationale (root-cause, not appeal-to-plan): there is **no existing dedicated
actions/services module** in the package — the `bb/throw` action and every
`_svc_*`/`_run_*` live directly on the bridge node, which owns CAN3 + the RPC client
+ the verbs these wrap. A `conduit` module holding four verbs "because they were
implemented together" is cohesion-by-implementation-moment; if the large
`teensy_bridge_node.py` (~2990 lines) warrants splitting, the cut should follow the
*relay* / *hand* / *cold-start* seams (these verbs would land in the cold-start seam alongside
`_run_home`/`_run_activate`), a deliberate refactor — not this. A separate conduit
*node* was also ruled out: it would need its own `TeensyLinkClient`/`RpcClient` and
duplicate CAN3 ownership for zero benefit (same-process direct method calls are the
whole point).

### One ReentrantCallbackGroup for every blocking cold-start verb

Each blocking verb is a multi-second `_run_*` poll loop. Left in the node's default
`MutuallyExclusiveCallbackGroup`, a blocking verb serializes with — and stalls — the
100 Hz `robot_state`/heartbeat timers for the whole move (under `MultiThreadedExecutor`
a mutually-exclusive group runs at most one callback at a time). So all nine — the
four new verbs **and** the pre-existing `encoder_search`/`home`/`configure`/
`activate`/`deactivate` services — share one `ReentrantCallbackGroup`, letting a move
run on one executor thread while the telemetry timers run on others. This is the same
pattern the `bb/throw` action already uses. The plan's folded-critique named only the
new verbs; we extended it to the existing ones too (operator-approved) because
`encoder_search` is on the orchestrator-driven cold-start path — leaving it in the
default group would starve the publish during the index-search phase — and the whole
class is closed for one line each. `clear_errors`/`reboot_odrives`/`odrive_command`
stay in the default group (quick single RPCs, not multi-second moves).

Reentrancy makes concurrent execution *possible*; three things keep it safe: the
firmware busy-rejects a second concurrent move, the orchestrator drives strictly
sequentially (one `operation_pending` at a time), and the one residual race (two
`home_motors` goals both racing `_write_is_homed`) is closed by a `_home_action_goal`
in-progress guard (mirrors `bb/throw`'s single-in-flight guard).

### ACTIVATE folds configure (rows 27/28)

`activate_or_deactivate('activate')` runs `_run_activate` **then** `_run_configure`,
in that order. `_run_activate` ends the legs in TRAP_TRAJ holding the active pose;
`_run_configure` switches them to POSITION/PASSTHROUGH so `run_mpc.py`'s 40 Hz interp
is the sole setpoint source (`can_node` ended PASSTHROUGH; the bridge's `_run_activate`
alone ends TRAP_TRAJ — audit rows 27/28). Configure is safe post-move (legs
CLOSED_LOOP holding pose, motion-free). A configure failure fails the result (legs at
pose but not interp-ready is a real failure for `run_mpc`), mirroring `_svc_home`'s
fold. The configure scopes to `activate_axes` (the legs that moved; the hand is not
part of ACTIVATE — it's rejected on axis 6).

### The runtime drift-guard + the mock-conftest enhancement

The robustness contract is a **runtime** contract test (`test_orchestrator_conduit_
contract.py`) that constructs BOTH nodes and asserts, by `(name, type)`, that the
bridge SERVES every service/action the orchestrator declares as a client, and that
`robot_state` (bridge→orch) and `set_level_state` (orch→bridge) agree. A future
orchestrator rename or type change fails this test, not the powered bench.

Making it deterministic required a small, **additive** enhancement to the mock rclpy
layer (`tests/ros/conftest.py`): it previously recorded service/subscription *names*
only and registered nothing for actions, so a `(name, type)` introspection — and the
action surface — were invisible. The mock now records `(srv_name, srv_type)` /
`(topic_name, msg_type)` on each service/client/publisher/subscription and registers
each action's `(name, type)` on the node. This is preferable to a **real-rclpy**
contract test: real rclpy hides an action's sub-services inside a `Waitable` (they do
*not* appear in `node.services`/`node.clients`), and a real-node test would need
`/opt/ros/foxy` sourced + DDS discovery — non-deterministic and unavailable in the
venv/CI. Both nodes import the same mock interface classes, so `srv_type`/`action_type`
compare by identity; the enhancement is a strict superset (existing tests read the
same dicts by name and are unaffected — 702 `tests/ros/` pass unchanged).

## Implementation

- **`teensy_bridge_node.py`**: refactored `_svc_home`'s body into a shared
  `_do_home() → (ok, msg)` (home+persist+configure) called by both `/home` and the
  action; added `_write_level_state()` (STATE_WRITE read-modify-write preserving
  `is_homed`, mirroring `_write_is_homed`); one `_coldstart_cbgroup =
  ReentrantCallbackGroup()`; moved the five existing blocking services into it and
  registered `home_motors` (ActionServer, with a `_home_action_goal` guard +
  `_home_action_execute`), `activate_or_deactivate`, `get_platform_tilt`,
  `set_level_state` in it; added the four handler methods; added imports
  (`ActivateOrDeactivate`, `GetTiltReadingService`, `HomeMotors`, `Float64MultiArray`).
- **`tests/ros/conftest.py`**: the additive `(name, type)` recording described above.
- **`tests/ros/test_orchestrator_conduit_contract.py`** (new): the drift-guard.
- **`tests/ros/test_teensy_bridge_node_conduit.py`** (new): behaviour of each wrapper
  (home action success/failure/goal-guard/real-`_run_home` e2e; activate-folds-
  configure order + failure modes; deactivate; get_platform_tilt success/NaN/validity;
  set_level_state is_homed-preservation + short-message).
- **`tests/ros/test_orchestrator_conduit_integration.py`** (new): the full BOOT→search
  →home→level→IDLE transition driven against the real conduit handlers (in-process
  DDS stand-in), plus the skip-if-homed path.

## Verification

- **Baseline** (`pytest tests/ -q`, run 2026-07-02, pre-change): **1972 passed, 1
  xfailed in 494.48 s** — fully green (no order-flaky failures this run).
- **`tests/ros/` subset** (`pytest tests/ros/ -q`, 2026-07-02): **702 passed in
  33.26 s** (confirms the conftest enhancement + the `_svc_home` refactor + the
  callback-group change break no existing ros test).
- **New Phase-4 tests** (`pytest tests/ros/test_orchestrator_conduit_contract.py
  tests/ros/test_teensy_bridge_node_conduit.py
  tests/ros/test_orchestrator_conduit_integration.py -q`, 2026-07-02): **23 passed**
  (7 contract + 14 behaviour + 2 integration).
- **Drift-guard divergence-catch — proven**: with the four conduit interfaces
  removed from the bridge's introspection dicts, the contract assertions fire on
  exactly `{activate_or_deactivate, get_platform_tilt}` (services) + `{home_motors}`
  (action) — so the guard is load-bearing, not vacuous.
- **Full suite** (`pytest tests/ -q`, final pre-commit run 2026-07-02): **1995
  passed, 1 xfailed in 456.28 s** — fully green, net **+23** over the 1972 baseline
  (7 drift-guard + 14 behaviour + 2 integration). An earlier run this session hit
  the documented order/load-flaky allocation test
  `test_t3b_h4_on_post_solve_allocates_within_budget` (1994 passed, 1 failed, 1
  xfailed in 462.64 s), which **passes isolated**
  (`pytest …::test_t3b_h4_on_post_solve_allocates_within_budget -q`, 2026-07-02:
  **1 passed in 7.49 s**) and passed clean in the pre-commit run — not a regression
  (memory `project_hot_loop_alloc_test_flaky`).
- **Powered sitting**: TBD (this session — see Open Questions).

Host-only (no firmware change → no `pio run` needed; no wire ids added — codegen
untouched).

## Discussion

### Why the drift-guard is a *runtime introspection* test, not a hardcoded list

A contract test could assert a hardcoded `{(name, type)}` set. But the whole failure
class is **drift between two independently-edited nodes** — the orchestrator is locked
today, but a future edit could rename `home_motors` or change a srv type, and a
hardcoded list would silently agree with the *bridge* while diverging from the
*orchestrator*. Introspecting BOTH nodes and asserting `orch_clients ⊆ bridge_servers`
by `(name, type)` catches exactly that: the orchestrator's declaration is the source
of truth, and the bridge must satisfy it. This is the "contract with one enforcement
point + a test that fails when violated" pattern from the reference-layer work.

### What was ruled out

- **A separate `orchestrator_conduit.py` module / node** — cohesion-by-implementation-
  moment; no existing precedent; a node would duplicate CAN3 ownership (see Design).
- **A real-rclpy contract test** — non-deterministic (DDS discovery), unavailable in
  the venv, and actions hide their sub-services in a `Waitable` (see Design).
- **No home-action goal guard** (matching `can_node`, which had none) — rejected
  because the ReentrantCallbackGroup newly makes two concurrent `_do_home()`s
  *possible*, and their `_write_is_homed` calls could race to persist a stale result;
  the guard is one `bb/throw`-style check.

### The transient-FAULT ESTOP question (hardware, benign by construction)

The parity matrix flagged that a wedged orchestrator publishing `control_mode='ERROR'`
could ESTOP via `motion_bridge_node`. During automated cold-start this is benign:
`FaultHandler.on_enter` sets only `control_mode='ERROR'` (no deactivate request); the
bridge does **not** subscribe `control_mode`; and `motion_bridge`'s estop→motor_guard
is inert while `enable_setpoint_output=false` and `run_mpc.py` is not running. With
Phase 4 the orchestrator no longer *reaches* a transient FAULT on the happy path
(BOOT→HOMING→…→IDLE). Verified read-only at the sitting (see Open Questions).

## Open Questions

- **Powered sitting** — TBD this session: launch `jugglebot_launch.py` and observe the
  orchestrator drive BOOT→encoder-search→home(legs+hand)→level→IDLE **automatically**
  (no manual service calls). Read-only pre-check: a `/control_mode_topic` +
  `/orchestrator_state` rclpy subscriber probe to confirm no unwanted ESTOP on a
  transient FAULT. On PASS, flip the sitting-exercised matrix rows to
  `ported+validated` in a follow-up commit (mirroring Phase 5's two-step).
- **Runtime vel/curr limit persistence across configure** (matrix row 42/25) — the
  ACTIVATE fold calls `_run_configure`, which re-applies YAML-default limits; a prior
  `set_motor_vel_curr_limits` push is reset. Pre-existing `_run_configure` behaviour,
  not introduced here — noted for the eventual limits-persistence follow-up.

## Related

- Plan: [`canbridge-foundation-coldstart-parity.md`](../plans/active/canbridge-foundation-coldstart-parity.md) — Phase 4 (final).
- Parity matrix: `ros_ws/docs/can-node-teensy-parity.md` (rows 6, 7, 24, 27, 28, 44, 45, 54, 60 + the headline).
- Prior phases: [[2026-07-01-canbridge-phase5-hand-conduit]] (hand homing → `_do_home` homes legs + hand; the sitting that surfaced the Phase-4 gap), [[2026-06-29-canbridge-phase2-coldstart-relay-state]] (the `robot_state` fields + `_write_is_homed` this mirrors), [[2026-06-29-canbridge-phase3-version-validated]] (`firmware_validated` un-wedges BOOT), [[2026-06-27-can-node-teensy-parity-audit]] (the headline finding this closes).
