---
title: One evening, three arming failures, one root cause — the ARMING CONTRACT (A1–A5) makes the setpoint-wire lifecycle a single-owner choreography
type: investigation
date: 2026-07-15
status: resolved
phase: "MVP trajectory bringup — arming lifecycle structural fix before the S1/S4 bench sessions"
related_plan: mvp-trajectory-bringup.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md
  - ros_ws/src/jugglebot/jugglebot/state_machine.py
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/src/jugglebot/launch/teensy_bridge_launch.py
  - tests/ros/test_state_machine.py
  - tests/ros/test_orchestrator_node.py
  - tests/ros/test_orchestrator_conduit_integration.py
  - tests/ros/test_trajectory_node.py
  - tests/ros/test_teensy_bridge_node_read.py
  - tests/ros/test_teensy_bridge_node_deactivate.py
  - tests/ros/test_teensy_bridge_node_recover.py
  - tests/hardware/mvp_bench_runbook.md
  - tests/hardware/session_phase1_hold.md
  - tests/hardware/session_phase4_ramp.md
  - tests/hardware/session_phase5_timed.md
  - tests/hardware/session_phase7_reload.md
  - tests/hardware/session_gain_retune.md
  - tests/hardware/session_torque_ff.md
  - tests/hardware/teensy_guard_validation.py
  - tests/ros/test_teensy_bridge_node_setpoint.py
  - controller/teensy_link/setpoint_pump.py
  - ros_ws/docs/can-node-teensy-parity.md
  - plans/active/mvp-trajectory-bringup.md
  - docs/can_bridge/index.md
commits:
  - 5cc8fc9
subsystem:
  - ros
  - motion
  - can
tags:
  - safety
  - choreography
  - contracts
---

# One evening, three arming failures, one root cause

## Summary

The first attempt at `session_torque_ff.md` S1 (2026-07-15 evening) hit **three
distinct arming-lifecycle failures in under two hours**, each with a different
surface signature and the same root cause: *"armed" (`mpc_active=1`) is a
cross-process invariant — it asserts the Teensy owns the legs AND a seeded
producer is streaming — but no single component owned it.* Lifecycle
(orchestrator), streaming (trajectory_node), and the wire (bridge/Teensy) were
three independently-owned booleans whose legal orderings lived in
`mvp_bench_runbook.md`'s "Sharp edges" section instead of in code.

Fix: the **ARMING CONTRACT** (`ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md`,
invariants A1–A5) — the orchestrator owns WHEN (auto-arm on ACTIVE entry,
disarm on real-fault entry), the bridge's existing 5-precondition
stream-then-arm check owns SAFE-TO, disarm-before-stow is enforced in-process
inside `_run_deactivate`, the streaming mode outlives the armed window, the
zero-precondition boot-arm is removed, stale prior-session guard latches are
cleared by a disarmed BOOT pre-flight, and every formerly-silent illegal state
is now loud.

## The three incidents (all logs on disk, `~/.ros/log/2026-07-15-*`)

**Incident 1 — the stale-latch HOMING wedge (19:19 and 18:35, identical).**
The afternoon's Kt bench session left the can-bridge Teensy's guard latched
MPC_STALE (the Teensy is Jetson-5V-powered — latches survive ROS relaunches;
the final disarm heartbeat is consumed by a 10 Hz thread tick and can lose a
teardown race). At launch the bridge reported the latch **7 ms after node-up**;
encoder index search ran fine (SET_AXIS_STATE is not guard-gated) but HOME —
guard-gated, `leg_homing.cpp:97-127` — bounced with ERR_BUS_DOWN → orchestrator
FAULT. The orchestrator's design comment called a boot-time latch "benign …
caught by the ACTIVATE arming pre-check" (`orchestrator_node.py`, FIX 2) — the
firmware's HOME gate refuses *first*. Two components, individually deliberate,
holding contradictory assumptions about the same latch. Meanwhile
trajectory_node's unthrottled seed-rejection ERROR spammed at the 100 Hz
robot_state rate (4,091 lines in 41 s — legs mid-homing sit below the 5 mm
workspace margin) and buried the bridge's every-5 s "CLEAR_ERRORS required"
line. A parallel Claude session correctly analysed the spam mechanics but the
spam was noise: homing failed 300 ms *before* the first spam line, and the
emitter is hard-gated on `_streaming and _seeded` — nothing was ever published.

**Incident 2 — the silent no-op battery (21:26).** Correct launch
(`enable_setpoint_output:=false` — the runbook's documented invocation), clean
recovery, ACTIVE:TRAJECTORY, `traj_ramp_battery.py` ran two z-moves — both
"accepted", `/trajectory/diagnostics` shows the emitter realised the full
64 mm/s profiles — and the robot never moved. The manual arm step
(`set_setpoint_output true`, runbook S2 step 5) had been skipped. Rosbag
`~/Desktop/rosbags/2026-07-15_21-26-12`: `mpc_active=0` the whole bag,
`setpoints_sent=0`, **`setpoints_rejected=0`**, `/leg_setpoint_echo` empty,
`/link_status` level=OK throughout. Doubly silent by construction: a disarmed
bridge holds **no** :5557 subscription (`teensy_bridge_node.py:841-846`), so the
frames died inside trajectory_node's ZMQ PUB, uncounted; trajectory_node read
only `fault_state` off /link_status and had no idea; and
`traj_ramp_battery.py`'s docstring claim that disarmed moves are "rejected
loudly" is simply false for this case.

**Incident 3 — the boot-arm trap + recover deadlock (21:32).** Retry with
`enable_setpoint_output:=true`: `__init__` armed with **zero preconditions**
106 ms before anything could stream (nothing *can* stream at boot — the
workspace gate refuses to seed at STOW) → MPC_STALE latched within one guard
tick (`ever_cmd` was true from the afternoon session, so `age > 250 ms`
instantly). Every leg command refused; the operator's clear attempt, being
armed, rerouted through the converge-first `/recover`, which demanded a
trajectory reseed — impossible at STOW → **"trajectory_node reseed refused"**,
a hard dead end. This is the arming-order trap first recorded at the
2026-07-04 can-hub sitting, now reconfirmed live.

## Diagnosis

The operator proposed gating trajectory_node to stream only in
ACTIVE+TRAJECTORY. Analysis rejected this (with the reasoning, not by
authority): (a) while armed, a continuous stream is *mandatory* in every mode —
MPC_STALE fires 250 ms after the stream stops by design, so the armed
ACTIVE:STANDBY hold between batteries is load-bearing; gating streaming to
TRAJECTORY-only converts every armed mode change into an E-STOP; (b) the arm
pre-check *requires* a fresh :5557 frame, so stream-while-disarmed is a legal,
required phase — the bug is that it was *invisible*, not that it exists;
(c) none of the three incidents was caused by trajectory_node streaming at a
wrong time. The root cause is ownership, not stream-timing.

## Fix — the contract (A1–A5)

See `ARMING_CONTRACT.md` for the normative text and enforcement table. In
brief:

- **A1** `mpc_active` 0→1 only via `_arm_setpoint_output`'s 5 preconditions.
  Boot-arm **removed** (`enable_setpoint_output` param retained but inert +
  loud ERROR — stale launch invocations fail loud, not weird).
- **A2** Orchestrator owns WHEN: ActiveHandler gains an **arm phase** after
  activation (mode published first, `arm_setpoints` request, bounded retries
  over the producer's ~100–300 ms seed window, persistent refusal → FAULT);
  real-fault FAULT entry requests disarm; guard-only faults stay armed (the
  resume path depends on it) and the resume re-verifies the arm (idempotent).
  `auto_arm:=false` launch arg preserves the manual probe-first flow.
- **A3** `_run_deactivate` disarms **in-process, first** — one airtight
  ordering point covering orchestrator deactivate, direct `/deactivate`, and
  shutdown stow. Sharp Edge #6 closed structurally.
- **A4** The streaming mode stays published until deactivate *resolves*
  (ActiveHandler.on_exit no longer blanks; IdleHandler blanks after the op) —
  the 250 ms mode-blank race is removed, not merely won. Sharp Edge #1's
  deferred auto-disarm item landed via A3+A4.
- **A5** Loud everywhere: trajectory_node mirrors `mpc_active` off /link_status
  and WARNs + tags accept responses `[wire DISARMED …]`; seeding gated on
  `is_homed` (kills the 100 Hz spam class); "streaming ENABLED — awaiting
  seed" disambiguated; the bridge surfaces the firmware's arm-took bit
  (HeartbeatT2J bit3) as `teensy_mpc_active`; BOOT gains a **disarmed
  stale-latch pre-flight** (one-shot clear + loud; a returning latch → FAULT;
  waits for the first /link_status so the default `guard_latched=False` is
  never trusted blind).
- **FaultHandler origin fix**: guard-only classification now requires
  `prev_state == ACTIVE` (`StateMachine` records `ctx.prev_state` on every
  transition). Before this, a stale latch surfacing in a HOMING fault
  classified guard-only and, on clear, exited "back" to ACTIVE:STANDBY on a
  robot that never activated (found by code-trace during tonight's
  investigation, latent since F1).
- **Armed clear fallback (one canonical path)**: `_svc_odrive_command`'s
  `clear_errors` — the conduit the orchestrator's command actually takes —
  routes through `_svc_clear_errors`, so both share the armed converge-first
  reroute AND the fallback: when `/recover` cannot reseed/converge, **disarm,
  wait for the wire to confirm the disarm (T2J bit3), then clear directly**
  instead of the old hard refusal. This deliberately supersedes the 2026-07-11
  "no raw armed escape hatch" decision *by its own root cause*: that refusal
  existed to prevent a clear-onto-diverged-u0 jolt, and a clear that lands only
  after the confirmed disarm (guard terms inert, output gate off at
  `mpc_active=0`) cannot jolt — an unconfirmed disarm refuses the clear rather
  than racing it. The wire is left disarmed; re-arming must re-pass A1.

## The adversarial review round (before landing)

A 4-lens × adversarial-verify workflow (choreography races / hardware safety /
Foxy+mock divergence / contract completeness; 27 agents) reviewed the diff
before commit: 23 raw findings, 21 confirmed. The important ones, all fixed in
the same change:

- **BLOCKING — the A3 disarm never reached the wire before the DEACTIVATE
  RPC.** `set_heartbeat_flags(0)` only *stages* the disarm for the next 10 Hz
  heartbeat tick, so the immediately-fired RPC raced `s_mpc_active=1` on the
  Teensy and would be rejected almost every time — the same
  staged-flag-vs-wire race class as the bench harness's teardown-disarm race.
  Fix: `_wait_wire_disarmed()` — poll the firmware's arm-took bit (T2J bit3,
  freshly surfaced by this very change) until the disarm confirms; used by
  both `_run_deactivate` and the clear fallback (which now *refuses* the
  direct clear if the disarm does not confirm — the no-jolt property depends
  on it).
- **HIGH — the fallback lived on the wrong service.** The orchestrator's
  `clear_errors` rides the `odrive_command` conduit, which still had the old
  hard-refusal armed reroute — the production command path could dead-end
  forever while the Trigger service had the fallback. Fix: one canonical path
  (`_svc_odrive_command` routes `clear_errors` through `_svc_clear_errors`).
- **HIGH — the FaultHandler disarm orphaned the in-flight deactivate.** The
  natural ACTIVE→FAULT path dispatches the multi-second deactivate one tick
  before the disarm; the single-slot `_pending_future` overwrite opened
  IdleHandler's wait-for-deactivate gate while the platform was still
  physically descending (and silently swallowed a failed descent). Fix:
  the disarm dispatches fire-and-forget, outside the operation tracking.
- **HIGH — guard-only→real promotion starved an armed wire.** The promotion
  publishes 'ERROR' (stopping the producer) but `on_enter` cannot re-run to
  issue the disarm. Fix: the promotion branch requests it.
- **MEDIUM — `prev_state==ACTIVE` was a proxy.** A latch-caused ACTIVATE
  refusal (FAULT raised *by* ActiveHandler on an unarmed robot) classified
  guard-only and would "resume" a robot that never activated. Fix: guard-only
  additionally requires the `wire_armed` mirror (fail-safe: stale/absent →
  the heavier real-fault path).
- Plus: arm refused while a deactivate is in progress (an early-descent u0 can
  pass the 0.25 rev tolerance); the arm service moved to the reentrant
  callback group (its 0.5 s stream-wait was starving the 100 Hz telemetry the
  producer needs to seed); `go_home`/`hold` accepts now carry the wire-state
  suffix too; retry-budget arithmetic corrected (each refusal ≈ 0.6–1.0 s, not
  one tick — budget set to 10 ≈ 6–10 s); the contract doc's launch-arg name
  fixed (`auto_arm:=false`); four hardware session docs bannered.

Notable rejections (verified not-real): the "two conflicting TRAP_TRAJ
streams" escalation (firmware busy-rejects ACTIVATE during a descent —
`leg_activate.cpp` `deactivate_active()` gate), and the BOOT pre-flight
"jam-latch auto-clear" concern (one-shot + FAULT-on-return already covers it;
a motion-triggered relatch fails the subsequent HOMING loudly).

A subsequent `/audit --unstaged` round (audit-reporter) caught what the review
missed: the arm service's move to the Reentrant group had silently removed its
mutual exclusion (two overlapping arms could double-start the ingest thread,
and a deactivate starting during the 0.5 s frame-wait could evade precondition
(a3)) — fixed with `_arm_lock` (non-blocking; the loser is refused loudly) and
a TOCTOU re-check of `_deactivate_in_progress` immediately before
`_start_setpoint_output`; `on_shutdown`'s fixed 0.2 s disarm settle upgraded
to the same `_wait_wire_disarmed()` bit3 confirm as the other two paths; the
pre-contract "benign latch is caught by the ACTIVATE arming pre-check"
comments (the exact contradictory-assumption documentation Incident 1
indicts) rewritten in orchestrator_node and the bridge's module docstring;
`session_phase1_hold.md`'s Step 0 corrected to `auto_arm:=false` (verbatim
following would have auto-armed before the probe step, defeating the
protocol's premise); supersession banners added to `session_phase4_ramp` /
`session_gain_retune` / `session_torque_ff` (the latter noting the FF ramp now
starts at the automatic arm — or use `auto_arm:=false` for a deliberate arm
edge, recommended for the first FF arming).

## Verification

- **Final full suite, post-review + post-audit fixes** (`pytest tests/ -q`,
  run 2026-07-15): **2803 passed, 1 xfailed in 584.04 s**. (Pre-review
  baseline, same command, same date: 2795 passed, 1 xfailed in 604.39 s. An
  intermediate run showed 1 failure in
  `test_t3b_h4_on_post_solve_allocates_within_budget` under review-agent CPU
  load — the documented order/load-flaky allocation test; passed isolated,
  `pytest tests/sim/test_mpc_time_pathologies.py::TestT3bH4PostSolveAllocation -q`
  2026-07-15: 1 passed in 7.44 s — and passed in the final full run above.)
  Includes 40+ new/updated contract tests: BOOT pre-flight
  (clear/one-shot/timeout/failure), ActiveHandler arm phase
  (retry/budget-FAULT/manual-skip/command-queueing/resume-verify), A3
  disarm-before-DEACTIVATE RPC ordering (asserts `mpc_active` already 0 when
  the RPC reaches the FakeTeensy), A4 deferred mode blank, FaultHandler origin
  + disarm, boot-arm inert, `teensy_mpc_active` KeyValue, trajectory seed
  gate + loud-disarmed accepts, and the disarm-then-clear fallback.
- The production-ordering conduit test (`test_orchestrator_conduit_integration`)
  now routes the bridge's REAL `/link_status` publish into the orchestrator
  each tick — the A5 decode and BOOT pre-flight run against the real KeyValues
  end-to-end (the mocked-ROS choreography-blindness lesson applied).
- Firmware untouched (no reflash needed; bit3 was already on the wire).
- Hardware validation pending: the next launch should show BOOT clearing the
  currently-latched guard (left by tonight's 21:32 boot-arm), then
  activate → the arm banner ("mpc_active set to 1 — setpoint output ENABLED")
  with **zero motion at the arm edge**, then the S1/S4 batteries actually
  moving.

## Discussion

**Why a contract and not three patches.** Each incident had a one-line fix
(clear the latch by hand; run the arm service; don't use `:=true`). The class —
"an arming-lifecycle ordering that only the runbook enforces" — had already
produced Sharp Edges #1, #4, #5, #6, the 2026-07-09 false-success arming, the
2026-07-04 arm-before-stream trap, and tonight's three. Climbing one level
(CLAUDE.md's engineering philosophy) turns six documented operator burdens
into three code-enforced invariants with tests. The runbook keeps the history;
the code now owns the ordering.

**Why the operator's proposed gate was rejected.** "Stream only in
ACTIVE+TRAJECTORY" is the intuitive fix and would have made things worse: the
armed STANDBY hold is what keeps the staleness watchdog fed between batteries.
The analysis that killed it — the watchdog *requires* the stream that the gate
would remove — is the same analysis that produced A4 (keep the mode published
through deactivate). The half of the intuition that was right (lifecycle
should gate *something*) landed as the `is_homed` seed gate.

**Why auto-arm rather than loud-manual.** Operator decision (2026-07-15, via
explicit option choice): "activate means live." The deliberate cost is losing
the explicit go-live checkpoint during bringup; the mitigations are that the
arm still runs the full A1 pre-check (arming cannot succeed into a sick
stream), FAULTs loudly on persistent refusal, and `auto_arm:=false` restores
the old flow for probe-first sessions — where the disarmed wire is now loud
instead of silent.

**Why disarm-inside-deactivate instead of an orchestrator disarm request.**
Two service calls dispatched in one 10 Hz tick (`disarm` then `deactivate`)
execute on the bridge's MultiThreadedExecutor with no ordering guarantee, and
the orchestrator's single `_pending_future` slot can only track one. In-process
sequencing at the head of `_run_deactivate` is airtight by construction and
covers the two non-orchestrator entry points (direct service, shutdown stow)
for free.

**On superseding the 2026-07-11 armed-clear refusal.** The refusal's root
cause was jolt prevention, not latch preservation. Disarm-first satisfies the
root cause while removing the dead end that cost tonight's 21:32 session. The
old test (`test_armed_bare_clear_errors_refuses_on_diverged_command`) was
rewritten to pin the new contract, including the no-jolt reasoning, rather than
deleted.

**The parallel-session analysis.** A second Claude session (without this
project's context) analysed incident 1's spam correctly at the mechanism level
— every line-number claim verified — but concluded at the symptom: it
explained the spam, explicitly noted the emitter never publishes, and did not
reach the guard latch that actually wedged homing. Its one-line throttle fix
(`throttle_duration_sec=1.0` on the seed-rejection ERROR) was found uncommitted
in the working tree, was exactly right, and is folded into this change.

## Related

- `ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md` — the normative document.
- Rosbag evidence: `~/Desktop/rosbags/2026-07-15_21-26-12` (the silent no-op).
- `logbook/2026-07-02-canhub-hardening-tier2.md` — first record of the
  arm-before-stream trap.
- `tests/hardware/mvp_bench_runbook.md` — Sharp Edges #1/#6 now marked
  structurally closed; S1/S2 manual arm steps superseded.
- Known residual (operator's file, not touched): `tests/hardware/
  traj_ramp_battery.py`'s docstring still claims disarmed moves are "rejected
  loudly" — false; the accept-response now carries the wire state, so the
  script prints it, but the docstring should be corrected when that file's
  working-tree edits land.
