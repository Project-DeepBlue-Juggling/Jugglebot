# The Arming Contract (A1–A5)

**Status: NORMATIVE.** The code enforces these invariants; changing the behaviour
means changing this document first, then rippling the change through the
enforcement points and the tests. Pattern per
`controller/REFERENCE_LAYER_CONTRACT.md` (K1–K6).

## The problem this closes

"Armed" (`mpc_active=1`) is a **cross-process invariant**: it asserts that the
Teensy owns the legs and that a seeded producer is streaming fresh setpoints at
40 Hz. Before 2026-07-15 that invariant had **no owner** — lifecycle
(orchestrator), streaming (trajectory_node), and the wire (bridge/Teensy) were
three independently-owned booleans whose legal orderings lived in a runbook's
"Sharp edges" section instead of in code. One evening of first-contact bench
work (2026-07-15) hit three distinct doors into that hole:

1. **The silent no-op**: launch disarmed (the documented default), operator
   skipped the manual arm step → trajectory_node accepted and fully executed a
   move battery while `setpoints_sent` stayed 0 and `/link_status` read `OK`.
   No component could even see the mismatch (a disarmed bridge holds no :5557
   subscription).
2. **The boot-arm trap**: `enable_setpoint_output:=true` armed at `__init__`
   with **zero preconditions**, before any producer could stream (the workspace
   gate refuses to seed at STOW) → MPC_STALE latched within one guard tick →
   every leg command refused.
3. **The stale-latch wedge**: a guard latch left by an earlier bench session
   (the can-bridge Teensy is Jetson-5V-powered — latches survive ROS relaunches)
   wedged HOMING at the first guard-gated verb, while the orchestrator's design
   comment declared that latch "benign".

## Invariants

- **A1 — stream-before-arm.** `mpc_active` goes 0→1 **only** through
  `teensy_bridge_node._arm_setpoint_output`'s preconditions: fresh Teensy
  heartbeat + link not latched, guard fault NOT latched, a fresh mpccmd frame on
  :5557 within 0.5 s, that frame pump-acceptable, and its `u0` within
  0.25 rev of every live encoder. **There is no other arming path.** The
  zero-precondition boot-arm (`enable_setpoint_output:=true` →
  `_start_setpoint_output` at `__init__`) is removed; the parameter is retained
  but inert (loud ERROR if set).
- **A2 — the orchestrator owns WHEN.** In the production stack the orchestrator
  is the sole caller of `/set_setpoint_output`: it **arms** in ActiveHandler's
  arm phase (after the ACTIVATE move completes and the streaming mode is
  published, with bounded retries while the producer seeds — each refused
  attempt includes the bridge's 0.5 s stream-wait, so the 10-attempt budget is
  roughly 6–10 s wall-clock; persistent failure → FAULT), and **disarms** on
  real-fault FAULT entry *and* on the guard-only→real mid-FAULT promotion. The
  disarm is dispatched fire-and-forget, outside the single-slot operation
  tracking, so it can never orphan an in-flight tracked op (the multi-second
  deactivate whose completion gates IdleHandler). A guard-only fault (latched
  during ACTIVE **while armed** — classification requires `prev_state==ACTIVE`
  AND the `wire_armed` mirror) deliberately does NOT disarm — the resume path
  depends on the stream surviving. Manual arming remains possible for bench
  sessions via the launch arg `auto_arm:=false` (orchestrator parameter
  `auto_arm_setpoint_output`; the pre-contract probe-first flow), and is loud
  rather than silent when forgotten (A5).
- **A3 — disarm-before-stow.** `_run_deactivate` disarms **in-process, first**,
  then **waits for the firmware's arm-took bit (T2J bit3) to confirm the
  disarm landed on the wire** before firing DEACTIVATE — `set_heartbeat_flags(0)`
  only *stages* the disarm for the next 10 Hz heartbeat tick, so an immediate
  RPC would race `s_mpc_active=1` on the Teensy and be rejected. This one place
  makes the ordering airtight for every entry point: the orchestrator's
  deactivate, the direct `/deactivate` service, and the shutdown stow. The
  firmware's reject-DEACTIVATE-while-armed gate remains as the backstop, not
  the mechanism. The arm pre-check symmetrically refuses while a deactivate is
  in progress (an arm must never land mid-descent).
- **A4 — the stream outlives the armed window.** `control_mode` stays in the
  streaming set until the deactivate **completes**: ActiveHandler.on_exit no
  longer blanks it; IdleHandler blanks it only after the pending operation
  resolves. This removes the 250 ms race (blank → emitter stops → MPC_STALE
  beats the disarm) instead of merely winning it. A guard-only FAULT preserves
  the mode for the same reason (pre-existing).
- **A5 — illegal states are loud.** trajectory_node reads `mpc_active` from
  `/link_status` and (a) WARNs when a motion command is accepted while the wire
  is disarmed, (b) appends the wire state to the accept response so harnesses
  print it. The bridge surfaces the firmware's own arm-took flag
  (HeartbeatT2J bit3) as `teensy_mpc_active` on `/link_status`. A stale
  prior-session guard latch is cleared by the orchestrator's BOOT pre-flight
  (disarmed → the direct clear path; nothing is armed, no output path exists,
  so the clear cannot jolt) instead of wedging the first guard-gated verb of
  HOMING. Seeding is gated on `is_homed` so mid-homing telemetry no longer
  produces workspace-gate ERROR spam that buries real errors.

## Enforcement points

| Invariant | Code | Test |
|---|---|---|
| A1 | `teensy_bridge_node.py` `_arm_setpoint_output` (sole 0→1 path); `__init__` boot-arm removed | `tests/ros/test_teensy_bridge_node_read.py` (boot-arm inert) |
| A2 | `state_machine.py` `ActiveHandler` arm phase; `FaultHandler.on_enter` disarm; `orchestrator_node.py` `arm_setpoints`/`disarm_setpoints` dispatch | `tests/ros/test_state_machine.py` arm-phase / retry / FAULT cases |
| A3 | `teensy_bridge_node.py` `_run_deactivate` head | disarm-before-DEACTIVATE order test |
| A4 | `state_machine.py` `ActiveHandler.on_exit` (no blank) + `IdleHandler` deferred blank | mode-survives-deactivate ordering test |
| A5 | `trajectory_node.py` wire-state WARN + response suffix + `is_homed` seed gate; `teensy_bridge_node.py` `teensy_mpc_active` KeyValue; `state_machine.py` `BootHandler` latch pre-flight | disarmed-accept, seed-gate, pre-flight tests |

## The choreography (one cycle, walked at the control-system level)

1. Operator sends `activate`. ActiveHandler requests the profiled ACTIVATE
   (TRAP_TRAJ to the active pose — `mpc_active=0`, the stream is irrelevant to a
   profiled move).
2. ACTIVATE completes → ActiveHandler publishes the streaming mode (STANDBY).
   trajectory_node seeds a hold **from measured telemetry** at the active pose
   (`u0` ≈ encoder) and streams at 40 Hz.
3. Next tick(s): ActiveHandler requests `arm_setpoints`. The bridge pre-check
   (A1) confirms the stream it is about to trust; on pass it resets the pump
   step-gate and raises `mpc_active`. The Teensy's first applied setpoint
   commands the pose the ODrives are already holding — **no discontinuity by
   construction** (u0 == measured within 0.25 rev, vel_ff = 0 on a hold,
   torque_ff per its own AND-gate). Retries cover the ~100–300 ms seed window;
   ~6–10 s of persistent refusals (10 attempts × the 0.5 s stream-wait +
   round-trips) → FAULT, loudly. The arm service lives in the bridge's
   reentrant callback group so its stream-wait never starves the 100 Hz
   telemetry timers the producer needs to seed.
4. Mode changes within ACTIVE (STANDBY↔TRAJECTORY↔…) are streaming→streaming:
   the armed wire is never starved (this is why "stream only in TRAJECTORY
   mode" was rejected as a design — the armed STANDBY hold is load-bearing).
5. `deactivate` → ActiveHandler.on_exit requests the deactivate;
   **the streaming mode stays published** (A4). The bridge disarms in-process
   (A3) — from that instant the Teensy ignores the still-flowing stream and its
   staleness watchdog is inert (`s_mpc_active=0` gates it) — then runs the
   parallel no-tilt TRAP_TRAJ descent. IdleHandler blanks the mode only after
   the operation resolves; the emitter then stops, long since disarmed.
6. Real fault at any point → FAULT entry disarms (A2). Guard-only fault from
   ACTIVE → stays armed, guard suppresses output, resume re-verifies the arm
   through A1 (idempotent if still armed).

## What this deliberately does NOT change

- The **MPC_STALE watchdog** (250 ms, firmware) is untouched — it remains the
  backstop against a dead Jetson while armed. The contract's job is to make the
  legal orderings structural so the watchdog only ever fires on real faults.
- The **guard latch semantics** (latch-until-CLEAR_ERRORS) are untouched. The
  BOOT pre-flight clears exactly one class — a latch carried into a fresh
  disarmed boot — and does it once; if the latch returns, that is a real fault
  and the machine goes to FAULT.
- The bench harnesses (`bench_leg_sysid.py`, `kt_bench_test.py`,
  `teensy_setpoint_bench.py`) own their own UDP link and arming lifecycle;
  they are out of scope (and already implement stream-then-arm + startup
  latch clearing on their side).

## Provenance

Logbook: `logbook/2026-07-15-arming-contract.md` (the three incidents, the
rosbag evidence, the design discussion). Prior art: the runbook's Sharp Edge #1
already named the auto-disarm as a deferred structural item;
`tests/hardware/teensy_guard_validation.py` validated stream-then-arm on
hardware 2026-07-09.
