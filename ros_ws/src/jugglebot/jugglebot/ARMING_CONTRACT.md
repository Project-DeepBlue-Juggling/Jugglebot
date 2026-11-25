# The Arming Contract (A1–A7)

**Status: NORMATIVE.** The code enforces these invariants; changing the behaviour
means changing this document first, then rippling the change through the
enforcement points and the tests. Pattern per the K1–K6
`REFERENCE_LAYER_CONTRACT.md` (retired 2026-10-14; the pattern lives on in
`motion/skills/INVARIANTS.md`).

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
   gate refuses to seed at STOW) → SETPOINT_STALE latched within one guard tick →
   every leg command refused.
3. **The stale-latch wedge**: a guard latch left by an earlier bench session
   (the can-bridge Teensy is Jetson-5V-powered — latches survive ROS relaunches)
   wedged HOMING at the first guard-gated verb, while the orchestrator's design
   comment declared that latch "benign".

## Invariants

- **A1 — stream-before-arm.** `mpc_active` goes 0→1 **only** through
  `teensy_bridge_node._arm_setpoint_output`'s preconditions: fresh Teensy
  heartbeat + link not latched, guard fault NOT latched, a fresh setpoint frame on
  :5557 within 0.5 s, that frame pump-acceptable, its `u0` within
  0.25 rev of every live encoder, no deactivate descent in flight, and no
  guard-recovery hand park in flight (`_recovery_park_in_progress`, 2026-09-23:
  the park's ACTIVATE is rejected by the firmware under an armed stream, so the
  recovery disarms to park and publishes `fault_state=RECOVERING` on
  `/link_status` until the hand is home — the orchestrator stays in FAULT and
  re-arms through A2 on the NONE edge). **There is no other arming path.** The
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
  streaming set until the deactivate **completes**: the mode stays published
  because the machine is still ACTIVE through the descent (ActiveHandler leaves
  ACTIVE only on the bridge's confirmation, 2026-10-23); IdleHandler blanks it
  on entry, after any operation still in flight has resolved. This removes
  the 250 ms race (blank → emitter stops → SETPOINT_STALE beats the disarm)
  instead of merely winning it. A guard-only FAULT preserves
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
  While Armed the bridge also refuses the RPCs that re-zero, re-tune, re-mode
  or reboot a live loop (`/configure`, `set_hand_gains`, `set_hand_state` to
  anything but IDLE, `reboot_odrives`), in the firmware and with a message on
  the host (FW 52, `INVARIANTS.md` I-FW-18): disarm first.
- **A6 — Homing starts only from a Platform that is down** (owner,
  2026-11-25). Homing drives the Legs one at a time to their bottom stops
  (`leg_homing.cpp`, `_run_home` is sequential). If it starts while the
  Platform is up at the Active pose or a levelling pose, the Legs not yet homed
  hold their lengths while the one being homed retracts, and the Platform can
  collapse. So the machine enters HOMING only when **all six Legs report ODrive
  IDLE, none Closed-loop or in any other state, and have done so continuously
  for `HOMING_LEGS_IDLE_SETTLE_S` (2 s)**. This is the evidence that the
  Platform is down: an Idle Leg is unpowered and free to move, so six Idle Legs
  cannot hold the Platform up. It is at Stow, either lowered there by a
  confirmed DEACTIVATE (which IDLEs each Leg on arrival) or dropped there under
  gravity by the drive power loss of the E-stop button. The ACTIVATE and DEACTIVATE aborts do
  NOT make a third way to six Idle Legs: from FW 53 no abort IDLEs a Leg above
  Stow (A7, `INVARIANTS.md` I-FW-19); they hold the Legs, freeze them at their
  stopping points, or return the Platform to Stow first, so six Idle Legs still
  mean a Platform that is down. The firmware twin of this gate is
  `homing_request` in `leg_homing.cpp`: a Leg HOME is refused (ERR_REJECTED)
  unless every present Leg reads IDLE on a fresh heartbeat that is confirmed
  after the bridge's last Closed-loop send to it (`all_present_legs_idle`;
  ENCODER_INDEX_SEARCH and an at-Stow Closed-loop Leg both refuse; the one-Leg
  bench rig is exempt; HOME of the Hand is not gated); the firmware has no 2 s settle, only the confirmed
  fresh heartbeat. A Leg
  CLOSED_LOOP and a Leg SDO write are also refused while a Homing runs. The settle lets a drop finish. Leg positions are not
  used: they mean nothing once a reference is lost, which is exactly when
  Homing is needed. Unknown Leg states (no /robot_state yet, or one too short
  to carry every Leg) refuse: Homing never starts on a guess.
  **Freshness:** the Leg states count as known only while the `/robot_state`
  that set them is at most `LEG_STATES_MAX_AGE_S` (1 s) old; an older (or never
  stamped) snapshot is "not known" and refuses, with a message that says it is
  stale and how old. The bridge publishes `/robot_state` at the 100 Hz timer
  rate and, when telemetry freezes, skips at most 50 ticks (~0.5 s) before it
  publishes anyway (`_ROBOT_STATE_MAX_CONSEC_SKIPS`), so a live bridge never
  trips the bound. A gap longer than the bound also RESTARTS the 2 s settle
  (`orchestrator_node._on_robot_state` clears `legs_idle_since` before the first
  message after the gap): a Leg may have been re-energised and idled again in
  the gap, so the old all-Idle run proves nothing about the Platform now. Tests:
  `tests/ros/test_state_machine.py::TestHomingGate::test_predicate_stale_states_are_unknown`
  and `::test_stale_states_hold_boot_and_notify_once`;
  `tests/ros/test_orchestrator_node.py::TestHomingGateThroughTick::test_gap_in_robot_state_restarts_the_settle_and_stale_refuses`.
  **One enforcement point:** `StateMachine` checks `homing_refusal(ctx)` on
  every transition into HOMING, whether a handler returned it (BOOT not homed,
  IDLE `home`) or it was forced. On a refusal the machine stays where it was
  (BOOT or IDLE) and raises one operator notice per refusal episode, naming the
  reason and the remedy. The remedies, by case:
  - **All six Legs Closed-loop** (e.g. a relaunch under a Platform a previous
    session left posed, or a false IDLE): the profiled Stow, the bridge's
    `/deactivate` service. Homing then starts by itself (from BOOT) once the
    Legs are Idle.
  - **Some Legs Closed-loop, some not** (one Leg dropped out by itself during
    ACTIVE or LEVELLING, and the firmware's Soft E-stop holds the other five at
    their setpoints): a software stow only when exactly one Leg is not
    Closed-loop, and it reads Idle and provably kept its position reference
    (owner, 2026-11-25; FW 53 U6b, R3). Two or more dropped Legs are refused with
    a notice naming the count ("recovery is offered for one dropped Leg only")
    and the E-stop-button remedy below.
    `FaultHandler._mixed_recovery` decides with `leg_reference.reference_unproven`
    (homed flag kept, a Closed-loop baseline length seen, no `/robot_state` gap
    or odd drive state, a disarm reason seen and held until the operator's
    `clear_errors`, only error bits known to keep the estimate, and no more than
    `REF_DRIFT_TOL_REV` from the baseline), and after that `clear_errors` sends
    the profiled DEACTIVATE(AXIS_ALL): the firmware re-seeds the dropped Leg at
    its measured length and re-engages it there, then lowers all six together
    (`tests/firmware/native/test_leg_deactivate.cpp` "R3 mixed case"); once per
    FAULT visit. Otherwise no software stow: a Leg whose reference may be lost
    would be driven to "Stow" in a frame that may be turns off, and single-Leg
    DEACTIVATEs run one at a time and diverge the lengths exactly as Homing
    would. FAULT says why the reference is not proven, and the operator supports
    the Platform and presses the E-stop button (the drives lose power and the
    Platform comes down), then sends `clear_errors` once power is back. That
    fallback brings the Platform down by a drop, not a lowering, and needs a
    person at the Platform.
  - **A Leg in any other state** (a drive still starting, or calibrating):
    wait, or power-cycle it with the E-stop button, then `clear_errors`.
  Every path that reaches a Leg HOME or ENCODER_SEARCH without the StateMachine
  is gated twice. The bridge's `/home` and `/encoder_search` services typed by an
  operator (`ros2 service call`; the production orchestrator uses the home action
  and its own encoder-search client, both behind the StateMachine) refuse unless
  every other present Leg reads Idle (A7, no settle). The firmware's
  `homing_request` (above) refuses the same HOME whatever its sender, so the bench
  drivers that send the RPC straight over UDP
  (`tests/hardware/teensy_home_bench.py`, `tests/hardware/bench_leg_sysid.py`,
  `tests/hardware/teensy_encoder_search_bench.py`, through the
  `teensy_link/homing.py` and `encoder_search.py` libraries) and the instruction
  text of `tools/probes/canhub_tier2_hw_validation.py` (it prints
  `ros2 service call /home`) get ERR_REJECTED on a robot with a posed Platform
  and pass on the one-Leg bench. Checked by grep 2026-11-25: `RpcMethod.HOME`
  outside tests/ros and tests/teensy_link is sent by exactly those two bench
  drivers plus the bridge. Not callers, despite appearing in an earlier audit:
  `tests/hardware/bench_hand_frame_bench.py` (drives the Hand's ODrive directly),
  `tests/hardware/kincal_capture.py` (prints the operator steps; its `home` is
  the gated orchestrator command) and `tools/probes/gui_synthetic_stack.py`
  (registers a fake recording `/home` service). The firmware half is not
  verified on hardware. Not gated by A6: ENCODER_SEARCH reaching the firmware over
  UDP (the Leg SET_AXIS_STATE it sends is a de-energise decided by A7).
  **The `/deactivate` remedy works where the refusal is shown.** Read
  2026-11-25: `_svc_deactivate` -> `_run_deactivate` has no `is_homed` or fault
  term, and the firmware's `deactivate_request` (`leg_deactivate.cpp`) refuses
  only for a dead CAN bus, the guard in ESTOP (a latched Fault, which BOOT
  clears before it ever asks for HOMING), a homing/activate move or a pending
  fault stow, an armed setpoint stream (the bridge disarms first, A3), no
  present target Leg, or a deactivate already running. None depends on the
  homed flag, so it is accepted with `is_homed` false (a relaunch under a posed
  Platform, a Hand-only reference loss). If it IS refused (ERR_BUS_DOWN from a
  latched guard), `clear_errors` first; the E-stop button stays the fallback.
  Test: `tests/ros/test_teensy_bridge_node_deactivate.py::test_deactivate_is_not_gated_on_the_homed_flag`.
- **A7 — A Leg is made Idle only from a Platform at Stow, and IDLE means at
  Stow** (owner, 2026-11-25: "A Leg should never be made IDLE unless the
  Platform is already at the Stow position. Once the Platform is at Stow, then
  any/all Legs can freely enter IDLE." The Hand is exempt.) A6 is its
  companion for Homing. The host enforces it at two points that share one
  classifier, `state_machine.leg_idle_blockers` (the Legs not reading Idle,
  an unknown Leg counting as not Idle):
  - **The orchestrator's IDLE state means "Platform at Stow, Legs Idle".**
    BOOT enters IDLE (the homed skip) only when `idle_refusal` passes: all six
    Legs known, fresh (A6's `LEG_STATES_MAX_AGE_S`) and Idle, with no settle.
    Otherwise BOOT holds and raises one notice per episode: all six
    Closed-loop (a relaunch under a posed Platform) names the profiled Stow,
    `/deactivate`; a mixed case (one Leg dropped, five holding, homed flag
    kept) gets A6's mixed-case text (the profiled recovery when exactly one Leg
    dropped and its reference is proven kept, else the E-stop button); unknown or other states say to wait. BOOT then enters IDLE
    by itself once the Legs are Idle. ACTIVE's legs-out exits (the Legs out of
    Closed-loop for `LEGS_OUT_TICKS`, and the deactivate reconcile window) go
    to IDLE only when the same predicate passes, otherwise to BOOT, whose gate
    then holds. The other IDLE entries are already confirmed Idle by the
    bridge: a confirmed DEACTIVATE (ACTIVE) and `level_deactivate` (LEVELLING)
    succeed only after the DeactivateMonitor has seen every Leg IDLE at Stow;
    HOMING reaches IDLE after each Leg was IDLEd at its stop by the firmware,
    behind A6; FAULT never enters IDLE directly (it exits to BOOT, or to
    ACTIVE when guard-only).
  - **The bridge's operator services refuse a Leg unless the bridge's own
    fresh Leg states show every present Leg Idle:** `/reboot_odrives` and
    `odrive_command reboot_odrives` (both through `_reboot_odrives`; a reboot
    IDLEs every drive), `/encoder_search` when `encoder_search_axes` names a
    Leg, and `_do_home` (the manual `/home` and the orchestrator's
    `home_motors`; a Home of ONE Leg needs only every other present Leg Idle).
    The inputs are `TeensyBridgeNode._present_leg_states`, read from the
    DIAGNOSTIC cache: a Leg is present once its drive has heartbeated this
    can-bridge boot (flags `HB_SEEN`, so a one-Leg bench rig consults one Leg),
    and a present Leg is unknown, and refuses, when its DIAGNOSTIC is missing,
    older than 3 s (they arrive on change and at a forced 1 Hz) or flagged
    `HB_STALE`. The orchestrator's Homing passes, behind A6. Hand-only
    requests are exempt.
  - **The firmware enforces the same rule for every sender** (FW 53,
    `INVARIANTS.md` I-FW-19; the host classifiers above are the early, readable
    refusal, the firmware is the authority). `leg_stow_gate.cpp` is the only
    translation unit that encodes a Leg Set_Axis_State, reboot or SDO write; it
    sends one only when `rig_is_bench` or `platform_at_stow` passes. The Platform is
    at Stow when every present Leg has a fresh (0.5 s), Closed-loop-confirmed
    heartbeat reporting IDLE or ENCODER_INDEX_SEARCH, or Closed-loop with no
    error, a fresh encoder sample (150 ms), position in [-0.11, +0.01] rev and
    speed at most 0.1 rev/s. The Hand is exempt; so is the E-stop button's power
    cut (physical), the homing task's own Leg, and the one-Leg bench (exactly
    one drive heard, no Platform Teensy frame, 5 s settled, sticky toward robot;
    every doubt reads robot). Refusals are ERR_REJECTED with nothing on CAN and
    the host reads them as "Platform not at Stow". Covered: the ACTIVATE and
    DEACTIVATE aborts (hold, freeze, or return to Stow under the gentle profile
    then IDLE; a guard trip during a Stow descent lets it finish, a Soft E-stop
    or a timeout during a rise returns the Platform to Stow; a Leg that drops out
    mid-rise freezes the others, and a Leg counts as dropped out as soon as its
    drive reports an error or a disarm reason, even before its heartbeat leaves
    Closed-loop), the deferred fault stow (IDLE only once the encoders
    confirm Stow, within 10 s, else freeze at the measured position and HELD), a
    single-Leg DEACTIVATE, and
    the RPC passthroughs (SET_AXIS_STATE to anything but CLOSED_LOOP on a Leg,
    REBOOT_ODRIVES, a Leg SDO_WRITE). A Leg SET_AXIS_STATE IDLE still passes the
    Armed gate (I-FW-18) and is decided here. A Leg the firmware refused to IDLE is
    HELD (`/link_status` `legs_held`; `rig_bench` shows the rig reading);
    CLEAR_ERRORS(AXIS_ALL), or the next accepted ACTIVATE, DEACTIVATE or HOME
    naming the Leg, clears it.
  - **What the operator does, by case.** Posed Platform, Legs holding:
    `/deactivate` (the profiled Stow). One Leg dropped out by itself, five
    holding: FAULT, then `clear_errors`; the profiled recovery runs only if the
    dropped Leg's reference is proven kept (A6), else support the Platform and
    press the E-stop button. Two or more Legs dropped: always the E-stop button. A rise that returned to Stow: FAULT, then BOOT and
    IDLE; retry the activate. A HELD Leg after a refused abort: `/deactivate`
    again (or the E-stop button with the Platform supported). A manual
    `/reboot_odrives`, `/encoder_search` or Leg IDLE refused with "Platform not at
    Stow": `/deactivate` first. Not covered: a Leg IDLE the firmware is not asked
    to send (a drive that disarms itself on an error, which is the mixed case).
  - **Not verified on hardware (FW 53 is built, not flashed until the
    orchestrator says so):** the mid-move re-target of a Leg when an ACTIVATE is
    returned to Stow (continuity of the setpoint, the apex of about 0.10 rev above
    the abort position), the first post-Closed-loop heartbeat timing the
    confirmation depends on, the bench-versus-robot inference on the real bus, and
    the re-engage of a dropped Leg under load. Both ladders send no motion
    target to any Leg until every target Leg has confirmed Closed-loop by a
    heartbeat received at least 10 ms after its own Closed-loop command. A Leg
    still reporting Idle with no error is waited for. A Leg with an error, or
    no confirmation within 400 ms of the last Closed-loop command, fails the
    ladder before any motion. So if the dropped Leg re-faults on engage,
    nothing moves: the five hold, every Leg is HELD and the recovery ends FAILED.
    Also unverified: the engage latency, from the Closed-loop command to a
    Leg's first Closed-loop heartbeat. It must stay under about 190 ms (400 ms
    less two 99 ms heartbeat periods, one of them lost, and one 10 ms tick). A
    slower drive fails a normal activate or deactivate, with nothing moved.
    Two Sitting checks (FW 53 audit, 2026-11-26): (1) after Homing, log each
    Leg's rest position on its bottom stop and compare it with the Stow window's
    lower edge, -0.11 rev: a Leg resting deeper reads not-at-Stow while
    Closed-loop, so a pre-rise abort holds it HELD instead of IDLEing it (benign:
    the FAULT exit stow lifts it to 0 and IDLEs it), and the margin is only
    0.01 rev below the Homing stop; (2) a Leg in a drive state other than Idle,
    encoder search or Closed-loop blocks every Leg reboot or Idle: use the E-stop
    button.

## Enforcement points

| Invariant | Code | Test |
|---|---|---|
| A1 | `teensy_bridge_node.py` `_arm_setpoint_output` (sole 0→1 path); `__init__` boot-arm removed; `_park_hand_disarmed` + `_published_fault_state` (the recovery-park precondition) | `tests/ros/test_teensy_bridge_node_read.py` (boot-arm inert); `tests/ros/test_teensy_bridge_node_recover.py::test_arm_is_refused_while_the_recovery_park_runs`, `::test_fault_state_reads_recovering_only_while_the_park_runs` |
| A2 | `state_machine.py` `ActiveHandler` arm phase; `FaultHandler.on_enter` disarm; `FaultHandler` real-fault exit stow (choreography step 6); `orchestrator_node.py` `arm_setpoints`/`disarm_setpoints` dispatch and the `legs_closed_loop` derivation in `_on_robot_state` | `tests/ros/test_state_machine.py` arm-phase / retry / FAULT cases, `TestFaultExitStow` and `TestFaultExitStowFromLevelling`; `tests/ros/test_orchestrator_node.py::TestLegsClosedLoop`, `TestFaultExitStowThroughTick` and `TestFaultExitStowFromLevellingThroughTick` |
| A3 | `teensy_bridge_node.py` `_run_deactivate` head | disarm-before-DEACTIVATE order test |
| A4 | `state_machine.py` `ActiveHandler.on_exit` (no blank) + `IdleHandler` deferred blank | mode-survives-deactivate ordering test |
| A5 | `trajectory_node.py` wire-state WARN + response suffix + `is_homed` seed gate; `teensy_bridge_node.py` `teensy_mpc_active` KeyValue; `state_machine.py` `BootHandler` latch pre-flight | disarmed-accept, seed-gate, pre-flight tests |
| A6 | `state_machine.py` `homing_refusal` (the predicate) checked by `StateMachine._homing_entry_refused` on every transition into HOMING; `orchestrator_node.py` the `leg_states` / `legs_idle_since` / `leg_states_time` derivation in `_on_robot_state`; firmware twin `leg_homing.cpp` `homing_request` (`leg_stow_gate.cpp` `all_present_legs_idle`); the mixed case: `leg_reference.py` (`reference_unproven`, `LegDropWatch`, `mixed_case_verdict`) via `state_machine.py` `FaultHandler._mixed_recovery` | `tests/ros/test_state_machine.py::TestHomingGate`; `tests/ros/test_orchestrator_node.py::TestHomingGateThroughTick`; `tests/ros/test_leg_reference.py`; `tests/firmware/native/test_leg_homing.cpp` (the "FW 53" cases), `tests/firmware/native/test_leg_deactivate.cpp` "R3 mixed case" |
| A7 | `state_machine.py` `leg_idle_blockers` (the shared classifier) and `idle_refusal`, checked by `BootHandler.execute` (the homed skip to IDLE) and `ActiveHandler._legs_out_target`; `teensy_bridge_node.py` `_leg_idle_rule_refusal` over `_present_leg_states`, called by `_reboot_odrives`, `_svc_encoder_search` and `_do_home`; firmware authority `leg_stow_gate.cpp` (`INVARIANTS.md` I-FW-19) with the host twin `teensy_link/rpc_args.py` `requires_platform_at_stow` | `tests/ros/test_state_machine.py::TestBootIdleGate`; `tests/ros/test_state_machine_deactivate_confirm.py::test_legs_out_with_legs_not_all_idle_goes_to_boot_not_idle`; `tests/ros/test_teensy_bridge_node_leg_idle_rule.py`; `tests/ros/test_teensy_bridge_node_fw53_host.py`; `tests/firmware/test_stow_gate_xref.py`; `tests/firmware/native/test_leg_stow_gate.cpp` |

## The choreography (one cycle, walked at the control-system level)

0. (BUG-130, 2026-11-25; C-LEVEL-1.A in `ros_ws/docs/levelling_frame.md`.)
   Operator sends `activate`. **IdleHandler decides it, still in IDLE**: no fresh
   `trajectory/status` with `gravity_correction_loaded` → refused with a WARN
   naming `level`, machine stays IDLE, nothing moved; else it fetches the six
   Levelled Active pose Leg targets from `trajectory/levelled_activate_targets`,
   and a refusal there (correction gone, a Leg outside the 0.10 rev band) is the
   same IDLE + WARN. Only a fetched target set takes the machine to ACTIVE.
   Enforcement: `state_machine.activate_levelling_refusal` + `IdleHandler`; tests
   `tests/ros/test_state_machine.py::TestIdleLevelledActivate`,
   `tests/ros/test_orchestrator_node.py::TestLevelledActivate`.
   Step 0 and A6 are separate gates on separate commands from IDLE and neither
   reads the other's inputs: `home` passes only A6 (Leg states), `activate` only
   C-LEVEL-1.A (the loaded correction), so a loaded correction never lets `home`
   past Closed-loop Legs and six settled Idle Legs never let `activate` past a
   missing correction. Test:
   `tests/ros/test_state_machine.py::TestHomingGateAndLevelledActivateCompose`.
1. ActiveHandler requests the profiled ACTIVATE, which the orchestrator sends
   with those targets (the bridge fires can-bridge `ACTIVATE_TO`; TRAP_TRAJ to the
   Levelled Active pose — `mpc_active=0`, the stream is irrelevant to a profiled
   move). An `activate` request with no fetched targets is failed by the
   dispatch, never sent as the constant pose. `level`'s `level_activate` is the
   one ACTIVATE that still goes to the constant pose.
2. ACTIVATE completes → ActiveHandler publishes the streaming mode (STANDBY).
   trajectory_node seeds a hold **from measured telemetry** at the (levelled) active pose
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
5. `deactivate` → ActiveHandler requests the deactivate and **stays ACTIVE
   until the bridge confirms it** (2026-10-23: the machine used to leave ACTIVE
   on the command and reached IDLE on a refused DEACTIVATE with the legs still
   up — `logbook/2026-10-23-deactivate-rejected-state-machine-divergence.md`);
   **the streaming mode stays published** (A4). The bridge disarms in-process
   (A3) and waits for the firmware's T2J bit3 to drop before firing, whoever
   did the disarm — from that instant the Teensy ignores the still-flowing
   stream and its staleness watchdog is inert (`s_mpc_active=0` gates it) —
   then runs the parallel no-tilt TRAP_TRAJ descent. ERR_REJECTED = the firmware latched
   nothing: retried up to 3 attempts 1 s apart. ERR_TIMEOUT = the ack was lost
   and a start MAY have latched: the bridge observes the legs through the
   DeactivateMonitor window before any verdict and never re-sends. A genuine
   failure, a spent budget or no reply within 30 s opens a 1 s reconcile
   window: the machine follows the legs to IDLE if they leave closed loop,
   else stays ACTIVE (disarmed) with the reason logged; streaming mode
   switches are then refused until the operator re-arms (no auto re-arm).
   The same legs-out rule (5 consecutive ticks out of closed loop, after
   closed loop was seen) applies throughout armed ACTIVE. Both legs-out exits
   to IDLE request **disarm + Hand IDLE only** (`disarm_and_idle_hand`:
   `/set_setpoint_output false` + `/set_hand_state IDLE`), never a leg
   DEACTIVATE — legs that fell out of closed loop must not be re-energised
   and lowered a second time. The disarm lands on the host flag at once and on
   the wire within one heartbeat (~100 ms), inside the 250 ms SETPOINT_STALE
   window that opens when IdleHandler blanks the mode.
   IdleHandler blanks the mode on entry; the emitter then stops, long since
   disarmed.
6. Real fault at any point → FAULT entry disarms (A2). Guard-only fault from
   ACTIVE → stays armed, guard suppresses output, resume re-verifies the arm
   through A1 (idempotent if still armed).
   **The real-fault exit stows the legs whenever all six are Closed-loop**
   (2026-10-06 for ACTIVE, LEVELLING added 2026-10-07, any origin since FW 53
   on 2026-11-26: an aborted FW 53 rise holds the six Closed-loop mid-air
   after any ladder). The trigger: a forced
   ACTIVE→FAULT cancels ActiveHandler.on_exit's deactivate, and a fault during
   LEVELLING (whose `level_activate` holds the same active pose through its
   settle / tilt / correction / persist / mocap-check phases until
   `level_deactivate`) has no stow at all — LevellingHandler.on_exit only
   clears requests — so either way the legs could reach IDLE still holding an
   active pose. Once every error and the guard latch have cleared, a real
   fault (whatever state the visit began in, a guard-only fault promoted
   mid-FAULT included) requests the profiled DEACTIVATE when all six legs are
   in CLOSED_LOOP with no active error; otherwise it goes to BOOT, whose A7/A6
   predicate holds a mixed set. (Until 2026-11-20 it also required the robot to be homed.
   BUG-122 made the bridge clear `is_homed` whenever any drive loses its
   position reference, the Hand included, so that gate would have skipped the
   stow on a Hand-only loss and started HOMING with the legs at an active pose.
   CLOSED_LOOP with no active error already proves each leg's own reference,
   and DEACTIVATE only IDLEs the Hand.) It **holds FAULT until that
   DEACTIVATE succeeds**, so IDLE ("platform stowed, legs idle") is never
   claimed while the legs still hold a pose. A failed stow also holds FAULT,
   with one retry per operator `clear_errors`. The one exception is the proven mixed case
   (some Legs Closed-loop, exactly one dropped Leg, Idle, with a reference
   `leg_reference.reference_unproven` proves kept, A6): after the operator's
   `clear_errors` the exit requests the profiled DEACTIVATE(AXIS_ALL), once per
   FAULT visit. If any leg is not closed-loop in any other case,
   FAULT exits to BOOT at once, as before, with the reason the reference is not
   proven when it was a mixed case (re-arming a leg that dropped out is
   BOOT→HOMING's job). BOOT reaches HOMING only through A6 (2026-11-25): a
   BOOT whose robot is not homed while any Leg is still Closed-loop (the
   mixed case of one Leg lost during ACTIVE) holds in BOOT with the A6 notice
   instead of homing under the posed Platform, and a homed BOOT reaches IDLE
   only through A7: with any Leg not Idle it holds with the A7 notice instead
   of claiming a Stow it cannot show. The guard-only classification is unchanged: only a
   latch from ACTIVE with the wire armed is guard-only (LEVELLING never arms
   the wire, so a guard latch there is always a real fault). The
   mode stays `ERROR` through this stow. A4 does not apply: the wire was disarmed at FAULT entry (A2), and A3
   re-confirms the disarm before DEACTIVATE. The guard-only resume path is
   unchanged and does not stow.

## What this deliberately does NOT change

- The **SETPOINT_STALE watchdog** (250 ms, firmware) is untouched — it remains the
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
