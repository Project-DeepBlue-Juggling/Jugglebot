---
title: Can-bridge foundation Phase 5 — hand command conduit + hand homing + deactivate-idles-hand + cmd-echo telemetry
type: feature
date: 2026-07-01
status: resolved
phase: "5"
related_plan: canbridge-foundation-coldstart-parity.md
related_entries:
  - 2026-06-30-canbridge-phase6-reboot-latch
  - 2026-06-29-canbridge-phase3-version-validated
  - 2026-06-29-canbridge-phase2-coldstart-relay-state
  - 2026-06-29-canbridge-phase1-platform-relay-seam
  - 2026-06-29-canbridge-phase0-native-harness
  - 2026-06-27-can-node-teensy-parity-audit
  - 2026-06-18-temporal-accuracy-resolved-fractured-solution
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.py
  - config/generated/udp_protocol.h
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hand_ops.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hand_ops.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_homing.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_homing.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - controller/teensy_link/rpc_args.py
  - controller/teensy_link/protocol.py
  - controller/teensy_link/__init__.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/firmware/test_hand_traj_xref.py
  - tests/firmware/test_homing_xref.py
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/firmware/test_rpc_dispatch_lint.py
  - tests/firmware/test_native_firmware.py
  - tests/firmware/native/build.py
  - tests/firmware/native/fake_hal.h
  - tests/firmware/native/fake_hal.cpp
  - tests/firmware/native/test_hand_ops.cpp
  - tests/teensy_link/test_rpc_args.py
  - tests/ros/test_teensy_bridge_node_hand.py
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
commits:
  - 2556014
  - 121c692
subsystem:
  - can
  - firmware
  - hand
  - testing
tags:
  - cold-start
  - hand
  - catch
  - temporal-accuracy
  - parity
---

## Summary

Restored the **full hand surface** the can-bridge silently no-op'd against — the
four hand services `catch_coordinator` clients (`set_hand_state`, `set_hand_gains`,
`set_hand_traj_cmd`, `smooth_move_hand`), all GAPs against the bridge until now —
**plus hand homing**, so the cold-start sequence homes legs **and** the hand like
`can_node`. Six deliverables:

1. **Hand traj / smooth-move conduit** (`hand_ops.cpp`, new): the one
   `HAND_TRAJ_CMD` RPC (byte-0 discriminated) — the host builds the exact 8-byte
   `0x6D0` `PLATFORM_TRAJ_CMD` payload (byte-identical to `can_node`), the firmware
   sends the `CLOSED_LOOP` + `POSITION/PASSTHROUGH` preamble to axis 6 then forwards
   the payload on the **firmware-owned** `0x6D0` id — **aborting the traj TX if a
   preamble send fails**.
2. **Hand state / gains**: `set_hand_state` / `set_hand_gains` ride the Phase-1
   axis-6 allow-table (`SET_AXIS_STATE` / `SET_POS_GAIN` / `SET_VEL_GAINS` on axis
   6), byte-identical to `can_node`. Unknown `set_hand_state` strings are rejected
   Jetson-side.
3. **Hand homing** (`leg_homing.cpp` extended to axis 6): the same move-to-hardstop
   state machine, parameterised by `Homing::HAND_*` (speed −3.0, curr limit 8.0,
   headroom 3.0, abs-pos −0.1) via per-axis accessors. Its PID gains are applied
   **host-side** by the bridge (`_apply_hand_gains`, refuse-flash-defaults) **before**
   `HOME(6)` — byte-identical to `can_node._home_robot_steps`' HAND branch.
4. **Configure the hand** (`_run_configure`): the hand half of `_setup_odrives` —
   gains + vel/curr limits + `POSITION/PASSTHROUGH` on axis 6.
5. **Deactivate idles the hand** (`_run_deactivate`): `SET_AXIS_STATE(6, IDLE)` after
   the leg descent — `can_node` `JUGGLEBOT_AXES` parity (it idled legs + hand).
6. **Hand command-echo telemetry** (`HAND_CMD_ECHO`, new): the bridge sniffs the
   Platform Teensy's `Set_Input_Pos` to the hand ODrive on CAN3 and echoes it, so
   `hand_telemetry`'s `pos_cmd`/`vel_ff_cmd`/`tor_ff_cmd` (hardcoded 0 on the bridge)
   carry the hand's commanded-vs-measured tracking-error diagnostic again.

This unblocks **Phase 4** (its last dependency). Landed 2026-07-01 software-complete
to the powered-sitting gate; the hand-homing + catch validation subsequently **PASSED
at the 2026-07-02 powered sitting** (§Verification / §Open Questions).

## Motivation

The `can_node`→Teensy parity audit flagged the entire hand subsystem as GAPs (matrix
rows 36–40, 51, 58, plus 29/32). The production launch runs `catch_coordinator_node`,
which clients `smooth_move_hand` / `set_hand_traj_cmd` / `set_hand_gains` — all of
which resolve to nothing against the can-bridge, so **the catch pipeline is silently
inert** (`catch_coordinator` logs "service not ready" and no-ops). Cold-start homed
legs only; the hand was never homed, configured, or idled by the bridge. Phase 1
already un-rejected axis-6 ODrive ops behind the allow-table and Phase 0 reserved the
`HAND_TRAJ_CMD` (0x0054) + `HAND_CMD_ECHO` (0x8A) wire ids — Phase 5 implements them.

## Design

### Hand traj / smooth-move — host builds the payload, firmware owns the id

`hand_ops.cpp` (new, keeps `rpc.cpp` thin, mirrors `platform_relay.cpp`) is the
`HAND_TRAJ_CMD` dispatch: gate on `jugglebot_commands_allowed()` → send the
`CLOSED_LOOP` + `POSITION/PASSTHROUGH` preamble to axis 6 (**abort** on a failed
preamble send) → build `CanFrame{id = PlatformCanId::TRAJ_CMD, len = 8, buf =
payload}` and forward. The `ArgHandTraj` RPC arg is `u8 payload[8]` — the **exact**
`0x6D0` payload, built HOST-side byte-identical to `can_node._send_hand_traj_cmd` /
`_smooth_move_hand` (byte 0 = discriminator: 0/1/2 = catch-traj type, 3 =
smooth-move). `set_hand_traj_cmd` + `smooth_move_hand` both ride this one RPC.

### Hand homing — firmware moves, host applies gains

`leg_homing.cpp`'s `homing_request`/`homing_step`/`finish`/`homing_result` accept
`axis == HAND_AXIS`; the per-axis arrays/bounds widen `NUM_LEGS → NUM_AXES`; four
`static inline` accessors (`homing_speed_rps`/`homing_curr_limit_a`/
`homing_headroom_a`/`homing_abs_pos_rev`) select `Homing::HAND_*` for the hand,
`Homing::LEG_*` for legs. The hand's gains are applied **host-side** before `HOME(6)`
(`_apply_hand_gains`), so the firmware move stays gain-agnostic (it inherits the
just-applied gains, exactly as the leg move inherits the prior `_setup_odrives`).

### `robot_state`/cold-start wiring (host)

`home_axes`/`configure_axes` defaults now include the hand (axis 6); the bench rig's
existing `[0]` override excludes it automatically. `_run_home` applies hand gains
(refuse-flash-defaults, abort the sequence on failure) then homes the hand with the
same `HomingMonitor` (axis-agnostic). `_run_configure` configures the hand (gains +
limits + PASSTHROUGH). `_run_deactivate` idles the hand at the end.

### Hand command-echo

`decode_into_cache` sniffs `arb_id(HAND_AXIS, set_input_pos)` (cmd 0x0C) into a
single-slot dirty stash; `hand_cmd_echo_uplink_step` (task_telem) emits a
`HAND_CMD_ECHO` `{t_bridge_us, data[8]}` when a fresh command is pending. The host
decodes `data` as `<f h h>` ÷ `INPUT_SCALE_HAND_VEL/_TOR` (100) — byte-identical to
`can_node._handle_hand_input_pos` — into `_last_hand_cmd`, read by
`_publish_hand_telemetry`.

## Implementation

Codegen (`config/generate_udp_protocol.py`): `ArgHandTraj` (u8[8]) + the
`HandCmdEcho` message (0x8A, `{u64 t_bridge_us, u8 data[8]}`) — the enum ids were
reserved in Phase 0. Firmware: `hand_ops.{h,cpp}` (new) + `rpc.cpp`/`rpc.h` dispatch
(replacing the last Phase-0 `ERR_NOT_IMPL` reserved stub), `leg_homing.{h,cpp}`
axis-6 extension, `can_buses.{h,cpp}` sniff + stash, `telemetry.{h,cpp}` uplink,
`.ino` wiring. Host: `rpc_args.py` (`encode_hand_traj_cmd`/`encode_smooth_move_hand`
pure encoders + `decode_hand_cmd_echo`), `protocol.py`/`__init__.py` exports,
`teensy_bridge_node.py` (four services + handlers, `teensy_hand_traj_cmd` wrapper,
`_apply_hand_gains`, the `_run_*` extensions, the `HAND_CMD_ECHO` subscribe +
`_publish_hand_telemetry` wiring).

## Verification

- **Full suite** (`pytest tests/ -q`, run 2026-07-01): **1972 passed, 1 xfailed in
  482.79 s** — fully green (net **+45** over the 1927 baseline = the new Phase-5
  tests; no regressions, no failures). The order-flaky allocation tests
  (`test_hot_loop_allocation_contract`, `test_t3b_h4_on_post_solve_allocates_within_budget`)
  both passed this run.
- **Baseline** (`pytest tests/ -q`, run 2026-07-01, pre-change): **1927 passed, 1
  xfailed in 461.59 s**.
- **Firmware** (`pio run`, 2026-07-01): **green** ([SUCCESS]).
- **Native harness**: `test_hand_ops` — **5 cases / 20 assertions pass**;
  divergence-catch **proven** (neutering the first preamble abort failed 1 case / 2
  assertions; revert restored all-green).
- **Wire parity**: `tests/firmware/test_hand_traj_xref.py` (13 cases) — the host
  `0x6D0` payloads are byte-identical to `can_node`; `test_homing_xref.py` hand
  branch (5 new cases) — the firmware HAND homing arithmetic matches `can_node`'s
  HAND branch; `test_udp_protocol_xlang.py` — `ArgHandTraj` + `HandCmdEcho`
  round-trips.
- **Codegen deterministic**: `generate_config.py` + `generate_udp_protocol.py`
  regenerate with no diff across reruns.
- **Powered sitting — PASSED (2026-07-02, full Jugglebot powered, Phase-5 firmware
  flashed; operator actuated, e-stop in hand; Claude verified logs read-only).** All
  five prepped steps behaved exactly as predicted, no issues:
  - **Hand homing (isolation, `home_axes=[6]`, then full legs+hand):** the hand
    retracts at −3.0 rev/s into the top hardstop, current-trips, IDLEs, and sets the
    reference — clean, no over-travel (the physical-intuition checkpoint held: the
    hand retracts, as framed).
  - **Catch pipeline:** `set_hand_gains` → `smooth_move_hand(9.858)` primes to top of
    stroke → `set_hand_traj_cmd(delay=1.0, vel=3.0, type=1)` fires the hand **after
    the delay, not instantly** — the absolute-deadline / no-restamp contract confirmed
    on hardware. `hand_telemetry`'s `pos_cmd`/`vel_ff_cmd` populated during the move
    (HAND_CMD_ECHO working; previously hardcoded 0).
  - **Deactivate** idled the hand alongside the legs.
  - **Log corroboration** (`~/.ros/log/2026-07-02-09-21-57`): the `teensy_bridge_node`
    process ran the full ~7.8 min **cleanly** ("finished cleanly", no
    error/fault/estop/exception mid-run). *(The bridge's per-command INFO logs were
    not captured in `launch.log` at this sitting — a Foxy output-routing quirk — so
    per-command confirmation rests on operator witness + the `hand_telemetry` topic;
    the `launch.log` evidence is process-lifecycle + error-level + the orchestrator/
    rosout stream only.)* The `orchestrator_node` looped `BOOT→HOMING→FAULT` for
    ~3.3 min (the **Phase-4 gap** — its HOMING handler isn't yet wired to the bridge's
    `/home`) and then, the moment the manual homing completed, transitioned
    `BOOT→IDLE` and stayed there — transitively corroborating the Phase-2+3+5
    cold-start chain's `is_homed` write→persist→read→skip-homing path
    (`firmware_validated` un-wedging BOOT): the orchestrator only skips homing when
    `is_homed=True`, which requires a successful **legs + hand** home written via the
    Phase-2 relay `STATE_WRITE` and read back. (Full Phase-2/3 validation — reconnect
    re-read, REBOOT-clears, `levelling_complete`/`pose_offset` — remains the Phase-4
    sitting.) **Residual (trivial):** the standalone `set_hand_state` *service* was not
    directly invoked at the sitting — its mechanism (bridge `SET_AXIS_STATE` on axis 6)
    *was* exercised (homing's CLOSED_LOOP/IDLE transitions + the deactivate hand-idle),
    so row 36 is a one-command follow-up (`ros2 service call /set_hand_state …`).

## Discussion

### Why host-builds-the-8-byte-payload over firmware-assembles-from-semantic-fields

The `HAND_TRAJ_CMD` RPC could have carried semantic fields (discriminator, vel_u16,
wall_time_ms, target_rev) and let the firmware assemble the `0x6D0` bytes. It carries
the finished 8-byte payload instead. Three concrete failure modes this prevents:

1. **A firmware re-stamp of the deadline** becomes *structurally impossible*. The
   hand catch trajectory carries an **absolute** `wall_time_ms` the Platform Teensy
   fires at when its synced clock reaches it. If the firmware saw the deadline as a
   semantic field it could (through a future edit) re-stamp it relative to bridge-RX
   time — reintroducing exactly the transit-jitter error the BallButler
   temporal-accuracy arc fought (a re-stamp/feedforward error there made the lag
   *worse*, 44→56 ms; `2026-06-18-temporal-accuracy-resolved-fractured-solution`).
   Forwarding opaque bytes, the firmware *cannot* re-stamp — the invariant holds by
   construction, not by discipline.
2. **A second definition of the `0x6D0` byte layout** (drift risk). The Platform
   Teensy's `PLATFORM_TRAJ_CMD` decoder is the authority for that layout; `can_node`
   built it on the Jetson. Keeping the byte construction on the host (byte-identical
   to `can_node`) means the layout lives in exactly one place on the sending side,
   and the xref test pins it to `can_node`. A firmware re-assembly would be a second
   place to keep in sync with the Platform Teensy.
3. **It is still least-privilege.** The plan's rule is "the `0x6D0` `can_id` is
   firmware-owned, never a Jetson-supplied raw *frame*." The Jetson supplies the
   *payload*; the firmware supplies the *id* + dlc. That is not a raw frame — a
   compromised/buggy Jetson still cannot inject an arbitrary arbitration id (e.g. a
   leg-command id bypassing the interp step-gate), which is the actual escalation the
   rule guards against.

The firmware, as a result, does **not** branch on the discriminator at all: the
preamble is identical for a catch traj and a smooth-move (`can_node` issued the same
`CLOSED_LOOP` + `POSITION/PASSTHROUGH` in both), so the whole conduit is
preamble-then-forward. `test_hand_ops` asserts a smooth-move (byte 0 = 3) and a
catch-traj payload forward identically — the firmware treats the discriminator as
opaque.

### The preamble-abort is the safety crux, and it is compiled-tested

Running a hand trajectory against an ODrive that is **not** in
`CLOSED_LOOP`/`PASSTHROUGH` either faults or silently no-ops the move — and for a
*catch*, a no-op means the hand does not close on the ball. `can_node` always issued
the preamble before the `0x6D0` frame (the dropped precondition the audit flagged as
row 37). The firmware aborts the traj TX (returns `ERR_TIMEOUT`, sends **no** `0x6D0`
frame) if either preamble send fails to enqueue. This is exercised as a *compiled*
assertion in `test_hand_ops` (via a new `fake_set_send_fail_index` HAL hook) for both
the 1st and 2nd preamble send — and the divergence proof (neutering the abort → the
test bites) confirms it is load-bearing, not decorative.

### Hand gains: host-applies over firmware-applies (the HOME(6) decision)

The plan text reads as if the firmware HOME(6) applies the hand gains. It does not —
the host does, before `HOME(6)`. Failure modes this prevents / properties it buys:

- **The refuse-flash-defaults safety stays on the host, byte-identical to
  `can_node._set_hand_gains`** (which *raised* if a gain CAN send failed — "refusing
  to proceed" on flash defaults). `_apply_hand_gains` returns `(False, reason)` and
  `_run_home` aborts the sequence before `HOME(6)` — so the hand is never driven into
  a hardstop on flash-default gains (parity row 40). A `test_run_home_hand_gain_
  failure_aborts_before_home` pins that HOME never fires when a gain write fails.
- **The gain VALUES live on the host** (`self._hand_gains`, config default or a
  `set_hand_gains` update), so a catch's softer compliance gains — set via the
  restored `set_hand_gains` service — are the same object the cold-start path reads.
  A firmware-baked constant could not track a runtime gain change.
- **The safety-critical `leg_homing.cpp` change stays minimal** — accept axis 6, HAND
  constants, widen the arrays. No gain-send logic in the homing TU.

This matches the host-orchestrates / firmware-moves split the leg homing already uses
(there is no per-axis motion RPC, so the move must be firmware; everything else is
host).

### `HAND_CMD_ECHO` is a snapshot, not a lossless stream

The Platform Teensy streams `Set_Input_Pos` to the hand ODrive during a catch. The
bridge coalesces to the newest command (a single dirty-flagged slot) and emits at the
telemetry-task rate only when a fresh one was sniffed — event-driven, silent while
the hand is idle. `can_node` likewise published the *last* command at its telemetry
rate. A ring (like the relay reply ring) would be wrong here: a diagnostic echo wants
the freshest sample, not every intermediate setpoint. CAN3 `SRX_DIS` (Phase-1
bench-confirmed) means the bridge never sniffs its own leg TX, so only genuine
Platform→hand commands are echoed; the `axis == HAND_AXIS` guard is belt-and-braces.

### What was ruled out

- **A `home_hand` bool param** (vs adding axis 6 to `home_axes`): rejected because
  the bench rig already sets `home_axes=[0]`, which excludes the hand for free — a
  separate bool would be one more thing to remember to turn off on the bench.
- **Adding `leg_homing.cpp` to the native harness** (vs the Python xref): deferred.
  The hand HOMING is validated by the `test_homing_xref.py` hand branch (matching how
  the leg homing is tested); compiling `leg_homing.cpp` host-side needs shims for
  `axis_state`/`can_buses`/`fault_machine`/`leg_activate`/`leg_deactivate`/`time_base`
  — a larger, separate robustness investment (Open Question).

## Open Questions

- **Powered sitting — RESOLVED (2026-07-02, see Verification).** All four checks
  passed on hardware: (a) the hand homes without over-driving; (b) `smooth_move_hand`
  reaches the target; (c) `set_hand_traj_cmd` fires the hand **after the delay, not
  instantly** (the absolute-deadline / no-restamp contract confirmed); (d)
  `HAND_CMD_ECHO` populates `hand_telemetry`'s command fields. The orchestrator
  reaching IDLE the moment homing completed transitively corroborates the Phase-2+3+5
  cold-start chain. *Remaining trivial follow-up:* directly invoke the standalone
  `set_hand_state` service (its mechanism was exercised via deactivate's hand-idle;
  row 36 stays `ported+unvalidated` until then — a one-command call).
- **Phase-4 signal from the sitting.** With no orchestrator wiring yet, the
  orchestrator loops `BOOT→HOMING→FAULT` until `is_homed` is set (its HOMING handler
  can't drive the bridge's `/home`) — then correctly `BOOT→IDLE`. Phase 4's
  `home_motors` action shim closes exactly this: the skip-if-homed logic already
  works, so Phase 4 need only make the HOMING handler drive `/home` (+ configure)
  instead of faulting.
- **`leg_homing.cpp` in the native harness** — would make the hand HOME(6) decision
  logic a compiled assertion (not just a Python arithmetic xref). Deferred as a
  separate harness-growth task.
- **A reusable `tools/probes/` hand-homing / catch-traj probe** — the first sitting
  used a lightweight `/tmp` `/hand_telemetry` rclpy subscriber; promote to
  `tools/probes/` only if catch-traj characterization becomes repeated work (the
  memory `feedback_reusable_probes_in_repo` default-to-`/tmp`-promote-later rule).

## Related

- Plan: [`canbridge-foundation-coldstart-parity.md`](../plans/archived/canbridge-foundation-coldstart-parity.md) — Phase 5.
- Parity matrix: `ros_ws/docs/can-node-teensy-parity.md` (rows 29, 32, 36–40, 51, 58).
- Prior phases: [[2026-06-30-canbridge-phase6-reboot-latch]] (hand in AXIS_ALL — the
  reboot latch's leg-guard is preserved here), [[2026-06-29-canbridge-phase1-platform-relay-seam]]
  (the axis-6 allow-table this rides), [[2026-06-29-canbridge-phase0-native-harness]]
  (the harness `test_hand_ops` extends + the reserved ids it implements).
- Temporal-accuracy precedent: [[2026-06-18-temporal-accuracy-resolved-fractured-solution]]
  (why the absolute-deadline / no-restamp design is non-negotiable).
