---
title: Ball Butler cutover from can_node to teensy_bridge_node — Phase A bench bringup
type: feature
date: 2026-06-08
status: tuned
phase: teensy-can-offload
related_plan: teensy-can-offload.md
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/ball_butler_state.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/ball_butler_state.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/ball_butler_protocol.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/src/jugglebot/jugglebot/can_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/launch/teensy_bridge_launch.py
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - controller/teensy_link/protocol.py
  - controller/teensy_link/rpc_args.py
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - tests/ros/test_can_node.py
  - tests/ros/test_teensy_bridge_node_read.py
  - tests/ros/test_teensy_bridge_node_bb.py
  - tests/teensy_link/test_rpc_args.py
  - logbook/2026-06-08-bb-cutover-bench.md
  - logbook/INDEX.md
commits:
  - c4f56e3
  - 0dc0ca6
  - d679407
  - 13cda0a
  - e057e45
  - 0935c63
  - 5875531
  - e730010
subsystem:
  - can
  - ros
  - firmware
tags:
  - feature
  - hardware
  - firmware
  - bringup
  - can
  - cutover
---

# Ball Butler cutover from can_node to teensy_bridge_node — Phase A bench bringup

## Summary

First end-to-end migration of Ball Butler from the legacy `can_node` direct
USB-CAN path (now physically removed from the system) to the can-bridge Teensy
4.1 path on CAN1. Eight commits land: a typed BB heartbeat cache + decoder on
the firmware, four typed RPC commands (BB_THROW / BB_RELOAD / BB_RESET /
BB_CALIBRATE_LOC) with range validation + presence gating, BB state hoisted
onto the upstream `HeartbeatT2J` payload, and the four production `bb/*`
services + `bb/heartbeat` publisher absorbed by `teensy_bridge_node`.
`can_node` loses its BB code in the same commit (atomic — no dual-publisher
window), along with the structurally-wrong throw announcement publisher.
GUI Calibrate now drives through this new path end-to-end.

## Bench results

Phase A wraps up with the GUI Calibrate test passing. Validation evidence:

- **Firmware decode (USB serial `[bb]` line, 2026-06-08):** `[bb] state=IDLE
  ball=1 yaw=0.3 pitch=89.9 hand=0.0 age=80ms` — populated with BB's actual
  state, byte-identical to what `python-can` would have decoded. Confirms the
  CAN1 0x7D1 RX path + the seqlock snapshot work as designed.
- **Bridge publisher (2026-06-08):** `ros2 topic echo bb/heartbeat` returned
  matching values (`connected=true, ball_in_hand=true, state=1, yaw_deg≈0.3,
  pitch_deg≈89.9, hand_pos_mm=0.0`) at the expected 10 Hz cadence.
- **Service surface (2026-06-08):** `ros2 service list` showed all four
  production names — `/bb/calibrate`, `/bb/reload`, `/bb/reset`,
  `/bb/send_throw_command`. No `bb/heartbeat` on `can_node` post-cutover
  (verified by topic-list grep).
- **RPC round-trip (2026-06-08):** `ros2 service call /bb/reset
  std_srvs/srv/Trigger` returned `success=True, message='Reset command sent.'`
  — proves the firmware dispatch + UDP response path works on a clean call.
- **GUI Calibrate (2026-06-08, user-driven rerun):** browser-GUI Calibrate
  button drove BB through CALIBRATING → IDLE, mocap collected markers, and
  `bb/calibration_result` was published with success. First attempt failed
  with a 3.80 mm circle-fit deviation — a known transient mocap optical
  issue, unrelated to phase A code; the rerun succeeded.
- **Test suite (`pytest tests/ -q`, run 2026-06-08): 1691 passed, 1 xfailed
  in 437.35 s.** Delta vs the pre-cutover baseline (1687 passed, 1 xfailed):
  +4 — 11 new BB bridge tests minus 7 removed legacy `can_node` BB tests.
- **Firmware build:** `pio run` green throughout — every commit gated on
  green before AND after. Final size at A6: `text 214336 / data 34496 /
  bss 102016 / dec 350848`.

## Discussion

### Why the structurally-large surface (eight commits, not one)

The cutover touches firmware decode, firmware encode, UDP codegen, firmware
RPC dispatch, bridge Python, and `can_node` surgery. We split it across
eight build-gated commits rather than one atomic mega-commit because each
piece has an independent rollback target: a regression in the BB decode
shouldn't force a revert of the bridge node surgery. The trade-off accepted:
a brief sequence where firmware ships BB decode (A3) before the bridge knows
how to consume it (A6). The intermediate states are observable but inert
(`[bb]` diag line on USB serial, bb_state field populating with zeros on
the wire) — no production behaviour change between A3 and A7.

### The D1 deviation (production names vs `/teensy/*`)

The bridge HANDOFF's D1 set up a `/teensy/*` namespace for *all* bridge
topics/services on the rationale that running side-by-side with `can_node`
on the same name would create a dual-publisher conflict. Phase A is the
first commit that deliberately violates that. The reason: with USB-CAN
removed, `can_node`'s publishes go nowhere (the bus is dead) — there's no
second authority to conflict with. Preserving the production names
(`bb/calibrate`, `bb/heartbeat`, etc.) means the GUI / orchestrator /
mocap / throw_director see no name change across the cutover; otherwise
each would have needed a separate rewire commit. The bridge node is now
internally inconsistent (BB at production names; legs+hand still under
`/teensy/*`); that inconsistency is expected to resolve in phase C when
the leg/hand services migrate too.

### Why typed BB API on the firmware (D2)

The four BB commands could have been a single generic "send arbitrary frame
to CAN1" RPC with the Jetson doing all encoding. We chose typed (four
distinct `RpcMethod` enum values + `ArgBbThrow` arg struct) for one
load-bearing reason: it lets the firmware refuse a malformed throw
(`throw_args_valid()` checks yaw/pitch/speed/delay against the same ranges
as `ball_butler.encode_throw_command`'s Python validation) BEFORE a byte
goes onto CAN1. The defense layer is symmetric — Python validation at the
ROS service boundary AND firmware validation at the wire boundary — but
each layer protects against a different failure mode (Python catches
programming errors; firmware catches downstream-bug-corrupting-RPC-args
errors). The encoding logic is duplicated between
`ball_butler.py:77-119` and
`Teensy_code_canbridge/ball_butler_protocol.h:encode_throw()`; the byte-level
cross-reference test in `tests/teensy_link/test_rpc_args.py::
test_bb_throw_exact_bytes` (`'<ffff'`, 16 B) pins parity.

### Why the ThrowAnnouncement publisher was deleted, not migrated (D3)

`can_node._publish_throw_announcement` was a fallback publisher that ran
whenever `bb/send_throw_command` was called without `suppress_announcement=
True`. The original design had it predict the ball trajectory using
`predict_throw()` against the platform's default catch height. That worked
in 2024 when every catch was at the platform plane, but became
structurally-wrong once `throw_director_node` started managing
off-platform targets like the cone (the actual catch z + serial-chain ToF
differ from `can_node`'s predict). `throw_director` already published its
own, accurate `ThrowAnnouncement` and called `bb/send_throw_command` with
`suppress_announcement=True` to keep `can_node` quiet. The
`suppress_announcement` field became a workaround for the dual-publisher
problem.

We could have migrated the workaround pattern to the bridge node. We chose
to delete it instead. Reasons:

- One authority, one accurate announcement. No race between two publishers
  on the same topic.
- Removes the hardcoded `target_id="jugglebot"` TEMPORARY placeholder that
  was always wrong for cone catches.
- Decouples the CAN layer from BB calibration state — the bridge no longer
  subscribes to `bb/calibration_result`. The mocap solver is the sole
  consumer of that latched topic from the relevant CAN layer's perspective.
- The `suppress_announcement` field becomes dead protocol; retained on the
  `.srv` for interface stability during the cutover (one follow-up commit
  removes it cleanly).

The architectural separation that emerges:
**throw_director** (semantic layer): knows targets, owns inverse
ballistics, publishes intent. **Bridge** (wire layer): encodes 8 bytes,
sends to CAN1, reports back success/failure. Each layer owns exactly its
concern.

### CAN1 bus-health observability (separable follow-up)

During bench testing we observed CAN1 entering the error-passive state
(`[canhealth] bb tec=135 flt=passive flags=0x09`) — ACK + STUFF errors
climbing at ~600/sec. Symptoms:

- Intermittent `bb/calibrate` / `bb/reset` returning ERR_TIMEOUT after
  3 RPC attempts.
- A weird 1→5→1 BB state cycle at ~500 ms period that started ~5 s after
  the first calibrate command and continued for ~20 s.
- Eventually self-resolved (clean IDLE) once we stopped retrying.

This isn't introduced by phase A — `can_node` would have had the same
asymmetric ACK problem if it had been driving CAN1 directly. It WAS
hidden in the legacy USB-CAN topology because that bus had a different
node population. The most likely cause is BB Teensy's CAN filter
excluding our 0x7DD time-sync broadcasts (no ACK from BB for those
frames → TEC climbs). That should be tested by either:

- Inspecting BB Teensy firmware for its accept filter.
- Disabling the 0x7DD broadcast on CAN1 temporarily and watching TEC.
- Adding `0x7DD` to BB Teensy's filter.

Tracked as a separate phase-A.5 follow-up — not blocking phase A
sign-off but blocking comfortable use of the new path under load.

### Pre-existing infrastructure gaps surfaced

Two unrelated issues surfaced under the bench launch that aren't phase
A's responsibility but are now documented:

- **`controller/teensy_link` not on PYTHONPATH at ros2 launch time.**
  Fixed in commit `e730010` (launch file injects `JUGGLEBOT_REPO` to
  `additional_env`). Proper long-term fix is to install
  `controller/teensy_link/` as a sub-package of the `jugglebot` ROS
  package, deferred.
- **`motor_guard` `friction_ff_params.py:33` path-resolution bug.**
  `os.path.abspath(__file__)` plus a fixed-depth walk-up (5 levels)
  resolves to the repo root from `ros_ws/src/...` but lands at
  `ros_ws/install/jugglebot/` from the install tree. Pre-existing;
  the legacy launch was probably using a hand-edited or symlinked
  install. Not on the BB GUI Calibrate path so worked around by
  bypassing the full `jugglebot_launch.py`. Fix proposal: walk up
  looking for `config/hardware_config.yaml` as a marker, with
  `JUGGLEBOT_REPO` env-var fallback (10-line change). Deferred to a
  cleanup commit.
- **`rosbridge_websocket` requires `python3-bson`** which isn't
  installed by default on this Jetson. `sudo apt install python3-bson`
  fixes it. Was a blocker for the original browser-GUI test path until
  installed. Out of scope for phase A.

### Bench discipline lessons

During debugging I accumulated several orphan `teensy_bridge_node`
processes from `kill -TERM` calls that took out the `ros2 launch` parent
but left the actual Python node alive. The lesson: `kill -TERM
$LAUNCH_PID` on a `ros2 launch` parent only signals the launch process,
not its spawned children. Either use process-group kill (`kill -- -$PID`)
or, more robustly, PID-list explicit kill (`ps -eo pid,cmd | grep
teensy_bridge_node | awk '{print $1}' | xargs kill -9`). `pkill -f
teensy_bridge_node` matches its OWN parent shell when the shell's argv
contains the pattern as a string — kills itself before doing anything.
PID-list explicit kill is the only safe form.

## Scope deliberately deferred

- **`firmware_validated` for BB axes 7+8** — the BB ODrives broadcast
  heartbeats on CAN1 but the firmware currently decodes only the BB
  Teensy heartbeat (0x7D1), not the BB ODrive heartbeats. Phase A
  removes `can_node`'s version-check logic (which was dead anyway since
  USB-CAN went away) and ships `firmware_validated=False` for the BB
  group. Phase B restores it by decoding BB ODrive RX on CAN1 + surfacing
  validation via a new bridge field. State-machine HOMING that depended
  on full validation is not blocked because the GUI Calibrate test
  doesn't gate through HOMING — it triggers from IDLE directly.
- **`SendBallButlerCommand.suppress_announcement` field removal** — the
  field is dead protocol on the new path (D3) but stays on the `.srv`
  during this commit to avoid a coupled `.srv` rebuild. Single-commit
  follow-up.
- **Leg/hand cutover (phase C)** — phase A is BB only. Legs + hand
  still have `/teensy/*`-namespaced services on the bridge; their
  production-name migration is phase C.
- **`teensy_bridge_launch` integration into `jugglebot_launch.py`** —
  still standalone-launched per the bridge HANDOFF's side-by-side
  rationale. Folding into the main launch is part of `can_node` retirement.

## Follow-ups

- (phase B) BB ODrive axes 7+8 RX decode on CAN1; restore
  `firmware_validated` for BB.
- (phase A.5) CAN1 BB bus-health investigation — TEC growth +
  ACK/STUFF flags. Likely fix: BB Teensy CAN filter expansion to
  include `0x7DD`.
- (cleanup) `SendBallButlerCommand.srv` — remove
  `suppress_announcement` field + corresponding throw_director set.
- (cleanup) `motor_guard.friction_ff_params.py` — robust repo-root
  resolution.
- (cleanup) Install `controller/teensy_link/` as a sub-package of
  the `jugglebot` ROS package; remove the launch-time PYTHONPATH
  injection.
- (phase 13 prep) Once phase C lands, `can_node` is mostly empty —
  schedule its deletion.
