---
title: "BUG-99 FW 52: the can-bridge refuses reconfiguration RPCs while Armed, only an all-axes clear releases the global latches, a scheduled stream keeps its held clock offset through a sync loss and a legacy-to-scheduled entry is checked like a resume, UDP from any host but the Jetson is dropped and counted; I-FW-10 scoped to the bounded legacy Mode-1 hold; NOT flashed"
type: bugfix
date: 2026-11-25
status: in-progress
phase: "BUG-99"
subsystem: can-bridge
related_issues:
  - BUG-99
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/time_base.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/time_base.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_link.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_link.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - teensy_link/rpc_args.py
  - tools/probes/teensy_link_profiling/hermite_xref/teensy_interp.py
  - tests/hardware/bench_leg_sysid.py
  - tests/hardware/hand_stream_bench.py
  - tools/probes/lead_clamp_gap_bench.py
  - tests/firmware/native/test_rpc_dispatch.cpp
  - tests/firmware/native/test_fault_machine.cpp
  - tests/firmware/native/test_leg_interp.cpp
  - tests/firmware/native/test_udp_link.cpp
  - tests/firmware/native/fake_hal.cpp
  - tests/firmware/native/fake_hal.h
  - tests/firmware/native/hal_shims/QNEthernet.h
  - tests/firmware/test_armed_gate_xref.py
  - tests/firmware/test_sched_held_offset_twin.py
  - tests/firmware/test_bridge_fw_version_xref.py
  - tests/firmware/test_rpc_dispatch_lint.py
  - tests/motion/test_bench_fw52_armed_gate.py
  - tests/motion/test_hand_stream_bench_stall.py
  - tests/ros/test_teensy_bridge_node_armed_gate.py
---

# BUG-99 FW 52: Armed reconfiguration gate, per-axis clear scope, held clock offset, Jetson-only UDP

Owner decisions of 2026-11-25 on the FW 49 review of BUG-99 (rows 1, 3, 4a, 4b and A3; row 2, the
time-base slew, stays as it is). Built on FW 50 (observability, pins) and FW 51 (ACTIVATE_TO), its own
version. **Built, not flashed; the branch merges only with its flash** (the host now expects FW 52).

## Discussion (the control and safety walk, written before the code)

### D1 (row 4a): reconfiguration RPCs refused while Armed

*Failure prevented.* While the stream is live, SET_ABSOLUTE_POSITION on a Leg re-zeroes the encoder
under a running position loop: the next Setpoint is up to about 1 rev (70 mm) from the new encoder
reading and the Leg travels until MAX_DEVIATION trips. A Leg SET_POS_GAIN breaks the compiled
Kp x MAX_LEAD_REV headroom (A2) at runtime. SET_CONTROLLER_MODE / SET_AXIS_STATE CLOSED_LOOP
re-entries reset the ODrive's input filter mid-stream. SDO_WRITE writes any endpoint.

*Rule.* One predicate, `Rpc::refused_while_armed(method, axis, requested_state)`, called at one
point, `send_axis_frame`, which every per-axis ODrive RPC passes through. While `fault_mpc_active()`
it refuses (ERR_REJECTED, nothing on CAN): SET_ABSOLUTE_POSITION, SDO_WRITE, SET_CONTROLLER_MODE,
SET_VEL_GAINS (all axes), SET_POS_GAIN on a Leg, SET_AXIS_STATE to anything but IDLE. Still allowed:
CLEAR_ERRORS (`/recover` clears while Armed), SET_VEL_CURR_LIMITS (the Hand jam relief and the GUI
limits), SET_AXIS_STATE IDLE (a stop), SDO_READ.
The Hand SET_POS_GAIN stays allowed by the letter of the decision (the headroom invariant is a Leg
one); the host refuses `/set_hand_gains` whole while Armed, because SET_VEL_GAINS in the same call is
refused and a half-applied gain pair is worse than none.

*REBOOT_ODRIVES (orchestrator decision, 2026-11-25, the D1 intent).* A reboot drops a live
position loop mid-stream, so it is refused while Armed too, per axis and for the AXIS_ALL broadcast.
The broadcast loops `can_jugglebot_tx` itself and never reaches `send_axis_frame`, so its branch asks
the same predicate (`refused_while_armed(method, AXIS_ALL, 0)`): two call sites of one predicate, pinned
by `tests/firmware/test_armed_gate_xref.py`. Host call sites (grep `REBOOT_ODRIVES|reboot_odrives|
encode_reboot`): only `/reboot_odrives` and `odrive_command 'reboot_odrives'`, both through
`_reboot_odrives`; the orchestrator never sends it, no GUI page calls it, and both ODrive save-and-reboot
flows (`tools/odrive_fleet_reflash.py`, `tests/hardware/bench_hand_frame_bench.py`) reboot over USB, not
the bridge. `_reboot_odrives` now refuses while Armed BEFORE the RPC and before its cold-start clear, so
a refused reboot no longer drops is_homed / levelling for a reboot that never happened.

*The arming order.* `_arm_setpoint_output_locked` sends Hand CLOSED_LOOP and POSITION/PASSTHROUGH
BEFORE `_set_mpc_active(True)`; the firmware's `s_mpc_active` follows the J2T heartbeat bit 0, so at
the preamble it is still 0 and the gate passes. A gate keyed on "frames flowing" would refuse it,
because the pre-check streams before arming. The native test replays that order (frames staged and
promoted, CLOSED_LOOP and mode accepted, then mpc_active raised, then the same RPCs refused). One host
race is new: a disarm is staged until the next 10 Hz heartbeat, so an arm or a gain write within
~100-200 ms of a disarm would meet the stale bit 0. The arming preamble and the bench tool now wait for
the firmware's T2J MPC_ACTIVE bit to clear first (`_wait_wire_disarmed`, the wait DEACTIVATE already
uses), so a fast re-arm is delayed by at most one heartbeat instead of refused.

*No motion path changes.* Setpoints, the cold-start ops (HOME/ACTIVATE/DEACTIVATE, which do not route
through `send_axis_frame`) and the guard are untouched. One control cycle is unchanged; only the RPC
reply differs.

### D2 (row 4b): only an all-axes CLEAR_ERRORS releases the global latches

*Failure prevented.* A per-axis clear (encoder search, the sysid bench) silently released the Soft
E-stop latch, the Hand stall latch, the CAN_BUS_DOWN identity and every axis's disarm cache. The output
then re-enabled through the recovery slew on an acknowledgement that named one axis.

*Rule.* `fault_notify_clear_errors()` (global) is called only from the AXIS_ALL branch; a per-axis
clear that reached the wire calls the new `fault_notify_clear_axis_errors(axis)`, which zeroes that
axis's `disarm_reason` cache only (the stale-cache re-fault the global clear also guards against) and
releases nothing global, and does not refill the soft-reset budget. `s_fatal_error` is still
re-derived by `evaluate_errors` once every axis is clean, as before.

*Host.* Encoder search is disarmed by contract and never relied on the release (its clears target
the axis it searches). `bench_leg_sysid._clear_errors` now sends AXIS_ALL: on a one-Leg bench the
absent axes make the firmware answer ERR_TIMEOUT while the frame for axis 0 went out and the latch is
released (`any` is the notify condition), and `rpc.call` raises `RpcError` for any non-OK status, so
the tool treats ERR_TIMEOUT from the all-axes clear as done (logged) and re-raises everything else.
`lead_clamp_gap_bench` inherits it.

*Bench tools checked against D1 (fw52-verify).* `teensy_setpoint_bench.py` configures before it arms
and only sends IDLE after the disarm: clean. `hand_stream_bench.py`'s stream stages bring up before
arming and send nothing refused after: clean. Its `--stage stall` adapter was not: `_finish` restores the
encoder frame with SET_ABSOLUTE_POSITION right after `disarm()`, which only staged flags=0, so the
restore could meet the bridge's stale Armed bit and be refused (loud: "re-home before anything else",
but a spurious failure). `_StallBenchIO.disarm` now waits (bounded 1 s) for the bridge's T2J MPC_ACTIVE
bit to clear, the same wait as `bench_leg_sysid` and DEACTIVATE.

### D3 (row 1): scheduled playback keeps its clock through a sync loss; a legacy-to-scheduled entry is a resume

*Failure prevented.* `time_synced()` falls 90 s after the last anchor. Every later HAS_SCHED frame was
demoted to legacy arrival playback, which puts u0 at arrival instead of at its stamp: a step of about
v x frame lead (FW 50 pin: -1.71 rev on the Hand at -71.6 rev/s). The way back, a legacy lane promoted
to a scheduled group while the output is on, had no continuity check either.

*Rule, part 1 (net task, `interp_on_setpoint`).* A full HAS_SCHED frame is scheduled when
`time_synced()`; otherwise it stays scheduled on the held offset when the output is enabled AND a
scheduled group (Legs or Hand) is active AND the anchor is younger than
`TIME_OFFSET_HOLD_MAX_US` = 180 s (90 s past the loss). Counted in a console-only
`sched_held_offset` count. Past the bound, with a group active and output enabled, the frame is
DROPPED (counted with the out-of-range stamps in `sched_future_drops`, so the FEAT-41 ledger still
closes): the scheduled group runs its own C2 stop at cover exhaustion and SETPOINT_STALE trips the
Soft E-stop 250 ms later, which is loud. With the output disabled, or no scheduled group active, the
frame is demoted to legacy exactly as before (no motion is at stake: nothing reaches the wire, and the
next arm edge enters through the recovery slew); the FW 50 first-frame demotion test still holds.

*Drift bound.* The held offset is the last slewed offset; the bridge crystal against the Jetson clock
has measured a few ppm (-4 to -13 ppm, `logbook/2026-10-10-mocap-min-latency-clock-sync.md`).
At a 20 ppm design bound the mapping drifts at most 20 us per second, continuously (no step),
3.6 ms at the 180 s end, against a 50 ms frame cover. What ends the hold: a fresh anchor (normal; the
correction then goes through the existing `diff>>4` slew branch, at most 0.23 ms per anchor at 3.6 ms
error, inside row 2's documented 1.25 ms bound), the stream ending, or the bound (drop, stop, Soft
E-stop).

*Rule, part 2 (ISR, `sched_apply`).* When a frame would promote into an INACTIVE group whose lane is
playing legacy and the output is enabled, the new curve at `now` is compared with the legacy command
recorded on the previous tick (pre-clamp, pre-slew, extrapolated by p + v dt) against the RESUME
position and velocity tolerances (legacy carries no reliable acceleration). Continuous: accepted,
counted as today. Discontinuous: the group enters a C2 stop from the legacy state (p, v, a = 0), LATCHED,
the refused frame becomes the base command (so the guards see the plan walk away), `s_sched_refused`
counts it, and later frames meet the existing latched-hold rule until the output is disabled. While a
recovery slew is running the entry is accepted: the slew already owns continuity. A non-finite legacy
record skips the check (that lane is already being replaced by feedback).

*Twin.* `teensy_interp.py` now mirrors part 2 as well (`_sched_apply`; the legacy record in `tick` and
`tick_hand` at the firmware's places; `recover_slewing` caller-driven like `hand_recover_slewing`), so the
hermite cross-reference and the replay probes cannot read a legacy -> scheduled entry as accepted where
the firmware refuses it.

*One control cycle (2 ms tick) with the change.* Net task: frame arrives unsynced, Legs group active,
output on, anchor 100 s old: mapped with the held offset, queued. ISR: `sched_tick_begin` promotes it
into the active group through the unchanged PLAY continuity path (dp/dv/da counted); the Hand and Legs
evaluate the plan at its stamp; the lead clamp and guards are unchanged. No demotion, so no step. At
the entry case: tick n-1 recorded legacy p, v; tick n promotes, compares, and either plays the frame
or stops from (p, v): position and velocity are continuous either way; acceleration may jump to the
C2 stop's entry value, bounded by the stop's own A/J limits.

### D4 (row 3): I-FW-10 scoped, the Legs unchanged

Changing legacy Mode 1 to the Hand-style decay moves a Leg further (about 0.28 rev at 3.5 rev/s) than
today's bounded hold (|v1|/Kp <= 3.5/40 = 0.0875 rev), and drags the twin and `motor_guard` with it.
D3 also removes the production path into legacy playback. The invariant row now says "never a held
velocity" holds on the scheduled path and for legacy frames without u1, names the bounded legacy
Mode-1 hold, and names the FW 50 pin as its test.

### D5 (A3): UDP from any host but the Jetson is dropped and counted

*Rule.* `drain_socket` compares `remoteIP()` with `kJetsonIP` (192.168.42.1) inside the existing
NetLock read and drops a foreign frame before decode, counting it in the uplinked `crc_errors`
("dropped at the link") AND in a console-only `foreign_src` count with the last foreign address on the
`[diag]` line. No wire change.

*Who talks to the bridge.* The bridge link is a /30 on the Jetson's eth0 (192.168.42.1 <-> .2); the
kernel route pins the source address to .1 for every Jetson process (`teensy_bridge_node`,
`tools/teensy_link_bridge.py`, the bench tools, the probes); no tool binds a different source. The
native shim's `remoteIP()` returns .1.

*Failure mode if the Jetson's address changes or it sends from a second address.* The bridge already
sends every uplink frame to .1 only, so a Jetson without .1 sees no telemetry at all today (loud).
The new silent-looking case is a Jetson that holds .1 (so it still receives the uplink) but sends from
another source address: every downlink frame is dropped. Then the J2T heartbeat never lands, the link
watchdog reports LINK_LOST, RPCs time out, the arm refuses, and the uplinked `dn_crc_errors` climbs at
the full downlink rate while the bridge console prints `foreign_src=N last=a.b.c.d`. The robot ignores
its host, but nothing moves: every motion path needs the heartbeat.

## Fix

See `files_changed`. FW_VERSION 52, EXPECTED_BRIDGE_FW_VERSION 52, PROTOCOL_VERSION stays 14.

## Verification

Native binaries (`python tests/firmware/native/build.py`, then each `temp/firmware_native/test_*` run
directly; built and run with `nice -n 10` WITHOUT the box test lock, which other sessions held for
the whole unit: CPU-light and not wall-clock sensitive), 2026-11-25 16:19: **all 22 binaries pass**:
test_rpc_dispatch 42/42 cases (444 assertions), test_fault_machine 35/35 (818), test_leg_interp 80/80
(10693), test_udp_link 6/6 (79), test_leg_activate 53/53, test_leg_deactivate 14/14, test_leg_homing
19/19, test_platform_relay 32/32, test_version_check 14/14, test_gpio_poll 20/20, test_can_tx_result
4/4, test_udp_framing 6/6, test_odrive_protocol 3/3, test_ball_butler_protocol 4/4, and the eight
Platform binaries (imu, imu_service, imu_diag, imu_fw14, sync_rx, loop_service, hand_fan, icm42688).

What the first native run found (16:00, all test defects, none in the firmware):
- `test_rpc_dispatch`: the new REBOOT case did not compile (`NUM_AXES` ambiguous between JbUdp and
  CanBridge); qualified.
- `test_fault_machine` D2 subcase: the all-axes clear did not release because the setpoint was still
  stale, so SETPOINT_STALE re-tripped at once. Now the stream returns first (as the Gap 5 subcase does),
  so only the latch holds the E-stop when the per-axis clears are tried.
- `test_leg_interp` D3: the step meter sampled the reset target 0.0 before the Hand lane was live (a
  fake 5 rev "step"); the entry case also measured the legacy lane's own frame handovers (FW 21 arrival
  playback restarts at u0 at an arrival off the tick grid: up to 0.069 rev second difference at
  ~68 rev/s), which is pre-FW 52 behaviour D3 retires, not the entry. Both scoped.

The D3 thresholds are now DERIVED, not reasoned (comment above `fw52_after_sync_loss`): the command must
stay C1, so per tick |dp| <= V dt and |p[n] - 2p[n-1] + p[n-2]| <= A dt^2. Plan V = 71.62 rev/s, A = 1500:
held offset measured 0.143199 rev vs V dt = 0.143239, second difference 0.00597 vs 0.0060 (an exact
replay). Through the grace and the stop A = SCHED_STOP_ACCEL_HAND_RPS2 = 4500 (0.018 rev): drop case
0.00678, entry case 0.0108 (refused at 176 ms from v = 56 rev/s, HOLD at 208 ms); the drop case's |v|
bound adds the grace's growth and the jerk-limited stop's overshoot. The refusal count is exact (3:
frames 7, 8, 9 are the scheduled frames due in the window; the entry, then two latched-hold
refusals), as are held = 8 and dropped = 6. The FW 51 demotion step these replace was 1.71 rev.

Mutation evidence (each on an isolated copy of the sources, `$S/out/fw52-verify/mutate.py`, restored
and diffed clean; the Python ones in the worktree, restored, `md5sum` equal):

| Mutation | Fails |
|---|---|
| D1: drop the predicate call at `send_axis_frame` | test_rpc_dispatch: the D1 table case, the arming-order case, the REBOOT case |
| D1-reboot: drop REBOOT_ODRIVES from the predicate | test_rpc_dispatch: the REBOOT case |
| D1 host: `_reboot_odrives` without its refusal | test_teensy_bridge_node_armed_gate: the reboot case |
| D2: per-axis clear calls the global release | test_rpc_dispatch: the D2 case and the stale-bus CLEAR case |
| D2 (fault machine): `fault_notify_clear_axis_errors` also releases | test_fault_machine: the D2 subcase |
| D3a: demote on any sync loss (FW 51) | test_leg_interp: held-offset case, drop case |
| D3b: skip the legacy -> scheduled entry check | test_leg_interp: the entry case |
| D3b twin: skip the twin's entry check | test_sched_held_offset_twin: the discontinuous-entry case |
| D5: drop the source check | test_udp_link: the D5 case |
| bench: `_StallBenchIO.disarm` without the wait | test_hand_stream_bench_stall: the FW 52 restore case |

Python:
- 2026-11-25 15:56-16:12, `flock /tmp/jugglebot-tests.lock nice -n 10 python -m pytest -q -p no:cacheprovider -n 4 --dist loadfile tests/teensy_link tests/ros/test_teensy_bridge_node_*.py tests/motion/test_bench_fw52_armed_gate.py tests/motion/test_bench_sysid_bridge.py tests/motion/test_bench_sysid_logic.py tests/motion/test_hand_stream_bench_order.py tests/motion/test_hand_stream_bench_stall.py tests/motion/test_hand_stream_bench_stroke_cut.py tests/firmware --ignore=tests/firmware/test_native_firmware.py`: 2082 passed, 4 skipped, 3 failed, all in `test_rpc_dispatch_lint.py`: its case-body scan took the first `case RpcMethod::REBOOT_ODRIVES:`, now the predicate's label; it now prefers the braced dispatch label. Then 16:19, `nice -n 10 python -m pytest -q -p no:cacheprovider tests/firmware/test_native_firmware.py tests/firmware/test_rpc_dispatch_lint.py` (no lock: native wrapper): 33 passed.
- 2026-11-25 15:47, `python -m pytest -q tests/firmware/test_armed_gate_xref.py tests/ros/test_teensy_bridge_node_armed_gate.py tests/motion/test_hand_stream_bench_stall.py tests/firmware/test_sched_held_offset_twin.py`: 67 passed.

Firmware image: 2026-11-25 16:07, `pio run -e teensy41` (BUILD ONLY, never uploaded): SUCCESS; `arm-none-eabi-size firmware.elf`: text 259392, data 35520, bss 113440 (dec 408352); firmware.hex sha256 70696bd6cde321f4...

Status stays in-progress until FW 52 is flashed.

Integration gate (the FW 50 to 52 chain rebased onto main c21bd54e, tip f5dc77c4):
- 2026-11-25 20:27, `flock /tmp/jugglebot-tests.lock ./run_tests.sh --full`: 11306 passed, 57 skipped, 2 xfailed in 830 s; serial phase 16 passed; exit 0.
- 2026-11-25 21:10, after the document audit added the I-FW-18 row, the ARMING_CONTRACT A5 sentence and corrected one comment in `canbridge_config.h` (no code change), `python -m pytest -q -n 4 --dist loadfile -m "not nightly" tests/firmware tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_logbook_index.py tests/sim/test_logbook_front_matter.py tests/ros/test_state_machine.py`: 581 passed, 4 skipped.
