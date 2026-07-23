---
title: BB telemetry restored to the GUI charts — 9-axis robot_state + bus V/I on the Diagnostic wire (UDP protocol v4)
type: feature
date: 2026-07-24
status: in-progress
phase: "MVP trajectory bringup — GUI observability"
related_plan: ""
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/docs/can-node-teensy-parity.md
  - ros_ws/gui/js/main.js
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/ros/test_teensy_bridge_node_bb.py
  - tests/ros/test_teensy_bridge_node_read.py
  - tests/teensy_link/test_protocol_codec.py
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
commits:
  - b3c82f7   # feat(telemetry): 9-axis robot_state + v4 Diagnostic bus V/I
  - 563e28c   # fix(gui): clear BB fault dots on the 9->7 shrink
  - d9effff   # docs(logbook): this entry + INDEX row
subsystem:
  - ros
  - can
  - gui
  - config
tags:
  - telemetry
  - testing
---

# BB telemetry restored to the GUI charts — 9-axis robot_state + bus V/I on the Diagnostic wire (UDP protocol v4)

## Summary

The GUI's BB Pitch / BB Hand charts (and BB fault dots) had been silently dead
since the can-bridge cutover: they read `robot_state.motor_states[7]/[8]`, but
`teensy_bridge_node` publishes only the 7 CAN3 axes the can_node deletion left
it. This change restores the can_node-era 9-axis `motor_states` layout —
appending BB pitch/hand entries sourced from the BB DIAGNOSTIC stash + the
100 Hz `BB_AXIS_ESTIMATES` sample, gated to honest silence when BB is dark —
and adds DC **bus_current** to the Diagnostic wire frame (UDP
PROTOCOL_VERSION 3→4), which also revives the legs'/hand's silently-empty
"Bus I" chart signal. Firmware FW_VERSION 2→3, flashed and live-verified.

## Motivation

Operator report (2026-07-23): BB data stopped appearing in the GUI charts.
Investigation traced it structurally: the charts' only feed for stores 7/8 is
`robot_state`, whose publisher moved from can_node (9 axes, `ALL_AXES`) to
teensy_bridge_node (7 axes, `p.NUM_AXES`) at the cutover (~2026-07-02;
can_node deleted in `7c7f61b`). BB telemetry still flowed — on
`bb/axis_estimates` + `bb/odrive_diag`, which the GUI never subscribes to.
The operator additionally wanted BB bus voltage AND current as feedback;
bus_current was not carried by the DIAGNOSTIC frame for ANY axis (a
documented parity-ledger drop), so protocol + firmware work was unavoidable
under any fix shape.

## Design

Three layers, single-source-generated where possible:

1. **Protocol** (`config/generate_udp_protocol.py`, PROTOCOL_VERSION 3→4 —
   incompatible wire change, both sides drop mismatched-version frames at
   parse): DIAGNOSTIC grows 36→40 B with `bus_current` (f32), and `flags`
   gains bit1 `heartbeat_seen` (bit0 stays `heartbeat_stale`). Regenerated
   into `config/generated/udp_protocol.{h,py}`, the firmware header, the
   profiling-probe copy, and `docs/teensy-udp-protocol.md`.
2. **Firmware** (`telemetry.cpp`): both diag-build sites (`send_diag` axes
   0–6, `send_bb_diag` axes 7/8) fill `bus_current` and the heartbeat_seen
   bit. The firmware already decoded + cached `Get_Bus_Voltage_Current` on
   both CAN3 and CAN1 RX paths (`AxisState.bus_current`) — the value simply
   died at the uplink boundary until now. FW_VERSION 2→3.
3. **Bridge node** (`teensy_bridge_node.py`): `_build_motor_states` copies
   `bus_current` for the 7 platform axes; new `_build_bb_motor_states`
   appends `[7]`=BB pitch, `[8]`=BB hand — pos/vel from the latest
   `BB_AXIS_ESTIMATES` sample (100 Hz, same cadence as the publish timer),
   everything else (state/errors/Iq/temps/bus V+I) from the BB DIAGNOSTIC
   stash (fixed 1 Hz per axis). New RX-thread stashes (`_latest_diag_mono`,
   `_latest_bb_est(+_mono)`) written and snapshotted under the existing
   `self._lock`.

**Gating (all-or-nothing, honest silence)** — BB entries are appended only
when, for BOTH axes: a diag exists, its `heartbeat_seen` bit is set, its
`heartbeat_stale` bit is clear, the diag stash is <3 s old, and the estimate
sample is <0.5 s old. Otherwise robot_state stays 7 axes.

The GUI needed **no changes to revive**: charts, fault dots, and CSV export
come back the moment `motor_states` grows to 9 (they always indexed
positionally). One GUI fix WAS needed for a newly-reachable path — see
Implementation.

## Discussion

**Why restore the 9-axis robot_state instead of subscribing the GUI to
`bb/axis_estimates` + `bb/odrive_diag`?** The 9-axis layout *was* the
codebase contract — the chart stores, "CAN motor 7/8" labels, BB fault dots,
CSV export, and `MotorStateSingle` (which already carried `bus_voltage` AND
`bus_current` fields) were all built against it; the 7-axis narrowing was a
documented handoff compromise, not a design goal. The GUI-side alternative
would have added a second, parallel chart-feed path with its own staleness
plumbing, extended `bb/odrive_diag`'s untyped magic-index
`Float32MultiArray` layout (the direction the codebase has been moving away
from), and *still* required the protocol/firmware work for bus_current —
which, once done at the DIAGNOSTIC layer, fixes all nine axes at once
(the legs' "Bus I" chart signal had been silently dead since the cutover
too). Every robot_state consumer was audited for length assumptions
(orchestrator guards `len >= JUGGLEBOT_AXES`; motion_bridge slices `[:6]`;
trajectory_node is defensive): all were written in the can_node 9-axis era
and survive the extension untouched.

**Why all-or-nothing on the two BB entries?** The GUI indexes positionally:
a lone surviving hand entry would land at index 7 and be read as the pitch
motor by every consumer — a *mislabeled* axis is strictly worse than a
missing one. Both BB ODrives share one power rail and one CAN segment;
half-alive is a fault state, and an ODrive with an ACTIVE error still
heartbeats (so real faults stay visible in the appended entries — only
CAN-level death drops them).

**Why a `heartbeat_seen` wire bit instead of the `axis_state != 0`
convention?** A dark/absent BB axis still produces zero-filled 1 Hz diag
frames; without an explicit discriminator those decode as a plausible
"live at zero" motor. `axis_state != 0` would have worked (it is the
orchestrator's heartbeat convention) but rides an ODrive-enum coincidence;
since the version bump was already being paid for bus_current, the explicit
bit was free and self-documenting. Live-verified both ways on the bench:
legs report `seen=True`, dark BB axes report `seen=False`.

**Why does bus_current get NO change-detect threshold?** It rides along
like `iq_measured`: `iq_setpoint` (>0.5 A) is its load proxy, and the 1 Hz
forced refresh covers slow drift. A dedicated threshold would only add diag
traffic under load for no fidelity the charts can use.

**Honest silence over frozen flatlines** (the `leg_setpoint_echo`
discipline): a BB death mid-session drops the entries rather than freezing
their last values — a chart gap tells the truth; a flatline lies.

## Implementation

- `config/generate_udp_protocol.py`: PROTOCOL_VERSION 4, DIAGNOSTIC
  `bus_current` + flags bit1 + axis_id doc 0..8; artifacts regenerated.
- `telemetry.cpp`: both diag sites fill the new field/bit;
  `canbridge_config.h` FW_VERSION 3.
- `teensy_bridge_node.py`: `_build_bb_motor_states` + gating constants
  (`_BB_DIAG_FRESH_S`=3.0, `_BB_EST_FRESH_S`=0.5, flag masks, BB axis ids
  from `protocol_config`); `_build_motor_states` copies bus_current;
  RX stashes + locked snapshot in `_publish_robot_state`.
- Version-freeze contract tests re-pinned (`test_udp_protocol_xlang.py`
  pin 3→4 + new layout hash; `test_protocol_codec.py` pin 3→4) — the freeze
  worked exactly as designed, refusing the layout change until deliberately
  re-pinned.
- 7 new tests in `test_teensy_bridge_node_bb.py` (happy path with value
  round-trip, phantom never-seen, stale-heartbeat drop, stale-estimates
  drop, stale-diag-stash drop, all-or-nothing half-alive, BB-fault-visible-
  but-not-leg-fatal), `bus_current` asserts in
  `test_teensy_bridge_node_read.py`.
- **Opus review findings (both LOW, fixed same-session)**: (1) parity-ledger
  rows 61/436/453 in `ros_ws/docs/can-node-teensy-parity.md` still listed
  bus_current as dropped — amended with CLOSED annotations; (2) GUI fault
  dots could freeze red when the bridge shrinks 9→7 on a BB death
  (`main.js` fault loop never revisits absent indices) — explicit clear
  added on the shrink path. Review verdict: SHIP, no BLOCKING/HIGH/MEDIUM;
  concurrency, positional integrity, fault-flag isolation, and codegen
  reproducibility all verified clean.

## Verification

- Full suite (`pytest tests/ -q`, run 2026-07-24): **2954 passed, 1 failed,
  1 xfailed in 939.02 s** — the single failure is
  `test_t3b_h4_on_post_solve_allocates_within_budget`, the documented
  load-flaky allocation-budget test; isolated rerun
  (`pytest tests/sim/test_mpc_time_pathologies.py::TestT3bH4PostSolveAllocation -q`,
  2026-07-24): **1 passed in 7.25 s**.
- Targeted (`pytest tests/ros/test_teensy_bridge_node_bb.py
  tests/ros/test_teensy_bridge_node_read.py
  tests/firmware/test_udp_protocol_xlang.py tests/teensy_link/ -q`,
  2026-07-24): **282 passed** (after the codec-pin update).
- Firmware: `pio run` SUCCESS both envs; `pio run -e teensy41 -t upload`
  SUCCESS 2026-07-24 (can-hub Teensy over USB, HalfKay auto-entered); serial
  boot banner `jugglebot-canbridge v3`.
- **Live wire verification** (read-only listener probe on :5005, ROS stack
  down, 6 s window): PROTOCOL_VERSION=4 frames decode clean — 600 Telemetry
  + 599 BbAxisEstimates (the expected 100 Hz each), 0 undecodable; all
  DIAGNOSTIC payloads 40 B; legs/hand report `seen=True stale=False` with
  live Iq/temps (main DC bus off on the bench → busV≈0.04 V, correct); dark
  BB axes 7/8 arrive `seen=False` all-zero — the exact phantom case the
  append gate refuses.
- colcon build of `jugglebot` SUCCESS (installed copy updated).
- **Remaining to flip this entry to `resolved`** (needs a powered-BB
  sitting): with BB on, confirm robot_state carries 9 axes and the GUI's BB
  Pitch/BB Hand charts render pos/vel/Iq/temps/bus V+I live; confirm the
  charts empty again (and fault dots clear) when BB is switched off.

## Related

- `logbook/2026-07-10-s4-stutter-guard-forensics-recovery-stack.md` —
  previous UDP protocol bump (v2→v3, at FW_VERSION 1); FW_VERSION 1→2 was
  the 2026-07-16 guard raise
  (`2026-07-16-max-deviation-guard-tracking-lag.md`).
- `ros_ws/docs/can-node-teensy-parity.md` rows 61/436/453 (bus_current
  CLOSED annotations added by this change).
- GUI feed path: `ros_ws/gui/js/main.js` (`onRobotState` →
  `onTelemetryData`), `ros_ws/gui/js/telemetry-charts.js` stores 7/8.
