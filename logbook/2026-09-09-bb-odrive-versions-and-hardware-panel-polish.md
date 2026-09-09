---
title: "Ball Butler ODrive versions reach the Jetson at last (can-bridge FW 20), plus Hardware-panel alignment/contrast and the stale scenario1 bus-role expectations"
type: feature
date: 2026-09-09
status: resolved
phase: "gui — operator tooling / can-bridge FW 20"
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.py
  - config/generated/udp_protocol.h
  - config/hardware_config.yaml
  - ros_ws/src/jugglebot/Teensy_code_canbridge/version_check.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/version_check.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - teensy_link/rpc_args.py
  - teensy_link/protocol.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/gui/js/hardware-versions.js
  - ros_ws/gui/css/panels.css
  - tests/firmware/native/test_version_check.cpp
  - tests/firmware/native/test_rpc_dispatch.cpp
  - tests/firmware/native/fake_hal.cpp
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/ros/test_gui_geometry.py
  - tools/probes/gui_dom_probe.py
  - tools/probes/gui_synthetic_stack.py
subsystem:
  - gui
  - canbridge
  - config
related_entries:
  - 2026-09-09-gui-hardware-versions-panel.md
---

# BB ODrive versions (can-bridge FW 20) + Hardware-panel polish

## Summary

Four owner follow-ups to the Hardware panel that landed earlier today.

1. **Ball Butler ODrive versions now exist.** New additive RPC
   `GET_BB_AXIS_VERSIONS` (0x5A) + `ResultBbAxisVersions`, a BB-relative version
   cache and a bus-paced CAN1 Get_Version sweep on the cold-start monitor task;
   `decode_bb_odrive` gains its `get_version` case. Host side pulls it on a 1 Hz
   timer, runs the existing `validate_group(BB_AXES, "Ball Butler")` and
   publishes `bb_odrive_fw_versions` on `/link_status`. **can-bridge FW 19 → 20,
   BUILT NOT FLASHED.**
2. **Device column alignment.** Every row's Device cell now starts at the same x.
3. **Contrast.** Nothing in the panel body sits at `--text-muted` any more.
4. **`gui_dom_probe --scenario scenario1`** repointed off its pre-2026-07-31
   bus-role expectations; now 17/17 (was 14/17, and had been red since July).

## Discussion

**The owner's premise was nearly right, and the "nearly" cost a firmware
change.** The ask was "this comes through to the ROS2 launch, so it can't be
that hard to plumb in". It *did* come through — under `can_node`, over the
USB-CAN adapter that has since been physically removed. Commit `5875531` dropped
it in the BB cutover and said so explicitly: *"phase B will restore BB ODrive
validation by decoding axes 7+8 on CAN1 and surfacing the result via
teensy_bridge_node (a new T2J flag or RPC)"*. The RX decode landed; the version
half never did. So this is finishing a known-deferred phase, not new ground —
but it is firmware, and no amount of host plumbing could have substituted.

**`SDO_READ` looked like a free ride and is not.** It already exists and takes an
arbitrary axis. But `rpc.cpp` is explicit: *"The TxSdo reply has NO return path
to this RPC — nothing correlates it back to the caller."* Fire-only. Checked
before committing to firmware, precisely because a no-firmware route would have
been worth a lot.

**Why a new method rather than widening `ResultAxisVersions`.** The obvious move
is to grow the existing blob from 7 axes to 9. It is also the expensive one:
that blob is a fixed `u8[NUM_AXES*8]` whose decode is an exact-size unpack on
both ends, so growing it is an *incompatible* wire change — `PROTOCOL_VERSION`
bump, total link darkness, lockstep flash — to add two display rows and a
validation nothing gates on. A second method behind the existing `RpcRequest`
envelope costs none of that: `PROTOCOL_VERSION` stays 6, an FW 19 board is
wire-identical, and it answers the unknown id with `ERR_UNKNOWN_METHOD`. That is
the `HAND_SOURCE_SET` / `PLATFORM_FW_*` precedent, followed deliberately.

**The BB verdict is advisory and must stay that way.** `_firmware_validated` and
`_firmware_mismatch_error` hold the orchestrator out of BOOT. The Ball Butler is
optional hardware on a different bus — a missing or mismatched BB must not stop
the robot juggling — so `_bb_version_check_poll` touches neither, logs a
`warning` rather than an `error` on mismatch, and is pinned by
`test_bb_verdict_never_gates_boot`. `can_node` drew the same line; losing it
would have been invisible until the day a BB was unplugged.

**Three kinds of "no version" that must not collapse into each other.** A
pre-FW-20 bridge cannot be *asked* (`unsupported (bridge FW < 20)`); a swept axis
that has not answered *has* been asked (`7:?`); and a bridge that never sent
BRIDGE_IDENTITY is a third thing again (`unknown (never seen)`). Rendering the
first as "unknown" would blame the Ball Butler for an old bridge. The panel shows
the verdict *word* with the full sentence in the tooltip — the same split the
SKEW chip already used.

**The alignment bug was invisible in the DOM and needed a measurement.** The
header and each row are *separate* grid containers, so they share no track
sizing: a content-sized `auto` column resolved differently per row and slid the
Device column left and right down the list. Two decorations made it worse — the
row's 2 px horizontal padding (absent on the header) and the ODD row's
`border-left`, which occupies layout space no compensating padding can restore.
Fixes: a fixed first track, matching header padding, and an inset box-shadow
instead of a border. The pin is a `getBoundingClientRect` assertion that every
row's Device cell shares one x and every Firmware cell one right edge — run
twice, once with an ODD row on screen. Nothing weaker could have caught it.

**Contrast: the model is equally certain on every row.** It is config, not
telemetry. Dimming a Ball Butler row's *model* implied uncertainty about the
board, of which there is none — the uncertainty is about the version, and the
italic version cell already carries it. So `.hwver-model` is primary everywhere
and `.hwver-row-muted` no longer recolours anything. `×N`, the trailing note
(`proto 6`) and the absence values moved `--text-muted` → `--text-secondary`;
at `#64748b` on the dark ground they read as disabled rather than secondary.
Column headers stay muted, matching `.can-rows-head`.

**scenario1 was measuring the robot as it was wired in July.** It asserted wire
slot `can1_*` onto the CAN3 row and `bus1_health` onto CAN3's dot. Since
2026-07-31 the loom runs on the CAN2 controller (the bridge's CAN3 analog drive
path has a load-dependent fault) and the cone gained a traffic slot, so the GUI
was right and the probe was three years-stale-in-robot-time. Rather than just
repointing, all three slots now carry distinct fixture values and all three
health dots distinct states, so the mapping is pinned three ways instead of two.

## Verification

- Native firmware, direct compile of the real `version_check.cpp` (2026-09-09):
  **14 cases / 75 assertions pass**, 7 of them new BB cases (one-frame-per-tick
  pacing, absent axes never queried, the CAN1 partner-presence gate, absolute→
  BB-relative id conversion, out-of-range rejection, blob cap, and the two
  sweeps' independence in both directions).
  Mutation-checked: an *aliasing* record (`axis % NUM_BB_AXES`) fails 3 cases and
  a missing `BB_FIRST_NODE` subtraction fails 2; restored code green. The first
  version of that test caught neither — it asserted only the mask, so it now
  seeds both slots and requires the seeds to survive.
- `pytest tests/firmware/test_native_firmware.py -q` (2026-09-09): **18 passed
  in 198.09 s**.
- Real target build: `pio run -e teensy41` (2026-09-09) — **SUCCESS in 10.59 s**,
  238912 B text. This is the only compile `can_buses.cpp`, `rpc.cpp` and the
  `.ino` get; the native harness does not build them. **BUILT, NOT FLASHED.**
- `pytest tests/ros/test_gui_geometry.py -q` (2026-09-09): **135 passed**.
  `test_bb_verdict_never_gates_boot` mutation-checked (assigning
  `_firmware_validated` in the BB poll fails it; reverted green).
- Live browser + real rosbridge: `gui_dom_probe --scenario hardware`
  (2026-09-09) **PASS 12/12**, and `--scenario scenario1` **PASS 17/17**
  (was 14/17, pre-existing).
- Full gate: `./run_tests.sh --full`, run 2026-09-09 — see Outcome below.

## Outcome

`bb_odrive_fw_versions` renders `unsupported (bridge FW < 20)` until the board is
reflashed, and the panel shows that verdict verbatim — so the row is honest
about *why* it is empty rather than looking like two silent drives. Nothing else
changes before the flash: `PROTOCOL_VERSION` stays 6, so an FW 19 board and this
host tree are wire-identical in both directions.

**Operator action to light it up:** `pio run -e teensy41 -t upload` from
`ros_ws/src/jugglebot/Teensy_code_canbridge/`. The boot banner is the only
receipt; `/link_status`'s `bridge_fw_version` should then read `20 (proto 6)`
with no SKEW, and `bb_odrive_fw_versions` should fill in within a few seconds of
the Ball Butler heartbeating.
