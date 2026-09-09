---
title: "GUI: Hardware panel — board model + live firmware per device, grouped by (model, firmware) with the odd one out flagged"
type: feature
date: 2026-09-09
status: resolved
phase: "gui — operator tooling"
files_changed:
  - config/hardware_config.yaml
  - config/generate_config.py
  - config/generated/geometry-config.js
  - ros_ws/gui/js/hardware-versions.js
  - ros_ws/gui/js/geometry-config.js
  - ros_ws/gui/js/main.js
  - ros_ws/gui/js/panels.js
  - ros_ws/gui/index.html
  - ros_ws/gui/css/panels.css
  - tests/ros/test_gui_geometry.py
  - tools/probes/gui_dom_probe.py
  - tools/probes/gui_synthetic_stack.py
subsystem:
  - gui
  - config
---

# GUI: Hardware panel (board model + firmware version per device)

## Summary

Owner request: show every board's firmware version in the GUI, below the Event Log,
with each device's MODEL made clear, and same-type devices grouped — the odd one out
made obvious when they disagree.

New `#panel-hardware` renders one row per **(model, firmware)** pair over eleven
devices (2 Teensys + 9 ODrives). Seven agreeing ODrive Pros collapse to one
`ODrive Pro ×7 — L0–L5, Hand — 0.6.11` row; a leg on an older build splits onto its
own row with an amber ODD badge. `hardware-versions.js` is a new module in the
`can-traffic.js` / `udp-traffic.js` mould; `main.js` fans `link_status` to it.

**No node change was needed.** `/link_status` has carried `bridge_fw_version`,
`platform_fw_version` and `odrive_fw_versions` all along
(`teensy_bridge_node._publish_link_status`) — the GUI simply never read them.

Models come from a new `hardware_models` section in `hardware_config.yaml`,
generated into `geometry-config.js` as `HARDWARE_MODELS`.

## Discussion

**Why models are DECLARED, not read off the wire.** Neither Teensy reports its board
model: BridgeIdentity (0x8E) is `fw_version` + `protocol_version`, and the Platform's
0x6E0 reply carries `FW_VERSION` only. Adding a board byte to either is a wire change
→ `PROTOCOL_VERSION` bump → total link darkness → a lockstep flash sitting, which is a
wildly disproportionate price for a display label on a fact that changes only when
somebody physically swaps a board. Owner's call (2026-09-09): declare all of them in
`hardware_config.yaml`, informational only, nothing verifies them.

**Why `hardware_models` is not derived from `odrive_expected_versions`.** The product-line
byte there already encodes the model (4 = Pro, 5 = S1, 6 = Micro) and the two agree — but
that section is a validation gate with teeth: `motor_state.validate_group` latches
`firmware_validated`, and a mismatch holds the orchestrator out of BOOT. Wiring a cosmetic
string into it would hand a display label the power to stop the robot. The agreement is a
comment, not a contract.

**Why the two Ball Butler ODrives are listed as `n/a` rather than dropped.** They have no
firmware version on any wire — the can-bridge's Get_Version sweep covers the Jugglebot axes
only (`version_check.h`; the `GET_AXIS_VERSIONS` blob is a fixed 7-axis array) and BB's
heartbeat carries none. Widening the sweep is a wire change, same price as above. Dropping
the rows would make "not checked" indistinguishable from "not present" — the failure mode
every never-seen rendering in this codebase exists to prevent.

**Why absence is muted and never amber.** ODrive versions arrive on a bus-paced sweep (one
frame per cold-start tick), so unread axes are the NORMAL state for the first seconds of
every launch. Bucketing them as a mismatch would raise ODD on every launch, and a badge
that cries wolf at boot is a badge nobody reads at the sitting where it matters. MIXED is
computed over known versions only; absence gets its own muted bucket.

**Why this panel keeps its values while stale, unlike the two traffic panels.** Those blank
their readouts because a frozen *rate* is a live-looking lie. A firmware version is a
constant: the last read stays the best answer available, so the panel dims the column and
raises a STALE badge naming the cause instead of hiding the data.

**Consensus rule.** With one model group holding disagreeing versions, the strictly-largest
bucket is the consensus and the rest are flagged. On a genuine tie there is no consensus, so
*every* bucket is flagged — an even split is not a case where one half is quietly correct.

## Verification

- Node probe over 8 payload shapes (nominal, one-odd-leg, tie, dev-build, mid-sweep,
  never-seen, skew, pre-connection) — scratchpad, uncommitted.
- New pytest classes `TestHardwareModels` + `TestHardwareVersionKeyValueContract` pin the
  YAML→JS codegen, the DEVICES↔YAML key join, and the three `link_status` KeyValue names
  *and their renderer methods*. Mutation-checked 2026-09-09: renaming a consumed key,
  editing the YAML without regenerating, deleting an axis model, and changing the per-axis
  wire format each fail the intended test; all four reverted green.
- Live browser + real rosbridge, new `--scenario hardware` (5 stages, 9 assertions):
  `python3 tools/probes/gui_dom_probe.py --scenario hardware`, run 2026-09-09 —
  **PASS 9/9, 0 uncaught exceptions, 0 console.error**.
- Full gate: `./run_tests.sh --full`, run 2026-09-09 — **6990 passed + 4 serial, 4 skipped,
  2 xfailed, 478 s, RESULT: PASS**.

## Follow-up (pre-existing, NOT from this change)

`gui_dom_probe.py --scenario scenario1` fails 3/17 assertions
(`can3-shows-can1-slot-values`, `can2-na-and-disabled`, `health-dots-warn-ok-unknown`).
Confirmed pre-existing: it fails **identically at HEAD with this work stashed**. The
expectations predate the 2026-07-31 bus-role swap — they still assume wire slot `can1_*`
and `bus1_health` land on the CAN3 row, whereas `can-traffic.js`'s own `BUSES` registry
maps them to CAN2 ("Jugglebot core") since the loom moved off the faulty CAN3 drive path.
The GUI is right and the probe is stale; fixing it is three expectation updates in
`SCENARIO1`, deliberately left out of this commit so it is not buried inside a feature.
