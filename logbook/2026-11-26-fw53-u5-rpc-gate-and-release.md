---
title: "FW 53 unit 5: the can-bridge RPC layer sends every Leg SET_AXIS_STATE, REBOOT_ODRIVES and SDO_WRITE through the Stow gate (ERR_REJECTED, nothing on CAN, unless the Platform is at Stow or the rig is the one-Leg bench), and FW_VERSION goes 52 to 53 with the Diagnostic LEG_HELD and RIG_BENCH flag bits and the host twin classifiers"
type: feature
date: 2026-11-26
status: in-progress
phase: "FW 53 U5"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/gpio_poll.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_stow_gate.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - teensy_link/rpc_args.py
  - teensy_link/activate.py
  - tests/firmware/native/test_rpc_dispatch.cpp
  - tests/firmware/native/test_gpio_poll.cpp
  - tests/firmware/native/build.py
  - tests/firmware/test_stow_gate_xref.py
  - tests/firmware/test_armed_gate_xref.py
  - tests/firmware/test_bridge_fw_version_xref.py
  - tests/firmware/test_leg_deenergise_lint.py
  - logbook/2026-11-26-fw53-u5-rpc-gate-and-release.md
subsystem: [can-bridge, safety]
---

## What changed and why

Owner rule, 2026-11-25: "A Leg should never be made IDLE unless the Platform is already at the Stow position." The Hand is exempt, and so is the one-Leg bench. Units 1 to 4 moved the ladders, Homing and the deferred stow behind the gate (`leg_stow_gate`). U5 moves the last callers, the RPC layer, and makes FW 53 a release.

- **`rpc.cpp`**: `send_axis_frame` is split. `axis_precheck` (axis range, Hand allow-table, Armed gate, bus gate) is shared; `send_axis_frame` is the ungated remainder (gains, modes, absolute position, clear, SDO read). `gate_status` maps the gate's `SENT/REFUSED/TX_FAILED` to `OK/ERR_REJECTED/ERR_TIMEOUT` (no new status code: an FW 52 host's `RpcStatus(status).name` must not raise during skew).
  - SET_AXIS_STATE, per-axis REBOOT_ODRIVES and SDO_WRITE go through `odrive_set_state_tx / odrive_reboot_tx / odrive_sdo_write_tx` (`Who::RPC`). A Leg IDLE while Armed passes the Armed gate and is then refused by the Stow gate.
  - REBOOT AXIS_ALL asks `requires_platform_at_stow` and `leg_idle_permitted(micros64())` once, after the Armed and bus gates and before any frame or the watchdog-suppression latch: refused whole, no Hand frame.
  - Leg CLOSED_LOOP and any Leg SDO_WRITE are refused while `homing_active()` (an SDO write can set `requested_state`; the gate alone passes it when every Leg is IDLE).
  - The BUG-47 homing-abort IDLE (`homing_axis_in_flight() == axis`) is the single `HOMING_OWNER` use in `rpc.cpp`.
  - CLEAR_ERRORS(AXIS_ALL) also calls `leg_held_clear_all()`, only once a frame reached the bus.
- **`gpio_poll.cpp`**: the Hand ball-sensor function-invoke goes through `odrive_sdo_write_tx` (Hand pass-through; AWAIT on anything but TX_FAILED keeps "DEFERRED is in flight"). The lint allowlist is now empty and `test_allowlist_is_empty` pins it.
- **`telemetry.cpp` / `leg_stow_gate.h`**: Diagnostic flags bit4 `DIAG_FLAG_LEG_HELD` 0x10 (per Leg, from `leg_held_mask()`), bit5 `DIAG_FLAG_RIG_BENCH` 0x20 (every axis), in the existing flags byte; a flip of either is news (sent at once, not at the 1 Hz refresh). No struct or MsgType change, PROTOCOL_VERSION stays 14. The host reads the byte with masks only (`test_the_host_masks_unknown_diagnostic_bits`); the two host constants are in `teensy_link/activate.py`.
- **Version**: `FW_VERSION` 53, with a `// 52→53:` line summarising U1 to U5; `EXPECTED_BRIDGE_FW_VERSION` 53. The twin classifiers `requires_platform_at_stow(method, axis, requested_state=None)` and `home_requires_all_legs_idle(axis)` mirror firmware classifiers of the same name.

One RPC walked: `SET_AXIS_STATE(Leg 2, IDLE)`, Platform posed (Leg 3 at 2.2 rev). `dispatch` decodes, `axis_precheck` passes (Armed gate allows IDLE, bus up), the homing carve-out does not apply, `odrive_set_state_tx(2, IDLE, Who::RPC)` asks `leg_idle_permitted`, Leg 3 refuses, the gate returns REFUSED, nothing is queued, the census counts it, no HELD bit (an RPC refusal is the host's answer), the host receives ERR_REJECTED. At Stow the same call sends the byte-exact frame and returns OK.

Decisions: (1) the classifier is asked in `rpc.cpp` only for the AXIS_ALL reboot; per-axis requests are decided by the wrapper (single enforcement point), so the classifier's per-axis rows are pinned by the xref test and the native classifier case, not by an extra dispatch check that would double-count refusals. (2) A REFUSED mid-loop on REBOOT AXIS_ALL (state changed between the check and the loop) answers ERR_REJECTED only if nothing was sent, else ERR_TIMEOUT (partial).

## Verification

- 2026-11-26, `python $S/out/fw53-u5/quick.py <wt> <bin> - test_rpc_dispatch test_gpio_poll`: test_rpc_dispatch 53/53 cases (587 assertions; 42 cases before U5, all 42 unchanged), test_gpio_poll 20/20.
- 2026-11-26, `python $S/out/fw53-u5/mutate.py <wt>` (mutated copies of `rpc.cpp` shadow the tree via `-I`; table in `$S/out/fw53-u5/mutations.txt`): **20/20 caught** (raw SET_AXIS_STATE, raw reboot, raw SDO, lost BUG-47 carve-out, no homing CL/SDO refusals, no AXIS_ALL pre-check, latch armed before the gate, no bench exemption, HELD not cleared or cleared before the bus gate, REFUSED or TX_FAILED mis-mapped, four classifier rows, Armed pre-check skipped).
- 2026-11-26, six text-level mutations of the Python-side pins (twin REBOOT row, firmware SDO row, `platform_at_stow` in the reboot check, flag 0x40, allowlist entry, REFUSED mapping): each failed the named test in `test_stow_gate_xref.py` or `test_leg_deenergise_lint.py`. `telemetry.cpp` is not compiled natively: its flag wiring is pinned by text only (unchecked behaviourally).
- 2026-11-26, `flock /tmp/jugglebot-tests.lock nice -n 10 python -m pytest -q -p no:cacheprovider tests/firmware tests/teensy_link tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_no_legacy_item_ids.py`: **1248 passed, 4 skipped in 545.32 s** (includes the full native build and every native binary via `test_native_firmware.py`). `test_logbook_index.py` is not in the run: it stays red until the orchestrator runs `mapkit gen`.
- `pio run -e teensy41` (build only, never uploaded): see the unit report for the firmware.hex sha256.

Integration gate for the whole FW 53 chain (U1 to U9, tip 09669806, rebased on main ed2b0fd7): 2026-11-26 10:47, `flock /tmp/jugglebot-tests.lock ./run_tests.sh --full`: 11450 passed, 54 skipped, 2 xfailed in 786 s; serial phase 16 passed; exit 0. Independent audit of the diff the same morning: nothing blocking; its should-fix items are the audit follow-up sections of the U8 and U6b entries.
