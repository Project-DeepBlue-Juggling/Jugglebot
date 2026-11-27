---
title: "The Armed refusal text says the rule dates from can-bridge FW 52, not that the bridge is FW 52"
type: bugfix
date: 2026-11-27
status: complete
phase: "Sitting debrief 2026-11-27"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - teensy_link/rpc_args.py
subsystem: [can-bridge, ros]
---

## What changed

Two operator-facing strings read "The can-bridge (FW 52, BUG-99) refuses ...". They now read
"(since FW 52, BUG-99)": the host's `reboot_odrives REFUSED: the robot is Armed` reply in
`teensy_bridge_node.py` and `ARMED_REFUSAL_HINT` in `teensy_link/rpc_args.py`. Text only; no
refusal, gate or wire byte changes.

## Why

At the Sitting of 2026-11-27 the bridge ran FW 53 and the launch log printed the refusal with
"FW 52" in it (launch 1, 13:47:52). The number names the firmware that introduced the Armed
gate (I-FW-18), but it reads as the version on the board, which the same log's
`BRIDGE_FW_CHECK` line contradicts.

## Verification

No test pins either string (2026-11-27, `grep -rn "FW 52, BUG-99" tests`: no match). The gate
run that covers this change is cited in the debrief entry, `2026-11-27-sitting-debrief`.
