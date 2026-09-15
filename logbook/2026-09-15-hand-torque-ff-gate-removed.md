---
title: "Hand torque-FF readback gate removed — ACTIVATE never ran it"
type: bugfix
date: 2026-09-15
status: done
phase: "two-ball-skill-stack — R3"
related_plan: two-ball-skill-stack.md
files_changed:
  - teensy_link/setpoint_pump.py (drop hand_torque_scale_verified ctor arg/attr/setter; hand_ff_gain now forced 0 only on a legs-only frame)
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py (drop the verified flag, readback thread, WARN-once latch, and their /link_status row; _svc_set_hand_state reverts to a plain set-state service)
  - tests/ros/test_teensy_bridge_node_hand_torque_ff.py (delete the gate tests, adapt the default/requested/effective ones)
  - tests/teensy_link/test_setpoint_pump.py (drop the 3 set_hand_torque_scale_verified calls)
  - tests/hardware/session_skills_r3_apex_ladder.md (drop readback expectations; row 29 confirms input_torque_scale by hand)
  - plans/active/two-ball-skill-stack.md
  - logbook/2026-09-15-hand-c2-stream-and-torque-ff.md (superseded note on the readback-gate paragraph)
---

## What / Why

The 2026-09-15 hand torque-FF landing
([2026-09-15-hand-c2-stream-and-torque-ff](2026-09-15-hand-c2-stream-and-torque-ff.md))
gated the wire gain `hand_ff_gain` on a fail-safe: `SetpointPump` forced K to
0 on the wire unless `hand_torque_scale_verified` was True, and
`teensy_bridge_node` set that flag from a readback of the hand ODrive's
`can.input_torque_scale` (`GET_HAND_TORQUE_SCALE` RPC), started from
`_svc_set_hand_state` on a successful CLOSED_LOOP.

At today's sitting the gate never armed: ACTIVATE brings the hand up through
a *different* path (`teensy_bridge_node.py`'s arming sequence calls
`teensy_set_axis_state(_HAND_AXIS, CLOSED_LOOP)` directly), never
`_svc_set_hand_state`, so the readback thread never started and
`ros2 param set hand_torque_ff_gain 0.7` was silently forced to 0 on the
wire for the whole session — arm B ran at arm A's gain with no error, no
warning (the WARN-once latch lives behind the same never-reached setter),
and no visible symptom on `/link_status` beyond a requested/effective
mismatch nobody was watching for.

The owner confirmed the hand ODrive's `input_torque_scale=1000` (persists
across power cycles, re-verified via the ODrive GUI) and decided: remove the
gate rather than re-wire the readback into ACTIVATE's path. The wire gain
now follows the `hand_torque_ff_gain` param directly — validated (finite,
`0 <= K <= HAND_FF_GAIN_MAX`) as before, just no longer additionally forced
to 0 pending a readback. The `GET_HAND_TORQUE_SCALE` RPC
(`RpcClient.read_hand_input_torque_scale`) is untouched and stays available
as a manual diagnostic; the host no longer calls it or gates on it.

## Verification

(2026-09-15, `./run_tests.sh --full`, log `temp/logs/gate_removal_full_20260915.log`):
**PASS — parallel 6086 passed, 9 skipped, 2 xfailed in 300.19 s; serial 6
passed in 18.92 s** (the full tier is the pre-sitting gate; the next sitting
re-flies the apex ladder A/B on this tree).

(2026-09-15, `source ~/Desktop/PDJ_venv/venv/bin/activate && pytest
tests/ros/test_teensy_bridge_node_hand_torque_ff.py
tests/teensy_link/test_setpoint_pump.py
tests/ros/test_unified_cycle_integration.py tests/sim/test_skills_gate.py
tests/ros/test_teensy_bridge_node_*.py -q`): **620 passed, 1 xfailed
(pre-existing, unrelated) in 157.10 s**.

(2026-09-15, `pytest tests/sim/test_plans_index.py
tests/sim/test_logbook_front_matter.py -q`): **83 passed in 0.46 s**
(covers this entry's frontmatter and the `INDEX.md` row).

Grep for the removed symbols
(`grep -rnE "hand_torque_scale_verified|_hand_tscale|_maybe_warn_hand_ff_unverified|_hand_ff_unverified_warned|_start_hand_torque_scale_readback|_set_hand_torque_scale_verified" --include='*.py' --include='*.md' --include='*.yaml' .`)
returns zero hits outside `temp/`, `.git/`, `plans/archived/`, this entry,
and the superseded-note reference in the 2026-09-15 C2FF entry.
