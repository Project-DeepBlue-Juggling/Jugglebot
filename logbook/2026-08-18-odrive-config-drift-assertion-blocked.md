---
title: "ODrive config-drift assertion: the SDO read path is one-way, so the design is blocked on firmware"
type: investigation
date: 2026-08-18
status: resolved
related_plan: odrive-config-drift-assertion.md
files_changed:
  - plans/parked/odrive-config-drift-assertion.md
  - plans/parked/INDEX.md
  - teensy_link/rpc_args.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
subsystem:
  - can
  - ros
tags:
  - safety
  - docs
---

# ODrive config-drift assertion — blocked on firmware, and why

## Summary

Scoped a launch-time assertion that the live ODrives' safety registers still
match the committed snapshots in `config/ODrive config Files/` — the control that
would have caught the 2026-08-18 hand `torque_soft_min` clamp
(`-0.055133331567049026`, the **legs'** `torque_constant` pasted into a hand
clamp field, exactly −10.00 A) weeks earlier. **No code was written for it: the
transport it needs does not exist.**

Two firmware-resident blockers, both pinned by existing tests:

- **No SDO response reaches the Jetson.** `rpc.cpp:249-257` sends the `RxSdo`
  frame and returns an empty result blob — its own comment reads *"The TxSdo
  reply has NO return path to this RPC — nothing correlates it back to the
  caller."* `can_buses.cpp:248-262` consumes a TxSdo reply only for the hand's
  `get_gpio_states` (the ball sensor) and discards every other. No uplink frame
  carries an arbitrary SDO value.
- **The hand is refused `SDO_READ` outright.** It is absent from
  `hand_axis6_permitted` (`udp_protocol.h:593-608`), so axis 6 returns
  `ERR_REJECTED` before anything leaves the Teensy — pinned by
  `test_rpc_dispatch.cpp:187-190`, `test_platform_relay.cpp:284` and
  `test_hand_axis6_allow.py:52`. The one drive that actually drifted is the one
  the request cannot reach.

USB was checked as the alternative and **ruled out on evidence**: the drives are
operator-plugged for maintenance only (`lsusb -d 1209:` empty on this box today;
`docs/can_bridge/index.md:23-35` shows no USB leg), so a USB check would score
UNKNOWN on every launch. CAN3 is the only permanent path, which makes a
can-bridge firmware change the only route to an *automatic* control — and that
needs an operator flash nobody has authorised.

Landed instead: `plans/parked/odrive-config-drift-assertion.md` with the full
design (register set with per-register failure modes, verdict vocabulary where
UNKNOWN is never PASS, the float32-quantisation comparison rule, the round-trip
bound, and three costed options), plus a docstring fix on the two functions whose
wording invited the false premise in the first place.

## Discussion

**Why stop rather than build the USB version.** Reading over USB is genuinely
attractive — it needs no firmware, no endpoint registry, and it dissolves the
wrong-endpoint hazard entirely, because the `odrive` package resolves properties
by name. It was rejected on a *physical* fact rather than a design one: the
drives are not on USB while the robot runs, so the check would be structurally
incapable of ever returning an answer. Building it would have produced something
that looks like a control, passes its own tests, and reports UNKNOWN forever —
which is the same failure as the manual bench row H7.0c that this work exists to
replace, wearing better clothes. The distinction between "a check that can fail"
and "a check that can *pass*" is the one worth carrying forward.

**Why the tolerance in the original scope became a quantisation rule.** Every
value in both snapshots was written by `odrivetool backup-config` reading a
float32 and widening it, so it round-trips exactly — `values_equal` in
`tools/odrive_fleet_reflash.py` compares exactly and documents that "a tolerance
would hide a failed write". The exception is a **hand-edited** snapshot: today's
fix writes `0.7`, and `float32(0.7) = 0.699999988079071`. So the rule that
survives both cases is quantise-expected-to-float32-then-compare-exactly, not a
declared tolerance — same robustness against hand edits, without opening a band
that could mask real drift.

## Verification

`./run_tests.sh` (run 2026-08-18): **5245 passed in 238 s**, RESULT: PASS.
Scoped, each result with the command that produced it:
`pytest tests/firmware/test_config_drift.py -q` (run 2026-08-18) — **21 passed**;
`pytest tests/sim/test_plans_index.py -q` (run 2026-08-18) — **67 passed**,
pinning the new parked plan's INDEX row and its parked note.

No behavioural change: the two edits are docstrings, and no test asserts either
string (`grep -rn "response returns async" tests/` returns nothing).

**One transient RED worth recording, because it was not a regression.** An
earlier gate run the same evening reported 9 failures, all in
`tests/firmware/test_config_drift.py`. Cause: a parallel session edited
`config/hardware_config.yaml` and `config/generate_config.py` (mtime 20:26:07)
and rewrote `config/generated/*` (20:26:13) **while that run was in flight**
(~20:24→20:30), so the drift test compared a half-updated artifact set. Scoped
re-run immediately afterwards: 21/21 pass. The working tree grew from 6 to 32
modified files during the session. A config-regeneration race in a shared working
tree looks exactly like a real drift failure — check artifact mtimes against the
run window before believing one.
