---
title: Clapboard Phase 3 — the SetSlate action, on the bridge node that already owns the link
type: feature
date: 2026-08-16
status: resolved
phase: "clapboard-can3-integration Phase 3"
related_plan: clapboard-can3-integration.md
files_changed:
  - ros_ws/src/jugglebot_interfaces/action/SetSlate.action
  - ros_ws/src/jugglebot_interfaces/CMakeLists.txt
  - ros_ws/src/jugglebot/jugglebot/clapboard_slate.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/docs/choreography.md
  - tests/ros/conftest.py
  - tests/ros/test_clapboard_slate.py
  - tests/ros/test_teensy_bridge_node_set_slate.py
  - tests/ros/test_launch_nodes.py
subsystem:
  - ros2
  - can-bridge
  - testing
tags:
  - clapboard
  - protocol
  - action
---

# Clapboard Phase 3 — the SetSlate action

## Summary

`clapboard/set_slate` is live on `teensy_bridge_node`: a goal names a template
and a patch of field values, the action chunks them into `CLAP_FIELD` frames,
CRCs the padded buffers, appends `CLAP_COMMIT` **last**, dispatches the whole
transaction in one `CLAP_SEND` RPC, and blocks on the panel's own `CLAP_ACK`
before reporting anything. All the wire semantics live in a new pure-Python
`jugglebot/clapboard_slate.py`; the node does dispatch, correlation and
reporting only.

Also landed here because this is the first phase with a consumer: `CLAP_DIAG`
(0x93) is subscribed, published on `/clap_diag`, and folded into every failing
`SetSlate` result — it is the only thing that separates *the bridge never put
the frames on the wire* from *the panel mis-reassembled them* behind a
`CRC_MISMATCH`.

Nothing here can fire the flash. The clapboard accepts no fire command on
either transport, by design.

## Discussion

**Parallel `field_ids` / `field_values` arrays, not the handoff doc's
`string[8] fields`.** The suggested interface uses a fixed-size array with
"empty string = leave unchanged". Two things break under it. First, patch
semantics then have no way to *clear* a field — empty means "unchanged", so a
stale note line can never be blanked. Second, and decisive: with a fixed
8-element array an out-of-range `field_id` is structurally impossible, so the
plan's own T-U7 ("`field_id > 7` rejected") would be a test of nothing. Explicit
ids make the mask explicit, make clearing expressible, make the CLI ergonomic
(`{field_ids: [1], field_values: ['TAKE 13']}` rather than eight positional
strings), and give the validation boundary something real to guard.

**`outcome` is a string, not the wire `uint8`.** The obvious shape — mirror
`CLAP_ACK`'s outcome byte, as `BallButlerThrowCmd` mirrors its firmware enum —
runs into a wall on the host-side failures. TIMEOUT, INVALID_GOAL and
DISPATCH_FAILED never come from the panel, and there is no value in that byte to
give them: `ClapboardAckOutcome` is the *clapboard repo's* enum to allocate, and
minting a host-only `0x07` inside it collides the moment they allocate one. A
string carries both vocabularies without either side owning the other's, and
`tests/ros/test_clapboard_slate.py` pins them disjoint so a future peer
allocation named `TIMEOUT` reddens the suite instead of quietly overloading a
word. `render_ms` stays a real `uint16` — it is a measurement, not a verdict.

**Every present field sends all five chunks, even a two-character value.** This
looks like lazy padding and is actually a correctness requirement. Panel field
buffers PERSIST between transactions (that is what §8.4's patch semantics mean),
and the CRC is over the full 32-byte NUL-padded buffer. Sending only the
occupied prefix would leave the *previous* value's tail sitting in the
receiver's buffer beyond what we wrote — which both renders wrong text and
CRC-mismatches against a sender that padded with NULs. So a one-field update is
6 frames and a full 8-field slate is 41, always, and the worst case is what
`CLAP_MAX_FRAMES = 48` was sized from.

**Validation is ordered ahead of the presence check, and a malformed goal is
ACCEPTED rather than rejected.** This is the one place the implementation
deviates from the plan's wording, and it is the same root cause Phase 2's
self-audit found in the firmware: if the bus gate is evaluated before the
arguments, a malformed request on a quiet bus comes back `ERR_BUS_DOWN` and
sends an operator hunting a wiring fault for a typo. Here the analogue is
"no clapboard attached" answering a `field_id` of 9. So `goal_callback` runs
validation first — but a `GoalResponse.REJECT` carries no payload, so rejecting
there would tell the caller nothing about *which* field was wrong. The split
that satisfies both: **transient/environmental conditions REJECT** (not
attached, already in flight — retry later is the right response and there is
nothing more to say), **caller errors ACCEPT and then abort with the exact
offending index and value**. T-I11's case — a valid goal while disconnected —
still rejects in `goal_callback` with no RPC issued, exactly as specified.

The cost is a coupling: an invalid goal reaches `execute_callback` *without*
having claimed the single-flight slot, so that path must not release one. The
structure guarantees it (validation re-runs before the `try`/`finally` is
entered) and a test pins it — an invalid goal arriving while a real push is in
flight must not cancel the real one. Because that guarantee rests on both calls
reaching the same verdict, `validate_slate`'s docstring now states that it must
stay a pure function of its arguments; a future rule that consulted node state
could disagree between the two calls and release a slot it never took.

**No `RENDERING` feedback phase.** The handoff doc suggests
`"sending" | "awaiting_ack" | "rendering"`, but the host cannot observe the
drain-to-render transition: the RPC acks the *enqueue*, the frames then leave
the bridge one per tick, and the next thing the Jetson hears is the ack that
ends the goal. A phase string the node cannot honestly time would be decoration.
Two phases, `SENDING` and `AWAITING_ACK`, and the reason for the absent third is
written into the `.action`.

**`/clap_diag`'s WARN is keyed on a delta, uniquely among its rendered fields.**
The counters are cumulative since bridge boot and are published raw, matching
`can3_errors`/`cache_diag` — a node-side delta in the *values* would silently
widen across a dropped frame. But the *level* keyed on the cumulative total
would latch WARN for the rest of a session after one entirely legitimate loss
(a slate pushed before the clapboard's first heartbeat opened the TX gate), and
a permanently-yellow row teaches the operator to ignore it — the same trap
`_publish_cache_diag` avoids by qualifying its ages with `seen_mask`. So WARN
means "losing frames NOW", the first sample after launch is an explicit baseline
at OK (losses tallied before this node started are not this session's), and
`lost_since_previous` is rendered so a bag reader can see the level's own input
without differencing four columns by hand.

**A latent cross-thread window, closed here and left open next door.** The
CLAP_ACK handshake originally stashed the ack under `_clap_slate_lock` and set
the wake event *outside* it, copying bb/throw. That admits an interleave: the RX
thread stashes, is preempted before `set()`, the goal times out, a later goal
re-arms (clearing the stash), and the stale `set()` then wakes the new goal with
no ack to read — an `AttributeError` raised inside an action callback, i.e. a
goal that never terminates. It needs a multi-second preemption so it is
vanishingly unlikely, but `Event.set()` cannot re-enter that lock, so setting it
inside costs nothing and removes the class. A `None` guard after the wait backs
it up, because the two halves of the handshake live 4000 lines apart.
**`bb/throw` has the identical shape at `_on_cmd_result`** (stash under lock,
`set()` outside, `_bb_throw_result` cleared by a later `goal_callback`) and
would fail as a `TypeError` unpacking `None`. Not touched — out of this phase's
scope, and recorded here rather than fixed silently.

## Fix

- `SetSlate.action` — goal (`template_id`, parallel `field_ids`/`field_values`,
  `force_full_refresh`), result (`success`, string `outcome`, `render_ms`,
  `message`), feedback (`phase`), registered in `jugglebot_interfaces`.
- `jugglebot/clapboard_slate.py` — new, no ROS imports: boundary validation,
  32-byte NUL-padded field buffers, `crc16_over_fields` (reusing the repo's one
  CRC-16/CCITT-FALSE), 5-chunk splitting with the `(field_id | seq<<4)` nibble
  head, the commit builder, and a thread-safe `TxnIdAllocator` that never
  allocates 0 — a zero-filled frame decodes as "txn 0, outcome OK" and could
  otherwise report a phantom success on a panel that never painted.
- `teensy_bridge_node` — ActionServer on a dedicated `ReentrantCallbackGroup`,
  single-flight goal gate, `CancelResponse.REJECT`, 8 s ack wait with no retry,
  txn_id correlation extending the Phase-1 `CLAP_ACK` decode point rather than
  adding a second, `_clapboard_present()`, and the `CLAP_DIAG` subscribe /
  `/clap_diag` publish / result-message fold.
- Launch bag list + `RECORDED_TOPICS`: the two action channels and `/clap_diag`.

## Verification

- Scoped, first: `pytest tests/ros/test_clapboard_slate.py
  tests/ros/test_teensy_bridge_node_set_slate.py -q` (run 2026-08-16):
  **71 passed in 9.45 s**.
- T-R1 holds: `tests/ros/test_teensy_bridge_node_cone.py` and
  `test_teensy_bridge_node_clapboard.py` are **unmodified** by this phase
  (`git status --short` lists neither) and pass inside the gate below.
- Interfaces + package: `colcon build --packages-select jugglebot_interfaces`
  then `jugglebot` (run 2026-08-16): both **finished**. The generated IDL was
  then read back and compared field-for-field against the conftest mock —
  `sequence<uint8>` / `sequence<string>` / `uint16` / strings — and a real
  `SetSlate.Goal` driven end-to-end through `build_transaction`, because
  `field_ids` arrives as an `array.array` under the real interface and a `list`
  under the mock.
- Gate: `./run_tests.sh --full` (run 2026-08-16): **RESULT: PASS** — parallel
  phase **5849 passed, 3 xfailed in 489.48 s**; serial phase **9 passed,
  5852 deselected, in 40.94 s**; total 536 s.
- The pre-audit run of the same command reported one failure,
  `tests/ros/test_trajectory_tilt_map.py::test_reload_does_not_reframe_a_live_plan`,
  which **did not reproduce** in the post-audit run. Diagnosed rather than
  assumed: that test seeds `robot_state` once and then makes two `build_move`
  planning passes, and `_svc_go_to_pose` refuses on `_robot_state_fresh()` once
  the seed is older than the node's `robot_state_stale_s` = 0.5 s. Scoped, the
  whole 42-test file runs in 2.86 s; under four xdist workers on this box those
  planning passes can exceed the window. Same wall-clock-versus-contention
  mechanism as the known `test_decay_boundary_continuity` flake, in an unrelated
  node this phase does not touch. Re-run scoped 3x after the failure:
  `pytest tests/ros/test_trajectory_tilt_map.py -q` (run 2026-08-16):
  **42 passed** each time.

## Open Questions

- **The clapboard cannot answer a slate push yet.** Its `can.cpp:189-192`
  counts `CLAP_FIELD`/`CLAP_COMMIT` as received and drops them — *"Phase 16
  consumes these"* — so there is no reassembler and no `CLAP_ACK` source on the
  device. Everything here is exercised against a synthetic ack. **T-H6 (SetSlate
  end-to-end on the panel) is blocked on the clapboard repo's Phase 16**, and
  the bench sitting should expect `rx_frames` to climb with the panel unchanged.
  T-H1..T-H5 and T-H7..T-H9 are unaffected.
- Two unstated-but-enforced rules in the peer's `protocol.md` remain
  (strict DLC-8 on receive; the 48-bit fire timestamp's reconstruction), carried
  forward from Phases 0 and 1 for Phase 4's shared fixture.
- The `PROTOCOL_VERSION` question is carried forward unresolved for a third
  phase: the handoff doc §4 says bump, the plan declines, and §8 — the normative
  half — is silent because it governs CAN frames only.
- `bb/throw`'s identical set-outside-the-lock window, above.
