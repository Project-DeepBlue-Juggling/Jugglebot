---
title: Clapboard Phase 1 — the uplink becomes observable in ROS2 with no firmware change
type: feature
date: 2026-08-16
status: resolved
phase: "clapboard-can3-integration Phase 1"
related_plan: clapboard-can3-integration.md
files_changed:
  - config/protocol_config.yaml
  - config/generate_config.py
  - ros_ws/src/jugglebot/jugglebot/can/clapboard.py
  - ros_ws/src/jugglebot/jugglebot/can/__init__.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot_interfaces/msg/ClapboardHeartbeat.msg
  - ros_ws/src/jugglebot_interfaces/msg/ClapboardFireEvent.msg
  - ros_ws/src/jugglebot_interfaces/CMakeLists.txt
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/docs/choreography.md
  - tests/ros/conftest.py
  - tests/ros/test_clapboard.py
  - tests/ros/test_teensy_bridge_node_clapboard.py
  - tests/ros/test_launch_nodes.py
subsystem:
  - can-bridge
  - ros2
  - testing
tags:
  - clapboard
  - protocol
---

# Clapboard Phase 1 — uplink observability, Jetson-only

**What/why.** A plugged-in electronic clapboard is now visible in ROS2 on
`/clapboard/heartbeat` (10 Hz state + presence) and `/clapboard/fire_event` (one
per sync flash, with a real `header.stamp`), with **no firmware change, no
reflash and no `PROTOCOL_VERSION` bump**. That is possible because the
can-bridge's `on_cone_rx()` has no arbitration-id filter — it rings and relays
every frame on the cone bus verbatim as a `CONE_FRAME` — so clapboard frames
already arrived at the Jetson and were discarded by the dispatcher's
silent-drop fall-through. Making them useful was a pure `elif` extension, with
the clapboard branches placed ahead of the cone ones and the cone path left
alone.

New: a `can_ids.clapboard` block (0x7E8–0x7EF) and a top-level `clapboard:`
section in `protocol_config.yaml`; `jugglebot/can/clapboard.py` (pure decoder,
ids from the generated constants); two interface messages; the dispatch,
queues, publishers, timers and drains on `teensy_bridge_node`; both topics in
the launch bag list and in `RECORDED_TOPICS`; a regenerated `choreography.md`;
and two test files (77 new tests).

## Discussion

**The `protocol_config.yaml` trap is real, and no test can see it.** That YAML
has no `HW_SECTIONS`-style registration table — every section is a hard-coded
block in *both* `generate_cpp` and `generate_python`. A new key emits absolutely
nothing in either language until both blocks are hand-written, and
`test_config_drift` compares generator output against disk, so with the emitters
missing both sides agree the constants are absent and the gate stays green. The
only defence is to open the generated artifacts and look. Both were written and
both outputs verified (`ClapboardCanId`, `Clapboard`, `ClapboardState`,
`ClapboardAckOutcome` in the header; `CAN_ID_CLAP_*`, `CLAP_HEARTBEAT_TIMEOUT_MS`,
`ClapboardStates`, `ClapboardAckOutcome` in the Python) before anything imported
them.

**`CLAP_FIRE_EVENT` does not carry a wall clock that fits in its field, and the
spec does not say so.** `protocol.md` §8.8 reads as though it does: *"48 bits of
microseconds covers ~8.9 years — ample."* But the clapboard's `wall_us()` is
Unix-epoch microseconds (its 0x7DD anchor is `unix_s * 1e6 + usec`), and a 2026
Unix-µs value needs **51 bits** — 1.79e15 against a 48-bit ceiling of 2.81e14.
`can_frames.h::encode_fire_event` therefore masks with `0x0000FFFFFFFFFFFF` and
puts the **low 48 bits** on the wire. "~8.9 years" is the wrap period, not the
range. A decoder that took the field at face value would stamp every flash
somewhere in 1978 — a confidently wrong timestamp in the one record the device
exists to produce, and it would look plausible in isolation. So
`reconstruct_fire_time_us()` rebuilds the high bits against the host clock, the
48-bit sibling of `catching_cone.reconstruct_catch_time_us`, with a ±2^47 µs
(~4.46 year) window. This was found by reading the peer's code, not its prose;
the reconstruction rule is stated nowhere. It is the same class of trap as the
strict DLC-8 receive rule the plan already flags — enforced in `can_frames.h`,
absent from `protocol.md` — and both are worth raising with the clapboard repo.

**Unknown enum values pass through instead of raising.** `ConeHeartbeat` coerces
its state byte to `CatchingConeStates(...)`, which raises on an unrecognised
value. Inherited here that would mean: unknown state → `ValueError` → the RX
callback's broad `except` drops the frame → no heartbeat for 500 ms → the node
publishes `connected=False`. The clapboard would be reported **gone** because it
reported a state this host predates. Given the handoff doc's own standard —
*"a field that silently reports the wrong peripheral is worse than one that
reports nothing, because it will be believed"* — the asymmetry is decisive: a
raw `state: 9` is uninformative, `connected: false` is wrong. `state` and
`outcome` therefore decode as plain ints with `state_name` / `outcome_name`
rendering `UNKNOWN(n)`. The cone's strictness is left alone; this is a deviation,
recorded, not a refactor of the sibling.

**Downlink-only ids are counted on the RX thread and reported once from the
timer.** The handoff doc says 0x7E8–0x7EA arriving inbound should be "logged once
and dropped". But `teensy_bridge_node.py:2227-2235` records an explicit contract:
`_record_bridge_fw_version` is *the only* rclpy log call from the RX thread in
this node, carved out because it is announce-on-change, and the reason the rule
exists is that a malformed-frame storm must never become a logging storm on the
frame-receive path. A once-only flag would satisfy the letter of "logged once"
while putting a logger call on the hot path for the general case. The counter +
publish-timer report keeps the contract intact and says the same thing.

**A stashed but unconsumed `CLAP_ACK` is a deliberate cost.** Nothing can provoke
an ack until the downlink firmware exists (Phase 2), so the decode is dead code
for now. It lands anyway so the frame has exactly ONE decode point that the
Phase 3 `SetSlate` action extends rather than duplicates — the alternative is a
second decoder written against the same bytes six weeks later.

## Verification

- Scoped, first: `pytest tests/ros/test_clapboard.py
  tests/ros/test_teensy_bridge_node_clapboard.py
  tests/ros/test_teensy_bridge_node_cone.py -q` (run 2026-08-16):
  **87 passed in 6.33 s** — and the cone file is **unmodified** (T-R1, the
  byte-identical proof for the cone branch).
- Codegen: `python config/generate_config.py` (run 2026-08-16): 16 artifacts
  written (14 in-repo + the 2 external BallButler copies); clapboard constants
  confirmed present in `config/generated/protocol_config.h` and `.py` **by
  inspection, not by inference** — that is the only defence against this
  section's silent-emitter trap. Then `python config/generate_config.py --check`
  (run 2026-08-16): **CONFIG FRESH: 14 artifact(s) match the generator**.
- Interfaces + package: `colcon build --packages-select jugglebot_interfaces`
  then `jugglebot` (run 2026-08-16): both **finished**, and
  `_clapboard_heartbeat.py` / `_clapboard_fire_event.py` exist under
  `install/jugglebot_interfaces/`.
- Firmware unchanged: `pio run -e teensy41` in
  `ros_ws/src/jugglebot/Teensy_code_canbridge` (run 2026-08-16): SUCCESS,
  text 232768 / data 35520 / bss 107872, `firmware.hex` md5
  `ea705b4bb4026047318c0361750c87ab` — **byte-identical** to the Phase 0 build,
  so adopting the generated `ClapboardCanId` bounds implies no reflash. (The
  native harness's cache key hashes every firmware `.h` including the delivered
  `protocol_config.h`, so the gate genuinely recompiled and re-ran the
  `is_clapboard_id` truth table rather than reusing a stale binary.)
- Gate: `./run_tests.sh --full` (run 2026-08-16): **RESULT: PASS** — parallel
  phase **5746 passed, 3 xfailed in 487.21 s**; serial phase **9 passed,
  5749 deselected, in 41.08 s**; total 534 s. +79 collected over the Phase 0
  tree (5667): 77 new clapboard tests + 2 new `RECORDED_TOPICS` parametrisations.
  Run three times across the phase (implementation, post-audit, final
  pre-commit); PASS with identical counts each time.

## Open Questions

- **Two unstated-but-enforced rules in the peer's `protocol.md`**, both found in
  `can_frames.h`: the 48-bit fire timestamp is the *low* 48 bits of a 51-bit
  clock (§8.8 implies otherwise), and every decoder requires `len == 8` exactly
  (§8 never says so). Worth a note to the clapboard repo; Phase 4's cross-repo
  fixture is the natural place to pin both.
- `generate_config.py` still has only `_check_catching_cone_key_collision` — a
  section-specific guard against the two YAMLs reopening one C++ namespace. The
  new `clapboard` section is single-sourced today (no `clapboard:` in
  `hardware_config.yaml`, no `HW_SECTIONS` row), so the collision is not
  reachable, and generalising the guard is a refactor of shared codegen rather
  than Phase 1 work. Noted because the failure it prevents is a `constexpr`
  redefinition — loud at compile time, but only for whoever builds firmware next.
- The generator delivered into `../BallButler`, which was **already** carrying
  pre-existing external drift in `hardware_config.h` (catch/toss tuning values
  from earlier sittings: `CATCH_VEL_SCALE_DEFAULT` 0.8→0.9,
  `TOSS_REQUIRE_BALL_EVIDENCE` false→true, a new `TOSS_WORKSPACE_XY_MM`). That is
  a separate checkout on its own branch and is not committed here; it is called
  out so the next BB session knows the delivery happened.
- The plan's §6 files-affected table is **systematically one phase high for
  everything after Phase 2** — a leftover from the draft where the firmware was
  two phases. It lists the doc sweep under "7" (the plan has 0–6),
  `SetSlate.action` and `clapboard_slate.py` under 4 (§4 puts them in Phase 3),
  `teensy_bridge_node.py` under "1,2,4" and `conftest.py` under "1,4". §4's
  headings are the authority. Reported, deliberately not fixed here: renumbering
  a whole table is a plan edit past this phase's scope.
- Still open from Phase 0: six messages with no round-trip test
  (`CmdResultFrame`, `BbAxisEstimates`, `LegCmd`, `ClockDiag`, `CacheDiag`,
  `RingDiag`); `CLAP_DIAG` in Phase 2b lands in the same hole.
