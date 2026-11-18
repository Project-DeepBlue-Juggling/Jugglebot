---
title: "Platform IMU P3a: can-bridge FW 45 relays the Platform IMU (IMU_REPLY/IMU_STATUS on the reply whitelist, IMU_DATA on a dedicated lock-free ring uplinked as the additive PLATFORM_IMU_BATCH, PLATFORM_IMU_CTRL/_READ RPCs with the Armed gap clamp) (IDEA-20)"
type: feature
date: 2026-11-19
status: in-progress
phase: "platform-imu P3"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - teensy_link/platform_imu.py
  - teensy_link/platform_imu_capture.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - tools/teensy_link_bridge.py
  - tests/teensy_link/test_platform_imu.py
  - tests/teensy_link/test_platform_imu_native.py
  - tests/teensy_link/fake_platform_imu.py
  - tests/teensy_link/test_fw_update_begin_retry.py
  - tests/ros/test_teensy_bridge_node_platform_imu.py
  - tests/ros/test_teensy_bridge_node_conduit.py
  - ros_ws/src/jugglebot/jugglebot/platform_imu_gate.py
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/gui/js/imu-upload-panel.js
  - tests/ros/test_platform_imu_gate.py
  - tests/ros/test_teensy_bridge_node_imu_trigger.py
  - tests/ros/test_gui_imu_upload_panel.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/imu_ring.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/platform_relay.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/platform_relay.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - config/protocol_config.yaml
  - config/generate_udp_protocol.py
  - teensy_link/rpc_args.py
  - teensy_link/protocol.py
  - ros_ws/docs/platform_imu_protocol.md
  - plans/active/platform-imu.md
  - tests/firmware/native/test_platform_relay.cpp
  - tests/firmware/native/test_rpc_dispatch.cpp
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/firmware/test_bridge_fw_version_xref.py
  - tests/firmware/test_platform_imu_protocol.py
---

# Platform IMU P3a: the can-bridge side (FW 45)

**What.** The bridge half of `plans/active/platform-imu.md` P3, against the normative
`ros_ws/docs/platform_imu_protocol.md` section 6. Additive only: no existing message, struct, ISR,
task or priority changes, PROTOCOL_VERSION stays 14.

- IMU_REPLY 0x6E4 and IMU_STATUS 0x6E5 join `is_platform_reply_id` and ride the existing 8-slot
  reply ring as PLATFORM_FRAMEs. Why not their own ring: STATUS is 1 Hz and a REPLY answers one
  IMU_CMD, which the host sends after the previous reply (about 5/s at 1250 frames/s with 256-word
  blocks), so they add a few frames/s to a ring drained at 400/s and cannot evict tilt, fan or
  FwUpdate replies. A host spamming RPCs could fill it, as it can today with TILT_READ.
- IMU_DATA 0x6E6 goes to a dedicated lock-free SPSC ring (`imu_ring.h`, 128 frames = 80 ms at the
  1600 frames/s cap) filled in `on_jugglebot_rx` and drained by `platform_imu_uplink_step` into the
  new MsgType PLATFORM_IMU_BATCH 0x95: up to 24 frames per 230-byte datagram, at most 4 datagrams
  per tick, with cumulative counters (rx_frames, ring_drops, ring_hwm, batches_sent, send_fails) in
  every batch. Full = drop the newest and count it; the Jetson recovers any hole by `seq` with a READ
  (spec section 7), so which frame is lost does not matter, and drop-newest keeps each index
  single-writer.
- RPCs PLATFORM_IMU_CTRL 0x64 (ENABLE/DISABLE/FREEZE/RESUME/ABORT; READ and unknown ops are
  ERR_BAD_ARGS so a READ cannot bypass the clamp) and PLATFORM_IMU_READ 0x65 (count 1..256), both
  through the existing `send_gated` (jugglebot_commands_allowed, `can_jugglebot_tx(TxCls::SAFETY)`,
  as HAND_FAN_SET), both allowed while Armed. While `fault_mpc_active()` the READ gap is raised to
  the new YAML `platform_imu.bridge_armed_min_gap_10us` = 63 (630 us, ~1590 frames/s); not Armed it
  goes out verbatim and the Platform's 400 us floor applies (the spec puts no bridge clamp there).
  OK = queued; the verdict is the IMU_REPLY. No ABORT at arming.
- FW 44 -> 45. The held `night-shift/2026-11-15-BUG-93` branch also uses 45 and has not landed:
  whichever reaches main second renumbers. A host/board FW mismatch is warn-only
  (`BRIDGE_FW_CHECK: FAIL` at ERROR, commands not refused, `teensy_bridge_node.py`), and with no
  PROTOCOL bump the link stays up, so this branch can reach main before the flash; an FW 44 board
  answers the new RPCs with ERR_UNKNOWN_METHOD.

## Timing and control walk-through

Context, from the code: the 500 Hz setpoint emit is an IntervalTimer ISR at NVIC priority 16, above
the FreeRTOS syscall ceiling and above FlexCAN's ISR, and makes no FreeRTOS calls. CAN RX callbacks
run in `task_can_rx` (priority 5, 1 kHz, `events()` drained up to 32 frames per tick). Setpoint
staging (`interp_on_setpoint`) runs in `task_net` (priority 4) after it pumps lwIP under `NetLock`
(a recursive FreeRTOS mutex, so priority inheritance applies). Uplinks run in `task_telem`
(priority 3, 100 Hz); every `udp_send_stream` takes `NetLock` for one encode + send.

- **Per IMU_DATA frame** (task_can_rx): the existing callback prologue every frame already pays
  (stamps, lag fold), then one id compare and the ring push: two atomic index loads, an 8-byte copy,
  one release store, three relaxed counter stores. Tens of cycles, well under 1 us. No PRIMASK, no
  lock: the ISR is never masked by it. Every other id pays one extra compare.
- **Per telemetry tick** (task_telem): at 1250 frames/s 12.5 frames per tick, at 1600 16, so ONE
  datagram per tick at both (24 slots). Cost [I, unmeasured]: drain + 220-byte fill ~1-2 us, CRC-16
  (bitwise) over 228 B ~11 us at 600 MHz, lwIP send ~10-30 us, so ~25-45 us. Bounded worst case
  after a stall: 4 datagrams, ~100-180 us, each send releasing NetLock between sends. The rejected
  per-frame route would have been 13-16 sends and 13-16 NetLock takes per tick.
- **Shared with the ISR or the fault task**: nothing added. The ring is lock-free; the only lock
  touched is the existing NetLock, by the existing `udp_send_stream`, from the existing task.
- **Setpoint staging**: task_net can wait on NetLock for at most one in-progress send_to (~25-45 us)
  per contention, once per 10 ms tick in steady state. Against the 30 ms Leg scheduler grace and the
  25 ms setpoint period that is about 0.15 %; the 47 ms worst emit-gap residue is a host-side
  number and microseconds in the bridge cannot plausibly move it. task_fault shares priority 3 with
  task_telem and can be delayed by the same tens of microseconds per tick.
- **Bus side**: 1250-1600 extra RX frames/s, all lower CAN priority than every ODrive and setpoint
  frame (delay to a control frame at most one frame time, ~0.13 ms). RX path: today ~2130 RX
  frames/s (2.1 per 1 ms tick) plus 1.6 per tick, against a drain budget of 32 per tick and a 256-deep
  FlexCAN ring whose high-water mark is 9 today. Bus load during an Armed upload: spec section 8,
  about 73-78 % at 111 bit/frame; measured, not assumed, at the P4 ramp.

**Residual risks.** The send_to cost is estimated, not measured. `fault_mpc_active()` is the host's
own heartbeat claim, so the clamp is defence in depth behind the Jetson gate. `std::atomic` is new to
the bridge (16/32-bit, lock-free on Cortex-M7; the build confirms no libatomic call). The reply-ring
drop counter `can_platform_fwd_drops()` is not uplinked anywhere (pre-existing). Batch counters are
visible only while batches flow, which is when they can move.

**Counters the P4 rate ramp must watch.** PLATFORM_IMU_BATCH `ring_drops` (must stay 0),
`ring_hwm` (vs 128), `send_fails`, `rx_frames` rate vs the requested pace; `/profile`
jugglebot-role `can*_rx` util, `cpu_pct_x100`, `interp_deadline_misses`, `interp_max_jitter_us`;
the heartbeat's `rx_age_max_us` and age buckets, `dn_queue_dropped`, `dn_seq_gaps` (setpoint
staging); `/cache_diag` `rx_depth_hwm_jb`, `rx_cap_hits_jb`; `/ring_diag` `fifo_overflows`,
`true_depth_hwm`; `/bridge_tx_diag` jugglebot `tx_deferred`, `tx_q_hwm`.

## Verification

- Scoped Python readers (grep -rlE over tests/ for Teensy_code_canbridge, platform_imu_protocol,
  generate_udp_protocol, udp_protocol, rpc_args, teensy_link protocol, EXPECTED_BRIDGE_FW_VERSION,
  protocol_config, platform-imu; minus hardware scripts, conftest/harness and the native wrappers:
  87 files, list in the session scratchpad), run 2026-11-19:
  `nice -n 10 python -m pytest <those 87 files> -q -p no:cacheprovider -x -n 2` -> 2440 passed, 4 skipped in 427.60 s.
- Native + docs readers, run 2026-11-19: `nice -n 10 python -m pytest tests/firmware/test_native_firmware.py
  tests/firmware/test_platform_imu_native.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py
  tests/sim/test_docs_links.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_plans_index.py -q -p no:cacheprovider`
  -> 86 passed in 342.44 s (new: 9 relay/ring/batch TEST_CASEs in test_platform_relay.cpp, 1 in test_rpc_dispatch.cpp).
- Real build, run 2026-11-19 on a scratchpad copy of the sketch: `nice -n 10 pio run -e teensy41` -> SUCCESS,
  text 255296, data 35520, bss 113216 (dec 404032); no warning from any touched file (the 4 remaining are the
  pre-existing fault_machine.cpp unused-function and FlexCAN_T4 strncpy ones); no libatomic call needed.
- Not run: ./run_tests.sh (a sibling gate held the box); the tier-1 gate is owed before main.

Not flashed; the Jetson side (P3b) is next.

# Platform IMU P3b: the Jetson side (host to the fixed wire protocol)

**What.** `teensy_link/platform_imu.py` (codecs for IMU_REPLY / IMU_STATUS / header / anchors / the
PLATFORM_IMU_BATCH datagram; `UploadMachine`, a single-owner state machine driven by injected time and
send functions; `PlatformImuService`, one worker thread behind two bounded queues) and
`teensy_link/platform_imu_capture.py` (the timestamp model, SI scaling, the npz capture writer). Node
wiring in `teensy_bridge_node.py` (`_setup_platform_imu`): own queues, status topic `platform_imu/status`
(DiagnosticStatus, 2 Hz, added to the rosbag list), services `platform_imu/set_upload_after_attempt`
(SetBool), `platform_imu/upload` (SetString, JSON span), `platform_imu/abort` (Trigger), and the API unit
P3c calls: `attempt_ended(t_start_wall, t_end_wall, reason, attempt_id)`, `upload_in_progress()`,
`abort_upload(reason)`, `imu_upload_status()`, `set_imu_upload_after_attempt(on)`. Tilt: raw NaN words
classified before the float bound (`classify_tilt_reply`): not-ready waits at least 1 s between at most 4
tries, read-failed and (pi, pi) take the existing 3-attempt failure path. Uploader: Platform BEGIN is re-sent
through silence for 25 s (`_FW_BEGIN_RETRY_S`, one attempt per 1 s reply wait, every attempt logged); Ball
Butler stays single-BEGIN; nothing after BEGIN OK changed.

**Spec points settled in code.**
- The payload CRC covers all anchors plus ALL samples, so a span request (the Attempt plus margin) cannot be
  CRC-checked: a partial span records `crc_checked = false` and relies on the CAN frame CRC; a full upload is
  checked, re-read once on a mismatch, then written `crc_ok = false` and RESUMEd.
- The span is chosen after the anchors arrive (the fit maps wall time to sample index), so FREEZE, header and
  anchors always upload and only the sample region is trimmed.
- A FREEZE answered ALREADY_FROZEN on the first try is an earlier capture (flag `already_frozen`, uploaded in
  full, the span ignored); on a resend after a lost reply it is our own FREEZE and not flagged.
- STATUS right after FREEZE/RESUME may still show the old state, so "not FROZEN/UPLOADING" counts as a reboot
  only after a 2.5 s grace; a changed boot byte counts at once.
- Block timeout = count x EFFECTIVE gap + 150 ms (spec 50 ms plus the bridge's 10 ms batching and UDP slack),
  extended while frames keep arriving; effective gap = max(requested, 40, 63 while Armed).
- File name `<UTC stamp>_<capture_id hex>.npz` (+ `.json` sidecar) under `temp/imu_capture/`, not the spec's
  per-capture directory (one npz is one atomic rename; the sidecar carries the same metadata).

## Timing and control note (P3b)

- **UDP receive thread** (`_on_platform_imu_batch`, `_on_platform_frame`): per PLATFORM_IMU_BATCH datagram one
  fixed 28-byte `struct.unpack_from`, one slice of at most 192 bytes and one `queue.put_nowait` (about 50-70
  datagrams per second at 1250-1600 frames/s, a few microseconds each); a full queue drops and counts. Relayed
  IMU_REPLY/IMU_STATUS add one `put_nowait` on a separate 256-slot queue. No per-frame Python work, no I/O.
- **`platform_imu_worker` thread** (new, daemon): owns the machine, parses records into the region buffers,
  runs the timeouts, and issues every IMU RPC (`_call_rpc`, 0.3 s timeout, no retries; each blocks this thread
  only while its data queue absorbs up to 2048 batches). Builds the capture (a numpy fit, milliseconds) and hands
  it to the writer.
- **`platform_imu_writer` thread** (new, daemon): npz/JSON file I/O behind a 4-slot queue.
- **ROS callback groups:** a new `ReentrantCallbackGroup` (`_imu_cbgroup`) holds the 2 Hz status timer and the
  three services; the services only enqueue an event or read a snapshot and never block, so nothing is added to
  `_blocking_rpc_cbgroup` (BUG-98) or to the default group that runs the 100 Hz publishers. Nothing touches the
  setpoint emit path. The tilt service (coldstart group, already blocking) may now sleep up to 3 s in the
  FW 11 not-ready case.
- **Net effect on the 40 Hz planner / leg path:** none by construction; the extra load is the two threads and
  about 60 small callbacks per second during an upload, none while an Attempt runs (STATUS only, 1 Hz).

## Verification (P3b)

Tests run 2026-11-19 with `nice -n 10 python -m pytest` on a quiet box:
- new: `tests/teensy_link/test_platform_imu.py`, `test_platform_imu_native.py` (decodes header and anchors
  written by the compiled firmware headers, and uploads the firmware-written image), `test_fw_update_begin_retry.py`,
  `tests/ros/test_teensy_bridge_node_platform_imu.py`; updated `tests/ros/test_teensy_bridge_node_conduit.py`
  (tilt invalid replies, now explicit).
- scoped readers (`grep -rlE "teensy_bridge_node|teensy_link_bridge|jugglebot_launch|platform_imu|teensy_link\.platform"
  tests`, minus hardware scripts, conftest/harness: 84 files, `nice -n 10 python -m pytest <84 files> -q -p no:cacheprovider
  -n 2`): 2335 passed, 3 failed, 4 skipped in 297 s; the 3 failures were findings, fixed the same day: the generated
  `ros_ws/docs/choreography.md` (regenerated with `python tools/gen_choreography_map.py`) and the orchestrator
  integration test's `relay_read_tilt` stub (now `relay_read_tilt_ex`). Re-run of those three files
  (`test_choreography_map.py test_orchestrator_conduit_integration.py test_launch_nodes.py`): 49 passed in 12.6 s;
  docs readers (`test_logbook_front_matter/index`, `test_docs_links`, `test_no_legacy_item_ids`, `test_plans_index`): 61 passed.
- Not covered: real bus timing and loss (P4 ramp), the receive thread under real 1600 frames/s load, a real
  Platform boot-loop, the live `Armed` gate. `./run_tests.sh` is owed before main.

## P3c: the Attempt-end trigger, the command gate and the GUI toggle (2026-11-19)

**What.** After an Attempt (normal end, self-end, operator stop) or a Fault / Soft E-stop during one, the Jetson uploads
the Platform IMU capture of the Attempt's span (2 s before to 3 s after, last 32 s if longer). While an upload runs with
the toggle ON, motion-starting commands are refused with a reason that names the upload and the seconds left. The GUI
has the toggle, a progress bar and the last capture's result.

**Where and why.** The trigger is in `teensy_bridge_node` (in-process `attempt_ended`): it already owns the IMU and
the fault edge, and the Attempt's span arrives on `skills/attempt`, which `skill_node` publishes for every goal source,
so `skill_node`'s end-of-Attempt path gets no new code at all and a dead or slow IMU path cannot reach it. Every handler
is wrapped and only queues work for the IMU worker thread. The gate is `jugglebot/platform_imu_gate.py` (pure), one call
per entry point: `orchestrator_node._on_command`, `_svc_juggle_request`, `skill_node._juggle_goal` (so a
`ros2 action send_goal` cannot start an Attempt over a frozen ring). The gate reads cached status, never a service.
Gate rules, each tested: fail-open on a status older than 2 s or absent; a 180 s busy ceiling treats the upload as idle and
sends one abort per 10 s; pacifying commands (`deactivate`) and everything off the motion list always pass.

**Commands.** Refused: `standby`, `trajectory`, `spacemouse`, `gui`, `level`, `home`, the Juggle request. Always allowed:
`deactivate` (aborts), the Soft E-stop (the bridge aborts on the Fault edge), `activate`, `clear_errors`, the Juggle stop.
`activate` and `clear_errors` are not on the owner's list and start no Attempt or profiled move.

**Operator walk-through.**
- Toggle ON, upload running, Start pressed: the Juggle service answers failure with "'juggle' refused: the Platform IMU
  upload is running (samples, about 12 s remain). Wait for it to finish, send 'deactivate' to stop it, or switch off ..."
  (shown by the Juggle panel line); nothing is dispatched. After the upload RESUMEs, Start works.
- Toggle ON, upload running, deactivate: enqueued at once, `platform_imu/abort` sent fire-and-forget; the capture is
  written `complete = false` and the ring resumes.
- Bridge node dies mid-upload: `platform_imu/status` stops; after 2 s the gate allows everything (fail-open).
- Platform IMU absent: `attempt_ended` returns False (no IMU_STATUS), no upload, `busy` never rises, nothing refused.
- Toggle switched OFF mid-upload: the upload is aborted (flagged incomplete). Chosen over "finish but stop refusing"
  because OFF is meant to mean nothing frozen and nothing refused, and an unrefused upload would let an Attempt start over
  a frozen ring on a loaded bus; the cost is one partial capture, which the flag says.
- Start pressed within 0.5 s of an Attempt end (status is 2 Hz, so the gate may not know yet): allowed, and the bridge
  aborts the upload at the `juggle_start` event, so the Attempt is never run over a frozen ring.

**Control implications.** No change to the 40 Hz planner, feedforward or emit path: the new code is a subscriber
callback on the IMU callback group, a cached-dict read in the command path and a rate-limited fire-and-forget abort call.

**Verification.** 2026-11-19, box load average about 16 (another session's gate running):
- new tests: `nice -n 10 python -m pytest tests/ros/test_platform_imu_gate.py tests/ros/test_teensy_bridge_node_imu_trigger.py tests/ros/test_gui_imu_upload_panel.py -q -p no:cacheprovider`: 92 passed (80 + 12).
- readers (grep `-rlE "orchestrator_node|skill_node|teensy_bridge_node|main\.js|index\.html|choreography|platform_imu|replay/(schema|policy|allowlist)" tests`, narrowed to the cheap files): `... python -m pytest tests/ros/test_teensy_bridge_node_platform_imu.py test_teensy_bridge_node_deactivate.py test_teensy_bridge_node_console.py test_orchestrator_conduit_integration.py test_launch_nodes.py tests/teensy_link/test_platform_imu.py test_platform_imu_gate.py test_teensy_bridge_node_imu_trigger.py test_choreography_map.py -n 2`: 188 passed in 73 s; `test_orchestrator_node.py test_state_machine.py`: 282 passed; GUI and replay files (`test_gui_replay_policy test_replay_allowlist test_replay_mcap_oracle test_gui_replay_format_contract test_replay_api test_gui_replay_engine test_gui_replay_ui_hide test_gui_replay_ui_fence_dom test_gui_replay_feed test_gui_replay_mode test_gui_imu_upload_panel`): 99 passed in 36 s; `test_rosbridge_websocket_lean test_replay_mcap_oracle`: 45 passed.
- Findings fixed on the way: `skill_node`'s status subscription named its topic through a module constant (the generated `ros_ws/docs/choreography.md` could not resolve it; now a literal, doc regenerated); `test_gui_replay_policy` demanded `/platform_imu/status` in `replay/schema.py` SUBSCRIBED, `js/replay/allowlist.js` and `js/replay/policy.js` (added; the topic was already in the bag list).
- Not finished: the whole `tests/ros/test_skill_node.py` (planner-heavy, about 1 test per 25 s under that load) was stopped; its `TestJuggleAction` class ran 38 tests with no failure before the stop, and `test_skill_node_check`/`_nominate` were not re-run after the edit. Owed to the gate.
Not covered: a live GUI session, real DDS latency of `skills/attempt` (the 2 s margin absorbs it), the hardware ramp (P4).
There is no `gui_server` endpoint (toggle = rosbridge service, progress = topic), so no server test.

Full gate (`./run_tests.sh --full`): FULL-GATE-TRIPLE-PENDING
