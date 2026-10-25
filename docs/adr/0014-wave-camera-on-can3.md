# ADR-0014: The Wave camera adapter is a cone-role device on CAN3, kept recording by a streamed intent

- **Status**: Accepted
- **Date**: 2026-10-19
- **Deciders**: Harrison + Claude
- **Related**: [ADR-0008](0008-time-sync-master-on-can-bridge.md), [ADR-0013](0013-three-can-buses.md), plan `wave-camera-can3-integration.md`, [Wave Camera: CAN adapter design](../../ros_ws/docs/wave_camera_can_adapter.md) (`ros_ws/docs/wave_camera_can_adapter.md`, the binding wire contract); ESP32 firmware in `ros_ws/src/jugglebot/WaveCamera_ESP_code/`

## Context

The robot needs to start and stop recording on a Freefly Wave high-speed camera from ROS 2, and to know each start as a robot wall-clock instant good to about 1 ms, so footage can be lined up with the bag. The camera has a 6-pin GPIO connector: GPI2 (pin 6) starts a recording while it is held low (LVL mode), and pin 4 carries a 30 Hz tick whose first edge after the GPI2 falling edge starts the clip (measured on the bench, [Wave Camera findings](../../ros_ws/docs/wave_camera_findings.md)). An ESP32 drives GPI2, stamps that tick, and talks CAN.

The forces:

1. **A recording must not outlive its operator.** If the Jetson node dies, the network drops or the bridge reboots while the camera is recording, a stateful "start" command would leave the camera recording until someone notices (it fills the card, and a camera power-cycle with GPI2 still held low would restart recording on its own, because LVL mode samples the level).
2. **The start instant has to be the camera's, not the command's.** The command-to-clip latency includes the GPI2 stretch and up to one 33 ms tick period, so stamping the RPC send time would be wrong by tens of ms.
3. **A free bus and ID block.** The can-bridge has three subsystem-isolated buses ([ADR-0013](0013-three-can-buses.md)). The cone-role bus (the CAN3 peripheral since the 2026-07-31 controller swap) carries the catching cone only, and the cone is often unplugged. IDs 0x7E2-0x7E7 are unallocated and all sit above the 0x7DD time beacon.
4. **Bridge firmware churn is expensive** (a reflash on the Jetson, a `BRIDGE_FW_CHECK` skew until it happens), and the UDP `PROTOCOL_VERSION` bump forces a lockstep flash.

## Decision

1. **The adapter is a cone-role device with its own ID block, 0x7E2-0x7E7** (`WaveCameraCanId`: CMD 0x7E2 host to adapter, HEARTBEAT 0x7E3, REC_EVENT 0x7E4, INFO 0x7E5 adapter to host, 0x7E6-0x7E7 reserved). All frames are DLC 8, little-endian. The **cone and the camera adapter are mutually exclusive on the CAN3 drop** (owner decision): there is one 4-way drop, and nothing else changes on the bus.
2. **Recording is kept alive by a streamed intent, not a stateful command.** The node streams WAVE_CMD (`desired`, `cmd_seq`, `flags`, `session_id`) at **5 Hz whenever the adapter heartbeat is fresh**, `desired=0` included, so a lost stop is repaired within 200 ms. The adapter **auto-stops with GPI2 released after 3 s without an intent** (AUTO_STOP, reason INTENT_TIMEOUT) and releases GPI2 if the camera tick disappears (TICK_LOST). `cmd_seq` is bumped once per service call; each new seq produces exactly one REC_EVENT, which is the services' acknowledgement.
3. **The start stamp is tick-quantised.** The adapter stamps the first camera tick after the GPI2 falling edge, cross-checked against the camera's "Created new clip." UART line (expected 15.5-16.3 ms after the tick); its clock is slaved to the 0x7DD beacon. A missing clip line still reports START, with reason CLIP_MISSING, and keeps GPI2 low (the service reports success with a warning).
4. **Event time rides as 40 bits.** REC_EVENT bytes 0-4 carry the low 40 bits of the event's wall time in µs when the adapter is synced; the host rebuilds the high bits from its own clock within a ±6.4-day window (`reconstruct_event_time_us`, the 40-bit twin of the clapboard's 48-bit reconstruction). When the adapter is unsynced the field is the event's age at TX, and the host computes `ConeFrame.t_bridge_us - age` from the bridge's own RX stamp, flagged `time_synced=false`. The byte saved over 48 bits carries both sequence numbers.
5. **One additive RPC, no new message type, no `PROTOCOL_VERSION` bump.** Bridge FW 31 adds `WAVE_CMD` (0x0062, `ArgWaveCmd`, 8 B) that sends its args verbatim as the 0x7E2 frame through `can_cone_tx`; its presence gate (`FAILED`, no cone-bus partner heard) maps to ERR_BUS_DOWN. The uplink already rides the verbatim CONE_FRAME relay. `PROTOCOL_VERSION` stays 10: no existing message changes, and an FW 30 board answers 0x62 with ERR_UNKNOWN_METHOD, loudly (the BB_CANCEL precedent, FW 30).

## Consequences

**Positive**

- A dead node, a dropped link or a rebooted bridge stops the recording within about 3 s, with GPI2 released, with no host involvement.
- Starts are reported as the camera's own clip-start tick, so footage alignment is limited by the tick stamp and the time-sync error, not by RPC latency.
- The heartbeat opens the bridge's TX gate, so an unplugged adapter costs nothing on the bus and the bridge never transmits into a partner-less segment.
- Because WAVE_CMD is idempotent, it can be retried freely and is not in `NON_IDEMPOTENT_METHODS`.
- The only bridge change is one dispatch case; the bridge image stays wire-compatible with every host on `PROTOCOL_VERSION` 10.

**Negative**

- The catching cone and the camera cannot be on the robot at the same time without a second drop or a bus change.
- Steady-state traffic on the cone-role bus rises to about 14 frames/s from the adapter plus 5 Hz of intents, which is negligible against 1 Mbit/s but no longer zero while the cone is absent.
- Start-time accuracy is quantised to the camera tick; mapping the tick to a recorded frame number still needs the optical calibration (a clapboard flash), which is out of scope here.
- A bridge reflash (FW 30 to 31) is needed before the services work; until then `BRIDGE_FW_CHECK` reports the skew and every WAVE_CMD returns ERR_UNKNOWN_METHOD.

**Neutral**

- `session_id` is the node's start time (Unix s); a change resets the adapter's per-session recording count, so "recordings this session" counts from the ROS 2 launch.
- The GUI panel deliberately subscribes to `camera/heartbeat` only; `camera/record_event` is bagged and available to tools, but the panel does not render it.

## Alternatives considered

- **A stateful start/stop command pair.** Rejected: the failure it cannot handle is the one that matters (the host going away mid-recording), and it would need its own retry and confirmation logic for lost stops. The intent stream solves both with one mechanism.
- **A generic raw-frame RPC ("send these 8 bytes on the cone bus").** Rejected: it turns the bridge into an arbitrary-frame injector on a bus, which is what the typed relays (`state_write`, the FW-over-CAN ops) were designed to avoid. A typed method with a validated arg keeps the least-privilege property.
- **48-bit event time, as the clapboard uses.** Rejected: 40 bits already cover ±6.4 days around the host clock, and the freed byte carries `cmd_seq_echo` and `recording_seq` in the same frame.
- **Stamping the start on the GPI2 edge or the RPC send.** Rejected: the clip starts on the next camera tick, up to 33 ms later, plus the drive stretch.
- **A new uplink MsgType for the adapter.** Rejected: CONE_FRAME already relays any cone-bus frame verbatim with a bridge RX stamp, which is exactly what the unsynced-age path needs.
