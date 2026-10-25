# For relaying the heartbeat being received from the Wave camera adapter (ESP32
# on the cone-role CAN bus). Decoded from WAVE_HEARTBEAT (0x7E3, 10 Hz) and the
# WAVE_INFO pages (0x7E5) by teensy_bridge_node; published at 10 Hz on
# camera/heartbeat. Contract: docs/adr/0014-wave-camera-on-can3.md.

bool connected                   # True if a WAVE_HEARTBEAT was received within the timeout window
                                 # (host-monotonic; every other field is a placeholder when False)
uint8 state                      # Adapter state (0=BOOT, 1=IDLE, 2=STARTING, 3=RECORDING, 4=STOPPING, 5=FAULT)
bool camera_on                   # True if a camera 30 Hz tick was seen within 200 ms
bool uart_seen                   # True once a "Created new clip." line has been seen since boot
bool recording                   # True when state == RECORDING
uint32 elapsed_ms                # Time since the recording's start tick {ms}, 0 when not recording
                                 # (wire resolution 100 ms; extrapolated host-side past the u16 saturation)
builtin_interfaces/Time start_stamp  # Wall-clock instant of the current recording's start tick
                                     # (zero when not recording or no START event seen)
uint32 recordings_this_session   # Recordings started since this node's session_id took effect (unwrapped)
bool time_synced                 # True if the adapter is locked to the 0x7DD wall clock
uint32 sync_age_ms               # Time since the last accepted 0x7DD beacon {ms}
                                 # (4294967295 = never, or >= 25.5 s)
uint16 fw_version                # Adapter firmware version (WAVE_INFO page 0; 0 until first seen)
uint32 uptime_s                  # Adapter uptime {s} (WAVE_INFO page 0; 0 until first seen)
uint8 fault_bits                 # Latched faults: bit0 GPI2_REFUSED, bit1 TICK_LOST_MID_REC, bit2 CLIP_MISSING,
                                 # bit3 INTENT_TIMEOUT_AUTOSTOP, bit4 BUS_OFF_SEEN, bit5 SYNC_STEP, bit6 EDGE_RING_OVERFLOW
bool console_owns                # True if the USB console owns GPI2 (CAN intents are refused)
bool intent_fresh                # True if the adapter heard a WAVE_CMD within 3 s
uint16 offset_spread_us          # Time-sync latency spread in the last window {us} (WAVE_INFO page 2)
