# Liveness + state of the Electronic Clapboard, relayed from its CLAP_HEARTBEAT
# CAN frame (0x7EC, 10 Hz) by teensy_bridge_node.  Published at 10 Hz whether or
# not a clapboard is attached: consumers rely on the topic being alive to
# display "disconnected", so connected=False defaults appear before the first
# heartbeat ever arrives.
#
# Wire contract: Electronic-Clapboard docs/protocol.md section 8.7 (normative,
# cross-repository).

bool connected               # True if a CLAP_HEARTBEAT arrived within CLAP_HEARTBEAT_TIMEOUT_MS
uint8 state                  # 0=BOOT 1=IDLE 2=RENDERING 3=SCREENSAVER 4=ERROR
                             # (note: NOT the catching cone's numbering, whose ERROR is 127)
bool fire_ready              # Flash circuit charged and armed
bool template_loaded         # A slate template is present in the clapboard's flash
bool wifi_up                 # The clapboard's editor/HTTP interface is reachable
bool rail_ok                 # Flash rail voltage within spec
bool time_synced             # Locked to the bridge's 0x7DD wall clock.  While False the
                             # clapboard emits NO fire events at all — the flash still
                             # happens, only the record is suppressed, so a gap between
                             # fires_since_boot and published events is expected here.
uint16 fires_since_boot      # Monotonic flash counter, wraps at 65536
uint16 rail_mv               # Flash rail voltage {mV}; 0 when no divider is fitted
uint8 active_template_id     # Template currently composited on the panel
uint8 last_error             # Last non-OK CLAP_ACK outcome (0 = none)
