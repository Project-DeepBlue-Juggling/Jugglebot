# Recording start/stop event from the Wave camera adapter. Decoded from a
# WAVE_REC_EVENT CAN frame (ID 0x7E4) by teensy_bridge_node; published on
# camera/record_event. header.stamp == event_time.

uint8 START=1
uint8 STOP=2
uint8 AUTO_STOP=3
uint8 REFUSED=4

std_msgs/Header header
uint8 kind                           # START / STOP / AUTO_STOP / REFUSED
uint8 reason                         # 0=NONE, 1=NO_CHANGE, 2=INTENT_TIMEOUT, 3=TICK_LOST, 4=CAMERA_OFF,
                                     # 5=CLIP_MISSING, 6=CONSOLE_OWNS, 7=GPI2_FAULT
builtin_interfaces/Time event_time   # Wall-clock instant of the event (the camera tick when tick_stamped)
uint64 event_time_us                 # event_time as Unix-epoch microseconds
bool time_synced                     # False => the adapter was unsynced; event_time is the bridge RX
                                     # stamp minus the frame's event age (still bridge wall clock)
bool tick_stamped                    # True if stamped at a camera 30 Hz tick (else at the GPI2 edge)
uint32 recording_seq                 # Recording counter (unwrapped from the wire's u8)
uint8 cmd_seq                        # WAVE_CMD sequence that caused it (AUTO_STOP: last seen)
