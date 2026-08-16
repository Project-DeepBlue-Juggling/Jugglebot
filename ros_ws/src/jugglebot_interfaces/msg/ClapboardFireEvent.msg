# One accepted sync flash from the Electronic Clapboard, relayed from its
# CLAP_FIRE_EVENT CAN frame (0x7ED) by teensy_bridge_node.  This is the device's
# core output: the wall-clock instant an LED flash occurred, so multi-camera
# footage can be aligned against robot telemetry in post-production.
#
# The clapboard emits NOTHING when its wall clock has never been anchored to the
# bridge's 0x7DD broadcast (a missing sync record is recoverable in post; a wrong
# one silently corrupts an edit).  Every message on this topic is therefore
# time-anchored by construction — there is no "unsynced" variant to check.
#
# Wire contract: Electronic-Clapboard docs/protocol.md section 8.8 (normative,
# cross-repository).

std_msgs/Header header
builtin_interfaces/Time fire_time   # Wall-clock instant of the flash
uint64 fire_time_us                 # The same instant in raw microseconds since the Unix
                                    # epoch.  The wire carries only the LOW 48 BITS (a 2026
                                    # Unix-us value needs 51), so this is the host-
                                    # reconstructed full value, not the raw field.
uint16 fires_since_boot             # The clapboard's monotonic flash counter at this flash.
                                    # Gaps against this topic's message count are expected
                                    # whenever the device was unsynced — see above.
