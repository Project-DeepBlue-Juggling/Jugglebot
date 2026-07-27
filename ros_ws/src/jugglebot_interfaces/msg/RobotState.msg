# For distributing the state of the entire robot all at once

builtin_interfaces/Time timestamp
MotorStateSingle[] motor_states # states of each motor individually
string[] error                    # Human-readable error descriptions (for logging/display)

# Typed error flags — orchestrator uses these for error classification instead of parsing strings.
# Set by the CAN node from its internal motor state tracker flags.
bool has_fatal_odrive_error       # Fatal ODrive error (axis disarmed in CLOSED_LOOP, exceeded reset attempts, etc.)
bool has_fatal_can_error          # CAN bus disconnection or restoration failure
bool has_undervoltage             # DC bus undervoltage detected (e.g., E-stop hit)

bool firmware_validated       # All ODrive axes report consistent firmware/hardware versions

bool encoder_search_complete # Has the encoder search been run on every leg?
bool is_homed                # Have the ODrives been homed since the last bootup?
bool levelling_complete      # Has the platform been levelled since the last bootup?

float32[] pose_offset_rad                 # Pose offset in radians ([tiltX, tiltY])
geometry_msgs/Quaternion pose_offset_quat # Pose offset as a quaternion

# Pose offset is given as both rad and quat because the rad format is half the data and hence can more easily be sent to the
# Teensy over the CAN bus.

# Platform-Teensy firmware identity, decoded from bytes 5-6 of the SAME 0x6E0
# RobotState reply that carries is_homed / levelling_complete / pose_offset above.
# See ros_ws/docs/platform_fw_version.md (contract C-PLATFW-1).
#
# Read the two fields TOGETHER — platform_fw_version alone is ambiguous at 0:
#   platform_fw_version_read = False  -> nothing is known; no authoritative relay
#                                        read has landed. This is NOT evidence of
#                                        a stale flash, and usually not a fault at
#                                        all: a missed boot read on a launch-only
#                                        restart is a documented benign transient
#                                        (it also forces the re-home you may be
#                                        seeing). Relaunch and re-read before
#                                        investigating CAN3.
#   read = True,  version = 0         -> the board ANSWERED and carries
#                                        PRE-VERSIONING firmware: it has not been
#                                        flashed since 2026-07-27. A definite
#                                        verdict, not an absence — every older
#                                        firmware zero-filled those bytes.
#   read = True,  version >= 1        -> the board's own release number.
#
# REPORTED, NEVER ENFORCED. A skew warns (an ERROR log line tagged
# PLATFORM_FW_CHECK, plus link_status/platform_fw_version); no hand command,
# toss or state transition is ever refused on it. Refusing would put a cached
# remote read in front of the kind-3 retract, which is the only un-arm mechanism
# the Teensy offers.
uint16 platform_fw_version
bool platform_fw_version_read
