#!/bin/bash
# Buttonless Platform-Teensy (4.0) upload — invoked by platformio.ini's
# upload_command with the built .hex as $1.
#
# WHY A SCRIPT: with both Teensys on USB, teensy_loader_cli's generic -s
# soft-reboot targets "whichever Teensy answers first" — it cannot select a
# board. But the underlying Teensyduino soft-reboot mechanism is just a
# 134-baud touch on a SPECIFIC serial port, so pinning the platform board's
# USB serial number makes the reboot deterministic: touch only the 4.0's
# port, then program MCU-locked (--mcu=TEENSY40 refuses a 4.1 regardless).
#
# PLATFORM_SERIAL pins the physical board (udev ID_SERIAL_SHORT). If the
# board is ever replaced: with only the new board attached, run
#   udevadm info -q property /dev/ttyACM0 | grep ID_SERIAL_SHORT
# and update the constant.
#
# Failure modes:
#   - Pinned port absent (board unplugged, or already sitting in HalfKay):
#     falls through to the loader's -w wait — press the 4.0's button, the
#     pre-script behaviour.
#   - Wrong serial pinned: the 134-touch would bootloader the WRONG board;
#     the MCU-locked loader then refuses it and waits. Recover the stranded
#     board with `teensy_loader_cli --mcu=TEENSY41 -b` (boot only) or by
#     reflashing it.
set -u
PLATFORM_SERIAL="11744460"
LOADER=/home/jetson/tools/teensy_loader_cli/teensy_loader_cli
HEX="$1"

PORT="/dev/serial/by-id/usb-Teensyduino_USB_Serial_${PLATFORM_SERIAL}-if00"
if [ -e "${PORT}" ]; then
    echo "[upload_platform] soft-rebooting the 4.0 (serial ${PLATFORM_SERIAL}) via 134-baud touch"
    stty -F "${PORT}" 134 || true
    sleep 1.5
else
    echo "[upload_platform] pinned 4.0 (serial ${PLATFORM_SERIAL}) not in /dev/serial/by-id — press its program button"
fi
exec "${LOADER}" --mcu=TEENSY40 -w -v "${HEX}"
