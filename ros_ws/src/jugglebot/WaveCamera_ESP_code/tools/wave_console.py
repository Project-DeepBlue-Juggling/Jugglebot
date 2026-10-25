#!/usr/bin/env python3
"""Send commands to the Wave GPIO explorer and capture its console output.

Each received line is printed prefixed with the host wall-clock time
(ISO 8601, milliseconds). Examples:

    python tools/wave_console.py --cmd help --cmd status
    python tools/wave_console.py --no-reset --cmd "probe 2" --wait 4 --out bench_logs/probe.log
    python tools/wave_console.py --listen 20

Run with the PlatformIO python, which has pyserial:
    C:/Users/harri/.platformio/penv/Scripts/python.exe tools/wave_console.py ...
"""

import argparse
import datetime
import os
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # pragma: no cover
    sys.exit("pyserial is required (use the PlatformIO penv python)")

CP210X_HWID = "10C4:EA60"
DEFAULT_PORT = "COM5"


def pick_port(requested):
    ports = list(serial.tools.list_ports.comports())
    names = {p.device.upper() for p in ports}
    if requested and requested.upper() in names:
        return requested
    for p in ports:
        if CP210X_HWID in (p.hwid or "").upper():
            if requested:
                print(f"# {requested} not found; using {p.device} ({p.description})", file=sys.stderr)
            return p.device
    return requested  # let pyserial report the error


def stamp():
    return datetime.datetime.now().isoformat(timespec="milliseconds")


class Capture:
    def __init__(self, ser, out_file):
        self.ser = ser
        self.out = out_file
        self.buf = b""
        self.rx_bytes = 0

    def emit(self, text):
        line = f"{stamp()} {text}"
        print(line, flush=True)
        if self.out:
            self.out.write(line + "\n")
            self.out.flush()

    def pump(self, seconds):
        """Read and print complete lines for `seconds`."""
        end = time.monotonic() + seconds
        # Set the read timeout once. Assigning ser.timeout reconfigures the
        # Windows port each time, and that intermittently fails with
        # "Access is denied" mid-capture.
        if self.ser.timeout != 0.05:
            self.ser.timeout = 0.05
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0:
                break
            chunk = self.ser.read(4096)
            if not chunk:
                continue
            self.rx_bytes += len(chunk)
            self.buf += chunk
            while b"\n" in self.buf:
                raw, self.buf = self.buf.split(b"\n", 1)
                self.emit(raw.rstrip(b"\r").decode("utf-8", errors="backslashreplace"))

    def flush_partial(self):
        if self.buf:
            self.emit(self.buf.decode("utf-8", errors="backslashreplace") + "  [no newline]")
            self.buf = b""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=DEFAULT_PORT,
                    help="serial port (default COM5; falls back to the first CP210x, hwid 10C4:EA60)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--cmd", action="append", default=[], help="command to send (repeatable, sent in order)")
    ap.add_argument("--gap", type=float, default=0.5, help="seconds between commands (default 0.5)")
    ap.add_argument("--wait", type=float, default=3.0, help="seconds to keep capturing after the last command")
    ap.add_argument("--listen", type=float, default=None, metavar="N",
                    help="just capture for N seconds (commands, if any, are sent first)")
    ap.add_argument("--out", default=None, help="also append the capture to this file")
    ap.add_argument("--no-reset", action="store_true",
                    help="open with DTR/RTS deasserted so the CP2102 does not reset the ESP32")
    ap.add_argument("--settle", type=float, default=None,
                    help="seconds to capture before the first command (default 0.3, or 2.0 after a reset)")
    args = ap.parse_args()

    # Open the capture file first so a bad path fails before the board is touched.
    out_file = None
    if args.out:
        out_dir = os.path.dirname(os.path.abspath(args.out))
        os.makedirs(out_dir, exist_ok=True)
        out_file = open(args.out, "a", encoding="utf-8")

    port = pick_port(args.port)
    ser = serial.Serial()  # port=None: configure before opening
    ser.port = port
    ser.baudrate = args.baud
    ser.timeout = 0.05
    if args.no_reset:
        # Deasserted before open() so pyserial does not raise them on open.
        # Note: on Windows the CP210x driver may still glitch EN briefly at
        # open, which can reset the ESP32 anyway on some boards.
        ser.dtr = False
        ser.rts = False
    cap = None
    try:
        ser.open()
        cap = Capture(ser, out_file)
        if not args.no_reset:
            # Pulse EN through the CP2102's auto-reset circuit for a clean boot.
            # Inside try/finally so Ctrl-C cannot leave RTS (EN low) asserted.
            ser.dtr = False
            ser.rts = True
            time.sleep(0.1)
            ser.rts = False
        settle = args.settle if args.settle is not None else (0.3 if args.no_reset else 2.0)
        cap.pump(settle)
        for i, cmd in enumerate(args.cmd):
            cap.emit(f"# >>> {cmd}")
            ser.write((cmd + "\n").encode("utf-8"))
            ser.flush()
            cap.pump(args.gap if i + 1 < len(args.cmd) else 0)
        cap.pump(args.listen if args.listen is not None else args.wait)
        cap.flush_partial()
    except KeyboardInterrupt:
        pass
    finally:
        if ser.is_open:
            ser.rts = False
            ser.close()
        if out_file:
            out_file.close()
    if cap is None or cap.rx_bytes == 0:
        print(f"# no bytes received from {port}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
