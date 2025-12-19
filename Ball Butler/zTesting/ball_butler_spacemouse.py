#!/usr/bin/env python3
"""
spacemouse_to_robot.py
----------------------

Reads pitch & yaw from a 3D-connexion SpaceMouse and sends serial commands
to the Teensy CLI you're already running:

    odpos <deg>      (0 … 45, positive down)
    bmpwm <pwm>      (-255 … +255)

Pitch is clipped to 0-45 deg, yaw passes straight through (with an optional
dead-band and a 50-count minimum to overcome motor stiction).

Usage
-----
    Run directly in VSCode or: python spacemouse_to_robot.py

Keys
----
    Esc / Ctrl-C     exit   (SpaceMouse buttons are *not* trapped)
"""

import sys, time, math, threading, queue, signal
from dataclasses import dataclass

try:
    import pyspacemouse           # ↳ https://pypi.org/project/pyspacemouse/
    import serial                 # ↳ https://pyserial.readthedocs.io
    import serial.tools.list_ports
except ImportError as e:
    sys.exit(f"Missing required package: {e}. Install with: pip install pyspacemouse pyserial")

# ------------------------------------------------ CONFIG -------------------------------------------------
SERIAL_PORT       = "/dev/ttyACM1"    # Teensy serial port
SERIAL_BAUD       = 115200            # Baud rate
DEG_MAX           = 45.0              # 1.0 pitch  ->  45°
PWM_MAX           = 255               # 1.0 yaw    ->  ±255
DEADBAND          = 0.05              # below this → 0  (noise filter)  [-]
PWM_MIN_THRESHOLD = 50                # stiction overcome (±counts)

SEND_HZ           = 50                # Teensy CLI tolerates ~>100 Hz fine
PITCH_RESOL_DEG   = 0.2               # only send if change ≥ 0.2°
PWM_RESOL_COUNT   = 4                 # … or change ≥ 4 counts
# ---------------------------------------------------------------------------------------------------------

# Graceful Ctrl-C / SIGINT handling -------------------------------------------------
done_evt = threading.Event()
signal.signal(signal.SIGINT, lambda *_: done_evt.set())

@dataclass
class RobotCmd:
    deg: float = 0.0     # 0 … 45
    pwm: int   = 0       # -255 … +255

def clamp(v, lo, hi): return max(lo, min(hi, v))

def list_serial_ports():
    """List all available serial ports"""
    ports = serial.tools.list_ports.comports()
    print("Available serial ports:")
    for port in ports:
        print(f"  {port.device} - {port.description}")
    return [port.device for port in ports]

def check_serial_port():
    """Check if the configured serial port is available"""
    available_ports = [port.device for port in serial.tools.list_ports.comports()]
    if SERIAL_PORT not in available_ports:
        print(f"[WARNING] Configured port {SERIAL_PORT} not found in available ports:")
        list_serial_ports()
        return False
    return True

# ---------------------------------------------------------------------------------------------------------
def main():
    print(f"[INFO] Ball Butler SpaceMouse Controller")
    print(f"[INFO] Configured for Teensy on {SERIAL_PORT} @ {SERIAL_BAUD} baud")
    
    # Check if serial port is available
    if not check_serial_port():
        response = input(f"Continue anyway? (y/n): ")
        if response.lower() != 'y':
            sys.exit("Aborted by user")

    print(f"[INFO] Opening serial {SERIAL_PORT} @ {SERIAL_BAUD} ...")
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0)
    except serial.SerialException as e:
        print(f"[ERROR] Serial error: {e}")
        print("[INFO] Available ports:")
        list_serial_ports()
        sys.exit(1)

    print("[INFO] Opening SpaceMouse …")
    try:
        pyspacemouse.open()
    except Exception as e:
        ser.close()
        sys.exit(f"Cannot open SpaceMouse: {e}")

    # ── producer thread : read spacemouse, put in queue ───────────────────────────────────────────
    q: queue.Queue[RobotCmd] = queue.Queue(maxsize=1)

    def reader():
        while not done_evt.is_set():
            st = pyspacemouse.read()

            # ------ PITCH -> angle ------------------------------------------------
            pitch_norm = st.pitch                      # -1 … +1
            if pitch_norm < 0: pitch_norm = 0.0       # ignore negative pitch
            pitch_norm = clamp(pitch_norm, 0.0, 1.0)

            deg = pitch_norm * DEG_MAX                # 0 … 45

            # ------ YAW -> pwm ----------------------------------------------------
            yaw_norm = st.yaw                         # -1 … +1
            if abs(yaw_norm) < DEADBAND:
                pwm = 0
            else:
                pwm_f = yaw_norm * PWM_MAX            # float
                # apply stiction minimum
                if 0 < abs(pwm_f) < PWM_MIN_THRESHOLD:
                    pwm_f = math.copysign(PWM_MIN_THRESHOLD, pwm_f)
                pwm = int(round(pwm_f))

            # drop the newest if queue full (keep latest command only)
            try:
                q.put_nowait(RobotCmd(deg, pwm))
            except queue.Full:
                try: q.get_nowait()
                except queue.Empty: pass
                q.put_nowait(RobotCmd(deg, pwm))

            time.sleep(1.0 / SEND_HZ)

    t = threading.Thread(target=reader, daemon=True)
    t.start()

    # ── consumer loop : send serial when change big enough ────────────────────────────────────────
    last_deg  = None
    last_pwm  = None

    print("[INFO] Streaming…  (Ctrl-C to quit)")
    try:
        while not done_evt.is_set():
            try:
                cmd = q.get(timeout=0.1)
            except queue.Empty:
                continue

            # send only if change significant (rate-limit Serial spam)
            if last_deg is None or abs(cmd.deg - last_deg) >= PITCH_RESOL_DEG:
                ser.write(f"odpos {cmd.deg:.2f}\n".encode())
                last_deg = cmd.deg

            if last_pwm is None or abs(cmd.pwm - last_pwm) >= PWM_RESOL_COUNT:
                ser.write(f"bmpwm {cmd.pwm}\n".encode())
                last_pwm = cmd.pwm

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")
    finally:
        print("\n[INFO] shutting down …")
        pyspacemouse.close()
        ser.close()
        done_evt.set()
        t.join(timeout=0.5)

# ---------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    # Handle command line arguments for port listing
    if len(sys.argv) == 2 and sys.argv[1] == "--list-ports":
        list_serial_ports()
        sys.exit(0)
    
    main()
