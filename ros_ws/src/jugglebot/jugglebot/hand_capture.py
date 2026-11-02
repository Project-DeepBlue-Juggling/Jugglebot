"""TEMPORARY (JB-147 diagnostic capture): full-rate Hand signal recorder.

Pure Python (no ROS2).  One row per RECEIVED Hand sample of each signal, at the
native rate the can-bridge sends it, to a CSV -- the 100 Hz Jetson-stamped
``/hand_telemetry`` topic is too coarse to show why the Hand overshot at Apex 1.35.

Determinism contract (repo rule: no blocking I/O in control loops): ``offer`` is
called from the bridge's UDP RX thread and ONLY does ``queue.put_nowait`` on a
bounded queue; a dedicated daemon writer thread drains it and does all file I/O.
A full queue drops the row and counts it -- it never blocks, never raises.

Remove when JB-147 is diagnosed: this file, its node hooks (grep
``TEMPORARY (JB-147``), the launch arg, tools/probes/hand_capture_view.py and
tests/ros/test_hand_capture.py.
"""

from __future__ import annotations

import csv
import os
import queue
import threading
import time

COLUMNS = (
    't_jetson_mono_s',   # time.monotonic() at RX on the Jetson (always present)
    't_jetson_wall_s',   # time.time() at RX
    'kind',              # telem | diag | echo
    't_src_us',          # Teensy stamp: telem = t_teensy_us (Teensy clock);
                         #   echo = t_bridge_us (bridge CAN3-RX stamp); diag = '' (frame carries none)
    'pos_rev', 'vel_rps',                      # telem
    'iq_setpoint', 'iq_measured',              # diag
    'axis_state', 'active_errors', 'diag_flags',  # diag
    'cmd_pos_rev', 'cmd_vel_ff_rps', 'cmd_tor_ff',  # echo (decoded)
    'dropped_total',     # rows dropped by this writer so far (overflow)
)
_NCOL = len(COLUMNS)


class HandCaptureWriter:
    def __init__(self, directory: str, maxsize: int = 20000, name: str = 'hand_capture'):
        self.directory = os.path.expanduser(directory)
        os.makedirs(self.directory, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(self.directory, f'{name}_{stamp}.csv')
        n = 1
        while os.path.exists(self.path):   # two launches inside one second
            self.path = os.path.join(self.directory, f'{name}_{stamp}_{n}.csv')
            n += 1
        self._q: queue.Queue = queue.Queue(maxsize=maxsize)
        self.dropped = 0        # written only by offer() callers (the RX thread)
        self.written = 0        # written only by the writer thread
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='hand_capture_writer',
                                        daemon=True)
        self._started = False

    def start(self):
        if not self._started:
            self._started = True
            self._thread.start()
        return self

    # ── producer side (RX thread): never blocks, never raises ──
    def offer(self, kind, t_src_us='', pos='', vel='', iq_sp='', iq_meas='',
              axis_state='', active_errors='', diag_flags='',
              cmd_pos='', cmd_vel='', cmd_tor=''):
        row = (time.monotonic(), time.time(), kind, t_src_us, pos, vel, iq_sp,
               iq_meas, axis_state, active_errors, diag_flags, cmd_pos, cmd_vel,
               cmd_tor, self.dropped)
        try:
            self._q.put_nowait(row)
        except queue.Full:
            self.dropped += 1
        except Exception:  # noqa: BLE001 -- a diagnostic must never break the RX path
            self.dropped += 1

    # ── consumer side ──
    def _run(self):
        with open(self.path, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(COLUMNS)
            last_flush = time.monotonic()
            while True:
                try:
                    row = self._q.get(timeout=0.2)
                except queue.Empty:
                    row = None
                if row is not None:
                    w.writerow(row)
                    self.written += 1
                elif self._stop.is_set():
                    break
                now = time.monotonic()
                if now - last_flush > 0.5:
                    fh.flush()
                    last_flush = now
            fh.flush()

    def close(self, timeout: float = 5.0):
        """Drain what is queued, flush, and stop the writer (idempotent)."""
        self._stop.set()
        if self._started:
            self._thread.join(timeout)
