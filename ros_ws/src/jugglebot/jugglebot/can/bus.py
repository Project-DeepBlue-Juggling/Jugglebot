"""CAN bus lifecycle: setup, send/recv, flush, time-sync, and reconnection.

Provides a thin, thread-safe wrapper around python-can's Bus object.
"""

import struct
import threading
import time
from typing import Callable, Optional

import can
import jugglebot.protocol_config as proto


class CANBus:
    """Thread-safe CAN bus wrapper with reconnection support."""

    def __init__(self, logger, bus_name: str = 'can0',
                 bitrate: int = proto.CAN_BAUD_RATE,
                 interface: str = 'socketcan'):
        self._logger = logger
        self._bus_name = bus_name
        self._bitrate = bitrate
        self._interface = interface
        self._bus: Optional[can.Bus] = None
        self._lock = threading.RLock()

        # Time-sync CAN ID
        self._time_sync_id = proto.CAN_ID_SHARED_TIME_SYNC

    # ── Lifecycle ──────────────────────────────────────────────

    def setup(self):
        """Initialize (or re-initialize) the CAN bus connection."""
        self._bus = can.Bus(channel=self._bus_name, interface=self._interface,
                            bitrate=self._bitrate)
        self.flush()
        self._logger.info(f"CAN bus '{self._bus_name}' initialized")

    def close(self):
        """Shut down the CAN bus connection."""
        if self._bus is not None:
            try:
                self._bus.shutdown()
                self._logger.info('CAN bus closed')
            except Exception as e:
                self._logger.error(f'Error closing CAN bus: {e}')

    def flush(self):
        """Discard all pending messages on the bus."""
        if self._bus:
            while self._bus.recv(timeout=0):
                pass

    @property
    def is_open(self) -> bool:
        return self._bus is not None

    # ── Send / Receive ─────────────────────────────────────────

    def send(self, msg: can.Message, timeout: Optional[float] = None):
        """Send a CAN message (thread-safe).

        On buffer-full errors (errno 105), triggers reconnection.
        """
        if not self._bus:
            return
        try:
            with self._lock:
                self._bus.send(msg, timeout=timeout)
        except can.CanError as e:
            self._logger.warning(f'CAN send failed: {e}')
            if '105' in str(e):
                self.attempt_restore()

    def fetch_all(self, handler: Callable[[can.Message], None]):
        """Read all pending messages and dispatch each to handler(msg).

        Called from the node's CAN poll timer at ~1 kHz.
        """
        with self._lock:
            try:
                while True:
                    msg = self._bus.recv(timeout=0)
                    if msg is None:
                        break
                    handler(msg)
            except Exception as e:
                self._logger.error(f'Error fetching messages: {e}')

    # ── Time synchronization ───────────────────────────────────

    def broadcast_time(self):
        """Send current CLOCK_REALTIME (sec + µsec) on the time-sync CAN ID.

        Both Teensys (platform and Ball Butler) use this for clock alignment.
        """
        t = time.time()
        sec = int(t)
        usec = int((t - sec) * 1_000_000)
        payload = struct.pack('<II', sec, usec)
        msg = can.Message(arbitration_id=self._time_sync_id,
                          data=payload, is_extended_id=False)
        try:
            with self._lock:
                self._bus.send(msg, timeout=0.002)
        except can.CanError as e:
            self._logger.warning(f'Time-sync Tx failed: {e}')

    # ── Reconnection ───────────────────────────────────────────

    def attempt_restore(self, poll_callback: Optional[Callable[[], bool]] = None,
                        max_retries: int = 3, retry_delay: float = 5.0,
                        reconnect_timeout: float = 1.0) -> bool:
        """Try to re-establish the CAN connection.

        Args:
            poll_callback: Called repeatedly during the reconnect wait.
                           Should fetch/process CAN messages and return True
                           when all expected heartbeats have been received.
            max_retries: Number of reconnection attempts.
            retry_delay: Seconds to wait between retries.
            reconnect_timeout: Seconds to wait for heartbeats per attempt.

        Returns True if connection was restored, False otherwise.
        """
        self._logger.info('Attempting to restore CAN connection...')

        for attempt in range(1, max_retries + 1):
            rx_ok = False
            tx_ok = False
            try:
                if self._bus is not None:
                    self.close()
                self.setup()

                # Wait for heartbeats (rx check)
                if poll_callback:
                    time.sleep(0.1)
                    start = time.time()
                    while time.time() - start < reconnect_timeout:
                        if poll_callback():
                            self._logger.info(f'CAN bus read OK on attempt {attempt}')
                            rx_ok = True
                            break
                        time.sleep(0.1)
                else:
                    rx_ok = True  # No check requested

                # Test write
                test_msg = can.Message(arbitration_id=0x999, dlc=8,
                                       is_extended_id=False, data=bytes(8))
                try:
                    with self._lock:
                        self._bus.send(test_msg)
                    self._logger.info(f'CAN bus write OK on attempt {attempt}')
                    tx_ok = True
                except Exception as e:
                    self._logger.warning(f'Write test failed on attempt {attempt}: {e}')

                if rx_ok and tx_ok:
                    self._logger.info('CAN bus connection restored!')
                    return True

            except Exception as e:
                self._logger.warning(f'Reconnect attempt {attempt} failed: {e}')

            time.sleep(retry_delay)

        self._logger.error(f'Failed to restore CAN after {max_retries} attempts')
        self.close()
        return False
