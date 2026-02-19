"""Tests for jugglebot.can.bus.CANBus — CAN bus lifecycle and reconnection."""

import errno
import struct
import time
from unittest.mock import MagicMock, patch, PropertyMock, call

import can
import pytest

from jugglebot.can.bus import CANBus


def _make_bus(mock_can_bus_cls=None):
    """Create a CANBus with a mocked logger and optionally a mocked can.Bus."""
    logger = MagicMock()
    bus = CANBus(logger=logger)
    if mock_can_bus_cls is not None:
        bus._bus = mock_can_bus_cls()
    return bus, logger


# ════════════════════════════════════════════════════════════════
# Lifecycle: setup / close / flush
# ════════════════════════════════════════════════════════════════


class TestSetup:
    @patch('jugglebot.can.bus.can.Bus')
    def test_creates_bus(self, mock_bus_cls):
        mock_bus_cls.return_value.recv.return_value = None  # flush() needs this
        logger = MagicMock()
        bus = CANBus(logger=logger, bus_name='can0', bitrate=1_000_000)
        bus.setup()
        mock_bus_cls.assert_called_once_with(
            channel='can0', interface='socketcan', bitrate=1_000_000,
        )

    @patch('jugglebot.can.bus.can.Bus')
    def test_is_open_after_setup(self, mock_bus_cls):
        mock_bus_cls.return_value.recv.return_value = None
        bus, _ = _make_bus()
        bus.setup()
        assert bus.is_open is True

    @patch('jugglebot.can.bus.can.Bus')
    def test_flushes_after_setup(self, mock_bus_cls):
        mock_instance = MagicMock()
        mock_instance.recv.return_value = None
        mock_bus_cls.return_value = mock_instance
        bus, _ = _make_bus()
        bus.setup()
        mock_instance.recv.assert_called()

    def test_is_open_false_initially(self):
        bus, _ = _make_bus()
        assert bus.is_open is False


class TestClose:
    def test_calls_shutdown(self):
        mock_inner = MagicMock()
        bus, _ = _make_bus()
        bus._bus = mock_inner
        bus.close()
        mock_inner.shutdown.assert_called_once()

    def test_close_when_none(self):
        bus, _ = _make_bus()
        bus._bus = None
        bus.close()  # Should not raise

    def test_close_handles_exception(self):
        mock_inner = MagicMock()
        mock_inner.shutdown.side_effect = Exception("shutdown fail")
        bus, logger = _make_bus()
        bus._bus = mock_inner
        bus.close()  # Should not raise
        logger.error.assert_called()


class TestFlush:
    def test_reads_until_none(self):
        mock_inner = MagicMock()
        msg1 = can.Message(arbitration_id=0x01, data=bytes(8))
        msg2 = can.Message(arbitration_id=0x02, data=bytes(8))
        mock_inner.recv.side_effect = [msg1, msg2, None]
        bus, _ = _make_bus()
        bus._bus = mock_inner
        bus.flush()
        assert mock_inner.recv.call_count == 3

    def test_flush_when_no_bus(self):
        bus, _ = _make_bus()
        bus._bus = None
        bus.flush()  # Should not raise


# ════════════════════════════════════════════════════════════════
# send()
# ════════════════════════════════════════════════════════════════


class TestSend:
    def test_calls_bus_send(self):
        mock_inner = MagicMock()
        bus, _ = _make_bus()
        bus._bus = mock_inner
        msg = can.Message(arbitration_id=0x01, data=bytes(8))
        bus.send(msg)
        mock_inner.send.assert_called_once_with(msg, timeout=None)

    def test_send_when_no_bus(self):
        bus, _ = _make_bus()
        bus._bus = None
        msg = can.Message(arbitration_id=0x01, data=bytes(8))
        bus.send(msg)  # Should not raise

    def test_enobufs_triggers_restore(self):
        mock_inner = MagicMock()
        os_error = OSError(errno.ENOBUFS, "No buffer space")
        can_error = can.CanError("send failed")
        can_error.__cause__ = os_error
        mock_inner.send.side_effect = can_error
        bus, _ = _make_bus()
        bus._bus = mock_inner
        with patch.object(bus, 'attempt_restore') as mock_restore:
            msg = can.Message(arbitration_id=0x01, data=bytes(8))
            bus.send(msg)
            mock_restore.assert_called_once()

    def test_other_can_error_no_restore(self):
        mock_inner = MagicMock()
        can_error = can.CanError("generic error")
        mock_inner.send.side_effect = can_error
        bus, _ = _make_bus()
        bus._bus = mock_inner
        with patch.object(bus, 'attempt_restore') as mock_restore:
            msg = can.Message(arbitration_id=0x01, data=bytes(8))
            bus.send(msg)
            mock_restore.assert_not_called()


# ════════════════════════════════════════════════════════════════
# fetch_all()
# ════════════════════════════════════════════════════════════════


class TestFetchAll:
    def test_dispatches_all_messages(self):
        mock_inner = MagicMock()
        msg1 = can.Message(arbitration_id=0x01, data=bytes(8))
        msg2 = can.Message(arbitration_id=0x02, data=bytes(8))
        mock_inner.recv.side_effect = [msg1, msg2, None]
        bus, _ = _make_bus()
        bus._bus = mock_inner
        handler = MagicMock()
        bus.fetch_all(handler)
        assert handler.call_count == 2
        handler.assert_any_call(msg1)
        handler.assert_any_call(msg2)

    def test_empty_bus(self):
        mock_inner = MagicMock()
        mock_inner.recv.return_value = None
        bus, _ = _make_bus()
        bus._bus = mock_inner
        handler = MagicMock()
        bus.fetch_all(handler)
        handler.assert_not_called()

    def test_exception_logged(self):
        mock_inner = MagicMock()
        mock_inner.recv.side_effect = Exception("read error")
        bus, logger = _make_bus()
        bus._bus = mock_inner
        handler = MagicMock()
        bus.fetch_all(handler)
        logger.error.assert_called()


# ════════════════════════════════════════════════════════════════
# broadcast_time()
# ════════════════════════════════════════════════════════════════


class TestBroadcastTime:
    def test_sends_correct_format(self):
        mock_inner = MagicMock()
        bus, _ = _make_bus()
        bus._bus = mock_inner
        bus.broadcast_time()
        mock_inner.send.assert_called_once()
        sent_msg = mock_inner.send.call_args[0][0]
        assert len(sent_msg.data) == 8
        sec, usec = struct.unpack('<II', bytes(sent_msg.data))
        assert sec > 0  # Should be a reasonable epoch timestamp
        assert 0 <= usec < 1_000_000

    def test_send_failure_logged(self):
        mock_inner = MagicMock()
        mock_inner.send.side_effect = can.CanError("tx fail")
        bus, logger = _make_bus()
        bus._bus = mock_inner
        bus.broadcast_time()
        logger.warning.assert_called()


# ════════════════════════════════════════════════════════════════
# attempt_restore()
# ════════════════════════════════════════════════════════════════


class TestAttemptRestore:
    @patch('jugglebot.can.bus.can.Bus')
    def test_success_on_first_attempt(self, mock_bus_cls):
        mock_instance = MagicMock()
        mock_instance.recv.return_value = None  # flush
        mock_bus_cls.return_value = mock_instance
        bus, logger = _make_bus()
        result = bus.attempt_restore(max_retries=1, retry_delay=0.01,
                                     reconnect_timeout=0.01)
        assert result is True

    @patch('jugglebot.can.bus.can.Bus')
    def test_failure_after_max_retries(self, mock_bus_cls):
        mock_bus_cls.side_effect = Exception("cannot open bus")
        bus, logger = _make_bus()
        result = bus.attempt_restore(max_retries=2, retry_delay=0.01)
        assert result is False

    @patch('jugglebot.can.bus.can.Bus')
    def test_poll_callback_invoked(self, mock_bus_cls):
        mock_instance = MagicMock()
        mock_instance.recv.return_value = None
        mock_bus_cls.return_value = mock_instance
        callback = MagicMock(return_value=True)  # Immediate success
        bus, _ = _make_bus()
        result = bus.attempt_restore(
            poll_callback=callback, max_retries=1,
            retry_delay=0.01, reconnect_timeout=0.5,
        )
        assert result is True
        callback.assert_called()

    @patch('jugglebot.can.bus.can.Bus')
    def test_without_poll_callback(self, mock_bus_cls):
        mock_instance = MagicMock()
        mock_instance.recv.return_value = None
        mock_bus_cls.return_value = mock_instance
        bus, _ = _make_bus()
        result = bus.attempt_restore(max_retries=1, retry_delay=0.01)
        assert result is True
