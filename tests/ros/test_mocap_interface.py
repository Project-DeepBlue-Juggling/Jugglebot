"""Outage-logging tests for MocapInterface (the QTM connection handler).

Covers the state-transition logging contract for task (c): the "QTM
unavailable" WARNING logs the FIRST failure of an outage and then stays silent
until QTM returns (no throttled repeat), and the state TRANSITIONS
(available→unavailable, unavailable→available) are the loud events — matching
the connection-handling pattern used by the SpaceMouse handler and mocap's own
_on_qtm_disconnect.

ROS 2 is mocked by tests/ros/conftest.py (geometry_msgs in particular). qtm_rt
is the real library (installed on the Jetson). MocapInterface.start() spins an
asyncio thread that would attempt real network connects, so we patch it to a
no-op and drive the logging methods directly.
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch

from jugglebot.mocap_interface import MocapInterface


def _make_iface():
    """Construct a MocapInterface without starting its asyncio thread."""
    with patch.object(MocapInterface, 'start', lambda self: None):
        return MocapInterface(host='127.0.0.1', port=22223,
                              logger=MagicMock(), node=MagicMock())


def test_qtm_outage_logs_first_then_silent():
    """Repeated connect failures within one outage log exactly one WARNING."""
    iface = _make_iface()
    iface._log_qtm_outage('connection refused')
    iface._log_qtm_outage('connection refused')
    iface._log_qtm_outage('connection refused')
    assert iface.logger.warning.call_count == 1
    assert iface._qtm_outage_active is True
    assert 'QTM unavailable' in iface.logger.warning.call_args[0][0]


def test_qtm_reconnect_clears_outage_and_relogs_next_outage():
    """The unavailable→available edge clears the latch so a NEW outage logs its
    first failure again (state-transition logging, not a one-shot mute)."""
    iface = _make_iface()
    iface._log_qtm_outage('down')
    assert iface.logger.warning.call_count == 1
    # connect() clears this on the "Connected to QTM." edge.
    iface._qtm_outage_active = False
    iface._log_qtm_outage('down again')
    assert iface.logger.warning.call_count == 2


def test_qtm_disconnect_marks_outage_and_silences_reconnect():
    """A mid-session drop is the available→unavailable transition: it logs one
    'QTM disconnected' WARNING and marks the outage active so the reconnect
    loop's connect() failures stay silent (no duplicate 'QTM unavailable')."""
    iface = _make_iface()
    iface.loop = None  # no running loop → _on_qtm_disconnect schedules no task
    iface._on_qtm_disconnect(RuntimeError('cable pulled'))
    assert iface.logger.warning.call_count == 1
    assert 'QTM disconnected' in iface.logger.warning.call_args[0][0]
    assert iface._qtm_outage_active is True
    # The reconnect loop now retries silently.
    iface._log_qtm_outage('still down')
    assert iface.logger.warning.call_count == 1
